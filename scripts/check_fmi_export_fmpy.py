#!/usr/bin/env python3
"""Run the generated OpenBMP point-mass FMU inside FMPy and check its trace."""

from __future__ import annotations

import platform
import subprocess
import sys
import tempfile
import tomllib
from pathlib import Path


TRACE_CASE = "exported-fmi-point-mass-trace"


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    try:
        from fmpy import read_model_description, simulate_fmu
    except ImportError as exc:
        print(
            "fmi export fmpy: FMPy==0.3.26 is required; install it with "
            "python3 -m pip install FMPy==0.3.26",
            file=sys.stderr,
        )
        print(f"fmi export fmpy: import failed: {exc}", file=sys.stderr)
        return 1

    tolerance = load_trace_tolerance(root)
    run(root, ["cargo", "build", "-p", "openbmp-fmi", "--lib", "--locked"])
    library_path = current_debug_library(root)

    with tempfile.TemporaryDirectory(prefix="openbmp-fmpy-") as tmp:
        fmu_path = Path(tmp) / "openbmp_point_mass.fmu"
        run(
            root,
            [
                "cargo",
                "run",
                "--quiet",
                "--locked",
                "-p",
                "openbmp-fmi",
                "--example",
                "write_point_mass_fmu",
                "--",
                str(fmu_path),
                str(library_path),
            ],
        )

        description = read_model_description(fmu_path, validate=True)
        if description.fmiVersion != "3.0":
            raise CheckError(f"expected FMI 3.0 model, got {description.fmiVersion}")
        if description.coSimulation is None:
            raise CheckError("generated FMU does not declare CoSimulation")

        stop_time = tolerance["step_s"] * tolerance["steps"]
        result = simulate_fmu(
            str(fmu_path),
            start_time=0.0,
            stop_time=stop_time,
            output_interval=tolerance["step_s"],
            output=["altitude", "velocity", "step"],
            start_values={"throttle": tolerance["throttle"]},
            validate=True,
        )
        check_final_sample(result[-1], tolerance)

    print("fmi export fmpy: point-mass trace validated")
    return 0


class CheckError(RuntimeError):
    """FMPy export smoke validation failed."""


def run(root: Path, command: list[str]) -> None:
    result = subprocess.run(
        command,
        cwd=root,
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        raise SystemExit(result.returncode)


def current_debug_library(root: Path) -> Path:
    system = platform.system()
    if system == "Darwin":
        name = "libopenbmp_fmi.dylib"
    elif system == "Linux":
        name = "libopenbmp_fmi.so"
    elif system == "Windows":
        name = "openbmp_fmi.dll"
    else:
        raise CheckError(f"unsupported FMPy smoke platform {system}")

    path = root / "target" / "debug" / name
    if not path.is_file():
        raise CheckError(f"could not find built openbmp-fmi cdylib at {path}")
    return path


def load_trace_tolerance(root: Path) -> dict[str, float | int | str]:
    path = root / "crates" / "openbmp-fmi" / "tests" / "expected" / "export-point-mass-trace.toml"
    with path.open("rb") as fh:
        tolerance = tomllib.load(fh)
    if tolerance.get("case") != TRACE_CASE:
        raise CheckError(f"unexpected trace tolerance case {tolerance.get('case')!r}")
    return tolerance


def check_final_sample(sample, tolerance: dict[str, float | int | str]) -> None:
    throttle = float(tolerance["throttle"])
    stop_time = float(tolerance["step_s"]) * int(tolerance["steps"])
    acceleration = throttle * 20.0 - 9.80665
    expected_altitude = 0.5 * acceleration * stop_time * stop_time
    expected_velocity = acceleration * stop_time

    assert_close("time", float(sample["time"]), stop_time, tolerance)
    assert_close("altitude", float(sample["altitude"]), expected_altitude, tolerance)
    assert_close("velocity", float(sample["velocity"]), expected_velocity, tolerance)
    step = int(sample["step"])
    if step != int(tolerance["steps"]):
        raise CheckError(f"step: actual {step}, expected {tolerance['steps']}")


def assert_close(
    metric: str,
    actual: float,
    expected: float,
    tolerance: dict[str, float | int | str],
) -> None:
    diff = abs(actual - expected)
    bound = max(
        float(tolerance["absolute_tolerance"]),
        float(tolerance["relative_tolerance"]) * max(abs(expected), 1.0),
    )
    if diff > bound:
        raise CheckError(
            f"{metric}: actual {actual:e}, expected {expected:e}, diff {diff:e}, bound {bound:e}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
