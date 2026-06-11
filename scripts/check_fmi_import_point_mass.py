#!/usr/bin/env python3
"""Run the packaged OpenBMP point-mass FMU through OpenBMP's FMI importer."""

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
    tolerance = load_trace_tolerance(root)
    run(root, ["cargo", "build", "-p", "openbmp-fmi", "--lib", "--locked"])
    library_path = current_debug_library(root)

    with tempfile.TemporaryDirectory(prefix="openbmp-fmi-import-") as tmp:
        tmp_path = Path(tmp)
        fmu_path = tmp_path / "openbmp_point_mass.fmu"
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
        first = run_importer(root, tmp_path, fmu_path, tolerance, "materialized-a")
        second = run_importer(root, tmp_path, fmu_path, tolerance, "materialized-b")
        if first.stdout != second.stdout:
            raise CheckError(
                "importer output is not byte-stable across repeated fixed-step runs\n"
                f"first:\n{first.stdout}\nsecond:\n{second.stdout}"
            )
        check_final_sample(parse_csv_sample(first.stdout), tolerance)

    print("fmi import point-mass: trace validated and byte-stable")
    return 0


class CheckError(RuntimeError):
    """OpenBMP FMI import smoke validation failed."""


def run_importer(
    root: Path,
    tmp_path: Path,
    fmu_path: Path,
    tolerance: dict[str, float | int | str],
    materialized_name: str,
) -> subprocess.CompletedProcess[str]:
    return run(
        root,
        [
            "cargo",
            "run",
            "--quiet",
            "--locked",
            "-p",
            "openbmp-fmi",
            "--example",
            "import_point_mass_fmu",
            "--",
            str(fmu_path),
            str(tmp_path / materialized_name),
            str(tolerance["throttle"]),
            str(tolerance["step_s"]),
            str(tolerance["steps"]),
        ],
    )


def run(root: Path, command: list[str]) -> subprocess.CompletedProcess[str]:
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
    return result


def current_debug_library(root: Path) -> Path:
    system = platform.system()
    if system == "Darwin":
        name = "libopenbmp_fmi.dylib"
    elif system == "Linux":
        name = "libopenbmp_fmi.so"
    elif system == "Windows":
        name = "openbmp_fmi.dll"
    else:
        raise CheckError(f"unsupported FMI import smoke platform {system}")

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


def parse_csv_sample(stdout: str) -> dict[str, float | int]:
    lines = [line.strip() for line in stdout.splitlines() if line.strip()]
    if len(lines) != 2:
        raise CheckError(f"expected two CSV lines from importer example, got {len(lines)}")
    header = lines[0].split(",")
    if header != ["time_s", "altitude_m", "velocity_m_s", "step"]:
        raise CheckError(f"unexpected CSV header {header!r}")
    fields = lines[1].split(",")
    if len(fields) != 4:
        raise CheckError(f"expected four CSV fields, got {len(fields)}")
    return {
        "time": float(fields[0]),
        "altitude": float(fields[1]),
        "velocity": float(fields[2]),
        "step": int(fields[3]),
    }


def check_final_sample(
    sample: dict[str, float | int],
    tolerance: dict[str, float | int | str],
) -> None:
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
