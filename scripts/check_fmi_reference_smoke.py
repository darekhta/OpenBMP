#!/usr/bin/env python3
"""Run pinned FMI Reference-FMUs through OpenBMP and FMPy."""

from __future__ import annotations

import hashlib
import os
import subprocess
import sys
import tempfile
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path


REFERENCE_RELEASE_VERSION = "0.0.39"
REFERENCE_RELEASE_URL = (
    "https://github.com/modelica/Reference-FMUs/releases/download/"
    f"v{REFERENCE_RELEASE_VERSION}/Reference-FMUs-{REFERENCE_RELEASE_VERSION}.zip"
)
REFERENCE_RELEASE_SHA256 = "6863d55e5818e1ca4e4614c4d4ba4047a921b4495f6336e7002874ed791f6c2a"
REFERENCE_LICENSE_ENTRY = "LICENSE.txt"
ABS_TOLERANCE = 1.0e-12
REL_TOLERANCE = 1.0e-6
SUCCESS_MESSAGE = "fmi reference smoke: 5 Reference-FMUs match FMPy"


@dataclass(frozen=True)
class ReferenceOutput:
    """One typed output expected from a supported Reference-FMU."""

    name: str
    kind: str


@dataclass(frozen=True)
class ReferenceCase:
    """One supported Reference-FMU importer smoke case."""

    model: str
    step_s: float
    steps: int
    outputs: tuple[ReferenceOutput, ...]

    @property
    def fmu_entry(self) -> str:
        """FMU path inside the Reference-FMUs release archive."""
        return f"3.0/{self.model}.fmu"

    @property
    def output_names(self) -> tuple[str, ...]:
        """Output names in CSV/FMPy order."""
        return tuple(output.name for output in self.outputs)


CASES = (
    ReferenceCase("Dahlquist", 0.1, 10, (ReferenceOutput("x", "float64"),)),
    ReferenceCase(
        "VanDerPol",
        0.01,
        10,
        (ReferenceOutput("x0", "float64"), ReferenceOutput("x1", "float64")),
    ),
    # Stop before the first bounce. Full event-mode handling remains a separate gate.
    ReferenceCase(
        "BouncingBall",
        0.01,
        40,
        (ReferenceOutput("h", "float64"), ReferenceOutput("v", "float64")),
    ),
    ReferenceCase("Stair", 0.2, 5, (ReferenceOutput("counter", "int32"),)),
    ReferenceCase("Resource", 1.0, 2, (ReferenceOutput("y", "int32"),)),
)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    try:
        from fmpy import read_model_description
    except ImportError as exc:
        print(
            "fmi reference smoke: FMPy==0.3.26 is required; install it with "
            "python3 -m pip install FMPy==0.3.26",
            file=sys.stderr,
        )
        print(f"fmi reference smoke: import failed: {exc}", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory(prefix="openbmp-reference-fmu-") as tmp:
        tmp_path = Path(tmp)
        release_zip = load_reference_release(tmp_path)

        for case in CASES:
            fmu_path = extract_reference_fmu(release_zip, tmp_path, case)
            description = read_model_description(fmu_path, validate=True)
            if description.fmiVersion != "3.0":
                raise CheckError(
                    f"{case.model}: expected FMI 3.0 reference FMU, got {description.fmiVersion}"
                )
            if description.coSimulation is None:
                raise CheckError(f"{case.model}: reference FMU does not declare CoSimulation")

            first = run_openbmp_import(root, tmp_path, fmu_path, case, "materialized-a")
            second = run_openbmp_import(root, tmp_path, fmu_path, case, "materialized-b")
            if first.stdout != second.stdout:
                raise CheckError(
                    f"{case.model}: Reference-FMU importer output is not byte-stable across repeated runs\n"
                    f"first:\n{first.stdout}\nsecond:\n{second.stdout}"
                )

            openbmp = parse_openbmp_csv(case, first.stdout)
            fmpy = run_fmpy(fmu_path, case)
            assert_close(case.model, "time", openbmp["time"], fmpy["time"])
            for output in case.outputs:
                assert_output(case.model, output, openbmp[output.name], fmpy[output.name])

    print(SUCCESS_MESSAGE)
    return 0


class CheckError(RuntimeError):
    """Reference-FMU importer smoke validation failed."""


def load_reference_release(tmp_path: Path) -> Path:
    configured = os.environ.get("OPENBMP_REFERENCE_FMUS_ZIP")
    if configured:
        release_zip = Path(configured)
        if not release_zip.is_file():
            raise CheckError(f"OPENBMP_REFERENCE_FMUS_ZIP does not exist: {release_zip}")
    else:
        release_zip = tmp_path / f"Reference-FMUs-{REFERENCE_RELEASE_VERSION}.zip"
        urllib.request.urlretrieve(REFERENCE_RELEASE_URL, release_zip)

    digest = hashlib.sha256(release_zip.read_bytes()).hexdigest()
    if digest != REFERENCE_RELEASE_SHA256:
        raise CheckError(
            f"Reference-FMUs release SHA-256 mismatch: actual {digest}, "
            f"expected {REFERENCE_RELEASE_SHA256}"
        )
    with zipfile.ZipFile(release_zip) as archive:
        license_text = archive.read(REFERENCE_LICENSE_ENTRY).decode("utf-8", errors="replace")
        if "Redistribution and use in source and binary forms" not in license_text:
            raise CheckError("Reference-FMUs release license text did not match BSD-style grant")
    return release_zip


def extract_reference_fmu(release_zip: Path, tmp_path: Path, case: ReferenceCase) -> Path:
    with zipfile.ZipFile(release_zip) as archive:
        fmu_bytes = archive.read(case.fmu_entry)
    fmu_path = tmp_path / f"{case.model}.fmu"
    fmu_path.write_bytes(fmu_bytes)
    return fmu_path


def run_openbmp_import(
    root: Path,
    tmp_path: Path,
    fmu_path: Path,
    case: ReferenceCase,
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
            "import_fmu_final_sample",
            "--",
            str(fmu_path),
            str(tmp_path / f"{case.model}-{materialized_name}"),
            str(case.step_s),
            str(case.steps),
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


def parse_openbmp_csv(case: ReferenceCase, stdout: str) -> dict[str, float | int]:
    lines = [line.strip() for line in stdout.splitlines() if line.strip()]
    if len(lines) != 2:
        raise CheckError(
            f"{case.model}: expected two CSV lines from generic importer, got {len(lines)}"
        )
    header = lines[0].split(",")
    expected_header = ["time_s", *case.output_names]
    if header != expected_header:
        raise CheckError(f"{case.model}: unexpected CSV header {header!r}")
    fields = lines[1].split(",")
    if len(fields) != len(expected_header):
        raise CheckError(
            f"{case.model}: expected {len(expected_header)} CSV fields, got {len(fields)}"
        )
    values = {"time": float(fields[0])}
    for output, raw in zip(case.outputs, fields[1:]):
        if output.kind == "float64":
            values[output.name] = float(raw)
        elif output.kind == "int32":
            values[output.name] = int(raw)
        else:
            raise CheckError(f"{case.model}: unsupported output kind {output.kind}")
    return values


def run_fmpy(fmu_path: Path, case: ReferenceCase) -> dict[str, float | int]:
    from fmpy import simulate_fmu

    result = simulate_fmu(
        str(fmu_path),
        start_time=0.0,
        stop_time=case.step_s * case.steps,
        output_interval=case.step_s,
        output=list(case.output_names),
        validate=True,
    )
    final = result[-1]
    values = {"time": float(final["time"])}
    for output in case.outputs:
        if output.kind == "float64":
            values[output.name] = float(final[output.name])
        elif output.kind == "int32":
            values[output.name] = int(final[output.name])
        else:
            raise CheckError(f"{case.model}: unsupported output kind {output.kind}")
    return values


def assert_output(
    model: str,
    output: ReferenceOutput,
    actual: float | int,
    expected: float | int,
) -> None:
    if output.kind == "float64":
        assert_close(model, output.name, float(actual), float(expected))
    elif output.kind == "int32":
        if int(actual) != int(expected):
            raise CheckError(f"{model} {output.name}: OpenBMP {actual}, FMPy {expected}")
    else:
        raise CheckError(f"{model}: unsupported output kind {output.kind}")


def assert_close(model: str, metric: str, actual: float, expected: float) -> None:
    diff = abs(actual - expected)
    bound = max(ABS_TOLERANCE, REL_TOLERANCE * max(abs(expected), 1.0))
    if diff > bound:
        raise CheckError(
            f"{model} {metric}: OpenBMP {actual:e}, FMPy {expected:e}, "
            f"diff {diff:e}, bound {bound:e}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
