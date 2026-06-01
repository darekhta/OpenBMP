#!/usr/bin/env python3
"""Validate the machine-readable requirements traceability register."""

from __future__ import annotations

import re
import sys
import tomllib
from pathlib import Path
from typing import Any


REQ_ID = re.compile(r"^REQ-[A-Z0-9]+-\d{3}$")
VERIFICATION_ID = re.compile(r"^V-[A-Z0-9]+-\d{3}$")
ALLOWED_KINDS = {"ci", "test", "analysis"}


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    register = root / "requirements.toml"
    errors: list[str] = []

    try:
        with register.open("rb") as fh:
            data = tomllib.load(fh)
    except tomllib.TOMLDecodeError as exc:
        print(f"{register}: invalid TOML: {exc}", file=sys.stderr)
        return 1

    if data.get("schema_version") != 1:
        errors.append("schema_version must be 1")

    requirements = data.get("requirements")
    verifications = data.get("verification")
    if not isinstance(requirements, list) or not requirements:
        errors.append("requirements.toml must contain at least one [[requirements]] entry")
        requirements = []
    if not isinstance(verifications, list) or not verifications:
        errors.append("requirements.toml must contain at least one [[verification]] entry")
        verifications = []

    reqs_by_id = index_by_id(requirements, "requirement", REQ_ID, errors)
    verifications_by_id = index_by_id(verifications, "verification", VERIFICATION_ID, errors)

    for req in requirements:
        req_id = str(req.get("id", "<missing>"))
        require_non_empty_string(req, "title", f"requirement {req_id}", errors)
        require_non_empty_string(req, "statement", f"requirement {req_id}", errors)
        linked = require_string_list(req, "verification", f"requirement {req_id}", errors)
        if not linked:
            errors.append(f"requirement {req_id} has no verification evidence")
        for verification_id in linked:
            verification = verifications_by_id.get(verification_id)
            if verification is None:
                errors.append(f"requirement {req_id} references unknown verification {verification_id}")
                continue
            if req_id not in verification.get("requirements", []):
                errors.append(
                    f"requirement {req_id} references {verification_id}, but "
                    f"{verification_id} does not link back"
                )

    for verification in verifications:
        verification_id = str(verification.get("id", "<missing>"))
        kind = verification.get("kind")
        if kind not in ALLOWED_KINDS:
            errors.append(
                f"verification {verification_id} kind must be one of {sorted(ALLOWED_KINDS)}"
            )
        require_non_empty_string(verification, "command", f"verification {verification_id}", errors)
        linked_reqs = require_string_list(
            verification, "requirements", f"verification {verification_id}", errors
        )
        if not linked_reqs:
            errors.append(f"verification {verification_id} is orphaned: no requirements listed")
        for req_id in linked_reqs:
            req = reqs_by_id.get(req_id)
            if req is None:
                errors.append(f"verification {verification_id} references unknown requirement {req_id}")
                continue
            if verification_id not in req.get("verification", []):
                errors.append(
                    f"verification {verification_id} references {req_id}, but "
                    f"{req_id} does not link back"
                )
        validate_evidence_files(root, verification, verification_id, errors)

    if errors:
        for error in errors:
            print(f"requirements traceability: {error}", file=sys.stderr)
        return 1

    print(
        f"requirements traceability: {len(reqs_by_id)} requirements, "
        f"{len(verifications_by_id)} verification items"
    )
    return 0


def index_by_id(
    entries: list[Any],
    label: str,
    pattern: re.Pattern[str],
    errors: list[str],
) -> dict[str, dict[str, Any]]:
    indexed: dict[str, dict[str, Any]] = {}
    for i, entry in enumerate(entries):
        if not isinstance(entry, dict):
            errors.append(f"{label} #{i} must be a TOML table")
            continue
        entry_id = entry.get("id")
        if not isinstance(entry_id, str) or not entry_id:
            errors.append(f"{label} #{i} is missing a non-empty id")
            continue
        if pattern.fullmatch(entry_id) is None:
            errors.append(f"{label} {entry_id} has invalid id format")
        if entry_id in indexed:
            errors.append(f"duplicate {label} id {entry_id}")
        indexed[entry_id] = entry
    return indexed


def require_non_empty_string(
    table: dict[str, Any],
    field: str,
    owner: str,
    errors: list[str],
) -> str:
    value = table.get(field)
    if not isinstance(value, str) or not value.strip():
        errors.append(f"{owner} field {field} must be a non-empty string")
        return ""
    return value


def require_string_list(
    table: dict[str, Any],
    field: str,
    owner: str,
    errors: list[str],
) -> list[str]:
    value = table.get(field)
    if not isinstance(value, list):
        errors.append(f"{owner} field {field} must be a list of strings")
        return []
    strings: list[str] = []
    for i, item in enumerate(value):
        if not isinstance(item, str) or not item.strip():
            errors.append(f"{owner} field {field}[{i}] must be a non-empty string")
            continue
        strings.append(item)
    return strings


def validate_evidence_files(
    root: Path,
    verification: dict[str, Any],
    verification_id: str,
    errors: list[str],
) -> None:
    paths = require_string_list(verification, "paths", f"verification {verification_id}", errors)
    if not paths:
        errors.append(f"verification {verification_id} must list at least one evidence path")
        return

    contents: list[str] = []
    for rel_path in paths:
        if Path(rel_path).is_absolute() or ".." in Path(rel_path).parts:
            errors.append(f"verification {verification_id} path {rel_path} must stay inside repo")
            continue
        path = root / rel_path
        if not path.exists():
            errors.append(f"verification {verification_id} path {rel_path} does not exist")
            continue
        if path.is_file():
            contents.append(path.read_text(encoding="utf-8", errors="replace"))

    haystack = "\n".join(contents)
    for needle in require_string_list(
        verification, "contains", f"verification {verification_id}", errors
    ):
        if needle not in haystack:
            errors.append(
                f"verification {verification_id} evidence does not contain required text {needle!r}"
            )


if __name__ == "__main__":
    raise SystemExit(main())
