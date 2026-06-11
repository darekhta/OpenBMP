#!/usr/bin/env python3
"""Validate the parity-program documentation set."""

from __future__ import annotations

import re
import sys
import urllib.parse
from pathlib import Path


EXPECTED_FILES = {
    "00-overview.md",
    "01-flexible-multibody-dynamics.md",
    "02-structural-dynamics-loads-slosh-pogo.md",
    "03-aerodynamics-database-and-cfd-coupling.md",
    "04-aerothermal-realgas-and-tps.md",
    "05-propulsion-high-fidelity.md",
    "06-gnc-coupled-mimo-and-control.md",
    "07-trajectory-optimization-and-mission-design.md",
    "08-environment-gravity-and-frames.md",
    "09-sensors-navigation-and-actuators.md",
    "10-flight-software-in-the-loop-xil.md",
    "11-monte-carlo-uq-and-validation.md",
    "12-determinism-realtime-and-compute.md",
    "13-agent-execution-playbook.md",
    "14-contact-dynamics-touchdown-and-landing.md",
    "15-plume-environments-and-supersonic-retropropulsion.md",
    "16-parachute-decelerator-and-recovery-systems.md",
    "17-cryogenic-fluid-management.md",
    "18-ground-segment-countdown-and-launch-operations.md",
    "19-day-of-launch-winds-and-commit-operations.md",
    "20-telemetry-rf-links-and-ground-network.md",
    "21-run-data-regression-and-visualization.md",
    "22-acoustics-vibroacoustics-and-overpressure.md",
    "23-electrical-power-and-avionics-emulation.md",
    "24-postflight-reconstruction-and-model-correlation.md",
    "25-rendezvous-proximity-operations-and-docking.md",
    "progress.md",
}

SECTION_PREFIXES = (
    "## 1. Parity target",
    "## 2. Current state in source",
    "## 3. Target architecture",
    "## 4. Invariant preservation",
    "## 5. V&V plan",
    "## 7. Dependencies",
    "## 8. Open-source leverage",
    "## 9. Work-package backlog",
    "## 10. References",
)

REQUIRED_WP_FIELDS = (
    "goal",
    "fidelity_tier",
    "depends_on",
    "new_crates",
    "touched",
    "approach",
    "acceptance",
    "validation_label",
    "dual_use_note",
    "est_effort",
    "parity_ceiling",
)

FORBIDDEN_TEXT = (
    re.compile(r"\bTODO\b|\bTBD\b|\bFIXME\b|\bXXX\b"),
    re.compile(r"project memory|per memory|memory note|memory policy", re.IGNORECASE),
    re.compile(r"task #[0-9]+", re.IGNORECASE),
    re.compile(r"\bslop\b", re.IGNORECASE),
    re.compile(r"docs/dual-use-assessment\.md"),
)

WP_HEADING = re.compile(
    r"(?m)^(?:#{2,4}\s+|\*\*)"
    r"(?P<wp>WP-(?P<doc>\d{2})\.[A-Za-z0-9.-]+)"
    r"\s+[—-]\s+"
    r"(?P<title>.+?)"
    r"(?:\*\*)?\.?\s*$"
)

MARKDOWN_LINK = re.compile(r"\[[^\]]+\]\(([^)]+)\)")
BACKTICKED_DOC_PATH = re.compile(r"`(docs/[^`]+?\.md)`")


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parity_dir = root / "docs" / "parity"
    errors: list[str] = []

    if not parity_dir.is_dir():
        print("parity docs: docs/parity directory is missing", file=sys.stderr)
        return 1

    files = {path.name for path in parity_dir.glob("*.md")}
    for missing in sorted(EXPECTED_FILES - files):
        errors.append(f"docs/parity/{missing} is missing")
    for extra in sorted(files - EXPECTED_FILES):
        errors.append(f"docs/parity/{extra} is not part of the parity doc set")

    for path in sorted(parity_dir.glob("*.md")):
        text = path.read_text(encoding="utf-8")
        validate_forbidden_text(path, text, errors)
        validate_local_links(root, path, text, errors)

        if path.name[0:2].isdigit() and path.name not in {"00-overview.md", "13-agent-execution-playbook.md"}:
            validate_design_doc(path, text, errors)

    validate_index_links(root, errors)

    if errors:
        for error in errors:
            print(f"parity docs: {error}", file=sys.stderr)
        return 1

    print(f"parity docs: {len(EXPECTED_FILES)} files validated")
    return 0


def validate_forbidden_text(path: Path, text: str, errors: list[str]) -> None:
    for pattern in FORBIDDEN_TEXT:
        for match in pattern.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            errors.append(f"{path}:{line}: forbidden placeholder/internal text {match.group(0)!r}")


def validate_local_links(root: Path, path: Path, text: str, errors: list[str]) -> None:
    candidates: list[str] = []
    for match in MARKDOWN_LINK.finditer(text):
        candidates.append(match.group(1))
    for match in BACKTICKED_DOC_PATH.finditer(text):
        candidates.append(match.group(1))

    for raw_link in candidates:
        link = raw_link.split("#", 1)[0]
        if not link or re.match(r"^[a-z][a-z0-9+.-]*:", link):
            continue
        link = urllib.parse.unquote(link)
        if "*" in link:
            continue
        if link.startswith("/") or link.startswith(("docs/", "crates/", "data/", "scenarios/", "scripts/")):
            target = root / link.lstrip("/")
        else:
            target = path.parent / link
        if not target.exists():
            line = line_number_for(text, raw_link)
            errors.append(f"{path}:{line}: local link/path does not exist: {raw_link}")


def validate_design_doc(path: Path, text: str, errors: list[str]) -> None:
    for prefix in SECTION_PREFIXES:
        if prefix not in text:
            errors.append(f"{path}: missing required section prefix {prefix!r}")

    backlog = text.split("## 9. Work-package backlog", 1)
    if len(backlog) != 2:
        return

    matches = list(WP_HEADING.finditer(backlog[1]))
    if not matches:
        errors.append(f"{path}: work-package backlog contains no WP headings")
        return

    doc_number = path.name[:2]
    for index, match in enumerate(matches):
        wp_id = match.group("wp")
        wp_doc = match.group("doc")
        title = match.group("title").strip(" *")
        if wp_doc != doc_number:
            errors.append(f"{path}: {wp_id} does not match document number {doc_number}")
        if not title:
            errors.append(f"{path}: {wp_id} heading has no title")

        start = match.end()
        end = matches[index + 1].start() if index + 1 < len(matches) else len(backlog[1])
        section = backlog[1][start:end]
        for field in REQUIRED_WP_FIELDS:
            if re.search(rf"(?m)^-\s+\*\*{re.escape(field)}:\*\*", section) is None:
                errors.append(f"{path}: {wp_id} missing field {field}")


def validate_index_links(root: Path, errors: list[str]) -> None:
    readme = (root / "README.md").read_text(encoding="utf-8")
    docs_readme = (root / "docs" / "README.md").read_text(encoding="utf-8")

    if "docs/parity/00-overview.md" not in readme:
        errors.append("README.md must link to docs/parity/00-overview.md")
    if "parity/00-overview.md" not in docs_readme:
        errors.append("docs/README.md must link to parity/00-overview.md")
    if "parity/13-agent-execution-playbook.md" not in docs_readme:
        errors.append("docs/README.md must link to parity/13-agent-execution-playbook.md")


def line_number_for(text: str, needle: str) -> int:
    index = text.find(needle)
    if index < 0:
        return 1
    return text.count("\n", 0, index) + 1


if __name__ == "__main__":
    raise SystemExit(main())
