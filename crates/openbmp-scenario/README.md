# openbmp-scenario

L6 scenario parser and validator.

**Status:** Phase 1.5 — implemented strict TOML parser, Phase-1 model
registry, unit/frame linting, safety-name linting, and path resolution.

## Purpose

- TOML parser with `serde::deny_unknown_fields`.
- Required / optional table set per `docs/scenario-format.md`.
- Model registry — compile-time-known model names.
- Frame-suffix and unit-suffix linting.
- **Safety-name lint** — reject `target`, `seeker`, `warhead`,
  `strike`, `interceptor`, `kill`, `threat`, `engagement`, terminal-
  homing variants per `docs/safety-boundaries.md` § Naming Rules.
- File-resolution helper rooted at the scenario file path.

## Inputs and Outputs

Scenario file path → validated `Scenario` value, or structured
`ScenarioError` on error.

## Units and Frames

All vector fields carry frame names; all dimensional fields carry
unit suffixes. Parser rejects fields lacking these.

## Assumptions

Scenarios are local files and resolve referenced paths relative to the
scenario path. The parser does not fetch network resources.

## Validity Range

Valid only for known scenario schema versions. Unknown versions, unknown
fields, invalid units, invalid frames, and rejected vocabulary fail closed.

## Determinism

Parser is pure; deterministic ordering is preserved via `BTreeMap`
for registry and resolved path outputs.
Unknown fields produce parse errors (fail-closed).

## Validation

`checked` for the Phase 1.5 parser surface. Unit tests cover the minimal
scenario, path resolution, unknown-field rejection, model registry rejection,
safety-name linting, unit/frame suffix linting, empty force lists, invalid
time ranges, and missing telemetry outputs.

## Data Provenance

This crate ships no data files. It enforces provenance requirements for
scenario-referenced data paths.

## Safety Boundary

Parser-level enforcement of safety-limited names is a structural
backstop for `docs/safety-boundaries.md`.

## References

- `docs/scenario-format.md`.
- `docs/safety-boundaries.md` § Naming Rules.
