# openbmp-scenario

L6 scenario parser and validator.

**Status:** Phase 1.5 — stub.

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
`miette` diagnostic on error.

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

Parser is pure; deterministic ordering preserved via `IndexMap`.
Unknown fields produce parse errors (fail-closed).

## Validation

`experimental` (stub). Phase 1.5 adds parser unit tests, snapshot diagnostics,
property tests, and a fuzz target.

## Data Provenance

This crate ships no data files. It enforces provenance requirements for
scenario-referenced data paths.

## Safety Boundary

Parser-level enforcement of safety-limited names is a structural
backstop for `docs/safety-boundaries.md`.

## References

- `docs/scenario-format.md`.
- `docs/safety-boundaries.md` § Naming Rules.
