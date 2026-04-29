# openbmp-scenario

L6 scenario parser and validator.

**Status:** Phase 2.10 — Phase-2 structured blocks (`[aero]`,
`[propulsion.motor]`, `[wind]`, `[atmosphere]`,
`[frames.local_origin]`, typed `[sensors.<name>]`) plus optional
rigid-body initial-state fields and SHA-256 pin verification on
external-file references. Schema header stays `openbmp.scenario = 1`
(append-only — Phase-1 scenarios continue to parse byte-identically).

## Purpose

- TOML parser with `serde::deny_unknown_fields`.
- Required / optional table set per `docs/scenario-format.md`,
  including the Phase-2 structured blocks.
- Model registry: `ModelRegistry::phase1()` (analytic-toy) and
  `ModelRegistry::phase2()` (sounding rocket — adds rigid-body
  vehicle, `j2`/`point_mass` gravity, `us_standard_1976` atmosphere,
  `constant`/`layered`/`gust` wind, `aero`/`thrust` forces, three
  sensor kinds, and the `solid` motor variant).
- Frame-suffix and unit-suffix linting (`_n_s`, `_m3_s2`, `_deg`,
  `_xyzw` added in Phase 2.10).
- **Safety-name lint** — reject `target`, `seeker`, `warhead`,
  `strike`, `interceptor`, `kill`, `threat`, `engagement`, terminal-
  homing variants per `docs/safety-boundaries.md` § Naming Rules.
- File-resolution helper rooted at the scenario file path; the
  `Scenario::resolved_files()` API loads each external file
  reference, computes its SHA-256 digest, and verifies the optional
  `*_sha256` pin (fail-closed on mismatch).

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

`checked` for the Phase-2.10 parser surface. Unit tests cover the
Phase-1 minimal scenario (byte-stability guard), path resolution,
unknown-field rejection, model-registry resolution under both
`phase1` and `phase2` registries, safety-name linting, unit/frame
suffix linting, empty force lists, invalid time ranges, missing
telemetry outputs, and every Phase-2.10 cross-validation rule
(rigid-body-without-quaternion, point-mass-with-quaternion,
non-unit-quaternion, latitude-out-of-range, atmosphere/wind
kind-mismatch, ideal-state-sensor-with-pin-only, gravity-coefficient
mismatches, force-dependency-without-block, and frame-profile
disagreement). Determinism tests cover SHA-256 pin verification
(known FIPS PUB 180-4 vectors, missing-file fail-closure, malformed
pin, upper-case-pin acceptance, mismatch fail-closure).

## Data Provenance

This crate ships no data files. It enforces provenance requirements for
scenario-referenced data paths.

## Safety Boundary

Parser-level enforcement of safety-limited names is a structural
backstop for `docs/safety-boundaries.md`.

## References

- `docs/scenario-format.md`.
- `docs/safety-boundaries.md` § Naming Rules.
