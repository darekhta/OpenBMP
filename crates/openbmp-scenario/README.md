# openbmp-scenario

L6 scenario parser and validator.

**Status:** Implemented. Parses the structured blocks (`[aero]`,
`[propulsion.motor]`, `[wind]`, `[atmosphere]`,
`[frames.local_origin]`, typed `[sensors.<name>]`),
`[mission]` (phases / events / transitions),
the `[vehicle.assembly]` tree (bodies, effectors, engines, tanks,
recovery), aero deck schema-2 (control-effector axes), engine
clusters with per-engine throttle / gimbal / ignition / shutdown
commands, layered + gust winds, and the sensor variants
(`gnss`, `magnetometer`, `star_tracker`). The schema header is
`openbmp.scenario = 2`; the v1 flat vehicle shape is retired and
every scenario declares `[vehicle.assembly]`.

## Purpose

- TOML parser with `serde::deny_unknown_fields`.
- Required / optional table set per `docs/scenario-format.md`,
  including the structured blocks.
- Model registry: `ModelRegistry::base()` (analytic-toy) and
  `ModelRegistry::full()` (adds rigid-body
  vehicle, `j2`/`point_mass` gravity, `us_standard_1976`,
  `nrlmsise00`, and `nrlmsis2_compat` atmosphere,
  `constant`/`layered`/`gust` wind, `aero`/`thrust` forces, six
  sensor kinds (`ideal_state` / `imu` / `barometer` plus
  `gnss` / `magnetometer` / `star_tracker`), and the
  `solid` motor variant).
- Frame-suffix and unit-suffix linting (`_n_s`, `_m3_s2`, `_deg`,
  `_xyzw`).
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
fields, invalid units, and invalid frames fail closed.

## Determinism

Parser is pure; deterministic ordering is preserved via `BTreeMap`
for registry and resolved path outputs.
Unknown fields produce parse errors (fail-closed).

## Validation

`checked` for the parser surface. Unit tests cover the
schema-v2 analytic-toy scenario (byte-stability guard), path resolution,
unknown-field rejection, model-registry resolution under both
base and full registries, unit/frame
suffix linting, empty force lists, invalid time ranges, missing
telemetry outputs, and every cross-validation rule
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

## References

- `docs/scenario-format.md`.
