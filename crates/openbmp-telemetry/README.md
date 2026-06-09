# openbmp-telemetry

L5 telemetry crate.

**Status:** Implemented. Provides typed telemetry, a ring buffer, and
deterministic CSV / JSON / Parquet exporters plus the
runner-side channels that write per-effector channels, mission event
marker channels, recovery state channels
(`recovery.<id>.deployed/phase_index/drag_area_m2`), and the
`mass_kg` column used by the engine-cluster and
Calisto motor-mass validations. Per-engine and per-tank
diagnostic channels remain future work.

## Purpose

- `TelemetrySink` trait and `TelemetryValue` typed-channel pattern.
- In-memory ring buffer for current-value access.
- Exporters: CSV (deterministic field ordering, fixed-precision float
  formatting), JSON (sorted keys), Parquet (canonical archive with
  unit/frame/version metadata in column descriptors).
- Golden-mode helpers for byte comparison.
- Schema versioning per `docs/software-architecture.md` § Telemetry.

## Inputs and Outputs

Publish: typed value + channel ID + sim time → ring buffer + archive.
Read: ring buffer current values; archive files for offline analysis.

## Units and Frames

Every column carries unit and frame metadata. Consumers that don't
understand the schema version refuse to load.

## Assumptions

Telemetry is simulation output and diagnostics only. It is not a command link,
mission-control protocol, or hardware packet format.

## Validity Range

Valid for telemetry schema versions understood by the linked workspace
version. Unknown schema versions fail closed.

## Determinism

CSV format `"{:.17e}"` for floats; deterministic JSON object ordering is
written explicitly; Parquet column order is locked by schema version.

## Validation

`checked` for the telemetry surface. Unit tests cover
typed channels, bounded retention, CSV / JSON stability, Parquet byte
stability, schema duplicate rejection, non-finite float rejection,
and BTreeMap-ordered schema metadata serialisation. The
`openbmp-cli` end-to-end suite verifies the SHA-256 metadata appears
correctly in Niskanen Parquet output and that the per-model force
breakdown channels carry finite values.

## Data Provenance

This crate ships no data files. It records provenance hashes and scenario
metadata supplied by other crates.

## Scope Boundary

Telemetry **must not** encode hardware command packets, real-bus
messages, or operational mission formats.

## References

- arrow / parquet 58.1.0 (apache/arrow-rs).
- `docs/verification.md` § Golden Telemetry.
