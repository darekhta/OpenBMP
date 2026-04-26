# openbmp-telemetry

L5 telemetry crate.

**Status:** Phase 1.4 — stub.

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

CSV format `"{:.17e}"` for floats; deterministic key ordering in JSON
via `IndexMap`; Parquet column order locked by schema version.

## Validation

`experimental` (stub). Phase 1.4 adds round-trip, snapshot, and byte-stability
tests for canonical channels.

## Data Provenance

This crate ships no data files. It records provenance hashes and scenario
metadata supplied by other crates.

## Safety Boundary

Telemetry **must not** encode hardware command packets, real-bus
messages, or operational mission formats.

## References

- arrow / parquet 58.1.0 (apache/arrow-rs).
- `docs/verification.md` § Golden Telemetry.
