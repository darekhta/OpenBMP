# openbmp-cli

L7 command-line entry point. Produces the binary `openbmp`.

**Status:** Phase 2.11 — runner dispatcher with the byte-stable
analytic-toy path (Phase-1) and the Phase-2 point-mass-with-adapters
path (Niskanen-class sounding rocket). `vehicle.kind = "rigid_body"`
returns a typed Phase-3 deferral message. SHA-256 pin verification
fires before kernel construction. Telemetry extends to atmosphere
sample, per-model force breakdown, and SHA-256 schema metadata for
the Phase-2 path.

## Purpose

Expose the supported user entry points for running scenarios, comparing
golden telemetry, checking scenario/provenance policy, and later running
batch sweeps.

## Subcommands

| Subcommand | Purpose |
|---|---|
| `openbmp run <scenario.toml>` | Run the scenario through the kernel and write declared telemetry outputs. Dispatcher selects between Phase-1 byte-stable analytic-toy and Phase-2 point-mass-with-adapters paths. |
| `openbmp diff <golden.parquet> <actual.parquet>` | Report the first divergent row and column with strict value strings. |
| `openbmp check <scenario.toml>` | Lint: schema, provenance, units / frames, safety names, deterministic schedule. Surfaces the SHA-256 digests for every external-file reference. |
| `openbmp check-provenance <data/>` | Walk a data tree and verify provenance records. |

## Errors and Diagnostics

Errors are printed to stderr with stable exit-code classes. Structured JSON
output is planned for a later CLI phase. Tracing subscriber verbosity is
configurable via `RUST_LOG` / `--trace`.

## Inputs and Outputs

Inputs are local scenario, telemetry, provenance, and batch-manifest files.
Outputs are telemetry archives and human-readable diagnostics.

## Units and Frames

The CLI does not reinterpret units or frames. It reports the metadata produced
by `openbmp-scenario`, `openbmp-core`, and `openbmp-telemetry`.

## Assumptions

The CLI is an offline or simulation-runner tool. It does not provide
mission-control, command-and-control, or live hardware operation surfaces.

## Validity Range

Valid for OpenBMP scenario schemas and telemetry schemas understood by the
linked workspace version.

## Determinism

CLI does not pull `tokio` by default. The `bridge` feature flag gates
optional socket-bridge tooling.

## Validation

`checked` for the Phase-2.11 surface. Snapshot tests cover help-text
shape, scenario-run success on the analytic-toy path, structured-error
behaviour for malformed scenarios, `diff`'s self-compare-identical
golden path, the Niskanen-scenario `check` digest report, corrupt-pin
fail-closed, missing-motor-file fail-closed, and `rigid_body`
unsupported-scenario rejection. The end-to-end suite asserts tolerance
compliance + same-machine byte-stability of Parquet output for both
the analytic-toy drop and the Niskanen sounding-rocket scenarios.
The CI determinism gate runs both scenarios twice with byte-diff and
once with `RUST_LOG=trace` redirected to verify no tracing leak into
deterministic output.

## Data Provenance

This crate ships no data files. It checks provenance records for files that
scenarios reference.

## Safety Boundary

CLI is a thin wrapper. All safety enforcement lives in the underlying
crates. The project-wide accept/reject list and the rejected-naming
rules live in [`docs/safety-boundaries.md`](../../docs/safety-boundaries.md).

## See Also

- [`docs/software-architecture.md`](../../docs/software-architecture.md)
  — the workspace layout and the `openbmp-cli` crate's place in it.
- [`docs/scenario-format.md`](../../docs/scenario-format.md) — the
  scenario TOML schema this CLI parses and runs.
- [`docs/verification.md`](../../docs/verification.md) — golden
  telemetry, tolerance tables, and the determinism CI gate that
  consumes `openbmp diff`.
