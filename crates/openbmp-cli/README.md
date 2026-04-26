# openbmp-cli

L7 command-line entry point. Produces the binary `openbmp`.

**Status:** Phase 1.7 — stub binary that prints a deferred-implementation
message and exits with code 2.

## Purpose

Expose the supported user entry points for running scenarios, comparing
golden telemetry, checking scenario/provenance policy, and later running
batch sweeps.

## Subcommands (Phase 1.7)

| Subcommand | Purpose |
|---|---|
| `openbmp run <scenario.toml> --output <path>` | Run the scenario through the kernel and write telemetry. |
| `openbmp diff <golden.parquet> <actual.parquet>` | Report the first divergent row with channel + field + values + scenario hash. |
| `openbmp check <scenario.toml>` | Lint: schema, provenance, units / frames, safety names, deterministic schedule. |
| `openbmp check-provenance <data/>` | Walk a data tree and verify provenance records (Phase 2 full implementation). |
| `openbmp batch <sweep.toml>` | Parametric sweep / Monte-Carlo dispersion runs (Phase 5 fuller). |

## Errors and Diagnostics

Errors via `miette` for source-span pointing into TOML scenarios.
Structured output with `--json`. Tracing subscriber configurable via
`RUST_LOG` / `--trace`.

## Inputs and Outputs

Inputs are local scenario, telemetry, provenance, and batch-manifest files.
Outputs are telemetry archives, structured diagnostics, and optional JSON
reports.

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

`experimental` (stub). Phase 1.7 adds command-level snapshot tests and one
end-to-end analytic-toy scenario run.

## Data Provenance

This crate ships no data files. It checks provenance records for files that
scenarios reference.

## Safety Boundary

CLI is a thin wrapper. All safety enforcement lives in the underlying
crates.
