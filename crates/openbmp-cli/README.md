# openbmp-cli

L7 command-line entry point. Produces the binary `openbmp`.

**Status:** Phase 1.7 — implemented `run`, `diff`, `check`, and
`check-provenance` subcommands; lib + bin shape with `assert_cmd` /
`insta-cmd` snapshot tests on the binary surface and a Phase-1.8
end-to-end golden test for the analytic-toy constant-acceleration
drop scenario.

## Purpose

Expose the supported user entry points for running scenarios, comparing
golden telemetry, checking scenario/provenance policy, and later running
batch sweeps.

## Subcommands (Phase 1.7)

| Subcommand | Purpose |
|---|---|
| `openbmp run <scenario.toml>` | Run the scenario through the kernel and write declared telemetry outputs. |
| `openbmp diff <golden.parquet> <actual.parquet>` | Report the first divergent row and column with strict value strings. |
| `openbmp check <scenario.toml>` | Lint: schema, provenance, units / frames, safety names, deterministic schedule. |
| `openbmp check-provenance <data/>` | Walk a data tree and verify provenance records (Phase 2 full implementation). |

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

`checked` for the Phase-1.7 surface. Snapshot tests cover help-text
shape; integration tests cover scenario-run success, structured-error
behaviour for malformed scenarios, and `diff`'s self-compare-identical
golden path. The Phase-1.8 e2e test asserts tolerance compliance and
same-machine byte-stability of Parquet output for the analytic-toy
drop scenario.

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
