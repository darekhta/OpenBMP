# openbmp-cli

L7 command-line entry point. Produces the binary `openbmp`.

**Status:** Phase 3 — runner dispatcher with the byte-stable
analytic-toy path (Phase-1), the Phase-2 point-mass-with-adapters
path (Niskanen-class sounding rocket), and the Phase-3 rigid-body
runner consuming the assembly tree, engine clusters, control
effectors, tanks / moving-mass models, recovery devices, mission
events, and layered + gust winds. Phase-3.10 sensor declarations
parse and pin their external files; runner-side measurement
telemetry remains future work. SHA-256 pin verification fires before
kernel construction. Telemetry extends to atmosphere sample,
per-model force breakdown, SHA-256 schema metadata, mission event
marker channels, recovery state channels, and the mass column used
to validate the Phase-3.11 Calisto motor-mass profile.

## Purpose

Expose the supported user entry points for running scenarios, comparing
golden telemetry, checking scenario/provenance policy, and later running
batch sweeps.

## Subcommands

| Subcommand | Purpose |
|---|---|
| `openbmp run <scenario.toml>` | Run the scenario through the kernel and write declared telemetry outputs. Dispatcher selects between Phase-1 byte-stable analytic-toy, Phase-2 point-mass-with-adapters, and Phase-3 rigid-body paths by scenario shape. |
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

`checked` for the Phase-3 surface. Snapshot tests cover help-text
shape, scenario-run success on the analytic-toy path, structured-
error behaviour for malformed scenarios, `diff`'s self-compare-
identical golden path, the Niskanen-scenario `check` digest
report, corrupt-pin fail-closed, and missing-motor-file fail-
closed. The end-to-end suite covers the analytic-toy drop, the
Niskanen sounding-rocket case, the Phase-3 multi-body / engine-
cluster / effector / parachute-recovery / tank-slosh scenarios,
and the Phase-3.11 RocketPy Calisto cross-tool case (apogee
within an audited 2 % envelope of RocketPy's published 3349 m
AGL plus byte-stable Parquet across two reruns). The CI
determinism gate runs the analytic-toy, Niskanen, and Calisto
scenarios twice with byte-diff and once with `RUST_LOG=trace`
redirected to verify no tracing leak into deterministic output.

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
