# OpenBMP Verification

OpenBMP is autotest-driven. This document defines the verification process,
validation labels, golden-output workflow, and public-benchmark rules.

Verification in OpenBMP establishes simulator behavior for academic use. It
does not establish operational flight suitability, safety certification, or
hardware qualification.

## Validation Labels

Every model, dataset, scenario, and validation case declares one label:

| Label | Meaning | Required evidence |
|---|---|---|
| `experimental` | Implemented or drafted, not independently checked | Unit tests or parser tests only |
| `checked` | Internally consistent | Unit/property tests and documentation |
| `validated-toy` | Compared with analytic or simple public examples | Analytic-toy scenario and tolerance |
| `research` | Compared with public academic benchmark cases | Public source, provenance, tolerance table |

No OpenBMP artifact uses labels such as `flight-qualified`, `certified`,
`operational`, or `mission-ready`.

## Test Classes

| Class | Tool | Purpose | Merge gate |
|---|---|---|---|
| Unit | `cargo test` | Small deterministic behavior | Every PR |
| Property | `proptest` | Invariants and fail-closed behavior | Every PR |
| Snapshot | `insta` | Structured diagnostics and summaries | Every PR |
| Golden scenario | `openbmp diff` | Byte-stable telemetry | Every PR for small set |
| Analytic-toy | custom | Closed-form physics checks | Every PR |
| Public benchmark | custom | Public academic reference cases | Nightly, then PR for touched models |
| Fuzz | `cargo-fuzz` | Parsers and config surfaces | Nightly |
| Doc test | `rustdoc` | Public examples compile | Every PR |
| Microbenchmark | `criterion` | Performance trend detection | Nightly |
| Provenance check | `openbmp check-provenance` | Data source review | Every PR once data exists |
| Supply-chain check | `cargo deny`, `cargo vet` | Dependency policy | Every PR or dependency PR |

## Golden Telemetry

Golden tests compare canonical scenario output against committed reference
telemetry.

Rules:

- Golden scenarios run with the default deterministic profile unless they are
  explicitly marked `state-stable`.
- Golden output includes telemetry schema version, frame metadata, scenario
  hash, data hashes, toolchain profile, and OpenBMP version.
- Golden updates require review. A changed golden file is treated as a
  behavioral change, not as generated noise.
- CSV may be used for human-readable diffs; Parquet is the canonical archive
  when implemented.

The diff tool reports:

- First divergent row.
- Channel and field.
- Expected and actual values.
- Absolute and relative error.
- Scenario hash and determinism profile.

## Tolerance Tables

Every analytic or benchmark validation case has an `expected.toml` file:

```toml
case = "allen-eggers-ballistic-entry"
source = "NACA Report 1381"
validation = "validated-toy"

[[metric]]
name = "peak_deceleration_g"
expected = 12.3
absolute_tolerance = 0.05
relative_tolerance = 0.01
```

The tolerance file is part of the validation claim. If a tolerance is widened,
the PR must explain why the previous tolerance was wrong or too narrow.

## Determinism Gate

CI runs the canonical scenario set twice on the reference platform profile and
requires byte-identical output.

The reference profile is:

- `x86_64-unknown-linux-gnu`.
- Pinned Rust toolchain from `rust-toolchain.toml`.
- Default simulation profile.
- Fixed target features documented in [software-architecture.md](software-architecture.md).

Other supported platform profiles are `state-stable, not bit-stable` unless
the project later proves byte identity there too.

## Scenario Fuzzing

Fuzz targets should cover:

- Scenario parser.
- Aero deck parser.
- Motor parser.
- Provenance manifest parser once machine-readable.
- Telemetry metadata reader.

Fuzz failures must produce diagnostics, not panics, memory unsafety, or partial
runs.

## Public Benchmark Rules

Public benchmark cases require:

- A provenance record.
- A source URL or stable citation.
- A local expected-data file when redistribution is allowed.
- A documented tolerance envelope.
- A note describing what is intentionally not validated.

Benchmarks from real operational systems are rejected unless the case is a
public civilian/academic reference and does not introduce real fielded-vehicle
parameter sets outside the safety boundary.

## Review Checklist

Before accepting a model:

- Are units and frames documented?
- Is the validity range explicit?
- Is the validation label justified?
- Are failure modes fail-closed?
- Does at least one test exercise the model through a scenario?
- Does the model avoid wall-clock time, system RNG, network access, and
  unordered iteration?
- Is all external data covered by provenance?
- Does the model stay inside the safety boundary?

If any answer is unclear, the validation label remains `experimental`.
