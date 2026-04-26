# OpenBMP Supply-Chain Policy

OpenBMP is a Rust-first academic simulator, but it still needs a clear
dependency and release-artifact policy. This document defines the minimum
supply-chain checks for project code, generated data, and releases.

Supply-chain checks support reproducibility and review. They are not safety
certification, export-control clearance, or operational qualification.

## Dependency Policy

Rust dependencies must be:

- Declared in `Cargo.toml` and locked in `Cargo.lock`.
- Compatible with the project license policy.
- Free of known blocking security advisories at release time.
- Sourced from crates.io or an explicitly approved git source.
- Audited or accepted through the project's dependency-review process.

The deterministic kernel and model crates should keep dependencies small.
Optional tooling crates may have broader dependency trees, but they must stay
out of the kernel.

## Required Tools

Phase 1 CI should include:

| Tool | Purpose |
|---|---|
| `cargo deny` | Licenses, advisories, duplicate/banned crates, source policy |
| `cargo vet` | Third-party Rust dependency audit records |
| `cargo cyclonedx` | CycloneDX SBOM for release artifacts |
| `cargo audit` | Optional advisory check when not covered by `cargo deny` |

`cargo deny` should be blocking for licenses, banned crates, and sources.
Advisories may start as warning-only in early development to avoid surprise CI
breakage, then become blocking for releases.

## Banned Dependency Patterns

The OpenBMP repository rejects dependencies that introduce:

- Real hardware device drivers into core or model crates.
- Real bus protocol stacks such as MAVLink, CAN, DDS, MIL-STD-1553, I2C, SPI,
  or UART integrations.
- Runtime plugin loading for deterministic model execution.
- Hidden network access in the simulation kernel.
- System RNG or wall-clock access in deterministic model paths.
- Unreviewed native build scripts in core crates.

Exceptions must be documented in `supply-chain.md` or the future
`deny.toml`, and they must explain why the dependency does not affect the
deterministic kernel.

## Release Artifacts

Every release should include:

- Source archive.
- Built CLI binary, if releases publish binaries.
- `Cargo.lock`.
- CycloneDX SBOM.
- Dependency audit/vet summary.
- Data provenance report.
- Scenario and golden-test manifest.
- Non-suitability disclaimer from [safety-boundaries.md](safety-boundaries.md).

Release metadata should include the build platform, Rust toolchain, target
triple, enabled features, and simulation profile.

## Build Provenance

OpenBMP should adopt SLSA-style build provenance for releases:

- Build steps are scripted.
- Build inputs are named and hashed.
- The builder identity is recorded.
- The produced artifacts are named and hashed.
- Consumers can verify that an artifact came from the expected repository,
  commit, workflow, and toolchain.

This is release provenance, not a claim that OpenBMP is suitable for
operational flight.

## Data Supply Chain

Data files follow [data-provenance.md](data-provenance.md). Release builds
should fail when a shipped data file lacks a provenance record or when a
record names a rejected source class.

Generated data must be reproducible from committed scripts and documented
inputs. If the source license does not allow redistribution of the raw source,
the provenance record still names where and when it was retrieved and stores
the hash when legally practical.

## Dependency Review Checklist

Before adding a dependency:

- Is the dependency needed in the core runtime, or only tooling?
- Can the same result be achieved with a smaller existing dependency?
- Does the dependency use wall-clock time, threads, global state, network
  access, or native code in ways that affect determinism?
- Is the license allowed?
- Does it pull in banned protocol or hardware integrations?
- Is it maintained?
- Does it have known advisories?
- Can it be audited with `cargo vet`?

Dependencies that fail this review stay out of the deterministic core.
