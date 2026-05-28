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

## Related Work and License Boundaries

Academic or open-source simulators may be cited as related work when they
document the same public equations OpenBMP implements, but their source code is
not read, copied, ported, or translated into OpenBMP unless the license and
provenance record explicitly allow it.

The `openMotor` project is GPL-3.0 and is treated only as related work for
solid-motor internal-ballistics concepts. OpenBMP's grain regression solver is
implemented from published textbook and public technical references, not from
`openMotor` code.

OpenRocket is GPL-licensed and is likewise treated only as related work for
aerodynamics concepts already available from public technical references. The
continuum drag buildup in `openbmp-aero` is implemented from textbook/public
relations (Hoerner, Barrowman, DATCOM, Niskanen's published documentation),
not from OpenRocket source. No OpenRocket source is read or ported into
OpenBMP.

## Required Tools

CI and release tooling include:

| Tool | Purpose |
|---|---|
| `cargo deny` | Licenses, advisories, duplicate/banned crates, source policy |
| `cargo cyclonedx` | CycloneDX SBOM bundle for release artifacts |
| `cargo audit` | Advisory check in CI and release artifact generation |
| `cargo machete` | Unused dependency detection |

`cargo deny` should be blocking for licenses, banned crates, and sources.
Advisories may start as warning-only in early development to avoid surprise CI
breakage, then become blocking for releases.

`cargo vet` is the third-party dependency audit ledger, but it is
not wired as a gate until the repository carries a
`supply-chain/audits.toml` policy. Do not describe dependencies as vetted before
that configuration lands.

## Toolchain and Policy Pins

The reference toolchain is pinned in `rust-toolchain.toml`: Rust
`1.95`, `rustfmt`, `clippy`, and the `x86_64-unknown-linux-gnu` target. The
workspace MSRV remains `1.93` and is checked by CI with a separate Rust `1.93`
build. Byte-stable replay is guaranteed only inside the reference platform
profile documented in [verification.md](verification.md) and
[software-architecture.md](software-architecture.md).

The repository-level `.cargo/config.toml` disables incremental compilation and
turns off FMA code generation for the reference Linux target. That setting is
part of the deterministic numerics contract: a future change to target CPU,
target features, or FMA policy is a behavior change and must trigger
golden-output review.

The `deny.toml` graph uses `all-features = true` so policy checks include
optional workspace surfaces, not only the default dependency set. Its explicit
ban list records known yanked or deprecated dependency versions from the
dependency survey so a routine `cargo update` cannot reintroduce them.

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
- CycloneDX SBOM bundle with one JSON BOM per workspace crate.
- Dependency audit report; release-time advisory checks are blocking.
- Build manifest naming the commit, Rust toolchain, target triple, profile, and
  feature set.
- Scenario and golden-test manifest with hashes for committed scenario files and
  expected validation tables.
- Data provenance report once `data/` ships files.
- Non-suitability disclaimer from the top-level `DISCLAIMER.md`.

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
