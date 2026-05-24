# OpenBMP — Open Body Motion Platform

OpenBMP is a Rust-first, simulation-only academic research platform for
rigid-body dynamics with a primary focus on rocket-class and
launch-vehicle-class flight simulation. The project ships a simulator;
**its abstractions are designed to be re-implementable against real
hardware** through a downstream HAL, but the OpenBMP repository itself
ships no HAL and does not validate or support hardware deployment.

> OpenBMP is an academic simulation platform. It is not validated for
> operational flight, not suitable for hardware deployment, and not a
> substitute for any qualified flight-software stack. No compliance
> claims are made under IEC 61508, ISO 26262, DO-178C, or equivalent
> regimes.

## Status

**Phase 6 — Hypersonic / re-entry extensions** is in delivery. See
[`docs/phase-6-plan.md`](docs/phase-6-plan.md) for sub-phase status.

The deterministic kernel (Phase 1), physics fidelity (Phases 2-3),
flight-controller and estimator stack (Phases 4-5), and mission-graph
architecture refactor (Phase 5.X) all closed before Phase 6 opened.
Phase 6 ships the hypersonic atmosphere, real-gas thermodynamics,
hypersonic aero methods, aerothermal heat transfer, Park 2T
nonequilibrium thermochemistry, generic ablation toy, re-entry
trajectory infrastructure, external reference packages, and UQ /
credibility reporting described in
[`docs/hypersonic-extensions.md`](docs/hypersonic-extensions.md).

## Documentation

All authoritative project documentation is in [`docs/`](docs/):

- [Design Concept](docs/design-concept.md)
- [Software Architecture](docs/software-architecture.md)
- [Safety Boundaries](docs/safety-boundaries.md)
- [Hypersonic Extensions](docs/hypersonic-extensions.md)
- [Scenario Format](docs/scenario-format.md)
- [Verification](docs/verification.md)
- [Frames and Time](docs/frames-time.md)
- [Data Provenance](docs/data-provenance.md)
- [Supply Chain](docs/supply-chain.md)
- [Modeling Guide](docs/modeling-guide.md)
- [Glossary](docs/glossary.md)
- [Phase 6 Plan](docs/phase-6-plan.md)
- [Hypersonic Extensions](docs/hypersonic-extensions.md)

## Workspace Layout

```text
crates/
  openbmp-core/        # L0  math, units, frames, time, RNG
  openbmp-state/       # L1  state types
  openbmp-sim/         # L1  kernel, scheduler, integrators
  openbmp-physics/     # L2  atmosphere, gravity, wind, magnetic, error
  openbmp-vehicle/     # L2  rigid-body / mass models
  openbmp-aero/        # L2  aero decks + hypersonic methods
  openbmp-aerothermal/ # L2  heat transfer, BL, ablation toy
  openbmp-propulsion/  # L2  motors, thrust curves
  openbmp-sensors/     # L3  synthetic sensors, fault models
  openbmp-fc/          # L4  virtual flight controller
  openbmp-telemetry/   # L5  channels, ring buffer, exporters
  openbmp-scenario/    # L6  parser, validator, registry
  openbmp-testkit/     # L6  helpers, fixtures, fuzzers
  openbmp-cli/         # L7  scenario runner, diff, check
  openbmp-bridge/      # L7  optional generic socket-bridge HIL
```

See [`docs/software-architecture.md`](docs/software-architecture.md) for
the full layered design and crate dependency graph.

## Toolchain

- **Rust 1.95** stable, **Edition 2024**, MSRV `1.93`.
- Pinned in [`rust-toolchain.toml`](rust-toolchain.toml).
- `[workspace.dependencies]` in [`Cargo.toml`](Cargo.toml) pins all
  external crates to verified April 2026 versions.

## Building

```bash
cargo build --workspace
cargo nextest run --workspace --all-features
cargo clippy --workspace --all-targets -- -D warnings
cargo deny check
```

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for the contribution workflow,
the safety-boundary review checklist, and the documentation discipline
required before any code or data change is accepted.

Project conduct expectations are in
[`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).

## Security

See [`SECURITY.md`](SECURITY.md) for vulnerability disclosure.

## License

Dual licensed under either of:

- Apache License, Version 2.0 ([`LICENSE-APACHE`](LICENSE-APACHE) or
  <https://www.apache.org/licenses/LICENSE-2.0>).
- MIT License ([`LICENSE-MIT`](LICENSE-MIT) or
  <https://opensource.org/licenses/MIT>).

at your option.

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the
Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
