# Clarabel Solver Vetting

Phase 4.C uses `clarabel` only behind the `openbmp-fc/mpc` feature.

- Version: `0.9.0`
- Cargo checksum: `83e62eacd93b899251364a22bd4dca0f293f8175d05311e42bbf6dbb5edcc762`
- License: Apache-2.0
- Implementation: pure Rust conic interior-point solver; no C solver bindings.
- FC use sites: `crates/openbmp-fc/src/mpc.rs` and `crates/openbmp-fc/src/landing.rs`.
- Scheduling contract: solver calls occur only from deterministic FC tick code, under the lockstep simulation clock.
- Deterministic profile: `verbose = false`, fixed iteration cap, pinned feasibility/gap tolerances, and no wall-clock feedback in OpenBMP call sites.

Note: `clarabel 0.9.0` does not compile with `default-features = false` unless its `serde` feature is enabled because the crate leaves `#[serde(...)]` attributes present on generic types. OpenBMP therefore declares `default-features = false, features = ["serde"]`; no OpenBMP solver code uses JSON I/O.
