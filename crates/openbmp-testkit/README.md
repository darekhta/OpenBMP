# openbmp-testkit

L6 test helpers crate. Used as a `dev-dependency` by every other
crate.

**Status:** Phase 1.6 — stub.

## Purpose

- `proptest::Strategy` constructors for `SimTime`, `Duration`,
  `StepIndex`, `Position3`, `Velocity3`, `Quaternion`,
  `MassProperties`, `PointMassState`, `RigidBodyState`.
- Analytic-toy scenario generators: constant-acceleration drop,
  torque-free Euler rigid-body, two-body Keplerian, harmonic
  oscillator.
- Tolerance-table parser for `expected.toml` files per
  `docs/verification.md` § Tolerance Tables.
- `compare_filters` helper (NorthStarUAS pattern).
- Determinism oracle: run scenario twice, byte-compare outputs.

## Inputs and Outputs

Inputs are test fixtures, generated strategies, expected tolerance tables, and
scenario paths. Outputs are generated test cases, comparison reports, and
structured diffs.

## Units and Frames

Strategies generate frame-tagged and unit-typed values where the production
types require them.

## Assumptions

Helpers are test-only and may depend on dev tooling. They still follow the
determinism profile for generated cases.

## Validity Range

Valid only for test, benchmark, fuzz, and validation code. Runtime crates must
not depend on `openbmp-testkit` as a normal dependency.

## Determinism

All helpers respect the determinism profile. Generated test cases use
seeded RNG.

## Validation

`experimental` (stub). Each helper ships self-tests when implemented.

## Data Provenance

Fixture data follows `docs/data-provenance.md`; analytic toys cite their
derivation.

## Safety Boundary

Test helpers only; no operational vocabulary.
