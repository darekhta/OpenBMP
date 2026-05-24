# openbmp-testkit

L6 test helpers crate. Used as a `dev-dependency` by every other
crate.

**Status:** Implemented shared test helpers.

## Purpose

- `proptest::Strategy` constructors for `SimTime`, `Duration`,
  `StepIndex`, IDs, frame-tagged vectors, `Quaternion`,
  `MassProperties`, `PointMassState`, and `RigidBodyState`.
- Analytic-toy references: constant-acceleration drop, harmonic
  oscillator, and checked two-body Keplerian helpers. A torque-free
  Euler reference helper remains deferred.
- Tolerance-table parser for `expected.toml` files per
  `docs/verification.md` § Tolerance Tables.
- `compare_filters` helper (NorthStarUAS pattern).
- Determinism oracle: byte diff plus a closure-based replay scaffold
  that runs a fixture twice and compares outputs.

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

`checked` for the helper surface. Each helper ships
self-tests.

## Data Provenance

Fixture data follows `docs/data-provenance.md`; analytic toys cite their
derivation.

## Safety Boundary

Test helpers only; no operational vocabulary.
