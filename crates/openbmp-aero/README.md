# openbmp-aero

L2 aerodynamics crate.

**Status:** Phase 2 — schema-1 deck shipped (`AeroDeck` +
`DeckLookup`). Schema-2 with control-effector axes lands in Phase 3.
Hypersonic extensions in Phase 6.

## Purpose

- Aerodynamic deck format: tabular `(Mach, alpha, beta) → coefficient`
  with provenance.
- `AeroMethod` trait + implementations:
  - `DeckLookup` (Phase 2).
  - `ModifiedNewtonian`, `TangentCone`, `TangentWedge`,
    `LocalInclinationPanels`, `FreeMolecular` (Phase 6.3).
  - `HybridAeroMethod` dispatching by Mach + Knudsen (Phase 6.6).

## Inputs and Outputs

`AeroContext` (state, atmosphere, geometry) → `AeroForceMoment`
(force in `ECI`, moment in `Body`).

## Units and Frames

Forces in `ECI`, moments in `Body`. Aero context references both.

## Assumptions

Aerodynamic coefficients are academic, synthetic, textbook, or public
educational data only. Deck lookup is the v1 boundary; OpenBMP does not ship
real fielded-vehicle coefficient decks.

## Validity Range

Each deck or method declares its own Mach, angle, Reynolds/Knudsen, and
geometry validity ranges. Out-of-range use fails closed unless an explicit
extrapolation policy is documented in the deck.

## Determinism

Trilinear interpolation with documented out-of-grid behaviour
(fail-closed unless explicit extrapolation policy declared).

## Validation

`validated-toy` for the Phase-2.5 schema-1 deck surface. Validated
against per-corner exact-equality regression, eight-corner-centroid
average lookup, single-axis sub-grid lookup, and bit-stable
clone-equivalence. Phase 6 adds analytic hypersonic method checks.

## Data Provenance

All shipped decks are synthetic textbook examples (sphere, cone,
finned cylinder) per `docs/data-provenance.md`. **Real fielded-vehicle
aero decks are categorically rejected** per
`docs/safety-boundaries.md`.

## Safety Boundary

No operational vehicle decks, no targeting-driven aero configurations, and no
terminal-mode aerodynamic tuning.

## References

- Anderson, *Fundamentals of Aerodynamics* (6th ed., 2017) — synthetic
  deck examples.
- Anderson, *Hypersonic and High-Temperature Gas Dynamics* (3rd ed.,
  2019) — Modified Newtonian, tangent methods.
- Niskanen 2009 master's thesis — Barrowman extended component build-up.
