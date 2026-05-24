# openbmp-aero

L2 aerodynamics crate.

Schema-1 and schema-2 coefficient decks both ship
(`AeroDeck` + `DeckLookup`). Schema-2 adds optional control-effector
axes (e.g. `delta_e_deg`); the lookup signature gains a name-keyed
`BTreeMap<&str, f64>` for deflections and the runner-side
`EffectorRack` snapshot flows through the kernel into the deck. The
internal representation is N-D (3 ≤ N ≤ 6); at N = 3 the multilinear
reduction is bit-identical to the original schema-1 trilinear path.
Six-coefficient coefficient decks (`CY`, `Cl`, `Cn-yaw`) remain
deferred. Hypersonic methods and a separate strict
`openbmp.panel_mesh_aero = 1` TOML parser for
`LocalInclinationPanels` mesh studies are also provided.

## Purpose

- Aerodynamic deck format: tabular `(Mach, alpha, beta) → coefficient`
  with provenance.
- `AeroMethod` trait + implementations:
  - `DeckLookup`.
  - `ModifiedNewtonian`, `TangentCone`, `TangentWedge`,
    `LocalInclinationPanels`, `FreeMolecular`.
  - `HybridAeroMethod` dispatching by Mach + Knudsen.
- Strict panel-mesh TOML ingestion for `LocalInclinationPanels`;
  coefficient decks and mesh-panel decks use separate schema markers.

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

Multilinear interpolation (innermost-axis-first, locked operand order
per Demmel & Nguyen 2020) with documented out-of-grid behaviour
(fail-closed unless explicit extrapolation policy declared). FMA
disabled. At N = 3 (schema-1) the reduction is bit-identical to the
original trilinear formula, asserted by a 1024-case property test.

## Validation

`validated-toy` for both the schema-1 deck surface and the
schema-2 surface. Validated against per-corner
exact-equality regression, eight-corner-centroid average lookup,
single-axis sub-grid lookup, bit-stable clone-equivalence, and
schema-2 lookup at zero deflection matching the schema-1 companion
bit-for-bit, plus analytic hypersonic method checks.

## Data Provenance

All shipped decks are synthetic textbook examples (sphere, cone,
finned cylinder) plus the axisymmetric Calisto drag
deck rendered from RocketPy's MIT-licensed
`powerOff/powerOnDragCurve.csv` (byte-identical upstream), per
`docs/data-provenance.md`. **Real fielded-vehicle aero decks are
categorically rejected** per `docs/safety-boundaries.md`.

## Safety Boundary

No operational vehicle decks, no targeting-driven aero configurations, and no
terminal-mode aerodynamic tuning.

## References

- Anderson, *Fundamentals of Aerodynamics* (6th ed., 2017) — synthetic
  deck examples.
- Anderson, *Hypersonic and High-Temperature Gas Dynamics* (3rd ed.,
  2019) — Modified Newtonian, tangent methods.
- Niskanen 2009 master's thesis — Barrowman extended component build-up.
