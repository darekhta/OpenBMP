# openbmp-aerothermal

L2 aerothermal heat-transfer crate.

**Status:** Audited research-toy implementation. Full design in
[`docs/hypersonic-extensions.md`](../../docs/hypersonic-extensions.md).

## Purpose

- `HeatTransferModel` trait — stagnation + distributed heating.
- Stagnation: `FayRiddell` (cold-gas trait path, caller-supplied
  edge-state assembly, and neutral-composition `h_D` helper),
  `SuttonGraves` (point and Allen-Eggers
  trajectory heat-load diagnostics), `TauberSuttonRadiative`
  (typed-reserved pending coefficients).
- Distributed: reference-enthalpy method, Spalding-Chi, Van Driest II.
- Boundary layer: `Laminar`, `Transitional`, `Turbulent` with
  empirical, Reθ/M_e, e^N transition models.
- 1-D explicit-FTCS thermal-conduction toy.
- Generic ablation toy (`SteadyStateAblator`, `CharringAblator`).

## Inputs and Outputs

`AerothermalContext` → `StagnationHeating` / `SurfaceHeating`,
`BoundaryLayerState`, surface temperature evolution, recession rate.

## Units and Frames

Heat flux in W/m². Temperatures in K. Surface stations referenced in
body geometry.

## Assumptions

Models are engineering correlations and academic research toys. They are not
TPS sizing tools and do not claim flight qualification or operational thermal
protection design suitability.

## Validity Range

Each heat-transfer, boundary-layer, chemistry, and ablation model declares its
own flow-regime and material validity range. Invalid coupling or unstable
thermal steps fail closed.

## Determinism

Stiff-chemistry sub-step counts and tolerances declared in scenario
and locked into the determinism profile.

## Validation

Validation uses analytic heating checks, textbook ablation toys,
and public academic benchmark cases only. Models with missing public
coefficients fail closed instead of returning guessed values.

## Data Provenance

Any coefficients, reaction sets, or toy-material tables ship with
`provenance.md` records per `docs/data-provenance.md`.

## Scope Boundary

Generic textbook materials ship in-tree. Downstream users who need specific
material packages own the provenance, applicability, and validation evidence
for those packages.

## References

- Fay & Riddell 1958, DOI `10.2514/8.7517`.
- NIST Chemistry WebBook SRD 69, gas-phase formation enthalpies for
  atomic nitrogen, atomic oxygen, and nitric oxide.
- Sutton & Graves, NASA TR R-376, 1971.
- NASA/TP-2006-213486, Stardust SRC entry trajectory / heating tables.
- Tauber & Sutton, NASA TM-86767, 1986.
- Anderson, *Hypersonic and High-Temperature Gas Dynamics* (3rd ed.).
- Hirschel, *Basics of Aerothermodynamics* (2nd ed., 2015).
- Park, *Nonequilibrium Hypersonic Aerothermodynamics* (1990).
