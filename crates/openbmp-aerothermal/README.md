# openbmp-aerothermal

L2 aerothermal heat-transfer crate.

**Status:** Phase 6 — stub. Implementation begins after Phase 5
closes; full design in
[`docs/hypersonic-extensions.md`](../../docs/hypersonic-extensions.md).

## Purpose

- `HeatTransferModel` trait — stagnation + distributed heating.
- Stagnation: `FayRiddell`, `SuttonGraves`, `TauberSutton`.
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

`experimental` (stub). Phase 6 validation uses analytic heating checks,
textbook ablation toys, and public academic benchmark cases only.

## Data Provenance

Any coefficients, reaction sets, or toy-material tables ship with
`provenance.md` records per `docs/data-provenance.md`.

## Safety Boundary

**No real fielded TPS material parameters** (PICA, AVCOAT, RCC,
SLA-561V, etc.). Generic textbook materials only. **No operational
HGV / MaRV / cruise-weapon data.** See `docs/safety-boundaries.md`
§ Hypersonic Extensions.

## References

- Fay & Riddell 1958.
- Sutton & Graves, NASA TR R-376, 1971.
- Tauber & Sutton, NASA TM-86767, 1986.
- Anderson, *Hypersonic and High-Temperature Gas Dynamics* (3rd ed.).
- Hirschel, *Basics of Aerothermodynamics* (2nd ed., 2015).
- Park, *Nonequilibrium Hypersonic Aerothermodynamics* (1990).
