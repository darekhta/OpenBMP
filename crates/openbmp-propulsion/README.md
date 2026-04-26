# openbmp-propulsion

L2 propulsion crate.

**Status:** Phase 2 — stub.

## Purpose

- `Motor` trait combining `ForceModel` + `MassModel`.
- Variants: `Solid`, `Liquid`, `Hybrid`, `ColdGas`.
- In-house TOML thrust-curve format (RASP `.eng`-shaped).
- `MultiStageMotor` for staged academic launchers.

## Inputs and Outputs

Time → thrust force (`ECI`) and mass-flow derivative.

## Units and Frames

Thrust in `Body`, transformed to `ECI` via vehicle attitude. Mass flow
in kg/s.

## Assumptions

Motor curves are synthetic, textbook, or allowed public educational data.
The crate models simulator-local thrust and mass flow only.

## Validity Range

Each motor declares burn-time, thrust-curve, chamber/exit, and interpolation
validity ranges. Samples outside the declared range fail closed unless an
explicit shutdown or clamp rule is documented.

## Determinism

Thrust curves are tabulated and interpolated linearly with documented
out-of-range behaviour (fail-closed unless extrapolation declared).

## Validation

`experimental` (stub). Phase 2 validation starts with synthetic impulse and
mass-flow consistency checks.

## Data Provenance

Synthetic textbook motors only (see
`docs/data-provenance.md` source classes). **Real fielded operational
motor data is categorically rejected.**

## Safety Boundary

No operational motor data, no real fielded engine curves, and no propulsion
configuration intended for payload delivery or weapon employment.

## References

- RASP `.eng` motor format — public amateur-rocketry standard,
  re-implemented in-house.
- OpenRocket / Niskanen 2009 — version-controlled motor database
  pattern.
