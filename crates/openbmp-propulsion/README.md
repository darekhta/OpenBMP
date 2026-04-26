# openbmp-propulsion

L2 propulsion crate.

**Status:** Phase 2.6 — synthetic solid motor.

## Purpose

- Crate-local `Motor` trait exposing thrust, mass, and mass-rate lookups.
- Variant: `Solid`. Liquid, hybrid, cold-gas, and kernel
  `ForceModel` / `MassModel` adapters are deferred.
- In-house TOML thrust-curve format (RASP `.eng`-shaped).
- Impulse-weighted propellant mass depletion.

## Inputs and Outputs

Time since ignition → thrust, motor mass, and mass-flow derivative.

## Units and Frames

Thrust in newtons. Kernel-side frame transformation lands with the Phase 2.10
adapter. Mass flow is in kg/s.

## Assumptions

Motor curves are synthetic, textbook, or allowed public educational data.
The crate models simulator-local thrust and mass flow only.

## Validity Range

Each motor declares burn time, thrust curve, and nozzle exit geometry. Thrust
and mass rate are zero outside the burn window; mass clamps to the pre-ignition
or post-burn boundary value.

## Determinism

Thrust curves are tabulated and interpolated linearly with locked operand order.

## Validation

Phase 2.6 validation uses synthetic impulse and mass-flow consistency checks.

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
