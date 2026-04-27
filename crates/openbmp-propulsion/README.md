# openbmp-propulsion

L2 propulsion crate.

**Status:** Phase 2 — synthetic solid motor + public Estes hobby
motor files (B4, C6, D12) shipped as `data/motors/*.toml`.
Liquid / hybrid / cold-gas variants land in Phase 3 alongside
`EngineModel` and `EngineCluster`.

## Purpose

- Crate-local `Motor` trait exposing thrust, mass, and mass-rate
  lookups.
- Variant: `Solid`. Liquid, hybrid, and cold-gas variants are
  deferred to Phase 3.
- In-house TOML thrust-curve format (RASP `.eng`-shaped) with strict
  `serde(deny_unknown_fields)` schema-1 parser.
- Impulse-weighted propellant mass depletion.
- Kernel-side adapters (`MotorThrustForceAdapter`,
  `MotorMassAdapter`) live in `openbmp-vehicle`.

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

`validated-toy` for the shipped synthetic motors (`textbook`,
`d-class`) and scenario-backed Estes C6/D12 cases; `checked` for
the standalone B4 import. The Phase-2.6 regression suite asserts
integrated impulse vs. declared total to 1e-12 relative,
mass-at-burnout bit-equality with `dry_mass`, mass-rate ≤ 0
everywhere, monotone mass decrease, thrust-at-grid-corner
exact-equality, thrust-outside-window zero, and bit-stable lookups
across two evaluations for the parser-level decks it covers.

## Data Provenance

Synthetic textbook motors plus public Estes hobby motor files (B4,
C6, D12) derived from ThrustCurve.org RASP data, each with a
SHA-256-pinned source digest in
`data/motors/provenance.md`. **Real fielded operational motor data
is categorically rejected.**

## Safety Boundary

No operational motor data, no real fielded engine curves, and no propulsion
configuration intended for payload delivery or weapon employment.

## References

- RASP `.eng` motor format — public amateur-rocketry standard,
  re-implemented in-house.
- OpenRocket / Niskanen 2009 — version-controlled motor database
  pattern.
