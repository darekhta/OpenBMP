# openbmp-vehicle

L2 vehicle composition crate.

**Status:** Phase 2 — `Vehicle` trait + `BasicVehicle` composition +
kernel-side adapter family. The `VehicleAssembly` tree (Bodies /
Propulsion / Effectors / Tanks / Sensors) and rigid-body adapters
land in Phase 3.

## Purpose

- `Vehicle` trait per `docs/software-architecture.md`.
- `BasicVehicle<S>`: ordered force / moment lists + one mass model
  over a single `SimState` type. `evaluate_force_breakdown` returns
  per-model components plus the total for runner-side telemetry.
- `NamedForceModel` / `NamedMomentModel` wrap each model with a
  scenario-declared name for telemetry routing.
- Kernel-side adapter family wrapping L2 physics into point-mass
  `ForceModel` / `MassModel` impls: `GravityForceAdapter`,
  `MotorThrustForceAdapter`, `MotorMassAdapter`,
  `AxialDragForceAdapter`. The rigid-body adapter family lands in
  Phase 3.
- Phase-2.9 Niskanen sounding-rocket integration test exercises the
  adapter stack end-to-end with the Estes C6 motor and a reduced-CD
  aero deck against the published 151.5 m experimental apogee.

## Inputs and Outputs

Composition only — vehicle behaviour is the sum of its registered
providers.

## Units and Frames

Forces in `ECI`, moments in `Body`, mass properties in `Body`.

## Assumptions

Vehicles are compositions of simulator-local models. The crate does not ship
real fielded-vehicle mass properties or operational staging data.

## Validity Range

Each vehicle model declares mass, inertia, geometry, staging, and composition
validity ranges. Invalid mass properties or unsupported stage events fail
closed.

## Determinism

Composition order is scenario-declared and stable. No allocation on
the kernel hot path once Phase 2 closes.

## Validation

`validated-toy` for the Phase-2.8 `BasicVehicle` composition and the
Phase-2.9 adapter stack. Validated against ordered-sum correctness,
first-failing-model short-circuit, declared-order determinism, and
the Niskanen 2009 Chapter-6 sounding-rocket benchmark (apogee within
±5% of the 151.5 m experimental value).

## Data Provenance

Toy vehicles are synthetic. Any public example vehicle data must include
provenance and must not be a real operational parameter set.

## Safety Boundary

Generic vehicle composition only. No real fielded-vehicle parameter
sets ship in this crate; toy academic vehicles only.
