# openbmp-vehicle

L2 vehicle composition crate.

**Status:** Implemented. Provides the `Vehicle` / `KernelVehicle`
foundation plus the full modular composable rocket surface:
rigid-body implementations on the shared kernel adapter family, a
`VehicleAssembly` tree (Bodies / Effectors / Engines / Tanks /
Recovery), `EngineModel` + `EngineCluster`, `ControlEffector` with
rate / position / latency / deadband limits and the four canonical
fault modes, `Tank` + `MovingMassModel` (rigid-liquid,
equivalent-pendulum, equivalent-spring-mass, baffled-pendulum), and
recovery devices (`ParachuteDrag`, `DrogueMainRecovery`,
`DragDevice`).

## Purpose

- `Vehicle` trait per `docs/software-architecture.md`.
- `KernelVehicle<S>`: ordered force / moment lists + one mass model
  over a single `SimState` type. `evaluate_force_breakdown` returns
  per-model components plus the total for runner-side telemetry.
- `NamedForceModel` / `NamedMomentModel` wrap each model with a
  scenario-declared name for telemetry routing.
- Kernel-side adapter family wrapping L2 physics into point-mass
  `ForceModel` / `MassModel` impls: `GravityForceAdapter`,
  `MotorThrustForceAdapter`, `MotorMassAdapter`,
  `DeckDragForceAdapter`. The family includes rigid-body impls on those
  shared adapters plus `RigidMotorMassAdapter`,
  `EngineClusterForceAdapter`, `EngineClusterMomentAdapter`,
  `EngineClusterMassAdapter`, `TankRackForceAdapter`,
  `TankRackMomentAdapter`, `TankRackMassAdapter`, and
  `RecoveryRackForceAdapter`.
- A Niskanen sounding-rocket integration test exercises the
  adapter stack end-to-end with the Estes C6 motor and a reduced-CD
  aero deck against the published 151.5 m experimental apogee.
- A RocketPy Calisto cross-tool case exercises the full
  rigid-body stack (assembly tree → engine cluster → drogue + main
  recovery → mission events → telemetry) end-to-end against
  RocketPy's published 3349 m AGL apogee within an audited 2 %
  cross-tool envelope.

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
the kernel hot path.

## Validation

`validated-toy` for the `KernelVehicle` composition and the
adapter stack. Validated against ordered-sum correctness,
first-failing-model short-circuit, declared-order determinism, and
the Niskanen 2009 Chapter-6 sounding-rocket benchmark (apogee within
±5% of the 151.5 m experimental value). The
RocketPy Calisto cross-tool e2e covers the full assembly tree +
engine cluster + drogue/main recovery hot path within an audited
2 % apogee envelope.

## Data Provenance

Toy vehicles are synthetic. Any public example vehicle data must include
provenance and must not be a real operational parameter set.

## Scope Boundary

Generic vehicle composition only. No real fielded-vehicle parameter
sets ship in this crate; toy academic vehicles only.
