# openbmp-core

L0 foundation crate. Math, units, coordinate frames, simulation time,
deterministic RNG, project-wide error and validation primitives.

**Status:** Implemented foundation crate. The type surface includes
append-only typed ids (`EffectorId`, `TankId`,
`EngineId`, `RecoveryId`, `WindAxis`) plus the
`DeterministicRng::for_effector_component` and
`DeterministicRng::for_wind_component` domain-separated streams.
The `SensorId` /
`DeterministicRng::for_sensor_component` stream provides the sensor
RNG surface. `DeterministicRng::for_mc_sample` provides the campaign-layer
Monte Carlo sample stream using the `b"MCRN"` domain tag. The foundation
surface covers frames, units, time, and validation labels.

## Purpose

Provides the foundational vocabulary used by every other OpenBMP crate:

- `SimTime`, `Duration`, `StepIndex` newtypes.
- Frame tag types (`ECI`, `ECEF`, `NED`, `ENU`, `Body`) and parametric
  `Position3<F>`, `Displacement3<F>`, `Velocity3<F>`,
  `Acceleration3<F>`, `AngularVelocity3<F>`, `Quaternion<From, To>`.
- Typed frame values and quaternion rotations only; time-aware
  `FrameContext` / `FrameTransform` implementations live in
  `openbmp-physics::frames`.
- `uom`-typed quantity re-exports.
- `DeterministicRng` wrapping `rand_chacha::ChaCha8Rng`.
- `ChannelId`, `ModelId`, `ScenarioId` newtypes.
- `ValidationStatus` enum.
- Project-wide error types.
- Re-exported `nalgebra` primitives (`Vector3`, `Matrix3`,
  `UnitQuaternion`) used by downstream state and model crates.

## Inputs and Outputs

This crate exposes only types and pure functions. No IO, no state.

## Units and Frames

`uom` quantities at boundaries; `f64` internally. Frame tags are
compile-time-encoded so frame mismatches fail at the type system.

## Assumptions

- Single-threaded, synchronous, deterministic.
- No wall-clock time, no system RNG, no network access, no thread
  primitives.

## Validity Range

Frame transforms valid within the chosen Earth model and frame
profile. See `docs/frames-time.md`.

## Determinism

This crate **defines** the determinism contract for the rest of
OpenBMP. `FpEnvironment` reads and validates the host floating-point
control register on supported architectures before deterministic kernels
run. See `docs/software-architecture.md` § Determinism Profile.

## Validation

`checked` for the foundation surface: unit and property tests cover
time monotonicity, frame identity transforms, typed frame arithmetic, quaternion
round trips and composition, deterministic RNG replay, and validation labels.

## Data Provenance

This crate ships no data files.

## Scope Boundary

Foundational; no operational vocabulary, no real-vehicle data.
See `docs/safety-boundaries.md`.

## References

- nalgebra 0.34, uom 0.38, rand_chacha 0.9.
- Vallado, *Fundamentals of Astrodynamics and Applications* (4th ed.,
  Microcosm Press, 2013).
- IEEE 1139-2008.
- `docs/frames-time.md`, `docs/glossary.md`.
