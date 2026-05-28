# openbmp-physics

HAL-portable physics for OpenBMP.

## Purpose

Owns every physics formula and constant the workspace shares between
the simulator-side environment models and the flight controller.
Built deliberately as a foundation crate that both
`openbmp-fc` (controller) and `openbmp-sim` / `openbmp-cli`
(simulator) consume without crossing the HAL boundary.

This crate is the single home for the workspace's shared physics
models, consumed by both the controller and simulator sides.

## Module map

```
openbmp-physics/
├── earth        # Earth-radius constants for low-order toy models
├── error        # PhysicsError (out-of-envelope, non-finite, invalid parameter, frame)
├── frames       # FrameProfile, LocalGeodeticOrigin, FrameContext,
│                # FrameTransform trait + impls, and the WGS84 ellipsoid
│                # constants (WGS84_A_M, WGS84_INV_FLATTENING,
│                # WGS84_FLATTENING, WGS84_ECCENTRICITY_SQUARED,
│                # WGS84_MU_M3_S2, WGS84_OMEGA_RAD_S)
├── gravity      # GravityModel trait + ConstantGravity, PointMassGravity, J2Gravity;
│                # WGS84_J2 + STANDARD_GRAVITY_M_S2 (constants for which
│                # this crate is the only home — frame-shared WGS84
│                # constants live in `frames` above)
├── atmosphere   # AtmosphereModel trait + AtmosphereSample + ExoatmosphericPolicy
│   ├── isothermal           # IsothermalAtmosphere (toy)
│   ├── us_standard_1976     # Full 7-layer USSA76 (geopotential 0–86 km)
│   ├── nrlmsise00           # Direct NRLMSISE-00 coefficient evaluator
│   └── nrlmsis2_compat      # NRLMSIS 2.x compatibility profile
│                #
│                # plus closed-form helpers (geopotential ↔ geometric,
│                # pressure_altitude_troposphere_m, dynamic_pressure_pa)
│                # and USSA76_* constants including USSA76_SEA_LEVEL_DENSITY_KG_M3
├── kinematics   # quaternion_from_axis_angle / _omega, renormalize_quaternion,
│                # skew_symmetric — rigid-body math primitives shared between
│                # FC estimators and sim-side propagators
├── magnetic     # MagneticModel (NED) + MagneticFieldEci (ECI) traits
│   ├── dipole              # EarthDipoleField — degree-1 academic placeholder
│   └── wmm2025             # Wmm2025 — NOAA / NCEI 2025 12-degree spherical harmonic
├── statistics   # chi_square_inverse_cdf_wilson_hilferty,
│                # inverse_standard_normal_cdf — innovation-gate primitives
│                # shared between FC estimators and FDIR detectors
├── validity     # HalfOpenRange — model-envelope validity helpers
└── wind         # WindModel + NoWind, ConstantWind, LayeredWind, Hwm14Wind, GustWind
```

## Layering with `openbmp-core`

`openbmp-core::frames` owns the *type-level* frame machinery only:
the `Frame` trait, frame tag types (`Eci`, `Ecef`, `Ned`, `Enu`,
`Body`), value types (`Position3<F>`, `Velocity3<F>`,
`Acceleration3<F>`, `Displacement3<F>`, `VelocityDelta3<F>`,
`AngularVelocity3<F>`), `Quaternion<From, To>` rotation, and
`FrameError`. None of those depend on Earth-specific physics
constants.

`openbmp-physics::frames` owns the physics on top: the WGS84
ellipsoid constants, `LocalGeodeticOrigin`, `FrameContext`, the
time-aware ECI ↔ ECEF transforms, the ECEF ↔ NED helpers anchored at
the local origin, and the `FrameTransform` trait + impls.

## Inputs and outputs

`GravityModel` and rich `MagneticModel` take `Position3<Eci>` +
`SimTime`; `AtmosphereModel` takes geometric altitude + `SimTime`;
`WindModel` takes `Position3<Eci>` + `&FrameContext` + `SimTime`; and
the simple `MagneticFieldEci` takes `Vector3<f64>` ECI + `SimTime`.
Output is the model's native quantity: gravity in m/s² ECI, atmosphere as
`AtmosphereSample { density, pressure, temperature, speed_of_sound,
dynamic_viscosity }` with viscosity from Sutherland's law, mag in nT,
wind in m/s NED.

The two trait families coexist so:
- The simulator-side adapter layer (in `openbmp-vehicle::adapters`
  and `openbmp-runner`) consumes the rich traits with full
  envelope error handling.
- The FC's estimators (in `openbmp-fc::estimator`) consume the
  simpler ECI trait without dragging in geodetic-conversion
  machinery.

## Validity ranges

| Model | Altitude | Time | Notes |
|---|---|---|---|
| `ConstantGravity` | unbounded | unbounded | constant ECI vector |
| `PointMassGravity` | r > 0 | unbounded | singular at origin |
| `J2Gravity` | r > 0 | unbounded | WGS84 default |
| `IsothermalAtmosphere` | unbounded | unbounded | constant ρ, p, T |
| `UsStandard1976` | 0 .. 86 km geometric | unbounded | clamps or errors above ceiling per `ExoatmosphericPolicy` |
| `EarthDipoleField` | unbounded (clamped < 0.5 R_e) | ignored | degree-1 toy |
| `Wmm2025` | unbounded | 2025.0 .. 2030.0 decimal years | NOAA / NCEI / NGA / UK DGC, December 2024 release |
| `NoWind` / `ConstantWind` / `LayeredWind` | unbounded | unbounded | |
| `GustWind` | unbounded | unbounded | seeded RNG; deterministic by `(scenario_seed, step, axis)` |

## Determinism

Pure `f64` arithmetic with locked operand order on every model; no
FMA, no wall-clock time, no system RNG, no network, no file I/O. The
optional `GustWind` model uses
`openbmp_core::DeterministicRng` whose seed is derived from
`(scenario_seed, step, axis)`.

## Dependencies

`openbmp-physics` depends only on `openbmp-core` (foundation types:
`SimTime`, `Position3`, `Eci`, `Ned`, `Velocity3`,
`DeterministicRng`, and typed frame machinery) and `nalgebra`. This
crate itself owns `FrameContext` and the WGS84 constants in
`openbmp-physics::frames`. **No simulator trait surfaces or scenario
parsing live here.** Both `openbmp-fc` (controller) and `openbmp-sim`
/ `openbmp-cli` (simulator-side) consume this crate directly.

## Features

- `default` — enables `synthetic`.
- `synthetic` — enables `GustWind` (Dryden filter). Hardware adopters
  on the FC side can disable to compile down to the deterministic-only
  surface.

## References

- WGS84 Implementation Manual, NIMA TR8350.2, §3.
- US Standard Atmosphere 1976, NOAA-S/T 76-1562 / NASA-TM-X-74335.
- World Magnetic Model 2025, NOAA NCEI / NGA / UK DGC, December 2024
  release; coefficient table at `data/magnetic/WMM.COF`.
- Vallado, *Fundamentals of Astrodynamics and Applications* (4th ed.),
  §8.6 (Cartesian J2 acceleration).
- Montenbruck & Gill, *Satellite Orbits*, §3.2 (J2 perturbation).
- Bristow & Jacques 1999 (Gauss-Markov bias-dynamics deferred from FC).
