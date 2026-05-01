# openbmp-physics

HAL-portable physics for OpenBMP.

## Purpose

Owns every physics formula and constant the workspace shares between
the simulator-side environment models and the flight controller.
Built deliberately as a foundation crate that both
`openbmp-fc` (controller) and `openbmp-sim` / `openbmp-cli`
(simulator) consume without crossing the HAL boundary.

This crate replaced and absorbed the former `openbmp-env` crate
(retired 2026-04-30 per `docs/physics-consolidation-plan.md`).

## Module map

```
openbmp-physics/
├── earth        # Earth-radius constants for low-order toy models
├── error        # PhysicsError (out-of-envelope, non-finite, invalid parameter, frame)
├── gravity      # GravityModel trait + ConstantGravity, PointMassGravity, J2Gravity;
│                # WGS84_A_M / WGS84_MU_M3_S2 live here during the
│                # dual-residence migration and are const-checked against
│                # openbmp-core, plus WGS84_J2 + STANDARD_GRAVITY_M_S2
├── atmosphere   # AtmosphereModel trait + AtmosphereSample + ExoatmosphericPolicy
│   ├── isothermal           # IsothermalAtmosphere (toy)
│   └── us_standard_1976    # Full 7-layer USSA76 (geopotential 0–86 km)
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
└── wind         # WindModel + NoWind, ConstantWind, LayeredWind, GustWind (synthetic)
```

## Outstanding follow-up

`openbmp-core::frames` currently owns `FrameContext`, `LocalGeodeticOrigin`,
and the WGS84 constants — all of which are physics primitives that should
live in this crate. Two of the constants (`WGS84_A_M`, `WGS84_MU_M3_S2`)
are duplicated inline in `openbmp-physics::gravity`; the values match
bit-for-bit but are maintained in two places. Moving them, plus
`FrameContext` and the time-aware frame transformations (which depend on
those constants), into `openbmp-physics::frames` is tracked in
`docs/physics-consolidation-plan.md § Outstanding follow-up — frames +
WGS84 constants`. Frame *types* and *value types* (`Frame` trait, markers,
`Position3<F>`, `Velocity3<F>`, `Quaternion<From, To>`) will stay in
`openbmp-core` since they are foundation types and their non-physics math
methods (`norm`, `is_finite`, basic arithmetic, frame-tag-only
operations) don't depend on Earth-specific constants.

## Inputs and outputs

Models take `Position3<Eci>` + `SimTime` (rich `MagneticModel` /
`GravityModel` / `AtmosphereModel` / `WindModel`) or `Vector3<f64>` ECI
+ `SimTime` (simple `MagneticFieldEci`). Output is the model's native
quantity: gravity in m/s² ECI, atmosphere as
`AtmosphereSample { density, pressure, temperature, speed_of_sound }`,
mag in nT, wind in m/s NED.

The two trait families coexist so:
- The simulator-side adapter layer (in `openbmp-vehicle::adapters`
  and `openbmp-cli::runner`) consumes the rich traits with full
  envelope error handling.
- The FC's estimators (in `openbmp-fc::estimator`) consume the
  simpler ECI traits without dragging in geodetic-conversion
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
`(scenario_seed, step, channel_id)`.

## Dependencies

`openbmp-physics` depends only on `openbmp-core` (foundation types:
`SimTime`, `Position3`, `Eci`, `Ned`, `Velocity3`, `FrameContext`,
WGS84 constants, `DeterministicRng`) and `nalgebra`. **No simulator
trait surfaces or scenario parsing live here.** Both `openbmp-fc`
(controller) and `openbmp-sim` / `openbmp-cli` (simulator-side)
consume this crate directly.

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
