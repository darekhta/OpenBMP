# OpenBMP Ballistic Coast and Apogee

This document specifies the **exo-atmospheric coast** phases of a flight
profile — `coast`, `apogee_approach`, and the start of `ballistic_descent` —
and the **range-safety landing footprint** tool that reports where a body (a
spent stage, an inert test article, or the primary vehicle) is predicted to
come down. It is a companion to
[`flight-profiles-architecture.md`](flight-profiles-architecture.md).

These phases are unpowered ballistic propagation under gravity and (near the
ends) atmospheric drag. OpenBMP already has the propagators and gravity fields;
the work here is phase wiring and a post-processing analysis tool, both framed
for trajectory study and recovery planning.

## Current state

- Gravity to EGM2008 zonal degree 6 and an `AtmosphereModel` to 1000 km exist
  and are validated.
- The kernel propagates a coasting body correctly today, and scenarios can
  name the `coast` and `ballistic_descent` phases with the existing
  `at_apogee` / `at_altitude_descending` event machinery.
- Schema v3 now includes an offline `[landing_footprint]` block. It supports
  a deterministic closed-form `constant_gravity` method for short constant-
  gravity profiles plus fixed-step numerical J2 and zonal-only EGM2008
  footprint propagation for longer coast / descent profiles.
- `[landing_footprint.monte_carlo]` adds offline sampled dispersion over
  declared wind, ballistic-coefficient, and burnout-state uncertainty sources.
  The sampled path is drag/wind-aware and deterministic for the same seed.

## Design

### Coast phase wiring

`coast` begins at burnout (or at the final `jettison_stage`) and runs until
apogee. It needs no controller; the autopilot trajectory loop is idle (or holds
a passive attitude for spin stabilization). The phase is entered by the
existing event machinery:

```toml
[[mission.events]]
id = "evt_burnout"
trigger = { kind = "at_mass_fraction", remaining = 0.02 }
action  = { kind = "enter_phase", phase = "coast" }
```

Propagation during coast should select a gravity model appropriate to the
altitude: `point_mass` is adequate for short sounding-rocket arcs; `j2` or
`egm2008` for high, long arcs where oblateness matters. This is the existing
`environment.gravity` selection; the only new requirement is that the
**consistency check** confirm the chosen gravity model's validity envelope
covers the profile's apogee. A `coast` to 800 km under `constant` gravity is
rejected at load as a scenario-consumer disagreement.

### Apogee detection

Apogee is the existing `at_apogee` trigger (velocity radial-component
sign-flip). It transitions `coast → apogee_approach → ballistic_descent`. No
new trigger variant is needed; the canon simply assigns the reserved phase
names to the arcs on either side of the `at_apogee` fire.

```
  v_radial > 0        v_radial = 0        v_radial < 0
 ───coast──────► apogee_approach ─┤├─ ballistic_descent ───►
                                at_apogee
```

### Descent into the atmosphere

`ballistic_descent` runs from apogee down to the entry interface (conventionally
≈ 122 km, the altitude `EntryInterfaceBuilder` already uses). Crossing it is the
existing `at_altitude_descending` trigger, which hands off to the entry phases
documented in
[`descent-and-entry-profiles.md`](descent-and-entry-profiles.md).

> **Status.** `coast` and `ballistic_descent` are consumed phase names for the
> PR3 footprint slice. The triggers (`at_apogee`,
> `at_altitude_descending`) already ship. `entry_interface` and later entry
> phases remain reserved for the descent/entry slice.

### Landing Footprint

A flight-profile study wants to know *where the body lands* — for recovery
planning and dispersion analysis.

The new trait reports a landing prediction in **range-relative coordinates**
— downrange / crossrange distance and bearing from the launch point, plus
optional WGS84-geodetic latitude/longitude **only when the scenario declares a
launch-site origin** (the existing `LocalGeodeticOrigin`):

```rust
/// Recovery landing-footprint estimate for an
/// unpowered body. Reports where a body is predicted to come down,
/// in range-relative coordinates. (`profile` module.)
pub trait RangeSafetyFootprint {
    /// Predicted nominal landing point and dispersion for a body
    /// propagated from `state` to ground (or cull altitude).
    fn landing_footprint(
        &self,
        state: &BallisticState,
        env: &FootprintEnvironment,
    ) -> Result<LandingFootprint, PhysicsError>;
}
```

`LandingFootprint` carries:

- **Nominal landing point** — downrange distance, crossrange distance, and
  bearing from the launch origin (range-relative; geodetic lat/lon only if an
  origin is declared, for recovery mapping).
- **Dispersion ellipse** — optional 1-σ / 3-σ landing scatter. This can be a
  declared input ellipse or the covariance ellipse computed from a
  `[landing_footprint.monte_carlo]` sample cloud over declared wind,
  ballistic-coefficient, and burnout-state uncertainty sources.
- **Dispersion diagnostics** — the Monte-Carlo path emits output-only
  `radial_dispersion_p50_m` about the sample mean plus nominal-referenced radial-offset
  statistics. These are comparisons to the nominal forward footprint, not to a
  desired landing point.

```
        crossrange
            ▲
            │     ╭───────╮      ← 3-σ dispersion ellipse
            │    (   •     )       • = nominal landing point
   launch ──┼─────╰───────╯──────────► downrange
    origin
```

The footprint tool is **post-processing**: it runs over telemetry or a forward
propagation after the fact, in the offline analysis path. It is not on the
flight-controller's control loop, and producing it never feeds back into any
actuator command. This is the same boundary the platform draws for telemetry
viewers — *offline analysis only*.

## Fail-closed validation

- A `coast` or `ballistic_descent` phase whose apogee exceeds the selected
  gravity model's documented validity envelope is rejected
  (`ScenarioError::InconsistentSection`).
- The footprint's geodetic output requires a declared launch-site
  `LocalGeodeticOrigin`; absent one, only range-relative output is produced —
  never a fabricated geographic coordinate.
- Monte-Carlo dispersion requires declared input-uncertainty blocks under
  `wind`, `ballistic_coefficient`, or `burnout_state`; a dispersion request
  with no uncertainty source fails closed rather than reporting a degenerate
  zero-width ellipse. Dispersion diagnostics are computed only from the
  resulting forward sample cloud and nominal footprint. Persisted sample-cloud
  files omit the sampled burnout state, ballistic coefficient, and wind vector
  so the artifact is a landing-dispersion cloud, not an input-state table.
- `landing_footprint.method` must agree with the scenario gravity selector:
  `constant_gravity` requires `environment.gravity = "constant"`, `j2`
  requires `environment.gravity = "j2"`, and `egm2008` requires
  `environment.gravity = "egm2008"`.
- Every footprint method requires a mission phase named `coast` or
  `ballistic_descent`.

## Schema stub summary

| Item | Location | State |
|---|---|---|
| `RangeSafetyFootprint` trait + `LandingFootprint` / `BallisticState` | `openbmp-physics/src/profile.rs` | Consumed by `ConstantGravityRangeSafetyFootprint` and `NumericalGravityRangeSafetyFootprint`. |
| `coast`, `ballistic_descent` phases | mission vocabulary | Accepted with existing event machinery. |
| Footprint post-processing path | offline analysis (`openbmp-runner`) | `landing_footprint_for_state` consumes schema-v3 `[landing_footprint]` for constant-gravity, J2, and EGM2008 methods. |
| Footprint Monte-Carlo dispersion diagnostics | offline analysis (`openbmp-runner`) | `footprint-mc` writes `radial_dispersion_p50_m`, nominal-referenced radial-offset quantiles, and sample-cloud radial offsets from the declared sample set. |

## References

- Vallado, D. A. *Fundamentals of Astrodynamics and Applications*, 4th ed.,
  2013, Ch. 8 — ballistic-arc propagation, J2 secular effects.
- Montenbruck, O., Gill, E. *Satellite Orbits*, Springer, 2000, Ch. 3 —
  perturbed two-body propagation for long coast arcs.
- Range Commanders Council, *Common Risk Criteria Standards for National Test
  Ranges* (RCC 321), public standard — the range-safety footprint / debris-
  dispersion framing this tool adopts.
- JPL / NASA NTRS, *Calculating the CEP* (JPL D-4710 / NASA-CR-182996),
  1987 — circular-radius probability checks against elliptical probability.
- Knacke, T. W. *Parachute Recovery Systems Design Manual*, 1992 — recovery
  landing-dispersion conventions.
