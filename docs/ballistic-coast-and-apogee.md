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
for academic trajectory study and range-safety / recovery planning.

## Current state

- Gravity to EGM2008 zonal degree 6 and an `AtmosphereModel` to 1000 km exist
  and are validated.
- The kernel propagates a coasting body correctly today; what is missing is
  the *mission-phase* structure (`coast`, `apogee_approach`,
  `ballistic_descent`) and the apogee/entry-interface handoffs.
- There is no tool that summarizes *where the trajectory lands* — the lint
  forbids `impact-point`, and no range-relative equivalent exists.

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

> **Status.** `apogee_approach` and `ballistic_descent` are *reserved* phase
> names (see [`profile-vocabulary-and-guardrails.md`](profile-vocabulary-and-guardrails.md)).
> The triggers (`at_apogee`, `at_altitude_descending`) already ship. Wiring is
> a configuration over existing seams; no kernel change is required for the
> coast arc itself.

### Range-safety footprint

A flight-profile study wants to know *where the body lands* — for recovery
planning (where do we send the boat) and range-safety analysis (does the
predicted descent stay inside the cleared range). This is a standard,
non-operational astrodynamics output, and OpenBMP frames it carefully to keep
it on the safe side of the line.

The new trait reports a landing prediction in **range-relative coordinates**
— downrange / crossrange distance and bearing from the launch point, plus
optional WGS84-geodetic latitude/longitude **only when the scenario declares a
launch-site origin** (the existing `LocalGeodeticOrigin`). It never accepts a
desired landing location and never computes a steering command:

```rust
/// Range-safety / recovery landing-footprint estimate for an
/// unpowered body. Reports where a body is predicted to come down,
/// in range-relative coordinates. It accepts no desired landing
/// location and produces no guidance command. (`profile` module.)
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
- **Dispersion ellipse** — the 1-σ / 3-σ landing scatter from a Monte-Carlo
  sweep over declared input uncertainties (winds, ballistic coefficient,
  burnout state). This is the recovery / range-safety footprint, reported as a
  statistical ellipse, not an accuracy claim against any aimpoint.

```
        crossrange
            ▲
            │     ╭───────╮      ← 3-σ dispersion ellipse
            │    (   •     )       • = nominal landing point
   launch ──┼─────╰───────╯──────────► downrange
    origin
```

> **Naming.** The output is a **landing footprint** / **dispersion ellipse**,
> not an "impact point." The term `impact-point` is rejected by the lint; the
> range-safety vocabulary (`landing_footprint`, `dispersion_ellipse`,
> `downrange_m`, `crossrange_m`) is the accepted academic form. See
> [`profile-vocabulary-and-guardrails.md`](profile-vocabulary-and-guardrails.md).

The footprint tool is **post-processing**: it runs over telemetry or a forward
propagation after the fact, in the offline analysis path. It is not on the
flight-controller's control loop, and producing it never feeds back into any
actuator command. This is the same boundary the platform draws for telemetry
viewers — *offline analysis only*.

### Why this is not targeting

The footprint answers "given this trajectory, where does the body land?" — the
forward question. Targeting answers the inverse: "given a desired landing
place, what trajectory/steering gets there?" OpenBMP implements only the
forward question, accepts no desired-landing input, and emits no steering. The
inverse problem and any geographic aimpoint are rejected at load.

## Fail-closed validation

- A `coast` or `ballistic_descent` phase whose apogee exceeds the selected
  gravity model's documented validity envelope is rejected
  (`ScenarioError::InconsistentSection`).
- The footprint's geodetic output requires a declared launch-site
  `LocalGeodeticOrigin`; absent one, only range-relative output is produced —
  never a fabricated geographic coordinate.
- Any footprint-config field naming a *desired* landing location, aimpoint, or
  miss-distance is rejected by the lint.
- Monte-Carlo dispersion requires declared input-uncertainty blocks; a
  dispersion request with no uncertainty source fails closed rather than
  reporting a degenerate zero-width ellipse.

## Schema stub summary

| Item | Location | State |
|---|---|---|
| `RangeSafetyFootprint` trait + `LandingFootprint` / `BallisticState` | `openbmp-physics/src/profile.rs` | Trait + struct signatures. |
| `apogee_approach`, `ballistic_descent` phases | vocabulary canon | Reserved. |
| Footprint post-processing path | offline analysis (`openbmp-runner`) | Deferred; trait stub only. |

## References

- Vallado, D. A. *Fundamentals of Astrodynamics and Applications*, 4th ed.,
  2013, Ch. 8 — ballistic-arc propagation, J2 secular effects.
- Montenbruck, O., Gill, E. *Satellite Orbits*, Springer, 2000, Ch. 3 —
  perturbed two-body propagation for long coast arcs.
- Range Commanders Council, *Common Risk Criteria Standards for National Test
  Ranges* (RCC 321), public standard — the range-safety footprint / debris-
  dispersion framing this tool adopts.
- Knacke, T. W. *Parachute Recovery Systems Design Manual*, 1992 — recovery
  landing-dispersion conventions.
