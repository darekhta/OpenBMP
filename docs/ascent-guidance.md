# OpenBMP Ascent Guidance

This document specifies **powered-ascent reference-trajectory generation** for
the `powered_ascent` phase: the gravity-turn, pitch-program, and
explicit-guidance methods that shape a launch vehicle's climb. It is a
companion to
[`flight-profiles-architecture.md`](flight-profiles-architecture.md).

The scope boundary is precise and load-bearing for the platform's posture: the
methods here **generate a reference trajectory or reference attitude** that the
existing three-loop autopilot then tracks. They do not solve for, or steer
toward, any real-world location. This is the same boundary the platform
already draws — `openbmp-fc/src/guidance.rs` ships waypoint navigation in
inertial space and *"no proportional navigation, no terminal-homing, no
targeting, no real-world-location guidance."* Ascent guidance extends the
*reference-generation* side of that line only.

## Current state

OpenBMP ships the first consumed ascent-reference path:

- `pitch_program` — tabulated pitch angle versus time, linearly interpolated.
- `gravity_turn` — body `+x` aligned with the inertial velocity vector once
  speed is non-zero.

The explicit reference family remains reserved. The three-loop autopilot
(`openbmp-fc/src/autopilot.rs`, Stevens & Lewis 2015) is unchanged; the ascent
reference job publishes the reference quaternion it already tracks.

## Design

### The trait: `AscentReferenceGenerator`

A new trait in `openbmp-physics::profile` produces, at each guidance tick, the
reference attitude (and optionally reference rate) the trajectory loop should
track during `powered_ascent`. It consumes only the vehicle's own estimated
state and inertial/atmospheric quantities — never a target.

```rust
/// Generates a powered-ascent attitude reference. The output is a
/// reference the three-loop autopilot tracks; it is not a guidance
/// solution to any location. (`profile` module, openbmp-physics.)
pub trait AscentReferenceGenerator {
    /// Reference body-to-ECI attitude for the current ascent state.
    fn ascent_reference(
        &self,
        state: &AscentState,
        time: SimTime,
    ) -> Result<AscentReference, PhysicsError>;
}
```

`AscentState` carries inertial position/velocity, flight-path angle, dynamic
pressure, and mass fraction — all already on the simulation bus. `AscentReference`
carries the reference quaternion and an optional reference angular rate.

### Method 1 — Pitch program (open-loop)

The simplest academic ascent law: a tabulated pitch angle versus time or versus
altitude, linearly interpolated. This is the open-loop "tip-over" schedule used
in sounding-rocket and early-launch pedagogy.

```toml
[fc.ascent_reference]
method = "pitch_program"
# pitch (from vertical) vs time; reference only, tracked by the autopilot
schedule_s   = [0.0, 10.0, 30.0, 60.0]
pitch_rad    = [0.0, 0.05, 0.20, 0.55]
```

### Method 2 — Gravity turn (zero-lift)

The classic gravity-turn: after a programmed pitch-over kick, the reference
attitude is aligned with the inertial velocity vector so that aerodynamic
angle of attack stays near zero and gravity itself bends the trajectory. The
reference quaternion is constructed to point the body x-axis along **v**; no
target enters the computation.

```
pitch-over kick ──► align body +x with velocity ──► gravity bends arc
        (open-loop)            (closed on own velocity)
```

This is the textbook launch-vehicle ascent (Tewari, *Atmospheric and Space
Flight Dynamics*, Ch. 11) and is what an academic study of a sounding rocket's
climb wants.

### Method 3 — Explicit reference (PEG-style, reserved)

A linear-tangent / explicit reference generator (the public PEG family;
Brown, Johnson & Mumford, *Powered Explicit Guidance*, NASA, 1970s; McHenry et
al., Space Shuttle ascent) computes a reference attitude that drives the
vehicle toward a *cutoff condition expressed as inertial state* — a target
altitude, speed, and flight-path angle for engine cutoff. Crucially, the
cutoff condition is an **orbital/energy state**, not a ground location: "reach
this speed and flight-path angle at this altitude," which is how every launch
vehicle reaches orbit insertion.

> **Safety boundary.** The explicit reference accepts only an
> **inertial cutoff state** (altitude, inertial speed, flight-path angle). It
> rejects any geographic coordinate. The lint additions in
> [`profile-vocabulary-and-guardrails.md`](profile-vocabulary-and-guardrails.md)
> forbid aimpoint-shaped fields here. This keeps the method on the
> reach-an-orbit-state side of the line and off the strike-a-place side.

> **Status.** `AscentReferenceGenerator`, `PitchProgramAscentReference`, and
> `GravityTurnAscentReference` are implemented in
> `openbmp-physics::profile`. `FcGuidanceKind::AscentReference`,
> `FcAscentReferenceConfig`, the FC `AscentReferenceGuidance` job, and runner
> wiring are consumed under schema v3. `explicit_reference` still parses only
> as a reserved method and fails closed with
> `ScenarioError::ElementNotYetSupported`.

### Wiring into the autopilot

The generator sits *upstream* of the existing trajectory loop. At each tick:

```
AscentReferenceGenerator ──► reference quaternion
                                   │
                                   ▼
        three-loop autopilot: trajectory → attitude → rate loops
                                   │
                                   ▼
                          actuator deflections / gimbal
```

The autopilot itself is unchanged: it already accepts a reference quaternion.
Ascent guidance only supplies where, previously, a waypoint or hand-authored
reference did. The scenario selector activates it:

```toml
[fc]
guidance = "ascent_reference"

[fc.ascent_reference]
method = "gravity_turn"
```

Phase-gating uses the existing gain schedule: the `powered_ascent` phase
selects the ascent reference; at burnout, a phase transition hands off to
coast. The reserved `select_guidance_profile` action can add an explicit
selector later — see
[`flight-profiles-architecture.md`](flight-profiles-architecture.md) and
[`ballistic-coast-and-apogee.md`](ballistic-coast-and-apogee.md).

## Fail-closed validation

- `guidance = "ascent_reference"` requires an `[fc.ascent_reference]` block,
  mirroring how `attitude_hold` requires `reference_q_xyzw` — else
  `ScenarioError::InvalidFc`.
- `pitch_program` requires `schedule_s` and `pitch_rad` of equal length ≥ 2,
  strictly increasing `schedule_s`; reuses `require_*` helpers.
- `gravity_turn` rejects pitch-program fields and fails at runtime if inertial
  speed is still effectively zero.
- The explicit-reference cutoff block accepts only inertial-state fields; any
  field whose name normalizes to an aimpoint / geographic term is rejected by
  the lint before deserialization.
- `ascent_reference` guidance requires a vehicle that can be attitude-
  controlled (`rigid_body`); a `point_mass` vehicle is rejected.
- `ascent_reference` guidance requires a declared `powered_ascent` mission
  phase/state and a matching gain-schedule entry.

## Implementation summary

| Item | Location | State |
|---|---|---|
| `AscentReferenceGenerator` trait + `AscentState` / `AscentReference` | `openbmp-physics/src/profile.rs` | Consumed API. |
| `PitchProgramAscentReference` / `GravityTurnAscentReference` | `openbmp-physics/src/profile.rs` | Implemented with unit tests. |
| `FcAscentReferenceConfig` + `FcConfig::ascent_reference` field | `openbmp-scenario/src/document.rs` | Consumed under schema v3; validates fail-closed. |
| `FcGuidanceKind::AscentReference` | `openbmp-scenario/src/document.rs` | Consumed under schema v3. |
| `AscentReferenceGuidance` | `openbmp-fc/src/guidance.rs` | Publishes reference states during `powered_ascent`. |
| Runner wiring | `openbmp-runner/src/fc.rs` | Builds physics generators from scenario config. |

## References

- Tewari, A. *Atmospheric and Space Flight Dynamics*, Birkhäuser, 2007,
  Ch. 11 — gravity turn and pitch programs.
- Brown, K. R., Johnson, G. W., et al. *Powered Explicit Guidance*, and
  McHenry, R. L., et al. "Space Shuttle Ascent Guidance, Navigation, and
  Control," *J. Astronautical Sciences*, 1979 — the public PEG / linear-tangent
  family used here for the reserved explicit reference.
- Stevens, B. L., Lewis, F. L. *Aircraft Control and Simulation*, 3rd ed.,
  Wiley, 2015 — the three-loop autopilot the reference feeds.
- Niskanen, S. *OpenRocket Technical Documentation*, 2013 — sounding-rocket
  pitch-over conventions.
