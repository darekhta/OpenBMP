# OpenBMP Descent and Entry Profiles

This document specifies how OpenBMP wires its existing entry physics —
`AllenEggers` ballistic entry and the `Vinh` lifting-entry equations
(`openbmp-physics/src/reentry.rs`) — into **live mission phases**:
`entry_interface`, `lifting_entry`, `final_descent`, and `recovery`. It is a
companion to
[`flight-profiles-architecture.md`](flight-profiles-architecture.md).

The entry phase of a flight profile is the academically richest: it couples
the rigid-body dynamics, the atmosphere, hypersonic aerodynamics, and (for a
lifting body) the descent controller. OpenBMP ships the closed-form physics and
the aerothermal models; what is missing is the *handoff* from the ballistic
descent into a live, integrated entry phase, and a descent-phase control
configuration.

## Current state

- `EntryInterfaceBuilder` builds an entry state at ≈ 122 km; `AllenEggers` and
  `Vinh` reproduce textbook entry solutions. The module states plainly: *"no
  operational re-entry profiles or targeting logic are shipped."*
- These are *analysis tools*, run standalone against benchmarks (Apollo 4,
  Stardust). Schema v3 now adds `[entry_profile]` so a scenario can declare
  and validate the `ballistic_descent → entry_interface → final_descent`
  handoff and the runner can produce entry diagnostics from a supplied sample.
- `maneuvering` entry is a rejected operational term; `lifting_entry` is its
  accepted academic replacement (already in the vocabulary canon).

## Design

### Entry-interface handoff

`ballistic_descent` (see
[`ballistic-coast-and-apogee.md`](ballistic-coast-and-apogee.md)) transitions
to `entry_interface` when the body crosses ≈ 122 km descending — the existing
`at_altitude_descending` trigger:

```toml
[[mission.events]]
id = "evt_entry_interface"
trigger = { kind = "at_altitude_descending", altitude_m = 122000.0 }
action  = { kind = "enter_phase", phase = "entry_interface" }
```

At `entry_interface` the kernel's full force stack (gravity + atmosphere + aero
+ aerothermal) becomes active for the body if it was coasting on gravity alone.
This is a phase-gated force-stack selection, which the assembly / force-model
seam already supports — the new requirement is that the gate be keyed on the
reserved entry phases.

> **Status.** `entry_interface` is now consumed by schema-v3
> `[entry_profile]` validation. The handoff uses the existing
> `at_altitude_descending` trigger; no new trigger variant is required.

### Ballistic vs lifting entry

Two academic entry modes, selected by the vehicle and its aero model:

- **Ballistic entry.** No lift; the body decelerates per Allen-Eggers physics.
  Suitable for a `point_mass` or a non-lifting `rigid_body` with a tabulated
  drag deck. The descent is uncontrolled; the autopilot trajectory loop is
  idle.
- **Lifting entry** (`lifting_entry`). Non-zero L/D with a commanded bank angle
  σ — the Vinh six-state formulation (radius, longitude, latitude, speed,
  flight-path angle, heading), with bank as the single control. This is the
  academic lifting-entry problem (Apollo, Shuttle, IXV in the literature).

```
entry_interface
   │
   ├─ ballistic ──► Allen-Eggers deceleration ──► final_descent
   │
   └─ lifting_entry ──► Vinh L/D, bank-angle modulation ──► final_descent
```

> **Safety boundary — bank-angle control.** The bank-angle control here
> modulates the **vertical lift component** to manage the deceleration /
> heat-rate profile and downrange extension — the academic entry-corridor
> problem. It is **not** a terminal-homing or skid-to-turn / bank-to-turn
> end-game autopilot steering to a location; those are categorically rejected
> by [`safety-boundaries.md`](safety-boundaries.md). The bank command is driven
> by an *entry-corridor* reference (heat-rate, load-factor, flight-path-angle
> limits), never by a target. The lint forbids `terminal-homing`,
> `bank-to-turn`-shaped, and aimpoint fields in the entry block.

### Descent-phase control and recovery

For a controlled lifting entry, the descent uses the existing three-loop
autopilot with a `final_descent` entry in the gain schedule — the platform
already gain-schedules per phase via `VehicleStatus.phase_id`. The bank-angle
reference is supplied by an *entry-corridor* law analogous to the ascent
reference generator: it tracks heat-rate and load-factor limits, not a place.

`final_descent` ends with recovery deployment via the existing
`deploy_recovery` action (drogue, then main) — already implemented and
validated:

```toml
[[mission.events]]
id = "evt_drogue"
trigger = { kind = "at_altitude_descending", altitude_m = 5000.0 }
action  = { kind = "deploy_recovery", id = "drogue", command = "deploy_drogue" }
```

After touchdown the body reaches `recovery` → `post_flight`, and its
range-safety footprint is reported (see
[`ballistic-coast-and-apogee.md`](ballistic-coast-and-apogee.md) § Range-safety
footprint).

### Aerothermal coupling

During `entry_interface` and `lifting_entry` the aerothermal models
(`openbmp-aerothermal`: Sutton-Graves stagnation heating, boundary-layer
state, the ablation toy) run as research-grade diagnostics over the entry
state. These are already shipped at research grade; the only new wiring is that
the entry phases activate them. No new heating physics is introduced here.

## Fail-closed validation

- `lifting_entry` requires a `rigid_body` vehicle with a lift-capable aero
  model (a drag-only deck with `lifting_entry` is rejected — a scenario-consumer
  disagreement between the declared phase and the aero capability).
- An entry profile whose peak altitude or velocity exceeds the atmosphere
  model's validity ceiling is rejected (e.g. requesting USSA76 above 86 km for
  the entire entry).
- The entry-corridor reference accepts only corridor limits (heat-rate,
  load-factor, flight-path-angle); any aimpoint / geographic field is rejected
  by the lint.
- `deploy_recovery` ordering is validated as today (drogue before main; device
  references resolve).

## Schema stub summary

| Item | Location | State |
|---|---|---|
| `entry_interface`, `lifting_entry`, `final_descent` phases | vocabulary canon + scenario validation | Consumed by schema-v3 `[entry_profile]` agreement checks. |
| `EntryCorridorReference` trait | `openbmp-physics/src/profile.rs` | Implemented as a bounded corridor-reference helper. |
| Entry diagnostics | `openbmp-runner/src/entry.rs` | `entry_profile_for_sample` consumes Allen-Eggers / Vinh without producing commands. |
| Phase-gated entry force-stack activation | `openbmp-runner` | Deferred richer coupling; current slice validates the handoff and requires aero/atmosphere support. |

## References

- Allen, H. J., Eggers, A. J. NACA TR-1381, 1958 — ballistic entry
  deceleration and heating (physics only; no operational parameters shipped).
- Vinh, N. X., Busemann, A., Culp, R. D. *Hypersonic and Planetary Entry Flight
  Mechanics*, Univ. Michigan Press, 1980 — the six-state lifting-entry
  equations and the bank-angle entry-corridor problem.
- Anderson, J. D. *Hypersonic and High-Temperature Gas Dynamics*, 3rd ed.,
  AIAA, 2019 — aerothermal context for the entry phase.
- Knacke, T. W. *Parachute Recovery Systems Design Manual*, 1992 — drogue /
  main recovery sequencing.
