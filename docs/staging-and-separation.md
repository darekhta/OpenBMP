# OpenBMP Staging and Separation

This document specifies how OpenBMP executes **multi-stage separation** — the
moment a spent stage, strap-on booster, or payload fairing parts from the
continuing stack, and both bodies thereafter propagate as independent rigid
bodies. It is a companion to
[`flight-profiles-architecture.md`](flight-profiles-architecture.md) and is
purely rigid-body dynamics: staging is how every multi-stage sounding rocket
and launch vehicle flies, and the physics is the conservation of linear and
angular momentum across a jettison.

## Current state

Stage separation has a fixed-step RK4 rigid-body implementation with explicit
per-body ownership for the force and mass resources that can survive a split:

- `[[multi_body.separation]]` parses into `MultiBodyConfig` /
  `MultiBodySeparationConfig` (`openbmp-scenario/src/document.rs`), v3-gated,
  and is cross-checked against matching mission `jettison_stage` events.
- `ScenarioActionConfig::JettisonStage { body }` validates when the body
  exists, the vehicle is `rigid_body`, the body is jettisoned only once, and a
  matching `[multi_body]` separation conserves linear momentum to tolerance.
- `ScenarioActionConfig::JettisonBodies { bodies }` validates the same body
  references but applies a batch separation from one pre-split state. This is
  the coordinated deployment path for a bus releasing multiple independent
  bodies on the same event tick.
- The runner executes `jettison_stage` for rigid-body, fixed-step RK4
  profiles. The kernel partitions the composite state into the continuing
  stack and departing body, then propagates both lanes in a deterministic body
  order.
- Aero decks, motors, liquid engines, tanks, recovery devices, and effectors
  participate after separation only when the scenario declares `mounted_to`
  ownership for each resource. Each lane evaluates only resources owned by its
  active body.
- Split-time mass properties are rebuilt from dry body properties plus the
  live owned motor, engine, and tank snapshots. Engine consumed mass is debited
  from the owning body; tank mass, CG offset, and inertia delta are applied to
  the owning body.
- Telemetry keeps the continuing stack in the existing rigid-body channels and
  records departing lanes under `body.<lower_body_id>.*`, including a
  `body.<lower_body_id>.separated` flag.
- `ScenarioActionConfig::Separation` exists but `validate()` rejects it with
  `ScenarioError::UnsupportedActionKind { kind: "separation",
  missing_capability: "scripted stage separation" }`.
- The generic rigid-body kernel also fails closed unless the active force,
  moment, and rigid-mass models explicitly declare that they are safe to reuse
  for independently propagated separated bodies.

The validation suite includes a textbook gravity-only two-body split and an
owned-engine split proving the continuing body receives thrust after
separation while the detached body coasts. This is not yet a footprint or
range-safety analysis implementation.

## Design

### Vehicle model: the stage tree

A multi-stage vehicle is the existing `[[vehicle.assembly.bodies]]` tree, read
as a *separation tree*: each body carries the id of the body it separates
*from* and the event that triggers it. Before any separation, the assembly is
a single composite rigid body whose mass, center of mass, and inertia tensor
are the parallel-axis composition of its constituent bodies — exactly the
composition the assembly tree already computes.

```
assembly (composite, t < t_sep)
├── body "stage1"   (separates_at = "evt_stage1_burnout")
├── body "stage2"
└── body "fairing"  (separates_at = "evt_fairing_jettison")
```

### Action: `jettison_stage`

Separation is commanded by a new scripted action, distinct from the legacy
`separation` placeholder so the data model carries *which* body leaves:

```toml
[[mission.events]]
id = "evt_stage1_burnout"
trigger = { kind = "at_mass_fraction", remaining = 0.05 }
action  = { kind = "jettison_stage", body = "stage1" }
once    = true
```

A single event can also release multiple bodies atomically:

```toml
[[mission.events]]
id      = "evt_rv_deploy"
trigger = { kind = "at_time", time_s = 120.0 }
action  = { kind = "jettison_bodies", bodies = ["rv1", "rv2", "rv3"] }
once    = true
```

The shipped
[`scenarios/multi-body/bus-rv-deployment.toml`](../scenarios/multi-body/bus-rv-deployment.toml)
fixture exercises this path end-to-end through `openbmp run`.

Post-separation events can observe detached lanes through relative
range crossings:

```toml
[[mission.events]]
id      = "rv1_clear"
trigger = { kind = "at_relative_distance", body = "rv1", distance_m = 25.0 }
action  = { kind = "emit_telemetry_marker", tag = "rv1_clear" }
once    = true
```

Omitting `reference_body` measures against the current primary lane.
Supplying `reference_body = "rv2"` measures the range between two
active propagated bodies. The trigger is false until all named lanes
exist, so it is safe to declare before the deployment event.

> **Status.** `ScenarioActionConfig::JettisonStage { body }` is implemented for
> fixed-step RK4 rigid-body profiles with explicit resource ownership.
> `ScenarioActionConfig::JettisonBodies { bodies }` is implemented as a batch
> partition from the same pre-separation state, requiring matching
> `[[multi_body.separation]]` entries for every body. The legacy bare
> `separation`
> variant remains for backward compatibility and stays deferred because it does
> not identify which body leaves the stack.

### Separation dynamics

At the firing tick the composite splits into two rigid bodies. Let the
pre-separation composite have velocity **v**, angular velocity **ω**, center
of mass **r**_cm, and the departing stage have mass `m_s`, the continuing
stack `m_c`.

1. **Mass / inertia rebuild.** Both bodies are re-derived from the assembly
   subtrees: the continuing stack drops the departing subtree from its
   parallel-axis composition; the departing stage becomes a standalone
   composite. This reuses the assembly composition already used at `t = 0`.
2. **State partition.** Each body inherits the composite translational and
   angular state at its own (new) center of mass:
   `v_i = v + ω × (r_cm,i − r_cm)`.
3. **Separation impulse.** An optional spring/pneumatic separation imparts
   equal-and-opposite momentum along the body-frame separation axis, drawn
   from the existing `upper_delta_v_body_m_s` / `lower_delta_v_body_m_s` fields
   with momentum conservation enforced (the `momentum_conservation` flag the
   config already carries):
   `m_c · Δv_c + m_s · Δv_s = 0`.

```
            before                         after
        ┌───────────┐               ┌───────┐   ┌───────┐
   ω,v  │  composite│      ──►       │ stack │   │ stage │
        └───────────┘               └───────┘   └───────┘
                                     +Δv_c        −Δv_s   (Σ p conserved)
```

### Simultaneous propagation

After separation the kernel integrates independent rigid-body states in
fixed-step RK4 and deterministic declared separation order. The implemented
default strategy is:

- **Independent bodies (default).** Once separated, the bodies share no force
  coupling. Each is its own integration target with a filtered force stack:
  the spent stage may keep gravity + drag + recovery; the continuing stack may
  keep propulsion + aero + control. Determinism is preserved by a fixed,
  declared body-iteration order.
- **Batch deployment.** When `jettison_bodies` fires, every listed lower body
  is partitioned from the same pre-split composite before the primary stack is
  updated. This avoids the mass-loss artefact that would occur if multiple RVs
  were jettisoned sequentially from an already-reduced bus state.
- **Relative event observation.** `at_relative_distance` lets mission events
  fire from ranges between detached bodies and the primary lane, or between two
  detached bodies. The kernel seeds the previous range at the separation instant
  so the first post-separation crossing is not missed.
- **Coupled (reserved).** Plume impingement or tether coupling between freshly
  separated bodies is out of scope and reserved.

> **Status.** Coupled-body effects are still deferred. Model-level capability
> gates prevent any ambiguous composite force stack from being silently reused
> for detached bodies.

### Spent-stage fate

A jettisoned body is not deleted in PR1: the kernel retains and propagates its
rigid-body state. Ground-intersection, cull policy, and range-safety footprint
reporting remain future work in
[`ballistic-coast-and-apogee.md`](ballistic-coast-and-apogee.md). This is the
recovery / range-safety use case — *where does the spent hardware come down* —
not targeting.

## Fail-closed validation

Following the project's scenario-consumer agreement convention, separation
validation is cross-block and fail-closed:

- `jettison_stage.body` must reference a declared
  `[[vehicle.assembly.bodies]]` id, else
  `ScenarioError::UnknownBodyReference`.
- A body may be jettisoned at most once across the event list (no double
  separation) — `ScenarioError::DuplicateValue`.
- If `[multi_body]` declares a separation for a body, a matching
  `jettison_stage` action must trigger it (and vice-versa): a separation
  declared in one block and never commanded in the other fails closed, mirror
  of the wind / atmosphere `kind` agreement checks.
- `conserve_momentum = true` with asymmetric `Δv` that violates
  `m_c·Δv_c + m_s·Δv_s = 0` beyond tolerance is rejected at load.
- Separation requires a `rigid_body` vehicle; a `point_mass` vehicle with a
  `jettison_stage` action is rejected (a point mass has no body to part).
- In a `[multi_body]` scenario, aero, motor, engine, recovery, and effector
  resources must declare `mounted_to`; tanks already require it. Missing or
  unknown ownership fails closed before the runner starts.
- The runtime still requires fixed-step RK4 for `[multi_body]`; adaptive
  trajectory solvers are rejected until event localization and lane insertion
  are implemented for them.

## Schema stub summary

| Item | Location | State |
|---|---|---|
| `ScenarioActionConfig::JettisonStage { body }` | `openbmp-scenario/src/document.rs` | Implemented for v3 rigid-body stage separation. |
| `ScenarioActionConfig::JettisonBodies { bodies }` | `openbmp-scenario/src/document.rs` | Implemented for coordinated same-tick multi-body deployment. |
| `StageSeparationModel` trait | `openbmp-physics/src/profile.rs` | Implemented by `MomentumConservingStageSeparation` with closed-form tests. |
| `ScenarioError::UnknownBodyReference` | `openbmp-scenario/src/error.rs` | Reused (already exists for tank→body checks; `{ field, value }`). |
| Multi-body simultaneous propagation | kernel (`openbmp-sim`) | Implemented for fixed-step RK4 rigid-body profiles with explicit per-body force, moment, snapshot, and mass ownership. |

## References

- Niskanen, S. *OpenRocket Technical Documentation*, 2013, Ch. 4 — staging and
  the moment of separation in sounding rockets.
- Wiesel, W. E. *Spaceflight Dynamics*, 3rd ed., 2010 — momentum partition
  across a separating composite.
- Sutton, G. P., Biblarz, O. *Rocket Propulsion Elements*, 9th ed., 2017,
  Ch. 4 — staging mass ratios and the rationale for jettison.
