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

Stage separation now has a PR1 implementation in a deliberately small,
validated envelope:

- `[[multi_body.separation]]` parses into `MultiBodyConfig` /
  `MultiBodySeparationConfig` (`openbmp-scenario/src/document.rs`), v3-gated,
  and is cross-checked against matching mission `jettison_stage` events.
- `ScenarioActionConfig::JettisonStage { body }` validates when the body
  exists, the vehicle is `rigid_body`, the body is jettisoned only once, and a
  matching `[multi_body]` separation conserves linear momentum to tolerance.
- The runner executes `jettison_stage` for rigid-body, fixed-step RK4,
  gravity-only profiles. The kernel partitions the composite state into the
  continuing stack and departing body, then propagates both lanes in a
  deterministic body order.
- `ScenarioActionConfig::Separation` exists but `validate()` rejects it with
  `ScenarioError::UnsupportedActionKind { kind: "separation",
  missing_capability: "scripted stage separation" }`.
- Force-rack-specific detached-body propagation (aero, thrust, tanks,
  recovery, coupled-body effects) remains deferred and fails closed at runner
  construction.

The PR1 validation case is a textbook two-body split with closed-form momentum
balance and byte-stable rerun evidence. It is not a footprint or recovery
analysis implementation.

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

> **Status.** `ScenarioActionConfig::JettisonStage { body }` is implemented for
> the PR1 rigid-body gravity-only envelope. The legacy bare `separation`
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

After separation the kernel integrates **two** rigid-body states in the PR1
envelope. The current implementation is fixed-step RK4, gravity-only, and
deterministic by declared separation order. Broader force-stack support remains
future work. The intended long-term strategies are:

- **Independent bodies (default).** Once separated, the bodies share no force
  coupling. Each is its own integration target with its own force stack
  (the spent stage typically: gravity + drag + recovery; the continuing stack:
  full propulsion + aero + control). Determinism is preserved by a fixed,
  declared body-iteration order.
- **Coupled (reserved).** Plume impingement or tether coupling between freshly
  separated bodies is out of scope and reserved.

> **Status.** PR1 ships independent two-lane propagation for the validated
> gravity-only case. Coupled-body effects and per-body force-stack ownership are
> still deferred.

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

## Schema stub summary

| Item | Location | State |
|---|---|---|
| `ScenarioActionConfig::JettisonStage { body }` | `openbmp-scenario/src/document.rs` | Implemented for v3 rigid-body stage separation. |
| `StageSeparationModel` trait | `openbmp-physics/src/profile.rs` | Implemented by `MomentumConservingStageSeparation` with closed-form tests. |
| `ScenarioError::UnknownBodyReference` | `openbmp-scenario/src/error.rs` | Reused (already exists for tank→body checks; `{ field, value }`). |
| Multi-body simultaneous propagation | kernel (`openbmp-sim`) | Implemented for fixed-step RK4, gravity-only PR1 validation profiles; broader force ownership deferred. |

## References

- Niskanen, S. *OpenRocket Technical Documentation*, 2013, Ch. 4 — staging and
  the moment of separation in sounding rockets.
- Wiesel, W. E. *Spaceflight Dynamics*, 3rd ed., 2010 — momentum partition
  across a separating composite.
- Sutton, G. P., Biblarz, O. *Rocket Propulsion Elements*, 9th ed., 2017,
  Ch. 4 — staging mass ratios and the rationale for jettison.
