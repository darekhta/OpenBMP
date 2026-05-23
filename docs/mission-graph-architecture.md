# Mission Graph Architecture

This document is the **authoritative architectural reference** for the
mission state machine in OpenBMP, lock-stepped with
[`phase-5x-plan.md`](phase-5x-plan.md). It describes what mission state
*is*, who owns it, how it's evaluated, and what guarantees the
architecture provides to downstream HAL adopters.

This document supersedes the Phase 3.2 / Phase 4 mission-FSM material in
[`software-architecture.md § Mission State Machine`](software-architecture.md)
and [`software-architecture.md § Event / Phase Timeline`](software-architecture.md).
Those sections are kept for historical context but reference back here.

The vocabulary canon (state names, region names, rejected operational
names) lives in
[`mission-states-vocabulary.md`](mission-states-vocabulary.md). This
document defines the *machinery*; the vocabulary doc defines the
*names that go into it*.

## Goals

The mission state machine is the structural backbone of the flight
controller. Its goals, in order of precedence:

1. **Single source of truth.** There is one authoritative mission
   state, owned by the FC commander, observed by everyone else.
2. **HAL-portable.** A downstream adopter who runs the FC commander
   on real hardware (no simulator) sees an FSM that behaves
   identically to the in-simulator version. The simulator's
   scenario-scripted physics overrides do not leak into the FSM
   contract.
3. **Hierarchical.** States can nest (Harel statechart semantics).
   Transitions declared on a parent state are inherited by every
   descendant. Entry / exit / do actions fire deterministically on
   transition traversal.
4. **Concurrent.** Orthogonal state axes (mission × health × comms ×
   estimator-regime) evolve independently in deterministic per-region
   tick order.
5. **Deterministic.** All identifiers are path-derived FNV-1a-64;
   canonical-form sorts make the in-memory representation
   declaration-order-independent; iteration order is fixed; no
   wall-clock, no system RNG, no allocation on hot paths.
6. **Validated at scenario load.** Every structural property the FSM
   relies on (acyclicity within a region, reachability of every
   declared state, single-target per `(state, event)` pair, cross-
   region guard well-formedness) is enforced at scenario parse time.
   Runtime cannot enter an invalid state.
7. **Explicit about scope.** The crate that ships the FSM does not
   contain simulator-specific physics overrides. Those live in a
   separate crate that the simulator depends on.

## Conceptual model

OpenBMP's mission FSM is a **hierarchical state machine with
orthogonal regions** — a Harel statechart, in academic terminology.
Every active execution carries:

- A current state path within each region (e.g.
  `mission.states.in_flight.boost.first_stage_burn`).
- A history pseudo-state recording the last-active child of each
  composite state (so resume-after-interrupt is well-defined).
- A per-tick previous snapshot of the scalars triggers depend on (so
  crossing detectors can fire deterministically).

The model has six core concepts:

### State

A point in the FSM where the system can rest. States can be:

- **Atomic** — no children; the system is "in" this state and
  nothing else along its branch.
- **Composite** — has at least one child; the system is in the
  composite state *and* one of its children.
- **History (pseudo)** — not a state itself; references the
  last-active child of a composite parent. Transitioning *to* a
  history pseudo-state means "re-enter the parent and resume the
  sub-state we were in last time."

Each state carries:

```rust
pub struct State {
    pub id: StateId,                          // FNV-1a-64(canonical path)
    pub label: String,                        // human-readable
    pub parent: Option<StateId>,              // None for region root
    pub on_entry: Vec<MissionAction>,         // fire on entering this state
    pub on_exit:  Vec<MissionAction>,         // fire on leaving this state
    pub on_active: Vec<MissionAction>,        // fire every tick this state is active
    pub allowed_effectors: Vec<String>,       // mixer authority gate
    pub allowed_engines: Vec<String>,         // engine authority gate
    pub history: Option<HistoryPolicy>,       // None | Shallow | Deep
}
```

### Transition

A directed edge from `from: StateId` to `to: StateId`, fired by an
event, optionally guarded by a cross-region precondition:

```rust
pub struct Transition {
    pub from: StateId,
    pub to: StateId,
    pub event: EventId,
    pub guard: Option<GuardClause>,           // optional cross-region precondition
    pub priority: u32,                        // tie-break for ambiguous (from, event)
}

pub enum GuardClause {
    InRegionState { region: RegionId, state: StateId },
    All(Vec<GuardClause>),
    Any(Vec<GuardClause>),
    Not(Box<GuardClause>),
}
```

A transition fires when:
1. Its `event` fires this tick.
2. The current state is `from` *or any descendant of `from`* (parent
   transitions are inherited; descendants override only when they
   declare a more specific transition for the same event).
3. The guard, if any, evaluates true against the *current* state of
   the named regions.

When multiple transitions match, the highest-priority transition
wins. If priorities tie, scenario load rejects the FSM with
`AmbiguousTransition`.

### Event

A predicate evaluated each tick. Events are crossing detectors
(see Phase 3.2 design): they fire on the tick the monitored value
transitions across the threshold, never re-fire while on the same
side. Events are declared per scenario; the trait surface is:

```rust
pub trait EventTrigger {
    fn fired(&self, state: &EventEvalState, t: SimTime, step: StepIndex) -> bool;
}
```

Built-in triggers (see [Phase-5 closed set](#phase-5-closed-trigger-set)).

### Region

An orthogonal concurrent state machine with its own root state and
event-binding list. Regions evolve independently each tick. The four
canonical regions:

| Region | What it tracks | Owner | Example states |
|---|---|---|---|
| `mission` | Flight phase | Commander | `Pad`, `InFlight.Boost`, `InFlight.Coast`, `Descent`, `Recovery` |
| `health` | FDIR-aggregated health regime | Commander, with FDIR subscriptions | `Nominal`, `Degraded`, `AbortRequested`, `SafedOnFault` |
| `comms` | Ground-link state | Commander, with comms subscriptions | `Linked`, `Degraded`, `LossOfSignal`, `SafedOnLossOfSignal` |
| `estimator_regime` | IMM mode probability argmax | Estimator (publishes), commander observes | `BoostMode`, `CoastMode`, `DescentMode` |

The `mission` and `health` regions are decided by the commander.
`estimator_regime` is *observed* by the commander (so it can guard
mission transitions on it) but *decided* by the IMM estimator. This
asymmetry is intentional: the commander does not infer regime from
sensor data; that's the estimator's job.

Custom regions can be declared in v4 scenarios via
`[[mission.regions]]` for downstream HAL adopters who need additional
axes (e.g. `payload_state`, `tank_state`, `crew_state` for crewed
vehicles). The four canonical regions are always present.

### Action

The actions an FSM transition can produce. Phase 5.X splits these into
two enums in two crates:

#### `MissionAction` (HAL-portable, in `openbmp-mission`)

Actions that any FC commander — sim or real-hardware — must support:

```rust
pub enum MissionAction {
    EnterState(StateId),
    EmitTelemetryMarker { tag: String },
    RaiseHealthAlarm { region: RegionId, alarm: AlarmCode },
    RequestSafeState { reason: String },
    Stop { label: String },
}
```

These actions describe *what the FC decides*. Every variant
represents a controller-side decision that has meaning regardless of
whether the actuators are simulated or real.

#### `ScenarioScriptAction` (sim-only, in `openbmp-scenario-script`)

Actions that override physics state directly, bypassing the FC. These
exist only for V&V scenarios that need to test "what does the FC do if
the engine is forced on / off / chute is forced deployed regardless of
controller decision":

```rust
pub enum ScenarioScriptAction {
    EngineCommand { id, throttle_unit, gimbal_pitch_rad, gimbal_yaw_rad,
                    ignite, shutdown },
    EffectorOverride { id, command },
    Separation { /* impulse and split parameters */ },
    DeployRecovery { id, command },
}
```

The simulator kernel holds a separate `Vec<EventBinding<ScenarioScriptAction>>`
that fires alongside (but independently of) the FC's mission FSM.
HAL deployments do not depend on this crate.

#### Why the split

The previous design merged both action types into a single
`EventAction` enum in `openbmp-mission`. That made the crate
*compile* without simulator dependency but did not make it
*semantically portable*: half the enum variants meant nothing in a
real-hardware deployment. The split makes the contract honest — the
enum a HAL adopter sees has only meaningful variants.

### Binding

A trigger + action + once-flag, generic over the action type:

```rust
pub struct EventBinding<A> {
    pub id: EventId,                          // FNV-1a-64(canonical path)
    pub trigger: BuiltInEventTrigger,         // or any EventTrigger impl
    pub action: A,                            // MissionAction or ScenarioScriptAction
    pub once: bool,                           // most events fire exactly once
}
```

The FC commander holds `Vec<EventBinding<MissionAction>>`. The
simulator kernel holds two binding lists:
`Vec<EventBinding<MissionAction>>` (a *mirror* of the FC's,
populated from the same scenario block, used only for telemetry
markers and observable-event recording) and
`Vec<EventBinding<ScenarioScriptAction>>` (sim-only physics
overrides). The mirror is permitted because it is *observation only*:
the simulator does not transition mission state from this list; it
only observes which mission actions fire so it can record telemetry.
In production HAL deployments, the simulator side disappears; the FC's
binding list is the only one that exists.

## Single source of truth

The FC commander is the **sole owner** of mission state. Every
consumer reads it from the bus.

```text
┌─────────────────────────────────────────────────────────────────┐
│ FC Commander                                                    │
│                                                                 │
│ owns: MissionStateMachine (region per axis, with hierarchy)     │
│ owns: per-region current_state, history pseudo-states           │
│ owns: per-binding fired_once tracking                           │
│ owns: previous_event_scalars                                    │
│                                                                 │
│ publishes: commander.mission_state                              │
│ publishes: commander.region.mission                             │
│ publishes: commander.region.health                              │
│ publishes: commander.region.comms                               │
│ publishes: commander.region.estimator_regime  (mirror)          │
│ publishes: commander.transition_log    (every transition fires) │
│ publishes: commander.guard_blocked     (when guards suppress)   │
│                                                                 │
│ subscribes: estimator.regime           (IMM publishes)          │
│ subscribes: fdir.status                (FDIR publishes)         │
│ subscribes: comms.status               (HAL-side, sim mocks)    │
└─────────────────────────────────────────────────────────────────┘

       ▼ everything else subscribes ▼

┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ Autopilot        │  │ Mixer            │  │ Simulator kernel │
│ (gain schedule)  │  │ (effector gate)  │  │ (telemetry only) │
└──────────────────┘  └──────────────────┘  └──────────────────┘
```

This means:

- The simulator kernel has *no* `mission_graph` field and *no*
  `current_phase` field. Its only role w.r.t. mission state is to
  receive `commander.mission_state` and use it for telemetry tagging.
- The simulator kernel still owns the
  `Vec<EventBinding<ScenarioScriptAction>>` because those bindings
  are physics-level overrides that fire from scenario triggers —
  these are *not* mission state.
- The FC commander runs in a HAL deployment unchanged. Whatever
  HAL the adopter writes for `Sensor` / `ControlEffector` /
  `EngineModel` / `Clock` plugs in below the commander; the
  commander's mission FSM logic is identical.

### Test-only override channel

Some V&V scenarios need to script the commander into a specific
mission state to test branch behavior (e.g. "what does the FDIR do if
we are scripted into `Recovery` early"). For those scenarios, an
opt-in topic `commander.scenario_state_override` carries a directive
the commander applies as an `EnterState` action.

This topic is **build-time gated**:

- The simulator and tests build with feature `mission-test-overrides`
  enabled (default).
- HAL crates set `default-features = false` and the topic compiles
  out entirely. The directive cannot reach the commander; the
  commander is the sole authority in HAL deployments.

This honors the architectural rule: the simulator may *script* the
commander for V&V purposes, but the same code path cannot ship to real
hardware.

## Hierarchical traversal semantics

Following Harel / `statig` semantics:

When a transition fires from state `A` to state `B`, the FSM:

1. Computes the **lowest common ancestor** (LCA) of `A` and `B` along
   the parent chain.
2. **Exits** states from `A` upward to (but not including) the LCA,
   firing each state's `on_exit` actions in bottom-up order.
3. **Enters** states from the LCA downward to `B`, firing each
   state's `on_entry` actions in top-down order.
4. If `B` is a composite state, **descends** to its initial child
   (or the history pseudo-state if `B` was previously visited and has
   `history: Shallow | Deep`), firing each descendant's `on_entry`.

After the transition completes, every state in the path from region
root to the deepest active descendant has fired its `on_active`
actions for this tick.

**Determinism contract for traversal:** within each level of the
hierarchy, action firing order is determined by the canonical-form
sort of state ids (FNV-1a-64 path-derived ids in ascending order).
This is the same ordering rule as the flat-DAG version; the new
contribution is that hierarchy depth is the primary sort key and
StateId.value() is the tie-break.

## Determinism contract

The FSM preserves OpenBMP's determinism rules, extended for hierarchy
and orthogonal regions:

| Property | Mechanism | Status |
|---|---|---|
| Declaration-order-independent ids | FNV-1a-64 of canonical path | Inherited from Phase 3.2 |
| Canonical-form sort | `(depth-from-region-root, parent-StateId.value(), StateId.value())` | New for 5.X |
| Cross-region tick order | Region declaration order, with `RegionId.value()` tie-break | New for 5.X |
| Transition evaluation order within a region | `(from-depth, from-id, to-depth, to-id, EventId.value())` | Extended from Phase 3.2 |
| Guard evaluation order within a transition | Guard tree depth-first, `AND` short-circuit, `OR` short-circuit | New for 5.X |
| Action firing order | Hierarchy traversal: exit bottom-up, enter top-down; within a level, by `StateId.value()` | New for 5.X |
| Telemetry publish order | One topic per region + global mission_state, all published before next tick | New for 5.X |

No allocation on hot path. The FSM uses pre-allocated `Vec`s sized at
scenario load.

No wall-clock. The `Clock` trait injection is the same as in Phase 4;
events read `SimTime` from the trait.

No system RNG. Mission graphs do not consume RNG.

## Validation guarantees

At scenario load:

1. **Per-region acyclicity** — Tarjan SCC on each region's
   transition graph. Self-loops on transitions where `from == to` and
   no `event` are rejected; they're not transitions, they're tick
   functions (use `on_active` for that).
2. **Reachability** — every state in every region is reachable from
   the region's `initial`, computed by BFS through both transitions
   and parent-chain edges.
3. **Single-target per `(from, event)` pair** within a region.
   Hierarchical override is allowed (a child state's transition
   for the same event takes priority over the parent's), but two
   transitions at the same hierarchy level for the same event must
   declare distinct `priority` values or scenario load fails.
4. **Cross-region guard well-formedness** — every guard's referenced
   region must exist; every referenced state must exist within that
   region; guards cannot self-reference (a guard in region R may not
   reference R's current state).
5. **Inter-region action validity** — `RaiseHealthAlarm { region, …
   }` is only valid when `region` is actually declared. The
   `EnterState(StateId)` action's target state must exist in *some*
   region; the FSM applies the action to whichever region owns that
   state.
6. **Action count bounds** — each state's `on_entry` /
   `on_exit` / `on_active` action lists are capped at a documented
   maximum (32 per list at Phase 5.X; the cap may grow in v4.x as
   needs surface).
7. **History policy consistency** — a state with
   `history: Some(_)` must be a composite state.
8. **Scope guardrail (vocabulary)** — every state name must satisfy
   the canonical vocabulary check from
   [`mission-states-vocabulary.md`](mission-states-vocabulary.md). At
   load time, names matching the rejection list are refused with an
   error pointing at the academic replacement.

## Crate boundaries

```text
┌─────────────────────────────────────┐
│ openbmp-mission                     │   ← HAL-portable
│ ────────────────                    │
│ • State, Transition, Region         │
│ • MissionStateMachine               │
│ • EventTrigger trait + builtins     │
│ • EventBinding<MissionAction>       │
│ • MissionAction enum                │
│ • Vocabulary canon (state-name lint)│
│ • Determinism primitives            │
│                                     │
│ deps: openbmp-core, thiserror       │
│ no_std-friendly (Phase 5.X.C goal)  │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│ openbmp-scenario-script (NEW)       │   ← simulator-only
│ ─────────────────────────           │
│ • EventBinding<ScenarioScriptAction>│
│ • ScenarioScriptAction enum         │
│ • Sim-side physics-override helpers │
│                                     │
│ deps: openbmp-core, openbmp-mission,│
│       thiserror                     │
│ NOT consumed by HAL deployments.    │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│ openbmp-fc                          │   ← FC commander uses mission
│ ────────────                        │
│ • Commander job (single owner)      │
│ • Subscribes to estimator regime    │
│ • Publishes mission_state           │
│                                     │
│ deps: openbmp-mission only          │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│ openbmp-sim                         │   ← kernel runs script overrides
│ ─────────                           │
│ • Holds Vec<EventBinding<Script…>>  │
│ • Subscribes to mission_state       │
│ • NO mission_graph field            │
│ • NO current_phase field            │
│                                     │
│ deps: openbmp-mission,              │
│       openbmp-scenario-script       │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│ openbmp-scenario                    │   ← parses both binding lists
│ ────────────────                    │
│ • Phase-5.X.F v4 schema             │
│ • Routes mission/script bindings    │
│ • Validates vocabulary canon        │
│                                     │
│ deps: openbmp-mission,              │
│       openbmp-scenario-script       │
└─────────────────────────────────────┘
```

The dependency direction is strict: `openbmp-mission` cannot depend
on `openbmp-scenario-script`. A HAL adopter can compile and use
`openbmp-mission` and `openbmp-fc` without ever pulling in the
script crate.

## HAL adopter contract

A downstream HAL adopter integrating OpenBMP into real hardware:

1. **Depends on `openbmp-mission`** for the FSM types.
2. **Depends on `openbmp-fc`** for the commander.
3. **Does NOT depend on `openbmp-scenario-script`.** Scripted physics
   overrides are simulator-only.
4. **Does NOT depend on `openbmp-sim`.** Implements its own kernel
   adapter against `Sensor` / `ControlEffector` / `EngineModel` /
   `Clock` traits.
5. **Builds with `default-features = false` on `openbmp-fc`** to
   compile out the test-only override topic.
6. **Wires their HAL's command-bus to `commander.mission_state`** to
   route mission-state changes to real actuators (e.g. real CAN-FD or
   MAVLink commands).
7. **Owns their own qualification posture.** OpenBMP makes no
   compliance claim; the adopter accepts DO-178C / IEC 61508 / ISO
   26262 work for their target.

The mission graph contract — hierarchy, regions, transitions,
determinism — is identical in sim and HAL deployments. The only
runtime difference is the absence of scripted-physics overrides in HAL.

## Phase-5 closed trigger set

Phase 5.X locks the trigger vocabulary to what Phase 5 ships. New
trigger types are Phase 6 if hypersonic flight requires them
(e.g. `AtMachThreshold`, `AtBoundaryLayerTransition`); otherwise
rejected.

| Trigger | Description | Phase shipped |
|---|---|---|
| `AtTime { time_s }` | Crossing of elapsed monotonic time. | 3.2 |
| `AtAltitudeAscending { meters }` | Altitude crossing up through threshold. | 3.2 |
| `AtAltitudeDescending { meters }` | Altitude crossing down through threshold. | 3.2 |
| `AtApogee` | Vertical velocity flip from positive to non-positive. | 3.2 |
| `AtMassFraction { remaining }` | Mass fraction crossing down through threshold. | 3.2 |
| `AtDynamicPressure { pa, falling }` | Dynamic pressure crossing in either direction. | 3.4 (atmosphere wired) |

Composite guards from these triggers are expressible via the
`GuardClause` trees from § Transition. Closure-based / scripted
triggers (the deferred `Scripted` variant) remain rejected at scenario
parse time per Phase 3.2's design.

## Why not SCXML

The W3C SCXML standard
([W3C SCXML Recommendation](https://www.w3.org/TR/scxml/)) covers
substantially the same semantic ground (hierarchical states, parallel
regions, transitions with guards, history pseudo-states). OpenBMP
deliberately does *not* import SCXML for three reasons:

1. **Format.** SCXML is XML. OpenBMP's scenario format is TOML and
   the project's discipline is in-house Rust types with
   `serde(deny_unknown_fields)`. Importing XML adds a parser and a
   conformance burden for negligible benefit.
2. **Runtime engine.** SCXML conformance requires a runtime
   interpreter with documented execution semantics. OpenBMP's FSM is
   *compiled* into validated Rust types at scenario load — there is
   no interpreter to maintain. SCXML's runtime would either become
   another dependency surface or another in-house implementation, both
   negative-value.
3. **Tooling lock-in.** SCXML's value comes from interoperability
   with editors and verifiers (Yakindu, scxmlcc). OpenBMP's research
   posture does not benefit from interop with closed-source UML
   toolchains; the project's verification path is property tests +
   determinism CI + (later) model checking via SPIN-like tools, which
   do not require SCXML.

We borrow the *semantics* of Harel statecharts; we reject the
*serialization*.

## Why not Behavior Trees

Modern robotics autonomy stacks often prefer Behavior Trees (BTs) over
FSMs ([BehaviorTree.CPP](https://www.behaviortree.dev/),
ROS2 navigation stack). OpenBMP's regime model is not a fit for BTs
because:

- BTs are *behavior-shaped*: leaves are tasks, internal nodes are
  composite control-flow operators (sequence, fallback, parallel).
  Mission FSMs are *state-shaped*: leaves are stable resting points,
  edges are transitions on observable events.
- BTs do not naturally express orthogonal concurrent regions (BT
  parallel nodes coordinate sub-tree completion, not concurrent
  state).
- BT execution is tick-evaluated top-down; FSM execution is event-
  evaluated, which fits OpenBMP's deterministic crossing detector
  model better.

Both paradigms are valid for different problems. OpenBMP's mission
state is FSM-shaped, so we ship an FSM.

## Migration from Phase-5 flat DAG

Every shipped Phase-5 scenario migrates to the hierarchical model as a
**flat hierarchy** — every state is a direct child of the region
root, no composite states. Under flat-hierarchy migration:

- LCA of any two states is the region root, so transitions traverse
  exactly two states (exit `from`, enter `to`).
- The `on_entry` / `on_exit` action firing reduces to the existing
  Phase 3.2 semantics.
- Determinism CI passes byte-identical against the Phase-5 baseline
  (see [`phase-5x-plan.md § Determinism preservation rules`](phase-5x-plan.md#determinism-preservation-rules)).

The hierarchy *primitives* land in Phase 5.X.C; the academic state
hierarchy that exploits them lands in Phase 5.X.E together with the
vocabulary migration. Phase 6 hypersonic work then declares re-entry
sub-state hierarchies on top.

## Forward compatibility (Phase 6)

Phase 6 hypersonic work is expected to extend the architecture along
three axes that Phase 5.X intentionally does not pre-empt:

1. **Re-entry mission sub-hierarchy.** The `Descent` composite state
   gains `EntryInterface`, `LiftingEntry`, `PeakHeating`,
   `PeakDeceleration`, `MainDescent`, `FinalDescent` children. Phase
   5.X reserves these names in the vocabulary canon but does not
   ship them as states.
2. **Health region hierarchy.** Re-entry abort logic needs nested
   health states (`Degraded.Sensor`, `Degraded.Effector`,
   `AbortRequested.Aerothermal`, etc.). Phase 5.X ships flat health
   states; Phase 6 deepens.
3. **Aerodynamic regime region.** A possible fifth canonical region
   tracking `Subsonic / Transonic / Supersonic / Hypersonic` for
   aero-method selection and validity-range gating. Phase 5.X does
   *not* ship this region; Phase 6 may add it through the existing
   `[[mission.regions]]` extension point. The architecture supports
   adding regions without modifying the FSM core.

These extensions land in Phase 6 against this contract; no Phase 5.X
sub-phase ships them.

## References

- Harel, D. *Statecharts: A Visual Formalism for Complex Systems.*
  Sci. Comput. Program., 1987.
- Drusinsky, D. *Modeling and Verification using UML Statecharts.*
  Newnes, 2006 (foundational reference for Harel-style FSMs in
  embedded software).
- Samek, M. *Practical UML Statecharts in C/C++.* 2nd ed., Newnes,
  2009 (the QP framework reference; describes the active-object
  pattern OpenBMP's bus + commander pair adopts).
- Benowitz, E. *Auto-Coding UML Statecharts for Flight Software.*
  IEEE Aerospace Conference, 2006.
- F-Prime State Machines documentation:
  <https://fprime.jpl.nasa.gov/latest/docs/user-manual/framework/state-machines/>
- F-Prime FPP Language Specification:
  <https://nasa.github.io/fpp/fpp-spec.html>
- W3C State Chart XML (SCXML) Recommendation:
  <https://www.w3.org/TR/scxml/> (referenced for vocabulary; not
  imported as a serialization).
- `statig` Rust HSM crate:
  <https://github.com/mdeloof/statig> — the reference Rust HSM
  implementation OpenBMP's design draws on.
- ArduPilot AP_Mode flight-mode hierarchy:
  <https://deepwiki.com/ArduPilot/ardupilot/3.1.1-flight-modes-and-state-machine>
- PX4 Commander flight modes:
  <https://docs.px4.io/main/en/concept/flight_modes>
- BPS.space Signal phase vocabulary (referenced in
  `design-concept.md` for academic mission-phase names).

See also:
[`phase-5x-plan.md`](phase-5x-plan.md),
[`mission-states-vocabulary.md`](mission-states-vocabulary.md),
[`software-architecture.md`](software-architecture.md),
[`safety-boundaries.md`](safety-boundaries.md),
[`real-rocket-integration.md`](real-rocket-integration.md).
