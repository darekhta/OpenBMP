//! Event triggers and mission-phase graph (Phase 3.2).
//!
//! Phase-3.2 introduced declarative event-driven scheduling that
//! replaced the Phase-2 hard-coded apogee detector. Scenarios declare
//! a `[mission]` block of phases, events, and transitions; the
//! consumer (sim-side: kernel post-step hook; HAL-side: controller
//! tick or hardware-timer ISR) evaluates events at its own cadence
//! and either emits telemetry markers, transitions the active phase,
//! or halts the run.
//!
//! Phase-3.15.D made the cadence vocabulary explicit: this crate
//! ships only the data shapes + the trigger trait + the graph
//! validator. The consumer drives the evaluation cadence; the
//! mission graph itself is cadence-agnostic. `SimTime` and
//! `StepIndex` arguments to [`EventTrigger::fired`] are
//! "monotonic time at the tick" and "monotonic tick counter"
//! respectively — sim-side they bind to scenario time + kernel step
//! index; HAL-side they bind to wall-clock proxy + controller tick.
//!
//! # Module surface
//!
//! - [`PhaseId`], [`EventId`] — stable, path-derived identifiers.
//! - [`Phase`], [`PhaseTransition`] — graph node / edge data shapes.
//! - [`MissionPhaseGraph`] — acyclic phase graph with topological-sort
//!   stability. Phase 3.2.A ships the data shape only; the
//!   constructor + reachability checks land in 3.2.B.
//! - [`EventTrigger`] — trait implemented by event predicates.
//! - [`BuiltInEventTrigger`] — Phase-3.2 declarative trigger set.
//! - [`EventBinding`], [`EventAction`] — bridges trigger → action.
//! - [`EventEvalState`], [`EventScalars`] — per-step snapshot threaded
//!   into trigger evaluation.
//! - [`FiredEvent`] — kernel-side queue entry for runner fan-out.
//! - [`MissionGraphError`] — typed graph-construction errors.
//!
//! # Determinism
//!
//! - All ids are FNV-1a-64 of the canonical scenario path; reordering
//!   declarations in the TOML cannot shift any id.
//! - Triggers are crossing detectors: they fire on the step where the
//!   monitored value transitions across the trigger threshold, never
//!   re-firing while the value remains on the same side.
//! - The `once: bool` flag guards against re-firing across multiple
//!   crossings (e.g. a rocket bouncing through an altitude bound on
//!   ascent and again on descent).
//!
//! See `docs/scenario-format.md § Mission blocks (Phase 3.2)` and
//! `docs/software-architecture.md § MissionPhaseGraph and Event
//! Scheduling` for the contract.

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

use openbmp_core::{SimTime, StepIndex};
use thiserror::Error;

// ---------------------------------------------------------------------
// Identifiers
// ---------------------------------------------------------------------

/// Stable identifier for a mission phase.
///
/// Derived from the canonical scenario phase path (e.g.
/// `"mission.phases.ascent"`) via FNV-1a-64. Reordering the
/// `[[mission.phases]]` blocks in a scenario file does not shift
/// any phase's id — this is the load-bearing invariant for
/// declaration-order-independent determinism.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct PhaseId(u64);

impl PhaseId {
    /// Construct from a raw integer value.
    ///
    /// Prefer [`PhaseId::from_path`] for scenario-declared phases.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Construct from a stable canonical scenario phase path
    /// (e.g. `"mission.phases.ascent"`) via FNV-1a-64. Mirrors
    /// [`openbmp_core::SensorId::from_path`].
    #[must_use]
    pub const fn from_path(path: &str) -> Self {
        Self(fnv1a_64(path.as_bytes()))
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Stable identifier for an event binding.
///
/// Derived from the canonical scenario event path (e.g.
/// `"mission.events.at_apogee_marker"`) via FNV-1a-64. Reordering
/// the `[[mission.events]]` blocks in a scenario file does not
/// shift any event's id.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct EventId(u64);

impl EventId {
    /// Construct from a raw integer value.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Construct from a stable canonical scenario event path.
    #[must_use]
    pub const fn from_path(path: &str) -> Self {
        Self(fnv1a_64(path.as_bytes()))
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// FNV-1a-64 with the standard pinned constants. Same hash as
/// [`openbmp_core::SensorId::from_path`] — re-implemented locally so
/// [`PhaseId::from_path`] and [`EventId::from_path`] remain `const fn`.
const fn fnv1a_64(bytes: &[u8]) -> u64 {
    const FNV_OFFSET_BASIS_64: u64 = 0xcbf2_9ce4_8422_2325;
    const FNV_PRIME_64: u64 = 0x100_0000_01b3;

    let mut hash = FNV_OFFSET_BASIS_64;
    let mut i = 0;
    while i < bytes.len() {
        hash ^= bytes[i] as u64;
        hash = hash.wrapping_mul(FNV_PRIME_64);
        i += 1;
    }
    hash
}

// ---------------------------------------------------------------------
// Snapshots threaded into trigger evaluation
// ---------------------------------------------------------------------

/// Scalar values pre-computed by the kernel and threaded into
/// [`EventTrigger::fired`] evaluation. The kernel builds one of these
/// from each post-step state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EventScalars {
    /// Simulation time (seconds since scenario start).
    pub time_s: f64,
    /// ECI +z component of the position vector. Phase-3.2 treats this
    /// as the altitude proxy; multi-launch-site coordinates are deferred
    /// to a later phase.
    pub altitude_m: f64,
    /// ECI +z component of the velocity vector. Phase-3.2 treats this
    /// as vertical velocity for apogee / ascent / descent detection.
    pub vertical_velocity_m_s: f64,
    /// Current mass divided by initial mass.
    pub mass_fraction: f64,
    /// Dynamic pressure (Pa). Phase 3.2 leaves this at `0.0`; scenario
    /// parsing rejects [`BuiltInEventTrigger::AtDynamicPressure`] until
    /// Phase 3.4 wires atmosphere into event evaluation.
    pub dynamic_pressure_pa: f64,
}

/// Per-step snapshot threaded into trigger evaluation. Carries both
/// the post-step scalars and the previous-step scalars (`None` on
/// step 0) so triggers can detect crossings without interior
/// mutability.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EventEvalState {
    /// Post-step scalar values.
    pub current: EventScalars,
    /// Previous-step scalar values. `None` on step 0.
    pub previous: Option<EventScalars>,
    /// Active phase id at the start of this step. `None` when no
    /// mission graph is wired.
    pub current_phase: Option<PhaseId>,
}

// ---------------------------------------------------------------------
// Trigger trait + builtins
// ---------------------------------------------------------------------

/// Trait implemented by event triggers.
///
/// Evaluated once per **event-evaluation tick** by the consumer. The
/// simulator binds the tick to the kernel's `integrator.advance()`
/// post-step hook; a HAL adopter binds it to whichever cadence is
/// natural in their environment (sensor-sample tick, controller
/// tick, hardware-timer interrupt). Returning `true` causes the
/// consumer to record a [`FiredEvent`] for the binding.
///
/// Phase-3.15.D clarification: the `t: SimTime` and `step: StepIndex`
/// arguments are intentionally cadence-neutral — `SimTime` is the
/// monotonic time at the tick (sim-side: scenario time; HAL-side:
/// wall-clock proxy or hardware monotonic counter), and `step` is
/// the monotonic tick counter (sim-side: kernel step index;
/// HAL-side: any monotonic event-evaluation tick). The trait surface
/// does not bake in any sim-specific cadence.
pub trait EventTrigger {
    /// Returns `true` if the trigger fires this tick.
    fn fired(&self, state: &EventEvalState, t: SimTime, step: StepIndex) -> bool;
}

/// Phase-3.2 declarative trigger set. Every variant is a crossing
/// detector: it returns `true` only on the step where the monitored
/// value transitions across the trigger threshold.
///
/// `Scripted` is intentionally not part of the enum — closure-based
/// triggers are deferred to Phase 3.4 alongside `ControlEffector`.
/// Scenarios that declare `kind = "scripted"` are rejected at parse
/// time.
#[derive(Clone, Debug, PartialEq)]
pub enum BuiltInEventTrigger {
    /// Fires the first step where simulation time crosses `time_s`.
    AtTime {
        /// Trigger threshold in seconds since scenario start.
        time_s: f64,
    },
    /// Fires the first step where altitude crosses up through
    /// `meters` (i.e. `previous_alt < meters` and
    /// `current_alt >= meters`).
    AtAltitudeAscending {
        /// Altitude threshold (m).
        meters: f64,
    },
    /// Fires the first step where altitude crosses down through
    /// `meters`.
    AtAltitudeDescending {
        /// Altitude threshold (m).
        meters: f64,
    },
    /// Fires the first step where vertical velocity flips from
    /// strictly positive to non-positive — the apogee step under
    /// fixed-step integration. Sub-step apogee localization is a
    /// Phase-5 adaptive-integrator concern.
    AtApogee,
    /// Fires the first step where mass fraction (current / initial)
    /// drops to or below `remaining`.
    AtMassFraction {
        /// Threshold mass fraction in `[0, 1]`.
        remaining: f64,
    },
    /// Fires when dynamic pressure crosses `pa`. The `falling` flag
    /// selects rising-edge (`false`) or falling-edge (`true`) crossing.
    AtDynamicPressure {
        /// Threshold dynamic pressure (Pa).
        pa: f64,
        /// `false`: rising-edge crossing (q increasing through pa).
        /// `true`: falling-edge crossing (q decreasing through pa).
        falling: bool,
    },
}

impl EventTrigger for BuiltInEventTrigger {
    fn fired(&self, state: &EventEvalState, _t: SimTime, _step: StepIndex) -> bool {
        // All Phase-3.2 triggers are crossing detectors and return
        // `false` on step 0 (no previous-step snapshot).
        let Some(prev) = state.previous.as_ref() else {
            return false;
        };
        let curr = &state.current;
        match self {
            Self::AtTime { time_s } => prev.time_s < *time_s && curr.time_s >= *time_s,
            Self::AtAltitudeAscending { meters } => {
                prev.altitude_m < *meters && curr.altitude_m >= *meters
            }
            Self::AtAltitudeDescending { meters } => {
                prev.altitude_m > *meters && curr.altitude_m <= *meters
            }
            Self::AtApogee => prev.vertical_velocity_m_s > 0.0 && curr.vertical_velocity_m_s <= 0.0,
            Self::AtMassFraction { remaining } => {
                prev.mass_fraction > *remaining && curr.mass_fraction <= *remaining
            }
            Self::AtDynamicPressure { pa, falling: false } => {
                prev.dynamic_pressure_pa < *pa && curr.dynamic_pressure_pa >= *pa
            }
            Self::AtDynamicPressure { pa, falling: true } => {
                prev.dynamic_pressure_pa > *pa && curr.dynamic_pressure_pa <= *pa
            }
        }
    }
}

// ---------------------------------------------------------------------
// Bindings + actions
// ---------------------------------------------------------------------

/// Action taken when an event fires.
///
/// `Separation` is reserved-but-unwired in Phase 3.2: scenarios that
/// declare it are rejected at parse time with a typed deferral
/// error pointing at the future phase that will land it.
#[derive(Clone, Debug, PartialEq)]
pub enum EventAction {
    /// Transition the active mission phase.
    EnterPhase(PhaseId),
    /// Emit a `bool` telemetry marker. The runner allocates a channel
    /// named `tag` and writes `true` on every step the associated
    /// event fires.
    EmitTelemetryMarker {
        /// Channel tag (`snake_case`, e.g. `"at_apogee_marker"`).
        tag: String,
    },
    /// Halt the kernel with the simulator-side mission-ended stop reason.
    Stop {
        /// Human-readable label for the stop reason.
        label: String,
    },
    // -------------------------------------------------------------
    // Phase-3.4 / 3.6 / 3.7 / 3.9 actions. Variants exist in the
    // enum so future sub-phases don't need to expand it (which would
    // touch every match site).
    // -------------------------------------------------------------
    /// Phase-3.6: per-engine command targeting a declared engine by
    /// [`openbmp_core::EngineId`]. The kernel records the firing;
    /// the runner-side `EngineRack::apply_commands` drains it and
    /// applies the command to the engine on the next rack tick.
    ///
    /// Phase-3.15.C: the field shape is engine-domain-shaped but
    /// the mission graph crate does *not* depend on
    /// `openbmp-propulsion`. The runner translates these scalar
    /// fields into a typed `openbmp_propulsion::EngineCommand` at
    /// apply time — same pattern as [`Self::DeployRecovery`] which
    /// carries a `String` command name and the runner maps it to
    /// `openbmp_vehicle::RecoveryCommand`. Keeping the mission graph
    /// free of actuator-domain dependencies is the load-bearing
    /// HAL-portability rule for `openbmp-mission`.
    EngineCommand {
        /// Target engine id.
        id: openbmp_core::EngineId,
        /// Throttle setting in `[0, 1]`. Clamped at apply time.
        throttle_unit: f64,
        /// Gimbal pitch angle in radians. Clamped at apply time.
        gimbal_pitch_rad: f64,
        /// Gimbal yaw angle in radians. Clamped at apply time.
        gimbal_yaw_rad: f64,
        /// Ignition request. Honoured only from `Idle`.
        ignite: bool,
        /// Shutdown request. Honoured only from `Igniting` or
        /// `Burning`. When both `ignite` and `shutdown` are `true`,
        /// shutdown wins.
        shutdown: bool,
    },
    /// Phase-3.4: scenario-driven effector command override. Targets
    /// a declared effector by [`openbmp_core::EffectorId`]; the
    /// runner-side `EffectorRack::apply_overrides` consumes the
    /// fired event and stores the override into the rack's
    /// per-effector override map for the next rack tick.
    EffectorOverride {
        /// Target effector id.
        id: openbmp_core::EffectorId,
        /// Command value.
        command: f64,
    },
    /// Phase-3.6 / 3.7 deferred: stage-separation event.
    Separation,
    /// Phase-3.9: deploy / stow a recovery device. Targets a
    /// declared recovery device by [`openbmp_core::RecoveryId`]; the
    /// runner-side `RecoveryRack::apply_deploys` consumes the fired
    /// event and applies the command to the device's state machine.
    DeployRecovery {
        /// Target recovery-device id.
        id: openbmp_core::RecoveryId,
        /// Command name (one of `"deploy"`, `"deploy_drogue"`,
        /// `"deploy_main"`, `"stow"`). The string is the rack-side
        /// canonical name; the rack maps it to the typed
        /// `openbmp_vehicle::RecoveryCommand` enum at apply time.
        ///
        /// We carry the canonical-name `String` (rather than the
        /// typed enum) because `openbmp-sim` is L1 — it cannot
        /// depend on `openbmp-vehicle::recovery`. The rack lives in
        /// the runner (`openbmp-cli`) which depends on both crates
        /// and performs the typed-mapping there.
        command: String,
    },
}

/// One event's full declaration: trigger + action + once-flag.
#[derive(Clone, Debug, PartialEq)]
pub struct EventBinding {
    /// Path-derived stable id.
    pub id: EventId,
    /// Trigger predicate.
    pub trigger: BuiltInEventTrigger,
    /// Action taken when the trigger fires.
    pub action: EventAction,
    /// `true`: the binding fires at most once per simulation run.
    /// `false`: the binding may re-fire on every crossing.
    pub once: bool,
}

// ---------------------------------------------------------------------
// Phase graph
// ---------------------------------------------------------------------

/// Mission-phase node. `allowed_effectors` and `allowed_engines` are
/// declared but not actively gated by the kernel yet. Scenario loading
/// validates effector references in Phase 3.4; active command gating
/// lands with the later controller / propulsion phases.
#[derive(Clone, Debug, PartialEq)]
pub struct Phase {
    /// Path-derived stable id.
    pub id: PhaseId,
    /// Human-readable label for telemetry / diagnostics.
    pub label: String,
    /// Effectors permitted while this phase is active. Scenario loading
    /// validates ids; active command gating is deferred.
    pub allowed_effectors: Vec<String>,
    /// Engines permitted while this phase is active. Phase-3.6 will
    /// enforce; Phase-3.2 leaves the list informational.
    pub allowed_engines: Vec<String>,
}

/// Directed edge from `from` to `to`, fired by `event`.
#[derive(Clone, Debug, PartialEq)]
pub struct PhaseTransition {
    /// Source phase id.
    pub from: PhaseId,
    /// Destination phase id.
    pub to: PhaseId,
    /// Event whose firing triggers this transition.
    pub event: EventId,
}

/// Acyclic mission-phase graph.
///
/// Construct via [`MissionPhaseGraph::new`]; direct field assignment
/// works for tests but bypasses the validation invariants that the
/// kernel relies on. The constructor enforces:
///
/// 1. No duplicate phase ids.
/// 2. `initial` references a declared phase.
/// 3. Every transition references declared phases and a declared
///    event; event ids are unique.
/// 4. Each `(from phase, event)` pair selects at most one target.
/// 5. The graph is acyclic (Tarjan SCC).
/// 6. Every declared phase is reachable from `initial`.
/// 7. Phases are sorted by `(longest-path-depth-from-initial,
///    PhaseId.value())`; transitions by `(from-depth, from-id,
///    to-depth, to-id, EventId.value())`. The canonical form is
///    order-independent — re-ordering inputs produces an identical graph.
#[derive(Clone, Debug, PartialEq)]
pub struct MissionPhaseGraph {
    /// Phases sorted by `(depth, PhaseId.value())`.
    pub phases: Vec<Phase>,
    /// Transitions sorted by `(from-depth, from-id, to-depth, to-id,
    /// EventId.value())`.
    pub transitions: Vec<PhaseTransition>,
    /// Initial phase. Resolved at construction.
    pub initial: PhaseId,
}

impl MissionPhaseGraph {
    /// Validate and construct a canonical mission-phase graph.
    ///
    /// `declared_events` lists every [`EventId`] declared in the
    /// scenario `[[mission.events]]` block. Empty when no events are
    /// declared (the constructor still requires every transition's
    /// event to be in the list, so a transition + empty event list is
    /// always rejected).
    ///
    /// # Errors
    ///
    /// Returns [`MissionGraphError`] for any of the validation
    /// failures listed in the struct-level documentation.
    pub fn new(
        phases: Vec<Phase>,
        transitions: Vec<PhaseTransition>,
        initial: PhaseId,
        declared_events: &[EventId],
    ) -> Result<Self, MissionGraphError> {
        // 1. No duplicate phase ids.
        let mut seen = BTreeSet::new();
        for phase in &phases {
            if !seen.insert(phase.id) {
                return Err(MissionGraphError::DuplicatePhase { phase: phase.id });
            }
        }

        // 2. `initial` is a declared phase.
        if !seen.contains(&initial) {
            return Err(MissionGraphError::MissingInitial);
        }

        // 3. Every transition's `from`/`to` references a declared phase.
        for transition in &transitions {
            if !seen.contains(&transition.from) {
                return Err(MissionGraphError::UnknownPhaseId {
                    phase: transition.from,
                    in_field: Cow::Borrowed("transition.from"),
                });
            }
            if !seen.contains(&transition.to) {
                return Err(MissionGraphError::UnknownPhaseId {
                    phase: transition.to,
                    in_field: Cow::Borrowed("transition.to"),
                });
            }
        }

        // 4. Every transition's event is declared.
        let event_set: BTreeSet<EventId> = declared_events.iter().copied().collect();
        if event_set.len() != declared_events.len() {
            let mut seen_events = BTreeSet::new();
            for event in declared_events {
                if !seen_events.insert(*event) {
                    return Err(MissionGraphError::DuplicateEvent { event: *event });
                }
            }
        }
        for (i, transition) in transitions.iter().enumerate() {
            if !event_set.contains(&transition.event) {
                return Err(MissionGraphError::UnknownEvent {
                    event: transition.event,
                    in_transition: i,
                });
            }
        }

        // Each `(from phase, event)` pair must select at most one target
        // phase. Otherwise runtime transition application is ambiguous.
        let mut transition_keys = BTreeSet::new();
        for transition in &transitions {
            if !transition_keys.insert((transition.from, transition.event)) {
                return Err(MissionGraphError::AmbiguousTransition {
                    from: transition.from,
                    event: transition.event,
                });
            }
        }

        // 5. Cycle detection — Tarjan SCC.
        if let Some(involving) = detect_cycle_tarjan(&phases, &transitions) {
            return Err(MissionGraphError::Cycle { involving });
        }

        // 6. Reachability via BFS from `initial`.
        let reachable = bfs_reachable(&phases, &transitions, initial);
        for phase in &phases {
            if !reachable.contains(&phase.id) {
                return Err(MissionGraphError::UnreachablePhase { phase: phase.id });
            }
        }

        // 7. Depth = longest path from `initial`. Computed via Kahn's
        //    on the relaxation order; safe because we've established
        //    acyclicity above.
        let depth = compute_longest_path_depth(&phases, &transitions, initial);

        // 8. Canonicalise.
        let mut sorted_phases = phases;
        sorted_phases.sort_by_key(|p| {
            (
                depth.get(&p.id).copied().unwrap_or(usize::MAX),
                p.id.value(),
            )
        });

        let mut sorted_transitions = transitions;
        sorted_transitions.sort_by_key(|t| {
            (
                depth.get(&t.from).copied().unwrap_or(usize::MAX),
                t.from.value(),
                depth.get(&t.to).copied().unwrap_or(usize::MAX),
                t.to.value(),
                t.event.value(),
            )
        });

        Ok(Self {
            phases: sorted_phases,
            transitions: sorted_transitions,
            initial,
        })
    }
}

// ---------------------------------------------------------------------
// Graph algorithms
// ---------------------------------------------------------------------

/// Tarjan's strongly-connected-components algorithm. Returns
/// `Some(phase_ids)` if any SCC has size > 1 or contains a self-loop;
/// `None` if the graph is acyclic.
///
/// Implementation uses recursive depth-first search. Mission graphs are
/// expected to be small in Phase 3; switch to an explicit stack if that
/// assumption changes.
fn detect_cycle_tarjan(phases: &[Phase], transitions: &[PhaseTransition]) -> Option<Vec<PhaseId>> {
    // Self-loop is a trivial single-vertex cycle.
    for transition in transitions {
        if transition.from == transition.to {
            return Some(vec![transition.from]);
        }
    }

    // Build adjacency in deterministic order: outgoing edges per phase
    // sorted by destination `PhaseId.value()`.
    let mut adjacency: BTreeMap<PhaseId, Vec<PhaseId>> = BTreeMap::new();
    for phase in phases {
        adjacency.entry(phase.id).or_default();
    }
    for transition in transitions {
        adjacency
            .entry(transition.from)
            .or_default()
            .push(transition.to);
    }
    for outgoing in adjacency.values_mut() {
        outgoing.sort_by_key(|p| p.value());
    }

    let mut index_counter: usize = 0;
    let mut indices: BTreeMap<PhaseId, usize> = BTreeMap::new();
    let mut lowlinks: BTreeMap<PhaseId, usize> = BTreeMap::new();
    let mut on_stack: BTreeSet<PhaseId> = BTreeSet::new();
    let mut tarjan_stack: Vec<PhaseId> = Vec::new();

    for phase in phases {
        if !indices.contains_key(&phase.id)
            && let Some(cycle) = strongconnect(
                phase.id,
                &adjacency,
                &mut index_counter,
                &mut indices,
                &mut lowlinks,
                &mut on_stack,
                &mut tarjan_stack,
            )
        {
            return Some(cycle);
        }
    }
    None
}

fn strongconnect(
    v: PhaseId,
    adjacency: &BTreeMap<PhaseId, Vec<PhaseId>>,
    index_counter: &mut usize,
    indices: &mut BTreeMap<PhaseId, usize>,
    lowlinks: &mut BTreeMap<PhaseId, usize>,
    on_stack: &mut BTreeSet<PhaseId>,
    tarjan_stack: &mut Vec<PhaseId>,
) -> Option<Vec<PhaseId>> {
    indices.insert(v, *index_counter);
    lowlinks.insert(v, *index_counter);
    *index_counter += 1;
    tarjan_stack.push(v);
    on_stack.insert(v);

    if let Some(neighbours) = adjacency.get(&v) {
        for &w in neighbours {
            if !indices.contains_key(&w) {
                if let Some(cycle) = strongconnect(
                    w,
                    adjacency,
                    index_counter,
                    indices,
                    lowlinks,
                    on_stack,
                    tarjan_stack,
                ) {
                    return Some(cycle);
                }
                let w_low = lowlinks[&w];
                let v_low = lowlinks[&v];
                lowlinks.insert(v, v_low.min(w_low));
            } else if on_stack.contains(&w) {
                let w_idx = indices[&w];
                let v_low = lowlinks[&v];
                lowlinks.insert(v, v_low.min(w_idx));
            }
        }
    }

    if lowlinks[&v] == indices[&v] {
        // SCC roots: pop until we find `v`. SCC of size > 1 is a cycle.
        let mut scc = Vec::new();
        while let Some(w) = tarjan_stack.pop() {
            on_stack.remove(&w);
            scc.push(w);
            if w == v {
                break;
            }
        }
        if scc.len() > 1 {
            // Sort the cycle deterministically so the error message is
            // stable across reruns.
            scc.sort_by_key(|id| id.value());
            return Some(scc);
        }
    }
    None
}

/// BFS from `start` returning the set of reachable phase ids
/// (including `start`).
fn bfs_reachable(
    phases: &[Phase],
    transitions: &[PhaseTransition],
    start: PhaseId,
) -> BTreeSet<PhaseId> {
    let mut adjacency: BTreeMap<PhaseId, Vec<PhaseId>> = BTreeMap::new();
    for phase in phases {
        adjacency.entry(phase.id).or_default();
    }
    for transition in transitions {
        adjacency
            .entry(transition.from)
            .or_default()
            .push(transition.to);
    }

    let mut reachable = BTreeSet::new();
    let mut queue: VecDeque<PhaseId> = VecDeque::new();
    if adjacency.contains_key(&start) {
        reachable.insert(start);
        queue.push_back(start);
    }
    while let Some(v) = queue.pop_front() {
        if let Some(neighbours) = adjacency.get(&v) {
            for &w in neighbours {
                if reachable.insert(w) {
                    queue.push_back(w);
                }
            }
        }
    }
    reachable
}

/// Longest-path depth from `initial` in the (already-validated as
/// acyclic) phase graph. Uses Kahn's topological sort and relaxes
/// `depth[v] = max(depth[u] + 1)` for each incoming edge `(u, v)`.
fn compute_longest_path_depth(
    phases: &[Phase],
    transitions: &[PhaseTransition],
    initial: PhaseId,
) -> BTreeMap<PhaseId, usize> {
    // Build successor adjacency + in-degree.
    let mut successors: BTreeMap<PhaseId, Vec<PhaseId>> = BTreeMap::new();
    let mut in_degree: BTreeMap<PhaseId, usize> = BTreeMap::new();
    for phase in phases {
        successors.entry(phase.id).or_default();
        in_degree.entry(phase.id).or_insert(0);
    }
    for transition in transitions {
        successors
            .entry(transition.from)
            .or_default()
            .push(transition.to);
        *in_degree.entry(transition.to).or_insert(0) += 1;
    }

    // Initialise depth: `initial` at 0, others at 0 too — relaxation
    // monotonically lifts them. Phases unreachable from `initial`
    // would keep depth 0 but we've validated reachability above.
    let mut depth: BTreeMap<PhaseId, usize> = BTreeMap::new();
    for phase in phases {
        depth.insert(phase.id, 0);
    }
    depth.insert(initial, 0);

    // Kahn topological order, with `PhaseId.value()` tie-break for
    // determinism.
    let mut frontier: BTreeSet<PhaseId> = BTreeSet::new();
    for (id, &deg) in &in_degree {
        if deg == 0 {
            frontier.insert(*id);
        }
    }

    let mut visited = 0_usize;
    while let Some(&u) = frontier.iter().next() {
        frontier.remove(&u);
        visited += 1;
        if let Some(succs) = successors.get(&u) {
            let u_depth = depth[&u];
            // Visit successors in deterministic order.
            let mut succs_sorted = succs.clone();
            succs_sorted.sort_by_key(|p| p.value());
            for v in succs_sorted {
                let cand = u_depth + 1;
                if depth[&v] < cand {
                    depth.insert(v, cand);
                }
                let entry = in_degree.entry(v).or_insert(0);
                *entry = entry.saturating_sub(1);
                if *entry == 0 {
                    frontier.insert(v);
                }
            }
        }
    }
    debug_assert_eq!(visited, phases.len(), "topological order incomplete");

    depth
}

/// Kernel-side queue entry recorded per fired event.
#[derive(Clone, Debug, PartialEq)]
pub struct FiredEvent {
    /// Binding that fired.
    pub binding_id: EventId,
    /// Step at which the event fired.
    pub step: StepIndex,
    /// Simulation time at which the event fired.
    pub time: SimTime,
    /// Action to apply (cloned at fire time so the runner can drain
    /// without holding a borrow on the kernel).
    pub action: EventAction,
}

// ---------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------

/// Errors produced when constructing a [`MissionPhaseGraph`] or
/// validating event bindings against it.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum MissionGraphError {
    /// The transitions form a cycle. Phase 3.2's spec locks the graph
    /// as acyclic; cycles are deferred to a later mission-phase phase.
    #[error("mission graph contains a cycle involving phases {involving:?}")]
    Cycle {
        /// Phase ids participating in the cycle.
        involving: Vec<PhaseId>,
    },
    /// A phase is declared but unreachable from `initial`.
    #[error("phase {phase:?} is unreachable from the initial phase")]
    UnreachablePhase {
        /// Unreachable phase.
        phase: PhaseId,
    },
    /// A reference to a phase id that does not exist in the phase
    /// list. The `in_field` hint names the offending field.
    #[error("unknown phase id {phase:?} referenced in {in_field}")]
    UnknownPhaseId {
        /// Unknown phase id.
        phase: PhaseId,
        /// Where the unknown id appeared (e.g. `"transition.from"`,
        /// `"transition.to"`, `"action.enter_phase"`,
        /// `"mission.initial_phase"`).
        in_field: Cow<'static, str>,
    },
    /// Two phases share the same id.
    #[error("duplicate phase id {phase:?}")]
    DuplicatePhase {
        /// Duplicated phase id.
        phase: PhaseId,
    },
    /// Two event declarations share the same id.
    #[error("duplicate event id {event:?}")]
    DuplicateEvent {
        /// Duplicated event id.
        event: EventId,
    },
    /// More than one transition leaves the same phase on the same event.
    #[error("phase {from:?} has multiple transitions for event {event:?}")]
    AmbiguousTransition {
        /// Source phase.
        from: PhaseId,
        /// Event whose firing would be ambiguous.
        event: EventId,
    },
    /// `initial_phase` references an unknown id, or is otherwise
    /// missing.
    #[error("missing or unknown initial phase")]
    MissingInitial,
    /// A transition references an event that is not declared in the
    /// scenario `[[mission.events]]` list.
    #[error("transition #{in_transition} references unknown event {event:?}")]
    UnknownEvent {
        /// Unknown event id.
        event: EventId,
        /// Index of the offending transition in the `transitions` vec.
        in_transition: usize,
    },
}

#[cfg(test)]
mod fnv_tests {
    use super::*;

    #[test]
    fn phase_id_from_path_is_deterministic() {
        assert_eq!(
            PhaseId::from_path("mission.phases.ascent"),
            PhaseId::from_path("mission.phases.ascent"),
        );
    }

    #[test]
    fn phase_id_from_distinct_paths_diverges() {
        let a = PhaseId::from_path("mission.phases.ascent");
        let b = PhaseId::from_path("mission.phases.descent");
        assert_ne!(a, b);
    }

    #[test]
    fn event_id_from_path_is_deterministic() {
        assert_eq!(
            EventId::from_path("mission.events.at_apogee_marker"),
            EventId::from_path("mission.events.at_apogee_marker"),
        );
    }

    #[test]
    fn fnv1a_matches_sensor_id_for_known_path() {
        // Identical pinned FNV-1a-64 result as
        // `openbmp_core::SensorId::from_path("sensors.imu")` — see
        // `crates/openbmp-core/src/ids.rs:181`.
        let probe = PhaseId(fnv1a_64(b"sensors.imu"));
        assert_eq!(probe.value(), 0x8943_cc6e_91ce_20d3);
    }
}
