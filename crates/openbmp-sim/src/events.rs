//! Event triggers and mission-phase graph (Phase 3.2).
//!
//! Phase-3.2 introduces declarative event-driven scheduling that
//! replaces the Phase-2 hard-coded apogee detector. Scenarios declare
//! a `[mission]` block of phases, events, and transitions; the kernel
//! evaluates events post-step and either emits telemetry markers,
//! transitions the active phase, or halts the run.
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
//! See `docs/phase-3-plan.md § 3.2` for the contract; the Phase-3
//! plan locks the architectural shape.

use std::borrow::Cow;

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
    /// Dynamic pressure (Pa). NaN when no atmosphere model is wired —
    /// the parser rejects [`BuiltInEventTrigger::AtDynamicPressure`]
    /// in that case.
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
/// Evaluated once per kernel base tick after `integrator.advance()`
/// completes and before post-step state validation. Returning `true`
/// causes the kernel to record a [`FiredEvent`] for the binding;
/// the runner fans the fired events out to telemetry markers.
pub trait EventTrigger {
    /// Returns `true` if the trigger fires this step.
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
/// The four `Engine*` / `Effector*` / `Separation` / `DeployRecovery`
/// variants are reserved-but-unwired in Phase 3.2: scenarios that
/// declare them are rejected at parse time with a typed deferral
/// error pointing at the future phase that will land them.
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
    /// Halt the kernel with a [`crate::StopReason::MissionEnded`].
    Stop {
        /// Human-readable label for the stop reason.
        label: String,
    },
    // -------------------------------------------------------------
    // Phase-3.4 / 3.6 / 3.7 / 3.9 deferred actions. The variants
    // exist in the enum so 3.4+ does not need to expand it (which
    // would touch every match site); the parser rejects them in 3.2.
    // -------------------------------------------------------------
    /// Phase-3.6 deferred: gimbal / throttle / ignition / shutdown
    /// command to a named engine.
    EngineCommand,
    /// Phase-3.4 deferred: override a control-effector deflection.
    EffectorOverride,
    /// Phase-3.6 / 3.7 deferred: stage-separation event.
    Separation,
    /// Phase-3.9 deferred: deploy a recovery device.
    DeployRecovery,
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
/// declared but unenforced in Phase 3.2 — the kernel does not yet
/// consult them. Phase 3.4 / 3.6 will wire the enforcement.
#[derive(Clone, Debug, PartialEq)]
pub struct Phase {
    /// Path-derived stable id.
    pub id: PhaseId,
    /// Human-readable label for telemetry / diagnostics.
    pub label: String,
    /// Effectors permitted while this phase is active. Phase-3.4
    /// will enforce; Phase-3.2 leaves the list informational.
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
/// Phase 3.2.A ships the data shape and error type only; the
/// constructor with topological sort + reachability + cycle checks
/// lands in Phase 3.2.B. Until then the only construction path is
/// the public field assignment, which downstream callers should not
/// use directly — they should wait for `MissionPhaseGraph::new`.
#[derive(Clone, Debug, PartialEq)]
pub struct MissionPhaseGraph {
    /// Phases in canonical order. After 3.2.B: sorted by topological
    /// depth then `PhaseId.value()`.
    pub phases: Vec<Phase>,
    /// Transitions in canonical order. After 3.2.B: sorted by source
    /// phase depth then destination phase depth then `EventId.value()`.
    pub transitions: Vec<PhaseTransition>,
    /// Initial phase. Resolved at construction.
    pub initial: PhaseId,
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
