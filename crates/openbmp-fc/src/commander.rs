//! Commander — single state-machine owner.
//!
//! The commander is the only module that owns flight phase. Inputs are
//! bus topics (estimator status, failsafe flags, position estimate);
//! outputs are `commander.vehicle_status`. Builds on the Phase-3.2
//! [`MissionPhaseGraph`] from `openbmp-mission`.
//!
//! Phase transitions are evaluated against the
//! [`BuiltInEventTrigger`] of each
//! [`EventBinding`] supplied alongside the graph. The commander
//! evaluates events using the trigger's `EventTrigger::fired`
//! predicate against an [`EventEvalState`] it constructs from current
//! bus topics.

use std::collections::BTreeSet;

use openbmp_mission::{
    BuiltInEventTrigger, EventAction, EventBinding, EventEvalState, EventScalars, EventTrigger,
    MissionPhaseGraph, PhaseId,
};

use crate::bus::Bus;
use crate::error::{CommanderError, ControllerError};
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::topics::{
    BarometerSample, EstimatorStatus, FailsafeFlags, FdirStatus, GnssSample, ImuSample,
    PositionEstimate, VehicleStatus,
};

/// Commander parameters.
#[derive(Clone, Debug)]
pub struct CommanderParams {
    /// `true` if the commander should treat any
    /// [`FailsafeFlags::any`] as a hard arming-block.
    pub failsafe_blocks_arming: bool,
    /// Minimum altitude (m) before the commander can transition into
    /// the in-flight state from a pad-armed state.
    pub liftoff_altitude_m: f64,
    /// Minimum vertical velocity (m/s) before transitioning into the
    /// in-flight state.
    pub liftoff_velocity_m_s: f64,
}

impl Default for CommanderParams {
    fn default() -> Self {
        Self {
            failsafe_blocks_arming: true,
            liftoff_altitude_m: 1.0,
            liftoff_velocity_m_s: 1.0,
        }
    }
}

impl ParamSection for CommanderParams {
    const NAME: &'static str = "commander";
}

/// Commander job. Reads bus topics, evaluates the
/// [`MissionPhaseGraph`] + bindings, and publishes `VehicleStatus`
/// updates.
#[derive(Debug)]
pub struct Commander {
    graph: MissionPhaseGraph,
    bindings: Vec<EventBinding>,
    current_phase: PhaseId,
    params: CommanderParams,
    armed: bool,
    in_flight: bool,
    /// `true` once a tripped FDIR detector has demanded a safe-state
    /// transition. The flag latches; it is re-published as
    /// [`VehicleStatus::safe_state_requested`] every tick so a
    /// downstream scenario binding can consume it.
    safe_state_requested: bool,
    fired_once: BTreeSet<u64>,
    previous_scalars: Option<EventScalars>,
}

impl Commander {
    /// Constructs the commander with a given mission graph + bindings
    /// and starting phase. The starting phase must exist in the
    /// graph.
    ///
    /// # Errors
    ///
    /// Returns [`CommanderError::UnknownPhase`] if `start_phase` is
    /// not in the graph.
    pub fn new(
        graph: MissionPhaseGraph,
        bindings: Vec<EventBinding>,
        start_phase: PhaseId,
        params: CommanderParams,
    ) -> Result<Self, CommanderError> {
        if !graph.phases.iter().any(|p| p.id == start_phase) {
            return Err(CommanderError::UnknownPhase {
                phase_id: start_phase.value(),
            });
        }
        Ok(Self {
            graph,
            bindings,
            current_phase: start_phase,
            params,
            armed: false,
            in_flight: false,
            safe_state_requested: false,
            fired_once: BTreeSet::new(),
            previous_scalars: None,
        })
    }

    /// Returns the active phase.
    #[must_use]
    pub fn current_phase(&self) -> PhaseId {
        self.current_phase
    }

    /// Returns `true` if the commander has armed the vehicle.
    #[must_use]
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Returns the underlying mission graph.
    #[must_use]
    pub fn graph(&self) -> &MissionPhaseGraph {
        &self.graph
    }

    fn build_eval_state(&self, bus: &Bus) -> EventEvalState {
        let pos = bus
            .latest::<PositionEstimate>()
            .ok()
            .flatten()
            .map(|(p, _)| p);
        let _imu = bus.latest::<ImuSample>().ok().flatten().map(|(s, _)| s);
        let _gnss = bus.latest::<GnssSample>().ok().flatten().map(|(s, _)| s);
        let baro = bus
            .latest::<BarometerSample>()
            .ok()
            .flatten()
            .map(|(s, _)| s);

        let altitude_m = pos.map_or(0.0, |p| p.position_eci_m.z);
        let vertical_velocity_m_s = pos.map_or(0.0, |p| p.velocity_eci_m_s.z);
        let dynamic_pressure_pa = baro.map_or(0.0, |_| {
            0.5 * 1.225 * vertical_velocity_m_s * vertical_velocity_m_s
        });
        let mass_fraction = 1.0; // not currently estimated by the controller.

        let current = EventScalars {
            time_s: 0.0,
            altitude_m,
            vertical_velocity_m_s,
            mass_fraction,
            dynamic_pressure_pa,
        };

        EventEvalState {
            current,
            previous: self.previous_scalars,
            current_phase: Some(self.current_phase),
        }
    }

    fn try_arm(&mut self, bus: &Bus) {
        if self.armed {
            return;
        }
        if self.params.failsafe_blocks_arming
            && let Ok(Some((flags, _))) = bus.latest::<FailsafeFlags>()
            && flags.any()
        {
            return;
        }
        if let Ok(Some((status, _))) = bus.latest::<EstimatorStatus>()
            && (!status.initialized || status.dead_reckoning)
        {
            return;
        }
        if let Ok(Some((fdir, _))) = bus.latest::<FdirStatus>()
            && fdir.triggered
        {
            // FDIR trip blocks arming. The commander records that a
            // safe-state transition has been requested so a
            // scenario can declare a safe-state phase to fall through
            // into.
            self.safe_state_requested = true;
            return;
        }
        self.armed = true;
    }

    fn evaluate_fdir_safe_state(&mut self, bus: &Bus) {
        // While armed, if FDIR trips, request a safe state. The
        // scenario decides what phase that maps to via an
        // `EventTrigger::SafeStateRequested` binding (Phase 4.C).
        if self.armed
            && let Ok(Some((fdir, _))) = bus.latest::<FdirStatus>()
            && fdir.triggered
        {
            self.safe_state_requested = true;
        }
    }

    fn evaluate_liftoff(&mut self, bus: &Bus) {
        if self.in_flight || !self.armed {
            return;
        }
        if let Ok(Some((pos, _))) = bus.latest::<PositionEstimate>()
            && pos.position_eci_m.z >= self.params.liftoff_altitude_m
            && pos.velocity_eci_m_s.z >= self.params.liftoff_velocity_m_s
        {
            self.in_flight = true;
        }
    }
}

impl Job for Commander {
    fn name(&self) -> &'static str {
        "commander.tick"
    }

    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        let now = ctx.clock.now();
        let tick = ctx.clock.tick();
        let mut eval_state = self.build_eval_state(ctx.bus);
        eval_state.current.time_s = now.as_seconds();

        let mut transitions: Vec<(u64, PhaseId)> = Vec::new();
        let bindings_snapshot = self.bindings.clone();
        for binding in &bindings_snapshot {
            if binding.once && self.fired_once.contains(&binding.id.value()) {
                continue;
            }
            if BuiltInEventTrigger::fired(&binding.trigger, &eval_state, now, tick) {
                if binding.once {
                    self.fired_once.insert(binding.id.value());
                }
                if let EventAction::EnterPhase(target) = binding.action {
                    transitions.push((binding.id.value(), target));
                }
            }
        }
        for (_event_id, target) in transitions {
            // Validate transition against the graph.
            let allowed = self
                .graph
                .transitions
                .iter()
                .any(|t| t.from == self.current_phase && t.to == target);
            if allowed {
                self.current_phase = target;
            }
        }

        self.try_arm(ctx.bus);
        self.evaluate_fdir_safe_state(ctx.bus);
        self.evaluate_liftoff(ctx.bus);

        let status = VehicleStatus {
            phase_id: self.current_phase.value(),
            armed: self.armed,
            in_flight: self.in_flight,
            safe_state_requested: self.safe_state_requested,
        };
        let _ = ctx.bus.publish(status);

        self.previous_scalars = Some(eval_state.current);
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use openbmp_core::{SimTime, StepIndex};
    use openbmp_mission::{
        BuiltInEventTrigger, EventAction, EventBinding, EventId, MissionPhaseGraph, Phase, PhaseId,
        PhaseTransition,
    };

    use super::*;
    use crate::clock::SimulatedClock;

    fn build_graph() -> (MissionPhaseGraph, Vec<EventBinding>, PhaseId) {
        let pad = PhaseId::from_path("mission.phases.pad");
        let ascent = PhaseId::from_path("mission.phases.ascent");
        let liftoff = EventId::from_path("mission.events.liftoff");
        let phases = vec![
            Phase {
                id: pad,
                label: "pad".to_string(),
                allowed_effectors: Vec::new(),
                allowed_engines: Vec::new(),
            },
            Phase {
                id: ascent,
                label: "ascent".to_string(),
                allowed_effectors: Vec::new(),
                allowed_engines: Vec::new(),
            },
        ];
        let transitions = vec![PhaseTransition {
            from: pad,
            to: ascent,
            event: liftoff,
        }];
        let graph = MissionPhaseGraph::new(phases, transitions, pad, &[liftoff]).unwrap();
        let bindings = vec![EventBinding {
            id: liftoff,
            trigger: BuiltInEventTrigger::AtTime { time_s: 0.5 },
            action: EventAction::EnterPhase(ascent),
            once: true,
        }];
        (graph, bindings, pad)
    }

    fn fresh_bus() -> Bus {
        let bus = Bus::new();
        bus.register::<ImuSample>().unwrap();
        bus.register::<GnssSample>().unwrap();
        bus.register::<BarometerSample>().unwrap();
        bus.register::<EstimatorStatus>().unwrap();
        bus.register::<PositionEstimate>().unwrap();
        bus.register::<FailsafeFlags>().unwrap();
        bus.register::<FdirStatus>().unwrap();
        bus.register::<VehicleStatus>().unwrap();
        bus
    }

    #[test]
    fn fdir_trip_blocks_arming_and_sets_safe_state_request() {
        let (graph, bindings, pad) = build_graph();
        let mut commander =
            Commander::new(graph, bindings, pad, CommanderParams::default()).unwrap();

        let bus = fresh_bus();
        let clock = SimulatedClock::new();

        // Estimator is initialised so arming is otherwise allowed.
        bus.publish(EstimatorStatus {
            time: SimTime::ZERO,
            initialized: true,
            dead_reckoning: false,
            imu_chi2: 0.0,
            gnss_chi2: 0.0,
            baro_chi2: 0.0,
            mag_chi2: 0.0,
            star_tracker_chi2: 0.0,
            innovation_rejected: false,
        })
        .unwrap();
        // FDIR has tripped.
        bus.publish(FdirStatus {
            triggered: true,
            tripped_mask: 0b1,
            ticks_since_trip: 0,
        })
        .unwrap();

        clock.set(SimTime::ZERO, StepIndex::new(0));
        commander
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();
        let (status, _) = bus.latest::<VehicleStatus>().unwrap().unwrap();
        assert!(!status.armed, "FDIR trip should block arming");
        assert!(
            status.safe_state_requested,
            "FDIR trip should latch safe-state request"
        );
    }
}
