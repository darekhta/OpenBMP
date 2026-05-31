//! Commander — single state-machine owner.
//!
//! The commander is the only module that owns flight phase. Inputs are
//! bus topics (estimator status, failsafe flags, position estimate);
//! outputs are `commander.vehicle_status`. Builds on the
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
    AlarmCode, BuiltInEventTrigger, CanonicalRegionStates, CanonicalRegions, EventBinding,
    EventEvalState, EventId, EventScalars, EventTrigger, MissionAction, MissionPhaseGraph,
    MissionStateMachine, PhaseId, RegionId, RegionSet,
};

use crate::bus::Bus;
use crate::error::{CommanderError, ControllerError};
use crate::nav_metrics;
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::topics::{
    BarometerSample, CommsRegionStatePublish, EnvironmentEstimate,
    EstimatorRegimeRegionStatePublish, EstimatorStatus, FailsafeFlags, FdirStatus, GnssSample,
    GuidanceCutoff, HealthRegionStatePublish, ImuSample, MissionRegionStatePublish,
    MissionStatePublish, PositionEstimate, VehicleStatus,
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
    hsm: MissionStateMachine,
    regions: RegionSet,
    bindings: Vec<EventBinding<MissionAction>>,
    current_phase: PhaseId,
    params: CommanderParams,
    armed: bool,
    in_flight: bool,
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
        hsm: MissionStateMachine,
        regions: RegionSet,
        bindings: Vec<EventBinding<MissionAction>>,
        start_phase: PhaseId,
        params: CommanderParams,
    ) -> Result<Self, CommanderError> {
        if !graph.phases.iter().any(|p| p.id == start_phase) {
            return Err(CommanderError::UnknownPhase {
                phase_id: start_phase.value(),
            });
        }
        if !hsm.contains_state(start_phase) {
            return Err(CommanderError::UnknownPhase {
                phase_id: start_phase.value(),
            });
        }
        for phase in &graph.phases {
            if !hsm.contains_state(phase.id) {
                return Err(CommanderError::UnknownPhase {
                    phase_id: phase.id.value(),
                });
            }
        }
        let mut commander = Self {
            graph,
            hsm,
            regions,
            bindings,
            current_phase: start_phase,
            params,
            armed: false,
            in_flight: false,
            fired_once: BTreeSet::new(),
            previous_scalars: None,
        };
        commander.sync_mission_region();
        Ok(commander)
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

    /// Returns the active state of a canonical or custom region.
    #[must_use]
    pub fn region_state(&self, region: openbmp_mission::RegionId) -> Option<PhaseId> {
        self.regions.current_state(region)
    }

    fn build_eval_state(&self, bus: &Bus) -> EventEvalState {
        let pos = bus
            .latest::<PositionEstimate>()
            .ok()
            .flatten()
            .map(|(p, _)| p);
        let _imu = bus.latest::<ImuSample>().ok().flatten().map(|(s, _)| s);
        let _gnss = bus.latest::<GnssSample>().ok().flatten().map(|(s, _)| s);
        let _baro = bus
            .latest::<BarometerSample>()
            .ok()
            .flatten()
            .map(|(s, _)| s);

        let altitude_m = pos.map_or(0.0, |p| nav_metrics::altitude_m(p.position_eci_m));
        let vertical_velocity_m_s = pos.map_or(0.0, |p| {
            nav_metrics::vertical_velocity_m_s(p.position_eci_m, p.velocity_eci_m_s)
        });
        let velocity_m_s = pos.map_or(0.0, |p| p.velocity_eci_m_s.norm());
        let environment = bus
            .latest::<EnvironmentEstimate>()
            .ok()
            .flatten()
            .map(|(e, _)| e);
        let dynamic_pressure_pa = pos.as_ref().map_or(0.0, |p| {
            environment.map_or_else(
                || nav_metrics::dynamic_pressure_air_relative(p),
                |env| nav_metrics::dynamic_pressure_air_relative_with_density(p, env.density_kg_m3),
            )
        });
        let mass_fraction = 1.0; // not currently estimated by the controller.
        let guidance_time_to_go_s = bus
            .latest::<GuidanceCutoff>()
            .ok()
            .flatten()
            .map_or(f64::INFINITY, |(c, _)| c.time_to_go_s);

        let current = EventScalars {
            time_s: 0.0,
            altitude_m,
            vertical_velocity_m_s,
            velocity_m_s,
            mass_fraction,
            dynamic_pressure_pa,
            guidance_time_to_go_s,
        };

        EventEvalState {
            current,
            previous: self.previous_scalars,
            current_phase: Some(self.current_phase),
            relative_distances_m: std::collections::BTreeMap::new(),
            previous_relative_distances_m: Some(std::collections::BTreeMap::new()),
            relative_speeds_m_s: std::collections::BTreeMap::new(),
            previous_relative_speeds_m_s: Some(std::collections::BTreeMap::new()),
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
            self.request_safe_state();
            return;
        }
        self.armed = true;
    }

    fn evaluate_fdir_safe_state(&mut self, bus: &Bus) {
        // While armed, if FDIR trips, request a safe state. The
        // scenario decides what phase that maps to via an
        // `EventTrigger::SafeStateRequested` binding.
        if self.armed
            && let Ok(Some((fdir, _))) = bus.latest::<FdirStatus>()
            && fdir.triggered
        {
            self.request_safe_state();
        }
    }

    fn evaluate_liftoff(&mut self, bus: &Bus) {
        if self.in_flight || !self.armed {
            return;
        }
        if self.current_phase_allows_effectors() || self.current_phase_allows_engines() {
            self.in_flight = true;
            return;
        }
        if let Ok(Some((pos, _))) = bus.latest::<PositionEstimate>()
            && pos.position_eci_m.z >= self.params.liftoff_altitude_m
            && pos.velocity_eci_m_s.z >= self.params.liftoff_velocity_m_s
        {
            self.in_flight = true;
        }
    }

    fn current_phase_allows_effectors(&self) -> bool {
        self.graph
            .phases
            .iter()
            .find(|phase| phase.id == self.current_phase)
            .is_some_and(|phase| !phase.allowed_effectors.is_empty())
            || self
                .hsm
                .state(self.current_phase)
                .is_some_and(|state| !state.allowed_effectors.is_empty())
    }

    /// `true` when the current phase authorizes engine commands. A
    /// thrust-vector-controlled vehicle (gimballed engines, no aero
    /// effectors) lifts off and goes in-flight on its engines, so
    /// engine authority is a valid liftoff trigger alongside effector
    /// authority.
    fn current_phase_allows_engines(&self) -> bool {
        self.graph
            .phases
            .iter()
            .find(|phase| phase.id == self.current_phase)
            .is_some_and(|phase| !phase.allowed_engines.is_empty())
            || self
                .hsm
                .state(self.current_phase)
                .is_some_and(|state| !state.allowed_engines.is_empty())
    }

    fn request_safe_state(&mut self) {
        let _ = self.regions.set_current_state(
            CanonicalRegions::health(),
            CanonicalRegionStates::health_abort_requested(),
        );
    }

    fn safe_state_requested(&self) -> bool {
        matches!(
            self.regions.current_state(CanonicalRegions::health()),
            Some(state)
                if state == CanonicalRegionStates::health_abort_requested()
                    || state == CanonicalRegionStates::health_safed_on_fault()
        )
    }

    fn sync_mission_region(&mut self) {
        let _ = self
            .regions
            .set_current_state(CanonicalRegions::mission(), self.current_phase);
    }

    fn apply_transition(&mut self, target: PhaseId) {
        let from = self.current_phase;
        self.fire_hsm_exit_actions(from, target);
        self.current_phase = target;
        self.sync_mission_region();
        self.fire_hsm_entry_actions(from, target);
    }

    fn fire_hsm_exit_actions(&mut self, from: PhaseId, to: PhaseId) {
        // The FC commander owns mission state, but physical script
        // actions are intentionally not part of `MissionAction`. HSM
        // action lists here can only request mission-state changes or
        // telemetry markers; telemetry fan-out remains simulator-side.
        for state in self.hsm.exit_chain(from, to) {
            let actions: Vec<_> = self.hsm.on_exit_actions(state).to_vec();
            self.apply_hsm_actions(&actions);
        }
    }

    fn fire_hsm_entry_actions(&mut self, from: PhaseId, to: PhaseId) {
        for state in self.hsm.enter_chain(from, to) {
            let actions: Vec<_> = self.hsm.on_entry_actions(state).to_vec();
            self.apply_hsm_actions(&actions);
        }
    }

    fn apply_hsm_actions(&mut self, actions: &[MissionAction]) {
        for action in actions {
            match action {
                MissionAction::EnterState(target) => {
                    if self
                        .graph
                        .transitions
                        .iter()
                        .any(|t| t.from == self.current_phase && t.to == *target)
                    {
                        self.current_phase = *target;
                        self.sync_mission_region();
                    }
                }
                MissionAction::EmitTelemetryMarker { .. } | MissionAction::Stop { .. } => {}
                MissionAction::RaiseHealthAlarm { region, alarm } => {
                    self.handle_health_alarm(*region, *alarm);
                }
                MissionAction::RequestSafeState { .. } => {
                    self.request_safe_state();
                }
            }
        }
    }

    /// Demote the named orthogonal region in response to a fired
    /// [`MissionAction::RaiseHealthAlarm`]. Currently the commander
    /// only knows the `mission.regions.health` machine; non-health
    /// targets are silently ignored (a future sub-phase wires
    /// per-region demotion tables when additional health-like
    /// regions land). A zero `alarm` code is a no-op so a binding
    /// can be wired with a placeholder code without driving the
    /// region.
    fn handle_health_alarm(&mut self, region: RegionId, alarm: AlarmCode) {
        if alarm.value() == 0 {
            return;
        }
        if region == CanonicalRegions::health() {
            let _ = self.regions.set_current_state(
                CanonicalRegions::health(),
                CanonicalRegionStates::health_abort_requested(),
            );
        }
    }
}

impl Job for Commander {
    fn name(&self) -> &'static str {
        "commander.tick"
    }

    #[allow(clippy::too_many_lines)] // per-region publish loop expands the tick body.
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        let now = ctx.clock.now();
        let tick = ctx.clock.tick();
        let mut eval_state = self.build_eval_state(ctx.bus);
        eval_state.current.time_s = now.as_seconds();

        let mut transitions: Vec<(EventId, Option<PhaseId>)> = Vec::new();
        let bindings_snapshot = self.bindings.clone();
        for binding in &bindings_snapshot {
            if binding.once && self.fired_once.contains(&binding.id.value()) {
                continue;
            }
            if BuiltInEventTrigger::fired(&binding.trigger, &eval_state, now, tick) {
                if binding.once {
                    self.fired_once.insert(binding.id.value());
                }
                let action_target = match &binding.action {
                    MissionAction::EnterState(target) => Some(*target),
                    MissionAction::EmitTelemetryMarker { .. } | MissionAction::Stop { .. } => None,
                    MissionAction::RaiseHealthAlarm { region, alarm } => {
                        self.handle_health_alarm(*region, *alarm);
                        None
                    }
                    MissionAction::RequestSafeState { .. } => {
                        self.request_safe_state();
                        None
                    }
                };
                transitions.push((binding.id, action_target));
            }
        }
        for (event_id, action_target) in transitions {
            let graph_target = self
                .graph
                .transitions
                .iter()
                .find(|t| t.from == self.current_phase && t.event == event_id)
                .map(|t| t.to);
            if let Some(target) = graph_target {
                self.apply_transition(target);
            } else if let Some(target) = action_target {
                // Legacy direct-entry fallback for scenarios that use
                // `EnterState` as the transition declaration itself.
                let allowed = self
                    .graph
                    .transitions
                    .iter()
                    .any(|t| t.from == self.current_phase && t.to == target);
                if allowed {
                    self.apply_transition(target);
                }
            }
        }

        self.try_arm(ctx.bus);
        self.evaluate_fdir_safe_state(ctx.bus);
        self.evaluate_liftoff(ctx.bus);

        let status = VehicleStatus {
            phase_id: self.current_phase.value(),
            armed: self.armed,
            in_flight: self.in_flight,
            safe_state_requested: self.safe_state_requested(),
        };
        let _ = ctx.bus.publish(status);

        // Publish the single-source-of-truth
        // mission-state topic alongside the per-region topics. The
        // simulator subscribes to `commander.mission_state` for the
        // aggregate snapshot; consumers that only care about one
        // region (e.g. a health watchdog) subscribe to the
        // per-region topic to avoid parsing the aggregate.
        let mission_state_id = self
            .regions
            .current_state(CanonicalRegions::mission())
            .unwrap_or(self.current_phase)
            .value();
        let health_state_id = self
            .regions
            .current_state(CanonicalRegions::health())
            .unwrap_or_default()
            .value();
        let comms_state_id = self
            .regions
            .current_state(CanonicalRegions::comms())
            .unwrap_or_default()
            .value();
        let estimator_regime_state_id = self
            .regions
            .current_state(CanonicalRegions::estimator_regime())
            .unwrap_or_default()
            .value();
        let safe_state_requested = self.safe_state_requested();

        let mission_state = MissionStatePublish {
            mission_state_id,
            health_state_id,
            comms_state_id,
            estimator_regime_state_id,
            safe_state_requested,
        };
        let _ = ctx.bus.publish(mission_state);
        let _ = ctx.bus.publish(MissionRegionStatePublish {
            state_id: mission_state_id,
        });
        let _ = ctx.bus.publish(HealthRegionStatePublish {
            state_id: health_state_id,
        });
        let _ = ctx.bus.publish(CommsRegionStatePublish {
            state_id: comms_state_id,
        });
        let _ = ctx.bus.publish(EstimatorRegimeRegionStatePublish {
            state_id: estimator_regime_state_id,
        });

        self.previous_scalars = Some(eval_state.current);
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use openbmp_core::{SimTime, StepIndex};
    use openbmp_mission::{
        BuiltInEventTrigger, CanonicalRegionStates, CanonicalRegions, EventBinding, EventId,
        MissionAction, MissionPhaseGraph, MissionState, MissionStateMachine, Phase, PhaseId,
        PhaseTransition, Region, RegionSet,
    };

    use super::*;
    use crate::clock::SimulatedClock;

    fn build_graph() -> (MissionPhaseGraph, Vec<EventBinding<MissionAction>>, PhaseId) {
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
            action: MissionAction::EnterState(ascent),
            once: true,
        }];
        (graph, bindings, pad)
    }

    fn hsm_from_graph(graph: &MissionPhaseGraph) -> MissionStateMachine {
        let states = graph
            .phases
            .iter()
            .map(|phase| MissionState {
                id: phase.id,
                label: phase.label.clone(),
                parent: None,
                on_entry: Vec::new(),
                on_exit: Vec::new(),
                on_active: Vec::new(),
                allowed_effectors: phase.allowed_effectors.clone(),
                allowed_engines: phase.allowed_engines.clone(),
            })
            .collect();
        MissionStateMachine::new(states, graph.initial).unwrap()
    }

    fn regions_from_graph(graph: &MissionPhaseGraph) -> RegionSet {
        let mut regions = RegionSet::new();
        regions.insert(Region::new(CanonicalRegions::mission(), graph.clone()));
        regions.insert(
            Region::from_states(
                CanonicalRegions::health(),
                vec![
                    Phase {
                        id: CanonicalRegionStates::health_nominal(),
                        label: "nominal".into(),
                        allowed_effectors: Vec::new(),
                        allowed_engines: Vec::new(),
                    },
                    Phase {
                        id: CanonicalRegionStates::health_abort_requested(),
                        label: "abort_requested".into(),
                        allowed_effectors: Vec::new(),
                        allowed_engines: Vec::new(),
                    },
                ],
                CanonicalRegionStates::health_nominal(),
            )
            .unwrap(),
        );
        regions
    }

    fn commander_from_graph(
        graph: MissionPhaseGraph,
        bindings: Vec<EventBinding<MissionAction>>,
        start: PhaseId,
    ) -> Commander {
        let hsm = hsm_from_graph(&graph);
        let regions = regions_from_graph(&graph);
        Commander::new(
            graph,
            hsm,
            regions,
            bindings,
            start,
            CommanderParams::default(),
        )
        .unwrap()
    }

    fn fresh_bus() -> Bus {
        let bus = Bus::new();
        bus.register::<ImuSample>().unwrap();
        bus.register::<GnssSample>().unwrap();
        bus.register::<BarometerSample>().unwrap();
        bus.register::<EstimatorStatus>().unwrap();
        bus.register::<PositionEstimate>().unwrap();
        bus.register::<EnvironmentEstimate>().unwrap();
        bus.register::<FailsafeFlags>().unwrap();
        bus.register::<FdirStatus>().unwrap();
        bus.register::<VehicleStatus>().unwrap();
        bus.register::<MissionStatePublish>().unwrap();
        bus.register::<MissionRegionStatePublish>().unwrap();
        bus.register::<HealthRegionStatePublish>().unwrap();
        bus.register::<CommsRegionStatePublish>().unwrap();
        bus.register::<EstimatorRegimeRegionStatePublish>().unwrap();
        bus
    }

    fn publish_initialized_estimator(bus: &Bus) {
        bus.publish(EstimatorStatus {
            time: SimTime::ZERO,
            initialized: true,
            ..EstimatorStatus::default()
        })
        .unwrap();
    }

    #[test]
    fn fdir_trip_blocks_arming_and_sets_safe_state_request() {
        let (graph, bindings, pad) = build_graph();
        let mut commander = commander_from_graph(graph, bindings, pad);

        let bus = fresh_bus();
        let clock = SimulatedClock::new();

        // Estimator is initialised so arming is otherwise allowed.
        publish_initialized_estimator(&bus);
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
        assert_eq!(
            commander.region_state(CanonicalRegions::health()),
            Some(CanonicalRegionStates::health_abort_requested()),
            "safe-state request should be represented by the health region"
        );
    }

    #[test]
    fn effector_authorized_start_phase_enters_in_flight_without_vertical_motion() {
        let ascent = PhaseId::from_path("mission.phases.ascent");
        let graph = MissionPhaseGraph::new(
            vec![Phase {
                id: ascent,
                label: "ascent".to_string(),
                allowed_effectors: vec!["roll-torque".to_string()],
                allowed_engines: Vec::new(),
            }],
            Vec::new(),
            ascent,
            &[],
        )
        .unwrap();
        let mut commander = commander_from_graph(graph, Vec::new(), ascent);

        let bus = fresh_bus();
        let clock = SimulatedClock::new();
        publish_initialized_estimator(&bus);
        bus.publish(PositionEstimate {
            time: SimTime::ZERO,
            position_eci_m: nalgebra::Vector3::zeros(),
            velocity_eci_m_s: nalgebra::Vector3::zeros(),
            accel_bias_body_m_s2: nalgebra::Vector3::zeros(),
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
        assert!(status.armed);
        assert!(status.in_flight);
    }

    #[test]
    fn graph_transition_uses_fired_event_even_for_marker_action() {
        let pad = PhaseId::from_path("mission.phases.pad");
        let ascent = PhaseId::from_path("mission.phases.ascent");
        let liftoff = EventId::from_path("mission.events.liftoff");
        let graph = MissionPhaseGraph::new(
            vec![
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
            ],
            vec![PhaseTransition {
                from: pad,
                to: ascent,
                event: liftoff,
            }],
            pad,
            &[liftoff],
        )
        .unwrap();
        let bindings = vec![EventBinding {
            id: liftoff,
            trigger: BuiltInEventTrigger::AtTime { time_s: 0.5 },
            action: MissionAction::EmitTelemetryMarker {
                tag: "liftoff".to_string(),
            },
            once: true,
        }];
        let mut commander = commander_from_graph(graph, bindings, pad);
        let bus = fresh_bus();
        let clock = SimulatedClock::new();

        clock.set(SimTime::ZERO, StepIndex::ZERO);
        commander
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();
        assert_eq!(commander.current_phase(), pad);

        clock.set(SimTime::from_seconds(0.5), StepIndex::new(1));
        commander
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        assert_eq!(commander.current_phase(), ascent);
    }

    #[test]
    fn eval_state_dynamic_pressure_uses_air_relative_velocity() {
        let (graph, bindings, pad) = build_graph();
        let commander = commander_from_graph(graph, bindings, pad);
        let bus = fresh_bus();
        let surface_radius_m = 6_371_000.0;
        let omega = openbmp_physics::frames::WGS84_OMEGA_RAD_S;

        bus.publish(PositionEstimate {
            time: SimTime::ZERO,
            position_eci_m: nalgebra::Vector3::new(surface_radius_m, 0.0, 0.0),
            velocity_eci_m_s: nalgebra::Vector3::new(0.0, omega * surface_radius_m, 0.0),
            accel_bias_body_m_s2: nalgebra::Vector3::zeros(),
        })
        .unwrap();

        let corotating = commander.build_eval_state(&bus);
        assert!(
            corotating.current.dynamic_pressure_pa < 1.0,
            "surface-corotating vehicle should not see inertial-speed q: {}",
            corotating.current.dynamic_pressure_pa
        );

        bus.publish(PositionEstimate {
            time: SimTime::ZERO,
            position_eci_m: nalgebra::Vector3::new(surface_radius_m, 0.0, 0.0),
            velocity_eci_m_s: nalgebra::Vector3::new(0.0, omega * surface_radius_m + 300.0, 0.0),
            accel_bias_body_m_s2: nalgebra::Vector3::zeros(),
        })
        .unwrap();

        let flying = commander.build_eval_state(&bus);
        assert!(
            flying.current.dynamic_pressure_pa > 50_000.0,
            "300 m/s air-relative speed at sea level should produce real q: {}",
            flying.current.dynamic_pressure_pa
        );
    }

    #[test]
    fn eval_state_dynamic_pressure_uses_published_environment_density() {
        let (graph, bindings, pad) = build_graph();
        let commander = commander_from_graph(graph, bindings, pad);
        let bus = fresh_bus();
        let surface_radius_m = 6_371_000.0;
        let omega = openbmp_physics::frames::WGS84_OMEGA_RAD_S;

        bus.publish(PositionEstimate {
            time: SimTime::ZERO,
            position_eci_m: nalgebra::Vector3::new(surface_radius_m, 0.0, 0.0),
            velocity_eci_m_s: nalgebra::Vector3::new(0.0, omega * surface_radius_m + 300.0, 0.0),
            accel_bias_body_m_s2: nalgebra::Vector3::zeros(),
        })
        .unwrap();
        bus.publish(EnvironmentEstimate {
            time: SimTime::ZERO,
            density_kg_m3: 0.01,
        })
        .unwrap();

        let state = commander.build_eval_state(&bus);
        assert!(
            (state.current.dynamic_pressure_pa - 450.0).abs() < 1.0e-9,
            "commander should derive event q from the runtime atmosphere density: {}",
            state.current.dynamic_pressure_pa
        );
    }

    #[test]
    fn raise_health_alarm_binding_demotes_health_region() {
        use openbmp_mission::AlarmCode;

        let (graph, _legacy_bindings, pad) = build_graph();
        let alarm_event = EventId::from_path("mission.events.raise_alarm");
        let bindings = vec![EventBinding {
            id: alarm_event,
            trigger: BuiltInEventTrigger::AtTime { time_s: 0.5 },
            action: MissionAction::RaiseHealthAlarm {
                region: CanonicalRegions::health(),
                alarm: AlarmCode::new(1),
            },
            once: true,
        }];
        let mut commander = commander_from_graph(graph, bindings, pad);

        let bus = fresh_bus();
        let clock = SimulatedClock::new();
        publish_initialized_estimator(&bus);

        // First tick: trigger has not fired; health remains nominal.
        clock.set(SimTime::ZERO, StepIndex::ZERO);
        commander
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();
        let (status, _) = bus.latest::<VehicleStatus>().unwrap().unwrap();
        assert!(
            !status.safe_state_requested,
            "health region should be nominal before the alarm trigger fires"
        );

        // Second tick: trigger crosses 0.5 s; alarm fires and demotes
        // the health region to `abort_requested`. The aggregate
        // `MissionStatePublish` and the per-region health topic both
        // observe the new state.
        clock.set(SimTime::from_seconds(0.5), StepIndex::new(1));
        commander
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();
        let (status, _) = bus.latest::<VehicleStatus>().unwrap().unwrap();
        assert!(
            status.safe_state_requested,
            "RaiseHealthAlarm binding should demote the health region"
        );
        let (aggregate, _) = bus.latest::<MissionStatePublish>().unwrap().unwrap();
        assert_eq!(
            aggregate.health_state_id,
            CanonicalRegionStates::health_abort_requested().value(),
        );
        assert!(aggregate.safe_state_requested);
        let (health, _) = bus.latest::<HealthRegionStatePublish>().unwrap().unwrap();
        assert_eq!(
            health.state_id,
            CanonicalRegionStates::health_abort_requested().value(),
        );
    }

    #[test]
    fn request_safe_state_binding_demotes_health_region() {
        let (graph, _legacy_bindings, pad) = build_graph();
        let safe_event = EventId::from_path("mission.events.request_safe");
        let bindings = vec![EventBinding {
            id: safe_event,
            trigger: BuiltInEventTrigger::AtTime { time_s: 0.5 },
            action: MissionAction::RequestSafeState {
                reason: "scenario-driven safe state".into(),
            },
            once: true,
        }];
        let mut commander = commander_from_graph(graph, bindings, pad);

        let bus = fresh_bus();
        let clock = SimulatedClock::new();
        publish_initialized_estimator(&bus);

        clock.set(SimTime::ZERO, StepIndex::ZERO);
        commander
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();
        clock.set(SimTime::from_seconds(0.5), StepIndex::new(1));
        commander
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (status, _) = bus.latest::<VehicleStatus>().unwrap().unwrap();
        assert!(status.safe_state_requested);
        let (health, _) = bus.latest::<HealthRegionStatePublish>().unwrap().unwrap();
        assert_eq!(
            health.state_id,
            CanonicalRegionStates::health_abort_requested().value(),
        );
    }
}
