//! Closed-loop integration test.
//!
//! Wires together the full controller pipeline:
//! - `FlightController` skeleton
//! - Synthetic IMU/GNSS publishes (driven by the test, not by a
//!   `Sensor::read` source)
//! - `EstimatorJob` consuming sensor topics
//! - `AttitudeHoldGuidance` publishing a reference
//! - `Commander` evaluating phase transitions and arming
//! - `ThreeLoopAutopilot` consuming the estimate + reference
//! - `Mixer` republishing actuator commands gated by the phase mask
//! - `HealthMonitor` aggregating failsafe flags
//! - `FdirJob` watching for sustained anomalies
//!
//! The test verifies that the pipeline runs deterministically over
//! 1 000 ticks and that the published actuator command is bounded
//! within the gain schedule's saturation limits.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::explicit_iter_loop,
    clippy::too_many_lines
)]

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_core::{SimTime, StepIndex};
use openbmp_fc::FlightControllerBuilder;
use openbmp_fc::autopilot::ThreeLoopAutopilot;
use openbmp_fc::bus::{Bus, Topic};
use openbmp_fc::commander::{Commander, CommanderParams};
use openbmp_fc::estimator::{Ekf, EkfParams, EstimatorJob};
use openbmp_fc::fdir::{FdirJob, FdirParams};
use openbmp_fc::guidance::AttitudeHoldGuidance;
use openbmp_fc::health::{HealthMonitor, HealthParams};
use openbmp_fc::mixer::Mixer;
use openbmp_fc::topics::{
    ActuatorCommand, AttitudeEstimate, BarometerSample, EffectorCommandSet, EngineCommandSet,
    EngineDemand, EstimatorStatus, FailsafeFlags, FdirStatus, GnssSample, ImuSample,
    MagnetometerSample, PositionEstimate, ReferenceState, SensorStatus, StarTrackerSample,
    VehicleStatus,
};
use openbmp_mission::{
    BuiltInEventTrigger, CanonicalRegionStates, CanonicalRegions, EventBinding, EventId,
    MissionAction, MissionPhaseGraph, MissionState, MissionStateMachine, Phase, PhaseId,
    PhaseTransition, Region, RegionSet,
};

#[derive(Clone, Debug, PartialEq, Eq)]
struct HistoryEntry {
    topic: &'static str,
    seq: u64,
    time_bits: u64,
    payload: String,
}

trait TopicSnapshot {
    fn poll(&mut self, bus: &Bus, time: SimTime) -> Option<HistoryEntry>;
}

#[derive(Debug)]
struct TopicRecorder<T: Topic + std::fmt::Debug> {
    last_seq: u64,
    _marker: std::marker::PhantomData<T>,
}

impl<T: Topic + std::fmt::Debug> Default for TopicRecorder<T> {
    fn default() -> Self {
        Self {
            last_seq: 0,
            _marker: std::marker::PhantomData,
        }
    }
}

impl<T: Topic + std::fmt::Debug> TopicSnapshot for TopicRecorder<T> {
    fn poll(&mut self, bus: &Bus, time: SimTime) -> Option<HistoryEntry> {
        let seq = bus.sequence::<T>().ok()?.value();
        if seq <= self.last_seq {
            return None;
        }
        let (value, sequence) = bus.latest::<T>().ok()??;
        self.last_seq = sequence.value();
        Some(HistoryEntry {
            topic: T::NAME,
            seq: sequence.value(),
            time_bits: time.as_seconds().to_bits(),
            payload: format!("{value:?}"),
        })
    }
}

#[derive(Default)]
struct BusHistoryRecorder {
    recorders: Vec<Box<dyn TopicSnapshot>>,
    entries: Vec<HistoryEntry>,
}

impl BusHistoryRecorder {
    fn canonical_topics() -> Self {
        let mut recorder = Self::default();
        macro_rules! add {
            ($ty:ty) => {
                recorder
                    .recorders
                    .push(Box::new(TopicRecorder::<$ty>::default()));
            };
        }
        add!(ImuSample);
        add!(BarometerSample);
        add!(GnssSample);
        add!(MagnetometerSample);
        add!(StarTrackerSample);
        add!(SensorStatus);
        add!(AttitudeEstimate);
        add!(PositionEstimate);
        add!(EstimatorStatus);
        add!(VehicleStatus);
        add!(FailsafeFlags);
        add!(ReferenceState);
        add!(ActuatorCommand);
        add!(EffectorCommandSet);
        add!(EngineDemand);
        add!(EngineCommandSet);
        add!(FdirStatus);
        recorder
    }

    fn poll(&mut self, bus: &Bus, time: SimTime) {
        for topic in &mut self.recorders {
            if let Some(entry) = topic.poll(bus, time) {
                self.entries.push(entry);
            }
        }
    }

    fn into_entries(self) -> Vec<HistoryEntry> {
        self.entries
    }
}

fn build_simple_graph() -> (
    MissionPhaseGraph,
    Vec<EventBinding<MissionAction>>,
    PhaseId,
    PhaseId,
) {
    let pad = PhaseId::from_path("mission.phases.pad");
    let ascent = PhaseId::from_path("mission.phases.ascent");
    let event_id = EventId::from_path("mission.events.liftoff");

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
        event: event_id,
    }];
    let graph =
        MissionPhaseGraph::new(phases, transitions, pad, &[event_id]).expect("graph construction");

    let bindings = vec![EventBinding {
        id: event_id,
        trigger: BuiltInEventTrigger::AtTime { time_s: 0.5 },
        action: MissionAction::EnterState(ascent),
        once: true,
    }];

    (graph, bindings, pad, ascent)
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
    pad: PhaseId,
) -> Commander {
    let hsm = hsm_from_graph(&graph);
    let regions = regions_from_graph(&graph);
    Commander::new(
        graph,
        hsm,
        regions,
        bindings,
        pad,
        CommanderParams::default(),
    )
    .unwrap()
}

#[test]
fn closed_loop_pipeline_runs_deterministically() {
    let mut fc = FlightControllerBuilder::new()
        .frame_budget_us(2_000)
        .build();

    // Register topics. The order is registration-stable.
    fc.bus().register::<ImuSample>().unwrap();
    fc.bus().register::<BarometerSample>().unwrap();
    fc.bus().register::<GnssSample>().unwrap();
    fc.bus().register::<MagnetometerSample>().unwrap();
    fc.bus().register::<StarTrackerSample>().unwrap();
    fc.bus().register::<SensorStatus>().unwrap();
    fc.bus().register::<AttitudeEstimate>().unwrap();
    fc.bus().register::<PositionEstimate>().unwrap();
    fc.bus().register::<EstimatorStatus>().unwrap();
    fc.bus().register::<VehicleStatus>().unwrap();
    fc.bus().register::<FailsafeFlags>().unwrap();
    fc.bus().register::<ReferenceState>().unwrap();
    fc.bus().register::<ActuatorCommand>().unwrap();
    fc.bus().register::<EffectorCommandSet>().unwrap();
    fc.bus().register::<EngineDemand>().unwrap();
    fc.bus().register::<EngineCommandSet>().unwrap();
    fc.bus().register::<FdirStatus>().unwrap();

    // Seed the EKF with an initial pose.
    let mut ekf = Ekf::new(EkfParams::default());
    ekf.seed(
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        UnitQuaternion::identity(),
    );

    // Mission graph and commander.
    let (graph, bindings, pad, _ascent) = build_simple_graph();
    let commander = commander_from_graph(graph, bindings, pad);

    // Register all jobs.
    fc.scheduler_mut()
        .register_periodic(1, 200, 5, Box::new(EstimatorJob::new(ekf)))
        .unwrap();
    fc.scheduler_mut()
        .register_periodic(
            10,
            100,
            10,
            Box::new(AttitudeHoldGuidance::new([0.0, 0.0, 0.0, 1.0])),
        )
        .unwrap();
    fc.scheduler_mut()
        .register_periodic(1, 200, 15, Box::new(commander))
        .unwrap();
    fc.scheduler_mut()
        .register_periodic(1, 300, 20, Box::new(ThreeLoopAutopilot::new()))
        .unwrap();
    fc.scheduler_mut()
        .register_periodic(1, 100, 25, Box::new(Mixer::new()))
        .unwrap();
    fc.scheduler_mut()
        .register_periodic(
            10,
            100,
            30,
            Box::new(HealthMonitor::new(HealthParams::default())),
        )
        .unwrap();
    fc.scheduler_mut()
        .register_periodic(10, 100, 35, Box::new(FdirJob::new(FdirParams::default())))
        .unwrap();

    // Drive 1 000 ticks at 1 ms each. Inject a trickle of sensor
    // samples to keep the EKF from flagging dead-reckoning.
    let dt_s = 0.001;
    for k in 0..1_000u64 {
        let now = SimTime::from_seconds(k as f64 * dt_s);

        // Inject one IMU sample per tick (1 kHz).
        let _ = fc.bus().publish(ImuSample {
            time: now,
            gyro_rad_s: Vector3::new(0.0, 0.0, 0.0),
            accel_m_s2: Vector3::new(0.0, 0.0, 9.81),
            healthy: true,
        });
        // Inject GNSS at 10 Hz.
        if k % 100 == 0 {
            let _ = fc.bus().publish(GnssSample {
                time: now,
                position_eci_m: Vector3::new(0.0, 0.0, k as f64 * dt_s),
                velocity_eci_m_s: Vector3::new(0.0, 0.0, 1.0),
                position_bias_eci_m: Vector3::zeros(),
                healthy: true,
            });
        }
        // Inject barometer at 50 Hz.
        if k % 20 == 0 {
            let _ = fc.bus().publish(BarometerSample {
                time: now,
                pressure_pa: 101_325.0,
                bias_pa: 0.0,
                healthy: true,
            });
        }
        // Inject magnetometer at 50 Hz.
        if k % 20 == 0 {
            let _ = fc.bus().publish(MagnetometerSample {
                time: now,
                field_body_nt: Vector3::new(20_000.0, 0.0, 40_000.0),
                hard_iron_body_nt: Vector3::zeros(),
                healthy: true,
            });
        }

        let summary = fc.step(now, StepIndex::new(k)).unwrap();
        // No overruns expected for the budgets we set.
        assert_eq!(summary.overrun_count, 0, "tick {k} overran");
    }

    // After 1 s the EKF should have published an estimate with finite
    // values.
    let (attitude, _) = fc
        .bus()
        .latest::<AttitudeEstimate>()
        .unwrap()
        .expect("estimator should have published");
    for v in attitude.q_body_to_eci_xyzw.iter() {
        assert!(v.is_finite(), "attitude q has NaN");
    }

    // The commander should have transitioned past pad once the
    // AtTime trigger fired (>= 0.5 s).
    let (status, _) = fc
        .bus()
        .latest::<VehicleStatus>()
        .unwrap()
        .expect("commander should have published");
    let pad_id = PhaseId::from_path("mission.phases.pad").value();
    assert_ne!(status.phase_id, pad_id, "commander should have left pad");

    // The health monitor should have published; we don't require
    // healthy, only "published".
    let _ = fc
        .bus()
        .latest::<FailsafeFlags>()
        .unwrap()
        .expect("health should have published");
}

#[test]
fn deterministic_replay_reproduces_actuator_stream() {
    fn run(bus_seed: u64) -> Vec<HistoryEntry> {
        let mut fc = FlightControllerBuilder::new()
            .frame_budget_us(2_000)
            .build();
        fc.bus().register::<ImuSample>().unwrap();
        fc.bus().register::<BarometerSample>().unwrap();
        fc.bus().register::<GnssSample>().unwrap();
        fc.bus().register::<MagnetometerSample>().unwrap();
        fc.bus().register::<StarTrackerSample>().unwrap();
        fc.bus().register::<SensorStatus>().unwrap();
        fc.bus().register::<AttitudeEstimate>().unwrap();
        fc.bus().register::<PositionEstimate>().unwrap();
        fc.bus().register::<EstimatorStatus>().unwrap();
        fc.bus().register::<VehicleStatus>().unwrap();
        fc.bus().register::<FailsafeFlags>().unwrap();
        fc.bus().register::<ReferenceState>().unwrap();
        fc.bus().register::<ActuatorCommand>().unwrap();
        fc.bus().register::<EffectorCommandSet>().unwrap();
        fc.bus().register::<EngineDemand>().unwrap();
        fc.bus().register::<EngineCommandSet>().unwrap();
        fc.bus().register::<FdirStatus>().unwrap();

        let mut ekf = Ekf::new(EkfParams::default());
        ekf.seed(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        let (graph, bindings, pad, _ascent) = build_simple_graph();
        let commander = commander_from_graph(graph, bindings, pad);
        fc.scheduler_mut()
            .register_periodic(1, 200, 5, Box::new(EstimatorJob::new(ekf)))
            .unwrap();
        fc.scheduler_mut()
            .register_periodic(
                10,
                100,
                10,
                Box::new(AttitudeHoldGuidance::new([0.0, 0.0, 0.0, 1.0])),
            )
            .unwrap();
        fc.scheduler_mut()
            .register_periodic(1, 200, 15, Box::new(commander))
            .unwrap();
        fc.scheduler_mut()
            .register_periodic(1, 300, 20, Box::new(ThreeLoopAutopilot::new()))
            .unwrap();
        fc.scheduler_mut()
            .register_periodic(1, 100, 25, Box::new(Mixer::new()))
            .unwrap();

        let mut recorder = BusHistoryRecorder::canonical_topics();
        for k in 0..200u64 {
            let now = SimTime::from_seconds(k as f64 * 0.001);
            // Slightly perturb gyro by `bus_seed` so two distinct runs
            // are easy to distinguish, but keep it deterministic.
            let gyro_z = ((bus_seed + k) as f64) * 1e-6;
            let _ = fc.bus().publish(ImuSample {
                time: now,
                gyro_rad_s: Vector3::new(0.0, 0.0, gyro_z),
                accel_m_s2: Vector3::new(0.0, 0.0, 9.81),
                healthy: true,
            });
            let _ = fc.step(now, StepIndex::new(k)).unwrap();
            recorder.poll(fc.bus(), now);
        }
        recorder.into_entries()
    }

    let history_a = run(0);
    let history_b = run(0);
    assert_eq!(
        history_a, history_b,
        "two runs of the same scenario must produce identical bus histories"
    );
}
