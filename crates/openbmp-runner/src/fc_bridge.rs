//! Kernel-side flight-controller bridge.
//!
//! `fc.rs` constructs a standalone controller from a parsed `[fc]`
//! block. This module is the runtime bridge between a kernel run and
//! that controller: it owns synthetic sensor adapters, translates the
//! kernel state into `openbmp-sensors::SensorTruth`, steps the FC once
//! per kernel tick, then pushes gated FC commands back into the
//! runner-side effector / engine racks.

use std::collections::BTreeMap;
use std::fmt;
use std::net::TcpStream;
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
#[cfg(unix)]
use std::sync::atomic::{AtomicU64, Ordering};

use nalgebra::{UnitQuaternion, Vector3};
#[cfg(unix)]
use openbmp_bridge::UnixBridgeListener;
use openbmp_bridge::{
    ActuatorCommandPacket, BridgeEndpointRole, BridgeFaultRule, BridgeFaultTransformSet,
    BridgeHelloPacket, BridgeMessage, BridgePacketDirection, BridgePacketDisposition,
    BridgePacketFaultRule, BridgePacketTransform, BridgeScalarSignal, BridgeScalarTransform,
    ImuIncrementPacket, QuaternionAxis, SensorPacket, SplitStreamTransport, StreamTransport,
    TcpBridgeListener, Transport, VectorAxis, in_process_transport_pair, validate_ack_for_sensor,
    validate_command_for_sensor, validate_protocol_version,
};
use openbmp_comm::{LinkPacketDisposition, LinkPacketEffect};
use openbmp_core::{DeterministicRng, Position3, SensorId, StepIndex, Velocity3};
use openbmp_fc::topics::{
    AirDataSample, BarometerSample, EffectorCommand, EffectorCommandSet, EngineCommand,
    EngineCommandSet, EnvironmentEstimate, GnssSample, ImuIncrementWindow, ImuInertialIncrement,
    ImuSample, MagnetometerSample, PropellantState, StarTrackerSample,
};
pub(crate) use openbmp_fc::topics::{GuidanceCutoff, ReferenceState};
use openbmp_mission::{FiredEvent, MissionAction};
use openbmp_physics::atmosphere::{AtmosphereModel, ExoatmosphericPolicy, UsStandard1976};
use openbmp_physics::magnetic::{EarthDipoleField, MagneticFieldEci, Wmm2025};
use openbmp_physics::{FrameContext, stationary_rotating_frame_imu_truth_eci};
use openbmp_scenario::{
    FcConfig, FcEstimatorKind, FcMagFieldKind, FcSilFaultKind, FcTransportFaultSignalConfig,
    FcTransportFaultTransformConfig, FcTransportModeConfig, FcTransportPacketDirectionConfig,
    FcTransportPacketTransformConfig, FcTransportQuaternionAxisConfig, FcTransportVectorAxisConfig,
    ResolvedFile, Scenario, ScenarioDocument, SensorConfig, SpecificForceSourceConfig,
};
use openbmp_sensors::{
    AirDataNoiseBudget, GnssNoiseBudget, HighRateImuConfig, IdealStateSensor, ImuNoiseBudget,
    MagnetometerNoiseBudget, MeasurementStimulus, Sensor as SensorTrait, SensorMeasurement,
    SensorTruth, SpecificForceTruth, StarTrackerNoiseBudget, SyntheticAirData, SyntheticBarometer,
    SyntheticGnss, SyntheticImu, SyntheticMagnetometer, SyntheticSensorAdapter,
    SyntheticStarTracker,
};
use openbmp_state::{PointMassState, RigidBodyState};

use crate::atmosphere::{
    RuntimeAtmosphere, atmosphere_altitude_m_with_surface_radius,
    build_document_runtime_atmosphere, document_geocentric_surface_radius_m,
    is_runtime_atmosphere_kind, scenario_atmosphere_kind,
};
use crate::error::RunnerError;
use crate::fc::{EstimatorSeed, FcAutopilotLqrContext, FcRunner, FcRunnerMission};

const DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR: f64 = 2025.0;
const DEFAULT_FC_TRANSPORT_MAX_PAYLOAD_LEN: u32 = 4096;
#[cfg(unix)]
static UNIX_LOOPBACK_SOCKET_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Optional FC bridge. Absent when the scenario has no `[fc]` block.
#[derive(Debug)]
pub struct FcBridge {
    runner: FcRunner,
    sensors: Vec<BridgeSensor>,
    atmosphere: Option<RuntimeAtmosphere>,
    fallback_atmosphere: UsStandard1976,
    geocentric_surface_radius_m: Option<f64>,
    magnetic: Box<dyn MagneticFieldEci>,
    frame: FrameContext,
    specific_force_source: SpecificForceSourceConfig,
    scenario_seed: u64,
    stimulus_schedule: StimulusSchedule,
    outage_windows: Vec<ArmedOutage>,
    previous_velocity_eci_m_s: Option<Vector3<f64>>,
    previous_time_s: Option<f64>,
    previous_angular_velocity_body_rad_s: Option<Vector3<f64>>,
    previous_angular_time_s: Option<f64>,
    last_mission_action_sequence: Option<u64>,
    transport_session: Option<FcTransportSession>,
    transport_faults: BridgeFaultTransformSet,
    comm_link_effects: Option<crate::comm::CommBridgePacketEffects>,
    comm_frame_queues: CommLinkFrameQueues,
    bridge_dt_s: f64,
    actuator_stream: Vec<ActuatorCommandPacket>,
}

/// One armed, open-loop sensor fault: a measurement-domain transform that
/// applies to a single sensor while the simulation clock is inside
/// `[start_s, stop_s)`.
#[derive(Debug)]
struct ArmedFault {
    sensor_id: SensorId,
    start_s: f64,
    stop_s: f64,
    stimulus: MeasurementStimulus,
    /// Distinguishes this fault's RNG stream from sibling faults on the
    /// same sensor so adding a fault never shifts another's draws.
    component_id: u32,
}

/// The scenario-declared SIL fault schedule, frozen before the run.
///
/// Empty for any scenario without an `[fc.sil_stimulus]` block, in which
/// case [`Self::apply`] is a no-op and draws nothing — the run is
/// byte-identical to an unstimulated run.
#[derive(Debug, Default)]
struct StimulusSchedule {
    faults: Vec<ArmedFault>,
}

/// One armed, open-loop sensor outage: while the simulation clock is
/// inside `[start_s, stop_s)` the named sensor is still primed and read
/// every tick (its noise-stream draws stay byte-identical), but the
/// measurement is withheld from the flight controller — the
/// communications-loss twin of the measurement-domain [`ArmedFault`].
#[derive(Debug)]
struct ArmedOutage {
    sensor_id: SensorId,
    start_s: f64,
    stop_s: f64,
}

struct FcTransportSession {
    simulator: Box<dyn Transport>,
    controller_peer: FcControllerPeer,
    mode: &'static str,
    max_payload_len: u32,
    peer_protocol_version: u16,
    handshaken: bool,
}

#[derive(Debug, Default)]
struct CommLinkFrameQueues {
    sensors: BTreeMap<u64, Vec<SensorPacket>>,
    commands: BTreeMap<u64, Vec<ActuatorCommandPacket>>,
}

enum FcControllerPeer {
    Local(Box<dyn Transport>),
    ExternalProcess(ExternalFcProcess),
}

struct ExternalFcProcess {
    child: Child,
}

impl Drop for ExternalFcProcess {
    fn drop(&mut self) {
        if matches!(self.child.try_wait(), Ok(None)) {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

impl FcTransportSession {
    fn new_in_process(max_payload_len: u32, peer_protocol_version: u16) -> Self {
        let (simulator, controller) = in_process_transport_pair();
        Self {
            simulator: Box::new(simulator),
            controller_peer: FcControllerPeer::Local(Box::new(controller)),
            mode: "in_process",
            max_payload_len,
            peer_protocol_version,
            handshaken: false,
        }
    }

    fn new_tcp_loopback(
        max_payload_len: u32,
        peer_protocol_version: u16,
    ) -> Result<Self, RunnerError> {
        let max_payload_len_usize = usize::try_from(max_payload_len).unwrap_or(usize::MAX);
        let listener = TcpBridgeListener::bind("127.0.0.1:0")?;
        let addr = listener.local_addr()?;
        let controller_stream =
            TcpStream::connect(addr).map_err(openbmp_bridge::BridgeError::Io)?;
        controller_stream
            .set_nodelay(true)
            .map_err(openbmp_bridge::BridgeError::Io)?;
        let (simulator_transport, _) = listener.accept()?;
        let simulator_stream = simulator_transport.into_inner();
        simulator_stream
            .set_nodelay(true)
            .map_err(openbmp_bridge::BridgeError::Io)?;
        Ok(Self {
            simulator: Box::new(StreamTransport::with_max_payload_len(
                simulator_stream,
                max_payload_len_usize,
            )),
            controller_peer: FcControllerPeer::Local(Box::new(
                StreamTransport::with_max_payload_len(controller_stream, max_payload_len_usize),
            )),
            mode: "tcp_loopback",
            max_payload_len,
            peer_protocol_version,
            handshaken: false,
        })
    }

    #[cfg(unix)]
    fn new_unix_loopback(
        max_payload_len: u32,
        peer_protocol_version: u16,
    ) -> Result<Self, RunnerError> {
        let max_payload_len_usize = usize::try_from(max_payload_len).unwrap_or(usize::MAX);
        let socket_path = std::env::temp_dir().join(format!(
            "openbmp-fc-{}-{}.sock",
            std::process::id(),
            UNIX_LOOPBACK_SOCKET_COUNTER.fetch_add(1, Ordering::Relaxed)
        ));
        let _ = std::fs::remove_file(&socket_path);
        let listener = UnixBridgeListener::bind(&socket_path)?;
        let controller_stream = StreamTransport::connect_unix(listener.path())?.into_inner();
        let simulator_stream = listener.accept()?.into_inner();
        let _ = std::fs::remove_file(listener.path());
        Ok(Self {
            simulator: Box::new(StreamTransport::with_max_payload_len(
                simulator_stream,
                max_payload_len_usize,
            )),
            controller_peer: FcControllerPeer::Local(Box::new(
                StreamTransport::with_max_payload_len(controller_stream, max_payload_len_usize),
            )),
            mode: "unix_loopback",
            max_payload_len,
            peer_protocol_version,
            handshaken: false,
        })
    }

    fn new_external_process(
        command: &str,
        args: &[String],
        working_dir: Option<&std::path::Path>,
        max_payload_len: u32,
        peer_protocol_version: u16,
    ) -> Result<Self, RunnerError> {
        let max_payload_len_usize = usize::try_from(max_payload_len).unwrap_or(usize::MAX);
        let mut process = Command::new(command);
        process
            .args(args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit());
        if let Some(working_dir) = working_dir {
            process.current_dir(working_dir);
        }
        let mut child = process
            .spawn()
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("fc.transport external_process spawn failed for `{command}`: {err}"),
            })?;
        let child_stdin = child
            .stdin
            .take()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "fc.transport external_process child stdin was not piped".to_owned(),
            })?;
        let child_stdout = child
            .stdout
            .take()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "fc.transport external_process child stdout was not piped".to_owned(),
            })?;
        Ok(Self {
            simulator: Box::new(
                SplitStreamTransport::<ChildStdout, ChildStdin>::with_max_payload_len(
                    child_stdout,
                    child_stdin,
                    max_payload_len_usize,
                ),
            ),
            controller_peer: FcControllerPeer::ExternalProcess(ExternalFcProcess { child }),
            mode: "external_process",
            max_payload_len,
            peer_protocol_version,
            handshaken: false,
        })
    }

    fn local_controller_mut(&mut self) -> Result<&mut dyn Transport, RunnerError> {
        match &mut self.controller_peer {
            FcControllerPeer::Local(controller) => Ok(controller.as_mut()),
            FcControllerPeer::ExternalProcess(_) => Err(RunnerError::UnsupportedScenario {
                what: "fc.transport external_process has no local controller endpoint".to_owned(),
            }),
        }
    }

    fn has_external_controller(&self) -> bool {
        matches!(self.controller_peer, FcControllerPeer::ExternalProcess(_))
    }

    fn ensure_handshake(&mut self) -> Result<(), RunnerError> {
        if self.handshaken {
            return Ok(());
        }

        self.simulator
            .send(&BridgeMessage::Hello(BridgeHelloPacket::new(
                BridgeEndpointRole::Simulator,
                self.max_payload_len,
            )))?;
        if matches!(self.controller_peer, FcControllerPeer::Local(_)) {
            let BridgeMessage::Hello(sim_hello) = self.local_controller_mut()?.recv()? else {
                return Err(RunnerError::UnsupportedScenario {
                    what: "fc.transport expected simulator hello during local handshake".to_owned(),
                });
            };
            validate_protocol_version(sim_hello.protocol_version).map_err(|err| {
                RunnerError::UnsupportedScenario {
                    what: format!("fc.transport protocol validation failed: {err}"),
                }
            })?;
            if sim_hello.role != BridgeEndpointRole::Simulator {
                return Err(RunnerError::UnsupportedScenario {
                    what: "fc.transport local handshake expected simulator role".to_owned(),
                });
            }

            let max_payload_len = self.max_payload_len;
            let peer_protocol_version = self.peer_protocol_version;
            self.local_controller_mut()?
                .send(&BridgeMessage::Hello(BridgeHelloPacket {
                    protocol_version: peer_protocol_version,
                    role: BridgeEndpointRole::FlightController,
                    max_payload_len,
                }))?;
        }
        let BridgeMessage::Hello(fc_hello) = self.simulator.recv()? else {
            return Err(RunnerError::UnsupportedScenario {
                what: "fc.transport expected controller hello during local handshake".to_owned(),
            });
        };
        validate_protocol_version(fc_hello.protocol_version).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("fc.transport protocol validation failed: {err}"),
            }
        })?;
        if fc_hello.role != BridgeEndpointRole::FlightController {
            return Err(RunnerError::UnsupportedScenario {
                what: "fc.transport local handshake expected flight-controller role".to_owned(),
            });
        }

        self.handshaken = true;
        Ok(())
    }
}

impl CommLinkFrameQueues {
    fn enqueue_sensor(&mut self, arrival_step: u64, sensor: SensorPacket) {
        self.sensors.entry(arrival_step).or_default().push(sensor);
    }

    fn enqueue_command(&mut self, arrival_step: u64, command: ActuatorCommandPacket) {
        self.commands.entry(arrival_step).or_default().push(command);
    }

    fn pop_due_sensor(&mut self, current_step: u64) -> Option<SensorPacket> {
        pop_due_packet(&mut self.sensors, current_step)
    }

    fn pop_due_command(&mut self, current_step: u64) -> Option<ActuatorCommandPacket> {
        pop_due_packet(&mut self.commands, current_step)
    }
}

fn pop_due_packet<T>(queue: &mut BTreeMap<u64, Vec<T>>, current_step: u64) -> Option<T> {
    let arrival_step = *queue.keys().next()?;
    if arrival_step > current_step {
        return None;
    }
    let mut packets = queue.remove(&arrival_step)?;
    let packet = packets.remove(0);
    if !packets.is_empty() {
        queue.insert(arrival_step, packets);
    }
    Some(packet)
}

impl fmt::Debug for FcTransportSession {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FcTransportSession")
            .field("mode", &self.mode)
            .field("max_payload_len", &self.max_payload_len)
            .field("peer_protocol_version", &self.peer_protocol_version)
            .field("handshaken", &self.handshaken)
            .finish_non_exhaustive()
    }
}

impl fmt::Debug for FcControllerPeer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Local(_) => f.write_str("Local(..)"),
            Self::ExternalProcess(process) => {
                f.debug_tuple("ExternalProcess").field(process).finish()
            }
        }
    }
}

impl fmt::Debug for ExternalFcProcess {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ExternalFcProcess")
            .field("id", &self.child.id())
            .finish()
    }
}

impl StimulusSchedule {
    /// Build the schedule from the scenario `[fc.sil_stimulus]` block,
    /// failing closed if a fault names a sensor absent from `[sensors]`.
    fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let mut faults = Vec::new();
        let Some(fc) = &document.fc else {
            return Ok(Self::default());
        };
        let Some(config) = &fc.sil_stimulus else {
            return Ok(Self::default());
        };
        for (index, fault) in config.faults.iter().enumerate() {
            let declared = document
                .sensors
                .as_ref()
                .is_some_and(|sensors| sensors.contains_key(&fault.sensor));
            if !declared {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!(
                        "[fc.sil_stimulus] fault targets sensor `{}`, which is not declared in [sensors]",
                        fault.sensor
                    ),
                });
            }
            if fault.stop_s <= fault.start_s {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!(
                        "[fc.sil_stimulus] fault on `{}` requires stop_s > start_s (got start_s = {}, stop_s = {})",
                        fault.sensor, fault.start_s, fault.stop_s
                    ),
                });
            }
            let stimulus = match &fault.fault {
                FcSilFaultKind::Bias { offset } => MeasurementStimulus::Bias {
                    offset: Vector3::from(*offset),
                },
            };
            faults.push(ArmedFault {
                sensor_id: SensorId::from_path(&format!("sensors.{}", fault.sensor)),
                start_s: fault.start_s,
                stop_s: fault.stop_s,
                stimulus,
                component_id: u32::try_from(index).unwrap_or(u32::MAX),
            });
        }
        Ok(Self { faults })
    }

    /// Whether any fault is scheduled. When empty, the seam is an identity.
    fn is_empty(&self) -> bool {
        self.faults.is_empty()
    }

    /// Apply every armed fault for `sensor_id` whose window contains
    /// `time_s`, in declaration order, mutating the freshly-read
    /// measurement before the flight controller sees it.
    fn apply(
        &self,
        sensor_id: SensorId,
        measurement: &mut SensorMeasurement,
        step: StepIndex,
        time_s: f64,
        scenario_seed: u64,
    ) {
        for fault in &self.faults {
            if fault.sensor_id == sensor_id && time_s >= fault.start_s && time_s < fault.stop_s {
                let mut rng = DeterministicRng::for_stimulus_component(
                    scenario_seed,
                    step,
                    sensor_id,
                    fault.component_id,
                );
                fault.stimulus.apply(measurement, &mut rng);
            }
        }
    }
}

impl FcBridge {
    /// Build the bridge when `[fc]` is present; otherwise return
    /// `None` so legacy scenarios remain byte-stable.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when the FC config, mission graph, frame
    /// profile, or synthetic sensor config cannot be resolved.
    pub fn maybe_new(
        scenario: &Scenario,
        resolved_files: &BTreeMap<String, ResolvedFile>,
    ) -> Result<Option<Self>, RunnerError> {
        let Some(fc_config) = &scenario.document.fc else {
            return Ok(None);
        };
        require_bridge_frame(&scenario.document)?;
        if scenario.document.mission.is_none() {
            return Err(RunnerError::UnsupportedScenario {
                what:
                    "[fc] requires a [mission] graph so the FC commander and kernel share phase ids"
                        .to_owned(),
            });
        }
        let mission_runtime = crate::mission::build_mission_runtime_from_document(
            &scenario.document,
        )?
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "[fc] requires a [mission] graph so the FC commander and kernel share phase ids"
                .to_owned(),
        })?;
        let start_phase = mission_runtime.graph.initial;
        let fc_mission = FcRunnerMission::new(
            mission_runtime.graph,
            mission_runtime.hsm,
            mission_runtime.regions,
            mission_runtime.mission_bindings,
            start_phase,
        );
        let magnetic = build_magnetic_field(fc_config)?;
        let frame = crate::frames::build_frame_context(&scenario.document, resolved_files)?;
        let lqr_ctx = build_autopilot_lqr_context(scenario)?;
        let allocator = build_autopilot_allocator(scenario)?;
        // Seed the estimator from the scenario's known initial state so
        // the navigation filter starts near truth. Without this the EKF
        // seeds at the ECI origin and its first GNSS fix lands outside
        // the innovation gate (the vehicle is thousands of km away), so
        // it rejects every correction and dead-reckons from zero — any
        // guidance keyed off the position estimate is then wrong for the
        // whole flight.
        let v = &scenario.document.vehicle;
        let estimator_seed = EstimatorSeed {
            position_eci_m: Vector3::from(v.initial_position_eci_m),
            velocity_eci_m_s: Vector3::from(v.initial_velocity_eci_m_s),
            attitude_body_to_eci: v.initial_quaternion_body_to_eci_xyzw.map_or_else(
                UnitQuaternion::identity,
                |q| {
                    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        q[3], q[0], q[1], q[2],
                    ))
                },
            ),
        };
        let runner = FcRunner::new(
            fc_config,
            fc_mission,
            lqr_ctx,
            scenario.document.time.dt_s,
            allocator,
            estimator_seed,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("flight-controller construction failed: {err}"),
        })?;
        let sensors = build_sensors(&scenario.document, resolved_files)?;
        let specific_force_source = bridge_specific_force_source(&scenario.document);
        let transport_session = build_transport_session(fc_config, &scenario.document)?;
        let transport_faults = build_transport_faults(fc_config);
        let atmosphere_kind = scenario_atmosphere_kind(&scenario.document);
        let atmosphere = if is_runtime_atmosphere_kind(atmosphere_kind) {
            Some(build_document_runtime_atmosphere(&scenario.document)?)
        } else {
            None
        };
        Ok(Some(Self {
            runner,
            sensors,
            atmosphere,
            fallback_atmosphere: UsStandard1976::with_exoatmospheric_policy(
                ExoatmosphericPolicy::ZeroDensityAboveCeiling,
            ),
            geocentric_surface_radius_m: document_geocentric_surface_radius_m(&scenario.document),
            magnetic,
            frame,
            specific_force_source,
            scenario_seed: scenario.document.time.seed,
            stimulus_schedule: StimulusSchedule::build(&scenario.document)?,
            outage_windows: Vec::new(),
            previous_velocity_eci_m_s: None,
            previous_time_s: None,
            previous_angular_velocity_body_rad_s: None,
            previous_angular_time_s: None,
            last_mission_action_sequence: None,
            transport_session,
            transport_faults,
            comm_link_effects: None,
            comm_frame_queues: CommLinkFrameQueues::default(),
            bridge_dt_s: scenario.document.time.dt_s,
            actuator_stream: Vec::new(),
        }))
    }

    /// Return a compact digest report for applied FC actuator packets.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Bridge`] if bridge command serialization fails.
    pub fn actuator_stream_report(
        &self,
    ) -> Result<Option<crate::determinism::ActuatorStreamReport>, RunnerError> {
        crate::determinism::actuator_stream_report(&self.actuator_stream).map_err(Into::into)
    }

    /// Set packet effects evaluated by the comm link model for this bridge tick.
    pub(crate) fn set_comm_link_effects(
        &mut self,
        effects: Option<crate::comm::CommBridgePacketEffects>,
    ) {
        self.comm_link_effects = effects;
    }

    /// Returns the FC commander's most recent
    /// mission-state publication, or `None` when no FC is wired or
    /// the commander has not yet ticked. The scenario runner reads
    /// this between FC and kernel ticks and forwards into
    /// `kernel.set_external_mission_state` so the kernel observes
    /// (rather than duplicates) the FC's mission-state ownership.
    #[must_use]
    pub fn latest_mission_state_id(&self) -> Option<u64> {
        self.runner
            .latest_mission_state()
            .map(|s| s.mission_state_id)
    }

    /// Drain FC-fired mission actions published since the previous
    /// bridge read.
    #[must_use]
    pub fn drain_mission_actions(&mut self) -> Vec<FiredEvent<MissionAction>> {
        let Some((batch, sequence)) = self.runner.latest_mission_action_batch() else {
            return Vec::new();
        };
        if self.last_mission_action_sequence == Some(sequence) {
            return Vec::new();
        }
        self.last_mission_action_sequence = Some(sequence);
        batch.events
    }

    /// Returns the most recent guidance reference published by the FC.
    #[must_use]
    pub fn latest_reference_state(&self) -> Option<ReferenceState> {
        self.runner.latest_reference_state()
    }

    /// Whether the bridge needs runner-supplied force/moment truth for IMU
    /// specific force and angular acceleration.
    #[must_use]
    pub(crate) fn uses_force_accumulator_truth(&self) -> bool {
        self.specific_force_source == SpecificForceSourceConfig::ForceAccumulator
    }

    /// Returns the most recent guidance cutoff estimate published by the FC.
    #[must_use]
    pub fn latest_guidance_cutoff(&self) -> Option<GuidanceCutoff> {
        self.runner.latest_guidance_cutoff()
    }

    /// Run one point-mass bridge tick and push FC commands into the
    /// runner-side racks.
    ///
    /// # Errors
    ///
    /// Propagates sensor, controller, or rack errors as [`RunnerError`].
    #[allow(clippy::too_many_arguments)]
    pub fn tick_point_mass(
        &mut self,
        state: &PointMassState,
        step: StepIndex,
        gravity_eci_m_s2: Vector3<f64>,
        specific_force_truth: Option<SpecificForceTruth>,
        propellant_state: Option<PropellantState>,
        effectors: &mut crate::effectors::EffectorRack,
        engines: &mut crate::engines::EngineRack,
        monitor: Option<&mut (dyn crate::sil::SilMonitor + '_)>,
    ) -> Result<(), RunnerError> {
        let truth = self.point_mass_truth(state, gravity_eci_m_s2, specific_force_truth)?;
        self.step_from_truth(truth, step, propellant_state, effectors, engines, monitor)
    }

    /// Run one rigid-body bridge tick and push FC commands into the
    /// runner-side racks.
    ///
    /// # Errors
    ///
    /// Propagates sensor, controller, or rack errors as [`RunnerError`].
    #[allow(clippy::too_many_arguments)]
    pub fn tick_rigid_body(
        &mut self,
        state: &RigidBodyState,
        step: StepIndex,
        gravity_eci_m_s2: Vector3<f64>,
        specific_force_truth: Option<SpecificForceTruth>,
        propellant_state: Option<PropellantState>,
        effectors: &mut crate::effectors::EffectorRack,
        engines: &mut crate::engines::EngineRack,
        gyro_pickup_rad_s: Vector3<f64>,
        monitor: Option<&mut (dyn crate::sil::SilMonitor + '_)>,
    ) -> Result<(), RunnerError> {
        let truth = self.rigid_body_truth(
            state,
            gravity_eci_m_s2,
            specific_force_truth,
            gyro_pickup_rad_s,
        )?;
        self.step_from_truth(truth, step, propellant_state, effectors, engines, monitor)
    }

    /// Arm a sensor outage window at run time, addressing a bridge
    /// sensor by its `[sensors.<name>]` key.
    ///
    /// While the simulation clock is inside `[start_s, stop_s)` the
    /// sensor is still primed and read each tick — its deterministic
    /// noise draws are unchanged — but the measurement is withheld from
    /// the flight controller, modelling a communications or tracking
    /// loss. The estimator sees a stale topic and falls back to its own
    /// dead-reckoning / staleness handling. A run that arms the same
    /// window reproduces byte-identically.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::UnsupportedScenario`] when `sensor` names
    /// no bridge sensor (fail-closed) or the window is empty
    /// (`stop_s <= start_s`).
    pub(crate) fn arm_sensor_outage(
        &mut self,
        sensor: &str,
        start_s: f64,
        stop_s: f64,
    ) -> Result<(), RunnerError> {
        let sensor_id = SensorId::from_path(&format!("sensors.{sensor}"));
        if !self.sensors.iter().any(|s| s.sensor_id() == sensor_id) {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "sensor outage targets `{sensor}`, which is not a bridge sensor declared in [sensors]"
                ),
            });
        }
        if stop_s <= start_s {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "sensor outage on `{sensor}` requires stop_s > start_s (got start_s = {start_s}, stop_s = {stop_s})"
                ),
            });
        }
        self.outage_windows.push(ArmedOutage {
            sensor_id,
            start_s,
            stop_s,
        });
        Ok(())
    }

    /// Whether an armed outage window covers `sensor_id` at `time_s`.
    fn outage_active(&self, sensor_id: SensorId, time_s: f64) -> bool {
        self.outage_windows
            .iter()
            .any(|o| o.sensor_id == sensor_id && time_s >= o.start_s && time_s < o.stop_s)
    }

    /// Assemble a read-only observation of the controller's latest
    /// published estimates and health.
    ///
    /// Shared by the per-tick SIL monitor tap and the steppable
    /// [`crate::Session`] accessor; both are observation-only surfaces
    /// (see [`crate::sil`]) and the assembly does no work unless called.
    pub(crate) fn collect_observation(&self) -> crate::sil::FcObservation {
        crate::sil::FcObservation {
            attitude: self.runner.latest_attitude_estimate(),
            position: self.runner.latest_position_estimate(),
            estimator: self.runner.latest_estimator_status(),
            fdir: self.runner.latest_fdir_status(),
            mode: self.runner.latest_estimator_mode(),
            reference: self.runner.latest_reference_state(),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn step_from_truth(
        &mut self,
        truth: SensorTruth,
        step: StepIndex,
        propellant_state: Option<PropellantState>,
        effectors: &mut crate::effectors::EffectorRack,
        engines: &mut crate::engines::EngineRack,
        monitor: Option<&mut (dyn crate::sil::SilMonitor + '_)>,
    ) -> Result<(), RunnerError> {
        let mut bridge_packet = SensorPacket {
            sim_time_s: truth.time.as_seconds(),
            step: step.value(),
            imu_accel_body_m_s2: [0.0; 3],
            imu_gyro_body_rad_s: [0.0; 3],
            imu_increments: Vec::new(),
            baro_altitude_m: None,
            gnss_position_eci_m: None,
            gnss_velocity_eci_m_s: None,
            gnss_position_bias_eci_m: None,
            mag_body_tesla: None,
            mag_body_nt: None,
            mag_hard_iron_body_nt: None,
            baro_pressure_pa: None,
            baro_bias_pa: None,
            airdata_static_pressure_pa: None,
            airdata_impact_pressure_pa: None,
            airdata_mach: None,
            airdata_calibrated_airspeed_m_s: None,
            airdata_true_airspeed_m_s: None,
            airdata_angle_of_attack_rad: None,
            airdata_sideslip_rad: None,
            airdata_pressure_altitude_m: None,
            star_tracker_attitude_eci_to_body_xyzw: None,
        };
        let mut has_bridge_measurement = false;
        let use_transport = self.transport_session.is_some();

        for index in 0..self.sensors.len() {
            let sensor_id = self.sensors[index].sensor_id();
            let (mut measurement, imu_increment_window) = {
                let sensor = &mut self.sensors[index];
                sensor.prime(truth, step, self.scenario_seed);
                let measurement = sensor.read()?;
                let window = sensor.imu_increment_window(measurement.time, true);
                (measurement, window)
            };
            // Open-loop SIL fault injection at the read -> publish seam.
            // With no armed fault the schedule is empty, so this draws
            // nothing and leaves the measurement byte-identical.
            if !self.stimulus_schedule.is_empty() {
                self.stimulus_schedule.apply(
                    sensor_id,
                    &mut measurement.value,
                    step,
                    truth.time.as_seconds(),
                    self.scenario_seed,
                );
            }
            // Outage windows withhold the measurement AFTER the read so
            // the sensor's deterministic noise stream is unaffected:
            // every other channel of the run stays byte-identical to the
            // outage-free run.
            if self.outage_active(sensor_id, truth.time.as_seconds()) {
                continue;
            }
            if use_transport {
                has_bridge_measurement |=
                    append_bridge_measurement(&mut bridge_packet, &measurement.value)?;
                if let Some(window) = imu_increment_window {
                    append_bridge_imu_increment_window(&mut bridge_packet, &window);
                    has_bridge_measurement = true;
                }
            } else {
                self.publish_measurement(&measurement);
                if let Some(window) = imu_increment_window {
                    self.runner.publish_imu_increment_window(window);
                }
            }
        }
        self.runner.publish_environment(EnvironmentEstimate {
            time: truth.time,
            density_kg_m3: self
                .sample_atmosphere(truth.altitude_geometric_m, truth.time)
                .density_kg_m3,
        });
        if let Some(state) = propellant_state {
            self.runner.publish_propellant_state(state);
        }
        let transport_command = if use_transport {
            if !has_bridge_measurement {
                return Err(RunnerError::UnsupportedScenario {
                    what:
                        "[fc.transport] requires at least one bridge-representable sensor measurement"
                            .to_owned(),
                });
            }
            self.step_controller_via_transport(bridge_packet)?
        } else {
            self.step_controller_direct(truth.time, step)?;
            None
        };
        let evidence_command = if let Some(command) = transport_command.as_ref() {
            Some(command.clone())
        } else if !use_transport {
            self.runner
                .latest_bridge_command_packet(truth.time, step)
                .ok()
        } else {
            None
        };
        if let Some(command) = evidence_command {
            self.actuator_stream.push(command);
        }
        // Observe AFTER the controller steps (estimate is fresh) and BEFORE
        // its commands reach the racks. Guard-gated so the default path
        // does zero extra work and remains byte-identical.
        if let Some(monitor) = monitor {
            let observation = self.collect_observation();
            monitor.observe(step, &truth, &observation);
        }
        if let Some(command) = transport_command {
            self.apply_transport_command(&command, effectors, engines)?;
        } else if !use_transport {
            if let Some(commands) = self.runner.latest_effector_command_set()
                && !effectors.is_empty()
            {
                effectors.apply_fc_commands(&commands)?;
            }
            if let Some(commands) = self.runner.latest_engine_command_set()
                && !engines.is_empty()
            {
                engines.apply_fc_commands(&commands)?;
            }
        }
        Ok(())
    }

    fn step_controller_direct(
        &mut self,
        time: openbmp_core::SimTime,
        step: StepIndex,
    ) -> Result<(), RunnerError> {
        self.runner
            .step(time, step)
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("flight-controller tick failed: {err}"),
            })?;
        Ok(())
    }

    fn step_controller_via_transport(
        &mut self,
        sensor: SensorPacket,
    ) -> Result<Option<ActuatorCommandPacket>, RunnerError> {
        let current_step = sensor.step;
        self.enqueue_sensor_for_comm_delivery(sensor)?;

        if let Some(delivered_sensor) = self.comm_frame_queues.pop_due_sensor(current_step) {
            let mut session =
                self.transport_session
                    .take()
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "fc.transport session was not configured".to_owned(),
                    })?;
            let result = self.step_delivered_sensor_via_transport(&mut session, &delivered_sensor);
            self.transport_session = Some(session);
            let command = result?;
            self.enqueue_command_for_comm_delivery(command, current_step)?;
        }

        Ok(self.comm_frame_queues.pop_due_command(current_step))
    }

    fn step_delivered_sensor_via_transport(
        &mut self,
        session: &mut FcTransportSession,
        sensor: &SensorPacket,
    ) -> Result<ActuatorCommandPacket, RunnerError> {
        session.ensure_handshake()?;
        if session.has_external_controller() {
            let mut transmitted_sensor = sensor.clone();
            self.apply_sensor_transport_faults(&mut transmitted_sensor)?;
            session
                .simulator
                .send(&BridgeMessage::Sensor(transmitted_sensor))?;
        } else {
            session
                .simulator
                .send(&BridgeMessage::Sensor(sensor.clone()))?;

            let BridgeMessage::Sensor(received_sensor) = session.local_controller_mut()?.recv()?
            else {
                return Err(RunnerError::UnsupportedScenario {
                    what: "fc.transport expected sensor frame on controller endpoint".to_owned(),
                });
            };
            let mut received_sensor = received_sensor;
            self.apply_sensor_transport_faults(&mut received_sensor)?;
            let command = self.runner.step_bridge_packet(&received_sensor)?;
            session
                .local_controller_mut()?
                .send(&BridgeMessage::Command(command))?;
        }

        let command = match session.simulator.recv()? {
            BridgeMessage::Command(command) => command,
            BridgeMessage::Ack(ack) => {
                validate_ack_for_sensor(sensor, &ack).map_err(|err| {
                    RunnerError::UnsupportedScenario {
                        what: format!("fc.transport lockstep validation failed: {err}"),
                    }
                })?;
                ActuatorCommandPacket {
                    sim_time_s: ack.sim_time_s,
                    step: ack.step,
                    effector_commands: Vec::new(),
                    engine_throttles: Vec::new(),
                    engine_commands: Vec::new(),
                }
            }
            BridgeMessage::Fault(fault) => {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!("fc.transport peer reported fault: {:?}", fault.code),
                });
            }
            _ => {
                return Err(RunnerError::UnsupportedScenario {
                    what: "fc.transport expected command frame on simulator endpoint".to_owned(),
                });
            }
        };
        let mut command = command;
        self.apply_command_transport_faults(&mut command)?;
        validate_command_for_sensor(sensor, &command).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("fc.transport lockstep validation failed: {err}"),
            }
        })?;
        Ok(command)
    }

    fn enqueue_sensor_for_comm_delivery(
        &mut self,
        sensor: SensorPacket,
    ) -> Result<(), RunnerError> {
        let arrival_step = if let Some(effects) = &self.comm_link_effects {
            apply_comm_link_packet_effect(
                "sensor",
                sensor.step,
                &effects.link_id,
                &effects.sensor,
            )?;
            comm_arrival_step(sensor.step, effects.sensor.latency_s, self.bridge_dt_s)?
        } else {
            sensor.step
        };
        self.comm_frame_queues.enqueue_sensor(arrival_step, sensor);
        Ok(())
    }

    fn enqueue_command_for_comm_delivery(
        &mut self,
        command: ActuatorCommandPacket,
        transmit_step: u64,
    ) -> Result<(), RunnerError> {
        let arrival_step = if let Some(effects) = &self.comm_link_effects {
            apply_comm_link_packet_effect(
                "command",
                command.step,
                &effects.link_id,
                &effects.command,
            )?;
            comm_arrival_step(transmit_step, effects.command.latency_s, self.bridge_dt_s)?
        } else {
            transmit_step
        };
        self.comm_frame_queues
            .enqueue_command(arrival_step, command);
        Ok(())
    }

    fn apply_sensor_transport_faults(&self, sensor: &mut SensorPacket) -> Result<(), RunnerError> {
        let sensor_faults = self
            .transport_faults
            .apply_sensor_frame(sensor)
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("fc.transport fault transform failed: {err}"),
            })?;
        match sensor_faults.disposition {
            BridgePacketDisposition::Deliver => Ok(()),
            BridgePacketDisposition::Drop => Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "fc.transport sensor frame step {} dropped by fault rule(s): {}",
                    sensor.step,
                    sensor_faults.applied_rule_ids.join(", ")
                ),
            }),
            BridgePacketDisposition::Duplicate => Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "fc.transport sensor frame step {} duplicated by fault rule(s): {}",
                    sensor.step,
                    sensor_faults.applied_rule_ids.join(", ")
                ),
            }),
            BridgePacketDisposition::Delay { steps } => Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "fc.transport sensor frame step {} delayed by {} step(s) by fault rule(s): {}",
                    sensor.step,
                    steps,
                    sensor_faults.applied_rule_ids.join(", ")
                ),
            }),
            BridgePacketDisposition::BitFlip { mask } => Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "fc.transport sensor frame step {} corrupted by bit-flip mask 0x{mask:02x} by fault rule(s): {}",
                    sensor.step,
                    sensor_faults.applied_rule_ids.join(", ")
                ),
            }),
        }
    }

    fn apply_command_transport_faults(
        &self,
        command: &mut ActuatorCommandPacket,
    ) -> Result<(), RunnerError> {
        let command_faults = self
            .transport_faults
            .apply_command_frame(command)
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("fc.transport fault transform failed: {err}"),
            })?;
        match command_faults.disposition {
            BridgePacketDisposition::Deliver => Ok(()),
            BridgePacketDisposition::Drop => Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "fc.transport command frame step {} dropped by fault rule(s): {}",
                    command.step,
                    command_faults.applied_rule_ids.join(", ")
                ),
            }),
            BridgePacketDisposition::Duplicate => Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "fc.transport command frame step {} duplicated by fault rule(s): {}",
                    command.step,
                    command_faults.applied_rule_ids.join(", ")
                ),
            }),
            BridgePacketDisposition::Delay { steps } => Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "fc.transport command frame step {} delayed by {} step(s) by fault rule(s): {}",
                    command.step,
                    steps,
                    command_faults.applied_rule_ids.join(", ")
                ),
            }),
            BridgePacketDisposition::BitFlip { mask } => Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "fc.transport command frame step {} corrupted by bit-flip mask 0x{mask:02x} by fault rule(s): {}",
                    command.step,
                    command_faults.applied_rule_ids.join(", ")
                ),
            }),
        }
    }

    fn apply_transport_command(
        &self,
        command: &ActuatorCommandPacket,
        effectors: &mut crate::effectors::EffectorRack,
        engines: &mut crate::engines::EngineRack,
    ) -> Result<(), RunnerError> {
        if !command.effector_commands.is_empty() && !effectors.is_empty() {
            let commands = effector_command_set_from_packet(command)?;
            effectors.apply_fc_commands(&commands)?;
        }
        if (!command.engine_commands.is_empty() || !command.engine_throttles.is_empty())
            && !engines.is_empty()
        {
            let commands = engine_command_set_from_packet(command)?;
            engines.apply_fc_commands(&commands)?;
        }
        Ok(())
    }

    fn publish_measurement(&self, measurement: &openbmp_sensors::Timestamped<SensorMeasurement>) {
        match measurement.value {
            SensorMeasurement::IdealState(_) => {}
            SensorMeasurement::Imu {
                gyro_rad_s,
                accel_m_s2,
            } => self.runner.publish_imu(ImuSample {
                time: measurement.time,
                gyro_rad_s,
                accel_m_s2,
                healthy: true,
            }),
            SensorMeasurement::Barometer {
                pressure_pa,
                bias_pa,
            } => self.runner.publish_barometer(BarometerSample {
                time: measurement.time,
                pressure_pa,
                bias_pa,
                healthy: true,
            }),
            SensorMeasurement::AirData {
                static_pressure_pa,
                impact_pressure_pa,
                mach,
                calibrated_airspeed_m_s,
                true_airspeed_m_s,
                angle_of_attack_rad,
                sideslip_rad,
                pressure_altitude_m,
            } => self.runner.publish_airdata(AirDataSample {
                time: measurement.time,
                static_pressure_pa,
                impact_pressure_pa,
                mach,
                calibrated_airspeed_m_s,
                true_airspeed_m_s,
                angle_of_attack_rad,
                sideslip_rad,
                pressure_altitude_m,
                healthy: true,
            }),
            SensorMeasurement::Gnss {
                position_eci_m,
                velocity_eci_m_s,
                position_bias_eci_m,
            } => self.runner.publish_gnss(GnssSample {
                time: measurement.time,
                position_eci_m,
                velocity_eci_m_s,
                position_bias_eci_m,
                healthy: true,
            }),
            SensorMeasurement::Magnetometer {
                field_body_nt,
                hard_iron_body_nt,
            } => self.runner.publish_magnetometer(MagnetometerSample {
                time: measurement.time,
                field_body_nt,
                hard_iron_body_nt,
                healthy: true,
            }),
            SensorMeasurement::StarTracker {
                attitude_eci_to_body,
            } => {
                let q = attitude_eci_to_body.into_inner();
                self.runner.publish_star_tracker(StarTrackerSample {
                    time: measurement.time,
                    q_eci_to_body_xyzw: [q.i, q.j, q.k, q.w],
                    healthy: true,
                });
            }
        }
    }

    fn point_mass_truth(
        &mut self,
        state: &PointMassState,
        gravity_eci_m_s2: Vector3<f64>,
        force_truth: Option<SpecificForceTruth>,
    ) -> Result<SensorTruth, RunnerError> {
        let attitude_body_to_eci = UnitQuaternion::identity();
        let (angular_velocity_body_rad_s, angular_acceleration_body_rad_s2, specific_force_body) =
            match self.specific_force_source {
                SpecificForceSourceConfig::FiniteDifference => {
                    let specific_force_eci = self.specific_force_eci(
                        state.velocity.vector,
                        state.time.as_seconds(),
                        gravity_eci_m_s2,
                    );
                    (
                        Vector3::zeros(),
                        Vector3::zeros(),
                        attitude_body_to_eci.inverse() * specific_force_eci,
                    )
                }
                SpecificForceSourceConfig::ForceAccumulator => {
                    let truth = force_truth.ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "specific_force_source = \"force_accumulator\" requires runner force truth"
                            .to_owned(),
                    })?;
                    (
                        truth.omega_body_rad_s,
                        truth.alpha_body_rad_s2,
                        truth.f_cg_body_m_s2,
                    )
                }
                SpecificForceSourceConfig::StationaryRotatingFrame => {
                    let truth = stationary_rotating_frame_imu_truth_eci(
                        state.position,
                        gravity_eci_m_s2,
                        self.frame.angular_velocity_z_at(state.time),
                    )
                    .map_err(|err| RunnerError::UnsupportedScenario {
                        what: format!(
                            "specific_force_source = \"stationary_rotating_frame\" failed: {err}"
                        ),
                    })?;
                    (
                        attitude_body_to_eci.inverse() * truth.angular_velocity_eci_rad_s,
                        Vector3::zeros(),
                        attitude_body_to_eci.inverse() * truth.specific_force_eci_m_s2,
                    )
                }
            };
        Ok(self.truth_common(
            state.position,
            state.velocity,
            attitude_body_to_eci,
            angular_velocity_body_rad_s,
            angular_acceleration_body_rad_s2,
            specific_force_body,
            state.time,
        ))
    }

    fn rigid_body_truth(
        &mut self,
        state: &RigidBodyState,
        gravity_eci_m_s2: Vector3<f64>,
        force_truth: Option<SpecificForceTruth>,
        gyro_pickup_rad_s: Vector3<f64>,
    ) -> Result<SensorTruth, RunnerError> {
        let attitude_body_to_eci = state.orientation.q;
        // The rate gyro senses the rigid-body rate PLUS the local structural
        // bending slope rate (zero for a rigid vehicle). This is what lets the
        // autopilot — and its gyro notch — interact with the flex mode.
        let (angular_velocity_body_rad_s, angular_acceleration_body_rad_s2, specific_force_body) =
            match self.specific_force_source {
                SpecificForceSourceConfig::FiniteDifference => {
                    let specific_force_eci = self.specific_force_eci(
                        state.velocity.vector,
                        state.time.as_seconds(),
                        gravity_eci_m_s2,
                    );
                    let angular_velocity_body_rad_s =
                        state.angular_velocity.vector + gyro_pickup_rad_s;
                    let angular_acceleration_body_rad_s2 = self.angular_acceleration_body(
                        angular_velocity_body_rad_s,
                        state.time.as_seconds(),
                    );
                    (
                        angular_velocity_body_rad_s,
                        angular_acceleration_body_rad_s2,
                        attitude_body_to_eci.inverse() * specific_force_eci,
                    )
                }
                SpecificForceSourceConfig::ForceAccumulator => {
                    let truth = force_truth.ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "specific_force_source = \"force_accumulator\" requires runner force truth"
                            .to_owned(),
                    })?;
                    (
                        truth.omega_body_rad_s + gyro_pickup_rad_s,
                        truth.alpha_body_rad_s2,
                        truth.f_cg_body_m_s2,
                    )
                }
                SpecificForceSourceConfig::StationaryRotatingFrame => {
                    let truth = stationary_rotating_frame_imu_truth_eci(
                        state.position,
                        gravity_eci_m_s2,
                        self.frame.angular_velocity_z_at(state.time),
                    )
                    .map_err(|err| RunnerError::UnsupportedScenario {
                        what: format!(
                            "specific_force_source = \"stationary_rotating_frame\" failed: {err}"
                        ),
                    })?;
                    (
                        attitude_body_to_eci.inverse() * truth.angular_velocity_eci_rad_s
                            + gyro_pickup_rad_s,
                        Vector3::zeros(),
                        attitude_body_to_eci.inverse() * truth.specific_force_eci_m_s2,
                    )
                }
            };
        Ok(self.truth_common(
            state.position,
            state.velocity,
            attitude_body_to_eci,
            angular_velocity_body_rad_s,
            angular_acceleration_body_rad_s2,
            specific_force_body,
            state.time,
        ))
    }

    // Assembles one full truth-state record; the eight components
    // (position, velocity, attitude, rates, mass, time, ...) are
    // intrinsic to the record, not a refactor smell.
    #[allow(clippy::too_many_arguments)]
    fn truth_common(
        &self,
        position: Position3<openbmp_core::Eci>,
        velocity: Velocity3<openbmp_core::Eci>,
        attitude_body_to_eci: UnitQuaternion<f64>,
        angular_velocity_body_rad_s: Vector3<f64>,
        angular_acceleration_body_rad_s2: Vector3<f64>,
        specific_force_body_m_s2: Vector3<f64>,
        time: openbmp_core::SimTime,
    ) -> SensorTruth {
        let altitude_m =
            bridge_sensor_altitude_m(position.vector, self.geocentric_surface_radius_m);
        let atmosphere = self.sample_atmosphere(altitude_m, time);
        let static_pressure_pa = atmosphere.pressure_pa;
        let attitude_eci_to_body = attitude_body_to_eci.inverse();
        let magnetic_field_body_nt =
            attitude_eci_to_body * self.magnetic.field_eci_nt(position.vector, time);
        let air_relative_velocity_body_m_s = attitude_eci_to_body * velocity.vector;
        SensorTruth {
            position_eci: position,
            velocity_eci: velocity,
            attitude_eci_to_body,
            angular_velocity_body_rad_s,
            angular_acceleration_body_rad_s2,
            specific_force_body_m_s2,
            static_pressure_pa,
            atmosphere_density_kg_m3: atmosphere.density_kg_m3,
            speed_of_sound_m_s: atmosphere.speed_of_sound_m_s,
            air_relative_velocity_body_m_s,
            altitude_geometric_m: altitude_m,
            magnetic_field_body_nt,
            time,
        }
    }

    fn sample_atmosphere(
        &self,
        altitude_m: f64,
        time: openbmp_core::SimTime,
    ) -> openbmp_physics::AtmosphereSample {
        let sample = self.atmosphere.as_ref().map_or_else(
            || self.fallback_atmosphere.sample(altitude_m, time),
            |atmosphere| atmosphere.sample(altitude_m, time),
        );
        sample.unwrap_or_else(|_| openbmp_physics::AtmosphereSample {
            pressure_pa: openbmp_physics::atmosphere::USSA76_SEA_LEVEL_PRESSURE_PA,
            ..openbmp_physics::AtmosphereSample::default()
        })
    }

    fn specific_force_eci(
        &mut self,
        velocity_eci_m_s: Vector3<f64>,
        time_s: f64,
        gravity_eci_m_s2: Vector3<f64>,
    ) -> Vector3<f64> {
        let total_accel = match (self.previous_velocity_eci_m_s, self.previous_time_s) {
            (Some(prev_v), Some(prev_t)) if time_s > prev_t => {
                (velocity_eci_m_s - prev_v) / (time_s - prev_t)
            }
            _ => Vector3::zeros(),
        };
        self.previous_velocity_eci_m_s = Some(velocity_eci_m_s);
        self.previous_time_s = Some(time_s);
        total_accel - gravity_eci_m_s2
    }

    fn angular_acceleration_body(
        &mut self,
        angular_velocity_body_rad_s: Vector3<f64>,
        time_s: f64,
    ) -> Vector3<f64> {
        let angular_accel = match (
            self.previous_angular_velocity_body_rad_s,
            self.previous_angular_time_s,
        ) {
            (Some(prev_omega), Some(prev_t)) if time_s > prev_t => {
                (angular_velocity_body_rad_s - prev_omega) / (time_s - prev_t)
            }
            _ => Vector3::zeros(),
        };
        self.previous_angular_velocity_body_rad_s = Some(angular_velocity_body_rad_s);
        self.previous_angular_time_s = Some(time_s);
        angular_accel
    }
}

fn apply_comm_link_packet_effect(
    frame_kind: &'static str,
    step: u64,
    link_id: &str,
    effect: &LinkPacketEffect,
) -> Result<(), RunnerError> {
    match effect.disposition {
        LinkPacketDisposition::Deliver => Ok(()),
        LinkPacketDisposition::Drop => Err(RunnerError::UnsupportedScenario {
            what: format!(
                "comm link `{link_id}` dropped fc.transport {frame_kind} frame step {step} (latency_s = {:.17e}, fer_draw = {})",
                effect.latency_s,
                format_optional_draw(effect.error_draw),
            ),
        }),
        LinkPacketDisposition::BitFlip { mask } => Err(RunnerError::UnsupportedScenario {
            what: format!(
                "comm link `{link_id}` corrupted fc.transport {frame_kind} frame step {step} with bit-flip mask 0x{mask:02x} (latency_s = {:.17e}, fer_draw = {})",
                effect.latency_s,
                format_optional_draw(effect.error_draw),
            ),
        }),
    }
}

fn comm_arrival_step(transmit_step: u64, latency_s: f64, dt_s: f64) -> Result<u64, RunnerError> {
    let delay_steps = comm_latency_step_delay(latency_s, dt_s)?;
    transmit_step
        .checked_add(delay_steps)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!(
                "comm link frame latency overflows step index: transmit_step = {transmit_step}, delay_steps = {delay_steps}"
            ),
        })
}

fn comm_latency_step_delay(latency_s: f64, dt_s: f64) -> Result<u64, RunnerError> {
    if !latency_s.is_finite() || latency_s < 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "comm link frame latency must be finite and non-negative, got {latency_s}"
            ),
        });
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("comm link frame delay requires positive finite dt_s, got {dt_s}"),
        });
    }
    let delay_steps = (latency_s / dt_s).floor();
    if delay_steps > u64::MAX as f64 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "comm link frame latency is too large for step-index delay: latency_s = {latency_s}, dt_s = {dt_s}"
            ),
        });
    }
    Ok(delay_steps as u64)
}

fn format_optional_draw(draw: Option<f64>) -> String {
    draw.map_or_else(|| "none".to_owned(), |value| format!("{value:.17e}"))
}

pub(crate) fn propellant_state_from_tanks(
    time: openbmp_core::SimTime,
    tanks: &BTreeMap<openbmp_core::TankId, openbmp_vehicle::PropellantTankState>,
) -> Option<PropellantState> {
    if tanks.is_empty() {
        return None;
    }
    let mut initial_kg = 0.0;
    let mut remaining_kg = 0.0;
    for tank in tanks.values() {
        if !tank.initial_fluid_mass_kg.is_finite() || !tank.fluid_remaining_kg.is_finite() {
            return None;
        }
        initial_kg += tank.initial_fluid_mass_kg.max(0.0);
        remaining_kg += tank.fluid_remaining_kg.max(0.0);
    }
    if !initial_kg.is_finite() || initial_kg <= 0.0 {
        return None;
    }
    let remaining_kg = remaining_kg.min(initial_kg);
    let mass_fraction = (remaining_kg / initial_kg).clamp(0.0, 1.0);
    Some(PropellantState {
        time,
        mass_fraction,
        mass_remaining_kg: remaining_kg,
        mass_initial_kg: initial_kg,
        depleted: remaining_kg <= f64::EPSILON * initial_kg.max(1.0),
    })
}

fn build_transport_session(
    fc_config: &FcConfig,
    document: &ScenarioDocument,
) -> Result<Option<FcTransportSession>, RunnerError> {
    let Some(transport) = fc_config.transport.as_ref() else {
        return Ok(None);
    };

    let Some(sensors) = document.sensors.as_ref() else {
        return Err(RunnerError::UnsupportedScenario {
            what: "[fc.transport] requires an IMU sensor declaration".to_owned(),
        });
    };
    let mut has_imu = false;
    for (name, sensor) in sensors {
        if sensor.kind == "imu" {
            has_imu = true;
        } else if !matches!(
            sensor.kind.as_str(),
            "gnss" | "magnetometer" | "barometer" | "airdata" | "star_tracker"
        ) {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "[fc.transport] does not support sensors.<name>.kind = `{}`; \
                     `{name}` declares kind `{}`",
                    sensor.kind, sensor.kind
                ),
            });
        }
    }
    if !has_imu {
        return Err(RunnerError::UnsupportedScenario {
            what: "[fc.transport] requires an IMU sensor declaration".to_owned(),
        });
    }

    let max_payload_len = transport
        .max_payload_len
        .unwrap_or(DEFAULT_FC_TRANSPORT_MAX_PAYLOAD_LEN);
    let peer_protocol_version = transport
        .peer_protocol_version
        .unwrap_or(openbmp_bridge::PROTOCOL_VERSION);
    match transport.mode {
        FcTransportModeConfig::InProcess => Ok(Some(FcTransportSession::new_in_process(
            max_payload_len,
            peer_protocol_version,
        ))),
        FcTransportModeConfig::TcpLoopback => {
            FcTransportSession::new_tcp_loopback(max_payload_len, peer_protocol_version).map(Some)
        }
        FcTransportModeConfig::UnixLoopback => {
            #[cfg(unix)]
            {
                FcTransportSession::new_unix_loopback(max_payload_len, peer_protocol_version)
                    .map(Some)
            }
            #[cfg(not(unix))]
            {
                Err(RunnerError::UnsupportedScenario {
                    what: "[fc.transport] mode = \"unix_loopback\" is only supported on Unix platforms"
                        .to_owned(),
                })
            }
        }
        FcTransportModeConfig::ExternalProcess => {
            let command =
                transport
                    .command
                    .as_deref()
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "[fc.transport] mode = \"external_process\" requires command"
                            .to_owned(),
                    })?;
            FcTransportSession::new_external_process(
                command,
                &transport.args,
                transport.working_dir.as_deref(),
                max_payload_len,
                peer_protocol_version,
            )
            .map(Some)
        }
    }
}

fn build_transport_faults(fc_config: &FcConfig) -> BridgeFaultTransformSet {
    let Some(faults) = &fc_config.transport_faults else {
        return BridgeFaultTransformSet::default();
    };
    BridgeFaultTransformSet::new_with_packet_rules(
        faults
            .rules
            .iter()
            .map(|rule| {
                BridgeFaultRule::new(
                    rule.id.clone(),
                    rule.start_step,
                    rule.end_step,
                    bridge_fault_signal(&rule.signal),
                    bridge_fault_transform(rule.transform),
                )
            })
            .collect(),
        faults
            .packet_rules
            .iter()
            .map(|rule| {
                BridgePacketFaultRule::new(
                    rule.id.clone(),
                    rule.start_step,
                    rule.end_step,
                    packet_direction(rule.direction),
                    packet_transform(rule.transform),
                )
            })
            .collect(),
    )
}

fn bridge_fault_signal(signal: &FcTransportFaultSignalConfig) -> BridgeScalarSignal {
    match signal {
        FcTransportFaultSignalConfig::ImuAccelBodyMps2 { axis } => {
            BridgeScalarSignal::ImuAccelBodyMps2(vector_axis(*axis))
        }
        FcTransportFaultSignalConfig::ImuGyroBodyRadS { axis } => {
            BridgeScalarSignal::ImuGyroBodyRadS(vector_axis(*axis))
        }
        FcTransportFaultSignalConfig::ImuIncrementDeltaThetaRad { sample_index, axis } => {
            BridgeScalarSignal::ImuIncrementDeltaThetaRad {
                sample_index: *sample_index,
                axis: vector_axis(*axis),
            }
        }
        FcTransportFaultSignalConfig::ImuIncrementDeltaVMs { sample_index, axis } => {
            BridgeScalarSignal::ImuIncrementDeltaVMs {
                sample_index: *sample_index,
                axis: vector_axis(*axis),
            }
        }
        FcTransportFaultSignalConfig::ImuIncrementDtS { sample_index } => {
            BridgeScalarSignal::ImuIncrementDtS {
                sample_index: *sample_index,
            }
        }
        FcTransportFaultSignalConfig::BaroAltitudeM => BridgeScalarSignal::BaroAltitudeM,
        FcTransportFaultSignalConfig::BaroPressurePa => BridgeScalarSignal::BaroPressurePa,
        FcTransportFaultSignalConfig::BaroBiasPa => BridgeScalarSignal::BaroBiasPa,
        FcTransportFaultSignalConfig::GnssPositionEciM { axis } => {
            BridgeScalarSignal::GnssPositionEciM(vector_axis(*axis))
        }
        FcTransportFaultSignalConfig::GnssVelocityEciMS { axis } => {
            BridgeScalarSignal::GnssVelocityEciMS(vector_axis(*axis))
        }
        FcTransportFaultSignalConfig::GnssPositionBiasEciM { axis } => {
            BridgeScalarSignal::GnssPositionBiasEciM(vector_axis(*axis))
        }
        FcTransportFaultSignalConfig::MagBodyTesla { axis } => {
            BridgeScalarSignal::MagBodyTesla(vector_axis(*axis))
        }
        FcTransportFaultSignalConfig::MagBodyNt { axis } => {
            BridgeScalarSignal::MagBodyNt(vector_axis(*axis))
        }
        FcTransportFaultSignalConfig::MagHardIronBodyNt { axis } => {
            BridgeScalarSignal::MagHardIronBodyNt(vector_axis(*axis))
        }
        FcTransportFaultSignalConfig::StarTrackerAttitudeEciToBody { axis } => {
            BridgeScalarSignal::StarTrackerAttitudeEciToBody(quaternion_axis(*axis))
        }
        FcTransportFaultSignalConfig::EffectorCommand { effector_id } => {
            BridgeScalarSignal::EffectorCommand {
                effector_id: *effector_id,
            }
        }
        FcTransportFaultSignalConfig::EngineThrottle { engine_id } => {
            BridgeScalarSignal::EngineThrottle {
                engine_id: *engine_id,
            }
        }
        FcTransportFaultSignalConfig::EngineGimbalPitch { engine_id } => {
            BridgeScalarSignal::EngineGimbalPitch {
                engine_id: *engine_id,
            }
        }
        FcTransportFaultSignalConfig::EngineGimbalYaw { engine_id } => {
            BridgeScalarSignal::EngineGimbalYaw {
                engine_id: *engine_id,
            }
        }
    }
}

fn bridge_fault_transform(transform: FcTransportFaultTransformConfig) -> BridgeScalarTransform {
    match transform {
        FcTransportFaultTransformConfig::AdditiveBias { offset } => {
            BridgeScalarTransform::AdditiveBias { offset }
        }
        FcTransportFaultTransformConfig::Scale { factor } => {
            BridgeScalarTransform::Scale { factor }
        }
        FcTransportFaultTransformConfig::Stuck { value } => BridgeScalarTransform::Stuck { value },
        FcTransportFaultTransformConfig::Saturate { min, max } => {
            BridgeScalarTransform::Saturate { min, max }
        }
        FcTransportFaultTransformConfig::Quantize { quantum } => {
            BridgeScalarTransform::Quantize { quantum }
        }
        FcTransportFaultTransformConfig::Drift {
            rate_per_s,
            reference_time_s,
        } => BridgeScalarTransform::Drift {
            rate_per_s,
            reference_time_s,
        },
        FcTransportFaultTransformConfig::NoiseBurst { amplitude, seed } => {
            BridgeScalarTransform::NoiseBurst { amplitude, seed }
        }
        FcTransportFaultTransformConfig::ReverseSign => BridgeScalarTransform::ReverseSign,
    }
}

const fn vector_axis(axis: FcTransportVectorAxisConfig) -> VectorAxis {
    match axis {
        FcTransportVectorAxisConfig::X => VectorAxis::X,
        FcTransportVectorAxisConfig::Y => VectorAxis::Y,
        FcTransportVectorAxisConfig::Z => VectorAxis::Z,
    }
}

const fn quaternion_axis(axis: FcTransportQuaternionAxisConfig) -> QuaternionAxis {
    match axis {
        FcTransportQuaternionAxisConfig::X => QuaternionAxis::X,
        FcTransportQuaternionAxisConfig::Y => QuaternionAxis::Y,
        FcTransportQuaternionAxisConfig::Z => QuaternionAxis::Z,
        FcTransportQuaternionAxisConfig::W => QuaternionAxis::W,
    }
}

const fn packet_direction(direction: FcTransportPacketDirectionConfig) -> BridgePacketDirection {
    match direction {
        FcTransportPacketDirectionConfig::Sensor => BridgePacketDirection::Sensor,
        FcTransportPacketDirectionConfig::Command => BridgePacketDirection::Command,
    }
}

const fn packet_transform(transform: FcTransportPacketTransformConfig) -> BridgePacketTransform {
    match transform {
        FcTransportPacketTransformConfig::Drop => BridgePacketTransform::Drop,
        FcTransportPacketTransformConfig::Duplicate => BridgePacketTransform::Duplicate,
        FcTransportPacketTransformConfig::Delay { steps } => BridgePacketTransform::Delay { steps },
        FcTransportPacketTransformConfig::BitFlip { mask } => {
            BridgePacketTransform::BitFlip { mask }
        }
        FcTransportPacketTransformConfig::StepOffset { offset } => {
            BridgePacketTransform::StepOffset { offset }
        }
        FcTransportPacketTransformConfig::TimeOffset { offset_s } => {
            BridgePacketTransform::TimeOffset { offset_s }
        }
    }
}

fn append_bridge_measurement(
    packet: &mut SensorPacket,
    measurement: &SensorMeasurement,
) -> Result<bool, RunnerError> {
    match measurement {
        SensorMeasurement::Imu {
            gyro_rad_s,
            accel_m_s2,
        } => {
            packet.imu_accel_body_m_s2 = [accel_m_s2.x, accel_m_s2.y, accel_m_s2.z];
            packet.imu_gyro_body_rad_s = [gyro_rad_s.x, gyro_rad_s.y, gyro_rad_s.z];
            Ok(true)
        }
        SensorMeasurement::Barometer {
            pressure_pa,
            bias_pa,
        } => {
            packet.baro_pressure_pa = Some(*pressure_pa);
            packet.baro_bias_pa = Some(*bias_pa);
            Ok(true)
        }
        SensorMeasurement::AirData {
            static_pressure_pa,
            impact_pressure_pa,
            mach,
            calibrated_airspeed_m_s,
            true_airspeed_m_s,
            angle_of_attack_rad,
            sideslip_rad,
            pressure_altitude_m,
        } => {
            packet.airdata_static_pressure_pa = Some(*static_pressure_pa);
            packet.airdata_impact_pressure_pa = Some(*impact_pressure_pa);
            packet.airdata_mach = Some(*mach);
            packet.airdata_calibrated_airspeed_m_s = Some(*calibrated_airspeed_m_s);
            packet.airdata_true_airspeed_m_s = Some(*true_airspeed_m_s);
            packet.airdata_angle_of_attack_rad = Some(*angle_of_attack_rad);
            packet.airdata_sideslip_rad = Some(*sideslip_rad);
            packet.airdata_pressure_altitude_m = Some(*pressure_altitude_m);
            Ok(true)
        }
        SensorMeasurement::Gnss {
            position_eci_m,
            velocity_eci_m_s,
            position_bias_eci_m,
        } => {
            packet.gnss_position_eci_m =
                Some([position_eci_m.x, position_eci_m.y, position_eci_m.z]);
            packet.gnss_velocity_eci_m_s =
                Some([velocity_eci_m_s.x, velocity_eci_m_s.y, velocity_eci_m_s.z]);
            packet.gnss_position_bias_eci_m = Some([
                position_bias_eci_m.x,
                position_bias_eci_m.y,
                position_bias_eci_m.z,
            ]);
            Ok(true)
        }
        SensorMeasurement::Magnetometer {
            field_body_nt,
            hard_iron_body_nt,
        } => {
            packet.mag_body_nt = Some([field_body_nt.x, field_body_nt.y, field_body_nt.z]);
            packet.mag_hard_iron_body_nt = Some([
                hard_iron_body_nt.x,
                hard_iron_body_nt.y,
                hard_iron_body_nt.z,
            ]);
            Ok(true)
        }
        SensorMeasurement::StarTracker {
            attitude_eci_to_body,
        } => {
            let q = attitude_eci_to_body.into_inner();
            packet.star_tracker_attitude_eci_to_body_xyzw = Some([q.i, q.j, q.k, q.w]);
            Ok(true)
        }
        SensorMeasurement::IdealState(_) => Err(RunnerError::UnsupportedScenario {
            what: "[fc.transport] does not support ideal_state sensor packets".to_owned(),
        }),
    }
}

fn append_bridge_imu_increment_window(packet: &mut SensorPacket, window: &ImuIncrementWindow) {
    packet.imu_increments = window
        .increments
        .iter()
        .map(|increment| ImuIncrementPacket {
            delta_theta_rad: [
                increment.delta_theta_rad.x,
                increment.delta_theta_rad.y,
                increment.delta_theta_rad.z,
            ],
            delta_v_m_s: [
                increment.delta_v_m_s.x,
                increment.delta_v_m_s.y,
                increment.delta_v_m_s.z,
            ],
            dt_s: increment.dt_s,
            seq: increment.seq,
        })
        .collect();
}

fn effector_command_set_from_packet(
    packet: &ActuatorCommandPacket,
) -> Result<EffectorCommandSet, RunnerError> {
    let mut set = EffectorCommandSet {
        time: openbmp_core::SimTime::from_seconds(packet.sim_time_s),
        ..EffectorCommandSet::default()
    };
    if packet.effector_commands.len() > set.commands.len() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "fc.transport command carries {} effector commands but runner capacity is {}",
                packet.effector_commands.len(),
                set.commands.len()
            ),
        });
    }
    set.count = u8::try_from(packet.effector_commands.len()).map_err(|_| {
        RunnerError::UnsupportedScenario {
            what: "fc.transport effector command count exceeds u8 range".to_owned(),
        }
    })?;
    for (slot, (effector_id, command)) in set
        .commands
        .iter_mut()
        .zip(packet.effector_commands.iter().copied())
    {
        *slot = EffectorCommand {
            effector_id: u64::from(effector_id),
            command,
            saturated: false,
        };
    }
    Ok(set)
}

fn engine_command_set_from_packet(
    packet: &ActuatorCommandPacket,
) -> Result<EngineCommandSet, RunnerError> {
    let mut set = EngineCommandSet {
        time: openbmp_core::SimTime::from_seconds(packet.sim_time_s),
        ..EngineCommandSet::default()
    };
    let command_count = if packet.engine_commands.is_empty() {
        packet.engine_throttles.len()
    } else {
        packet.engine_commands.len()
    };
    if command_count > set.commands.len() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "fc.transport command carries {} engine commands but runner capacity is {}",
                command_count,
                set.commands.len()
            ),
        });
    }
    set.count = u8::try_from(command_count).map_err(|_| RunnerError::UnsupportedScenario {
        what: "fc.transport engine command count exceeds u8 range".to_owned(),
    })?;
    if !packet.engine_commands.is_empty() {
        for (slot, command) in set
            .commands
            .iter_mut()
            .zip(packet.engine_commands.iter().copied())
        {
            *slot = EngineCommand {
                engine_id: u64::from(command.engine_id),
                throttle_unit: command.throttle_unit,
                gimbal_pitch_rad: command.gimbal_pitch_rad,
                gimbal_yaw_rad: command.gimbal_yaw_rad,
                ignite: command.ignite,
                shutdown: command.shutdown,
            };
        }
        return Ok(set);
    }

    for (slot, (engine_id, throttle_unit)) in set
        .commands
        .iter_mut()
        .zip(packet.engine_throttles.iter().copied())
    {
        *slot = EngineCommand {
            engine_id: u64::from(engine_id),
            throttle_unit,
            ..EngineCommand::default()
        };
    }
    Ok(set)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod fc_transport_packet_tests {
    use openbmp_bridge::EngineCommandPacket as BridgeEngineCommandPacket;

    use super::*;

    #[test]
    fn engine_command_set_from_packet_preserves_full_engine_fields() {
        let packet = ActuatorCommandPacket {
            sim_time_s: 1.25,
            step: 125,
            effector_commands: Vec::new(),
            engine_throttles: vec![(3, 0.1)],
            engine_commands: vec![BridgeEngineCommandPacket {
                engine_id: 3,
                throttle_unit: 0.75,
                gimbal_pitch_rad: 0.02,
                gimbal_yaw_rad: -0.03,
                ignite: true,
                shutdown: false,
            }],
        };
        let set = engine_command_set_from_packet(&packet).unwrap();
        assert_eq!(set.time.as_seconds().to_bits(), 1.25f64.to_bits());
        assert_eq!(set.count, 1);
        assert_eq!(set.commands[0].engine_id, 3);
        assert_eq!(set.commands[0].throttle_unit.to_bits(), 0.75f64.to_bits());
        assert_eq!(
            set.commands[0].gimbal_pitch_rad.to_bits(),
            0.02f64.to_bits()
        );
        assert_eq!(
            set.commands[0].gimbal_yaw_rad.to_bits(),
            (-0.03f64).to_bits()
        );
        assert!(set.commands[0].ignite);
        assert!(!set.commands[0].shutdown);
    }

    #[test]
    fn engine_command_set_from_packet_keeps_throttle_fallback() {
        let packet = ActuatorCommandPacket {
            sim_time_s: 2.0,
            step: 200,
            effector_commands: Vec::new(),
            engine_throttles: vec![(4, 0.4)],
            engine_commands: Vec::new(),
        };
        let set = engine_command_set_from_packet(&packet).unwrap();
        assert_eq!(set.count, 1);
        assert_eq!(set.commands[0].engine_id, 4);
        assert_eq!(set.commands[0].throttle_unit.to_bits(), 0.4f64.to_bits());
        assert_eq!(set.commands[0].gimbal_pitch_rad.to_bits(), 0.0f64.to_bits());
        assert!(!set.commands[0].ignite);
    }

    #[test]
    fn transport_fault_config_mapping_mutates_sensor_packet() {
        let mut packet = SensorPacket {
            sim_time_s: 0.125,
            step: 2,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            imu_gyro_body_rad_s: [0.01, 0.02, 0.026],
            imu_increments: vec![
                ImuIncrementPacket {
                    delta_theta_rad: [1.0e-4, 2.0e-4, 3.0e-4],
                    delta_v_m_s: [0.001, 0.002, 0.003],
                    dt_s: 0.00025,
                    seq: 20,
                },
                ImuIncrementPacket {
                    delta_theta_rad: [4.0e-4, 5.0e-4, 6.0e-4],
                    delta_v_m_s: [0.004, 0.005, 0.006],
                    dt_s: 0.00025,
                    seq: 21,
                },
            ],
            baro_pressure_pa: Some(100_000.0),
            ..SensorPacket::default()
        };
        let mut repeat = packet.clone();
        let faults = BridgeFaultTransformSet::new(vec![
            BridgeFaultRule::new(
                "imu-x-bias",
                2,
                Some(2),
                bridge_fault_signal(&FcTransportFaultSignalConfig::ImuAccelBodyMps2 {
                    axis: FcTransportVectorAxisConfig::X,
                }),
                bridge_fault_transform(FcTransportFaultTransformConfig::AdditiveBias {
                    offset: 0.25,
                }),
            ),
            BridgeFaultRule::new(
                "gyro-z-quantize",
                2,
                Some(2),
                bridge_fault_signal(&FcTransportFaultSignalConfig::ImuGyroBodyRadS {
                    axis: FcTransportVectorAxisConfig::Z,
                }),
                bridge_fault_transform(FcTransportFaultTransformConfig::Quantize { quantum: 0.01 }),
            ),
            BridgeFaultRule::new(
                "baro-drift",
                2,
                Some(2),
                bridge_fault_signal(&FcTransportFaultSignalConfig::BaroPressurePa),
                bridge_fault_transform(FcTransportFaultTransformConfig::Drift {
                    rate_per_s: 128.0,
                    reference_time_s: 0.0,
                }),
            ),
            BridgeFaultRule::new(
                "dtheta-y-bias",
                2,
                Some(2),
                bridge_fault_signal(&FcTransportFaultSignalConfig::ImuIncrementDeltaThetaRad {
                    sample_index: 1,
                    axis: FcTransportVectorAxisConfig::Y,
                }),
                bridge_fault_transform(FcTransportFaultTransformConfig::AdditiveBias {
                    offset: 1.0e-5,
                }),
            ),
            BridgeFaultRule::new(
                "dv-z-scale",
                2,
                Some(2),
                bridge_fault_signal(&FcTransportFaultSignalConfig::ImuIncrementDeltaVMs {
                    sample_index: 0,
                    axis: FcTransportVectorAxisConfig::Z,
                }),
                bridge_fault_transform(FcTransportFaultTransformConfig::Scale { factor: -2.0 }),
            ),
            BridgeFaultRule::new(
                "increment-dt-stuck",
                2,
                Some(2),
                bridge_fault_signal(&FcTransportFaultSignalConfig::ImuIncrementDtS {
                    sample_index: 1,
                }),
                bridge_fault_transform(FcTransportFaultTransformConfig::Stuck { value: 0.0005 }),
            ),
            BridgeFaultRule::new(
                "gyro-y-noise",
                2,
                Some(2),
                bridge_fault_signal(&FcTransportFaultSignalConfig::ImuGyroBodyRadS {
                    axis: FcTransportVectorAxisConfig::Y,
                }),
                bridge_fault_transform(FcTransportFaultTransformConfig::NoiseBurst {
                    amplitude: 0.01,
                    seed: 42,
                }),
            ),
        ]);

        let applied = faults.apply_to_sensor(&mut packet).unwrap();
        let repeat_applied = faults.apply_to_sensor(&mut repeat).unwrap();

        assert_eq!(
            applied,
            vec![
                "imu-x-bias",
                "gyro-z-quantize",
                "baro-drift",
                "dtheta-y-bias",
                "dv-z-scale",
                "increment-dt-stuck",
                "gyro-y-noise"
            ]
        );
        assert_eq!(repeat_applied, applied);
        assert_eq!(repeat, packet);
        assert_eq!(
            packet.imu_accel_body_m_s2.map(f64::to_bits),
            [1.25_f64.to_bits(), 2.0_f64.to_bits(), 3.0_f64.to_bits()]
        );
        assert!((packet.imu_gyro_body_rad_s[1] - 0.02).abs() <= 0.01);
        assert_ne!(packet.imu_gyro_body_rad_s[1].to_bits(), 0.02_f64.to_bits());
        assert_eq!(packet.imu_gyro_body_rad_s[0].to_bits(), 0.01_f64.to_bits());
        assert_eq!(packet.imu_gyro_body_rad_s[2].to_bits(), 0.03_f64.to_bits());
        assert_eq!(
            packet.baro_pressure_pa.map(f64::to_bits),
            Some(100_016.0_f64.to_bits())
        );
        assert_eq!(
            packet.imu_increments[1].delta_theta_rad[1].to_bits(),
            5.1e-4_f64.to_bits()
        );
        assert_eq!(
            packet.imu_increments[0].delta_v_m_s[2].to_bits(),
            (-0.006_f64).to_bits()
        );
        assert_eq!(
            packet.imu_increments[1].dt_s.to_bits(),
            0.0005_f64.to_bits()
        );
    }

    #[test]
    fn transport_fault_config_mapping_mutates_command_packet() {
        let mut packet = ActuatorCommandPacket {
            sim_time_s: 0.003,
            step: 3,
            effector_commands: Vec::new(),
            engine_throttles: vec![(9, 0.8)],
            engine_commands: vec![BridgeEngineCommandPacket {
                engine_id: 9,
                throttle_unit: 0.8,
                gimbal_pitch_rad: 0.01,
                gimbal_yaw_rad: 0.02,
                ignite: true,
                shutdown: false,
            }],
        };
        let faults = BridgeFaultTransformSet::new(vec![BridgeFaultRule::new(
            "engine-limit",
            3,
            None,
            bridge_fault_signal(&FcTransportFaultSignalConfig::EngineThrottle { engine_id: 9 }),
            bridge_fault_transform(FcTransportFaultTransformConfig::Saturate {
                min: 0.0,
                max: 0.5,
            }),
        )]);

        let applied = faults.apply_to_command(&mut packet).unwrap();

        assert_eq!(applied, vec!["engine-limit"]);
        assert_eq!(packet.engine_throttles[0].1.to_bits(), 0.5_f64.to_bits());
        assert_eq!(
            packet.engine_commands[0].throttle_unit.to_bits(),
            0.5_f64.to_bits()
        );
        assert_eq!(
            packet.engine_commands[0].gimbal_pitch_rad.to_bits(),
            0.01_f64.to_bits()
        );
    }

    #[test]
    fn transport_packet_fault_mapping_drops_sensor_frame() {
        let mut packet = SensorPacket {
            sim_time_s: 0.004,
            step: 4,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "drop-sensor",
                4,
                Some(4),
                packet_direction(FcTransportPacketDirectionConfig::Sensor),
                packet_transform(FcTransportPacketTransformConfig::Drop),
            )],
        );

        let application = faults.apply_sensor_frame(&mut packet).unwrap();

        assert_eq!(application.applied_rule_ids, vec!["drop-sensor"]);
        assert_eq!(application.disposition, BridgePacketDisposition::Drop);
    }

    #[test]
    fn transport_packet_fault_mapping_drops_command_frame() {
        let mut packet = ActuatorCommandPacket {
            sim_time_s: 0.005,
            step: 5,
            effector_commands: vec![(1, 0.2)],
            engine_throttles: Vec::new(),
            engine_commands: Vec::new(),
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "drop-command",
                5,
                None,
                packet_direction(FcTransportPacketDirectionConfig::Command),
                packet_transform(FcTransportPacketTransformConfig::Drop),
            )],
        );

        let application = faults.apply_command_frame(&mut packet).unwrap();

        assert_eq!(application.applied_rule_ids, vec!["drop-command"]);
        assert_eq!(application.disposition, BridgePacketDisposition::Drop);
    }

    #[test]
    fn transport_packet_fault_mapping_duplicates_command_frame() {
        let mut packet = ActuatorCommandPacket {
            sim_time_s: 0.006,
            step: 6,
            effector_commands: vec![(1, 0.2)],
            engine_throttles: Vec::new(),
            engine_commands: Vec::new(),
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "duplicate-command",
                6,
                None,
                packet_direction(FcTransportPacketDirectionConfig::Command),
                packet_transform(FcTransportPacketTransformConfig::Duplicate),
            )],
        );

        let application = faults.apply_command_frame(&mut packet).unwrap();

        assert_eq!(application.applied_rule_ids, vec!["duplicate-command"]);
        assert_eq!(application.disposition, BridgePacketDisposition::Duplicate);
    }

    #[test]
    fn transport_packet_fault_mapping_delays_sensor_frame() {
        let mut packet = SensorPacket {
            sim_time_s: 0.007,
            step: 7,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "delay-sensor",
                7,
                None,
                packet_direction(FcTransportPacketDirectionConfig::Sensor),
                packet_transform(FcTransportPacketTransformConfig::Delay { steps: 3 }),
            )],
        );

        let application = faults.apply_sensor_frame(&mut packet).unwrap();

        assert_eq!(application.applied_rule_ids, vec!["delay-sensor"]);
        assert_eq!(
            application.disposition,
            BridgePacketDisposition::Delay { steps: 3 }
        );
    }

    #[test]
    fn transport_packet_fault_mapping_bit_flips_command_frame() {
        let mut packet = ActuatorCommandPacket {
            sim_time_s: 0.008,
            step: 8,
            effector_commands: vec![(1, 0.2)],
            engine_throttles: Vec::new(),
            engine_commands: Vec::new(),
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "bit-flip-command",
                8,
                None,
                packet_direction(FcTransportPacketDirectionConfig::Command),
                packet_transform(FcTransportPacketTransformConfig::BitFlip { mask: 0x05 }),
            )],
        );

        let application = faults.apply_command_frame(&mut packet).unwrap();

        assert_eq!(application.applied_rule_ids, vec!["bit-flip-command"]);
        assert_eq!(
            application.disposition,
            BridgePacketDisposition::BitFlip { mask: 0x05 }
        );
    }
}

#[derive(Debug)]
enum BridgeSensor {
    Imu(SyntheticSensorAdapter<SyntheticImu>),
    Barometer(SyntheticSensorAdapter<SyntheticBarometer>),
    AirData(SyntheticSensorAdapter<SyntheticAirData>),
    Gnss(SyntheticSensorAdapter<SyntheticGnss>),
    Magnetometer(SyntheticSensorAdapter<SyntheticMagnetometer>),
    StarTracker(SyntheticSensorAdapter<SyntheticStarTracker>),
    Ideal(SyntheticSensorAdapter<IdealStateSensor>),
}

impl BridgeSensor {
    /// Canonical id of the wrapped sensor (`sensors.<name>`), used to match
    /// SIL fault-injection targets.
    fn sensor_id(&self) -> SensorId {
        match self {
            Self::Imu(s) => s.sensor_id(),
            Self::Barometer(s) => s.sensor_id(),
            Self::AirData(s) => s.sensor_id(),
            Self::Gnss(s) => s.sensor_id(),
            Self::Magnetometer(s) => s.sensor_id(),
            Self::StarTracker(s) => s.sensor_id(),
            Self::Ideal(s) => s.sensor_id(),
        }
    }

    fn prime(&mut self, truth: SensorTruth, step: StepIndex, seed: u64) {
        match self {
            Self::Imu(s) => s.prime(truth, step, seed),
            Self::Barometer(s) => s.prime(truth, step, seed),
            Self::AirData(s) => s.prime(truth, step, seed),
            Self::Gnss(s) => s.prime(truth, step, seed),
            Self::Magnetometer(s) => s.prime(truth, step, seed),
            Self::StarTracker(s) => s.prime(truth, step, seed),
            Self::Ideal(s) => s.prime(truth, step, seed),
        }
    }

    fn read(&mut self) -> Result<openbmp_sensors::Timestamped<SensorMeasurement>, RunnerError> {
        match self {
            Self::Imu(s) => s.read(),
            Self::Barometer(s) => s.read(),
            Self::AirData(s) => s.read(),
            Self::Gnss(s) => s.read(),
            Self::Magnetometer(s) => s.read(),
            Self::StarTracker(s) => s.read(),
            Self::Ideal(s) => s.read(),
        }
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("synthetic sensor read failed: {err}"),
        })
    }

    fn imu_increment_window(
        &self,
        time: openbmp_core::SimTime,
        healthy: bool,
    ) -> Option<ImuIncrementWindow> {
        let Self::Imu(sensor) = self else {
            return None;
        };
        let increments = sensor.inner().last_high_rate_increments();
        if increments.is_empty() {
            return None;
        }
        Some(ImuIncrementWindow {
            time,
            increments: increments
                .iter()
                .map(|increment| ImuInertialIncrement {
                    delta_theta_rad: increment.delta_theta_rad,
                    delta_v_m_s: increment.delta_v_m_s,
                    dt_s: increment.dt_s,
                    seq: increment.seq,
                })
                .collect(),
            healthy,
        })
    }
}

fn require_bridge_frame(document: &ScenarioDocument) -> Result<(), RunnerError> {
    let has_barometer = document
        .sensors
        .as_ref()
        .is_some_and(|sensors| sensors.values().any(|s| s.kind == "barometer"));
    bridge_frame_supported(
        &document.environment.frame_profile,
        &document.environment.atmosphere,
        has_barometer,
    )
    .map_err(|what| RunnerError::UnsupportedScenario { what })
}

fn bridge_sensor_altitude_m(
    position_eci_m: Vector3<f64>,
    geocentric_surface_radius_m: Option<f64>,
) -> f64 {
    atmosphere_altitude_m_with_surface_radius(position_eci_m, geocentric_surface_radius_m)
}

/// Pure decision for whether the FC sensor bridge supports a given
/// `(frame_profile, atmosphere, has_barometer)` configuration. Extracted
/// from [`require_bridge_frame`] so the policy is unit-testable without
/// constructing a full [`ScenarioDocument`].
///
/// `wgs84-uniform-rotation` differs from `toy-fixed-earth` only by a
/// rotation of the ECEF frame about the inertial +z axis. The kernel
/// integrates the dynamics in ECI and the bridge derives sensor truth
/// entirely in ECI: the IMU specific force is `d/dt v_eci - g_eci` (so it
/// senses atmospheric drag correctly through the truth velocity), GNSS and
/// magnetometer truth are ECI quantities, and barometric pressure now uses
/// the same frame-aware altitude helper as the kernel atmosphere path.
///
/// `iers-tabulated` and `iers-cio` are treated identically: they differ from
/// `wgs84-uniform-rotation` only in the ECI↔ECEF transform, which the KERNEL
/// applies to the dynamics (atmospheric co-rotation, gravity). The bridge
/// sensor truth is derived in ECI and so is identical under these rotating
/// frames.
fn bridge_frame_supported(
    frame_profile: &str,
    _atmosphere: &str,
    _has_barometer: bool,
) -> Result<(), String> {
    match frame_profile {
        "toy-fixed-earth" => Ok(()),
        "wgs84-uniform-rotation" | "iers-tabulated" | "iers-cio" => Ok(()),
        other => Err(format!(
            "[fc] bridge requires frame_profile = \"toy-fixed-earth\", \
             \"wgs84-uniform-rotation\", \"iers-tabulated\", or \"iers-cio\"; got `{other}`"
        )),
    }
}

/// Precondition: per-axis rate loops (LQR
/// and INDI) require a single-body assembly with diagonal inertia
/// in body axes. Multi-body assemblies fail closed because solving
/// gains / parameters against the full assembled mass
/// properties is not supported; non-diagonal inertia breaks the per-axis decoupling
/// assumption both rate loops are built on.
///
/// Returns `Ok([Jxx, Jyy, Jzz])` after passing the precondition.
/// Callers only invoke this helper after a per-axis rate loop has
/// been selected. Fails closed when the precondition is violated;
/// the error includes the offending rate-loop kind label.
fn verify_per_axis_rate_loop_preconditions(
    scenario: &Scenario,
    kind_label: &str,
) -> Result<[f64; 3], RunnerError> {
    let bodies = &scenario.document.vehicle.assembly.bodies;
    if bodies.len() != 1 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "rate_loop_kind = \"{kind_label}\" requires exactly one \
                 [[vehicle.assembly.bodies]] entry with diagonal inertia; \
                 got {} bodies",
                bodies.len()
            ),
        });
    }
    let body = bodies
        .first()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!(
                "rate_loop_kind = \"{kind_label}\" requires at least one \
                 [[vehicle.assembly.bodies]] entry"
            ),
        })?;
    let inertia_matrix =
        body.dry_inertia_body_kg_m2
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "rate_loop_kind = \"{kind_label}\" requires \
                     vehicle.assembly.bodies[0].dry_inertia_body_kg_m2 to be declared"
                ),
            })?;
    // Reject non-diagonal inertia: per-axis decoupling only holds
    // for diagonal J in body axes.
    for (i, row) in inertia_matrix.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            if i != j && *value != 0.0 {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!(
                        "rate_loop_kind = \"{kind_label}\" requires diagonal inertia; body[0] \
                         dry_inertia_body_kg_m2[{i}][{j}] = {value} ≠ 0"
                    ),
                });
            }
        }
    }
    Ok([
        inertia_matrix[0][0],
        inertia_matrix[1][1],
        inertia_matrix[2][2],
    ])
}

/// Helper to extract the diagonal moments of inertia
/// from a single-body assembly so the runner can solve the per-axis
/// LQR DARE at scenario load. Returns `Ok(None)` when the FC
/// scenario does not request a rate loop that needs the precondition
/// check. Fails closed when LQR is requested but the precondition
/// (single body, diagonal inertia) is violated; the matching INDI
/// precondition runs through [`verify_per_axis_rate_loop_preconditions`]
/// for its fail-closed side effect.
fn build_autopilot_lqr_context(
    scenario: &Scenario,
) -> Result<Option<FcAutopilotLqrContext>, RunnerError> {
    let Some(fc_config) = &scenario.document.fc else {
        return Ok(None);
    };
    let Some(autopilot_params) = fc_config.autopilot_params.as_ref() else {
        return Ok(None);
    };
    let kind = autopilot_params.rate_loop_kind;
    if kind == Some(openbmp_scenario::FcRateLoopKind::Lqr) {
        let diagonal_inertia_kg_m2 = verify_per_axis_rate_loop_preconditions(scenario, "lqr")?;
        return Ok(Some(FcAutopilotLqrContext {
            dt_s: scenario.document.time.dt_s,
            diagonal_inertia_kg_m2,
        }));
    }
    if kind == Some(openbmp_scenario::FcRateLoopKind::Indi) {
        // INDI shares the precondition (single body, diagonal
        // inertia) but uses scenario-config inertia for its
        // inversion, not the truth-side body inertia. Run the
        // check for its side-effect; LQR-context is None.
        let _ = verify_per_axis_rate_loop_preconditions(scenario, "indi")?;
        return Ok(None);
    }
    Ok(None)
}

/// Helper to derive a control allocator
/// from `[fc.autopilot_allocation]` plus the
/// `[[vehicle.assembly.effectors]]` declarations.
///
/// Returns `Ok(None)` when the FC config has no allocation block, or
/// when the scenario has no `[fc]` block. Walks every
/// `direct_torque` effector in the assembly and groups them by axis.
/// For `prioritised_redistributed`, the optional scenario
/// `axis_priority` is honoured in priority order; absent → the
/// documented default `[roll, yaw, pitch]`. For `pseudo_inverse`, the
/// current direct-axis surface is allocated by a bounded weighted
/// pseudo-inverse.
fn build_autopilot_allocator(
    scenario: &Scenario,
) -> Result<Option<openbmp_fc::allocation::ControlAllocator>, RunnerError> {
    use openbmp_fc::allocation::{
        BodyAxis, ControlAllocator, EffectorAxisAssignment, PrioritisedRedistributedAllocator,
        WeightedPseudoInverseAllocator,
    };
    let Some(fc_config) = &scenario.document.fc else {
        return Ok(None);
    };
    let Some(alloc_cfg) = fc_config.autopilot_allocation.as_ref() else {
        return Ok(None);
    };
    let allocation_kind = alloc_cfg.kind;
    let kind_label = match allocation_kind {
        openbmp_scenario::FcAutopilotAllocationKind::PrioritisedRedistributed => {
            "prioritised_redistributed"
        }
        openbmp_scenario::FcAutopilotAllocationKind::PseudoInverse => "pseudo_inverse",
    };
    // Walk effectors and pull out direct_torque assignments.
    let mut assignments: Vec<EffectorAxisAssignment> = Vec::new();
    for effector in &scenario.document.vehicle.assembly.effectors {
        let openbmp_scenario::EffectorKindConfig::DirectTorque { axis, .. } = effector.kind else {
            continue;
        };
        let body_axis = match axis {
            openbmp_scenario::TorqueAxis::Roll => BodyAxis::Roll,
            openbmp_scenario::TorqueAxis::Pitch => BodyAxis::Pitch,
            openbmp_scenario::TorqueAxis::Yaw => BodyAxis::Yaw,
        };
        // Symmetric box check: the allocator consumes one positive
        // capacity per effector, so asymmetric authority must fail
        // closed instead of being hidden by a tolerance.
        if !limits_are_exactly_symmetric(effector.limits.min, effector.limits.max) {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "fc.autopilot_allocation.kind = \"{kind_label}\" requires symmetric \
                     effector limits; effector \"{}\" has min = {}, max = {}",
                    effector.id, effector.limits.min, effector.limits.max
                ),
            });
        }
        assignments.push(EffectorAxisAssignment {
            effector_id: openbmp_core::EffectorId::from_path(&format!(
                "vehicle.assembly.effectors.{}",
                effector.id
            )),
            axis: body_axis,
            max_abs: effector.limits.max,
        });
    }
    if assignments.is_empty() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "fc.autopilot_allocation.kind = \"{kind_label}\" requires at least one \
                 direct_torque effector in vehicle.assembly.effectors"
            ),
        });
    }
    let allocator = match allocation_kind {
        openbmp_scenario::FcAutopilotAllocationKind::PrioritisedRedistributed => {
            // Resolve axis priority. Default per the scenario block:
            // [roll, yaw, pitch]. Any axis named in `axis_priority`
            // must appear; absent → fall back to the default
            // permutation.
            let priority = if let Some(priority_strs) = alloc_cfg.axis_priority.as_ref() {
                let mut axes = [BodyAxis::Roll, BodyAxis::Yaw, BodyAxis::Pitch];
                // The scenario validator already requires unique
                // entries drawn from {roll, pitch, yaw}; we still
                // defend in depth.
                if priority_strs.len() != 3 {
                    return Err(RunnerError::UnsupportedScenario {
                        what: "fc.autopilot_allocation.axis_priority must list each of \
                               [roll, pitch, yaw] exactly once"
                            .to_owned(),
                    });
                }
                for (slot, label) in axes.iter_mut().zip(priority_strs.iter()) {
                    *slot = match label.as_str() {
                        "roll" => BodyAxis::Roll,
                        "pitch" => BodyAxis::Pitch,
                        "yaw" => BodyAxis::Yaw,
                        other => {
                            return Err(RunnerError::UnsupportedScenario {
                                what: format!(
                                    "fc.autopilot_allocation.axis_priority entry \"{other}\" is \
                                     not a body axis"
                                ),
                            });
                        }
                    };
                }
                axes
            } else {
                [BodyAxis::Roll, BodyAxis::Yaw, BodyAxis::Pitch]
            };
            ControlAllocator::from(
                PrioritisedRedistributedAllocator::new(priority, assignments).map_err(|err| {
                    RunnerError::UnsupportedScenario {
                        what: format!("control allocator construction failed: {err}"),
                    }
                })?,
            )
        }
        openbmp_scenario::FcAutopilotAllocationKind::PseudoInverse => {
            ControlAllocator::from(WeightedPseudoInverseAllocator::new(assignments).map_err(
                |err| RunnerError::UnsupportedScenario {
                    what: format!("control allocator construction failed: {err}"),
                },
            )?)
        }
    };
    Ok(Some(allocator))
}

fn limits_are_exactly_symmetric(min: f64, max: f64) -> bool {
    max.to_bits() == (-min).to_bits()
}

fn build_magnetic_field(config: &FcConfig) -> Result<Box<dyn MagneticFieldEci>, RunnerError> {
    let (kind, epoch) = magnetic_field_settings(config);
    match kind {
        FcMagFieldKind::EarthDipole => Ok(Box::new(EarthDipoleField::default())),
        FcMagFieldKind::Wmm2025 => {
            let model = Wmm2025::new_for_decimal_year(epoch).map_err(|err| {
                RunnerError::UnsupportedScenario {
                    what: format!("WMM 2025 magnetic model rejected epoch {epoch}: {err}"),
                }
            })?;
            Ok(Box::new(model))
        }
    }
}

fn magnetic_field_settings(config: &FcConfig) -> (FcMagFieldKind, f64) {
    if let Some(lanes) = config.estimator_lanes.as_ref()
        && let Some(first_lane) = lanes.lanes.first()
    {
        // `[fc.estimator_lanes]` replaces the top-level estimator
        // selector. The synthetic magnetometer bridge follows the
        // same declaration-order contract as the lane runner and uses
        // the first lane's estimator family to choose the shared truth
        // field model.
        return magnetic_field_settings_for_estimator(config, first_lane.estimator);
    }
    magnetic_field_settings_for_estimator(config, config.estimator)
}

fn magnetic_field_settings_for_estimator(
    config: &FcConfig,
    estimator: FcEstimatorKind,
) -> (FcMagFieldKind, f64) {
    match estimator {
        FcEstimatorKind::Ekf => {
            let cfg = config.ekf.as_ref();
            (
                cfg.and_then(|c| c.mag_field).unwrap_or_default(),
                cfg.and_then(|c| c.mag_epoch_decimal_year)
                    .unwrap_or(DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR),
            )
        }
        FcEstimatorKind::Mekf => {
            let cfg = config.mekf.as_ref();
            (
                cfg.and_then(|c| c.mag_field).unwrap_or_default(),
                cfg.and_then(|c| c.mag_epoch_decimal_year)
                    .unwrap_or(DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR),
            )
        }
        FcEstimatorKind::Imm | FcEstimatorKind::SrUkf | FcEstimatorKind::SrUkfAttitude => {
            // IMM and SR-UKF (full and attitude
            // variants) all use the [fc.ekf] base for the magnetic-field
            // model; per-mode / per-lane overrides do not alter the
            // field-evaluation reference.
            let cfg = config.ekf.as_ref();
            (
                cfg.and_then(|c| c.mag_field).unwrap_or_default(),
                cfg.and_then(|c| c.mag_epoch_decimal_year)
                    .unwrap_or(DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR),
            )
        }
    }
}

fn bridge_specific_force_source(document: &ScenarioDocument) -> SpecificForceSourceConfig {
    document
        .sensors
        .as_ref()
        .and_then(|sensors| {
            sensors
                .values()
                .filter(|sensor| sensor.kind == "imu")
                .filter_map(|sensor| sensor.specific_force_source)
                .max_by_key(|source| match source {
                    SpecificForceSourceConfig::FiniteDifference => 0,
                    SpecificForceSourceConfig::ForceAccumulator => 1,
                    SpecificForceSourceConfig::StationaryRotatingFrame => 2,
                })
        })
        .unwrap_or_default()
}

fn build_sensors(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Vec<BridgeSensor>, RunnerError> {
    let mut sensors = Vec::new();
    let Some(configs) = &document.sensors else {
        return Ok(sensors);
    };
    for (name, config) in configs {
        sensors.push(build_sensor(
            name,
            config,
            document.time.dt_s,
            resolved_files,
        )?);
    }
    Ok(sensors)
}

fn build_sensor(
    name: &str,
    config: &SensorConfig,
    dt_s: f64,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<BridgeSensor, RunnerError> {
    let id = SensorId::from_path(&format!("sensors.{name}"));
    match config.kind.as_str() {
        "ideal_state" => Ok(BridgeSensor::Ideal(SyntheticSensorAdapter::new(
            IdealStateSensor::new(id),
        ))),
        "imu" => {
            let budget = ImuNoiseBudget::load_from_str(sensor_text(name, resolved_files)?)?;
            let mut sensor = SyntheticImu::new(id, budget)?;
            if let Some(high_rate) = config.high_rate {
                let high_rate = HighRateImuConfig::new(
                    high_rate.sub_samples,
                    high_rate.delta_theta_lsb_rad,
                    high_rate.delta_v_lsb_m_s,
                )?;
                sensor = sensor.with_high_rate(high_rate);
            }
            Ok(BridgeSensor::Imu(SyntheticSensorAdapter::new(sensor)))
        }
        "barometer" => {
            let budget = parse_baro_budget(sensor_text(name, resolved_files)?, dt_s)?;
            let sensor = SyntheticBarometer::new(id, budget.0, budget.1, budget.2, dt_s)?;
            Ok(BridgeSensor::Barometer(SyntheticSensorAdapter::new(sensor)))
        }
        "airdata" => {
            let budget = AirDataNoiseBudget::load_from_str(sensor_text(name, resolved_files)?)?;
            let sensor = SyntheticAirData::new(id, budget);
            Ok(BridgeSensor::AirData(SyntheticSensorAdapter::new(sensor)))
        }
        "gnss" => {
            let budget = parse_gnss_budget(sensor_text(name, resolved_files)?, dt_s)?;
            let sensor = SyntheticGnss::new(id, budget)?;
            Ok(BridgeSensor::Gnss(SyntheticSensorAdapter::new(sensor)))
        }
        "magnetometer" => {
            let budget = parse_magnetometer_budget(sensor_text(name, resolved_files)?)?;
            let sensor = SyntheticMagnetometer::new(id, budget);
            Ok(BridgeSensor::Magnetometer(SyntheticSensorAdapter::new(
                sensor,
            )))
        }
        "star_tracker" => {
            let budget = parse_star_tracker_budget(sensor_text(name, resolved_files)?)?;
            let sensor = SyntheticStarTracker::new(id, budget);
            Ok(BridgeSensor::StarTracker(SyntheticSensorAdapter::new(
                sensor,
            )))
        }
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("unsupported FC bridge sensor kind `{other}`"),
        }),
    }
}

fn sensor_text<'a>(
    name: &str,
    resolved_files: &'a BTreeMap<String, ResolvedFile>,
) -> Result<&'a str, RunnerError> {
    let key = format!("sensors.{name}.file");
    let resolved = resolved_files
        .get(&key)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("sensor budget `{key}` was not resolved"),
        })?;
    std::str::from_utf8(&resolved.bytes).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("sensor budget `{key}` is not UTF-8: {err}"),
    })
}

fn parse_toml_budget(text: &str) -> Result<toml::Value, RunnerError> {
    // In `toml` 1.x `text.parse::<toml::Value>()`
    // expects a single TOML scalar / inline-table / array, not a
    // top-level document — leading comments + a `key = value` line
    // surface as "unexpected content, expected nothing". Parse as a
    // `Table` (the canonical document shape) and wrap it back into a
    // `Value` so the existing `as_table` consumers keep working.
    let table =
        toml::from_str::<toml::Table>(text).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("sensor budget TOML parse failed: {err}"),
        })?;
    Ok(toml::Value::Table(table))
}

fn toml_number_as_f64(value: &toml::Value) -> Option<f64> {
    value.as_float().or_else(|| {
        value
            .as_integer()
            .and_then(|integer| integer.to_string().parse::<f64>().ok())
    })
}

fn array3(table: &toml::value::Table, key: &str) -> Result<[f64; 3], RunnerError> {
    let value = table
        .get(key)
        .and_then(toml::Value::as_array)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("sensor budget missing array `{key}`"),
        })?;
    if value.len() != 3 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("sensor budget `{key}` must contain 3 values"),
        });
    }
    let mut out = [0.0; 3];
    for (slot, item) in out.iter_mut().zip(value) {
        *slot = toml_number_as_f64(item).ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("sensor budget `{key}` contains a non-number"),
        })?;
    }
    Ok(out)
}

fn matrix3(table: &toml::value::Table, key: &str) -> Result<[[f64; 3]; 3], RunnerError> {
    let value = table
        .get(key)
        .and_then(toml::Value::as_array)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("sensor budget missing matrix `{key}`"),
        })?;
    if value.len() != 3 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("sensor budget `{key}` must contain 3 rows"),
        });
    }
    let mut out = [[0.0; 3]; 3];
    for (row, item) in out.iter_mut().zip(value) {
        let row_values = item
            .as_array()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!("sensor budget `{key}` contains a non-array row"),
            })?;
        if row_values.len() != 3 {
            return Err(RunnerError::UnsupportedScenario {
                what: format!("sensor budget `{key}` rows must contain 3 values"),
            });
        }
        for (slot, item) in row.iter_mut().zip(row_values) {
            *slot = toml_number_as_f64(item).ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!("sensor budget `{key}` contains a non-number"),
            })?;
        }
    }
    Ok(out)
}

fn budget_table(value: &toml::Value) -> Result<&toml::value::Table, RunnerError> {
    value
        .get("budget")
        .and_then(toml::Value::as_table)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "sensor budget missing [budget] table".to_owned(),
        })
}

fn parse_gnss_budget(text: &str, dt_s: f64) -> Result<GnssNoiseBudget, RunnerError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    let mut budget = GnssNoiseBudget::new(
        array3(table, "sigma_position_m")?,
        array3(table, "sigma_velocity_m_s")?,
        array3(table, "position_bias_ou_theta_per_s")?,
        array3(table, "position_bias_ou_sigma_m_sqrt_s")?,
        dt_s,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("GNSS budget rejected: {err}"),
    })?;
    let mount_offset_body_m =
        optional_array3(table, "mount_offset_body_m")?.map_or_else(Vector3::zeros, Vector3::from);
    let clock_bias_s = optional_number(table, "clock_bias_s")?.unwrap_or(0.0);
    let clock_drift_s_per_s = optional_number(table, "clock_drift_s_per_s")?.unwrap_or(0.0);
    let fixed_latency_s = optional_number(table, "fixed_latency_s")?.unwrap_or(0.0);
    budget = budget
        .with_deterministic_errors(
            mount_offset_body_m,
            clock_bias_s,
            clock_drift_s_per_s,
            fixed_latency_s,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("GNSS deterministic error budget rejected: {err}"),
        })?;
    let clock_bias_rw_sigma_s_sqrt_s =
        optional_number(table, "clock_bias_rw_sigma_s_sqrt_s")?.unwrap_or(0.0);
    let clock_drift_rw_sigma_s_per_s_sqrt_s =
        optional_number(table, "clock_drift_rw_sigma_s_per_s_sqrt_s")?.unwrap_or(0.0);
    budget
        .with_clock_random_walk(
            clock_bias_rw_sigma_s_sqrt_s,
            clock_drift_rw_sigma_s_per_s_sqrt_s,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("GNSS clock random-walk budget rejected: {err}"),
        })
}

fn optional_number(table: &toml::value::Table, key: &str) -> Result<Option<f64>, RunnerError> {
    table
        .get(key)
        .map(|value| {
            toml_number_as_f64(value).ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!("sensor budget `{key}` contains a non-number"),
            })
        })
        .transpose()
}

fn optional_array3(table: &toml::value::Table, key: &str) -> Result<Option<[f64; 3]>, RunnerError> {
    if !table.contains_key(key) {
        return Ok(None);
    }
    array3(table, key).map(Some)
}

fn parse_magnetometer_budget(text: &str) -> Result<MagnetometerNoiseBudget, RunnerError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    MagnetometerNoiseBudget::new(
        array3(table, "sigma_body_nt")?,
        array3(table, "hard_iron_body_nt")?,
        matrix3(table, "soft_iron_body")?,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("magnetometer budget rejected: {err}"),
    })
}

fn parse_star_tracker_budget(text: &str) -> Result<StarTrackerNoiseBudget, RunnerError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    let sigma_arcsec = table
        .get("sigma_per_axis_arcsec")
        .and_then(toml_number_as_f64)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "star-tracker budget missing `sigma_per_axis_arcsec`".to_owned(),
        })?;
    StarTrackerNoiseBudget::from_arcsec(sigma_arcsec).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!("star-tracker budget rejected: {err}"),
        }
    })
}

fn parse_baro_budget(text: &str, dt_s: f64) -> Result<(f64, f64, f64), RunnerError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    let get = |key: &str| -> Result<f64, RunnerError> {
        table.get(key).and_then(toml_number_as_f64).ok_or_else(|| {
            RunnerError::UnsupportedScenario {
                what: format!("barometer budget missing `{key}`"),
            }
        })
    };
    let measurement_stddev_pa = get("measurement_stddev_pa")?;
    let bias_theta = get("bias_ou_theta_per_s")?;
    let bias_sigma = get("bias_ou_sigma_pa_sqrt_s")?;
    if dt_s <= 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "barometer sensor dt must be positive".to_owned(),
        });
    }
    Ok((measurement_stddev_pa, bias_theta, bias_sigma))
}

impl From<openbmp_sensors::SensorError> for RunnerError {
    fn from(value: openbmp_sensors::SensorError) -> Self {
        Self::UnsupportedScenario {
            what: format!("sensor bridge error: {value}"),
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    const ALLOCATOR_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/diff-flatness-figure-eight-allocator/scenario.toml"
    ));
    const FC_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
    ));

    #[test]
    fn comm_latency_step_delay_quantizes_by_runner_dt() {
        assert_eq!(comm_latency_step_delay(0.0, 0.001).unwrap(), 0);
        assert_eq!(comm_latency_step_delay(0.000_999, 0.001).unwrap(), 0);
        assert_eq!(comm_latency_step_delay(0.001, 0.001).unwrap(), 1);
        assert_eq!(comm_latency_step_delay(0.003_9, 0.001).unwrap(), 3);
        assert_eq!(comm_arrival_step(10, 0.003_9, 0.001).unwrap(), 13);
    }

    #[test]
    fn comm_latency_step_delay_rejects_invalid_inputs() {
        assert!(matches!(
            comm_latency_step_delay(f64::NAN, 0.001),
            Err(RunnerError::UnsupportedScenario { .. })
        ));
        assert!(matches!(
            comm_latency_step_delay(-0.001, 0.001),
            Err(RunnerError::UnsupportedScenario { .. })
        ));
        assert!(matches!(
            comm_latency_step_delay(0.001, 0.0),
            Err(RunnerError::UnsupportedScenario { .. })
        ));
        assert!(matches!(
            comm_arrival_step(u64::MAX, 0.001, 0.001),
            Err(RunnerError::UnsupportedScenario { .. })
        ));
    }

    #[test]
    fn comm_frame_queue_releases_oldest_due_sensor_and_command() {
        let mut queues = CommLinkFrameQueues::default();
        queues.enqueue_sensor(
            3,
            SensorPacket {
                step: 30,
                ..SensorPacket::default()
            },
        );
        queues.enqueue_sensor(
            1,
            SensorPacket {
                step: 10,
                ..SensorPacket::default()
            },
        );
        queues.enqueue_command(
            2,
            ActuatorCommandPacket {
                step: 20,
                sim_time_s: 0.02,
                effector_commands: Vec::new(),
                engine_throttles: Vec::new(),
                engine_commands: Vec::new(),
            },
        );

        assert!(queues.pop_due_sensor(0).is_none());
        assert_eq!(queues.pop_due_sensor(1).expect("due sensor").step, 10);
        assert!(queues.pop_due_sensor(2).is_none());
        assert_eq!(queues.pop_due_command(2).expect("due command").step, 20);
        assert_eq!(queues.pop_due_sensor(3).expect("later sensor").step, 30);
        assert!(queues.pop_due_command(3).is_none());
    }

    #[test]
    fn comm_link_effect_drops_sensor_frame_fail_closed() {
        let err = apply_comm_link_packet_effect(
            "sensor",
            4,
            "s-band",
            &LinkPacketEffect {
                disposition: LinkPacketDisposition::Drop,
                propagation_delay_s: 1.0,
                processing_delay_s: 0.0,
                latency_s: 1.0,
                error_draw: Some(0.25),
            },
        )
        .expect_err("drop disposition fails closed");
        assert!(
            matches!(
                err,
                RunnerError::UnsupportedScenario { ref what }
                    if what.contains("comm link `s-band` dropped fc.transport sensor frame step 4")
                        && what.contains("fer_draw = 2.50000000000000000e-1")
            ),
            "expected comm-link drop UnsupportedScenario, got {err:?}"
        );
    }

    #[test]
    fn comm_link_effect_corrupts_command_frame_fail_closed() {
        let err = apply_comm_link_packet_effect(
            "command",
            7,
            "s-band",
            &LinkPacketEffect {
                disposition: LinkPacketDisposition::BitFlip { mask: 0xa5 },
                propagation_delay_s: 1.0,
                processing_delay_s: 0.02,
                latency_s: 1.02,
                error_draw: Some(0.75),
            },
        )
        .expect_err("bit-flip disposition fails closed");
        assert!(
            matches!(
                err,
                RunnerError::UnsupportedScenario { ref what }
                    if what.contains("comm link `s-band` corrupted fc.transport command frame step 7")
                        && what.contains("bit-flip mask 0xa5")
            ),
            "expected comm-link bit-flip UnsupportedScenario, got {err:?}"
        );
    }

    #[test]
    fn allocator_builder_rejects_even_tiny_asymmetric_limits() {
        let toml = ALLOCATOR_SCENARIO.replacen(
            "limits           = { min = -0.2, max = 0.2",
            "limits           = { min = -0.2000000000001, max = 0.2",
            1,
        );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let err = build_autopilot_allocator(&scenario).expect_err("asymmetric limits rejected");
        assert!(
            matches!(err, RunnerError::UnsupportedScenario { ref what } if what.contains("requires symmetric effector limits")),
            "expected symmetric-limit UnsupportedScenario, got {err:?}"
        );
    }

    #[test]
    fn allocator_builder_consumes_pseudo_inverse_kind() {
        let toml = ALLOCATOR_SCENARIO.replace(
            "kind          = \"prioritised_redistributed\"",
            "kind          = \"pseudo_inverse\"",
        );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let allocator = build_autopilot_allocator(&scenario)
            .expect("pseudo-inverse allocator builds")
            .expect("allocator present");
        assert!(matches!(
            allocator,
            openbmp_fc::allocation::ControlAllocator::WeightedPseudoInverse(_)
        ));
    }

    #[test]
    fn magnetic_field_settings_follow_first_estimator_lane() {
        let lanes = r#"
[fc.estimator_lanes]
voter = "simplex_pass_through"

[[fc.estimator_lanes.lane]]
id        = "ekf_lane"
estimator = "ekf"
"#;
        let toml = format!(
            "{}\n{lanes}",
            FC_SCENARIO
                .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
                .replace("estimator        = \"ekf\"", "estimator        = \"mekf\"")
        );
        let scenario = Scenario::from_toml_str(&toml).expect("lane scenario validates");
        let fc = scenario.document.fc.as_ref().expect("fc block present");
        let (kind, epoch) = magnetic_field_settings(fc);
        assert_eq!(kind, FcMagFieldKind::Wmm2025);
        assert_eq!(epoch.to_bits(), 2025.0f64.to_bits());
    }

    #[test]
    fn bridge_sensor_altitude_is_frame_aware() {
        // Local-frame launch (near origin): preserve the legacy flat-earth
        // altitude convention.
        assert_eq!(
            bridge_sensor_altitude_m(Vector3::new(0.0, 0.0, 1_000.0), None),
            1_000.0
        );

        // Geocentric launch: use radius above the Earth model, not ECI z.
        let surface = Vector3::new(6_371_000.0, 0.0, 0.0);
        assert_eq!(bridge_sensor_altitude_m(surface, None), 0.0);
        let up_100km = Vector3::new(6_371_000.0 + 100_000.0, 0.0, 0.0);
        assert!((bridge_sensor_altitude_m(up_100km, None) - 100_000.0).abs() < 1.0e-6);

        let rounded_surface = Vector3::new(6_370_000.0, 0.0, 0.0);
        assert_eq!(
            bridge_sensor_altitude_m(rounded_surface, Some(6_370_000.0)),
            0.0
        );
        let rounded_up_1km = Vector3::new(6_371_000.0, 0.0, 0.0);
        assert!(
            (bridge_sensor_altitude_m(rounded_up_1km, Some(6_370_000.0)) - 1_000.0).abs() < 1.0e-6
        );
    }

    #[test]
    fn build_sensor_accepts_airdata_budget() {
        let budget = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/sensors/airdata-textbook.toml"
        ));
        let config = SensorConfig {
            kind: "airdata".to_owned(),
            specific_force_source: None,
            high_rate: None,
            file: Some("airdata-textbook.toml".into()),
            file_sha256: None,
        };
        let resolved = BTreeMap::from([(
            "sensors.airdata.file".to_owned(),
            ResolvedFile::from_bytes("airdata-textbook.toml", budget.as_bytes().to_vec()),
        )]);

        let sensor = build_sensor("airdata", &config, 0.01, &resolved).expect("airdata sensor");

        assert!(matches!(sensor, BridgeSensor::AirData(_)));
    }

    #[test]
    fn build_sensor_accepts_imu_high_rate_config() {
        let budget = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../crates/openbmp-sensors/tests/fixtures/minimal-imu-budget.toml"
        ));
        let config = SensorConfig {
            kind: "imu".to_owned(),
            specific_force_source: None,
            high_rate: Some(openbmp_scenario::SensorHighRateImuConfig {
                sub_samples: 4,
                delta_theta_lsb_rad: 1.0e-6,
                delta_v_lsb_m_s: 1.0e-5,
            }),
            file: Some("minimal-imu-budget.toml".into()),
            file_sha256: None,
        };
        let resolved = BTreeMap::from([(
            "sensors.imu.file".to_owned(),
            ResolvedFile::from_bytes("minimal-imu-budget.toml", budget.as_bytes().to_vec()),
        )]);

        let sensor = build_sensor("imu", &config, 0.01, &resolved).expect("imu sensor");

        let BridgeSensor::Imu(sensor) = sensor else {
            panic!("expected imu sensor");
        };
        assert_eq!(
            sensor.inner().high_rate_config(),
            Some(HighRateImuConfig::new(4, 1.0e-6, 1.0e-5).unwrap())
        );
    }

    #[test]
    fn bridge_sensor_reports_imu_increment_window_after_read() {
        let budget = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../crates/openbmp-sensors/tests/fixtures/minimal-imu-budget.toml"
        ));
        let config = SensorConfig {
            kind: "imu".to_owned(),
            specific_force_source: None,
            high_rate: Some(openbmp_scenario::SensorHighRateImuConfig {
                sub_samples: 4,
                delta_theta_lsb_rad: 1.0e-6,
                delta_v_lsb_m_s: 1.0e-5,
            }),
            file: Some("minimal-imu-budget.toml".into()),
            file_sha256: None,
        };
        let resolved = BTreeMap::from([(
            "sensors.imu.file".to_owned(),
            ResolvedFile::from_bytes("minimal-imu-budget.toml", budget.as_bytes().to_vec()),
        )]);
        let mut sensor = build_sensor("imu", &config, 0.001, &resolved).expect("imu sensor");
        let time = openbmp_core::SimTime::from_seconds(0.125);
        let truth = SensorTruth {
            position_eci: Position3::new(0.0, 0.0, 0.0),
            velocity_eci: Velocity3::new(0.0, 0.0, 0.0),
            attitude_eci_to_body: UnitQuaternion::identity(),
            angular_velocity_body_rad_s: Vector3::new(0.2, -0.4, 0.8),
            angular_acceleration_body_rad_s2: Vector3::zeros(),
            specific_force_body_m_s2: Vector3::new(8.0, -4.0, 2.0),
            static_pressure_pa: 101_325.0,
            atmosphere_density_kg_m3: 1.225,
            speed_of_sound_m_s: 340.3,
            air_relative_velocity_body_m_s: Vector3::zeros(),
            altitude_geometric_m: 0.0,
            magnetic_field_body_nt: Vector3::zeros(),
            time,
        };

        sensor.prime(truth, StepIndex::new(7), 42);
        let measurement = sensor.read().expect("imu measurement");
        let window = sensor
            .imu_increment_window(measurement.time, true)
            .expect("increment window");

        assert_eq!(window.time, time);
        assert!(window.healthy);
        assert_eq!(window.increments.len(), 4);
        assert_eq!(window.increments[0].seq, 28);
        assert_eq!(window.increments[3].seq, 31);
        assert_eq!(window.increments[0].dt_s.to_bits(), 0.00025_f64.to_bits());
        assert!(
            (window.increments[0].delta_theta_rad - Vector3::new(50.0e-6, -100.0e-6, 200.0e-6))
                .norm()
                < 1.0e-15
        );
        assert!(
            (window.increments[0].delta_v_m_s - Vector3::new(0.002, -0.001, 0.0005)).norm()
                < 1.0e-15
        );
        assert!(window.increments[0].delta_theta_rad == window.increments[3].delta_theta_rad);
        assert!(window.increments[0].delta_v_m_s == window.increments[3].delta_v_m_s);
    }

    #[test]
    fn append_bridge_measurement_serializes_airdata_fields() {
        let mut packet = SensorPacket::default();
        let measurement = SensorMeasurement::AirData {
            static_pressure_pa: 88_500.0,
            impact_pressure_pa: 1_250.0,
            mach: 0.15,
            calibrated_airspeed_m_s: 51.0,
            true_airspeed_m_s: 52.0,
            angle_of_attack_rad: 0.02,
            sideslip_rad: -0.01,
            pressure_altitude_m: 1_100.0,
        };

        let representable =
            append_bridge_measurement(&mut packet, &measurement).expect("append airdata");

        assert!(representable);
        assert_eq!(
            packet.airdata_static_pressure_pa.map(f64::to_bits),
            Some(88_500.0_f64.to_bits())
        );
        assert_eq!(
            packet.airdata_impact_pressure_pa.map(f64::to_bits),
            Some(1_250.0_f64.to_bits())
        );
        assert_eq!(
            packet.airdata_mach.map(f64::to_bits),
            Some(0.15_f64.to_bits())
        );
        assert_eq!(
            packet.airdata_angle_of_attack_rad.map(f64::to_bits),
            Some(0.02_f64.to_bits())
        );
        assert_eq!(
            packet.airdata_sideslip_rad.map(f64::to_bits),
            Some((-0.01_f64).to_bits())
        );
        assert_eq!(
            packet.airdata_pressure_altitude_m.map(f64::to_bits),
            Some(1_100.0_f64.to_bits())
        );
    }

    #[test]
    fn propellant_state_from_tanks_reports_remaining_fraction() {
        let tanks = BTreeMap::from([
            (
                openbmp_core::TankId::from_path("vehicle.assembly.tanks.fuel"),
                openbmp_vehicle::PropellantTankState {
                    fluid_remaining_kg: 30.0,
                    initial_fluid_mass_kg: 80.0,
                    volume_m3: 1.0,
                    density_kg_m3: 800.0,
                    has_ullage: false,
                    ullage_gamma: 1.4,
                },
            ),
            (
                openbmp_core::TankId::from_path("vehicle.assembly.tanks.oxidizer"),
                openbmp_vehicle::PropellantTankState {
                    fluid_remaining_kg: 20.0,
                    initial_fluid_mass_kg: 120.0,
                    volume_m3: 1.0,
                    density_kg_m3: 1_000.0,
                    has_ullage: false,
                    ullage_gamma: 1.4,
                },
            ),
        ]);

        let state = propellant_state_from_tanks(openbmp_core::SimTime::from_seconds(3.0), &tanks)
            .expect("propellant state");

        assert_eq!(state.time, openbmp_core::SimTime::from_seconds(3.0));
        assert_eq!(state.mass_remaining_kg.to_bits(), 50.0_f64.to_bits());
        assert_eq!(state.mass_initial_kg.to_bits(), 200.0_f64.to_bits());
        assert_eq!(state.mass_fraction.to_bits(), 0.25_f64.to_bits());
        assert!(!state.depleted);
    }

    #[test]
    fn bridge_frame_policy_allows_rotating_frame_with_barometer() {
        // toy-fixed-earth is always supported, with or without a baro.
        assert!(bridge_frame_supported("toy-fixed-earth", "none", false).is_ok());
        assert!(bridge_frame_supported("toy-fixed-earth", "us_standard_1976", true).is_ok());

        // wgs84-uniform-rotation in vacuum is fine regardless of baro.
        assert!(bridge_frame_supported("wgs84-uniform-rotation", "none", false).is_ok());
        assert!(bridge_frame_supported("wgs84-uniform-rotation", "none", true).is_ok());

        // wgs84-uniform-rotation + atmosphere is supported with and without
        // a barometer because bridge sensor truth now uses frame-aware
        // altitude for pressure.
        assert!(
            bridge_frame_supported("wgs84-uniform-rotation", "us_standard_1976", false).is_ok()
        );
        assert!(bridge_frame_supported("wgs84-uniform-rotation", "us_standard_1976", true).is_ok());

        // IERS profiles share the rotating-Earth bridge contract: the
        // sensor truth is frame-invariant in ECI, so they are accepted under
        // the same conditions as wgs84-uniform-rotation.
        assert!(bridge_frame_supported("iers-tabulated", "none", true).is_ok());
        assert!(bridge_frame_supported("iers-tabulated", "us_standard_1976", false).is_ok());
        assert!(bridge_frame_supported("iers-tabulated", "us_standard_1976", true).is_ok());
        assert!(bridge_frame_supported("iers-cio", "none", true).is_ok());
        assert!(bridge_frame_supported("iers-cio", "us_standard_1976", false).is_ok());
        assert!(bridge_frame_supported("iers-cio", "us_standard_1976", true).is_ok());

        // A genuinely unsupported frame is still rejected.
        let err = bridge_frame_supported("spice-reference", "none", false)
            .expect_err("spice-reference is rejected");
        assert!(err.contains("toy-fixed-earth"), "unexpected message: {err}");
    }
}
