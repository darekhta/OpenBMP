//! Phase 4.B FC runner — bridges a parsed `[fc]` block to a fully
//! constructed [`FlightController`] with all academic algorithms wired
//! up.
//!
//! The runner is intentionally a thin adapter: it owns the
//! [`FlightController`], registers the canonical bus topics, registers
//! the per-tick jobs that map to the configured estimator / autopilot
//! / guidance / mixer / health / FDIR, and exposes accessors so the
//! kernel-side code can publish sensor samples and read the gated
//! actuator/engine commands without depending on `openbmp-fc` types
//! directly.

use std::collections::BTreeMap;

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_core::{SimTime, StepIndex};
use openbmp_fc::autopilot::{
    GainSchedule, PidGains, ThreeLoopAutopilot, ThreeLoopGains, default_gains,
};
use openbmp_fc::commander::{Commander, CommanderParams};
use openbmp_fc::estimator::{Ekf, EkfParams, EstimatorJob, Mekf, MekfParams};
use openbmp_fc::fdir::{FdirJob, FdirParams};
use openbmp_fc::guidance::{
    AttitudeHoldGuidance, GuidanceParams, WaypointGuidance, WaypointSequence,
};
use openbmp_fc::health::{HealthMonitor, HealthParams};
use openbmp_fc::mixer::{Mixer, PhaseAuthority, PhaseAuthorityTable};
use openbmp_fc::topics::{
    ActuatorCommand, AttitudeEstimate, BarometerSample, EngineDemand, EstimatorStatus,
    FailsafeFlags, FdirStatus, GnssSample, ImuSample, MagnetometerSample, PositionEstimate,
    ReferenceState, StarTrackerSample, VehicleStatus,
};
use openbmp_fc::{DispatchSummary, FlightController, FlightControllerBuilder};
use openbmp_mission::{EventBinding, MissionPhaseGraph, PhaseId};
use openbmp_scenario::{
    FcAutopilotKind, FcConfig, FcEkfConfig, FcEstimatorKind, FcGainsConfig, FcGuidanceKind,
    FcMekfConfig, FcPhaseAuthorityConfig,
};

/// Bridges an [`FcConfig`] to a fully wired [`FlightController`].
///
/// The runner owns the controller; the kernel pushes sensor samples
/// in via [`FcRunner::publish_imu`] / [`FcRunner::publish_gnss`] /
/// etc., calls [`FcRunner::step`] once per kernel tick, and reads the
/// resulting actuator / engine demands via
/// [`FcRunner::latest_actuator_command`] /
/// [`FcRunner::latest_engine_command`].
#[derive(Debug)]
pub struct FcRunner {
    fc: FlightController,
}

impl FcRunner {
    /// Builds the runner from the parsed `[fc]` block. The kernel is
    /// responsible for constructing a mission graph and event bindings
    /// (Phase 4.B reuses the runner-side mission builder).
    ///
    /// # Errors
    ///
    /// Returns the underlying controller error if topic registration
    /// or job registration fails.
    #[allow(clippy::too_many_lines)]
    pub fn new(
        config: &FcConfig,
        mission_graph: MissionPhaseGraph,
        event_bindings: Vec<EventBinding>,
        start_phase: PhaseId,
    ) -> Result<Self, openbmp_fc::ControllerError> {
        let mut fc = FlightControllerBuilder::new()
            .frame_budget_us(2_000)
            .build();

        Self::register_canonical_topics(&fc)?;

        // Tick-rate ratio: the FC scheduler runs every kernel tick.
        // `base_rate_hz` is informational at this layer (the kernel
        // already drives at a known dt); we use it to translate
        // higher-level "10 Hz guidance" into a tick count.
        let kernel_tick_per_fc_tick = (1_000 / u64::from(config.base_rate_hz)).max(1);
        let _ = kernel_tick_per_fc_tick;
        let mut next_priority = 5_u8;
        match config.estimator {
            FcEstimatorKind::Ekf => {
                let mut params = EkfParams::default();
                if let Some(ekf_cfg) = &config.ekf {
                    apply_ekf_overrides(&mut params, ekf_cfg);
                }
                let mut ekf = Ekf::new(params);
                ekf.seed(
                    Vector3::zeros(),
                    Vector3::zeros(),
                    UnitQuaternion::identity(),
                );
                fc.scheduler_mut().register_periodic(
                    1,
                    200,
                    next_priority,
                    Box::new(EstimatorJob::new(ekf)),
                )?;
            }
            FcEstimatorKind::Mekf => {
                let mut params = MekfParams::default();
                if let Some(mekf_cfg) = &config.mekf {
                    apply_mekf_overrides(&mut params, mekf_cfg);
                }
                let mut mekf = Mekf::new(params);
                mekf.seed(UnitQuaternion::identity());
                fc.scheduler_mut().register_periodic(
                    1,
                    200,
                    next_priority,
                    Box::new(EstimatorJob::new(mekf)),
                )?;
            }
        }
        next_priority = next_priority.saturating_add(5);

        // Guidance: 10× slower than the base FC tick (i.e. 100 Hz at
        // 1 kHz base rate).
        match config.guidance {
            FcGuidanceKind::AttitudeHold => {
                let q = config.reference_q_xyzw.unwrap_or([0.0, 0.0, 0.0, 1.0]);
                fc.scheduler_mut().register_periodic(
                    10,
                    100,
                    next_priority,
                    Box::new(AttitudeHoldGuidance::new(q)),
                )?;
            }
            FcGuidanceKind::Waypoint => {
                let sequence = WaypointSequence {
                    waypoints: Vec::new(),
                };
                fc.scheduler_mut().register_periodic(
                    10,
                    100,
                    next_priority,
                    Box::new(WaypointGuidance::new(sequence, GuidanceParams::default())),
                )?;
            }
        }
        next_priority = next_priority.saturating_add(5);

        // Commander.
        let commander = Commander::new(
            mission_graph,
            event_bindings,
            start_phase,
            CommanderParams::default(),
        )
        .map_err(openbmp_fc::ControllerError::from)?;
        fc.scheduler_mut()
            .register_periodic(1, 200, next_priority, Box::new(commander))?;
        next_priority = next_priority.saturating_add(5);

        // Autopilot.
        let schedule = build_gain_schedule(config.gain_schedule.as_ref());
        let autopilot = match config.autopilot {
            FcAutopilotKind::ThreeLoop => ThreeLoopAutopilot::with_schedule(schedule),
        };
        fc.scheduler_mut()
            .register_periodic(1, 300, next_priority, Box::new(autopilot))?;
        next_priority = next_priority.saturating_add(5);

        // Mixer.
        let authority = build_authority(config.phase_authority.as_ref());
        let mixer = Mixer::new().with_authority(authority);
        fc.scheduler_mut()
            .register_periodic(1, 100, next_priority, Box::new(mixer))?;
        next_priority = next_priority.saturating_add(5);

        // Health monitor (10 Hz at 1 kHz base rate).
        fc.scheduler_mut().register_periodic(
            10,
            100,
            next_priority,
            Box::new(HealthMonitor::new(HealthParams::default())),
        )?;
        next_priority = next_priority.saturating_add(5);

        // FDIR (10 Hz).
        fc.scheduler_mut().register_periodic(
            10,
            100,
            next_priority,
            Box::new(FdirJob::new(FdirParams::default())),
        )?;

        Ok(Self { fc })
    }

    /// Steps the controller once. Call after the kernel has published
    /// every sensor sample for this tick.
    ///
    /// # Errors
    ///
    /// Propagates any error returned by a registered job.
    pub fn step(
        &mut self,
        time: SimTime,
        tick: StepIndex,
    ) -> Result<DispatchSummary, openbmp_fc::ControllerError> {
        self.fc.step(time, tick)
    }

    /// Borrows the underlying flight controller.
    #[must_use]
    pub fn controller(&self) -> &FlightController {
        &self.fc
    }

    /// Publishes an IMU sample on the controller's bus.
    pub fn publish_imu(&self, sample: ImuSample) {
        let _ = self.fc.bus().publish(sample);
    }

    /// Publishes a GNSS sample.
    pub fn publish_gnss(&self, sample: GnssSample) {
        let _ = self.fc.bus().publish(sample);
    }

    /// Publishes a barometer sample.
    pub fn publish_barometer(&self, sample: BarometerSample) {
        let _ = self.fc.bus().publish(sample);
    }

    /// Publishes a magnetometer sample.
    pub fn publish_magnetometer(&self, sample: MagnetometerSample) {
        let _ = self.fc.bus().publish(sample);
    }

    /// Returns the latest gated actuator command, if any.
    #[must_use]
    pub fn latest_actuator_command(&self) -> Option<ActuatorCommand> {
        self.fc
            .bus()
            .latest::<ActuatorCommand>()
            .ok()
            .flatten()
            .map(|(c, _)| c)
    }

    /// Returns the latest gated engine demand, if any.
    #[must_use]
    pub fn latest_engine_command(&self) -> Option<EngineDemand> {
        self.fc
            .bus()
            .latest::<EngineDemand>()
            .ok()
            .flatten()
            .map(|(c, _)| c)
    }

    /// Returns the latest vehicle status, if any.
    #[must_use]
    pub fn latest_vehicle_status(&self) -> Option<VehicleStatus> {
        self.fc
            .bus()
            .latest::<VehicleStatus>()
            .ok()
            .flatten()
            .map(|(s, _)| s)
    }

    /// Returns the latest failsafe-flag publication, if any.
    #[must_use]
    pub fn latest_failsafe_flags(&self) -> Option<FailsafeFlags> {
        self.fc
            .bus()
            .latest::<FailsafeFlags>()
            .ok()
            .flatten()
            .map(|(f, _)| f)
    }

    /// Returns the latest attitude estimate, if any.
    #[must_use]
    pub fn latest_attitude_estimate(&self) -> Option<AttitudeEstimate> {
        self.fc
            .bus()
            .latest::<AttitudeEstimate>()
            .ok()
            .flatten()
            .map(|(a, _)| a)
    }

    fn register_canonical_topics(fc: &FlightController) -> Result<(), openbmp_fc::ControllerError> {
        let bus = fc.bus();
        bus.register::<ImuSample>()?;
        bus.register::<BarometerSample>()?;
        bus.register::<GnssSample>()?;
        bus.register::<MagnetometerSample>()?;
        bus.register::<StarTrackerSample>()?;
        bus.register::<AttitudeEstimate>()?;
        bus.register::<PositionEstimate>()?;
        bus.register::<EstimatorStatus>()?;
        bus.register::<VehicleStatus>()?;
        bus.register::<FailsafeFlags>()?;
        bus.register::<ReferenceState>()?;
        bus.register::<ActuatorCommand>()?;
        bus.register::<EngineDemand>()?;
        bus.register::<FdirStatus>()?;
        Ok(())
    }
}

fn apply_ekf_overrides(params: &mut EkfParams, cfg: &FcEkfConfig) {
    if let Some(v) = cfg.sigma_w_gyro {
        params.sigma_w_gyro = v;
    }
    if let Some(v) = cfg.sigma_w_accel_bias {
        params.sigma_w_accel_bias = v;
    }
    if let Some(v) = cfg.sigma_w_gyro_bias {
        params.sigma_w_gyro_bias = v;
    }
    if let Some(v) = cfg.sigma_gnss_pos_m {
        params.sigma_gnss_pos_m = v;
    }
    if let Some(v) = cfg.sigma_gnss_vel_m_s {
        params.sigma_gnss_vel_m_s = v;
    }
    if let Some(v) = cfg.sigma_baro_alt_m {
        params.sigma_baro_alt_m = v;
    }
    if let Some(v) = cfg.sigma_mag_nt {
        params.sigma_mag_nt = v;
    }
    if let Some(v) = cfg.innovation_gate {
        params.innovation_gate = v;
    }
    if let Some(v) = cfg.dead_reckon_timeout_s {
        params.dead_reckon_timeout_s = v;
    }
}

fn apply_mekf_overrides(params: &mut MekfParams, cfg: &FcMekfConfig) {
    if let Some(v) = cfg.sigma_w_gyro {
        params.sigma_w_gyro = v;
    }
    if let Some(v) = cfg.sigma_w_gyro_bias {
        params.sigma_w_gyro_bias = v;
    }
    if let Some(v) = cfg.sigma_mag_nt {
        params.sigma_mag_nt = v;
    }
    if let Some(v) = cfg.innovation_gate {
        params.innovation_gate = v;
    }
}

fn build_gain_schedule(cfg: Option<&BTreeMap<String, FcGainsConfig>>) -> GainSchedule {
    let mut schedule = GainSchedule {
        by_phase: BTreeMap::new(),
        default: default_gains(),
    };
    if let Some(map) = cfg {
        for (path, gains) in map {
            let phase_id = PhaseId::from_path(path);
            let mut g = default_gains();
            apply_gains_overrides(&mut g, gains);
            schedule.by_phase.insert(phase_id.value(), g);
        }
    }
    schedule
}

fn apply_gains_overrides(g: &mut ThreeLoopGains, cfg: &FcGainsConfig) {
    fn copy_triple(triple: &mut [PidGains; 3], kp: Option<[f64; 3]>, axis: GainAxis) {
        let Some(values) = kp else { return };
        for (slot, value) in triple.iter_mut().zip(values) {
            match axis {
                GainAxis::Kp => slot.kp = value,
                GainAxis::Ki => slot.ki = value,
                GainAxis::Kd => slot.kd = value,
            }
        }
    }
    copy_triple(&mut g.rate, cfg.rate_kp, GainAxis::Kp);
    copy_triple(&mut g.rate, cfg.rate_ki, GainAxis::Ki);
    copy_triple(&mut g.rate, cfg.rate_kd, GainAxis::Kd);
    copy_triple(&mut g.attitude, cfg.attitude_kp, GainAxis::Kp);
    copy_triple(&mut g.attitude, cfg.attitude_ki, GainAxis::Ki);
    copy_triple(&mut g.attitude, cfg.attitude_kd, GainAxis::Kd);
    copy_triple(&mut g.trajectory, cfg.trajectory_kp, GainAxis::Kp);
    copy_triple(&mut g.trajectory, cfg.trajectory_ki, GainAxis::Ki);
    copy_triple(&mut g.trajectory, cfg.trajectory_kd, GainAxis::Kd);
    if let Some(v) = cfg.aileron_limit_rad {
        g.aileron_limit_rad = v;
    }
    if let Some(v) = cfg.elevator_limit_rad {
        g.elevator_limit_rad = v;
    }
    if let Some(v) = cfg.rudder_limit_rad {
        g.rudder_limit_rad = v;
    }
    if let Some(v) = cfg.throttle_baseline {
        g.throttle_baseline = v;
    }
}

#[derive(Copy, Clone)]
enum GainAxis {
    Kp,
    Ki,
    Kd,
}

fn build_authority(cfg: Option<&BTreeMap<String, FcPhaseAuthorityConfig>>) -> PhaseAuthorityTable {
    let mut table = PhaseAuthorityTable::default();
    if let Some(map) = cfg {
        for (path, entry) in map {
            let phase_id = PhaseId::from_path(path);
            table.allowed.insert(
                phase_id.value(),
                PhaseAuthority {
                    effectors: Vec::new(),
                    engines: Vec::new(),
                    autopilot_allowed: entry.autopilot_allowed,
                    engines_allowed: entry.engines_allowed,
                },
            );
        }
    }
    table
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use openbmp_mission::{BuiltInEventTrigger, EventAction, EventId, Phase, PhaseTransition};
    use openbmp_scenario::{FcAutopilotKind, FcEstimatorKind, FcGuidanceKind};

    use super::*;

    fn minimal_graph() -> (MissionPhaseGraph, Vec<EventBinding>, PhaseId) {
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

    #[test]
    fn fc_runner_builds_from_attitude_hold_config() {
        let config = FcConfig {
            estimator: FcEstimatorKind::Ekf,
            autopilot: FcAutopilotKind::ThreeLoop,
            guidance: FcGuidanceKind::AttitudeHold,
            reference_q_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
            base_rate_hz: 1_000,
            ekf: Some(FcEkfConfig::default()),
            mekf: None,
            autopilot_params: None,
            gain_schedule: None,
            phase_authority: None,
        };
        let (graph, bindings, pad) = minimal_graph();
        let mut runner = FcRunner::new(&config, graph, bindings, pad).unwrap();
        // Drive 100 ticks at 1 ms each.
        let dt_s = 0.001;
        for k in 0..100u64 {
            let now = SimTime::from_seconds(f64::from(u32::try_from(k).unwrap()) * dt_s);
            runner.publish_imu(ImuSample {
                time: now,
                gyro_rad_s: Vector3::zeros(),
                accel_m_s2: Vector3::new(0.0, 0.0, 9.81),
                healthy: true,
            });
            let _ = runner.step(now, StepIndex::new(k)).unwrap();
        }
        let attitude = runner
            .latest_attitude_estimate()
            .expect("estimator should publish");
        for v in attitude.q_body_to_eci_xyzw {
            assert!(v.is_finite(), "attitude q has NaN");
        }
    }
}
