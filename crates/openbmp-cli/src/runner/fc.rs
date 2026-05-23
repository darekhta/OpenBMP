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
use openbmp_core::{EffectorId, EngineId, SimTime, StepIndex};
use openbmp_fc::autopilot::{
    AutopilotParams, GainSchedule, PidGains, ThreeLoopAutopilot, ThreeLoopGains, TrajectoryKind,
    default_gains,
};
use openbmp_fc::commander::{Commander, CommanderParams};
use openbmp_fc::estimator::{Ekf, EkfParams, Estimator, EstimatorJob, Mekf, MekfParams};
use openbmp_fc::estimator_lanes::{LaneId, MultiLaneEstimator, VoterPolicy};
use openbmp_fc::fdir::{DetectorKind, FdirJob, FdirParams};
use openbmp_fc::guidance::{
    AttitudeHoldGuidance, GuidanceParams, WaypointGuidance, WaypointSequence,
};
use openbmp_fc::health::{HealthMonitor, HealthParams};
use openbmp_fc::imm::ImmEstimator;
use openbmp_fc::mixer::{ActuatorChannelMap, Mixer, PhaseAuthority, PhaseAuthorityTable};
use openbmp_fc::sr_ukf::{SquareRootUkf, SquareRootUkfAttitude, SquareRootUkfParams};
use openbmp_fc::topics::{
    ActuatorCommand, AttitudeEstimate, AutopilotStatus, BarometerSample, EffectorCommandSet,
    EngineCommandSet, EngineDemand, EstimatorMode, EstimatorStatus, FailsafeFlags,
    FdirGlrtDiagnostic, FdirStatus, GnssSample, ImuSample, MagnetometerSample,
    MissionStatePublish, PositionEstimate, ReferenceState, SensorStatus, StarTrackerSample,
    VehicleStatus,
};
use openbmp_fc::{
    ControllerError, DispatchSummary, EstimatorError, FlightController, FlightControllerBuilder,
};
use openbmp_mission::{EventBinding, MissionPhaseGraph, PhaseId};

use crate::runner::mission::project_mission_bindings;
use openbmp_physics::magnetic::Wmm2025;
use openbmp_scenario::{
    FcActuatorChannelsConfig, FcAntiWindupConfig, FcAutopilotKind, FcAutopilotParams, FcConfig,
    FcEkfConfig, FcEstimatorKind, FcEstimatorLanesConfig, FcEstimatorVoterKind, FcFdirConfig,
    FcFdirDetectorKind, FcFdirDetectorKindV5, FcGainsConfig, FcGuidanceKind, FcHealthConfig,
    FcMagFieldKind, FcMekfConfig, FcPhaseAuthorityConfig, FcTrajectoryKind,
};

const DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR: f64 = 2025.0;

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
    /// Returns the underlying controller error if topic registration,
    /// job registration, or estimator construction fails.
    #[allow(clippy::too_many_lines, clippy::expect_used)]
    pub fn new(
        config: &FcConfig,
        mission_graph: MissionPhaseGraph,
        event_bindings: Vec<EventBinding>,
        hsm: Option<openbmp_mission::MissionStateMachine>,
        start_phase: PhaseId,
        autopilot_lqr_context: Option<FcAutopilotLqrContext>,
        loop_step_dt_s: f64,
        allocator: Option<openbmp_fc::allocation::PrioritisedRedistributedAllocator>,
    ) -> Result<Self, openbmp_fc::ControllerError> {
        let mut fc = FlightControllerBuilder::new()
            .frame_budget_us(config.frame_budget_us)
            .build();

        Self::register_canonical_topics(&fc)?;

        let slow_period_ticks = period_ticks_for_hz(config.base_rate_hz, 100);
        let mut next_priority = 5_u8;
        // Phase-5.B.2 — when the scenario declares `[fc.estimator_lanes]`,
        // build a `MultiLaneEstimator` containing one estimator per
        // lane and register it as the single scheduled estimator
        // job. The voter policy from the scenario block selects the
        // active lane per tick. When the block is absent, fall back
        // to the single-estimator construction path keyed off
        // `config.estimator`.
        if let Some(lanes_cfg) = &config.estimator_lanes {
            let multi = build_multi_lane_estimator(config, lanes_cfg)?;
            fc.scheduler_mut().register_periodic(
                1,
                200,
                next_priority,
                Box::new(EstimatorJob::new(multi)),
            )?;
        } else {
            let estimator = build_single_estimator(config, config.estimator)?;
            // The trait object is wrapped in EstimatorJob the same way
            // any concrete Estimator would be; the boxed-dyn shape is
            // because the per-lane construction in the multi-lane path
            // also returns `Box<dyn Estimator + Send>`.
            fc.scheduler_mut().register_periodic(
                1,
                200,
                next_priority,
                Box::new(EstimatorJob::new(BoxedEstimator(estimator))),
            )?;
        }
        next_priority = next_priority.saturating_add(5);

        // Guidance: 100 Hz expressed as a period in configured base-rate ticks.
        match config.guidance {
            FcGuidanceKind::AttitudeHold => {
                let q = config.reference_q_xyzw.unwrap_or([0.0, 0.0, 0.0, 1.0]);
                fc.scheduler_mut().register_periodic(
                    slow_period_ticks,
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
                    slow_period_ticks,
                    100,
                    next_priority,
                    Box::new(WaypointGuidance::new(sequence, GuidanceParams::default())),
                )?;
            }
        }
        next_priority = next_priority.saturating_add(5);

        let authority = build_authority(&mission_graph, config.phase_authority.as_ref());

        // Commander.
        //
        // Phase 5.X.A: the commander now consumes
        // `Vec<EventBinding<MissionAction>>` (HAL-portable) rather than
        // the legacy unified `Vec<EventBinding<EventAction>>`. The
        // simulator-only physics-override variants (engine / effector /
        // separation / recovery) are dropped here — they continue to
        // flow through the simulator kernel via the same
        // `event_bindings` list during the migration window. Phase
        // 5.X.B splits the kernel-side list to mirror this projection.
        let commander_bindings = project_mission_bindings(&event_bindings);
        let commander = Commander::new(
            mission_graph,
            commander_bindings,
            start_phase,
            CommanderParams::default(),
        )
        .map_err(openbmp_fc::ControllerError::from)?;
        // Phase 5.X.F: attach the hierarchical view when available so
        // downstream consumers can query LCA / exit / enter chains on
        // the commander's HSM accessor.
        let commander = if let Some(hsm) = hsm {
            commander.with_hierarchical(hsm)
        } else {
            commander
        };
        fc.scheduler_mut()
            .register_periodic(1, 200, next_priority, Box::new(commander))?;
        next_priority = next_priority.saturating_add(5);

        // Autopilot.
        let schedule = build_gain_schedule(&config.gain_schedule);
        let mut autopilot = match config.autopilot {
            FcAutopilotKind::ThreeLoop => ThreeLoopAutopilot::with_schedule(schedule),
        };
        if let Some(params) = &config.autopilot_params {
            autopilot = autopilot.with_params(build_autopilot_params(
                params,
                loop_step_dt_s,
                autopilot_lqr_context.as_ref(),
            )?);
        }
        if let Some(trajectory_cfg) = config.trajectory.as_ref() {
            let trajectory = build_minimum_snap_trajectory(trajectory_cfg)
                .map_err(openbmp_fc::ControllerError::from)?;
            autopilot = autopilot
                .with_minimum_snap_trajectory(trajectory)
                .with_minimum_snap_yaw_rad(trajectory_cfg.yaw_rad.unwrap_or(0.0));
        }
        fc.scheduler_mut()
            .register_periodic(1, 300, next_priority, Box::new(autopilot))?;
        next_priority = next_priority.saturating_add(5);

        // Mixer.
        let mut mixer = Mixer::new()
            .with_authority(authority)
            .with_actuator_channel_map(build_actuator_channel_map(
                config.actuator_channels.as_ref(),
            ));
        if let Some(alloc) = allocator {
            mixer = mixer.with_allocator(alloc);
        }
        fc.scheduler_mut()
            .register_periodic(1, 100, next_priority, Box::new(mixer))?;
        next_priority = next_priority.saturating_add(5);

        // Health monitor (100 Hz).
        fc.scheduler_mut().register_periodic(
            slow_period_ticks,
            100,
            next_priority,
            Box::new(HealthMonitor::new(build_health_params(&config.health))),
        )?;
        next_priority = next_priority.saturating_add(5);

        // FDIR (100 Hz).
        fc.scheduler_mut().register_periodic(
            slow_period_ticks,
            100,
            next_priority,
            Box::new(FdirJob::new(build_fdir_params(config.fdir.as_ref()))),
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

    /// Publishes a star-tracker sample.
    pub fn publish_star_tracker(&self, sample: StarTrackerSample) {
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

    /// Returns the latest effector-id command set, if any.
    #[must_use]
    pub fn latest_effector_command_set(&self) -> Option<EffectorCommandSet> {
        self.fc
            .bus()
            .latest::<EffectorCommandSet>()
            .ok()
            .flatten()
            .map(|(c, _)| c)
    }

    /// Returns the latest engine-id command set, if any.
    #[must_use]
    pub fn latest_engine_command_set(&self) -> Option<EngineCommandSet> {
        self.fc
            .bus()
            .latest::<EngineCommandSet>()
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

    /// Phase 5.X.B: returns the latest mission-state publication
    /// (the single-source-of-truth topic) for downstream consumers
    /// that should subscribe instead of holding their own
    /// mission-graph copy. Returns `None` when the commander has
    /// not yet published (pre-first-tick).
    #[must_use]
    pub fn latest_mission_state(&self) -> Option<MissionStatePublish> {
        self.fc
            .bus()
            .latest::<MissionStatePublish>()
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
        bus.register::<SensorStatus>()?;
        bus.register::<AttitudeEstimate>()?;
        bus.register::<PositionEstimate>()?;
        bus.register::<EstimatorStatus>()?;
        bus.register::<VehicleStatus>()?;
        bus.register::<FailsafeFlags>()?;
        bus.register::<ReferenceState>()?;
        bus.register::<ActuatorCommand>()?;
        bus.register::<AutopilotStatus>()?;
        bus.register::<EffectorCommandSet>()?;
        bus.register::<EngineDemand>()?;
        bus.register::<EngineCommandSet>()?;
        bus.register::<FdirStatus>()?;
        // Phase-5.B.4: GLRT diagnostic topic. Always registered so
        // scenarios that opt into the windowed-mean-shift GLRT can
        // publish without a separate setup step. Idle when the
        // detector kind is the legacy burst-counter / single-sample /
        // CUSUM family.
        bus.register::<FdirGlrtDiagnostic>()?;
        // Phase-5.B.3: IMM mode-probability snapshot. Always
        // registered so scenarios that opt into `kind = "imm"` can
        // publish without a separate setup step. Idle when the
        // selected estimator is not IMM.
        bus.register::<EstimatorMode>()?;
        // Phase 5.X.B: single-source-of-truth mission state topic.
        // The commander publishes here every tick; downstream code
        // (simulator-side subscriber, telemetry recorder) reads in
        // place of the legacy parallel-state pattern.
        bus.register::<MissionStatePublish>()?;
        Ok(())
    }
}

fn period_ticks_for_hz(base_rate_hz: u32, task_rate_hz: u32) -> u64 {
    let base = u64::from(base_rate_hz);
    let task = u64::from(task_rate_hz.max(1));
    base.div_ceil(task).max(1)
}

fn apply_ekf_mag_model(ekf: Ekf, cfg: Option<&FcEkfConfig>) -> Result<Ekf, ControllerError> {
    let kind = cfg.and_then(|c| c.mag_field).unwrap_or_default();
    match kind {
        FcMagFieldKind::EarthDipole => Ok(ekf),
        FcMagFieldKind::Wmm2025 => {
            let epoch = cfg
                .and_then(|c| c.mag_epoch_decimal_year)
                .unwrap_or(DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR);
            Ok(ekf.with_mag_field_model(build_wmm_2025(epoch)?))
        }
    }
}

fn apply_mekf_mag_model(mekf: Mekf, cfg: Option<&FcMekfConfig>) -> Result<Mekf, ControllerError> {
    let kind = cfg.and_then(|c| c.mag_field).unwrap_or_default();
    match kind {
        FcMagFieldKind::EarthDipole => Ok(mekf),
        FcMagFieldKind::Wmm2025 => {
            let epoch = cfg
                .and_then(|c| c.mag_epoch_decimal_year)
                .unwrap_or(DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR);
            Ok(mekf.with_mag_field_model(build_wmm_2025(epoch)?))
        }
    }
}

fn build_wmm_2025(epoch: f64) -> Result<Wmm2025, ControllerError> {
    Wmm2025::new_for_decimal_year(epoch).map_err(|err| {
        EstimatorError::InvalidConfig {
            reason: format!("WMM 2025 magnetic model rejected epoch {epoch}: {err}"),
        }
        .into()
    })
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
    if let Some(v) = cfg.tau_gyro_bias_s {
        params.tau_gyro_bias_s = v;
    }
    if let Some(v) = cfg.tau_accel_bias_s {
        params.tau_accel_bias_s = v;
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
    if let Some(v) = cfg.innovation_false_alarm_rate {
        params.innovation_false_alarm_rate = v;
        params.innovation_gate = f64::NAN;
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
    if let Some(v) = cfg.tau_gyro_bias_s {
        params.tau_gyro_bias_s = v;
    }
    if let Some(v) = cfg.sigma_mag_nt {
        params.sigma_mag_nt = v;
    }
    if let Some(v) = cfg.innovation_gate {
        params.innovation_gate = v;
    }
    if let Some(v) = cfg.innovation_false_alarm_rate {
        params.innovation_false_alarm_rate = v;
        params.innovation_gate = f64::NAN;
    }
}

// ---------------------------------------------------------------------
// Phase-5.B.2 — multi-instance estimator routing helpers.
// ---------------------------------------------------------------------

/// Wrapper that exposes a `Box<dyn Estimator + Send>` as a concrete
/// type implementing the [`Estimator`] trait, so the existing
/// `EstimatorJob<E>` (which is generic over a sized type) can wrap
/// it without further plumbing.
struct BoxedEstimator(Box<dyn Estimator + Send>);

impl std::fmt::Debug for BoxedEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BoxedEstimator").finish_non_exhaustive()
    }
}

impl Estimator for BoxedEstimator {
    fn name(&self) -> &'static str {
        self.0.name()
    }

    fn predict(&mut self, dt: f64) -> Result<(), EstimatorError> {
        self.0.predict(dt)
    }

    fn update_imu(&mut self, sample: &openbmp_fc::topics::ImuSample) -> Result<(), EstimatorError> {
        self.0.update_imu(sample)
    }

    fn update_gnss(
        &mut self,
        sample: &openbmp_fc::topics::GnssSample,
    ) -> Result<(), EstimatorError> {
        self.0.update_gnss(sample)
    }

    fn update_baro(
        &mut self,
        sample: &openbmp_fc::topics::BarometerSample,
    ) -> Result<(), EstimatorError> {
        self.0.update_baro(sample)
    }

    fn update_mag(
        &mut self,
        sample: &openbmp_fc::topics::MagnetometerSample,
    ) -> Result<(), EstimatorError> {
        self.0.update_mag(sample)
    }

    fn attitude(&self) -> AttitudeEstimate {
        self.0.attitude()
    }

    fn position(&self) -> PositionEstimate {
        self.0.position()
    }

    fn status(&self) -> EstimatorStatus {
        self.0.status()
    }

    fn estimator_mode(&self) -> Option<EstimatorMode> {
        self.0.estimator_mode()
    }

    fn begin_tick(&mut self) {
        self.0.begin_tick();
    }
}

/// Build a single estimator instance for the given kind. The
/// scenario validator already guarantees the required `[fc.*]`
/// blocks are present for each kind; this builder still fails closed
/// if called with an unvalidated config.
fn build_single_estimator(
    config: &FcConfig,
    kind: FcEstimatorKind,
) -> Result<Box<dyn Estimator + Send>, ControllerError> {
    match kind {
        FcEstimatorKind::Ekf => {
            let ekf_cfg = required_ekf_config(config, "ekf")?;
            let mut params = EkfParams::default();
            apply_ekf_overrides(&mut params, ekf_cfg);
            let mut ekf = apply_ekf_mag_model(Ekf::new(params), Some(ekf_cfg))?;
            ekf.seed(
                Vector3::zeros(),
                Vector3::zeros(),
                UnitQuaternion::identity(),
            );
            Ok(Box::new(ekf))
        }
        FcEstimatorKind::Mekf => {
            let mekf_cfg = config
                .mekf
                .as_ref()
                .ok_or_else(|| EstimatorError::InvalidConfig {
                    reason: "estimator kind mekf requires [fc.mekf]".to_owned(),
                })?;
            let mut params = MekfParams::default();
            apply_mekf_overrides(&mut params, mekf_cfg);
            let mut mekf = apply_mekf_mag_model(Mekf::new(params), Some(mekf_cfg))?;
            mekf.seed(UnitQuaternion::identity());
            Ok(Box::new(mekf))
        }
        FcEstimatorKind::Imm => {
            let imm_cfg = config
                .imm
                .as_ref()
                .ok_or_else(|| EstimatorError::InvalidConfig {
                    reason: "estimator kind imm requires [fc.imm]".to_owned(),
                })?;
            let base_ekf_cfg = required_ekf_config(config, "imm")?;
            let mut base_params = EkfParams::default();
            apply_ekf_overrides(&mut base_params, base_ekf_cfg);
            let mut per_mode_params: Vec<EkfParams> = Vec::with_capacity(imm_cfg.modes.len());
            for mode in &imm_cfg.modes {
                let mut p = base_params.clone();
                if let Some(v) = mode.sigma_w_gyro {
                    p.sigma_w_gyro = v;
                }
                if let Some(v) = mode.sigma_w_gyro_bias {
                    p.sigma_w_gyro_bias = v;
                }
                if let Some(v) = mode.sigma_w_accel_bias {
                    p.sigma_w_accel_bias = v;
                }
                if let Some(v) = mode.tau_gyro_bias_s {
                    p.tau_gyro_bias_s = v;
                }
                if let Some(v) = mode.tau_accel_bias_s {
                    p.tau_accel_bias_s = v;
                }
                per_mode_params.push(p);
            }
            let imm = ImmEstimator::new(
                per_mode_params,
                imm_cfg.transition_matrix.clone(),
                imm_cfg.initial_mode_probabilities.clone(),
            )
            .map_err(|err| EstimatorError::InvalidConfig {
                reason: format!("IMM estimator rejected scenario parameters: {err:?}"),
            })?;
            Ok(Box::new(imm))
        }
        FcEstimatorKind::SrUkf => {
            let ekf_cfg = required_ekf_config(config, "sr_ukf")?;
            let mut sr_params = SquareRootUkfParams::default();
            let mut ekf_params = EkfParams::default();
            apply_ekf_overrides(&mut ekf_params, ekf_cfg);
            copy_ekf_to_sr_ukf_params(&ekf_params, &mut sr_params);
            let mut sr_ukf = SquareRootUkf::new(sr_params);
            // Apply the [fc.ekf] mag-field model selection — same
            // resolution path as the EKF / IMM lanes.
            sr_ukf = apply_sr_ukf_mag_model(sr_ukf, Some(ekf_cfg))?;
            sr_ukf.seed(
                Vector3::zeros(),
                Vector3::zeros(),
                UnitQuaternion::identity(),
            );
            Ok(Box::new(sr_ukf))
        }
        FcEstimatorKind::SrUkfAttitude => {
            let ekf_cfg = required_ekf_config(config, "sr_ukf_attitude")?;
            let mut sr_params = SquareRootUkfParams::default();
            let mut ekf_params = EkfParams::default();
            apply_ekf_overrides(&mut ekf_params, ekf_cfg);
            copy_ekf_to_sr_ukf_params(&ekf_params, &mut sr_params);
            let mut sr_ukf = SquareRootUkfAttitude::new(sr_params);
            sr_ukf = apply_sr_ukf_attitude_mag_model(sr_ukf, Some(ekf_cfg))?;
            sr_ukf.seed(UnitQuaternion::identity());
            Ok(Box::new(sr_ukf))
        }
    }
}

fn required_ekf_config<'a>(
    config: &'a FcConfig,
    estimator_kind: &str,
) -> Result<&'a FcEkfConfig, EstimatorError> {
    config
        .ekf
        .as_ref()
        .ok_or_else(|| EstimatorError::InvalidConfig {
            reason: format!("estimator kind {estimator_kind} requires [fc.ekf]"),
        })
}

fn copy_ekf_to_sr_ukf_params(ekf: &EkfParams, sr: &mut SquareRootUkfParams) {
    sr.sigma_w_gyro = ekf.sigma_w_gyro;
    sr.sigma_w_accel_bias = ekf.sigma_w_accel_bias;
    sr.sigma_w_gyro_bias = ekf.sigma_w_gyro_bias;
    sr.tau_gyro_bias_s = ekf.tau_gyro_bias_s;
    sr.tau_accel_bias_s = ekf.tau_accel_bias_s;
    sr.sigma_gnss_pos_m = ekf.sigma_gnss_pos_m;
    sr.sigma_gnss_vel_m_s = ekf.sigma_gnss_vel_m_s;
    sr.sigma_baro_alt_m = ekf.sigma_baro_alt_m;
    sr.sigma_mag_nt = ekf.sigma_mag_nt;
    sr.innovation_gate = ekf.innovation_gate;
    sr.innovation_false_alarm_rate = ekf.innovation_false_alarm_rate;
    sr.dead_reckon_timeout_s = ekf.dead_reckon_timeout_s;
}

fn apply_sr_ukf_mag_model(
    sr_ukf: SquareRootUkf,
    cfg: Option<&FcEkfConfig>,
) -> Result<SquareRootUkf, ControllerError> {
    let kind = cfg.and_then(|c| c.mag_field).unwrap_or_default();
    match kind {
        FcMagFieldKind::EarthDipole => Ok(sr_ukf),
        FcMagFieldKind::Wmm2025 => {
            let epoch = cfg
                .and_then(|c| c.mag_epoch_decimal_year)
                .unwrap_or(DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR);
            Ok(sr_ukf.with_mag_field_model(build_wmm_2025(epoch)?))
        }
    }
}

fn apply_sr_ukf_attitude_mag_model(
    sr_ukf: SquareRootUkfAttitude,
    cfg: Option<&FcEkfConfig>,
) -> Result<SquareRootUkfAttitude, ControllerError> {
    let kind = cfg.and_then(|c| c.mag_field).unwrap_or_default();
    match kind {
        FcMagFieldKind::EarthDipole => Ok(sr_ukf),
        FcMagFieldKind::Wmm2025 => {
            let epoch = cfg
                .and_then(|c| c.mag_epoch_decimal_year)
                .unwrap_or(DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR);
            Ok(sr_ukf.with_mag_field_model(build_wmm_2025(epoch)?))
        }
    }
}

/// Build a [`MultiLaneEstimator`] from the scenario's
/// `[fc.estimator_lanes]` block.
fn build_multi_lane_estimator(
    config: &FcConfig,
    lanes_cfg: &FcEstimatorLanesConfig,
) -> Result<MultiLaneEstimator, ControllerError> {
    let mut lanes: Vec<(LaneId, Box<dyn Estimator + Send>)> =
        Vec::with_capacity(lanes_cfg.lanes.len());
    for lane in &lanes_cfg.lanes {
        let estimator = build_single_estimator(config, lane.estimator)?;
        lanes.push((LaneId::from(lane.id.clone()), estimator));
    }
    let policy = match lanes_cfg.voter {
        FcEstimatorVoterKind::SimplexPassThrough => VoterPolicy::SimplexPassThrough,
        FcEstimatorVoterKind::MidValueSelectByInnovation => VoterPolicy::MidValueSelectByInnovation,
        FcEstimatorVoterKind::BestByCovarianceTrace => VoterPolicy::BestByCovarianceTrace,
    };
    Ok(MultiLaneEstimator::new(lanes, policy))
}

/// Phase 5.A.3.B context required to translate
/// `[fc.autopilot_params.lqr]` into solved per-axis gains. The
/// runner pre-extracts these from the vehicle config since the
/// autopilot needs them to solve the per-axis DARE before the FC
/// starts ticking.
#[derive(Copy, Clone, Debug)]
pub struct FcAutopilotLqrContext {
    /// Loop step `time.dt_s`.
    pub dt_s: f64,
    /// Diagonal moments of inertia `[J_xx, J_yy, J_zz]` (kg·m²).
    pub diagonal_inertia_kg_m2: [f64; 3],
}

#[allow(clippy::too_many_lines)]
fn build_autopilot_params(
    cfg: &FcAutopilotParams,
    dt_s: f64,
    lqr_ctx: Option<&FcAutopilotLqrContext>,
) -> Result<AutopilotParams, openbmp_fc::ControllerError> {
    use openbmp_fc::anti_windup::AntiWindupKind;
    #[cfg(any(feature = "lqr", feature = "indi", feature = "mpc"))]
    use openbmp_fc::error::AutopilotError;
    // Phase 5.A.3.A: explicit `[fc.autopilot_params.anti_windup]`
    // wins over the legacy `anti_windup_gain` scalar; otherwise the
    // legacy scalar maps to back-calculation, preserving Phase-4
    // bit-stable behaviour for scenarios that have neither block.
    let anti_windup = match cfg.anti_windup {
        Some(FcAntiWindupConfig::BackCalculation { gain }) => {
            AntiWindupKind::BackCalculation { gain }
        }
        Some(FcAntiWindupConfig::ObserverForm { tracking_time_s }) => {
            AntiWindupKind::ObserverForm { tracking_time_s }
        }
        None => match cfg.anti_windup_gain {
            Some(gain) => AntiWindupKind::BackCalculation { gain },
            None => AntiWindupKind::default(),
        },
    };
    let mut params = AutopilotParams {
        anti_windup,
        ..AutopilotParams::default()
    };
    if let Some(v) = cfg.rate_deadband_rad_s {
        params.rate_deadband_rad_s = v;
    }
    if let Some(v) = cfg.trajectory_loop_enabled {
        params.trajectory_loop_enabled = v;
    }
    if let Some(kind) = cfg.trajectory_kind {
        params.trajectory_kind = match kind {
            FcTrajectoryKind::Pid => TrajectoryKind::Pid,
            FcTrajectoryKind::MinimumSnap => TrajectoryKind::DifferentialFlatness,
        };
    }
    #[cfg(feature = "l1-adaptive")]
    if let Some(l1) = cfg.l1_adaptive.as_ref() {
        params.l1_adaptive = Some(openbmp_fc::l1_adaptive_full::L1AdaptiveParams {
            reference_model_a_m: l1.reference_model_a_m,
            reference_model_b: l1.reference_model_b,
            reference_model_k_g: l1.reference_model_k_g,
            adaptation_sample_time_s: l1.adaptation_sample_time_s,
            low_pass_cutoff_rad_s: l1.low_pass_cutoff_rad_s,
            lipschitz_bound: l1.lipschitz_bound,
            projection_bound: l1.projection_bound,
        });
    }
    // Phase 5.A.3.B — rate-loop kind dispatch. Default keeps the
    // PID loop (Phase-4 behaviour); selecting LQR triggers a
    // per-axis DARE solve at scenario load using the diagonal
    // inertia of the single-body assembly.
    if let Some(kind) = cfg.rate_loop_kind {
        match kind {
            openbmp_scenario::FcRateLoopKind::Pid => {
                params.rate_loop_kind = openbmp_fc::autopilot::RateLoopKind::Pid;
            }
            openbmp_scenario::FcRateLoopKind::Lqr => {
                #[cfg(feature = "lqr")]
                {
                    let ctx = lqr_ctx.ok_or_else(|| {
                        openbmp_fc::ControllerError::from(AutopilotError::Trajectory {
                            reason: "rate_loop_kind = \"lqr\" requires the runner to supply \
                                 FcAutopilotLqrContext (dt + diagonal inertia)"
                                .to_string(),
                        })
                    })?;
                    let lqr_cfg = cfg.lqr.as_ref().ok_or_else(|| {
                        openbmp_fc::ControllerError::from(AutopilotError::Trajectory {
                            reason: "rate_loop_kind = \"lqr\" but [fc.autopilot_params.lqr] is \
                                 absent (parser should have caught this)"
                                .to_string(),
                        })
                    })?;
                    let mut gains = [openbmp_fc::lqr::LqrGains::default(); 3];
                    for (axis, gain_slot) in gains.iter_mut().enumerate() {
                        *gain_slot = openbmp_fc::lqr::solve_lqr_rate_loop(
                            ctx.dt_s,
                            ctx.diagonal_inertia_kg_m2[axis],
                            lqr_cfg.q_omega[axis],
                            lqr_cfg.q_int[axis],
                            lqr_cfg.r[axis],
                        )
                        .map_err(|err| {
                            openbmp_fc::ControllerError::from(AutopilotError::Trajectory {
                                reason: format!("LQR DARE solve failed on axis {axis}: {err}"),
                            })
                        })?;
                    }
                    params.rate_loop_kind = openbmp_fc::autopilot::RateLoopKind::Lqr;
                    params.lqr_gains = Some(gains);
                    let _ = lqr_ctx;
                }
                #[cfg(not(feature = "lqr"))]
                {
                    let _ = (lqr_ctx, cfg.lqr.as_ref());
                    return Err(openbmp_fc::ControllerError::from(
                        openbmp_fc::error::AutopilotError::Trajectory {
                            reason: "rate_loop_kind = \"lqr\" requires the openbmp-cli \
                                     `lqr` Cargo feature; rebuild with --features lqr."
                                .to_string(),
                        },
                    ));
                }
            }
            openbmp_scenario::FcRateLoopKind::Indi => {
                #[cfg(feature = "indi")]
                {
                    let indi_cfg = cfg.indi.as_ref().ok_or_else(|| {
                        openbmp_fc::ControllerError::from(AutopilotError::Trajectory {
                            reason: "rate_loop_kind = \"indi\" but [fc.autopilot_params.indi] \
                                     is absent (parser should have caught this)"
                                .to_string(),
                        })
                    })?;
                    let indi_params = openbmp_fc::indi::IndiParams {
                        inertia_per_axis_kg_m2: indi_cfg.inertia_per_axis_kg_m2,
                        control_effectiveness_per_axis: indi_cfg.control_effectiveness_per_axis,
                        filter_cutoff_rad_s: indi_cfg.filter_cutoff_rad_s,
                        filter_kind: match indi_cfg.filter_kind {
                            openbmp_scenario::FcIndiFilterKind::FirstOrderLowPass => {
                                openbmp_fc::indi::IndiFilterKind::FirstOrderLowPass
                            }
                            openbmp_scenario::FcIndiFilterKind::SecondOrderButterworth => {
                                openbmp_fc::indi::IndiFilterKind::SecondOrderButterworth
                            }
                        },
                        attitude_to_omega_dot_gain: indi_cfg.attitude_to_omega_dot_gain,
                    };
                    params.rate_loop_kind = openbmp_fc::autopilot::RateLoopKind::Indi;
                    params.indi_params = Some(indi_params);
                }
                #[cfg(not(feature = "indi"))]
                {
                    let _ = cfg.indi.as_ref();
                    return Err(openbmp_fc::ControllerError::from(
                        openbmp_fc::error::AutopilotError::Trajectory {
                            reason: "rate_loop_kind = \"indi\" requires the openbmp-cli \
                                     `indi` Cargo feature; rebuild with --features indi."
                                .to_string(),
                        },
                    ));
                }
            }
        }
    }
    let _ = lqr_ctx;
    let _ = dt_s; // referenced under `mpc` feature only
    // Phase 5.A.4 — attitude-loop kind dispatch. PID is the
    // Phase-4/5.A.2/5.A.3 default; selecting MPC triggers a
    // RecedingHorizonAttitudeMpc construction at scenario load using
    // the configured params and the loop step `time.dt_s`.
    if let Some(kind) = cfg.attitude_loop_kind {
        match kind {
            openbmp_scenario::FcAttitudeLoopKind::Pid => {
                params.attitude_loop_kind = openbmp_fc::autopilot::AttitudeLoopKind::Pid;
            }
            openbmp_scenario::FcAttitudeLoopKind::Mpc => {
                #[cfg(feature = "mpc")]
                {
                    let attitude_mpc_cfg = cfg.attitude_mpc.as_ref().ok_or_else(|| {
                        openbmp_fc::ControllerError::from(AutopilotError::Trajectory {
                            reason: "attitude_loop_kind = \"mpc\" but \
                                     [fc.autopilot_params.attitude_mpc] is absent (parser \
                                     should have caught this)"
                                .to_string(),
                        })
                    })?;
                    let mpc_params = openbmp_fc::mpc::AttitudeMpcParams {
                        horizon_n: attitude_mpc_cfg.horizon_n,
                        q_x: attitude_mpc_cfg.q_x,
                        r_u: attitude_mpc_cfg.r_u,
                        terminal_p: attitude_mpc_cfg.terminal_p,
                        rate_limit_rad_s: attitude_mpc_cfg.rate_limit_rad_s,
                    };
                    let mpc = openbmp_fc::mpc::RecedingHorizonAttitudeMpc::new(mpc_params, dt_s)
                        .map_err(|err| {
                            openbmp_fc::ControllerError::from(AutopilotError::Trajectory {
                                reason: format!("attitude MPC construction failed: {err}"),
                            })
                        })?;
                    params.attitude_loop_kind = openbmp_fc::autopilot::AttitudeLoopKind::Mpc;
                    params.attitude_mpc = Some(std::sync::Arc::new(mpc));
                }
                #[cfg(not(feature = "mpc"))]
                {
                    let _ = cfg.attitude_mpc.as_ref();
                    return Err(openbmp_fc::ControllerError::from(
                        openbmp_fc::error::AutopilotError::Trajectory {
                            reason: "attitude_loop_kind = \"mpc\" requires the openbmp-cli \
                                     `mpc` Cargo feature; rebuild with --features mpc."
                                .to_string(),
                        },
                    ));
                }
            }
        }
    }
    Ok(params)
}

fn build_minimum_snap_trajectory(
    cfg: &openbmp_scenario::FcTrajectoryConfig,
) -> Result<openbmp_fc::trajectory::MinimumSnapTrajectory, openbmp_fc::trajectory::TrajectoryError>
{
    use openbmp_fc::trajectory::{MinimumSnapTrajectory, MinimumSnapWaypoint};
    let waypoints = cfg
        .waypoints
        .iter()
        .map(|w| MinimumSnapWaypoint {
            position_eci_m: nalgebra::Vector3::new(
                w.position_eci_m[0],
                w.position_eci_m[1],
                w.position_eci_m[2],
            ),
            time_s: w.time_s,
        })
        .collect();
    MinimumSnapTrajectory::new(waypoints)
}

fn build_health_params(cfg: &FcHealthConfig) -> HealthParams {
    HealthParams {
        imu_stale_after_s: cfg.imu_stale_after_s,
        gnss_stale_after_s: cfg.gnss_stale_after_s,
        baro_stale_after_s: cfg.baro_stale_after_s,
        mag_stale_after_s: cfg.mag_stale_after_s,
        overrun_burst_count: cfg.overrun_burst_count,
    }
}

#[allow(clippy::expect_used)] // scenario validator (`FcFdirDetectorConfig::validate`) guarantees the GLRT-required fields
fn build_fdir_params(cfg: Option<&FcFdirConfig>) -> FdirParams {
    let mut params = FdirParams::default();
    let Some(cfg) = cfg else { return params };
    params.detector_kind = match cfg.detector_kind {
        FcFdirDetectorKind::BurstCounter => DetectorKind::BurstCounter,
        FcFdirDetectorKind::SingleSampleGlrt => DetectorKind::SingleSampleGlrt,
        FcFdirDetectorKind::Cusum => DetectorKind::Cusum,
    };
    if let Some(v) = cfg.innovation_threshold {
        params.innovation_threshold = v;
    }
    if let Some(v) = cfg.innovation_burst_count {
        params.innovation_burst_count = v;
    }
    if let Some(v) = cfg.failsafe_burst_count {
        params.failsafe_burst_count = v;
    }
    if let Some(v) = cfg.cusum_drift {
        params.cusum_drift = v;
    }
    if let Some(v) = cfg.cusum_threshold {
        params.cusum_threshold = v;
    }
    // Phase-5.B.4: when `[fc.fdir.detector]` is present its `kind`
    // overrides the legacy `detector_kind`. The scenario validator
    // already guarantees `window_samples` is set for
    // `windowed_mean_shift_glrt`, so the unwrap below is safe.
    if let Some(detector) = cfg.detector.as_ref() {
        match detector.kind {
            FcFdirDetectorKindV5::WindowedMeanShiftGlrt => {
                params.detector_kind = DetectorKind::WindowedMeanShiftGlrt;
                params.glrt_window_samples = detector
                    .window_samples
                    .expect("scenario validator ensures window_samples is set for windowed GLRT");
                if let Some(alpha) = detector.false_alarm_rate {
                    params.glrt_false_alarm_rate = alpha;
                }
            }
        }
    }
    params
}

fn build_gain_schedule(cfg: &BTreeMap<String, FcGainsConfig>) -> GainSchedule {
    let mut schedule = GainSchedule {
        by_phase: BTreeMap::new(),
        default: ThreeLoopGains::default(),
    };
    for (path, gains) in cfg {
        let phase_id = PhaseId::from_path(path);
        let mut g = default_gains();
        apply_gains_overrides(&mut g, gains);
        schedule.by_phase.insert(phase_id.value(), g);
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

fn build_authority(
    graph: &MissionPhaseGraph,
    cfg: Option<&BTreeMap<String, FcPhaseAuthorityConfig>>,
) -> PhaseAuthorityTable {
    let mut table = PhaseAuthorityTable::default();
    for phase in &graph.phases {
        table.allowed.insert(
            phase.id.value(),
            PhaseAuthority {
                effectors: phase
                    .allowed_effectors
                    .iter()
                    .map(|id| effector_id_from_config(id))
                    .collect(),
                engines: phase
                    .allowed_engines
                    .iter()
                    .map(|id| engine_id_from_config(id))
                    .collect(),
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
    }
    if let Some(map) = cfg {
        for (path, entry) in map {
            let phase_id = PhaseId::from_path(path);
            let phase = table.allowed.entry(phase_id.value()).or_default();
            phase.autopilot_allowed = entry.autopilot_allowed;
            phase.engines_allowed = entry.engines_allowed;
        }
    }
    table
}

fn build_actuator_channel_map(cfg: Option<&FcActuatorChannelsConfig>) -> ActuatorChannelMap {
    let Some(cfg) = cfg else {
        return ActuatorChannelMap::default();
    };
    ActuatorChannelMap {
        aileron: cfg.aileron.as_deref().map(effector_id_from_config),
        elevator: cfg.elevator.as_deref().map(effector_id_from_config),
        rudder: cfg.rudder.as_deref().map(effector_id_from_config),
        body_flap: cfg.body_flap.as_deref().map(effector_id_from_config),
    }
}

fn effector_id_from_config(id: &str) -> EffectorId {
    if id.starts_with("vehicle.assembly.effectors.") {
        EffectorId::from_path(id)
    } else {
        EffectorId::from_path(&format!("vehicle.assembly.effectors.{id}"))
    }
}

fn engine_id_from_config(id: &str) -> EngineId {
    if id.starts_with("vehicle.assembly.engines.") {
        EngineId::from_path(id)
    } else {
        EngineId::from_path(&format!("vehicle.assembly.engines.{id}"))
    }
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

    /// Phase-5.B.2 — multi-lane configuration runs end-to-end through
    /// the `FcRunner`, with the voter selecting the active lane each
    /// tick. This is the integration-side smoke test; the per-policy
    /// voter unit tests live in
    /// `crates/openbmp-fc/src/estimator_lanes.rs`.
    #[test]
    fn fc_runner_builds_from_multi_lane_config() {
        use openbmp_scenario::{
            FcEstimatorLaneConfig, FcEstimatorLanesConfig, FcEstimatorVoterKind,
        };

        let mut gain_schedule = BTreeMap::new();
        gain_schedule.insert(
            "mission.phases.ascent".to_string(),
            FcGainsConfig {
                rate_kp: Some([0.5, 0.5, 0.5]),
                rate_ki: None,
                rate_kd: Some([0.05, 0.05, 0.05]),
                attitude_kp: Some([2.0, 2.0, 1.0]),
                attitude_ki: None,
                attitude_kd: None,
                trajectory_kp: None,
                trajectory_ki: None,
                trajectory_kd: None,
                aileron_limit_rad: Some(0.35),
                elevator_limit_rad: Some(0.35),
                rudder_limit_rad: Some(0.35),
                throttle_baseline: Some(0.0),
            },
        );
        let lanes = FcEstimatorLanesConfig {
            voter: FcEstimatorVoterKind::SimplexPassThrough,
            lanes: vec![
                FcEstimatorLaneConfig {
                    id: "primary_ekf".to_string(),
                    estimator: FcEstimatorKind::Ekf,
                },
                FcEstimatorLaneConfig {
                    id: "spare_sr_ukf".to_string(),
                    estimator: FcEstimatorKind::SrUkf,
                },
            ],
        };
        let config = FcConfig {
            // The top-level estimator field is irrelevant when
            // estimator_lanes is set — but we still set it to a
            // valid kind for the validator.
            estimator: FcEstimatorKind::Ekf,
            autopilot: FcAutopilotKind::ThreeLoop,
            guidance: FcGuidanceKind::AttitudeHold,
            reference_q_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
            base_rate_hz: 1_000,
            frame_budget_us: 2_000,
            ekf: Some(FcEkfConfig::default()),
            mekf: None,
            imm: None,
            autopilot_params: None,
            health: FcHealthConfig {
                imu_stale_after_s: 0.05,
                gnss_stale_after_s: 0.5,
                baro_stale_after_s: 10.0,
                mag_stale_after_s: 0.2,
                overrun_burst_count: 5,
            },
            fdir: None,
            actuator_channels: None,
            gain_schedule,
            phase_authority: None,
            estimator_lanes: Some(lanes),
            autopilot_allocation: None,
            trajectory: None,
        };
        let (graph, bindings, pad) = minimal_graph();
        let mut runner =
            FcRunner::new(&config, graph, bindings, None, pad, None, 0.001, None).unwrap();
        // Drive 100 ticks at 1 ms each — same workload as the
        // single-lane attitude-hold smoke test.
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
        // The active lane should drive a finite attitude estimate.
        let attitude = runner
            .latest_attitude_estimate()
            .expect("multi-lane estimator should publish");
        for v in attitude.q_body_to_eci_xyzw {
            assert!(v.is_finite(), "active-lane attitude q has NaN");
        }
    }

    #[test]
    fn fc_runner_builds_from_attitude_hold_config() {
        let mut gain_schedule = BTreeMap::new();
        gain_schedule.insert(
            "mission.phases.ascent".to_string(),
            FcGainsConfig {
                rate_kp: Some([0.5, 0.5, 0.5]),
                rate_ki: None,
                rate_kd: Some([0.05, 0.05, 0.05]),
                attitude_kp: Some([2.0, 2.0, 1.0]),
                attitude_ki: None,
                attitude_kd: None,
                trajectory_kp: None,
                trajectory_ki: None,
                trajectory_kd: None,
                aileron_limit_rad: Some(0.35),
                elevator_limit_rad: Some(0.35),
                rudder_limit_rad: Some(0.35),
                throttle_baseline: Some(0.0),
            },
        );
        let config = FcConfig {
            estimator: FcEstimatorKind::Ekf,
            autopilot: FcAutopilotKind::ThreeLoop,
            guidance: FcGuidanceKind::AttitudeHold,
            reference_q_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
            base_rate_hz: 1_000,
            frame_budget_us: 2_000,
            ekf: Some(FcEkfConfig {
                mag_field: Some(FcMagFieldKind::Wmm2025),
                mag_epoch_decimal_year: Some(2025.0),
                ..FcEkfConfig::default()
            }),
            mekf: None,
            imm: None,
            autopilot_params: None,
            health: FcHealthConfig {
                imu_stale_after_s: 0.05,
                gnss_stale_after_s: 0.5,
                baro_stale_after_s: 10.0,
                mag_stale_after_s: 0.2,
                overrun_burst_count: 5,
            },
            fdir: None,
            actuator_channels: None,
            gain_schedule,
            phase_authority: None,
            estimator_lanes: None,
            autopilot_allocation: None,
            trajectory: None,
        };
        let (graph, bindings, pad) = minimal_graph();
        let mut runner =
            FcRunner::new(&config, graph, bindings, None, pad, None, 0.001, None).unwrap();
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

    #[test]
    fn explicit_anti_windup_block_wins_over_legacy_gain() {
        let cfg = FcAutopilotParams {
            anti_windup_gain: Some(9.0),
            anti_windup: Some(FcAntiWindupConfig::ObserverForm {
                tracking_time_s: 0.25,
            }),
            ..FcAutopilotParams::default()
        };
        let params = build_autopilot_params(&cfg, 0.001, None).expect("autopilot params build");
        assert_eq!(
            params.anti_windup,
            openbmp_fc::anti_windup::AntiWindupKind::ObserverForm {
                tracking_time_s: 0.25
            }
        );
    }
}
