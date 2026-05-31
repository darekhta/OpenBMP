//! Three-loop autopilot.
//!
//! A Stevens & Lewis 2015 academic three-loop
//! formulation: an inner rate loop, an outer attitude loop, and an
//! optional trajectory loop. Gains are scheduled by mission phase via
//! the parameter registry.
//!
//! - Rate loop: `gyro_estimate -> torque_command -> actuator deflection`.
//! - Attitude loop: `attitude_estimate -> rate_command`.
//! - Trajectory loop: `position_estimate -> attitude_command`.
//!
//! Each loop is a PID with anti-windup. When the actuator demand
//! saturates, the integrator freezes (back-calculation). This
//! baseline holds after the observer-form anti-windup review:
//! no boundedness or determinism test in the current academic
//! envelope justifies replacing the simpler back-calculation path.

use std::collections::BTreeMap;

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_physics::kinematics::quaternion_error_small_angle;

use crate::error::{AutopilotError, ControllerError};
use crate::filters::Biquad;
use crate::nav_metrics::{
    dynamic_pressure_air_relative, dynamic_pressure_air_relative_with_density,
};
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::tables::Table;
use crate::topics::{
    ActuatorCommand, AttitudeEstimate, AutopilotStatus, EngineDemand, EnvironmentEstimate,
    PositionEstimate, ReferenceState, VehicleStatus,
};
use crate::trajectory::{MinimumSnapTrajectory, YawProfile, flat_output_attitude_reference};

/// PID gain triple (Kp, Ki, Kd).
#[derive(Copy, Clone, Debug, Default)]
pub struct PidGains {
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,
}

/// Per-phase set of three-loop gains.
#[derive(Clone, Debug, Default)]
pub struct ThreeLoopGains {
    /// Rate-loop gains, body axes (roll, pitch, yaw).
    pub rate: [PidGains; 3],
    /// Attitude-loop gains, body axes.
    pub attitude: [PidGains; 3],
    /// Trajectory-loop gains, ECI axes.
    pub trajectory: [PidGains; 3],
    /// Maximum elevator deflection (rad).
    pub elevator_limit_rad: f64,
    /// Maximum aileron deflection (rad).
    pub aileron_limit_rad: f64,
    /// Maximum rudder deflection (rad).
    pub rudder_limit_rad: f64,
    /// Throttle baseline command in `[0, 1]`.
    pub throttle_baseline: f64,
}

/// Gain schedule keyed on mission phase.
#[derive(Clone, Debug, Default)]
pub struct GainSchedule {
    /// Mapping from `PhaseId.value()` to gains for that phase.
    pub by_phase: BTreeMap<u64, ThreeLoopGains>,
    /// Default gains used when no per-phase entry exists.
    pub default: ThreeLoopGains,
}

impl Table for GainSchedule {
    const NAME: &'static str = "autopilot.gain_schedule";
    fn validate(&self) -> Result<(), String> {
        for (phase_id, gains) in &self.by_phase {
            if gains.elevator_limit_rad < 0.0
                || gains.aileron_limit_rad < 0.0
                || gains.rudder_limit_rad < 0.0
            {
                return Err(format!(
                    "phase 0x{phase_id:016x}: actuator limits must be non-negative"
                ));
            }
            if !(0.0..=1.0).contains(&gains.throttle_baseline) {
                return Err(format!(
                    "phase 0x{phase_id:016x}: throttle_baseline must be in [0,1]"
                ));
            }
        }
        Ok(())
    }
}

impl ThreeLoopGains {
    /// Returns `true` when every per-axis trajectory gain (kp, ki, kd)
    /// is exactly zero. The gain schedule's default trajectory entry
    /// is all-zero, so a scenario that enables the trajectory loop
    /// without overriding this entry would silently run a no-op
    /// trajectory loop. Callers that opt into the trajectory loop
    /// should reject the configuration if this returns `true`.
    #[must_use]
    pub fn trajectory_gains_are_all_zero(&self) -> bool {
        self.trajectory
            .iter()
            .all(|g| g.kp == 0.0 && g.ki == 0.0 && g.kd == 0.0)
    }
}

/// Autopilot configuration parameters.
#[derive(Clone, Debug)]
pub struct AutopilotParams {
    /// Anti-windup strategy applied to all three PID loops
    /// (trajectory / attitude / rate). See
    /// [`crate::anti_windup::AntiWindupKind`] for the supported
    /// kinds. Defaults to back-calculation with unit gain.
    pub anti_windup: crate::anti_windup::AntiWindupKind,
    /// Deadband in body angular velocity below which the rate loop
    /// integrator is frozen.
    pub rate_deadband_rad_s: f64,
    /// `true` to steer by engine-gimbal thrust-vector control (TVC).
    /// When set, the rate-loop pitch/yaw output (`torque[1]`/`torque[2]`,
    /// already clamped to the gain-schedule limits, which a TVC scenario
    /// sets to the engine gimbal limit) is emitted as the
    /// `EngineDemand` gimbal angles instead of the hardcoded zero. The
    /// aerodynamic `ActuatorCommand` path is unchanged. Defaults to
    /// `false`, so aero-effector scenarios and all existing golden
    /// runs keep zero gimbal.
    pub thrust_vector_control: bool,
    /// Settling time (s, scenario clock) before TVC gimbal output is
    /// enabled. Until this time the gimbal command is held at zero so
    /// the vehicle flies open-loop (axial thrust, attitude neutrally
    /// stable) while the navigation estimate converges — preventing an
    /// EKF-initialisation transient from slamming the gimbal and
    /// exciting the (uniform-cluster-uncontrollable) roll axis. Defaults
    /// to `0.0` (gimbal active immediately). Only consulted when
    /// `thrust_vector_control` is set.
    pub thrust_vector_settle_s: f64,
    /// Maximum dynamic pressure (Pa) for closed-loop max-Q load relief.
    /// When the real dynamic pressure q = ½·ρ(h)·|v_air|² (actual
    /// atmospheric density at the navigated geocentric altitude and the
    /// AIR-RELATIVE velocity) exceeds this limit, the engine throttle is
    /// proportionally reduced toward `q_max/q` so the structural load is
    /// held at the limit — the standard launch-vehicle "throttle bucket"
    /// done closed-loop on q rather than open-loop on a schedule. Defaults
    /// to `f64::INFINITY` (no limit), so existing scenarios are unchanged.
    pub max_dynamic_pressure_pa: f64,
    /// `true` to enable the trajectory loop. When `false`, the
    /// reference attitude is taken directly from the guidance topic.
    pub trajectory_loop_enabled: bool,
    /// Trajectory-loop strategy.
    pub trajectory_kind: TrajectoryKind,
    /// Optional per-axis gyro notch filters.
    pub gyro_notch: Option<[crate::filters::NotchConfig; 3]>,
    /// Optional Cao-Hovakimyan L1 adaptive augmentation on the rate
    /// loop. When `Some(_)` the autopilot installs a
    /// per-axis [`crate::l1_adaptive_full::L1AdaptiveChannel`] and
    /// adds its augmentation to the PID rate-loop output every tick.
    /// The configured `L1AdaptiveParams` must already satisfy the
    /// bandwidth-projection inequality `ω_c · L < 1` — the runner's
    /// `[fc.autopilot_params.l1_adaptive]` parser asserts this at
    /// scenario load.
    #[cfg(feature = "l1-adaptive")]
    pub l1_adaptive: Option<crate::l1_adaptive_full::L1AdaptiveParams>,
    /// Rate-loop dispatch strategy. Defaults to
    /// [`RateLoopKind::Pid`] for byte-stable PID behaviour;
    /// scenarios that select [`RateLoopKind::Lqr`] must also install
    /// `lqr_gains` (the runner solves DARE at scenario load).
    pub rate_loop_kind: RateLoopKind,
    /// Per-axis LQR feedback gains. Only consulted
    /// when `rate_loop_kind == RateLoopKind::Lqr`. The runner solves
    /// the per-axis DARE at scenario load using
    /// [`crate::lqr::solve_lqr_rate_loop`] and installs the result
    /// here; the autopilot fails closed at first tick if `Lqr` is
    /// selected without gains.
    #[cfg(feature = "lqr")]
    pub lqr_gains: Option<[crate::lqr::LqrGains; 3]>,
    /// Per-axis INDI parameters. Only consulted when
    /// `rate_loop_kind == RateLoopKind::Indi`. The runner forwards
    /// the validated `[fc.autopilot_params.indi]` block here;
    /// composition with the L1 adaptive augmentation is rejected at
    /// scenario load to avoid filter-interaction concerns. The
    /// autopilot fails closed at first tick if `Indi` is selected
    /// without params.
    #[cfg(feature = "indi")]
    pub indi_params: Option<crate::indi::IndiParams>,
    /// Attitude-loop dispatch strategy. Defaults to
    /// [`AttitudeLoopKind::Pid`] for byte-stable PID behaviour;
    /// scenarios that select [`AttitudeLoopKind::Mpc`] must also
    /// install a built `RecedingHorizonAttitudeMpc` via
    /// [`AutopilotParams::attitude_mpc`] (the runner constructs it
    /// at scenario load using the configured params and the loop
    /// step `time.dt_s`).
    pub attitude_loop_kind: AttitudeLoopKind,
    /// Optional pre-built attitude MPC. Only consulted
    /// when `attitude_loop_kind == AttitudeLoopKind::Mpc`. The
    /// autopilot fails closed at first tick if `Mpc` is selected
    /// without an installed controller.
    #[cfg(feature = "mpc")]
    pub attitude_mpc: Option<std::sync::Arc<crate::mpc::RecedingHorizonAttitudeMpc>>,
}

impl Default for AutopilotParams {
    fn default() -> Self {
        Self {
            anti_windup: crate::anti_windup::AntiWindupKind::default(),
            rate_deadband_rad_s: 1e-3,
            thrust_vector_control: false,
            thrust_vector_settle_s: 0.0,
            max_dynamic_pressure_pa: f64::INFINITY,
            trajectory_loop_enabled: false,
            trajectory_kind: TrajectoryKind::Pid,
            gyro_notch: None,
            #[cfg(feature = "l1-adaptive")]
            l1_adaptive: None,
            rate_loop_kind: RateLoopKind::Pid,
            #[cfg(feature = "lqr")]
            lqr_gains: None,
            #[cfg(feature = "indi")]
            indi_params: None,
            attitude_loop_kind: AttitudeLoopKind::Pid,
            #[cfg(feature = "mpc")]
            attitude_mpc: None,
        }
    }
}

/// Attitude-loop dispatch strategy.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum AttitudeLoopKind {
    /// Per-axis PID attitude loop (default). Reads gain
    /// schedule per phase; integrator handled by the shared
    /// `pid_step` helper.
    #[default]
    Pid,
    /// Receding-horizon attitude MPC. Requires the
    /// `mpc` Cargo feature and a populated
    /// [`AutopilotParams::attitude_mpc`] field. The autopilot fails
    /// closed at first tick if either is missing.
    Mpc,
}

/// Rate-loop dispatch strategy.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum RateLoopKind {
    /// PID rate loop (default). Reads gain schedule per
    /// phase; integrator handled by the shared `pid_step` helper.
    #[default]
    Pid,
    /// Per-axis LQR rate loop with augmented integral
    /// state. Requires the `lqr` Cargo feature and a populated
    /// [`AutopilotParams::lqr_gains`] field. The autopilot fails
    /// closed at first tick if either is missing.
    Lqr,
    /// Per-axis INDI rate loop (Smeur-Chu-de Croon
    /// 2016). Requires the `indi` Cargo feature and a populated
    /// [`AutopilotParams::indi_params`] field. The autopilot fails
    /// closed at first tick if either is missing. Composition with
    /// the L1 adaptive augmentation is rejected at scenario load.
    Indi,
}

impl ParamSection for AutopilotParams {
    const NAME: &'static str = "autopilot.three_loop";
}

#[derive(Debug, Default)]
struct PidState {
    integral: f64,
    last_error: f64,
}

impl PidState {
    fn reset_and_prime(&mut self, error: f64) {
        self.integral = 0.0;
        self.last_error = error;
    }
}

/// Trajectory-loop strategy.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum TrajectoryKind {
    /// Existing PID position-to-attitude correction.
    #[default]
    Pid,
    /// Mellinger & Kumar 2011 differential-flatness trajectory
    /// tracker. The autopilot's trajectory loop builds the attitude
    /// reference (and exposes body-rate / angular-acceleration
    /// references) analytically from the flat outputs of a piecewise
    /// minimum-snap polynomial. Selecting this variant requires
    /// installing a `MinimumSnapTrajectory` via
    /// [`ThreeLoopAutopilot::with_minimum_snap_trajectory`] (or by
    /// declaring `[fc.trajectory]` in the scenario); otherwise the
    /// autopilot fails closed at first tick.
    DifferentialFlatness,
}

/// Three-loop autopilot job.
#[derive(Debug)]
pub struct ThreeLoopAutopilot {
    name: &'static str,
    rate_state: [PidState; 3],
    attitude_state: [PidState; 3],
    trajectory_state: [PidState; 3],
    last_predict_time_s: f64,
    last_phase_id: Option<u64>,
    schedule: GainSchedule,
    params: AutopilotParams,
    gyro_notch_state: Option<[Biquad; 3]>,
    /// Optional minimum-snap trajectory consumed when
    /// `params.trajectory_kind == TrajectoryKind::DifferentialFlatness`.
    /// The autopilot owns the trajectory.
    minimum_snap_trajectory: Option<MinimumSnapTrajectory>,
    /// Optional constant yaw override for the installed trajectory.
    /// Scenario `[fc.trajectory].yaw_rad` sets this; programmatic users
    /// that omit it inherit yaw from the bus reference.
    minimum_snap_yaw_rad: Option<f64>,
    /// Per-axis L1 adaptive state. Used only when
    /// [`AutopilotParams::l1_adaptive`] is `Some(_)` and the
    /// `l1-adaptive` feature is on.
    #[cfg(feature = "l1-adaptive")]
    l1_state: [crate::l1_adaptive_full::L1AdaptiveChannel; 3],
    /// Per-axis LQR integrator state. Updated by
    /// the rate loop only when
    /// `params.rate_loop_kind == RateLoopKind::Lqr`.
    #[cfg(feature = "lqr")]
    lqr_integrators: [f64; 3],
    /// Per-axis INDI channel state. Filter and
    /// previous-command state; consulted only when
    /// `params.rate_loop_kind == RateLoopKind::Indi`. Constructed
    /// lazily on the first INDI step so the channels can be sized
    /// against the loop step `dt` discovered from the bus clock.
    #[cfg(feature = "indi")]
    indi_state: Option<[crate::indi::IndiChannel; 3]>,
}

impl ThreeLoopAutopilot {
    /// Constructs the autopilot job with the academic-default gain
    /// schedule (every phase falls through to the default gains, which
    /// are calm textbook PID tunings — see [`default_gains`]).
    #[must_use]
    pub fn new() -> Self {
        Self::with_schedule(default_gain_schedule())
    }

    /// Constructs the autopilot job with the given gain schedule.
    #[must_use]
    pub fn with_schedule(schedule: GainSchedule) -> Self {
        Self {
            name: "autopilot.three_loop",
            rate_state: <[PidState; 3] as Default>::default(),
            attitude_state: <[PidState; 3] as Default>::default(),
            trajectory_state: <[PidState; 3] as Default>::default(),
            last_predict_time_s: 0.0,
            last_phase_id: None,
            schedule,
            params: AutopilotParams::default(),
            gyro_notch_state: None,
            minimum_snap_trajectory: None,
            minimum_snap_yaw_rad: None,
            #[cfg(feature = "l1-adaptive")]
            l1_state: [crate::l1_adaptive_full::L1AdaptiveChannel::new(); 3],
            #[cfg(feature = "lqr")]
            lqr_integrators: [0.0; 3],
            #[cfg(feature = "indi")]
            indi_state: None,
        }
    }

    /// Installs a minimum-snap differential-flatness trajectory.
    /// Required when [`AutopilotParams::trajectory_kind`] is
    /// [`TrajectoryKind::DifferentialFlatness`]; without it the
    /// trajectory loop fails closed at first tick.
    #[must_use]
    pub fn with_minimum_snap_trajectory(mut self, trajectory: MinimumSnapTrajectory) -> Self {
        self.minimum_snap_trajectory = Some(trajectory);
        self
    }

    /// Installs a constant yaw angle for the minimum-snap
    /// differential-flatness trajectory. Programmatic callers may omit
    /// this and keep the legacy bus-reference yaw inheritance.
    #[must_use]
    pub fn with_minimum_snap_yaw_rad(mut self, yaw_rad: f64) -> Self {
        self.minimum_snap_yaw_rad = Some(yaw_rad);
        self
    }

    /// Replaces the gain schedule, returning the updated autopilot.
    #[must_use]
    pub fn with_gain_schedule(mut self, schedule: GainSchedule) -> Self {
        self.schedule = schedule;
        self
    }

    /// Replaces the autopilot parameters (trajectory-loop enable, etc).
    #[must_use]
    pub fn with_params(mut self, params: AutopilotParams) -> Self {
        self.params = params;
        self.gyro_notch_state = None;
        #[cfg(feature = "indi")]
        {
            // Filter coefficients depend on cutoff/kind; force a
            // fresh build on the first INDI step.
            self.indi_state = None;
        }
        self
    }

    /// Per-axis LQR rate-loop step. Mirrors the
    /// PID-loop interface so the rate-loop dispatch site treats both
    /// kinds uniformly. Updates the per-axis integrator (subject to
    /// the rate-deadband freeze) and applies the configured anti-
    /// windup strategy on saturation.
    #[cfg(feature = "lqr")]
    #[allow(clippy::too_many_arguments)]
    fn lqr_step(
        &mut self,
        axis: usize,
        rate_cmd: f64,
        omega_meas: f64,
        gains: &crate::lqr::LqrGains,
        dt: f64,
        torque_min: f64,
        torque_max: f64,
        integrate: bool,
    ) -> (f64, bool) {
        let omega_err = omega_meas - rate_cmd;
        if integrate {
            self.lqr_integrators[axis] += dt * omega_err;
        }
        let raw = -gains.k_omega * omega_err - gains.k_int * self.lqr_integrators[axis];
        let clamped = raw.clamp(torque_min, torque_max);
        let saturated = (raw - clamped).abs() > 0.0;
        if saturated && integrate {
            let excess = raw - clamped;
            // PID uses `+ki * integral`; LQR uses `-k_int * integral`.
            // Feed the opposite excess sign so a high clamp moves the
            // LQR integral upward, reducing the next raw command.
            self.params
                .anti_windup
                .apply(&mut self.lqr_integrators[axis], -excess, dt);
        }
        (clamped, saturated)
    }

    /// Per-axis INDI rate-loop step. Lazily
    /// constructs the channel state on first use (the filter
    /// coefficients depend on `dt` which the autopilot only sees at
    /// runtime). Anti-windup is implicit through the clamp; the
    /// `AutopilotParams::anti_windup` field is intentionally
    /// ignored here — saturation is bounded by construction.
    #[cfg(feature = "indi")]
    #[allow(clippy::too_many_arguments)]
    fn indi_step(
        &mut self,
        params: &crate::indi::IndiParams,
        axis: usize,
        rate_cmd: f64,
        omega_meas: f64,
        dt: f64,
        torque_min: f64,
        torque_max: f64,
    ) -> Result<(f64, bool), ControllerError> {
        if self.indi_state.is_none() {
            let ch = crate::indi::IndiChannel::new(params, dt).map_err(|err| {
                ControllerError::from(AutopilotError::Trajectory {
                    reason: format!("INDI channel construction failed (axis {axis}): {err}"),
                })
            })?;
            self.indi_state = Some([ch; 3]);
        }
        let Some(channels) = self.indi_state.as_mut() else {
            return Err(ControllerError::from(AutopilotError::Trajectory {
                reason: "INDI state lost between primer and step (impossible)".to_string(),
            }));
        };
        Ok(channels[axis].step(
            params, axis, omega_meas, rate_cmd, torque_min, torque_max, dt,
        ))
    }

    fn gains_for_phase(&self, phase: u64) -> &ThreeLoopGains {
        self.schedule
            .by_phase
            .get(&phase)
            .unwrap_or(&self.schedule.default)
    }

    fn filtered_omega_body(&mut self, omega_body_rad_s: Vector3<f64>, dt_s: f64) -> Vector3<f64> {
        let Some(configs) = self.params.gyro_notch else {
            return omega_body_rad_s;
        };
        if self.gyro_notch_state.is_none() {
            let sample_rate_hz = 1.0 / dt_s;
            let mut filters = [Biquad::default(); 3];
            for (slot, config) in filters.iter_mut().zip(configs) {
                let Ok(filter) = Biquad::notch(config, sample_rate_hz) else {
                    return omega_body_rad_s;
                };
                *slot = filter;
            }
            self.gyro_notch_state = Some(filters);
        }
        let Some(filters) = &mut self.gyro_notch_state else {
            return omega_body_rad_s;
        };
        Vector3::new(
            filters[0].step(omega_body_rad_s.x),
            filters[1].step(omega_body_rad_s.y),
            filters[2].step(omega_body_rad_s.z),
        )
    }
}

impl Default for ThreeLoopAutopilot {
    fn default() -> Self {
        Self::new()
    }
}

#[allow(clippy::too_many_arguments)]
fn pid_step(
    state: &mut PidState,
    gains: &PidGains,
    error: f64,
    dt: f64,
    saturate_against_min: f64,
    saturate_against_max: f64,
    anti_windup: &crate::anti_windup::AntiWindupKind,
    integrate: bool,
) -> (f64, bool) {
    if integrate {
        state.integral += error * dt;
    }
    let derivative = if dt > 0.0 {
        (error - state.last_error) / dt
    } else {
        0.0
    };
    state.last_error = error;
    let raw = gains.kp * error + gains.ki * state.integral + gains.kd * derivative;
    let clamped = raw.clamp(saturate_against_min, saturate_against_max);
    let saturated = (raw - clamped).abs() > 0.0;
    if saturated && integrate {
        let excess = raw - clamped;
        anti_windup.apply(&mut state.integral, excess, dt);
    }
    (clamped, saturated)
}

impl Job for ThreeLoopAutopilot {
    fn name(&self) -> &'static str {
        self.name
    }

    #[allow(clippy::too_many_lines)]
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        let now_s = ctx.clock.now().as_seconds();
        let dt = if self.last_predict_time_s > 0.0 {
            (now_s - self.last_predict_time_s).max(0.0)
        } else {
            0.0
        };
        self.last_predict_time_s = now_s;
        if dt <= 0.0 {
            return Ok(());
        }

        let Some((attitude, _)) = ctx.bus.latest::<AttitudeEstimate>()? else {
            return Ok(());
        };
        let position = ctx.bus.latest::<PositionEstimate>()?.map(|(p, _)| p);
        let environment = ctx
            .bus
            .latest::<EnvironmentEstimate>()
            .ok()
            .flatten()
            .map(|(e, _)| e);
        let reference = ctx
            .bus
            .latest::<ReferenceState>()?
            .map(|(r, _)| r)
            .unwrap_or_default();
        let Some((status, _)) = ctx.bus.latest::<VehicleStatus>()? else {
            return Ok(());
        };
        if !(status.armed && status.in_flight) {
            return Ok(());
        }
        let phase_changed = self.last_phase_id != Some(status.phase_id);
        if phase_changed {
            self.last_phase_id = Some(status.phase_id);
            #[cfg(feature = "lqr")]
            {
                self.lqr_integrators = [0.0; 3];
            }
            #[cfg(feature = "indi")]
            {
                self.indi_state = None;
            }
        }
        let gains = self.gains_for_phase(status.phase_id).clone();
        let mut saturated = false;

        // Attitude loop input: small-angle error in body frame.
        let mut attitude_error = if self.params.thrust_vector_control {
            thrust_axis_error_body(attitude.q_body_to_eci_xyzw, reference.q_body_to_eci_xyzw)
        } else {
            quaternion_error_small_angle(attitude.q_body_to_eci_xyzw, reference.q_body_to_eci_xyzw)
        };
        let mut differential_flatness_active = false;
        let mut differential_flatness_reference_suppressed = false;

        // DifferentialFlatness trajectory loop.
        // Generates the attitude reference from the installed
        // minimum-snap trajectory's flat outputs, independent of the
        // bus `ReferenceState.position_eci_m`. The `Pid` variant
        // continues to read the bus reference and an estimator
        // position, gated below.
        if self.params.trajectory_loop_enabled
            && self.params.trajectory_kind == TrajectoryKind::DifferentialFlatness
        {
            differential_flatness_active = true;
            let trajectory = self.minimum_snap_trajectory.as_ref().ok_or_else(|| {
                ControllerError::from(AutopilotError::Trajectory {
                    reason: "DifferentialFlatness trajectory_kind selected without an \
                             installed MinimumSnapTrajectory; install one via \
                             ThreeLoopAutopilot::with_minimum_snap_trajectory."
                        .to_string(),
                })
            })?;
            let now_s = ctx.clock.now().as_seconds();
            let flat = trajectory.evaluate(now_s);
            let yaw_rad = self
                .minimum_snap_yaw_rad
                .unwrap_or_else(|| reference_yaw_rad(reference.q_body_to_eci_xyzw));
            let yaw = YawProfile {
                yaw_rad,
                yaw_rate_rad_s: 0.0,
                yaw_accel_rad_s2: 0.0,
            };
            if let Some(reference_kin) = flat_output_attitude_reference(&flat, yaw) {
                let q = reference_kin.q_body_to_eci.into_inner();
                let q_xyzw = [q.i, q.j, q.k, q.w];
                attitude_error = if self.params.thrust_vector_control {
                    thrust_axis_error_body(attitude.q_body_to_eci_xyzw, q_xyzw)
                } else {
                    quaternion_error_small_angle(attitude.q_body_to_eci_xyzw, q_xyzw)
                };
            } else {
                differential_flatness_reference_suppressed = true;
            }
            // Free-fall (returned None): leave the attitude_error
            // initialised from the bus reference. The flat-output
            // path has no thrust direction in that regime and publishes
            // an autopilot status bit for FDIR.
        }
        let _ = ctx.bus.publish(AutopilotStatus {
            differential_flatness_active,
            differential_flatness_reference_suppressed,
        });

        // Trajectory loop: feeds an attitude-error correction. Active
        // only when enabled by params and a position reference is
        // present. The DifferentialFlatness branch is handled above;
        // this block covers the bus-reference-driven variants only.
        if self.params.trajectory_loop_enabled
            && self.params.trajectory_kind != TrajectoryKind::DifferentialFlatness
            && reference.position_eci_m.norm() > 0.0
            && let Some(pos) = position
        {
            match self.params.trajectory_kind {
                TrajectoryKind::Pid => {
                    let pos_error_eci = reference.position_eci_m - pos.position_eci_m;
                    let q_est =
                        nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                            attitude.q_body_to_eci_xyzw[3],
                            attitude.q_body_to_eci_xyzw[0],
                            attitude.q_body_to_eci_xyzw[1],
                            attitude.q_body_to_eci_xyzw[2],
                        ));
                    let r_eci_to_body = q_est.to_rotation_matrix().transpose();
                    let pos_error_body = r_eci_to_body * pos_error_eci;
                    for i in 0..3 {
                        if phase_changed {
                            self.trajectory_state[i].reset_and_prime(pos_error_body[i]);
                        }
                        let (cmd, sat) = pid_step(
                            &mut self.trajectory_state[i],
                            &gains.trajectory[i],
                            pos_error_body[i],
                            dt,
                            -1.0,
                            1.0,
                            &self.params.anti_windup,
                            true,
                        );
                        attitude_error[i] += cmd;
                        saturated |= sat;
                    }
                }
                // The outer `if` excludes DifferentialFlatness; the
                // dedicated branch above handles it.
                TrajectoryKind::DifferentialFlatness => {}
            }
        }

        // Attitude-loop dispatch. PID is the
        // default; MPC consumes the same
        // attitude-error vector and returns the optimal first-step
        // commanded body rate via the pre-built receding-horizon
        // controller. The MPC's saturation is internal to its box
        // constraint (rate_limit_rad_s) so the legacy `saturated`
        // flag is not affected on MPC tick paths.
        let mut rate_cmd = Vector3::zeros();
        match self.params.attitude_loop_kind {
            AttitudeLoopKind::Pid => {
                for i in 0..3 {
                    if phase_changed {
                        self.attitude_state[i].reset_and_prime(attitude_error[i]);
                    }
                    let (cmd, sat) = pid_step(
                        &mut self.attitude_state[i],
                        &gains.attitude[i],
                        attitude_error[i],
                        dt,
                        -1e3,
                        1e3,
                        &self.params.anti_windup,
                        true,
                    );
                    rate_cmd[i] = cmd;
                    saturated |= sat;
                }
            }
            #[cfg(feature = "mpc")]
            AttitudeLoopKind::Mpc => {
                let mpc = self.params.attitude_mpc.as_ref().ok_or_else(|| {
                    ControllerError::from(AutopilotError::Trajectory {
                        reason: "AttitudeLoopKind::Mpc selected without an installed \
                                 RecedingHorizonAttitudeMpc; the runner must build one via \
                                 AutopilotParams.attitude_mpc."
                            .to_string(),
                    })
                })?;
                let u0 = mpc
                    .solve([attitude_error[0], attitude_error[1], attitude_error[2]])
                    .map_err(|err| {
                        ControllerError::from(AutopilotError::Trajectory {
                            reason: format!("attitude MPC solve failed: {err}"),
                        })
                    })?;
                for (i, value) in u0.iter().enumerate() {
                    rate_cmd[i] = *value;
                }
            }
            #[cfg(not(feature = "mpc"))]
            AttitudeLoopKind::Mpc => {
                return Err(ControllerError::from(AutopilotError::Trajectory {
                    reason: "AttitudeLoopKind::Mpc selected but the `mpc` feature is not \
                             enabled; rebuild with --features mpc."
                        .to_string(),
                }));
            }
        }

        // Rate loop: rate_cmd vs measured -> actuator deflection.
        // Per-axis integrator freeze when measured body rate is below
        // the configured deadband — prevents integrator wind-up at
        // very low rates where measurement noise dominates the signal.
        let omega_body_rad_s = self.filtered_omega_body(attitude.omega_body_rad_s, dt);
        let rate_error = rate_cmd - omega_body_rad_s;
        let mut torque = Vector3::zeros();
        for i in 0..3 {
            if phase_changed {
                self.rate_state[i].reset_and_prime(rate_error[i]);
            }
            let limit = match i {
                0 => gains.aileron_limit_rad,
                1 => gains.elevator_limit_rad,
                _ => gains.rudder_limit_rad,
            };
            let integrate = omega_body_rad_s[i].abs() >= self.params.rate_deadband_rad_s;
            // Rate-loop dispatch. PID is the
            // default; LQR uses the per-axis gains the
            // runner pre-solved at scenario load.
            let (cmd, sat) = match self.params.rate_loop_kind {
                RateLoopKind::Pid => pid_step(
                    &mut self.rate_state[i],
                    &gains.rate[i],
                    rate_error[i],
                    dt,
                    -limit,
                    limit,
                    &self.params.anti_windup,
                    integrate,
                ),
                #[cfg(feature = "lqr")]
                RateLoopKind::Lqr => {
                    let gains_axis = self.params.lqr_gains.as_ref().ok_or_else(|| {
                        ControllerError::from(AutopilotError::Trajectory {
                            reason: "RateLoopKind::Lqr selected without lqr_gains; runner must \
                                     install solved gains via AutopilotParams.lqr_gains."
                                .to_string(),
                        })
                    })?[i];
                    self.lqr_step(
                        i,
                        rate_cmd[i],
                        omega_body_rad_s[i],
                        &gains_axis,
                        dt,
                        -limit,
                        limit,
                        integrate,
                    )
                }
                #[cfg(not(feature = "lqr"))]
                RateLoopKind::Lqr => {
                    return Err(ControllerError::from(AutopilotError::Trajectory {
                        reason: "RateLoopKind::Lqr selected but the `lqr` feature is not \
                                 enabled; rebuild with --features lqr."
                            .to_string(),
                    }));
                }
                #[cfg(feature = "indi")]
                RateLoopKind::Indi => {
                    let indi_params = self.params.indi_params.ok_or_else(|| {
                        ControllerError::from(AutopilotError::Trajectory {
                            reason: "RateLoopKind::Indi selected without indi_params; runner must \
                                 install validated INDI params via AutopilotParams.indi_params."
                                .to_string(),
                        })
                    })?;
                    self.indi_step(
                        &indi_params,
                        i,
                        rate_cmd[i],
                        omega_body_rad_s[i],
                        dt,
                        -limit,
                        limit,
                    )?
                }
                #[cfg(not(feature = "indi"))]
                RateLoopKind::Indi => {
                    return Err(ControllerError::from(AutopilotError::Trajectory {
                        reason: "RateLoopKind::Indi selected but the `indi` feature is not \
                                 enabled; rebuild with --features indi."
                            .to_string(),
                    }));
                }
            };
            #[cfg(feature = "l1-adaptive")]
            let mut axis_cmd = cmd;
            #[cfg(not(feature = "l1-adaptive"))]
            let axis_cmd = cmd;
            // Full Cao-Hovakimyan L1 adaptive
            // augmentation: the predictor sees the measured body
            // angular rate as plant state, the rate loop's command as
            // reference, and the PID output as baseline command.
            // The augmentation is added to the baseline; the result
            // is re-clamped to the per-axis actuator limit.
            //
            // L1 augmentation is intentionally
            // suppressed when the rate loop is INDI: INDI's filtered
            // ω̇_meas term already absorbs matched disturbance, so
            // L1 on top creates filter-interaction concerns. The
            // scenario parser rejects this combination; the runtime
            // guard is defence-in-depth.
            #[cfg(feature = "l1-adaptive")]
            if self.params.rate_loop_kind != RateLoopKind::Indi
                && let Some(l1_params) = self.params.l1_adaptive
            {
                let augmentation =
                    self.l1_state[i].step(&l1_params, omega_body_rad_s[i], rate_cmd[i], cmd, dt);
                axis_cmd += augmentation;
                let l1_limited = axis_cmd.clamp(-limit, limit);
                saturated |= (axis_cmd - l1_limited).abs() > 0.0;
                axis_cmd = l1_limited;
            }
            torque[i] = axis_cmd;
            saturated |= sat;
        }

        let cmd = ActuatorCommand {
            time: ctx.clock.now(),
            aileron_rad: torque[0],
            elevator_rad: torque[1],
            rudder_rad: torque[2],
            body_flap_rad: 0.0,
            saturated,
        };
        let _ = ctx.bus.publish(cmd);

        // Thrust-vector control: steer by gimbaling engines.
        //
        // Axis convention: the engine cluster thrusts along body +z, so
        // body +z is the ROLL (thrust) axis and body +x / +y are the two
        // TRANSVERSE axes a gimbal can control. The autopilot's `torque`
        // vector is the control effort about body axes [x, y, z]. A
        // gimbal-pitch deflection produces a body-y moment; a gimbal-yaw
        // deflection produces a body-x moment. So the two transverse
        // commands `torque[0]` (body-x) and `torque[1]` (body-y) drive
        // gimbal-yaw and gimbal-pitch respectively. `torque[2]` is roll
        // about the thrust axis, which a uniform cluster cannot produce
        // (it would need differential gimbal / RCS) and is left unmapped.
        //
        // Sign: the cluster is mounted aft of the CG, so a positive
        // gimbal deflection yields a negative moment about its control
        // axis; the gimbal command is therefore the negated rate-loop
        // output for both transverse axes.
        let tvc_settled = ctx.clock.now().as_seconds() >= self.params.thrust_vector_settle_s;
        let (gimbal_pitch_rad, gimbal_yaw_rad) = if self.params.thrust_vector_control && tvc_settled
        {
            // Defensive: never emit a non-finite gimbal command to the
            // actuator stack — a transient non-finite control signal
            // commands zero gimbal rather than faulting the engine rack.
            let gp = if torque[1].is_finite() {
                -torque[1]
            } else {
                0.0
            };
            let gy = if torque[0].is_finite() {
                -torque[0]
            } else {
                0.0
            };
            (gp, gy)
        } else {
            (0.0, 0.0)
        };
        // Closed-loop max-Q load relief: if the real dynamic pressure
        // (actual density at the navigated geocentric altitude × the
        // air-relative speed²) exceeds the configured limit, throttle
        // down toward q_max/q so the structural load is held at the limit.
        // No-op when max_dynamic_pressure_pa is infinite (the default) or
        // there is no navigation estimate yet.
        let throttle_unit = match position.as_ref() {
            Some(p) if self.params.max_dynamic_pressure_pa.is_finite() => {
                let q = environment.map_or_else(
                    || dynamic_pressure_air_relative(p),
                    |env| dynamic_pressure_air_relative_with_density(p, env.density_kg_m3),
                );
                if q > self.params.max_dynamic_pressure_pa && q > 0.0 {
                    (gains.throttle_baseline * (self.params.max_dynamic_pressure_pa / q))
                        .clamp(0.0, gains.throttle_baseline)
                } else {
                    gains.throttle_baseline
                }
            }
            _ => gains.throttle_baseline,
        };
        let engine = EngineDemand {
            time: ctx.clock.now(),
            throttle_unit,
            gimbal_pitch_rad,
            gimbal_yaw_rad,
            ignite: false,
            shutdown: false,
        };
        let _ = ctx.bus.publish(engine);
        Ok(())
    }
}

/// Extract the yaw (rotation about the inertial z-axis) of a body→ECI
/// quaternion in scalar-last `[x, y, z, w]` ordering. Used by the
/// differential-flatness trajectory loop so the attitude reference
/// inherits the scenario's commanded heading instead of pinning yaw
/// to zero.
fn reference_yaw_rad(q_body_to_eci_xyzw: [f64; 4]) -> f64 {
    let q = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q_body_to_eci_xyzw[3],
        q_body_to_eci_xyzw[0],
        q_body_to_eci_xyzw[1],
        q_body_to_eci_xyzw[2],
    ));
    let (_roll, _pitch, yaw) = q.euler_angles();
    yaw
}

fn unit_quat_xyzw(q_body_to_eci_xyzw: [f64; 4]) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q_body_to_eci_xyzw[3],
        q_body_to_eci_xyzw[0],
        q_body_to_eci_xyzw[1],
        q_body_to_eci_xyzw[2],
    ))
}

fn thrust_axis_error_body(q_estimate_xyzw: [f64; 4], q_reference_xyzw: [f64; 4]) -> Vector3<f64> {
    let q_est = unit_quat_xyzw(q_estimate_xyzw);
    let q_ref = unit_quat_xyzw(q_reference_xyzw);
    let current_axis_eci = q_est * Vector3::z();
    let reference_axis_eci = q_ref * Vector3::z();
    let axis_cross = current_axis_eci.cross(&reference_axis_eci);
    let sin_angle = axis_cross.norm();
    if sin_angle <= f64::EPSILON {
        return Vector3::zeros();
    }
    let cos_angle = current_axis_eci.dot(&reference_axis_eci).clamp(-1.0, 1.0);
    let error_eci = (axis_cross / sin_angle) * sin_angle.atan2(cos_angle);
    q_est.inverse_transform_vector(&error_eci)
}

/// Default academic gain schedule — every phase falls through to the
/// default gains. The runner overrides these via the table registry.
#[must_use]
pub fn default_gain_schedule() -> GainSchedule {
    GainSchedule {
        by_phase: BTreeMap::new(),
        default: default_gains(),
    }
}

/// Default academic gains — calm textbook PID tunings shipped as the
/// fallback when no phase-specific gain entry exists.
#[must_use]
pub fn default_gains() -> ThreeLoopGains {
    ThreeLoopGains {
        rate: [PidGains {
            kp: 0.5,
            ki: 0.0,
            kd: 0.05,
        }; 3],
        attitude: [
            PidGains {
                kp: 2.0,
                ki: 0.0,
                kd: 0.0,
            },
            PidGains {
                kp: 2.0,
                ki: 0.0,
                kd: 0.0,
            },
            PidGains {
                kp: 1.0,
                ki: 0.0,
                kd: 0.0,
            },
        ],
        trajectory: [PidGains::default(); 3],
        elevator_limit_rad: 0.35, // 20 deg
        aileron_limit_rad: 0.35,
        rudder_limit_rad: 0.35,
        throttle_baseline: 0.0,
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};
    use openbmp_core::SimTime;

    use super::{PidGains, PidState, pid_step, reference_yaw_rad, thrust_axis_error_body};

    #[test]
    fn reference_yaw_rad_recovers_z_rotation() {
        let theta: f64 = 0.4;
        let half = theta / 2.0;
        let q_xyzw = [0.0, 0.0, half.sin(), half.cos()];
        let yaw = reference_yaw_rad(q_xyzw);
        assert!((yaw - theta).abs() < 1.0e-12);
    }

    #[test]
    fn thrust_axis_error_ignores_roll_about_thrust_axis() {
        let pitch = 0.3;
        let roll = 1.2;
        let q_pitch = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), pitch);
        let q_roll = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), roll);
        let q_ref = (q_pitch * q_roll).into_inner();
        let err =
            thrust_axis_error_body([0.0, 0.0, 0.0, 1.0], [q_ref.i, q_ref.j, q_ref.k, q_ref.w]);

        assert!(err.x.abs() < 1.0e-12, "unexpected body-x error: {err:?}");
        assert!(
            (err.y - pitch).abs() < 1.0e-12,
            "unexpected pitch error: {err:?}"
        );
        assert!(
            err.z.abs() < 1.0e-12,
            "roll about thrust axis must be ignored: {err:?}"
        );

        let q_est = q_pitch.into_inner();
        let err_roll_only = thrust_axis_error_body(
            [q_est.i, q_est.j, q_est.k, q_est.w],
            [q_ref.i, q_ref.j, q_ref.k, q_ref.w],
        );
        assert!(
            err_roll_only.norm() < 1.0e-12,
            "same thrust axis with different roll should have no TVC error: {err_roll_only:?}"
        );
    }

    #[test]
    fn pid_reset_and_prime_suppresses_derivative_kick() {
        let mut state = PidState {
            integral: 5.0,
            last_error: -2.0,
        };
        let gains = PidGains {
            kp: 0.0,
            ki: 0.0,
            kd: 10.0,
        };

        state.reset_and_prime(1.25);
        assert_eq!(state.integral.to_bits(), 0.0_f64.to_bits());
        let (cmd, saturated) = pid_step(
            &mut state,
            &gains,
            1.25,
            0.01,
            -1000.0,
            1000.0,
            &crate::anti_windup::AntiWindupKind::default(),
            true,
        );

        assert!(!saturated);
        assert_eq!(cmd.to_bits(), 0.0_f64.to_bits());
        assert!((state.integral - 0.0125).abs() < 1.0e-15);
    }

    #[test]
    fn dynamic_pressure_air_relative_zero_corotating_positive_in_airflow() {
        use crate::nav_metrics::dynamic_pressure_air_relative;
        use crate::topics::PositionEstimate;
        let omega = openbmp_physics::frames::WGS84_OMEGA_RAD_S;
        let r = 6_371_000.0;
        // Co-rotating at the equatorial surface (v_eci = Ω×r): the
        // air-relative speed is ~zero, so the real dynamic pressure is ~0
        // even though the inertial speed is ~465 m/s (the bug the
        // sea-level-density |v_eci|² proxy had).
        let corotating = PositionEstimate {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(r, 0.0, 0.0),
            velocity_eci_m_s: Vector3::new(0.0, omega * r, 0.0),
            accel_bias_body_m_s2: Vector3::zeros(),
        };
        assert!(
            dynamic_pressure_air_relative(&corotating) < 1.0,
            "co-rotating surface q should be ~0, got {}",
            dynamic_pressure_air_relative(&corotating)
        );
        // 300 m/s eastward airspeed at ~10 km altitude → a sizeable q.
        let r2 = r + 10_000.0;
        let flying = PositionEstimate {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(r2, 0.0, 0.0),
            velocity_eci_m_s: Vector3::new(0.0, omega * r2 + 300.0, 0.0),
            accel_bias_body_m_s2: Vector3::zeros(),
        };
        let q = dynamic_pressure_air_relative(&flying);
        assert!(
            (5_000.0..50_000.0).contains(&q),
            "10 km / 300 m/s air-relative q out of expected range: {q}"
        );
    }

    #[test]
    fn max_q_throttle_uses_published_environment_density() {
        use std::collections::BTreeMap;

        use openbmp_core::StepIndex;

        use crate::autopilot::{
            AutopilotParams, GainSchedule, PidGains, ThreeLoopAutopilot, ThreeLoopGains,
        };
        use crate::bus::Bus;
        use crate::clock::SimulatedClock;
        use crate::scheduler::{Job as _, JobContext};
        use crate::topics::{
            ActuatorCommand, AttitudeEstimate, EngineDemand, EnvironmentEstimate, PositionEstimate,
            ReferenceState, VehicleStatus,
        };

        let bus = Bus::new();
        bus.register::<AttitudeEstimate>().unwrap();
        bus.register::<PositionEstimate>().unwrap();
        bus.register::<EnvironmentEstimate>().unwrap();
        bus.register::<ReferenceState>().unwrap();
        bus.register::<VehicleStatus>().unwrap();
        bus.register::<ActuatorCommand>().unwrap();
        bus.register::<EngineDemand>().unwrap();

        let omega = openbmp_physics::frames::WGS84_OMEGA_RAD_S;
        let r = 6_371_000.0;
        bus.publish(AttitudeEstimate {
            time: SimTime::from_seconds(0.001),
            q_body_to_eci_xyzw: [0.0, 0.0, 0.0, 1.0],
            omega_body_rad_s: Vector3::zeros(),
            gyro_bias_body_rad_s: Vector3::zeros(),
        })
        .unwrap();
        bus.publish(PositionEstimate {
            time: SimTime::from_seconds(0.001),
            position_eci_m: Vector3::new(r, 0.0, 0.0),
            velocity_eci_m_s: Vector3::new(0.0, omega * r + 300.0, 0.0),
            accel_bias_body_m_s2: Vector3::zeros(),
        })
        .unwrap();
        bus.publish(EnvironmentEstimate {
            time: SimTime::from_seconds(0.001),
            density_kg_m3: 0.0,
        })
        .unwrap();
        bus.publish(VehicleStatus {
            armed: true,
            in_flight: true,
            phase_id: 0,
            safe_state_requested: false,
        })
        .unwrap();
        bus.publish(ReferenceState {
            q_body_to_eci_xyzw: [0.0, 0.0, 0.0, 1.0],
            ..ReferenceState::default()
        })
        .unwrap();

        let gains = ThreeLoopGains {
            rate: [PidGains::default(); 3],
            attitude: [PidGains::default(); 3],
            trajectory: [PidGains::default(); 3],
            elevator_limit_rad: 1.0,
            aileron_limit_rad: 1.0,
            rudder_limit_rad: 1.0,
            throttle_baseline: 1.0,
        };
        let schedule = GainSchedule {
            by_phase: BTreeMap::new(),
            default: gains,
        };
        let mut autopilot =
            ThreeLoopAutopilot::with_schedule(schedule).with_params(AutopilotParams {
                max_dynamic_pressure_pa: 10_000.0,
                ..AutopilotParams::default()
            });
        let clock = SimulatedClock::new();

        clock.set(SimTime::from_seconds(0.001), StepIndex::new(1));
        autopilot
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .expect("warm-up tick");
        clock.set(SimTime::from_seconds(0.002), StepIndex::new(2));
        autopilot
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .expect("control tick");

        let (cmd, _) = bus
            .latest::<EngineDemand>()
            .unwrap()
            .expect("engine demand");
        assert_eq!(cmd.throttle_unit, 1.0);
    }

    #[test]
    fn differential_flatness_without_trajectory_fails_closed() {
        use openbmp_core::StepIndex;

        use crate::autopilot::{AutopilotParams, ThreeLoopAutopilot, TrajectoryKind};
        use crate::bus::Bus;
        use crate::clock::SimulatedClock;
        use crate::error::{AutopilotError, ControllerError};
        use crate::scheduler::{Job as _, JobContext};
        use crate::topics::{
            ActuatorCommand, AttitudeEstimate, EngineDemand, PositionEstimate, ReferenceState,
            VehicleStatus,
        };

        let bus = Bus::new();
        bus.register::<AttitudeEstimate>().unwrap();
        bus.register::<PositionEstimate>().unwrap();
        bus.register::<ReferenceState>().unwrap();
        bus.register::<VehicleStatus>().unwrap();
        bus.register::<ActuatorCommand>().unwrap();
        bus.register::<EngineDemand>().unwrap();
        // Publish a recent attitude + armed status so the autopilot
        // reaches the trajectory-loop branch.
        bus.publish(AttitudeEstimate {
            time: SimTime::from_seconds(0.001),
            q_body_to_eci_xyzw: [0.0, 0.0, 0.0, 1.0],
            omega_body_rad_s: Vector3::zeros(),
            gyro_bias_body_rad_s: Vector3::zeros(),
        })
        .unwrap();
        bus.publish(VehicleStatus {
            armed: true,
            in_flight: true,
            phase_id: 0,
            safe_state_requested: false,
        })
        .unwrap();
        bus.publish(ReferenceState {
            q_body_to_eci_xyzw: [0.0, 0.0, 0.0, 1.0],
            ..ReferenceState::default()
        })
        .unwrap();

        let mut autopilot = ThreeLoopAutopilot::new().with_params(AutopilotParams {
            trajectory_loop_enabled: true,
            trajectory_kind: TrajectoryKind::DifferentialFlatness,
            ..AutopilotParams::default()
        });

        let clock = SimulatedClock::new();
        // Warm-up tick — the autopilot records `last_predict_time_s`
        // and short-circuits on dt = 0 the first time it runs. The
        // second tick has dt > 0 and reaches the trajectory branch.
        clock.set(SimTime::from_seconds(0.001), StepIndex::new(1));
        autopilot
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .expect("warm-up tick");
        clock.set(SimTime::from_seconds(0.002), StepIndex::new(2));
        let err = autopilot
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap_err();

        match err {
            ControllerError::Autopilot(AutopilotError::Trajectory { reason }) => {
                assert!(
                    reason.contains("DifferentialFlatness"),
                    "unexpected trajectory-error reason: {reason}"
                );
                assert!(
                    reason.contains("MinimumSnapTrajectory"),
                    "expected hint at the installer: {reason}"
                );
            }
            other => panic!("expected AutopilotError::Trajectory, got {other:?}"),
        }
    }

    #[cfg(feature = "l1-adaptive")]
    #[test]
    fn l1_adaptive_rate_loop_augmentation_reaches_actuator_command() {
        use std::collections::BTreeMap;

        use openbmp_core::StepIndex;

        use crate::autopilot::{
            AutopilotParams, GainSchedule, PidGains, ThreeLoopAutopilot, ThreeLoopGains,
        };
        use crate::bus::Bus;
        use crate::clock::SimulatedClock;
        use crate::l1_adaptive_full::L1AdaptiveParams;
        use crate::scheduler::{Job as _, JobContext};
        use crate::topics::{
            ActuatorCommand, AttitudeEstimate, EngineDemand, PositionEstimate, ReferenceState,
            VehicleStatus,
        };

        let bus = Bus::new();
        bus.register::<AttitudeEstimate>().unwrap();
        bus.register::<PositionEstimate>().unwrap();
        bus.register::<ReferenceState>().unwrap();
        bus.register::<VehicleStatus>().unwrap();
        bus.register::<ActuatorCommand>().unwrap();
        bus.register::<EngineDemand>().unwrap();
        bus.publish(AttitudeEstimate {
            time: SimTime::from_seconds(0.001),
            q_body_to_eci_xyzw: [0.0, 0.0, 0.0, 1.0],
            omega_body_rad_s: Vector3::new(0.2, 0.0, 0.0),
            gyro_bias_body_rad_s: Vector3::zeros(),
        })
        .unwrap();
        bus.publish(VehicleStatus {
            armed: true,
            in_flight: true,
            phase_id: 0,
            safe_state_requested: false,
        })
        .unwrap();
        bus.publish(ReferenceState {
            q_body_to_eci_xyzw: [0.0, 0.0, 0.0, 1.0],
            ..ReferenceState::default()
        })
        .unwrap();

        let gains = ThreeLoopGains {
            rate: [PidGains::default(); 3],
            attitude: [PidGains::default(); 3],
            trajectory: [PidGains::default(); 3],
            elevator_limit_rad: 1.0,
            aileron_limit_rad: 1.0,
            rudder_limit_rad: 1.0,
            throttle_baseline: 0.0,
        };
        let schedule = GainSchedule {
            by_phase: BTreeMap::new(),
            default: gains,
        };
        let params = AutopilotParams {
            l1_adaptive: Some(L1AdaptiveParams {
                reference_model_a_m: -10.0,
                reference_model_b: 1.0,
                reference_model_k_g: 10.0,
                adaptation_sample_time_s: 0.001,
                low_pass_cutoff_rad_s: 5.0,
                lipschitz_bound: 0.1,
                projection_bound: 1.0,
            }),
            ..AutopilotParams::default()
        };
        let mut autopilot = ThreeLoopAutopilot::with_schedule(schedule).with_params(params);
        let clock = SimulatedClock::new();

        clock.set(SimTime::from_seconds(0.001), StepIndex::new(1));
        autopilot
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .expect("warm-up tick");
        clock.set(SimTime::from_seconds(0.002), StepIndex::new(2));
        autopilot
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .expect("L1 tick");

        let (cmd, _) = bus
            .latest::<ActuatorCommand>()
            .unwrap()
            .expect("actuator command");
        assert!(
            cmd.aileron_rad.abs() > 1.0e-6,
            "L1 augmentation should produce a nonzero roll command, got {cmd:?}"
        );
        assert_eq!(cmd.elevator_rad.to_bits(), 0.0_f64.to_bits());
        assert_eq!(cmd.rudder_rad.to_bits(), 0.0_f64.to_bits());
    }

    #[cfg(feature = "lqr")]
    #[test]
    fn lqr_anti_windup_reduces_saturated_command_on_next_step() {
        use crate::autopilot::{AutopilotParams, ThreeLoopAutopilot};
        use crate::lqr::LqrGains;

        let mut autopilot = ThreeLoopAutopilot::new().with_params(AutopilotParams {
            anti_windup: crate::anti_windup::AntiWindupKind::BackCalculation { gain: 1.0 },
            ..AutopilotParams::default()
        });
        let gains = LqrGains {
            k_omega: 0.0,
            k_int: 1.0,
        };
        autopilot.lqr_integrators[0] = -2.0;

        let raw_before = -gains.k_int * autopilot.lqr_integrators[0];
        assert!(raw_before > 1.0);
        let (cmd, saturated) = autopilot.lqr_step(0, 0.0, 0.0, &gains, 0.1, -1.0, 1.0, true);
        assert!(saturated);
        assert_eq!(cmd, 1.0);

        let raw_after = -gains.k_int * autopilot.lqr_integrators[0];
        assert!(
            raw_after < raw_before,
            "LQR anti-windup should move raw command toward the high clamp; \
             before={raw_before}, after={raw_after}"
        );
        assert!(
            raw_after >= 1.0,
            "single bleed step should not cross the high clamp in this fixture"
        );
    }
}
