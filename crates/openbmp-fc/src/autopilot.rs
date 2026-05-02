//! Three-loop autopilot.
//!
//! Phase 4.5 ships a Stevens & Lewis 2015 academic three-loop
//! formulation: an inner rate loop, an outer attitude loop, and an
//! optional trajectory loop. Gains are scheduled by mission phase via
//! the parameter registry.
//!
//! - Rate loop: `gyro_estimate -> torque_command -> actuator deflection`.
//! - Attitude loop: `attitude_estimate -> rate_command`.
//! - Trajectory loop: `position_estimate -> attitude_command`.
//!
//! Each loop is a PID with anti-windup. When the actuator demand
//! saturates, the integrator freezes (back-calculation). Phase 4.C
//! keeps this baseline after the observer-form anti-windup review:
//! no boundedness or determinism test in the current academic
//! envelope justifies replacing the simpler back-calculation path.

use std::collections::BTreeMap;

use nalgebra::Vector3;
use openbmp_physics::kinematics::quaternion_error_small_angle;

use crate::error::{AutopilotError, ControllerError};
use crate::filters::Biquad;
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::tables::Table;
use crate::topics::{
    ActuatorCommand, AttitudeEstimate, AutopilotStatus, EngineDemand, PositionEstimate,
    ReferenceState, VehicleStatus,
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
    /// kinds. Defaults to back-calculation with unit gain (Phase-4
    /// behaviour).
    pub anti_windup: crate::anti_windup::AntiWindupKind,
    /// Deadband in body angular velocity below which the rate loop
    /// integrator is frozen.
    pub rate_deadband_rad_s: f64,
    /// `true` to enable the trajectory loop. When `false`, the
    /// reference attitude is taken directly from the guidance topic.
    pub trajectory_loop_enabled: bool,
    /// Trajectory-loop strategy.
    pub trajectory_kind: TrajectoryKind,
    /// Optional per-axis gyro notch filters.
    pub gyro_notch: Option<[crate::filters::NotchConfig; 3]>,
    /// Optional Cao-Hovakimyan L1 adaptive augmentation on the rate
    /// loop (Phase 5.A.2.C). When `Some(_)` the autopilot installs a
    /// per-axis [`crate::l1_adaptive_full::L1AdaptiveChannel`] and
    /// adds its augmentation to the PID rate-loop output every tick.
    /// The configured `L1AdaptiveParams` must already satisfy the
    /// bandwidth-projection inequality `ω_c · L < 1` — the runner's
    /// `[fc.autopilot_params.l1_adaptive]` parser asserts this at
    /// scenario load.
    #[cfg(feature = "l1-adaptive")]
    pub l1_adaptive: Option<crate::l1_adaptive_full::L1AdaptiveParams>,
    /// Rate-loop dispatch strategy (Phase 5.A.3.B). Defaults to
    /// [`RateLoopKind::Pid`] for byte-stable Phase-4/5.A.2 behaviour;
    /// scenarios that select [`RateLoopKind::Lqr`] must also install
    /// `lqr_gains` (the runner solves DARE at scenario load).
    pub rate_loop_kind: RateLoopKind,
    /// Per-axis LQR feedback gains (Phase 5.A.3.B). Only consulted
    /// when `rate_loop_kind == RateLoopKind::Lqr`. The runner solves
    /// the per-axis DARE at scenario load using
    /// [`crate::lqr::solve_lqr_rate_loop`] and installs the result
    /// here; the autopilot fails closed at first tick if `Lqr` is
    /// selected without gains.
    #[cfg(feature = "lqr")]
    pub lqr_gains: Option<[crate::lqr::LqrGains; 3]>,
}

impl Default for AutopilotParams {
    fn default() -> Self {
        Self {
            anti_windup: crate::anti_windup::AntiWindupKind::default(),
            rate_deadband_rad_s: 1e-3,
            trajectory_loop_enabled: false,
            trajectory_kind: TrajectoryKind::Pid,
            gyro_notch: None,
            #[cfg(feature = "l1-adaptive")]
            l1_adaptive: None,
            rate_loop_kind: RateLoopKind::Pid,
            #[cfg(feature = "lqr")]
            lqr_gains: None,
        }
    }
}

/// Rate-loop dispatch strategy.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum RateLoopKind {
    /// Phase-4 PID rate loop (default). Reads gain schedule per
    /// phase; integrator handled by the shared `pid_step` helper.
    #[default]
    Pid,
    /// Phase-5.A.3.B per-axis LQR rate loop with augmented integral
    /// state. Requires the `lqr` Cargo feature and a populated
    /// [`AutopilotParams::lqr_gains`] field. The autopilot fails
    /// closed at first tick if either is missing.
    Lqr,
}

impl ParamSection for AutopilotParams {
    const NAME: &'static str = "autopilot.three_loop";
}

#[derive(Debug, Default)]
struct PidState {
    integral: f64,
    last_error: f64,
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
    schedule: GainSchedule,
    params: AutopilotParams,
    gyro_notch_state: Option<[Biquad; 3]>,
    /// Optional minimum-snap trajectory consumed when
    /// `params.trajectory_kind == TrajectoryKind::DifferentialFlatness`.
    /// The autopilot owns the trajectory in Phase 5.A.1.
    minimum_snap_trajectory: Option<MinimumSnapTrajectory>,
    /// Optional constant yaw override for the installed trajectory.
    /// Scenario `[fc.trajectory].yaw_rad` sets this; programmatic users
    /// that omit it inherit yaw from the bus reference.
    minimum_snap_yaw_rad: Option<f64>,
    /// Per-axis L1 adaptive state (Phase 5.A.2.C). Used only when
    /// [`AutopilotParams::l1_adaptive`] is `Some(_)` and the
    /// `l1-adaptive` feature is on.
    #[cfg(feature = "l1-adaptive")]
    l1_state: [crate::l1_adaptive_full::L1AdaptiveChannel; 3],
    /// Per-axis LQR integrator state (Phase 5.A.3.B). Updated by
    /// the rate loop only when
    /// `params.rate_loop_kind == RateLoopKind::Lqr`.
    #[cfg(feature = "lqr")]
    lqr_integrators: [f64; 3],
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
            schedule,
            params: AutopilotParams::default(),
            gyro_notch_state: None,
            minimum_snap_trajectory: None,
            minimum_snap_yaw_rad: None,
            #[cfg(feature = "l1-adaptive")]
            l1_state: [crate::l1_adaptive_full::L1AdaptiveChannel::new(); 3],
            #[cfg(feature = "lqr")]
            lqr_integrators: [0.0; 3],
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
        self
    }

    /// Phase 5.A.3.B per-axis LQR rate-loop step. Mirrors the
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
            self.params
                .anti_windup
                .apply(&mut self.lqr_integrators[axis], excess, dt);
        }
        (clamped, saturated)
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
        let gains = self.gains_for_phase(status.phase_id).clone();
        let mut saturated = false;

        // Attitude loop input: small-angle error in body frame.
        let mut attitude_error =
            quaternion_error_small_angle(attitude.q_body_to_eci_xyzw, reference.q_body_to_eci_xyzw);
        let mut differential_flatness_active = false;
        let mut differential_flatness_reference_suppressed = false;

        // Phase 5.A.1 — DifferentialFlatness trajectory loop.
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
                attitude_error =
                    quaternion_error_small_angle(attitude.q_body_to_eci_xyzw, [q.i, q.j, q.k, q.w]);
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

        let mut rate_cmd = Vector3::zeros();
        for i in 0..3 {
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

        // Rate loop: rate_cmd vs measured -> actuator deflection.
        // Per-axis integrator freeze when measured body rate is below
        // the configured deadband — prevents integrator wind-up at
        // very low rates where measurement noise dominates the signal.
        let omega_body_rad_s = self.filtered_omega_body(attitude.omega_body_rad_s, dt);
        let rate_error = rate_cmd - omega_body_rad_s;
        let mut torque = Vector3::zeros();
        for i in 0..3 {
            let limit = match i {
                0 => gains.aileron_limit_rad,
                1 => gains.elevator_limit_rad,
                _ => gains.rudder_limit_rad,
            };
            let integrate = omega_body_rad_s[i].abs() >= self.params.rate_deadband_rad_s;
            // Phase 5.A.3.B — rate-loop dispatch. PID is the
            // Phase-4/5.A.2 default; LQR uses the per-axis gains the
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
            };
            #[cfg(feature = "l1-adaptive")]
            let mut axis_cmd = cmd;
            #[cfg(not(feature = "l1-adaptive"))]
            let axis_cmd = cmd;
            // Phase 5.A.2.C — full Cao-Hovakimyan L1 adaptive
            // augmentation: the predictor sees the measured body
            // angular rate as plant state, the rate loop's command as
            // reference, and the PID output as baseline command.
            // The augmentation is added to the baseline; the result
            // is re-clamped to the per-axis actuator limit.
            #[cfg(feature = "l1-adaptive")]
            if let Some(l1_params) = self.params.l1_adaptive {
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

        let engine = EngineDemand {
            time: ctx.clock.now(),
            throttle_unit: gains.throttle_baseline,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
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
    use nalgebra::Vector3;
    use openbmp_core::SimTime;

    use super::reference_yaw_rad;

    #[test]
    fn reference_yaw_rad_recovers_z_rotation() {
        let theta: f64 = 0.4;
        let half = theta / 2.0;
        let q_xyzw = [0.0, 0.0, half.sin(), half.cos()];
        let yaw = reference_yaw_rad(q_xyzw);
        assert!((yaw - theta).abs() < 1.0e-12);
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
}
