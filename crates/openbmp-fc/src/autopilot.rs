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
//! saturates, the integrator freezes (back-calculation).

use std::collections::BTreeMap;

use nalgebra::Vector3;

use crate::error::ControllerError;
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::tables::Table;
use crate::topics::{
    ActuatorCommand, AttitudeEstimate, EngineDemand, PositionEstimate, ReferenceState,
    VehicleStatus,
};

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

/// Autopilot configuration parameters.
#[derive(Clone, Debug)]
pub struct AutopilotParams {
    /// Anti-windup back-calculation gain.
    pub anti_windup_gain: f64,
    /// Deadband in body angular velocity below which the rate loop
    /// integrator is frozen.
    pub rate_deadband_rad_s: f64,
    /// `true` to enable the trajectory loop. When `false`, the
    /// reference attitude is taken directly from the guidance topic.
    pub trajectory_loop_enabled: bool,
}

impl Default for AutopilotParams {
    fn default() -> Self {
        Self {
            anti_windup_gain: 1.0,
            rate_deadband_rad_s: 1e-3,
            trajectory_loop_enabled: false,
        }
    }
}

impl ParamSection for AutopilotParams {
    const NAME: &'static str = "autopilot.three_loop";
}

#[derive(Debug, Default)]
struct PidState {
    integral: f64,
    last_error: f64,
}

/// Three-loop autopilot job.
#[derive(Debug)]
pub struct ThreeLoopAutopilot {
    name: &'static str,
    rate_state: [PidState; 3],
    attitude_state: [PidState; 3],
    trajectory_state: [PidState; 3],
    last_predict_time_s: f64,
    last_attitude_seq: u64,
    schedule: GainSchedule,
    params: AutopilotParams,
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
            last_attitude_seq: 0,
            schedule,
            params: AutopilotParams::default(),
        }
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
        self
    }

    fn gains_for_phase(&self, phase: u64) -> &ThreeLoopGains {
        self.schedule
            .by_phase
            .get(&phase)
            .unwrap_or(&self.schedule.default)
    }
}

impl Default for ThreeLoopAutopilot {
    fn default() -> Self {
        Self::new()
    }
}

fn pid_step(
    state: &mut PidState,
    gains: &PidGains,
    error: f64,
    dt: f64,
    saturate_against_min: f64,
    saturate_against_max: f64,
    anti_windup: f64,
) -> (f64, bool) {
    state.integral += error * dt;
    let derivative = if dt > 0.0 {
        (error - state.last_error) / dt
    } else {
        0.0
    };
    state.last_error = error;
    let raw = gains.kp * error + gains.ki * state.integral + gains.kd * derivative;
    let clamped = raw.clamp(saturate_against_min, saturate_against_max);
    let saturated = (raw - clamped).abs() > 0.0;
    if saturated {
        // Back-calculate anti-windup.
        let excess = raw - clamped;
        state.integral -= anti_windup * excess * dt;
    }
    (clamped, saturated)
}

fn quaternion_error_axis(q_estimate_xyzw: [f64; 4], q_reference_xyzw: [f64; 4]) -> Vector3<f64> {
    // Compute the rotation that takes the estimate into the reference,
    // expressed as a small-angle vector in body axes. This is the
    // standard small-angle linearisation used in attitude PID
    // controllers.
    let q_est = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q_estimate_xyzw[3],
        q_estimate_xyzw[0],
        q_estimate_xyzw[1],
        q_estimate_xyzw[2],
    ));
    let q_ref = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q_reference_xyzw[3],
        q_reference_xyzw[0],
        q_reference_xyzw[1],
        q_reference_xyzw[2],
    ));
    let q_err = q_est.inverse() * q_ref;
    // Small-angle: error_axis = 2 * (qx, qy, qz) * sign(qw)
    let qx = q_err.i;
    let qy = q_err.j;
    let qz = q_err.k;
    let qw = q_err.w;
    let sign = if qw >= 0.0 { 1.0 } else { -1.0 };
    Vector3::new(2.0 * qx * sign, 2.0 * qy * sign, 2.0 * qz * sign)
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

        let attitude = match ctx.bus.latest::<AttitudeEstimate>()? {
            Some((a, seq)) => {
                self.last_attitude_seq = seq.value();
                a
            }
            None => return Ok(()),
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

        // Attitude loop input: small-angle error in body frame.
        let mut attitude_error =
            quaternion_error_axis(attitude.q_body_to_eci_xyzw, reference.q_body_to_eci_xyzw);

        // Trajectory loop: feeds an attitude-error correction. Active
        // only when enabled by params and a position reference is
        // present.
        if self.params.trajectory_loop_enabled
            && reference.position_eci_m.norm() > 0.0
            && let Some(pos) = position
        {
            let pos_error_eci = reference.position_eci_m - pos.position_eci_m;
            let q_est = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                attitude.q_body_to_eci_xyzw[3],
                attitude.q_body_to_eci_xyzw[0],
                attitude.q_body_to_eci_xyzw[1],
                attitude.q_body_to_eci_xyzw[2],
            ));
            let r_eci_to_body = q_est.to_rotation_matrix().transpose();
            let pos_error_body = r_eci_to_body * pos_error_eci;
            for i in 0..3 {
                let (cmd, _) = pid_step(
                    &mut self.trajectory_state[i],
                    &gains.trajectory[i],
                    pos_error_body[i],
                    dt,
                    -1.0,
                    1.0,
                    self.params.anti_windup_gain,
                );
                attitude_error[i] += cmd;
            }
        }

        let mut rate_cmd = Vector3::zeros();
        for i in 0..3 {
            let (cmd, _) = pid_step(
                &mut self.attitude_state[i],
                &gains.attitude[i],
                attitude_error[i],
                dt,
                -1e3,
                1e3,
                self.params.anti_windup_gain,
            );
            rate_cmd[i] = cmd;
        }

        // Rate loop: rate_cmd vs measured -> actuator deflection.
        let rate_error = rate_cmd - attitude.omega_body_rad_s;
        let mut torque = Vector3::zeros();
        let mut saturated = false;
        for i in 0..3 {
            let limit = match i {
                0 => gains.aileron_limit_rad,
                1 => gains.elevator_limit_rad,
                _ => gains.rudder_limit_rad,
            };
            let (cmd, sat) = pid_step(
                &mut self.rate_state[i],
                &gains.rate[i],
                rate_error[i],
                dt,
                -limit,
                limit,
                self.params.anti_windup_gain,
            );
            torque[i] = cmd;
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
