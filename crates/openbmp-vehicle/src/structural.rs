//! First lateral structural **bending mode** of a slender launch vehicle.
//!
//! A real launch vehicle is not rigid: its lowest body-bending mode couples
//! to the flight controller through the rate-gyro pickup — the gyro at its
//! mounting station senses the *local bending slope rate* on top of the
//! rigid-body rate. An over-reactive rate loop can drive the mode unstable,
//! which is why launch-vehicle autopilots gain-stabilise it with a notch
//! filter at the bending frequency (see `AutopilotParams.gyro_notch`).
//!
//! # Model
//!
//! Each transverse axis carries one generalised modal coordinate `q`
//! (units: rad of local slope, so the gyro pickup is dimensionally a rate):
//!
//! ```text
//! q̈ + 2ζ_b ω_b q̇ + ω_b² q = (slope_engine / m_q) · a_lateral
//! ```
//!
//! where `a_lateral` is the body-frame lateral specific force at the engine
//! station (which the thrust gimbal drives), `ω_b` is the modal frequency,
//! `ζ_b` the modal damping, `m_q` the modal mass, and `slope_engine` the mode
//! shape's slope at the engine station (the forcing lever).
//!
//! Two independent coordinates are carried: `q_x` (bending in the body x–z
//! plane, forced by `a_x`) and `q_y` (y–z plane, forced by `a_y`). The
//! decoupled single-mode treatment ignores higher modes and cross coupling.
//!
//! # Sensor pickup
//!
//! The gyro senses an extra body-rate `slope_gyro · q̇` about the axis
//! perpendicular to the bending plane: bending in x (`q_x`) tilts the gyro
//! about body **+y** (pitch), bending in y (`q_y`) about body **+x**. So the
//! pickup vector is `[slope_gyro·q̇_y, slope_gyro·q̇_x, 0]`.
//!
//! # Reaction
//!
//! The modal acceleration reacts a small moment back on the rigid body about
//! the same axes: `reaction = -m_q · slope_engine · q̈` (per axis), mapped to
//! body pitch/yaw. (First-increment: a lever-scaled reaction; the full
//! distributed-load coupling is a follow-up — see
//! `docs/launch-vehicle-fidelity-frontier.md`.)
//!
//! # Determinism
//!
//! Pure `f64`, no FMA, single symplectic-Euler sub-step per call, locked
//! operand order — identical to the slosh integrator's contract.

use nalgebra::Vector3;
use openbmp_core::Duration;

/// Construction / step error for [`BendingMode`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BendingError {
    /// A parameter was non-finite or out of range.
    InvalidParameter(&'static str),
    /// A step input was non-finite.
    NonFiniteStepInput(&'static str),
}

impl core::fmt::Display for BendingError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidParameter(r) => write!(f, "bending mode invalid parameter: {r}"),
            Self::NonFiniteStepInput(r) => write!(f, "bending mode non-finite step input: {r}"),
        }
    }
}

impl std::error::Error for BendingError {}

/// First lateral structural bending mode (one generalised coordinate per
/// transverse axis).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BendingMode {
    omega_b_rad_s: f64,
    zeta_b: f64,
    modal_mass_kg: f64,
    slope_at_engine: f64,
    slope_at_gyro: f64,

    q_x: f64,
    q_dot_x: f64,
    q_y: f64,
    q_dot_y: f64,

    last_q_ddot_x: f64,
    last_q_ddot_y: f64,
}

impl BendingMode {
    /// Construct a bending mode. `omega_b_rad_s`, `zeta_b`, `modal_mass_kg`
    /// must be finite with `ω_b > 0`, `ζ_b ≥ 0`, `m_q > 0`; the two mode-shape
    /// slopes must be finite. The mode starts at rest (`q = q̇ = 0`).
    ///
    /// # Errors
    ///
    /// Returns [`BendingError::InvalidParameter`] for any violation.
    pub fn new(
        omega_b_rad_s: f64,
        zeta_b: f64,
        modal_mass_kg: f64,
        slope_at_engine: f64,
        slope_at_gyro: f64,
    ) -> Result<Self, BendingError> {
        if !omega_b_rad_s.is_finite() || omega_b_rad_s <= 0.0 {
            return Err(BendingError::InvalidParameter(
                "omega_b_rad_s must be finite and > 0",
            ));
        }
        if !zeta_b.is_finite() || zeta_b < 0.0 {
            return Err(BendingError::InvalidParameter(
                "zeta_b must be finite and >= 0",
            ));
        }
        if !modal_mass_kg.is_finite() || modal_mass_kg <= 0.0 {
            return Err(BendingError::InvalidParameter(
                "modal_mass_kg must be finite and > 0",
            ));
        }
        if !slope_at_engine.is_finite() || !slope_at_gyro.is_finite() {
            return Err(BendingError::InvalidParameter(
                "mode-shape slopes must be finite",
            ));
        }
        Ok(Self {
            omega_b_rad_s,
            zeta_b,
            modal_mass_kg,
            slope_at_engine,
            slope_at_gyro,
            q_x: 0.0,
            q_dot_x: 0.0,
            q_y: 0.0,
            q_dot_y: 0.0,
            last_q_ddot_x: 0.0,
            last_q_ddot_y: 0.0,
        })
    }

    /// Advance the modal state one sub-step under the body-frame lateral
    /// specific force `accel_body_m_s2` (only x and y force the transverse
    /// modes). Symplectic Euler, locked operand order, single sub-step.
    ///
    /// # Errors
    ///
    /// Returns [`BendingError::NonFiniteStepInput`] for non-finite inputs.
    pub fn step(
        &mut self,
        accel_body_m_s2: Vector3<f64>,
        dt: Duration,
    ) -> Result<(), BendingError> {
        if !accel_body_m_s2.iter().all(|c| c.is_finite()) {
            return Err(BendingError::NonFiniteStepInput("accel_body_m_s2"));
        }
        let dt_s = dt.as_seconds();
        if !dt_s.is_finite() || dt_s < 0.0 {
            return Err(BendingError::NonFiniteStepInput("dt"));
        }
        let two_zeta_omega = 2.0 * self.zeta_b * self.omega_b_rad_s;
        let omega_sq = self.omega_b_rad_s * self.omega_b_rad_s;
        let forcing_gain = self.slope_at_engine / self.modal_mass_kg;

        // x axis (forced by a_x): q̈ = forcing - damping - restoring.
        let forcing_x = forcing_gain * accel_body_m_s2.x;
        let damping_x = two_zeta_omega * self.q_dot_x;
        let restoring_x = omega_sq * self.q_x;
        let q_ddot_x = forcing_x - damping_x - restoring_x;
        self.last_q_ddot_x = q_ddot_x;
        self.q_dot_x += dt_s * q_ddot_x;
        self.q_x += dt_s * self.q_dot_x;

        // y axis (declared after x).
        let forcing_y = forcing_gain * accel_body_m_s2.y;
        let damping_y = two_zeta_omega * self.q_dot_y;
        let restoring_y = omega_sq * self.q_y;
        let q_ddot_y = forcing_y - damping_y - restoring_y;
        self.last_q_ddot_y = q_ddot_y;
        self.q_dot_y += dt_s * q_ddot_y;
        self.q_y += dt_s * self.q_dot_y;
        Ok(())
    }

    /// Body-rate pickup the gyro senses from the bending slope rate, added to
    /// the rigid-body rate. Bending in x → pitch (+y) pickup; bending in y →
    /// roll/transverse (+x) pickup. `[slope·q̇_y, slope·q̇_x, 0]`.
    #[must_use]
    pub fn gyro_pickup_rad_s(&self) -> Vector3<f64> {
        Vector3::new(
            self.slope_at_gyro * self.q_dot_y,
            self.slope_at_gyro * self.q_dot_x,
            0.0,
        )
    }

    /// Reaction moment on the rigid body from the modal acceleration
    /// (lever-scaled first-increment coupling), about body pitch/yaw.
    #[must_use]
    pub fn reaction_moment_body_n_m(&self) -> Vector3<f64> {
        let k = -self.modal_mass_kg * self.slope_at_engine;
        Vector3::new(k * self.last_q_ddot_y, k * self.last_q_ddot_x, 0.0)
    }

    /// Current modal coordinates `(q_x, q_y)` (rad of local slope).
    #[must_use]
    pub fn modal_coordinates(&self) -> (f64, f64) {
        (self.q_x, self.q_y)
    }

    /// Current modal rates `(q̇_x, q̇_y)`.
    #[must_use]
    pub fn modal_rates(&self) -> (f64, f64) {
        (self.q_dot_x, self.q_dot_y)
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    fn mode() -> BendingMode {
        // ω_b ≈ 6.28 rad/s (1 Hz), light damping, unit slopes.
        BendingMode::new(6.283_185_307_2, 0.01, 500.0, 1.0, 1.0).unwrap()
    }

    #[test]
    fn rejects_bad_parameters() {
        assert!(BendingMode::new(0.0, 0.01, 500.0, 1.0, 1.0).is_err());
        assert!(BendingMode::new(6.0, -0.1, 500.0, 1.0, 1.0).is_err());
        assert!(BendingMode::new(6.0, 0.01, 0.0, 1.0, 1.0).is_err());
        assert!(BendingMode::new(6.0, 0.01, 500.0, f64::NAN, 1.0).is_err());
    }

    #[test]
    fn lateral_acceleration_excites_the_mode_and_gyro_picks_it_up() {
        let mut m = mode();
        let dt = Duration::from_seconds(0.001);
        // A steady lateral push excites q_x; the gyro pickup (pitch, +y) is
        // non-zero once the mode has velocity.
        for _ in 0..200 {
            m.step(Vector3::new(5.0, 0.0, 0.0), dt).unwrap();
        }
        let (qx, _qy) = m.modal_coordinates();
        assert!(qx.abs() > 0.0, "lateral accel must excite q_x");
        let pickup = m.gyro_pickup_rad_s();
        assert!(
            pickup.y.abs() > 0.0,
            "gyro must pick up the bending slope rate on pitch"
        );
        assert!(
            pickup.x.abs() < 1.0e-15,
            "no y-plane bending => no x pickup"
        );
        // Reaction moment opposes the modal acceleration about pitch.
        let reaction = m.reaction_moment_body_n_m();
        assert!(reaction.iter().all(|c| c.is_finite()));
    }

    #[test]
    fn at_rest_no_pickup_no_reaction() {
        let m = mode();
        assert_eq!(m.gyro_pickup_rad_s(), Vector3::zeros());
        assert_eq!(m.reaction_moment_body_n_m(), Vector3::zeros());
    }

    #[test]
    fn free_response_decays_with_damping() {
        let mut m = BendingMode::new(6.283_185_307_2, 0.05, 500.0, 1.0, 1.0).unwrap();
        // Excite, then let it ring down with no forcing.
        for _ in 0..100 {
            m.step(Vector3::new(10.0, 0.0, 0.0), Duration::from_seconds(0.001))
                .unwrap();
        }
        let peak = m.modal_coordinates().0.abs();
        for _ in 0..20_000 {
            m.step(Vector3::zeros(), Duration::from_seconds(0.001))
                .unwrap();
        }
        let after = m.modal_coordinates().0.abs();
        assert!(
            after < 0.1 * peak,
            "damped free response must decay: {after} vs {peak}"
        );
    }
}
