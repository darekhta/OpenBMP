//! Lateral structural **bending modes** of a slender launch vehicle.
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
//! Accelerometers can also sense the local modal acceleration at their
//! mounting station. The multi-mode block reports this as
//! `Σ shape_at_accelerometer·q̈_j` in the bending plane axes.
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
    /// A vector did not match the modal coordinate count.
    DimensionMismatch(&'static str),
    /// A step input was non-finite.
    NonFiniteStepInput(&'static str),
}

impl core::fmt::Display for BendingError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidParameter(r) => write!(f, "bending mode invalid parameter: {r}"),
            Self::DimensionMismatch(r) => write!(f, "bending mode dimension mismatch: {r}"),
            Self::NonFiniteStepInput(r) => write!(f, "bending mode non-finite step input: {r}"),
        }
    }
}

impl std::error::Error for BendingError {}

/// One declared lateral structural mode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModeParams {
    /// Modal natural frequency, rad/s.
    pub omega_rad_s: f64,
    /// Modal damping ratio, non-negative.
    pub zeta: f64,
    /// Generalised modal mass, kg.
    pub modal_mass_kg: f64,
    /// Mode-shape slope at the engine station.
    pub slope_at_engine: f64,
    /// Mode-shape slope at the rate-gyro station.
    pub slope_at_gyro: f64,
    /// Mode-shape value at the accelerometer station.
    pub shape_at_accelerometer: f64,
}

impl ModeParams {
    /// Construct and validate one lateral mode declaration.
    ///
    /// # Errors
    ///
    /// Returns [`BendingError::InvalidParameter`] for non-finite values,
    /// non-positive frequency / mass, or negative damping.
    pub fn new(
        omega_rad_s: f64,
        zeta: f64,
        modal_mass_kg: f64,
        slope_at_engine: f64,
        slope_at_gyro: f64,
        shape_at_accelerometer: f64,
    ) -> Result<Self, BendingError> {
        if !omega_rad_s.is_finite() || omega_rad_s <= 0.0 {
            return Err(BendingError::InvalidParameter(
                "omega_rad_s must be finite and > 0",
            ));
        }
        if !zeta.is_finite() || zeta < 0.0 {
            return Err(BendingError::InvalidParameter(
                "zeta must be finite and >= 0",
            ));
        }
        if !modal_mass_kg.is_finite() || modal_mass_kg <= 0.0 {
            return Err(BendingError::InvalidParameter(
                "modal_mass_kg must be finite and > 0",
            ));
        }
        if !slope_at_engine.is_finite()
            || !slope_at_gyro.is_finite()
            || !shape_at_accelerometer.is_finite()
        {
            return Err(BendingError::InvalidParameter(
                "mode-shape values must be finite",
            ));
        }
        Ok(Self {
            omega_rad_s,
            zeta,
            modal_mass_kg,
            slope_at_engine,
            slope_at_gyro,
            shape_at_accelerometer,
        })
    }
}

/// Current coordinates for one lateral structural mode.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ModalState {
    /// Modal coordinate for bending in the body x-z plane.
    pub q_x: f64,
    /// Modal coordinate rate for bending in the body x-z plane.
    pub q_dot_x: f64,
    /// Modal acceleration for bending in the body x-z plane from the last step.
    pub q_ddot_x: f64,
    /// Modal coordinate for bending in the body y-z plane.
    pub q_y: f64,
    /// Modal coordinate rate for bending in the body y-z plane.
    pub q_dot_y: f64,
    /// Modal acceleration for bending in the body y-z plane from the last step.
    pub q_ddot_y: f64,
}

/// Declaration-ordered block of independent lateral structural modes.
#[derive(Clone, Debug, PartialEq)]
pub struct ModalBlock {
    modes: Vec<ModeParams>,
    states: Vec<ModalState>,
}

impl ModalBlock {
    /// Construct a modal block from declaration-ordered mode parameters.
    ///
    /// # Errors
    ///
    /// Returns [`BendingError::InvalidParameter`] when no modes are declared
    /// or any parameter is non-finite / out of range.
    pub fn new(modes: Vec<ModeParams>) -> Result<Self, BendingError> {
        if modes.is_empty() {
            return Err(BendingError::InvalidParameter(
                "modes must contain at least one mode",
            ));
        }
        for mode in &modes {
            ModeParams::new(
                mode.omega_rad_s,
                mode.zeta,
                mode.modal_mass_kg,
                mode.slope_at_engine,
                mode.slope_at_gyro,
                mode.shape_at_accelerometer,
            )?;
        }
        let states = vec![ModalState::default(); modes.len()];
        Ok(Self { modes, states })
    }

    /// Build the byte-compatible one-mode block represented by legacy
    /// `[vehicle.bending]`. Accelerometer pickup is zero by default so old
    /// bending scenarios keep their sensor truth unchanged.
    ///
    /// # Errors
    ///
    /// Returns [`BendingError::InvalidParameter`] for invalid parameters.
    pub fn from_legacy_bending(
        omega_b_rad_s: f64,
        zeta_b: f64,
        modal_mass_kg: f64,
        slope_at_engine: f64,
        slope_at_gyro: f64,
    ) -> Result<Self, BendingError> {
        Self::new(vec![ModeParams::new(
            omega_b_rad_s,
            zeta_b,
            modal_mass_kg,
            slope_at_engine,
            slope_at_gyro,
            0.0,
        )?])
    }

    /// Number of declared modes.
    #[must_use]
    pub fn len(&self) -> usize {
        self.modes.len()
    }

    /// Whether the block contains no modes. A constructed block is never empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.modes.is_empty()
    }

    /// Reset every modal coordinate to rest.
    pub fn reset(&mut self) {
        self.states.fill(ModalState::default());
    }

    /// Advance every mode one sub-step under the body-frame lateral specific
    /// force `accel_body_m_s2`, preserving declaration order.
    ///
    /// # Errors
    ///
    /// Returns [`BendingError::NonFiniteStepInput`] for non-finite inputs.
    pub fn step(
        &mut self,
        accel_body_m_s2: Vector3<f64>,
        dt: Duration,
    ) -> Result<(), BendingError> {
        self.step_with_modal_force(accel_body_m_s2, &[], dt)
    }

    /// Advance every mode one sub-step under body-frame lateral specific force
    /// plus an optional declaration-ordered generalized modal force vector.
    ///
    /// `modal_force_n` must either be empty or contain one generalized force
    /// per locked modal coordinate:
    /// `[mode0.q_x, mode0.q_y, mode1.q_x, mode1.q_y, ...]`.
    ///
    /// # Errors
    ///
    /// Returns [`BendingError::DimensionMismatch`] when `modal_force_n` has the
    /// wrong length, or [`BendingError::NonFiniteStepInput`] for non-finite
    /// inputs.
    pub fn step_with_modal_force(
        &mut self,
        accel_body_m_s2: Vector3<f64>,
        modal_force_n: &[f64],
        dt: Duration,
    ) -> Result<(), BendingError> {
        self.step_with_modal_force_and_stiffness_scale(accel_body_m_s2, modal_force_n, &[], dt)
    }

    /// Advance every mode one sub-step under body-frame lateral specific force,
    /// optional generalized modal force, and optional declaration-ordered modal
    /// stiffness multipliers.
    ///
    /// `modal_stiffness_scale_1` must either be empty or contain one positive
    /// finite multiplier per locked modal coordinate:
    /// `[mode0.q_x, mode0.q_y, mode1.q_x, mode1.q_y, ...]`.
    ///
    /// # Errors
    ///
    /// Returns [`BendingError::DimensionMismatch`] when either modal vector has
    /// the wrong length, or [`BendingError::NonFiniteStepInput`] for non-finite
    /// inputs or non-positive stiffness scales.
    pub fn step_with_modal_force_and_stiffness_scale(
        &mut self,
        accel_body_m_s2: Vector3<f64>,
        modal_force_n: &[f64],
        modal_stiffness_scale_1: &[f64],
        dt: Duration,
    ) -> Result<(), BendingError> {
        if !accel_body_m_s2.iter().all(|c| c.is_finite()) {
            return Err(BendingError::NonFiniteStepInput("accel_body_m_s2"));
        }
        let coordinate_count = self.states.len() * 2;
        if !modal_force_n.is_empty() && modal_force_n.len() != coordinate_count {
            return Err(BendingError::DimensionMismatch(
                "modal_force_n length must be zero or match modal coordinate count",
            ));
        }
        if !modal_force_n.iter().all(|c| c.is_finite()) {
            return Err(BendingError::NonFiniteStepInput("modal_force_n"));
        }
        if !modal_stiffness_scale_1.is_empty() && modal_stiffness_scale_1.len() != coordinate_count
        {
            return Err(BendingError::DimensionMismatch(
                "modal_stiffness_scale_1 length must be zero or match modal coordinate count",
            ));
        }
        if !modal_stiffness_scale_1
            .iter()
            .all(|scale| scale.is_finite() && *scale > 0.0)
        {
            return Err(BendingError::NonFiniteStepInput("modal_stiffness_scale_1"));
        }
        let dt_s = dt.as_seconds();
        if !dt_s.is_finite() || dt_s < 0.0 {
            return Err(BendingError::NonFiniteStepInput("dt"));
        }
        for (mode_index, (mode, state)) in self.modes.iter().zip(self.states.iter_mut()).enumerate()
        {
            let two_zeta_omega = 2.0 * mode.zeta * mode.omega_rad_s;
            let omega_sq = mode.omega_rad_s * mode.omega_rad_s;
            let forcing_gain = mode.slope_at_engine / mode.modal_mass_kg;
            let modal_force_offset = mode_index * 2;
            let external_force_x = modal_force_n
                .get(modal_force_offset)
                .copied()
                .unwrap_or(0.0);
            let external_force_y = modal_force_n
                .get(modal_force_offset + 1)
                .copied()
                .unwrap_or(0.0);
            let stiffness_scale_x = modal_stiffness_scale_1
                .get(modal_force_offset)
                .copied()
                .unwrap_or(1.0);
            let stiffness_scale_y = modal_stiffness_scale_1
                .get(modal_force_offset + 1)
                .copied()
                .unwrap_or(1.0);

            state.q_ddot_x = advance_axis_with_forcing(
                &mut state.q_x,
                &mut state.q_dot_x,
                (forcing_gain * accel_body_m_s2.x) + (external_force_x / mode.modal_mass_kg),
                two_zeta_omega * stiffness_scale_x.sqrt(),
                omega_sq * stiffness_scale_x,
                dt_s,
            );
            state.q_ddot_y = advance_axis_with_forcing(
                &mut state.q_y,
                &mut state.q_dot_y,
                (forcing_gain * accel_body_m_s2.y) + (external_force_y / mode.modal_mass_kg),
                two_zeta_omega * stiffness_scale_y.sqrt(),
                omega_sq * stiffness_scale_y,
                dt_s,
            );
        }
        Ok(())
    }

    /// Body-rate pickup the gyro senses from all modal slope rates.
    #[must_use]
    pub fn gyro_pickup_rad_s(&self) -> Vector3<f64> {
        self.modes
            .iter()
            .zip(&self.states)
            .fold(Vector3::zeros(), |sum, (mode, state)| {
                sum + Vector3::new(
                    mode.slope_at_gyro * state.q_dot_y,
                    mode.slope_at_gyro * state.q_dot_x,
                    0.0,
                )
            })
    }

    /// Specific-force pickup from modal acceleration at the accelerometer
    /// station. Zero for legacy bending unless an explicit mode declares a
    /// non-zero accelerometer shape value.
    #[must_use]
    pub fn accel_pickup_m_s2(&self) -> Vector3<f64> {
        self.modes
            .iter()
            .zip(&self.states)
            .fold(Vector3::zeros(), |sum, (mode, state)| {
                sum + Vector3::new(
                    mode.shape_at_accelerometer * state.q_ddot_x,
                    mode.shape_at_accelerometer * state.q_ddot_y,
                    0.0,
                )
            })
    }

    /// Reaction moment on the rigid body from all modal accelerations.
    #[must_use]
    pub fn reaction_moment_body_n_m(&self) -> Vector3<f64> {
        self.modes
            .iter()
            .zip(&self.states)
            .fold(Vector3::zeros(), |sum, (mode, state)| {
                let k = -mode.modal_mass_kg * mode.slope_at_engine;
                sum + Vector3::new(k * state.q_ddot_y, k * state.q_ddot_x, 0.0)
            })
    }

    /// Declaration-ordered mode parameters.
    #[must_use]
    pub fn modes(&self) -> &[ModeParams] {
        &self.modes
    }

    /// Declaration-ordered modal states.
    #[must_use]
    pub fn modal_states(&self) -> &[ModalState] {
        &self.states
    }

    /// Replace declaration-ordered modal states after an external integrator
    /// advances the same mode block.
    ///
    /// # Errors
    ///
    /// Returns [`BendingError::InvalidParameter`] when the state count differs
    /// from the mode count or any state component is non-finite.
    pub fn set_modal_states(&mut self, states: &[ModalState]) -> Result<(), BendingError> {
        if states.len() != self.states.len() {
            return Err(BendingError::InvalidParameter(
                "modal state count must match mode count",
            ));
        }
        if states.iter().any(|state| {
            !state.q_x.is_finite()
                || !state.q_dot_x.is_finite()
                || !state.q_ddot_x.is_finite()
                || !state.q_y.is_finite()
                || !state.q_dot_y.is_finite()
                || !state.q_ddot_y.is_finite()
        }) {
            return Err(BendingError::InvalidParameter(
                "modal state values must be finite",
            ));
        }
        self.states.copy_from_slice(states);
        Ok(())
    }
}

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
        ModeParams::new(
            omega_b_rad_s,
            zeta_b,
            modal_mass_kg,
            slope_at_engine,
            slope_at_gyro,
            0.0,
        )?;
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
        self.last_q_ddot_x = advance_axis(
            &mut self.q_x,
            &mut self.q_dot_x,
            accel_body_m_s2.x,
            forcing_gain,
            two_zeta_omega,
            omega_sq,
            dt_s,
        );

        // y axis (declared after x).
        self.last_q_ddot_y = advance_axis(
            &mut self.q_y,
            &mut self.q_dot_y,
            accel_body_m_s2.y,
            forcing_gain,
            two_zeta_omega,
            omega_sq,
            dt_s,
        );
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

fn advance_axis(
    q: &mut f64,
    q_dot: &mut f64,
    accel: f64,
    forcing_gain: f64,
    two_zeta_omega: f64,
    omega_sq: f64,
    dt_s: f64,
) -> f64 {
    let forcing = forcing_gain * accel;
    let damping = two_zeta_omega * *q_dot;
    let restoring = omega_sq * *q;
    let q_ddot = forcing - damping - restoring;
    *q_dot += dt_s * q_ddot;
    *q += dt_s * *q_dot;
    q_ddot
}

fn advance_axis_with_forcing(
    q: &mut f64,
    q_dot: &mut f64,
    forcing: f64,
    two_zeta_omega: f64,
    omega_sq: f64,
    dt_s: f64,
) -> f64 {
    let damping = two_zeta_omega * *q_dot;
    let restoring = omega_sq * *q;
    let q_ddot = forcing - damping - restoring;
    *q_dot += dt_s * q_ddot;
    *q += dt_s * *q_dot;
    q_ddot
}

#[cfg(test)]
#[allow(clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use super::*;

    fn mode() -> BendingMode {
        // ω_b ≈ 6.28 rad/s (1 Hz), light damping, unit slopes.
        BendingMode::new(std::f64::consts::TAU, 0.01, 500.0, 1.0, 1.0).unwrap()
    }

    #[test]
    fn rejects_bad_parameters() {
        assert!(BendingMode::new(0.0, 0.01, 500.0, 1.0, 1.0).is_err());
        assert!(BendingMode::new(6.0, -0.1, 500.0, 1.0, 1.0).is_err());
        assert!(BendingMode::new(6.0, 0.01, 0.0, 1.0, 1.0).is_err());
        assert!(BendingMode::new(6.0, 0.01, 500.0, f64::NAN, 1.0).is_err());
        assert!(ModalBlock::new(Vec::new()).is_err());
        assert!(ModeParams::new(6.0, 0.01, 500.0, 1.0, 1.0, f64::NAN).is_err());
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
        let mut m = BendingMode::new(std::f64::consts::TAU, 0.05, 500.0, 1.0, 1.0).unwrap();
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

    #[test]
    fn modal_block_single_mode_matches_legacy_bending() {
        let mut legacy = BendingMode::new(8.0, 0.03, 450.0, 1.2, -0.8).unwrap();
        let mut block = ModalBlock::from_legacy_bending(8.0, 0.03, 450.0, 1.2, -0.8).unwrap();
        let dt = Duration::from_seconds(0.002);
        let inputs = [
            Vector3::new(2.0, -1.0, 0.3),
            Vector3::new(-0.5, 3.0, -0.2),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(7.0, -2.0, 1.0),
        ];

        for accel in inputs.into_iter().cycle().take(200) {
            legacy.step(accel, dt).unwrap();
            block.step(accel, dt).unwrap();
            let state = block.modal_states()[0];
            assert_eq!(legacy.modal_coordinates(), (state.q_x, state.q_y));
            assert_eq!(legacy.modal_rates(), (state.q_dot_x, state.q_dot_y));
            assert_eq!(legacy.gyro_pickup_rad_s(), block.gyro_pickup_rad_s());
            assert_eq!(
                legacy.reaction_moment_body_n_m(),
                block.reaction_moment_body_n_m()
            );
            assert_eq!(block.accel_pickup_m_s2(), Vector3::zeros());
        }
    }

    #[test]
    fn modal_block_accepts_external_modal_force_in_locked_order() {
        let mut block = ModalBlock::new(vec![
            ModeParams::new(6.0, 0.0, 10.0, 0.0, 1.0, 1.0).unwrap(),
        ])
        .unwrap();

        block
            .step_with_modal_force(Vector3::zeros(), &[20.0, -5.0], Duration::from_seconds(0.1))
            .unwrap();

        let state = block.modal_states()[0];
        assert!((state.q_ddot_x - 2.0).abs() < 1.0e-14);
        assert!((state.q_ddot_y + 0.5).abs() < 1.0e-14);
        assert!(
            block
                .step_with_modal_force(Vector3::zeros(), &[1.0], Duration::from_seconds(0.1))
                .is_err()
        );
    }

    #[test]
    fn modal_block_accepts_external_stiffness_scale_in_locked_order() {
        let mut block = ModalBlock::new(vec![
            ModeParams::new(5.0, 0.0, 10.0, 0.0, 1.0, 1.0).unwrap(),
        ])
        .unwrap();
        block
            .set_modal_states(&[ModalState {
                q_x: 2.0,
                q_y: -3.0,
                ..ModalState::default()
            }])
            .unwrap();

        block
            .step_with_modal_force_and_stiffness_scale(
                Vector3::zeros(),
                &[],
                &[0.25, 4.0],
                Duration::from_seconds(0.0),
            )
            .unwrap();

        let state = block.modal_states()[0];
        assert!((state.q_ddot_x + 12.5).abs() < 1.0e-14);
        assert!((state.q_ddot_y - 300.0).abs() < 1.0e-14);
        assert!(
            block
                .step_with_modal_force_and_stiffness_scale(
                    Vector3::zeros(),
                    &[],
                    &[1.0],
                    Duration::from_seconds(0.0),
                )
                .is_err()
        );
        assert!(
            block
                .step_with_modal_force_and_stiffness_scale(
                    Vector3::zeros(),
                    &[],
                    &[1.0, 0.0],
                    Duration::from_seconds(0.0),
                )
                .is_err()
        );
    }

    #[test]
    fn modal_block_sums_pickups_in_declaration_order() {
        let mut block = ModalBlock::new(vec![
            ModeParams::new(6.0, 0.02, 500.0, 1.0, 2.0, 0.25).unwrap(),
            ModeParams::new(11.0, 0.04, 750.0, -0.5, -3.0, -0.10).unwrap(),
        ])
        .unwrap();
        block
            .step(Vector3::new(4.0, -2.0, 0.0), Duration::from_seconds(0.01))
            .unwrap();
        let states = block.modal_states();
        let expected_gyro = Vector3::new(
            2.0 * states[0].q_dot_y + -3.0 * states[1].q_dot_y,
            2.0 * states[0].q_dot_x + -3.0 * states[1].q_dot_x,
            0.0,
        );
        let expected_accel = Vector3::new(
            0.25 * states[0].q_ddot_x + -0.10 * states[1].q_ddot_x,
            0.25 * states[0].q_ddot_y + -0.10 * states[1].q_ddot_y,
            0.0,
        );
        let expected_reaction = Vector3::new(
            -500.0 * 1.0 * states[0].q_ddot_y + -750.0 * -0.5 * states[1].q_ddot_y,
            -500.0 * 1.0 * states[0].q_ddot_x + -750.0 * -0.5 * states[1].q_ddot_x,
            0.0,
        );
        assert_eq!(block.gyro_pickup_rad_s(), expected_gyro);
        assert_eq!(block.accel_pickup_m_s2(), expected_accel);
        assert_eq!(block.reaction_moment_body_n_m(), expected_reaction);
    }

    #[test]
    fn modal_block_accepts_external_integrator_state_in_declaration_order() {
        let mut block = ModalBlock::new(vec![
            ModeParams::new(6.0, 0.02, 500.0, 1.0, 2.0, 0.25).unwrap(),
            ModeParams::new(11.0, 0.04, 750.0, -0.5, -3.0, -0.10).unwrap(),
        ])
        .unwrap();
        let states = [
            ModalState {
                q_x: 0.1,
                q_dot_x: 0.2,
                q_ddot_x: 0.3,
                q_y: -0.4,
                q_dot_y: -0.5,
                q_ddot_y: -0.6,
            },
            ModalState {
                q_x: 0.7,
                q_dot_x: 0.8,
                q_ddot_x: 0.9,
                q_y: -1.0,
                q_dot_y: -1.1,
                q_ddot_y: -1.2,
            },
        ];

        block.set_modal_states(&states).unwrap();

        assert_eq!(block.modal_states(), states);
        assert!(block.set_modal_states(&states[..1]).is_err());
        let mut non_finite = states;
        non_finite[0].q_x = f64::NAN;
        assert!(block.set_modal_states(&non_finite).is_err());
    }

    #[test]
    fn modal_block_axis_equation_uses_each_mode_frequency_and_damping() {
        let mut block = ModalBlock::new(vec![
            ModeParams::new(5.0, 0.1, 100.0, 0.0, 0.0, 0.0).unwrap(),
            ModeParams::new(9.0, 0.2, 100.0, 0.0, 0.0, 0.0).unwrap(),
        ])
        .unwrap();
        block.states[0].q_x = 1.0;
        block.states[0].q_dot_x = 0.5;
        block.states[1].q_y = -2.0;
        block.states[1].q_dot_y = -0.25;

        block
            .step(Vector3::zeros(), Duration::from_seconds(0.0))
            .unwrap();
        assert_eq!(
            block.modal_states()[0].q_ddot_x,
            -(5.0 * 5.0) * 1.0 - (2.0 * 0.1 * 5.0) * 0.5
        );
        assert_eq!(
            block.modal_states()[1].q_ddot_y,
            -(9.0 * 9.0) * -2.0 - (2.0 * 0.2 * 9.0) * -0.25
        );
    }
}
