//! Anchored stick/slip friction and rest-detection helpers.

use core::f64::consts::TAU;

use crate::{
    ContactError, ContactVector3, add,
    error::{require_finite, require_non_negative, require_positive},
    finite_vector, norm, scale,
};

/// Stick/slip mode for anchored tangential friction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StictionMode {
    /// No active normal contact.
    Free,
    /// Tangential spring anchor is engaged and inside the static cone.
    Sticking,
    /// Contact is sliding under kinetic friction.
    Sliding,
}

/// Stateful anchor for [`AnchoredStictionFriction`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnchoredStictionState {
    mode: StictionMode,
    anchor_displacement_m: ContactVector3,
}

impl AnchoredStictionState {
    /// Creates a free, unanchored stiction state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            mode: StictionMode::Free,
            anchor_displacement_m: [0.0, 0.0, 0.0],
        }
    }

    /// Returns the current stick/slip mode.
    #[must_use]
    pub const fn mode(self) -> StictionMode {
        self.mode
    }

    /// Returns the tangential displacement from the current stick anchor.
    #[must_use]
    pub const fn anchor_displacement_m(self) -> ContactVector3 {
        self.anchor_displacement_m
    }
}

impl Default for AnchoredStictionState {
    fn default() -> Self {
        Self::new()
    }
}

/// Deterministic anchored stiction with a Karnopp restick velocity window.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnchoredStictionFriction {
    static_coefficient: f64,
    kinetic_coefficient: f64,
    tangential_stiffness_n_m: f64,
    tangential_damping_n_s_m: f64,
    restick_speed_m_s: f64,
}

impl AnchoredStictionFriction {
    /// Creates an anchored stiction friction model.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when coefficients are
    /// invalid, `static_coefficient < kinetic_coefficient`, tangential
    /// stiffness is not positive, damping is negative/non-finite, or the
    /// restick speed is not positive.
    pub fn new(
        static_coefficient: f64,
        kinetic_coefficient: f64,
        tangential_stiffness_n_m: f64,
        tangential_damping_n_s_m: f64,
        restick_speed_m_s: f64,
    ) -> Result<Self, ContactError> {
        let static_coefficient = require_non_negative("static_coefficient", static_coefficient)?;
        let kinetic_coefficient = require_non_negative("kinetic_coefficient", kinetic_coefficient)?;
        if static_coefficient < kinetic_coefficient {
            return Err(ContactError::InvalidParameter {
                field: "static_coefficient",
                reason: "must be greater than or equal to kinetic_coefficient",
            });
        }
        Ok(Self {
            static_coefficient,
            kinetic_coefficient,
            tangential_stiffness_n_m: require_positive(
                "tangential_stiffness_n_m",
                tangential_stiffness_n_m,
            )?,
            tangential_damping_n_s_m: require_non_negative(
                "tangential_damping_n_s_m",
                tangential_damping_n_s_m,
            )?,
            restick_speed_m_s: require_positive("restick_speed_m_s", restick_speed_m_s)?,
        })
    }

    /// Evaluates one fixed sub-step and updates the stick/slip state.
    ///
    /// Stick/slip transitions are decided once per call. A sticking contact
    /// integrates anchor displacement explicitly, breaks when the trial
    /// spring-damper force exceeds `mu_s * normal_force_n`, and re-sticks from
    /// sliding only when tangential speed is inside the Karnopp window.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] or
    /// [`ContactError::InvalidVector`] for non-finite state inputs or a
    /// negative/non-finite step size.
    pub fn evaluate(
        self,
        state: &mut AnchoredStictionState,
        normal_force_n: f64,
        tangential_velocity_m_s: ContactVector3,
        dt_s: f64,
    ) -> Result<AnchoredStictionResponse, ContactError> {
        let normal_force_n = require_non_negative("normal_force_n", normal_force_n)?;
        let tangential_velocity_m_s =
            finite_vector("tangential_velocity_m_s", tangential_velocity_m_s)?;
        let dt_s = require_non_negative("dt_s", dt_s)?;
        let speed_m_s = norm(tangential_velocity_m_s);

        if normal_force_n <= 0.0 {
            *state = AnchoredStictionState::new();
            return Ok(AnchoredStictionResponse::free());
        }

        if matches!(state.mode, StictionMode::Free) {
            state.mode = if speed_m_s <= self.restick_speed_m_s {
                StictionMode::Sticking
            } else {
                StictionMode::Sliding
            };
            state.anchor_displacement_m = [0.0, 0.0, 0.0];
        }

        if matches!(state.mode, StictionMode::Sliding) && speed_m_s <= self.restick_speed_m_s {
            state.mode = StictionMode::Sticking;
            state.anchor_displacement_m = [0.0, 0.0, 0.0];
        }

        match state.mode {
            StictionMode::Free => Ok(AnchoredStictionResponse::free()),
            StictionMode::Sticking => {
                state.anchor_displacement_m = [
                    state.anchor_displacement_m[0] + tangential_velocity_m_s[0] * dt_s,
                    state.anchor_displacement_m[1] + tangential_velocity_m_s[1] * dt_s,
                    state.anchor_displacement_m[2] + tangential_velocity_m_s[2] * dt_s,
                ];
                let elastic_force =
                    scale(state.anchor_displacement_m, -self.tangential_stiffness_n_m);
                let damping_force = scale(tangential_velocity_m_s, -self.tangential_damping_n_s_m);
                let trial_force_n = add(elastic_force, damping_force);
                let static_limit_n = self.static_coefficient * normal_force_n;
                let trial_magnitude_n = norm(trial_force_n);

                if trial_magnitude_n <= static_limit_n {
                    let anchor_displacement_m = norm(state.anchor_displacement_m);
                    Ok(AnchoredStictionResponse {
                        mode: StictionMode::Sticking,
                        sticking: true,
                        friction_force_n: trial_force_n,
                        static_limit_n,
                        elastic_energy_j: 0.5
                            * self.tangential_stiffness_n_m
                            * anchor_displacement_m
                            * anchor_displacement_m,
                        dissipated_energy_j: self.tangential_damping_n_s_m
                            * speed_m_s
                            * speed_m_s
                            * dt_s,
                    })
                } else {
                    state.mode = StictionMode::Sliding;
                    state.anchor_displacement_m = [0.0, 0.0, 0.0];
                    Ok(self.sliding_response(normal_force_n, tangential_velocity_m_s, dt_s))
                }
            }
            StictionMode::Sliding => {
                Ok(self.sliding_response(normal_force_n, tangential_velocity_m_s, dt_s))
            }
        }
    }

    fn sliding_response(
        self,
        normal_force_n: f64,
        tangential_velocity_m_s: ContactVector3,
        dt_s: f64,
    ) -> AnchoredStictionResponse {
        let speed_m_s = norm(tangential_velocity_m_s);
        let static_limit_n = self.static_coefficient * normal_force_n;
        if speed_m_s <= 0.0 || self.kinetic_coefficient == 0.0 {
            return AnchoredStictionResponse {
                mode: StictionMode::Sliding,
                sticking: false,
                friction_force_n: [0.0, 0.0, 0.0],
                static_limit_n,
                elastic_energy_j: 0.0,
                dissipated_energy_j: 0.0,
            };
        }
        let magnitude_n = self.kinetic_coefficient * normal_force_n;
        AnchoredStictionResponse {
            mode: StictionMode::Sliding,
            sticking: false,
            friction_force_n: scale(tangential_velocity_m_s, -magnitude_n / speed_m_s),
            static_limit_n,
            elastic_energy_j: 0.0,
            dissipated_energy_j: magnitude_n * speed_m_s * dt_s,
        }
    }
}

/// Force and energy output from anchored stiction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnchoredStictionResponse {
    /// Stick/slip mode after this sub-step.
    pub mode: StictionMode,
    /// `true` when the contact is statically stuck after this sub-step.
    pub sticking: bool,
    /// Tangential friction force in newtons.
    pub friction_force_n: ContactVector3,
    /// Static friction cone limit `mu_s * normal_force_n`.
    pub static_limit_n: f64,
    /// Elastic energy stored in the tangential anchor.
    pub elastic_energy_j: f64,
    /// Non-negative work dissipated during the sub-step.
    pub dissipated_energy_j: f64,
}

impl AnchoredStictionResponse {
    const fn free() -> Self {
        Self {
            mode: StictionMode::Free,
            sticking: false,
            friction_force_n: [0.0, 0.0, 0.0],
            static_limit_n: 0.0,
            elastic_energy_j: 0.0,
            dissipated_energy_j: 0.0,
        }
    }
}

/// Returns `tan(abs(incline_angle_rad))`, the exact static coefficient needed
/// to hold a block on an incline.
///
/// # Errors
///
/// Returns [`ContactError::InvalidParameter`] when the angle is not finite.
pub fn incline_required_static_coefficient(incline_angle_rad: f64) -> Result<f64, ContactError> {
    Ok(require_finite("incline_angle_rad", incline_angle_rad)?
        .abs()
        .tan())
}

/// Returns whether static friction can hold a block on the incline.
///
/// # Errors
///
/// Returns [`ContactError::InvalidParameter`] when inputs are invalid.
pub fn incline_stiction_holds(
    incline_angle_rad: f64,
    static_coefficient: f64,
) -> Result<bool, ContactError> {
    let static_coefficient = require_non_negative("static_coefficient", static_coefficient)?;
    Ok(incline_required_static_coefficient(incline_angle_rad)? <= static_coefficient)
}

/// Returns the down-slope kinetic acceleration for a sliding block.
///
/// The expression is `g * (sin(theta) - mu_k * cos(theta))`; positive values
/// accelerate down the positive incline direction.
///
/// # Errors
///
/// Returns [`ContactError::InvalidParameter`] when inputs are invalid.
pub fn incline_sliding_acceleration_m_s2(
    incline_angle_rad: f64,
    kinetic_coefficient: f64,
    gravity_m_s2: f64,
) -> Result<f64, ContactError> {
    let incline_angle_rad = require_finite("incline_angle_rad", incline_angle_rad)?;
    let kinetic_coefficient = require_non_negative("kinetic_coefficient", kinetic_coefficient)?;
    let gravity_m_s2 = require_positive("gravity_m_s2", gravity_m_s2)?;
    Ok(gravity_m_s2 * (incline_angle_rad.sin() - kinetic_coefficient * incline_angle_rad.cos()))
}

/// Rest detector configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RestDetectorConfig {
    kinetic_energy_floor_j: f64,
    hold_steps: u32,
}

impl RestDetectorConfig {
    /// Creates a rest detector configuration.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when the energy floor is
    /// negative/non-finite or `hold_steps` is zero.
    pub fn new(kinetic_energy_floor_j: f64, hold_steps: u32) -> Result<Self, ContactError> {
        if hold_steps == 0 {
            return Err(ContactError::InvalidParameter {
                field: "hold_steps",
                reason: "must be positive",
            });
        }
        Ok(Self {
            kinetic_energy_floor_j: require_non_negative(
                "kinetic_energy_floor_j",
                kinetic_energy_floor_j,
            )?,
            hold_steps,
        })
    }
}

/// Deterministic rest detector with a consecutive-quiet-step hold.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RestDetector {
    config: RestDetectorConfig,
}

impl RestDetector {
    /// Creates a rest detector.
    #[must_use]
    pub const fn new(config: RestDetectorConfig) -> Self {
        Self { config }
    }

    /// Updates detector state from kinetic energy and contact sticking state.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when kinetic energy is
    /// negative/non-finite.
    pub fn update(
        self,
        state: &mut RestDetectorState,
        kinetic_energy_j: f64,
        all_contacts_sticking: bool,
    ) -> Result<RestStatus, ContactError> {
        let kinetic_energy_j = require_non_negative("kinetic_energy_j", kinetic_energy_j)?;
        if kinetic_energy_j <= self.config.kinetic_energy_floor_j && all_contacts_sticking {
            state.quiet_steps = state.quiet_steps.saturating_add(1);
        } else {
            state.quiet_steps = 0;
        }
        Ok(RestStatus {
            quiet_steps: state.quiet_steps,
            at_rest: state.quiet_steps >= self.config.hold_steps,
        })
    }
}

/// Runtime state for [`RestDetector`].
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RestDetectorState {
    quiet_steps: u32,
}

impl RestDetectorState {
    /// Creates a detector state with no accumulated quiet steps.
    #[must_use]
    pub const fn new() -> Self {
        Self { quiet_steps: 0 }
    }

    /// Returns consecutive quiet steps.
    #[must_use]
    pub const fn quiet_steps(self) -> u32 {
        self.quiet_steps
    }
}

/// Rest detector output.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RestStatus {
    /// Consecutive quiet steps after this update.
    pub quiet_steps: u32,
    /// `true` when the hold count has been met.
    pub at_rest: bool,
}

/// Closed-form Housner rocking-block anchor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HousnerRockingBlock {
    half_width_m: f64,
    half_height_m: f64,
    gravity_m_s2: f64,
}

impl HousnerRockingBlock {
    /// Creates a rocking-block anchor from half-width, half-height, and
    /// gravity.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when any value is not
    /// positive.
    pub fn new(
        half_width_m: f64,
        half_height_m: f64,
        gravity_m_s2: f64,
    ) -> Result<Self, ContactError> {
        Ok(Self {
            half_width_m: require_positive("half_width_m", half_width_m)?,
            half_height_m: require_positive("half_height_m", half_height_m)?,
            gravity_m_s2: require_positive("gravity_m_s2", gravity_m_s2)?,
        })
    }

    /// Returns the exact static tip threshold `tan(theta) = b / h`.
    #[must_use]
    pub fn static_tip_threshold_slope(self) -> f64 {
        self.half_width_m / self.half_height_m
    }

    /// Returns the exact static tip angle in radians.
    #[must_use]
    pub fn static_tip_threshold_angle_rad(self) -> f64 {
        self.static_tip_threshold_slope().atan()
    }

    /// Returns `R = sqrt(b^2 + h^2)`.
    #[must_use]
    pub fn radius_m(self) -> f64 {
        self.half_width_m.hypot(self.half_height_m)
    }

    /// Returns Housner's rocking frequency `p = sqrt(3g / (4R))`.
    #[must_use]
    pub fn rocking_frequency_rad_s(self) -> f64 {
        (3.0 * self.gravity_m_s2 / (4.0 * self.radius_m())).sqrt()
    }

    /// Returns the small-amplitude period `2*pi / p`.
    #[must_use]
    pub fn small_amplitude_period_s(self) -> f64 {
        TAU / self.rocking_frequency_rad_s()
    }
}

#[cfg(test)]
mod tests {
    use approx::{assert_abs_diff_eq, assert_relative_eq};

    use super::*;

    #[test]
    fn stiction_incline_threshold_is_exact() {
        let theta = 0.2_f64;
        let required = incline_required_static_coefficient(theta).unwrap();
        assert_abs_diff_eq!(required, theta.tan(), epsilon = 0.0);
        assert!(incline_stiction_holds(theta, required).unwrap());
        assert!(!incline_stiction_holds(theta, required - 1.0e-12).unwrap());
    }

    #[test]
    fn stiction_sliding_incline_acceleration_matches_closed_form() {
        let theta = 0.3_f64;
        let mu_k = 0.2_f64;
        let gravity = 9.80665_f64;
        let accel = incline_sliding_acceleration_m_s2(theta, mu_k, gravity).unwrap();
        assert_abs_diff_eq!(
            accel,
            gravity * (theta.sin() - mu_k * theta.cos()),
            epsilon = 1.0e-15
        );
    }

    #[test]
    fn stiction_anchor_does_not_drift_at_rest_over_10000_steps() {
        let model = AnchoredStictionFriction::new(0.6, 0.4, 10_000.0, 20.0, 1.0e-4).unwrap();
        let mut state = AnchoredStictionState::new();

        for _ in 0..10_000 {
            let response = model
                .evaluate(&mut state, 100.0, [0.0, 0.0, 0.0], 0.001)
                .unwrap();
            assert!(response.sticking);
        }

        assert_eq!(state.mode(), StictionMode::Sticking);
        assert_abs_diff_eq!(state.anchor_displacement_m()[0], 0.0, epsilon = 0.0);
        assert_abs_diff_eq!(state.anchor_displacement_m()[1], 0.0, epsilon = 0.0);
        assert_abs_diff_eq!(state.anchor_displacement_m()[2], 0.0, epsilon = 0.0);
    }

    #[test]
    fn stiction_breaks_to_kinetic_when_static_cone_is_exceeded() {
        let model = AnchoredStictionFriction::new(0.6, 0.4, 100.0, 0.0, 1.0).unwrap();
        let mut state = AnchoredStictionState::new();
        let response = model
            .evaluate(&mut state, 10.0, [10.0, 0.0, 0.0], 0.01)
            .unwrap();

        assert_eq!(response.mode, StictionMode::Sliding);
        assert!(!response.sticking);
        assert_abs_diff_eq!(response.friction_force_n[0], -4.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(response.static_limit_n, 6.0, epsilon = 1.0e-15);
        assert_eq!(state.mode(), StictionMode::Sliding);
        assert_abs_diff_eq!(state.anchor_displacement_m()[0], 0.0, epsilon = 0.0);
    }

    #[test]
    fn stiction_drive_spring_oscillator_matches_karnopp_reference() {
        let reference = KarnoppDriveSpringReference::new(1.0, 50.0, 0.1, 10.0, 0.6, 0.4);
        let model = AnchoredStictionFriction::new(0.6, 0.4, 20_000.0, 60.0, 1.0e-5).unwrap();
        let measured = simulate_drive_spring_oscillator(model, reference, 1.0e-5, 4.6);

        assert_relative_eq!(
            measured.first_slip_duration_s,
            reference.slip_duration_s(),
            max_relative = 0.02
        );
        assert_relative_eq!(
            measured.first_cycle_period_s,
            reference.cycle_period_s(),
            max_relative = 0.02
        );
        assert_relative_eq!(
            measured.first_cycle_displacement_m,
            reference.cycle_displacement_m(),
            max_relative = 0.02
        );
    }

    #[test]
    fn rest_detector_requires_energy_floor_sticking_and_hold_count() {
        let detector = RestDetector::new(RestDetectorConfig::new(1.0e-6, 3).unwrap());
        let mut state = RestDetectorState::new();

        assert!(!detector.update(&mut state, 1.0e-7, true).unwrap().at_rest);
        assert!(!detector.update(&mut state, 1.0e-7, true).unwrap().at_rest);
        assert!(detector.update(&mut state, 1.0e-7, true).unwrap().at_rest);
        assert_eq!(state.quiet_steps(), 3);

        let status = detector.update(&mut state, 1.0e-3, true).unwrap();
        assert!(!status.at_rest);
        assert_eq!(status.quiet_steps, 0);
    }

    #[test]
    fn rocking_block_static_tip_threshold_and_period_match_housner_forms() {
        let block = HousnerRockingBlock::new(0.4, 1.6, 9.80665).unwrap();
        assert_abs_diff_eq!(block.static_tip_threshold_slope(), 0.25, epsilon = 0.0);
        assert_abs_diff_eq!(
            block.static_tip_threshold_angle_rad(),
            0.25_f64.atan(),
            epsilon = 0.0
        );

        let expected_radius = 0.4_f64.hypot(1.6);
        let expected_frequency = (3.0 * 9.80665 / (4.0 * expected_radius)).sqrt();
        assert_relative_eq!(
            block.rocking_frequency_rad_s(),
            expected_frequency,
            max_relative = 1.0e-15
        );
        assert_relative_eq!(
            block.small_amplitude_period_s(),
            TAU / expected_frequency,
            max_relative = 1.0e-15
        );
    }

    #[derive(Clone, Copy, Debug)]
    struct KarnoppDriveSpringReference {
        mass_kg: f64,
        drive_stiffness_n_m: f64,
        drive_speed_m_s: f64,
        normal_force_n: f64,
        static_coefficient: f64,
        kinetic_coefficient: f64,
    }

    impl KarnoppDriveSpringReference {
        const fn new(
            mass_kg: f64,
            drive_stiffness_n_m: f64,
            drive_speed_m_s: f64,
            normal_force_n: f64,
            static_coefficient: f64,
            kinetic_coefficient: f64,
        ) -> Self {
            Self {
                mass_kg,
                drive_stiffness_n_m,
                drive_speed_m_s,
                normal_force_n,
                static_coefficient,
                kinetic_coefficient,
            }
        }

        fn omega_rad_s(self) -> f64 {
            (self.drive_stiffness_n_m / self.mass_kg).sqrt()
        }

        fn static_extension_m(self) -> f64 {
            self.static_coefficient * self.normal_force_n / self.drive_stiffness_n_m
        }

        fn kinetic_extension_m(self) -> f64 {
            self.kinetic_coefficient * self.normal_force_n / self.drive_stiffness_n_m
        }

        fn slip_duration_s(self) -> f64 {
            let omega = self.omega_rad_s();
            let extension_delta_m = self.static_extension_m() - self.kinetic_extension_m();
            2.0 * (core::f64::consts::PI
                - (extension_delta_m * omega / self.drive_speed_m_s).atan())
                / omega
        }

        fn restick_extension_m(self) -> f64 {
            let omega = self.omega_rad_s();
            let slip_phase_rad = omega * self.slip_duration_s();
            let extension_delta_m = self.static_extension_m() - self.kinetic_extension_m();
            self.kinetic_extension_m()
                + extension_delta_m * slip_phase_rad.cos()
                + (self.drive_speed_m_s / omega) * slip_phase_rad.sin()
        }

        fn stick_duration_s(self) -> f64 {
            (self.static_extension_m() - self.restick_extension_m()) / self.drive_speed_m_s
        }

        fn cycle_period_s(self) -> f64 {
            self.slip_duration_s() + self.stick_duration_s()
        }

        fn cycle_displacement_m(self) -> f64 {
            self.drive_speed_m_s * self.cycle_period_s()
        }
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct DriveSpringMeasurement {
        first_slip_start_s: Option<f64>,
        first_resticking_s: Option<f64>,
        second_slip_start_s: Option<f64>,
        first_resticking_position_m: Option<f64>,
        second_resticking_position_m: Option<f64>,
    }

    impl DriveSpringMeasurement {
        fn finish(self) -> MeasuredDriveSpringCycle {
            let first_slip_start_s = self.first_slip_start_s.unwrap();
            let first_resticking_s = self.first_resticking_s.unwrap();
            let second_slip_start_s = self.second_slip_start_s.unwrap();
            let first_resticking_position_m = self.first_resticking_position_m.unwrap();
            let second_resticking_position_m = self.second_resticking_position_m.unwrap();
            MeasuredDriveSpringCycle {
                first_slip_duration_s: first_resticking_s - first_slip_start_s,
                first_cycle_period_s: second_slip_start_s - first_slip_start_s,
                first_cycle_displacement_m: second_resticking_position_m
                    - first_resticking_position_m,
            }
        }
    }

    #[derive(Clone, Copy, Debug)]
    struct MeasuredDriveSpringCycle {
        first_slip_duration_s: f64,
        first_cycle_period_s: f64,
        first_cycle_displacement_m: f64,
    }

    fn simulate_drive_spring_oscillator(
        friction: AnchoredStictionFriction,
        reference: KarnoppDriveSpringReference,
        dt_s: f64,
        stop_s: f64,
    ) -> MeasuredDriveSpringCycle {
        let mut friction_state = AnchoredStictionState::new();
        let mut measurement = DriveSpringMeasurement::default();
        let mut previous_mode = StictionMode::Free;
        let mut time_s = 0.0;
        let mut position_m = 0.0;
        let mut velocity_m_s = 0.0;

        while time_s < stop_s && measurement.second_resticking_position_m.is_none() {
            let drive_force_n =
                reference.drive_stiffness_n_m * (reference.drive_speed_m_s * time_s - position_m);
            let response = friction
                .evaluate(
                    &mut friction_state,
                    reference.normal_force_n,
                    [velocity_m_s, 0.0, 0.0],
                    dt_s,
                )
                .unwrap();

            match (previous_mode, response.mode) {
                (StictionMode::Sticking, StictionMode::Sliding) => {
                    if measurement.first_slip_start_s.is_none() {
                        measurement.first_slip_start_s = Some(time_s);
                    } else if measurement.second_slip_start_s.is_none() {
                        measurement.second_slip_start_s = Some(time_s);
                    }
                }
                (StictionMode::Sliding, StictionMode::Sticking) => {
                    if measurement.first_resticking_s.is_none() {
                        measurement.first_resticking_s = Some(time_s);
                        measurement.first_resticking_position_m = Some(position_m);
                    } else if measurement.second_resticking_position_m.is_none() {
                        measurement.second_resticking_position_m = Some(position_m);
                    }
                }
                _ => {}
            }
            previous_mode = response.mode;

            let acceleration_m_s2 =
                (drive_force_n + response.friction_force_n[0]) / reference.mass_kg;
            velocity_m_s += acceleration_m_s2 * dt_s;
            position_m += velocity_m_s * dt_s;
            time_s += dt_s;
        }

        measurement.finish()
    }
}
