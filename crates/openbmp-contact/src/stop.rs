//! Scalar one-sided penalty stops.

use crate::{
    ContactError,
    error::{require_finite, require_non_negative, require_positive},
};

/// Active side of a one-sided scalar stop.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StopSide {
    /// A lower stop engages when the coordinate falls below the limit and
    /// pushes the coordinate upward.
    Lower,
    /// An upper stop engages when the coordinate rises above the limit and
    /// pushes the coordinate downward.
    Upper,
}

/// One-sided Kelvin-Voigt penalty stop on a scalar coordinate.
///
/// The generalized force is positive for a lower stop and negative for an
/// upper stop. Damping is evaluated only while the coordinate penetrates the
/// stop, and the normal force is clamped to prevent tensile contact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScalarStop {
    limit: f64,
    side: StopSide,
    stiffness_per_unit: f64,
    damping_per_unit_rate: f64,
}

impl ScalarStop {
    /// Creates a scalar stop.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when `limit` is not finite,
    /// stiffness is not positive, or damping is negative/non-finite.
    pub fn new(
        limit: f64,
        side: StopSide,
        stiffness_per_unit: f64,
        damping_per_unit_rate: f64,
    ) -> Result<Self, ContactError> {
        Ok(Self {
            limit: require_finite("limit", limit)?,
            side,
            stiffness_per_unit: require_positive("stiffness_per_unit", stiffness_per_unit)?,
            damping_per_unit_rate: require_non_negative(
                "damping_per_unit_rate",
                damping_per_unit_rate,
            )?,
        })
    }

    /// Creates a lower stop that engages below `limit`.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] for invalid parameters.
    pub fn lower(
        limit: f64,
        stiffness_per_unit: f64,
        damping_per_unit_rate: f64,
    ) -> Result<Self, ContactError> {
        Self::new(
            limit,
            StopSide::Lower,
            stiffness_per_unit,
            damping_per_unit_rate,
        )
    }

    /// Creates an upper stop that engages above `limit`.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] for invalid parameters.
    pub fn upper(
        limit: f64,
        stiffness_per_unit: f64,
        damping_per_unit_rate: f64,
    ) -> Result<Self, ContactError> {
        Self::new(
            limit,
            StopSide::Upper,
            stiffness_per_unit,
            damping_per_unit_rate,
        )
    }

    /// Evaluates the stop response at a finite coordinate and coordinate rate.
    ///
    /// Positive coordinate rate moves toward increasing coordinate values.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] if either state component is
    /// non-finite.
    pub fn evaluate(
        self,
        coordinate: f64,
        coordinate_rate: f64,
    ) -> Result<ScalarStopResponse, ContactError> {
        let coordinate = require_finite("coordinate", coordinate)?;
        let coordinate_rate = require_finite("coordinate_rate", coordinate_rate)?;
        let response = match self.side {
            StopSide::Lower => evaluate_unilateral_penalty(
                self.limit - coordinate,
                coordinate_rate,
                1.0,
                self.stiffness_per_unit,
                self.damping_per_unit_rate,
            ),
            StopSide::Upper => evaluate_unilateral_penalty(
                coordinate - self.limit,
                -coordinate_rate,
                -1.0,
                self.stiffness_per_unit,
                self.damping_per_unit_rate,
            ),
        };
        Ok(response)
    }

    /// Returns the closed-form Kelvin-Voigt impact restitution for this stop.
    ///
    /// Release occurs when the spring-damper force first reaches zero on
    /// unloading; with damping, this precedes zero penetration. For damping ratio
    /// `zeta < 1`, restitution is
    /// `exp(-2*zeta*acos(zeta)/sqrt(1-zeta^2))`. At critical damping it is
    /// `exp(-2)`; for `zeta > 1`, `acos` and `sqrt(1-zeta^2)` are replaced by
    /// `acosh` and `sqrt(zeta^2-1)`. These expressions describe the clamped
    /// unilateral law used by [`Self::evaluate`], without external loads.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when effective mass is not
    /// positive.
    pub fn closed_form_restitution(self, effective_mass: f64) -> Result<f64, ContactError> {
        let effective_mass = require_positive("effective_mass", effective_mass)?;
        let damping_ratio =
            self.damping_per_unit_rate / (2.0 * (self.stiffness_per_unit * effective_mass).sqrt());
        if damping_ratio <= 0.0 {
            Ok(1.0)
        } else if damping_ratio < 1.0 {
            let frequency_ratio = ((1.0 - damping_ratio) * (1.0 + damping_ratio)).sqrt();
            Ok((-2.0 * damping_ratio * damping_ratio.acos() / frequency_ratio).exp())
        } else if damping_ratio > 1.0 {
            let inverse_ratio = 1.0 / damping_ratio;
            let frequency_ratio = ((1.0 - inverse_ratio) * (1.0 + inverse_ratio)).sqrt();
            Ok((-2.0 * damping_ratio.acosh() / frequency_ratio).exp())
        } else {
            Ok((-2.0_f64).exp())
        }
    }

    /// Returns the stop limit.
    #[must_use]
    pub const fn limit(self) -> f64 {
        self.limit
    }

    /// Returns the active stop side.
    #[must_use]
    pub const fn side(self) -> StopSide {
        self.side
    }

    /// Returns the penalty stiffness.
    #[must_use]
    pub const fn stiffness_per_unit(self) -> f64 {
        self.stiffness_per_unit
    }

    /// Returns the penalty damping.
    #[must_use]
    pub const fn damping_per_unit_rate(self) -> f64 {
        self.damping_per_unit_rate
    }
}

/// Generalized force and energy response from a scalar stop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScalarStopResponse {
    /// `true` when the clamped unilateral force is non-zero.
    pub active: bool,
    /// Non-negative stop penetration.
    pub penetration: f64,
    /// Generalized force conjugate to the scalar coordinate.
    pub generalized_force: f64,
    /// Elastic energy stored in the stop spring.
    pub elastic_energy_j: f64,
    /// Instantaneous non-negative loss from damping and unilateral clamping.
    pub damping_power_w: f64,
}

impl ScalarStopResponse {
    pub(crate) const fn free() -> Self {
        Self {
            active: false,
            penetration: 0.0,
            generalized_force: 0.0,
            elastic_energy_j: 0.0,
            damping_power_w: 0.0,
        }
    }
}

pub(crate) fn evaluate_unilateral_penalty(
    penetration: f64,
    normal_velocity: f64,
    outward_sign: f64,
    stiffness_per_unit: f64,
    damping_per_unit_rate: f64,
) -> ScalarStopResponse {
    if penetration <= 0.0 {
        return ScalarStopResponse::free();
    }

    let elastic_force = stiffness_per_unit * penetration;
    let damping_force = -damping_per_unit_rate * normal_velocity;
    let normal_force = (elastic_force + damping_force).max(0.0);
    let penetration_rate = -normal_velocity;
    ScalarStopResponse {
        active: normal_force > 0.0,
        penetration,
        generalized_force: outward_sign * normal_force,
        elastic_energy_j: 0.5 * stiffness_per_unit * penetration * penetration,
        // The elastic energy still decreases during force-clamped release;
        // account for this loss even though no work is done on the body.
        damping_power_w: ((normal_force - elastic_force) * penetration_rate).max(0.0),
    }
}

#[cfg(test)]
mod tests {
    use core::f64::consts::PI;

    use approx::{assert_abs_diff_eq, assert_relative_eq};

    use super::*;

    #[test]
    fn scalar_stop_lower_penalty_pushes_outward_and_clamps_tension() {
        let stop = ScalarStop::lower(0.0, 100.0, 10.0).unwrap();

        let closing = stop.evaluate(-0.05, -1.0).unwrap();
        assert!(closing.active);
        assert_abs_diff_eq!(closing.penetration, 0.05, epsilon = 1.0e-15);
        assert_abs_diff_eq!(closing.generalized_force, 15.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(closing.elastic_energy_j, 0.125, epsilon = 1.0e-15);
        assert_abs_diff_eq!(closing.damping_power_w, 10.0, epsilon = 1.0e-15);

        let separating = stop.evaluate(-0.05, 10.0).unwrap();
        assert!(!separating.active);
        assert_abs_diff_eq!(separating.penetration, 0.05, epsilon = 1.0e-15);
        assert_abs_diff_eq!(separating.generalized_force, 0.0, epsilon = 1.0e-15);
    }

    #[test]
    fn scalar_stop_upper_penalty_pushes_outward() {
        let stop = ScalarStop::upper(1.0, 200.0, 0.0).unwrap();
        let response = stop.evaluate(1.25, 0.0).unwrap();
        assert!(response.active);
        assert_abs_diff_eq!(response.penetration, 0.25, epsilon = 1.0e-15);
        assert_abs_diff_eq!(response.generalized_force, -50.0, epsilon = 1.0e-15);
    }

    #[test]
    fn scalar_stop_dissipation_closes_power_balance_for_both_sides() {
        for outward_sign in [-1.0, 1.0] {
            let stop = if outward_sign > 0.0 {
                ScalarStop::lower(0.0, 100.0, 10.0).unwrap()
            } else {
                ScalarStop::upper(0.0, 100.0, 10.0).unwrap()
            };
            for normal_velocity in [-1.0, 0.25, 1.0] {
                let coordinate_rate = outward_sign * normal_velocity;
                let response = stop
                    .evaluate(-outward_sign * 0.05, coordinate_rate)
                    .unwrap();
                let body_power_w = response.generalized_force * coordinate_rate;
                let elastic_power_w = -100.0 * 0.05 * normal_velocity;
                assert_abs_diff_eq!(
                    body_power_w + elastic_power_w + response.damping_power_w,
                    0.0,
                    epsilon = 1.0e-14
                );
            }
        }
    }

    fn integrated_restitution(stop: ScalarStop, mass: f64, dimensionless_step: f64) -> f64 {
        let omega = (stop.stiffness_per_unit() / mass).sqrt();
        let dt = dimensionless_step / omega;
        // Start infinitesimally inside the stop to avoid its force jump at
        // exactly zero penetration obscuring the release convergence check.
        let mut coordinate = -f64::EPSILON;
        let mut velocity = -1.0;
        for _ in 0..100_000 {
            let acceleration = |q, v| stop.evaluate(q, v).unwrap().generalized_force / mass;
            let a1 = acceleration(coordinate, velocity);
            let v2 = velocity + 0.5 * dt * a1;
            let a2 = acceleration(coordinate + 0.5 * dt * velocity, v2);
            let v3 = velocity + 0.5 * dt * a2;
            let a3 = acceleration(coordinate + 0.5 * dt * v2, v3);
            let v4 = velocity + dt * a3;
            let a4 = acceleration(coordinate + dt * v3, v4);
            coordinate += (dt / 6.0) * (velocity + 2.0 * v2 + 2.0 * v3 + v4);
            velocity += (dt / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4);
            if coordinate >= 0.0 && velocity > 0.0 {
                return velocity;
            }
        }
        unreachable!("impact must separate within the integration window");
    }

    #[test]
    fn scalar_stop_restitution_matches_unilateral_release_across_damping_regimes() {
        let mass = 2.0_f64;
        let stiffness = 4_000.0_f64;
        for damping_ratio in [0.08, 0.5, 1.0, 2.0] {
            let damping = 2.0 * damping_ratio * (stiffness * mass).sqrt();
            let stop = ScalarStop::lower(0.0, stiffness, damping).unwrap();
            let expected = stop.closed_form_restitution(mass).unwrap();
            let coarse = integrated_restitution(stop, mass, 0.002);
            let fine = integrated_restitution(stop, mass, 0.0005);

            assert!(
                fine > 0.0,
                "unilateral release still rebounds at zeta={damping_ratio}"
            );
            assert_relative_eq!(fine, expected, max_relative = 1.0e-6);
            assert!((fine - expected).abs() <= (coarse - expected).abs() + 1.0e-12);
        }
    }

    #[test]
    fn scalar_stop_restitution_matches_closed_form() {
        let mass = 2.0_f64;
        let stiffness = 4_000.0_f64;
        let damping_ratio = 0.08_f64;
        let damping = 2.0 * damping_ratio * (stiffness * mass).sqrt();
        let stop = ScalarStop::lower(0.0, stiffness, damping).unwrap();
        let expected = stop.closed_form_restitution(mass).unwrap();

        let natural_frequency = (stiffness / mass).sqrt();
        let contact_duration = PI / (natural_frequency * (1.0 - damping_ratio.powi(2)).sqrt());
        let dt = contact_duration / 30_000.0;
        let mut coordinate = 0.0;
        let mut velocity = -1.0;
        let mut entered_contact = false;

        for _ in 0..80_000 {
            let response = stop.evaluate(coordinate, velocity).unwrap();
            entered_contact |= response.penetration > 0.0;
            velocity += (response.generalized_force / mass) * dt;
            coordinate += velocity * dt;
            if entered_contact && coordinate >= 0.0 && velocity > 0.0 {
                break;
            }
        }

        let actual = velocity;
        assert_relative_eq!(actual, expected, max_relative = 0.02);
    }
}
