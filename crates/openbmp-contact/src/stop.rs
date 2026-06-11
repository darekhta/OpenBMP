//! Scalar one-sided penalty stops.

use core::f64::consts::PI;

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
    /// The expression is the underdamped linear oscillator result
    /// `exp(-zeta*pi/sqrt(1-zeta^2))`; critically damped and overdamped stops
    /// return zero restitution.
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
        } else if damping_ratio >= 1.0 {
            Ok(0.0)
        } else {
            Ok((-damping_ratio * PI / (1.0 - damping_ratio * damping_ratio).sqrt()).exp())
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
    /// Instantaneous non-negative damping power.
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
        damping_power_w: if normal_force > 0.0 {
            damping_per_unit_rate * penetration_rate * penetration_rate
        } else {
            0.0
        },
    }
}

#[cfg(test)]
mod tests {
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
