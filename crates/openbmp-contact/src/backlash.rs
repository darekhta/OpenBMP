//! Two-sided scalar backlash gap.

use crate::{
    ContactError,
    error::{require_finite, require_non_negative, require_positive},
    stop::evaluate_unilateral_penalty,
};

/// Active flank of a two-sided backlash gap.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BacklashFlank {
    /// Coordinate is inside the dead zone.
    Free,
    /// Coordinate is below the lower dead-zone boundary.
    Lower,
    /// Coordinate is above the upper dead-zone boundary.
    Upper,
}

/// Symmetric dead-zone backlash with Kelvin-Voigt flank springs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BacklashGap {
    center: f64,
    dead_zone_width: f64,
    stiffness_per_unit: f64,
    damping_per_unit_rate: f64,
}

impl BacklashGap {
    /// Creates a backlash gap centered on `center`.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when the center is not
    /// finite, dead-zone width is negative/non-finite, stiffness is not
    /// positive, or damping is negative/non-finite.
    pub fn new(
        center: f64,
        dead_zone_width: f64,
        stiffness_per_unit: f64,
        damping_per_unit_rate: f64,
    ) -> Result<Self, ContactError> {
        Ok(Self {
            center: require_finite("center", center)?,
            dead_zone_width: require_non_negative("dead_zone_width", dead_zone_width)?,
            stiffness_per_unit: require_positive("stiffness_per_unit", stiffness_per_unit)?,
            damping_per_unit_rate: require_non_negative(
                "damping_per_unit_rate",
                damping_per_unit_rate,
            )?,
        })
    }

    /// Evaluates the backlash response at a finite coordinate and rate.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] if either state component is
    /// non-finite.
    pub fn evaluate(
        self,
        coordinate: f64,
        coordinate_rate: f64,
    ) -> Result<BacklashResponse, ContactError> {
        let coordinate = require_finite("coordinate", coordinate)?;
        let coordinate_rate = require_finite("coordinate_rate", coordinate_rate)?;
        let lower_limit = self.lower_limit();
        let upper_limit = self.upper_limit();

        if coordinate < lower_limit {
            let stop = evaluate_unilateral_penalty(
                lower_limit - coordinate,
                coordinate_rate,
                1.0,
                self.stiffness_per_unit,
                self.damping_per_unit_rate,
            );
            Ok(BacklashResponse::from_stop(BacklashFlank::Lower, stop))
        } else if coordinate > upper_limit {
            let stop = evaluate_unilateral_penalty(
                coordinate - upper_limit,
                -coordinate_rate,
                -1.0,
                self.stiffness_per_unit,
                self.damping_per_unit_rate,
            );
            Ok(BacklashResponse::from_stop(BacklashFlank::Upper, stop))
        } else {
            Ok(BacklashResponse::free())
        }
    }

    /// Returns the lower dead-zone boundary.
    #[must_use]
    pub fn lower_limit(self) -> f64 {
        self.center - (0.5 * self.dead_zone_width)
    }

    /// Returns the upper dead-zone boundary.
    #[must_use]
    pub fn upper_limit(self) -> f64 {
        self.center + (0.5 * self.dead_zone_width)
    }

    /// Returns the dead-zone center.
    #[must_use]
    pub const fn center(self) -> f64 {
        self.center
    }

    /// Returns the exact configured dead-zone width.
    #[must_use]
    pub const fn dead_zone_width(self) -> f64 {
        self.dead_zone_width
    }
}

/// Generalized force and energy response from a backlash gap.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BacklashResponse {
    /// Active flank, or [`BacklashFlank::Free`] inside the dead zone.
    pub flank: BacklashFlank,
    /// `true` when the clamped flank force is non-zero.
    pub active: bool,
    /// Non-negative flank penetration.
    pub penetration: f64,
    /// Generalized force conjugate to the scalar coordinate.
    pub generalized_force: f64,
    /// Elastic energy stored in the active flank spring.
    pub elastic_energy_j: f64,
    /// Instantaneous non-negative damping power.
    pub damping_power_w: f64,
}

impl BacklashResponse {
    const fn free() -> Self {
        Self {
            flank: BacklashFlank::Free,
            active: false,
            penetration: 0.0,
            generalized_force: 0.0,
            elastic_energy_j: 0.0,
            damping_power_w: 0.0,
        }
    }

    const fn from_stop(flank: BacklashFlank, stop: crate::ScalarStopResponse) -> Self {
        Self {
            flank,
            active: stop.active,
            penetration: stop.penetration,
            generalized_force: stop.generalized_force,
            elastic_energy_j: stop.elastic_energy_j,
            damping_power_w: stop.damping_power_w,
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn backlash_dead_zone_width_is_exact() {
        let gap = BacklashGap::new(10.0, 0.5, 1_000.0, 0.0).unwrap();
        assert_abs_diff_eq!(gap.dead_zone_width(), 0.5, epsilon = 0.0);
        assert_abs_diff_eq!(gap.lower_limit(), 9.75, epsilon = 0.0);
        assert_abs_diff_eq!(gap.upper_limit(), 10.25, epsilon = 0.0);

        assert_eq!(
            gap.evaluate(gap.lower_limit(), 0.0).unwrap().flank,
            BacklashFlank::Free
        );
        assert_eq!(
            gap.evaluate(gap.upper_limit(), 0.0).unwrap().flank,
            BacklashFlank::Free
        );
    }

    #[test]
    fn backlash_flank_springs_apply_opposing_force() {
        let gap = BacklashGap::new(10.0, 0.5, 1_000.0, 0.0).unwrap();

        let lower = gap.evaluate(9.625, 0.0).unwrap();
        assert_eq!(lower.flank, BacklashFlank::Lower);
        assert!(lower.active);
        assert_abs_diff_eq!(lower.penetration, 0.125, epsilon = 1.0e-15);
        assert_abs_diff_eq!(lower.generalized_force, 125.0, epsilon = 1.0e-15);

        let upper = gap.evaluate(10.5, 0.0).unwrap();
        assert_eq!(upper.flank, BacklashFlank::Upper);
        assert!(upper.active);
        assert_abs_diff_eq!(upper.penetration, 0.25, epsilon = 1.0e-15);
        assert_abs_diff_eq!(upper.generalized_force, -250.0, epsilon = 1.0e-15);
    }
}
