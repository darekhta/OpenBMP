//! 3-DOF point-mass state.

use openbmp_core::{Eci, Position3, SimTime, Velocity3};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::StateError;

/// 3-DOF point-mass state.
///
/// A point-mass state has time, position, velocity, and mass — no
/// orientation, no angular velocity, no inertia. It is the simplest
/// vehicle state OpenBMP supports and is the projection target of
/// [`crate::RigidBodyState`].
#[derive(Copy, Clone, Debug)]
pub struct PointMassState {
    /// Simulation time at which this state holds.
    pub time: SimTime,
    /// Position in the inertial (`Eci`) frame.
    pub position: Position3<Eci>,
    /// Velocity in the inertial (`Eci`) frame.
    pub velocity: Velocity3<Eci>,
    /// Total mass.
    pub mass: Mass,
}

impl PointMassState {
    /// Construct from explicit parts.
    #[must_use]
    pub fn new(
        time: SimTime,
        position: Position3<Eci>,
        velocity: Velocity3<Eci>,
        mass: Mass,
    ) -> Self {
        Self {
            time,
            position,
            velocity,
            mass,
        }
    }

    /// Convenience: mass in kilograms.
    #[must_use]
    pub fn mass_kg(&self) -> f64 {
        self.mass.get::<kilogram>()
    }

    /// Returns `true` if every numeric component is finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.time.is_finite()
            && self.position.is_finite()
            && self.velocity.is_finite()
            && self.mass_kg().is_finite()
    }

    /// Returns `true` if all point-mass state invariants hold.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.require_valid().is_ok()
    }

    /// Validate the state.
    ///
    /// Checks time validity, frame-vector finiteness, and strictly
    /// positive mass.
    ///
    /// # Errors
    ///
    /// Returns the first failed check as a [`StateError`] variant.
    pub fn require_valid(&self) -> Result<(), StateError> {
        self.time.require_valid()?;
        self.position.require_finite()?;
        self.velocity.require_finite()?;
        let mass_kg = self.mass_kg();
        if !mass_kg.is_finite() {
            return Err(StateError::MassNotFinite { mass_kg });
        }
        if mass_kg <= 0.0 {
            return Err(StateError::NonPositiveMass { mass_kg });
        }
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn one_kg() -> Mass {
        Mass::new::<kilogram>(1.0)
    }

    #[test]
    fn constructor_round_trip() {
        let state = PointMassState::new(
            SimTime::from_seconds(1.0),
            Position3::new(1.0, 2.0, 3.0),
            Velocity3::new(0.0, 0.0, 0.0),
            one_kg(),
        );
        assert_abs_diff_eq!(state.time.as_seconds(), 1.0);
        assert_abs_diff_eq!(state.position.vector.x, 1.0);
        assert_abs_diff_eq!(state.mass_kg(), 1.0);
    }

    #[test]
    fn require_valid_accepts_normal_state() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            one_kg(),
        );
        assert!(state.require_valid().is_ok());
        assert!(state.is_finite());
        assert!(state.is_valid());
    }

    #[test]
    fn require_valid_rejects_negative_time() {
        let state = PointMassState::new(
            SimTime::from_seconds(-1.0),
            Position3::origin(),
            Velocity3::zero(),
            one_kg(),
        );
        assert!(state.require_valid().is_err());
    }

    #[test]
    fn require_valid_rejects_nan_position() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::new(f64::NAN, 0.0, 0.0),
            Velocity3::zero(),
            one_kg(),
        );
        let err = state.require_valid().unwrap_err();
        assert!(matches!(err, StateError::Frame(_)));
    }

    #[test]
    fn require_valid_rejects_zero_mass() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(0.0),
        );
        let err = state.require_valid().unwrap_err();
        assert!(matches!(err, StateError::NonPositiveMass { .. }));
    }

    #[test]
    fn require_valid_rejects_nan_mass() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(f64::NAN),
        );
        let err = state.require_valid().unwrap_err();
        assert!(matches!(err, StateError::MassNotFinite { .. }));
    }
}
