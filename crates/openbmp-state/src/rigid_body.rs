//! 6-DOF rigid-body state.

use openbmp_core::{AngularVelocity3, Body, Eci, Position3, Quaternion, SimTime, Velocity3};

use crate::error::StateError;
use crate::mass_properties::MassProperties;
use crate::point_mass::PointMassState;

/// 6-DOF rigid-body state.
///
/// Position and velocity are in `Eci`. Orientation maps `Body` →
/// `Eci`. Angular velocity is expressed in `Body` (the rotating
/// frame's own basis, the standard convention for body-frame angular
/// rate).
#[derive(Copy, Clone, Debug)]
pub struct RigidBodyState {
    /// Monotonic timestamp at which this state holds.
    pub time: SimTime,
    /// Position in the inertial (`Eci`) frame.
    pub position: Position3<Eci>,
    /// Velocity in the inertial (`Eci`) frame.
    pub velocity: Velocity3<Eci>,
    /// Body-to-inertial orientation quaternion.
    pub orientation: Quaternion<Body, Eci>,
    /// Angular velocity expressed in the body frame.
    pub angular_velocity: AngularVelocity3<Body>,
    /// Mass properties (mass, body-frame CG, body-frame inertia).
    pub mass_props: MassProperties,
}

impl RigidBodyState {
    /// Construct from explicit parts.
    #[must_use]
    pub fn new(
        time: SimTime,
        position: Position3<Eci>,
        velocity: Velocity3<Eci>,
        orientation: Quaternion<Body, Eci>,
        angular_velocity: AngularVelocity3<Body>,
        mass_props: MassProperties,
    ) -> Self {
        Self {
            time,
            position,
            velocity,
            orientation,
            angular_velocity,
            mass_props,
        }
    }

    /// Returns `true` if every numeric component is finite. Does
    /// **not** check quaternion normalisation or structural mass
    /// property validity; use [`RigidBodyState::is_valid`] for that.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.time.is_finite()
            && self.position.is_finite()
            && self.velocity.is_finite()
            && self.orientation.is_finite()
            && self.angular_velocity.is_finite()
            && self.mass_props.is_finite()
    }

    /// Returns `true` if the orientation quaternion is normalised
    /// within `tolerance` of unit magnitude.
    #[must_use]
    pub fn is_normalised(&self, tolerance: f64) -> bool {
        self.orientation.is_normalised(tolerance)
    }

    /// Returns `true` if all rigid-body state invariants hold.
    #[must_use]
    pub fn is_valid(&self, quaternion_tolerance: f64, inertia_symmetry_tolerance: f64) -> bool {
        self.require_valid(quaternion_tolerance, inertia_symmetry_tolerance)
            .is_ok()
    }

    /// Validate the state.
    ///
    /// Checks, in order:
    ///
    /// 1. Time validity.
    /// 2. Position/velocity finite.
    /// 3. Angular velocity finite.
    /// 4. Orientation normalised within `quaternion_tolerance`.
    /// 5. Mass-properties valid (delegates to
    ///    [`MassProperties::require_valid`]).
    ///
    /// # Errors
    ///
    /// Returns the first failed check as a [`StateError`] variant.
    pub fn require_valid(
        &self,
        quaternion_tolerance: f64,
        inertia_symmetry_tolerance: f64,
    ) -> Result<(), StateError> {
        self.time.require_valid()?;
        self.position.require_finite()?;
        self.velocity.require_finite()?;
        self.angular_velocity.require_finite()?;
        self.orientation.require_normalised(quaternion_tolerance)?;
        self.mass_props.require_valid(inertia_symmetry_tolerance)?;
        Ok(())
    }
}

/// Project a rigid-body state down to a point-mass state by dropping
/// orientation, angular velocity, and the inertia tensor (mass alone
/// is preserved).
impl From<&RigidBodyState> for PointMassState {
    fn from(rb: &RigidBodyState) -> Self {
        Self {
            time: rb.time,
            position: rb.position,
            velocity: rb.velocity,
            mass: rb.mass_props.mass,
        }
    }
}

impl PointMassState {
    /// Promote a point-mass state to a rigid-body state by supplying
    /// the missing rigid-body fields.
    #[must_use]
    pub fn into_rigid_body(
        self,
        orientation: Quaternion<Body, Eci>,
        angular_velocity: AngularVelocity3<Body>,
        mass_properties: MassProperties,
    ) -> RigidBodyState {
        RigidBodyState {
            time: self.time,
            position: self.position,
            velocity: self.velocity,
            orientation,
            angular_velocity,
            mass_props: mass_properties,
        }
    }

    /// Promote a point-mass state to a rigid-body state and validate
    /// the result before returning it.
    ///
    /// # Errors
    ///
    /// Returns [`StateError`] if the supplied rigid-body fields or the
    /// carried point-mass fields violate state invariants.
    pub fn try_into_rigid_body(
        self,
        orientation: Quaternion<Body, Eci>,
        angular_velocity: AngularVelocity3<Body>,
        mass_properties: MassProperties,
        quaternion_tolerance: f64,
        inertia_symmetry_tolerance: f64,
    ) -> Result<RigidBodyState, StateError> {
        let state = self.into_rigid_body(orientation, angular_velocity, mass_properties);
        state.require_valid(quaternion_tolerance, inertia_symmetry_tolerance)?;
        Ok(state)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra::Quaternion as NalgebraQuaternion;
    use openbmp_core::{Position3, UnitQuaternion, Velocity3};
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    fn one_kg() -> Mass {
        Mass::new::<kilogram>(1.0)
    }

    fn unit_inertia_props() -> MassProperties {
        MassProperties::with_diagonal_inertia(one_kg(), Position3::origin(), 1.0, 1.0, 1.0)
    }

    fn canonical_state() -> RigidBodyState {
        RigidBodyState::new(
            SimTime::ZERO,
            Position3::new(7000.0, 0.0, 0.0),
            Velocity3::new(0.0, 7500.0, 0.0),
            Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
            AngularVelocity3::zero(),
            unit_inertia_props(),
        )
    }

    #[test]
    fn require_valid_accepts_canonical() {
        let s = canonical_state();
        assert!(s.is_finite());
        assert!(s.is_normalised(1.0e-12));
        assert!(s.require_valid(1.0e-12, 1.0e-12).is_ok());
        assert!(s.is_valid(1.0e-12, 1.0e-12));
    }

    #[test]
    fn project_to_point_mass_drops_attitude() {
        let s = canonical_state();
        let pm: PointMassState = (&s).into();
        assert_abs_diff_eq!(pm.time.as_seconds(), s.time.as_seconds());
        assert_abs_diff_eq!(pm.mass_kg(), 1.0);
        assert_abs_diff_eq!(pm.position.vector.x, 7000.0);
        assert_abs_diff_eq!(pm.velocity.vector.y, 7500.0);
    }

    #[test]
    fn promote_point_mass_round_trip() {
        let pm = PointMassState::new(
            SimTime::from_seconds(2.5),
            Position3::new(1.0, 2.0, 3.0),
            Velocity3::new(0.1, 0.2, 0.3),
            one_kg(),
        );
        let rb = pm.into_rigid_body(
            Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
            AngularVelocity3::zero(),
            unit_inertia_props(),
        );
        let back: PointMassState = (&rb).into();
        assert_abs_diff_eq!(back.time.as_seconds(), 2.5);
        assert_abs_diff_eq!(back.position.vector.x, 1.0);
        assert_abs_diff_eq!(back.velocity.vector.z, 0.3);
        assert_abs_diff_eq!(back.mass_kg(), 1.0);
    }

    #[test]
    fn try_promote_point_mass_validates_result() {
        let pm = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            one_kg(),
        );
        let rb = pm
            .try_into_rigid_body(
                Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
                AngularVelocity3::zero(),
                unit_inertia_props(),
                1.0e-12,
                1.0e-12,
            )
            .unwrap();
        assert!(rb.require_valid(1.0e-12, 1.0e-12).is_ok());
    }

    #[test]
    fn require_valid_rejects_nan_angular_velocity() {
        let mut s = canonical_state();
        s.angular_velocity = AngularVelocity3::new(f64::NAN, 0.0, 0.0);
        let err = s.require_valid(1.0e-12, 1.0e-12).unwrap_err();
        assert!(matches!(err, StateError::Frame(_)));
    }

    #[test]
    fn require_valid_rejects_invalid_quaternion_tolerance() {
        let s = canonical_state();
        let err = s.require_valid(-0.1, 0.0).unwrap_err();
        assert!(matches!(err, StateError::Frame(_)));
    }

    #[test]
    fn require_valid_rejects_unnormalised_quaternion() {
        let mut s = canonical_state();
        s.orientation = Quaternion::<Body, Eci>::from_unit_quaternion(
            UnitQuaternion::new_unchecked(NalgebraQuaternion::new(2.0, 0.0, 0.0, 0.0)),
        );
        let err = s.require_valid(1.0e-12, 1.0e-12).unwrap_err();
        assert!(matches!(err, StateError::Frame(_)));
    }

    #[test]
    fn require_valid_rejects_nonfinite_quaternion() {
        let mut s = canonical_state();
        s.orientation = Quaternion::<Body, Eci>::from_unit_quaternion(
            UnitQuaternion::new_unchecked(NalgebraQuaternion::new(f64::NAN, 0.0, 0.0, 0.0)),
        );
        let err = s.require_valid(1.0e-12, 1.0e-12).unwrap_err();
        assert!(matches!(err, StateError::Frame(_)));
    }

    #[test]
    fn from_wxyz_checked_rejects_non_unit_quaternion() {
        let result = Quaternion::<Body, Eci>::from_wxyz_checked(2.0, 0.0, 0.0, 0.0, 1.0e-9);
        assert!(result.is_err());
    }

    #[test]
    fn from_wxyz_checked_accepts_unit_quaternion() {
        let result = Quaternion::<Body, Eci>::from_wxyz_checked(1.0, 0.0, 0.0, 0.0, 1.0e-9);
        assert!(result.is_ok());
    }
}
