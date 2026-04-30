//! `SimState` trait — the data shape an integrator advances.
//!
//! Phase-3.14.A: extracted from `openbmp-sim` so model trait surfaces
//! can implement / consume `SimState` without depending on the
//! integrator. The integrator algorithm itself (in `openbmp-sim`)
//! still consumes this trait.

use openbmp_core::SimTime;

use crate::derivative::SimStateDerivative;

/// State that an integrator can advance.
///
/// Implementations supply the linear-arithmetic glue and an optional
/// post-step manifold projection hook.
pub trait SimState: Copy + std::fmt::Debug {
    /// Time-derivative type for this state.
    type Derivative: SimStateDerivative;

    /// The state's current simulation time.
    #[must_use]
    fn time(&self) -> SimTime;

    /// Returns `true` if every numeric component is finite.
    #[must_use]
    fn is_finite(&self) -> bool;

    /// Returns `true` if the state is valid for integration.
    ///
    /// The default only checks finiteness. State implementations with
    /// structural invariants, such as strictly-positive mass, should
    /// override this method.
    #[must_use]
    fn is_valid_for_integration(&self) -> bool {
        self.is_finite()
    }

    /// Returns a copy of this state with its time field replaced by
    /// `t`. Used by the kernel to overwrite the integrator's accumulated
    /// time with the canonical `start + step * dt` value, eliminating
    /// O(N · ε) drift.
    #[must_use]
    fn with_time(self, t: SimTime) -> Self;

    /// Compute `state + h_seconds * derivative` componentwise. Time
    /// advances by `h_seconds * 1.0 = h_seconds` (the implicit
    /// time-rate is unity).
    #[must_use]
    fn advance_by(&self, h_seconds: f64, derivative: &Self::Derivative) -> Self;

    /// Post-integration manifold projection. Default no-op for
    /// unconstrained states. Rigid-body implementations override to
    /// renormalise the orientation quaternion.
    fn project(&mut self) {}
}

// ---------------------------------------------------------------------
// SimState impl for openbmp_state::PointMassState
// ---------------------------------------------------------------------

mod point_mass_impl {
    use openbmp_core::{Position3, SimTime, Velocity3};
    use openbmp_state::PointMassState;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    use crate::derivative::PointMassDerivative;

    use super::SimState;

    impl SimState for PointMassState {
        type Derivative = PointMassDerivative;

        fn time(&self) -> SimTime {
            self.time
        }

        fn is_finite(&self) -> bool {
            PointMassState::is_finite(self)
        }

        fn is_valid_for_integration(&self) -> bool {
            self.require_valid().is_ok()
        }

        fn with_time(mut self, t: SimTime) -> Self {
            self.time = t;
            self
        }

        fn advance_by(&self, h_seconds: f64, derivative: &PointMassDerivative) -> Self {
            // DETERMINISM: explicit `(scaled) + base` order; no FMA.
            let new_time = SimTime::from_seconds(self.time.as_seconds() + h_seconds);
            let new_position =
                Position3::from_vector(self.position.vector + h_seconds * derivative.velocity_m_s);
            let new_velocity = Velocity3::from_vector(
                self.velocity.vector + h_seconds * derivative.acceleration_m_s2,
            );
            let new_mass_kg = self.mass.get::<kilogram>() + h_seconds * derivative.mass_rate_kg_s;
            let new_mass = Mass::new::<kilogram>(new_mass_kg);
            Self::new(new_time, new_position, new_velocity, new_mass)
        }

        // `project()` defaults to no-op for PointMassState; an
        // unconstrained 3-DOF state has no manifold to project onto.
    }
}

// ---------------------------------------------------------------------
// SimState impl for openbmp_state::RigidBodyState
// ---------------------------------------------------------------------

mod rigid_body_impl {
    use nalgebra::Quaternion as NalgebraQuaternion;
    use openbmp_core::{
        AngularVelocity3, Position3, Quaternion, SimTime, UnitQuaternion, Velocity3,
    };
    use openbmp_state::{MassProperties, RigidBodyState};
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    use crate::derivative::RigidBodyDerivative;

    use super::SimState;

    /// Tolerances used when `RigidBodyState` is treated as valid for
    /// RK4 sub-step purposes. The integrator accepts intermediate
    /// states with mildly non-unit quaternions; the kernel's post-step
    /// validation uses much tighter tolerances.
    const SUBSTEP_QUATERNION_TOL: f64 = 1.0e-2;
    const SUBSTEP_INERTIA_SYMMETRY_TOL: f64 = 1.0e-6;

    impl SimState for RigidBodyState {
        type Derivative = RigidBodyDerivative;

        fn time(&self) -> SimTime {
            self.time
        }

        fn is_finite(&self) -> bool {
            RigidBodyState::is_finite(self)
        }

        fn is_valid_for_integration(&self) -> bool {
            self.require_valid(SUBSTEP_QUATERNION_TOL, SUBSTEP_INERTIA_SYMMETRY_TOL)
                .is_ok()
        }

        fn with_time(mut self, t: SimTime) -> Self {
            self.time = t;
            self
        }

        fn advance_by(&self, h_seconds: f64, d: &RigidBodyDerivative) -> Self {
            // DETERMINISM: explicit `(scaled) + base` order on every
            // component; single multiplication; no FMA.

            let new_time = SimTime::from_seconds(self.time.as_seconds() + h_seconds);
            let new_position =
                Position3::from_vector(self.position.vector + h_seconds * d.velocity_m_s_eci);
            let new_velocity =
                Velocity3::from_vector(self.velocity.vector + h_seconds * d.acceleration_m_s2_eci);

            // Linear-sum the quaternion's underlying coords. Result is
            // intentionally non-unit; project() restores the manifold
            // constraint after the final weighted sum.
            let q_new_coords = self.orientation.q.coords + h_seconds * d.quaternion_rate.coords;
            let q_new_raw = NalgebraQuaternion::from(q_new_coords);
            let new_orientation =
                Quaternion::<openbmp_core::Body, openbmp_core::Eci>::from_unit_quaternion(
                    UnitQuaternion::new_unchecked(q_new_raw),
                );

            let new_angular_velocity = AngularVelocity3::from_vector(
                self.angular_velocity.vector + h_seconds * d.angular_acceleration_rad_s2_body,
            );

            let mass_kg = self.mass_props.mass.get::<kilogram>();
            let new_mass_kg = mass_kg + h_seconds * d.mass_rate_kg_s;
            let new_mass = Mass::new::<kilogram>(new_mass_kg);
            let new_center_of_mass = Position3::from_vector(
                self.mass_props.center_of_mass_body.vector
                    + h_seconds * d.center_of_mass_rate_body_m_s,
            );
            let new_inertia = self.mass_props.inertia_body + h_seconds * d.inertia_rate_body;
            let new_mass_props = MassProperties::new(new_mass, new_center_of_mass, new_inertia);

            Self::new(
                new_time,
                new_position,
                new_velocity,
                new_orientation,
                new_angular_velocity,
                new_mass_props,
            )
        }

        fn project(&mut self) {
            // Quaternion renormalisation. Single sqrt + four divisions;
            // no rotation matrix construction.
            let q = self.orientation.q.into_inner();
            let n2 = q.coords.x * q.coords.x
                + q.coords.y * q.coords.y
                + q.coords.z * q.coords.z
                + q.coords.w * q.coords.w;
            if n2 > 0.0 {
                let inv = 1.0 / n2.sqrt();
                let normalised = NalgebraQuaternion::from(q.coords * inv);
                self.orientation =
                    Quaternion::<openbmp_core::Body, openbmp_core::Eci>::from_unit_quaternion(
                        UnitQuaternion::new_unchecked(normalised),
                    );
            }
        }
    }
}
