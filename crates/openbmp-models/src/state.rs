//! `VehicleState` (data shape) + `Integratable` (integration extension).
//!
//! Phase-3.15.E split the integrator-shaped `SimState` trait into two:
//!
//! * [`VehicleState`] — the pure base state contract: a time stamp,
//!   finiteness, and a way to overwrite the time field.
//! * [`TranslationalState`] / [`RigidBodyKinematicState`] — read-only
//!   snapshot accessors for controller / telemetry consumers that need
//!   position, velocity, mass, attitude, or angular rate without the
//!   integration extension.
//! * [`Integratable`] — the integration extension. Adds the
//!   derivative type, the `advance_by` step combinator, the
//!   `is_valid_for_integration` validity check, and the
//!   post-step `project()` manifold hook. The integrator
//!   (`openbmp_sim::Rk4FixedStep` today, future adaptive variants
//!   in Phase 5+) consumes this.
//!
//! The legacy [`SimState`] name persists as a marker that requires
//! both — every existing `<S: SimState>` bound keeps compiling.
//! New code that only needs the data shape (e.g., a controller's
//! state-snapshot consumer) bounds on the narrowest snapshot trait it
//! needs.

use openbmp_core::SimTime;

use crate::derivative::SimStateDerivative;

// ---------------------------------------------------------------------
// VehicleState — base state shape
// ---------------------------------------------------------------------

/// Hardware-portable base contract for vehicle state values.
///
/// Phase-3.15.E extracted this from the integrator-shaped
/// [`SimState`] so a controller that *receives* state snapshots from
/// the runner / HAL doesn't have to satisfy the integrator's
/// `advance_by` / `project` / derivative-type contract. A real
/// flight controller implementing generic timestamp handling or
/// state-history validity checks depends on `VehicleState` only; code
/// that needs kinematics should use [`TranslationalState`] or
/// [`RigidBodyKinematicState`].
pub trait VehicleState: Copy + std::fmt::Debug {
    /// The state's current monotonic timestamp.
    #[must_use]
    fn time(&self) -> SimTime;

    /// Returns `true` if every numeric component is finite.
    #[must_use]
    fn is_finite(&self) -> bool;

    /// Returns a copy of this state with its time field replaced by
    /// `t`. Simulator code uses this to overwrite the integrator's
    /// accumulated time with the canonical `start + tick * dt` value,
    /// eliminating O(N · ε) drift.
    #[must_use]
    fn with_time(self, t: SimTime) -> Self;
}

// ---------------------------------------------------------------------
// Snapshot-reader traits
// ---------------------------------------------------------------------

/// Read-only translational state snapshot.
///
/// A Phase-4 estimator, controller, telemetry sink, or HAL adapter can
/// bound on this trait when it needs position / velocity / mass from a
/// vehicle-state snapshot but must not require integrator operations.
pub trait TranslationalState: VehicleState {
    /// Position in the inertial (`Eci`) frame.
    #[must_use]
    fn position_eci(&self) -> openbmp_core::Position3<openbmp_core::Eci>;

    /// Velocity in the inertial (`Eci`) frame.
    #[must_use]
    fn velocity_eci(&self) -> openbmp_core::Velocity3<openbmp_core::Eci>;

    /// Total vehicle mass in kilograms.
    #[must_use]
    fn mass_kg(&self) -> f64;
}

/// Read-only rigid-body kinematic state snapshot.
///
/// Consumers that need attitude or angular rate can bound on this
/// trait without depending on [`Integratable`]. The trait extends
/// [`TranslationalState`] because rigid-body snapshots also expose the
/// translational state inherited from the vehicle body.
pub trait RigidBodyKinematicState: TranslationalState {
    /// Body-to-inertial orientation quaternion.
    #[must_use]
    fn orientation_body_to_eci(
        &self,
    ) -> openbmp_core::Quaternion<openbmp_core::Body, openbmp_core::Eci>;

    /// Angular velocity expressed in the body frame.
    #[must_use]
    fn angular_velocity_body(&self) -> openbmp_core::AngularVelocity3<openbmp_core::Body>;
}

// ---------------------------------------------------------------------
// Integratable — integration extension
// ---------------------------------------------------------------------

/// Integration-side extension trait. Adds the derivative type and the
/// step combinator that an explicit Runge-Kutta family integrator
/// needs.
///
/// A consumer that only reads vehicle state (a controller, a
/// telemetry sink, a HAL adapter) depends on [`VehicleState`] alone;
/// only an integrator (the simulator's plant integrator, or a
/// controller's internal process model) needs `Integratable`.
pub trait Integratable: VehicleState {
    /// Time-derivative type for this state.
    type Derivative: SimStateDerivative;

    /// Returns `true` if the state is valid for integration.
    ///
    /// The default only checks finiteness. State implementations
    /// with structural invariants — strictly-positive mass,
    /// near-unit quaternion within a sub-step tolerance — should
    /// override this method.
    #[must_use]
    fn is_valid_for_integration(&self) -> bool {
        self.is_finite()
    }

    /// Compute `state + h_seconds * derivative` componentwise. Time
    /// advances by `h_seconds * 1.0 = h_seconds` (the implicit
    /// time-rate is unity).
    #[must_use]
    fn advance_by(&self, h_seconds: f64, derivative: &Self::Derivative) -> Self;

    /// Post-integration manifold projection. Default no-op for
    /// unconstrained states. Rigid-body implementations override to
    /// renormalise the orientation quaternion.
    fn project(&mut self) {}

    /// Phase-5.D.4 — scalar L2 norm of the state's vector components,
    /// used by the adaptive-step integrator's tolerance scaling
    /// `err = h · ||e'|| / (atol + rtol · scalar_state_size)`.
    ///
    /// The shipped implementations sum every numeric component in a
    /// locked order (no FMA); see the per-state impls in
    /// `crate::point_mass_impl` / `crate::rigid_body_impl` for the
    /// component lists.
    #[must_use]
    fn scalar_state_size(&self) -> f64;
}

// ---------------------------------------------------------------------
// SimState — back-compat marker
// ---------------------------------------------------------------------

/// Convenience marker: `<S: SimState>` means "base state shape +
/// integratable" exactly as it did before Phase-3.15.E.
///
/// New code should bound on the narrower trait it actually needs
/// (`VehicleState`, [`TranslationalState`],
/// [`RigidBodyKinematicState`], or `Integratable`). The marker is
/// retained as a permanent convenience for code that genuinely needs
/// both the state-shape and integration contracts.
pub trait SimState: VehicleState + Integratable {}
impl<T: VehicleState + Integratable> SimState for T {}

// ---------------------------------------------------------------------
// VehicleState + Integratable impl for openbmp_state::PointMassState
// ---------------------------------------------------------------------

mod point_mass_impl {
    use openbmp_core::{Position3, SimTime, Velocity3};
    use openbmp_state::PointMassState;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    use crate::derivative::PointMassDerivative;

    use super::{Integratable, TranslationalState, VehicleState};

    impl VehicleState for PointMassState {
        fn time(&self) -> SimTime {
            self.time
        }

        fn is_finite(&self) -> bool {
            PointMassState::is_finite(self)
        }

        fn with_time(mut self, t: SimTime) -> Self {
            self.time = t;
            self
        }
    }

    impl TranslationalState for PointMassState {
        fn position_eci(&self) -> Position3<openbmp_core::Eci> {
            self.position
        }

        fn velocity_eci(&self) -> Velocity3<openbmp_core::Eci> {
            self.velocity
        }

        fn mass_kg(&self) -> f64 {
            PointMassState::mass_kg(self)
        }
    }

    impl Integratable for PointMassState {
        type Derivative = PointMassDerivative;

        fn is_valid_for_integration(&self) -> bool {
            self.require_valid().is_ok()
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

        fn scalar_state_size(&self) -> f64 {
            // Locked-order squared sum: position₀..₂, velocity₀..₂, mass.
            // No FMA.
            let mut s = 0.0_f64;
            s += self.position.vector.x * self.position.vector.x;
            s += self.position.vector.y * self.position.vector.y;
            s += self.position.vector.z * self.position.vector.z;
            s += self.velocity.vector.x * self.velocity.vector.x;
            s += self.velocity.vector.y * self.velocity.vector.y;
            s += self.velocity.vector.z * self.velocity.vector.z;
            let mass_kg = self.mass.get::<kilogram>();
            s += mass_kg * mass_kg;
            s.sqrt()
        }
    }
}

// ---------------------------------------------------------------------
// VehicleState + Integratable impl for openbmp_state::RigidBodyState
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

    use super::{Integratable, RigidBodyKinematicState, TranslationalState, VehicleState};

    /// Tolerances used when `RigidBodyState` is treated as valid for
    /// RK4 sub-step purposes. The integrator accepts intermediate
    /// states with mildly non-unit quaternions; simulator post-step
    /// validation uses much tighter tolerances.
    const SUBSTEP_QUATERNION_TOL: f64 = 1.0e-2;
    const SUBSTEP_INERTIA_SYMMETRY_TOL: f64 = 1.0e-6;

    impl VehicleState for RigidBodyState {
        fn time(&self) -> SimTime {
            self.time
        }

        fn is_finite(&self) -> bool {
            RigidBodyState::is_finite(self)
        }

        fn with_time(mut self, t: SimTime) -> Self {
            self.time = t;
            self
        }
    }

    impl TranslationalState for RigidBodyState {
        fn position_eci(&self) -> Position3<openbmp_core::Eci> {
            self.position
        }

        fn velocity_eci(&self) -> Velocity3<openbmp_core::Eci> {
            self.velocity
        }

        fn mass_kg(&self) -> f64 {
            self.mass_props.mass.get::<kilogram>()
        }
    }

    impl RigidBodyKinematicState for RigidBodyState {
        fn orientation_body_to_eci(
            &self,
        ) -> openbmp_core::Quaternion<openbmp_core::Body, openbmp_core::Eci> {
            self.orientation
        }

        fn angular_velocity_body(&self) -> openbmp_core::AngularVelocity3<openbmp_core::Body> {
            self.angular_velocity
        }
    }

    impl Integratable for RigidBodyState {
        type Derivative = RigidBodyDerivative;

        fn is_valid_for_integration(&self) -> bool {
            self.require_valid(SUBSTEP_QUATERNION_TOL, SUBSTEP_INERTIA_SYMMETRY_TOL)
                .is_ok()
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

        fn scalar_state_size(&self) -> f64 {
            // Locked-order squared sum: position, velocity, quaternion
            // (4 components — magnitude ≈ 1 by manifold constraint),
            // angular velocity, mass, center-of-mass offset, inertia
            // diagonal (3 entries — off-diagonal entries are skipped to
            // avoid double-counting the symmetric structure). No FMA.
            let mut s = 0.0_f64;
            for v in self.position.vector.iter() {
                s += v * v;
            }
            for v in self.velocity.vector.iter() {
                s += v * v;
            }
            for v in self.orientation.q.coords.iter() {
                s += v * v;
            }
            for v in self.angular_velocity.vector.iter() {
                s += v * v;
            }
            let mass_kg = self.mass_props.mass.get::<kilogram>();
            s += mass_kg * mass_kg;
            for v in self.mass_props.center_of_mass_body.vector.iter() {
                s += v * v;
            }
            for i in 0..3 {
                let d = self.mass_props.inertia_body[(i, i)];
                s += d * d;
            }
            s.sqrt()
        }
    }
}
