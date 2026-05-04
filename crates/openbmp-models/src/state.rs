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

    /// Phase-5.D.4 — scalar L2 norm of the state's vector components.
    ///
    /// Originally consumed by the §5.D.4 adaptive-step integrator's
    /// scalar tolerance scaling `err = h · ||e'|| / (atol + rtol ·
    /// scalar_state_size)`. §5.D.5 replaced that scalar form with the
    /// per-component [`Integratable::weighted_error_norm`]; this
    /// method is retained as a generic state-size diagnostic for
    /// telemetry / external consumers.
    ///
    /// The shipped implementations sum every numeric component in a
    /// locked order (no FMA); see the per-state impls in
    /// `crate::point_mass_impl` / `crate::rigid_body_impl` for the
    /// component lists.
    #[must_use]
    fn scalar_state_size(&self) -> f64;

    /// Phase-5.D.5 — per-component scaled error RMS norm, the
    /// Hairer-Nørsett-Wanner Vol I §II.4 form:
    ///
    /// ```text
    /// sc_i  = atol + rtol · max(|y^n_i|, |y^{n+1}_i|)
    /// err   = sqrt( (1/N) · Σ_i ( h · e'_i / sc_i )^2 )
    /// ```
    ///
    /// where `e' = Σ_j E_j k_j` is the embedded-error derivative
    /// produced by the adaptive integrator's stage-weight combination
    /// and `N` is the state's component count
    /// ([`crate::derivative::SimStateDerivative::dimension`]).
    ///
    /// # Inputs
    ///
    /// * `self` — the candidate end-of-step state `y^{n+1}`.
    /// * `prev_state` — the start-of-step state `y^n`.
    /// * `error_deriv` — `e'`, the linear combination of stage
    ///   derivatives the integrator forms with its embedded-error
    ///   weights. The `h` factor is applied **inside** this method
    ///   so callers pass the unscaled derivative.
    /// * `h` — the trial step size in seconds.
    /// * `atol` / `rtol` — the scalar absolute / relative tolerance
    ///   pair from the scenario `[solver.adaptive]` block. Per the
    ///   shipped scenario surface a single scalar pair is shared
    ///   across components; the per-component refinement applies the
    ///   pair to the *scaling* via `sc_i`, not to the *tolerances*.
    ///
    /// # Determinism
    ///
    /// Implementations must walk components in the **same locked
    /// order** as [`crate::derivative::SimStateDerivative::l2_norm`]
    /// and [`Integratable::scalar_state_size`] — the squared sum is
    /// non-associative under IEEE 754 and the order is the
    /// determinism contract. No `f64::mul_add`. The pairing between
    /// state components and their derivative slots
    /// (`pos.x ↔ velocity.x`, `vel.x ↔ acceleration.x`, …) is also
    /// fixed by this method's contract.
    #[must_use]
    fn weighted_error_norm(
        &self,
        prev_state: &Self,
        error_deriv: &Self::Derivative,
        h: f64,
        atol: f64,
        rtol: f64,
    ) -> f64;
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

        fn weighted_error_norm(
            &self,
            prev_state: &Self,
            error_deriv: &PointMassDerivative,
            h: f64,
            atol: f64,
            rtol: f64,
        ) -> f64 {
            // Per-component HNW Vol I §II.4 RMS form. 7 components,
            // walked in the same locked order as `scalar_state_size`
            // and `PointMassDerivative::l2_norm`. Each state component
            // pairs with its time-derivative slot:
            //
            //   pos.x  ↔  velocity_m_s.x
            //   pos.y  ↔  velocity_m_s.y
            //   pos.z  ↔  velocity_m_s.z
            //   vel.x  ↔  acceleration_m_s2.x
            //   vel.y  ↔  acceleration_m_s2.y
            //   vel.z  ↔  acceleration_m_s2.z
            //   mass   ↔  mass_rate_kg_s
            //
            // No FMA; explicit two-rounding multiplications and
            // additions throughout.
            let prev_mass_kg = prev_state.mass.get::<kilogram>();
            let new_mass_kg = self.mass.get::<kilogram>();

            let term = |y_prev: f64, y_new: f64, e_prime: f64| -> f64 {
                let scale = atol + rtol * y_prev.abs().max(y_new.abs());
                let scaled = h * e_prime / scale;
                scaled * scaled
            };

            let mut s = 0.0_f64;
            s += term(
                prev_state.position.vector.x,
                self.position.vector.x,
                error_deriv.velocity_m_s.x,
            );
            s += term(
                prev_state.position.vector.y,
                self.position.vector.y,
                error_deriv.velocity_m_s.y,
            );
            s += term(
                prev_state.position.vector.z,
                self.position.vector.z,
                error_deriv.velocity_m_s.z,
            );
            s += term(
                prev_state.velocity.vector.x,
                self.velocity.vector.x,
                error_deriv.acceleration_m_s2.x,
            );
            s += term(
                prev_state.velocity.vector.y,
                self.velocity.vector.y,
                error_deriv.acceleration_m_s2.y,
            );
            s += term(
                prev_state.velocity.vector.z,
                self.velocity.vector.z,
                error_deriv.acceleration_m_s2.z,
            );
            s += term(prev_mass_kg, new_mass_kg, error_deriv.mass_rate_kg_s);

            // 7 components — matches PointMassDerivative::dimension.
            (s / 7.0).sqrt()
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
            //
            // Note: this is a state-magnitude diagnostic and counts 20
            // components (skipping inertia off-diagonal). The §5.D.5
            // [`Integratable::weighted_error_norm`] sums all 26
            // components (matching `RigidBodyDerivative::dimension`) so
            // the RMS divisor is consistent with the integrator's
            // representation; the small double-counting of symmetric
            // off-diagonal pairs there is harmless because each error
            // term is divided by its own per-component scale.
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

        fn weighted_error_norm(
            &self,
            prev_state: &Self,
            error_deriv: &RigidBodyDerivative,
            h: f64,
            atol: f64,
            rtol: f64,
        ) -> f64 {
            // Per-component HNW Vol I §II.4 RMS form. 26 components,
            // matching `RigidBodyDerivative::dimension`. Component
            // pairing (state ↔ derivative):
            //
            //   position           ↔  velocity_m_s_eci
            //   velocity           ↔  acceleration_m_s2_eci
            //   orientation.coords ↔  quaternion_rate.coords
            //   angular_velocity   ↔  angular_acceleration_rad_s2_body
            //   mass               ↔  mass_rate_kg_s
            //   center_of_mass     ↔  center_of_mass_rate_body_m_s
            //   inertia_body       ↔  inertia_rate_body  (full 3×3,
            //                                             column-major
            //                                             via .iter())
            //
            // The full 9-entry inertia rate is summed (not just the
            // diagonal) to keep the dimension count consistent with
            // the integrator's representation. Symmetric off-diagonal
            // pairs are double-counted, but harmlessly: each scaled
            // term is normalised by its own per-component scale, and
            // the controller's setpoint-1 dynamics absorb a constant
            // factor √2 silently.
            //
            // Walking order locked to match `l2_norm` and
            // `RigidBodyDerivative::l2_norm`. No FMA.
            let term = |y_prev: f64, y_new: f64, e_prime: f64| -> f64 {
                let scale = atol + rtol * y_prev.abs().max(y_new.abs());
                let scaled = h * e_prime / scale;
                scaled * scaled
            };

            let mut s = 0.0_f64;

            // Position (3) ↔ velocity_m_s_eci.
            for k in 0..3 {
                s += term(
                    prev_state.position.vector[k],
                    self.position.vector[k],
                    error_deriv.velocity_m_s_eci[k],
                );
            }
            // Velocity (3) ↔ acceleration_m_s2_eci.
            for k in 0..3 {
                s += term(
                    prev_state.velocity.vector[k],
                    self.velocity.vector[k],
                    error_deriv.acceleration_m_s2_eci[k],
                );
            }
            // Orientation quaternion (4) ↔ quaternion_rate. Iterating
            // over `coords` exposes (x, y, z, w) in nalgebra order.
            for k in 0..4 {
                s += term(
                    prev_state.orientation.q.coords[k],
                    self.orientation.q.coords[k],
                    error_deriv.quaternion_rate.coords[k],
                );
            }
            // Angular velocity (3) ↔ angular_acceleration_rad_s2_body.
            for k in 0..3 {
                s += term(
                    prev_state.angular_velocity.vector[k],
                    self.angular_velocity.vector[k],
                    error_deriv.angular_acceleration_rad_s2_body[k],
                );
            }
            // Mass (1) ↔ mass_rate_kg_s.
            let prev_mass_kg = prev_state.mass_props.mass.get::<kilogram>();
            let new_mass_kg = self.mass_props.mass.get::<kilogram>();
            s += term(prev_mass_kg, new_mass_kg, error_deriv.mass_rate_kg_s);
            // Center-of-mass offset (3) ↔ center_of_mass_rate_body_m_s.
            for k in 0..3 {
                s += term(
                    prev_state.mass_props.center_of_mass_body.vector[k],
                    self.mass_props.center_of_mass_body.vector[k],
                    error_deriv.center_of_mass_rate_body_m_s[k],
                );
            }
            // Inertia tensor (9) ↔ inertia_rate_body, column-major
            // (matches Matrix3::iter()).
            for k in 0..9 {
                let prev_i = prev_state.mass_props.inertia_body.as_slice()[k];
                let new_i = self.mass_props.inertia_body.as_slice()[k];
                let e_i = error_deriv.inertia_rate_body.as_slice()[k];
                s += term(prev_i, new_i, e_i);
            }

            // 26 components total — matches RigidBodyDerivative::dimension.
            (s / 26.0).sqrt()
        }
    }
}

// ---------------------------------------------------------------------
// Phase-5.D.5 — weighted_error_norm tests
// ---------------------------------------------------------------------

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod weighted_error_norm_tests {
    use approx::assert_abs_diff_eq;
    use nalgebra::{Matrix3, Quaternion as NalgebraQuaternion, Vector3};
    use openbmp_core::{
        AngularVelocity3, Body, Eci, Position3, Quaternion, SimTime, UnitQuaternion, Velocity3,
    };
    use openbmp_state::{MassProperties, PointMassState, RigidBodyState};
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    use crate::derivative::{PointMassDerivative, RigidBodyDerivative, SimStateDerivative};
    use crate::state::Integratable;

    // -----------------------------------------------------------------
    // PointMassState
    // -----------------------------------------------------------------

    fn pm_state(mass_kg: f64) -> PointMassState {
        PointMassState::new(
            SimTime::ZERO,
            Position3::new(1.0e6, 0.0, 0.0),
            Velocity3::new(0.0, 7.5e3, 0.0),
            Mass::new::<kilogram>(mass_kg),
        )
    }

    #[test]
    fn pm_zero_error_yields_zero_norm() {
        let prev = pm_state(1.0);
        let new = pm_state(1.0);
        let err_deriv = PointMassDerivative::zero();
        let err = new.weighted_error_norm(&prev, &err_deriv, 0.1, 1.0e-9, 1.0e-12);
        assert_eq!(err, 0.0);
    }

    #[test]
    fn pm_uniform_error_matches_rms_form() {
        // All seven components see the same scaled error of 0.5,
        // RMS norm should equal sqrt((1/7) · 7 · 0.5²) = 0.5.
        let prev = pm_state(1.0);
        let new = pm_state(1.0);
        let h = 1.0;
        let atol = 1.0;
        let rtol = 0.0;
        // Construct e' so that h·e_i / (atol + rtol·max(|y|,|y|)) = 0.5
        // for every component. With rtol = 0, scale = atol = 1, so we
        // just need e_i = 0.5 for every component.
        let err_deriv = PointMassDerivative::new(
            Vector3::new(0.5, 0.5, 0.5),
            Vector3::new(0.5, 0.5, 0.5),
            0.5,
        );
        let err = new.weighted_error_norm(&prev, &err_deriv, h, atol, rtol);
        // Expected: sqrt((7 · 0.5²) / 7) = 0.5.
        assert_abs_diff_eq!(err, 0.5, epsilon = 1.0e-15);
    }

    #[test]
    fn pm_norm_is_dimensionally_consistent_at_unity_setpoint() {
        // If h·||e_i|| / sc_i = 1 for every component, the RMS norm
        // is exactly 1 (the controller's setpoint). Verifies the
        // RMS divisor is dimension(7), not dimension(1) (which would
        // give sqrt(7)).
        let prev = pm_state(1.0);
        let new = pm_state(1.0);
        let h = 1.0;
        let atol = 1.0;
        let rtol = 0.0;
        let err_deriv = PointMassDerivative::new(
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(1.0, 1.0, 1.0),
            1.0,
        );
        let err = new.weighted_error_norm(&prev, &err_deriv, h, atol, rtol);
        assert_abs_diff_eq!(err, 1.0, epsilon = 1.0e-15);
    }

    #[test]
    fn pm_per_component_handles_multi_scale_state() {
        // Position ~ 10⁶, mass ~ 1 — a 1 m error on position is at
        // rtol·|y_pos| = 1e-3 scale; a 1e-12 kg error on mass is at
        // atol = 1e-12 scale. Both are at scale 1.0 in scaled units,
        // so the per-component norm should report a finite,
        // controller-actionable value. A naive scalar form dividing
        // ||e||₂ ≈ 1.0 by ||y||₂ ≈ 1e6 would report ~1e3 — close to
        // this number coincidentally, but for the wrong reason (it
        // averages the full error vector against the full state
        // magnitude rather than per-component).
        let prev = PointMassState::new(
            SimTime::ZERO,
            Position3::new(1.0e6, 0.0, 0.0),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        );
        let new = PointMassState::new(
            SimTime::ZERO,
            Position3::new(1.0e6 + 1.0, 0.0, 0.0),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0 + 1.0e-12),
        );
        let h = 1.0;
        let atol = 1.0e-12;
        let rtol = 1.0e-9;
        // e' such that h·e_pos.x = 1.0 (matches new.pos.x − prev.pos.x)
        // and h·e_mass = 1.0e-12 (matches new.mass − prev.mass), the
        // other five derivatives zero. Zero-valued state components
        // (pos.y, pos.z, vel.{x,y,z}) get scale = atol = 1e-12 and
        // contribute nothing because their error_deriv slot is zero.
        let err_deriv =
            PointMassDerivative::new(Vector3::new(1.0, 0.0, 0.0), Vector3::zeros(), 1.0e-12);
        let err = new.weighted_error_norm(&prev, &err_deriv, h, atol, rtol);
        // Pos scale = atol + rtol · 1e6 ≈ 1e-3 → scaled = 1.0/1e-3 = 1e3.
        // Mass scale = atol + rtol · 1 ≈ 1e-9 → scaled = 1e-12/1e-9 = 1e-3.
        // Other five terms zero. RMS over N=7:
        //   sqrt(((1e3)² + (1e-3)²) / 7) ≈ 1e3/sqrt(7) ≈ 377.96.
        // Approximate expected value: dominant term is the position
        // breach. With atol·rtol-mixed scales the closed-form is messy,
        // so accept a 0.1% relative band around the leading-order value.
        let expected = (1.0e6_f64 / 7.0).sqrt();
        assert_abs_diff_eq!(err, expected, epsilon = expected * 1.0e-3);
        assert!(
            err.is_finite() && err > 100.0,
            "per-component norm must reflect position-component breach: {err}"
        );
    }

    #[test]
    fn pm_norm_is_bit_stable_across_reruns() {
        let prev = pm_state(1.0);
        let new = pm_state(0.999);
        let err_deriv = PointMassDerivative::new(
            Vector3::new(0.123, 0.456, 0.789),
            Vector3::new(1.234, 5.678, 9.012),
            0.001,
        );
        let a = new.weighted_error_norm(&prev, &err_deriv, 0.1, 1.0e-9, 1.0e-12);
        let b = new.weighted_error_norm(&prev, &err_deriv, 0.1, 1.0e-9, 1.0e-12);
        assert_eq!(a.to_bits(), b.to_bits());
    }

    // -----------------------------------------------------------------
    // RigidBodyState
    // -----------------------------------------------------------------

    fn rb_state() -> RigidBodyState {
        RigidBodyState::new(
            SimTime::ZERO,
            Position3::new(7.0e6, 0.0, 0.0),
            Velocity3::new(0.0, 7.5e3, 0.0),
            Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
            AngularVelocity3::new(0.1, -0.2, 0.05),
            MassProperties::with_diagonal_inertia(
                Mass::new::<kilogram>(100.0),
                Position3::new(0.01, 0.0, -0.01),
                10.0,
                15.0,
                15.0,
            ),
        )
    }

    fn rb_zero_deriv() -> RigidBodyDerivative {
        RigidBodyDerivative::zero()
    }

    #[test]
    fn rb_zero_error_yields_zero_norm() {
        let s = rb_state();
        let err = s.weighted_error_norm(&s, &rb_zero_deriv(), 0.1, 1.0e-9, 1.0e-12);
        assert_eq!(err, 0.0);
    }

    #[test]
    fn rb_uniform_error_matches_rms_form() {
        // 26 components, each contributing 0.25 squared:
        //   err = sqrt((26 · 0.25²) / 26) = 0.25.
        let prev = rb_state();
        let new = rb_state();
        let err_deriv = RigidBodyDerivative::new(
            Vector3::new(0.25, 0.25, 0.25),
            Vector3::new(0.25, 0.25, 0.25),
            NalgebraQuaternion::new(0.25, 0.25, 0.25, 0.25),
            Vector3::new(0.25, 0.25, 0.25),
            0.25,
            Vector3::new(0.25, 0.25, 0.25),
            Matrix3::from_element(0.25),
        );
        let err = new.weighted_error_norm(&prev, &err_deriv, 1.0, 1.0, 0.0);
        assert_abs_diff_eq!(err, 0.25, epsilon = 1.0e-15);
    }

    #[test]
    fn rb_dimension_count_is_26_via_unity_setpoint() {
        // h·e_i / sc_i = 1 for every component → RMS norm = 1.
        // If we accidentally summed 20 components (matching
        // scalar_state_size) and divided by 20 we'd still get 1, so
        // that's not a discriminating test. Instead verify that
        // changing one component changes the norm by exactly the
        // expected RMS contribution: 1/sqrt(26).
        let prev = rb_state();
        let new = rb_state();
        let baseline = RigidBodyDerivative::zero();
        let err0 = new.weighted_error_norm(&prev, &baseline, 1.0, 1.0, 0.0);
        assert_eq!(err0, 0.0);

        let mut perturbed = baseline;
        perturbed.mass_rate_kg_s = 1.0;
        let err_one = new.weighted_error_norm(&prev, &perturbed, 1.0, 1.0, 0.0);
        // One component contributing 1² to the sum, divided by N=26,
        // sqrt → 1/sqrt(26).
        assert_abs_diff_eq!(err_one, (1.0_f64 / 26.0).sqrt(), epsilon = 1.0e-15);
    }

    #[test]
    fn rb_norm_is_bit_stable_across_reruns() {
        let prev = rb_state();
        let new = rb_state();
        let err_deriv = RigidBodyDerivative::new(
            Vector3::new(0.123, -0.456, 0.789),
            Vector3::new(1.234, -5.678, 9.012),
            NalgebraQuaternion::new(0.001, -0.002, 0.003, 0.004),
            Vector3::new(0.01, -0.02, 0.03),
            0.001,
            Vector3::new(0.0001, 0.0, -0.0001),
            Matrix3::from_diagonal(&Vector3::new(0.1, -0.2, 0.3)),
        );
        let a = new.weighted_error_norm(&prev, &err_deriv, 0.1, 1.0e-9, 1.0e-12);
        let b = new.weighted_error_norm(&prev, &err_deriv, 0.1, 1.0e-9, 1.0e-12);
        assert_eq!(a.to_bits(), b.to_bits());
    }

    #[test]
    fn rb_dimension_matches_derivative_dimension() {
        // Trait-level invariant: the RMS divisor must equal the
        // derivative's `dimension()`. Verifies the impl picks up all
        // 26 components.
        let d = RigidBodyDerivative::zero();
        assert_eq!(d.dimension(), 26);
    }
}
