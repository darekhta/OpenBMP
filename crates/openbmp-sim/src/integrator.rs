//! Numerical integration: [`SimState`] trait, [`Integrator`] trait,
//! and the canonical fixed-step Runge-Kutta 4 implementation
//! [`Rk4FixedStep`].
//!
//! The integrator is generic over [`SimState`]. Phase 1.3 ships only
//! the [`openbmp_state::PointMassState`] implementation; rigid-body
//! integration with quaternion attitude is deferred to a follow-on
//! sub-phase.
//!
//! # Determinism contract
//!
//! `Rk4FixedStep` is tagged [`IntegratorDeterminism::BitStable`]. The
//! contract requires:
//!
//! 1. The locked weighted-sum order of [`crate::SimStateDerivative::rk4_weighted_sum`].
//! 2. No `f64::mul_add` in the integrator hot path.
//! 3. Sub-step times computed by addition from the current step time
//!    (not by accumulating across steps — the kernel handles
//!    cross-step time via canonical multiplication).
//! 4. Post-step `project()` call to apply manifold constraints (no-op
//!    for `PointMassState`; quaternion renormalisation when rigid-body
//!    integration lands).
//!
//! See `docs/software-architecture.md § Determinism Profile` for the
//! rationale (locked weighted-sum order, no FMA, MXCSR guard).

use openbmp_core::{Duration, SimTime};

use crate::derivative::SimStateDerivative;
use crate::error::{IntegratorError, ModelEvalError};

/// State that an [`Integrator`] can advance.
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

/// Determinism class of an integrator.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum IntegratorDeterminism {
    /// Same input → bit-identical output across reruns on the same
    /// platform profile.
    BitStable,
    /// Same input → physically equivalent output, but byte-equality
    /// is not guaranteed (e.g. adaptive-step integrators where the
    /// step-size search may produce slightly different intermediate
    /// values across runs).
    StateStable,
}

/// Numerical integrator trait.
///
/// `S` is the state type the integrator advances. The derivative
/// closure `derive_fn` is called at each Runge-Kutta sub-step with the
/// intermediate state and the corresponding sub-step time. It returns
/// either the derivative or a typed model-evaluation error; the
/// integrator short-circuits before any state mutation on the first
/// failing stage.
pub trait Integrator<S: SimState> {
    /// Determinism class declared by this integrator.
    #[must_use]
    fn determinism(&self) -> IntegratorDeterminism;

    /// Advance `state` by `dt`.
    ///
    /// `derive_fn` is the user-supplied closure that computes
    /// `dy/dt` at the given (state, time) pair. The integrator may
    /// call it multiple times per step at intermediate sub-step states.
    /// A `ModelEvalError` returned by the closure short-circuits the
    /// step (no state mutation, no later RK stages run) and surfaces
    /// as [`IntegratorError::ModelEval`].
    ///
    /// # Errors
    ///
    /// Returns [`IntegratorError::InvalidStep`] if `dt` is not strictly
    /// positive and finite, [`IntegratorError::NonFiniteDerivative`] if
    /// any RK stage produces a `NaN`/`Inf` derivative,
    /// [`IntegratorError::NonFiniteState`] if any start, intermediate,
    /// or final state is not valid for integration, or
    /// [`IntegratorError::ModelEval`] if any stage's derivative
    /// closure reports a typed model failure.
    fn advance<F>(&self, state: &S, derive_fn: F, dt: Duration) -> Result<S, IntegratorError>
    where
        F: Fn(&S, SimTime) -> Result<S::Derivative, ModelEvalError>;
}

/// Canonical fixed-step Runge-Kutta 4 integrator.
///
/// 4th-order accurate, single-stage, four derivative evaluations per
/// step. Bit-stable across reruns on the same platform profile.
#[derive(Copy, Clone, Debug, Default)]
pub struct Rk4FixedStep;

impl<S: SimState> Integrator<S> for Rk4FixedStep {
    fn determinism(&self) -> IntegratorDeterminism {
        IntegratorDeterminism::BitStable
    }

    fn advance<F>(&self, state: &S, derive_fn: F, dt: Duration) -> Result<S, IntegratorError>
    where
        F: Fn(&S, SimTime) -> Result<S::Derivative, ModelEvalError>,
    {
        let h = dt.as_seconds();
        if !h.is_finite() || h <= 0.0 {
            return Err(IntegratorError::InvalidStep { dt_seconds: h });
        }
        if !state.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let half_h = h * 0.5;

        let t0 = state.time();
        let t0_s = t0.as_seconds();
        let t_mid = SimTime::from_seconds(t0_s + half_h);
        let t_end = SimTime::from_seconds(t0_s + h);

        // Stage 1: derivative at the start of the step.
        let k1 = derive_fn(state, t0)?;
        if !k1.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 2: midpoint state predicted by k1.
        let mid_k2 = state.advance_by(half_h, &k1);
        if !mid_k2.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k2 = derive_fn(&mid_k2, t_mid)?;
        if !k2.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 3: midpoint state predicted by k2.
        let mid_k3 = state.advance_by(half_h, &k2);
        if !mid_k3.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k3 = derive_fn(&mid_k3, t_mid)?;
        if !k3.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 4: end-state predicted by k3.
        let end_k4 = state.advance_by(h, &k3);
        if !end_k4.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k4 = derive_fn(&end_k4, t_end)?;
        if !k4.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Combine: state + h * (k1 + 2 k2 + 2 k3 + k4) / 6.
        // Locked-order weighted sum lives in `SimStateDerivative`.
        let weighted = S::Derivative::rk4_weighted_sum(&k1, &k2, &k3, &k4);
        let mut new_state = state.advance_by(h, &weighted);

        // Apply manifold constraints (no-op for PointMassState;
        // quaternion renormalisation for RigidBodyState in a future
        // sub-phase).
        new_state.project();

        if !new_state.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }

        Ok(new_state)
    }
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
//
// Phase-2.1.B impl. The integration shape is:
//
//   position_new = position + h * velocity_eci
//   velocity_new = velocity + h * acceleration_eci
//   q_new        = q        + h * q_dot     (NON-UNIT until project())
//   ω_new        = ω        + h * ω_dot
//   mass_new     = mass     + h * mass_rate
//   inertia_new  = inertia  + h * inertia_rate
//
// `project()` re-normalises the orientation quaternion to unit
// magnitude after the integrator's final weighted sum so the state
// stays on the SO(3) manifold. The post-step kernel hook calls
// `project()` exactly once per step (RK4 stages may produce non-unit
// intermediate quaternions, which is correct for the linear sum and
// is the locked convention).
//
// Quaternion kinematics convention (locked, see
// `docs/phase-2-plan.md § 2.1`):
//
//   q_dot = 0.5 · q_body_to_eci ⊗ [0, ω_body]
//
// Computed by the user-supplied derivative closure in 2.1.C/D; this
// `SimState` impl only handles the *integration* of `q_dot` and the
// post-step renormalisation.
//
// `is_valid_for_integration` calls `RigidBodyState::require_valid` with
// loose intermediate quaternion / inertia tolerances so RK4 sub-step
// states (with non-unit quaternions) are accepted; the kernel's
// post-step validation uses tight tolerances.

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
            let new_inertia = self.mass_props.inertia_body + h_seconds * d.inertia_rate_body;
            let new_mass_props =
                MassProperties::new(new_mass, self.mass_props.center_of_mass_body, new_inertia);

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

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::derivative::PointMassDerivative;
    use approx::assert_abs_diff_eq;
    use nalgebra::Vector3;
    use openbmp_core::{Position3, SimTime, Velocity3};
    use openbmp_state::PointMassState;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    fn unit_kg() -> Mass {
        Mass::new::<kilogram>(1.0)
    }

    fn one_kg_at_origin() -> PointMassState {
        PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            unit_kg(),
        )
    }

    #[test]
    fn rk4_rejects_zero_dt() {
        let state = one_kg_at_origin();
        let result = Rk4FixedStep.advance(
            &state,
            |_s, _t| Ok(PointMassDerivative::zero()),
            Duration::from_seconds(0.0),
        );
        assert!(matches!(result, Err(IntegratorError::InvalidStep { .. })));
    }

    #[test]
    fn rk4_rejects_negative_dt() {
        let state = one_kg_at_origin();
        let result = Rk4FixedStep.advance(
            &state,
            |_s, _t| Ok(PointMassDerivative::zero()),
            Duration::from_seconds(-0.01),
        );
        assert!(matches!(result, Err(IntegratorError::InvalidStep { .. })));
    }

    #[test]
    fn rk4_zero_derivative_returns_state_with_advanced_time() {
        let state = PointMassState::new(
            SimTime::from_seconds(1.0),
            Position3::new(7.0, 0.0, 0.0),
            Velocity3::new(0.0, 8.0, 0.0),
            unit_kg(),
        );
        let dt = Duration::from_seconds(0.5);
        let next = Rk4FixedStep
            .advance(&state, |_s, _t| Ok(PointMassDerivative::zero()), dt)
            .expect("zero derivative must succeed");
        assert_abs_diff_eq!(next.time.as_seconds(), 1.5);
        // Position and velocity unchanged because derivative is zero.
        assert_abs_diff_eq!(next.position.vector.x, 7.0);
        assert_abs_diff_eq!(next.velocity.vector.y, 8.0);
    }

    /// RK4 is exact for polynomials of degree ≤ 4. Constant
    /// acceleration produces a quadratic `x(t)` and linear `v(t)`,
    /// both well within RK4's exactness range. The integrated state
    /// should match the analytic closed form to floating-point
    /// rounding only.
    #[test]
    fn rk4_constant_acceleration_matches_analytic_for_one_step() {
        // 1 kg point mass at rest; constant acceleration -9.81 m/s² in z.
        let state = one_kg_at_origin();
        let g_eci = Vector3::new(0.0, 0.0, -9.81);

        let derive =
            |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: s.velocity.vector,
                    acceleration_m_s2: g_eci, // mass=1, so force/mass = g.
                    mass_rate_kg_s: 0.0,
                })
            };

        let dt = Duration::from_seconds(0.01);
        let next = Rk4FixedStep
            .advance(&state, derive, dt)
            .expect("step must succeed");

        // Analytic: x(0.01) = 0 + 0·0.01 + ½·(-9.81)·0.0001 = -4.905e-4.
        let expected_z = 0.5 * -9.81 * 0.01_f64 * 0.01;
        assert_abs_diff_eq!(next.position.vector.z, expected_z, epsilon = 1.0e-15);
        // Analytic: v(0.01) = 0 + (-9.81)·0.01 = -0.0981.
        assert_abs_diff_eq!(next.velocity.vector.z, -0.0981, epsilon = 1.0e-15);
        // Time advanced.
        assert_abs_diff_eq!(next.time.as_seconds(), 0.01);
    }

    /// RK4 on `dy/dt = -y` over 10 steps of `dt = 0.1` should match
    /// `exp(-1) ≈ 0.36787944...` to 4th-order accuracy.
    ///
    /// Per-step truncation error scales as `(h⁵ / 120) · y^(5)`. For
    /// `y(t) = exp(-t)`, `|y^(5)|` is bounded by 1 over `[0, 1]`, so
    /// the per-step error is `~8.3e-7`. After 10 steps the
    /// accumulated error stays bounded by a small multiple of that
    /// because the equation is dissipative; observed value is
    /// `~3.4e-7`.
    #[test]
    fn rk4_exponential_decay_step_within_4th_order_error() {
        // We piggy-back on PointMassState by treating mass as the scalar
        // y. `dy/dt = -y` ↔ `mass_rate = -mass`.
        let mut state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        );
        let derive =
            |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: Vector3::zeros(),
                    acceleration_m_s2: Vector3::zeros(),
                    mass_rate_kg_s: -s.mass.get::<kilogram>(),
                })
            };

        let dt = Duration::from_seconds(0.1);
        for _ in 0..10 {
            state = Rk4FixedStep.advance(&state, derive, dt).expect("step");
        }
        let exp_neg_one = (-1.0_f64).exp();
        assert_abs_diff_eq!(state.mass.get::<kilogram>(), exp_neg_one, epsilon = 1.0e-6);
    }

    #[test]
    fn rk4_propagates_non_finite_derivative_as_error() {
        let state = one_kg_at_origin();
        let derive =
            |_s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: Vector3::new(f64::NAN, 0.0, 0.0),
                    acceleration_m_s2: Vector3::zeros(),
                    mass_rate_kg_s: 0.0,
                })
            };
        let dt = Duration::from_seconds(0.01);
        let result = Rk4FixedStep.advance(&state, derive, dt);
        assert!(matches!(result, Err(IntegratorError::NonFiniteDerivative)));
    }

    #[test]
    fn rk4_rejects_invalid_initial_state() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(0.0),
        );
        let result = Rk4FixedStep.advance(
            &state,
            |_s, _t| Ok(PointMassDerivative::zero()),
            Duration::from_seconds(0.01),
        );
        assert!(matches!(result, Err(IntegratorError::NonFiniteState)));
    }

    #[test]
    fn rk4_rejects_invalid_intermediate_state() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        );
        let derive =
            |_s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: Vector3::zeros(),
                    acceleration_m_s2: Vector3::zeros(),
                    mass_rate_kg_s: -10.0,
                })
            };
        let result = Rk4FixedStep.advance(&state, derive, Duration::from_seconds(1.0));
        assert!(matches!(result, Err(IntegratorError::NonFiniteState)));
    }

    #[test]
    fn rk4_is_bit_stable_across_two_runs() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::new(1.0, 2.0, 3.0),
            Velocity3::new(0.4, 0.5, 0.6),
            Mass::new::<kilogram>(1.5),
        );
        let g = Vector3::new(0.1, -0.2, -9.81);
        let derive =
            |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: s.velocity.vector,
                    acceleration_m_s2: g,
                    mass_rate_kg_s: 0.0,
                })
            };

        let dt = Duration::from_seconds(0.01);
        let a = Rk4FixedStep.advance(&state, derive, dt).expect("step a");
        let b = Rk4FixedStep.advance(&state, derive, dt).expect("step b");

        // Bit equality.
        assert_eq!(a.position.vector.x.to_bits(), b.position.vector.x.to_bits());
        assert_eq!(a.position.vector.y.to_bits(), b.position.vector.y.to_bits());
        assert_eq!(a.position.vector.z.to_bits(), b.position.vector.z.to_bits());
        assert_eq!(a.velocity.vector.z.to_bits(), b.velocity.vector.z.to_bits());
        assert_eq!(
            a.mass.get::<kilogram>().to_bits(),
            b.mass.get::<kilogram>().to_bits()
        );
    }

    #[test]
    fn rk4_determinism_class_is_bit_stable() {
        let i = Rk4FixedStep;
        assert_eq!(
            <Rk4FixedStep as Integrator<PointMassState>>::determinism(&i),
            IntegratorDeterminism::BitStable
        );
    }

    // -----------------------------------------------------------------
    // RigidBodyState SimState impl tests (Phase 2.1.B)
    // -----------------------------------------------------------------

    mod rigid_body {
        use super::ModelEvalError;
        use super::*;
        use crate::derivative::RigidBodyDerivative;
        use approx::assert_abs_diff_eq;
        use nalgebra::{
            Matrix3, Quaternion as NalgebraQuaternion, UnitQuaternion as NalgebraUnitQuaternion,
            Vector3,
        };
        use openbmp_core::{
            AngularVelocity3, Body, Eci, Position3, Quaternion, SimTime, UnitQuaternion, Velocity3,
        };
        use openbmp_state::{MassProperties, RigidBodyState};
        use uom::si::f64::Mass;
        use uom::si::mass::kilogram;

        fn unit_state() -> RigidBodyState {
            RigidBodyState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
                AngularVelocity3::zero(),
                MassProperties::with_uniform_inertia(
                    Mass::new::<kilogram>(1.0),
                    Position3::origin(),
                    1.0,
                ),
            )
        }

        #[test]
        fn advance_by_zero_derivative_only_advances_time() {
            let s = unit_state();
            let d = RigidBodyDerivative::zero();
            let s2 = s.advance_by(0.5, &d);
            assert_abs_diff_eq!(s2.time.as_seconds(), 0.5);
            assert_abs_diff_eq!(s2.position.vector.x, 0.0);
            assert_abs_diff_eq!(s2.velocity.vector.x, 0.0);
            assert_abs_diff_eq!(s2.angular_velocity.vector.x, 0.0);
            assert_abs_diff_eq!(s2.mass_props.mass.get::<kilogram>(), 1.0);
        }

        #[test]
        fn project_renormalises_unit_quaternion() {
            let mut s = unit_state();
            // Inflate the quaternion to magnitude 2 to test the
            // renormalisation path.
            let inflated = NalgebraQuaternion::new(2.0, 0.0, 0.0, 0.0);
            s.orientation = Quaternion::<Body, Eci>::from_unit_quaternion(
                NalgebraUnitQuaternion::new_unchecked(inflated),
            );
            // Sanity: non-unit before project.
            let before = s.orientation.q.coords;
            assert_abs_diff_eq!((before.x.powi(2) + before.w.powi(2)).sqrt(), 2.0);
            <RigidBodyState as SimState>::project(&mut s);
            let after = s.orientation.q.coords;
            let n = (after.x.powi(2) + after.y.powi(2) + after.z.powi(2) + after.w.powi(2)).sqrt();
            assert_abs_diff_eq!(n, 1.0, epsilon = 1.0e-15);
        }

        #[test]
        fn rk4_zero_derivative_runs_to_completion_under_rk4() {
            let state = unit_state();
            let dt = Duration::from_seconds(0.01);
            let derive =
                |_s: &RigidBodyState, _t: SimTime| -> Result<RigidBodyDerivative, ModelEvalError> {
                    Ok(RigidBodyDerivative::zero())
                };
            let next = Rk4FixedStep
                .advance(&state, derive, dt)
                .expect("rk4 zero step must succeed");
            assert_abs_diff_eq!(next.time.as_seconds(), 0.01);
            // Quaternion still unit after project().
            let n2 = next.orientation.q.coords.x.powi(2)
                + next.orientation.q.coords.y.powi(2)
                + next.orientation.q.coords.z.powi(2)
                + next.orientation.q.coords.w.powi(2);
            assert_abs_diff_eq!(n2, 1.0, epsilon = 1.0e-15);
        }

        /// Sign / convention check: integrate a constant body-axis
        /// rate `ω = (1, 0, 0)` rad/s (1 rad/s about body +x) for one
        /// small step and compare against the expected finite rotation.
        ///
        /// With locked convention `q_dot = 0.5 · q ⊗ [0, ω_body]`,
        /// starting from identity quaternion and ω = (1, 0, 0), the
        /// quaternion-rate at t=0 is `0.5 · 1 ⊗ (0+1i) = 0.5i`. After
        /// `dt = 0.01 s` of integration, the quaternion's i-component
        /// is approximately `0.5 * 0.01 = 0.005` (and w drops slightly
        /// after renorm). The exact closed-form rotation is
        /// `q(dt) = (cos(dt/2), sin(dt/2), 0, 0)`.
        ///
        /// Note that this *does* require the user-supplied derivative
        /// closure to compute `q_dot` correctly. Phase-2.1.B ships
        /// only the integration shape; the closure is supplied by the
        /// kernel in 2.1.D's torque-free scenario. The test here builds
        /// the closure inline, exercising only `advance_by` /
        /// `project()` from this sub-phase.
        #[test]
        fn quaternion_kinematics_sign_test_for_single_step() {
            let state = unit_state();
            let omega_body = Vector3::new(1.0, 0.0, 0.0); // 1 rad/s about +x
            let dt_s = 0.01;

            let derive =
                |s: &RigidBodyState, _t: SimTime| -> Result<RigidBodyDerivative, ModelEvalError> {
                    // q_dot = 0.5 · q ⊗ [0, ω]
                    let q = s.orientation.q.into_inner();
                    let omega_quat =
                        NalgebraQuaternion::new(0.0, omega_body[0], omega_body[1], omega_body[2]);
                    let q_dot = q * omega_quat * 0.5;
                    Ok(RigidBodyDerivative {
                        velocity_m_s_eci: Vector3::zeros(),
                        acceleration_m_s2_eci: Vector3::zeros(),
                        quaternion_rate: q_dot,
                        angular_acceleration_rad_s2_body: Vector3::zeros(),
                        mass_rate_kg_s: 0.0,
                        inertia_rate_body: Matrix3::zeros(),
                    })
                };

            let next = Rk4FixedStep
                .advance(&state, derive, Duration::from_seconds(dt_s))
                .expect("step must succeed");

            // Closed form: (cos(dt/2), sin(dt/2), 0, 0). Note nalgebra's
            // coords order is (i, j, k, w).
            let expected_w = (dt_s / 2.0).cos();
            let expected_i = (dt_s / 2.0).sin();
            let coords = next.orientation.q.coords;
            assert_abs_diff_eq!(coords.w, expected_w, epsilon = 1.0e-12);
            assert_abs_diff_eq!(coords.x, expected_i, epsilon = 1.0e-12);
            assert_abs_diff_eq!(coords.y, 0.0, epsilon = 1.0e-15);
            assert_abs_diff_eq!(coords.z, 0.0, epsilon = 1.0e-15);
        }
    }
}
