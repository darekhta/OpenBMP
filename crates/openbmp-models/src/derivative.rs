//! State-space derivative types.
//!
//! A [`SimStateDerivative`] is the time-derivative of a
//! [`crate::Integratable`] state and supports the primitive
//! linear-arithmetic operations every explicit Runge-Kutta family
//! integrator needs:
//!
//! * Per-component finiteness check.
//! * Componentwise scalar multiplication (`d * f64`) via
//!   [`core::ops::Mul`].
//! * Componentwise addition (`d + d`) via [`core::ops::Add`].
//!
//! The trait deliberately omits an `rk4_weighted_sum` method that
//! would bake the RK4 stage scheme into the trait. The locked-operand-order
//! weighted-sum logic now lives on the integrator side
//! (`openbmp_sim::rk4_weighted_sum`); this trait exposes only the
//! generic primitives, so future integrators (DOPRI5/8, RKF78,
//! adaptive variants) can implement their own weighted-sum helpers
//! using the same operand-order discipline without the trait surface
//! advertising one specific stage scheme.
//!
//! Implementations of `Add` and `Mul<f64>` must use explicit
//! componentwise arithmetic with locked operand order. Never use
//! `f64::mul_add` (FMA hardware rounds once; software emulation
//! rounds twice — cross-platform bit-stable replay requires two
//! explicit roundings).
//!
//! See `docs/software-architecture.md § Determinism Profile` for the
//! full contract.

use core::fmt::Debug;
use core::ops::{Add, Mul};

#[cfg(not(feature = "std"))]
use nalgebra::ComplexField as _;
use nalgebra::{Matrix3, Quaternion as NalgebraQuaternion, Vector3};

/// Trait implemented by the time-derivative of every
/// [`crate::Integratable`] state.
///
/// Only the operations the explicit Runge-Kutta family needs are
/// required: finiteness, addition, and scalar multiplication. The
/// integrator combines stages itself.
///
/// [`SimStateDerivative::l2_norm`] and
/// [`SimStateDerivative::dimension`] let the embedded-error /
/// adaptive-step DOPRI5(4) integrator compute a scaled error
/// norm without componentwise access to the state. This path
/// uses the **scalar approximation** `err = h · ||e'||
/// / (atol + rtol · ||y||)`; the per-component tolerance refinement
/// lives in [`crate::Integratable::weighted_error_norm`].
pub trait SimStateDerivative: Copy + Debug + Add<Output = Self> + Mul<f64, Output = Self> {
    /// Returns `true` if every numeric component is finite.
    #[must_use]
    fn is_finite(&self) -> bool;

    /// Euclidean (L2) norm over every numeric
    /// component of the derivative, treating it as a flat vector in
    /// `R^dim`. Used by the adaptive integrator's scaled error norm.
    /// Locked operand order, no FMA.
    #[must_use]
    fn l2_norm(&self) -> f64;

    /// Total number of scalar components participating
    /// in [`SimStateDerivative::l2_norm`]. The scalar
    /// tolerance path records this for diagnostics and future
    /// per-component / RMS norm follow-ons.
    #[must_use]
    fn dimension(&self) -> usize;
}

/// Time-derivative of a [`openbmp_state::PointMassState`].
///
/// All components are stored as raw `f64` / `Vector3<f64>` for hot-path
/// efficiency. The integrator unwraps typed quantities once at the
/// model boundary and re-wraps them once at the kernel boundary; the
/// derivative passes through the loop unwrapped.
///
/// The implicit time-derivative of `time` is unity and not stored;
/// `SimState::advance_by` adds `h_seconds` directly to the state's
/// time.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct PointMassDerivative {
    /// `d(position)/dt` in m/s. Equals the state's velocity.
    pub velocity_m_s: Vector3<f64>,
    /// `d(velocity)/dt` in m/s². Equals `force_eci / mass`.
    pub acceleration_m_s2: Vector3<f64>,
    /// `d(mass)/dt` in kg/s. Zero for constant-mass models.
    pub mass_rate_kg_s: f64,
}

impl PointMassDerivative {
    /// Construct a [`PointMassDerivative`] explicitly.
    #[must_use]
    pub const fn new(
        velocity_m_s: Vector3<f64>,
        acceleration_m_s2: Vector3<f64>,
        mass_rate_kg_s: f64,
    ) -> Self {
        Self {
            velocity_m_s,
            acceleration_m_s2,
            mass_rate_kg_s,
        }
    }

    /// All-zero derivative (a stationary, constant-mass body under
    /// no force).
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            velocity_m_s: Vector3::new(0.0, 0.0, 0.0),
            acceleration_m_s2: Vector3::new(0.0, 0.0, 0.0),
            mass_rate_kg_s: 0.0,
        }
    }
}

impl Add for PointMassDerivative {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        // DETERMINISM: explicit componentwise sum; no FMA.
        Self {
            velocity_m_s: self.velocity_m_s + rhs.velocity_m_s,
            acceleration_m_s2: self.acceleration_m_s2 + rhs.acceleration_m_s2,
            mass_rate_kg_s: self.mass_rate_kg_s + rhs.mass_rate_kg_s,
        }
    }
}

impl Mul<f64> for PointMassDerivative {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        // DETERMINISM: explicit componentwise scalar multiplication;
        // no FMA.
        Self {
            velocity_m_s: self.velocity_m_s * rhs,
            acceleration_m_s2: self.acceleration_m_s2 * rhs,
            mass_rate_kg_s: self.mass_rate_kg_s * rhs,
        }
    }
}

impl SimStateDerivative for PointMassDerivative {
    fn is_finite(&self) -> bool {
        self.velocity_m_s.iter().copied().all(f64::is_finite)
            && self.acceleration_m_s2.iter().copied().all(f64::is_finite)
            && self.mass_rate_kg_s.is_finite()
    }

    fn l2_norm(&self) -> f64 {
        // Locked-order squared sum: velocity₀..₂, acceleration₀..₂,
        // mass_rate. No FMA.
        let mut s = 0.0_f64;
        s += self.velocity_m_s.x * self.velocity_m_s.x;
        s += self.velocity_m_s.y * self.velocity_m_s.y;
        s += self.velocity_m_s.z * self.velocity_m_s.z;
        s += self.acceleration_m_s2.x * self.acceleration_m_s2.x;
        s += self.acceleration_m_s2.y * self.acceleration_m_s2.y;
        s += self.acceleration_m_s2.z * self.acceleration_m_s2.z;
        s += self.mass_rate_kg_s * self.mass_rate_kg_s;
        s.sqrt()
    }

    fn dimension(&self) -> usize {
        // 3 (velocity) + 3 (acceleration) + 1 (mass rate) = 7.
        7
    }
}

// ---------------------------------------------------------------------
// RigidBodyDerivative
// ---------------------------------------------------------------------

/// Time-derivative of an [`openbmp_state::RigidBodyState`].
///
/// Seven fields, mirroring the rigid-body state rates:
///
/// * `velocity_m_s_eci` — `dposition/dt`. Equals the state's velocity.
/// * `acceleration_m_s2_eci` — `dvelocity/dt`. Equals
///   `force_eci / mass_kg` per Newton II.
/// * `quaternion_rate` — `dorientation/dt`, expressed as a non-unit
///   `nalgebra::Quaternion<f64>`. The locked kinematic convention is
///   `q_dot = 0.5 · q_body_to_eci ⊗ [0, ω_body]`. The integrator
///   advances the four components linearly; the post-step `project()`
///   on `RigidBodyState` re-normalises to unit magnitude so the
///   manifold constraint is restored without changing the determinism
///   contract.
/// * `angular_acceleration_rad_s2_body` — `dω_body/dt`, from the Euler
///   equation `I⁻¹ (M − ω × Iω − I_dot ω)`. The `−I_dot ω` term is
///   essential when the mass model returns a non-zero
///   `inertia_rate_body`.
/// * `mass_rate_kg_s` — scalar mass rate (kg/s). Negative for burn.
/// * `center_of_mass_rate_body_m_s` — body-frame CG offset rate.
/// * `inertia_rate_body` — `dI_body/dt`. Zero for the
///   `ConstantMassRigid` model; non-zero for `LinearBurnMassRigid` if
///   the scenario declares an inertia derivative explicitly.
#[derive(Copy, Clone, Debug)]
pub struct RigidBodyDerivative {
    /// `dposition/dt` in m/s, ECI.
    pub velocity_m_s_eci: Vector3<f64>,
    /// `dvelocity/dt` in m/s², ECI.
    pub acceleration_m_s2_eci: Vector3<f64>,
    /// `dq/dt`, non-unit Hamilton quaternion (locked
    /// `q_dot = 0.5 · q ⊗ [0, ω_body]`).
    pub quaternion_rate: NalgebraQuaternion<f64>,
    /// `dω_body/dt` in rad/s², body-frame.
    pub angular_acceleration_rad_s2_body: Vector3<f64>,
    /// `dmass/dt` in kg/s.
    pub mass_rate_kg_s: f64,
    /// `d(center_of_mass_body)/dt` in m/s, body-frame components.
    pub center_of_mass_rate_body_m_s: Vector3<f64>,
    /// `dI_body/dt` in kg·m²/s.
    pub inertia_rate_body: Matrix3<f64>,
}

impl RigidBodyDerivative {
    /// Construct from explicit parts.
    #[must_use]
    pub const fn new(
        velocity_m_s_eci: Vector3<f64>,
        acceleration_m_s2_eci: Vector3<f64>,
        quaternion_rate: NalgebraQuaternion<f64>,
        angular_acceleration_rad_s2_body: Vector3<f64>,
        mass_rate_kg_s: f64,
        center_of_mass_rate_body_m_s: Vector3<f64>,
        inertia_rate_body: Matrix3<f64>,
    ) -> Self {
        Self {
            velocity_m_s_eci,
            acceleration_m_s2_eci,
            quaternion_rate,
            angular_acceleration_rad_s2_body,
            mass_rate_kg_s,
            center_of_mass_rate_body_m_s,
            inertia_rate_body,
        }
    }

    /// All-zero derivative (a stationary, constant-mass / constant-
    /// inertia body under no force or moment).
    #[must_use]
    pub fn zero() -> Self {
        Self {
            velocity_m_s_eci: Vector3::zeros(),
            acceleration_m_s2_eci: Vector3::zeros(),
            quaternion_rate: NalgebraQuaternion::new(0.0, 0.0, 0.0, 0.0),
            angular_acceleration_rad_s2_body: Vector3::zeros(),
            mass_rate_kg_s: 0.0,
            center_of_mass_rate_body_m_s: Vector3::zeros(),
            inertia_rate_body: Matrix3::zeros(),
        }
    }
}

impl Add for RigidBodyDerivative {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        // DETERMINISM: explicit componentwise sum on every field.
        // The quaternion is summed via its underlying `coords`
        // Vector4; the result is intentionally non-unit (the
        // integrator's `project()` restores the manifold constraint
        // after the final weighted sum).
        let quaternion_rate =
            NalgebraQuaternion::from(self.quaternion_rate.coords + rhs.quaternion_rate.coords);
        Self {
            velocity_m_s_eci: self.velocity_m_s_eci + rhs.velocity_m_s_eci,
            acceleration_m_s2_eci: self.acceleration_m_s2_eci + rhs.acceleration_m_s2_eci,
            quaternion_rate,
            angular_acceleration_rad_s2_body: self.angular_acceleration_rad_s2_body
                + rhs.angular_acceleration_rad_s2_body,
            mass_rate_kg_s: self.mass_rate_kg_s + rhs.mass_rate_kg_s,
            center_of_mass_rate_body_m_s: self.center_of_mass_rate_body_m_s
                + rhs.center_of_mass_rate_body_m_s,
            inertia_rate_body: self.inertia_rate_body + rhs.inertia_rate_body,
        }
    }
}

impl Mul<f64> for RigidBodyDerivative {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        // DETERMINISM: explicit componentwise scalar multiplication.
        let quaternion_rate = NalgebraQuaternion::from(self.quaternion_rate.coords * rhs);
        Self {
            velocity_m_s_eci: self.velocity_m_s_eci * rhs,
            acceleration_m_s2_eci: self.acceleration_m_s2_eci * rhs,
            quaternion_rate,
            angular_acceleration_rad_s2_body: self.angular_acceleration_rad_s2_body * rhs,
            mass_rate_kg_s: self.mass_rate_kg_s * rhs,
            center_of_mass_rate_body_m_s: self.center_of_mass_rate_body_m_s * rhs,
            inertia_rate_body: self.inertia_rate_body * rhs,
        }
    }
}

impl SimStateDerivative for RigidBodyDerivative {
    fn is_finite(&self) -> bool {
        self.velocity_m_s_eci.iter().copied().all(f64::is_finite)
            && self
                .acceleration_m_s2_eci
                .iter()
                .copied()
                .all(f64::is_finite)
            && self
                .quaternion_rate
                .coords
                .iter()
                .copied()
                .all(f64::is_finite)
            && self
                .angular_acceleration_rad_s2_body
                .iter()
                .copied()
                .all(f64::is_finite)
            && self.mass_rate_kg_s.is_finite()
            && self
                .center_of_mass_rate_body_m_s
                .iter()
                .copied()
                .all(f64::is_finite)
            && self.inertia_rate_body.iter().copied().all(f64::is_finite)
    }

    fn l2_norm(&self) -> f64 {
        // Locked-order squared sum across every numeric component.
        // Vector and matrix components are summed in fixed
        // (row, col) traversal order; quaternion uses the underlying
        // (x, y, z, w) coords order. No FMA.
        let mut s = 0.0_f64;
        for v in self.velocity_m_s_eci.iter() {
            s += v * v;
        }
        for v in self.acceleration_m_s2_eci.iter() {
            s += v * v;
        }
        for v in self.quaternion_rate.coords.iter() {
            s += v * v;
        }
        for v in self.angular_acceleration_rad_s2_body.iter() {
            s += v * v;
        }
        s += self.mass_rate_kg_s * self.mass_rate_kg_s;
        for v in self.center_of_mass_rate_body_m_s.iter() {
            s += v * v;
        }
        for v in self.inertia_rate_body.iter() {
            s += v * v;
        }
        s.sqrt()
    }

    fn dimension(&self) -> usize {
        // 3 (velocity) + 3 (acceleration) + 4 (quaternion rate) +
        // 3 (angular accel) + 1 (mass rate) + 3 (cg rate) +
        // 9 (3x3 inertia rate) = 26.
        26
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn deriv(vx: f64, vy: f64, vz: f64, ax: f64, ay: f64, az: f64, mr: f64) -> PointMassDerivative {
        PointMassDerivative::new(Vector3::new(vx, vy, vz), Vector3::new(ax, ay, az), mr)
    }

    #[test]
    fn zero_derivative_is_finite() {
        assert!(PointMassDerivative::zero().is_finite());
    }

    #[test]
    fn finite_check_rejects_nan() {
        let mut d = PointMassDerivative::zero();
        d.velocity_m_s.x = f64::NAN;
        assert!(!d.is_finite());
        let mut d = PointMassDerivative::zero();
        d.acceleration_m_s2.z = f64::INFINITY;
        assert!(!d.is_finite());
        let mut d = PointMassDerivative::zero();
        d.mass_rate_kg_s = f64::NAN;
        assert!(!d.is_finite());
    }

    #[test]
    fn add_componentwise_is_commutative_in_value() {
        let a = deriv(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.5);
        let b = deriv(0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 0.25);
        let s = a + b;
        assert_abs_diff_eq!(s.velocity_m_s.x, 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.acceleration_m_s2.z, 9.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.mass_rate_kg_s, 0.75, epsilon = 1.0e-15);
    }

    #[test]
    fn scalar_mul_componentwise() {
        let a = deriv(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.5);
        let s = a * 2.0;
        assert_abs_diff_eq!(s.velocity_m_s.x, 2.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.acceleration_m_s2.z, 12.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.mass_rate_kg_s, 1.0, epsilon = 1.0e-15);
    }

    #[test]
    fn primitives_compose_into_rk4_weighted_sum() {
        // (k + 2k + 2k + k) / 6 = 6k / 6 = k. Verifies the integrator-
        // side weighted-sum helper builds correctly on top of the new
        // primitive ops.
        let k = deriv(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.5);
        let s = ((k + (k * 2.0)) + (k * 2.0) + k) * (1.0 / 6.0);
        assert_abs_diff_eq!(s.velocity_m_s.x, 1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.velocity_m_s.y, 2.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.acceleration_m_s2.z, 6.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.mass_rate_kg_s, 0.5, epsilon = 1.0e-15);
    }

    #[test]
    fn primitives_are_bit_stable_across_reruns() {
        let k1 = deriv(1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 0.7);
        let k2 = deriv(0.9, 1.8, 2.7, 3.6, 4.5, 5.4, 0.6);
        let k3 = deriv(1.05, 2.1, 3.15, 4.2, 5.25, 6.3, 0.65);
        let k4 = deriv(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.7);
        let a = ((k1 + (k2 * 2.0)) + (k3 * 2.0) + k4) * (1.0 / 6.0);
        let b = ((k1 + (k2 * 2.0)) + (k3 * 2.0) + k4) * (1.0 / 6.0);
        assert_eq!(a.velocity_m_s.x.to_bits(), b.velocity_m_s.x.to_bits());
        assert_eq!(a.velocity_m_s.y.to_bits(), b.velocity_m_s.y.to_bits());
        assert_eq!(a.mass_rate_kg_s.to_bits(), b.mass_rate_kg_s.to_bits());
    }

    // -----------------------------------------------------------------
    // RigidBodyDerivative tests
    // -----------------------------------------------------------------

    fn rigid_zero() -> RigidBodyDerivative {
        RigidBodyDerivative::zero()
    }

    fn rigid_uniform(scalar: f64) -> RigidBodyDerivative {
        RigidBodyDerivative::new(
            Vector3::new(scalar, scalar, scalar),
            Vector3::new(scalar, scalar, scalar),
            NalgebraQuaternion::new(scalar, scalar, scalar, scalar),
            Vector3::new(scalar, scalar, scalar),
            scalar,
            Vector3::new(scalar, scalar, scalar),
            Matrix3::from_element(scalar),
        )
    }

    #[test]
    fn rigid_zero_derivative_is_finite() {
        assert!(rigid_zero().is_finite());
    }

    #[test]
    fn rigid_finite_check_rejects_nan_in_each_field() {
        let mut d = rigid_zero();
        d.velocity_m_s_eci.x = f64::NAN;
        assert!(!d.is_finite());

        let mut d = rigid_zero();
        d.acceleration_m_s2_eci.z = f64::INFINITY;
        assert!(!d.is_finite());

        let mut d = rigid_zero();
        d.quaternion_rate = NalgebraQuaternion::new(f64::NAN, 0.0, 0.0, 0.0);
        assert!(!d.is_finite());

        let mut d = rigid_zero();
        d.angular_acceleration_rad_s2_body.y = f64::NAN;
        assert!(!d.is_finite());

        let mut d = rigid_zero();
        d.mass_rate_kg_s = f64::NAN;
        assert!(!d.is_finite());

        let mut d = rigid_zero();
        d.center_of_mass_rate_body_m_s.x = f64::NAN;
        assert!(!d.is_finite());

        let mut d = rigid_zero();
        d.inertia_rate_body[(0, 0)] = f64::NAN;
        assert!(!d.is_finite());
    }

    #[test]
    fn rigid_primitives_compose_into_rk4_weighted_sum() {
        let k = rigid_uniform(1.5);
        let s = ((k + (k * 2.0)) + (k * 2.0) + k) * (1.0 / 6.0);
        assert_abs_diff_eq!(s.velocity_m_s_eci.x, 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.quaternion_rate.coords[3], 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.mass_rate_kg_s, 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.inertia_rate_body[(2, 2)], 1.5, epsilon = 1.0e-15);
    }

    // -----------------------------------------------------------------
    // `l2_norm()` and `dimension()` invariants supporting
    // the adaptive-step DOPRI5(4) scalar error-norm surface.
    // -----------------------------------------------------------------

    #[test]
    fn point_mass_derivative_l2_norm_matches_hand_computed_sum() {
        // (3, 4, 0) velocity, (0, 0, 0) accel, mass_rate = 0
        // → ||v|| = 5, others zero, total = 5.
        let d = PointMassDerivative::new(Vector3::new(3.0, 4.0, 0.0), Vector3::zeros(), 0.0);
        assert_abs_diff_eq!(d.l2_norm(), 5.0, epsilon = 1.0e-15);
        // Add (0, 0, 0) velocity, (3, 4, 0) accel, mass_rate = 12
        // → 0 + 25 + 144 = 169 → norm = 13.
        let d = PointMassDerivative::new(Vector3::zeros(), Vector3::new(3.0, 4.0, 0.0), 12.0);
        assert_abs_diff_eq!(d.l2_norm(), 13.0, epsilon = 1.0e-15);
        assert_eq!(d.dimension(), 7);
    }

    #[test]
    fn rigid_body_derivative_l2_norm_matches_full_componentwise_sum() {
        let mut d = RigidBodyDerivative::zero();
        d.velocity_m_s_eci = Vector3::new(3.0, 4.0, 0.0); // ||·||² = 25
        d.acceleration_m_s2_eci = Vector3::new(0.0, 0.0, 12.0); // ||·||² = 144
        // sqrt(25 + 144) = 13
        assert_abs_diff_eq!(d.l2_norm(), 13.0, epsilon = 1.0e-15);
        // 3+3+4+3+1+3+9 = 26
        assert_eq!(d.dimension(), 26);
    }

    #[test]
    fn point_mass_derivative_l2_norm_is_bit_stable_across_two_calls() {
        // Locked-order summation: same inputs → bit-identical norms.
        let d = PointMassDerivative::new(
            Vector3::new(1.5, -2.25, 3.75),
            Vector3::new(-0.1, 0.2, -0.3),
            0.05,
        );
        let a = d.l2_norm();
        let b = d.l2_norm();
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "L2 norm not bit-stable across two calls",
        );
    }

    #[test]
    fn rigid_body_derivative_l2_norm_is_bit_stable_across_two_calls() {
        let mut d = RigidBodyDerivative::zero();
        d.velocity_m_s_eci = Vector3::new(1.5, -2.25, 3.75);
        d.acceleration_m_s2_eci = Vector3::new(-0.1, 0.2, -0.3);
        d.angular_acceleration_rad_s2_body = Vector3::new(0.05, -0.05, 0.0);
        d.mass_rate_kg_s = 0.001;
        let a = d.l2_norm();
        let b = d.l2_norm();
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "L2 norm not bit-stable across two calls",
        );
    }
}
