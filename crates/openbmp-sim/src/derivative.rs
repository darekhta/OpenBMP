//! State-space derivative types.
//!
//! A [`SimStateDerivative`] is the time-derivative of a [`crate::SimState`]
//! and supports the linear-arithmetic operations an explicit Runge-Kutta
//! integrator needs:
//!
//! * Per-component finiteness check.
//! * The canonical RK4 weighted sum
//!   `(k1 + 2·k2 + 2·k3 + k4) / 6` with **locked** evaluation order.
//!
//! The locked sum order is part of the OpenBMP determinism contract.
//! Implementations must:
//!
//! * Use explicit parentheses to prevent any future compiler from
//!   reassociating the sum.
//! * Multiply by `1.0 / 6.0` once at the end (not pre-distribute the
//!   `2/6` constant — that introduces an extra rounding before the sum).
//! * Never use `f64::mul_add` (FMA hardware rounds once; software
//!   emulation rounds twice — cross-platform bit-stable replay requires
//!   two explicit roundings).
//!
//! See `docs/software-architecture.md § Determinism Profile` for
//! the full contract.

use nalgebra::{Matrix3, Quaternion as NalgebraQuaternion, Vector3};

/// Trait implemented by the time-derivative of every [`crate::SimState`].
///
/// Only the operations the explicit Runge-Kutta family needs are
/// required.
pub trait SimStateDerivative: Copy + std::fmt::Debug {
    /// Returns `true` if every numeric component is finite.
    #[must_use]
    fn is_finite(&self) -> bool;

    /// Compute the canonical RK4 weighted sum
    /// `(k1 + 2·k2 + 2·k3 + k4) / 6`.
    ///
    /// Implementations **must** preserve the source-level order of
    /// operations. The OpenBMP determinism contract depends on this.
    #[must_use]
    fn rk4_weighted_sum(k1: &Self, k2: &Self, k3: &Self, k4: &Self) -> Self;
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

impl SimStateDerivative for PointMassDerivative {
    fn is_finite(&self) -> bool {
        self.velocity_m_s.iter().all(|v| v.is_finite())
            && self.acceleration_m_s2.iter().all(|v| v.is_finite())
            && self.mass_rate_kg_s.is_finite()
    }

    fn rk4_weighted_sum(k1: &Self, k2: &Self, k3: &Self, k4: &Self) -> Self {
        // DETERMINISM CONTRACT (see module docs):
        // - Locked left-to-right summation order.
        // - Single final divide by 6 (not pre-distributed 2/6).
        // - No `f64::mul_add` anywhere.
        const TWO: f64 = 2.0;
        const SIXTH: f64 = 1.0 / 6.0;

        let velocity_m_s = (((k1.velocity_m_s + TWO * k2.velocity_m_s) + TWO * k3.velocity_m_s)
            + k4.velocity_m_s)
            * SIXTH;

        let acceleration_m_s2 = (((k1.acceleration_m_s2 + TWO * k2.acceleration_m_s2)
            + TWO * k3.acceleration_m_s2)
            + k4.acceleration_m_s2)
            * SIXTH;

        let mass_rate_kg_s = (((k1.mass_rate_kg_s + TWO * k2.mass_rate_kg_s)
            + TWO * k3.mass_rate_kg_s)
            + k4.mass_rate_kg_s)
            * SIXTH;

        Self {
            velocity_m_s,
            acceleration_m_s2,
            mass_rate_kg_s,
        }
    }
}

// ---------------------------------------------------------------------
// RigidBodyDerivative
// ---------------------------------------------------------------------

/// Time-derivative of an [`openbmp_state::RigidBodyState`].
///
/// Six fields, mirroring the six elements of the rigid-body state:
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
/// * `inertia_rate_body` — `dI_body/dt`. Zero for the Phase-2
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
        inertia_rate_body: Matrix3<f64>,
    ) -> Self {
        Self {
            velocity_m_s_eci,
            acceleration_m_s2_eci,
            quaternion_rate,
            angular_acceleration_rad_s2_body,
            mass_rate_kg_s,
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
            inertia_rate_body: Matrix3::zeros(),
        }
    }
}

impl SimStateDerivative for RigidBodyDerivative {
    fn is_finite(&self) -> bool {
        self.velocity_m_s_eci.iter().all(|v| v.is_finite())
            && self.acceleration_m_s2_eci.iter().all(|v| v.is_finite())
            && self.quaternion_rate.coords.iter().all(|v| v.is_finite())
            && self
                .angular_acceleration_rad_s2_body
                .iter()
                .all(|v| v.is_finite())
            && self.mass_rate_kg_s.is_finite()
            && self.inertia_rate_body.iter().all(|v| v.is_finite())
    }

    fn rk4_weighted_sum(k1: &Self, k2: &Self, k3: &Self, k4: &Self) -> Self {
        // DETERMINISM CONTRACT: locked left-to-right summation order
        // for every field; single final divide by 6 (not pre-distributed
        // 2/6); no `f64::mul_add`.
        const TWO: f64 = 2.0;
        const SIXTH: f64 = 1.0 / 6.0;

        let velocity_m_s_eci = (((k1.velocity_m_s_eci + TWO * k2.velocity_m_s_eci)
            + TWO * k3.velocity_m_s_eci)
            + k4.velocity_m_s_eci)
            * SIXTH;

        let acceleration_m_s2_eci = (((k1.acceleration_m_s2_eci + TWO * k2.acceleration_m_s2_eci)
            + TWO * k3.acceleration_m_s2_eci)
            + k4.acceleration_m_s2_eci)
            * SIXTH;

        // Quaternion rate is a non-unit `Quaternion<f64>`; sum the
        // underlying `coords` Vector4 in locked order.
        let q_coords = (((k1.quaternion_rate.coords + TWO * k2.quaternion_rate.coords)
            + TWO * k3.quaternion_rate.coords)
            + k4.quaternion_rate.coords)
            * SIXTH;
        // nalgebra `Quaternion::new(w, i, j, k)` reorders into
        // `coords = (i, j, k, w)`; we already have `coords` in the
        // correct internal order, so reconstruct from the pre-summed
        // coords directly via `Quaternion::from(...)`.
        let quaternion_rate = NalgebraQuaternion::from(q_coords);

        let angular_acceleration_rad_s2_body = (((k1.angular_acceleration_rad_s2_body
            + TWO * k2.angular_acceleration_rad_s2_body)
            + TWO * k3.angular_acceleration_rad_s2_body)
            + k4.angular_acceleration_rad_s2_body)
            * SIXTH;

        let mass_rate_kg_s = (((k1.mass_rate_kg_s + TWO * k2.mass_rate_kg_s)
            + TWO * k3.mass_rate_kg_s)
            + k4.mass_rate_kg_s)
            * SIXTH;

        let inertia_rate_body = (((k1.inertia_rate_body + TWO * k2.inertia_rate_body)
            + TWO * k3.inertia_rate_body)
            + k4.inertia_rate_body)
            * SIXTH;

        Self {
            velocity_m_s_eci,
            acceleration_m_s2_eci,
            quaternion_rate,
            angular_acceleration_rad_s2_body,
            mass_rate_kg_s,
            inertia_rate_body,
        }
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
    fn rk4_sum_of_zeros_is_zero() {
        let z = PointMassDerivative::zero();
        let s = PointMassDerivative::rk4_weighted_sum(&z, &z, &z, &z);
        assert_abs_diff_eq!(s.velocity_m_s.x, 0.0);
        assert_abs_diff_eq!(s.acceleration_m_s2.x, 0.0);
        assert_abs_diff_eq!(s.mass_rate_kg_s, 0.0);
    }

    #[test]
    fn rk4_sum_of_identical_derivatives_returns_that_derivative() {
        // (k + 2k + 2k + k) / 6 = 6k / 6 = k.
        let k = deriv(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.5);
        let s = PointMassDerivative::rk4_weighted_sum(&k, &k, &k, &k);
        assert_abs_diff_eq!(s.velocity_m_s.x, 1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.velocity_m_s.y, 2.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.velocity_m_s.z, 3.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.acceleration_m_s2.x, 4.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.acceleration_m_s2.y, 5.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.acceleration_m_s2.z, 6.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.mass_rate_kg_s, 0.5, epsilon = 1.0e-15);
    }

    #[test]
    fn rk4_sum_is_bit_stable_across_reruns() {
        // Exact bit-stability is the determinism contract.
        let k1 = deriv(1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 0.7);
        let k2 = deriv(0.9, 1.8, 2.7, 3.6, 4.5, 5.4, 0.6);
        let k3 = deriv(1.05, 2.1, 3.15, 4.2, 5.25, 6.3, 0.65);
        let k4 = deriv(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.7);
        let a = PointMassDerivative::rk4_weighted_sum(&k1, &k2, &k3, &k4);
        let b = PointMassDerivative::rk4_weighted_sum(&k1, &k2, &k3, &k4);
        // Bit-equality, not approximate.
        assert_eq!(a.velocity_m_s.x.to_bits(), b.velocity_m_s.x.to_bits());
        assert_eq!(a.velocity_m_s.y.to_bits(), b.velocity_m_s.y.to_bits());
        assert_eq!(a.velocity_m_s.z.to_bits(), b.velocity_m_s.z.to_bits());
        assert_eq!(
            a.acceleration_m_s2.x.to_bits(),
            b.acceleration_m_s2.x.to_bits()
        );
        assert_eq!(a.mass_rate_kg_s.to_bits(), b.mass_rate_kg_s.to_bits());
    }

    #[test]
    fn rk4_sum_handles_constant_acceleration_correctly() {
        // For constant `k`, all four stages are equal so the weighted
        // sum equals `k` modulo the rounding from the divide-by-6:
        // `(k + 2k + 2k + k) / 6 = 6k / 6` differs from `k` by at most
        // a couple of ULPs because 1/6 is not exactly representable
        // in IEEE-754 binary64.
        let g = deriv(0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0);
        let s = PointMassDerivative::rk4_weighted_sum(&g, &g, &g, &g);
        assert_abs_diff_eq!(s.velocity_m_s.z, -9.81, epsilon = 1.0e-13);
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
        // Replace coords with one carrying a NaN.
        d.quaternion_rate = NalgebraQuaternion::new(f64::NAN, 0.0, 0.0, 0.0);
        assert!(!d.is_finite());

        let mut d = rigid_zero();
        d.angular_acceleration_rad_s2_body.y = f64::NAN;
        assert!(!d.is_finite());

        let mut d = rigid_zero();
        d.mass_rate_kg_s = f64::NAN;
        assert!(!d.is_finite());

        let mut d = rigid_zero();
        d.inertia_rate_body[(0, 0)] = f64::NAN;
        assert!(!d.is_finite());
    }

    #[test]
    fn rigid_rk4_sum_of_zeros_is_zero() {
        let z = rigid_zero();
        let s = RigidBodyDerivative::rk4_weighted_sum(&z, &z, &z, &z);
        assert!(s.is_finite());
        assert_abs_diff_eq!(s.velocity_m_s_eci.x, 0.0);
        assert_abs_diff_eq!(s.acceleration_m_s2_eci.x, 0.0);
        assert_abs_diff_eq!(s.quaternion_rate.coords[0], 0.0);
        assert_abs_diff_eq!(s.angular_acceleration_rad_s2_body.x, 0.0);
        assert_abs_diff_eq!(s.mass_rate_kg_s, 0.0);
        assert_abs_diff_eq!(s.inertia_rate_body[(0, 0)], 0.0);
    }

    #[test]
    fn rigid_rk4_sum_of_identical_derivatives_returns_that_derivative() {
        let k = rigid_uniform(1.5);
        let s = RigidBodyDerivative::rk4_weighted_sum(&k, &k, &k, &k);
        assert_abs_diff_eq!(s.velocity_m_s_eci.x, 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.acceleration_m_s2_eci.x, 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.quaternion_rate.coords[3], 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.angular_acceleration_rad_s2_body.z, 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.mass_rate_kg_s, 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(s.inertia_rate_body[(2, 2)], 1.5, epsilon = 1.0e-15);
    }

    #[test]
    fn rigid_rk4_sum_is_bit_stable_across_reruns() {
        let k1 = rigid_uniform(1.1);
        let k2 = rigid_uniform(0.9);
        let k3 = rigid_uniform(1.05);
        let k4 = rigid_uniform(1.0);
        let a = RigidBodyDerivative::rk4_weighted_sum(&k1, &k2, &k3, &k4);
        let b = RigidBodyDerivative::rk4_weighted_sum(&k1, &k2, &k3, &k4);
        // Bit equality on a representative field from each kind.
        assert_eq!(
            a.velocity_m_s_eci.x.to_bits(),
            b.velocity_m_s_eci.x.to_bits()
        );
        assert_eq!(
            a.quaternion_rate.coords[0].to_bits(),
            b.quaternion_rate.coords[0].to_bits(),
        );
        assert_eq!(
            a.angular_acceleration_rad_s2_body.y.to_bits(),
            b.angular_acceleration_rad_s2_body.y.to_bits(),
        );
        assert_eq!(a.mass_rate_kg_s.to_bits(), b.mass_rate_kg_s.to_bits());
        assert_eq!(
            a.inertia_rate_body[(1, 1)].to_bits(),
            b.inertia_rate_body[(1, 1)].to_bits(),
        );
    }
}
