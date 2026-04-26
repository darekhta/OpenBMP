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

use nalgebra::Vector3;

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
}
