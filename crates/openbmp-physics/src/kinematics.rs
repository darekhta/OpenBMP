//! Rigid-body kinematics primitives.
//!
//! Small, dependency-light math used by both simulator-side rigid-
//! body propagators and FC-side estimators. Quaternion construction
//! from rotation vectors, renormalization, and the cross-product
//! skew-symmetric matrix all live here so consumers don't reinvent
//! them with subtly different operand ordering.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic; locked operand order; no FMA. The
//! quaternion helpers delegate to `nalgebra::UnitQuaternion`'s exact-
//! axis-angle constructor on inputs above the small-rotation cutoff
//! and return identity below it.

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

/// Cutoff below which a rotation vector is treated as identity.
///
/// `sqrt(EPSILON)` keeps the small-angle approximation accurate to
/// ~1e-8 rad while avoiding division-by-zero in axis normalisation.
const SMALL_ROTATION_CUTOFF: f64 = 1.490_116_119_384_765_6e-8; // f64::EPSILON.sqrt()

/// Quaternion from a body-frame rotation vector (axis-angle, with the
/// magnitude carrying the angle in radians).
///
/// Below [`SMALL_ROTATION_CUTOFF`] the result is the identity
/// quaternion; above it `nalgebra`'s exact constructor is used.
#[must_use]
pub fn quaternion_from_axis_angle(axis_angle: Vector3<f64>) -> UnitQuaternion<f64> {
    let mag = axis_angle.norm();
    if mag <= SMALL_ROTATION_CUTOFF {
        UnitQuaternion::identity()
    } else {
        UnitQuaternion::from_scaled_axis(axis_angle)
    }
}

/// Quaternion increment from a body-frame angular velocity (rad/s)
/// integrated over `dt` seconds. Equivalent to
/// `quaternion_from_axis_angle(omega · dt)`.
#[must_use]
pub fn quaternion_from_omega(omega: Vector3<f64>, dt: f64) -> UnitQuaternion<f64> {
    quaternion_from_axis_angle(omega * dt)
}

/// Renormalise a unit quaternion in place. Combats finite-precision
/// drift after sequences of multiplications by reprojecting onto the
/// unit sphere via `nalgebra::UnitQuaternion::new_normalize`.
pub fn renormalize_quaternion(q: &mut UnitQuaternion<f64>) {
    *q = UnitQuaternion::new_normalize((*q).into_inner());
}

/// 3×3 skew-symmetric (cross-product) matrix `[v]_x` such that
/// `[v]_x · w = v × w` for any 3-vector `w`.
#[must_use]
pub fn skew_symmetric(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v.z, v.y, //
        v.z, 0.0, -v.x, //
        -v.y, v.x, 0.0,
    )
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn quaternion_from_axis_angle_zero_returns_identity() {
        let q = quaternion_from_axis_angle(Vector3::zeros());
        let identity = UnitQuaternion::<f64>::identity();
        assert_abs_diff_eq!(q.w, identity.w, epsilon = 1.0e-15);
    }

    #[test]
    fn quaternion_from_axis_angle_below_cutoff_returns_identity() {
        let q = quaternion_from_axis_angle(Vector3::new(1.0e-12, 0.0, 0.0));
        let identity = UnitQuaternion::<f64>::identity();
        assert_eq!(q.w, identity.w);
    }

    #[test]
    fn quaternion_from_axis_angle_pi_around_z_flips_x() {
        let q = quaternion_from_axis_angle(Vector3::new(0.0, 0.0, std::f64::consts::PI));
        let v = Vector3::new(1.0, 0.0, 0.0);
        let rotated = q * v;
        assert_abs_diff_eq!(rotated.x, -1.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(rotated.y, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(rotated.z, 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn quaternion_from_omega_dt_zero_is_identity() {
        let q = quaternion_from_omega(Vector3::new(1.0, 2.0, 3.0), 0.0);
        let identity = UnitQuaternion::<f64>::identity();
        assert_eq!(q.w, identity.w);
    }

    #[test]
    fn renormalize_keeps_norm_unit() {
        let mut q = UnitQuaternion::from_scaled_axis(Vector3::new(0.1, 0.2, 0.3));
        renormalize_quaternion(&mut q);
        let n = q.into_inner().norm();
        assert_abs_diff_eq!(n, 1.0, epsilon = 1.0e-15);
    }

    #[test]
    fn skew_symmetric_implements_cross_product() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let w = Vector3::new(4.0, 5.0, 6.0);
        let cross = skew_symmetric(v) * w;
        let expected = v.cross(&w);
        assert_abs_diff_eq!(cross.x, expected.x, epsilon = 1.0e-15);
        assert_abs_diff_eq!(cross.y, expected.y, epsilon = 1.0e-15);
        assert_abs_diff_eq!(cross.z, expected.z, epsilon = 1.0e-15);
    }

    #[test]
    fn skew_symmetric_is_antisymmetric() {
        let v = Vector3::new(0.7, -1.3, 4.2);
        let m = skew_symmetric(v);
        let mt = m.transpose();
        let sum = m + mt;
        for entry in sum.iter() {
            assert_abs_diff_eq!(*entry, 0.0, epsilon = 1.0e-15);
        }
    }
}
