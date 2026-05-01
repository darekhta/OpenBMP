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
/// Below the small-rotation cutoff the result is the identity
/// quaternion; above the cutoff `nalgebra`'s exact constructor is used.
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

/// Small-angle attitude-error vector from an estimate quaternion to a
/// reference quaternion, expressed in the source frame of the
/// estimate. Both inputs are `[x, y, z, w]`-ordered scalar arrays
/// (the OpenBMP bus convention) and are interpreted as unit
/// quaternions.
///
/// Returns `2 · sign(q_err.w) · vec(q_err)` where
/// `q_err = q_estimate^{-1} · q_reference`. For small attitude errors
/// this is the standard linearisation of the rotation vector that
/// takes the estimate into the reference, with `sign(q_err.w)`
/// resolving the quaternion double-cover ambiguity so the returned
/// vector is the *short-arc* rotation.
///
/// Used by attitude PID controllers and MEKF reset paths to feed an
/// error-state quaternion into a small-angle linear update.
#[must_use]
pub fn quaternion_error_small_angle(
    q_estimate_xyzw: [f64; 4],
    q_reference_xyzw: [f64; 4],
) -> Vector3<f64> {
    let q_est = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q_estimate_xyzw[3],
        q_estimate_xyzw[0],
        q_estimate_xyzw[1],
        q_estimate_xyzw[2],
    ));
    let q_ref = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q_reference_xyzw[3],
        q_reference_xyzw[0],
        q_reference_xyzw[1],
        q_reference_xyzw[2],
    ));
    let q_err = q_est.inverse() * q_ref;
    let qx = q_err.i;
    let qy = q_err.j;
    let qz = q_err.k;
    let qw = q_err.w;
    let sign = if qw >= 0.0 { 1.0 } else { -1.0 };
    Vector3::new(2.0 * qx * sign, 2.0 * qy * sign, 2.0 * qz * sign)
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

    #[test]
    fn quaternion_error_small_angle_identity_pair_is_zero() {
        let identity = [0.0, 0.0, 0.0, 1.0];
        let err = quaternion_error_small_angle(identity, identity);
        assert_eq!(err, Vector3::zeros());
    }

    #[test]
    fn quaternion_error_small_angle_small_z_rotation_recovers_axis_angle() {
        // q_ref = R_z(theta), q_est = identity. Error in body axes
        // should be ~ (0, 0, theta) for small theta.
        let theta: f64 = 1.0e-3;
        let half = theta / 2.0;
        let q_ref = [0.0, 0.0, half.sin(), half.cos()];
        let q_est = [0.0, 0.0, 0.0, 1.0];
        let err = quaternion_error_small_angle(q_est, q_ref);
        assert_abs_diff_eq!(err.x, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(err.y, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(err.z, theta, epsilon = 1.0e-9);
    }

    #[test]
    fn quaternion_error_small_angle_resolves_short_arc() {
        // q_err.w < 0 should flip sign so the returned axis points the
        // short way around. Build a rotation just past pi about z so
        // the quaternion's w is negative; the small-angle linearisation
        // is no longer accurate here, but the sign-flip rule must keep
        // the result on the short arc (negative z direction).
        let half: f64 = f64::midpoint(std::f64::consts::PI, 0.1);
        let q_ref = [0.0, 0.0, half.sin(), half.cos()];
        let q_est = [0.0, 0.0, 0.0, 1.0];
        let err = quaternion_error_small_angle(q_est, q_ref);
        // Short-arc rotation away from a +z spin past pi is the
        // residual -z spin; the sign flip is what produces this.
        assert!(err.z < 0.0);
    }
}
