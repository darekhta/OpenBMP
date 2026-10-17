//! Rigid-body mass properties.
//!
//! [`MassProperties`] aggregates total mass, body-frame center of mass,
//! and the body-frame inertia tensor. Inertia is stored as
//! `Matrix3<f64>` in units of kg*m^2; we keep the component type as a
//! plain scalar because nalgebra matrices do not compose ergonomically
//! with `uom`-wrapped quantities.

use nalgebra::{Matrix3, Vector3};
use openbmp_core::{Body, Position3};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::StateError;

/// Mass properties of a rigid body.
///
/// `mass` is the total mass. `center_of_mass_body` is the position of
/// the center of mass in the body-fixed frame. `inertia_body` is the
/// 3-by-3 inertia tensor in the body frame, in kg*m^2. A valid inertia
/// tensor is finite, symmetric, strictly positive-definite, and has
/// positive principal moments that satisfy the rigid-body triangle
/// inequalities, independently of the body's choice of axes.
#[derive(Copy, Clone, Debug)]
pub struct MassProperties {
    /// Total mass.
    pub mass: Mass,
    /// Center of mass in the body frame (metres).
    pub center_of_mass_body: Position3<Body>,
    /// Inertia tensor in the body frame (kg*m^2).
    pub inertia_body: Matrix3<f64>,
}

impl MassProperties {
    /// Construct from explicit parts.
    #[must_use]
    pub fn new(
        mass: Mass,
        center_of_mass_body: Position3<Body>,
        inertia_body: Matrix3<f64>,
    ) -> Self {
        Self {
            mass,
            center_of_mass_body,
            inertia_body,
        }
    }

    /// Construct with a diagonal inertia tensor `(Ixx, Iyy, Izz)`,
    /// kg*m^2.
    #[must_use]
    pub fn with_diagonal_inertia(
        mass: Mass,
        center_of_mass_body: Position3<Body>,
        ixx: f64,
        iyy: f64,
        izz: f64,
    ) -> Self {
        let inertia = Matrix3::from_diagonal(&Vector3::new(ixx, iyy, izz));
        Self::new(mass, center_of_mass_body, inertia)
    }

    /// Construct with a uniform diagonal inertia tensor
    /// `(moment, moment, moment)`, kg*m^2.
    #[must_use]
    pub fn with_uniform_inertia(
        mass: Mass,
        center_of_mass_body: Position3<Body>,
        moment: f64,
    ) -> Self {
        Self::with_diagonal_inertia(mass, center_of_mass_body, moment, moment, moment)
    }

    /// Convenience: mass in kilograms (the SI base unit).
    #[must_use]
    pub fn mass_kg(&self) -> f64 {
        self.mass.get::<kilogram>()
    }

    /// Returns `true` if all numeric components are finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.mass_kg().is_finite()
            && self.center_of_mass_body.is_finite()
            && self.inertia_body.iter().all(|v| v.is_finite())
    }

    /// Returns `true` if all mass-property invariants hold.
    #[must_use]
    pub fn is_valid(&self, symmetry_tolerance: f64) -> bool {
        self.require_valid(symmetry_tolerance).is_ok()
    }

    /// Validate mass-property invariants.
    ///
    /// Checks, in order:
    ///
    /// 1. Mass is finite and strictly positive.
    /// 2. Center of mass has finite components.
    /// 3. Inertia tensor has finite components.
    /// 4. Inertia diagonal is strictly positive.
    /// 5. Inertia tensor is symmetric within `symmetry_tolerance`.
    /// 6. Symmetric inertia tensor is strictly positive-definite.
    /// 7. Principal moments satisfy rigid-body triangle inequalities.
    ///
    /// Matrix checks use the symmetric part, scaled by its largest input
    /// component to avoid determinant overflow and underflow. The triangle
    /// check permits `symmetry_tolerance` in kg*m^2 and a small relative
    /// roundoff margin, including the equality case for planar bodies.
    ///
    /// # Errors
    ///
    /// Returns the first failed check as a [`StateError`] variant.
    pub fn require_valid(&self, symmetry_tolerance: f64) -> Result<(), StateError> {
        if !symmetry_tolerance.is_finite() || symmetry_tolerance < 0.0 {
            return Err(StateError::InvalidSymmetryTolerance {
                tolerance: symmetry_tolerance,
            });
        }
        let mass_kg = self.mass_kg();
        if !mass_kg.is_finite() {
            return Err(StateError::MassNotFinite { mass_kg });
        }
        if mass_kg <= 0.0 {
            return Err(StateError::NonPositiveMass { mass_kg });
        }
        self.center_of_mass_body.require_finite()?;
        if !self.inertia_body.iter().all(|v| v.is_finite()) {
            return Err(StateError::InertiaNotFinite);
        }
        let ixx = self.inertia_body[(0, 0)];
        let iyy = self.inertia_body[(1, 1)];
        let izz = self.inertia_body[(2, 2)];
        if ixx <= 0.0 || iyy <= 0.0 || izz <= 0.0 {
            return Err(StateError::InertiaDiagonalNotPositive { ixx, iyy, izz });
        }
        let pairs = [(0usize, 1usize), (0, 2), (1, 2)];
        let max_asymmetry = pairs
            .iter()
            .map(|&(i, j)| (self.inertia_body[(i, j)] - self.inertia_body[(j, i)]).abs())
            .fold(0.0_f64, f64::max);
        if max_asymmetry > symmetry_tolerance {
            return Err(StateError::InertiaNotSymmetric {
                tolerance: symmetry_tolerance,
                max_asymmetry,
            });
        }
        // Scaling is positive and finite because the diagonal checks passed.
        // All entries now have magnitude <= 1, so finite tensors cannot slip
        // through comparisons involving NaN from overflowing raw products.
        let scale = self
            .inertia_body
            .iter()
            .map(|value| value.abs())
            .fold(0.0_f64, f64::max);
        let mut inertia = self.inertia_body / scale;
        for (i, j) in pairs {
            let symmetric = 0.5 * (inertia[(i, j)] + inertia[(j, i)]);
            inertia[(i, j)] = symmetric;
            inertia[(j, i)] = symmetric;
        }
        let leading_minor_1 = inertia[(0, 0)];
        let leading_minor_2 = inertia[(0, 0)] * inertia[(1, 1)] - inertia[(0, 1)] * inertia[(1, 0)];
        let determinant = inertia.determinant();
        if leading_minor_1 <= 0.0 || leading_minor_2 <= 0.0 || determinant <= 0.0 {
            return Err(StateError::InertiaNotPositiveDefinite {
                leading_minor_1,
                leading_minor_2,
                determinant,
            });
        }

        // For a realizable rigid body, C = tr(I)/2 * identity - I is the
        // second-moment matrix integral(r r^T dm), hence positive semidefinite.
        // Its eigenvalues are (Ij + Ik - Ii)/2, so checking C enforces every
        // principal-moment triangle inequality even away from principal axes.
        // Shift C by the allowed margin and use fixed-size Cholesky. Direct
        // determinant expansion can lose the shift through cancellation for
        // slender planar bodies, spuriously rejecting a valid PSD tensor.
        // Bound the dimensionless tolerance before addition (a large physical
        // tolerance divided by a tiny inertia scale can otherwise overflow).
        let margin = 0.5 * (symmetry_tolerance / scale).min(6.0) + 32.0 * f64::EPSILON;
        let mut second_moment = -inertia;
        for i in 0..3 {
            let j = (i + 1) % 3;
            let k = (i + 2) % 3;
            second_moment[(i, i)] =
                0.5 * (inertia[(j, j)] + inertia[(k, k)] - inertia[(i, i)]) + margin;
        }
        if second_moment.cholesky().is_none() {
            return Err(StateError::InertiaTriangleInequalityViolated { ixx, iyy, izz });
        }
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use openbmp_core::Position3;

    fn unit_kg() -> Mass {
        Mass::new::<kilogram>(1.0)
    }

    fn body_origin() -> Position3<Body> {
        Position3::origin()
    }

    #[test]
    fn diagonal_inertia_constructor() {
        let m = MassProperties::with_diagonal_inertia(unit_kg(), body_origin(), 1.0, 2.0, 3.0);
        assert_abs_diff_eq!(m.inertia_body[(0, 0)], 1.0);
        assert_abs_diff_eq!(m.inertia_body[(1, 1)], 2.0);
        assert_abs_diff_eq!(m.inertia_body[(2, 2)], 3.0);
        assert_abs_diff_eq!(m.inertia_body[(0, 1)], 0.0);
        assert_abs_diff_eq!(m.inertia_body[(1, 0)], 0.0);
    }

    #[test]
    fn uniform_inertia_constructor() {
        let m = MassProperties::with_uniform_inertia(unit_kg(), body_origin(), 2.0);
        assert_abs_diff_eq!(m.inertia_body[(0, 0)], 2.0);
        assert_abs_diff_eq!(m.inertia_body[(1, 1)], 2.0);
        assert_abs_diff_eq!(m.inertia_body[(2, 2)], 2.0);
        assert!(m.require_valid(0.0).is_ok());
    }

    #[test]
    fn require_valid_accepts_diagonal_positive() {
        let m = MassProperties::with_diagonal_inertia(unit_kg(), body_origin(), 1.0, 2.0, 3.0);
        assert!(m.require_valid(1.0e-12).is_ok());
        assert!(m.is_finite());
        assert!(m.is_valid(1.0e-12));
    }

    #[test]
    fn require_valid_rejects_zero_mass() {
        let zero_kg = Mass::new::<kilogram>(0.0);
        let m = MassProperties::with_diagonal_inertia(zero_kg, body_origin(), 1.0, 1.0, 1.0);
        let err = m.require_valid(0.0).unwrap_err();
        assert!(matches!(err, StateError::NonPositiveMass { .. }));
    }

    #[test]
    fn require_valid_rejects_negative_mass() {
        let neg = Mass::new::<kilogram>(-1.0);
        let m = MassProperties::with_diagonal_inertia(neg, body_origin(), 1.0, 1.0, 1.0);
        let err = m.require_valid(0.0).unwrap_err();
        assert!(matches!(err, StateError::NonPositiveMass { .. }));
    }

    #[test]
    fn require_valid_rejects_nan_mass() {
        let nan = Mass::new::<kilogram>(f64::NAN);
        let m = MassProperties::with_diagonal_inertia(nan, body_origin(), 1.0, 1.0, 1.0);
        let err = m.require_valid(0.0).unwrap_err();
        assert!(matches!(err, StateError::MassNotFinite { .. }));
    }

    #[test]
    fn require_valid_rejects_nan_inertia() {
        let inertia = Matrix3::from_diagonal(&Vector3::new(1.0, f64::NAN, 1.0));
        let m = MassProperties::new(unit_kg(), body_origin(), inertia);
        let err = m.require_valid(0.0).unwrap_err();
        assert!(matches!(err, StateError::InertiaNotFinite));
    }

    #[test]
    fn require_valid_rejects_zero_diagonal() {
        let m = MassProperties::with_diagonal_inertia(unit_kg(), body_origin(), 1.0, 0.0, 1.0);
        let err = m.require_valid(0.0).unwrap_err();
        assert!(matches!(err, StateError::InertiaDiagonalNotPositive { .. }));
    }

    #[test]
    fn require_valid_rejects_triangle_inequality_violation() {
        let m = MassProperties::with_diagonal_inertia(unit_kg(), body_origin(), 1.0, 1.0, 3.0);
        let err = m.require_valid(0.0).unwrap_err();
        assert!(matches!(
            err,
            StateError::InertiaTriangleInequalityViolated { .. }
        ));
    }

    #[test]
    fn require_valid_rejects_asymmetric() {
        let mut inertia = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, 1.0));
        inertia[(0, 1)] = 0.1;
        inertia[(1, 0)] = 0.0; // off by 0.1
        let m = MassProperties::new(unit_kg(), body_origin(), inertia);
        let err = m.require_valid(1.0e-6).unwrap_err();
        assert!(matches!(err, StateError::InertiaNotSymmetric { .. }));
    }

    #[test]
    fn require_valid_rejects_indefinite_symmetric_inertia() {
        let inertia = Matrix3::new(1.0, 1.1, 0.0, 1.1, 1.0, 0.0, 0.0, 0.0, 1.0);
        let m = MassProperties::new(unit_kg(), body_origin(), inertia);
        let err = m.require_valid(1.0e-12).unwrap_err();
        assert!(matches!(err, StateError::InertiaNotPositiveDefinite { .. }));
    }

    #[test]
    fn require_valid_rejects_unphysical_principal_moments() {
        // Body-axis diagonal moments satisfy every triangle inequality,
        // but principal moments (0.1, 1.0, 1.9) do not. Rotating a tensor
        // must not hide its physically impossible mass distribution.
        let inertia = Matrix3::new(1.0, 0.9, 0.0, 0.9, 1.0, 0.0, 0.0, 0.0, 1.0);
        let m = MassProperties::new(unit_kg(), body_origin(), inertia);
        assert!(matches!(
            m.require_valid(0.0),
            Err(StateError::InertiaTriangleInequalityViolated { .. })
        ));
    }

    #[test]
    fn require_valid_accepts_rotated_planar_inertia() {
        // Principal moments (1, 2, 3) describe a planar mass distribution.
        // The limiting triangle equality remains valid off the principal axes.
        let inertia = Matrix3::new(1.5, 0.5, 0.0, 0.5, 1.5, 0.0, 0.0, 0.0, 3.0);
        let m = MassProperties::new(unit_kg(), body_origin(), inertia);
        assert!(m.require_valid(0.0).is_ok());
    }

    #[test]
    fn require_valid_accepts_slender_rotated_planar_inertia() {
        // C = u*u^T + 1e-5*w*w^T, with orthogonal unit directions
        // u = (1, 2, 3)/sqrt(14), w = (2, -1, 0)/sqrt(5). Therefore
        // I = tr(C)*identity - C has principal moments (1e-5, 1, 1.00001).
        // Expanding det(C) loses the tiny positive roundoff margin through
        // cancellation even though this planar mass distribution is valid.
        let inertia = Matrix3::new(
            0.9285734285714287,
            -0.14285314285714287,
            -0.2142857142857143,
            -0.14285314285714287,
            0.7142937142857143,
            -0.4285714285714286,
            -0.2142857142857143,
            -0.4285714285714286,
            0.35715285714285716,
        );
        let m = MassProperties::new(unit_kg(), body_origin(), inertia);
        assert!(m.require_valid(0.0).is_ok());
    }

    #[test]
    fn triangle_check_includes_the_full_second_moment_determinant() {
        // C has diagonal 1 and off-diagonal -0.6. All its 1x1 and 2x2
        // principal minors are positive, but det(C) < 0. The corresponding
        // inertia is SPD with impossible principal moments (1.4, 1.4, 3.2).
        let inertia = Matrix3::new(2.0, 0.6, 0.6, 0.6, 2.0, 0.6, 0.6, 0.6, 2.0);
        let m = MassProperties::new(unit_kg(), body_origin(), inertia);
        assert!(matches!(
            m.require_valid(0.0),
            Err(StateError::InertiaTriangleInequalityViolated { .. })
        ));
    }

    #[test]
    fn triangle_tolerance_retains_physical_units() {
        let tolerance = 1.0e-8;
        let within = MassProperties::with_diagonal_inertia(
            unit_kg(),
            body_origin(),
            1.0,
            1.0,
            2.0 + 0.5 * tolerance,
        );
        assert!(within.require_valid(tolerance).is_ok());
        let outside = MassProperties::with_diagonal_inertia(
            unit_kg(),
            body_origin(),
            1.0,
            1.0,
            2.0 + 2.0 * tolerance,
        );
        assert!(matches!(
            outside.require_valid(tolerance),
            Err(StateError::InertiaTriangleInequalityViolated { .. })
        ));
    }

    #[test]
    fn require_valid_is_independent_of_inertia_scale() {
        // Scaling length or mass preserves positive definiteness and the
        // principal-moment triangle inequalities. Raw determinant products
        // overflow/underflow even though every input component is finite.
        for scale in [1.0e-200, 1.0, 1.0e200] {
            let m = MassProperties::with_diagonal_inertia(
                unit_kg(),
                body_origin(),
                scale,
                2.0 * scale,
                3.0 * scale,
            );
            assert!(m.require_valid(0.0).is_ok(), "scale {scale}");

            let indefinite = Matrix3::new(1.0, 1.1, 0.0, 1.1, 1.0, 0.0, 0.0, 0.0, 1.0) * scale;
            let m = MassProperties::new(unit_kg(), body_origin(), indefinite);
            assert!(
                matches!(
                    m.require_valid(0.0),
                    Err(StateError::InertiaNotPositiveDefinite { .. })
                ),
                "scale {scale}"
            );

            let unphysical = Matrix3::new(1.0, 0.9, 0.0, 0.9, 1.0, 0.0, 0.0, 0.0, 1.0) * scale;
            let m = MassProperties::new(unit_kg(), body_origin(), unphysical);
            assert!(
                matches!(
                    m.require_valid(0.0),
                    Err(StateError::InertiaTriangleInequalityViolated { .. })
                ),
                "scale {scale}"
            );
        }
    }

    #[test]
    fn require_valid_rejects_indefinite_inertia_with_overflowing_raw_minors() {
        // Both raw Ixx*Iyy - Ixy*Iyx and the determinant become NaN
        // through infinity subtraction. NaN <= 0 is false, so the old
        // unscaled checks silently accepted this indefinite tensor.
        let inertia = Matrix3::new(1.0, 1.1, 0.0, 1.1, 1.0, 0.0, 0.0, 0.0, 1.0) * 1.0e200;
        let m = MassProperties::new(unit_kg(), body_origin(), inertia);
        assert!(matches!(
            m.require_valid(0.0),
            Err(StateError::InertiaNotPositiveDefinite { .. })
        ));
    }

    #[test]
    fn require_valid_rejects_invalid_tolerance() {
        let m = MassProperties::with_diagonal_inertia(unit_kg(), body_origin(), 1.0, 1.0, 1.0);
        assert!(matches!(
            m.require_valid(-1.0).unwrap_err(),
            StateError::InvalidSymmetryTolerance { .. }
        ));
        assert!(matches!(
            m.require_valid(f64::NAN).unwrap_err(),
            StateError::InvalidSymmetryTolerance { .. }
        ));
    }
}
