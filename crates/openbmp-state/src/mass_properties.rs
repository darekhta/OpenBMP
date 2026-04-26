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
/// positive diagonal moments that satisfy the rigid-body triangle
/// inequalities.
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
    /// 5. Inertia diagonal satisfies rigid-body triangle inequalities.
    /// 6. Inertia tensor is symmetric within `symmetry_tolerance`.
    /// 7. Symmetric inertia tensor is strictly positive-definite.
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
        if ixx + iyy + symmetry_tolerance < izz
            || ixx + izz + symmetry_tolerance < iyy
            || iyy + izz + symmetry_tolerance < ixx
        {
            return Err(StateError::InertiaTriangleInequalityViolated { ixx, iyy, izz });
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
        let leading_minor_1 = ixx;
        let leading_minor_2 = self.inertia_body[(0, 0)] * self.inertia_body[(1, 1)]
            - self.inertia_body[(0, 1)] * self.inertia_body[(1, 0)];
        let determinant = self.inertia_body.determinant();
        if leading_minor_1 <= 0.0 || leading_minor_2 <= 0.0 || determinant <= 0.0 {
            return Err(StateError::InertiaNotPositiveDefinite {
                leading_minor_1,
                leading_minor_2,
                determinant,
            });
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
