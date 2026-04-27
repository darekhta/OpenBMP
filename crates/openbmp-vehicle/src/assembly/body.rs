//! Phase-3.3 vehicle-assembly body data shape.
//!
//! A [`Body`] is one rigid member of a [`crate::assembly::BasicAssembly`].
//! It carries dry mass, body-frame center of mass, body-frame inertia
//! tensor, and a [`BodyGeometry`] descriptor used by the aero deck for
//! reference-area / reference-length normalisation.
//!
//! All numeric inputs are validated finite at construction. Inertia
//! tensors are validated for symmetry and positive diagonal at
//! construction; the full positive-definite + triangle-inequality
//! check happens at kernel construction via
//! [`openbmp_state::MassProperties::require_valid`].

use nalgebra::Matrix3;
use openbmp_core::{Body as BodyFrame, BodyId, Position3};

use crate::assembly::AssemblyError;

/// Geometry descriptor for a single body. Phase-3.3 ships three
/// variants spanning the textbook single-body / two-body cases.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BodyGeometry {
    /// Right circular cylinder along the body +z axis.
    Cylinder {
        /// Cylinder length (m).
        length_m: f64,
        /// Cylinder diameter (m).
        diameter_m: f64,
    },
    /// Right circular cone whose base sits at the bottom and whose
    /// apex is at the top of the body +z axis.
    Cone {
        /// Cone length / height (m).
        length_m: f64,
        /// Cone base diameter (m).
        base_diameter_m: f64,
    },
    /// Pre-computed reference geometry for non-axisymmetric or
    /// otherwise irregular bodies. Mirrors the aero deck's
    /// `reference.{length_m, area_m2}` block.
    Reference {
        /// Reference length (m).
        length_m: f64,
        /// Reference area (m²).
        area_m2: f64,
    },
}

impl BodyGeometry {
    /// Reference length used by aero-deck normalisation.
    #[must_use]
    pub fn reference_length_m(self) -> f64 {
        match self {
            Self::Cylinder { length_m, .. }
            | Self::Cone { length_m, .. }
            | Self::Reference { length_m, .. } => length_m,
        }
    }

    /// Reference area used by aero-deck normalisation. Computed from
    /// the geometric primitives where possible.
    #[must_use]
    pub fn reference_area_m2(self) -> f64 {
        match self {
            Self::Cylinder { diameter_m, .. } => {
                std::f64::consts::PI * (diameter_m * 0.5) * (diameter_m * 0.5)
            }
            Self::Cone {
                base_diameter_m, ..
            } => std::f64::consts::PI * (base_diameter_m * 0.5) * (base_diameter_m * 0.5),
            Self::Reference { area_m2, .. } => area_m2,
        }
    }

    /// Validate all numeric components are finite and strictly
    /// positive.
    fn require_valid(self) -> Result<(), AssemblyError> {
        let (a, b) = match self {
            Self::Cylinder {
                length_m,
                diameter_m,
            } => (length_m, diameter_m),
            Self::Cone {
                length_m,
                base_diameter_m,
            } => (length_m, base_diameter_m),
            Self::Reference { length_m, area_m2 } => (length_m, area_m2),
        };
        if !a.is_finite() || !b.is_finite() || a <= 0.0 || b <= 0.0 {
            return Err(AssemblyError::InvalidBodyGeometry {
                reason: "body geometry components must be finite and strictly positive",
            });
        }
        Ok(())
    }
}

/// One rigid member of a [`crate::assembly::BasicAssembly`].
#[derive(Clone, Debug)]
#[allow(clippy::struct_field_names)] // mass-property fields carry the body-frame suffix per project convention
pub struct Body {
    id: BodyId,
    geometry: BodyGeometry,
    dry_mass_kg: f64,
    dry_cg_body: Position3<BodyFrame>,
    dry_inertia_body: Matrix3<f64>,
}

impl Body {
    /// Construct a body from its dry mass-property components.
    ///
    /// `dry_inertia_body` is validated finite and symmetric within
    /// `1e-9` tolerance with strictly positive diagonal entries. The
    /// full positive-definite + triangle-inequality check is the
    /// kernel's responsibility (it runs the same
    /// [`openbmp_state::MassProperties::require_valid`] check at
    /// kernel construction).
    ///
    /// # Errors
    ///
    /// Returns [`AssemblyError`] for non-finite mass / CG / inertia,
    /// non-positive mass, asymmetric inertia, or non-positive inertia
    /// diagonals. Geometry components are also validated.
    pub fn new(
        id: BodyId,
        geometry: BodyGeometry,
        dry_mass_kg: f64,
        dry_cg_body: Position3<BodyFrame>,
        dry_inertia_body: Matrix3<f64>,
    ) -> Result<Self, AssemblyError> {
        geometry.require_valid()?;
        if !dry_mass_kg.is_finite() || dry_mass_kg <= 0.0 {
            return Err(AssemblyError::InvalidNumber {
                field: "Body.dry_mass_kg",
                value: dry_mass_kg,
                rule: "must be finite and strictly positive",
            });
        }
        if !dry_cg_body.is_finite() {
            return Err(AssemblyError::InvalidNumber {
                field: "Body.dry_cg_body",
                value: f64::NAN,
                rule: "components must be finite",
            });
        }
        for value in dry_inertia_body.iter() {
            if !value.is_finite() {
                return Err(AssemblyError::InvalidInertia {
                    reason: "inertia tensor components must be finite",
                });
            }
        }
        for diag in 0..3 {
            if dry_inertia_body[(diag, diag)] <= 0.0 {
                return Err(AssemblyError::InvalidInertia {
                    reason: "inertia tensor diagonal must be strictly positive",
                });
            }
        }
        let symmetry_tol = 1.0e-9;
        let pairs = [(0_usize, 1_usize), (0, 2), (1, 2)];
        for (i, j) in pairs {
            if (dry_inertia_body[(i, j)] - dry_inertia_body[(j, i)]).abs() > symmetry_tol {
                return Err(AssemblyError::InvalidInertia {
                    reason: "inertia tensor must be symmetric",
                });
            }
        }
        Ok(Self {
            id,
            geometry,
            dry_mass_kg,
            dry_cg_body,
            dry_inertia_body,
        })
    }

    /// Stable identifier.
    #[must_use]
    pub const fn id(&self) -> BodyId {
        self.id
    }

    /// Geometry descriptor.
    #[must_use]
    pub const fn geometry(&self) -> BodyGeometry {
        self.geometry
    }

    /// Dry mass in kg.
    #[must_use]
    pub const fn dry_mass_kg(&self) -> f64 {
        self.dry_mass_kg
    }

    /// Body-frame center of mass.
    #[must_use]
    pub const fn dry_cg_body(&self) -> &Position3<BodyFrame> {
        &self.dry_cg_body
    }

    /// Body-frame inertia tensor.
    #[must_use]
    pub const fn dry_inertia_body(&self) -> &Matrix3<f64> {
        &self.dry_inertia_body
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;
    use nalgebra::{Matrix3, Vector3};

    fn ok_inertia() -> Matrix3<f64> {
        Matrix3::from_diagonal(&Vector3::new(0.01, 0.01, 0.001))
    }

    fn body(id: &str) -> Body {
        Body::new(
            BodyId::from_path(id),
            BodyGeometry::Cylinder {
                length_m: 0.56,
                diameter_m: 0.029,
            },
            0.080,
            Position3::origin(),
            ok_inertia(),
        )
        .expect("ok body")
    }

    #[test]
    fn constructor_accepts_valid_input() {
        let b = body("vehicle.assembly.bodies.main");
        assert!((b.dry_mass_kg() - 0.080).abs() < 1.0e-12);
        assert!(
            (b.geometry().reference_area_m2() - std::f64::consts::PI * 0.0145_f64.powi(2)).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn rejects_non_positive_mass() {
        let err = Body::new(
            BodyId::from_path("vehicle.assembly.bodies.main"),
            BodyGeometry::Cylinder {
                length_m: 1.0,
                diameter_m: 0.1,
            },
            0.0,
            Position3::origin(),
            ok_inertia(),
        )
        .unwrap_err();
        assert!(matches!(err, AssemblyError::InvalidNumber { .. }));
    }

    #[test]
    fn rejects_zero_geometry() {
        let err = Body::new(
            BodyId::from_path("vehicle.assembly.bodies.main"),
            BodyGeometry::Cone {
                length_m: 0.0,
                base_diameter_m: 0.1,
            },
            1.0,
            Position3::origin(),
            ok_inertia(),
        )
        .unwrap_err();
        assert!(matches!(err, AssemblyError::InvalidBodyGeometry { .. }));
    }

    #[test]
    fn rejects_asymmetric_inertia() {
        let mut inertia = ok_inertia();
        inertia[(0, 1)] = 0.1;
        inertia[(1, 0)] = 0.0;
        let err = Body::new(
            BodyId::from_path("b"),
            BodyGeometry::Cylinder {
                length_m: 1.0,
                diameter_m: 0.1,
            },
            1.0,
            Position3::origin(),
            inertia,
        )
        .unwrap_err();
        assert!(matches!(err, AssemblyError::InvalidInertia { .. }));
    }

    #[test]
    fn rejects_non_positive_inertia_diagonal() {
        let inertia = Matrix3::from_diagonal(&Vector3::new(1.0, 0.0, 1.0));
        let err = Body::new(
            BodyId::from_path("b"),
            BodyGeometry::Cylinder {
                length_m: 1.0,
                diameter_m: 0.1,
            },
            1.0,
            Position3::origin(),
            inertia,
        )
        .unwrap_err();
        assert!(matches!(err, AssemblyError::InvalidInertia { .. }));
    }
}
