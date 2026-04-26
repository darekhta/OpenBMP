//! State validation errors.

use openbmp_core::{FrameError, TimeError};
use thiserror::Error;

/// Errors produced by state-type validation in [`crate`].
#[derive(Error, Debug, Clone, PartialEq)]
pub enum StateError {
    /// A wrapped [`TimeError`] from `openbmp_core::time`.
    #[error(transparent)]
    Time(#[from] TimeError),
    /// A wrapped [`FrameError`] from `openbmp_core::frames`.
    #[error(transparent)]
    Frame(#[from] FrameError),
    /// Mass was `NaN` or infinite.
    #[error("mass is not finite: {mass_kg} kg")]
    MassNotFinite {
        /// The offending mass in kilograms.
        mass_kg: f64,
    },
    /// Mass was not strictly positive.
    #[error("mass is not strictly positive: {mass_kg} kg")]
    NonPositiveMass {
        /// The offending mass in kilograms.
        mass_kg: f64,
    },
    /// At least one inertia-tensor component was `NaN` or infinite.
    #[error("inertia tensor has non-finite components")]
    InertiaNotFinite,
    /// One or more diagonal entries of the inertia tensor were not
    /// strictly positive.
    #[error(
        "inertia tensor diagonal is not strictly positive: \
         (Ixx={ixx}, Iyy={iyy}, Izz={izz}) kg*m^2"
    )]
    InertiaDiagonalNotPositive {
        /// `I_xx`.
        ixx: f64,
        /// `I_yy`.
        iyy: f64,
        /// `I_zz`.
        izz: f64,
    },
    /// Diagonal inertia moments violated rigid-body triangle
    /// inequalities.
    #[error(
        "inertia tensor diagonal violates triangle inequalities: \
         (Ixx={ixx}, Iyy={iyy}, Izz={izz}) kg*m^2"
    )]
    InertiaTriangleInequalityViolated {
        /// `I_xx`.
        ixx: f64,
        /// `I_yy`.
        iyy: f64,
        /// `I_zz`.
        izz: f64,
    },
    /// A symmetric inertia tensor was not positive-definite.
    #[error(
        "inertia tensor is not positive-definite: \
         leading minors were ({leading_minor_1}, {leading_minor_2}, {determinant})"
    )]
    InertiaNotPositiveDefinite {
        /// First leading principal minor.
        leading_minor_1: f64,
        /// Second leading principal minor.
        leading_minor_2: f64,
        /// Full determinant.
        determinant: f64,
    },
    /// Off-diagonal entries violated symmetry within tolerance.
    #[error(
        "inertia tensor is not symmetric within tolerance {tolerance}: \
         max asymmetry was {max_asymmetry}"
    )]
    InertiaNotSymmetric {
        /// The acceptable tolerance.
        tolerance: f64,
        /// Largest observed `|I[i,j] - I[j,i]|`.
        max_asymmetry: f64,
    },
    /// A symmetry tolerance was outside `[0, ∞)` and finite.
    #[error("invalid symmetry tolerance: {tolerance}")]
    InvalidSymmetryTolerance {
        /// The offending value.
        tolerance: f64,
    },
}
