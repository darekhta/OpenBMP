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
    /// Mass was non-finite or not strictly positive.
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
         (Ixx={ixx}, Iyy={iyy}, Izz={izz}) kg·m²"
    )]
    InertiaDiagonalNotPositive {
        /// `I_xx`.
        ixx: f64,
        /// `I_yy`.
        iyy: f64,
        /// `I_zz`.
        izz: f64,
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
