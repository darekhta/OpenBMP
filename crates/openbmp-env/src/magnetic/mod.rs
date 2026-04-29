//! Phase-3.10 magnetic-field models.
//!
//! [`MagneticModel`] is the trait surface; [`wmm2025::Wmm2025`] is
//! the canonical Phase-3.10 implementation (NOAA / NGA / UK DGC
//! World Magnetic Model 2025, valid through 2030-01-01).
//!
//! # Conventions
//!
//! - Output is the **vector magnetic flux density** at the field
//!   point, expressed in geodetic NED (`+north`, `+east`, `+down`),
//!   in **nanotesla (nT)**. The WMM convention.
//! - Input position is `Position3<Eci>`; the model converts
//!   ECI → ECEF → geodetic internally.
//! - Time input is [`SimTime`] (seconds since scenario start). The
//!   model also requires a scenario *epoch* (decimal year) at
//!   construction, so it can apply WMM secular variation
//!   `g_n^m(t) = g_n^m(t0) + (t − 2025.0) · ġ_n^m`. The scenario
//!   loader supplies the decimal-year epoch.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic; no FMA; locked operand order on the
//! Schmidt-normalized Legendre recursion and the spherical-
//! harmonic summation. Cross-platform last-bit determinism for
//! `sin` / `cos` / `sqrt` is the same posture as the rest of
//! `openbmp-env` — Linux CI gate is the only proof point.
//!
//! # Crate layering
//!
//! `magnetic` is an L2 module alongside `atmosphere`, `gravity`,
//! and `wind`. It depends only on `openbmp-core` (L0) and **not**
//! on `openbmp-sim` (L1). The kernel-side adapter that wires
//! `MagneticModel` into the synthetic-magnetometer measurement
//! chain ships with Phase 3.10.C.

pub mod wmm2025;

pub use wmm2025::Wmm2025;

use nalgebra::Vector3;
use openbmp_core::{Eci, Position3, SimTime};

use crate::error::EnvError;

/// Trait implemented by magnetic-field models.
///
/// Phase 3.10 ships [`Wmm2025`] as the canonical impl. Future
/// phases may add IGRF or higher-resolution regional models.
pub trait MagneticModel {
    /// Sample the geodetic-NED magnetic flux density at a given
    /// inertial position and time.
    ///
    /// # Errors
    ///
    /// Returns [`EnvError::OutOfEnvelope`] when the model's
    /// validity envelope rejects the query (e.g. WMM 2025 outside
    /// `[2025.0, 2030.0]` decimal-year range), and
    /// [`EnvError::NonFinite`] when intermediate evaluation
    /// produces a non-finite result.
    fn field_ned_nt(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<Vector3<f64>, EnvError>;
}
