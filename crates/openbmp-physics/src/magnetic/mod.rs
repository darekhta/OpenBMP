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
//! `openbmp-physics` — Linux CI gate is the only proof point.
//!
//! # Crate layering
//!
//! `magnetic` is an L2 module alongside `atmosphere`, `gravity`,
//! and `wind`. It depends only on `openbmp-core` (L0) and **not**
//! on `openbmp-sim` (L1). The kernel-side adapter that wires
//! `MagneticModel` into the synthetic-magnetometer measurement
//! chain ships with Phase 3.10.C.

pub mod dipole;
pub mod wmm2025;

pub use dipole::EarthDipoleField;
pub use wmm2025::Wmm2025;

use nalgebra::Vector3;
use openbmp_core::{Eci, Position3, SimTime};

use crate::error::PhysicsError;

/// Textbook equatorial surface magnetic-field magnitude (nT) for the
/// degree-1 dipole placeholder.
pub const EARTH_DIPOLE_EQUATORIAL_FIELD_NT: f64 = 30_000.0;

/// Trait implemented by full geodetic-NED magnetic-field models.
///
/// [`Wmm2025`] is the canonical sim-side impl: full 12-degree
/// spherical harmonic, NOAA / NCEI 2025 dataset, NED output. The
/// kernel-side adapter rotates NED → body via the active
/// [`openbmp_core::FrameContext`].
///
/// Models that produce ECI directly (e.g. [`EarthDipoleField`])
/// implement [`MagneticFieldEci`] instead. Both traits coexist so
/// FC-side estimators (which work in ECI without geodetic
/// machinery) and sim-side magnetometer adapters (which work in NED
/// + frame context) each consume the trait that fits.
pub trait MagneticModel {
    /// Sample the geodetic-NED magnetic flux density at a given
    /// inertial position and time.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the model's
    /// validity envelope rejects the query (e.g. WMM 2025 outside
    /// `[2025.0, 2030.0]` decimal-year range), and
    /// [`PhysicsError::NonFinite`] when intermediate evaluation
    /// produces a non-finite result.
    fn field_ned_nt(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError>;
}

/// Trait implemented by magnetic-field models that emit ECI vectors
/// directly. Used by the FC's estimators which operate in ECI without
/// dragging in geodetic-conversion machinery.
///
/// Out-of-envelope inputs return a finite vector (clamped to the
/// nearest physical fallback), never an error — the FC's hot path
/// must remain total. Models that need to surface envelope errors
/// implement [`MagneticModel`] instead.
pub trait MagneticFieldEci: std::fmt::Debug + Send + Sync {
    /// Sample the magnetic flux density at an ECI position and
    /// simulation time, in **nanotesla (nT)**, expressed in ECI.
    fn field_eci_nt(&self, position_eci_m: Vector3<f64>, time: SimTime) -> Vector3<f64>;
}
