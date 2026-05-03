//! `openbmp-physics` — HAL-portable physics for OpenBMP.
//!
//! Owns every physics formula and constant the workspace shares
//! between the simulator-side environment and the flight controller:
//!
//! * [`frames`] — `FrameProfile`, `LocalGeodeticOrigin`, `FrameContext`,
//!   the time-aware [`FrameTransform`] impls,
//!   and the WGS84 ellipsoid constants
//!   (`WGS84_A_M`, `WGS84_INV_FLATTENING`, `WGS84_FLATTENING`,
//!   `WGS84_ECCENTRICITY_SQUARED`, `WGS84_MU_M3_S2`,
//!   `WGS84_OMEGA_RAD_S`).
//! * [`gravity`] — `GravityModel` trait + `ConstantGravity`,
//!   `PointMassGravity`, `J2Gravity`, `Egm2008ZonalGravity`, plus
//!   standard-gravity, J2, and EGM2008 zonal-harmonic constants.
//! * [`atmosphere`] — `AtmosphereModel` trait + `IsothermalAtmosphere`,
//!   `UsStandard1976` (full 7-layer, 0–86 km), plus USSA76 constants
//!   and closed-form helpers (`pressure_altitude_troposphere_m`).
//! * [`magnetic`] — `MagneticModel` and `MagneticFieldEci` traits,
//!   `EarthDipoleField` (degree-1 academic toy), and `Wmm2025`
//!   (NOAA / NCEI 2025 release, 12-degree spherical harmonic).
//! * [`wind`] — `WindModel` trait + `NoWind`, `ConstantWind`,
//!   `LayeredWind`, `GustWind` (Dryden, gated by the `synthetic`
//!   feature).
//! * [`earth`] — Earth-radius constants used by low-order toy models.
//! * [`validity`] — small finite-range helpers for model envelopes.
//! * [`error::PhysicsError`] — uniform error type for runtime
//!   evaluation failures (out-of-envelope, non-finite, invalid
//!   parameter).
//!
//! # Layering
//!
//! `openbmp-physics` depends only on `openbmp-core` (foundation
//! types: `SimTime`, `Position3`, `Eci`, `Ned`, `Velocity3`,
//! `Quaternion`, `FrameError`, `Frame` trait + tag types,
//! `DeterministicRng`) and `nalgebra`. **Both `openbmp-fc`
//! (controller-side) and `openbmp-sim` / `openbmp-cli`
//! (simulator-side) consume this crate directly.** No simulator
//! trait surfaces or scenario parsing live here.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic with locked operand order on every model;
//! no FMA, no wall-clock time, no system RNG, no network, no file
//! I/O. The optional `GustWind` model uses
//! [`openbmp_core::DeterministicRng`], whose seed is derived
//! deterministically from `(scenario_seed, step, channel_id)`.

#![deny(missing_docs)]

pub mod atmosphere;
pub mod error;
pub mod frames;
pub mod gravity;
pub mod kinematics;
pub mod magnetic;
pub mod statistics;
pub mod validity;
pub mod wind;

pub use atmosphere::{
    AtmosphereModel, AtmosphereSample, ExoatmosphericPolicy, ExponentialLayer,
    IsothermalAtmosphere, PIECEWISE_EXP_MAX_GEOMETRIC_M, PiecewiseExpExoatmosphericPolicy,
    PiecewiseExponentialAtmosphere, UsStandard1976,
};
pub use error::PhysicsError;
pub use frames::{
    FrameContext, FrameProfile, FrameTransform, LocalGeodeticOrigin, WGS84_A_M,
    WGS84_ECCENTRICITY_SQUARED, WGS84_FLATTENING, WGS84_INV_FLATTENING, WGS84_MU_M3_S2,
    WGS84_OMEGA_RAD_S,
};
pub use gravity::{
    ConstantGravity, EGM2008_J3, EGM2008_J4, EGM2008_J5, EGM2008_J6, EGM2008_MAX_DEGREE,
    Egm2008ZonalGravity, GravityModel, J2Gravity, PointMassGravity, STANDARD_GRAVITY_M_S2,
    WGS84_J2, standard_down_z_eci_m_s2,
};
pub use kinematics::{
    quaternion_error_small_angle, quaternion_from_axis_angle, quaternion_from_omega,
    renormalize_quaternion, skew_symmetric,
};
pub use magnetic::{
    EARTH_DIPOLE_EQUATORIAL_FIELD_NT, EarthDipoleField, MagneticFieldEci, MagneticModel, Wmm2025,
};
pub use statistics::{chi_square_inverse_cdf_wilson_hilferty, inverse_standard_normal_cdf};
pub use validity::HalfOpenRange;
pub use wind::{ConstantWind, LayerEntry, LayeredWind, NoWind, WindModel};
#[cfg(feature = "synthetic")]
pub use wind::{GustWind, GustWindParams};

/// Earth constants used by low-order academic reference models.
pub mod earth {
    /// Mean spherical Earth radius (m) used by degree-1 toy models
    /// (e.g. the `EarthDipoleField` magnetic placeholder).
    ///
    /// Distinct from the WGS84 semi-major axis used by geodetic and
    /// J2 models — see [`crate::frames::WGS84_A_M`].
    pub const MEAN_RADIUS_M: f64 = 6_371_000.0;
}
