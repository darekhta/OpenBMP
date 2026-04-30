//! `openbmp-physics` — HAL-portable physics for OpenBMP.
//!
//! Owns every physics formula and constant the workspace shares
//! between the simulator-side environment and the flight controller:
//!
//! * [`gravity`] — `GravityModel` trait + `ConstantGravity`,
//!   `PointMassGravity`, `J2Gravity`, plus WGS84 / standard-gravity
//!   constants.
//! * [`atmosphere`] — `AtmosphereModel` trait + `IsothermalAtmosphere`,
//!   `UsStandard1976` (full 7-layer, 0–86 km), plus USSA76 constants
//!   and closed-form helpers (`pressure_altitude_troposphere_m`).
//! * [`magnetic`] — `MagneticModel` trait + `EarthDipoleField`
//!   (degree-1 academic toy) + `Wmm2025` (NOAA / NCEI 2025 release,
//!   12-degree spherical harmonic).
//! * [`wind`] — `WindModel` trait + `NoWind`, `ConstantWind`,
//!   `LayeredWind`, `GustWind` (Dryden, gated by the `synthetic`
//!   feature).
//! * [`earth`] — Earth-radius constants used by low-order toy models.
//! * [`error::PhysicsError`] — uniform error type for runtime
//!   evaluation failures (out-of-envelope, non-finite, invalid
//!   parameter).
//!
//! # Layering
//!
//! `openbmp-physics` depends only on `openbmp-core` (foundation
//! types: `SimTime`, `Position3`, `Eci`, `Ned`, `Velocity3`,
//! `FrameContext`, WGS84 constants, `DeterministicRng`) and
//! `nalgebra`. **Both `openbmp-fc` (controller-side) and
//! `openbmp-sim` / `openbmp-cli` (simulator-side) consume this crate
//! directly.** No simulator trait surfaces or scenario parsing live
//! here.
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
pub mod gravity;
pub mod magnetic;
pub mod wind;

pub use atmosphere::{
    AtmosphereModel, AtmosphereSample, ExoatmosphericPolicy, IsothermalAtmosphere, UsStandard1976,
};
pub use error::PhysicsError;
pub use gravity::{
    ConstantGravity, GravityModel, J2Gravity, PointMassGravity, STANDARD_GRAVITY_M_S2, WGS84_A_M,
    WGS84_J2, WGS84_MU_M3_S2, standard_down_z_eci_m_s2,
};
pub use magnetic::{
    EARTH_DIPOLE_EQUATORIAL_FIELD_NT, EarthDipoleField, MagneticFieldEci, MagneticModel, Wmm2025,
};
pub use wind::{ConstantWind, LayerEntry, LayeredWind, NoWind, WindModel};
#[cfg(feature = "synthetic")]
pub use wind::{GustWind, GustWindParams};

/// Earth constants used by low-order academic reference models.
pub mod earth {
    /// Mean spherical Earth radius (m) used by degree-1 toy models
    /// (e.g. the `EarthDipoleField` magnetic placeholder).
    ///
    /// Distinct from the WGS84 semi-major axis used by geodetic and
    /// J2 models — see [`crate::gravity::WGS84_A_M`].
    pub const MEAN_RADIUS_M: f64 = 6_371_000.0;
}
