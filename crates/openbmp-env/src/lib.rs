//! `openbmp-env` — OpenBMP environment models.
//!
//! Phase 2.2 – 2.4 ship:
//!
//! * [`gravity`] — [`gravity::ConstantGravity`],
//!   [`gravity::PointMassGravity`], [`gravity::J2Gravity`]. WGS84
//!   defaults pinned to NIMA TR 8350.2 values.
//! * [`atmosphere`] — [`atmosphere::AtmosphereSample`],
//!   [`atmosphere::AtmosphereModel`] trait,
//!   [`atmosphere::IsothermalAtmosphere`] (toy), and
//!   [`atmosphere::UsStandard1976`] (geopotential 0–86 km, in-house
//!   port of NOAA-S/T 76-1562 / NASA-TM-X-74335).
//! * [`wind`] — [`wind::WindModel`] trait, [`wind::NoWind`] (toy),
//!   and [`wind::ConstantWind`]. Layered profiles, gust spectra, and
//!   altitude-shear winds are deferred to Phase 3.
//!
//! Magnetic field (Phase 5+) lands in its own sub-phase.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; locked operand order on every model;
//! no FMA. No wall-clock time, no system RNG, no network, no file I/O.
//!
//! # Crate layering
//!
//! `openbmp-env` is an L2 crate. It depends only on `openbmp-core`
//! (L0) and **not** on `openbmp-sim` (L1) — the kernel adapts these
//! models into its `ForceModel` / `EnvironmentModel` surfaces at a
//! higher layer. See `docs/phase-2-plan.md § Implementation Seams`.

pub mod atmosphere;
pub mod error;
pub mod gravity;
pub mod wind;

pub use atmosphere::{
    AtmosphereModel, AtmosphereSample, ExoatmosphericPolicy, IsothermalAtmosphere, UsStandard1976,
};
pub use error::EnvError;
pub use gravity::{ConstantGravity, GravityModel, J2Gravity, PointMassGravity, WGS84_J2};
pub use wind::{ConstantWind, LayerEntry, LayeredWind, NoWind, WindModel};
