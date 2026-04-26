//! `openbmp-env` — OpenBMP environment models.
//!
//! Phase 2.2 ships:
//!
//! * [`gravity`] — [`gravity::ConstantGravity`],
//!   [`gravity::PointMassGravity`], [`gravity::J2Gravity`]. WGS84
//!   defaults pinned to NIMA TR 8350.2 values.
//!
//! Atmosphere (Phase 2.3 — US Standard 1976), wind (Phase 2.4 —
//! `NoWind` and `ConstantWind`), and magnetic field (Phase 5+) land
//! in their own sub-phases.
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

pub mod error;
pub mod gravity;

pub use error::EnvError;
pub use gravity::{ConstantGravity, GravityModel, J2Gravity, PointMassGravity, WGS84_J2};
