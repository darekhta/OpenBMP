//! `openbmp-propulsion` — OpenBMP propulsion.
//!
//! Phase 2.6 ships:
//!
//! * [`motor`] — [`motor::Motor`] trait, [`motor::MotorVariant`]
//!   (`Solid` only in Phase 2; `Liquid`, `Hybrid`, `ColdGas`
//!   deferred to Phase 3), and [`motor::SolidMotor`] with
//!   piecewise-linear thrust-curve interpolation and an
//!   impulse-weighted mass model.
//! * [`error`] — [`error::MotorError`], the crate's typed error
//!   surface (out-of-envelope, non-finite, invalid parameter,
//!   malformed motor, file I/O).
//!
//! Multi-stage motor composition (`MultiStageMotor` +
//! `SeparationEvent`) and liquid / hybrid / cold-gas variants ship
//! in Phase 3.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; locked operand order on the
//! cumulative-impulse trapezoidal table built at construction; no
//! FMA, no wall-clock, no system RNG, no network, no file I/O on the
//! hot path. The TOML parser performs file I/O at motor-load time
//! only.
//!
//! # Crate layering
//!
//! `openbmp-propulsion` is an L2 crate. It depends only on
//! third-party `serde` / `toml` / `thiserror` and **not** on
//! `openbmp-sim` (L1) — the kernel-side `ForceModel` / `MassModel`
//! adapter that wraps a [`motor::Motor`] lands in Phase 2.10
//! alongside the gravity / atmosphere / wind / aero adapters. See
//! `docs/phase-2-plan.md § Implementation Seams`.

pub mod error;
pub mod motor;
pub mod parser;

pub use error::MotorError;
pub use motor::{
    AmbientPressureCorrection, BurnSpec, Motor, MotorGeometry, MotorMeta, MotorVariant, SolidMotor,
    ThrustCurve, Validation,
};
