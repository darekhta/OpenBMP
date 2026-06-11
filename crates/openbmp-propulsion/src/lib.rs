//! `openbmp-propulsion` — OpenBMP propulsion.
//!
//! Provides:
//!
//! * [`motor`] — [`motor::Motor`] trait, [`motor::MotorVariant`]
//!   (`Solid` only), and [`motor::SolidMotor`] with piecewise-linear
//!   thrust-curve interpolation and an impulse-weighted mass model.
//!   The `Motor` trait is purely time-driven (`thrust_n_at(t)`,
//!   `mass_kg(t)`, `mass_rate_kg_s(t)`); no state machine, no
//!   gimbal, no throttle. Used for solid-propellant rockets (e.g.
//!   the Niskanen Chapter-6 benchmark).
//! * [`engine`] — [`engine::EngineModel`] trait with
//!   per-engine throttle / gimbal / ignition lifecycle.
//!   [`engine::LiquidEngine`] is the reference impl: linear ignition
//!   and shutdown transients, constant-throttle burn, gimbal applied
//!   as a locked-order pitch-then-yaw rotation. Mass flow is
//!   `thrust / (g0 · isp)`. Used for liquid (and later hybrid /
//!   cold-gas) propulsion.
//! * [`grain`] — solid-grain geometry and quasi-steady regression
//!   models that produce validated [`motor::SolidMotor`] instances
//!   from textbook/synthetic internal-ballistics constants.
//! * [`cluster`] — [`cluster::EngineCluster`] container:
//!   `Vec<Box<dyn EngineModel>>` plus body-frame mount points and a
//!   layout tag. Not a kernel-side force model — the kernel-side
//!   adapter trio in `openbmp-vehicle::adapters` consumes the
//!   per-step [`engine::EngineSnapshot`] map via the runner-pushed
//!   kernel snapshot path.
//! * [`error`] — typed error surface. [`error::MotorError`] for the
//!   solid-motor trait, [`error::EngineError`] for the liquid-engine
//!   trait. The two trait families don't intersect, so they keep
//!   separate error surfaces.
//!
//! Multi-stage motor composition (`MultiStageMotor` +
//! `SeparationEvent`) remains future work, as do hybrid and cold-gas
//! engine variants.
//!
//! # Trait families
//!
//! `Motor` and `EngineModel` are **parallel,
//! non-intersecting** trait families. Solid-propellant rockets use
//! the `Motor` path through the legacy `[propulsion.motor]`
//! scenario block; liquid-engine clusters use the `EngineModel` /
//! `EngineCluster` path through `[[vehicle.assembly.engines]]`.
//! Scenarios that try to declare both blocks are rejected at parse
//! time.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; locked operand order on the
//! cumulative-impulse trapezoidal table built at motor construction
//! and on the gimbal rotation in [`engine::LiquidEngine`]; no FMA,
//! no wall-clock, no system RNG, no network, no file I/O on the
//! hot path. The TOML parser performs file I/O at motor-load time
//! only.
//!
//! # Crate layering
//!
//! `openbmp-propulsion` is an L2 crate. It depends only on
//! `openbmp-core`, `nalgebra`, `serde`, `toml`, and `thiserror` —
//! **not** on `openbmp-sim` (L1). The kernel-side `ForceModel` /
//! `MassModel` adapters that wrap [`motor::Motor`] and
//! [`cluster::EngineCluster`] live in `openbmp-vehicle`
//! alongside the gravity / atmosphere / wind / aero adapters.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod cluster;
pub mod engine;
pub mod error;
pub mod grain;
pub mod motor;
#[cfg(feature = "parser")]
pub mod parser;

pub use cluster::{ClusterLayout, EngineCluster};
pub use engine::{
    EngineCommand, EngineFault, EngineLimits, EngineModel, EngineSnapshot, EngineState,
    LiquidEngine, LiquidEngineCStarEfficiencyBand, LiquidEngineNozzle, LiquidEnginePerformance,
    LiquidEngineScalarBand, LiquidEngineThermochemistry,
};
pub use error::{EngineError, MotorError};
pub use grain::{
    BatesGrain, EndBurnerGrain, EquilibriumInternalBallistics, GrainGeometry, GrainPropellant,
    GrainRegressionMode, GrainRegressionModel, TabulatedGrain, TransientChamber,
};
pub use motor::{
    AmbientPressureCorrection, BurnSpec, Motor, MotorGeometry, MotorMeta, MotorVariant,
    NozzleSeparationCriterion, SolidMotor, ThrustCurve, Validation,
};
pub use motor::{ChamberState, IdealNozzlePerformance, NozzlePerformance, NozzleSolution};
