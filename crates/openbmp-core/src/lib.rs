//! `openbmp-core` — OpenBMP foundation crate.
//!
//! Provides math, units, coordinate frames, simulation time, and the
//! deterministic RNG primitives used by every other OpenBMP crate.
//!
//! # Determinism contract
//!
//! `openbmp-core` exists to define and uphold the OpenBMP determinism
//! contract. Every other crate inherits these guarantees:
//!
//! * **No wall-clock time.** [`SimTime`] is monotonic seconds since
//!   scenario start. There is no `std::time` access in this crate or
//!   any model crate.
//! * **No system RNG.** [`DeterministicRng`] wraps `ChaCha8Rng` and is
//!   seeded explicitly from `(scenario_seed, step_index, channel_id)`
//!   triples. The same triple always produces the same byte stream.
//! * **No threads, no atomics, no async.** All types here are designed
//!   for synchronous, single-threaded use inside the kernel.
//! * **No allocation on hot paths.** Foundation types are `Copy` or
//!   small clone-friendly types.
//!
//! # Frames as types
//!
//! Coordinate frames are encoded in the type system. A
//! [`Position3<Eci>`] cannot be added to a [`Position3<Ecef>`]; the
//! compiler refuses. Conversions are explicit through time-aware
//! transforms whose Earth-physics implementations live in
//! `openbmp-physics::frames` (the WGS84 ellipsoid constants,
//! `LocalGeodeticOrigin`, `FrameContext`, `FrameTransform` trait +
//! impls).
//!
//! # Modules
//!
//! * [`time`] — [`SimTime`], [`Duration`], [`StepIndex`].
//! * [`frames`] — frame tag types, [`Position3`], [`Displacement3`],
//!   [`Velocity3`], [`Acceleration3`], [`AngularVelocity3`],
//!   [`Quaternion`].
//! * [`quantities`] — `uom`-typed re-exports for the public API
//!   surface.
//! * [`rng`] — [`DeterministicRng`].
//! * [`ids`] — [`ChannelId`], [`ModelId`], [`ScenarioId`].
//! * [`error`] — project-wide error types.
//! * [`validation`] — [`ValidationStatus`] enum.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

pub mod error;
pub mod frames;
pub mod ids;
pub mod quantities;
pub mod rng;
pub mod time;
pub mod validation;

pub use error::{CoreError, FrameError, TimeError};
pub use frames::{
    Acceleration3, AngularVelocity3, Body, Displacement3, Ecef, Eci, Enu, Frame, FrameId, Ned,
    Position3, Quaternion, Velocity3, VelocityDelta3,
};
pub use ids::{
    BodyId, ChannelId, EffectorId, EngineId, ModelId, RecoveryId, ScenarioId, SensorId, TankId,
    VehicleId, WindAxis,
};
pub use nalgebra::{Matrix3, UnitQuaternion, Vector3};
pub use rng::DeterministicRng;
pub use time::{Duration, SimTime, StepIndex};
pub use validation::ValidationStatus;
