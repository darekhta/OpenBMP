//! `openbmp-sensors` — OpenBMP synthetic sensors.
//!
//! Phase 2.7 ships:
//!
//! * [`error`] — [`error::SensorError`].
//! * [`noise`] — three deterministic noise primitives:
//!   [`noise::BoxMullerGaussian`], [`noise::OrnsteinUhlenbeck`],
//!   [`noise::IntegratedWhiteNoise`]. Building blocks of the IEEE
//!   952 five-component IMU noise model.
//!
//! GNSS / magnetometer / star-tracker sensors are deferred to Phase 3.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; locked operand order on every noise
//! primitive; no FMA, no wall-clock, no system RNG, no network, no
//! file I/O on the hot path. RNG seeding flows through
//! `openbmp_core::DeterministicRng::for_sensor_component`, which
//! domain-tags the seed material so per-component noise streams
//! cannot collide with `for_channel` streams.
//!
//! Bit-stability of the noise primitives is guaranteed within the
//! reference platform profile (`x86_64-unknown-linux-gnu`); cross-
//! libm bit equality is not claimed because `ln`, `sqrt`, `cos`, `sin`
//! may differ across libm implementations.
//!
//! # Crate layering
//!
//! `openbmp-sensors` is an L2 crate. It depends on `openbmp-core`
//! (L0) for the deterministic RNG and the `SensorId` newtype, and
//! **not** on `openbmp-sim` (L1) — the kernel-side adapter that
//! wires sensors into the kernel's measurement chain lands in
//! Phase 2.10.

pub mod error;
pub mod noise;

pub use error::SensorError;
pub use noise::{BoxMullerGaussian, IntegratedWhiteNoise, OrnsteinUhlenbeck};
