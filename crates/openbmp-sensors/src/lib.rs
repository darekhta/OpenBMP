//! `openbmp-sensors` — OpenBMP synthetic sensors.
//!
//! Phase 2.7 ships:
//!
//! * [`error`] — [`error::SensorError`].
//! * `noise` — three deterministic noise primitives:
//!   `BoxMullerGaussian`, `OrnsteinUhlenbeck`, and
//!   `IntegratedWhiteNoise`. Building blocks of the IEEE 952
//!   five-component IMU noise model.
//!
//! Phase 3.10 adds `SyntheticGnss`, `SyntheticMagnetometer`, and
//! `SyntheticStarTracker`.
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
//! wires sensors into the runner's measurement chain remains a later
//! integration phase.

// Always-available items: the abstract `Sensor` trait + the typed
// measurement / truth value shapes. A HAL adopter compiles
// `openbmp-sensors` with `default-features = false` and gets only
// these.
pub mod error;
pub mod sensor;

// Synthetic-side parser for IMU noise-budget TOML configs. Stays
// gated with the synthetic implementations because it parses
// budgets that only the synthetic IMU consumes.
#[cfg(feature = "synthetic")]
pub mod parser;

pub use error::SensorError;
#[cfg(feature = "synthetic")]
pub use sensor::SyntheticSensor;
pub use sensor::{Sensor, SensorMeasurement, SensorTruth};

// Synthetic noise infrastructure — gated by the `synthetic` feature.
// Default-on for the simulator binary; hardware adopters disable
// via `default-features = false`.
#[cfg(feature = "synthetic")]
pub mod barometer;
#[cfg(feature = "synthetic")]
pub mod gnss;
#[cfg(feature = "synthetic")]
pub mod ideal;
#[cfg(feature = "synthetic")]
pub mod imu;
#[cfg(feature = "synthetic")]
pub mod magnetometer;
#[cfg(feature = "synthetic")]
pub mod noise;
#[cfg(feature = "synthetic")]
pub mod star_tracker;

#[cfg(feature = "synthetic")]
pub use barometer::SyntheticBarometer;
#[cfg(feature = "synthetic")]
pub use gnss::{GnssNoiseBudget, SyntheticGnss};
#[cfg(feature = "synthetic")]
pub use ideal::IdealStateSensor;
#[cfg(feature = "synthetic")]
pub use imu::{ImuNoiseBudget, SyntheticImu, TriaxialNoiseBudget};
#[cfg(feature = "synthetic")]
pub use magnetometer::{MagnetometerNoiseBudget, SyntheticMagnetometer};
#[cfg(feature = "synthetic")]
pub use noise::{BoxMullerGaussian, IntegratedWhiteNoise, OrnsteinUhlenbeck};
#[cfg(feature = "synthetic")]
pub use star_tracker::{ARCSEC_TO_RAD, StarTrackerNoiseBudget, SyntheticStarTracker};
