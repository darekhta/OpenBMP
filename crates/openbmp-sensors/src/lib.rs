//! `openbmp-sensors` — OpenBMP synthetic sensors and fault models.
//!
//! `SyntheticSensor` trait. Implementations: `IdealStateSensor`,
//! `SyntheticImu` (Allan-variance per IEEE 1139), `SyntheticBarometer`,
//! `SyntheticGnss`, `SyntheticMagnetometer`, `SyntheticStarTracker`.
//! Fault models: `StuckFault`, `DropoutFault`, `BiasShiftFault`,
//! `NoiseSpikeFault`.
//!
//! **No device drivers, no real bus protocols, no real sensor
//! parameters.**
//!
//! **Status:** Phase 2/3 stub.
