//! `openbmp-testkit` — shared test helpers for the OpenBMP workspace.
//!
//! This crate is intended to be a `dev-dependency` of every other
//! OpenBMP crate. It provides:
//!
//! - [`strategies`] — `proptest::Strategy` constructors for the
//!   foundation and state types.
//! - [`analytic`] — closed-form solutions used by analytic-toy
//!   validation cases (constant-acceleration drop, torque-free Euler
//!   rigid-body, two-body Keplerian, harmonic oscillator).
//! - [`tolerance`] — `expected.toml` parser per
//!   `docs/verification.md § Tolerance Tables`.
//! - [`determinism`] — byte-stable diff utility for the determinism
//!   oracle.
//! - [`filters`] — placeholder for the Phase-4 `compare_filters`
//!   harness.
//!
//! All helpers respect the OpenBMP determinism contract: no
//! wall-clock time, no system RNG, seeded RNG only.

pub mod analytic;
pub mod determinism;
pub mod error;
pub mod filters;
pub mod strategies;
pub mod tolerance;

pub use error::TestkitError;
