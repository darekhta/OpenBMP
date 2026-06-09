//! `openbmp-testkit` — shared test helpers for the OpenBMP workspace.
//!
//! This crate is intended to be a `dev-dependency` of every other
//! OpenBMP crate. It provides:
//!
//! - [`strategies`] — `proptest::Strategy` constructors for the
//!   foundation vector, time, ID, and state types.
//! - [`analytic`] — closed-form solutions used by analytic-toy
//!   validation cases (constant-acceleration drop, harmonic
//!   oscillator, two-body Keplerian).
//! - [`tolerance`] — `expected.toml` parser per
//!   `docs/verification.md § Tolerance Tables`.
//! - [`determinism`] — byte-stable diff and replay utilities for the
//!   determinism oracle.
//! - [`filters`] — placeholder for the `compare_filters` harness.
//! - [`fc_lints`] — tripwires that fail CI if
//!   `openbmp-fc` imports `std::time` wall-clock APIs or regresses
//!   known hot-path allocation fixes.
//!
//! All helpers respect the OpenBMP determinism contract: no
//! wall-clock time, no system RNG, seeded RNG only.

pub mod analytic;
pub mod determinism;
pub mod error;
pub mod fc_lints;
pub mod filters;
pub mod strategies;
pub mod tolerance;

pub use error::TestkitError;
