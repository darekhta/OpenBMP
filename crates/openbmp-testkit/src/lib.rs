//! `openbmp-testkit` — shared test helpers for the OpenBMP workspace.
//!
//! This crate provides helpers for workspace tests and host-side validation
//! commands. It provides:
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
//! - [`fc_lints`] — tripwires that fail CI if
//!   `openbmp-fc` imports `std::time` wall-clock APIs or regresses
//!   known hot-path allocation fixes.
//! - [`verification`] — manufactured-solution, observed-order, and
//!   Richardson/GCI reducers for code-verification gates.
//! - [`reconstruction`] — linear-Gaussian RTS smoothing, batch
//!   Gauss-Newton, and NEES/NIS consistency helpers.
//!
//! All helpers respect the OpenBMP determinism contract: no
//! wall-clock time, no system RNG, seeded RNG only.

pub mod analytic;
pub mod determinism;
pub mod error;
pub mod fc_lints;
pub mod reconstruction;
pub mod strategies;
pub mod tolerance;
pub mod verification;

pub use error::TestkitError;
