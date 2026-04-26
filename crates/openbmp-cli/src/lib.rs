//! `openbmp-cli` — library surface for the `openbmp` binary.
//!
//! The binary in [`src/main.rs`](crate) parses CLI arguments and
//! dispatches to one of the four entry functions exposed here. Tests
//! and downstream consumers may call them directly without spawning a
//! subprocess.
//!
//! Phase 1.7 surface:
//!
//! - [`commands::run::run`] — load a scenario, run it, write declared
//!   telemetry outputs.
//! - [`commands::diff::run`] — compare two Parquet archives row by row.
//! - [`commands::check::run`] — parse + validate a scenario, no run.
//! - [`commands::provenance::run`] — list files lacking sibling
//!   `provenance.md`.
//!
//! See `docs/software-architecture.md § Workspace Layout` for the
//! crate's place in the workspace and `docs/scenario-format.md` for
//! the scenario TOML schema this CLI parses.

pub mod cli;
pub mod commands;
pub mod error;
pub mod runner;
pub mod tracing;

pub use cli::{Cli, Command};
pub use error::CliError;
