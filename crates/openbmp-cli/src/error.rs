//! Top-level CLI error type.
//!
//! Wraps every underlying crate's error so the binary can map any
//! failure to a stable exit code (see [`CliError::exit_code`]).

use std::path::PathBuf;

use openbmp_aero::AeroError;
use openbmp_physics::PhysicsError;
use openbmp_propulsion::MotorError;
use openbmp_scenario::ScenarioError;
use openbmp_sim::SimulationError;
use openbmp_telemetry::TelemetryError;
use thiserror::Error;

/// Errors surfaced by the CLI.
#[derive(Debug, Error)]
pub enum CliError {
    /// The scenario file could not be parsed or validated.
    #[error("scenario error")]
    Scenario(#[from] ScenarioError),
    /// The parsed scenario could not be resolved into a vehicle assembly.
    #[error("vehicle assembly error at {field}: {reason}")]
    Assembly {
        /// Scenario field path where resolution failed.
        field: String,
        /// Human-readable assembly construction failure.
        reason: String,
    },
    /// A control effector failed to construct or step.
    #[error("effector error at {field}: {reason}")]
    Effector {
        /// Scenario field path where the failure occurred.
        field: String,
        /// Human-readable effector failure.
        reason: String,
    },
    /// An engine failed to construct, command, or step.
    #[error("engine error at {field}: {reason}")]
    Engine {
        /// Scenario field path where the failure occurred.
        field: String,
        /// Human-readable engine failure.
        reason: String,
    },
    /// A tank failed to construct, drain, or step.
    #[error("tank error at {field}: {reason}")]
    Tank {
        /// Scenario field path where the failure occurred.
        field: String,
        /// Human-readable tank failure.
        reason: String,
    },
    /// A recovery device failed to construct or accept a deploy /
    /// stow command.
    #[error("recovery error at {field}: {reason}")]
    Recovery {
        /// Scenario field path where the failure occurred.
        field: String,
        /// Human-readable recovery failure.
        reason: String,
    },
    /// The kernel could not be constructed or stepped.
    #[error("simulation error")]
    Simulation(#[from] SimulationError),
    /// A telemetry channel, schema, or exporter failed.
    #[error("telemetry error")]
    Telemetry(#[from] TelemetryError),
    /// The scenario uses a model the Phase-1 runner does not yet
    /// support.
    #[error("Phase-1 runner does not yet support: {what}")]
    UnsupportedScenario {
        /// Human-readable description of the unsupported feature.
        what: String,
    },
    /// IO failed.
    #[error("io error: {path}")]
    Io {
        /// Offending path.
        path: PathBuf,
        /// Source IO error.
        #[source]
        source: std::io::Error,
    },
    /// Two telemetry archives differ.
    #[error("telemetry diff: {summary}")]
    Diff {
        /// Short, human-readable divergence description.
        summary: String,
    },
    /// A Parquet read failed.
    #[error("parquet read error")]
    Parquet(#[from] parquet::errors::ParquetError),
    /// An Arrow read failed.
    #[error("arrow read error")]
    Arrow(#[from] arrow::error::ArrowError),
    /// An aerodynamic-deck loader or sample evaluation failed.
    #[error("aerodynamic deck error")]
    Aero(#[from] AeroError),
    /// A schema-2 aero deck axis could not be matched against a
    /// scenario effector at runner build time (Phase 3.5.C).
    #[error("aero/effector mismatch at {field}: {reason}")]
    AeroEffectorMismatch {
        /// Scenario or deck field path where the mismatch was
        /// detected.
        field: String,
        /// Human-readable description of the mismatch (missing
        /// effector, unit suffix mismatch, etc.).
        reason: String,
    },
    /// A motor-file loader or thrust-curve evaluation failed.
    #[error("motor error")]
    Motor(#[from] MotorError),
    /// A physics-model construction or evaluation failed.
    #[error("physics error")]
    Env(#[from] PhysicsError),
}

impl CliError {
    /// Stable exit code for the binary.
    ///
    /// `1` is reserved for "operation completed but result was negative"
    /// (e.g. diff found a difference); `2` for argument and configuration
    /// errors; `3` for IO; `4` for everything else.
    #[must_use]
    pub const fn exit_code(&self) -> u8 {
        match self {
            Self::Diff { .. } => 1,
            Self::Scenario(_)
            | Self::Assembly { .. }
            | Self::Effector { .. }
            | Self::Engine { .. }
            | Self::Tank { .. }
            | Self::Recovery { .. }
            | Self::UnsupportedScenario { .. }
            | Self::Aero(_)
            | Self::AeroEffectorMismatch { .. }
            | Self::Motor(_)
            | Self::Env(_) => 2,
            Self::Io { .. } => 3,
            _ => 4,
        }
    }
}
