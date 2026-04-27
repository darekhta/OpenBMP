//! Top-level CLI error type.
//!
//! Wraps every underlying crate's error so the binary can map any
//! failure to a stable exit code (see [`CliError::exit_code`]).

use std::path::PathBuf;

use openbmp_aero::AeroError;
use openbmp_env::EnvError;
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
    /// A motor-file loader or thrust-curve evaluation failed.
    #[error("motor error")]
    Motor(#[from] MotorError),
    /// An environment-model construction or evaluation failed.
    #[error("environment error")]
    Env(#[from] EnvError),
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
            | Self::UnsupportedScenario { .. }
            | Self::Aero(_)
            | Self::Motor(_)
            | Self::Env(_) => 2,
            Self::Io { .. } => 3,
            _ => 4,
        }
    }
}
