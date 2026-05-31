//! Top-level CLI error type.
//!
//! Wraps the runner error plus the CLI's own IO, diff, and archive-read
//! failures so the binary can map any failure to a stable exit code
//! (see [`CliError::exit_code`]).

use std::path::PathBuf;

use openbmp_runner::RunnerError;
use openbmp_scenario::ScenarioError;
use openbmp_telemetry::TelemetryError;
use thiserror::Error;

/// Errors surfaced by the CLI.
#[derive(Debug, Error)]
pub enum CliError {
    /// A scenario file could not be parsed or validated (CLI-side
    /// `check` / `run` load before the runner sees it).
    #[error("scenario error")]
    Scenario(#[from] ScenarioError),
    /// The runner failed to build or drive the simulation.
    #[error("runner error")]
    Run(#[from] RunnerError),
    /// Writing a telemetry archive (CSV / JSON / Parquet) failed.
    #[error("telemetry error")]
    Telemetry(#[from] TelemetryError),
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
    /// A telemetry comparison exceeded its declared envelope.
    #[error("telemetry comparison: {summary}")]
    TelemetryCompare {
        /// Short, human-readable comparison failure summary.
        summary: String,
    },
    /// A telemetry comparison mapping is malformed.
    #[error("telemetry comparison config error: {path}: {summary}")]
    TelemetryCompareConfig {
        /// Offending mapping path.
        path: PathBuf,
        /// Short, human-readable configuration error.
        summary: String,
    },
    /// A telemetry comparison report could not be serialized.
    #[error("telemetry comparison report json error: {path}")]
    TelemetryCompareReportJson {
        /// Offending output path.
        path: PathBuf,
        /// Source JSON serialization error.
        #[source]
        source: serde_json::Error,
    },
    /// CSV parsing failed.
    #[error("csv error: {path}")]
    Csv {
        /// Offending CSV path.
        path: PathBuf,
        /// Source CSV error.
        #[source]
        source: csv::Error,
    },
    /// A Parquet read failed.
    #[error("parquet read error")]
    Parquet(#[from] parquet::errors::ParquetError),
    /// An Arrow read failed.
    #[error("arrow read error")]
    Arrow(#[from] arrow::error::ArrowError),
}

impl CliError {
    /// Stable exit code for the binary.
    ///
    /// `1` is reserved for "operation completed but result was negative"
    /// (e.g. diff found a difference); `2` for argument and configuration
    /// errors; `3` for IO; `4` for everything else. Runner failures
    /// delegate to [`RunnerError::exit_code`].
    #[must_use]
    pub const fn exit_code(&self) -> u8 {
        match self {
            Self::Diff { .. } | Self::TelemetryCompare { .. } => 1,
            Self::Scenario(_) | Self::TelemetryCompareConfig { .. } => 2,
            Self::Run(err) => err.exit_code(),
            Self::Io { .. } | Self::Csv { .. } => 3,
            Self::Telemetry(_)
            | Self::TelemetryCompareReportJson { .. }
            | Self::Parquet(_)
            | Self::Arrow(_) => 4,
        }
    }
}
