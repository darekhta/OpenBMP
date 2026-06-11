//! Top-level CLI error type.
//!
//! Wraps the runner error plus the CLI's own IO, diff, and archive-read
//! failures so the binary can map any failure to a stable exit code
//! (see [`CliError::exit_code`]).

use std::path::PathBuf;

use openbmp_runner::RunnerError;
use openbmp_scenario::ScenarioError;
use openbmp_sil::SilError;
use openbmp_telemetry::TelemetryError;
use openbmp_trajopt::TrajoptError;
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
    /// The native SIL package/testbench API failed.
    #[error("sil error")]
    Sil(#[from] SilError),
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
    /// TOML editing failed while applying a migration.
    #[error("migration TOML parse error: {path}")]
    MigrateToml {
        /// Offending scenario path.
        path: PathBuf,
        /// TOML editor parse error.
        #[source]
        source: toml_edit::TomlError,
    },
    /// A scenario migration could not be applied.
    #[error("migration error: {path}: {summary}")]
    Migrate {
        /// Offending scenario path.
        path: PathBuf,
        /// Short, human-readable failure summary.
        summary: String,
    },
    /// A Monte Carlo utility command failed.
    #[error("monte-carlo error: {summary}")]
    MonteCarlo {
        /// Short, human-readable failure summary.
        summary: String,
    },
    /// Dictionary export failed before writing output.
    #[error("dictionary error: {summary}")]
    Dictionary {
        /// Short, human-readable failure summary.
        summary: String,
    },
    /// A code-verification command failed acceptance or could not run.
    #[error("code verification: {summary}")]
    CodeVerification {
        /// Short, human-readable failure summary.
        summary: String,
    },
    /// Offline trajectory optimization failed acceptance or could not run.
    #[error("trajectory optimization: {summary}")]
    Trajopt {
        /// Short, human-readable failure summary.
        summary: String,
    },
    /// The trajectory-optimization library rejected an input or failed
    /// serialization.
    #[error("trajectory optimization error")]
    TrajoptLib(#[from] TrajoptError),
    /// Two telemetry archives differ.
    #[error("telemetry diff: {summary}")]
    Diff {
        /// Short, human-readable divergence description.
        summary: String,
    },
    /// A telemetry diff report could not be serialized.
    #[error("telemetry diff report json error: {path}")]
    DiffReportJson {
        /// Offending output path.
        path: PathBuf,
        /// Source JSON serialization error.
        #[source]
        source: serde_json::Error,
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
            Self::CodeVerification { .. } => 1,
            Self::Trajopt { .. } => 1,
            Self::Scenario(_)
            | Self::TelemetryCompareConfig { .. }
            | Self::MigrateToml { .. }
            | Self::Migrate { .. }
            | Self::MonteCarlo { .. }
            | Self::Dictionary { .. } => 2,
            Self::Run(err) => err.exit_code(),
            Self::Io { .. } | Self::Csv { .. } => 3,
            Self::Telemetry(_)
            | Self::Sil(_)
            | Self::TrajoptLib(_)
            | Self::TelemetryCompareReportJson { .. }
            | Self::DiffReportJson { .. }
            | Self::Parquet(_)
            | Self::Arrow(_) => 4,
        }
    }
}
