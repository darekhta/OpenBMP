//! Error type for the scenario crate.

use std::path::PathBuf;

use thiserror::Error;

use crate::registry::ModelRole;

/// Scenario parser and validation error.
#[derive(Debug, Error)]
pub enum ScenarioError {
    /// TOML parsing or serde decoding failed.
    #[error("failed to parse scenario TOML")]
    ParseToml(#[from] toml::de::Error),
    /// Reading a scenario file failed.
    #[error("failed to read scenario file {path}")]
    ReadFile {
        /// File path that could not be read.
        path: PathBuf,
        /// Source IO error.
        source: std::io::Error,
    },
    /// Scenario schema version is unsupported.
    #[error("unsupported scenario schema version {found}; expected {expected}")]
    UnsupportedSchemaVersion {
        /// Version found in the scenario.
        found: u16,
        /// Version expected by this crate.
        expected: u16,
    },
    /// A required string field is empty.
    #[error("{field} must not be empty")]
    EmptyField {
        /// Field path.
        field: String,
    },
    /// A numeric field is invalid.
    #[error("{field}={value} violates rule: {rule}")]
    InvalidNumber {
        /// Field path.
        field: String,
        /// Invalid value.
        value: f64,
        /// Human-readable rule.
        rule: &'static str,
    },
    /// A list field is empty.
    #[error("{field} must not be empty")]
    EmptyList {
        /// Field path.
        field: String,
    },
    /// A model name was not known for the required role.
    #[error("unknown {role} model {name}")]
    UnknownModel {
        /// Required model role.
        role: ModelRole,
        /// Unknown model name.
        name: String,
    },
    /// A model name exists but is registered for a different role.
    #[error("model {name} has role {actual}, expected {expected}")]
    WrongModelRole {
        /// Model name.
        name: String,
        /// Expected role.
        expected: ModelRole,
        /// Actual registered role.
        actual: ModelRole,
    },
    /// A safety-limited term was found in a scenario key or
    /// model-like value.
    #[error("safety-limited term {term} found at {path}: {value}")]
    SafetyName {
        /// TOML path where the term was found.
        path: String,
        /// Original string value.
        value: String,
        /// Forbidden term.
        term: &'static str,
    },
    /// A dimensional field did not carry a unit suffix.
    #[error("dimensional field {field} is missing an explicit unit suffix")]
    MissingUnitSuffix {
        /// Field path.
        field: String,
    },
    /// A vector field did not carry an explicit frame suffix.
    #[error("vector field {field} is missing an explicit frame suffix")]
    MissingFrameSuffix {
        /// Field path.
        field: String,
    },
    /// A duplicate string occurred in a deterministic list.
    #[error("duplicate value {value} in {field}")]
    DuplicateValue {
        /// Field path.
        field: String,
        /// Duplicate value.
        value: String,
    },
    /// A required output path group was absent.
    #[error("telemetry must declare at least one output path")]
    MissingTelemetryOutput,
    /// An enum-like string had an unsupported value.
    #[error("{field} has unsupported value {value}")]
    UnsupportedValue {
        /// Field path.
        field: String,
        /// Unsupported value.
        value: String,
    },
}
