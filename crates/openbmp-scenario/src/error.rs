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
    /// A scenario-referenced external file could not be read.
    #[error("referenced file {path} could not be read")]
    ReferencedFileMissing {
        /// Resolved file path that could not be read.
        path: PathBuf,
        /// Source IO error.
        source: std::io::Error,
    },
    /// A scenario-referenced external file's SHA-256 digest disagreed
    /// with the declared pin.
    #[error("SHA-256 mismatch for {path}: expected {expected}, got {actual}")]
    Sha256Mismatch {
        /// Resolved file path whose digest disagrees with the pin.
        path: PathBuf,
        /// Lower-case hex pin declared in the scenario.
        expected: String,
        /// Lower-case hex digest computed at load time.
        actual: String,
    },
    /// A SHA-256 pin string was not 64 hex characters.
    #[error("invalid SHA-256 pin for {path}: {value}")]
    InvalidSha256Pin {
        /// Resolved file path whose pin is malformed.
        path: PathBuf,
        /// Malformed pin string.
        value: String,
    },
    /// Two scenario sections disagreed about the same model selection.
    #[error("{field_a}={value_a} disagrees with {field_b}={value_b}")]
    InconsistentSection {
        /// First field.
        field_a: String,
        /// First value.
        value_a: String,
        /// Second field.
        field_b: String,
        /// Second value.
        value_b: String,
    },
    /// A field was required for the selected model but is absent.
    #[error("{field} is required for {role}={name}")]
    MissingRequiredField {
        /// Required field path.
        field: String,
        /// Role for which the field is required.
        role: ModelRole,
        /// Model name that demands the field.
        name: String,
    },
    /// A field was provided but the selected model does not accept it.
    #[error("{field} is not accepted for {role}={name}")]
    UnexpectedField {
        /// Provided field path.
        field: String,
        /// Role for which the field is unexpected.
        role: ModelRole,
        /// Model name that rejects the field.
        name: String,
    },
}
