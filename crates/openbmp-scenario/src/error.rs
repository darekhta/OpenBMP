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
    #[error(
        "unsupported scenario schema version: found openbmp.scenario = {found}, expected one of {supported:?}; v1 flat-vehicle scenarios were retired in Phase 3.13, see docs/scenario-format.md#migrating-v1-scenarios-to-v2"
    )]
    UnsupportedSchemaVersion {
        /// Version found in the scenario.
        found: u16,
        /// Versions accepted by this crate.
        supported: &'static [u16],
    },
    /// A scenario field is reserved for a later schema version than the
    /// header declares.
    #[error(
        "scenario field {field} requires openbmp.scenario >= {required}, but header declares {found}"
    )]
    SchemaVersionFieldReserved {
        /// Field path that triggered the rejection.
        field: String,
        /// Minimum schema version required.
        required: u16,
        /// Schema version found in the header.
        found: u16,
    },
    /// A v3 scenario element parses syntactically but its runtime
    /// consumer is not yet implemented. The schema accepts the block so
    /// authors can write it ahead of the capability landing; validation
    /// rejects it until the named capability ships.
    #[error(
        "scenario v3 element {field} parses but is not yet supported: {missing_capability}"
    )]
    ElementNotYetSupported {
        /// Field path that triggered the rejection.
        field: String,
        /// Short description of the capability that would consume it.
        missing_capability: &'static str,
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
    /// A scenario-declared event trigger kind is not yet supported.
    #[error("trigger kind {kind} is not supported: {reason}")]
    UnsupportedTriggerKind {
        /// Trigger kind string (e.g. `"scripted"`).
        kind: String,
        /// Why this kind is rejected (typically a future-phase deferral).
        reason: String,
    },
    /// A scenario-declared event action kind parses but is not yet
    /// supported by the runtime.
    #[error("action kind {kind} is not yet supported: {missing_capability}")]
    UnsupportedActionKind {
        /// Action kind string (e.g. `"engine_command"`).
        kind: String,
        /// Short description of the capability that would consume it.
        missing_capability: String,
    },
    /// A mission-graph shape error (cycle, unreachable phase, unknown
    /// id reference, etc.). The full diagnostic is in `reason`.
    #[error("mission graph error: {reason}")]
    MissionGraph {
        /// Human-readable description from the graph validator.
        reason: String,
    },
    /// A mission or phase entry references an effector id that is not
    /// declared in `[[vehicle.assembly.effectors]]`.
    #[error("{field} references unknown effector id {id}")]
    UnknownEffectorReference {
        /// Field path.
        field: String,
        /// Referenced scenario-text effector id.
        id: String,
    },
    /// A mission or phase entry references an engine id that is not
    /// declared in `[[vehicle.assembly.engines]]`.
    #[error("{field} references unknown engine id {id}")]
    UnknownEngineReference {
        /// Field path.
        field: String,
        /// Referenced scenario-text engine id.
        id: String,
    },
    /// A scenario simultaneously declares both `[propulsion.motor]`
    /// and `[[vehicle.assembly.engines]]`. Phase 3.6 contract: a
    /// vehicle uses one propulsion path or the other, never both.
    #[error(
        "scenario declares both [propulsion.motor] and [[vehicle.assembly.engines]]; pick one path per vehicle"
    )]
    AmbiguousPropulsion,
    /// A tank references a body id that is not declared in
    /// `[[vehicle.assembly.bodies]]`.
    #[error("{field} references unknown body id {value}")]
    UnknownBodyReference {
        /// Field path.
        field: String,
        /// Referenced scenario-text body id.
        value: String,
    },
    /// A `[vehicle.assembly]` entry is incompatible with another
    /// declared field (e.g. non-`RigidLiquid` slosh in a point-mass
    /// kernel, baffle declared on a non-cylindrical tank).
    #[error("{field} is incompatible: {reason}")]
    IncompatibleAssemblyEntry {
        /// Field path.
        field: String,
        /// Human-readable reason.
        reason: String,
    },
    /// A mission event's `deploy_recovery` action references a
    /// recovery-device id that is not declared in
    /// `[[vehicle.assembly.recovery]]`.
    #[error("{field} references unknown recovery id {id}")]
    UnknownRecoveryReference {
        /// Field path.
        field: String,
        /// Referenced scenario-text recovery-device id.
        id: String,
    },
    /// A mission event's `deploy_recovery` command is incompatible
    /// with the recovery device's declared kind (e.g.
    /// `deploy_drogue` against a `parachute_drag`).
    #[error("{field} command {command} is not supported by recovery kind {kind}")]
    IncompatibleRecoveryCommand {
        /// Field path of the offending event action.
        field: String,
        /// Command name (one of `deploy`, `deploy_drogue`,
        /// `deploy_main`, `stow`).
        command: String,
        /// Declared recovery-device kind (`parachute_drag`,
        /// `drogue_main`, or `drag_device`).
        kind: String,
    },
    /// A scenario `[fc]` block is internally inconsistent
    /// (missing required sub-block, invalid base rate, etc.).
    #[error("[fc] block invalid: {reason}")]
    InvalidFc {
        /// Human-readable reason.
        reason: String,
    },
}
