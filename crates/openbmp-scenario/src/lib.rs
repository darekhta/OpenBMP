//! `openbmp-scenario` — OpenBMP scenario format and parser.
//!
//! Phase 1.5 provides a strict in-house TOML scenario parser. The
//! parser rejects unknown top-level tables and unknown fields in the
//! Phase-1 schema, validates model names through a compile-time registry,
//! checks dimensional field suffixes, rejects safety-limited operational
//! vocabulary, and resolves relative paths against the scenario file
//! directory.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

use openbmp_core::ValidationStatus;
use serde::Deserialize;
use thiserror::Error;

/// Scenario schema version supported by this crate.
pub const SUPPORTED_SCENARIO_VERSION: u16 = 1;

const FORBIDDEN_SAFETY_TERMS: &[(&str, &str)] = &[
    ("target", "target"),
    ("seeker", "seeker"),
    ("warhead", "warhead"),
    ("strike", "strike"),
    ("interceptor", "interceptor"),
    ("kill", "kill"),
    ("threat", "threat"),
    ("engagement", "engagement"),
    ("terminalhoming", "terminal-homing"),
    ("terminalwaypoint", "terminal-waypoint"),
    ("impactpoint", "impact-point"),
    ("weapon", "weapon"),
];

/// Scenario parser and validation error.
#[derive(Debug, Error)]
pub enum ScenarioError {
    /// TOML parsing or serde decoding failed.
    #[error("failed to parse scenario TOML")]
    ParseToml {
        /// Source TOML error.
        #[from]
        source: toml::de::Error,
    },
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
    /// A safety-limited term was found in a scenario key or model-like value.
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

/// Compile-time model role used by the Phase-1 registry.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum ModelRole {
    /// Vehicle model.
    Vehicle,
    /// Gravity model.
    Gravity,
    /// Atmosphere model.
    Atmosphere,
    /// Wind model.
    Wind,
    /// Virtual controller model.
    Controller,
    /// Force-model entry in deterministic force ordering.
    Force,
}

impl std::fmt::Display for ModelRole {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let label = match self {
            Self::Vehicle => "vehicle",
            Self::Gravity => "gravity",
            Self::Atmosphere => "atmosphere",
            Self::Wind => "wind",
            Self::Controller => "controller",
            Self::Force => "force",
        };
        f.write_str(label)
    }
}

/// Registered model descriptor.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ModelDescriptor {
    /// Stable model name as used in scenario TOML.
    pub name: String,
    /// Model role.
    pub role: ModelRole,
}

impl ModelDescriptor {
    /// Construct a model descriptor.
    #[must_use]
    pub fn new(name: impl Into<String>, role: ModelRole) -> Self {
        Self {
            name: name.into(),
            role,
        }
    }
}

/// Compile-time model registry for scenario validation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ModelRegistry {
    models: BTreeMap<String, ModelDescriptor>,
}

impl ModelRegistry {
    /// Return the Phase-1 registry.
    #[must_use]
    pub fn phase1() -> Self {
        let descriptors = [
            ModelDescriptor::new("point_mass", ModelRole::Vehicle),
            ModelDescriptor::new("constant", ModelRole::Gravity),
            ModelDescriptor::new("none", ModelRole::Atmosphere),
            ModelDescriptor::new("none", ModelRole::Wind),
            ModelDescriptor::new("noop", ModelRole::Controller),
            ModelDescriptor::new("gravity", ModelRole::Force),
        ];
        let mut models = BTreeMap::new();
        for descriptor in descriptors {
            let key = registry_key(descriptor.role, &descriptor.name);
            models.insert(key, descriptor);
        }
        Self { models }
    }

    /// Resolve a model by role and name.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError::UnknownModel`] when the `(role, name)`
    /// pair is not registered.
    pub fn resolve(&self, role: ModelRole, name: &str) -> Result<&ModelDescriptor, ScenarioError> {
        let key = registry_key(role, name);
        self.models.get(&key).ok_or_else(|| {
            let actual = self
                .models
                .values()
                .find(|descriptor| descriptor.name == name)
                .map(|descriptor| descriptor.role);
            match actual {
                Some(actual) => ScenarioError::WrongModelRole {
                    name: name.to_owned(),
                    expected: role,
                    actual,
                },
                None => ScenarioError::UnknownModel {
                    role,
                    name: name.to_owned(),
                },
            }
        })
    }
}

impl Default for ModelRegistry {
    fn default() -> Self {
        Self::phase1()
    }
}

/// Parsed scenario plus source-directory context.
#[derive(Clone, Debug, PartialEq)]
pub struct Scenario {
    /// Parsed scenario document.
    pub document: ScenarioDocument,
    source_dir: Option<PathBuf>,
}

impl Scenario {
    /// Parse and validate a scenario from a TOML string.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for TOML parse failures and validation
    /// failures.
    pub fn from_toml_str(toml: &str) -> Result<Self, ScenarioError> {
        Self::from_toml_str_with_source_dir(toml, None::<PathBuf>)
    }

    /// Parse and validate a scenario from a TOML string with a source directory.
    ///
    /// Relative paths are resolved against `source_dir`.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for TOML parse failures and validation
    /// failures.
    pub fn from_toml_str_with_source_dir(
        toml: &str,
        source_dir: Option<impl Into<PathBuf>>,
    ) -> Result<Self, ScenarioError> {
        let value: toml::Value = toml::from_str(toml)?;
        lint_safety_names(&value)?;
        lint_dimensional_suffixes(&value)?;
        let document: ScenarioDocument = toml::from_str(toml)?;
        let scenario = Self {
            document,
            source_dir: source_dir.map(Into::into),
        };
        scenario.validate_with_registry(&ModelRegistry::phase1())?;
        Ok(scenario)
    }

    /// Parse and validate a scenario file.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError::ReadFile`] when the file cannot be read,
    /// or validation errors when parsing fails.
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, ScenarioError> {
        let path = path.as_ref();
        let content = fs::read_to_string(path).map_err(|source| ScenarioError::ReadFile {
            path: path.to_path_buf(),
            source,
        })?;
        let source_dir = path.parent().map(Path::to_path_buf);
        Self::from_toml_str_with_source_dir(&content, source_dir)
    }

    /// Validate against a model registry.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for invalid values or unknown models.
    pub fn validate_with_registry(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        self.document.validate(registry)
    }

    /// Resolve a path relative to the scenario file directory.
    #[must_use]
    pub fn resolve_path(&self, path: impl AsRef<Path>) -> PathBuf {
        let path = path.as_ref();
        if path.is_absolute() {
            return path.to_path_buf();
        }
        match &self.source_dir {
            Some(source_dir) => source_dir.join(path),
            None => path.to_path_buf(),
        }
    }

    /// Return declared telemetry and data-package paths after resolution.
    #[must_use]
    pub fn resolved_paths(&self) -> BTreeMap<String, PathBuf> {
        let mut paths = BTreeMap::new();
        if let Some(path) = &self.document.telemetry.output.csv {
            paths.insert("telemetry.output.csv".to_owned(), self.resolve_path(path));
        }
        if let Some(path) = &self.document.telemetry.output.json {
            paths.insert("telemetry.output.json".to_owned(), self.resolve_path(path));
        }
        if let Some(path) = &self.document.telemetry.output.parquet {
            paths.insert(
                "telemetry.output.parquet".to_owned(),
                self.resolve_path(path),
            );
        }
        if let Some(data_packages) = &self.document.data_packages {
            for (name, path) in data_packages {
                paths.insert(format!("data_packages.{name}"), self.resolve_path(path));
            }
        }
        paths
    }
}

/// Strict Phase-1 scenario document.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ScenarioDocument {
    /// OpenBMP schema header.
    pub openbmp: OpenBmpHeader,
    /// Scenario metadata.
    pub meta: MetaConfig,
    /// Time and deterministic seed configuration.
    pub time: TimeConfig,
    /// Vehicle initial state and model selection.
    pub vehicle: VehicleConfig,
    /// Environment model selection.
    pub environment: EnvironmentConfig,
    /// Deterministic force ordering.
    pub forces: ForcesConfig,
    /// Telemetry output configuration.
    pub telemetry: TelemetryConfig,
    /// Runtime validation switches.
    pub validation: ValidationConfig,
    /// Optional epoch metadata.
    pub epoch: Option<EpochConfig>,
    /// Optional frame profile metadata.
    pub frames: Option<FramesConfig>,
    /// Optional solver profile metadata.
    pub solver: Option<SolverConfig>,
    /// Optional data-package sidecar paths.
    pub data_packages: Option<BTreeMap<String, PathBuf>>,
    /// Optional synthetic sensor hook table.
    pub sensors: Option<BTreeMap<String, toml::Value>>,
    /// Optional virtual flight-controller hook table.
    pub fc: Option<BTreeMap<String, toml::Value>>,
    /// Optional fault-injection hook table.
    pub faults: Option<BTreeMap<String, toml::Value>>,
    /// Optional batch metadata.
    pub batch: Option<BatchConfig>,
}

impl ScenarioDocument {
    /// Validate semantic constraints.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] when the document violates the Phase-1
    /// contract.
    pub fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        if self.openbmp.scenario != SUPPORTED_SCENARIO_VERSION {
            return Err(ScenarioError::UnsupportedSchemaVersion {
                found: self.openbmp.scenario,
                expected: SUPPORTED_SCENARIO_VERSION,
            });
        }
        require_non_empty("meta.name", &self.meta.name)?;
        require_non_empty("meta.description", &self.meta.description)?;
        require_finite("time.start_s", self.time.start_s)?;
        require_finite("time.stop_s", self.time.stop_s)?;
        require_positive("time.dt_s", self.time.dt_s)?;
        if self.time.stop_s <= self.time.start_s {
            return Err(ScenarioError::InvalidNumber {
                field: "time.stop_s".to_owned(),
                value: self.time.stop_s,
                rule: "must be greater than time.start_s",
            });
        }
        registry.resolve(ModelRole::Vehicle, &self.vehicle.kind)?;
        require_positive("vehicle.mass_kg", self.vehicle.mass_kg)?;
        require_finite_array(
            "vehicle.initial_position_eci_m",
            &self.vehicle.initial_position_eci_m,
        )?;
        require_finite_array(
            "vehicle.initial_velocity_eci_m_s",
            &self.vehicle.initial_velocity_eci_m_s,
        )?;
        validate_frame_profile("environment.frame_profile", &self.environment.frame_profile)?;
        registry.resolve(ModelRole::Gravity, &self.environment.gravity)?;
        if self.environment.gravity == "constant" {
            require_finite(
                "environment.gravity_m_s2",
                self.environment
                    .gravity_m_s2
                    .ok_or_else(|| ScenarioError::InvalidNumber {
                        field: "environment.gravity_m_s2".to_owned(),
                        value: f64::NAN,
                        rule: "required for constant gravity",
                    })?,
            )?;
        }
        registry.resolve(ModelRole::Atmosphere, &self.environment.atmosphere)?;
        registry.resolve(ModelRole::Wind, &self.environment.wind)?;
        if let Some(magnetic) = &self.environment.magnetic {
            require_supported("environment.magnetic", magnetic, &["none"])?;
        }
        require_non_empty_list("forces.models", &self.forces.models)?;
        require_unique("forces.models", &self.forces.models)?;
        for model in &self.forces.models {
            registry.resolve(ModelRole::Force, model)?;
        }
        if self.telemetry.output.csv.is_none()
            && self.telemetry.output.json.is_none()
            && self.telemetry.output.parquet.is_none()
        {
            return Err(ScenarioError::MissingTelemetryOutput);
        }
        if let Some(frames) = &self.frames {
            validate_frame_profile("frames.profile", &frames.profile)?;
        }
        if let Some(solver) = &self.solver {
            solver.validate()?;
        }
        if let Some(batch) = &self.batch {
            require_non_empty("batch.run_id", &batch.run_id)?;
            if batch.worker_count == 0 {
                return Err(ScenarioError::InvalidNumber {
                    field: "batch.worker_count".to_owned(),
                    value: 0.0,
                    rule: "must be greater than zero",
                });
            }
            if batch.worker_index >= batch.worker_count {
                return Err(ScenarioError::InvalidNumber {
                    field: "batch.worker_index".to_owned(),
                    value: f64::from(batch.worker_index),
                    rule: "must be less than batch.worker_count",
                });
            }
        }
        Ok(())
    }
}

/// OpenBMP schema header.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OpenBmpHeader {
    /// Scenario schema version.
    pub scenario: u16,
}

/// Scenario metadata table.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MetaConfig {
    /// Stable scenario name.
    pub name: String,
    /// Human-readable description.
    pub description: String,
    /// Validation label claimed by this scenario.
    pub validation: ValidationStatus,
    /// Optional provenance note.
    pub provenance: Option<String>,
}

/// Time and deterministic seed table.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TimeConfig {
    /// Start time in seconds since scenario start.
    pub start_s: f64,
    /// Stop time in seconds since scenario start.
    pub stop_s: f64,
    /// Base deterministic time step in seconds.
    pub dt_s: f64,
    /// Deterministic scenario seed.
    pub seed: u64,
}

/// Vehicle model and initial state table.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct VehicleConfig {
    /// Vehicle model kind.
    pub kind: String,
    /// Vehicle mass in kilograms.
    pub mass_kg: f64,
    /// Initial inertial position in metres.
    pub initial_position_eci_m: [f64; 3],
    /// Initial inertial velocity in metres per second.
    pub initial_velocity_eci_m_s: [f64; 3],
}

/// Environment model table.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EnvironmentConfig {
    /// Frame profile used by environment transforms.
    pub frame_profile: String,
    /// Gravity model name.
    pub gravity: String,
    /// Constant gravity magnitude in metres per second squared.
    pub gravity_m_s2: Option<f64>,
    /// Atmosphere model name.
    pub atmosphere: String,
    /// Wind model name.
    pub wind: String,
    /// Optional magnetic-field model name.
    pub magnetic: Option<String>,
}

/// Deterministic force ordering table.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ForcesConfig {
    /// Force model names in evaluation order.
    pub models: Vec<String>,
}

/// Telemetry table.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TelemetryConfig {
    /// Output archive paths.
    #[serde(default)]
    pub output: TelemetryOutputConfig,
}

/// Telemetry output path group.
#[derive(Clone, Debug, Default, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TelemetryOutputConfig {
    /// Optional CSV output path.
    pub csv: Option<PathBuf>,
    /// Optional JSON output path.
    pub json: Option<PathBuf>,
    /// Optional Parquet output path.
    pub parquet: Option<PathBuf>,
}

/// Runtime validation switches.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ValidationConfig {
    /// Require state finiteness at runtime.
    pub require_finite_state: bool,
    /// Require monotonic simulation time at runtime.
    pub require_monotonic_time: bool,
}

/// Optional epoch metadata.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EpochConfig {
    /// Time scale, for example `UTC`.
    pub scale: String,
    /// ISO-8601 epoch timestamp.
    pub iso8601: String,
    /// Optional leap-second table path.
    pub leap_second_table: Option<PathBuf>,
}

/// Optional frames table.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FramesConfig {
    /// Frame profile.
    pub profile: String,
}

/// Optional solver profile.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolverConfig {
    /// Solver profile.
    pub profile: Option<String>,
    /// Trajectory method.
    pub trajectory_method: Option<String>,
    /// Determinism claim.
    pub determinism: Option<String>,
    /// Adaptive-solver controls.
    pub adaptive: Option<AdaptiveSolverConfig>,
    /// Source-term sub-step controls.
    pub source_terms: Option<SourceTermSolverConfig>,
}

impl SolverConfig {
    /// Validate solver constraints.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for unsupported solver settings or
    /// invalid tolerances.
    pub fn validate(&self) -> Result<(), ScenarioError> {
        let profile = self.profile.as_deref().unwrap_or("fixed-step-explicit");
        require_supported(
            "solver.profile",
            profile,
            &[
                "fixed-step-explicit",
                "adaptive-explicit",
                "implicit-source-term",
                "partitioned-hypersonic",
            ],
        )?;
        let method = self.trajectory_method.as_deref().unwrap_or("rk4");
        require_supported(
            "solver.trajectory_method",
            method,
            &["rk4", "dopri54", "dopri853", "rkf78"],
        )?;
        let determinism = self.determinism.as_deref().unwrap_or("bit-stable");
        require_supported(
            "solver.determinism",
            determinism,
            &["bit-stable", "state-stable"],
        )?;
        if profile == "adaptive-explicit" && self.adaptive.is_none() {
            return Err(ScenarioError::UnsupportedValue {
                field: "solver.adaptive".to_owned(),
                value: "missing".to_owned(),
            });
        }
        if determinism == "bit-stable" && profile != "fixed-step-explicit" {
            return Err(ScenarioError::UnsupportedValue {
                field: "solver.determinism".to_owned(),
                value: "bit-stable adaptive/implicit solver".to_owned(),
            });
        }
        if let Some(adaptive) = &self.adaptive {
            adaptive.validate()?;
        }
        if let Some(source_terms) = &self.source_terms {
            source_terms.validate()?;
        }
        Ok(())
    }
}

/// Adaptive solver controls.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AdaptiveSolverConfig {
    /// Relative tolerance.
    pub rtol: f64,
    /// Absolute tolerance.
    pub atol: f64,
    /// Minimum time step in seconds.
    pub min_dt_s: f64,
    /// Maximum time step in seconds.
    pub max_dt_s: f64,
    /// Whether dense output is enabled.
    pub dense_output: bool,
}

impl AdaptiveSolverConfig {
    /// Validate adaptive solver controls.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for invalid numeric controls.
    pub fn validate(&self) -> Result<(), ScenarioError> {
        require_positive("solver.adaptive.rtol", self.rtol)?;
        require_positive("solver.adaptive.atol", self.atol)?;
        require_positive("solver.adaptive.min_dt_s", self.min_dt_s)?;
        require_positive("solver.adaptive.max_dt_s", self.max_dt_s)?;
        if self.max_dt_s < self.min_dt_s {
            return Err(ScenarioError::InvalidNumber {
                field: "solver.adaptive.max_dt_s".to_owned(),
                value: self.max_dt_s,
                rule: "must be greater than or equal to min_dt_s",
            });
        }
        Ok(())
    }
}

/// Source-term sub-step solver controls.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SourceTermSolverConfig {
    /// Chemistry integration method.
    pub chemistry_method: String,
    /// Number of chemistry substeps.
    pub chemistry_substeps: u32,
    /// Material integration method.
    pub material_method: String,
    /// Number of material substeps.
    pub material_substeps: u32,
    /// Nonlinear solve tolerance.
    pub nonlinear_tolerance: f64,
    /// Maximum nonlinear iterations.
    pub nonlinear_max_iter: u32,
}

impl SourceTermSolverConfig {
    /// Validate source-term solver controls.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for unsupported methods or invalid
    /// numeric controls.
    pub fn validate(&self) -> Result<(), ScenarioError> {
        require_supported(
            "solver.source_terms.chemistry_method",
            &self.chemistry_method,
            &["implicit-euler", "rosenbrock-wanner", "bdf"],
        )?;
        require_supported(
            "solver.source_terms.material_method",
            &self.material_method,
            &["implicit-euler"],
        )?;
        if self.chemistry_substeps == 0 {
            return Err(ScenarioError::InvalidNumber {
                field: "solver.source_terms.chemistry_substeps".to_owned(),
                value: 0.0,
                rule: "must be greater than zero",
            });
        }
        if self.material_substeps == 0 {
            return Err(ScenarioError::InvalidNumber {
                field: "solver.source_terms.material_substeps".to_owned(),
                value: 0.0,
                rule: "must be greater than zero",
            });
        }
        require_positive(
            "solver.source_terms.nonlinear_tolerance",
            self.nonlinear_tolerance,
        )?;
        if self.nonlinear_max_iter == 0 {
            return Err(ScenarioError::InvalidNumber {
                field: "solver.source_terms.nonlinear_max_iter".to_owned(),
                value: 0.0,
                rule: "must be greater than zero",
            });
        }
        Ok(())
    }
}

/// Optional batch metadata.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct BatchConfig {
    /// Batch manifest path.
    pub manifest: PathBuf,
    /// Stable batch run id.
    pub run_id: String,
    /// Zero-based worker index.
    pub worker_index: u32,
    /// Worker count.
    pub worker_count: u32,
}

fn registry_key(role: ModelRole, name: &str) -> String {
    format!("{role}:{name}")
}

fn require_non_empty(field: &str, value: &str) -> Result<(), ScenarioError> {
    if value.trim().is_empty() {
        Err(ScenarioError::EmptyField {
            field: field.to_owned(),
        })
    } else {
        Ok(())
    }
}

fn require_finite(field: &str, value: f64) -> Result<(), ScenarioError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value,
            rule: "must be finite",
        })
    }
}

fn require_positive(field: &str, value: f64) -> Result<(), ScenarioError> {
    if !value.is_finite() {
        return Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value,
            rule: "must be finite",
        });
    }
    if value <= 0.0 {
        return Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value,
            rule: "must be positive",
        });
    }
    Ok(())
}

fn require_finite_array(field: &str, values: &[f64]) -> Result<(), ScenarioError> {
    for value in values {
        require_finite(field, *value)?;
    }
    Ok(())
}

fn require_non_empty_list(field: &str, values: &[String]) -> Result<(), ScenarioError> {
    if values.is_empty() {
        Err(ScenarioError::EmptyList {
            field: field.to_owned(),
        })
    } else {
        Ok(())
    }
}

fn require_unique(field: &str, values: &[String]) -> Result<(), ScenarioError> {
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(value.as_str()) {
            return Err(ScenarioError::DuplicateValue {
                field: field.to_owned(),
                value: value.clone(),
            });
        }
    }
    Ok(())
}

fn require_supported(field: &str, value: &str, supported: &[&str]) -> Result<(), ScenarioError> {
    if supported.contains(&value) {
        Ok(())
    } else {
        Err(ScenarioError::UnsupportedValue {
            field: field.to_owned(),
            value: value.to_owned(),
        })
    }
}

fn validate_frame_profile(field: &str, value: &str) -> Result<(), ScenarioError> {
    require_supported(
        field,
        value,
        &[
            "toy-fixed-earth",
            "wgs84-uniform-rotation",
            "iers-tabulated",
            "spice-reference",
        ],
    )
}

fn lint_safety_names(value: &toml::Value) -> Result<(), ScenarioError> {
    lint_safety_value("$", value)
}

fn lint_safety_value(path: &str, value: &toml::Value) -> Result<(), ScenarioError> {
    match value {
        toml::Value::Table(table) => {
            for (key, value) in table {
                let child_path = format!("{path}.{key}");
                check_safety_term(&child_path, key)?;
                lint_safety_value(&child_path, value)?;
            }
        }
        toml::Value::Array(values) => {
            for (index, value) in values.iter().enumerate() {
                lint_safety_value(&format!("{path}[{index}]"), value)?;
            }
        }
        toml::Value::String(text) if should_lint_string_value(path, text) => {
            check_safety_term(path, text)?;
        }
        _ => {}
    }
    Ok(())
}

fn should_lint_string_value(path: &str, text: &str) -> bool {
    if path.ends_with(".description")
        || path.ends_with(".provenance")
        || path.starts_with("$.telemetry.output.")
        || path.starts_with("$.data_packages.")
        || path.ends_with(".manifest")
        || path.ends_with(".leap_second_table")
    {
        return false;
    }
    !(text.contains('/') || text.contains('\\'))
}

fn check_safety_term(path: &str, value: &str) -> Result<(), ScenarioError> {
    let normalized = value
        .chars()
        .filter(char::is_ascii_alphanumeric)
        .flat_map(char::to_lowercase)
        .collect::<String>();
    for (needle, term) in FORBIDDEN_SAFETY_TERMS {
        if normalized.contains(needle) {
            return Err(ScenarioError::SafetyName {
                path: path.to_owned(),
                value: value.to_owned(),
                term,
            });
        }
    }
    Ok(())
}

fn lint_dimensional_suffixes(value: &toml::Value) -> Result<(), ScenarioError> {
    lint_dimensional_value("$", value)
}

fn lint_dimensional_value(path: &str, value: &toml::Value) -> Result<(), ScenarioError> {
    match value {
        toml::Value::Table(table) => {
            for (key, value) in table {
                lint_dimensional_field(&format!("{path}.{key}"), key, value)?;
                lint_dimensional_value(&format!("{path}.{key}"), value)?;
            }
        }
        toml::Value::Array(values) => {
            for (index, value) in values.iter().enumerate() {
                lint_dimensional_value(&format!("{path}[{index}]"), value)?;
            }
        }
        _ => {}
    }
    Ok(())
}

fn lint_dimensional_field(path: &str, key: &str, value: &toml::Value) -> Result<(), ScenarioError> {
    if is_numeric_value(value) && !is_dimensionless_key(key) && !has_unit_suffix(key) {
        return Err(ScenarioError::MissingUnitSuffix {
            field: path.to_owned(),
        });
    }
    if is_numeric_vector(value) && !has_frame_suffix(key) {
        return Err(ScenarioError::MissingFrameSuffix {
            field: path.to_owned(),
        });
    }
    Ok(())
}

fn is_numeric_value(value: &toml::Value) -> bool {
    matches!(value, toml::Value::Integer(_) | toml::Value::Float(_))
        || matches!(value, toml::Value::Array(values) if !values.is_empty() && values.iter().all(is_numeric_value))
}

fn is_numeric_vector(value: &toml::Value) -> bool {
    matches!(value, toml::Value::Array(values) if values.len() == 3 && values.iter().all(is_numeric_value))
}

fn is_dimensionless_key(key: &str) -> bool {
    matches!(
        key,
        "scenario"
            | "seed"
            | "worker_index"
            | "worker_count"
            | "chemistry_substeps"
            | "material_substeps"
            | "nonlinear_max_iter"
            | "rtol"
            | "atol"
    )
}

fn has_unit_suffix(key: &str) -> bool {
    const SUFFIXES: &[&str] = &[
        "_s", "_dt_s", "_kg", "_m", "_m_s", "_m_s2", "_hz", "_rad", "_rad_s", "_pa", "_k", "_n",
        "_n_m",
    ];
    SUFFIXES.iter().any(|suffix| key.ends_with(suffix))
}

fn has_frame_suffix(key: &str) -> bool {
    const FRAMES: &[&str] = &["_eci_", "_ecef_", "_ned_", "_enu_", "_body_"];
    FRAMES.iter().any(|frame| key.contains(frame))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    const MINIMAL: &str = r#"
openbmp.scenario = 1

[meta]
name = "constant-acceleration-drop"
description = "Analytic toy scenario for vertical acceleration."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 10.0
dt_s = 0.01
seed = 42

[vehicle]
kind = "point_mass"
mass_kg = 1.0
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/constant-acceleration-drop.csv"
output.parquet = "out/constant-acceleration-drop.parquet"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    #[test]
    fn parses_minimal_scenario() {
        let scenario = Scenario::from_toml_str(MINIMAL).unwrap();
        assert_eq!(scenario.document.openbmp.scenario, 1);
        assert_eq!(
            scenario.document.meta.validation,
            ValidationStatus::ValidatedToy
        );
        assert_eq!(scenario.document.vehicle.kind, "point_mass");
    }

    #[test]
    fn resolves_paths_against_source_dir() {
        let scenario =
            Scenario::from_toml_str_with_source_dir(MINIMAL, Some(PathBuf::from("/tmp/scenario")))
                .unwrap();
        let paths = scenario.resolved_paths();
        assert_eq!(
            paths.get("telemetry.output.csv").unwrap(),
            &PathBuf::from("/tmp/scenario/out/constant-acceleration-drop.csv")
        );
    }

    #[test]
    fn rejects_unknown_fields() {
        let toml = MINIMAL.replace("mass_kg = 1.0", "mass_kg = 1.0\nextra_kg = 2.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::ParseToml { .. }));
    }

    #[test]
    fn rejects_unknown_models() {
        let toml = MINIMAL.replace("kind = \"point_mass\"", "kind = \"rigid_stick\"");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::UnknownModel { .. }));
    }

    #[test]
    fn rejects_safety_limited_names() {
        let toml = MINIMAL.replace(
            "name = \"constant-acceleration-drop\"",
            "name = \"target-demo\"",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::SafetyName { .. }));
    }

    #[test]
    fn rejects_missing_unit_suffix_before_serde_unknown_field() {
        let toml = MINIMAL.replace("mass_kg = 1.0", "mass = 1.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::MissingUnitSuffix { .. }));
    }

    #[test]
    fn rejects_missing_frame_suffix_on_numeric_vector() {
        let toml = MINIMAL.replace(
            "initial_position_eci_m = [0.0, 0.0, 0.0]",
            "initial_position_m = [0.0, 0.0, 0.0]",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::MissingFrameSuffix { .. }));
    }

    #[test]
    fn rejects_empty_force_model_list() {
        let toml = MINIMAL.replace("models = [\"gravity\"]", "models = []");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::EmptyList { .. }));
    }

    #[test]
    fn rejects_invalid_time_range() {
        let toml = MINIMAL.replace("stop_s = 10.0", "stop_s = 0.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::InvalidNumber { .. }));
    }

    #[test]
    fn rejects_missing_telemetry_outputs() {
        let toml = MINIMAL.replace(
            "output.csv = \"out/constant-acceleration-drop.csv\"\noutput.parquet = \"out/constant-acceleration-drop.parquet\"",
            "",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::MissingTelemetryOutput));
    }
}
