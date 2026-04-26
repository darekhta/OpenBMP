//! Strict Phase-1 scenario document and per-section config structs.
//!
//! All `Config`-suffixed structs use `#[serde(deny_unknown_fields)]`.
//! Validation is performed in [`ScenarioDocument::validate`], which the
//! [`crate::scenario::Scenario`] entry point calls after deserialisation.

use std::collections::BTreeMap;
use std::path::PathBuf;

use openbmp_core::ValidationStatus;
use serde::Deserialize;

use crate::checks::{
    require_finite, require_finite_array, require_non_empty, require_non_empty_list,
    require_positive, require_positive_u32, require_supported, require_unique,
    validate_frame_profile,
};
use crate::error::ScenarioError;
use crate::registry::{ModelRegistry, ModelRole};
use crate::solver::SolverConfig;

/// Scenario schema version supported by this crate.
pub const SUPPORTED_SCENARIO_VERSION: u16 = 1;

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
    /// Validate semantic constraints against a model registry.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] when the document violates the
    /// Phase-1 contract.
    pub fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        self.validate_header()?;
        self.meta.validate()?;
        self.time.validate()?;
        self.vehicle.validate(registry)?;
        self.environment.validate(registry)?;
        self.forces.validate(registry)?;
        self.telemetry.validate()?;
        if let Some(frames) = &self.frames {
            validate_frame_profile("frames.profile", &frames.profile)?;
        }
        if let Some(solver) = &self.solver {
            solver.validate()?;
        }
        if let Some(batch) = &self.batch {
            batch.validate()?;
        }
        Ok(())
    }

    fn validate_header(&self) -> Result<(), ScenarioError> {
        if self.openbmp.scenario != SUPPORTED_SCENARIO_VERSION {
            return Err(ScenarioError::UnsupportedSchemaVersion {
                found: self.openbmp.scenario,
                expected: SUPPORTED_SCENARIO_VERSION,
            });
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

impl MetaConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_non_empty("meta.name", &self.name)?;
        require_non_empty("meta.description", &self.description)?;
        Ok(())
    }
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

impl TimeConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_finite("time.start_s", self.start_s)?;
        require_finite("time.stop_s", self.stop_s)?;
        require_positive("time.dt_s", self.dt_s)?;
        if self.stop_s <= self.start_s {
            return Err(ScenarioError::InvalidNumber {
                field: "time.stop_s".to_owned(),
                value: self.stop_s,
                rule: "must be greater than time.start_s",
            });
        }
        Ok(())
    }
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

impl VehicleConfig {
    fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        registry.resolve(ModelRole::Vehicle, &self.kind)?;
        require_positive("vehicle.mass_kg", self.mass_kg)?;
        require_finite_array(
            "vehicle.initial_position_eci_m",
            &self.initial_position_eci_m,
        )?;
        require_finite_array(
            "vehicle.initial_velocity_eci_m_s",
            &self.initial_velocity_eci_m_s,
        )?;
        Ok(())
    }
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

impl EnvironmentConfig {
    fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        validate_frame_profile("environment.frame_profile", &self.frame_profile)?;
        registry.resolve(ModelRole::Gravity, &self.gravity)?;
        if self.gravity == "constant" {
            let gravity_m_s2 = self
                .gravity_m_s2
                .ok_or_else(|| ScenarioError::InvalidNumber {
                    field: "environment.gravity_m_s2".to_owned(),
                    value: f64::NAN,
                    rule: "required for constant gravity",
                })?;
            require_finite("environment.gravity_m_s2", gravity_m_s2)?;
        }
        registry.resolve(ModelRole::Atmosphere, &self.atmosphere)?;
        registry.resolve(ModelRole::Wind, &self.wind)?;
        if let Some(magnetic) = &self.magnetic {
            require_supported("environment.magnetic", magnetic, &["none"])?;
        }
        Ok(())
    }
}

/// Deterministic force ordering table.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ForcesConfig {
    /// Force model names in evaluation order.
    pub models: Vec<String>,
}

impl ForcesConfig {
    fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        require_non_empty_list("forces.models", &self.models)?;
        require_unique("forces.models", &self.models)?;
        for model in &self.models {
            registry.resolve(ModelRole::Force, model)?;
        }
        Ok(())
    }
}

/// Telemetry table.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TelemetryConfig {
    /// Output archive paths.
    #[serde(default)]
    pub output: TelemetryOutputConfig,
}

impl TelemetryConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if self.output.csv.is_none() && self.output.json.is_none() && self.output.parquet.is_none()
        {
            return Err(ScenarioError::MissingTelemetryOutput);
        }
        Ok(())
    }
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

impl BatchConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_non_empty("batch.run_id", &self.run_id)?;
        require_positive_u32("batch.worker_count", self.worker_count)?;
        if self.worker_index >= self.worker_count {
            return Err(ScenarioError::InvalidNumber {
                field: "batch.worker_index".to_owned(),
                value: f64::from(self.worker_index),
                rule: "must be less than batch.worker_count",
            });
        }
        Ok(())
    }
}
