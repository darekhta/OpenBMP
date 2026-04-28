//! Strict Phase-1 scenario document and per-section config structs.
//!
//! All `Config`-suffixed structs use `#[serde(deny_unknown_fields)]`.
//! Validation is performed in [`ScenarioDocument::validate`], which the
//! [`crate::scenario::Scenario`] entry point calls after deserialisation.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::path::PathBuf;

use openbmp_core::ValidationStatus;
use serde::Deserialize;

use crate::checks::{
    require_finite, require_finite_array, require_in_range, require_non_empty,
    require_non_empty_list, require_positive, require_positive_u32, require_supported,
    require_unique, validate_frame_profile,
};
use crate::error::ScenarioError;
use crate::registry::{ModelRegistry, ModelRole};
use crate::solver::SolverConfig;

/// Scenario schema version supported by this crate.
pub const SUPPORTED_SCENARIO_VERSION: u16 = 1;

/// Default unnormalised WGS84 J2 zonal coefficient used when a scenario
/// selects `gravity = "j2"` and omits `environment.j2`.
///
/// Source: NIMA TR 8350.2 (NGA WGS84), 3rd edition (2000), table 3.5.
pub const WGS84_J2_DEFAULT: f64 = 1.082_626_683e-3;

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
    /// Optional aerodynamic deck reference.
    pub aero: Option<AeroConfig>,
    /// Optional propulsion table.
    pub propulsion: Option<PropulsionConfig>,
    /// Optional structured wind block (overrides
    /// [`EnvironmentConfig::wind`] when both are present).
    pub wind: Option<WindConfig>,
    /// Optional structured atmosphere block (overrides
    /// [`EnvironmentConfig::atmosphere`] when both are present).
    pub atmosphere: Option<AtmosphereConfig>,
    /// Optional solver profile metadata.
    pub solver: Option<SolverConfig>,
    /// Optional data-package sidecar paths.
    pub data_packages: Option<BTreeMap<String, PathBuf>>,
    /// Optional synthetic sensor table.
    pub sensors: Option<BTreeMap<String, SensorConfig>>,
    /// Optional virtual flight-controller hook table.
    pub fc: Option<BTreeMap<String, toml::Value>>,
    /// Optional fault-injection hook table.
    pub faults: Option<BTreeMap<String, toml::Value>>,
    /// Optional batch metadata.
    pub batch: Option<BatchConfig>,
    /// Optional declarative mission block (Phase 3.2).
    ///
    /// When present, the runner builds an `openbmp_sim::MissionPhaseGraph`
    /// and a list of `openbmp_sim::EventBinding`s from the parsed
    /// config. When absent, the kernel runs in legacy mode with no
    /// event evaluation — Phase-1 and pre-3.2 Phase-2 scenarios stay
    /// byte-stable.
    pub mission: Option<MissionConfig>,
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
        self.vehicle.validate(registry, self.time.dt_s)?;
        self.environment.validate(registry)?;
        self.forces.validate(registry)?;
        self.telemetry.validate()?;
        if let Some(frames) = &self.frames {
            frames.validate()?;
            if self.environment.frame_profile != frames.profile {
                return Err(ScenarioError::InconsistentSection {
                    field_a: "environment.frame_profile".to_owned(),
                    value_a: self.environment.frame_profile.clone(),
                    field_b: "frames.profile".to_owned(),
                    value_b: frames.profile.clone(),
                });
            }
        }
        if let Some(aero) = &self.aero {
            aero.validate()?;
        }
        if let Some(propulsion) = &self.propulsion {
            propulsion.validate(registry)?;
        }
        if let Some(wind) = &self.wind {
            wind.validate(registry)?;
            if self.environment.wind != "none" && self.environment.wind != wind.kind {
                return Err(ScenarioError::InconsistentSection {
                    field_a: "environment.wind".to_owned(),
                    value_a: self.environment.wind.clone(),
                    field_b: "wind.kind".to_owned(),
                    value_b: wind.kind.clone(),
                });
            }
        } else if self.environment.wind == "constant" {
            return Err(ScenarioError::MissingRequiredField {
                field: "wind".to_owned(),
                role: ModelRole::Wind,
                name: "constant".to_owned(),
            });
        }
        if let Some(atmosphere) = &self.atmosphere {
            atmosphere.validate(registry)?;
            if self.environment.atmosphere != "none"
                && self.environment.atmosphere != atmosphere.kind
            {
                return Err(ScenarioError::InconsistentSection {
                    field_a: "environment.atmosphere".to_owned(),
                    value_a: self.environment.atmosphere.clone(),
                    field_b: "atmosphere.kind".to_owned(),
                    value_b: atmosphere.kind.clone(),
                });
            }
        } else if self.environment.atmosphere == "isothermal" {
            return Err(ScenarioError::MissingRequiredField {
                field: "atmosphere".to_owned(),
                role: ModelRole::Atmosphere,
                name: "isothermal".to_owned(),
            });
        }
        self.validate_force_dependencies()?;
        if let Some(sensors) = &self.sensors {
            for (name, config) in sensors {
                config.validate(name, registry)?;
            }
        }
        if let Some(solver) = &self.solver {
            solver.validate()?;
        }
        if let Some(batch) = &self.batch {
            batch.validate()?;
        }
        if let Some(mission) = &self.mission {
            mission.validate()?;
        }
        self.validate_effector_references()?;
        self.validate_engine_references()?;
        self.validate_propulsion_unambiguous()?;
        Ok(())
    }

    fn validate_propulsion_unambiguous(&self) -> Result<(), ScenarioError> {
        let has_motor = self.propulsion.as_ref().is_some_and(|p| p.motor.is_some());
        let has_engines = self
            .vehicle
            .assembly
            .as_ref()
            .is_some_and(|a| !a.engines.is_empty());
        if has_motor && has_engines {
            return Err(ScenarioError::AmbiguousPropulsion);
        }
        Ok(())
    }

    fn validate_engine_references(&self) -> Result<(), ScenarioError> {
        let Some(mission) = &self.mission else {
            return Ok(());
        };
        let declared: BTreeSet<&str> = self
            .vehicle
            .assembly
            .as_ref()
            .map(|assembly| assembly.engines.iter().map(|e| e.id.as_str()).collect())
            .unwrap_or_default();

        for (phase_index, phase) in mission.phases.iter().enumerate() {
            for (engine_index, id) in phase.allowed_engines.iter().enumerate() {
                if !declared.contains(id.as_str()) {
                    return Err(ScenarioError::UnknownEngineReference {
                        field: format!(
                            "mission.phases[{phase_index}].allowed_engines[{engine_index}]"
                        ),
                        id: id.clone(),
                    });
                }
            }
        }
        for (event_index, event) in mission.events.iter().enumerate() {
            if let EventActionConfig::EngineCommand { id, .. } = &event.action
                && !declared.contains(id.as_str())
            {
                return Err(ScenarioError::UnknownEngineReference {
                    field: format!("mission.events[{event_index}].action.id"),
                    id: id.clone(),
                });
            }
        }
        Ok(())
    }

    fn validate_effector_references(&self) -> Result<(), ScenarioError> {
        let Some(mission) = &self.mission else {
            return Ok(());
        };
        let declared: BTreeSet<&str> = self
            .vehicle
            .assembly
            .as_ref()
            .map(|assembly| assembly.effectors.iter().map(|e| e.id.as_str()).collect())
            .unwrap_or_default();

        for (phase_index, phase) in mission.phases.iter().enumerate() {
            for (effector_index, id) in phase.allowed_effectors.iter().enumerate() {
                if !declared.contains(id.as_str()) {
                    return Err(ScenarioError::UnknownEffectorReference {
                        field: format!(
                            "mission.phases[{phase_index}].allowed_effectors[{effector_index}]"
                        ),
                        id: id.clone(),
                    });
                }
            }
        }
        for (event_index, event) in mission.events.iter().enumerate() {
            if let EventActionConfig::EffectorOverride { id, .. } = &event.action
                && !declared.contains(id.as_str())
            {
                return Err(ScenarioError::UnknownEffectorReference {
                    field: format!("mission.events[{event_index}].action.id"),
                    id: id.clone(),
                });
            }
        }
        Ok(())
    }

    fn validate_force_dependencies(&self) -> Result<(), ScenarioError> {
        if self.forces.models.iter().any(|model| model == "aero") && self.aero.is_none() {
            return Err(ScenarioError::MissingRequiredField {
                field: "aero".to_owned(),
                role: ModelRole::Force,
                name: "aero".to_owned(),
            });
        }
        let has_thrust_force = self.forces.models.iter().any(|model| model == "thrust");
        let has_motor = self
            .propulsion
            .as_ref()
            .and_then(|propulsion| propulsion.motor.as_ref())
            .is_some();
        // Phase 3.6: `forces.models = ["thrust"]` is satisfied by
        // EITHER a `[propulsion.motor]` block OR a non-empty
        // `[[vehicle.assembly.engines]]` block (cluster path).
        // Ambiguous co-declaration is rejected separately by
        // `validate_propulsion_unambiguous`.
        let engine_count = self
            .vehicle
            .assembly
            .as_ref()
            .map_or(0, |a| a.engines.len());
        let has_engines = engine_count > 0;
        if has_engines && !has_thrust_force {
            return Err(ScenarioError::InconsistentSection {
                field_a: "vehicle.assembly.engines".to_owned(),
                value_a: format!("{engine_count} declared engine(s)"),
                field_b: "forces.models".to_owned(),
                value_b: "no `\"thrust\"` entry".to_owned(),
            });
        }
        if has_thrust_force && !has_motor && !has_engines {
            return Err(ScenarioError::MissingRequiredField {
                field: "propulsion.motor or vehicle.assembly.engines".to_owned(),
                role: ModelRole::Force,
                name: "thrust".to_owned(),
            });
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
    /// Initial body-to-ECI quaternion `[x, y, z, w]`. Required when
    /// `kind = "rigid_body"`, rejected otherwise.
    pub initial_quaternion_body_to_eci_xyzw: Option<[f64; 4]>,
    /// Initial body-frame angular velocity in rad/s. Required when
    /// `kind = "rigid_body"`, rejected otherwise.
    pub initial_angular_velocity_body_rad_s: Option<[f64; 3]>,
    /// Body-frame inertia tensor in kg·m², row-major
    /// `[[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]]`.
    /// Required when `kind = "rigid_body"`, rejected otherwise.
    /// Validated finite, symmetric, and positive-diagonal at parse
    /// time; full positive-definite + triangle-inequality validation
    /// happens at kernel construction via
    /// `MassProperties::require_valid`.
    pub inertia_tensor_body_kg_m2: Option<[[f64; 3]; 3]>,
    /// Optional declarative vehicle composition tree (Phase 3.3+).
    ///
    /// When present, the runner builds an
    /// `openbmp_vehicle::BasicAssembly` from the declared bodies
    /// and resolves it into the kernel's flat model lists. The
    /// legacy `mass_kg` and `inertia_tensor_body_kg_m2` flat fields
    /// must agree with the assembly's summed body masses / inertias
    /// — mutual consistency rather than mutual exclusivity, so
    /// existing scenarios can opt into the assembly tree without
    /// removing fields.
    pub assembly: Option<AssemblyConfig>,
}

impl VehicleConfig {
    fn validate(&self, registry: &ModelRegistry, dt_s: f64) -> Result<(), ScenarioError> {
        let descriptor = registry.resolve(ModelRole::Vehicle, &self.kind)?;
        require_positive("vehicle.mass_kg", self.mass_kg)?;
        require_finite_array(
            "vehicle.initial_position_eci_m",
            &self.initial_position_eci_m,
        )?;
        require_finite_array(
            "vehicle.initial_velocity_eci_m_s",
            &self.initial_velocity_eci_m_s,
        )?;
        match descriptor.name.as_str() {
            "rigid_body" => {
                let quaternion = self.initial_quaternion_body_to_eci_xyzw.ok_or_else(|| {
                    ScenarioError::MissingRequiredField {
                        field: "vehicle.initial_quaternion_body_to_eci_xyzw".to_owned(),
                        role: ModelRole::Vehicle,
                        name: "rigid_body".to_owned(),
                    }
                })?;
                require_finite_array("vehicle.initial_quaternion_body_to_eci_xyzw", &quaternion)?;
                let norm_sq: f64 = quaternion.iter().map(|q| q * q).sum();
                if (norm_sq - 1.0).abs() > 1.0e-9 {
                    return Err(ScenarioError::InvalidNumber {
                        field: "vehicle.initial_quaternion_body_to_eci_xyzw".to_owned(),
                        value: norm_sq,
                        rule: "must be a unit quaternion (||q||² = 1)",
                    });
                }
                let angular = self.initial_angular_velocity_body_rad_s.ok_or_else(|| {
                    ScenarioError::MissingRequiredField {
                        field: "vehicle.initial_angular_velocity_body_rad_s".to_owned(),
                        role: ModelRole::Vehicle,
                        name: "rigid_body".to_owned(),
                    }
                })?;
                require_finite_array("vehicle.initial_angular_velocity_body_rad_s", &angular)?;
                let inertia = self.inertia_tensor_body_kg_m2.ok_or_else(|| {
                    ScenarioError::MissingRequiredField {
                        field: "vehicle.inertia_tensor_body_kg_m2".to_owned(),
                        role: ModelRole::Vehicle,
                        name: "rigid_body".to_owned(),
                    }
                })?;
                validate_inertia_tensor(&inertia)?;
            }
            other => {
                if self.initial_quaternion_body_to_eci_xyzw.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "vehicle.initial_quaternion_body_to_eci_xyzw".to_owned(),
                        role: ModelRole::Vehicle,
                        name: other.to_owned(),
                    });
                }
                if self.initial_angular_velocity_body_rad_s.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "vehicle.initial_angular_velocity_body_rad_s".to_owned(),
                        role: ModelRole::Vehicle,
                        name: other.to_owned(),
                    });
                }
                if self.inertia_tensor_body_kg_m2.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "vehicle.inertia_tensor_body_kg_m2".to_owned(),
                        role: ModelRole::Vehicle,
                        name: other.to_owned(),
                    });
                }
            }
        }
        if let Some(assembly) = &self.assembly {
            assembly.validate(
                descriptor.name.as_str(),
                self.mass_kg,
                self.inertia_tensor_body_kg_m2.as_ref(),
                dt_s,
            )?;
        }
        Ok(())
    }
}

fn validate_inertia_tensor(inertia: &[[f64; 3]; 3]) -> Result<(), ScenarioError> {
    // Finiteness, symmetry (within 1e-9 tolerance), and positive
    // diagonal entries. Full positive-definite + triangle-inequality
    // validation lives in `MassProperties::require_valid` at kernel
    // construction.
    for (i, row) in inertia.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            if !value.is_finite() {
                return Err(ScenarioError::InvalidNumber {
                    field: format!("vehicle.inertia_tensor_body_kg_m2[{i}][{j}]"),
                    value: *value,
                    rule: "must be finite",
                });
            }
        }
        if row[i] <= 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("vehicle.inertia_tensor_body_kg_m2[{i}][{i}]"),
                value: row[i],
                rule: "diagonal moment must be strictly positive",
            });
        }
    }
    let symmetry_tolerance = 1.0e-9;
    let off_diagonals = [((0, 1), (1, 0)), ((0, 2), (2, 0)), ((1, 2), (2, 1))];
    for ((i, j), (ji, jj)) in off_diagonals {
        let upper = inertia[i][j];
        let lower = inertia[ji][jj];
        if (upper - lower).abs() > symmetry_tolerance {
            return Err(ScenarioError::InvalidNumber {
                field: format!("vehicle.inertia_tensor_body_kg_m2[{i}][{j}]"),
                value: upper - lower,
                rule: "inertia tensor must be symmetric",
            });
        }
    }
    Ok(())
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
    /// Required when `gravity = "constant"`.
    pub gravity_m_s2: Option<f64>,
    /// Earth gravitational parameter in m³/s². Required when
    /// `gravity = "point_mass"` or `gravity = "j2"`.
    pub mu_m3_s2: Option<f64>,
    /// Equatorial radius in metres. Required when `gravity = "j2"`.
    pub r_e_m: Option<f64>,
    /// J2 zonal coefficient (dimensionless). Optional when
    /// `gravity = "j2"`; defaults to the WGS84 value when absent.
    pub j2: Option<f64>,
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
        let gravity_descriptor = registry.resolve(ModelRole::Gravity, &self.gravity)?;
        match gravity_descriptor.name.as_str() {
            "constant" => {
                let gravity_m_s2 =
                    self.gravity_m_s2
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: "environment.gravity_m_s2".to_owned(),
                            role: ModelRole::Gravity,
                            name: "constant".to_owned(),
                        })?;
                require_finite("environment.gravity_m_s2", gravity_m_s2)?;
                if self.mu_m3_s2.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "environment.mu_m3_s2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "constant".to_owned(),
                    });
                }
                if self.r_e_m.is_some() || self.j2.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "environment.r_e_m / environment.j2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "constant".to_owned(),
                    });
                }
            }
            "point_mass" => {
                let mu = self
                    .mu_m3_s2
                    .ok_or_else(|| ScenarioError::MissingRequiredField {
                        field: "environment.mu_m3_s2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "point_mass".to_owned(),
                    })?;
                require_positive("environment.mu_m3_s2", mu)?;
                if self.gravity_m_s2.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "environment.gravity_m_s2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "point_mass".to_owned(),
                    });
                }
                if self.r_e_m.is_some() || self.j2.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "environment.r_e_m / environment.j2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "point_mass".to_owned(),
                    });
                }
            }
            "j2" => {
                let mu = self
                    .mu_m3_s2
                    .ok_or_else(|| ScenarioError::MissingRequiredField {
                        field: "environment.mu_m3_s2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "j2".to_owned(),
                    })?;
                require_positive("environment.mu_m3_s2", mu)?;
                let r_e = self
                    .r_e_m
                    .ok_or_else(|| ScenarioError::MissingRequiredField {
                        field: "environment.r_e_m".to_owned(),
                        role: ModelRole::Gravity,
                        name: "j2".to_owned(),
                    })?;
                require_positive("environment.r_e_m", r_e)?;
                if let Some(j2) = self.j2 {
                    require_finite("environment.j2", j2)?;
                }
                if self.gravity_m_s2.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "environment.gravity_m_s2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "j2".to_owned(),
                    });
                }
            }
            _ => {}
        }
        registry.resolve(ModelRole::Atmosphere, &self.atmosphere)?;
        registry.resolve(ModelRole::Wind, &self.wind)?;
        if let Some(magnetic) = &self.magnetic {
            require_supported("environment.magnetic", magnetic, &["none"])?;
        }
        Ok(())
    }

    /// Return the scenario J2 coefficient after applying the WGS84
    /// default for `gravity = "j2"`.
    #[must_use]
    pub fn j2_or_wgs84_default(&self) -> Option<f64> {
        if self.gravity == "j2" {
            Some(self.j2.unwrap_or(WGS84_J2_DEFAULT))
        } else {
            None
        }
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
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FramesConfig {
    /// Frame profile.
    pub profile: String,
    /// Optional declared scenario local origin (geodetic).
    pub local_origin: Option<LocalOriginConfig>,
}

impl FramesConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        validate_frame_profile("frames.profile", &self.profile)?;
        if let Some(local_origin) = &self.local_origin {
            local_origin.validate()?;
        }
        Ok(())
    }
}

/// Geodetic launch-site reference declared by the scenario.
///
/// Phase-2 scenarios that select `frames.profile =
/// "wgs84-uniform-rotation"` declare a local origin so altitude, NED
/// wind, and vertical-launch initialisation are anchored to a stable
/// reference. The Phase-2.11 runner consumes this block; Phase 2.10
/// only validates the field shapes.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct LocalOriginConfig {
    /// Geodetic latitude in degrees, range \[−90, 90\].
    pub latitude_deg: f64,
    /// Geodetic longitude in degrees, range \[−180, 180\].
    pub longitude_deg: f64,
    /// WGS84 height in metres above the reference ellipsoid.
    pub height_m: f64,
    /// Provenance string for the declared origin.
    pub source: String,
}

impl LocalOriginConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_in_range(
            "frames.local_origin.latitude_deg",
            self.latitude_deg,
            -90.0,
            90.0,
        )?;
        require_in_range(
            "frames.local_origin.longitude_deg",
            self.longitude_deg,
            -180.0,
            180.0,
        )?;
        require_finite("frames.local_origin.height_m", self.height_m)?;
        require_non_empty("frames.local_origin.source", &self.source)?;
        Ok(())
    }
}

/// Aerodynamic deck reference.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroConfig {
    /// Path to a Phase-2.5 aero deck TOML file (resolved relative to
    /// the scenario directory).
    pub deck: PathBuf,
    /// Optional pinned SHA-256 digest (lower-case hex). When present,
    /// a mismatch with the file's actual digest fails closed.
    pub deck_sha256: Option<String>,
}

impl AeroConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if self.deck.as_os_str().is_empty() {
            return Err(ScenarioError::EmptyField {
                field: "aero.deck".to_owned(),
            });
        }
        Ok(())
    }
}

/// Propulsion table.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct PropulsionConfig {
    /// Optional motor reference.
    pub motor: Option<MotorConfig>,
}

impl PropulsionConfig {
    fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        if let Some(motor) = &self.motor {
            motor.validate(registry)?;
        }
        Ok(())
    }
}

/// Motor reference inside `[propulsion.motor]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MotorConfig {
    /// Path to a Phase-2.6 motor TOML file (resolved relative to the
    /// scenario directory).
    pub file: PathBuf,
    /// Ignition time in seconds since scenario start.
    pub ignite_at_s: f64,
    /// Optional motor variant (defaults to whatever the motor file
    /// declares; when present, must match).
    pub variant: Option<String>,
    /// Optional pinned SHA-256 digest of the motor file.
    pub file_sha256: Option<String>,
}

impl MotorConfig {
    fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        if self.file.as_os_str().is_empty() {
            return Err(ScenarioError::EmptyField {
                field: "propulsion.motor.file".to_owned(),
            });
        }
        require_finite("propulsion.motor.ignite_at_s", self.ignite_at_s)?;
        if let Some(variant) = &self.variant {
            registry.resolve(ModelRole::Motor, variant)?;
        }
        Ok(())
    }
}

/// Structured wind block.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct WindConfig {
    /// Wind model name (must match a registered wind model). Phase
    /// 2.4 ships `"none"` and `"constant"`; Phase 3.8 adds
    /// `"layered"` and `"gust"`.
    pub kind: String,
    /// Constant wind in NED frame, m/s. Required when
    /// `kind = "constant"`; rejected for every other kind.
    pub wind_ned_m_s: Option<[f64; 3]>,
    /// Phase-3.8.B: per-altitude NED wind table. Required when
    /// `kind = "layered"`; rejected for every other kind. Layers
    /// must be strictly ascending in altitude.
    #[serde(default)]
    pub layers: Option<Vec<WindLayerConfig>>,
}

impl WindConfig {
    fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        registry.resolve(ModelRole::Wind, &self.kind)?;
        match self.kind.as_str() {
            "constant" => {
                let vector =
                    self.wind_ned_m_s
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: "wind.wind_ned_m_s".to_owned(),
                            role: ModelRole::Wind,
                            name: "constant".to_owned(),
                        })?;
                require_finite_array("wind.wind_ned_m_s", &vector)?;
                if self.layers.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "wind.layers".to_owned(),
                        role: ModelRole::Wind,
                        name: "constant".to_owned(),
                    });
                }
            }
            "layered" => {
                if self.wind_ned_m_s.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "wind.wind_ned_m_s".to_owned(),
                        role: ModelRole::Wind,
                        name: "layered".to_owned(),
                    });
                }
                let layers = self
                    .layers
                    .as_ref()
                    .ok_or_else(|| ScenarioError::MissingRequiredField {
                        field: "wind.layers".to_owned(),
                        role: ModelRole::Wind,
                        name: "layered".to_owned(),
                    })?;
                if layers.is_empty() {
                    return Err(ScenarioError::EmptyList {
                        field: "wind.layers".to_owned(),
                    });
                }
                for (index, layer) in layers.iter().enumerate() {
                    layer.validate(index)?;
                }
                for window in layers.windows(2) {
                    if window[1].altitude_m <= window[0].altitude_m {
                        return Err(ScenarioError::InvalidNumber {
                            field: "wind.layers[*].altitude_m".to_owned(),
                            value: window[1].altitude_m,
                            rule: "altitudes must be strictly ascending across layers",
                        });
                    }
                }
            }
            other => {
                if self.wind_ned_m_s.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "wind.wind_ned_m_s".to_owned(),
                        role: ModelRole::Wind,
                        name: other.to_owned(),
                    });
                }
                if self.layers.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "wind.layers".to_owned(),
                        role: ModelRole::Wind,
                        name: other.to_owned(),
                    });
                }
            }
        }
        Ok(())
    }
}

/// One row of a Phase-3.8.B `[[wind.layers]]` table.
///
/// `altitude_m` is metres above the launch-pad reference (matching
/// the Phase-2.4 axial-drag adapter convention). `wind_ned_m_s` is
/// the NED wind vector at that altitude. The runner builds a
/// `LayeredWind` from the parsed table; layer ordering is
/// scenario-declared (the validator rejects non-ascending altitudes
/// at parse time).
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct WindLayerConfig {
    /// Altitude in metres at which this layer's wind applies.
    pub altitude_m: f64,
    /// NED wind vector in m/s.
    pub wind_ned_m_s: [f64; 3],
}

impl WindLayerConfig {
    fn validate(self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("wind.layers[{index}].{field}");
        require_finite(&path("altitude_m"), self.altitude_m)?;
        require_finite_array(&path("wind_ned_m_s"), &self.wind_ned_m_s)?;
        Ok(())
    }
}

/// Structured atmosphere block.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AtmosphereConfig {
    /// Atmosphere model name (must match a registered atmosphere model).
    pub kind: String,
    /// Density in kg/m³. Required when `kind = "isothermal"`.
    pub density_kg_m3: Option<f64>,
    /// Pressure in pascals. Required when `kind = "isothermal"`.
    pub pressure_pa: Option<f64>,
    /// Temperature in kelvin. Required when `kind = "isothermal"`.
    pub temperature_k: Option<f64>,
}

impl AtmosphereConfig {
    fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        registry.resolve(ModelRole::Atmosphere, &self.kind)?;
        match self.kind.as_str() {
            "isothermal" => {
                let density =
                    self.density_kg_m3
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: "atmosphere.density_kg_m3".to_owned(),
                            role: ModelRole::Atmosphere,
                            name: "isothermal".to_owned(),
                        })?;
                require_positive("atmosphere.density_kg_m3", density)?;
                let pressure =
                    self.pressure_pa
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: "atmosphere.pressure_pa".to_owned(),
                            role: ModelRole::Atmosphere,
                            name: "isothermal".to_owned(),
                        })?;
                require_positive("atmosphere.pressure_pa", pressure)?;
                let temperature =
                    self.temperature_k
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: "atmosphere.temperature_k".to_owned(),
                            role: ModelRole::Atmosphere,
                            name: "isothermal".to_owned(),
                        })?;
                require_positive("atmosphere.temperature_k", temperature)?;
            }
            other => {
                if self.density_kg_m3.is_some()
                    || self.pressure_pa.is_some()
                    || self.temperature_k.is_some()
                {
                    return Err(ScenarioError::UnexpectedField {
                        field: "atmosphere.density_kg_m3 / pressure_pa / temperature_k".to_owned(),
                        role: ModelRole::Atmosphere,
                        name: other.to_owned(),
                    });
                }
            }
        }
        Ok(())
    }
}

/// Synthetic-sensor entry under `[sensors.<name>]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SensorConfig {
    /// Sensor kind (must match a registered sensor model).
    pub kind: String,
    /// Path to the sensor's noise-budget TOML file (Phase-2.7 schema).
    /// Required for `kind = "imu"` and `kind = "barometer"`, rejected
    /// for `kind = "ideal_state"`.
    pub file: Option<PathBuf>,
    /// Optional pinned SHA-256 digest of the sensor noise budget file.
    pub file_sha256: Option<String>,
}

impl SensorConfig {
    fn validate(&self, name: &str, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        registry.resolve(ModelRole::Sensor, &self.kind)?;
        match self.kind.as_str() {
            "ideal_state" => {
                if self.file.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: format!("sensors.{name}.file"),
                        role: ModelRole::Sensor,
                        name: "ideal_state".to_owned(),
                    });
                }
                if self.file_sha256.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: format!("sensors.{name}.file_sha256"),
                        role: ModelRole::Sensor,
                        name: "ideal_state".to_owned(),
                    });
                }
            }
            kind => {
                let file =
                    self.file
                        .as_ref()
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("sensors.{name}.file"),
                            role: ModelRole::Sensor,
                            name: kind.to_owned(),
                        })?;
                if file.as_os_str().is_empty() {
                    return Err(ScenarioError::EmptyField {
                        field: format!("sensors.{name}.file"),
                    });
                }
            }
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

// ---------------------------------------------------------------------
// Mission block (Phase 3.2)
// ---------------------------------------------------------------------

/// Top-level `[mission]` block.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MissionConfig {
    /// Id of the initial phase.
    pub initial_phase: String,
    /// Declared phases.
    #[serde(default)]
    pub phases: Vec<PhaseConfig>,
    /// Declared events.
    #[serde(default)]
    pub events: Vec<EventConfig>,
    /// Declared transitions between phases.
    #[serde(default)]
    pub transitions: Vec<PhaseTransitionConfig>,
}

impl MissionConfig {
    /// Validate structural shape: id non-emptiness, kind enums, finite
    /// numeric values, deferred-action rejection, plus graph-shape
    /// validation (cycles, reachability, unknown ids).
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for any structural violation.
    pub fn validate(&self) -> Result<(), ScenarioError> {
        require_non_empty("mission.initial_phase", &self.initial_phase)?;
        if self.phases.is_empty() {
            return Err(ScenarioError::EmptyList {
                field: "mission.phases".to_owned(),
            });
        }
        for (i, phase) in self.phases.iter().enumerate() {
            phase.validate(i)?;
        }
        for (i, event) in self.events.iter().enumerate() {
            event.validate(i)?;
        }
        for (i, transition) in self.transitions.iter().enumerate() {
            transition.validate(i)?;
        }
        self.validate_graph_shape()?;
        Ok(())
    }

    fn validate_graph_shape(&self) -> Result<(), ScenarioError> {
        let phase_ids: Vec<String> = self.phases.iter().map(|phase| phase.id.clone()).collect();
        let event_ids: Vec<String> = self.events.iter().map(|event| event.id.clone()).collect();
        require_unique("mission.phases.id", &phase_ids)?;
        require_unique("mission.events.id", &event_ids)?;

        let phase_set: BTreeSet<&str> = phase_ids.iter().map(String::as_str).collect();
        let event_set: BTreeSet<&str> = event_ids.iter().map(String::as_str).collect();

        if !phase_set.contains(self.initial_phase.as_str()) {
            return Err(ScenarioError::MissionGraph {
                reason: format!(
                    "mission.initial_phase = `{}` does not match any declared phase",
                    self.initial_phase
                ),
            });
        }

        let mut transition_keys = BTreeSet::new();
        for (index, transition) in self.transitions.iter().enumerate() {
            if !phase_set.contains(transition.from.as_str()) {
                return Err(ScenarioError::MissionGraph {
                    reason: format!(
                        "transition #{index}.from = `{}` is not a declared phase",
                        transition.from
                    ),
                });
            }
            if !phase_set.contains(transition.to.as_str()) {
                return Err(ScenarioError::MissionGraph {
                    reason: format!(
                        "transition #{index}.to = `{}` is not a declared phase",
                        transition.to
                    ),
                });
            }
            if !event_set.contains(transition.event.as_str()) {
                return Err(ScenarioError::MissionGraph {
                    reason: format!(
                        "transition #{index}.event = `{}` is not a declared event",
                        transition.event
                    ),
                });
            }
            if !transition_keys.insert((transition.from.as_str(), transition.event.as_str())) {
                return Err(ScenarioError::MissionGraph {
                    reason: format!(
                        "phase `{}` has multiple transitions for event `{}`",
                        transition.from, transition.event
                    ),
                });
            }
        }

        if let Some(involving) = mission_cycle_involving(&phase_ids, &self.transitions) {
            return Err(ScenarioError::MissionGraph {
                reason: format!("mission graph contains a cycle involving phases {involving:?}"),
            });
        }

        let reachable = mission_reachable(&phase_ids, &self.transitions, &self.initial_phase);
        for phase_id in &phase_ids {
            if !reachable.contains(phase_id) {
                return Err(ScenarioError::MissionGraph {
                    reason: format!("phase `{phase_id}` is unreachable from the initial phase"),
                });
            }
        }

        Ok(())
    }
}

fn mission_cycle_involving(
    phase_ids: &[String],
    transitions: &[PhaseTransitionConfig],
) -> Option<Vec<String>> {
    let mut successors: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    let mut in_degree: BTreeMap<&str, usize> = BTreeMap::new();
    for phase_id in phase_ids {
        successors.entry(phase_id.as_str()).or_default();
        in_degree.entry(phase_id.as_str()).or_insert(0);
    }
    for transition in transitions {
        successors
            .entry(transition.from.as_str())
            .or_default()
            .push(transition.to.as_str());
        *in_degree.entry(transition.to.as_str()).or_insert(0) += 1;
    }

    let mut frontier = BTreeSet::new();
    for (phase_id, degree) in &in_degree {
        if *degree == 0 {
            frontier.insert(*phase_id);
        }
    }

    let mut visited = BTreeSet::new();
    while let Some(phase_id) = frontier.iter().next().copied() {
        frontier.remove(phase_id);
        visited.insert(phase_id);
        if let Some(next) = successors.get(phase_id) {
            for successor in next {
                let degree = in_degree.entry(successor).or_insert(0);
                *degree = degree.saturating_sub(1);
                if *degree == 0 {
                    frontier.insert(*successor);
                }
            }
        }
    }

    if visited.len() == phase_ids.len() {
        None
    } else {
        Some(
            phase_ids
                .iter()
                .filter(|phase_id| !visited.contains(phase_id.as_str()))
                .cloned()
                .collect(),
        )
    }
}

fn mission_reachable(
    phase_ids: &[String],
    transitions: &[PhaseTransitionConfig],
    start: &str,
) -> BTreeSet<String> {
    let phase_set: BTreeSet<&str> = phase_ids.iter().map(String::as_str).collect();
    let mut successors: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for phase_id in phase_ids {
        successors.entry(phase_id.as_str()).or_default();
    }
    for transition in transitions {
        successors
            .entry(transition.from.as_str())
            .or_default()
            .push(transition.to.as_str());
    }

    let mut reachable = BTreeSet::new();
    let mut queue = VecDeque::new();
    if phase_set.contains(start) {
        reachable.insert(start.to_owned());
        queue.push_back(start);
    }
    while let Some(phase_id) = queue.pop_front() {
        if let Some(next) = successors.get(phase_id) {
            for successor in next {
                if reachable.insert((*successor).to_owned()) {
                    queue.push_back(*successor);
                }
            }
        }
    }
    reachable
}

/// One declared mission phase.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct PhaseConfig {
    /// Stable phase id (`snake_case` scenario-text identifier).
    pub id: String,
    /// Human-readable label.
    pub label: String,
    /// Optional list of effector ids permitted while this phase is
    /// active. Phase 3.4 validates references; active-phase command
    /// gating is deferred.
    #[serde(default)]
    pub allowed_effectors: Vec<String>,
    /// Optional list of engine ids permitted while this phase is
    /// active. Phase-3.6 will enforce; Phase-3.2 leaves it
    /// informational.
    #[serde(default)]
    pub allowed_engines: Vec<String>,
}

impl PhaseConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        require_non_empty(&format!("mission.phases[{index}].id"), &self.id)?;
        require_non_empty(&format!("mission.phases[{index}].label"), &self.label)?;
        Ok(())
    }
}

/// One declared mission event (binding of trigger → action).
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EventConfig {
    /// Stable event id (`snake_case` scenario-text identifier).
    pub id: String,
    /// Trigger predicate.
    pub trigger: EventTriggerConfig,
    /// Action taken when the trigger fires.
    pub action: EventActionConfig,
    /// Whether the event fires at most once per simulation run.
    /// Defaults to `true` — most events have one-shot semantics.
    #[serde(default = "default_once")]
    pub once: bool,
}

const fn default_once() -> bool {
    true
}

impl EventConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        require_non_empty(&format!("mission.events[{index}].id"), &self.id)?;
        self.trigger.validate(index)?;
        self.action.validate(index)?;
        Ok(())
    }
}

/// Trigger predicate. Tagged enum dispatched on the `kind` string.
///
/// `kind = "scripted"` is rejected at parse time with a typed
/// deferral error pointing at Phase 3.4.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EventTriggerConfig {
    /// Time-bound crossing.
    AtTime {
        /// Trigger threshold in seconds.
        time_s: f64,
    },
    /// Altitude crossing on the way up.
    AtAltitudeAscending {
        /// Altitude threshold (m).
        altitude_m: f64,
    },
    /// Altitude crossing on the way down.
    AtAltitudeDescending {
        /// Altitude threshold (m).
        altitude_m: f64,
    },
    /// Velocity sign-flip apogee detector.
    AtApogee,
    /// Mass-fraction crossing (current/initial mass below threshold).
    AtMassFraction {
        /// Threshold mass fraction in `[0, 1]`.
        remaining: f64,
    },
    /// Phase-3.4 deferred: rejected at parse time until atmosphere is
    /// wired into event evaluation.
    AtDynamicPressure {
        /// Threshold dynamic pressure (Pa).
        pressure_pa: f64,
        /// `false`: rising-edge crossing. `true`: falling-edge.
        falling: bool,
    },
    /// Phase-3.4 deferred: rejected at parse time.
    Scripted,
}

impl EventTriggerConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("mission.events[{index}].trigger.{field}");
        match self {
            Self::AtTime { time_s } => {
                require_finite(&path("time_s"), *time_s)?;
            }
            Self::AtAltitudeAscending { altitude_m }
            | Self::AtAltitudeDescending { altitude_m } => {
                require_finite(&path("altitude_m"), *altitude_m)?;
            }
            Self::AtApogee => {}
            Self::AtMassFraction { remaining } => {
                require_finite(&path("remaining"), *remaining)?;
                require_in_range(&path("remaining"), *remaining, 0.0, 1.0)?;
            }
            Self::AtDynamicPressure { .. } => {
                return Err(ScenarioError::UnsupportedTriggerKind {
                    kind: "at_dynamic_pressure".to_owned(),
                    reason: "dynamic-pressure triggers ship in Phase 3.4 when atmosphere is wired into event evaluation".to_owned(),
                });
            }
            Self::Scripted => {
                return Err(ScenarioError::UnsupportedTriggerKind {
                    kind: "scripted".to_owned(),
                    reason: "scripted triggers are deferred to a later Phase-3 sub-phase; use effector command_schedule for deterministic actuator scripts".to_owned(),
                });
            }
        }
        Ok(())
    }
}

/// Action taken when an event fires. Tagged enum dispatched on the
/// `kind` string.
///
/// `engine_command`, `effector_override`, `separation`, and
/// `deploy_recovery` are rejected at parse time with typed deferral
/// errors pointing at the future phase that will land them.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EventActionConfig {
    /// Transition the active mission phase.
    EnterPhase {
        /// Destination phase id.
        phase: String,
    },
    /// Emit a `bool` telemetry marker.
    EmitTelemetryMarker {
        /// Marker channel tag.
        tag: String,
    },
    /// Halt the kernel with an `openbmp_sim::StopReason::MissionEnded`.
    Stop {
        /// Human-readable label for the stop reason.
        label: String,
    },
    /// Phase-3.6: per-engine command targeting a declared
    /// `[[vehicle.assembly.engines]]` by id. The kernel records the
    /// firing; the runner-side `EngineRack` drains and applies the
    /// command on the next rack tick.
    EngineCommand {
        /// Target engine id (must reference a declared engine).
        id: String,
        /// Command payload.
        command: EngineCommandConfig,
    },
    /// Phase-3.4: scenario-driven effector command override. Targets
    /// a declared `[[vehicle.assembly.effectors]]` by id and commits
    /// `command` at fire time (single-shot).
    EffectorOverride {
        /// Target effector id (must reference a declared effector).
        id: String,
        /// Command value (post-fault, pre-clamp).
        command: f64,
    },
    /// Phase-3.6 / 3.7 deferred.
    Separation,
    /// Phase-3.9 deferred.
    DeployRecovery,
}

impl EventActionConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("mission.events[{index}].action.{field}");
        match self {
            Self::EnterPhase { phase } => {
                require_non_empty(&path("phase"), phase)?;
            }
            Self::EmitTelemetryMarker { tag } => {
                require_non_empty(&path("tag"), tag)?;
            }
            Self::Stop { label } => {
                require_non_empty(&path("label"), label)?;
            }
            Self::EngineCommand { id, command } => {
                require_non_empty(&path("id"), id)?;
                command.validate(&path("command"))?;
            }
            Self::EffectorOverride { id, command } => {
                require_non_empty(&path("id"), id)?;
                require_finite(&path("command"), *command)?;
            }
            Self::Separation => {
                return Err(ScenarioError::UnsupportedActionKind {
                    kind: "separation".to_owned(),
                    deferred_to: "Phase 3.6 / 3.7".to_owned(),
                });
            }
            Self::DeployRecovery => {
                return Err(ScenarioError::UnsupportedActionKind {
                    kind: "deploy_recovery".to_owned(),
                    deferred_to: "Phase 3.9".to_owned(),
                });
            }
        }
        Ok(())
    }
}

/// One declared mission transition: `from --event--> to`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct PhaseTransitionConfig {
    /// Source phase id.
    pub from: String,
    /// Destination phase id.
    pub to: String,
    /// Event id whose firing triggers this transition.
    pub event: String,
}

impl PhaseTransitionConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        require_non_empty(&format!("mission.transitions[{index}].from"), &self.from)?;
        require_non_empty(&format!("mission.transitions[{index}].to"), &self.to)?;
        require_non_empty(&format!("mission.transitions[{index}].event"), &self.event)?;
        Ok(())
    }
}

// ---------------------------------------------------------------------
// Vehicle assembly block (Phase 3.3)
// ---------------------------------------------------------------------

/// Top-level `[vehicle.assembly]` block.
///
/// Declares the vehicle as a tree of bodies. The Phase-3.3 sub-trees
/// for `effectors` / `engines` / `tanks` are reserved-but-rejected:
/// declarations parse via serde but the validator surfaces a typed
/// `UnsupportedAssemblyChild` error pointing at the future phase
/// that will land them.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AssemblyConfig {
    /// Optional assembly id. Defaults to FNV(scenario.meta.name)
    /// when absent (resolved at runtime).
    #[serde(default)]
    pub id: Option<String>,
    /// Bodies in scenario-declared order.
    #[serde(default)]
    pub bodies: Vec<AssemblyBodyConfig>,
    /// Phase-3.4: control effectors in scenario-declared order. The
    /// runner builds `Box<dyn ControlEffector>` instances from these
    /// configs and adds them to the assembly's effector vec.
    #[serde(default)]
    pub effectors: Vec<EffectorConfig>,
    /// Phase-3.6: liquid engines in scenario-declared order. The
    /// runner builds `Box<dyn EngineModel>` instances from these
    /// configs and assembles them into a runner-side `EngineRack`.
    /// A scenario that also declares `[propulsion.motor]` is
    /// rejected with `ScenarioError::AmbiguousPropulsion`.
    #[serde(default)]
    pub engines: Vec<EngineConfig>,
    /// Phase-3.6: optional cluster-layout tag for telemetry / docs.
    /// Defaults to `Custom` when omitted.
    #[serde(default)]
    pub cluster_layout: Option<ClusterLayoutConfig>,
    /// Phase-3.7: tanks in scenario-declared order. The runner
    /// builds `Box<dyn MovingMassModel>` instances from these
    /// configs and assembles them into a runner-side `TankRack`.
    /// Tanks contribute their mass, CG offset, inertia delta, and
    /// reaction force / moment to the vehicle dynamics. Phase-3.7
    /// ships drain decoupled from engines (`drain_rate_kg_per_s`
    /// is scenario-declared); engine-cluster drain coupling is a
    /// Phase-3.X follow-on.
    #[serde(default)]
    pub tanks: Vec<TankConfig>,
}

impl AssemblyConfig {
    /// Validate the assembly block. `flat_mass_kg` is the
    /// `[vehicle].mass_kg` field used for cross-consistency
    /// checking; the validator requires the sum of body dry masses
    /// to equal `flat_mass_kg` within the assembly consistency
    /// tolerance.
    pub(crate) fn validate(
        &self,
        vehicle_kind: &str,
        flat_mass_kg: f64,
        flat_inertia_body_kg_m2: Option<&[[f64; 3]; 3]>,
        dt_s: f64,
    ) -> Result<(), ScenarioError> {
        if self.bodies.is_empty() {
            return Err(ScenarioError::EmptyList {
                field: "vehicle.assembly.bodies".to_owned(),
            });
        }
        let rigid_body = vehicle_kind == "rigid_body";
        let mut seen_ids: std::collections::BTreeSet<&str> = std::collections::BTreeSet::new();
        for (index, body) in self.bodies.iter().enumerate() {
            body.validate(index, rigid_body)?;
            if !seen_ids.insert(body.id.as_str()) {
                return Err(ScenarioError::DuplicateValue {
                    field: format!("vehicle.assembly.bodies[{index}].id"),
                    value: body.id.clone(),
                });
            }
        }
        let body_mass_sum: f64 = self.bodies.iter().map(|b| b.dry_mass_kg).sum();
        if !assembly_values_consistent(body_mass_sum, flat_mass_kg) {
            return Err(ScenarioError::InconsistentSection {
                field_a: "vehicle.mass_kg".to_owned(),
                value_a: format!("{flat_mass_kg}"),
                field_b: "vehicle.assembly.bodies[*].dry_mass_kg sum".to_owned(),
                value_b: format!("{body_mass_sum}"),
            });
        }
        if rigid_body {
            let flat_inertia =
                flat_inertia_body_kg_m2.ok_or_else(|| ScenarioError::MissingRequiredField {
                    field: "vehicle.inertia_tensor_body_kg_m2".to_owned(),
                    role: ModelRole::Vehicle,
                    name: "rigid_body".to_owned(),
                })?;
            let assembly_inertia = summed_assembly_inertia_body_kg_m2(&self.bodies);
            for i in 0..3 {
                for j in 0..3 {
                    let assembly_value = assembly_inertia[i][j];
                    let flat_value = flat_inertia[i][j];
                    if !assembly_values_consistent(assembly_value, flat_value) {
                        return Err(ScenarioError::InconsistentSection {
                            field_a: format!("vehicle.inertia_tensor_body_kg_m2[{i}][{j}]"),
                            value_a: format!("{flat_value}"),
                            field_b: format!(
                                "vehicle.assembly summed dry_inertia_body_kg_m2[{i}][{j}]"
                            ),
                            value_b: format!("{assembly_value}"),
                        });
                    }
                }
            }
        }
        let mut seen_effector_ids: std::collections::BTreeSet<&str> =
            std::collections::BTreeSet::new();
        for (index, effector) in self.effectors.iter().enumerate() {
            effector.validate(index, dt_s)?;
            if !seen_effector_ids.insert(effector.id.as_str()) {
                return Err(ScenarioError::DuplicateValue {
                    field: format!("vehicle.assembly.effectors[{index}].id"),
                    value: effector.id.clone(),
                });
            }
        }
        let mut seen_engine_ids: std::collections::BTreeSet<&str> =
            std::collections::BTreeSet::new();
        for (index, engine) in self.engines.iter().enumerate() {
            engine.validate(index)?;
            if !seen_engine_ids.insert(engine.id.as_str()) {
                return Err(ScenarioError::DuplicateValue {
                    field: format!("vehicle.assembly.engines[{index}].id"),
                    value: engine.id.clone(),
                });
            }
        }
        let mut seen_tank_ids: std::collections::BTreeSet<&str> = std::collections::BTreeSet::new();
        let body_ids: std::collections::BTreeSet<&str> =
            self.bodies.iter().map(|b| b.id.as_str()).collect();
        for (index, tank) in self.tanks.iter().enumerate() {
            tank.validate(index, vehicle_kind, &body_ids, dt_s)?;
            if !seen_tank_ids.insert(tank.id.as_str()) {
                return Err(ScenarioError::DuplicateValue {
                    field: format!("vehicle.assembly.tanks[{index}].id"),
                    value: tank.id.clone(),
                });
            }
        }
        Ok(())
    }
}

const ASSEMBLY_CONSISTENCY_ABS_TOL: f64 = 1.0e-12;
const ASSEMBLY_CONSISTENCY_REL_TOL: f64 = 1.0e-9;

fn assembly_values_consistent(a: f64, b: f64) -> bool {
    let scale = a.abs().max(b.abs());
    let tolerance = ASSEMBLY_CONSISTENCY_ABS_TOL.max(ASSEMBLY_CONSISTENCY_REL_TOL * scale);
    (a - b).abs() <= tolerance
}

fn summed_assembly_inertia_body_kg_m2(bodies: &[AssemblyBodyConfig]) -> [[f64; 3]; 3] {
    let total_mass: f64 = bodies.iter().map(|body| body.dry_mass_kg).sum();
    let mut weighted_cg = [0.0_f64; 3];
    for body in bodies {
        for (axis, component) in weighted_cg.iter_mut().enumerate() {
            *component += body.dry_cg_body_m[axis] * body.dry_mass_kg;
        }
    }
    let assembly_cg = [
        weighted_cg[0] / total_mass,
        weighted_cg[1] / total_mass,
        weighted_cg[2] / total_mass,
    ];

    let mut assembly_inertia = [[0.0_f64; 3]; 3];
    for body in bodies {
        let Some(inertia) = body.dry_inertia_body_kg_m2 else {
            continue;
        };
        let r = [
            body.dry_cg_body_m[0] - assembly_cg[0],
            body.dry_cg_body_m[1] - assembly_cg[1],
            body.dry_cg_body_m[2] - assembly_cg[2],
        ];
        let r_dot_r = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        for i in 0..3 {
            for j in 0..3 {
                let identity_component = if i == j { r_dot_r } else { 0.0 };
                let parallel_axis = identity_component - r[i] * r[j];
                assembly_inertia[i][j] += inertia[i][j] + body.dry_mass_kg * parallel_axis;
            }
        }
    }
    assembly_inertia
}

/// One declared body within `[vehicle.assembly]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AssemblyBodyConfig {
    /// Stable body id (`snake_case` scenario-text identifier).
    pub id: String,
    /// Geometry descriptor.
    pub geometry: BodyGeometryConfig,
    /// Dry mass (kg). Strictly positive, finite.
    pub dry_mass_kg: f64,
    /// Body-frame center of mass. Defaults to origin.
    #[serde(default = "zero_vec3_meters")]
    pub dry_cg_body_m: [f64; 3],
    /// Body-frame inertia tensor (kg·m²). Required when the
    /// scenario uses `kind = "rigid_body"`; ignored for point-mass.
    #[serde(default)]
    pub dry_inertia_body_kg_m2: Option<[[f64; 3]; 3]>,
}

const fn zero_vec3_meters() -> [f64; 3] {
    [0.0, 0.0, 0.0]
}

impl AssemblyBodyConfig {
    fn validate(&self, index: usize, rigid_body: bool) -> Result<(), ScenarioError> {
        require_non_empty(&format!("vehicle.assembly.bodies[{index}].id"), &self.id)?;
        require_finite(
            &format!("vehicle.assembly.bodies[{index}].dry_mass_kg"),
            self.dry_mass_kg,
        )?;
        require_positive(
            &format!("vehicle.assembly.bodies[{index}].dry_mass_kg"),
            self.dry_mass_kg,
        )?;
        require_finite_array(
            &format!("vehicle.assembly.bodies[{index}].dry_cg_body_m"),
            &self.dry_cg_body_m,
        )?;
        self.geometry.validate(index)?;
        if rigid_body && self.dry_inertia_body_kg_m2.is_none() {
            return Err(ScenarioError::MissingRequiredField {
                field: format!("vehicle.assembly.bodies[{index}].dry_inertia_body_kg_m2"),
                role: ModelRole::Vehicle,
                name: "rigid_body".to_owned(),
            });
        }
        if let Some(inertia) = &self.dry_inertia_body_kg_m2 {
            validate_inertia_tensor(inertia)?;
        }
        Ok(())
    }
}

/// Body geometry descriptor — tagged enum dispatched on `kind`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum BodyGeometryConfig {
    /// Right circular cylinder.
    Cylinder {
        /// Length (m).
        length_m: f64,
        /// Diameter (m).
        diameter_m: f64,
    },
    /// Right circular cone.
    Cone {
        /// Length / height (m).
        length_m: f64,
        /// Base diameter (m).
        base_diameter_m: f64,
    },
    /// Pre-computed reference geometry.
    Reference {
        /// Reference length (m).
        length_m: f64,
        /// Reference area (m²).
        area_m2: f64,
    },
}

impl BodyGeometryConfig {
    fn validate(&self, body_index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.bodies[{body_index}].geometry.{field}");
        let (a_name, a, b_name, b) = match self {
            Self::Cylinder {
                length_m,
                diameter_m,
            } => ("length_m", *length_m, "diameter_m", *diameter_m),
            Self::Cone {
                length_m,
                base_diameter_m,
            } => ("length_m", *length_m, "base_diameter_m", *base_diameter_m),
            Self::Reference { length_m, area_m2 } => ("length_m", *length_m, "area_m2", *area_m2),
        };
        require_finite(&path(a_name), a)?;
        require_positive(&path(a_name), a)?;
        require_finite(&path(b_name), b)?;
        require_positive(&path(b_name), b)?;
        Ok(())
    }
}

// ---------------------------------------------------------------------
// Effector block (Phase 3.4)
// ---------------------------------------------------------------------

/// One declared control effector within `[vehicle.assembly]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EffectorConfig {
    /// Stable effector id (`snake_case` scenario-text identifier).
    pub id: String,
    /// Effector kind + per-kind parameters (tagged on `kind`).
    pub kind: EffectorKindConfig,
    /// Position / rate / latency limits.
    pub limits: EffectorLimitsConfig,
    /// Optional initial position (defaults to 0.0). Must lie within
    /// `[limits.min, limits.max]`.
    #[serde(default)]
    pub initial_position: Option<f64>,
    /// Optional telemetry unit label for this scalar effector axis.
    /// Defaults to `"1"` when omitted.
    #[serde(default)]
    pub unit: Option<String>,
    /// Optional fault mounted at scenario load time.
    #[serde(default)]
    pub fault: Option<EffectorFaultConfig>,
    /// Optional deterministic command schedule. Resolves the
    /// per-step command at runtime; superseded by an
    /// `EventAction::EffectorOverride` on the next rack tick.
    #[serde(default)]
    pub command_schedule: Option<EffectorCommandScheduleConfig>,
}

impl EffectorConfig {
    fn validate(&self, index: usize, dt_s: f64) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.effectors[{index}].{field}");
        require_non_empty(&path("id"), &self.id)?;
        self.kind.validate(index)?;
        self.limits.validate(index, dt_s)?;
        if let Some(unit) = &self.unit {
            require_non_empty(&path("unit"), unit)?;
        }
        if let Some(initial) = self.initial_position {
            require_finite(&path("initial_position"), initial)?;
            if initial < self.limits.min || initial > self.limits.max {
                return Err(ScenarioError::InvalidNumber {
                    field: path("initial_position"),
                    value: initial,
                    rule: "must lie within [limits.min, limits.max]",
                });
            }
        }
        if let Some(fault) = &self.fault {
            fault.validate(index, &self.limits)?;
        }
        if let Some(schedule) = &self.command_schedule {
            schedule.validate(index)?;
        }
        Ok(())
    }
}

/// Effector kind tagged enum. Phase 3.4 ships `linear_actuator` only.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EffectorKindConfig {
    /// First-order linear actuator with optional time constant.
    LinearActuator {
        /// Optional first-order lag time constant (s). Defaults to 0
        /// (pure rate-clamped tracker).
        #[serde(default)]
        tau_s: Option<f64>,
    },
}

impl EffectorKindConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.effectors[{index}].kind.{field}");
        match self {
            Self::LinearActuator { tau_s } => {
                if let Some(tau_s) = tau_s {
                    require_finite(&path("tau_s"), *tau_s)?;
                    if *tau_s < 0.0 {
                        return Err(ScenarioError::InvalidNumber {
                            field: path("tau_s"),
                            value: *tau_s,
                            rule: "must be non-negative",
                        });
                    }
                }
            }
        }
        Ok(())
    }
}

/// Position / rate / latency limits.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EffectorLimitsConfig {
    /// Minimum permitted deflection.
    pub min: f64,
    /// Maximum permitted deflection.
    pub max: f64,
    /// Maximum slew rate magnitude (units / s).
    pub max_rate_per_s: f64,
    /// Deadband.
    pub deadband: f64,
    /// Pure-delay latency (s).
    pub latency_s: f64,
}

impl EffectorLimitsConfig {
    fn validate(&self, index: usize, dt_s: f64) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.effectors[{index}].limits.{field}");
        require_finite(&path("min"), self.min)?;
        require_finite(&path("max"), self.max)?;
        if self.min >= self.max {
            return Err(ScenarioError::InvalidNumber {
                field: path("min"),
                value: self.min,
                rule: "must be strictly less than max",
            });
        }
        require_finite(&path("max_rate_per_s"), self.max_rate_per_s)?;
        require_positive(&path("max_rate_per_s"), self.max_rate_per_s)?;
        require_finite(&path("deadband"), self.deadband)?;
        if self.deadband < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("deadband"),
                value: self.deadband,
                rule: "must be non-negative",
            });
        }
        if self.deadband > (self.max - self.min) {
            return Err(ScenarioError::InvalidNumber {
                field: path("deadband"),
                value: self.deadband,
                rule: "must be at most (max - min)",
            });
        }
        require_finite(&path("latency_s"), self.latency_s)?;
        if self.latency_s < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("latency_s"),
                value: self.latency_s,
                rule: "must be non-negative",
            });
        }
        if self.latency_s > 0.0 && self.latency_s + 1.0e-12 < dt_s {
            return Err(ScenarioError::InvalidNumber {
                field: path("latency_s"),
                value: self.latency_s,
                rule: "must be either zero or at least time.dt_s",
            });
        }
        if self.latency_s > 0.0 {
            let ratio = self.latency_s / dt_s;
            let rounded = ratio.round();
            if (ratio - rounded).abs() > 1.0e-9 * ratio {
                return Err(ScenarioError::InvalidNumber {
                    field: path("latency_s"),
                    value: self.latency_s,
                    rule: "must be an integer multiple of time.dt_s",
                });
            }
        }
        Ok(())
    }
}

/// Effector fault tagged enum.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EffectorFaultConfig {
    /// Effector locked at `at`. `at` must lie within `[min, max]`.
    Jam {
        /// Locked deflection.
        at: f64,
    },
    /// Effector slews at `rate_per_s` ignoring command.
    Runaway {
        /// Signed slew rate (units / s).
        rate_per_s: f64,
    },
    /// Effector tracks the command but at a reduced max rate.
    /// `factor ∈ [0, 1]`.
    ReducedRate {
        /// Multiplier on the max rate.
        factor: f64,
    },
    /// Effector steps to `to` then jams.
    Hardover {
        /// Target deflection.
        to: f64,
    },
}

impl EffectorFaultConfig {
    fn validate(&self, index: usize, limits: &EffectorLimitsConfig) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.effectors[{index}].fault.{field}");
        match self {
            Self::Jam { at } => {
                require_finite(&path("at"), *at)?;
                if *at < limits.min || *at > limits.max {
                    return Err(ScenarioError::InvalidNumber {
                        field: path("at"),
                        value: *at,
                        rule: "must lie within [limits.min, limits.max]",
                    });
                }
            }
            Self::Runaway { rate_per_s } => {
                require_finite(&path("rate_per_s"), *rate_per_s)?;
            }
            Self::ReducedRate { factor } => {
                require_finite(&path("factor"), *factor)?;
                require_in_range(&path("factor"), *factor, 0.0, 1.0)?;
            }
            Self::Hardover { to } => {
                require_finite(&path("to"), *to)?;
                if *to < limits.min || *to > limits.max {
                    return Err(ScenarioError::InvalidNumber {
                        field: path("to"),
                        value: *to,
                        rule: "must lie within [limits.min, limits.max]",
                    });
                }
            }
        }
        Ok(())
    }
}

/// Optional deterministic command schedule. Resolves the per-step
/// command at runtime when no override fires.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EffectorCommandScheduleConfig {
    /// Constant command for the whole run.
    Constant {
        /// Command value.
        value: f64,
    },
    /// Step at `time_s`: emit `before` until `time_s`, then `after`.
    StepAt {
        /// Step time (s).
        time_s: f64,
        /// Pre-step command.
        before: f64,
        /// Post-step command.
        after: f64,
    },
    /// Linear ramp from `start` at `start_time_s` to `end` at
    /// `end_time_s`. Holds at endpoints outside the interval.
    LinearRamp {
        /// Ramp start time (s).
        start_time_s: f64,
        /// Ramp end time (s).
        end_time_s: f64,
        /// Command at `start_time_s`.
        start: f64,
        /// Command at `end_time_s`.
        end: f64,
    },
}

impl EffectorCommandScheduleConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path =
            |field: &str| format!("vehicle.assembly.effectors[{index}].command_schedule.{field}");
        match self {
            Self::Constant { value } => require_finite(&path("value"), *value)?,
            Self::StepAt {
                time_s,
                before,
                after,
            } => {
                require_finite(&path("time_s"), *time_s)?;
                require_finite(&path("before"), *before)?;
                require_finite(&path("after"), *after)?;
            }
            Self::LinearRamp {
                start_time_s,
                end_time_s,
                start,
                end,
            } => {
                require_finite(&path("start_time_s"), *start_time_s)?;
                require_finite(&path("end_time_s"), *end_time_s)?;
                require_finite(&path("start"), *start)?;
                require_finite(&path("end"), *end)?;
                if *start_time_s >= *end_time_s {
                    return Err(ScenarioError::InvalidNumber {
                        field: path("end_time_s"),
                        value: *end_time_s,
                        rule: "must be strictly greater than start_time_s",
                    });
                }
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------
// EngineConfig (Phase 3.6)
// ---------------------------------------------------------------------

/// Cluster-layout tag for telemetry / docs. Phase-3.6 has no
/// behavioural use; later sub-phases may key symmetry-aware fault
/// scenarios off the layout.
#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum ClusterLayoutConfig {
    /// Single axial engine.
    Axial,
    /// Ring of N engines around the body `+z` axis.
    Ring,
    /// Octaweb: 1 axial + N ring-mounted.
    Octaweb,
    /// Custom geometry (default).
    #[default]
    Custom,
}

/// One declared liquid-engine entry under
/// `[[vehicle.assembly.engines]]`. The runner builds a
/// `Box<dyn EngineModel>` from this config and adds it to the
/// per-step `EngineRack`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EngineConfig {
    /// Stable engine id (`snake_case` scenario-text identifier).
    pub id: String,
    /// Engine kind + per-kind parameters (tagged on `kind`).
    pub kind: EngineKindConfig,
    /// Body-frame mount point (m). `[x, y, z]`.
    pub mount_point_body_m: [f64; 3],
    /// Position / rate / gimbal limits.
    pub limits: EngineLimitsConfig,
    /// Optional fault mounted at scenario load time.
    #[serde(default)]
    pub fault: Option<EngineFaultConfig>,
}

impl EngineConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.engines[{index}].{field}");
        require_non_empty(&path("id"), &self.id)?;
        self.kind.validate(index)?;
        self.limits.validate(index)?;
        require_finite_array(&path("mount_point_body_m"), &self.mount_point_body_m)?;
        if let Some(fault) = &self.fault {
            fault.validate(index, &self.limits)?;
        }
        Ok(())
    }
}

/// Engine kind tagged enum. Phase 3.6 ships `liquid_engine` only.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EngineKindConfig {
    /// Phase-3.6 reference impl: linear ignition + constant burn +
    /// linear shutdown transients.
    LiquidEngine,
}

impl EngineKindConfig {
    // The `Result` return is forward-compat: future engine kinds
    // (hybrid, cold-gas) will carry payload that needs validation.
    // Phase 3.6 ships only `LiquidEngine` with no payload, so this
    // arm always returns `Ok(())`.
    #[allow(clippy::unnecessary_wraps)]
    fn validate(&self, _index: usize) -> Result<(), ScenarioError> {
        match self {
            Self::LiquidEngine => Ok(()),
        }
    }
}

/// Engine limits.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EngineLimitsConfig {
    /// Maximum thrust at full throttle (N).
    pub max_thrust_n: f64,
    /// Specific impulse (s).
    pub isp_s: f64,
    /// Linear ignition transient duration (s). `0.0` →
    /// instantaneous ignition.
    pub ignition_transient_s: f64,
    /// Linear shutdown transient duration (s). `0.0` →
    /// instantaneous shutdown.
    pub shutdown_transient_s: f64,
    /// Maximum absolute gimbal angle on either axis (rad). `0.0` →
    /// fixed-axis engine (no gimbal).
    pub max_gimbal_rad: f64,
}

impl EngineLimitsConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.engines[{index}].limits.{field}");
        require_finite(&path("max_thrust_n"), self.max_thrust_n)?;
        require_positive(&path("max_thrust_n"), self.max_thrust_n)?;
        require_finite(&path("isp_s"), self.isp_s)?;
        require_positive(&path("isp_s"), self.isp_s)?;
        require_finite(&path("ignition_transient_s"), self.ignition_transient_s)?;
        if self.ignition_transient_s < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("ignition_transient_s"),
                value: self.ignition_transient_s,
                rule: "must be non-negative",
            });
        }
        require_finite(&path("shutdown_transient_s"), self.shutdown_transient_s)?;
        if self.shutdown_transient_s < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("shutdown_transient_s"),
                value: self.shutdown_transient_s,
                rule: "must be non-negative",
            });
        }
        require_finite(&path("max_gimbal_rad"), self.max_gimbal_rad)?;
        if self.max_gimbal_rad < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("max_gimbal_rad"),
                value: self.max_gimbal_rad,
                rule: "must be non-negative",
            });
        }
        Ok(())
    }
}

/// Engine fault tagged enum. Phase-3.6 ships four canonical modes;
/// load-time injection only.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EngineFaultConfig {
    /// Throttle stuck at `at_throttle ∈ [0, 1]`.
    Stuck {
        /// Stuck throttle value.
        at_throttle: f64,
    },
    /// Engine off and never restarts.
    HardOff,
    /// Thrust scaled by `factor` (≥ 0, finite).
    OverThrust {
        /// Thrust multiplier.
        factor: f64,
    },
    /// Gimbal frozen at the given angles (rad). Both must lie within
    /// `±max_gimbal_rad`.
    GimbalLocked {
        /// Locked pitch angle.
        pitch_rad: f64,
        /// Locked yaw angle.
        yaw_rad: f64,
    },
}

impl EngineFaultConfig {
    fn validate(&self, index: usize, limits: &EngineLimitsConfig) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.engines[{index}].fault.{field}");
        match *self {
            Self::Stuck { at_throttle } => {
                require_finite(&path("at_throttle"), at_throttle)?;
                if !(0.0..=1.0).contains(&at_throttle) {
                    return Err(ScenarioError::InvalidNumber {
                        field: path("at_throttle"),
                        value: at_throttle,
                        rule: "must lie in [0, 1]",
                    });
                }
            }
            Self::HardOff => {}
            Self::OverThrust { factor } => {
                require_finite(&path("factor"), factor)?;
                if factor < 0.0 {
                    return Err(ScenarioError::InvalidNumber {
                        field: path("factor"),
                        value: factor,
                        rule: "must be non-negative",
                    });
                }
            }
            Self::GimbalLocked { pitch_rad, yaw_rad } => {
                require_finite(&path("pitch_rad"), pitch_rad)?;
                require_finite(&path("yaw_rad"), yaw_rad)?;
                if pitch_rad.abs() > limits.max_gimbal_rad || yaw_rad.abs() > limits.max_gimbal_rad
                {
                    return Err(ScenarioError::InvalidNumber {
                        field: path("pitch_rad"),
                        value: pitch_rad,
                        rule: "must lie within [-max_gimbal_rad, max_gimbal_rad]",
                    });
                }
            }
        }
        Ok(())
    }
}

/// Per-engine command payload. Mirrors the `EngineCommand` struct in
/// `openbmp-propulsion`; the runner translates `EngineCommandConfig
/// → EngineCommand` at mission-event resolution time.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EngineCommandConfig {
    /// Throttle setting in `[0, 1]`. Values outside are rejected at
    /// parse time (the runtime engine clamps but the scenario format
    /// is strict).
    pub throttle_unit: f64,
    /// Gimbal pitch angle (rad). Finite; runtime clamping is per
    /// engine `max_gimbal_rad`.
    pub gimbal_pitch_rad: f64,
    /// Gimbal yaw angle (rad). Finite; runtime clamping is per
    /// engine `max_gimbal_rad`.
    pub gimbal_yaw_rad: f64,
    /// Ignition request. Honoured only from `Idle`.
    pub ignite: bool,
    /// Shutdown request. Honoured only from `Igniting` / `Burning`.
    pub shutdown: bool,
}

impl EngineCommandConfig {
    fn validate(&self, prefix: &str) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("{prefix}.{field}");
        require_finite(&path("throttle_unit"), self.throttle_unit)?;
        if !(0.0..=1.0).contains(&self.throttle_unit) {
            return Err(ScenarioError::InvalidNumber {
                field: path("throttle_unit"),
                value: self.throttle_unit,
                rule: "must lie in [0, 1]",
            });
        }
        require_finite(&path("gimbal_pitch_rad"), self.gimbal_pitch_rad)?;
        require_finite(&path("gimbal_yaw_rad"), self.gimbal_yaw_rad)?;
        Ok(())
    }
}

// ---------------------------------------------------------------------
// Tank block (Phase 3.7)
// ---------------------------------------------------------------------

/// One declared tank entry under `[[vehicle.assembly.tanks]]`.
///
/// The runner builds a `Box<dyn MovingMassModel>` from this config
/// and assembles it into a runner-side `TankRack` mirroring the
/// Phase-3.6 `EngineRack` pattern.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TankConfig {
    /// Stable tank id (`snake_case` scenario-text identifier).
    pub id: String,
    /// `id` of the parent body the tank is rigidly mounted to.
    /// Must match one of `vehicle.assembly.bodies[*].id`.
    pub mounted_to: String,
    /// Body-frame mount point (m). `[x, y, z]`.
    pub mount_point_body_m: [f64; 3],
    /// Closed-form geometry tagged on `kind`.
    pub geometry: TankGeometryConfig,
    /// Propellant constants. Toy / textbook only — fielded
    /// propellant data is rejected per `safety-boundaries.md`.
    pub propellant: PropellantSpecConfig,
    /// Initial fill fraction in `[0, 1]`.
    pub initial_fill_fraction: f64,
    /// Moving-mass kind + per-kind parameters tagged on `kind`.
    pub moving_mass: MovingMassKindConfig,
    /// Optional baffle model (consumed only by `BaffledPendulum`).
    #[serde(default)]
    pub baffle_model: Option<BaffleModelConfig>,
    /// Phase-3.7 decoupled drain. Constant `kg/s`; defaults to
    /// `0.0` when omitted. Future phase ties this to the engine
    /// cluster's per-step total mdot.
    #[serde(default)]
    pub drain_rate_kg_per_s: Option<f64>,
    /// Optional initial slosh perturbation (used for free-response
    /// scenarios and Phase-3.7.E exit-criterion testing).
    #[serde(default)]
    pub initial_slosh: Option<InitialSloshConfig>,
}

impl TankConfig {
    pub(crate) fn validate(
        &self,
        index: usize,
        vehicle_kind: &str,
        body_ids: &std::collections::BTreeSet<&str>,
        dt_s: f64,
    ) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.tanks[{index}].{field}");
        require_non_empty(&path("id"), &self.id)?;
        require_non_empty(&path("mounted_to"), &self.mounted_to)?;
        if !body_ids.contains(self.mounted_to.as_str()) {
            return Err(ScenarioError::UnknownBodyReference {
                field: path("mounted_to"),
                value: self.mounted_to.clone(),
            });
        }
        require_finite_array(&path("mount_point_body_m"), &self.mount_point_body_m)?;
        require_finite(&path("initial_fill_fraction"), self.initial_fill_fraction)?;
        if !(0.0..=1.0).contains(&self.initial_fill_fraction) {
            return Err(ScenarioError::InvalidNumber {
                field: path("initial_fill_fraction"),
                value: self.initial_fill_fraction,
                rule: "must lie in [0, 1]",
            });
        }
        self.geometry.validate(index)?;
        self.propellant.validate(index)?;
        self.moving_mass
            .validate(index, &self.geometry, vehicle_kind)?;
        if let Some(baffle) = &self.baffle_model {
            baffle.validate(index)?;
            if !matches!(
                self.moving_mass,
                MovingMassKindConfig::BaffledPendulum { .. }
            ) {
                return Err(ScenarioError::IncompatibleAssemblyEntry {
                    field: path("baffle_model"),
                    reason:
                        "baffle_model is only consumed by moving_mass.kind = \"baffled_pendulum\""
                            .to_owned(),
                });
            }
        }
        if let Some(rate) = self.drain_rate_kg_per_s {
            require_finite(&path("drain_rate_kg_per_s"), rate)?;
            if rate < 0.0 {
                return Err(ScenarioError::InvalidNumber {
                    field: path("drain_rate_kg_per_s"),
                    value: rate,
                    rule: "must be non-negative",
                });
            }
            let drained_per_step_kg = rate * dt_s;
            let initial_fluid_kg = self.geometry.volume_m3()
                * self.propellant.density_kg_m3
                * self.initial_fill_fraction;
            if drained_per_step_kg > initial_fluid_kg {
                return Err(ScenarioError::InvalidNumber {
                    field: path("drain_rate_kg_per_s"),
                    value: rate,
                    rule: "must not empty the tank in a single time.dt_s step",
                });
            }
        }
        if let Some(initial) = &self.initial_slosh {
            initial.validate(index, self.moving_mass)?;
        }
        Ok(())
    }
}

/// Closed-form tank geometry tagged enum.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum TankGeometryConfig {
    /// Right circular cylinder.
    Cylinder {
        /// Internal radius (m).
        radius_m: f64,
        /// Internal height (m).
        height_m: f64,
    },
    /// Sphere.
    Sphere {
        /// Internal radius (m).
        radius_m: f64,
    },
    /// Triaxial ellipsoid (a, b, c semi-axes, m).
    EllipsoidTextbook {
        /// Semi-axis along body x.
        a_m: f64,
        /// Semi-axis along body y.
        b_m: f64,
        /// Semi-axis along body z.
        c_m: f64,
    },
}

impl TankGeometryConfig {
    fn volume_m3(&self) -> f64 {
        match *self {
            Self::Cylinder { radius_m, height_m } => {
                std::f64::consts::PI * radius_m * radius_m * height_m
            }
            Self::Sphere { radius_m } => {
                (4.0 / 3.0) * std::f64::consts::PI * radius_m * radius_m * radius_m
            }
            Self::EllipsoidTextbook { a_m, b_m, c_m } => {
                (4.0 / 3.0) * std::f64::consts::PI * a_m * b_m * c_m
            }
        }
    }

    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.tanks[{index}].geometry.{field}");
        match *self {
            Self::Cylinder { radius_m, height_m } => {
                require_finite(&path("radius_m"), radius_m)?;
                require_positive(&path("radius_m"), radius_m)?;
                require_finite(&path("height_m"), height_m)?;
                require_positive(&path("height_m"), height_m)?;
            }
            Self::Sphere { radius_m } => {
                require_finite(&path("radius_m"), radius_m)?;
                require_positive(&path("radius_m"), radius_m)?;
            }
            Self::EllipsoidTextbook { a_m, b_m, c_m } => {
                require_finite(&path("a_m"), a_m)?;
                require_positive(&path("a_m"), a_m)?;
                require_finite(&path("b_m"), b_m)?;
                require_positive(&path("b_m"), b_m)?;
                require_finite(&path("c_m"), c_m)?;
                require_positive(&path("c_m"), c_m)?;
            }
        }
        Ok(())
    }
}

/// Propellant spec scenario-side. The runner converts to
/// `openbmp_vehicle::PropellantSpec` (which holds `label: &'static
/// str`) by leaking the `label` String once at scenario load.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct PropellantSpecConfig {
    /// Bulk liquid density at nominal storage conditions (kg/m³).
    pub density_kg_m3: f64,
    /// Display label (e.g. `"water_textbook"`).
    pub label: String,
}

impl PropellantSpecConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.tanks[{index}].propellant.{field}");
        require_finite(&path("density_kg_m3"), self.density_kg_m3)?;
        require_positive(&path("density_kg_m3"), self.density_kg_m3)?;
        require_non_empty(&path("label"), &self.label)?;
        Ok(())
    }
}

/// Moving-mass kind tagged enum. Phase 3.7 ships four impls.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum MovingMassKindConfig {
    /// Phase-3.7.A no-slosh toy.
    RigidLiquid,
    /// Phase-3.7.B Abramson cylindrical-tank equivalent pendulum.
    EquivalentPendulum {
        /// Bare-tank damping ratio (typical academic value `0.005`).
        #[serde(default)]
        damping_ratio_zeta: f64,
    },
    /// Phase-3.7.C linear translational alternative.
    EquivalentSpringMass {
        /// Bare-tank damping ratio.
        #[serde(default)]
        damping_ratio_zeta: f64,
    },
    /// Phase-3.7.C `EquivalentPendulum` with `BaffleModel`-supplied
    /// damping increment.
    BaffledPendulum {
        /// Bare-tank base damping ratio.
        #[serde(default)]
        base_damping_ratio_zeta: f64,
    },
}

impl MovingMassKindConfig {
    fn validate(
        &self,
        index: usize,
        geometry: &TankGeometryConfig,
        vehicle_kind: &str,
    ) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.tanks[{index}].moving_mass.{field}");
        let requires_cylinder = !matches!(self, Self::RigidLiquid);
        if requires_cylinder && !matches!(geometry, TankGeometryConfig::Cylinder { .. }) {
            return Err(ScenarioError::IncompatibleAssemblyEntry {
                field: format!("vehicle.assembly.tanks[{index}]"),
                reason: "non-RigidLiquid moving_mass requires Cylinder geometry".to_owned(),
            });
        }
        let non_rigid_in_point_mass =
            !matches!(self, Self::RigidLiquid) && vehicle_kind == "point_mass";
        if non_rigid_in_point_mass {
            return Err(ScenarioError::IncompatibleAssemblyEntry {
                field: format!("vehicle.assembly.tanks[{index}]"),
                reason: "non-RigidLiquid moving_mass requires vehicle.kind = \"rigid_body\""
                    .to_owned(),
            });
        }
        match self {
            Self::RigidLiquid => {}
            Self::EquivalentPendulum { damping_ratio_zeta }
            | Self::EquivalentSpringMass { damping_ratio_zeta } => {
                require_finite(&path("damping_ratio_zeta"), *damping_ratio_zeta)?;
                if *damping_ratio_zeta < 0.0 {
                    return Err(ScenarioError::InvalidNumber {
                        field: path("damping_ratio_zeta"),
                        value: *damping_ratio_zeta,
                        rule: "must be non-negative",
                    });
                }
            }
            Self::BaffledPendulum {
                base_damping_ratio_zeta,
            } => {
                require_finite(&path("base_damping_ratio_zeta"), *base_damping_ratio_zeta)?;
                if *base_damping_ratio_zeta < 0.0 {
                    return Err(ScenarioError::InvalidNumber {
                        field: path("base_damping_ratio_zeta"),
                        value: *base_damping_ratio_zeta,
                        rule: "must be non-negative",
                    });
                }
            }
        }
        Ok(())
    }
}

/// Baffle model.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct BaffleModelConfig {
    /// Damping-ratio increment added to the inner pendulum's
    /// `damping_ratio_zeta`. Phase-3.7 minimum surface; future
    /// phases may add baffle-area integration per Abramson Eq 7-46.
    pub damping_increment_zeta: f64,
}

impl BaffleModelConfig {
    fn validate(self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.tanks[{index}].baffle_model.{field}");
        require_finite(&path("damping_increment_zeta"), self.damping_increment_zeta)?;
        if self.damping_increment_zeta < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("damping_increment_zeta"),
                value: self.damping_increment_zeta,
                rule: "must be non-negative",
            });
        }
        Ok(())
    }
}

/// Optional initial slosh perturbation. Pendulum variants use angle /
/// angular-rate fields; spring-mass uses displacement / velocity fields.
/// The runner picks the appropriate setter.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct InitialSloshConfig {
    /// `(theta_x_rad, theta_y_rad)` for pendulum kinds, or
    /// absent for spring-mass.
    #[serde(default)]
    pub angles_rad: Option<[f64; 2]>,
    /// `(theta_dot_x_rad_s, theta_dot_y_rad_s)` for pendulum kinds, or
    /// absent for spring-mass.
    #[serde(default)]
    pub rates_rad_s: Option<[f64; 2]>,
    /// `(displacement_x_m, displacement_y_m)` for spring-mass, or
    /// absent for pendulum kinds.
    #[serde(default)]
    pub displacement_body_m: Option<[f64; 2]>,
    /// `(velocity_x_m_s, velocity_y_m_s)` for spring-mass, or absent
    /// for pendulum kinds.
    #[serde(default)]
    pub velocity_body_m_s: Option<[f64; 2]>,
}

impl InitialSloshConfig {
    fn validate(
        &self,
        index: usize,
        moving_mass: MovingMassKindConfig,
    ) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.tanks[{index}].initial_slosh.{field}");
        match moving_mass {
            MovingMassKindConfig::RigidLiquid => {
                return Err(ScenarioError::IncompatibleAssemblyEntry {
                    field: format!("vehicle.assembly.tanks[{index}].initial_slosh"),
                    reason: "rigid_liquid has no slosh state to initialise".to_owned(),
                });
            }
            MovingMassKindConfig::EquivalentPendulum { .. }
            | MovingMassKindConfig::BaffledPendulum { .. } => {
                let angles =
                    self.angles_rad
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: path("angles_rad"),
                            role: ModelRole::Vehicle,
                            name: "equivalent_pendulum".to_owned(),
                        })?;
                let rates =
                    self.rates_rad_s
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: path("rates_rad_s"),
                            role: ModelRole::Vehicle,
                            name: "equivalent_pendulum".to_owned(),
                        })?;
                require_finite_array(&path("angles_rad"), &angles)?;
                require_finite_array(&path("rates_rad_s"), &rates)?;
                if self.displacement_body_m.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: path("displacement_body_m"),
                        role: ModelRole::Vehicle,
                        name: "equivalent_pendulum".to_owned(),
                    });
                }
                if self.velocity_body_m_s.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: path("velocity_body_m_s"),
                        role: ModelRole::Vehicle,
                        name: "equivalent_pendulum".to_owned(),
                    });
                }
            }
            MovingMassKindConfig::EquivalentSpringMass { .. } => {
                let displacement = self.displacement_body_m.ok_or_else(|| {
                    ScenarioError::MissingRequiredField {
                        field: path("displacement_body_m"),
                        role: ModelRole::Vehicle,
                        name: "equivalent_spring_mass".to_owned(),
                    }
                })?;
                let velocity =
                    self.velocity_body_m_s
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: path("velocity_body_m_s"),
                            role: ModelRole::Vehicle,
                            name: "equivalent_spring_mass".to_owned(),
                        })?;
                require_finite_array(&path("displacement_body_m"), &displacement)?;
                require_finite_array(&path("velocity_body_m_s"), &velocity)?;
                if self.angles_rad.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: path("angles_rad"),
                        role: ModelRole::Vehicle,
                        name: "equivalent_spring_mass".to_owned(),
                    });
                }
                if self.rates_rad_s.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: path("rates_rad_s"),
                        role: ModelRole::Vehicle,
                        name: "equivalent_spring_mass".to_owned(),
                    });
                }
            }
        }
        Ok(())
    }
}
