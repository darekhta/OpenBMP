//! Strict scenario document and per-section config structs.
//!
//! All `Config`-suffixed structs use `#[serde(deny_unknown_fields)]`.
//! Validation is performed in [`ScenarioDocument::validate`], which the
//! [`crate::scenario::Scenario`] entry point calls after deserialisation.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::path::PathBuf;

use openbmp_core::ValidationStatus;
use serde::{Deserialize, Serialize};

use crate::checks::{
    require_finite, require_finite_array, require_in_range, require_non_empty,
    require_non_empty_list, require_positive, require_positive_u32, require_supported,
    require_unique, validate_frame_profile,
};
use crate::error::ScenarioError;
use crate::registry::{ModelRegistry, ModelRole};
use crate::solver::SolverConfig;

/// Scenario schema versions supported by this crate.
///
/// The v1 flat scenario shape is retired. v2 is the
/// structured shape (mandatory `[vehicle.assembly]` block, per-body
/// mass and inertia on `[[vehicle.assembly.bodies]]`). v3 is the
/// superset that adds opt-in blocks for multi-instance
/// estimator lanes, autopilot control allocation, FDIR detector
/// tuning, NRLMSISE-00 atmosphere, EGM2008 truncated
/// spherical-harmonic gravity, multi-rate scheduling, and
/// multi-body simultaneous propagation. The v3-only blocks are
/// rejected at validate time when the header declares v2.
pub const SUPPORTED_SCENARIO_VERSIONS: &[u16] = &[2, 3];

/// Latest supported scenario schema version.
pub const LATEST_SCENARIO_VERSION: u16 = 3;

/// v3 schema version. v3-only fields require this header value.
pub const SCENARIO_VERSION_V3: u16 = 3;

/// v2 schema version. v2 scenarios continue to parse byte-identically.
pub const SCENARIO_VERSION_V2: u16 = 2;

/// Minimum accepted `[fc.trajectory]` segment duration (s).
pub const FC_TRAJECTORY_MIN_SEGMENT_DURATION_S: f64 = 1.0e-3;
/// Maximum accepted `[fc.trajectory]` segment duration (s).
pub const FC_TRAJECTORY_MAX_SEGMENT_DURATION_S: f64 = 600.0;

/// Absolute load-time tolerance for stage-separation linear-momentum
/// residuals, in kg*m/s.
pub const STAGE_SEPARATION_MOMENTUM_TOLERANCE_KG_M_S: f64 = 1.0e-9;

const ENTRY_ALTITUDE_MATCH_TOLERANCE_M: f64 = 1.0e-9;
const USSA76_ENTRY_INTERFACE_CEILING_M: f64 = 86_000.0;

/// Default unnormalised WGS84 J2 zonal coefficient used when a scenario
/// selects `gravity = "j2"` and omits `environment.j2`.
///
/// Source: NIMA TR 8350.2 (NGA WGS84), 3rd edition (2000), table 3.5.
pub const WGS84_J2_DEFAULT: f64 = 1.082_626_683e-3;

/// Strict scenario document.
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
    /// Deterministic force ordering. Optional in v2 — when absent the
    /// loader derives the force list from the assembly (`gravity`
    /// always; `thrust` when a motor or engine cluster is declared;
    /// `aero` when an aero deck is declared). Scenarios that need a
    /// non-default order or want to disable a specific force keep
    /// declaring `[forces]` as an explicit override.
    #[serde(default)]
    pub forces: Option<ForcesConfig>,
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
    /// Optional flight-controller configuration block. When present,
    /// the runner constructs an `openbmp-fc` `FlightController` from
    /// this config and drives it lockstepped with the kernel
    /// integrator.
    pub fc: Option<FcConfig>,
    /// Optional simulator-side director controls. This is intended
    /// for test stimulus only; flight-controller scenarios default to
    /// FC-owned mission state unless this block explicitly opts into
    /// kernel authority.
    pub scenario_director: Option<ScenarioDirectorConfig>,
    /// Optional fault-injection hook table.
    pub faults: Option<BTreeMap<String, toml::Value>>,
    /// Optional batch metadata.
    pub batch: Option<BatchConfig>,
    /// Optional offline range-safety landing-footprint
    /// post-processing configuration (v3 only).
    pub landing_footprint: Option<LandingFootprintConfig>,
    /// Optional offline ideal staging budget / mass-optimal split
    /// analysis (v3 only).
    pub staging_analysis: Option<StagingAnalysisConfig>,
    /// Optional descent / entry profile configuration (v3 only).
    pub entry_profile: Option<EntryProfileConfig>,
    /// Optional live aerothermal driver configuration.
    pub aerothermal: Option<AerothermalConfig>,
    /// Optional declarative mission block.
    ///
    /// When present, the runner builds an `openbmp_sim::MissionPhaseGraph`
    /// and a list of `openbmp_sim::EventBinding`s from the parsed
    /// config. When absent, the kernel runs in missionless mode with
    /// no event evaluation.
    pub mission: Option<MissionConfig>,
    /// Optional first-class multi-rate scheduling block (v3 only).
    ///
    /// Parsed under v3 only. Scenarios that declare a
    /// `[schedule]` block must have `openbmp.scenario = 3`.
    pub schedule: Option<ScheduleConfig>,
    /// Optional first-class multi-body propagation block (v3 only).
    ///
    /// Parsed under v3 only. Scenarios that declare a
    /// `[multi_body]` block must have `openbmp.scenario = 3`.
    pub multi_body: Option<MultiBodyConfig>,
}

impl ScenarioDocument {
    /// Borrow the resolved force-model list as a slice.
    ///
    /// Always populated after [`crate::Scenario::from_toml_str`] /
    /// `from_file` because the parse layer synthesises a default list
    /// from the assembly when `[forces]` is absent. Callers that
    /// build a `ScenarioDocument` directly (without going through
    /// `Scenario::from_*`) are expected to populate `forces`
    /// themselves; the empty-slice fallback keeps this method total.
    #[must_use]
    pub fn force_models(&self) -> &[String] {
        self.forces
            .as_ref()
            .map_or(&[][..], |f| f.models.as_slice())
    }

    /// Return the resolved force-model list. When `[forces]` is
    /// declared explicitly, returns the declared list; otherwise
    /// returns the list derived from the assembly:
    ///
    /// - `gravity` (always),
    /// - `thrust` if `[propulsion.motor]` or
    ///   `[[vehicle.assembly.engines]]` is present,
    /// - `aero` if `[aero]` is present.
    ///
    /// The derived order matches the conventional ordering
    /// `gravity → thrust → aero` so existing hand-listed scenarios
    /// can drop `[forces]` and continue to produce the same kernel
    /// hot path.
    #[must_use]
    pub fn resolved_force_models(&self) -> Vec<String> {
        if let Some(forces) = &self.forces {
            return forces.models.clone();
        }
        let mut models = vec!["gravity".to_owned()];
        let has_thrust = self.propulsion.as_ref().is_some_and(|p| p.motor.is_some())
            || !self.vehicle.assembly.engines.is_empty();
        if has_thrust {
            models.push("thrust".to_owned());
        }
        if self.aero.is_some() {
            models.push("aero".to_owned());
        }
        models
    }

    /// Force-like model names used either by the default force stack
    /// or any per-phase override.
    #[must_use]
    pub fn force_model_universe(&self) -> Vec<String> {
        let mut models = self.resolved_force_models();
        if let Some(forces) = &self.forces {
            for override_config in &forces.phase_override {
                for model in &override_config.models {
                    if !models.contains(model) {
                        models.push(model.clone());
                    }
                }
            }
        }
        models
    }

    /// Validate semantic constraints against a model registry.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] when the document violates the
    /// scenario contract.
    pub fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        self.validate_header()?;
        // Reject v2 scenarios that name a v3-only kind selector before
        // running the per-kind field validation, so users see "egm2008
        // is reserved for v3" rather than a downstream
        // `UnexpectedField` for a config field that is only valid
        // alongside that selector under v3. `egm2008` is consumed
        // under v3, so the reservation gate fires only under v2.
        self.validate_v3_kind_availability()?;
        self.meta.validate()?;
        self.time.validate()?;
        if let Some(epoch) = &self.epoch {
            epoch.validate()?;
        }
        self.vehicle.validate(registry, self.time.dt_s)?;
        self.environment.validate(registry)?;
        if let Some(forces) = &self.forces {
            forces.validate(registry)?;
        }
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
        if self.environment.gravity == "third_body"
            && self.environment.ephemeris.as_deref() == Some("spk")
        {
            let epoch = self
                .epoch
                .as_ref()
                .ok_or_else(|| ScenarioError::MissingRequiredField {
                    field: "epoch".to_owned(),
                    role: ModelRole::Gravity,
                    name: "third_body".to_owned(),
                })?;
            match epoch.scale.to_ascii_uppercase().as_str() {
                "TDB" | "TT" => {}
                "UTC" => {
                    if epoch.leap_second_table.is_none()
                        && self.environment.ephemeris_meta_kernel.is_none()
                    {
                        return Err(ScenarioError::MissingRequiredField {
                            field: "epoch.leap_second_table".to_owned(),
                            role: ModelRole::Gravity,
                            name: "third_body spk ephemeris".to_owned(),
                        });
                    }
                }
                _ => {
                    return Err(ScenarioError::UnsupportedValue {
                        field: "epoch.scale".to_owned(),
                        value: epoch.scale.clone(),
                    });
                }
            }
        }
        self.validate_frame_epoch_requirements()?;
        if let Some(aero) = &self.aero {
            aero.validate()?;
        }
        if let Some(aerothermal) = &self.aerothermal {
            aerothermal.validate(self.time.dt_s)?;
            if aerothermal
                .ablation
                .as_ref()
                .is_some_and(|ablation| ablation.feedback == "mass")
                && self.vehicle.kind != "rigid_body"
            {
                return Err(ScenarioError::InconsistentSection {
                    field_a: "aerothermal.ablation.feedback".to_owned(),
                    value_a: "mass".to_owned(),
                    field_b: "vehicle.kind".to_owned(),
                    value_b: self.vehicle.kind.clone(),
                });
            }
        }
        if let Some(propulsion) = &self.propulsion {
            propulsion.validate(registry)?;
        }
        self.validate_top_level_resource_owners()?;
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
        } else if self.environment.wind != "none" {
            return Err(ScenarioError::MissingRequiredField {
                field: "wind".to_owned(),
                role: ModelRole::Wind,
                name: self.environment.wind.clone(),
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
        self.validate_force_phase_overrides()?;
        if let Some(fc) = &self.fc {
            fc.validate()?;
        }
        self.validate_scenario_director()?;
        self.validate_fc_sensor_boundary()?;
        self.validate_fc_relative_nav_observability()?;
        self.validate_landing_footprint_agreement()?;
        self.validate_entry_profile_agreement()?;
        self.validate_ascent_reference_agreement()?;
        self.validate_effector_references()?;
        self.validate_engine_references()?;
        self.validate_recovery_references()?;
        self.validate_relative_distance_trigger_references()?;
        self.validate_multi_body_attitude_target_references()?;
        self.validate_propulsion_unambiguous()?;
        self.validate_v3_blocks()?;
        self.validate_initial_multi_body_references()?;
        self.validate_stage_separation_agreement()?;
        Ok(())
    }

    /// Returns `true` when the flight controller should own mission
    /// state for this scenario.
    #[must_use]
    pub fn flight_controller_owns_mission_state(&self) -> bool {
        self.fc.is_some()
            && self.mission.is_some()
            && !matches!(
                self.scenario_director
                    .as_ref()
                    .map(|director| director.mission_authority),
                Some(ScenarioMissionAuthority::Kernel)
            )
    }

    fn validate_scenario_director(&self) -> Result<(), ScenarioError> {
        let Some(director) = &self.scenario_director else {
            return Ok(());
        };
        if director.mission_authority == ScenarioMissionAuthority::FlightController
            && self.fc.is_none()
        {
            return Err(ScenarioError::InconsistentSection {
                field_a: "scenario_director.mission_authority".to_owned(),
                value_a: "flight_controller".to_owned(),
                field_b: "fc".to_owned(),
                value_b: "missing".to_owned(),
            });
        }
        if director.mission_authority == ScenarioMissionAuthority::FlightController
            && self.mission.is_none()
        {
            return Err(ScenarioError::InconsistentSection {
                field_a: "scenario_director.mission_authority".to_owned(),
                value_a: "flight_controller".to_owned(),
                field_b: "mission".to_owned(),
                value_b: "missing".to_owned(),
            });
        }
        Ok(())
    }

    fn validate_fc_sensor_boundary(&self) -> Result<(), ScenarioError> {
        if self.fc.is_none() {
            return Ok(());
        }
        let Some(sensors) = &self.sensors else {
            return Ok(());
        };
        if let Some((name, _)) = sensors
            .iter()
            .find(|(_, config)| config.kind == "ideal_state")
        {
            return Err(ScenarioError::InvalidFc {
                reason: format!(
                    "[fc] scenarios may not use sensors.{name}.kind = \"ideal_state\"; \
                     IdealStateSensor echoes integrated truth and bypasses the sensor boundary"
                ),
            });
        }
        Ok(())
    }

    fn validate_fc_relative_nav_observability(&self) -> Result<(), ScenarioError> {
        if self.fc.is_none() {
            return Ok(());
        }
        let Some(mission) = &self.mission else {
            return Ok(());
        };
        for (event_index, event) in mission.events.iter().enumerate() {
            let kind = match event.trigger {
                EventTriggerConfig::AtRelativeDistance { .. } => "at_relative_distance",
                EventTriggerConfig::AtRelativeSpeed { .. } => "at_relative_speed",
                _ => continue,
            };
            return Err(ScenarioError::InvalidFc {
                reason: format!(
                    "[fc] scenarios may not use mission.events[{event_index}].trigger.kind = \
                     \"{kind}\" until an onboard relative-navigation observable is wired; \
                     simulator truth-relative maps are not available to the flight controller"
                ),
            });
        }
        Ok(())
    }

    fn validate_frame_epoch_requirements(&self) -> Result<(), ScenarioError> {
        let profile = self.environment.frame_profile.as_str();
        if profile == "iers-tabulated" {
            let epoch = self
                .epoch
                .as_ref()
                .ok_or_else(|| ScenarioError::MissingRequiredField {
                    field: "epoch".to_owned(),
                    role: ModelRole::Frame,
                    name: "iers-tabulated".to_owned(),
                })?;
            if epoch.eop.is_none() {
                return Err(ScenarioError::MissingRequiredField {
                    field: "epoch.eop".to_owned(),
                    role: ModelRole::Frame,
                    name: "iers-tabulated".to_owned(),
                });
            }
        } else if let Some(epoch) = &self.epoch {
            if epoch.eop.is_some() {
                return Err(ScenarioError::UnexpectedField {
                    field: "epoch.eop".to_owned(),
                    role: ModelRole::Frame,
                    name: profile.to_owned(),
                });
            }
            if epoch.eop_sha256.is_some() {
                return Err(ScenarioError::UnexpectedField {
                    field: "epoch.eop_sha256".to_owned(),
                    role: ModelRole::Frame,
                    name: profile.to_owned(),
                });
            }
        }
        Ok(())
    }

    fn validate_v3_blocks(&self) -> Result<(), ScenarioError> {
        let header = self.openbmp.scenario;
        self.validate_v3_top_level_blocks(header)?;
        self.validate_v3_fc_blocks(header, self.time.dt_s)?;
        self.validate_v3_effector_kinds(header)?;
        Ok(())
    }

    fn validate_top_level_resource_owners(&self) -> Result<(), ScenarioError> {
        let body_ids: std::collections::BTreeSet<&str> = self
            .vehicle
            .assembly
            .bodies
            .iter()
            .map(|body| body.id.as_str())
            .collect();
        if let Some(aero) = &self.aero
            && let Some(owner) = &aero.mounted_to
            && !body_ids.contains(owner.as_str())
        {
            return Err(ScenarioError::UnknownBodyReference {
                field: "aero.mounted_to".to_owned(),
                value: owner.clone(),
            });
        }
        if let Some(motor) = self
            .propulsion
            .as_ref()
            .and_then(|propulsion| propulsion.motor.as_ref())
            && let Some(owner) = &motor.mounted_to
            && !body_ids.contains(owner.as_str())
        {
            return Err(ScenarioError::UnknownBodyReference {
                field: "propulsion.motor.mounted_to".to_owned(),
                value: owner.clone(),
            });
        }
        Ok(())
    }

    fn validate_v3_effector_kinds(&self, header: u16) -> Result<(), ScenarioError> {
        if header >= SCENARIO_VERSION_V3 {
            return Ok(());
        }
        for (index, effector) in self.vehicle.assembly.effectors.iter().enumerate() {
            if matches!(effector.kind, EffectorKindConfig::DirectTorque { .. }) {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: format!("vehicle.assembly.effectors[{index}].kind = \"direct_torque\""),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
        }
        Ok(())
    }

    fn validate_v3_top_level_blocks(&self, header: u16) -> Result<(), ScenarioError> {
        gate_v3_block(
            header,
            "schedule",
            "multi-rate scheduling",
            self.schedule.as_ref(),
            || {
                self.schedule
                    .as_ref()
                    .map_or(Ok(()), ScheduleConfig::validate)
            },
        )?;
        if let Some(multi_body) = self.multi_body.as_ref() {
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "multi_body".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            multi_body.validate()?;
        }
        if let Some(landing_footprint) = self.landing_footprint.as_ref() {
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "landing_footprint".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            landing_footprint.validate()?;
        }
        if let Some(staging_analysis) = self.staging_analysis.as_ref() {
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "staging_analysis".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            staging_analysis.validate()?;
        }
        if let Some(entry_profile) = self.entry_profile.as_ref() {
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "entry_profile".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            entry_profile.validate()?;
        }
        if self.aerothermal.is_some() && header < SCENARIO_VERSION_V3 {
            return Err(ScenarioError::SchemaVersionFieldReserved {
                field: "aerothermal".to_owned(),
                required: SCENARIO_VERSION_V3,
                found: header,
            });
        }
        Ok(())
    }

    #[allow(clippy::too_many_lines)]
    fn validate_v3_fc_blocks(&self, header: u16, dt_s: f64) -> Result<(), ScenarioError> {
        let Some(fc) = &self.fc else {
            return Ok(());
        };
        // fc.estimator v3-only additions — IMM
        // and SR-UKF estimator surfaces. The runner builds IMM from
        // [fc.ekf] plus [fc.imm], and SR-UKF variants from [fc.ekf].
        if let Some(field) = v3_only_fc_estimator_field(fc.estimator)
            && header < SCENARIO_VERSION_V3
        {
            return Err(ScenarioError::SchemaVersionFieldReserved {
                field: field.to_owned(),
                required: SCENARIO_VERSION_V3,
                found: header,
            });
        }
        if let Some(field) = v3_only_fc_guidance_field(fc.guidance)
            && header < SCENARIO_VERSION_V3
        {
            return Err(ScenarioError::SchemaVersionFieldReserved {
                field: field.to_owned(),
                required: SCENARIO_VERSION_V3,
                found: header,
            });
        }
        if fc.ascent_reference.is_some() && header < SCENARIO_VERSION_V3 {
            return Err(ScenarioError::SchemaVersionFieldReserved {
                field: "fc.ascent_reference".to_owned(),
                required: SCENARIO_VERSION_V3,
                found: header,
            });
        }
        if let Some(imm) = fc.imm.as_ref() {
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.imm".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            imm.validate()?;
        }
        // `[fc.estimator_lanes]` is consumed under v3 only; the
        // runner builds a `MultiLaneEstimator` containing one
        // estimator per lane config and registers it as the single
        // scheduled estimator job, with the configured voter policy
        // selecting the active lane each tick.
        if let Some(lanes_cfg) = fc.estimator_lanes.as_ref() {
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.estimator_lanes".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            lanes_cfg.validate()?;
        }
        // fc.autopilot_allocation — consumed block,
        // v3-only; the runner builds a `ControlAllocator` from this
        // block plus the per-effector axis declarations and installs
        // it on the mixer. The PseudoInverse kind is consumed by the
        // bounded direct-axis pseudo-inverse path; the future coupled
        // `G_eff` schema remains out of scope here.
        if let Some(allocation) = fc.autopilot_allocation.as_ref() {
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.autopilot_allocation".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            allocation.validate()?;
        }
        if let Some(fdir) = &fc.fdir
            && let Some(detector) = fdir.detector.as_ref()
        {
            // `[fc.fdir.detector]` consumed block, v3-only;
            // the runner promotes the typed `kind` enum into a
            // `DetectorKind::WindowedMeanShiftGlrt` on the FDIR job.
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.fdir.detector".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            detector.validate()?;
        }
        if let Some(fdir) = &fc.fdir
            && let Some(redlines) = fdir.redlines.as_ref()
        {
            // `[fc.fdir.redlines]` is a consumed v3-only I-load
            // block; the runner maps it into `FdirParams`
            // watchpoints evaluated by the FDIR job.
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.fdir.redlines".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            redlines.validate()?;
        }
        // fc.autopilot_params.l1_adaptive — consumed
        // block, v3-only; the runner translates to AutopilotParams.l1_adaptive
        // which the rate loop consumes.
        if let Some(autopilot_params) = fc.autopilot_params.as_ref()
            && let Some(l1) = autopilot_params.l1_adaptive.as_ref()
        {
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.autopilot_params.l1_adaptive".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            l1.validate(dt_s)?;
        }

        // fc.autopilot_params.anti_windup — v3-only block. The runner
        // translates it to AutopilotParams.anti_windup, which all three
        // PID loops consume. Absent → AntiWindupKind::default()
        // (back-calculation, unit gain).
        if let Some(autopilot_params) = fc.autopilot_params.as_ref()
            && let Some(anti_windup) = autopilot_params.anti_windup.as_ref()
        {
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.autopilot_params.anti_windup".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            anti_windup.validate()?;
        }

        // fc.autopilot_params.rate_loop_kind / .lqr / .indi —
        // the LQR rate-loop pair and the INDI rate-loop pair.
        // Both kinds are v3-only and require their
        // matching parameter block. The parser fails closed on:
        //   - rate_loop_kind = "lqr" without [fc.autopilot_params.lqr]
        //   - rate_loop_kind = "indi" without [fc.autopilot_params.indi]
        //   - either parameter block declared without the matching
        //     rate_loop_kind value
        //   - rate_loop_kind = "indi" combined with l1_adaptive (the
        //     filter-interaction concerns documented in
        //     openbmp_fc::indi)
        if let Some(autopilot_params) = fc.autopilot_params.as_ref() {
            if let Some(kind) = autopilot_params.rate_loop_kind {
                if header < SCENARIO_VERSION_V3 {
                    return Err(ScenarioError::SchemaVersionFieldReserved {
                        field: "fc.autopilot_params.rate_loop_kind".to_owned(),
                        required: SCENARIO_VERSION_V3,
                        found: header,
                    });
                }
                if kind == FcRateLoopKind::Lqr && autopilot_params.lqr.is_none() {
                    return Err(ScenarioError::MissingRequiredField {
                        field: "fc.autopilot_params.lqr".to_owned(),
                        role: ModelRole::Controller,
                        name: "lqr".to_owned(),
                    });
                }
                if kind == FcRateLoopKind::Indi && autopilot_params.indi.is_none() {
                    return Err(ScenarioError::MissingRequiredField {
                        field: "fc.autopilot_params.indi".to_owned(),
                        role: ModelRole::Controller,
                        name: "indi".to_owned(),
                    });
                }
                if kind == FcRateLoopKind::Indi && autopilot_params.l1_adaptive.is_some() {
                    return Err(ScenarioError::InconsistentSection {
                        field_a: "fc.autopilot_params.rate_loop_kind".to_owned(),
                        value_a: "indi".to_owned(),
                        field_b: "fc.autopilot_params.l1_adaptive".to_owned(),
                        value_b: "present".to_owned(),
                    });
                }
            }
            if let Some(lqr) = autopilot_params.lqr.as_ref() {
                if header < SCENARIO_VERSION_V3 {
                    return Err(ScenarioError::SchemaVersionFieldReserved {
                        field: "fc.autopilot_params.lqr".to_owned(),
                        required: SCENARIO_VERSION_V3,
                        found: header,
                    });
                }
                if autopilot_params.rate_loop_kind != Some(FcRateLoopKind::Lqr) {
                    return Err(ScenarioError::InconsistentSection {
                        field_a: "fc.autopilot_params.lqr".to_owned(),
                        value_a: "present".to_owned(),
                        field_b: "fc.autopilot_params.rate_loop_kind".to_owned(),
                        value_b: format!("{:?}", autopilot_params.rate_loop_kind),
                    });
                }
                lqr.validate()?;
            }
            if let Some(indi) = autopilot_params.indi.as_ref() {
                if header < SCENARIO_VERSION_V3 {
                    return Err(ScenarioError::SchemaVersionFieldReserved {
                        field: "fc.autopilot_params.indi".to_owned(),
                        required: SCENARIO_VERSION_V3,
                        found: header,
                    });
                }
                if autopilot_params.rate_loop_kind != Some(FcRateLoopKind::Indi) {
                    return Err(ScenarioError::InconsistentSection {
                        field_a: "fc.autopilot_params.indi".to_owned(),
                        value_a: "present".to_owned(),
                        field_b: "fc.autopilot_params.rate_loop_kind".to_owned(),
                        value_b: format!("{:?}", autopilot_params.rate_loop_kind),
                    });
                }
                indi.validate(dt_s)?;
            }

            // fc.autopilot_params.attitude_loop_kind / .attitude_mpc
            // — consumed pair. Both v3-only. When
            // attitude_loop_kind = "mpc" the [fc.autopilot_params.attitude_mpc]
            // block is required so the runner can build the
            // RecedingHorizonAttitudeMpc; conversely the block is
            // meaningless without the kind selector.
            if let Some(attitude_kind) = autopilot_params.attitude_loop_kind {
                if header < SCENARIO_VERSION_V3 {
                    return Err(ScenarioError::SchemaVersionFieldReserved {
                        field: "fc.autopilot_params.attitude_loop_kind".to_owned(),
                        required: SCENARIO_VERSION_V3,
                        found: header,
                    });
                }
                if attitude_kind == FcAttitudeLoopKind::Mpc
                    && autopilot_params.attitude_mpc.is_none()
                {
                    return Err(ScenarioError::MissingRequiredField {
                        field: "fc.autopilot_params.attitude_mpc".to_owned(),
                        role: ModelRole::Controller,
                        name: "attitude_mpc".to_owned(),
                    });
                }
            }
            if let Some(attitude_mpc) = autopilot_params.attitude_mpc.as_ref() {
                if header < SCENARIO_VERSION_V3 {
                    return Err(ScenarioError::SchemaVersionFieldReserved {
                        field: "fc.autopilot_params.attitude_mpc".to_owned(),
                        required: SCENARIO_VERSION_V3,
                        found: header,
                    });
                }
                if autopilot_params.attitude_loop_kind != Some(FcAttitudeLoopKind::Mpc) {
                    return Err(ScenarioError::InconsistentSection {
                        field_a: "fc.autopilot_params.attitude_mpc".to_owned(),
                        value_a: "present".to_owned(),
                        field_b: "fc.autopilot_params.attitude_loop_kind".to_owned(),
                        value_b: format!("{:?}", autopilot_params.attitude_loop_kind),
                    });
                }
                attitude_mpc.validate()?;
            }
            if let Some(notches) = autopilot_params.gyro_notch.as_ref() {
                for (axis, notch) in notches.iter().enumerate() {
                    notch.validate(axis)?;
                }
            }
        }

        // fc.trajectory — consumed block, v3-only; the
        // runner builds a `MinimumSnapTrajectory` from this block and
        // installs it on the autopilot.
        if let Some(trajectory) = &fc.trajectory {
            if header < SCENARIO_VERSION_V3 {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.trajectory".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            trajectory.validate()?;
            // Cross-check with autopilot_params.trajectory_kind.
            let autopilot_kind = fc.autopilot_params.as_ref().and_then(|p| p.trajectory_kind);
            match (autopilot_kind, trajectory.kind) {
                (Some(FcTrajectoryKind::MinimumSnap), FcTrajectoryConfigKind::MinimumSnap) => {}
                (Some(other), FcTrajectoryConfigKind::MinimumSnap) => {
                    return Err(ScenarioError::InconsistentSection {
                        field_a: "fc.autopilot_params.trajectory_kind".to_owned(),
                        value_a: format!("{other:?}"),
                        field_b: "fc.trajectory.kind".to_owned(),
                        value_b: "minimum_snap".to_owned(),
                    });
                }
                (None, FcTrajectoryConfigKind::MinimumSnap) => {
                    return Err(ScenarioError::MissingRequiredField {
                        field: "fc.autopilot_params.trajectory_kind".to_owned(),
                        role: ModelRole::Trajectory,
                        name: "minimum_snap".to_owned(),
                    });
                }
            }
        } else if matches!(
            fc.autopilot_params.as_ref().and_then(|p| p.trajectory_kind),
            Some(FcTrajectoryKind::MinimumSnap)
        ) {
            return Err(ScenarioError::MissingRequiredField {
                field: "fc.trajectory".to_owned(),
                role: ModelRole::Trajectory,
                name: "minimum_snap".to_owned(),
            });
        }
        Ok(())
    }

    /// Pre-pass v2 availability gate for v3-only kind selectors.
    ///
    /// Runs before per-kind field validation so users see a clean
    /// "feature is reserved for v3" diagnostic instead of a downstream
    /// `UnexpectedField` for fields that are only valid alongside that
    /// selector. Always emits `SchemaVersionFieldReserved` because the
    /// currently listed selectors are consumed under v3.
    fn validate_v3_kind_availability(&self) -> Result<(), ScenarioError> {
        let header = self.openbmp.scenario;
        if header >= SCENARIO_VERSION_V3 {
            return Ok(());
        }
        if self.environment.gravity == "egm2008" || self.environment.gravity == "third_body" {
            return Err(ScenarioError::SchemaVersionFieldReserved {
                field: format!("environment.gravity = \"{}\"", self.environment.gravity),
                required: SCENARIO_VERSION_V3,
                found: header,
            });
        }
        if self.environment.atmosphere == "nrlmsise00"
            || self.environment.atmosphere == "nrlmsis2_compat"
            || self
                .atmosphere
                .as_ref()
                .is_some_and(|a| a.kind == "nrlmsise00" || a.kind == "nrlmsis2_compat")
        {
            let kind = self
                .atmosphere
                .as_ref()
                .map_or(self.environment.atmosphere.as_str(), |a| a.kind.as_str());
            return Err(ScenarioError::SchemaVersionFieldReserved {
                field: format!("atmosphere.kind = \"{kind}\""),
                required: SCENARIO_VERSION_V3,
                found: header,
            });
        }
        if self.environment.atmosphere == "piecewise_exponential"
            || self
                .atmosphere
                .as_ref()
                .is_some_and(|a| a.kind == "piecewise_exponential")
        {
            return Err(ScenarioError::SchemaVersionFieldReserved {
                field: "atmosphere.kind = \"piecewise_exponential\"".to_owned(),
                required: SCENARIO_VERSION_V3,
                found: header,
            });
        }
        if self.landing_footprint.is_some() {
            return Err(ScenarioError::SchemaVersionFieldReserved {
                field: "landing_footprint".to_owned(),
                required: SCENARIO_VERSION_V3,
                found: header,
            });
        }
        if self.entry_profile.is_some() {
            return Err(ScenarioError::SchemaVersionFieldReserved {
                field: "entry_profile".to_owned(),
                required: SCENARIO_VERSION_V3,
                found: header,
            });
        }
        if let Some(fc) = self.fc.as_ref() {
            if let Some(field) = v3_only_fc_estimator_field(fc.estimator) {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: field.to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            if let Some(field) = v3_only_fc_guidance_field(fc.guidance) {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: field.to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            if fc.ascent_reference.is_some() {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.ascent_reference".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            if fc.imm.is_some() {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.imm".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
            if fc.estimator_lanes.is_some() {
                return Err(ScenarioError::SchemaVersionFieldReserved {
                    field: "fc.estimator_lanes".to_owned(),
                    required: SCENARIO_VERSION_V3,
                    found: header,
                });
            }
        }
        Ok(())
    }

    fn validate_propulsion_unambiguous(&self) -> Result<(), ScenarioError> {
        let has_motor = self.propulsion.as_ref().is_some_and(|p| p.motor.is_some());
        let has_engines = !self.vehicle.assembly.engines.is_empty();
        if has_motor && has_engines {
            return Err(ScenarioError::AmbiguousPropulsion);
        }
        Ok(())
    }

    fn validate_ascent_reference_agreement(&self) -> Result<(), ScenarioError> {
        let Some(fc) = &self.fc else {
            return Ok(());
        };
        let uses_ascent_reference =
            matches!(fc.guidance, FcGuidanceKind::AscentReference) || fc.ascent_reference.is_some();
        if !uses_ascent_reference {
            return Ok(());
        }
        if self.vehicle.kind != "rigid_body" {
            return Err(ScenarioError::IncompatibleAssemblyEntry {
                field: "fc.ascent_reference".to_owned(),
                reason: "ascent reference requires vehicle.kind = \"rigid_body\"".to_owned(),
            });
        }
        let Some(mission) = &self.mission else {
            return Err(ScenarioError::InvalidFc {
                reason: "guidance = \"ascent_reference\" requires a powered_ascent mission phase"
                    .to_owned(),
            });
        };
        if !mission_declares_powered_ascent(mission) {
            return Err(ScenarioError::InvalidFc {
                reason: "guidance = \"ascent_reference\" requires a declared powered_ascent mission phase"
                    .to_owned(),
            });
        }
        if !fc_gain_schedule_declares_powered_ascent(&fc.gain_schedule) {
            return Err(ScenarioError::InvalidFc {
                reason: "guidance = \"ascent_reference\" requires [fc.gain_schedule.\"mission.phases.powered_ascent\"] or [fc.gain_schedule.\"mission.states.powered_ascent\"]"
                    .to_owned(),
            });
        }
        Ok(())
    }

    fn validate_landing_footprint_agreement(&self) -> Result<(), ScenarioError> {
        let Some(landing_footprint) = &self.landing_footprint else {
            return Ok(());
        };
        let Some(mission) = &self.mission else {
            return Err(ScenarioError::InconsistentSection {
                field_a: "landing_footprint".to_owned(),
                value_a: "declared".to_owned(),
                field_b: "mission".to_owned(),
                value_b: "missing".to_owned(),
            });
        };
        if !mission_declares_coast_or_ballistic_descent(mission) {
            return Err(ScenarioError::InconsistentSection {
                field_a: "landing_footprint".to_owned(),
                value_a: "declared".to_owned(),
                field_b: "mission.phase_or_state.id".to_owned(),
                value_b: "no coast or ballistic_descent phase".to_owned(),
            });
        }
        let expected_gravity = landing_footprint.method.required_gravity_name();
        if self.environment.gravity != expected_gravity {
            return Err(ScenarioError::InconsistentSection {
                field_a: "landing_footprint.method".to_owned(),
                value_a: landing_footprint.method.as_str().to_owned(),
                field_b: "environment.gravity".to_owned(),
                value_b: self.environment.gravity.clone(),
            });
        }
        if landing_footprint.include_geodetic
            && self
                .frames
                .as_ref()
                .is_none_or(|frames| frames.local_origin.is_none())
        {
            return Err(ScenarioError::InconsistentSection {
                field_a: "landing_footprint.include_geodetic".to_owned(),
                value_a: "true".to_owned(),
                field_b: "frames.local_origin".to_owned(),
                value_b: "missing".to_owned(),
            });
        }
        Ok(())
    }

    fn validate_entry_profile_agreement(&self) -> Result<(), ScenarioError> {
        let Some(entry_profile) = &self.entry_profile else {
            return Ok(());
        };
        let Some(mission) = &self.mission else {
            return Err(ScenarioError::InconsistentSection {
                field_a: "entry_profile".to_owned(),
                value_a: "declared".to_owned(),
                field_b: "mission".to_owned(),
                value_b: "missing".to_owned(),
            });
        };
        for phase in ["entry_interface", "final_descent", "recovery"] {
            if !mission_declares_phase_or_state(mission, phase) {
                return Err(ScenarioError::InconsistentSection {
                    field_a: "entry_profile".to_owned(),
                    value_a: "declared".to_owned(),
                    field_b: "mission.phase_or_state.id".to_owned(),
                    value_b: format!("missing {phase} phase"),
                });
            }
        }
        if !mission_has_descending_entry_handoff(
            mission,
            "entry_interface",
            entry_profile.entry_interface_altitude_m,
        ) {
            return Err(ScenarioError::InconsistentSection {
                field_a: "entry_profile.entry_interface_altitude_m".to_owned(),
                value_a: entry_profile.entry_interface_altitude_m.to_string(),
                field_b: "mission.events".to_owned(),
                value_b: "no at_altitude_descending enter_phase entry_interface event".to_owned(),
            });
        }
        if let Some(final_descent_altitude_m) = entry_profile.final_descent_altitude_m
            && !mission_has_descending_entry_handoff(
                mission,
                "final_descent",
                final_descent_altitude_m,
            )
        {
            return Err(ScenarioError::InconsistentSection {
                field_a: "entry_profile.final_descent_altitude_m".to_owned(),
                value_a: final_descent_altitude_m.to_string(),
                field_b: "mission.events".to_owned(),
                value_b: "no at_altitude_descending enter_phase final_descent event".to_owned(),
            });
        }
        if self.aero.is_none() || !self.force_models().iter().any(|name| name == "aero") {
            return Err(ScenarioError::InconsistentSection {
                field_a: "entry_profile".to_owned(),
                value_a: "declared".to_owned(),
                field_b: "aero/forces.models".to_owned(),
                value_b: "missing aero force model".to_owned(),
            });
        }
        let atmosphere_kind = self
            .atmosphere
            .as_ref()
            .map_or(self.environment.atmosphere.as_str(), |a| a.kind.as_str());
        validate_entry_atmosphere_envelope(
            atmosphere_kind,
            entry_profile.entry_interface_altitude_m,
        )?;
        if entry_profile.mode == EntryProfileMode::Lifting {
            if self.vehicle.kind != "rigid_body" {
                return Err(ScenarioError::IncompatibleAssemblyEntry {
                    field: "entry_profile.mode = \"lifting\"".to_owned(),
                    reason: "lifting entry requires vehicle.kind = \"rigid_body\"".to_owned(),
                });
            }
            if !mission_declares_phase_or_state(mission, "lifting_entry") {
                return Err(ScenarioError::InconsistentSection {
                    field_a: "entry_profile.mode".to_owned(),
                    value_a: "lifting".to_owned(),
                    field_b: "mission.phase_or_state.id".to_owned(),
                    value_b: "missing lifting_entry phase".to_owned(),
                });
            }
        }
        Ok(())
    }

    fn validate_stage_separation_agreement(&self) -> Result<(), ScenarioError> {
        let Some(mission) = &self.mission else {
            if let Some(multi_body) = &self.multi_body
                && !multi_body.separations.is_empty()
            {
                return Err(ScenarioError::InconsistentSection {
                    field_a: "multi_body.separation".to_owned(),
                    value_a: "declared".to_owned(),
                    field_b: "mission.events".to_owned(),
                    value_b: "missing".to_owned(),
                });
            }
            return Ok(());
        };

        let jettisons = collect_jettison_stage_actions(mission);
        let multi_body_separations = self
            .multi_body
            .as_ref()
            .map_or(&[][..], |multi_body| multi_body.separations.as_slice());
        if jettisons.is_empty() && multi_body_separations.is_empty() {
            return Ok(());
        }
        if self.vehicle.kind != "rigid_body" {
            return Err(ScenarioError::IncompatibleAssemblyEntry {
                field: "mission.events.action.kind = \"jettison_stage\"".to_owned(),
                reason: "stage separation requires vehicle.kind = \"rigid_body\"".to_owned(),
            });
        }
        if self.multi_body.is_none() {
            return Err(ScenarioError::InconsistentSection {
                field_a: "mission.events.action.kind".to_owned(),
                value_a: "jettison_stage".to_owned(),
                field_b: "multi_body".to_owned(),
                value_b: "missing".to_owned(),
            });
        }

        let body_masses = assembly_body_mass_lookup(&self.vehicle.assembly);
        validate_multi_body_resource_ownership(self, &body_masses)?;
        validate_stage_separation_body_references(
            &jettisons,
            multi_body_separations,
            &body_masses,
        )?;
        validate_stage_separation_uniqueness(&jettisons, multi_body_separations)?;
        validate_stage_separation_symmetry_and_momentum(
            &jettisons,
            multi_body_separations,
            &body_masses,
        )?;
        Ok(())
    }

    fn validate_engine_references(&self) -> Result<(), ScenarioError> {
        let Some(mission) = &self.mission else {
            return Ok(());
        };
        let declared: BTreeSet<&str> = self
            .vehicle
            .assembly
            .engines
            .iter()
            .map(|e| e.id.as_str())
            .collect();

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
        for (state_index, state) in mission.states.iter().enumerate() {
            for (engine_index, id) in state.allowed_engines.iter().enumerate() {
                if !declared.contains(id.as_str()) {
                    return Err(ScenarioError::UnknownEngineReference {
                        field: format!(
                            "mission.states[{state_index}].allowed_engines[{engine_index}]"
                        ),
                        id: id.clone(),
                    });
                }
            }
        }
        for (event_index, event) in mission.events.iter().enumerate() {
            if let ScenarioActionConfig::EngineCommand { id, .. } = &event.action
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

    fn validate_recovery_references(&self) -> Result<(), ScenarioError> {
        let Some(mission) = &self.mission else {
            return Ok(());
        };
        let recovery_kinds: BTreeMap<&str, &'static str> = self
            .vehicle
            .assembly
            .recovery
            .iter()
            .map(|r| (r.id.as_str(), r.kind.kind_name()))
            .collect();

        for (event_index, event) in mission.events.iter().enumerate() {
            if let ScenarioActionConfig::DeployRecovery { id, command } = &event.action {
                let Some(kind_name) = recovery_kinds.get(id.as_str()) else {
                    return Err(ScenarioError::UnknownRecoveryReference {
                        field: format!("mission.events[{event_index}].action.id"),
                        id: id.clone(),
                    });
                };
                if !is_recovery_command_compatible(kind_name, command) {
                    return Err(ScenarioError::IncompatibleRecoveryCommand {
                        field: format!("mission.events[{event_index}].action.command"),
                        command: command.clone(),
                        kind: (*kind_name).to_owned(),
                    });
                }
            }
        }
        Ok(())
    }

    fn validate_relative_distance_trigger_references(&self) -> Result<(), ScenarioError> {
        let Some(mission) = &self.mission else {
            return Ok(());
        };
        let body_ids: BTreeSet<&str> = self
            .vehicle
            .assembly
            .bodies
            .iter()
            .map(|body| body.id.as_str())
            .collect();

        for (event_index, event) in mission.events.iter().enumerate() {
            let (kind, body, reference_body) = match &event.trigger {
                EventTriggerConfig::AtRelativeDistance {
                    body,
                    reference_body,
                    ..
                } => ("at_relative_distance", body, reference_body),
                EventTriggerConfig::AtRelativeSpeed {
                    body,
                    reference_body,
                    ..
                } => ("at_relative_speed", body, reference_body),
                _ => continue,
            };
            if self.vehicle.kind != "rigid_body" {
                return Err(ScenarioError::IncompatibleAssemblyEntry {
                    field: format!("mission.events[{event_index}].trigger.kind"),
                    reason: format!("{kind} requires vehicle.kind = \"rigid_body\""),
                });
            }
            if self.multi_body.is_none() {
                return Err(ScenarioError::InconsistentSection {
                    field_a: format!("mission.events[{event_index}].trigger.kind"),
                    value_a: kind.to_owned(),
                    field_b: "multi_body".to_owned(),
                    value_b: "missing".to_owned(),
                });
            }
            if !body_ids.contains(body.as_str()) {
                return Err(ScenarioError::UnknownBodyReference {
                    field: format!("mission.events[{event_index}].trigger.body"),
                    value: body.clone(),
                });
            }
            if let Some(reference_body) = reference_body {
                if reference_body == body {
                    return Err(ScenarioError::InconsistentSection {
                        field_a: format!("mission.events[{event_index}].trigger.body"),
                        value_a: body.clone(),
                        field_b: format!("mission.events[{event_index}].trigger.reference_body"),
                        value_b: reference_body.clone(),
                    });
                }
                if !body_ids.contains(reference_body.as_str()) {
                    return Err(ScenarioError::UnknownBodyReference {
                        field: format!("mission.events[{event_index}].trigger.reference_body"),
                        value: reference_body.clone(),
                    });
                }
            }
        }
        Ok(())
    }

    fn validate_initial_multi_body_references(&self) -> Result<(), ScenarioError> {
        let Some(multi_body) = &self.multi_body else {
            return Ok(());
        };
        if multi_body.initial_lanes.is_empty() {
            return Ok(());
        }
        if self.vehicle.kind != "rigid_body" {
            return Err(ScenarioError::IncompatibleAssemblyEntry {
                field: "multi_body.initial_lane".to_owned(),
                reason: "initial multi-body lanes require vehicle.kind = \"rigid_body\"".to_owned(),
            });
        }

        let body_masses = assembly_body_mass_lookup(&self.vehicle.assembly);
        let Some(primary_body_id) = multi_body.primary_body_id.as_ref() else {
            return Err(ScenarioError::MissingRequiredField {
                field: "multi_body.primary_body_id".to_owned(),
                role: ModelRole::Vehicle,
                name: "multi_body initial lanes".to_owned(),
            });
        };
        if !body_masses.contains_key(primary_body_id.as_str()) {
            return Err(ScenarioError::UnknownBodyReference {
                field: "multi_body.primary_body_id".to_owned(),
                value: primary_body_id.clone(),
            });
        }
        let mut initial_lane_bodies = BTreeSet::new();
        for (index, lane) in multi_body.initial_lanes.iter().enumerate() {
            if lane.body_id == *primary_body_id {
                return Err(ScenarioError::InconsistentSection {
                    field_a: format!("multi_body.initial_lane[{index}].body_id"),
                    value_a: lane.body_id.clone(),
                    field_b: "multi_body.primary_body_id".to_owned(),
                    value_b: primary_body_id.clone(),
                });
            }
            if !body_masses.contains_key(lane.body_id.as_str()) {
                return Err(ScenarioError::UnknownBodyReference {
                    field: format!("multi_body.initial_lane[{index}].body_id"),
                    value: lane.body_id.clone(),
                });
            }
            initial_lane_bodies.insert(lane.body_id.as_str());
        }
        for (index, separation) in multi_body.separations.iter().enumerate() {
            if separation.upper_body_id != *primary_body_id {
                return Err(ScenarioError::InconsistentSection {
                    field_a: format!("multi_body.separation[{index}].upper_body_id"),
                    value_a: separation.upper_body_id.clone(),
                    field_b: "multi_body.primary_body_id".to_owned(),
                    value_b: primary_body_id.clone(),
                });
            }
            if initial_lane_bodies.contains(separation.lower_body_id.as_str()) {
                return Err(ScenarioError::InconsistentSection {
                    field_a: format!("multi_body.separation[{index}].lower_body_id"),
                    value_a: separation.lower_body_id.clone(),
                    field_b: "multi_body.initial_lane.body_id".to_owned(),
                    value_b: separation.lower_body_id.clone(),
                });
            }
        }
        validate_multi_body_resource_ownership(self, &body_masses)?;
        Ok(())
    }

    fn validate_effector_references(&self) -> Result<(), ScenarioError> {
        let Some(mission) = &self.mission else {
            return Ok(());
        };
        let declared: BTreeSet<&str> = self
            .vehicle
            .assembly
            .effectors
            .iter()
            .map(|e| e.id.as_str())
            .collect();

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
        for (state_index, state) in mission.states.iter().enumerate() {
            for (effector_index, id) in state.allowed_effectors.iter().enumerate() {
                if !declared.contains(id.as_str()) {
                    return Err(ScenarioError::UnknownEffectorReference {
                        field: format!(
                            "mission.states[{state_index}].allowed_effectors[{effector_index}]"
                        ),
                        id: id.clone(),
                    });
                }
            }
        }
        for (event_index, event) in mission.events.iter().enumerate() {
            if let ScenarioActionConfig::EffectorOverride { id, .. } = &event.action
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

    fn validate_multi_body_attitude_target_references(&self) -> Result<(), ScenarioError> {
        let Some(multi_body) = &self.multi_body else {
            return Ok(());
        };
        if multi_body.attitude_targets.is_empty() {
            return Ok(());
        }
        if self.vehicle.kind != "rigid_body" {
            return Err(ScenarioError::IncompatibleAssemblyEntry {
                field: "multi_body.attitude_target".to_owned(),
                reason: "multi-body attitude targets require vehicle.kind = \"rigid_body\""
                    .to_owned(),
            });
        }
        let body_ids: BTreeSet<&str> = self
            .vehicle
            .assembly
            .bodies
            .iter()
            .map(|body| body.id.as_str())
            .collect();
        let effectors: BTreeMap<&str, &EffectorConfig> = self
            .vehicle
            .assembly
            .effectors
            .iter()
            .map(|effector| (effector.id.as_str(), effector))
            .collect();
        for (index, target) in multi_body.attitude_targets.iter().enumerate() {
            if !body_ids.contains(target.body_id.as_str()) {
                return Err(ScenarioError::UnknownBodyReference {
                    field: format!("multi_body.attitude_target[{index}].body_id"),
                    value: target.body_id.clone(),
                });
            }
            validate_attitude_target_effector(
                index,
                &target.body_id,
                "roll_effector",
                target.roll_effector.as_ref(),
                TorqueAxis::Roll,
                &effectors,
            )?;
            validate_attitude_target_effector(
                index,
                &target.body_id,
                "pitch_effector",
                target.pitch_effector.as_ref(),
                TorqueAxis::Pitch,
                &effectors,
            )?;
            validate_attitude_target_effector(
                index,
                &target.body_id,
                "yaw_effector",
                target.yaw_effector.as_ref(),
                TorqueAxis::Yaw,
                &effectors,
            )?;
        }
        Ok(())
    }

    fn validate_force_dependencies(&self) -> Result<(), ScenarioError> {
        // `forces` is auto-synthesised at parse time
        // from the assembly when absent, so this hook always sees a
        // populated model list. The `unwrap_or` keeps the helper
        // total-defined for future call paths that bypass parser
        // synthesis.
        let force_models = self.force_model_universe();
        if force_models.iter().any(|model| model == "aero") && self.aero.is_none() {
            return Err(ScenarioError::MissingRequiredField {
                field: "aero".to_owned(),
                role: ModelRole::Force,
                name: "aero".to_owned(),
            });
        }
        let has_thrust_force = force_models.iter().any(|model| model == "thrust");
        let has_motor = self
            .propulsion
            .as_ref()
            .and_then(|propulsion| propulsion.motor.as_ref())
            .is_some();
        // `forces.models = ["thrust"]` is satisfied by
        // EITHER a `[propulsion.motor]` block OR a non-empty
        // `[[vehicle.assembly.engines]]` block (cluster path).
        // Ambiguous co-declaration is rejected separately by
        // `validate_propulsion_unambiguous`.
        let engine_count = self.vehicle.assembly.engines.len();
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

    fn validate_force_phase_overrides(&self) -> Result<(), ScenarioError> {
        let Some(forces) = &self.forces else {
            return Ok(());
        };
        if forces.phase_override.is_empty() {
            return Ok(());
        }
        let Some(mission) = &self.mission else {
            return Err(ScenarioError::MissingRequiredField {
                field: "mission".to_owned(),
                role: ModelRole::Force,
                name: "forces.phase_override".to_owned(),
            });
        };
        let declared: BTreeSet<String> = if mission.states.is_empty() {
            mission
                .phases
                .iter()
                .map(|phase| phase.id.clone())
                .collect()
        } else {
            mission
                .states
                .iter()
                .map(|state| state.id.clone())
                .collect()
        };
        for override_config in &forces.phase_override {
            let phase = override_config.phase.as_str();
            let bare = phase
                .strip_prefix("mission.phases.")
                .or_else(|| phase.strip_prefix("mission.states."))
                .unwrap_or(phase);
            if !declared.contains(phase) && !declared.contains(bare) {
                return Err(ScenarioError::MissionGraph {
                    reason: format!(
                        "forces.phase_override phase `{}` does not match any declared mission phase/state",
                        override_config.phase
                    ),
                });
            }
        }
        Ok(())
    }

    fn validate_header(&self) -> Result<(), ScenarioError> {
        if !SUPPORTED_SCENARIO_VERSIONS.contains(&self.openbmp.scenario) {
            return Err(ScenarioError::UnsupportedSchemaVersion {
                found: self.openbmp.scenario,
                supported: SUPPORTED_SCENARIO_VERSIONS,
            });
        }
        Ok(())
    }
}

fn validate_attitude_target_effector(
    controller_index: usize,
    body_id: &str,
    field_name: &str,
    effector_id: Option<&String>,
    expected_axis: TorqueAxis,
    effectors: &BTreeMap<&str, &EffectorConfig>,
) -> Result<(), ScenarioError> {
    let Some(effector_id) = effector_id else {
        return Ok(());
    };
    let field = format!("multi_body.attitude_target[{controller_index}].{field_name}");
    let Some(effector) = effectors.get(effector_id.as_str()) else {
        return Err(ScenarioError::UnknownEffectorReference {
            field,
            id: effector_id.clone(),
        });
    };
    if effector.mounted_to.as_deref() != Some(body_id) {
        return Err(ScenarioError::IncompatibleAssemblyEntry {
            field,
            reason: format!(
                "effector `{effector_id}` must be mounted_to the attitude target body `{body_id}`"
            ),
        });
    }
    match effector.kind {
        EffectorKindConfig::DirectTorque { axis, .. } if axis == expected_axis => Ok(()),
        EffectorKindConfig::DirectTorque { axis, .. } => {
            Err(ScenarioError::IncompatibleAssemblyEntry {
                field,
                reason: format!(
                    "effector `{effector_id}` is a direct_torque {:?} axis, expected {:?}",
                    axis, expected_axis
                ),
            })
        }
        EffectorKindConfig::LinearActuator { .. } => {
            Err(ScenarioError::IncompatibleAssemblyEntry {
                field,
                reason: format!("effector `{effector_id}` must be kind = \"direct_torque\""),
            })
        }
    }
}

fn mission_declares_powered_ascent(mission: &MissionConfig) -> bool {
    mission
        .phases
        .iter()
        .any(|phase| is_powered_ascent_id(&phase.id))
        || mission
            .states
            .iter()
            .any(|state| is_powered_ascent_id(&state.id))
}

fn mission_declares_coast_or_ballistic_descent(mission: &MissionConfig) -> bool {
    mission
        .phases
        .iter()
        .any(|phase| is_coast_or_ballistic_descent_id(&phase.id))
        || mission
            .states
            .iter()
            .any(|state| is_coast_or_ballistic_descent_id(&state.id))
}

fn mission_declares_phase_or_state(mission: &MissionConfig, terminal_id: &str) -> bool {
    mission
        .phases
        .iter()
        .any(|phase| terminal_mission_id(&phase.id) == terminal_id)
        || mission
            .states
            .iter()
            .any(|state| terminal_mission_id(&state.id) == terminal_id)
}

fn mission_has_descending_entry_handoff(
    mission: &MissionConfig,
    phase_id: &str,
    altitude_m: f64,
) -> bool {
    mission.events.iter().any(|event| {
        matches!(
            event.trigger,
            EventTriggerConfig::AtAltitudeDescending { altitude_m: event_altitude_m }
                if (event_altitude_m - altitude_m).abs() <= ENTRY_ALTITUDE_MATCH_TOLERANCE_M
        ) && matches!(
            &event.action,
            ScenarioActionConfig::EnterPhase { phase }
                if terminal_mission_id(phase) == phase_id
        )
    })
}

fn validate_entry_atmosphere_envelope(
    atmosphere: &str,
    entry_interface_altitude_m: f64,
) -> Result<(), ScenarioError> {
    match atmosphere {
        "piecewise_exponential" | "nrlmsise00" | "nrlmsis2_compat" => Ok(()),
        "us_standard_1976" if entry_interface_altitude_m <= USSA76_ENTRY_INTERFACE_CEILING_M => {
            Ok(())
        }
        "us_standard_1976" => Err(ScenarioError::InconsistentSection {
            field_a: "entry_profile.entry_interface_altitude_m".to_owned(),
            value_a: entry_interface_altitude_m.to_string(),
            field_b: "environment.atmosphere".to_owned(),
            value_b: "us_standard_1976 envelope ceiling is 86000 m".to_owned(),
        }),
        other => Err(ScenarioError::InconsistentSection {
            field_a: "entry_profile".to_owned(),
            value_a: "declared".to_owned(),
            field_b: "environment.atmosphere".to_owned(),
            value_b: format!("{other} is not wired for entry profiles"),
        }),
    }
}

fn fc_gain_schedule_declares_powered_ascent(
    gain_schedule: &BTreeMap<String, FcGainsConfig>,
) -> bool {
    gain_schedule.keys().any(|key| is_powered_ascent_id(key))
}

fn is_powered_ascent_id(id: &str) -> bool {
    matches!(
        id,
        "powered_ascent" | "mission.phases.powered_ascent" | "mission.states.powered_ascent"
    )
}

fn is_coast_or_ballistic_descent_id(id: &str) -> bool {
    matches!(terminal_mission_id(id), "coast" | "ballistic_descent")
}

fn terminal_mission_id(id: &str) -> &str {
    id.rsplit('.').next().unwrap_or(id)
}

#[derive(Clone, Debug)]
struct JettisonStageAction<'a> {
    event_index: usize,
    event_id: &'a str,
    body: &'a str,
}

fn collect_jettison_stage_actions(mission: &MissionConfig) -> Vec<JettisonStageAction<'_>> {
    let mut actions = Vec::new();
    for (event_index, event) in mission.events.iter().enumerate() {
        match &event.action {
            ScenarioActionConfig::JettisonStage { body } => {
                actions.push(JettisonStageAction {
                    event_index,
                    event_id: event.id.as_str(),
                    body: body.as_str(),
                });
            }
            ScenarioActionConfig::JettisonBodies { bodies } => {
                for body in bodies {
                    actions.push(JettisonStageAction {
                        event_index,
                        event_id: event.id.as_str(),
                        body: body.as_str(),
                    });
                }
            }
            _ => {}
        }
    }
    actions
}

fn assembly_body_mass_lookup(assembly: &AssemblyConfig) -> BTreeMap<&str, f64> {
    assembly
        .bodies
        .iter()
        .map(|body| (body.id.as_str(), body.dry_mass_kg))
        .collect()
}

fn validate_multi_body_resource_ownership(
    document: &ScenarioDocument,
    body_masses: &BTreeMap<&str, f64>,
) -> Result<(), ScenarioError> {
    let require_owner = |field: String, value: &Option<String>| -> Result<(), ScenarioError> {
        let Some(owner) = value else {
            return Err(ScenarioError::IncompatibleAssemblyEntry {
                field,
                reason: "multi_body force-stack ownership requires an explicit `mounted_to` body"
                    .to_owned(),
            });
        };
        if !body_masses.contains_key(owner.as_str()) {
            return Err(ScenarioError::UnknownBodyReference {
                field,
                value: owner.clone(),
            });
        }
        Ok(())
    };

    if let Some(aero) = &document.aero
        && document.force_models().iter().any(|name| name == "aero")
    {
        require_owner("aero.mounted_to".to_owned(), &aero.mounted_to)?;
    }
    if let Some(motor) = document
        .propulsion
        .as_ref()
        .and_then(|propulsion| propulsion.motor.as_ref())
    {
        require_owner("propulsion.motor.mounted_to".to_owned(), &motor.mounted_to)?;
    }
    for (index, effector) in document.vehicle.assembly.effectors.iter().enumerate() {
        require_owner(
            format!("vehicle.assembly.effectors[{index}].mounted_to"),
            &effector.mounted_to,
        )?;
    }
    for (index, engine) in document.vehicle.assembly.engines.iter().enumerate() {
        require_owner(
            format!("vehicle.assembly.engines[{index}].mounted_to"),
            &engine.mounted_to,
        )?;
    }
    for (index, recovery) in document.vehicle.assembly.recovery.iter().enumerate() {
        require_owner(
            format!("vehicle.assembly.recovery[{index}].mounted_to"),
            &recovery.mounted_to,
        )?;
    }
    Ok(())
}

fn validate_stage_separation_body_references(
    jettisons: &[JettisonStageAction<'_>],
    separations: &[MultiBodySeparationConfig],
    body_masses: &BTreeMap<&str, f64>,
) -> Result<(), ScenarioError> {
    for action in jettisons {
        if !body_masses.contains_key(action.body) {
            return Err(ScenarioError::UnknownBodyReference {
                field: format!("mission.events[{}].action.body", action.event_index),
                value: action.body.to_owned(),
            });
        }
    }
    for (index, separation) in separations.iter().enumerate() {
        if !body_masses.contains_key(separation.upper_body_id.as_str()) {
            return Err(ScenarioError::UnknownBodyReference {
                field: format!("multi_body.separation[{index}].upper_body_id"),
                value: separation.upper_body_id.clone(),
            });
        }
        if !body_masses.contains_key(separation.lower_body_id.as_str()) {
            return Err(ScenarioError::UnknownBodyReference {
                field: format!("multi_body.separation[{index}].lower_body_id"),
                value: separation.lower_body_id.clone(),
            });
        }
    }
    Ok(())
}

fn validate_stage_separation_uniqueness(
    jettisons: &[JettisonStageAction<'_>],
    separations: &[MultiBodySeparationConfig],
) -> Result<(), ScenarioError> {
    let mut seen_jettisoned_bodies = BTreeSet::new();
    for action in jettisons {
        if !seen_jettisoned_bodies.insert(action.body) {
            return Err(ScenarioError::DuplicateValue {
                field: format!("mission.events[{}].action.body", action.event_index),
                value: action.body.to_owned(),
            });
        }
    }
    let mut seen_separation_bodies = BTreeSet::new();
    for (index, separation) in separations.iter().enumerate() {
        if !seen_separation_bodies.insert(separation.lower_body_id.as_str()) {
            return Err(ScenarioError::DuplicateValue {
                field: format!("multi_body.separation[{index}].lower_body_id"),
                value: separation.lower_body_id.clone(),
            });
        }
    }
    Ok(())
}

fn validate_stage_separation_symmetry_and_momentum(
    jettisons: &[JettisonStageAction<'_>],
    separations: &[MultiBodySeparationConfig],
    body_masses: &BTreeMap<&str, f64>,
) -> Result<(), ScenarioError> {
    for action in jettisons {
        let matching = separations.iter().any(|separation| {
            separation.event_id == action.event_id && separation.lower_body_id == action.body
        });
        if !matching {
            return Err(ScenarioError::InconsistentSection {
                field_a: format!("mission.events[{}].action.body", action.event_index),
                value_a: format!("{} at {}", action.body, action.event_id),
                field_b: "multi_body.separation".to_owned(),
                value_b: "no matching event_id/lower_body_id".to_owned(),
            });
        }
    }
    for (index, separation) in separations.iter().enumerate() {
        let matching = jettisons.iter().any(|action| {
            action.event_id == separation.event_id && action.body == separation.lower_body_id
        });
        if !matching {
            return Err(ScenarioError::InconsistentSection {
                field_a: format!("multi_body.separation[{index}]"),
                value_a: format!("{} at {}", separation.lower_body_id, separation.event_id),
                field_b: "mission.events.action.kind".to_owned(),
                value_b: "no matching jettison_stage".to_owned(),
            });
        }
        if separation.conserve_momentum {
            validate_separation_momentum(index, separation, body_masses)?;
        }
    }
    Ok(())
}

fn validate_separation_momentum(
    index: usize,
    separation: &MultiBodySeparationConfig,
    body_masses: &BTreeMap<&str, f64>,
) -> Result<(), ScenarioError> {
    let upper_mass = body_masses[separation.upper_body_id.as_str()];
    let lower_mass = body_masses[separation.lower_body_id.as_str()];
    let upper_dv = separation.upper_delta_v_body_m_s.unwrap_or([0.0, 0.0, 0.0]);
    let lower_dv = separation.lower_delta_v_body_m_s.unwrap_or([0.0, 0.0, 0.0]);
    let residual = [
        upper_mass * upper_dv[0] + lower_mass * lower_dv[0],
        upper_mass * upper_dv[1] + lower_mass * lower_dv[1],
        upper_mass * upper_dv[2] + lower_mass * lower_dv[2],
    ];
    let mut squared = 0.0_f64;
    squared += residual[0] * residual[0];
    squared += residual[1] * residual[1];
    squared += residual[2] * residual[2];
    let norm = squared.sqrt();
    if !norm.is_finite() || norm > STAGE_SEPARATION_MOMENTUM_TOLERANCE_KG_M_S {
        return Err(ScenarioError::SeparationMomentumMismatch {
            field: format!("multi_body.separation[{index}]"),
            residual_kg_m_s: norm,
            tolerance_kg_m_s: STAGE_SEPARATION_MOMENTUM_TOLERANCE_KG_M_S,
        });
    }
    Ok(())
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
    /// Mandatory declarative vehicle composition tree.
    ///
    /// The runner builds an `openbmp_vehicle::Assembly` from
    /// the declared bodies and resolves it into the kernel's flat
    /// model lists. Per-body dry mass lives on
    /// `[[vehicle.assembly.bodies]].dry_mass_kg`; per-body
    /// inertia (rigid-body only) lives on
    /// `[[vehicle.assembly.bodies]].dry_inertia_body_kg_m2`.
    pub assembly: AssemblyConfig,
    /// Optional first lateral structural bending mode (rigid-body only).
    /// When present, the FC rate gyro picks up the bending slope rate (so
    /// the autopilot senses flex and `[fc.autopilot_params].gyro_notch` can
    /// gain-stabilise it). Absent = perfectly rigid (the default;
    /// byte-identical).
    #[serde(default)]
    pub bending: Option<BendingConfig>,
}

/// First lateral structural bending mode (`[vehicle.bending]`).
#[derive(Copy, Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct BendingConfig {
    /// Modal natural frequency (Hz), strictly positive. Set the autopilot
    /// gyro notch near this to gain-stabilise the mode.
    pub frequency_hz: f64,
    /// Modal damping ratio, non-negative.
    pub damping_ratio: f64,
    /// Generalised modal mass (kg), strictly positive.
    pub modal_mass_kg: f64,
    /// Mode-shape slope at the engine station (forcing lever).
    pub slope_at_engine: f64,
    /// Mode-shape slope at the rate-gyro station (sensor pickup lever).
    pub slope_at_gyro: f64,
}

impl BendingConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_finite("vehicle.bending.frequency_hz", self.frequency_hz)?;
        require_positive("vehicle.bending.frequency_hz", self.frequency_hz)?;
        require_finite("vehicle.bending.modal_mass_kg", self.modal_mass_kg)?;
        require_positive("vehicle.bending.modal_mass_kg", self.modal_mass_kg)?;
        require_finite("vehicle.bending.damping_ratio", self.damping_ratio)?;
        if self.damping_ratio < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: "vehicle.bending.damping_ratio".to_owned(),
                value: self.damping_ratio,
                rule: "must be non-negative",
            });
        }
        require_finite("vehicle.bending.slope_at_engine", self.slope_at_engine)?;
        require_finite("vehicle.bending.slope_at_gyro", self.slope_at_gyro)?;
        Ok(())
    }
}

impl VehicleConfig {
    fn validate(&self, registry: &ModelRegistry, dt_s: f64) -> Result<(), ScenarioError> {
        let descriptor = registry.resolve(ModelRole::Vehicle, &self.kind)?;
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
            }
        }
        self.assembly.validate(descriptor.name.as_str(), dt_s)?;
        if let Some(bending) = &self.bending {
            if descriptor.name.as_str() != "rigid_body" {
                return Err(ScenarioError::UnexpectedField {
                    field: "vehicle.bending".to_owned(),
                    role: ModelRole::Vehicle,
                    name: descriptor.name.clone(),
                });
            }
            bending.validate()?;
        }
        Ok(())
    }
}

fn validate_inertia_tensor(
    field_prefix: &str,
    inertia: &[[f64; 3]; 3],
) -> Result<(), ScenarioError> {
    // Finiteness, symmetry (within 1e-9 tolerance), and positive
    // diagonal entries. Full positive-definite + triangle-inequality
    // validation lives in `MassProperties::require_valid` at kernel
    // construction.
    for (i, row) in inertia.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            if !value.is_finite() {
                return Err(ScenarioError::InvalidNumber {
                    field: format!("{field_prefix}[{i}][{j}]"),
                    value: *value,
                    rule: "must be finite",
                });
            }
        }
        if row[i] <= 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{field_prefix}[{i}][{i}]"),
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
                field: format!("{field_prefix}[{i}][{j}]"),
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
    /// Central gravity used when `gravity = "third_body"`.
    pub gravity_base: Option<String>,
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
    /// Perturbing celestial bodies used when
    /// `gravity = "third_body"`.
    #[serde(default)]
    pub third_bodies: Vec<String>,
    /// Celestial ephemeris provider used when
    /// `gravity = "third_body"`.
    pub ephemeris: Option<String>,
    /// Binary SPK/BSP ephemeris kernel path used when
    /// `ephemeris = "spk"`.
    pub ephemeris_file: Option<PathBuf>,
    /// Optional SHA-256 pin for [`Self::ephemeris_file`].
    pub ephemeris_file_sha256: Option<String>,
    /// Ordered binary SPK/BSP ephemeris kernel list used when
    /// `ephemeris = "spk"`. Later files take precedence over earlier
    /// files for overlapping SPK segments.
    #[serde(default)]
    pub ephemeris_files: Vec<PathBuf>,
    /// Optional SHA-256 pins for [`Self::ephemeris_files`]. When
    /// present, the list length must match `ephemeris_files`.
    #[serde(default)]
    pub ephemeris_files_sha256: Vec<String>,
    /// NAIF `KPL/MK` meta-kernel path used when `ephemeris = "spk"`.
    /// Referenced kernels are resolved by the scenario loader.
    pub ephemeris_meta_kernel: Option<PathBuf>,
    /// Optional SHA-256 pin for [`Self::ephemeris_meta_kernel`].
    pub ephemeris_meta_kernel_sha256: Option<String>,
    /// Optional SHA-256 pins for kernels referenced by
    /// [`Self::ephemeris_meta_kernel`], in `KERNELS_TO_LOAD` order.
    #[serde(default)]
    pub ephemeris_meta_kernel_files_sha256: Vec<String>,
    /// Atmosphere model name.
    pub atmosphere: String,
    /// Wind model name.
    pub wind: String,
    /// Optional magnetic-field model name.
    pub magnetic: Option<String>,
}

impl EnvironmentConfig {
    #[allow(clippy::too_many_lines)] // the egm2008 arm adds length
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
            "egm2008" => {
                // Zonal-only EGM2008 (degrees 2-6). Pinned
                // to WGS84 µ, R_e and the Pavlis et al. 2012 J_n
                // tables; the scenario block carries no per-field
                // overrides on purpose to keep the determinism contract
                // tight. Reject any leftover gravity-config keys.
                if self.gravity_m_s2.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "environment.gravity_m_s2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "egm2008".to_owned(),
                    });
                }
                if self.mu_m3_s2.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "environment.mu_m3_s2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "egm2008".to_owned(),
                    });
                }
                if self.r_e_m.is_some() || self.j2.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "environment.r_e_m / environment.j2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "egm2008".to_owned(),
                    });
                }
            }
            "third_body" => {
                let base = self.gravity_base.as_deref().ok_or_else(|| {
                    ScenarioError::MissingRequiredField {
                        field: "environment.gravity_base".to_owned(),
                        role: ModelRole::Gravity,
                        name: "third_body".to_owned(),
                    }
                })?;
                require_supported(
                    "environment.gravity_base",
                    base,
                    &["point_mass", "j2", "egm2008"],
                )?;
                require_non_empty_list("environment.third_bodies", &self.third_bodies)?;
                require_unique("environment.third_bodies", &self.third_bodies)?;
                for (index, body) in self.third_bodies.iter().enumerate() {
                    require_supported(
                        &format!("environment.third_bodies[{index}]"),
                        body,
                        &["sun", "moon"],
                    )?;
                }
                if let Some(ephemeris) = &self.ephemeris {
                    require_supported(
                        "environment.ephemeris",
                        ephemeris,
                        &["low_precision_sun_moon", "spk"],
                    )?;
                }
                match self.ephemeris.as_deref() {
                    Some("spk") => {
                        let has_single = self.ephemeris_file.is_some();
                        let has_list = !self.ephemeris_files.is_empty();
                        let has_meta = self.ephemeris_meta_kernel.is_some();
                        if !has_single && !has_list && !has_meta {
                            return Err(ScenarioError::MissingRequiredField {
                                field:
                                    "environment.ephemeris_file, environment.ephemeris_files, or environment.ephemeris_meta_kernel"
                                        .to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if has_single && has_list {
                            return Err(ScenarioError::InconsistentSection {
                                field_a: "environment.ephemeris_file".to_owned(),
                                value_a: "declared".to_owned(),
                                field_b: "environment.ephemeris_files".to_owned(),
                                value_b: "declared".to_owned(),
                            });
                        }
                        if has_single && has_meta {
                            return Err(ScenarioError::InconsistentSection {
                                field_a: "environment.ephemeris_file".to_owned(),
                                value_a: "declared".to_owned(),
                                field_b: "environment.ephemeris_meta_kernel".to_owned(),
                                value_b: "declared".to_owned(),
                            });
                        }
                        if has_list && has_meta {
                            return Err(ScenarioError::InconsistentSection {
                                field_a: "environment.ephemeris_files".to_owned(),
                                value_a: "declared".to_owned(),
                                field_b: "environment.ephemeris_meta_kernel".to_owned(),
                                value_b: "declared".to_owned(),
                            });
                        }
                        if has_single && !self.ephemeris_files_sha256.is_empty() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_files_sha256".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if has_list && self.ephemeris_file_sha256.is_some() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_file_sha256".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if !has_meta && self.ephemeris_meta_kernel_sha256.is_some() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_meta_kernel_sha256".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if !has_meta && !self.ephemeris_meta_kernel_files_sha256.is_empty() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_meta_kernel_files_sha256".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if has_meta && self.ephemeris_file_sha256.is_some() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_file_sha256".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if has_meta && !self.ephemeris_files_sha256.is_empty() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_files_sha256".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if has_list
                            && !self.ephemeris_files_sha256.is_empty()
                            && self.ephemeris_files_sha256.len() != self.ephemeris_files.len()
                        {
                            return Err(ScenarioError::InconsistentSection {
                                field_a: "environment.ephemeris_files".to_owned(),
                                value_a: self.ephemeris_files.len().to_string(),
                                field_b: "environment.ephemeris_files_sha256".to_owned(),
                                value_b: self.ephemeris_files_sha256.len().to_string(),
                            });
                        }
                    }
                    _ => {
                        if self.ephemeris_file.is_some() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_file".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if self.ephemeris_file_sha256.is_some() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_file_sha256".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if !self.ephemeris_files.is_empty() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_files".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if !self.ephemeris_files_sha256.is_empty() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_files_sha256".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if self.ephemeris_meta_kernel.is_some() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_meta_kernel".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if self.ephemeris_meta_kernel_sha256.is_some() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_meta_kernel_sha256".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                        if !self.ephemeris_meta_kernel_files_sha256.is_empty() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.ephemeris_meta_kernel_files_sha256".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body".to_owned(),
                            });
                        }
                    }
                }
                if self.gravity_m_s2.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "environment.gravity_m_s2".to_owned(),
                        role: ModelRole::Gravity,
                        name: "third_body".to_owned(),
                    });
                }
                match base {
                    "point_mass" => {
                        let mu =
                            self.mu_m3_s2
                                .ok_or_else(|| ScenarioError::MissingRequiredField {
                                    field: "environment.mu_m3_s2".to_owned(),
                                    role: ModelRole::Gravity,
                                    name: "third_body".to_owned(),
                                })?;
                        require_positive("environment.mu_m3_s2", mu)?;
                        if self.r_e_m.is_some() || self.j2.is_some() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.r_e_m / environment.j2".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body point_mass base".to_owned(),
                            });
                        }
                    }
                    "j2" => {
                        let mu =
                            self.mu_m3_s2
                                .ok_or_else(|| ScenarioError::MissingRequiredField {
                                    field: "environment.mu_m3_s2".to_owned(),
                                    role: ModelRole::Gravity,
                                    name: "third_body".to_owned(),
                                })?;
                        require_positive("environment.mu_m3_s2", mu)?;
                        let r_e =
                            self.r_e_m
                                .ok_or_else(|| ScenarioError::MissingRequiredField {
                                    field: "environment.r_e_m".to_owned(),
                                    role: ModelRole::Gravity,
                                    name: "third_body".to_owned(),
                                })?;
                        require_positive("environment.r_e_m", r_e)?;
                        if let Some(j2) = self.j2 {
                            require_finite("environment.j2", j2)?;
                        }
                    }
                    "egm2008" => {
                        if self.mu_m3_s2.is_some() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.mu_m3_s2".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body egm2008 base".to_owned(),
                            });
                        }
                        if self.r_e_m.is_some() || self.j2.is_some() {
                            return Err(ScenarioError::UnexpectedField {
                                field: "environment.r_e_m / environment.j2".to_owned(),
                                role: ModelRole::Gravity,
                                name: "third_body egm2008 base".to_owned(),
                            });
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
        if self.gravity != "third_body" {
            if self.gravity_base.is_some() {
                return Err(ScenarioError::UnexpectedField {
                    field: "environment.gravity_base".to_owned(),
                    role: ModelRole::Gravity,
                    name: self.gravity.clone(),
                });
            }
            if !self.third_bodies.is_empty() {
                return Err(ScenarioError::UnexpectedField {
                    field: "environment.third_bodies".to_owned(),
                    role: ModelRole::Gravity,
                    name: self.gravity.clone(),
                });
            }
            if self.ephemeris.is_some() {
                return Err(ScenarioError::UnexpectedField {
                    field: "environment.ephemeris".to_owned(),
                    role: ModelRole::Gravity,
                    name: self.gravity.clone(),
                });
            }
            if self.ephemeris_file.is_some() {
                return Err(ScenarioError::UnexpectedField {
                    field: "environment.ephemeris_file".to_owned(),
                    role: ModelRole::Gravity,
                    name: self.gravity.clone(),
                });
            }
            if self.ephemeris_file_sha256.is_some() {
                return Err(ScenarioError::UnexpectedField {
                    field: "environment.ephemeris_file_sha256".to_owned(),
                    role: ModelRole::Gravity,
                    name: self.gravity.clone(),
                });
            }
            if !self.ephemeris_files.is_empty() {
                return Err(ScenarioError::UnexpectedField {
                    field: "environment.ephemeris_files".to_owned(),
                    role: ModelRole::Gravity,
                    name: self.gravity.clone(),
                });
            }
            if !self.ephemeris_files_sha256.is_empty() {
                return Err(ScenarioError::UnexpectedField {
                    field: "environment.ephemeris_files_sha256".to_owned(),
                    role: ModelRole::Gravity,
                    name: self.gravity.clone(),
                });
            }
            if self.ephemeris_meta_kernel.is_some() {
                return Err(ScenarioError::UnexpectedField {
                    field: "environment.ephemeris_meta_kernel".to_owned(),
                    role: ModelRole::Gravity,
                    name: self.gravity.clone(),
                });
            }
            if self.ephemeris_meta_kernel_sha256.is_some() {
                return Err(ScenarioError::UnexpectedField {
                    field: "environment.ephemeris_meta_kernel_sha256".to_owned(),
                    role: ModelRole::Gravity,
                    name: self.gravity.clone(),
                });
            }
            if !self.ephemeris_meta_kernel_files_sha256.is_empty() {
                return Err(ScenarioError::UnexpectedField {
                    field: "environment.ephemeris_meta_kernel_files_sha256".to_owned(),
                    role: ModelRole::Gravity,
                    name: self.gravity.clone(),
                });
            }
        }
        registry.resolve(ModelRole::Atmosphere, &self.atmosphere)?;
        registry.resolve(ModelRole::Wind, &self.wind)?;
        if let Some(magnetic) = &self.magnetic {
            require_supported(
                "environment.magnetic",
                magnetic,
                &["none", "earth_dipole", "wmm_2025"],
            )?;
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
    /// Optional per-mission-phase model-selection overrides.
    #[serde(default)]
    pub phase_override: Vec<ForcePhaseOverrideConfig>,
}

impl ForcesConfig {
    fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        require_non_empty_list("forces.models", &self.models)?;
        require_unique("forces.models", &self.models)?;
        for model in &self.models {
            validate_force_stack_model(registry, "forces.models", model)?;
        }
        let mut phases = BTreeSet::new();
        for (index, override_config) in self.phase_override.iter().enumerate() {
            override_config.validate(index, registry)?;
            if !phases.insert(override_config.phase.clone()) {
                return Err(ScenarioError::DuplicateValue {
                    field: "forces.phase_override.phase".to_owned(),
                    value: override_config.phase.clone(),
                });
            }
        }
        Ok(())
    }
}

/// Per-phase force-stack model selection.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ForcePhaseOverrideConfig {
    /// Mission phase id. Bare ids use the `mission.phases.<id>`
    /// namespace; canonical `mission.phases.*` and `mission.states.*`
    /// paths are also accepted by the runner.
    pub phase: String,
    /// Active model names during this phase.
    pub models: Vec<String>,
}

impl ForcePhaseOverrideConfig {
    fn validate(&self, index: usize, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        require_non_empty(
            &format!("forces.phase_override[{index}].phase"),
            &self.phase,
        )?;
        require_non_empty_list(
            &format!("forces.phase_override[{index}].models"),
            &self.models,
        )?;
        require_unique(
            &format!("forces.phase_override[{index}].models"),
            &self.models,
        )?;
        for model in &self.models {
            validate_force_stack_model(
                registry,
                &format!("forces.phase_override[{index}].models"),
                model,
            )?;
        }
        Ok(())
    }
}

fn validate_force_stack_model(
    registry: &ModelRegistry,
    field: &str,
    model: &str,
) -> Result<(), ScenarioError> {
    if model == "aerothermal_diagnostics" {
        return Ok(());
    }
    registry
        .resolve(ModelRole::Force, model)
        .map(|_| ())
        .map_err(|err| match err {
            ScenarioError::UnknownModel { .. } => ScenarioError::UnknownModel {
                role: ModelRole::Force,
                name: format!("{field}:{model}"),
            },
            other => other,
        })
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

/// Simulator-side director controls.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ScenarioDirectorConfig {
    /// Mission-state authority. Defaults to `flight_controller` for
    /// scenarios that declare both `[fc]` and `[mission]`; use
    /// `kernel` only for simulator test stimulus that must evaluate
    /// mission events on truth.
    #[serde(default)]
    pub mission_authority: ScenarioMissionAuthority,
}

/// Mission-state authority selection for `[scenario_director]`.
#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioMissionAuthority {
    /// FC commander owns mission decisions and the kernel observes
    /// the published phase.
    #[default]
    FlightController,
    /// Kernel evaluates mission events against integrated truth. This
    /// is a simulator test-stimulus escape hatch, not SIL authority.
    Kernel,
}

/// Optional epoch metadata.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EpochConfig {
    /// Time scale, for example `UTC`.
    pub scale: String,
    /// ISO-8601 epoch timestamp.
    pub iso8601: String,
    /// Optional leap-second table path. The runner accepts either the
    /// deterministic OpenBMP TOML table or a NAIF `KPL/LSK` text
    /// kernel containing `DELTET/DELTA_AT`.
    pub leap_second_table: Option<PathBuf>,
    /// Optional SHA-256 pin for [`Self::leap_second_table`].
    pub leap_second_table_sha256: Option<String>,
    /// Optional Earth-orientation parameter table path.
    pub eop: Option<PathBuf>,
    /// Optional SHA-256 pin for [`Self::eop`].
    pub eop_sha256: Option<String>,
}

impl EpochConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_non_empty("epoch.scale", &self.scale)?;
        require_non_empty("epoch.iso8601", &self.iso8601)?;
        if self.leap_second_table_sha256.is_some() && self.leap_second_table.is_none() {
            return Err(ScenarioError::UnexpectedField {
                field: "epoch.leap_second_table_sha256".to_owned(),
                role: ModelRole::Frame,
                name: "epoch without leap_second_table".to_owned(),
            });
        }
        if self.eop_sha256.is_some() && self.eop.is_none() {
            return Err(ScenarioError::UnexpectedField {
                field: "epoch.eop_sha256".to_owned(),
                role: ModelRole::Frame,
                name: "epoch without eop".to_owned(),
            });
        }
        Ok(())
    }
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
/// Scenarios that select `frames.profile =
/// "wgs84-uniform-rotation"` declare a local origin so altitude, NED
/// wind, and vertical-launch initialisation are anchored to a stable
/// reference. The runner consumes this block; the parser
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

/// Offline range-safety landing-footprint configuration.
///
/// This block configures post-processing only. It accepts no desired
/// landing location and is never consumed by the flight-controller
/// loop.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct LandingFootprintConfig {
    /// Footprint prediction method.
    pub method: LandingFootprintMethod,
    /// Cull altitude where propagation stops (m).
    pub cull_altitude_m: f64,
    /// Whether the offline report should include geodetic
    /// latitude/longitude. Requires `[frames.local_origin]`; when
    /// `false`, only range-relative downrange/crossrange output is
    /// produced.
    #[serde(default)]
    pub include_geodetic: bool,
    /// Optional declared dispersion ellipse input.
    #[serde(default)]
    pub dispersion: Option<LandingFootprintDispersionConfig>,
    /// Optional Monte-Carlo dispersion analysis configuration.
    #[serde(default)]
    pub monte_carlo: Option<LandingFootprintMonteCarloConfig>,
}

impl LandingFootprintConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_finite("landing_footprint.cull_altitude_m", self.cull_altitude_m)?;
        if let Some(dispersion) = &self.dispersion {
            dispersion.validate()?;
        }
        if let Some(monte_carlo) = &self.monte_carlo {
            monte_carlo.validate()?;
        }
        Ok(())
    }
}

/// Offline ideal staging budget / optimal split configuration. This
/// is post-processing only and carries no trajectory, range, target,
/// or location fields.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct StagingAnalysisConfig {
    /// Analysis mode.
    pub mode: StagingAnalysisMode,
    /// Required ideal ΔV for `mode = "optimal"`; rejected for
    /// `mode = "budget"`.
    #[serde(default)]
    pub delta_v_budget_m_s: Option<f64>,
    /// Payload mass (kg).
    pub payload_mass_kg: f64,
    /// Stages ordered bottom-up.
    pub stages: Vec<StagingAnalysisStageConfig>,
}

impl StagingAnalysisConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_finite("staging_analysis.payload_mass_kg", self.payload_mass_kg)?;
        require_positive("staging_analysis.payload_mass_kg", self.payload_mass_kg)?;
        if self.stages.is_empty() {
            return Err(ScenarioError::EmptyList {
                field: "staging_analysis.stages".to_owned(),
            });
        }
        match self.mode {
            StagingAnalysisMode::Budget => {
                if self.delta_v_budget_m_s.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "staging_analysis.delta_v_budget_m_s".to_owned(),
                        role: ModelRole::Vehicle,
                        name: "staging_budget".to_owned(),
                    });
                }
            }
            StagingAnalysisMode::Optimal => {
                let delta_v =
                    self.delta_v_budget_m_s
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: "staging_analysis.delta_v_budget_m_s".to_owned(),
                            role: ModelRole::Vehicle,
                            name: "staging_optimal".to_owned(),
                        })?;
                require_finite("staging_analysis.delta_v_budget_m_s", delta_v)?;
                require_positive("staging_analysis.delta_v_budget_m_s", delta_v)?;
            }
        }
        for (index, stage) in self.stages.iter().enumerate() {
            stage.validate(index, self.mode)?;
        }
        Ok(())
    }
}

/// Staging analysis mode.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum StagingAnalysisMode {
    /// Forward budget from declared stage masses.
    Budget,
    /// Mass-optimal split for the declared ideal ΔV.
    Optimal,
}

/// One stage in `[staging_analysis]`, ordered bottom-up.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct StagingAnalysisStageConfig {
    /// Specific impulse (s).
    pub isp_s: f64,
    /// Structural coefficient.
    pub structural_coefficient: f64,
    /// Structural mass (kg), required for budget mode.
    #[serde(default)]
    pub structural_mass_kg: Option<f64>,
    /// Propellant mass (kg), required for budget mode.
    #[serde(default)]
    pub propellant_mass_kg: Option<f64>,
}

impl StagingAnalysisStageConfig {
    fn validate(&self, index: usize, mode: StagingAnalysisMode) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("staging_analysis.stages[{index}].{field}");
        require_finite(&path("isp_s"), self.isp_s)?;
        require_positive(&path("isp_s"), self.isp_s)?;
        require_finite(&path("structural_coefficient"), self.structural_coefficient)?;
        if !(0.0..1.0).contains(&self.structural_coefficient) {
            return Err(ScenarioError::InvalidNumber {
                field: path("structural_coefficient"),
                value: self.structural_coefficient,
                rule: "must lie in (0, 1)",
            });
        }
        match mode {
            StagingAnalysisMode::Budget => {
                let structural =
                    self.structural_mass_kg
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: path("structural_mass_kg"),
                            role: ModelRole::Vehicle,
                            name: "staging_budget".to_owned(),
                        })?;
                let propellant =
                    self.propellant_mass_kg
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: path("propellant_mass_kg"),
                            role: ModelRole::Vehicle,
                            name: "staging_budget".to_owned(),
                        })?;
                require_finite(&path("structural_mass_kg"), structural)?;
                require_positive(&path("structural_mass_kg"), structural)?;
                require_finite(&path("propellant_mass_kg"), propellant)?;
                require_positive(&path("propellant_mass_kg"), propellant)?;
            }
            StagingAnalysisMode::Optimal => {
                if self.structural_mass_kg.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: path("structural_mass_kg"),
                        role: ModelRole::Vehicle,
                        name: "staging_optimal".to_owned(),
                    });
                }
                if self.propellant_mass_kg.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: path("propellant_mass_kg"),
                        role: ModelRole::Vehicle,
                        name: "staging_optimal".to_owned(),
                    });
                }
            }
        }
        Ok(())
    }
}

/// Landing-footprint prediction method.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum LandingFootprintMethod {
    /// Constant-gravity closed-form toy method.
    ConstantGravity,
    /// Fixed-step numerical propagation under the scenario's J2
    /// gravity model.
    J2,
    /// Fixed-step numerical propagation under the scenario's
    /// zonal-only EGM2008 gravity model.
    Egm2008,
}

impl LandingFootprintMethod {
    /// Canonical method label used in scenario TOML.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::ConstantGravity => "constant_gravity",
            Self::J2 => "j2",
            Self::Egm2008 => "egm2008",
        }
    }

    const fn required_gravity_name(self) -> &'static str {
        match self {
            Self::ConstantGravity => "constant",
            Self::J2 => "j2",
            Self::Egm2008 => "egm2008",
        }
    }
}

/// Declared dispersion ellipse for a landing-footprint report.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct LandingFootprintDispersionConfig {
    /// One-sigma semi-major axis (m).
    pub one_sigma_semi_major_m: f64,
    /// One-sigma semi-minor axis (m).
    pub one_sigma_semi_minor_m: f64,
    /// Ellipse orientation in the downrange/crossrange plane (rad).
    pub orientation_rad: f64,
}

impl LandingFootprintDispersionConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_positive(
            "landing_footprint.dispersion.one_sigma_semi_major_m",
            self.one_sigma_semi_major_m,
        )?;
        require_positive(
            "landing_footprint.dispersion.one_sigma_semi_minor_m",
            self.one_sigma_semi_minor_m,
        )?;
        if self.one_sigma_semi_major_m < self.one_sigma_semi_minor_m {
            return Err(ScenarioError::InvalidNumber {
                field: "landing_footprint.dispersion.one_sigma_semi_major_m".to_owned(),
                value: self.one_sigma_semi_major_m,
                rule: "must be greater than or equal to one_sigma_semi_minor_m",
            });
        }
        require_finite(
            "landing_footprint.dispersion.orientation_rad",
            self.orientation_rad,
        )?;
        Ok(())
    }
}

/// Monte-Carlo landing-footprint dispersion configuration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct LandingFootprintMonteCarloConfig {
    /// Number of samples to propagate.
    pub samples: u32,
    /// Optional seed override. When absent, `[time].seed` is used.
    #[serde(default)]
    pub seed: Option<u64>,
    /// Confidence levels used for radial-distance quantiles.
    pub confidence_levels: Vec<f64>,
    /// Declared output paths for the offline analysis products.
    pub output: LandingFootprintMonteCarloOutputConfig,
    /// Optional wind uncertainty source.
    #[serde(default)]
    pub wind: Option<LandingFootprintMonteCarloWindConfig>,
    /// Optional ballistic-coefficient uncertainty source.
    #[serde(default)]
    pub ballistic_coefficient: Option<LandingFootprintMonteCarloBallisticCoefficientConfig>,
    /// Optional burnout-state uncertainty source.
    #[serde(default)]
    pub burnout_state: Option<LandingFootprintMonteCarloBurnoutStateConfig>,
}

impl LandingFootprintMonteCarloConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_positive_u32("landing_footprint.monte_carlo.samples", self.samples)?;
        if self.confidence_levels.is_empty() {
            return Err(ScenarioError::EmptyList {
                field: "landing_footprint.monte_carlo.confidence_levels".to_owned(),
            });
        }
        for (index, value) in self.confidence_levels.iter().copied().enumerate() {
            require_in_range(
                &format!("landing_footprint.monte_carlo.confidence_levels[{index}]"),
                value,
                f64::EPSILON,
                1.0 - f64::EPSILON,
            )?;
        }
        self.output.validate()?;
        if self.wind.is_none()
            && self.ballistic_coefficient.is_none()
            && self.burnout_state.is_none()
        {
            return Err(ScenarioError::InconsistentSection {
                field_a: "landing_footprint.monte_carlo".to_owned(),
                value_a: "declared".to_owned(),
                field_b: "landing_footprint.monte_carlo.uncertainty_source".to_owned(),
                value_b: "missing".to_owned(),
            });
        }
        if let Some(wind) = &self.wind {
            wind.validate()?;
        }
        if let Some(ballistic_coefficient) = &self.ballistic_coefficient {
            ballistic_coefficient.validate()?;
        }
        if let Some(burnout_state) = &self.burnout_state {
            burnout_state.validate()?;
        }
        Ok(())
    }
}

/// Output files produced by a Monte-Carlo landing-footprint run.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct LandingFootprintMonteCarloOutputConfig {
    /// Optional deterministic CSV sample-cloud output path.
    #[serde(default)]
    pub samples_csv: Option<PathBuf>,
    /// Optional deterministic Parquet sample-cloud output path.
    #[serde(default)]
    pub samples_parquet: Option<PathBuf>,
    /// Optional TOML summary output path.
    #[serde(default)]
    pub summary_toml: Option<PathBuf>,
}

impl LandingFootprintMonteCarloOutputConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if self.samples_csv.is_none() && self.samples_parquet.is_none() {
            return Err(ScenarioError::InconsistentSection {
                field_a: "landing_footprint.monte_carlo.output".to_owned(),
                value_a: "declared".to_owned(),
                field_b: "landing_footprint.monte_carlo.output.samples_csv_or_samples_parquet"
                    .to_owned(),
                value_b: "missing".to_owned(),
            });
        }
        if self.summary_toml.is_none() {
            return Err(ScenarioError::InconsistentSection {
                field_a: "landing_footprint.monte_carlo.output".to_owned(),
                value_a: "declared".to_owned(),
                field_b: "landing_footprint.monte_carlo.output.summary_toml".to_owned(),
                value_b: "missing".to_owned(),
            });
        }
        Ok(())
    }
}

/// Wind uncertainty model for Monte-Carlo footprint sampling.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct LandingFootprintMonteCarloWindConfig {
    /// Wind uncertainty shape.
    pub kind: LandingFootprintMonteCarloWindKind,
    /// Independent one-sigma local-NED additive wind perturbation (m/s).
    #[serde(default)]
    pub sigma_ned_m_s: Option<[f64; 3]>,
    /// One-sigma multiplicative speed-scale perturbation.
    #[serde(default)]
    pub speed_scale_sigma: Option<f64>,
    /// Optional deterministic local-NED ensemble member vectors (m/s).
    #[serde(default)]
    pub ensemble_members_ned_m_s: Option<Vec<[f64; 3]>>,
}

impl LandingFootprintMonteCarloWindConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if let Some(sigma) = self.sigma_ned_m_s {
            require_non_negative_array("landing_footprint.monte_carlo.wind.sigma_ned_m_s", &sigma)?;
        }
        if let Some(sigma) = self.speed_scale_sigma {
            require_non_negative(
                "landing_footprint.monte_carlo.wind.speed_scale_sigma",
                sigma,
            )?;
        }
        if let Some(members) = &self.ensemble_members_ned_m_s {
            if members.is_empty() {
                return Err(ScenarioError::EmptyList {
                    field: "landing_footprint.monte_carlo.wind.ensemble_members_ned_m_s".to_owned(),
                });
            }
            for (index, member) in members.iter().enumerate() {
                require_finite_array(
                    &format!(
                        "landing_footprint.monte_carlo.wind.ensemble_members_ned_m_s[{index}]"
                    ),
                    member,
                )?;
            }
        }
        let has_parameter = self.sigma_ned_m_s.is_some()
            || self.speed_scale_sigma.is_some()
            || self.ensemble_members_ned_m_s.is_some();
        if !has_parameter {
            return Err(ScenarioError::InconsistentSection {
                field_a: "landing_footprint.monte_carlo.wind".to_owned(),
                value_a: self.kind.as_str().to_owned(),
                field_b: "landing_footprint.monte_carlo.wind.uncertainty_parameter".to_owned(),
                value_b: "missing".to_owned(),
            });
        }
        if self.kind == LandingFootprintMonteCarloWindKind::Ensemble
            && self.ensemble_members_ned_m_s.is_none()
        {
            return Err(ScenarioError::MissingRequiredField {
                field: "landing_footprint.monte_carlo.wind.ensemble_members_ned_m_s".to_owned(),
                role: ModelRole::Wind,
                name: "ensemble".to_owned(),
            });
        }
        Ok(())
    }
}

/// Supported wind uncertainty shapes for footprint Monte Carlo.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum LandingFootprintMonteCarloWindKind {
    /// Additive perturbation around a constant wind vector.
    Constant,
    /// Layered-wind perturbation collapsed to the sampled footprint
    /// propagation vector for the offline run.
    Layered,
    /// HWM14 perturbation factor collapsed to the sampled footprint
    /// propagation vector for the offline run.
    Hwm14,
    /// Deterministically selected ensemble member.
    Ensemble,
}

impl LandingFootprintMonteCarloWindKind {
    const fn as_str(self) -> &'static str {
        match self {
            Self::Constant => "constant",
            Self::Layered => "layered",
            Self::Hwm14 => "hwm14",
            Self::Ensemble => "ensemble",
        }
    }
}

/// Ballistic-coefficient uncertainty for footprint Monte Carlo.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct LandingFootprintMonteCarloBallisticCoefficientConfig {
    /// Nominal `C_d A / m` value (m²/kg).
    pub nominal_m2_kg: f64,
    /// One-sigma uncertainty (m²/kg).
    pub sigma_m2_kg: f64,
    /// Sampling distribution.
    pub distribution: LandingFootprintMonteCarloDistribution,
    /// Optional lower bound (m²/kg).
    #[serde(default)]
    pub min_m2_kg: Option<f64>,
    /// Optional upper bound (m²/kg).
    #[serde(default)]
    pub max_m2_kg: Option<f64>,
}

impl LandingFootprintMonteCarloBallisticCoefficientConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_non_negative(
            "landing_footprint.monte_carlo.ballistic_coefficient.nominal_m2_kg",
            self.nominal_m2_kg,
        )?;
        require_non_negative(
            "landing_footprint.monte_carlo.ballistic_coefficient.sigma_m2_kg",
            self.sigma_m2_kg,
        )?;
        if let Some(min) = self.min_m2_kg {
            require_non_negative(
                "landing_footprint.monte_carlo.ballistic_coefficient.min_m2_kg",
                min,
            )?;
        }
        if let Some(max) = self.max_m2_kg {
            require_non_negative(
                "landing_footprint.monte_carlo.ballistic_coefficient.max_m2_kg",
                max,
            )?;
        }
        if let (Some(min), Some(max)) = (self.min_m2_kg, self.max_m2_kg)
            && min > max
        {
            return Err(ScenarioError::InvalidNumber {
                field: "landing_footprint.monte_carlo.ballistic_coefficient.min_m2_kg".to_owned(),
                value: min,
                rule: "must be less than or equal to max_m2_kg",
            });
        }
        Ok(())
    }
}

/// Scalar sampling distribution for footprint Monte Carlo sources.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum LandingFootprintMonteCarloDistribution {
    /// Gaussian sampling with the declared one-sigma value.
    Normal,
    /// Uniform sampling with the same standard deviation as the
    /// declared one-sigma value.
    Uniform,
}

/// Burnout-state uncertainty for footprint Monte Carlo.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct LandingFootprintMonteCarloBurnoutStateConfig {
    /// Independent one-sigma ECI position perturbations (m).
    #[serde(default)]
    pub position_sigma_eci_m: Option<[f64; 3]>,
    /// One-sigma inertial speed-magnitude perturbation (m/s).
    ///
    /// This intentionally perturbs only the scalar speed along the
    /// nominal velocity direction. It does not expose per-axis
    /// velocity-direction control.
    #[serde(default)]
    pub speed_sigma_m_s: Option<f64>,
    /// One-sigma timestamp perturbation (s).
    #[serde(default)]
    pub time_sigma_s: Option<f64>,
}

impl LandingFootprintMonteCarloBurnoutStateConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if let Some(sigma) = self.position_sigma_eci_m {
            require_non_negative_array(
                "landing_footprint.monte_carlo.burnout_state.position_sigma_eci_m",
                &sigma,
            )?;
        }
        if let Some(sigma) = self.speed_sigma_m_s {
            require_non_negative(
                "landing_footprint.monte_carlo.burnout_state.speed_sigma_m_s",
                sigma,
            )?;
        }
        if let Some(sigma) = self.time_sigma_s {
            require_non_negative(
                "landing_footprint.monte_carlo.burnout_state.time_sigma_s",
                sigma,
            )?;
        }
        if self.position_sigma_eci_m.is_none()
            && self.speed_sigma_m_s.is_none()
            && self.time_sigma_s.is_none()
        {
            return Err(ScenarioError::InconsistentSection {
                field_a: "landing_footprint.monte_carlo.burnout_state".to_owned(),
                value_a: "declared".to_owned(),
                field_b: "landing_footprint.monte_carlo.burnout_state.sigma".to_owned(),
                value_b: "missing".to_owned(),
            });
        }
        Ok(())
    }
}

fn require_non_negative(field: &str, value: f64) -> Result<(), ScenarioError> {
    require_finite(field, value)?;
    if value < 0.0 {
        return Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value,
            rule: "must be non-negative",
        });
    }
    Ok(())
}

fn require_non_negative_array(field: &str, values: &[f64]) -> Result<(), ScenarioError> {
    for value in values {
        require_non_negative(field, *value)?;
    }
    Ok(())
}

const fn default_entry_surface_density_kg_m3() -> f64 {
    1.225
}

const fn default_entry_scale_height_m() -> f64 {
    7_000.0
}

/// Descent / entry profile configuration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EntryProfileConfig {
    /// Entry mode: ballistic analytic diagnostic or lifting-entry
    /// corridor reference.
    pub mode: EntryProfileMode,
    /// Entry-interface altitude (m), usually around 122 km when the
    /// atmosphere model supports that envelope.
    pub entry_interface_altitude_m: f64,
    /// Optional final-descent handoff altitude (m).
    #[serde(default)]
    pub final_descent_altitude_m: Option<f64>,
    /// Reference surface density for the analytic entry models
    /// (kg/m³).
    #[serde(default = "default_entry_surface_density_kg_m3")]
    pub surface_density_kg_m3: f64,
    /// Reference exponential atmosphere scale height (m).
    #[serde(default = "default_entry_scale_height_m")]
    pub scale_height_m: f64,
    /// Optional nose radius for stagnation-point heating diagnostics (m).
    #[serde(default)]
    pub nose_radius_m: Option<f64>,
    /// Lift-to-drag ratio, required for `mode = "lifting"`.
    #[serde(default)]
    pub lift_to_drag_ratio: Option<f64>,
    /// Corridor limits and bank command bounds for lifting entry.
    #[serde(default)]
    pub corridor: Option<EntryCorridorConfig>,
}

impl EntryProfileConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_positive(
            "entry_profile.entry_interface_altitude_m",
            self.entry_interface_altitude_m,
        )?;
        if let Some(final_descent_altitude_m) = self.final_descent_altitude_m {
            require_positive(
                "entry_profile.final_descent_altitude_m",
                final_descent_altitude_m,
            )?;
            if final_descent_altitude_m >= self.entry_interface_altitude_m {
                return Err(ScenarioError::InvalidNumber {
                    field: "entry_profile.final_descent_altitude_m".to_owned(),
                    value: final_descent_altitude_m,
                    rule: "must be below entry_interface_altitude_m",
                });
            }
        }
        require_positive(
            "entry_profile.surface_density_kg_m3",
            self.surface_density_kg_m3,
        )?;
        require_positive("entry_profile.scale_height_m", self.scale_height_m)?;
        if let Some(nose_radius_m) = self.nose_radius_m {
            require_positive("entry_profile.nose_radius_m", nose_radius_m)?;
        }
        if let Some(lift_to_drag_ratio) = self.lift_to_drag_ratio {
            require_positive("entry_profile.lift_to_drag_ratio", lift_to_drag_ratio)?;
        }
        match self.mode {
            EntryProfileMode::Ballistic => {
                if self.lift_to_drag_ratio.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "entry_profile.lift_to_drag_ratio".to_owned(),
                        role: ModelRole::Trajectory,
                        name: "ballistic".to_owned(),
                    });
                }
                if self.corridor.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "entry_profile.corridor".to_owned(),
                        role: ModelRole::Trajectory,
                        name: "ballistic".to_owned(),
                    });
                }
            }
            EntryProfileMode::Lifting => {
                if self.lift_to_drag_ratio.is_none() {
                    return Err(ScenarioError::MissingRequiredField {
                        field: "entry_profile.lift_to_drag_ratio".to_owned(),
                        role: ModelRole::Trajectory,
                        name: "lifting".to_owned(),
                    });
                }
                let corridor =
                    self.corridor
                        .as_ref()
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: "entry_profile.corridor".to_owned(),
                            role: ModelRole::Trajectory,
                            name: "lifting".to_owned(),
                        })?;
                corridor.validate()?;
            }
        }
        Ok(())
    }
}

/// Entry-profile mode.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum EntryProfileMode {
    /// Ballistic entry diagnostics using Allen-Eggers closed forms.
    Ballistic,
    /// Lifting entry reference using a corridor-limited bank command.
    Lifting,
}

/// Lifting-entry corridor and bank-reference configuration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EntryCorridorConfig {
    /// Maximum stagnation-point heat rate (W/m²).
    pub max_heat_rate_w_m2: f64,
    /// Maximum deceleration load factor (g).
    pub max_load_factor_g: f64,
    /// Flight-path-angle corridor half-width (rad).
    pub flight_path_angle_band_rad: f64,
    /// Nominal bank angle (rad).
    #[serde(default)]
    pub nominal_bank_rad: f64,
    /// Maximum absolute bank angle (rad).
    pub max_bank_rad: f64,
}

impl EntryCorridorConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_positive(
            "entry_profile.corridor.max_heat_rate_w_m2",
            self.max_heat_rate_w_m2,
        )?;
        require_positive(
            "entry_profile.corridor.max_load_factor_g",
            self.max_load_factor_g,
        )?;
        require_positive(
            "entry_profile.corridor.flight_path_angle_band_rad",
            self.flight_path_angle_band_rad,
        )?;
        if self.flight_path_angle_band_rad >= std::f64::consts::FRAC_PI_2 {
            return Err(ScenarioError::InvalidNumber {
                field: "entry_profile.corridor.flight_path_angle_band_rad".to_owned(),
                value: self.flight_path_angle_band_rad,
                rule: "must be below pi/2",
            });
        }
        require_finite(
            "entry_profile.corridor.nominal_bank_rad",
            self.nominal_bank_rad,
        )?;
        require_positive("entry_profile.corridor.max_bank_rad", self.max_bank_rad)?;
        if self.max_bank_rad > std::f64::consts::PI {
            return Err(ScenarioError::InvalidNumber {
                field: "entry_profile.corridor.max_bank_rad".to_owned(),
                value: self.max_bank_rad,
                rule: "must be <= pi",
            });
        }
        if self.nominal_bank_rad.abs() > self.max_bank_rad {
            return Err(ScenarioError::InvalidNumber {
                field: "entry_profile.corridor.nominal_bank_rad".to_owned(),
                value: self.nominal_bank_rad,
                rule: "absolute value must be <= max_bank_rad",
            });
        }
        Ok(())
    }
}

/// Aerodynamic coefficient source.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroConfig {
    /// Path to an aero deck TOML file (resolved relative to
    /// the scenario directory). Mutually exclusive with
    /// [`Self::buildup`].
    #[serde(default)]
    pub deck: Option<PathBuf>,
    /// Inline geometry-driven continuum drag buildup. Mutually
    /// exclusive with [`Self::deck`].
    #[serde(default)]
    pub buildup: Option<AeroBuildupConfig>,
    /// Runtime aero method dispatch. Absent or `kind = "deck"` keeps
    /// the legacy deck / buildup path; hypersonic kinds are evaluated
    /// directly in the kernel loop.
    #[serde(default)]
    pub method: Option<AeroMethodConfig>,
    /// Optional owner body for post-separation force-stack routing.
    /// Required when `[multi_body]` is declared and `forces.models`
    /// includes `aero`.
    #[serde(default)]
    pub mounted_to: Option<String>,
    /// Optional pinned SHA-256 digest (lower-case hex). When present,
    /// a mismatch with the file's actual digest fails closed.
    pub deck_sha256: Option<String>,
}

impl AeroConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        let method_kind = self
            .method
            .as_ref()
            .map_or("deck", |method| method.kind.as_str());
        if let Some(method) = &self.method {
            method.validate("aero.method")?;
        }
        let method_uses_deck = self.method.as_ref().is_none_or(AeroMethodConfig::uses_deck);
        if method_uses_deck {
            match (&self.deck, &self.buildup) {
                (Some(_), Some(_)) => return Err(ScenarioError::AmbiguousAero),
                (None, None) => {
                    return Err(ScenarioError::MissingRequiredField {
                        field: "aero.deck_or_buildup".to_owned(),
                        role: ModelRole::Force,
                        name: "aero".to_owned(),
                    });
                }
                (Some(deck), None) => {
                    if deck.as_os_str().is_empty() {
                        return Err(ScenarioError::EmptyField {
                            field: "aero.deck".to_owned(),
                        });
                    }
                }
                (None, Some(buildup)) => {
                    if self.deck_sha256.is_some() {
                        return Err(ScenarioError::UnexpectedField {
                            field: "aero.deck_sha256".to_owned(),
                            role: ModelRole::Force,
                            name: "buildup".to_owned(),
                        });
                    }
                    buildup.validate("aero.buildup")?;
                }
            }
        } else {
            if self.deck.is_some() || self.buildup.is_some() || self.deck_sha256.is_some() {
                return Err(ScenarioError::UnexpectedField {
                    field: "aero.deck/aero.buildup/aero.deck_sha256".to_owned(),
                    role: ModelRole::Force,
                    name: method_kind.to_owned(),
                });
            }
        }
        if let Some(mounted_to) = &self.mounted_to {
            require_non_empty("aero.mounted_to", mounted_to)?;
        }
        Ok(())
    }
}

/// Runtime aerodynamic method selector.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroMethodConfig {
    /// Method kind: `deck`, `modified_newtonian`, `tangent_cone`,
    /// `tangent_wedge`, `free_molecular`, or `hybrid`.
    pub kind: String,
    /// Parameters for `kind = "modified_newtonian"`.
    #[serde(default)]
    pub modified_newtonian: Option<AeroModifiedNewtonianConfig>,
    /// Parameters for `kind = "tangent_cone"`.
    #[serde(default)]
    pub tangent_cone: Option<AeroTangentConeConfig>,
    /// Parameters for `kind = "tangent_wedge"`.
    #[serde(default)]
    pub tangent_wedge: Option<AeroTangentWedgeConfig>,
    /// Parameters for `kind = "free_molecular"`.
    #[serde(default)]
    pub free_molecular: Option<AeroFreeMolecularConfig>,
    /// Parameters for `kind = "hybrid"`.
    #[serde(default)]
    pub hybrid: Option<AeroHybridConfig>,
}

impl AeroMethodConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_supported(
            &format!("{path}.kind"),
            &self.kind,
            &[
                "deck",
                "modified_newtonian",
                "tangent_cone",
                "tangent_wedge",
                "free_molecular",
                "hybrid",
            ],
        )?;
        match self.kind.as_str() {
            "deck" => self.reject_subtables(path, "deck"),
            "modified_newtonian" => {
                required_method(&self.modified_newtonian, path, "modified_newtonian")?
                    .validate(&format!("{path}.modified_newtonian"))?;
                self.reject_unselected(path, &["modified_newtonian"])
            }
            "tangent_cone" => {
                required_method(&self.tangent_cone, path, "tangent_cone")?
                    .validate(&format!("{path}.tangent_cone"))?;
                self.reject_unselected(path, &["tangent_cone"])
            }
            "tangent_wedge" => {
                required_method(&self.tangent_wedge, path, "tangent_wedge")?
                    .validate(&format!("{path}.tangent_wedge"))?;
                self.reject_unselected(path, &["tangent_wedge"])
            }
            "free_molecular" => {
                required_method(&self.free_molecular, path, "free_molecular")?
                    .validate(&format!("{path}.free_molecular"))?;
                self.reject_unselected(path, &["free_molecular"])
            }
            "hybrid" => {
                required_method(&self.hybrid, path, "hybrid")?
                    .validate(&format!("{path}.hybrid"))?;
                self.reject_unselected(path, &["hybrid"])
            }
            _ => Ok(()),
        }
    }

    fn uses_deck(&self) -> bool {
        self.kind == "deck"
            || self
                .hybrid
                .as_ref()
                .is_some_and(AeroHybridConfig::uses_deck)
    }

    fn reject_subtables(&self, path: &str, kind: &str) -> Result<(), ScenarioError> {
        self.reject_unselected(path, &[])
            .map_err(|_| ScenarioError::UnexpectedField {
                field: format!("{path}.*"),
                role: ModelRole::Force,
                name: kind.to_owned(),
            })
    }

    fn reject_unselected(&self, path: &str, selected: &[&str]) -> Result<(), ScenarioError> {
        for (name, present) in [
            ("modified_newtonian", self.modified_newtonian.is_some()),
            ("tangent_cone", self.tangent_cone.is_some()),
            ("tangent_wedge", self.tangent_wedge.is_some()),
            ("free_molecular", self.free_molecular.is_some()),
            ("hybrid", self.hybrid.is_some()),
        ] {
            if present && !selected.contains(&name) {
                return Err(ScenarioError::UnexpectedField {
                    field: format!("{path}.{name}"),
                    role: ModelRole::Force,
                    name: self.kind.clone(),
                });
            }
        }
        Ok(())
    }
}

fn required_method<'a, T>(
    value: &'a Option<T>,
    path: &str,
    kind: &str,
) -> Result<&'a T, ScenarioError> {
    value
        .as_ref()
        .ok_or_else(|| ScenarioError::MissingRequiredField {
            field: format!("{path}.{kind}"),
            role: ModelRole::Force,
            name: kind.to_owned(),
        })
}

/// Modified-Newtonian method parameters.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroModifiedNewtonianConfig {
    /// Stagnation pressure coefficient.
    pub cp_max: f64,
    /// Reference area (m^2).
    pub reference_area_m2: f64,
    /// Reference length (m).
    pub reference_length_m: f64,
}

impl AeroModifiedNewtonianConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_finite(&format!("{path}.cp_max"), self.cp_max)?;
        if self.cp_max < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.cp_max"),
                value: self.cp_max,
                rule: "must be non-negative",
            });
        }
        require_positive(&format!("{path}.reference_area_m2"), self.reference_area_m2)?;
        require_positive(
            &format!("{path}.reference_length_m"),
            self.reference_length_m,
        )?;
        Ok(())
    }
}

/// Tangent-cone method parameters.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroTangentConeConfig {
    /// Cone half angle (rad).
    pub cone_half_angle_rad: f64,
    /// Reference area (m^2).
    pub reference_area_m2: f64,
    /// Reference length (m).
    pub reference_length_m: f64,
    /// Ratio of specific heats.
    #[serde(default = "default_gamma")]
    pub gamma: f64,
}

impl AeroTangentConeConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_positive(
            &format!("{path}.cone_half_angle_rad"),
            self.cone_half_angle_rad,
        )?;
        if self.cone_half_angle_rad >= std::f64::consts::FRAC_PI_2 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.cone_half_angle_rad"),
                value: self.cone_half_angle_rad,
                rule: "must be less than pi/2",
            });
        }
        require_positive(&format!("{path}.reference_area_m2"), self.reference_area_m2)?;
        require_positive(
            &format!("{path}.reference_length_m"),
            self.reference_length_m,
        )?;
        require_positive(&format!("{path}.gamma"), self.gamma)?;
        Ok(())
    }
}

/// Tangent-wedge method parameters.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroTangentWedgeConfig {
    /// Wedge half angle (rad).
    pub wedge_half_angle_rad: f64,
    /// Reference area (m^2).
    pub reference_area_m2: f64,
    /// Ratio of specific heats.
    #[serde(default = "default_gamma")]
    pub gamma: f64,
}

impl AeroTangentWedgeConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_positive(
            &format!("{path}.wedge_half_angle_rad"),
            self.wedge_half_angle_rad,
        )?;
        if self.wedge_half_angle_rad >= std::f64::consts::FRAC_PI_2 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.wedge_half_angle_rad"),
                value: self.wedge_half_angle_rad,
                rule: "must be less than pi/2",
            });
        }
        require_positive(&format!("{path}.reference_area_m2"), self.reference_area_m2)?;
        require_positive(&format!("{path}.gamma"), self.gamma)?;
        Ok(())
    }
}

/// Free-molecular method parameters.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroFreeMolecularConfig {
    /// Reference area (m^2).
    pub reference_area_m2: f64,
    /// Normal accommodation coefficient.
    pub accommodation_normal: f64,
    /// Tangential accommodation coefficient.
    pub accommodation_tangential: f64,
}

impl AeroFreeMolecularConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_positive(&format!("{path}.reference_area_m2"), self.reference_area_m2)?;
        validate_unit_interval(
            &format!("{path}.accommodation_normal"),
            self.accommodation_normal,
        )?;
        validate_unit_interval(
            &format!("{path}.accommodation_tangential"),
            self.accommodation_tangential,
        )?;
        Ok(())
    }
}

/// Runtime hybrid aero dispatch across continuum and free-molecular
/// regimes.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroHybridConfig {
    /// Reference length (m) used for Knudsen/Reynolds context fields.
    pub reference_length_m: f64,
    /// Continuum method below the Mach handoff. Use `kind = "deck"`
    /// to reuse the enclosing `[aero].deck` or `[aero.buildup]`.
    pub continuum_low_mach: Box<AeroMethodConfig>,
    /// Continuum method above the Mach handoff.
    pub continuum_high_mach: Box<AeroMethodConfig>,
    /// Free-molecular method for the high-Knudsen-number limit.
    pub free_molecular: AeroFreeMolecularConfig,
    /// Mach number where low/high continuum methods switch when no
    /// Mach bridge is configured.
    #[serde(default = "default_hybrid_mach_handoff")]
    pub mach_handoff: f64,
    /// Optional linear Mach bridge across the continuum-method handoff.
    #[serde(default)]
    pub mach_bridge: Option<AeroLinearMachBridgeConfig>,
    /// Knudsen bridge blending continuum and free-molecular methods.
    #[serde(default)]
    pub bridge: AeroKnudsenBridgeConfig,
}

impl AeroHybridConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_positive(
            &format!("{path}.reference_length_m"),
            self.reference_length_m,
        )?;
        require_positive(&format!("{path}.mach_handoff"), self.mach_handoff)?;
        self.continuum_low_mach
            .validate(&format!("{path}.continuum_low_mach"))?;
        self.continuum_high_mach
            .validate(&format!("{path}.continuum_high_mach"))?;
        self.free_molecular
            .validate(&format!("{path}.free_molecular"))?;
        if let Some(mach_bridge) = &self.mach_bridge {
            mach_bridge.validate(&format!("{path}.mach_bridge"))?;
        }
        self.bridge.validate(&format!("{path}.bridge"))?;
        Ok(())
    }

    fn uses_deck(&self) -> bool {
        self.continuum_low_mach.uses_deck() || self.continuum_high_mach.uses_deck()
    }
}

/// Linear Mach bridge parameters for hybrid aero handoff.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroLinearMachBridgeConfig {
    /// Lower Mach end of the transition band.
    #[serde(default = "default_hybrid_mach_bridge_lo")]
    pub mach_lo: f64,
    /// Upper Mach end of the transition band.
    #[serde(default = "default_hybrid_mach_bridge_hi")]
    pub mach_hi: f64,
}

impl AeroLinearMachBridgeConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_positive(&format!("{path}.mach_lo"), self.mach_lo)?;
        require_positive(&format!("{path}.mach_hi"), self.mach_hi)?;
        if self.mach_hi <= self.mach_lo {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.mach_hi"),
                value: self.mach_hi,
                rule: "must be greater than mach_lo",
            });
        }
        Ok(())
    }
}

/// Knudsen bridge-function selector for hybrid aero.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroKnudsenBridgeConfig {
    /// Bridge kind: `cheng`, `erfc`, or `linear`.
    #[serde(default = "default_hybrid_knudsen_bridge_kind")]
    pub kind: String,
    /// Width parameter for `kind = "erfc"`.
    #[serde(default = "default_hybrid_erfc_sigma")]
    pub sigma: f64,
    /// Lower Knudsen end of the transition band for
    /// `kind = "linear"`.
    #[serde(default = "default_hybrid_kn_lo")]
    pub kn_lo: f64,
    /// Upper Knudsen end of the transition band for
    /// `kind = "linear"`.
    #[serde(default = "default_hybrid_kn_hi")]
    pub kn_hi: f64,
}

impl Default for AeroKnudsenBridgeConfig {
    fn default() -> Self {
        Self {
            kind: default_hybrid_knudsen_bridge_kind(),
            sigma: default_hybrid_erfc_sigma(),
            kn_lo: default_hybrid_kn_lo(),
            kn_hi: default_hybrid_kn_hi(),
        }
    }
}

impl AeroKnudsenBridgeConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_supported(
            &format!("{path}.kind"),
            &self.kind,
            &["cheng", "erfc", "linear"],
        )?;
        match self.kind.as_str() {
            "cheng" => {}
            "erfc" => require_positive(&format!("{path}.sigma"), self.sigma)?,
            "linear" => {
                require_positive(&format!("{path}.kn_lo"), self.kn_lo)?;
                require_positive(&format!("{path}.kn_hi"), self.kn_hi)?;
                if self.kn_hi <= self.kn_lo {
                    return Err(ScenarioError::InvalidNumber {
                        field: format!("{path}.kn_hi"),
                        value: self.kn_hi,
                        rule: "must be greater than kn_lo",
                    });
                }
            }
            _ => unreachable!("validated bridge kind"),
        }
        Ok(())
    }
}

const fn default_hybrid_mach_handoff() -> f64 {
    4.0
}

const fn default_hybrid_mach_bridge_lo() -> f64 {
    4.5
}

const fn default_hybrid_mach_bridge_hi() -> f64 {
    5.5
}

fn default_hybrid_knudsen_bridge_kind() -> String {
    "cheng".to_owned()
}

const fn default_hybrid_erfc_sigma() -> f64 {
    1.5
}

const fn default_hybrid_kn_lo() -> f64 {
    0.01
}

const fn default_hybrid_kn_hi() -> f64 {
    10.0
}

const fn default_gamma() -> f64 {
    1.4
}

fn validate_unit_interval(field: &str, value: f64) -> Result<(), ScenarioError> {
    require_finite(field, value)?;
    if !(0.0..=1.0).contains(&value) {
        return Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value,
            rule: "must be in [0, 1]",
        });
    }
    Ok(())
}

/// Live aerothermal driver configuration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AerothermalConfig {
    /// Stagnation model kind: `sutton_graves` or `fay_riddell`.
    pub stagnation_kind: String,
    /// Stagnation-point nose radius (m).
    pub nose_radius_m: f64,
    /// Constant wall temperature unless a thermal toy is declared (K).
    pub wall_temperature_k: f64,
    /// Wall catalysis selector.
    #[serde(default = "default_wall_catalysis")]
    pub wall_catalysis: String,
    /// Fay-Riddell parameters required when `stagnation_kind =
    /// "fay_riddell"`.
    #[serde(default)]
    pub fay_riddell: Option<AerothermalFayRiddellConfig>,
    /// Optional stateful thermal-conduction toy.
    #[serde(default)]
    pub thermal_toy: Option<AerothermalThermalToyConfig>,
    /// Optional stateful depth-resolved ablation toy.
    #[serde(default)]
    pub ablation: Option<AerothermalAblationConfig>,
}

impl AerothermalConfig {
    fn validate(&self, dt_s: f64) -> Result<(), ScenarioError> {
        require_supported(
            "aerothermal.stagnation_kind",
            &self.stagnation_kind,
            &["sutton_graves", "fay_riddell"],
        )?;
        require_positive("aerothermal.nose_radius_m", self.nose_radius_m)?;
        require_finite("aerothermal.wall_temperature_k", self.wall_temperature_k)?;
        if !(100.0..=5000.0).contains(&self.wall_temperature_k) {
            return Err(ScenarioError::InvalidNumber {
                field: "aerothermal.wall_temperature_k".to_owned(),
                value: self.wall_temperature_k,
                rule: "must be in [100, 5000]",
            });
        }
        require_supported(
            "aerothermal.wall_catalysis",
            &self.wall_catalysis,
            &["fully_catalytic", "non_catalytic"],
        )?;
        match self.stagnation_kind.as_str() {
            "sutton_graves" if self.fay_riddell.is_some() => {
                return Err(ScenarioError::UnexpectedField {
                    field: "aerothermal.fay_riddell".to_owned(),
                    role: ModelRole::Force,
                    name: "sutton_graves".to_owned(),
                });
            }
            "fay_riddell" => {
                self.fay_riddell
                    .as_ref()
                    .ok_or_else(|| ScenarioError::MissingRequiredField {
                        field: "aerothermal.fay_riddell".to_owned(),
                        role: ModelRole::Force,
                        name: "fay_riddell".to_owned(),
                    })?
                    .validate("aerothermal.fay_riddell")?;
            }
            _ => {}
        }
        if let Some(thermal) = &self.thermal_toy {
            thermal.validate("aerothermal.thermal_toy", dt_s)?;
        }
        if let Some(ablation) = &self.ablation {
            ablation.validate("aerothermal.ablation")?;
        }
        Ok(())
    }
}

fn default_wall_catalysis() -> String {
    "fully_catalytic".to_owned()
}

/// Fay-Riddell scenario parameters.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AerothermalFayRiddellConfig {
    /// Lewis number.
    pub lewis_number: f64,
}

impl AerothermalFayRiddellConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_positive(&format!("{path}.lewis_number"), self.lewis_number)
    }
}

/// Stateful thermal-toy configuration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AerothermalThermalToyConfig {
    /// Textbook material selector.
    pub material: String,
    /// Slab thickness (m).
    pub thickness_m: f64,
    /// Number of interior nodes.
    pub n_nodes: u32,
    /// Initial uniform slab temperature (K).
    pub initial_temperature_k: f64,
    /// Backwall boundary condition.
    #[serde(default)]
    pub backwall: Option<AerothermalBackwallConfig>,
}

impl AerothermalThermalToyConfig {
    fn validate(&self, path: &str, dt_s: f64) -> Result<(), ScenarioError> {
        require_supported(
            &format!("{path}.material"),
            &self.material,
            &["textbook_pica_like", "textbook_avcoat_like"],
        )?;
        require_positive(&format!("{path}.thickness_m"), self.thickness_m)?;
        if self.n_nodes < 5 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.n_nodes"),
                value: f64::from(self.n_nodes),
                rule: "must be >= 5",
            });
        }
        require_positive(
            &format!("{path}.initial_temperature_k"),
            self.initial_temperature_k,
        )?;
        if let Some(backwall) = &self.backwall {
            backwall.validate(&format!("{path}.backwall"))?;
        }
        let alpha = match self.material.as_str() {
            "textbook_pica_like" => 0.08 / (270.0 * 1200.0),
            "textbook_avcoat_like" => 0.16 / (512.0 * 1250.0),
            _ => 0.0,
        };
        let dx = self.thickness_m / f64::from(self.n_nodes + 1);
        if alpha * dt_s / (dx * dx) >= 0.5 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.n_nodes"),
                value: f64::from(self.n_nodes),
                rule: "violates explicit thermal-toy Fourier stability bound for time.dt_s",
            });
        }
        Ok(())
    }
}

/// Thermal-toy backwall boundary condition.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AerothermalBackwallConfig {
    /// `adiabatic`, `prescribed`, or `convective`.
    pub kind: String,
    /// Prescribed backwall temperature (K), required for
    /// `kind = "prescribed"`.
    #[serde(default)]
    pub t_k: Option<f64>,
    /// Convective heat-transfer coefficient, required for
    /// `kind = "convective"`.
    #[serde(default)]
    pub h_w_m2_k: Option<f64>,
    /// Convective sink temperature (K), required for
    /// `kind = "convective"`.
    #[serde(default)]
    pub t_inf_k: Option<f64>,
}

impl AerothermalBackwallConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_supported(
            &format!("{path}.kind"),
            &self.kind,
            &["adiabatic", "prescribed", "convective"],
        )?;
        match self.kind.as_str() {
            "adiabatic"
                if self.t_k.is_some() || self.h_w_m2_k.is_some() || self.t_inf_k.is_some() =>
            {
                return Err(ScenarioError::UnexpectedField {
                    field: path.to_owned(),
                    role: ModelRole::Force,
                    name: "adiabatic".to_owned(),
                });
            }
            "prescribed" => {
                require_positive(
                    &format!("{path}.t_k"),
                    self.t_k
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{path}.t_k"),
                            role: ModelRole::Force,
                            name: "prescribed".to_owned(),
                        })?,
                )?;
                if self.h_w_m2_k.is_some() || self.t_inf_k.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: format!("{path}.h_w_m2_k/t_inf_k"),
                        role: ModelRole::Force,
                        name: "prescribed".to_owned(),
                    });
                }
            }
            "convective" => {
                require_positive(
                    &format!("{path}.h_w_m2_k"),
                    self.h_w_m2_k
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{path}.h_w_m2_k"),
                            role: ModelRole::Force,
                            name: "convective".to_owned(),
                        })?,
                )?;
                require_positive(
                    &format!("{path}.t_inf_k"),
                    self.t_inf_k
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{path}.t_inf_k"),
                            role: ModelRole::Force,
                            name: "convective".to_owned(),
                        })?,
                )?;
                if self.t_k.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: format!("{path}.t_k"),
                        role: ModelRole::Force,
                        name: "convective".to_owned(),
                    });
                }
            }
            _ => {}
        }
        Ok(())
    }
}

/// Stateful ablation configuration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AerothermalAblationConfig {
    /// Virgin textbook material selector.
    pub virgin_material: String,
    /// Char textbook material selector.
    pub char_material: String,
    /// Modeled slab thickness (m).
    pub thickness_m: f64,
    /// Number of depth nodes.
    pub n_nodes: u32,
    /// Pyrolysis enthalpy (J/kg).
    pub pyrolysis_enthalpy_j_kg: f64,
    /// Fraction of pyrolyzed mass emitted as gas.
    pub gas_yield_fraction: f64,
    /// Feedback mode: `none` or `mass`.
    #[serde(default = "default_ablation_feedback")]
    pub feedback: String,
}

impl AerothermalAblationConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_supported(
            &format!("{path}.virgin_material"),
            &self.virgin_material,
            &[
                "textbook_pica_like",
                "textbook_avcoat_like",
                "textbook_graphite",
            ],
        )?;
        require_supported(
            &format!("{path}.char_material"),
            &self.char_material,
            &["textbook_char", "textbook_graphite"],
        )?;
        require_positive(&format!("{path}.thickness_m"), self.thickness_m)?;
        if self.n_nodes < 5 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.n_nodes"),
                value: f64::from(self.n_nodes),
                rule: "must be >= 5",
            });
        }
        require_positive(
            &format!("{path}.pyrolysis_enthalpy_j_kg"),
            self.pyrolysis_enthalpy_j_kg,
        )?;
        validate_unit_interval(
            &format!("{path}.gas_yield_fraction"),
            self.gas_yield_fraction,
        )?;
        require_supported(
            &format!("{path}.feedback"),
            &self.feedback,
            &["none", "mass"],
        )?;
        Ok(())
    }
}

fn default_ablation_feedback() -> String {
    "none".to_owned()
}

/// Inline launch-vehicle continuum aero buildup configuration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroBuildupConfig {
    /// Maximum body diameter (m).
    pub body_diameter_m: f64,
    /// Total body length including nose and afterbody (m).
    pub body_length_m: f64,
    /// Surface roughness height (m).
    pub surface_roughness_m: f64,
    /// Coefficient reference area (m^2).
    pub reference_area_m2: f64,
    /// Moment reference length (m).
    pub reference_length_m: f64,
    /// Optional center of gravity from nose tip (m).
    #[serde(default)]
    pub center_of_gravity_from_nose_m: Option<f64>,
    /// Inclusive Mach grid range.
    pub mach_grid: AeroBuildupGridConfig,
    /// Inclusive angle-of-attack grid range in degrees.
    pub alpha_grid_deg: AeroBuildupGridConfig,
    /// Reference altitude used to compute bake-time Reynolds number.
    pub reference_altitude_m: f64,
    /// Nose / forebody declaration.
    pub nose: AeroBuildupNoseConfig,
    /// Optional afterbody taper.
    #[serde(default)]
    pub afterbody: Option<AeroBuildupAfterbodyConfig>,
    /// Optional fin set.
    #[serde(default)]
    pub fins: Option<AeroBuildupFinsConfig>,
}

impl AeroBuildupConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_positive(&format!("{path}.body_diameter_m"), self.body_diameter_m)?;
        require_positive(&format!("{path}.body_length_m"), self.body_length_m)?;
        if self.body_length_m <= self.body_diameter_m {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.body_length_m"),
                value: self.body_length_m,
                rule: "must be greater than body_diameter_m",
            });
        }
        require_finite(
            &format!("{path}.surface_roughness_m"),
            self.surface_roughness_m,
        )?;
        if self.surface_roughness_m < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.surface_roughness_m"),
                value: self.surface_roughness_m,
                rule: "must be non-negative",
            });
        }
        require_positive(&format!("{path}.reference_area_m2"), self.reference_area_m2)?;
        require_positive(
            &format!("{path}.reference_length_m"),
            self.reference_length_m,
        )?;
        if let Some(x_cg) = self.center_of_gravity_from_nose_m {
            require_finite(&format!("{path}.center_of_gravity_from_nose_m"), x_cg)?;
            if !(0.0..=self.body_length_m).contains(&x_cg) {
                return Err(ScenarioError::InvalidNumber {
                    field: format!("{path}.center_of_gravity_from_nose_m"),
                    value: x_cg,
                    rule: "must lie within [0, body_length_m]",
                });
            }
        }
        self.mach_grid.validate(&format!("{path}.mach_grid"))?;
        self.alpha_grid_deg
            .validate(&format!("{path}.alpha_grid_deg"))?;
        require_finite(
            &format!("{path}.reference_altitude_m"),
            self.reference_altitude_m,
        )?;
        if self.reference_altitude_m < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.reference_altitude_m"),
                value: self.reference_altitude_m,
                rule: "must be non-negative",
            });
        }
        self.nose.validate(&format!("{path}.nose"))?;
        if let Some(afterbody) = &self.afterbody {
            afterbody.validate(&format!("{path}.afterbody"), self.body_diameter_m)?;
        }
        if let Some(fins) = &self.fins {
            fins.validate(&format!("{path}.fins"), self.body_length_m)?;
        }
        Ok(())
    }
}

/// Inclusive scalar grid declaration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroBuildupGridConfig {
    /// Minimum grid value.
    pub min: f64,
    /// Maximum grid value.
    pub max: f64,
    /// Number of grid points.
    pub steps: u32,
}

impl AeroBuildupGridConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_finite(&format!("{path}.min"), self.min)?;
        require_finite(&format!("{path}.max"), self.max)?;
        if self.max < self.min {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.max"),
                value: self.max,
                rule: "must be greater than or equal to min",
            });
        }
        require_positive_u32(&format!("{path}.steps"), self.steps)?;
        Ok(())
    }
}

/// Nose declaration for the aero buildup.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroBuildupNoseConfig {
    /// Shape name: `conical`, `ogive`, `von_karman`, or `hemispherical`.
    pub shape: String,
    /// Nose fineness ratio for ogive or von Karman shapes.
    #[serde(default)]
    pub fineness: Option<f64>,
    /// Cone half-angle for conical shapes.
    #[serde(default)]
    pub half_angle_rad: Option<f64>,
}

impl AeroBuildupNoseConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        require_non_empty(&format!("{path}.shape"), &self.shape)?;
        match self.shape.as_str() {
            "conical" => {
                let angle =
                    self.half_angle_rad
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{path}.half_angle_rad"),
                            role: ModelRole::Force,
                            name: "conical".to_owned(),
                        })?;
                require_positive(&format!("{path}.half_angle_rad"), angle)?;
                if self.fineness.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: format!("{path}.fineness"),
                        role: ModelRole::Force,
                        name: "conical".to_owned(),
                    });
                }
            }
            "ogive" | "von_karman" => {
                let fineness =
                    self.fineness
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{path}.fineness"),
                            role: ModelRole::Force,
                            name: self.shape.clone(),
                        })?;
                require_positive(&format!("{path}.fineness"), fineness)?;
                if self.half_angle_rad.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: format!("{path}.half_angle_rad"),
                        role: ModelRole::Force,
                        name: self.shape.clone(),
                    });
                }
            }
            "hemispherical" => {
                if self.fineness.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: format!("{path}.fineness"),
                        role: ModelRole::Force,
                        name: "hemispherical".to_owned(),
                    });
                }
                if self.half_angle_rad.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: format!("{path}.half_angle_rad"),
                        role: ModelRole::Force,
                        name: "hemispherical".to_owned(),
                    });
                }
            }
            other => {
                return Err(ScenarioError::UnsupportedValue {
                    field: format!("{path}.shape"),
                    value: other.to_owned(),
                });
            }
        }
        Ok(())
    }
}

/// Afterbody taper declaration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroBuildupAfterbodyConfig {
    /// Exit diameter (m).
    pub exit_diameter_m: f64,
    /// Taper length (m).
    pub length_m: f64,
}

impl AeroBuildupAfterbodyConfig {
    fn validate(&self, path: &str, _body_diameter_m: f64) -> Result<(), ScenarioError> {
        require_positive(&format!("{path}.exit_diameter_m"), self.exit_diameter_m)?;
        require_positive(&format!("{path}.length_m"), self.length_m)?;
        Ok(())
    }
}

/// Fin-set declaration for the aero buildup.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AeroBuildupFinsConfig {
    /// Number of identical fins.
    pub count: u32,
    /// Root chord (m).
    pub root_chord_m: f64,
    /// Tip chord (m).
    pub tip_chord_m: f64,
    /// Semispan (m).
    pub span_m: f64,
    /// Thickness divided by chord.
    pub thickness_ratio: f64,
    /// Leading-edge sweep angle (rad).
    pub sweep_rad: f64,
}

impl AeroBuildupFinsConfig {
    fn validate(&self, path: &str, body_length_m: f64) -> Result<(), ScenarioError> {
        require_positive_u32(&format!("{path}.count"), self.count)?;
        require_positive(&format!("{path}.root_chord_m"), self.root_chord_m)?;
        require_finite(&format!("{path}.tip_chord_m"), self.tip_chord_m)?;
        if self.tip_chord_m < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.tip_chord_m"),
                value: self.tip_chord_m,
                rule: "must be non-negative",
            });
        }
        require_positive(&format!("{path}.span_m"), self.span_m)?;
        require_positive(&format!("{path}.thickness_ratio"), self.thickness_ratio)?;
        require_finite(&format!("{path}.sweep_rad"), self.sweep_rad)?;
        if self.root_chord_m > body_length_m {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.root_chord_m"),
                value: self.root_chord_m,
                rule: "must be less than or equal to body_length_m",
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
    /// Path to a motor TOML file (resolved relative to the
    /// scenario directory). Mutually exclusive with `grain`.
    #[serde(default)]
    pub file: Option<PathBuf>,
    /// Optional grain-regression producer. Mutually exclusive with
    /// `file`.
    #[serde(default)]
    pub grain: Option<MotorGrainConfig>,
    /// Ignition time in seconds since scenario start.
    pub ignite_at_s: f64,
    /// Optional owner body for post-separation force / mass routing.
    /// Required when `[multi_body]` is declared and `[propulsion.motor]`
    /// is present.
    #[serde(default)]
    pub mounted_to: Option<String>,
    /// Optional motor variant (defaults to whatever the motor file
    /// declares; when present, must match).
    pub variant: Option<String>,
    /// Optional pinned SHA-256 digest of the motor file.
    pub file_sha256: Option<String>,
}

impl MotorConfig {
    fn validate(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        match (&self.file, &self.grain) {
            (Some(_), Some(_)) => return Err(ScenarioError::AmbiguousPropulsion),
            (None, None) => {
                return Err(ScenarioError::MissingRequiredField {
                    field: "propulsion.motor.file_or_grain".to_owned(),
                    role: ModelRole::Motor,
                    name: "solid".to_owned(),
                });
            }
            (Some(file), None) => {
                if file.as_os_str().is_empty() {
                    return Err(ScenarioError::EmptyField {
                        field: "propulsion.motor.file".to_owned(),
                    });
                }
            }
            (None, Some(grain)) => {
                if self.file_sha256.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "propulsion.motor.file_sha256".to_owned(),
                        role: ModelRole::Motor,
                        name: "grain".to_owned(),
                    });
                }
                grain.validate()?;
            }
        }
        require_finite("propulsion.motor.ignite_at_s", self.ignite_at_s)?;
        if let Some(mounted_to) = &self.mounted_to {
            require_non_empty("propulsion.motor.mounted_to", mounted_to)?;
        }
        if let Some(variant) = &self.variant {
            registry.resolve(ModelRole::Motor, variant)?;
        }
        Ok(())
    }
}

/// Inline solid-grain regression configuration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MotorGrainConfig {
    /// Grain geometry kind.
    pub geometry: GrainGeometryConfig,
    /// Number of BATES segments.
    #[serde(default)]
    pub segments: Option<u32>,
    /// End-burner cross-section area (m²).
    #[serde(default)]
    pub cross_section_area_m2: Option<f64>,
    /// End-burner grain length (m).
    #[serde(default)]
    pub length_m: Option<f64>,
    /// BATES outer radius (m).
    #[serde(default)]
    pub outer_radius_m: Option<f64>,
    /// BATES initial core radius (m).
    #[serde(default)]
    pub core_radius_m: Option<f64>,
    /// BATES segment length (m).
    #[serde(default)]
    pub segment_length_m: Option<f64>,
    /// Tabulated `(web_m, burn_area_m2)` points.
    #[serde(default)]
    pub points: Option<Vec<[f64; 2]>>,
    /// Tabulated-grain propellant volume (m³).
    #[serde(default)]
    pub propellant_volume_m3: Option<f64>,
    /// Nozzle throat radius (m).
    pub throat_radius_m: f64,
    /// Nozzle expansion ratio `Ae / At`.
    pub expansion_ratio: f64,
    /// Optional dry case/nozzle mass (kg). Defaults to zero.
    #[serde(default)]
    pub dry_mass_kg: f64,
    /// Optional display name. Defaults to the propellant label.
    #[serde(default)]
    pub name: Option<String>,
    /// Optional provenance sentence.
    #[serde(default)]
    pub provenance: Option<String>,
    /// Propellant constants and fixed web-grid count.
    pub propellant: GrainPropellantConfig,
}

impl MotorGrainConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        self.validate_geometry_fields()?;
        require_finite(
            "propulsion.motor.grain.throat_radius_m",
            self.throat_radius_m,
        )?;
        require_positive(
            "propulsion.motor.grain.throat_radius_m",
            self.throat_radius_m,
        )?;
        require_finite(
            "propulsion.motor.grain.expansion_ratio",
            self.expansion_ratio,
        )?;
        if self.expansion_ratio < 1.0 {
            return Err(ScenarioError::InvalidNumber {
                field: "propulsion.motor.grain.expansion_ratio".to_owned(),
                value: self.expansion_ratio,
                rule: "must be at least 1",
            });
        }
        require_finite("propulsion.motor.grain.dry_mass_kg", self.dry_mass_kg)?;
        if self.dry_mass_kg < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: "propulsion.motor.grain.dry_mass_kg".to_owned(),
                value: self.dry_mass_kg,
                rule: "must be non-negative",
            });
        }
        if let Some(name) = &self.name {
            require_non_empty("propulsion.motor.grain.name", name)?;
        }
        if let Some(provenance) = &self.provenance {
            require_non_empty("propulsion.motor.grain.provenance", provenance)?;
        }
        self.propellant.validate()?;
        Ok(())
    }

    fn validate_geometry_fields(&self) -> Result<(), ScenarioError> {
        let prefix = "propulsion.motor.grain";
        match self.geometry {
            GrainGeometryConfig::EndBurner => {
                let cross_section_area_m2 = self.cross_section_area_m2.ok_or_else(|| {
                    ScenarioError::MissingRequiredField {
                        field: format!("{prefix}.cross_section_area_m2"),
                        role: ModelRole::Motor,
                        name: "end_burner".to_owned(),
                    }
                })?;
                let length_m =
                    self.length_m
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{prefix}.length_m"),
                            role: ModelRole::Motor,
                            name: "end_burner".to_owned(),
                        })?;
                require_finite(
                    &format!("{prefix}.cross_section_area_m2"),
                    cross_section_area_m2,
                )?;
                require_positive(
                    &format!("{prefix}.cross_section_area_m2"),
                    cross_section_area_m2,
                )?;
                require_finite(&format!("{prefix}.length_m"), length_m)?;
                require_positive(&format!("{prefix}.length_m"), length_m)?;
            }
            GrainGeometryConfig::Bates => {
                let segments =
                    self.segments
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{prefix}.segments"),
                            role: ModelRole::Motor,
                            name: "bates".to_owned(),
                        })?;
                let outer_radius_m =
                    self.outer_radius_m
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{prefix}.outer_radius_m"),
                            role: ModelRole::Motor,
                            name: "bates".to_owned(),
                        })?;
                let core_radius_m =
                    self.core_radius_m
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{prefix}.core_radius_m"),
                            role: ModelRole::Motor,
                            name: "bates".to_owned(),
                        })?;
                let segment_length_m =
                    self.segment_length_m
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{prefix}.segment_length_m"),
                            role: ModelRole::Motor,
                            name: "bates".to_owned(),
                        })?;
                if segments == 0 {
                    return Err(ScenarioError::InvalidNumber {
                        field: format!("{prefix}.segments"),
                        value: f64::from(segments),
                        rule: "must be positive",
                    });
                }
                require_finite(&format!("{prefix}.outer_radius_m"), outer_radius_m)?;
                require_positive(&format!("{prefix}.outer_radius_m"), outer_radius_m)?;
                require_finite(&format!("{prefix}.core_radius_m"), core_radius_m)?;
                require_positive(&format!("{prefix}.core_radius_m"), core_radius_m)?;
                if core_radius_m >= outer_radius_m {
                    return Err(ScenarioError::InvalidNumber {
                        field: format!("{prefix}.core_radius_m"),
                        value: core_radius_m,
                        rule: "must be smaller than outer_radius_m",
                    });
                }
                require_finite(&format!("{prefix}.segment_length_m"), segment_length_m)?;
                require_positive(&format!("{prefix}.segment_length_m"), segment_length_m)?;
            }
            GrainGeometryConfig::Tabulated => {
                let points =
                    self.points
                        .as_ref()
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: format!("{prefix}.points"),
                            role: ModelRole::Motor,
                            name: "tabulated".to_owned(),
                        })?;
                let propellant_volume_m3 = self.propellant_volume_m3.ok_or_else(|| {
                    ScenarioError::MissingRequiredField {
                        field: format!("{prefix}.propellant_volume_m3"),
                        role: ModelRole::Motor,
                        name: "tabulated".to_owned(),
                    }
                })?;
                if points.len() < 2 {
                    return Err(ScenarioError::EmptyList {
                        field: format!("{prefix}.points"),
                    });
                }
                require_finite(
                    &format!("{prefix}.propellant_volume_m3"),
                    propellant_volume_m3,
                )?;
                require_positive(
                    &format!("{prefix}.propellant_volume_m3"),
                    propellant_volume_m3,
                )?;
                for (index, point) in points.iter().enumerate() {
                    require_finite_array(&format!("{prefix}.points[{index}]"), point)?;
                    if point[0] < 0.0 || point[1] < 0.0 {
                        return Err(ScenarioError::InvalidNumber {
                            field: format!("{prefix}.points[{index}]"),
                            value: point[0].min(point[1]),
                            rule: "web and area values must be non-negative",
                        });
                    }
                    if index > 0 && point[0] <= points[index - 1][0] {
                        return Err(ScenarioError::InvalidNumber {
                            field: format!("{prefix}.points[{index}][0]"),
                            value: point[0],
                            rule: "web grid must be strictly increasing",
                        });
                    }
                }
            }
        }
        Ok(())
    }
}

/// Grain geometry kind for inline regression.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum GrainGeometryConfig {
    /// Constant-area end burner.
    EndBurner,
    /// BATES/tubular segments.
    Bates,
    /// User-supplied web/area table.
    Tabulated,
}

/// Grain propellant constants.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct GrainPropellantConfig {
    /// Synthetic/textbook label.
    pub label: String,
    /// Bulk density (kg/m³).
    pub density_kg_m3: f64,
    /// Saint-Robert coefficient, SI.
    pub burn_rate_a: f64,
    /// Saint-Robert pressure exponent.
    pub burn_rate_n: f64,
    /// Characteristic velocity (m/s).
    pub c_star_m_s: f64,
    /// Specific heat ratio.
    pub gamma: f64,
    /// Fixed web-grid count.
    pub web_steps: u32,
}

impl GrainPropellantConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("propulsion.motor.grain.propellant.{field}");
        require_non_empty(&path("label"), &self.label)?;
        require_finite(&path("density_kg_m3"), self.density_kg_m3)?;
        require_positive(&path("density_kg_m3"), self.density_kg_m3)?;
        require_finite(&path("burn_rate_a"), self.burn_rate_a)?;
        require_positive(&path("burn_rate_a"), self.burn_rate_a)?;
        require_finite(&path("burn_rate_n"), self.burn_rate_n)?;
        if !(0.0..1.0).contains(&self.burn_rate_n) {
            return Err(ScenarioError::InvalidNumber {
                field: path("burn_rate_n"),
                value: self.burn_rate_n,
                rule: "must lie in (0, 1)",
            });
        }
        require_finite(&path("c_star_m_s"), self.c_star_m_s)?;
        require_positive(&path("c_star_m_s"), self.c_star_m_s)?;
        require_finite(&path("gamma"), self.gamma)?;
        if self.gamma <= 1.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("gamma"),
                value: self.gamma,
                rule: "must be greater than 1",
            });
        }
        require_positive_u32(&path("web_steps"), self.web_steps)?;
        Ok(())
    }
}

/// Structured wind block.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct WindConfig {
    /// Wind model name (must match a registered wind model). Supported
    /// kinds are `"none"`, `"constant"`, `"layered"`, `"gust"`,
    /// and `"hwm14"`.
    pub kind: String,
    /// Constant wind in NED frame, m/s. Required when
    /// `kind = "constant"`; rejected for every other kind.
    pub wind_ned_m_s: Option<[f64; 3]>,
    /// Per-altitude NED wind table. Required when
    /// `kind = "layered"`; rejected for every other kind. Layers
    /// must be strictly ascending in altitude.
    #[serde(default)]
    pub layers: Option<Vec<WindLayerConfig>>,
    /// Dryden gust intensities `[σ_u, σ_v, σ_w]`, m/s.
    /// Required when `kind = "gust"`; rejected for every other kind.
    /// All non-negative.
    #[serde(default)]
    pub intensity_m_s: Option<[f64; 3]>,
    /// Dryden gust length scales `[L_u, L_v, L_w]`, m.
    /// Required when `kind = "gust"`; rejected for every other kind.
    /// All strictly positive.
    #[serde(default)]
    pub length_scale_m: Option<[f64; 3]>,
    /// Reference airspeed used to convert length scale
    /// to time scale, m/s. Required when `kind = "gust"`; rejected
    /// for every other kind. Strictly positive.
    #[serde(default)]
    pub airspeed_m_s: Option<f64>,
    /// Optional mean wind in NED, m/s. Defaults to
    /// `[0, 0, 0]`. Accepted only when `kind = "gust"`.
    #[serde(default)]
    pub mean_wind_ned_m_s: Option<[f64; 3]>,
    /// Calendar year for `kind = "hwm14"`; defaults to 1995.
    #[serde(default)]
    pub year: Option<u16>,
    /// Day of year for `kind = "hwm14"`; defaults to the public
    /// `checkhwm14` height-profile day, 150. Day 0 is accepted to
    /// match the public verification driver.
    #[serde(default)]
    pub day_of_year: Option<u16>,
    /// UTC seconds within the day for `kind = "hwm14"`; defaults to noon.
    #[serde(default)]
    pub utc_s: Option<f64>,
    /// Geodetic latitude in degrees for `kind = "hwm14"`; defaults to -45.
    #[serde(default)]
    pub latitude_deg: Option<f64>,
    /// Geodetic longitude in degrees for `kind = "hwm14"`; defaults to -85.
    #[serde(default)]
    pub longitude_deg: Option<f64>,
    /// Current 3-hour Ap index for `kind = "hwm14"`.
    /// Defaults to 80. Use `-1` for quiet-time winds only.
    #[serde(default)]
    pub ap_current_3h: Option<f64>,
}

impl WindConfig {
    #[allow(clippy::too_many_lines)] // kind = "gust" adds cross-field rejection arms.
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
                self.reject_gust_fields("constant")?;
                self.reject_hwm14_fields("constant")?;
            }
            "layered" => {
                if self.wind_ned_m_s.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "wind.wind_ned_m_s".to_owned(),
                        role: ModelRole::Wind,
                        name: "layered".to_owned(),
                    });
                }
                self.reject_gust_fields("layered")?;
                self.reject_hwm14_fields("layered")?;
                let layers =
                    self.layers
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
            "gust" => {
                if self.wind_ned_m_s.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "wind.wind_ned_m_s".to_owned(),
                        role: ModelRole::Wind,
                        name: "gust".to_owned(),
                    });
                }
                if self.layers.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "wind.layers".to_owned(),
                        role: ModelRole::Wind,
                        name: "gust".to_owned(),
                    });
                }
                let intensity =
                    self.intensity_m_s
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: "wind.intensity_m_s".to_owned(),
                            role: ModelRole::Wind,
                            name: "gust".to_owned(),
                        })?;
                require_finite_array("wind.intensity_m_s", &intensity)?;
                for (axis, value) in ['u', 'v', 'w'].iter().zip(intensity.iter()) {
                    if *value < 0.0 {
                        return Err(ScenarioError::InvalidNumber {
                            field: format!("wind.intensity_m_s[{axis}]"),
                            value: *value,
                            rule: "must be non-negative",
                        });
                    }
                }
                let length_scale =
                    self.length_scale_m
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: "wind.length_scale_m".to_owned(),
                            role: ModelRole::Wind,
                            name: "gust".to_owned(),
                        })?;
                require_finite_array("wind.length_scale_m", &length_scale)?;
                for (axis, value) in ['u', 'v', 'w'].iter().zip(length_scale.iter()) {
                    if *value <= 0.0 {
                        return Err(ScenarioError::InvalidNumber {
                            field: format!("wind.length_scale_m[{axis}]"),
                            value: *value,
                            rule: "must be strictly positive",
                        });
                    }
                }
                let airspeed =
                    self.airspeed_m_s
                        .ok_or_else(|| ScenarioError::MissingRequiredField {
                            field: "wind.airspeed_m_s".to_owned(),
                            role: ModelRole::Wind,
                            name: "gust".to_owned(),
                        })?;
                require_finite("wind.airspeed_m_s", airspeed)?;
                if airspeed <= 0.0 {
                    return Err(ScenarioError::InvalidNumber {
                        field: "wind.airspeed_m_s".to_owned(),
                        value: airspeed,
                        rule: "must be strictly positive",
                    });
                }
                if let Some(mean) = self.mean_wind_ned_m_s {
                    require_finite_array("wind.mean_wind_ned_m_s", &mean)?;
                }
                self.reject_hwm14_fields("gust")?;
            }
            "hwm14" => {
                if self.wind_ned_m_s.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "wind.wind_ned_m_s".to_owned(),
                        role: ModelRole::Wind,
                        name: "hwm14".to_owned(),
                    });
                }
                if self.layers.is_some() {
                    return Err(ScenarioError::UnexpectedField {
                        field: "wind.layers".to_owned(),
                        role: ModelRole::Wind,
                        name: "hwm14".to_owned(),
                    });
                }
                self.reject_gust_fields("hwm14")?;
                self.validate_hwm14_fields()?;
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
                self.reject_gust_fields(other)?;
                self.reject_hwm14_fields(other)?;
            }
        }
        Ok(())
    }

    fn reject_gust_fields(&self, kind: &str) -> Result<(), ScenarioError> {
        if self.intensity_m_s.is_some() {
            return Err(ScenarioError::UnexpectedField {
                field: "wind.intensity_m_s".to_owned(),
                role: ModelRole::Wind,
                name: kind.to_owned(),
            });
        }
        if self.length_scale_m.is_some() {
            return Err(ScenarioError::UnexpectedField {
                field: "wind.length_scale_m".to_owned(),
                role: ModelRole::Wind,
                name: kind.to_owned(),
            });
        }
        if self.airspeed_m_s.is_some() {
            return Err(ScenarioError::UnexpectedField {
                field: "wind.airspeed_m_s".to_owned(),
                role: ModelRole::Wind,
                name: kind.to_owned(),
            });
        }
        if self.mean_wind_ned_m_s.is_some() {
            return Err(ScenarioError::UnexpectedField {
                field: "wind.mean_wind_ned_m_s".to_owned(),
                role: ModelRole::Wind,
                name: kind.to_owned(),
            });
        }
        Ok(())
    }

    fn validate_hwm14_fields(&self) -> Result<(), ScenarioError> {
        if let Some(day_of_year) = self.day_of_year
            && day_of_year > 366
        {
            return Err(ScenarioError::InvalidNumber {
                field: "wind.day_of_year".to_owned(),
                value: f64::from(day_of_year),
                rule: "must be in 0..=366",
            });
        }
        if let Some(utc_s) = self.utc_s {
            require_finite("wind.utc_s", utc_s)?;
            if !(0.0..86_400.0).contains(&utc_s) {
                return Err(ScenarioError::InvalidNumber {
                    field: "wind.utc_s".to_owned(),
                    value: utc_s,
                    rule: "must be in [0, 86400)",
                });
            }
        }
        if let Some(latitude_deg) = self.latitude_deg {
            require_finite("wind.latitude_deg", latitude_deg)?;
            if !(-90.0..=90.0).contains(&latitude_deg) {
                return Err(ScenarioError::InvalidNumber {
                    field: "wind.latitude_deg".to_owned(),
                    value: latitude_deg,
                    rule: "must be in [-90, 90]",
                });
            }
        }
        if let Some(longitude_deg) = self.longitude_deg {
            require_finite("wind.longitude_deg", longitude_deg)?;
            if !(-180.0..=180.0).contains(&longitude_deg) {
                return Err(ScenarioError::InvalidNumber {
                    field: "wind.longitude_deg".to_owned(),
                    value: longitude_deg,
                    rule: "must be in [-180, 180]",
                });
            }
        }
        if let Some(ap_current_3h) = self.ap_current_3h {
            require_finite("wind.ap_current_3h", ap_current_3h)?;
            if !((-1.0..=400.0).contains(&ap_current_3h)) {
                return Err(ScenarioError::InvalidNumber {
                    field: "wind.ap_current_3h".to_owned(),
                    value: ap_current_3h,
                    rule: "must be -1 or in [0, 400]",
                });
            }
        }
        Ok(())
    }

    fn reject_hwm14_fields(&self, kind: &str) -> Result<(), ScenarioError> {
        if self.year.is_some()
            || self.day_of_year.is_some()
            || self.utc_s.is_some()
            || self.latitude_deg.is_some()
            || self.longitude_deg.is_some()
            || self.ap_current_3h.is_some()
        {
            return Err(ScenarioError::UnexpectedField {
                field:
                    "wind.year / day_of_year / utc_s / latitude_deg / longitude_deg / ap_current_3h"
                        .to_owned(),
                role: ModelRole::Wind,
                name: kind.to_owned(),
            });
        }
        Ok(())
    }
}

/// One row of a `[[wind.layers]]` table.
///
/// `altitude_m` is metres above the launch-pad reference (matching
/// the axial-drag adapter convention). `wind_ned_m_s` is
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
    /// Calendar year for MSIS-family atmosphere kinds; defaults to 2024.
    pub year: Option<u16>,
    /// Day of year for MSIS-family atmosphere kinds; defaults to 80.
    pub day_of_year: Option<u16>,
    /// UTC seconds within the day for MSIS-family kinds; defaults to noon.
    pub utc_s: Option<f64>,
    /// Geodetic latitude in degrees for MSIS-family kinds; defaults to 0.
    pub latitude_deg: Option<f64>,
    /// Geodetic longitude in degrees for MSIS-family kinds; defaults to 0.
    pub longitude_deg: Option<f64>,
    /// Local apparent solar time in hours for MSIS-family kinds; defaults to 12.
    pub local_apparent_solar_time_h: Option<f64>,
    /// 81-day average F10.7 solar flux for MSIS-family kinds; defaults to 150.
    pub f107_average_81day_sfu: Option<f64>,
    /// Previous-day F10.7 solar flux for MSIS-family kinds; defaults to 150.
    pub f107_yesterday_sfu: Option<f64>,
    /// Daily Ap geomagnetic index for MSIS-family kinds; defaults to 4.
    pub ap_average: Option<f64>,
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
                self.reject_nrlmsise00_fields("isothermal")?;
            }
            "nrlmsise00" => {
                self.reject_isothermal_fields("nrlmsise00")?;
                self.validate_nrlmsise00_fields()?;
            }
            "nrlmsis2_compat" => {
                self.reject_isothermal_fields("nrlmsis2_compat")?;
                self.validate_nrlmsise00_fields()?;
            }
            other => {
                self.reject_isothermal_fields(other)?;
                self.reject_nrlmsise00_fields(other)?;
            }
        }
        Ok(())
    }

    fn validate_nrlmsise00_fields(&self) -> Result<(), ScenarioError> {
        if let Some(day_of_year) = self.day_of_year
            && !(1..=366).contains(&day_of_year)
        {
            return Err(ScenarioError::InvalidNumber {
                field: "atmosphere.day_of_year".to_owned(),
                value: f64::from(day_of_year),
                rule: "must be in 1..=366",
            });
        }
        if let Some(utc_s) = self.utc_s {
            require_finite("atmosphere.utc_s", utc_s)?;
            if !(0.0..86_400.0).contains(&utc_s) {
                return Err(ScenarioError::InvalidNumber {
                    field: "atmosphere.utc_s".to_owned(),
                    value: utc_s,
                    rule: "must be in [0, 86400)",
                });
            }
        }
        if let Some(latitude_deg) = self.latitude_deg {
            require_finite("atmosphere.latitude_deg", latitude_deg)?;
            if !(-90.0..=90.0).contains(&latitude_deg) {
                return Err(ScenarioError::InvalidNumber {
                    field: "atmosphere.latitude_deg".to_owned(),
                    value: latitude_deg,
                    rule: "must be in [-90, 90]",
                });
            }
        }
        if let Some(longitude_deg) = self.longitude_deg {
            require_finite("atmosphere.longitude_deg", longitude_deg)?;
            if !(-180.0..=180.0).contains(&longitude_deg) {
                return Err(ScenarioError::InvalidNumber {
                    field: "atmosphere.longitude_deg".to_owned(),
                    value: longitude_deg,
                    rule: "must be in [-180, 180]",
                });
            }
        }
        if let Some(local_solar_time) = self.local_apparent_solar_time_h {
            require_finite("atmosphere.local_apparent_solar_time_h", local_solar_time)?;
            if !(0.0..24.0).contains(&local_solar_time) {
                return Err(ScenarioError::InvalidNumber {
                    field: "atmosphere.local_apparent_solar_time_h".to_owned(),
                    value: local_solar_time,
                    rule: "must be in [0, 24)",
                });
            }
        }
        if let Some(f107_average) = self.f107_average_81day_sfu {
            require_positive("atmosphere.f107_average_81day_sfu", f107_average)?;
        }
        if let Some(f107_yesterday_sfu) = self.f107_yesterday_sfu {
            require_positive("atmosphere.f107_yesterday_sfu", f107_yesterday_sfu)?;
        }
        if let Some(ap_average) = self.ap_average {
            require_finite("atmosphere.ap_average", ap_average)?;
            if ap_average < 0.0 {
                return Err(ScenarioError::InvalidNumber {
                    field: "atmosphere.ap_average".to_owned(),
                    value: ap_average,
                    rule: "must be non-negative",
                });
            }
        }
        Ok(())
    }

    fn reject_isothermal_fields(&self, kind: &str) -> Result<(), ScenarioError> {
        if self.density_kg_m3.is_some()
            || self.pressure_pa.is_some()
            || self.temperature_k.is_some()
        {
            return Err(ScenarioError::UnexpectedField {
                field: "atmosphere.density_kg_m3 / pressure_pa / temperature_k".to_owned(),
                role: ModelRole::Atmosphere,
                name: kind.to_owned(),
            });
        }
        Ok(())
    }

    fn reject_nrlmsise00_fields(&self, kind: &str) -> Result<(), ScenarioError> {
        if self.year.is_some()
            || self.day_of_year.is_some()
            || self.utc_s.is_some()
            || self.latitude_deg.is_some()
            || self.longitude_deg.is_some()
            || self.local_apparent_solar_time_h.is_some()
            || self.f107_average_81day_sfu.is_some()
            || self.f107_yesterday_sfu.is_some()
            || self.ap_average.is_some()
        {
            return Err(ScenarioError::UnexpectedField {
                field: "atmosphere.year / day_of_year / utc_s / latitude_deg / longitude_deg / local_apparent_solar_time_h / f107_average_81day_sfu / f107_yesterday_sfu / ap_average".to_owned(),
                role: ModelRole::Atmosphere,
                name: kind.to_owned(),
            });
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
    /// Path to the sensor's noise-budget TOML file.
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
// Mission block
// ---------------------------------------------------------------------

/// Top-level `[mission]` block.
///
/// The v4 fields (`states`, `regions`, `scope`) are accepted
/// optionally and default to empty / unset. v3 scenarios continue to
/// parse byte-identically because the v4 fields are `#[serde(default)]`
/// and ignored when empty. A v3 → v4 lifting pass promotes
/// `phases` to `states`, preserving every path-derived id.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MissionConfig {
    /// Id of the initial phase / state.
    pub initial_phase: String,
    /// Declared phases (v3 flat-DAG vocabulary).
    #[serde(default)]
    pub phases: Vec<PhaseConfig>,
    /// Declared events.
    #[serde(default)]
    pub events: Vec<EventConfig>,
    /// Declared transitions between phases.
    #[serde(default)]
    pub transitions: Vec<PhaseTransitionConfig>,
    /// Declared hierarchical states. When non-empty, the
    /// parser uses these in preference to `phases` and applies
    /// hierarchical-state-machine validation. v3 scenarios omit this
    /// field; parse is byte-identical.
    #[serde(default)]
    pub states: Vec<StateConfig>,
    /// Declared orthogonal regions. Defaults to the four
    /// canonical regions (`mission`, `health`, `comms`,
    /// `estimator_regime`) when omitted.
    #[serde(default)]
    pub regions: Vec<RegionConfig>,
    /// Scenario-scope human-readable tag (e.g.
    /// `"sounding_rocket"`, `"propulsive_landing"`,
    /// `"orbital_insertion"`). Used only for telemetry-tag breadcrumbs;
    /// does not gate any behaviour.
    #[serde(default)]
    pub scope: Option<MissionScope>,
    /// Test-only override channel activation flag. When
    /// `true`, the simulator may write the
    /// `commander.scenario_state_override` topic to force the
    /// commander into a specific state for validation. Refused by HAL
    /// builds via the `openbmp-fc` `hal` feature gate.
    #[serde(default)]
    pub test_only_state_override: bool,
}

/// Scenario-scope classifier.
///
/// Accepts both the documented table form
/// `[mission.scope] kind = "sounding_rocket"` and the compact form
/// `scope = "sounding_rocket"`. Used only for human-readable
/// telemetry tags; does not gate behaviour.
#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(untagged)]
pub enum MissionScope {
    /// Compact string form.
    Kind(MissionScopeKind),
    /// Table form with a `kind` field.
    Config(MissionScopeConfig),
}

/// `[mission.scope]` table form.
#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct MissionScopeConfig {
    /// Scope kind.
    pub kind: MissionScopeKind,
}

/// Scenario-scope kind.
#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MissionScopeKind {
    /// Sounding rocket profile (Niskanen-2009 chapter-6 reference).
    SoundingRocket,
    /// Propulsive-landing profile (Calisto-class reference).
    PropulsiveLanding,
    /// Orbital insertion profile (LEO targets).
    OrbitalInsertion,
    /// Re-entry profile (hypersonic reference).
    ReEntry,
    /// Closed-loop FC test profile (no specific mission shape).
    ClosedLoopTest,
}

/// One hierarchical state declaration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct StateConfig {
    /// State id (canonical path or bare).
    pub id: String,
    /// Human-readable label.
    #[serde(default)]
    pub label: String,
    /// Parent state id, omitted for top-level states.
    #[serde(default)]
    pub parent: Option<String>,
    /// Effectors allowed while this state is active.
    #[serde(default)]
    pub allowed_effectors: Vec<String>,
    /// Engines allowed while this state is active.
    #[serde(default)]
    pub allowed_engines: Vec<String>,
    /// Actions fired in declaration order when the state is entered.
    /// Restricted to mission-vocabulary actions
    /// (`enter_state`, `emit_telemetry_marker`, `stop`).
    #[serde(default)]
    pub on_entry: Vec<ScenarioActionConfig>,
    /// Actions fired in declaration order when the state is exited.
    #[serde(default)]
    pub on_exit: Vec<ScenarioActionConfig>,
    /// Actions fired in declaration order on every tick the state is
    /// active.
    #[serde(default)]
    pub on_active: Vec<ScenarioActionConfig>,
}

impl StateConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        require_non_empty(&format!("mission.states[{index}].id"), &self.id)?;
        for (action_index, action) in self.on_entry.iter().enumerate() {
            action.validate(index)?;
            require_mission_only_action(
                &format!("mission.states[{index}].on_entry[{action_index}]"),
                action,
            )?;
        }
        for (action_index, action) in self.on_exit.iter().enumerate() {
            action.validate(index)?;
            require_mission_only_action(
                &format!("mission.states[{index}].on_exit[{action_index}]"),
                action,
            )?;
        }
        for (action_index, action) in self.on_active.iter().enumerate() {
            action.validate(index)?;
            require_mission_only_action(
                &format!("mission.states[{index}].on_active[{action_index}]"),
                action,
            )?;
        }
        Ok(())
    }
}

/// One orthogonal region declaration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct RegionConfig {
    /// Region id (canonical path or bare — e.g. `"health"` resolves
    /// to `"mission.regions.health"`).
    pub id: String,
    /// Region's initial state id.
    pub initial_state: String,
    /// States in this region's state machine.
    #[serde(default)]
    pub states: Vec<RegionStateConfig>,
    /// Transitions in this region's state machine.
    #[serde(default)]
    pub transitions: Vec<PhaseTransitionConfig>,
}

impl RegionConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        require_non_empty(&format!("mission.regions[{index}].id"), &self.id)?;
        require_non_empty(
            &format!("mission.regions[{index}].initial_state"),
            &self.initial_state,
        )?;
        if self.states.is_empty() {
            return Err(ScenarioError::EmptyList {
                field: format!("mission.regions[{index}].states"),
            });
        }
        let state_ids: Vec<String> = self.states.iter().map(|state| state.id.clone()).collect();
        require_unique(&format!("mission.regions[{index}].states.id"), &state_ids)?;
        let state_set: BTreeSet<&str> = state_ids.iter().map(String::as_str).collect();
        if !state_set.contains(self.initial_state.as_str()) {
            return Err(ScenarioError::MissionGraph {
                reason: format!(
                    "mission.regions[{index}].initial_state = `{}` is not a declared region state",
                    self.initial_state
                ),
            });
        }
        for (state_index, state) in self.states.iter().enumerate() {
            state.validate(index, state_index)?;
        }
        for (transition_index, transition) in self.transitions.iter().enumerate() {
            transition.validate(transition_index)?;
            if !state_set.contains(transition.from.as_str()) {
                return Err(ScenarioError::MissionGraph {
                    reason: format!(
                        "mission.regions[{index}].transitions[{transition_index}].from = `{}` is not a declared region state",
                        transition.from
                    ),
                });
            }
            if !state_set.contains(transition.to.as_str()) {
                return Err(ScenarioError::MissionGraph {
                    reason: format!(
                        "mission.regions[{index}].transitions[{transition_index}].to = `{}` is not a declared region state",
                        transition.to
                    ),
                });
            }
        }
        Ok(())
    }
}

/// One region state. Region states are flat (the
/// `mission` region is the only one with hierarchy).
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct RegionStateConfig {
    /// State id within this region.
    pub id: String,
    /// Human-readable label.
    #[serde(default)]
    pub label: String,
}

impl RegionStateConfig {
    fn validate(&self, region_index: usize, state_index: usize) -> Result<(), ScenarioError> {
        require_non_empty(
            &format!("mission.regions[{region_index}].states[{state_index}].id"),
            &self.id,
        )?;
        Ok(())
    }
}

fn require_mission_only_action(
    field: &str,
    action: &ScenarioActionConfig,
) -> Result<(), ScenarioError> {
    match action {
        ScenarioActionConfig::EnterPhase { .. }
        | ScenarioActionConfig::EmitTelemetryMarker { .. }
        | ScenarioActionConfig::Stop { .. }
        | ScenarioActionConfig::RaiseHealthAlarm { .. }
        | ScenarioActionConfig::SetRegionState { .. }
        | ScenarioActionConfig::RequestSafeState { .. } => Ok(()),
        ScenarioActionConfig::EffectorOverride { .. }
        | ScenarioActionConfig::EngineCommand { .. }
        | ScenarioActionConfig::Separation
        | ScenarioActionConfig::JettisonStage { .. }
        | ScenarioActionConfig::JettisonBodies { .. }
        | ScenarioActionConfig::SelectGuidanceProfile { .. }
        | ScenarioActionConfig::DeployRecovery { .. } => Err(ScenarioError::MissionGraph {
            reason: format!("{field} may contain only mission actions"),
        }),
    }
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
        if self.phases.is_empty() && self.states.is_empty() {
            return Err(ScenarioError::EmptyList {
                field: "mission.phases|mission.states".to_owned(),
            });
        }
        if !self.phases.is_empty() && !self.states.is_empty() {
            return Err(ScenarioError::MissionGraph {
                reason: "mission may declare either phases or states, not both".to_owned(),
            });
        }
        if self.states.is_empty() {
            for (i, phase) in self.phases.iter().enumerate() {
                phase.validate(i)?;
            }
        } else {
            for (i, state) in self.states.iter().enumerate() {
                state.validate(i)?;
            }
        }
        for (i, event) in self.events.iter().enumerate() {
            event.validate(i)?;
        }
        for (i, transition) in self.transitions.iter().enumerate() {
            transition.validate(i)?;
        }
        for (i, region) in self.regions.iter().enumerate() {
            region.validate(i)?;
        }
        self.validate_graph_shape()?;
        Ok(())
    }

    fn validate_graph_shape(&self) -> Result<(), ScenarioError> {
        let phase_ids: Vec<String> = if self.states.is_empty() {
            self.phases.iter().map(|phase| phase.id.clone()).collect()
        } else {
            self.states.iter().map(|state| state.id.clone()).collect()
        };
        let event_ids: Vec<String> = self.events.iter().map(|event| event.id.clone()).collect();
        require_unique("mission.phase_or_state.id", &phase_ids)?;
        require_unique("mission.events.id", &event_ids)?;

        let phase_set: BTreeSet<&str> = phase_ids.iter().map(String::as_str).collect();
        let event_set: BTreeSet<&str> = event_ids.iter().map(String::as_str).collect();

        if !self.states.is_empty() {
            for (index, state) in self.states.iter().enumerate() {
                if let Some(parent) = &state.parent
                    && !phase_set.contains(parent.as_str())
                {
                    return Err(ScenarioError::MissionGraph {
                        reason: format!(
                            "mission.states[{index}].parent = `{parent}` is not a declared state"
                        ),
                    });
                }
            }
        }

        if !phase_set.contains(self.initial_phase.as_str()) {
            return Err(ScenarioError::MissionGraph {
                reason: format!(
                    "mission.initial_phase = `{}` does not match any declared phase/state",
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
    /// active. References are validated; active-phase command
    /// gating is not enforced.
    #[serde(default)]
    pub allowed_effectors: Vec<String>,
    /// Optional list of engine ids permitted while this phase is
    /// active. Informational only.
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
    pub action: ScenarioActionConfig,
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
/// deferral error.
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
    /// Closed-loop guidance time-to-go drops to/through a threshold —
    /// schedules engine cutoff at orbital insertion (PEG).
    AtGuidanceCutoff {
        /// Time-to-go threshold (s).
        time_to_go_s: f64,
    },
    /// Mass-fraction crossing (current/initial mass below threshold).
    AtMassFraction {
        /// Threshold mass fraction in `[0, 1]`.
        remaining: f64,
    },
    /// Speed-magnitude crossing through a target velocity.
    AtVelocity {
        /// Speed threshold (m/s).
        velocity_m_s: f64,
        /// `false`: rising-edge crossing. `true`: falling-edge.
        #[serde(default)]
        falling: bool,
    },
    /// Dynamic-pressure crossing.
    AtDynamicPressure {
        /// Threshold dynamic pressure (Pa).
        pressure_pa: f64,
        /// `false`: rising-edge crossing. `true`: falling-edge.
        falling: bool,
    },
    /// Relative distance crossing between a body and the current
    /// primary lane, or another body if `reference_body` is supplied.
    AtRelativeDistance {
        /// Assembly body id to monitor.
        body: String,
        /// Optional reference assembly body id. When omitted, the
        /// current primary rigid-body lane is used.
        #[serde(default)]
        reference_body: Option<String>,
        /// Distance threshold (m).
        distance_m: f64,
        /// `false`: rising-edge crossing. `true`: falling-edge.
        #[serde(default)]
        falling: bool,
    },
    /// Relative speed crossing between a body and the current primary
    /// lane, or another body if `reference_body` is supplied.
    AtRelativeSpeed {
        /// Assembly body id to monitor.
        body: String,
        /// Optional reference assembly body id. When omitted, the
        /// current primary rigid-body lane is used.
        #[serde(default)]
        reference_body: Option<String>,
        /// Relative-speed threshold (m/s).
        speed_m_s: f64,
        /// `false`: rising-edge crossing. `true`: falling-edge.
        #[serde(default)]
        falling: bool,
    },
    /// Deferred: rejected at parse time.
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
            Self::AtGuidanceCutoff { time_to_go_s } => {
                require_non_negative(&path("time_to_go_s"), *time_to_go_s)?;
            }
            Self::AtMassFraction { remaining } => {
                require_finite(&path("remaining"), *remaining)?;
                require_in_range(&path("remaining"), *remaining, 0.0, 1.0)?;
            }
            Self::AtVelocity { velocity_m_s, .. } => {
                require_positive(&path("velocity_m_s"), *velocity_m_s)?;
            }
            Self::AtDynamicPressure { pressure_pa, .. } => {
                require_non_negative(&path("pressure_pa"), *pressure_pa)?;
            }
            Self::AtRelativeDistance {
                body,
                reference_body,
                distance_m,
                ..
            } => {
                require_non_empty(&path("body"), body)?;
                if let Some(reference_body) = reference_body {
                    require_non_empty(&path("reference_body"), reference_body)?;
                }
                require_positive(&path("distance_m"), *distance_m)?;
            }
            Self::AtRelativeSpeed {
                body,
                reference_body,
                speed_m_s,
                ..
            } => {
                require_non_empty(&path("body"), body)?;
                if let Some(reference_body) = reference_body {
                    require_non_empty(&path("reference_body"), reference_body)?;
                }
                require_positive(&path("speed_m_s"), *speed_m_s)?;
            }
            Self::Scripted => {
                return Err(ScenarioError::UnsupportedTriggerKind {
                    kind: "scripted".to_owned(),
                    reason: "scripted triggers are not yet supported; use effector command_schedule for deterministic actuator scripts".to_owned(),
                });
            }
        }
        Ok(())
    }
}

/// Action taken when an event fires. Tagged enum dispatched on the
/// `kind` string.
///
/// `separation` is rejected at parse time with a typed deferral
/// error pointing at the future phase that will land it.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ScenarioActionConfig {
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
    /// Declarative health-region demotion. The commander
    /// reads the fire and applies it to the named region (canonically
    /// `mission.regions.health`).
    RaiseHealthAlarm {
        /// Target region id (canonical or bare; bare resolves to
        /// `mission.regions.<id>`). Defaults to
        /// `mission.regions.health` when omitted.
        #[serde(default)]
        region: Option<String>,
        /// Canonical alarm code; non-zero values demote the
        /// `health` region to `abort_requested`.
        alarm: u32,
    },
    /// Declaratively set an orthogonal region state. Used by mission
    /// HSM entry/exit actions to publish regime signals such as
    /// `estimator_regime.boost_mode`.
    SetRegionState {
        /// Target region id (canonical or bare; bare resolves to
        /// `mission.regions.<region>`).
        region: String,
        /// Target state id (canonical or bare; bare resolves under
        /// the target region).
        state: String,
    },
    /// Declarative safe-state request. Symmetric to
    /// `raise_health_alarm` but does not require the scenario author
    /// to pick a region or alarm code; the commander applies a
    /// generic `health → abort_requested` demotion and publishes the
    /// reason on telemetry.
    RequestSafeState {
        /// Human-readable reason published alongside the request.
        reason: String,
    },
    /// Per-engine command targeting a declared
    /// `[[vehicle.assembly.engines]]` by id. The kernel records the
    /// firing; the runner-side `EngineRack` drains and applies the
    /// command on the next rack tick.
    EngineCommand {
        /// Target engine id (must reference a declared engine).
        id: String,
        /// Command payload.
        command: EngineCommandConfig,
    },
    /// Scenario-driven effector command override. Targets
    /// a declared `[[vehicle.assembly.effectors]]` by id and commits
    /// `command` at fire time (single-shot).
    EffectorOverride {
        /// Target effector id (must reference a declared effector).
        id: String,
        /// Command value (post-fault, pre-clamp).
        command: f64,
    },
    /// Deferred.
    Separation,
    /// Commanded multi-body stage separation: jettison the named
    /// assembly body. Distinct from the legacy bare `separation` in
    /// that it records *which* body departs, so the spent stage can be
    /// propagated and its range-safety footprint reported. See
    /// `docs/staging-and-separation.md`.
    JettisonStage {
        /// Assembly body id to jettison (must reference a declared
        /// `[[vehicle.assembly.bodies]]`).
        body: String,
    },
    /// Commanded coordinated multi-body separation: jettison every
    /// named assembly body from the same pre-separation state.
    JettisonBodies {
        /// Assembly body ids to jettison.
        bodies: Vec<String>,
    },
    /// Deferred. Switch the active guidance profile at a phase boundary
    /// (e.g. hand off from `ascent_reference` to passive `coast`). See
    /// `docs/ascent-guidance.md`.
    SelectGuidanceProfile {
        /// Guidance profile id to activate.
        profile: String,
    },
    /// Deploy / stow command targeting a declared
    /// `[[vehicle.assembly.recovery]]` device by id. The command
    /// string must be one of `"deploy"`, `"deploy_drogue"`,
    /// `"deploy_main"`, or `"stow"`; scenario validation checks the
    /// (kind, command) pairing and the runner-side `RecoveryRack`
    /// defensively rejects unsupported transitions with a typed error.
    DeployRecovery {
        /// Target recovery-device id (must reference a declared
        /// `[[vehicle.assembly.recovery]]` block).
        id: String,
        /// Command name (one of `"deploy"`, `"deploy_drogue"`,
        /// `"deploy_main"`, `"stow"`).
        command: String,
    },
}

impl ScenarioActionConfig {
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
            Self::RaiseHealthAlarm { region, alarm: _ } => {
                if let Some(region) = region {
                    require_non_empty(&path("region"), region)?;
                }
            }
            Self::SetRegionState { region, state } => {
                require_non_empty(&path("region"), region)?;
                require_non_empty(&path("state"), state)?;
            }
            Self::RequestSafeState { reason } => {
                require_non_empty(&path("reason"), reason)?;
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
                    missing_capability: "scripted stage separation".to_owned(),
                });
            }
            Self::JettisonStage { body } => {
                require_non_empty(&path("body"), body)?;
            }
            Self::JettisonBodies { bodies } => {
                require_non_empty_list(&path("bodies"), bodies)?;
                require_unique(&path("bodies"), bodies)?;
                for (body_index, body) in bodies.iter().enumerate() {
                    require_non_empty(&path(&format!("bodies[{body_index}]")), body)?;
                }
            }
            Self::SelectGuidanceProfile { profile } => {
                require_non_empty(&path("profile"), profile)?;
                return Err(ScenarioError::UnsupportedActionKind {
                    kind: "select_guidance_profile".to_owned(),
                    missing_capability: "runtime guidance-profile switching".to_owned(),
                });
            }
            Self::DeployRecovery { id, command } => {
                require_non_empty(&path("id"), id)?;
                require_recovery_command_name(&path("command"), command)?;
            }
        }
        Ok(())
    }
}

/// Validate that a `deploy_recovery.command` value is one of the
/// canonical names accepted by `openbmp_vehicle::RecoveryCommand`.
fn require_recovery_command_name(path: &str, command: &str) -> Result<(), ScenarioError> {
    match command {
        "deploy" | "deploy_drogue" | "deploy_main" | "stow" => Ok(()),
        other => Err(ScenarioError::UnsupportedValue {
            field: path.to_owned(),
            value: other.to_owned(),
        }),
    }
}

/// Returns `true` when the canonical command name is meaningful for
/// the declared recovery kind. Used by
/// `Self::validate_recovery_references` to fail-closed at scenario
/// load when a `deploy_recovery` event names a command its target
/// device cannot accept.
///
/// | kind            | deploy | deploy_drogue | deploy_main | stow |
/// |-----------------|--------|---------------|-------------|------|
/// | parachute_drag  | ✓      |               |             |      |
/// | drogue_main     |        | ✓             | ✓           |      |
/// | drag_device     | ✓      |               |             | ✓    |
fn is_recovery_command_compatible(kind_name: &str, command: &str) -> bool {
    matches!(
        (kind_name, command),
        ("parachute_drag", "deploy")
            | ("drogue_main", "deploy_drogue" | "deploy_main")
            | ("drag_device", "deploy" | "stow")
    )
}

// ---------------------------------------------------------------------
// RecoveryConfig
// ---------------------------------------------------------------------

/// One declared recovery device within `[vehicle.assembly]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct RecoveryConfig {
    /// Stable recovery-device id (`snake_case` scenario-text
    /// identifier). Resolved to a canonical
    /// `vehicle.assembly.recovery.<id>` path and FNV-hashed into a
    /// stable `RecoveryId` at scenario load.
    pub id: String,
    /// Optional owner body for post-separation recovery-drag routing.
    /// Required when `[multi_body]` is declared and recovery devices
    /// are present.
    #[serde(default)]
    pub mounted_to: Option<String>,
    /// Recovery-device kind + per-kind parameters.
    pub kind: RecoveryKindConfig,
}

impl RecoveryConfig {
    pub(crate) fn validate(
        &self,
        index: usize,
        body_ids: &std::collections::BTreeSet<&str>,
    ) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.recovery[{index}].{field}");
        require_non_empty(&path("id"), &self.id)?;
        if let Some(mounted_to) = &self.mounted_to {
            require_non_empty(&path("mounted_to"), mounted_to)?;
            if !body_ids.contains(mounted_to.as_str()) {
                return Err(ScenarioError::UnknownBodyReference {
                    field: path("mounted_to"),
                    value: mounted_to.clone(),
                });
            }
        }
        self.kind.validate(index)?;
        Ok(())
    }
}

/// Recovery-device kind tagged enum. Drives `RecoveryModel`
/// construction in the runner.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum RecoveryKindConfig {
    /// Single-stage parachute (`Stowed → Main` on `deploy`).
    ParachuteDrag {
        /// Drag coefficient `C_D` after deployment. Strictly
        /// positive, finite.
        c_d: f64,
        /// Inflated drag area, m². Strictly positive, finite.
        area_inflated_m2: f64,
    },
    /// Two-stage drogue + main (`Stowed → Drogue → Main` on
    /// `deploy_drogue` then `deploy_main`).
    DrogueMain {
        /// Drogue-stage drag coefficient. Strictly positive, finite.
        drogue_c_d: f64,
        /// Drogue-stage drag area, m². Strictly positive, finite.
        drogue_area_m2: f64,
        /// Main-stage drag coefficient. Strictly positive, finite.
        main_c_d: f64,
        /// Main-stage drag area, m². Strictly positive, finite.
        main_area_m2: f64,
    },
    /// Generic airbrake. Cycles `Stowed ↔ Main` under `deploy` /
    /// `stow` commands.
    DragDevice {
        /// Drag coefficient when deployed. Strictly positive, finite.
        c_d: f64,
        /// Deployed-state drag area, m². Strictly positive, finite.
        area_deployed_m2: f64,
    },
}

impl RecoveryKindConfig {
    /// Stable kind discriminant string used in cross-reference
    /// validation and telemetry tags.
    #[must_use]
    pub const fn kind_name(&self) -> &'static str {
        match self {
            Self::ParachuteDrag { .. } => "parachute_drag",
            Self::DrogueMain { .. } => "drogue_main",
            Self::DragDevice { .. } => "drag_device",
        }
    }

    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.recovery[{index}].kind.{field}");
        match self {
            Self::ParachuteDrag {
                c_d,
                area_inflated_m2,
            } => {
                require_finite(&path("c_d"), *c_d)?;
                require_positive(&path("c_d"), *c_d)?;
                require_finite(&path("area_inflated_m2"), *area_inflated_m2)?;
                require_positive(&path("area_inflated_m2"), *area_inflated_m2)?;
            }
            Self::DrogueMain {
                drogue_c_d,
                drogue_area_m2,
                main_c_d,
                main_area_m2,
            } => {
                require_finite(&path("drogue_c_d"), *drogue_c_d)?;
                require_positive(&path("drogue_c_d"), *drogue_c_d)?;
                require_finite(&path("drogue_area_m2"), *drogue_area_m2)?;
                require_positive(&path("drogue_area_m2"), *drogue_area_m2)?;
                require_finite(&path("main_c_d"), *main_c_d)?;
                require_positive(&path("main_c_d"), *main_c_d)?;
                require_finite(&path("main_area_m2"), *main_area_m2)?;
                require_positive(&path("main_area_m2"), *main_area_m2)?;
            }
            Self::DragDevice {
                c_d,
                area_deployed_m2,
            } => {
                require_finite(&path("c_d"), *c_d)?;
                require_positive(&path("c_d"), *c_d)?;
                require_finite(&path("area_deployed_m2"), *area_deployed_m2)?;
                require_positive(&path("area_deployed_m2"), *area_deployed_m2)?;
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
// Vehicle assembly block
// ---------------------------------------------------------------------

/// Top-level `[vehicle.assembly]` block.
///
/// Declares the vehicle as a tree of bodies plus optional
/// child blocks for effectors, engines, tanks, and recovery devices.
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
    /// Control effectors in scenario-declared order. The
    /// runner builds `Box<dyn ControlEffector>` instances from these
    /// configs and adds them to the assembly's effector vec.
    #[serde(default)]
    pub effectors: Vec<EffectorConfig>,
    /// Liquid engines in scenario-declared order. The
    /// runner builds `Box<dyn EngineModel>` instances from these
    /// configs and assembles them into a runner-side `EngineRack`.
    /// A scenario that also declares `[propulsion.motor]` is
    /// rejected with `ScenarioError::AmbiguousPropulsion`.
    #[serde(default)]
    pub engines: Vec<EngineConfig>,
    /// Optional cluster-layout tag for telemetry / docs.
    /// Defaults to `Custom` when omitted.
    #[serde(default)]
    pub cluster_layout: Option<ClusterLayoutConfig>,
    /// Tanks in scenario-declared order. The runner
    /// builds `Box<dyn MovingMassModel>` instances from these
    /// configs and assembles them into a runner-side `TankRack`.
    /// Tanks contribute their mass, CG offset, inertia delta, and
    /// reaction force / moment to the vehicle dynamics. Drain is
    /// decoupled from engines (`drain_rate_kg_per_s`
    /// is scenario-declared); engine-cluster drain coupling is
    /// future work.
    #[serde(default)]
    pub tanks: Vec<TankConfig>,
    /// Recovery devices (parachutes, drogue/main, drag
    /// devices) in scenario-declared order. The runner builds
    /// `Box<dyn RecoveryModel>` instances from these configs and
    /// assembles them into a runner-side `RecoveryRack`. Deploy /
    /// stow events are driven by `[mission.events]` declarations
    /// with `action.kind = "deploy_recovery"` targeting the
    /// device's id.
    #[serde(default)]
    pub recovery: Vec<RecoveryConfig>,
}

impl AssemblyConfig {
    /// Validate the assembly block.
    pub(crate) fn validate(&self, vehicle_kind: &str, dt_s: f64) -> Result<(), ScenarioError> {
        if self.bodies.is_empty() {
            return Err(ScenarioError::EmptyList {
                field: "vehicle.assembly.bodies".to_owned(),
            });
        }
        let rigid_body = vehicle_kind == "rigid_body";
        let body_ids: std::collections::BTreeSet<&str> =
            self.bodies.iter().map(|b| b.id.as_str()).collect();
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
        let mut seen_effector_ids: std::collections::BTreeSet<&str> =
            std::collections::BTreeSet::new();
        for (index, effector) in self.effectors.iter().enumerate() {
            effector.validate(index, dt_s, &body_ids)?;
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
            engine.validate(index, &body_ids)?;
            if !seen_engine_ids.insert(engine.id.as_str()) {
                return Err(ScenarioError::DuplicateValue {
                    field: format!("vehicle.assembly.engines[{index}].id"),
                    value: engine.id.clone(),
                });
            }
        }
        let mut seen_tank_ids: std::collections::BTreeSet<&str> = std::collections::BTreeSet::new();
        for (index, tank) in self.tanks.iter().enumerate() {
            tank.validate(index, vehicle_kind, &body_ids, dt_s)?;
            if !seen_tank_ids.insert(tank.id.as_str()) {
                return Err(ScenarioError::DuplicateValue {
                    field: format!("vehicle.assembly.tanks[{index}].id"),
                    value: tank.id.clone(),
                });
            }
        }
        self.validate_engine_propellant_references()?;
        let mut seen_recovery_ids: std::collections::BTreeSet<&str> =
            std::collections::BTreeSet::new();
        for (index, recovery) in self.recovery.iter().enumerate() {
            recovery.validate(index, &body_ids)?;
            if !seen_recovery_ids.insert(recovery.id.as_str()) {
                return Err(ScenarioError::DuplicateValue {
                    field: format!("vehicle.assembly.recovery[{index}].id"),
                    value: recovery.id.clone(),
                });
            }
        }
        Ok(())
    }

    fn validate_engine_propellant_references(&self) -> Result<(), ScenarioError> {
        let tank_lookup: std::collections::BTreeMap<&str, &TankConfig> = self
            .tanks
            .iter()
            .map(|tank| (tank.id.as_str(), tank))
            .collect();
        for (engine_index, engine) in self.engines.iter().enumerate() {
            let Some(propellant) = &engine.propellant else {
                continue;
            };
            let path = |field: &str| {
                format!("vehicle.assembly.engines[{engine_index}].propellant.{field}")
            };
            let Some(engine_body) = engine.mounted_to.as_deref() else {
                return Err(ScenarioError::MissingRequiredField {
                    field: format!("vehicle.assembly.engines[{engine_index}].mounted_to"),
                    role: ModelRole::Vehicle,
                    name: "engine_propellant".to_owned(),
                });
            };
            let fuel = tank_lookup
                .get(propellant.fuel_tank.as_str())
                .ok_or_else(|| ScenarioError::IncompatibleAssemblyEntry {
                    field: path("fuel_tank"),
                    reason: format!("unknown tank id `{}`", propellant.fuel_tank),
                })?;
            if fuel.mounted_to != engine_body {
                return Err(ScenarioError::IncompatibleAssemblyEntry {
                    field: path("fuel_tank"),
                    reason: "fuel_tank must be mounted_to the same body as the engine".to_owned(),
                });
            }
            if propellant.feed == FeedModeConfig::Blowdown && fuel.ullage.is_none() {
                return Err(ScenarioError::IncompatibleAssemblyEntry {
                    field: path("feed"),
                    reason: "feed = \"blowdown\" requires fuel_tank.ullage".to_owned(),
                });
            }
            if let Some(oxidizer_tank) = &propellant.oxidizer_tank {
                let oxidizer = tank_lookup.get(oxidizer_tank.as_str()).ok_or_else(|| {
                    ScenarioError::IncompatibleAssemblyEntry {
                        field: path("oxidizer_tank"),
                        reason: format!("unknown tank id `{oxidizer_tank}`"),
                    }
                })?;
                if oxidizer.mounted_to != engine_body {
                    return Err(ScenarioError::IncompatibleAssemblyEntry {
                        field: path("oxidizer_tank"),
                        reason: "oxidizer_tank must be mounted_to the same body as the engine"
                            .to_owned(),
                    });
                }
                if propellant.feed == FeedModeConfig::Blowdown && oxidizer.ullage.is_none() {
                    return Err(ScenarioError::IncompatibleAssemblyEntry {
                        field: path("feed"),
                        reason: "feed = \"blowdown\" requires oxidizer_tank.ullage".to_owned(),
                    });
                }
            }
        }
        Ok(())
    }
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
            validate_inertia_tensor(
                &format!("vehicle.assembly.bodies[{index}].dry_inertia_body_kg_m2"),
                inertia,
            )?;
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
// Effector block
// ---------------------------------------------------------------------

/// One declared control effector within `[vehicle.assembly]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EffectorConfig {
    /// Stable effector id (`snake_case` scenario-text identifier).
    pub id: String,
    /// Optional owner body for post-separation moment / aero-axis
    /// routing. Required when `[multi_body]` is declared and effectors
    /// are present.
    #[serde(default)]
    pub mounted_to: Option<String>,
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
    /// per-step command at runtime; superseded by a scenario-script
    /// effector override on the next rack tick.
    #[serde(default)]
    pub command_schedule: Option<EffectorCommandScheduleConfig>,
}

impl EffectorConfig {
    fn validate(
        &self,
        index: usize,
        dt_s: f64,
        body_ids: &std::collections::BTreeSet<&str>,
    ) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.effectors[{index}].{field}");
        require_non_empty(&path("id"), &self.id)?;
        if let Some(mounted_to) = &self.mounted_to {
            require_non_empty(&path("mounted_to"), mounted_to)?;
            if !body_ids.contains(mounted_to.as_str()) {
                return Err(ScenarioError::UnknownBodyReference {
                    field: path("mounted_to"),
                    value: mounted_to.clone(),
                });
            }
        }
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

/// Effector kind tagged enum. `linear_actuator` is the baseline kind;
/// `direct_torque` (v3-only) supports closed-loop
/// autopilot-validation scenarios that need the kernel to respond
/// to autopilot torque commands without an aero deck in the loop.
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
    /// Direct body-torque source (v3-only). The
    /// effector's deflection is interpreted as a body-frame torque
    /// command on the named `axis`, scaled by
    /// `effectiveness_n_m_per_rad`. The actuator dynamics still go
    /// through the same first-order `LinearActuator` shape; the kind
    /// tag tells the runner to feed the effector's deflection into a
    /// `DirectTorqueMomentAdapter` instead of (or in addition to) any
    /// aero-deck axis. Used in closed-loop FC validation scenarios
    /// (e.g. `diff-flatness-figure-eight`).
    DirectTorque {
        /// Body axis the deflection drives (`roll` / `pitch` / `yaw`).
        axis: TorqueAxis,
        /// Per-rad torque effectiveness (N·m / rad).
        effectiveness_n_m_per_rad: f64,
    },
}

/// Body-frame axis a `DirectTorque` effector drives.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TorqueAxis {
    /// Roll body-frame moment about +x.
    Roll,
    /// Pitch body-frame moment about +y.
    Pitch,
    /// Yaw body-frame moment about +z.
    Yaw,
}

impl TorqueAxis {
    /// Index into a body-frame `[roll, pitch, yaw]` vector.
    #[must_use]
    pub const fn body_axis_index(self) -> usize {
        match self {
            Self::Roll => 0,
            Self::Pitch => 1,
            Self::Yaw => 2,
        }
    }
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
            Self::DirectTorque {
                effectiveness_n_m_per_rad,
                ..
            } => {
                require_finite(
                    &path("effectiveness_n_m_per_rad"),
                    *effectiveness_n_m_per_rad,
                )?;
                require_positive(
                    &path("effectiveness_n_m_per_rad"),
                    *effectiveness_n_m_per_rad,
                )?;
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
    /// Piecewise-linear schedule. Holds the first point before its
    /// time, linearly interpolates between adjacent points, and holds
    /// the final point thereafter.
    PiecewiseLinear {
        /// Strictly time-ordered schedule points.
        points: Vec<EffectorSchedulePointConfig>,
    },
}

/// One point in a piecewise-linear effector command schedule.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EffectorSchedulePointConfig {
    /// Point time (s).
    pub time_s: f64,
    /// Command value at `time_s`.
    pub value: f64,
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
            Self::PiecewiseLinear { points } => {
                if points.is_empty() {
                    return Err(ScenarioError::EmptyList {
                        field: path("points"),
                    });
                }
                let mut previous_time_s: Option<f64> = None;
                for (point_index, point) in points.iter().enumerate() {
                    let point_path = |field: &str| path(&format!("points[{point_index}].{field}"));
                    require_finite(&point_path("time_s"), point.time_s)?;
                    require_finite(&point_path("value"), point.value)?;
                    if let Some(previous) = previous_time_s
                        && point.time_s <= previous
                    {
                        return Err(ScenarioError::InvalidNumber {
                            field: point_path("time_s"),
                            value: point.time_s,
                            rule: "must be strictly greater than the previous point time_s",
                        });
                    }
                    previous_time_s = Some(point.time_s);
                }
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------
// EngineConfig
// ---------------------------------------------------------------------

/// Cluster-layout tag for telemetry / docs. It has no
/// behavioural use; symmetry-aware fault scenarios may key off the
/// layout in the future.
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
    /// Optional owner body for post-separation thrust / moment / mass
    /// routing. Required when `[multi_body]` is declared and engines
    /// are present.
    #[serde(default)]
    pub mounted_to: Option<String>,
    /// Engine kind + per-kind parameters (tagged on `kind`).
    pub kind: EngineKindConfig,
    /// Body-frame mount point (m). `[x, y, z]`.
    pub mount_point_body_m: [f64; 3],
    /// Position / rate / gimbal limits.
    pub limits: EngineLimitsConfig,
    /// Optional engine-to-tank propellant budget.
    #[serde(default)]
    pub propellant: Option<EnginePropellantConfig>,
    /// Optional fault mounted at scenario load time.
    #[serde(default)]
    pub fault: Option<EngineFaultConfig>,
}

impl EngineConfig {
    fn validate(
        &self,
        index: usize,
        body_ids: &std::collections::BTreeSet<&str>,
    ) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.engines[{index}].{field}");
        require_non_empty(&path("id"), &self.id)?;
        if let Some(mounted_to) = &self.mounted_to {
            require_non_empty(&path("mounted_to"), mounted_to)?;
            if !body_ids.contains(mounted_to.as_str()) {
                return Err(ScenarioError::UnknownBodyReference {
                    field: path("mounted_to"),
                    value: mounted_to.clone(),
                });
            }
        }
        self.kind.validate(index)?;
        self.limits.validate(index)?;
        if let Some(propellant) = &self.propellant {
            propellant.validate(index)?;
        }
        require_finite_array(&path("mount_point_body_m"), &self.mount_point_body_m)?;
        if let Some(fault) = &self.fault {
            fault.validate(index, &self.limits)?;
        }
        Ok(())
    }
}

/// Engine propellant budget declaration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EnginePropellantConfig {
    /// Oxidizer/fuel mass ratio. `0.0` denotes monopropellant.
    pub oxidizer_fuel_ratio: f64,
    /// Fuel or monopropellant tank id.
    pub fuel_tank: String,
    /// Oxidizer tank id, required when `oxidizer_fuel_ratio > 0`.
    #[serde(default)]
    pub oxidizer_tank: Option<String>,
    /// Feed model.
    #[serde(default)]
    pub feed: FeedModeConfig,
    /// Per-tank residual reserve (kg).
    #[serde(default)]
    pub residual_reserve_kg: f64,
}

impl EnginePropellantConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.engines[{index}].propellant.{field}");
        require_finite(&path("oxidizer_fuel_ratio"), self.oxidizer_fuel_ratio)?;
        if self.oxidizer_fuel_ratio < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("oxidizer_fuel_ratio"),
                value: self.oxidizer_fuel_ratio,
                rule: "must be non-negative",
            });
        }
        require_non_empty(&path("fuel_tank"), &self.fuel_tank)?;
        if self.oxidizer_fuel_ratio > 0.0 {
            let Some(oxidizer_tank) = &self.oxidizer_tank else {
                return Err(ScenarioError::MissingRequiredField {
                    field: path("oxidizer_tank"),
                    role: ModelRole::Vehicle,
                    name: "bipropellant_engine".to_owned(),
                });
            };
            require_non_empty(&path("oxidizer_tank"), oxidizer_tank)?;
        } else if self.oxidizer_tank.is_some() {
            return Err(ScenarioError::UnexpectedField {
                field: path("oxidizer_tank"),
                role: ModelRole::Vehicle,
                name: "monopropellant_engine".to_owned(),
            });
        }
        require_finite(&path("residual_reserve_kg"), self.residual_reserve_kg)?;
        if self.residual_reserve_kg < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("residual_reserve_kg"),
                value: self.residual_reserve_kg,
                rule: "must be non-negative",
            });
        }
        Ok(())
    }
}

/// Feed model selector.
#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum FeedModeConfig {
    /// Regulated feed pressure.
    #[default]
    Regulated,
    /// Pressure-fed blowdown.
    Blowdown,
}

/// Engine kind tagged enum. `liquid_engine` is the only kind.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EngineKindConfig {
    /// Reference impl: linear ignition + constant burn +
    /// linear shutdown transients.
    LiquidEngine,
}

impl EngineKindConfig {
    // The `Result` return is forward-compat: future engine kinds
    // (hybrid, cold-gas) will carry payload that needs validation.
    // Only `LiquidEngine` exists, with no payload, so this
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
    /// Maximum gimbal slew rate (rad/s). Defaults to `+∞`, preserving
    /// instantaneous gimbal latching; set a finite rate to model a
    /// real actuator and damp step-frequency gimbal chatter.
    #[serde(default = "default_infinite")]
    pub gimbal_slew_rad_per_s: f64,
    /// Maximum throttle slew rate (1/s). Defaults to `+∞`,
    /// preserving instantaneous latching.
    #[serde(default = "default_infinite")]
    pub throttle_slew_per_s: f64,
    /// Deep-throttle floor. Defaults to `0.0`.
    #[serde(default)]
    pub min_throttle_unit: f64,
    /// Linear Isp derate coefficient at low throttle. Defaults to
    /// `0.0`.
    #[serde(default)]
    pub isp_throttle_falloff: f64,
    /// When `true`, the engine may be RESTARTED: after a completed
    /// shutdown transient it re-arms and a later ignition begins a new
    /// burn (e.g. an upper-stage restart for a second burn). Defaults to
    /// `false` — `shutdown` is then a one-shot terminal command.
    #[serde(default)]
    pub restartable: bool,
}

fn default_infinite() -> f64 {
    f64::INFINITY
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
        if self.gimbal_slew_rad_per_s.is_nan() || self.gimbal_slew_rad_per_s < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("gimbal_slew_rad_per_s"),
                value: self.gimbal_slew_rad_per_s,
                rule: "must be non-negative or +infinity",
            });
        }
        if self.throttle_slew_per_s.is_nan() || self.throttle_slew_per_s < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("throttle_slew_per_s"),
                value: self.throttle_slew_per_s,
                rule: "must be non-negative or +infinity",
            });
        }
        require_finite(&path("min_throttle_unit"), self.min_throttle_unit)?;
        if !(0.0..=1.0).contains(&self.min_throttle_unit) {
            return Err(ScenarioError::InvalidNumber {
                field: path("min_throttle_unit"),
                value: self.min_throttle_unit,
                rule: "must lie in [0, 1]",
            });
        }
        require_finite(&path("isp_throttle_falloff"), self.isp_throttle_falloff)?;
        if !(0.0..1.0).contains(&self.isp_throttle_falloff) {
            return Err(ScenarioError::InvalidNumber {
                field: path("isp_throttle_falloff"),
                value: self.isp_throttle_falloff,
                rule: "must lie in [0, 1)",
            });
        }
        Ok(())
    }
}

/// Engine fault tagged enum. Four canonical modes;
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
// Tank block
// ---------------------------------------------------------------------

/// One declared tank entry under `[[vehicle.assembly.tanks]]`.
///
/// The runner builds a `Box<dyn MovingMassModel>` from this config
/// and assembles it into a runner-side `TankRack` mirroring the
/// `EngineRack` pattern.
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
    /// Optional ullage/pressurant declaration, required by engine
    /// propellant budgets that select `feed = "blowdown"`.
    #[serde(default)]
    pub ullage: Option<TankUllageConfig>,
    /// Decoupled drain. Constant `kg/s`; defaults to
    /// `0.0` when omitted. A future revision may tie this to the engine
    /// cluster's per-step total mdot.
    #[serde(default)]
    pub drain_rate_kg_per_s: Option<f64>,
    /// Optional initial slosh perturbation (used for free-response
    /// scenarios and exit-criterion testing).
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
        if let Some(ullage) = &self.ullage {
            ullage.validate(index)?;
            let initial_fluid_volume_m3 = self.geometry.volume_m3() * self.initial_fill_fraction;
            if initial_fluid_volume_m3 >= self.geometry.volume_m3() {
                return Err(ScenarioError::InvalidNumber {
                    field: path("initial_fill_fraction"),
                    value: self.initial_fill_fraction,
                    rule: "must leave positive ullage volume when ullage is declared",
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

/// Tank ullage/pressurant declaration for blowdown feed coupling.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TankUllageConfig {
    /// Initial tank pressure (Pa). Recorded for provenance; the
    /// current normalized feed-pressure scale uses the volume ratio.
    pub initial_pressure_pa: f64,
    /// Pressurant heat-capacity ratio.
    pub gas_gamma: f64,
}

impl TankUllageConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("vehicle.assembly.tanks[{index}].ullage.{field}");
        require_finite(&path("initial_pressure_pa"), self.initial_pressure_pa)?;
        require_positive(&path("initial_pressure_pa"), self.initial_pressure_pa)?;
        require_finite(&path("gas_gamma"), self.gas_gamma)?;
        if self.gas_gamma <= 1.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("gas_gamma"),
                value: self.gas_gamma,
                rule: "must be greater than 1",
            });
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

/// Low-g / freefall restoring model for an equivalent-pendulum tank.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum FreefallRestoringConfig {
    /// Capillary surface-wave term,
    /// `ω_cap² = (σ/ρ) · k³ · tanh(kh)`.
    CapillarySurfaceWave {
        /// Liquid-vapour surface tension, N/m.
        surface_tension_n_m: f64,
        /// Damping ratio used when the capillary term dominates; `1.0`
        /// gives critical damping.
        damping_ratio_zeta: f64,
    },
}

impl FreefallRestoringConfig {
    fn validate(&self, path: &str) -> Result<(), ScenarioError> {
        match *self {
            Self::CapillarySurfaceWave {
                surface_tension_n_m,
                damping_ratio_zeta,
            } => {
                require_finite(&format!("{path}.surface_tension_n_m"), surface_tension_n_m)?;
                require_positive(&format!("{path}.surface_tension_n_m"), surface_tension_n_m)?;
                require_finite(&format!("{path}.damping_ratio_zeta"), damping_ratio_zeta)?;
                if damping_ratio_zeta < 0.0 {
                    return Err(ScenarioError::InvalidNumber {
                        field: format!("{path}.damping_ratio_zeta"),
                        value: damping_ratio_zeta,
                        rule: "must be non-negative",
                    });
                }
            }
        }
        Ok(())
    }
}

/// Moving-mass kind tagged enum. Four implementations.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum MovingMassKindConfig {
    /// No-slosh toy.
    RigidLiquid,
    /// Abramson cylindrical-tank equivalent pendulum.
    EquivalentPendulum {
        /// Bare-tank damping ratio (typical academic value `0.005`).
        #[serde(default)]
        damping_ratio_zeta: f64,
        /// Optional capillary surface-wave restoring term for low-g /
        /// freefall slosh. Defaults to `None` (bare Abramson pendulum).
        #[serde(default)]
        freefall_restoring: Option<FreefallRestoringConfig>,
    },
    /// Linear translational alternative.
    EquivalentSpringMass {
        /// Bare-tank damping ratio.
        #[serde(default)]
        damping_ratio_zeta: f64,
    },
    /// `EquivalentPendulum` with `BaffleModel`-supplied
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
            Self::EquivalentPendulum {
                damping_ratio_zeta,
                freefall_restoring,
            } => {
                require_finite(&path("damping_ratio_zeta"), *damping_ratio_zeta)?;
                if *damping_ratio_zeta < 0.0 {
                    return Err(ScenarioError::InvalidNumber {
                        field: path("damping_ratio_zeta"),
                        value: *damping_ratio_zeta,
                        rule: "must be non-negative",
                    });
                }
                if let Some(freefall_restoring) = freefall_restoring {
                    freefall_restoring.validate(&path("freefall_restoring"))?;
                }
            }
            Self::EquivalentSpringMass { damping_ratio_zeta } => {
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
    /// `damping_ratio_zeta`. Minimum surface; future revisions
    /// may add baffle-area integration per Abramson Eq 7-46.
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

// ---------------------------------------------------------------------
// Flight-controller config
// ---------------------------------------------------------------------

/// Flight-controller config parsed from a scenario `[fc]` block.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcConfig {
    /// Estimator kind. Ignored when `[fc.estimator_lanes]` is
    /// present; otherwise selects the single estimator instance.
    pub estimator: FcEstimatorKind,
    /// Autopilot kind. Must be `three_loop`.
    pub autopilot: FcAutopilotKind,
    /// Guidance kind. Must be one of `attitude_hold`, `waypoint`,
    /// `ascent_reference`.
    pub guidance: FcGuidanceKind,
    /// Reference attitude quaternion `[x, y, z, w]` (optional; required
    /// when `guidance = "attitude_hold"`).
    pub reference_q_xyzw: Option<[f64; 4]>,
    /// Base scheduler tick rate in Hz. Determines the kernel-tick to
    /// FC-tick mapping.
    pub base_rate_hz: u32,
    /// Controller frame budget in microseconds.
    pub frame_budget_us: u64,
    /// Optional scheduler job cadence / declared-budget overrides.
    #[serde(default)]
    pub scheduler: Option<FcSchedulerConfig>,
    /// Optional EKF parameter overrides. Required when
    /// `estimator = "ekf"`.
    pub ekf: Option<FcEkfConfig>,
    /// Optional MEKF parameter overrides. Required when
    /// `estimator = "mekf"`.
    pub mekf: Option<FcMekfConfig>,
    /// Optional Bar-Shalom IMM bank (v3 only). Required
    /// when `estimator = "imm"`. Carries the Markov mode-transition
    /// matrix, initial mode probabilities, and per-mode EKF tuning
    /// overrides for each of the `2 ≤ N ≤ 4` mode-conditioned
    /// sub-filters.
    pub imm: Option<FcImmConfig>,
    /// Optional autopilot anti-windup / trajectory-loop config.
    pub autopilot_params: Option<FcAutopilotParams>,
    /// Required health-monitor thresholds.
    pub health: FcHealthConfig,
    /// Optional FDIR detector configuration.
    pub fdir: Option<FcFdirConfig>,
    /// Optional semantic FC actuator-channel to vehicle effector-id
    /// mapping.
    pub actuator_channels: Option<FcActuatorChannelsConfig>,
    /// Gain schedule keyed by `mission.phases.<name>` paths.
    pub gain_schedule: BTreeMap<String, FcGainsConfig>,
    /// Optional phase-authority mask keyed by mission-phase path.
    pub phase_authority: Option<BTreeMap<String, FcPhaseAuthorityConfig>>,
    /// Optional multi-instance estimator-routing block (v3 only).
    ///
    /// Consumed under v3 only. Scenarios that declare
    /// `[fc.estimator_lanes]` must have `openbmp.scenario = 3`.
    pub estimator_lanes: Option<FcEstimatorLanesConfig>,
    /// Optional control-allocation policy block (v3 only).
    ///
    /// Parsed under v3 only. Scenarios that declare
    /// `[fc.autopilot_allocation]` must have `openbmp.scenario = 3`.
    pub autopilot_allocation: Option<FcAutopilotAllocationConfig>,
    /// Optional minimum-snap differential-flatness trajectory block
    /// (v3 only). Required when
    /// `autopilot_params.trajectory_kind == FcTrajectoryKind::MinimumSnap`;
    /// scenarios that declare `[fc.trajectory]` must have
    /// `openbmp.scenario = 3`.
    pub trajectory: Option<FcTrajectoryConfig>,
    /// Optional powered-ascent reference generator block. Required
    /// when `guidance = "ascent_reference"` unless
    /// `ascent_reference_by_phase` is given. See
    /// `docs/ascent-guidance.md`.
    #[serde(default)]
    pub ascent_reference: Option<FcAscentReferenceConfig>,
    /// Optional per-phase ascent reference generators, keyed by mission
    /// phase id. When present, a phase-gated guidance job is built for
    /// each entry, so a staged vehicle can fly different guidance per
    /// phase (e.g. a gravity turn on the booster, PEG on the upper
    /// stage). Coexists with `ascent_reference` (the default for any
    /// powered-ascent phase not listed here).
    #[serde(default)]
    pub ascent_reference_by_phase: Option<BTreeMap<String, FcAscentReferenceConfig>>,
}

/// Optional scheduler I-loads for FC job cadences and declared budgets.
///
/// Missing fields preserve OpenBMP's current reference topology:
/// estimator, commander, autopilot, and mixer run at the base FC rate;
/// guidance, health, and FDIR run at 100 Hz.
#[derive(Clone, Debug, Deserialize, PartialEq, Default)]
#[serde(deny_unknown_fields)]
pub struct FcSchedulerConfig {
    /// Enables host-side per-job wall-clock timing in the runner.
    /// The measurement hook lives outside `openbmp-fc`; the controller
    /// only receives measured microseconds.
    pub instrument_timing: Option<bool>,
    /// Estimator job cadence in Hz.
    pub estimator_rate_hz: Option<u32>,
    /// Guidance job cadence in Hz.
    pub guidance_rate_hz: Option<u32>,
    /// Commander job cadence in Hz.
    pub commander_rate_hz: Option<u32>,
    /// Autopilot job cadence in Hz.
    pub autopilot_rate_hz: Option<u32>,
    /// Mixer job cadence in Hz.
    pub mixer_rate_hz: Option<u32>,
    /// Health-monitor job cadence in Hz.
    pub health_rate_hz: Option<u32>,
    /// FDIR job cadence in Hz.
    pub fdir_rate_hz: Option<u32>,
    /// Estimator declared budget in microseconds.
    pub estimator_budget_us: Option<u64>,
    /// Guidance declared budget in microseconds.
    pub guidance_budget_us: Option<u64>,
    /// Commander declared budget in microseconds.
    pub commander_budget_us: Option<u64>,
    /// Autopilot declared budget in microseconds.
    pub autopilot_budget_us: Option<u64>,
    /// Mixer declared budget in microseconds.
    pub mixer_budget_us: Option<u64>,
    /// Health-monitor declared budget in microseconds.
    pub health_budget_us: Option<u64>,
    /// FDIR declared budget in microseconds.
    pub fdir_budget_us: Option<u64>,
}

impl FcSchedulerConfig {
    fn validate(&self, base_rate_hz: u32) -> Result<(), ScenarioError> {
        for (name, rate_hz) in [
            ("estimator_rate_hz", self.estimator_rate_hz),
            ("guidance_rate_hz", self.guidance_rate_hz),
            ("commander_rate_hz", self.commander_rate_hz),
            ("autopilot_rate_hz", self.autopilot_rate_hz),
            ("mixer_rate_hz", self.mixer_rate_hz),
            ("health_rate_hz", self.health_rate_hz),
            ("fdir_rate_hz", self.fdir_rate_hz),
        ] {
            if let Some(rate_hz) = rate_hz
                && (rate_hz == 0 || rate_hz > base_rate_hz)
            {
                return Err(ScenarioError::InvalidFc {
                    reason: format!("fc.scheduler.{name} must be in 1..={base_rate_hz} Hz"),
                });
            }
        }
        for (name, budget_us) in [
            ("estimator_budget_us", self.estimator_budget_us),
            ("guidance_budget_us", self.guidance_budget_us),
            ("commander_budget_us", self.commander_budget_us),
            ("autopilot_budget_us", self.autopilot_budget_us),
            ("mixer_budget_us", self.mixer_budget_us),
            ("health_budget_us", self.health_budget_us),
            ("fdir_budget_us", self.fdir_budget_us),
        ] {
            if let Some(0) = budget_us {
                return Err(ScenarioError::InvalidFc {
                    reason: format!("fc.scheduler.{name} must be > 0"),
                });
            }
        }
        Ok(())
    }
}

/// Powered-ascent reference generator configuration.
///
/// The reference it produces is a *reference trajectory the
/// three-loop autopilot tracks* — never a guidance solution to a
/// real-world location. See `docs/ascent-guidance.md` and
/// `docs/profile-vocabulary-and-guardrails.md`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcAscentReferenceConfig {
    /// Reference method: `pitch_program`, `gravity_turn`, or the
    /// reserved `explicit_reference`.
    pub method: FcAscentReferenceMethod,
    /// Pitch-program schedule timestamps (s), monotonic ascending.
    #[serde(default)]
    pub schedule_s: Option<Vec<f64>>,
    /// Pitch-program reference pitch from vertical (rad), one per
    /// `schedule_s` entry.
    #[serde(default)]
    pub pitch_rad: Option<Vec<f64>>,
    /// Closed-loop insertion: desired orbital radius (m, from Earth
    /// centre) the guidance flies to with zero radial velocity.
    #[serde(default)]
    pub insertion_radius_m: Option<f64>,
    /// Closed-loop insertion: altitude-error gain (rad per m).
    #[serde(default)]
    pub k_alt_rad_per_m: Option<f64>,
    /// Closed-loop insertion: radial-velocity damping gain (rad per
    /// m/s).
    #[serde(default)]
    pub k_vr_rad_per_m_s: Option<f64>,
    /// Closed-loop insertion: minimum thrust elevation above horizon
    /// (rad); may be negative to push the flight-path angle down.
    #[serde(default)]
    pub theta_min_rad: Option<f64>,
    /// Closed-loop insertion: maximum thrust elevation above horizon
    /// (rad).
    #[serde(default)]
    pub theta_max_rad: Option<f64>,
    /// Closed-loop insertion: ECI downrange reference axis (projected
    /// into the local horizontal each step).
    #[serde(default)]
    pub downrange_axis_eci: Option<[f64; 3]>,
    /// PEG / ascent_sequence: effective exhaust velocity `ve = Isp · g0`
    /// (m/s).
    #[serde(default)]
    pub exhaust_velocity_m_s: Option<f64>,
    /// PEG / ascent_sequence: thrust acceleration at burn start
    /// `a0 = T / m0` (m/s²); seeds PEG's constant-thrust burn-time model.
    #[serde(default)]
    pub initial_thrust_accel_m_s2: Option<f64>,
    /// PEG / ascent_sequence: initial time-to-go estimate (s).
    #[serde(default)]
    pub peg_initial_t_go_s: Option<f64>,
    /// PEG: minimum inertial speed below which PEG is not run (m/s).
    #[serde(default)]
    pub peg_min_speed_m_s: Option<f64>,
    /// ascent_sequence: surface-relative speed to begin the pitch kick
    /// (m/s). `0` means the kick/gravity-turn sequence may start
    /// immediately.
    #[serde(default)]
    pub kick_start_speed_m_s: Option<f64>,
    /// ascent_sequence: surface-relative speed to end the pitch kick
    /// (m/s). Set equal to `kick_start_speed_m_s` to skip the pitch kick.
    #[serde(default)]
    pub kick_end_speed_m_s: Option<f64>,
    /// ascent_sequence: pitch-kick angle off vertical toward downrange
    /// (rad).
    #[serde(default)]
    pub kick_angle_rad: Option<f64>,
    /// ascent_sequence: surface-relative speed at which to hand off to PEG
    /// (m/s).
    #[serde(default)]
    pub peg_handoff_speed_m_s: Option<f64>,
    /// Yaw (out-of-plane) steering: inertial normal of the desired orbital
    /// plane (its angular-momentum direction). When set, `closed_loop_insertion`
    /// and `peg` add a cross-track thrust component that nulls velocity out
    /// of this plane, holding inclination. Forward-only: an orbital element,
    /// not a ground location.
    #[serde(default)]
    pub orbital_plane_normal_eci: Option<[f64; 3]>,
    /// Yaw steering: cross-track velocity gain (rad per m/s). Used only
    /// when `orbital_plane_normal_eci` is set.
    #[serde(default)]
    pub k_cross_rad_per_m_s: Option<f64>,
    /// Yaw steering: yaw-angle clamp (rad). Used only when
    /// `orbital_plane_normal_eci` is set.
    #[serde(default)]
    pub psi_max_rad: Option<f64>,
}

/// Supported powered-ascent reference methods.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcAscentReferenceMethod {
    /// Tabulated pitch angle versus time.
    PitchProgram,
    /// Body `+x` aligned with inertial velocity after motion starts.
    GravityTurn,
    /// Closed-loop orbital insertion: steer the thrust from the live
    /// state to reach a target radius with zero radial velocity
    /// (flight-path angle → 0). Uses `[fc.ascent_reference]`'s
    /// `target_radius_m` / `k_alt_rad_per_m` / `k_vr_rad_per_m_s` /
    /// `theta_min_rad` / `theta_max_rad` / `downrange_axis_eci`.
    ClosedLoopInsertion,
    /// Powered Explicit Guidance: closed-form fuel-optimal insertion
    /// targeting orbital radius + circular speed + zero radial velocity.
    /// Requires `insertion_radius_m`, `exhaust_velocity_m_s`,
    /// `initial_thrust_accel_m_s2`; intended for the fast (upper-stage)
    /// phase — pair with a gravity turn for the launch phase.
    Peg,
    /// Sequenced launch-to-orbit reference: vertical rise → pitch kick →
    /// gravity turn → PEG, selected by surface-relative speed. The full
    /// single-reference ascent. Requires the PEG fields plus
    /// `kick_start_speed_m_s` / `kick_end_speed_m_s` / `kick_angle_rad` /
    /// `peg_handoff_speed_m_s`.
    AscentSequence,
    /// Reserved future ingestion of explicit inertial references.
    ExplicitReference,
}

impl FcAscentReferenceConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        match self.method {
            FcAscentReferenceMethod::PitchProgram => self.validate_pitch_program(),
            FcAscentReferenceMethod::GravityTurn => self.validate_gravity_turn(),
            FcAscentReferenceMethod::ClosedLoopInsertion => {
                let radius = self
                    .insertion_radius_m
                    .ok_or_else(|| ScenarioError::InvalidFc {
                        reason: "fc.ascent_reference.method = \"closed_loop_insertion\" requires \
                                 insertion_radius_m"
                            .to_owned(),
                    })?;
                if !radius.is_finite() || radius <= 0.0 {
                    return Err(ScenarioError::InvalidFc {
                        reason:
                            "fc.ascent_reference.insertion_radius_m must be finite and positive"
                                .to_owned(),
                    });
                }
                Ok(())
            }
            FcAscentReferenceMethod::Peg => self.validate_peg(),
            FcAscentReferenceMethod::AscentSequence => self.validate_ascent_sequence(),
            FcAscentReferenceMethod::ExplicitReference => {
                Err(ScenarioError::ElementNotYetSupported {
                    field: "fc.ascent_reference.method = \"explicit_reference\"".to_owned(),
                    missing_capability: "explicit inertial ascent-reference ingestion",
                })
            }
        }
    }

    fn require_positive_field(&self, name: &str, value: Option<f64>) -> Result<f64, ScenarioError> {
        let v = value.ok_or_else(|| ScenarioError::InvalidFc {
            reason: format!("fc.ascent_reference.{name} is required for this method"),
        })?;
        if !v.is_finite() || v <= 0.0 {
            return Err(ScenarioError::InvalidFc {
                reason: format!("fc.ascent_reference.{name} must be finite and positive"),
            });
        }
        Ok(v)
    }

    fn require_non_negative_field(
        &self,
        name: &str,
        value: Option<f64>,
    ) -> Result<f64, ScenarioError> {
        let v = value.ok_or_else(|| ScenarioError::InvalidFc {
            reason: format!("fc.ascent_reference.{name} is required for this method"),
        })?;
        if !v.is_finite() || v < 0.0 {
            return Err(ScenarioError::InvalidFc {
                reason: format!("fc.ascent_reference.{name} must be finite and non-negative"),
            });
        }
        Ok(v)
    }

    fn validate_peg(&self) -> Result<(), ScenarioError> {
        self.require_positive_field("insertion_radius_m", self.insertion_radius_m)?;
        self.require_positive_field("exhaust_velocity_m_s", self.exhaust_velocity_m_s)?;
        self.require_positive_field("initial_thrust_accel_m_s2", self.initial_thrust_accel_m_s2)?;
        Ok(())
    }

    fn validate_ascent_sequence(&self) -> Result<(), ScenarioError> {
        self.validate_peg()?;
        let kick_start =
            self.require_non_negative_field("kick_start_speed_m_s", self.kick_start_speed_m_s)?;
        let kick_end =
            self.require_non_negative_field("kick_end_speed_m_s", self.kick_end_speed_m_s)?;
        let handoff =
            self.require_positive_field("peg_handoff_speed_m_s", self.peg_handoff_speed_m_s)?;
        let kick_angle = self
            .kick_angle_rad
            .ok_or_else(|| ScenarioError::InvalidFc {
                reason: "fc.ascent_reference.kick_angle_rad is required for ascent_sequence"
                    .to_owned(),
            })?;
        require_finite("fc.ascent_reference.kick_angle_rad", kick_angle)?;
        if kick_end < kick_start || handoff < kick_end {
            return Err(ScenarioError::InvalidFc {
                reason: "fc.ascent_reference: require kick_start <= kick_end <= peg_handoff speeds"
                    .to_owned(),
            });
        }
        Ok(())
    }

    fn validate_pitch_program(&self) -> Result<(), ScenarioError> {
        let schedule = self
            .schedule_s
            .as_ref()
            .ok_or_else(|| ScenarioError::InvalidFc {
                reason: "fc.ascent_reference.method = \"pitch_program\" requires schedule_s"
                    .to_owned(),
            })?;
        let pitch = self
            .pitch_rad
            .as_ref()
            .ok_or_else(|| ScenarioError::InvalidFc {
                reason: "fc.ascent_reference.method = \"pitch_program\" requires pitch_rad"
                    .to_owned(),
            })?;
        if schedule.len() != pitch.len() {
            return Err(ScenarioError::InvalidFc {
                reason: format!(
                    "fc.ascent_reference schedule_s and pitch_rad must have equal length (got {} and {})",
                    schedule.len(),
                    pitch.len()
                ),
            });
        }
        if schedule.len() < 2 {
            return Err(ScenarioError::InvalidFc {
                reason: "fc.ascent_reference pitch_program requires at least two schedule entries"
                    .to_owned(),
            });
        }
        for (index, value) in schedule.iter().enumerate() {
            require_finite(&format!("fc.ascent_reference.schedule_s[{index}]"), *value)?;
            if index > 0 && *value <= schedule[index - 1] {
                return Err(ScenarioError::InvalidNumber {
                    field: format!("fc.ascent_reference.schedule_s[{index}]"),
                    value: *value,
                    rule: "must be strictly increasing",
                });
            }
        }
        for (index, value) in pitch.iter().enumerate() {
            require_finite(&format!("fc.ascent_reference.pitch_rad[{index}]"), *value)?;
        }
        Ok(())
    }

    fn validate_gravity_turn(&self) -> Result<(), ScenarioError> {
        if self.schedule_s.is_some() {
            return Err(ScenarioError::UnexpectedField {
                field: "fc.ascent_reference.schedule_s".to_owned(),
                role: ModelRole::Controller,
                name: "gravity_turn".to_owned(),
            });
        }
        if self.pitch_rad.is_some() {
            return Err(ScenarioError::UnexpectedField {
                field: "fc.ascent_reference.pitch_rad".to_owned(),
                role: ModelRole::Controller,
                name: "gravity_turn".to_owned(),
            });
        }
        Ok(())
    }
}

impl FcConfig {
    /// Validates internal cross-fields.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError::InvalidFc`] when:
    /// - the selected single estimator, or any estimator lane when
    ///   `[fc.estimator_lanes]` is present, is missing its required
    ///   `[fc.*]` parameter block
    /// - `guidance = attitude_hold` and `reference_q_xyzw` is missing
    pub fn validate(&self) -> Result<(), ScenarioError> {
        if let Some(lanes) = self.estimator_lanes.as_ref() {
            lanes.validate()?;
            for (index, lane) in lanes.lanes.iter().enumerate() {
                self.validate_estimator_dependencies(
                    lane.estimator,
                    &format!("fc.estimator_lanes.lane[{index}].estimator"),
                )?;
            }
        } else {
            self.validate_estimator_dependencies(self.estimator, "estimator")?;
        }
        if let Some(imm) = self.imm.as_ref() {
            imm.validate()?;
        }
        if matches!(self.guidance, FcGuidanceKind::AttitudeHold) && self.reference_q_xyzw.is_none()
        {
            return Err(ScenarioError::InvalidFc {
                reason: "guidance = \"attitude_hold\" requires reference_q_xyzw".to_string(),
            });
        }
        if matches!(self.guidance, FcGuidanceKind::AscentReference) {
            let has_by_phase = self
                .ascent_reference_by_phase
                .as_ref()
                .is_some_and(|m| !m.is_empty());
            if self.ascent_reference.is_none() && !has_by_phase {
                return Err(ScenarioError::InvalidFc {
                    reason: "guidance = \"ascent_reference\" requires [fc.ascent_reference] or \
                             at least one [fc.ascent_reference_by_phase.<phase>]"
                        .to_owned(),
                });
            }
            if let Some(ascent_reference) = &self.ascent_reference {
                ascent_reference.validate()?;
            }
            if let Some(by_phase) = &self.ascent_reference_by_phase {
                for cfg in by_phase.values() {
                    cfg.validate()?;
                }
            }
        } else if self.ascent_reference.is_some() || self.ascent_reference_by_phase.is_some() {
            return Err(ScenarioError::InvalidFc {
                reason: "[fc.ascent_reference*] requires guidance = \"ascent_reference\""
                    .to_owned(),
            });
        }
        if self.base_rate_hz == 0 {
            return Err(ScenarioError::InvalidFc {
                reason: "base_rate_hz must be > 0".to_string(),
            });
        }
        if self.frame_budget_us == 0 {
            return Err(ScenarioError::InvalidFc {
                reason: "frame_budget_us must be > 0".to_string(),
            });
        }
        if let Some(scheduler) = self.scheduler.as_ref() {
            scheduler.validate(self.base_rate_hz)?;
        }
        if self.gain_schedule.is_empty() {
            return Err(ScenarioError::InvalidFc {
                reason: "[fc.gain_schedule] must declare at least one phase-specific tuning"
                    .to_string(),
            });
        }
        self.health.validate()?;
        if let Some(ekf) = &self.ekf {
            if let Some(v) = ekf.tau_gyro_bias_s {
                require_positive("fc.ekf.tau_gyro_bias_s", v)?;
            }
            if let Some(v) = ekf.tau_accel_bias_s {
                require_positive("fc.ekf.tau_accel_bias_s", v)?;
            }
            if let Some(v) = ekf.mag_epoch_decimal_year {
                require_finite("fc.ekf.mag_epoch_decimal_year", v)?;
            }
            if let Some(v) = ekf.high_dynamics_q_scale {
                require_positive("fc.ekf.high_dynamics_q_scale", v)?;
            }
        }
        if let Some(mekf) = &self.mekf
            && let Some(v) = mekf.tau_gyro_bias_s
        {
            require_positive("fc.mekf.tau_gyro_bias_s", v)?;
        }
        if let Some(mekf) = &self.mekf
            && let Some(v) = mekf.mag_epoch_decimal_year
        {
            require_finite("fc.mekf.mag_epoch_decimal_year", v)?;
        }
        if let Some(fdir) = &self.fdir {
            fdir.validate()?;
        }
        Ok(())
    }

    fn validate_estimator_dependencies(
        &self,
        estimator: FcEstimatorKind,
        selector: &str,
    ) -> Result<(), ScenarioError> {
        match estimator {
            FcEstimatorKind::Ekf => {
                if self.ekf.is_none() {
                    return Err(ScenarioError::InvalidFc {
                        reason: format!("{selector} = \"ekf\" requires [fc.ekf]"),
                    });
                }
            }
            FcEstimatorKind::Mekf => {
                if self.mekf.is_none() {
                    return Err(ScenarioError::InvalidFc {
                        reason: format!("{selector} = \"mekf\" requires [fc.mekf]"),
                    });
                }
            }
            FcEstimatorKind::Imm => {
                if self.imm.is_none() {
                    return Err(ScenarioError::InvalidFc {
                        reason: format!("{selector} = \"imm\" requires [fc.imm]"),
                    });
                }
                if self.ekf.is_none() {
                    return Err(ScenarioError::InvalidFc {
                        reason: format!(
                            "{selector} = \"imm\" requires [fc.ekf] for the per-mode base \
                             EKF parameters; per-mode overrides go under [[fc.imm.mode]]"
                        ),
                    });
                }
            }
            FcEstimatorKind::SrUkf | FcEstimatorKind::SrUkfAttitude => {
                if self.ekf.is_none() {
                    return Err(ScenarioError::InvalidFc {
                        reason: format!(
                            "{selector} = \"{}\" requires [fc.ekf] for the noise budget",
                            fc_estimator_kind_name(estimator)
                        ),
                    });
                }
            }
        }
        Ok(())
    }
}

/// Supported estimator kinds for the FC scenario block.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcEstimatorKind {
    /// 15-state error-state Extended Kalman Filter.
    Ekf,
    /// 6-state Multiplicative EKF (attitude + gyro bias).
    Mekf,
    /// Bar-Shalom IMM (Interacting Multiple Model)
    /// estimator over a bank of 2 to 4 mode-conditioned EKFs.
    /// Requires `[fc.imm]` block; `[fc.ekf]` is used as the per-mode
    /// base parameters (further refined by per-mode overrides under
    /// `[[fc.imm.mode]]`).
    Imm,
    /// Square-Root Unscented Kalman Filter (15-state),
    /// per Van der Merwe & Wan 2001. Uses the same parameter set as
    /// the EKF (`[fc.ekf]` block).
    SrUkf,
    /// Square-Root UKF restricted to the 6-state
    /// attitude + gyro-bias subspace; mirrors the retired
    /// classical 6-state `Ukf` shape but on the new square-root
    /// machinery.
    SrUkfAttitude,
}

fn fc_estimator_kind_name(kind: FcEstimatorKind) -> &'static str {
    match kind {
        FcEstimatorKind::Ekf => "ekf",
        FcEstimatorKind::Mekf => "mekf",
        FcEstimatorKind::Imm => "imm",
        FcEstimatorKind::SrUkf => "sr_ukf",
        FcEstimatorKind::SrUkfAttitude => "sr_ukf_attitude",
    }
}

fn v3_only_fc_estimator_field(kind: FcEstimatorKind) -> Option<&'static str> {
    match kind {
        FcEstimatorKind::Imm => Some("fc.estimator = \"imm\""),
        FcEstimatorKind::SrUkf => Some("fc.estimator = \"sr_ukf\""),
        FcEstimatorKind::SrUkfAttitude => Some("fc.estimator = \"sr_ukf_attitude\""),
        FcEstimatorKind::Ekf | FcEstimatorKind::Mekf => None,
    }
}

fn v3_only_fc_guidance_field(kind: FcGuidanceKind) -> Option<&'static str> {
    match kind {
        FcGuidanceKind::AscentReference => Some("fc.guidance = \"ascent_reference\""),
        FcGuidanceKind::AttitudeHold | FcGuidanceKind::Waypoint => None,
    }
}

/// Supported autopilot kinds.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcAutopilotKind {
    /// Stevens & Lewis 2015 academic three-loop autopilot.
    ThreeLoop,
}

/// Supported guidance kinds.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcGuidanceKind {
    /// Constant-attitude reference.
    AttitudeHold,
    /// Scenario-defined inertial-waypoint navigation.
    Waypoint,
    /// Powered-ascent reference attitude generated from vehicle state.
    AscentReference,
}

/// Supported FC magnetic-field models.
#[derive(Copy, Clone, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcMagFieldKind {
    /// Degree-1 academic dipole baseline.
    #[default]
    EarthDipole,
    /// NOAA / NCEI WMM 2025 spherical-harmonic field.
    #[serde(rename = "wmm_2025")]
    Wmm2025,
}

/// EKF navigation gravity model selector.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcGravityModelKind {
    /// Uniform −z gravity at standard magnitude. Byte-identical to the
    /// historical EKF behaviour; the default.
    #[default]
    ConstantZ,
    /// Newtonian point-mass central gravity, WGS84 µ.
    PointMass,
    /// Point-mass plus the J2 zonal harmonic (WGS84 µ, R_e, J2).
    J2,
    /// EGM2008 zonal harmonics (degrees 2–6), matching the runner's
    /// `egm2008` truth gravity.
    Egm2008,
}

/// EKF parameter overrides.
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcEkfConfig {
    /// Magnetic-field model used by magnetometer prediction.
    pub mag_field: Option<FcMagFieldKind>,
    /// Navigation gravity model used in the EKF velocity propagation.
    /// Defaults to `constant_z` (uniform −z gravity), which is accurate
    /// near a fixed launch site but mismatches a central field as the
    /// vehicle travels. Use `point_mass`, `j2`, or `egm2008` for ascent
    /// / orbital trajectories so the nav estimate stays valid as the
    /// gravity direction rotates with position.
    pub gravity_model: Option<FcGravityModelKind>,
    /// Scenario-start decimal year for WMM secular variation.
    /// Defaults to 2025.0 when `mag_field = "wmm_2025"`.
    pub mag_epoch_decimal_year: Option<f64>,
    /// Process-noise stddev on attitude rate (rad/s).
    pub sigma_w_gyro: Option<f64>,
    /// Process-noise stddev on accel-bias random walk (m/s²/√s).
    pub sigma_w_accel_bias: Option<f64>,
    /// Process-noise stddev on gyro-bias random walk (rad/s/√s).
    pub sigma_w_gyro_bias: Option<f64>,
    /// Process-noise stddev on the position state (m/√s). Keeps the
    /// position covariance from collapsing so GNSS keeps correcting.
    pub sigma_w_position_m: Option<f64>,
    /// Process-noise stddev on the velocity state (m/s/√s). Prevents
    /// velocity-covariance collapse during high-dynamics flight, which
    /// otherwise makes the filter ignore GNSS velocity and drift through
    /// a subsequent low-specific-force coast.
    pub sigma_w_velocity_m_s: Option<f64>,
    /// First-order Gauss-Markov gyro-bias time constant (s).
    pub tau_gyro_bias_s: Option<f64>,
    /// First-order Gauss-Markov accelerometer-bias time constant (s).
    pub tau_accel_bias_s: Option<f64>,
    /// Measurement-noise stddev on each GNSS position component (m).
    pub sigma_gnss_pos_m: Option<f64>,
    /// Measurement-noise stddev on each GNSS velocity component (m/s).
    pub sigma_gnss_vel_m_s: Option<f64>,
    /// Measurement-noise stddev on barometer altitude (m).
    pub sigma_baro_alt_m: Option<f64>,
    /// Measurement-noise stddev on each magnetometer component (nT).
    pub sigma_mag_nt: Option<f64>,
    /// Legacy explicit innovation-gate chi-square threshold. Prefer
    /// `innovation_false_alarm_rate` so the controller derives the
    /// correct gate for each measurement dimension.
    pub innovation_gate: Option<f64>,
    /// False-alarm probability used for chi-square innovation gates.
    pub innovation_false_alarm_rate: Option<f64>,
    /// Dead-reckoning timeout (s).
    pub dead_reckon_timeout_s: Option<f64>,
    /// Multiplier applied to EKF position/velocity process-noise
    /// covariance while the commander-published `estimator_regime`
    /// region is `boost_mode`. `1.0` preserves the base schedule.
    pub high_dynamics_q_scale: Option<f64>,
}

/// MEKF parameter overrides.
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcMekfConfig {
    /// Magnetic-field model used by magnetometer prediction.
    pub mag_field: Option<FcMagFieldKind>,
    /// Scenario-start decimal year for WMM secular variation.
    /// Defaults to 2025.0 when `mag_field = "wmm_2025"`.
    pub mag_epoch_decimal_year: Option<f64>,
    /// Process-noise stddev on attitude rate (rad/s).
    pub sigma_w_gyro: Option<f64>,
    /// Process-noise stddev on gyro-bias random walk (rad/s/√s).
    pub sigma_w_gyro_bias: Option<f64>,
    /// First-order Gauss-Markov gyro-bias time constant (s).
    pub tau_gyro_bias_s: Option<f64>,
    /// Measurement-noise stddev on each magnetometer component (nT).
    pub sigma_mag_nt: Option<f64>,
    /// Legacy explicit innovation-gate chi-square threshold. Prefer
    /// `innovation_false_alarm_rate`.
    pub innovation_gate: Option<f64>,
    /// False-alarm probability used for chi-square innovation gates.
    pub innovation_false_alarm_rate: Option<f64>,
}

/// Bar-Shalom IMM bank. Required when
/// `[fc].estimator = "imm"`.
///
/// Each `[[fc.imm.mode]]` entry overrides the per-mode process-noise
/// tuning on top of the base `[fc.ekf]` block. The transition matrix
/// must be square (rows = N = number of modes), each row sums to 1
/// within `1e-9`, and `2 ≤ N ≤ 4`. The `initial_mode_probabilities`
/// vector must also sum to 1 within `1e-9`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcImmConfig {
    /// Markov mode-transition matrix `Π_ij = P(mode j next | mode i
    /// now)`. Square (N × N) with `2 ≤ N ≤ 4`; each row sums to 1
    /// within `1e-9`.
    pub transition_matrix: Vec<Vec<f64>>,
    /// Initial mode probabilities `μ_i^0`. Length N; sums to 1 within
    /// `1e-9`.
    pub initial_mode_probabilities: Vec<f64>,
    /// Per-mode tuning. Length must equal `transition_matrix.len()`.
    /// Each entry overrides selected fields of the base `[fc.ekf]`
    /// parameters; unset fields fall back to the base.
    #[serde(default, rename = "mode")]
    pub modes: Vec<FcImmModeConfig>,
}

impl FcImmConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        let n = self.transition_matrix.len();
        if !(2..=4).contains(&n) {
            return Err(ScenarioError::InvalidFc {
                reason: format!("fc.imm.transition_matrix must have between 2 and 4 rows; got {n}"),
            });
        }
        for (i, row) in self.transition_matrix.iter().enumerate() {
            if row.len() != n {
                return Err(ScenarioError::InvalidFc {
                    reason: format!(
                        "fc.imm.transition_matrix row {i} has {} entries, expected {n} \
                         (matrix must be square)",
                        row.len()
                    ),
                });
            }
            let row_sum: f64 = row.iter().sum();
            if (row_sum - 1.0).abs() > 1.0e-9 {
                return Err(ScenarioError::InvalidFc {
                    reason: format!(
                        "fc.imm.transition_matrix row {i} sums to {row_sum}, must equal 1.0 \
                         within 1e-9"
                    ),
                });
            }
            for (j, &p) in row.iter().enumerate() {
                if !(0.0..=1.0).contains(&p) || !p.is_finite() {
                    return Err(ScenarioError::InvalidFc {
                        reason: format!(
                            "fc.imm.transition_matrix[{i}][{j}] = {p} must lie in [0, 1]"
                        ),
                    });
                }
            }
        }
        if self.initial_mode_probabilities.len() != n {
            return Err(ScenarioError::InvalidFc {
                reason: format!(
                    "fc.imm.initial_mode_probabilities has {} entries, must equal \
                     transition_matrix size N = {n}",
                    self.initial_mode_probabilities.len()
                ),
            });
        }
        let prob_sum: f64 = self.initial_mode_probabilities.iter().sum();
        if (prob_sum - 1.0).abs() > 1.0e-9 {
            return Err(ScenarioError::InvalidFc {
                reason: format!(
                    "fc.imm.initial_mode_probabilities sum to {prob_sum}, must equal 1.0 \
                     within 1e-9"
                ),
            });
        }
        for (i, &p) in self.initial_mode_probabilities.iter().enumerate() {
            if !(0.0..=1.0).contains(&p) || !p.is_finite() {
                return Err(ScenarioError::InvalidFc {
                    reason: format!(
                        "fc.imm.initial_mode_probabilities[{i}] = {p} must lie in [0, 1]"
                    ),
                });
            }
        }
        if self.modes.len() != n {
            return Err(ScenarioError::InvalidFc {
                reason: format!(
                    "fc.imm.mode count is {}, must equal transition_matrix size N = {n}",
                    self.modes.len()
                ),
            });
        }
        for (i, mode) in self.modes.iter().enumerate() {
            mode.validate(i)?;
        }
        Ok(())
    }
}

/// Per-mode tuning override under `[[fc.imm.mode]]`.
///
/// Fields left unset fall back to the base `[fc.ekf]` block; setting
/// a field overrides that field for this mode only.
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcImmModeConfig {
    /// Override on `EkfParams::sigma_w_gyro` for this mode.
    pub sigma_w_gyro: Option<f64>,
    /// Override on `EkfParams::sigma_w_gyro_bias`.
    pub sigma_w_gyro_bias: Option<f64>,
    /// Override on `EkfParams::sigma_w_accel_bias`.
    pub sigma_w_accel_bias: Option<f64>,
    /// Override on `EkfParams::tau_gyro_bias_s`.
    pub tau_gyro_bias_s: Option<f64>,
    /// Override on `EkfParams::tau_accel_bias_s`.
    pub tau_accel_bias_s: Option<f64>,
}

impl FcImmModeConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        for (label, value) in [
            ("sigma_w_gyro", self.sigma_w_gyro),
            ("sigma_w_gyro_bias", self.sigma_w_gyro_bias),
            ("sigma_w_accel_bias", self.sigma_w_accel_bias),
            ("tau_gyro_bias_s", self.tau_gyro_bias_s),
            ("tau_accel_bias_s", self.tau_accel_bias_s),
        ] {
            if let Some(v) = value {
                require_positive(&format!("fc.imm.mode[{index}].{label}"), v)?;
            }
        }
        Ok(())
    }
}

/// Autopilot params overrides.
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcAutopilotParams {
    /// Rate-loop integrator deadband (rad/s).
    pub rate_deadband_rad_s: Option<f64>,
    /// Steer by engine-gimbal thrust-vector control (TVC). When `true`,
    /// the autopilot emits its rate-loop pitch/yaw command as the
    /// engine `gimbal_pitch_rad`/`gimbal_yaw_rad` (clamped to the
    /// gain-schedule limits). Defaults to `false` (aerodynamic
    /// effectors only; zero gimbal).
    pub thrust_vector_control: Option<bool>,
    /// Settling time (s) before TVC gimbal output engages. Holds the
    /// gimbal at zero (open-loop axial thrust) until the nav estimate
    /// converges, suppressing an EKF-init transient. Defaults to `0.0`.
    pub thrust_vector_settle_s: Option<f64>,
    /// Maximum dynamic pressure (Pa) for closed-loop max-Q load relief.
    /// The autopilot throttles down toward `q_max/q` whenever the real
    /// dynamic pressure (density at the navigated geocentric altitude ×
    /// air-relative speed²) exceeds this. Omit for no limit.
    pub max_dynamic_pressure_pa: Option<f64>,
    /// Whether to enable the trajectory loop.
    pub trajectory_loop_enabled: Option<bool>,
    /// Trajectory-loop strategy.
    pub trajectory_kind: Option<FcTrajectoryKind>,
    /// Optional Cao-Hovakimyan L1 adaptive rate-loop augmentation
    /// (v3-only). When `Some`, the runner installs a
    /// per-axis `L1AdaptiveChannel` augmentation on the rate loop;
    /// the FC's `l1-adaptive` feature flag must be on for the
    /// augmentation to compile.
    pub l1_adaptive: Option<FcL1AdaptiveConfig>,
    /// Optional anti-windup strategy declaration (v3-only). Selects
    /// between back-calculation and observer-form integrator bleeding.
    /// When absent, the runner uses `AntiWindupKind::default()`
    /// (back-calculation, unit gain).
    pub anti_windup: Option<FcAntiWindupConfig>,
    /// Rate-loop dispatch strategy (v3-only). When
    /// `Some(FcRateLoopKind::Lqr)`, the runner solves the per-axis
    /// DARE using `[fc.autopilot_params.lqr]` and installs the LQR
    /// gains on the autopilot. Defaults to `Pid`.
    /// LQR runner support is limited to single-body
    /// diagonal inertia.
    pub rate_loop_kind: Option<FcRateLoopKind>,
    /// Per-axis LQR cost weights (v3-only). Required
    /// when `rate_loop_kind = "lqr"`; ignored otherwise.
    pub lqr: Option<FcLqrConfig>,
    /// Per-axis INDI parameters (v3-only). Required
    /// when `rate_loop_kind = "indi"`; ignored otherwise. Composition
    /// with `[fc.autopilot_params.l1_adaptive]` is rejected at
    /// scenario load.
    pub indi: Option<FcIndiConfig>,
    /// Attitude-loop dispatch (v3-only). When
    /// `Some(FcAttitudeLoopKind::Mpc)`, the runner builds a
    /// `RecedingHorizonAttitudeMpc` from `[fc.autopilot_params.attitude_mpc]`
    /// and installs it on the autopilot. Defaults to `Pid`.
    pub attitude_loop_kind: Option<FcAttitudeLoopKind>,
    /// Receding-horizon attitude-MPC parameters
    /// (v3-only). Required when `attitude_loop_kind = "mpc"`; ignored
    /// otherwise.
    pub attitude_mpc: Option<FcAttitudeMpcConfig>,
    /// Optional per-axis gyro notch filters, applied to the body-rate
    /// feedback before the rate loop — the standard slosh / structural-flex
    /// GAIN-STABILISATION technique (attenuate the rate loop's response in
    /// the slosh/flex band so the controller does not chase, and amplify,
    /// the oscillation). `[roll, pitch, yaw]` order. Omit for no notch.
    #[serde(default)]
    pub gyro_notch: Option<[FcGyroNotchConfig; 3]>,
}

/// One axis' gyro notch-filter configuration
/// (`[fc.autopilot_params.gyro_notch]`).
#[derive(Copy, Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcGyroNotchConfig {
    /// Notch centre frequency (Hz), strictly positive. Set near the slosh
    /// or first structural-bending frequency to be rejected.
    pub center_hz: f64,
    /// Notch bandwidth (Hz), strictly positive.
    pub bandwidth_hz: f64,
    /// Requested notch depth (dB), non-negative.
    pub depth_db: f64,
}

impl FcGyroNotchConfig {
    fn validate(&self, axis: usize) -> Result<(), ScenarioError> {
        let path = |f: &str| format!("fc.autopilot_params.gyro_notch[{axis}].{f}");
        require_finite(&path("center_hz"), self.center_hz)?;
        require_positive(&path("center_hz"), self.center_hz)?;
        require_finite(&path("bandwidth_hz"), self.bandwidth_hz)?;
        require_positive(&path("bandwidth_hz"), self.bandwidth_hz)?;
        require_finite(&path("depth_db"), self.depth_db)?;
        if self.depth_db < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("depth_db"),
                value: self.depth_db,
                rule: "must be non-negative",
            });
        }
        Ok(())
    }
}

/// Per-axis L1 adaptive parameters declared in
/// `[fc.autopilot_params.l1_adaptive]` (v3-only).
///
/// The fields mirror `openbmp_fc::l1_adaptive_full::L1AdaptiveParams`
/// one-for-one and are validated against the bandwidth-projection
/// inequality `ω_c · L < 1` at scenario load.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcL1AdaptiveConfig {
    /// Reference-model bandwidth `a_m` (rad/s). Must be `< 0`.
    pub reference_model_a_m: f64,
    /// Reference-model / state-predictor input gain `b`. Must be
    /// non-zero.
    pub reference_model_b: f64,
    /// Reference-model feedforward gain `k_g`.
    pub reference_model_k_g: f64,
    /// PCA sample time `T_s` (s). Must be `> 0`.
    pub adaptation_sample_time_s: f64,
    /// Strictly-proper LPF cutoff `ω_c` (rad/s). Must be `> 0`.
    pub low_pass_cutoff_rad_s: f64,
    /// Lipschitz bound `L` on the matched uncertainty. Must be `> 0`
    /// and satisfy `ω_c · L < 1`.
    pub lipschitz_bound: f64,
    /// Symmetric projection bound on `σ̂`. Must be `> 0`.
    pub projection_bound: f64,
}

impl FcL1AdaptiveConfig {
    fn validate(&self, dt_s: f64) -> Result<(), ScenarioError> {
        let path = "fc.autopilot_params.l1_adaptive";
        require_finite(
            &format!("{path}.reference_model_a_m"),
            self.reference_model_a_m,
        )?;
        require_finite(&format!("{path}.reference_model_b"), self.reference_model_b)?;
        require_finite(
            &format!("{path}.reference_model_k_g"),
            self.reference_model_k_g,
        )?;
        require_finite(
            &format!("{path}.adaptation_sample_time_s"),
            self.adaptation_sample_time_s,
        )?;
        require_finite(
            &format!("{path}.low_pass_cutoff_rad_s"),
            self.low_pass_cutoff_rad_s,
        )?;
        require_finite(&format!("{path}.lipschitz_bound"), self.lipschitz_bound)?;
        require_finite(&format!("{path}.projection_bound"), self.projection_bound)?;
        if self.reference_model_a_m >= 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.reference_model_a_m"),
                value: self.reference_model_a_m,
                rule: "must be strictly negative for the reference model to be stable",
            });
        }
        if self.reference_model_b == 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.reference_model_b"),
                value: self.reference_model_b,
                rule: "must be non-zero",
            });
        }
        require_positive(
            &format!("{path}.adaptation_sample_time_s"),
            self.adaptation_sample_time_s,
        )?;
        require_positive(
            &format!("{path}.low_pass_cutoff_rad_s"),
            self.low_pass_cutoff_rad_s,
        )?;
        require_positive(&format!("{path}.lipschitz_bound"), self.lipschitz_bound)?;
        require_positive(&format!("{path}.projection_bound"), self.projection_bound)?;
        let product = self.low_pass_cutoff_rad_s * self.lipschitz_bound;
        if product >= 1.0 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.low_pass_cutoff_rad_s · lipschitz_bound"),
                value: product,
                rule: "Cao-Hovakimyan bandwidth-projection inequality requires ω_c · L < 1",
            });
        }
        let max_dt_s = -2.0 / self.reference_model_a_m;
        if dt_s >= max_dt_s {
            return Err(ScenarioError::InvalidNumber {
                field: "time.dt_s".to_owned(),
                value: dt_s,
                rule: "must satisfy time.dt_s < -2 / fc.autopilot_params.l1_adaptive.reference_model_a_m for forward-Euler L1 stability",
            });
        }
        Ok(())
    }
}

/// Anti-windup strategy declared in
/// `[fc.autopilot_params.anti_windup]` (v3-only).
///
/// Mirrors `openbmp_fc::anti_windup::AntiWindupKind`. The two
/// variants are mathematically equivalent on a SISO PID (with
/// `back_calculation.gain = 1 / observer_form.tracking_time_s`) but
/// expose distinct design intents — empirical gain tuning vs
/// observer pole placement.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum FcAntiWindupConfig {
    /// Åström-Wittenmark 1984 back-calculation. The integrator is
    /// bled by `gain * excess * dt` whenever the unsaturated PID
    /// command exceeds the actuator limit. `gain` must be `> 0`.
    BackCalculation {
        /// Back-calculation gain `k_aw` (unitless). Must be `> 0`.
        gain: f64,
    },
    /// Åström-Rundqwist 1989 observer-form anti-windup. The
    /// integrator is bled by `excess * dt / tracking_time_s` where
    /// `tracking_time_s` is the observer time constant (seconds).
    /// Must be `> 0`.
    ObserverForm {
        /// Observer tracking time constant `T_t` (seconds). Must be
        /// `> 0`.
        tracking_time_s: f64,
    },
}

impl FcAntiWindupConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        let path = "fc.autopilot_params.anti_windup";
        match *self {
            Self::BackCalculation { gain } => {
                require_finite(&format!("{path}.gain"), gain)?;
                require_positive(&format!("{path}.gain"), gain)?;
            }
            Self::ObserverForm { tracking_time_s } => {
                require_finite(&format!("{path}.tracking_time_s"), tracking_time_s)?;
                require_positive(&format!("{path}.tracking_time_s"), tracking_time_s)?;
            }
        }
        Ok(())
    }
}

/// Rate-loop dispatch declared in
/// `[fc.autopilot_params.rate_loop_kind]` (v3-only).
///
/// Mirrors `openbmp_fc::autopilot::RateLoopKind`.
#[derive(Copy, Clone, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcRateLoopKind {
    /// PID rate loop. Default.
    #[default]
    Pid,
    /// Per-axis LQR rate loop. Requires a populated
    /// `[fc.autopilot_params.lqr]` block; the runner solves the
    /// per-axis DARE at scenario load using the diagonal inertia of
    /// a single-body assembly.
    Lqr,
    /// Per-axis INDI rate loop (Smeur-Chu-de Croon
    /// 2016). Requires a populated `[fc.autopilot_params.indi]`
    /// block. Single-body assembly with diagonal inertia only;
    /// composition with `[fc.autopilot_params.l1_adaptive]` is
    /// rejected at scenario load.
    Indi,
}

/// Synchronised filter shape for INDI's ω and u filters
/// (v3-only). Mirrors
/// `openbmp_fc::indi::IndiFilterKind`.
#[derive(Copy, Clone, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcIndiFilterKind {
    /// Bilinear-transform first-order low-pass.
    FirstOrderLowPass,
    /// Bilinear-transform second-order Butterworth low-pass
    /// (Smeur 2016 default).
    #[default]
    SecondOrderButterworth,
}

/// Per-axis INDI configuration declared in
/// `[fc.autopilot_params.indi]` (v3-only).
///
/// Each `[f64; 3]` is `[roll, pitch, yaw]`. The runner validates
/// the filter cutoff against the loop step `time.dt_s` (must be
/// strictly below `π / dt_s`).
#[derive(Copy, Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcIndiConfig {
    /// INDI's working estimate of body-axis diagonal inertia (kg·m²).
    /// May intentionally differ from the truth-side vehicle inertia
    /// — INDI's robustness rests on tolerating that mismatch.
    pub inertia_per_axis_kg_m2: [f64; 3],
    /// Per-axis control effectiveness `g` such that
    /// `Δτ_axis = g_axis · Δu_axis`. For `direct_torque` effectors
    /// with effectiveness 1 N·m / rad and 1:1 channel mapping this
    /// is `[1.0, 1.0, 1.0]`.
    pub control_effectiveness_per_axis: [f64; 3],
    /// Cutoff (rad/s) applied identically to the ω and u filters.
    /// Must be `> 0` and strictly below the discrete Nyquist
    /// boundary `π / time.dt_s`.
    pub filter_cutoff_rad_s: f64,
    /// Filter shape; see [`FcIndiFilterKind`]. Defaults to
    /// `second_order_butterworth`.
    #[serde(default)]
    pub filter_kind: FcIndiFilterKind,
    /// Outer-loop attitude-to-angular-acceleration P-gain per axis
    /// (`ω̇_des = K_p · (ω_ref − ω_meas)`). Each entry must be `> 0`.
    pub attitude_to_omega_dot_gain: [f64; 3],
}

impl FcIndiConfig {
    fn validate(&self, dt_s: f64) -> Result<(), ScenarioError> {
        let path = "fc.autopilot_params.indi";
        for (axis_label, axis) in ["roll", "pitch", "yaw"].iter().zip(0..3) {
            require_finite(
                &format!("{path}.inertia_per_axis_kg_m2[{axis_label}]"),
                self.inertia_per_axis_kg_m2[axis],
            )?;
            require_positive(
                &format!("{path}.inertia_per_axis_kg_m2[{axis_label}]"),
                self.inertia_per_axis_kg_m2[axis],
            )?;
            require_finite(
                &format!("{path}.control_effectiveness_per_axis[{axis_label}]"),
                self.control_effectiveness_per_axis[axis],
            )?;
            require_positive(
                &format!("{path}.control_effectiveness_per_axis[{axis_label}]"),
                self.control_effectiveness_per_axis[axis],
            )?;
            require_finite(
                &format!("{path}.attitude_to_omega_dot_gain[{axis_label}]"),
                self.attitude_to_omega_dot_gain[axis],
            )?;
            require_positive(
                &format!("{path}.attitude_to_omega_dot_gain[{axis_label}]"),
                self.attitude_to_omega_dot_gain[axis],
            )?;
        }
        require_finite(
            &format!("{path}.filter_cutoff_rad_s"),
            self.filter_cutoff_rad_s,
        )?;
        require_positive(
            &format!("{path}.filter_cutoff_rad_s"),
            self.filter_cutoff_rad_s,
        )?;
        let nyquist_rad_s = std::f64::consts::PI / dt_s;
        if self.filter_cutoff_rad_s >= nyquist_rad_s {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.filter_cutoff_rad_s"),
                value: self.filter_cutoff_rad_s,
                rule: "must be strictly below π / time.dt_s (discrete Nyquist) so the bilinear-\
                       transform LPF is well-posed",
            });
        }
        Ok(())
    }
}

/// Per-axis LQR cost weights declared in
/// `[fc.autopilot_params.lqr]` (v3-only).
///
/// Each `[f64; 3]` is `[roll, pitch, yaw]` and must contain
/// strictly positive values. The runner translates these weights to
/// `openbmp_fc::lqr::solve_lqr_rate_loop` per axis using the
/// diagonal inertia of a single-body assembly and the loop step
/// `time.dt_s`.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcLqrConfig {
    /// Per-axis state cost on the rate error `(ω − ω_ref)`. Larger
    /// → tighter rate tracking but bigger control effort.
    pub q_omega: [f64; 3],
    /// Per-axis state cost on the integrated rate error
    /// `∫(ω − ω_ref) dt`. Larger → faster zero-steady-state-error
    /// recovery from disturbances.
    pub q_int: [f64; 3],
    /// Per-axis control cost. Larger → less aggressive torque
    /// commands.
    pub r: [f64; 3],
}

impl FcLqrConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        let path = "fc.autopilot_params.lqr";
        for (axis_label, axis) in ["roll", "pitch", "yaw"].iter().zip(0..3) {
            require_finite(&format!("{path}.q_omega[{axis_label}]"), self.q_omega[axis])?;
            require_positive(&format!("{path}.q_omega[{axis_label}]"), self.q_omega[axis])?;
            require_finite(&format!("{path}.q_int[{axis_label}]"), self.q_int[axis])?;
            require_positive(&format!("{path}.q_int[{axis_label}]"), self.q_int[axis])?;
            require_finite(&format!("{path}.r[{axis_label}]"), self.r[axis])?;
            require_positive(&format!("{path}.r[{axis_label}]"), self.r[axis])?;
        }
        Ok(())
    }
}

/// Attitude-loop dispatch declared in
/// `[fc.autopilot_params.attitude_loop_kind]` (v3-only).
///
/// Mirrors `openbmp_fc::autopilot::AttitudeLoopKind`.
#[derive(Copy, Clone, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcAttitudeLoopKind {
    /// Per-axis PID attitude loop. Default.
    #[default]
    Pid,
    /// Receding-horizon attitude MPC. Requires a
    /// populated `[fc.autopilot_params.attitude_mpc]` block; the
    /// runner builds a `RecedingHorizonAttitudeMpc` at scenario load
    /// using the loop step `time.dt_s`.
    Mpc,
}

/// Receding-horizon attitude-MPC configuration declared in
/// `[fc.autopilot_params.attitude_mpc]` (v3-only).
///
/// Mirrors `openbmp_fc::mpc::AttitudeMpcParams`. Each `[f64; 3]` is
/// `[roll, pitch, yaw]` and must contain strictly positive values.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcAttitudeMpcConfig {
    /// Number of horizon steps `N`. Must be `>= 1`. Bounded above
    /// by `openbmp_fc::mpc::ATTITUDE_MPC_MAX_HORIZON` at runtime.
    pub horizon_n: usize,
    /// Per-axis stage cost on the small-angle attitude error
    /// (rad²-weight). Must be `> 0`.
    pub q_x: [f64; 3],
    /// Per-axis stage cost on the commanded body rate
    /// ((rad/s)²-weight). Must be `> 0` for strong convexity.
    pub r_u: [f64; 3],
    /// Per-axis terminal cost on the final attitude error. Must be
    /// `> 0`.
    pub terminal_p: [f64; 3],
    /// Per-axis symmetric rate-command bound (rad/s) the MPC
    /// enforces on every horizon step. Must be `> 0`.
    pub rate_limit_rad_s: [f64; 3],
}

impl FcAttitudeMpcConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        let path = "fc.autopilot_params.attitude_mpc";
        if self.horizon_n == 0 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("{path}.horizon_n"),
                value: 0.0,
                rule: "must be >= 1",
            });
        }
        for (axis_label, axis) in ["roll", "pitch", "yaw"].iter().zip(0..3) {
            require_finite(&format!("{path}.q_x[{axis_label}]"), self.q_x[axis])?;
            require_positive(&format!("{path}.q_x[{axis_label}]"), self.q_x[axis])?;
            require_finite(&format!("{path}.r_u[{axis_label}]"), self.r_u[axis])?;
            require_positive(&format!("{path}.r_u[{axis_label}]"), self.r_u[axis])?;
            require_finite(
                &format!("{path}.terminal_p[{axis_label}]"),
                self.terminal_p[axis],
            )?;
            require_positive(
                &format!("{path}.terminal_p[{axis_label}]"),
                self.terminal_p[axis],
            )?;
            require_finite(
                &format!("{path}.rate_limit_rad_s[{axis_label}]"),
                self.rate_limit_rad_s[axis],
            )?;
            require_positive(
                &format!("{path}.rate_limit_rad_s[{axis_label}]"),
                self.rate_limit_rad_s[axis],
            )?;
        }
        Ok(())
    }
}

/// Supported trajectory-loop kinds.
#[derive(Copy, Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcTrajectoryKind {
    /// Existing PID trajectory loop.
    Pid,
    /// Mellinger & Kumar 2011 minimum-snap differential-flatness
    /// trajectory tracker. Requires a `[fc.trajectory]` block (v3 only)
    /// declaring the waypoint sequence and yaw profile that the
    /// trajectory generator solves at scenario load.
    MinimumSnap,
}

/// FC health-monitor thresholds.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcHealthConfig {
    /// Maximum allowed IMU sample age (s).
    pub imu_stale_after_s: f64,
    /// Maximum allowed GNSS sample age (s).
    pub gnss_stale_after_s: f64,
    /// Maximum allowed barometer sample age (s).
    pub baro_stale_after_s: f64,
    /// Maximum allowed magnetometer sample age (s).
    pub mag_stale_after_s: f64,
    /// Consecutive scheduler overruns before health trips.
    pub overrun_burst_count: u32,
}

impl FcHealthConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        for (name, value) in [
            ("imu_stale_after_s", self.imu_stale_after_s),
            ("gnss_stale_after_s", self.gnss_stale_after_s),
            ("baro_stale_after_s", self.baro_stale_after_s),
            ("mag_stale_after_s", self.mag_stale_after_s),
        ] {
            require_positive(&format!("fc.health.{name}"), value)?;
        }
        if self.overrun_burst_count == 0 {
            return Err(ScenarioError::InvalidFc {
                reason: "fc.health.overrun_burst_count must be > 0".to_string(),
            });
        }
        Ok(())
    }
}

/// FC FDIR detector configuration.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcFdirConfig {
    /// Detector family.
    pub detector_kind: FcFdirDetectorKind,
    /// Innovation chi-square threshold.
    pub innovation_threshold: Option<f64>,
    /// Consecutive innovation breaches before burst-counter trips.
    pub innovation_burst_count: Option<u32>,
    /// Consecutive failsafe breaches before burst-counter trips.
    pub failsafe_burst_count: Option<u32>,
    /// CUSUM drift term.
    pub cusum_drift: Option<f64>,
    /// CUSUM trip threshold.
    pub cusum_threshold: Option<f64>,
    /// Optional detector tuning block (`[fc.fdir.detector]`).
    ///
    /// v3 only. The windowed-mean-shift GLRT
    /// and parity-space residual generator consume it.
    /// Scenarios that declare `[fc.fdir.detector]` must
    /// have `openbmp.scenario = 3`.
    pub detector: Option<FcFdirDetectorConfig>,
    /// Optional declarative redline watchpoint block
    /// (`[fc.fdir.redlines]`).
    ///
    /// v3 only. The runner maps these I-loads into `FdirParams`.
    pub redlines: Option<FcFdirRedlinesConfig>,
}

impl FcFdirConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if let Some(v) = self.innovation_threshold {
            require_positive("fc.fdir.innovation_threshold", v)?;
        }
        if let Some(v) = self.innovation_burst_count
            && v == 0
        {
            return Err(ScenarioError::InvalidFc {
                reason: "fc.fdir.innovation_burst_count must be > 0".to_string(),
            });
        }
        if let Some(v) = self.failsafe_burst_count
            && v == 0
        {
            return Err(ScenarioError::InvalidFc {
                reason: "fc.fdir.failsafe_burst_count must be > 0".to_string(),
            });
        }
        if let Some(v) = self.cusum_drift {
            require_finite("fc.fdir.cusum_drift", v)?;
        }
        if let Some(v) = self.cusum_threshold {
            require_positive("fc.fdir.cusum_threshold", v)?;
        }
        Ok(())
    }
}

/// FDIR redline watchpoints (`[fc.fdir.redlines]`, v3 only).
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcFdirRedlinesConfig {
    /// Body-rate magnitude redline (rad/s). When exceeded, FDIR
    /// latches the body-rate redline fault bit.
    pub body_rate_rad_s: Option<f64>,
}

impl FcFdirRedlinesConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if self.body_rate_rad_s.is_none() {
            return Err(ScenarioError::InvalidFc {
                reason: "fc.fdir.redlines must declare at least one redline".to_string(),
            });
        }
        if let Some(v) = self.body_rate_rad_s {
            require_positive("fc.fdir.redlines.body_rate_rad_s", v)?;
        }
        Ok(())
    }
}

/// FDIR detector families.
#[derive(Copy, Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcFdirDetectorKind {
    /// Existing burst-counter detector.
    BurstCounter,
    /// Single-sample GLRT detector. The chi-square innovation
    /// statistic published by the estimator is the unconstrained
    /// mean-shift GLRT statistic, so this variant thresholds it
    /// directly. The windowed-mean-shift GLRT (Willsky 1976) is a
    /// separate detector kind.
    SingleSampleGlrt,
    /// Cumulative-sum detector.
    Cusum,
}

/// Semantic FC actuator-channel to vehicle effector-id mapping.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct FcActuatorChannelsConfig {
    /// Effector id or canonical effector path for aileron commands.
    pub aileron: Option<String>,
    /// Effector id or canonical effector path for elevator commands.
    pub elevator: Option<String>,
    /// Effector id or canonical effector path for rudder commands.
    pub rudder: Option<String>,
    /// Effector id or canonical effector path for body-flap commands.
    pub body_flap: Option<String>,
}

/// Per-phase three-loop gain entry.
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcGainsConfig {
    /// Rate-loop Kp triple `[roll, pitch, yaw]`.
    pub rate_kp: Option<[f64; 3]>,
    /// Rate-loop Ki triple.
    pub rate_ki: Option<[f64; 3]>,
    /// Rate-loop Kd triple.
    pub rate_kd: Option<[f64; 3]>,
    /// Attitude-loop Kp triple.
    pub attitude_kp: Option<[f64; 3]>,
    /// Attitude-loop Ki triple.
    pub attitude_ki: Option<[f64; 3]>,
    /// Attitude-loop Kd triple.
    pub attitude_kd: Option<[f64; 3]>,
    /// Trajectory-loop Kp triple.
    pub trajectory_kp: Option<[f64; 3]>,
    /// Trajectory-loop Ki triple.
    pub trajectory_ki: Option<[f64; 3]>,
    /// Trajectory-loop Kd triple.
    pub trajectory_kd: Option<[f64; 3]>,
    /// Aileron deflection limit (rad).
    pub aileron_limit_rad: Option<f64>,
    /// Elevator deflection limit (rad).
    pub elevator_limit_rad: Option<f64>,
    /// Rudder deflection limit (rad).
    pub rudder_limit_rad: Option<f64>,
    /// Throttle baseline.
    pub throttle_baseline: Option<f64>,
}

/// Per-phase actuator-authority record.
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcPhaseAuthorityConfig {
    /// `true` if the phase permits autopilot actuator commands.
    pub autopilot_allowed: bool,
    /// `true` if the phase permits autopilot engine commands.
    pub engines_allowed: bool,
}

/// v3 block-presence gate: emits `SchemaVersionFieldReserved` on a v2
/// scenario, or runs `per_block` and returns `ElementNotYetSupported`
/// on v3 (the block parses but its runtime consumer is not yet wired).
fn gate_v3_block<T, F>(
    header: u16,
    field: &str,
    missing_capability: &'static str,
    block: Option<&T>,
    per_block: F,
) -> Result<(), ScenarioError>
where
    F: FnOnce() -> Result<(), ScenarioError>,
{
    if block.is_none() {
        return Ok(());
    }
    if header < SCENARIO_VERSION_V3 {
        return Err(ScenarioError::SchemaVersionFieldReserved {
            field: field.to_owned(),
            required: SCENARIO_VERSION_V3,
            found: header,
        });
    }
    per_block()?;
    Err(ScenarioError::ElementNotYetSupported {
        field: field.to_owned(),
        missing_capability,
    })
}

// ---------------------------------------------------------------------
// v3-only scenario blocks.
//
// Each block has a runtime consumer reached through its
// `validate_runtime` method. A block whose consumer is not yet wired
// causes `ScenarioDocument::validate` to reject any v3 scenario that
// declares it with a `ScenarioError::ElementNotYetSupported`
// diagnostic. v2 scenarios that declare any of these blocks
// are rejected earlier with `SchemaVersionFieldReserved`.
//
// When a consumer is wired, its matching deferred-block rejection in
// `ScenarioDocument::validate_v3_blocks` is removed. New fields added
// under the consumer's authority must keep `serde(deny_unknown_fields)`
// and must remain v3-only.
// ---------------------------------------------------------------------

/// First-class multi-rate scheduling block (`[schedule]`, v3 only).
///
/// The parser surface for the kernel-side rate-plan resolver.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ScheduleConfig {
    /// Master tick rate (Hz). Every group's `hz` must divide this exactly.
    pub base_hz: u32,
    /// Rate groups. The serde key is `[[schedule.group]]` per the
    /// architecture sketch; the field is exposed as `groups` in Rust.
    #[serde(default, rename = "group")]
    pub groups: Vec<ScheduleGroupConfig>,
}

impl ScheduleConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        require_positive_u32("schedule.base_hz", self.base_hz)?;
        for (index, group) in self.groups.iter().enumerate() {
            group.validate(index)?;
            if !self.base_hz.is_multiple_of(group.hz) {
                return Err(ScenarioError::InvalidNumber {
                    field: format!("schedule.group[{index}].hz"),
                    value: f64::from(group.hz),
                    rule: "must divide schedule.base_hz exactly",
                });
            }
        }
        let mut seen_labels = BTreeSet::new();
        for (index, group) in self.groups.iter().enumerate() {
            if !seen_labels.insert(group.label.clone()) {
                return Err(ScenarioError::DuplicateValue {
                    field: format!("schedule.group[{index}].label"),
                    value: group.label.clone(),
                });
            }
        }
        Ok(())
    }
}

/// One entry under `[[schedule.group]]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ScheduleGroupConfig {
    /// Group label (e.g. `"env"`, `"fc"`, `"telemetry"`). Must be unique.
    pub label: String,
    /// Group rate in Hz; must divide `ScheduleConfig::base_hz` exactly.
    pub hz: u32,
    /// Subsystem ids belonging to this group. Must not be empty.
    pub members: Vec<String>,
}

impl ScheduleGroupConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        require_non_empty(&format!("schedule.group[{index}].label"), &self.label)?;
        require_positive_u32(&format!("schedule.group[{index}].hz"), self.hz)?;
        require_non_empty_list(&format!("schedule.group[{index}].members"), &self.members)?;
        for (member_index, member) in self.members.iter().enumerate() {
            require_non_empty(
                &format!("schedule.group[{index}].members[{member_index}]"),
                member,
            )?;
        }
        require_unique(&format!("schedule.group[{index}].members"), &self.members)?;
        Ok(())
    }
}

/// First-class multi-body propagation block (`[multi_body]`, v3 only).
///
/// The parser surface for the multi-body kernel propagation path.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MultiBodyConfig {
    /// Assembly body represented by the primary rigid-body lane when
    /// `[[multi_body.initial_lane]]` entries are declared. Existing
    /// separation-only scenarios leave this unset so the primary lane
    /// starts as the whole composite assembly until the first split.
    pub primary_body_id: Option<String>,
    /// Rigid-body lanes active from simulation start. Serde key is
    /// `[[multi_body.initial_lane]]`; the field is exposed as
    /// `initial_lanes` in Rust.
    #[serde(default, rename = "initial_lane")]
    pub initial_lanes: Vec<MultiBodyInitialLaneConfig>,
    /// Separation events that promote a single-body scenario into a
    /// multi-body simulation post-event. Serde key is
    /// `[[multi_body.separation]]`; the field is exposed as
    /// `separations` in Rust.
    #[serde(default, rename = "separation")]
    pub separations: Vec<MultiBodySeparationConfig>,
    /// Optional closed-loop attitude targets for already-separated
    /// rigid-body lanes. Serde key is
    /// `[[multi_body.attitude_target]]`; the field is exposed as
    /// `attitude_targets` in Rust.
    #[serde(default, rename = "attitude_target")]
    pub attitude_targets: Vec<MultiBodyAttitudeTargetConfig>,
}

impl MultiBodyConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if self.initial_lanes.is_empty()
            && self.separations.is_empty()
            && self.attitude_targets.is_empty()
        {
            return Err(ScenarioError::EmptyList {
                field:
                    "multi_body.initial_lane, multi_body.separation, or multi_body.attitude_target"
                        .to_owned(),
            });
        }
        if !self.initial_lanes.is_empty() {
            let primary_body_id = self.primary_body_id.as_ref().ok_or_else(|| {
                ScenarioError::MissingRequiredField {
                    field: "multi_body.primary_body_id".to_owned(),
                    role: ModelRole::Vehicle,
                    name: "multi_body initial lanes".to_owned(),
                }
            })?;
            require_non_empty("multi_body.primary_body_id", primary_body_id)?;
        }
        if let Some(primary_body_id) = &self.primary_body_id {
            require_non_empty("multi_body.primary_body_id", primary_body_id)?;
        }
        let mut seen_initial_lanes = BTreeSet::new();
        for (index, lane) in self.initial_lanes.iter().enumerate() {
            lane.validate(index)?;
            if !seen_initial_lanes.insert(lane.body_id.as_str()) {
                return Err(ScenarioError::DuplicateValue {
                    field: format!("multi_body.initial_lane[{index}].body_id"),
                    value: lane.body_id.clone(),
                });
            }
            if self
                .primary_body_id
                .as_ref()
                .is_some_and(|primary| primary == &lane.body_id)
            {
                return Err(ScenarioError::InconsistentSection {
                    field_a: format!("multi_body.initial_lane[{index}].body_id"),
                    value_a: lane.body_id.clone(),
                    field_b: "multi_body.primary_body_id".to_owned(),
                    value_b: lane.body_id.clone(),
                });
            }
        }
        for (index, sep) in self.separations.iter().enumerate() {
            sep.validate(index)?;
        }
        for (index, target) in self.attitude_targets.iter().enumerate() {
            target.validate(index)?;
        }
        Ok(())
    }
}

/// One entry under `[[multi_body.initial_lane]]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MultiBodyInitialLaneConfig {
    /// Assembly body id propagated as an independent lane from
    /// simulation start.
    pub body_id: String,
    /// Initial inertial position in metres.
    pub position_eci_m: [f64; 3],
    /// Initial inertial velocity in metres per second.
    pub velocity_eci_m_s: [f64; 3],
    /// Initial body-to-ECI quaternion `[x, y, z, w]`.
    pub quaternion_body_to_eci_xyzw: [f64; 4],
    /// Initial body-frame angular velocity in rad/s.
    pub angular_velocity_body_rad_s: [f64; 3],
}

impl MultiBodyInitialLaneConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        require_non_empty(
            &format!("multi_body.initial_lane[{index}].body_id"),
            &self.body_id,
        )?;
        require_finite_array(
            &format!("multi_body.initial_lane[{index}].position_eci_m"),
            &self.position_eci_m,
        )?;
        require_finite_array(
            &format!("multi_body.initial_lane[{index}].velocity_eci_m_s"),
            &self.velocity_eci_m_s,
        )?;
        require_finite_array(
            &format!("multi_body.initial_lane[{index}].quaternion_body_to_eci_xyzw"),
            &self.quaternion_body_to_eci_xyzw,
        )?;
        let norm_sq: f64 = self.quaternion_body_to_eci_xyzw.iter().map(|q| q * q).sum();
        if (norm_sq - 1.0).abs() > 1.0e-9 {
            return Err(ScenarioError::InvalidNumber {
                field: format!("multi_body.initial_lane[{index}].quaternion_body_to_eci_xyzw"),
                value: norm_sq,
                rule: "must be a unit quaternion (||q||² = 1)",
            });
        }
        require_finite_array(
            &format!("multi_body.initial_lane[{index}].angular_velocity_body_rad_s"),
            &self.angular_velocity_body_rad_s,
        )?;
        Ok(())
    }
}

/// One entry under `[[multi_body.separation]]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MultiBodySeparationConfig {
    /// Stable scenario-text id for the mission event triggering this
    /// separation. Must match an event declared in `[[mission.events]]`.
    pub event_id: String,
    /// Body id retained on the controller side (continues with FC).
    pub upper_body_id: String,
    /// Body id detached as ballistic / spent-stage (no FC).
    pub lower_body_id: String,
    /// Optional impulsive delta-V applied to the upper body in body
    /// frame at separation (m/s).
    pub upper_delta_v_body_m_s: Option<[f64; 3]>,
    /// Optional impulsive delta-V applied to the lower body in body
    /// frame at separation (m/s).
    pub lower_delta_v_body_m_s: Option<[f64; 3]>,
    /// Optional body-frame angular-rate TIP-OFF applied to the upper
    /// (continuing) body at separation (rad/s). Models the residual
    /// torque a real separation imparts (uneven push-off, pyro asymmetry,
    /// latch friction). Defaults to zero (clean torque-free separation).
    #[serde(default)]
    pub upper_delta_omega_body_rad_s: Option<[f64; 3]>,
    /// Optional body-frame angular-rate tip-off applied to the lower
    /// (departing) body at separation (rad/s).
    #[serde(default)]
    pub lower_delta_omega_body_rad_s: Option<[f64; 3]>,
    /// Optional body-frame attitude offset `[x, y, z, w]` applied to the
    /// lower (departing) body at separation — a re-orientation such as a
    /// booster flipping retrograde for a boostback burn. Defaults to identity
    /// (the departing body keeps the stack attitude).
    #[serde(default)]
    pub lower_attitude_offset_body_xyzw: Option<[f64; 4]>,
    /// Whether the loader must verify momentum conservation
    /// (`m_u·Δv_u + m_l·Δv_l ≈ 0`). Default `true`.
    #[serde(default = "default_true")]
    pub conserve_momentum: bool,
}

impl MultiBodySeparationConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        require_non_empty(
            &format!("multi_body.separation[{index}].event_id"),
            &self.event_id,
        )?;
        require_non_empty(
            &format!("multi_body.separation[{index}].upper_body_id"),
            &self.upper_body_id,
        )?;
        require_non_empty(
            &format!("multi_body.separation[{index}].lower_body_id"),
            &self.lower_body_id,
        )?;
        if self.upper_body_id == self.lower_body_id {
            return Err(ScenarioError::InconsistentSection {
                field_a: format!("multi_body.separation[{index}].upper_body_id"),
                value_a: self.upper_body_id.clone(),
                field_b: format!("multi_body.separation[{index}].lower_body_id"),
                value_b: self.lower_body_id.clone(),
            });
        }
        if let Some(dv) = self.upper_delta_v_body_m_s {
            require_finite_array(
                &format!("multi_body.separation[{index}].upper_delta_v_body_m_s"),
                &dv,
            )?;
        }
        if let Some(dv) = self.lower_delta_v_body_m_s {
            require_finite_array(
                &format!("multi_body.separation[{index}].lower_delta_v_body_m_s"),
                &dv,
            )?;
        }
        if let Some(dw) = self.upper_delta_omega_body_rad_s {
            require_finite_array(
                &format!("multi_body.separation[{index}].upper_delta_omega_body_rad_s"),
                &dw,
            )?;
        }
        if let Some(dw) = self.lower_delta_omega_body_rad_s {
            require_finite_array(
                &format!("multi_body.separation[{index}].lower_delta_omega_body_rad_s"),
                &dw,
            )?;
        }
        if let Some(q) = self.lower_attitude_offset_body_xyzw {
            let field = format!("multi_body.separation[{index}].lower_attitude_offset_body_xyzw");
            require_finite_array(&field, &q)?;
            let norm_sq: f64 = q.iter().map(|c| c * c).sum();
            if (norm_sq - 1.0).abs() > 1.0e-6 {
                return Err(ScenarioError::InvalidNumber {
                    field,
                    value: norm_sq,
                    rule: "must be a unit quaternion (||q||² = 1)",
                });
            }
        }
        Ok(())
    }
}

fn default_true() -> bool {
    true
}

/// One entry under `[[multi_body.attitude_target]]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MultiBodyAttitudeTargetConfig {
    /// Separated body id this controller acts on.
    pub body_id: String,
    /// Optional start time (s). Before this time the controller emits no
    /// commands.
    #[serde(default)]
    pub start_time_s: Option<f64>,
    /// Optional end time (s). At and after this time the controller
    /// emits no commands.
    #[serde(default)]
    pub end_time_s: Option<f64>,
    /// Body-frame axis to align to the target ECI direction. Defaults
    /// to body +z when omitted.
    #[serde(default)]
    pub body_axis_body: Option<[f64; 3]>,
    /// Optional roll-axis direct-torque effector id.
    #[serde(default)]
    pub roll_effector: Option<String>,
    /// Optional pitch-axis direct-torque effector id.
    #[serde(default)]
    pub pitch_effector: Option<String>,
    /// Optional yaw-axis direct-torque effector id.
    #[serde(default)]
    pub yaw_effector: Option<String>,
    /// Proportional attitude-error gain.
    pub kp: f64,
    /// Body-rate damping gain. Defaults to zero.
    #[serde(default)]
    pub kd: f64,
    /// Optional symmetric command clamp applied before the effector's
    /// own limits.
    #[serde(default)]
    pub max_command: Option<f64>,
    /// Target direction provider.
    pub target: MultiBodyAttitudeTargetKindConfig,
}

impl MultiBodyAttitudeTargetConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        let path = |field: &str| format!("multi_body.attitude_target[{index}].{field}");
        require_non_empty(&path("body_id"), &self.body_id)?;
        if let Some(start_time_s) = self.start_time_s {
            require_finite(&path("start_time_s"), start_time_s)?;
        }
        if let Some(end_time_s) = self.end_time_s {
            require_finite(&path("end_time_s"), end_time_s)?;
        }
        if let (Some(start_time_s), Some(end_time_s)) = (self.start_time_s, self.end_time_s)
            && start_time_s >= end_time_s
        {
            return Err(ScenarioError::InvalidNumber {
                field: path("end_time_s"),
                value: end_time_s,
                rule: "must be strictly greater than start_time_s",
            });
        }
        if let Some(axis) = self.body_axis_body {
            require_finite_array(&path("body_axis_body"), &axis)?;
            require_nonzero_vector(&path("body_axis_body"), &axis)?;
        }
        validate_optional_effector_id(&path("roll_effector"), &self.roll_effector)?;
        validate_optional_effector_id(&path("pitch_effector"), &self.pitch_effector)?;
        validate_optional_effector_id(&path("yaw_effector"), &self.yaw_effector)?;
        if self.roll_effector.is_none()
            && self.pitch_effector.is_none()
            && self.yaw_effector.is_none()
        {
            return Err(ScenarioError::EmptyList {
                field: path("roll_effector, pitch_effector, or yaw_effector"),
            });
        }
        require_finite(&path("kp"), self.kp)?;
        require_positive(&path("kp"), self.kp)?;
        require_finite(&path("kd"), self.kd)?;
        if self.kd < 0.0 {
            return Err(ScenarioError::InvalidNumber {
                field: path("kd"),
                value: self.kd,
                rule: "must be greater than or equal to zero",
            });
        }
        if let Some(max_command) = self.max_command {
            require_finite(&path("max_command"), max_command)?;
            require_positive(&path("max_command"), max_command)?;
        }
        self.target.validate(index)?;
        Ok(())
    }
}

fn validate_optional_effector_id(field: &str, value: &Option<String>) -> Result<(), ScenarioError> {
    if let Some(value) = value {
        require_non_empty(field, value)?;
    }
    Ok(())
}

fn require_nonzero_vector(field: &str, value: &[f64; 3]) -> Result<(), ScenarioError> {
    let norm_sq = value
        .iter()
        .map(|component| component * component)
        .sum::<f64>();
    if norm_sq <= 0.0 {
        return Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value: norm_sq,
            rule: "must have non-zero norm",
        });
    }
    Ok(())
}

/// Direction provider for `[[multi_body.attitude_target]]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum MultiBodyAttitudeTargetKindConfig {
    /// Fixed ECI direction.
    EciVector {
        /// Target ECI vector. It is normalised at runtime.
        vector_eci: [f64; 3],
    },
    /// State-relative direction in the local radial / downrange /
    /// crossrange basis.
    SurfaceRelativeAxes {
        /// Radial component, positive outward from Earth centre.
        radial: f64,
        /// Downrange component along `downrange_axis_eci` projected into
        /// the local horizontal plane.
        downrange: f64,
        /// Crossrange component completing the right-handed local basis.
        crossrange: f64,
        /// Nominal downrange ECI axis.
        downrange_axis_eci: [f64; 3],
    },
}

impl MultiBodyAttitudeTargetKindConfig {
    fn validate(&self, controller_index: usize) -> Result<(), ScenarioError> {
        let path =
            |field: &str| format!("multi_body.attitude_target[{controller_index}].target.{field}");
        match self {
            Self::EciVector { vector_eci } => {
                require_finite_array(&path("vector_eci"), vector_eci)?;
                require_nonzero_vector(&path("vector_eci"), vector_eci)?;
            }
            Self::SurfaceRelativeAxes {
                radial,
                downrange,
                crossrange,
                downrange_axis_eci,
            } => {
                require_finite(&path("radial"), *radial)?;
                require_finite(&path("downrange"), *downrange)?;
                require_finite(&path("crossrange"), *crossrange)?;
                require_finite_array(&path("downrange_axis_eci"), downrange_axis_eci)?;
                require_nonzero_vector(&path("downrange_axis_eci"), downrange_axis_eci)?;
                let norm_sq = radial * radial + downrange * downrange + crossrange * crossrange;
                if norm_sq <= 0.0 {
                    return Err(ScenarioError::InvalidNumber {
                        field: path("radial/downrange/crossrange"),
                        value: norm_sq,
                        rule: "combined reference vector must have non-zero norm",
                    });
                }
            }
        }
        Ok(())
    }
}

/// Multi-instance estimator-routing block (`[fc.estimator_lanes]`, v3 only).
///
/// The parser surface for the lane voter and active-lane
/// selection path.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcEstimatorLanesConfig {
    /// Lane voter selection policy.
    pub voter: FcEstimatorVoterKind,
    /// Registered lane definitions. Serde key `[[fc.estimator_lanes.lane]]`;
    /// the field is exposed as `lanes` in Rust.
    #[serde(default, rename = "lane")]
    pub lanes: Vec<FcEstimatorLaneConfig>,
}

impl FcEstimatorLanesConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if self.lanes.is_empty() {
            return Err(ScenarioError::EmptyList {
                field: "fc.estimator_lanes.lane".to_owned(),
            });
        }
        let mut seen = BTreeSet::new();
        for (index, lane) in self.lanes.iter().enumerate() {
            lane.validate(index)?;
            if !seen.insert(lane.id.clone()) {
                return Err(ScenarioError::DuplicateValue {
                    field: format!("fc.estimator_lanes.lane[{index}].id"),
                    value: lane.id.clone(),
                });
            }
        }
        Ok(())
    }
}

/// One entry under `[[fc.estimator_lanes.lane]]`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcEstimatorLaneConfig {
    /// Stable lane id used in telemetry / FDIR addressing.
    pub id: String,
    /// Estimator kind to instantiate for this lane.
    pub estimator: FcEstimatorKind,
}

impl FcEstimatorLaneConfig {
    fn validate(&self, index: usize) -> Result<(), ScenarioError> {
        require_non_empty(&format!("fc.estimator_lanes.lane[{index}].id"), &self.id)
    }
}

/// Lane voter selection policy.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcEstimatorVoterKind {
    /// First lane wins; no cross-lane vote. Useful for warm-spare.
    SimplexPassThrough,
    /// Median-of-three selection by per-lane innovation chi-square.
    MidValueSelectByInnovation,
    /// Pick the lane with the smallest covariance-trace proxy each tick.
    BestByCovarianceTrace,
}

/// Control-allocation policy block (`[fc.autopilot_allocation]`, v3 only).
///
/// The parser surface for the pseudo-inverse and Härkegård
/// 2002 prioritised redistributed allocators.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcAutopilotAllocationConfig {
    /// Allocation strategy.
    pub kind: FcAutopilotAllocationKind,
    /// Optional per-axis priority order (highest first). Field names
    /// are body-frame axis labels (`"roll"`, `"pitch"`, `"yaw"`).
    /// Applies to `prioritised_redistributed`; default:
    /// `["roll", "yaw", "pitch"]`.
    pub axis_priority: Option<Vec<String>>,
}

impl FcAutopilotAllocationConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if let Some(priority) = &self.axis_priority {
            require_non_empty_list("fc.autopilot_allocation.axis_priority", priority)?;
            if priority.len() != 3 {
                let value = f64::from(u32::try_from(priority.len()).unwrap_or(u32::MAX));
                return Err(ScenarioError::InvalidNumber {
                    field: "fc.autopilot_allocation.axis_priority".to_owned(),
                    value,
                    rule: "must list exactly [roll, pitch, yaw] once each",
                });
            }
            require_unique("fc.autopilot_allocation.axis_priority", priority)?;
            for (index, axis) in priority.iter().enumerate() {
                require_supported(
                    &format!("fc.autopilot_allocation.axis_priority[{index}]"),
                    axis,
                    &["roll", "pitch", "yaw"],
                )?;
            }
        }
        Ok(())
    }
}

/// Control-allocation strategy.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcAutopilotAllocationKind {
    /// Stevens & Lewis 2015 §3.5 pseudo-inverse allocator.
    PseudoInverse,
    /// Härkegård 2002 prioritised-redistributed allocator.
    PrioritisedRedistributed,
}

/// FDIR detector tuning block (`[fc.fdir.detector]`, v3 only).
///
/// `kind` is a typed enum, and `window_samples` /
/// `false_alarm_rate` drive the Willsky 1976 windowed-mean-shift GLRT
/// detector. The existing `detector_kind` field on `FcFdirConfig`
/// continues to drive the burst / single-sample-GLRT / CUSUM
/// detectors; when this block is present its `kind` field is the
/// authoritative selector and overrides the legacy `detector_kind`.
///
/// `parity_threshold` remains parser-only — the Patton-Frank
/// parity-space residual generator that consumes it is future work.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcFdirDetectorConfig {
    /// Detector kind.
    pub kind: FcFdirDetectorKindV5,
    /// Window length (samples) for the windowed-mean-shift GLRT.
    /// Required when `kind = "windowed_mean_shift_glrt"`; rejected
    /// for other kinds.
    pub window_samples: Option<u32>,
    /// Family-wise false-alarm rate over the GLRT window. Optional;
    /// defaults to `0.001` when omitted. Bonferroni-corrected
    /// internally per candidate jump time.
    pub false_alarm_rate: Option<f64>,
    /// Per-residual chi-square threshold for parity-space isolation.
    /// Reserved for future work; rejected for
    /// `windowed_mean_shift_glrt` until then.
    pub parity_threshold: Option<f64>,
}

/// Detector kinds selectable from the `[fc.fdir.detector]`
/// block.
#[derive(Copy, Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcFdirDetectorKindV5 {
    /// Willsky 1976 windowed-mean-shift GLRT,
    /// vector-form, running on whitened innovation streams.
    WindowedMeanShiftGlrt,
}

impl FcFdirDetectorConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        match self.kind {
            FcFdirDetectorKindV5::WindowedMeanShiftGlrt => {
                let window = self
                    .window_samples
                    .ok_or_else(|| ScenarioError::InvalidFc {
                        reason: "fc.fdir.detector.window_samples is required for \
                                     kind = \"windowed_mean_shift_glrt\""
                            .to_string(),
                    })?;
                if window == 0 {
                    return Err(ScenarioError::InvalidFc {
                        reason: "fc.fdir.detector.window_samples must be > 0".to_string(),
                    });
                }
                if let Some(alpha) = self.false_alarm_rate {
                    require_finite("fc.fdir.detector.false_alarm_rate", alpha)?;
                    if alpha <= 0.0 || alpha >= 1.0 {
                        return Err(ScenarioError::InvalidFc {
                            reason: format!(
                                "fc.fdir.detector.false_alarm_rate must be in (0, 1); \
                                 got {alpha}"
                            ),
                        });
                    }
                }
                if self.parity_threshold.is_some() {
                    return Err(ScenarioError::InvalidFc {
                        reason: "fc.fdir.detector.parity_threshold is reserved for the \
                                 parity-space residual generator follow-on; not consumed by \
                                 windowed_mean_shift_glrt"
                            .to_string(),
                    });
                }
            }
        }
        Ok(())
    }
}

/// Minimum-snap differential-flatness trajectory block (`[fc.trajectory]`,
/// v3 only). When `autopilot_params.trajectory_kind`
/// is [`FcTrajectoryKind::MinimumSnap`] this block declares the
/// waypoint sequence and yaw profile that the trajectory generator
/// solves at scenario load.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcTrajectoryConfig {
    /// Trajectory generator kind. `minimum_snap` is the only supported
    /// value.
    pub kind: FcTrajectoryConfigKind,
    /// Constant body-frame yaw applied at every sample (rad). Defaults
    /// to `0.0`. Yaw splines are tracked for follow-up sub-phase.
    pub yaw_rad: Option<f64>,
    /// Ordered list of waypoints. Serde key `[[fc.trajectory.waypoint]]`;
    /// the field is exposed as `waypoints` in Rust.
    #[serde(default, rename = "waypoint")]
    pub waypoints: Vec<FcTrajectoryWaypointConfig>,
}

/// Trajectory generator kind.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FcTrajectoryConfigKind {
    /// Mellinger & Kumar 2011 minimum-snap polynomial trajectory.
    MinimumSnap,
}

/// One entry under `[[fc.trajectory.waypoint]]`.
#[derive(Copy, Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct FcTrajectoryWaypointConfig {
    /// ECI position (m).
    pub position_eci_m: [f64; 3],
    /// Scenario-time of this waypoint (s).
    pub time_s: f64,
}

impl FcTrajectoryConfig {
    fn validate(&self) -> Result<(), ScenarioError> {
        if self.waypoints.len() < 2 {
            return Err(ScenarioError::InvalidFc {
                reason: format!(
                    "fc.trajectory.waypoint must declare >= 2 entries (got {})",
                    self.waypoints.len()
                ),
            });
        }
        if let Some(v) = self.yaw_rad {
            require_finite("fc.trajectory.yaw_rad", v)?;
        }
        for (index, w) in self.waypoints.iter().enumerate() {
            require_finite_array(
                &format!("fc.trajectory.waypoint[{index}].position_eci_m"),
                &w.position_eci_m,
            )?;
            require_finite(&format!("fc.trajectory.waypoint[{index}].time_s"), w.time_s)?;
        }
        for i in 0..self.waypoints.len() - 1 {
            let duration_s = self.waypoints[i + 1].time_s - self.waypoints[i].time_s;
            if duration_s <= 0.0 {
                return Err(ScenarioError::InvalidFc {
                    reason: format!(
                        "fc.trajectory.waypoint[{i}].time_s={} must be strictly less than \
                         waypoint[{}].time_s={}",
                        self.waypoints[i].time_s,
                        i + 1,
                        self.waypoints[i + 1].time_s
                    ),
                });
            }
            if !(FC_TRAJECTORY_MIN_SEGMENT_DURATION_S..=FC_TRAJECTORY_MAX_SEGMENT_DURATION_S)
                .contains(&duration_s)
            {
                return Err(ScenarioError::InvalidFc {
                    reason: format!(
                        "fc.trajectory segment {i} duration {duration_s} s must be within \
                         [{FC_TRAJECTORY_MIN_SEGMENT_DURATION_S}, \
                         {FC_TRAJECTORY_MAX_SEGMENT_DURATION_S}] s"
                    ),
                });
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod fc_string_tests {
    use serde::{Deserialize, Serialize};

    use super::{FcFdirDetectorKind, FcTrajectoryKind};

    #[derive(Debug, Deserialize, Serialize, PartialEq, Eq)]
    struct TrajectoryWrapper {
        trajectory_kind: FcTrajectoryKind,
    }

    #[derive(Debug, Deserialize, Serialize, PartialEq, Eq)]
    struct DetectorWrapper {
        detector_kind: FcFdirDetectorKind,
    }

    #[test]
    fn fc_variant_strings_round_trip_and_old_spellings_reject() {
        let traj: TrajectoryWrapper = toml::from_str("trajectory_kind = \"minimum_snap\"").unwrap();
        assert_eq!(traj.trajectory_kind, FcTrajectoryKind::MinimumSnap);
        assert_eq!(
            toml::to_string(&traj).unwrap(),
            "trajectory_kind = \"minimum_snap\"\n"
        );
        // `flatness_inspired` is retired; the old spelling
        // must reject so v2 scenarios that still carry it surface a
        // serde error rather than silently selecting `Pid`.
        assert!(
            toml::from_str::<TrajectoryWrapper>("trajectory_kind = \"flatness_inspired\"").is_err()
        );
        assert!(
            toml::from_str::<TrajectoryWrapper>("trajectory_kind = \"flatness-inspired\"").is_err()
        );

        let detector: DetectorWrapper =
            toml::from_str("detector_kind = \"single_sample_glrt\"").unwrap();
        assert_eq!(detector.detector_kind, FcFdirDetectorKind::SingleSampleGlrt);
        assert_eq!(
            toml::to_string(&detector).unwrap(),
            "detector_kind = \"single_sample_glrt\"\n"
        );
        assert!(toml::from_str::<DetectorWrapper>("detector_kind = \"glrt\"").is_err());
        assert!(
            toml::from_str::<DetectorWrapper>("detector_kind = \"single-sample-glrt\"").is_err()
        );
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod gyro_notch_tests {
    use super::FcGyroNotchConfig;

    #[test]
    fn gyro_notch_deserializes_and_validates() {
        let notch: FcGyroNotchConfig =
            toml::from_str("center_hz = 0.8\nbandwidth_hz = 0.6\ndepth_db = 18.0").unwrap();
        assert_eq!(notch.center_hz.to_bits(), 0.8_f64.to_bits());
        assert_eq!(notch.bandwidth_hz.to_bits(), 0.6_f64.to_bits());
        assert!(notch.validate(1).is_ok());
    }

    #[test]
    fn gyro_notch_rejects_nonpositive_and_negative_fields() {
        let zero_center = FcGyroNotchConfig {
            center_hz: 0.0,
            bandwidth_hz: 0.6,
            depth_db: 18.0,
        };
        assert!(zero_center.validate(0).is_err());
        let zero_bw = FcGyroNotchConfig {
            center_hz: 0.8,
            bandwidth_hz: 0.0,
            depth_db: 18.0,
        };
        assert!(zero_bw.validate(0).is_err());
        let neg_depth = FcGyroNotchConfig {
            center_hz: 0.8,
            bandwidth_hz: 0.6,
            depth_db: -1.0,
        };
        assert!(neg_depth.validate(0).is_err());
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod ascent_reference_config_tests {
    use super::{FcAscentReferenceConfig, FcAscentReferenceMethod};

    fn ascent_sequence_config(kick_start: f64, kick_end: f64) -> FcAscentReferenceConfig {
        FcAscentReferenceConfig {
            method: FcAscentReferenceMethod::AscentSequence,
            schedule_s: None,
            pitch_rad: None,
            insertion_radius_m: Some(6_710_000.0),
            k_alt_rad_per_m: None,
            k_vr_rad_per_m_s: None,
            theta_min_rad: None,
            theta_max_rad: None,
            downrange_axis_eci: Some([0.0, 1.0, 0.0]),
            exhaust_velocity_m_s: Some(3_413.0),
            initial_thrust_accel_m_s2: Some(5.0),
            peg_initial_t_go_s: Some(220.0),
            peg_min_speed_m_s: None,
            kick_start_speed_m_s: Some(kick_start),
            kick_end_speed_m_s: Some(kick_end),
            kick_angle_rad: Some(0.0),
            peg_handoff_speed_m_s: Some(3_500.0),
            orbital_plane_normal_eci: None,
            k_cross_rad_per_m_s: None,
            psi_max_rad: None,
        }
    }

    #[test]
    fn ascent_sequence_accepts_zero_kick_thresholds() {
        ascent_sequence_config(0.0, 0.0).validate().unwrap();
    }

    #[test]
    fn ascent_sequence_rejects_negative_kick_thresholds() {
        assert!(ascent_sequence_config(-1.0, 0.0).validate().is_err());
        assert!(ascent_sequence_config(0.0, -1.0).validate().is_err());
    }
}

#[cfg(test)]
mod recovery_command_tests {
    use super::is_recovery_command_compatible;

    #[test]
    fn recovery_command_compatibility_matrix_is_locked() {
        let kinds = ["parachute_drag", "drogue_main", "drag_device"];
        let commands = ["deploy", "deploy_drogue", "deploy_main", "stow"];

        for kind in kinds {
            for command in commands {
                let expected = matches!(
                    (kind, command),
                    ("parachute_drag", "deploy")
                        | ("drogue_main", "deploy_drogue" | "deploy_main")
                        | ("drag_device", "deploy" | "stow")
                );
                assert_eq!(
                    is_recovery_command_compatible(kind, command),
                    expected,
                    "unexpected recovery command compatibility for kind={kind}, command={command}",
                );
            }
        }
    }
}
