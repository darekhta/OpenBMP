//! Scenario → kernel → telemetry adapter for Phase-2 point-mass
//! scenarios that use the structured Phase-2.10 blocks (`[aero]`,
//! `[propulsion.motor]`, `[atmosphere]`, …).
//!
//! Accepted scenario shape (any combination of the following relative
//! to the [`crate::runner::phase1`] shape):
//!
//! - `vehicle.kind = "point_mass"`
//! - `environment.gravity = "constant"` with non-negative
//!   `gravity_m_s2`
//! - `[atmosphere].kind = "us_standard_1976"` (when `[aero]` declared
//!   AND `forces` includes `aero`)
//! - `[aero].deck = "<path>"` with optional pinned digest
//! - `[propulsion.motor].file = "<path>"` with optional pinned digest
//! - `forces.models` entries permuted from `["gravity", "aero",
//!   "thrust"]`
//!
//! Force evaluation order respects the scenario-declared
//! `forces.models` order; this is the determinism contract. Bad
//! SHA-256 pins fail closed before kernel construction (via
//! [`Scenario::resolved_files`] called in
//! [`crate::runner::run`]).
//!
//! Telemetry layout (Phase-2.11.B): the seven Phase-1 base channels
//! (position×3, velocity×3, mass) plus, conditionally:
//!
//! - **Atmosphere sample** when `[atmosphere].kind = "us_standard_1976"`:
//!   `atmosphere.density_kg_m3`, `atmosphere.pressure_pa`,
//!   `atmosphere.temperature_k`, `atmosphere.speed_of_sound_m_s`
//!   (atmospheric scalars; no frame metadata).
//! - **Per-model force breakdown** for every entry in `forces.models`:
//!   `force.<name>.x_n`, `.y_n`, `.z_n` with frame metadata `"ECI"`.
//!   `<name>` matches the scenario-declared model name (`gravity`,
//!   `thrust`, `aero`).
//!
//! The schema also carries the resolved-file SHA-256 digests as
//! Arrow schema metadata under `openbmp.scenario_files.<field>` keys.

use std::collections::BTreeMap;

use nalgebra::Vector3;
use openbmp_aero::{AeroDeck, AeroError};
use openbmp_core::{ChannelId, Duration, ModelId, Position3, SimTime, Velocity3};
use openbmp_env::{AtmosphereModel, UsStandard1976};
use openbmp_propulsion::{Motor, MotorError, SolidMotor};
use openbmp_scenario::{ResolvedFile, Scenario, ScenarioDocument};
use openbmp_sim::{
    ConstantGravityForce, ConstantMass, EndTime, ForceContext, ForceModel, MassModel,
    NullEnvironment, Rk4FixedStep, SimulationConfig, SimulationKernel, StopReason,
};
use openbmp_state::PointMassState;
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use openbmp_vehicle::{
    AxialDragForceAdapter, BasicVehicle, BoxedMassModel, MotorMassAdapter, MotorThrustForceAdapter,
    NamedForceModel, Vehicle,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::CliError;
use crate::runner::RunOutcome;
use crate::runner::assembly::dry_mass_kg_at;
use openbmp_vehicle::BasicAssembly;

// Stable model-ids assigned to each force / mass model the runner
// wires. Scenario-supplied force-model names ("aero", "thrust") are
// mapped to these; the constant-gravity force is registered via
// `ConstantGravityForce::new` which carries its own internal id.
// Phase-2.7 uses model-ids in the determinism oracle; the runner
// picks fixed values so the per-model telemetry stream is keyed
// deterministically.
const PHASE2_AERO_MODEL_ID: ModelId = ModelId::new(102);
const PHASE2_THRUST_MODEL_ID: ModelId = ModelId::new(103);
const PHASE2_MOTOR_MASS_MODEL_ID: ModelId = ModelId::new(104);

#[derive(Clone, Debug, Default)]
struct LoadedModels {
    aero_deck: Option<AeroDeck>,
    motor: Option<SolidMotor>,
}

/// Run a Phase-2 point-mass scenario through a freshly-built kernel
/// and return the populated telemetry table.
///
/// `resolved_files` is the digest map produced by
/// [`Scenario::resolved_files`]; the runner records each entry as
/// `openbmp.scenario_files.<field>` schema metadata so the Parquet
/// header carries the SHA-256 pins for replay verification.
///
/// # Errors
///
/// Returns [`CliError::UnsupportedScenario`] when the scenario shape
/// does not match the Phase-2 point-mass contract,
/// [`CliError::Aero`] / [`CliError::Motor`] / [`CliError::Env`] for
/// loader failures, and [`CliError::Simulation`] / [`CliError::Telemetry`]
/// for kernel- or telemetry-side failures.
pub fn run(
    scenario: &Scenario,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<RunOutcome, CliError> {
    let document = &scenario.document;
    require_supported_shape(document)?;
    let assembly = crate::runner::assembly::synthesize_assembly(document)?;

    let loaded_models = load_models(document, resolved_files)?;
    let initial_state = build_initial_state(document, &loaded_models, &assembly)?;
    let kernel_vehicle = build_vehicle(document, &loaded_models, &assembly)?;
    // The runner-side breakdown vehicle is a *separate* construction
    // of the same models. `BasicVehicle::evaluate_force_breakdown`
    // takes `&self`, but `BasicVehicle` is not `Clone` (the inner
    // `Box<dyn ForceModel>` lists are not). Re-building from scratch
    // avoids interior-mutability or Arc gymnastics; both copies are
    // stateless and evaluate identically per the Phase-2.6/2.5
    // contracts.
    let breakdown_vehicle = build_vehicle(document, &loaded_models, &assembly)?;
    let mass_model = BoxedMassModel(build_mass_model(document, &loaded_models, &assembly)?);

    let config = SimulationConfig {
        initial_state,
        integrator: Rk4FixedStep,
        force_model: kernel_vehicle,
        mass_model,
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(document.time.stop_s)),
        dt: Duration::from_seconds(document.time.dt_s),
        scenario_seed: document.time.seed,
    };

    let kernel_base = SimulationKernel::new(config)?;
    let mut kernel = if let Some(mission) = &document.mission {
        let (events, graph) = crate::runner::mission::build_mission_runtime(mission)?;
        kernel_base.with_mission(events, Some(graph))?
    } else {
        kernel_base
    };
    let channel_set = Phase2ChannelSet::new(document)?;
    let breakdown_atmosphere = if channel_set.has_atmosphere {
        Some(UsStandard1976::new())
    } else {
        None
    };
    let metadata = build_schema_metadata(resolved_files);
    let mut table = TelemetryTable::new(channel_set.schema(metadata)?);

    // Step 0 has no fired events.
    record_step(
        &mut table,
        &kernel,
        &channel_set,
        &breakdown_vehicle,
        breakdown_atmosphere.as_ref(),
        &[],
    )?;
    while kernel.stop_reason().is_none() {
        kernel.step()?;
        let fired = kernel.drain_events();
        record_step(
            &mut table,
            &kernel,
            &channel_set,
            &breakdown_vehicle,
            breakdown_atmosphere.as_ref(),
            &fired,
        )?;
    }

    let stop_reason = kernel
        .stop_reason()
        .cloned()
        .unwrap_or(StopReason::EndTime { reached_s: 0.0 });

    Ok(RunOutcome {
        final_step: kernel.current_step().value(),
        final_time_s: kernel.current_time().as_seconds(),
        stop_reason,
        table,
    })
}

fn require_supported_shape(document: &ScenarioDocument) -> Result<(), CliError> {
    if document.vehicle.kind != "point_mass" {
        return Err(CliError::UnsupportedScenario {
            what: format!("vehicle.kind = {}", document.vehicle.kind),
        });
    }
    if document.environment.gravity != "constant" {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "environment.gravity = {} (only `constant` wired in 2.11.A)",
                document.environment.gravity
            ),
        });
    }

    // Force list: subset of {gravity, aero, thrust}, scenario-declared
    // order is the determinism contract.
    for name in &document.forces.models {
        if !matches!(name.as_str(), "gravity" | "aero" | "thrust") {
            return Err(CliError::UnsupportedScenario {
                what: format!("forces.models entry `{name}` (only gravity, aero, thrust wired)"),
            });
        }
    }

    // Wind: only `none` is wired in 2.11.A.
    if document.environment.wind != "none" {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "environment.wind = {} (Phase-2.11.A wires no wind models)",
                document.environment.wind
            ),
        });
    }
    if let Some(wind) = &document.wind
        && wind.kind != "none"
    {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "[wind].kind = {} (Phase-2.11.A wires no wind models)",
                wind.kind
            ),
        });
    }

    // Atmosphere: when `aero` is in the force list, require USSA76.
    let has_aero = document.forces.models.iter().any(|m| m == "aero");
    if has_aero {
        let kind = document.atmosphere.as_ref().map_or_else(
            || document.environment.atmosphere.as_str(),
            |a| a.kind.as_str(),
        );
        if kind != "us_standard_1976" {
            return Err(CliError::UnsupportedScenario {
                what: format!(
                    "atmosphere `{kind}` is not wired with the aero force in 2.11.A; \
                     use `us_standard_1976`"
                ),
            });
        }
    }

    Ok(())
}

fn load_models(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<LoadedModels, CliError> {
    let aero_deck = if document.aero.is_some() {
        let resolved = required_resolved_file(resolved_files, "aero.deck")?;
        let text = std::str::from_utf8(&resolved.bytes).map_err(|e| {
            CliError::Aero(AeroError::Io {
                reason: format!(
                    "could not read deck file {} as UTF-8: {e}",
                    resolved.path.display()
                ),
            })
        })?;
        Some(AeroDeck::load_from_str(text)?)
    } else {
        None
    };

    let motor = if document
        .propulsion
        .as_ref()
        .and_then(|p| p.motor.as_ref())
        .is_some()
    {
        let resolved = required_resolved_file(resolved_files, "propulsion.motor.file")?;
        let text = std::str::from_utf8(&resolved.bytes).map_err(|e| {
            CliError::Motor(MotorError::Io {
                reason: format!(
                    "could not read motor file {} as UTF-8: {e}",
                    resolved.path.display()
                ),
            })
        })?;
        Some(SolidMotor::load_from_str(text)?)
    } else {
        None
    };

    Ok(LoadedModels { aero_deck, motor })
}

fn required_resolved_file<'a>(
    resolved_files: &'a BTreeMap<String, ResolvedFile>,
    field: &str,
) -> Result<&'a ResolvedFile, CliError> {
    resolved_files
        .get(field)
        .ok_or_else(|| CliError::UnsupportedScenario {
            what: format!(
                "internal invariant: resolved file `{field}` missing after pin verification"
            ),
        })
}

fn build_initial_state(
    document: &ScenarioDocument,
    loaded_models: &LoadedModels,
    assembly: &BasicAssembly,
) -> Result<PointMassState, CliError> {
    let p = document.vehicle.initial_position_eci_m;
    let v = document.vehicle.initial_velocity_eci_m_s;
    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_mass_kg = dry_mass_kg_at(assembly, start_time, "vehicle.assembly")?;

    // Total mass at the initial state = assembly dry mass PLUS the
    // motor's current mass when a motor is declared. The scenario-side
    // `mass_kg` is the dry-airframe mass (no motor). MotorMassAdapter
    // mirrors this by adding the motor mass at the initial state time.
    // `ignite_at_s` is relative to scenario start, while kernel model
    // adapters take absolute simulation time, so pre-roll / delayed
    // ignition scenarios stay consistent.
    let total_mass_kg = if let Some(motor) = &loaded_models.motor {
        let t_since_ignition_s = motor_elapsed_at_start_s(document)?;
        dry_mass_kg + motor.mass_kg(t_since_ignition_s)?
    } else {
        dry_mass_kg
    };

    Ok(PointMassState::new(
        start_time,
        Position3::new(p[0], p[1], p[2]),
        Velocity3::new(v[0], v[1], v[2]),
        Mass::new::<kilogram>(total_mass_kg),
    ))
}

fn build_vehicle(
    document: &ScenarioDocument,
    loaded_models: &LoadedModels,
    assembly: &BasicAssembly,
) -> Result<BasicVehicle<PointMassState>, CliError> {
    let mut named: Vec<NamedForceModel<PointMassState>> = Vec::new();

    for name in &document.forces.models {
        match name.as_str() {
            "gravity" => {
                let g = document.environment.gravity_m_s2.ok_or_else(|| {
                    CliError::UnsupportedScenario {
                        what: "environment.gravity_m_s2 missing for constant gravity".to_owned(),
                    }
                })?;
                if g < 0.0 {
                    return Err(CliError::UnsupportedScenario {
                        what: "environment.gravity_m_s2 must be a non-negative magnitude; \
                             Phase-2 constant gravity is -z in ECI"
                            .to_owned(),
                    });
                }
                let force = ConstantGravityForce::new(Vector3::new(0.0, 0.0, -g));
                named.push(NamedForceModel::new("gravity", Box::new(force)));
            }
            "aero" => {
                let deck = loaded_models.aero_deck.clone().ok_or_else(|| {
                    CliError::UnsupportedScenario {
                        what: "forces includes `aero` but [aero] block is missing".to_owned(),
                    }
                })?;
                let atmosphere = UsStandard1976::new();
                let drag = AxialDragForceAdapter::new(deck, atmosphere, PHASE2_AERO_MODEL_ID);
                named.push(NamedForceModel::new("aero", Box::new(drag)));
            }
            "thrust" => {
                let motor =
                    loaded_models
                        .motor
                        .clone()
                        .ok_or_else(|| CliError::UnsupportedScenario {
                            what: "forces includes `thrust` but [propulsion.motor] is missing"
                                .to_owned(),
                        })?;
                let ignition_time_s = motor_ignition_time_s(document)?;
                let thrust =
                    MotorThrustForceAdapter::new(motor, ignition_time_s, PHASE2_THRUST_MODEL_ID);
                named.push(NamedForceModel::new("thrust", Box::new(thrust)));
            }
            other => unreachable!("require_supported_shape rejects unknown force model `{other}`"),
        }
    }

    // BasicVehicle requires a mass model even for vehicle-internal
    // queries (Phase-2.8 contract). The kernel's mass model is built
    // separately in `build_mass_model` because it owns its own copy.
    let vehicle_mass = build_mass_model(document, loaded_models, assembly)?;
    BasicVehicle::new(named, vec![], vehicle_mass).map_err(|e| CliError::UnsupportedScenario {
        what: format!("BasicVehicle construction failed: {e}"),
    })
}

fn build_mass_model(
    document: &ScenarioDocument,
    loaded_models: &LoadedModels,
    assembly: &BasicAssembly,
) -> Result<Box<dyn MassModel>, CliError> {
    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_mass_kg = dry_mass_kg_at(assembly, start_time, "vehicle.assembly")?;
    if let Some(motor) = &loaded_models.motor {
        let ignition_time_s = motor_ignition_time_s(document)?;
        Ok(Box::new(MotorMassAdapter::new(
            motor.clone(),
            dry_mass_kg,
            ignition_time_s,
            PHASE2_MOTOR_MASS_MODEL_ID,
        )))
    } else {
        Ok(Box::new(ConstantMass::new(dry_mass_kg)))
    }
}

fn motor_ignition_time_s(document: &ScenarioDocument) -> Result<f64, CliError> {
    let motor = document
        .propulsion
        .as_ref()
        .and_then(|p| p.motor.as_ref())
        .ok_or_else(|| CliError::UnsupportedScenario {
            what: "[propulsion.motor] block missing".to_owned(),
        })?;
    Ok(document.time.start_s + motor.ignite_at_s)
}

fn motor_elapsed_at_start_s(document: &ScenarioDocument) -> Result<f64, CliError> {
    Ok(document.time.start_s - motor_ignition_time_s(document)?)
}

fn build_schema_metadata(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> BTreeMap<String, String> {
    let mut metadata = BTreeMap::new();
    for (field, file) in resolved_files {
        metadata.insert(
            format!("openbmp.scenario_files.{field}"),
            file.sha256_hex.clone(),
        );
    }
    metadata
}

// ---------------------------------------------------------------------
// Phase-2 channel set: base + atmosphere + per-model force breakdown
// ---------------------------------------------------------------------

/// Per-model force-component channels in declared order.
/// Each entry is `(declared_name, x_channel, y_channel, z_channel)`.
type ForceComponentChannels = Vec<(
    String,
    TelemetryChannel<f64>,
    TelemetryChannel<f64>,
    TelemetryChannel<f64>,
)>;

#[derive(Debug)]
struct Phase2ChannelSet {
    position_x: TelemetryChannel<f64>,
    position_y: TelemetryChannel<f64>,
    position_z: TelemetryChannel<f64>,
    velocity_x: TelemetryChannel<f64>,
    velocity_y: TelemetryChannel<f64>,
    velocity_z: TelemetryChannel<f64>,
    mass: TelemetryChannel<f64>,
    has_atmosphere: bool,
    atmosphere_density: Option<TelemetryChannel<f64>>,
    atmosphere_pressure: Option<TelemetryChannel<f64>>,
    atmosphere_temperature: Option<TelemetryChannel<f64>>,
    atmosphere_speed_of_sound: Option<TelemetryChannel<f64>>,
    /// Force-model components in declared order.
    force_components: ForceComponentChannels,
    /// Phase-3.2 mission-event telemetry markers, keyed by tag.
    /// `BTreeMap` order is alphabetical for deterministic channel
    /// allocation regardless of scenario-text declaration order.
    mission_markers: BTreeMap<String, TelemetryChannel<bool>>,
}

impl Phase2ChannelSet {
    #[allow(clippy::too_many_lines)] // Phase-2/3 channel inventory grows with each schema extension
    fn new(document: &ScenarioDocument) -> Result<Self, CliError> {
        let mut next_id: u64 = 1;
        let mut alloc = || {
            let id = ChannelId::new(next_id);
            next_id += 1;
            id
        };

        let position_x = TelemetryChannel::<f64>::new(alloc(), "position_x_m", "m", Some("ECI"))?;
        let position_y = TelemetryChannel::<f64>::new(alloc(), "position_y_m", "m", Some("ECI"))?;
        let position_z = TelemetryChannel::<f64>::new(alloc(), "position_z_m", "m", Some("ECI"))?;
        let velocity_x =
            TelemetryChannel::<f64>::new(alloc(), "velocity_x_m_s", "m/s", Some("ECI"))?;
        let velocity_y =
            TelemetryChannel::<f64>::new(alloc(), "velocity_y_m_s", "m/s", Some("ECI"))?;
        let velocity_z =
            TelemetryChannel::<f64>::new(alloc(), "velocity_z_m_s", "m/s", Some("ECI"))?;
        let mass = TelemetryChannel::<f64>::new(alloc(), "mass_kg", "kg", None::<&str>)?;

        // Atmosphere channels: only when the scenario declares
        // [atmosphere].kind = "us_standard_1976".
        let atmosphere_kind = document
            .atmosphere
            .as_ref()
            .map_or(document.environment.atmosphere.as_str(), |a| {
                a.kind.as_str()
            });
        let has_atmosphere = atmosphere_kind == "us_standard_1976";
        let (
            atmosphere_density,
            atmosphere_pressure,
            atmosphere_temperature,
            atmosphere_speed_of_sound,
        ) = if has_atmosphere {
            let density = TelemetryChannel::<f64>::new(
                alloc(),
                "atmosphere.density_kg_m3",
                "kg/m^3",
                None::<&str>,
            )?;
            let pressure = TelemetryChannel::<f64>::new(
                alloc(),
                "atmosphere.pressure_pa",
                "Pa",
                None::<&str>,
            )?;
            let temperature = TelemetryChannel::<f64>::new(
                alloc(),
                "atmosphere.temperature_k",
                "K",
                None::<&str>,
            )?;
            let speed_of_sound = TelemetryChannel::<f64>::new(
                alloc(),
                "atmosphere.speed_of_sound_m_s",
                "m/s",
                None::<&str>,
            )?;
            (
                Some(density),
                Some(pressure),
                Some(temperature),
                Some(speed_of_sound),
            )
        } else {
            (None, None, None, None)
        };

        // Per-model force breakdown channels, in scenario-declared
        // order — the same order the kernel uses for the RK4 sum.
        let mut force_components = Vec::with_capacity(document.forces.models.len());
        for name in &document.forces.models {
            let x_channel = TelemetryChannel::<f64>::new(
                alloc(),
                format!("force.{name}.x_n"),
                "N",
                Some("ECI"),
            )?;
            let y_channel = TelemetryChannel::<f64>::new(
                alloc(),
                format!("force.{name}.y_n"),
                "N",
                Some("ECI"),
            )?;
            let z_channel = TelemetryChannel::<f64>::new(
                alloc(),
                format!("force.{name}.z_n"),
                "N",
                Some("ECI"),
            )?;
            force_components.push((name.clone(), x_channel, y_channel, z_channel));
        }

        // Phase-3.2 mission marker channels. `BTreeMap` ordering on
        // tag keys keeps channel id allocation deterministic even
        // when the scenario reorders `[[mission.events]]` blocks.
        let mut mission_markers: BTreeMap<String, TelemetryChannel<bool>> = BTreeMap::new();
        if let Some(mission) = &document.mission {
            for tag in crate::runner::mission::marker_tags(mission) {
                let channel = TelemetryChannel::<bool>::new(
                    alloc(),
                    format!("mission.marker.{tag}"),
                    "bool",
                    None::<&str>,
                )?;
                mission_markers.insert(tag, channel);
            }
        }

        Ok(Self {
            position_x,
            position_y,
            position_z,
            velocity_x,
            velocity_y,
            velocity_z,
            mass,
            has_atmosphere,
            atmosphere_density,
            atmosphere_pressure,
            atmosphere_temperature,
            atmosphere_speed_of_sound,
            force_components,
            mission_markers,
        })
    }

    fn schema(&self, metadata: BTreeMap<String, String>) -> Result<TelemetrySchema, CliError> {
        let mut channels = vec![
            self.position_x.metadata().clone(),
            self.position_y.metadata().clone(),
            self.position_z.metadata().clone(),
            self.velocity_x.metadata().clone(),
            self.velocity_y.metadata().clone(),
            self.velocity_z.metadata().clone(),
            self.mass.metadata().clone(),
        ];
        if let (Some(d), Some(p), Some(t), Some(s)) = (
            &self.atmosphere_density,
            &self.atmosphere_pressure,
            &self.atmosphere_temperature,
            &self.atmosphere_speed_of_sound,
        ) {
            channels.push(d.metadata().clone());
            channels.push(p.metadata().clone());
            channels.push(t.metadata().clone());
            channels.push(s.metadata().clone());
        }
        for (_, x, y, z) in &self.force_components {
            channels.push(x.metadata().clone());
            channels.push(y.metadata().clone());
            channels.push(z.metadata().clone());
        }
        // Marker channels last, in alphabetical (BTreeMap) order.
        for marker in self.mission_markers.values() {
            channels.push(marker.metadata().clone());
        }
        Ok(TelemetrySchema::new(channels)?.with_metadata(metadata))
    }
}

#[allow(clippy::too_many_arguments)]
fn record_step<I, F, MM, E, SC>(
    table: &mut TelemetryTable,
    kernel: &SimulationKernel<PointMassState, I, F, MM, E, SC>,
    channels: &Phase2ChannelSet,
    breakdown_vehicle: &BasicVehicle<PointMassState>,
    breakdown_atmosphere: Option<&UsStandard1976>,
    fired_events: &[openbmp_sim::FiredEvent],
) -> Result<(), CliError>
where
    I: openbmp_sim::Integrator<PointMassState>,
    F: ForceModel<PointMassState>,
    MM: openbmp_sim::MassModel,
    E: openbmp_sim::EnvironmentModel,
    SC: openbmp_sim::StopCondition<PointMassState>,
{
    let state = kernel.current_state();
    let mut row = TelemetryRow::new(state.time, kernel.current_step())?;

    row.insert(&channels.position_x, state.position.vector.x)?;
    row.insert(&channels.position_y, state.position.vector.y)?;
    row.insert(&channels.position_z, state.position.vector.z)?;
    row.insert(&channels.velocity_x, state.velocity.vector.x)?;
    row.insert(&channels.velocity_y, state.velocity.vector.y)?;
    row.insert(&channels.velocity_z, state.velocity.vector.z)?;
    row.insert(&channels.mass, state.mass.get::<kilogram>())?;

    // Atmosphere sample at the post-step state. The runner uses ECI
    // +z as the altitude proxy, matching the AxialDragForceAdapter
    // convention. Sub-zero altitudes are clamped to 0 m so the
    // atmosphere model does not reject post-apogee descent past
    // ground.
    if let Some(atmosphere) = breakdown_atmosphere {
        let altitude_m = state.position.vector.z.max(0.0);
        let sample = atmosphere.sample(altitude_m, state.time)?;
        if let (Some(d), Some(p), Some(t), Some(s)) = (
            &channels.atmosphere_density,
            &channels.atmosphere_pressure,
            &channels.atmosphere_temperature,
            &channels.atmosphere_speed_of_sound,
        ) {
            row.insert(d, sample.density_kg_m3)?;
            row.insert(p, sample.pressure_pa)?;
            row.insert(t, sample.temperature_k)?;
            row.insert(s, sample.speed_of_sound_m_s)?;
        }
    }

    // Per-model force breakdown evaluated at the post-step state.
    // The breakdown vehicle is a separate construction of the same
    // models the kernel uses; both are stateless and evaluate
    // identically. The breakdown is therefore the per-model
    // contribution to the kernel's total at the step boundary.
    let env_sample = kernel.current_environment_sample()?;
    let ctx = ForceContext {
        state,
        environment: &env_sample,
        mass_kg: state.mass.get::<kilogram>(),
        time: state.time,
    };
    let breakdown = breakdown_vehicle
        .evaluate_force_breakdown(ctx)
        .map_err(|e| CliError::UnsupportedScenario {
            what: format!("force-breakdown evaluation failed: {e}"),
        })?;
    for (declared_name, x_channel, y_channel, z_channel) in &channels.force_components {
        let component = breakdown
            .components
            .iter()
            .find(|(name, _)| name == declared_name)
            .map(|(_, vector)| *vector)
            .ok_or_else(|| CliError::UnsupportedScenario {
                what: format!(
                    "force-breakdown component `{declared_name}` missing from vehicle evaluation"
                ),
            })?;
        row.insert(x_channel, component.x)?;
        row.insert(y_channel, component.y)?;
        row.insert(z_channel, component.z)?;
    }

    // Phase-3.2 marker channels: write `true` for any tag whose
    // event fired this step, `false` for the rest. The marker
    // channel order is alphabetical (BTreeMap iteration); the
    // `fired_events` slice is in scenario-declared event order, so
    // a tag may match more than one fired event in a single step
    // (the row is `true` if any matched).
    let mut fired_tags: std::collections::BTreeSet<&str> = std::collections::BTreeSet::new();
    for fired in fired_events {
        if let openbmp_sim::EventAction::EmitTelemetryMarker { tag } = &fired.action {
            fired_tags.insert(tag.as_str());
        }
    }
    for (tag, channel) in &channels.mission_markers {
        let value = fired_tags.contains(tag.as_str());
        row.insert(channel, value)?;
    }

    table.push_row(row)?;
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use std::path::{Path, PathBuf};

    use openbmp_telemetry::TelemetryValue;

    use super::*;

    fn workspace_root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(Path::parent)
            .map(Path::to_path_buf)
            .expect("workspace root must exist")
    }

    fn niskanen_scenario() -> Scenario {
        Scenario::from_file(
            workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6.toml"),
        )
        .expect("canonical Niskanen scenario must parse")
    }

    fn first_mass_kg(outcome: &RunOutcome) -> f64 {
        let mass_channel = outcome
            .table
            .schema()
            .channels()
            .iter()
            .find(|channel| channel.name == "mass_kg")
            .expect("mass channel must exist");
        let row = outcome.table.rows().first().expect("initial row exists");
        match row.get(mass_channel.id) {
            Some(TelemetryValue::Float64(value)) => *value,
            other => panic!("unexpected mass value: {other:?}"),
        }
    }

    #[test]
    fn phase2_point_mass_supports_motorless_aero_scenarios() {
        let mut scenario = niskanen_scenario();
        scenario.document.time.stop_s = 0.010;
        scenario.document.propulsion = None;
        scenario.document.forces.models = vec!["gravity".to_owned(), "aero".to_owned()];

        let resolved_files = scenario.resolved_files().expect("resolve aero deck");
        assert!(resolved_files.contains_key("aero.deck"));
        assert!(!resolved_files.contains_key("propulsion.motor.file"));

        let outcome = run(&scenario, &resolved_files).expect("motorless aero run succeeds");
        assert_eq!(outcome.final_step, 10);
        assert!(
            outcome
                .table
                .schema()
                .channels()
                .iter()
                .any(|channel| channel.name == "force.aero.z_n"),
            "aero force channel should be present"
        );
    }

    #[test]
    fn motor_ignition_time_is_relative_to_scenario_start() {
        let mut scenario = niskanen_scenario();
        scenario.document.time.start_s = 10.0;
        scenario.document.time.stop_s = 10.001;
        scenario.document.time.dt_s = 0.001;
        scenario
            .document
            .propulsion
            .as_mut()
            .and_then(|propulsion| propulsion.motor.as_mut())
            .expect("canonical scenario has motor")
            .ignite_at_s = 0.0;

        let resolved_files = scenario.resolved_files().expect("resolve files");
        let loaded_models = load_models(&scenario.document, &resolved_files).expect("load models");
        let assembly =
            crate::runner::assembly::synthesize_assembly(&scenario.document).expect("assembly");
        let motor = loaded_models.motor.as_ref().expect("motor loaded");
        let expected_initial_mass_kg = scenario.document.vehicle.mass_kg
            + motor
                .mass_kg(0.0)
                .expect("motor mass at scenario-relative ignition");

        let initial_state = build_initial_state(&scenario.document, &loaded_models, &assembly)
            .expect("initial state");
        assert_eq!(
            initial_state.time.as_seconds().to_bits(),
            10.0_f64.to_bits()
        );
        assert_eq!(
            initial_state.mass.get::<kilogram>().to_bits(),
            expected_initial_mass_kg.to_bits()
        );

        let mass_model =
            build_mass_model(&scenario.document, &loaded_models, &assembly).expect("mass model");
        assert_eq!(
            mass_model
                .mass_kg(SimTime::from_seconds(10.0))
                .expect("mass at scenario start")
                .to_bits(),
            expected_initial_mass_kg.to_bits()
        );

        let outcome = run(&scenario, &resolved_files).expect("run succeeds");
        assert_eq!(
            first_mass_kg(&outcome).to_bits(),
            expected_initial_mass_kg.to_bits()
        );
    }

    #[test]
    fn mass_construction_uses_resolved_assembly_mass() {
        let mut scenario = niskanen_scenario();
        scenario.document.aero = None;
        scenario.document.propulsion = None;
        scenario.document.forces.models = vec!["gravity".to_owned()];

        let resolved_files = scenario.resolved_files().expect("resolve files");
        let loaded_models = load_models(&scenario.document, &resolved_files).expect("load models");
        let assembly = BasicAssembly::single_body_legacy(
            openbmp_core::VehicleId::from_path("test.vehicle"),
            openbmp_core::BodyId::from_path("test.vehicle.body"),
            12.5,
            None,
            openbmp_vehicle::BodyGeometry::Reference {
                length_m: 1.0,
                area_m2: 1.0,
            },
        )
        .expect("assembly");

        let initial_state = build_initial_state(&scenario.document, &loaded_models, &assembly)
            .expect("initial state");
        assert_eq!(
            initial_state.mass.get::<kilogram>().to_bits(),
            12.5_f64.to_bits()
        );

        let mass_model =
            build_mass_model(&scenario.document, &loaded_models, &assembly).expect("mass model");
        assert_eq!(
            mass_model
                .mass_kg(SimTime::ZERO)
                .expect("mass from model")
                .to_bits(),
            12.5_f64.to_bits()
        );
    }
}
