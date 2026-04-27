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
//! [`crate::runner::dispatch`]).
//!
//! Telemetry stays on the Phase-1 seven-channel schema in 2.11.A;
//! atmosphere sample, per-model force breakdown, and resolved-file
//! digests in the header land in 2.11.B.

use nalgebra::Vector3;
use openbmp_aero::AeroDeck;
use openbmp_core::{ChannelId, Duration, ModelId, Position3, SimTime, Velocity3};
use openbmp_env::UsStandard1976;
use openbmp_propulsion::{Motor, SolidMotor};
use openbmp_scenario::{Scenario, ScenarioDocument};
use openbmp_sim::{
    ConstantGravityForce, EndTime, EnvironmentModel, ForceModel, Integrator, MassModel,
    NullEnvironment, Rk4FixedStep, SimulationConfig, SimulationKernel, StopCondition, StopReason,
};
use openbmp_state::PointMassState;
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use openbmp_vehicle::{
    AxialDragForceAdapter, BasicVehicle, MotorMassAdapter, MotorThrustForceAdapter, NamedForceModel,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::CliError;
use crate::runner::RunOutcome;

// Stable model-ids assigned to each force / mass model the runner
// wires. Scenario-supplied force-model names ("aero", "thrust") are
// mapped to these; the constant-gravity force is registered via
// `ConstantGravityForce::new` which carries its own internal id.
// Phase-2.7 uses model-ids in the determinism oracle; the runner
// picks fixed values so the per-model telemetry stream (Phase 2.11.B)
// is keyed deterministically.
const PHASE2_AERO_MODEL_ID: ModelId = ModelId::new(102);
const PHASE2_THRUST_MODEL_ID: ModelId = ModelId::new(103);
const PHASE2_MOTOR_MASS_MODEL_ID: ModelId = ModelId::new(104);

#[derive(Debug)]
struct Phase2TelemetryChannels {
    position_x: TelemetryChannel<f64>,
    position_y: TelemetryChannel<f64>,
    position_z: TelemetryChannel<f64>,
    velocity_x: TelemetryChannel<f64>,
    velocity_y: TelemetryChannel<f64>,
    velocity_z: TelemetryChannel<f64>,
    mass: TelemetryChannel<f64>,
}

impl Phase2TelemetryChannels {
    fn new() -> Result<Self, CliError> {
        Ok(Self {
            position_x: TelemetryChannel::<f64>::new(
                ChannelId::new(1),
                "position_x_m",
                "m",
                Some("ECI"),
            )?,
            position_y: TelemetryChannel::<f64>::new(
                ChannelId::new(2),
                "position_y_m",
                "m",
                Some("ECI"),
            )?,
            position_z: TelemetryChannel::<f64>::new(
                ChannelId::new(3),
                "position_z_m",
                "m",
                Some("ECI"),
            )?,
            velocity_x: TelemetryChannel::<f64>::new(
                ChannelId::new(4),
                "velocity_x_m_s",
                "m/s",
                Some("ECI"),
            )?,
            velocity_y: TelemetryChannel::<f64>::new(
                ChannelId::new(5),
                "velocity_y_m_s",
                "m/s",
                Some("ECI"),
            )?,
            velocity_z: TelemetryChannel::<f64>::new(
                ChannelId::new(6),
                "velocity_z_m_s",
                "m/s",
                Some("ECI"),
            )?,
            mass: TelemetryChannel::<f64>::new(ChannelId::new(7), "mass_kg", "kg", None::<&str>)?,
        })
    }

    fn schema(&self) -> Result<TelemetrySchema, CliError> {
        Ok(TelemetrySchema::new(vec![
            self.position_x.metadata().clone(),
            self.position_y.metadata().clone(),
            self.position_z.metadata().clone(),
            self.velocity_x.metadata().clone(),
            self.velocity_y.metadata().clone(),
            self.velocity_z.metadata().clone(),
            self.mass.metadata().clone(),
        ])?)
    }
}

/// Run a Phase-2 point-mass scenario through a freshly-built kernel
/// and return the populated telemetry table.
///
/// # Errors
///
/// Returns [`CliError::UnsupportedScenario`] when the scenario shape
/// does not match the Phase-2 point-mass contract,
/// [`CliError::Aero`] / [`CliError::Motor`] / [`CliError::Env`] for
/// loader failures, and [`CliError::Simulation`] / [`CliError::Telemetry`]
/// for kernel- or telemetry-side failures.
pub fn run(scenario: &Scenario) -> Result<RunOutcome, CliError> {
    let document = &scenario.document;
    require_supported_shape(document)?;

    let initial_state = build_initial_state(scenario)?;
    let vehicle = build_vehicle(scenario)?;
    let mass_model = build_mass_model(scenario)?;

    let config = SimulationConfig {
        initial_state,
        integrator: Rk4FixedStep,
        force_model: vehicle,
        mass_model,
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(document.time.stop_s)),
        dt: Duration::from_seconds(document.time.dt_s),
        scenario_seed: document.time.seed,
    };

    let mut kernel = SimulationKernel::new(config)?;
    let channels = Phase2TelemetryChannels::new()?;
    let mut table = TelemetryTable::new(channels.schema()?);

    record_step(&mut table, &kernel, &channels)?;
    while kernel.stop_reason().is_none() {
        kernel.step()?;
        record_step(&mut table, &kernel, &channels)?;
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

fn build_initial_state(scenario: &Scenario) -> Result<PointMassState, CliError> {
    let document = &scenario.document;
    let p = document.vehicle.initial_position_eci_m;
    let v = document.vehicle.initial_velocity_eci_m_s;

    // Total mass at t=0 = scenario `mass_kg` PLUS the motor's loaded
    // mass when a motor is declared. The scenario-side `mass_kg` is
    // the dry-airframe mass (no motor). MotorMassAdapter mirrors this
    // by adding `dry_mass_kg + propellant_mass_kg` from the motor file
    // at ignition time.
    let total_mass_kg = if let Some(motor_path) = motor_file_path(scenario) {
        let motor = SolidMotor::load_from_toml(motor_path.as_path())?;
        document.vehicle.mass_kg + motor.dry_mass_kg() + motor.propellant_mass_kg()
    } else {
        document.vehicle.mass_kg
    };

    Ok(PointMassState::new(
        SimTime::from_seconds(document.time.start_s),
        Position3::new(p[0], p[1], p[2]),
        Velocity3::new(v[0], v[1], v[2]),
        Mass::new::<kilogram>(total_mass_kg),
    ))
}

fn build_vehicle(scenario: &Scenario) -> Result<BasicVehicle<PointMassState>, CliError> {
    let document = &scenario.document;
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
                let aero_path =
                    aero_deck_path(scenario).ok_or_else(|| CliError::UnsupportedScenario {
                        what: "forces includes `aero` but [aero] block is missing".to_owned(),
                    })?;
                let deck = AeroDeck::load_from_toml(aero_path.as_path())?;
                let atmosphere = UsStandard1976::new();
                let drag = AxialDragForceAdapter::new(deck, atmosphere, PHASE2_AERO_MODEL_ID);
                named.push(NamedForceModel::new("aero", Box::new(drag)));
            }
            "thrust" => {
                let motor_path =
                    motor_file_path(scenario).ok_or_else(|| CliError::UnsupportedScenario {
                        what: "forces includes `thrust` but [propulsion.motor] is missing"
                            .to_owned(),
                    })?;
                let motor = SolidMotor::load_from_toml(motor_path.as_path())?;
                let ignition_time_s = document
                    .propulsion
                    .as_ref()
                    .and_then(|p| p.motor.as_ref())
                    .map_or(0.0, |m| m.ignite_at_s);
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
    let vehicle_mass = build_mass_model(scenario)?;
    BasicVehicle::new(named, vec![], Box::new(vehicle_mass)).map_err(|e| {
        CliError::UnsupportedScenario {
            what: format!("BasicVehicle construction failed: {e}"),
        }
    })
}

fn build_mass_model(scenario: &Scenario) -> Result<MotorMassAdapter<SolidMotor>, CliError> {
    let document = &scenario.document;
    let motor_path = motor_file_path(scenario).ok_or_else(|| CliError::UnsupportedScenario {
        what: "Phase-2.11.A point-mass runner requires [propulsion.motor]; \
               Phase-1 byte-stable runner handles motorless cases"
            .to_owned(),
    })?;
    let motor = SolidMotor::load_from_toml(motor_path.as_path())?;
    let ignition_time_s = document
        .propulsion
        .as_ref()
        .and_then(|p| p.motor.as_ref())
        .map_or(0.0, |m| m.ignite_at_s);
    Ok(MotorMassAdapter::new(
        motor,
        document.vehicle.mass_kg,
        ignition_time_s,
        PHASE2_MOTOR_MASS_MODEL_ID,
    ))
}

fn aero_deck_path(scenario: &Scenario) -> Option<std::path::PathBuf> {
    scenario
        .document
        .aero
        .as_ref()
        .map(|a| scenario.resolve_path(&a.deck))
}

fn motor_file_path(scenario: &Scenario) -> Option<std::path::PathBuf> {
    scenario
        .document
        .propulsion
        .as_ref()
        .and_then(|p| p.motor.as_ref())
        .map(|m| scenario.resolve_path(&m.file))
}

fn record_step<I, F, MM, E, SC>(
    table: &mut TelemetryTable,
    kernel: &SimulationKernel<PointMassState, I, F, MM, E, SC>,
    channels: &Phase2TelemetryChannels,
) -> Result<(), CliError>
where
    I: Integrator<PointMassState>,
    F: ForceModel<PointMassState>,
    MM: MassModel,
    E: EnvironmentModel,
    SC: StopCondition<PointMassState>,
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

    table.push_row(row)?;
    Ok(())
}
