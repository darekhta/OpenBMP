//! Scenario → kernel → telemetry adapter for Phase-2 rigid-body
//! scenarios. Mirrors [`crate::runner::phase2_point_mass`] but
//! consumes the Phase-3.1 rigid-body adapter family in
//! [`openbmp_vehicle::adapters`].
//!
//! Accepted scenario shape:
//!
//! - `vehicle.kind = "rigid_body"`
//! - `environment.gravity = "constant"` with non-negative
//!   `gravity_m_s2`
//! - `[atmosphere].kind = "us_standard_1976"` (when `forces`
//!   includes `aero`)
//! - `[aero].deck = "<path>"` with optional pinned digest
//! - `[propulsion.motor].file = "<path>"` with optional pinned digest
//! - `forces.models` entries permuted from `["gravity", "aero",
//!   "thrust"]`
//! - `vehicle.initial_quaternion_body_to_eci_xyzw`,
//!   `vehicle.initial_angular_velocity_body_rad_s`, and
//!   `vehicle.inertia_tensor_body_kg_m2` declared (parser already
//!   enforces these for `kind = "rigid_body"`).
//!
//! Wind models, body-frame moments, and aero side-force / pitching
//! moment are deferred to Phase 3.4 / 3.5 / 3.8. Phase 3.1 ships
//! the *adapter* family and a runner that exercises it on the
//! Niskanen scenario; the moment model defaults to `ZeroMoment` so
//! identity-orientation scenarios produce trajectories
//! indistinguishable (within IEEE 754 reduction order) from the
//! point-mass path.
//!
//! Telemetry layout: same as the point-mass path
//! ([`crate::runner::phase2_point_mass`]) plus four quaternion
//! channels (`attitude.q_x`, `q_y`, `q_z`, `q_w`) and three
//! body-frame angular-velocity channels
//! (`angular_velocity.x_rad_s` etc., frame `Body`).

use std::collections::BTreeMap;

use nalgebra::{Matrix3, Vector3};
use openbmp_aero::{AeroDeck, AeroError};
use openbmp_core::{
    AngularVelocity3, Body, ChannelId, Duration, ModelId, Position3, Quaternion, SimTime, Velocity3,
};
use openbmp_env::{AtmosphereModel, ConstantGravity, UsStandard1976};
use openbmp_propulsion::{Motor, MotorError, SolidMotor};
use openbmp_scenario::{ResolvedFile, Scenario, ScenarioDocument};
use openbmp_sim::{
    ConstantMassRigid, EndTime, EnvironmentSample, ForceContext, ForceModel, NullEnvironment,
    RigidModels, Rk4FixedStep, SimulationConfig, SimulationKernel, StopReason, ZeroMoment,
};
use openbmp_state::{MassProperties, RigidBodyState};
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use openbmp_vehicle::{
    AxialDragForceAdapter, BasicVehicle, BoxedMassModel, GravityForceAdapter,
    MotorThrustForceAdapter, NamedForceModel, RigidMotorMassAdapter, Vehicle,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::CliError;
use crate::runner::RunOutcome;

// Stable model-ids assigned to each force / mass model the rigid
// runner wires. Reserves a separate range from the Phase-2 point-mass
// runner so Phase-2.7 determinism tooling can distinguish the two
// paths.
const PHASE3_GRAVITY_MODEL_ID: ModelId = ModelId::new(301);
const PHASE3_AERO_MODEL_ID: ModelId = ModelId::new(302);
const PHASE3_THRUST_MODEL_ID: ModelId = ModelId::new(303);
const PHASE3_MOTOR_MASS_MODEL_ID: ModelId = ModelId::new(304);

/// Run a Phase-2 rigid-body scenario through a freshly-built kernel
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
/// does not match the rigid-body Phase-3.1 contract,
/// [`CliError::Aero`] / [`CliError::Motor`] / [`CliError::Env`] for
/// loader failures, and [`CliError::Simulation`] / [`CliError::Telemetry`]
/// for kernel- or telemetry-side failures.
pub fn run(
    scenario: &Scenario,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<RunOutcome, CliError> {
    let document = &scenario.document;
    require_supported_shape(document)?;

    let loaded = load_models(document, resolved_files)?;
    let initial_state = build_initial_state(document, &loaded)?;
    let kernel_vehicle = build_vehicle(document, &loaded)?;
    let breakdown_vehicle = build_vehicle(document, &loaded)?;
    let mass_model = build_mass_model(document, &loaded)?;
    let rigid_models = RigidModels::new(ZeroMoment, mass_model);

    let config = SimulationConfig {
        initial_state,
        integrator: Rk4FixedStep,
        force_model: kernel_vehicle,
        mass_model: rigid_models,
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(document.time.stop_s)),
        dt: Duration::from_seconds(document.time.dt_s),
        scenario_seed: document.time.seed,
    };

    let kernel_base = SimulationKernel::new_rigid(config)?;
    let mut kernel = if let Some(mission) = &document.mission {
        let (events, graph) = crate::runner::mission::build_mission_runtime(mission)?;
        kernel_base.with_mission(events, Some(graph))?
    } else {
        kernel_base
    };
    let channel_set = RigidChannelSet::new(document)?;
    let breakdown_atmosphere = if channel_set.has_atmosphere {
        Some(UsStandard1976::new())
    } else {
        None
    };
    let metadata = build_schema_metadata(resolved_files);
    let mut table = TelemetryTable::new(channel_set.schema(metadata)?);

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

#[derive(Clone, Debug, Default)]
struct LoadedModels {
    aero_deck: Option<AeroDeck>,
    motor: Option<SolidMotor>,
}

fn require_supported_shape(document: &ScenarioDocument) -> Result<(), CliError> {
    if document.vehicle.kind != "rigid_body" {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "vehicle.kind = {} (expected rigid_body)",
                document.vehicle.kind
            ),
        });
    }
    if document.environment.gravity != "constant" {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "environment.gravity = {} (only `constant` wired in 3.1)",
                document.environment.gravity
            ),
        });
    }
    for name in &document.forces.models {
        if !matches!(name.as_str(), "gravity" | "aero" | "thrust") {
            return Err(CliError::UnsupportedScenario {
                what: format!("forces.models entry `{name}` (only gravity, aero, thrust wired)"),
            });
        }
    }
    if document.environment.wind != "none" {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "environment.wind = {} (Phase-3.1 wires no wind models)",
                document.environment.wind
            ),
        });
    }
    if let Some(wind) = &document.wind
        && wind.kind != "none"
    {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "[wind].kind = {} (Phase-3.1 wires no wind models)",
                wind.kind
            ),
        });
    }
    let has_aero = document.forces.models.iter().any(|m| m == "aero");
    if has_aero {
        let kind = document.atmosphere.as_ref().map_or_else(
            || document.environment.atmosphere.as_str(),
            |a| a.kind.as_str(),
        );
        if kind != "us_standard_1976" {
            return Err(CliError::UnsupportedScenario {
                what: format!(
                    "atmosphere `{kind}` is not wired with the aero force in 3.1; \
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
    loaded: &LoadedModels,
) -> Result<RigidBodyState, CliError> {
    let p = document.vehicle.initial_position_eci_m;
    let v = document.vehicle.initial_velocity_eci_m_s;
    let q = document
        .vehicle
        .initial_quaternion_body_to_eci_xyzw
        .ok_or_else(|| CliError::UnsupportedScenario {
            what: "internal invariant: initial_quaternion missing for rigid_body scenario"
                .to_owned(),
        })?;
    let omega = document
        .vehicle
        .initial_angular_velocity_body_rad_s
        .ok_or_else(|| CliError::UnsupportedScenario {
            what: "internal invariant: initial_angular_velocity missing for rigid_body scenario"
                .to_owned(),
        })?;
    let inertia_rows = document.vehicle.inertia_tensor_body_kg_m2.ok_or_else(|| {
        CliError::UnsupportedScenario {
            what: "internal invariant: inertia_tensor missing for rigid_body scenario".to_owned(),
        }
    })?;

    // Total mass at the initial state = scenario `mass_kg` PLUS the
    // motor's current mass when a motor is declared. Mirrors the
    // point-mass runner so a rigid-body Niskanen reproduces the
    // point-mass Niskanen physics under an identity orientation.
    let total_mass_kg = if let Some(motor) = &loaded.motor {
        let t_since_ignition_s = motor_elapsed_at_start_s(document)?;
        document.vehicle.mass_kg + motor.mass_kg(t_since_ignition_s)?
    } else {
        document.vehicle.mass_kg
    };

    let inertia_body = matrix3_from_rows(inertia_rows);
    let mass_props = MassProperties::new(
        Mass::new::<kilogram>(total_mass_kg),
        Position3::origin(),
        inertia_body,
    );

    // Quaternion is [x, y, z, w] in the scenario file; nalgebra
    // expects (w, x, y, z) for `Quaternion::new`. Validation in the
    // scenario layer guarantees unit-norm to 1e-9.
    let raw = nalgebra::Quaternion::new(q[3], q[0], q[1], q[2]);
    let unit = nalgebra::UnitQuaternion::from_quaternion(raw);
    let orientation =
        Quaternion::<openbmp_core::Body, openbmp_core::Eci>::from_unit_quaternion(unit);

    Ok(RigidBodyState::new(
        SimTime::from_seconds(document.time.start_s),
        Position3::new(p[0], p[1], p[2]),
        Velocity3::new(v[0], v[1], v[2]),
        orientation,
        AngularVelocity3::<Body>::new(omega[0], omega[1], omega[2]),
        mass_props,
    ))
}

fn matrix3_from_rows(rows: [[f64; 3]; 3]) -> Matrix3<f64> {
    Matrix3::new(
        rows[0][0], rows[0][1], rows[0][2], rows[1][0], rows[1][1], rows[1][2], rows[2][0],
        rows[2][1], rows[2][2],
    )
}

fn build_vehicle(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
) -> Result<BasicVehicle<RigidBodyState>, CliError> {
    let mut named: Vec<NamedForceModel<RigidBodyState>> = Vec::new();
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
                let model = ConstantGravity::down_z(g)?;
                let force = GravityForceAdapter::new(model, PHASE3_GRAVITY_MODEL_ID);
                named.push(NamedForceModel::new("gravity", Box::new(force)));
            }
            "aero" => {
                let deck =
                    loaded
                        .aero_deck
                        .clone()
                        .ok_or_else(|| CliError::UnsupportedScenario {
                            what: "forces includes `aero` but [aero] block is missing".to_owned(),
                        })?;
                let atmosphere = UsStandard1976::new();
                let drag = AxialDragForceAdapter::new(deck, atmosphere, PHASE3_AERO_MODEL_ID);
                named.push(NamedForceModel::new("aero", Box::new(drag)));
            }
            "thrust" => {
                let motor = loaded
                    .motor
                    .clone()
                    .ok_or_else(|| CliError::UnsupportedScenario {
                        what: "forces includes `thrust` but [propulsion.motor] is missing"
                            .to_owned(),
                    })?;
                let ignition_time_s = motor_ignition_time_s(document)?;
                let thrust =
                    MotorThrustForceAdapter::new(motor, ignition_time_s, PHASE3_THRUST_MODEL_ID);
                named.push(NamedForceModel::new("thrust", Box::new(thrust)));
            }
            other => unreachable!("require_supported_shape rejects unknown force model `{other}`"),
        }
    }

    // BasicVehicle requires a mass model; the kernel keeps a separate
    // copy through `RigidMotorMassAdapter` / `ConstantMassRigid` for
    // its own state propagation. We give the vehicle a scalar
    // `BoxedMassModel` view so the breakdown evaluator can query mass
    // when it needs to.
    let vehicle_mass = build_vehicle_scalar_mass_model(document, loaded)?;
    BasicVehicle::new(named, vec![], Box::new(vehicle_mass)).map_err(|e| {
        CliError::UnsupportedScenario {
            what: format!("BasicVehicle construction failed: {e}"),
        }
    })
}

fn build_vehicle_scalar_mass_model(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
) -> Result<BoxedMassModel, CliError> {
    use openbmp_sim::{ConstantMass, MassModel};
    use openbmp_vehicle::MotorMassAdapter;

    let inner: Box<dyn MassModel> = if let Some(motor) = &loaded.motor {
        Box::new(MotorMassAdapter::new(
            motor.clone(),
            document.vehicle.mass_kg,
            motor_ignition_time_s(document)?,
            PHASE3_MOTOR_MASS_MODEL_ID,
        ))
    } else {
        Box::new(ConstantMass::new(document.vehicle.mass_kg))
    };
    Ok(BoxedMassModel(inner))
}

/// Build the kernel's rigid mass model. When a motor is declared
/// the runner uses `RigidMotorMassAdapter`; otherwise
/// `ConstantMassRigid` over the dry-vehicle mass + scenario inertia.
type RigidMassEither = RigidMassEitherKind;

#[derive(Debug)]
enum RigidMassEitherKind {
    Motor(RigidMotorMassAdapter<SolidMotor>),
    Constant(ConstantMassRigid),
}

impl openbmp_sim::RigidMassModel for RigidMassEitherKind {
    fn mass_properties(&self, t: SimTime) -> Result<MassProperties, openbmp_sim::ModelEvalError> {
        match self {
            Self::Motor(m) => m.mass_properties(t),
            Self::Constant(c) => c.mass_properties(t),
        }
    }

    fn mass_properties_rate(
        &self,
        t: SimTime,
    ) -> Result<openbmp_sim::MassPropertiesRate, openbmp_sim::ModelEvalError> {
        match self {
            Self::Motor(m) => m.mass_properties_rate(t),
            Self::Constant(c) => c.mass_properties_rate(t),
        }
    }
}

fn build_mass_model(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
) -> Result<RigidMassEither, CliError> {
    let inertia_rows = document.vehicle.inertia_tensor_body_kg_m2.ok_or_else(|| {
        CliError::UnsupportedScenario {
            what: "internal invariant: inertia_tensor missing for rigid_body scenario".to_owned(),
        }
    })?;
    let inertia_body = matrix3_from_rows(inertia_rows);

    if let Some(motor) = &loaded.motor {
        Ok(RigidMassEitherKind::Motor(RigidMotorMassAdapter::new(
            motor.clone(),
            document.vehicle.mass_kg,
            Position3::origin(),
            inertia_body,
            motor_ignition_time_s(document)?,
            PHASE3_MOTOR_MASS_MODEL_ID,
        )))
    } else {
        Ok(RigidMassEitherKind::Constant(ConstantMassRigid::new(
            MassProperties::new(
                Mass::new::<kilogram>(document.vehicle.mass_kg),
                Position3::origin(),
                inertia_body,
            ),
        )))
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
// Channel set
// ---------------------------------------------------------------------

type ForceComponentChannels = Vec<(
    String,
    TelemetryChannel<f64>,
    TelemetryChannel<f64>,
    TelemetryChannel<f64>,
)>;

#[derive(Debug)]
struct RigidChannelSet {
    position_x: TelemetryChannel<f64>,
    position_y: TelemetryChannel<f64>,
    position_z: TelemetryChannel<f64>,
    velocity_x: TelemetryChannel<f64>,
    velocity_y: TelemetryChannel<f64>,
    velocity_z: TelemetryChannel<f64>,
    mass: TelemetryChannel<f64>,
    quaternion_x: TelemetryChannel<f64>,
    quaternion_y: TelemetryChannel<f64>,
    quaternion_z: TelemetryChannel<f64>,
    quaternion_w: TelemetryChannel<f64>,
    angular_velocity_x: TelemetryChannel<f64>,
    angular_velocity_y: TelemetryChannel<f64>,
    angular_velocity_z: TelemetryChannel<f64>,
    has_atmosphere: bool,
    atmosphere_density: Option<TelemetryChannel<f64>>,
    atmosphere_pressure: Option<TelemetryChannel<f64>>,
    atmosphere_temperature: Option<TelemetryChannel<f64>>,
    atmosphere_speed_of_sound: Option<TelemetryChannel<f64>>,
    force_components: ForceComponentChannels,
    /// Phase-3.2 mission-event telemetry markers, keyed by tag.
    mission_markers: BTreeMap<String, TelemetryChannel<bool>>,
}

impl RigidChannelSet {
    #[allow(clippy::too_many_lines)]
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

        let quaternion_x =
            TelemetryChannel::<f64>::new(alloc(), "attitude.q_x", "1", None::<&str>)?;
        let quaternion_y =
            TelemetryChannel::<f64>::new(alloc(), "attitude.q_y", "1", None::<&str>)?;
        let quaternion_z =
            TelemetryChannel::<f64>::new(alloc(), "attitude.q_z", "1", None::<&str>)?;
        let quaternion_w =
            TelemetryChannel::<f64>::new(alloc(), "attitude.q_w", "1", None::<&str>)?;
        let angular_velocity_x = TelemetryChannel::<f64>::new(
            alloc(),
            "angular_velocity.x_rad_s",
            "rad/s",
            Some("Body"),
        )?;
        let angular_velocity_y = TelemetryChannel::<f64>::new(
            alloc(),
            "angular_velocity.y_rad_s",
            "rad/s",
            Some("Body"),
        )?;
        let angular_velocity_z = TelemetryChannel::<f64>::new(
            alloc(),
            "angular_velocity.z_rad_s",
            "rad/s",
            Some("Body"),
        )?;

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

        // Phase-3.2 mission marker channels.
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
            quaternion_x,
            quaternion_y,
            quaternion_z,
            quaternion_w,
            angular_velocity_x,
            angular_velocity_y,
            angular_velocity_z,
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
            self.quaternion_x.metadata().clone(),
            self.quaternion_y.metadata().clone(),
            self.quaternion_z.metadata().clone(),
            self.quaternion_w.metadata().clone(),
            self.angular_velocity_x.metadata().clone(),
            self.angular_velocity_y.metadata().clone(),
            self.angular_velocity_z.metadata().clone(),
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
fn record_step<I, F, MOM, MM, E, SC>(
    table: &mut TelemetryTable,
    kernel: &SimulationKernel<RigidBodyState, I, F, RigidModels<MOM, MM>, E, SC>,
    channels: &RigidChannelSet,
    breakdown_vehicle: &BasicVehicle<RigidBodyState>,
    breakdown_atmosphere: Option<&UsStandard1976>,
    fired_events: &[openbmp_sim::FiredEvent],
) -> Result<(), CliError>
where
    I: openbmp_sim::Integrator<RigidBodyState>,
    F: ForceModel<RigidBodyState>,
    MOM: openbmp_sim::MomentModel<RigidBodyState>,
    MM: openbmp_sim::RigidMassModel,
    E: openbmp_sim::EnvironmentModel,
    SC: openbmp_sim::StopCondition<RigidBodyState>,
{
    let state = kernel.current_state();
    let mut row = TelemetryRow::new(state.time, kernel.current_step())?;

    row.insert(&channels.position_x, state.position.vector.x)?;
    row.insert(&channels.position_y, state.position.vector.y)?;
    row.insert(&channels.position_z, state.position.vector.z)?;
    row.insert(&channels.velocity_x, state.velocity.vector.x)?;
    row.insert(&channels.velocity_y, state.velocity.vector.y)?;
    row.insert(&channels.velocity_z, state.velocity.vector.z)?;
    row.insert(&channels.mass, state.mass_props.mass_kg())?;

    let raw = state.orientation.q.into_inner();
    row.insert(&channels.quaternion_x, raw.coords.x)?;
    row.insert(&channels.quaternion_y, raw.coords.y)?;
    row.insert(&channels.quaternion_z, raw.coords.z)?;
    row.insert(&channels.quaternion_w, raw.coords.w)?;
    row.insert(
        &channels.angular_velocity_x,
        state.angular_velocity.vector.x,
    )?;
    row.insert(
        &channels.angular_velocity_y,
        state.angular_velocity.vector.y,
    )?;
    row.insert(
        &channels.angular_velocity_z,
        state.angular_velocity.vector.z,
    )?;

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

    let env_sample = EnvironmentSample::default();
    let ctx = ForceContext {
        state,
        environment: &env_sample,
        mass_kg: state.mass_props.mass_kg(),
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
            .map_or_else(Vector3::<f64>::zeros, |(_, vector)| *vector);
        row.insert(x_channel, component.x)?;
        row.insert(y_channel, component.y)?;
        row.insert(z_channel, component.z)?;
    }

    // Phase-3.2 marker channels.
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

// Silence the unused alias warning when no consumer references it
// directly; the alias keeps `RigidMassEither` available as the public
// shape of the kernel mass model.
#[allow(dead_code)]
type _RigidMassEitherAlias = RigidMassEither;
