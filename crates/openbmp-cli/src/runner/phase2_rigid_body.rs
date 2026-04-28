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
//! Wind models and aero side-force / pitching moment are deferred to
//! later Phase-3 sub-phases. Phase 3.6 wires rigid-body engine-cluster
//! moments through `EngineClusterMomentAdapter`; scenarios without
//! engine clusters still default to `ZeroMoment` so identity-orientation
//! single-motor scenarios produce trajectories indistinguishable
//! (within IEEE 754 reduction order) from the point-mass path.
//!
//! Telemetry layout: same as the point-mass path
//! ([`crate::runner::phase2_point_mass`]) plus four quaternion
//! channels (`attitude.q_x`, `q_y`, `q_z`, `q_w`) and three
//! body-frame angular-velocity channels
//! (`angular_velocity.x_rad_s` etc., frame `Body`).

use std::collections::BTreeMap;

use nalgebra::Vector3;
use openbmp_aero::{AeroDeck, AeroError};
use openbmp_core::{
    AngularVelocity3, Body, ChannelId, Duration, ModelId, Position3, Quaternion, SimTime,
    ValidationStatus, Velocity3,
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
    AxialDragForceAdapter, BasicAssembly, BasicVehicle, BoxedMassModel, EngineClusterForceAdapter,
    EngineClusterMassAdapter, EngineClusterMomentAdapter, GravityForceAdapter,
    MotorThrustForceAdapter, NamedForceModel, RigidMotorMassAdapter, Vehicle,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::CliError;
use crate::runner::RunOutcome;
use crate::runner::assembly::{dry_mass_kg_at, dry_mass_properties_at};

// Stable model-ids assigned to each force / mass model the rigid
// runner wires. Reserves a separate range from the Phase-2 point-mass
// runner so Phase-2.7 determinism tooling can distinguish the two
// paths.
const PHASE3_GRAVITY_MODEL_ID: ModelId = ModelId::new(301);
const PHASE3_AERO_MODEL_ID: ModelId = ModelId::new(302);
const PHASE3_THRUST_MODEL_ID: ModelId = ModelId::new(303);
const PHASE3_MOTOR_MASS_MODEL_ID: ModelId = ModelId::new(304);
// Phase-3.6: distinct model ids for the engine-cluster path on the
// rigid-body kernel.
const PHASE3_ENGINE_CLUSTER_THRUST_MODEL_ID: ModelId = ModelId::new(320);
const PHASE3_ENGINE_CLUSTER_MASS_MODEL_ID: ModelId = ModelId::new(321);
const PHASE3_ENGINE_CLUSTER_MOMENT_MODEL_ID: ModelId = ModelId::new(322);
const PHASE3_TANK_RACK_FORCE_MODEL_ID: ModelId = ModelId::new(340);
const PHASE3_TANK_RACK_MOMENT_MODEL_ID: ModelId = ModelId::new(341);

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
#[allow(clippy::too_many_lines)] // Phase-3.6: per-step orchestration grew
pub fn run(
    scenario: &Scenario,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<RunOutcome, CliError> {
    let document = &scenario.document;
    require_supported_shape(document)?;
    let assembly = crate::runner::assembly::synthesize_assembly(document)?;
    // Phase-3.4: same effector-rack pattern as the point-mass
    // runner. Empty rack means no per-step effector operations.
    let mut effector_rack = crate::runner::effectors::EffectorRack::build(document)?;
    // Phase-3.6: see phase2_point_mass.rs for the rationale.
    let mut engine_rack = crate::runner::engines::EngineRack::build(document)?;
    // Phase-3.7: tank rack mirroring the point-mass runner.
    let mut tank_rack = crate::runner::tanks::TankRack::build(document)?;
    // Phase-3.8: build the runner-side wind rack. Inactive when no
    // `[wind]` block is declared (or `kind = "none"`).
    let wind_rack = crate::runner::wind::WindRack::build(document)?;
    wind_rack.reset();

    let loaded = load_models(document, resolved_files)?;
    let initial_state = build_initial_state(document, &loaded, &assembly)?;
    let kernel_vehicle = build_vehicle(document, &loaded, &assembly)?;
    let breakdown_vehicle = build_vehicle(document, &loaded, &assembly)?;
    let mass_model = build_mass_model(document, &loaded, &assembly)?;
    let moment_model = build_moment_model(document)?;
    let rigid_models = RigidModels::new(moment_model, mass_model);

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

    // Phase-3.5.C: see phase2_point_mass.rs sibling for the rationale.
    let deck_bindings = crate::runner::aero_effector_match::assert_axes_match_effectors(
        loaded.aero_deck.as_ref(),
        document,
    )?;

    let initial_snapshot = effector_rack.snapshot();
    if !deck_bindings.is_empty() {
        let snapshot_map = crate::runner::aero_effector_match::build_snapshot_map(
            &deck_bindings,
            &initial_snapshot,
        );
        kernel.set_effector_actuals(snapshot_map);
    }
    if !engine_rack.is_empty() {
        kernel.set_engine_snapshot(engine_rack.snapshot_map());
    }
    if !tank_rack.is_empty() {
        kernel.set_tank_snapshot(tank_rack.snapshot_map());
    }
    if !wind_rack.is_inactive() {
        let s = kernel.current_state();
        let frame = openbmp_core::FrameContext::toy_fixed_earth();
        let wind = wind_rack.sample(s.position, &frame, s.time)?;
        kernel.set_wind_sample(wind);
    }
    record_step(
        &mut table,
        &kernel,
        &channel_set,
        &breakdown_vehicle,
        breakdown_atmosphere.as_ref(),
        &[],
        &initial_snapshot,
    )?;
    let mut pending_effector_events = Vec::new();
    let mut pending_engine_events: Vec<openbmp_sim::FiredEvent> = Vec::new();
    while kernel.stop_reason().is_none() {
        effector_rack.apply_overrides(&pending_effector_events)?;
        if !effector_rack.is_empty() {
            effector_rack.step(kernel.current_time())?;
        }
        if !engine_rack.is_empty() {
            engine_rack.apply_commands(&pending_engine_events)?;
            engine_rack.step()?;
        }
        // Phase-3.7: advance tanks using prior-step cached drivers.
        // The drivers are updated post-step from the new rigid-body
        // state's angular_velocity (omega_body) and a finite-
        // difference body-frame acceleration; the first step uses
        // zeros (initialised by `TankRack::build`).
        if !tank_rack.is_empty() {
            tank_rack.step()?;
        }
        if !deck_bindings.is_empty() {
            let rack_snapshot = effector_rack.snapshot();
            let snapshot_map = crate::runner::aero_effector_match::build_snapshot_map(
                &deck_bindings,
                &rack_snapshot,
            );
            kernel.set_effector_actuals(snapshot_map);
        }
        if !engine_rack.is_empty() {
            kernel.set_engine_snapshot(engine_rack.snapshot_map());
        }
        if !tank_rack.is_empty() {
            kernel.set_tank_snapshot(tank_rack.snapshot_map());
        }
        if !wind_rack.is_inactive() {
            wind_rack.advance(kernel.current_step());
            let s = kernel.current_state();
            let frame = openbmp_core::FrameContext::toy_fixed_earth();
            let wind = wind_rack.sample(s.position, &frame, s.time)?;
            kernel.set_wind_sample(wind);
        }
        let prev_velocity_eci = kernel.current_state().velocity.vector;
        let prev_orientation = kernel.current_state().orientation.q;
        kernel.step()?;
        // Phase-3.7: refresh tank-rack drivers from the post-step
        // rigid-body state. `accel_body_m_s2` is finite-differenced
        // from the velocity change rotated into the prior-step body
        // frame; `omega_body_rad_s` is read directly from the new
        // state. Slosh state on the next tick uses these drivers
        // (one-step lag, see TankRack module docs).
        if !tank_rack.is_empty() {
            let dt_s = document.time.dt_s;
            let new_state = kernel.current_state();
            let dv_eci = new_state.velocity.vector - prev_velocity_eci;
            let accel_eci = if dt_s > 0.0 {
                dv_eci / dt_s
            } else {
                nalgebra::Vector3::zeros()
            };
            // Rotate ECI accel into prior-step body frame: the slosh
            // dynamics react to body-frame accel, and the prior body
            // frame matches the slosh state's reference.
            let inverse_orientation = prev_orientation.inverse();
            let accel_body = inverse_orientation * accel_eci;
            let omega_body = new_state.angular_velocity.vector;
            tank_rack.update_drivers(accel_body, omega_body);
        }
        let fired = kernel.drain_events();
        let snapshot = effector_rack.snapshot();
        record_step(
            &mut table,
            &kernel,
            &channel_set,
            &breakdown_vehicle,
            breakdown_atmosphere.as_ref(),
            &fired,
            &snapshot,
        )?;
        pending_engine_events = fired
            .iter()
            .filter(|e| matches!(e.action, openbmp_sim::EventAction::EngineCommand { .. }))
            .cloned()
            .collect();
        pending_effector_events = fired;
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
    assembly: &BasicAssembly,
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
    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_props = dry_mass_properties_at(assembly, start_time, "vehicle.assembly")?;

    // Total mass at the initial state = assembly dry mass PLUS the
    // motor's current mass when a motor is declared. Mirrors the
    // point-mass runner so a rigid-body Niskanen reproduces the
    // point-mass Niskanen physics under an identity orientation.
    let mass_props = if let Some(motor) = &loaded.motor {
        let t_since_ignition_s = motor_elapsed_at_start_s(document)?;
        MassProperties::new(
            Mass::new::<kilogram>(
                dry_props.mass.get::<kilogram>() + motor.mass_kg(t_since_ignition_s)?,
            ),
            dry_props.center_of_mass_body,
            dry_props.inertia_body,
        )
    } else {
        dry_props
    };

    // Quaternion is [x, y, z, w] in the scenario file; nalgebra
    // expects (w, x, y, z) for `Quaternion::new`. Validation in the
    // scenario layer guarantees unit-norm to 1e-9.
    let raw = nalgebra::Quaternion::new(q[3], q[0], q[1], q[2]);
    let unit = nalgebra::UnitQuaternion::from_quaternion(raw);
    let orientation =
        Quaternion::<openbmp_core::Body, openbmp_core::Eci>::from_unit_quaternion(unit);

    Ok(RigidBodyState::new(
        start_time,
        Position3::new(p[0], p[1], p[2]),
        Velocity3::new(v[0], v[1], v[2]),
        orientation,
        AngularVelocity3::<Body>::new(omega[0], omega[1], omega[2]),
        mass_props,
    ))
}

fn build_vehicle(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    assembly: &BasicAssembly,
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
                // Phase-3.6: dispatch between single-motor and
                // engine-cluster paths. AmbiguousPropulsion is
                // rejected at parse time.
                if let Some(scenario_assembly) = &document.vehicle.assembly
                    && !scenario_assembly.engines.is_empty()
                {
                    let engine_ids: Vec<openbmp_core::EngineId> = scenario_assembly
                        .engines
                        .iter()
                        .map(|e| {
                            openbmp_core::EngineId::from_path(&format!(
                                "vehicle.assembly.engines.{id}",
                                id = e.id
                            ))
                        })
                        .collect();
                    let thrust = EngineClusterForceAdapter::new(
                        engine_ids,
                        PHASE3_ENGINE_CLUSTER_THRUST_MODEL_ID,
                    );
                    named.push(NamedForceModel::new("thrust", Box::new(thrust)));
                } else {
                    let motor =
                        loaded
                            .motor
                            .clone()
                            .ok_or_else(|| CliError::UnsupportedScenario {
                                what:
                                    "forces includes `thrust` but neither [propulsion.motor] nor \
                                   [[vehicle.assembly.engines]] is declared"
                                        .to_owned(),
                            })?;
                    let ignition_time_s = motor_ignition_time_s(document)?;
                    let thrust = MotorThrustForceAdapter::new(
                        motor,
                        ignition_time_s,
                        PHASE3_THRUST_MODEL_ID,
                    );
                    named.push(NamedForceModel::new("thrust", Box::new(thrust)));
                }
            }
            other => unreachable!("require_supported_shape rejects unknown force model `{other}`"),
        }
    }

    // Phase-3.7: tank-rack reaction-force adapter (rigid).
    if let Some(scenario_assembly) = &document.vehicle.assembly
        && !scenario_assembly.tanks.is_empty()
    {
        let tank_ids: Vec<openbmp_core::TankId> = scenario_assembly
            .tanks
            .iter()
            .map(|t| {
                openbmp_core::TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = t.id))
            })
            .collect();
        let tank_force =
            openbmp_vehicle::TankRackForceAdapter::new(tank_ids, PHASE3_TANK_RACK_FORCE_MODEL_ID);
        named.push(NamedForceModel::new("tank_reaction", Box::new(tank_force)));
    }

    // BasicVehicle requires a mass model; the kernel keeps a separate
    // copy through `RigidMotorMassAdapter` / `ConstantMassRigid` for
    // its own state propagation. We give the vehicle a scalar
    // `BoxedMassModel` view so the breakdown evaluator can query mass
    // when it needs to.
    let vehicle_mass = build_vehicle_scalar_mass_model(document, loaded, assembly)?;
    BasicVehicle::new(named, vec![], Box::new(vehicle_mass)).map_err(|e| {
        CliError::UnsupportedScenario {
            what: format!("BasicVehicle construction failed: {e}"),
        }
    })
}

fn build_vehicle_scalar_mass_model(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    assembly: &BasicAssembly,
) -> Result<BoxedMassModel, CliError> {
    use openbmp_sim::{ConstantMass, MassModel};
    use openbmp_vehicle::MotorMassAdapter;

    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_mass_kg = dry_mass_kg_at(assembly, start_time, "vehicle.assembly")?;
    let inner: Box<dyn MassModel> = if let Some(scenario_assembly) = &document.vehicle.assembly
        && !scenario_assembly.engines.is_empty()
    {
        // Phase-3.6 cluster path: engine-cluster mass adapter
        // tracks per-engine `consumed_kg` from the kernel snapshot.
        let engine_ids: Vec<openbmp_core::EngineId> = scenario_assembly
            .engines
            .iter()
            .map(|e| {
                openbmp_core::EngineId::from_path(&format!(
                    "vehicle.assembly.engines.{id}",
                    id = e.id
                ))
            })
            .collect();
        Box::new(EngineClusterMassAdapter::new(
            dry_mass_kg,
            engine_ids,
            PHASE3_ENGINE_CLUSTER_MASS_MODEL_ID,
        ))
    } else if let Some(motor) = &loaded.motor {
        Box::new(MotorMassAdapter::new(
            motor.clone(),
            dry_mass_kg,
            motor_ignition_time_s(document)?,
            PHASE3_MOTOR_MASS_MODEL_ID,
        ))
    } else {
        Box::new(ConstantMass::new(dry_mass_kg))
    };
    Ok(BoxedMassModel(inner))
}

type RigidMomentEither = RigidMomentEitherKind;

#[derive(Debug)]
enum RigidMomentEitherKind {
    Zero(ZeroMoment),
    EngineCluster(EngineClusterMomentAdapter),
    TankRack(openbmp_vehicle::TankRackMomentAdapter),
    EngineClusterAndTankRack(
        EngineClusterMomentAdapter,
        openbmp_vehicle::TankRackMomentAdapter,
    ),
}

impl openbmp_sim::MomentModel<RigidBodyState> for RigidMomentEitherKind {
    fn moment_n_m_body(
        &self,
        ctx: openbmp_sim::MomentContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, openbmp_sim::ModelEvalError> {
        match self {
            Self::Zero(z) => {
                <ZeroMoment as openbmp_sim::MomentModel<RigidBodyState>>::moment_n_m_body(z, ctx)
            }
            Self::EngineCluster(c) => <EngineClusterMomentAdapter as openbmp_sim::MomentModel<
                RigidBodyState,
            >>::moment_n_m_body(c, ctx),
            Self::TankRack(t) => {
                <openbmp_vehicle::TankRackMomentAdapter as openbmp_sim::MomentModel<
                    RigidBodyState,
                >>::moment_n_m_body(t, ctx)
            }
            Self::EngineClusterAndTankRack(c, t) => {
                let cluster = <EngineClusterMomentAdapter as openbmp_sim::MomentModel<
                    RigidBodyState,
                >>::moment_n_m_body(c, ctx)?;
                let tank = <openbmp_vehicle::TankRackMomentAdapter as openbmp_sim::MomentModel<
                    RigidBodyState,
                >>::moment_n_m_body(t, ctx)?;
                Ok(cluster + tank)
            }
        }
    }

    fn validation(&self) -> ValidationStatus {
        match self {
            Self::Zero(z) => {
                <ZeroMoment as openbmp_sim::MomentModel<RigidBodyState>>::validation(z)
            }
            Self::EngineCluster(c) => <EngineClusterMomentAdapter as openbmp_sim::MomentModel<
                RigidBodyState,
            >>::validation(c),
            Self::TankRack(t) | Self::EngineClusterAndTankRack(_, t) => {
                <openbmp_vehicle::TankRackMomentAdapter as openbmp_sim::MomentModel<
                    RigidBodyState,
                >>::validation(t)
            }
        }
    }
}

fn build_moment_model(document: &ScenarioDocument) -> Result<RigidMomentEither, CliError> {
    let cluster_adapter = if let Some(scenario_assembly) = &document.vehicle.assembly
        && !scenario_assembly.engines.is_empty()
    {
        let engine_ids: Vec<openbmp_core::EngineId> = scenario_assembly
            .engines
            .iter()
            .map(|e| {
                openbmp_core::EngineId::from_path(&format!(
                    "vehicle.assembly.engines.{id}",
                    id = e.id
                ))
            })
            .collect();
        let mount_points_body: Vec<Position3<Body>> = scenario_assembly
            .engines
            .iter()
            .map(|e| {
                Position3::<Body>::new(
                    e.mount_point_body_m[0],
                    e.mount_point_body_m[1],
                    e.mount_point_body_m[2],
                )
            })
            .collect();
        let adapter = EngineClusterMomentAdapter::new(
            engine_ids,
            mount_points_body,
            PHASE3_ENGINE_CLUSTER_MOMENT_MODEL_ID,
        )
        .map_err(|err| CliError::UnsupportedScenario {
            what: format!("EngineClusterMomentAdapter construction failed: {err}"),
        })?;
        Some(adapter)
    } else {
        None
    };

    let tank_adapter = if let Some(scenario_assembly) = &document.vehicle.assembly
        && !scenario_assembly.tanks.is_empty()
    {
        let tank_ids: Vec<openbmp_core::TankId> = scenario_assembly
            .tanks
            .iter()
            .map(|t| {
                openbmp_core::TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = t.id))
            })
            .collect();
        Some(openbmp_vehicle::TankRackMomentAdapter::new(
            tank_ids,
            PHASE3_TANK_RACK_MOMENT_MODEL_ID,
        ))
    } else {
        None
    };

    Ok(match (cluster_adapter, tank_adapter) {
        (Some(c), Some(t)) => RigidMomentEitherKind::EngineClusterAndTankRack(c, t),
        (Some(c), None) => RigidMomentEitherKind::EngineCluster(c),
        (None, Some(t)) => RigidMomentEitherKind::TankRack(t),
        (None, None) => RigidMomentEitherKind::Zero(ZeroMoment),
    })
}

/// Build the kernel's rigid mass model. When a motor is declared
/// the runner uses `RigidMotorMassAdapter`; otherwise
/// `ConstantMassRigid` over assembly dry mass properties.
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
    assembly: &BasicAssembly,
) -> Result<RigidMassEither, CliError> {
    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_props = dry_mass_properties_at(assembly, start_time, "vehicle.assembly")?;

    // Phase-3.6 rigid + engine cluster: propellant deficit isn't
    // tracked in rigid mass-properties yet (that's Phase 3.7's
    // tank-driven mass-property dynamics work). Fall through to
    // `ConstantMassRigid` — the cluster's `EngineClusterForceAdapter`
    // still applies thrust normally; only mass-properties is
    // simplified.
    if let Some(scenario_assembly) = &document.vehicle.assembly
        && !scenario_assembly.engines.is_empty()
    {
        Ok(RigidMassEitherKind::Constant(ConstantMassRigid::new(
            dry_props,
        )))
    } else if let Some(motor) = &loaded.motor {
        Ok(RigidMassEitherKind::Motor(RigidMotorMassAdapter::new(
            motor.clone(),
            dry_props.mass.get::<kilogram>(),
            dry_props.center_of_mass_body,
            dry_props.inertia_body,
            motor_ignition_time_s(document)?,
            PHASE3_MOTOR_MASS_MODEL_ID,
        )))
    } else {
        Ok(RigidMassEitherKind::Constant(ConstantMassRigid::new(
            dry_props,
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
    /// Phase-3.4 effector deflection channels, in scenario-declared
    /// order. One `effector.<id>.actual` `f64` channel per declared
    /// effector. Allocated AFTER force breakdown channels and BEFORE
    /// mission markers — same ordering contract as the point-mass
    /// runner.
    effector_actuals: Vec<TelemetryChannel<f64>>,
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

        // Phase-3.4 effector deflection channels, in scenario-declared
        // order. Allocated BEFORE mission markers so adding effectors
        // does not shift marker channel ids.
        let mut effector_actuals: Vec<TelemetryChannel<f64>> = Vec::new();
        if let Some(assembly) = &document.vehicle.assembly {
            for config in &assembly.effectors {
                let channel = TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("effector.{}.actual", config.id),
                    config.unit.as_deref().unwrap_or("1"),
                    None::<&str>,
                )?;
                effector_actuals.push(channel);
            }
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
            effector_actuals,
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
        // Phase-3.4 effector deflection channels, in scenario-declared
        // order, between force breakdown and mission markers.
        for actual in &self.effector_actuals {
            channels.push(actual.metadata().clone());
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
    effector_snapshot: &[openbmp_vehicle::EffectorState],
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

    // Phase-3.5.C: see phase2_point_mass.rs sibling for the
    // breakdown / kernel snapshot symmetry rationale.
    let env_sample = EnvironmentSample::default();
    let kernel_actuals = kernel.effector_actuals();
    let kernel_engine_snapshot = kernel.engine_snapshot();
    let kernel_tank_snapshot = kernel.tank_snapshot();
    let ctx = ForceContext {
        state,
        environment: &env_sample,
        mass_kg: state.mass_props.mass_kg(),
        time: state.time,
        effector_actuals: openbmp_sim::EffectorActualsView::new(kernel_actuals),
        engine_snapshot: openbmp_sim::EngineSnapshotView::new(kernel_engine_snapshot),
        tank_snapshot: openbmp_sim::TankSnapshotView::new(kernel_tank_snapshot),
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

    // Phase-3.4 effector deflection channels, in scenario-declared
    // order, matching `channels.effector_actuals`.
    debug_assert_eq!(effector_snapshot.len(), channels.effector_actuals.len());
    for (channel, state) in channels
        .effector_actuals
        .iter()
        .zip(effector_snapshot.iter())
    {
        row.insert(channel, state.actual)?;
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
