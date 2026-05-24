//! Scenario → kernel → telemetry adapter for rigid-body
//! scenarios. Mirrors [`crate::point_mass`] but
//! consumes the rigid-body adapter family in
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
//!   `vehicle.initial_angular_velocity_body_rad_s`, and per-body
//!   `vehicle.assembly.bodies[*].dry_inertia_body_kg_m2` declared
//!   (parser already enforces these for `kind = "rigid_body"`).
//!
//! Rigid-body engine-cluster
//! moments are wired through `EngineClusterMomentAdapter`; scenarios without
//! engine clusters still default to `ZeroMoment` so identity-orientation
//! single-motor scenarios produce trajectories indistinguishable
//! (within IEEE 754 reduction order) from the point-mass path.
//!
//! Telemetry layout: same as the point-mass path
//! ([`crate::point_mass`]) plus four quaternion
//! channels (`attitude.q_x`, `q_y`, `q_z`, `q_w`) and three
//! body-frame angular-velocity channels
//! (`angular_velocity.x_rad_s` etc., frame `Body`).

use std::collections::BTreeMap;

use nalgebra::Vector3;
use openbmp_aero::{AeroDeck, AeroError};
use openbmp_core::{
    AngularVelocity3, Body, ChannelId, Duration, ModelId, Position3, Quaternion, RecoveryId,
    SimTime, ValidationStatus, Velocity3,
};
use openbmp_physics::{
    AtmosphereModel, ConstantGravity, Egm2008ZonalGravity, J2Gravity, PointMassGravity, WGS84_J2,
};
use openbmp_propulsion::{Motor, MotorError, SolidMotor};
use openbmp_scenario::{ResolvedFile, Scenario, ScenarioDocument};
use openbmp_sim::{
    ConstantMassRigid, EndTime, ForceContext, ForceModel, NullEnvironment, RigidModels,
    ScenarioScriptAction, SimulationConfig, SimulationKernel, StopReason, ZeroMoment,
};
use openbmp_state::{MassProperties, RigidBodyState};
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use openbmp_vehicle::{
    Assembly, BoxedMassModel, DeckDragForceAdapter, EngineClusterForceAdapter,
    EngineClusterMassAdapter, EngineClusterMomentAdapter, GravityForceAdapter, KernelVehicle,
    MotorThrustForceAdapter, NamedForceModel, RigidMotorMassAdapter, Vehicle,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::RunnerError;
use crate::RunOutcome;
use crate::assembly::{dry_mass_kg_at, dry_mass_properties_at};
use crate::atmosphere::{
    RuntimeAtmosphere, build_runtime_atmosphere, is_runtime_atmosphere_kind,
    scenario_atmosphere_kind,
};
use crate::integrator::build_runtime_integrator;

// Stable model-ids assigned to each force / mass model the rigid
// runner wires. Reserves a separate range from the point-mass
// runner so determinism tooling can distinguish the two
// paths.
const RIGID_BODY_GRAVITY_MODEL_ID: ModelId = ModelId::new(301);
const RIGID_BODY_AERO_MODEL_ID: ModelId = ModelId::new(302);
const RIGID_BODY_THRUST_MODEL_ID: ModelId = ModelId::new(303);
const RIGID_BODY_MOTOR_MASS_MODEL_ID: ModelId = ModelId::new(304);
// Distinct model ids for the engine-cluster path on the
// rigid-body kernel.
const RIGID_BODY_ENGINE_CLUSTER_THRUST_MODEL_ID: ModelId = ModelId::new(320);
const RIGID_BODY_ENGINE_CLUSTER_MASS_MODEL_ID: ModelId = ModelId::new(321);
const RIGID_BODY_ENGINE_CLUSTER_MOMENT_MODEL_ID: ModelId = ModelId::new(322);
const RIGID_BODY_TANK_RACK_FORCE_MODEL_ID: ModelId = ModelId::new(340);
const RIGID_BODY_TANK_RACK_MOMENT_MODEL_ID: ModelId = ModelId::new(341);
const RIGID_BODY_RECOVERY_RACK_FORCE_MODEL_ID: ModelId = ModelId::new(380);
// Distinct model id for the direct-torque moment
// adapter on the rigid-body kernel.
const RIGID_BODY_DIRECT_TORQUE_MOMENT_MODEL_ID: ModelId = ModelId::new(500);

/// Run a rigid-body scenario through a freshly-built kernel
/// and return the populated telemetry table.
///
/// `resolved_files` is the digest map produced by
/// [`Scenario::resolved_files`]; the runner records each entry as
/// `openbmp.scenario_files.<field>` schema metadata so the Parquet
/// header carries the SHA-256 pins for replay verification.
///
/// # Errors
///
/// Returns [`RunnerError::UnsupportedScenario`] when the scenario shape
/// does not match the rigid-body contract,
/// [`RunnerError::Aero`] / [`RunnerError::Motor`] / [`RunnerError::Env`] for
/// loader failures, and [`RunnerError::Simulation`] / [`RunnerError::Telemetry`]
/// for kernel- or telemetry-side failures.
#[allow(clippy::too_many_lines)] // per-step orchestration is large
pub fn run(
    scenario: &Scenario,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<RunOutcome, RunnerError> {
    let document = &scenario.document;
    require_supported_shape(document)?;
    let assembly = crate::assembly::synthesize_assembly(document)?;
    // Same effector-rack pattern as the point-mass
    // runner. Empty rack means no per-step effector operations.
    let mut effector_rack = crate::effectors::EffectorRack::build(document)?;
    // See point_mass.rs for the rationale.
    let mut engine_rack = crate::engines::EngineRack::build(document)?;
    // Tank rack mirroring the point-mass runner.
    let mut tank_rack = crate::tanks::TankRack::build(document)?;
    // Recovery rack mirroring the point-mass runner.
    let mut recovery_rack = crate::recovery::RecoveryRack::build(document)?;
    // Build the runner-side wind rack. Inactive when no
    // `[wind]` block is declared (or `kind = "none"`).
    let wind_rack = crate::wind::WindRack::build(document)?;
    wind_rack.reset();

    let loaded = load_models(document, resolved_files)?;
    let initial_state = build_initial_state(document, &loaded, &assembly)?;
    let kernel_vehicle = build_vehicle(document, &loaded, &assembly)?;
    let breakdown_vehicle = build_vehicle(document, &loaded, &assembly)?;
    let mass_model = build_mass_model(document, &loaded, &assembly)?;
    let moment_model = build_moment_model(document)?;
    let rigid_models = RigidModels::new(moment_model, mass_model);

    // Runner-side `[solver]` block dispatch on the
    // rigid-body path. Default (no `[solver]`) selects `Rk4FixedStep`,
    // preserving byte-stability for every existing rigid-body
    // scenario. Adaptive / fixed-DOPRI selections now drive
    // `Dopri54Adaptive` / `Dopri54FixedStep` end-to-end through the
    // rigid-body kernel — the §5.D.4 audit-follow-up reject gate that
    // refused non-RK4 selections has been removed.
    let runtime_integrator = build_runtime_integrator(document)?;

    let config = SimulationConfig {
        initial_state,
        integrator: runtime_integrator,
        force_model: kernel_vehicle,
        mass_model: rigid_models,
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(document.time.stop_s)),
        dt: Duration::from_seconds(document.time.dt_s),
        scenario_seed: document.time.seed,
    };

    let kernel_base = SimulationKernel::new_rigid(config)?;
    let mut kernel = if let Some(mission) = &document.mission {
        let mission_runtime = crate::mission::build_mission_runtime_typed(mission)?;
        kernel_base.with_mission_split(
            mission_runtime.mission_bindings,
            mission_runtime.script_bindings,
            Some(mission_runtime.graph),
            Some(mission_runtime.hsm),
        )?
    } else {
        kernel_base
    };
    let channel_set = RigidChannelSet::new(document)?;
    let breakdown_atmosphere = if channel_set.has_atmosphere {
        Some(build_runtime_atmosphere(scenario_atmosphere_kind(
            document,
        ))?)
    } else {
        None
    };
    let metadata = build_schema_metadata(document, resolved_files);
    let mut table = TelemetryTable::new(channel_set.schema(metadata)?);

    // See point_mass.rs sibling for the rationale.
    let deck_bindings = crate::aero_effector_match::assert_axes_match_effectors(
        loaded.aero_deck.as_ref(),
        document,
    )?;

    let initial_snapshot = effector_rack.snapshot();
    let direct_torque_present = document.vehicle.assembly.effectors.iter().any(|e| {
        matches!(
            e.kind,
            openbmp_scenario::EffectorKindConfig::DirectTorque { .. }
        )
    });
    if !deck_bindings.is_empty() || direct_torque_present {
        let mut snapshot_map = crate::aero_effector_match::build_snapshot_map(
            &deck_bindings,
            &initial_snapshot,
        );
        if direct_torque_present {
            let dt_map = crate::aero_effector_match::build_direct_torque_snapshot_map(
                document,
                &initial_snapshot,
            );
            merge_direct_torque_snapshot_map(&mut snapshot_map, dt_map)?;
        }
        kernel.set_effector_actuals(snapshot_map);
    }
    if !engine_rack.is_empty() {
        kernel.set_engine_snapshot(engine_rack.snapshot_map());
    }
    if !tank_rack.is_empty() {
        kernel.set_tank_snapshot(tank_rack.snapshot_map());
    }
    if !recovery_rack.is_empty() {
        kernel.set_recovery_snapshot(recovery_rack.snapshot_map());
    }
    if !wind_rack.is_inactive() {
        let s = kernel.current_state();
        let frame = openbmp_physics::FrameContext::toy_fixed_earth();
        let wind = wind_rack.sample(s.position, &frame, s.time)?;
        kernel.set_wind_sample(wind);
    }
    let mut fc_bridge = crate::fc_bridge::FcBridge::maybe_new(scenario, resolved_files)?;
    record_step(
        &mut table,
        &kernel,
        &channel_set,
        &breakdown_vehicle,
        breakdown_atmosphere.as_ref(),
        &[],
        &initial_snapshot,
    )?;
    let mut pending_effector_events: Vec<openbmp_sim::FiredEvent<ScenarioScriptAction>> =
        Vec::new();
    let mut pending_engine_events: Vec<openbmp_sim::FiredEvent<ScenarioScriptAction>> = Vec::new();
    let mut pending_recovery_events: Vec<openbmp_sim::FiredEvent<ScenarioScriptAction>> =
        Vec::new();
    while kernel.stop_reason().is_none() {
        effector_rack.apply_overrides(&pending_effector_events)?;
        if !engine_rack.is_empty() {
            engine_rack.apply_commands(&pending_engine_events)?;
        }
        if let Some(bridge) = &mut fc_bridge {
            let gravity = Vector3::new(
                0.0,
                0.0,
                -document
                    .environment
                    .gravity_m_s2
                    .unwrap_or(openbmp_physics::gravity::STANDARD_GRAVITY_M_S2),
            );
            bridge.tick_rigid_body(
                kernel.current_state(),
                kernel.current_step(),
                gravity,
                &mut effector_rack,
                &mut engine_rack,
            )?;
            // Forward the mission state published by
            // this FC tick into the kernel before the kernel evaluates
            // mission events for the next integrated state.
            if let Some(state_id) = bridge.latest_mission_state_id() {
                kernel.set_external_mission_state(Some(openbmp_sim::PhaseId::new(state_id)));
            }
        }
        if !effector_rack.is_empty() {
            effector_rack.step(kernel.current_time())?;
        }
        if !engine_rack.is_empty() {
            engine_rack.step()?;
        }
        // Advance tanks using prior-step cached drivers.
        // The drivers are updated post-step from the new rigid-body
        // state's angular_velocity (omega_body) and a finite-
        // difference body-frame acceleration; the first step uses
        // zeros (initialised by `TankRack::build`).
        if !tank_rack.is_empty() {
            tank_rack.step()?;
        }
        // Drain pending deploy/stow events and step the
        // recovery rack (no-op step for the instantaneous-
        // deploy models).
        if !recovery_rack.is_empty() {
            recovery_rack.apply_deploys(&pending_recovery_events)?;
            recovery_rack.step(document.time.dt_s)?;
        }
        if !deck_bindings.is_empty() || direct_torque_present {
            let rack_snapshot = effector_rack.snapshot();
            let mut snapshot_map = crate::aero_effector_match::build_snapshot_map(
                &deck_bindings,
                &rack_snapshot,
            );
            if direct_torque_present {
                let dt_map = crate::aero_effector_match::build_direct_torque_snapshot_map(
                    document,
                    &rack_snapshot,
                );
                merge_direct_torque_snapshot_map(&mut snapshot_map, dt_map)?;
            }
            kernel.set_effector_actuals(snapshot_map);
        }
        if !engine_rack.is_empty() {
            kernel.set_engine_snapshot(engine_rack.snapshot_map());
        }
        if !tank_rack.is_empty() {
            kernel.set_tank_snapshot(tank_rack.snapshot_map());
        }
        if !recovery_rack.is_empty() {
            kernel.set_recovery_snapshot(recovery_rack.snapshot_map());
        }
        if !wind_rack.is_inactive() {
            wind_rack.advance(kernel.current_step());
            let s = kernel.current_state();
            let frame = openbmp_physics::FrameContext::toy_fixed_earth();
            let wind = wind_rack.sample(s.position, &frame, s.time)?;
            kernel.set_wind_sample(wind);
        }
        let prev_velocity_eci = kernel.current_state().velocity.vector;
        let prev_orientation = kernel.current_state().orientation.q;
        kernel.step()?;
        // Refresh tank-rack drivers from the post-step
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
        let mission_fired = kernel.drain_mission_fired_events();
        let script_fired = kernel.drain_script_fired_events();
        let snapshot = effector_rack.snapshot();
        record_step(
            &mut table,
            &kernel,
            &channel_set,
            &breakdown_vehicle,
            breakdown_atmosphere.as_ref(),
            &mission_fired,
            &snapshot,
        )?;
        // Partition typed script-action fired queue.
        pending_engine_events = script_fired
            .iter()
            .filter(|e| matches!(e.action, ScenarioScriptAction::EngineCommand { .. }))
            .cloned()
            .collect();
        pending_recovery_events = script_fired
            .iter()
            .filter(|e| matches!(e.action, ScenarioScriptAction::DeployRecovery { .. }))
            .cloned()
            .collect();
        pending_effector_events = script_fired;
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

fn merge_direct_torque_snapshot_map(
    snapshot_map: &mut BTreeMap<String, f64>,
    direct_torque_map: BTreeMap<String, f64>,
) -> Result<(), RunnerError> {
    for (key, value) in direct_torque_map {
        if snapshot_map.insert(key.clone(), value).is_some() {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "effector snapshot key `{key}` is used by both an aero-deck axis and a \
                     direct_torque effector; rename the direct_torque effector or deck axis"
                ),
            });
        }
    }
    Ok(())
}

#[derive(Clone, Debug, Default)]
struct LoadedModels {
    aero_deck: Option<AeroDeck>,
    motor: Option<SolidMotor>,
}

fn require_supported_shape(document: &ScenarioDocument) -> Result<(), RunnerError> {
    if document.vehicle.kind != "rigid_body" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "vehicle.kind = {} (expected rigid_body)",
                document.vehicle.kind
            ),
        });
    }
    // `[solver]` block dispatch is wired end-to-end on
    // the rigid-body runner. The actual `RuntimeIntegrator`
    // construction lives in the kernel-config block in `run()` so the
    // adaptive integrator's persistent state (last_h, last_err_prev)
    // is owned by the kernel for the entire run. The §5.D.4
    // audit-follow-up reject gate that refused non-RK4 selections
    // has been removed.
    if !matches!(
        document.environment.gravity.as_str(),
        "constant" | "point_mass" | "j2" | "egm2008"
    ) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "environment.gravity = {} (wired: constant, point_mass, j2, egm2008)",
                document.environment.gravity
            ),
        });
    }
    for name in document.force_models() {
        if !matches!(name.as_str(), "gravity" | "aero" | "thrust") {
            return Err(RunnerError::UnsupportedScenario {
                what: format!("forces.models entry `{name}` (only gravity, aero, thrust wired)"),
            });
        }
    }
    // Wind models are resolved by WindRack. Scenario
    // validation guarantees that non-`none` flat selections carry a
    // structured `[wind]` block and that the kind names agree.
    let atmosphere_kind = scenario_atmosphere_kind(document);
    let has_aero = document.force_models().iter().any(|m| m == "aero");
    if has_aero && !is_runtime_atmosphere_kind(atmosphere_kind) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "atmosphere `{atmosphere_kind}` is not wired with the aero force; \
                 use `us_standard_1976` or `piecewise_exponential`"
            ),
        });
    }
    let has_recovery = !document.vehicle.assembly.recovery.is_empty();
    if has_recovery && !is_runtime_atmosphere_kind(atmosphere_kind) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "atmosphere `{atmosphere_kind}` is not wired with recovery drag; \
                 use `us_standard_1976` or `piecewise_exponential`"
            ),
        });
    }
    Ok(())
}

fn load_models(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<LoadedModels, RunnerError> {
    let aero_deck = if document.aero.is_some() {
        let resolved = required_resolved_file(resolved_files, "aero.deck")?;
        let text = std::str::from_utf8(&resolved.bytes).map_err(|e| {
            RunnerError::Aero(AeroError::Io {
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
            RunnerError::Motor(MotorError::Io {
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
) -> Result<&'a ResolvedFile, RunnerError> {
    resolved_files
        .get(field)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!(
                "internal invariant: resolved file `{field}` missing after pin verification"
            ),
        })
}

fn build_initial_state(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    assembly: &Assembly,
) -> Result<RigidBodyState, RunnerError> {
    let p = document.vehicle.initial_position_eci_m;
    let v = document.vehicle.initial_velocity_eci_m_s;
    let q = document
        .vehicle
        .initial_quaternion_body_to_eci_xyzw
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: initial_quaternion missing for rigid_body scenario"
                .to_owned(),
        })?;
    let omega = document
        .vehicle
        .initial_angular_velocity_body_rad_s
        .ok_or_else(|| RunnerError::UnsupportedScenario {
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

/// Construct the gravity-force adapter for the runtime gravity model
/// declared by the scenario.
///
/// The `egm2008` arm joins the `constant`,
/// `point_mass`, and `j2` arms, all of which keep the per-scenario parameter contracts
/// validated by `EnvironmentConfig::validate`. The returned trait object
/// is `Send` + `Sync` so the kernel can store it in its force list.
fn build_gravity_force_adapter_rigid_body(
    document: &ScenarioDocument,
) -> Result<Box<dyn ForceModel<RigidBodyState> + Send + Sync>, RunnerError> {
    match document.environment.gravity.as_str() {
        "constant" => {
            let g =
                document
                    .environment
                    .gravity_m_s2
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "environment.gravity_m_s2 missing for constant gravity".to_owned(),
                    })?;
            if g < 0.0 {
                return Err(RunnerError::UnsupportedScenario {
                    what: "environment.gravity_m_s2 must be a non-negative magnitude; \
                         constant gravity is -z in ECI"
                        .to_owned(),
                });
            }
            let model = ConstantGravity::down_z(g)?;
            Ok(Box::new(GravityForceAdapter::new(
                model,
                RIGID_BODY_GRAVITY_MODEL_ID,
            )))
        }
        "point_mass" => {
            let mu =
                document
                    .environment
                    .mu_m3_s2
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "environment.mu_m3_s2 missing for point_mass gravity".to_owned(),
                    })?;
            let model = PointMassGravity::new(mu)?;
            Ok(Box::new(GravityForceAdapter::new(
                model,
                RIGID_BODY_GRAVITY_MODEL_ID,
            )))
        }
        "j2" => {
            let mu =
                document
                    .environment
                    .mu_m3_s2
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "environment.mu_m3_s2 missing for j2 gravity".to_owned(),
                    })?;
            let r_e = document
                .environment
                .r_e_m
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "environment.r_e_m missing for j2 gravity".to_owned(),
                })?;
            let j2 = document.environment.j2.unwrap_or(WGS84_J2);
            let model = J2Gravity::new(mu, r_e, j2)?;
            Ok(Box::new(GravityForceAdapter::new(
                model,
                RIGID_BODY_GRAVITY_MODEL_ID,
            )))
        }
        "egm2008" => {
            // Zonal-only EGM2008 (degrees 2-6), pinned to
            // WGS84 µ / R_e and the Pavlis et al. 2012 J_n table. No
            // per-scenario overrides are accepted, matching the parser
            // contract in `EnvironmentConfig::validate`.
            let model = Egm2008ZonalGravity::wgs84_egm2008_zonal();
            Ok(Box::new(GravityForceAdapter::new(
                model,
                RIGID_BODY_GRAVITY_MODEL_ID,
            )))
        }
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("environment.gravity = {other} is not wired"),
        }),
    }
}

#[allow(clippy::too_many_lines)] // the recovery-rack force-adapter wiring branch is large
fn build_vehicle(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    assembly: &Assembly,
) -> Result<KernelVehicle<RigidBodyState>, RunnerError> {
    let mut named: Vec<NamedForceModel<RigidBodyState>> = Vec::new();
    for name in document.force_models() {
        match name.as_str() {
            "gravity" => {
                let force = build_gravity_force_adapter_rigid_body(document)?;
                named.push(NamedForceModel::new("gravity", force));
            }
            "aero" => {
                let deck =
                    loaded
                        .aero_deck
                        .clone()
                        .ok_or_else(|| RunnerError::UnsupportedScenario {
                            what: "forces includes `aero` but [aero] block is missing".to_owned(),
                        })?;
                let atmosphere = build_runtime_atmosphere(scenario_atmosphere_kind(document))?;
                let drag = DeckDragForceAdapter::new(deck, atmosphere, RIGID_BODY_AERO_MODEL_ID);
                named.push(NamedForceModel::new("aero", Box::new(drag)));
            }
            "thrust" => {
                // Dispatch between single-motor and
                // engine-cluster paths. AmbiguousPropulsion is
                // rejected at parse time.
                if document.vehicle.assembly.engines.is_empty() {
                    let motor =
                        loaded
                            .motor
                            .clone()
                            .ok_or_else(|| RunnerError::UnsupportedScenario {
                                what:
                                    "forces includes `thrust` but neither [propulsion.motor] nor \
                                   [[vehicle.assembly.engines]] is declared"
                                        .to_owned(),
                            })?;
                    let ignition_time_s = motor_ignition_time_s(document)?;
                    let thrust = MotorThrustForceAdapter::new(
                        motor,
                        ignition_time_s,
                        RIGID_BODY_THRUST_MODEL_ID,
                    );
                    named.push(NamedForceModel::new("thrust", Box::new(thrust)));
                } else {
                    let engine_ids: Vec<openbmp_core::EngineId> = document
                        .vehicle
                        .assembly
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
                        RIGID_BODY_ENGINE_CLUSTER_THRUST_MODEL_ID,
                    );
                    named.push(NamedForceModel::new("thrust", Box::new(thrust)));
                }
            }
            other => unreachable!("require_supported_shape rejects unknown force model `{other}`"),
        }
    }

    // Tank-rack reaction-force adapter (rigid).
    if !document.vehicle.assembly.tanks.is_empty() {
        let tank_ids: Vec<openbmp_core::TankId> = document
            .vehicle
            .assembly
            .tanks
            .iter()
            .map(|t| {
                openbmp_core::TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = t.id))
            })
            .collect();
        let tank_force =
            openbmp_vehicle::TankRackForceAdapter::new(tank_ids, RIGID_BODY_TANK_RACK_FORCE_MODEL_ID);
        named.push(NamedForceModel::new("tank_reaction", Box::new(tank_force)));
    }

    // Recovery-rack drag-force adapter (rigid).
    if !document.vehicle.assembly.recovery.is_empty() {
        let recovery_ids: Vec<openbmp_core::RecoveryId> = document
            .vehicle
            .assembly
            .recovery
            .iter()
            .map(|r| {
                openbmp_core::RecoveryId::from_path(&format!(
                    "vehicle.assembly.recovery.{id}",
                    id = r.id
                ))
            })
            .collect();
        let atmosphere = build_runtime_atmosphere(scenario_atmosphere_kind(document))?;
        let recovery_force = openbmp_vehicle::RecoveryRackForceAdapter::new(
            recovery_ids,
            atmosphere,
            RIGID_BODY_RECOVERY_RACK_FORCE_MODEL_ID,
        );
        named.push(NamedForceModel::new(
            "recovery_drag",
            Box::new(recovery_force),
        ));
    }

    // KernelVehicle requires a mass model; the kernel keeps a separate
    // copy through `RigidMotorMassAdapter` / `ConstantMassRigid` for
    // its own state propagation. We give the vehicle a scalar
    // `BoxedMassModel` view so the breakdown evaluator can query mass
    // when it needs to.
    let vehicle_mass = build_vehicle_scalar_mass_model(document, loaded, assembly)?;
    KernelVehicle::new(named, vec![], Box::new(vehicle_mass)).map_err(|e| {
        RunnerError::UnsupportedScenario {
            what: format!("KernelVehicle construction failed: {e}"),
        }
    })
}

fn build_vehicle_scalar_mass_model(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    assembly: &Assembly,
) -> Result<BoxedMassModel, RunnerError> {
    use openbmp_sim::{ConstantMass, MassModel};
    use openbmp_vehicle::MotorMassAdapter;

    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_mass_kg = dry_mass_kg_at(assembly, start_time, "vehicle.assembly")?;
    let inner: Box<dyn MassModel> = if !document.vehicle.assembly.engines.is_empty() {
        // Cluster path: engine-cluster mass adapter
        // tracks per-engine `consumed_kg` from the kernel snapshot.
        let engine_ids: Vec<openbmp_core::EngineId> = document
            .vehicle
            .assembly
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
            RIGID_BODY_ENGINE_CLUSTER_MASS_MODEL_ID,
        ))
    } else if let Some(motor) = &loaded.motor {
        Box::new(MotorMassAdapter::new(
            motor.clone(),
            dry_mass_kg,
            motor_ignition_time_s(document)?,
            RIGID_BODY_MOTOR_MASS_MODEL_ID,
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
    /// Direct-torque effectors only (no engine cluster,
    /// no tanks). The closed-loop FC validation scenario for the
    /// differential-flatness tracker uses this path.
    DirectTorque(openbmp_vehicle::DirectTorqueMomentAdapter),
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
            Self::DirectTorque(d) => {
                <openbmp_vehicle::DirectTorqueMomentAdapter as openbmp_sim::MomentModel<
                    RigidBodyState,
                >>::moment_n_m_body(d, ctx)
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
            Self::DirectTorque(d) => {
                <openbmp_vehicle::DirectTorqueMomentAdapter as openbmp_sim::MomentModel<
                    RigidBodyState,
                >>::validation(d)
            }
        }
    }
}

#[allow(clippy::if_not_else)]
fn build_moment_model(document: &ScenarioDocument) -> Result<RigidMomentEither, RunnerError> {
    let assembly = &document.vehicle.assembly;
    let cluster_adapter = if !assembly.engines.is_empty() {
        let engine_ids: Vec<openbmp_core::EngineId> = assembly
            .engines
            .iter()
            .map(|e| {
                openbmp_core::EngineId::from_path(&format!(
                    "vehicle.assembly.engines.{id}",
                    id = e.id
                ))
            })
            .collect();
        let mount_points_body: Vec<Position3<Body>> = assembly
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
            RIGID_BODY_ENGINE_CLUSTER_MOMENT_MODEL_ID,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("EngineClusterMomentAdapter construction failed: {err}"),
        })?;
        Some(adapter)
    } else {
        None
    };

    let tank_adapter = if !assembly.tanks.is_empty() {
        let tank_ids: Vec<openbmp_core::TankId> = assembly
            .tanks
            .iter()
            .map(|t| {
                openbmp_core::TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = t.id))
            })
            .collect();
        Some(openbmp_vehicle::TankRackMomentAdapter::new(
            tank_ids,
            RIGID_BODY_TANK_RACK_MOMENT_MODEL_ID,
        ))
    } else {
        None
    };

    let direct_torque_adapter = build_direct_torque_adapter(document);

    // Combinations of direct-torque with engine-cluster
    // or tank-rack moment models are not supported.
    // Closed-loop FC validation scenarios use direct-torque alone; if
    // a downstream scenario combines them, fail closed.
    if direct_torque_adapter.is_some() && (cluster_adapter.is_some() || tank_adapter.is_some()) {
        return Err(RunnerError::UnsupportedScenario {
            what: "direct_torque effectors combined with engine-cluster or tank moment models \
                   is not supported; use a dedicated closed-loop validation \
                   scenario without engines/tanks"
                .to_string(),
        });
    }

    Ok(
        match (cluster_adapter, tank_adapter, direct_torque_adapter) {
            (Some(c), Some(t), None) => RigidMomentEitherKind::EngineClusterAndTankRack(c, t),
            (Some(c), None, None) => RigidMomentEitherKind::EngineCluster(c),
            (None, Some(t), None) => RigidMomentEitherKind::TankRack(t),
            (None, None, Some(d)) => RigidMomentEitherKind::DirectTorque(d),
            (None, None, None) => RigidMomentEitherKind::Zero(ZeroMoment),
            // Combinations with DirectTorque rejected above.
            _ => unreachable!(),
        },
    )
}

fn build_direct_torque_adapter(
    document: &ScenarioDocument,
) -> Option<openbmp_vehicle::DirectTorqueMomentAdapter> {
    let mut bindings = Vec::new();
    for effector in &document.vehicle.assembly.effectors {
        if let openbmp_scenario::EffectorKindConfig::DirectTorque {
            axis,
            effectiveness_n_m_per_rad,
        } = effector.kind
        {
            bindings.push(openbmp_vehicle::DirectTorqueBinding {
                snapshot_key: effector.id.clone(),
                body_axis_index: axis.body_axis_index(),
                effectiveness_n_m_per_rad,
            });
        }
    }
    if bindings.is_empty() {
        None
    } else {
        Some(openbmp_vehicle::DirectTorqueMomentAdapter::new(
            bindings,
            RIGID_BODY_DIRECT_TORQUE_MOMENT_MODEL_ID,
        ))
    }
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
    assembly: &Assembly,
) -> Result<RigidMassEither, RunnerError> {
    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_props = dry_mass_properties_at(assembly, start_time, "vehicle.assembly")?;

    // Rigid + engine cluster: propellant deficit isn't
    // tracked in rigid mass-properties (that requires
    // tank-driven mass-property dynamics). Fall through to
    // `ConstantMassRigid` — the cluster's `EngineClusterForceAdapter`
    // still applies thrust normally; only mass-properties is
    // simplified.
    if !document.vehicle.assembly.engines.is_empty() {
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
            RIGID_BODY_MOTOR_MASS_MODEL_ID,
        )))
    } else {
        Ok(RigidMassEitherKind::Constant(ConstantMassRigid::new(
            dry_props,
        )))
    }
}

fn motor_ignition_time_s(document: &ScenarioDocument) -> Result<f64, RunnerError> {
    let motor = document
        .propulsion
        .as_ref()
        .and_then(|p| p.motor.as_ref())
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "[propulsion.motor] block missing".to_owned(),
        })?;
    Ok(document.time.start_s + motor.ignite_at_s)
}

fn motor_elapsed_at_start_s(document: &ScenarioDocument) -> Result<f64, RunnerError> {
    Ok(document.time.start_s - motor_ignition_time_s(document)?)
}

fn build_schema_metadata(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> BTreeMap<String, String> {
    let mut metadata = BTreeMap::new();
    for (field, file) in resolved_files {
        metadata.insert(
            format!("openbmp.scenario_files.{field}"),
            file.sha256_hex.clone(),
        );
    }
    super::append_solver_metadata(document, &mut metadata);
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

/// Per-recovery telemetry channels in scenario-declared order.
/// Each entry is `(id, deployed, phase_index, drag_area)`.
type RecoveryTelemetryChannels = Vec<(
    RecoveryId,
    TelemetryChannel<bool>,
    TelemetryChannel<i64>,
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
    /// Effector deflection channels, in scenario-declared
    /// order. One `effector.<id>.actual` `f64` channel per declared
    /// effector. Allocated AFTER force breakdown channels and BEFORE
    /// mission markers — same ordering contract as the point-mass
    /// runner.
    effector_actuals: Vec<TelemetryChannel<f64>>,
    /// Recovery-state channels, in scenario-declared order.
    /// Allocated after effectors and before mission markers.
    recovery_states: RecoveryTelemetryChannels,
    /// Mission-event telemetry markers, keyed by tag.
    mission_markers: BTreeMap<String, TelemetryChannel<bool>>,
}

impl RigidChannelSet {
    #[allow(clippy::too_many_lines)]
    fn new(document: &ScenarioDocument) -> Result<Self, RunnerError> {
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

        let atmosphere_kind = scenario_atmosphere_kind(document);
        let has_atmosphere = is_runtime_atmosphere_kind(atmosphere_kind);
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

        let mut force_components = Vec::with_capacity(document.force_models().len());
        for name in document.force_models() {
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

        // Effector deflection channels, in scenario-declared
        // order. Allocated BEFORE mission markers so adding effectors
        // does not shift marker channel ids.
        let mut effector_actuals: Vec<TelemetryChannel<f64>> = Vec::new();
        for config in &document.vehicle.assembly.effectors {
            let channel = TelemetryChannel::<f64>::new(
                alloc(),
                format!("effector.{}.actual", config.id),
                config.unit.as_deref().unwrap_or("1"),
                None::<&str>,
            )?;
            effector_actuals.push(channel);
        }

        // Recovery telemetry channels, one triple per
        // declared device. Scenario-declared order matches the
        // recovery force-adapter operand order.
        let mut recovery_states: RecoveryTelemetryChannels = Vec::new();
        for config in &document.vehicle.assembly.recovery {
            let id =
                RecoveryId::from_path(&format!("vehicle.assembly.recovery.{id}", id = config.id));
            let deployed = TelemetryChannel::<bool>::new(
                alloc(),
                format!("recovery.{}.deployed", config.id),
                "bool",
                None::<&str>,
            )?;
            let phase_index = TelemetryChannel::<i64>::new(
                alloc(),
                format!("recovery.{}.phase_index", config.id),
                "1",
                None::<&str>,
            )?;
            let drag_area = TelemetryChannel::<f64>::new(
                alloc(),
                format!("recovery.{}.drag_area_m2", config.id),
                "m^2",
                None::<&str>,
            )?;
            recovery_states.push((id, deployed, phase_index, drag_area));
        }

        // Mission marker channels.
        let mut mission_markers: BTreeMap<String, TelemetryChannel<bool>> = BTreeMap::new();
        if let Some(mission) = &document.mission {
            for tag in crate::mission::marker_tags(mission) {
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
            recovery_states,
            mission_markers,
        })
    }

    fn schema(&self, metadata: BTreeMap<String, String>) -> Result<TelemetrySchema, RunnerError> {
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
        // Effector deflection channels, in scenario-declared
        // order, between force breakdown and mission markers.
        for actual in &self.effector_actuals {
            channels.push(actual.metadata().clone());
        }
        // Recovery channels, in scenario-declared order,
        // between effectors and mission markers.
        for (_, deployed, phase_index, drag_area) in &self.recovery_states {
            channels.push(deployed.metadata().clone());
            channels.push(phase_index.metadata().clone());
            channels.push(drag_area.metadata().clone());
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
    breakdown_vehicle: &KernelVehicle<RigidBodyState>,
    breakdown_atmosphere: Option<&RuntimeAtmosphere>,
    fired_events: &[openbmp_sim::FiredEvent<openbmp_sim::MissionAction>],
    effector_snapshot: &[openbmp_vehicle::EffectorState],
) -> Result<(), RunnerError>
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

    // See point_mass.rs sibling for the
    // breakdown / kernel snapshot symmetry rationale.
    let env_sample = kernel.current_environment_sample()?;
    let kernel_actuals = kernel.effector_actuals();
    let kernel_engine_snapshot = kernel.engine_snapshot();
    let kernel_tank_snapshot = kernel.tank_snapshot();
    let kernel_recovery_snapshot = kernel.recovery_snapshot();
    let ctx = ForceContext {
        state,
        environment: &env_sample,
        mass_kg: state.mass_props.mass_kg(),
        time: state.time,
        effector_actuals: openbmp_sim::EffectorActualsView::new(kernel_actuals),
        engine_snapshot: openbmp_sim::EngineSnapshotView::new(kernel_engine_snapshot),
        tank_snapshot: openbmp_sim::TankSnapshotView::new(kernel_tank_snapshot),
        recovery_snapshot: openbmp_sim::RecoverySnapshotView::new(kernel_recovery_snapshot),
    };
    let breakdown = breakdown_vehicle
        .evaluate_force_breakdown(ctx)
        .map_err(|e| RunnerError::UnsupportedScenario {
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

    // Effector deflection channels, in scenario-declared
    // order, matching `channels.effector_actuals`.
    debug_assert_eq!(effector_snapshot.len(), channels.effector_actuals.len());
    for (channel, state) in channels
        .effector_actuals
        .iter()
        .zip(effector_snapshot.iter())
    {
        row.insert(channel, state.actual)?;
    }

    insert_recovery_state_channels(
        &mut row,
        kernel_recovery_snapshot,
        &channels.recovery_states,
    )?;

    // Marker channels.
    let mut fired_tags: std::collections::BTreeSet<&str> = std::collections::BTreeSet::new();
    for fired in fired_events {
        if let openbmp_sim::MissionAction::EmitTelemetryMarker { tag } = &fired.action {
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

fn insert_recovery_state_channels(
    row: &mut TelemetryRow,
    snapshot: &BTreeMap<RecoveryId, openbmp_sim::RecoverySnapshot>,
    channels: &RecoveryTelemetryChannels,
) -> Result<(), RunnerError> {
    for (id, deployed, phase_index, drag_area) in channels {
        let state = snapshot
            .get(id)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "recovery telemetry snapshot missing declared recovery id {}",
                    id.value()
                ),
            })?;
        row.insert(deployed, state.deployed)?;
        row.insert(phase_index, i64::from(state.phase_index))?;
        row.insert(drag_area, state.drag_area_m2)?;
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn direct_torque_snapshot_key_collision_fails_closed() {
        let mut aero_map = BTreeMap::from([("roll-torque".to_string(), 0.1)]);
        let direct_torque_map = BTreeMap::from([("roll-torque".to_string(), 0.2)]);

        let err = merge_direct_torque_snapshot_map(&mut aero_map, direct_torque_map).unwrap_err();
        match err {
            RunnerError::UnsupportedScenario { what } => {
                assert!(what.contains("roll-torque"));
                assert!(what.contains("aero-deck axis"));
                assert!(what.contains("direct_torque effector"));
            }
            other => panic!("expected UnsupportedScenario, got {other:?}"),
        }
    }
}
