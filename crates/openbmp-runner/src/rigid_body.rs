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
//!   "thrust", "aerothermal_diagnostics"]`
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

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};

use nalgebra::Vector3;
use openbmp_aero::AeroDeck;
use openbmp_core::{
    AngularVelocity3, Body, BodyId, ChannelId, Duration, EngineId, ModelId, Position3, Quaternion,
    RecoveryId, SimTime, TankId, ValidationStatus, Velocity3,
};
use openbmp_physics::{
    AtmosphereModel, ConstantGravity, Egm2008ZonalGravity, J2Gravity, PointMassGravity, WGS84_J2,
};
use openbmp_propulsion::{Motor, SolidMotor};
use openbmp_scenario::{ResolvedFile, Scenario, ScenarioDocument};
use openbmp_sim::{
    AnyStop, ConstantMass, EndTime, ForceContext, ForceModel, GroundImpact, InitialRigidBodyLane,
    RigidBodySeparation, RigidMassModel, RigidModels, ScenarioScriptAction, SimulationConfig,
    SimulationKernel, StopReason,
};
use openbmp_state::{MassProperties, RigidBodyState};
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use openbmp_vehicle::{
    AeroMethodForceAdapter, AeroMethodMomentAdapter, Assembly, BoxedMassModel,
    DeckDragForceAdapter, EngineClusterForceAdapter, EngineClusterMassAdapter,
    EngineClusterMomentAdapter, GravityForceAdapter, KernelVehicle, MotorThrustForceAdapter,
    NamedForceModel, NamedMomentModel, Vehicle, VehicleAssembly,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::RunOutcome;
use crate::assembly::{dry_mass_kg_at, dry_mass_properties_at};
use crate::atmosphere::{
    RuntimeAtmosphere, RuntimeEnvironment, build_document_runtime_atmosphere,
    is_runtime_atmosphere_kind, scenario_atmosphere_kind,
};
use crate::error::RunnerError;
use crate::integrator::build_runtime_integrator;

// Stable model-ids assigned to each force / mass model the rigid
// runner wires. Reserves a separate range from the point-mass
// runner so determinism tooling can distinguish the two
// paths.
const RIGID_BODY_GRAVITY_MODEL_ID: ModelId = ModelId::new(301);
const RIGID_BODY_AERO_MODEL_ID: ModelId = ModelId::new(302);
const RIGID_BODY_THRUST_MODEL_ID: ModelId = ModelId::new(303);
const RIGID_BODY_MOTOR_MASS_MODEL_ID: ModelId = ModelId::new(304);
const RIGID_BODY_AEROTHERMAL_MODEL_ID: ModelId = ModelId::new(305);
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
const MISSING_AEROTHERMAL_DIAGNOSTICS_MESSAGE: &str =
    "forces includes `aerothermal_diagnostics` but [aerothermal] block is missing";

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
    let propellant_budget = crate::propulsion::build_propellant_budget(document)?;
    // Recovery rack mirroring the point-mass runner.
    let mut recovery_rack = crate::recovery::RecoveryRack::build(document)?;
    // Build the runner-side wind rack. Inactive when no
    // `[wind]` block is declared (or `kind = "none"`).
    let wind_rack = crate::wind::WindRack::build(document)?;
    wind_rack.reset();
    // Structural bending-mode rack. Inactive when no `[vehicle.bending]`.
    let mut structural_rack = crate::structural::StructuralRack::build(document)?;
    structural_rack.reset();
    let frame = crate::frames::build_frame_context(document, resolved_files)?;

    let loaded = load_models(document, resolved_files)?;
    crate::aero::reject_hypersonic_deck_only_out_of_envelope(document, loaded.aero_deck.as_ref())?;
    let mut aerothermal_driver = crate::aerothermal::LiveAerothermalDriver::maybe_new(document)?;
    let aerothermal_sink = aerothermal_driver
        .as_ref()
        .map(crate::aerothermal::LiveAerothermalDriver::sink);
    let aerothermal_feedback = aerothermal_driver
        .as_ref()
        .and_then(crate::aerothermal::LiveAerothermalDriver::mass_feedback);
    let mass_resources = RigidMassResources::new(document, &assembly)?;
    let initial_engine_snapshot = if engine_rack.is_empty() {
        BTreeMap::new()
    } else {
        engine_rack.snapshot_map()
    };
    let initial_tank_snapshot = if tank_rack.is_empty() {
        BTreeMap::new()
    } else {
        tank_rack.snapshot_map()
    };
    let initial_state = build_initial_state(
        document,
        &loaded,
        &mass_resources,
        &initial_engine_snapshot,
        &initial_tank_snapshot,
    )?;
    let kernel_vehicle = build_vehicle(
        document,
        &loaded,
        &assembly,
        resolved_files,
        aerothermal_sink.clone(),
    )?;
    let breakdown_vehicle = build_vehicle(
        document,
        &loaded,
        &assembly,
        resolved_files,
        aerothermal_sink,
    )?;
    let mass_model = build_mass_model(
        &loaded,
        &mass_resources,
        &initial_engine_snapshot,
        &initial_tank_snapshot,
        aerothermal_feedback,
    );
    let moment_model = build_moment_model(document, &loaded)?;
    let rigid_models = RigidModels::new(moment_model, mass_model.clone());
    let separation_specs = build_rigid_body_separations(document, &mass_resources)?;
    // Bodies currently attached to the primary continuing stack. Starts as
    // every configured body (minus any seeded as independent initial lanes)
    // and shrinks as jettison events fire. Used so a multi-body continuing
    // stack (e.g. an upper stage still carrying a fairing + payload)
    // conserves mass at each separation.
    let mut stack_bodies: BTreeSet<BodyId> = mass_resources.dry_bodies.keys().copied().collect();
    if let Some(multi_body) = document.multi_body.as_ref() {
        for lane in &multi_body.initial_lanes {
            stack_bodies.remove(&body_id_from_scenario_text(&lane.body_id));
        }
    }

    // Runner-side `[solver]` block dispatch on the
    // rigid-body path. Default (no `[solver]`) selects `Rk4FixedStep`,
    // preserving byte-stability for every existing rigid-body
    // scenario. Adaptive / fixed-DOPRI selections now drive
    // `Dopri54Adaptive` / `Dopri54FixedStep` end-to-end through the
    // rigid-body kernel; the old reject gate that refused non-RK4
    // selections has been removed.
    let runtime_integrator = build_runtime_integrator(document)?;

    let config = SimulationConfig {
        initial_state,
        integrator: runtime_integrator,
        force_model: kernel_vehicle,
        mass_model: rigid_models,
        environment: RuntimeEnvironment::from_document(document, &frame)?,
        stop_condition: AnyStop::new(
            automatic_ground_impact(document),
            EndTime::new(SimTime::from_seconds(document.time.stop_s)),
        ),
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
    seed_initial_rigid_body_lanes(
        &mut kernel,
        document,
        &loaded,
        &mass_resources,
        &initial_engine_snapshot,
        &initial_tank_snapshot,
    )?;
    let channel_set = RigidChannelSet::new(document)?;
    let breakdown_atmosphere = if channel_set.has_atmosphere {
        Some(build_document_runtime_atmosphere(document)?)
    } else {
        None
    };
    let metadata = build_schema_metadata(document, resolved_files)?;
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
        let mut snapshot_map =
            crate::aero_effector_match::build_snapshot_map(&deck_bindings, &initial_snapshot);
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
        let wind = wind_rack.sample(s.position, &frame, s.time)?;
        kernel.set_wind_sample(wind);
    }
    if let Some(driver) = &mut aerothermal_driver {
        let environment = kernel.current_environment_sample()?;
        driver.evaluate_rigid_body(kernel.current_state(), &environment, 0.0)?;
    }
    let mut fc_bridge = crate::fc_bridge::FcBridge::maybe_new(scenario, resolved_files)?;
    record_step(
        document,
        &mut table,
        &kernel,
        &channel_set,
        &breakdown_vehicle,
        breakdown_atmosphere.as_ref(),
        aerothermal_driver.as_ref().map(|driver| driver.output()),
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
                // Bending slope-rate pickup from the previous tick's modal
                // state — contemporaneous with the body rate read above
                // (one-step lag, mirroring the slosh rack). Zero when rigid.
                structural_rack.gyro_pickup_rad_s(),
            )?;
            // Forward the mission state published by
            // this FC tick into the kernel before the kernel evaluates
            // mission events for the next integrated state.
            if let Some(state_id) = bridge.latest_mission_state_id() {
                kernel.set_external_mission_state(Some(openbmp_sim::PhaseId::new(state_id)));
            }
        }
        if let Some(propellant_budget) = &propellant_budget {
            let report = propellant_budget
                .evaluate(
                    &engine_rack.propulsion_snapshot_map(),
                    &tank_rack.propellant_tank_states(document),
                )
                .map_err(|err| RunnerError::Engine {
                    field: "vehicle.assembly.engines[*].propellant".to_owned(),
                    reason: err.to_string(),
                })?;
            tank_rack.set_propellant_budget_drain_rates(report.tank_drain_rates_kg_per_s.clone());
            engine_rack.apply_propellant_budget(&report)?;
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
        // Advance the bending mode against the prior-step body lateral accel
        // (one-step lag, like the slosh rack); refreshes the gyro pickup the
        // next bridge tick reads.
        if !structural_rack.is_inactive() {
            structural_rack.step()?;
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
            let mut snapshot_map =
                crate::aero_effector_match::build_snapshot_map(&deck_bindings, &rack_snapshot);
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
            let wind = wind_rack.sample(s.position, &frame, s.time)?;
            kernel.set_wind_sample(wind);
        }
        // Feed the bending mode's reaction moment to the rigid-body torque for
        // this step (held across the RK4 stages). Zero when no flex mode.
        if !structural_rack.is_inactive() {
            kernel.set_bending_reaction_moment(structural_rack.reaction_moment_body_n_m());
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
        if !tank_rack.is_empty() || !structural_rack.is_inactive() {
            let dt_s = document.time.dt_s;
            let new_state = kernel.current_state();
            let dv_eci = new_state.velocity.vector - prev_velocity_eci;
            let accel_eci = if dt_s > 0.0 {
                dv_eci / dt_s
            } else {
                nalgebra::Vector3::zeros()
            };
            // Rotate ECI accel into prior-step body frame: the slosh and
            // bending dynamics react to body-frame accel, and the prior body
            // frame matches their state's reference.
            let inverse_orientation = prev_orientation.inverse();
            let accel_body = inverse_orientation * accel_eci;
            let omega_body = new_state.angular_velocity.vector;
            if !tank_rack.is_empty() {
                tank_rack.update_drivers(accel_body, omega_body);
            }
            // The bending mode is forced by the body lateral specific force.
            structural_rack.update_drivers(accel_body);
        }
        let mission_fired = kernel.drain_mission_fired_events();
        let script_fired = kernel.drain_script_fired_events();
        apply_jettison_events(
            &mut kernel,
            &script_fired,
            &separation_specs,
            &mass_model,
            &mut stack_bodies,
        )?;
        if let Some(driver) = &mut aerothermal_driver {
            let environment = kernel.current_environment_sample()?;
            driver.evaluate_rigid_body(kernel.current_state(), &environment, document.time.dt_s)?;
        }
        let snapshot = effector_rack.snapshot();
        record_step(
            document,
            &mut table,
            &kernel,
            &channel_set,
            &breakdown_vehicle,
            breakdown_atmosphere.as_ref(),
            aerothermal_driver.as_ref().map(|driver| driver.output()),
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
        pending_effector_events = script_fired
            .iter()
            .filter(|e| matches!(e.action, ScenarioScriptAction::EffectorOverride { .. }))
            .cloned()
            .collect();
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

fn automatic_ground_impact(document: &ScenarioDocument) -> GroundImpact {
    if document.environment.gravity == "constant" {
        GroundImpact::sea_level()
    } else {
        GroundImpact::disabled()
    }
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
    // is owned by the kernel for the entire run. The old reject gate
    // that refused non-RK4 selections has been removed.
    if !matches!(
        document.environment.gravity.as_str(),
        "constant" | "point_mass" | "j2" | "egm2008" | "third_body"
    ) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "environment.gravity = {} (wired: constant, point_mass, j2, egm2008, third_body)",
                document.environment.gravity
            ),
        });
    }
    for name in document.force_model_universe() {
        if !matches!(
            name.as_str(),
            "gravity" | "aero" | "thrust" | "aerothermal_diagnostics"
        ) {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "forces.models entry `{name}` (only gravity, aero, thrust, \
                     aerothermal_diagnostics wired)"
                ),
            });
        }
    }
    require_supported_multi_body_shape(document)?;
    // Wind models are resolved by WindRack. Scenario
    // validation guarantees that non-`none` flat selections carry a
    // structured `[wind]` block and that the kind names agree.
    let atmosphere_kind = scenario_atmosphere_kind(document);
    let has_aero = document.force_model_universe().iter().any(|m| m == "aero");
    let has_aerothermal = document
        .force_model_universe()
        .iter()
        .any(|m| m == "aerothermal_diagnostics");
    if crate::mission::uses_dynamic_pressure_trigger(document)
        && !is_runtime_atmosphere_kind(atmosphere_kind)
    {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "atmosphere `{atmosphere_kind}` is not wired with dynamic-pressure triggers; \
                 use `us_standard_1976`, `piecewise_exponential`, `nrlmsise00`, or \
                 `nrlmsis2_compat`"
            ),
        });
    }
    if has_aero && !is_runtime_atmosphere_kind(atmosphere_kind) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "atmosphere `{atmosphere_kind}` is not wired with the aero force; \
                 use `us_standard_1976` or `piecewise_exponential`"
            ),
        });
    }
    if has_aerothermal && document.aerothermal.is_none() {
        return Err(RunnerError::UnsupportedScenario {
            what: MISSING_AEROTHERMAL_DIAGNOSTICS_MESSAGE.to_owned(),
        });
    }
    if has_aerothermal && !is_runtime_atmosphere_kind(atmosphere_kind) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "atmosphere `{atmosphere_kind}` is not wired with aerothermal diagnostics; \
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

fn require_supported_multi_body_shape(document: &ScenarioDocument) -> Result<(), RunnerError> {
    if document.multi_body.is_none() {
        return Ok(());
    }
    if let Some(solver) = &document.solver {
        let profile = solver.profile.as_deref().unwrap_or("fixed-step-explicit");
        let method = solver.trajectory_method.as_deref().unwrap_or("rk4");
        if profile != "fixed-step-explicit" || method != "rk4" {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "[multi_body] rigid-body lanes currently require the fixed-step RK4 \
                     trajectory solver; got profile={profile:?}, method={method:?}"
                ),
            });
        }
    }
    Ok(())
}

#[derive(Clone, Debug)]
struct TankMassRoute {
    owner: BodyId,
    mount_point_body_m: Vector3<f64>,
}

#[derive(Clone, Debug)]
struct RigidMassResources {
    dry_total: MassProperties,
    dry_bodies: BTreeMap<BodyId, MassProperties>,
    motor_owner: Option<BodyId>,
    motor_ignition_time_s: Option<f64>,
    engine_ids: Vec<EngineId>,
    engine_owners: BTreeMap<EngineId, BodyId>,
    /// Engines whose propellant is owned by a tank (they declare a
    /// `[propellant]` binding). In the ABSOLUTE mass path the tank's
    /// `mass_kg` already reflects depletion, so the engine cluster must
    /// NOT also subtract its integrated `consumed_kg` there — doing so
    /// double-counts the burned propellant and can drive a separated
    /// stage to negative mass. (The mass-RATE path still uses
    /// `-mass_flow`; it carries no separate tank term.)
    tank_coupled_engines: std::collections::BTreeSet<EngineId>,
    tank_routes: BTreeMap<TankId, TankMassRoute>,
}

impl RigidMassResources {
    fn new(document: &ScenarioDocument, assembly: &Assembly) -> Result<Self, RunnerError> {
        let start_time = SimTime::from_seconds(document.time.start_s);
        let dry_total = dry_mass_properties_at(assembly, start_time, "vehicle.assembly")?;
        let dry_bodies: BTreeMap<BodyId, MassProperties> = assembly
            .bodies()
            .iter()
            .map(|body| {
                (
                    body.id(),
                    MassProperties::new(
                        Mass::new::<kilogram>(body.dry_mass_kg()),
                        *body.dry_cg_body(),
                        *body.dry_inertia_body(),
                    ),
                )
            })
            .collect();
        let motor_owner = document
            .propulsion
            .as_ref()
            .and_then(|p| p.motor.as_ref())
            .and_then(|m| m.mounted_to.as_deref())
            .map(body_id_from_scenario_text);
        let motor_ignition_time_s = if document
            .propulsion
            .as_ref()
            .and_then(|p| p.motor.as_ref())
            .is_some()
        {
            Some(motor_ignition_time_s(document)?)
        } else {
            None
        };
        let engine_ids = document
            .vehicle
            .assembly
            .engines
            .iter()
            .map(|engine| {
                EngineId::from_path(&format!("vehicle.assembly.engines.{id}", id = engine.id))
            })
            .collect();
        let engine_owners = engine_owner_map(document)?;
        let tank_coupled_engines = document
            .vehicle
            .assembly
            .engines
            .iter()
            .filter(|engine| engine.propellant.is_some())
            .map(|engine| {
                EngineId::from_path(&format!("vehicle.assembly.engines.{id}", id = engine.id))
            })
            .collect();
        let tank_routes = document
            .vehicle
            .assembly
            .tanks
            .iter()
            .map(|tank| {
                let id = TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = tank.id));
                (
                    id,
                    TankMassRoute {
                        owner: body_id_from_scenario_text(&tank.mounted_to),
                        mount_point_body_m: Vector3::new(
                            tank.mount_point_body_m[0],
                            tank.mount_point_body_m[1],
                            tank.mount_point_body_m[2],
                        ),
                    },
                )
            })
            .collect();
        Ok(Self {
            dry_total,
            dry_bodies,
            motor_owner,
            motor_ignition_time_s,
            engine_ids,
            engine_owners,
            tank_coupled_engines,
            tank_routes,
        })
    }
}

#[derive(Clone, Debug)]
struct RigidMassResourceModel<M> {
    resources: RigidMassResources,
    motor: Option<M>,
    aerothermal_feedback: Option<crate::aerothermal::AerothermalMassFeedback>,
    default_engine_snapshot: BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    default_tank_snapshot: BTreeMap<TankId, openbmp_sim::TankSnapshot>,
    model_id: ModelId,
}

impl<M> RigidMassResourceModel<M> {
    fn new(
        resources: RigidMassResources,
        motor: Option<M>,
        aerothermal_feedback: Option<crate::aerothermal::AerothermalMassFeedback>,
        model_id: ModelId,
    ) -> Self {
        Self {
            resources,
            motor,
            aerothermal_feedback,
            default_engine_snapshot: BTreeMap::new(),
            default_tank_snapshot: BTreeMap::new(),
            model_id,
        }
    }

    fn with_default_snapshots(
        mut self,
        engine_snapshot: BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
        tank_snapshot: BTreeMap<TankId, openbmp_sim::TankSnapshot>,
    ) -> Self {
        self.default_engine_snapshot = engine_snapshot;
        self.default_tank_snapshot = tank_snapshot;
        self
    }
}

impl<M: Motor> RigidMassResourceModel<M> {
    fn base_properties_for(
        &self,
        active_body: Option<BodyId>,
    ) -> Result<MassProperties, openbmp_sim::ModelEvalError> {
        if let Some(body) = active_body {
            self.resources
                .dry_bodies
                .get(&body)
                .copied()
                .ok_or_else(|| openbmp_sim::ModelEvalError::InvalidState {
                    model: self.model_id,
                    reason: format!("rigid mass resources missing body {}", body.value()).into(),
                })
        } else {
            Ok(self.resources.dry_total)
        }
    }

    /// Dry mass properties of a single configured body (no engines / tanks
    /// / motor). Used to aggregate inert bodies that ride along with a
    /// multi-body continuing stack at a separation.
    fn dry_body_properties(&self, body: BodyId) -> Option<MassProperties> {
        self.resources.dry_bodies.get(&body).copied()
    }

    fn owner_matches(
        &self,
        active_body: Option<BodyId>,
        owner: Option<BodyId>,
        resource: &'static str,
    ) -> Result<bool, openbmp_sim::ModelEvalError> {
        match (active_body, owner) {
            (None, _) => Ok(true),
            (Some(active), Some(owner)) => Ok(active == owner),
            (Some(_), None) => Err(openbmp_sim::ModelEvalError::InvalidState {
                model: self.model_id,
                reason: format!(
                    "{resource}: mounted_to is required for separated-body propagation"
                )
                .into(),
            }),
        }
    }

    fn mass_properties_from_snapshots(
        &self,
        time: SimTime,
        active_body: Option<BodyId>,
        engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
        tank_snapshot: &BTreeMap<TankId, openbmp_sim::TankSnapshot>,
    ) -> Result<MassProperties, openbmp_sim::ModelEvalError> {
        let base = self.base_properties_for(active_body)?;
        let mut total_mass_kg = base.mass.get::<kilogram>();
        let mut weighted_cg = base.center_of_mass_body.vector * total_mass_kg;
        let mut inertia_body = base.inertia_body;

        if let (Some(motor), Some(ignition_time_s)) =
            (&self.motor, self.resources.motor_ignition_time_s)
            && self.owner_matches(active_body, self.resources.motor_owner, "rigid motor mass")?
        {
            let t_since = time.as_seconds() - ignition_time_s;
            let motor_mass_kg =
                motor
                    .mass_kg(t_since)
                    .map_err(|_| openbmp_sim::ModelEvalError::OutOfEnvelope {
                        model: self.model_id,
                        reason: Cow::Borrowed("motor mass query failed"),
                    })?;
            total_mass_kg += motor_mass_kg;
            weighted_cg += base.center_of_mass_body.vector * motor_mass_kg;
        }

        for id in &self.resources.engine_ids {
            let owner = self.resources.engine_owners.get(id).copied();
            if !self.owner_matches(active_body, owner, "engine cluster mass")? {
                continue;
            }
            let Some(snap) = engine_snapshot.get(id) else {
                if active_body.is_none() && engine_snapshot.is_empty() {
                    continue;
                }
                return Err(openbmp_sim::ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "engine cluster mass: snapshot missing declared engine id",
                    ),
                });
            };
            // Tank-coupled engines burn propellant owned and depleted by
            // their tank; subtracting consumed_kg here too would
            // double-count it. Self-contained engines (no binding) burn
            // vehicle mass directly.
            if self.resources.tank_coupled_engines.contains(id) {
                continue;
            }
            total_mass_kg -= snap.consumed_kg;
            weighted_cg -= base.center_of_mass_body.vector * snap.consumed_kg;
        }

        for (id, route) in &self.resources.tank_routes {
            if !self.owner_matches(active_body, Some(route.owner), "tank mass")? {
                continue;
            }
            let Some(snap) = tank_snapshot.get(id) else {
                if active_body.is_none() && tank_snapshot.is_empty() {
                    continue;
                }
                return Err(openbmp_sim::ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed("tank mass: snapshot missing declared tank id"),
                });
            };
            let tank_cg = route.mount_point_body_m + snap.cg_offset_body_m;
            total_mass_kg += snap.mass_kg;
            weighted_cg += tank_cg * snap.mass_kg;
            inertia_body += snap.inertia_delta_body_kg_m2;
        }

        if !total_mass_kg.is_finite() || total_mass_kg <= 0.0 {
            return Err(openbmp_sim::ModelEvalError::InvalidState {
                model: self.model_id,
                reason: format!("rigid mass resources produced invalid mass {total_mass_kg}")
                    .into(),
            });
        }
        let cg = weighted_cg / total_mass_kg;
        Ok(MassProperties::new(
            Mass::new::<kilogram>(total_mass_kg),
            Position3::<Body>::new(cg.x, cg.y, cg.z),
            inertia_body,
        ))
    }

    fn mass_properties_rate_from_snapshots(
        &self,
        time: SimTime,
        active_body: Option<BodyId>,
        engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    ) -> Result<openbmp_sim::MassPropertiesRate, openbmp_sim::ModelEvalError> {
        let mut mass_rate_kg_s = 0.0_f64;
        if let (Some(motor), Some(ignition_time_s)) =
            (&self.motor, self.resources.motor_ignition_time_s)
            && self.owner_matches(active_body, self.resources.motor_owner, "rigid motor mass")?
        {
            let t_since = time.as_seconds() - ignition_time_s;
            mass_rate_kg_s += motor.mass_rate_kg_s(t_since).map_err(|_| {
                openbmp_sim::ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed("motor mass-rate query failed"),
                }
            })?;
        }
        for id in &self.resources.engine_ids {
            let owner = self.resources.engine_owners.get(id).copied();
            if !self.owner_matches(active_body, owner, "engine cluster mass")? {
                continue;
            }
            let Some(snap) = engine_snapshot.get(id) else {
                if active_body.is_none() && engine_snapshot.is_empty() {
                    continue;
                }
                return Err(openbmp_sim::ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "engine cluster mass-rate: snapshot missing declared engine id",
                    ),
                });
            };
            // Mass-loss RATE: -mass_flow is exactly the propellant
            // leaving the engine, whether it is self-contained or
            // tank-coupled. The rate path carries no separate tank term,
            // so (unlike the absolute mass_properties path) tank-coupled
            // engines are NOT skipped here. The kernel reseeds the
            // integrated mass from the absolute partition at separation,
            // keeping the two paths consistent.
            mass_rate_kg_s -= snap.mass_flow_kg_per_s;
        }
        if active_body.is_none()
            && let Some(feedback) = &self.aerothermal_feedback
        {
            mass_rate_kg_s -= feedback.mass_loss_kg_s();
        }
        if !mass_rate_kg_s.is_finite() {
            return Err(openbmp_sim::ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(openbmp_sim::MassPropertiesRate {
            mass_rate_kg_s,
            center_of_mass_rate_body_m_s: Vector3::zeros(),
            inertia_rate_body: nalgebra::Matrix3::zeros(),
        })
    }
}

impl<M: Motor> openbmp_sim::RigidMassModel for RigidMassResourceModel<M> {
    fn mass_properties(&self, t: SimTime) -> Result<MassProperties, openbmp_sim::ModelEvalError> {
        self.mass_properties_from_snapshots(
            t,
            None,
            &self.default_engine_snapshot,
            &self.default_tank_snapshot,
        )
    }

    fn mass_properties_rate(
        &self,
        t: SimTime,
    ) -> Result<openbmp_sim::MassPropertiesRate, openbmp_sim::ModelEvalError> {
        self.mass_properties_rate_from_snapshots(t, None, &self.default_engine_snapshot)
    }

    fn mass_properties_at(
        &self,
        ctx: openbmp_sim::MassContext<'_>,
    ) -> Result<MassProperties, openbmp_sim::ModelEvalError> {
        let engine_snapshot: BTreeMap<EngineId, openbmp_sim::EngineSnapshot> =
            ctx.engine_snapshot.iter().collect();
        let tank_snapshot: BTreeMap<TankId, openbmp_sim::TankSnapshot> =
            ctx.tank_snapshot.iter().collect();
        self.mass_properties_from_snapshots(
            ctx.time,
            ctx.active_body,
            &engine_snapshot,
            &tank_snapshot,
        )
    }

    fn mass_properties_rate_at(
        &self,
        ctx: openbmp_sim::MassContext<'_>,
    ) -> Result<openbmp_sim::MassPropertiesRate, openbmp_sim::ModelEvalError> {
        let engine_snapshot: BTreeMap<EngineId, openbmp_sim::EngineSnapshot> =
            ctx.engine_snapshot.iter().collect();
        self.mass_properties_rate_from_snapshots(ctx.time, ctx.active_body, &engine_snapshot)
    }

    fn supports_separated_body_propagation(&self) -> bool {
        (self.motor.is_none() || self.resources.motor_owner.is_some())
            && self
                .resources
                .engine_ids
                .iter()
                .all(|id| self.resources.engine_owners.contains_key(id))
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

#[derive(Copy, Clone, Debug)]
struct RigidBodySeparationSpec {
    stack_body: BodyId,
    body: BodyId,
    stack_delta_v_body_m_s: [f64; 3],
    stage_delta_v_body_m_s: [f64; 3],
    stack_delta_omega_body_rad_s: [f64; 3],
    stage_delta_omega_body_rad_s: [f64; 3],
}

fn build_rigid_body_separations(
    document: &ScenarioDocument,
    mass_resources: &RigidMassResources,
) -> Result<BTreeMap<BodyId, RigidBodySeparationSpec>, RunnerError> {
    let Some(multi_body) = document.multi_body.as_ref() else {
        return Ok(BTreeMap::new());
    };

    let mut specs = BTreeMap::new();
    for separation in &multi_body.separations {
        let upper_body = BodyId::from_path(&format!(
            "vehicle.assembly.bodies.{id}",
            id = separation.upper_body_id
        ));
        let lower_body = BodyId::from_path(&format!(
            "vehicle.assembly.bodies.{id}",
            id = separation.lower_body_id
        ));
        if !mass_resources.dry_bodies.contains_key(&upper_body) {
            return Err(RunnerError::Assembly {
                field: format!(
                    "multi_body.separation[event_id={}].upper_body_id",
                    separation.event_id
                ),
                reason: format!("body `{}` was not resolved", separation.upper_body_id),
            });
        }
        if !mass_resources.dry_bodies.contains_key(&lower_body) {
            return Err(RunnerError::Assembly {
                field: format!(
                    "multi_body.separation[event_id={}].lower_body_id",
                    separation.event_id
                ),
                reason: format!("body `{}` was not resolved", separation.lower_body_id),
            });
        }
        let previous = specs.insert(
            lower_body,
            RigidBodySeparationSpec {
                stack_body: upper_body,
                body: lower_body,
                stack_delta_v_body_m_s: separation
                    .upper_delta_v_body_m_s
                    .unwrap_or([0.0, 0.0, 0.0]),
                stage_delta_v_body_m_s: separation
                    .lower_delta_v_body_m_s
                    .unwrap_or([0.0, 0.0, 0.0]),
                stack_delta_omega_body_rad_s: separation
                    .upper_delta_omega_body_rad_s
                    .unwrap_or([0.0, 0.0, 0.0]),
                stage_delta_omega_body_rad_s: separation
                    .lower_delta_omega_body_rad_s
                    .unwrap_or([0.0, 0.0, 0.0]),
            },
        );
        if previous.is_some() {
            return Err(RunnerError::Assembly {
                field: "multi_body.separation.lower_body_id".to_owned(),
                reason: format!(
                    "body `{}` has multiple separation specs",
                    separation.lower_body_id
                ),
            });
        }
    }
    Ok(specs)
}

/// Bodies (other than `stack_body`) still attached to the continuing stack,
/// i.e. the membership minus the lead body. Their dry mass must be folded
/// into `stack_mass_properties` so a multi-body stack conserves mass.
fn continuing_inert_bodies(stack_bodies: &BTreeSet<BodyId>, stack_body: BodyId) -> Vec<BodyId> {
    stack_bodies
        .iter()
        .copied()
        .filter(|b| *b != stack_body)
        .collect()
}

fn apply_jettison_events<I, F, MOM, MM, E, SC>(
    kernel: &mut openbmp_sim::RigidBodyKernel<I, F, MOM, MM, E, SC>,
    fired: &[openbmp_sim::FiredEvent<ScenarioScriptAction>],
    separation_specs: &BTreeMap<BodyId, RigidBodySeparationSpec>,
    mass_model: &RigidMassEither,
    stack_bodies: &mut BTreeSet<BodyId>,
) -> Result<(), RunnerError>
where
    I: openbmp_sim::Integrator<RigidBodyState>,
    F: ForceModel<RigidBodyState>,
    MOM: openbmp_sim::MomentModel<RigidBodyState>,
    MM: openbmp_sim::RigidMassModel,
    E: openbmp_sim::EnvironmentModel,
    SC: openbmp_sim::StopCondition<RigidBodyState>,
{
    for event in fired {
        match &event.action {
            ScenarioScriptAction::JettisonStage { body } => {
                let separation = separation_specs.get(body).copied().ok_or_else(|| {
                    RunnerError::UnsupportedScenario {
                        what: format!(
                            "jettison_stage event {} fired for body id {} with no \
                                 matching [multi_body] separation",
                            event.binding_id.value(),
                            body.value()
                        ),
                    }
                })?;
                // The departing body leaves the stack; the remaining
                // members (minus the lead stack body) ride along and their
                // dry mass is folded into the continuing-stack mass.
                stack_bodies.remove(body);
                let inert = continuing_inert_bodies(stack_bodies, separation.stack_body);
                let runtime =
                    build_runtime_rigid_body_separation(kernel, mass_model, separation, &inert)?;
                kernel.jettison_rigid_body(runtime)?;
            }
            ScenarioScriptAction::JettisonBodies { bodies } => {
                // Remove every departing body from the stack FIRST so the
                // continuing-inert set reflects the post-batch membership.
                for body in bodies {
                    stack_bodies.remove(body);
                }
                let mut batch = Vec::with_capacity(bodies.len());
                for body in bodies {
                    let separation = separation_specs.get(body).copied().ok_or_else(|| {
                        RunnerError::UnsupportedScenario {
                            what: format!(
                                "jettison_bodies event {} fired for body id {} with no \
                                     matching [multi_body] separation",
                                event.binding_id.value(),
                                body.value()
                            ),
                        }
                    })?;
                    let inert = continuing_inert_bodies(stack_bodies, separation.stack_body);
                    batch.push(build_runtime_rigid_body_separation(
                        kernel, mass_model, separation, &inert,
                    )?);
                }
                kernel.jettison_rigid_bodies(&batch)?;
            }
            _ => {}
        }
    }
    Ok(())
}

fn build_runtime_rigid_body_separation<I, F, MOM, MM, E, SC>(
    kernel: &openbmp_sim::RigidBodyKernel<I, F, MOM, MM, E, SC>,
    mass_model: &RigidMassEither,
    separation: RigidBodySeparationSpec,
    continuing_inert: &[BodyId],
) -> Result<RigidBodySeparation, RunnerError>
where
    I: openbmp_sim::Integrator<RigidBodyState>,
    F: ForceModel<RigidBodyState>,
    MOM: openbmp_sim::MomentModel<RigidBodyState>,
    MM: openbmp_sim::RigidMassModel,
    E: openbmp_sim::EnvironmentModel,
    SC: openbmp_sim::StopCondition<RigidBodyState>,
{
    let time = kernel.current_time();
    let engine_snapshot = kernel.engine_snapshot();
    let tank_snapshot = kernel.tank_snapshot();
    let stack_mass_properties = mass_model
        .mass_properties_at(openbmp_sim::MassContext {
            time,
            active_body: Some(separation.stack_body),
            engine_snapshot: openbmp_sim::EngineSnapshotView::new(engine_snapshot),
            tank_snapshot: openbmp_sim::TankSnapshotView::new(tank_snapshot),
        })
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("continuing-stack mass properties at separation failed: {err}"),
        })?;
    let stage_mass_properties = mass_model
        .mass_properties_at(openbmp_sim::MassContext {
            time,
            active_body: Some(separation.body),
            engine_snapshot: openbmp_sim::EngineSnapshotView::new(engine_snapshot),
            tank_snapshot: openbmp_sim::TankSnapshotView::new(tank_snapshot),
        })
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("departing-stage mass properties at separation failed: {err}"),
        })?;
    // A continuing stack may be MORE than the single `stack_body` (e.g. an
    // upper stage that still carries an unjettisoned fairing and payload).
    // mass_properties_at(stack_body) covers only that body's dry mass plus
    // its own engines/tanks, so fold in the dry mass of every other body
    // still attached to the stack. Without this the still-attached inert
    // bodies' mass would vanish at this separation (stack + stage would not
    // conserve the pre-separation composite). For a single-body continuing
    // stack `continuing_inert` is empty and this is a no-op (byte-identical
    // to the prior behaviour).
    let mut stack_mass_properties = stack_mass_properties;
    for body in continuing_inert {
        let dry = mass_model.dry_body_properties(*body).ok_or_else(|| {
            RunnerError::UnsupportedScenario {
                what: format!(
                    "continuing-stack inert body {} has no dry mass properties",
                    body.value()
                ),
            }
        })?;
        stack_mass_properties = combine_dry_body(stack_mass_properties, dry);
    }
    Ok(RigidBodySeparation {
        stack_body: separation.stack_body,
        body: separation.body,
        stack_mass_properties,
        stage_mass_properties,
        stack_delta_v_body_m_s: separation.stack_delta_v_body_m_s,
        stage_delta_v_body_m_s: separation.stage_delta_v_body_m_s,
        stack_delta_omega_body_rad_s: separation.stack_delta_omega_body_rad_s,
        stage_delta_omega_body_rad_s: separation.stage_delta_omega_body_rad_s,
    })
}

fn load_models(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<LoadedModels, RunnerError> {
    let aero_deck = crate::aero::load_aero_deck(document, resolved_files)?;
    let motor = crate::propulsion::load_solid_motor(document, resolved_files)?;
    Ok(LoadedModels { aero_deck, motor })
}

fn build_initial_state(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    mass_resources: &RigidMassResources,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    tank_snapshot: &BTreeMap<TankId, openbmp_sim::TankSnapshot>,
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
    let mass_props = RigidMassResourceModel::new(
        mass_resources.clone(),
        loaded.motor.clone(),
        None,
        RIGID_BODY_MOTOR_MASS_MODEL_ID,
    )
    .with_default_snapshots(engine_snapshot.clone(), tank_snapshot.clone())
    .mass_properties(start_time)
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("initial rigid-body mass properties failed: {err}"),
    })?;

    Ok(rigid_body_state_from_parts(
        start_time, p, v, q, omega, mass_props,
    ))
}

fn rigid_body_state_from_parts(
    time: SimTime,
    position_eci_m: [f64; 3],
    velocity_eci_m_s: [f64; 3],
    quaternion_body_to_eci_xyzw: [f64; 4],
    angular_velocity_body_rad_s: [f64; 3],
    mass_props: MassProperties,
) -> RigidBodyState {
    // Quaternion is [x, y, z, w] in the scenario file; nalgebra
    // expects (w, x, y, z) for `Quaternion::new`. Validation in the
    // scenario layer guarantees unit-norm to 1e-9.
    let q = quaternion_body_to_eci_xyzw;
    let raw = nalgebra::Quaternion::new(q[3], q[0], q[1], q[2]);
    let unit = nalgebra::UnitQuaternion::from_quaternion(raw);
    let orientation =
        Quaternion::<openbmp_core::Body, openbmp_core::Eci>::from_unit_quaternion(unit);

    let p = position_eci_m;
    let v = velocity_eci_m_s;
    let omega = angular_velocity_body_rad_s;
    RigidBodyState::new(
        time,
        Position3::new(p[0], p[1], p[2]),
        Velocity3::new(v[0], v[1], v[2]),
        orientation,
        AngularVelocity3::<Body>::new(omega[0], omega[1], omega[2]),
        mass_props,
    )
}

fn seed_initial_rigid_body_lanes<I, F, MOM, MM, E, SC>(
    kernel: &mut openbmp_sim::RigidBodyKernel<I, F, MOM, MM, E, SC>,
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    mass_resources: &RigidMassResources,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    tank_snapshot: &BTreeMap<TankId, openbmp_sim::TankSnapshot>,
) -> Result<(), RunnerError>
where
    I: openbmp_sim::Integrator<RigidBodyState>,
    F: ForceModel<RigidBodyState>,
    MOM: openbmp_sim::MomentModel<RigidBodyState>,
    MM: openbmp_sim::RigidMassModel,
    E: openbmp_sim::EnvironmentModel,
    SC: openbmp_sim::StopCondition<RigidBodyState>,
{
    let Some(multi_body) = document.multi_body.as_ref() else {
        return Ok(());
    };
    if multi_body.initial_lanes.is_empty() {
        return Ok(());
    }

    let primary_body_id =
        multi_body
            .primary_body_id
            .as_ref()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "internal invariant: multi_body.primary_body_id missing for initial lanes"
                    .to_owned(),
            })?;
    let primary_body = body_id_from_scenario_text(primary_body_id);
    let start_time = SimTime::from_seconds(document.time.start_s);
    let mass_model = RigidMassResourceModel::new(
        mass_resources.clone(),
        loaded.motor.clone(),
        None,
        RIGID_BODY_MOTOR_MASS_MODEL_ID,
    );
    let primary_mass_properties = mass_model
        .mass_properties_from_snapshots(
            start_time,
            Some(primary_body),
            engine_snapshot,
            tank_snapshot,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("initial primary body `{primary_body_id}` mass properties failed: {err}"),
        })?;

    let mut lanes = Vec::with_capacity(multi_body.initial_lanes.len());
    for lane in &multi_body.initial_lanes {
        let body = body_id_from_scenario_text(&lane.body_id);
        let mass_props = mass_model
            .mass_properties_from_snapshots(start_time, Some(body), engine_snapshot, tank_snapshot)
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!(
                    "initial lane body `{}` mass properties failed: {err}",
                    lane.body_id
                ),
            })?;
        lanes.push(InitialRigidBodyLane {
            body,
            state: rigid_body_state_from_parts(
                start_time,
                lane.position_eci_m,
                lane.velocity_eci_m_s,
                lane.quaternion_body_to_eci_xyzw,
                lane.angular_velocity_body_rad_s,
                mass_props,
            ),
        });
    }

    kernel
        .seed_rigid_body_lanes(primary_body, primary_mass_properties, &lanes)
        .map_err(RunnerError::Simulation)
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
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Box<dyn ForceModel<RigidBodyState> + Send + Sync>, RunnerError> {
    match document.environment.gravity.as_str() {
        "constant" => {
            let g = document.environment.gravity_m_s2.ok_or_else(|| {
                RunnerError::UnsupportedScenario {
                    what: "environment.gravity_m_s2 missing for constant gravity".to_owned(),
                }
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
            let r_e =
                document
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
        "third_body" => {
            let model = crate::celestial::build_third_body_gravity(document, resolved_files)?;
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

fn body_id_from_scenario_text(id: &str) -> BodyId {
    BodyId::from_path(&format!("vehicle.assembly.bodies.{id}"))
}

fn optional_body_owner(owner: Option<&str>) -> Option<BodyId> {
    owner.map(body_id_from_scenario_text)
}

fn engine_owner_map(
    document: &ScenarioDocument,
) -> Result<BTreeMap<openbmp_core::EngineId, BodyId>, RunnerError> {
    let mut owners = BTreeMap::new();
    for engine in &document.vehicle.assembly.engines {
        let id = openbmp_core::EngineId::from_path(&format!(
            "vehicle.assembly.engines.{id}",
            id = engine.id
        ));
        if let Some(owner) = optional_body_owner(engine.mounted_to.as_deref()) {
            owners.insert(id, owner);
        } else if document.multi_body.is_some() {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "vehicle.assembly.engines.{}.mounted_to is required for multi_body \
                     per-body force-stack ownership",
                    engine.id
                ),
            });
        }
    }
    Ok(owners)
}

fn tank_owner_map(document: &ScenarioDocument) -> BTreeMap<openbmp_core::TankId, BodyId> {
    let mut owners = BTreeMap::new();
    for tank in &document.vehicle.assembly.tanks {
        let id =
            openbmp_core::TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = tank.id));
        owners.insert(id, body_id_from_scenario_text(&tank.mounted_to));
    }
    owners
}

fn recovery_owner_map(
    document: &ScenarioDocument,
) -> Result<BTreeMap<openbmp_core::RecoveryId, BodyId>, RunnerError> {
    let mut owners = BTreeMap::new();
    for recovery in &document.vehicle.assembly.recovery {
        let id = openbmp_core::RecoveryId::from_path(&format!(
            "vehicle.assembly.recovery.{id}",
            id = recovery.id
        ));
        if let Some(owner) = optional_body_owner(recovery.mounted_to.as_deref()) {
            owners.insert(id, owner);
        } else if document.multi_body.is_some() {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "vehicle.assembly.recovery.{}.mounted_to is required for multi_body \
                     per-body force-stack ownership",
                    recovery.id
                ),
            });
        }
    }
    Ok(owners)
}

#[allow(clippy::too_many_lines)] // the recovery-rack force-adapter wiring branch is large
fn build_vehicle(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    assembly: &Assembly,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    aerothermal_sink: Option<crate::aerothermal::LiveAerothermalSink>,
) -> Result<KernelVehicle<RigidBodyState>, RunnerError> {
    let mut named: Vec<NamedForceModel<RigidBodyState>> = Vec::new();
    for name in document.force_model_universe() {
        match name.as_str() {
            "gravity" => {
                let force = build_gravity_force_adapter_rigid_body(document, resolved_files)?;
                named.push(NamedForceModel::new("gravity", force));
            }
            "aero" => {
                let atmosphere = build_document_runtime_atmosphere(document)?;
                let owner = optional_body_owner(
                    document.aero.as_ref().and_then(|a| a.mounted_to.as_deref()),
                );
                let method_kind = document
                    .aero
                    .as_ref()
                    .and_then(|aero| aero.method.as_ref())
                    .map_or("deck", |method| method.kind.as_str());
                if method_kind == "deck" {
                    let deck = loaded.aero_deck.clone().ok_or_else(|| {
                        RunnerError::UnsupportedScenario {
                            what: "forces includes `aero` but [aero] block is missing".to_owned(),
                        }
                    })?;
                    let drag = if let Some(owner) = owner {
                        DeckDragForceAdapter::new_owned(
                            deck,
                            atmosphere,
                            RIGID_BODY_AERO_MODEL_ID,
                            owner,
                        )
                    } else {
                        DeckDragForceAdapter::new(deck, atmosphere, RIGID_BODY_AERO_MODEL_ID)
                    };
                    named.push(NamedForceModel::new("aero", Box::new(drag)));
                } else {
                    let (method, reference_length_m) =
                        crate::aero::build_aero_method(document, loaded.aero_deck.clone())?
                            .ok_or_else(|| RunnerError::UnsupportedScenario {
                                what: "forces includes `aero` but [aero] block is missing"
                                    .to_owned(),
                            })?;
                    let adapter = if let Some(owner) = owner {
                        AeroMethodForceAdapter::new_owned(
                            method,
                            atmosphere,
                            RIGID_BODY_AERO_MODEL_ID,
                            reference_length_m,
                            owner,
                        )
                    } else {
                        AeroMethodForceAdapter::new(
                            method,
                            atmosphere,
                            RIGID_BODY_AERO_MODEL_ID,
                            reference_length_m,
                        )
                    };
                    named.push(NamedForceModel::new("aero", Box::new(adapter)));
                }
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
                    let thrust = if let Some(owner) = optional_body_owner(
                        document
                            .propulsion
                            .as_ref()
                            .and_then(|p| p.motor.as_ref())
                            .and_then(|m| m.mounted_to.as_deref()),
                    ) {
                        MotorThrustForceAdapter::new_owned(
                            motor,
                            ignition_time_s,
                            RIGID_BODY_THRUST_MODEL_ID,
                            owner,
                        )
                    } else {
                        MotorThrustForceAdapter::new(
                            motor,
                            ignition_time_s,
                            RIGID_BODY_THRUST_MODEL_ID,
                        )
                    };
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
                    let engine_owners = engine_owner_map(document)?;
                    let thrust = if engine_owners.is_empty() {
                        EngineClusterForceAdapter::new(
                            engine_ids,
                            RIGID_BODY_ENGINE_CLUSTER_THRUST_MODEL_ID,
                        )
                    } else {
                        EngineClusterForceAdapter::new_with_owners(
                            engine_ids,
                            engine_owners,
                            RIGID_BODY_ENGINE_CLUSTER_THRUST_MODEL_ID,
                        )
                    };
                    named.push(NamedForceModel::new("thrust", Box::new(thrust)));
                }
            }
            "aerothermal_diagnostics" => {
                let sink =
                    aerothermal_sink
                        .clone()
                        .ok_or_else(|| RunnerError::UnsupportedScenario {
                            what: MISSING_AEROTHERMAL_DIAGNOSTICS_MESSAGE.to_owned(),
                        })?;
                let adapter = crate::aerothermal::StagnationHeatingForceAdapter::new(
                    sink,
                    RIGID_BODY_AEROTHERMAL_MODEL_ID,
                );
                named.push(NamedForceModel::new(
                    "aerothermal_diagnostics",
                    Box::new(adapter),
                ));
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
        let tank_owners = tank_owner_map(document);
        let tank_force = openbmp_vehicle::TankRackForceAdapter::new_with_owners(
            tank_ids,
            tank_owners,
            RIGID_BODY_TANK_RACK_FORCE_MODEL_ID,
        );
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
        let atmosphere = build_document_runtime_atmosphere(document)?;
        let recovery_owners = recovery_owner_map(document)?;
        let recovery_force = if recovery_owners.is_empty() {
            openbmp_vehicle::RecoveryRackForceAdapter::new(
                recovery_ids,
                atmosphere,
                RIGID_BODY_RECOVERY_RACK_FORCE_MODEL_ID,
            )
        } else {
            openbmp_vehicle::RecoveryRackForceAdapter::new_with_owners(
                recovery_ids,
                recovery_owners,
                atmosphere,
                RIGID_BODY_RECOVERY_RACK_FORCE_MODEL_ID,
            )
        };
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
    KernelVehicle::new_with_default_active_models(
        named,
        vec![],
        Box::new(vehicle_mass),
        crate::default_active_force_models(document),
        crate::phase_force_overrides(document),
    )
    .map_err(|e| RunnerError::UnsupportedScenario {
        what: format!("KernelVehicle construction failed: {e}"),
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
        let engine_owners = engine_owner_map(document)?;
        if engine_owners.is_empty() {
            Box::new(EngineClusterMassAdapter::new(
                dry_mass_kg,
                engine_ids,
                RIGID_BODY_ENGINE_CLUSTER_MASS_MODEL_ID,
            ))
        } else {
            Box::new(EngineClusterMassAdapter::new_with_owners(
                dry_mass_kg,
                engine_ids,
                engine_owners,
                RIGID_BODY_ENGINE_CLUSTER_MASS_MODEL_ID,
            ))
        }
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

#[allow(clippy::if_not_else)]
fn build_moment_model(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
) -> Result<KernelVehicle<RigidBodyState>, RunnerError> {
    let assembly = &document.vehicle.assembly;
    let mut named: Vec<NamedMomentModel<RigidBodyState>> = Vec::new();

    if document
        .force_model_universe()
        .iter()
        .any(|model| model == "aero")
    {
        let atmosphere = build_document_runtime_atmosphere(document)?;
        let owner =
            optional_body_owner(document.aero.as_ref().and_then(|a| a.mounted_to.as_deref()));
        let (method, reference_length_m) =
            crate::aero::build_aero_method(document, loaded.aero_deck.clone())?.ok_or_else(
                || RunnerError::UnsupportedScenario {
                    what: "forces includes `aero` but [aero] block is missing".to_owned(),
                },
            )?;
        let adapter = if let Some(owner) = owner {
            AeroMethodMomentAdapter::new_owned(
                method,
                atmosphere,
                RIGID_BODY_AERO_MODEL_ID,
                reference_length_m,
                owner,
            )
        } else {
            AeroMethodMomentAdapter::new(
                method,
                atmosphere,
                RIGID_BODY_AERO_MODEL_ID,
                reference_length_m,
            )
        };
        named.push(NamedMomentModel::new("aero", Box::new(adapter)));
    }

    if !assembly.engines.is_empty() {
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
        let engine_owners = engine_owner_map(document)?;
        let adapter = if engine_owners.is_empty() {
            EngineClusterMomentAdapter::new(
                engine_ids,
                mount_points_body,
                RIGID_BODY_ENGINE_CLUSTER_MOMENT_MODEL_ID,
            )
        } else {
            EngineClusterMomentAdapter::new_with_owners(
                engine_ids,
                mount_points_body,
                engine_owners,
                RIGID_BODY_ENGINE_CLUSTER_MOMENT_MODEL_ID,
            )
        }
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("EngineClusterMomentAdapter construction failed: {err}"),
        })?;
        named.push(NamedMomentModel::new("thrust", Box::new(adapter)));
    }

    if !assembly.tanks.is_empty() {
        let tank_ids: Vec<openbmp_core::TankId> = assembly
            .tanks
            .iter()
            .map(|t| {
                openbmp_core::TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = t.id))
            })
            .collect();
        named.push(NamedMomentModel::new(
            "tank_reaction",
            Box::new(openbmp_vehicle::TankRackMomentAdapter::new_with_owners(
                tank_ids,
                tank_owner_map(document),
                RIGID_BODY_TANK_RACK_MOMENT_MODEL_ID,
            )),
        ));
    }

    let direct_torque_adapter = build_direct_torque_adapter(document);

    // Combinations of direct-torque with engine-cluster or tank-rack
    // moment models are not supported. Closed-loop FC validation
    // scenarios use direct-torque alone; if a downstream scenario
    // combines them, fail closed.
    if direct_torque_adapter.is_some()
        && (!assembly.engines.is_empty() || !assembly.tanks.is_empty())
    {
        return Err(RunnerError::UnsupportedScenario {
            what: "direct_torque effectors combined with engine-cluster or tank moment models \
                   is not supported; use a dedicated closed-loop validation \
                   scenario without engines/tanks"
                .to_string(),
        });
    }
    if let Some(adapter) = direct_torque_adapter {
        named.push(NamedMomentModel::new("direct_torque", Box::new(adapter)));
    }

    let known_moments: Vec<String> = named.iter().map(|entry| entry.name.clone()).collect();
    let mut default_active = crate::default_active_force_models(document);
    if known_moments.iter().any(|name| name == "direct_torque")
        && !default_active.iter().any(|name| name == "direct_torque")
    {
        default_active.push("direct_torque".to_owned());
    }
    let default_active = filter_models_to(&default_active, &known_moments);
    KernelVehicle::new_with_default_active_models(
        vec![],
        named,
        Box::new(ConstantMass::new(1.0)),
        default_active,
        phase_overrides_filtered_to(document, &known_moments),
    )
    .map_err(|e| RunnerError::UnsupportedScenario {
        what: format!("rigid moment KernelVehicle construction failed: {e}"),
    })
}

fn phase_overrides_filtered_to(
    document: &ScenarioDocument,
    model_names: &[String],
) -> BTreeMap<u64, Vec<String>> {
    crate::phase_force_overrides(document)
        .into_iter()
        .map(|(phase, models)| (phase, filter_models_to(&models, model_names)))
        .collect()
}

fn filter_models_to(models: &[String], model_names: &[String]) -> Vec<String> {
    let known: BTreeSet<&str> = model_names.iter().map(String::as_str).collect();
    models
        .iter()
        .filter(|model| known.contains(model.as_str()))
        .cloned()
        .collect()
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
                owner: optional_body_owner(effector.mounted_to.as_deref()),
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

/// Build the kernel's rigid mass model. Dry body mass properties are
/// augmented by body-owned motor, engine, and tank resources. Before
/// separation `active_body = None` includes all resources; after
/// separation each lane evaluates only resources mounted to that
/// lane's body.
type RigidMassEither = RigidMassEitherKind;

#[derive(Clone, Debug)]
enum RigidMassEitherKind {
    Resource(RigidMassResourceModel<SolidMotor>),
}

impl openbmp_sim::RigidMassModel for RigidMassEitherKind {
    fn mass_properties(&self, t: SimTime) -> Result<MassProperties, openbmp_sim::ModelEvalError> {
        match self {
            Self::Resource(m) => m.mass_properties(t),
        }
    }

    fn mass_properties_rate(
        &self,
        t: SimTime,
    ) -> Result<openbmp_sim::MassPropertiesRate, openbmp_sim::ModelEvalError> {
        match self {
            Self::Resource(m) => m.mass_properties_rate(t),
        }
    }

    fn mass_properties_at(
        &self,
        ctx: openbmp_sim::MassContext<'_>,
    ) -> Result<MassProperties, openbmp_sim::ModelEvalError> {
        match self {
            Self::Resource(m) => m.mass_properties_at(ctx),
        }
    }

    fn mass_properties_rate_at(
        &self,
        ctx: openbmp_sim::MassContext<'_>,
    ) -> Result<openbmp_sim::MassPropertiesRate, openbmp_sim::ModelEvalError> {
        match self {
            Self::Resource(m) => m.mass_properties_rate_at(ctx),
        }
    }

    fn supports_separated_body_propagation(&self) -> bool {
        match self {
            Self::Resource(m) => m.supports_separated_body_propagation(),
        }
    }
}

impl RigidMassEitherKind {
    /// Dry mass properties of one configured body (no engines / tanks /
    /// motor). Used to fold still-attached inert bodies into a multi-body
    /// continuing stack at separation time.
    fn dry_body_properties(&self, body: BodyId) -> Option<MassProperties> {
        match self {
            Self::Resource(m) => m.dry_body_properties(body),
        }
    }
}

/// Fold an inert body's dry mass properties into a running aggregate, using
/// the same convention as [`mass_properties_from_snapshots`]: masses add,
/// the center of mass is the mass-weighted centroid, and the inertia tensor
/// (expressed about the shared body-frame origin) sums. This is how a
/// multi-body continuing stack's mass is assembled at a separation so that
/// stack + departing stage conserve the pre-separation composite mass.
fn combine_dry_body(base: MassProperties, add: MassProperties) -> MassProperties {
    let m_base = base.mass.get::<kilogram>();
    let m_add = add.mass.get::<kilogram>();
    let total = m_base + m_add;
    let weighted =
        base.center_of_mass_body.vector * m_base + add.center_of_mass_body.vector * m_add;
    let cg = if total > 0.0 {
        weighted / total
    } else {
        base.center_of_mass_body.vector
    };
    MassProperties::new(
        Mass::new::<kilogram>(total),
        Position3::<Body>::new(cg.x, cg.y, cg.z),
        base.inertia_body + add.inertia_body,
    )
}

fn build_mass_model(
    loaded: &LoadedModels,
    mass_resources: &RigidMassResources,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    tank_snapshot: &BTreeMap<TankId, openbmp_sim::TankSnapshot>,
    aerothermal_feedback: Option<crate::aerothermal::AerothermalMassFeedback>,
) -> RigidMassEither {
    RigidMassEitherKind::Resource(
        RigidMassResourceModel::new(
            mass_resources.clone(),
            loaded.motor.clone(),
            aerothermal_feedback,
            RIGID_BODY_MOTOR_MASS_MODEL_ID,
        )
        .with_default_snapshots(engine_snapshot.clone(), tank_snapshot.clone()),
    )
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

fn build_schema_metadata(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<BTreeMap<String, String>, RunnerError> {
    let mut metadata = BTreeMap::new();
    for (field, file) in resolved_files {
        metadata.insert(
            format!("openbmp.scenario_files.{field}"),
            file.sha256_hex.clone(),
        );
    }
    super::append_frame_time_metadata(document, &mut metadata);
    super::append_solver_metadata(document, &mut metadata);
    crate::propulsion::append_staging_analysis_metadata(document, &mut metadata)?;
    Ok(metadata)
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
struct AerothermalTelemetryChannels {
    q_conv: TelemetryChannel<f64>,
    q_rad: TelemetryChannel<f64>,
    h_aw: TelemetryChannel<f64>,
    recovery_temperature: TelemetryChannel<f64>,
    stagnation_temperature: TelemetryChannel<f64>,
    knudsen: TelemetryChannel<f64>,
    wall_temperature: TelemetryChannel<f64>,
    backwall_temperature: TelemetryChannel<f64>,
    recession_depth: TelemetryChannel<f64>,
    gas_mdot: TelemetryChannel<f64>,
    mass_loss: TelemetryChannel<f64>,
}

#[derive(Debug)]
struct SeparatedBodyTelemetryChannels {
    body: BodyId,
    separated: TelemetryChannel<bool>,
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
}

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
    separated_bodies: Vec<SeparatedBodyTelemetryChannels>,
    has_atmosphere: bool,
    atmosphere_density: Option<TelemetryChannel<f64>>,
    atmosphere_pressure: Option<TelemetryChannel<f64>>,
    atmosphere_temperature: Option<TelemetryChannel<f64>>,
    atmosphere_speed_of_sound: Option<TelemetryChannel<f64>>,
    force_components: ForceComponentChannels,
    active_models: Option<TelemetryChannel<String>>,
    aerothermal: Option<AerothermalTelemetryChannels>,
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

        let mut separated_bodies = Vec::new();
        if let Some(multi_body) = &document.multi_body {
            let mut seen_lanes = BTreeSet::new();
            let mut lane_ids: Vec<&str> = Vec::new();
            for lane in &multi_body.initial_lanes {
                if seen_lanes.insert(lane.body_id.as_str()) {
                    lane_ids.push(lane.body_id.as_str());
                }
            }
            for separation in &multi_body.separations {
                if seen_lanes.insert(separation.lower_body_id.as_str()) {
                    lane_ids.push(separation.lower_body_id.as_str());
                }
            }
            for lane_id in lane_ids {
                let body = body_id_from_scenario_text(lane_id);
                let prefix = format!("body.{lane_id}");
                separated_bodies.push(SeparatedBodyTelemetryChannels {
                    body,
                    separated: TelemetryChannel::<bool>::new(
                        alloc(),
                        format!("{prefix}.separated"),
                        "bool",
                        None::<&str>,
                    )?,
                    position_x: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.position_x_m"),
                        "m",
                        Some("ECI"),
                    )?,
                    position_y: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.position_y_m"),
                        "m",
                        Some("ECI"),
                    )?,
                    position_z: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.position_z_m"),
                        "m",
                        Some("ECI"),
                    )?,
                    velocity_x: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.velocity_x_m_s"),
                        "m/s",
                        Some("ECI"),
                    )?,
                    velocity_y: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.velocity_y_m_s"),
                        "m/s",
                        Some("ECI"),
                    )?,
                    velocity_z: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.velocity_z_m_s"),
                        "m/s",
                        Some("ECI"),
                    )?,
                    mass: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.mass_kg"),
                        "kg",
                        None::<&str>,
                    )?,
                    quaternion_x: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.attitude.q_x"),
                        "1",
                        None::<&str>,
                    )?,
                    quaternion_y: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.attitude.q_y"),
                        "1",
                        None::<&str>,
                    )?,
                    quaternion_z: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.attitude.q_z"),
                        "1",
                        None::<&str>,
                    )?,
                    quaternion_w: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.attitude.q_w"),
                        "1",
                        None::<&str>,
                    )?,
                    angular_velocity_x: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.angular_velocity.x_rad_s"),
                        "rad/s",
                        Some("Body"),
                    )?,
                    angular_velocity_y: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.angular_velocity.y_rad_s"),
                        "rad/s",
                        Some("Body"),
                    )?,
                    angular_velocity_z: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("{prefix}.angular_velocity.z_rad_s"),
                        "rad/s",
                        Some("Body"),
                    )?,
                });
            }
        }

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

        let mut force_components = Vec::with_capacity(document.force_model_universe().len());
        for name in document.force_model_universe() {
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

        let active_models = document
            .forces
            .as_ref()
            .is_some_and(|forces| !forces.phase_override.is_empty())
            .then(|| {
                TelemetryChannel::<String>::new(
                    alloc(),
                    "forces.active_models",
                    "text",
                    None::<&str>,
                )
            })
            .transpose()?;

        let aerothermal = if document.aerothermal.is_some() {
            Some(AerothermalTelemetryChannels {
                q_conv: TelemetryChannel::<f64>::new(
                    alloc(),
                    "aerothermal.q_conv_w_m2",
                    "W/m^2",
                    None::<&str>,
                )?,
                q_rad: TelemetryChannel::<f64>::new(
                    alloc(),
                    "aerothermal.q_rad_w_m2",
                    "W/m^2",
                    None::<&str>,
                )?,
                h_aw: TelemetryChannel::<f64>::new(
                    alloc(),
                    "aerothermal.h_aw_j_kg",
                    "J/kg",
                    None::<&str>,
                )?,
                recovery_temperature: TelemetryChannel::<f64>::new(
                    alloc(),
                    "aerothermal.recovery_temperature_k",
                    "K",
                    None::<&str>,
                )?,
                stagnation_temperature: TelemetryChannel::<f64>::new(
                    alloc(),
                    "aerothermal.stagnation_temperature_k",
                    "K",
                    None::<&str>,
                )?,
                knudsen: TelemetryChannel::<f64>::new(
                    alloc(),
                    "aerothermal.knudsen",
                    "1",
                    None::<&str>,
                )?,
                wall_temperature: TelemetryChannel::<f64>::new(
                    alloc(),
                    "aerothermal.wall_temperature_k",
                    "K",
                    None::<&str>,
                )?,
                backwall_temperature: TelemetryChannel::<f64>::new(
                    alloc(),
                    "aerothermal.backwall_temperature_k",
                    "K",
                    None::<&str>,
                )?,
                recession_depth: TelemetryChannel::<f64>::new(
                    alloc(),
                    "aerothermal.recession_depth_m",
                    "m",
                    None::<&str>,
                )?,
                gas_mdot: TelemetryChannel::<f64>::new(
                    alloc(),
                    "aerothermal.gas_mdot_kg_m2_s",
                    "kg/(m^2*s)",
                    None::<&str>,
                )?,
                mass_loss: TelemetryChannel::<f64>::new(
                    alloc(),
                    "mass.aerothermal_mass_loss_kg_s",
                    "kg/s",
                    None::<&str>,
                )?,
            })
        } else {
            None
        };

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
            separated_bodies,
            has_atmosphere,
            atmosphere_density,
            atmosphere_pressure,
            atmosphere_temperature,
            atmosphere_speed_of_sound,
            force_components,
            active_models,
            aerothermal,
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
        for separated in &self.separated_bodies {
            channels.push(separated.separated.metadata().clone());
            channels.push(separated.position_x.metadata().clone());
            channels.push(separated.position_y.metadata().clone());
            channels.push(separated.position_z.metadata().clone());
            channels.push(separated.velocity_x.metadata().clone());
            channels.push(separated.velocity_y.metadata().clone());
            channels.push(separated.velocity_z.metadata().clone());
            channels.push(separated.mass.metadata().clone());
            channels.push(separated.quaternion_x.metadata().clone());
            channels.push(separated.quaternion_y.metadata().clone());
            channels.push(separated.quaternion_z.metadata().clone());
            channels.push(separated.quaternion_w.metadata().clone());
            channels.push(separated.angular_velocity_x.metadata().clone());
            channels.push(separated.angular_velocity_y.metadata().clone());
            channels.push(separated.angular_velocity_z.metadata().clone());
        }
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
        if let Some(active_models) = &self.active_models {
            channels.push(active_models.metadata().clone());
        }
        if let Some(aerothermal) = &self.aerothermal {
            channels.push(aerothermal.q_conv.metadata().clone());
            channels.push(aerothermal.q_rad.metadata().clone());
            channels.push(aerothermal.h_aw.metadata().clone());
            channels.push(aerothermal.recovery_temperature.metadata().clone());
            channels.push(aerothermal.stagnation_temperature.metadata().clone());
            channels.push(aerothermal.knudsen.metadata().clone());
            channels.push(aerothermal.wall_temperature.metadata().clone());
            channels.push(aerothermal.backwall_temperature.metadata().clone());
            channels.push(aerothermal.recession_depth.metadata().clone());
            channels.push(aerothermal.gas_mdot.metadata().clone());
            channels.push(aerothermal.mass_loss.metadata().clone());
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
    document: &ScenarioDocument,
    table: &mut TelemetryTable,
    kernel: &SimulationKernel<RigidBodyState, I, F, RigidModels<MOM, MM>, E, SC>,
    channels: &RigidChannelSet,
    breakdown_vehicle: &KernelVehicle<RigidBodyState>,
    breakdown_atmosphere: Option<&RuntimeAtmosphere>,
    aerothermal: Option<&crate::aerothermal::LiveAerothermalOutput>,
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

    insert_separated_body_channels(
        &mut row,
        kernel.separated_rigid_bodies(),
        &channels.separated_bodies,
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
        active_body: kernel.primary_rigid_body(),
        phase_id: kernel.current_phase().map(|phase| phase.value()),
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

    if let Some(channel) = &channels.active_models {
        row.insert(
            channel,
            crate::active_model_label(document, kernel.current_phase().map(|phase| phase.value())),
        )?;
    }
    if let Some(aerothermal_channels) = &channels.aerothermal {
        let sample = aerothermal.copied().unwrap_or_default();
        row.insert(&aerothermal_channels.q_conv, sample.q_conv_w_m2)?;
        row.insert(&aerothermal_channels.q_rad, sample.q_rad_w_m2)?;
        row.insert(&aerothermal_channels.h_aw, sample.h_aw_j_kg)?;
        row.insert(
            &aerothermal_channels.recovery_temperature,
            sample.recovery_temperature_k,
        )?;
        row.insert(
            &aerothermal_channels.stagnation_temperature,
            sample.stagnation_temperature_k,
        )?;
        row.insert(&aerothermal_channels.knudsen, sample.knudsen)?;
        row.insert(
            &aerothermal_channels.wall_temperature,
            sample.wall_temperature_k,
        )?;
        row.insert(
            &aerothermal_channels.backwall_temperature,
            sample.backwall_temperature_k,
        )?;
        row.insert(
            &aerothermal_channels.recession_depth,
            sample.recession_depth_m,
        )?;
        row.insert(&aerothermal_channels.gas_mdot, sample.gas_mdot_kg_m2_s)?;
        row.insert(&aerothermal_channels.mass_loss, sample.mass_loss_kg_s)?;
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

fn insert_separated_body_channels(
    row: &mut TelemetryRow,
    separated_bodies: &[openbmp_sim::SeparatedRigidBody],
    channels: &[SeparatedBodyTelemetryChannels],
) -> Result<(), RunnerError> {
    for channel in channels {
        let separated = separated_bodies
            .iter()
            .find(|body| body.body == channel.body)
            .map(|body| &body.state);
        row.insert(&channel.separated, separated.is_some())?;
        if let Some(state) = separated {
            row.insert(&channel.position_x, state.position.vector.x)?;
            row.insert(&channel.position_y, state.position.vector.y)?;
            row.insert(&channel.position_z, state.position.vector.z)?;
            row.insert(&channel.velocity_x, state.velocity.vector.x)?;
            row.insert(&channel.velocity_y, state.velocity.vector.y)?;
            row.insert(&channel.velocity_z, state.velocity.vector.z)?;
            row.insert(&channel.mass, state.mass_props.mass_kg())?;
            let raw = state.orientation.q.into_inner();
            row.insert(&channel.quaternion_x, raw.coords.x)?;
            row.insert(&channel.quaternion_y, raw.coords.y)?;
            row.insert(&channel.quaternion_z, raw.coords.z)?;
            row.insert(&channel.quaternion_w, raw.coords.w)?;
            row.insert(&channel.angular_velocity_x, state.angular_velocity.vector.x)?;
            row.insert(&channel.angular_velocity_y, state.angular_velocity.vector.y)?;
            row.insert(&channel.angular_velocity_z, state.angular_velocity.vector.z)?;
        } else {
            row.insert(&channel.position_x, 0.0)?;
            row.insert(&channel.position_y, 0.0)?;
            row.insert(&channel.position_z, 0.0)?;
            row.insert(&channel.velocity_x, 0.0)?;
            row.insert(&channel.velocity_y, 0.0)?;
            row.insert(&channel.velocity_z, 0.0)?;
            row.insert(&channel.mass, 0.0)?;
            row.insert(&channel.quaternion_x, 0.0)?;
            row.insert(&channel.quaternion_y, 0.0)?;
            row.insert(&channel.quaternion_z, 0.0)?;
            row.insert(&channel.quaternion_w, 1.0)?;
            row.insert(&channel.angular_velocity_x, 0.0)?;
            row.insert(&channel.angular_velocity_y, 0.0)?;
            row.insert(&channel.angular_velocity_z, 0.0)?;
        }
    }
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
    use openbmp_telemetry::TelemetryValue;

    #[test]
    fn combine_dry_body_adds_mass_weights_cg_and_sums_inertia() {
        let base = MassProperties::new(
            Mass::new::<kilogram>(6900.0),
            Position3::<Body>::new(0.0, 0.0, 16.0),
            nalgebra::Matrix3::from_diagonal(&nalgebra::Vector3::new(0.92e6, 0.92e6, 4.6e4)),
        );
        let add = MassProperties::new(
            Mass::new::<kilogram>(600.0),
            Position3::<Body>::new(0.0, 0.0, 16.0),
            nalgebra::Matrix3::from_diagonal(&nalgebra::Vector3::new(0.08e6, 0.08e6, 0.4e4)),
        );
        let c = combine_dry_body(base, add);
        // Mass adds.
        assert!((c.mass.get::<kilogram>() - 7500.0).abs() < 1.0e-9);
        // Shared CG is preserved (both at z=16).
        assert!((c.center_of_mass_body.vector.z - 16.0).abs() < 1.0e-9);
        // Inertia (about the shared origin) sums.
        assert!((c.inertia_body[(0, 0)] - 1.0e6).abs() < 1.0);
        assert!((c.inertia_body[(2, 2)] - 5.0e4).abs() < 1.0);

        // Distinct CGs: the combined CG is the mass-weighted centroid.
        let a2 = MassProperties::new(
            Mass::new::<kilogram>(100.0),
            Position3::<Body>::new(0.0, 0.0, 0.0),
            nalgebra::Matrix3::zeros(),
        );
        let b2 = MassProperties::new(
            Mass::new::<kilogram>(300.0),
            Position3::<Body>::new(0.0, 0.0, 4.0),
            nalgebra::Matrix3::zeros(),
        );
        let c2 = combine_dry_body(a2, b2);
        assert!((c2.mass.get::<kilogram>() - 400.0).abs() < 1.0e-9);
        // (100*0 + 300*4) / 400 = 3.0
        assert!((c2.center_of_mass_body.vector.z - 3.0).abs() < 1.0e-9);
    }

    const LIVE_ENTRY_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "live-entry-coupling-test"
description = "Synthetic live entry coupling regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.3
dt_s = 0.1
seed = 7

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 80000.0]
initial_velocity_eci_m_s = [7600.0, 0.0, -100.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "live-entry-coupling-test"

[[vehicle.assembly.bodies]]
id = "capsule"
geometry = { kind = "cylinder", length_m = 1.0, diameter_m = 1.0 }
dry_mass_kg = 100.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "piecewise_exponential"
wind = "none"

[atmosphere]
kind = "piecewise_exponential"

[aero]

[aero.method]
kind = "modified_newtonian"

[aero.method.modified_newtonian]
cp_max = 2.0
reference_area_m2 = 1.0
reference_length_m = 1.0

[aerothermal]
stagnation_kind = "sutton_graves"
nose_radius_m = 0.5
wall_temperature_k = 1500.0
wall_catalysis = "fully_catalytic"

[aerothermal.ablation]
virgin_material = "textbook_pica_like"
char_material = "textbook_char"
thickness_m = 0.05
n_nodes = 7
pyrolysis_enthalpy_j_kg = 2.4e6
gas_yield_fraction = 0.6
feedback = "mass"

[forces]
models = ["gravity"]

[[forces.phase_override]]
phase = "entry"
models = ["gravity", "aero", "aerothermal_diagnostics"]

[telemetry]
output.csv = "out/live-entry-coupling-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true

[mission]
initial_phase = "coast"

[[mission.phases]]
id = "coast"
label = "coast"

[[mission.phases]]
id = "entry"
label = "entry"

[[mission.events]]
id = "entry_interface"
trigger = { kind = "at_time", time_s = 0.1 }
action = { kind = "emit_telemetry_marker", tag = "entry" }
once = true

[[mission.transitions]]
from = "coast"
to = "entry"
event = "entry_interface"
"#;

    const BATCH_RV_DEPLOY_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "batch-rv-deploy-test"
description = "Synthetic bus deploying two rigid bodies on one event tick."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.3
dt_s = 0.1
seed = 19

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 100.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "batch-rv-deploy-test"

[[vehicle.assembly.bodies]]
id = "bus"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 3.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "rv1"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, -1.0]
dry_inertia_body_kg_m2 = [[0.2, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.0, 0.2]]

[[vehicle.assembly.bodies]]
id = "rv2"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 1.0]
dry_inertia_body_kg_m2 = [[0.2, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.0, 0.2]]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[mission]
initial_phase = "coast"

[[mission.phases]]
id = "coast"
label = "coast"

[[mission.events]]
id = "deploy_rvs"
trigger = { kind = "at_time", time_s = 0.1 }
action = { kind = "jettison_bodies", bodies = ["rv1", "rv2"] }
once = true

[[mission.events]]
id = "rv1_clear"
trigger = { kind = "at_relative_distance", body = "rv1", distance_m = 1.01 }
action = { kind = "emit_telemetry_marker", tag = "rv1_clear" }
once = true

[multi_body]

[[multi_body.separation]]
event_id = "deploy_rvs"
upper_body_id = "bus"
lower_body_id = "rv1"
lower_delta_v_body_m_s = [0.0, 1.0, 0.0]
conserve_momentum = false

[[multi_body.separation]]
event_id = "deploy_rvs"
upper_body_id = "bus"
lower_body_id = "rv2"
lower_delta_v_body_m_s = [0.0, -1.0, 0.0]
conserve_momentum = false

[telemetry]
output.csv = "out/batch-rv-deploy-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const INITIAL_MULTI_BODY_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "initial-multi-body-test"
description = "Synthetic two rigid bodies active from simulation start."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.3
dt_s = 0.1
seed = 23

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 100.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "initial-multi-body-test"

[[vehicle.assembly.bodies]]
id = "bus"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 3.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "observer"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[0.2, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.0, 0.2]]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[mission]
initial_phase = "coast"

[[mission.phases]]
id = "coast"
label = "coast"

[[mission.events]]
id = "observer_clear"
trigger = { kind = "at_relative_distance", body = "observer", distance_m = 1.01 }
action = { kind = "emit_telemetry_marker", tag = "observer_clear" }
once = true

[multi_body]
primary_body_id = "bus"

[[multi_body.initial_lane]]
body_id = "observer"
position_eci_m = [1.0, 0.0, 100.0]
velocity_eci_m_s = [0.0, 1.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[telemetry]
output.csv = "out/initial-multi-body-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    fn valid_stage_separation_document() -> ScenarioDocument {
        openbmp_scenario::Scenario::from_toml_str(include_str!(
            "../../openbmp-scenario/tests/fixtures/stage-separation-valid.toml"
        ))
        .expect("fixture must parse")
        .document
    }

    fn channel_id(outcome: &RunOutcome, name: &str) -> openbmp_core::ChannelId {
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .find(|channel| channel.name == name)
            .unwrap_or_else(|| panic!("channel `{name}` must exist"))
            .id
    }

    fn f64_column(outcome: &RunOutcome, name: &str) -> Vec<f64> {
        let id = channel_id(outcome, name);
        outcome
            .table
            .rows()
            .iter()
            .map(|row| match row.get(id) {
                Some(TelemetryValue::Float64(value)) => *value,
                other => panic!("unexpected value in {name}: {other:?}"),
            })
            .collect()
    }

    fn text_column(outcome: &RunOutcome, name: &str) -> Vec<String> {
        let id = channel_id(outcome, name);
        outcome
            .table
            .rows()
            .iter()
            .map(|row| match row.get(id) {
                Some(TelemetryValue::Text(value)) => value.clone(),
                other => panic!("unexpected value in {name}: {other:?}"),
            })
            .collect()
    }

    fn bool_column(outcome: &RunOutcome, name: &str) -> Vec<bool> {
        let id = channel_id(outcome, name);
        outcome
            .table
            .rows()
            .iter()
            .map(|row| match row.get(id) {
                Some(TelemetryValue::Bool(value)) => *value,
                other => panic!("unexpected value in {name}: {other:?}"),
            })
            .collect()
    }

    #[test]
    fn live_entry_phase_turns_on_aero_heating_and_mass_feedback() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(LIVE_ENTRY_SCENARIO)
            .expect("live entry scenario must parse");
        let outcome = crate::run(&scenario).expect("live entry scenario must run");

        let force_aero_x = f64_column(&outcome, "force.aero.x_n");
        assert_eq!(force_aero_x[0].to_bits(), 0.0_f64.to_bits());
        assert!(
            force_aero_x.iter().skip(1).any(|value| value.abs() > 0.0),
            "aero force should activate after entry phase transition: {force_aero_x:?}"
        );
        for channel in [
            "force.aerothermal_diagnostics.x_n",
            "force.aerothermal_diagnostics.y_n",
            "force.aerothermal_diagnostics.z_n",
        ] {
            let force = f64_column(&outcome, channel);
            assert!(
                force
                    .iter()
                    .all(|value| value.to_bits() == 0.0_f64.to_bits()),
                "{channel} should be present and zero-force: {force:?}"
            );
        }

        let active = text_column(&outcome, "forces.active_models");
        assert_eq!(active[0], "gravity");
        assert!(
            active
                .iter()
                .skip(1)
                .any(|value| value == "gravity,aero,aerothermal_diagnostics"),
            "active model telemetry should show the entry override: {active:?}"
        );

        let heat_flux = f64_column(&outcome, "aerothermal.q_conv_w_m2");
        assert!(
            heat_flux.iter().any(|value| *value > 0.0),
            "live aerothermal driver should emit positive heating: {heat_flux:?}"
        );

        let stagnation_temperature = f64_column(&outcome, "aerothermal.stagnation_temperature_k");
        assert!(
            stagnation_temperature.iter().any(|value| *value > 0.0),
            "live aerothermal driver should emit stagnation temperature: {stagnation_temperature:?}"
        );

        let mass_loss = f64_column(&outcome, "mass.aerothermal_mass_loss_kg_s");
        assert!(
            mass_loss.iter().any(|value| *value > 0.0),
            "ablation mass feedback should be observable: {mass_loss:?}"
        );
    }

    #[test]
    fn batch_jettison_bodies_deploys_multiple_rigid_lanes() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(BATCH_RV_DEPLOY_SCENARIO)
            .expect("batch deploy scenario must parse");
        let outcome = crate::run(&scenario).expect("batch deploy scenario must run");

        let rv1_separated = bool_column(&outcome, "body.rv1.separated");
        let rv2_separated = bool_column(&outcome, "body.rv2.separated");
        assert!(rv1_separated.iter().any(|value| *value));
        assert!(rv2_separated.iter().any(|value| *value));

        let rv1_vy = f64_column(&outcome, "body.rv1.velocity_y_m_s");
        let rv2_vy = f64_column(&outcome, "body.rv2.velocity_y_m_s");
        assert!(rv1_vy.iter().any(|value| (*value - 1.0).abs() < 1.0e-12));
        assert!(rv2_vy.iter().any(|value| (*value + 1.0).abs() < 1.0e-12));

        let rv1_clear = bool_column(&outcome, "mission.marker.rv1_clear");
        assert!(
            rv1_clear.iter().any(|value| *value),
            "relative-distance event should mark when rv1 clears the bus: {rv1_clear:?}"
        );
    }

    #[test]
    fn initial_multi_body_lanes_run_from_step_zero() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(INITIAL_MULTI_BODY_SCENARIO)
            .expect("initial multi-body scenario must parse");
        let outcome = crate::run(&scenario).expect("initial multi-body scenario must run");

        let observer_active = bool_column(&outcome, "body.observer.separated");
        assert_eq!(observer_active.first().copied(), Some(true));
        assert!(observer_active.iter().all(|value| *value));

        let bus_mass = f64_column(&outcome, "mass_kg");
        assert_eq!(bus_mass[0].to_bits(), 3.0_f64.to_bits());

        let observer_x = f64_column(&outcome, "body.observer.position_x_m");
        let observer_y = f64_column(&outcome, "body.observer.position_y_m");
        assert_eq!(observer_x[0].to_bits(), 1.0_f64.to_bits());
        assert!(
            observer_y
                .iter()
                .any(|value| (*value - 0.3).abs() < 1.0e-12),
            "observer lane should advance independently from t=0: {observer_y:?}"
        );

        let marker = bool_column(&outcome, "mission.marker.observer_clear");
        assert!(
            marker.iter().any(|value| *value),
            "relative-distance event should observe initially active lanes: {marker:?}"
        );
    }

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

    #[test]
    fn rigid_mass_resources_filter_engine_mass_by_active_body() {
        let upper = body_id_from_scenario_text("upper");
        let lower = body_id_from_scenario_text("lower");
        let engine_upper = EngineId::from_path("vehicle.assembly.engines.upper");
        let engine_lower = EngineId::from_path("vehicle.assembly.engines.lower");
        let dry_bodies = BTreeMap::from([
            (
                upper,
                MassProperties::with_uniform_inertia(
                    Mass::new::<kilogram>(4.0),
                    Position3::<Body>::origin(),
                    1.0,
                ),
            ),
            (
                lower,
                MassProperties::with_uniform_inertia(
                    Mass::new::<kilogram>(1.0),
                    Position3::<Body>::origin(),
                    0.25,
                ),
            ),
        ]);
        let resources = RigidMassResources {
            dry_total: MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(5.0),
                Position3::<Body>::origin(),
                1.25,
            ),
            dry_bodies,
            motor_owner: None,
            motor_ignition_time_s: None,
            engine_ids: vec![engine_upper, engine_lower],
            engine_owners: BTreeMap::from([(engine_upper, upper), (engine_lower, lower)]),
            tank_coupled_engines: std::collections::BTreeSet::new(),
            tank_routes: BTreeMap::new(),
        };
        let model =
            RigidMassResourceModel::<SolidMotor>::new(resources, None, None, ModelId::new(999));
        let engine_snapshot = BTreeMap::from([
            (
                engine_upper,
                openbmp_sim::EngineSnapshot {
                    thrust_body: Vector3::zeros(),
                    mass_flow_kg_per_s: 1.0,
                    consumed_kg: 0.25,
                    lifecycle_state_index: 2,
                },
            ),
            (
                engine_lower,
                openbmp_sim::EngineSnapshot {
                    thrust_body: Vector3::zeros(),
                    mass_flow_kg_per_s: 2.0,
                    consumed_kg: 0.75,
                    lifecycle_state_index: 2,
                },
            ),
        ]);
        let tanks = BTreeMap::new();

        let upper_props = model
            .mass_properties_from_snapshots(SimTime::ZERO, Some(upper), &engine_snapshot, &tanks)
            .unwrap();
        let lower_props = model
            .mass_properties_from_snapshots(SimTime::ZERO, Some(lower), &engine_snapshot, &tanks)
            .unwrap();
        let all_props = model
            .mass_properties_from_snapshots(SimTime::ZERO, None, &engine_snapshot, &tanks)
            .unwrap();
        assert!((upper_props.mass_kg() - 3.75).abs() < 1.0e-12);
        assert!((lower_props.mass_kg() - 0.25).abs() < 1.0e-12);
        assert!((all_props.mass_kg() - 4.0).abs() < 1.0e-12);

        let upper_rate = model
            .mass_properties_rate_from_snapshots(SimTime::ZERO, Some(upper), &engine_snapshot)
            .unwrap();
        let lower_rate = model
            .mass_properties_rate_from_snapshots(SimTime::ZERO, Some(lower), &engine_snapshot)
            .unwrap();
        let all_rate = model
            .mass_properties_rate_from_snapshots(SimTime::ZERO, None, &engine_snapshot)
            .unwrap();
        assert_eq!(upper_rate.mass_rate_kg_s.to_bits(), (-1.0_f64).to_bits());
        assert_eq!(lower_rate.mass_rate_kg_s.to_bits(), (-2.0_f64).to_bits());
        assert_eq!(all_rate.mass_rate_kg_s.to_bits(), (-3.0_f64).to_bits());
    }

    #[test]
    fn tank_coupled_engine_mass_is_not_double_counted() {
        // An engine that draws from a tank must NOT also subtract its
        // integrated consumed_kg in the absolute mass path — the tank's
        // mass_kg already reflects depletion. The mass RATE, which has
        // no tank term, still uses -mass_flow. Regression for a
        // negative departing-stage mass at separation.
        let body = body_id_from_scenario_text("core");
        let engine = EngineId::from_path("vehicle.assembly.engines.main");
        let tank = TankId::from_path("vehicle.assembly.tanks.prop");
        let dry_bodies = BTreeMap::from([(
            body,
            MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(20.0),
                Position3::<Body>::origin(),
                1.0,
            ),
        )]);
        let resources = RigidMassResources {
            dry_total: MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(20.0),
                Position3::<Body>::origin(),
                1.0,
            ),
            dry_bodies,
            motor_owner: None,
            motor_ignition_time_s: None,
            engine_ids: vec![engine],
            engine_owners: BTreeMap::from([(engine, body)]),
            tank_coupled_engines: std::collections::BTreeSet::from([engine]),
            tank_routes: BTreeMap::from([(
                tank,
                TankMassRoute {
                    owner: body,
                    mount_point_body_m: Vector3::zeros(),
                },
            )]),
        };
        let model =
            RigidMassResourceModel::<SolidMotor>::new(resources, None, None, ModelId::new(999));
        // Engine has burned 5 kg (consumed_kg); tank retains 70 kg.
        let engine_snapshot = BTreeMap::from([(
            engine,
            openbmp_sim::EngineSnapshot {
                thrust_body: Vector3::zeros(),
                mass_flow_kg_per_s: 3.0,
                consumed_kg: 5.0,
                lifecycle_state_index: 2,
            },
        )]);
        let tank_snapshot = BTreeMap::from([(
            tank,
            openbmp_sim::TankSnapshot {
                mass_kg: 70.0,
                cg_offset_body_m: Vector3::zeros(),
                inertia_delta_body_kg_m2: nalgebra::Matrix3::zeros(),
                reaction_force_body_n: Vector3::zeros(),
                reaction_moment_body_n_m: Vector3::zeros(),
                fluid_remaining_kg: 70.0,
            },
        )]);

        // Absolute mass = dry(20) + tank(70), with consumed_kg (5) NOT
        // subtracted. The buggy path would give 85.
        let props = model
            .mass_properties_from_snapshots(
                SimTime::ZERO,
                Some(body),
                &engine_snapshot,
                &tank_snapshot,
            )
            .unwrap();
        assert!((props.mass_kg() - 90.0).abs() < 1.0e-9);

        // Rate still tracks the engine mass-flow (the tank drain).
        let rate = model
            .mass_properties_rate_from_snapshots(SimTime::ZERO, Some(body), &engine_snapshot)
            .unwrap();
        assert_eq!(rate.mass_rate_kg_s.to_bits(), (-3.0_f64).to_bits());
    }

    #[test]
    fn multi_body_allows_owned_non_gravity_force_models_but_still_requires_rk4() {
        let mut document = valid_stage_separation_document();
        document.forces = Some(openbmp_scenario::ForcesConfig {
            models: vec!["gravity".to_owned(), "aero".to_owned()],
            phase_override: Vec::new(),
        });
        document.aero = Some(openbmp_scenario::AeroConfig {
            deck: Some("aero.csv".into()),
            buildup: None,
            method: None,
            mounted_to: Some("upper".to_owned()),
            deck_sha256: None,
        });
        require_supported_multi_body_shape(&document).expect("owned non-gravity force is allowed");

        document.solver = Some(openbmp_scenario::SolverConfig {
            profile: Some("adaptive-explicit".to_owned()),
            trajectory_method: Some("dopri853".to_owned()),
            determinism: None,
            adaptive: None,
            source_terms: None,
        });
        let err = require_supported_multi_body_shape(&document).unwrap_err();
        match err {
            RunnerError::UnsupportedScenario { what } => {
                assert!(what.contains("fixed-step RK4"));
                assert!(what.contains("adaptive-explicit"));
            }
            other => panic!("expected UnsupportedScenario, got {other:?}"),
        }
    }
}
