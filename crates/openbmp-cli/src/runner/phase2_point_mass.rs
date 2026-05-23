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
//! - **Recovery state** when `[[vehicle.assembly.recovery]]` is
//!   declared: `recovery.<id>.deployed`, `.phase_index`, and
//!   `.drag_area_m2`.
//!
//! The schema also carries the resolved-file SHA-256 digests as
//! Arrow schema metadata under `openbmp.scenario_files.<field>` keys.

use std::collections::BTreeMap;

use nalgebra::Vector3;
use openbmp_aero::{AeroDeck, AeroError};
use openbmp_core::{ChannelId, Duration, ModelId, Position3, RecoveryId, SimTime, Velocity3};
use openbmp_physics::{
    AtmosphereModel, Egm2008ZonalGravity, J2Gravity, PointMassGravity, WGS84_J2,
};
use openbmp_propulsion::{Motor, MotorError, SolidMotor};
use openbmp_scenario::{ResolvedFile, Scenario, ScenarioDocument};
use openbmp_sim::{
    ConstantGravityForce, ConstantMass, EndTime, ForceContext, ForceModel, MassModel,
    NullEnvironment, ScenarioScriptAction, SimulationConfig, SimulationKernel, StopReason,
};
use openbmp_state::PointMassState;
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use openbmp_vehicle::{
    BoxedMassModel, DeckDragForceAdapter, EngineClusterForceAdapter, EngineClusterMassAdapter,
    GravityForceAdapter, KernelVehicle, MotorMassAdapter, MotorThrustForceAdapter, NamedForceModel,
    RecoveryRackForceAdapter, TankRackForceAdapter, TankRackMassAdapter, Vehicle,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::CliError;
use crate::runner::RunOutcome;
use crate::runner::assembly::dry_mass_kg_at;
use crate::runner::atmosphere::{
    RuntimeAtmosphere, build_runtime_atmosphere, is_runtime_atmosphere_kind,
    scenario_atmosphere_kind,
};
use crate::runner::integrator::build_runtime_integrator;
use openbmp_vehicle::Assembly;

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
// Phase-5.C.2 — non-constant gravity (point_mass / j2 / egm2008) routes
// through `GravityForceAdapter`, which keys its per-model telemetry
// stream on this id. The legacy `constant` arm continues to use
// `ConstantGravityForce` (a Phase-1-shaped model with no model id) so
// every existing constant-gravity scenario stays byte-stable.
const PHASE2_GRAVITY_MODEL_ID: ModelId = ModelId::new(105);
// Phase-3.6: distinct model ids for the engine-cluster path so the
// determinism oracle can tell legacy single-motor scenarios apart
// from cluster scenarios in the per-model force breakdown.
const PHASE3_ENGINE_CLUSTER_THRUST_MODEL_ID: ModelId = ModelId::new(120);
const PHASE3_ENGINE_CLUSTER_MASS_MODEL_ID: ModelId = ModelId::new(121);
const PHASE3_TANK_RACK_FORCE_MODEL_ID: ModelId = ModelId::new(330);
const PHASE3_TANK_RACK_MASS_MODEL_ID: ModelId = ModelId::new(331);
const PHASE3_RECOVERY_RACK_FORCE_MODEL_ID: ModelId = ModelId::new(370);

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
#[allow(clippy::too_many_lines)] // Phase-3.6: per-step orchestration grew
pub fn run(
    scenario: &Scenario,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<RunOutcome, CliError> {
    let document = &scenario.document;
    require_supported_shape(document)?;
    let assembly = crate::runner::assembly::synthesize_assembly(document)?;
    // Phase-3.4: build the runner-side effector rack. Empty when
    // no `[[vehicle.assembly.effectors]]` are declared, in which
    // case every per-step rack operation short-circuits and the
    // legacy byte-stable kernel path is preserved.
    let mut effector_rack = crate::runner::effectors::EffectorRack::build(document)?;
    // Phase-3.6: build the runner-side engine rack. Empty when no
    // `[[vehicle.assembly.engines]]` are declared, in which case
    // every per-step rack operation short-circuits and the legacy
    // single-motor byte-stable path is preserved.
    let mut engine_rack = crate::runner::engines::EngineRack::build(document)?;
    // Phase-3.7: build the runner-side tank rack. Empty when no
    // `[[vehicle.assembly.tanks]]` are declared, in which case every
    // per-step rack operation short-circuits and the legacy
    // byte-stable path is preserved.
    let mut tank_rack = crate::runner::tanks::TankRack::build(document)?;
    // Phase-3.9: build the runner-side recovery rack. Empty when no
    // `[[vehicle.assembly.recovery]]` are declared, in which case
    // every per-step rack operation short-circuits and the kernel's
    // recovery snapshot stays empty (byte-stable for pre-3.9
    // scenarios).
    let mut recovery_rack = crate::runner::recovery::RecoveryRack::build(document)?;
    // Phase-3.8: build the runner-side wind rack. Inactive when no
    // `[wind]` block is declared (or `kind = "none"`); the per-step
    // rack op short-circuits and the kernel's wind override stays
    // `None`, preserving pre-3.8 byte output.
    let wind_rack = crate::runner::wind::WindRack::build(document)?;
    wind_rack.reset();

    let loaded_models = load_models(document, resolved_files)?;
    let initial_state = build_initial_state(document, &loaded_models, &assembly)?;
    let kernel_vehicle = build_vehicle(document, &loaded_models, &assembly)?;
    // The runner-side breakdown vehicle is a *separate* construction
    // of the same models. `KernelVehicle::evaluate_force_breakdown`
    // takes `&self`, but `KernelVehicle` is not `Clone` (the inner
    // `Box<dyn ForceModel>` lists are not). Re-building from scratch
    // avoids interior-mutability or Arc gymnastics; both copies are
    // stateless and evaluate identically per the Phase-2.6/2.5
    // contracts.
    let breakdown_vehicle = build_vehicle(document, &loaded_models, &assembly)?;
    let mass_model = BoxedMassModel(build_mass_model(document, &loaded_models, &assembly)?);

    // Phase-5.D.4 — runtime integrator dispatch from the scenario
    // [solver] block. Defaults to Rk4FixedStep when [solver] is
    // absent, preserving the byte-stable Phase-1 contract for every
    // existing scenario.
    let runtime_integrator = build_runtime_integrator(document)?;
    let config = SimulationConfig {
        initial_state,
        integrator: runtime_integrator,
        force_model: kernel_vehicle,
        mass_model,
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(document.time.stop_s)),
        dt: Duration::from_seconds(document.time.dt_s),
        scenario_seed: document.time.seed,
    };

    let kernel_base = SimulationKernel::new(config)?;
    let mut kernel = if let Some(mission) = &document.mission {
        let mission_runtime = crate::runner::mission::build_mission_runtime_typed(mission)?;
        kernel_base.with_mission_split(
            mission_runtime.mission_bindings,
            mission_runtime.script_bindings,
            Some(mission_runtime.graph),
            Some(mission_runtime.hsm),
        )?
    } else {
        kernel_base
    };
    let channel_set = Phase2ChannelSet::new(document)?;
    let breakdown_atmosphere = if channel_set.has_atmosphere {
        Some(build_runtime_atmosphere(scenario_atmosphere_kind(
            document,
        ))?)
    } else {
        None
    };
    let metadata = build_schema_metadata(resolved_files);
    let mut table = TelemetryTable::new(channel_set.schema(metadata)?);

    // Phase-3.5.C: pair schema-2 deck axes with scenario effectors.
    // Schema-1 decks and decks without effector axes produce empty
    // bindings; the per-step snapshot push then short-circuits and
    // the kernel's effector-actuals map stays empty (byte-stable).
    let deck_bindings = crate::runner::aero_effector_match::assert_axes_match_effectors(
        loaded_models.aero_deck.as_ref(),
        document,
    )?;

    // Step 0 has no fired events. The effector snapshot at step 0 is
    // each effector's load-time at-rest state (initial position).
    let initial_snapshot = effector_rack.snapshot();
    if !deck_bindings.is_empty() {
        let snapshot_map = crate::runner::aero_effector_match::build_snapshot_map(
            &deck_bindings,
            &initial_snapshot,
        );
        kernel.set_effector_actuals(snapshot_map);
    }
    // Phase-3.6: at step 0, push the rack's initial (Idle) snapshot
    // to the kernel so the breakdown's mass adapter sees the same
    // empty-consumption view the kernel will see on its first step.
    if !engine_rack.is_empty() {
        kernel.set_engine_snapshot(engine_rack.snapshot_map());
    }
    // Phase-3.7: at step 0, push the rack's initial snapshot to the
    // kernel so the breakdown's mass adapter and force adapter see
    // the same view the kernel will see.
    if !tank_rack.is_empty() {
        kernel.set_tank_snapshot(tank_rack.snapshot_map());
    }
    // Phase-3.9: at step 0, push the rack's initial (Stowed)
    // snapshot to the kernel so the breakdown's recovery-drag force
    // adapter sees the same view the kernel will see.
    if !recovery_rack.is_empty() {
        kernel.set_recovery_snapshot(recovery_rack.snapshot_map());
    }
    // Phase-3.8: at step 0, sample the wind at the initial state
    // and push to the kernel so the breakdown's force adapter sees
    // the same wind the kernel will see on its first step.
    if !wind_rack.is_inactive() {
        let initial_state = kernel.current_state();
        let frame = openbmp_physics::FrameContext::toy_fixed_earth();
        let wind = wind_rack.sample(initial_state.position, &frame, initial_state.time)?;
        kernel.set_wind_sample(wind);
    }
    let mut fc_bridge = crate::runner::fc_bridge::FcBridge::maybe_new(scenario, resolved_files)?;
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
        // Phase-3.6: drain pending engine commands from the previous
        // kernel step, apply to the rack, then advance the rack.
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
            bridge.tick_point_mass(
                kernel.current_state(),
                kernel.current_step(),
                gravity,
                &mut effector_rack,
                &mut engine_rack,
            )?;
            // Phase 5.X.B: forward the FC commander's published
            // mission state into the kernel's external view. The
            // kernel uses the externally-supplied state in
            // preference to its internal current_phase during event
            // evaluation.
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
        // Phase-3.7: advance the tank rack using prior-step cached
        // drivers (set after the previous kernel step). For point-
        // mass kernels the drivers are zeros — slosh in point-mass
        // is RigidLiquid-only per the scenario validator (D10), so
        // the dynamic drivers are irrelevant.
        if !tank_rack.is_empty() {
            tank_rack.step()?;
        }
        // Phase-3.9: drain pending deploy/stow events from the prior
        // kernel step, apply to the rack, then advance internal state
        // (no-op for the Phase-3.9 instantaneous-deploy models).
        if !recovery_rack.is_empty() {
            recovery_rack.apply_deploys(&pending_recovery_events)?;
            recovery_rack.step(document.time.dt_s)?;
        }
        // Phase-3.5.C: push the rack's actuals snapshot to the kernel
        // BEFORE `step()` so all four RK4 stages see the same view.
        // Empty bindings → zero allocation, zero state change.
        if !deck_bindings.is_empty() {
            let rack_snapshot = effector_rack.snapshot();
            let snapshot_map = crate::runner::aero_effector_match::build_snapshot_map(
                &deck_bindings,
                &rack_snapshot,
            );
            kernel.set_effector_actuals(snapshot_map);
        }
        // Phase-3.6: push the rack's engine snapshot to the kernel
        // BEFORE `step()` so all four RK4 stages see the same view.
        // Empty rack → zero allocation, zero state change (the
        // kernel's `engine_snapshot` field stays at the empty
        // `BTreeMap` set in `new()`).
        if !engine_rack.is_empty() {
            kernel.set_engine_snapshot(engine_rack.snapshot_map());
        }
        // Phase-3.7: push tank snapshot to the kernel before
        // `step()` so all four RK4 stages see the same view.
        if !tank_rack.is_empty() {
            kernel.set_tank_snapshot(tank_rack.snapshot_map());
        }
        // Phase-3.9: push recovery snapshot to the kernel before
        // `step()` so all four RK4 stages see the same view.
        if !recovery_rack.is_empty() {
            kernel.set_recovery_snapshot(recovery_rack.snapshot_map());
        }
        // Phase-3.8: advance the wind rack and push the new sample
        // to the kernel before `step()`. For `GustWind` this rolls
        // the Dryden filter forward by one tick; the time-only
        // models are no-ops.
        if !wind_rack.is_inactive() {
            wind_rack.advance(kernel.current_step());
            let s = kernel.current_state();
            let frame = openbmp_physics::FrameContext::toy_fixed_earth();
            let wind = wind_rack.sample(s.position, &frame, s.time)?;
            kernel.set_wind_sample(wind);
        }
        kernel.step()?;
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
        // Phase 5.X.E: partition the typed script-action fired
        // queue (engine commands -> engine rack, recovery deploys
        // -> recovery rack, effector overrides -> effector rack).
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

fn require_supported_shape(document: &ScenarioDocument) -> Result<(), CliError> {
    if document.vehicle.kind != "point_mass" {
        return Err(CliError::UnsupportedScenario {
            what: format!("vehicle.kind = {}", document.vehicle.kind),
        });
    }
    if !matches!(
        document.environment.gravity.as_str(),
        "constant" | "point_mass" | "j2" | "egm2008"
    ) {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "environment.gravity = {} (wired: constant, point_mass, j2, egm2008)",
                document.environment.gravity
            ),
        });
    }

    // Force list: subset of {gravity, aero, thrust}, scenario-declared
    // order is the determinism contract.
    for name in document.force_models() {
        if !matches!(name.as_str(), "gravity" | "aero" | "thrust") {
            return Err(CliError::UnsupportedScenario {
                what: format!("forces.models entry `{name}` (only gravity, aero, thrust wired)"),
            });
        }
    }

    // Phase-3.8 wind models are resolved by WindRack. Scenario
    // validation guarantees that non-`none` flat selections carry a
    // structured `[wind]` block and that the kind names agree.

    // Atmosphere: when `aero` is in the force list, require one of the
    // wired layered atmospheres (USSA76 for the historical sounding-
    // rocket envelope, piecewise-exponential for the Phase-5.C.1
    // engineering 0-1000 km envelope).
    let atmosphere_kind = scenario_atmosphere_kind(document);
    let has_aero = document.force_models().iter().any(|m| m == "aero");
    if has_aero && !is_runtime_atmosphere_kind(atmosphere_kind) {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "atmosphere `{atmosphere_kind}` is not wired with the aero force; \
                 use `us_standard_1976` or `piecewise_exponential`"
            ),
        });
    }
    let has_recovery = !document.vehicle.assembly.recovery.is_empty();
    if has_recovery && !is_runtime_atmosphere_kind(atmosphere_kind) {
        return Err(CliError::UnsupportedScenario {
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
    assembly: &Assembly,
) -> Result<PointMassState, CliError> {
    let p = document.vehicle.initial_position_eci_m;
    let v = document.vehicle.initial_velocity_eci_m_s;
    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_mass_kg = dry_mass_kg_at(assembly, start_time, "vehicle.assembly")?;

    // Total mass at the initial state = assembly dry mass PLUS the
    // motor's current mass when a motor is declared. The assembly dry
    // mass is the dry-airframe mass (no motor). MotorMassAdapter
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

/// Construct the gravity-force adapter for the runtime gravity model
/// declared by the scenario.
///
/// The `constant` arm intentionally retains the legacy
/// `ConstantGravityForce` (Phase-1-shaped) to keep byte-for-byte
/// reproducibility on every existing point-mass scenario. The new arms
/// (`point_mass`, `j2`, `egm2008` from Phase 5.C.2) route through
/// `GravityForceAdapter`, which wraps the corresponding
/// `openbmp_physics::GravityModel`.
fn build_gravity_force_adapter_point_mass(
    document: &ScenarioDocument,
) -> Result<Box<dyn ForceModel<PointMassState> + Send + Sync>, CliError> {
    match document.environment.gravity.as_str() {
        "constant" => {
            let g =
                document
                    .environment
                    .gravity_m_s2
                    .ok_or_else(|| CliError::UnsupportedScenario {
                        what: "environment.gravity_m_s2 missing for constant gravity".to_owned(),
                    })?;
            if g < 0.0 {
                return Err(CliError::UnsupportedScenario {
                    what: "environment.gravity_m_s2 must be a non-negative magnitude; \
                         constant gravity is -z in ECI"
                        .to_owned(),
                });
            }
            // Legacy Phase-1 force model — retained verbatim to keep
            // byte-stable Parquet on every existing constant-gravity
            // point-mass scenario.
            Ok(Box::new(ConstantGravityForce::new(Vector3::new(
                0.0, 0.0, -g,
            ))))
        }
        "point_mass" => {
            let mu =
                document
                    .environment
                    .mu_m3_s2
                    .ok_or_else(|| CliError::UnsupportedScenario {
                        what: "environment.mu_m3_s2 missing for point_mass gravity".to_owned(),
                    })?;
            let model = PointMassGravity::new(mu)?;
            Ok(Box::new(GravityForceAdapter::new(
                model,
                PHASE2_GRAVITY_MODEL_ID,
            )))
        }
        "j2" => {
            let mu =
                document
                    .environment
                    .mu_m3_s2
                    .ok_or_else(|| CliError::UnsupportedScenario {
                        what: "environment.mu_m3_s2 missing for j2 gravity".to_owned(),
                    })?;
            let r_e = document
                .environment
                .r_e_m
                .ok_or_else(|| CliError::UnsupportedScenario {
                    what: "environment.r_e_m missing for j2 gravity".to_owned(),
                })?;
            let j2 = document.environment.j2.unwrap_or(WGS84_J2);
            let model = J2Gravity::new(mu, r_e, j2)?;
            Ok(Box::new(GravityForceAdapter::new(
                model,
                PHASE2_GRAVITY_MODEL_ID,
            )))
        }
        "egm2008" => {
            // Phase 5.C.2: zonal-only EGM2008 (degrees 2-6), pinned to
            // WGS84 µ / R_e and the Pavlis et al. 2012 J_n table. No
            // per-scenario overrides are accepted, matching the parser
            // contract in `EnvironmentConfig::validate`.
            let model = Egm2008ZonalGravity::wgs84_egm2008_zonal();
            Ok(Box::new(GravityForceAdapter::new(
                model,
                PHASE2_GRAVITY_MODEL_ID,
            )))
        }
        other => Err(CliError::UnsupportedScenario {
            what: format!("environment.gravity = {other} is not wired"),
        }),
    }
}

#[allow(clippy::too_many_lines)] // Phase-3.9 added the recovery-rack force-adapter wiring branch
fn build_vehicle(
    document: &ScenarioDocument,
    loaded_models: &LoadedModels,
    assembly: &Assembly,
) -> Result<KernelVehicle<PointMassState>, CliError> {
    let mut named: Vec<NamedForceModel<PointMassState>> = Vec::new();

    for name in document.force_models() {
        match name.as_str() {
            "gravity" => {
                let force = build_gravity_force_adapter_point_mass(document)?;
                named.push(NamedForceModel::new("gravity", force));
            }
            "aero" => {
                let deck = loaded_models.aero_deck.clone().ok_or_else(|| {
                    CliError::UnsupportedScenario {
                        what: "forces includes `aero` but [aero] block is missing".to_owned(),
                    }
                })?;
                let atmosphere = build_runtime_atmosphere(scenario_atmosphere_kind(document))?;
                let drag = DeckDragForceAdapter::new(deck, atmosphere, PHASE2_AERO_MODEL_ID);
                named.push(NamedForceModel::new("aero", Box::new(drag)));
            }
            "thrust" => {
                // Phase-3.6: dispatch between single-motor (legacy)
                // and engine-cluster paths based on whether
                // `[[vehicle.assembly.engines]]` is declared. The
                // scenario validator rejects scenarios that declare
                // both blocks (`AmbiguousPropulsion`), so exactly
                // one path resolves.
                if document.vehicle.assembly.engines.is_empty() {
                    let motor = loaded_models.motor.clone().ok_or_else(|| {
                        CliError::UnsupportedScenario {
                            what: "forces includes `thrust` but neither [propulsion.motor] nor \
                                   [[vehicle.assembly.engines]] is declared"
                                .to_owned(),
                        }
                    })?;
                    let ignition_time_s = motor_ignition_time_s(document)?;
                    let thrust = MotorThrustForceAdapter::new(
                        motor,
                        ignition_time_s,
                        PHASE2_THRUST_MODEL_ID,
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
                        PHASE3_ENGINE_CLUSTER_THRUST_MODEL_ID,
                    );
                    named.push(NamedForceModel::new("thrust", Box::new(thrust)));
                }
            }
            other => unreachable!("require_supported_shape rejects unknown force model `{other}`"),
        }
    }

    // Phase-3.7: tank-rack reaction-force adapter (when tanks
    // declared). Last in the named list so the locked left-fold
    // operand order keeps prior force entries unchanged.
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
        let tank_force = TankRackForceAdapter::new(tank_ids, PHASE3_TANK_RACK_FORCE_MODEL_ID);
        named.push(NamedForceModel::new("tank_reaction", Box::new(tank_force)));
    }

    // Phase-3.9: recovery-rack drag-force adapter (when recovery
    // devices declared). Appended last so legacy force-list
    // summation order is unchanged for pre-3.9 scenarios; the locked
    // left-fold places recovery drag at the end of the breakdown.
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
        let recovery_force = RecoveryRackForceAdapter::new(
            recovery_ids,
            atmosphere,
            PHASE3_RECOVERY_RACK_FORCE_MODEL_ID,
        );
        named.push(NamedForceModel::new(
            "recovery_drag",
            Box::new(recovery_force),
        ));
    }

    // KernelVehicle requires a mass model even for vehicle-internal
    // queries (Phase-2.8 contract). The kernel's mass model is built
    // separately in `build_mass_model` because it owns its own copy.
    let vehicle_mass = build_mass_model(document, loaded_models, assembly)?;
    KernelVehicle::new(named, vec![], vehicle_mass).map_err(|e| CliError::UnsupportedScenario {
        what: format!("KernelVehicle construction failed: {e}"),
    })
}

fn build_mass_model(
    document: &ScenarioDocument,
    loaded_models: &LoadedModels,
    assembly: &Assembly,
) -> Result<Box<dyn MassModel>, CliError> {
    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_mass_kg = dry_mass_kg_at(assembly, start_time, "vehicle.assembly")?;
    // Phase-3.6: dispatch to the cluster mass adapter when
    // `[[vehicle.assembly.engines]]` is declared. Scenarios with
    // both motor and engines are rejected at parse time
    // (`AmbiguousPropulsion`), so the three arms are exclusive.
    let base: Box<dyn MassModel> = if !document.vehicle.assembly.engines.is_empty() {
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
            PHASE3_ENGINE_CLUSTER_MASS_MODEL_ID,
        ))
    } else if let Some(motor) = &loaded_models.motor {
        let ignition_time_s = motor_ignition_time_s(document)?;
        Box::new(MotorMassAdapter::new(
            motor.clone(),
            dry_mass_kg,
            ignition_time_s,
            PHASE2_MOTOR_MASS_MODEL_ID,
        ))
    } else {
        Box::new(ConstantMass::new(dry_mass_kg))
    };

    // Phase-3.7: wrap the base mass model with a tank-rack mass
    // adapter when tanks are declared. The wrapper adds each tank's
    // `mass_kg` from the kernel snapshot to the base mass.
    if document.vehicle.assembly.tanks.is_empty() {
        Ok(base)
    } else {
        let tank_ids: Vec<openbmp_core::TankId> = document
            .vehicle
            .assembly
            .tanks
            .iter()
            .map(|t| {
                openbmp_core::TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = t.id))
            })
            .collect();
        Ok(Box::new(TankRackMassAdapter::new(
            base,
            tank_ids,
            PHASE3_TANK_RACK_MASS_MODEL_ID,
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

/// Per-recovery telemetry channels in scenario-declared order.
/// Each entry is `(id, deployed, phase_index, drag_area)`.
type RecoveryTelemetryChannels = Vec<(
    RecoveryId,
    TelemetryChannel<bool>,
    TelemetryChannel<i64>,
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
    /// Phase-3.4 effector deflection channels, in scenario-declared
    /// order. One `effector.<id>.actual` `f64` channel per declared
    /// effector. Allocated AFTER force breakdown channels and BEFORE
    /// mission markers — this ordering is the determinism contract.
    effector_actuals: Vec<TelemetryChannel<f64>>,
    /// Phase-3.9 recovery-state channels, in scenario-declared order.
    /// Allocated after effectors and before mission markers so marker
    /// ordering remains alphabetical and recovery-free scenarios keep
    /// their legacy schema unchanged.
    recovery_states: RecoveryTelemetryChannels,
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

        // Atmosphere channels: emitted whenever the scenario declares
        // a layered atmosphere the runner can sample (USSA76 since
        // Phase 2.3, plus piecewise-exponential since Phase 5.C.1).
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

        // Per-model force breakdown channels, in scenario-declared
        // order — the same order the kernel uses for the RK4 sum.
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

        // Phase-3.4 effector deflection channels, in scenario-
        // declared order. One `effector.<id>.actual` channel per
        // declared effector. Allocated BEFORE mission markers so
        // adding effectors to a scenario does not shift marker
        // channel ids.
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

        // Phase-3.9 recovery telemetry channels, one triple per
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
            effector_actuals,
            recovery_states,
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
        // Phase-3.4 effector deflection channels, in scenario-declared
        // order. Allocated AFTER force breakdown channels and BEFORE
        // mission markers — this ordering is the determinism contract.
        for actual in &self.effector_actuals {
            channels.push(actual.metadata().clone());
        }
        // Phase-3.9 recovery channels, in scenario-declared order,
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
fn record_step<I, F, MM, E, SC>(
    table: &mut TelemetryTable,
    kernel: &SimulationKernel<PointMassState, I, F, MM, E, SC>,
    channels: &Phase2ChannelSet,
    breakdown_vehicle: &KernelVehicle<PointMassState>,
    breakdown_atmosphere: Option<&RuntimeAtmosphere>,
    fired_events: &[openbmp_sim::FiredEvent<openbmp_sim::MissionAction>],
    effector_snapshot: &[openbmp_vehicle::EffectorState],
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
    // +z as the altitude proxy, matching the DeckDragForceAdapter
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
    //
    // Phase-3.5.C: the breakdown's `effector_actuals` view mirrors
    // the kernel's snapshot via `kernel.effector_actuals()`. For
    // schema-1 scenarios this is the empty map and the breakdown is
    // byte-identical to pre-3.5; for schema-2 scenarios the
    // breakdown sees the same deflections the kernel just consumed.
    let env_sample = kernel.current_environment_sample()?;
    let kernel_actuals = kernel.effector_actuals();
    let kernel_engine_snapshot = kernel.engine_snapshot();
    let kernel_tank_snapshot = kernel.tank_snapshot();
    let kernel_recovery_snapshot = kernel.recovery_snapshot();
    let ctx = ForceContext {
        state,
        environment: &env_sample,
        mass_kg: state.mass.get::<kilogram>(),
        time: state.time,
        effector_actuals: openbmp_sim::EffectorActualsView::new(kernel_actuals),
        engine_snapshot: openbmp_sim::EngineSnapshotView::new(kernel_engine_snapshot),
        tank_snapshot: openbmp_sim::TankSnapshotView::new(kernel_tank_snapshot),
        recovery_snapshot: openbmp_sim::RecoverySnapshotView::new(kernel_recovery_snapshot),
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

    // Phase-3.4 effector deflection channels. The snapshot is in
    // scenario-declared order, matching `channels.effector_actuals`.
    // When the rack is empty (legacy scenarios) the snapshot is empty
    // and the loop is a no-op.
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

    // Phase-3.2 marker channels: write `true` for any tag whose
    // event fired this step, `false` for the rest. The marker
    // channel order is alphabetical (BTreeMap iteration); the
    // `fired_events` slice is in scenario-declared event order, so
    // a tag may match more than one fired event in a single step
    // (the row is `true` if any matched).
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
) -> Result<(), CliError> {
    for (id, deployed, phase_index, drag_area) in channels {
        let state = snapshot
            .get(id)
            .ok_or_else(|| CliError::UnsupportedScenario {
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

    fn parachute_scenario() -> Scenario {
        Scenario::from_file(
            workspace_root().join("scenarios/parachute-recovery/parachute-descent.toml"),
        )
        .expect("canonical parachute scenario must parse")
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

    fn first_row_value<'a>(outcome: &'a RunOutcome, name: &str) -> &'a TelemetryValue {
        let channel = outcome
            .table
            .schema()
            .channels()
            .iter()
            .find(|channel| channel.name == name)
            .unwrap_or_else(|| panic!("channel `{name}` must exist"));
        let row = outcome.table.rows().first().expect("initial row exists");
        row.get(channel.id)
            .unwrap_or_else(|| panic!("channel `{name}` must have an initial value"))
    }

    #[test]
    fn phase2_point_mass_supports_motorless_aero_scenarios() {
        let mut scenario = niskanen_scenario();
        scenario.document.time.stop_s = 0.010;
        scenario.document.propulsion = None;
        scenario.document.forces = Some(openbmp_scenario::ForcesConfig {
            models: vec!["gravity".to_owned(), "aero".to_owned()],
        });

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
        let dry_mass_kg = crate::runner::assembly::dry_mass_kg_at(
            &assembly,
            SimTime::from_seconds(scenario.document.time.start_s),
            "vehicle.assembly",
        )
        .expect("assembly dry mass");
        let expected_initial_mass_kg = dry_mass_kg
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
        scenario.document.forces = Some(openbmp_scenario::ForcesConfig {
            models: vec!["gravity".to_owned()],
        });
        // Override the assembly's single body's dry mass to a known
        // 12.5 kg value so the assertion below is checking that the
        // mass model picks up the assembly value (not the canonical
        // Niskanen 0.080 kg).
        let body = scenario
            .document
            .vehicle
            .assembly
            .bodies
            .first_mut()
            .expect("niskanen scenario has at least one body");
        body.dry_mass_kg = 12.5;

        let resolved_files = scenario.resolved_files().expect("resolve files");
        let loaded_models = load_models(&scenario.document, &resolved_files).expect("load models");
        let assembly =
            crate::runner::assembly::synthesize_assembly(&scenario.document).expect("assembly");

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

    #[test]
    fn recovery_requires_explicit_ussa76_atmosphere() {
        let mut scenario = parachute_scenario();
        scenario.document.atmosphere = None;
        scenario.document.environment.atmosphere = "none".to_owned();

        let err = require_supported_shape(&scenario.document).unwrap_err();
        assert!(
            matches!(err, CliError::UnsupportedScenario { ref what }
                if what.contains("recovery drag") && what.contains("us_standard_1976")),
            "expected recovery atmosphere rejection, got {err:?}"
        );
    }

    #[test]
    fn recovery_telemetry_channels_are_recorded() {
        let mut scenario = parachute_scenario();
        scenario.document.time.stop_s = scenario.document.time.dt_s;
        let resolved_files = scenario
            .resolved_files()
            .expect("resolve parachute scenario");

        let outcome = run(&scenario, &resolved_files).expect("short parachute run succeeds");

        assert_eq!(
            first_row_value(&outcome, "recovery.dual_chute.deployed"),
            &TelemetryValue::Bool(false)
        );
        assert_eq!(
            first_row_value(&outcome, "recovery.dual_chute.phase_index"),
            &TelemetryValue::Int64(0)
        );
        assert_eq!(
            first_row_value(&outcome, "recovery.dual_chute.drag_area_m2"),
            &TelemetryValue::Float64(0.0)
        );
    }
}
