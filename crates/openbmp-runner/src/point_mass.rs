//! Scenario → kernel → telemetry adapter for point-mass
//! scenarios that use the structured blocks (`[aero]`,
//! `[propulsion.motor]`, `[atmosphere]`, …).
//!
//! Accepted scenario shape (any combination of the following relative
//! to the baseline structured scenario shape):
//!
//! - `vehicle.kind = "point_mass"`
//! - `environment.gravity = "constant"` with non-negative
//!   `gravity_m_s2`
//! - `[atmosphere].kind = "us_standard_1976"` (when `[aero]` declared
//!   AND `forces` includes `aero`)
//! - `[aero].deck = "<path>"` with optional pinned digest
//! - `[propulsion.motor].file = "<path>"` with optional pinned digest
//! - `forces.models` entries permuted from `["gravity", "aero",
//!   "thrust", "aerothermal_diagnostics"]`
//!
//! Force evaluation order respects the scenario-declared
//! `forces.models` order; this is the determinism contract. Bad
//! SHA-256 pins fail closed before kernel construction (via
//! [`Scenario::resolved_files`] called in
//! [`crate::run`]).
//!
//! Telemetry layout: the seven base channels
//! (position×3, velocity×3, mass) plus, conditionally:
//!
//! - **Atmosphere sample** when `[atmosphere].kind = "us_standard_1976"`:
//!   `atmosphere.density_kg_m3`, `atmosphere.pressure_pa`,
//!   `atmosphere.temperature_k`, `atmosphere.speed_of_sound_m_s`
//!   (atmospheric scalars; no frame metadata).
//! - **Per-model force breakdown** for every entry in `forces.models`:
//!   `force.<name>.x_n`, `.y_n`, `.z_n` with frame metadata `"ECI"`.
//!   `<name>` matches the scenario-declared model name (`gravity`,
//!   `thrust`, `aero`, `aerothermal_diagnostics`).
//! - **Recovery state** when `[[vehicle.assembly.recovery]]` is
//!   declared: `recovery.<id>.deployed`, `.phase_index`, and
//!   `.drag_area_m2`.
//!
//! The schema also carries the resolved-file SHA-256 digests as
//! Arrow schema metadata under `openbmp.scenario_files.<field>` keys.

use std::collections::BTreeMap;

use nalgebra::Vector3;
use openbmp_aero::AeroDeck;
use openbmp_core::{ChannelId, Duration, ModelId, Position3, RecoveryId, SimTime, Velocity3};
use openbmp_physics::{
    AtmosphereModel, Egm2008ZonalGravity, J2Gravity, PointMassGravity, WGS84_J2,
};
use openbmp_propulsion::{Motor, SolidMotor};
use openbmp_scenario::{ResolvedFile, Scenario, ScenarioDocument};
use openbmp_sim::{
    AnyStop, ConstantGravityForce, ConstantMass, EndTime, ForceContext, ForceModel, GroundImpact,
    MassModel, ScenarioScriptAction, SimulationConfig, SimulationKernel, StopReason,
};
use openbmp_state::PointMassState;
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use openbmp_vehicle::{
    AeroMethodForceAdapter, BoxedMassModel, DeckDragForceAdapter, EngineClusterForceAdapter,
    EngineClusterMassAdapter, GravityForceAdapter, KernelVehicle, MotorMassAdapter,
    MotorThrustForceAdapter, NamedForceModel, RecoveryRackForceAdapter, TankRackForceAdapter,
    TankRackMassAdapter, Vehicle,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::RunOutcome;
use crate::assembly::dry_mass_kg_at;
use crate::atmosphere::{
    RuntimeAtmosphere, RuntimeEnvironment, atmosphere_altitude_m_with_surface_radius,
    build_document_runtime_atmosphere, document_geocentric_surface_radius_m,
    is_runtime_atmosphere_kind, scenario_atmosphere_kind,
};
use crate::error::RunnerError;
use crate::integrator::build_runtime_integrator;
use openbmp_vehicle::Assembly;

// Stable model-ids assigned to each force / mass model the runner
// wires. Scenario-supplied force-model names ("aero", "thrust") are
// mapped to these; the constant-gravity force is registered via
// `ConstantGravityForce::new` which carries its own internal id.
// The determinism oracle uses model-ids; the runner
// picks fixed values so the per-model telemetry stream is keyed
// deterministically.
const POINT_MASS_AERO_MODEL_ID: ModelId = ModelId::new(102);
const POINT_MASS_THRUST_MODEL_ID: ModelId = ModelId::new(103);
const POINT_MASS_MOTOR_MASS_MODEL_ID: ModelId = ModelId::new(104);
// Non-constant gravity (point_mass / j2 / egm2008) routes
// through `GravityForceAdapter`, which keys its per-model telemetry
// stream on this id. The legacy `constant` arm continues to use
// `ConstantGravityForce` (a model with no model id) so
// every existing constant-gravity scenario stays byte-stable.
const POINT_MASS_GRAVITY_MODEL_ID: ModelId = ModelId::new(105);
const POINT_MASS_AEROTHERMAL_MODEL_ID: ModelId = ModelId::new(106);
const POINT_MASS_CONTACT_MODEL_ID: ModelId = ModelId::new(107);
// Distinct model ids for the engine-cluster path so the
// determinism oracle can tell legacy single-motor scenarios apart
// from cluster scenarios in the per-model force breakdown.
const RIGID_BODY_ENGINE_CLUSTER_THRUST_MODEL_ID: ModelId = ModelId::new(120);
const RIGID_BODY_ENGINE_CLUSTER_MASS_MODEL_ID: ModelId = ModelId::new(121);
const RIGID_BODY_TANK_RACK_FORCE_MODEL_ID: ModelId = ModelId::new(330);
const RIGID_BODY_TANK_RACK_MASS_MODEL_ID: ModelId = ModelId::new(331);
const RIGID_BODY_RECOVERY_RACK_FORCE_MODEL_ID: ModelId = ModelId::new(370);
const MISSING_AEROTHERMAL_DIAGNOSTICS_MESSAGE: &str =
    "forces includes `aerothermal_diagnostics` but [aerothermal] block is missing";

#[derive(Clone, Debug, Default)]
struct LoadedModels {
    aero_deck: Option<AeroDeck>,
    motor: Option<SolidMotor>,
}

/// Run a point-mass scenario through a freshly-built kernel
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
/// does not match the point-mass contract,
/// [`RunnerError::Aero`] / [`RunnerError::Motor`] / [`RunnerError::Env`] for
/// loader failures, and [`RunnerError::Simulation`] / [`RunnerError::Telemetry`]
/// for kernel- or telemetry-side failures.
#[allow(clippy::too_many_lines)] // per-step orchestration is large
pub fn run(
    scenario: &Scenario,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    mut monitor: Option<&mut (dyn crate::sil::SilMonitor + '_)>,
) -> Result<RunOutcome, RunnerError> {
    let document = &scenario.document;
    require_supported_shape(document)?;
    let assembly = crate::assembly::synthesize_assembly(document)?;
    // Build the runner-side effector rack. Empty when
    // no `[[vehicle.assembly.effectors]]` are declared, in which
    // case every per-step rack operation short-circuits and the
    // legacy byte-stable kernel path is preserved.
    let mut effector_rack = crate::effectors::EffectorRack::build(document)?;
    // Build the runner-side engine rack. Empty when no
    // `[[vehicle.assembly.engines]]` are declared, in which case
    // every per-step rack operation short-circuits and the legacy
    // single-motor byte-stable path is preserved.
    let mut engine_rack = crate::engines::EngineRack::build(document)?;
    // Build the runner-side tank rack. Empty when no
    // `[[vehicle.assembly.tanks]]` are declared, in which case every
    // per-step rack operation short-circuits and the legacy
    // byte-stable path is preserved.
    let mut tank_rack = crate::tanks::TankRack::build(document)?;
    let propellant_budget = crate::propulsion::build_propellant_budget(document)?;
    let mut feed_network_rack = crate::feed_network::FeedNetworkRack::build(document)?;
    let _pogo_rack = crate::pogo::PogoStabilityRack::build(document)?;
    // Build the runner-side recovery rack. Empty when no
    // `[[vehicle.assembly.recovery]]` are declared, in which case
    // every per-step rack operation short-circuits and the kernel's
    // recovery snapshot stays empty (byte-stable for scenarios
    // without recovery devices).
    let mut recovery_rack = crate::recovery::RecoveryRack::build(document)?;
    // Build the runner-side wind rack. Inactive when no
    // `[wind]` block is declared (or `kind = "none"`); the per-step
    // rack op short-circuits and the kernel's wind override stays
    // `None`, preserving byte output for scenarios without wind.
    let wind_rack = crate::wind::WindRack::build(document)?;
    wind_rack.reset();
    let frame = crate::frames::build_frame_context(document, resolved_files)?;

    let loaded_models = load_models(document, resolved_files)?;
    crate::aero::reject_hypersonic_deck_only_out_of_envelope(
        document,
        loaded_models.aero_deck.as_ref(),
    )?;
    let initial_state = build_initial_state(document, &loaded_models, &assembly)?;
    let mut aerothermal_driver = crate::aerothermal::LiveAerothermalDriver::maybe_new(document)?;
    let aerothermal_sink = aerothermal_driver
        .as_ref()
        .map(crate::aerothermal::LiveAerothermalDriver::sink);
    let kernel_vehicle = build_vehicle(
        document,
        &loaded_models,
        &assembly,
        resolved_files,
        aerothermal_sink.clone(),
    )?;
    // The runner-side breakdown vehicle is a *separate* construction
    // of the same models. `KernelVehicle::evaluate_force_breakdown`
    // takes `&self`, but `KernelVehicle` is not `Clone` (the inner
    // `Box<dyn ForceModel>` lists are not). Re-building from scratch
    // keeps the force-list ownership simple. Stateful aerothermal
    // integration remains in the live driver; both vehicle copies get
    // zero-force diagnostic adapters over the same latest-output sink.
    let breakdown_vehicle = build_vehicle(
        document,
        &loaded_models,
        &assembly,
        resolved_files,
        aerothermal_sink,
    )?;
    let mass_model = BoxedMassModel(build_mass_model(document, &loaded_models, &assembly)?);

    // Runtime integrator dispatch from the scenario
    // [solver] block. Defaults to Rk4FixedStep when [solver] is
    // absent, preserving the byte-stable contract for every
    // existing scenario.
    let runtime_integrator = build_runtime_integrator(document)?;
    let config = SimulationConfig {
        initial_state,
        integrator: runtime_integrator,
        force_model: kernel_vehicle,
        mass_model,
        environment: RuntimeEnvironment::from_document(document, resolved_files, &frame)?,
        stop_condition: AnyStop::new(
            automatic_ground_impact(document),
            EndTime::new(SimTime::from_seconds(document.time.stop_s)),
        ),
        dt: Duration::from_seconds(document.time.dt_s),
        scenario_seed: document.time.seed,
    };

    let kernel_base = SimulationKernel::new(config)?;
    let mut kernel = if let Some(mission_runtime) =
        crate::mission::build_mission_runtime_from_document(document)?
    {
        kernel_base.with_mission_split(
            mission_runtime.mission_bindings,
            mission_runtime.script_bindings,
            Some(mission_runtime.graph),
            Some(mission_runtime.hsm),
        )?
    } else {
        kernel_base
    };
    if document.flight_controller_owns_mission_state()
        && let Some(initial_phase) = kernel.current_phase()
    {
        kernel.set_external_mission_state(Some(initial_phase));
    }
    let channel_set = PointMassChannelSet::new(document)?;
    let geocentric_surface_radius_m = document_geocentric_surface_radius_m(document);
    let breakdown_atmosphere = if channel_set.has_atmosphere {
        Some(build_document_runtime_atmosphere(document)?)
    } else {
        None
    };
    let metadata = build_schema_metadata(document, resolved_files)?;
    let mut table = TelemetryTable::new(channel_set.schema(metadata)?);

    // Pair schema-2 deck axes with scenario effectors.
    // Schema-1 decks and decks without effector axes produce empty
    // bindings; the per-step snapshot push then short-circuits and
    // the kernel's effector-actuals map stays empty (byte-stable).
    let deck_bindings = crate::aero_effector_match::assert_axes_match_effectors(
        loaded_models.aero_deck.as_ref(),
        document,
    )?;

    // Step 0 has no fired events. The effector snapshot at step 0 is
    // each effector's load-time at-rest state (initial position).
    let initial_snapshot = effector_rack.snapshot();
    if !deck_bindings.is_empty() {
        let snapshot_map =
            crate::aero_effector_match::build_snapshot_map(&deck_bindings, &initial_snapshot);
        kernel.set_effector_actuals(snapshot_map);
    }
    // At step 0, push the rack's initial (Idle) snapshot
    // to the kernel so the breakdown's mass adapter sees the same
    // empty-consumption view the kernel will see on its first step.
    if !engine_rack.is_empty() {
        kernel.set_engine_snapshot(engine_rack.snapshot_map());
    }
    // At step 0, push the rack's initial snapshot to the
    // kernel so the breakdown's mass adapter and force adapter see
    // the same view the kernel will see.
    if !tank_rack.is_empty() {
        kernel.set_tank_snapshot(tank_rack.snapshot_map());
    }
    // At step 0, push the rack's initial (Stowed)
    // snapshot to the kernel so the breakdown's recovery-drag force
    // adapter sees the same view the kernel will see.
    if !recovery_rack.is_empty() {
        kernel.set_recovery_snapshot(recovery_rack.snapshot_map());
    }
    // At step 0, sample the wind at the initial state
    // and push to the kernel so the breakdown's force adapter sees
    // the same wind the kernel will see on its first step.
    if !wind_rack.is_inactive() {
        let initial_state = kernel.current_state();
        let wind = wind_rack.sample(initial_state.position, &frame, initial_state.time)?;
        kernel.set_wind_sample(wind);
    }
    if let Some(driver) = &mut aerothermal_driver {
        let environment = kernel.current_environment_sample()?;
        driver.evaluate_point_mass(kernel.current_state(), &environment, 0.0)?;
    }
    let mut fc_bridge = crate::fc_bridge::FcBridge::maybe_new(scenario, resolved_files)?;
    let mut mission_region_trace =
        crate::MissionRegionTraceState::new(&crate::mission_region_declarations(document));
    record_step(
        document,
        &mut table,
        &kernel,
        &channel_set,
        &breakdown_vehicle,
        breakdown_atmosphere.as_ref(),
        geocentric_surface_radius_m,
        aerothermal_driver.as_ref().map(|driver| driver.output()),
        fc_bridge.as_ref(),
        &[],
        &mut mission_region_trace,
        &initial_snapshot,
    )?;
    let mut pending_effector_events: Vec<openbmp_sim::FiredEvent<ScenarioScriptAction>> =
        Vec::new();
    let mut pending_engine_events: Vec<openbmp_sim::FiredEvent<ScenarioScriptAction>> = Vec::new();
    let mut pending_recovery_events: Vec<openbmp_sim::FiredEvent<ScenarioScriptAction>> =
        Vec::new();
    let mut realtime_pacer = crate::rt::RunnerRealtimePacer::from_document(document)?;
    while kernel.stop_reason().is_none() {
        realtime_pacer.wait_next_frame();
        realtime_pacer.begin_frame_execution();
        effector_rack.apply_overrides(&pending_effector_events)?;
        // Drain pending engine commands from the previous
        // kernel step, apply to the rack, then advance the rack.
        if !engine_rack.is_empty() {
            engine_rack.apply_commands(&pending_engine_events)?;
        }
        if let Some(bridge) = &mut fc_bridge {
            let gravity = kernel.current_environment_sample()?.gravity_eci_m_s2;
            let propellant_state = crate::fc_bridge::propellant_state_from_tanks(
                kernel.current_time(),
                &tank_rack.propellant_tank_states(document),
            );
            bridge.tick_point_mass(
                kernel.current_state(),
                kernel.current_step(),
                gravity,
                propellant_state,
                &mut effector_rack,
                &mut engine_rack,
                monitor.as_deref_mut(),
            )?;
            if document.flight_controller_owns_mission_state() {
                // Forward the FC commander's published mission state into
                // the kernel's external view. Kernel-directed scenarios
                // intentionally keep the kernel event stream authoritative.
                if let Some(state_id) = bridge.latest_mission_state_id() {
                    kernel.set_external_mission_state(Some(openbmp_sim::PhaseId::new(state_id)));
                }
                for fired in bridge.drain_mission_actions() {
                    kernel.record_external_mission_fired(fired);
                }
            }
        }
        if let Some(propellant_budget) = &propellant_budget {
            let mut report = propellant_budget
                .evaluate(
                    &engine_rack.propulsion_snapshot_map(),
                    &tank_rack.propellant_tank_states(document),
                )
                .map_err(|err| RunnerError::Engine {
                    field: "vehicle.assembly.engines[*].propellant".to_owned(),
                    reason: err.to_string(),
                })?;
            feed_network_rack.apply_to_report(
                &mut report,
                document.time.dt_s,
                kernel.current_step(),
            )?;
            tank_rack.set_propellant_budget_drain_rates(report.tank_drain_rates_kg_per_s.clone());
            engine_rack.apply_propellant_budget(&report)?;
        }
        if !effector_rack.is_empty() {
            effector_rack.step(kernel.current_time())?;
        }
        if !engine_rack.is_empty() {
            let cavitation_events = feed_network_rack.cavitation_events();
            engine_rack.apply_cavitation_faults(&cavitation_events)?;
            engine_rack.apply_scheduled_faults(kernel.current_step())?;
            engine_rack.step()?;
        }
        // Advance the tank rack using prior-step cached
        // drivers (set after the previous kernel step). For point-
        // mass kernels the drivers are zeros — slosh in point-mass
        // is RigidLiquid-only per the scenario validator (D10), so
        // the dynamic drivers are irrelevant.
        if !tank_rack.is_empty() {
            tank_rack.step()?;
        }
        // Drain pending deploy/stow events from the prior
        // kernel step, apply to the rack, then advance internal state
        // (no-op for the instantaneous-deploy models).
        if !recovery_rack.is_empty() {
            recovery_rack.apply_deploys(&pending_recovery_events)?;
            recovery_rack.step(document.time.dt_s)?;
        }
        // Push the rack's actuals snapshot to the kernel
        // BEFORE `step()` so all four RK4 stages see the same view.
        // Empty bindings → zero allocation, zero state change.
        if !deck_bindings.is_empty() {
            let rack_snapshot = effector_rack.snapshot();
            let snapshot_map =
                crate::aero_effector_match::build_snapshot_map(&deck_bindings, &rack_snapshot);
            kernel.set_effector_actuals(snapshot_map);
        }
        // Push the rack's engine snapshot to the kernel
        // BEFORE `step()` so all four RK4 stages see the same view.
        // Empty rack → zero allocation, zero state change (the
        // kernel's `engine_snapshot` field stays at the empty
        // `BTreeMap` set in `new()`).
        if !engine_rack.is_empty() {
            kernel.set_engine_snapshot(engine_rack.snapshot_map());
        }
        // Push tank snapshot to the kernel before
        // `step()` so all four RK4 stages see the same view.
        if !tank_rack.is_empty() {
            kernel.set_tank_snapshot(tank_rack.snapshot_map());
        }
        // Push recovery snapshot to the kernel before
        // `step()` so all four RK4 stages see the same view.
        if !recovery_rack.is_empty() {
            kernel.set_recovery_snapshot(recovery_rack.snapshot_map());
        }
        // Advance the wind rack and push the new sample
        // to the kernel before `step()`. For `GustWind` this rolls
        // the Dryden filter forward by one tick; the time-only
        // models are no-ops.
        if !wind_rack.is_inactive() {
            wind_rack.advance(kernel.current_step());
            let s = kernel.current_state();
            let wind = wind_rack.sample(s.position, &frame, s.time)?;
            kernel.set_wind_sample(wind);
        }
        kernel.step()?;
        let mission_fired = kernel.drain_mission_fired_events();
        let script_fired = kernel.drain_script_fired_events();
        if let Some(driver) = &mut aerothermal_driver {
            let environment = kernel.current_environment_sample()?;
            driver.evaluate_point_mass(kernel.current_state(), &environment, document.time.dt_s)?;
        }
        let snapshot = effector_rack.snapshot();
        record_step(
            document,
            &mut table,
            &kernel,
            &channel_set,
            &breakdown_vehicle,
            breakdown_atmosphere.as_ref(),
            geocentric_surface_radius_m,
            aerothermal_driver.as_ref().map(|driver| driver.output()),
            fc_bridge.as_ref(),
            &mission_fired,
            &mut mission_region_trace,
            &snapshot,
        )?;
        // Partition the typed script-action fired
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
        pending_effector_events = script_fired
            .iter()
            .filter(|e| matches!(e.action, ScenarioScriptAction::EffectorOverride { .. }))
            .cloned()
            .collect();
        realtime_pacer.finish_frame_execution();
    }

    let stop_reason = kernel
        .stop_reason()
        .cloned()
        .unwrap_or(StopReason::EndTime { reached_s: 0.0 });
    let actuator_stream = fc_bridge
        .as_ref()
        .map(crate::fc_bridge::FcBridge::actuator_stream_report)
        .transpose()?
        .flatten();

    Ok(RunOutcome {
        final_step: kernel.current_step().value(),
        final_time_s: kernel.current_time().as_seconds(),
        stop_reason,
        table,
        realtime: realtime_pacer.finish(),
        actuator_stream,
    })
}

fn automatic_ground_impact(document: &ScenarioDocument) -> GroundImpact {
    if document.contact.is_some() {
        return GroundImpact::disabled();
    }
    if document.environment.gravity == "constant" {
        GroundImpact::sea_level()
    } else {
        GroundImpact::disabled()
    }
}

fn require_supported_shape(document: &ScenarioDocument) -> Result<(), RunnerError> {
    if document.vehicle.kind != "point_mass" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("vehicle.kind = {}", document.vehicle.kind),
        });
    }
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

    // Force list: subset of {gravity, aero, thrust,
    // aerothermal_diagnostics}, scenario-declared order is the
    // determinism contract.
    for name in document.force_model_universe() {
        if !matches!(
            name.as_str(),
            "gravity" | "aero" | "thrust" | "aerothermal_diagnostics" | "contact"
        ) {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "forces.models entry `{name}` (only gravity, aero, thrust, \
                     aerothermal_diagnostics, contact wired)"
                ),
            });
        }
    }

    // Wind models are resolved by WindRack. Scenario
    // validation guarantees that non-`none` flat selections carry a
    // structured `[wind]` block and that the kind names agree.

    // Atmosphere: when `aero` is in the force list, require one of the
    // wired layered atmospheres (USSA76 for the historical sounding-
    // rocket envelope, piecewise-exponential for the
    // engineering 0-1000 km envelope).
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
    loaded_models: &LoadedModels,
    assembly: &Assembly,
) -> Result<PointMassState, RunnerError> {
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
/// `ConstantGravityForce` to keep byte-for-byte
/// reproducibility on every existing point-mass scenario. The other arms
/// (`point_mass`, `j2`, `egm2008`) route through
/// `GravityForceAdapter`, which wraps the corresponding
/// `openbmp_physics::GravityModel`.
fn build_gravity_force_adapter_point_mass(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Box<dyn ForceModel<PointMassState> + Send + Sync>, RunnerError> {
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
            // Legacy force model — retained verbatim to keep
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
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "environment.mu_m3_s2 missing for point_mass gravity".to_owned(),
                    })?;
            let model = PointMassGravity::new(mu)?;
            Ok(Box::new(GravityForceAdapter::new(
                model,
                POINT_MASS_GRAVITY_MODEL_ID,
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
                POINT_MASS_GRAVITY_MODEL_ID,
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
                POINT_MASS_GRAVITY_MODEL_ID,
            )))
        }
        "third_body" => {
            let model = crate::celestial::build_third_body_gravity(document, resolved_files)?;
            Ok(Box::new(GravityForceAdapter::new(
                model,
                POINT_MASS_GRAVITY_MODEL_ID,
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
    loaded_models: &LoadedModels,
    assembly: &Assembly,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    aerothermal_sink: Option<crate::aerothermal::LiveAerothermalSink>,
) -> Result<KernelVehicle<PointMassState>, RunnerError> {
    let mut named: Vec<NamedForceModel<PointMassState>> = Vec::new();

    for name in document.force_model_universe() {
        match name.as_str() {
            "gravity" => {
                let force = build_gravity_force_adapter_point_mass(document, resolved_files)?;
                named.push(NamedForceModel::new("gravity", force));
            }
            "aero" => {
                let atmosphere = build_document_runtime_atmosphere(document)?;
                let method_kind = document
                    .aero
                    .as_ref()
                    .and_then(|aero| aero.method.as_ref())
                    .map_or("deck", |method| method.kind.as_str());
                if method_kind == "deck" {
                    let deck = loaded_models.aero_deck.clone().ok_or_else(|| {
                        RunnerError::UnsupportedScenario {
                            what: "forces includes `aero` but [aero] block is missing".to_owned(),
                        }
                    })?;
                    let drag =
                        DeckDragForceAdapter::new(deck, atmosphere, POINT_MASS_AERO_MODEL_ID);
                    named.push(NamedForceModel::new("aero", Box::new(drag)));
                } else {
                    let (method, reference_length_m) =
                        crate::aero::build_aero_method(document, loaded_models.aero_deck.clone())?
                            .ok_or_else(|| RunnerError::UnsupportedScenario {
                                what: "forces includes `aero` but [aero] block is missing"
                                    .to_owned(),
                            })?;
                    let adapter = AeroMethodForceAdapter::new(
                        method,
                        atmosphere,
                        POINT_MASS_AERO_MODEL_ID,
                        reference_length_m,
                    );
                    named.push(NamedForceModel::new("aero", Box::new(adapter)));
                }
            }
            "thrust" => {
                // Dispatch between single-motor (legacy)
                // and engine-cluster paths based on whether
                // `[[vehicle.assembly.engines]]` is declared. The
                // scenario validator rejects scenarios that declare
                // both blocks (`AmbiguousPropulsion`), so exactly
                // one path resolves.
                if document.vehicle.assembly.engines.is_empty() {
                    let motor = loaded_models.motor.clone().ok_or_else(|| {
                        RunnerError::UnsupportedScenario {
                            what: "forces includes `thrust` but neither [propulsion.motor] nor \
                                   [[vehicle.assembly.engines]] is declared"
                                .to_owned(),
                        }
                    })?;
                    let ignition_time_s = motor_ignition_time_s(document)?;
                    let thrust = MotorThrustForceAdapter::new(
                        motor,
                        ignition_time_s,
                        POINT_MASS_THRUST_MODEL_ID,
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
            "aerothermal_diagnostics" => {
                let sink =
                    aerothermal_sink
                        .clone()
                        .ok_or_else(|| RunnerError::UnsupportedScenario {
                            what: MISSING_AEROTHERMAL_DIAGNOSTICS_MESSAGE.to_owned(),
                        })?;
                let adapter = crate::aerothermal::StagnationHeatingForceAdapter::new(
                    sink,
                    POINT_MASS_AEROTHERMAL_MODEL_ID,
                );
                named.push(NamedForceModel::new(
                    "aerothermal_diagnostics",
                    Box::new(adapter),
                ));
            }
            "contact" => {
                let contact =
                    document
                        .contact
                        .as_ref()
                        .ok_or_else(|| RunnerError::UnsupportedScenario {
                            what: "forces includes `contact` but [contact] block is missing"
                                .to_owned(),
                        })?;
                let adapter = crate::contact::build_half_space_contact_force_adapter(
                    contact,
                    POINT_MASS_CONTACT_MODEL_ID,
                )?;
                named.push(NamedForceModel::new("contact", Box::new(adapter)));
            }
            other => unreachable!("require_supported_shape rejects unknown force model `{other}`"),
        }
    }

    // Tank-rack reaction-force adapter (when tanks
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
        let tank_force = TankRackForceAdapter::new(tank_ids, RIGID_BODY_TANK_RACK_FORCE_MODEL_ID);
        named.push(NamedForceModel::new("tank_reaction", Box::new(tank_force)));
    }

    // Recovery-rack drag-force adapter (when recovery
    // devices declared). Appended last so legacy force-list
    // summation order is unchanged for scenarios without recovery
    // devices; the locked
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
        let atmosphere = build_document_runtime_atmosphere(document)?;
        let recovery_force = RecoveryRackForceAdapter::new(
            recovery_ids,
            atmosphere,
            RIGID_BODY_RECOVERY_RACK_FORCE_MODEL_ID,
        );
        named.push(NamedForceModel::new(
            "recovery_drag",
            Box::new(recovery_force),
        ));
    }

    // KernelVehicle requires a mass model even for vehicle-internal
    // queries. The kernel's mass model is built
    // separately in `build_mass_model` because it owns its own copy.
    let vehicle_mass = build_mass_model(document, loaded_models, assembly)?;
    KernelVehicle::new_with_default_active_models(
        named,
        vec![],
        vehicle_mass,
        crate::default_active_force_models(document),
        crate::phase_force_overrides(document),
    )
    .map_err(|e| RunnerError::UnsupportedScenario {
        what: format!("KernelVehicle construction failed: {e}"),
    })
}

fn build_mass_model(
    document: &ScenarioDocument,
    loaded_models: &LoadedModels,
    assembly: &Assembly,
) -> Result<Box<dyn MassModel>, RunnerError> {
    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_mass_kg = dry_mass_kg_at(assembly, start_time, "vehicle.assembly")?;
    // Dispatch to the cluster mass adapter when
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
            RIGID_BODY_ENGINE_CLUSTER_MASS_MODEL_ID,
        ))
    } else if let Some(motor) = &loaded_models.motor {
        let ignition_time_s = motor_ignition_time_s(document)?;
        Box::new(MotorMassAdapter::new(
            motor.clone(),
            dry_mass_kg,
            ignition_time_s,
            POINT_MASS_MOTOR_MASS_MODEL_ID,
        ))
    } else {
        Box::new(ConstantMass::new(dry_mass_kg))
    };

    // Wrap the base mass model with a tank-rack mass
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
            RIGID_BODY_TANK_RACK_MASS_MODEL_ID,
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
// Channel set: base + atmosphere + per-model force breakdown
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
struct FcReferenceTelemetryChannels {
    valid: TelemetryChannel<bool>,
    quaternion_x: TelemetryChannel<f64>,
    quaternion_y: TelemetryChannel<f64>,
    quaternion_z: TelemetryChannel<f64>,
    quaternion_w: TelemetryChannel<f64>,
    cutoff_valid: Option<TelemetryChannel<bool>>,
    cutoff_time_to_go_s: Option<TelemetryChannel<f64>>,
}

#[derive(Debug)]
struct PointMassChannelSet {
    position_x: TelemetryChannel<f64>,
    position_y: TelemetryChannel<f64>,
    position_z: TelemetryChannel<f64>,
    velocity_x: TelemetryChannel<f64>,
    velocity_y: TelemetryChannel<f64>,
    velocity_z: TelemetryChannel<f64>,
    mass: TelemetryChannel<f64>,
    mission_phase: Option<TelemetryChannel<String>>,
    mission_regions: Vec<(crate::MissionRegionDeclaration, TelemetryChannel<String>)>,
    has_atmosphere: bool,
    atmosphere_density: Option<TelemetryChannel<f64>>,
    atmosphere_pressure: Option<TelemetryChannel<f64>>,
    atmosphere_temperature: Option<TelemetryChannel<f64>>,
    atmosphere_speed_of_sound: Option<TelemetryChannel<f64>>,
    /// Latest FC guidance reference, present only for `[fc]` scenarios.
    fc_reference: Option<FcReferenceTelemetryChannels>,
    /// Force-model components in declared order.
    force_components: ForceComponentChannels,
    /// Active per-phase model list, present when phase overrides are
    /// declared.
    active_models: Option<TelemetryChannel<String>>,
    /// Live aerothermal diagnostic channels.
    aerothermal: Option<AerothermalTelemetryChannels>,
    /// Effector deflection channels, in scenario-declared
    /// order. One `effector.<id>.actual` `f64` channel per declared
    /// effector. Allocated AFTER force breakdown channels and BEFORE
    /// mission markers — this ordering is the determinism contract.
    effector_actuals: Vec<TelemetryChannel<f64>>,
    /// Recovery-state channels, in scenario-declared order.
    /// Allocated after effectors and before mission markers so marker
    /// ordering remains alphabetical and recovery-free scenarios keep
    /// their legacy schema unchanged.
    recovery_states: RecoveryTelemetryChannels,
    /// Mission-event telemetry markers, keyed by tag.
    /// `BTreeMap` order is alphabetical for deterministic channel
    /// allocation regardless of scenario-text declaration order.
    mission_markers: BTreeMap<String, TelemetryChannel<bool>>,
}

impl PointMassChannelSet {
    #[allow(clippy::too_many_lines)] // channel inventory grows with each schema extension
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
        let mission_phase = document
            .mission
            .is_some()
            .then(|| {
                TelemetryChannel::<String>::new(alloc(), "mission.phase", "text", None::<&str>)
            })
            .transpose()?;
        let mut mission_regions = Vec::new();
        for declaration in crate::mission_region_declarations(document) {
            let channel = TelemetryChannel::<String>::new(
                alloc(),
                format!("mission.region.{}", declaration.channel_suffix),
                "text",
                None::<&str>,
            )?;
            mission_regions.push((declaration, channel));
        }

        let fc_reference = if document.fc.is_some() {
            Some(FcReferenceTelemetryChannels {
                valid: TelemetryChannel::<bool>::new(
                    alloc(),
                    "guidance.reference.valid",
                    "bool",
                    None::<&str>,
                )?,
                quaternion_x: TelemetryChannel::<f64>::new(
                    alloc(),
                    "guidance.reference.q_x",
                    "1",
                    None::<&str>,
                )?,
                quaternion_y: TelemetryChannel::<f64>::new(
                    alloc(),
                    "guidance.reference.q_y",
                    "1",
                    None::<&str>,
                )?,
                quaternion_z: TelemetryChannel::<f64>::new(
                    alloc(),
                    "guidance.reference.q_z",
                    "1",
                    None::<&str>,
                )?,
                quaternion_w: TelemetryChannel::<f64>::new(
                    alloc(),
                    "guidance.reference.q_w",
                    "1",
                    None::<&str>,
                )?,
                cutoff_valid: if document_exports_guidance_cutoff(document) {
                    Some(TelemetryChannel::<bool>::new(
                        alloc(),
                        "guidance.cutoff.valid",
                        "bool",
                        None::<&str>,
                    )?)
                } else {
                    None
                },
                cutoff_time_to_go_s: if document_exports_guidance_cutoff(document) {
                    Some(TelemetryChannel::<f64>::new(
                        alloc(),
                        "guidance.cutoff.time_to_go_s",
                        "s",
                        None::<&str>,
                    )?)
                } else {
                    None
                },
            })
        } else {
            None
        };

        // Atmosphere channels: emitted whenever the scenario declares
        // a layered atmosphere the runner can sample (USSA76 and
        // piecewise-exponential).
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

        // Effector deflection channels, in scenario-
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

        // Mission marker channels. `BTreeMap` ordering on
        // tag keys keeps channel id allocation deterministic even
        // when the scenario reorders `[[mission.events]]` blocks.
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
            mission_phase,
            mission_regions,
            has_atmosphere,
            atmosphere_density,
            atmosphere_pressure,
            atmosphere_temperature,
            atmosphere_speed_of_sound,
            fc_reference,
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
        ];
        if let Some(mission_phase) = &self.mission_phase {
            channels.push(mission_phase.metadata().clone());
        }
        for (_, region) in &self.mission_regions {
            channels.push(region.metadata().clone());
        }
        if let Some(reference) = &self.fc_reference {
            channels.push(reference.valid.metadata().clone());
            channels.push(reference.quaternion_x.metadata().clone());
            channels.push(reference.quaternion_y.metadata().clone());
            channels.push(reference.quaternion_z.metadata().clone());
            channels.push(reference.quaternion_w.metadata().clone());
            if let Some(cutoff_valid) = &reference.cutoff_valid {
                channels.push(cutoff_valid.metadata().clone());
            }
            if let Some(cutoff_time_to_go_s) = &reference.cutoff_time_to_go_s {
                channels.push(cutoff_time_to_go_s.metadata().clone());
            }
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
        // order. Allocated AFTER force breakdown channels and BEFORE
        // mission markers — this ordering is the determinism contract.
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
fn record_step<I, F, MM, E, SC>(
    document: &ScenarioDocument,
    table: &mut TelemetryTable,
    kernel: &SimulationKernel<PointMassState, I, F, MM, E, SC>,
    channels: &PointMassChannelSet,
    breakdown_vehicle: &KernelVehicle<PointMassState>,
    breakdown_atmosphere: Option<&RuntimeAtmosphere>,
    geocentric_surface_radius_m: Option<f64>,
    aerothermal: Option<&crate::aerothermal::LiveAerothermalOutput>,
    fc_bridge: Option<&crate::fc_bridge::FcBridge>,
    fired_events: &[openbmp_sim::FiredEvent<openbmp_sim::MissionAction>],
    mission_region_trace: &mut crate::MissionRegionTraceState,
    effector_snapshot: &[openbmp_vehicle::EffectorState],
) -> Result<(), RunnerError>
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
    if let Some(mission_phase) = &channels.mission_phase {
        row.insert(
            mission_phase,
            crate::mission_phase_label(document, kernel.current_phase().map(|phase| phase.value())),
        )?;
    }
    mission_region_trace.apply_fired_events(fired_events);
    for (declaration, channel) in &channels.mission_regions {
        row.insert(channel, mission_region_trace.label(declaration))?;
    }
    insert_fc_reference_channels(&mut row, &channels.fc_reference, fc_bridge)?;

    // Atmosphere sample at the post-step state. Match the runtime
    // environment's frame-aware altitude conversion so ECI launches
    // do not report sea-level telemetry density at orbital radius.
    if let Some(atmosphere) = breakdown_atmosphere {
        let altitude_m = atmosphere_altitude_m_with_surface_radius(
            state.position.vector,
            geocentric_surface_radius_m,
        );
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
    // models the kernel uses. Stateful aerothermal integration is
    // owned by the live driver, while the zero-force diagnostic
    // adapter reads the shared latest-output sink. The breakdown is
    // therefore the per-model contribution to the kernel's total at
    // the step boundary.
    //
    // The breakdown's `effector_actuals` view mirrors
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
        active_body: None,
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
            .map(|(_, vector)| *vector)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "force-breakdown component `{declared_name}` missing from vehicle evaluation"
                ),
            })?;
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

    // Effector deflection channels. The snapshot is in
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

    // Marker channels: write `true` for any tag whose
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

fn insert_fc_reference_channels(
    row: &mut TelemetryRow,
    channels: &Option<FcReferenceTelemetryChannels>,
    fc_bridge: Option<&crate::fc_bridge::FcBridge>,
) -> Result<(), RunnerError> {
    let Some(channels) = channels else {
        return Ok(());
    };
    let reference = fc_bridge.and_then(crate::fc_bridge::FcBridge::latest_reference_state);
    row.insert(&channels.valid, reference.is_some())?;
    let q = reference.map_or([0.0, 0.0, 0.0, 1.0], |r| r.q_body_to_eci_xyzw);
    row.insert(&channels.quaternion_x, q[0])?;
    row.insert(&channels.quaternion_y, q[1])?;
    row.insert(&channels.quaternion_z, q[2])?;
    row.insert(&channels.quaternion_w, q[3])?;
    if let (Some(valid_channel), Some(time_channel)) =
        (&channels.cutoff_valid, &channels.cutoff_time_to_go_s)
    {
        let time_to_go_s = fc_bridge
            .and_then(crate::fc_bridge::FcBridge::latest_guidance_cutoff)
            .map(|cutoff| cutoff.time_to_go_s)
            .filter(|time_to_go_s| time_to_go_s.is_finite());
        row.insert(valid_channel, time_to_go_s.is_some())?;
        row.insert(time_channel, time_to_go_s.unwrap_or(0.0))?;
    }
    Ok(())
}

fn document_exports_guidance_cutoff(document: &ScenarioDocument) -> bool {
    document.fc.as_ref().is_some_and(|fc| {
        matches!(
            fc.guidance,
            openbmp_scenario::FcGuidanceKind::AscentReference
        )
    })
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
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use std::path::{Path, PathBuf};

    use openbmp_telemetry::TelemetryValue;

    use super::*;

    const POINT_MASS_ENTRY_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "point-mass-live-entry-coupling-test"
description = "Synthetic point-mass live entry coupling regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 7

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 80000.0]
initial_velocity_eci_m_s = [7600.0, 0.0, -100.0]

[vehicle.assembly]
id = "point-mass-live-entry-coupling-test"

[[vehicle.assembly.bodies]]
id = "capsule"
geometry = { kind = "cylinder", length_m = 1.0, diameter_m = 1.0 }
dry_mass_kg = 100.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "piecewise_exponential"
wind = "none"

[atmosphere]
kind = "piecewise_exponential"

[aerothermal]
stagnation_kind = "sutton_graves"
nose_radius_m = 0.5
wall_temperature_k = 1500.0
wall_catalysis = "fully_catalytic"

[forces]
models = ["gravity", "aerothermal_diagnostics"]

[telemetry]
output.csv = "out/point-mass-live-entry-coupling-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const USSA76_EXO_AERO_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "ussa76-exo-aero-test"
description = "Synthetic point-mass aero run above the USSA76 ceiling."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 8

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 90000.0]
initial_velocity_eci_m_s = [0.0, 0.0, -1000.0]

[vehicle.assembly]
id = "ussa76-exo-aero-test"

[[vehicle.assembly.bodies]]
id = "capsule"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 100.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "us_standard_1976"
wind = "none"

[atmosphere]
kind = "us_standard_1976"

[aero]

[aero.method]
kind = "modified_newtonian"

[aero.method.modified_newtonian]
cp_max = 2.0
reference_area_m2 = 1.0
reference_length_m = 1.0

[forces]
models = ["gravity", "aero"]

[telemetry]
output.csv = "out/ussa76-exo-aero-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const GROUND_IMPACT_AUTO_STOP_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "ground-impact-auto-stop-test"
description = "Synthetic point-mass descent without a manual ground-stop event."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 10.0
dt_s = 1.0
seed = 9

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, -10.0]

[vehicle.assembly]
id = "ground-impact-auto-stop-test"

[[vehicle.assembly.bodies]]
id = "mass"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/ground-impact-auto-stop-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const CONTACT_POINT_MASS_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "contact-point-mass-test"
description = "Synthetic point-mass contact run with an initial half-space penetration."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.002
dt_s = 0.001
seed = 11

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, -0.01]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "contact-point-mass-test"

[[vehicle.assembly.bodies]]
id = "mass"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity", "contact"]

[contact]
kind = "half_space"
ground_altitude_m = 0.0
geometry = "point"
normal_law = "kelvin_voigt"
stiffness_n_m = 2000.0
damping_n_s_m = 0.0
friction_coefficient = 0.0
effective_mass_kg = 1.0
substeps = 1

[telemetry]
output.csv = "out/contact-point-mass-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const DYNAMIC_PRESSURE_EVENT_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "dynamic-pressure-event-test"
description = "Synthetic point-mass run with a dynamic-pressure mission stop."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 10.0
dt_s = 1.0
seed = 10

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 1000.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "dynamic-pressure-event-test"

[[vehicle.assembly.bodies]]
id = "mass"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "us_standard_1976"
wind = "none"

[atmosphere]
kind = "us_standard_1976"

[forces]
models = ["gravity"]

[mission]
initial_phase = "descent"

[[mission.phases]]
id = "descent"
label = "descent"

[[mission.events]]
id = "evt_dynamic_pressure"
trigger = { kind = "at_dynamic_pressure", pressure_pa = 10.0, falling = false }
action = { kind = "stop", label = "q-rise" }
once = true

[telemetry]
output.csv = "out/dynamic-pressure-event-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const THIRD_BODY_POINT_MASS_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "third-body-point-mass-test"
description = "Synthetic high-apogee point-mass run with lunar third-body perturbation."
validation = "validated-toy"

[epoch]
scale = "UTC"
iso8601 = "2000-01-01T12:00:00Z"

[time]
start_s = 0.0
stop_s = 1.0
dt_s = 1.0
seed = 11

[vehicle]
kind = "point_mass"
initial_position_eci_m = [100000000.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "third-body-point-mass-test"

[[vehicle.assembly.bodies]]
id = "mass"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "third_body"
gravity_base = "point_mass"
mu_m3_s2 = 3.986004418e14
third_bodies = ["moon"]
ephemeris = "low_precision_sun_moon"
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/third-body-point-mass-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

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

    fn hypersonic_hybrid_scenario() -> Scenario {
        Scenario::from_file(
            workspace_root().join("scenarios/hypersonic-hybrid-entry/scenario.toml"),
        )
        .expect("canonical hypersonic hybrid scenario must parse")
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

    fn f64_column(outcome: &RunOutcome, name: &str) -> Vec<f64> {
        let channel = outcome
            .table
            .schema()
            .channels()
            .iter()
            .find(|channel| channel.name == name)
            .unwrap_or_else(|| panic!("channel `{name}` must exist"));
        outcome
            .table
            .rows()
            .iter()
            .map(|row| match row.get(channel.id) {
                Some(TelemetryValue::Float64(value)) => *value,
                other => panic!("unexpected value in {name}: {other:?}"),
            })
            .collect()
    }

    #[test]
    fn point_mass_wires_aerothermal_diagnostics_into_force_stack() {
        let scenario = Scenario::from_toml_str(POINT_MASS_ENTRY_SCENARIO)
            .expect("point-mass live entry scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome =
            run(&scenario, &resolved_files, None).expect("point-mass live entry run succeeds");

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
    }

    #[test]
    fn point_mass_supports_motorless_aero_scenarios() {
        let mut scenario = niskanen_scenario();
        scenario.document.time.stop_s = 0.010;
        scenario.document.vehicle.initial_position_eci_m[2] = 10.0;
        scenario.document.propulsion = None;
        scenario.document.forces = Some(openbmp_scenario::ForcesConfig {
            models: vec!["gravity".to_owned(), "aero".to_owned()],
            phase_override: Vec::new(),
        });

        let resolved_files = scenario.resolved_files().expect("resolve aero deck");
        assert!(resolved_files.contains_key("aero.deck"));
        assert!(!resolved_files.contains_key("propulsion.motor.file"));

        let outcome = run(&scenario, &resolved_files, None).expect("motorless aero run succeeds");
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
    fn ussa76_aero_above_ceiling_uses_vacuum_fallback() {
        let scenario =
            Scenario::from_toml_str(USSA76_EXO_AERO_SCENARIO).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome =
            run(&scenario, &resolved_files, None).expect("USSA76 above ceiling should not fault");

        let aero_force_z = f64_column(&outcome, "force.aero.z_n");
        assert!(
            aero_force_z
                .iter()
                .all(|value| value.to_bits() == 0.0_f64.to_bits()),
            "USSA76 exo fallback should produce zero aero force above 86 km: {aero_force_z:?}"
        );
    }

    #[test]
    fn point_mass_auto_stops_on_ground_impact() {
        let scenario =
            Scenario::from_toml_str(GROUND_IMPACT_AUTO_STOP_SCENARIO).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("ground-impact run succeeds");

        assert_eq!(outcome.final_step, 1);
        assert!(matches!(
            outcome.stop_reason,
            StopReason::GroundImpact {
                ground_altitude_m,
                ..
            } if ground_altitude_m == 0.0
        ));
    }

    #[test]
    fn point_mass_contact_force_disables_ground_stop_and_publishes_force() {
        let scenario =
            Scenario::from_toml_str(CONTACT_POINT_MASS_SCENARIO).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("contact run succeeds");

        assert!(
            matches!(outcome.stop_reason, StopReason::EndTime { .. }),
            "contact scenario should run to end time, got {:?}",
            outcome.stop_reason
        );
        let contact_z = f64_column(&outcome, "force.contact.z_n");
        assert!(
            contact_z.iter().any(|value| *value > 0.0),
            "contact force should push upward for the initial penetration: {contact_z:?}"
        );
    }

    #[test]
    fn at_dynamic_pressure_event_stops_when_q_crosses_threshold() {
        let scenario =
            Scenario::from_toml_str(DYNAMIC_PRESSURE_EVENT_SCENARIO).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome =
            run(&scenario, &resolved_files, None).expect("dynamic-pressure event run succeeds");

        assert_eq!(outcome.final_step, 1);
        assert!(matches!(
            outcome.stop_reason,
            StopReason::MissionEnded { ref label, .. } if label == "q-rise"
        ));
    }

    #[test]
    fn point_mass_wires_lunar_third_body_perturbation() {
        let scenario = Scenario::from_toml_str(THIRD_BODY_POINT_MASS_SCENARIO)
            .expect("third-body scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("third-body run succeeds");

        let gravity_y = f64_column(&outcome, "force.gravity.y_n");
        assert!(
            gravity_y.iter().any(|value| value.abs() > 1.0e-8),
            "lunar third-body force should produce a measurable cross-axis component: {gravity_y:?}"
        );
    }

    #[test]
    fn deck_only_hypersonic_entry_is_rejected_with_hybrid_guidance() {
        let mut scenario = hypersonic_hybrid_scenario();
        scenario
            .document
            .aero
            .as_mut()
            .expect("scenario has aero")
            .method = None;
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let err =
            run(&scenario, &resolved_files, None).expect_err("deck-only Mach-20 entry rejects");

        assert!(
            matches!(&err, RunnerError::UnsupportedScenario { what }
                if what.contains("deck-only aero initial Mach")
                    && what.contains("kind = \"hybrid\"")),
            "unexpected error: {err:?}"
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
        let assembly = crate::assembly::synthesize_assembly(&scenario.document).expect("assembly");
        let motor = loaded_models.motor.as_ref().expect("motor loaded");
        let dry_mass_kg = crate::assembly::dry_mass_kg_at(
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

        let outcome = run(&scenario, &resolved_files, None).expect("run succeeds");
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
            phase_override: Vec::new(),
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
        let assembly = crate::assembly::synthesize_assembly(&scenario.document).expect("assembly");

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
            matches!(err, RunnerError::UnsupportedScenario { ref what }
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

        let outcome = run(&scenario, &resolved_files, None).expect("short parachute run succeeds");

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
