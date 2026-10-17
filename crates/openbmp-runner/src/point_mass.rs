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
//!   `.drag_area_m2`. Recovery devices with an active `inflation`,
//!   `drogue_inflation`, or `main_inflation` table additionally publish `.inflation_progress`,
//!   `.inflation_complete`, `.opening_dynamic_pressure_pa`,
//!   `.opening_load_n`, and `.opening_peak_load_n`; configured Pflanz
//!   tables additionally publish `.opening_ballistic_parameter_a`,
//!   `.opening_pflanz_reduction_factor_x1`, and
//!   `.opening_finite_mass_peak_load_n`.
//!   Devices with declared `risers` additionally publish aggregate
//!   `.riser_tension_sum_n`, `.riser_tension_max_n`, body-frame
//!   riser force components, body-frame riser moment components,
//!   and `.riser_elastic_energy_j`.
//!
//! The schema also carries the resolved-file SHA-256 digests as
//! Arrow schema metadata under `openbmp.scenario_files.<field>` keys.

use std::collections::BTreeMap;

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_aero::AeroDeck;
use openbmp_core::{ChannelId, Duration, ModelId, Position3, RecoveryId, SimTime, Velocity3};
use openbmp_physics::{AtmosphereModel, J2Gravity, PointMassGravity, WGS84_J2};
use openbmp_propulsion::{Motor, SolidMotor};
use openbmp_scenario::{RecoveryConfig, ResolvedFile, Scenario, ScenarioDocument};
use openbmp_sensors::SpecificForceTruth;
use openbmp_sim::{
    AnyStop, ConstantGravityForce, ConstantMass, EffectorActualsView, EndTime, EngineSnapshotView,
    ForceContext, ForceModel, GroundImpact, MassModel, RecoverySnapshotView, ScenarioScriptAction,
    SimulationConfig, SimulationKernel, StopReason, TankSnapshotView,
};
use openbmp_state::PointMassState;
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use openbmp_vehicle::{
    AeroMethodForceAdapter, BoxedMassModel, DeckDragForceAdapter, EngineClusterForceAdapter,
    EngineClusterMassAdapter, GravityForceAdapter, KernelVehicle, MotorMassAdapter,
    MotorThrustForceAdapter, NamedForceModel, RecoveryRackForceAdapter, RecoveryRackMassAdapter,
    RecoveryRiserForceAdapter, RecoverySequenceForceAdapter, TankRackForceAdapter,
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
const POINT_MASS_RECOVERY_RACK_MASS_MODEL_ID: ModelId = ModelId::new(371);
const POINT_MASS_RECOVERY_RISER_FORCE_MODEL_ID: ModelId = ModelId::new(372);
const POINT_MASS_RECOVERY_SEQUENCE_FORCE_MODEL_ID: ModelId = ModelId::new(373);
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
    let mut engine_rack = crate::engines::EngineRack::build(document, resolved_files)?;
    // Build the runner-side tank rack. Empty when no
    // `[[vehicle.assembly.tanks]]` are declared, in which case every
    // per-step rack operation short-circuits and the legacy
    // byte-stable path is preserved.
    let mut tank_rack = crate::tanks::TankRack::build(document)?;
    let propellant_budget = crate::propulsion::build_propellant_budget(document)?;
    let mut feed_network_rack =
        crate::feed_network::FeedNetworkRack::build(document, resolved_files)?;
    let pogo = crate::pogo::PogoStabilityRack::build(document)?.into_report();
    let countdown = crate::ground::evaluate_countdown_document(document)?;
    // Build the runner-side recovery rack. Empty when no
    // `[[vehicle.assembly.recovery]]` are declared, in which case
    // every per-step rack operation short-circuits and the kernel's
    // recovery snapshot stays empty (byte-stable for scenarios
    // without recovery devices).
    let mut recovery_rack = crate::recovery::RecoveryRack::build(document, resolved_files)?;
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
    let contact_evaluator = document
        .contact
        .as_ref()
        .map(crate::contact::ContactDiagnosticsEvaluator::from_config)
        .transpose()?;
    let kernel_vehicle = build_vehicle(
        document,
        &loaded_models,
        &assembly,
        resolved_files,
        aerothermal_sink.clone(),
        contact_evaluator.as_ref(),
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
        contact_evaluator.as_ref(),
    )?;
    let mass_model = BoxedMassModel(build_mass_model(document, &loaded_models, &assembly)?);

    // Runtime integrator dispatch from the scenario
    // [solver] block. Defaults to Rk4FixedStep when [solver] is
    // absent, preserving the byte-stable contract for every
    // existing scenario.
    let runtime_integrator = build_runtime_integrator(document)?;
    let kernel_step_s = crate::contact::kernel_step_s(document);
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
        dt: Duration::from_seconds(kernel_step_s),
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
    if let Some(contact) = &contact_evaluator {
        contact
            .set_active(
                breakdown_vehicle
                    .model_active(kernel.current_phase().map(|phase| phase.value()), "contact"),
                kernel.current_time(),
            )
            .map_err(openbmp_sim::SimulationError::from)?;
        let state = kernel.current_state();
        contact
            .commit_accepted(state.position.vector, state.velocity.vector, state.time)
            .map_err(openbmp_sim::SimulationError::from)?;
    }
    let channel_set = PointMassChannelSet::new(document)?;
    let mut acoustics_report = crate::acoustics::evaluate_document(document, resolved_files)?;
    let plume_evaluator =
        crate::plume::PointMassPlumeEvaluator::maybe_new(document, loaded_models.motor.as_ref())?;
    let mut contact_accumulator = document
        .contact
        .as_ref()
        .map(crate::contact::ContactRunAccumulator::from_config)
        .transpose()?;
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
    let mut fc_bridge = crate::fc_bridge::FcBridge::maybe_new(scenario, resolved_files, None)?;
    let mut mission_region_trace =
        crate::MissionRegionTraceState::new(&crate::mission_region_declarations(document));
    let mut afts_monitor = crate::afts::RunnerAftsMonitor::maybe_new(document)?;
    let mut comm_accumulator =
        crate::comm::CommRunAccumulator::maybe_new(document, resolved_files)?;
    let initial_aerothermal_electron_number_density_m3 = aerothermal_driver
        .as_ref()
        .map(|driver| driver.output().electron_number_density_m3);
    let initial_comm_observation = comm_accumulator
        .as_mut()
        .map(|accumulator| {
            let state = kernel.current_state();
            accumulator.observe_eci(crate::comm::CommEciObservationInput {
                frame: &frame,
                step: kernel.current_step(),
                time: state.time,
                position_eci: state.position,
                velocity_eci: state.velocity,
                body_to_eci: UnitQuaternion::identity(),
                aerothermal_electron_number_density_m3:
                    initial_aerothermal_electron_number_density_m3,
            })
        })
        .transpose()?;
    record_step(
        document,
        &mut table,
        &kernel,
        &channel_set,
        &breakdown_vehicle,
        contact_evaluator.as_ref(),
        contact_accumulator.as_mut(),
        breakdown_atmosphere.as_ref(),
        plume_evaluator.as_ref(),
        geocentric_surface_radius_m,
        aerothermal_driver.as_ref().map(|driver| driver.output()),
        fc_bridge.as_ref(),
        initial_comm_observation.as_ref(),
        acoustics_report.as_ref(),
        countdown.as_ref(),
        &[],
        &mut mission_region_trace,
        &initial_snapshot,
    )?;
    if let Some(monitor) = &mut afts_monitor {
        monitor.observe_point_mass(
            kernel.current_step().value(),
            kernel.current_time().as_seconds(),
            kernel.current_state(),
        )?;
    }
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
            let environment = kernel.current_environment_sample()?;
            let gravity = environment.gravity_eci_m_s2;
            let specific_force_truth = if bridge.uses_force_accumulator_truth() {
                Some(point_mass_specific_force_truth(
                    &kernel,
                    &breakdown_vehicle,
                )?)
            } else {
                None
            };
            let propellant_state = crate::fc_bridge::propellant_state_from_tanks(
                kernel.current_time(),
                &tank_rack.propellant_tank_states(document),
            );
            let comm_link_effects = comm_accumulator
                .as_mut()
                .map(|accumulator| {
                    accumulator.bridge_packet_effects(document.time.seed, kernel.current_step())
                })
                .transpose()?
                .flatten();
            bridge.set_comm_link_effects(comm_link_effects);
            bridge.tick_point_mass(
                kernel.current_state(),
                kernel.current_step(),
                &environment,
                gravity,
                specific_force_truth,
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
        let propellant_safing_command = fc_bridge
            .as_ref()
            .and_then(crate::fc_bridge::FcBridge::latest_propellant_safing_command);
        if !tank_rack.is_empty() {
            tank_rack.apply_propellant_safing_command(propellant_safing_command);
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
            feed_network_rack.apply_propellant_safing_command(propellant_safing_command);
            feed_network_rack.apply_to_report(&mut report, kernel_step_s, kernel.current_step())?;
            crate::propulsion::apply_propellant_safing_to_report(
                &mut report,
                propellant_safing_command,
            );
            tank_rack.set_propellant_budget_drain_rates(report.tank_drain_rates_kg_per_s.clone());
            engine_rack.apply_propellant_budget(&report)?;
        }
        if !effector_rack.is_empty() {
            let tank_states = tank_rack.propellant_tank_states(document);
            effector_rack.step_with_tanks(kernel.current_time(), &tank_states)?;
        }
        if !tank_rack.is_empty() {
            tank_rack.set_rcs_feed_drain_rates(effector_rack.rcs_feed_drain_rates());
        }
        if !engine_rack.is_empty() {
            let cavitation_events = feed_network_rack.cavitation_events();
            engine_rack.apply_cavitation_faults(&cavitation_events)?;
            engine_rack.apply_scheduled_faults(kernel.current_step())?;
            engine_rack.step()?;
        }
        crate::acoustics::observe_source_engine_runtime_sample(
            &mut acoustics_report,
            kernel.current_step().value(),
            kernel.current_time().as_seconds(),
            &engine_rack.propulsion_snapshot_map(),
        )?;
        // Advance the tank rack using prior-step cached
        // drivers (set after the previous kernel step). For point-
        // mass kernels the drivers are zeros — slosh in point-mass
        // is RigidLiquid-only per the scenario validator (D10), so
        // the dynamic drivers are irrelevant.
        if !tank_rack.is_empty() {
            if tank_rack.uses_safe_dump_ambient_backpressure() {
                let environment = kernel.current_environment_sample()?;
                tank_rack
                    .set_safe_dump_ambient_backpressure_pa(environment.atmosphere_pressure_pa)?;
            }
            tank_rack.step()?;
        }
        // Drain pending deploy/stow events from the prior
        // kernel step, apply to the rack, then advance internal state
        // (no-op for the instantaneous-deploy models).
        if !recovery_rack.is_empty() {
            recovery_rack.apply_deploys(&pending_recovery_events)?;
            recovery_rack.step(kernel_step_s)?;
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
        // Push recovery snapshot after wind sampling so opening-load
        // diagnostics use the same air-relative velocity the recovery
        // force adapter will see during this kernel step.
        if !recovery_rack.is_empty() {
            let environment = kernel.current_environment_sample()?;
            let context = crate::recovery::recovery_opening_load_context_with_mass(
                kernel.current_state().velocity.vector,
                &environment,
                kernel.current_state().mass.get::<kilogram>(),
            )?;
            kernel.set_recovery_snapshot(
                recovery_rack.snapshot_map_with_opening_load_context(context)?,
            );
        }
        if let Some(contact) = &contact_evaluator {
            let checkpoint = contact
                .snapshot()
                .map_err(openbmp_sim::SimulationError::from)?;
            let result = (|| -> Result<(), openbmp_sim::SimulationError> {
                // FC authority may have changed the phase before this step.
                contact.set_active(
                    breakdown_vehicle
                        .model_active(kernel.current_phase().map(|phase| phase.value()), "contact"),
                    kernel.current_time(),
                )?;
                let state = kernel.current_state();
                contact.commit_accepted(
                    state.position.vector,
                    state.velocity.vector,
                    state.time,
                )?;
                kernel.step_with_accepted_steps(|state, _active_body| {
                    contact.commit_accepted(
                        state.position.vector,
                        state.velocity.vector,
                        state.time,
                    )
                })?;
                contact.retime_accepted(kernel.current_time())?;
                // Kernel mission events apply after the preceding interval.
                // Start the next force gate at this endpoint with zero elapsed time.
                contact.set_active(
                    breakdown_vehicle
                        .model_active(kernel.current_phase().map(|phase| phase.value()), "contact"),
                    kernel.current_time(),
                )?;
                let state = kernel.current_state();
                contact.commit_accepted(
                    state.position.vector,
                    state.velocity.vector,
                    state.time,
                )?;
                Ok(())
            })();
            if let Err(err) = result {
                contact
                    .restore(checkpoint)
                    .map_err(openbmp_sim::SimulationError::from)?;
                return Err(err.into());
            }
        } else {
            kernel.step()?;
        }
        let mission_fired = kernel.drain_mission_fired_events();
        let script_fired = kernel.drain_script_fired_events();
        crate::acoustics::observe_source_engine_ignitions(&mut acoustics_report, &script_fired);
        if let Some(driver) = &mut aerothermal_driver {
            let environment = kernel.current_environment_sample()?;
            driver.evaluate_point_mass(kernel.current_state(), &environment, kernel_step_s)?;
        }
        let snapshot = effector_rack.snapshot();
        let aerothermal_electron_number_density_m3 = aerothermal_driver
            .as_ref()
            .map(|driver| driver.output().electron_number_density_m3);
        let comm_observation = comm_accumulator
            .as_mut()
            .map(|accumulator| {
                let state = kernel.current_state();
                accumulator.observe_eci(crate::comm::CommEciObservationInput {
                    frame: &frame,
                    step: kernel.current_step(),
                    time: state.time,
                    position_eci: state.position,
                    velocity_eci: state.velocity,
                    body_to_eci: UnitQuaternion::identity(),
                    aerothermal_electron_number_density_m3,
                })
            })
            .transpose()?;
        record_step(
            document,
            &mut table,
            &kernel,
            &channel_set,
            &breakdown_vehicle,
            contact_evaluator.as_ref(),
            contact_accumulator.as_mut(),
            breakdown_atmosphere.as_ref(),
            plume_evaluator.as_ref(),
            geocentric_surface_radius_m,
            aerothermal_driver.as_ref().map(|driver| driver.output()),
            fc_bridge.as_ref(),
            comm_observation.as_ref(),
            acoustics_report.as_ref(),
            countdown.as_ref(),
            &mission_fired,
            &mut mission_region_trace,
            &snapshot,
        )?;
        if let Some(monitor) = &mut afts_monitor {
            monitor.observe_point_mass(
                kernel.current_step().value(),
                kernel.current_time().as_seconds(),
                kernel.current_state(),
            )?;
        }
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
    let contact = contact_accumulator
        .map(crate::contact::ContactRunAccumulator::finish)
        .transpose()?
        .flatten();
    let mut upstream_uq = engine_rack.upstream_uq_budget();
    upstream_uq
        .sources
        .extend(crate::aero::upstream_uq_budget(document)?.sources);
    upstream_uq
        .sources
        .extend(crate::aerothermal::upstream_uq_budget(document)?.sources);
    let comm = comm_accumulator
        .map(crate::comm::CommRunAccumulator::finish)
        .transpose()?;

    Ok(RunOutcome {
        final_step: kernel.current_step().value(),
        final_time_s: kernel.current_time().as_seconds(),
        stop_reason,
        table,
        realtime: realtime_pacer.finish(),
        actuator_stream,
        contact,
        landing_gear: None,
        afts: afts_monitor.map(crate::afts::RunnerAftsMonitor::finish),
        comm,
        acoustics: acoustics_report,
        structural_acoustics: None,
        structural_events: None,
        pogo,
        csi: None,
        countdown,
        upstream_uq,
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
        "constant" | "point_mass" | "j2" | "egm2008" | "third_body" | "tesseral"
    ) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "environment.gravity = {} (wired: constant, point_mass, j2, egm2008, third_body, tesseral)",
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
/// (`point_mass`, `j2`, `egm2008`, `tesseral`) route through
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
            let model = crate::celestial::build_egm2008_gravity(document, resolved_files)?;
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
        "tesseral" => {
            let model = crate::celestial::build_tesseral_gravity(document, resolved_files)?;
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

fn riser_recovery_ids(document: &ScenarioDocument) -> Vec<RecoveryId> {
    document
        .vehicle
        .assembly
        .recovery
        .iter()
        .filter(|recovery| !recovery.risers.is_empty())
        .map(|recovery| {
            RecoveryId::from_path(&format!("vehicle.assembly.recovery.{id}", id = recovery.id))
        })
        .collect()
}

fn sequence_force_recovery_ids(document: &ScenarioDocument) -> Vec<RecoveryId> {
    document
        .vehicle
        .assembly
        .recovery
        .iter()
        .filter(|recovery| {
            recovery
                .sequence
                .as_ref()
                .is_some_and(|sequence| sequence.mortar_pack_mass_kg.is_some())
        })
        .map(|recovery| {
            RecoveryId::from_path(&format!("vehicle.assembly.recovery.{id}", id = recovery.id))
        })
        .collect()
}

#[allow(clippy::too_many_lines)] // the recovery-rack force-adapter wiring branch is large
fn build_vehicle(
    document: &ScenarioDocument,
    loaded_models: &LoadedModels,
    assembly: &Assembly,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    aerothermal_sink: Option<crate::aerothermal::LiveAerothermalSink>,
    contact_evaluator: Option<&crate::contact::ContactDiagnosticsEvaluator>,
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
                    contact_evaluator.ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "forces includes `contact` but shared contact runtime is missing"
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

    // Declared-riser aggregate line-force adapter. This consumes the recovery
    // rack's body-frame riser-force snapshot as a separate force-breakdown
    // component; it does not replace aerodynamic recovery drag.
    let recovery_riser_ids = riser_recovery_ids(document);
    if !recovery_riser_ids.is_empty() {
        let recovery_riser_force = RecoveryRiserForceAdapter::new(
            recovery_riser_ids,
            POINT_MASS_RECOVERY_RISER_FORCE_MODEL_ID,
        );
        named.push(NamedForceModel::new(
            "recovery_riser_force",
            Box::new(recovery_riser_force),
        ));
    }

    let recovery_sequence_force_ids = sequence_force_recovery_ids(document);
    if !recovery_sequence_force_ids.is_empty() {
        let recovery_sequence_force = RecoverySequenceForceAdapter::new(
            recovery_sequence_force_ids,
            POINT_MASS_RECOVERY_SEQUENCE_FORCE_MODEL_ID,
        );
        named.push(NamedForceModel::new(
            "recovery_sequence_force",
            Box::new(recovery_sequence_force),
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
    let mass: Box<dyn MassModel> = if document.vehicle.assembly.tanks.is_empty() {
        base
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
        Box::new(TankRackMassAdapter::new(
            base,
            tank_ids,
            RIGID_BODY_TANK_RACK_MASS_MODEL_ID,
        ))
    };
    let recovery_ids = apparent_mass_recovery_ids(document);
    if recovery_ids.is_empty() {
        Ok(mass)
    } else {
        Ok(Box::new(RecoveryRackMassAdapter::new(
            mass,
            recovery_ids,
            POINT_MASS_RECOVERY_RACK_MASS_MODEL_ID,
        )))
    }
}

fn apparent_mass_recovery_ids(document: &ScenarioDocument) -> Vec<RecoveryId> {
    document
        .vehicle
        .assembly
        .recovery
        .iter()
        .filter(|recovery| recovery_has_apparent_mass(recovery))
        .map(|recovery| {
            RecoveryId::from_path(&format!("vehicle.assembly.recovery.{id}", id = recovery.id))
        })
        .collect()
}

fn recovery_has_inflation(recovery: &RecoveryConfig) -> bool {
    recovery.inflation.is_some()
        || recovery.drogue_inflation.is_some()
        || recovery.main_inflation.is_some()
}

fn recovery_has_apparent_mass(recovery: &RecoveryConfig) -> bool {
    [
        recovery.inflation.as_ref(),
        recovery.drogue_inflation.as_ref(),
        recovery.main_inflation.as_ref(),
    ]
    .into_iter()
    .flatten()
    .any(|inflation| inflation.apparent_mass_coeff_k_a.is_some())
}

fn recovery_has_pflanz(recovery: &RecoveryConfig) -> bool {
    [
        recovery.inflation.as_ref(),
        recovery.drogue_inflation.as_ref(),
        recovery.main_inflation.as_ref(),
    ]
    .into_iter()
    .flatten()
    .any(|inflation| inflation.pflanz.is_some())
}

fn recovery_has_sequence(recovery: &RecoveryConfig) -> bool {
    recovery.sequence.is_some()
}

fn recovery_sequence_has_mortar(recovery: &RecoveryConfig) -> bool {
    recovery
        .sequence
        .as_ref()
        .is_some_and(|sequence| sequence.mortar_pack_mass_kg.is_some())
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
type RecoveryTelemetryChannels = Vec<RecoveryTelemetryChannelSet>;

#[derive(Debug)]
struct RecoveryTelemetryChannelSet {
    id: RecoveryId,
    deployed: TelemetryChannel<bool>,
    phase_index: TelemetryChannel<i64>,
    drag_area: TelemetryChannel<f64>,
    inflation: Option<RecoveryInflationTelemetryChannels>,
    sequence: Option<RecoverySequenceTelemetryChannels>,
    riser: Option<RecoveryRiserTelemetryChannels>,
}

#[derive(Debug)]
struct RecoveryInflationTelemetryChannels {
    progress: TelemetryChannel<f64>,
    complete: TelemetryChannel<bool>,
    dynamic_pressure: TelemetryChannel<f64>,
    opening_load: TelemetryChannel<f64>,
    opening_peak_load: TelemetryChannel<f64>,
    apparent_mass: Option<RecoveryApparentMassTelemetryChannels>,
    pflanz: Option<RecoveryPflanzTelemetryChannels>,
}

#[derive(Debug)]
struct RecoveryApparentMassTelemetryChannels {
    apparent_mass: TelemetryChannel<f64>,
    apparent_mass_rate: TelemetryChannel<f64>,
}

#[derive(Debug)]
struct RecoveryPflanzTelemetryChannels {
    ballistic_parameter: TelemetryChannel<f64>,
    reduction_factor: TelemetryChannel<f64>,
    finite_mass_peak_load: TelemetryChannel<f64>,
}

#[derive(Debug)]
struct RecoverySequenceTelemetryChannels {
    delta_v: TelemetryChannel<f64>,
    stiffness: TelemetryChannel<f64>,
    effective_mass: TelemetryChannel<f64>,
    pack_drag_load: TelemetryChannel<f64>,
    peak_load: TelemetryChannel<f64>,
    mortar: Option<RecoveryMortarTelemetryChannels>,
}

#[derive(Debug)]
struct RecoveryMortarTelemetryChannels {
    impulse_body_x: TelemetryChannel<f64>,
    impulse_body_y: TelemetryChannel<f64>,
    impulse_body_z: TelemetryChannel<f64>,
    force_body_x: TelemetryChannel<f64>,
    force_body_y: TelemetryChannel<f64>,
    force_body_z: TelemetryChannel<f64>,
}

#[derive(Debug)]
struct RecoveryRiserTelemetryChannels {
    tension_sum: TelemetryChannel<f64>,
    tension_max: TelemetryChannel<f64>,
    force_body_x: TelemetryChannel<f64>,
    force_body_y: TelemetryChannel<f64>,
    force_body_z: TelemetryChannel<f64>,
    moment_body_x: TelemetryChannel<f64>,
    moment_body_y: TelemetryChannel<f64>,
    moment_body_z: TelemetryChannel<f64>,
    elastic_energy: TelemetryChannel<f64>,
}

#[derive(Debug)]
struct AerothermalTelemetryChannels {
    q_conv: TelemetryChannel<f64>,
    q_rad: TelemetryChannel<f64>,
    h_aw: TelemetryChannel<f64>,
    recovery_temperature: TelemetryChannel<f64>,
    stagnation_temperature: TelemetryChannel<f64>,
    electron_number_density: Option<TelemetryChannel<f64>>,
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
    comm: Option<crate::comm::CommTelemetryChannels>,
    acoustics: Option<crate::acoustics::AcousticsTelemetryChannels>,
    countdown: Option<crate::ground::GroundCountdownTelemetryChannels>,
    has_atmosphere: bool,
    atmosphere_density: Option<TelemetryChannel<f64>>,
    atmosphere_pressure: Option<TelemetryChannel<f64>>,
    atmosphere_temperature: Option<TelemetryChannel<f64>>,
    atmosphere_speed_of_sound: Option<TelemetryChannel<f64>>,
    /// Latest FC guidance reference, present only for `[fc]` scenarios.
    fc_reference: Option<FcReferenceTelemetryChannels>,
    /// Force-model components in declared order.
    force_components: ForceComponentChannels,
    /// Contact diagnostics, present only for `[contact]` scenarios.
    contact: Option<crate::contact::ContactTelemetryChannels>,
    /// Active per-phase model list, present when phase overrides are
    /// declared.
    active_models: Option<TelemetryChannel<String>>,
    /// Live aerothermal diagnostic channels.
    aerothermal: Option<AerothermalTelemetryChannels>,
    /// Live plume-similarity diagnostic channels.
    plume: Option<crate::plume::PlumeTelemetryChannels>,
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
        let comm = crate::comm::CommTelemetryChannels::maybe_new(document, &mut alloc)?;
        let acoustics =
            crate::acoustics::AcousticsTelemetryChannels::maybe_new(document, &mut alloc)?;
        let countdown =
            crate::ground::GroundCountdownTelemetryChannels::maybe_new(document, &mut alloc)?;

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

        // Per-model force breakdown channels: scenario-declared force
        // universe plus runner-internal force components, in the same order
        // the kernel vehicle registers them for the force sum.
        let force_model_universe = crate::force_breakdown_model_universe(document);
        let mut force_components = Vec::with_capacity(force_model_universe.len());
        for name in force_model_universe {
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

        let contact = document
            .contact
            .is_some()
            .then(|| crate::contact::ContactTelemetryChannels::new(&mut alloc))
            .transpose()?;

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
                electron_number_density: document
                    .aerothermal
                    .as_ref()
                    .and_then(|aerothermal| aerothermal.electron_density_proxy.as_ref())
                    .map(|_| {
                        TelemetryChannel::<f64>::new(
                            alloc(),
                            "aerothermal.electron_number_density_m3",
                            "1/m^3",
                            None::<&str>,
                        )
                    })
                    .transpose()?,
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

        let plume = document
            .aero
            .as_ref()
            .and_then(|aero| aero.plume.as_ref())
            .is_some()
            .then(|| crate::plume::PlumeTelemetryChannels::new(&mut alloc))
            .transpose()?;

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

        // Recovery telemetry channels per declared device. Scenario-
        // declared order matches the recovery force-adapter operand order.
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
            let inflation = if recovery_has_inflation(config) {
                Some(RecoveryInflationTelemetryChannels {
                    progress: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("recovery.{}.inflation_progress", config.id),
                        "1",
                        None::<&str>,
                    )?,
                    complete: TelemetryChannel::<bool>::new(
                        alloc(),
                        format!("recovery.{}.inflation_complete", config.id),
                        "bool",
                        None::<&str>,
                    )?,
                    dynamic_pressure: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("recovery.{}.opening_dynamic_pressure_pa", config.id),
                        "Pa",
                        None::<&str>,
                    )?,
                    opening_load: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("recovery.{}.opening_load_n", config.id),
                        "N",
                        None::<&str>,
                    )?,
                    opening_peak_load: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("recovery.{}.opening_peak_load_n", config.id),
                        "N",
                        None::<&str>,
                    )?,
                    apparent_mass: if recovery_has_apparent_mass(config) {
                        Some(RecoveryApparentMassTelemetryChannels {
                            apparent_mass: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.apparent_mass_kg", config.id),
                                "kg",
                                None::<&str>,
                            )?,
                            apparent_mass_rate: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.apparent_mass_rate_kg_s", config.id),
                                "kg/s",
                                None::<&str>,
                            )?,
                        })
                    } else {
                        None
                    },
                    pflanz: if recovery_has_pflanz(config) {
                        Some(RecoveryPflanzTelemetryChannels {
                            ballistic_parameter: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.opening_ballistic_parameter_a", config.id),
                                "1",
                                None::<&str>,
                            )?,
                            reduction_factor: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!(
                                    "recovery.{}.opening_pflanz_reduction_factor_x1",
                                    config.id
                                ),
                                "1",
                                None::<&str>,
                            )?,
                            finite_mass_peak_load: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.opening_finite_mass_peak_load_n", config.id),
                                "N",
                                None::<&str>,
                            )?,
                        })
                    } else {
                        None
                    },
                })
            } else {
                None
            };
            let sequence = if recovery_has_sequence(config) {
                Some(RecoverySequenceTelemetryChannels {
                    delta_v: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("recovery.{}.snatch_delta_v_m_s", config.id),
                        "m/s",
                        None::<&str>,
                    )?,
                    stiffness: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("recovery.{}.snatch_stiffness_n_m", config.id),
                        "N/m",
                        None::<&str>,
                    )?,
                    effective_mass: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("recovery.{}.snatch_effective_mass_kg", config.id),
                        "kg",
                        None::<&str>,
                    )?,
                    pack_drag_load: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("recovery.{}.snatch_pack_drag_load_n", config.id),
                        "N",
                        None::<&str>,
                    )?,
                    peak_load: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("recovery.{}.snatch_peak_load_n", config.id),
                        "N",
                        None::<&str>,
                    )?,
                    mortar: if recovery_sequence_has_mortar(config) {
                        Some(RecoveryMortarTelemetryChannels {
                            impulse_body_x: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.mortar_impulse_body_x_n_s", config.id),
                                "N*s",
                                None::<&str>,
                            )?,
                            impulse_body_y: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.mortar_impulse_body_y_n_s", config.id),
                                "N*s",
                                None::<&str>,
                            )?,
                            impulse_body_z: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.mortar_impulse_body_z_n_s", config.id),
                                "N*s",
                                None::<&str>,
                            )?,
                            force_body_x: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.mortar_force_body_x_n", config.id),
                                "N",
                                None::<&str>,
                            )?,
                            force_body_y: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.mortar_force_body_y_n", config.id),
                                "N",
                                None::<&str>,
                            )?,
                            force_body_z: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.mortar_force_body_z_n", config.id),
                                "N",
                                None::<&str>,
                            )?,
                        })
                    } else {
                        None
                    },
                })
            } else {
                None
            };
            let riser = (!config.risers.is_empty())
                .then(|| {
                    Ok::<RecoveryRiserTelemetryChannels, RunnerError>(
                        RecoveryRiserTelemetryChannels {
                            tension_sum: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.riser_tension_sum_n", config.id),
                                "N",
                                None::<&str>,
                            )?,
                            tension_max: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.riser_tension_max_n", config.id),
                                "N",
                                None::<&str>,
                            )?,
                            force_body_x: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.riser_force_body_x_n", config.id),
                                "N",
                                None::<&str>,
                            )?,
                            force_body_y: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.riser_force_body_y_n", config.id),
                                "N",
                                None::<&str>,
                            )?,
                            force_body_z: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.riser_force_body_z_n", config.id),
                                "N",
                                None::<&str>,
                            )?,
                            moment_body_x: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.riser_moment_body_x_n_m", config.id),
                                "N*m",
                                None::<&str>,
                            )?,
                            moment_body_y: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.riser_moment_body_y_n_m", config.id),
                                "N*m",
                                None::<&str>,
                            )?,
                            moment_body_z: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.riser_moment_body_z_n_m", config.id),
                                "N*m",
                                None::<&str>,
                            )?,
                            elastic_energy: TelemetryChannel::<f64>::new(
                                alloc(),
                                format!("recovery.{}.riser_elastic_energy_j", config.id),
                                "J",
                                None::<&str>,
                            )?,
                        },
                    )
                })
                .transpose()?;
            recovery_states.push(RecoveryTelemetryChannelSet {
                id,
                deployed,
                phase_index,
                drag_area,
                inflation,
                sequence,
                riser,
            });
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
            comm,
            acoustics,
            countdown,
            has_atmosphere,
            atmosphere_density,
            atmosphere_pressure,
            atmosphere_temperature,
            atmosphere_speed_of_sound,
            fc_reference,
            force_components,
            contact,
            active_models,
            aerothermal,
            plume,
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
        if let Some(comm) = &self.comm {
            comm.push_metadata(&mut channels);
        }
        if let Some(acoustics) = &self.acoustics {
            acoustics.push_metadata(&mut channels);
        }
        if let Some(countdown) = &self.countdown {
            countdown.push_metadata(&mut channels);
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
        if let Some(contact) = &self.contact {
            contact.push_metadata(&mut channels);
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
            if let Some(channel) = &aerothermal.electron_number_density {
                channels.push(channel.metadata().clone());
            }
            channels.push(aerothermal.knudsen.metadata().clone());
            channels.push(aerothermal.wall_temperature.metadata().clone());
            channels.push(aerothermal.backwall_temperature.metadata().clone());
            channels.push(aerothermal.recession_depth.metadata().clone());
            channels.push(aerothermal.gas_mdot.metadata().clone());
            channels.push(aerothermal.mass_loss.metadata().clone());
        }
        if let Some(plume) = &self.plume {
            plume.push_metadata(&mut channels);
        }
        // Effector deflection channels, in scenario-declared
        // order. Allocated AFTER force breakdown channels and BEFORE
        // mission markers — this ordering is the determinism contract.
        for actual in &self.effector_actuals {
            channels.push(actual.metadata().clone());
        }
        // Recovery channels, in scenario-declared order,
        // between effectors and mission markers.
        for recovery in &self.recovery_states {
            channels.push(recovery.deployed.metadata().clone());
            channels.push(recovery.phase_index.metadata().clone());
            channels.push(recovery.drag_area.metadata().clone());
            if let Some(inflation) = &recovery.inflation {
                channels.push(inflation.progress.metadata().clone());
                channels.push(inflation.complete.metadata().clone());
                channels.push(inflation.dynamic_pressure.metadata().clone());
                channels.push(inflation.opening_load.metadata().clone());
                channels.push(inflation.opening_peak_load.metadata().clone());
                if let Some(apparent_mass) = &inflation.apparent_mass {
                    channels.push(apparent_mass.apparent_mass.metadata().clone());
                    channels.push(apparent_mass.apparent_mass_rate.metadata().clone());
                }
                if let Some(pflanz) = &inflation.pflanz {
                    channels.push(pflanz.ballistic_parameter.metadata().clone());
                    channels.push(pflanz.reduction_factor.metadata().clone());
                    channels.push(pflanz.finite_mass_peak_load.metadata().clone());
                }
            }
            if let Some(sequence) = &recovery.sequence {
                channels.push(sequence.delta_v.metadata().clone());
                channels.push(sequence.stiffness.metadata().clone());
                channels.push(sequence.effective_mass.metadata().clone());
                channels.push(sequence.pack_drag_load.metadata().clone());
                channels.push(sequence.peak_load.metadata().clone());
                if let Some(mortar) = &sequence.mortar {
                    channels.push(mortar.impulse_body_x.metadata().clone());
                    channels.push(mortar.impulse_body_y.metadata().clone());
                    channels.push(mortar.impulse_body_z.metadata().clone());
                    channels.push(mortar.force_body_x.metadata().clone());
                    channels.push(mortar.force_body_y.metadata().clone());
                    channels.push(mortar.force_body_z.metadata().clone());
                }
            }
            if let Some(riser) = &recovery.riser {
                channels.push(riser.tension_sum.metadata().clone());
                channels.push(riser.tension_max.metadata().clone());
                channels.push(riser.force_body_x.metadata().clone());
                channels.push(riser.force_body_y.metadata().clone());
                channels.push(riser.force_body_z.metadata().clone());
                channels.push(riser.moment_body_x.metadata().clone());
                channels.push(riser.moment_body_y.metadata().clone());
                channels.push(riser.moment_body_z.metadata().clone());
                channels.push(riser.elastic_energy.metadata().clone());
            }
        }
        // Marker channels last, in alphabetical (BTreeMap) order.
        for marker in self.mission_markers.values() {
            channels.push(marker.metadata().clone());
        }
        Ok(TelemetrySchema::new(channels)?.with_metadata(metadata))
    }
}

#[allow(clippy::too_many_arguments)]
fn point_mass_specific_force_truth<I, F, MM, E, SC>(
    kernel: &SimulationKernel<PointMassState, I, F, MM, E, SC>,
    breakdown_vehicle: &KernelVehicle<PointMassState>,
) -> Result<SpecificForceTruth, RunnerError>
where
    I: openbmp_sim::Integrator<PointMassState>,
    F: ForceModel<PointMassState>,
    MM: openbmp_sim::MassModel,
    E: openbmp_sim::EnvironmentModel,
    SC: openbmp_sim::StopCondition<PointMassState>,
{
    let state = kernel.current_state();
    let environment = kernel.current_environment_sample()?;
    let mass_kg = state.mass.get::<kilogram>();
    let ctx = ForceContext {
        state,
        environment: &environment,
        mass_kg,
        time: state.time,
        active_body: None,
        phase_id: kernel.current_phase().map(|phase| phase.value()),
        effector_actuals: EffectorActualsView::new(kernel.effector_actuals()),
        engine_snapshot: EngineSnapshotView::new(kernel.engine_snapshot()),
        tank_snapshot: TankSnapshotView::new(kernel.tank_snapshot()),
        recovery_snapshot: RecoverySnapshotView::new(kernel.recovery_snapshot()),
    };
    let breakdown = breakdown_vehicle
        .evaluate_force_breakdown(ctx)
        .map_err(|e| RunnerError::UnsupportedScenario {
            what: format!("force-accumulator sensor truth evaluation failed: {e}"),
        })?;
    let gravity_force = breakdown
        .components
        .iter()
        .filter(|(name, _)| name == "gravity")
        .fold(Vector3::zeros(), |sum, (_, force)| sum + *force);
    Ok(SpecificForceTruth {
        f_cg_body_m_s2: (breakdown.total - gravity_force) / mass_kg,
        omega_body_rad_s: Vector3::zeros(),
        alpha_body_rad_s2: Vector3::zeros(),
    })
}

#[allow(clippy::too_many_arguments)]
fn record_step<I, F, MM, E, SC>(
    document: &ScenarioDocument,
    table: &mut TelemetryTable,
    kernel: &SimulationKernel<PointMassState, I, F, MM, E, SC>,
    channels: &PointMassChannelSet,
    breakdown_vehicle: &KernelVehicle<PointMassState>,
    contact_evaluator: Option<&crate::contact::ContactDiagnosticsEvaluator>,
    contact_accumulator: Option<&mut crate::contact::ContactRunAccumulator>,
    breakdown_atmosphere: Option<&RuntimeAtmosphere>,
    plume_evaluator: Option<&crate::plume::PointMassPlumeEvaluator<'_>>,
    geocentric_surface_radius_m: Option<f64>,
    aerothermal: Option<&crate::aerothermal::LiveAerothermalOutput>,
    fc_bridge: Option<&crate::fc_bridge::FcBridge>,
    comm_observation: Option<&crate::comm::CommObservation>,
    acoustics_report: Option<&crate::acoustics::AcousticsRunReport>,
    countdown_report: Option<&crate::ground::GroundCountdownRunReport>,
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
    if let (Some(comm_channels), Some(observation)) = (&channels.comm, comm_observation) {
        comm_channels.insert_samples(&mut row, observation)?;
    }
    if let Some(acoustics_channels) = &channels.acoustics {
        acoustics_channels.insert_samples(&mut row, acoustics_report)?;
    }
    if let Some(countdown_channels) = &channels.countdown {
        countdown_channels.insert_samples(&mut row, countdown_report)?;
    }
    insert_fc_reference_channels(&mut row, &channels.fc_reference, fc_bridge)?;

    // Atmosphere sample at the post-step state. Match the runtime
    // environment's frame-aware altitude conversion so ECI launches
    // do not report sea-level telemetry density at orbital radius.
    let mut atmosphere_sample = None;
    if let Some(atmosphere) = breakdown_atmosphere {
        let altitude_m = atmosphere_altitude_m_with_surface_radius(
            state.position.vector,
            geocentric_surface_radius_m,
        );
        let sample = atmosphere.sample(altitude_m, state.time)?;
        atmosphere_sample = Some(sample);
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

    if let Some(contact_channels) = &channels.contact {
        let evaluator = contact_evaluator.ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "[contact] telemetry requested without a contact evaluator".to_owned(),
        })?;
        let diagnostics = evaluator.diagnostics_from_state_vectors(
            state.position.vector,
            state.velocity.vector,
            state.time.as_seconds(),
        )?;
        contact_channels.insert(&mut row, diagnostics)?;
        if let Some(accumulator) = contact_accumulator {
            accumulator.record(state.time.as_seconds(), diagnostics);
        }
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
        if let Some(channel) = &aerothermal_channels.electron_number_density {
            row.insert(channel, sample.electron_number_density_m3)?;
        }
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
    if let Some(plume_channels) = &channels.plume {
        let plume_state = if let Some(evaluator) = plume_evaluator {
            evaluator.evaluate(state, atmosphere_sample)?
        } else {
            None
        };
        plume_channels.insert(&mut row, plume_state)?;
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
    for recovery in channels {
        let state = snapshot
            .get(&recovery.id)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "recovery telemetry snapshot missing declared recovery id {}",
                    recovery.id.value()
                ),
            })?;
        row.insert(&recovery.deployed, state.deployed)?;
        row.insert(&recovery.phase_index, i64::from(state.phase_index))?;
        row.insert(&recovery.drag_area, state.drag_area_m2)?;
        if let Some(inflation) = &recovery.inflation {
            row.insert(&inflation.progress, state.inflation_progress)?;
            row.insert(&inflation.complete, state.inflation_complete)?;
            row.insert(
                &inflation.dynamic_pressure,
                state.opening_dynamic_pressure_pa,
            )?;
            row.insert(&inflation.opening_load, state.opening_load_n)?;
            row.insert(&inflation.opening_peak_load, state.opening_peak_load_n)?;
            if let Some(apparent_mass) = &inflation.apparent_mass {
                row.insert(&apparent_mass.apparent_mass, state.apparent_mass_kg)?;
                row.insert(
                    &apparent_mass.apparent_mass_rate,
                    state.apparent_mass_rate_kg_s,
                )?;
            }
            if let Some(pflanz) = &inflation.pflanz {
                row.insert(
                    &pflanz.ballistic_parameter,
                    state.opening_ballistic_parameter_a,
                )?;
                row.insert(
                    &pflanz.reduction_factor,
                    state.opening_pflanz_reduction_factor_x1,
                )?;
                row.insert(
                    &pflanz.finite_mass_peak_load,
                    state.opening_finite_mass_peak_load_n,
                )?;
            }
        }
        if let Some(sequence) = &recovery.sequence {
            row.insert(&sequence.delta_v, state.snatch_delta_v_m_s)?;
            row.insert(&sequence.stiffness, state.snatch_stiffness_n_m)?;
            row.insert(&sequence.effective_mass, state.snatch_effective_mass_kg)?;
            row.insert(&sequence.pack_drag_load, state.snatch_pack_drag_load_n)?;
            row.insert(&sequence.peak_load, state.snatch_peak_load_n)?;
            if let Some(mortar) = &sequence.mortar {
                row.insert(&mortar.impulse_body_x, state.mortar_impulse_body_n_s.x)?;
                row.insert(&mortar.impulse_body_y, state.mortar_impulse_body_n_s.y)?;
                row.insert(&mortar.impulse_body_z, state.mortar_impulse_body_n_s.z)?;
                row.insert(&mortar.force_body_x, state.mortar_force_body_n.x)?;
                row.insert(&mortar.force_body_y, state.mortar_force_body_n.y)?;
                row.insert(&mortar.force_body_z, state.mortar_force_body_n.z)?;
            }
        }
        if let Some(riser) = &recovery.riser {
            row.insert(&riser.tension_sum, state.riser_tension_sum_n)?;
            row.insert(&riser.tension_max, state.riser_tension_max_n)?;
            row.insert(&riser.force_body_x, state.riser_force_body_n.x)?;
            row.insert(&riser.force_body_y, state.riser_force_body_n.y)?;
            row.insert(&riser.force_body_z, state.riser_force_body_n.z)?;
            row.insert(&riser.moment_body_x, state.riser_moment_body_n_m.x)?;
            row.insert(&riser.moment_body_y, state.riser_moment_body_n_m.y)?;
            row.insert(&riser.moment_body_z, state.riser_moment_body_n_m.z)?;
            row.insert(&riser.elastic_energy, state.riser_elastic_energy_j)?;
        }
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use std::path::{Path, PathBuf};

    use approx::{assert_abs_diff_eq, assert_relative_eq};
    use openbmp_physics::{AtmosphereModel, UsStandard1976};
    use openbmp_telemetry::TelemetryValue;
    use openbmp_thermochem::ThermochemDeck;
    use serde::Deserialize;

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

    const TESSERAL_POINT_MASS_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "tesseral-point-mass-test"
description = "Synthetic point-mass run with frame-coupled degree-2 tesseral gravity."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 1.0
dt_s = 1.0
seed = 13

[vehicle]
kind = "point_mass"
initial_position_eci_m = [6900000.0, 400000.0, 300000.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "tesseral-point-mass-test"

[[vehicle.assembly.bodies]]
id = "mass"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "wgs84-uniform-rotation"
gravity = "tesseral"
mu_m3_s2 = 3.986004418e14
r_e_m = "@WGS84_A_M@"
tesseral_degree = 2
tesseral_order = 2
tesseral_c20 = "@NEG_WGS84_J2@"
tesseral_c21 = 2.0e-7
tesseral_s21 = -3.0e-7
tesseral_c22 = 1.0e-7
tesseral_s22 = -2.0e-7
tesseral_tide_system = "tide_free"
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/tesseral-point-mass-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const EGM2008_PINES_POINT_MASS_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "egm2008-pines-point-mass-test"
description = "Synthetic point-mass run with file-backed EGM2008 finite-difference Pines gravity."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 1.0
dt_s = 1.0
seed = 14

[vehicle]
kind = "point_mass"
initial_position_eci_m = [6900000.0, 400000.0, 300000.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "egm2008-pines-point-mass-test"

[[vehicle.assembly.bodies]]
id = "mass"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "wgs84-uniform-rotation"
gravity = "egm2008"
egm2008_coefficients_file = "data/gravity/egm2008-degree10-normalized-icgem-v1.gfc"
egm2008_degree = 10
egm2008_order = 10
egm2008_finite_difference_step_m = 10.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/egm2008-pines-point-mass-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const COMM_OFF_DEFAULT_POINT_MASS_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "comm-off-default-point-mass-test"
description = "Synthetic point-mass run used to prove the no-comm default path is byte-neutral."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.2
dt_s = 0.1
seed = 17

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 100.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "comm-off-default-point-mass-test"

[[vehicle.assembly.bodies]]
id = "stage"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 10.0
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
output.csv = "out/comm-off-default-point-mass-test.csv"

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

    fn add_iop_source_engine(toml: &str) -> String {
        toml.replace("models = [\"gravity\"]", "models = [\"gravity\", \"thrust\"]").replace(
            "[environment]",
            r#"[[vehicle.assembly.engines]]
id = "main"
kind = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, 0.0]
limits = { max_thrust_n = 600000.0, isp_s = 250.0, ignition_transient_s = 0.25, shutdown_transient_s = 0.1, max_gimbal_rad = 0.0 }

[environment]"#,
        )
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

    fn inflating_parachute_scenario() -> Scenario {
        Scenario::from_toml_str(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-inflating-parachute-telemetry"
description = "Synthetic point-mass recovery inflation telemetry test."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 1.0
dt_s = 0.25
seed = 16

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 1000.0]
initial_velocity_eci_m_s = [0.0, 0.0, -50.0]

[vehicle.assembly]
id = "point-mass-inflating-parachute-telemetry"

[[vehicle.assembly.bodies]]
id = "main"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 0.5 }
dry_mass_kg = 10.0

[[vehicle.assembly.recovery]]
id = "main_chute"
kind = { kind = "parachute_drag", c_d = 1.5, area_inflated_m2 = 8.0 }
inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = "polynomial", exponent_j = 1.0 }

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "us_standard_1976"
wind = "none"

[atmosphere]
kind = "us_standard_1976"

[telemetry]
output.csv = "out/point-mass-inflating-parachute-telemetry.csv"

[validation]
require_finite_state = true
require_monotonic_time = true

[mission]
initial_phase = "descent"

[[mission.phases]]
id = "descent"
label = "descent"

[[scenario_script.events]]
id = "deploy_main_chute"
trigger = { kind = "at_time", time_s = 0.25 }
action = { kind = "deploy_recovery", id = "main_chute", command = "deploy" }

[scenario_script]
"#,
        )
        .expect("inflating parachute scenario must parse")
    }

    fn pflanz_integrated_peak_scenario(expected_reduction_factor: f64) -> Scenario {
        pflanz_integrated_peak_scenario_from_values(
            "single",
            10.0,
            50.0,
            1.0,
            0.002,
            1.5,
            8.0,
            expected_reduction_factor,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn pflanz_integrated_peak_scenario_from_values(
        case: &str,
        dry_mass_kg: f64,
        initial_speed_m_s: f64,
        fill_time_s: f64,
        dt_s: f64,
        c_d: f64,
        area_m2: f64,
        expected_reduction_factor: f64,
    ) -> Scenario {
        let toml = format!(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-pflanz-integrated-peak-{case}"
description = "Synthetic point-mass recovery inflation integrated peak-load validation."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = {stop_s:.17e}
dt_s = {dt_s:.17e}
seed = 20

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 1000.0]
initial_velocity_eci_m_s = [{initial_speed_m_s:.17e}, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-pflanz-integrated-peak-{case}"

[[vehicle.assembly.bodies]]
id = "main"
geometry = {{ kind = "reference", length_m = 1.0, area_m2 = 0.5 }}
dry_mass_kg = {dry_mass_kg:.17e}

[[vehicle.assembly.recovery]]
id = "main_chute"
kind = {{ kind = "parachute_drag", c_d = {c_d:.17e}, area_inflated_m2 = {area_m2:.17e} }}
inflation = {{ fill_time_s = {fill_time_s:.17e}, opening_force_coeff_c_x = 1.0, growth = "polynomial", exponent_j = 1.0, pflanz = {{ ballistic_parameter_a = [0.01, 100.0], exponent_j = [1.0], reduction_factors = [{expected_reduction_factor:.17e}, {expected_reduction_factor:.17e}] }} }}

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "us_standard_1976"
wind = "none"

[atmosphere]
kind = "us_standard_1976"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/point-mass-pflanz-integrated-peak-{case}.csv"

[validation]
require_finite_state = true
require_monotonic_time = true

[mission]
initial_phase = "descent"

[[mission.phases]]
id = "descent"
label = "descent"

[[scenario_script.events]]
id = "deploy_main_chute"
trigger = {{ kind = "at_time", time_s = {dt_s:.17e} }}
action = {{ kind = "deploy_recovery", id = "main_chute", command = "deploy" }}

[scenario_script]
"#,
            stop_s = fill_time_s + 0.2,
        );
        Scenario::from_toml_str(&toml).expect("Pflanz integrated peak scenario must parse")
    }

    #[derive(Copy, Clone, Debug, PartialEq)]
    struct IntegratedOpeningLoadResult {
        peak_ratio: f64,
        final_speed_m_s: f64,
    }

    fn integrated_opening_load_fixed_step(
        density_kg_m3: f64,
        mass_kg: f64,
        full_c_d_s_m2: f64,
        initial_speed_m_s: f64,
        fill_time_s: f64,
        dt_s: f64,
    ) -> IntegratedOpeningLoadResult {
        fn derivative(
            density_kg_m3: f64,
            mass_kg: f64,
            full_c_d_s_m2: f64,
            fraction: f64,
            speed_m_s: f64,
        ) -> f64 {
            -0.5 * density_kg_m3 * full_c_d_s_m2 * fraction * speed_m_s * speed_m_s / mass_kg
        }

        let mut speed_m_s = initial_speed_m_s;
        let mut elapsed_s = 0.0_f64;
        let initial_full_load_n = 0.5 * density_kg_m3 * initial_speed_m_s.powi(2) * full_c_d_s_m2;
        let mut peak_load_n = 0.0_f64;
        while elapsed_s < fill_time_s {
            elapsed_s = (elapsed_s + dt_s).min(fill_time_s);
            let fraction = (elapsed_s / fill_time_s).clamp(0.0, 1.0);
            let load_n = 0.5 * density_kg_m3 * speed_m_s.powi(2) * full_c_d_s_m2 * fraction;
            peak_load_n = peak_load_n.max(load_n);
            let k1 = derivative(density_kg_m3, mass_kg, full_c_d_s_m2, fraction, speed_m_s);
            let k2 = derivative(
                density_kg_m3,
                mass_kg,
                full_c_d_s_m2,
                fraction,
                speed_m_s + 0.5 * dt_s * k1,
            );
            let k3 = derivative(
                density_kg_m3,
                mass_kg,
                full_c_d_s_m2,
                fraction,
                speed_m_s + 0.5 * dt_s * k2,
            );
            let k4 = derivative(
                density_kg_m3,
                mass_kg,
                full_c_d_s_m2,
                fraction,
                speed_m_s + dt_s * k3,
            );
            speed_m_s += dt_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        }
        IntegratedOpeningLoadResult {
            peak_ratio: peak_load_n / initial_full_load_n,
            final_speed_m_s: speed_m_s,
        }
    }

    fn integrated_opening_load_ratio_fixed_step(
        density_kg_m3: f64,
        mass_kg: f64,
        full_c_d_s_m2: f64,
        initial_speed_m_s: f64,
        fill_time_s: f64,
        dt_s: f64,
    ) -> f64 {
        integrated_opening_load_fixed_step(
            density_kg_m3,
            mass_kg,
            full_c_d_s_m2,
            initial_speed_m_s,
            fill_time_s,
            dt_s,
        )
        .peak_ratio
    }

    #[derive(Debug, Deserialize)]
    struct RecoveryPflanzIntegratedSweepFixture {
        format: String,
        cases: Vec<RecoveryPflanzIntegratedSweepCase>,
        tolerances: RecoveryPflanzIntegratedSweepTolerances,
    }

    #[derive(Debug, Deserialize)]
    struct RecoveryPflanzIntegratedSweepCase {
        case: String,
        target_ballistic_parameter_a: f64,
        initial_speed_m_s: f64,
        fill_time_s: f64,
        dt_s: f64,
        c_d: f64,
        area_m2: f64,
    }

    #[derive(Debug, Deserialize)]
    struct RecoveryPflanzIntegratedSweepTolerances {
        peak_ratio_abs: f64,
        final_speed_abs_m_s: f64,
        ballistic_parameter_rel: f64,
        finite_peak_ratio_abs: f64,
    }

    fn load_recovery_pflanz_integrated_sweep_fixture() -> RecoveryPflanzIntegratedSweepFixture {
        toml::from_str(include_str!(
            "../tests/expected/recovery-pflanz-integrated-sweep-v1.toml"
        ))
        .expect("recovery Pflanz integrated sweep fixture parses")
    }

    fn dry_mass_for_target_pflanz_a(
        target_ballistic_parameter_a: f64,
        density_kg_m3: f64,
        full_c_d_s_m2: f64,
        initial_speed_m_s: f64,
        fill_time_s: f64,
    ) -> f64 {
        0.5 * target_ballistic_parameter_a
            * density_kg_m3
            * full_c_d_s_m2
            * initial_speed_m_s
            * fill_time_s
    }

    fn recovery_wind_drift_scenario(with_constant_wind: bool) -> Scenario {
        let (environment_wind, wind_block) = if with_constant_wind {
            (
                "constant",
                r#"
[wind]
kind = "constant"
wind_ned_m_s = [12.0, 0.0, 0.0]
"#,
            )
        } else {
            ("none", "")
        };
        let toml = format!(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-recovery-wind-drift"
description = "Synthetic point-mass recovery drift validation with declared constant wind."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 5.0
dt_s = 0.1
seed = 18

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 1000.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-recovery-wind-drift"

[[vehicle.assembly.bodies]]
id = "main"
geometry = {{ kind = "reference", length_m = 1.0, area_m2 = 0.5 }}
dry_mass_kg = 10.0

[[vehicle.assembly.recovery]]
id = "main_chute"
kind = {{ kind = "parachute_drag", c_d = 1.0, area_inflated_m2 = 1.0 }}

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "us_standard_1976"
wind = "{environment_wind}"

[atmosphere]
kind = "us_standard_1976"
{wind_block}
[telemetry]
output.csv = "out/point-mass-recovery-wind-drift.csv"

[validation]
require_finite_state = true
require_monotonic_time = true

[mission]
initial_phase = "drift"

[[mission.phases]]
id = "drift"
label = "drift"

[[scenario_script.events]]
id = "deploy_main_chute"
trigger = {{ kind = "at_time", time_s = 0.05 }}
action = {{ kind = "deploy_recovery", id = "main_chute", command = "deploy" }}

[scenario_script]
"#
        );
        Scenario::from_toml_str(&toml).expect("recovery wind drift scenario must parse")
    }

    fn recovery_splash_drift_scenario(with_constant_wind: bool) -> Scenario {
        let (environment_wind, wind_block) = if with_constant_wind {
            (
                "constant",
                r#"
[wind]
kind = "constant"
wind_ned_m_s = [6.0, 0.0, 0.0]
"#,
            )
        } else {
            ("none", "")
        };
        let toml = format!(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-recovery-splash-drift"
description = "Synthetic point-mass recovery splash-point drift validation."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 20.0
dt_s = 0.05
seed = 19

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 60.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-recovery-splash-drift"

[[vehicle.assembly.bodies]]
id = "main"
geometry = {{ kind = "reference", length_m = 1.0, area_m2 = 0.5 }}
dry_mass_kg = 10.0

[[vehicle.assembly.recovery]]
id = "main_chute"
kind = {{ kind = "parachute_drag", c_d = 1.0, area_inflated_m2 = 1.0 }}

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "us_standard_1976"
wind = "{environment_wind}"

[atmosphere]
kind = "us_standard_1976"
{wind_block}
[telemetry]
output.csv = "out/point-mass-recovery-splash-drift.csv"

[validation]
require_finite_state = true
require_monotonic_time = true

[mission]
initial_phase = "descent"

[[mission.phases]]
id = "descent"
label = "descent"

[[scenario_script.events]]
id = "deploy_main_chute"
trigger = {{ kind = "at_time", time_s = 0.025 }}
action = {{ kind = "deploy_recovery", id = "main_chute", command = "deploy" }}

[scenario_script]
"#
        );
        Scenario::from_toml_str(&toml).expect("recovery splash drift scenario must parse")
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

    fn has_channel(outcome: &RunOutcome, name: &str) -> bool {
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .any(|channel| channel.name == name)
    }

    fn first_row_f64(outcome: &RunOutcome, name: &str) -> f64 {
        match first_row_value(outcome, name) {
            TelemetryValue::Float64(value) => *value,
            other => panic!("unexpected first-row value for {name}: {other:?}"),
        }
    }

    fn first_row_bool(outcome: &RunOutcome, name: &str) -> bool {
        match first_row_value(outcome, name) {
            TelemetryValue::Bool(value) => *value,
            other => panic!("unexpected first-row value for {name}: {other:?}"),
        }
    }

    fn first_row_i64(outcome: &RunOutcome, name: &str) -> i64 {
        match first_row_value(outcome, name) {
            TelemetryValue::Int64(value) => *value,
            other => panic!("unexpected first-row value for {name}: {other:?}"),
        }
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

    fn row_times_s(outcome: &RunOutcome) -> Vec<f64> {
        outcome
            .table
            .rows()
            .iter()
            .map(|row| row.time.as_seconds())
            .collect()
    }

    fn point_mass_afts_outside_range_scenario() -> Scenario {
        let radius_m = openbmp_physics::WGS84_A_M + 1_000.0;
        let mu_m3_s2 = openbmp_physics::WGS84_MU_M3_S2;
        let toml = format!(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-afts-outside-range-test"
description = "Synthetic point-mass AFTS range-containment regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 12

[vehicle]
kind = "point_mass"
initial_position_eci_m = [{radius_m}, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-afts-outside-range-test"

[[vehicle.assembly.bodies]]
id = "body"
geometry = {{ kind = "reference", length_m = 1.0, area_m2 = 1.0 }}
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "point_mass"
mu_m3_s2 = {mu_m3_s2}
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[afts]
enabled = true

[[afts.keep_inside]]
id = "north-box"
evidence = "tests/fixtures/afts/north-box.md#synthetic"
vertices = [
  {{ latitude_deg = 10.0, longitude_deg = 10.0 }},
  {{ latitude_deg = 10.0, longitude_deg = 20.0 }},
  {{ latitude_deg = 20.0, longitude_deg = 20.0 }},
  {{ latitude_deg = 20.0, longitude_deg = 10.0 }},
]

[telemetry]
output.csv = "out/point-mass-afts-outside-range-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#
        );
        Scenario::from_toml_str(&toml).expect("AFTS point-mass scenario must parse")
    }

    fn point_mass_afts_keep_out_scenario() -> Scenario {
        let radius_m = openbmp_physics::WGS84_A_M + 1_000.0;
        let mu_m3_s2 = openbmp_physics::WGS84_MU_M3_S2;
        let toml = format!(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-afts-keep-out-test"
description = "Synthetic point-mass AFTS keep-out regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 13

[vehicle]
kind = "point_mass"
initial_position_eci_m = [{radius_m}, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-afts-keep-out-test"

[[vehicle.assembly.bodies]]
id = "body"
geometry = {{ kind = "reference", length_m = 1.0, area_m2 = 1.0 }}
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "point_mass"
mu_m3_s2 = {mu_m3_s2}
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[afts]
enabled = true

[[afts.keep_out]]
id = "hazard-zone"
evidence = "tests/fixtures/afts/hazard-zone.md#synthetic"
vertices = [
  {{ latitude_deg = -1.0, longitude_deg = -1.0 }},
  {{ latitude_deg = -1.0, longitude_deg = 1.0 }},
  {{ latitude_deg = 1.0, longitude_deg = 1.0 }},
  {{ latitude_deg = 1.0, longitude_deg = -1.0 }},
]

[telemetry]
output.csv = "out/point-mass-afts-keep-out-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#
        );
        Scenario::from_toml_str(&toml).expect("AFTS keep-out point-mass scenario must parse")
    }

    fn point_mass_afts_corridor_scenario() -> Scenario {
        let radius_m = openbmp_physics::WGS84_A_M + 1_000.0;
        let mu_m3_s2 = openbmp_physics::WGS84_MU_M3_S2;
        let toml = format!(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-afts-corridor-test"
description = "Synthetic point-mass AFTS corridor regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 14

[vehicle]
kind = "point_mass"
initial_position_eci_m = [{radius_m}, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-afts-corridor-test"

[[vehicle.assembly.bodies]]
id = "body"
geometry = {{ kind = "reference", length_m = 1.0, area_m2 = 1.0 }}
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "point_mass"
mu_m3_s2 = {mu_m3_s2}
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[afts]
enabled = true

[[afts.corridor]]
id = "altitude-band"
evidence = "tests/fixtures/afts/altitude-band.md#synthetic"
metric = "altitude_m"
min_altitude_m = 0.0
max_altitude_m = 100.0

[telemetry]
output.csv = "out/point-mass-afts-corridor-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#
        );
        Scenario::from_toml_str(&toml).expect("AFTS corridor point-mass scenario must parse")
    }

    fn point_mass_afts_gate_scenario() -> Scenario {
        let radius_m = openbmp_physics::WGS84_A_M;
        let mu_m3_s2 = openbmp_physics::WGS84_MU_M3_S2;
        let toml = format!(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-afts-gate-test"
description = "Synthetic point-mass AFTS gate-crossing regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.2
dt_s = 0.1
seed = 15

[vehicle]
kind = "point_mass"
initial_position_eci_m = [{radius_m}, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 10.0, 0.0]

[vehicle.assembly]
id = "point-mass-afts-gate-test"

[[vehicle.assembly.bodies]]
id = "body"
geometry = {{ kind = "reference", length_m = 1.0, area_m2 = 1.0 }}
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "point_mass"
mu_m3_s2 = {mu_m3_s2}
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[afts]
enabled = true

[[afts.gate]]
id = "zero-longitude-gate"
evidence = "tests/fixtures/afts/zero-longitude-gate.md#synthetic"
start = {{ latitude_deg = -1.0, longitude_deg = 0.0 }}
end = {{ latitude_deg = 1.0, longitude_deg = 0.0 }}
direction = "any"

[telemetry]
output.csv = "out/point-mass-afts-gate-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#
        );
        Scenario::from_toml_str(&toml).expect("AFTS gate point-mass scenario must parse")
    }

    fn point_mass_afts_red_zone_scenario() -> Scenario {
        let radius_m = openbmp_physics::WGS84_A_M + 1_000.0;
        let mu_m3_s2 = openbmp_physics::WGS84_MU_M3_S2;
        let toml = format!(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-afts-red-zone-test"
description = "Synthetic point-mass AFTS red-zone regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 16

[vehicle]
kind = "point_mass"
initial_position_eci_m = [{radius_m}, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-afts-red-zone-test"

[[vehicle.assembly.bodies]]
id = "body"
geometry = {{ kind = "reference", length_m = 1.0, area_m2 = 1.0 }}
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "point_mass"
mu_m3_s2 = {mu_m3_s2}
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[afts]
enabled = true

[[afts.zone]]
id = "red-zone"
evidence = "tests/fixtures/afts/red-zone.md#synthetic"
color = "red"
vertices = [
  {{ latitude_deg = -1.0, longitude_deg = -1.0 }},
  {{ latitude_deg = -1.0, longitude_deg = 1.0 }},
  {{ latitude_deg = 1.0, longitude_deg = 1.0 }},
  {{ latitude_deg = 1.0, longitude_deg = -1.0 }},
]

[telemetry]
output.csv = "out/point-mass-afts-red-zone-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#
        );
        Scenario::from_toml_str(&toml).expect("AFTS red-zone point-mass scenario must parse")
    }

    fn point_mass_afts_ellipsoid_corridor_scenario() -> Scenario {
        let polar_radius_m = openbmp_physics::WGS84_A_M * (1.0 - openbmp_physics::WGS84_FLATTENING);
        let initial_z_m = polar_radius_m + 1_000.0;
        let mu_m3_s2 = openbmp_physics::WGS84_MU_M3_S2;
        let toml = format!(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-afts-ellipsoid-corridor-test"
description = "Synthetic point-mass AFTS ellipsoid corridor regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 17

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, {initial_z_m}]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-afts-ellipsoid-corridor-test"

[[vehicle.assembly.bodies]]
id = "body"
geometry = {{ kind = "reference", length_m = 1.0, area_m2 = 1.0 }}
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "point_mass"
mu_m3_s2 = {mu_m3_s2}
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[afts]
enabled = true

[afts.propagator]
surface = "wgs84_ellipsoid"
earth_rotation = "disabled"

[[afts.corridor]]
id = "ellipsoid-altitude-band"
evidence = "tests/fixtures/afts/ellipsoid-altitude-band.md#synthetic"
metric = "altitude_m"
min_altitude_m = 500.0
max_altitude_m = 1500.0

[telemetry]
output.csv = "out/point-mass-afts-ellipsoid-corridor-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#
        );
        Scenario::from_toml_str(&toml).expect("AFTS ellipsoid corridor scenario must parse")
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
    fn point_mass_upstream_uq_gathers_aero_and_aerothermal_bands() {
        let toml = POINT_MASS_ENTRY_SCENARIO
            .replace(
                "[aerothermal]\n",
                "[aero]\n\
                 \n\
                 [aero.uq]\n\
                 deck_id = \"synthetic_finned_cylinder.v1\"\n\
                 coefficient_id = \"cd\"\n\
                 coefficient_lower = 0.18\n\
                 coefficient_nominal = 0.20\n\
                 coefficient_upper = 0.23\n\
                 credibility_level = 2\n\
                 evidence = \"data/aero/synthetic-finned-cylinder.toml#coefficient=cd\"\n\
                 \n\
                 [aerothermal]\n",
            )
            .replace(
                "[forces]\n",
                "[aerothermal.uq]\n\
                 deck_id = \"sutton_graves_earth.v1\"\n\
                 q_conv_lower_w_m2 = 900000.0\n\
                 q_conv_nominal_w_m2 = 1000000.0\n\
                 q_conv_upper_w_m2 = 1200000.0\n\
                 credibility_level = 2\n\
                 evidence = \"docs/parity/04-aerothermal-realgas-and-tps.md#sutton-graves\"\n\
                 \n\
                 [forces]\n",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("point-mass UQ scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("point-mass UQ run succeeds");

        let upstream_sources = &outcome.upstream_uq.sources;
        assert_eq!(upstream_sources.len(), 2);
        assert_eq!(
            upstream_sources[0].source_id,
            "03.aero.synthetic_finned_cylinder.v1.cd"
        );
        assert!((upstream_sources[0].one_sigma - 0.03).abs() < 1.0e-15);
        assert_eq!(
            upstream_sources[1].source_id,
            "04.aerothermal.sutton_graves_earth.v1.q_conv_w_m2"
        );
        assert!((upstream_sources[1].one_sigma - 200000.0).abs() < 1.0e-9);
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
    fn point_mass_afts_report_latches_without_changing_kernel_stop_reason() {
        let scenario = point_mass_afts_outside_range_scenario();
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("AFTS run succeeds");

        assert!(
            matches!(outcome.stop_reason, StopReason::EndTime { .. }),
            "AFTS observer must not replace the kernel stop reason: {:?}",
            outcome.stop_reason
        );
        let report = outcome.afts.as_ref().expect("AFTS report is present");
        assert_eq!(report.samples as usize, outcome.table.rows().len());
        assert!(report.terminate);
        assert_eq!(report.rule_id.as_deref(), Some("north-box"));
        assert_eq!(
            report.rule_evidence.as_deref(),
            Some("tests/fixtures/afts/north-box.md#synthetic")
        );
        assert_eq!(report.first_trigger_step, Some(0));
        assert_eq!(report.first_trigger_time_s, Some(0.0));
        assert!(
            report
                .impact_latitude_rad
                .is_some_and(|latitude| latitude.abs() < 1.0e-12)
        );
        assert!(
            report
                .impact_longitude_rad
                .is_some_and(|longitude| longitude.abs() < 1.0e-12)
        );
    }

    #[test]
    fn point_mass_afts_keep_out_report_latches_inside_forbidden_polygon() {
        let scenario = point_mass_afts_keep_out_scenario();
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("AFTS keep-out run succeeds");

        assert!(
            matches!(outcome.stop_reason, StopReason::EndTime { .. }),
            "AFTS observer must not replace the kernel stop reason: {:?}",
            outcome.stop_reason
        );
        let report = outcome.afts.as_ref().expect("AFTS report is present");
        assert!(report.terminate);
        assert_eq!(report.rule_id.as_deref(), Some("hazard-zone"));
        assert_eq!(report.rule_table.len(), 1);
        assert_eq!(report.rule_table[0].kind, "keep_out");
        assert_eq!(
            report.rule_evidence.as_deref(),
            Some("tests/fixtures/afts/hazard-zone.md#synthetic")
        );
    }

    #[test]
    fn point_mass_afts_corridor_report_latches_without_iip_rule() {
        let scenario = point_mass_afts_corridor_scenario();
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("AFTS corridor run succeeds");

        assert!(
            matches!(outcome.stop_reason, StopReason::EndTime { .. }),
            "AFTS observer must not replace the kernel stop reason: {:?}",
            outcome.stop_reason
        );
        let report = outcome.afts.as_ref().expect("AFTS report is present");
        assert!(report.terminate);
        assert_eq!(report.rule_id.as_deref(), Some("altitude-band"));
        assert_eq!(report.rule_table.len(), 1);
        assert_eq!(report.rule_table[0].kind, "corridor");
        assert_eq!(report.rule_table[0].metric.as_deref(), Some("altitude_m"));
        assert_eq!(report.rule_table[0].max_value, Some(100.0));
        assert_eq!(
            report.rule_evidence.as_deref(),
            Some("tests/fixtures/afts/altitude-band.md#synthetic")
        );
        assert_eq!(report.first_trigger_step, Some(0));
        assert_eq!(report.first_trigger_time_s, Some(0.0));
        assert!(report.impact_latitude_rad.is_none());
        assert!(
            report
                .altitude_m
                .is_some_and(|altitude| (altitude - 1_000.0).abs() < 1.0e-9)
        );
    }

    #[test]
    fn point_mass_afts_gate_report_latches_on_iip_crossing() {
        let scenario = point_mass_afts_gate_scenario();
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("AFTS gate run succeeds");

        assert!(
            matches!(outcome.stop_reason, StopReason::EndTime { .. }),
            "AFTS observer must not replace the kernel stop reason: {:?}",
            outcome.stop_reason
        );
        let report = outcome.afts.as_ref().expect("AFTS report is present");
        assert!(report.terminate);
        assert_eq!(report.rule_id.as_deref(), Some("zero-longitude-gate"));
        assert_eq!(report.rule_table.len(), 1);
        assert_eq!(report.rule_table[0].kind, "gate");
        assert_eq!(report.rule_table[0].direction.as_deref(), Some("any"));
        assert_eq!(
            report.rule_evidence.as_deref(),
            Some("tests/fixtures/afts/zero-longitude-gate.md#synthetic")
        );
        assert_eq!(report.first_trigger_step, Some(1));
        assert_eq!(report.first_trigger_time_s, Some(0.1));
        assert!(
            report
                .impact_longitude_rad
                .is_some_and(|longitude| longitude > 0.0)
        );
    }

    #[test]
    fn point_mass_afts_zone_report_latches_inside_red_zone() {
        let scenario = point_mass_afts_red_zone_scenario();
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("AFTS red-zone run succeeds");

        assert!(
            matches!(outcome.stop_reason, StopReason::EndTime { .. }),
            "AFTS observer must not replace the kernel stop reason: {:?}",
            outcome.stop_reason
        );
        let report = outcome.afts.as_ref().expect("AFTS report is present");
        assert!(report.terminate);
        assert_eq!(report.rule_id.as_deref(), Some("red-zone"));
        assert_eq!(report.rule_table.len(), 1);
        assert_eq!(report.rule_table[0].kind, "zone");
        assert_eq!(report.rule_table[0].zone_color.as_deref(), Some("red"));
        assert_eq!(
            report.rule_evidence.as_deref(),
            Some("tests/fixtures/afts/red-zone.md#synthetic")
        );
        assert_eq!(report.first_trigger_step, Some(0));
        assert_eq!(report.first_trigger_time_s, Some(0.0));
    }

    #[test]
    fn point_mass_afts_ellipsoid_corridor_uses_surface_altitude() {
        let scenario = point_mass_afts_ellipsoid_corridor_scenario();
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome =
            run(&scenario, &resolved_files, None).expect("AFTS ellipsoid corridor run succeeds");

        assert!(matches!(outcome.stop_reason, StopReason::EndTime { .. }));
        let report = outcome.afts.as_ref().expect("AFTS report is present");
        assert!(!report.terminate);
        assert_eq!(report.rule_id, None);
        assert_eq!(report.rule_table.len(), 1);
        assert_eq!(report.rule_table[0].kind, "corridor");
        assert_eq!(report.rule_table[0].id, "ellipsoid-altitude-band");
        assert!(
            matches!(report.altitude_m, Some(altitude_m) if (999.0..=1000.0).contains(&altitude_m)),
            "ellipsoid altitude should stay near the declared 1000 m polar height: {report:?}"
        );
    }

    #[test]
    fn point_mass_no_comm_block_has_no_comm_channels_and_stable_hash() {
        let baseline =
            Scenario::from_toml_str(COMM_OFF_DEFAULT_POINT_MASS_SCENARIO).expect("baseline parses");
        let repeat =
            Scenario::from_toml_str(COMM_OFF_DEFAULT_POINT_MASS_SCENARIO).expect("repeat parses");

        let baseline = crate::run(&baseline).expect("baseline run");
        let repeat = crate::run(&repeat).expect("repeat run");

        assert!(
            baseline.comm.is_none(),
            "baseline scenario should not emit a comm report"
        );
        assert!(
            baseline.acoustics.is_none(),
            "baseline scenario should not emit an acoustics report"
        );
        assert!(
            baseline
                .table
                .schema()
                .channels()
                .iter()
                .all(|channel| !channel.name.starts_with("comm.")),
            "scenario without [comm] must not emit comm telemetry channels"
        );
        assert!(
            baseline
                .table
                .schema()
                .channels()
                .iter()
                .all(|channel| !channel.name.starts_with("acoustics.")),
            "scenario without [acoustics] must not emit acoustics telemetry channels"
        );
        assert_eq!(
            crate::determinism::telemetry_sha256_hex(&baseline.table)
                .expect("baseline telemetry hash"),
            crate::determinism::telemetry_sha256_hex(&repeat.table).expect("repeat telemetry hash"),
            "a scenario without [comm] must preserve the canonical telemetry hash across runs"
        );
        assert_eq!(
            baseline.table, repeat.table,
            "a scenario without [comm] must remain byte-identical across runs"
        );
    }

    #[test]
    fn point_mass_acoustics_ascent_deck_report_carries_provenance() {
        let deck_bytes = include_bytes!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/ascent-acoustic-env-v1.toml",
            deck_bytes.to_vec(),
        )
        .sha256_hex;
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            &format!(
                r#"[acoustics.ascent]
station_id = "station-a"
deck_file = "data/acoustics/ascent-acoustic-env-v1.toml"
deck_file_sha256 = "{deck_sha256}"
class_id = "attached_turbulent"
dynamic_pressure_pa = 50000.0
mach = 0.8

[telemetry]"#,
            ),
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("ascent acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("ascent acoustics scenario runs");
        let acoustics = outcome.acoustics.as_ref().expect("acoustics report");
        assert!(acoustics.dsm.is_none());
        assert!(acoustics.iop.is_none());
        let ascent = acoustics.ascent.as_ref().expect("ascent report emitted");

        assert_eq!(ascent.station_id, "station-a");
        assert_eq!(ascent.class_id, "attached_turbulent");
        assert_eq!(ascent.dynamic_pressure_pa.to_bits(), 50_000.0_f64.to_bits());
        assert_eq!(ascent.mach.to_bits(), 0.8_f64.to_bits());
        assert_eq!(ascent.sample_count, 1);
        assert_eq!(ascent.peak_sample_index, 0);
        assert_eq!(ascent.peak_sample_id, None);
        assert_eq!(
            ascent.peak_sample_oaspl_db.to_bits(),
            ascent.oaspl_db.to_bits()
        );
        assert_eq!(ascent.deck_file_sha256_hex, deck_sha256);
        assert_eq!(ascent.deck_dataset_id, "openbmp.acoustics.ascent_env.v1");
        assert_eq!(ascent.deck_source_class, "synthetic-openbmp");
        assert_eq!(ascent.uncertainty_db.to_bits(), 3.0_f64.to_bits());
        assert!((ascent.oaspl_db - 132.0).abs() <= 1.0e-12);
        assert_eq!(ascent.p95_50_oaspl_db, None);
        assert_eq!(ascent.p95_50_tolerance_factor, None);
        assert!(ascent.p95_50_environment_csv.is_empty());
        assert_eq!(ascent.p95_50_wilks_required_sample_count, None);
        assert_eq!(ascent.p95_50_wilks_achieved_confidence, None);
        assert_eq!(ascent.p95_50_wilks_sample_count_sufficient, None);
        assert!(ascent.p95_50_wilks_sizing_csv.is_empty());
        assert_eq!(
            std::str::from_utf8(&ascent.spectrum_csv)
                .expect("spectrum CSV is UTF-8")
                .lines()
                .count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        assert_eq!(
            std::str::from_utf8(&ascent.sample_trace_csv)
                .expect("sample trace CSV is UTF-8")
                .lines()
                .count(),
            2
        );
        assert_eq!(
            first_row_f64(&outcome, "acoustics.ascent.oaspl_db").to_bits(),
            ascent.oaspl_db.to_bits()
        );
        assert_eq!(
            first_row_f64(&outcome, "acoustics.ascent.uncertainty_db").to_bits(),
            ascent.uncertainty_db.to_bits()
        );
    }

    #[test]
    fn point_mass_acoustics_ascent_mach_gate_selects_sample_classes() {
        let deck_bytes = include_bytes!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/ascent-acoustic-env-v1.toml",
            deck_bytes.to_vec(),
        )
        .sha256_hex;
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            &format!(
                r#"[acoustics.ascent]
station_id = "station-a"
deck_file = "data/acoustics/ascent-acoustic-env-v1.toml"
deck_file_sha256 = "{deck_sha256}"
class_selection = "mach_gate"

[[acoustics.ascent.samples]]
sample_id = "attached"
dynamic_pressure_pa = 50000.0
mach = 0.5

[[acoustics.ascent.samples]]
sample_id = "transonic"
dynamic_pressure_pa = 65000.0
mach = 1.1

[telemetry]"#,
            ),
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("mach-gated ascent acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("mach-gated ascent acoustics scenario runs");
        let ascent = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.ascent.as_ref())
            .expect("ascent report emitted");

        assert_eq!(ascent.sample_count, 2);
        assert_eq!(ascent.peak_sample_index, 1);
        assert_eq!(ascent.peak_sample_id.as_deref(), Some("transonic"));
        assert_eq!(ascent.class_id, "transonic_buffet");
        assert_eq!(ascent.dynamic_pressure_pa.to_bits(), 65_000.0_f64.to_bits());
        assert_eq!(ascent.mach.to_bits(), 1.1_f64.to_bits());
        assert!((ascent.peak_sample_oaspl_db - 138.0).abs() <= 1.0e-12);
        assert!((ascent.oaspl_db - 138.0).abs() <= 1.0e-12);
        let sample_trace =
            std::str::from_utf8(&ascent.sample_trace_csv).expect("sample trace CSV is UTF-8");
        assert!(sample_trace.contains("attached,attached_turbulent"));
        assert!(sample_trace.contains("transonic,transonic_buffet"));
    }

    #[test]
    fn point_mass_acoustics_ascent_sample_trace_envelopes_fairing_source() {
        let ascent_deck_bytes =
            include_bytes!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let ascent_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/ascent-acoustic-env-v1.toml",
            ascent_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let fairing_deck_bytes = include_bytes!("../../../data/acoustics/fairing-internal-v1.toml");
        let fairing_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/fairing-internal-v1.toml",
            fairing_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            &format!(
                r#"[acoustics.ascent]
station_id = "station-a"
deck_file = "data/acoustics/ascent-acoustic-env-v1.toml"
deck_file_sha256 = "{ascent_deck_sha256}"
class_id = "attached_turbulent"

[[acoustics.ascent.samples]]
sample_id = "max-q"
dynamic_pressure_pa = 50000.0
mach = 0.8

[[acoustics.ascent.samples]]
sample_id = "late-max-q"
dynamic_pressure_pa = 200000.0
mach = 0.8

[acoustics.fairing_internal]
station_id = "payload-a"
deck_file = "data/acoustics/fairing-internal-v1.toml"
deck_file_sha256 = "{fairing_deck_sha256}"
class_id = "synthetic_light_fairing"
fill_factor = 1.0

[telemetry]"#,
            ),
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("ascent trace acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("ascent trace acoustics scenario runs");
        let acoustics = outcome.acoustics.as_ref().expect("acoustics report");
        let ascent = acoustics.ascent.as_ref().expect("ascent report emitted");
        let fairing = acoustics
            .fairing_internal
            .as_ref()
            .expect("fairing report emitted");

        let expected_ascent_oaspl_db = 132.0 + 10.0 * 0.6 * 4.0_f64.log10();
        let expected_fairing_oaspl_db = expected_ascent_oaspl_db - 8.0 + 10.0 * 2.0_f64.log10();
        let expected_wilks_required = openbmp_mc::wilks_one_sided_n(
            openbmp_acoustics::stats::P95_50_COVERAGE_PROBABILITY,
            openbmp_acoustics::stats::P95_50_CONFIDENCE_PROBABILITY,
        )
        .expect("Wilks floor");
        let expected_wilks_achieved = openbmp_mc::wilks_one_sided_confidence(
            openbmp_acoustics::stats::P95_50_COVERAGE_PROBABILITY,
            2,
        )
        .expect("Wilks confidence");
        assert_eq!(ascent.sample_count, 2);
        assert_eq!(ascent.peak_sample_index, 1);
        assert_eq!(ascent.peak_sample_id.as_deref(), Some("late-max-q"));
        assert_eq!(ascent.class_id, "attached_turbulent");
        assert_eq!(
            ascent.dynamic_pressure_pa.to_bits(),
            200_000.0_f64.to_bits()
        );
        assert_eq!(ascent.mach.to_bits(), 0.8_f64.to_bits());
        assert!((ascent.peak_sample_oaspl_db - expected_ascent_oaspl_db).abs() <= 1.0e-12);
        assert!((ascent.oaspl_db - expected_ascent_oaspl_db).abs() <= 1.0e-12);
        let p95_50_oaspl_db = ascent.p95_50_oaspl_db.expect("P95/50 emitted");
        let p95_50_tolerance_factor = ascent
            .p95_50_tolerance_factor
            .expect("P95/50 factor emitted");
        assert!(
            p95_50_oaspl_db > ascent.oaspl_db,
            "P95/50 environment should exceed the two-sample envelope for this synthetic dispersion"
        );
        assert!(
            (p95_50_tolerance_factor
                - openbmp_physics::inverse_standard_normal_cdf(
                    openbmp_acoustics::stats::P95_50_COVERAGE_PROBABILITY,
                ))
            .abs()
                <= 1.0e-12
        );
        assert_eq!(
            std::str::from_utf8(&ascent.sample_trace_csv)
                .expect("sample trace CSV is UTF-8")
                .lines()
                .count(),
            3
        );
        assert_eq!(
            std::str::from_utf8(&ascent.p95_50_environment_csv)
                .expect("P95/50 CSV is UTF-8")
                .lines()
                .count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        assert_eq!(
            ascent.p95_50_wilks_required_sample_count,
            Some(expected_wilks_required)
        );
        assert_eq!(
            ascent
                .p95_50_wilks_achieved_confidence
                .expect("ascent Wilks confidence")
                .to_bits(),
            expected_wilks_achieved.to_bits()
        );
        assert_eq!(ascent.p95_50_wilks_sample_count_sufficient, Some(false));
        let ascent_wilks_csv = std::str::from_utf8(&ascent.p95_50_wilks_sizing_csv)
            .expect("ascent Wilks CSV is UTF-8");
        assert!(ascent_wilks_csv.contains("required_sample_count"));
        assert_eq!(ascent_wilks_csv.lines().count(), 2);
        assert!((fairing.oaspl_db - expected_fairing_oaspl_db).abs() <= 1.0e-12);
        let fairing_p95_50_oaspl_db = fairing.p95_50_oaspl_db.expect("fairing P95/50 emitted");
        assert!(
            fairing_p95_50_oaspl_db > fairing.oaspl_db,
            "fairing P95/50 environment should exceed the two-sample internal envelope"
        );
        assert_eq!(
            fairing
                .p95_50_tolerance_factor
                .expect("fairing P95/50 factor")
                .to_bits(),
            p95_50_tolerance_factor.to_bits()
        );
        assert_eq!(
            std::str::from_utf8(&fairing.p95_50_environment_csv)
                .expect("fairing P95/50 CSV is UTF-8")
                .lines()
                .count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        assert_eq!(
            fairing.p95_50_wilks_required_sample_count,
            Some(expected_wilks_required)
        );
        assert_eq!(
            fairing
                .p95_50_wilks_achieved_confidence
                .expect("fairing Wilks confidence")
                .to_bits(),
            expected_wilks_achieved.to_bits()
        );
        assert_eq!(fairing.p95_50_wilks_sample_count_sufficient, Some(false));
        let fairing_wilks_csv = std::str::from_utf8(&fairing.p95_50_wilks_sizing_csv)
            .expect("fairing Wilks CSV is UTF-8");
        assert!(fairing_wilks_csv.contains("sample_count_sufficient"));
        assert_eq!(fairing_wilks_csv.lines().count(), 2);
        assert_eq!(
            first_row_f64(&outcome, "acoustics.ascent.oaspl_db").to_bits(),
            ascent.oaspl_db.to_bits()
        );
        assert_eq!(
            first_row_f64(&outcome, "acoustics.fairing_internal.oaspl_db").to_bits(),
            fairing.oaspl_db.to_bits()
        );
    }

    #[test]
    fn point_mass_acoustics_fairing_internal_report_carries_provenance() {
        let ascent_deck_bytes =
            include_bytes!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let ascent_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/ascent-acoustic-env-v1.toml",
            ascent_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let fairing_deck_bytes = include_bytes!("../../../data/acoustics/fairing-internal-v1.toml");
        let fairing_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/fairing-internal-v1.toml",
            fairing_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            &format!(
                r#"[acoustics.ascent]
station_id = "station-a"
deck_file = "data/acoustics/ascent-acoustic-env-v1.toml"
deck_file_sha256 = "{ascent_deck_sha256}"
class_id = "attached_turbulent"
dynamic_pressure_pa = 50000.0
mach = 0.8

[acoustics.fairing_internal]
station_id = "payload-a"
deck_file = "data/acoustics/fairing-internal-v1.toml"
deck_file_sha256 = "{fairing_deck_sha256}"
class_id = "synthetic_light_fairing"
fill_factor = 1.0

[telemetry]"#,
            ),
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("fairing acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("fairing acoustics scenario runs");
        let acoustics = outcome.acoustics.as_ref().expect("acoustics report");
        assert!(acoustics.dsm.is_none());
        assert!(acoustics.iop.is_none());
        assert!(acoustics.ascent.is_some());
        let fairing = acoustics
            .fairing_internal
            .as_ref()
            .expect("fairing report emitted");

        let expected_gain_db = 10.0 * 2.0_f64.log10();
        let expected_oaspl_db = 132.0 - 8.0 + expected_gain_db;
        assert_eq!(fairing.station_id, "payload-a");
        assert_eq!(fairing.class_id, "synthetic_light_fairing");
        assert_eq!(fairing.fill_factor.to_bits(), 1.0_f64.to_bits());
        assert_eq!(fairing.deck_file_sha256_hex, fairing_deck_sha256);
        assert_eq!(
            fairing.deck_dataset_id,
            "openbmp.acoustics.fairing_internal.v1"
        );
        assert_eq!(fairing.deck_source_class, "synthetic-openbmp");
        assert_eq!(fairing.uncertainty_db.to_bits(), 2.5_f64.to_bits());
        assert!((fairing.fill_factor_gain_db - expected_gain_db).abs() <= 1.0e-12);
        assert!((fairing.oaspl_db - expected_oaspl_db).abs() <= 1.0e-12);
        assert_eq!(fairing.p95_50_oaspl_db, None);
        assert_eq!(fairing.p95_50_tolerance_factor, None);
        assert!(fairing.p95_50_environment_csv.is_empty());
        assert!(fairing.p95_50_sample_trace_csv.is_empty());
        assert_eq!(fairing.p95_50_wilks_required_sample_count, None);
        assert_eq!(fairing.p95_50_wilks_achieved_confidence, None);
        assert_eq!(fairing.p95_50_wilks_sample_count_sufficient, None);
        assert!(fairing.p95_50_wilks_sizing_csv.is_empty());
        assert_eq!(
            std::str::from_utf8(&fairing.spectrum_csv)
                .expect("spectrum CSV is UTF-8")
                .lines()
                .count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        assert_eq!(
            first_row_f64(&outcome, "acoustics.fairing_internal.oaspl_db").to_bits(),
            fairing.oaspl_db.to_bits()
        );
        assert_eq!(
            first_row_f64(&outcome, "acoustics.fairing_internal.uncertainty_db").to_bits(),
            fairing.uncertainty_db.to_bits()
        );
        assert_eq!(
            first_row_f64(&outcome, "acoustics.fairing_internal.fill_factor_gain_db").to_bits(),
            fairing.fill_factor_gain_db.to_bits()
        );
    }

    #[test]
    fn point_mass_acoustics_fairing_internal_p95_50_uses_sample_inputs() {
        let ascent_deck_bytes =
            include_bytes!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let ascent_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/ascent-acoustic-env-v1.toml",
            ascent_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let fairing_deck_bytes = include_bytes!("../../../data/acoustics/fairing-internal-v1.toml");
        let fairing_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/fairing-internal-v1.toml",
            fairing_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            &format!(
                r#"[acoustics.ascent]
station_id = "station-a"
deck_file = "data/acoustics/ascent-acoustic-env-v1.toml"
deck_file_sha256 = "{ascent_deck_sha256}"
class_id = "attached_turbulent"

[[acoustics.ascent.samples]]
sample_id = "max-q"
dynamic_pressure_pa = 50000.0
mach = 0.8

[[acoustics.ascent.samples]]
sample_id = "late-max-q"
dynamic_pressure_pa = 200000.0
mach = 0.8

[acoustics.fairing_internal]
station_id = "payload-a"
deck_file = "data/acoustics/fairing-internal-v1.toml"
deck_file_sha256 = "{fairing_deck_sha256}"
class_id = "synthetic_light_fairing"
fill_factor = 1.0

[[acoustics.fairing_internal.samples]]
sample_id = "fairing-open"
fill_factor = 0.25

[[acoustics.fairing_internal.samples]]
sample_id = "fairing-packed"
class_id = "synthetic_heavy_fairing"
fill_factor = 0.75

[telemetry]"#,
            ),
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("fairing sample acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("fairing sample acoustics scenario runs");
        let fairing = outcome
            .acoustics
            .as_ref()
            .expect("acoustics report")
            .fairing_internal
            .as_ref()
            .expect("fairing report emitted");

        assert_eq!(fairing.class_id, "synthetic_light_fairing");
        assert_eq!(fairing.fill_factor.to_bits(), 1.0_f64.to_bits());
        assert!(fairing.p95_50_oaspl_db.is_some());
        assert!(fairing.p95_50_tolerance_factor.is_some());
        let environment_csv = std::str::from_utf8(&fairing.p95_50_environment_csv)
            .expect("fairing P95/50 environment CSV is UTF-8");
        assert!(environment_csv.contains("p95_50_spl_db"));
        let sample_trace_csv = std::str::from_utf8(&fairing.p95_50_sample_trace_csv)
            .expect("fairing P95/50 sample trace CSV is UTF-8");
        assert!(
            sample_trace_csv.contains(
                "sample_index,sample_id,class_id,fill_factor,fill_factor_gain_db,oaspl_db,uncertainty_db"
            )
        );
        assert!(sample_trace_csv.contains("fairing-open"));
        assert!(sample_trace_csv.contains("synthetic_light_fairing"));
        assert!(sample_trace_csv.contains("fairing-packed"));
        assert!(sample_trace_csv.contains("synthetic_heavy_fairing"));
        assert!(sample_trace_csv.contains("2.50000000000000000e-1"));
        assert!(sample_trace_csv.contains("7.50000000000000000e-1"));
        assert_eq!(sample_trace_csv.lines().count(), 3);

        let mismatch_toml = scenario_toml.replace(
            "[telemetry]",
            r#"[[acoustics.fairing_internal.samples]]
sample_id = "extra"
fill_factor = 0.5

[telemetry]"#,
        );
        let mismatch =
            Scenario::from_toml_str_with_source_dir(&mismatch_toml, Some(workspace_root()))
                .expect("mismatched fairing sample scenario parses");
        let err = crate::run(&mismatch).expect_err("fairing sample count mismatch rejects");
        assert!(
            err.to_string()
                .contains("fairing P95/50 sample input count"),
            "expected fairing sample count mismatch error, got {err:?}"
        );
    }

    #[test]
    fn point_mass_acoustics_response_report_carries_modal_force_handoff() {
        let ascent_deck_bytes =
            include_bytes!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let ascent_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/ascent-acoustic-env-v1.toml",
            ascent_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let fairing_deck_bytes = include_bytes!("../../../data/acoustics/fairing-internal-v1.toml");
        let fairing_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/fairing-internal-v1.toml",
            fairing_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let response_deck_bytes =
            include_bytes!("../../../data/acoustics/vibroacoustic-transfer-v1.toml");
        let response_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/vibroacoustic-transfer-v1.toml",
            response_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            &format!(
                r#"[acoustics.ascent]
station_id = "station-a"
deck_file = "data/acoustics/ascent-acoustic-env-v1.toml"
deck_file_sha256 = "{ascent_deck_sha256}"
class_id = "attached_turbulent"
dynamic_pressure_pa = 50000.0
mach = 0.8

[acoustics.fairing_internal]
station_id = "payload-a"
deck_file = "data/acoustics/fairing-internal-v1.toml"
deck_file_sha256 = "{fairing_deck_sha256}"
class_id = "synthetic_light_fairing"
fill_factor = 1.0

[acoustics.response]
station_id = "payload-a"
source = "fairing_internal"
deck_file = "data/acoustics/vibroacoustic-transfer-v1.toml"
deck_file_sha256 = "{response_deck_sha256}"
class_id = "synthetic_panel_light"
surface_mass_kg_m2 = 12.0
area_m2 = 3.0
mode_shape_factor = 0.5

[telemetry]"#,
            ),
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("response acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("response acoustics scenario runs");
        let acoustics = outcome.acoustics.as_ref().expect("acoustics report");
        let response = acoustics
            .response
            .as_ref()
            .expect("response report emitted");

        let ascent_deck = openbmp_acoustics::ascent::AscentAcousticDeck::load_from_str(
            std::str::from_utf8(ascent_deck_bytes).unwrap(),
        )
        .unwrap();
        let ascent_prediction = ascent_deck
            .evaluate_class(
                "attached_turbulent",
                openbmp_acoustics::ascent::AscentAcousticInput::new(50_000.0, 0.8).unwrap(),
            )
            .unwrap();
        let fairing_deck = openbmp_acoustics::ascent::FairingInternalDeck::load_from_str(
            std::str::from_utf8(fairing_deck_bytes).unwrap(),
        )
        .unwrap();
        let fairing_prediction = fairing_deck
            .evaluate_class(
                "synthetic_light_fairing",
                &ascent_prediction.spectrum,
                openbmp_acoustics::ascent::FairingInternalInput::new(1.0).unwrap(),
            )
            .unwrap();
        let response_deck = openbmp_acoustics::response::VibroacousticTransferDeck::load_from_str(
            std::str::from_utf8(response_deck_bytes).unwrap(),
        )
        .unwrap();
        let expected = response_deck
            .evaluate_class(
                "synthetic_panel_light",
                &fairing_prediction.spectrum,
                openbmp_acoustics::response::VibroacousticResponseInput::new(12.0, 3.0, 0.5)
                    .unwrap(),
            )
            .unwrap();

        assert_eq!(response.station_id, "payload-a");
        assert_eq!(
            response.source,
            openbmp_scenario::AcousticsResponseSourceConfig::FairingInternal
        );
        assert_eq!(response.class_id, "synthetic_panel_light");
        assert_eq!(response.deck_file_sha256_hex, response_deck_sha256);
        assert_eq!(
            response.deck_dataset_id,
            "openbmp.acoustics.vibro_transfer.v1"
        );
        assert_eq!(response.deck_source_class, "synthetic-openbmp");
        assert_eq!(response.uncertainty_db.to_bits(), 5.0_f64.to_bits());
        assert_eq!(response.surface_mass_gain.to_bits(), 1.0_f64.to_bits());
        assert_eq!(
            response.modal_force_participation_m2.to_bits(),
            1.5_f64.to_bits()
        );
        assert!(
            (response.total_mean_square_acceleration_m2_s4
                - expected
                    .acceleration_psd
                    .total_mean_square_acceleration_m2_s4())
            .abs()
                <= 1.0e-18
        );
        assert!(
            (response.total_mean_square_force_n2
                - expected.modal_force_psd.total_mean_square_force_n2())
            .abs()
                <= 1.0e-18
        );
        assert_eq!(
            std::str::from_utf8(&response.acceleration_psd_csv)
                .expect("acceleration CSV is UTF-8")
                .lines()
                .count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        assert_eq!(
            std::str::from_utf8(&response.modal_force_psd_csv)
                .expect("modal force CSV is UTF-8")
                .lines()
                .count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        let loads_handoff_csv =
            std::str::from_utf8(&response.loads_handoff_csv).expect("loads handoff CSV is UTF-8");
        assert!(loads_handoff_csv.contains("band_mean_square_force_n2"));
        assert!(loads_handoff_csv.contains("total_mean_square_force_n2"));
        assert_eq!(
            loads_handoff_csv.lines().count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        assert_eq!(response.p95_50_tolerance_factor, None);
        assert_eq!(response.p95_50_total_mean_square_acceleration_m2_s4, None);
        assert_eq!(response.p95_50_total_mean_square_force_n2, None);
        assert!(response.p95_50_acceleration_psd_csv.is_empty());
        assert!(response.p95_50_modal_force_psd_csv.is_empty());
        assert!(response.p95_50_sample_trace_csv.is_empty());
        assert_eq!(response.p95_50_wilks_required_sample_count, None);
        assert_eq!(response.p95_50_wilks_achieved_confidence, None);
        assert_eq!(response.p95_50_wilks_sample_count_sufficient, None);
        assert!(response.p95_50_wilks_sizing_csv.is_empty());
        assert_eq!(
            first_row_f64(
                &outcome,
                "acoustics.response.total_mean_square_acceleration_m2_s4"
            )
            .to_bits(),
            response.total_mean_square_acceleration_m2_s4.to_bits()
        );
        assert_eq!(
            first_row_f64(&outcome, "acoustics.response.total_mean_square_force_n2").to_bits(),
            response.total_mean_square_force_n2.to_bits()
        );
        assert_eq!(
            first_row_f64(&outcome, "acoustics.response.uncertainty_db").to_bits(),
            response.uncertainty_db.to_bits()
        );
    }

    #[test]
    fn point_mass_acoustics_response_p95_50_from_fairing_trace() {
        let ascent_deck_bytes =
            include_bytes!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let ascent_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/ascent-acoustic-env-v1.toml",
            ascent_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let fairing_deck_bytes = include_bytes!("../../../data/acoustics/fairing-internal-v1.toml");
        let fairing_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/fairing-internal-v1.toml",
            fairing_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let response_deck_bytes =
            include_bytes!("../../../data/acoustics/vibroacoustic-transfer-v1.toml");
        let response_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/vibroacoustic-transfer-v1.toml",
            response_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            &format!(
                r#"[acoustics.ascent]
station_id = "station-a"
deck_file = "data/acoustics/ascent-acoustic-env-v1.toml"
deck_file_sha256 = "{ascent_deck_sha256}"
class_id = "attached_turbulent"

[[acoustics.ascent.samples]]
sample_id = "max-q"
dynamic_pressure_pa = 50000.0
mach = 0.8

[[acoustics.ascent.samples]]
sample_id = "late-max-q"
dynamic_pressure_pa = 200000.0
mach = 0.8

[acoustics.fairing_internal]
station_id = "payload-a"
deck_file = "data/acoustics/fairing-internal-v1.toml"
deck_file_sha256 = "{fairing_deck_sha256}"
class_id = "synthetic_light_fairing"
fill_factor = 1.0

[acoustics.response]
station_id = "payload-a"
source = "fairing_internal"
deck_file = "data/acoustics/vibroacoustic-transfer-v1.toml"
deck_file_sha256 = "{response_deck_sha256}"
class_id = "synthetic_panel_light"
surface_mass_kg_m2 = 12.0
area_m2 = 3.0
mode_shape_factor = 0.5

[telemetry]"#,
            ),
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("response P95/50 acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("response P95/50 acoustics scenario runs");
        let response = outcome
            .acoustics
            .as_ref()
            .expect("acoustics report")
            .response
            .as_ref()
            .expect("response report emitted");

        assert_eq!(
            response.source,
            openbmp_scenario::AcousticsResponseSourceConfig::FairingInternal
        );
        let tolerance_factor = response
            .p95_50_tolerance_factor
            .expect("response P95/50 factor emitted");
        let expected_wilks_required = openbmp_mc::wilks_one_sided_n(
            openbmp_acoustics::stats::P95_50_COVERAGE_PROBABILITY,
            openbmp_acoustics::stats::P95_50_CONFIDENCE_PROBABILITY,
        )
        .expect("Wilks floor");
        let expected_wilks_achieved = openbmp_mc::wilks_one_sided_confidence(
            openbmp_acoustics::stats::P95_50_COVERAGE_PROBABILITY,
            2,
        )
        .expect("Wilks confidence");
        assert!(
            (tolerance_factor
                - openbmp_physics::inverse_standard_normal_cdf(
                    openbmp_acoustics::stats::P95_50_COVERAGE_PROBABILITY,
                ))
            .abs()
                <= 1.0e-12
        );
        assert!(
            response
                .p95_50_total_mean_square_acceleration_m2_s4
                .expect("response P95/50 acceleration emitted")
                > response.total_mean_square_acceleration_m2_s4
        );
        assert!(
            response
                .p95_50_total_mean_square_force_n2
                .expect("response P95/50 modal force emitted")
                > response.total_mean_square_force_n2
        );
        let acceleration_csv = std::str::from_utf8(&response.p95_50_acceleration_psd_csv)
            .expect("response P95/50 acceleration CSV is UTF-8");
        let modal_force_csv = std::str::from_utf8(&response.p95_50_modal_force_psd_csv)
            .expect("response P95/50 modal-force CSV is UTF-8");
        assert!(acceleration_csv.contains("p95_50_acceleration_psd_m2_s4_per_hz"));
        assert!(modal_force_csv.contains("p95_50_modal_force_psd_n2_per_hz"));
        let sample_trace_csv = std::str::from_utf8(&response.p95_50_sample_trace_csv)
            .expect("response P95/50 sample trace CSV is UTF-8");
        assert!(sample_trace_csv.contains(
            "sample_index,sample_id,class_id,surface_mass_kg_m2,area_m2,mode_shape_factor"
        ));
        assert!(sample_trace_csv.contains("0,,synthetic_panel_light"));
        assert!(sample_trace_csv.contains("1,,synthetic_panel_light"));
        assert_eq!(sample_trace_csv.lines().count(), 3);
        assert_eq!(
            acceleration_csv.lines().count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        assert_eq!(
            modal_force_csv.lines().count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        assert_eq!(
            response.p95_50_wilks_required_sample_count,
            Some(expected_wilks_required)
        );
        assert_eq!(
            response
                .p95_50_wilks_achieved_confidence
                .expect("response Wilks confidence")
                .to_bits(),
            expected_wilks_achieved.to_bits()
        );
        assert_eq!(response.p95_50_wilks_sample_count_sufficient, Some(false));
        let response_wilks_csv = std::str::from_utf8(&response.p95_50_wilks_sizing_csv)
            .expect("response Wilks CSV is UTF-8");
        assert!(response_wilks_csv.contains("required_sample_count"));
        assert_eq!(response_wilks_csv.lines().count(), 2);
    }

    #[test]
    fn point_mass_acoustics_response_p95_50_uses_sample_inputs() {
        let ascent_deck_bytes =
            include_bytes!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let ascent_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/ascent-acoustic-env-v1.toml",
            ascent_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let fairing_deck_bytes = include_bytes!("../../../data/acoustics/fairing-internal-v1.toml");
        let fairing_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/fairing-internal-v1.toml",
            fairing_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let response_deck_bytes =
            include_bytes!("../../../data/acoustics/vibroacoustic-transfer-v1.toml");
        let response_deck_sha256 = openbmp_scenario::ResolvedFile::from_bytes(
            "data/acoustics/vibroacoustic-transfer-v1.toml",
            response_deck_bytes.to_vec(),
        )
        .sha256_hex;
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            &format!(
                r#"[acoustics.ascent]
station_id = "station-a"
deck_file = "data/acoustics/ascent-acoustic-env-v1.toml"
deck_file_sha256 = "{ascent_deck_sha256}"
class_id = "attached_turbulent"

[[acoustics.ascent.samples]]
sample_id = "max-q"
dynamic_pressure_pa = 50000.0
mach = 0.8

[[acoustics.ascent.samples]]
sample_id = "late-max-q"
dynamic_pressure_pa = 200000.0
mach = 0.8

[acoustics.fairing_internal]
station_id = "payload-a"
deck_file = "data/acoustics/fairing-internal-v1.toml"
deck_file_sha256 = "{fairing_deck_sha256}"
class_id = "synthetic_light_fairing"
fill_factor = 1.0

[acoustics.response]
station_id = "payload-a"
source = "fairing_internal"
deck_file = "data/acoustics/vibroacoustic-transfer-v1.toml"
deck_file_sha256 = "{response_deck_sha256}"
class_id = "synthetic_panel_light"
surface_mass_kg_m2 = 12.0
area_m2 = 3.0
mode_shape_factor = 0.5

[[acoustics.response.samples]]
sample_id = "panel-light"
surface_mass_kg_m2 = 12.0
area_m2 = 3.0
mode_shape_factor = 0.5

[[acoustics.response.samples]]
sample_id = "panel-heavy"
class_id = "synthetic_panel_heavy"
surface_mass_kg_m2 = 24.0
area_m2 = 4.0
mode_shape_factor = 0.75

[telemetry]"#,
            ),
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("sampled response P95/50 acoustics scenario parses");
        let outcome =
            crate::run(&scenario).expect("sampled response P95/50 acoustics scenario runs");
        let response = outcome
            .acoustics
            .as_ref()
            .expect("acoustics report")
            .response
            .as_ref()
            .expect("response report emitted");

        assert!(
            response
                .p95_50_total_mean_square_acceleration_m2_s4
                .is_some()
        );
        assert!(response.p95_50_total_mean_square_force_n2.is_some());
        let sample_trace_csv = std::str::from_utf8(&response.p95_50_sample_trace_csv)
            .expect("response P95/50 sample trace CSV is UTF-8");
        assert!(sample_trace_csv.contains("0,panel-light,synthetic_panel_light"));
        assert!(sample_trace_csv.contains("1,panel-heavy,synthetic_panel_heavy"));
        assert!(sample_trace_csv.contains(",2.40000000000000000e1,4.00000000000000000e0,7.50000000000000000e-1,3.00000000000000000e0,"));

        let mismatched = scenario_toml.replace(
            "[telemetry]",
            r#"[[acoustics.response.samples]]
sample_id = "extra-panel"
surface_mass_kg_m2 = 18.0
area_m2 = 3.0
mode_shape_factor = 0.5

[telemetry]"#,
        );
        let scenario = Scenario::from_toml_str_with_source_dir(&mismatched, Some(workspace_root()))
            .expect("mismatched response sample scenario parses");
        let err = crate::run(&scenario).expect_err("mismatched response sample count rejects");
        assert!(
            err.to_string().contains(
                "response P95/50 sample input count 3 does not match source sample spectrum count 2"
            ),
            "expected response sample count mismatch rejection, got {err:?}"
        );
    }

    #[test]
    fn point_mass_acoustics_iop_report_matches_closed_form_and_telemetry() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.iop]
station_id = "station-a"
gas_generation_rate_n_per_s = 2400000.0
ramp_duration_s = 0.25
duct_volume_m3 = 120.0
duct_exit_area_m2 = 6.0
duct_discharge_velocity_m_s = 40.0
duct_geometry_gain = 1.5
receiver_distance_m = 30.0
suppression_loss_db = 3.0
uncertainty_minus_db = 2.0
uncertainty_plus_db = 4.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str(&scenario_toml).expect("IOP acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("IOP acoustics scenario runs");
        let acoustics = outcome
            .acoustics
            .as_ref()
            .expect("acoustics report emitted");
        assert!(acoustics.dsm.is_none());
        let iop = acoustics.iop.as_ref().expect("IOP report emitted");
        let residence_time_s = 120.0 / (6.0 * 40.0);
        let unsuppressed_peak_pa =
            2_400_000.0 * residence_time_s * 1.5 / (4.0 * std::f64::consts::PI * 30.0 * 30.0);
        let expected_peak_pa = unsuppressed_peak_pa * 10.0_f64.powf(-3.0 / 20.0);
        let expected_lower_pa = expected_peak_pa * 10.0_f64.powf(-2.0 / 20.0);
        let expected_upper_pa = expected_peak_pa * 10.0_f64.powf(4.0 / 20.0);

        assert_eq!(iop.station_id, "station-a");
        assert_eq!(iop.source_engine_id, None);
        assert_eq!(iop.source_engine_ignition_event_id, None);
        assert_eq!(iop.source_engine_ignition_time_s, None);
        assert_eq!(iop.source_engine_observed_ignition_event_id, None);
        assert_eq!(iop.source_engine_observed_ignition_step, None);
        assert_eq!(iop.source_engine_observed_ignition_time_s, None);
        assert_eq!(iop.source_engine_max_thrust_n, None);
        assert_eq!(iop.source_engine_mdot_cstar_n, None);
        assert_eq!(iop.source_engine_effective_cstar_m_s, None);
        assert_eq!(iop.source_engine_runtime_sample_count, 0);
        assert_eq!(iop.source_engine_runtime_peak_source_rate_n_per_s, None);
        assert_eq!(iop.source_engine_runtime_peak_overpressure_pa, None);
        assert_eq!(iop.source_engine_runtime_lower_peak_overpressure_pa, None);
        assert_eq!(iop.source_engine_runtime_upper_peak_overpressure_pa, None);
        assert_eq!(iop.source_engine_runtime_duration_s, None);
        assert!(iop.source_engine_runtime_history_csv.is_empty());
        assert!(!iop.source_engine_thermochemical_performance);
        assert_eq!(
            iop.gas_generation_rate_n_per_s.to_bits(),
            2_400_000.0_f64.to_bits()
        );
        assert_eq!(iop.ramp_duration_s.to_bits(), 0.25_f64.to_bits());
        assert_eq!(
            iop.duct_residence_time_s.to_bits(),
            residence_time_s.to_bits()
        );
        assert_eq!(iop.duct_volume_m3.to_bits(), 120.0_f64.to_bits());
        assert_eq!(iop.duct_exit_area_m2.to_bits(), 6.0_f64.to_bits());
        assert_eq!(
            iop.duct_discharge_velocity_m_s.to_bits(),
            40.0_f64.to_bits()
        );
        assert_eq!(iop.duct_geometry_gain.to_bits(), 1.5_f64.to_bits());
        assert_eq!(iop.duct_deck_file_sha256_hex, None);
        assert_eq!(iop.duct_deck_dataset_id, None);
        assert_eq!(iop.duct_deck_source_class, None);
        assert_eq!(iop.duct_class_id, None);
        assert_eq!(iop.receiver_distance_m.to_bits(), 30.0_f64.to_bits());
        assert_eq!(iop.suppression_loss_db.to_bits(), 3.0_f64.to_bits());
        assert_eq!(iop.suppression_deck_file_sha256_hex, None);
        assert_eq!(iop.suppression_deck_dataset_id, None);
        assert_eq!(iop.suppression_deck_source_class, None);
        assert_eq!(iop.suppression_class_id, None);
        assert_eq!(iop.suppression_class_loss_db, None);
        assert_eq!(iop.suppression_class_uncertainty_db, None);
        assert_eq!(iop.uncertainty_minus_db.to_bits(), 2.0_f64.to_bits());
        assert_eq!(iop.uncertainty_plus_db.to_bits(), 4.0_f64.to_bits());
        assert_relative_eq!(
            iop.peak_overpressure_pa,
            expected_peak_pa,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            iop.lower_peak_overpressure_pa,
            expected_lower_pa,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            iop.upper_peak_overpressure_pa,
            expected_upper_pa,
            epsilon = 1.0e-12
        );
        assert_eq!(iop.duration_s.to_bits(), 0.75_f64.to_bits());
        assert_relative_eq!(
            first_row_f64(&outcome, "acoustics.iop.peak_overpressure_pa"),
            iop.peak_overpressure_pa,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            first_row_f64(&outcome, "acoustics.iop.lower_peak_overpressure_pa"),
            iop.lower_peak_overpressure_pa,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            first_row_f64(&outcome, "acoustics.iop.upper_peak_overpressure_pa"),
            iop.upper_peak_overpressure_pa,
            epsilon = 1.0e-12
        );
        assert_eq!(
            first_row_f64(&outcome, "acoustics.iop.duration_s").to_bits(),
            iop.duration_s.to_bits()
        );
    }

    #[test]
    fn point_mass_acoustics_iop_source_engine_derives_start_transient() {
        let scenario_toml = add_iop_source_engine(&COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.iop]
station_id = "station-a"
source_engine_id = "main"
duct_volume_m3 = 120.0
duct_exit_area_m2 = 6.0
duct_discharge_velocity_m_s = 40.0
duct_geometry_gain = 1.5
receiver_distance_m = 30.0
suppression_loss_db = 3.0
uncertainty_minus_db = 2.0
uncertainty_plus_db = 4.0

[telemetry]"#,
        ));
        let scenario = Scenario::from_toml_str(&scenario_toml)
            .expect("IOP source-engine acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("IOP source-engine acoustics scenario runs");
        let iop = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.iop.as_ref())
            .expect("IOP report emitted");
        let residence_time_s = 120.0 / (6.0 * 40.0);
        let gas_generation_rate_n_per_s = 600_000.0 / 0.25;
        let unsuppressed_peak_pa = gas_generation_rate_n_per_s * residence_time_s * 1.5
            / (4.0 * std::f64::consts::PI * 30.0 * 30.0);
        let expected_peak_pa = unsuppressed_peak_pa * 10.0_f64.powf(-3.0 / 20.0);

        assert_eq!(iop.source_engine_id.as_deref(), Some("main"));
        assert_eq!(iop.source_engine_ignition_event_id, None);
        assert_eq!(iop.source_engine_ignition_time_s, None);
        assert_eq!(iop.source_engine_observed_ignition_event_id, None);
        assert_eq!(iop.source_engine_observed_ignition_step, None);
        assert_eq!(iop.source_engine_observed_ignition_time_s, None);
        assert_eq!(
            iop.source_engine_max_thrust_n.map(f64::to_bits),
            Some(600_000.0_f64.to_bits())
        );
        assert_eq!(iop.source_engine_mdot_cstar_n, None);
        assert_eq!(iop.source_engine_effective_cstar_m_s, None);
        assert!(!iop.source_engine_thermochemical_performance);
        assert_eq!(
            iop.gas_generation_rate_n_per_s.to_bits(),
            gas_generation_rate_n_per_s.to_bits()
        );
        assert_eq!(iop.ramp_duration_s.to_bits(), 0.25_f64.to_bits());
        assert_relative_eq!(
            iop.peak_overpressure_pa,
            expected_peak_pa,
            epsilon = 1.0e-12
        );
        assert_eq!(iop.duration_s.to_bits(), 0.75_f64.to_bits());
        assert_relative_eq!(
            first_row_f64(&outcome, "acoustics.iop.peak_overpressure_pa"),
            iop.peak_overpressure_pa,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn point_mass_acoustics_iop_source_engine_uses_thermochemical_performance() {
        let scenario_toml = add_iop_source_engine(&COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[propulsion.thermochem]
file = "crates/openbmp-runner/tests/fixtures/thermochem/synthetic-grain.toml"
file_sha256 = "fa3ff43bc91d04317d44a141cf5ff28680e4e0b35d89ff3c91d92ee60f1a842b"
chamber_pressure_pa = 2000000.0
mixture_ratio = 2.5

[vehicle.assembly.engines.thermochemical_performance]
throat_area_m2 = 0.02
exit_area_m2 = 0.24
ambient_pressure_pa = 101325.0
separation = "off"

[acoustics.iop]
station_id = "station-a"
source_engine_id = "main"
duct_volume_m3 = 120.0
duct_exit_area_m2 = 6.0
duct_discharge_velocity_m_s = 40.0
duct_geometry_gain = 1.5
receiver_distance_m = 30.0
suppression_loss_db = 3.0
uncertainty_minus_db = 2.0
uncertainty_plus_db = 4.0

[telemetry]"#,
        ));
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("IOP thermochemical source-engine acoustics scenario parses");
        let outcome = crate::run(&scenario)
            .expect("IOP thermochemical source-engine acoustics scenario runs");
        let iop = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.iop.as_ref())
            .expect("IOP report emitted");
        let deck = openbmp_thermochem::ThermochemTable::load_from_str(include_str!(
            "../tests/fixtures/thermochem/synthetic-grain.toml"
        ))
        .expect("fixture thermochemistry deck parses");
        let state = deck
            .lookup(openbmp_thermochem::ThermochemQuery {
                chamber_pressure_pa: 2_000_000.0,
                mixture_ratio: 2.5,
            })
            .expect("fixture thermochemistry lookup succeeds");
        let expected =
            openbmp_propulsion::LiquidEnginePerformance::from_thermochemistry_with_efficiency(
                openbmp_propulsion::LiquidEngineThermochemistry {
                    chamber_pressure_pa: 2_000_000.0,
                    c_star_m_s: state.c_star_m_s,
                    gamma: state.gamma,
                },
                openbmp_propulsion::LiquidEngineNozzle {
                    throat_area_m2: 0.02,
                    exit_area_m2: 0.24,
                    ambient_pressure_pa: 101_325.0,
                    separation: openbmp_propulsion::NozzleSeparationCriterion::Off,
                },
                openbmp_propulsion::LiquidEngineCStarEfficiencyBand {
                    min: state.c_star_efficiency.min,
                    nominal: state.c_star_efficiency.nominal,
                    max: state.c_star_efficiency.max,
                },
            )
            .expect("fixture thermochemical performance solves");
        let expected_mdot_cstar_n =
            expected.mass_flow_kg_per_s * state.c_star_m_s * state.c_star_efficiency.nominal;
        let gas_generation_rate_n_per_s = expected_mdot_cstar_n / 0.25;
        let residence_time_s = 120.0 / (6.0 * 40.0);
        let unsuppressed_peak_pa = gas_generation_rate_n_per_s * residence_time_s * 1.5
            / (4.0 * std::f64::consts::PI * 30.0 * 30.0);
        let expected_peak_pa = unsuppressed_peak_pa * 10.0_f64.powf(-3.0 / 20.0);

        assert_eq!(iop.source_engine_id.as_deref(), Some("main"));
        assert_eq!(iop.source_engine_ignition_event_id, None);
        assert_eq!(iop.source_engine_ignition_time_s, None);
        assert_eq!(iop.source_engine_observed_ignition_event_id, None);
        assert_eq!(iop.source_engine_observed_ignition_step, None);
        assert_eq!(iop.source_engine_observed_ignition_time_s, None);
        assert!(iop.source_engine_thermochemical_performance);
        assert_eq!(
            iop.source_engine_max_thrust_n.map(f64::to_bits),
            Some(expected.max_thrust_n.to_bits())
        );
        assert_eq!(
            iop.source_engine_mdot_cstar_n.map(f64::to_bits),
            Some(expected_mdot_cstar_n.to_bits())
        );
        assert_eq!(
            iop.source_engine_effective_cstar_m_s.map(f64::to_bits),
            Some((state.c_star_m_s * state.c_star_efficiency.nominal).to_bits())
        );
        assert_ne!(
            iop.source_engine_max_thrust_n.map(f64::to_bits),
            Some(600_000.0_f64.to_bits())
        );
        assert_ne!(
            iop.gas_generation_rate_n_per_s.to_bits(),
            (expected.max_thrust_n / 0.25).to_bits()
        );
        assert_eq!(
            iop.gas_generation_rate_n_per_s.to_bits(),
            gas_generation_rate_n_per_s.to_bits()
        );
        assert_relative_eq!(
            iop.peak_overpressure_pa,
            expected_peak_pa,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            first_row_f64(&outcome, "acoustics.iop.peak_overpressure_pa"),
            iop.peak_overpressure_pa,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn point_mass_acoustics_iop_source_engine_reports_scripted_ignite_time() {
        let scenario_toml = add_iop_source_engine(&COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.iop]
station_id = "station-a"
source_engine_id = "main"
duct_volume_m3 = 120.0
duct_exit_area_m2 = 6.0
duct_discharge_velocity_m_s = 40.0
duct_geometry_gain = 1.5
receiver_distance_m = 30.0
suppression_loss_db = 3.0
uncertainty_minus_db = 2.0
uncertainty_plus_db = 4.0

[mission]
initial_phase = "powered"

[[mission.phases]]
id = "powered"
label = "Powered"

[scenario_script]
[[scenario_script.events]]
id = "main_ignite_late"
trigger = { kind = "at_time", time_s = 2.0 }
action = { kind = "engine_command", id = "main", command = { throttle_unit = 1.0, gimbal_pitch_rad = 0.0, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }

[[scenario_script.events]]
id = "main_ignite"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "engine_command", id = "main", command = { throttle_unit = 1.0, gimbal_pitch_rad = 0.0, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }

[telemetry]"#,
        ));
        let scenario = Scenario::from_toml_str(&scenario_toml)
            .expect("IOP timed source-engine acoustics scenario parses");
        let outcome =
            crate::run(&scenario).expect("IOP timed source-engine acoustics scenario runs");
        let iop = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.iop.as_ref())
            .expect("IOP report emitted");

        assert_eq!(iop.source_engine_id.as_deref(), Some("main"));
        assert_eq!(
            iop.source_engine_max_thrust_n.map(f64::to_bits),
            Some(600_000.0_f64.to_bits())
        );
        assert_eq!(iop.source_engine_mdot_cstar_n, None);
        assert_eq!(
            iop.source_engine_ignition_event_id.as_deref(),
            Some("main_ignite")
        );
        assert_eq!(
            iop.source_engine_ignition_time_s.map(f64::to_bits),
            Some(0.05_f64.to_bits())
        );
        assert_eq!(
            iop.source_engine_observed_ignition_event_id,
            Some(openbmp_sim::EventId::from_path("scenario_script.events.main_ignite").value())
        );
        assert_eq!(iop.source_engine_observed_ignition_step, Some(1));
        assert_eq!(
            iop.source_engine_observed_ignition_time_s.map(f64::to_bits),
            Some(0.1_f64.to_bits())
        );
        assert_eq!(iop.source_engine_runtime_sample_count, 2);
        assert_eq!(
            iop.source_engine_runtime_last_source_strength_n
                .map(f64::to_bits),
            Some(240_000.0_f64.to_bits())
        );
        assert_eq!(
            iop.source_engine_runtime_last_time_s.map(f64::to_bits),
            Some(0.1_f64.to_bits())
        );
        assert_relative_eq!(
            iop.source_engine_runtime_peak_source_rate_n_per_s
                .expect("runtime source-rate peak recorded"),
            2_400_000.0,
            epsilon = 1.0e-9
        );
        assert_relative_eq!(
            iop.source_engine_runtime_peak_overpressure_pa
                .expect("runtime peak pressure recorded"),
            iop.peak_overpressure_pa,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            iop.source_engine_runtime_lower_peak_overpressure_pa
                .expect("runtime lower peak pressure recorded"),
            iop.lower_peak_overpressure_pa,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            iop.source_engine_runtime_upper_peak_overpressure_pa
                .expect("runtime upper peak pressure recorded"),
            iop.upper_peak_overpressure_pa,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            iop.source_engine_runtime_duration_s
                .expect("runtime duration recorded"),
            iop.duration_s,
            epsilon = 1.0e-12
        );
        let history = std::str::from_utf8(&iop.source_engine_runtime_history_csv)
            .expect("runtime source history is UTF-8 CSV");
        let rows = history.trim_end().lines().collect::<Vec<_>>();
        assert_eq!(
            rows[0],
            "step,time_s,source_strength_n,source_rate_n_per_s,thrust_n,mass_flow_kg_per_s"
        );
        assert_eq!(rows.len(), 3);
        let final_row = rows[2].split(',').collect::<Vec<_>>();
        assert_eq!(final_row[0], "1");
        assert_relative_eq!(
            final_row[2]
                .parse::<f64>()
                .expect("source strength column parses"),
            240_000.0,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            final_row[3]
                .parse::<f64>()
                .expect("source rate column parses"),
            2_400_000.0,
            epsilon = 1.0e-9
        );
    }

    #[test]
    fn point_mass_acoustics_iop_duct_deck_report_carries_provenance() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.iop]
station_id = "station-a"
gas_generation_rate_n_per_s = 2400000.0
ramp_duration_s = 0.25
duct_deck_file = "data/acoustics/iop-duct-v1.toml"
duct_deck_file_sha256 = "7711a6c472fc5343fda1452626ad7ff41323c8510b6d82d8de6d7b707e807c7e"
duct_class = "long_trench"
receiver_distance_m = 30.0
suppression_loss_db = 3.0
uncertainty_minus_db = 2.0
uncertainty_plus_db = 4.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("IOP duct-deck acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("IOP duct-deck acoustics scenario runs");
        let iop = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.iop.as_ref())
            .expect("IOP report emitted");
        let residence_time_s = 300.0 / (9.0 * 35.0);
        let unsuppressed_peak_pa =
            2_400_000.0 * residence_time_s * 2.2 / (4.0 * std::f64::consts::PI * 30.0 * 30.0);
        let expected_peak_pa = unsuppressed_peak_pa * 10.0_f64.powf(-3.0 / 20.0);

        assert_eq!(iop.source_engine_id, None);
        assert_eq!(iop.source_engine_observed_ignition_event_id, None);
        assert_eq!(iop.source_engine_observed_ignition_step, None);
        assert_eq!(iop.source_engine_observed_ignition_time_s, None);
        assert_eq!(iop.source_engine_max_thrust_n, None);
        assert_eq!(iop.source_engine_mdot_cstar_n, None);
        assert!(!iop.source_engine_thermochemical_performance);
        assert_eq!(
            iop.duct_deck_file_sha256_hex.as_deref(),
            Some("7711a6c472fc5343fda1452626ad7ff41323c8510b6d82d8de6d7b707e807c7e")
        );
        assert_eq!(
            iop.duct_deck_dataset_id.as_deref(),
            Some("openbmp.acoustics.iop_duct.v1")
        );
        assert_eq!(
            iop.duct_deck_source_class.as_deref(),
            Some("synthetic-openbmp")
        );
        assert_eq!(iop.duct_class_id.as_deref(), Some("long_trench"));
        assert_eq!(iop.duct_volume_m3.to_bits(), 300.0_f64.to_bits());
        assert_eq!(iop.duct_exit_area_m2.to_bits(), 9.0_f64.to_bits());
        assert_eq!(
            iop.duct_discharge_velocity_m_s.to_bits(),
            35.0_f64.to_bits()
        );
        assert_eq!(iop.duct_geometry_gain.to_bits(), 2.2_f64.to_bits());
        assert_eq!(
            iop.duct_residence_time_s.to_bits(),
            residence_time_s.to_bits()
        );
        assert_eq!(iop.suppression_deck_file_sha256_hex, None);
        assert_eq!(iop.suppression_deck_dataset_id, None);
        assert_eq!(iop.suppression_deck_source_class, None);
        assert_eq!(iop.suppression_class_id, None);
        assert_eq!(iop.suppression_class_loss_db, None);
        assert_eq!(iop.suppression_class_uncertainty_db, None);
        assert_relative_eq!(
            iop.peak_overpressure_pa,
            expected_peak_pa,
            epsilon = 1.0e-12
        );
        assert_eq!(
            iop.duration_s.to_bits(),
            (0.25 + residence_time_s).to_bits()
        );
        assert_relative_eq!(
            first_row_f64(&outcome, "acoustics.iop.peak_overpressure_pa"),
            iop.peak_overpressure_pa,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn point_mass_acoustics_iop_suppression_deck_report_carries_provenance() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.iop]
station_id = "station-a"
gas_generation_rate_n_per_s = 2400000.0
ramp_duration_s = 0.25
duct_volume_m3 = 120.0
duct_exit_area_m2 = 6.0
duct_discharge_velocity_m_s = 40.0
duct_geometry_gain = 1.5
receiver_distance_m = 30.0
suppression_deck_file = "data/acoustics/iop-suppression-v1.toml"
suppression_deck_file_sha256 = "641894d3cd5ac3dbc8529219b47e39deb1c0974121724fb5023f879f98e9ecc7"
suppression_class = "water_deluge_nominal"
suppression_loss_db = 1.0
uncertainty_minus_db = 2.0
uncertainty_plus_db = 4.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("IOP suppression-deck acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("IOP suppression-deck acoustics scenario runs");
        let iop = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.iop.as_ref())
            .expect("IOP report emitted");
        let residence_time_s = 120.0 / (6.0 * 40.0);
        let unsuppressed_peak_pa =
            2_400_000.0 * residence_time_s * 1.5 / (4.0 * std::f64::consts::PI * 30.0 * 30.0);
        let expected_peak_pa = unsuppressed_peak_pa * 10.0_f64.powf(-4.0 / 20.0);

        assert_eq!(iop.source_engine_id, None);
        assert_eq!(iop.source_engine_observed_ignition_event_id, None);
        assert_eq!(iop.source_engine_observed_ignition_step, None);
        assert_eq!(iop.source_engine_observed_ignition_time_s, None);
        assert_eq!(iop.source_engine_max_thrust_n, None);
        assert_eq!(iop.source_engine_mdot_cstar_n, None);
        assert!(!iop.source_engine_thermochemical_performance);
        assert_eq!(
            iop.suppression_deck_file_sha256_hex.as_deref(),
            Some("641894d3cd5ac3dbc8529219b47e39deb1c0974121724fb5023f879f98e9ecc7")
        );
        assert_eq!(
            iop.suppression_deck_dataset_id.as_deref(),
            Some("openbmp.acoustics.iop_suppression.v1")
        );
        assert_eq!(
            iop.suppression_deck_source_class.as_deref(),
            Some("synthetic-openbmp")
        );
        assert_eq!(
            iop.suppression_class_id.as_deref(),
            Some("water_deluge_nominal")
        );
        assert_eq!(
            iop.suppression_class_loss_db.map(f64::to_bits),
            Some(3.0_f64.to_bits())
        );
        assert_eq!(
            iop.suppression_class_uncertainty_db.map(f64::to_bits),
            Some(1.5_f64.to_bits())
        );
        assert_eq!(iop.suppression_loss_db.to_bits(), 4.0_f64.to_bits());
        assert_relative_eq!(
            iop.peak_overpressure_pa,
            expected_peak_pa,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            first_row_f64(&outcome, "acoustics.iop.peak_overpressure_pa"),
            iop.peak_overpressure_pa,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn point_mass_acoustics_report_matches_dsm_fixtures() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.dsm]
station_id = "station-a"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001

[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 2.0

[[acoustics.dsm.source_line]]
axial_m = 20.0
weight = 1.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[[acoustics.dsm.liftoff_steps]]
time_s = 2.0
vehicle_height_m = 100.0

[telemetry]"#,
        );
        let scenario = Scenario::from_toml_str(&scenario_toml).expect("scenario parses");
        let outcome = crate::run(&scenario).expect("acoustics scenario runs");
        let acoustics = outcome
            .acoustics
            .as_ref()
            .expect("acoustics report emitted");
        let dsm = acoustics.dsm.as_ref().expect("dsm report emitted");

        assert_eq!(dsm.station_id, "station-a");
        assert_eq!(dsm.source_count, 3);
        assert_eq!(dsm.acoustic_power_w.to_bits(), 200_000.0_f64.to_bits());
        assert_relative_eq!(
            dsm.sound_power_level_db,
            10.0 * (200_000.0_f64 / 1.0e-12).log10(),
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            dsm.station_oaspl_db,
            132.214_258_788_974_58,
            epsilon = 1.0e-12
        );
        assert_eq!(
            first_row_f64(&outcome, "acoustics.dsm.acoustic_power_w").to_bits(),
            dsm.acoustic_power_w.to_bits()
        );
        assert_relative_eq!(
            first_row_f64(&outcome, "acoustics.dsm.sound_power_level_db"),
            dsm.sound_power_level_db,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            first_row_f64(&outcome, "acoustics.dsm.station_oaspl_db"),
            dsm.station_oaspl_db,
            epsilon = 1.0e-12
        );
        assert_eq!(first_row_i64(&outcome, "acoustics.dsm.source_count"), 3);
        assert_eq!(
            dsm.station_spectrum_csv.as_slice(),
            include_bytes!("../../openbmp-acoustics/tests/fixtures/dsm-free-field-station.csv")
        );
        assert_eq!(
            dsm.liftoff_oaspl_csv.as_slice(),
            include_bytes!("../../openbmp-acoustics/tests/fixtures/dsm-liftoff-oaspl.csv")
        );
    }

    #[test]
    fn point_mass_acoustics_cluster_engine_count_scales_oaspl() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.dsm]
station_id = "station-a"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
mass_flow_kg_s = 10.0
cluster_engine_count = 4
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001

[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 2.0

[[acoustics.dsm.source_line]]
axial_m = 20.0
weight = 1.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str(&scenario_toml).expect("cluster acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("cluster acoustics scenario runs");
        let dsm = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.dsm.as_ref())
            .expect("dsm report emitted");
        let cluster_gain_db = 10.0 * 4.0_f64.log10();

        assert_eq!(dsm.cluster_engine_count, 4);
        assert_relative_eq!(
            dsm.cluster_power_gain_db,
            cluster_gain_db,
            epsilon = 1.0e-12
        );
        assert_eq!(dsm.source_count, 3);
        assert_eq!(dsm.acoustic_power_w.to_bits(), 800_000.0_f64.to_bits());
        assert_relative_eq!(
            dsm.sound_power_level_db,
            10.0 * (800_000.0_f64 / 1.0e-12).log10(),
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            dsm.station_oaspl_db,
            132.214_258_788_974_58 + cluster_gain_db,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            first_row_f64(&outcome, "acoustics.dsm.station_oaspl_db"),
            dsm.station_oaspl_db,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn point_mass_acoustics_cluster_engine_offsets_sum_geometrically() {
        fn scenario_toml(receiver_radial_m: f64, cluster_engines: &str) -> String {
            COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
                "[telemetry]",
                &format!(
                    r#"[acoustics.dsm]
station_id = "station-a"
receiver_axial_m = 5.0
receiver_radial_m = {receiver_radial_m:.1}
receiver_azimuth_deg = 0.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001
{cluster_engines}

[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 2.0

[[acoustics.dsm.source_line]]
axial_m = 20.0
weight = 1.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#
                ),
            )
        }

        fn run_dsm(scenario_toml: &str) -> crate::acoustics::AcousticsDsmRunReport {
            let scenario = Scenario::from_toml_str(scenario_toml).expect("scenario parses");
            crate::run(&scenario)
                .expect("scenario runs")
                .acoustics
                .expect("acoustics report")
                .dsm
                .expect("dsm report")
        }

        fn energy_add_db(lhs_db: f64, rhs_db: f64) -> f64 {
            10.0 * (10.0_f64.powf(lhs_db / 10.0) + 10.0_f64.powf(rhs_db / 10.0)).log10()
        }

        let center = run_dsm(&scenario_toml(30.0, ""));
        let near = run_dsm(&scenario_toml(20.0, ""));
        let cluster = run_dsm(&scenario_toml(
            30.0,
            r#"[[acoustics.dsm.cluster_engines]]
id = "center"
offset_x_m = 0.0
offset_y_m = 0.0

[[acoustics.dsm.cluster_engines]]
id = "near"
offset_x_m = 10.0
offset_y_m = 0.0"#,
        ));
        let expected_oaspl_db = energy_add_db(center.station_oaspl_db, near.station_oaspl_db);
        let same_location_gain_db = 10.0 * 2.0_f64.log10();

        assert_eq!(cluster.cluster_engine_count, 2);
        assert_relative_eq!(
            cluster.cluster_power_gain_db,
            same_location_gain_db,
            epsilon = 1.0e-12
        );
        assert_eq!(cluster.source_count, 6);
        assert_eq!(cluster.acoustic_power_w.to_bits(), 400_000.0_f64.to_bits());
        assert_relative_eq!(
            cluster.station_oaspl_db,
            expected_oaspl_db,
            epsilon = 1.0e-10
        );
        assert!(
            cluster.station_oaspl_db > center.station_oaspl_db + same_location_gain_db + 0.1,
            "offset engine should use geometric range/directivity, not same-location scalar gain"
        );
    }

    #[test]
    fn point_mass_acoustics_cluster_interaction_matches_coherence_oracle() {
        fn scenario_toml(cluster_interaction: &str) -> String {
            COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
                "[telemetry]",
                &format!(
                    r#"[acoustics.dsm]
station_id = "station-a"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
receiver_azimuth_deg = 0.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001
{cluster_interaction}

[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 2.0

[[acoustics.dsm.source_line]]
axial_m = 20.0
weight = 1.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#
                ),
            )
        }

        fn run_dsm(cluster_interaction: &str) -> crate::acoustics::AcousticsDsmRunReport {
            let scenario = Scenario::from_toml_str(&scenario_toml(cluster_interaction))
                .expect("scenario parses");
            crate::run(&scenario)
                .expect("scenario runs")
                .acoustics
                .expect("acoustics report")
                .dsm
                .expect("dsm report")
        }

        let single_engine = run_dsm("");
        let cluster = run_dsm(
            r#"cluster_correlation_coefficient = 1.0
cluster_shielding_loss_db = 1.5

[[acoustics.dsm.cluster_engines]]
id = "engine-a"
offset_x_m = 0.0
offset_y_m = 0.0

[[acoustics.dsm.cluster_engines]]
id = "engine-b"
offset_x_m = 0.0
offset_y_m = 0.0"#,
        );
        let expected_delta_db = 20.0 * 2.0_f64.log10() - 1.5;

        assert_eq!(cluster.cluster_engine_count, 2);
        assert_eq!(cluster.source_count, 6);
        assert_eq!(
            cluster.cluster_correlation_coefficient.to_bits(),
            1.0_f64.to_bits()
        );
        assert_eq!(
            cluster.cluster_shielding_loss_db.to_bits(),
            1.5_f64.to_bits()
        );
        assert_relative_eq!(
            cluster.station_oaspl_db - single_engine.station_oaspl_db,
            expected_delta_db,
            epsilon = 1.0e-12
        );
        assert!(cluster.station_oaspl_db > single_engine.station_oaspl_db);
    }

    #[test]
    fn point_mass_acoustics_pad_path_length_extra_matches_spreading_oracle() {
        fn scenario_toml(path_extra_m: f64) -> String {
            COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
                "[telemetry]",
                &format!(
                    r#"[acoustics.dsm]
station_id = "station-a"
receiver_axial_m = 0.0
receiver_radial_m = 10.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.0
pad_path_length_extra_m = {path_extra_m:.1}

[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 10.0

[telemetry]"#
                ),
            )
        }

        fn run_dsm(path_extra_m: f64) -> crate::acoustics::AcousticsDsmRunReport {
            let scenario =
                Scenario::from_toml_str(&scenario_toml(path_extra_m)).expect("scenario parses");
            crate::run(&scenario)
                .expect("scenario runs")
                .acoustics
                .expect("acoustics report")
                .dsm
                .expect("dsm report")
        }

        let baseline = run_dsm(0.0);
        let pad_path = run_dsm(5.0);
        let expected_delta_db = -20.0 * (15.0_f64 / 10.0).log10();

        assert_eq!(
            pad_path.pad_path_length_extra_m.to_bits(),
            5.0_f64.to_bits()
        );
        assert_eq!(pad_path.source_count, 1);
        assert_relative_eq!(
            pad_path.station_oaspl_db - baseline.station_oaspl_db,
            expected_delta_db,
            epsilon = 1.0e-12
        );
        assert!(pad_path.station_oaspl_db < baseline.station_oaspl_db);
    }

    #[test]
    fn point_mass_acoustics_pad_deflection_matches_spreading_oracle() {
        fn scenario_toml(deflection: &str) -> String {
            COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
                "[telemetry]",
                &format!(
                    r#"[acoustics.dsm]
station_id = "station-a"
receiver_axial_m = 0.0
receiver_radial_m = 20.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.0
{deflection}

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 1.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 10.0

[telemetry]"#
                ),
            )
        }

        fn run_dsm(deflection: &str) -> crate::acoustics::AcousticsDsmRunReport {
            let scenario =
                Scenario::from_toml_str(&scenario_toml(deflection)).expect("scenario parses");
            crate::run(&scenario)
                .expect("scenario runs")
                .acoustics
                .expect("acoustics report")
                .dsm
                .expect("dsm report")
        }

        let straight = run_dsm("");
        let deflected =
            run_dsm("pad_deflection_origin_axial_m = 0.0\npad_deflection_angle_deg = 90.0");
        let straight_distance_m = 10.0_f64.hypot(20.0);
        let deflected_distance_m = 10.0_f64;
        let expected_delta_db = 20.0 * (straight_distance_m / deflected_distance_m).log10();

        assert_eq!(deflected.source_count, 1);
        assert_eq!(
            deflected.pad_deflection_origin_axial_m.map(f64::to_bits),
            Some(0.0_f64.to_bits())
        );
        assert_eq!(
            deflected.pad_deflection_angle_deg.map(f64::to_bits),
            Some(90.0_f64.to_bits())
        );
        assert_relative_eq!(
            deflected.station_oaspl_db - straight.station_oaspl_db,
            expected_delta_db,
            epsilon = 1.0e-12
        );
        assert!(deflected.station_oaspl_db > straight.station_oaspl_db);
    }

    #[test]
    fn point_mass_acoustics_deck_file_report_carries_provenance() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.dsm]
station_id = "station-a"
deck_file = "data/acoustics/dsm-source-directivity-v1.toml"
deck_file_sha256 = "169a6c35e513aafd8f111e6ba7e0b8d707cd0f1c8b1904eabc056d5d7536fe00"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("deck-backed acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("deck-backed acoustics scenario runs");
        let dsm = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.dsm.as_ref())
            .expect("dsm report emitted");

        assert_eq!(dsm.source_count, 3);
        assert_eq!(
            dsm.deck_file_sha256_hex.as_deref(),
            Some("169a6c35e513aafd8f111e6ba7e0b8d707cd0f1c8b1904eabc056d5d7536fe00")
        );
        assert_eq!(
            dsm.deck_dataset_id.as_deref(),
            Some("openbmp.acoustics.dsm_source_directivity.v1")
        );
        assert_eq!(dsm.deck_source_class.as_deref(), Some("synthetic-openbmp"));
        assert!(
            (first_row_f64(&outcome, "acoustics.dsm.station_oaspl_db") - dsm.station_oaspl_db)
                .abs()
                <= 1.0e-12
        );
        assert!(
            (dsm.station_oaspl_db - 132.214_258_788_974_58).abs() > 1.0e-3,
            "deck-backed directivity/allocation should change the synthetic omni fixture"
        );
    }

    #[test]
    fn point_mass_acoustics_absorption_deck_report_carries_provenance() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.dsm]
station_id = "station-a"
absorption_deck_file = "data/acoustics/dsm-air-absorption-v1.toml"
absorption_deck_file_sha256 = "cda1136c9a9f935ac576fbe06671f00d8e0d1ed56bbb5d9c2cd42dc34847af48"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001

[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 2.0

[[acoustics.dsm.source_line]]
axial_m = 20.0
weight = 1.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("absorption-deck acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("absorption-deck acoustics scenario runs");
        let dsm = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.dsm.as_ref())
            .expect("dsm report emitted");

        assert_eq!(dsm.source_count, 3);
        assert_eq!(
            dsm.absorption_deck_file_sha256_hex.as_deref(),
            Some("cda1136c9a9f935ac576fbe06671f00d8e0d1ed56bbb5d9c2cd42dc34847af48")
        );
        assert_eq!(
            dsm.absorption_deck_dataset_id.as_deref(),
            Some("openbmp.acoustics.dsm_air_absorption.v1")
        );
        assert_eq!(
            dsm.absorption_deck_source_class.as_deref(),
            Some("synthetic-openbmp")
        );
        assert!(
            (first_row_f64(&outcome, "acoustics.dsm.station_oaspl_db") - dsm.station_oaspl_db)
                .abs()
                <= 1.0e-12
        );
        assert!(
            (dsm.station_oaspl_db - 132.214_258_788_974_58).abs() > 1.0e-3,
            "band absorption deck should change the scalar-absorption fixture"
        );
    }

    #[test]
    fn point_mass_acoustics_absorption_profile_selects_atmosphere() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.dsm]
station_id = "station-a"
absorption_deck_file = "data/acoustics/dsm-air-absorption-atmospheres-v1.toml"
absorption_deck_file_sha256 = "c4f425557a8a99204b1edb05383abf7a07abcf846345679544ac2894ba841fca"
absorption_profile_id = "humid_pad_303k_70rh"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001

[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 2.0

[[acoustics.dsm.source_line]]
axial_m = 20.0
weight = 1.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("atmosphere-profile absorption scenario parses");
        let outcome = crate::run(&scenario).expect("atmosphere-profile absorption scenario runs");
        let dsm = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.dsm.as_ref())
            .expect("dsm report emitted");

        assert_eq!(
            dsm.absorption_deck_file_sha256_hex.as_deref(),
            Some("c4f425557a8a99204b1edb05383abf7a07abcf846345679544ac2894ba841fca")
        );
        assert_eq!(
            dsm.absorption_deck_dataset_id.as_deref(),
            Some("openbmp.acoustics.dsm_air_absorption_atmospheres.v1")
        );
        assert_eq!(
            dsm.absorption_deck_source_class.as_deref(),
            Some("synthetic-openbmp-atmosphere-profiles")
        );
        assert_eq!(
            dsm.absorption_profile_id.as_deref(),
            Some("humid_pad_303k_70rh")
        );
        assert!(
            (first_row_f64(&outcome, "acoustics.dsm.station_oaspl_db") - dsm.station_oaspl_db)
                .abs()
                <= 1.0e-12
        );
        assert!(
            (dsm.station_oaspl_db - 132.214_258_788_974_58).abs() > 1.0e-3,
            "selected atmosphere profile should change the scalar-absorption fixture"
        );
    }

    #[test]
    fn point_mass_acoustics_suppression_deck_report_carries_provenance() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.dsm]
station_id = "station-a"
suppression_deck_file = "data/acoustics/pad-suppression-v1.toml"
suppression_deck_file_sha256 = "4888aeb8dde08333638d7d1127d864ea544a1a2ab78e4eabd95ad3d0b42cf932"
suppression_class = "water_nominal"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001

[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 2.0

[[acoustics.dsm.source_line]]
axial_m = 20.0
weight = 1.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("suppression-deck acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("suppression-deck acoustics scenario runs");
        let dsm = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.dsm.as_ref())
            .expect("dsm report emitted");

        assert_eq!(dsm.source_count, 3);
        assert_eq!(
            dsm.suppression_deck_file_sha256_hex.as_deref(),
            Some("4888aeb8dde08333638d7d1127d864ea544a1a2ab78e4eabd95ad3d0b42cf932")
        );
        assert_eq!(
            dsm.suppression_deck_dataset_id.as_deref(),
            Some("openbmp.acoustics.pad_suppression.v1")
        );
        assert_eq!(
            dsm.suppression_deck_source_class.as_deref(),
            Some("synthetic-openbmp")
        );
        assert_eq!(dsm.suppression_class_id.as_deref(), Some("water_nominal"));
        assert_eq!(
            dsm.suppression_uncertainty_db.map(f64::to_bits),
            Some(3.0_f64.to_bits())
        );
        assert!(
            (first_row_f64(&outcome, "acoustics.dsm.station_oaspl_db") - dsm.station_oaspl_db)
                .abs()
                <= 1.0e-12
        );
        assert!(
            dsm.station_oaspl_db < 132.214_258_788_974_58,
            "water suppression should reduce the unsuppressed omni fixture OASPL"
        );
    }

    #[test]
    fn point_mass_acoustics_cluster_interaction_deck_report_carries_provenance() {
        fn scenario_toml(cluster_interaction: &str) -> String {
            COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
                "[telemetry]",
                &format!(
                    r#"[acoustics.dsm]
station_id = "station-a"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
receiver_azimuth_deg = 0.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001
{cluster_interaction}

[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 2.0

[[acoustics.dsm.source_line]]
axial_m = 20.0
weight = 1.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#
                ),
            )
        }

        fn run_dsm(cluster_interaction: &str) -> crate::acoustics::AcousticsDsmRunReport {
            let scenario = Scenario::from_toml_str_with_source_dir(
                &scenario_toml(cluster_interaction),
                Some(workspace_root()),
            )
            .expect("scenario parses");
            crate::run(&scenario)
                .expect("scenario runs")
                .acoustics
                .expect("acoustics report")
                .dsm
                .expect("dsm report")
        }

        let single_engine = run_dsm("");
        let cluster = run_dsm(
            r#"cluster_interaction_deck_file = "data/acoustics/pad-cluster-interaction-v1.toml"
cluster_interaction_deck_file_sha256 = "301aaf420d9231834a89962184b8e57e9a3117a704f556f61a4eb1ab0c836b37"
cluster_interaction_class = "coherent_shielded"

[[acoustics.dsm.cluster_engines]]
id = "engine-a"
offset_x_m = 0.0
offset_y_m = 0.0

[[acoustics.dsm.cluster_engines]]
id = "engine-b"
offset_x_m = 0.0
offset_y_m = 0.0"#,
        );
        let expected_delta_db = 20.0 * 2.0_f64.log10() - 1.5;

        assert_eq!(cluster.cluster_engine_count, 2);
        assert_eq!(cluster.source_count, 6);
        assert_eq!(
            cluster.cluster_correlation_coefficient.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            cluster.cluster_shielding_loss_db.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            cluster.cluster_interaction_deck_file_sha256_hex.as_deref(),
            Some("301aaf420d9231834a89962184b8e57e9a3117a704f556f61a4eb1ab0c836b37")
        );
        assert_eq!(
            cluster.cluster_interaction_deck_dataset_id.as_deref(),
            Some("openbmp.acoustics.pad_cluster_interaction.v1")
        );
        assert_eq!(
            cluster.cluster_interaction_deck_source_class.as_deref(),
            Some("synthetic-openbmp")
        );
        assert_eq!(
            cluster.cluster_interaction_class_id.as_deref(),
            Some("coherent_shielded")
        );
        assert_eq!(
            cluster.cluster_interaction_uncertainty_db.map(f64::to_bits),
            Some(2.0_f64.to_bits())
        );
        assert_relative_eq!(
            cluster.station_oaspl_db - single_engine.station_oaspl_db,
            expected_delta_db,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn point_mass_acoustics_apparent_source_curve_report_carries_provenance() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.dsm]
station_id = "station-a"
apparent_source_curve_file = "data/acoustics/sp8072-figure14-apparent-source-v1.toml"
apparent_source_curve_file_sha256 = "5c768b2ce628c07ad2cc8e2a1a6df99e893318ce79757c0675e77bc79f1b6000"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("apparent-source acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("apparent-source acoustics scenario runs");
        let dsm = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.dsm.as_ref())
            .expect("dsm report emitted");

        assert_eq!(
            dsm.source_count,
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len()
        );
        assert_eq!(
            dsm.apparent_source_curve_file_sha256_hex.as_deref(),
            Some("5c768b2ce628c07ad2cc8e2a1a6df99e893318ce79757c0675e77bc79f1b6000")
        );
        assert_eq!(
            dsm.apparent_source_curve_dataset_id.as_deref(),
            Some("openbmp.acoustics.sp8072_figure14_apparent_source.v1")
        );
        assert_eq!(
            dsm.apparent_source_curve_figure_id.as_deref(),
            Some("figure_14")
        );
        assert_eq!(
            first_row_i64(&outcome, "acoustics.dsm.source_count"),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() as i64
        );
        assert!(
            (first_row_f64(&outcome, "acoustics.dsm.station_oaspl_db") - dsm.station_oaspl_db)
                .abs()
                <= 1.0e-12
        );
        assert!(dsm.station_oaspl_db.is_finite());
    }

    #[test]
    fn point_mass_acoustics_source_curve_report_carries_provenance() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.dsm]
station_id = "station-a"
source_curve_file = "data/acoustics/sp8072-figure12-source-power-v1.toml"
source_curve_file_sha256 = "d199bebbb4bcd217f03f7a8ad717f875aa33419f7dbb2532e3b04649fb848b62"
directivity_curve_file = "data/acoustics/sp8072-figure10-directivity-v1.toml"
directivity_curve_file_sha256 = "b374017177dae2cb5028ae7d096d253e9fd300665d05f33dd82d1ce2aaad7894"
spectrum_curve_file = "data/acoustics/sp8072-figure5-spectrum-v1.toml"
spectrum_curve_file_sha256 = "0d5c9ed47b94e835562f13e71d3ace4207b5a56b0991bfdbf0b9ebed11bb8b19"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
exit_mach = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("source-curve acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("source-curve acoustics scenario runs");
        let dsm = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.dsm.as_ref())
            .expect("dsm report emitted");

        assert_eq!(dsm.source_count, 15);
        assert_eq!(
            dsm.source_curve_file_sha256_hex.as_deref(),
            Some("d199bebbb4bcd217f03f7a8ad717f875aa33419f7dbb2532e3b04649fb848b62")
        );
        assert_eq!(
            dsm.source_curve_dataset_id.as_deref(),
            Some("openbmp.acoustics.sp8072_figure12_source_power.v1")
        );
        assert_eq!(dsm.source_curve_figure_id.as_deref(), Some("figure_12"));
        assert_eq!(
            dsm.directivity_curve_file_sha256_hex.as_deref(),
            Some("b374017177dae2cb5028ae7d096d253e9fd300665d05f33dd82d1ce2aaad7894")
        );
        assert_eq!(
            dsm.directivity_curve_dataset_id.as_deref(),
            Some("openbmp.acoustics.sp8072_figure10_directivity.v1")
        );
        assert_eq!(
            dsm.directivity_curve_figure_id.as_deref(),
            Some("figure_10")
        );
        assert_eq!(
            dsm.spectrum_curve_file_sha256_hex.as_deref(),
            Some("0d5c9ed47b94e835562f13e71d3ace4207b5a56b0991bfdbf0b9ebed11bb8b19")
        );
        assert_eq!(
            dsm.spectrum_curve_dataset_id.as_deref(),
            Some("openbmp.acoustics.sp8072_figure5_spectrum.v1")
        );
        assert_eq!(dsm.spectrum_curve_figure_id.as_deref(), Some("figure_5"));
        assert!(
            (dsm.source_curve_core_length_m.unwrap() - 6.570_18).abs() <= 1.0e-12,
            "source-curve path should derive SP-8072 Figure 11 core length"
        );
        assert_eq!(first_row_i64(&outcome, "acoustics.dsm.source_count"), 15);
        assert!(
            (first_row_f64(&outcome, "acoustics.dsm.station_oaspl_db") - dsm.station_oaspl_db)
                .abs()
                <= 1.0e-12
        );
        assert!(dsm.station_oaspl_db.is_finite());
    }

    #[test]
    fn point_mass_acoustics_sp8072_curve_loaders_reject_manifest_mismatch() {
        struct CurveCase {
            scenario_field: &'static str,
            resolved_key: &'static str,
            path: &'static str,
            contents: &'static str,
            extra_toml: &'static str,
            needs_inline_sources: bool,
        }

        const INLINE_SOURCES: &str = r#"
[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 2.0

[[acoustics.dsm.source_line]]
axial_m = 20.0
weight = 1.0
"#;
        let cases = [
            CurveCase {
                scenario_field: "source_curve_file",
                resolved_key: "acoustics.dsm.source_curve_file",
                path: "data/acoustics/sp8072-figure12-source-power-v1.toml",
                contents: include_str!(
                    "../../../data/acoustics/sp8072-figure12-source-power-v1.toml"
                ),
                extra_toml: "exit_mach = 1.0",
                needs_inline_sources: false,
            },
            CurveCase {
                scenario_field: "apparent_source_curve_file",
                resolved_key: "acoustics.dsm.apparent_source_curve_file",
                path: "data/acoustics/sp8072-figure14-apparent-source-v1.toml",
                contents: include_str!(
                    "../../../data/acoustics/sp8072-figure14-apparent-source-v1.toml"
                ),
                extra_toml: "",
                needs_inline_sources: false,
            },
            CurveCase {
                scenario_field: "axial_spectrum_curve_file",
                resolved_key: "acoustics.dsm.axial_spectrum_curve_file",
                path: "data/acoustics/sp8072-figure13-axial-spectrum-v1.toml",
                contents: include_str!(
                    "../../../data/acoustics/sp8072-figure13-axial-spectrum-v1.toml"
                ),
                extra_toml: "",
                needs_inline_sources: true,
            },
            CurveCase {
                scenario_field: "spectrum_curve_file",
                resolved_key: "acoustics.dsm.spectrum_curve_file",
                path: "data/acoustics/sp8072-figure5-spectrum-v1.toml",
                contents: include_str!("../../../data/acoustics/sp8072-figure5-spectrum-v1.toml"),
                extra_toml: "",
                needs_inline_sources: true,
            },
            CurveCase {
                scenario_field: "directivity_curve_file",
                resolved_key: "acoustics.dsm.directivity_curve_file",
                path: "data/acoustics/sp8072-figure10-directivity-v1.toml",
                contents: include_str!(
                    "../../../data/acoustics/sp8072-figure10-directivity-v1.toml"
                ),
                extra_toml: "",
                needs_inline_sources: true,
            },
        ];

        for case in cases {
            let bad_curve = case.contents.replace(
                "source_manifest_id = \"openbmp.acoustics.sp8072_source.v1\"",
                "source_manifest_id = \"openbmp.acoustics.sp8072_source.bad\"",
            );
            assert_ne!(
                bad_curve, case.contents,
                "{} fixture should contain a source manifest id",
                case.scenario_field
            );
            let resolved =
                openbmp_scenario::ResolvedFile::from_bytes(case.path, bad_curve.into_bytes());
            let inline_sources = if case.needs_inline_sources {
                INLINE_SOURCES
            } else {
                ""
            };
            let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
                "[telemetry]",
                &format!(
                    r#"[acoustics.dsm]
station_id = "station-a"
{scenario_field} = "{path}"
{scenario_field}_sha256 = "{sha256}"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
{extra_toml}
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001
{inline_sources}
[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#,
                    scenario_field = case.scenario_field,
                    path = case.path,
                    sha256 = resolved.sha256_hex.as_str(),
                    extra_toml = case.extra_toml,
                    inline_sources = inline_sources,
                ),
            );
            let scenario =
                Scenario::from_toml_str(&scenario_toml).expect("manifest-mismatch scenario parses");
            let resolved_files = BTreeMap::from([(case.resolved_key.to_owned(), resolved)]);
            let err = crate::acoustics::evaluate_document(&scenario.document, &resolved_files)
                .expect_err("curve manifest mismatch must fail runner evaluation");

            match err {
                RunnerError::UnsupportedScenario { what } => {
                    assert!(
                        what.contains("source_manifest_id"),
                        "{} emitted unexpected error: {what}",
                        case.scenario_field
                    );
                }
                other => panic!(
                    "{} emitted unexpected error: {other:?}",
                    case.scenario_field
                ),
            }
        }
    }

    #[test]
    fn point_mass_acoustics_axial_spectrum_curve_report_carries_provenance() {
        let scenario_toml = COMM_OFF_DEFAULT_POINT_MASS_SCENARIO.replace(
            "[telemetry]",
            r#"[acoustics.dsm]
station_id = "station-a"
source_curve_file = "data/acoustics/sp8072-figure12-source-power-v1.toml"
source_curve_file_sha256 = "d199bebbb4bcd217f03f7a8ad717f875aa33419f7dbb2532e3b04649fb848b62"
axial_spectrum_curve_file = "data/acoustics/sp8072-figure13-axial-spectrum-v1.toml"
axial_spectrum_curve_file_sha256 = "41d90c99e53afe77d1d4503d7bf17df627341d4ac26323ec2a32f601faa318da"
spectrum_curve_file = "data/acoustics/sp8072-figure5-spectrum-v1.toml"
spectrum_curve_file_sha256 = "0d5c9ed47b94e835562f13e71d3ace4207b5a56b0991bfdbf0b9ebed11bb8b19"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
mass_flow_kg_s = 10.0
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
exit_mach = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0

[telemetry]"#,
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&scenario_toml, Some(workspace_root()))
                .expect("axial-spectrum acoustics scenario parses");
        let outcome = crate::run(&scenario).expect("axial-spectrum acoustics scenario runs");
        let dsm = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.dsm.as_ref())
            .expect("dsm report emitted");

        assert_eq!(dsm.source_count, 15);
        assert_eq!(
            dsm.axial_spectrum_curve_file_sha256_hex.as_deref(),
            Some("41d90c99e53afe77d1d4503d7bf17df627341d4ac26323ec2a32f601faa318da")
        );
        assert_eq!(
            dsm.axial_spectrum_curve_dataset_id.as_deref(),
            Some("openbmp.acoustics.sp8072_figure13_axial_spectrum.v1")
        );
        assert_eq!(
            dsm.axial_spectrum_curve_figure_id.as_deref(),
            Some("figure_13")
        );
        assert_eq!(
            dsm.source_curve_file_sha256_hex.as_deref(),
            Some("d199bebbb4bcd217f03f7a8ad717f875aa33419f7dbb2532e3b04649fb848b62")
        );
        assert_eq!(
            dsm.spectrum_curve_file_sha256_hex.as_deref(),
            Some("0d5c9ed47b94e835562f13e71d3ace4207b5a56b0991bfdbf0b9ebed11bb8b19")
        );
        assert_eq!(
            first_row_i64(&outcome, "acoustics.dsm.source_count"),
            dsm.source_count as i64
        );
        assert!(
            (first_row_f64(&outcome, "acoustics.dsm.station_oaspl_db") - dsm.station_oaspl_db)
                .abs()
                <= 1.0e-12
        );
        assert!(dsm.station_oaspl_db.is_finite());
    }

    #[test]
    fn point_mass_comm_report_contains_byte_stable_pass_table() {
        let scenario = Scenario::from_toml_str_with_source_dir(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-comm-pass-table-test"
description = "Synthetic comm pass table regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 7

[vehicle]
kind = "point_mass"
initial_position_eci_m = [6379137.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-comm-pass-table-test"

[[vehicle.assembly.bodies]]
id = "stage"
geometry = { kind = "cylinder", length_m = 1.0, diameter_m = 1.0 }
dry_mass_kg = 10.0
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
output.csv = "out/point-mass-comm-pass-table-test.csv"

[comm]
[[comm.sites]]
id = "equator-zero"
latitude_deg = 0.0
longitude_deg = 0.0
altitude_m = 0.0
min_elevation_deg = 5.0

[[comm.antennas]]
id = "s-band-low-gain"
gain_deck = "data/comm/antenna-link-budget-v1.toml"
body_mask_deck = "data/comm/antenna-link-budget-v1.toml"

[[comm.links]]
id = "s-band-equator"
site_id = "equator-zero"
antenna_id = "s-band-low-gain"
eirp_dbw = -40.0
receiver_g_over_t_db_k = 0.0
frequency_hz = 2.0e9
bit_rate_bps = 1000.0
required_eb_n0_db = 3.0
atmospheric_loss_db = 1.0
atmospheric_loss_deck = "data/comm/attenuation-loss-v1.toml"
rain_loss_db = 0.5
rain_loss_deck = "data/comm/attenuation-loss-v1.toml"
pointing_loss_db = 0.25
polarization_loss_db = 0.1
implementation_loss_db = 0.0
fer_curve_deck = "data/comm/link-budget-fer-v1.toml"
packet_processing_delay_s = 0.02
packet_error_action = { kind = "bit_flip", mask = 165 }
packet_loss_rate_gate = { packet_count = 64, alpha = 0.001 }
data_loss_timeout_s = 1.0

[validation]
require_finite_state = true
require_monotonic_time = true
"#,
            Some(workspace_root()),
        )
        .unwrap();

        let outcome = crate::run(&scenario).unwrap();
        let report = outcome.comm.as_ref().expect("comm report");

        assert_eq!(
            std::str::from_utf8(&report.pass_table_csv).unwrap(),
            "site_id,aos_s,los_s,max_elevation_rad\nequator-zero,0.00000000000000000e0,1.00000000000000006e-1,1.57079632679489656e0\n"
        );
        assert_abs_diff_eq!(
            first_row_f64(&outcome, "comm.equator-zero.slant_range_m"),
            1000.0,
            epsilon = 1.0e-9
        );
        assert_abs_diff_eq!(
            first_row_f64(&outcome, "comm.equator-zero.elevation_rad"),
            core::f64::consts::FRAC_PI_2,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            first_row_f64(&outcome, "comm.equator-zero.mask_elevation_rad"),
            5.0_f64.to_radians(),
            epsilon = 1.0e-15
        );
        assert!(first_row_bool(&outcome, "comm.equator-zero.visible"));
        assert_eq!(report.antenna_decks.len(), 1);
        let antenna_deck = &report.antenna_decks[0];
        assert_eq!(antenna_deck.antenna_id, "s-band-low-gain");
        assert_eq!(antenna_deck.gain_sample_count, 3);
        assert_eq!(antenna_deck.body_mask_sample_count, 3);
        assert_eq!(
            antenna_deck.gain_deck_file_sha256_hex,
            antenna_deck.body_mask_deck_file_sha256_hex
        );
        let body_mask_digest = antenna_deck
            .body_mask_derived_sha256_hex
            .as_ref()
            .expect("derived body-mask digest");
        assert_eq!(body_mask_digest.len(), 64);
        assert!(
            body_mask_digest
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit())
        );
        assert_eq!(report.links.len(), 1);
        let link = &report.links[0];
        assert_eq!(link.link_id, "s-band-equator");
        assert_eq!(link.site_id, "equator-zero");
        assert_eq!(link.antenna_id, "s-band-low-gain");
        assert_eq!(link.fer_curve_code_id, "synthetic-log-linear");
        assert_eq!(link.fer_curve_sample_count, 3);
        assert_eq!(link.fer_curve_deck_file_sha256_hex.len(), 64);
        assert_eq!(link.atmospheric_loss_sample_count, 3);
        assert_eq!(link.rain_loss_sample_count, 3);
        assert_eq!(link.packet_processing_delay_s.to_bits(), 0.02_f64.to_bits());
        assert_eq!(link.packet_error_action, "bit_flip");
        assert_eq!(link.packet_error_bit_flip_mask, Some(165));
        let loss_gate = link
            .packet_loss_rate_gate
            .as_ref()
            .expect("packet loss-rate gate report");
        assert_eq!(loss_gate.packet_count, 64);
        assert_eq!(loss_gate.alpha.to_bits(), 0.001_f64.to_bits());
        assert_eq!(loss_gate.sample_step, outcome.final_step);
        assert!(loss_gate.downlink.passed);
        assert!(loss_gate.uplink.passed);
        assert_eq!(loss_gate.downlink.direction, "downlink");
        assert_eq!(loss_gate.uplink.direction, "uplink");
        let data_loss = link
            .data_loss_timeout
            .as_ref()
            .expect("data-loss timeout report");
        assert_eq!(data_loss.timeout_s.to_bits(), 1.0_f64.to_bits());
        assert_eq!(data_loss.verdict, "hold");
        assert!(!data_loss.triggered);
        assert_eq!(data_loss.first_trigger_step, None);
        assert!(data_loss.delivered_sample_count > 0);
        assert_eq!(
            link.atmospheric_loss_deck_file_sha256_hex
                .as_deref()
                .map(str::len),
            Some(64)
        );
        assert_eq!(
            link.rain_loss_deck_file_sha256_hex.as_deref().map(str::len),
            Some(64)
        );
        assert_abs_diff_eq!(
            first_row_f64(&outcome, "comm.link.s-band-equator.free_space_loss_db"),
            98.468_383_135_163,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            first_row_f64(&outcome, "comm.link.s-band-equator.antenna_gain_dbi"),
            3.0,
            epsilon = 1.0e-12
        );
        assert!(!first_row_bool(
            &outcome,
            "comm.link.s-band-equator.body_blocked"
        ));
        assert_abs_diff_eq!(
            first_row_f64(&outcome, "comm.link.s-band-equator.c_n0_dbhz"),
            91.030_784_038_054_7,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            first_row_f64(&outcome, "comm.link.s-band-equator.eb_n0_db"),
            61.030_784_038_054_7,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            first_row_f64(&outcome, "comm.link.s-band-equator.margin_db"),
            58.030_784_038_054_7,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            first_row_f64(&outcome, "comm.link.s-band-equator.fer"),
            6.220_756_362_024_405e-5,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            loss_gate.downlink.expected_error_rate,
            6.220_756_362_024_405e-5,
            epsilon = 1.0e-15
        );
        assert!(first_row_bool(&outcome, "comm.link.s-band-equator.visible"));
        assert!(!first_row_bool(
            &outcome,
            "comm.link.s-band-equator.blackout"
        ));
    }

    #[test]
    fn point_mass_comm_data_loss_timeout_latches_on_no_link() {
        let scenario = Scenario::from_toml_str_with_source_dir(
            r#"
openbmp.scenario = 3

[meta]
name = "point-mass-comm-data-loss-timeout-test"
description = "Synthetic comm data-loss timeout regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.2
dt_s = 0.1
seed = 7

[vehicle]
kind = "point_mass"
initial_position_eci_m = [6379137.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-comm-data-loss-timeout-test"

[[vehicle.assembly.bodies]]
id = "stage"
geometry = { kind = "cylinder", length_m = 1.0, diameter_m = 1.0 }
dry_mass_kg = 10.0
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
output.csv = "out/point-mass-comm-data-loss-timeout-test.csv"

[comm]
[[comm.sites]]
id = "opposite-side"
latitude_deg = 0.0
longitude_deg = 180.0
altitude_m = 0.0
min_elevation_deg = 5.0

[[comm.antennas]]
id = "s-band-low-gain"
gain_deck = "data/comm/antenna-link-budget-v1.toml"
body_mask_deck = "data/comm/antenna-link-budget-v1.toml"

[[comm.links]]
id = "s-band-opposite"
site_id = "opposite-side"
antenna_id = "s-band-low-gain"
eirp_dbw = -40.0
receiver_g_over_t_db_k = 0.0
frequency_hz = 2.0e9
bit_rate_bps = 1000.0
required_eb_n0_db = 3.0
atmospheric_loss_db = 1.0
atmospheric_loss_deck = "data/comm/attenuation-loss-v1.toml"
rain_loss_db = 0.5
rain_loss_deck = "data/comm/attenuation-loss-v1.toml"
pointing_loss_db = 0.25
polarization_loss_db = 0.1
implementation_loss_db = 0.0
fer_curve_deck = "data/comm/link-budget-fer-v1.toml"
data_loss_timeout_s = 0.05

[validation]
require_finite_state = true
require_monotonic_time = true
"#,
            Some(workspace_root()),
        )
        .unwrap();

        let outcome = crate::run(&scenario).unwrap();
        let report = outcome.comm.as_ref().expect("comm report");
        let link = &report.links[0];
        let data_loss = link
            .data_loss_timeout
            .as_ref()
            .expect("data-loss timeout report");

        assert!(data_loss.triggered);
        assert_eq!(data_loss.verdict, "terminate");
        assert_eq!(data_loss.first_loss_step, Some(0));
        assert_eq!(data_loss.first_loss_time_s, Some(0.0));
        assert_eq!(data_loss.first_trigger_step, Some(1));
        assert_eq!(data_loss.first_trigger_time_s, Some(0.1));
        assert_eq!(data_loss.delivered_sample_count, 0);
        assert!(data_loss.lost_sample_count >= 2);
        assert!(data_loss.max_loss_gap_s > data_loss.timeout_s);
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
        assert_eq!(
            first_row_f64(&outcome, "contact.gap_m").to_bits(),
            (-0.01_f64).to_bits()
        );
        assert_eq!(
            first_row_f64(&outcome, "contact.penetration_m").to_bits(),
            0.01_f64.to_bits()
        );
        assert_eq!(
            first_row_f64(&outcome, "contact.normal_velocity_m_s").to_bits(),
            0.0_f64.to_bits()
        );
        assert!((first_row_f64(&outcome, "contact.normal_force_n") - 20.0).abs() <= 1.0e-12);

        let contact = outcome
            .contact
            .as_ref()
            .expect("contact report should be present");
        assert_eq!(contact.samples as usize, outcome.table.rows().len());
        assert_eq!(contact.max_penetration_m.to_bits(), 0.01_f64.to_bits());
        assert!((contact.max_normal_force_n - 20.0).abs() <= 1.0e-12);
        assert_eq!(
            contact.outcome,
            crate::contact::ContactOutcomeKind::Unsettled
        );
    }

    #[test]
    fn point_mass_contact_anchored_stiction_slides_with_kinetic_force() {
        let scenario_toml = CONTACT_POINT_MASS_SCENARIO
            .replace(
                "initial_velocity_eci_m_s = [0.0, 0.0, 0.0]",
                "initial_velocity_eci_m_s = [1.0, 0.0, 0.0]",
            )
            .replace(
                "friction_coefficient = 0.0",
                "friction_law = \"anchored_stiction\"\nstatic_friction_coefficient = 0.5\nkinetic_friction_coefficient = 0.25\ntangential_stiffness_n_m = 1000.0\ntangential_damping_n_s_m = 0.0\nrestick_speed_m_s = 0.01",
            );
        let scenario = Scenario::from_toml_str(&scenario_toml).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("contact run succeeds");

        assert!((first_row_f64(&outcome, "force.contact.x_n") + 5.0).abs() <= 1.0e-12);
        assert!((first_row_f64(&outcome, "force.contact.z_n") - 20.0).abs() <= 1.0e-12);
        let contact = outcome
            .contact
            .as_ref()
            .expect("contact report should be present");
        assert!(
            contact.energy.dissipated_energy_j > 0.0,
            "anchored stiction sliding should dissipate energy: {:?}",
            contact.energy
        );
    }

    #[test]
    fn point_mass_phase_disabled_contact_has_no_anchor_preload_or_work() {
        let scenario_toml = CONTACT_POINT_MASS_SCENARIO
            .replace("gravity_m_s2 = 9.80665", "gravity_m_s2 = 0.0")
            .replace(
                "initial_velocity_eci_m_s = [0.0, 0.0, 0.0]",
                "initial_velocity_eci_m_s = [0.1, 0.0, 0.0]",
            )
            .replace(
                "friction_coefficient = 0.0",
                "friction_law = \"anchored_stiction\"\nstatic_friction_coefficient = 0.5\nkinetic_friction_coefficient = 0.25\ntangential_stiffness_n_m = 1000.0\ntangential_damping_n_s_m = 0.0\nrestick_speed_m_s = 1.0",
            )
            .replace(
                "models = [\"gravity\", \"contact\"]",
                "models = [\"gravity\"]\n\n[[forces.phase_override]]\nphase = \"contact_enabled\"\nmodels = [\"gravity\", \"contact\"]",
            )
            + r#"

[mission]
initial_phase = "contact_disabled"

[[mission.phases]]
id = "contact_disabled"
label = "contact disabled"

[[mission.phases]]
id = "contact_enabled"
label = "contact enabled"

[[mission.events]]
id = "enable_contact"
trigger = { kind = "at_time", time_s = 0.002 }
action = { kind = "emit_telemetry_marker", tag = "enable_contact" }
once = true

[[mission.transitions]]
from = "contact_disabled"
to = "contact_enabled"
event = "enable_contact"
"#;
        let scenario = Scenario::from_toml_str(&scenario_toml).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("contact run succeeds");
        let normal_force = f64_column(&outcome, "force.contact.z_n");
        assert_abs_diff_eq!(normal_force[0], 0.0, epsilon = 0.0);
        assert_abs_diff_eq!(normal_force[1], 0.0, epsilon = 0.0);
        assert!((normal_force[2] - 20.0).abs() < 1.0e-12);
        let tangential_force = f64_column(&outcome, "force.contact.x_n");
        assert!(
            tangential_force[2].abs() <= 0.0,
            "first activation must not inherit an anchor loaded while the force was disabled"
        );
        assert_abs_diff_eq!(
            first_row_f64(&outcome, "contact.normal_force_n"),
            0.0,
            epsilon = 0.0
        );
        let report = outcome.contact.as_ref().expect("contact report");
        assert_abs_diff_eq!(report.energy.contact_work_on_vehicle_j, 0.0, epsilon = 0.0);
        assert_abs_diff_eq!(report.energy.dissipated_energy_j, 0.0, epsilon = 0.0);
    }

    #[test]
    fn point_mass_contact_anchored_stiction_reports_sticking_rest() {
        let scenario_toml = CONTACT_POINT_MASS_SCENARIO
            .replace("gravity_m_s2 = 9.80665", "gravity_m_s2 = 20.0")
            .replace(
                "friction_coefficient = 0.0",
                "friction_law = \"anchored_stiction\"\nstatic_friction_coefficient = 0.5\nkinetic_friction_coefficient = 0.25\ntangential_stiffness_n_m = 1000.0\ntangential_damping_n_s_m = 0.0\nrestick_speed_m_s = 0.01",
            );
        let scenario = Scenario::from_toml_str(&scenario_toml).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("contact run succeeds");

        let contact = outcome
            .contact
            .as_ref()
            .expect("contact report should be present");
        assert_eq!(contact.outcome, crate::contact::ContactOutcomeKind::Rest);
        assert!(contact.final_diagnostics.sticking);
        assert_eq!(
            contact.final_diagnostics.tangential_speed_m_s.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            contact.energy.contact_work_on_vehicle_j.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            contact.energy.dissipated_energy_j.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(contact.energy.closure_error_j.to_bits(), 0.0_f64.to_bits());
        assert!(
            f64_column(&outcome, "force.contact.z_n")
                .iter()
                .all(|value| (*value - 20.0).abs() <= 1.0e-12)
        );
    }

    #[test]
    fn point_mass_contact_report_classifies_no_contact() {
        let scenario_toml = CONTACT_POINT_MASS_SCENARIO
            .replace(
                "initial_position_eci_m = [0.0, 0.0, -0.01]",
                "initial_position_eci_m = [0.0, 0.0, 1.0]",
            )
            .replace("gravity_m_s2 = 9.80665", "gravity_m_s2 = 0.0");
        let scenario = Scenario::from_toml_str(&scenario_toml).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("contact run succeeds");

        let contact = outcome
            .contact
            .as_ref()
            .expect("contact report should be present");
        assert_eq!(
            contact.outcome,
            crate::contact::ContactOutcomeKind::NoContact
        );
        assert_eq!(contact.max_penetration_m.to_bits(), 0.0_f64.to_bits());
        assert_eq!(contact.max_normal_force_n.to_bits(), 0.0_f64.to_bits());
        assert!(
            f64_column(&outcome, "contact.penetration_m")
                .iter()
                .all(|value| value.to_bits() == 0.0_f64.to_bits())
        );
    }

    #[test]
    fn point_mass_contact_energy_audit_closes_for_balanced_static_penalty() {
        let scenario_toml =
            CONTACT_POINT_MASS_SCENARIO.replace("gravity_m_s2 = 9.80665", "gravity_m_s2 = 20.0");
        let scenario = Scenario::from_toml_str(&scenario_toml).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("contact run succeeds");

        let contact = outcome
            .contact
            .as_ref()
            .expect("contact report should be present");
        assert_eq!(contact.outcome, crate::contact::ContactOutcomeKind::Rest);
        assert!((contact.energy.initial_elastic_energy_j - 0.1).abs() <= 1.0e-15);
        assert!((contact.energy.final_elastic_energy_j - 0.1).abs() <= 1.0e-15);
        assert_eq!(
            contact.energy.contact_work_on_vehicle_j.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            contact.energy.dissipated_energy_j.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(contact.energy.closure_error_j.to_bits(), 0.0_f64.to_bits());
        assert_eq!(
            contact.energy.relative_closure_error.to_bits(),
            0.0_f64.to_bits()
        );
        assert!(
            f64_column(&outcome, "force.contact.z_n")
                .iter()
                .all(|value| (*value - 20.0).abs() <= 1.0e-12)
        );
    }

    #[test]
    fn point_mass_contact_substeps_drive_kernel_step_size() {
        let scenario_toml = CONTACT_POINT_MASS_SCENARIO.replace("substeps = 1", "substeps = 2");
        let scenario = Scenario::from_toml_str(&scenario_toml).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("contact run succeeds");

        assert_eq!(outcome.final_step, 4);
        assert_eq!(outcome.final_time_s.to_bits(), 0.002_f64.to_bits());
        assert_eq!(outcome.table.rows().len(), 5);
        assert_eq!(
            row_times_s(&outcome)
                .iter()
                .map(|time_s| time_s.to_bits())
                .collect::<Vec<_>>(),
            [0.0_f64, 0.0005, 0.001, 0.0015, 0.002]
                .iter()
                .map(|time_s| time_s.to_bits())
                .collect::<Vec<_>>()
        );
        let contact = outcome
            .contact
            .as_ref()
            .expect("contact report should be present");
        assert_eq!(contact.samples, 5);
    }

    #[test]
    fn point_mass_contact_diagnostics_are_golden_stable() {
        let scenario =
            Scenario::from_toml_str(CONTACT_POINT_MASS_SCENARIO).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let first = run(&scenario, &resolved_files, None).expect("first contact run succeeds");
        let second = run(&scenario, &resolved_files, None).expect("second contact run succeeds");

        assert_eq!(first.contact.as_ref(), second.contact.as_ref());
        for channel in [
            "force.contact.z_n",
            "contact.gap_m",
            "contact.penetration_m",
            "contact.normal_velocity_m_s",
            "contact.normal_force_n",
        ] {
            let first_bits: Vec<u64> = f64_column(&first, channel)
                .iter()
                .map(|value| value.to_bits())
                .collect();
            let second_bits: Vec<u64> = f64_column(&second, channel)
                .iter()
                .map(|value| value.to_bits())
                .collect();
            assert_eq!(first_bits, second_bits, "{channel} should be bit-stable");
        }
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
    fn point_mass_wires_frame_coupled_tesseral_gravity() {
        let scenario = Scenario::from_toml_str(
            &TESSERAL_POINT_MASS_SCENARIO
                .replace(
                    "\"@WGS84_A_M@\"",
                    &format!("{:.1}", openbmp_physics::WGS84_A_M),
                )
                .replace(
                    "\"@NEG_WGS84_J2@\"",
                    &format!("{:.9e}", -openbmp_physics::WGS84_J2),
                ),
        )
        .expect("tesseral scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        let outcome = run(&scenario, &resolved_files, None).expect("tesseral run succeeds");

        let gravity_y = f64_column(&outcome, "force.gravity.y_n");
        assert!(
            gravity_y.iter().any(|value| value.abs() > 1.0e-3),
            "tesseral gravity should produce a finite cross-axis force: {gravity_y:?}"
        );
    }

    #[test]
    fn point_mass_wires_file_backed_egm2008_pines_gravity() {
        let source_dir = workspace_root();
        let scenario = Scenario::from_toml_str_with_source_dir(
            EGM2008_PINES_POINT_MASS_SCENARIO,
            Some(&source_dir),
        )
        .expect("file-backed EGM2008 scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolve files");
        assert!(resolved_files.contains_key("environment.egm2008_coefficients_file"));
        let outcome = run(&scenario, &resolved_files, None).expect("EGM2008 Pines run succeeds");

        let legacy_toml = EGM2008_PINES_POINT_MASS_SCENARIO
            .replace(
                "egm2008_coefficients_file = \"data/gravity/egm2008-degree10-normalized-icgem-v1.gfc\"\n",
                "",
            )
            .replace("egm2008_degree = 10\n", "")
            .replace("egm2008_order = 10\n", "")
            .replace("egm2008_finite_difference_step_m = 10.0\n", "");
        let legacy = Scenario::from_toml_str_with_source_dir(&legacy_toml, Some(&source_dir))
            .expect("legacy EGM2008 scenario must parse");
        let legacy_files = legacy.resolved_files().expect("resolve legacy files");
        let legacy_outcome =
            run(&legacy, &legacy_files, None).expect("legacy EGM2008 run succeeds");

        let x = f64_column(&outcome, "force.gravity.x_n");
        let legacy_x = f64_column(&legacy_outcome, "force.gravity.x_n");
        assert!(
            x.iter()
                .zip(legacy_x)
                .any(|(actual, legacy)| (actual - legacy).abs() > 1.0e-6),
            "file-backed EGM2008 should not silently use the legacy zonal model"
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
        assert!(!has_channel(&outcome, "recovery.dual_chute.opening_load_n"));
    }

    #[test]
    fn inflating_recovery_telemetry_channels_are_recorded() {
        let mut scenario = inflating_parachute_scenario();
        scenario.document.vehicle.assembly.recovery[0]
            .inflation
            .as_mut()
            .expect("inflation config present")
            .apparent_mass_coeff_k_a = Some(0.65);
        let resolved_files = scenario
            .resolved_files()
            .expect("resolve inflating parachute scenario");

        let outcome =
            run(&scenario, &resolved_files, None).expect("inflating parachute run succeeds");

        assert!(has_channel(
            &outcome,
            "recovery.main_chute.inflation_progress"
        ));
        let progress = f64_column(&outcome, "recovery.main_chute.inflation_progress");
        let first_inflating = progress
            .iter()
            .position(|value| *value > 0.0)
            .expect("inflation progress should become positive");
        assert!(progress[first_inflating] < 1.0);

        let drag_area = f64_column(&outcome, "recovery.main_chute.drag_area_m2");
        let dynamic_pressure =
            f64_column(&outcome, "recovery.main_chute.opening_dynamic_pressure_pa");
        let opening_load = f64_column(&outcome, "recovery.main_chute.opening_load_n");
        let opening_peak = f64_column(&outcome, "recovery.main_chute.opening_peak_load_n");
        let apparent_mass = f64_column(&outcome, "recovery.main_chute.apparent_mass_kg");
        let apparent_mass_rate =
            f64_column(&outcome, "recovery.main_chute.apparent_mass_rate_kg_s");
        assert!(drag_area[first_inflating] > 0.0);
        assert!(dynamic_pressure[first_inflating] > 0.0);
        assert!(opening_load[first_inflating] > 0.0);
        assert!(opening_peak[first_inflating] > opening_load[first_inflating]);
        assert!(apparent_mass[first_inflating] > 0.0);
        assert!(apparent_mass_rate[first_inflating] > 0.0);
        assert_relative_eq!(
            opening_load[first_inflating] / opening_peak[first_inflating],
            drag_area[first_inflating] / 8.0,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn recovery_riser_telemetry_channels_are_recorded() {
        let mut scenario = inflating_parachute_scenario();
        scenario.document.vehicle.assembly.recovery[0].risers.push(
            openbmp_scenario::RecoveryRiserConfig {
                id: "main_riser".to_owned(),
                unstretched_length_m: 0.5,
                axial_rigidity_n: 200.0,
                damping_n_s_m: 0.0,
                vehicle_attach_body_m: [0.0, 1.0, 0.0],
                canopy_attach_body_m: [1.0, 1.0, 0.0],
                canopy_velocity_body_m_s: None,
                canopy_mass_kg: None,
            },
        );
        let resolved_files = scenario
            .resolved_files()
            .expect("resolve riser parachute scenario");

        let outcome = run(&scenario, &resolved_files, None).expect("riser run succeeds");

        assert!(has_channel(
            &outcome,
            "recovery.main_chute.riser_tension_sum_n"
        ));
        assert!(has_channel(
            &outcome,
            "recovery.main_chute.riser_moment_body_z_n_m"
        ));
        assert!(has_channel(&outcome, "force.recovery_riser_force.x_n"));
        let tension = f64_column(&outcome, "recovery.main_chute.riser_tension_sum_n");
        let first_loaded = tension
            .iter()
            .position(|value| *value > 0.0)
            .expect("riser tension should become positive after deploy");
        assert_eq!(tension[first_loaded].to_bits(), 200.0_f64.to_bits());
        assert_eq!(
            f64_column(&outcome, "recovery.main_chute.riser_tension_max_n")[first_loaded].to_bits(),
            200.0_f64.to_bits()
        );
        assert_eq!(
            f64_column(&outcome, "recovery.main_chute.riser_force_body_x_n")[first_loaded]
                .to_bits(),
            200.0_f64.to_bits()
        );
        assert_eq!(
            f64_column(&outcome, "force.recovery_riser_force.x_n")[first_loaded].to_bits(),
            200.0_f64.to_bits()
        );
        assert_eq!(
            f64_column(&outcome, "force.recovery_riser_force.y_n")[first_loaded].to_bits(),
            0.0_f64.to_bits()
        );
        let riser_force_z =
            f64_column(&outcome, "recovery.main_chute.riser_force_body_z_n")[first_loaded];
        assert!(
            riser_force_z > 0.0,
            "riser drag handoff should add a positive body-Z force"
        );
        assert_eq!(
            f64_column(&outcome, "force.recovery_riser_force.z_n")[first_loaded].to_bits(),
            riser_force_z.to_bits()
        );
        assert_eq!(
            f64_column(&outcome, "force.recovery_drag.z_n")[first_loaded].to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            f64_column(&outcome, "recovery.main_chute.riser_moment_body_z_n_m")[first_loaded]
                .to_bits(),
            (-200.0_f64).to_bits()
        );
        assert_eq!(
            f64_column(&outcome, "recovery.main_chute.riser_elastic_energy_j")[first_loaded]
                .to_bits(),
            50.0_f64.to_bits()
        );
    }

    #[test]
    fn recovery_sequence_snatch_telemetry_channels_are_recorded() {
        let mut scenario = inflating_parachute_scenario();
        scenario.document.vehicle.assembly.recovery[0].sequence =
            Some(openbmp_scenario::RecoverySequenceConfig {
                snatch_delta_v_m_s: 3.0,
                snatch_effective_mass_kg: 5.0,
                pack_drag_load_n: 4.0,
                mortar_pack_mass_kg: None,
                mortar_pack_velocity_body_m_s: None,
            });
        scenario.document.vehicle.assembly.recovery[0].risers.push(
            openbmp_scenario::RecoveryRiserConfig {
                id: "main_riser".to_owned(),
                unstretched_length_m: 0.5,
                axial_rigidity_n: 200.0,
                damping_n_s_m: 0.0,
                vehicle_attach_body_m: [0.0, 1.0, 0.0],
                canopy_attach_body_m: [1.0, 1.0, 0.0],
                canopy_velocity_body_m_s: None,
                canopy_mass_kg: None,
            },
        );
        let resolved_files = scenario
            .resolved_files()
            .expect("resolve sequence parachute scenario");

        let outcome = run(&scenario, &resolved_files, None).expect("sequence run succeeds");

        assert!(has_channel(
            &outcome,
            "recovery.main_chute.snatch_peak_load_n"
        ));
        assert!(has_channel(
            &outcome,
            "recovery.main_chute.snatch_stiffness_n_m"
        ));
        let peak = f64_column(&outcome, "recovery.main_chute.snatch_peak_load_n");
        let first_loaded = peak
            .iter()
            .position(|value| *value > 0.0)
            .expect("snatch diagnostic should become positive after deploy");
        assert_eq!(
            f64_column(&outcome, "recovery.main_chute.snatch_delta_v_m_s")[first_loaded].to_bits(),
            3.0_f64.to_bits()
        );
        assert_eq!(
            f64_column(&outcome, "recovery.main_chute.snatch_stiffness_n_m")[first_loaded]
                .to_bits(),
            400.0_f64.to_bits()
        );
        assert_eq!(
            f64_column(&outcome, "recovery.main_chute.snatch_effective_mass_kg")[first_loaded]
                .to_bits(),
            5.0_f64.to_bits()
        );
        assert_eq!(
            f64_column(&outcome, "recovery.main_chute.snatch_pack_drag_load_n")[first_loaded]
                .to_bits(),
            4.0_f64.to_bits()
        );
        let expected_peak = 3.0 * (400.0_f64 * 5.0).sqrt() + 4.0;
        assert!((peak[first_loaded] - expected_peak).abs() <= 1.0e-12);
    }

    #[test]
    fn recovery_sequence_mortar_impulse_force_channels_are_recorded() {
        let mut scenario = inflating_parachute_scenario();
        let dt_s = scenario.document.time.dt_s;
        scenario.document.vehicle.assembly.recovery[0].sequence =
            Some(openbmp_scenario::RecoverySequenceConfig {
                snatch_delta_v_m_s: 3.0,
                snatch_effective_mass_kg: 5.0,
                pack_drag_load_n: 4.0,
                mortar_pack_mass_kg: Some(2.0),
                mortar_pack_velocity_body_m_s: Some([0.0, 0.0, 6.0]),
            });
        scenario.document.vehicle.assembly.recovery[0].risers.push(
            openbmp_scenario::RecoveryRiserConfig {
                id: "main_riser".to_owned(),
                unstretched_length_m: 0.5,
                axial_rigidity_n: 200.0,
                damping_n_s_m: 0.0,
                vehicle_attach_body_m: [0.0, 1.0, 0.0],
                canopy_attach_body_m: [1.0, 1.0, 0.0],
                canopy_velocity_body_m_s: None,
                canopy_mass_kg: None,
            },
        );
        let resolved_files = scenario
            .resolved_files()
            .expect("resolve mortar sequence parachute scenario");

        let outcome = run(&scenario, &resolved_files, None).expect("mortar sequence run succeeds");

        assert!(has_channel(&outcome, "force.recovery_sequence_force.z_n"));
        assert!(has_channel(
            &outcome,
            "recovery.main_chute.mortar_impulse_body_z_n_s"
        ));
        let force_z = f64_column(&outcome, "force.recovery_sequence_force.z_n");
        let first_impulse = force_z
            .iter()
            .position(|value| *value != 0.0)
            .expect("mortar impulse force should be applied for one tick");
        let impulse_z =
            f64_column(&outcome, "recovery.main_chute.mortar_impulse_body_z_n_s")[first_impulse];
        let mortar_force_z =
            f64_column(&outcome, "recovery.main_chute.mortar_force_body_z_n")[first_impulse];
        assert_eq!(impulse_z.to_bits(), (-12.0_f64).to_bits());
        assert_eq!(mortar_force_z.to_bits(), force_z[first_impulse].to_bits());
        assert!((force_z[first_impulse] * dt_s - impulse_z).abs() <= 1.0e-12);
        assert!(
            force_z
                .iter()
                .enumerate()
                .filter(|(_, value)| **value != 0.0)
                .all(|(index, _)| index == first_impulse),
            "mortar force should be nonzero for exactly one telemetry row"
        );
    }

    #[test]
    fn pflanz_recovery_telemetry_channels_are_recorded() {
        let mut scenario = inflating_parachute_scenario();
        scenario.document.vehicle.assembly.recovery[0]
            .inflation
            .as_mut()
            .expect("inflation config present")
            .pflanz = Some(openbmp_scenario::RecoveryPflanzReductionConfig {
            format: None,
            provenance: None,
            file: Some(
                workspace_root()
                    .join("data/recovery/pflanz-x1-synthetic-v1.toml")
                    .display()
                    .to_string(),
            ),
            file_sha256: Some(
                "2d679136f9f123eced753fbbbb3f6b67432be2530462b5ab2366cb239737546c".to_owned(),
            ),
            ballistic_parameter_a: Vec::new(),
            exponent_j: Vec::new(),
            reduction_factors: Vec::new(),
        });
        let resolved_files = scenario
            .resolved_files()
            .expect("resolve Pflanz parachute scenario");

        let outcome = run(&scenario, &resolved_files, None).expect("Pflanz parachute run succeeds");

        assert!(has_channel(
            &outcome,
            "recovery.main_chute.opening_ballistic_parameter_a"
        ));
        assert!(has_channel(
            &outcome,
            "recovery.main_chute.opening_pflanz_reduction_factor_x1"
        ));
        assert!(has_channel(
            &outcome,
            "recovery.main_chute.opening_finite_mass_peak_load_n"
        ));

        let progress = f64_column(&outcome, "recovery.main_chute.inflation_progress");
        let first_inflating = progress
            .iter()
            .position(|value| *value > 0.0)
            .expect("inflation progress should become positive");
        let opening_peak = f64_column(&outcome, "recovery.main_chute.opening_peak_load_n");
        let ballistic = f64_column(
            &outcome,
            "recovery.main_chute.opening_ballistic_parameter_a",
        );
        let reduction = f64_column(
            &outcome,
            "recovery.main_chute.opening_pflanz_reduction_factor_x1",
        );
        let finite_peak = f64_column(
            &outcome,
            "recovery.main_chute.opening_finite_mass_peak_load_n",
        );
        assert!(ballistic[first_inflating] > 0.0);
        assert!((0.25..=0.97).contains(&reduction[first_inflating]));
        assert!(finite_peak[first_inflating] > 0.0);
        assert!(finite_peak[first_inflating] < opening_peak[first_inflating]);
        assert_relative_eq!(
            finite_peak[first_inflating] / opening_peak[first_inflating],
            reduction[first_inflating],
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn pflanz_recovery_integrated_descent_peak_matches_synthetic_x1() {
        let density_kg_m3 = UsStandard1976::new()
            .sample(1000.0, SimTime::ZERO)
            .expect("USSA76 density at inflation test altitude")
            .density_kg_m3;
        let expected_ratio = integrated_opening_load_ratio_fixed_step(
            density_kg_m3,
            10.0,
            1.5 * 8.0,
            50.0,
            1.0,
            0.002,
        );
        assert!(expected_ratio > 0.0);
        assert!(expected_ratio < 1.0);

        let scenario = pflanz_integrated_peak_scenario(expected_ratio);
        let resolved_files = scenario
            .resolved_files()
            .expect("resolve Pflanz integrated peak scenario");
        let outcome =
            run(&scenario, &resolved_files, None).expect("Pflanz integrated peak run succeeds");

        let progress = f64_column(&outcome, "recovery.main_chute.inflation_progress");
        let first_inflating = progress
            .iter()
            .position(|value| *value > 0.0)
            .expect("inflation progress should become positive");
        let opening_load = f64_column(&outcome, "recovery.main_chute.opening_load_n");
        let opening_peak = f64_column(&outcome, "recovery.main_chute.opening_peak_load_n");
        let reduction = f64_column(
            &outcome,
            "recovery.main_chute.opening_pflanz_reduction_factor_x1",
        );
        let finite_peak = f64_column(
            &outcome,
            "recovery.main_chute.opening_finite_mass_peak_load_n",
        );
        let velocity_x = f64_column(&outcome, "velocity_x_m_s");

        let (peak_index, measured_peak_load_n) = opening_load
            .iter()
            .copied()
            .enumerate()
            .filter(|(index, _)| progress[*index] > 0.0 && progress[*index] <= 1.0)
            .max_by(|(_, lhs), (_, rhs)| lhs.total_cmp(rhs))
            .expect("inflation load samples exist");
        let measured_ratio = measured_peak_load_n / opening_peak[first_inflating];

        assert!(peak_index > first_inflating);
        assert!(progress[peak_index] < 1.0);
        assert!(velocity_x[peak_index] < velocity_x[first_inflating]);
        assert_relative_eq!(
            reduction[first_inflating],
            expected_ratio,
            epsilon = 1.0e-15
        );
        assert_relative_eq!(
            finite_peak[first_inflating] / opening_peak[first_inflating],
            expected_ratio,
            epsilon = 1.0e-15
        );
        assert_relative_eq!(measured_ratio, expected_ratio, epsilon = 2.0e-4);
        assert!(measured_ratio < 1.0);
    }

    #[test]
    fn pflanz_recovery_integrated_descent_peak_sweep_matches_fixture() {
        let fixture = load_recovery_pflanz_integrated_sweep_fixture();
        assert_eq!(
            fixture.format,
            "openbmp.runner.recovery_pflanz_integrated_sweep.v1"
        );
        assert!(fixture.cases.len() >= 5);

        let density_kg_m3 = UsStandard1976::new()
            .sample(1000.0, SimTime::ZERO)
            .expect("USSA76 density at inflation sweep altitude")
            .density_kg_m3;

        for fixture_case in &fixture.cases {
            let full_c_d_s_m2 = fixture_case.c_d * fixture_case.area_m2;
            let dry_mass_kg = dry_mass_for_target_pflanz_a(
                fixture_case.target_ballistic_parameter_a,
                density_kg_m3,
                full_c_d_s_m2,
                fixture_case.initial_speed_m_s,
                fixture_case.fill_time_s,
            );
            let expected = integrated_opening_load_fixed_step(
                density_kg_m3,
                dry_mass_kg,
                full_c_d_s_m2,
                fixture_case.initial_speed_m_s,
                fixture_case.fill_time_s,
                fixture_case.dt_s,
            );
            assert!(
                expected.peak_ratio > 0.0 && expected.peak_ratio <= 1.0,
                "{} expected ratio out of range: {:?}",
                fixture_case.case,
                expected
            );

            let scenario = pflanz_integrated_peak_scenario_from_values(
                &fixture_case.case,
                dry_mass_kg,
                fixture_case.initial_speed_m_s,
                fixture_case.fill_time_s,
                fixture_case.dt_s,
                fixture_case.c_d,
                fixture_case.area_m2,
                expected.peak_ratio,
            );
            let resolved_files = scenario
                .resolved_files()
                .expect("resolve Pflanz integrated sweep scenario");
            let outcome = run(&scenario, &resolved_files, None)
                .expect("Pflanz integrated sweep run succeeds");

            let progress = f64_column(&outcome, "recovery.main_chute.inflation_progress");
            let first_inflating = progress
                .iter()
                .position(|value| *value > 0.0)
                .expect("inflation progress should become positive");
            let first_complete = progress
                .iter()
                .position(|value| *value >= 1.0)
                .expect("inflation progress should complete");
            let opening_load = f64_column(&outcome, "recovery.main_chute.opening_load_n");
            let opening_peak = f64_column(&outcome, "recovery.main_chute.opening_peak_load_n");
            let ballistic = f64_column(
                &outcome,
                "recovery.main_chute.opening_ballistic_parameter_a",
            );
            let reduction = f64_column(
                &outcome,
                "recovery.main_chute.opening_pflanz_reduction_factor_x1",
            );
            let finite_peak = f64_column(
                &outcome,
                "recovery.main_chute.opening_finite_mass_peak_load_n",
            );
            let velocity_x = f64_column(&outcome, "velocity_x_m_s");

            let (peak_index, measured_peak_load_n) = opening_load
                .iter()
                .copied()
                .enumerate()
                .filter(|(index, _)| progress[*index] > 0.0 && progress[*index] <= 1.0)
                .max_by(|(_, lhs), (_, rhs)| lhs.total_cmp(rhs))
                .expect("inflation load samples exist");
            let measured_ratio = measured_peak_load_n / opening_peak[first_inflating];
            let ballistic_rel =
                ((ballistic[first_inflating] / fixture_case.target_ballistic_parameter_a) - 1.0)
                    .abs();
            let finite_peak_ratio = finite_peak[first_inflating] / opening_peak[first_inflating];

            assert!(
                ballistic_rel <= fixture.tolerances.ballistic_parameter_rel,
                "{} ballistic parameter rel error {ballistic_rel:e}",
                fixture_case.case
            );
            assert!(
                (reduction[first_inflating] - expected.peak_ratio).abs()
                    <= fixture.tolerances.finite_peak_ratio_abs,
                "{} reduction factor mismatch",
                fixture_case.case
            );
            assert!(
                (finite_peak_ratio - expected.peak_ratio).abs()
                    <= fixture.tolerances.finite_peak_ratio_abs,
                "{} finite peak ratio mismatch",
                fixture_case.case
            );
            assert!(
                (measured_ratio - expected.peak_ratio).abs() <= fixture.tolerances.peak_ratio_abs,
                "{} measured peak ratio {measured_ratio:e}, expected {:e}, peak_index {peak_index}",
                fixture_case.case,
                expected.peak_ratio
            );
            assert!(
                (velocity_x[first_complete] - expected.final_speed_m_s).abs()
                    <= fixture.tolerances.final_speed_abs_m_s,
                "{} final speed {}, expected {}",
                fixture_case.case,
                velocity_x[first_complete],
                expected.final_speed_m_s
            );
            assert!(velocity_x[first_complete] < fixture_case.initial_speed_m_s);
        }
    }

    #[test]
    fn recovery_drag_declared_constant_wind_matches_closed_form_drift() {
        let no_wind_scenario = recovery_wind_drift_scenario(false);
        let no_wind_files = no_wind_scenario
            .resolved_files()
            .expect("resolve no-wind recovery drift scenario");
        let no_wind = run(&no_wind_scenario, &no_wind_files, None)
            .expect("no-wind recovery drift scenario runs");

        let wind_scenario = recovery_wind_drift_scenario(true);
        let wind_files = wind_scenario
            .resolved_files()
            .expect("resolve wind recovery drift scenario");
        let wind =
            run(&wind_scenario, &wind_files, None).expect("wind recovery drift scenario runs");

        assert_eq!(
            f64_column(&no_wind, "position_x_m")
                .last()
                .copied()
                .expect("no-wind final x")
                .to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            f64_column(&no_wind, "velocity_x_m_s")
                .last()
                .copied()
                .expect("no-wind final vx")
                .to_bits(),
            0.0_f64.to_bits()
        );

        let wind_speed_m_s = 12.0;
        let mass_kg = 10.0;
        let c_d_area_m2 = 1.0;
        let density_kg_m3 = UsStandard1976::new()
            .sample(1000.0, SimTime::ZERO)
            .expect("USSA76 density at recovery drift altitude")
            .density_kg_m3;
        // Closed-form one-dimensional constant-wind solution for
        // dv/dt = a * (wind - v)^2 after the deploy command reaches the rack.
        let accel_coeff = 0.5 * density_kg_m3 * c_d_area_m2 / mass_kg;
        let active_time_s = wind_scenario.document.time.stop_s - wind_scenario.document.time.dt_s;
        let factor = 1.0 + accel_coeff * wind_speed_m_s * active_time_s;
        let expected_vx_m_s = wind_speed_m_s * (1.0 - factor.recip());
        let expected_x_m = wind_speed_m_s * active_time_s - factor.ln() / accel_coeff;

        let wind_x = f64_column(&wind, "position_x_m")
            .last()
            .copied()
            .expect("wind final x");
        let wind_vx = f64_column(&wind, "velocity_x_m_s")
            .last()
            .copied()
            .expect("wind final vx");
        assert_relative_eq!(wind_x, expected_x_m, epsilon = 3.0e-5);
        assert_relative_eq!(wind_vx, expected_vx_m_s, epsilon = 1.0e-5);
        assert!(wind_x > 0.0);
        assert!(wind_vx > 0.0);
        assert!(wind_vx < wind_speed_m_s);
        assert_eq!(
            f64_column(&wind, "position_y_m")
                .last()
                .copied()
                .expect("wind final y")
                .to_bits(),
            0.0_f64.to_bits()
        );
    }

    #[test]
    fn recovery_drag_declared_constant_wind_shifts_splash_point() {
        let no_wind_scenario = recovery_splash_drift_scenario(false);
        let no_wind_files = no_wind_scenario
            .resolved_files()
            .expect("resolve no-wind recovery splash drift scenario");
        let no_wind = run(&no_wind_scenario, &no_wind_files, None)
            .expect("no-wind recovery splash drift scenario runs");

        let wind_scenario = recovery_splash_drift_scenario(true);
        let wind_files = wind_scenario
            .resolved_files()
            .expect("resolve wind recovery splash drift scenario");
        let wind = run(&wind_scenario, &wind_files, None)
            .expect("wind recovery splash drift scenario runs");

        assert!(matches!(
            no_wind.stop_reason,
            StopReason::GroundImpact {
                ground_altitude_m,
                ..
            } if ground_altitude_m == 0.0
        ));
        assert!(matches!(
            wind.stop_reason,
            StopReason::GroundImpact {
                ground_altitude_m,
                ..
            } if ground_altitude_m == 0.0
        ));

        let no_wind_x = f64_column(&no_wind, "position_x_m")
            .last()
            .copied()
            .expect("no-wind final x");
        let wind_x = f64_column(&wind, "position_x_m")
            .last()
            .copied()
            .expect("wind final x");
        assert_eq!(no_wind_x.to_bits(), 0.0_f64.to_bits());
        assert!(wind_x > 0.0);
        assert!(wind_x < 6.0 * wind.final_time_s);

        assert!(
            f64_column(&wind, "position_z_m")
                .last()
                .copied()
                .expect("wind final z")
                <= 0.0
        );
        assert!(
            f64_column(&wind, "recovery.main_chute.drag_area_m2")
                .iter()
                .any(|area| area.to_bits() == 1.0_f64.to_bits())
        );
        assert_eq!(
            f64_column(&wind, "position_y_m")
                .last()
                .copied()
                .expect("wind final y")
                .to_bits(),
            0.0_f64.to_bits()
        );
    }
}
