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

use nalgebra::{Matrix3, Vector3};
use openbmp_aero::AeroDeck;
use openbmp_contact::{
    BacklashGap, BaumgarteCfeConfig, LatchState, LatchWindow, MonotoneLatch, PlanarFourBarLoop,
    ScalarMechanismElement, ScalarStop, TreeCfeLoopSolveBuffers, TreeCfeLoopSystem,
    evaluate_scalar_mechanism, scalar_mechanism_latch_count, solve_tree_loop_baumgarte_cfe,
};
use openbmp_core::{
    AngularVelocity3, Body, BodyId, ChannelId, Duration, EngineId, ModelId, Position3, Quaternion,
    RecoveryId, SimTime, TankId, ValidationStatus, Velocity3,
};
use openbmp_multibody::{
    Joint, MultibodyDerivative, MultibodySimState, MultibodyState, MultibodyTree, PluckerTransform,
    SpatialForce, SpatialInertia, SpatialMatrix, SpatialMotion, TreeBodySpec,
};
use openbmp_physics::{AtmosphereModel, ConstantGravity, J2Gravity, PointMassGravity, WGS84_J2};
use openbmp_propulsion::{Motor, SolidMotor};
use openbmp_scenario::{
    ContactMechanismConfig, ContactMechanismKindConfig, MultiBodyCfeGimbalLockConfig,
    MultiBodyCfeLoopConfig, MultiBodyCfePlanarFourBarConfig, MultiBodyGimbalJointConfig,
    MultiBodyGimbalMechanismAxisConfig, MultiBodyPropagationAuthorityConfig, RecoveryConfig,
    ResolvedFile, Scenario, ScenarioDocument,
};
use openbmp_sensors::SpecificForceTruth;
use openbmp_sim::{
    AnyStop, ConstantMass, EffectorActualsView, EndTime, EngineSnapshotView, EnvironmentModel,
    EnvironmentQuery, ForceContext, ForceModel, GroundImpact, InitialRigidBodyLane, Integrator,
    MassContext, MomentContext, RecoverySnapshotView, RigidBodyDerivative, RigidBodySeparation,
    RigidBodySeparationStates, RigidMassModel, RigidModels, Rk4FixedStep, ScenarioScriptAction,
    SimulationConfig, SimulationKernel, StopReason, TankSnapshotView,
};
use openbmp_state::{MassProperties, RigidBodyState};
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use openbmp_vehicle::{
    AeroFrameTransform, AeroMethodForceAdapter, AeroMethodMomentAdapter, Assembly, BoxedMassModel,
    DeckDragForceAdapter, EngineClusterForceAdapter, EngineClusterFrameTransform,
    EngineClusterMassAdapter, EngineClusterMomentAdapter, GravityForceAdapter, KernelVehicle,
    MotorThrustForceAdapter, MotorThrustFrameTransform, NamedForceModel, NamedMomentModel,
    RecoveryRackMassAdapter, RecoverySequenceForceAdapter, TankRackFrameTransform, Vehicle,
    VehicleAssembly,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::RunOutcome;
use crate::assembly::{dry_mass_kg_at, dry_mass_properties_at};
use crate::atmosphere::{
    RuntimeAtmosphere, RuntimeEnvironment, atmosphere_altitude_m_with_surface_radius,
    build_document_runtime_atmosphere, document_geocentric_surface_radius_m,
    is_runtime_atmosphere_kind, scenario_atmosphere_kind,
};
use crate::error::RunnerError;
use crate::integrator::{RuntimeIntegrator, build_runtime_integrator};

// Stable model-ids assigned to each force / mass model the rigid
// runner wires. Reserves a separate range from the point-mass
// runner so determinism tooling can distinguish the two
// paths.
const RIGID_BODY_GRAVITY_MODEL_ID: ModelId = ModelId::new(301);
const RIGID_BODY_AERO_MODEL_ID: ModelId = ModelId::new(302);
const RIGID_BODY_THRUST_MODEL_ID: ModelId = ModelId::new(303);
const RIGID_BODY_MOTOR_MASS_MODEL_ID: ModelId = ModelId::new(304);
const RIGID_BODY_AEROTHERMAL_MODEL_ID: ModelId = ModelId::new(305);
const RIGID_BODY_CONTACT_MODEL_ID: ModelId = ModelId::new(306);
const RIGID_BODY_LANDING_GEAR_FORCE_MODEL_ID: ModelId = ModelId::new(307);
const RIGID_BODY_LANDING_GEAR_MOMENT_MODEL_ID: ModelId = ModelId::new(308);
// Distinct model ids for the engine-cluster path on the
// rigid-body kernel.
const RIGID_BODY_ENGINE_CLUSTER_THRUST_MODEL_ID: ModelId = ModelId::new(320);
const RIGID_BODY_ENGINE_CLUSTER_MASS_MODEL_ID: ModelId = ModelId::new(321);
const RIGID_BODY_ENGINE_CLUSTER_MOMENT_MODEL_ID: ModelId = ModelId::new(322);
const RIGID_BODY_TANK_RACK_FORCE_MODEL_ID: ModelId = ModelId::new(340);
const RIGID_BODY_TANK_RACK_MOMENT_MODEL_ID: ModelId = ModelId::new(341);
const RIGID_BODY_RECOVERY_RACK_FORCE_MODEL_ID: ModelId = ModelId::new(380);
const RIGID_BODY_RECOVERY_RACK_MASS_MODEL_ID: ModelId = ModelId::new(381);
const RIGID_BODY_RECOVERY_RISER_MOMENT_MODEL_ID: ModelId = ModelId::new(382);
const RIGID_BODY_RECOVERY_RISER_FORCE_MODEL_ID: ModelId = ModelId::new(383);
const RIGID_BODY_RECOVERY_SEQUENCE_FORCE_MODEL_ID: ModelId = ModelId::new(384);
// Distinct model id for the direct-torque moment
// adapter on the rigid-body kernel.
const RIGID_BODY_DIRECT_TORQUE_MOMENT_MODEL_ID: ModelId = ModelId::new(500);
const RIGID_BODY_MULTIBODY_SHADOW_MODEL_ID: ModelId = ModelId::new(520);
const MISSING_AEROTHERMAL_DIAGNOSTICS_MESSAGE: &str =
    "forces includes `aerothermal_diagnostics` but [aerothermal] block is missing";
const ARTICULATED_GIMBAL_THRUST_DIRECTION_EPS: f64 = 1.0e-12;
const ARTICULATED_GIMBAL_LIVE_JOINT_ABS_TOLERANCE_RAD: f64 = 1.0e-12;
const MULTIBODY_AUTHORITY_TIME_ABS_TOLERANCE_S: f64 = 1.0e-12;
const WELDED_RELEASE_MOMENTUM_ABS_TOLERANCE: f64 = 1.0e-8;
const WELDED_RELEASE_MOMENTUM_REL_TOLERANCE: f64 = 1.0e-10;

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
pub fn run(
    scenario: &Scenario,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    mut monitor: Option<&mut (dyn crate::sil::SilMonitor + '_)>,
) -> Result<RunOutcome, RunnerError> {
    let mut session = RigidBodySession::prepare(scenario, resolved_files)?;
    while !session.is_finished() {
        session.step_once(&scenario.document, monitor.as_deref_mut())?;
    }
    session.finish()
}

/// Concrete kernel type the rigid-body runner instantiates: every model
/// slot is a runtime-dispatch adapter, so the prepared run is a single
/// nameable type a host can own and drive incrementally.
type RigidKernel = SimulationKernel<
    RigidBodyState,
    RuntimeIntegrator,
    KernelVehicle<RigidBodyState>,
    RigidModels<KernelVehicle<RigidBodyState>, RigidMassEither>,
    RuntimeEnvironment,
    AnyStop<GroundImpact, EndTime>,
>;

/// A prepared rigid-body run that the caller drives one kernel tick at a
/// time.
///
/// This struct owns exactly the state the one-shot [`run`] loop kept in
/// locals — racks, kernel, flight-controller bridge, telemetry table,
/// pending script-event queues — so [`run`] is itself implemented as
/// `prepare → step_once* → finish` and a session-stepped run is
/// byte-identical to a one-shot run by construction.
///
/// The session additionally accepts deterministic runtime malfunction
/// stimuli (engine faults, sensor outage windows, a wind override)
/// between ticks; see [`crate::Session`] for the public surface and the
/// determinism contract.
#[derive(Debug)]
pub(crate) struct RigidBodySession {
    effector_rack: crate::effectors::EffectorRack,
    engine_rack: crate::engines::EngineRack,
    tank_rack: crate::tanks::TankRack,
    propellant_budget: Option<openbmp_vehicle::PropellantBudget>,
    feed_network_rack: crate::feed_network::FeedNetworkRack,
    recovery_rack: crate::recovery::RecoveryRack,
    separated_attitude_targets: crate::separated_attitude::SeparatedAttitudeTargets,
    separated_landing_controllers: crate::separated_landing::SeparatedLandingControllers,
    wind_rack: crate::wind::WindRack,
    structural_rack: crate::structural::StructuralRack,
    upstream_uq: openbmp_uq::CorrelatedErrorBudget,
    frame: openbmp_physics::FrameContext,
    aerothermal_driver: Option<crate::aerothermal::LiveAerothermalDriver>,
    mass_model: RigidMassEither,
    separation_specs: BTreeMap<BodyId, RigidBodySeparationSpec>,
    stack_bodies: BTreeSet<BodyId>,
    kernel_step_s: f64,
    separated_ground_radius_m: Option<f64>,
    kernel: RigidKernel,
    channel_set: RigidChannelSet,
    contact_evaluator: Option<crate::contact::ContactDiagnosticsEvaluator>,
    contact_accumulator: Option<crate::contact::ContactRunAccumulator>,
    landing_gear_runtime: Option<crate::landing_gear::LandingGearRuntime>,
    landing_gear_accumulator: Option<crate::landing_gear::LandingGearRunAccumulator>,
    afts_monitor: Option<crate::afts::RunnerAftsMonitor>,
    comm_accumulator: Option<crate::comm::CommRunAccumulator>,
    acoustics_report: Option<crate::acoustics::AcousticsRunReport>,
    structural_acoustics: Option<crate::structural::StructuralAcousticLoadsReport>,
    structural_events: Option<crate::structural::StructuralEventLoadsReport>,
    pogo: Option<crate::pogo::PogoRunReport>,
    csi: Option<crate::csi::CsiRunReport>,
    countdown: Option<crate::ground::GroundCountdownRunReport>,
    geocentric_surface_radius_m: Option<f64>,
    breakdown_atmosphere: Option<RuntimeAtmosphere>,
    breakdown_vehicle: KernelVehicle<RigidBodyState>,
    moment_breakdown_vehicle: KernelVehicle<RigidBodyState>,
    primary_multibody_shadow: Option<RootFreeFlyerMultibodyShadow>,
    separated_multibody_shadows: BTreeMap<BodyId, RootFreeFlyerMultibodyShadow>,
    articulated_gimbal_shadows: BTreeMap<EngineId, ArticulatedGimbalMultibodyShadow>,
    articulated_gimbal_root_authority: Option<ArticulatedGimbalRootAuthorityShadow>,
    multibody_moment_model: Option<KernelVehicle<RigidBodyState>>,
    table: TelemetryTable,
    deck_bindings: Vec<crate::aero_effector_match::DeckAxisBinding>,
    direct_torque_present: bool,
    fc_bridge: Option<crate::fc_bridge::FcBridge>,
    mission_region_trace: crate::MissionRegionTraceState,
    plume_evaluator: Option<crate::plume::RigidPlumeEvaluator>,
    pending_effector_events: Vec<openbmp_sim::FiredEvent<ScenarioScriptAction>>,
    pending_engine_events: Vec<openbmp_sim::FiredEvent<ScenarioScriptAction>>,
    pending_recovery_events: Vec<openbmp_sim::FiredEvent<ScenarioScriptAction>>,
    realtime_pacer: crate::rt::RunnerRealtimePacer,
    /// Runtime wind override in NED metres per second. While `Some`, it
    /// replaces the wind sample pushed into the kernel each tick (and is
    /// pushed even for scenarios without a `[wind]` block). `None` leaves
    /// the scenario's own wind path byte-identical to a plain [`run`].
    wind_override_ned_m_s: Option<Vector3<f64>>,
    /// Monotonic sequence for runtime malfunction ids, so repeated
    /// injections get unique scheduled-fault ids.
    runtime_malfunction_seq: u64,
}

impl RigidBodySession {
    /// Build every rack, model, kernel, and telemetry channel for the
    /// scenario and record the step-zero telemetry row — the exact setup
    /// the one-shot [`run`] performs before its first tick.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`run`]'s setup phase.
    #[allow(clippy::too_many_lines)] // mirrors the one-shot setup verbatim
    pub(crate) fn prepare(
        scenario: &Scenario,
        resolved_files: &BTreeMap<String, ResolvedFile>,
    ) -> Result<Self, RunnerError> {
        let document = &scenario.document;
        require_supported_shape(document)?;
        let assembly = crate::assembly::synthesize_assembly(document)?;
        // Same effector-rack pattern as the point-mass
        // runner. Empty rack means no per-step effector operations.
        let effector_rack = crate::effectors::EffectorRack::build(document)?;
        // See point_mass.rs for the rationale.
        let engine_rack = crate::engines::EngineRack::build(document, resolved_files)?;
        // Tank rack mirroring the point-mass runner.
        let tank_rack = crate::tanks::TankRack::build(document)?;
        let propellant_budget = crate::propulsion::build_propellant_budget(document)?;
        let feed_network_rack =
            crate::feed_network::FeedNetworkRack::build(document, resolved_files)?;
        let pogo = crate::pogo::PogoStabilityRack::build(document)?.into_report();
        let countdown = crate::ground::evaluate_countdown_document(document)?;
        let landing_gear_owner_aliases = landing_gear_owner_aliases(document);
        let landing_gear_runtime =
            crate::landing_gear::LandingGearRuntime::maybe_build_with_owner_aliases(
                document,
                &landing_gear_owner_aliases,
            )?;
        let contact_evaluator = document
            .contact
            .as_ref()
            .map(crate::contact::ContactDiagnosticsEvaluator::from_config)
            .transpose()?;
        // Recovery rack mirroring the point-mass runner.
        let recovery_frame_aliases = recovery_frame_aliases(document);
        let recovery_rack = crate::recovery::RecoveryRack::build_with_frame_aliases(
            document,
            resolved_files,
            &recovery_frame_aliases,
        )?;
        let separated_attitude_targets =
            crate::separated_attitude::SeparatedAttitudeTargets::build(document)?;
        let separated_landing_controllers =
            crate::separated_landing::SeparatedLandingControllers::build(document)?;
        // Build the runner-side wind rack. Inactive when no
        // `[wind]` block is declared (or `kind = "none"`).
        let wind_rack = crate::wind::WindRack::build(document)?;
        wind_rack.reset();
        // Structural modal rack. Inactive when no structural mode block exists.
        let mut structural_rack =
            crate::structural::StructuralRack::build_with_resolved_files(document, resolved_files)?;
        structural_rack.reset();
        let mut upstream_uq = structural_rack.upstream_uq_budget();
        upstream_uq
            .sources
            .extend(crate::aero::upstream_uq_budget(document)?.sources);
        upstream_uq
            .sources
            .extend(crate::aerothermal::upstream_uq_budget(document)?.sources);
        upstream_uq
            .sources
            .extend(engine_rack.upstream_uq_budget().sources);
        let frame = crate::frames::build_frame_context(document, resolved_files)?;

        let loaded = load_models(document, resolved_files)?;
        crate::aero::reject_hypersonic_deck_only_out_of_envelope(
            document,
            loaded.aero_deck.as_ref(),
        )?;
        let mut aerothermal_driver =
            crate::aerothermal::LiveAerothermalDriver::maybe_new(document)?;
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
        let kernel_step_s = crate::contact::kernel_step_s(document);
        validate_multi_body_cfe_loop_telemetry_suffixes(document)?;
        let initial_multibody_root = build_initial_root_free_flyer_multibody_seed(
            document,
            &loaded,
            &mass_resources,
            &initial_engine_snapshot,
            &initial_tank_snapshot,
            &initial_state,
        )?;
        let initial_articulated_gimbal_seeds = build_initial_articulated_gimbal_multibody_seeds(
            document,
            &loaded,
            &mass_resources,
            &initial_engine_snapshot,
            &initial_tank_snapshot,
            &initial_state,
        )?;
        let articulated_gimbal_root_authority =
            build_initial_articulated_gimbal_root_authority_shadow(
                document,
                &initial_articulated_gimbal_seeds,
                &initial_engine_snapshot,
                kernel_step_s,
            )?;
        let mut articulated_gimbal_shadows = BTreeMap::new();
        for seed in initial_articulated_gimbal_seeds {
            seed.tree
                .joint_space_inertia_crba_at_state(seed.sim_state.state())
                .map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!("initial articulated gimbal inertia validation failed: {err}"),
                })?;
            let mut mechanism_latch_states = articulated_gimbal_mechanism_latch_states(&seed);
            let derivative_eval = articulated_gimbal_multibody_derivative_from_engine_thrust(
                &seed,
                &initial_engine_snapshot,
                kernel_step_s,
                &mut mechanism_latch_states,
            )?;
            let forecast = articulated_gimbal_multibody_rk4_forecast_from_engine_thrust(
                &seed,
                &initial_engine_snapshot,
                kernel_step_s,
                &mechanism_latch_states,
            )?;
            let root_forecast = articulated_gimbal_root_rigid_state_from_seed(&forecast)?;
            articulated_gimbal_shadows.insert(
                seed.engine,
                ArticulatedGimbalMultibodyShadow {
                    last_live_joint_state: articulated_gimbal_joint_state_from_engine_thrust(
                        &seed,
                        &initial_engine_snapshot,
                        kernel_step_s,
                    )?,
                    seed,
                    last_derivative: Some(derivative_eval.derivative),
                    last_cfe_loop_solves: derivative_eval.cfe_loop_solves,
                    last_rk4_forecast: Some(forecast),
                    last_root_rk4_forecast: Some(root_forecast),
                    last_authoritative_root_state: None,
                    mechanism_latch_states,
                },
            );
        }
        let kernel_vehicle = build_vehicle(
            document,
            &loaded,
            &assembly,
            resolved_files,
            landing_gear_runtime.clone(),
            aerothermal_sink.clone(),
            contact_evaluator.as_ref(),
        )?;
        let breakdown_vehicle = build_vehicle(
            document,
            &loaded,
            &assembly,
            resolved_files,
            landing_gear_runtime.clone(),
            aerothermal_sink,
            contact_evaluator.as_ref(),
        )?;
        let mass_model = build_mass_model(
            &loaded,
            &mass_resources,
            &initial_engine_snapshot,
            &initial_tank_snapshot,
            aerothermal_feedback,
        );
        let moment_model = build_moment_model(document, &loaded, landing_gear_runtime.clone())?;
        let moment_breakdown_vehicle =
            build_moment_model(document, &loaded, landing_gear_runtime.clone())?;
        let runtime_environment =
            RuntimeEnvironment::from_document(document, resolved_files, &frame)?;
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
        let initial_effector_actuals = if !deck_bindings.is_empty() || direct_torque_present {
            build_effector_actual_snapshot_map(
                document,
                &deck_bindings,
                &initial_snapshot,
                direct_torque_present,
            )?
        } else {
            BTreeMap::new()
        };
        let initial_recovery_snapshot = if recovery_rack.is_empty() {
            BTreeMap::new()
        } else {
            recovery_rack.snapshot_map()
        };
        if let Some(seed) = &initial_multibody_root {
            let mut environment = runtime_environment
                .sample(EnvironmentQuery {
                    time: seed.rigid_state.time,
                    position_eci: seed.rigid_state.position,
                })
                .map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!("initial root free-flyer environment sample failed: {err}"),
                })?;
            if !wind_rack.is_inactive() {
                let wind =
                    wind_rack.sample(seed.rigid_state.position, &frame, seed.rigid_state.time)?;
                environment.set_wind_ned_m_s(wind);
            }
            let _initial_multibody_derivative =
                root_free_flyer_multibody_derivative_from_runner_models(
                    seed,
                    &kernel_vehicle,
                    &moment_model,
                    RootFreeFlyerLoadViews {
                        phase_id: None,
                        environment: &environment,
                        effector_actuals: &initial_effector_actuals,
                        engine_snapshot: &initial_engine_snapshot,
                        tank_snapshot: &initial_tank_snapshot,
                        recovery_snapshot: &initial_recovery_snapshot,
                    },
                )?;
        }
        let primary_multibody_shadow =
            initial_multibody_root.map(|seed| RootFreeFlyerMultibodyShadow {
                seed,
                last_derivative: None,
                last_rk4_forecast: None,
                last_authoritative_state: None,
            });
        let multibody_moment_model = if document.multi_body.is_some() {
            Some(build_moment_model(
                document,
                &loaded,
                landing_gear_runtime.clone(),
            )?)
        } else {
            None
        };
        let rigid_models = RigidModels::new(moment_model, mass_model.clone());
        let separation_specs = build_rigid_body_separations(document, &mass_resources)?;
        // Bodies currently attached to the primary continuing stack. Starts as
        // every configured body (minus any seeded as independent initial lanes)
        // and shrinks as jettison events fire. Used so a multi-body continuing
        // stack (e.g. an upper stage still carrying a fairing + payload)
        // conserves mass at each separation.
        let mut stack_bodies: BTreeSet<BodyId> =
            mass_resources.dry_bodies.keys().copied().collect();
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
        let separated_ground_radius_m = infer_near_surface_geocentric_radius_m(&initial_state);

        let config = SimulationConfig {
            initial_state,
            integrator: runtime_integrator,
            force_model: kernel_vehicle,
            mass_model: rigid_models,
            environment: runtime_environment,
            stop_condition: AnyStop::new(
                automatic_ground_impact(document),
                EndTime::new(SimTime::from_seconds(document.time.stop_s)),
            ),
            dt: Duration::from_seconds(kernel_step_s),
            scenario_seed: document.time.seed,
        };

        let mut kernel_base = SimulationKernel::new_rigid(config)?;
        if let Some(radius_m) = separated_ground_radius_m {
            kernel_base.set_separated_geocentric_ground_radius_m(radius_m)?;
        }
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
        seed_initial_rigid_body_lanes(
            &mut kernel,
            document,
            &loaded,
            &mass_resources,
            &initial_engine_snapshot,
            &initial_tank_snapshot,
        )?;
        let channel_set = RigidChannelSet::new(document)?;
        let acoustics_report = crate::acoustics::evaluate_document(document, resolved_files)?;
        let structural_acoustics = crate::structural::evaluate_acoustic_loads_handoff(
            document,
            acoustics_report.as_ref(),
        )?;
        let structural_events =
            crate::structural::evaluate_event_loads(document, &structural_rack)?;
        let csi = crate::csi::evaluate_document(document, &structural_rack)?;
        let mut contact_accumulator = document
            .contact
            .as_ref()
            .map(crate::contact::ContactRunAccumulator::from_config)
            .transpose()?;
        let mut landing_gear_accumulator = if landing_gear_runtime.is_some() {
            Some(crate::landing_gear::LandingGearRunAccumulator::new()?)
        } else {
            None
        };
        let geocentric_surface_radius_m = document_geocentric_surface_radius_m(document);
        let breakdown_atmosphere = if channel_set.has_atmosphere {
            Some(build_document_runtime_atmosphere(document)?)
        } else {
            None
        };
        let metadata = build_schema_metadata(document, resolved_files)?;
        let mut table = TelemetryTable::new(channel_set.schema(metadata)?);

        if !deck_bindings.is_empty() || direct_torque_present {
            kernel.set_effector_actuals(initial_effector_actuals);
        }
        if !engine_rack.is_empty() {
            kernel.set_engine_snapshot(engine_rack.snapshot_map());
        }
        if !tank_rack.is_empty() {
            kernel.set_tank_snapshot(tank_rack.snapshot_map());
        }
        if !recovery_rack.is_empty() {
            kernel.set_recovery_snapshot(initial_recovery_snapshot);
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
        let csi_gyro_notch = csi.as_ref().and_then(|report| report.gyro_notch);
        let fc_bridge =
            crate::fc_bridge::FcBridge::maybe_new(scenario, resolved_files, csi_gyro_notch)?;
        let mut mission_region_trace =
            crate::MissionRegionTraceState::new(&crate::mission_region_declarations(document));
        let plume_evaluator = crate::plume::RigidPlumeEvaluator::maybe_new(
            document,
            engine_rack.liquid_plume_metadata(),
            engine_rack.engine_ids(),
            engine_rack.mount_points_body(),
        )?;
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
                    body_to_eci: state.orientation.q,
                    aerothermal_electron_number_density_m3:
                        initial_aerothermal_electron_number_density_m3,
                })
            })
            .transpose()?;
        if let Some(runtime) = &landing_gear_runtime {
            let active = breakdown_vehicle.model_active(
                kernel.current_phase().map(openbmp_sim::PhaseId::value),
                "landing_gear",
            );
            accept_landing_gear_state(
                runtime,
                kernel.current_state(),
                kernel.primary_rigid_body(),
                active,
            )
            .map_err(openbmp_sim::SimulationError::from)?;
            for separated in kernel.separated_rigid_bodies() {
                accept_landing_gear_state(runtime, &separated.state, Some(separated.body), active)
                    .map_err(openbmp_sim::SimulationError::from)?;
            }
            // Initial preloading is part of the initial condition, not work
            // performed during the simulated interval.
            drop(runtime.take_samples()?);
        }
        if let Some(contact) = &contact_evaluator {
            let state = kernel.current_state();
            contact
                .set_active(
                    breakdown_vehicle.model_active(
                        kernel.current_phase().map(openbmp_sim::PhaseId::value),
                        "contact",
                    ),
                    state.time,
                )
                .map_err(openbmp_sim::SimulationError::from)?;
            contact
                .commit_accepted(state.position.vector, state.velocity.vector, state.time)
                .map_err(openbmp_sim::SimulationError::from)?;
        }
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
            landing_gear_runtime.as_ref(),
            landing_gear_accumulator.as_mut(),
            fc_bridge.as_ref(),
            initial_comm_observation.as_ref(),
            acoustics_report.as_ref(),
            countdown.as_ref(),
            &structural_rack,
            &articulated_gimbal_shadows,
            &[],
            &mut mission_region_trace,
            &initial_snapshot,
        )?;
        if let Some(monitor) = &mut afts_monitor {
            monitor.observe_rigid_body(
                kernel.current_step().value(),
                kernel.current_time().as_seconds(),
                kernel.current_state(),
            )?;
        }
        let realtime_pacer = crate::rt::RunnerRealtimePacer::from_document(document)?;
        Ok(Self {
            effector_rack,
            engine_rack,
            tank_rack,
            propellant_budget,
            feed_network_rack,
            recovery_rack,
            separated_attitude_targets,
            separated_landing_controllers,
            wind_rack,
            structural_rack,
            upstream_uq,
            frame,
            aerothermal_driver,
            mass_model,
            separation_specs,
            stack_bodies,
            kernel_step_s,
            separated_ground_radius_m,
            kernel,
            channel_set,
            contact_evaluator,
            contact_accumulator,
            landing_gear_runtime,
            landing_gear_accumulator,
            afts_monitor,
            comm_accumulator,
            acoustics_report,
            structural_acoustics,
            structural_events,
            pogo,
            csi,
            countdown,
            geocentric_surface_radius_m,
            breakdown_atmosphere,
            breakdown_vehicle,
            moment_breakdown_vehicle,
            primary_multibody_shadow,
            separated_multibody_shadows: BTreeMap::new(),
            articulated_gimbal_shadows,
            articulated_gimbal_root_authority,
            multibody_moment_model,
            table,
            deck_bindings,
            direct_torque_present,
            fc_bridge,
            mission_region_trace,
            plume_evaluator,
            pending_effector_events: Vec::new(),
            pending_engine_events: Vec::new(),
            pending_recovery_events: Vec::new(),
            realtime_pacer,
            wind_override_ned_m_s: None,
            runtime_malfunction_seq: 0,
        })
    }

    /// Whether the kernel has reported a stop reason. Once `true`,
    /// further [`Self::step_once`] calls are rejected by the kernel.
    pub(crate) fn is_finished(&self) -> bool {
        self.kernel.stop_reason().is_some()
    }

    /// Advance the run by exactly one kernel tick — the body of the
    /// one-shot [`run`] loop, verbatim, over session-owned state.
    ///
    /// `document` must be the same scenario document the session was
    /// prepared from; [`crate::Session`] owns both and guarantees this.
    ///
    /// # Errors
    ///
    /// Returns the same errors as the one-shot [`run`] loop.
    #[allow(clippy::too_many_lines)] // mirrors the one-shot loop verbatim
    pub(crate) fn step_once(
        &mut self,
        document: &ScenarioDocument,
        monitor: Option<&mut (dyn crate::sil::SilMonitor + '_)>,
    ) -> Result<(), RunnerError> {
        self.realtime_pacer.wait_next_frame();
        self.realtime_pacer.begin_frame_execution();
        self.effector_rack
            .apply_overrides(&self.pending_effector_events)?;
        if !self.engine_rack.is_empty() {
            self.engine_rack
                .set_retired_bodies(retired_separated_body_ids(
                    self.kernel.separated_rigid_bodies(),
                ));
            self.engine_rack
                .apply_commands(&self.pending_engine_events)?;
        }
        let specific_force_truth = if self
            .fc_bridge
            .as_ref()
            .is_some_and(crate::fc_bridge::FcBridge::uses_force_accumulator_truth)
        {
            Some(rigid_body_specific_force_truth(
                &self.kernel,
                &self.breakdown_vehicle,
                &self.moment_breakdown_vehicle,
                &self.mass_model,
                self.structural_rack.reaction_moment_body_n_m(),
            )?)
        } else {
            None
        };
        let comm_link_effects = self
            .comm_accumulator
            .as_mut()
            .map(|accumulator| {
                accumulator.bridge_packet_effects(document.time.seed, self.kernel.current_step())
            })
            .transpose()?
            .flatten();
        if let Some(bridge) = &mut self.fc_bridge {
            let environment = self.kernel.current_environment_sample()?;
            let gravity = environment.gravity_eci_m_s2;
            let propellant_state = crate::fc_bridge::propellant_state_from_tanks(
                self.kernel.current_time(),
                &self.tank_rack.propellant_tank_states(document),
            );
            bridge.set_comm_link_effects(comm_link_effects);
            bridge.tick_rigid_body(
                self.kernel.current_state(),
                self.kernel.current_step(),
                &environment,
                gravity,
                specific_force_truth,
                propellant_state,
                &mut self.effector_rack,
                &mut self.engine_rack,
                // Bending slope-rate pickup from the previous tick's modal
                // state — contemporaneous with the body rate read above
                // (one-step lag, mirroring the slosh rack). Zero when rigid.
                self.structural_rack.gyro_pickup_rad_s(),
                self.structural_rack.accel_pickup_m_s2(),
                monitor,
            )?;
            if document.flight_controller_owns_mission_state() {
                // Forward the FC commander's published mission state into
                // the kernel's external view. Kernel-directed scenarios
                // intentionally keep the kernel event stream authoritative.
                if let Some(state_id) = bridge.latest_mission_state_id() {
                    self.kernel
                        .set_external_mission_state(Some(openbmp_sim::PhaseId::new(state_id)));
                }
                for fired in bridge.drain_mission_actions() {
                    self.kernel.record_external_mission_fired(fired);
                }
            }
        }
        let propellant_safing_command = self
            .fc_bridge
            .as_ref()
            .and_then(crate::fc_bridge::FcBridge::latest_propellant_safing_command);
        if !self.tank_rack.is_empty() {
            self.tank_rack
                .apply_propellant_safing_command(propellant_safing_command);
        }
        if !self.separated_attitude_targets.is_empty() {
            self.separated_attitude_targets.apply(
                self.kernel.separated_rigid_bodies(),
                self.kernel.current_time(),
                &mut self.effector_rack,
            )?;
        }
        if !self.separated_landing_controllers.is_empty() {
            self.separated_landing_controllers.apply(
                self.kernel.separated_rigid_bodies(),
                self.kernel.current_time(),
                self.separated_ground_radius_m,
                &mut self.engine_rack,
            )?;
        }
        if let Some(propellant_budget) = &self.propellant_budget {
            let mut report = propellant_budget
                .evaluate(
                    &self.engine_rack.propulsion_snapshot_map(),
                    &self.tank_rack.propellant_tank_states(document),
                )
                .map_err(|err| RunnerError::Engine {
                    field: "vehicle.assembly.engines[*].propellant".to_owned(),
                    reason: err.to_string(),
                })?;
            self.feed_network_rack
                .apply_propellant_safing_command(propellant_safing_command);
            self.feed_network_rack.apply_to_report(
                &mut report,
                self.kernel_step_s,
                self.kernel.current_step(),
            )?;
            crate::propulsion::apply_propellant_safing_to_report(
                &mut report,
                propellant_safing_command,
            );
            self.tank_rack
                .set_propellant_budget_drain_rates(report.tank_drain_rates_kg_per_s.clone());
            self.engine_rack.apply_propellant_budget(&report)?;
        }
        if !self.effector_rack.is_empty() {
            let tank_states = self.tank_rack.propellant_tank_states(document);
            self.effector_rack
                .step_with_tanks(self.kernel.current_time(), &tank_states)?;
        }
        if !self.tank_rack.is_empty() {
            self.tank_rack
                .set_rcs_feed_drain_rates(self.effector_rack.rcs_feed_drain_rates());
        }
        if !self.engine_rack.is_empty() {
            let cavitation_events = self.feed_network_rack.cavitation_events();
            self.engine_rack
                .apply_cavitation_faults(&cavitation_events)?;
            self.engine_rack
                .apply_scheduled_faults(self.kernel.current_step())?;
            self.engine_rack.step()?;
        }
        crate::acoustics::observe_source_engine_runtime_sample(
            &mut self.acoustics_report,
            self.kernel.current_step().value(),
            self.kernel.current_time().as_seconds(),
            &self.engine_rack.propulsion_snapshot_map(),
        )?;
        // Advance tanks using prior-step cached drivers.
        // The drivers are updated post-step from the new rigid-body
        // state's angular_velocity (omega_body) and a finite-
        // difference body-frame acceleration; the first step uses
        // zeros (initialised by `TankRack::build`).
        if !self.tank_rack.is_empty() {
            if self.tank_rack.uses_safe_dump_ambient_backpressure() {
                let environment = self.kernel.current_environment_sample()?;
                self.tank_rack
                    .set_safe_dump_ambient_backpressure_pa(environment.atmosphere_pressure_pa)?;
            }
            self.tank_rack.step()?;
        }
        // Advance the bending mode against the prior-step body lateral accel
        // (one-step lag, like the slosh rack) plus declared structural event
        // forces at the current kernel time; refreshes the gyro pickup the
        // next bridge tick reads.
        if !self.structural_rack.is_inactive() {
            self.structural_rack
                .step_at(self.kernel.current_time().as_seconds())?;
        }
        // Drain pending deploy/stow events and step the
        // recovery rack (no-op step for the instantaneous-
        // deploy models).
        if !self.recovery_rack.is_empty() {
            self.recovery_rack
                .apply_deploys(&self.pending_recovery_events)?;
            self.recovery_rack.step(self.kernel_step_s)?;
        }
        if !self.deck_bindings.is_empty() || self.direct_torque_present {
            let rack_snapshot = self.effector_rack.snapshot();
            let snapshot_map = build_effector_actual_snapshot_map(
                document,
                &self.deck_bindings,
                &rack_snapshot,
                self.direct_torque_present,
            )?;
            self.kernel.set_effector_actuals(snapshot_map);
        }
        if !self.engine_rack.is_empty() {
            self.kernel
                .set_engine_snapshot(self.engine_rack.snapshot_map());
        }
        if !self.tank_rack.is_empty() {
            self.kernel.set_tank_snapshot(self.tank_rack.snapshot_map());
        }
        if !self.wind_rack.is_inactive() {
            self.wind_rack.advance(self.kernel.current_step());
            let s = self.kernel.current_state();
            let wind = self.wind_rack.sample(s.position, &self.frame, s.time)?;
            // A session wind override replaces the scenario wind for this
            // tick; `None` (the default, and the only state a one-shot
            // [`run`] can be in) keeps the rack sample byte-identical.
            let wind = self.wind_override_ned_m_s.unwrap_or(wind);
            self.kernel.set_wind_sample(wind);
        } else if let Some(wind) = self.wind_override_ned_m_s {
            // Scenarios without a `[wind]` block never push a sample, so
            // an armed override pushes one explicitly; clearing the
            // override pushes zero wind again (see `set_wind_override`).
            self.kernel.set_wind_sample(wind);
        }
        if !self.recovery_rack.is_empty() {
            let environment = self.kernel.current_environment_sample()?;
            let context = crate::recovery::recovery_opening_load_context_with_mass_and_attitude(
                self.kernel.current_state().velocity.vector,
                &environment,
                self.kernel.current_state().mass_props.mass_kg(),
                self.kernel.current_state().orientation.q,
            )?;
            self.kernel.set_recovery_snapshot(
                self.recovery_rack
                    .snapshot_map_with_opening_load_context(context)?,
            );
        }
        // Feed the bending mode's reaction moment to the rigid-body torque for
        // this step (held across the RK4 stages). Zero when no flex mode.
        if !self.structural_rack.is_inactive() {
            self.kernel
                .set_bending_reaction_moment(self.structural_rack.reaction_moment_body_n_m());
        }
        let prev_velocity_eci = self.kernel.current_state().velocity.vector;
        let prev_orientation = self.kernel.current_state().orientation.q;
        let gear_checkpoint = self
            .landing_gear_runtime
            .as_ref()
            .map(crate::landing_gear::LandingGearRuntime::checkpoint)
            .transpose()
            .map_err(openbmp_sim::SimulationError::from)?;
        let contact_checkpoint = self
            .contact_evaluator
            .as_ref()
            .map(crate::contact::ContactDiagnosticsEvaluator::snapshot)
            .transpose()
            .map_err(openbmp_sim::SimulationError::from)?;
        let primary_body = self.kernel.primary_rigid_body();
        let replace_primary = primary_root_free_flyer_authority_enabled(document)
            || articulated_gimbal_root_authority_enabled(document);
        let replace_separated = separated_root_free_flyer_authority_enabled(document);
        let deferred_separated_bodies: Vec<_> = if replace_separated {
            self.kernel
                .separated_rigid_bodies()
                .iter()
                .filter(|body| body.propagating)
                .map(|body| body.body)
                .collect()
        } else {
            Vec::new()
        };
        let interval_phase = self.kernel.current_phase().map(openbmp_sim::PhaseId::value);
        let gear_active = self
            .breakdown_vehicle
            .model_active(interval_phase, "landing_gear");
        let contact_active = self
            .breakdown_vehicle
            .model_active(interval_phase, "contact");
        let step_result = (|| -> Result<(), RunnerError> {
            if let Some(runtime) = &self.landing_gear_runtime {
                runtime
                    .begin_interval(self.kernel.current_state(), primary_body, gear_active)
                    .map_err(openbmp_sim::SimulationError::from)?;
                for body in self.kernel.separated_rigid_bodies() {
                    runtime
                        .begin_interval(&body.state, Some(body.body), gear_active)
                        .map_err(openbmp_sim::SimulationError::from)?;
                }
            }
            if let Some(contact) = &self.contact_evaluator {
                let state = self.kernel.current_state();
                contact
                    .set_active(contact_active, state.time)
                    .map_err(openbmp_sim::SimulationError::from)?;
                contact
                    .commit_accepted(state.position.vector, state.velocity.vector, state.time)
                    .map_err(openbmp_sim::SimulationError::from)?;
            }
            // Forecasts use the same phase-selected constitutive state as
            // the kernel, including any phase change from the controller.
            self.mirror_primary_multibody_root_step()?;
            self.mirror_separated_multibody_root_steps()?;
            let articulated_gimbal_root_authority =
                articulated_gimbal_root_authority_enabled(document);
            self.mirror_articulated_gimbal_steps(articulated_gimbal_root_authority)?;
            self.mirror_articulated_gimbal_root_authority_step(articulated_gimbal_root_authority)?;
            self.kernel.step_with_accepted_steps(|state, active_body| {
                let replacement_pending = if active_body == primary_body {
                    replace_primary
                } else {
                    replace_separated
                };
                // These bodies are ultimately advanced by the pure RK4
                // forecast. The provisional kernel path cannot own their
                // irreversible history, even when it compresses more deeply.
                if replacement_pending {
                    return Ok(());
                }
                if let Some(runtime) = &self.landing_gear_runtime {
                    accept_landing_gear_state(runtime, state, active_body, gear_active)?;
                }
                if active_body == primary_body
                    && let Some(contact) = &self.contact_evaluator
                {
                    contact.commit_accepted(
                        state.position.vector,
                        state.velocity.vector,
                        state.time,
                    )?;
                }
                Ok(())
            })?;
            self.apply_primary_root_free_flyer_authority(document)?;
            self.apply_separated_root_free_flyer_authority(document)?;
            self.apply_articulated_gimbal_root_authority(document)?;
            if replace_primary {
                let state = self.kernel.current_state();
                if let Some(runtime) = &self.landing_gear_runtime {
                    accept_landing_gear_state(runtime, state, primary_body, gear_active)
                        .map_err(openbmp_sim::SimulationError::from)?;
                }
                if let Some(contact) = &self.contact_evaluator {
                    contact
                        .commit_accepted(state.position.vector, state.velocity.vector, state.time)
                        .map_err(openbmp_sim::SimulationError::from)?;
                }
            }
            if let Some(runtime) = &self.landing_gear_runtime {
                for body in self.kernel.separated_rigid_bodies() {
                    if deferred_separated_bodies.contains(&body.body) {
                        // Include lanes retired at this endpoint: their last
                        // actual state must still update diagnostics/history.
                        accept_landing_gear_state(
                            runtime,
                            &body.state,
                            Some(body.body),
                            gear_active,
                        )
                        .map_err(openbmp_sim::SimulationError::from)?;
                    }
                }
            }
            // Events can change the force set at this endpoint. Refresh its
            // diagnostics and the next interval's power baseline without
            // assigning any work from the completed interval to the new set.
            let endpoint_phase = self.kernel.current_phase().map(openbmp_sim::PhaseId::value);
            if let Some(runtime) = &self.landing_gear_runtime {
                let active = self
                    .breakdown_vehicle
                    .model_active(endpoint_phase, "landing_gear");
                runtime
                    .begin_interval(self.kernel.current_state(), primary_body, active)
                    .map_err(openbmp_sim::SimulationError::from)?;
                for body in self.kernel.separated_rigid_bodies() {
                    runtime
                        .begin_interval(&body.state, Some(body.body), active)
                        .map_err(openbmp_sim::SimulationError::from)?;
                }
            }
            if let Some(contact) = &self.contact_evaluator {
                let state = self.kernel.current_state();
                contact
                    .retime_accepted(self.kernel.current_time())
                    .map_err(openbmp_sim::SimulationError::from)?;
                contact
                    .set_active(
                        self.breakdown_vehicle
                            .model_active(endpoint_phase, "contact"),
                        state.time,
                    )
                    .map_err(openbmp_sim::SimulationError::from)?;
                contact
                    .commit_accepted(state.position.vector, state.velocity.vector, state.time)
                    .map_err(openbmp_sim::SimulationError::from)?;
            }
            Ok(())
        })();
        if let Err(error) = step_result {
            if let (Some(runtime), Some(checkpoint)) = (&self.landing_gear_runtime, gear_checkpoint)
            {
                runtime
                    .restore(checkpoint)
                    .map_err(openbmp_sim::SimulationError::from)?;
            }
            if let (Some(contact), Some(checkpoint)) = (&self.contact_evaluator, contact_checkpoint)
            {
                contact
                    .restore(checkpoint)
                    .map_err(openbmp_sim::SimulationError::from)?;
            }
            return Err(error);
        }
        // Refresh tank-rack drivers from the post-step
        // rigid-body state. `accel_body_m_s2` is finite-differenced
        // from the velocity change rotated into the prior-step body
        // frame; `omega_body_rad_s` is read directly from the new
        // state. Slosh state on the next tick uses these drivers
        // (one-step lag, see TankRack module docs).
        if !self.tank_rack.is_empty() || !self.structural_rack.is_inactive() {
            let dt_s = self.kernel_step_s;
            let new_state = self.kernel.current_state();
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
            let omega_body = new_state.angular_velocity.vector;
            // Propellant slosh and structural bending are driven by the
            // SPECIFIC FORCE (proper, non-gravitational acceleration — what the
            // tank / structure feels, and what an accelerometer reads), NOT the
            // total kinematic acceleration. `accel_eci` (finite-differenced
            // from the integrated velocity) includes gravity; feeding it as
            // lateral forcing makes the body-frame gravity component
            // spuriously drive internal modes. Subtract the scenario's own
            // gravitational acceleration to recover the specific force: ~0 in
            // coast (no spurious drive), ~thrust/m under power.
            let gravity_eci = self.kernel.current_environment_sample()?.gravity_eci_m_s2;
            let specific_accel_body = inverse_orientation * (accel_eci - gravity_eci);
            if !self.tank_rack.is_empty() {
                self.tank_rack
                    .update_drivers(specific_accel_body, omega_body);
            }
            if !self.structural_rack.is_inactive() {
                self.structural_rack.update_drivers(specific_accel_body);
            }
        }
        let mission_fired = self.kernel.drain_mission_fired_events();
        let script_fired = self.kernel.drain_script_fired_events();
        crate::acoustics::observe_source_engine_ignitions(
            &mut self.acoustics_report,
            &script_fired,
        );
        let jettison_applied = apply_jettison_events(
            &mut self.kernel,
            &script_fired,
            &self.separation_specs,
            &self.mass_model,
            &mut self.stack_bodies,
            welded_release_jettison_authority_enabled(document),
        )?;
        if jettison_applied {
            self.mirror_separated_multibody_root_steps()?;
        }
        if !self.engine_rack.is_empty() {
            self.engine_rack
                .set_retired_bodies(retired_separated_body_ids(
                    self.kernel.separated_rigid_bodies(),
                ));
            self.engine_rack.shutdown_retired_body_engines()?;
            self.kernel
                .set_engine_snapshot(self.engine_rack.snapshot_map());
        }
        if let Some(driver) = &mut self.aerothermal_driver {
            let environment = self.kernel.current_environment_sample()?;
            driver.evaluate_rigid_body(
                self.kernel.current_state(),
                &environment,
                self.kernel_step_s,
            )?;
        }
        let snapshot = self.effector_rack.snapshot();
        let aerothermal_electron_number_density_m3 = self
            .aerothermal_driver
            .as_ref()
            .map(|driver| driver.output().electron_number_density_m3);
        let comm_observation = self
            .comm_accumulator
            .as_mut()
            .map(|accumulator| {
                let state = self.kernel.current_state();
                accumulator.observe_eci(crate::comm::CommEciObservationInput {
                    frame: &self.frame,
                    step: self.kernel.current_step(),
                    time: state.time,
                    position_eci: state.position,
                    velocity_eci: state.velocity,
                    body_to_eci: state.orientation.q,
                    aerothermal_electron_number_density_m3,
                })
            })
            .transpose()?;
        record_step(
            document,
            &mut self.table,
            &self.kernel,
            &self.channel_set,
            &self.breakdown_vehicle,
            self.contact_evaluator.as_ref(),
            self.contact_accumulator.as_mut(),
            self.breakdown_atmosphere.as_ref(),
            self.plume_evaluator.as_ref(),
            self.geocentric_surface_radius_m,
            self.aerothermal_driver
                .as_ref()
                .map(|driver| driver.output()),
            self.landing_gear_runtime.as_ref(),
            self.landing_gear_accumulator.as_mut(),
            self.fc_bridge.as_ref(),
            comm_observation.as_ref(),
            self.acoustics_report.as_ref(),
            self.countdown.as_ref(),
            &self.structural_rack,
            &self.articulated_gimbal_shadows,
            &mission_fired,
            &mut self.mission_region_trace,
            &snapshot,
        )?;
        if let Some(monitor) = &mut self.afts_monitor {
            monitor.observe_rigid_body(
                self.kernel.current_step().value(),
                self.kernel.current_time().as_seconds(),
                self.kernel.current_state(),
            )?;
        }
        // Partition typed script-action fired queue.
        self.pending_engine_events = script_fired
            .iter()
            .filter(|e| matches!(e.action, ScenarioScriptAction::EngineCommand { .. }))
            .cloned()
            .collect();
        self.pending_recovery_events = script_fired
            .iter()
            .filter(|e| matches!(e.action, ScenarioScriptAction::DeployRecovery { .. }))
            .cloned()
            .collect();
        self.pending_effector_events = script_fired
            .iter()
            .filter(|e| matches!(e.action, ScenarioScriptAction::EffectorOverride { .. }))
            .cloned()
            .collect();
        self.realtime_pacer.finish_frame_execution();
        Ok(())
    }

    fn mirror_primary_multibody_root_step(&mut self) -> Result<(), RunnerError> {
        let Some(existing_shadow) = self.primary_multibody_shadow.as_ref() else {
            return Ok(());
        };
        if let Some(active_body) = self.kernel.primary_rigid_body()
            && active_body != existing_shadow.seed.body
        {
            return Ok(());
        }
        let body = self
            .kernel
            .primary_rigid_body()
            .unwrap_or(existing_shadow.seed.body);
        let current_state = *self.kernel.current_state();
        let mass_properties = if self.stack_bodies.len() == 1 && self.stack_bodies.contains(&body) {
            current_state.mass_props
        } else {
            self.mass_model
                .mass_properties_at(MassContext {
                    time: current_state.time,
                    active_body: Some(body),
                    engine_snapshot: EngineSnapshotView::new(self.kernel.engine_snapshot()),
                    tank_snapshot: TankSnapshotView::new(self.kernel.tank_snapshot()),
                    recovery_snapshot: openbmp_sim::RecoverySnapshotView::new(
                        self.kernel.recovery_snapshot(),
                    ),
                })
                .map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!("per-step primary root multibody mass properties failed: {err}"),
                })?
        };
        let rigid_state = RigidBodyState::new(
            current_state.time,
            current_state.position,
            current_state.velocity,
            current_state.orientation,
            current_state.angular_velocity,
            mass_properties,
        );
        let seed = root_free_flyer_multibody_seed_from_rigid_state(body, rigid_state)?;
        let environment = self.kernel.current_environment_sample().map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("per-step primary root multibody environment sample failed: {err}"),
            }
        })?;
        let phase_id = self.kernel.current_phase().map(openbmp_sim::PhaseId::value);
        let moment_model = self.multibody_moment_model.as_ref().ok_or_else(|| {
            RunnerError::UnsupportedScenario {
                what: "internal invariant: primary multibody moment model missing".to_owned(),
            }
        })?;
        let derivative = {
            root_free_flyer_multibody_derivative_from_runner_models(
                &seed,
                &self.breakdown_vehicle,
                moment_model,
                RootFreeFlyerLoadViews {
                    phase_id,
                    environment: &environment,
                    effector_actuals: self.kernel.effector_actuals(),
                    engine_snapshot: self.kernel.engine_snapshot(),
                    tank_snapshot: self.kernel.tank_snapshot(),
                    recovery_snapshot: self.kernel.recovery_snapshot(),
                },
            )?
        };
        let forecast = root_free_flyer_multibody_rk4_forecast_from_runner_models(
            &seed,
            &self.breakdown_vehicle,
            moment_model,
            &self.mass_model,
            self.kernel_step_s,
            RootFreeFlyerHeldLoadViews {
                phase_id,
                effector_actuals: self.kernel.effector_actuals(),
                engine_snapshot: self.kernel.engine_snapshot(),
                tank_snapshot: self.kernel.tank_snapshot(),
                recovery_snapshot: self.kernel.recovery_snapshot(),
            },
            |rigid_state| {
                self.kernel
                    .environment_sample_for_rigid_state(rigid_state)
                    .map_err(|err| RunnerError::UnsupportedScenario {
                        what: format!(
                            "primary root multibody RK4 forecast environment sample failed: {err}"
                        ),
                    })
            },
        )?;
        let shadow = self.primary_multibody_shadow.as_mut().ok_or_else(|| {
            RunnerError::UnsupportedScenario {
                what: "internal invariant: primary multibody shadow missing".to_owned(),
            }
        })?;
        shadow.seed = seed;
        shadow.last_derivative = Some(derivative);
        shadow.last_rk4_forecast = Some(forecast);
        shadow.last_authoritative_state = None;
        debug_assert_eq!(
            shadow
                .last_derivative
                .as_ref()
                .map(|derivative| derivative.qd_dot().len()),
            Some(shadow.seed.sim_state.qd().len())
        );
        Ok(())
    }

    fn mirror_separated_multibody_root_steps(&mut self) -> Result<(), RunnerError> {
        let Some(moment_model) = self.multibody_moment_model.as_ref() else {
            return Ok(());
        };
        let phase_id = self.kernel.current_phase().map(openbmp_sim::PhaseId::value);
        let mut active_bodies = BTreeSet::new();
        let mut updates = Vec::new();
        for separated in self.kernel.separated_rigid_bodies() {
            if !separated.propagating {
                continue;
            }
            active_bodies.insert(separated.body);
            let seed =
                root_free_flyer_multibody_seed_from_rigid_state(separated.body, separated.state)?;
            let environment = self
                .kernel
                .environment_sample_for_rigid_state(&separated.state)
                .map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!(
                        "per-step separated root multibody environment sample failed: {err}"
                    ),
                })?;
            let derivative = root_free_flyer_multibody_derivative_from_runner_models(
                &seed,
                &self.breakdown_vehicle,
                moment_model,
                RootFreeFlyerLoadViews {
                    phase_id,
                    environment: &environment,
                    effector_actuals: self.kernel.effector_actuals(),
                    engine_snapshot: self.kernel.engine_snapshot(),
                    tank_snapshot: self.kernel.tank_snapshot(),
                    recovery_snapshot: self.kernel.recovery_snapshot(),
                },
            )?;
            let forecast = root_free_flyer_multibody_rk4_forecast_from_runner_models(
                &seed,
                &self.breakdown_vehicle,
                moment_model,
                &self.mass_model,
                self.kernel_step_s,
                RootFreeFlyerHeldLoadViews {
                    phase_id,
                    effector_actuals: self.kernel.effector_actuals(),
                    engine_snapshot: self.kernel.engine_snapshot(),
                    tank_snapshot: self.kernel.tank_snapshot(),
                    recovery_snapshot: self.kernel.recovery_snapshot(),
                },
                |rigid_state| {
                    self.kernel
                        .environment_sample_for_rigid_state(rigid_state)
                        .map_err(|err| RunnerError::UnsupportedScenario {
                            what: format!(
                                "separated root multibody RK4 forecast environment sample failed: {err}"
                            ),
                        })
                },
            )?;
            updates.push((separated.body, seed, derivative, forecast));
        }

        self.separated_multibody_shadows
            .retain(|body, _| active_bodies.contains(body));
        for (body, seed, derivative, forecast) in updates {
            self.separated_multibody_shadows.insert(
                body,
                RootFreeFlyerMultibodyShadow {
                    seed,
                    last_derivative: Some(derivative),
                    last_rk4_forecast: Some(forecast),
                    last_authoritative_state: None,
                },
            );
        }
        Ok(())
    }

    fn apply_primary_root_free_flyer_authority(
        &mut self,
        document: &ScenarioDocument,
    ) -> Result<(), RunnerError> {
        if !primary_root_free_flyer_authority_enabled(document) {
            return Ok(());
        }
        if !self.kernel.separated_rigid_bodies().is_empty() {
            return Err(RunnerError::UnsupportedScenario {
                what:
                    "primary_root_free_flyer propagation authority does not support separated lanes"
                        .to_owned(),
            });
        }
        let replacement = self
            .primary_multibody_shadow
            .as_ref()
            .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
            .map(|forecast| forecast.rigid_state)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "primary_root_free_flyer propagation authority requires a recorded primary root multibody forecast"
                    .to_owned(),
            })?;
        let replacement = multibody_authority_replacement_at_kernel_time(
            replacement,
            self.kernel.current_state().time,
            "primary_root_free_flyer",
        )?;
        self.kernel.replace_current_rigid_state(replacement)?;
        if let Some(shadow) = self.primary_multibody_shadow.as_mut() {
            shadow.last_authoritative_state = Some(replacement);
        }
        Ok(())
    }

    fn apply_separated_root_free_flyer_authority(
        &mut self,
        document: &ScenarioDocument,
    ) -> Result<(), RunnerError> {
        if !separated_root_free_flyer_authority_enabled(document) {
            return Ok(());
        }
        let current_time = self.kernel.current_state().time;
        let replacements: Vec<(BodyId, RigidBodyState)> = self
            .kernel
            .separated_rigid_bodies()
            .iter()
            .filter(|separated| separated.propagating)
            .map(|separated| {
                let replacement = self
                    .separated_multibody_shadows
                    .get(&separated.body)
                    .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
                    .map(|forecast| forecast.rigid_state)
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: format!(
                            "separated_root_free_flyer propagation authority requires a recorded root multibody forecast for body {}",
                            separated.body.value()
                        ),
                    })?;
                let replacement = multibody_authority_replacement_at_kernel_time(
                    replacement,
                    current_time,
                    "separated_root_free_flyer",
                )?;
                Ok((separated.body, replacement))
            })
            .collect::<Result<_, RunnerError>>()?;
        for (body, replacement) in replacements {
            self.kernel
                .replace_separated_rigid_body_state(body, replacement)?;
            if let Some(shadow) = self.separated_multibody_shadows.get_mut(&body) {
                shadow.last_authoritative_state = Some(replacement);
            }
        }
        Ok(())
    }

    fn apply_articulated_gimbal_root_authority(
        &mut self,
        document: &ScenarioDocument,
    ) -> Result<(), RunnerError> {
        if !articulated_gimbal_root_authority_enabled(document) {
            return Ok(());
        }
        if !self.kernel.separated_rigid_bodies().is_empty() {
            return Err(RunnerError::UnsupportedScenario {
                what:
                    "articulated_gimbal_root propagation authority does not support separated lanes"
                        .to_owned(),
            });
        }
        if let Some(shadow) = self.articulated_gimbal_root_authority.as_mut() {
            let replacement = shadow
                .last_root_rk4_forecast
                .as_ref()
                .copied()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "articulated_gimbal_root propagation authority requires a recorded coupled articulated gimbal root forecast"
                        .to_owned(),
                })?;
            let replacement = multibody_authority_replacement_at_kernel_time(
                replacement,
                self.kernel.current_state().time,
                "articulated_gimbal_root",
            )?;
            self.kernel.replace_current_rigid_state(replacement)?;
            shadow.last_authoritative_root_state = Some(replacement);
            return Ok(());
        }
        if self.articulated_gimbal_shadows.len() != 1 {
            return Err(RunnerError::UnsupportedScenario {
                what: "articulated_gimbal_root propagation authority requires exactly one articulated gimbal shadow"
                    .to_owned(),
            });
        }
        let engine = self
            .articulated_gimbal_shadows
            .keys()
            .next()
            .copied()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "articulated_gimbal_root propagation authority requires exactly one articulated gimbal shadow"
                    .to_owned(),
            })?;
        let replacement = self
            .articulated_gimbal_shadows
            .get(&engine)
            .and_then(|shadow| shadow.last_root_rk4_forecast.as_ref())
            .copied()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "articulated_gimbal_root propagation authority requires a recorded articulated gimbal root forecast"
                    .to_owned(),
            })?;
        let replacement = multibody_authority_replacement_at_kernel_time(
            replacement,
            self.kernel.current_state().time,
            "articulated_gimbal_root",
        )?;
        self.kernel.replace_current_rigid_state(replacement)?;
        if let Some(shadow) = self.articulated_gimbal_shadows.get_mut(&engine) {
            shadow.last_authoritative_root_state = Some(replacement);
        }
        Ok(())
    }

    fn mirror_articulated_gimbal_root_authority_step(
        &mut self,
        carry_authoritative_internal_state: bool,
    ) -> Result<(), RunnerError> {
        let Some(previous_shadow) = self.articulated_gimbal_root_authority.as_ref() else {
            return Ok(());
        };
        if !carry_authoritative_internal_state {
            return Ok(());
        }
        let previous_seed = previous_shadow.seed.clone();
        let previous_forecast = previous_shadow.last_rk4_forecast.clone();
        let previous_live_joint_states = previous_shadow.last_live_joint_states.clone();
        let had_authoritative_handoff = previous_shadow.last_authoritative_root_state.is_some();
        let current_state = *self.kernel.current_state();
        let primary_body = self.kernel.primary_rigid_body();
        if let Some(active_body) = primary_body
            && active_body != previous_seed.body
        {
            return Ok(());
        }
        let engine_snapshot = self.kernel.engine_snapshot();
        let tank_snapshot = self.kernel.tank_snapshot();
        let use_current_mass_properties =
            self.stack_bodies.len() == 1 && self.stack_bodies.contains(&previous_seed.body);
        let mass_properties = if use_current_mass_properties {
            current_state.mass_props
        } else {
            let context = MassContext {
                time: current_state.time,
                active_body: Some(previous_seed.body),
                engine_snapshot: EngineSnapshotView::new(engine_snapshot),
                tank_snapshot: TankSnapshotView::new(tank_snapshot),
                recovery_snapshot: openbmp_sim::RecoverySnapshotView::new(
                    self.kernel.recovery_snapshot(),
                ),
            };
            self.mass_model.mass_properties_at(context).map_err(|err| {
                RunnerError::UnsupportedScenario {
                    what: format!(
                        "per-step articulated gimbal root-authority mass properties failed: {err}"
                    ),
                }
            })?
        };
        let parent_state = RigidBodyState::new(
            current_state.time,
            current_state.position,
            current_state.velocity,
            current_state.orientation,
            current_state.angular_velocity,
            mass_properties,
        );
        let live_joint_states = articulated_gimbal_root_authority_live_joint_states_from_previous(
            &previous_seed,
            &previous_live_joint_states,
            engine_snapshot,
            self.kernel_step_s,
        )?;
        let live_joint_changed = articulated_gimbal_joint_states_angles_changed(
            &previous_live_joint_states,
            &live_joint_states,
        )?;
        let seed = if had_authoritative_handoff && !live_joint_changed {
            let forecast = previous_forecast.as_ref().ok_or_else(|| {
                RunnerError::UnsupportedScenario {
                    what: "articulated_gimbal_root propagation authority requires a coupled articulated forecast to carry internal state"
                        .to_owned(),
                }
            })?;
            if (forecast.sim_state.state().time.as_seconds() - parent_state.time.as_seconds()).abs()
                > 1.0e-12
            {
                return Err(RunnerError::UnsupportedScenario {
                    what: "articulated_gimbal_root coupled internal-state carry requires a same-time articulated forecast"
                        .to_owned(),
                });
            }
            articulated_gimbal_root_authority_seed_from_authoritative_forecast(
                forecast,
                &parent_state,
            )?
        } else {
            let joint_specs = articulated_gimbal_root_authority_joint_specs(&previous_seed);
            articulated_gimbal_root_authority_seed_from_joint_states(
                &joint_specs,
                &parent_state,
                &live_joint_states,
            )?
        };
        let derivative = articulated_gimbal_root_authority_derivative_from_engine_thrust(
            &seed,
            engine_snapshot,
        )?;
        let forecast = articulated_gimbal_root_authority_rk4_forecast_from_engine_thrust(
            &seed,
            engine_snapshot,
            self.kernel_step_s,
        )?;
        let root_forecast = articulated_gimbal_root_authority_rigid_state_from_seed(&forecast)?;
        if let Some(shadow) = self.articulated_gimbal_root_authority.as_mut() {
            shadow.last_live_joint_states = live_joint_states;
            shadow.seed = seed;
            shadow.last_derivative = Some(derivative);
            shadow.last_rk4_forecast = Some(forecast);
            shadow.last_root_rk4_forecast = Some(root_forecast);
            shadow.last_authoritative_root_state = None;
        }
        Ok(())
    }

    fn mirror_articulated_gimbal_steps(
        &mut self,
        carry_authoritative_internal_state: bool,
    ) -> Result<(), RunnerError> {
        if self.articulated_gimbal_shadows.is_empty() {
            return Ok(());
        }
        let current_state = *self.kernel.current_state();
        let primary_body = self.kernel.primary_rigid_body();
        let engine_snapshot = self.kernel.engine_snapshot();
        let tank_snapshot = self.kernel.tank_snapshot();
        let mut updates = Vec::with_capacity(self.articulated_gimbal_shadows.len());
        for shadow in self.articulated_gimbal_shadows.values_mut() {
            let body = shadow.seed.body;
            if let Some(active_body) = primary_body
                && active_body != body
            {
                continue;
            }
            let mass_properties =
                if self.stack_bodies.len() == 1 && self.stack_bodies.contains(&body) {
                    current_state.mass_props
                } else {
                    self.mass_model
                        .mass_properties_at(MassContext {
                            time: current_state.time,
                            active_body: Some(body),
                            engine_snapshot: EngineSnapshotView::new(engine_snapshot),
                            tank_snapshot: TankSnapshotView::new(tank_snapshot),
                            recovery_snapshot: openbmp_sim::RecoverySnapshotView::new(
                                self.kernel.recovery_snapshot(),
                            ),
                        })
                        .map_err(|err| RunnerError::UnsupportedScenario {
                            what: format!(
                                "per-step articulated gimbal body mass properties failed: {err}"
                            ),
                        })?
                };
            let parent_state = RigidBodyState::new(
                current_state.time,
                current_state.position,
                current_state.velocity,
                current_state.orientation,
                current_state.angular_velocity,
                mass_properties,
            );
            let live_joint_state = articulated_gimbal_joint_state_from_engine_thrust_for_joint(
                shadow.seed.engine,
                &shadow.seed.joint,
                &shadow.last_live_joint_state.angles_rad,
                engine_snapshot,
                self.kernel_step_s,
            )?;
            let live_joint_changed = articulated_gimbal_joint_state_angles_changed(
                &shadow.last_live_joint_state,
                &live_joint_state,
            )?;
            let seed = if carry_authoritative_internal_state
                && shadow.last_authoritative_root_state.is_some()
                && !live_joint_changed
            {
                let forecast =
                    shadow
                        .last_rk4_forecast
                        .as_ref()
                        .ok_or_else(|| RunnerError::UnsupportedScenario {
                            what: "articulated_gimbal_root propagation authority requires an articulated forecast to carry internal state"
                                .to_owned(),
                        })?;
                if (forecast.sim_state.state().time.as_seconds() - parent_state.time.as_seconds())
                    .abs()
                    > 1.0e-12
                {
                    return Err(RunnerError::UnsupportedScenario {
                        what: "articulated_gimbal_root internal-state carry requires a same-time articulated forecast"
                            .to_owned(),
                    });
                }
                articulated_gimbal_multibody_seed_from_authoritative_forecast(
                    forecast,
                    &parent_state,
                )?
            } else {
                let mut seed = articulated_gimbal_multibody_seed_from_config_with_joint_state(
                    body,
                    shadow.seed.engine,
                    &shadow.seed.joint,
                    shadow.seed.mechanism.clone(),
                    &parent_state,
                    &live_joint_state.angles_rad,
                    &live_joint_state.rates_rad_s,
                )?;
                seed.cfe_loops = shadow.seed.cfe_loops.clone();
                seed.cfe_gimbal_locks = shadow.seed.cfe_gimbal_locks.clone();
                seed.cfe_planar_four_bars = shadow.seed.cfe_planar_four_bars.clone();
                seed
            };
            let mut mechanism_latch_states = shadow.mechanism_latch_states.clone();
            let derivative_eval = articulated_gimbal_multibody_derivative_from_engine_thrust(
                &seed,
                engine_snapshot,
                self.kernel_step_s,
                &mut mechanism_latch_states,
            )?;
            let forecast = articulated_gimbal_multibody_rk4_forecast_from_engine_thrust(
                &seed,
                engine_snapshot,
                self.kernel_step_s,
                &mechanism_latch_states,
            )?;
            let root_forecast = articulated_gimbal_root_rigid_state_from_seed(&forecast)?;
            updates.push((
                seed.engine,
                seed,
                derivative_eval.derivative,
                derivative_eval.cfe_loop_solves,
                forecast,
                root_forecast,
                mechanism_latch_states,
                live_joint_state,
            ));
        }
        for (
            engine,
            seed,
            derivative,
            cfe_loop_solves,
            forecast,
            root_forecast,
            mechanism_latch_states,
            live_joint_state,
        ) in updates
        {
            if let Some(shadow) = self.articulated_gimbal_shadows.get_mut(&engine) {
                shadow.last_live_joint_state = live_joint_state;
                shadow.seed = seed;
                shadow.last_derivative = Some(derivative);
                shadow.last_cfe_loop_solves = cfe_loop_solves;
                shadow.last_rk4_forecast = Some(forecast);
                shadow.last_root_rk4_forecast = Some(root_forecast);
                shadow.last_authoritative_root_state = None;
                shadow.mechanism_latch_states = mechanism_latch_states;
            }
        }
        Ok(())
    }

    /// Consume the session and assemble the [`RunOutcome`] — the exact
    /// post-loop assembly the one-shot [`run`] performs.
    ///
    /// # Errors
    ///
    /// Returns the same errors as the one-shot [`run`] tail (actuator
    /// stream digest, contact, and landing-gear report assembly).
    pub(crate) fn finish(mut self) -> Result<RunOutcome, RunnerError> {
        let stop_reason = self
            .kernel
            .stop_reason()
            .cloned()
            .unwrap_or(StopReason::EndTime { reached_s: 0.0 });
        let actuator_stream = self
            .fc_bridge
            .as_ref()
            .map(crate::fc_bridge::FcBridge::actuator_stream_report)
            .transpose()?
            .flatten();
        let contact = self
            .contact_accumulator
            .take()
            .map(crate::contact::ContactRunAccumulator::finish)
            .transpose()?
            .flatten();
        let landing_gear = self
            .landing_gear_accumulator
            .take()
            .map(crate::landing_gear::LandingGearRunAccumulator::finish)
            .transpose()?
            .flatten();
        let comm = self
            .comm_accumulator
            .map(crate::comm::CommRunAccumulator::finish)
            .transpose()?;
        Ok(RunOutcome {
            final_step: self.kernel.current_step().value(),
            final_time_s: self.kernel.current_time().as_seconds(),
            stop_reason,
            table: self.table,
            realtime: self.realtime_pacer.finish(),
            actuator_stream,
            contact,
            landing_gear,
            afts: self
                .afts_monitor
                .map(crate::afts::RunnerAftsMonitor::finish),
            comm,
            acoustics: self.acoustics_report,
            structural_acoustics: self.structural_acoustics,
            structural_events: self.structural_events,
            pogo: self.pogo,
            csi: self.csi,
            countdown: self.countdown,
            upstream_uq: self.upstream_uq,
        })
    }

    /// Current simulation time in seconds.
    pub(crate) fn time_s(&self) -> f64 {
        self.kernel.current_time().as_seconds()
    }

    /// Current kernel step index.
    pub(crate) fn step_index(&self) -> u64 {
        self.kernel.current_step().value()
    }

    /// Truth rigid-body state of the continuing stack.
    pub(crate) fn state(&self) -> &RigidBodyState {
        self.kernel.current_state()
    }

    /// Separated bodies currently propagated alongside the stack.
    pub(crate) fn separated_bodies(&self) -> &[openbmp_sim::SeparatedRigidBody] {
        self.kernel.separated_rigid_bodies()
    }

    /// Per-engine actuation snapshots keyed by engine id.
    pub(crate) fn engine_snapshots(&self) -> BTreeMap<EngineId, openbmp_sim::EngineSnapshot> {
        self.engine_rack.snapshot_map()
    }

    /// Environment sample (atmosphere, gravity, wind) at the current state.
    pub(crate) fn environment_sample(&self) -> Result<openbmp_sim::EnvironmentSample, RunnerError> {
        Ok(self.kernel.current_environment_sample()?)
    }

    /// Kernel mission phase id, when a mission graph is wired.
    pub(crate) fn current_phase(&self) -> Option<openbmp_sim::PhaseId> {
        self.kernel.current_phase()
    }

    /// Latest flight-controller observation (estimates, health,
    /// guidance reference), when the scenario wires a controller. This is
    /// the same read-only assembly the SIL monitor receives.
    pub(crate) fn fc_observation(&self) -> Option<crate::sil::FcObservation> {
        self.fc_bridge
            .as_ref()
            .map(crate::fc_bridge::FcBridge::collect_observation)
    }

    /// Latest guidance cutoff estimate published by the FC.
    pub(crate) fn latest_guidance_cutoff(&self) -> Option<openbmp_fc::topics::GuidanceCutoff> {
        self.fc_bridge
            .as_ref()
            .and_then(crate::fc_bridge::FcBridge::latest_guidance_cutoff)
    }

    /// Drain recorded telemetry rows, bounding table memory for
    /// streaming hosts. The schema is unchanged; see
    /// [`TelemetryTable::take_rows`].
    pub(crate) fn take_recorded_rows(&mut self) -> Vec<TelemetryRow> {
        self.table.take_rows()
    }

    /// Telemetry schema for interpreting drained rows.
    pub(crate) fn telemetry_schema(&self) -> &TelemetrySchema {
        self.table.schema()
    }

    /// Schedule a deterministic engine malfunction, addressed by the
    /// scenario engine name (`[[vehicle.assembly.engines]] id`), taking
    /// effect on the next kernel tick. Returns the scheduled fault id.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Engine`] when the name matches no declared
    /// engine.
    pub(crate) fn schedule_engine_fault(
        &mut self,
        engine: &str,
        fault: openbmp_propulsion::EngineFault,
    ) -> Result<String, RunnerError> {
        let engine_id = EngineId::from_path(&format!("vehicle.assembly.engines.{engine}"));
        let id = format!(
            "session.malfunctions.{}.{engine}",
            self.runtime_malfunction_seq
        );
        self.runtime_malfunction_seq += 1;
        let start_step = self.kernel.current_step().value();
        self.engine_rack
            .schedule_runtime_fault(id.clone(), engine_id, start_step, fault)?;
        Ok(id)
    }

    /// Arm a sensor outage window, addressed by the `[sensors.<name>]`
    /// key. See `FcBridge::arm_sensor_outage` for semantics.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::UnsupportedScenario`] when the scenario has
    /// no `[fc]` block, the sensor is unknown, or the window is empty.
    pub(crate) fn arm_sensor_outage(
        &mut self,
        sensor: &str,
        start_s: f64,
        stop_s: f64,
    ) -> Result<(), RunnerError> {
        let Some(bridge) = &mut self.fc_bridge else {
            return Err(RunnerError::UnsupportedScenario {
                what: "sensor outage stimuli require a scenario with an [fc] block".to_owned(),
            });
        };
        bridge.arm_sensor_outage(sensor, start_s, stop_s)
    }

    /// Set or clear the wind override (NED metres per second). While
    /// set, the override replaces the scenario wind sample pushed into
    /// the kernel each tick; clearing it restores the scenario wind (or
    /// zero wind for scenarios without a `[wind]` block).
    pub(crate) fn set_wind_override(&mut self, wind_ned_m_s: Option<Vector3<f64>>) {
        if wind_ned_m_s.is_none() && self.wind_override_ned_m_s.is_some() {
            // Scenarios without a wind rack never push samples, so the
            // kernel would otherwise hold the last override forever.
            self.kernel.set_wind_sample(Vector3::zeros());
        }
        self.wind_override_ned_m_s = wind_ned_m_s;
    }
}

fn accept_landing_gear_state(
    runtime: &crate::landing_gear::LandingGearRuntime,
    state: &RigidBodyState,
    body: Option<BodyId>,
    active: bool,
) -> Result<(), openbmp_sim::ModelEvalError> {
    if active {
        runtime.commit_accepted(state, body)
    } else {
        runtime.begin_interval(state, body, false)
    }
}

fn retired_separated_body_ids(
    separated_bodies: &[openbmp_sim::SeparatedRigidBody],
) -> BTreeSet<BodyId> {
    separated_bodies
        .iter()
        .filter(|body| !body.propagating)
        .map(|body| body.body)
        .collect()
}

fn automatic_ground_impact(document: &ScenarioDocument) -> GroundImpact {
    if document.contact.is_some() || document.vehicle.landing_gear.is_some() {
        return GroundImpact::disabled();
    }
    if document.environment.gravity == "constant" {
        GroundImpact::sea_level()
    } else {
        GroundImpact::disabled()
    }
}

fn infer_near_surface_geocentric_radius_m(initial_state: &RigidBodyState) -> Option<f64> {
    let radius_m = initial_state.position.vector.norm();
    // Sea-level Earth radii are roughly 6.357e6..6.378e6 m. Include a
    // little margin for rounded synthetic launch radii, but avoid
    // treating high-altitude entry/orbit initial states as the ground.
    if radius_m.is_finite() && (6_330_000.0..=6_390_000.0).contains(&radius_m) {
        Some(radius_m)
    } else {
        None
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

fn build_effector_actual_snapshot_map(
    document: &ScenarioDocument,
    deck_bindings: &[crate::aero_effector_match::DeckAxisBinding],
    snapshot: &[openbmp_vehicle::EffectorState],
    direct_torque_present: bool,
) -> Result<BTreeMap<String, f64>, RunnerError> {
    let mut snapshot_map = crate::aero_effector_match::build_snapshot_map(deck_bindings, snapshot);
    if direct_torque_present {
        let direct_torque_map =
            crate::aero_effector_match::build_direct_torque_snapshot_map(document, snapshot);
        merge_direct_torque_snapshot_map(&mut snapshot_map, direct_torque_map)?;
    }
    Ok(snapshot_map)
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
        "constant" | "point_mass" | "j2" | "egm2008" | "third_body" | "tesseral"
    ) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "environment.gravity = {} (wired: constant, point_mass, j2, egm2008, third_body, tesseral)",
                document.environment.gravity
            ),
        });
    }
    for name in document.force_model_universe() {
        if !matches!(
            name.as_str(),
            "gravity" | "aero" | "thrust" | "aerothermal_diagnostics" | "contact" | "landing_gear"
        ) {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "forces.models entry `{name}` (only gravity, aero, thrust, \
                     aerothermal_diagnostics, contact, landing_gear wired)"
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

fn multibody_authority_replacement_at_kernel_time(
    mut replacement: RigidBodyState,
    kernel_time: SimTime,
    authority: &str,
) -> Result<RigidBodyState, RunnerError> {
    let replacement_time_s = replacement.time.as_seconds();
    let kernel_time_s = kernel_time.as_seconds();
    let delta_s = replacement_time_s - kernel_time_s;
    if !delta_s.is_finite() || delta_s.abs() > MULTIBODY_AUTHORITY_TIME_ABS_TOLERANCE_S {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "{authority} propagation authority forecast time {replacement_time_s} s does not match current kernel time {kernel_time_s} s"
            ),
        });
    }
    replacement.time = kernel_time;
    Ok(replacement)
}

fn primary_root_free_flyer_authority_enabled(document: &ScenarioDocument) -> bool {
    document.multi_body.as_ref().is_some_and(|multi_body| {
        multi_body.propagation_authority
            == MultiBodyPropagationAuthorityConfig::PrimaryRootFreeFlyer
    })
}

fn separated_root_free_flyer_authority_enabled(document: &ScenarioDocument) -> bool {
    document.multi_body.as_ref().is_some_and(|multi_body| {
        multi_body.propagation_authority
            == MultiBodyPropagationAuthorityConfig::SeparatedRootFreeFlyer
    })
}

fn welded_release_jettison_authority_enabled(document: &ScenarioDocument) -> bool {
    document.multi_body.as_ref().is_some_and(|multi_body| {
        multi_body.propagation_authority
            == MultiBodyPropagationAuthorityConfig::WeldedReleaseJettison
    })
}

fn articulated_gimbal_root_authority_enabled(document: &ScenarioDocument) -> bool {
    document.multi_body.as_ref().is_some_and(|multi_body| {
        multi_body.propagation_authority
            == MultiBodyPropagationAuthorityConfig::ArticulatedGimbalRoot
    })
}

#[derive(Clone, Debug)]
struct TankMassRoute {
    owner: BodyId,
    mount_point_tank_body_m: Vector3<f64>,
    rot_owner_from_tank_body: Matrix3<f64>,
    translation_owner_from_tank_body_m: Vector3<f64>,
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
    recovery_apparent_mass_ids: Vec<RecoveryId>,
    recovery_owners: BTreeMap<RecoveryId, BodyId>,
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
        let raw_motor_owner = document
            .propulsion
            .as_ref()
            .and_then(|p| p.motor.as_ref())
            .and_then(|m| m.mounted_to.as_deref())
            .map(body_id_from_scenario_text);
        let welded_release_owner_aliases = welded_release_descendant_owner_aliases(document);
        let motor_owner = raw_motor_owner.map(|owner| {
            welded_release_owner_aliases
                .get(&owner)
                .map_or(owner, |alias| alias.root)
        });
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
                let mounted_body = body_id_from_scenario_text(&tank.mounted_to);
                let alias = welded_release_owner_aliases.get(&mounted_body);
                (
                    id,
                    TankMassRoute {
                        owner: alias.map_or(mounted_body, |alias| alias.root),
                        mount_point_tank_body_m: Vector3::new(
                            tank.mount_point_body_m[0],
                            tank.mount_point_body_m[1],
                            tank.mount_point_body_m[2],
                        ),
                        rot_owner_from_tank_body: alias
                            .map_or_else(Matrix3::identity, |alias| alias.rot_root_from_body),
                        translation_owner_from_tank_body_m: alias
                            .map_or_else(Vector3::zeros, |alias| {
                                alias.translation_root_from_body_m
                            }),
                    },
                )
            })
            .collect();
        let recovery_apparent_mass_ids = apparent_mass_recovery_ids(document);
        let recovery_owners = recovery_owner_map(document)?;
        Ok(Self {
            dry_total,
            dry_bodies,
            motor_owner,
            motor_ignition_time_s,
            engine_ids,
            engine_owners,
            tank_coupled_engines,
            tank_routes,
            recovery_apparent_mass_ids,
            recovery_owners,
        })
    }
}

fn transform_tank_inertia_delta_to_owner_origin(
    inertia_tank_body_origin: Matrix3<f64>,
    mass_kg: f64,
    center_tank_body_m: Vector3<f64>,
    rot_owner_from_tank_body: Matrix3<f64>,
    translation_owner_from_tank_body_m: Vector3<f64>,
) -> Matrix3<f64> {
    let center_owner_from_tank_body_m = rot_owner_from_tank_body * center_tank_body_m;
    let inertia_tank_com =
        inertia_tank_body_origin - point_mass_parallel_axis_inertia(mass_kg, center_tank_body_m);
    let inertia_owner_com =
        rot_owner_from_tank_body * inertia_tank_com * rot_owner_from_tank_body.transpose();
    let center_owner_m = translation_owner_from_tank_body_m + center_owner_from_tank_body_m;
    inertia_owner_com + point_mass_parallel_axis_inertia(mass_kg, center_owner_m)
}

fn point_mass_parallel_axis_inertia(mass_kg: f64, offset_m: Vector3<f64>) -> Matrix3<f64> {
    mass_kg * (Matrix3::identity() * offset_m.dot(&offset_m) - offset_m * offset_m.transpose())
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
        recovery_snapshot: &BTreeMap<RecoveryId, openbmp_sim::RecoverySnapshot>,
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
            let tank_center_tank_body_m = route.mount_point_tank_body_m + snap.cg_offset_body_m;
            let tank_cg = route.translation_owner_from_tank_body_m
                + route.rot_owner_from_tank_body * tank_center_tank_body_m;
            total_mass_kg += snap.mass_kg;
            weighted_cg += tank_cg * snap.mass_kg;
            inertia_body += transform_tank_inertia_delta_to_owner_origin(
                snap.inertia_delta_body_kg_m2,
                snap.mass_kg,
                tank_center_tank_body_m,
                route.rot_owner_from_tank_body,
                route.translation_owner_from_tank_body_m,
            );
        }

        for id in &self.resources.recovery_apparent_mass_ids {
            let owner = self.resources.recovery_owners.get(id).copied();
            if !self.owner_matches(active_body, owner, "recovery apparent mass")? {
                continue;
            }
            let Some(snap) = recovery_snapshot.get(id) else {
                if active_body.is_none() && recovery_snapshot.is_empty() {
                    continue;
                }
                return Err(openbmp_sim::ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "recovery apparent mass: snapshot missing declared recovery id",
                    ),
                });
            };
            if !snap.apparent_mass_kg.is_finite() || snap.apparent_mass_kg < 0.0 {
                return Err(openbmp_sim::ModelEvalError::InvalidState {
                    model: self.model_id,
                    reason: Cow::Borrowed("recovery apparent mass must be finite and non-negative"),
                });
            }
            total_mass_kg += snap.apparent_mass_kg;
            weighted_cg += base.center_of_mass_body.vector * snap.apparent_mass_kg;
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
        tank_snapshot: &BTreeMap<TankId, openbmp_sim::TankSnapshot>,
        recovery_snapshot: &BTreeMap<RecoveryId, openbmp_sim::RecoverySnapshot>,
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
            // Tank snapshots already report their total drain (engine feed,
            // venting, and RCS). Count that loss once, consistently with the
            // absolute mass path above. Only self-contained engines remove
            // mass directly here.
            if self.resources.tank_coupled_engines.contains(id) {
                continue;
            }
            mass_rate_kg_s -= snap.mass_flow_kg_per_s;
        }
        for (id, route) in &self.resources.tank_routes {
            if !self.owner_matches(active_body, Some(route.owner), "tank mass-rate")? {
                continue;
            }
            let Some(snap) = tank_snapshot.get(id) else {
                if active_body.is_none() && tank_snapshot.is_empty() {
                    continue;
                }
                return Err(openbmp_sim::ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed("tank mass-rate: snapshot missing declared tank id"),
                });
            };
            if !snap.mass_rate_kg_s.is_finite() {
                return Err(openbmp_sim::ModelEvalError::NonFinite {
                    model: self.model_id,
                });
            }
            mass_rate_kg_s += snap.mass_rate_kg_s;
        }
        if active_body.is_none()
            && let Some(feedback) = &self.aerothermal_feedback
        {
            mass_rate_kg_s -= feedback.mass_loss_kg_s();
        }
        for id in &self.resources.recovery_apparent_mass_ids {
            let owner = self.resources.recovery_owners.get(id).copied();
            if !self.owner_matches(active_body, owner, "recovery apparent mass")? {
                continue;
            }
            let Some(snap) = recovery_snapshot.get(id) else {
                if active_body.is_none() && recovery_snapshot.is_empty() {
                    continue;
                }
                return Err(openbmp_sim::ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "recovery apparent mass-rate: snapshot missing declared recovery id",
                    ),
                });
            };
            if !snap.apparent_mass_rate_kg_s.is_finite() || snap.apparent_mass_rate_kg_s < 0.0 {
                return Err(openbmp_sim::ModelEvalError::InvalidState {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "recovery apparent mass rate must be finite and non-negative",
                    ),
                });
            }
            mass_rate_kg_s += snap.apparent_mass_rate_kg_s;
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
            &BTreeMap::new(),
        )
    }

    fn mass_properties_rate(
        &self,
        t: SimTime,
    ) -> Result<openbmp_sim::MassPropertiesRate, openbmp_sim::ModelEvalError> {
        self.mass_properties_rate_from_snapshots(
            t,
            None,
            &self.default_engine_snapshot,
            &self.default_tank_snapshot,
            &BTreeMap::new(),
        )
    }

    fn mass_properties_at(
        &self,
        ctx: openbmp_sim::MassContext<'_>,
    ) -> Result<MassProperties, openbmp_sim::ModelEvalError> {
        let engine_snapshot: BTreeMap<EngineId, openbmp_sim::EngineSnapshot> =
            ctx.engine_snapshot.iter().collect();
        let tank_snapshot: BTreeMap<TankId, openbmp_sim::TankSnapshot> =
            ctx.tank_snapshot.iter().collect();
        let recovery_snapshot: BTreeMap<RecoveryId, openbmp_sim::RecoverySnapshot> =
            ctx.recovery_snapshot.iter().collect();
        self.mass_properties_from_snapshots(
            ctx.time,
            ctx.active_body,
            &engine_snapshot,
            &tank_snapshot,
            &recovery_snapshot,
        )
    }

    fn mass_properties_rate_at(
        &self,
        ctx: openbmp_sim::MassContext<'_>,
    ) -> Result<openbmp_sim::MassPropertiesRate, openbmp_sim::ModelEvalError> {
        let engine_snapshot: BTreeMap<EngineId, openbmp_sim::EngineSnapshot> =
            ctx.engine_snapshot.iter().collect();
        let tank_snapshot: BTreeMap<TankId, openbmp_sim::TankSnapshot> =
            ctx.tank_snapshot.iter().collect();
        let recovery_snapshot: BTreeMap<RecoveryId, openbmp_sim::RecoverySnapshot> =
            ctx.recovery_snapshot.iter().collect();
        self.mass_properties_rate_from_snapshots(
            ctx.time,
            ctx.active_body,
            &engine_snapshot,
            &tank_snapshot,
            &recovery_snapshot,
        )
    }

    fn supports_separated_body_propagation(&self) -> bool {
        (self.motor.is_none() || self.resources.motor_owner.is_some())
            && self
                .resources
                .engine_ids
                .iter()
                .all(|id| self.resources.engine_owners.contains_key(id))
            && self
                .resources
                .recovery_apparent_mass_ids
                .iter()
                .all(|id| self.resources.recovery_owners.contains_key(id))
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

#[derive(Clone, Debug)]
struct RigidBodySeparationSpec {
    stack_body: BodyId,
    body: BodyId,
    stack_delta_v_body_m_s: [f64; 3],
    stage_delta_v_body_m_s: [f64; 3],
    stack_delta_omega_body_rad_s: [f64; 3],
    stage_delta_omega_body_rad_s: [f64; 3],
    stage_attitude_offset_body_xyzw: [f64; 4],
    stage_weld_translation_stack_body_m: [f64; 3],
    stage_weld_quaternion_stack_to_stage_xyzw: [f64; 4],
    subtree_welded_bodies: Vec<RigidBodyWeldedBodySpec>,
}

#[derive(Copy, Clone, Debug)]
struct RigidBodyWeldedBodySpec {
    parent_body: BodyId,
    body: BodyId,
    child_weld_translation_parent_body_m: [f64; 3],
    child_weld_quaternion_parent_to_child_xyzw: [f64; 4],
}

impl RigidBodySeparationSpec {
    fn departing_body_ids(&self) -> Vec<BodyId> {
        let mut bodies = Vec::with_capacity(self.subtree_welded_bodies.len() + 1);
        bodies.push(self.body);
        bodies.extend(
            self.subtree_welded_bodies
                .iter()
                .map(|welded_body| welded_body.body),
        );
        bodies
    }
}

fn build_rigid_body_separations(
    document: &ScenarioDocument,
    mass_resources: &RigidMassResources,
) -> Result<BTreeMap<BodyId, RigidBodySeparationSpec>, RunnerError> {
    let Some(multi_body) = document.multi_body.as_ref() else {
        return Ok(BTreeMap::new());
    };

    let mut specs = BTreeMap::new();
    let mut welded_children: BTreeMap<BodyId, Vec<RigidBodyWeldedBodySpec>> = BTreeMap::new();
    for welded_body in &multi_body.welded_bodies {
        let parent_body = body_id_from_scenario_text(&welded_body.parent_body_id);
        let child_body = body_id_from_scenario_text(&welded_body.child_body_id);
        if !mass_resources.dry_bodies.contains_key(&parent_body) {
            return Err(RunnerError::Assembly {
                field: "multi_body.welded_body.parent_body_id".to_owned(),
                reason: format!("body `{}` was not resolved", welded_body.parent_body_id),
            });
        }
        if !mass_resources.dry_bodies.contains_key(&child_body) {
            return Err(RunnerError::Assembly {
                field: "multi_body.welded_body.child_body_id".to_owned(),
                reason: format!("body `{}` was not resolved", welded_body.child_body_id),
            });
        }
        welded_children
            .entry(parent_body)
            .or_default()
            .push(RigidBodyWeldedBodySpec {
                parent_body,
                body: child_body,
                child_weld_translation_parent_body_m: welded_body
                    .child_weld_translation_parent_body_m,
                child_weld_quaternion_parent_to_child_xyzw: welded_body
                    .child_weld_quaternion_parent_to_child_xyzw,
            });
    }
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
        let subtree_welded_bodies =
            collect_welded_subtree_specs(document, lower_body, &welded_children)?;
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
                stage_attitude_offset_body_xyzw: separation
                    .lower_attitude_offset_body_xyzw
                    .unwrap_or([0.0, 0.0, 0.0, 1.0]),
                stage_weld_translation_stack_body_m: separation
                    .lower_weld_translation_upper_body_m
                    .unwrap_or([0.0, 0.0, 0.0]),
                stage_weld_quaternion_stack_to_stage_xyzw: separation
                    .lower_weld_quaternion_upper_to_lower_xyzw
                    .unwrap_or([0.0, 0.0, 0.0, 1.0]),
                subtree_welded_bodies,
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

fn collect_welded_subtree_specs(
    document: &ScenarioDocument,
    root_body: BodyId,
    welded_children: &BTreeMap<BodyId, Vec<RigidBodyWeldedBodySpec>>,
) -> Result<Vec<RigidBodyWeldedBodySpec>, RunnerError> {
    let mut specs = Vec::new();
    collect_welded_subtree_specs_from_parent(document, root_body, welded_children, &mut specs)?;
    Ok(specs)
}

fn collect_welded_subtree_specs_from_parent(
    document: &ScenarioDocument,
    parent_body: BodyId,
    welded_children: &BTreeMap<BodyId, Vec<RigidBodyWeldedBodySpec>>,
    specs: &mut Vec<RigidBodyWeldedBodySpec>,
) -> Result<(), RunnerError> {
    let Some(children) = welded_children.get(&parent_body) else {
        return Ok(());
    };
    for child in children {
        if let Some(resource) = unsupported_welded_descendant_runtime_resource(document, child.body)
        {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "welded_release_jettison does not support this welded-descendant runtime \
                     resource; body {} owns unsupported {resource}",
                    child.body.value(),
                ),
            });
        }
        specs.push(*child);
        collect_welded_subtree_specs_from_parent(document, child.body, welded_children, specs)?;
    }
    Ok(())
}

fn unsupported_welded_descendant_runtime_resource(
    document: &ScenarioDocument,
    body: BodyId,
) -> Option<&'static str> {
    let owner_matches = |owner: &str| body_id_from_scenario_text(owner) == body;
    if document.vehicle.assembly.engines.iter().any(|engine| {
        engine.mounted_to.as_deref().is_some_and(owner_matches)
            && engine.propellant.is_some()
            && !welded_release_tank_coupled_engine_supported(document, engine)
    }) {
        return Some("tank-coupled engine");
    }
    let descendant_tank_ids: BTreeSet<&str> = document
        .vehicle
        .assembly
        .tanks
        .iter()
        .filter(|tank| owner_matches(&tank.mounted_to))
        .map(|tank| tank.id.as_str())
        .collect();
    if !descendant_tank_ids.is_empty()
        && document.vehicle.assembly.engines.iter().any(|engine| {
            engine.propellant.as_ref().is_some_and(|propellant| {
                (descendant_tank_ids.contains(propellant.fuel_tank.as_str())
                    || propellant
                        .oxidizer_tank
                        .as_deref()
                        .is_some_and(|tank| descendant_tank_ids.contains(tank)))
                    && !welded_release_tank_coupled_engine_supported(document, engine)
            })
        })
    {
        return Some("tank-coupled tank");
    }
    for effector in &document.vehicle.assembly.effectors {
        if !effector.mounted_to.as_deref().is_some_and(owner_matches) {
            continue;
        }
        if matches!(
            effector.kind,
            openbmp_scenario::EffectorKindConfig::DirectTorque { .. }
        ) {
            continue;
        }
        if welded_release_scalar_effector_supported(document, effector) {
            continue;
        }
        return Some("effector");
    }
    for recovery in &document.vehicle.assembly.recovery {
        if !recovery.mounted_to.as_deref().is_some_and(owner_matches) {
            continue;
        }
        if !recovery.risers.is_empty()
            && !welded_release_recovery_frame_supported(document, recovery)
        {
            return Some("recovery riser");
        }
        if recovery.sequence.is_some()
            && !welded_release_recovery_frame_supported(document, recovery)
        {
            return Some("recovery sequence");
        }
    }
    if document
        .vehicle
        .landing_gear
        .as_ref()
        .is_some_and(|landing_gear| {
            landing_gear.legs.iter().any(|leg| {
                owner_matches(&leg.mounted_to)
                    && !welded_release_landing_gear_leg_supported(document, leg)
            })
        })
    {
        return Some("landing gear");
    }
    None
}

fn welded_release_landing_gear_leg_supported(
    document: &ScenarioDocument,
    leg: &openbmp_scenario::LandingGearLegConfig,
) -> bool {
    let owner = body_id_from_scenario_text(&leg.mounted_to);
    welded_release_descendant_owner_aliases(document).contains_key(&owner)
}

fn welded_release_scalar_effector_supported(
    document: &ScenarioDocument,
    effector: &openbmp_scenario::EffectorConfig,
) -> bool {
    if !matches!(
        effector.kind,
        openbmp_scenario::EffectorKindConfig::LinearActuator { .. }
            | openbmp_scenario::EffectorKindConfig::SecondOrderServo { .. }
    ) {
        return false;
    }
    let Some(owner_text) = effector.mounted_to.as_deref() else {
        return false;
    };
    let owner = body_id_from_scenario_text(owner_text);
    welded_release_descendant_owner_aliases(document).contains_key(&owner)
}

fn welded_release_tank_coupled_engine_supported(
    document: &ScenarioDocument,
    engine: &openbmp_scenario::EngineConfig,
) -> bool {
    let Some(propellant) = engine.propellant.as_ref() else {
        return false;
    };
    let Some(engine_owner_text) = engine.mounted_to.as_deref() else {
        return false;
    };
    let aliases = welded_release_descendant_owner_aliases(document);
    let engine_owner = body_id_from_scenario_text(engine_owner_text);
    let Some(engine_alias) = aliases.get(&engine_owner) else {
        return false;
    };
    let routed_engine_owner = engine_alias.root;
    propellant_tank_aliases_to_owner(
        document,
        &aliases,
        &propellant.fuel_tank,
        routed_engine_owner,
    ) && propellant.oxidizer_tank.as_deref().is_none_or(|tank| {
        propellant_tank_aliases_to_owner(document, &aliases, tank, routed_engine_owner)
    })
}

fn propellant_tank_aliases_to_owner(
    document: &ScenarioDocument,
    aliases: &BTreeMap<BodyId, WeldedReleaseOwnerAlias>,
    tank_id: &str,
    expected_owner: BodyId,
) -> bool {
    let Some(tank) = document
        .vehicle
        .assembly
        .tanks
        .iter()
        .find(|tank| tank.id == tank_id)
    else {
        return false;
    };
    let tank_owner = body_id_from_scenario_text(&tank.mounted_to);
    let Some(tank_alias) = aliases.get(&tank_owner) else {
        return false;
    };
    tank_alias.root == expected_owner
}

fn welded_release_recovery_frame_supported(
    document: &ScenarioDocument,
    recovery: &RecoveryConfig,
) -> bool {
    let Some(owner_text) = recovery.mounted_to.as_deref() else {
        return false;
    };
    let owner = body_id_from_scenario_text(owner_text);
    welded_release_descendant_owner_aliases(document).contains_key(&owner)
}

fn recovery_frame_aliases(
    document: &ScenarioDocument,
) -> BTreeMap<BodyId, crate::recovery::RecoveryFrameAlias> {
    welded_release_descendant_owner_aliases(document)
        .into_iter()
        .map(|(body, alias)| {
            (
                body,
                crate::recovery::RecoveryFrameAlias {
                    rot_owner_from_body: alias.rot_root_from_body,
                    translation_owner_from_body_m: alias.translation_root_from_body_m,
                },
            )
        })
        .collect()
}

fn landing_gear_owner_aliases(
    document: &ScenarioDocument,
) -> BTreeMap<BodyId, crate::landing_gear::LandingGearOwnerAlias> {
    welded_release_descendant_owner_aliases(document)
        .into_iter()
        .map(|(body, alias)| {
            (
                body,
                crate::landing_gear::LandingGearOwnerAlias {
                    root: alias.root,
                    rot_root_from_body: alias.rot_root_from_body,
                    translation_root_from_body_m: alias.translation_root_from_body_m,
                },
            )
        })
        .collect()
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
    use_welded_release_states: bool,
) -> Result<bool, RunnerError>
where
    I: openbmp_sim::Integrator<RigidBodyState>,
    F: ForceModel<RigidBodyState>,
    MOM: openbmp_sim::MomentModel<RigidBodyState>,
    MM: openbmp_sim::RigidMassModel,
    E: openbmp_sim::EnvironmentModel,
    SC: openbmp_sim::StopCondition<RigidBodyState>,
{
    let mut applied = false;
    for event in fired {
        match &event.action {
            ScenarioScriptAction::JettisonStage { body } => {
                let separation = separation_specs.get(body).cloned().ok_or_else(|| {
                    RunnerError::UnsupportedScenario {
                        what: format!(
                            "jettison_stage event {} fired for body id {} with no \
                                 matching [multi_body] separation",
                            event.binding_id.value(),
                            body.value()
                        ),
                    }
                })?;
                // The departing subtree leaves the stack; the remaining
                // members (minus the lead stack body) ride along and their
                // dry mass is folded into the continuing-stack mass.
                for departing in separation.departing_body_ids() {
                    stack_bodies.remove(&departing);
                }
                let inert = continuing_inert_bodies(stack_bodies, separation.stack_body);
                if use_welded_release_states {
                    let runtime = build_runtime_welded_release_separation_states(
                        kernel, mass_model, separation, &inert,
                    )?;
                    kernel.jettison_rigid_body_with_states(runtime)?;
                } else {
                    let runtime = build_runtime_rigid_body_separation(
                        kernel, mass_model, separation, &inert,
                    )?;
                    kernel.jettison_rigid_body(runtime)?;
                }
                applied = true;
            }
            ScenarioScriptAction::JettisonBodies { bodies } => {
                if use_welded_release_states {
                    // Remove every departing body from the stack FIRST so the
                    // continuing-inert set reflects the post-batch membership.
                    let separations = bodies
                        .iter()
                        .map(|body| {
                            separation_specs.get(body).cloned().ok_or_else(|| {
                                RunnerError::UnsupportedScenario {
                                    what: format!(
                                        "jettison_bodies event {} fired for body id {} with no \
                                             matching [multi_body] separation",
                                        event.binding_id.value(),
                                        body.value()
                                    ),
                                }
                            })
                        })
                        .collect::<Result<Vec<_>, _>>()?;
                    for separation in &separations {
                        for departing in separation.departing_body_ids() {
                            stack_bodies.remove(&departing);
                        }
                    }
                    let mut batch = Vec::with_capacity(bodies.len());
                    for separation in separations {
                        let inert = continuing_inert_bodies(stack_bodies, separation.stack_body);
                        batch.push(build_runtime_welded_release_separation_states(
                            kernel, mass_model, separation, &inert,
                        )?);
                    }
                    let Some(stack_body) = batch.first().map(|separation| separation.stack_body)
                    else {
                        return Err(RunnerError::UnsupportedScenario {
                            what: format!(
                                "welded_release_jettison batch event {} had no departing bodies",
                                event.binding_id.value()
                            ),
                        });
                    };
                    if let Some(mismatched) = batch
                        .iter()
                        .find(|separation| separation.stack_body != stack_body)
                    {
                        return Err(RunnerError::UnsupportedScenario {
                            what: format!(
                                "welded_release_jettison batch event {} mixed stack body {} with {}",
                                event.binding_id.value(),
                                mismatched.stack_body.value(),
                                stack_body.value()
                            ),
                        });
                    }
                    let stack_state = batch[0].stack_state;
                    for runtime in &mut batch {
                        runtime.stack_state = stack_state;
                    }
                    for runtime in batch {
                        kernel.jettison_rigid_body_with_states(runtime)?;
                    }
                    applied = true;
                } else {
                    // Remove every departing body from the stack FIRST so the
                    // continuing-inert set reflects the post-batch membership.
                    let separations = bodies
                        .iter()
                        .map(|body| {
                            separation_specs.get(body).cloned().ok_or_else(|| {
                                RunnerError::UnsupportedScenario {
                                    what: format!(
                                        "jettison_bodies event {} fired for body id {} with no \
                                             matching [multi_body] separation",
                                        event.binding_id.value(),
                                        body.value()
                                    ),
                                }
                            })
                        })
                        .collect::<Result<Vec<_>, _>>()?;
                    for separation in &separations {
                        for departing in separation.departing_body_ids() {
                            stack_bodies.remove(&departing);
                        }
                    }
                    let mut batch = Vec::with_capacity(bodies.len());
                    for separation in separations {
                        let inert = continuing_inert_bodies(stack_bodies, separation.stack_body);
                        batch.push(build_runtime_rigid_body_separation(
                            kernel, mass_model, separation, &inert,
                        )?);
                    }
                    kernel.jettison_rigid_bodies(&batch)?;
                    applied = true;
                }
            }
            _ => {}
        }
    }
    Ok(applied)
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
    let recovery_snapshot = kernel.recovery_snapshot();
    let stack_mass_properties = mass_model
        .mass_properties_at(openbmp_sim::MassContext {
            time,
            active_body: Some(separation.stack_body),
            engine_snapshot: openbmp_sim::EngineSnapshotView::new(engine_snapshot),
            tank_snapshot: openbmp_sim::TankSnapshotView::new(tank_snapshot),
            recovery_snapshot: openbmp_sim::RecoverySnapshotView::new(recovery_snapshot),
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
            recovery_snapshot: openbmp_sim::RecoverySnapshotView::new(recovery_snapshot),
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
        stage_attitude_offset_body_xyzw: separation.stage_attitude_offset_body_xyzw,
    })
}

fn build_runtime_welded_release_separation_states<I, F, MOM, MM, E, SC>(
    kernel: &openbmp_sim::RigidBodyKernel<I, F, MOM, MM, E, SC>,
    mass_model: &RigidMassEither,
    separation: RigidBodySeparationSpec,
    continuing_inert: &[BodyId],
) -> Result<RigidBodySeparationStates, RunnerError>
where
    I: openbmp_sim::Integrator<RigidBodyState>,
    F: ForceModel<RigidBodyState>,
    MOM: openbmp_sim::MomentModel<RigidBodyState>,
    MM: openbmp_sim::RigidMassModel,
    E: openbmp_sim::EnvironmentModel,
    SC: openbmp_sim::StopCondition<RigidBodyState>,
{
    let runtime = build_runtime_rigid_body_separation(
        kernel,
        mass_model,
        separation.clone(),
        continuing_inert,
    )?;
    let current = *kernel.current_state();
    let stack_inertia = SpatialInertia::from_mass_properties(&runtime.stack_mass_properties)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("welded-release stack spatial inertia failed: {err}"),
        })?;
    let stage_inertia = SpatialInertia::from_mass_properties(&runtime.stage_mass_properties)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("welded-release stage spatial inertia failed: {err}"),
        })?;
    let stage_parent_to_body = welded_transform_from_arrays(
        separation.stage_weld_translation_stack_body_m[0],
        separation.stage_weld_translation_stack_body_m[1],
        separation.stage_weld_translation_stack_body_m[2],
        separation.stage_weld_quaternion_stack_to_stage_xyzw,
        "welded-release fixed transform failed",
    )?;
    let stage_translation_stack_body_m = *stage_parent_to_body.translation_parent_m();
    let rot_stack_from_stage = stage_parent_to_body.rot_child_from_parent().transpose();
    let mut tree_specs = vec![
        TreeBodySpec {
            id: runtime.stack_body,
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: stack_inertia,
            parent_to_body: PluckerTransform::identity(),
        },
        TreeBodySpec {
            id: runtime.body,
            parent: Some(openbmp_multibody::BodyIndex::new(0)),
            joint: Joint::Welded { released: false },
            inertia: stage_inertia,
            parent_to_body: stage_parent_to_body,
        },
    ];
    let mut body_indices = BTreeMap::from([
        (runtime.stack_body, openbmp_multibody::BodyIndex::new(0)),
        (runtime.body, openbmp_multibody::BodyIndex::new(1)),
    ]);
    let mut root_transforms = BTreeMap::from([(runtime.body, PluckerTransform::identity())]);
    let mut subtree_mass_components = Vec::new();
    for welded_body in &separation.subtree_welded_bodies {
        let parent_index = body_indices
            .get(&welded_body.parent_body)
            .copied()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "welded-release descendant parent {} was not built before child {}",
                    welded_body.parent_body.value(),
                    welded_body.body.value()
                ),
            })?;
        let parent_root_transform =
            *root_transforms
                .get(&welded_body.parent_body)
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: format!(
                        "welded-release descendant parent {} is not inside released subtree",
                        welded_body.parent_body.value()
                    ),
                })?;
        let parent_to_body = welded_transform_from_arrays(
            welded_body.child_weld_translation_parent_body_m[0],
            welded_body.child_weld_translation_parent_body_m[1],
            welded_body.child_weld_translation_parent_body_m[2],
            welded_body.child_weld_quaternion_parent_to_child_xyzw,
            "welded-release descendant transform failed",
        )?;
        let root_to_body = parent_root_transform.then(&parent_to_body).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("welded-release descendant transform chain failed: {err}"),
            }
        })?;
        let dry_mass_properties = mass_model
            .dry_body_properties(welded_body.body)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "welded-release descendant body {} has no dry mass properties",
                    welded_body.body.value()
                ),
            })?;
        let inertia =
            SpatialInertia::from_mass_properties(&dry_mass_properties).map_err(|err| {
                RunnerError::UnsupportedScenario {
                    what: format!("welded-release descendant spatial inertia failed: {err}"),
                }
            })?;
        let body_index = openbmp_multibody::BodyIndex::new(tree_specs.len());
        tree_specs.push(TreeBodySpec {
            id: welded_body.body,
            parent: Some(parent_index),
            joint: Joint::Welded { released: false },
            inertia,
            parent_to_body,
        });
        body_indices.insert(welded_body.body, body_index);
        root_transforms.insert(welded_body.body, root_to_body);
        subtree_mass_components.push(WeldedSubtreeMassComponent {
            root_to_body,
            mass_properties: dry_mass_properties,
        });
    }
    let stage_mass_properties = aggregate_welded_subtree_mass_properties(
        runtime.stage_mass_properties,
        &subtree_mass_components,
    );
    let tree = MultibodyTree::new(tree_specs).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("welded-release multibody tree failed: {err}"),
    })?;
    let orientation = current.orientation.q.into_inner();
    let linear_velocity_body = current
        .orientation
        .inverse()
        .rotate_velocity(current.velocity)
        .vector;
    let q = vec![
        orientation.w,
        orientation.i,
        orientation.j,
        orientation.k,
        current.position.vector.x,
        current.position.vector.y,
        current.position.vector.z,
    ];
    let qd = vec![
        current.angular_velocity.vector.x,
        current.angular_velocity.vector.y,
        current.angular_velocity.vector.z,
        linear_velocity_body.x,
        linear_velocity_body.y,
        linear_velocity_body.z,
    ];
    let stack_sim_state = tree
        .sim_state_from_state(MultibodyState::new(current.time, q, qd))
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("welded-release multibody state failed: {err}"),
        })?;
    let release_body_index = openbmp_multibody::BodyIndex::new(1);
    let subtree_momentum_before = tree
        .subtree_spatial_momentum_at_state(stack_sim_state.state(), release_body_index)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("welded-release subtree momentum audit failed: {err}"),
        })?;
    let (released_tree, released_state) = tree
        .release_welded_subtree_as_free_flyer(stack_sim_state.state(), release_body_index)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("welded-release subtree handoff failed: {err}"),
        })?;
    let released_momentum = released_tree
        .system_spatial_momentum_at_state(&released_state)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("welded-release released momentum audit failed: {err}"),
        })?;
    validate_welded_release_momentum_conservation(subtree_momentum_before, released_momentum)?;
    let released_sim_state = released_tree
        .sim_state_from_state(released_state)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("welded-release free-flyer state failed: {err}"),
        })?;
    let stage_state = root_free_flyer_rigid_state_from_multibody_state(
        runtime.body,
        &released_sim_state,
        stage_mass_properties,
    )?;
    let stack_state = RigidBodyState::new(
        current.time,
        current.position,
        current.velocity,
        current.orientation,
        current.angular_velocity,
        runtime.stack_mass_properties,
    );
    let stack_state = apply_welded_release_impulses(
        &current,
        &stack_state,
        runtime.stack_mass_properties,
        WeldedReleaseGeometry {
            body_origin_parent_m: Vector3::zeros(),
            rot_parent_from_body: nalgebra::Matrix3::identity(),
        },
        runtime.stack_delta_v_body_m_s,
        runtime.stack_delta_omega_body_rad_s,
        [0.0, 0.0, 0.0, 1.0],
    );
    let stage_state = apply_welded_release_impulses(
        &current,
        &stage_state,
        stage_mass_properties,
        WeldedReleaseGeometry {
            body_origin_parent_m: stage_translation_stack_body_m,
            rot_parent_from_body: rot_stack_from_stage,
        },
        runtime.stage_delta_v_body_m_s,
        runtime.stage_delta_omega_body_rad_s,
        runtime.stage_attitude_offset_body_xyzw,
    );
    Ok(RigidBodySeparationStates {
        stack_body: runtime.stack_body,
        body: runtime.body,
        stack_state,
        stage_state,
    })
}

fn validate_welded_release_momentum_conservation(
    subtree_momentum_before: SpatialForce,
    released_momentum: SpatialForce,
) -> Result<(), RunnerError> {
    let mut residual_norm_sq = 0.0_f64;
    let mut before_norm_sq = 0.0_f64;
    let mut released_norm_sq = 0.0_f64;
    for component in 0..6 {
        let before = subtree_momentum_before.vector()[component];
        let released = released_momentum.vector()[component];
        let residual = released - before;
        residual_norm_sq += residual * residual;
        before_norm_sq += before * before;
        released_norm_sq += released * released;
    }
    let residual_norm = residual_norm_sq.sqrt();
    let scale = before_norm_sq.sqrt().max(released_norm_sq.sqrt()).max(1.0);
    let tolerance =
        WELDED_RELEASE_MOMENTUM_ABS_TOLERANCE + WELDED_RELEASE_MOMENTUM_REL_TOLERANCE * scale;
    if !residual_norm.is_finite() || residual_norm > tolerance {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "welded-release subtree momentum residual {residual_norm:.6e} exceeded tolerance {tolerance:.6e}"
            ),
        });
    }
    Ok(())
}

struct WeldedSubtreeMassComponent {
    root_to_body: PluckerTransform,
    mass_properties: MassProperties,
}

fn welded_transform_from_arrays(
    translation_x_m: f64,
    translation_y_m: f64,
    translation_z_m: f64,
    quaternion_parent_to_child_xyzw: [f64; 4],
    context: &str,
) -> Result<PluckerTransform, RunnerError> {
    let quaternion_parent_to_child =
        nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            quaternion_parent_to_child_xyzw[3],
            quaternion_parent_to_child_xyzw[0],
            quaternion_parent_to_child_xyzw[1],
            quaternion_parent_to_child_xyzw[2],
        ));
    let rot_child_from_parent = quaternion_parent_to_child
        .to_rotation_matrix()
        .matrix()
        .clone_owned();
    PluckerTransform::new(
        rot_child_from_parent,
        Vector3::new(translation_x_m, translation_y_m, translation_z_m),
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("{context}: {err}"),
    })
}

fn aggregate_welded_subtree_mass_properties(
    root: MassProperties,
    descendants: &[WeldedSubtreeMassComponent],
) -> MassProperties {
    if descendants.is_empty() {
        return root;
    }
    let mut components = Vec::with_capacity(descendants.len() + 1);
    components.push((
        root.mass_kg(),
        root.center_of_mass_body.vector,
        root.inertia_body,
    ));
    for descendant in descendants {
        let rot_root_from_body = descendant.root_to_body.rot_child_from_parent().transpose();
        let body_origin_root_m = *descendant.root_to_body.translation_parent_m();
        let mass_properties = descendant.mass_properties;
        let center_of_mass_root_m =
            body_origin_root_m + rot_root_from_body * mass_properties.center_of_mass_body.vector;
        let inertia_root =
            rot_root_from_body * mass_properties.inertia_body * rot_root_from_body.transpose();
        components.push((
            mass_properties.mass_kg(),
            center_of_mass_root_m,
            inertia_root,
        ));
    }

    let mut total_mass_kg = 0.0_f64;
    let mut weighted_center = Vector3::zeros();
    for (mass_kg, center, _) in &components {
        total_mass_kg += *mass_kg;
        weighted_center += center * *mass_kg;
    }
    let aggregate_center = weighted_center / total_mass_kg;
    let mut aggregate_inertia = nalgebra::Matrix3::zeros();
    let identity = nalgebra::Matrix3::identity();
    for (mass_kg, center, inertia) in components {
        let offset = center - aggregate_center;
        aggregate_inertia +=
            inertia + (identity * offset.dot(&offset) - offset * offset.transpose()) * mass_kg;
    }

    MassProperties::new(
        Mass::new::<kilogram>(total_mass_kg),
        Position3::<Body>::new(aggregate_center.x, aggregate_center.y, aggregate_center.z),
        aggregate_inertia,
    )
}

struct WeldedReleaseGeometry {
    body_origin_parent_m: Vector3<f64>,
    rot_parent_from_body: nalgebra::Matrix3<f64>,
}

fn apply_welded_release_impulses(
    pre_split: &RigidBodyState,
    released: &RigidBodyState,
    mass_properties: MassProperties,
    geometry: WeldedReleaseGeometry,
    delta_v_body_m_s: [f64; 3],
    delta_omega_body_rad_s: [f64; 3],
    attitude_offset_body_xyzw: [f64; 4],
) -> RigidBodyState {
    let relative_parent_m = geometry.body_origin_parent_m
        + geometry.rot_parent_from_body * mass_properties.center_of_mass_body.vector
        - pre_split.mass_props.center_of_mass_body.vector;
    let position_offset_eci_m = pre_split.orientation.q * relative_parent_m;
    let rotational_velocity_body_m_s = pre_split.angular_velocity.vector.cross(&relative_parent_m);
    let rotational_velocity_eci_m_s = pre_split.orientation.q * rotational_velocity_body_m_s;
    let delta_v_eci_m_s = released.orientation.q
        * Vector3::new(
            delta_v_body_m_s[0],
            delta_v_body_m_s[1],
            delta_v_body_m_s[2],
        );
    let delta_omega_body = Vector3::new(
        delta_omega_body_rad_s[0],
        delta_omega_body_rad_s[1],
        delta_omega_body_rad_s[2],
    );
    let offset = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        attitude_offset_body_xyzw[3],
        attitude_offset_body_xyzw[0],
        attitude_offset_body_xyzw[1],
        attitude_offset_body_xyzw[2],
    ));
    RigidBodyState::new(
        released.time,
        Position3::from_vector(pre_split.position.vector + position_offset_eci_m),
        Velocity3::from_vector(
            pre_split.velocity.vector + rotational_velocity_eci_m_s + delta_v_eci_m_s,
        ),
        Quaternion::from_unit_quaternion(released.orientation.q * offset),
        AngularVelocity3::from_vector(released.angular_velocity.vector + delta_omega_body),
        mass_properties,
    )
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

#[derive(Clone, Debug)]
struct RootFreeFlyerMultibodySeed {
    body: BodyId,
    rigid_state: RigidBodyState,
    tree: MultibodyTree,
    sim_state: MultibodySimState,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalMultibodySeed {
    body: BodyId,
    engine: EngineId,
    joint: MultiBodyGimbalJointConfig,
    mechanism: Option<ArticulatedGimbalMechanism>,
    cfe_loops: Vec<ArticulatedGimbalCfeLoop>,
    cfe_gimbal_locks: Vec<ArticulatedGimbalCfeGimbalLock>,
    cfe_planar_four_bars: Vec<ArticulatedGimbalCfePlanarFourBar>,
    parent_mass_props: MassProperties,
    tree: MultibodyTree,
    sim_state: MultibodySimState,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalRootAuthoritySeed {
    body: BodyId,
    joints: Vec<ArticulatedGimbalRootAuthorityJoint>,
    parent_mass_props: MassProperties,
    tree: MultibodyTree,
    sim_state: MultibodySimState,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalRootAuthorityJoint {
    engine: EngineId,
    joint: MultiBodyGimbalJointConfig,
    terminal_body_index: openbmp_multibody::BodyIndex,
    q_start: usize,
    qd_start: usize,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalRootAuthorityJointSpec {
    body: BodyId,
    engine: EngineId,
    joint: MultiBodyGimbalJointConfig,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalMechanism {
    id: String,
    axis: MultiBodyGimbalMechanismAxisConfig,
    elements: Vec<ScalarMechanismElement>,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalCfeLoop {
    id: String,
    position_residual: f64,
    velocity_residual_per_s: f64,
    velocity_bias_acceleration_per_s2: f64,
    jacobian_row: Vec<f64>,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalCfeGimbalLock {
    id: String,
    axis: MultiBodyGimbalMechanismAxisConfig,
    locked_angle_rad: f64,
    velocity_bias_acceleration_per_s2: f64,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalCfePlanarFourBar {
    id: String,
    input_axis: MultiBodyGimbalMechanismAxisConfig,
    output_axis: MultiBodyGimbalMechanismAxisConfig,
    loop_geometry: PlanarFourBarLoop,
    velocity_bias_acceleration_per_s2: f64,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalCfeLoopSolveSample {
    loop_id: String,
    lagrange_multiplier: f64,
    stabilized_acceleration_per_s2: f64,
    acceleration_residual_per_s2: f64,
}

#[derive(Debug)]
struct ArticulatedGimbalDerivativeEvaluation {
    derivative: MultibodyDerivative,
    cfe_loop_solves: Vec<ArticulatedGimbalCfeLoopSolveSample>,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalJointState {
    angles_rad: Vec<f64>,
    rates_rad_s: Vec<f64>,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalMultibodyShadow {
    last_live_joint_state: ArticulatedGimbalJointState,
    seed: ArticulatedGimbalMultibodySeed,
    last_derivative: Option<MultibodyDerivative>,
    last_cfe_loop_solves: Vec<ArticulatedGimbalCfeLoopSolveSample>,
    last_rk4_forecast: Option<ArticulatedGimbalMultibodySeed>,
    last_root_rk4_forecast: Option<RigidBodyState>,
    last_authoritative_root_state: Option<RigidBodyState>,
    mechanism_latch_states: Vec<LatchState>,
}

#[derive(Clone, Debug)]
struct ArticulatedGimbalRootAuthorityShadow {
    last_live_joint_states: Vec<ArticulatedGimbalJointState>,
    seed: ArticulatedGimbalRootAuthoritySeed,
    last_derivative: Option<MultibodyDerivative>,
    last_rk4_forecast: Option<ArticulatedGimbalRootAuthoritySeed>,
    last_root_rk4_forecast: Option<RigidBodyState>,
    last_authoritative_root_state: Option<RigidBodyState>,
}

#[derive(Debug)]
struct RootFreeFlyerMultibodyShadow {
    seed: RootFreeFlyerMultibodySeed,
    last_derivative: Option<MultibodyDerivative>,
    last_rk4_forecast: Option<RootFreeFlyerMultibodySeed>,
    last_authoritative_state: Option<RigidBodyState>,
}

fn build_initial_root_free_flyer_multibody_seed(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    mass_resources: &RigidMassResources,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    tank_snapshot: &BTreeMap<TankId, openbmp_sim::TankSnapshot>,
    initial_state: &RigidBodyState,
) -> Result<Option<RootFreeFlyerMultibodySeed>, RunnerError> {
    let Some(primary_body_id) = document
        .multi_body
        .as_ref()
        .and_then(|multi_body| multi_body.primary_body_id.as_deref())
    else {
        return Ok(None);
    };
    let body = body_id_from_scenario_text(primary_body_id);
    let primary_mass_properties = RigidMassResourceModel::new(
        mass_resources.clone(),
        loaded.motor.clone(),
        None,
        RIGID_BODY_MOTOR_MASS_MODEL_ID,
    )
    .mass_properties_from_snapshots(
        initial_state.time,
        Some(body),
        engine_snapshot,
        tank_snapshot,
        &BTreeMap::new(),
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("initial primary body `{primary_body_id}` mass properties failed: {err}"),
    })?;
    let rigid_state = RigidBodyState::new(
        initial_state.time,
        initial_state.position,
        initial_state.velocity,
        initial_state.orientation,
        initial_state.angular_velocity,
        primary_mass_properties,
    );
    root_free_flyer_multibody_seed_from_rigid_state(body, rigid_state).map(Some)
}

fn build_initial_articulated_gimbal_multibody_seeds(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    mass_resources: &RigidMassResources,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    tank_snapshot: &BTreeMap<TankId, openbmp_sim::TankSnapshot>,
    initial_state: &RigidBodyState,
) -> Result<Vec<ArticulatedGimbalMultibodySeed>, RunnerError> {
    let Some(multi_body) = document.multi_body.as_ref() else {
        return Ok(Vec::new());
    };
    let has_cfe_constraints = !multi_body.cfe_loops.is_empty()
        || !multi_body.cfe_gimbal_locks.is_empty()
        || !multi_body.cfe_planar_four_bars.is_empty();
    if multi_body.gimbal_joints.is_empty() {
        if has_cfe_constraints {
            return Err(RunnerError::UnsupportedScenario {
                what: "multi_body CFE constraints require exactly one articulated gimbal shadow in the current runner"
                    .to_owned(),
            });
        }
        return Ok(Vec::new());
    }
    if has_cfe_constraints && multi_body.gimbal_joints.len() != 1 {
        return Err(RunnerError::UnsupportedScenario {
            what:
                "multi_body CFE constraints currently require exactly one articulated gimbal shadow"
                    .to_owned(),
        });
    }

    let mass_model = RigidMassResourceModel::new(
        mass_resources.clone(),
        loaded.motor.clone(),
        None,
        RIGID_BODY_MOTOR_MASS_MODEL_ID,
    );
    let mut seeds = Vec::with_capacity(multi_body.gimbal_joints.len());
    for joint in &multi_body.gimbal_joints {
        let body = body_id_from_scenario_text(&joint.body_id);
        let parent_mass_properties = mass_model
            .mass_properties_from_snapshots(
                initial_state.time,
                Some(body),
                engine_snapshot,
                tank_snapshot,
                &BTreeMap::new(),
            )
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!(
                    "initial articulated gimbal body `{}` mass properties failed: {err}",
                    joint.body_id
                ),
            })?;
        let parent_state = RigidBodyState::new(
            initial_state.time,
            initial_state.position,
            initial_state.velocity,
            initial_state.orientation,
            initial_state.angular_velocity,
            parent_mass_properties,
        );
        let mut seed = articulated_gimbal_multibody_seed_from_config(
            body,
            engine_id_from_scenario_text(&joint.engine_id),
            joint,
            articulated_gimbal_mechanism_from_document(document, joint)?,
            &parent_state,
        )?;
        seed.cfe_loops =
            articulated_gimbal_cfe_loops_from_config(&multi_body.cfe_loops, seed.tree.n_qd())?;
        seed.cfe_gimbal_locks = articulated_gimbal_cfe_gimbal_locks_from_config(
            &multi_body.cfe_gimbal_locks,
            seed.engine,
        )?;
        seed.cfe_planar_four_bars = articulated_gimbal_cfe_planar_four_bars_from_config(
            &multi_body.cfe_planar_four_bars,
            seed.engine,
        )?;
        seeds.push(seed);
    }
    Ok(seeds)
}

fn build_initial_articulated_gimbal_root_authority_shadow(
    document: &ScenarioDocument,
    seeds: &[ArticulatedGimbalMultibodySeed],
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    dt_s: f64,
) -> Result<Option<ArticulatedGimbalRootAuthorityShadow>, RunnerError> {
    if !articulated_gimbal_root_authority_enabled(document) || seeds.len() <= 1 {
        return Ok(None);
    }
    let Some((first_seed, remaining)) = seeds.split_first() else {
        return Ok(None);
    };
    if first_seed.mechanism.is_some() || remaining.iter().any(|seed| seed.mechanism.is_some()) {
        return Err(RunnerError::UnsupportedScenario {
            what: "multi-gimbal articulated_gimbal_root authority does not yet support contact mechanism state"
                .to_owned(),
        });
    }
    if remaining.iter().any(|seed| seed.body != first_seed.body) {
        return Err(RunnerError::UnsupportedScenario {
            what: "multi-gimbal articulated_gimbal_root authority requires all gimbals on one primary body"
                .to_owned(),
        });
    }
    let parent_state = articulated_gimbal_root_rigid_state_from_seed(first_seed)?;
    let mut joint_states = Vec::with_capacity(seeds.len());
    for seed in seeds {
        let joint_dof = articulated_gimbal_joint_dof(&seed.joint);
        let joint_angles_rad =
            seed.sim_state
                .q()
                .get(7..)
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "articulated gimbal root-authority seed is missing joint coordinates"
                        .to_owned(),
                })?;
        let joint_rates_rad_s =
            seed.sim_state
                .qd()
                .get(6..)
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "articulated gimbal root-authority seed is missing joint rates"
                        .to_owned(),
                })?;
        if joint_angles_rad.len() != joint_dof || joint_rates_rad_s.len() != joint_dof {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "articulated gimbal root-authority seed for engine {} has {} q and {} qd joint entries (expected {joint_dof})",
                    seed.engine.value(),
                    joint_angles_rad.len(),
                    joint_rates_rad_s.len()
                ),
            });
        }
        joint_states.push(ArticulatedGimbalJointState {
            angles_rad: joint_angles_rad.to_vec(),
            rates_rad_s: joint_rates_rad_s.to_vec(),
        });
    }
    let joint_specs: Vec<_> = seeds
        .iter()
        .map(|seed| ArticulatedGimbalRootAuthorityJointSpec {
            body: seed.body,
            engine: seed.engine,
            joint: seed.joint.clone(),
        })
        .collect();
    let authority_seed = articulated_gimbal_root_authority_seed_from_joint_states(
        &joint_specs,
        &parent_state,
        &joint_states,
    )?;
    authority_seed
        .tree
        .joint_space_inertia_crba_at_state(authority_seed.sim_state.state())
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("initial coupled articulated gimbal inertia validation failed: {err}"),
        })?;
    let derivative = articulated_gimbal_root_authority_derivative_from_engine_thrust(
        &authority_seed,
        engine_snapshot,
    )?;
    let forecast = articulated_gimbal_root_authority_rk4_forecast_from_engine_thrust(
        &authority_seed,
        engine_snapshot,
        dt_s,
    )?;
    let root_forecast = articulated_gimbal_root_authority_rigid_state_from_seed(&forecast)?;
    Ok(Some(ArticulatedGimbalRootAuthorityShadow {
        last_live_joint_states: joint_states,
        seed: authority_seed,
        last_derivative: Some(derivative),
        last_rk4_forecast: Some(forecast),
        last_root_rk4_forecast: Some(root_forecast),
        last_authoritative_root_state: None,
    }))
}

fn articulated_gimbal_multibody_seed_from_config(
    body: BodyId,
    engine: EngineId,
    joint: &MultiBodyGimbalJointConfig,
    mechanism: Option<ArticulatedGimbalMechanism>,
    parent_state: &RigidBodyState,
) -> Result<ArticulatedGimbalMultibodySeed, RunnerError> {
    let (joint_angles_rad, joint_rates_rad_s) = if joint.secondary_axis_body.is_some() {
        (
            vec![0.0, joint.initial_angle_rad],
            vec![0.0, joint.initial_rate_rad_s],
        )
    } else {
        (
            vec![joint.initial_angle_rad],
            vec![joint.initial_rate_rad_s],
        )
    };
    articulated_gimbal_multibody_seed_from_config_with_joint_state(
        body,
        engine,
        joint,
        mechanism,
        parent_state,
        &joint_angles_rad,
        &joint_rates_rad_s,
    )
}

fn articulated_gimbal_mechanism_from_document(
    document: &ScenarioDocument,
    joint: &MultiBodyGimbalJointConfig,
) -> Result<Option<ArticulatedGimbalMechanism>, RunnerError> {
    let Some(mechanism_id) = joint.mechanism_id.as_ref() else {
        return Ok(None);
    };
    let Some(contact) = document.contact.as_ref() else {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal mechanism `{mechanism_id}` requires a [contact] block"
            ),
        });
    };
    let mechanism = contact
        .mechanisms
        .iter()
        .find(|mechanism| mechanism.id == *mechanism_id)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("articulated gimbal mechanism `{mechanism_id}` was not declared"),
        })?;
    if mechanism.coordinate != joint.engine_id {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal mechanism `{mechanism_id}` coordinate `{}` does not match engine `{}`",
                mechanism.coordinate, joint.engine_id
            ),
        });
    }
    Ok(Some(ArticulatedGimbalMechanism {
        id: mechanism.id.clone(),
        axis: joint.mechanism_axis,
        elements: vec![scalar_mechanism_element_from_config(mechanism)?],
    }))
}

fn scalar_mechanism_element_from_config(
    mechanism: &ContactMechanismConfig,
) -> Result<ScalarMechanismElement, RunnerError> {
    let element = match mechanism.kind {
        ContactMechanismKindConfig::Stop => {
            let side = mechanism
                .side
                .ok_or_else(|| mechanism_config_error(&mechanism.id, "side"))?;
            let limit = mechanism
                .limit_rad
                .ok_or_else(|| mechanism_config_error(&mechanism.id, "limit_rad"))?;
            let stiffness = mechanism
                .stiffness_n_m
                .ok_or_else(|| mechanism_config_error(&mechanism.id, "stiffness_n_m"))?;
            ScalarStop::new(
                limit,
                side.into(),
                stiffness,
                mechanism.damping_n_s_m.unwrap_or(0.0),
            )
            .map(ScalarMechanismElement::stop)
            .map_err(|err| mechanism_contact_error(&mechanism.id, err))?
        }
        ContactMechanismKindConfig::Backlash => {
            let center = mechanism
                .center_rad
                .ok_or_else(|| mechanism_config_error(&mechanism.id, "center_rad"))?;
            let dead_zone_width = mechanism
                .dead_zone_width_rad
                .ok_or_else(|| mechanism_config_error(&mechanism.id, "dead_zone_width_rad"))?;
            let stiffness = mechanism
                .stiffness_n_m
                .ok_or_else(|| mechanism_config_error(&mechanism.id, "stiffness_n_m"))?;
            BacklashGap::new(
                center,
                dead_zone_width,
                stiffness,
                mechanism.damping_n_s_m.unwrap_or(0.0),
            )
            .map(ScalarMechanismElement::backlash)
            .map_err(|err| mechanism_contact_error(&mechanism.id, err))?
        }
        ContactMechanismKindConfig::Latch => {
            let center = mechanism
                .center_rad
                .ok_or_else(|| mechanism_config_error(&mechanism.id, "center_rad"))?;
            let half_width = mechanism
                .half_width_rad
                .ok_or_else(|| mechanism_config_error(&mechanism.id, "half_width_rad"))?;
            LatchWindow::new(center, half_width)
                .map(MonotoneLatch::new)
                .map(ScalarMechanismElement::latch)
                .map_err(|err| mechanism_contact_error(&mechanism.id, err))?
        }
    };
    Ok(element)
}

fn mechanism_config_error(id: &str, field: &'static str) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("articulated gimbal mechanism `{id}` is missing {field}"),
    }
}

fn mechanism_contact_error(id: &str, err: openbmp_contact::ContactError) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("articulated gimbal mechanism `{id}` failed: {err}"),
    }
}

fn articulated_gimbal_mechanism_latch_states(
    seed: &ArticulatedGimbalMultibodySeed,
) -> Vec<LatchState> {
    seed.mechanism
        .as_ref()
        .map(|mechanism| {
            vec![LatchState::disengaged(); scalar_mechanism_latch_count(&mechanism.elements)]
        })
        .unwrap_or_default()
}

fn articulated_gimbal_multibody_seed_from_config_with_joint_state(
    body: BodyId,
    engine: EngineId,
    joint: &MultiBodyGimbalJointConfig,
    mechanism: Option<ArticulatedGimbalMechanism>,
    parent_state: &RigidBodyState,
    joint_angles_rad: &[f64],
    joint_rates_rad_s: &[f64],
) -> Result<ArticulatedGimbalMultibodySeed, RunnerError> {
    let joint_dof = articulated_gimbal_joint_dof(joint);
    if joint_angles_rad.len() != joint_dof || joint_rates_rad_s.len() != joint_dof {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal joint state for engine {} has {} q and {} qd entries (expected {joint_dof})",
                engine.value(),
                joint_angles_rad.len(),
                joint_rates_rad_s.len()
            ),
        });
    }
    let root_inertia =
        SpatialInertia::from_mass_properties(&parent_state.mass_props).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("articulated gimbal root spatial inertia failed: {err}"),
            }
        })?;
    let engine_mass_properties = MassProperties::new(
        Mass::new::<kilogram>(joint.engine_mass_kg),
        Position3::<Body>::new(
            joint.engine_cg_body_m[0],
            joint.engine_cg_body_m[1],
            joint.engine_cg_body_m[2],
        ),
        nalgebra::Matrix3::new(
            joint.engine_inertia_body_kg_m2[0][0],
            joint.engine_inertia_body_kg_m2[0][1],
            joint.engine_inertia_body_kg_m2[0][2],
            joint.engine_inertia_body_kg_m2[1][0],
            joint.engine_inertia_body_kg_m2[1][1],
            joint.engine_inertia_body_kg_m2[1][2],
            joint.engine_inertia_body_kg_m2[2][0],
            joint.engine_inertia_body_kg_m2[2][1],
            joint.engine_inertia_body_kg_m2[2][2],
        ),
    );
    let engine_inertia =
        SpatialInertia::from_mass_properties(&engine_mass_properties).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("articulated gimbal engine spatial inertia failed: {err}"),
            }
        })?;
    let pivot = Vector3::new(
        joint.pivot_body_m[0],
        joint.pivot_body_m[1],
        joint.pivot_body_m[2],
    );
    let root_spec = TreeBodySpec {
        id: body,
        parent: None,
        joint: Joint::FreeFlyer,
        inertia: root_inertia,
        parent_to_body: PluckerTransform::identity(),
    };
    let primary_joint = Joint::revolute(Vector3::new(
        joint.axis_body[0],
        joint.axis_body[1],
        joint.axis_body[2],
    ))
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("articulated gimbal joint axis failed: {err}"),
    })?;
    let pivot_transform =
        PluckerTransform::new(nalgebra::Matrix3::identity(), pivot).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("articulated gimbal pivot transform failed: {err}"),
            }
        })?;
    let tree_specs = if let Some(secondary_axis_body) = joint.secondary_axis_body {
        let frame_inertia = SpatialInertia::from_matrix(SpatialMatrix::zeros()).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("articulated gimbal frame spatial inertia failed: {err}"),
            }
        })?;
        let secondary_joint = Joint::revolute(Vector3::new(
            secondary_axis_body[0],
            secondary_axis_body[1],
            secondary_axis_body[2],
        ))
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("articulated gimbal secondary joint axis failed: {err}"),
        })?;
        vec![
            root_spec,
            TreeBodySpec {
                id: articulated_gimbal_frame_body_id(engine),
                parent: Some(openbmp_multibody::BodyIndex::new(0)),
                joint: secondary_joint,
                inertia: frame_inertia,
                parent_to_body: pivot_transform,
            },
            TreeBodySpec {
                id: articulated_engine_body_id(engine),
                parent: Some(openbmp_multibody::BodyIndex::new(1)),
                joint: primary_joint,
                inertia: engine_inertia,
                parent_to_body: PluckerTransform::identity(),
            },
        ]
    } else {
        vec![
            root_spec,
            TreeBodySpec {
                id: articulated_engine_body_id(engine),
                parent: Some(openbmp_multibody::BodyIndex::new(0)),
                joint: primary_joint,
                inertia: engine_inertia,
                parent_to_body: pivot_transform,
            },
        ]
    };
    let tree = MultibodyTree::new(tree_specs).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("articulated gimbal multibody tree failed: {err}"),
    })?;

    let orientation = parent_state.orientation.q.into_inner();
    let linear_velocity_body = parent_state
        .orientation
        .inverse()
        .rotate_velocity(parent_state.velocity)
        .vector;
    let mut q = vec![
        orientation.w,
        orientation.i,
        orientation.j,
        orientation.k,
        parent_state.position.vector.x,
        parent_state.position.vector.y,
        parent_state.position.vector.z,
    ];
    q.extend_from_slice(joint_angles_rad);
    let mut qd = vec![
        parent_state.angular_velocity.vector.x,
        parent_state.angular_velocity.vector.y,
        parent_state.angular_velocity.vector.z,
        linear_velocity_body.x,
        linear_velocity_body.y,
        linear_velocity_body.z,
    ];
    qd.extend_from_slice(joint_rates_rad_s);
    tree.sim_state_from_state(MultibodyState::new(parent_state.time, q, qd))
        .map(|sim_state| ArticulatedGimbalMultibodySeed {
            body,
            engine,
            joint: joint.clone(),
            mechanism,
            cfe_loops: Vec::new(),
            cfe_gimbal_locks: Vec::new(),
            cfe_planar_four_bars: Vec::new(),
            parent_mass_props: parent_state.mass_props,
            tree,
            sim_state,
        })
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("articulated gimbal multibody state failed: {err}"),
        })
}

fn articulated_gimbal_root_authority_seed_from_joint_states(
    joint_specs: &[ArticulatedGimbalRootAuthorityJointSpec],
    parent_state: &RigidBodyState,
    joint_states: &[ArticulatedGimbalJointState],
) -> Result<ArticulatedGimbalRootAuthoritySeed, RunnerError> {
    if joint_specs.len() != joint_states.len() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "coupled articulated gimbal root-authority seed has {} joints and {} joint-state entries",
                joint_specs.len(),
                joint_states.len()
            ),
        });
    }
    let Some(first_joint) = joint_specs.first() else {
        return Err(RunnerError::UnsupportedScenario {
            what: "coupled articulated gimbal root-authority seed requires at least one gimbal"
                .to_owned(),
        });
    };
    let body = first_joint.body;
    if joint_specs.iter().any(|joint| joint.body != body) {
        return Err(RunnerError::UnsupportedScenario {
            what: "coupled articulated gimbal root-authority seed requires one parent body"
                .to_owned(),
        });
    }
    let root_inertia =
        SpatialInertia::from_mass_properties(&parent_state.mass_props).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("coupled articulated gimbal root spatial inertia failed: {err}"),
            }
        })?;
    let mut tree_specs = vec![TreeBodySpec {
        id: body,
        parent: None,
        joint: Joint::FreeFlyer,
        inertia: root_inertia,
        parent_to_body: PluckerTransform::identity(),
    }];
    let orientation = parent_state.orientation.q.into_inner();
    let linear_velocity_body = parent_state
        .orientation
        .inverse()
        .rotate_velocity(parent_state.velocity)
        .vector;
    let mut q = vec![
        orientation.w,
        orientation.i,
        orientation.j,
        orientation.k,
        parent_state.position.vector.x,
        parent_state.position.vector.y,
        parent_state.position.vector.z,
    ];
    let mut qd = vec![
        parent_state.angular_velocity.vector.x,
        parent_state.angular_velocity.vector.y,
        parent_state.angular_velocity.vector.z,
        linear_velocity_body.x,
        linear_velocity_body.y,
        linear_velocity_body.z,
    ];
    let mut joints = Vec::with_capacity(joint_specs.len());
    for (joint_spec, joint_state) in joint_specs.iter().zip(joint_states) {
        let joint = &joint_spec.joint;
        let joint_dof = articulated_gimbal_joint_dof(joint);
        if joint_state.angles_rad.len() != joint_dof || joint_state.rates_rad_s.len() != joint_dof {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "coupled articulated gimbal root-authority state for engine {} has {} q and {} qd entries (expected {joint_dof})",
                    joint_spec.engine.value(),
                    joint_state.angles_rad.len(),
                    joint_state.rates_rad_s.len()
                ),
            });
        }
        let q_start = q.len();
        let qd_start = qd.len();
        q.extend_from_slice(&joint_state.angles_rad);
        qd.extend_from_slice(&joint_state.rates_rad_s);

        let engine_mass_properties = MassProperties::new(
            Mass::new::<kilogram>(joint.engine_mass_kg),
            Position3::<Body>::new(
                joint.engine_cg_body_m[0],
                joint.engine_cg_body_m[1],
                joint.engine_cg_body_m[2],
            ),
            Matrix3::new(
                joint.engine_inertia_body_kg_m2[0][0],
                joint.engine_inertia_body_kg_m2[0][1],
                joint.engine_inertia_body_kg_m2[0][2],
                joint.engine_inertia_body_kg_m2[1][0],
                joint.engine_inertia_body_kg_m2[1][1],
                joint.engine_inertia_body_kg_m2[1][2],
                joint.engine_inertia_body_kg_m2[2][0],
                joint.engine_inertia_body_kg_m2[2][1],
                joint.engine_inertia_body_kg_m2[2][2],
            ),
        );
        let engine_inertia = SpatialInertia::from_mass_properties(&engine_mass_properties)
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!(
                    "coupled articulated gimbal engine {} spatial inertia failed: {err}",
                    joint_spec.engine.value()
                ),
            })?;
        let pivot = Vector3::new(
            joint.pivot_body_m[0],
            joint.pivot_body_m[1],
            joint.pivot_body_m[2],
        );
        let pivot_transform = PluckerTransform::new(Matrix3::identity(), pivot).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!(
                    "coupled articulated gimbal engine {} pivot transform failed: {err}",
                    joint_spec.engine.value()
                ),
            }
        })?;
        let primary_joint = Joint::revolute(Vector3::new(
            joint.axis_body[0],
            joint.axis_body[1],
            joint.axis_body[2],
        ))
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!(
                "coupled articulated gimbal engine {} primary joint failed: {err}",
                joint_spec.engine.value()
            ),
        })?;
        let terminal_body_index = if let Some(secondary_axis_body) = joint.secondary_axis_body {
            let frame_inertia =
                SpatialInertia::from_matrix(SpatialMatrix::zeros()).map_err(|err| {
                    RunnerError::UnsupportedScenario {
                        what: format!(
                            "coupled articulated gimbal engine {} frame inertia failed: {err}",
                            joint_spec.engine.value()
                        ),
                    }
                })?;
            let secondary_joint = Joint::revolute(Vector3::new(
                secondary_axis_body[0],
                secondary_axis_body[1],
                secondary_axis_body[2],
            ))
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!(
                    "coupled articulated gimbal engine {} secondary joint failed: {err}",
                    joint_spec.engine.value()
                ),
            })?;
            let frame_index = openbmp_multibody::BodyIndex::new(tree_specs.len());
            tree_specs.push(TreeBodySpec {
                id: articulated_gimbal_frame_body_id(joint_spec.engine),
                parent: Some(openbmp_multibody::BodyIndex::new(0)),
                joint: secondary_joint,
                inertia: frame_inertia,
                parent_to_body: pivot_transform,
            });
            let engine_index = openbmp_multibody::BodyIndex::new(tree_specs.len());
            tree_specs.push(TreeBodySpec {
                id: articulated_engine_body_id(joint_spec.engine),
                parent: Some(frame_index),
                joint: primary_joint,
                inertia: engine_inertia,
                parent_to_body: PluckerTransform::identity(),
            });
            engine_index
        } else {
            let engine_index = openbmp_multibody::BodyIndex::new(tree_specs.len());
            tree_specs.push(TreeBodySpec {
                id: articulated_engine_body_id(joint_spec.engine),
                parent: Some(openbmp_multibody::BodyIndex::new(0)),
                joint: primary_joint,
                inertia: engine_inertia,
                parent_to_body: pivot_transform,
            });
            engine_index
        };
        joints.push(ArticulatedGimbalRootAuthorityJoint {
            engine: joint_spec.engine,
            joint: joint.clone(),
            terminal_body_index,
            q_start,
            qd_start,
        });
    }
    let tree = MultibodyTree::new(tree_specs).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("coupled articulated gimbal root-authority tree failed: {err}"),
    })?;
    tree.sim_state_from_state(MultibodyState::new(parent_state.time, q, qd))
        .map(|sim_state| ArticulatedGimbalRootAuthoritySeed {
            body,
            joints,
            parent_mass_props: parent_state.mass_props,
            tree,
            sim_state,
        })
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("coupled articulated gimbal root-authority state failed: {err}"),
        })
}

fn articulated_gimbal_root_authority_joint_specs(
    seed: &ArticulatedGimbalRootAuthoritySeed,
) -> Vec<ArticulatedGimbalRootAuthorityJointSpec> {
    seed.joints
        .iter()
        .map(|joint| ArticulatedGimbalRootAuthorityJointSpec {
            body: seed.body,
            engine: joint.engine,
            joint: joint.joint.clone(),
        })
        .collect()
}

fn articulated_gimbal_root_authority_live_joint_states_from_previous(
    previous_seed: &ArticulatedGimbalRootAuthoritySeed,
    previous_live_joint_states: &[ArticulatedGimbalJointState],
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    dt_s: f64,
) -> Result<Vec<ArticulatedGimbalJointState>, RunnerError> {
    if previous_seed.joints.len() != previous_live_joint_states.len() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "coupled articulated gimbal root-authority live state has {} joints and {} live joint-state entries",
                previous_seed.joints.len(),
                previous_live_joint_states.len()
            ),
        });
    }
    previous_seed
        .joints
        .iter()
        .zip(previous_live_joint_states)
        .map(|(joint, previous_live)| {
            articulated_gimbal_joint_state_from_engine_thrust_for_joint(
                joint.engine,
                &joint.joint,
                &previous_live.angles_rad,
                engine_snapshot,
                dt_s,
            )
        })
        .collect()
}

fn articulated_gimbal_root_authority_seed_from_authoritative_forecast(
    forecast: &ArticulatedGimbalRootAuthoritySeed,
    parent_state: &RigidBodyState,
) -> Result<ArticulatedGimbalRootAuthoritySeed, RunnerError> {
    let mut joint_specs = Vec::with_capacity(forecast.joints.len());
    let mut joint_states = Vec::with_capacity(forecast.joints.len());
    for joint in &forecast.joints {
        let joint_dof = articulated_gimbal_joint_dof(&joint.joint);
        let joint_angles_rad = forecast
            .sim_state
            .q()
            .get(joint.q_start..joint.q_start + joint_dof)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "coupled articulated gimbal authoritative forecast for engine {} is missing joint coordinates",
                    joint.engine.value()
                ),
            })?;
        let joint_rates_rad_s = forecast
            .sim_state
            .qd()
            .get(joint.qd_start..joint.qd_start + joint_dof)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "coupled articulated gimbal authoritative forecast for engine {} is missing joint rates",
                    joint.engine.value()
                ),
            })?;
        joint_states.push(ArticulatedGimbalJointState {
            angles_rad: joint_angles_rad.to_vec(),
            rates_rad_s: joint_rates_rad_s.to_vec(),
        });
        joint_specs.push(ArticulatedGimbalRootAuthorityJointSpec {
            body: forecast.body,
            engine: joint.engine,
            joint: joint.joint.clone(),
        });
    }
    articulated_gimbal_root_authority_seed_from_joint_states(
        &joint_specs,
        parent_state,
        &joint_states,
    )
}

fn articulated_gimbal_root_authority_derivative_from_engine_thrust(
    seed: &ArticulatedGimbalRootAuthoritySeed,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
) -> Result<MultibodyDerivative, RunnerError> {
    articulated_gimbal_root_authority_derivative_from_engine_thrust_at_state(
        seed,
        &seed.sim_state,
        engine_snapshot,
    )
}

fn articulated_gimbal_root_authority_derivative_from_engine_thrust_at_state(
    seed: &ArticulatedGimbalRootAuthoritySeed,
    sim_state: &MultibodySimState,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
) -> Result<MultibodyDerivative, RunnerError> {
    let tree = &seed.tree;
    let mut external_forces_body = vec![SpatialForce::zero(); tree.bodies().len()];
    for joint in &seed.joints {
        let snapshot =
            engine_snapshot
                .get(&joint.engine)
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: format!(
                        "coupled articulated gimbal engine {} snapshot is missing",
                        joint.engine.value()
                    ),
                })?;
        let child_transform = tree
            .body_transform_parent_to_child_at_state(joint.terminal_body_index, sim_state.state())
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!(
                    "coupled articulated gimbal engine {} child transform failed: {err}",
                    joint.engine.value()
                ),
            })?;
        let thrust_child_n = child_transform.rot_child_from_parent() * snapshot.thrust_body;
        let thrust_application_child_m = Vector3::new(
            joint.joint.thrust_application_body_m[0],
            joint.joint.thrust_application_body_m[1],
            joint.joint.thrust_application_body_m[2],
        );
        let thrust_moment_child_n_m = thrust_application_child_m.cross(&thrust_child_n);
        external_forces_body[joint.terminal_body_index.index()] =
            SpatialForce::new(thrust_moment_child_n_m, thrust_child_n);
        add_articulated_gimbal_tvc_inertial_reaction_to_root(snapshot, &mut external_forces_body)?;
    }
    let mut generalized_forces = vec![0.0; tree.n_qd()];
    for joint in &seed.joints {
        let snapshot =
            engine_snapshot
                .get(&joint.engine)
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: format!(
                        "coupled articulated gimbal engine {} snapshot is missing",
                        joint.engine.value()
                    ),
                })?;
        apply_articulated_gimbal_root_authority_tvc_load_torque(
            joint,
            snapshot,
            &mut generalized_forces,
        )?;
    }
    tree.derivative_from_forward_dynamics(
        sim_state.state(),
        &generalized_forces,
        SpatialMotion::zero(),
        &external_forces_body,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("coupled articulated gimbal thrust derivative failed: {err}"),
    })
}

fn articulated_gimbal_root_authority_rk4_forecast_from_engine_thrust(
    seed: &ArticulatedGimbalRootAuthoritySeed,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    dt_s: f64,
) -> Result<ArticulatedGimbalRootAuthoritySeed, RunnerError> {
    let next_sim_state = Rk4FixedStep
        .advance(
            &seed.sim_state,
            |sim_state, _time| {
                articulated_gimbal_root_authority_derivative_from_engine_thrust_at_state(
                    seed,
                    sim_state,
                    engine_snapshot,
                )
                .map_err(multibody_shadow_model_eval_error)
            },
            Duration::from_seconds(dt_s),
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("coupled articulated gimbal RK4 forecast failed: {err}"),
        })?;
    Ok(ArticulatedGimbalRootAuthoritySeed {
        body: seed.body,
        joints: seed.joints.clone(),
        parent_mass_props: seed.parent_mass_props,
        tree: seed.tree.clone(),
        sim_state: next_sim_state,
    })
}

fn articulated_gimbal_root_authority_rigid_state_from_seed(
    seed: &ArticulatedGimbalRootAuthoritySeed,
) -> Result<RigidBodyState, RunnerError> {
    root_rigid_state_from_multibody_root_slots(seed.body, &seed.sim_state, seed.parent_mass_props)
}

fn articulated_gimbal_multibody_derivative_from_engine_thrust(
    seed: &ArticulatedGimbalMultibodySeed,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    dt_s: f64,
    mechanism_latch_states: &mut [LatchState],
) -> Result<ArticulatedGimbalDerivativeEvaluation, RunnerError> {
    articulated_gimbal_multibody_derivative_from_engine_thrust_at_state(
        seed,
        &seed.sim_state,
        engine_snapshot,
        dt_s,
        mechanism_latch_states,
    )
}

fn articulated_gimbal_multibody_derivative_from_engine_thrust_at_state(
    seed: &ArticulatedGimbalMultibodySeed,
    sim_state: &MultibodySimState,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    dt_s: f64,
    mechanism_latch_states: &mut [LatchState],
) -> Result<ArticulatedGimbalDerivativeEvaluation, RunnerError> {
    let tree = &seed.tree;
    let engine = seed.engine;
    let joint = &seed.joint;
    let body_count = tree.bodies().len();
    if !(2..=3).contains(&body_count) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal tree for engine {} has {} bodies (expected 2 or 3)",
                engine.value(),
                body_count
            ),
        });
    }
    let snapshot =
        engine_snapshot
            .get(&engine)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "articulated gimbal engine {} snapshot is missing",
                    engine.value()
                ),
            })?;
    let child_transform = tree
        .body_transform_parent_to_child_at_state(
            openbmp_multibody::BodyIndex::new(body_count - 1),
            sim_state.state(),
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("articulated gimbal child transform failed: {err}"),
        })?;
    let thrust_child_n = child_transform.rot_child_from_parent() * snapshot.thrust_body;
    let thrust_application_child_m = Vector3::new(
        joint.thrust_application_body_m[0],
        joint.thrust_application_body_m[1],
        joint.thrust_application_body_m[2],
    );
    let thrust_moment_child_n_m = thrust_application_child_m.cross(&thrust_child_n);
    let mut external_forces_body = vec![SpatialForce::zero(); body_count];
    external_forces_body[body_count - 1] =
        SpatialForce::new(thrust_moment_child_n_m, thrust_child_n);
    add_articulated_gimbal_tvc_inertial_reaction_to_root(snapshot, &mut external_forces_body)?;
    let mut generalized_forces = vec![0.0; tree.n_qd()];
    apply_articulated_gimbal_tvc_load_torque(
        joint,
        snapshot,
        sim_state.q().len(),
        sim_state.qd().len(),
        &mut generalized_forces,
    )?;
    if let Some(mechanism) = seed.mechanism.as_ref() {
        let q = sim_state.q();
        let qd = sim_state.qd();
        let (coordinate_index, rate_index) =
            articulated_gimbal_mechanism_coordinate_indices(joint, mechanism, q.len(), qd.len())?;
        let response = evaluate_scalar_mechanism(
            &mechanism.elements,
            mechanism_latch_states,
            q[coordinate_index],
            qd[rate_index],
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal mechanism {} failed: {err}",
                mechanism.id
            ),
        })?;
        generalized_forces[rate_index] += response.generalized_force;
    }
    let cfe_loops = articulated_gimbal_materialized_cfe_loops(seed, sim_state)?;
    let cfe_loop_solves = apply_articulated_gimbal_cfe_loops(
        tree,
        &cfe_loops,
        sim_state,
        &mut generalized_forces,
        &external_forces_body,
        dt_s,
    )?;
    let derivative = tree
        .derivative_from_forward_dynamics(
            sim_state.state(),
            &generalized_forces,
            SpatialMotion::zero(),
            &external_forces_body,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("articulated gimbal thrust derivative failed: {err}"),
        })?;
    Ok(ArticulatedGimbalDerivativeEvaluation {
        derivative,
        cfe_loop_solves,
    })
}

fn apply_articulated_gimbal_tvc_load_torque(
    joint: &MultiBodyGimbalJointConfig,
    snapshot: &openbmp_sim::EngineSnapshot,
    q_len: usize,
    qd_len: usize,
    generalized_forces: &mut [f64],
) -> Result<(), RunnerError> {
    let primary = articulated_gimbal_axis_coordinate_indices(
        joint,
        MultiBodyGimbalMechanismAxisConfig::Primary,
        q_len,
        qd_len,
        "articulated gimbal TVC load torque",
    )?
    .1;
    let secondary = if joint.secondary_axis_body.is_some() {
        Some(
            articulated_gimbal_axis_coordinate_indices(
                joint,
                MultiBodyGimbalMechanismAxisConfig::Secondary,
                q_len,
                qd_len,
                "articulated gimbal TVC load torque",
            )?
            .1,
        )
    } else {
        None
    };
    apply_articulated_gimbal_tvc_load_torque_at_indices(
        joint,
        snapshot,
        primary,
        secondary,
        generalized_forces,
    )
}

fn apply_articulated_gimbal_root_authority_tvc_load_torque(
    joint: &ArticulatedGimbalRootAuthorityJoint,
    snapshot: &openbmp_sim::EngineSnapshot,
    generalized_forces: &mut [f64],
) -> Result<(), RunnerError> {
    let joint_dof = articulated_gimbal_joint_dof(&joint.joint);
    let primary = joint.qd_start + joint_dof.saturating_sub(1);
    let secondary = if joint.joint.secondary_axis_body.is_some() {
        Some(joint.qd_start)
    } else {
        None
    };
    apply_articulated_gimbal_tvc_load_torque_at_indices(
        &joint.joint,
        snapshot,
        primary,
        secondary,
        generalized_forces,
    )
}

fn apply_articulated_gimbal_tvc_load_torque_at_indices(
    joint: &MultiBodyGimbalJointConfig,
    snapshot: &openbmp_sim::EngineSnapshot,
    primary_rate_index: usize,
    secondary_rate_index: Option<usize>,
    generalized_forces: &mut [f64],
) -> Result<(), RunnerError> {
    let load_torque_parent_n_m = articulated_gimbal_tvc_load_torque_parent(snapshot)?;
    if load_torque_parent_n_m.norm() <= ARTICULATED_GIMBAL_THRUST_DIRECTION_EPS {
        return Ok(());
    }
    let primary_axis = normalized_vec3(
        joint.axis_body,
        "articulated gimbal TVC load torque primary axis is invalid",
    )?;
    let Some(primary_force) = generalized_forces.get_mut(primary_rate_index) else {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal TVC load torque primary index {primary_rate_index} exceeds qd dimension {}",
                generalized_forces.len()
            ),
        });
    };
    *primary_force += load_torque_parent_n_m.dot(&primary_axis);
    if let Some(secondary_rate_index) = secondary_rate_index {
        let secondary_axis_body =
            joint.secondary_axis_body
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "articulated gimbal TVC load torque selected secondary axis without a two-axis gimbal".to_owned(),
                })?;
        let secondary_axis = normalized_vec3(
            secondary_axis_body,
            "articulated gimbal TVC load torque secondary axis is invalid",
        )?;
        let qd_dimension = generalized_forces.len();
        let Some(secondary_force) = generalized_forces.get_mut(secondary_rate_index) else {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "articulated gimbal TVC load torque secondary index {secondary_rate_index} exceeds qd dimension {qd_dimension}",
                ),
            });
        };
        *secondary_force += load_torque_parent_n_m.dot(&secondary_axis);
    }
    Ok(())
}

fn articulated_gimbal_tvc_load_torque_parent(
    snapshot: &openbmp_sim::EngineSnapshot,
) -> Result<Vector3<f64>, RunnerError> {
    if ![
        snapshot.tvc_pitch_load_torque_n_m,
        snapshot.tvc_yaw_load_torque_n_m,
    ]
    .iter()
    .all(|value| value.is_finite())
    {
        return Err(RunnerError::UnsupportedScenario {
            what: "articulated gimbal TVC load torque snapshot is non-finite".to_owned(),
        });
    }
    Ok(Vector3::new(
        snapshot.tvc_yaw_load_torque_n_m,
        snapshot.tvc_pitch_load_torque_n_m,
        0.0,
    ))
}

fn add_articulated_gimbal_tvc_inertial_reaction_to_root(
    snapshot: &openbmp_sim::EngineSnapshot,
    external_forces_body: &mut [SpatialForce],
) -> Result<(), RunnerError> {
    let reaction_moment_body_n_m = articulated_gimbal_tvc_inertial_reaction_parent(snapshot)?;
    if reaction_moment_body_n_m.norm() <= ARTICULATED_GIMBAL_THRUST_DIRECTION_EPS {
        return Ok(());
    }
    let Some(root_force) = external_forces_body.get_mut(0) else {
        return Err(RunnerError::UnsupportedScenario {
            what: "articulated gimbal TVC inertial reaction requires a root body".to_owned(),
        });
    };
    *root_force = SpatialForce::new(
        root_force.moment_n_m() + reaction_moment_body_n_m,
        root_force.force_n(),
    );
    Ok(())
}

fn articulated_gimbal_tvc_inertial_reaction_parent(
    snapshot: &openbmp_sim::EngineSnapshot,
) -> Result<Vector3<f64>, RunnerError> {
    if ![
        snapshot.tvc_pitch_inertial_reaction_torque_n_m,
        snapshot.tvc_yaw_inertial_reaction_torque_n_m,
    ]
    .iter()
    .all(|value| value.is_finite())
    {
        return Err(RunnerError::UnsupportedScenario {
            what: "articulated gimbal TVC inertial reaction snapshot is non-finite".to_owned(),
        });
    }
    Ok(Vector3::new(
        snapshot.tvc_yaw_inertial_reaction_torque_n_m,
        snapshot.tvc_pitch_inertial_reaction_torque_n_m,
        0.0,
    ))
}

fn articulated_gimbal_mechanism_coordinate_indices(
    joint: &MultiBodyGimbalJointConfig,
    mechanism: &ArticulatedGimbalMechanism,
    q_len: usize,
    qd_len: usize,
) -> Result<(usize, usize), RunnerError> {
    articulated_gimbal_axis_coordinate_indices(
        joint,
        mechanism.axis,
        q_len,
        qd_len,
        &format!("articulated gimbal mechanism {}", mechanism.id),
    )
}

fn articulated_gimbal_axis_coordinate_indices(
    joint: &MultiBodyGimbalJointConfig,
    axis: MultiBodyGimbalMechanismAxisConfig,
    q_len: usize,
    qd_len: usize,
    label: &str,
) -> Result<(usize, usize), RunnerError> {
    let joint_dof = articulated_gimbal_joint_dof(joint);
    if joint_dof == 0 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("{label} cannot bind to malformed gimbal state"),
        });
    }
    let axis_offset = match axis {
        MultiBodyGimbalMechanismAxisConfig::Primary => joint_dof - 1,
        MultiBodyGimbalMechanismAxisConfig::Secondary => {
            if joint_dof < 2 {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!("{label} selected secondary axis without a two-axis gimbal"),
                });
            }
            0
        }
    };
    let q_start = q_len
        .checked_sub(joint_dof)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("{label} has no selected coordinate"),
        })?;
    let qd_start =
        qd_len
            .checked_sub(joint_dof)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!("{label} has no selected coordinate rate"),
            })?;
    let coordinate_index = q_start + axis_offset;
    let rate_index = qd_start + axis_offset;
    if coordinate_index < 7 || rate_index < 6 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("{label} cannot bind to malformed gimbal state"),
        });
    }
    Ok((coordinate_index, rate_index))
}

fn articulated_gimbal_cfe_loops_from_config(
    cfe_loops: &[MultiBodyCfeLoopConfig],
    generalized_dof_count: usize,
) -> Result<Vec<ArticulatedGimbalCfeLoop>, RunnerError> {
    let mut rows = Vec::with_capacity(cfe_loops.len());
    for cfe_loop in cfe_loops {
        let mut jacobian_row = vec![0.0; generalized_dof_count];
        for entry in &cfe_loop.jacobian_entries {
            if entry.generalized_velocity_index >= generalized_dof_count {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!(
                        "multi_body.cfe_loop `{}` generalized_velocity_index {} is outside articulated gimbal qd dimension {}",
                        cfe_loop.loop_id, entry.generalized_velocity_index, generalized_dof_count
                    ),
                });
            }
            jacobian_row[entry.generalized_velocity_index] = entry.jacobian_value;
        }
        rows.push(ArticulatedGimbalCfeLoop {
            id: cfe_loop.loop_id.clone(),
            position_residual: cfe_loop.position_residual,
            velocity_residual_per_s: cfe_loop.velocity_residual_per_s,
            velocity_bias_acceleration_per_s2: cfe_loop.velocity_bias_acceleration_per_s2,
            jacobian_row,
        });
    }
    Ok(rows)
}

fn articulated_gimbal_cfe_gimbal_locks_from_config(
    cfe_gimbal_locks: &[MultiBodyCfeGimbalLockConfig],
    engine: EngineId,
) -> Result<Vec<ArticulatedGimbalCfeGimbalLock>, RunnerError> {
    let mut locks = Vec::with_capacity(cfe_gimbal_locks.len());
    for lock in cfe_gimbal_locks {
        let lock_engine = engine_id_from_scenario_text(&lock.engine_id);
        if lock_engine != engine {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "multi_body.cfe_gimbal_lock `{}` targets engine {} but this articulated shadow is {}",
                    lock.loop_id,
                    lock_engine.value(),
                    engine.value()
                ),
            });
        }
        locks.push(ArticulatedGimbalCfeGimbalLock {
            id: lock.loop_id.clone(),
            axis: lock.axis,
            locked_angle_rad: lock.locked_angle_rad,
            velocity_bias_acceleration_per_s2: lock.velocity_bias_acceleration_per_s2,
        });
    }
    Ok(locks)
}

fn articulated_gimbal_cfe_planar_four_bars_from_config(
    cfe_planar_four_bars: &[MultiBodyCfePlanarFourBarConfig],
    engine: EngineId,
) -> Result<Vec<ArticulatedGimbalCfePlanarFourBar>, RunnerError> {
    let mut four_bars = Vec::with_capacity(cfe_planar_four_bars.len());
    for four_bar in cfe_planar_four_bars {
        let four_bar_engine = engine_id_from_scenario_text(&four_bar.engine_id);
        if four_bar_engine != engine {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "multi_body.cfe_planar_four_bar `{}` targets engine {} but this articulated shadow is {}",
                    four_bar.loop_id,
                    four_bar_engine.value(),
                    engine.value()
                ),
            });
        }
        let loop_geometry = PlanarFourBarLoop::new(
            four_bar.ground_pivot_a_m,
            four_bar.ground_pivot_d_m,
            four_bar.input_link_m,
            four_bar.coupler_link_m,
            four_bar.output_link_m,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!(
                "multi_body.cfe_planar_four_bar `{}` geometry failed: {err}",
                four_bar.loop_id
            ),
        })?;
        four_bars.push(ArticulatedGimbalCfePlanarFourBar {
            id: four_bar.loop_id.clone(),
            input_axis: four_bar.input_axis,
            output_axis: four_bar.output_axis,
            loop_geometry,
            velocity_bias_acceleration_per_s2: four_bar.velocity_bias_acceleration_per_s2,
        });
    }
    Ok(four_bars)
}

fn articulated_gimbal_materialized_cfe_loops(
    seed: &ArticulatedGimbalMultibodySeed,
    sim_state: &MultibodySimState,
) -> Result<Vec<ArticulatedGimbalCfeLoop>, RunnerError> {
    let mut loops = seed.cfe_loops.clone();
    if seed.cfe_gimbal_locks.is_empty() && seed.cfe_planar_four_bars.is_empty() {
        return Ok(loops);
    }
    let generalized_dof_count = seed.tree.n_qd();
    for lock in &seed.cfe_gimbal_locks {
        let (coordinate_index, rate_index) = articulated_gimbal_axis_coordinate_indices(
            &seed.joint,
            lock.axis,
            sim_state.q().len(),
            sim_state.qd().len(),
            &format!("multi_body.cfe_gimbal_lock `{}`", lock.id),
        )?;
        let mut jacobian_row = vec![0.0; generalized_dof_count];
        jacobian_row[rate_index] = 1.0;
        loops.push(ArticulatedGimbalCfeLoop {
            id: lock.id.clone(),
            position_residual: sim_state.q()[coordinate_index] - lock.locked_angle_rad,
            velocity_residual_per_s: sim_state.qd()[rate_index],
            velocity_bias_acceleration_per_s2: lock.velocity_bias_acceleration_per_s2,
            jacobian_row,
        });
    }
    for four_bar in &seed.cfe_planar_four_bars {
        let (input_coordinate_index, input_rate_index) =
            articulated_gimbal_axis_coordinate_indices(
                &seed.joint,
                four_bar.input_axis,
                sim_state.q().len(),
                sim_state.qd().len(),
                &format!(
                    "multi_body.cfe_planar_four_bar `{}` input_axis",
                    four_bar.id
                ),
            )?;
        let (output_coordinate_index, output_rate_index) =
            articulated_gimbal_axis_coordinate_indices(
                &seed.joint,
                four_bar.output_axis,
                sim_state.q().len(),
                sim_state.qd().len(),
                &format!(
                    "multi_body.cfe_planar_four_bar `{}` output_axis",
                    four_bar.id
                ),
            )?;
        if input_rate_index == output_rate_index {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "multi_body.cfe_planar_four_bar `{}` input_axis and output_axis select the same generalized velocity",
                    four_bar.id
                ),
            });
        }
        let row = four_bar
            .loop_geometry
            .evaluate(
                sim_state.q()[input_coordinate_index],
                sim_state.q()[output_coordinate_index],
                sim_state.qd()[input_rate_index],
                sim_state.qd()[output_rate_index],
            )
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!(
                    "multi_body.cfe_planar_four_bar `{}` materialization failed: {err}",
                    four_bar.id
                ),
            })?;
        let mut jacobian_row = vec![0.0; generalized_dof_count];
        jacobian_row[input_rate_index] = row.jacobian_row[0];
        jacobian_row[output_rate_index] = row.jacobian_row[1];
        loops.push(ArticulatedGimbalCfeLoop {
            id: four_bar.id.clone(),
            position_residual: row.position_residual_m,
            velocity_residual_per_s: row.velocity_residual_m_s,
            velocity_bias_acceleration_per_s2: four_bar.velocity_bias_acceleration_per_s2,
            jacobian_row,
        });
    }
    Ok(loops)
}

fn apply_articulated_gimbal_cfe_loops(
    tree: &MultibodyTree,
    cfe_loops: &[ArticulatedGimbalCfeLoop],
    sim_state: &MultibodySimState,
    generalized_forces: &mut [f64],
    external_forces_body: &[SpatialForce],
    dt_s: f64,
) -> Result<Vec<ArticulatedGimbalCfeLoopSolveSample>, RunnerError> {
    if cfe_loops.is_empty() {
        return Ok(Vec::new());
    }
    let generalized_dof_count = tree.n_qd();
    if generalized_forces.len() != generalized_dof_count {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal CFE generalized force dimension {} does not match tree qd dimension {generalized_dof_count}",
                generalized_forces.len()
            ),
        });
    }
    let constraint_count = cfe_loops.len();
    let mut position_residuals = Vec::with_capacity(constraint_count);
    let mut velocity_residuals_per_s = Vec::with_capacity(constraint_count);
    let mut velocity_bias_accelerations_per_s2 = Vec::with_capacity(constraint_count);
    let mut constraint_jacobian_row_major =
        Vec::with_capacity(constraint_count * generalized_dof_count);
    for cfe_loop in cfe_loops {
        position_residuals.push(cfe_loop.position_residual);
        velocity_residuals_per_s.push(cfe_loop.velocity_residual_per_s);
        velocity_bias_accelerations_per_s2.push(cfe_loop.velocity_bias_acceleration_per_s2);
        constraint_jacobian_row_major.extend_from_slice(&cfe_loop.jacobian_row);
    }

    let assembly = tree
        .assemble_cfe_loop_terms_at_state(
            sim_state.state(),
            generalized_forces,
            SpatialMotion::zero(),
            external_forces_body,
            &position_residuals,
            &velocity_residuals_per_s,
            &velocity_bias_accelerations_per_s2,
            &constraint_jacobian_row_major,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("articulated gimbal CFE loop assembly failed: {err}"),
        })?;
    let system = TreeCfeLoopSystem::new(
        assembly.generalized_dof_count(),
        assembly.position_residuals(),
        assembly.velocity_residuals_per_s(),
        assembly.unconstrained_constraint_accelerations_per_s2(),
        assembly.effective_inverse_mass_row_major(),
        assembly.constraint_jacobian_row_major(),
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("articulated gimbal CFE loop system failed: {err}"),
    })?;
    let mut matrix_scratch_row_major = vec![0.0; constraint_count * constraint_count];
    let mut rhs_scratch = vec![0.0; constraint_count];
    let mut lagrange_multipliers = vec![0.0; constraint_count];
    let mut stabilized_constraint_accelerations_per_s2 = vec![0.0; constraint_count];
    let mut acceleration_residuals_per_s2 = vec![0.0; constraint_count];
    let mut generalized_constraint_forces = vec![0.0; generalized_dof_count];
    let buffers = TreeCfeLoopSolveBuffers::new(
        constraint_count,
        generalized_dof_count,
        &mut matrix_scratch_row_major,
        &mut rhs_scratch,
        &mut lagrange_multipliers,
        &mut stabilized_constraint_accelerations_per_s2,
        &mut acceleration_residuals_per_s2,
        &mut generalized_constraint_forces,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("articulated gimbal CFE loop buffers failed: {err}"),
    })?;
    solve_tree_loop_baumgarte_cfe(
        articulated_gimbal_cfe_baumgarte_config(dt_s)?,
        system,
        buffers,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("articulated gimbal CFE loop solve failed: {err}"),
    })?;
    let solve_samples = cfe_loops
        .iter()
        .enumerate()
        .map(|(index, cfe_loop)| ArticulatedGimbalCfeLoopSolveSample {
            loop_id: cfe_loop.id.clone(),
            lagrange_multiplier: lagrange_multipliers[index],
            stabilized_acceleration_per_s2: stabilized_constraint_accelerations_per_s2[index],
            acceleration_residual_per_s2: acceleration_residuals_per_s2[index],
        })
        .collect();
    for (force, constraint_force) in generalized_forces
        .iter_mut()
        .zip(generalized_constraint_forces.iter())
    {
        *force += *constraint_force;
    }
    Ok(solve_samples)
}

fn articulated_gimbal_cfe_baumgarte_config(dt_s: f64) -> Result<BaumgarteCfeConfig, RunnerError> {
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("articulated gimbal CFE loop requires positive finite dt_s, got {dt_s}"),
        });
    }
    let gain_rad_s = 1.0 / dt_s;
    BaumgarteCfeConfig::new(gain_rad_s, gain_rad_s).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!("articulated gimbal CFE Baumgarte gains failed: {err}"),
        }
    })
}

fn articulated_gimbal_multibody_rk4_forecast_from_engine_thrust(
    seed: &ArticulatedGimbalMultibodySeed,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    dt_s: f64,
    mechanism_latch_states: &[LatchState],
) -> Result<ArticulatedGimbalMultibodySeed, RunnerError> {
    let next_sim_state = Rk4FixedStep
        .advance(
            &seed.sim_state,
            |sim_state, _time| {
                let mut stage_latches = mechanism_latch_states.to_vec();
                articulated_gimbal_multibody_derivative_from_engine_thrust_at_state(
                    seed,
                    sim_state,
                    engine_snapshot,
                    dt_s,
                    &mut stage_latches,
                )
                .map(|evaluation| evaluation.derivative)
                .map_err(multibody_shadow_model_eval_error)
            },
            Duration::from_seconds(dt_s),
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("articulated gimbal multibody RK4 forecast failed: {err}"),
        })?;
    Ok(ArticulatedGimbalMultibodySeed {
        body: seed.body,
        engine: seed.engine,
        joint: seed.joint.clone(),
        mechanism: seed.mechanism.clone(),
        cfe_loops: seed.cfe_loops.clone(),
        cfe_gimbal_locks: seed.cfe_gimbal_locks.clone(),
        cfe_planar_four_bars: seed.cfe_planar_four_bars.clone(),
        parent_mass_props: seed.parent_mass_props,
        tree: seed.tree.clone(),
        sim_state: next_sim_state,
    })
}

fn articulated_gimbal_multibody_seed_from_authoritative_forecast(
    forecast: &ArticulatedGimbalMultibodySeed,
    parent_state: &RigidBodyState,
) -> Result<ArticulatedGimbalMultibodySeed, RunnerError> {
    let joint_dof = articulated_gimbal_joint_dof(&forecast.joint);
    let joint_angles_rad =
        forecast
            .sim_state
            .q()
            .get(7..)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "articulated gimbal authoritative forecast is missing joint coordinates"
                    .to_owned(),
            })?;
    let joint_rates_rad_s =
        forecast
            .sim_state
            .qd()
            .get(6..)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "articulated gimbal authoritative forecast is missing joint rates".to_owned(),
            })?;
    if joint_angles_rad.len() != joint_dof || joint_rates_rad_s.len() != joint_dof {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal authoritative forecast for engine {} has {} q and {} qd joint entries (expected {joint_dof})",
                forecast.engine.value(),
                joint_angles_rad.len(),
                joint_rates_rad_s.len()
            ),
        });
    }
    let mut seed = articulated_gimbal_multibody_seed_from_config_with_joint_state(
        forecast.body,
        forecast.engine,
        &forecast.joint,
        forecast.mechanism.clone(),
        parent_state,
        joint_angles_rad,
        joint_rates_rad_s,
    )?;
    seed.cfe_loops = forecast.cfe_loops.clone();
    seed.cfe_gimbal_locks = forecast.cfe_gimbal_locks.clone();
    seed.cfe_planar_four_bars = forecast.cfe_planar_four_bars.clone();
    Ok(seed)
}

fn articulated_gimbal_root_rigid_state_from_seed(
    seed: &ArticulatedGimbalMultibodySeed,
) -> Result<RigidBodyState, RunnerError> {
    root_rigid_state_from_multibody_root_slots(seed.body, &seed.sim_state, seed.parent_mass_props)
}

fn articulated_gimbal_joint_state_from_engine_thrust(
    seed: &ArticulatedGimbalMultibodySeed,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    dt_s: f64,
) -> Result<ArticulatedGimbalJointState, RunnerError> {
    let joint_dof = articulated_gimbal_joint_dof(&seed.joint);
    let previous_angles =
        seed.sim_state
            .q()
            .get(7..)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "articulated gimbal state is missing joint coordinates".to_owned(),
            })?;
    if previous_angles.len() != joint_dof {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal state for engine {} has {} joint coordinates (expected {joint_dof})",
                seed.engine.value(),
                previous_angles.len()
            ),
        });
    }
    articulated_gimbal_joint_state_from_engine_thrust_for_joint(
        seed.engine,
        &seed.joint,
        previous_angles,
        engine_snapshot,
        dt_s,
    )
}

fn articulated_gimbal_joint_state_from_engine_thrust_for_joint(
    engine: EngineId,
    joint: &MultiBodyGimbalJointConfig,
    previous_angles: &[f64],
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    dt_s: f64,
) -> Result<ArticulatedGimbalJointState, RunnerError> {
    let joint_dof = articulated_gimbal_joint_dof(joint);
    if previous_angles.len() != joint_dof {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal state for engine {} has {} joint coordinates (expected {joint_dof})",
                engine.value(),
                previous_angles.len()
            ),
        });
    }
    let snapshot =
        engine_snapshot
            .get(&engine)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "articulated gimbal engine {} snapshot is missing",
                    engine.value()
                ),
            })?;
    if let Some(joint_state) =
        articulated_gimbal_explicit_joint_state_from_snapshot(joint, snapshot)
    {
        return Ok(joint_state);
    }
    let thrust_norm = snapshot.thrust_body.norm();
    if !thrust_norm.is_finite() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal engine {} thrust snapshot is non-finite",
                engine.value()
            ),
        });
    }
    if thrust_norm > ARTICULATED_GIMBAL_THRUST_DIRECTION_EPS {
        return articulated_gimbal_joint_state_from_unit_thrust_direction(
            joint,
            previous_angles,
            snapshot.thrust_body / thrust_norm,
            dt_s,
        );
    }
    if let Some(thrust_parent) = articulated_gimbal_snapshot_unit_thrust_direction(snapshot) {
        return articulated_gimbal_joint_state_from_unit_thrust_direction(
            joint,
            previous_angles,
            thrust_parent,
            dt_s,
        );
    }
    Ok(ArticulatedGimbalJointState {
        angles_rad: previous_angles.to_vec(),
        rates_rad_s: vec![0.0; joint_dof],
    })
}

fn articulated_gimbal_joint_state_from_unit_thrust_direction(
    joint: &MultiBodyGimbalJointConfig,
    previous_angles: &[f64],
    thrust_parent: Vector3<f64>,
    dt_s: f64,
) -> Result<ArticulatedGimbalJointState, RunnerError> {
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "articulated gimbal joint-state handoff requires positive finite dt".to_owned(),
        });
    }
    let axis = normalized_vec3(joint.axis_body, "articulated gimbal axis_body is invalid")?;
    let neutral = normalized_vec3(
        joint.neutral_thrust_body,
        "articulated gimbal neutral_thrust_body is invalid",
    )?;
    if let Some(secondary_axis_body) = joint.secondary_axis_body {
        let secondary = normalized_vec3(
            secondary_axis_body,
            "articulated gimbal secondary_axis_body is invalid",
        )?;
        let pitch_command = thrust_parent.dot(&secondary).clamp(-1.0, 1.0).asin();
        let yaw_command = (-thrust_parent.dot(&axis)).atan2(thrust_parent.dot(&neutral));
        let angles_rad = vec![-yaw_command, -pitch_command];
        let rates_rad_s = angles_rad
            .iter()
            .zip(previous_angles)
            .map(|(angle, previous_angle)| (angle - previous_angle) / dt_s)
            .collect();
        return Ok(ArticulatedGimbalJointState {
            angles_rad,
            rates_rad_s,
        });
    }
    let neutral_perp = vector_reject_axis(neutral, axis);
    let thrust_perp = vector_reject_axis(thrust_parent, axis);
    let neutral_perp_norm = neutral_perp.norm();
    let thrust_perp_norm = thrust_perp.norm();
    if neutral_perp_norm <= ARTICULATED_GIMBAL_THRUST_DIRECTION_EPS {
        return Err(RunnerError::UnsupportedScenario {
            what: "articulated gimbal neutral_thrust_body is parallel to axis_body".to_owned(),
        });
    }
    if thrust_perp_norm <= ARTICULATED_GIMBAL_THRUST_DIRECTION_EPS {
        return Ok(ArticulatedGimbalJointState {
            angles_rad: previous_angles.to_vec(),
            rates_rad_s: vec![0.0],
        });
    }
    let neutral_perp = neutral_perp / neutral_perp_norm;
    let thrust_perp = thrust_perp / thrust_perp_norm;
    let sin_angle = axis.dot(&thrust_perp.cross(&neutral_perp));
    let cos_angle = thrust_perp.dot(&neutral_perp);
    let angle = sin_angle.atan2(cos_angle);
    let rate = (angle - previous_angles[0]) / dt_s;
    Ok(ArticulatedGimbalJointState {
        angles_rad: vec![angle],
        rates_rad_s: vec![rate],
    })
}

fn articulated_gimbal_snapshot_unit_thrust_direction(
    snapshot: &openbmp_sim::EngineSnapshot,
) -> Option<Vector3<f64>> {
    if !snapshot.gimbal_pitch_rad.is_finite() || !snapshot.gimbal_yaw_rad.is_finite() {
        return None;
    }
    let cp = snapshot.gimbal_pitch_rad.cos();
    let sp = snapshot.gimbal_pitch_rad.sin();
    let cy = snapshot.gimbal_yaw_rad.cos();
    let sy = snapshot.gimbal_yaw_rad.sin();
    let thrust_parent = Vector3::new(sp, -(cp * sy), cp * cy);
    let norm = thrust_parent.norm();
    if !norm.is_finite() || norm <= ARTICULATED_GIMBAL_THRUST_DIRECTION_EPS {
        return None;
    }
    Some(thrust_parent / norm)
}

fn articulated_gimbal_explicit_joint_state_from_snapshot(
    joint: &MultiBodyGimbalJointConfig,
    snapshot: &openbmp_sim::EngineSnapshot,
) -> Option<ArticulatedGimbalJointState> {
    if ![
        snapshot.gimbal_pitch_rad,
        snapshot.gimbal_yaw_rad,
        snapshot.gimbal_pitch_rate_rad_s,
        snapshot.gimbal_yaw_rate_rad_s,
    ]
    .iter()
    .all(|value| value.is_finite())
    {
        return None;
    }
    if !articulated_gimbal_axis_matches(joint.neutral_thrust_body, [0.0, 0.0, 1.0]) {
        return None;
    }
    if let Some(secondary_axis_body) = joint.secondary_axis_body {
        if articulated_gimbal_axis_matches(joint.axis_body, [0.0, 1.0, 0.0])
            && articulated_gimbal_axis_matches(secondary_axis_body, [1.0, 0.0, 0.0])
        {
            return Some(ArticulatedGimbalJointState {
                angles_rad: vec![-snapshot.gimbal_yaw_rad, -snapshot.gimbal_pitch_rad],
                rates_rad_s: vec![
                    -snapshot.gimbal_yaw_rate_rad_s,
                    -snapshot.gimbal_pitch_rate_rad_s,
                ],
            });
        }
        return None;
    }
    if articulated_gimbal_axis_matches(joint.axis_body, [0.0, 1.0, 0.0]) {
        return Some(ArticulatedGimbalJointState {
            angles_rad: vec![-snapshot.gimbal_pitch_rad],
            rates_rad_s: vec![-snapshot.gimbal_pitch_rate_rad_s],
        });
    }
    if articulated_gimbal_axis_matches(joint.axis_body, [1.0, 0.0, 0.0]) {
        return Some(ArticulatedGimbalJointState {
            angles_rad: vec![-snapshot.gimbal_yaw_rad],
            rates_rad_s: vec![-snapshot.gimbal_yaw_rate_rad_s],
        });
    }
    None
}

fn articulated_gimbal_axis_matches(actual: [f64; 3], expected: [f64; 3]) -> bool {
    actual
        .iter()
        .zip(expected)
        .all(|(actual, expected)| (*actual - expected).abs() <= 1.0e-12)
}

fn articulated_gimbal_joint_state_angles_changed(
    previous: &ArticulatedGimbalJointState,
    current: &ArticulatedGimbalJointState,
) -> Result<bool, RunnerError> {
    if previous.angles_rad.len() != current.angles_rad.len() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal live joint-state angle dimensions changed from {} to {}",
                previous.angles_rad.len(),
                current.angles_rad.len()
            ),
        });
    }
    Ok(previous.angles_rad.iter().zip(&current.angles_rad).any(
        |(previous_angle, current_angle)| {
            (current_angle - previous_angle).abs() > ARTICULATED_GIMBAL_LIVE_JOINT_ABS_TOLERANCE_RAD
        },
    ))
}

fn articulated_gimbal_joint_states_angles_changed(
    previous: &[ArticulatedGimbalJointState],
    current: &[ArticulatedGimbalJointState],
) -> Result<bool, RunnerError> {
    if previous.len() != current.len() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "articulated gimbal live joint-state count changed from {} to {}",
                previous.len(),
                current.len()
            ),
        });
    }
    previous
        .iter()
        .zip(current)
        .try_fold(false, |changed, (previous_state, current_state)| {
            Ok(changed
                || articulated_gimbal_joint_state_angles_changed(previous_state, current_state)?)
        })
}

fn articulated_gimbal_joint_dof(joint: &MultiBodyGimbalJointConfig) -> usize {
    if joint.secondary_axis_body.is_some() {
        2
    } else {
        1
    }
}

fn normalized_vec3(value: [f64; 3], reason: &'static str) -> Result<Vector3<f64>, RunnerError> {
    let vector = Vector3::new(value[0], value[1], value[2]);
    let norm = vector.norm();
    if !norm.is_finite() || norm <= ARTICULATED_GIMBAL_THRUST_DIRECTION_EPS {
        return Err(RunnerError::UnsupportedScenario {
            what: reason.to_owned(),
        });
    }
    Ok(vector / norm)
}

fn vector_reject_axis(vector: Vector3<f64>, axis: Vector3<f64>) -> Vector3<f64> {
    vector - axis * vector.dot(&axis)
}

fn root_free_flyer_multibody_seed_from_rigid_state(
    body: BodyId,
    rigid_state: RigidBodyState,
) -> Result<RootFreeFlyerMultibodySeed, RunnerError> {
    let (tree, sim_state) = build_root_free_flyer_multibody_state(body, &rigid_state)?;
    Ok(RootFreeFlyerMultibodySeed {
        body,
        rigid_state,
        tree,
        sim_state,
    })
}

fn root_free_flyer_rigid_state_from_multibody_state(
    body: BodyId,
    state: &MultibodySimState,
    mass_props: MassProperties,
) -> Result<RigidBodyState, RunnerError> {
    let q = state.q();
    let qd = state.qd();
    if q.len() != 7 || qd.len() != 6 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "root free-flyer multibody state for body {} has q/qd dimensions {}/{}",
                body.value(),
                q.len(),
                qd.len()
            ),
        });
    }
    root_rigid_state_from_multibody_root_slots(body, state, mass_props)
}

fn root_rigid_state_from_multibody_root_slots(
    body: BodyId,
    state: &MultibodySimState,
    mass_props: MassProperties,
) -> Result<RigidBodyState, RunnerError> {
    let q = state.q();
    let qd = state.qd();
    if q.len() < 7 || qd.len() < 6 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "multibody root projection for body {} has q/qd dimensions {}/{}",
                body.value(),
                q.len(),
                qd.len()
            ),
        });
    }
    let raw_orientation = nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]);
    if !raw_orientation
        .coords
        .iter()
        .all(|component| component.is_finite())
        || raw_orientation.norm_squared() <= 0.0
    {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "root free-flyer multibody orientation projection failed for body {}",
                body.value()
            ),
        });
    }
    // RK sub-step states intentionally carry a linearly advanced, not-yet
    // projected quaternion, matching `RigidBodyState::advance_by`.
    let orientation = Quaternion::<Body, openbmp_core::Eci>::from_unit_quaternion(
        nalgebra::UnitQuaternion::new_unchecked(raw_orientation),
    );
    let velocity_body = Velocity3::<Body>::new(qd[3], qd[4], qd[5]);
    Ok(RigidBodyState::new(
        state.state().time,
        Position3::<openbmp_core::Eci>::new(q[4], q[5], q[6]),
        orientation.rotate_velocity(velocity_body),
        orientation,
        AngularVelocity3::<Body>::new(qd[0], qd[1], qd[2]),
        mass_props,
    ))
}

fn build_root_free_flyer_multibody_state(
    body: BodyId,
    state: &RigidBodyState,
) -> Result<(MultibodyTree, MultibodySimState), RunnerError> {
    let inertia = SpatialInertia::from_mass_properties(&state.mass_props).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!("root free-flyer multibody spatial inertia failed: {err}"),
        }
    })?;
    let tree = MultibodyTree::new(vec![TreeBodySpec {
        id: body,
        parent: None,
        joint: Joint::FreeFlyer,
        inertia,
        parent_to_body: PluckerTransform::identity(),
    }])
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("root free-flyer multibody tree failed: {err}"),
    })?;

    let orientation = state.orientation.q.into_inner();
    let linear_velocity_body = state
        .orientation
        .inverse()
        .rotate_velocity(state.velocity)
        .vector;
    let q = vec![
        orientation.w,
        orientation.i,
        orientation.j,
        orientation.k,
        state.position.vector.x,
        state.position.vector.y,
        state.position.vector.z,
    ];
    let qd = vec![
        state.angular_velocity.vector.x,
        state.angular_velocity.vector.y,
        state.angular_velocity.vector.z,
        linear_velocity_body.x,
        linear_velocity_body.y,
        linear_velocity_body.z,
    ];
    tree.sim_state_from_state(MultibodyState::new(state.time, q, qd))
        .map(|sim_state| (tree, sim_state))
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("root free-flyer multibody state failed: {err}"),
        })
}

fn root_free_flyer_multibody_derivative_from_runner_models<F, M>(
    seed: &RootFreeFlyerMultibodySeed,
    force_model: &F,
    moment_model: &M,
    views: RootFreeFlyerLoadViews<'_>,
) -> Result<MultibodyDerivative, RunnerError>
where
    F: ForceModel<RigidBodyState>,
    M: openbmp_sim::MomentModel<RigidBodyState>,
{
    root_free_flyer_multibody_stage_derivative_from_runner_models(
        seed,
        force_model,
        moment_model,
        views,
    )
    .map(|stage| stage.multibody)
}

#[derive(Clone, Debug)]
struct RootFreeFlyerStageDerivative {
    multibody: MultibodyDerivative,
    force_eci_n: Vector3<f64>,
}

fn root_free_flyer_multibody_stage_derivative_from_runner_models<F, M>(
    seed: &RootFreeFlyerMultibodySeed,
    force_model: &F,
    moment_model: &M,
    views: RootFreeFlyerLoadViews<'_>,
) -> Result<RootFreeFlyerStageDerivative, RunnerError>
where
    F: ForceModel<RigidBodyState>,
    M: openbmp_sim::MomentModel<RigidBodyState>,
{
    let mass_kg = seed.rigid_state.mass_props.mass_kg();
    let force_eci_n = force_model
        .force_n_eci(ForceContext {
            state: &seed.rigid_state,
            environment: views.environment,
            mass_kg,
            time: seed.rigid_state.time,
            active_body: Some(seed.body),
            phase_id: views.phase_id,
            effector_actuals: EffectorActualsView::new(views.effector_actuals),
            engine_snapshot: EngineSnapshotView::new(views.engine_snapshot),
            tank_snapshot: TankSnapshotView::new(views.tank_snapshot),
            recovery_snapshot: RecoverySnapshotView::new(views.recovery_snapshot),
        })
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("root free-flyer force adapter evaluation failed: {err}"),
        })?;
    let moment_body_n_m = moment_model
        .moment_n_m_body(MomentContext {
            state: &seed.rigid_state,
            environment: views.environment,
            time: seed.rigid_state.time,
            active_body: Some(seed.body),
            phase_id: views.phase_id,
            effector_actuals: EffectorActualsView::new(views.effector_actuals),
            engine_snapshot: EngineSnapshotView::new(views.engine_snapshot),
            tank_snapshot: TankSnapshotView::new(views.tank_snapshot),
            recovery_snapshot: RecoverySnapshotView::new(views.recovery_snapshot),
        })
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("root free-flyer moment adapter evaluation failed: {err}"),
        })?;
    let multibody = root_free_flyer_multibody_derivative_from_rigid_loads(
        &seed.tree,
        &seed.sim_state,
        seed.rigid_state.mass_props,
        moment_body_n_m,
        force_eci_n,
    )?;
    Ok(RootFreeFlyerStageDerivative {
        multibody,
        force_eci_n,
    })
}

#[derive(Copy, Clone, Debug)]
struct RootFreeFlyerHeldLoadViews<'a> {
    phase_id: Option<u64>,
    effector_actuals: &'a BTreeMap<String, f64>,
    engine_snapshot: &'a BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    tank_snapshot: &'a BTreeMap<TankId, openbmp_sim::TankSnapshot>,
    recovery_snapshot: &'a BTreeMap<RecoveryId, openbmp_sim::RecoverySnapshot>,
}

fn root_free_flyer_multibody_rk4_forecast_from_runner_models<F, M, MM, ENV>(
    seed: &RootFreeFlyerMultibodySeed,
    force_model: &F,
    moment_model: &M,
    mass_model: &MM,
    dt_s: f64,
    held_views: RootFreeFlyerHeldLoadViews<'_>,
    environment_for_state: ENV,
) -> Result<RootFreeFlyerMultibodySeed, RunnerError>
where
    F: ForceModel<RigidBodyState>,
    M: openbmp_sim::MomentModel<RigidBodyState>,
    MM: RigidMassModel,
    ENV: Fn(&RigidBodyState) -> Result<openbmp_sim::EnvironmentSample, RunnerError>,
{
    let next_rigid_state = Rk4FixedStep
        .advance(
            &seed.rigid_state,
            |rigid_state, _time| {
                let environment = environment_for_state(rigid_state)
                    .map_err(multibody_shadow_model_eval_error)?;
                let stage_seed =
                    root_free_flyer_multibody_seed_from_rigid_state(seed.body, *rigid_state)
                        .map_err(multibody_shadow_model_eval_error)?;
                let stage = root_free_flyer_multibody_stage_derivative_from_runner_models(
                    &stage_seed,
                    force_model,
                    moment_model,
                    RootFreeFlyerLoadViews {
                        phase_id: held_views.phase_id,
                        environment: &environment,
                        effector_actuals: held_views.effector_actuals,
                        engine_snapshot: held_views.engine_snapshot,
                        tank_snapshot: held_views.tank_snapshot,
                        recovery_snapshot: held_views.recovery_snapshot,
                    },
                )
                .map_err(multibody_shadow_model_eval_error)?;
                let rate = mass_model
                    .mass_properties_rate_at(MassContext {
                        time: rigid_state.time,
                        active_body: Some(seed.body),
                        engine_snapshot: EngineSnapshotView::new(held_views.engine_snapshot),
                        tank_snapshot: TankSnapshotView::new(held_views.tank_snapshot),
                        recovery_snapshot: openbmp_sim::RecoverySnapshotView::new(
                            held_views.recovery_snapshot,
                        ),
                    })
                    .map_err(|err| {
                        multibody_shadow_model_eval_error(RunnerError::UnsupportedScenario {
                            what: format!(
                                "root free-flyer mass-rate adapter evaluation failed: {err}"
                            ),
                        })
                    })?;
                let q_dot = stage.multibody.q_dot();
                let qd_dot = stage.multibody.qd_dot();
                let inv_inertia = rigid_state.mass_props.inertia_body.try_inverse().ok_or_else(
                    || {
                        multibody_shadow_model_eval_error(RunnerError::UnsupportedScenario {
                            what: "root free-flyer mass-rate forecast inertia tensor is not invertible"
                                .to_owned(),
                        })
                    },
                )?;
                let i_dot_omega = rate.inertia_rate_body * rigid_state.angular_velocity.vector;
                let angular_acceleration =
                    Vector3::new(qd_dot[0], qd_dot[1], qd_dot[2]) - inv_inertia * i_dot_omega;
                Ok(RigidBodyDerivative::new(
                    rigid_state.velocity.vector,
                    stage.force_eci_n / rigid_state.mass_props.mass_kg(),
                    nalgebra::Quaternion::new(q_dot[0], q_dot[1], q_dot[2], q_dot[3]),
                    angular_acceleration,
                    rate.mass_rate_kg_s,
                    rate.center_of_mass_rate_body_m_s,
                    rate.inertia_rate_body,
                ))
            },
            Duration::from_seconds(dt_s),
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("root free-flyer multibody RK4 forecast failed: {err}"),
        })?;
    root_free_flyer_multibody_seed_from_rigid_state(seed.body, next_rigid_state)
}

fn multibody_shadow_model_eval_error(error: RunnerError) -> openbmp_sim::ModelEvalError {
    openbmp_sim::ModelEvalError::InvalidState {
        model: RIGID_BODY_MULTIBODY_SHADOW_MODEL_ID,
        reason: Cow::Owned(error.to_string()),
    }
}

#[derive(Copy, Clone, Debug)]
struct RootFreeFlyerLoadViews<'a> {
    phase_id: Option<u64>,
    environment: &'a openbmp_sim::EnvironmentSample,
    effector_actuals: &'a BTreeMap<String, f64>,
    engine_snapshot: &'a BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    tank_snapshot: &'a BTreeMap<TankId, openbmp_sim::TankSnapshot>,
    recovery_snapshot: &'a BTreeMap<RecoveryId, openbmp_sim::RecoverySnapshot>,
}

fn root_free_flyer_multibody_derivative_from_rigid_loads(
    tree: &MultibodyTree,
    state: &MultibodySimState,
    mass_props: MassProperties,
    moment_body_n_m: Vector3<f64>,
    force_eci_n: Vector3<f64>,
) -> Result<MultibodyDerivative, RunnerError> {
    let external_forces_body = vec![SpatialForce::zero(); tree.bodies().len()];
    let derivative = tree
        .derivative_from_root_free_flyer_loads(
            state.state(),
            moment_body_n_m,
            force_eci_n,
            SpatialMotion::zero(),
            &external_forces_body,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("root free-flyer multibody derivative failed: {err}"),
        })?;
    let qd = state.qd();
    let body_acceleration =
        root_free_flyer_rigid_state_from_multibody_state(tree.bodies()[0].id, state, mass_props)?
            .orientation
            .inverse()
            .q
            * (force_eci_n / mass_props.mass_kg());
    let omega_body = Vector3::new(qd[0], qd[1], qd[2]);
    let velocity_body = Vector3::new(qd[3], qd[4], qd[5]);
    let mut qd_dot = derivative.qd_dot().to_vec();
    let linear_velocity_derivative_body = body_acceleration - omega_body.cross(&velocity_body);
    qd_dot[3] = linear_velocity_derivative_body.x;
    qd_dot[4] = linear_velocity_derivative_body.y;
    qd_dot[5] = linear_velocity_derivative_body.z;
    Ok(MultibodyDerivative::new(
        derivative.q_dot().to_vec(),
        qd_dot,
    ))
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
            &BTreeMap::new(),
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("initial primary body `{primary_body_id}` mass properties failed: {err}"),
        })?;

    let mut lanes = Vec::with_capacity(multi_body.initial_lanes.len());
    for lane in &multi_body.initial_lanes {
        let body = body_id_from_scenario_text(&lane.body_id);
        let mass_props = mass_model
            .mass_properties_from_snapshots(
                start_time,
                Some(body),
                engine_snapshot,
                tank_snapshot,
                &BTreeMap::new(),
            )
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
            let model = crate::celestial::build_egm2008_gravity(document, resolved_files)?;
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
        "tesseral" => {
            let model = crate::celestial::build_tesseral_gravity(document, resolved_files)?;
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

fn engine_id_from_scenario_text(id: &str) -> EngineId {
    EngineId::from_path(&format!("vehicle.assembly.engines.{id}"))
}

fn articulated_engine_body_id(engine: EngineId) -> BodyId {
    BodyId::from_path(&format!("multibody.articulated_engine.{}", engine.value()))
}

fn articulated_gimbal_frame_body_id(engine: EngineId) -> BodyId {
    BodyId::from_path(&format!(
        "multibody.articulated_gimbal_frame.{}",
        engine.value()
    ))
}

fn optional_body_owner(owner: Option<&str>) -> Option<BodyId> {
    owner.map(body_id_from_scenario_text)
}

#[derive(Copy, Clone, Debug)]
struct WeldedReleaseOwnerAlias {
    root: BodyId,
    rot_root_from_body: nalgebra::Matrix3<f64>,
    translation_root_from_body_m: Vector3<f64>,
}

#[derive(Copy, Clone, Debug)]
struct WeldedReleaseChildAliasSpec {
    body: BodyId,
    child_weld_translation_parent_body_m: [f64; 3],
    child_weld_quaternion_parent_to_child_xyzw: [f64; 4],
}

type WeldedReleaseChildAliasMap = BTreeMap<BodyId, Vec<WeldedReleaseChildAliasSpec>>;

fn welded_release_descendant_owner_aliases(
    document: &ScenarioDocument,
) -> BTreeMap<BodyId, WeldedReleaseOwnerAlias> {
    let mut aliases = BTreeMap::new();
    if !welded_release_jettison_authority_enabled(document) {
        return aliases;
    }
    let Some(multi_body) = document.multi_body.as_ref() else {
        return aliases;
    };
    let mut welded_children: WeldedReleaseChildAliasMap = BTreeMap::new();
    for welded_body in &multi_body.welded_bodies {
        welded_children
            .entry(body_id_from_scenario_text(&welded_body.parent_body_id))
            .or_default()
            .push(WeldedReleaseChildAliasSpec {
                body: body_id_from_scenario_text(&welded_body.child_body_id),
                child_weld_translation_parent_body_m: welded_body
                    .child_weld_translation_parent_body_m,
                child_weld_quaternion_parent_to_child_xyzw: welded_body
                    .child_weld_quaternion_parent_to_child_xyzw,
            });
    }
    for separation in &multi_body.separations {
        let root = body_id_from_scenario_text(&separation.lower_body_id);
        collect_welded_release_descendant_owner_aliases(
            root,
            root,
            nalgebra::Matrix3::identity(),
            Vector3::zeros(),
            &welded_children,
            &mut aliases,
        );
    }
    aliases
}

fn collect_welded_release_descendant_owner_aliases(
    root: BodyId,
    parent: BodyId,
    rot_root_from_parent: nalgebra::Matrix3<f64>,
    translation_root_from_parent_m: Vector3<f64>,
    welded_children: &WeldedReleaseChildAliasMap,
    aliases: &mut BTreeMap<BodyId, WeldedReleaseOwnerAlias>,
) {
    let Some(children) = welded_children.get(&parent) else {
        return;
    };
    for child in children {
        let rot_child_from_parent =
            rotation_child_from_parent(child.child_weld_quaternion_parent_to_child_xyzw);
        let rot_root_from_child = rot_root_from_parent * rot_child_from_parent.transpose();
        let translation_parent_to_child_parent_m =
            Vector3::from(child.child_weld_translation_parent_body_m);
        let translation_root_from_child_m = translation_root_from_parent_m
            + rot_root_from_parent * translation_parent_to_child_parent_m;
        aliases.insert(
            child.body,
            WeldedReleaseOwnerAlias {
                root,
                rot_root_from_body: rot_root_from_child,
                translation_root_from_body_m: translation_root_from_child_m,
            },
        );
        collect_welded_release_descendant_owner_aliases(
            root,
            child.body,
            rot_root_from_child,
            translation_root_from_child_m,
            welded_children,
            aliases,
        );
    }
}

fn rotation_child_from_parent(quaternion_parent_to_child_xyzw: [f64; 4]) -> nalgebra::Matrix3<f64> {
    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        quaternion_parent_to_child_xyzw[3],
        quaternion_parent_to_child_xyzw[0],
        quaternion_parent_to_child_xyzw[1],
        quaternion_parent_to_child_xyzw[2],
    ))
    .to_rotation_matrix()
    .matrix()
    .clone_owned()
}

fn direct_torque_axis_body(axis_index: usize) -> Vector3<f64> {
    let mut axis = Vector3::zeros();
    if axis_index < 3 {
        axis[axis_index] = 1.0;
    }
    axis
}

fn engine_owner_map(
    document: &ScenarioDocument,
) -> Result<BTreeMap<openbmp_core::EngineId, BodyId>, RunnerError> {
    let mut owners = BTreeMap::new();
    let welded_release_owner_aliases = welded_release_descendant_owner_aliases(document);
    for engine in &document.vehicle.assembly.engines {
        let id = openbmp_core::EngineId::from_path(&format!(
            "vehicle.assembly.engines.{id}",
            id = engine.id
        ));
        if let Some(owner) = optional_body_owner(engine.mounted_to.as_deref()) {
            owners.insert(
                id,
                welded_release_owner_aliases
                    .get(&owner)
                    .map_or(owner, |alias| alias.root),
            );
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

fn welded_release_engine_frame_transforms(
    document: &ScenarioDocument,
) -> BTreeMap<openbmp_core::EngineId, EngineClusterFrameTransform> {
    let welded_release_owner_aliases = welded_release_descendant_owner_aliases(document);
    let mut transforms = BTreeMap::new();
    for engine in &document.vehicle.assembly.engines {
        let Some(owner) = optional_body_owner(engine.mounted_to.as_deref()) else {
            continue;
        };
        let Some(alias) = welded_release_owner_aliases.get(&owner) else {
            continue;
        };
        let id = engine_id_from_scenario_text(&engine.id);
        let mount_point_body = Vector3::from(engine.mount_point_body_m);
        transforms.insert(
            id,
            EngineClusterFrameTransform {
                rot_owner_from_engine_body: alias.rot_root_from_body,
                mount_point_owner_m: alias.translation_root_from_body_m
                    + alias.rot_root_from_body * mount_point_body,
            },
        );
    }
    transforms
}

fn tank_owner_map(document: &ScenarioDocument) -> BTreeMap<openbmp_core::TankId, BodyId> {
    let welded_release_owner_aliases = welded_release_descendant_owner_aliases(document);
    let mut owners = BTreeMap::new();
    for tank in &document.vehicle.assembly.tanks {
        let id =
            openbmp_core::TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = tank.id));
        let owner = body_id_from_scenario_text(&tank.mounted_to);
        owners.insert(
            id,
            welded_release_owner_aliases
                .get(&owner)
                .map_or(owner, |alias| alias.root),
        );
    }
    owners
}

fn welded_release_tank_frame_transforms(
    document: &ScenarioDocument,
) -> BTreeMap<openbmp_core::TankId, TankRackFrameTransform> {
    let welded_release_owner_aliases = welded_release_descendant_owner_aliases(document);
    let mut transforms = BTreeMap::new();
    for tank in &document.vehicle.assembly.tanks {
        let owner = body_id_from_scenario_text(&tank.mounted_to);
        let Some(alias) = welded_release_owner_aliases.get(&owner) else {
            continue;
        };
        let id =
            openbmp_core::TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = tank.id));
        transforms.insert(
            id,
            TankRackFrameTransform {
                rot_owner_from_tank_body: alias.rot_root_from_body,
                translation_owner_from_tank_body_m: alias.translation_root_from_body_m,
            },
        );
    }
    transforms
}

fn motor_owner(document: &ScenarioDocument) -> Option<BodyId> {
    let owner = optional_body_owner(
        document
            .propulsion
            .as_ref()
            .and_then(|propulsion| propulsion.motor.as_ref())
            .and_then(|motor| motor.mounted_to.as_deref()),
    )?;
    Some(
        welded_release_descendant_owner_aliases(document)
            .get(&owner)
            .map_or(owner, |alias| alias.root),
    )
}

fn welded_release_motor_frame_transform(
    document: &ScenarioDocument,
) -> Option<MotorThrustFrameTransform> {
    let owner = optional_body_owner(
        document
            .propulsion
            .as_ref()
            .and_then(|propulsion| propulsion.motor.as_ref())
            .and_then(|motor| motor.mounted_to.as_deref()),
    )?;
    let aliases = welded_release_descendant_owner_aliases(document);
    let alias = aliases.get(&owner)?;
    Some(MotorThrustFrameTransform {
        rot_owner_from_motor_body: alias.rot_root_from_body,
    })
}

fn aero_owner(document: &ScenarioDocument) -> Option<BodyId> {
    let owner = optional_body_owner(
        document
            .aero
            .as_ref()
            .and_then(|aero| aero.mounted_to.as_deref()),
    )?;
    Some(
        welded_release_descendant_owner_aliases(document)
            .get(&owner)
            .map_or(owner, |alias| alias.root),
    )
}

fn welded_release_aero_frame_transform(document: &ScenarioDocument) -> Option<AeroFrameTransform> {
    let owner = optional_body_owner(
        document
            .aero
            .as_ref()
            .and_then(|aero| aero.mounted_to.as_deref()),
    )?;
    let aliases = welded_release_descendant_owner_aliases(document);
    let alias = aliases.get(&owner)?;
    Some(AeroFrameTransform {
        rot_owner_from_aero_body: alias.rot_root_from_body,
        translation_owner_from_aero_body_m: alias.translation_root_from_body_m,
    })
}

fn recovery_owner_map(
    document: &ScenarioDocument,
) -> Result<BTreeMap<openbmp_core::RecoveryId, BodyId>, RunnerError> {
    let welded_release_owner_aliases = welded_release_descendant_owner_aliases(document);
    let mut owners = BTreeMap::new();
    for recovery in &document.vehicle.assembly.recovery {
        let id = openbmp_core::RecoveryId::from_path(&format!(
            "vehicle.assembly.recovery.{id}",
            id = recovery.id
        ));
        if let Some(owner) = optional_body_owner(recovery.mounted_to.as_deref()) {
            owners.insert(
                id,
                welded_release_owner_aliases
                    .get(&owner)
                    .map_or(owner, |alias| alias.root),
            );
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
    landing_gear_runtime: Option<crate::landing_gear::LandingGearRuntime>,
    aerothermal_sink: Option<crate::aerothermal::LiveAerothermalSink>,
    contact_evaluator: Option<&crate::contact::ContactDiagnosticsEvaluator>,
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
                let owner = aero_owner(document);
                let aero_frame_transform = welded_release_aero_frame_transform(document);
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
                    let drag = match (owner, aero_frame_transform) {
                        (Some(owner), Some(transform)) => {
                            DeckDragForceAdapter::new_owned_with_frame_transform(
                                deck,
                                atmosphere,
                                RIGID_BODY_AERO_MODEL_ID,
                                owner,
                                transform,
                            )
                        }
                        (Some(owner), None) => DeckDragForceAdapter::new_owned(
                            deck,
                            atmosphere,
                            RIGID_BODY_AERO_MODEL_ID,
                            owner,
                        ),
                        (None, _) => {
                            DeckDragForceAdapter::new(deck, atmosphere, RIGID_BODY_AERO_MODEL_ID)
                        }
                    };
                    named.push(NamedForceModel::new("aero", Box::new(drag)));
                } else {
                    let (method, reference_length_m) =
                        crate::aero::build_aero_method(document, loaded.aero_deck.clone())?
                            .ok_or_else(|| RunnerError::UnsupportedScenario {
                                what: "forces includes `aero` but [aero] block is missing"
                                    .to_owned(),
                            })?;
                    let adapter = match (owner, aero_frame_transform) {
                        (Some(owner), Some(transform)) => {
                            AeroMethodForceAdapter::new_owned_with_frame_transform(
                                method,
                                atmosphere,
                                RIGID_BODY_AERO_MODEL_ID,
                                reference_length_m,
                                owner,
                                transform,
                            )
                        }
                        (Some(owner), None) => AeroMethodForceAdapter::new_owned(
                            method,
                            atmosphere,
                            RIGID_BODY_AERO_MODEL_ID,
                            reference_length_m,
                            owner,
                        ),
                        (None, _) => AeroMethodForceAdapter::new(
                            method,
                            atmosphere,
                            RIGID_BODY_AERO_MODEL_ID,
                            reference_length_m,
                        ),
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
                    let owner = motor_owner(document);
                    let motor_frame_transform = welded_release_motor_frame_transform(document);
                    let thrust = match (owner, motor_frame_transform) {
                        (Some(owner), Some(transform)) => {
                            MotorThrustForceAdapter::new_owned_with_frame_transform(
                                motor,
                                ignition_time_s,
                                RIGID_BODY_THRUST_MODEL_ID,
                                owner,
                                transform,
                            )
                        }
                        (Some(owner), None) => MotorThrustForceAdapter::new_owned(
                            motor,
                            ignition_time_s,
                            RIGID_BODY_THRUST_MODEL_ID,
                            owner,
                        ),
                        (None, _) => MotorThrustForceAdapter::new(
                            motor,
                            ignition_time_s,
                            RIGID_BODY_THRUST_MODEL_ID,
                        ),
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
                    let engine_frame_transforms = welded_release_engine_frame_transforms(document);
                    let thrust = if engine_owners.is_empty() {
                        EngineClusterForceAdapter::new(
                            engine_ids,
                            RIGID_BODY_ENGINE_CLUSTER_THRUST_MODEL_ID,
                        )
                    } else if engine_frame_transforms.is_empty() {
                        EngineClusterForceAdapter::new_with_owners(
                            engine_ids,
                            engine_owners,
                            RIGID_BODY_ENGINE_CLUSTER_THRUST_MODEL_ID,
                        )
                    } else {
                        EngineClusterForceAdapter::new_with_owners_and_frame_transforms(
                            engine_ids,
                            engine_owners,
                            engine_frame_transforms,
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
            "contact" => {
                let contact =
                    contact_evaluator.ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "forces includes `contact` but shared contact runtime is missing"
                            .to_owned(),
                    })?;
                let adapter = crate::contact::build_half_space_contact_force_adapter(
                    contact,
                    RIGID_BODY_CONTACT_MODEL_ID,
                )?;
                named.push(NamedForceModel::new("contact", Box::new(adapter)));
            }
            "landing_gear" => {
                let runtime =
                    landing_gear_runtime
                        .clone()
                        .ok_or_else(|| RunnerError::UnsupportedScenario {
                            what: "forces includes `landing_gear` but [vehicle.landing_gear] block is missing".to_owned(),
                        })?;
                named.push(NamedForceModel::new(
                    "landing_gear",
                    Box::new(crate::landing_gear::LandingGearForceAdapter::new(
                        runtime,
                        RIGID_BODY_LANDING_GEAR_FORCE_MODEL_ID,
                    )),
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
        let tank_frame_transforms = welded_release_tank_frame_transforms(document);
        let tank_force =
            openbmp_vehicle::TankRackForceAdapter::new_with_owners_and_frame_transforms(
                tank_ids,
                tank_owners,
                tank_frame_transforms,
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

    // Declared-riser aggregate line-force adapter (rigid). This consumes the
    // recovery rack snapshot as a separate force component and leaves the
    // aerodynamic recovery-drag adapter unchanged.
    let recovery_riser_ids = riser_recovery_ids(document);
    if !recovery_riser_ids.is_empty() {
        let recovery_owners = recovery_owner_map(document)?;
        let recovery_riser_force = if recovery_owners.is_empty() {
            openbmp_vehicle::RecoveryRiserForceAdapter::new(
                recovery_riser_ids,
                RIGID_BODY_RECOVERY_RISER_FORCE_MODEL_ID,
            )
        } else {
            openbmp_vehicle::RecoveryRiserForceAdapter::new_with_owners(
                recovery_riser_ids,
                recovery_owners,
                RIGID_BODY_RECOVERY_RISER_FORCE_MODEL_ID,
            )
        };
        named.push(NamedForceModel::new(
            "recovery_riser_force",
            Box::new(recovery_riser_force),
        ));
    }

    let recovery_sequence_force_ids = sequence_force_recovery_ids(document);
    if !recovery_sequence_force_ids.is_empty() {
        let recovery_owners = recovery_owner_map(document)?;
        let recovery_sequence_force = if recovery_owners.is_empty() {
            RecoverySequenceForceAdapter::new(
                recovery_sequence_force_ids,
                RIGID_BODY_RECOVERY_SEQUENCE_FORCE_MODEL_ID,
            )
        } else {
            RecoverySequenceForceAdapter::new_with_owners(
                recovery_sequence_force_ids,
                recovery_owners,
                RIGID_BODY_RECOVERY_SEQUENCE_FORCE_MODEL_ID,
            )
        };
        named.push(NamedForceModel::new(
            "recovery_sequence_force",
            Box::new(recovery_sequence_force),
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
    let recovery_ids = apparent_mass_recovery_ids(document);
    let mass: Box<dyn MassModel> = if recovery_ids.is_empty() {
        inner
    } else {
        let recovery_owners = recovery_owner_map(document)?;
        if recovery_owners.is_empty() {
            Box::new(RecoveryRackMassAdapter::new(
                inner,
                recovery_ids,
                RIGID_BODY_RECOVERY_RACK_MASS_MODEL_ID,
            ))
        } else {
            Box::new(RecoveryRackMassAdapter::new_with_owners(
                inner,
                recovery_ids,
                recovery_owners,
                RIGID_BODY_RECOVERY_RACK_MASS_MODEL_ID,
            ))
        }
    };
    Ok(BoxedMassModel(mass))
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

#[allow(clippy::if_not_else)]
fn build_moment_model(
    document: &ScenarioDocument,
    loaded: &LoadedModels,
    landing_gear_runtime: Option<crate::landing_gear::LandingGearRuntime>,
) -> Result<KernelVehicle<RigidBodyState>, RunnerError> {
    let assembly = &document.vehicle.assembly;
    let mut named: Vec<NamedMomentModel<RigidBodyState>> = Vec::new();

    if document
        .force_model_universe()
        .iter()
        .any(|model| model == "aero")
    {
        let atmosphere = build_document_runtime_atmosphere(document)?;
        let owner = aero_owner(document);
        let aero_frame_transform = welded_release_aero_frame_transform(document);
        let (method, reference_length_m) =
            crate::aero::build_aero_method(document, loaded.aero_deck.clone())?.ok_or_else(
                || RunnerError::UnsupportedScenario {
                    what: "forces includes `aero` but [aero] block is missing".to_owned(),
                },
            )?;
        let adapter = match (owner, aero_frame_transform) {
            (Some(owner), Some(transform)) => {
                AeroMethodMomentAdapter::new_owned_with_frame_transform(
                    method,
                    atmosphere,
                    RIGID_BODY_AERO_MODEL_ID,
                    reference_length_m,
                    owner,
                    transform,
                )
            }
            (Some(owner), None) => AeroMethodMomentAdapter::new_owned(
                method,
                atmosphere,
                RIGID_BODY_AERO_MODEL_ID,
                reference_length_m,
                owner,
            ),
            (None, _) => AeroMethodMomentAdapter::new(
                method,
                atmosphere,
                RIGID_BODY_AERO_MODEL_ID,
                reference_length_m,
            ),
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
        let engine_frame_transforms = welded_release_engine_frame_transforms(document);
        let adapter = if engine_owners.is_empty() {
            EngineClusterMomentAdapter::new(
                engine_ids,
                mount_points_body,
                RIGID_BODY_ENGINE_CLUSTER_MOMENT_MODEL_ID,
            )
        } else if engine_frame_transforms.is_empty() {
            EngineClusterMomentAdapter::new_with_owners(
                engine_ids,
                mount_points_body,
                engine_owners,
                RIGID_BODY_ENGINE_CLUSTER_MOMENT_MODEL_ID,
            )
        } else {
            EngineClusterMomentAdapter::new_with_owners_and_frame_transforms(
                engine_ids,
                mount_points_body,
                engine_owners,
                engine_frame_transforms,
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
            Box::new(
                openbmp_vehicle::TankRackMomentAdapter::new_with_owners_and_frame_transforms(
                    tank_ids,
                    tank_owner_map(document),
                    welded_release_tank_frame_transforms(document),
                    RIGID_BODY_TANK_RACK_MOMENT_MODEL_ID,
                ),
            ),
        ));
    }

    if document
        .force_model_universe()
        .iter()
        .any(|model| model == "landing_gear")
    {
        let runtime = landing_gear_runtime.ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "forces includes `landing_gear` but [vehicle.landing_gear] block is missing"
                .to_owned(),
        })?;
        named.push(NamedMomentModel::new(
            "landing_gear",
            Box::new(crate::landing_gear::LandingGearMomentAdapter::new(
                runtime,
                RIGID_BODY_LANDING_GEAR_MOMENT_MODEL_ID,
            )),
        ));
    }

    let recovery_riser_ids = riser_recovery_ids(document);
    if !recovery_riser_ids.is_empty() {
        let recovery_owners = recovery_owner_map(document)?;
        let adapter = if recovery_owners.is_empty() {
            openbmp_vehicle::RecoveryRiserMomentAdapter::new(
                recovery_riser_ids,
                RIGID_BODY_RECOVERY_RISER_MOMENT_MODEL_ID,
            )
        } else {
            openbmp_vehicle::RecoveryRiserMomentAdapter::new_with_owners(
                recovery_riser_ids,
                recovery_owners,
                RIGID_BODY_RECOVERY_RISER_MOMENT_MODEL_ID,
            )
        };
        named.push(NamedMomentModel::new("recovery_riser", Box::new(adapter)));
    }

    let direct_torque_adapter = build_direct_torque_adapter(document);

    // direct_torque effectors coexist with the engine-cluster and
    // tank-rack moment models: every tick the per-tick loop builds and
    // sets the engine, tank, and effector-actual snapshots independently
    // (see the `set_engine_snapshot` / `set_tank_snapshot` /
    // `set_effector_actuals` calls in `run_rigid_body`), and the kernel
    // sums each NamedMomentModel. This combination drives RCS-class
    // body-torque effectors — e.g. a coast attitude-hold thruster set
    // that arrests propellant-slosh-induced tumble with the main engines
    // off (no gimbal authority). Validated end-to-end by the
    // `phalcon9-orbit-slosh-rcs` scenario / `phalcon9_orbit_slosh_rcs_e2e`.
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
    if known_moments.iter().any(|name| name == "recovery_riser")
        && !default_active.iter().any(|name| name == "recovery_riser")
    {
        default_active.push("recovery_riser".to_owned());
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
    let welded_release_owner_aliases = welded_release_descendant_owner_aliases(document);
    for effector in &document.vehicle.assembly.effectors {
        if let openbmp_scenario::EffectorKindConfig::DirectTorque {
            axis,
            effectiveness_n_m_per_rad,
            ..
        } = effector.kind
        {
            let axis_index = axis.body_axis_index();
            let mut moment_axis_body = None;
            let owner = optional_body_owner(effector.mounted_to.as_deref()).map(|owner| {
                if let Some(alias) = welded_release_owner_aliases.get(&owner).copied() {
                    moment_axis_body =
                        Some(alias.rot_root_from_body * direct_torque_axis_body(axis_index));
                    alias.root
                } else {
                    owner
                }
            });
            bindings.push(openbmp_vehicle::DirectTorqueBinding {
                snapshot_key: effector.id.clone(),
                owner,
                body_axis_index: axis_index,
                moment_axis_body,
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
/// the same convention as `mass_properties_from_snapshots`: masses add,
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
struct SeparatedBodyTelemetryChannels {
    body: BodyId,
    separated: TelemetryChannel<bool>,
    propagating: TelemetryChannel<bool>,
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
struct EngineTelemetryChannels {
    engine: EngineId,
    thrust: TelemetryChannel<f64>,
    mass_flow: TelemetryChannel<f64>,
    consumed: TelemetryChannel<f64>,
    state_index: TelemetryChannel<i64>,
}

#[derive(Debug)]
struct MultiBodyCfeLoopTelemetryChannels {
    loop_id: String,
    lagrange_multiplier: TelemetryChannel<f64>,
    stabilized_acceleration: TelemetryChannel<f64>,
    acceleration_residual: TelemetryChannel<f64>,
}

impl MultiBodyCfeLoopTelemetryChannels {
    fn push_metadata(&self, channels: &mut Vec<openbmp_telemetry::ChannelMetadata>) {
        channels.push(self.lagrange_multiplier.metadata().clone());
        channels.push(self.stabilized_acceleration.metadata().clone());
        channels.push(self.acceleration_residual.metadata().clone());
    }

    fn insert_samples(
        &self,
        row: &mut TelemetryRow,
        samples: &BTreeMap<&str, &ArticulatedGimbalCfeLoopSolveSample>,
    ) -> Result<(), RunnerError> {
        let sample = samples.get(self.loop_id.as_str()).copied();
        row.insert(
            &self.lagrange_multiplier,
            sample.map_or(0.0, |sample| sample.lagrange_multiplier),
        )?;
        row.insert(
            &self.stabilized_acceleration,
            sample.map_or(0.0, |sample| sample.stabilized_acceleration_per_s2),
        )?;
        row.insert(
            &self.acceleration_residual,
            sample.map_or(0.0, |sample| sample.acceleration_residual_per_s2),
        )?;
        Ok(())
    }
}

fn validate_multi_body_cfe_loop_telemetry_suffixes(
    document: &ScenarioDocument,
) -> Result<(), RunnerError> {
    let mut suffixes = BTreeSet::new();
    for loop_id in multi_body_cfe_telemetry_loop_ids(document) {
        let suffix = crate::telemetry_suffix(loop_id);
        if !suffixes.insert(suffix.clone()) {
            return Err(RunnerError::UnsupportedScenario {
                what: format!("multi_body CFE telemetry suffix `{suffix}` is not unique"),
            });
        }
    }
    Ok(())
}

fn multi_body_cfe_telemetry_loop_ids(document: &ScenarioDocument) -> Vec<&str> {
    let Some(multi_body) = &document.multi_body else {
        return Vec::new();
    };
    multi_body
        .cfe_loops
        .iter()
        .map(|cfe_loop| cfe_loop.loop_id.as_str())
        .chain(
            multi_body
                .cfe_gimbal_locks
                .iter()
                .map(|lock| lock.loop_id.as_str()),
        )
        .chain(
            multi_body
                .cfe_planar_four_bars
                .iter()
                .map(|four_bar| four_bar.loop_id.as_str()),
        )
        .collect()
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
    mission_phase: Option<TelemetryChannel<String>>,
    mission_regions: Vec<(crate::MissionRegionDeclaration, TelemetryChannel<String>)>,
    comm: Option<crate::comm::CommTelemetryChannels>,
    acoustics: Option<crate::acoustics::AcousticsTelemetryChannels>,
    countdown: Option<crate::ground::GroundCountdownTelemetryChannels>,
    structural_loads: Option<crate::structural::StructuralLineLoadTelemetryChannels>,
    quaternion_x: TelemetryChannel<f64>,
    quaternion_y: TelemetryChannel<f64>,
    quaternion_z: TelemetryChannel<f64>,
    quaternion_w: TelemetryChannel<f64>,
    angular_velocity_x: TelemetryChannel<f64>,
    angular_velocity_y: TelemetryChannel<f64>,
    angular_velocity_z: TelemetryChannel<f64>,
    fc_reference: Option<FcReferenceTelemetryChannels>,
    separated_bodies: Vec<SeparatedBodyTelemetryChannels>,
    engine_states: Vec<EngineTelemetryChannels>,
    has_atmosphere: bool,
    atmosphere_density: Option<TelemetryChannel<f64>>,
    atmosphere_pressure: Option<TelemetryChannel<f64>>,
    atmosphere_temperature: Option<TelemetryChannel<f64>>,
    atmosphere_speed_of_sound: Option<TelemetryChannel<f64>>,
    force_components: ForceComponentChannels,
    contact: Option<crate::contact::ContactTelemetryChannels>,
    landing_gear: Option<crate::landing_gear::LandingGearTelemetryChannels>,
    multi_body_cfe_loops: Vec<MultiBodyCfeLoopTelemetryChannels>,
    active_models: Option<TelemetryChannel<String>>,
    aerothermal: Option<AerothermalTelemetryChannels>,
    plume: Option<crate::plume::PlumeTelemetryChannels>,
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
        validate_multi_body_cfe_loop_telemetry_suffixes(document)?;
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
        let structural_loads = crate::structural::StructuralLineLoadTelemetryChannels::maybe_new(
            document, &mut alloc,
        )?;

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
                    propagating: TelemetryChannel::<bool>::new(
                        alloc(),
                        format!("{prefix}.propagating"),
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

        let mut engine_states = Vec::new();
        for engine in &document.vehicle.assembly.engines {
            let id = EngineId::from_path(&format!("vehicle.assembly.engines.{id}", id = engine.id));
            let prefix = format!("engine.{}", engine.id);
            engine_states.push(EngineTelemetryChannels {
                engine: id,
                thrust: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.thrust_n"),
                    "N",
                    Some("Body"),
                )?,
                mass_flow: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.mass_flow_kg_s"),
                    "kg/s",
                    None::<&str>,
                )?,
                consumed: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.consumed_kg"),
                    "kg",
                    None::<&str>,
                )?,
                state_index: TelemetryChannel::<i64>::new(
                    alloc(),
                    format!("{prefix}.state_index"),
                    "1",
                    None::<&str>,
                )?,
            });
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

        let landing_gear = document
            .vehicle
            .landing_gear
            .is_some()
            .then(|| crate::landing_gear::LandingGearTelemetryChannels::new(document, &mut alloc))
            .transpose()?;

        let mut multi_body_cfe_loops = Vec::new();
        for loop_id in multi_body_cfe_telemetry_loop_ids(document) {
            let suffix = crate::telemetry_suffix(loop_id);
            let prefix = format!("multi_body.cfe_loop.{suffix}");
            multi_body_cfe_loops.push(MultiBodyCfeLoopTelemetryChannels {
                loop_id: loop_id.to_owned(),
                lagrange_multiplier: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.lagrange_multiplier"),
                    "1",
                    None::<&str>,
                )?,
                stabilized_acceleration: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.stabilized_acceleration_per_s2"),
                    "1/s^2",
                    None::<&str>,
                )?,
                acceleration_residual: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.acceleration_residual_per_s2"),
                    "1/s^2",
                    None::<&str>,
                )?,
            });
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
            mission_phase,
            mission_regions,
            comm,
            acoustics,
            countdown,
            structural_loads,
            quaternion_x,
            quaternion_y,
            quaternion_z,
            quaternion_w,
            angular_velocity_x,
            angular_velocity_y,
            angular_velocity_z,
            fc_reference,
            separated_bodies,
            engine_states,
            has_atmosphere,
            atmosphere_density,
            atmosphere_pressure,
            atmosphere_temperature,
            atmosphere_speed_of_sound,
            force_components,
            contact,
            landing_gear,
            multi_body_cfe_loops,
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
            self.quaternion_x.metadata().clone(),
            self.quaternion_y.metadata().clone(),
            self.quaternion_z.metadata().clone(),
            self.quaternion_w.metadata().clone(),
            self.angular_velocity_x.metadata().clone(),
            self.angular_velocity_y.metadata().clone(),
            self.angular_velocity_z.metadata().clone(),
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
        if let Some(structural_loads) = &self.structural_loads {
            structural_loads.push_metadata(&mut channels);
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
        for separated in &self.separated_bodies {
            channels.push(separated.separated.metadata().clone());
            channels.push(separated.propagating.metadata().clone());
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
        for engine in &self.engine_states {
            channels.push(engine.thrust.metadata().clone());
            channels.push(engine.mass_flow.metadata().clone());
            channels.push(engine.consumed.metadata().clone());
            channels.push(engine.state_index.metadata().clone());
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
        if let Some(landing_gear) = &self.landing_gear {
            landing_gear.push_metadata(&mut channels);
        }
        for cfe_loop in &self.multi_body_cfe_loops {
            cfe_loop.push_metadata(&mut channels);
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
        // order, between force breakdown and mission markers.
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

fn rigid_body_specific_force_truth(
    kernel: &RigidKernel,
    force_vehicle: &KernelVehicle<RigidBodyState>,
    moment_vehicle: &KernelVehicle<RigidBodyState>,
    mass_model: &RigidMassEither,
    bending_reaction_moment_body_n_m: Vector3<f64>,
) -> Result<SpecificForceTruth, RunnerError> {
    let state = kernel.current_state();
    let environment = kernel.current_environment_sample()?;
    let active_body = kernel.primary_rigid_body();
    let phase_id = kernel.current_phase().map(|phase| phase.value());
    let mass_kg = state.mass_props.mass_kg();
    let force_ctx = ForceContext {
        state,
        environment: &environment,
        mass_kg,
        time: state.time,
        active_body,
        phase_id,
        effector_actuals: EffectorActualsView::new(kernel.effector_actuals()),
        engine_snapshot: EngineSnapshotView::new(kernel.engine_snapshot()),
        tank_snapshot: TankSnapshotView::new(kernel.tank_snapshot()),
        recovery_snapshot: RecoverySnapshotView::new(kernel.recovery_snapshot()),
    };
    let force_breakdown = force_vehicle
        .evaluate_force_breakdown(force_ctx)
        .map_err(|e| RunnerError::UnsupportedScenario {
            what: format!("force-accumulator sensor force evaluation failed: {e}"),
        })?;
    let gravity_force = force_breakdown
        .components
        .iter()
        .filter(|(name, _)| name == "gravity")
        .fold(Vector3::zeros(), |sum, (_, force)| sum + *force);
    let non_gravity_specific_force_eci = (force_breakdown.total - gravity_force) / mass_kg;
    let f_cg_body_m_s2 = state.orientation.q.inverse() * non_gravity_specific_force_eci;

    let moment_ctx = MomentContext {
        state,
        environment: &environment,
        time: state.time,
        active_body,
        phase_id,
        effector_actuals: EffectorActualsView::new(kernel.effector_actuals()),
        engine_snapshot: EngineSnapshotView::new(kernel.engine_snapshot()),
        tank_snapshot: TankSnapshotView::new(kernel.tank_snapshot()),
        recovery_snapshot: RecoverySnapshotView::new(kernel.recovery_snapshot()),
    };
    let moment_breakdown = moment_vehicle
        .evaluate_moment_breakdown(moment_ctx)
        .map_err(|e| RunnerError::UnsupportedScenario {
            what: format!("force-accumulator sensor moment evaluation failed: {e}"),
        })?;
    let rate = mass_model
        .mass_properties_rate_at(MassContext {
            time: state.time,
            active_body,
            engine_snapshot: EngineSnapshotView::new(kernel.engine_snapshot()),
            tank_snapshot: TankSnapshotView::new(kernel.tank_snapshot()),
            recovery_snapshot: openbmp_sim::RecoverySnapshotView::new(kernel.recovery_snapshot()),
        })
        .map_err(|e| RunnerError::UnsupportedScenario {
            what: format!("force-accumulator sensor mass-rate evaluation failed: {e}"),
        })?;
    let inertia = state.mass_props.inertia_body;
    let omega = state.angular_velocity.vector;
    let i_omega = inertia * omega;
    let omega_cross_iomega = omega.cross(&i_omega);
    let i_dot_omega = rate.inertia_rate_body * omega;
    let net_moment = moment_breakdown.total + bending_reaction_moment_body_n_m
        - omega_cross_iomega
        - i_dot_omega;
    let inv_inertia = inertia
        .try_inverse()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "force-accumulator sensor truth requires invertible rigid-body inertia"
                .to_owned(),
        })?;

    Ok(SpecificForceTruth {
        f_cg_body_m_s2,
        omega_body_rad_s: omega,
        alpha_body_rad_s2: inv_inertia * net_moment,
    })
}

#[allow(clippy::too_many_arguments)]
fn record_step<I, F, MOM, MM, E, SC>(
    document: &ScenarioDocument,
    table: &mut TelemetryTable,
    kernel: &SimulationKernel<RigidBodyState, I, F, RigidModels<MOM, MM>, E, SC>,
    channels: &RigidChannelSet,
    breakdown_vehicle: &KernelVehicle<RigidBodyState>,
    contact_evaluator: Option<&crate::contact::ContactDiagnosticsEvaluator>,
    contact_accumulator: Option<&mut crate::contact::ContactRunAccumulator>,
    breakdown_atmosphere: Option<&RuntimeAtmosphere>,
    plume_evaluator: Option<&crate::plume::RigidPlumeEvaluator>,
    geocentric_surface_radius_m: Option<f64>,
    aerothermal: Option<&crate::aerothermal::LiveAerothermalOutput>,
    landing_gear_runtime: Option<&crate::landing_gear::LandingGearRuntime>,
    landing_gear_accumulator: Option<&mut crate::landing_gear::LandingGearRunAccumulator>,
    fc_bridge: Option<&crate::fc_bridge::FcBridge>,
    comm_observation: Option<&crate::comm::CommObservation>,
    acoustics_report: Option<&crate::acoustics::AcousticsRunReport>,
    countdown_report: Option<&crate::ground::GroundCountdownRunReport>,
    structural_rack: &crate::structural::StructuralRack,
    articulated_gimbal_shadows: &BTreeMap<EngineId, ArticulatedGimbalMultibodyShadow>,
    fired_events: &[openbmp_sim::FiredEvent<openbmp_sim::MissionAction>],
    mission_region_trace: &mut crate::MissionRegionTraceState,
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
    if let Some(structural_load_channels) = &channels.structural_loads {
        structural_load_channels.insert_samples(&mut row, structural_rack.line_loads())?;
    }

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
    insert_fc_reference_channels(&mut row, &channels.fc_reference, fc_bridge)?;

    insert_separated_body_channels(
        &mut row,
        kernel.separated_rigid_bodies(),
        &channels.separated_bodies,
    )?;

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

    // See point_mass.rs sibling for the
    // breakdown / kernel snapshot symmetry rationale.
    let env_sample = kernel.current_environment_sample()?;
    let kernel_actuals = kernel.effector_actuals();
    let kernel_engine_snapshot = kernel.engine_snapshot();
    let kernel_tank_snapshot = kernel.tank_snapshot();
    let kernel_recovery_snapshot = kernel.recovery_snapshot();
    insert_engine_state_channels(&mut row, kernel_engine_snapshot, &channels.engine_states)?;
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

    if let Some(landing_gear_channels) = &channels.landing_gear {
        let runtime = landing_gear_runtime.ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "[vehicle.landing_gear] telemetry requested without landing gear runtime"
                .to_owned(),
        })?;
        let samples = runtime.take_samples()?;
        landing_gear_channels.insert(&mut row, &samples)?;
        if let Some(accumulator) = landing_gear_accumulator {
            accumulator.record(
                state.time.as_seconds(),
                state.mass_props.mass_kg(),
                &samples,
                runtime.energy_integrals()?,
            )?;
        }
    }

    insert_multi_body_cfe_loop_channels(
        &mut row,
        &channels.multi_body_cfe_loops,
        articulated_gimbal_shadows,
    )?;

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
            evaluator.evaluate(state, atmosphere_sample, kernel_engine_snapshot)?
        } else {
            None
        };
        plume_channels.insert(&mut row, plume_state)?;
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

fn insert_multi_body_cfe_loop_channels(
    row: &mut TelemetryRow,
    channels: &[MultiBodyCfeLoopTelemetryChannels],
    articulated_gimbal_shadows: &BTreeMap<EngineId, ArticulatedGimbalMultibodyShadow>,
) -> Result<(), RunnerError> {
    if channels.is_empty() {
        return Ok(());
    }
    let mut samples = BTreeMap::new();
    for shadow in articulated_gimbal_shadows.values() {
        for sample in &shadow.last_cfe_loop_solves {
            samples.insert(sample.loop_id.as_str(), sample);
        }
    }
    for channel in channels {
        channel.insert_samples(row, &samples)?;
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

fn insert_separated_body_channels(
    row: &mut TelemetryRow,
    separated_bodies: &[openbmp_sim::SeparatedRigidBody],
    channels: &[SeparatedBodyTelemetryChannels],
) -> Result<(), RunnerError> {
    for channel in channels {
        let separated = separated_bodies
            .iter()
            .find(|body| body.body == channel.body);
        row.insert(&channel.separated, separated.is_some())?;
        row.insert(
            &channel.propagating,
            separated.is_some_and(|body| body.propagating),
        )?;
        if let Some(state) = separated.map(|body| &body.state) {
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

fn insert_engine_state_channels(
    row: &mut TelemetryRow,
    engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
    channels: &[EngineTelemetryChannels],
) -> Result<(), RunnerError> {
    for channel in channels {
        if let Some(snapshot) = engine_snapshot.get(&channel.engine) {
            row.insert(&channel.thrust, snapshot.thrust_body.norm())?;
            row.insert(&channel.mass_flow, snapshot.mass_flow_kg_per_s)?;
            row.insert(&channel.consumed, snapshot.consumed_kg)?;
            row.insert(
                &channel.state_index,
                i64::from(snapshot.lifecycle_state_index),
            )?;
        } else {
            row.insert(&channel.thrust, 0.0)?;
            row.insert(&channel.mass_flow, 0.0)?;
            row.insert(&channel.consumed, 0.0)?;
            row.insert(&channel.state_index, 0_i64)?;
        }
    }
    Ok(())
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
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    use openbmp_telemetry::TelemetryValue;

    fn workspace_root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
    }

    fn assert_rigid_forecast_matches_state(
        predicted: &RigidBodyState,
        actual: &RigidBodyState,
        expected_time_s: f64,
    ) {
        assert!(
            (predicted.time.as_seconds() - expected_time_s).abs() < 1.0e-12,
            "time predicted={} expected={}",
            predicted.time.as_seconds(),
            expected_time_s
        );
        assert!(
            (predicted.mass_props.mass_kg() - actual.mass_props.mass_kg()).abs() < 1.0e-12,
            "mass predicted={} actual={}",
            predicted.mass_props.mass_kg(),
            actual.mass_props.mass_kg()
        );
        assert!(
            (predicted.position.vector - actual.position.vector).norm() < 1.0e-12,
            "position predicted={:?} actual={:?} diff={:?}",
            predicted.position.vector,
            actual.position.vector,
            predicted.position.vector - actual.position.vector
        );
        assert!(
            (predicted.velocity.vector - actual.velocity.vector).norm() < 1.0e-12,
            "velocity predicted={:?} actual={:?} diff={:?}",
            predicted.velocity.vector,
            actual.velocity.vector,
            predicted.velocity.vector - actual.velocity.vector
        );
        assert!(
            (predicted.angular_velocity.vector - actual.angular_velocity.vector).norm() < 1.0e-12,
            "omega predicted={:?} actual={:?} diff={:?}",
            predicted.angular_velocity.vector,
            actual.angular_velocity.vector,
            predicted.angular_velocity.vector - actual.angular_velocity.vector
        );
        let predicted_q = predicted.orientation.q.into_inner();
        let actual_q = actual.orientation.q.into_inner();
        assert!(
            (predicted_q.coords - actual_q.coords).norm() < 1.0e-12,
            "q predicted={:?} actual={:?} diff={:?}",
            predicted_q.coords,
            actual_q.coords,
            predicted_q.coords - actual_q.coords
        );
    }

    fn separated_lane_state(session: &RigidBodySession, body: BodyId) -> &RigidBodyState {
        &session
            .separated_bodies()
            .iter()
            .find(|lane| lane.body == body)
            .expect("separated lane exists")
            .state
    }

    fn phalcon9_tvc_probe_multibody_scenario(accelerated: bool) -> openbmp_scenario::Scenario {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../scenarios/phalcon9/phalcon9-tvc-probe.toml");
        let mut toml = std::fs::read_to_string(&path).expect("read Phalcon-9 TVC probe scenario");
        if accelerated {
            toml = toml.replace("stop_s  = 40.0", "stop_s  = 2.0");
            toml = toml.replace(
                "schedule_s = [0.0, 10.0, 30.0, 40.0]",
                "schedule_s = [0.0, 0.2, 1.0, 2.0]",
            );
            toml = toml.replace(
                "pitch_rad  = [0.0, 0.0, 0.5, 0.5]",
                "pitch_rad  = [0.0, 0.0, 0.2, 0.2]",
            )
        }
        let toml = toml
            .replace(
                "[environment]\n",
                "[[vehicle.assembly.effectors]]\n\
                 id = \"shadow-proof-torque\"\n\
                 mounted_to = \"core\"\n\
                 kind = { kind = \"direct_torque\", axis = \"pitch\", effectiveness_n_m_per_rad = 1.0 }\n\
                 limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }\n\
                 \n\
                 [environment]\n",
            )
            .replace(
                "[telemetry]\n",
                "[multi_body]\n\
                 primary_body_id = \"core\"\n\
                 \n\
                 [[multi_body.attitude_target]]\n\
                 body_id = \"core\"\n\
                 start_time_s = 999.0\n\
                 pitch_effector = \"shadow-proof-torque\"\n\
                 kp = 1.0\n\
                 kd = 0.1\n\
                 max_command = 1.0\n\
                 target = { kind = \"eci_vector\", vector_eci = [0.0, 0.0, 1.0] }\n\
                 \n\
                 [telemetry]\n",
            );
        let source_dir = path.parent().map(std::path::Path::to_path_buf);
        openbmp_scenario::Scenario::from_toml_str_with_source_dir(&toml, source_dir)
            .expect("Phalcon-9 TVC multibody probe parses")
    }

    fn assert_phalcon9_tvc_multibody_forecasts(
        scenario: &openbmp_scenario::Scenario,
        max_steps: Option<usize>,
    ) -> (usize, f64) {
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(scenario, &resolved_files).expect("session prepares");
        let mut max_lateral_thrust_n = 0.0_f64;
        let mut step_count = 0usize;

        while !session.is_finished() && max_steps.is_none_or(|limit| step_count < limit) {
            session
                .step_once(&scenario.document, None)
                .expect("Phalcon-9 TVC multibody step forecasts primary root");
            step_count += 1;
            let forecast = session
                .primary_multibody_shadow
                .as_ref()
                .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
                .expect("Phalcon-9 TVC RK4 forecast recorded");
            assert_rigid_forecast_matches_state(
                &forecast.rigid_state,
                session.state(),
                session.state().time.as_seconds(),
            );

            for snapshot in session.engine_snapshots().values() {
                max_lateral_thrust_n = max_lateral_thrust_n.max(
                    (snapshot.thrust_body.x * snapshot.thrust_body.x
                        + snapshot.thrust_body.y * snapshot.thrust_body.y)
                        .sqrt(),
                );
            }
        }

        if max_steps.is_none() {
            assert!(
                matches!(
                    session.kernel.stop_reason(),
                    Some(StopReason::EndTime { .. })
                ),
                "full-duration TVC probe stopped early: {:?}",
                session.kernel.stop_reason()
            );
        }
        assert!(step_count > 0, "Phalcon-9 TVC probe must advance");
        assert!(
            max_lateral_thrust_n > 1.0,
            "vehicle-scale TVC probe should apply lateral gimballed thrust"
        );
        (step_count, max_lateral_thrust_n)
    }

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

    #[test]
    fn aggregate_welded_subtree_mass_properties_shifts_descendant_inertia_to_root() {
        let root = MassProperties::new(
            Mass::new::<kilogram>(1.0),
            Position3::<Body>::new(0.0, 0.0, 0.0),
            nalgebra::Matrix3::identity(),
        );
        let descendant = MassProperties::new(
            Mass::new::<kilogram>(2.0),
            Position3::<Body>::new(1.0, 0.0, 0.0),
            nalgebra::Matrix3::from_diagonal(&nalgebra::Vector3::new(2.0, 3.0, 4.0)),
        );
        let component = WeldedSubtreeMassComponent {
            root_to_body: PluckerTransform::new(
                nalgebra::Matrix3::identity(),
                Vector3::new(0.0, 3.0, 0.0),
            )
            .unwrap(),
            mass_properties: descendant,
        };

        let aggregate = aggregate_welded_subtree_mass_properties(root, &[component]);

        assert_eq!(aggregate.mass_kg().to_bits(), 3.0_f64.to_bits());
        assert!((aggregate.center_of_mass_body.vector.x - (2.0 / 3.0)).abs() < 1.0e-12);
        assert!((aggregate.center_of_mass_body.vector.y - 2.0).abs() < 1.0e-12);
        assert!((aggregate.inertia_body[(0, 0)] - 9.0).abs() < 1.0e-12);
        assert!((aggregate.inertia_body[(0, 1)] + 2.0).abs() < 1.0e-12);
        assert!((aggregate.inertia_body[(1, 0)] + 2.0).abs() < 1.0e-12);
        assert!((aggregate.inertia_body[(1, 1)] - (14.0 / 3.0)).abs() < 1.0e-12);
        assert!((aggregate.inertia_body[(2, 2)] - (35.0 / 3.0)).abs() < 1.0e-12);
    }

    #[test]
    fn welded_release_jettison_authority_audits_subtree_momentum() {
        let before = SpatialForce::new(Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0));
        assert!(validate_welded_release_momentum_conservation(before, before).is_ok());

        let residual = SpatialForce::new(
            Vector3::new(1.0 + 1.0e-3, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
        );
        let err = validate_welded_release_momentum_conservation(before, residual).unwrap_err();
        assert!(matches!(
            err,
            RunnerError::UnsupportedScenario { what } if what.contains("welded-release subtree momentum residual")
        ));
    }

    #[test]
    fn root_free_flyer_multibody_seed_maps_rigid_state() {
        let state = RigidBodyState::new(
            SimTime::from_seconds(4.5),
            Position3::<openbmp_core::Eci>::new(1.0, -2.0, 3.0),
            Velocity3::<openbmp_core::Eci>::new(4.0, -5.0, 6.0),
            Quaternion::<Body, openbmp_core::Eci>::from_unit_quaternion(
                nalgebra::UnitQuaternion::identity(),
            ),
            AngularVelocity3::<Body>::new(0.1, -0.2, 0.3),
            MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(12.0),
                Position3::<Body>::origin(),
                4.0,
            ),
        );
        let body = body_id_from_scenario_text("main");

        let (tree, sim_state) =
            build_root_free_flyer_multibody_state(body, &state).expect("root seed builds");

        assert_eq!(tree.n_q(), 7);
        assert_eq!(tree.n_qd(), 6);
        assert_eq!(tree.bodies()[0].id, body);
        assert_eq!(sim_state.state().time, SimTime::from_seconds(4.5));
        assert_eq!(sim_state.quaternion_offsets(), &[0]);
        assert_eq!(
            sim_state
                .q()
                .iter()
                .map(|value| value.to_bits())
                .collect::<Vec<_>>(),
            [1.0_f64, 0.0, 0.0, 0.0, 1.0, -2.0, 3.0]
                .into_iter()
                .map(f64::to_bits)
                .collect::<Vec<_>>()
        );
        assert_eq!(
            sim_state
                .qd()
                .iter()
                .map(|value| value.to_bits())
                .collect::<Vec<_>>(),
            [0.1_f64, -0.2, 0.3, 4.0, -5.0, 6.0]
                .into_iter()
                .map(f64::to_bits)
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn root_free_flyer_multibody_seed_rotates_eci_velocity_to_body_qd() {
        let orientation = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
            core::f64::consts::FRAC_PI_2,
        );
        let state = RigidBodyState::new(
            SimTime::ZERO,
            Position3::<openbmp_core::Eci>::origin(),
            Velocity3::<openbmp_core::Eci>::new(0.0, 2.0, 0.0),
            Quaternion::<Body, openbmp_core::Eci>::from_unit_quaternion(orientation),
            AngularVelocity3::<Body>::zero(),
            MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(12.0),
                Position3::<Body>::origin(),
                4.0,
            ),
        );

        let (_, sim_state) =
            build_root_free_flyer_multibody_state(body_id_from_scenario_text("main"), &state)
                .expect("root seed builds");

        assert!((sim_state.qd()[3] - 2.0).abs() < 1.0e-12);
        assert!(sim_state.qd()[4].abs() < 1.0e-12);
        assert!(sim_state.qd()[5].abs() < 1.0e-12);
    }

    #[test]
    fn root_free_flyer_multibody_projection_round_trips_rigid_state() {
        let orientation = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
            0.25,
        );
        let state = RigidBodyState::new(
            SimTime::from_seconds(2.0),
            Position3::<openbmp_core::Eci>::new(1.0, -2.0, 3.0),
            Velocity3::<openbmp_core::Eci>::new(4.0, -5.0, 6.0),
            Quaternion::<Body, openbmp_core::Eci>::from_unit_quaternion(orientation),
            AngularVelocity3::<Body>::new(0.1, -0.2, 0.3),
            MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(12.0),
                Position3::<Body>::origin(),
                4.0,
            ),
        );
        let body = body_id_from_scenario_text("main");
        let (_, sim_state) = build_root_free_flyer_multibody_state(body, &state).expect("seed");

        let projected =
            root_free_flyer_rigid_state_from_multibody_state(body, &sim_state, state.mass_props)
                .expect("projection");

        assert_eq!(projected.time, state.time);
        assert!((projected.position.vector - state.position.vector).norm() < 1.0e-12);
        assert!((projected.velocity.vector - state.velocity.vector).norm() < 1.0e-12);
        assert!(
            (projected.angular_velocity.vector - state.angular_velocity.vector).norm() < 1.0e-12
        );
        let projected_q = projected.orientation.q.into_inner();
        let expected_q = state.orientation.q.into_inner();
        assert!((projected_q.coords - expected_q.coords).norm() < 1.0e-12);
    }

    #[test]
    fn root_free_flyer_multibody_seed_rejects_invalid_mass_properties() {
        let state = RigidBodyState::new(
            SimTime::ZERO,
            Position3::<openbmp_core::Eci>::origin(),
            Velocity3::<openbmp_core::Eci>::zero(),
            Quaternion::<Body, openbmp_core::Eci>::from_unit_quaternion(
                nalgebra::UnitQuaternion::identity(),
            ),
            AngularVelocity3::<Body>::zero(),
            MassProperties::new(
                Mass::new::<kilogram>(0.0),
                Position3::<Body>::origin(),
                nalgebra::Matrix3::zeros(),
            ),
        );

        let err = build_root_free_flyer_multibody_state(body_id_from_scenario_text("main"), &state)
            .unwrap_err();

        match err {
            RunnerError::UnsupportedScenario { what } => {
                assert!(what.contains("root free-flyer multibody spatial inertia"));
            }
            other => panic!("expected UnsupportedScenario, got {other:?}"),
        }
    }

    #[test]
    fn initial_root_free_flyer_multibody_seed_uses_primary_body_mass() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(INITIAL_MULTI_BODY_SCENARIO)
            .expect("initial multi-body scenario must parse");
        let document = &scenario.document;
        let assembly = crate::assembly::synthesize_assembly(document).expect("assembly");
        let mass_resources = RigidMassResources::new(document, &assembly).expect("mass resources");
        let loaded = LoadedModels::default();
        let engine_snapshot = BTreeMap::new();
        let tank_snapshot = BTreeMap::new();
        let initial_state = build_initial_state(
            document,
            &loaded,
            &mass_resources,
            &engine_snapshot,
            &tank_snapshot,
        )
        .expect("initial state");

        let seed = build_initial_root_free_flyer_multibody_seed(
            document,
            &loaded,
            &mass_resources,
            &engine_snapshot,
            &tank_snapshot,
            &initial_state,
        )
        .expect("seed builds")
        .expect("primary body seed present");

        assert_eq!(
            initial_state.mass_props.mass_kg().to_bits(),
            4.0_f64.to_bits()
        );
        assert_eq!(
            seed.rigid_state.mass_props.mass_kg().to_bits(),
            3.0_f64.to_bits()
        );
        assert_eq!(seed.body, body_id_from_scenario_text("bus"));
        assert_eq!(seed.tree.bodies()[0].id, body_id_from_scenario_text("bus"));
    }

    #[test]
    fn initial_articulated_gimbal_multibody_seed_builds_revolute_engine_tree() {
        let toml = PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO.replace(
            "[telemetry]\n",
            "[[multi_body.gimbal_joint]]\n\
             body_id = \"main\"\n\
             engine_id = \"main_engine\"\n\
             axis_body = [0.0, 1.0, 0.0]\n\
             pivot_body_m = [0.0, 1.0, 0.0]\n\
             engine_mass_kg = 1.5\n\
             engine_cg_body_m = [0.2, 0.0, -0.4]\n\
             engine_inertia_body_kg_m2 = [[0.08, 0.0, 0.0], [0.0, 0.12, 0.0], [0.0, 0.0, 0.1]]\n\
             initial_angle_rad = 0.05\n\
             initial_rate_rad_s = -0.2\n\
             \n\
             [telemetry]\n",
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let document = &scenario.document;
        let assembly = crate::assembly::synthesize_assembly(document).expect("assembly");
        let loaded = LoadedModels::default();
        let mass_resources = RigidMassResources::new(document, &assembly).expect("mass resources");
        let engine_rack =
            crate::engines::EngineRack::build(document, &BTreeMap::new()).expect("engine rack");
        let engine_snapshot = engine_rack.snapshot_map();
        let tank_snapshot = BTreeMap::new();
        let initial_state = build_initial_state(
            document,
            &loaded,
            &mass_resources,
            &engine_snapshot,
            &tank_snapshot,
        )
        .expect("initial state");

        let seeds = build_initial_articulated_gimbal_multibody_seeds(
            document,
            &loaded,
            &mass_resources,
            &engine_snapshot,
            &tank_snapshot,
            &initial_state,
        )
        .expect("articulated gimbal seeds build");

        assert_eq!(seeds.len(), 1);
        let seed = &seeds[0];
        assert_eq!(seed.body, body_id_from_scenario_text("main"));
        assert_eq!(seed.engine, engine_id_from_scenario_text("main_engine"));
        assert_eq!(seed.tree.n_q(), 8);
        assert_eq!(seed.tree.n_qd(), 7);
        assert_eq!(seed.sim_state.q()[7].to_bits(), 0.05_f64.to_bits());
        assert_eq!(seed.sim_state.qd()[6].to_bits(), (-0.2_f64).to_bits());

        let h = seed
            .tree
            .joint_space_inertia_crba_at_state(seed.sim_state.state())
            .expect("gimbal inertia matrix");
        let gimbal_col = 6;
        let root_coupling = (0..6)
            .map(|row| h.at(row, gimbal_col).unwrap().abs())
            .fold(0.0_f64, f64::max);
        assert!(
            root_coupling > 1.0e-3,
            "scenario-declared gimbal should couple into root inertia: {root_coupling:.17e}"
        );

        let resolved_files = scenario.resolved_files().expect("resolved files");
        RigidBodySession::prepare(&scenario, &resolved_files)
            .expect("session prepare validates articulated gimbal seed");
    }

    #[test]
    fn articulated_gimbal_shadow_derivative_uses_live_engine_thrust() {
        let toml = PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO.replace(
            "[[multi_body.attitude_target]]\n",
            "[[multi_body.gimbal_joint]]\n\
             body_id = \"main\"\n\
             engine_id = \"main_engine\"\n\
             axis_body = [0.0, 1.0, 0.0]\n\
             pivot_body_m = [0.0, 1.0, 0.0]\n\
             engine_mass_kg = 1.5\n\
             engine_cg_body_m = [0.2, 0.0, -0.4]\n\
             engine_inertia_body_kg_m2 = [[0.08, 0.0, 0.0], [0.0, 0.12, 0.0], [0.0, 0.0, 0.1]]\n\
             thrust_application_body_m = [0.2, 0.0, 0.0]\n\
             initial_angle_rad = 0.0\n\
             initial_rate_rad_s = 0.0\n\
             \n\
             [[multi_body.attitude_target]]\n",
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step fires engine command");
        session
            .step_once(&scenario.document, None)
            .expect("second step mirrors articulated gimbal thrust");

        let engine = engine_id_from_scenario_text("main_engine");
        let powered_snapshot = session
            .engine_snapshots()
            .get(&engine)
            .copied()
            .expect("engine snapshot present");
        assert!(
            powered_snapshot.thrust_body.norm() > 1.0,
            "scenario command should produce powered thrust: {:?}",
            powered_snapshot.thrust_body
        );
        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        assert!(
            (shadow.seed.sim_state.q()[7] + 0.1).abs() < 1.0e-12,
            "positive propulsion pitch should map to child-from-parent gimbal angle: {:?}",
            shadow.seed.sim_state.q()
        );
        assert!(
            (shadow.seed.sim_state.qd()[6] + 1.0).abs() < 1.0e-12,
            "gimbal joint rate should finite-difference the live handoff: {:?}",
            shadow.seed.sim_state.qd()
        );
        let derivative = shadow
            .last_derivative
            .as_ref()
            .expect("articulated gimbal derivative recorded");
        assert_eq!(derivative.qd_dot().len(), 7);
        let root_accel = (0..6)
            .map(|index| derivative.qd_dot()[index].abs())
            .fold(0.0_f64, f64::max);
        assert!(
            root_accel > 1.0e-6,
            "engine thrust should drive root acceleration through articulated child: {:?}",
            derivative.qd_dot()
        );
        assert!(
            derivative.qd_dot()[6].abs() > 1.0e-6,
            "offset thrust should drive revolute gimbal acceleration: {:?}",
            derivative.qd_dot()
        );
    }

    fn gimbal_qdd_from_scenario_after_steps(toml: &str, steps: usize) -> Vec<f64> {
        let scenario = openbmp_scenario::Scenario::from_toml_str(toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        for _ in 0..steps {
            session
                .step_once(&scenario.document, None)
                .expect("scenario step mirrors articulated gimbal thrust");
        }

        let engine = engine_id_from_scenario_text("main_engine");
        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        shadow
            .last_derivative
            .as_ref()
            .expect("articulated gimbal derivative recorded")
            .qd_dot()
            .to_vec()
    }

    fn gimbal_qdd_from_scenario(toml: &str) -> Vec<f64> {
        gimbal_qdd_from_scenario_after_steps(toml, 2)
    }

    fn gimbal_acceleration_from_scenario(toml: &str, qd_index: usize) -> f64 {
        gimbal_qdd_from_scenario(toml)[qd_index]
    }

    fn primary_gimbal_acceleration_from_scenario(toml: &str) -> f64 {
        gimbal_acceleration_from_scenario(toml, 6)
    }

    fn primary_gimbal_acceleration_from_scenario_after_steps(toml: &str, steps: usize) -> f64 {
        gimbal_qdd_from_scenario_after_steps(toml, steps)[6]
    }

    fn primary_gimbal_contact_mechanism_joint_toml(mechanism_id: &str) -> String {
        format!(
            "[[multi_body.gimbal_joint]]\n\
             body_id = \"main\"\n\
             engine_id = \"main_engine\"\n\
             axis_body = [0.0, 1.0, 0.0]\n\
             pivot_body_m = [0.0, 1.0, 0.0]\n\
             engine_mass_kg = 1.5\n\
             engine_cg_body_m = [0.2, 0.0, -0.4]\n\
             engine_inertia_body_kg_m2 = [[0.08, 0.0, 0.0], [0.0, 0.12, 0.0], [0.0, 0.0, 0.1]]\n\
             thrust_application_body_m = [0.2, 0.0, 0.0]\n\
             initial_angle_rad = 0.0\n\
             initial_rate_rad_s = 0.0\n\
             mechanism_id = \"{mechanism_id}\"\n\
             \n\
             [[multi_body.attitude_target]]\n"
        )
    }

    fn primary_gimbal_contact_stop_block_toml() -> &'static str {
        "[contact]\n\
             kind = \"half_space\"\n\
             ground_altitude_m = 0.0\n\
             geometry = \"point\"\n\
             normal_law = \"kelvin_voigt\"\n\
             stiffness_n_m = 2000.0\n\
             damping_n_s_m = 0.0\n\
             friction_coefficient = 0.0\n\
             effective_mass_kg = 1.0\n\
             substeps = 10\n\
             \n\
             [[contact.mechanism]]\n\
             id = \"main-engine-lower-stop\"\n\
             coordinate = \"main_engine\"\n\
             kind = \"stop\"\n\
             side = \"lower\"\n\
             limit_rad = 0.0\n\
             stiffness_n_m = 1000.0\n\
             damping_n_s_m = 0.0\n\
             \n\
             [multi_body]\n"
    }

    fn primary_gimbal_contact_latch_block_toml() -> &'static str {
        "[contact]\n\
             kind = \"half_space\"\n\
             ground_altitude_m = 0.0\n\
             geometry = \"point\"\n\
             normal_law = \"kelvin_voigt\"\n\
             stiffness_n_m = 2000.0\n\
             damping_n_s_m = 0.0\n\
             friction_coefficient = 0.0\n\
             effective_mass_kg = 1.0\n\
             substeps = 10\n\
             \n\
             [[contact.mechanism]]\n\
             id = \"main-engine-latch\"\n\
             coordinate = \"main_engine\"\n\
             kind = \"latch\"\n\
             center_rad = 0.0\n\
             half_width_rad = 1.0e-9\n\
             \n\
             [multi_body]\n"
    }

    fn primary_gimbal_contact_live_latch_block_toml() -> &'static str {
        "[contact]\n\
             kind = \"half_space\"\n\
             ground_altitude_m = 0.0\n\
             geometry = \"point\"\n\
             normal_law = \"kelvin_voigt\"\n\
             stiffness_n_m = 2000.0\n\
             damping_n_s_m = 0.0\n\
             friction_coefficient = 0.0\n\
             effective_mass_kg = 1.0\n\
             substeps = 10\n\
             \n\
             [[contact.mechanism]]\n\
             id = \"main-engine-live-latch\"\n\
             coordinate = \"main_engine\"\n\
             kind = \"latch\"\n\
             center_rad = -0.1\n\
             half_width_rad = 1.0e-9\n\
             \n\
             [multi_body]\n"
    }

    fn primary_gimbal_contact_mechanism_scenario_toml(
        mechanism_id: &str,
        contact_block: &str,
        root_authority: bool,
    ) -> String {
        let joint = primary_gimbal_contact_mechanism_joint_toml(mechanism_id);
        let scenario = PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace(
                "models = [\"thrust\"]",
                "models = [\"thrust\", \"contact\"]",
            )
            .replace("[multi_body]\n", contact_block)
            .replace("[[multi_body.attitude_target]]\n", &joint);
        if root_authority {
            scenario.replace(
                "[multi_body]\nprimary_body_id = \"main\"\n",
                "[multi_body]\nprimary_body_id = \"main\"\npropagation_authority = \"articulated_gimbal_root\"\n",
            )
        } else {
            scenario
        }
    }

    fn primary_gimbal_contact_unbound_mechanism_scenario_toml(
        contact_block: &str,
        root_authority: bool,
    ) -> String {
        let scenario = PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace(
                "models = [\"thrust\"]",
                "models = [\"thrust\", \"contact\"]",
            )
            .replace("[multi_body]\n", contact_block)
            .replace(
                "[[multi_body.attitude_target]]\n",
                primary_gimbal_cfe_lock_joint_toml(),
            );
        if root_authority {
            scenario.replace(
                "[multi_body]\nprimary_body_id = \"main\"\n",
                "[multi_body]\nprimary_body_id = \"main\"\npropagation_authority = \"articulated_gimbal_root\"\n",
            )
        } else {
            scenario
        }
    }

    #[test]
    fn articulated_gimbal_contact_mechanism_stop_adds_joint_generalized_force() {
        let gimbal_joint = "[[multi_body.gimbal_joint]]\n\
             body_id = \"main\"\n\
             engine_id = \"main_engine\"\n\
             axis_body = [0.0, 1.0, 0.0]\n\
             pivot_body_m = [0.0, 1.0, 0.0]\n\
             engine_mass_kg = 1.5\n\
             engine_cg_body_m = [0.2, 0.0, -0.4]\n\
             engine_inertia_body_kg_m2 = [[0.08, 0.0, 0.0], [0.0, 0.12, 0.0], [0.0, 0.0, 0.1]]\n\
             thrust_application_body_m = [0.2, 0.0, 0.0]\n\
             initial_angle_rad = 0.0\n\
             initial_rate_rad_s = 0.0\n\
             \n\
             [[multi_body.attitude_target]]\n";
        let baseline = PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace("[[multi_body.attitude_target]]\n", gimbal_joint);
        let mechanism_gimbal_joint = gimbal_joint.replace(
            "initial_rate_rad_s = 0.0\n",
            "initial_rate_rad_s = 0.0\n\
             mechanism_id = \"main-engine-lower-stop\"\n",
        );
        let mechanism_contact = "[contact]\n\
             kind = \"half_space\"\n\
             ground_altitude_m = 0.0\n\
             geometry = \"point\"\n\
             normal_law = \"kelvin_voigt\"\n\
             stiffness_n_m = 2000.0\n\
             damping_n_s_m = 0.0\n\
             friction_coefficient = 0.0\n\
             effective_mass_kg = 1.0\n\
             substeps = 10\n\
             \n\
             [[contact.mechanism]]\n\
             id = \"main-engine-lower-stop\"\n\
             coordinate = \"main_engine\"\n\
             kind = \"stop\"\n\
             side = \"lower\"\n\
             limit_rad = 0.0\n\
             stiffness_n_m = 1000.0\n\
             damping_n_s_m = 0.0\n\
             \n\
             [multi_body]\n";
        let with_mechanism = PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace(
                "models = [\"thrust\"]",
                "models = [\"thrust\", \"contact\"]",
            )
            .replace("[multi_body]\n", mechanism_contact)
            .replace("[[multi_body.attitude_target]]\n", &mechanism_gimbal_joint);

        let baseline_qdd = primary_gimbal_acceleration_from_scenario(&baseline);
        let mechanism_qdd = primary_gimbal_acceleration_from_scenario(&with_mechanism);

        assert!(
            mechanism_qdd - baseline_qdd > 1.0,
            "lower-stop mechanism should add positive primary gimbal acceleration: baseline={baseline_qdd:.17e} mechanism={mechanism_qdd:.17e}"
        );
    }

    #[test]
    fn articulated_gimbal_contact_mechanism_stop_participates_in_root_authority() {
        let baseline = primary_gimbal_contact_unbound_mechanism_scenario_toml(
            primary_gimbal_contact_stop_block_toml(),
            true,
        );
        let with_stop = primary_gimbal_contact_mechanism_scenario_toml(
            "main-engine-lower-stop",
            primary_gimbal_contact_stop_block_toml(),
            true,
        );

        let baseline_qdd = primary_gimbal_acceleration_from_scenario_after_steps(&baseline, 7);
        let mechanism_qdd = primary_gimbal_acceleration_from_scenario_after_steps(&with_stop, 7);
        assert!(
            mechanism_qdd - baseline_qdd > 1.0,
            "root-authority stop mechanism should alter the authoritative gimbal acceleration: baseline={baseline_qdd:.17e} mechanism={mechanism_qdd:.17e}"
        );

        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&with_stop).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");
        for _ in 0..7 {
            session
                .step_once(&scenario.document, None)
                .expect("step applies stop-coupled root authority");
        }
        let engine = engine_id_from_scenario_text("main_engine");
        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        let authoritative = shadow
            .last_authoritative_root_state
            .as_ref()
            .expect("stop-coupled root handoff recorded");
        assert_rigid_forecast_matches_state(
            authoritative,
            session.state(),
            session.state().time.as_seconds(),
        );
    }

    #[test]
    fn articulated_gimbal_tvc_load_torque_participates_in_root_authority_generalized_force() {
        let toml = PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace(
                "[multi_body]\nprimary_body_id = \"main\"\n",
                "[multi_body]\nprimary_body_id = \"main\"\npropagation_authority = \"articulated_gimbal_root\"\n",
            )
            .replace(
                "[[multi_body.attitude_target]]\n",
                primary_gimbal_cfe_lock_joint_toml(),
            );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");
        let engine = engine_id_from_scenario_text("main_engine");
        let seed = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present")
            .seed
            .clone();

        let baseline_snapshot = BTreeMap::from([(engine, openbmp_sim::EngineSnapshot::idle())]);
        let loaded_snapshot = BTreeMap::from([(
            engine,
            openbmp_sim::EngineSnapshot {
                tvc_pitch_load_torque_n_m: 10.0,
                ..openbmp_sim::EngineSnapshot::idle()
            },
        )]);
        let mut baseline_latches = Vec::new();
        let baseline = articulated_gimbal_multibody_derivative_from_engine_thrust(
            &seed,
            &baseline_snapshot,
            session.kernel_step_s,
            &mut baseline_latches,
        )
        .expect("baseline derivative");
        let mut loaded_latches = Vec::new();
        let loaded = articulated_gimbal_multibody_derivative_from_engine_thrust(
            &seed,
            &loaded_snapshot,
            session.kernel_step_s,
            &mut loaded_latches,
        )
        .expect("loaded derivative");

        assert!(
            loaded.derivative.qd_dot()[6] - baseline.derivative.qd_dot()[6] > 1.0,
            "positive TVC pitch load torque should add positive primary generalized acceleration: baseline={:.17e} loaded={:.17e}",
            baseline.derivative.qd_dot()[6],
            loaded.derivative.qd_dot()[6]
        );
    }

    #[test]
    fn articulated_gimbal_tvc_inertial_reaction_participates_in_root_authority_body_moment() {
        let toml = PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace(
                "[multi_body]\nprimary_body_id = \"main\"\n",
                "[multi_body]\nprimary_body_id = \"main\"\npropagation_authority = \"articulated_gimbal_root\"\n",
            )
            .replace(
                "[[multi_body.attitude_target]]\n",
                primary_gimbal_cfe_lock_joint_toml(),
            );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");
        let engine = engine_id_from_scenario_text("main_engine");
        let seed = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present")
            .seed
            .clone();

        let baseline_snapshot = BTreeMap::from([(engine, openbmp_sim::EngineSnapshot::idle())]);
        let reaction_snapshot = BTreeMap::from([(
            engine,
            openbmp_sim::EngineSnapshot {
                tvc_pitch_inertial_reaction_torque_n_m: 10.0,
                ..openbmp_sim::EngineSnapshot::idle()
            },
        )]);
        let mut baseline_latches = Vec::new();
        let baseline = articulated_gimbal_multibody_derivative_from_engine_thrust(
            &seed,
            &baseline_snapshot,
            session.kernel_step_s,
            &mut baseline_latches,
        )
        .expect("baseline derivative");
        let mut reaction_latches = Vec::new();
        let reaction = articulated_gimbal_multibody_derivative_from_engine_thrust(
            &seed,
            &reaction_snapshot,
            session.kernel_step_s,
            &mut reaction_latches,
        )
        .expect("reaction derivative");

        assert!(
            reaction.derivative.qd_dot()[1] - baseline.derivative.qd_dot()[1] > 1.0,
            "positive TVC pitch inertial reaction should add positive root pitch acceleration: baseline={:.17e} reaction={:.17e}",
            baseline.derivative.qd_dot()[1],
            reaction.derivative.qd_dot()[1]
        );
    }

    #[test]
    fn articulated_gimbal_contact_mechanism_latch_state_carries_in_root_authority() {
        let toml = primary_gimbal_contact_mechanism_scenario_toml(
            "main-engine-latch",
            primary_gimbal_contact_latch_block_toml(),
            true,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");
        let engine = engine_id_from_scenario_text("main_engine");
        assert_eq!(
            session
                .articulated_gimbal_shadows
                .get(&engine)
                .expect("articulated gimbal shadow present")
                .mechanism_latch_states
                .len(),
            1
        );
        assert!(
            session
                .articulated_gimbal_shadows
                .get(&engine)
                .expect("articulated gimbal shadow present")
                .mechanism_latch_states[0]
                .is_engaged(),
            "initial articulated gimbal state should engage the monotone latch"
        );

        for _ in 0..7 {
            session
                .step_once(&scenario.document, None)
                .expect("step carries latch state and applies root authority");
        }
        let first_authoritative = *session
            .articulated_gimbal_shadows
            .get(&engine)
            .and_then(|shadow| shadow.last_authoritative_root_state.as_ref())
            .expect("latch-coupled root handoff recorded");
        assert!(
            session
                .articulated_gimbal_shadows
                .get(&engine)
                .expect("articulated gimbal shadow present")
                .mechanism_latch_states[0]
                .is_engaged(),
            "root authority should preserve the monotone latch state"
        );

        session
            .step_once(&scenario.document, None)
            .expect("next step carries latch-coupled internal state");
        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        assert!(
            shadow.mechanism_latch_states[0].is_engaged(),
            "root-authority carry should preserve monotone latch state"
        );
        assert_rigid_forecast_matches_state(
            &articulated_gimbal_root_rigid_state_from_seed(&shadow.seed)
                .expect("carried latch root projects"),
            &first_authoritative,
            first_authoritative.time.as_seconds(),
        );
    }

    #[test]
    fn articulated_gimbal_contact_mechanism_latch_captures_live_deflection_after_root_handoff() {
        let toml = primary_gimbal_contact_mechanism_scenario_toml(
            "main-engine-live-latch",
            primary_gimbal_contact_live_latch_block_toml(),
            true,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");
        let engine = engine_id_from_scenario_text("main_engine");
        assert_eq!(
            session
                .articulated_gimbal_shadows
                .get(&engine)
                .expect("articulated gimbal shadow present")
                .mechanism_latch_states,
            vec![LatchState::disengaged()],
            "live-angle latch should start open before commanded deflection"
        );

        session
            .step_once(&scenario.document, None)
            .expect("first idle root-authority handoff stays outside latch window");
        assert_eq!(
            session
                .articulated_gimbal_shadows
                .get(&engine)
                .expect("articulated gimbal shadow present")
                .mechanism_latch_states,
            vec![LatchState::disengaged()],
            "idle handoff should not engage the live-angle latch"
        );

        for _ in 0..5 {
            session
                .step_once(&scenario.document, None)
                .expect("advance to first post-command contact substep");
        }
        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        assert!(
            (shadow.seed.sim_state.q()[7] + 0.1).abs() < 1.0e-12,
            "live gimbal deflection should refresh the authoritative joint coordinate: {:?}",
            shadow.seed.sim_state.q()
        );
        assert_eq!(
            shadow.mechanism_latch_states,
            vec![LatchState::engaged()],
            "live captured joint coordinate should engage the mechanism latch after handoff"
        );
    }

    fn primary_gimbal_cfe_lock_joint_toml() -> &'static str {
        "[[multi_body.gimbal_joint]]\n\
             body_id = \"main\"\n\
             engine_id = \"main_engine\"\n\
             axis_body = [0.0, 1.0, 0.0]\n\
             pivot_body_m = [0.0, 1.0, 0.0]\n\
             engine_mass_kg = 1.5\n\
             engine_cg_body_m = [0.2, 0.0, -0.4]\n\
             engine_inertia_body_kg_m2 = [[0.08, 0.0, 0.0], [0.0, 0.12, 0.0], [0.0, 0.0, 0.1]]\n\
             thrust_application_body_m = [0.2, 0.0, 0.0]\n\
             initial_angle_rad = 0.0\n\
             initial_rate_rad_s = 0.0\n\
             \n\
             [[multi_body.attitude_target]]\n"
    }

    fn primary_gimbal_cfe_lock_scenario_toml() -> String {
        PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace(
                "[[multi_body.attitude_target]]\n",
                primary_gimbal_cfe_lock_joint_toml(),
            )
            .replace(
                "[[multi_body.attitude_target]]\n",
                "[[multi_body.cfe_loop]]\n\
                 loop_id = \"main-engine-primary-lock\"\n\
                 position_residual = 0.0\n\
                 velocity_residual_per_s = 0.0\n\
                 velocity_bias_acceleration_per_s2 = 0.0\n\
                 \n\
                 [[multi_body.cfe_loop.jacobian_entry]]\n\
                 generalized_velocity_index = 6\n\
                 jacobian_value = 1.0\n\
                 \n\
                 [[multi_body.attitude_target]]\n",
            )
    }

    fn primary_gimbal_cfe_lock_with_extra_loop_toml(extra_loop: &str) -> String {
        primary_gimbal_cfe_lock_scenario_toml().replace(
            "[[multi_body.attitude_target]]\n",
            &format!("{extra_loop}\n[[multi_body.attitude_target]]\n"),
        )
    }

    fn primary_gimbal_concrete_cfe_lock_scenario_toml() -> String {
        PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace("gimbal_pitch_rad = 0.1", "gimbal_pitch_rad = 0.0")
            .replace(
                "[[multi_body.attitude_target]]\n",
                primary_gimbal_cfe_lock_joint_toml(),
            )
            .replace(
                "[[multi_body.attitude_target]]\n",
                "[[multi_body.cfe_gimbal_lock]]\n\
                 loop_id = \"main-engine-concrete-primary-lock\"\n\
                 engine_id = \"main_engine\"\n\
                 axis = \"primary\"\n\
                 locked_angle_rad = 0.0\n\
                 velocity_bias_acceleration_per_s2 = 0.0\n\
                 \n\
                 [[multi_body.attitude_target]]\n",
            )
    }

    fn primary_gimbal_concrete_cfe_lock_baseline_toml() -> String {
        PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace("gimbal_pitch_rad = 0.1", "gimbal_pitch_rad = 0.0")
            .replace(
                "[[multi_body.attitude_target]]\n",
                primary_gimbal_cfe_lock_joint_toml(),
            )
    }

    fn primary_gimbal_concrete_cfe_planar_four_bar_scenario_toml() -> String {
        primary_multibody_two_axis_gimbal_shadow_scenario_toml().replace(
            "[[multi_body.attitude_target]]\n",
            "[[multi_body.cfe_planar_four_bar]]\n\
             loop_id = \"main-engine-planar-four-bar\"\n\
             engine_id = \"main_engine\"\n\
             input_axis = \"secondary\"\n\
             output_axis = \"primary\"\n\
             ground_pivot_a_m = [0.0, 0.0]\n\
             ground_pivot_d_m = [-0.996877212836496, 0.07490629295875324]\n\
             input_link_m = 0.5\n\
             coupler_link_m = 1.0\n\
             output_link_m = 0.5\n\
             velocity_bias_acceleration_per_s2 = 0.0\n\
             \n\
             [[multi_body.attitude_target]]\n",
        )
    }

    fn primary_gimbal_cfe_lock_root_authority_scenario_toml() -> String {
        primary_gimbal_cfe_lock_scenario_toml().replace(
            "[multi_body]\nprimary_body_id = \"main\"\n",
            "[multi_body]\nprimary_body_id = \"main\"\npropagation_authority = \"articulated_gimbal_root\"\n",
        )
    }

    #[test]
    fn articulated_gimbal_cfe_loop_constrains_shadow_generalized_acceleration() {
        let gimbal_joint = primary_gimbal_cfe_lock_joint_toml();
        let baseline = PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace("[[multi_body.attitude_target]]\n", gimbal_joint);
        let with_cfe = primary_gimbal_cfe_lock_scenario_toml();

        let baseline_qdd = primary_gimbal_acceleration_from_scenario(&baseline);
        let cfe_qdd = primary_gimbal_acceleration_from_scenario(&with_cfe);

        assert!(
            baseline_qdd.abs() > 1.0e-6,
            "baseline thrust should accelerate the primary gimbal slot: {baseline_qdd:.17e}"
        );
        assert!(
            cfe_qdd.abs() < 1.0e-8,
            "zero-residual CFE row should drive selected slot acceleration to zero: baseline={baseline_qdd:.17e} cfe={cfe_qdd:.17e}"
        );
    }

    #[test]
    fn articulated_gimbal_cfe_loop_constrains_root_authority_handoff() {
        let toml = primary_gimbal_cfe_lock_root_authority_scenario_toml();
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step fires engine command");
        session
            .step_once(&scenario.document, None)
            .expect("second step applies CFE-constrained root authority");

        let engine = engine_id_from_scenario_text("main_engine");
        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        let derivative = shadow
            .last_derivative
            .as_ref()
            .expect("CFE-constrained derivative recorded");
        assert!(
            derivative.qd_dot()[6].abs() < 1.0e-8,
            "root-authority forecast should use the CFE-constrained primary gimbal acceleration: {:?}",
            derivative.qd_dot()
        );
        let authoritative = shadow
            .last_authoritative_root_state
            .as_ref()
            .expect("CFE-constrained root handoff recorded");
        assert_rigid_forecast_matches_state(authoritative, session.state(), 0.2);
    }

    #[test]
    fn articulated_gimbal_cfe_loop_emits_solve_telemetry() {
        let toml = primary_gimbal_cfe_lock_scenario_toml();
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let outcome = run(&scenario, &resolved_files, None).expect("scenario runs");

        let prefix = "multi_body.cfe_loop.main-engine-primary-lock";
        let lambda_name = format!("{prefix}.lagrange_multiplier");
        let stabilized_name = format!("{prefix}.stabilized_acceleration_per_s2");
        let residual_name = format!("{prefix}.acceleration_residual_per_s2");
        assert!(has_channel(&outcome, &lambda_name));
        assert!(has_channel(&outcome, &stabilized_name));
        assert!(has_channel(&outcome, &residual_name));

        let lambda = f64_column(&outcome, &lambda_name);
        let stabilized = f64_column(&outcome, &stabilized_name);
        let residual = f64_column(&outcome, &residual_name);
        assert_eq!(lambda.len(), outcome.table.rows().len());
        assert_eq!(stabilized.len(), lambda.len());
        assert_eq!(residual.len(), lambda.len());
        assert!(lambda.iter().all(|value| value.is_finite()));
        assert!(stabilized.iter().all(|value| value.is_finite()));
        assert!(residual.iter().all(|value| value.is_finite()));
        assert!(
            lambda.iter().any(|value| value.abs() > 1.0e-6),
            "powered CFE solve should emit a nonzero multiplier: {lambda:?}"
        );
        assert!(
            residual.iter().all(|value| value.abs() < 1.0e-8),
            "zero-residual CFE row should solve to near-zero acceleration residuals: {residual:?}"
        );
    }

    #[test]
    fn articulated_gimbal_cfe_gimbal_lock_constrains_live_generalized_acceleration() {
        let baseline = primary_gimbal_concrete_cfe_lock_baseline_toml();
        let with_lock = primary_gimbal_concrete_cfe_lock_scenario_toml();

        let baseline_qdd = primary_gimbal_acceleration_from_scenario(&baseline);
        let lock_qdd = primary_gimbal_acceleration_from_scenario(&with_lock);

        assert!(
            baseline_qdd.abs() > 1.0e-6,
            "baseline offset thrust should accelerate the primary gimbal slot: {baseline_qdd:.17e}"
        );
        assert!(
            lock_qdd.abs() < 1.0e-8,
            "concrete zero-residual gimbal lock should drive selected slot acceleration to zero: baseline={baseline_qdd:.17e} lock={lock_qdd:.17e}"
        );
    }

    #[test]
    fn articulated_gimbal_cfe_gimbal_lock_emits_solve_telemetry() {
        let toml = primary_gimbal_concrete_cfe_lock_scenario_toml();
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let outcome = run(&scenario, &resolved_files, None).expect("scenario runs");

        let prefix = "multi_body.cfe_loop.main-engine-concrete-primary-lock";
        let lambda = f64_column(&outcome, &format!("{prefix}.lagrange_multiplier"));
        let residual = f64_column(&outcome, &format!("{prefix}.acceleration_residual_per_s2"));
        assert!(lambda.iter().all(|value| value.is_finite()));
        assert!(residual.iter().all(|value| value.is_finite()));
        assert!(
            lambda.iter().any(|value| value.abs() > 1.0e-6),
            "powered concrete CFE lock should emit a nonzero multiplier: {lambda:?}"
        );
        assert!(
            residual.iter().all(|value| value.abs() < 1.0e-8),
            "concrete CFE lock should solve to near-zero acceleration residuals: {residual:?}"
        );
    }

    #[test]
    fn articulated_gimbal_cfe_planar_four_bar_emits_solve_telemetry() {
        let toml = primary_gimbal_concrete_cfe_planar_four_bar_scenario_toml();
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let outcome = run(&scenario, &resolved_files, None).expect("scenario runs");

        let prefix = "multi_body.cfe_loop.main-engine-planar-four-bar";
        let lambda = f64_column(&outcome, &format!("{prefix}.lagrange_multiplier"));
        let residual = f64_column(&outcome, &format!("{prefix}.acceleration_residual_per_s2"));
        assert!(lambda.iter().all(|value| value.is_finite()));
        assert!(residual.iter().all(|value| value.is_finite()));
        assert!(
            lambda.iter().any(|value| value.abs() > 1.0e-6),
            "powered concrete four-bar CFE solve should emit a nonzero multiplier: {lambda:?}"
        );
        assert!(
            residual.iter().all(|value| value.abs() < 1.0e-8),
            "concrete four-bar CFE row should solve to near-zero acceleration residuals: {residual:?}"
        );
    }

    #[test]
    fn articulated_gimbal_cfe_loop_rejects_telemetry_suffix_collision() {
        let extra_loop = "[[multi_body.cfe_loop]]\n\
             loop_id = \"main_engine_primary_lock\"\n\
             position_residual = 0.0\n\
             velocity_residual_per_s = 0.0\n\
             velocity_bias_acceleration_per_s2 = 0.0\n\
             \n\
             [[multi_body.cfe_loop.jacobian_entry]]\n\
             generalized_velocity_index = 6\n\
             jacobian_value = 1.0\n";
        let toml = primary_gimbal_cfe_lock_with_extra_loop_toml(extra_loop)
            .replace("main-engine-primary-lock", "main engine primary lock");
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let err = RigidBodySession::prepare(&scenario, &resolved_files).unwrap_err();
        match err {
            RunnerError::UnsupportedScenario { what } => {
                assert!(
                    what.contains(
                        "multi_body CFE telemetry suffix `main_engine_primary_lock` is not unique"
                    ),
                    "unexpected CFE telemetry suffix error: {what}"
                );
            }
            other => panic!("expected UnsupportedScenario, got {other:?}"),
        }
    }

    #[test]
    fn articulated_gimbal_cfe_loop_rejects_out_of_range_generalized_velocity_index() {
        let gimbal_joint = "[[multi_body.gimbal_joint]]\n\
             body_id = \"main\"\n\
             engine_id = \"main_engine\"\n\
             axis_body = [0.0, 1.0, 0.0]\n\
             pivot_body_m = [0.0, 1.0, 0.0]\n\
             engine_mass_kg = 1.5\n\
             engine_cg_body_m = [0.2, 0.0, -0.4]\n\
             engine_inertia_body_kg_m2 = [[0.08, 0.0, 0.0], [0.0, 0.12, 0.0], [0.0, 0.0, 0.1]]\n\
             thrust_application_body_m = [0.2, 0.0, 0.0]\n\
             initial_angle_rad = 0.0\n\
             initial_rate_rad_s = 0.0\n\
             \n\
             [[multi_body.cfe_loop]]\n\
             loop_id = \"bad-column\"\n\
             position_residual = 0.0\n\
             velocity_residual_per_s = 0.0\n\
             \n\
             [[multi_body.cfe_loop.jacobian_entry]]\n\
             generalized_velocity_index = 99\n\
             jacobian_value = 1.0\n\
             \n\
             [[multi_body.attitude_target]]\n";
        let toml = PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace("[[multi_body.attitude_target]]\n", gimbal_joint);
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let err = RigidBodySession::prepare(&scenario, &resolved_files).unwrap_err();
        match err {
            RunnerError::UnsupportedScenario { what } => {
                assert!(
                    what.contains("multi_body.cfe_loop `bad-column` generalized_velocity_index 99"),
                    "unexpected CFE loop bounds error: {what}"
                );
            }
            other => panic!("expected UnsupportedScenario, got {other:?}"),
        }
    }

    fn primary_multibody_two_axis_gimbal_shadow_scenario_toml() -> String {
        PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace(
                "gimbal_pitch_rad = 0.1, gimbal_yaw_rad = 0.0",
                "gimbal_pitch_rad = 0.1, gimbal_yaw_rad = 0.05",
            )
            .replace(
                "[[multi_body.attitude_target]]\n",
                "[[multi_body.gimbal_joint]]\n\
                 body_id = \"main\"\n\
                 engine_id = \"main_engine\"\n\
                 axis_body = [0.0, 1.0, 0.0]\n\
                 secondary_axis_body = [1.0, 0.0, 0.0]\n\
                 neutral_thrust_body = [0.0, 0.0, 1.0]\n\
                 pivot_body_m = [0.0, 1.0, 0.0]\n\
                 engine_mass_kg = 1.5\n\
                 engine_cg_body_m = [0.2, 0.0, -0.4]\n\
                 engine_inertia_body_kg_m2 = [[0.08, 0.0, 0.0], [0.0, 0.12, 0.0], [0.0, 0.0, 0.1]]\n\
                 thrust_application_body_m = [0.2, 0.3, 0.0]\n\
                 initial_angle_rad = 0.0\n\
                 initial_rate_rad_s = 0.0\n\
                 \n\
                 [[multi_body.attitude_target]]\n",
            )
    }

    fn primary_multibody_dual_gimbal_shadow_scenario_toml() -> String {
        PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO
            .replace(
                "[[vehicle.assembly.effectors]]\n",
                "[[vehicle.assembly.engines]]\n\
                 id = \"aux_engine\"\n\
                 mounted_to = \"main\"\n\
                 kind = { kind = \"liquid_engine\" }\n\
                 mount_point_body_m = [0.0, -1.2, 0.0]\n\
                 limits = { max_thrust_n = 80.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.2 }\n\
                 \n\
                 [[vehicle.assembly.effectors]]\n",
            )
            .replace(
                "once = true\n\n[telemetry]\n",
                "once = true\n\
                 \n\
                 [[scenario_script.events]]\n\
                 id = \"ignite_aux\"\n\
                 trigger = { kind = \"at_time\", time_s = 0.05 }\n\
                 action = { kind = \"engine_command\", id = \"aux_engine\", command = { throttle_unit = 0.8, gimbal_pitch_rad = -0.06, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }\n\
                 once = true\n\
                 \n\
                 [telemetry]\n",
            )
            .replace(
                "[[multi_body.attitude_target]]\n",
                "[[multi_body.gimbal_joint]]\n\
                 body_id = \"main\"\n\
                 engine_id = \"main_engine\"\n\
                 axis_body = [0.0, 1.0, 0.0]\n\
                 pivot_body_m = [0.0, 1.0, 0.0]\n\
                 engine_mass_kg = 1.5\n\
                 engine_cg_body_m = [0.2, 0.0, -0.4]\n\
                 engine_inertia_body_kg_m2 = [[0.08, 0.0, 0.0], [0.0, 0.12, 0.0], [0.0, 0.0, 0.1]]\n\
                 thrust_application_body_m = [0.2, 0.0, 0.0]\n\
                 initial_angle_rad = 0.0\n\
                 initial_rate_rad_s = 0.0\n\
                 \n\
                 [[multi_body.gimbal_joint]]\n\
                 body_id = \"main\"\n\
                 engine_id = \"aux_engine\"\n\
                 axis_body = [0.0, 1.0, 0.0]\n\
                 pivot_body_m = [0.0, -1.2, 0.0]\n\
                 engine_mass_kg = 1.0\n\
                 engine_cg_body_m = [0.15, 0.0, -0.35]\n\
                 engine_inertia_body_kg_m2 = [[0.05, 0.0, 0.0], [0.0, 0.08, 0.0], [0.0, 0.0, 0.06]]\n\
                 thrust_application_body_m = [0.18, 0.0, 0.0]\n\
                 initial_angle_rad = 0.0\n\
                 initial_rate_rad_s = 0.0\n\
                 \n\
                 [[multi_body.attitude_target]]\n",
            )
    }

    #[test]
    fn articulated_gimbal_shadow_derivative_tracks_multiple_primary_engine_joints() {
        let toml = primary_multibody_dual_gimbal_shadow_scenario_toml();
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        let main_engine = engine_id_from_scenario_text("main_engine");
        let aux_engine = engine_id_from_scenario_text("aux_engine");
        assert_eq!(session.articulated_gimbal_shadows.len(), 2);
        assert!(
            session
                .articulated_gimbal_shadows
                .contains_key(&main_engine)
        );
        assert!(session.articulated_gimbal_shadows.contains_key(&aux_engine));

        session
            .step_once(&scenario.document, None)
            .expect("first step fires engine commands");
        session
            .step_once(&scenario.document, None)
            .expect("second step mirrors both articulated gimbals");

        for (engine, expected_q, expected_qd) in
            [(main_engine, -0.1, -1.0), (aux_engine, 0.06, 0.6)]
        {
            let powered_snapshot = session
                .engine_snapshots()
                .get(&engine)
                .copied()
                .expect("engine snapshot present");
            assert!(
                powered_snapshot.thrust_body.norm() > 1.0,
                "scenario command should produce powered thrust for {engine:?}: {:?}",
                powered_snapshot.thrust_body
            );

            let shadow = session
                .articulated_gimbal_shadows
                .get(&engine)
                .expect("articulated gimbal shadow present");
            assert_eq!(shadow.seed.engine, engine);
            assert_eq!(shadow.seed.body, body_id_from_scenario_text("main"));
            assert_eq!(shadow.seed.tree.bodies().len(), 2);
            assert_eq!(shadow.seed.tree.n_q(), 8);
            assert_eq!(shadow.seed.tree.n_qd(), 7);
            assert!(
                shadow.last_authoritative_root_state.is_none(),
                "default multi-gimbal shadows must remain non-authoritative"
            );
            assert!(
                (shadow.seed.sim_state.q()[7] - expected_q).abs() < 1.0e-12,
                "live handoff should update the gimbal coordinate for {engine:?}: {:?}",
                shadow.seed.sim_state.q()
            );
            assert!(
                (shadow.seed.sim_state.qd()[6] - expected_qd).abs() < 1.0e-12,
                "live handoff should finite-difference the gimbal rate for {engine:?}: {:?}",
                shadow.seed.sim_state.qd()
            );

            let derivative = shadow
                .last_derivative
                .as_ref()
                .expect("articulated gimbal derivative recorded");
            assert_eq!(derivative.qd_dot().len(), 7);
            let root_accel = (0..6)
                .map(|index| derivative.qd_dot()[index].abs())
                .fold(0.0_f64, f64::max);
            assert!(
                root_accel > 1.0e-6,
                "powered thrust should drive root acceleration for {engine:?}: {:?}",
                derivative.qd_dot()
            );
            assert!(
                derivative.qd_dot()[6].abs() > 1.0e-6,
                "offset thrust should drive gimbal acceleration for {engine:?}: {:?}",
                derivative.qd_dot()
            );
        }
    }

    #[test]
    fn articulated_gimbal_contact_mechanism_secondary_axis_routes_generalized_force_slot() {
        let toml = primary_multibody_two_axis_gimbal_shadow_scenario_toml();
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let joint = &scenario
            .document
            .multi_body
            .as_ref()
            .expect("multi_body present")
            .gimbal_joints[0];
        let primary = ArticulatedGimbalMechanism {
            id: "primary-stop".to_owned(),
            axis: MultiBodyGimbalMechanismAxisConfig::Primary,
            elements: Vec::new(),
        };
        let secondary = ArticulatedGimbalMechanism {
            id: "secondary-stop".to_owned(),
            axis: MultiBodyGimbalMechanismAxisConfig::Secondary,
            elements: Vec::new(),
        };

        assert_eq!(
            articulated_gimbal_mechanism_coordinate_indices(joint, &primary, 9, 8)
                .expect("primary mechanism routes"),
            (8, 7)
        );
        assert_eq!(
            articulated_gimbal_mechanism_coordinate_indices(joint, &secondary, 9, 8)
                .expect("secondary mechanism routes"),
            (7, 6)
        );
    }

    #[test]
    fn articulated_gimbal_shadow_derivative_uses_two_axis_live_engine_thrust() {
        let toml = primary_multibody_two_axis_gimbal_shadow_scenario_toml();
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step fires engine command");
        session
            .step_once(&scenario.document, None)
            .expect("second step mirrors two-axis articulated gimbal thrust");

        let engine = engine_id_from_scenario_text("main_engine");
        let powered_snapshot = session
            .engine_snapshots()
            .get(&engine)
            .copied()
            .expect("engine snapshot present");
        assert!(
            powered_snapshot.thrust_body.norm() > 1.0,
            "scenario command should produce powered thrust: {:?}",
            powered_snapshot.thrust_body
        );
        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        assert_eq!(shadow.seed.tree.bodies().len(), 3);
        assert_eq!(shadow.seed.tree.n_q(), 9);
        assert_eq!(shadow.seed.tree.n_qd(), 8);
        assert_eq!(
            shadow.seed.tree.bodies()[1].id,
            articulated_gimbal_frame_body_id(engine)
        );
        assert_eq!(
            shadow.seed.tree.bodies()[2].id,
            articulated_engine_body_id(engine)
        );
        assert!(
            (shadow.seed.sim_state.q()[7] + 0.05).abs() < 1.0e-12,
            "positive propulsion yaw should map to child-from-parent secondary angle: {:?}",
            shadow.seed.sim_state.q()
        );
        assert!(
            (shadow.seed.sim_state.q()[8] + 0.1).abs() < 1.0e-12,
            "positive propulsion pitch should map to child-from-parent primary angle: {:?}",
            shadow.seed.sim_state.q()
        );
        assert!(
            (shadow.seed.sim_state.qd()[6] + 0.5).abs() < 1.0e-12,
            "secondary gimbal rate should finite-difference live yaw handoff: {:?}",
            shadow.seed.sim_state.qd()
        );
        assert!(
            (shadow.seed.sim_state.qd()[7] + 1.0).abs() < 1.0e-12,
            "primary gimbal rate should finite-difference live pitch handoff: {:?}",
            shadow.seed.sim_state.qd()
        );
        let derivative = shadow
            .last_derivative
            .as_ref()
            .expect("articulated gimbal derivative recorded");
        assert_eq!(derivative.qd_dot().len(), 8);
        let root_accel = (0..6)
            .map(|index| derivative.qd_dot()[index].abs())
            .fold(0.0_f64, f64::max);
        assert!(
            root_accel > 1.0e-6,
            "engine thrust should drive root acceleration through articulated children: {:?}",
            derivative.qd_dot()
        );
        assert!(
            derivative.qd_dot()[6].abs() > 1.0e-6,
            "offset thrust should drive secondary gimbal acceleration: {:?}",
            derivative.qd_dot()
        );
        assert!(
            derivative.qd_dot()[7].abs() > 1.0e-6,
            "offset thrust should drive primary gimbal acceleration: {:?}",
            derivative.qd_dot()
        );
    }

    #[test]
    fn articulated_gimbal_shadow_rk4_forecast_advances_two_axis_tree() {
        let toml = primary_multibody_two_axis_gimbal_shadow_scenario_toml();
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step fires engine command");
        session
            .step_once(&scenario.document, None)
            .expect("second step records two-axis articulated forecast");

        let engine = engine_id_from_scenario_text("main_engine");
        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        assert!(
            shadow.last_authoritative_root_state.is_none(),
            "default articulated gimbal shadow must not record an authoritative handoff"
        );
        let forecast = shadow
            .last_rk4_forecast
            .as_ref()
            .expect("articulated gimbal RK4 forecast recorded");
        assert_eq!(forecast.tree.bodies().len(), 3);
        assert_eq!(forecast.tree.n_q(), shadow.seed.tree.n_q());
        assert_eq!(forecast.tree.n_qd(), shadow.seed.tree.n_qd());
        assert!(
            (forecast.sim_state.state().time.as_seconds()
                - shadow.seed.sim_state.state().time.as_seconds()
                - session.kernel_step_s)
                .abs()
                < 1.0e-12,
            "forecast should advance exactly one session step: seed={} forecast={} dt={}",
            shadow.seed.sim_state.state().time.as_seconds(),
            forecast.sim_state.state().time.as_seconds(),
            session.kernel_step_s
        );
        let max_joint_q_delta = forecast.sim_state.q()[7..]
            .iter()
            .zip(&shadow.seed.sim_state.q()[7..])
            .map(|(forecast_q, seed_q)| (forecast_q - seed_q).abs())
            .fold(0.0_f64, f64::max);
        let max_joint_qd_delta = forecast.sim_state.qd()[6..]
            .iter()
            .zip(&shadow.seed.sim_state.qd()[6..])
            .map(|(forecast_qd, seed_qd)| (forecast_qd - seed_qd).abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_joint_q_delta > 1.0e-6,
            "held live thrust should advance two-axis gimbal coordinates: seed={:?} forecast={:?}",
            shadow.seed.sim_state.q(),
            forecast.sim_state.q()
        );
        assert!(
            max_joint_qd_delta > 1.0e-6,
            "held live thrust should advance two-axis gimbal rates: seed={:?} forecast={:?}",
            shadow.seed.sim_state.qd(),
            forecast.sim_state.qd()
        );
        let seed_root = articulated_gimbal_root_rigid_state_from_seed(&shadow.seed)
            .expect("seed root projects");
        let root_forecast = shadow
            .last_root_rk4_forecast
            .as_ref()
            .expect("articulated gimbal root RK4 forecast recorded");
        assert!(
            (root_forecast.time.as_seconds() - seed_root.time.as_seconds() - session.kernel_step_s)
                .abs()
                < 1.0e-12,
            "projected root forecast should advance exactly one session step: seed={} forecast={} dt={}",
            seed_root.time.as_seconds(),
            root_forecast.time.as_seconds(),
            session.kernel_step_s
        );
        assert_eq!(
            root_forecast.mass_props.mass_kg().to_bits(),
            shadow.seed.parent_mass_props.mass_kg().to_bits()
        );
        let velocity_delta = (root_forecast.velocity.vector - seed_root.velocity.vector).norm();
        let angular_delta =
            (root_forecast.angular_velocity.vector - seed_root.angular_velocity.vector).norm();
        assert!(
            velocity_delta > 1.0e-6,
            "projected root forecast should carry powered translational response: seed={:?} forecast={:?}",
            seed_root.velocity.vector,
            root_forecast.velocity.vector
        );
        assert!(
            angular_delta > 1.0e-6,
            "projected root forecast should carry powered angular response: seed={:?} forecast={:?}",
            seed_root.angular_velocity.vector,
            root_forecast.angular_velocity.vector
        );
    }

    #[test]
    fn articulated_gimbal_root_authority_records_handoff() {
        let toml = primary_multibody_two_axis_gimbal_shadow_scenario_toml().replace(
            "[multi_body]\n",
            "[multi_body]\npropagation_authority = \"articulated_gimbal_root\"\n",
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step fires engine command");
        session
            .step_once(&scenario.document, None)
            .expect("second step applies articulated gimbal root authority");

        let engine = engine_id_from_scenario_text("main_engine");
        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        let root_forecast = shadow
            .last_root_rk4_forecast
            .as_ref()
            .expect("articulated gimbal root RK4 forecast recorded");
        let authoritative = shadow
            .last_authoritative_root_state
            .as_ref()
            .expect("articulated gimbal root handoff recorded");
        assert_rigid_forecast_matches_state(root_forecast, session.state(), 0.2);
        assert_rigid_forecast_matches_state(authoritative, session.state(), 0.2);
    }

    #[test]
    fn articulated_gimbal_root_authority_records_coupled_multi_gimbal_handoff() {
        let toml = primary_multibody_dual_gimbal_shadow_scenario_toml().replace(
            "[multi_body]\n",
            "[multi_body]\npropagation_authority = \"articulated_gimbal_root\"\n",
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        let authority = session
            .articulated_gimbal_root_authority
            .as_ref()
            .expect("coupled articulated gimbal authority shadow present");
        assert_eq!(authority.seed.tree.bodies().len(), 3);
        assert_eq!(authority.seed.tree.n_q(), 9);
        assert_eq!(authority.seed.tree.n_qd(), 8);
        assert_eq!(authority.seed.joints.len(), 2);

        session
            .step_once(&scenario.document, None)
            .expect("first step fires engine commands");
        session
            .step_once(&scenario.document, None)
            .expect("second step applies coupled multi-gimbal root authority");

        let authority = session
            .articulated_gimbal_root_authority
            .as_ref()
            .expect("coupled articulated gimbal authority shadow present");
        let root_forecast = authority
            .last_root_rk4_forecast
            .as_ref()
            .expect("coupled articulated gimbal root RK4 forecast recorded");
        let authoritative = *authority
            .last_authoritative_root_state
            .as_ref()
            .expect("coupled articulated gimbal root handoff recorded");
        assert_rigid_forecast_matches_state(root_forecast, session.state(), 0.2);
        assert_rigid_forecast_matches_state(&authoritative, session.state(), 0.2);
        assert!(
            session
                .articulated_gimbal_shadows
                .values()
                .all(|shadow| shadow.last_authoritative_root_state.is_none()),
            "per-engine shadows remain diagnostics when coupled multi-gimbal authority is active"
        );
        let forecast = authority
            .last_rk4_forecast
            .as_ref()
            .expect("coupled articulated forecast recorded");
        let carried_q = forecast.sim_state.q()[7..].to_vec();
        let carried_qd = forecast.sim_state.qd()[6..].to_vec();

        session
            .step_once(&scenario.document, None)
            .expect("third step carries coupled multi-gimbal internal state");

        let authority = session
            .articulated_gimbal_root_authority
            .as_ref()
            .expect("coupled articulated gimbal authority shadow present");
        assert_eq!(
            &authority.seed.sim_state.q()[7..],
            carried_q.as_slice(),
            "coupled authority seed should carry prior multi-gimbal coordinates"
        );
        assert_eq!(
            &authority.seed.sim_state.qd()[6..],
            carried_qd.as_slice(),
            "coupled authority seed should carry prior multi-gimbal rates"
        );
        assert_rigid_forecast_matches_state(
            &articulated_gimbal_root_authority_rigid_state_from_seed(&authority.seed)
                .expect("coupled carried root projects"),
            &authoritative,
            0.2,
        );
    }

    #[test]
    fn articulated_gimbal_root_authority_carries_internal_forecast_state() {
        let toml = primary_multibody_two_axis_gimbal_shadow_scenario_toml().replace(
            "[multi_body]\n",
            "[multi_body]\npropagation_authority = \"articulated_gimbal_root\"\n",
        );
        let toml = toml.replace("time_s = 0.05", "time_s = 0.25");
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step applies articulated gimbal root authority");

        let engine = engine_id_from_scenario_text("main_engine");
        let first_forecast = session
            .articulated_gimbal_shadows
            .get(&engine)
            .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
            .expect("first authoritative articulated forecast recorded")
            .clone();
        let carried_q = first_forecast.sim_state.q()[7..].to_vec();
        let carried_qd = first_forecast.sim_state.qd()[6..].to_vec();
        let carried_root =
            articulated_gimbal_root_rigid_state_from_seed(&first_forecast).expect("root projects");

        session
            .step_once(&scenario.document, None)
            .expect("second step carries articulated internal state");

        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        assert_eq!(
            &shadow.seed.sim_state.q()[7..],
            carried_q.as_slice(),
            "authoritative articulated seed should carry prior forecast joint coordinates"
        );
        assert_eq!(
            &shadow.seed.sim_state.qd()[6..],
            carried_qd.as_slice(),
            "authoritative articulated seed should carry prior forecast joint rates"
        );
        assert_rigid_forecast_matches_state(
            &articulated_gimbal_root_rigid_state_from_seed(&shadow.seed)
                .expect("carried root projects"),
            &carried_root,
            0.1,
        );
    }

    #[test]
    fn articulated_gimbal_root_authority_captures_live_gimbal_after_handoff() {
        let toml = primary_multibody_two_axis_gimbal_shadow_scenario_toml().replace(
            "[multi_body]\n",
            "[multi_body]\npropagation_authority = \"articulated_gimbal_root\"\n",
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first idle step applies root authority");

        let engine = engine_id_from_scenario_text("main_engine");
        let first_forecast_q = session
            .articulated_gimbal_shadows
            .get(&engine)
            .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
            .expect("first authoritative forecast recorded")
            .sim_state
            .q()[7..]
            .to_vec();

        session
            .step_once(&scenario.document, None)
            .expect("second step captures live gimbal after handoff");

        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        assert_ne!(
            &shadow.seed.sim_state.q()[7..],
            first_forecast_q.as_slice(),
            "new live gimbal deflection should refresh joint coordinates instead of carrying idle forecast"
        );
        assert!(
            (shadow.seed.sim_state.q()[7] + 0.05).abs() < 1.0e-12,
            "live yaw deflection should refresh secondary coordinate after handoff: {:?}",
            shadow.seed.sim_state.q()
        );
        assert!(
            (shadow.seed.sim_state.q()[8] + 0.1).abs() < 1.0e-12,
            "live pitch deflection should refresh primary coordinate after handoff: {:?}",
            shadow.seed.sim_state.q()
        );
        assert!(
            (shadow.seed.sim_state.qd()[6] + 0.5).abs() < 1.0e-12,
            "live yaw deflection should finite-difference from prior live target: {:?}",
            shadow.seed.sim_state.qd()
        );
        assert!(
            (shadow.seed.sim_state.qd()[7] + 1.0).abs() < 1.0e-12,
            "live pitch deflection should finite-difference from prior live target: {:?}",
            shadow.seed.sim_state.qd()
        );
    }

    #[test]
    fn articulated_gimbal_root_authority_uses_explicit_tvc_snapshot_without_thrust() {
        let toml = primary_multibody_two_axis_gimbal_shadow_scenario_toml()
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"articulated_gimbal_root\"\n",
            )
            .replace(
                "throttle_unit = 1.0, gimbal_pitch_rad = 0.1, gimbal_yaw_rad = 0.05, ignite = true",
                "throttle_unit = 0.0, gimbal_pitch_rad = 0.1, gimbal_yaw_rad = 0.05, ignite = false",
            );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first idle step applies root authority");
        session
            .step_once(&scenario.document, None)
            .expect("second step captures explicit TVC snapshot without thrust");

        let engine = engine_id_from_scenario_text("main_engine");
        let snapshot = session
            .engine_snapshots()
            .get(&engine)
            .copied()
            .expect("engine snapshot present");
        assert_eq!(snapshot.thrust_body, Vector3::zeros());
        assert_eq!(snapshot.gimbal_pitch_rad.to_bits(), 0.1_f64.to_bits());
        assert_eq!(snapshot.gimbal_yaw_rad.to_bits(), 0.05_f64.to_bits());

        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        assert!(
            (shadow.seed.sim_state.q()[7] + 0.05).abs() < 1.0e-12,
            "zero-thrust explicit yaw snapshot should refresh secondary coordinate: {:?}",
            shadow.seed.sim_state.q()
        );
        assert!(
            (shadow.seed.sim_state.q()[8] + 0.1).abs() < 1.0e-12,
            "zero-thrust explicit pitch snapshot should refresh primary coordinate: {:?}",
            shadow.seed.sim_state.q()
        );
        assert!(
            (shadow.seed.sim_state.qd()[6] + 0.5).abs() < 1.0e-12,
            "zero-thrust explicit yaw snapshot should refresh secondary rate: {:?}",
            shadow.seed.sim_state.qd()
        );
        assert!(
            (shadow.seed.sim_state.qd()[7] + 1.0).abs() < 1.0e-12,
            "zero-thrust explicit pitch snapshot should refresh primary rate: {:?}",
            shadow.seed.sim_state.qd()
        );
    }

    #[test]
    fn articulated_gimbal_root_authority_uses_explicit_tvc_snapshot_for_nonstandard_axes_without_thrust()
     {
        let toml = primary_multibody_two_axis_gimbal_shadow_scenario_toml()
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"articulated_gimbal_root\"\n",
            )
            .replace(
                "axis_body = [0.0, 1.0, 0.0]",
                "axis_body = [0.0, -1.0, 0.0]",
            )
            .replace(
                "secondary_axis_body = [1.0, 0.0, 0.0]",
                "secondary_axis_body = [-1.0, 0.0, 0.0]",
            )
            .replace(
                "throttle_unit = 1.0, gimbal_pitch_rad = 0.1, gimbal_yaw_rad = 0.05, ignite = true",
                "throttle_unit = 0.0, gimbal_pitch_rad = 0.1, gimbal_yaw_rad = 0.05, ignite = false",
            );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first idle step applies root authority");
        session
            .step_once(&scenario.document, None)
            .expect("second step captures explicit TVC snapshot without thrust");

        let engine = engine_id_from_scenario_text("main_engine");
        let snapshot = session
            .engine_snapshots()
            .get(&engine)
            .copied()
            .expect("engine snapshot present");
        assert_eq!(snapshot.thrust_body, Vector3::zeros());
        assert_eq!(snapshot.gimbal_pitch_rad.to_bits(), 0.1_f64.to_bits());
        assert_eq!(snapshot.gimbal_yaw_rad.to_bits(), 0.05_f64.to_bits());

        let shadow = session
            .articulated_gimbal_shadows
            .get(&engine)
            .expect("articulated gimbal shadow present");
        assert!(
            (shadow.seed.sim_state.q()[7] - 0.05).abs() < 1.0e-12,
            "zero-thrust explicit yaw snapshot should respect flipped secondary coordinate: {:?}",
            shadow.seed.sim_state.q()
        );
        assert!(
            (shadow.seed.sim_state.q()[8] - 0.1).abs() < 1.0e-12,
            "zero-thrust explicit pitch snapshot should respect flipped primary coordinate: {:?}",
            shadow.seed.sim_state.q()
        );
        assert!(
            (shadow.seed.sim_state.qd()[6] - 0.5).abs() < 1.0e-12,
            "zero-thrust explicit yaw snapshot should respect flipped secondary rate: {:?}",
            shadow.seed.sim_state.qd()
        );
        assert!(
            (shadow.seed.sim_state.qd()[7] - 1.0).abs() < 1.0e-12,
            "zero-thrust explicit pitch snapshot should respect flipped primary rate: {:?}",
            shadow.seed.sim_state.qd()
        );
    }

    #[test]
    fn articulated_gimbal_root_authority_advances_two_axis_internal_state_over_multi_step_ascent() {
        let toml = primary_multibody_two_axis_gimbal_shadow_scenario_toml().replace(
            "[multi_body]\n",
            "[multi_body]\npropagation_authority = \"articulated_gimbal_root\"\n",
        );
        let toml = toml.replace("stop_s = 0.3", "stop_s = 0.5");
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario parses");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");
        let engine = engine_id_from_scenario_text("main_engine");
        let mut previous_authoritative_root = None::<RigidBodyState>;
        let mut previous_forecast_q = None::<Vec<f64>>;
        let mut previous_forecast_qd = None::<Vec<f64>>;
        let mut max_q_delta = 0.0_f64;
        let mut max_qd_delta = 0.0_f64;
        let mut captured_live_after_handoff = false;

        for expected_time_s in [0.1, 0.2, 0.3, 0.4] {
            session
                .step_once(&scenario.document, None)
                .expect("authoritative two-axis articulated gimbal step advances");

            let shadow = session
                .articulated_gimbal_shadows
                .get(&engine)
                .expect("articulated gimbal shadow present");
            let root_forecast = shadow
                .last_root_rk4_forecast
                .as_ref()
                .expect("articulated gimbal root RK4 forecast recorded");
            let authoritative = *shadow
                .last_authoritative_root_state
                .as_ref()
                .expect("articulated gimbal root handoff recorded");

            assert_rigid_forecast_matches_state(root_forecast, session.state(), expected_time_s);
            assert_rigid_forecast_matches_state(&authoritative, session.state(), expected_time_s);

            if let (Some(previous_root), Some(previous_q), Some(previous_qd)) = (
                previous_authoritative_root.as_ref(),
                previous_forecast_q.as_ref(),
                previous_forecast_qd.as_ref(),
            ) {
                if (expected_time_s - 0.2).abs() < 1.0e-12 {
                    assert!(
                        (shadow.seed.sim_state.q()[7] + 0.05).abs() < 1.0e-12
                            && (shadow.seed.sim_state.q()[8] + 0.1).abs() < 1.0e-12,
                        "first live gimbal command after handoff should refresh carried coordinates: {:?}",
                        shadow.seed.sim_state.q()
                    );
                    assert!(
                        (shadow.seed.sim_state.qd()[6] + 0.5).abs() < 1.0e-12
                            && (shadow.seed.sim_state.qd()[7] + 1.0).abs() < 1.0e-12,
                        "first live gimbal command after handoff should refresh carried rates: {:?}",
                        shadow.seed.sim_state.qd()
                    );
                    captured_live_after_handoff = true;
                } else {
                    assert_rigid_forecast_matches_state(
                        &articulated_gimbal_root_rigid_state_from_seed(&shadow.seed)
                            .expect("carried seed root projects"),
                        previous_root,
                        expected_time_s - session.kernel_step_s,
                    );
                    assert_eq!(
                        &shadow.seed.sim_state.q()[7..],
                        previous_q.as_slice(),
                        "authoritative multi-step seed should carry prior two-axis coordinates"
                    );
                    assert_eq!(
                        &shadow.seed.sim_state.qd()[6..],
                        previous_qd.as_slice(),
                        "authoritative multi-step seed should carry prior two-axis rates"
                    );
                }
            }

            let forecast = shadow
                .last_rk4_forecast
                .as_ref()
                .expect("authoritative articulated gimbal RK4 forecast recorded");
            assert_eq!(forecast.tree.n_q(), 9);
            assert_eq!(forecast.tree.n_qd(), 8);
            let q = forecast.sim_state.q()[7..].to_vec();
            let qd = forecast.sim_state.qd()[6..].to_vec();
            assert!(
                q.iter().chain(&qd).all(|value| value.is_finite()),
                "two-axis authoritative forecast must remain finite: q={q:?} qd={qd:?}"
            );

            if let Some(previous_q) = previous_forecast_q.as_ref() {
                max_q_delta = max_q_delta.max(
                    q.iter()
                        .zip(previous_q)
                        .map(|(current, previous)| (current - previous).abs())
                        .fold(0.0_f64, f64::max),
                );
            }
            if let Some(previous_qd) = previous_forecast_qd.as_ref() {
                max_qd_delta = max_qd_delta.max(
                    qd.iter()
                        .zip(previous_qd)
                        .map(|(current, previous)| (current - previous).abs())
                        .fold(0.0_f64, f64::max),
                );
            }

            previous_authoritative_root = Some(authoritative);
            previous_forecast_q = Some(q);
            previous_forecast_qd = Some(qd);
        }

        assert!(
            max_q_delta > 1.0e-8,
            "two-axis authoritative coordinates should keep advancing over repeated steps"
        );
        assert!(
            max_qd_delta > 1.0e-8,
            "two-axis authoritative rates should keep advancing over repeated steps"
        );
        assert!(
            captured_live_after_handoff,
            "multi-step authority should exercise live gimbal capture after the initial handoff"
        );
    }

    #[test]
    fn root_free_flyer_multibody_derivative_bridge_matches_rigid_equations() {
        let state = RigidBodyState::new(
            SimTime::ZERO,
            Position3::<openbmp_core::Eci>::new(100.0, -200.0, 300.0),
            Velocity3::<openbmp_core::Eci>::zero(),
            Quaternion::<Body, openbmp_core::Eci>::from_unit_quaternion(
                nalgebra::UnitQuaternion::identity(),
            ),
            AngularVelocity3::<Body>::new(2.0, 4.0, 6.0),
            MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(2.0),
                Position3::<Body>::origin(),
                2.0,
            ),
        );
        let (tree, sim_state) =
            build_root_free_flyer_multibody_state(body_id_from_scenario_text("main"), &state)
                .expect("root seed builds");

        let derivative = root_free_flyer_multibody_derivative_from_rigid_loads(
            &tree,
            &sim_state,
            state.mass_props,
            Vector3::new(4.0, 8.0, 12.0),
            Vector3::new(14.0, 16.0, 18.0),
        )
        .expect("root derivative builds");
        let expected_rigid = openbmp_sim::RigidBodyDerivative::new(
            Vector3::zeros(),
            Vector3::new(7.0, 8.0, 9.0),
            nalgebra::Quaternion::new(0.0, 1.0, 2.0, 3.0),
            Vector3::new(2.0, 4.0, 6.0),
            0.0,
            Vector3::zeros(),
            nalgebra::Matrix3::zeros(),
        );

        assert_eq!(
            derivative.q_dot()[0].to_bits(),
            expected_rigid.quaternion_rate.w.to_bits()
        );
        assert_eq!(
            derivative.q_dot()[1].to_bits(),
            expected_rigid.quaternion_rate.i.to_bits()
        );
        assert_eq!(
            derivative.q_dot()[2].to_bits(),
            expected_rigid.quaternion_rate.j.to_bits()
        );
        assert_eq!(
            derivative.q_dot()[3].to_bits(),
            expected_rigid.quaternion_rate.k.to_bits()
        );
        for axis in 0..3 {
            assert_eq!(
                derivative.q_dot()[4 + axis].to_bits(),
                expected_rigid.velocity_m_s_eci[axis].to_bits()
            );
            assert_eq!(
                derivative.qd_dot()[axis].to_bits(),
                expected_rigid.angular_acceleration_rad_s2_body[axis].to_bits()
            );
            assert_eq!(
                derivative.qd_dot()[3 + axis].to_bits(),
                expected_rigid.acceleration_m_s2_eci[axis].to_bits()
            );
        }
    }

    #[test]
    fn root_free_flyer_multibody_derivative_bridge_rejects_nonfinite_loads() {
        let state = RigidBodyState::new(
            SimTime::ZERO,
            Position3::<openbmp_core::Eci>::origin(),
            Velocity3::<openbmp_core::Eci>::zero(),
            Quaternion::<Body, openbmp_core::Eci>::from_unit_quaternion(
                nalgebra::UnitQuaternion::identity(),
            ),
            AngularVelocity3::<Body>::zero(),
            MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(2.0),
                Position3::<Body>::origin(),
                2.0,
            ),
        );
        let (tree, sim_state) =
            build_root_free_flyer_multibody_state(body_id_from_scenario_text("main"), &state)
                .expect("root seed builds");

        let err = root_free_flyer_multibody_derivative_from_rigid_loads(
            &tree,
            &sim_state,
            state.mass_props,
            Vector3::new(f64::NAN, 0.0, 0.0),
            Vector3::zeros(),
        )
        .unwrap_err();

        match err {
            RunnerError::UnsupportedScenario { what } => {
                assert!(what.contains("root free-flyer multibody derivative"));
            }
            other => panic!("expected UnsupportedScenario, got {other:?}"),
        }
    }

    #[test]
    fn root_free_flyer_multibody_derivative_bridge_uses_runner_force_and_moment_models() {
        let toml = PRIMARY_MULTIBODY_BRIDGE_BASELINE_SCENARIO
            .replace("gravity_m_s2 = 0.0", "gravity_m_s2 = 2.0")
            .replace(
                "limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }\n",
                "limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }\ninitial_position = 0.5\n",
            )
            .replace(
                "[telemetry]\n",
                "[multi_body]\nprimary_body_id = \"main\"\n\n[[multi_body.attitude_target]]\nbody_id = \"main\"\nstart_time_s = 1.0\npitch_effector = \"main-pitch-torque\"\nkp = 1.0\nkd = 0.0\nmax_command = 1.0\ntarget = { kind = \"eci_vector\", vector_eci = [0.0, 0.0, 1.0] }\n\n[telemetry]\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let assembly = crate::assembly::synthesize_assembly(document).expect("assembly");
        let loaded = LoadedModels::default();
        let mass_resources = RigidMassResources::new(document, &assembly).expect("mass resources");
        let engine_snapshot = BTreeMap::new();
        let tank_snapshot = BTreeMap::new();
        let recovery_snapshot = BTreeMap::new();
        let initial_state = build_initial_state(
            document,
            &loaded,
            &mass_resources,
            &engine_snapshot,
            &tank_snapshot,
        )
        .expect("initial state");
        let seed = build_initial_root_free_flyer_multibody_seed(
            document,
            &loaded,
            &mass_resources,
            &engine_snapshot,
            &tank_snapshot,
            &initial_state,
        )
        .expect("seed builds")
        .expect("primary body seed present");
        let force_model = build_vehicle(
            document,
            &loaded,
            &assembly,
            &BTreeMap::new(),
            None,
            None,
            None,
        )
        .expect("force model");
        let moment_model = build_moment_model(document, &loaded, None).expect("moment model");
        let effector_rack = crate::effectors::EffectorRack::build(document).expect("effectors");
        let effector_actuals =
            build_effector_actual_snapshot_map(document, &[], &effector_rack.snapshot(), true)
                .expect("effector actuals");
        let environment = openbmp_sim::EnvironmentSample::default();

        let derivative = root_free_flyer_multibody_derivative_from_runner_models(
            &seed,
            &force_model,
            &moment_model,
            RootFreeFlyerLoadViews {
                phase_id: None,
                environment: &environment,
                effector_actuals: &effector_actuals,
                engine_snapshot: &engine_snapshot,
                tank_snapshot: &tank_snapshot,
                recovery_snapshot: &recovery_snapshot,
            },
        )
        .expect("runner model bridge derivative");

        assert_eq!(derivative.qd_dot()[1].to_bits(), 2.5_f64.to_bits());
        assert_eq!(derivative.qd_dot()[5].to_bits(), (-2.0_f64).to_bits());
    }

    #[test]
    fn primary_multibody_bridge_scenario_is_byte_identical_when_disabled() {
        let baseline =
            openbmp_scenario::Scenario::from_toml_str(PRIMARY_MULTIBODY_BRIDGE_BASELINE_SCENARIO)
                .expect("baseline scenario must parse");
        let multibody_toml = PRIMARY_MULTIBODY_BRIDGE_BASELINE_SCENARIO.replace(
            "[telemetry]\n",
            "[multi_body]\n\
             primary_body_id = \"main\"\n\
             \n\
             [[multi_body.attitude_target]]\n\
             body_id = \"main\"\n\
             start_time_s = 1.0\n\
             pitch_effector = \"main-pitch-torque\"\n\
             kp = 1.0\n\
             kd = 0.0\n\
             max_command = 1.0\n\
             target = { kind = \"eci_vector\", vector_eci = [0.0, 0.0, 1.0] }\n\
             \n\
             [telemetry]\n",
        );
        let multibody = openbmp_scenario::Scenario::from_toml_str(&multibody_toml)
            .expect("primary multibody bridge scenario must parse");

        let baseline_outcome = crate::run(&baseline).expect("baseline scenario must run");
        let multibody_outcome =
            crate::run(&multibody).expect("primary multibody bridge scenario must run");

        assert_eq!(
            baseline_outcome.table.schema().channels().len(),
            multibody_outcome.table.schema().channels().len()
        );
        assert_eq!(
            telemetry_csv_bytes(&multibody_outcome),
            telemetry_csv_bytes(&baseline_outcome),
            "disabled primary multi_body setup must not perturb rigid-kernel telemetry"
        );
    }

    #[test]
    fn primary_multibody_shadow_mirrors_pre_step_runner_loads() {
        let toml = PRIMARY_MULTIBODY_BRIDGE_BASELINE_SCENARIO
            .replace("gravity_m_s2 = 0.0", "gravity_m_s2 = 2.0")
            .replace(
                "limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }\n",
                "limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }\ninitial_position = 0.5\n",
            )
            .replace(
                "[telemetry]\n",
                "[multi_body]\nprimary_body_id = \"main\"\n\n[[multi_body.attitude_target]]\nbody_id = \"main\"\nstart_time_s = 1.0\npitch_effector = \"main-pitch-torque\"\nkp = 1.0\nkd = 0.0\nmax_command = 1.0\ntarget = { kind = \"eci_vector\", vector_eci = [0.0, 0.0, 1.0] }\n\n[telemetry]\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        assert!(
            session
                .primary_multibody_shadow
                .as_ref()
                .expect("shadow present")
                .last_derivative
                .is_none()
        );

        session
            .step_once(&scenario.document, None)
            .expect("first step mirrors primary root");
        let shadow = session
            .primary_multibody_shadow
            .as_ref()
            .expect("shadow present after first step");
        let derivative = shadow
            .last_derivative
            .as_ref()
            .expect("per-step derivative recorded");
        assert_eq!(
            shadow.seed.rigid_state.time.as_seconds().to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(derivative.qd_dot()[1].to_bits(), 2.5_f64.to_bits());
        assert_eq!(derivative.qd_dot()[5].to_bits(), (-2.0_f64).to_bits());

        session
            .step_once(&scenario.document, None)
            .expect("second step mirrors primary root");
        let shadow = session
            .primary_multibody_shadow
            .as_ref()
            .expect("shadow present after second step");
        assert_eq!(
            shadow.seed.rigid_state.time.as_seconds().to_bits(),
            0.1_f64.to_bits()
        );
        assert!(shadow.last_derivative.is_some());
    }

    #[test]
    fn primary_multibody_shadow_rk4_forecast_matches_constant_mass_no_rotation_rigid_step() {
        let toml = PRIMARY_MULTIBODY_BRIDGE_BASELINE_SCENARIO
            .replace("gravity_m_s2 = 0.0", "gravity_m_s2 = 2.0")
            .replace(
                "[telemetry]\n",
                "[multi_body]\nprimary_body_id = \"main\"\n\n[[multi_body.attitude_target]]\nbody_id = \"main\"\nstart_time_s = 1.0\npitch_effector = \"main-pitch-torque\"\nkp = 1.0\nkd = 0.0\nmax_command = 1.0\ntarget = { kind = \"eci_vector\", vector_eci = [0.0, 0.0, 1.0] }\n\n[telemetry]\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step forecasts primary root");

        let forecast = session
            .primary_multibody_shadow
            .as_ref()
            .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
            .expect("primary RK4 forecast recorded");
        assert!(
            session
                .primary_multibody_shadow
                .as_ref()
                .and_then(|shadow| shadow.last_authoritative_state.as_ref())
                .is_none(),
            "default rigid-kernel authority must not record a multibody handoff"
        );
        let predicted = &forecast.rigid_state;
        let actual = session.state();
        assert_eq!(predicted.time.as_seconds().to_bits(), 0.1_f64.to_bits());
        assert!(
            (predicted.position.vector - actual.position.vector).norm() < 1.0e-12,
            "position predicted={:?} actual={:?} diff={:?}",
            predicted.position.vector,
            actual.position.vector,
            predicted.position.vector - actual.position.vector
        );
        assert!(
            (predicted.velocity.vector - actual.velocity.vector).norm() < 1.0e-12,
            "velocity predicted={:?} actual={:?} diff={:?}",
            predicted.velocity.vector,
            actual.velocity.vector,
            predicted.velocity.vector - actual.velocity.vector
        );
        assert!(
            (predicted.angular_velocity.vector - actual.angular_velocity.vector).norm() < 1.0e-12,
            "omega predicted={:?} actual={:?} diff={:?}",
            predicted.angular_velocity.vector,
            actual.angular_velocity.vector,
            predicted.angular_velocity.vector - actual.angular_velocity.vector
        );
        let predicted_q = predicted.orientation.q.into_inner();
        let actual_q = actual.orientation.q.into_inner();
        assert!(
            (predicted_q.coords - actual_q.coords).norm() < 1.0e-12,
            "q predicted={:?} actual={:?} diff={:?}",
            predicted_q.coords,
            actual_q.coords,
            predicted_q.coords - actual_q.coords
        );
    }

    fn primary_authority_crush_scenario() -> Scenario {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../scenarios/landing-gear-four-leg-drop/scenario.toml");
        let mut scenario = Scenario::from_file(path).expect("landing gear scenario");
        scenario.document.time.dt_s = 0.2;
        scenario.document.time.stop_s = 0.2;
        scenario.document.vehicle.initial_position_eci_m = [0.0, 0.0, 1.0];
        scenario
            .document
            .vehicle
            .initial_angular_velocity_body_rad_s = Some([0.5, 0.25, 0.1]);
        let mut ballast = scenario.document.vehicle.assembly.bodies[0].clone();
        ballast.id = "ballast".to_owned();
        scenario.document.vehicle.assembly.bodies.push(ballast);
        scenario.document.forces = Some(openbmp_scenario::ForcesConfig {
            models: vec!["landing_gear".to_owned()],
            phase_override: Vec::new(),
        });
        let gear = scenario
            .document
            .vehicle
            .landing_gear
            .as_mut()
            .expect("gear");
        gear.data_file = None;
        gear.data_file_sha256 = None;
        for leg in &mut gear.legs {
            leg.footpad = openbmp_scenario::ContactGeometryConfig::Point;
            leg.footpad_radius_m = None;
            leg.oleo = None;
            leg.crush = Some(openbmp_scenario::LandingGearCrushConfig {
                f_crush_n: 1_000.0,
                stroke_max_m: 0.5,
                k_elastic_n_m: 10_000.0,
            });
        }
        scenario.document.multi_body = Some(
            toml::from_str(
                "primary_body_id = 'core'\npropagation_authority = 'primary_root_free_flyer'\n",
            )
            .expect("primary authority"),
        );
        scenario
    }

    #[test]
    fn primary_authority_commits_landing_gear_history_from_its_actual_endpoint() {
        let scenario = primary_authority_crush_scenario();
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session = RigidBodySession::prepare(&scenario, &resolved_files).expect("session");
        session
            .step_once(&scenario.document, None)
            .expect("authority step");

        // The primary-body authority advances the selected body's properties,
        // while the discarded kernel initially advances the aggregate stack.
        // Accepted history and diagnostics must follow the authoritative pose.
        let state = session.state();
        let samples = session
            .landing_gear_runtime
            .as_ref()
            .expect("gear runtime")
            .take_samples()
            .expect("accepted samples");
        let legs = &scenario
            .document
            .vehicle
            .landing_gear
            .as_ref()
            .expect("gear")
            .legs;
        assert!(samples.iter().any(|sample| sample.crushed_m > 0.0));
        for (sample, leg) in samples.iter().zip(legs) {
            let foot_body = Vector3::from(leg.attach_body_m)
                + Vector3::from(leg.strut_axis_body) * leg.free_length_m;
            let offset_eci = state.orientation.q * foot_body;
            let stroke_m = (-(state.position.vector + offset_eci).z).clamp(0.0, 0.5);
            let expected_crushed_m = (stroke_m - 0.1).max(0.0);
            let expected_force_n = (10_000.0 * (stroke_m - expected_crushed_m)).min(1_000.0);
            let omega_eci = state.orientation.q * state.angular_velocity.vector;
            let foot_velocity = state.velocity.vector + omega_eci.cross(&offset_eci);
            assert!(
                (sample.crushed_m - expected_crushed_m).abs() < 1e-10,
                "discarded kernel path changed permanent stroke: {sample:?}; actual stroke={stroke_m}"
            );
            assert!((sample.stroke_m - stroke_m).abs() < 1e-10);
            assert!((sample.force_n - expected_force_n).abs() < 1e-9);
            assert!((sample.normal_velocity_m_s - foot_velocity.z).abs() < 1e-10);
        }
    }

    #[test]
    fn phase_disabled_landing_gear_preserves_uncrushed_history() {
        let mut scenario = primary_authority_crush_scenario();
        scenario.document.multi_body = None;
        scenario.document.vehicle.assembly.bodies.truncate(1);
        scenario.document.time.dt_s = 0.01;
        scenario.document.time.stop_s = 0.02;
        scenario.document.environment.gravity_m_s2 = Some(0.0);
        scenario.document.vehicle.initial_position_eci_m = [0.0, 0.0, 0.8];
        scenario.document.vehicle.initial_velocity_eci_m_s = [0.0, 0.0, -0.1];
        scenario
            .document
            .vehicle
            .initial_angular_velocity_body_rad_s = Some([0.0; 3]);
        scenario.document.forces = Some(toml::from_str(
            "models = ['gravity']\n[[phase_override]]\nphase = 'enabled'\nmodels = ['gravity', 'landing_gear']\n"
        ).expect("phase force selection"));
        scenario.document.mission = Some(
            toml::from_str(
                r#"
initial_phase = "disabled"
[[phases]]
id = "disabled"
label = "gear disabled"
[[phases]]
id = "enabled"
label = "gear enabled"
[[events]]
id = "enable_gear"
trigger = { kind = "at_time", time_s = 0.02 }
action = { kind = "emit_telemetry_marker", tag = "enable_gear" }
once = true
[[transitions]]
from = "disabled"
to = "enabled"
event = "enable_gear"
"#,
            )
            .expect("phase transition"),
        );
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session = RigidBodySession::prepare(&scenario, &resolved_files).expect("session");
        let runtime = session
            .landing_gear_runtime
            .as_ref()
            .expect("gear runtime")
            .clone();
        for sample in runtime.take_samples().expect("initial samples") {
            // Disabled gear cannot acquire initial plastic preload.
            approx::assert_abs_diff_eq!(sample.crushed_m, 0.0, epsilon = 0.0);
            approx::assert_abs_diff_eq!(sample.force_n, 0.0, epsilon = 0.0);
        }
        session
            .step_once(&scenario.document, None)
            .expect("disabled step");
        for sample in runtime.take_samples().expect("inactive samples") {
            approx::assert_abs_diff_eq!(sample.crushed_m, 0.0, epsilon = 0.0);
            approx::assert_abs_diff_eq!(sample.force_n, 0.0, epsilon = 0.0);
        }
        session
            .step_once(&scenario.document, None)
            .expect("activation endpoint");
        for sample in runtime.take_samples().expect("active endpoint samples") {
            // The interval before activation was inactive.
            approx::assert_abs_diff_eq!(sample.crushed_m, 0.0, epsilon = 0.0);
            assert!((sample.force_n - 1_000.0).abs() < 1e-9);
        }
        let energy = runtime.energy_integrals().expect("accepted energy");
        approx::assert_abs_diff_eq!(energy.contact_work_on_vehicle_j, 0.0, epsilon = 0.0);
        approx::assert_abs_diff_eq!(energy.dissipated_energy_j, 0.0, epsilon = 0.0);
    }

    #[test]
    fn primary_multibody_root_free_flyer_authority_records_handoff() {
        let toml = PRIMARY_MULTIBODY_BRIDGE_BASELINE_SCENARIO
            .replace("gravity_m_s2 = 0.0", "gravity_m_s2 = 2.0")
            .replace(
                "[telemetry]\n",
                "[multi_body]\nprimary_body_id = \"main\"\npropagation_authority = \"primary_root_free_flyer\"\n\n[[multi_body.attitude_target]]\nbody_id = \"main\"\nstart_time_s = 1.0\npitch_effector = \"main-pitch-torque\"\nkp = 1.0\nkd = 0.0\nmax_command = 1.0\ntarget = { kind = \"eci_vector\", vector_eci = [0.0, 0.0, 1.0] }\n\n[telemetry]\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step applies primary root authority");

        let shadow = session
            .primary_multibody_shadow
            .as_ref()
            .expect("primary multibody shadow exists");
        let forecast = shadow
            .last_rk4_forecast
            .as_ref()
            .expect("primary RK4 forecast recorded");
        let authoritative = shadow
            .last_authoritative_state
            .as_ref()
            .expect("primary root multibody handoff recorded");
        assert_rigid_forecast_matches_state(&forecast.rigid_state, session.state(), 0.1);
        assert_rigid_forecast_matches_state(authoritative, session.state(), 0.1);
    }

    #[test]
    fn primary_multibody_shadow_rk4_forecast_matches_constant_mass_direct_torque_rigid_step() {
        let toml = PRIMARY_MULTIBODY_BRIDGE_BASELINE_SCENARIO
            .replace("gravity_m_s2 = 0.0", "gravity_m_s2 = 2.0")
            .replace(
                "limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }\n",
                "limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }\ninitial_position = 0.5\n",
            )
            .replace(
                "[telemetry]\n",
                "[multi_body]\nprimary_body_id = \"main\"\n\n[[multi_body.attitude_target]]\nbody_id = \"main\"\nstart_time_s = 1.0\npitch_effector = \"main-pitch-torque\"\nkp = 1.0\nkd = 0.0\nmax_command = 1.0\ntarget = { kind = \"eci_vector\", vector_eci = [0.0, 0.0, 1.0] }\n\n[telemetry]\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step forecasts primary root");

        let forecast = session
            .primary_multibody_shadow
            .as_ref()
            .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
            .expect("primary RK4 forecast recorded");
        let predicted = &forecast.rigid_state;
        let actual = session.state();
        assert_eq!(predicted.time.as_seconds().to_bits(), 0.1_f64.to_bits());
        assert!(
            (predicted.position.vector - actual.position.vector).norm() < 1.0e-12,
            "position predicted={:?} actual={:?} diff={:?}",
            predicted.position.vector,
            actual.position.vector,
            predicted.position.vector - actual.position.vector
        );
        assert!(
            (predicted.velocity.vector - actual.velocity.vector).norm() < 1.0e-12,
            "velocity predicted={:?} actual={:?} diff={:?}",
            predicted.velocity.vector,
            actual.velocity.vector,
            predicted.velocity.vector - actual.velocity.vector
        );
        assert!(
            (predicted.angular_velocity.vector - actual.angular_velocity.vector).norm() < 1.0e-12,
            "omega predicted={:?} actual={:?} diff={:?}",
            predicted.angular_velocity.vector,
            actual.angular_velocity.vector,
            predicted.angular_velocity.vector - actual.angular_velocity.vector
        );
        let predicted_q = predicted.orientation.q.into_inner();
        let actual_q = actual.orientation.q.into_inner();
        assert!(
            (predicted_q.coords - actual_q.coords).norm() < 1.0e-12,
            "q predicted={:?} actual={:?} diff={:?}",
            predicted_q.coords,
            actual_q.coords,
            predicted_q.coords - actual_q.coords
        );
    }

    #[test]
    fn primary_multibody_shadow_rk4_forecast_matches_variable_mass_solid_motor_step() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(
            PRIMARY_MULTIBODY_SOLID_MOTOR_SHADOW_SCENARIO,
        )
        .expect("solid-motor primary multibody scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");
        let initial_mass_kg = session.state().mass_props.mass_kg();

        session
            .step_once(&scenario.document, None)
            .expect("first step forecasts variable-mass primary root");

        let shadow = session
            .primary_multibody_shadow
            .as_ref()
            .expect("primary shadow recorded");
        let forecast = shadow
            .last_rk4_forecast
            .as_ref()
            .expect("primary RK4 forecast recorded");
        let predicted = &forecast.rigid_state;
        let actual = session.state();
        assert_eq!(predicted.time.as_seconds().to_bits(), 0.1_f64.to_bits());
        assert!(
            predicted.mass_props.mass_kg() < initial_mass_kg,
            "solid motor forecast should drain mass: predicted={} initial={}",
            predicted.mass_props.mass_kg(),
            initial_mass_kg
        );
        assert!(
            (predicted.mass_props.mass_kg() - actual.mass_props.mass_kg()).abs() < 1.0e-12,
            "mass predicted={} actual={}",
            predicted.mass_props.mass_kg(),
            actual.mass_props.mass_kg()
        );
        assert!(
            (predicted.position.vector - actual.position.vector).norm() < 1.0e-12,
            "position predicted={:?} actual={:?} diff={:?}",
            predicted.position.vector,
            actual.position.vector,
            predicted.position.vector - actual.position.vector
        );
        assert!(
            (predicted.velocity.vector - actual.velocity.vector).norm() < 1.0e-12,
            "velocity predicted={:?} actual={:?} diff={:?}",
            predicted.velocity.vector,
            actual.velocity.vector,
            predicted.velocity.vector - actual.velocity.vector
        );
        assert!(
            (predicted.angular_velocity.vector - actual.angular_velocity.vector).norm() < 1.0e-12,
            "omega predicted={:?} actual={:?} diff={:?}",
            predicted.angular_velocity.vector,
            actual.angular_velocity.vector,
            predicted.angular_velocity.vector - actual.angular_velocity.vector
        );
        let predicted_q = predicted.orientation.q.into_inner();
        let actual_q = actual.orientation.q.into_inner();
        assert!(
            (predicted_q.coords - actual_q.coords).norm() < 1.0e-12,
            "q predicted={:?} actual={:?} diff={:?}",
            predicted_q.coords,
            actual_q.coords,
            predicted_q.coords - actual_q.coords
        );
    }

    #[test]
    fn primary_multibody_shadow_mirrors_gimballed_engine_loads() {
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO)
                .expect("gimballed primary multibody scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step fires engine command");
        session
            .step_once(&scenario.document, None)
            .expect("second step mirrors gimballed engine loads");

        let shadow = session
            .primary_multibody_shadow
            .as_ref()
            .expect("shadow present after gimballed step");
        let derivative = shadow
            .last_derivative
            .as_ref()
            .expect("gimballed pre-step derivative recorded");
        assert_eq!(
            shadow.seed.rigid_state.time.as_seconds().to_bits(),
            0.1_f64.to_bits()
        );
        assert!(
            derivative.qd_dot()[0] > 0.0,
            "off-axis axial thrust should create positive roll/pitch-slot acceleration: {:?}",
            derivative.qd_dot()
        );
        assert!(
            derivative.qd_dot()[2] < 0.0,
            "pitch gimbal lateral thrust should create signed yaw-slot acceleration: {:?}",
            derivative.qd_dot()
        );
        assert!(
            derivative.qd_dot()[3] > 0.0,
            "pitch gimbal should create positive lateral acceleration: {:?}",
            derivative.qd_dot()
        );
        assert!(
            derivative.qd_dot()[5] > 0.0,
            "engine thrust should create positive axial acceleration: {:?}",
            derivative.qd_dot()
        );
    }

    #[test]
    fn primary_multibody_shadow_rk4_forecast_matches_gimballed_liquid_engine_step() {
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO)
                .expect("gimballed primary multibody scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step fires engine command");
        session
            .step_once(&scenario.document, None)
            .expect("second step forecasts gimballed engine load");

        let forecast = session
            .primary_multibody_shadow
            .as_ref()
            .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
            .expect("gimballed primary RK4 forecast recorded");
        let predicted = &forecast.rigid_state;
        let actual = session.state();
        assert_rigid_forecast_matches_state(predicted, actual, 0.2);
    }

    #[test]
    fn primary_multibody_shadow_rk4_forecast_matches_multi_step_gimballed_ascent() {
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO)
                .expect("gimballed primary multibody scenario must parse");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("first step fires engine command");

        for expected_time_s in [0.2, 0.3] {
            session
                .step_once(&scenario.document, None)
                .expect("powered gimballed step forecasts primary root");

            let forecast = session
                .primary_multibody_shadow
                .as_ref()
                .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
                .expect("gimballed primary RK4 forecast recorded");
            assert_rigid_forecast_matches_state(
                &forecast.rigid_state,
                session.state(),
                expected_time_s,
            );
        }
    }

    #[test]
    fn primary_multibody_shadow_rk4_forecast_matches_phalcon9_tvc_probe_ascent() {
        let scenario = phalcon9_tvc_probe_multibody_scenario(true);
        let (step_count, _) = assert_phalcon9_tvc_multibody_forecasts(&scenario, Some(150));
        assert_eq!(step_count, 150);
    }

    #[test]
    fn primary_multibody_shadow_rk4_forecast_matches_full_duration_phalcon9_tvc_probe() {
        let scenario = phalcon9_tvc_probe_multibody_scenario(false);
        let (step_count, _) = assert_phalcon9_tvc_multibody_forecasts(&scenario, None);
        assert_eq!(step_count, 4000);
    }

    #[test]
    fn separated_multibody_shadow_mirrors_initial_lane_loads() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(SEPARATED_DIRECT_TORQUE_SCENARIO)
            .expect("separated direct-torque scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");
        assert!(
            session.separated_multibody_shadows.is_empty(),
            "separated shadows should be populated by the first pre-step mirror"
        );

        session
            .step_once(&scenario.document, None)
            .expect("session step succeeds");
        let booster = body_id_from_scenario_text("booster");
        let shadow = session
            .separated_multibody_shadows
            .get(&booster)
            .expect("booster separated-lane shadow is recorded");
        assert_eq!(shadow.seed.body, booster);
        assert_eq!(
            shadow.seed.rigid_state.time.as_seconds().to_bits(),
            0.0_f64.to_bits()
        );
        let derivative = shadow
            .last_derivative
            .as_ref()
            .expect("separated shadow derivative is recorded");
        let qd_dot = derivative.qd_dot();
        assert!(
            qd_dot[1] > 0.0,
            "booster-owned pitch torque should reach the separated multibody shadow: {qd_dot:?}"
        );
        assert!(
            qd_dot[3].abs() < 1.0e-12 && qd_dot[4].abs() < 1.0e-12 && qd_dot[5].abs() < 1.0e-12,
            "direct torque scenario should not add separated translational acceleration: {qd_dot:?}"
        );
    }

    #[test]
    fn separated_multibody_shadow_rk4_forecast_matches_initial_lane_rigid_step() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(SEPARATED_DIRECT_TORQUE_SCENARIO)
            .expect("separated direct-torque scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("session step succeeds");

        let booster = body_id_from_scenario_text("booster");
        let forecast = session
            .separated_multibody_shadows
            .get(&booster)
            .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
            .expect("booster separated-lane RK4 forecast is recorded");
        assert!(
            session
                .separated_multibody_shadows
                .get(&booster)
                .and_then(|shadow| shadow.last_authoritative_state.as_ref())
                .is_none(),
            "default rigid separated-lane authority must not record a multibody handoff"
        );
        let predicted = &forecast.rigid_state;
        let actual = separated_lane_state(&session, booster);
        assert_rigid_forecast_matches_state(predicted, actual, 0.1);
    }

    #[test]
    fn separated_multibody_root_free_flyer_authority_records_initial_lane_handoff() {
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO.replace(
            "[multi_body]\n",
            "[multi_body]\npropagation_authority = \"separated_root_free_flyer\"\n",
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml)
            .expect("separated authority scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("session step applies separated authority");

        let booster = body_id_from_scenario_text("booster");
        let shadow = session
            .separated_multibody_shadows
            .get(&booster)
            .expect("booster separated-lane shadow is recorded");
        let forecast = shadow
            .last_rk4_forecast
            .as_ref()
            .expect("booster separated-lane forecast is recorded");
        let authoritative = shadow
            .last_authoritative_state
            .as_ref()
            .expect("booster separated-lane handoff is recorded");
        assert_rigid_forecast_matches_state(
            &forecast.rigid_state,
            separated_lane_state(&session, booster),
            0.1,
        );
        assert_rigid_forecast_matches_state(
            authoritative,
            separated_lane_state(&session, booster),
            0.1,
        );
    }

    #[test]
    fn separated_multibody_shadow_refreshes_after_jettison_event() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO.replace(initial_lane, jettison);
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("jettison step succeeds");
        let booster = body_id_from_scenario_text("booster");
        let shadow = session
            .separated_multibody_shadows
            .get(&booster)
            .expect("jettisoned booster shadow is recorded immediately");
        assert_eq!(
            shadow.seed.rigid_state.time.as_seconds().to_bits(),
            0.1_f64.to_bits()
        );
        let derivative = shadow
            .last_derivative
            .as_ref()
            .expect("jettisoned separated shadow derivative is recorded");
        assert!(
            derivative.qd_dot()[1] > 0.0,
            "booster-owned pitch torque should reach jettisoned shadow: {:?}",
            derivative.qd_dot()
        );
    }

    #[test]
    fn welded_release_jettison_authority_installs_release_states() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("welded-release jettison step succeeds");

        let booster = body_id_from_scenario_text("booster");
        let lane = separated_lane_state(&session, booster);
        assert_eq!(lane.time.as_seconds().to_bits(), 0.1_f64.to_bits());
        assert_eq!(
            lane.mass_props.mass_kg().to_bits(),
            1.0_f64.to_bits(),
            "departing lane should carry lower-body mass"
        );
        assert_eq!(
            session.state().mass_props.mass_kg().to_bits(),
            3.0_f64.to_bits(),
            "continuing stack should carry upper-body mass"
        );
        assert!(
            (lane.position.vector - session.state().position.vector).norm() < 1.0e-12,
            "identity welded release should preserve same release position"
        );
        assert!(
            (lane.velocity.vector - session.state().velocity.vector).norm() < 1.0e-12,
            "identity welded release should preserve same release velocity"
        );
        assert!(
            (lane.angular_velocity.vector - session.state().angular_velocity.vector).norm()
                < 1.0e-12,
            "identity welded release should preserve same angular velocity"
        );
    }

    #[test]
    fn welded_release_jettison_authority_applies_weld_transform() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("welded-release transform jettison step succeeds");

        let booster = body_id_from_scenario_text("booster");
        let lane = separated_lane_state(&session, booster);
        let expected_offset = session.state().orientation.q * Vector3::new(0.0, 2.0, 0.0);
        assert!(
            (lane.position.vector - session.state().position.vector - expected_offset).norm()
                < 1.0e-12,
            "departing welded transform should offset lane position: lane={:?} stack={:?} expected_offset={expected_offset:?}",
            lane.position.vector,
            session.state().position.vector
        );
        let q = (session.state().orientation.q.inverse() * lane.orientation.q).into_inner();
        assert!((q.i - 0.0).abs() < 1.0e-12);
        assert!((q.j - 0.0).abs() < 1.0e-12);
        assert!((q.k + std::f64::consts::FRAC_1_SQRT_2).abs() < 1.0e-12);
        assert!((q.w - std::f64::consts::FRAC_1_SQRT_2).abs() < 1.0e-12);
    }

    #[test]
    fn welded_release_jettison_authority_releases_dry_subtree() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let booster_and_payload_bodies = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(booster_body, booster_and_payload_bodies)
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("welded-release subtree jettison step succeeds");

        let booster = body_id_from_scenario_text("booster");
        let lane = separated_lane_state(&session, booster);
        assert_eq!(
            lane.mass_props.mass_kg().to_bits(),
            3.0_f64.to_bits(),
            "departing lane should preserve lower-root plus dry payload mass"
        );
        assert_eq!(
            session.state().mass_props.mass_kg().to_bits(),
            3.0_f64.to_bits(),
            "continuing stack should not retain dry payload mass"
        );
        let expected_offset = session.state().orientation.q * Vector3::new(0.0, 4.0, 0.0);
        assert!(
            (lane.position.vector - session.state().position.vector - expected_offset).norm()
                < 1.0e-12,
            "departing subtree COM should offset lane position: lane={:?} stack={:?} expected_offset={expected_offset:?}",
            lane.position.vector,
            session.state().position.vector
        );
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_direct_torque_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let booster_and_payload_bodies = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(booster_body, booster_and_payload_bodies)
            .replace(
                "mounted_to = \"booster\"\nkind = { kind = \"direct_torque\"",
                "mounted_to = \"payload\"\nkind = { kind = \"direct_torque\"",
            )
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let outcome = crate::run(&scenario)
            .expect("welded-release descendant direct torque scenario must run");

        let primary_roll_rate = f64_column(&outcome, "angular_velocity.x_rad_s");
        let booster_roll_rate = f64_column(&outcome, "body.booster.angular_velocity.x_rad_s");
        let primary_final = primary_roll_rate.last().copied().unwrap_or_default();
        let booster_final = booster_roll_rate.last().copied().unwrap_or_default();
        assert!(
            booster_final > primary_final + 0.1,
            "payload-owned pitch torque should rotate into departing-root roll torque after release: primary={primary_roll_rate:?} booster={booster_roll_rate:?}",
        );

        let effector_actual = f64_column(&outcome, "effector.booster-pitch-torque.actual");
        assert!(
            effector_actual.iter().any(|value| *value > 0.0),
            "descendant direct-torque effector command should remain observable: {effector_actual:?}"
        );
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_scalar_effectors_to_aero_axes() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let direct_torque_effector = r#"
[[vehicle.assembly.effectors]]
id = "booster-pitch-torque"
mounted_to = "booster"
kind = { kind = "direct_torque", axis = "pitch", effectiveness_n_m_per_rad = 10.0 }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
command_schedule = { kind = "constant", value = 1.0 }
"#;
        let booster_payload_and_effectors = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.effectors]]
id = "delta_e"
mounted_to = "payload"
kind = { kind = "linear_actuator", tau_s = 0.0 }
limits = { min = -20.0, max = 20.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
initial_position = 5.0
unit = "deg"

[[vehicle.assembly.effectors]]
id = "delta_r"
mounted_to = "payload"
kind = { kind = "second_order_servo", natural_frequency_rad_s = 20.0, damping_ratio = 0.8, max_accel_per_s2 = 1000.0, backlash_half_width = 0.0 }
limits = { min = -20.0, max = 20.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
initial_position = -3.0
unit = "deg"
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(booster_body, booster_payload_and_effectors)
            .replace(direct_torque_effector, "")
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let payload = body_id_from_scenario_text("payload");

        assert_eq!(
            unsupported_welded_descendant_runtime_resource(document, payload),
            None
        );
        require_supported_shape(document)
            .expect("welded descendant scalar effectors are supported");
        assert!(
            document
                .vehicle
                .assembly
                .effectors
                .iter()
                .all(|effector| welded_release_scalar_effector_supported(document, effector))
        );

        let deck = openbmp_aero::AeroDeck::new_n_d(
            vec![
                "mach".to_owned(),
                "alpha".to_owned(),
                "beta".to_owned(),
                "delta_e_deg".to_owned(),
                "delta_r_deg".to_owned(),
            ],
            vec![
                vec![0.0, 1.0],
                vec![0.0, 1.0],
                vec![0.0],
                vec![-20.0, 0.0, 20.0],
                vec![-20.0, 0.0, 20.0],
            ],
            vec![0.0; 36],
            vec![0.1; 36],
            vec![0.0; 36],
            1.0,
            1.0,
        )
        .expect("schema-2 deck builds");
        let bindings =
            crate::aero_effector_match::assert_axes_match_effectors(Some(&deck), document)
                .expect("descendant scalar effectors bind to aero axes");
        let rack = crate::effectors::EffectorRack::build(document).expect("effector rack builds");
        let snapshot_map =
            build_effector_actual_snapshot_map(document, &bindings, &rack.snapshot(), false)
                .expect("effector snapshot map builds");

        assert_eq!(snapshot_map.get("delta_e_deg").copied(), Some(5.0));
        assert_eq!(snapshot_map.get("delta_r_deg").copied(), Some(-3.0));
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_rcs_feed_tank_drain_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let direct_torque_effector = r#"
[[vehicle.assembly.effectors]]
id = "booster-pitch-torque"
mounted_to = "booster"
kind = { kind = "direct_torque", axis = "pitch", effectiveness_n_m_per_rad = 10.0 }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
command_schedule = { kind = "constant", value = 1.0 }
"#;
        let booster_payload_tank_and_rcs = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.tanks]]
id = "payload_rcs_prop"
mounted_to = "payload"
mount_point_body_m = [0.0, 0.0, 0.0]
geometry = { kind = "cylinder", radius_m = 0.2, height_m = 1.0 }
propellant = { density_kg_m3 = 1000.0, label = "synthetic_mono" }
initial_fill_fraction = 0.5
moving_mass = { kind = "rigid_liquid" }
ullage = { initial_pressure_pa = 2000000.0, gas_gamma = 2.0 }

[[vehicle.assembly.effectors]]
id = "payload-pitch-rcs"
mounted_to = "payload"
kind = { kind = "direct_torque", axis = "pitch", effectiveness_n_m_per_rad = 10.0, rcs = { mode = "bank", thrusters = [
  { id = "pitch_pos", position_body_m = [1.0, 0.0, 0.0], direction_body = [0.0, 0.0, -1.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "pitch_neg", position_body_m = [1.0, 0.0, 0.0], direction_body = [0.0, 0.0, 1.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0 }
] } }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
initial_position = 0.0
unit = "rad"
command_schedule = { kind = "constant", value = 1.0 }
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace("stop_s = 0.3", "stop_s = 0.6")
            .replace(booster_body, booster_payload_tank_and_rcs)
            .replace(direct_torque_effector, "")
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let payload = body_id_from_scenario_text("payload");

        assert_eq!(
            unsupported_welded_descendant_runtime_resource(document, payload),
            None
        );
        require_supported_shape(document)
            .expect("descendant direct-torque RCS feed tank is supported");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");
        let tank = openbmp_core::TankId::from_path("vehicle.assembly.tanks.payload_rcs_prop");
        let initial_tank_mass = session
            .tank_rack
            .propellant_tank_states(document)
            .get(&tank)
            .expect("payload RCS tank state exists")
            .fluid_remaining_kg;
        let mut separated_masses = Vec::new();

        while !session.is_finished() {
            session
                .step_once(document, None)
                .expect("welded-release descendant RCS feed step succeeds");
            if let Some(lane) = session
                .separated_bodies()
                .iter()
                .find(|lane| lane.body == body_id_from_scenario_text("booster"))
            {
                separated_masses.push(lane.state.mass_props.mass_kg());
            }
        }

        let booster = body_id_from_scenario_text("booster");
        let booster_final_roll_rate = separated_lane_state(&session, booster)
            .angular_velocity
            .vector
            .x;
        assert!(
            booster_final_roll_rate.abs() > 1.0e-3,
            "payload RCS direct-torque output should rotate into the departed root lane frame: {booster_final_roll_rate}"
        );

        let effector_actual = session.effector_rack.snapshot()[0].actual;
        assert!(
            effector_actual > 0.0,
            "descendant RCS effector command should remain observable: {effector_actual}"
        );

        let final_tank_mass = session
            .tank_rack
            .propellant_tank_states(document)
            .get(&tank)
            .expect("payload RCS tank state exists after run")
            .fluid_remaining_kg;
        assert!(
            initial_tank_mass - final_tank_mass > 1.0e-3,
            "descendant RCS tank feed should drain the routed payload tank: initial={initial_tank_mass} final={final_tank_mass}"
        );
        let separated_mass_drop = separated_masses
            .first()
            .zip(separated_masses.last())
            .map(|(first, last)| first - last)
            .unwrap_or_default();
        assert!(
            separated_mass_drop > 1.0e-3,
            "descendant RCS tank drain should propagate into departed root mass: {separated_masses:?}"
        );
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_coupled_rcs_three_axis_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let direct_torque_effector = r#"
[[vehicle.assembly.effectors]]
id = "booster-pitch-torque"
mounted_to = "booster"
kind = { kind = "direct_torque", axis = "pitch", effectiveness_n_m_per_rad = 10.0 }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
command_schedule = { kind = "constant", value = 1.0 }
"#;
        let booster_payload_tank_and_coupled_rcs = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.tanks]]
id = "payload_rcs_prop"
mounted_to = "payload"
mount_point_body_m = [0.0, 0.0, 0.0]
geometry = { kind = "cylinder", radius_m = 0.2, height_m = 1.0 }
propellant = { density_kg_m3 = 1000.0, label = "synthetic_mono" }
initial_fill_fraction = 0.5
moving_mass = { kind = "rigid_liquid" }
ullage = { initial_pressure_pa = 2000000.0, gas_gamma = 2.0 }

[[vehicle.assembly.effectors]]
id = "payload-roll-rcs"
mounted_to = "payload"
kind = { kind = "direct_torque", axis = "roll", effectiveness_n_m_per_rad = 10.0, rcs = { mode = "coupled_bank", group = "payload_acs", thrusters = [
  { id = "roll_pos", position_body_m = [0.0, 1.0, 0.0], direction_body = [0.0, 0.0, 1.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "roll_neg", position_body_m = [0.0, 1.0, 0.0], direction_body = [0.0, 0.0, -1.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "pitch_pos", position_body_m = [0.0, 0.0, 1.0], direction_body = [1.0, 0.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "pitch_neg", position_body_m = [0.0, 0.0, 1.0], direction_body = [-1.0, 0.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "yaw_pos", position_body_m = [1.0, 0.0, 0.0], direction_body = [0.0, 1.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "yaw_neg", position_body_m = [1.0, 0.0, 0.0], direction_body = [0.0, -1.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } }
] } }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
initial_position = 0.0
unit = "rad"
command_schedule = { kind = "constant", value = 0.6 }

[[vehicle.assembly.effectors]]
id = "payload-pitch-rcs"
mounted_to = "payload"
kind = { kind = "direct_torque", axis = "pitch", effectiveness_n_m_per_rad = 10.0, rcs = { mode = "coupled_bank", group = "payload_acs", thrusters = [
  { id = "roll_pos", position_body_m = [0.0, 1.0, 0.0], direction_body = [0.0, 0.0, 1.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "roll_neg", position_body_m = [0.0, 1.0, 0.0], direction_body = [0.0, 0.0, -1.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "pitch_pos", position_body_m = [0.0, 0.0, 1.0], direction_body = [1.0, 0.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "pitch_neg", position_body_m = [0.0, 0.0, 1.0], direction_body = [-1.0, 0.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "yaw_pos", position_body_m = [1.0, 0.0, 0.0], direction_body = [0.0, 1.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "yaw_neg", position_body_m = [1.0, 0.0, 0.0], direction_body = [0.0, -1.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } }
] } }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
initial_position = 0.0
unit = "rad"
command_schedule = { kind = "constant", value = 0.4 }

[[vehicle.assembly.effectors]]
id = "payload-yaw-rcs"
mounted_to = "payload"
kind = { kind = "direct_torque", axis = "yaw", effectiveness_n_m_per_rad = 10.0, rcs = { mode = "coupled_bank", group = "payload_acs", thrusters = [
  { id = "roll_pos", position_body_m = [0.0, 1.0, 0.0], direction_body = [0.0, 0.0, 1.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "roll_neg", position_body_m = [0.0, 1.0, 0.0], direction_body = [0.0, 0.0, -1.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "pitch_pos", position_body_m = [0.0, 0.0, 1.0], direction_body = [1.0, 0.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "pitch_neg", position_body_m = [0.0, 0.0, 1.0], direction_body = [-1.0, 0.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "yaw_pos", position_body_m = [1.0, 0.0, 0.0], direction_body = [0.0, 1.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } },
  { id = "yaw_neg", position_body_m = [1.0, 0.0, 0.0], direction_body = [0.0, -1.0, 0.0], minimum_impulse_n_s = 0.001, nominal_thrust_n = 2000.0, feed = { tank = "payload_rcs_prop", specific_impulse_s = 220.0, pressure_exponent = 1.0 } }
] } }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
initial_position = 0.0
unit = "rad"
command_schedule = { kind = "constant", value = -0.5 }
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace("stop_s = 0.3", "stop_s = 0.6")
            .replace(booster_body, booster_payload_tank_and_coupled_rcs)
            .replace(direct_torque_effector, "")
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let payload = body_id_from_scenario_text("payload");

        assert_eq!(
            unsupported_welded_descendant_runtime_resource(document, payload),
            None
        );
        require_supported_shape(document)
            .expect("descendant direct-torque coupled-bank RCS feed tank is supported");
        let resolved_files = scenario.resolved_files().expect("resolved files");
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");
        let tank = openbmp_core::TankId::from_path("vehicle.assembly.tanks.payload_rcs_prop");
        let initial_tank_mass = session
            .tank_rack
            .propellant_tank_states(document)
            .get(&tank)
            .expect("payload coupled RCS tank state exists")
            .fluid_remaining_kg;
        let mut separated_masses = Vec::new();

        while !session.is_finished() {
            session
                .step_once(document, None)
                .expect("welded-release descendant coupled RCS feed step succeeds");
            if let Some(lane) = session
                .separated_bodies()
                .iter()
                .find(|lane| lane.body == body_id_from_scenario_text("booster"))
            {
                separated_masses.push(lane.state.mass_props.mass_kg());
            }
        }

        let booster_state = separated_lane_state(&session, body_id_from_scenario_text("booster"));
        let omega = booster_state.angular_velocity.vector;
        assert!(
            omega.x.abs() > 1.0e-3 && omega.y.abs() > 1.0e-3 && omega.z.abs() > 1.0e-3,
            "payload coupled-bank RCS should route three transformed axes into the departed root lane: {omega:?}"
        );

        let actuals = session
            .effector_rack
            .snapshot()
            .into_iter()
            .map(|state| state.actual)
            .collect::<Vec<_>>();
        assert_eq!(actuals.len(), 3);
        assert!(
            actuals.iter().all(|actual| actual.abs() > 1.0e-3),
            "descendant coupled-bank RCS effectors should remain observable: {actuals:?}"
        );

        let final_tank_mass = session
            .tank_rack
            .propellant_tank_states(document)
            .get(&tank)
            .expect("payload coupled RCS tank state exists after run")
            .fluid_remaining_kg;
        assert!(
            initial_tank_mass - final_tank_mass > 1.0e-3,
            "descendant coupled-bank RCS tank feed should drain the routed payload tank: initial={initial_tank_mass} final={final_tank_mass}"
        );
        let separated_mass_drop = separated_masses
            .first()
            .zip(separated_masses.last())
            .map(|(first, last)| first - last)
            .unwrap_or_default();
        assert!(
            separated_mass_drop > 1.0e-3,
            "descendant coupled-bank RCS tank drain should propagate into departed root mass: {separated_masses:?}"
        );
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_engine_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let booster_payload_and_engine = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.engines]]
id = "payload_engine"
mounted_to = "payload"
kind = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, 0.0]
limits = { max_thrust_n = 100.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }
"#;
        let jettison_and_ignite = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.7071067811865475, 0.0, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true

[[scenario_script.events]]
id = "ignite_payload_engine"
trigger = { kind = "at_time", time_s = 0.15 }
action = { kind = "engine_command", id = "payload_engine", command = { throttle_unit = 1.0, gimbal_pitch_rad = 0.0, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace("stop_s = 0.3", "stop_s = 0.4")
            .replace(booster_body, booster_payload_and_engine)
            .replace(
                "models = [\"gravity\"]",
                "models = [\"gravity\", \"thrust\"]",
            )
            .replace(initial_lane, jettison_and_ignite)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let outcome =
            crate::run(&scenario).expect("welded-release descendant engine scenario must run");

        let primary_vx = f64_column(&outcome, "velocity_x_m_s");
        let booster_vx = f64_column(&outcome, "body.booster.velocity_x_m_s");
        let payload_thrust = f64_column(&outcome, "engine.payload_engine.thrust_n");
        assert!(
            payload_thrust.iter().any(|value| *value > 0.0),
            "payload-mounted engine should ignite after welded release: {payload_thrust:?}"
        );
        assert!(
            booster_vx.last().copied().unwrap_or_default() < -1.0,
            "payload-mounted child-frame +Z thrust should rotate into departing-root -X velocity: {booster_vx:?}"
        );
        assert!(
            primary_vx.last().copied().unwrap_or_default().abs() < 1.0e-12,
            "descendant engine should not thrust the continuing primary stack: {primary_vx:?}"
        );
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_motor_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let booster_and_payload_bodies = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.7071067811865475, 0.0, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let motor = r#"
[propulsion.motor]
variant = "solid"
ignite_at_s = 0.0
mounted_to = "payload"

[propulsion.motor.grain]
geometry = "end_burner"
cross_section_area_m2 = 0.02
length_m = 0.05
throat_radius_m = 0.005641895835477563
expansion_ratio = 20.0
dry_mass_kg = 0.2
name = "descendant-solid-motor-grain"
provenance = "synthetic welded-release descendant motor regression"

[propulsion.motor.grain.propellant]
label = "synthetic_textbook"
density_kg_m3 = 1700.0
burn_rate_a = 0.00004
burn_rate_n = 0.32
c_star_m_s = 1400.0
gamma = 1.2
web_steps = 64
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(booster_body, booster_and_payload_bodies)
            .replace(initial_lane, jettison)
            .replace("[forces]\n", &format!("{motor}\n[forces]\n"))
            .replace(
                "models = [\"gravity\"]",
                "models = [\"gravity\", \"thrust\"]",
            )
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let booster = body_id_from_scenario_text("booster");
        let payload = body_id_from_scenario_text("payload");

        assert_eq!(
            unsupported_welded_descendant_runtime_resource(document, payload),
            None
        );
        assert_eq!(motor_owner(document), Some(booster));
        let transform =
            welded_release_motor_frame_transform(document).expect("descendant motor transform");
        let child_z_in_root = transform.rot_owner_from_motor_body * Vector3::new(0.0, 0.0, 1.0);
        assert!((child_z_in_root - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1.0e-12);

        let assembly =
            crate::assembly::synthesize_assembly(document).expect("assembly synthesizes");
        let resources = RigidMassResources::new(document, &assembly).expect("mass resources");
        assert_eq!(resources.motor_owner, Some(booster));
        let loaded_motor = crate::propulsion::load_solid_motor(document, &BTreeMap::new())
            .expect("motor load succeeds")
            .expect("motor is present");
        let initial_motor_mass = loaded_motor.mass_kg(0.0).expect("motor mass");
        let model = RigidMassResourceModel::new(
            resources,
            Some(loaded_motor),
            None,
            RIGID_BODY_MOTOR_MASS_MODEL_ID,
        );
        let root_props = model
            .mass_properties_from_snapshots(
                SimTime::ZERO,
                Some(booster),
                &BTreeMap::new(),
                &BTreeMap::new(),
                &BTreeMap::new(),
            )
            .expect("root motor mass properties");
        let child_props = model
            .mass_properties_from_snapshots(
                SimTime::ZERO,
                Some(payload),
                &BTreeMap::new(),
                &BTreeMap::new(),
                &BTreeMap::new(),
            )
            .expect("child dry mass properties");
        let root_rate = model
            .mass_properties_rate_from_snapshots(
                SimTime::from_seconds(0.01),
                Some(booster),
                &BTreeMap::new(),
                &BTreeMap::new(),
                &BTreeMap::new(),
            )
            .expect("root motor mass rate");

        assert!((root_props.mass_kg() - (1.0 + initial_motor_mass)).abs() < 1.0e-12);
        assert_eq!(child_props.mass_kg().to_bits(), 2.0_f64.to_bits());
        assert!(
            root_rate.mass_rate_kg_s < 0.0,
            "departing root should carry descendant motor mass flow: {root_rate:?}"
        );
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_tank_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let booster_payload_and_tank = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.tanks]]
id = "payload_tank"
mounted_to = "payload"
mount_point_body_m = [2.0, 0.0, 0.0]
geometry = { kind = "sphere", radius_m = 0.25 }
propellant = { label = "synthetic_water", density_kg_m3 = 1000.0 }
initial_fill_fraction = 0.5
moving_mass = { kind = "rigid_liquid" }
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(booster_body, booster_payload_and_tank)
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let booster = body_id_from_scenario_text("booster");
        let tank = TankId::from_path("vehicle.assembly.tanks.payload_tank");

        let tank_owners = tank_owner_map(document);
        assert_eq!(tank_owners.get(&tank).copied(), Some(booster));
        let transforms = welded_release_tank_frame_transforms(document);
        let transform = transforms.get(&tank).expect("descendant tank transform");
        let child_y_in_root = transform.rot_owner_from_tank_body * Vector3::new(0.0, 1.0, 0.0);
        assert!((child_y_in_root - Vector3::new(1.0, 0.0, 0.0)).norm() < 1.0e-12);
        assert!(
            (transform.translation_owner_from_tank_body_m - Vector3::new(0.0, 3.0, 0.0)).norm()
                < 1.0e-12
        );

        let assembly =
            crate::assembly::synthesize_assembly(document).expect("assembly synthesizes");
        let resources = RigidMassResources::new(document, &assembly).expect("mass resources");
        let route = resources.tank_routes.get(&tank).expect("tank mass route");
        assert_eq!(route.owner, booster);
        assert!((route.mount_point_tank_body_m - Vector3::new(2.0, 0.0, 0.0)).norm() < 1.0e-12);

        let model =
            RigidMassResourceModel::<SolidMotor>::new(resources, None, None, ModelId::new(999));
        let center_tank_body = Vector3::new(2.0, 1.0, 0.0);
        let tank_snapshot = BTreeMap::from([(
            tank,
            openbmp_sim::TankSnapshot {
                mass_kg: 5.0,
                cg_offset_body_m: Vector3::new(0.0, 1.0, 0.0),
                inertia_delta_body_kg_m2: point_mass_parallel_axis_inertia(5.0, center_tank_body),
                reaction_force_body_n: Vector3::zeros(),
                reaction_moment_body_n_m: Vector3::zeros(),
                fluid_remaining_kg: 5.0,
                mass_rate_kg_s: 0.0,
            },
        )]);
        let props = model
            .mass_properties_from_snapshots(
                SimTime::ZERO,
                Some(booster),
                &BTreeMap::new(),
                &tank_snapshot,
                &BTreeMap::new(),
            )
            .expect("tank mass properties");

        assert!((props.mass_kg() - 6.0).abs() < 1.0e-12);
        assert!(
            (props.center_of_mass_body.vector - Vector3::new(5.0 / 6.0, 5.0 / 6.0, 0.0)).norm()
                < 1.0e-12
        );
        let expected_inertia = Matrix3::new(6.0, -5.0, 0.0, -5.0, 6.0, 0.0, 0.0, 0.0, 11.0);
        assert!((props.inertia_body - expected_inertia).norm() < 1.0e-12);
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_tank_coupled_engine_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let booster_payload_tank_and_engine = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.tanks]]
id = "payload_tank"
mounted_to = "payload"
mount_point_body_m = [2.0, 0.0, 0.0]
geometry = { kind = "sphere", radius_m = 0.25 }
propellant = { label = "synthetic_water", density_kg_m3 = 1000.0 }
initial_fill_fraction = 0.5
moving_mass = { kind = "rigid_liquid" }

[[vehicle.assembly.engines]]
id = "boost"
mounted_to = "payload"
kind = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 1.0, 0.0]
limits = { max_thrust_n = 100.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }

[vehicle.assembly.engines.propellant]
oxidizer_fuel_ratio = 0.0
fuel_tank = "payload_tank"
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(booster_body, booster_payload_tank_and_engine)
            .replace(
                "models = [\"gravity\"]",
                "models = [\"gravity\", \"thrust\"]",
            )
            .replace(initial_lane, "")
            .replace(
                "[multi_body]\nprimary_body_id = \"bus\"\n",
                "[multi_body]\nprimary_body_id = \"bus\"\npropagation_authority = \"welded_release_jettison\"\n\
                 [[multi_body.separation]]\n\
                 event_id = \"drop_booster\"\n\
                 upper_body_id = \"bus\"\n\
                 lower_body_id = \"booster\"\n\
                 lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]\n\
                 lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]\n\
                 \n\
                 [[multi_body.welded_body]]\n\
                 parent_body_id = \"booster\"\n\
                 child_body_id = \"payload\"\n\
                 child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]\n\
                 child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]\n\
                 \n\
                 [scenario_script]\n\
                 [[scenario_script.events]]\n\
                 id = \"drop_booster\"\n\
                 trigger = { kind = \"at_time\", time_s = 0.05 }\n\
                 action = { kind = \"jettison_stage\", body = \"booster\" }\n\
                 once = true\n\
                 \n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let booster = body_id_from_scenario_text("booster");
        let payload = body_id_from_scenario_text("payload");
        let engine = EngineId::from_path("vehicle.assembly.engines.boost");
        let tank = TankId::from_path("vehicle.assembly.tanks.payload_tank");

        assert_eq!(
            unsupported_welded_descendant_runtime_resource(document, payload),
            None
        );

        let engine_owners = engine_owner_map(document).expect("engine owners");
        assert_eq!(engine_owners.get(&engine).copied(), Some(booster));
        let engine_transforms = welded_release_engine_frame_transforms(document);
        let engine_transform = engine_transforms
            .get(&engine)
            .expect("descendant engine transform");
        let child_y_in_root =
            engine_transform.rot_owner_from_engine_body * Vector3::new(0.0, 1.0, 0.0);
        assert!((child_y_in_root - Vector3::new(1.0, 0.0, 0.0)).norm() < 1.0e-12);
        assert!(
            (engine_transform.mount_point_owner_m - Vector3::new(1.0, 3.0, 0.0)).norm() < 1.0e-12
        );

        let tank_owners = tank_owner_map(document);
        assert_eq!(tank_owners.get(&tank).copied(), Some(booster));
        let tank_transforms = welded_release_tank_frame_transforms(document);
        assert!(
            tank_transforms.contains_key(&tank),
            "tank-coupled descendant tank should carry welded frame transform"
        );

        let assembly =
            crate::assembly::synthesize_assembly(document).expect("assembly synthesizes");
        let resources = RigidMassResources::new(document, &assembly).expect("mass resources");
        assert!(resources.tank_coupled_engines.contains(&engine));
        let route = resources.tank_routes.get(&tank).expect("tank mass route");
        assert_eq!(route.owner, booster);

        let model =
            RigidMassResourceModel::<SolidMotor>::new(resources, None, None, ModelId::new(999));
        let engine_snapshot = BTreeMap::from([(
            engine,
            openbmp_sim::EngineSnapshot {
                thrust_body: Vector3::new(0.0, 0.0, 100.0),
                mass_flow_kg_per_s: 2.0,
                consumed_kg: 9.0,
                gimbal_pitch_rad: 0.0,
                gimbal_yaw_rad: 0.0,
                gimbal_pitch_rate_rad_s: 0.0,
                gimbal_yaw_rate_rad_s: 0.0,
                tvc_pitch_load_torque_n_m: 0.0,
                tvc_yaw_load_torque_n_m: 0.0,
                tvc_pitch_inertial_reaction_torque_n_m: 0.0,
                tvc_yaw_inertial_reaction_torque_n_m: 0.0,
                lifecycle_state_index: 2,
            },
        )]);
        let tank_snapshot = BTreeMap::from([(
            tank,
            openbmp_sim::TankSnapshot {
                mass_kg: 5.0,
                cg_offset_body_m: Vector3::zeros(),
                inertia_delta_body_kg_m2: Matrix3::zeros(),
                reaction_force_body_n: Vector3::zeros(),
                reaction_moment_body_n_m: Vector3::zeros(),
                fluid_remaining_kg: 5.0,
                mass_rate_kg_s: -2.0,
            },
        )]);
        let props = model
            .mass_properties_from_snapshots(
                SimTime::ZERO,
                Some(booster),
                &engine_snapshot,
                &tank_snapshot,
                &BTreeMap::new(),
            )
            .expect("tank-coupled mass properties");
        let rate = model
            .mass_properties_rate_from_snapshots(
                SimTime::ZERO,
                Some(booster),
                &engine_snapshot,
                &tank_snapshot,
                &BTreeMap::new(),
            )
            .expect("tank-coupled mass rate");

        assert!(
            (props.mass_kg() - 6.0).abs() < 1.0e-12,
            "tank owns depletion; consumed_kg must not be subtracted twice"
        );
        assert_eq!(rate.mass_rate_kg_s.to_bits(), (-2.0_f64).to_bits());
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_landing_gear_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let booster_payload_and_landing_gear = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[vehicle.landing_gear]
ground_altitude_m = 0.0

[[vehicle.landing_gear.legs]]
id = "payload_leg"
mounted_to = "payload"
attach_body_m = [0.0, 1.0, 0.0]
strut_axis_body = [0.0, 0.0, -1.0]
free_length_m = 1.0
footpad = "point"
oleo = { p0_pa = 100000.0, v0_m3 = 0.01, gamma_unit = 1.2, orifice_c_n_s2_m2 = 0.0, stroke_max_m = 0.2, piston_area_m2 = 0.01 }
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(booster_body, booster_payload_and_landing_gear)
            .replace(
                "models = [\"gravity\"]",
                "models = [\"gravity\", \"landing_gear\"]",
            )
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let booster = body_id_from_scenario_text("booster");
        let payload = body_id_from_scenario_text("payload");

        assert_eq!(
            unsupported_welded_descendant_runtime_resource(document, payload),
            None
        );
        require_supported_shape(document).expect("welded descendant landing gear is supported");
        let aliases = landing_gear_owner_aliases(document);
        let alias = aliases.get(&payload).expect("payload landing gear alias");
        assert_eq!(alias.root, booster);
        let attach_owner = alias.translation_root_from_body_m
            + alias.rot_root_from_body * Vector3::new(0.0, 1.0, 0.0);
        let axis_owner = alias.rot_root_from_body * Vector3::new(0.0, 0.0, -1.0);
        assert!((attach_owner - Vector3::new(1.0, 3.0, 0.0)).norm() < 1.0e-12);
        assert!((axis_owner - Vector3::new(0.0, 0.0, -1.0)).norm() < 1.0e-12);
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_aero_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let booster_and_payload_bodies = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let aero = r#"
[aero]
mounted_to = "payload"

[aero.method]
kind = "modified_newtonian"

[aero.method.modified_newtonian]
cp_max = 2.0
reference_area_m2 = 1.0
reference_length_m = 1.0
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(booster_body, booster_and_payload_bodies)
            .replace(initial_lane, jettison)
            .replace("[environment]\n", &format!("{aero}\n[environment]\n"))
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let booster = body_id_from_scenario_text("booster");
        let payload = body_id_from_scenario_text("payload");

        assert_eq!(
            unsupported_welded_descendant_runtime_resource(document, payload),
            None
        );
        assert_eq!(aero_owner(document), Some(booster));
        let transform =
            welded_release_aero_frame_transform(document).expect("descendant aero transform");
        let child_y_in_root = transform.rot_owner_from_aero_body * Vector3::new(0.0, 1.0, 0.0);
        assert!((child_y_in_root - Vector3::new(1.0, 0.0, 0.0)).norm() < 1.0e-12);
        assert!(
            (transform.translation_owner_from_aero_body_m - Vector3::new(0.0, 3.0, 0.0)).norm()
                < 1.0e-12
        );
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_scalar_recovery_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let booster_payload_and_recovery = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.recovery]]
id = "payload_chute"
mounted_to = "payload"
kind = { kind = "parachute_drag", c_d = 1.5, area_inflated_m2 = 8.0 }
inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = "polynomial", exponent_j = 1.0, apparent_mass_coeff_k_a = 0.65 }
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]

[[multi_body.welded_body]]
parent_body_id = "booster"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(booster_body, booster_payload_and_recovery)
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let booster = body_id_from_scenario_text("booster");
        let payload = body_id_from_scenario_text("payload");
        let recovery = RecoveryId::from_path("vehicle.assembly.recovery.payload_chute");

        assert_eq!(
            unsupported_welded_descendant_runtime_resource(document, payload),
            None
        );
        let recovery_owners = recovery_owner_map(document).expect("recovery owner map");
        assert_eq!(recovery_owners.get(&recovery).copied(), Some(booster));

        let assembly =
            crate::assembly::synthesize_assembly(document).expect("assembly synthesizes");
        let resources = RigidMassResources::new(document, &assembly).expect("mass resources");
        assert_eq!(
            resources.recovery_owners.get(&recovery).copied(),
            Some(booster)
        );
        let model =
            RigidMassResourceModel::<SolidMotor>::new(resources, None, None, ModelId::new(999));
        let recovery_snapshot = BTreeMap::from([(
            recovery,
            openbmp_sim::RecoverySnapshot {
                apparent_mass_kg: 4.0,
                apparent_mass_rate_kg_s: 0.25,
                ..openbmp_sim::RecoverySnapshot::default()
            },
        )]);

        let props = model
            .mass_properties_from_snapshots(
                SimTime::ZERO,
                Some(booster),
                &BTreeMap::new(),
                &BTreeMap::new(),
                &recovery_snapshot,
            )
            .expect("recovery apparent mass properties");
        let rate = model
            .mass_properties_rate_from_snapshots(
                SimTime::ZERO,
                Some(booster),
                &BTreeMap::new(),
                &BTreeMap::new(),
                &recovery_snapshot,
            )
            .expect("recovery apparent mass rate");

        assert_eq!(props.mass_kg().to_bits(), 5.0_f64.to_bits());
        assert_eq!(rate.mass_rate_kg_s.to_bits(), 0.25_f64.to_bits());
    }

    #[test]
    fn welded_release_jettison_authority_routes_descendant_recovery_risers_to_root_lane() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let booster_body = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"#;
        let booster_payload_and_recovery = r#"
[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "payload"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.recovery]]
id = "payload_chute"
mounted_to = "payload"
kind = { kind = "parachute_drag", c_d = 1.5, area_inflated_m2 = 8.0 }
sequence = { snatch_delta_v_m_s = 3.0, snatch_effective_mass_kg = 5.0, mortar_pack_mass_kg = 2.0, mortar_pack_velocity_body_m_s = [1.0, 0.0, 0.0] }
risers = [{ id = "main_riser", unstretched_length_m = 0.5, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 1.0, 0.0], canopy_attach_body_m = [1.0, 1.0, 0.0] }]
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(booster_body, booster_payload_and_recovery)
            .replace("atmosphere = \"none\"", "atmosphere = \"us_standard_1976\"")
            .replace(initial_lane, "")
            .replace(
                "[multi_body]\nprimary_body_id = \"bus\"\n",
                "[multi_body]\nprimary_body_id = \"bus\"\npropagation_authority = \"welded_release_jettison\"\n\
                 [[multi_body.separation]]\n\
                 event_id = \"drop_booster\"\n\
                 upper_body_id = \"bus\"\n\
                 lower_body_id = \"booster\"\n\
                 lower_weld_translation_upper_body_m = [0.0, 2.0, 0.0]\n\
                 lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]\n\
                 \n\
                 [[multi_body.welded_body]]\n\
                 parent_body_id = \"booster\"\n\
                 child_body_id = \"payload\"\n\
                 child_weld_translation_parent_body_m = [0.0, 3.0, 0.0]\n\
                 child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]\n\
                 \n\
                 [scenario_script]\n\
                 [[scenario_script.events]]\n\
                 id = \"drop_booster\"\n\
                 trigger = { kind = \"at_time\", time_s = 0.05 }\n\
                 action = { kind = \"jettison_stage\", body = \"booster\" }\n\
                 once = true\n\
                 \n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let document = &scenario.document;
        let booster = body_id_from_scenario_text("booster");
        let payload = body_id_from_scenario_text("payload");
        let recovery = RecoveryId::from_path("vehicle.assembly.recovery.payload_chute");

        assert_eq!(
            unsupported_welded_descendant_runtime_resource(document, payload),
            None
        );
        require_supported_shape(document).expect("welded descendant recovery risers are supported");
        let recovery_owners = recovery_owner_map(document).expect("recovery owner map");
        assert_eq!(recovery_owners.get(&recovery).copied(), Some(booster));

        let frame_aliases = recovery_frame_aliases(document);
        let alias = frame_aliases.get(&payload).expect("payload recovery alias");
        let attach_owner = alias.translation_owner_from_body_m
            + alias.rot_owner_from_body * Vector3::new(0.0, 1.0, 0.0);
        let mortar_velocity_owner = alias.rot_owner_from_body * Vector3::new(1.0, 0.0, 0.0);
        assert!((attach_owner - Vector3::new(1.0, 3.0, 0.0)).norm() < 1.0e-12);
        assert!((mortar_velocity_owner - Vector3::new(0.0, -1.0, 0.0)).norm() < 1.0e-12);
    }

    #[test]
    fn welded_release_jettison_authority_applies_impulses_and_attitude_offset() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"
upper_delta_v_body_m_s = [1.0, 0.0, 0.0]
lower_delta_v_body_m_s = [0.0, 2.0, 0.0]
upper_delta_omega_body_rad_s = [0.5, 0.0, 0.0]
lower_delta_omega_body_rad_s = [0.0, 0.0, 0.25]
lower_attitude_offset_body_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]
conserve_momentum = false

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"welded_release_jettison\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("welded-release jettison should apply configured impulses");

        let booster = body_id_from_scenario_text("booster");
        let lane = separated_lane_state(&session, booster);
        let stack_delta_v_eci = session.state().orientation.q * Vector3::new(1.0, 0.0, 0.0);
        let stage_delta_v_eci = session.state().orientation.q * Vector3::new(0.0, 2.0, 0.0);
        assert!(
            (session.state().velocity.vector - stack_delta_v_eci).norm() < 1.0e-12,
            "stack delta-v should be applied: {:?}",
            session.state().velocity.vector
        );
        assert!(
            (lane.velocity.vector - stage_delta_v_eci).norm() < 1.0e-12,
            "departing delta-v should be applied: {:?}",
            lane.velocity.vector
        );
        assert!(
            (session.state().angular_velocity.vector.x - lane.angular_velocity.vector.x - 0.5)
                .abs()
                < 1.0e-12,
            "stack tip-off should be applied: {:?}",
            session.state().angular_velocity.vector
        );
        assert!(
            (lane.angular_velocity.vector.z - session.state().angular_velocity.vector.z - 0.25)
                .abs()
                < 1.0e-12,
            "departing tip-off should be applied: {:?}",
            lane.angular_velocity.vector
        );
        let q = (session.state().orientation.q.inverse() * lane.orientation.q).into_inner();
        assert!((q.i - 0.0).abs() < 1.0e-12);
        assert!((q.j - 0.0).abs() < 1.0e-12);
        assert!((q.k - std::f64::consts::FRAC_1_SQRT_2).abs() < 1.0e-12);
        assert!((q.w - std::f64::consts::FRAC_1_SQRT_2).abs() < 1.0e-12);
    }

    #[test]
    fn separated_multibody_shadow_rk4_forecast_matches_jettisoned_lane_rigid_step() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO.replace(initial_lane, jettison);
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("jettison step records forecast");
        let booster = body_id_from_scenario_text("booster");
        let predicted = session
            .separated_multibody_shadows
            .get(&booster)
            .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
            .expect("jettisoned booster RK4 forecast is recorded")
            .rigid_state;

        session
            .step_once(&scenario.document, None)
            .expect("jettisoned lane rigid step succeeds");

        assert_rigid_forecast_matches_state(
            &predicted,
            separated_lane_state(&session, booster),
            0.2,
        );
    }

    #[test]
    fn separated_multibody_root_free_flyer_authority_records_jettisoned_lane_handoff() {
        let initial_lane = r#"
[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]
"#;
        let jettison = r#"
[[multi_body.separation]]
event_id = "drop_booster"
upper_body_id = "bus"
lower_body_id = "booster"

[scenario_script]
[[scenario_script.events]]
id = "drop_booster"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "jettison_stage", body = "booster" }
once = true
"#;
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(initial_lane, jettison)
            .replace(
                "[multi_body]\n",
                "[multi_body]\npropagation_authority = \"separated_root_free_flyer\"\n",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("jettison step records next separated forecast");
        let booster = body_id_from_scenario_text("booster");
        assert!(
            session
                .separated_multibody_shadows
                .get(&booster)
                .and_then(|shadow| shadow.last_authoritative_state.as_ref())
                .is_none(),
            "jettison-created lane cannot be overwritten before a pre-step forecast exists"
        );

        session
            .step_once(&scenario.document, None)
            .expect("post-jettison step applies separated authority");
        let shadow = session
            .separated_multibody_shadows
            .get(&booster)
            .expect("jettisoned booster shadow remains recorded");
        let authoritative = shadow
            .last_authoritative_state
            .as_ref()
            .expect("jettisoned booster handoff is recorded");
        assert_rigid_forecast_matches_state(
            authoritative,
            separated_lane_state(&session, booster),
            0.2,
        );
    }

    #[test]
    fn separated_multibody_shadow_rk4_forecast_matches_variable_mass_engine_lane_step() {
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(SEPARATED_LANDING_CONTROLLER_SCENARIO)
                .expect("separated landing-controller scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        let booster = body_id_from_scenario_text("booster");
        let initial_mass_kg = separated_lane_state(&session, booster).mass_props.mass_kg();

        session
            .step_once(&scenario.document, None)
            .expect("powered separated lane step records forecast");
        let forecast = session
            .separated_multibody_shadows
            .get(&booster)
            .and_then(|shadow| shadow.last_rk4_forecast.as_ref())
            .expect("powered booster separated-lane RK4 forecast is recorded");
        let predicted = &forecast.rigid_state;
        assert!(
            predicted.mass_props.mass_kg() < initial_mass_kg,
            "powered separated forecast should drain mass: predicted={} initial={}",
            predicted.mass_props.mass_kg(),
            initial_mass_kg
        );
        assert_rigid_forecast_matches_state(
            predicted,
            separated_lane_state(&session, booster),
            0.1,
        );
    }

    const PRIMARY_MULTIBODY_BRIDGE_BASELINE_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "primary-multibody-bridge-byte-equivalence-test"
description = "Synthetic rigid-body run used to prove disabled primary multibody setup is non-perturbing."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.2
dt_s = 0.1
seed = 43

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [1.0, 2.0, 3.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "primary-multibody-bridge-byte-equivalence-test"

[[vehicle.assembly.bodies]]
id = "main"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 2.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.effectors]]
id = "main-pitch-torque"
mounted_to = "main"
kind = { kind = "direct_torque", axis = "pitch", effectiveness_n_m_per_rad = 10.0 }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }

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

[telemetry]
output.csv = "out/primary-multibody-bridge-byte-equivalence-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const PRIMARY_MULTIBODY_GIMBAL_SHADOW_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "primary-multibody-gimbal-shadow-test"
description = "Synthetic rigid-body run used to prove the primary multibody shadow sees gimballed engine loads."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.3
dt_s = 0.1
seed = 44

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "primary-multibody-gimbal-shadow-test"

[[vehicle.assembly.bodies]]
id = "main"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 10.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.engines]]
id = "main_engine"
mounted_to = "main"
kind = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 1.0, 0.0]
limits = { max_thrust_n = 100.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.2 }

[[vehicle.assembly.effectors]]
id = "unused_pitch"
mounted_to = "main"
kind = { kind = "direct_torque", axis = "pitch", effectiveness_n_m_per_rad = 1.0 }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["thrust"]

[mission]
initial_phase = "coast"

[[mission.phases]]
id = "coast"
label = "coast"

[multi_body]
primary_body_id = "main"

[[multi_body.attitude_target]]
body_id = "main"
start_time_s = 1.0
pitch_effector = "unused_pitch"
kp = 1.0
kd = 0.0
max_command = 1.0
target = { kind = "eci_vector", vector_eci = [0.0, 0.0, 1.0] }

[scenario_script]
[[scenario_script.events]]
id = "ignite_main"
trigger = { kind = "at_time", time_s = 0.05 }
action = { kind = "engine_command", id = "main_engine", command = { throttle_unit = 1.0, gimbal_pitch_rad = 0.1, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }
once = true

[telemetry]
output.csv = "out/primary-multibody-gimbal-shadow-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const PRIMARY_MULTIBODY_SOLID_MOTOR_SHADOW_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "primary-multibody-solid-motor-shadow-test"
description = "Synthetic rigid-body run used to prove the primary multibody shadow forecasts a variable-mass burn."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.2
dt_s = 0.1
seed = 45

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "primary-multibody-solid-motor-shadow-test"

[[vehicle.assembly.bodies]]
id = "main"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 10.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]

[[vehicle.assembly.effectors]]
id = "unused_pitch"
mounted_to = "main"
kind = { kind = "direct_torque", axis = "pitch", effectiveness_n_m_per_rad = 1.0 }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[propulsion.motor]
variant = "solid"
ignite_at_s = 0.0
mounted_to = "main"

[propulsion.motor.grain]
geometry = "end_burner"
cross_section_area_m2 = 0.02
length_m = 0.05
throat_radius_m = 0.005641895835477563
expansion_ratio = 20.0
dry_mass_kg = 0.2
name = "primary-multibody-solid-motor-shadow-grain"
provenance = "synthetic primary multibody variable-mass regression"

[propulsion.motor.grain.propellant]
label = "synthetic_textbook"
density_kg_m3 = 1700.0
burn_rate_a = 0.00004
burn_rate_n = 0.32
c_star_m_s = 1400.0
gamma = 1.2
web_steps = 64

[forces]
models = ["thrust"]

[mission]
initial_phase = "coast"

[[mission.phases]]
id = "coast"
label = "coast"

[multi_body]
primary_body_id = "main"

[[multi_body.attitude_target]]
body_id = "main"
start_time_s = 1.0
pitch_effector = "unused_pitch"
kp = 1.0
kd = 0.0
max_command = 1.0
target = { kind = "eci_vector", vector_eci = [0.0, 0.0, 1.0] }

[telemetry]
output.csv = "out/primary-multibody-solid-motor-shadow-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

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

[[scenario_script.events]]
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

    const RIGID_ENGINE_TELEMETRY_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "rigid-engine-telemetry-test"
description = "Synthetic rigid-body engine telemetry smoke test."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.5
dt_s = 0.1
seed = 29

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "rigid-engine-telemetry-test"

[[vehicle.assembly.bodies]]
id = "core"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 100.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]]

[[vehicle.assembly.engines]]
id = "main"
mounted_to = "core"
kind = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, 0.0]
limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity", "thrust"]

[mission]
initial_phase = "flight"

[[mission.phases]]
id = "flight"
label = "flight"

[[scenario_script.events]]
id = "ignite"
trigger = { kind = "at_time", time_s = 0.1 }
action = { kind = "engine_command", id = "main", command = { throttle_unit = 1.0, gimbal_pitch_rad = 0.0, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }
once = true

[telemetry]
output.csv = "out/rigid-engine-telemetry-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const SEPARATED_RETIREMENT_TELEMETRY_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "separated-retirement-telemetry-test"
description = "Synthetic separated-lane retirement telemetry regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 2.0
dt_s = 1.0
seed = 31

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [6371100.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "separated-retirement-telemetry-test"

[[vehicle.assembly.bodies]]
id = "bus"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 3.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "booster"
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

[multi_body]
primary_body_id = "bus"

[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [6371101.0, 0.0, 0.0]
velocity_eci_m_s = [-2.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[telemetry]
output.csv = "out/separated-retirement-telemetry-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const RETIRED_BODY_ENGINE_COMMAND_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "retired-body-engine-command-test"
description = "Synthetic retired separated-lane engine command regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 3.0
dt_s = 1.0
seed = 37

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [6371100.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "retired-body-engine-command-test"

[[vehicle.assembly.bodies]]
id = "bus"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 3.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[0.2, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.0, 0.2]]

[[vehicle.assembly.engines]]
id = "boost"
mounted_to = "booster"
kind = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, 0.0]
limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity", "thrust"]

[mission]
initial_phase = "coast"

[[mission.phases]]
id = "coast"
label = "coast"

[[scenario_script.events]]
id = "ignite_retired_booster"
trigger = { kind = "at_time", time_s = 2.0 }
action = { kind = "engine_command", id = "boost", command = { throttle_unit = 1.0, gimbal_pitch_rad = 0.0, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }
once = true

[multi_body]
primary_body_id = "bus"

[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [6371101.0, 0.0, 0.0]
velocity_eci_m_s = [-2.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[telemetry]
output.csv = "out/retired-body-engine-command-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const SEPARATED_DIRECT_TORQUE_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "separated-direct-torque-test"
description = "Synthetic separated-lane direct-torque routing regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.3
dt_s = 0.1
seed = 41

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "separated-direct-torque-test"

[[vehicle.assembly.bodies]]
id = "bus"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 3.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[3.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 3.0]]

[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.effectors]]
id = "booster-pitch-torque"
mounted_to = "booster"
kind = { kind = "direct_torque", axis = "pitch", effectiveness_n_m_per_rad = 10.0 }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
command_schedule = { kind = "constant", value = 1.0 }

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

[multi_body]
primary_body_id = "bus"

[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [0.0, 0.0, 10.0]
velocity_eci_m_s = [0.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[telemetry]
output.csv = "out/separated-direct-torque-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const SEPARATED_LANDING_CONTROLLER_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "separated-landing-controller-test"
description = "Synthetic separated-lane terminal throttle controller regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.6
dt_s = 0.1
seed = 43

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [6371000.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.7071067811865475, 0.0, 0.7071067811865475]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "separated-landing-controller-test"

[[vehicle.assembly.bodies]]
id = "bus"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1000.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 100.0]]

[[vehicle.assembly.bodies]]
id = "booster"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1000.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 100.0]]

[[vehicle.assembly.engines]]
id = "land"
mounted_to = "booster"
kind = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, -0.5]
limits = { max_thrust_n = 30000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity", "thrust"]

[mission]
initial_phase = "coast"

[[mission.phases]]
id = "coast"
label = "coast"

[multi_body]
primary_body_id = "bus"

[[multi_body.initial_lane]]
body_id = "booster"
position_eci_m = [6371500.0, 0.0, 0.0]
velocity_eci_m_s = [-50.0, 0.0, 0.0]
quaternion_body_to_eci_xyzw = [0.0, 0.7071067811865475, 0.0, 0.7071067811865475]
angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[[multi_body.landing_controller]]
body_id = "booster"
engine_id = "land"
start_altitude_m = 1000.0
target_altitude_m = 0.0
target_vertical_speed_m_s = -5.0
gravity_margin_m_s2 = 9.80665
min_throttle_unit = 0.0
max_throttle_unit = 1.0

[telemetry]
output.csv = "out/separated-landing-controller-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const CONTACT_RIGID_BODY_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "contact-rigid-body-test"
description = "Synthetic rigid-body contact run with an initial half-space penetration."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.002
dt_s = 0.001
seed = 17

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, -0.01]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "contact-rigid-body-test"

[[vehicle.assembly.bodies]]
id = "body"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

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
output.csv = "out/contact-rigid-body-test.csv"

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

    fn has_channel(outcome: &RunOutcome, name: &str) -> bool {
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .any(|channel| channel.name == name)
    }

    fn schema_has_channel(schema: &openbmp_telemetry::TelemetrySchema, name: &str) -> bool {
        schema.channels().iter().any(|channel| channel.name == name)
    }

    fn telemetry_csv_bytes(outcome: &RunOutcome) -> Vec<u8> {
        let mut bytes = Vec::new();
        outcome
            .table
            .write_csv(&mut bytes)
            .expect("telemetry CSV should serialize");
        bytes
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

    fn i64_column(outcome: &RunOutcome, name: &str) -> Vec<i64> {
        let id = channel_id(outcome, name);
        outcome
            .table
            .rows()
            .iter()
            .map(|row| match row.get(id) {
                Some(TelemetryValue::Int64(value)) => *value,
                other => panic!("unexpected value in {name}: {other:?}"),
            })
            .collect()
    }

    #[test]
    fn rigid_body_contact_publishes_diagnostics_and_report() {
        let scenario =
            Scenario::from_toml_str(CONTACT_RIGID_BODY_SCENARIO).expect("scenario must parse");
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
            f64_column(&outcome, "contact.gap_m")[0].to_bits(),
            (-0.01_f64).to_bits()
        );
        assert!((f64_column(&outcome, "contact.normal_force_n")[0] - 20.0).abs() <= 1.0e-12);

        let report = outcome
            .contact
            .as_ref()
            .expect("contact report should be present");
        assert_eq!(report.samples as usize, outcome.table.rows().len());
        assert_eq!(report.max_penetration_m.to_bits(), 0.01_f64.to_bits());
        assert!((report.max_normal_force_n - 20.0).abs() <= 1.0e-12);
        assert_eq!(
            report.outcome,
            crate::contact::ContactOutcomeKind::Unsettled
        );
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
    fn welded_release_jettison_authority_installs_batch_release_states() {
        let toml = BATCH_RV_DEPLOY_SCENARIO
            .replace(
                "[multi_body]\n",
                "[multi_body]\nprimary_body_id = \"bus\"\npropagation_authority = \"welded_release_jettison\"\n",
            )
            .replace(
                "lower_delta_v_body_m_s = [0.0, 1.0, 0.0]\nconserve_momentum = false\n",
                "",
            )
            .replace(
                "lower_delta_v_body_m_s = [0.0, -1.0, 0.0]\nconserve_momentum = false\n",
                "",
            );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("scenario must parse");
        let resolved_files = BTreeMap::new();
        let mut session =
            RigidBodySession::prepare(&scenario, &resolved_files).expect("session prepares");

        session
            .step_once(&scenario.document, None)
            .expect("batch welded-release jettison step succeeds");

        let rv1 = body_id_from_scenario_text("rv1");
        let rv2 = body_id_from_scenario_text("rv2");
        let rv1_state = separated_lane_state(&session, rv1);
        let rv2_state = separated_lane_state(&session, rv2);
        assert_eq!(rv1_state.time.as_seconds().to_bits(), 0.1_f64.to_bits());
        assert_eq!(rv2_state.time.as_seconds().to_bits(), 0.1_f64.to_bits());
        assert_eq!(
            session.state().mass_props.mass_kg().to_bits(),
            3.0_f64.to_bits()
        );
        assert_eq!(rv1_state.mass_props.mass_kg().to_bits(), 1.0_f64.to_bits());
        assert_eq!(rv2_state.mass_props.mass_kg().to_bits(), 1.0_f64.to_bits());
        let rv1_offset = rv1_state.position.vector - session.state().position.vector;
        let rv2_offset = rv2_state.position.vector - session.state().position.vector;
        assert!((rv1_offset - Vector3::new(0.0, 0.0, -1.0)).norm() < 1.0e-12);
        assert!((rv2_offset - Vector3::new(0.0, 0.0, 1.0)).norm() < 1.0e-12);
        assert!((rv1_state.velocity.vector - session.state().velocity.vector).norm() < 1.0e-12);
        assert!((rv2_state.velocity.vector - session.state().velocity.vector).norm() < 1.0e-12);
    }

    #[test]
    fn initial_multi_body_lanes_run_from_step_zero() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(INITIAL_MULTI_BODY_SCENARIO)
            .expect("initial multi-body scenario must parse");
        let outcome = crate::run(&scenario).expect("initial multi-body scenario must run");

        let observer_active = bool_column(&outcome, "body.observer.separated");
        assert_eq!(observer_active.first().copied(), Some(true));
        assert!(observer_active.iter().all(|value| *value));
        let observer_propagating = bool_column(&outcome, "body.observer.propagating");
        assert_eq!(observer_propagating.first().copied(), Some(true));
        assert!(observer_propagating.iter().all(|value| *value));

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
    fn separated_body_retirement_is_recorded_without_clearing_lane() {
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(SEPARATED_RETIREMENT_TELEMETRY_SCENARIO)
                .expect("separated retirement telemetry scenario must parse");
        let outcome =
            crate::run(&scenario).expect("separated retirement telemetry scenario must run");

        let separated = bool_column(&outcome, "body.booster.separated");
        assert!(
            separated.iter().all(|value| *value),
            "retired lane should remain present for diagnostics: {separated:?}"
        );

        let propagating = bool_column(&outcome, "body.booster.propagating");
        assert_eq!(propagating.first().copied(), Some(true));
        assert!(
            propagating.iter().any(|value| !*value),
            "ground-crossing lane should be marked non-propagating: {propagating:?}"
        );
    }

    #[test]
    fn engine_command_to_retired_separated_body_fails_closed() {
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(RETIRED_BODY_ENGINE_COMMAND_SCENARIO)
                .expect("retired-body engine command scenario must parse");
        let err = match crate::run(&scenario) {
            Ok(_) => panic!("retired-body engine command scenario should fail"),
            Err(err) => err,
        };
        let message = err.to_string();
        assert!(
            message.contains("retired separated body"),
            "unexpected error: {message}"
        );
    }

    #[test]
    fn direct_torque_routes_to_separated_body_owner() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(SEPARATED_DIRECT_TORQUE_SCENARIO)
            .expect("separated direct-torque scenario must parse");
        let outcome = crate::run(&scenario).expect("separated direct-torque scenario must run");

        let primary_pitch_rate = f64_column(&outcome, "angular_velocity.y_rad_s");
        assert!(
            primary_pitch_rate.iter().all(|value| value.abs() < 1.0e-12),
            "booster-owned torque must not rotate the primary lane: {primary_pitch_rate:?}"
        );

        let booster_pitch_rate = f64_column(&outcome, "body.booster.angular_velocity.y_rad_s");
        assert!(
            booster_pitch_rate.iter().any(|value| *value > 0.5),
            "booster-owned torque should spin the separated lane: {booster_pitch_rate:?}"
        );

        let effector_actual = f64_column(&outcome, "effector.booster-pitch-torque.actual");
        assert!(
            effector_actual.iter().any(|value| *value > 0.0),
            "direct-torque effector command should be observable: {effector_actual:?}"
        );
    }

    #[test]
    fn separated_attitude_target_commands_owned_direct_torque_effector() {
        let toml = SEPARATED_DIRECT_TORQUE_SCENARIO
            .replace(
                "command_schedule = { kind = \"constant\", value = 1.0 }\n",
                "",
            )
            .replace(
                "primary_body_id = \"bus\"\n",
                "primary_body_id = \"bus\"\n\n[[multi_body.attitude_target]]\nbody_id = \"booster\"\npitch_effector = \"booster-pitch-torque\"\nkp = 1.0\nkd = 0.0\nmax_command = 1.0\ntarget = { kind = \"eci_vector\", vector_eci = [1.0, 0.0, 0.0] }\n\n",
            );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml)
            .expect("separated attitude-target scenario must parse");
        let outcome = crate::run(&scenario).expect("separated attitude-target scenario must run");

        let primary_pitch_rate = f64_column(&outcome, "angular_velocity.y_rad_s");
        assert!(
            primary_pitch_rate.iter().all(|value| value.abs() < 1.0e-12),
            "attitude target must not rotate the primary lane: {primary_pitch_rate:?}"
        );

        let booster_pitch_rate = f64_column(&outcome, "body.booster.angular_velocity.y_rad_s");
        assert!(
            booster_pitch_rate.iter().any(|value| *value > 0.5),
            "attitude target should spin the separated lane toward +ECI x: {booster_pitch_rate:?}"
        );

        let effector_actual = f64_column(&outcome, "effector.booster-pitch-torque.actual");
        assert!(
            effector_actual.iter().any(|value| *value > 0.0),
            "attitude target should command the direct-torque effector: {effector_actual:?}"
        );
    }

    #[test]
    fn separated_landing_controller_commands_owned_engine() {
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(SEPARATED_LANDING_CONTROLLER_SCENARIO)
                .expect("separated landing-controller scenario must parse");
        let outcome =
            crate::run(&scenario).expect("separated landing-controller scenario must run");

        let thrust = f64_column(&outcome, "engine.land.thrust_n");
        assert!(
            thrust.iter().any(|value| *value > 0.0),
            "landing controller should command terminal thrust: {thrust:?}"
        );

        let booster_separated = bool_column(&outcome, "body.booster.separated");
        assert!(
            booster_separated.iter().any(|value| *value),
            "booster lane should remain observable: {booster_separated:?}"
        );
    }

    #[test]
    fn separated_landing_controller_site_target_adds_lateral_authority() {
        let toml = SEPARATED_LANDING_CONTROLLER_SCENARIO
            .replace("max_gimbal_rad = 0.0", "max_gimbal_rad = 0.2")
            .replace(
                "max_throttle_unit = 1.0\n",
                "max_throttle_unit = 1.0\ntarget_position_eci_m = [6371000.0, 1000.0, 0.0]\nlateral_kp_s2 = 0.0002\nlateral_kd_s = 0.0\nmax_lateral_accel_m_s2 = 20.0\n",
            );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml)
            .expect("site-target landing-controller scenario must parse");
        let outcome =
            crate::run(&scenario).expect("site-target landing-controller scenario must run");

        let booster_vy = f64_column(&outcome, "body.booster.velocity_y_m_s");
        assert!(
            booster_vy.iter().any(|value| *value > 0.01),
            "site-target controller should add lateral velocity toward +ECI y: {booster_vy:?}"
        );
    }

    #[test]
    fn rigid_engine_snapshots_are_recorded() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(RIGID_ENGINE_TELEMETRY_SCENARIO)
            .expect("engine telemetry scenario must parse");
        let outcome = crate::run(&scenario).expect("engine telemetry scenario must run");

        let thrust = f64_column(&outcome, "engine.main.thrust_n");
        assert!(
            thrust.iter().any(|value| *value > 0.0),
            "engine thrust telemetry should reflect the ignited engine: {thrust:?}"
        );

        let state = i64_column(&outcome, "engine.main.state_index");
        assert!(
            state.contains(&2),
            "engine state telemetry should show Burning state: {state:?}"
        );

        let consumed = f64_column(&outcome, "engine.main.consumed_kg");
        assert!(
            consumed.windows(2).any(|pair| pair[1] > pair[0]),
            "engine consumed mass telemetry should increase during burn: {consumed:?}"
        );
    }

    #[test]
    fn rigid_body_acoustics_iop_source_engine_reports_runtime_history() {
        let toml = RIGID_ENGINE_TELEMETRY_SCENARIO
            .replace("stop_s = 0.5", "stop_s = 0.2")
            .replace(
                "limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }",
                "limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.25, shutdown_transient_s = 0.1, max_gimbal_rad = 0.0 }",
            )
            .replace("trigger = { kind = \"at_time\", time_s = 0.1 }", "trigger = { kind = \"at_time\", time_s = 0.05 }")
            .replace(
                "[telemetry]\n",
                "[acoustics.iop]\n\
                 station_id = \"station-a\"\n\
                 source_engine_id = \"main\"\n\
                 duct_volume_m3 = 120.0\n\
                 duct_exit_area_m2 = 6.0\n\
                 duct_discharge_velocity_m_s = 40.0\n\
                 duct_geometry_gain = 1.5\n\
                 receiver_distance_m = 30.0\n\
                 suppression_loss_db = 3.0\n\
                 uncertainty_minus_db = 2.0\n\
                 uncertainty_plus_db = 4.0\n\
                 \n\
                 [telemetry]\n",
            );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml)
            .expect("rigid IOP source-engine acoustics scenario must parse");
        let outcome =
            crate::run(&scenario).expect("rigid IOP source-engine acoustics scenario must run");
        let iop = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.iop.as_ref())
            .expect("IOP report emitted");

        assert_eq!(iop.source_engine_id.as_deref(), Some("main"));
        assert_eq!(
            iop.source_engine_max_thrust_n.map(f64::to_bits),
            Some(1000.0_f64.to_bits())
        );
        assert_eq!(iop.source_engine_mdot_cstar_n, None);
        assert_eq!(
            iop.source_engine_ignition_event_id.as_deref(),
            Some("ignite")
        );
        assert_eq!(
            iop.source_engine_ignition_time_s.map(f64::to_bits),
            Some(0.05_f64.to_bits())
        );
        assert_eq!(
            iop.source_engine_observed_ignition_event_id,
            Some(openbmp_sim::EventId::from_path("scenario_script.events.ignite").value())
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
            Some(400.0_f64.to_bits())
        );
        assert_eq!(
            iop.source_engine_runtime_last_time_s.map(f64::to_bits),
            Some(0.1_f64.to_bits())
        );
        let peak_rate = iop
            .source_engine_runtime_peak_source_rate_n_per_s
            .expect("runtime source-rate peak recorded");
        assert!(
            (peak_rate - 4000.0).abs() <= 1.0e-12,
            "unexpected runtime source-rate peak: {peak_rate}"
        );
        let runtime_peak = iop
            .source_engine_runtime_peak_overpressure_pa
            .expect("runtime peak pressure recorded");
        assert!(
            (runtime_peak - iop.peak_overpressure_pa).abs() <= 1.0e-12,
            "unexpected runtime peak pressure: {runtime_peak}"
        );
        let runtime_lower = iop
            .source_engine_runtime_lower_peak_overpressure_pa
            .expect("runtime lower peak pressure recorded");
        assert!(
            (runtime_lower - iop.lower_peak_overpressure_pa).abs() <= 1.0e-12,
            "unexpected runtime lower peak pressure: {runtime_lower}"
        );
        let runtime_upper = iop
            .source_engine_runtime_upper_peak_overpressure_pa
            .expect("runtime upper peak pressure recorded");
        assert!(
            (runtime_upper - iop.upper_peak_overpressure_pa).abs() <= 1.0e-12,
            "unexpected runtime upper peak pressure: {runtime_upper}"
        );
        assert_eq!(
            iop.source_engine_runtime_duration_s.map(f64::to_bits),
            Some(iop.duration_s.to_bits())
        );
        let peak_overpressure = f64_column(&outcome, "acoustics.iop.peak_overpressure_pa")[0];
        assert!(
            (peak_overpressure - iop.peak_overpressure_pa).abs() <= 1.0e-12,
            "IOP telemetry should match report peak"
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
        let source_strength = final_row[2]
            .parse::<f64>()
            .expect("source strength column parses");
        assert!(
            (source_strength - 400.0).abs() <= 1.0e-12,
            "unexpected runtime source strength: {source_strength}"
        );
        let source_rate = final_row[3]
            .parse::<f64>()
            .expect("source rate column parses");
        assert!(
            (source_rate - 4000.0).abs() <= 1.0e-12,
            "unexpected runtime source rate: {source_rate}"
        );
    }

    #[test]
    fn rigid_bending_uq_band_flows_into_upstream_budget() {
        let toml = RIGID_ENGINE_TELEMETRY_SCENARIO.replace(
            "[vehicle.assembly]\n",
            "[vehicle.bending]\n\
             frequency_hz = 1.2\n\
             damping_ratio = 0.02\n\
             modal_mass_kg = 25.0\n\
             slope_at_engine = 0.1\n\
             slope_at_gyro = 0.05\n\
             \n\
             [vehicle.bending.uq]\n\
             deck_id = \"first_mode.generic.v1\"\n\
             frequency_lower_hz = 1.0\n\
             frequency_upper_hz = 1.4\n\
             credibility_level = 2\n\
             evidence = \"docs/parity/02-structural-dynamics-loads-slosh-pogo.md#wp-02-bending\"\n\
             \n\
             [vehicle.assembly]\n",
        );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("bending UQ scenario parses");
        let outcome = crate::run(&scenario).expect("bending UQ scenario runs");

        let upstream_sources = &outcome.upstream_uq.sources;
        assert_eq!(upstream_sources.len(), 1);
        assert_eq!(
            upstream_sources[0].source_id,
            "02.structural.first_mode.generic.v1.frequency_hz"
        );
        assert_eq!(
            upstream_sources[0].class,
            openbmp_uq::UncertaintyClass::Epistemic
        );
        assert_eq!(
            upstream_sources[0].credibility.binding_level(),
            openbmp_uq::CredibilityLevel::L2
        );
        assert!(
            upstream_sources[0]
                .justification
                .contains("docs/parity/02-structural-dynamics-loads-slosh-pogo.md")
        );
        assert!((upstream_sources[0].one_sigma - 0.2).abs() < 1.0e-15);
        assert!(
            (outcome.upstream_uq.epistemic_one_sigma().unwrap() - upstream_sources[0].one_sigma)
                .abs()
                < 1.0e-15
        );
    }

    #[test]
    fn structural_line_load_recovery_channels_are_emitted() {
        let base = include_str!("../tests/fixtures/structural-rack-base.toml");
        let toml = base.replace(
            "[vehicle.assembly]\n",
            r#"[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1

[loads.recovery]

[[loads.recovery.stations]]
id = "interstage"
axial_m = 12.0

[[loads.recovery.stations.outputs]]
id = "shear_x"
displacement_coefficients = [0.0, 0.0]
acceleration_coefficients = [10.0, 5.0]
mode_acceleration_coefficients = [1.5, -0.5]

[vehicle.assembly]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml)
            .expect("structural recovery telemetry scenario parses");
        let outcome = crate::run(&scenario).expect("structural recovery telemetry scenario runs");

        assert!(has_channel(
            &outcome,
            "structural.load.interstage.shear_x_n"
        ));
        let loads = f64_column(&outcome, "structural.load.interstage.shear_x_n");
        assert_eq!(loads.len(), outcome.table.rows().len());
        assert!(loads.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn rigid_bending_consumes_acoustic_modal_force_handoff() {
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
        let toml = RIGID_ENGINE_TELEMETRY_SCENARIO
            .replace(
                "[vehicle.assembly]\n",
                "[vehicle.bending]\n\
                 frequency_hz = 125.0\n\
                 damping_ratio = 0.02\n\
                 modal_mass_kg = 25.0\n\
                 slope_at_engine = 0.1\n\
                 slope_at_gyro = 0.05\n\
                 \n\
                 [vehicle.assembly]\n",
            )
            .replace(
                "[telemetry]\n",
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

[telemetry]
"#,
                ),
            );
        let scenario = openbmp_scenario::Scenario::from_toml_str_with_source_dir(
            &toml,
            Some(workspace_root()),
        )
        .expect("rigid acoustic structural scenario parses");
        let outcome = crate::run(&scenario).expect("rigid acoustic structural scenario runs");
        let response = outcome
            .acoustics
            .as_ref()
            .and_then(|acoustics| acoustics.response.as_ref())
            .expect("acoustic response report emitted");
        let structural = outcome
            .structural_acoustics
            .as_ref()
            .expect("structural acoustic consumer report emitted");

        assert_eq!(structural.station_id, "payload-a");
        assert_eq!(
            structural.source,
            openbmp_scenario::AcousticsResponseSourceConfig::FairingInternal
        );
        assert_eq!(structural.response_class_id, "synthetic_panel_light");
        assert_eq!(structural.mode_frequency_hz.to_bits(), 125.0_f64.to_bits());
        assert_eq!(structural.mode_damping_ratio.to_bits(), 0.02_f64.to_bits());
        assert_eq!(structural.modal_mass_kg.to_bits(), 25.0_f64.to_bits());
        assert_eq!(
            structural.total_input_mean_square_force_n2.to_bits(),
            response.total_mean_square_force_n2.to_bits()
        );
        assert!(structural.total_modal_displacement_mean_square_m2 > 0.0);
        assert!(structural.total_modal_acceleration_mean_square_m2_s4 > 0.0);
        assert!(
            structural.peak_acceleration_band_center_hz.is_finite()
                && structural.peak_acceleration_band_center_hz > 0.0
        );
        assert!(structural.total_equivalent_engine_reaction_moment_mean_square_n2_m2 > 0.0);
        assert!(structural.equivalent_engine_reaction_moment_rms_n_m > 0.0);
        let csv = std::str::from_utf8(&structural.band_response_csv)
            .expect("structural acoustic response CSV is UTF-8");
        assert!(csv.contains("modal_acceleration_psd_m2_s4_per_hz"));
        assert_eq!(
            csv.lines().count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        let line_load_csv = std::str::from_utf8(&structural.line_load_recovery_csv)
            .expect("structural acoustic line-load CSV is UTF-8");
        assert!(line_load_csv.contains("first_bending_reaction"));
        assert!(line_load_csv.contains("band_equivalent_reaction_moment_mean_square_n2_m2"));
        assert_eq!(
            line_load_csv.lines().count(),
            openbmp_acoustics::STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
    }

    #[test]
    fn rigid_liquid_plume_telemetry_uses_engine_snapshots() {
        let baseline = openbmp_scenario::Scenario::from_toml_str(RIGID_ENGINE_TELEMETRY_SCENARIO)
            .expect("baseline engine telemetry scenario must parse");
        let baseline_outcome =
            crate::run(&baseline).expect("baseline engine telemetry scenario must run");
        assert!(
            !has_channel(&baseline_outcome, "plume.nozzle_pressure_ratio"),
            "rigid plume telemetry must remain opt-in"
        );

        let toml = RIGID_ENGINE_TELEMETRY_SCENARIO
            .replace(
                "initial_velocity_eci_m_s = [0.0, 0.0, 0.0]",
                "initial_velocity_eci_m_s = [0.0, 0.0, 25.0]",
            )
            .replace("atmosphere = \"none\"", "atmosphere = \"piecewise_exponential\"")
            .replace(
                "limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }\n",
                "limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }\n\
                 \n\
                 [vehicle.assembly.engines.thermochemical_performance]\n\
                 throat_area_m2 = 0.02\n\
                 exit_area_m2 = 0.24\n\
                 ambient_pressure_pa = 101325.0\n\
                 separation = \"off\"\n",
            )
            .replace(
                "[forces]\n",
                "[atmosphere]\n\
                 kind = \"piecewise_exponential\"\n\
                 \n\
                 [propulsion.thermochem]\n\
                 file = \"tests/fixtures/thermochem/synthetic-grain.toml\"\n\
                 file_sha256 = \"fa3ff43bc91d04317d44a141cf5ff28680e4e0b35d89ff3c91d92ee60f1a842b\"\n\
                 chamber_pressure_pa = 2000000.0\n\
                 mixture_ratio = 2.5\n\
                 \n\
                 [aero]\n\
                 \n\
                 [aero.plume]\n\
                 engine_count = 1\n\
                 reference_area_m2 = 1.0\n\
                 exit_area_total_m2 = 0.24\n\
                 base_area_m2 = 1.0\n\
                 center_spacing_m = 0.0\n\
                 merge_evaluation_distance_m = 0.0\n\
                 pifs_onset_angle_rad = 0.05\n\
                 \n\
                 [forces]\n",
            );
        let source_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let scenario =
            openbmp_scenario::Scenario::from_toml_str_with_source_dir(&toml, Some(source_dir))
                .expect("plume engine telemetry scenario must parse");
        let resolved_files = scenario
            .resolved_files()
            .expect("resolve thermochemistry deck");
        let outcome = run(&scenario, &resolved_files, None)
            .expect("plume engine telemetry scenario must run");
        let upstream_sources = &outcome.upstream_uq.sources;
        assert_eq!(upstream_sources.len(), 1);
        assert_eq!(
            upstream_sources[0].source_id,
            "05.propulsion.synthetic-grain.fa3ff43bc91d.c_star_efficiency"
        );
        assert_eq!(
            upstream_sources[0].class,
            openbmp_uq::UncertaintyClass::Epistemic
        );
        assert!(
            upstream_sources[0]
                .justification
                .contains("tests/fixtures/thermochem/synthetic-grain.toml#sha256=fa3ff43bc91d")
        );
        assert!(
            (outcome.upstream_uq.epistemic_one_sigma().unwrap() - upstream_sources[0].one_sigma)
                .abs()
                < 1.0e-15
        );

        let npr = f64_column(&outcome, "plume.nozzle_pressure_ratio");
        assert!(
            npr.iter().any(|value| *value > 1.0),
            "rigid plume telemetry should emit nozzle pressure ratio during burn: {npr:?}"
        );

        let thrust_coefficient = f64_column(&outcome, "plume.thrust_coefficient");
        assert!(
            thrust_coefficient.iter().any(|value| *value > 0.0),
            "rigid plume telemetry should emit thrust coefficient during burn: {thrust_coefficient:?}"
        );
    }

    #[test]
    fn rigid_plume_absent_aero_block_is_byte_identical() {
        let baseline = openbmp_scenario::Scenario::from_toml_str(RIGID_ENGINE_TELEMETRY_SCENARIO)
            .expect("baseline engine telemetry scenario must parse");
        let baseline_outcome =
            crate::run(&baseline).expect("baseline engine telemetry scenario must run");
        let empty_aero =
            openbmp_scenario::Scenario::from_toml_str(&RIGID_ENGINE_TELEMETRY_SCENARIO.replace(
                "[forces]\n",
                "[aero]\n\
                 \n\
                 [forces]\n",
            ))
            .expect("empty-aero rigid telemetry scenario must parse");
        let empty_aero_outcome =
            crate::run(&empty_aero).expect("empty-aero rigid telemetry scenario must run");

        assert!(!has_channel(
            &empty_aero_outcome,
            "plume.nozzle_pressure_ratio"
        ));
        assert_eq!(
            telemetry_csv_bytes(&baseline_outcome),
            telemetry_csv_bytes(&empty_aero_outcome)
        );
    }

    #[test]
    fn rigid_liquid_plume_derives_cluster_spacing_from_engine_mounts() {
        let limits = "limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, \
             shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }\n";
        let performance = "\n\
             [vehicle.assembly.engines.thermochemical_performance]\n\
             throat_area_m2 = 0.02\n\
             exit_area_m2 = 0.24\n\
             ambient_pressure_pa = 101325.0\n\
             separation = \"off\"\n";
        let second_engine = "\n\
             [[vehicle.assembly.engines]]\n\
             id = \"aux\"\n\
             mounted_to = \"core\"\n\
             kind = { kind = \"liquid_engine\" }\n\
             mount_point_body_m = [0.25, 0.0, 0.0]\n\
             limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }\n\
             \n\
             [vehicle.assembly.engines.thermochemical_performance]\n\
             throat_area_m2 = 0.02\n\
             exit_area_m2 = 0.24\n\
             ambient_pressure_pa = 101325.0\n\
             separation = \"off\"\n";
        let plume_config = "[atmosphere]\n\
             kind = \"piecewise_exponential\"\n\
             \n\
             [propulsion.thermochem]\n\
             file = \"tests/fixtures/thermochem/synthetic-grain.toml\"\n\
             file_sha256 = \"fa3ff43bc91d04317d44a141cf5ff28680e4e0b35d89ff3c91d92ee60f1a842b\"\n\
             chamber_pressure_pa = 2000000.0\n\
             mixture_ratio = 2.5\n\
             \n\
             [aero]\n\
             \n\
             [aero.plume]\n\
             engine_count = 2\n\
             reference_area_m2 = 1.0\n\
             exit_area_total_m2 = 0.48\n\
             base_area_m2 = 1.0\n\
             center_spacing_m = 100.0\n\
             merge_evaluation_distance_m = 0.01\n\
             pifs_onset_angle_rad = 0.0\n\
             \n\
             [forces]\n";
        let toml = RIGID_ENGINE_TELEMETRY_SCENARIO
            .replace(
                "initial_position_eci_m = [0.0, 0.0, 10.0]",
                "initial_position_eci_m = [0.0, 0.0, 50000.0]",
            )
            .replace(
                "initial_velocity_eci_m_s = [0.0, 0.0, 0.0]",
                "initial_velocity_eci_m_s = [0.0, 0.0, 25.0]",
            )
            .replace("atmosphere = \"none\"", "atmosphere = \"piecewise_exponential\"")
            .replacen(limits, &format!("{limits}{performance}{second_engine}"), 1)
            .replace("[forces]\n", plume_config)
            .replace(
                "once = true\n",
                "once = true\n\
                 \n\
                 [[scenario_script.events]]\n\
                 id = \"ignite_aux\"\n\
                 trigger = { kind = \"at_time\", time_s = 0.1 }\n\
                 action = { kind = \"engine_command\", id = \"aux\", command = { throttle_unit = 1.0, gimbal_pitch_rad = 0.0, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }\n\
                 once = true\n",
            );
        let source_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let scenario =
            openbmp_scenario::Scenario::from_toml_str_with_source_dir(&toml, Some(source_dir))
                .expect("multi-engine plume scenario must parse");
        let resolved_files = scenario
            .resolved_files()
            .expect("resolve thermochemistry deck");
        let document = &scenario.document;
        let engine_rack = crate::engines::EngineRack::build(document, &resolved_files)
            .expect("multi-engine plume rack must build");
        assert_eq!(engine_rack.len(), 2);
        let evaluator = crate::plume::RigidPlumeEvaluator::maybe_new(
            document,
            engine_rack.liquid_plume_metadata(),
            engine_rack.engine_ids(),
            engine_rack.mount_points_body(),
        )
        .expect("multi-engine plume evaluator should build")
        .expect("plume evaluator should be enabled");
        let state = RigidBodyState::new(
            SimTime::from_seconds(0.2),
            Position3::<openbmp_core::Eci>::new(0.0, 0.0, 50_000.0),
            Velocity3::<openbmp_core::Eci>::new(0.0, 0.0, 25.0),
            Quaternion::<Body, openbmp_core::Eci>::from_unit_quaternion(
                nalgebra::UnitQuaternion::identity(),
            ),
            AngularVelocity3::<Body>::zero(),
            MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(10_000.0),
                Position3::<Body>::origin(),
                1_000_000.0,
            ),
        );
        let engine_snapshot = BTreeMap::from([
            (
                EngineId::from_path("vehicle.assembly.engines.main"),
                openbmp_sim::EngineSnapshot {
                    thrust_body: Vector3::new(0.0, 0.0, 1000.0),
                    mass_flow_kg_per_s: 25.0,
                    consumed_kg: 0.0,
                    gimbal_pitch_rad: 0.0,
                    gimbal_yaw_rad: 0.0,
                    gimbal_pitch_rate_rad_s: 0.0,
                    gimbal_yaw_rate_rad_s: 0.0,
                    tvc_pitch_load_torque_n_m: 0.0,
                    tvc_yaw_load_torque_n_m: 0.0,
                    tvc_pitch_inertial_reaction_torque_n_m: 0.0,
                    tvc_yaw_inertial_reaction_torque_n_m: 0.0,
                    lifecycle_state_index: 2,
                },
            ),
            (
                EngineId::from_path("vehicle.assembly.engines.aux"),
                openbmp_sim::EngineSnapshot {
                    thrust_body: Vector3::new(0.0, 0.0, 1000.0),
                    mass_flow_kg_per_s: 25.0,
                    consumed_kg: 0.0,
                    gimbal_pitch_rad: 0.0,
                    gimbal_yaw_rad: 0.0,
                    gimbal_pitch_rate_rad_s: 0.0,
                    gimbal_yaw_rate_rad_s: 0.0,
                    tvc_pitch_load_torque_n_m: 0.0,
                    tvc_yaw_load_torque_n_m: 0.0,
                    tvc_pitch_inertial_reaction_torque_n_m: 0.0,
                    tvc_yaw_inertial_reaction_torque_n_m: 0.0,
                    lifecycle_state_index: 2,
                },
            ),
        ]);
        let atmosphere = openbmp_physics::AtmosphereSample::new(0.001, 50.0, 250.0, 300.0).unwrap();
        let plume = evaluator
            .evaluate(&state, Some(atmosphere), &engine_snapshot)
            .expect("plume evaluator should run")
            .expect("active engines should produce plume state");

        assert!(
            plume.initial_turn_angle_rad > 0.0,
            "controlled atmosphere should produce an underexpanded plume: {plume:?}"
        );
        assert!(
            plume.cluster_merged,
            "cluster merge should use active engine mount spacing, not the oversized configured spacing: {plume:?}"
        );
        assert_eq!(
            plume.merge_distance_m.map(f64::to_bits),
            Some(0.0_f64.to_bits())
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
            recovery_apparent_mass_ids: Vec::new(),
            recovery_owners: BTreeMap::new(),
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
                    gimbal_pitch_rad: 0.0,
                    gimbal_yaw_rad: 0.0,
                    gimbal_pitch_rate_rad_s: 0.0,
                    gimbal_yaw_rate_rad_s: 0.0,
                    tvc_pitch_load_torque_n_m: 0.0,
                    tvc_yaw_load_torque_n_m: 0.0,
                    tvc_pitch_inertial_reaction_torque_n_m: 0.0,
                    tvc_yaw_inertial_reaction_torque_n_m: 0.0,
                    lifecycle_state_index: 2,
                },
            ),
            (
                engine_lower,
                openbmp_sim::EngineSnapshot {
                    thrust_body: Vector3::zeros(),
                    mass_flow_kg_per_s: 2.0,
                    consumed_kg: 0.75,
                    gimbal_pitch_rad: 0.0,
                    gimbal_yaw_rad: 0.0,
                    gimbal_pitch_rate_rad_s: 0.0,
                    gimbal_yaw_rate_rad_s: 0.0,
                    tvc_pitch_load_torque_n_m: 0.0,
                    tvc_yaw_load_torque_n_m: 0.0,
                    tvc_pitch_inertial_reaction_torque_n_m: 0.0,
                    tvc_yaw_inertial_reaction_torque_n_m: 0.0,
                    lifecycle_state_index: 2,
                },
            ),
        ]);
        let tanks = BTreeMap::new();
        let recovery = BTreeMap::new();

        let upper_props = model
            .mass_properties_from_snapshots(
                SimTime::ZERO,
                Some(upper),
                &engine_snapshot,
                &tanks,
                &recovery,
            )
            .unwrap();
        let lower_props = model
            .mass_properties_from_snapshots(
                SimTime::ZERO,
                Some(lower),
                &engine_snapshot,
                &tanks,
                &recovery,
            )
            .unwrap();
        let all_props = model
            .mass_properties_from_snapshots(
                SimTime::ZERO,
                None,
                &engine_snapshot,
                &tanks,
                &recovery,
            )
            .unwrap();
        assert!((upper_props.mass_kg() - 3.75).abs() < 1.0e-12);
        assert!((lower_props.mass_kg() - 0.25).abs() < 1.0e-12);
        assert!((all_props.mass_kg() - 4.0).abs() < 1.0e-12);

        let upper_rate = model
            .mass_properties_rate_from_snapshots(
                SimTime::ZERO,
                Some(upper),
                &engine_snapshot,
                &tanks,
                &recovery,
            )
            .unwrap();
        let lower_rate = model
            .mass_properties_rate_from_snapshots(
                SimTime::ZERO,
                Some(lower),
                &engine_snapshot,
                &tanks,
                &recovery,
            )
            .unwrap();
        let all_rate = model
            .mass_properties_rate_from_snapshots(
                SimTime::ZERO,
                None,
                &engine_snapshot,
                &tanks,
                &recovery,
            )
            .unwrap();
        assert_eq!(upper_rate.mass_rate_kg_s.to_bits(), (-1.0_f64).to_bits());
        assert_eq!(lower_rate.mass_rate_kg_s.to_bits(), (-2.0_f64).to_bits());
        assert_eq!(all_rate.mass_rate_kg_s.to_bits(), (-3.0_f64).to_bits());
    }

    #[test]
    fn rigid_recovery_apparent_mass_telemetry_channels_are_allocated() {
        let toml = RIGID_ENGINE_TELEMETRY_SCENARIO.replace(
            "[[vehicle.assembly.engines]]",
            r#"[[vehicle.assembly.recovery]]
id = "main_chute"
mounted_to = "core"
kind = { kind = "parachute_drag", c_d = 1.5, area_inflated_m2 = 8.0 }
inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = "polynomial", exponent_j = 1.0, apparent_mass_coeff_k_a = 0.65 }

[[vehicle.assembly.engines]]"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml)
            .expect("rigid apparent-mass recovery scenario parses");
        let schema = RigidChannelSet::new(&scenario.document)
            .expect("rigid channels build")
            .schema(BTreeMap::new())
            .expect("rigid schema builds");

        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.apparent_mass_kg"
        ));
        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.apparent_mass_rate_kg_s"
        ));
    }

    #[test]
    fn rigid_recovery_riser_telemetry_channels_are_allocated() {
        let toml = RIGID_ENGINE_TELEMETRY_SCENARIO.replace(
            "[[vehicle.assembly.engines]]",
            r#"[[vehicle.assembly.recovery]]
id = "main_chute"
mounted_to = "core"
kind = { kind = "parachute_drag", c_d = 1.5, area_inflated_m2 = 8.0 }
risers = [{ id = "main_riser", unstretched_length_m = 0.5, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 1.0, 0.0], canopy_attach_body_m = [1.0, 1.0, 0.0] }]

[[vehicle.assembly.engines]]"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml)
            .expect("rigid riser recovery scenario parses");
        let schema = RigidChannelSet::new(&scenario.document)
            .expect("rigid channels build")
            .schema(BTreeMap::new())
            .expect("rigid schema builds");

        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.riser_tension_sum_n"
        ));
        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.riser_force_body_x_n"
        ));
        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.riser_moment_body_z_n_m"
        ));
        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.riser_elastic_energy_j"
        ));
    }

    #[test]
    fn rigid_recovery_sequence_snatch_telemetry_channels_are_allocated() {
        let toml = RIGID_ENGINE_TELEMETRY_SCENARIO.replace(
            "[[vehicle.assembly.engines]]",
            r#"[[vehicle.assembly.recovery]]
id = "main_chute"
mounted_to = "core"
kind = { kind = "parachute_drag", c_d = 1.5, area_inflated_m2 = 8.0 }
sequence = { snatch_delta_v_m_s = 3.0, snatch_effective_mass_kg = 5.0, pack_drag_load_n = 4.0, mortar_pack_mass_kg = 2.0, mortar_pack_velocity_body_m_s = [0.0, 0.0, 6.0] }
risers = [{ id = "main_riser", unstretched_length_m = 0.5, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 1.0, 0.0], canopy_attach_body_m = [1.0, 1.0, 0.0] }]

[[vehicle.assembly.engines]]"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml)
            .expect("rigid sequence recovery scenario parses");
        let schema = RigidChannelSet::new(&scenario.document)
            .expect("rigid channels build")
            .schema(BTreeMap::new())
            .expect("rigid schema builds");

        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.snatch_delta_v_m_s"
        ));
        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.snatch_stiffness_n_m"
        ));
        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.snatch_peak_load_n"
        ));
        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.mortar_impulse_body_z_n_s"
        ));
        assert!(schema_has_channel(
            &schema,
            "recovery.main_chute.mortar_force_body_z_n"
        ));
        assert!(schema_has_channel(
            &schema,
            "force.recovery_sequence_force.z_n"
        ));
    }

    #[test]
    fn rigid_moment_stack_consumes_recovery_riser_snapshot() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(
            r#"
openbmp.scenario = 3

[meta]
name = "rigid-recovery-riser-moment-test"
description = "Synthetic rigid recovery-riser moment test."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 43

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "rigid-recovery-riser-moment-test"

[[vehicle.assembly.bodies]]
id = "core"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 10.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.recovery]]
id = "main_chute"
mounted_to = "core"
kind = { kind = "parachute_drag", c_d = 1.5, area_inflated_m2 = 8.0 }
risers = [{ id = "main_riser", unstretched_length_m = 0.5, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 1.0, 0.0], canopy_attach_body_m = [1.0, 1.0, 0.0] }]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/rigid-recovery-riser-moment-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#,
        )
        .expect("rigid recovery-riser moment scenario parses");
        let moment_vehicle = build_moment_model(&scenario.document, &LoadedModels::default(), None)
            .expect("moment vehicle builds");
        let state = RigidBodyState::new(
            SimTime::ZERO,
            Position3::<openbmp_core::Eci>::new(0.0, 0.0, 10.0),
            Velocity3::<openbmp_core::Eci>::zero(),
            Quaternion::<Body, openbmp_core::Eci>::from_unit_quaternion(
                nalgebra::UnitQuaternion::identity(),
            ),
            AngularVelocity3::<Body>::zero(),
            MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(10.0),
                Position3::<Body>::origin(),
                1.0,
            ),
        );
        let id = RecoveryId::from_path("vehicle.assembly.recovery.main_chute");
        let recovery_snapshot = BTreeMap::from([(
            id,
            openbmp_sim::RecoverySnapshot {
                riser_force_body_n: Vector3::new(200.0, 0.0, 0.0),
                riser_moment_body_n_m: Vector3::new(0.0, 0.0, -200.0),
                ..openbmp_sim::RecoverySnapshot::default()
            },
        )]);
        let env = openbmp_sim::EnvironmentSample::default();
        let breakdown = moment_vehicle
            .evaluate_moment_breakdown(MomentContext {
                state: &state,
                environment: &env,
                time: SimTime::ZERO,
                active_body: None,
                phase_id: None,
                effector_actuals: EffectorActualsView::empty(),
                engine_snapshot: EngineSnapshotView::empty(),
                tank_snapshot: TankSnapshotView::empty(),
                recovery_snapshot: RecoverySnapshotView::new(&recovery_snapshot),
            })
            .expect("moment breakdown evaluates");

        let recovery_component = breakdown
            .components
            .iter()
            .find(|(name, _)| name == "recovery_riser")
            .expect("recovery riser moment component present")
            .1;
        assert_eq!(recovery_component.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(recovery_component.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(recovery_component.z.to_bits(), (-200.0_f64).to_bits());
        assert_eq!(breakdown.total.z.to_bits(), (-200.0_f64).to_bits());
    }

    #[test]
    fn rigid_force_stack_consumes_recovery_riser_snapshot() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(
            r#"
openbmp.scenario = 3

[meta]
name = "rigid-recovery-riser-force-test"
description = "Synthetic rigid recovery-riser force test."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 44

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "rigid-recovery-riser-force-test"

[[vehicle.assembly.bodies]]
id = "core"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 10.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.recovery]]
id = "main_chute"
mounted_to = "core"
kind = { kind = "parachute_drag", c_d = 1.5, area_inflated_m2 = 8.0 }
risers = [{ id = "main_riser", unstretched_length_m = 0.5, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 1.0, 0.0], canopy_attach_body_m = [1.0, 1.0, 0.0] }]

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
output.csv = "out/rigid-recovery-riser-force-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#,
        )
        .expect("rigid recovery-riser force scenario parses");
        let assembly =
            crate::assembly::synthesize_assembly(&scenario.document).expect("assembly builds");
        let force_vehicle = build_vehicle(
            &scenario.document,
            &LoadedModels::default(),
            &assembly,
            &BTreeMap::new(),
            None,
            None,
            None,
        )
        .expect("force vehicle builds");
        let body_to_eci = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
            core::f64::consts::FRAC_PI_2,
        );
        let state = RigidBodyState::new(
            SimTime::ZERO,
            Position3::<openbmp_core::Eci>::new(0.0, 0.0, 10.0),
            Velocity3::<openbmp_core::Eci>::zero(),
            Quaternion::<Body, openbmp_core::Eci>::from_unit_quaternion(body_to_eci),
            AngularVelocity3::<Body>::zero(),
            MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(10.0),
                Position3::<Body>::origin(),
                1.0,
            ),
        );
        let id = RecoveryId::from_path("vehicle.assembly.recovery.main_chute");
        let recovery_snapshot = BTreeMap::from([(
            id,
            openbmp_sim::RecoverySnapshot {
                riser_force_body_n: Vector3::new(200.0, 0.0, 0.0),
                ..openbmp_sim::RecoverySnapshot::default()
            },
        )]);
        let env = openbmp_sim::EnvironmentSample::default();
        let breakdown = force_vehicle
            .evaluate_force_breakdown(ForceContext {
                state: &state,
                environment: &env,
                mass_kg: 10.0,
                time: SimTime::ZERO,
                active_body: None,
                phase_id: None,
                effector_actuals: EffectorActualsView::empty(),
                engine_snapshot: EngineSnapshotView::empty(),
                tank_snapshot: TankSnapshotView::empty(),
                recovery_snapshot: RecoverySnapshotView::new(&recovery_snapshot),
            })
            .expect("force breakdown evaluates");

        let recovery_component = breakdown
            .components
            .iter()
            .find(|(name, _)| name == "recovery_riser_force")
            .expect("recovery riser force component present")
            .1;
        assert!(
            recovery_component.x.abs() <= 1.0e-12,
            "force.x = {}",
            recovery_component.x
        );
        assert!(
            (recovery_component.y - 200.0).abs() <= 1.0e-12,
            "force.y = {}",
            recovery_component.y
        );
        assert!(
            recovery_component.z.abs() <= 1.0e-12,
            "force.z = {}",
            recovery_component.z
        );
        assert!((breakdown.total.y - 200.0).abs() <= 1.0e-12);
    }

    #[test]
    fn rigid_mass_resources_add_recovery_apparent_mass_and_rate() {
        let body = body_id_from_scenario_text("core");
        let recovery = RecoveryId::from_path("vehicle.assembly.recovery.main_chute");
        let resources = RigidMassResources {
            dry_total: MassProperties::with_uniform_inertia(
                Mass::new::<kilogram>(20.0),
                Position3::<Body>::origin(),
                1.0,
            ),
            dry_bodies: BTreeMap::from([(
                body,
                MassProperties::with_uniform_inertia(
                    Mass::new::<kilogram>(20.0),
                    Position3::<Body>::origin(),
                    1.0,
                ),
            )]),
            motor_owner: None,
            motor_ignition_time_s: None,
            engine_ids: Vec::new(),
            engine_owners: BTreeMap::new(),
            tank_coupled_engines: std::collections::BTreeSet::new(),
            tank_routes: BTreeMap::new(),
            recovery_apparent_mass_ids: vec![recovery],
            recovery_owners: BTreeMap::from([(recovery, body)]),
        };
        let model =
            RigidMassResourceModel::<SolidMotor>::new(resources, None, None, ModelId::new(999));
        let recovery_snapshot = BTreeMap::from([(
            recovery,
            openbmp_sim::RecoverySnapshot {
                apparent_mass_kg: 3.5,
                apparent_mass_rate_kg_s: 0.25,
                ..openbmp_sim::RecoverySnapshot::default()
            },
        )]);

        let props = model
            .mass_properties_from_snapshots(
                SimTime::ZERO,
                Some(body),
                &BTreeMap::new(),
                &BTreeMap::new(),
                &recovery_snapshot,
            )
            .unwrap();
        let rate = model
            .mass_properties_rate_from_snapshots(
                SimTime::ZERO,
                Some(body),
                &BTreeMap::new(),
                &BTreeMap::new(),
                &recovery_snapshot,
            )
            .unwrap();

        assert_eq!(props.mass_kg().to_bits(), 23.5_f64.to_bits());
        assert_eq!(rate.mass_rate_kg_s.to_bits(), 0.25_f64.to_bits());
    }

    #[test]
    fn tank_coupled_engine_mass_is_not_double_counted() {
        // An engine that draws from a tank must NOT also subtract its
        // integrated consumed_kg in the absolute mass path — the tank's
        // mass_kg already reflects depletion. The tank rate includes
        // engine feed and other drains; it must also be counted once.
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
                    mount_point_tank_body_m: Vector3::zeros(),
                    rot_owner_from_tank_body: Matrix3::identity(),
                    translation_owner_from_tank_body_m: Vector3::zeros(),
                },
            )]),
            recovery_apparent_mass_ids: Vec::new(),
            recovery_owners: BTreeMap::new(),
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
                gimbal_pitch_rad: 0.0,
                gimbal_yaw_rad: 0.0,
                gimbal_pitch_rate_rad_s: 0.0,
                gimbal_yaw_rate_rad_s: 0.0,
                tvc_pitch_load_torque_n_m: 0.0,
                tvc_yaw_load_torque_n_m: 0.0,
                tvc_pitch_inertial_reaction_torque_n_m: 0.0,
                tvc_yaw_inertial_reaction_torque_n_m: 0.0,
                lifecycle_state_index: 2,
            },
        )]);
        let mut tank_snapshot = BTreeMap::from([(
            tank,
            openbmp_sim::TankSnapshot {
                mass_kg: 70.0,
                cg_offset_body_m: Vector3::zeros(),
                inertia_delta_body_kg_m2: nalgebra::Matrix3::zeros(),
                reaction_force_body_n: Vector3::zeros(),
                reaction_moment_body_n_m: Vector3::zeros(),
                fluid_remaining_kg: 70.0,
                mass_rate_kg_s: -3.0,
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
                &BTreeMap::new(),
            )
            .unwrap();
        assert!((props.mass_kg() - 90.0).abs() < 1.0e-9);

        // Rate tracks the tank drain once, not engine + tank (-6 kg/s).
        let rate = model
            .mass_properties_rate_from_snapshots(
                SimTime::ZERO,
                Some(body),
                &engine_snapshot,
                &tank_snapshot,
                &BTreeMap::new(),
            )
            .unwrap();
        assert_eq!(rate.mass_rate_kg_s.to_bits(), (-3.0_f64).to_bits());
        // Additional vent loss belongs to the tank; a zero drain must not
        // reuse an old engine flow. The rate matches finite-differenced mass.
        for drain in [-5.0_f64, 0.0] {
            tank_snapshot.get_mut(&tank).unwrap().mass_rate_kg_s = drain;
            let rate = model
                .mass_properties_rate_from_snapshots(
                    SimTime::ZERO,
                    Some(body),
                    &engine_snapshot,
                    &tank_snapshot,
                    &BTreeMap::new(),
                )
                .unwrap();
            tank_snapshot.get_mut(&tank).unwrap().mass_kg = 70.0 + drain;
            let next = model
                .mass_properties_from_snapshots(
                    SimTime::from_seconds(1.0),
                    Some(body),
                    &engine_snapshot,
                    &tank_snapshot,
                    &BTreeMap::new(),
                )
                .unwrap();
            assert_eq!(rate.mass_rate_kg_s.to_bits(), drain.to_bits());
            assert!((next.mass_kg() - props.mass_kg() - drain).abs() < 1.0e-12);
        }
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
            plume: None,
            uq: None,
            run_matrix: None,
            deck_draw: None,
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
