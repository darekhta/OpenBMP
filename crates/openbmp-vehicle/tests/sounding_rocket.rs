//! Phase 2.9 — sounding-rocket validation case.
//!
//! Vertical-launch model rocket with the real Estes D12 motor
//! (`data/motors/estes-d12-eng-derived.toml`), the synthetic
//! D12-class aero deck (`data/aero/synthetic-d12-class-rocket.toml`),
//! USSA76 atmosphere, and ECI-`+z` constant gravity. The kernel
//! integrates the point-mass equations of motion and the test
//! computes apogee, max velocity, and max acceleration from the
//! trace.
//!
//! # Niskanen 2009 reference
//!
//! The public OpenRocket technical-documentation text derived from
//! Niskanen 2009 Chapter 6 gives the small-rocket geometry and Table
//! 6.1 apogees. The full component mass/CG override table is not
//! published in that extract, so the benchmark below is a reduced
//! point-mass reconstruction: published 56 cm × 29 mm geometry,
//! published C6-family apogee, an Estes C6 RASP thrust-curve shape
//! scaled to Niskanen's published 7.5 N·s C6 impulse, and explicit
//! airframe/CD assumptions encoded as constants.

#![allow(missing_docs)]
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]

use nalgebra::Vector3;
use openbmp_aero::AeroDeck;
use openbmp_core::{Duration, ModelId, Position3, SimTime, Velocity3};
use openbmp_env::{IsothermalAtmosphere, UsStandard1976};
use openbmp_propulsion::{Motor, SolidMotor};
use openbmp_sim::{
    ConstantGravityForce, EndTime, EnvironmentModel, ForceModel, Integrator, MassModel,
    NullEnvironment, Rk4FixedStep, SimulationConfig, SimulationKernel, StopCondition,
};
use openbmp_state::PointMassState;
use openbmp_vehicle::{
    AxialDragForceAdapter, BasicVehicle, MotorMassAdapter, MotorThrustForceAdapter, NamedForceModel,
};
use serde::Deserialize;
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

const D12_DECK: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/motors/estes-d12-eng-derived.toml"
));

const D12_AERO_DECK: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/aero/synthetic-d12-class-rocket.toml"
));

const NISKANEN_C6_MOTOR_DECK: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/motors/estes-c6-eng-derived.toml"
));

const NISKANEN_AERO_DECK: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/aero/synthetic-niskanen-ch6-rocket.toml"
));

const NISKANEN_SCENARIO: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/scenarios/niskanen-2009-chapter6.toml"
));

// Vehicle parameters for the canonical D12 sounding-rocket case.
// 70 g dry-airframe mass matches the lower end of the Estes
// Alpha-III / Big-Bertha class small rockets that the D12 motor
// is most commonly used in.
const DRY_VEHICLE_MASS_KG: f64 = 0.070;
const IGNITION_TIME_S: f64 = 0.0;
const G_M_S2: f64 = 9.80665;
const DT_S: f64 = 0.001; // 1 ms — fine enough for D12 burn dynamics
const STOP_S: f64 = 20.0; // covers ascent + true post-apogee descent

const D12_DRAG_MODEL_ID: ModelId = ModelId::new(1);
const D12_THRUST_MODEL_ID: ModelId = ModelId::new(2);
const D12_MASS_MODEL_ID: ModelId = ModelId::new(3);
const NISKANEN_DRAG_MODEL_ID: ModelId = ModelId::new(11);
const NISKANEN_THRUST_MODEL_ID: ModelId = ModelId::new(12);
const NISKANEN_MASS_MODEL_ID: ModelId = ModelId::new(13);

const D12_PINNED_FINAL_STATE_HASH: u64 = 18_411_426_573_570_814_820;
const D12_PINNED_APOGEE_ALTITUDE_M: f64 = 497.837_685_347_295_23;
const D12_PINNED_APOGEE_TIME_S: f64 = 9.421;
const D12_PINNED_MAX_VELOCITY_M_S: f64 = 127.932_725_498_234_65;
const D12_PINNED_MAX_ACCELERATION_M_S2: f64 = 263.440_645_283_683_47;
const NISKANEN_PINNED_FINAL_STATE_HASH: u64 = 9_201_412_638_053_891_872;
const NISKANEN_PINNED_APOGEE_ALTITUDE_M: f64 = 150.844_023_363_995_77;
const NISKANEN_PINNED_APOGEE_TIME_S: f64 = 6.042;
const NISKANEN_PINNED_MAX_VELOCITY_M_S: f64 = 52.293_275_581_836_824;
const NISKANEN_PINNED_MAX_ACCELERATION_M_S2: f64 = 107.552_665_036_573_46;

// Niskanen Chapter-6 vehicle parameters, simulation settings, and
// reference apogees live in `data/scenarios/niskanen-2009-chapter6.toml`.
// The `niskanen_chapter6_scenario_file_matches_published_table` test
// asserts every parsed value bit-exactly matches the Chapter-6
// published source.

fn load_d12_motor() -> SolidMotor {
    SolidMotor::load_from_str(D12_DECK).expect("D12 motor deck must parse")
}

fn load_niskanen_c6_motor() -> SolidMotor {
    SolidMotor::load_from_str(NISKANEN_C6_MOTOR_DECK).expect("C6 motor deck must parse")
}

fn load_d12_aero_deck() -> AeroDeck {
    AeroDeck::load_from_str(D12_AERO_DECK).expect("D12 aero deck must parse")
}

fn load_niskanen_aero_deck() -> AeroDeck {
    AeroDeck::load_from_str(NISKANEN_AERO_DECK)
        .expect("Niskanen Chapter 6 reduced aero deck must parse")
}

// ---------------------------------------------------------------------
// Niskanen Chapter 6 scenario file (TOML) — minimal benchmark schema
// ---------------------------------------------------------------------

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct NiskanenScenarioFile {
    openbmp: NiskanenSchemaMarker,
    #[allow(dead_code)] // documented in provenance.md, not consumed at runtime.
    meta: NiskanenMeta,
    #[allow(dead_code)] // resolved by the Phase-2.10 scenario runner; the
    // Phase-2.9 test loads the referenced files via include_str! directly.
    references: NiskanenReferences,
    vehicle: NiskanenVehicle,
    simulation: NiskanenSimulation,
    reference_apogees_m: NiskanenReferenceApogees,
    tolerance: NiskanenTolerance,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct NiskanenSchemaMarker {
    benchmark: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct NiskanenMeta {
    #[allow(dead_code)]
    name: String,
    #[allow(dead_code)]
    provenance: String,
    #[allow(dead_code)]
    validation: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct NiskanenReferences {
    #[allow(dead_code)]
    motor_file: String,
    #[allow(dead_code)]
    aero_file: String,
}

#[derive(Deserialize, Copy, Clone)]
#[serde(deny_unknown_fields)]
struct NiskanenVehicle {
    body_length_m: f64,
    body_diameter_m: f64,
    nose_length_m: f64,
    dry_vehicle_mass_kg: f64,
    axial_cd: f64,
}

#[derive(Deserialize, Copy, Clone)]
#[serde(deny_unknown_fields)]
#[allow(clippy::struct_field_names)] // physics fields carry their unit suffix per project convention.
struct NiskanenSimulation {
    ignition_time_s: f64,
    dt_s: f64,
    stop_s: f64,
}

#[derive(Deserialize, Copy, Clone)]
#[serde(deny_unknown_fields)]
struct NiskanenReferenceApogees {
    b4_experimental: f64,
    b4_openrocket: f64,
    b4_rocksim: f64,
    c6_experimental: f64,
    c6_openrocket: f64,
    c6_rocksim: f64,
}

#[derive(Deserialize, Copy, Clone)]
#[serde(deny_unknown_fields)]
struct NiskanenTolerance {
    apogee_relative: f64,
}

fn load_niskanen_scenario() -> NiskanenScenarioFile {
    let parsed: NiskanenScenarioFile =
        toml::from_str(NISKANEN_SCENARIO).expect("Niskanen scenario file must parse");
    assert_eq!(parsed.openbmp.benchmark, 1);
    parsed
}

fn build_initial_state() -> PointMassState {
    let motor = load_d12_motor();
    let initial_total_mass = DRY_VEHICLE_MASS_KG + motor.dry_mass_kg() + motor.propellant_mass_kg();
    PointMassState::new(
        SimTime::ZERO,
        Position3::new(0.0, 0.0, 0.0), // launch from sea level on z-axis
        Velocity3::new(0.0, 0.0, 0.0),
        Mass::new::<kilogram>(initial_total_mass),
    )
}

fn build_d12_vehicle() -> BasicVehicle<PointMassState> {
    let motor = load_d12_motor();
    let deck = load_d12_aero_deck();
    let atmosphere = UsStandard1976::new();

    // Three force models in declared order: gravity, drag, thrust.
    let gravity = ConstantGravityForce::new(Vector3::new(0.0, 0.0, -G_M_S2));
    let drag = AxialDragForceAdapter::new(deck, atmosphere, D12_DRAG_MODEL_ID);
    let thrust = MotorThrustForceAdapter::new(motor, IGNITION_TIME_S, D12_THRUST_MODEL_ID);

    BasicVehicle::new(
        vec![
            NamedForceModel::new("gravity", Box::new(gravity)),
            NamedForceModel::new("drag", Box::new(drag)),
            NamedForceModel::new("thrust", Box::new(thrust)),
        ],
        vec![],
        Box::new(MotorMassAdapter::new(
            load_d12_motor(),
            DRY_VEHICLE_MASS_KG,
            IGNITION_TIME_S,
            D12_MASS_MODEL_ID,
        )),
    )
    .expect("vehicle construction must succeed")
}

fn build_niskanen_initial_state(scenario: &NiskanenScenarioFile) -> PointMassState {
    let motor = load_niskanen_c6_motor();
    let initial_total_mass =
        scenario.vehicle.dry_vehicle_mass_kg + motor.dry_mass_kg() + motor.propellant_mass_kg();
    PointMassState::new(
        SimTime::ZERO,
        Position3::new(0.0, 0.0, 0.0),
        Velocity3::new(0.0, 0.0, 0.0),
        Mass::new::<kilogram>(initial_total_mass),
    )
}

fn build_niskanen_chapter6_vehicle(
    scenario: &NiskanenScenarioFile,
) -> BasicVehicle<PointMassState> {
    let motor = load_niskanen_c6_motor();
    let deck = load_niskanen_aero_deck();
    let atmosphere = UsStandard1976::new();

    let gravity = ConstantGravityForce::new(Vector3::new(0.0, 0.0, -G_M_S2));
    let drag = AxialDragForceAdapter::new(deck, atmosphere, NISKANEN_DRAG_MODEL_ID);
    let thrust = MotorThrustForceAdapter::new(
        motor,
        scenario.simulation.ignition_time_s,
        NISKANEN_THRUST_MODEL_ID,
    );

    BasicVehicle::new(
        vec![
            NamedForceModel::new("gravity", Box::new(gravity)),
            NamedForceModel::new("drag", Box::new(drag)),
            NamedForceModel::new("thrust", Box::new(thrust)),
        ],
        vec![],
        Box::new(MotorMassAdapter::new(
            load_niskanen_c6_motor(),
            scenario.vehicle.dry_vehicle_mass_kg,
            scenario.simulation.ignition_time_s,
            NISKANEN_MASS_MODEL_ID,
        )),
    )
    .expect("Niskanen vehicle construction must succeed")
}

#[derive(Copy, Clone, Debug)]
struct TraceMetrics {
    apogee_altitude_m: f64,
    apogee_time_s: f64,
    max_velocity_m_s: f64,
    max_acceleration_m_s2: f64,
    final_velocity_z_m_s: f64,
    final_state_byte_hash: u64,
}

fn collect_trace<I, F, MM, E, SC>(
    kernel: &mut SimulationKernel<PointMassState, I, F, MM, E, SC>,
    dt_s: f64,
) -> TraceMetrics
where
    I: Integrator<PointMassState>,
    F: ForceModel<PointMassState>,
    MM: MassModel,
    E: EnvironmentModel,
    SC: StopCondition<PointMassState>,
{
    // Step the kernel, recording the per-step state for trace metrics.
    let mut max_altitude_m = 0.0_f64;
    let mut apogee_time_s = 0.0_f64;
    let mut max_velocity_m_s = 0.0_f64;
    let mut max_accel_m_s2 = 0.0_f64;
    let mut prev_velocity_z: f64 = 0.0;

    while kernel.stop_reason().is_none() {
        kernel.step().expect("step must succeed");
        let s = kernel.current_state();
        let altitude = s.position.vector.z;
        let velocity_z = s.velocity.vector.z;
        let speed = velocity_z.abs(); // 1-D vertical motion

        if altitude > max_altitude_m {
            max_altitude_m = altitude;
            apogee_time_s = s.time.as_seconds();
        }
        if speed > max_velocity_m_s {
            max_velocity_m_s = speed;
        }
        // Numerical derivative of velocity for max acceleration.
        let accel = (velocity_z - prev_velocity_z).abs() / dt_s;
        if accel > max_accel_m_s2 {
            max_accel_m_s2 = accel;
        }
        prev_velocity_z = velocity_z;
    }

    let final_state = kernel.current_state();
    // Simple deterministic hash of the final state for the
    // byte-stability assertion below (XOR of the f64 bit patterns).
    let final_state_byte_hash = final_state.time.as_seconds().to_bits()
        ^ final_state.position.vector.z.to_bits()
        ^ final_state.velocity.vector.z.to_bits()
        ^ final_state.mass.get::<kilogram>().to_bits();

    TraceMetrics {
        apogee_altitude_m: max_altitude_m,
        apogee_time_s,
        max_velocity_m_s,
        max_acceleration_m_s2: max_accel_m_s2,
        final_velocity_z_m_s: final_state.velocity.vector.z,
        final_state_byte_hash,
    }
}

fn run_d12_scenario_and_collect_trace() -> TraceMetrics {
    let vehicle = build_d12_vehicle();
    let mass_model = MotorMassAdapter::new(
        load_d12_motor(),
        DRY_VEHICLE_MASS_KG,
        IGNITION_TIME_S,
        D12_MASS_MODEL_ID,
    );
    assert_eq!(
        vehicle
            .mass_model()
            .mass_kg(SimTime::ZERO)
            .unwrap()
            .to_bits(),
        mass_model.mass_kg(SimTime::ZERO).unwrap().to_bits(),
        "vehicle-owned and kernel mass models must agree at start",
    );

    let config = SimulationConfig {
        initial_state: build_initial_state(),
        integrator: Rk4FixedStep,
        force_model: vehicle,
        mass_model,
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(STOP_S)),
        dt: Duration::from_seconds(DT_S),
        scenario_seed: 0x00C0_FFEE_BEEF_F00D,
    };

    let mut kernel = SimulationKernel::new(config).expect("valid kernel");
    collect_trace(&mut kernel, DT_S)
}

fn run_niskanen_scenario_and_collect_trace() -> TraceMetrics {
    let scenario = load_niskanen_scenario();
    let vehicle = build_niskanen_chapter6_vehicle(&scenario);
    let mass_model = MotorMassAdapter::new(
        load_niskanen_c6_motor(),
        scenario.vehicle.dry_vehicle_mass_kg,
        scenario.simulation.ignition_time_s,
        NISKANEN_MASS_MODEL_ID,
    );
    assert_eq!(
        vehicle
            .mass_model()
            .mass_kg(SimTime::ZERO)
            .unwrap()
            .to_bits(),
        mass_model.mass_kg(SimTime::ZERO).unwrap().to_bits(),
        "vehicle-owned and kernel mass models must agree at start",
    );

    let config = SimulationConfig {
        initial_state: build_niskanen_initial_state(&scenario),
        integrator: Rk4FixedStep,
        force_model: vehicle,
        mass_model,
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(scenario.simulation.stop_s)),
        dt: Duration::from_seconds(scenario.simulation.dt_s),
        scenario_seed: 0x004E_4953_4B41_4E45,
    };

    let mut kernel = SimulationKernel::new(config).expect("valid Niskanen kernel");
    collect_trace(&mut kernel, scenario.simulation.dt_s)
}

#[test]
fn d12_sounding_rocket_apogee_matches_envelope() {
    let metrics = run_d12_scenario_and_collect_trace();

    // Tight physical sanity ranges around the pinned D12 trace below.
    // These ranges catch wrong units or sign errors without replacing
    // the exact replay pin.
    assert!(
        (450.0..=550.0).contains(&metrics.apogee_altitude_m),
        "apogee {} m outside envelope [450, 550]",
        metrics.apogee_altitude_m,
    );
    assert!(
        (8.5..=10.5).contains(&metrics.apogee_time_s),
        "apogee time {} s outside envelope [8.5, 10.5]",
        metrics.apogee_time_s,
    );
    assert!(
        (110.0..=140.0).contains(&metrics.max_velocity_m_s),
        "max velocity {} m/s outside envelope [110, 140]",
        metrics.max_velocity_m_s,
    );
    assert!(
        (240.0..=285.0).contains(&metrics.max_acceleration_m_s2),
        "max accel {} m/s² outside envelope [240, 285]",
        metrics.max_acceleration_m_s2,
    );
    assert!(
        metrics.apogee_time_s < STOP_S - DT_S,
        "apogee must occur before the final sample: {metrics:?}",
    );
    assert!(
        metrics.final_velocity_z_m_s < 0.0,
        "run must continue into post-apogee descent: {metrics:?}",
    );
}

#[test]
fn d12_sounding_rocket_run_is_byte_stable_across_two_reruns() {
    let a = run_d12_scenario_and_collect_trace();
    let b = run_d12_scenario_and_collect_trace();
    assert_eq!(
        a.final_state_byte_hash, b.final_state_byte_hash,
        "byte-stable replay required: {a:?} vs {b:?}",
    );
    // Also check the headline metrics are bit-identical.
    assert_eq!(a.apogee_altitude_m.to_bits(), b.apogee_altitude_m.to_bits());
    assert_eq!(a.apogee_time_s.to_bits(), b.apogee_time_s.to_bits());
    assert_eq!(a.max_velocity_m_s.to_bits(), b.max_velocity_m_s.to_bits());
    assert_eq!(
        a.max_acceleration_m_s2.to_bits(),
        b.max_acceleration_m_s2.to_bits()
    );

    assert_eq!(a.final_state_byte_hash, D12_PINNED_FINAL_STATE_HASH);
    assert_eq!(
        a.apogee_altitude_m.to_bits(),
        D12_PINNED_APOGEE_ALTITUDE_M.to_bits()
    );
    assert_eq!(
        a.apogee_time_s.to_bits(),
        D12_PINNED_APOGEE_TIME_S.to_bits()
    );
    assert_eq!(
        a.max_velocity_m_s.to_bits(),
        D12_PINNED_MAX_VELOCITY_M_S.to_bits()
    );
    assert_eq!(
        a.max_acceleration_m_s2.to_bits(),
        D12_PINNED_MAX_ACCELERATION_M_S2.to_bits()
    );
}

#[test]
fn d12_motor_and_aero_decks_load_independently() {
    let motor = load_d12_motor();
    assert_eq!(motor.burn_duration_s().to_bits(), 1.65_f64.to_bits());
    assert_eq!(
        motor.total_impulse_n_s().to_bits(),
        16.839_139_5_f64.to_bits()
    );
    assert_eq!(motor.propellant_mass_kg().to_bits(), 0.0211_f64.to_bits());

    let deck = load_d12_aero_deck();
    assert_eq!(deck.mach_grid().len(), 5);
    assert_eq!(deck.alpha_grid_deg().len(), 5);
    assert_eq!(deck.beta_grid_deg().len(), 1);
    // CD at (M=0, α=0) = 0.6 per the closed form.
    let coeffs = deck.lookup(0.0, 0.0, 0.0).unwrap();
    assert_eq!(coeffs.cd.to_bits(), 0.6_f64.to_bits());
    assert_eq!(coeffs.cn.to_bits(), 0.0_f64.to_bits());
}

#[test]
fn ussa76_atmosphere_loads_via_default_constructor() {
    let _atm = UsStandard1976::new();
    // Smoke check: the kernel uses this in the integration test
    // above; if construction fails the run would never start.
    // Sanity: USSA76 isothermal-fixture for a sea-level reference.
    let _atm_iso = IsothermalAtmosphere::ussa_sea_level();
}

#[test]
fn niskanen_chapter6_scenario_file_matches_published_table() {
    // Scenario TOML round-trips bit-exactly to the published Chapter-6
    // values. This is the data-pin counterpart of the published-value
    // reference: a typo in the TOML or in the published table fails
    // CI before the integration test reaches the kernel.
    let s = load_niskanen_scenario();
    assert_eq!(s.vehicle.body_length_m.to_bits(), 0.56_f64.to_bits());
    assert_eq!(s.vehicle.body_diameter_m.to_bits(), 0.029_f64.to_bits());
    assert_eq!(s.vehicle.nose_length_m.to_bits(), 0.10_f64.to_bits());
    assert_eq!(s.vehicle.dry_vehicle_mass_kg.to_bits(), 0.080_f64.to_bits());
    assert_eq!(s.vehicle.axial_cd.to_bits(), 0.8_f64.to_bits());
    assert_eq!(
        s.reference_apogees_m.b4_experimental.to_bits(),
        64.0_f64.to_bits()
    );
    assert_eq!(
        s.reference_apogees_m.b4_openrocket.to_bits(),
        74.4_f64.to_bits()
    );
    assert_eq!(
        s.reference_apogees_m.b4_rocksim.to_bits(),
        79.1_f64.to_bits()
    );
    assert_eq!(
        s.reference_apogees_m.c6_experimental.to_bits(),
        151.5_f64.to_bits()
    );
    assert_eq!(
        s.reference_apogees_m.c6_openrocket.to_bits(),
        161.4_f64.to_bits()
    );
    assert_eq!(
        s.reference_apogees_m.c6_rocksim.to_bits(),
        180.1_f64.to_bits()
    );
    assert_eq!(s.tolerance.apogee_relative.to_bits(), 0.05_f64.to_bits());
    assert_eq!(s.simulation.stop_s.to_bits(), 20.0_f64.to_bits());
    assert_eq!(s.simulation.dt_s.to_bits(), 0.001_f64.to_bits());
}

#[test]
fn niskanen_chapter6_c6_apogee_matches_published_experiment() {
    let scenario = load_niskanen_scenario();
    let metrics = run_niskanen_scenario_and_collect_trace();

    let experimental = scenario.reference_apogees_m.c6_experimental;
    let tol = scenario.tolerance.apogee_relative;
    let low = experimental * (1.0 - tol);
    let high = experimental * (1.0 + tol);

    assert!(
        (low..=high).contains(&metrics.apogee_altitude_m),
        "Niskanen C6 apogee {} m outside [{low}, {high}] m: {metrics:?}",
        metrics.apogee_altitude_m,
    );
    assert!(
        metrics.apogee_time_s < scenario.simulation.stop_s - scenario.simulation.dt_s,
        "Niskanen apogee must occur before the final sample: {metrics:?}",
    );
    assert!(
        metrics.final_velocity_z_m_s < 0.0,
        "Niskanen run must continue into post-apogee descent: {metrics:?}",
    );
}

#[test]
fn niskanen_chapter6_run_is_byte_stable_across_two_reruns() {
    let a = run_niskanen_scenario_and_collect_trace();
    let b = run_niskanen_scenario_and_collect_trace();
    assert_eq!(
        a.final_state_byte_hash, b.final_state_byte_hash,
        "byte-stable replay required: {a:?} vs {b:?}",
    );
    assert_eq!(a.apogee_altitude_m.to_bits(), b.apogee_altitude_m.to_bits());
    assert_eq!(a.apogee_time_s.to_bits(), b.apogee_time_s.to_bits());
    assert_eq!(a.max_velocity_m_s.to_bits(), b.max_velocity_m_s.to_bits());
    assert_eq!(
        a.max_acceleration_m_s2.to_bits(),
        b.max_acceleration_m_s2.to_bits()
    );

    assert_eq!(a.final_state_byte_hash, NISKANEN_PINNED_FINAL_STATE_HASH);
    assert_eq!(
        a.apogee_altitude_m.to_bits(),
        NISKANEN_PINNED_APOGEE_ALTITUDE_M.to_bits()
    );
    assert_eq!(
        a.apogee_time_s.to_bits(),
        NISKANEN_PINNED_APOGEE_TIME_S.to_bits()
    );
    assert_eq!(
        a.max_velocity_m_s.to_bits(),
        NISKANEN_PINNED_MAX_VELOCITY_M_S.to_bits()
    );
    assert_eq!(
        a.max_acceleration_m_s2.to_bits(),
        NISKANEN_PINNED_MAX_ACCELERATION_M_S2.to_bits()
    );
}

#[test]
fn sounding_rocket_model_ids_are_distinct() {
    let mut ids = [
        D12_DRAG_MODEL_ID.value(),
        D12_THRUST_MODEL_ID.value(),
        D12_MASS_MODEL_ID.value(),
        NISKANEN_DRAG_MODEL_ID.value(),
        NISKANEN_THRUST_MODEL_ID.value(),
        NISKANEN_MASS_MODEL_ID.value(),
    ];
    ids.sort_unstable();
    for pair in ids.windows(2) {
        assert_ne!(pair[0], pair[1]);
    }
}
