//! Phase 2.9 — sounding-rocket validation case.
//!
//! Vertical-launch model rocket with the real Estes D12 motor
//! (`data/motors/estes-d12-eng-derived.toml`), the synthetic
//! D12-class aero deck (`data/aero/synthetic-d12-class-rocket.toml`),
//! USSA76 atmosphere, and ECI-`+z` constant gravity. The kernel
//! integrates the point-mass equations of motion and the test
//! computes apogee, max velocity, and max acceleration from the
//! trace, asserting against a tolerance table derived from the
//! analytic rocket-equation envelope and the published Estes D12
//! motor specs.
//!
//! # Niskanen 2009 deferral
//!
//! The Phase-2 plan calls for comparing against Niskanen 2009
//! thesis §6 worked-example apogee. The thesis PDF is not text-
//! extractable from the project's research environment, so the
//! Phase-2.9 case is built around the **real Estes D12** (which is
//! transcribable from ThrustCurve.org) plus a **synthetic D12-
//! class airframe**. The Niskanen-specific numerical comparison
//! stays as a deferred follow-up where the thesis values can be
//! extracted.

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
    ConstantGravityForce, EndTime, NullEnvironment, Rk4FixedStep, SimulationConfig,
    SimulationKernel,
};
use openbmp_state::PointMassState;
use openbmp_vehicle::{
    AxialDragForceAdapter, BasicVehicle, MotorMassAdapter, MotorThrustForceAdapter, NamedForceModel,
};
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

// Vehicle parameters for the canonical D12 sounding-rocket case.
// 70 g dry-airframe mass matches the lower end of the Estes
// Alpha-III / Big-Bertha class small rockets that the D12 motor
// is most commonly used in.
const DRY_VEHICLE_MASS_KG: f64 = 0.070;
const IGNITION_TIME_S: f64 = 0.0;
const G_M_S2: f64 = 9.80665;
const DT_S: f64 = 0.001; // 1 ms — fine enough for D12 burn dynamics
const STOP_S: f64 = 8.0; // covers ascent + apogee; stops before the
// rocket would re-enter z=0 (the USSA76 atmosphere rejects sub-surface
// queries). Apogee for a D12 + 50 g airframe occurs at ~5–7 s; 8 s
// gives a clean post-apogee margin without falling back through 0.

fn load_d12_motor() -> SolidMotor {
    SolidMotor::load_from_str(D12_DECK).expect("D12 motor deck must parse")
}

fn load_d12_aero_deck() -> AeroDeck {
    AeroDeck::load_from_str(D12_AERO_DECK).expect("D12 aero deck must parse")
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
    let drag = AxialDragForceAdapter::new(deck, atmosphere, ModelId::new(1));
    let thrust = MotorThrustForceAdapter::new(motor, IGNITION_TIME_S, ModelId::new(2));

    BasicVehicle::new(
        vec![
            NamedForceModel::new("gravity", Box::new(gravity)),
            NamedForceModel::new("drag", Box::new(drag)),
            NamedForceModel::new("thrust", Box::new(thrust)),
        ],
        vec![],
        Box::new(openbmp_sim::ConstantMass::new(
            DRY_VEHICLE_MASS_KG
                + load_d12_motor().dry_mass_kg()
                + load_d12_motor().propellant_mass_kg(),
        )),
    )
    .expect("vehicle construction must succeed")
}

#[derive(Copy, Clone, Debug)]
struct TraceMetrics {
    apogee_altitude_m: f64,
    apogee_time_s: f64,
    max_velocity_m_s: f64,
    max_acceleration_m_s2: f64,
    final_state_byte_hash: u64,
}

fn run_d12_scenario_and_collect_trace() -> TraceMetrics {
    let vehicle = build_d12_vehicle();
    let mass_model = MotorMassAdapter::new(
        load_d12_motor(),
        DRY_VEHICLE_MASS_KG,
        IGNITION_TIME_S,
        ModelId::new(3),
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

    // Step the kernel, recording the per-step state for trace metrics.
    let mut max_altitude_m = 0.0_f64;
    let mut apogee_time_s = 0.0_f64;
    let mut max_velocity_m_s = 0.0_f64;
    let mut max_accel_m_s2 = 0.0_f64;
    let mut prev_velocity_z: f64 = 0.0;

    let initial = kernel.current_state();
    let initial_altitude = initial.position.vector.z;

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
        let accel = (velocity_z - prev_velocity_z).abs() / DT_S;
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

    let _ = initial_altitude; // launch altitude is 0; keep for clarity
    TraceMetrics {
        apogee_altitude_m: max_altitude_m,
        apogee_time_s,
        max_velocity_m_s,
        max_acceleration_m_s2: max_accel_m_s2,
        final_state_byte_hash,
    }
}

#[test]
fn d12_sounding_rocket_apogee_matches_envelope() {
    let metrics = run_d12_scenario_and_collect_trace();

    // Envelope-based tolerance asserts. With a 70 g dry airframe +
    // Estes D12 (16.84 N·s impulse, 1.65 s burn), the analytic
    // rocket-equation envelope (delta_V ≈ I/m_avg minus gravity drag,
    // followed by ballistic coast with axial drag) produces an
    // apogee in the 250–650 m band depending on the assumed drag
    // coefficient. Apogee time falls in the 6–9 s band. These
    // envelopes derive from the closed-form rocket equation plus
    // published D12 motor-spec values, NOT from a third-party
    // benchmark — see the deferral note at the top of the file.
    assert!(
        (200.0..=750.0).contains(&metrics.apogee_altitude_m),
        "apogee {} m outside envelope [200, 750]",
        metrics.apogee_altitude_m,
    );
    assert!(
        (3.0..=12.0).contains(&metrics.apogee_time_s),
        "apogee time {} s outside envelope [3, 12]",
        metrics.apogee_time_s,
    );
    assert!(
        (50.0..=250.0).contains(&metrics.max_velocity_m_s),
        "max velocity {} m/s outside envelope [50, 250]",
        metrics.max_velocity_m_s,
    );
    // Peak D12 thrust 29.73 N ÷ initial total mass.
    // (70 + 21.5 + 21.1 = 112.6 g) gives ~264 m/s² peak acceleration.
    // Numerical-derivative noise broadens the envelope.
    assert!(
        (100.0..=500.0).contains(&metrics.max_acceleration_m_s2),
        "max accel {} m/s² outside envelope [100, 500]",
        metrics.max_acceleration_m_s2,
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
