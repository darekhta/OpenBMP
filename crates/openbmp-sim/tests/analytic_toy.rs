//! Headline validation: analytic-toy comparison.
//!
//! Runs the kernel against
//! `openbmp-testkit::analytic::ConstantAccelerationDrop`, asserts each
//! result metric against
//! `tests/expected/constant-acceleration-drop.toml`, and verifies the
//! whole run is byte-stable across two reruns via
//! `openbmp-testkit::determinism::require_replay_byte_stable`.

#![allow(missing_docs)] // integration test; doc lints relaxed
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::unreadable_literal,
    clippy::unusual_byte_groupings
)]

use std::path::PathBuf;

use nalgebra::Vector3;
use openbmp_core::{Duration, Position3, SimTime, Velocity3};
use openbmp_sim::{
    AlwaysContinue, ConstantGravityForce, ConstantMass, EndTime, NullEnvironment, Rk4FixedStep,
    SimulationConfig, SimulationKernel, ZeroForce,
};
use openbmp_state::PointMassState;
use openbmp_testkit::{
    analytic::ConstantAccelerationDrop, determinism::require_replay_byte_stable,
    tolerance::ToleranceTable,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

const G_M_S2: f64 = 9.80665;
const DT_S: f64 = 0.01;
const STOP_S: f64 = 10.0;

fn build_kernel(
    stop_at_s: f64,
) -> SimulationKernel<
    PointMassState,
    Rk4FixedStep,
    ConstantGravityForce,
    ConstantMass,
    NullEnvironment,
    EndTime,
> {
    let initial = PointMassState::new(
        SimTime::ZERO,
        Position3::origin(),
        Velocity3::zero(),
        Mass::new::<kilogram>(1.0),
    );
    let config = SimulationConfig {
        initial_state: initial,
        integrator: Rk4FixedStep,
        force_model: ConstantGravityForce::new(Vector3::new(0.0, 0.0, -G_M_S2)),
        mass_model: ConstantMass::new(1.0),
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(stop_at_s)),
        dt: Duration::from_seconds(DT_S),
        scenario_seed: 0xcafe_f00d_dead_beef,
    };
    SimulationKernel::new(config).expect("valid configuration must construct a kernel")
}

#[test]
fn constant_acceleration_drop_matches_analytic_under_tolerance_table() {
    let mut kernel = build_kernel(STOP_S);
    kernel.run().expect("kernel run must succeed");

    let final_state = kernel.current_state();

    // Closed-form reference for sanity-checking the kernel's output
    // against the tolerance table.
    let analytic = ConstantAccelerationDrop {
        initial_position_m: 0.0,
        initial_velocity_m_s: 0.0,
        acceleration_m_s2: -G_M_S2,
    };
    let t = final_state.time.as_seconds();
    let expected_position = analytic.position_at(t);
    let expected_velocity = analytic.velocity_at(t);

    // Compare against the openbmp-testkit closed form to check the
    // kernel's numbers are physically right (independent of the TOML
    // file).
    let position_z = final_state.position.vector.z;
    let velocity_z = final_state.velocity.vector.z;
    assert!(
        (position_z - expected_position).abs() < 1.0e-9,
        "position z = {position_z}, analytic = {expected_position}"
    );
    assert!(
        (velocity_z - expected_velocity).abs() < 1.0e-10,
        "velocity z = {velocity_z}, analytic = {expected_velocity}"
    );

    // Now check the same metrics against the committed tolerance table.
    let mut path: PathBuf = std::env::var_os("CARGO_MANIFEST_DIR")
        .expect("CARGO_MANIFEST_DIR must be set for cargo test")
        .into();
    path.push("tests");
    path.push("expected");
    path.push("constant-acceleration-drop.toml");

    let table = ToleranceTable::from_path(&path).expect("tolerance table must parse");

    table
        .check_metric("final_time_s", t)
        .expect("final_time_s within tolerance");
    table
        .check_metric("final_position_z_m", position_z)
        .expect("final_position_z_m within tolerance");
    table
        .check_metric("final_velocity_z_m_s", velocity_z)
        .expect("final_velocity_z_m_s within tolerance");
}

#[test]
fn constant_acceleration_drop_is_byte_stable_across_two_runs() {
    // Pack the final state's f64 fields little-endian and require
    // byte-stable replay across two independent kernel runs.
    let runner = || -> Result<Vec<u8>, &'static str> {
        let mut kernel = build_kernel(STOP_S);
        kernel.run().map_err(|_| "run failed")?;
        let s = kernel.current_state();
        let mut bytes = Vec::with_capacity(8 * 8);
        for v in [
            s.time.as_seconds(),
            s.position.vector.x,
            s.position.vector.y,
            s.position.vector.z,
            s.velocity.vector.x,
            s.velocity.vector.y,
            s.velocity.vector.z,
            s.mass.get::<kilogram>(),
        ] {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        Ok(bytes)
    };

    require_replay_byte_stable(runner).expect("two reruns must be byte-identical");
}

#[test]
fn shorter_run_also_validates_analytically() {
    // Spot check at t = 1.0 s for an additional grid point.
    let mut kernel = build_kernel(1.0);
    kernel.run().expect("kernel run");

    let analytic = ConstantAccelerationDrop {
        initial_position_m: 0.0,
        initial_velocity_m_s: 0.0,
        acceleration_m_s2: -G_M_S2,
    };
    let t = kernel.current_time().as_seconds();
    let expected_position = analytic.position_at(t);

    let position_z = kernel.current_state().position.vector.z;
    assert!(
        (position_z - expected_position).abs() < 1.0e-12,
        "position z = {position_z}, analytic = {expected_position}, |Δ| = {}",
        (position_z - expected_position).abs()
    );
}

#[test]
fn run_can_be_resumed_step_by_step_with_same_result_as_continuous_run() {
    // Property check: stepping manually N times then querying the
    // state must produce the same byte pattern as run() for N steps.
    let mut continuous = build_kernel(0.05);
    continuous.run().expect("run");
    let continuous_z = continuous.current_state().position.vector.z;
    let continuous_t = continuous.current_time().as_seconds();

    let mut stepwise = build_kernel(0.05);
    while stepwise.stop_reason().is_none() {
        stepwise.step().expect("step");
    }
    let stepwise_z = stepwise.current_state().position.vector.z;
    let stepwise_t = stepwise.current_time().as_seconds();

    assert_eq!(continuous_z.to_bits(), stepwise_z.to_bits());
    assert_eq!(continuous_t.to_bits(), stepwise_t.to_bits());
}

#[test]
fn always_continue_kernel_can_be_driven_one_step_manually() {
    let initial = PointMassState::new(
        SimTime::ZERO,
        Position3::origin(),
        Velocity3::zero(),
        Mass::new::<kilogram>(1.0),
    );
    let config = SimulationConfig {
        initial_state: initial,
        integrator: Rk4FixedStep,
        force_model: ZeroForce,
        mass_model: ConstantMass::new(1.0),
        environment: NullEnvironment,
        stop_condition: AlwaysContinue,
        dt: Duration::from_seconds(DT_S),
        scenario_seed: 0,
    };
    let mut kernel = SimulationKernel::new(config).expect("valid configuration");
    kernel.step().expect("one manual step");
    assert_eq!(kernel.current_step().value(), 1);
    assert!(kernel.stop_reason().is_none());
}
