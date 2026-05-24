//! Kernel integration: a kernel running with a
//! single-element `KernelVehicle` wrapping `ConstantGravityForce`
//! produces byte-identical final state to a kernel running the
//! raw `ConstantGravityForce` directly.
//!
//! This is the headline byte-stability gate — the
//! `analytic_toy` regression must remain replayable when
//! the kernel is fed a Vehicle composition instead of a raw model.

#![allow(missing_docs)] // integration test
#![allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]

use nalgebra::Vector3;
use openbmp_core::{Duration, Position3, SimTime, Velocity3};
use openbmp_sim::{
    ConstantGravityForce, ConstantMass, EndTime, NullEnvironment, Rk4FixedStep, SimulationConfig,
    SimulationKernel,
};
use openbmp_state::PointMassState;
use openbmp_vehicle::{KernelVehicle, NamedForceModel};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

const G_M_S2: f64 = 9.80665;
const DT_S: f64 = 0.01;
const STOP_S: f64 = 10.0;

fn initial_state() -> PointMassState {
    PointMassState::new(
        SimTime::ZERO,
        Position3::origin(),
        Velocity3::zero(),
        Mass::new::<kilogram>(1.0),
    )
}

fn run_with_raw_force() -> PointMassState {
    let config = SimulationConfig {
        initial_state: initial_state(),
        integrator: Rk4FixedStep,
        force_model: ConstantGravityForce::new(Vector3::new(0.0, 0.0, -G_M_S2)),
        mass_model: ConstantMass::new(1.0),
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(STOP_S)),
        dt: Duration::from_seconds(DT_S),
        scenario_seed: 0xcafe_f00d_dead_beef,
    };
    let mut kernel = SimulationKernel::new(config).expect("valid configuration");
    kernel.run().expect("run");
    *kernel.current_state()
}

fn run_with_basic_vehicle() -> PointMassState {
    let vehicle: KernelVehicle<PointMassState> = KernelVehicle::new(
        vec![NamedForceModel::new(
            "gravity",
            Box::new(ConstantGravityForce::new(Vector3::new(0.0, 0.0, -G_M_S2))),
        )],
        vec![],
        Box::new(ConstantMass::new(1.0)),
    )
    .expect("vehicle construction");
    let config = SimulationConfig {
        initial_state: initial_state(),
        integrator: Rk4FixedStep,
        force_model: vehicle,
        mass_model: ConstantMass::new(1.0),
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(STOP_S)),
        dt: Duration::from_seconds(DT_S),
        scenario_seed: 0xcafe_f00d_dead_beef,
    };
    let mut kernel = SimulationKernel::new(config).expect("valid configuration");
    kernel.run().expect("run");
    *kernel.current_state()
}

#[test]
fn single_element_vehicle_produces_byte_identical_final_state() {
    let raw = run_with_raw_force();
    let via_vehicle = run_with_basic_vehicle();
    // The headline guarantee: a single-force vehicle must produce
    // byte-identical output to the raw-force kernel. This is the
    // `analytic_toy` byte-stability claim, exercised through
    // the KernelVehicle code path.
    assert_eq!(
        raw.time.as_seconds().to_bits(),
        via_vehicle.time.as_seconds().to_bits()
    );
    assert_eq!(
        raw.position.vector.x.to_bits(),
        via_vehicle.position.vector.x.to_bits()
    );
    assert_eq!(
        raw.position.vector.y.to_bits(),
        via_vehicle.position.vector.y.to_bits()
    );
    assert_eq!(
        raw.position.vector.z.to_bits(),
        via_vehicle.position.vector.z.to_bits()
    );
    assert_eq!(
        raw.velocity.vector.x.to_bits(),
        via_vehicle.velocity.vector.x.to_bits()
    );
    assert_eq!(
        raw.velocity.vector.y.to_bits(),
        via_vehicle.velocity.vector.y.to_bits()
    );
    assert_eq!(
        raw.velocity.vector.z.to_bits(),
        via_vehicle.velocity.vector.z.to_bits()
    );
    assert_eq!(
        raw.mass.get::<kilogram>().to_bits(),
        via_vehicle.mass.get::<kilogram>().to_bits()
    );
}

#[test]
fn basic_vehicle_run_is_byte_stable_across_two_runs() {
    let a = run_with_basic_vehicle();
    let b = run_with_basic_vehicle();
    assert_eq!(a.position.vector.z.to_bits(), b.position.vector.z.to_bits());
    assert_eq!(a.velocity.vector.z.to_bits(), b.velocity.vector.z.to_bits());
    assert_eq!(a.time.as_seconds().to_bits(), b.time.as_seconds().to_bits());
}
