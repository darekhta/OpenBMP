//! Rigid-body stage-separation validation cases.
//!
//! These tests keep the PR1 propagation case intentionally small:
//! no force, no moment, fixed-step RK4, and a closed-form two-body
//! momentum partition.

#![allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::unwrap_used
)]

use approx::assert_abs_diff_eq;
use nalgebra::{Matrix3, Vector3};
use openbmp_core::{
    AngularVelocity3, Body, BodyId, Duration, Eci, Position3, Quaternion, SimTime, UnitQuaternion,
    Velocity3,
};
use openbmp_sim::{
    ConstantMassRigid, EndTime, ForceContext, ForceModel, InitialRigidBodyLane, ModelEvalError,
    NullEnvironment, RigidBodyKernel, RigidBodySeparation, RigidModels, Rk4FixedStep,
    SimulationConfig, ZeroForce, ZeroMoment,
};
use openbmp_state::{MassProperties, RigidBodyState};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

fn mass_props(mass_kg: f64, cg_z_m: f64, inertia: [f64; 3]) -> MassProperties {
    MassProperties::with_diagonal_inertia(
        Mass::new::<kilogram>(mass_kg),
        Position3::new(0.0, 0.0, cg_z_m),
        inertia[0],
        inertia[1],
        inertia[2],
    )
}

fn composite_mass_props() -> MassProperties {
    MassProperties::new(
        Mass::new::<kilogram>(5.0),
        Position3::new(0.0, 0.0, -0.2),
        Matrix3::from_diagonal(&Vector3::new(2.0, 2.0, 1.0)),
    )
}

#[derive(Copy, Clone, Debug)]
struct BodyOwnedForce;

impl ForceModel<RigidBodyState> for BodyOwnedForce {
    fn force_n_eci(
        &self,
        _ctx: ForceContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        Ok(Vector3::zeros())
    }
}

fn build_kernel_with_force<F>(
    stop_s: f64,
    force_model: F,
) -> RigidBodyKernel<Rk4FixedStep, F, ZeroMoment, ConstantMassRigid, NullEnvironment, EndTime>
where
    F: ForceModel<RigidBodyState>,
{
    let mass_props = composite_mass_props();
    let initial_state = RigidBodyState::new(
        SimTime::ZERO,
        Position3::origin(),
        Velocity3::zero(),
        Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
        AngularVelocity3::new(0.0, 0.0, 0.0),
        mass_props,
    );
    let config = SimulationConfig {
        initial_state,
        integrator: Rk4FixedStep,
        force_model,
        mass_model: RigidModels::new(ZeroMoment, ConstantMassRigid::new(mass_props)),
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(stop_s)),
        dt: Duration::from_seconds(0.1),
        scenario_seed: 0x51a9_e5e0_0000_0001,
    };
    RigidBodyKernel::new_rigid(config).expect("valid rigid-body config must construct")
}

fn build_kernel(
    stop_s: f64,
) -> RigidBodyKernel<Rk4FixedStep, ZeroForce, ZeroMoment, ConstantMassRigid, NullEnvironment, EndTime>
{
    build_kernel_with_force(stop_s, ZeroForce)
}

fn textbook_separation() -> RigidBodySeparation {
    RigidBodySeparation {
        stack_body: BodyId::from_path("vehicle.assembly.bodies.upper"),
        body: BodyId::from_path("vehicle.assembly.bodies.lower"),
        stack_mass_properties: mass_props(4.0, 0.0, [1.0, 1.0, 0.5]),
        stage_mass_properties: mass_props(1.0, -1.0, [0.2, 0.2, 0.1]),
        stack_delta_v_body_m_s: [0.0, 0.0, 0.25],
        stage_delta_v_body_m_s: [0.0, 0.0, -1.0],
    }
}

fn batch_separations() -> [RigidBodySeparation; 2] {
    [
        RigidBodySeparation {
            stack_body: BodyId::from_path("vehicle.assembly.bodies.bus"),
            body: BodyId::from_path("vehicle.assembly.bodies.rv1"),
            stack_mass_properties: mass_props(3.0, 0.0, [1.0, 1.0, 0.5]),
            stage_mass_properties: mass_props(1.0, -1.0, [0.2, 0.2, 0.1]),
            stack_delta_v_body_m_s: [0.0, 0.0, 0.0],
            stage_delta_v_body_m_s: [0.0, 1.0, 0.0],
        },
        RigidBodySeparation {
            stack_body: BodyId::from_path("vehicle.assembly.bodies.bus"),
            body: BodyId::from_path("vehicle.assembly.bodies.rv2"),
            stack_mass_properties: mass_props(3.0, 0.0, [1.0, 1.0, 0.5]),
            stage_mass_properties: mass_props(1.0, 1.0, [0.2, 0.2, 0.1]),
            stack_delta_v_body_m_s: [0.0, 0.0, 0.0],
            stage_delta_v_body_m_s: [0.0, -1.0, 0.0],
        },
    ]
}

fn assert_rigid_states_bit_equal(a: &RigidBodyState, b: &RigidBodyState) {
    assert_eq!(a.time.as_seconds().to_bits(), b.time.as_seconds().to_bits());
    for axis in 0..3 {
        assert_eq!(
            a.position.vector[axis].to_bits(),
            b.position.vector[axis].to_bits()
        );
        assert_eq!(
            a.velocity.vector[axis].to_bits(),
            b.velocity.vector[axis].to_bits()
        );
        assert_eq!(
            a.angular_velocity.vector[axis].to_bits(),
            b.angular_velocity.vector[axis].to_bits()
        );
        assert_eq!(
            a.mass_props.center_of_mass_body.vector[axis].to_bits(),
            b.mass_props.center_of_mass_body.vector[axis].to_bits()
        );
    }
    for axis in 0..4 {
        assert_eq!(
            a.orientation.q.coords[axis].to_bits(),
            b.orientation.q.coords[axis].to_bits()
        );
    }
    assert_eq!(
        a.mass_props.mass.get::<kilogram>().to_bits(),
        b.mass_props.mass.get::<kilogram>().to_bits()
    );
    for row in 0..3 {
        for col in 0..3 {
            assert_eq!(
                a.mass_props.inertia_body[(row, col)].to_bits(),
                b.mass_props.inertia_body[(row, col)].to_bits()
            );
        }
    }
}

#[test]
fn jettison_partitions_textbook_two_stage_state() {
    let mut kernel = build_kernel(0.3);
    kernel
        .jettison_rigid_body(textbook_separation())
        .expect("stage separation must apply");

    let stack = kernel.current_state();
    let stage = &kernel.separated_rigid_bodies()[0].state;
    assert_eq!(kernel.separated_rigid_bodies().len(), 1);
    assert_eq!(stack.mass_props.mass.get::<kilogram>(), 4.0);
    assert_eq!(stage.mass_props.mass.get::<kilogram>(), 1.0);

    assert_abs_diff_eq!(stack.position.vector.z, 0.2, epsilon = 1.0e-15);
    assert_abs_diff_eq!(stage.position.vector.z, -0.8, epsilon = 1.0e-15);
    assert_abs_diff_eq!(stack.velocity.vector.z, 0.25, epsilon = 1.0e-15);
    assert_abs_diff_eq!(stage.velocity.vector.z, -1.0, epsilon = 1.0e-15);

    let total_momentum_z = 4.0 * stack.velocity.vector.z + stage.velocity.vector.z;
    assert_abs_diff_eq!(total_momentum_z, 0.0, epsilon = 1.0e-15);
}

#[test]
fn separated_body_lane_advances_with_primary_lane() {
    let mut kernel = build_kernel(0.3);
    kernel
        .jettison_rigid_body(textbook_separation())
        .expect("stage separation must apply");
    kernel.run().expect("kernel run must succeed");

    assert_eq!(kernel.current_step().value(), 3);
    assert_abs_diff_eq!(kernel.current_time().as_seconds(), 0.3, epsilon = 1.0e-15);
    assert_abs_diff_eq!(
        kernel.current_state().position.vector.z,
        0.275,
        epsilon = 1.0e-15
    );
    assert_abs_diff_eq!(
        kernel.separated_rigid_bodies()[0].state.position.vector.z,
        -1.1,
        epsilon = 1.0e-15
    );
}

#[test]
fn batch_jettison_partitions_multiple_bodies_from_same_pre_split_state() {
    let mut kernel = build_kernel(0.3);
    kernel
        .jettison_rigid_bodies(&batch_separations())
        .expect("batch separation must apply");

    assert_eq!(kernel.separated_rigid_bodies().len(), 2);
    assert_eq!(
        kernel.current_state().mass_props.mass.get::<kilogram>(),
        3.0
    );
    assert_eq!(
        kernel.primary_rigid_body(),
        Some(BodyId::from_path("vehicle.assembly.bodies.bus"))
    );
    assert_abs_diff_eq!(
        kernel.separated_rigid_bodies()[0].state.velocity.vector.y,
        1.0,
        epsilon = 1.0e-15
    );
    assert_abs_diff_eq!(
        kernel.separated_rigid_bodies()[1].state.velocity.vector.y,
        -1.0,
        epsilon = 1.0e-15
    );
}

#[test]
fn initial_rigid_body_lanes_are_active_from_step_zero() {
    let bus = BodyId::from_path("vehicle.assembly.bodies.bus");
    let observer = BodyId::from_path("vehicle.assembly.bodies.observer");
    let mut kernel = build_kernel(0.3);
    let observer_state = RigidBodyState::new(
        SimTime::ZERO,
        Position3::new(10.0, 0.0, 0.0),
        Velocity3::new(0.0, 1.0, 0.0),
        Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
        AngularVelocity3::new(0.0, 0.0, 0.0),
        mass_props(1.0, 0.0, [0.2, 0.2, 0.1]),
    );

    kernel
        .seed_rigid_body_lanes(
            bus,
            mass_props(3.0, 0.0, [1.0, 1.0, 0.5]),
            &[InitialRigidBodyLane {
                body: observer,
                state: observer_state,
            }],
        )
        .expect("initial lane seeding must apply");

    assert_eq!(kernel.primary_rigid_body(), Some(bus));
    assert_eq!(
        kernel.current_state().mass_props.mass.get::<kilogram>(),
        3.0
    );
    assert_eq!(kernel.separated_rigid_bodies().len(), 1);
    assert_eq!(kernel.separated_rigid_bodies()[0].body, observer);
    assert_abs_diff_eq!(
        kernel.separated_rigid_bodies()[0].state.position.vector.x,
        10.0,
        epsilon = 1.0e-15
    );

    kernel.run().expect("kernel run must succeed");
    assert_abs_diff_eq!(
        kernel.separated_rigid_bodies()[0].state.position.vector.y,
        0.3,
        epsilon = 1.0e-15
    );
}

#[test]
fn separated_body_reruns_are_bit_stable() {
    let mut a = build_kernel(0.3);
    a.jettison_rigid_body(textbook_separation())
        .expect("stage separation must apply");
    a.run().expect("first run");

    let mut b = build_kernel(0.3);
    b.jettison_rigid_body(textbook_separation())
        .expect("stage separation must apply");
    b.run().expect("second run");

    assert_rigid_states_bit_equal(a.current_state(), b.current_state());
    assert_eq!(
        a.separated_rigid_bodies().len(),
        b.separated_rigid_bodies().len()
    );
    assert_rigid_states_bit_equal(
        &a.separated_rigid_bodies()[0].state,
        &b.separated_rigid_bodies()[0].state,
    );
}

#[test]
fn duplicate_stage_jettison_is_rejected() {
    let mut kernel = build_kernel(0.3);
    kernel
        .jettison_rigid_body(textbook_separation())
        .expect("first stage separation must apply");
    let err = kernel
        .jettison_rigid_body(textbook_separation())
        .expect_err("duplicate body separation must fail");
    assert!(
        err.to_string().contains("already been jettisoned"),
        "unexpected error: {err}"
    );
}

#[test]
fn stage_jettison_rejects_force_stack_without_separated_body_support() {
    let mut kernel = build_kernel_with_force(0.3, BodyOwnedForce);
    let err = kernel
        .jettison_rigid_body(textbook_separation())
        .expect_err("body-owned force stack must fail closed");
    assert!(
        err.to_string()
            .contains("per-body force-stack ownership is required"),
        "unexpected error: {err}"
    );
}
