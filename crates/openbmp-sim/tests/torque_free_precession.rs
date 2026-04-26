//! Phase-2.1.D analytic-toy validation: torque-free rigid-body precession.
//!
//! Drives the new rigid-body kernel end-to-end against the analytic
//! torque-free Euler problem.
//!
//! # Reference physics
//!
//! For a rigid body with **zero applied moment**, Euler's equations
//! give:
//!
//! ```text
//!   d/dt (Iω_body) = -ω_body × (Iω_body)
//! ```
//!
//! Two integrals of motion follow from the no-torque condition:
//!
//! * **Angular-momentum magnitude conservation.** `‖Iω_body_eci‖`
//!   is constant in time. (When expressed in the body frame, `Iω_body`
//!   precesses; only its magnitude is conserved. In the inertial frame,
//!   the angular-momentum *vector* itself is conserved exactly. We
//!   check the body-frame magnitude.)
//! * **Rotational kinetic energy conservation.** `½ ω_body · Iω_body`
//!   is constant.
//!
//! For an **axisymmetric** rigid body (two equal principal moments,
//! `I_xx = I_yy = I_t`, distinct from `I_zz`), the body-frame
//! transverse angular velocity precesses at the constant rate
//!
//! ```text
//!   Ω_precess = (I_zz - I_t) / I_t * ω_z
//! ```
//!
//! around the symmetry axis `body +z`. Period `T_precess =
//! 2π / |Ω_precess|`.
//!
//! # What this test asserts
//!
//! 1. The rigid-body kernel runs to completion under [`ZeroMoment`] +
//!    [`ZeroForce`] with [`ConstantMassRigid`].
//! 2. The orientation quaternion magnitude stays inside `1.0 ± 1e-12`
//!    after every `project()`.
//! 3. Body-frame angular-momentum magnitude `‖Iω_body‖` is conserved
//!    within tolerance over the run.
//! 4. Body-frame kinetic energy `½ ω·Iω` is conserved within tolerance.
//! 5. The precession period matches the closed-form value within
//!    tolerance.
//! 6. Two reruns of the same scenario produce bit-identical final
//!    state on the reference platform.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::float_cmp
)]

use approx::assert_abs_diff_eq;
use nalgebra::{Matrix3, Vector3};
use openbmp_core::{
    AngularVelocity3, Body, Duration, Eci, Position3, Quaternion, SimTime, UnitQuaternion,
    Velocity3,
};
use openbmp_sim::{
    ConstantMassRigid, EndTime, NullEnvironment, RigidBodyKernel, RigidModels, Rk4FixedStep,
    SimulationConfig, ZeroForce, ZeroMoment,
};
use openbmp_state::{MassProperties, RigidBodyState};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

/// Build the canonical torque-free precession scenario.
///
/// 1 kg axisymmetric body. Diagonal inertia `(I_t, I_t, I_zz)` with
/// `I_t = 1.0 kg·m²` and `I_zz = 2.0 kg·m²`. Initial body-frame angular
/// velocity is tilted off the symmetry axis: `ω = (0.1, 0, 1.0) rad/s`.
/// At rest at the ECI origin (translation is decoupled).
fn build_kernel(
    stop_s: f64,
) -> RigidBodyKernel<Rk4FixedStep, ZeroForce, ZeroMoment, ConstantMassRigid, NullEnvironment, EndTime>
{
    let inertia = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, 2.0));
    let mass_props = MassProperties::new(Mass::new::<kilogram>(1.0), Position3::origin(), inertia);
    let initial_state = RigidBodyState::new(
        SimTime::ZERO,
        Position3::origin(),
        Velocity3::zero(),
        Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
        AngularVelocity3::new(0.1, 0.0, 1.0),
        mass_props,
    );
    let config = SimulationConfig {
        initial_state,
        integrator: Rk4FixedStep,
        force_model: ZeroForce,
        mass_model: RigidModels::new(ZeroMoment, ConstantMassRigid::new(mass_props)),
        environment: NullEnvironment,
        stop_condition: EndTime::new(SimTime::from_seconds(stop_s)),
        dt: Duration::from_seconds(0.01),
        scenario_seed: 0x0afe_f00d_dead_beef,
    };
    RigidBodyKernel::new_rigid(config).expect("valid config must construct")
}

fn body_angular_momentum_magnitude(state: &RigidBodyState) -> f64 {
    let l = state.mass_props.inertia_body * state.angular_velocity.vector;
    l.norm()
}

fn body_kinetic_energy(state: &RigidBodyState) -> f64 {
    let l = state.mass_props.inertia_body * state.angular_velocity.vector;
    0.5 * state.angular_velocity.vector.dot(&l)
}

fn quaternion_unit_residual(state: &RigidBodyState) -> f64 {
    let c = state.orientation.q.into_inner().coords;
    let n2 = c.x * c.x + c.y * c.y + c.z * c.z + c.w * c.w;
    (n2.sqrt() - 1.0).abs()
}

fn assert_rigid_states_bit_equal(a: &RigidBodyState, b: &RigidBodyState) {
    assert_eq!(a.time.as_seconds().to_bits(), b.time.as_seconds().to_bits());

    for axis in 0..3 {
        assert_eq!(
            a.position.vector[axis].to_bits(),
            b.position.vector[axis].to_bits(),
            "position axis {axis} differs",
        );
        assert_eq!(
            a.velocity.vector[axis].to_bits(),
            b.velocity.vector[axis].to_bits(),
            "velocity axis {axis} differs",
        );
        assert_eq!(
            a.angular_velocity.vector[axis].to_bits(),
            b.angular_velocity.vector[axis].to_bits(),
            "angular velocity axis {axis} differs",
        );
        assert_eq!(
            a.mass_props.center_of_mass_body.vector[axis].to_bits(),
            b.mass_props.center_of_mass_body.vector[axis].to_bits(),
            "center of mass axis {axis} differs",
        );
    }

    for axis in 0..4 {
        assert_eq!(
            a.orientation.q.coords[axis].to_bits(),
            b.orientation.q.coords[axis].to_bits(),
            "orientation coord {axis} differs",
        );
    }

    assert_eq!(
        a.mass_props.mass.get::<kilogram>().to_bits(),
        b.mass_props.mass.get::<kilogram>().to_bits(),
        "mass differs",
    );

    for row in 0..3 {
        for col in 0..3 {
            assert_eq!(
                a.mass_props.inertia_body[(row, col)].to_bits(),
                b.mass_props.inertia_body[(row, col)].to_bits(),
                "inertia entry ({row}, {col}) differs",
            );
        }
    }
}

#[test]
fn torque_free_precession_runs_to_completion() {
    let mut kernel = build_kernel(10.0);
    kernel.run().expect("kernel run must succeed");
    assert_eq!(kernel.current_step().value(), 1000);
    assert_abs_diff_eq!(kernel.current_time().as_seconds(), 10.0, epsilon = 1.0e-12);
}

#[test]
fn torque_free_quaternion_stays_unit_norm() {
    let mut kernel = build_kernel(10.0);
    while kernel.stop_reason().is_none() {
        kernel.step().expect("step");
        let residual = quaternion_unit_residual(kernel.current_state());
        assert!(
            residual < 1.0e-12,
            "quaternion drifted off the unit sphere by {residual} at t = {} s",
            kernel.current_time().as_seconds(),
        );
    }
}

#[test]
fn torque_free_conserves_angular_momentum_magnitude() {
    let mut kernel = build_kernel(10.0);
    let initial_l = body_angular_momentum_magnitude(kernel.current_state());
    kernel.run().expect("kernel run");
    let final_l = body_angular_momentum_magnitude(kernel.current_state());
    let relative_drift = (final_l - initial_l).abs() / initial_l;
    assert!(
        relative_drift < 1.0e-9,
        "angular-momentum magnitude drifted by {relative_drift} \
         (initial = {initial_l}, final = {final_l})",
    );
}

#[test]
fn torque_free_conserves_kinetic_energy() {
    let mut kernel = build_kernel(10.0);
    let initial_e = body_kinetic_energy(kernel.current_state());
    kernel.run().expect("kernel run");
    let final_e = body_kinetic_energy(kernel.current_state());
    let relative_drift = (final_e - initial_e).abs() / initial_e;
    assert!(
        relative_drift < 1.0e-9,
        "kinetic energy drifted by {relative_drift} \
         (initial = {initial_e}, final = {final_e})",
    );
}

/// Closed-form precession period: with `I_t = 1.0`, `I_zz = 2.0`, and
/// `ω_z = 1.0`, `Ω_precess = (2 - 1) / 1 * 1 = 1.0 rad/s`, so the body-
/// frame transverse angular velocity completes one full precession in
/// `2π ≈ 6.2831853 s`. We run for slightly more than one full period and
/// look for the body-frame transverse rate vector returning near its
/// initial direction.
#[test]
fn torque_free_precession_period_matches_closed_form() {
    let mut kernel = build_kernel(7.0); // > 2π for one full revolution
    let initial_omega_xy = Vector3::new(
        kernel.current_state().angular_velocity.vector.x,
        kernel.current_state().angular_velocity.vector.y,
        0.0,
    );
    let initial_norm = initial_omega_xy.norm();
    assert!(
        initial_norm > 0.0,
        "initial transverse rate must be non-zero"
    );

    // Sample the angular velocity around the expected period. With
    // `Ω = 1.0 rad/s`, the period is `2π s`. We probe at times within
    // a half-period of `2π` and find the closest match to the initial
    // transverse vector direction.
    let target_period_s = 2.0 * std::f64::consts::PI;

    let mut best_dt: f64 = 1.0; // worst-case dot value (1 = perfectly aligned)
    let mut observed_period_s = 0.0_f64;
    while kernel.stop_reason().is_none() {
        kernel.step().expect("step");
        let t = kernel.current_time().as_seconds();
        if t < target_period_s - 0.5 || t > target_period_s + 0.5 {
            continue;
        }
        let here = Vector3::new(
            kernel.current_state().angular_velocity.vector.x,
            kernel.current_state().angular_velocity.vector.y,
            0.0,
        );
        let cos_angle = here.dot(&initial_omega_xy) / (initial_norm * here.norm());
        let angular_distance = (1.0 - cos_angle).abs();
        if angular_distance < best_dt {
            best_dt = angular_distance;
            observed_period_s = t;
        }
    }
    let period_error_s = (observed_period_s - target_period_s).abs();
    assert!(
        period_error_s < 0.05,
        "observed precession period = {observed_period_s} s, \
         expected = {target_period_s} s, error = {period_error_s} s",
    );
}

#[test]
fn torque_free_precession_is_byte_stable_across_two_runs() {
    let mut a = build_kernel(2.0);
    let mut b = build_kernel(2.0);
    a.run().expect("run a");
    b.run().expect("run b");
    assert_eq!(a.current_step(), b.current_step());
    assert_rigid_states_bit_equal(a.current_state(), b.current_state());
}
