//! Non-autonomous analytic checks for the timestamp carried by RK stages.
#![allow(clippy::expect_used)]

use approx::assert_abs_diff_eq;
use nalgebra::Vector3;
use openbmp_core::{Duration, Position3, SimTime, Velocity3};
use openbmp_sim::{
    DenseOutput, Dopri54Adaptive, Dopri54FixedStep, Dopri853Adaptive, Dopri853FixedStep,
    Integrator, ModelEvalError, PointMassDerivative, Rk4FixedStep,
};
use openbmp_state::PointMassState;
use uom::si::{f64::Mass, mass::kilogram};

fn initial() -> PointMassState {
    PointMassState::new(
        SimTime::from_seconds(2.0),
        Position3::origin(),
        Velocity3::zero(),
        Mass::new::<kilogram>(1.0),
    )
}

// A model may read the timestamp from its state snapshot. For a(t) = t,
// v(t0 + h) = t0*h + h^2/2 and x(t0 + h) = t0*h^2/2 + h^3/6.
fn ramp_acceleration(
    s: &PointMassState,
    t: SimTime,
) -> Result<PointMassDerivative, ModelEvalError> {
    assert_eq!(s.time.as_seconds().to_bits(), t.as_seconds().to_bits());
    Ok(PointMassDerivative {
        velocity_m_s: s.velocity.vector,
        acceleration_m_s2: Vector3::new(s.time.as_seconds(), 0.0, 0.0),
        mass_rate_kg_s: 0.0,
    })
}

fn check_solution(s: &PointMassState, h: f64) {
    assert_abs_diff_eq!(s.time.as_seconds(), 2.0 + h, epsilon = 2e-14);
    assert_abs_diff_eq!(s.velocity.vector.x, 2.0 * h + h * h / 2.0, epsilon = 2e-12);
    assert_abs_diff_eq!(
        s.position.vector.x,
        h * h + h * h * h / 6.0,
        epsilon = 2e-12
    );
}

fn check_integrator(i: impl Integrator<PointMassState>) {
    let end = i
        .advance(&initial(), ramp_acceleration, Duration::from_seconds(0.5))
        .expect("ramp step");
    check_solution(&end, 0.5);
}

#[test]
fn rk4_stage_time_matches_nonautonomous_solution() {
    check_integrator(Rk4FixedStep);
}

#[test]
fn dopri54_fixed_stage_time_matches_nonautonomous_solution() {
    check_integrator(Dopri54FixedStep);
}

#[test]
fn dopri853_fixed_stage_time_matches_nonautonomous_solution() {
    check_integrator(Dopri853FixedStep);
}

#[test]
fn dopri54_adaptive_stage_time_matches_nonautonomous_solution() {
    check_integrator(Dopri54Adaptive::new(1e-10, 1e-10, 1e-6, 0.5).expect("integrator"));
}

#[test]
fn dopri853_adaptive_stage_time_matches_nonautonomous_solution() {
    check_integrator(Dopri853Adaptive::new(1e-10, 1e-10, 1e-6, 0.5).expect("integrator"));
}

#[test]
fn dopri54_dense_stage_time_matches_nonautonomous_solution() {
    let i = Dopri54Adaptive::new(1e-10, 1e-10, 1e-6, 0.5).expect("integrator");
    let dense = i
        .advance_with_dense_output(&initial(), ramp_acceleration, Duration::from_seconds(0.5))
        .expect("dense step");
    check_solution(&dense.final_state(), 0.5);
    check_solution(
        &dense
            .interpolate(SimTime::from_seconds(2.25))
            .expect("midpoint"),
        0.25,
    );
}

#[test]
fn dopri853_dense_stage_time_matches_nonautonomous_solution() {
    let i = Dopri853Adaptive::new(1e-10, 1e-10, 1e-6, 0.5).expect("integrator");
    let dense = i
        .advance_with_dense_output(&initial(), ramp_acceleration, Duration::from_seconds(0.5))
        .expect("dense step");
    check_solution(&dense.final_state(), 0.5);
    check_solution(
        &dense
            .interpolate(SimTime::from_seconds(2.25))
            .expect("midpoint"),
        0.25,
    );
}
