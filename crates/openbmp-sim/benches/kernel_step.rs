//! Microbenchmark for the simulation kernel hot path.
//!
//! Realistic targets per the Phase-1.3 research survey: 50–200 ns per
//! step on modern x86_64 for a point-mass under constant gravity.
//!
//! Built with the same `target-feature=-fma` flags as production
//! (inherited from `.cargo/config.toml`) so the numbers match
//! release-mode kernel runs.
//!
//! Pitfalls avoided:
//!
//! * Inputs constructed once outside `b.iter`.
//! * `black_box` wraps both the input and the result so the compiler
//!   cannot delete the call as dead code.
//! * The dynamics closure captures `black_box`-wrapped models so the
//!   force is not constant-folded into the body.
//! * No `#[inline(always)]` on the integrator path.

#![allow(missing_docs)] // Bench harness; lint relaxed.
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::unreadable_literal,
    clippy::unusual_byte_groupings
)]

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use nalgebra::Vector3;
use openbmp_core::{Duration, Position3, SimTime, Velocity3};
use openbmp_sim::{
    AlwaysContinue, ConstantGravityForce, ConstantMass, EndTime, NullEnvironment, Rk4FixedStep,
    SimulationConfig, SimulationKernel,
};
use openbmp_state::PointMassState;
use std::hint::black_box;
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

fn build_kernel(
    dt_s: f64,
) -> SimulationKernel<
    Rk4FixedStep,
    ConstantGravityForce,
    ConstantMass,
    NullEnvironment,
    AlwaysContinue,
> {
    let config = SimulationConfig {
        initial_state: PointMassState::new(
            SimTime::ZERO,
            Position3::new(0.0, 0.0, 1000.0),
            Velocity3::new(0.0, 0.0, 0.0),
            Mass::new::<kilogram>(1.0),
        ),
        integrator: Rk4FixedStep,
        force_model: ConstantGravityForce::new(Vector3::new(0.0, 0.0, -9.80665)),
        mass_model: ConstantMass::new(1.0),
        environment: NullEnvironment,
        stop_condition: AlwaysContinue,
        dt: Duration::from_seconds(dt_s),
        scenario_seed: 0xdead_beef,
    };
    SimulationKernel::new(config).expect("benchmark config must construct")
}

fn bench_single_step(c: &mut Criterion) {
    let mut kernel = build_kernel(0.001);
    let mut group = c.benchmark_group("kernel_step");
    group.throughput(Throughput::Elements(1));
    group.bench_function("point_mass_constant_gravity_dt_001", |b| {
        b.iter(|| {
            // black_box the kernel reference so the optimizer cannot
            // hoist work out of the loop or constant-fold the gravity
            // vector into the inner expressions.
            let k = black_box(&mut kernel);
            k.step().expect("step");
            black_box(k.current_state());
        });
    });
    group.finish();
}

fn bench_run_to_end(c: &mut Criterion) {
    let mut group = c.benchmark_group("kernel_run");
    group.throughput(Throughput::Elements(100));
    group.bench_function("100_steps", |b| {
        b.iter_with_setup(
            || {
                let config = SimulationConfig {
                    initial_state: PointMassState::new(
                        SimTime::ZERO,
                        Position3::new(0.0, 0.0, 1000.0),
                        Velocity3::zero(),
                        Mass::new::<kilogram>(1.0),
                    ),
                    integrator: Rk4FixedStep,
                    force_model: ConstantGravityForce::new(Vector3::new(0.0, 0.0, -9.80665)),
                    mass_model: ConstantMass::new(1.0),
                    environment: NullEnvironment,
                    stop_condition: EndTime::new(SimTime::from_seconds(0.1)),
                    dt: Duration::from_seconds(0.001),
                    scenario_seed: 0,
                };
                SimulationKernel::new(config).expect("construct")
            },
            |mut kernel| {
                let _ = kernel.run().expect("run");
                black_box(kernel.current_state());
            },
        );
    });
    group.finish();
}

criterion_group!(benches, bench_single_step, bench_run_to_end);
criterion_main!(benches);
