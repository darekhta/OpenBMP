//! Accepted-substep observers for constitutive model history updates.
#![allow(clippy::expect_used, clippy::float_cmp)]

use std::cell::Cell;

use approx::assert_abs_diff_eq;
use nalgebra::Vector3;
use openbmp_core::{Duration, ModelId, Position3, SimTime, Velocity3};
use openbmp_sim::{
    Dopri54Adaptive, Dopri54FixedStep, Dopri853Adaptive, Dopri853FixedStep, Integrator,
    IntegratorError, ModelEvalError, PointMassDerivative, ProfiledIntegrator, Rk4FixedStep,
    SourceTermCouplingProfile, SourceTermProfile,
};
use openbmp_state::PointMassState;
use uom::si::{f64::Mass, mass::kilogram};

fn initial() -> PointMassState {
    PointMassState::new(
        SimTime::ZERO,
        Position3::origin(),
        Velocity3::zero(),
        Mass::new::<kilogram>(1.0),
    )
}

fn adaptive_integrators(min_h: f64, max_h: f64) -> [(ProfiledIntegrator, usize); 2] {
    [
        (
            ProfiledIntegrator::Dopri54Adaptive(Box::new(
                Dopri54Adaptive::new(1e-10, 1e-10, min_h, max_h).expect("DOPRI54"),
            )),
            7,
        ),
        (
            ProfiledIntegrator::Dopri853Adaptive(Box::new(
                Dopri853Adaptive::new(1e-10, 1e-10, min_h, max_h).expect("DOP853"),
            )),
            12,
        ),
    ]
}

fn fixed_integrators() -> [(ProfiledIntegrator, usize); 3] {
    [
        (ProfiledIntegrator::Rk4(Rk4FixedStep), 4),
        (ProfiledIntegrator::Dopri54Fixed(Dopri54FixedStep), 6),
        (ProfiledIntegrator::Dopri853Fixed(Dopri853FixedStep), 12),
    ]
}

fn model_error() -> ModelEvalError {
    ModelEvalError::InvalidState {
        model: ModelId::new(0xacce_0001),
        reason: "synthetic observer failure".into(),
    }
}

#[test]
fn fixed_integrators_report_one_completed_step() {
    for (integrator, _) in fixed_integrators() {
        let mut accepted = Vec::new();
        let end = integrator
            .advance_with_accepted_steps(
                &initial(),
                |_s, _t| Ok(PointMassDerivative::zero()),
                Duration::from_seconds(0.5),
                |state| {
                    accepted.push(*state);
                    Ok(())
                },
            )
            .expect("fixed step");
        assert_eq!(accepted.len(), 1);
        assert_eq!(accepted[0].time, end.time);
        assert_eq!(end.time.as_seconds(), 0.5);
    }
}

#[test]
fn adaptive_integrators_report_every_accepted_substep() {
    for (integrator, _) in adaptive_integrators(1e-6, 0.125) {
        let mut accepted_times = Vec::new();
        integrator
            .advance_with_accepted_steps(
                &initial(),
                |_s, _t| Ok(PointMassDerivative::zero()),
                Duration::from_seconds(0.5),
                |state| {
                    accepted_times.push(state.time.as_seconds());
                    Ok(())
                },
            )
            .expect("adaptive step");
        assert_eq!(accepted_times, [0.125, 0.25, 0.375, 0.5]);
    }
}

#[test]
fn source_term_profile_forwards_each_adaptive_observation() {
    let trajectory = ProfiledIntegrator::Dopri54Adaptive(Box::new(
        Dopri54Adaptive::new(1e-10, 1e-10, 1e-6, 0.125).expect("DOPRI54"),
    ));
    let integrator = ProfiledIntegrator::with_source_terms(
        SourceTermCouplingProfile::PartitionedHypersonic,
        trajectory,
        SourceTermProfile::new(4, 2, 1e-10, 12).expect("source terms"),
    );
    let mut accepted = 0;
    integrator
        .advance_with_accepted_steps(
            &initial(),
            |_s, _t| Ok(PointMassDerivative::zero()),
            Duration::from_seconds(0.5),
            |_| {
                accepted += 1;
                Ok(())
            },
        )
        .expect("wrapped adaptive step");
    assert_eq!(accepted, 4);
}

#[test]
fn rejected_trials_do_not_notify_observers() {
    for (integrator, stages_per_trial) in adaptive_integrators(1e-8, 0.5) {
        let evaluations = Cell::new(0_usize);
        let mut accepted = Vec::new();
        let mut state = initial();
        state.velocity.vector.x = 1.0;
        integrator
            .advance_with_accepted_steps(
                &state,
                |state, _t| {
                    evaluations.set(evaluations.get() + 1);
                    Ok(PointMassDerivative::new(
                        state.velocity.vector,
                        Vector3::new(-100.0 * state.velocity.vector.x, 0.0, 0.0),
                        0.0,
                    ))
                },
                Duration::from_seconds(0.5),
                |state| {
                    accepted.push((state.time.as_seconds(), evaluations.get()));
                    // Exact dissipative solution: rejected large trials have
                    // wildly wrong velocities and must never reach this path.
                    assert_abs_diff_eq!(
                        state.velocity.vector.x,
                        (-100.0 * state.time.as_seconds()).exp(),
                        epsilon = 1e-8
                    );
                    Ok(())
                },
            )
            .expect("adaptive decay");
        assert!(!accepted.is_empty());
        assert!(
            accepted[0].1 > stages_per_trial,
            "initial trial must reject"
        );
        assert!(evaluations.get() > accepted.len() * stages_per_trial);
        assert!(accepted.windows(2).all(|pair| pair[0].0 < pair[1].0));
        assert_abs_diff_eq!(accepted.last().expect("endpoint").0, 0.5, epsilon = 1e-14);
    }
}

#[test]
fn accepted_history_updates_change_the_next_substep_derivative() {
    for (integrator, _) in adaptive_integrators(0.25, 0.25) {
        let acceleration = Cell::new(0.0);
        let mut accepted = 0;
        let end = integrator
            .advance_with_accepted_steps(
                &initial(),
                |state, _t| {
                    Ok(PointMassDerivative::new(
                        state.velocity.vector,
                        Vector3::new(acceleration.get(), 0.0, 0.0),
                        0.0,
                    ))
                },
                Duration::from_seconds(0.5),
                |_| {
                    accepted += 1;
                    acceleration.set(2.0);
                    Ok(())
                },
            )
            .expect("piecewise acceleration");
        assert_eq!(accepted, 2);
        // First quarter-second: stationary. Second: a=2 m/s^2. Reusing
        // a pre-observer FSAL derivative would corrupt this exact result.
        assert_abs_diff_eq!(end.velocity.vector.x, 0.5, epsilon = 1e-14);
        assert_abs_diff_eq!(end.position.vector.x, 0.0625, epsilon = 1e-14);
    }
}

#[test]
fn observer_error_stops_subsequent_evaluations() {
    for (integrator, stages_per_trial) in fixed_integrators()
        .into_iter()
        .chain(adaptive_integrators(0.25, 0.25))
    {
        let evaluations = Cell::new(0);
        let mut observations = 0;
        let error = integrator
            .advance_with_accepted_steps(
                &initial(),
                |_s, _t| {
                    evaluations.set(evaluations.get() + 1);
                    Ok(PointMassDerivative::zero())
                },
                Duration::from_seconds(0.5),
                |_| {
                    observations += 1;
                    Err(model_error())
                },
            )
            .expect_err("observer error propagates");
        assert_eq!(error, IntegratorError::ModelEval(model_error()));
        assert_eq!(observations, 1);
        assert_eq!(evaluations.get(), stages_per_trial);
    }
}

#[test]
fn derivative_error_never_reports_an_unaccepted_state() {
    for (integrator, _) in fixed_integrators()
        .into_iter()
        .chain(adaptive_integrators(0.25, 0.25))
    {
        let mut observations = 0;
        let error = integrator
            .advance_with_accepted_steps(
                &initial(),
                |_s, _t| Err(model_error()),
                Duration::from_seconds(0.5),
                |_| {
                    observations += 1;
                    Ok(())
                },
            )
            .expect_err("derivative error propagates");
        assert_eq!(error, IntegratorError::ModelEval(model_error()));
        assert_eq!(observations, 0);
    }
}

#[test]
fn invalid_step_or_start_never_evaluates_models_or_observers() {
    for (integrator, _) in fixed_integrators()
        .into_iter()
        .chain(adaptive_integrators(0.25, 0.25))
    {
        let evaluations = Cell::new(0);
        let mut observations = 0;
        for seconds in [0.0, -0.5, f64::INFINITY, f64::NAN] {
            let error = integrator
                .advance_with_accepted_steps(
                    &initial(),
                    |_s, _t| {
                        evaluations.set(evaluations.get() + 1);
                        Ok(PointMassDerivative::zero())
                    },
                    Duration::from_seconds(seconds),
                    |_| {
                        observations += 1;
                        Ok(())
                    },
                )
                .expect_err("invalid step");
            assert!(matches!(error, IntegratorError::InvalidStep { .. }));
        }
        let mut invalid = initial();
        invalid.mass = Mass::new::<kilogram>(0.0);
        let error = integrator
            .advance_with_accepted_steps(
                &invalid,
                |_s, _t| {
                    evaluations.set(evaluations.get() + 1);
                    Ok(PointMassDerivative::zero())
                },
                Duration::from_seconds(0.5),
                |_| {
                    observations += 1;
                    Ok(())
                },
            )
            .expect_err("invalid start");
        assert_eq!(error, IntegratorError::NonFiniteState);
        assert_eq!(evaluations.get(), 0);
        assert_eq!(observations, 0);
    }
}

#[test]
fn a_later_failure_can_follow_an_already_reported_substep() {
    for (integrator, _) in adaptive_integrators(0.25, 0.25) {
        let committed = Cell::new(false);
        let mut observations = 0;
        let error = integrator
            .advance_with_accepted_steps(
                &initial(),
                |_s, _t| {
                    if committed.get() {
                        Err(model_error())
                    } else {
                        Ok(PointMassDerivative::zero())
                    }
                },
                Duration::from_seconds(0.5),
                |state| {
                    assert_eq!(state.time.as_seconds(), 0.25);
                    committed.set(true);
                    observations += 1;
                    Ok(())
                },
            )
            .expect_err("second substep fails");
        assert_eq!(error, IntegratorError::ModelEval(model_error()));
        assert_eq!(observations, 1);
        // External history remains the caller's responsibility on failure.
        assert!(committed.get());
    }
}

#[test]
fn noop_observer_preserves_plain_advance_bits() {
    let plain = fixed_integrators()
        .into_iter()
        .chain(adaptive_integrators(1e-8, 0.25));
    let observed = fixed_integrators()
        .into_iter()
        .chain(adaptive_integrators(1e-8, 0.25));
    for ((plain, _), (observed, _)) in plain.zip(observed) {
        let derive = |state: &PointMassState, time: SimTime| {
            Ok(PointMassDerivative::new(
                state.velocity.vector,
                Vector3::new(time.as_seconds().sin(), 0.0, 0.0),
                0.0,
            ))
        };
        let dt = Duration::from_seconds(0.5);
        let expected = plain.advance(&initial(), derive, dt).expect("plain step");
        let actual = observed
            .advance_with_accepted_steps(&initial(), derive, dt, |_| Ok(()))
            .expect("observed step");
        assert_eq!(
            expected.time.as_seconds().to_bits(),
            actual.time.as_seconds().to_bits()
        );
        assert_eq!(
            expected.position.vector.x.to_bits(),
            actual.position.vector.x.to_bits()
        );
        assert_eq!(
            expected.velocity.vector.x.to_bits(),
            actual.velocity.vector.x.to_bits()
        );
        assert_eq!(expected.mass_kg().to_bits(), actual.mass_kg().to_bits());
    }
}
