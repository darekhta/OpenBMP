//! Stop-condition trait and Phase-1.3 implementations.
//!
//! Stop conditions are checked **before** each kernel step. When a
//! condition fires, the kernel records the [`crate::StopReason`] and
//! `step()` becomes a no-op until the kernel is recreated.
//!
//! Phase-2 generalisation: `StopCondition` is now generic over
//! `S: SimState` so the same trait serves point-mass and rigid-body
//! kernels. The Phase-1 stops (`AlwaysContinue`, `EndTime`,
//! `MaxSteps`) only need `state.time()` from the trait, so they
//! impl `StopCondition<S>` for **any** `S`.

use openbmp_core::{SimTime, StepIndex};

use crate::error::StopReason;
use crate::integrator::SimState;

/// Trait implemented by stop conditions.
///
/// Returns `Some(reason)` when the kernel should halt, `None` to
/// continue. The kernel evaluates this at the **start** of each step;
/// the state passed in is the state that step would have advanced
/// from.
///
/// Generic over the [`SimState`] the kernel integrates. Most stops
/// only inspect `state.time()` and the step counter, so they impl
/// `StopCondition<S>` for any `S`.
pub trait StopCondition<S: SimState> {
    /// Evaluate the condition.
    fn evaluate(&self, state: &S, step: StepIndex) -> Option<StopReason>;
}

/// Stop condition that never fires.
///
/// Useful as a placeholder when the user drives stopping externally
/// or when running a fixed number of steps via a step-count loop.
#[derive(Copy, Clone, Debug, Default)]
pub struct AlwaysContinue;

impl<S: SimState> StopCondition<S> for AlwaysContinue {
    fn evaluate(&self, _state: &S, _step: StepIndex) -> Option<StopReason> {
        None
    }
}

/// Stop when the simulation time reaches or exceeds `stop_at`.
#[derive(Copy, Clone, Debug)]
pub struct EndTime {
    /// Inclusive stop time. The kernel halts on the **first** step
    /// whose start-of-step time is `>= stop_at`.
    pub stop_at: SimTime,
}

impl EndTime {
    /// Construct.
    #[must_use]
    pub const fn new(stop_at: SimTime) -> Self {
        Self { stop_at }
    }
}

impl<S: SimState> StopCondition<S> for EndTime {
    fn evaluate(&self, state: &S, _step: StepIndex) -> Option<StopReason> {
        let t = state.time();
        if t.as_seconds() >= self.stop_at.as_seconds() {
            Some(StopReason::EndTime {
                reached_s: t.as_seconds(),
            })
        } else {
            None
        }
    }
}

/// Stop after the kernel has advanced `max_steps` times.
#[derive(Copy, Clone, Debug)]
pub struct MaxSteps {
    /// The maximum number of completed steps.
    pub max_steps: u64,
}

impl MaxSteps {
    /// Construct.
    #[must_use]
    pub const fn new(max_steps: u64) -> Self {
        Self { max_steps }
    }
}

impl<S: SimState> StopCondition<S> for MaxSteps {
    fn evaluate(&self, _state: &S, step: StepIndex) -> Option<StopReason> {
        if step.value() >= self.max_steps {
            Some(StopReason::UserRequested { label: "max-steps" })
        } else {
            None
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use openbmp_core::{Position3, Velocity3};
    use openbmp_state::PointMassState;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    fn state_at(t_s: f64) -> PointMassState {
        PointMassState::new(
            SimTime::from_seconds(t_s),
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        )
    }

    #[test]
    fn always_continue_never_fires() {
        let s = state_at(0.0);
        assert!(
            <AlwaysContinue as StopCondition<PointMassState>>::evaluate(
                &AlwaysContinue,
                &s,
                StepIndex::ZERO
            )
            .is_none()
        );
        let s = state_at(1.0e9);
        assert!(
            <AlwaysContinue as StopCondition<PointMassState>>::evaluate(
                &AlwaysContinue,
                &s,
                StepIndex::new(1_000_000)
            )
            .is_none()
        );
    }

    #[test]
    fn end_time_does_not_fire_before_stop() {
        let cond = EndTime::new(SimTime::from_seconds(10.0));
        assert!(
            <EndTime as StopCondition<PointMassState>>::evaluate(
                &cond,
                &state_at(0.0),
                StepIndex::ZERO
            )
            .is_none()
        );
        assert!(
            <EndTime as StopCondition<PointMassState>>::evaluate(
                &cond,
                &state_at(9.99),
                StepIndex::new(999)
            )
            .is_none()
        );
    }

    #[test]
    fn end_time_fires_at_or_past_stop() {
        let cond = EndTime::new(SimTime::from_seconds(10.0));
        let reason = <EndTime as StopCondition<PointMassState>>::evaluate(
            &cond,
            &state_at(10.0),
            StepIndex::new(1000),
        )
        .expect("should fire at stop");
        match reason {
            StopReason::EndTime { reached_s } => {
                assert!((reached_s - 10.0).abs() < 1.0e-12);
            }
            other => panic!("unexpected stop reason: {other:?}"),
        }
    }

    #[test]
    fn max_steps_fires_on_count() {
        let cond = MaxSteps::new(100);
        let s = state_at(0.0);
        assert!(
            <MaxSteps as StopCondition<PointMassState>>::evaluate(&cond, &s, StepIndex::new(99))
                .is_none()
        );
        assert!(
            <MaxSteps as StopCondition<PointMassState>>::evaluate(&cond, &s, StepIndex::new(100))
                .is_some()
        );
        assert!(
            <MaxSteps as StopCondition<PointMassState>>::evaluate(&cond, &s, StepIndex::new(101))
                .is_some()
        );
    }
}
