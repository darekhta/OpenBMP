//! Stop-condition trait and Phase-1.3 implementations.
//!
//! Stop conditions are checked **before** each kernel step. When a
//! condition fires, the kernel records the [`crate::StopReason`] and
//! `step()` becomes a no-op until the kernel is recreated.

use openbmp_core::{SimTime, StepIndex};
use openbmp_state::PointMassState;

use crate::error::StopReason;

/// Trait implemented by stop conditions.
///
/// Returns `Some(reason)` when the kernel should halt, `None` to
/// continue. The kernel evaluates this at the **start** of each step;
/// the state passed in is the state that step would have advanced
/// from.
pub trait StopCondition {
    /// Evaluate the condition.
    fn evaluate(&self, state: &PointMassState, step: StepIndex) -> Option<StopReason>;
}

/// Stop condition that never fires.
///
/// Useful as a placeholder when the user drives stopping externally
/// or when running a fixed number of steps via a step-count loop.
#[derive(Copy, Clone, Debug, Default)]
pub struct AlwaysContinue;

impl StopCondition for AlwaysContinue {
    fn evaluate(&self, _state: &PointMassState, _step: StepIndex) -> Option<StopReason> {
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

impl StopCondition for EndTime {
    fn evaluate(&self, state: &PointMassState, _step: StepIndex) -> Option<StopReason> {
        if state.time.as_seconds() >= self.stop_at.as_seconds() {
            Some(StopReason::EndTime {
                reached_s: state.time.as_seconds(),
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

impl StopCondition for MaxSteps {
    fn evaluate(&self, _state: &PointMassState, step: StepIndex) -> Option<StopReason> {
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
        assert!(
            AlwaysContinue
                .evaluate(&state_at(0.0), StepIndex::ZERO)
                .is_none()
        );
        assert!(
            AlwaysContinue
                .evaluate(&state_at(1.0e9), StepIndex::new(1_000_000))
                .is_none()
        );
    }

    #[test]
    fn end_time_does_not_fire_before_stop() {
        let cond = EndTime::new(SimTime::from_seconds(10.0));
        assert!(cond.evaluate(&state_at(0.0), StepIndex::ZERO).is_none());
        assert!(
            cond.evaluate(&state_at(9.99), StepIndex::new(999))
                .is_none()
        );
    }

    #[test]
    fn end_time_fires_at_or_past_stop() {
        let cond = EndTime::new(SimTime::from_seconds(10.0));
        let reason = cond
            .evaluate(&state_at(10.0), StepIndex::new(1000))
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
        assert!(cond.evaluate(&state_at(0.0), StepIndex::new(99)).is_none());
        assert!(cond.evaluate(&state_at(0.0), StepIndex::new(100)).is_some());
        assert!(cond.evaluate(&state_at(0.0), StepIndex::new(101)).is_some());
    }
}
