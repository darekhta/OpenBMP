//! Stop-condition trait and implementations.
//!
//! Stop conditions are checked before each kernel step and, for
//! crossing-sensitive stops, after each successful step. When a
//! condition fires, the kernel records the [`crate::StopReason`] and
//! `step()` becomes a no-op until the kernel is recreated.
//!
//! `StopCondition` is generic over
//! `S: SimState` so the same trait serves point-mass and rigid-body
//! kernels. The simple stops (`AlwaysContinue`, `EndTime`,
//! `MaxSteps`) only need `state.time()` from the trait, so they
//! impl `StopCondition<S>` for **any** `S`. Translational stops such
//! as [`GroundImpact`] also bound `S: TranslationalState`.

use openbmp_core::{SimTime, StepIndex};

use openbmp_models::{SimState, TranslationalState};

use crate::error::StopReason;

/// Trait implemented by stop conditions.
///
/// Returns `Some(reason)` when the kernel should halt, `None` to
/// continue. The kernel evaluates [`Self::evaluate`] at the start of
/// each step, with the state that step would advance from. It
/// evaluates [`Self::evaluate_step`] after each successful step, with
/// both the previous and current state, so crossing detectors can halt
/// on the first step that crosses a threshold.
///
/// Generic over the [`SimState`] the kernel integrates. Most stops
/// only inspect `state.time()` and the step counter, so they impl
/// `StopCondition<S>` for any `S`.
pub trait StopCondition<S: SimState> {
    /// Evaluate the condition.
    fn evaluate(&self, state: &S, step: StepIndex) -> Option<StopReason>;

    /// Evaluate a completed step transition.
    ///
    /// The default checks the current state, which makes scalar
    /// conditions such as [`EndTime`] fire as soon as a step reaches
    /// the terminal state. Crossing-sensitive conditions can override
    /// this to inspect both sides of the step.
    fn evaluate_step(&self, _previous: &S, current: &S, step: StepIndex) -> Option<StopReason> {
        self.evaluate(current, step)
    }
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
    /// Inclusive stop time. The kernel halts once the state time is
    /// `>= stop_at`.
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

/// Stop when the translational altitude proxy crosses ground while
/// descending.
///
/// This uses `position.z` as the altitude proxy, matching the
/// simulator's existing `AtAltitudeDescending` event convention. It is
/// intended for local-flat / toy-fixed-earth scenarios where `+z` is
/// altitude above the launch reference. Orbital callers whose ECI
/// `z` component is not altitude should not install this condition.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GroundImpact {
    /// Ground altitude threshold in metres.
    pub ground_altitude_m: f64,
    enabled: bool,
}

impl Default for GroundImpact {
    fn default() -> Self {
        Self::sea_level()
    }
}

impl GroundImpact {
    /// Construct a sea-level ground-impact detector.
    #[must_use]
    pub const fn sea_level() -> Self {
        Self::new(0.0)
    }

    /// Construct a ground-impact detector with a custom ground
    /// altitude threshold.
    #[must_use]
    pub const fn new(ground_altitude_m: f64) -> Self {
        Self {
            ground_altitude_m,
            enabled: true,
        }
    }

    /// Construct a disabled detector.
    ///
    /// This is useful when a higher-level builder wants a stable stop
    /// condition type but needs to opt out for frames where
    /// `position.z` is not altitude.
    #[must_use]
    pub const fn disabled() -> Self {
        Self {
            ground_altitude_m: 0.0,
            enabled: false,
        }
    }

    fn reason<S>(&self, state: &S, step: StepIndex) -> StopReason
    where
        S: SimState + TranslationalState,
    {
        StopReason::GroundImpact {
            step,
            time_s: state.time().as_seconds(),
            altitude_m: state.position_eci().vector.z,
            ground_altitude_m: self.ground_altitude_m,
        }
    }
}

impl<S> StopCondition<S> for GroundImpact
where
    S: SimState + TranslationalState,
{
    fn evaluate(&self, _state: &S, _step: StepIndex) -> Option<StopReason> {
        None
    }

    fn evaluate_step(&self, previous: &S, current: &S, step: StepIndex) -> Option<StopReason> {
        if !self.enabled {
            return None;
        }

        let previous_altitude_m = previous.position_eci().vector.z;
        let current_altitude_m = current.position_eci().vector.z;
        let current_vertical_velocity_m_s = current.velocity_eci().vector.z;
        let crossed_from_above = previous_altitude_m > self.ground_altitude_m
            && current_altitude_m <= self.ground_altitude_m;
        if crossed_from_above && current_vertical_velocity_m_s <= 0.0 {
            Some(self.reason(current, step))
        } else {
            None
        }
    }
}

/// Stop when either of two stop conditions fires.
///
/// Conditions are evaluated in declaration order; when both would fire
/// on the same state, `first` wins.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AnyStop<A, B> {
    /// First condition to evaluate.
    pub first: A,
    /// Second condition to evaluate if the first does not fire.
    pub second: B,
}

impl<A, B> AnyStop<A, B> {
    /// Construct.
    #[must_use]
    pub const fn new(first: A, second: B) -> Self {
        Self { first, second }
    }
}

impl<S, A, B> StopCondition<S> for AnyStop<A, B>
where
    S: SimState,
    A: StopCondition<S>,
    B: StopCondition<S>,
{
    fn evaluate(&self, state: &S, step: StepIndex) -> Option<StopReason> {
        self.first
            .evaluate(state, step)
            .or_else(|| self.second.evaluate(state, step))
    }

    fn evaluate_step(&self, previous: &S, current: &S, step: StepIndex) -> Option<StopReason> {
        self.first
            .evaluate_step(previous, current, step)
            .or_else(|| self.second.evaluate_step(previous, current, step))
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use openbmp_core::{
        AngularVelocity3, Body, Eci, Position3, Quaternion, UnitQuaternion, Velocity3,
    };
    use openbmp_state::{MassProperties, PointMassState, RigidBodyState};
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

    #[test]
    fn ground_impact_fires_on_downward_crossing_for_point_mass() {
        let cond = GroundImpact::sea_level();
        let previous = PointMassState::new(
            SimTime::ZERO,
            Position3::new(0.0, 0.0, 1.0),
            Velocity3::new(0.0, 0.0, -1.0),
            Mass::new::<kilogram>(1.0),
        );
        let current = PointMassState::new(
            SimTime::from_seconds(0.2),
            Position3::new(0.0, 0.0, -0.1),
            Velocity3::new(0.0, 0.0, -2.0),
            Mass::new::<kilogram>(1.0),
        );

        let reason = <GroundImpact as StopCondition<PointMassState>>::evaluate_step(
            &cond,
            &previous,
            &current,
            StepIndex::new(2),
        )
        .expect("ground impact should fire");

        match reason {
            StopReason::GroundImpact {
                step,
                time_s,
                altitude_m,
                ground_altitude_m,
            } => {
                assert_eq!(step.value(), 2);
                assert_abs_diff(time_s, 0.2);
                assert_abs_diff(altitude_m, -0.1);
                assert_abs_diff(ground_altitude_m, 0.0);
            }
            other => panic!("unexpected stop reason: {other:?}"),
        }
    }

    #[test]
    fn ground_impact_does_not_fire_for_launch_pad_rest_state() {
        let cond = GroundImpact::sea_level();
        let state = state_at(0.0);
        assert!(
            <GroundImpact as StopCondition<PointMassState>>::evaluate(
                &cond,
                &state,
                StepIndex::ZERO
            )
            .is_none()
        );
    }

    #[test]
    fn ground_impact_does_not_fire_while_moving_upward() {
        let cond = GroundImpact::sea_level();
        let previous = PointMassState::new(
            SimTime::ZERO,
            Position3::new(0.0, 0.0, -0.1),
            Velocity3::new(0.0, 0.0, 2.0),
            Mass::new::<kilogram>(1.0),
        );
        let current = PointMassState::new(
            SimTime::from_seconds(0.2),
            Position3::new(0.0, 0.0, 0.1),
            Velocity3::new(0.0, 0.0, 2.0),
            Mass::new::<kilogram>(1.0),
        );

        assert!(
            <GroundImpact as StopCondition<PointMassState>>::evaluate_step(
                &cond,
                &previous,
                &current,
                StepIndex::new(2),
            )
            .is_none()
        );
    }

    #[test]
    fn ground_impact_supports_rigid_body_state() {
        let cond = GroundImpact::sea_level();
        let mass_props = MassProperties::with_diagonal_inertia(
            Mass::new::<kilogram>(1.0),
            Position3::origin(),
            1.0,
            1.0,
            1.0,
        );
        let previous = RigidBodyState::new(
            SimTime::ZERO,
            Position3::new(0.0, 0.0, 1.0),
            Velocity3::new(0.0, 0.0, -1.0),
            Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
            AngularVelocity3::zero(),
            mass_props,
        );
        let current = RigidBodyState::new(
            SimTime::from_seconds(0.2),
            Position3::new(0.0, 0.0, -0.1),
            Velocity3::new(0.0, 0.0, -2.0),
            Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
            AngularVelocity3::zero(),
            mass_props,
        );

        assert!(
            <GroundImpact as StopCondition<RigidBodyState>>::evaluate_step(
                &cond,
                &previous,
                &current,
                StepIndex::new(2),
            )
            .is_some()
        );
    }

    #[test]
    fn any_stop_prefers_first_condition() {
        let cond = AnyStop::new(MaxSteps::new(0), EndTime::new(SimTime::ZERO));
        let reason = <AnyStop<MaxSteps, EndTime> as StopCondition<PointMassState>>::evaluate(
            &cond,
            &state_at(0.0),
            StepIndex::ZERO,
        )
        .expect("both conditions fire");

        assert!(matches!(
            reason,
            StopReason::UserRequested { label } if label == "max-steps"
        ));
    }

    fn assert_abs_diff(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1.0e-12,
            "actual={actual}, expected={expected}"
        );
    }
}
