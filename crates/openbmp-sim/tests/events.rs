//! Event-trigger unit tests.
//!
//! Covers each [`openbmp_sim::BuiltInEventTrigger`] variant under
//! prescribed [`openbmp_sim::EventEvalState`] sequences. Crossing
//! semantics: every trigger returns `false` on step 0 (no previous
//! snapshot) and fires on the step where the monitored value
//! transitions across the trigger threshold.

#![allow(clippy::float_cmp, clippy::expect_used)]

use openbmp_core::{SimTime, StepIndex};
use openbmp_sim::{
    BuiltInEventTrigger, EventEvalState, EventId, EventScalars, EventTrigger, PhaseId,
};

const ANY_TIME: SimTime = SimTime::ZERO;
const ANY_STEP: StepIndex = StepIndex::ZERO;

fn scalars(time_s: f64, alt: f64, v_z: f64, speed: f64, mf: f64, q: f64) -> EventScalars {
    EventScalars {
        time_s,
        altitude_m: alt,
        vertical_velocity_m_s: v_z,
        velocity_m_s: speed,
        mass_fraction: mf,
        dynamic_pressure_pa: q,
    }
}

fn step(curr: EventScalars, prev: Option<EventScalars>) -> EventEvalState {
    EventEvalState {
        current: curr,
        previous: prev,
        current_phase: None,
    }
}

// ---------------------------------------------------------------------
// AtTime
// ---------------------------------------------------------------------

#[test]
fn at_time_does_not_fire_on_step_zero() {
    // No previous-step snapshot ⇒ no crossing detection possible.
    let s = step(scalars(0.0, 0.0, 100.0, 100.0, 1.0, 0.0), None);
    assert!(!BuiltInEventTrigger::AtTime { time_s: 0.0 }.fired(&s, ANY_TIME, ANY_STEP));
}

#[test]
fn at_time_fires_on_first_crossing() {
    let prev = scalars(0.99, 0.0, 0.0, 0.0, 1.0, 0.0);
    let curr = scalars(1.00, 0.0, 0.0, 0.0, 1.0, 0.0);
    let s = step(curr, Some(prev));
    assert!(BuiltInEventTrigger::AtTime { time_s: 1.0 }.fired(&s, ANY_TIME, ANY_STEP));
}

#[test]
fn at_time_does_not_re_fire_after_crossing() {
    let prev = scalars(1.5, 0.0, 0.0, 0.0, 1.0, 0.0);
    let curr = scalars(2.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    let s = step(curr, Some(prev));
    assert!(!BuiltInEventTrigger::AtTime { time_s: 1.0 }.fired(&s, ANY_TIME, ANY_STEP));
}

#[test]
fn at_time_does_not_fire_before_threshold() {
    let prev = scalars(0.5, 0.0, 0.0, 0.0, 1.0, 0.0);
    let curr = scalars(0.6, 0.0, 0.0, 0.0, 1.0, 0.0);
    let s = step(curr, Some(prev));
    assert!(!BuiltInEventTrigger::AtTime { time_s: 1.0 }.fired(&s, ANY_TIME, ANY_STEP));
}

// ---------------------------------------------------------------------
// AtAltitudeAscending / AtAltitudeDescending
// ---------------------------------------------------------------------

#[test]
fn at_altitude_ascending_fires_on_upward_crossing() {
    let prev = scalars(1.0, 95.0, 50.0, 50.0, 1.0, 0.0);
    let curr = scalars(1.1, 105.0, 50.0, 50.0, 1.0, 0.0);
    let s = step(curr, Some(prev));
    assert!(
        BuiltInEventTrigger::AtAltitudeAscending { meters: 100.0 }.fired(&s, ANY_TIME, ANY_STEP)
    );
}

#[test]
fn at_altitude_ascending_does_not_fire_going_down() {
    let prev = scalars(5.0, 105.0, -10.0, 10.0, 1.0, 0.0);
    let curr = scalars(5.1, 95.0, -10.0, 10.0, 1.0, 0.0);
    let s = step(curr, Some(prev));
    assert!(
        !BuiltInEventTrigger::AtAltitudeAscending { meters: 100.0 }.fired(&s, ANY_TIME, ANY_STEP)
    );
}

#[test]
fn at_altitude_descending_fires_on_downward_crossing() {
    let prev = scalars(5.0, 105.0, -10.0, 10.0, 1.0, 0.0);
    let curr = scalars(5.1, 95.0, -10.0, 10.0, 1.0, 0.0);
    let s = step(curr, Some(prev));
    assert!(
        BuiltInEventTrigger::AtAltitudeDescending { meters: 100.0 }.fired(&s, ANY_TIME, ANY_STEP)
    );
}

#[test]
fn at_altitude_descending_does_not_fire_going_up() {
    let prev = scalars(1.0, 95.0, 50.0, 50.0, 1.0, 0.0);
    let curr = scalars(1.1, 105.0, 50.0, 50.0, 1.0, 0.0);
    let s = step(curr, Some(prev));
    assert!(
        !BuiltInEventTrigger::AtAltitudeDescending { meters: 100.0 }.fired(&s, ANY_TIME, ANY_STEP)
    );
}

// ---------------------------------------------------------------------
// AtApogee
// ---------------------------------------------------------------------

#[test]
fn at_apogee_fires_on_velocity_sign_flip() {
    let prev = scalars(5.0, 150.0, 5.0, 5.0, 0.5, 0.0);
    let curr = scalars(5.1, 151.0, -1.0, 1.0, 0.5, 0.0);
    let s = step(curr, Some(prev));
    assert!(BuiltInEventTrigger::AtApogee.fired(&s, ANY_TIME, ANY_STEP));
}

#[test]
fn at_apogee_does_not_fire_during_ascent() {
    let prev = scalars(1.0, 50.0, 100.0, 100.0, 1.0, 0.0);
    let curr = scalars(1.1, 60.0, 95.0, 95.0, 1.0, 0.0);
    let s = step(curr, Some(prev));
    assert!(!BuiltInEventTrigger::AtApogee.fired(&s, ANY_TIME, ANY_STEP));
}

#[test]
fn at_apogee_does_not_fire_during_descent() {
    let prev = scalars(10.0, 100.0, -10.0, 10.0, 0.5, 0.0);
    let curr = scalars(10.1, 99.0, -11.0, 11.0, 0.5, 0.0);
    let s = step(curr, Some(prev));
    assert!(!BuiltInEventTrigger::AtApogee.fired(&s, ANY_TIME, ANY_STEP));
}

#[test]
fn at_apogee_fires_when_velocity_reaches_zero() {
    // The apogee step is the first step with v_z <= 0 after positive
    // v_z. Exactly-zero velocity at the post-step is the apogee.
    let prev = scalars(5.0, 150.0, 0.5, 0.5, 0.5, 0.0);
    let curr = scalars(5.1, 150.5, 0.0, 0.0, 0.5, 0.0);
    let s = step(curr, Some(prev));
    assert!(BuiltInEventTrigger::AtApogee.fired(&s, ANY_TIME, ANY_STEP));
}

// ---------------------------------------------------------------------
// AtMassFraction
// ---------------------------------------------------------------------

#[test]
fn at_mass_fraction_fires_on_threshold_cross() {
    let prev = scalars(2.0, 100.0, 50.0, 50.0, 0.51, 0.0);
    let curr = scalars(2.1, 105.0, 49.0, 49.0, 0.49, 0.0);
    let s = step(curr, Some(prev));
    assert!(BuiltInEventTrigger::AtMassFraction { remaining: 0.5 }.fired(&s, ANY_TIME, ANY_STEP));
}

#[test]
fn at_mass_fraction_does_not_fire_above_threshold() {
    let prev = scalars(1.0, 50.0, 100.0, 100.0, 0.9, 0.0);
    let curr = scalars(1.1, 60.0, 95.0, 95.0, 0.85, 0.0);
    let s = step(curr, Some(prev));
    assert!(!BuiltInEventTrigger::AtMassFraction { remaining: 0.5 }.fired(&s, ANY_TIME, ANY_STEP));
}

// ---------------------------------------------------------------------
// AtVelocity
// ---------------------------------------------------------------------

#[test]
fn at_velocity_fires_on_upward_speed_crossing() {
    let prev = scalars(1.0, 50.0, 100.0, 245.0, 1.0, 0.0);
    let curr = scalars(1.1, 60.0, 100.0, 255.0, 1.0, 0.0);
    let s = step(curr, Some(prev));
    assert!(
        BuiltInEventTrigger::AtVelocity {
            velocity_m_s: 250.0
        }
        .fired(&s, ANY_TIME, ANY_STEP)
    );
}

#[test]
fn at_velocity_does_not_fire_on_downward_speed_crossing() {
    let prev = scalars(1.0, 50.0, 100.0, 255.0, 1.0, 0.0);
    let curr = scalars(1.1, 60.0, 100.0, 245.0, 1.0, 0.0);
    let s = step(curr, Some(prev));
    assert!(
        !BuiltInEventTrigger::AtVelocity {
            velocity_m_s: 250.0
        }
        .fired(&s, ANY_TIME, ANY_STEP)
    );
}

// ---------------------------------------------------------------------
// AtDynamicPressure
// ---------------------------------------------------------------------

#[test]
fn at_dynamic_pressure_rising_edge_fires() {
    let prev = scalars(1.0, 50.0, 100.0, 100.0, 1.0, 9_500.0);
    let curr = scalars(1.1, 60.0, 100.0, 100.0, 1.0, 10_500.0);
    let s = step(curr, Some(prev));
    assert!(
        BuiltInEventTrigger::AtDynamicPressure {
            pa: 10_000.0,
            falling: false,
        }
        .fired(&s, ANY_TIME, ANY_STEP)
    );
}

#[test]
fn at_dynamic_pressure_rising_edge_ignores_falling_crossing() {
    // Same crossing, but on the way down (q above threshold → below).
    // Rising-edge variant should not fire.
    let prev = scalars(8.0, 200.0, -10.0, 10.0, 0.4, 10_500.0);
    let curr = scalars(8.1, 199.0, -11.0, 11.0, 0.4, 9_500.0);
    let s = step(curr, Some(prev));
    assert!(
        !BuiltInEventTrigger::AtDynamicPressure {
            pa: 10_000.0,
            falling: false,
        }
        .fired(&s, ANY_TIME, ANY_STEP)
    );
}

#[test]
fn at_dynamic_pressure_falling_edge_fires() {
    let prev = scalars(8.0, 200.0, -10.0, 10.0, 0.4, 10_500.0);
    let curr = scalars(8.1, 199.0, -11.0, 11.0, 0.4, 9_500.0);
    let s = step(curr, Some(prev));
    assert!(
        BuiltInEventTrigger::AtDynamicPressure {
            pa: 10_000.0,
            falling: true,
        }
        .fired(&s, ANY_TIME, ANY_STEP)
    );
}

#[test]
fn at_dynamic_pressure_falling_edge_ignores_rising_crossing() {
    let prev = scalars(1.0, 50.0, 100.0, 100.0, 1.0, 9_500.0);
    let curr = scalars(1.1, 60.0, 100.0, 100.0, 1.0, 10_500.0);
    let s = step(curr, Some(prev));
    assert!(
        !BuiltInEventTrigger::AtDynamicPressure {
            pa: 10_000.0,
            falling: true,
        }
        .fired(&s, ANY_TIME, ANY_STEP)
    );
}

// ---------------------------------------------------------------------
// Identifier round-trip / determinism
// ---------------------------------------------------------------------

#[test]
fn phase_id_round_trip() {
    let id = PhaseId::from_path("mission.phases.ascent");
    assert_eq!(PhaseId::new(id.value()), id);
}

#[test]
fn event_id_round_trip() {
    let id = EventId::from_path("mission.events.at_apogee_marker");
    assert_eq!(EventId::new(id.value()), id);
}

#[test]
fn ids_are_path_sensitive_not_order_sensitive() {
    // The two phases differ only in id. Their FNV hashes diverge —
    // this is the load-bearing invariant for declaration-order
    // independence.
    let a = PhaseId::from_path("mission.phases.ascent");
    let b = PhaseId::from_path("mission.phases.descent");
    assert_ne!(a, b);

    // Two events with similar prefix also diverge.
    let x = EventId::from_path("mission.events.at_apogee_marker");
    let y = EventId::from_path("mission.events.at_burnout_marker");
    assert_ne!(x, y);
}
