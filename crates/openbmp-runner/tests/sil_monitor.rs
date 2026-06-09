//! Increment B verification: the SIL observation hook is a read-only tap.
//!
//! An installed [`SilMonitor`] must (a) observe the in-loop flight
//! controller's published estimates each tick and (b) leave the run
//! byte-identical to a plain [`openbmp_runner::run`] — proving the hook is
//! a non-perturbing observation tap, not a feedback path.

#![allow(clippy::expect_used, clippy::panic)]

use openbmp_core::StepIndex;
use openbmp_runner::sil::{FcObservation, SilMonitor};
use openbmp_sensors::SensorTruth;

#[derive(Default)]
struct CountingMonitor {
    ticks: usize,
    saw_position: bool,
    saw_estimator: bool,
    saw_attitude: bool,
}

impl SilMonitor for CountingMonitor {
    fn observe(&mut self, _step: StepIndex, _truth: &SensorTruth, observation: &FcObservation) {
        self.ticks += 1;
        self.saw_position |= observation.position.is_some();
        self.saw_estimator |= observation.estimator.is_some();
        self.saw_attitude |= observation.attitude.is_some();
    }
}

#[test]
fn installed_monitor_observes_fc_and_does_not_perturb_run() {
    let scenario = openbmp_scenario::Scenario::from_toml_str(include_str!(
        "fixtures/fc-reference-telemetry-attitude-hold.toml"
    ))
    .expect("scenario parses");

    // Baseline: the plain run() path (== monitor None).
    let baseline = openbmp_runner::run(&scenario).expect("baseline run");

    // Same-commit equality: an installed read-only monitor must produce a
    // byte-identical telemetry table.
    let mut monitor = CountingMonitor::default();
    let observed =
        openbmp_runner::run_with_monitor(&scenario, &mut monitor).expect("observed run");

    assert_eq!(
        baseline.table, observed.table,
        "an installed read-only SIL monitor must not change the telemetry bytes"
    );
    assert_eq!(
        baseline.final_step, observed.final_step,
        "observed run must reach the same final step"
    );

    // The taps (increment A) are reachable through the observation and
    // populated on the in-loop run.
    assert!(monitor.ticks > 0, "monitor must observe at least one tick");
    assert!(
        monitor.saw_position,
        "FC translational estimate must be observable on an [fc] run"
    );
    assert!(
        monitor.saw_estimator,
        "FC estimator-health status must be observable on an [fc] run"
    );
    assert!(
        monitor.saw_attitude,
        "FC attitude estimate must be observable on an [fc] run"
    );
}
