//! Increment D1 verification: open-loop sensor fault injection at the
//! FC/sensor boundary.
//!
//! Two guarantees: (1) an absent or empty `[fc.sil_stimulus]` block draws
//! nothing from the stimulus RNG and is byte-identical to a plain run;
//! (2) an armed GNSS bias actually reaches the EKF (its innovation chi²
//! spikes) and shifts the estimate — proving the fault is in the loop, not
//! a pre-run scenario edit.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::Path;

use openbmp_core::StepIndex;
use openbmp_runner::sil::{FcObservation, SilMonitor};
use openbmp_scenario::Scenario;
use openbmp_sensors::SensorTruth;

const SCENARIO_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
);

/// Load the closed-loop-attitude-hold `[fc]` scenario with `extra` TOML
/// appended, resolving the scenario's relative sensor-budget paths against
/// its real directory.
fn load(extra: &str) -> Scenario {
    let base = std::fs::read_to_string(SCENARIO_PATH).expect("read scenario");
    let toml = format!("{base}\n{extra}");
    let source_dir = Path::new(SCENARIO_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

#[derive(Default)]
struct PeakMonitor {
    ticks: u64,
    max_gnss_chi2: f64,
    innovation_rejected_ticks: u64,
}

impl SilMonitor for PeakMonitor {
    fn observe(&mut self, _step: StepIndex, _truth: &SensorTruth, observation: &FcObservation) {
        self.ticks += 1;
        if let Some(estimator) = observation.estimator {
            self.max_gnss_chi2 = self.max_gnss_chi2.max(estimator.gnss_chi2);
            if estimator.innovation_rejected {
                self.innovation_rejected_ticks += 1;
            }
        }
    }
}

const GNSS_BIAS: &str = "\
[[fc.sil_stimulus.faults]]
sensor = \"gnss\"
start_s = 0.2
stop_s = 1.0
fault = { kind = \"bias\", offset = [80.0, 0.0, 0.0] }
";

#[test]
fn empty_stimulus_block_is_byte_identical() {
    let baseline = openbmp_runner::run(&load("")).expect("baseline run");
    let empty_block = openbmp_runner::run(&load("[fc.sil_stimulus]\n")).expect("empty-block run");
    assert_eq!(
        baseline.table, empty_block.table,
        "an empty [fc.sil_stimulus] block must draw nothing and stay byte-identical"
    );
}

#[test]
fn armed_gnss_bias_reaches_the_ekf() {
    let mut baseline = PeakMonitor::default();
    openbmp_runner::run_with_monitor(&load(""), &mut baseline).expect("baseline observed run");

    let mut faulted = PeakMonitor::default();
    openbmp_runner::run_with_monitor(&load(GNSS_BIAS), &mut faulted).expect("faulted observed run");

    assert!(baseline.ticks > 0 && faulted.ticks == baseline.ticks);

    // A biased GNSS measurement inflates the EKF innovation: the faulted
    // run's peak GNSS chi² must clearly exceed the nominal run's. This is
    // the fault reaching the flight software in the loop.
    assert!(
        faulted.max_gnss_chi2 > baseline.max_gnss_chi2 * 2.0 + 1.0,
        "armed GNSS bias must inflate the EKF innovation chi²: baseline {:.3}, faulted {:.3}",
        baseline.max_gnss_chi2,
        faulted.max_gnss_chi2
    );
    // The inflated innovation trips the EKF's chi² gate, so the faulted run
    // rejects GNSS corrections the nominal run accepts — the FDIR-relevant
    // response a SIL check asserts on.
    assert!(
        faulted.innovation_rejected_ticks > baseline.innovation_rejected_ticks,
        "armed GNSS bias must cause more innovation-gate rejections: baseline {}, faulted {}",
        baseline.innovation_rejected_ticks,
        faulted.innovation_rejected_ticks
    );
}
