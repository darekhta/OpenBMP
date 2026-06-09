//! Increment F verification + the minimum-real-SIL hard gate.
//!
//! A single declared `SilCheck` must PASS on a nominal in-loop run and FAIL
//! the SAME package under an armed sensor fault — proving the bench injects
//! at the FC/sensor boundary, observes the flight software's response, and
//! renders an honest fail-capable verdict.

#![allow(clippy::expect_used, clippy::panic)]

use std::fs;
use std::path::Path;

use openbmp_sil::{MissionPackage, SilRunReport};

/// Absolute path to the shared sensor-budget data dir, so a scenario copied
/// into a tempdir still resolves its sensor `file =` references.
fn data_sensors_abs() -> String {
    fs::canonicalize(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/sensors"))
        .expect("canonicalize data/sensors")
        .to_string_lossy()
        .into_owned()
}

const CHECK: &str = "\
[[test_cases.checks]]
kind = \"no_innovation_rejection_storm\"
max_rejected_ticks = 10
";

const GNSS_BIAS: &str = "\
[[fc.sil_stimulus.faults]]
sensor = \"gnss\"
start_s = 0.2
stop_s = 1.0
fault = { kind = \"bias\", offset = [80.0, 0.0, 0.0] }
";

/// Write a tempdir package wrapping the closed-loop-attitude-hold `[fc]`
/// scenario (sensor paths made absolute), appending `extra_scenario` to the
/// scenario and declaring the `no_innovation_rejection_storm` check.
fn write_package(dir: &Path, extra_scenario: &str) -> MissionPackage {
    let base = fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
    ))
    .expect("read scenario");
    let scenario_body = base.replace("../../data/sensors/", &format!("{}/", data_sensors_abs()));
    fs::write(
        dir.join("scenario.toml"),
        format!("{scenario_body}\n{extra_scenario}"),
    )
    .expect("write scenario");
    let manifest = dir.join("mission-package.toml");
    fs::write(
        &manifest,
        format!(
            "[package]\nid = \"test.checks\"\nversion = \"0.0.1\"\n\n[files]\nscenario = \"scenario.toml\"\n\n[[test_cases]]\nid = \"case\"\n{CHECK}"
        ),
    )
    .expect("write manifest");
    MissionPackage::load(&manifest).expect("load package")
}

fn check_verdict(report: &SilRunReport, id: &str) -> String {
    report
        .evidence
        .requirement_verdicts
        .iter()
        .find(|verdict| verdict.id == id)
        .map(|verdict| verdict.verdict.clone())
        .unwrap_or_else(|| panic!("requirement `{id}` must be present"))
}

#[test]
fn declared_check_passes_nominal_and_fails_under_armed_fault() {
    const CHECK_ID: &str = "SIL-NO-INNOVATION-REJECTION-STORM";

    // Nominal: no injected fault, the EKF accepts its GNSS fixes, the check
    // passes, and the run verdict is pass.
    let nominal_dir = tempfile::tempdir().expect("tempdir");
    let nominal = write_package(nominal_dir.path(), "")
        .run_case_observed(Some("case"), 1)
        .expect("nominal observed run");
    assert_eq!(
        check_verdict(&nominal, CHECK_ID),
        "pass",
        "nominal run must pass the innovation-rejection-storm check"
    );
    assert_eq!(
        nominal.evidence.verdict, "pass",
        "nominal run verdict must be pass"
    );
    assert!(nominal.evidence.observations.is_some());

    // Faulted: an armed GNSS bias storms the EKF innovation gate, the SAME
    // declared check fails, and the run verdict flips to fail.
    let faulted_dir = tempfile::tempdir().expect("tempdir");
    let faulted = write_package(faulted_dir.path(), GNSS_BIAS)
        .run_case_observed(Some("case"), 1)
        .expect("faulted observed run");
    assert_eq!(
        check_verdict(&faulted, CHECK_ID),
        "fail",
        "armed GNSS bias must fail the innovation-rejection-storm check"
    );
    assert_eq!(
        faulted.evidence.verdict, "fail",
        "armed fault must flip the run verdict to fail"
    );
    assert!(
        faulted
            .evidence
            .failures
            .iter()
            .any(|failure| failure.summary.contains(CHECK_ID)),
        "the failure list must record the failed check"
    );
}
