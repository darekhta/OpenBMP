//! Increment C verification: an in-loop `[fc]` run produces populated
//! estimate-vs-truth observations, and installing the monitor does not
//! change the telemetry.
//!
//! Uses the short closed-loop-attitude-hold `[fc]` scenario (point-mass,
//! GNSS + IMU + magnetometer, EKF). The manifest points `files.scenario` at
//! the real scenario by absolute path so the scenario's own relative sensor
//! budget paths still resolve.

#![allow(clippy::expect_used)]

use std::fs;

use openbmp_bridge::{
    CaptureTrigger, EesPort, ElectricalErrorType, FaultWindow, MaPort, PinId, SignalId,
    TestbenchTransition, XilValue,
};
use openbmp_sil::{InMemoryXilBench, MissionPackage};

const FC_SCENARIO: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
);

fn load_package(dir: &std::path::Path) -> MissionPackage {
    let manifest = dir.join("mission-package.toml");
    fs::write(
        &manifest,
        format!(
            "[package]\nid = \"test.observed\"\nversion = \"0.0.1\"\n\n[files]\nscenario = \"{FC_SCENARIO}\"\n"
        ),
    )
    .expect("write manifest");
    MissionPackage::load(&manifest).expect("load package")
}

#[test]
fn observed_run_populates_estimate_vs_truth_residuals() {
    let dir = tempfile::tempdir().expect("tempdir");
    let package = load_package(dir.path());

    let report = package
        .run_case_observed(None, 1)
        .expect("observed run succeeds");

    let observations = report
        .evidence
        .observations
        .as_ref()
        .expect("observed run must attach an observation report");

    // The full-fidelity summary covers every tick; samples (decimation 1)
    // record every tick too.
    assert!(
        observations.summary.ticks > 0,
        "must observe at least one tick"
    );
    assert_eq!(
        observations.samples.len() as u64,
        observations.summary.ticks,
        "decimation 1 records every tick"
    );

    // GNSS + IMU are wired, so the FC publishes a translational nav estimate
    // and an attitude estimate: the residual norms must be populated and
    // finite (the taps from increment A reach the monitor).
    let max_pos_err = observations
        .summary
        .max_pos_err_m
        .expect("FC translational estimate must be observed");
    assert!(
        max_pos_err.is_finite() && max_pos_err >= 0.0,
        "position residual must be a finite non-negative norm, got {max_pos_err}"
    );
    assert!(
        observations
            .samples
            .iter()
            .any(|sample| sample.att_err_rad.is_some()),
        "FC attitude estimate must be observed on at least one tick"
    );
    // Residual-only, forward-only: a well-tracking EKF keeps the estimate
    // close to truth, so the residual stays bounded (sanity, not a spec).
    assert!(
        observations.summary.max_att_err_rad.unwrap_or(0.0) < std::f64::consts::PI,
        "attitude error must be a valid geodesic angle"
    );
}

#[test]
fn installed_monitor_does_not_change_telemetry() {
    let dir = tempfile::tempdir().expect("tempdir");
    let package = load_package(dir.path());

    let plain = package.run_case(None).expect("plain run");
    let observed = package.run_case_observed(None, 1).expect("observed run");

    assert_eq!(
        plain.telemetry, observed.telemetry,
        "installing the SIL monitor must not change the telemetry table"
    );
}

#[test]
fn observed_run_can_attach_xil_evidence() {
    let dir = tempfile::tempdir().expect("tempdir");
    let package = load_package(dir.path());
    let mut xil = InMemoryXilBench::new();
    xil.declare_signal("fc.gnss.x", XilValue::Float64(0.0));
    xil.declare_pin("gnss.vcc");
    xil.transition(TestbenchTransition::Initialize)
        .expect("initialize XIL bench");
    xil.transition(TestbenchTransition::Connect)
        .expect("connect XIL bench");
    xil.transition(TestbenchTransition::Start)
        .expect("start XIL bench");
    xil.write(SignalId::from("fc.gnss.x"), XilValue::Float64(5.0))
        .expect("write XIL signal");
    xil.create_capture(&[SignalId::from("fc.gnss.x")], CaptureTrigger::Immediate, 1)
        .expect("create XIL capture");
    xil.set_error(
        PinId::from("gnss.vcc"),
        ElectricalErrorType::Open,
        FaultWindow::new(10, Some(12)).expect("valid XIL fault window"),
    )
    .expect("set XIL EES error");

    let report = package
        .run_case_observed_with_xil(None, 1, &xil)
        .expect("observed run with XIL evidence succeeds");

    assert!(
        report.evidence.observations.is_some(),
        "observed run must keep estimate-vs-truth evidence"
    );
    assert!(
        report
            .evidence
            .stimuli
            .iter()
            .any(|record| record.kind == "xil.ma.write" && record.target == "fc.gnss.x")
    );
    assert!(
        report
            .evidence
            .bus_frames
            .iter()
            .any(|frame| frame.stream == "xil.lifecycle" && frame.value == "Running")
    );
    assert!(
        report
            .evidence
            .bus_frames
            .iter()
            .any(|frame| frame.stream == "xil.ees.error" && frame.subject == "gnss.vcc")
    );
}
