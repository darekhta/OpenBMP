//! Session-stepped runs must reproduce one-shot runs byte-for-byte, and
//! runtime malfunction stimuli must be deterministic, effective, and
//! fail-closed on bad addressing.
//!
//! The equivalence contract is structural — [`openbmp_runner::run`] is
//! implemented over the same prepared session — and these tests pin it
//! against regressions that would split the two paths.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::path::Path;

use openbmp_core::EngineId;
use openbmp_runner::{EngineFault, RunOutcome, Session};
use openbmp_scenario::Scenario;
use openbmp_telemetry::TelemetryTable;

const TVC_PROBE_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scenarios/phalcon9/phalcon9-tvc-probe.toml"
);
const ORBIT_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scenarios/phalcon9/phalcon9-orbit.toml"
);

/// Load the TVC probe clipped to a short horizon so the closed-loop
/// EKF + nine-engine stack stays fast in debug CI.
fn load_tvc_probe() -> Scenario {
    let toml = std::fs::read_to_string(TVC_PROBE_PATH).expect("read scenario");
    let toml = toml.replace("stop_s  = 40.0", "stop_s  = 6.0");
    let source_dir = Path::new(TVC_PROBE_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

/// Load the orbital-insertion scenario clipped to the first seconds of
/// the pad climb — enough for the aero deck to see wind.
fn load_orbit_clipped() -> Scenario {
    let toml = std::fs::read_to_string(ORBIT_PATH).expect("read scenario");
    let toml = toml.replace("stop_s  = 1000.0", "stop_s  = 4.0");
    let source_dir = Path::new(ORBIT_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

fn sha(table: &TelemetryTable) -> String {
    openbmp_runner::determinism::telemetry_sha256_hex(table).expect("telemetry hash")
}

fn run_session_to_end(mut session: Session) -> RunOutcome {
    while !session.step().expect("session step") {}
    session.finish().expect("session finish")
}

#[test]
fn session_stepping_matches_one_shot_run_bytes() {
    let one_shot = openbmp_runner::run(&load_tvc_probe()).expect("one-shot run");
    let session = Session::prepare(load_tvc_probe()).expect("prepare session");
    let stepped = run_session_to_end(session);

    assert_eq!(
        one_shot.table, stepped.table,
        "session-stepped telemetry must be byte-identical to a one-shot run"
    );
    assert_eq!(sha(&one_shot.table), sha(&stepped.table));
    assert_eq!(one_shot.final_step, stepped.final_step);
    assert_eq!(
        one_shot.final_time_s.to_bits(),
        stepped.final_time_s.to_bits()
    );
    assert_eq!(
        format!("{:?}", one_shot.stop_reason),
        format!("{:?}", stepped.stop_reason)
    );
    assert_eq!(one_shot.actuator_stream, stepped.actuator_stream);
    assert_eq!(one_shot.afts, stepped.afts);
}

#[test]
fn session_reports_progress_and_phase() {
    let mut session = Session::prepare(load_tvc_probe()).expect("prepare session");
    assert!(!session.is_finished());
    assert_eq!(session.step_index(), 0);
    for _ in 0..50 {
        session.step().expect("session step");
    }
    assert_eq!(session.step_index(), 50);
    assert!((session.time_s() - 0.5).abs() < 1e-9);
    assert!(
        session
            .state()
            .position
            .vector
            .iter()
            .all(|c| c.is_finite())
    );
    // The TVC probe wires an FC, so the observation surface is present.
    let observation = session.fc_observation().expect("fc observation");
    assert!(
        observation.attitude.is_some(),
        "estimator published attitude"
    );
    assert_ne!(session.mission_phase(), "");
}

#[test]
fn engine_hard_off_is_terminal_effective_and_deterministic() {
    let run_with_fault = || {
        let mut session = Session::prepare(load_tvc_probe()).expect("prepare session");
        // Let the cluster ignite and settle, then kill one ring engine.
        while session.time_s() < 2.0 {
            session.step().expect("session step");
        }
        session
            .inject_engine_fault("eng_3", EngineFault::HardOff)
            .expect("inject hard-off");
        // Give the fault a tick to apply, then check the snapshot.
        session.step().expect("session step");
        session.step().expect("session step");
        let snapshots = session.engine_snapshots();
        let failed = snapshots
            .get(&EngineId::from_path("vehicle.assembly.engines.eng_3"))
            .expect("eng_3 snapshot");
        assert_eq!(failed.lifecycle_state_index, 4, "eng_3 must report Failed");
        assert!(
            failed.thrust_body.norm() == 0.0,
            "failed engine must produce zero thrust"
        );
        let healthy = snapshots
            .get(&EngineId::from_path("vehicle.assembly.engines.eng_0"))
            .expect("eng_0 snapshot");
        assert!(
            healthy.thrust_body.norm() > 0.0,
            "remaining engines keep burning"
        );
        run_session_to_end(session)
    };

    let baseline = openbmp_runner::run(&load_tvc_probe()).expect("baseline run");
    let faulted_a = run_with_fault();
    let faulted_b = run_with_fault();

    assert_ne!(
        sha(&baseline.table),
        sha(&faulted_a.table),
        "an engine-out must change the run"
    );
    assert_eq!(
        sha(&faulted_a.table),
        sha(&faulted_b.table),
        "the same stimulus timeline must reproduce byte-identically"
    );
}

#[test]
fn unknown_engine_name_fails_closed() {
    let mut session = Session::prepare(load_tvc_probe()).expect("prepare session");
    let err = session
        .inject_engine_fault("eng_99", EngineFault::HardOff)
        .expect_err("unknown engine must be rejected");
    assert!(err.to_string().contains("not declared"));
}

#[test]
fn gnss_outage_forces_dead_reckoning() {
    // The TVC probe's guidance is a time-based pitch program, so a GNSS
    // outage must show up in the ESTIMATOR (dead-reckoned translation
    // estimate diverges from the GNSS-aided one) even though the flown
    // trajectory only consumes the attitude estimate.
    let mut aided = Session::prepare(load_tvc_probe()).expect("prepare session");
    let mut outage = Session::prepare(load_tvc_probe()).expect("prepare session");
    outage
        .inject_sensor_outage("gnss", 1.0, 5.0)
        .expect("arm gnss outage");
    while aided.time_s() < 3.5 {
        aided.step().expect("session step");
        outage.step().expect("session step");
    }
    let aided_position = aided
        .fc_observation()
        .and_then(|o| o.position)
        .expect("aided position estimate");
    let outage_position = outage
        .fc_observation()
        .and_then(|o| o.position)
        .expect("outage position estimate");
    assert_ne!(
        format!("{aided_position:?}"),
        format!("{outage_position:?}"),
        "withholding GNSS must change the translation estimate"
    );
}

#[test]
fn gnss_outage_changes_closed_loop_run_and_is_deterministic() {
    // The orbital-insertion law steers on the NAVIGATED state, so a
    // GNSS outage must propagate all the way into the flown trajectory.
    let run_with_outage = || {
        let mut session = Session::prepare(load_orbit_clipped()).expect("prepare session");
        session
            .inject_sensor_outage("gnss", 1.0, 3.0)
            .expect("arm gnss outage");
        run_session_to_end(session)
    };

    let baseline = openbmp_runner::run(&load_orbit_clipped()).expect("baseline run");
    let outage_a = run_with_outage();
    let outage_b = run_with_outage();

    assert_ne!(
        sha(&baseline.table),
        sha(&outage_a.table),
        "a GNSS outage must change the navigation-steered run"
    );
    assert_eq!(
        sha(&outage_a.table),
        sha(&outage_b.table),
        "the same outage window must reproduce byte-identically"
    );
}

#[test]
fn unknown_sensor_outage_fails_closed() {
    let mut session = Session::prepare(load_tvc_probe()).expect("prepare session");
    let err = session
        .inject_sensor_outage("lidar", 1.0, 2.0)
        .expect_err("unknown sensor must be rejected");
    assert!(err.to_string().contains("not a bridge sensor"));
    let err = session
        .inject_sensor_outage("gnss", 3.0, 3.0)
        .expect_err("empty window must be rejected");
    assert!(err.to_string().contains("stop_s > start_s"));
}

#[test]
fn wind_override_changes_run_and_is_deterministic() {
    let run_with_wind = || {
        let mut session = Session::prepare(load_orbit_clipped()).expect("prepare session");
        for _ in 0..50 {
            session.step().expect("session step");
        }
        session.set_wind_override(Some([30.0, 0.0, 0.0]));
        for _ in 0..50 {
            session.step().expect("session step");
        }
        session.set_wind_override(None);
        run_session_to_end(session)
    };

    let baseline = openbmp_runner::run(&load_orbit_clipped()).expect("baseline run");
    let windy_a = run_with_wind();
    let windy_b = run_with_wind();

    assert_ne!(
        sha(&baseline.table),
        sha(&windy_a.table),
        "a wind override must perturb the aero-bearing run"
    );
    assert_eq!(
        sha(&windy_a.table),
        sha(&windy_b.table),
        "the same wind timeline must reproduce byte-identically"
    );
}

#[test]
fn point_mass_scenarios_are_rejected() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/sounding-rocket/niskanen-2009-chapter6.toml"
    );
    let toml = std::fs::read_to_string(path).expect("read scenario");
    let source_dir = Path::new(path).parent().map(Path::to_path_buf);
    let scenario =
        Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses");
    let err = Session::prepare(scenario).expect_err("point-mass must be rejected");
    assert!(err.to_string().contains("rigid_body"));
}
