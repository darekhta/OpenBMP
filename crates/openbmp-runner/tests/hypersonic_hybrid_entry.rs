//! End-to-end coverage for the hypersonic hybrid aero scenario example.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::PathBuf;

use openbmp_core::ChannelId;
use openbmp_sim::StopReason;
use openbmp_telemetry::TelemetryValue;

fn scenario_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios/hypersonic-hybrid-entry/scenario.toml")
}

fn channel_id(outcome: &openbmp_runner::RunOutcome, name: &str) -> ChannelId {
    outcome
        .table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .unwrap_or_else(|| panic!("channel `{name}` must exist"))
        .id
}

fn f64_column(outcome: &openbmp_runner::RunOutcome, name: &str) -> Vec<f64> {
    let id = channel_id(outcome, name);
    outcome
        .table
        .rows()
        .iter()
        .map(|row| match row.get(id) {
            Some(TelemetryValue::Float64(value)) => *value,
            other => panic!("unexpected value in {name}: {other:?}"),
        })
        .collect()
}

#[test]
fn hypersonic_hybrid_entry_scenario_runs_with_live_aero_force() {
    let scenario =
        openbmp_scenario::Scenario::from_file(scenario_path()).expect("scenario must parse");
    let outcome = openbmp_runner::run(&scenario).expect("scenario must run");

    let StopReason::MissionEnded { label, .. } = &outcome.stop_reason else {
        panic!(
            "expected mission ground stop, got {:?}",
            outcome.stop_reason
        );
    };
    assert_eq!(label, "ground-impact");
    assert!(outcome.final_time_s < scenario.document.time.stop_s);

    let aero_force_z = f64_column(&outcome, "force.aero.z_n");
    assert!(
        aero_force_z.iter().any(|value| value.abs() > 0.0),
        "hybrid aero scenario should produce live aero force: {aero_force_z:?}"
    );
}
