//! End-to-end scenario coverage for the full boost-to-ground mission chain.

use std::path::PathBuf;

use openbmp_core::ChannelId;
use openbmp_sim::{PhaseId, StopReason};
use openbmp_telemetry::TelemetryValue;

fn scenario_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios/full-trajectory/boost-coast-entry-ground.toml")
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

fn marker_fire_index(outcome: &openbmp_runner::RunOutcome, tag: &str) -> usize {
    let channel = channel_id(outcome, &format!("mission.marker.{tag}"));
    let fires: Vec<usize> = outcome
        .table
        .rows()
        .iter()
        .enumerate()
        .filter_map(|(index, row)| match row.get(channel) {
            Some(TelemetryValue::Bool(true)) => Some(index),
            Some(TelemetryValue::Bool(false)) => None,
            other => panic!("unexpected marker value for {tag}: {other:?}"),
        })
        .collect();
    assert_eq!(fires.len(), 1, "marker `{tag}` should fire exactly once");
    fires[0]
}

#[test]
fn boost_coast_entry_ground_scenario_runs_the_full_event_chain() {
    let scenario =
        openbmp_scenario::Scenario::from_file(scenario_path()).expect("scenario must parse");
    let outcome = openbmp_runner::run(&scenario).expect("scenario must run");

    let StopReason::MissionEnded { phase, label } = &outcome.stop_reason else {
        panic!(
            "expected mission ground stop, got {:?}",
            outcome.stop_reason
        );
    };
    assert_eq!(label, "ground-impact");
    assert_eq!(*phase, Some(PhaseId::from_path("mission.phases.ground")));

    let burnout = marker_fire_index(&outcome, "burnout");
    let stage_separation = marker_fire_index(&outcome, "stage_separation");
    let apogee = marker_fire_index(&outcome, "apogee");
    let entry_interface = marker_fire_index(&outcome, "entry_interface");
    let entry = marker_fire_index(&outcome, "entry");
    let ground_impact = marker_fire_index(&outcome, "ground_impact");

    assert!(burnout < stage_separation);
    assert!(stage_separation < apogee);
    assert!(apogee < entry_interface);
    assert!(entry_interface < entry);
    assert!(entry < ground_impact);

    let separated = channel_id(&outcome, "body.booster.separated");
    assert!(
        outcome.table.rows()[stage_separation..]
            .iter()
            .any(|row| matches!(row.get(separated), Some(TelemetryValue::Bool(true)))),
        "booster stage should be separated after the staging event"
    );

    assert!(
        outcome.final_time_s < scenario.document.time.stop_s,
        "mission stop should end at ground impact before the time horizon"
    );
}
