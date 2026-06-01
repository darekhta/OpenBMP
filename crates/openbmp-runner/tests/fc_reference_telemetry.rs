//! Regression coverage for exported flight-controller reference telemetry.

#![allow(clippy::expect_used, clippy::panic)]

use openbmp_core::ChannelId;
use openbmp_telemetry::TelemetryValue;

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

#[test]
fn fc_scenarios_export_guidance_reference_quaternion() {
    let scenario = openbmp_scenario::Scenario::from_toml_str(include_str!(
        "fixtures/fc-reference-telemetry-attitude-hold.toml"
    ))
    .expect("scenario parses");

    let outcome = openbmp_runner::run(&scenario).expect("scenario runs");

    let valid = channel_id(&outcome, "guidance.reference.valid");
    let q_x = channel_id(&outcome, "guidance.reference.q_x");
    let q_y = channel_id(&outcome, "guidance.reference.q_y");
    let q_z = channel_id(&outcome, "guidance.reference.q_z");
    let q_w = channel_id(&outcome, "guidance.reference.q_w");

    let valid_quaternion_rows = outcome
        .table
        .rows()
        .iter()
        .filter(|row| matches!(row.get(valid), Some(TelemetryValue::Bool(true))))
        .map(|row| {
            [q_x, q_y, q_z, q_w].map(|channel| match row.get(channel) {
                Some(TelemetryValue::Float64(value)) => *value,
                other => panic!("unexpected reference quaternion value: {other:?}"),
            })
        })
        .collect::<Vec<_>>();

    assert!(
        !valid_quaternion_rows.is_empty(),
        "FC guidance should publish at least one valid reference row"
    );
    for q in valid_quaternion_rows {
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!(
            (norm - 1.0).abs() < 1.0e-12,
            "reference quaternion must stay unit length, got {q:?} with norm {norm}"
        );
    }

    assert!(
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .all(|channel| channel.name != "guidance.cutoff.time_to_go_s"),
        "attitude-hold guidance should not export ascent cutoff telemetry"
    );

    let ascent_reference = openbmp_scenario::Scenario::from_toml_str(include_str!(
        "fixtures/fc-reference-telemetry-ascent-reference.toml"
    ))
    .expect("ascent-reference scenario parses");
    let ascent_outcome =
        openbmp_runner::run(&ascent_reference).expect("ascent-reference scenario runs");
    let _cutoff_valid = channel_id(&ascent_outcome, "guidance.cutoff.valid");
    let _cutoff_time = channel_id(&ascent_outcome, "guidance.cutoff.time_to_go_s");
}
