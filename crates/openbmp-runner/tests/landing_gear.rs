//! Integration coverage for scenario-wired rigid-body landing gear.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::PathBuf;

use openbmp_core::ChannelId;
use openbmp_telemetry::TelemetryValue;

const SYNTHETIC_GEAR_SHA256: &str =
    "efef5b573d83551fecd724383c9420de712bd62665e5ff989642916bbf2a2d5b";

fn scenario_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios/landing-gear-four-leg-drop/scenario.toml")
}

fn run_drop_fixture() -> openbmp_runner::RunOutcome {
    let scenario = openbmp_scenario::Scenario::from_file(scenario_path())
        .expect("landing gear scenario parses");
    openbmp_runner::run(&scenario).expect("landing gear scenario runs")
}

fn channel_id(outcome: &openbmp_runner::RunOutcome, name: &str) -> ChannelId {
    outcome
        .table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .unwrap_or_else(|| panic!("telemetry channel `{name}` must exist"))
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
            other => panic!("unexpected `{name}` telemetry value: {other:?}"),
        })
        .collect()
}

fn bool_column(outcome: &openbmp_runner::RunOutcome, name: &str) -> Vec<bool> {
    let id = channel_id(outcome, name);
    outcome
        .table
        .rows()
        .iter()
        .map(|row| match row.get(id) {
            Some(TelemetryValue::Bool(value)) => *value,
            other => panic!("unexpected `{name}` telemetry value: {other:?}"),
        })
        .collect()
}

#[test]
fn landing_gear_four_leg_drop_publishes_loads_and_leg_telemetry() {
    let outcome = run_drop_fixture();

    assert_eq!(outcome.final_step, 10_000);
    assert!((outcome.final_time_s - 10.0).abs() < 1.0e-12);
    assert_eq!(
        outcome
            .table
            .schema()
            .metadata()
            .get("openbmp.scenario_files.vehicle.landing_gear.data_file")
            .map(String::as_str),
        Some(SYNTHETIC_GEAR_SHA256)
    );
    let report = outcome
        .landing_gear
        .as_ref()
        .expect("landing gear run report");
    assert_eq!(
        report.outcome,
        openbmp_runner::contact::ContactOutcomeKind::Rest,
        "landing gear report: {report:#?}"
    );
    assert_eq!(report.final_legs.len(), 4);
    assert!(report.samples >= 10_000);
    assert!(report.contact_samples > 0);
    assert!(report.max_total_force_n > 1.0);
    assert!(report.max_leg_force_n > 0.0);
    assert!(report.max_stroke_m <= 0.80 + 1.0e-12);
    assert!(report.max_crushed_m <= 0.25 + 1.0e-12);
    assert!(report.final_summary.all_in_contact);
    assert!(report.final_summary.max_abs_normal_velocity_m_s <= 1.5e-2);
    assert!(report.energy.final_elastic_energy_j.is_finite());
    assert!(report.energy.contact_work_on_vehicle_j.is_finite());
    assert!(report.energy.dissipated_energy_j.is_finite());
    assert!(report.energy.relative_closure_error.is_finite());
    assert!(
        report.energy.relative_closure_error <= 1.0e-2,
        "landing gear audit should close to 1%: {:#?}",
        report.energy
    );

    let total_z = f64_column(&outcome, "force.landing_gear.z_n");
    let max_total_z = total_z.iter().copied().fold(0.0, f64::max);
    assert!(
        max_total_z > 1.0,
        "landing gear force channel should carry nonzero vertical load: {max_total_z}"
    );

    let leg_ids = ["front_left", "front_right", "rear_left", "rear_right"];
    let mut contact_columns = Vec::new();
    for leg_id in leg_ids {
        let force = f64_column(&outcome, &format!("landing_gear.{leg_id}.force_n"));
        let stroke = f64_column(&outcome, &format!("landing_gear.{leg_id}.stroke_m"));
        let crushed = f64_column(&outcome, &format!("landing_gear.{leg_id}.crushed_m"));
        let in_contact = bool_column(&outcome, &format!("landing_gear.{leg_id}.in_contact"));

        assert!(
            force.iter().any(|value| *value > 0.0),
            "{leg_id} should report positive strut load"
        );
        assert!(
            stroke
                .iter()
                .all(|value| value.is_finite() && *value >= 0.0 && *value <= 0.80 + 1.0e-12),
            "{leg_id} stroke must remain inside oleo+crush travel"
        );
        assert!(
            crushed
                .iter()
                .all(|value| value.is_finite() && *value >= 0.0 && *value <= 0.25 + 1.0e-12),
            "{leg_id} crush coordinate must stay inside crush travel"
        );
        assert!(
            in_contact.iter().any(|value| *value),
            "{leg_id} should contact the ground during the drop"
        );
        contact_columns.push(in_contact);
    }

    let has_four_leg_contact =
        (0..outcome.table.rows().len()).any(|row| contact_columns.iter().all(|column| column[row]));
    assert!(
        has_four_leg_contact,
        "all four legs should contact simultaneously"
    );
}
