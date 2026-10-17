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

fn run_prismatic_drop_fixture() -> openbmp_runner::RunOutcome {
    let toml = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/landing-gear-four-leg-drop/scenario.toml"
    ))
    .replace("stop_s  = 10.0", "stop_s  = 1.0")
    .replace("data_file = \"../../data/landing_gear/synthetic-four-leg-oleo-crush.toml\"\n", "")
    .replace(
        "data_file_sha256 = \"efef5b573d83551fecd724383c9420de712bd62665e5ff989642916bbf2a2d5b\"\n",
        "",
    )
    .replace(
        "crush = { f_crush_n = 40000.0, stroke_max_m = 0.25, k_elastic_n_m = 400000.0 }",
        "crush = { f_crush_n = 40000.0, stroke_max_m = 0.25, k_elastic_n_m = 400000.0 }\nprismatic_dof = { stroke_max_m = 0.80 }",
    );
    let scenario = openbmp_scenario::Scenario::from_toml_str(&toml)
        .expect("landing gear prismatic scenario parses");
    openbmp_runner::run(&scenario).expect("landing gear prismatic scenario runs")
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

fn run_crush_only_drop(dt_s: f64, method: Option<&str>) -> openbmp_runner::RunOutcome {
    let mut scenario = openbmp_scenario::Scenario::from_file(scenario_path()).expect("fixture");
    scenario.document.time.dt_s = dt_s;
    scenario.document.time.stop_s = 1.0;
    scenario.document.forces = Some(openbmp_scenario::ForcesConfig {
        models: vec!["landing_gear".to_owned()],
        phase_override: Vec::new(),
    });
    // A free drop into four identical elastic-perfectly-plastic struts,
    // without gravity or other work: m=1000 kg, v=-1.5 m/s, Fy=4000 N,
    // k=40000 N/m. At peak compression K0=1125 J = 200 J elastic
    // + 925 J irreversible work, giving p=0.23125 m per leg.
    scenario.document.vehicle.initial_position_eci_m = [0.0, 0.0, 1.0];
    let gear = scenario
        .document
        .vehicle
        .landing_gear
        .as_mut()
        .expect("gear");
    gear.data_file = None;
    gear.data_file_sha256 = None;
    for leg in &mut gear.legs {
        leg.footpad = openbmp_scenario::ContactGeometryConfig::Point;
        leg.footpad_radius_m = None;
        leg.oleo = None;
        leg.crush = Some(openbmp_scenario::LandingGearCrushConfig {
            f_crush_n: 1000.0,
            stroke_max_m: 0.5,
            k_elastic_n_m: 10_000.0,
        });
    }
    if let Some(method) = method {
        scenario.document.solver = Some(toml::from_str(&format!(
            "profile = 'adaptive-explicit'\ntrajectory_method = '{method}'\ndeterminism = 'state-stable'\n\
             [adaptive]\nrtol = 1e-8\natol = 1e-9\nmin_dt_s = 1e-8\nmax_dt_s = 0.001\ndense_output = false\n"
        )).expect("adaptive solver"));
    }
    openbmp_runner::run(&scenario).expect("crush drop")
}

#[test]
fn landing_gear_crush_drop_converges_to_analytic_work_and_rebound() {
    let mut errors = Vec::new();
    for dt_s in [0.002, 0.001] {
        let outcome = run_crush_only_drop(dt_s, None);
        let report = outcome.landing_gear.as_ref().expect("report");
        let plastic_work = 4000.0 * report.final_summary.max_crushed_m;
        errors.push((plastic_work - 925.0).abs());
        assert!(
            (plastic_work - 925.0).abs() < 2.0,
            "dt={dt_s}: {plastic_work}"
        );
        assert!((report.energy.dissipated_energy_j - plastic_work).abs() < 1e-9);
        let rebound = *f64_column(&outcome, "velocity_z_m_s")
            .last()
            .expect("velocity");
        assert!((0.5 * 1000.0 * rebound * rebound - 200.0).abs() < 2.0);
        assert!(
            report.energy.relative_closure_error < 0.005,
            "{:?}",
            report.energy
        );
    }
    assert!(
        errors[1] < errors[0],
        "refined plastic work errors: {errors:?}"
    );
}

#[test]
fn landing_gear_adaptive_drop_accounts_for_all_accepted_plastic_work() {
    for method in ["dopri54", "dopri853"] {
        // A telemetry interval 20 times the maximum adaptive substep
        // requires work to accumulate across several accepted states.
        let outcome = run_crush_only_drop(0.02, Some(method));
        let report = outcome.landing_gear.as_ref().expect("report");
        let plastic_work = 4000.0 * report.final_summary.max_crushed_m;
        assert!(
            (plastic_work - 925.0).abs() < 1.0,
            "{method}: {plastic_work}"
        );
        assert!((report.energy.dissipated_energy_j - plastic_work).abs() < 1e-9);
    }
}

#[test]
fn landing_gear_four_leg_drop_publishes_loads_and_leg_telemetry() {
    let outcome = run_drop_fixture();

    assert!(
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .all(|channel| !channel.name.contains(".prismatic_")),
        "base landing-gear fixture should not publish opt-in prismatic channels"
    );
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
    let recovered =
        openbmp_runner::landing_gear::recover_section_loads_body_x(&report.final_legs, &[0.0])
            .expect("recover mid-body landing gear section load");
    let mid_body_load = recovered[0];
    assert_eq!(mid_body_load.included_leg_count, 2);

    let total_z = f64_column(&outcome, "force.landing_gear.z_n");
    let max_total_z = total_z.iter().copied().fold(0.0, f64::max);
    assert!(
        max_total_z > 1.0,
        "landing gear force channel should carry nonzero vertical load: {max_total_z}"
    );
    let front_final_load_from_telemetry = ["front_left", "front_right"]
        .into_iter()
        .map(|leg_id| {
            f64_column(&outcome, &format!("landing_gear.{leg_id}.force_n"))
                .last()
                .copied()
                .expect("leg force has final row")
        })
        .sum::<f64>();
    assert!(
        (mid_body_load.shear_z_n - front_final_load_from_telemetry).abs()
            <= front_final_load_from_telemetry.abs().max(1.0) * 1.0e-12,
        "mid-body shear should recover from front leg final telemetry: \
         recovered={mid_body_load:?}, telemetry={front_final_load_from_telemetry}"
    );
    assert!(
        (mid_body_load.bending_moment_n_m() - front_final_load_from_telemetry).abs()
            <= front_final_load_from_telemetry.abs().max(1.0) * 1.0e-12,
        "mid-body bending should equal front-leg shear times 1 m lever: \
         recovered={mid_body_load:?}, telemetry={front_final_load_from_telemetry}"
    );
    assert!(mid_body_load.axial_n.abs() <= 1.0e-12);
    assert!(mid_body_load.shear_y_n.abs() <= 1.0e-12);
    assert!(mid_body_load.torsion_x_n_m.abs() <= 1.0e-12);
    assert!(mid_body_load.bending_z_n_m.abs() <= 1.0e-12);

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

#[test]
fn landing_gear_prismatic_dof_publishes_opt_in_telemetry() {
    let outcome = run_prismatic_drop_fixture();

    let stroke = f64_column(&outcome, "landing_gear.front_left.stroke_m");
    let compression_rate = f64_column(&outcome, "landing_gear.front_left.compression_rate_m_s");
    let displacement = f64_column(&outcome, "landing_gear.front_left.prismatic_displacement_m");
    let displacement_rate = f64_column(
        &outcome,
        "landing_gear.front_left.prismatic_displacement_rate_m_s",
    );
    let footpad_z = f64_column(
        &outcome,
        "landing_gear.front_left.prismatic_footpad_body_z_m",
    );

    assert!(stroke.iter().any(|value| *value > 0.0));
    for index in 0..stroke.len() {
        assert!(
            displacement[index].is_finite()
                && displacement_rate[index].is_finite()
                && footpad_z[index].is_finite()
        );
        assert!(
            (displacement[index] - (1.0 - stroke[index])).abs() <= 1.0e-12,
            "displacement should be free_length - stroke at row {index}"
        );
        assert!(
            (displacement_rate[index] + compression_rate[index]).abs() <= 1.0e-12,
            "displacement rate should oppose compression rate at row {index}"
        );
        assert!(
            (footpad_z[index] + displacement[index]).abs() <= 1.0e-12,
            "front-left z footpad coordinate should follow the body -z strut axis at row {index}"
        );
    }
}
