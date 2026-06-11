//! CLI-facing tests for Monte Carlo utility commands.

#![allow(clippy::expect_used)]

use std::fs;

use openbmp_cli::commands::mc;
use openbmp_uq::CredibilityLevel;
use tempfile::Builder;

const MC_ENGINE_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "mc-scheduled-engine-fault"
description = "Synthetic MC scheduled propulsion fault regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.6
dt_s = 0.1
seed = 29

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "mc-scheduled-engine-fault"

[[vehicle.assembly.bodies]]
id = "core"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 100.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]]

[[vehicle.assembly.engines]]
id = "main"
mounted_to = "core"
kind = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, 0.0]
limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity", "thrust"]

[mission]
initial_phase = "flight"

[[mission.phases]]
id = "flight"
label = "flight"

[scenario_script]

[[scenario_script.events]]
id = "ignite"
trigger = { kind = "at_time", time_s = 0.1 }
action = { kind = "engine_command", id = "main", command = { throttle_unit = 1.0, gimbal_pitch_rad = 0.0, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }
once = true

[telemetry]
output.csv = "out/mc-scheduled-engine-fault.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

#[test]
fn mc_summarize_reads_scalar_samples_and_success_interval() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let path = temp.path().join("samples.csv");
    fs::write(
        &path,
        "sample_index,perigee_km,success\n10,200.0,true\n12,220.0,true\n15,180.0,false\n",
    )
    .expect("write samples");

    let report = mc::summarize(&path, "perigee_km", Some("success"), 0.95).expect("summarize");
    assert_eq!(report.samples, 3);
    assert_eq!(report.mean.to_bits(), 200.0_f64.to_bits());
    assert!(report.standard_error.is_some());
    let success = report.success.expect("success summary");
    assert_eq!(success.trials, 3);
    assert_eq!(success.successes, 2);
    assert!(success.lower < success.fraction);
    assert!(success.upper > success.fraction);
}

#[test]
fn mc_summarize_rejects_missing_value_column() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let path = temp.path().join("samples.csv");
    fs::write(&path, "sample_index,value\n0,1.0\n").expect("write samples");

    let err = mc::summarize(&path, "missing", None, 0.95).expect_err("missing column");
    assert!(err.to_string().contains("missing column `missing`"));
}

#[test]
fn mc_propulsion_fault_campaign_runs_sampled_overlays() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let library = temp.path().join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    fs::write(&scenario, MC_ENGINE_SCENARIO).expect("write scenario");
    fs::write(
        &library,
        r#"
[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");

    let report = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 2,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: None,
    })
    .expect("run campaign");

    assert_eq!(report.samples, 2);
    assert_eq!(report.activated_samples, 2);
    assert!(report.summary.mean < 1.0);
    let success = report.summary.success.expect("success summary");
    assert_eq!(success.trials, 2);
    assert_eq!(success.successes, 2);

    let csv = fs::read_to_string(&output).expect("read campaign csv");
    assert!(csv.starts_with("sample_index,fault_ids,activated_fault_count,value,success\n"));
    assert!(csv.contains("0,mc-hardoff,1,"));
    assert!(csv.contains("1,mc-hardoff,1,"));
    assert!(csv.lines().skip(1).all(|line| line.ends_with(",true")));
}

#[test]
fn mc_propulsion_fault_campaign_attaches_uq_credibility_report() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let library = temp.path().join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::write(&scenario, MC_ENGINE_SCENARIO).expect("write scenario");
    fs::write(
        &library,
        r#"
[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");

    let report = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 1,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L2,
            report_md: Some(&report_md),
        }),
    })
    .expect("run campaign");

    let credibility = report.credibility.expect("credibility report");
    assert_eq!(credibility.binding_level, CredibilityLevel::L2);
    assert!(credibility.accepted);
    assert_eq!(
        report.credibility_report_md.as_deref(),
        Some(report_md.as_path())
    );
    assert!(output.exists());
    let markdown = fs::read_to_string(&report_md).expect("read markdown");
    assert!(markdown.contains("# Monte Carlo credibility report"));
    assert!(markdown.contains("Binding credibility:** L2"));
}

#[test]
fn mc_propulsion_fault_campaign_refuses_uq_below_credibility_floor() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let library = temp.path().join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::write(&scenario, MC_ENGINE_SCENARIO).expect("write scenario");
    fs::write(
        &library,
        r#"
[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");

    let err = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 1,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L3,
            report_md: Some(&report_md),
        }),
    })
    .expect_err("floor should fail");

    assert!(err.to_string().contains("below requested floor L3"));
    assert!(report_md.exists());
    assert!(!output.exists());
}

#[test]
fn mc_propulsion_fault_campaign_uses_library_declared_uq_report() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let library_dir = temp.path().join("faults");
    let library = library_dir.join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    let uq = temp.path().join("uq").join("budget.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::create_dir_all(&library_dir).expect("library dir");
    fs::create_dir_all(uq.parent().expect("uq parent")).expect("uq dir");
    fs::write(&scenario, MC_ENGINE_SCENARIO).expect("write scenario");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");
    fs::write(
        &library,
        r#"
[uq]
budget_toml = "../uq/budget.toml"
credibility_floor = "l2"
report_md = "../reports/credibility.md"

[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");

    let report = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 1,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: None,
    })
    .expect("run campaign");

    let credibility = report.credibility.expect("credibility report");
    assert_eq!(credibility.binding_level, CredibilityLevel::L2);
    assert!(credibility.accepted);
    let expected_report_md = library_dir.join("../reports/credibility.md");
    assert_eq!(
        report.credibility_report_md.as_deref(),
        Some(expected_report_md.as_path())
    );
    assert!(output.exists());
    let markdown = fs::read_to_string(&report_md).expect("read markdown");
    assert!(markdown.contains("Binding credibility:** L2"));
}

#[test]
fn mc_propulsion_fault_campaign_refuses_library_uq_below_floor() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let library_dir = temp.path().join("faults");
    let library = library_dir.join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    let uq = temp.path().join("uq").join("budget.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::create_dir_all(&library_dir).expect("library dir");
    fs::create_dir_all(uq.parent().expect("uq parent")).expect("uq dir");
    fs::write(&scenario, MC_ENGINE_SCENARIO).expect("write scenario");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");
    fs::write(
        &library,
        r#"
[uq]
budget_toml = "../uq/budget.toml"
credibility_floor = "l3"
report_md = "../reports/credibility.md"

[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");

    let err = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 1,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: None,
    })
    .expect_err("floor should fail");

    assert!(err.to_string().contains("below requested floor L3"));
    assert!(report_md.exists());
    assert!(!output.exists());
}

#[test]
fn mc_summarize_attaches_uq_credibility_report_and_floor() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("samples.csv");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::write(
        &samples,
        "sample_index,value,success\n0,3.0,true\n1,5.0,true\n",
    )
    .expect("write samples");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");

    let report = mc::summarize_with_credibility(
        &samples,
        "value",
        Some("success"),
        0.95,
        Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L2,
            report_md: Some(&report_md),
        }),
    )
    .expect("summarize with credibility");

    assert_eq!(report.summary.samples, 2);
    let credibility = report.credibility.expect("credibility report");
    assert!(credibility.accepted);
    assert_eq!(credibility.binding_level, CredibilityLevel::L2);
    assert!((credibility.aggregate_one_sigma - 5.0).abs() < 1.0e-12);
    assert!((credibility.aleatory_one_sigma - 3.0).abs() < 1.0e-12);
    assert!((credibility.epistemic_one_sigma - 4.0).abs() < 1.0e-12);
    let markdown = fs::read_to_string(&report_md).expect("read report");
    assert!(markdown.contains("Monte Carlo credibility report"));
    assert!(markdown.contains("accepted = true"));
}

#[test]
fn mc_summarize_refuses_uq_below_credibility_floor() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("samples.csv");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("credibility.md");
    fs::write(&samples, "sample_index,value\n0,3.0\n1,5.0\n").expect("write samples");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");

    let err = mc::summarize_with_credibility(
        &samples,
        "value",
        None,
        0.95,
        Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L3,
            report_md: Some(&report_md),
        }),
    )
    .expect_err("credibility floor must fail closed");

    assert!(
        err.to_string()
            .contains("credibility binding level L2 is below requested floor L3")
    );
    let markdown = fs::read_to_string(&report_md).expect("read report");
    assert!(markdown.contains("accepted = false"));
}

#[test]
fn mc_nested_summarize_reports_pbox_variance_split_and_writes_csv() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("nested.csv");
    let pbox = temp.path().join("out").join("pbox.csv");
    fs::write(
        &samples,
        "epistemic_index,value\n0,-3.0\n0,1.0\n1,-1.0\n1,3.0\n",
    )
    .expect("write nested samples");

    let report = mc::summarize_nested(&samples, "epistemic_index", "value", 1.0, 0.5, Some(&pbox))
        .expect("nested summarize");

    assert_eq!(report.analysis.epistemic_samples, 2);
    assert_eq!(report.analysis.min_aleatory_samples, 2);
    assert_eq!(report.analysis.max_aleatory_samples, 2);
    assert!((report.analysis.variance_split.aleatory - 4.0).abs() < 1.0e-12);
    assert!((report.analysis.variance_split.epistemic - 1.0).abs() < 1.0e-12);
    assert_eq!(report.analysis.lower_bound_probability, Some(0.5));
    assert_eq!(report.analysis.requirement_passed, Some(true));
    assert_eq!(report.pbox_csv.as_deref(), Some(pbox.as_path()));
    let pbox_text = fs::read_to_string(&pbox).expect("read pbox");
    assert!(pbox_text.starts_with("support,lower_cdf,upper_cdf\n"));
    assert!(
        pbox_text.contains("1.00000000000000000e0,5.00000000000000000e-1,1.00000000000000000e0")
    );
}

#[test]
fn mc_nested_summarize_requirement_uses_lower_pbox_bound() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("nested.csv");
    fs::write(
        &samples,
        "epistemic_index,value\n0,0.0\n0,0.0\n1,10.0\n1,10.0\n",
    )
    .expect("write nested samples");

    let report = mc::summarize_nested(&samples, "epistemic_index", "value", 0.0, 0.5, None)
        .expect("nested summarize");

    assert_eq!(report.analysis.lower_bound_probability, Some(0.0));
    assert_eq!(report.analysis.requirement_passed, Some(false));
}

#[test]
fn mc_resume_scalar_checkpoint_records_chunks_and_final_summary() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("samples.csv");
    let checkpoint = temp.path().join("checkpoint.json");
    fs::write(
        &samples,
        "sample_index,value,success\n0,1.0,true\n1,2.0,true\n2,3.0,false\n3,4.0,true\n",
    )
    .expect("write samples");

    let partial = mc::resume_scalar_checkpoint(
        &samples,
        &checkpoint,
        "value",
        Some("success"),
        4,
        0xC0FFEE,
        0,
        "scenario-a",
        "x86_64-clean",
        2,
        2,
        0.95,
    )
    .expect("partial resume");
    assert_eq!(partial.completed, 2);
    assert!(!partial.is_complete());

    let complete = mc::resume_scalar_checkpoint(
        &samples,
        &checkpoint,
        "value",
        Some("success"),
        4,
        0xC0FFEE,
        0,
        "scenario-a",
        "x86_64-clean",
        4,
        4,
        0.95,
    )
    .expect("complete resume");
    assert_eq!(complete.completed, 4);
    let summary = complete.summary.expect("final summary");
    assert_eq!(summary.samples, 4);
    assert_eq!(summary.mean.to_bits(), 2.5_f64.to_bits());
    let success = summary.success.expect("success summary");
    assert_eq!(success.trials, 4);
    assert_eq!(success.successes, 3);
}

#[test]
fn mc_resume_scalar_checkpoint_rejects_scenario_mismatch() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("samples.csv");
    let checkpoint = temp.path().join("checkpoint.json");
    fs::write(&samples, "sample_index,value\n0,1.0\n1,2.0\n").expect("write samples");

    mc::resume_scalar_checkpoint(
        &samples,
        &checkpoint,
        "value",
        None,
        2,
        0xC0FFEE,
        0,
        "scenario-a",
        "x86_64-clean",
        1,
        1,
        0.95,
    )
    .expect("create checkpoint");
    let err = mc::resume_scalar_checkpoint(
        &samples,
        &checkpoint,
        "value",
        None,
        2,
        0xC0FFEE,
        0,
        "scenario-b",
        "x86_64-clean",
        1,
        1,
        0.95,
    )
    .expect_err("scenario mismatch");
    assert!(
        err.to_string()
            .contains("checkpoint metadata mismatch in scenario_sha256")
    );
}

#[test]
fn mc_wilks_reports_two_sided_sample_size() {
    let report =
        mc::wilks_sample_size(mc::WilksSide::TwoSided, 0.99865, 0.90).expect("wilks sample size");
    assert_eq!(report.samples, 2880);
    assert!(report.attained_confidence >= report.confidence);
    assert_eq!(report.side.label(), "two-sided");
}

#[test]
fn mc_wilks_rejects_invalid_probability() {
    let err =
        mc::wilks_sample_size(mc::WilksSide::OneSided, 1.0, 0.90).expect_err("invalid coverage");
    assert!(
        err.to_string()
            .contains("coverage is outside the Monte Carlo statistics envelope")
    );
}

#[test]
fn mc_lhs_writes_unit_cube_design_csv() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let path = temp.path().join("lhs.csv");
    let report = mc::write_latin_hypercube(5, 2, 0x5EED, &path).expect("write lhs");
    assert_eq!(report.samples, 5);
    assert_eq!(report.dimensions, 2);

    let mut reader = csv::Reader::from_path(&path).expect("read lhs");
    let headers = reader.headers().expect("headers").clone();
    assert_eq!(&headers[0], "sample_index");
    assert_eq!(&headers[1], "u0");
    assert_eq!(&headers[2], "u1");

    let mut strata = [vec![false; 5], vec![false; 5]];
    for (row_index, row) in reader.records().enumerate() {
        let row = row.expect("row");
        assert_eq!(
            row.get(0).expect("sample_index"),
            row_index.to_string().as_str()
        );
        for (dimension, stratum_seen) in strata.iter_mut().enumerate() {
            let value = row
                .get(dimension + 1)
                .expect("value")
                .parse::<f64>()
                .expect("float");
            assert!((0.0..1.0).contains(&value));
            let stratum = (value * 5.0).floor() as usize;
            assert!(!stratum_seen[stratum]);
            stratum_seen[stratum] = true;
        }
    }
    assert!(strata.into_iter().flatten().all(|seen| seen));
}

#[test]
fn mc_sobol_writes_reference_design_csv() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let path = temp.path().join("sobol.csv");
    let report = mc::write_sobol(4, 2, None, &path).expect("write sobol");
    assert_eq!(report.samples, 4);
    assert_eq!(report.dimensions, 2);
    assert_eq!(report.scramble_seed, None);

    let (_, columns) = read_design_columns(&path);
    assert_eq!(
        columns[0]
            .iter()
            .map(|value| value.to_bits())
            .collect::<Vec<_>>(),
        [0.0_f64, 0.5, 0.75, 0.25]
            .into_iter()
            .map(f64::to_bits)
            .collect::<Vec<_>>()
    );
    assert_eq!(
        columns[1]
            .iter()
            .map(|value| value.to_bits())
            .collect::<Vec<_>>(),
        [0.0_f64, 0.5, 0.25, 0.75]
            .into_iter()
            .map(f64::to_bits)
            .collect::<Vec<_>>()
    );
}

#[test]
fn mc_sobol_writes_scrambled_design_csv() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let base_path = temp.path().join("sobol.csv");
    let scrambled_path = temp.path().join("sobol-scrambled.csv");
    mc::write_sobol(8, 2, None, &base_path).expect("write sobol");
    let report = mc::write_sobol(8, 2, Some(0x0E11), &scrambled_path).expect("write scrambled");
    assert_eq!(report.scramble_seed, Some(0x0E11));
    let (_, base_columns) = read_design_columns(&base_path);
    let (_, scrambled_columns) = read_design_columns(&scrambled_path);
    assert_ne!(base_columns, scrambled_columns);
    assert!(
        scrambled_columns
            .iter()
            .flatten()
            .all(|value| (0.0..1.0).contains(value))
    );
}

#[test]
fn mc_iman_conover_writes_correlated_design_csv() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let design_path = temp.path().join("lhs.csv");
    let correlation_path = temp.path().join("correlation.csv");
    let output_path = temp.path().join("correlated.csv");
    mc::write_latin_hypercube(128, 2, 0x1C0E, &design_path).expect("write lhs");
    fs::write(&correlation_path, "1.0,0.7\n0.7,1.0\n").expect("write correlation");

    let report =
        mc::write_iman_conover(&design_path, &correlation_path, &output_path).expect("correlate");
    assert_eq!(report.samples, 128);
    assert_eq!(report.dimensions, 2);

    let (input_headers, input_columns) = read_design_columns(&design_path);
    let (output_headers, output_columns) = read_design_columns(&output_path);
    assert_eq!(input_headers, output_headers);
    assert_eq!(
        sorted_bits(&input_columns[0]),
        sorted_bits(&output_columns[0])
    );
    assert_eq!(
        sorted_bits(&input_columns[1]),
        sorted_bits(&output_columns[1])
    );
    let correlation = pearson(&output_columns[0], &output_columns[1]);
    assert!(
        correlation > 0.6,
        "expected positive induced correlation, got {correlation}"
    );
}

fn uq_budget_toml(level: u8) -> String {
    let factors = [
        "verification",
        "validation",
        "input_pedigree",
        "results_uncertainty",
        "results_robustness",
        "use_history",
        "ms_management",
        "people_qualification",
    ];
    let mut text = String::new();
    for (id, one_sigma, class) in [("wind", 3.0, "aleatory"), ("aero", 4.0, "epistemic")] {
        text.push_str("[[sources]]\n");
        text.push_str(&format!("id = \"{id}\"\n"));
        text.push_str(&format!("one_sigma = {one_sigma}\n"));
        text.push_str(&format!("class = \"{class}\"\n"));
        text.push_str("justification = \"synthetic test budget\"\n");
        for factor in factors {
            text.push_str(&format!(
                "\n[sources.credibility.{factor}]\nlevel = {level}\nevidence = \"V-UQ-TEST\"\n"
            ));
        }
        text.push('\n');
    }
    text
}

fn read_design_columns(path: &std::path::Path) -> (Vec<String>, Vec<Vec<f64>>) {
    let mut reader = csv::Reader::from_path(path).expect("read design");
    let headers = reader
        .headers()
        .expect("headers")
        .iter()
        .map(str::to_owned)
        .collect::<Vec<_>>();
    let mut columns = vec![Vec::new(); headers.len() - 1];
    for row in reader.records() {
        let row = row.expect("row");
        for (dimension, column) in columns.iter_mut().enumerate() {
            column.push(
                row.get(dimension + 1)
                    .expect("value")
                    .parse::<f64>()
                    .expect("float"),
            );
        }
    }
    (headers, columns)
}

fn sorted_bits(values: &[f64]) -> Vec<u64> {
    let mut values = values.to_vec();
    values.sort_by(f64::total_cmp);
    values.into_iter().map(f64::to_bits).collect()
}

fn pearson(left: &[f64], right: &[f64]) -> f64 {
    let left_mean = left.iter().sum::<f64>() / left.len() as f64;
    let right_mean = right.iter().sum::<f64>() / right.len() as f64;
    let mut covariance = 0.0;
    let mut left_m2 = 0.0;
    let mut right_m2 = 0.0;
    for (left, right) in left.iter().zip(right.iter()) {
        let dl = left - left_mean;
        let dr = right - right_mean;
        covariance += dl * dr;
        left_m2 += dl * dl;
        right_m2 += dr * dr;
    }
    covariance / (left_m2 * right_m2).sqrt()
}
