//! CLI regression tests for offline trajectory optimization commands.

use std::error::Error;
use std::fs;
use std::path::PathBuf;

use assert_cmd::Command;
use openbmp_cli::commands::trajopt::{CorrectApogeeScenarioArgs, run_correct_apogee_scenario};
use predicates::prelude::*;
use tempfile::Builder;

#[test]
fn trajopt_correct_apogee_cli_writes_iload() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("apogee.iload");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "correct-apogee",
            "--initial-radius-m",
            "6778000",
            "--target-apogee-radius-m",
            "7578000",
            "--coast-duration-s",
            "120",
            "--step-s",
            "20",
            "--source-revision",
            "trajopt-cli-test",
            "--output-iload",
        ])
        .arg(&iload_path)
        .assert()
        .success()
        .stdout(predicate::str::contains(
            "openbmp trajopt correct-apogee: ok",
        ))
        .stdout(predicate::str::contains("wrote I-load"));

    let metadata = fs::metadata(iload_path)?;
    assert!(metadata.len() > 0);
    Ok(())
}

#[test]
fn trajopt_correct_apogee_cli_reports_non_convergence() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("nope.iload");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "correct-apogee",
            "--initial-radius-m",
            "6778000",
            "--target-apogee-radius-m",
            "7578000",
            "--coast-duration-s",
            "120",
            "--step-s",
            "20",
            "--max-iterations",
            "0",
            "--output-iload",
        ])
        .arg(&iload_path)
        .assert()
        .failure()
        .stderr(predicate::str::contains("trajectory optimization"))
        .stderr(predicate::str::contains("did not converge"));

    assert!(!iload_path.exists());
    Ok(())
}

#[test]
fn trajopt_correct_apogee_scenario_fixture_writes_iload() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("scenario-apogee.iload");
    let scenario = workspace_root()?.join("scenarios/trajopt-two-body-apogee/scenario.toml");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "correct-apogee-scenario",
            scenario.to_str().ok_or("scenario path is not UTF-8")?,
            "--target-apogee-radius-m",
            "7578000",
            "--source-revision",
            "trajopt-cli-test",
            "--output-iload",
        ])
        .arg(&iload_path)
        .assert()
        .success()
        .stdout(predicate::str::contains(
            "openbmp trajopt correct-apogee-scenario: ok",
        ))
        .stdout(predicate::str::contains("wrote I-load"));

    let metadata = fs::metadata(iload_path)?;
    assert!(metadata.len() > 0);
    Ok(())
}

#[test]
fn trajopt_correct_apogee_scenario_fixture_meets_tolerance() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("scenario-apogee.iload");
    let scenario = workspace_root()?.join("scenarios/trajopt-two-body-apogee/scenario.toml");
    let tolerance = read_trajopt_tolerance()?;

    let report = run_correct_apogee_scenario(
        CorrectApogeeScenarioArgs {
            scenario,
            target_apogee_radius_m: tolerance.target_apogee_radius_m,
            initial_speed_m_s: None,
            mu_m3_s2: None,
            residual_tolerance_m: tolerance.residual_tolerance_m,
            max_iterations: tolerance.max_iterations,
            synthesis_seed: 7,
            scenario_digest: None,
            source_revision: "trajopt-cli-test".to_owned(),
            producer: "openbmp-trajopt-cli-test".to_owned(),
        },
        &iload_path,
    )?;

    assert!(report.residual_norm_m < tolerance.residual_tolerance_m);
    assert!(report.reference_samples > 1);
    assert!(report.corrected_speed_m_s > 7_668.635_675);
    assert!(fs::metadata(iload_path)?.len() > 0);
    Ok(())
}

#[test]
fn trajopt_correct_apogee_scenario_reports_non_convergence() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("scenario-nope.iload");
    let scenario = workspace_root()?.join("scenarios/trajopt-two-body-apogee/scenario.toml");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "correct-apogee-scenario",
            scenario.to_str().ok_or("scenario path is not UTF-8")?,
            "--target-apogee-radius-m",
            "7578000",
            "--max-iterations",
            "0",
            "--output-iload",
        ])
        .arg(&iload_path)
        .assert()
        .failure()
        .stderr(predicate::str::contains("trajectory optimization"))
        .stderr(predicate::str::contains("did not converge"));

    assert!(!iload_path.exists());
    Ok(())
}

#[test]
fn trajopt_dol_redesign_cli_writes_iload_and_report() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("dol.iload");
    let report_path = temp.path().join("dol-report.json");
    let scenario =
        workspace_root()?.join("crates/openbmp-cli/tests/fixtures/day-of-launch-spliced-wind.toml");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-redesign",
            scenario.to_str().ok_or("scenario path is not UTF-8")?,
            "--target-apogee-radius-m",
            "7578000",
            "--load-window-min-altitude-m",
            "0",
            "--load-window-max-altitude-m",
            "3000",
            "--airspeed-m-s",
            "250",
            "--stale-pitch-bias-rad",
            "0.03",
            "--stale-yaw-bias-rad",
            "-0.03",
            "--load-limit-angle-rad",
            "0.05",
            "--insertion-residual-limit-m",
            "1.0",
            "--source-revision",
            "trajopt-cli-test",
            "--output-iload",
        ])
        .arg(&iload_path)
        .arg("--report-json")
        .arg(&report_path)
        .assert()
        .success()
        .stdout(predicate::str::contains("openbmp trajopt dol-redesign: ok"))
        .stdout(predicate::str::contains("wrote DOL report"));

    assert!(fs::metadata(iload_path)?.len() > 0);
    let report: serde_json::Value = serde_json::from_str(&fs::read_to_string(report_path)?)?;
    let stale = report
        .get("stale_min_margin_rad")
        .and_then(serde_json::Value::as_f64)
        .ok_or("missing stale_min_margin_rad")?;
    let redesigned = report
        .get("redesigned_min_margin_rad")
        .and_then(serde_json::Value::as_f64)
        .ok_or("missing redesigned_min_margin_rad")?;
    assert!(redesigned >= stale);
    Ok(())
}

#[test]
fn trajopt_dol_report_card_cli_writes_csv_and_json() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("dol.iload");
    let update_report_path = temp.path().join("dol-update.json");
    let telemetry_path = temp.path().join("dol-verification-telemetry.csv");
    let indicator_spec_path = temp.path().join("dol-indicator-spec.csv");
    let indicators_path = temp.path().join("dol-indicators.csv");
    let report_csv_path = temp.path().join("dol-report-card.csv");
    let report_json_path = temp.path().join("dol-report-card.json");
    let scenario =
        workspace_root()?.join("crates/openbmp-cli/tests/fixtures/day-of-launch-spliced-wind.toml");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-redesign",
            scenario.to_str().ok_or("scenario path is not UTF-8")?,
            "--target-apogee-radius-m",
            "7578000",
            "--load-window-min-altitude-m",
            "0",
            "--load-window-max-altitude-m",
            "3000",
            "--airspeed-m-s",
            "250",
            "--stale-pitch-bias-rad",
            "0.03",
            "--stale-yaw-bias-rad",
            "-0.03",
            "--load-limit-angle-rad",
            "0.05",
            "--insertion-residual-limit-m",
            "1.0",
            "--source-revision",
            "trajopt-cli-test",
            "--output-iload",
        ])
        .arg(&iload_path)
        .arg("--report-json")
        .arg(&update_report_path)
        .assert()
        .success();

    fs::write(
        &telemetry_path,
        "time_s,step,dol.q_alpha_station,dol.station_moment\n\
         0.0,0,11.8,4999.0\n\
         1.0,1,12.02,5000.25\n\
         2.0,2,12.01,4998.0\n",
    )?;
    fs::write(
        &indicator_spec_path,
        "name,design_value,telemetry_column,reducer,allowed_abs_error,time_min_s,time_max_s\n\
         q_alpha_station,12.0,dol.q_alpha_station,last,0.1,0.5,1.5\n\
         station_moment,5000.0,dol.station_moment,max_abs,1.0,0.0,2.0\n",
    )?;

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-materialize-indicators",
            telemetry_path
                .to_str()
                .ok_or("telemetry path is not UTF-8")?,
            indicator_spec_path
                .to_str()
                .ok_or("indicator spec path is not UTF-8")?,
            "--output-csv",
        ])
        .arg(&indicators_path)
        .assert()
        .success()
        .stdout(predicate::str::contains(
            "openbmp trajopt dol-materialize-indicators: ok",
        ))
        .stdout(predicate::str::contains("wrote DOL indicator CSV"));

    let indicators_csv = fs::read_to_string(&indicators_path)?;
    assert!(indicators_csv.contains("q_alpha_station"));
    assert!(indicators_csv.contains("station_moment"));

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-report-card",
            update_report_path
                .to_str()
                .ok_or("update report path is not UTF-8")?,
            indicators_path
                .to_str()
                .ok_or("indicators path is not UTF-8")?,
            "--report-id",
            "trajopt-cli-dol-report-card",
            "--verification-provenance",
            "runner_6dof_synthetic_cli",
            "--min-required-margin-rad",
            "-0.01",
            "--min-not-worse-fraction",
            "0.9",
            "--output-csv",
        ])
        .arg(&report_csv_path)
        .arg("--report-json")
        .arg(&report_json_path)
        .assert()
        .success()
        .stdout(predicate::str::contains(
            "openbmp trajopt dol-report-card: go",
        ))
        .stdout(predicate::str::contains("wrote DOL report card CSV"));

    let csv = fs::read_to_string(report_csv_path)?;
    assert!(csv.starts_with("openbmp.dol_report_card,1\n"));
    assert!(csv.contains("verdict,go,"));
    let report_json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(report_json_path)?)?;
    assert_eq!(report_json["verdict"].as_str(), Some("Go"));
    Ok(())
}

#[test]
fn trajopt_dol_commit_readiness_cli_writes_csv_and_json() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let (report_card_path, persistence_path, cola_path) =
        write_dol_commit_readiness_inputs(temp.path(), false)?;
    let readiness_csv_path = temp.path().join("dol-commit-readiness.csv");
    let readiness_json_path = temp.path().join("dol-commit-readiness.json");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-commit-readiness",
            report_card_path
                .to_str()
                .ok_or("report-card path is not UTF-8")?,
            persistence_path
                .to_str()
                .ok_or("persistence path is not UTF-8")?,
            cola_path.to_str().ok_or("COLA path is not UTF-8")?,
            "--readiness-id",
            "trajopt-cli-dol-commit-readiness",
            "--output-csv",
        ])
        .arg(&readiness_csv_path)
        .arg("--report-json")
        .arg(&readiness_json_path)
        .assert()
        .success()
        .stdout(predicate::str::contains(
            "openbmp trajopt dol-commit-readiness: go",
        ))
        .stdout(predicate::str::contains("wrote DOL commit-readiness CSV"));

    let csv = fs::read_to_string(readiness_csv_path)?;
    assert!(csv.starts_with("openbmp.dol_commit_readiness,1\n"));
    assert!(csv.contains("report_card,trajopt-cli-dol-report-card,go"));
    assert!(csv.contains("persistence,sha256:openbmp-dol-persistence-knockdown-v1,go"));
    assert!(csv.contains("cola,launch_cola_screening,go"));
    let report_json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(readiness_json_path)?)?;
    assert_eq!(report_json["verdict"].as_str(), Some("Go"));
    assert_eq!(
        report_json["readiness_id"].as_str(),
        Some("trajopt-cli-dol-commit-readiness")
    );
    Ok(())
}

#[test]
fn trajopt_dol_commit_readiness_cli_fails_closed_on_no_go() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let (report_card_path, persistence_path, cola_path) =
        write_dol_commit_readiness_inputs(temp.path(), true)?;
    let readiness_csv_path = temp.path().join("dol-commit-readiness.csv");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-commit-readiness",
            report_card_path
                .to_str()
                .ok_or("report-card path is not UTF-8")?,
            persistence_path
                .to_str()
                .ok_or("persistence path is not UTF-8")?,
            cola_path.to_str().ok_or("COLA path is not UTF-8")?,
            "--readiness-id",
            "trajopt-cli-dol-commit-readiness",
            "--output-csv",
        ])
        .arg(&readiness_csv_path)
        .assert()
        .failure()
        .stdout(predicate::str::contains(
            "openbmp trajopt dol-commit-readiness: no_go",
        ))
        .stderr(predicate::str::contains("trajectory optimization"))
        .stderr(predicate::str::contains(
            "DOL commit readiness verdict is no_go",
        ));

    let csv = fs::read_to_string(readiness_csv_path)?;
    assert!(csv.contains("verdict,no_go,persistence,q_alpha_station"));
    Ok(())
}

#[test]
fn trajopt_dol_materialize_indicators_rejects_missing_telemetry_column()
-> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let telemetry_path = temp.path().join("dol-verification-telemetry.csv");
    let indicator_spec_path = temp.path().join("dol-indicator-spec.csv");
    let indicators_path = temp.path().join("dol-indicators.csv");

    fs::write(
        &telemetry_path,
        "time_s,step,dol.q_alpha_station\n0.0,0,12.0\n",
    )?;
    fs::write(
        &indicator_spec_path,
        "name,design_value,telemetry_column,reducer,allowed_abs_error\n\
         station_moment,5000.0,dol.station_moment,last,1.0\n",
    )?;

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-materialize-indicators",
            telemetry_path
                .to_str()
                .ok_or("telemetry path is not UTF-8")?,
            indicator_spec_path
                .to_str()
                .ok_or("indicator spec path is not UTF-8")?,
            "--output-csv",
        ])
        .arg(&indicators_path)
        .assert()
        .failure()
        .stderr(predicate::str::contains("trajectory optimization"))
        .stderr(predicate::str::contains("missing telemetry column"));

    assert!(!indicators_path.exists());
    Ok(())
}

#[test]
fn trajopt_dol_verify_run_cli_invokes_runner_and_writes_report() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("dol.iload");
    let update_report_path = temp.path().join("dol-update.json");
    let indicator_spec_path = temp.path().join("dol-indicator-spec.csv");
    let telemetry_path = temp.path().join("dol-verification-telemetry.csv");
    let indicators_path = temp.path().join("dol-indicators.csv");
    let report_csv_path = temp.path().join("dol-report-card.csv");
    let report_json_path = temp.path().join("dol-report-card.json");
    let scenario =
        workspace_root()?.join("crates/openbmp-cli/tests/fixtures/day-of-launch-spliced-wind.toml");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-redesign",
            scenario.to_str().ok_or("scenario path is not UTF-8")?,
            "--target-apogee-radius-m",
            "7578000",
            "--load-window-min-altitude-m",
            "0",
            "--load-window-max-altitude-m",
            "3000",
            "--airspeed-m-s",
            "250",
            "--stale-pitch-bias-rad",
            "0.03",
            "--stale-yaw-bias-rad",
            "-0.03",
            "--load-limit-angle-rad",
            "0.05",
            "--insertion-residual-limit-m",
            "1.0",
            "--source-revision",
            "trajopt-cli-test",
            "--output-iload",
        ])
        .arg(&iload_path)
        .arg("--report-json")
        .arg(&update_report_path)
        .assert()
        .success();

    fs::write(
        &indicator_spec_path,
        "name,design_value,telemetry_column,reducer,allowed_abs_error,time_min_s,time_max_s\n\
         position_z_peak,1000.0,position_z_m,max,1.0,,\n\
         mass_mean,1.0,mass_kg,mean,1.0e-12,,\n",
    )?;

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-verify-run",
            scenario.to_str().ok_or("scenario path is not UTF-8")?,
            update_report_path
                .to_str()
                .ok_or("update report path is not UTF-8")?,
            iload_path.to_str().ok_or("I-load path is not UTF-8")?,
            indicator_spec_path
                .to_str()
                .ok_or("indicator spec path is not UTF-8")?,
            "--report-id",
            "trajopt-cli-dol-verify-run",
            "--verification-provenance",
            "native_runner_telemetry_cli",
            "--min-required-margin-rad",
            "-0.01",
            "--min-not-worse-fraction",
            "0.9",
            "--output-telemetry-csv",
        ])
        .arg(&telemetry_path)
        .arg("--output-indicators-csv")
        .arg(&indicators_path)
        .arg("--output-report-csv")
        .arg(&report_csv_path)
        .arg("--report-json")
        .arg(&report_json_path)
        .assert()
        .success()
        .stdout(predicate::str::contains(
            "openbmp trajopt dol-verify-run: go",
        ))
        .stdout(predicate::str::contains("design_profile_digest="))
        .stdout(predicate::str::contains("iload_sha256=sha256:"))
        .stdout(predicate::str::contains(
            "iload_application=no_fc_verifier_scenario",
        ))
        .stdout(predicate::str::contains("wrote verifier telemetry"))
        .stdout(predicate::str::contains("wrote DOL report card CSV"));

    assert!(fs::read_to_string(&telemetry_path)?.contains("position_z_m"));
    assert!(fs::read_to_string(&indicators_path)?.contains("position_z_peak"));
    let report_json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(report_json_path)?)?;
    assert_eq!(report_json["verdict"].as_str(), Some("Go"));
    assert!(
        report_json["iload_sha256_hex"]
            .as_str()
            .is_some_and(|value| value.starts_with("sha256:"))
    );
    assert!(fs::read_to_string(report_csv_path)?.contains("openbmp.dol_report_card"));
    Ok(())
}

#[test]
fn trajopt_dol_verify_run_applies_updated_iload_to_fc_ascent_reference()
-> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("dol.iload");
    let update_report_path = temp.path().join("dol-update.json");
    let indicator_spec_path = temp.path().join("dol-fc-indicator-spec.csv");
    let telemetry_path = temp.path().join("dol-fc-verification-telemetry.csv");
    let indicators_path = temp.path().join("dol-fc-indicators.csv");
    let report_csv_path = temp.path().join("dol-fc-report-card.csv");
    let dol_design_scenario =
        workspace_root()?.join("crates/openbmp-cli/tests/fixtures/day-of-launch-spliced-wind.toml");
    let verifier_scenario = workspace_root()?
        .join("crates/openbmp-runner/tests/fixtures/fc-reference-telemetry-ascent-reference.toml");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-redesign",
            dol_design_scenario
                .to_str()
                .ok_or("DOL design scenario path is not UTF-8")?,
            "--target-apogee-radius-m",
            "7578000",
            "--load-window-min-altitude-m",
            "0",
            "--load-window-max-altitude-m",
            "3000",
            "--airspeed-m-s",
            "250",
            "--stale-pitch-bias-rad",
            "0.03",
            "--stale-yaw-bias-rad",
            "-0.03",
            "--load-limit-angle-rad",
            "0.05",
            "--insertion-residual-limit-m",
            "1.0",
            "--source-revision",
            "trajopt-cli-test",
            "--output-iload",
        ])
        .arg(&iload_path)
        .arg("--report-json")
        .arg(&update_report_path)
        .assert()
        .success();

    let update_json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&update_report_path)?)?;
    let design_profile_digest = update_json["design_profile_digest"]
        .as_str()
        .ok_or("missing design_profile_digest")?;
    fs::write(
        &indicator_spec_path,
        "name,design_value,telemetry_column,reducer,allowed_abs_error,time_min_s,time_max_s\n\
         reference_q_x,0.0,guidance.reference.q_x,max_abs,2.0,,\n\
         reference_q_y,0.0,guidance.reference.q_y,max_abs,2.0,,\n",
    )?;

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-verify-run",
            verifier_scenario
                .to_str()
                .ok_or("verifier scenario path is not UTF-8")?,
            update_report_path
                .to_str()
                .ok_or("update report path is not UTF-8")?,
            iload_path.to_str().ok_or("I-load path is not UTF-8")?,
            indicator_spec_path
                .to_str()
                .ok_or("indicator spec path is not UTF-8")?,
            "--verifier-design-profile-digest",
            design_profile_digest,
            "--report-id",
            "trajopt-cli-dol-verify-run-fc",
            "--verification-provenance",
            "native_runner_fc_ascent_reference",
            "--min-required-margin-rad",
            "-0.01",
            "--min-not-worse-fraction",
            "0.9",
            "--output-telemetry-csv",
        ])
        .arg(&telemetry_path)
        .arg("--output-indicators-csv")
        .arg(&indicators_path)
        .arg("--output-report-csv")
        .arg(&report_csv_path)
        .assert()
        .success()
        .stdout(predicate::str::contains(
            "iload_application=fc_ascent_reference_pitch_yaw_program",
        ))
        .stdout(predicate::str::contains("applied_refs=1"))
        .stdout(predicate::str::contains("pitch_points=4"))
        .stdout(predicate::str::contains("yaw_points=4"));

    let telemetry = fs::read_to_string(telemetry_path)?;
    assert!(telemetry.contains("guidance.reference.q_x"));
    assert!(fs::read_to_string(indicators_path)?.contains("reference_q_y"));
    assert!(fs::read_to_string(report_csv_path)?.contains("openbmp.dol_report_card"));
    Ok(())
}

#[test]
fn trajopt_dol_verify_sil_applies_updated_iload_with_evidence() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("dol.iload");
    let update_report_path = temp.path().join("dol-update.json");
    let indicator_spec_path = temp.path().join("dol-sil-indicator-spec.csv");
    let telemetry_path = temp.path().join("dol-sil-verification-telemetry.csv");
    let indicators_path = temp.path().join("dol-sil-indicators.csv");
    let report_csv_path = temp.path().join("dol-sil-report-card.csv");
    let evidence_dir = temp.path().join("dol-sil-evidence");
    let package_path = temp.path().join("dol-sil-package.toml");
    let dol_design_scenario =
        workspace_root()?.join("crates/openbmp-cli/tests/fixtures/day-of-launch-spliced-wind.toml");
    let verifier_scenario = workspace_root()?
        .join("crates/openbmp-runner/tests/fixtures/fc-reference-telemetry-ascent-reference.toml");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-redesign",
            dol_design_scenario
                .to_str()
                .ok_or("DOL design scenario path is not UTF-8")?,
            "--target-apogee-radius-m",
            "7578000",
            "--load-window-min-altitude-m",
            "0",
            "--load-window-max-altitude-m",
            "3000",
            "--airspeed-m-s",
            "250",
            "--stale-pitch-bias-rad",
            "0.03",
            "--stale-yaw-bias-rad",
            "-0.03",
            "--load-limit-angle-rad",
            "0.05",
            "--insertion-residual-limit-m",
            "1.0",
            "--source-revision",
            "trajopt-cli-test",
            "--output-iload",
        ])
        .arg(&iload_path)
        .arg("--report-json")
        .arg(&update_report_path)
        .assert()
        .success();

    let update_json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&update_report_path)?)?;
    let design_profile_digest = update_json["design_profile_digest"]
        .as_str()
        .ok_or("missing design_profile_digest")?;
    fs::write(
        &indicator_spec_path,
        "name,design_value,telemetry_column,reducer,allowed_abs_error,time_min_s,time_max_s\n\
         reference_q_x,0.0,guidance.reference.q_x,max_abs,2.0,,\n\
         reference_q_y,0.0,guidance.reference.q_y,max_abs,2.0,,\n",
    )?;
    let scenario_path = verifier_scenario
        .to_str()
        .ok_or("verifier scenario path is not UTF-8")?
        .replace('"', "\\\"");
    fs::write(
        &package_path,
        format!(
            "[package]\nid = \"test.dol-verify-sil\"\nversion = \"0.0.1\"\n\n[files]\nscenario = \"{scenario_path}\"\n\n[[test_cases]]\nid = \"fc\"\n"
        ),
    )?;

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-verify-sil",
            package_path.to_str().ok_or("package path is not UTF-8")?,
            update_report_path
                .to_str()
                .ok_or("update report path is not UTF-8")?,
            iload_path.to_str().ok_or("I-load path is not UTF-8")?,
            indicator_spec_path
                .to_str()
                .ok_or("indicator spec path is not UTF-8")?,
            "--case",
            "fc",
            "--verifier-design-profile-digest",
            design_profile_digest,
            "--report-id",
            "trajopt-cli-dol-verify-sil",
            "--verification-provenance",
            "native_sil_stimulated_telemetry",
            "--min-required-margin-rad",
            "-0.01",
            "--min-not-worse-fraction",
            "0.9",
            "--output-telemetry-csv",
        ])
        .arg(&telemetry_path)
        .arg("--output-indicators-csv")
        .arg(&indicators_path)
        .arg("--output-report-csv")
        .arg(&report_csv_path)
        .arg("--output-evidence-dir")
        .arg(&evidence_dir)
        .assert()
        .success()
        .stdout(predicate::str::contains(
            "openbmp trajopt dol-verify-sil: go",
        ))
        .stdout(predicate::str::contains("package=test.dol-verify-sil"))
        .stdout(predicate::str::contains(
            "iload_application=fc_ascent_reference_pitch_yaw_program",
        ))
        .stdout(predicate::str::contains("applied_refs=1"))
        .stdout(predicate::str::contains("sil_stimuli=3"))
        .stdout(predicate::str::contains("wrote SIL evidence manifest"));

    let telemetry = fs::read_to_string(telemetry_path)?;
    assert!(telemetry.contains("guidance.reference.q_x"));
    assert!(fs::read_to_string(indicators_path)?.contains("reference_q_y"));
    assert!(fs::read_to_string(report_csv_path)?.contains("openbmp.dol_report_card"));
    let evidence: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(evidence_dir.join("manifest.json"))?)?;
    assert_eq!(evidence["package_id"].as_str(), Some("test.dol-verify-sil"));
    assert_eq!(evidence["case_id"].as_str(), Some("fc"));
    let stimuli = evidence["stimuli"]
        .as_array()
        .ok_or("missing evidence stimuli")?;
    assert!(
        stimuli
            .iter()
            .any(|record| { record["target"].as_str() == Some("fc.ascent_reference.schedule_s") })
    );
    assert!(
        stimuli
            .iter()
            .any(|record| record["target"].as_str() == Some("fc.ascent_reference.yaw_rad"))
    );
    Ok(())
}

#[test]
fn trajopt_dol_verify_run_rejects_update_binding_mismatch() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("dol.iload");
    let update_report_path = temp.path().join("dol-update.json");
    let indicator_spec_path = temp.path().join("dol-indicator-spec.csv");
    let telemetry_path = temp.path().join("dol-verification-telemetry.csv");
    let indicators_path = temp.path().join("dol-indicators.csv");
    let report_csv_path = temp.path().join("dol-report-card.csv");
    let scenario =
        workspace_root()?.join("crates/openbmp-cli/tests/fixtures/day-of-launch-spliced-wind.toml");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-redesign",
            scenario.to_str().ok_or("scenario path is not UTF-8")?,
            "--target-apogee-radius-m",
            "7578000",
            "--load-window-min-altitude-m",
            "0",
            "--load-window-max-altitude-m",
            "3000",
            "--airspeed-m-s",
            "250",
            "--stale-pitch-bias-rad",
            "0.03",
            "--stale-yaw-bias-rad",
            "-0.03",
            "--load-limit-angle-rad",
            "0.05",
            "--insertion-residual-limit-m",
            "1.0",
            "--source-revision",
            "trajopt-cli-test",
            "--output-iload",
        ])
        .arg(&iload_path)
        .arg("--report-json")
        .arg(&update_report_path)
        .assert()
        .success();

    fs::write(
        &indicator_spec_path,
        "name,design_value,telemetry_column,reducer,allowed_abs_error\n\
         position_z_peak,1000.0,position_z_m,max,1.0\n",
    )?;

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-verify-run",
            scenario.to_str().ok_or("scenario path is not UTF-8")?,
            update_report_path
                .to_str()
                .ok_or("update report path is not UTF-8")?,
            iload_path.to_str().ok_or("I-load path is not UTF-8")?,
            indicator_spec_path
                .to_str()
                .ok_or("indicator spec path is not UTF-8")?,
            "--verifier-design-profile-digest",
            "not-the-dol-update-digest",
            "--min-required-margin-rad",
            "-0.01",
            "--min-not-worse-fraction",
            "0.9",
            "--output-telemetry-csv",
        ])
        .arg(&telemetry_path)
        .arg("--output-indicators-csv")
        .arg(&indicators_path)
        .arg("--output-report-csv")
        .arg(&report_csv_path)
        .assert()
        .failure()
        .stderr(predicate::str::contains("trajectory optimization"))
        .stderr(predicate::str::contains("design-profile digest mismatch"));

    assert!(!telemetry_path.exists());
    assert!(!indicators_path.exists());
    assert!(!report_csv_path.exists());
    Ok(())
}

#[test]
fn trajopt_dol_verify_run_rejects_updated_iload_digest_mismatch() -> Result<(), Box<dyn Error>> {
    let temp = Builder::new().prefix("openbmp-trajopt-cli").tempdir()?;
    let iload_path = temp.path().join("dol.iload");
    let tampered_iload_path = temp.path().join("tampered-dol.iload");
    let update_report_path = temp.path().join("dol-update.json");
    let indicator_spec_path = temp.path().join("dol-indicator-spec.csv");
    let telemetry_path = temp.path().join("dol-verification-telemetry.csv");
    let indicators_path = temp.path().join("dol-indicators.csv");
    let report_csv_path = temp.path().join("dol-report-card.csv");
    let scenario =
        workspace_root()?.join("crates/openbmp-cli/tests/fixtures/day-of-launch-spliced-wind.toml");

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-redesign",
            scenario.to_str().ok_or("scenario path is not UTF-8")?,
            "--target-apogee-radius-m",
            "7578000",
            "--load-window-min-altitude-m",
            "0",
            "--load-window-max-altitude-m",
            "3000",
            "--airspeed-m-s",
            "250",
            "--stale-pitch-bias-rad",
            "0.03",
            "--stale-yaw-bias-rad",
            "-0.03",
            "--load-limit-angle-rad",
            "0.05",
            "--insertion-residual-limit-m",
            "1.0",
            "--source-revision",
            "trajopt-cli-test",
            "--output-iload",
        ])
        .arg(&iload_path)
        .arg("--report-json")
        .arg(&update_report_path)
        .assert()
        .success();

    let mut tampered = fs::read(&iload_path)?;
    tampered.push(0);
    fs::write(&tampered_iload_path, tampered)?;
    fs::write(
        &indicator_spec_path,
        "name,design_value,telemetry_column,reducer,allowed_abs_error\n\
         position_z_peak,1000.0,position_z_m,max,1.0\n",
    )?;

    Command::cargo_bin("openbmp")?
        .args([
            "trajopt",
            "dol-verify-run",
            scenario.to_str().ok_or("scenario path is not UTF-8")?,
            update_report_path
                .to_str()
                .ok_or("update report path is not UTF-8")?,
            tampered_iload_path
                .to_str()
                .ok_or("tampered I-load path is not UTF-8")?,
            indicator_spec_path
                .to_str()
                .ok_or("indicator spec path is not UTF-8")?,
            "--min-required-margin-rad",
            "-0.01",
            "--min-not-worse-fraction",
            "0.9",
            "--output-telemetry-csv",
        ])
        .arg(&telemetry_path)
        .arg("--output-indicators-csv")
        .arg(&indicators_path)
        .arg("--output-report-csv")
        .arg(&report_csv_path)
        .assert()
        .failure()
        .stderr(predicate::str::contains("trajectory optimization"))
        .stderr(predicate::str::contains("updated I-load digest mismatch"));

    assert!(!telemetry_path.exists());
    assert!(!indicators_path.exists());
    assert!(!report_csv_path.exists());
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct TrajoptTolerance {
    target_apogee_radius_m: f64,
    residual_tolerance_m: f64,
    max_iterations: usize,
}

fn read_trajopt_tolerance() -> Result<TrajoptTolerance, Box<dyn Error>> {
    let path = workspace_root()?.join("scenarios/trajopt-two-body-apogee/tolerance.toml");
    let value: toml::Value = toml::from_str(&fs::read_to_string(path)?)?;
    let table = value
        .get("trajopt_two_body_apogee")
        .and_then(toml::Value::as_table)
        .ok_or("missing trajopt_two_body_apogee tolerance table")?;
    let target_apogee_radius_m = table
        .get("target_apogee_radius_m")
        .and_then(toml::Value::as_float)
        .ok_or("missing target_apogee_radius_m")?;
    let residual_tolerance_m = table
        .get("residual_tolerance_m")
        .and_then(toml::Value::as_float)
        .ok_or("missing residual_tolerance_m")?;
    let max_iterations = table
        .get("max_iterations")
        .and_then(toml::Value::as_integer)
        .ok_or("missing max_iterations")?
        .try_into()?;
    Ok(TrajoptTolerance {
        target_apogee_radius_m,
        residual_tolerance_m,
        max_iterations,
    })
}

fn write_dol_commit_readiness_inputs(
    dir: &std::path::Path,
    persistence_no_go: bool,
) -> Result<(PathBuf, PathBuf, PathBuf), Box<dyn Error>> {
    let report_card_path = dir.join("dol-report-card.json");
    let persistence_path = dir.join("dol-persistence.json");
    let cola_path = dir.join("dol-cola.json");
    let total_knockdown_rad = (0.006_f64 * 0.006 + 0.008_f64 * 0.008 + 0.005_f64 * 0.005).sqrt();
    let nominal_margin_rad = if persistence_no_go { 0.005 } else { 0.05 };
    let knocked_down_margin_rad = nominal_margin_rad - total_knockdown_rad;
    let (persistence_verdict, persistence_binding) = if persistence_no_go {
        (
            serde_json::Value::String("NoGo".to_owned()),
            serde_json::Value::String("q_alpha_station".to_owned()),
        )
    } else {
        (
            serde_json::Value::String("Go".to_owned()),
            serde_json::Value::Null,
        )
    };

    write_json(
        &report_card_path,
        serde_json::json!({
            "schema_version": 1,
            "report_id": "trajopt-cli-dol-report-card",
            "verification_provenance": "runner_6dof_synthetic_cli",
            "design_profile_digest": "sha256:wind",
            "solver_provenance": "dol_spliced_wind_recenter:test",
            "iload_sha256_hex": "sha256:iload",
            "load_window_sample_count": 3,
            "redesigned_min_margin_rad": 0.02,
            "min_required_margin_rad": 0.0,
            "redesigned_not_worse_fraction": 1.0,
            "min_not_worse_fraction": 0.9,
            "insertion_residual_norm_m": 0.01,
            "insertion_residual_limit_m": 1.0,
            "indicators": [
                {
                    "name": "q_alpha_station",
                    "design_value": 12.0,
                    "verified_value": 12.05,
                    "abs_error": 0.05,
                    "allowed_abs_error": 0.1,
                    "passed": true
                }
            ],
            "verdict": "Go",
            "binding_indicator": null
        }),
    )?;
    write_json(
        &persistence_path,
        serde_json::json!({
            "schema_version": 1,
            "deck_hash": "sha256:openbmp-dol-persistence-knockdown-v1",
            "lead_time_s": 100.0,
            "min_required_margin_rad": 0.0,
            "indicators": [
                {
                    "name": "q_alpha_station",
                    "altitude_m": 500.0,
                    "nominal_margin_rad": nominal_margin_rad,
                    "persistence_sigma_m_s": 3.0,
                    "persistence_knockdown_rad": 0.006,
                    "gust_knockdown_rad": 0.008,
                    "system_knockdown_rad": 0.005,
                    "total_knockdown_rad": total_knockdown_rad,
                    "knocked_down_margin_rad": knocked_down_margin_rad,
                    "passed": !persistence_no_go
                }
            ],
            "min_knocked_down_margin_rad": knocked_down_margin_rad,
            "verdict": persistence_verdict,
            "binding_indicator": persistence_binding
        }),
    )?;
    write_json(
        &cola_path,
        serde_json::json!({
            "schema_version": 1,
            "vehicle_keepout_radius_m": 50.0,
            "slots": [
                {
                    "slot_id": "clear-slot",
                    "launch_time_s": 0.0,
                    "encounters": [
                        {
                            "slot_id": "clear-slot",
                            "object_id": "synthetic-catalog-001",
                            "trajectory_time_s": 50.0,
                            "closest_time_s": 50.0,
                            "miss_distance_m": 400.0,
                            "vehicle_keepout_radius_m": 50.0,
                            "object_radius_m": 10.0,
                            "clearance_m": 340.0,
                            "blocked": false
                        }
                    ],
                    "min_clearance_m": 340.0,
                    "blocked": false,
                    "binding_object": null
                }
            ],
            "blocked_slot_count": 0,
            "verdict": "Go",
            "binding_slot": null,
            "binding_object": null
        }),
    )?;
    Ok((report_card_path, persistence_path, cola_path))
}

fn write_json(path: &std::path::Path, value: serde_json::Value) -> Result<(), Box<dyn Error>> {
    fs::write(path, serde_json::to_vec_pretty(&value)?)?;
    Ok(())
}

fn workspace_root() -> Result<PathBuf, Box<dyn Error>> {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(std::path::Path::parent)
        .map(std::path::Path::to_path_buf)
        .ok_or_else(|| "workspace root does not exist".into())
}
