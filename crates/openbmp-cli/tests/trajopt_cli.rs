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

fn workspace_root() -> Result<PathBuf, Box<dyn Error>> {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(std::path::Path::parent)
        .map(std::path::Path::to_path_buf)
        .ok_or_else(|| "workspace root does not exist".into())
}
