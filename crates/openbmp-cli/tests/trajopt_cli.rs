//! CLI regression tests for offline trajectory optimization commands.

use std::error::Error;
use std::fs;

use assert_cmd::Command;
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
