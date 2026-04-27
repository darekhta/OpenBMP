//! Snapshot tests for the `openbmp` binary user surface.
//!
//! These spawn the compiled binary and capture stdout / stderr /
//! exit-code via `assert_cmd` + `insta-cmd`. The goal is to lock in
//! the command-line surface — flags, help text shape, error messages
//! — so a future Phase-2 refactor doesn't accidentally break the
//! contract.
//!
//! Snapshots are stored under `tests/snapshots/`. Update with
//! `cargo insta review` after deliberate UX changes.

#![allow(clippy::expect_used, clippy::panic)]

use std::fs;
use std::path::{Path, PathBuf};

use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use insta_cmd::assert_cmd_snapshot;
use tempfile::{Builder, TempDir};

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root must exist")
}

fn openbmp() -> std::process::Command {
    std::process::Command::cargo_bin("openbmp").expect("binary built")
}

fn tempdir(label: &str) -> TempDir {
    Builder::new()
        .prefix(&format!("openbmp-snap-{label}-"))
        .tempdir()
        .expect("temp dir")
}

#[test]
fn help_lists_all_subcommands() {
    let mut cmd = openbmp();
    cmd.arg("--help");
    assert_cmd_snapshot!("help", cmd);
}

#[test]
fn check_on_canonical_scenario_succeeds() {
    // Resolved-path output is machine-specific (absolute paths under
    // the workspace), so we don't snapshot the full output. We just
    // assert the success line and the scenario-name marker.
    let scenario = workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml");
    let mut cmd = openbmp();
    cmd.arg("check").arg(&scenario);
    let assert = cmd.assert().success();
    let stdout = String::from_utf8_lossy(&assert.get_output().stdout).to_string();
    assert!(
        stdout.starts_with("openbmp check: ok"),
        "stdout was: {stdout}"
    );
    assert!(stdout.contains("constant-acceleration-drop"));
    assert!(stdout.contains("ValidatedToy"));
}

#[test]
fn check_with_unknown_field_fails_with_structured_error() {
    let temp = tempdir("badfield");
    let staged = temp.path().join("scenario.toml");
    let original = fs::read_to_string(
        workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml"),
    )
    .expect("read canonical");
    fs::write(
        &staged,
        original.replace("mass_kg = 1.0", "mass_kg = 1.0\nextra_kg = 2.0"),
    )
    .expect("write staged");

    let mut cmd = openbmp();
    cmd.arg("check").arg(&staged);
    let assert = cmd.assert();
    let output = assert.get_output();
    assert!(
        !output.status.success(),
        "exit status should be non-zero, was {:?}",
        output.status
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("openbmp: scenario error"),
        "stderr should explain the failure, got: {stderr}"
    );
}

#[test]
fn run_writes_telemetry_outputs_and_reports_stop_reason() {
    let temp = tempdir("run");
    let staged = temp.path().join("scenario.toml");
    let csv = temp.path().join("out.csv");
    let parquet = temp.path().join("out.parquet");
    let original = fs::read_to_string(
        workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml"),
    )
    .expect("read canonical");
    let rewritten = original
        .replace(
            "output.csv = \"out/constant-acceleration-drop.csv\"",
            &format!("output.csv = \"{}\"", csv.display()),
        )
        .replace(
            "output.parquet = \"out/constant-acceleration-drop.parquet\"",
            &format!("output.parquet = \"{}\"", parquet.display()),
        );
    fs::write(&staged, rewritten).expect("write staged");

    let mut cmd = openbmp();
    cmd.arg("run").arg(&staged);
    let assert = cmd.assert().success();
    let stdout = String::from_utf8_lossy(&assert.get_output().stdout).to_string();
    assert!(stdout.contains("openbmp run: ok"));
    assert!(stdout.contains("1000 steps"));
    assert!(stdout.contains("stop = end-time"));
    assert!(parquet.exists());
    assert!(csv.exists());
}

#[test]
fn run_rejects_negative_gravity_magnitude() {
    let temp = tempdir("negative_gravity");
    let staged = temp.path().join("scenario.toml");
    let original = fs::read_to_string(
        workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml"),
    )
    .expect("read canonical");
    fs::write(
        &staged,
        original.replace("gravity_m_s2  = 9.80665", "gravity_m_s2  = -9.80665"),
    )
    .expect("write staged");

    let mut cmd = openbmp();
    cmd.arg("run").arg(&staged);
    let assert = cmd.assert();
    let output = assert.get_output();
    assert!(
        !output.status.success(),
        "exit status should be non-zero, was {:?}",
        output.status
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("non-negative magnitude"),
        "stderr should explain the rejected gravity magnitude, got: {stderr}"
    );
}

#[test]
fn diff_reports_identical_for_self_compare() {
    let temp = tempdir("diff_identical");
    let staged = temp.path().join("scenario.toml");
    let parquet = temp.path().join("out.parquet");
    let csv = temp.path().join("out.csv");
    let original = fs::read_to_string(
        workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml"),
    )
    .expect("read canonical");
    let rewritten = original
        .replace(
            "output.csv = \"out/constant-acceleration-drop.csv\"",
            &format!("output.csv = \"{}\"", csv.display()),
        )
        .replace(
            "output.parquet = \"out/constant-acceleration-drop.parquet\"",
            &format!("output.parquet = \"{}\"", parquet.display()),
        );
    fs::write(&staged, rewritten).expect("write staged");
    openbmp().arg("run").arg(&staged).assert().success();

    let mut cmd = openbmp();
    cmd.arg("diff").arg(&parquet).arg(&parquet);
    let assert = cmd.assert().success();
    let stdout = String::from_utf8_lossy(&assert.get_output().stdout).to_string();
    assert!(
        stdout.contains("openbmp diff: identical"),
        "stdout was: {stdout}"
    );
    assert!(stdout.contains("9 columns matched"), "stdout was: {stdout}");
}
