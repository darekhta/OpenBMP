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

fn niskanen_stage_path(temp: &TempDir) -> PathBuf {
    let scenario_dir = temp.path().join("scenarios/sounding-rocket");
    fs::create_dir_all(&scenario_dir).expect("scenario dir");
    scenario_dir.join("scenario.toml")
}

fn materialise_niskanen_refs(temp: &TempDir) {
    let data_dir = temp.path().join("data");
    let aero_dir = data_dir.join("aero");
    let motor_dir = data_dir.join("motors");
    fs::create_dir_all(&aero_dir).expect("aero dir");
    fs::create_dir_all(&motor_dir).expect("motor dir");
    fs::copy(
        workspace_root().join("data/aero/synthetic-niskanen-ch6-rocket.toml"),
        aero_dir.join("synthetic-niskanen-ch6-rocket.toml"),
    )
    .expect("copy deck");
    fs::copy(
        workspace_root().join("data/motors/estes-c6-eng-derived.toml"),
        motor_dir.join("estes-c6-eng-derived.toml"),
    )
    .expect("copy motor");
}

fn toml_literal_path(path: &Path) -> String {
    let path = path.to_string_lossy();
    assert!(
        !path.contains('\''),
        "test temp paths must be representable as TOML literal strings: {path}",
    );
    format!("'{path}'")
}

#[test]
fn help_lists_all_subcommands() {
    let mut cmd = openbmp();
    cmd.arg("--help");
    let assert = cmd.assert().success();
    let stdout = String::from_utf8_lossy(&assert.get_output().stdout)
        .replace("Usage: openbmp.exe", "Usage: openbmp");
    let stderr = String::from_utf8_lossy(&assert.get_output().stderr);
    insta::assert_snapshot!(
        "help",
        format!(
            "success: true\nexit_code: 0\n----- stdout -----\n{stdout}\n----- stderr -----\n{stderr}"
        )
    );
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
fn check_on_niskanen_scenario_resolves_aero_motor_and_pins() {
    // Phase-2.10 canonical scenario: parses, validates, resolves the
    // aero deck and Estes C6 motor file, and surfaces the SHA-256
    // digests of both. Pin verification is exercised by the negative
    // tests below.
    let scenario = workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6.toml");
    let mut cmd = openbmp();
    cmd.arg("check").arg(&scenario);
    let assert = cmd.assert().success();
    let stdout = String::from_utf8_lossy(&assert.get_output().stdout).to_string();
    assert!(
        stdout.starts_with("openbmp check: ok"),
        "stdout was: {stdout}"
    );
    assert!(stdout.contains("niskanen-2009-chapter6"));
    assert!(stdout.contains("Checked"));
    // Aero deck reference resolved with declared pin.
    assert!(
        stdout.contains("aero.deck -> ")
            && stdout.contains(
                "(sha256:cd862c2af98a1f28dc86c6e754d311c7a724081ca91b80704ad89b2ec4cb5c27)"
            ),
        "stdout was: {stdout}"
    );
    // Motor reference resolved with declared pin.
    assert!(
        stdout.contains("propulsion.motor.file -> ")
            && stdout.contains(
                "(sha256:da8272d3a7a135046c614e51b279971d37cac376f7aaaffdedc3ccc14d50ad4e)"
            ),
        "stdout was: {stdout}"
    );
}

#[test]
fn check_on_niskanen_with_corrupt_pin_fails_closed() {
    // Take the canonical Niskanen scenario, swap in an obviously-wrong
    // pin for the aero deck, and confirm the parser fails closed with
    // a SHA-256 mismatch error rather than silently running.
    let temp = tempdir("badpin");
    let staged = niskanen_stage_path(&temp);
    let canonical = workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6.toml");
    let original = fs::read_to_string(&canonical).expect("read canonical");
    let bad_pin = "0".repeat(64);
    let rewritten = original.replace(
        "deck_sha256  = \"cd862c2af98a1f28dc86c6e754d311c7a724081ca91b80704ad89b2ec4cb5c27\"",
        &format!("deck_sha256  = \"{bad_pin}\""),
    );
    // Sanity: the replace actually did something.
    assert_ne!(rewritten, original, "pin field not found in canonical");
    fs::write(&staged, rewritten).expect("write staged");
    // Materialise the referenced files via copy so the resolver can
    // read them from `<temp>/scenarios/sounding-rocket/../../data/...`.
    materialise_niskanen_refs(&temp);

    let mut cmd = openbmp();
    cmd.arg("check").arg(&staged);
    let assert = cmd.assert();
    let output = assert.get_output();
    assert!(
        !output.status.success(),
        "exit status should be non-zero, was {:?}",
        output.status
    );
    let stderr = String::from_utf8_lossy(&output.stderr).to_string();
    assert!(
        stderr.contains("SHA-256 mismatch"),
        "stderr should explain the failure, got: {stderr}"
    );
}

#[test]
fn check_on_niskanen_with_missing_motor_file_fails_closed() {
    // Take the canonical Niskanen scenario, repoint the motor at a
    // non-existent path, and confirm the parser fails closed with the
    // referenced-file-missing error rather than silently running.
    let temp = tempdir("missingmotor");
    let staged = niskanen_stage_path(&temp);
    let canonical = workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6.toml");
    let original = fs::read_to_string(&canonical).expect("read canonical");
    let rewritten = original.replace(
        "../../data/motors/estes-c6-eng-derived.toml",
        "../../data/motors/no-such-motor.toml",
    );
    assert_ne!(rewritten, original, "motor path not found in canonical");
    fs::write(&staged, rewritten).expect("write staged");

    let mut cmd = openbmp();
    cmd.arg("check").arg(&staged);
    let assert = cmd.assert();
    let output = assert.get_output();
    assert!(
        !output.status.success(),
        "exit status should be non-zero, was {:?}",
        output.status
    );
    let stderr = String::from_utf8_lossy(&output.stderr).to_string();
    assert!(
        stderr.contains("could not be read"),
        "stderr should explain the failure, got: {stderr}"
    );
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
            &format!("output.csv = {}", toml_literal_path(&csv)),
        )
        .replace(
            "output.parquet = \"out/constant-acceleration-drop.parquet\"",
            &format!("output.parquet = {}", toml_literal_path(&parquet)),
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
fn run_on_niskanen_with_corrupt_pin_fails_before_simulation() {
    // Phase-2.11.A: SHA-256 pin verification fires *before* the kernel
    // is constructed. A bad pin must surface as a scenario error with
    // exit-code 2 ("SHA-256 mismatch"), never as a partial-success run
    // that silently dropped the pin check.
    let temp = tempdir("niskanen_run_badpin");
    let staged = temp.path().join("scenarios/sounding-rocket/scenario.toml");
    fs::create_dir_all(staged.parent().expect("scenario parent")).expect("scenario dir");
    let canonical = workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6.toml");
    let original = fs::read_to_string(&canonical).expect("read canonical");
    let bad_pin = "0".repeat(64);
    let rewritten = original.replace(
        "deck_sha256  = \"cd862c2af98a1f28dc86c6e754d311c7a724081ca91b80704ad89b2ec4cb5c27\"",
        &format!("deck_sha256  = \"{bad_pin}\""),
    );
    assert_ne!(rewritten, original, "pin field not found in canonical");
    fs::write(&staged, rewritten).expect("write staged");

    // Materialise the referenced files via copy so the resolver can
    // read them from `<temp>/scenarios/sounding-rocket/../../data/...`.
    let aero_dir = temp.path().join("data/aero");
    let motor_dir = temp.path().join("data/motors");
    fs::create_dir_all(&aero_dir).expect("aero dir");
    fs::create_dir_all(&motor_dir).expect("motor dir");
    fs::copy(
        workspace_root().join("data/aero/synthetic-niskanen-ch6-rocket.toml"),
        aero_dir.join("synthetic-niskanen-ch6-rocket.toml"),
    )
    .expect("copy deck");
    fs::copy(
        workspace_root().join("data/motors/estes-c6-eng-derived.toml"),
        motor_dir.join("estes-c6-eng-derived.toml"),
    )
    .expect("copy motor");

    let parquet = temp.path().join("out.parquet");
    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(&staged)
        .arg("--output-parquet")
        .arg(&parquet);
    let assert = cmd.assert();
    let output = assert.get_output();
    assert!(
        !output.status.success(),
        "exit status should be non-zero, was {:?}",
        output.status
    );
    let stderr = String::from_utf8_lossy(&output.stderr).to_string();
    assert!(
        stderr.contains("SHA-256 mismatch"),
        "stderr should explain the failure, got: {stderr}"
    );
    assert!(
        !parquet.exists(),
        "parquet must not be written when pin verification fails"
    );
}

#[test]
fn run_on_rigid_body_scenario_fails_with_unsupported() {
    // Phase-2.11.A: `vehicle.kind = "rigid_body"` is rejected closed
    // until Phase 3 ships rigid mass-property scenario fields and
    // rigid force adapters. Verify the diagnostic surfaces clearly.
    let temp = tempdir("rigid_body_unsupported");
    let staged = temp.path().join("scenario.toml");
    let canonical = workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml");
    let original = fs::read_to_string(&canonical).expect("read canonical");
    // Convert the analytic-toy point-mass scenario to a rigid_body
    // shape just enough to trip the runner's vehicle-kind dispatch.
    let rewritten = original
        .replace("kind = \"point_mass\"", "kind = \"rigid_body\"")
        .replace(
            "initial_velocity_eci_m_s = [0.0, 0.0, 0.0]",
            "initial_velocity_eci_m_s = [0.0, 0.0, 0.0]\n\
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]\n\
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]",
        );
    fs::write(&staged, rewritten).expect("write staged");

    let mut cmd = openbmp();
    cmd.arg("run").arg(&staged);
    let assert = cmd.assert();
    let output = assert.get_output();
    assert!(
        !output.status.success(),
        "exit status should be non-zero, was {:?}",
        output.status
    );
    let stderr = String::from_utf8_lossy(&output.stderr).to_string();
    assert!(
        stderr.contains("rigid_body") && stderr.contains("Phase 3"),
        "stderr should explain rigid_body is deferred, got: {stderr}"
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
            &format!("output.csv = {}", toml_literal_path(&csv)),
        )
        .replace(
            "output.parquet = \"out/constant-acceleration-drop.parquet\"",
            &format!("output.parquet = {}", toml_literal_path(&parquet)),
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
