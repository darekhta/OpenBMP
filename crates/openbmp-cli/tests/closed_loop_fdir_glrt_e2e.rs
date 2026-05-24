//! End-to-end test: closed-loop attitude-hold pipeline
//! with the Willsky 1976 windowed-mean-shift GLRT FDIR detector
//! wired in.
//!
//! Asserts the wiring behaviour:
//!
//! 1. The scenario runs to completion via the `openbmp` CLI binary
//!    with `stop_label = "end-time"` after 1000 RK4 steps.
//! 2. Two reruns produce byte-identical Parquet — the new EKF
//!    whitened-innovation export and the new GLRT detector both
//!    honour the determinism contract (locked operand order,
//!    no FMA, no system RNG).
//!
//! Trip semantics under synthetic faults are covered by
//! `crates/openbmp-fc/src/glrt.rs` (9 unit tests, including a 4σ
//! step-injection that asserts the detector trips within ±2 samples
//! of the true τ).
//!
//! `‖ν̃‖² = chi2` invariants are covered by
//! `crates/openbmp-fc/src/estimator.rs` (5 invariant tests over
//! GNSS / baro / mag whitened innovations).

#![allow(clippy::expect_used, clippy::float_cmp, clippy::panic)]

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
        .prefix(&format!("openbmp-{label}-"))
        .tempdir()
        .expect("tempdir")
}

fn run_to_parquet(label: &str) -> PathBuf {
    // The scenario references sensor noise budgets via `../../data/...`
    // relative paths, so the scenario file must stay at its
    // committed location. We override only the parquet output path.
    let scenario = workspace_root().join("scenarios/closed-loop-fdir-glrt/scenario.toml");
    let temp = tempdir(label);
    let parquet = temp.path().join("closed-loop-fdir-glrt.parquet");
    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(&scenario)
        .arg("--output-parquet")
        .arg(&parquet);
    let assert = cmd.assert().success();
    let stdout = String::from_utf8_lossy(&assert.get_output().stdout).to_string();
    assert!(
        stdout.starts_with("openbmp run: ok"),
        "stdout was: {stdout}"
    );
    assert!(parquet.exists(), "parquet must be written");
    let copy = std::env::temp_dir().join(format!("openbmp-{label}.parquet"));
    let _ = fs::remove_file(&copy);
    fs::copy(&parquet, &copy).expect("copy parquet");
    copy
}

#[test]
fn closed_loop_fdir_glrt_runs_to_completion() {
    let parquet = run_to_parquet("fdir-glrt-run");
    let bytes = fs::read(&parquet).expect("read parquet");
    assert!(!bytes.is_empty(), "parquet must be non-empty");
    let _ = fs::remove_file(&parquet);
}

#[test]
fn closed_loop_fdir_glrt_is_byte_stable_across_two_runs() {
    let a = run_to_parquet("fdir-glrt-rerun-a");
    let b = run_to_parquet("fdir-glrt-rerun-b");
    let bytes_a = fs::read(&a).expect("read a");
    let bytes_b = fs::read(&b).expect("read b");
    assert_eq!(
        bytes_a, bytes_b,
        "two reruns of closed-loop-fdir-glrt must produce byte-identical Parquet \
         (the windowed-mean-shift GLRT and the EKF whitened-innovation export \
         both honour the determinism contract)",
    );
    let _ = fs::remove_file(&a);
    let _ = fs::remove_file(&b);
}
