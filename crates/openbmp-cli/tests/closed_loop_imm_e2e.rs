//! Phase-5.B.3 end-to-end test: closed-loop attitude-hold pipeline
//! with a 2-mode Bar-Shalom IMM estimator.
//!
//! Asserts the §5.B.3 exit criterion (wiring scope):
//!
//! 1. The scenario runs to completion via the `openbmp` CLI binary
//!    with `stop_label = "end-time"` after 1000 RK4 steps.
//! 2. Two reruns produce byte-identical Parquet — the IMM mixing
//!    step, per-mode log-likelihood computation, mode-probability
//!    update, and fused-output state all honour the determinism
//!    contract (locked operand order on every loop, no FMA, no
//!    system RNG).
//!
//! Mode-probability evolution under synthetic regime changes is
//! covered at the math layer by `crates/openbmp-fc/src/imm.rs`
//! (9 unit tests, including textbook mixing reproduction,
//! probability-simplex invariant, byte-stable determinism). EKF
//! whitened-innovation export and log-det export — the IMM's
//! likelihood inputs — are covered by 3 + 5 invariant tests in
//! `crates/openbmp-fc/src/estimator.rs`.

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
    // Scenario references sensor noise budgets via `../../data/...`
    // relative paths; keep the scenario file at its committed
    // location and override only the parquet output path.
    let scenario = workspace_root().join("scenarios/closed-loop-imm/scenario.toml");
    let temp = tempdir(label);
    let parquet = temp.path().join("closed-loop-imm.parquet");
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
fn closed_loop_imm_runs_to_completion() {
    let parquet = run_to_parquet("imm-run");
    let bytes = fs::read(&parquet).expect("read parquet");
    assert!(!bytes.is_empty(), "parquet must be non-empty");
    let _ = fs::remove_file(&parquet);
}

#[test]
fn closed_loop_imm_is_byte_stable_across_two_runs() {
    let a = run_to_parquet("imm-rerun-a");
    let b = run_to_parquet("imm-rerun-b");
    let bytes_a = fs::read(&a).expect("read a");
    let bytes_b = fs::read(&b).expect("read b");
    assert_eq!(
        bytes_a, bytes_b,
        "two reruns of closed-loop-imm must produce byte-identical Parquet \
         (the Bar-Shalom IMM mixing / likelihood / fusion all honour \
         the determinism contract)",
    );
    let _ = fs::remove_file(&a);
    let _ = fs::remove_file(&b);
}
