//! Phase-5.A.1.C end-to-end test: minimum-snap differential-flatness
//! figure-eight scenario loads, propagates, and emits byte-stable
//! Parquet output across reruns.
//!
//! Asserts the §5.A.1 exit criterion (parser + plumbing scope of
//! 5.A.1.C): `scenarios/diff-flatness-figure-eight/scenario.toml`
//! runs via `openbmp run`, the kernel completes with end-time stop
//! reason, the Parquet file is non-empty, and two consecutive runs
//! produce a byte-identical Parquet payload (the polynomial-
//! coefficient bit-stability proven by the trajectory unit tests
//! propagates through the closed-loop pipeline).
//!
//! Closed-loop attitude tracking tolerance is **not** asserted here;
//! that envelope tightens with the L1 adaptive (Phase 5.A.2) and
//! observer-form anti-windup (Phase 5.A.3) sub-phases.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

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
    let scenario = workspace_root().join("scenarios/diff-flatness-figure-eight/scenario.toml");
    let temp = tempdir(label);
    let parquet = temp.path().join("diff-flatness-figure-eight.parquet");
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
    // Copy out of the tempdir so the caller can compare across runs;
    // tempdir drops on return.
    let copy = std::env::temp_dir().join(format!("openbmp-{label}.parquet"));
    let _ = fs::remove_file(&copy);
    fs::copy(&parquet, &copy).expect("copy parquet");
    copy
}

#[test]
fn diff_flatness_figure_eight_runs_to_completion() {
    let parquet = run_to_parquet("diff-flatness-figure-eight-runs");
    let bytes = fs::read(&parquet).expect("read parquet");
    assert!(!bytes.is_empty(), "parquet must be non-empty");
    let _ = fs::remove_file(&parquet);
}

#[test]
fn diff_flatness_figure_eight_is_byte_stable_across_reruns() {
    let a = run_to_parquet("diff-flatness-figure-eight-rerun-a");
    let b = run_to_parquet("diff-flatness-figure-eight-rerun-b");
    let bytes_a = fs::read(&a).expect("read a");
    let bytes_b = fs::read(&b).expect("read b");
    assert_eq!(
        bytes_a, bytes_b,
        "two reruns of diff-flatness-figure-eight must produce byte-identical Parquet \
         (polynomial coefficients are bit-stable; closed-loop pipeline preserves that)"
    );
    let _ = fs::remove_file(&a);
    let _ = fs::remove_file(&b);
}
