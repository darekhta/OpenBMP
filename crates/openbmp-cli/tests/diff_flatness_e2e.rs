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

use arrow::array::Float64Array;
use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use openbmp_testkit::tolerance::ToleranceTable;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use tempfile::{Builder, TempDir};

struct RunOutput {
    parquet: PathBuf,
    kernel_steps: u32,
}

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

fn tolerance_table_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/expected/diff-flatness-figure-eight.toml")
}

fn tempdir(label: &str) -> TempDir {
    Builder::new()
        .prefix(&format!("openbmp-{label}-"))
        .tempdir()
        .expect("tempdir")
}

fn parse_kernel_steps(stdout: &str) -> u32 {
    let mut previous: Option<&str> = None;
    for token in stdout.split_whitespace() {
        if token.starts_with("steps") {
            return previous
                .expect("step count before steps token")
                .parse()
                .expect("kernel step count");
        }
        previous = Some(token);
    }
    panic!("stdout did not contain kernel step count: {stdout}");
}

fn run_to_parquet(label: &str) -> RunOutput {
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
    let kernel_steps = parse_kernel_steps(&stdout);
    assert!(parquet.exists(), "parquet must be written");
    // Copy out of the tempdir so the caller can compare across runs;
    // tempdir drops on return.
    let copy = std::env::temp_dir().join(format!("openbmp-{label}.parquet"));
    let _ = fs::remove_file(&copy);
    fs::copy(&parquet, &copy).expect("copy parquet");
    RunOutput {
        parquet: copy,
        kernel_steps,
    }
}

/// Read a single `f64` Parquet column into a `Vec<f64>`.
fn read_f64_column(parquet: &Path, column_name: &str) -> Vec<f64> {
    let file = fs::File::open(parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let col = schema
        .index_of(column_name)
        .unwrap_or_else(|_| panic!("{column_name} column present in parquet schema"));
    let reader = builder.build().expect("parquet reader");
    let mut out: Vec<f64> = Vec::new();
    for batch in reader {
        let batch = batch.expect("read batch");
        let arr = batch
            .column(col)
            .as_any()
            .downcast_ref::<Float64Array>()
            .unwrap_or_else(|| panic!("{column_name} is float64"));
        for i in 0..arr.len() {
            out.push(arr.value(i));
        }
    }
    out
}

/// Body-frame angular-velocity magnitude `|ω|` per row, derived from
/// the rigid-body kernel's truth-state telemetry. Phase-5.A.2.A asserts
/// the autopilot keeps this bounded.
fn max_angular_velocity_magnitude_rad_s(parquet: &Path) -> f64 {
    let wx = read_f64_column(parquet, "angular_velocity.x_rad_s");
    let wy = read_f64_column(parquet, "angular_velocity.y_rad_s");
    let wz = read_f64_column(parquet, "angular_velocity.z_rad_s");
    let mut max = 0.0_f64;
    for ((x, y), z) in wx.iter().zip(wy.iter()).zip(wz.iter()) {
        let magnitude = (x * x + y * y + z * z).sqrt();
        if magnitude > max {
            max = magnitude;
        }
    }
    max
}

#[test]
fn diff_flatness_figure_eight_runs_to_completion() {
    let run = run_to_parquet("diff-flatness-figure-eight-runs");
    let table = ToleranceTable::from_path(tolerance_table_path()).expect("tolerance table");
    table
        .check_metric("kernel_steps", f64::from(run.kernel_steps))
        .expect("kernel_steps within tolerance");
    let max_omega = max_angular_velocity_magnitude_rad_s(&run.parquet);
    table
        .check_metric("max_angular_velocity_magnitude_rad_s", max_omega)
        .unwrap_or_else(|err| {
            panic!("max_angular_velocity_magnitude_rad_s = {max_omega} outside tolerance: {err}")
        });
    let bytes = fs::read(&run.parquet).expect("read parquet");
    assert!(!bytes.is_empty(), "parquet must be non-empty");
    let _ = fs::remove_file(&run.parquet);
}

#[test]
fn diff_flatness_figure_eight_is_byte_stable_across_reruns() {
    let a = run_to_parquet("diff-flatness-figure-eight-rerun-a");
    let b = run_to_parquet("diff-flatness-figure-eight-rerun-b");
    let bytes_a = fs::read(&a.parquet).expect("read a");
    let bytes_b = fs::read(&b.parquet).expect("read b");
    assert_eq!(
        bytes_a, bytes_b,
        "two reruns of diff-flatness-figure-eight must produce byte-identical Parquet \
         (polynomial coefficients are bit-stable; closed-loop pipeline preserves that)"
    );
    let _ = fs::remove_file(&a.parquet);
    let _ = fs::remove_file(&b.parquet);
}
