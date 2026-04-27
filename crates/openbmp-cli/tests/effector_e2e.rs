//! Phase-3.4.D end-to-end test: single-elevon effector scenario
//! loads, propagates, and emits the effector deflection telemetry
//! channel.
//!
//! Asserts the §3.4 exit criterion: the synthetic single-elevon
//! scenario `scenarios/effector-elevon/single-elevon-elevator-step.toml`
//! runs via `openbmp run`, the kernel completes with
//! `StopReason::EndTime`, the Parquet contains the
//! `effector.delta_e.actual` column, and the recorded actual
//! deflection (a) starts at 0 (the initial position), (b) is still 0
//! immediately before the commanded step time + latency, and
//! (c) tracks toward the commanded value (0.087 rad ≈ 5°) after
//! the step + latency window.

#![allow(clippy::expect_used, clippy::float_cmp, clippy::panic)]

use std::fs;
use std::path::{Path, PathBuf};

use arrow::array::Float64Array;
use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
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

/// Read all rows of a single `f64` Parquet column into a `Vec<f64>`.
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

/// Read all rows of the kernel's `time_s` Parquet column.
fn read_time_column(parquet: &Path) -> Vec<f64> {
    read_f64_column(parquet, "time_s")
}

#[test]
fn single_elevon_scenario_runs_to_completion() {
    let scenario =
        workspace_root().join("scenarios/effector-elevon/single-elevon-elevator-step.toml");
    let temp = tempdir("effector-elevon-e2e");
    let parquet = temp.path().join("single-elevon-elevator-step.parquet");

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
    assert!(parquet.exists(), "parquet file should have been written");

    // Schema sanity: the effector channel must exist.
    let file = fs::File::open(&parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let _ = schema
        .index_of("effector.delta_e.actual")
        .expect("effector.delta_e.actual column present");

    let times = read_time_column(&parquet);
    let actuals = read_f64_column(&parquet, "effector.delta_e.actual");
    assert_eq!(
        times.len(),
        actuals.len(),
        "time and effector columns must have matching lengths"
    );
    assert!(times.len() > 1, "scenario must produce more than one row");

    // Step 0: actual deflection equals the initial position (0.0).
    assert!(
        actuals[0].abs() < 1.0e-12,
        "step-0 effector deflection should be 0 rad, got {}",
        actuals[0]
    );

    // The scenario commands a step at t = 0.5 s with a 0.020 s pure
    // delay. Just before the latency window expires, the actual must
    // still be 0 (the latency buffer hasn't released the new command
    // yet). We sample at t = 0.515 s (well inside [0.5, 0.520]).
    let pre_latency_idx = times
        .iter()
        .position(|&t| t >= 0.515)
        .expect("scenario must include t >= 0.515");
    assert!(
        actuals[pre_latency_idx].abs() < 1.0e-9,
        "before latency expiry (t = {}), actual should still be 0, got {}",
        times[pre_latency_idx],
        actuals[pre_latency_idx]
    );

    // After the step + latency + a few first-order time constants
    // (tau = 0.05 s), the actual must have moved from 0 toward 0.087.
    // We assert positivity at t = 0.6 s (4 * tau past the latency
    // window) — strict tracking error envelopes are owned by the
    // unit-level tests on `LinearActuator`.
    let post_idx = times
        .iter()
        .position(|&t| t >= 0.6)
        .expect("scenario must include t >= 0.6");
    assert!(
        actuals[post_idx] > 0.0,
        "after step + latency, actual should have moved positive at t = {}, got {}",
        times[post_idx],
        actuals[post_idx]
    );
    assert!(
        actuals[post_idx] <= 0.087 + 1.0e-9,
        "actual must not exceed the commanded value, got {}",
        actuals[post_idx]
    );

    // By the end of the run (2 s, with command issued at 0.5 s, so
    // ~30 first-order time constants after settling), the actual
    // must be within 1e-6 rad of the commanded 0.087 rad.
    let final_actual = *actuals.last().expect("at least one row");
    assert!(
        (final_actual - 0.087).abs() < 1.0e-6,
        "final actual deflection should be within 1e-6 rad of the commanded 0.087 rad, got {final_actual}",
    );
}
