//! Phase-3.7.E end-to-end test: rigid-body vehicle with one
//! axial liquid engine and a cylindrical tank carrying an
//! `EquivalentPendulum` slosh model.
//!
//! Asserts the §3.7 exit criterion: the synthetic scenario
//! `scenarios/sloshing-tank/sloshing-tank.toml` runs via
//! `openbmp run`, the kernel completes with `StopReason::EndTime`,
//! the vehicle altitude at end of run is positive and finite (so
//! the engine fired and the tank-rack force adapter didn't
//! corrupt the rigid-body force evaluation), and two reruns of
//! the same scenario produce byte-identical Parquet (replay
//! determinism with the tank-rack hot path engaged).

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

fn run_scenario(scenario_path: &Path, parquet_path: &Path) {
    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(scenario_path)
        .arg("--output-parquet")
        .arg(parquet_path);
    let assert = cmd.assert().success();
    let stdout = String::from_utf8_lossy(&assert.get_output().stdout).to_string();
    assert!(
        stdout.starts_with("openbmp run: ok"),
        "stdout was: {stdout}"
    );
    assert!(
        parquet_path.exists(),
        "parquet file should have been written"
    );
}

#[test]
fn sloshing_tank_scenario_runs_to_completion() {
    let scenario = workspace_root().join("scenarios/sloshing-tank/sloshing-tank.toml");
    let temp = tempdir("sloshing-tank-runs");
    let parquet = temp.path().join("sloshing-tank.parquet");
    run_scenario(&scenario, &parquet);

    // The rigid-body kernel writes position and velocity in ECI;
    // confirm the eci columns exist and the trajectory is finite.
    let pos_z = read_f64_column(&parquet, "position_z_m");
    assert!(!pos_z.is_empty(), "position_z_m must be present");
    let final_z = *pos_z.last().expect("non-empty");
    assert!(
        final_z.is_finite(),
        "final position_z_m must be finite, got {final_z}"
    );
    // Initial state has position 0 and velocity +100 m/s; over 5s
    // with thrust > weight, the vehicle should be well above 500 m
    // (purely ballistic minimum is 500 m).
    assert!(
        final_z > 500.0,
        "expected final altitude > 500 m, got {final_z}"
    );
}

#[test]
fn sloshing_tank_byte_stable_across_two_runs() {
    let scenario = workspace_root().join("scenarios/sloshing-tank/sloshing-tank.toml");
    let temp_a = tempdir("sloshing-tank-a");
    let temp_b = tempdir("sloshing-tank-b");
    let parquet_a = temp_a.path().join("sloshing-tank-a.parquet");
    let parquet_b = temp_b.path().join("sloshing-tank-b.parquet");
    run_scenario(&scenario, &parquet_a);
    run_scenario(&scenario, &parquet_b);

    let bytes_a = fs::read(&parquet_a).expect("read a");
    let bytes_b = fs::read(&parquet_b).expect("read b");
    assert_eq!(
        bytes_a.len(),
        bytes_b.len(),
        "Parquet sizes differ: {} vs {}",
        bytes_a.len(),
        bytes_b.len()
    );
    assert!(
        bytes_a == bytes_b,
        "Parquet bytes differ across two runs of the same sloshing-tank scenario; tank-rack determinism gate failed"
    );
}
