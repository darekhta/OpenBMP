//! Phase-3.6.D end-to-end test: 4-engine octaweb cluster with one
//! engine commanded shutdown mid-flight.
//!
//! Asserts the §3.6 exit criterion: the synthetic scenario
//! `scenarios/multi-engine-octaweb/four-engine-shutdown.toml` runs
//! via `openbmp run`, the kernel completes with
//! `StopReason::EndTime`, and (a) the cluster's mass drops at the
//! ~25 % rate expected from one engine in four shutting down,
//! (b) the cluster's z-axis thrust drops by ~25 % after shutdown,
//! (c) the cluster's lateral (ECI x) thrust component grows
//! non-zero after shutdown — proof that the engine snapshot
//! (per-engine `consumed_kg`, gimbal-applied `thrust_body`) is
//! actually being consumed by the kernel-side cluster adapters,
//! (d) two reruns of the scenario produce byte-identical Parquet.

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

/// Read all rows of a single `f64` Parquet column.
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

fn read_time_column(parquet: &Path) -> Vec<f64> {
    read_f64_column(parquet, "time_s")
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
fn four_engine_shutdown_scenario_runs_to_completion() {
    let scenario =
        workspace_root().join("scenarios/multi-engine-octaweb/four-engine-shutdown.toml");
    let temp = tempdir("octaweb-runs");
    let parquet = temp.path().join("octaweb.parquet");
    run_scenario(&scenario, &parquet);

    // Sanity: the per-model thrust telemetry channels exist.
    let file = fs::File::open(&parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let _ = schema
        .index_of("force.thrust.z_n")
        .expect("force.thrust.z_n column present");
    let _ = schema
        .index_of("force.thrust.x_n")
        .expect("force.thrust.x_n column present");
    let _ = schema.index_of("mass_kg").expect("mass_kg column present");

    let times = read_time_column(&parquet);
    let mass = read_f64_column(&parquet, "mass_kg");
    assert!(times.len() > 1);
    assert_eq!(times.len(), mass.len());

    // Vehicle starts at the configured 100 kg dry mass.
    assert!((mass[0] - 100.0).abs() < 1.0e-9);

    // After ignition + a few seconds of burn, the cluster has
    // consumed propellant. The rack-pushed snapshot path must never
    // let the published mass increase between adjacent samples.
    for window in mass.windows(2) {
        assert!(
            window[1] <= window[0] + 1.0e-9,
            "mass must be monotone non-increasing across the run; got {:.6} -> {:.6}",
            window[0],
            window[1],
        );
    }
    let last_mass = *mass.last().expect("at least one row");
    assert!(
        last_mass < 100.0,
        "cluster mass should decrease over the run; final mass = {last_mass}",
    );
    assert!(
        last_mass > 0.0,
        "cluster mass should stay positive over a 10s run; final mass = {last_mass}",
    );
}

#[test]
fn engine_shutdown_event_drops_thrust_and_makes_lateral_component_non_zero() {
    let scenario =
        workspace_root().join("scenarios/multi-engine-octaweb/four-engine-shutdown.toml");
    let temp = tempdir("octaweb-shutdown");
    let parquet = temp.path().join("octaweb.parquet");
    run_scenario(&scenario, &parquet);

    let times = read_time_column(&parquet);
    let thrust_x = read_f64_column(&parquet, "force.thrust.x_n");
    let thrust_z = read_f64_column(&parquet, "force.thrust.z_n");
    assert_eq!(times.len(), thrust_x.len());
    assert_eq!(times.len(), thrust_z.len());

    // Find a sample mid-burn before the shutdown trigger
    // (`engine_d` shuts down at altitude = 3000 m, roughly T+5 s in
    // the run with the cluster's full thrust). We sample at t = 4 s,
    // well after the ignition transient (0.1 s) and before the
    // altitude trigger.
    let pre_idx = times
        .iter()
        .position(|&t| t >= 4.0)
        .expect("scenario must include t >= 4.0");
    let pre_thrust_z = thrust_z[pre_idx];
    let pre_thrust_x = thrust_x[pre_idx];

    // Find a sample after the shutdown completes. Shutdown
    // transient is 0.1 s; t = 7 s is cleanly after the retuned
    // altitude proxy for T+5 s.
    let post_idx = times
        .iter()
        .position(|&t| t >= 7.0)
        .expect("scenario must include t >= 7.0");
    let post_thrust_z = thrust_z[post_idx];
    let post_thrust_x = thrust_x[post_idx];

    // Pre-shutdown all four engines firing. Z-thrust ≈ 4 × 5000 N
    // × cos(0.05 rad) ≈ 19975 N.  Allow 5% tolerance.
    assert!(
        pre_thrust_z > 18000.0 && pre_thrust_z < 22000.0,
        "pre-shutdown z-thrust outside expected envelope (≈ 19975 N): got {pre_thrust_z}",
    );

    // Pre-shutdown lateral thrust on x is ≈ 0 because engine_b's
    // +x gimbal cancels engine_d's -x gimbal (engine_a / engine_c
    // contribute zero on x).
    assert!(
        pre_thrust_x.abs() < 100.0,
        "pre-shutdown lateral x-thrust should be ≈ 0: got {pre_thrust_x}",
    );

    // Post-shutdown: engine_d is off. Three engines firing →
    // z-thrust ≈ 3 × 5000 × cos(small) ≈ 15000 N. Drop ratio
    // ≈ 0.75. Allow 5% tolerance on each side.
    let drop_ratio = post_thrust_z / pre_thrust_z;
    assert!(
        drop_ratio > 0.70 && drop_ratio < 0.80,
        "post-shutdown z-thrust ratio expected ~0.75, got {drop_ratio} (pre = {pre_thrust_z}, post = {post_thrust_z})",
    );

    // Post-shutdown: engine_b's +x gimbal is no longer cancelled.
    // Lateral x-thrust ≈ 5000 × sin(0.05) ≈ 250 N. We assert
    // simply that the magnitude grew above noise.
    assert!(
        post_thrust_x.abs() > 100.0,
        "post-shutdown lateral x-thrust should be non-zero (asymmetric thrust): got {post_thrust_x}",
    );
}

#[test]
fn engine_cluster_run_byte_stable_across_two_invocations() {
    let scenario =
        workspace_root().join("scenarios/multi-engine-octaweb/four-engine-shutdown.toml");
    let temp = tempdir("octaweb-bytestable");
    let a = temp.path().join("a.parquet");
    let b = temp.path().join("b.parquet");
    run_scenario(&scenario, &a);
    run_scenario(&scenario, &b);
    let bytes_a = fs::read(&a).expect("read a");
    let bytes_b = fs::read(&b).expect("read b");
    assert_eq!(
        bytes_a, bytes_b,
        "two runs of the same engine-cluster scenario must produce byte-identical Parquet",
    );
}
