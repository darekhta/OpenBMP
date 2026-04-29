//! Phase-3.9.E end-to-end test: `openbmp run` on the canonical
//! parachute-descent scenario.
//!
//! Asserts:
//! - `openbmp run scenarios/parachute-recovery/parachute-descent.toml`
//!   succeeds and writes a Parquet file.
//! - The vehicle reaches a finite low altitude by the end of the
//!   simulation (descended past the main-deploy threshold of 300 m
//!   and continued descending under main canopy drag).
//! - Two reruns of the same scenario produce byte-identical Parquet
//!   output (Phase-3.9 determinism gate for the full recovery hot
//!   path: scenario parse → `RecoveryRack` build → mission-event
//!   firing → kernel snapshot push → `RecoveryRackForceAdapter`
//!   evaluation in the kernel's RK4 stages).

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

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

fn read_position_z_column(parquet_path: &Path) -> Vec<f64> {
    let file = fs::File::open(parquet_path).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet reader builder");
    let reader = builder.build().expect("parquet reader build");
    let mut out = Vec::new();
    for batch in reader {
        let batch = batch.expect("batch read");
        let z_index = batch
            .schema()
            .index_of("position_z_m")
            .expect("position_z_m column");
        let z_array = batch
            .column(z_index)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("position_z_m as f64");
        for i in 0..z_array.len() {
            out.push(z_array.value(i));
        }
    }
    out
}

#[test]
fn parachute_descent_reaches_low_altitude_under_drag() {
    let scenario = workspace_root().join("scenarios/parachute-recovery/parachute-descent.toml");
    let temp = tempdir("parachute-altitude");
    let parquet = temp.path().join("parachute.parquet");

    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(&scenario)
        .arg("--output-parquet")
        .arg(&parquet);
    cmd.assert().success();

    let z = read_position_z_column(&parquet);
    assert!(!z.is_empty(), "parquet produced no rows");

    // The vehicle starts at z = 1000 m, climbs to ≈ 1127 m, then
    // descends. Drogue opens at apogee, main opens at z = 300 m.
    // After 120 s under main-chute drag the vehicle must be well
    // below the main-deploy altitude — assert z_final < 250 m to
    // give the test margin against integrator noise. (Without
    // recovery the vehicle would crash to z ≪ 0 long before t = 120 s
    // under free fall; recovery is the only thing keeping it
    // positive.)
    let final_z = *z.last().expect("at least one row");
    assert!(
        final_z.is_finite(),
        "final altitude must be finite, got {final_z}"
    );
    assert!(
        final_z < 250.0,
        "final altitude must be < 250 m, got {final_z} m"
    );
    assert!(
        final_z > 0.0,
        "final altitude must stay positive (recovery decelerates), got {final_z} m"
    );

    // Apogee must be > 1100 m: 50 m/s upward initial velocity adds
    // ≈ 127 m of climb under 9.80665 m/s² gravity. Apogee proves the
    // ascent phase ran with no recovery drag (devices stay stowed
    // until the apogee event fires).
    let apogee = z.iter().copied().fold(f64::MIN, f64::max);
    assert!(apogee > 1100.0, "apogee must be > 1100 m, got {apogee} m");
}

#[test]
fn parachute_descent_byte_stable_across_two_runs() {
    let scenario = workspace_root().join("scenarios/parachute-recovery/parachute-descent.toml");
    let temp = tempdir("parachute-byte-stable");
    let parquet_a = temp.path().join("parachute-a.parquet");
    let parquet_b = temp.path().join("parachute-b.parquet");

    for parquet in [&parquet_a, &parquet_b] {
        let mut cmd = openbmp();
        cmd.arg("run")
            .arg(&scenario)
            .arg("--output-parquet")
            .arg(parquet);
        cmd.assert().success();
    }

    let bytes_a = fs::read(&parquet_a).expect("read parquet a");
    let bytes_b = fs::read(&parquet_b).expect("read parquet b");
    assert_eq!(
        bytes_a, bytes_b,
        "parachute scenario changed Parquet bytes across reruns",
    );
}
