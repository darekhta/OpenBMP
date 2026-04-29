//! Phase-3.11.E end-to-end test: `openbmp run` on the RocketPy
//! Calisto cross-tool validation scenario.
//!
//! Asserts:
//!  - `openbmp run scenarios/sounding-rocket/calisto/rocketpy-calisto.toml`
//!    succeeds and writes a Parquet file.
//!  - The vehicle reaches an apogee within the Phase-3.11 risk-register
//!    envelope around RocketPy's published 3 349 m AGL.
//!    The Phase-3 plan documents two envelopes: a 1 % stretch goal
//!    (33.5 m) and a 5 % fallback that absorbs the integrator-mismatch
//!    between RocketPy's LSODA adaptive step and OpenBMP's RK4
//!    fixed-step kernel. We pin the 5 % envelope here; tightening to
//!    1 % is a follow-up once the Phase-3.11 closure report
//!    characterises the residual mismatch.
//!  - Two reruns of the same scenario produce byte-identical Parquet
//!    output (Phase-3 determinism gate end-to-end across the full
//!    rigid-body hot path: scenario parse → motor + drag deck load →
//!    aero + thrust force adapters → `DrogueMainRecovery` state machine
//!    → RK4 integrator → Parquet sink).

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use std::fs;
use std::path::{Path, PathBuf};

use arrow::array::Float64Array;
use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use tempfile::{Builder, TempDir};

/// RocketPy's published Calisto apogee, AGL, in metres.
/// Source: RocketPy Calisto example (Souza et al. 2022).
const ROCKETPY_APOGEE_AGL_M: f64 = 3349.0;

/// Phase-3.11 risk-register fallback envelope: ±5 % of the
/// RocketPy reference apogee. Documented in `docs/phase-3-plan.md
/// §3.11` as the integrator-mismatch tolerance.
const APOGEE_TOLERANCE_FRAC: f64 = 0.05;

/// The Calisto scenario offsets initial altitude by Spaceport
/// America's geodetic elevation (1 400 m) so the USSA76 atmosphere
/// sees the right altitude profile. Apogee AGL = max(z) − offset.
const SPACEPORT_ALTITUDE_OFFSET_M: f64 = 1400.0;

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
fn calisto_apogee_within_rocketpy_envelope() {
    let scenario = workspace_root().join("scenarios/sounding-rocket/calisto/rocketpy-calisto.toml");
    let temp = tempdir("calisto-apogee");
    let parquet = temp.path().join("calisto.parquet");

    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(&scenario)
        .arg("--output-parquet")
        .arg(&parquet);
    cmd.assert().success();

    let z = read_position_z_column(&parquet);
    assert!(!z.is_empty(), "parquet produced no rows");

    let apogee_scenario_m = z.iter().copied().fold(f64::MIN, f64::max);
    assert!(
        apogee_scenario_m.is_finite(),
        "apogee must be finite, got {apogee_scenario_m}"
    );

    let apogee_agl_m = apogee_scenario_m - SPACEPORT_ALTITUDE_OFFSET_M;
    let envelope_m = ROCKETPY_APOGEE_AGL_M * APOGEE_TOLERANCE_FRAC;
    let lower = ROCKETPY_APOGEE_AGL_M - envelope_m;
    let upper = ROCKETPY_APOGEE_AGL_M + envelope_m;

    assert!(
        apogee_agl_m >= lower && apogee_agl_m <= upper,
        "Calisto apogee AGL {apogee_agl_m:.3} m outside {APOGEE_TOLERANCE_FRAC} envelope around \
         RocketPy reference {ROCKETPY_APOGEE_AGL_M} m (allowed [{lower:.3}, {upper:.3}] m)",
    );
}

#[test]
fn calisto_byte_stable_across_two_runs() {
    let scenario = workspace_root().join("scenarios/sounding-rocket/calisto/rocketpy-calisto.toml");
    let temp = tempdir("calisto-byte-stable");
    let parquet_a = temp.path().join("calisto-a.parquet");
    let parquet_b = temp.path().join("calisto-b.parquet");

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
        "Calisto scenario changed Parquet bytes across reruns",
    );
}
