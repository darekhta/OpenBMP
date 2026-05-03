//! Phase-5.C.1 end-to-end test: vertical ballistic profile with
//! per-step piecewise-exponential atmosphere telemetry sampling.
//!
//! Asserts the §5.C.1 exit criterion:
//!
//! 1. The scenario runs to completion via `openbmp_cli::commands::run`
//!    with `stop_label = "end-time"` after 4100 RK4 steps.
//! 2. The per-step atmosphere telemetry channels are populated with
//!    finite positive values throughout the flight.
//! 3. The sea-level density (at t = 0, z = 0) matches the layered
//!    model's tabulated base value (1.225 kg/m³).
//! 4. The minimum sampled density (at apogee, z ≈ 204 km) lies in the
//!    documented LEO engineering envelope.
//! 5. Two reruns produce byte-identical Parquet (deterministic
//!    layered atmosphere model propagates through the runner).
//!
//! The math-side correctness of `PiecewiseExponentialAtmosphere`
//! (layer transitions, density monotonicity, validity envelope,
//! determinism) is covered by the unit tests in
//! `crates/openbmp-physics/src/atmosphere/piecewise_exponential.rs`.

#![allow(clippy::expect_used, clippy::float_cmp, clippy::panic)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::run;
use tempfile::{Builder, TempDir};

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn workspace_root() -> PathBuf {
    manifest_dir()
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root")
}

fn scenario_path() -> PathBuf {
    workspace_root().join("scenarios/sounding-piecewise-exp-atmosphere/scenario.toml")
}

fn toml_literal_path(path: &Path) -> String {
    let path = path.to_string_lossy();
    assert!(
        !path.contains('\''),
        "test temp paths must be representable as TOML literal strings: {path}",
    );
    format!("'{path}'")
}

fn stage_scenario(temp_dir: &Path, label: &str) -> PathBuf {
    let original = fs::read_to_string(scenario_path()).expect("read canonical scenario");
    let parquet = temp_dir.join(format!("{label}.parquet"));
    let rewritten = original.replace(
        "output.parquet = \"out/sounding-piecewise-exp-atmosphere.parquet\"",
        &format!("output.parquet = {}", toml_literal_path(&parquet)),
    );
    let staged = temp_dir.join(format!("{label}-scenario.toml"));
    fs::write(&staged, rewritten).expect("write staged scenario");
    staged
}

fn tempdir_for(label: &str) -> TempDir {
    Builder::new()
        .prefix(&format!("{label}-"))
        .tempdir()
        .expect("tempdir")
}

fn read_f64_column(parquet: &Path, column_name: &str) -> Vec<f64> {
    use arrow::array::Float64Array;
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

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

#[test]
fn sounding_piecewise_exp_atmosphere_runs_with_populated_atmosphere_channels() {
    let temp = tempdir_for("openbmp_piecewise_exp_run");
    let staged = stage_scenario(temp.path(), "run");
    let report = run::run(&staged).expect("scenario run must succeed");

    assert_eq!(report.stop_label, "end-time");
    assert_eq!(report.final_step, 4100);

    let parquet_path = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("parquet"))
        .expect("parquet output path must be in report");

    let densities = read_f64_column(parquet_path, "atmosphere.density_kg_m3");
    let pressures = read_f64_column(parquet_path, "atmosphere.pressure_pa");
    let temperatures = read_f64_column(parquet_path, "atmosphere.temperature_k");
    let speeds = read_f64_column(parquet_path, "atmosphere.speed_of_sound_m_s");

    assert!(
        !densities.is_empty(),
        "atmosphere channels must be populated"
    );
    assert_eq!(densities.len(), pressures.len());
    assert_eq!(densities.len(), temperatures.len());
    assert_eq!(densities.len(), speeds.len());

    // All samples must be finite and positive.
    for (i, &rho) in densities.iter().enumerate() {
        assert!(
            rho.is_finite() && rho > 0.0,
            "density at step {i} = {rho} not positive-finite",
        );
    }
    for (i, &p) in pressures.iter().enumerate() {
        assert!(
            p.is_finite() && p > 0.0,
            "pressure at step {i} = {p} not positive-finite",
        );
    }
    for (i, &t) in temperatures.iter().enumerate() {
        assert!(
            t.is_finite() && t > 0.0,
            "temperature at step {i} = {t} not positive-finite",
        );
    }
    for (i, &s) in speeds.iter().enumerate() {
        assert!(
            s.is_finite() && s > 0.0,
            "speed of sound at step {i} = {s} not positive-finite",
        );
    }

    // Sea-level (t = 0) density should match the layered model's
    // tabulated base value 1.225 kg/m³.
    let sea_level_rho = densities[0];
    assert!(
        (sea_level_rho - 1.225).abs() < 1.0e-9,
        "sea-level density at t=0 was {sea_level_rho}, expected 1.225 kg/m³",
    );

    // Minimum density across the flight must land somewhere near the
    // documented engineering envelope for the 200-250 km altitude
    // band (Vallado Table 8-4: ρ(200 km) ≈ 2.5e-10, ρ(250 km) ≈ 6e-11).
    // The flight reaches apogee at z ≈ 204 km, so accept any value
    // below 1e-9 kg/m³.
    let min_density = densities.iter().copied().fold(f64::INFINITY, f64::min);
    assert!(
        min_density < 1.0e-9,
        "minimum density across flight = {min_density:.3e} kg/m³ \
         did not reach the upper-atmosphere regime (expected < 1e-9 \
         at the ≈204 km apogee)",
    );
}

#[test]
fn sounding_piecewise_exp_atmosphere_is_byte_stable_across_two_runs() {
    let temp = tempdir_for("openbmp_piecewise_exp_bytestable");

    let staged_a = stage_scenario(temp.path(), "run_a");
    let staged_b = stage_scenario(temp.path(), "run_b");

    let report_a = run::run(&staged_a).expect("run a");
    let report_b = run::run(&staged_b).expect("run b");

    assert_eq!(report_a.final_step, report_b.final_step);
    assert_eq!(
        report_a.final_time_s.to_bits(),
        report_b.final_time_s.to_bits()
    );

    let parquet_a = report_a
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("parquet"))
        .expect("parquet output a");
    let parquet_b = report_b
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("parquet"))
        .expect("parquet output b");
    let bytes_a = fs::read(parquet_a).expect("read a");
    let bytes_b = fs::read(parquet_b).expect("read b");

    assert_eq!(
        bytes_a, bytes_b,
        "two reruns of sounding-piecewise-exp-atmosphere must produce \
         byte-identical Parquet (the piecewise-exponential model is \
         deterministic with locked operand order)",
    );
}
