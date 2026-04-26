//! Phase-1.8 end-to-end golden test for the constant-acceleration drop.
//!
//! Runs the canonical scenario through `openbmp_cli::commands::run::run`
//! twice and asserts:
//!
//! 1. **Tolerance compliance** of the final time, position, and velocity
//!    against the committed `expected/constant-acceleration-drop.toml`.
//! 2. **Same-machine byte stability** of the Parquet archive across two
//!    independent runs (Phase-1.8 byte-stable claim; Phase-1.9 promotes
//!    this to a committed cross-machine reference golden).
//!
//! The CSV and Parquet outputs are written under a per-test temp dir
//! and never committed.

#![allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::similar_names
)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::run;
use openbmp_testkit::tolerance::ToleranceTable;
use tempfile::{Builder, TempDir};

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn workspace_root() -> PathBuf {
    // crates/openbmp-cli -> workspace
    manifest_dir()
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root must exist relative to CARGO_MANIFEST_DIR")
}

fn scenario_path() -> PathBuf {
    workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml")
}

fn tolerance_table_path() -> PathBuf {
    manifest_dir().join("tests/expected/constant-acceleration-drop.toml")
}

/// Copy the canonical scenario into a temp dir and rewrite the
/// telemetry output paths to point under that temp dir, so test runs
/// don't litter `out/` next to the committed scenario.
fn stage_scenario(temp_dir: &Path, label: &str) -> PathBuf {
    let original = fs::read_to_string(scenario_path()).expect("read canonical scenario");
    let parquet = temp_dir.join(format!("{label}.parquet"));
    let csv = temp_dir.join(format!("{label}.csv"));
    let rewritten = original
        .replace(
            "output.csv     = \"out/constant-acceleration-drop.csv\"",
            &format!("output.csv     = \"{}\"", csv.display()),
        )
        .replace(
            "output.parquet = \"out/constant-acceleration-drop.parquet\"",
            &format!("output.parquet = \"{}\"", parquet.display()),
        );
    let staged = temp_dir.join(format!("{label}-scenario.toml"));
    fs::write(&staged, rewritten).expect("write staged scenario");
    staged
}

#[test]
fn constant_acceleration_drop_meets_tolerance_table() {
    let temp = tempdir_for("openbmp_e2e_tolerance");
    let staged = stage_scenario(temp.path(), "tolerance");

    let report = run::run(&staged).expect("scenario run must succeed");

    assert_eq!(report.stop_label, "end-time");
    assert_eq!(report.final_step, 1000);

    // The final state values are recovered from the in-memory table by
    // re-loading the freshly-written Parquet — easier than threading
    // them out of `RunReport`, and exercises the writer.
    let parquet_path = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("parquet"))
        .expect("parquet output path must be in report");

    let final_state = read_final_state(parquet_path);

    let table =
        ToleranceTable::from_path(tolerance_table_path()).expect("tolerance table must parse");
    table
        .check_metric("final_time_s", report.final_time_s)
        .expect("final_time_s within tolerance");
    table
        .check_metric("final_position_z_m", final_state.position_z_m)
        .expect("final_position_z_m within tolerance");
    table
        .check_metric("final_velocity_z_m_s", final_state.velocity_z_m_s)
        .expect("final_velocity_z_m_s within tolerance");
}

#[test]
fn constant_acceleration_drop_is_byte_stable_across_two_runs() {
    let temp = tempdir_for("openbmp_e2e_bytestable");

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
        "Parquet bytes must be identical across runs"
    );
}

#[test]
fn output_overrides_replace_declared_paths_and_add_missing_kinds() {
    let temp = tempdir_for("openbmp_e2e_output_overrides");
    let override_dir = temp.path().join("override");

    let staged = stage_scenario(temp.path(), "declared");
    let declared_csv = temp.path().join("declared.csv");
    let declared_parquet = temp.path().join("declared.parquet");
    let override_csv = override_dir.join("drop.csv");
    let override_json = override_dir.join("drop.json");
    let override_parquet = override_dir.join("drop.parquet");

    let overrides = run::OutputOverrides::new(
        Some(override_csv.clone()),
        Some(override_json.clone()),
        Some(override_parquet.clone()),
    );
    let report = run::run_with_overrides(&staged, &overrides).expect("run with overrides");

    assert!(override_csv.exists(), "CSV override should be written");
    assert!(override_json.exists(), "JSON override should be written");
    assert!(
        override_parquet.exists(),
        "Parquet override should be written"
    );
    assert!(
        !declared_csv.exists(),
        "declared CSV path should not be written when overridden"
    );
    assert!(
        !declared_parquet.exists(),
        "declared Parquet path should not be written when overridden"
    );
    assert_eq!(
        report.written,
        vec![override_csv, override_json, override_parquet]
    );
}

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

#[derive(Debug)]
struct FinalState {
    position_z_m: f64,
    velocity_z_m_s: f64,
}

fn read_final_state(parquet_path: &Path) -> FinalState {
    use arrow::array::Float64Array;
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    let file = fs::File::open(parquet_path).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let position_z_col = schema
        .index_of("position_z_m")
        .expect("position_z_m column present");
    let velocity_z_col = schema
        .index_of("velocity_z_m_s")
        .expect("velocity_z_m_s column present");

    let reader = builder.build().expect("parquet reader");
    let mut last_pz = f64::NAN;
    let mut last_vz = f64::NAN;
    for batch in reader {
        let batch = batch.expect("read batch");
        let position_z = batch
            .column(position_z_col)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("position_z is float64");
        let velocity_z = batch
            .column(velocity_z_col)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("velocity_z is float64");
        if batch.num_rows() > 0 {
            last_pz = position_z.value(batch.num_rows() - 1);
            last_vz = velocity_z.value(batch.num_rows() - 1);
        }
    }
    FinalState {
        position_z_m: last_pz,
        velocity_z_m_s: last_vz,
    }
}

fn tempdir_for(label: &str) -> TempDir {
    Builder::new()
        .prefix(&format!("{label}-"))
        .tempdir()
        .expect("tempdir must construct")
}
