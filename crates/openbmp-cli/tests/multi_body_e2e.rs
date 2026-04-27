//! Phase-3.3.D end-to-end test: multi-body scenario loads and
//! propagates through the kernel.
//!
//! Asserts the §3.3 exit criterion: a scenario with two bodies
//! (`scenarios/multi-body/two-body-fairing.toml`) loads via
//! `openbmp run`, parses the assembly tree, and produces a Parquet
//! whose initial-row mass equals the sum of body dry masses
//! (0.085 kg).

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

#[test]
fn two_body_fairing_scenario_runs_to_completion() {
    use arrow::array::Float64Array;
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    let scenario = workspace_root().join("scenarios/multi-body/two-body-fairing.toml");
    let temp = tempdir("multi-body-e2e");
    let parquet = temp.path().join("two-body-fairing.parquet");

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

    // Validate initial-row mass = sum of body dry masses (0.080 + 0.005).
    let file = fs::File::open(&parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let mass_col = schema.index_of("mass_kg").expect("mass_kg column present");

    let reader = builder.build().expect("parquet reader");
    let mut first_mass: Option<f64> = None;
    for batch in reader {
        let batch = batch.expect("read batch");
        if first_mass.is_none() && batch.num_rows() > 0 {
            let mass = batch
                .column(mass_col)
                .as_any()
                .downcast_ref::<Float64Array>()
                .expect("mass_kg is float64");
            first_mass = Some(mass.value(0));
            break;
        }
    }
    let initial_mass = first_mass.expect("at least one row");
    assert!(
        (initial_mass - 0.085).abs() < 1.0e-9,
        "initial mass should equal flat mass_kg = sum of body dry masses (0.085 kg), got {initial_mass}",
    );
}
