//! End-to-end test for offline footprint Monte-Carlo reporting.

#![allow(clippy::expect_used, clippy::panic)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::footprint_mc;
use tempfile::Builder;

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root must exist relative to CARGO_MANIFEST_DIR")
}

fn fixture_path() -> PathBuf {
    workspace_root().join("crates/openbmp-scenario/tests/fixtures/coast-footprint-mc-valid.toml")
}

fn toml_literal_path(path: &Path) -> String {
    let path = path.to_string_lossy();
    assert!(
        !path.contains('\''),
        "test temp paths must be representable as TOML literal strings: {path}",
    );
    format!("'{path}'")
}

#[test]
fn footprint_mc_writes_declared_outputs() {
    let temp = Builder::new()
        .prefix("openbmp_footprint_mc")
        .tempdir()
        .expect("tempdir");
    let csv = temp.path().join("samples.csv");
    let parquet = temp.path().join("samples.parquet");
    let summary = temp.path().join("summary.toml");

    let original = fs::read_to_string(fixture_path()).expect("read fixture");
    let rewritten = original
        .replace(
            r#"samples_csv = "out/coast-footprint-mc-samples.csv""#,
            &format!("samples_csv = {}", toml_literal_path(&csv)),
        )
        .replace(
            r#"samples_parquet = "out/coast-footprint-mc-samples.parquet""#,
            &format!("samples_parquet = {}", toml_literal_path(&parquet)),
        )
        .replace(
            r#"summary_toml = "out/coast-footprint-mc-summary.toml""#,
            &format!("summary_toml = {}", toml_literal_path(&summary)),
        );
    let scenario = temp.path().join("scenario.toml");
    fs::write(&scenario, rewritten).expect("write staged scenario");

    let report = footprint_mc::run(&scenario).expect("footprint mc run");
    assert_eq!(report.samples_requested, 32);
    assert_eq!(report.samples_succeeded, 32);
    assert_eq!(report.samples_failed, 0);
    assert_eq!(report.written.len(), 3);
    let csv_text = fs::read_to_string(&csv).expect("read csv");
    assert!(csv_text.contains("radial_offset_from_nominal_m"));
    assert!(!csv_text.contains("miss_distance"));
    assert!(!csv_text.contains("position_x_eci_m"));
    assert!(!csv_text.contains("velocity_x_eci_m_s"));
    assert!(!csv_text.contains("ballistic_coefficient_m2_kg"));
    assert!(!csv_text.contains("wind_x_m_s"));
    assert!(parquet.metadata().expect("parquet metadata").len() > 0);
    let summary_text = fs::read_to_string(summary).expect("read summary");
    assert!(summary_text.contains("[dispersion_statistics]"));
    assert!(!summary_text.contains("[accuracy]"));
    assert!(!summary_text.contains("miss_distance"));
    assert!(summary_text.contains("radial_dispersion_p50_m"));
    assert!(summary_text.contains("[dispersion_ellipse]"));
    assert!(summary_text.contains("[[quantiles]]"));
    assert!(summary_text.contains("[[nominal_radial_offset_quantiles]]"));
}
