//! End-to-end tests for the external telemetry comparison harness.
//!
//! The fixtures are synthetic analytic-drop observations generated in
//! the test temp directory. No real or recovered flight telemetry is
//! committed here.

#![allow(clippy::expect_used, clippy::panic)]

use std::fs;
use std::path::PathBuf;

use openbmp_cli::commands::compare_telemetry;
use tempfile::Builder;

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(std::path::Path::parent)
        .map(std::path::Path::to_path_buf)
        .expect("workspace root")
}

fn scenario_path() -> PathBuf {
    workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml")
}

#[test]
fn compare_telemetry_accepts_external_reference_csv() {
    let temp = Builder::new()
        .prefix("openbmp_compare_telemetry")
        .tempdir()
        .expect("tempdir");
    let csv = temp.path().join("external-reference.csv");
    let mapping = temp.path().join("mapping.toml");

    fs::write(
        &csv,
        "time_s,altitude_m,speed_m_s\n\
         0.0,1000.0,0.0\n\
         5.0,877.416875,49.03325\n\
         10.0,509.6675,98.0665\n",
    )
    .expect("write reference csv");
    fs::write(
        &mapping,
        r#"
[reference]
time_column = "time_s"

[comparison]
time_tolerance_s = 0.02

[[metrics]]
id = "altitude"
reference_column = "altitude_m"
tolerance_abs = 1e-9
actual = { kind = "altitude_from_position", radius_m = 0.0 }

[[metrics]]
id = "speed"
reference_column = "speed_m_s"
tolerance_abs = 1e-9
actual = { kind = "speed_from_velocity" }

[[metrics]]
id = "surface_relative_speed"
reference_column = "speed_m_s"
tolerance_abs = 1e-9
actual = { kind = "surface_relative_speed", omega_rad_s = 0.0 }
"#,
    )
    .expect("write mapping");

    let report = compare_telemetry::run(&scenario_path(), &csv, &mapping).expect("compare");
    assert!(report.passed(), "{report:#?}");
    assert_eq!(report.reference_rows, 3);
    assert_eq!(report.metrics.len(), 3);
    assert!(report.metrics.iter().all(|metric| metric.exceedances == 0));
}

#[test]
fn compare_telemetry_reports_envelope_exceedances() {
    let temp = Builder::new()
        .prefix("openbmp_compare_telemetry_fail")
        .tempdir()
        .expect("tempdir");
    let csv = temp.path().join("external-reference.csv");
    let mapping = temp.path().join("mapping.toml");

    fs::write(
        &csv,
        "time_s,altitude_m\n\
         5.0,1000.0\n",
    )
    .expect("write reference csv");
    fs::write(
        &mapping,
        r#"
[reference]
time_column = "time_s"

[[metrics]]
id = "altitude"
reference_column = "altitude_m"
tolerance_abs = 1.0
actual = { kind = "altitude_from_position", radius_m = 0.0 }
"#,
    )
    .expect("write mapping");

    let report = compare_telemetry::run(&scenario_path(), &csv, &mapping).expect("compare");
    assert!(!report.passed(), "{report:#?}");
    assert_eq!(report.metrics[0].compared_samples, 1);
    assert_eq!(report.metrics[0].exceedances, 1);
    assert!(report.failure_summary().contains("exceeded tolerance"));
}
