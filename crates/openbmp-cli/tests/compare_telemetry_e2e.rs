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
        "time_s,altitude_m,speed_m_s,vertical_velocity_m_s,downrange_velocity_m_s\n\
         0.0,1000.0,0.0,0.0,0.0\n\
         5.0,877.416875,49.03325,-49.03325,0.0\n\
         10.0,509.6675,98.0665,-98.0665,0.0\n",
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

[[metrics]]
id = "surface_relative_radial_velocity"
reference_column = "vertical_velocity_m_s"
tolerance_abs = 1e-9
actual = { kind = "surface_relative_radial_velocity", omega_rad_s = 0.0 }

[[metrics]]
id = "surface_relative_axis_velocity"
reference_column = "downrange_velocity_m_s"
tolerance_abs = 1e-9
actual = { kind = "surface_relative_axis_velocity", axis_eci = [0.0, 1.0, 0.0], omega_rad_s = 0.0 }

[[metrics]]
id = "surface_relative_local_axis_velocity"
reference_column = "downrange_velocity_m_s"
tolerance_abs = 1e-9
actual = { kind = "surface_relative_local_axis_velocity", axis_eci = [0.0, 1.0, 0.0], omega_rad_s = 0.0 }

[[metrics]]
id = "surface_relative_horizontal_speed"
reference_column = "downrange_velocity_m_s"
tolerance_abs = 1e-9
actual = { kind = "surface_relative_horizontal_speed", omega_rad_s = 0.0 }
"#,
    )
    .expect("write mapping");

    let report = compare_telemetry::run(&scenario_path(), &csv, &mapping).expect("compare");
    assert!(report.passed(), "{report:#?}");
    assert_eq!(report.reference_rows, 3);
    assert_eq!(report.metrics.len(), 7);
    assert!(report.metrics.iter().all(|metric| metric.exceedances == 0));
}

#[test]
fn compare_telemetry_accepts_external_reference_json() {
    let temp = Builder::new()
        .prefix("openbmp_compare_telemetry_json")
        .tempdir()
        .expect("tempdir");
    let json = temp.path().join("external-reference.json");
    let mapping = temp.path().join("mapping.toml");

    fs::write(
        &json,
        r#"
{
  "time": [0.0, 5.0, 10.0],
  "altitude_km": [1.0, 0.877416875, 0.5096675],
  "speed_m_s": [0.0, 49.03325, 98.0665]
}
"#,
    )
    .expect("write reference json");
    fs::write(
        &mapping,
        r#"
[reference]
time_column = "time"

[comparison]
time_tolerance_s = 0.02

[[metrics]]
id = "altitude"
reference_column = "altitude_km"
reference_scale = 1000.0
tolerance_abs = 1e-9
actual = { kind = "altitude_from_position", radius_m = 0.0 }

[[metrics]]
id = "speed"
reference_column = "speed_m_s"
tolerance_abs = 1e-9
actual = { kind = "speed_from_velocity" }
"#,
    )
    .expect("write mapping");

    let report = compare_telemetry::run(&scenario_path(), &json, &mapping).expect("compare");
    assert!(report.passed(), "{report:#?}");
    assert_eq!(report.reference_rows, 3);
    assert_eq!(report.metrics.len(), 2);
}

#[test]
fn compare_telemetry_accepts_external_reference_ndjson() {
    let temp = Builder::new()
        .prefix("openbmp_compare_telemetry_ndjson")
        .tempdir()
        .expect("tempdir");
    let json = temp.path().join("external-reference.ndjson");
    let mapping = temp.path().join("mapping.toml");

    fs::write(
        &json,
        r#"
{"time": 0.0, "altitude_km": 1.0, "speed_m_s": 0.0}
{"time": 5.0, "altitude_km": 0.877416875, "speed_m_s": 49.03325}
{"time": 10.0, "altitude_km": 0.5096675, "speed_m_s": 98.0665}
"#,
    )
    .expect("write reference ndjson");
    fs::write(
        &mapping,
        r#"
[reference]
time_column = "time"

[comparison]
time_tolerance_s = 0.02

[[metrics]]
id = "altitude"
reference_column = "altitude_km"
reference_scale = 1000.0
tolerance_abs = 1e-9
actual = { kind = "altitude_from_position", radius_m = 0.0 }

[[metrics]]
id = "speed"
reference_column = "speed_m_s"
tolerance_abs = 1e-9
actual = { kind = "speed_from_velocity" }
"#,
    )
    .expect("write mapping");

    let report = compare_telemetry::run(&scenario_path(), &json, &mapping).expect("compare");
    assert!(report.passed(), "{report:#?}");
    assert_eq!(report.reference_rows, 3);
    assert_eq!(report.metrics.len(), 2);
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

#[test]
fn compare_telemetry_respects_metric_time_windows() {
    let temp = Builder::new()
        .prefix("openbmp_compare_telemetry_window")
        .tempdir()
        .expect("tempdir");
    let csv = temp.path().join("external-reference.csv");
    let mapping = temp.path().join("mapping.toml");

    fs::write(
        &csv,
        "time_s,altitude_m\n\
         0.0,1000.0\n\
         5.0,877.416875\n\
         10.0,509.6675\n",
    )
    .expect("write reference csv");
    fs::write(
        &mapping,
        r#"
[reference]
time_column = "time_s"

[[metrics]]
id = "coast_altitude"
reference_column = "altitude_m"
time_min_s = 4.0
time_max_s = 6.0
tolerance_abs = 1e-9
actual = { kind = "altitude_from_position", radius_m = 0.0 }
"#,
    )
    .expect("write mapping");

    let report = compare_telemetry::run(&scenario_path(), &csv, &mapping).expect("compare");
    assert!(report.passed(), "{report:#?}");
    assert_eq!(report.reference_rows, 3);
    assert_eq!(report.metrics[0].compared_samples, 1);
    assert_eq!(report.metrics[0].skipped_samples, 0);
}
