//! Offline system-identification and model-correlation commands.

use std::fs;
use std::path::Path;

use openbmp_sysid::{
    CsvChannelColumn, CsvSeriesSpec, PhaseSegment, ResidualChannel, ResidualComparison,
    ResidualReport, ResidualTolerance, SysidError, compare_residuals, read_csv_series,
    series_by_id,
};
use serde::Deserialize;

use crate::error::CliError;

/// Run the standalone residual comparison command.
///
/// # Errors
///
/// Returns [`CliError`] when the mapping/CSV inputs are invalid, the report
/// cannot be written, or the residual report is out of family.
pub fn run_residuals(
    reference_csv: &Path,
    actual_csv: &Path,
    mapping_path: &Path,
    report_json: &Path,
) -> Result<ResidualReport, CliError> {
    let mapping = read_mapping(mapping_path)?;
    let time_column = mapping
        .comparison
        .as_ref()
        .map_or_else(default_time_column, |comparison| {
            comparison.time_column.clone()
        });
    let phases = build_phases(&mapping, mapping_path)?;
    let reference_spec = build_csv_spec(&time_column, &mapping.metrics, false)?;
    let actual_spec = build_csv_spec(&time_column, &mapping.metrics, true)?;

    let reference = read_csv_series(reference_csv, &reference_spec)
        .and_then(series_by_id)
        .map_err(|err| sysid_config_error(reference_csv, err))?;
    let actual = read_csv_series(actual_csv, &actual_spec)
        .and_then(series_by_id)
        .map_err(|err| sysid_config_error(actual_csv, err))?;

    let mut channels = Vec::with_capacity(mapping.metrics.len());
    for metric in &mapping.metrics {
        let reference_series =
            reference
                .get(&metric.id)
                .cloned()
                .ok_or_else(|| CliError::SysidConfig {
                    path: mapping_path.to_path_buf(),
                    summary: format!("reference series {} was not loaded", metric.id),
                })?;
        let actual_series =
            actual
                .get(&metric.id)
                .cloned()
                .ok_or_else(|| CliError::SysidConfig {
                    path: mapping_path.to_path_buf(),
                    summary: format!("actual series {} was not loaded", metric.id),
                })?;
        let tolerance = ResidualTolerance::new(
            metric.tolerance_abs,
            metric.tolerance_rel.unwrap_or(0.0),
            metric.relative_floor.unwrap_or(1.0),
        )
        .map_err(|err| sysid_config_error(mapping_path, err))?;
        channels.push(
            ResidualChannel::new(&metric.id, reference_series, actual_series, tolerance)
                .map_err(|err| sysid_config_error(mapping_path, err))?,
        );
    }

    let request = ResidualComparison::new(channels, phases)
        .map_err(|err| sysid_config_error(mapping_path, err))?;
    let report =
        compare_residuals(&request).map_err(|err| sysid_config_error(mapping_path, err))?;
    let text =
        serde_json::to_string_pretty(&report).map_err(|source| CliError::SysidReportJson {
            path: report_json.to_path_buf(),
            source,
        })?;
    fs::write(report_json, text).map_err(|source| CliError::Io {
        path: report_json.to_path_buf(),
        source,
    })?;
    Ok(report)
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MappingDocument {
    comparison: Option<ComparisonConfig>,
    #[serde(default)]
    phases: Vec<PhaseConfig>,
    metrics: Vec<MetricConfig>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ComparisonConfig {
    #[serde(default = "default_time_column")]
    time_column: String,
}

impl Default for ComparisonConfig {
    fn default() -> Self {
        Self {
            time_column: default_time_column(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PhaseConfig {
    label: String,
    start_s: f64,
    end_s: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MetricConfig {
    id: String,
    reference_column: String,
    actual_column: String,
    unit: String,
    actual_unit: Option<String>,
    frame: Option<String>,
    actual_frame: Option<String>,
    tolerance_abs: f64,
    tolerance_rel: Option<f64>,
    relative_floor: Option<f64>,
}

fn read_mapping(path: &Path) -> Result<MappingDocument, CliError> {
    let text = fs::read_to_string(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let mapping: MappingDocument =
        toml::from_str(&text).map_err(|source| CliError::SysidConfig {
            path: path.to_path_buf(),
            summary: source.to_string(),
        })?;
    if mapping.metrics.is_empty() {
        return Err(CliError::SysidConfig {
            path: path.to_path_buf(),
            summary: "at least one [[metrics]] entry is required".to_owned(),
        });
    }
    Ok(mapping)
}

fn build_phases(
    mapping: &MappingDocument,
    mapping_path: &Path,
) -> Result<Vec<PhaseSegment>, CliError> {
    mapping
        .phases
        .iter()
        .map(|phase| {
            PhaseSegment::new(&phase.label, phase.start_s, phase.end_s)
                .map_err(|err| sysid_config_error(mapping_path, err))
        })
        .collect()
}

fn build_csv_spec(
    time_column: &str,
    metrics: &[MetricConfig],
    actual: bool,
) -> Result<CsvSeriesSpec, CliError> {
    let channels = metrics
        .iter()
        .map(|metric| {
            let column = if actual {
                &metric.actual_column
            } else {
                &metric.reference_column
            };
            let unit = if actual {
                metric.actual_unit.as_ref().unwrap_or(&metric.unit)
            } else {
                &metric.unit
            };
            let frame = if actual {
                metric.actual_frame.as_ref().or(metric.frame.as_ref())
            } else {
                metric.frame.as_ref()
            };
            CsvChannelColumn::new(&metric.id, column, unit, frame.map(String::as_str))
                .map_err(|err| sysid_config_error(Path::new("<mapping>"), err))
        })
        .collect::<Result<Vec<_>, _>>()?;
    CsvSeriesSpec::new(time_column, channels)
        .map_err(|err| sysid_config_error(Path::new("<mapping>"), err))
}

fn sysid_config_error(path: &Path, err: SysidError) -> CliError {
    CliError::SysidConfig {
        path: path.to_path_buf(),
        summary: err.to_string(),
    }
}

fn default_time_column() -> String {
    "time_s".to_owned()
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn sysid_residuals_command_writes_json_report() {
        let temp = tempfile::tempdir().unwrap();
        let reference = temp.path().join("reference.csv");
        let actual = temp.path().join("actual.csv");
        let mapping = temp.path().join("mapping.toml");
        let report = temp.path().join("report.json");
        fs::write(&reference, "time_s,altitude_m\n0,10\n1,20\n2,30\n").unwrap();
        fs::write(&actual, "time_s,altitude_m\n0,10\n1,20\n2,30\n").unwrap();
        fs::write(
            &mapping,
            r#"
[comparison]
time_column = "time_s"

[[phases]]
label = "ascent"
start_s = 0.0
end_s = 2.0

[[metrics]]
id = "altitude"
reference_column = "altitude_m"
actual_column = "altitude_m"
unit = "m"
frame = "ECI"
tolerance_abs = 1.0e-12
"#,
        )
        .unwrap();

        let residuals = run_residuals(&reference, &actual, &mapping, &report).unwrap();
        assert!(!residuals.any_out_of_family);
        let text = fs::read_to_string(report).unwrap();
        assert!(text.contains("\"channel_count\": 1"));
        assert!(text.contains("\"label\": \"ascent\""));
    }

    #[test]
    fn sysid_residuals_command_rejects_frame_mismatch() {
        let temp = tempfile::tempdir().unwrap();
        let reference = temp.path().join("reference.csv");
        let actual = temp.path().join("actual.csv");
        let mapping = temp.path().join("mapping.toml");
        let report = temp.path().join("report.json");
        fs::write(&reference, "time_s,range_m\n0,10\n").unwrap();
        fs::write(&actual, "time_s,range_m\n0,10\n").unwrap();
        fs::write(
            &mapping,
            r#"
[[metrics]]
id = "range"
reference_column = "range_m"
actual_column = "range_m"
unit = "m"
frame = "ECI"
actual_frame = "Body"
tolerance_abs = 1.0e-12
"#,
        )
        .unwrap();

        let err = run_residuals(&reference, &actual, &mapping, &report).unwrap_err();
        assert!(matches!(err, CliError::SysidConfig { .. }));
        assert!(!report.exists());
    }
}
