//! `openbmp compare-telemetry` — compare a scenario run against a
//! local external telemetry CSV / JSON without importing that data into the
//! repository.
//!
//! The command is deliberately source-agnostic. A mapping TOML names the
//! external columns, OpenBMP telemetry observables, and broad
//! tolerances. The command runs the scenario, interpolates OpenBMP
//! telemetry to the reference timestamps, and reports envelope
//! differences. It does not fit parameters, mutate scenarios, or persist
//! the external reference data.

use std::cmp::Ordering;
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

use openbmp_core::ChannelId;
use openbmp_runner as runner;
use openbmp_scenario::Scenario;
use openbmp_telemetry::{TelemetryRow, TelemetryTable, TelemetryValue};
use serde::{Deserialize, Serialize};

use crate::error::CliError;

const DEFAULT_TIME_TOLERANCE_S: f64 = 0.25;
const DEFAULT_RELATIVE_FLOOR: f64 = 1.0;
const WGS84_EARTH_ROTATION_RAD_S: f64 = 7.292_115_146_7e-5;

/// Top-level report from an external telemetry comparison.
#[derive(Clone, Debug, Serialize)]
pub struct CompareReport {
    /// Scenario name that was run.
    pub scenario_name: String,
    /// Number of rows in the external reference data.
    pub reference_rows: usize,
    /// Final simulation time in seconds.
    pub final_time_s: f64,
    /// Stop reason label from the scenario run.
    pub stop_label: String,
    /// Per-metric comparison reports.
    pub metrics: Vec<MetricReport>,
}

impl CompareReport {
    /// Returns true when every metric has at least one compared sample
    /// and no sample exceeded its declared tolerance envelope.
    #[must_use]
    pub fn passed(&self) -> bool {
        !self.metrics.is_empty()
            && self
                .metrics
                .iter()
                .all(|metric| metric.compared_samples > 0 && metric.exceedances == 0)
    }

    /// Compact failure summary suitable for the CLI error line.
    #[must_use]
    pub fn failure_summary(&self) -> String {
        let exceeded_metrics = self
            .metrics
            .iter()
            .filter(|metric| metric.exceedances > 0)
            .count();
        let exceeded_samples: usize = self.metrics.iter().map(|metric| metric.exceedances).sum();
        let empty_metrics = self
            .metrics
            .iter()
            .filter(|metric| metric.compared_samples == 0)
            .count();

        if exceeded_metrics == 0 && empty_metrics == 0 {
            return "comparison did not pass".to_owned();
        }

        let mut parts = Vec::new();
        if exceeded_metrics > 0 {
            parts.push(format!(
                "{exceeded_metrics} metric(s), {exceeded_samples} sample(s) exceeded tolerance"
            ));
        }
        if empty_metrics > 0 {
            parts.push(format!(
                "{empty_metrics} metric(s) had no comparable samples"
            ));
        }
        parts.join("; ")
    }
}

/// Per-observable comparison statistics.
#[derive(Clone, Debug, Serialize)]
pub struct MetricReport {
    /// Stable metric identifier from the mapping file.
    pub id: String,
    /// Absolute tolerance used for this metric.
    pub tolerance_abs: f64,
    /// Relative tolerance used for this metric.
    pub tolerance_rel: f64,
    /// Samples with both reference and interpolated simulation values.
    pub compared_samples: usize,
    /// Samples skipped because the reference value was absent or the
    /// simulation could not be interpolated near that reference time.
    pub skipped_samples: usize,
    /// Maximum absolute error over compared samples.
    pub max_abs_error: f64,
    /// Reference timestamp where `max_abs_error` occurred.
    pub max_abs_error_time_s: Option<f64>,
    /// Reference value at `max_abs_error_time_s`.
    pub max_abs_error_reference: Option<f64>,
    /// Interpolated OpenBMP value at `max_abs_error_time_s`.
    pub max_abs_error_actual: Option<f64>,
    /// Root-mean-square absolute error over compared samples.
    pub rms_abs_error: f64,
    /// Number of compared samples whose absolute error exceeded
    /// `max(tolerance_abs, tolerance_rel * max(abs(reference),
    /// relative_floor))`.
    pub exceedances: usize,
    #[serde(skip)]
    sum_sq_abs_error: f64,
}

impl MetricReport {
    fn new(config: &MetricConfig) -> Self {
        Self {
            id: config.id.clone(),
            tolerance_abs: config.tolerance_abs,
            tolerance_rel: config.tolerance_rel.unwrap_or(0.0),
            compared_samples: 0,
            skipped_samples: 0,
            max_abs_error: 0.0,
            max_abs_error_time_s: None,
            max_abs_error_reference: None,
            max_abs_error_actual: None,
            rms_abs_error: 0.0,
            exceedances: 0,
            sum_sq_abs_error: 0.0,
        }
    }

    fn record(&mut self, time_s: f64, reference: f64, actual: f64, abs_error: f64, tolerance: f64) {
        self.compared_samples += 1;
        if abs_error >= self.max_abs_error {
            self.max_abs_error = abs_error;
            self.max_abs_error_time_s = Some(time_s);
            self.max_abs_error_reference = Some(reference);
            self.max_abs_error_actual = Some(actual);
        }
        self.sum_sq_abs_error += abs_error * abs_error;
        if abs_error > tolerance {
            self.exceedances += 1;
        }
    }

    fn skip(&mut self) {
        self.skipped_samples += 1;
    }

    fn finish(&mut self) {
        if self.compared_samples > 0 {
            self.rms_abs_error = (self.sum_sq_abs_error / self.compared_samples as f64).sqrt();
        }
    }
}

/// Run a scenario and compare selected telemetry observables with a
/// local external CSV or JSON reference.
///
/// # Errors
///
/// Returns [`CliError`] for scenario/runner failures, unreadable mapping
/// or reference files, malformed mapping entries, missing telemetry channels,
/// and malformed reference rows.
pub fn run(
    scenario_path: &Path,
    reference_path: &Path,
    mapping_path: &Path,
) -> Result<CompareReport, CliError> {
    let mapping = read_mapping(mapping_path)?;
    validate_mapping(&mapping, mapping_path)?;

    let scenario = Scenario::from_file(scenario_path)?;
    let outcome = runner::run(&scenario)?;
    let reference = read_reference(reference_path, mapping_path, &mapping)?;
    let actual = build_actual_series(&outcome.table, mapping_path, &mapping.metrics)?;

    let mut metric_reports: Vec<MetricReport> =
        mapping.metrics.iter().map(MetricReport::new).collect();
    let time_tolerance_s = mapping
        .comparison
        .as_ref()
        .and_then(|comparison| comparison.time_tolerance_s)
        .unwrap_or(DEFAULT_TIME_TOLERANCE_S);

    for row in &reference.rows {
        for (index, maybe_reference) in row.values.iter().enumerate() {
            let metric = &mapping.metrics[index];
            if !metric.applies_at(row.time_s) {
                continue;
            }
            let Some(reference_value) = maybe_reference else {
                metric_reports[index].skip();
                continue;
            };
            let Some(actual_value) = actual[index].interpolate(row.time_s, time_tolerance_s) else {
                metric_reports[index].skip();
                continue;
            };
            let abs_error = (actual_value - reference_value).abs();
            let tolerance = metric.tolerance_for(*reference_value);
            metric_reports[index].record(
                row.time_s,
                *reference_value,
                actual_value,
                abs_error,
                tolerance,
            );
        }
    }
    for metric in &mut metric_reports {
        metric.finish();
    }

    Ok(CompareReport {
        scenario_name: scenario.document.meta.name,
        reference_rows: reference.rows.len(),
        final_time_s: outcome.final_time_s,
        stop_label: outcome.stop_reason.label().to_owned(),
        metrics: metric_reports,
    })
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MappingDocument {
    reference: ReferenceConfig,
    comparison: Option<ComparisonConfig>,
    metrics: Vec<MetricConfig>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReferenceConfig {
    time_column: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ComparisonConfig {
    time_tolerance_s: Option<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MetricConfig {
    id: String,
    reference_column: String,
    #[serde(default = "one")]
    reference_scale: f64,
    #[serde(default)]
    reference_offset: f64,
    actual: ActualSpec,
    tolerance_abs: f64,
    #[serde(default)]
    tolerance_rel: Option<f64>,
    #[serde(default)]
    relative_floor: Option<f64>,
    #[serde(default)]
    time_min_s: Option<f64>,
    #[serde(default)]
    time_max_s: Option<f64>,
}

impl MetricConfig {
    fn tolerance_for(&self, reference_value: f64) -> f64 {
        let rel = self.tolerance_rel.unwrap_or(0.0);
        let floor = self.relative_floor.unwrap_or(DEFAULT_RELATIVE_FLOOR);
        self.tolerance_abs
            .max(rel * reference_value.abs().max(floor))
    }

    fn applies_at(&self, time_s: f64) -> bool {
        self.time_min_s.is_none_or(|min| time_s >= min)
            && self.time_max_s.is_none_or(|max| time_s <= max)
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum ActualSpec {
    Channel {
        name: String,
    },
    AltitudeFromPosition {
        #[serde(default = "default_position_x")]
        x: String,
        #[serde(default = "default_position_y")]
        y: String,
        #[serde(default = "default_position_z")]
        z: String,
        radius_m: f64,
    },
    SpeedFromVelocity {
        #[serde(default = "default_velocity_x")]
        x: String,
        #[serde(default = "default_velocity_y")]
        y: String,
        #[serde(default = "default_velocity_z")]
        z: String,
    },
    SurfaceRelativeSpeed {
        #[serde(default = "default_position_x")]
        position_x: String,
        #[serde(default = "default_position_y")]
        position_y: String,
        #[serde(default = "default_position_z")]
        position_z: String,
        #[serde(default = "default_velocity_x")]
        velocity_x: String,
        #[serde(default = "default_velocity_y")]
        velocity_y: String,
        #[serde(default = "default_velocity_z")]
        velocity_z: String,
        #[serde(default = "default_earth_rotation_rad_s")]
        omega_rad_s: f64,
    },
    SurfaceRelativeAxisVelocity {
        #[serde(default = "default_position_x")]
        position_x: String,
        #[serde(default = "default_position_y")]
        position_y: String,
        #[serde(default = "default_position_z")]
        position_z: String,
        #[serde(default = "default_velocity_x")]
        velocity_x: String,
        #[serde(default = "default_velocity_y")]
        velocity_y: String,
        #[serde(default = "default_velocity_z")]
        velocity_z: String,
        axis_eci: [f64; 3],
        #[serde(default = "default_earth_rotation_rad_s")]
        omega_rad_s: f64,
    },
    SurfaceRelativeLocalAxisVelocity {
        #[serde(default = "default_position_x")]
        position_x: String,
        #[serde(default = "default_position_y")]
        position_y: String,
        #[serde(default = "default_position_z")]
        position_z: String,
        #[serde(default = "default_velocity_x")]
        velocity_x: String,
        #[serde(default = "default_velocity_y")]
        velocity_y: String,
        #[serde(default = "default_velocity_z")]
        velocity_z: String,
        axis_eci: [f64; 3],
        #[serde(default = "default_earth_rotation_rad_s")]
        omega_rad_s: f64,
    },
    SurfaceRelativeHorizontalSpeed {
        #[serde(default = "default_position_x")]
        position_x: String,
        #[serde(default = "default_position_y")]
        position_y: String,
        #[serde(default = "default_position_z")]
        position_z: String,
        #[serde(default = "default_velocity_x")]
        velocity_x: String,
        #[serde(default = "default_velocity_y")]
        velocity_y: String,
        #[serde(default = "default_velocity_z")]
        velocity_z: String,
        #[serde(default = "default_earth_rotation_rad_s")]
        omega_rad_s: f64,
    },
    SurfaceRelativeRadialVelocity {
        #[serde(default = "default_position_x")]
        position_x: String,
        #[serde(default = "default_position_y")]
        position_y: String,
        #[serde(default = "default_position_z")]
        position_z: String,
        #[serde(default = "default_velocity_x")]
        velocity_x: String,
        #[serde(default = "default_velocity_y")]
        velocity_y: String,
        #[serde(default = "default_velocity_z")]
        velocity_z: String,
        #[serde(default = "default_earth_rotation_rad_s")]
        omega_rad_s: f64,
    },
    Norm3 {
        x: String,
        y: String,
        z: String,
    },
}

#[derive(Debug)]
struct ReferenceTable {
    rows: Vec<ReferenceRow>,
}

#[derive(Debug)]
struct ReferenceRow {
    time_s: f64,
    values: Vec<Option<f64>>,
}

#[derive(Debug)]
struct ActualSeries {
    times: Vec<f64>,
    values: Vec<f64>,
}

impl ActualSeries {
    fn new() -> Self {
        Self {
            times: Vec::new(),
            values: Vec::new(),
        }
    }

    fn push(&mut self, time_s: f64, value: f64) {
        self.times.push(time_s);
        self.values.push(value);
    }

    fn interpolate(&self, time_s: f64, tolerance_s: f64) -> Option<f64> {
        if self.times.is_empty() || !time_s.is_finite() {
            return None;
        }
        match self
            .times
            .binary_search_by(|probe| probe.partial_cmp(&time_s).unwrap_or(Ordering::Less))
        {
            Ok(index) => self.values.get(index).copied(),
            Err(index) => {
                if index == 0 || index >= self.times.len() {
                    return None;
                }
                let low_index = index - 1;
                let high_index = index;
                let low_t = self.times[low_index];
                let high_t = self.times[high_index];
                let nearest_dt = (time_s - low_t).abs().min((high_t - time_s).abs());
                if nearest_dt > tolerance_s || high_t <= low_t {
                    return None;
                }
                let alpha = (time_s - low_t) / (high_t - low_t);
                let low = self.values[low_index];
                let high = self.values[high_index];
                Some(low + alpha * (high - low))
            }
        }
    }
}

fn read_mapping(path: &Path) -> Result<MappingDocument, CliError> {
    let text = fs::read_to_string(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    toml::from_str(&text).map_err(|source| CliError::TelemetryCompareConfig {
        path: path.to_path_buf(),
        summary: source.to_string(),
    })
}

fn validate_mapping(mapping: &MappingDocument, path: &Path) -> Result<(), CliError> {
    if mapping.reference.time_column.trim().is_empty() {
        return Err(config_error(
            path,
            "reference.time_column must not be empty",
        ));
    }
    if let Some(comparison) = &mapping.comparison
        && let Some(value) = comparison.time_tolerance_s
        && (!value.is_finite() || value < 0.0)
    {
        return Err(config_error(
            path,
            "comparison.time_tolerance_s must be finite and non-negative",
        ));
    }
    if mapping.metrics.is_empty() {
        return Err(config_error(
            path,
            "at least one [[metrics]] entry is required",
        ));
    }

    let mut ids = BTreeMap::new();
    for metric in &mapping.metrics {
        if metric.id.trim().is_empty() {
            return Err(config_error(path, "metrics.id must not be empty"));
        }
        if let Some(previous) = ids.insert(metric.id.clone(), metric.id.clone()) {
            return Err(config_error(
                path,
                format!("duplicate metric id {previous}"),
            ));
        }
        if metric.reference_column.trim().is_empty() {
            return Err(config_error(
                path,
                format!("metric {} reference_column must not be empty", metric.id),
            ));
        }
        if !metric.reference_scale.is_finite() {
            return Err(config_error(
                path,
                format!("metric {} reference_scale must be finite", metric.id),
            ));
        }
        if !metric.reference_offset.is_finite() {
            return Err(config_error(
                path,
                format!("metric {} reference_offset must be finite", metric.id),
            ));
        }
        if !metric.tolerance_abs.is_finite() || metric.tolerance_abs < 0.0 {
            return Err(config_error(
                path,
                format!(
                    "metric {} tolerance_abs must be finite and non-negative",
                    metric.id
                ),
            ));
        }
        if let Some(value) = metric.tolerance_rel
            && (!value.is_finite() || value < 0.0)
        {
            return Err(config_error(
                path,
                format!(
                    "metric {} tolerance_rel must be finite and non-negative",
                    metric.id
                ),
            ));
        }
        if let Some(value) = metric.relative_floor
            && (!value.is_finite() || value < 0.0)
        {
            return Err(config_error(
                path,
                format!(
                    "metric {} relative_floor must be finite and non-negative",
                    metric.id
                ),
            ));
        }
        if let Some(value) = metric.time_min_s
            && !value.is_finite()
        {
            return Err(config_error(
                path,
                format!("metric {} time_min_s must be finite", metric.id),
            ));
        }
        if let Some(value) = metric.time_max_s
            && !value.is_finite()
        {
            return Err(config_error(
                path,
                format!("metric {} time_max_s must be finite", metric.id),
            ));
        }
        if let (Some(min), Some(max)) = (metric.time_min_s, metric.time_max_s)
            && min > max
        {
            return Err(config_error(
                path,
                format!("metric {} time_min_s must be <= time_max_s", metric.id),
            ));
        }
    }
    Ok(())
}

fn read_reference(
    reference_path: &Path,
    mapping_path: &Path,
    mapping: &MappingDocument,
) -> Result<ReferenceTable, CliError> {
    if reference_path
        .extension()
        .and_then(std::ffi::OsStr::to_str)
        .is_some_and(is_json_reference_extension)
    {
        return read_reference_json(reference_path, mapping_path, mapping);
    }
    read_reference_csv(reference_path, mapping_path, mapping)
}

fn is_json_reference_extension(extension: &str) -> bool {
    extension.eq_ignore_ascii_case("json")
        || extension.eq_ignore_ascii_case("jsonl")
        || extension.eq_ignore_ascii_case("ndjson")
}

fn read_reference_csv(
    csv_path: &Path,
    mapping_path: &Path,
    mapping: &MappingDocument,
) -> Result<ReferenceTable, CliError> {
    let mut reader = csv::Reader::from_path(csv_path).map_err(|source| CliError::Csv {
        path: csv_path.to_path_buf(),
        source,
    })?;
    let headers = reader
        .headers()
        .map_err(|source| CliError::Csv {
            path: csv_path.to_path_buf(),
            source,
        })?
        .clone();
    let time_index = column_index(&headers, &mapping.reference.time_column, mapping_path)?;
    let metric_indices: Vec<usize> = mapping
        .metrics
        .iter()
        .map(|metric| column_index(&headers, &metric.reference_column, mapping_path))
        .collect::<Result<_, _>>()?;

    let mut rows = Vec::new();
    for result in reader.records() {
        let record = result.map_err(|source| CliError::Csv {
            path: csv_path.to_path_buf(),
            source,
        })?;
        let time_s = parse_required_cell(&record, time_index, "time", csv_path)?;
        let mut values = Vec::with_capacity(mapping.metrics.len());
        for (metric, index) in mapping.metrics.iter().zip(metric_indices.iter().copied()) {
            let value = parse_optional_cell(&record, index, csv_path)?
                .map(|raw| raw * metric.reference_scale + metric.reference_offset);
            values.push(value);
        }
        rows.push(ReferenceRow { time_s, values });
    }

    if rows.is_empty() {
        return Err(config_error(csv_path, "reference data contains no rows"));
    }
    Ok(ReferenceTable { rows })
}

fn read_reference_json(
    json_path: &Path,
    mapping_path: &Path,
    mapping: &MappingDocument,
) -> Result<ReferenceTable, CliError> {
    let text = fs::read_to_string(json_path).map_err(|source| CliError::Io {
        path: json_path.to_path_buf(),
        source,
    })?;
    let value: serde_json::Value = match serde_json::from_str(&text) {
        Ok(value) => value,
        Err(container_error) => {
            return read_reference_json_lines(&text, json_path, mapping).map_err(|line_error| {
                config_error(
                    json_path,
                    format!(
                        "could not parse reference JSON ({container_error}); also failed as \
                         newline-delimited JSON: {line_error}"
                    ),
                )
            });
        }
    };
    match value {
        serde_json::Value::Array(rows) => read_reference_json_rows(rows, json_path, mapping),
        serde_json::Value::Object(columns) => {
            read_reference_json_columns(columns, json_path, mapping_path, mapping)
        }
        _ => Err(config_error(
            json_path,
            "reference JSON must be an object of column arrays or an array of row objects",
        )),
    }
}

fn read_reference_json_lines(
    text: &str,
    json_path: &Path,
    mapping: &MappingDocument,
) -> Result<ReferenceTable, String> {
    let mut rows = Vec::new();
    for (line_index, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let value = serde_json::from_str(line)
            .map_err(|source| format!("line {}: {source}", line_index + 1))?;
        rows.push(value);
    }
    read_reference_json_rows(rows, json_path, mapping).map_err(|source| source.to_string())
}

fn read_reference_json_columns(
    columns: serde_json::Map<String, serde_json::Value>,
    json_path: &Path,
    mapping_path: &Path,
    mapping: &MappingDocument,
) -> Result<ReferenceTable, CliError> {
    let time_values = json_column(&columns, &mapping.reference.time_column, mapping_path)?;
    let metric_values: Vec<&[serde_json::Value]> = mapping
        .metrics
        .iter()
        .map(|metric| json_column(&columns, &metric.reference_column, mapping_path))
        .collect::<Result<_, _>>()?;

    let mut rows = Vec::with_capacity(time_values.len());
    for index in 0..time_values.len() {
        let time_s = parse_required_json_value(
            time_values
                .get(index)
                .ok_or_else(|| config_error(json_path, "missing required time value"))?,
            "time",
            json_path,
        )?;
        let mut values = Vec::with_capacity(mapping.metrics.len());
        for (metric, column) in mapping.metrics.iter().zip(metric_values.iter().copied()) {
            let value = column
                .get(index)
                .map(|value| parse_optional_json_value(value, json_path))
                .transpose()?
                .flatten()
                .map(|raw| raw * metric.reference_scale + metric.reference_offset);
            values.push(value);
        }
        rows.push(ReferenceRow { time_s, values });
    }

    if rows.is_empty() {
        return Err(config_error(json_path, "reference data contains no rows"));
    }
    Ok(ReferenceTable { rows })
}

fn read_reference_json_rows(
    raw_rows: Vec<serde_json::Value>,
    json_path: &Path,
    mapping: &MappingDocument,
) -> Result<ReferenceTable, CliError> {
    let mut rows = Vec::with_capacity(raw_rows.len());
    for (row_index, value) in raw_rows.iter().enumerate() {
        let serde_json::Value::Object(row) = value else {
            return Err(config_error(
                json_path,
                format!("reference JSON row {row_index} is not an object"),
            ));
        };
        let time_value = row.get(&mapping.reference.time_column).ok_or_else(|| {
            config_error(
                json_path,
                format!(
                    "reference JSON row {row_index} missing time column {:?}",
                    mapping.reference.time_column
                ),
            )
        })?;
        let time_s = parse_required_json_value(time_value, "time", json_path)?;
        let mut values = Vec::with_capacity(mapping.metrics.len());
        for metric in &mapping.metrics {
            let value = row
                .get(&metric.reference_column)
                .map(|value| parse_optional_json_value(value, json_path))
                .transpose()?
                .flatten()
                .map(|raw| raw * metric.reference_scale + metric.reference_offset);
            values.push(value);
        }
        rows.push(ReferenceRow { time_s, values });
    }
    if rows.is_empty() {
        return Err(config_error(json_path, "reference data contains no rows"));
    }
    Ok(ReferenceTable { rows })
}

fn json_column<'a>(
    columns: &'a serde_json::Map<String, serde_json::Value>,
    column: &str,
    mapping_path: &Path,
) -> Result<&'a [serde_json::Value], CliError> {
    let Some(value) = columns.get(column) else {
        return Err(config_error(
            mapping_path,
            format!("reference JSON column {column:?} is not present"),
        ));
    };
    let serde_json::Value::Array(values) = value else {
        return Err(config_error(
            mapping_path,
            format!("reference JSON column {column:?} is not an array"),
        ));
    };
    Ok(values)
}

fn build_actual_series(
    table: &TelemetryTable,
    mapping_path: &Path,
    metrics: &[MetricConfig],
) -> Result<Vec<ActualSeries>, CliError> {
    let lookup = channel_lookup(table);
    let mut series: Vec<ActualSeries> = metrics.iter().map(|_| ActualSeries::new()).collect();
    for row in table.rows() {
        let time_s = row.time.as_seconds();
        for (index, metric) in metrics.iter().enumerate() {
            if let Some(value) = actual_value(&metric.actual, row, &lookup, mapping_path)? {
                series[index].push(time_s, value);
            }
        }
    }
    Ok(series)
}

fn channel_lookup(table: &TelemetryTable) -> BTreeMap<String, ChannelId> {
    table
        .schema()
        .channels()
        .iter()
        .map(|channel| (channel.name.clone(), channel.id))
        .collect()
}

fn actual_value(
    spec: &ActualSpec,
    row: &TelemetryRow,
    lookup: &BTreeMap<String, ChannelId>,
    mapping_path: &Path,
) -> Result<Option<f64>, CliError> {
    match spec {
        ActualSpec::Channel { name } => f64_channel(row, lookup, name, mapping_path),
        ActualSpec::AltitudeFromPosition { x, y, z, radius_m } => {
            if !radius_m.is_finite() || *radius_m < 0.0 {
                return Err(config_error(
                    mapping_path,
                    "altitude_from_position.radius_m must be finite and non-negative",
                ));
            }
            let Some(px) = f64_channel(row, lookup, x, mapping_path)? else {
                return Ok(None);
            };
            let Some(py) = f64_channel(row, lookup, y, mapping_path)? else {
                return Ok(None);
            };
            let Some(pz) = f64_channel(row, lookup, z, mapping_path)? else {
                return Ok(None);
            };
            Ok(Some((px.mul_add(px, py * py) + pz * pz).sqrt() - radius_m))
        }
        ActualSpec::SpeedFromVelocity { x, y, z } | ActualSpec::Norm3 { x, y, z } => {
            let Some(vx) = f64_channel(row, lookup, x, mapping_path)? else {
                return Ok(None);
            };
            let Some(vy) = f64_channel(row, lookup, y, mapping_path)? else {
                return Ok(None);
            };
            let Some(vz) = f64_channel(row, lookup, z, mapping_path)? else {
                return Ok(None);
            };
            Ok(Some((vx.mul_add(vx, vy * vy) + vz * vz).sqrt()))
        }
        ActualSpec::SurfaceRelativeSpeed {
            position_x,
            position_y,
            position_z,
            velocity_x,
            velocity_y,
            velocity_z,
            omega_rad_s,
        } => {
            if !omega_rad_s.is_finite() {
                return Err(config_error(
                    mapping_path,
                    "surface_relative_speed.omega_rad_s must be finite",
                ));
            }
            let Some(px) = f64_channel(row, lookup, position_x, mapping_path)? else {
                return Ok(None);
            };
            let Some(py) = f64_channel(row, lookup, position_y, mapping_path)? else {
                return Ok(None);
            };
            let Some(_pz) = f64_channel(row, lookup, position_z, mapping_path)? else {
                return Ok(None);
            };
            let Some(vx) = f64_channel(row, lookup, velocity_x, mapping_path)? else {
                return Ok(None);
            };
            let Some(vy) = f64_channel(row, lookup, velocity_y, mapping_path)? else {
                return Ok(None);
            };
            let Some(vz) = f64_channel(row, lookup, velocity_z, mapping_path)? else {
                return Ok(None);
            };

            // For the current OpenBMP Earth-frame profiles, the body-fixed
            // surface rotates about ECI +z. Webcast velocity is surface
            // relative, so remove omega x r from the inertial velocity.
            let surface_vx = -omega_rad_s * py;
            let surface_vy = omega_rad_s * px;
            let rel_vx = vx - surface_vx;
            let rel_vy = vy - surface_vy;
            Ok(Some(
                (rel_vx.mul_add(rel_vx, rel_vy * rel_vy) + vz * vz).sqrt(),
            ))
        }
        ActualSpec::SurfaceRelativeAxisVelocity {
            position_x,
            position_y,
            position_z,
            velocity_x,
            velocity_y,
            velocity_z,
            axis_eci,
            omega_rad_s,
        } => {
            let Some((_, rel_v)) = surface_relative_state(
                row,
                lookup,
                mapping_path,
                position_x,
                position_y,
                position_z,
                velocity_x,
                velocity_y,
                velocity_z,
                *omega_rad_s,
            )?
            else {
                return Ok(None);
            };
            let axis_norm = (axis_eci[0].mul_add(axis_eci[0], axis_eci[1] * axis_eci[1])
                + axis_eci[2] * axis_eci[2])
                .sqrt();
            if !axis_norm.is_finite() || axis_norm <= 0.0 {
                return Err(config_error(
                    mapping_path,
                    "surface_relative_axis_velocity.axis_eci must be finite and non-zero",
                ));
            }
            Ok(Some(
                rel_v[0] * axis_eci[0] / axis_norm
                    + rel_v[1] * axis_eci[1] / axis_norm
                    + rel_v[2] * axis_eci[2] / axis_norm,
            ))
        }
        ActualSpec::SurfaceRelativeLocalAxisVelocity {
            position_x,
            position_y,
            position_z,
            velocity_x,
            velocity_y,
            velocity_z,
            axis_eci,
            omega_rad_s,
        } => {
            let Some((position, rel_v)) = surface_relative_state(
                row,
                lookup,
                mapping_path,
                position_x,
                position_y,
                position_z,
                velocity_x,
                velocity_y,
                velocity_z,
                *omega_rad_s,
            )?
            else {
                return Ok(None);
            };
            let axis_norm = (axis_eci[0].mul_add(axis_eci[0], axis_eci[1] * axis_eci[1])
                + axis_eci[2] * axis_eci[2])
                .sqrt();
            if !axis_norm.is_finite() || axis_norm <= 0.0 {
                return Err(config_error(
                    mapping_path,
                    "surface_relative_local_axis_velocity.axis_eci must be finite and non-zero",
                ));
            }
            let radius = (position[0].mul_add(position[0], position[1] * position[1])
                + position[2] * position[2])
                .sqrt();
            let local_axis = if radius > 1.0e6 {
                let radial = [
                    position[0] / radius,
                    position[1] / radius,
                    position[2] / radius,
                ];
                let axis_unit = [
                    axis_eci[0] / axis_norm,
                    axis_eci[1] / axis_norm,
                    axis_eci[2] / axis_norm,
                ];
                let radial_component =
                    axis_unit[0] * radial[0] + axis_unit[1] * radial[1] + axis_unit[2] * radial[2];
                let projected = [
                    axis_unit[0] - radial_component * radial[0],
                    axis_unit[1] - radial_component * radial[1],
                    axis_unit[2] - radial_component * radial[2],
                ];
                let projected_norm = (projected[0]
                    .mul_add(projected[0], projected[1] * projected[1])
                    + projected[2] * projected[2])
                    .sqrt();
                if !projected_norm.is_finite() || projected_norm <= 0.0 {
                    return Err(config_error(
                        mapping_path,
                        "surface_relative_local_axis_velocity.axis_eci must not be parallel to the local radial direction",
                    ));
                }
                [
                    projected[0] / projected_norm,
                    projected[1] / projected_norm,
                    projected[2] / projected_norm,
                ]
            } else {
                [
                    axis_eci[0] / axis_norm,
                    axis_eci[1] / axis_norm,
                    axis_eci[2] / axis_norm,
                ]
            };
            Ok(Some(
                rel_v[0] * local_axis[0] + rel_v[1] * local_axis[1] + rel_v[2] * local_axis[2],
            ))
        }
        ActualSpec::SurfaceRelativeHorizontalSpeed {
            position_x,
            position_y,
            position_z,
            velocity_x,
            velocity_y,
            velocity_z,
            omega_rad_s,
        } => {
            let Some((position, rel_v)) = surface_relative_state(
                row,
                lookup,
                mapping_path,
                position_x,
                position_y,
                position_z,
                velocity_x,
                velocity_y,
                velocity_z,
                *omega_rad_s,
            )?
            else {
                return Ok(None);
            };
            let speed_sq = rel_v[0].mul_add(rel_v[0], rel_v[1] * rel_v[1]) + rel_v[2] * rel_v[2];
            let radius = (position[0].mul_add(position[0], position[1] * position[1])
                + position[2] * position[2])
                .sqrt();
            if radius > 1.0e6 {
                let radial_v =
                    (rel_v[0] * position[0] + rel_v[1] * position[1] + rel_v[2] * position[2])
                        / radius;
                Ok(Some((speed_sq - radial_v * radial_v).max(0.0).sqrt()))
            } else {
                Ok(Some(
                    (rel_v[0].mul_add(rel_v[0], rel_v[1] * rel_v[1])).sqrt(),
                ))
            }
        }
        ActualSpec::SurfaceRelativeRadialVelocity {
            position_x,
            position_y,
            position_z,
            velocity_x,
            velocity_y,
            velocity_z,
            omega_rad_s,
        } => {
            let Some((position, rel_v)) = surface_relative_state(
                row,
                lookup,
                mapping_path,
                position_x,
                position_y,
                position_z,
                velocity_x,
                velocity_y,
                velocity_z,
                *omega_rad_s,
            )?
            else {
                return Ok(None);
            };
            let radius = (position[0].mul_add(position[0], position[1] * position[1])
                + position[2] * position[2])
                .sqrt();
            if radius > 1.0e6 {
                Ok(Some(
                    (rel_v[0] * position[0] + rel_v[1] * position[1] + rel_v[2] * position[2])
                        / radius,
                ))
            } else {
                Ok(Some(rel_v[2]))
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn surface_relative_state(
    row: &TelemetryRow,
    lookup: &BTreeMap<String, ChannelId>,
    mapping_path: &Path,
    position_x: &str,
    position_y: &str,
    position_z: &str,
    velocity_x: &str,
    velocity_y: &str,
    velocity_z: &str,
    omega_rad_s: f64,
) -> Result<Option<([f64; 3], [f64; 3])>, CliError> {
    if !omega_rad_s.is_finite() {
        return Err(config_error(
            mapping_path,
            "surface-relative velocity omega_rad_s must be finite",
        ));
    }
    let Some(px) = f64_channel(row, lookup, position_x, mapping_path)? else {
        return Ok(None);
    };
    let Some(py) = f64_channel(row, lookup, position_y, mapping_path)? else {
        return Ok(None);
    };
    let Some(pz) = f64_channel(row, lookup, position_z, mapping_path)? else {
        return Ok(None);
    };
    let Some(vx) = f64_channel(row, lookup, velocity_x, mapping_path)? else {
        return Ok(None);
    };
    let Some(vy) = f64_channel(row, lookup, velocity_y, mapping_path)? else {
        return Ok(None);
    };
    let Some(vz) = f64_channel(row, lookup, velocity_z, mapping_path)? else {
        return Ok(None);
    };

    let surface_vx = -omega_rad_s * py;
    let surface_vy = omega_rad_s * px;
    Ok(Some(([px, py, pz], [vx - surface_vx, vy - surface_vy, vz])))
}

fn f64_channel(
    row: &TelemetryRow,
    lookup: &BTreeMap<String, ChannelId>,
    name: &str,
    mapping_path: &Path,
) -> Result<Option<f64>, CliError> {
    let Some(id) = lookup.get(name).copied() else {
        return Err(config_error(
            mapping_path,
            format!("OpenBMP telemetry channel {name:?} is not present in the run output"),
        ));
    };
    let Some(value) = row.get(id) else {
        return Ok(None);
    };
    match value {
        TelemetryValue::Float64(value) => Ok(Some(*value)),
        _ => Err(config_error(
            mapping_path,
            format!("OpenBMP telemetry channel {name:?} is not float64"),
        )),
    }
}

fn column_index(
    headers: &csv::StringRecord,
    column: &str,
    mapping_path: &Path,
) -> Result<usize, CliError> {
    headers
        .iter()
        .position(|header| header == column)
        .ok_or_else(|| {
            config_error(
                mapping_path,
                format!("reference CSV column {column:?} is not present"),
            )
        })
}

fn parse_required_cell(
    record: &csv::StringRecord,
    index: usize,
    label: &str,
    path: &Path,
) -> Result<f64, CliError> {
    let Some(raw) = record.get(index) else {
        return Err(config_error(path, format!("missing required {label} cell")));
    };
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return Err(config_error(path, format!("empty required {label} cell")));
    }
    parse_finite(trimmed, path)
}

fn parse_optional_cell(
    record: &csv::StringRecord,
    index: usize,
    path: &Path,
) -> Result<Option<f64>, CliError> {
    let Some(raw) = record.get(index) else {
        return Ok(None);
    };
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return Ok(None);
    }
    parse_finite(trimmed, path).map(Some)
}

fn parse_required_json_value(
    value: &serde_json::Value,
    label: &str,
    path: &Path,
) -> Result<f64, CliError> {
    match parse_optional_json_value(value, path)? {
        Some(value) => Ok(value),
        None => Err(config_error(path, format!("empty required {label} value"))),
    }
}

fn parse_optional_json_value(
    value: &serde_json::Value,
    path: &Path,
) -> Result<Option<f64>, CliError> {
    match value {
        serde_json::Value::Null => Ok(None),
        serde_json::Value::Number(number) => {
            let value = number.as_f64().ok_or_else(|| {
                config_error(
                    path,
                    format!("could not read JSON number {number} as float64"),
                )
            })?;
            if value.is_finite() {
                Ok(Some(value))
            } else {
                Err(config_error(
                    path,
                    format!("non-finite JSON float64 value {number}"),
                ))
            }
        }
        serde_json::Value::String(raw) => {
            let trimmed = raw.trim();
            if trimmed.is_empty() {
                Ok(None)
            } else {
                parse_finite(trimmed, path).map(Some)
            }
        }
        _ => Err(config_error(
            path,
            format!("reference JSON value {value:?} is not numeric"),
        )),
    }
}

fn parse_finite(raw: &str, path: &Path) -> Result<f64, CliError> {
    let value = raw.parse::<f64>().map_err(|source| {
        config_error(
            path,
            format!("could not parse {raw:?} as float64: {source}"),
        )
    })?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(config_error(
            path,
            format!("non-finite float64 value {raw:?}"),
        ))
    }
}

fn config_error(path: &Path, summary: impl Into<String>) -> CliError {
    CliError::TelemetryCompareConfig {
        path: path.to_path_buf(),
        summary: summary.into(),
    }
}

const fn one() -> f64 {
    1.0
}

fn default_position_x() -> String {
    "position_x_m".to_owned()
}

fn default_position_y() -> String {
    "position_y_m".to_owned()
}

fn default_position_z() -> String {
    "position_z_m".to_owned()
}

fn default_velocity_x() -> String {
    "velocity_x_m_s".to_owned()
}

fn default_velocity_y() -> String {
    "velocity_y_m_s".to_owned()
}

fn default_velocity_z() -> String {
    "velocity_z_m_s".to_owned()
}

const fn default_earth_rotation_rad_s() -> f64 {
    WGS84_EARTH_ROTATION_RAD_S
}
