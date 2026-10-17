//! System-identification and model-correlation analysis helpers.
//!
//! This crate is intentionally offline and file-fed. It aligns scalar
//! time-series channels, checks unit/frame metadata before comparison,
//! interpolates actual samples to reference timestamps, and reports residual
//! statistics by channel and declared phase window.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Result alias for sys-id analysis operations.
pub type Result<T> = std::result::Result<T, SysidError>;

/// Errors returned by sys-id helpers.
#[derive(Debug, Error)]
pub enum SysidError {
    /// A required string field was blank after trimming.
    #[error("{field} must not be empty")]
    EmptyField {
        /// Field name.
        field: &'static str,
    },
    /// A numeric input was non-finite.
    #[error("{field} must be finite, got {value}")]
    NonFinite {
        /// Field name.
        field: &'static str,
        /// Offending value.
        value: f64,
    },
    /// A tolerance input was negative.
    #[error("{field} must be non-negative, got {value}")]
    NegativeTolerance {
        /// Field name.
        field: &'static str,
        /// Offending value.
        value: f64,
    },
    /// A time series has no samples.
    #[error("channel {channel} must contain at least one sample")]
    EmptySeries {
        /// Channel name.
        channel: String,
    },
    /// A time series timestamp did not increase strictly.
    #[error("channel {channel} timestamps must strictly increase at sample {sample_index}")]
    NonIncreasingTime {
        /// Channel name.
        channel: String,
        /// Offending sample index.
        sample_index: usize,
    },
    /// Reference and actual metadata are not compatible.
    #[error("channel {channel} {field} mismatch: reference={reference:?}, actual={actual:?}")]
    MetadataMismatch {
        /// Channel id.
        channel: String,
        /// Metadata field name.
        field: &'static str,
        /// Reference value.
        reference: Option<String>,
        /// Actual value.
        actual: Option<String>,
    },
    /// Duplicate channel id in a comparison request.
    #[error("duplicate channel id {channel}")]
    DuplicateChannel {
        /// Channel id.
        channel: String,
    },
    /// Duplicate phase label in a comparison request.
    #[error("duplicate phase label {label}")]
    DuplicatePhase {
        /// Phase label.
        label: String,
    },
    /// Invalid phase window.
    #[error("phase {label} requires start_s <= end_s")]
    InvalidPhaseWindow {
        /// Phase label.
        label: String,
    },
    /// A requested CSV column does not exist.
    #[error("CSV file {path} is missing column {column}")]
    MissingCsvColumn {
        /// CSV path.
        path: PathBuf,
        /// Missing column name.
        column: String,
    },
    /// A CSV value could not be parsed.
    #[error("CSV file {path} column {column} row {row_index} is not a finite f64: {value}")]
    CsvValue {
        /// CSV path.
        path: PathBuf,
        /// Column name.
        column: String,
        /// One-based data-row index.
        row_index: usize,
        /// Offending string value.
        value: String,
    },
    /// CSV parser error.
    #[error("CSV error: {path}")]
    Csv {
        /// CSV path.
        path: PathBuf,
        /// Source parser error.
        #[source]
        source: csv::Error,
    },
}

/// Metadata attached to one scalar time-series channel.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ChannelDescriptor {
    /// Stable channel name or metric id.
    pub name: String,
    /// Unit label such as `m`, `m/s`, or `rad`.
    pub unit: String,
    /// Optional coordinate-frame label, such as `ECI` or `Body`.
    pub frame: Option<String>,
}

impl ChannelDescriptor {
    /// Creates validated channel metadata.
    ///
    /// # Errors
    ///
    /// Returns [`SysidError::EmptyField`] for blank name, unit, or frame.
    pub fn new(
        name: impl Into<String>,
        unit: impl Into<String>,
        frame: Option<impl Into<String>>,
    ) -> Result<Self> {
        let name = require_non_empty("name", name.into())?;
        let unit = require_non_empty("unit", unit.into())?;
        let frame = match frame {
            Some(value) => Some(require_non_empty("frame", value.into())?),
            None => None,
        };
        Ok(Self { name, unit, frame })
    }
}

/// One scalar sample in a time series.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TimeSample {
    /// Timestamp in seconds.
    pub time_s: f64,
    /// Scalar sample value.
    pub value: f64,
}

impl TimeSample {
    /// Creates a validated finite sample.
    ///
    /// # Errors
    ///
    /// Returns [`SysidError::NonFinite`] when either field is non-finite.
    pub fn new(time_s: f64, value: f64) -> Result<Self> {
        Ok(Self {
            time_s: require_finite("time_s", time_s)?,
            value: require_finite("value", value)?,
        })
    }
}

/// A scalar time series with unit/frame metadata.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChannelSeries {
    /// Channel metadata.
    pub descriptor: ChannelDescriptor,
    /// Samples in strictly increasing timestamp order.
    pub samples: Vec<TimeSample>,
}

impl ChannelSeries {
    /// Creates a validated series.
    ///
    /// # Errors
    ///
    /// Returns an error when the series is empty, has non-finite samples, or
    /// timestamps do not strictly increase.
    pub fn new(descriptor: ChannelDescriptor, samples: Vec<TimeSample>) -> Result<Self> {
        if samples.is_empty() {
            return Err(SysidError::EmptySeries {
                channel: descriptor.name,
            });
        }
        for (index, sample) in samples.iter().enumerate() {
            require_finite("time_s", sample.time_s)?;
            require_finite("value", sample.value)?;
            if index > 0 && sample.time_s <= samples[index - 1].time_s {
                return Err(SysidError::NonIncreasingTime {
                    channel: descriptor.name,
                    sample_index: index,
                });
            }
        }
        Ok(Self {
            descriptor,
            samples,
        })
    }

    /// Linearly interpolates the series at `time_s`.
    ///
    /// Returns `None` outside the series time extent. A single-sample series
    /// can only be interpolated exactly at its timestamp.
    #[must_use]
    pub fn interpolate(&self, time_s: f64) -> Option<f64> {
        if !time_s.is_finite() {
            return None;
        }
        let first = self.samples.first()?;
        let last = self.samples.last()?;
        if time_s < first.time_s || time_s > last.time_s {
            return None;
        }
        if self.samples.len() == 1 {
            return matches!(time_s.total_cmp(&first.time_s), std::cmp::Ordering::Equal)
                .then_some(first.value);
        }

        match self
            .samples
            .binary_search_by(|sample| sample.time_s.total_cmp(&time_s))
        {
            Ok(index) => Some(self.samples[index].value),
            Err(upper) if upper > 0 && upper < self.samples.len() => {
                let before = self.samples[upper - 1];
                let after = self.samples[upper];
                let fraction = (time_s - before.time_s) / (after.time_s - before.time_s);
                Some(before.value + fraction * (after.value - before.value))
            }
            _ => None,
        }
    }
}

/// Residual tolerance envelope for one channel.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResidualTolerance {
    /// Absolute error tolerance.
    pub abs: f64,
    /// Relative error tolerance.
    pub rel: f64,
    /// Floor used by the relative envelope.
    pub relative_floor: f64,
}

impl ResidualTolerance {
    /// Creates a validated tolerance envelope.
    ///
    /// # Errors
    ///
    /// Returns an error for non-finite or negative values.
    pub fn new(abs: f64, rel: f64, relative_floor: f64) -> Result<Self> {
        Ok(Self {
            abs: require_non_negative("tolerance_abs", abs)?,
            rel: require_non_negative("tolerance_rel", rel)?,
            relative_floor: require_non_negative("relative_floor", relative_floor)?,
        })
    }

    /// Error limit for a reference value.
    #[must_use]
    pub fn limit_for(self, reference_value: f64) -> f64 {
        self.abs
            .max(self.rel * reference_value.abs().max(self.relative_floor))
    }
}

/// A named time window for phase-segmented residual statistics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhaseSegment {
    /// Stable phase label.
    pub label: String,
    /// Inclusive start time.
    pub start_s: f64,
    /// Inclusive end time.
    pub end_s: f64,
}

impl PhaseSegment {
    /// Creates a validated phase segment.
    ///
    /// # Errors
    ///
    /// Returns an error for blank labels, non-finite times, or inverted windows.
    pub fn new(label: impl Into<String>, start_s: f64, end_s: f64) -> Result<Self> {
        let label = require_non_empty("label", label.into())?;
        let start_s = require_finite("start_s", start_s)?;
        let end_s = require_finite("end_s", end_s)?;
        if start_s > end_s {
            return Err(SysidError::InvalidPhaseWindow { label });
        }
        Ok(Self {
            label,
            start_s,
            end_s,
        })
    }

    fn contains(&self, time_s: f64) -> bool {
        time_s >= self.start_s && time_s <= self.end_s
    }
}

/// One channel comparison request.
#[derive(Clone, Debug, PartialEq)]
pub struct ResidualChannel {
    /// Stable metric id.
    pub id: String,
    /// Reference or truth series.
    pub reference: ChannelSeries,
    /// Actual or candidate series.
    pub actual: ChannelSeries,
    /// Tolerance envelope.
    pub tolerance: ResidualTolerance,
}

impl ResidualChannel {
    /// Creates a validated channel comparison request.
    ///
    /// # Errors
    ///
    /// Returns an error for blank id or incompatible metadata.
    pub fn new(
        id: impl Into<String>,
        reference: ChannelSeries,
        actual: ChannelSeries,
        tolerance: ResidualTolerance,
    ) -> Result<Self> {
        let id = require_non_empty("id", id.into())?;
        reconcile_metadata(&id, &reference.descriptor, &actual.descriptor)?;
        Ok(Self {
            id,
            reference,
            actual,
            tolerance,
        })
    }
}

/// Full residual comparison request.
#[derive(Clone, Debug, PartialEq)]
pub struct ResidualComparison {
    /// Channels to compare, in report order.
    pub channels: Vec<ResidualChannel>,
    /// Phase windows to summarize for every channel.
    pub phases: Vec<PhaseSegment>,
}

impl ResidualComparison {
    /// Creates a validated comparison request.
    ///
    /// # Errors
    ///
    /// Returns an error for duplicate channel ids or phase labels.
    pub fn new(channels: Vec<ResidualChannel>, phases: Vec<PhaseSegment>) -> Result<Self> {
        let mut channel_ids = BTreeSet::new();
        for channel in &channels {
            if !channel_ids.insert(channel.id.clone()) {
                return Err(SysidError::DuplicateChannel {
                    channel: channel.id.clone(),
                });
            }
        }
        let mut phase_labels = BTreeSet::new();
        for phase in &phases {
            if !phase_labels.insert(phase.label.clone()) {
                return Err(SysidError::DuplicatePhase {
                    label: phase.label.clone(),
                });
            }
        }
        Ok(Self { channels, phases })
    }
}

/// Top-level residual comparison report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResidualReport {
    /// Number of compared channels.
    pub channel_count: usize,
    /// True when any channel or phase has no comparable samples or exceeds its
    /// declared tolerance envelope.
    pub any_out_of_family: bool,
    /// Per-channel reports in request order.
    pub channels: Vec<ChannelResidualReport>,
}

impl ResidualReport {
    /// Compact failure summary for command-line output.
    #[must_use]
    pub fn failure_summary(&self) -> String {
        let channels = self
            .channels
            .iter()
            .filter(|channel| channel.out_of_family)
            .count();
        let exceedances: usize = self
            .channels
            .iter()
            .map(|channel| channel.total.exceedances)
            .sum();
        let empty = self
            .channels
            .iter()
            .filter(|channel| channel.total.compared_samples == 0)
            .count();
        format!(
            "{channels} channel(s) out of family, {exceedances} sample exceedance(s), {empty} empty channel report(s)"
        )
    }
}

/// Per-channel residual report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChannelResidualReport {
    /// Stable metric id.
    pub id: String,
    /// Unit label that was reconciled before comparison.
    pub unit: String,
    /// Optional reconciled frame label.
    pub frame: Option<String>,
    /// Tolerance envelope used for all samples.
    pub tolerance: ResidualTolerance,
    /// Total statistics over the full reference series.
    pub total: ResidualStats,
    /// Statistics for every requested phase window.
    pub phases: Vec<PhaseResidualReport>,
    /// True when total or any phase has no comparable samples or sample
    /// exceedances.
    pub out_of_family: bool,
}

/// Per-phase residual report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhaseResidualReport {
    /// Phase label.
    pub label: String,
    /// Inclusive start time.
    pub start_s: f64,
    /// Inclusive end time.
    pub end_s: f64,
    /// Residual statistics in this window.
    pub stats: ResidualStats,
    /// True when the phase has no comparable samples or any sample exceedance.
    pub out_of_family: bool,
}

/// Residual statistics for a set of compared samples.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResidualStats {
    /// Samples with both reference and interpolated actual values.
    pub compared_samples: usize,
    /// Samples skipped because actual data did not cover the reference time.
    pub skipped_samples: usize,
    /// Mean signed error, actual minus reference.
    pub bias_error: f64,
    /// Mean absolute error.
    pub mean_abs_error: f64,
    /// Root-mean-square absolute error.
    pub rms_abs_error: f64,
    /// Maximum absolute error.
    pub max_abs_error: f64,
    /// Reference timestamp of the maximum absolute error.
    pub max_abs_error_time_s: Option<f64>,
    /// Reference value at the maximum absolute error.
    pub max_abs_error_reference: Option<f64>,
    /// Interpolated actual value at the maximum absolute error.
    pub max_abs_error_actual: Option<f64>,
    /// Compared samples outside the tolerance envelope.
    pub exceedances: usize,
}

/// Compares all requested channels.
///
/// # Errors
///
/// Returns an error when the request is internally inconsistent.
pub fn compare_residuals(request: &ResidualComparison) -> Result<ResidualReport> {
    let mut channel_reports = Vec::with_capacity(request.channels.len());
    for channel in &request.channels {
        channel_reports.push(compare_channel(channel, &request.phases));
    }
    let any_out_of_family = channel_reports.iter().any(|channel| channel.out_of_family);
    Ok(ResidualReport {
        channel_count: channel_reports.len(),
        any_out_of_family,
        channels: channel_reports,
    })
}

/// CSV channel declaration.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CsvChannelColumn {
    /// Metric id used in reports.
    pub id: String,
    /// CSV column name.
    pub column: String,
    /// Unit label.
    pub unit: String,
    /// Optional frame label.
    pub frame: Option<String>,
}

impl CsvChannelColumn {
    /// Creates a validated CSV channel declaration.
    ///
    /// # Errors
    ///
    /// Returns [`SysidError::EmptyField`] for blank id, column, unit, or frame.
    pub fn new(
        id: impl Into<String>,
        column: impl Into<String>,
        unit: impl Into<String>,
        frame: Option<impl Into<String>>,
    ) -> Result<Self> {
        Ok(Self {
            id: require_non_empty("id", id.into())?,
            column: require_non_empty("column", column.into())?,
            unit: require_non_empty("unit", unit.into())?,
            frame: match frame {
                Some(value) => Some(require_non_empty("frame", value.into())?),
                None => None,
            },
        })
    }
}

/// CSV series loading declaration.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CsvSeriesSpec {
    /// CSV time column in seconds.
    pub time_column: String,
    /// Channels to load from the CSV.
    pub channels: Vec<CsvChannelColumn>,
}

impl CsvSeriesSpec {
    /// Creates a validated CSV loading declaration.
    ///
    /// # Errors
    ///
    /// Returns [`SysidError::EmptyField`] for a blank time column.
    pub fn new(time_column: impl Into<String>, channels: Vec<CsvChannelColumn>) -> Result<Self> {
        Ok(Self {
            time_column: require_non_empty("time_column", time_column.into())?,
            channels,
        })
    }
}

/// Reads declared scalar time-series channels from a CSV file.
///
/// # Errors
///
/// Returns an error for missing columns, CSV parser failures, non-finite
/// numeric values, and invalid series ordering.
pub fn read_csv_series(path: impl AsRef<Path>, spec: &CsvSeriesSpec) -> Result<Vec<ChannelSeries>> {
    let path = path.as_ref();
    let mut reader = csv::Reader::from_path(path).map_err(|source| SysidError::Csv {
        path: path.to_path_buf(),
        source,
    })?;
    let headers = reader
        .headers()
        .map_err(|source| SysidError::Csv {
            path: path.to_path_buf(),
            source,
        })?
        .clone();
    let time_index = csv_column_index(path, &headers, &spec.time_column)?;
    let mut channel_indices = Vec::with_capacity(spec.channels.len());
    for channel in &spec.channels {
        channel_indices.push(csv_column_index(path, &headers, &channel.column)?);
    }

    let mut samples: Vec<Vec<TimeSample>> = vec![Vec::new(); spec.channels.len()];
    for (row_index, row) in reader.records().enumerate() {
        let row = row.map_err(|source| SysidError::Csv {
            path: path.to_path_buf(),
            source,
        })?;
        let time_s = parse_csv_f64(path, &spec.time_column, row_index + 1, &row[time_index])?;
        for (channel_index, channel) in spec.channels.iter().enumerate() {
            let value = parse_csv_f64(
                path,
                &channel.column,
                row_index + 1,
                &row[channel_indices[channel_index]],
            )?;
            samples[channel_index].push(TimeSample::new(time_s, value)?);
        }
    }

    let mut series = Vec::with_capacity(spec.channels.len());
    for (channel, channel_samples) in spec.channels.iter().zip(samples) {
        let descriptor =
            ChannelDescriptor::new(&channel.id, &channel.unit, channel.frame.as_deref())?;
        series.push(ChannelSeries::new(descriptor, channel_samples)?);
    }
    Ok(series)
}

/// Converts series into a deterministic id-keyed map.
///
/// # Errors
///
/// Returns an error when the same channel id appears twice.
pub fn series_by_id(series: Vec<ChannelSeries>) -> Result<BTreeMap<String, ChannelSeries>> {
    let mut by_id = BTreeMap::new();
    for item in series {
        let id = item.descriptor.name.clone();
        if by_id.insert(id.clone(), item).is_some() {
            return Err(SysidError::DuplicateChannel { channel: id });
        }
    }
    Ok(by_id)
}

fn compare_channel(channel: &ResidualChannel, phases: &[PhaseSegment]) -> ChannelResidualReport {
    let mut total = StatsAccumulator::default();
    let mut phase_accumulators: Vec<StatsAccumulator> =
        phases.iter().map(|_| StatsAccumulator::default()).collect();

    for reference in &channel.reference.samples {
        let actual = channel.actual.interpolate(reference.time_s);
        total.record_sample(reference.time_s, reference.value, actual, channel.tolerance);
        for (phase, accumulator) in phases.iter().zip(&mut phase_accumulators) {
            if phase.contains(reference.time_s) {
                accumulator.record_sample(
                    reference.time_s,
                    reference.value,
                    actual,
                    channel.tolerance,
                );
            }
        }
    }

    let total = total.finish();
    let phase_reports: Vec<PhaseResidualReport> = phases
        .iter()
        .zip(phase_accumulators)
        .map(|(phase, accumulator)| {
            let stats = accumulator.finish();
            let out_of_family = stats.compared_samples == 0 || stats.exceedances > 0;
            PhaseResidualReport {
                label: phase.label.clone(),
                start_s: phase.start_s,
                end_s: phase.end_s,
                stats,
                out_of_family,
            }
        })
        .collect();
    let out_of_family = total.compared_samples == 0
        || total.exceedances > 0
        || phase_reports.iter().any(|phase| phase.out_of_family);

    ChannelResidualReport {
        id: channel.id.clone(),
        unit: channel.reference.descriptor.unit.clone(),
        frame: channel.reference.descriptor.frame.clone(),
        tolerance: channel.tolerance,
        total,
        phases: phase_reports,
        out_of_family,
    }
}

fn reconcile_metadata(
    id: &str,
    reference: &ChannelDescriptor,
    actual: &ChannelDescriptor,
) -> Result<()> {
    if reference.unit != actual.unit {
        return Err(SysidError::MetadataMismatch {
            channel: id.to_owned(),
            field: "unit",
            reference: Some(reference.unit.clone()),
            actual: Some(actual.unit.clone()),
        });
    }
    if reference.frame != actual.frame {
        return Err(SysidError::MetadataMismatch {
            channel: id.to_owned(),
            field: "frame",
            reference: reference.frame.clone(),
            actual: actual.frame.clone(),
        });
    }
    Ok(())
}

#[derive(Default)]
struct StatsAccumulator {
    compared_samples: usize,
    skipped_samples: usize,
    sum_error: f64,
    sum_abs_error: f64,
    sum_sq_error: f64,
    max_abs_error: f64,
    max_abs_error_time_s: Option<f64>,
    max_abs_error_reference: Option<f64>,
    max_abs_error_actual: Option<f64>,
    exceedances: usize,
}

impl StatsAccumulator {
    fn record_sample(
        &mut self,
        time_s: f64,
        reference: f64,
        actual: Option<f64>,
        tolerance: ResidualTolerance,
    ) {
        let Some(actual) = actual else {
            self.skipped_samples += 1;
            return;
        };
        let error = actual - reference;
        let abs_error = error.abs();
        self.compared_samples += 1;
        self.sum_error += error;
        self.sum_abs_error += abs_error;
        self.sum_sq_error += abs_error * abs_error;
        if self.max_abs_error_time_s.is_none() || abs_error >= self.max_abs_error {
            self.max_abs_error = abs_error;
            self.max_abs_error_time_s = Some(time_s);
            self.max_abs_error_reference = Some(reference);
            self.max_abs_error_actual = Some(actual);
        }
        if abs_error > tolerance.limit_for(reference) {
            self.exceedances += 1;
        }
    }

    fn finish(self) -> ResidualStats {
        if self.compared_samples == 0 {
            return ResidualStats {
                compared_samples: 0,
                skipped_samples: self.skipped_samples,
                bias_error: 0.0,
                mean_abs_error: 0.0,
                rms_abs_error: 0.0,
                max_abs_error: 0.0,
                max_abs_error_time_s: None,
                max_abs_error_reference: None,
                max_abs_error_actual: None,
                exceedances: self.exceedances,
            };
        }
        #[allow(clippy::cast_precision_loss)]
        let n = self.compared_samples as f64;
        ResidualStats {
            compared_samples: self.compared_samples,
            skipped_samples: self.skipped_samples,
            bias_error: self.sum_error / n,
            mean_abs_error: self.sum_abs_error / n,
            rms_abs_error: (self.sum_sq_error / n).sqrt(),
            max_abs_error: self.max_abs_error,
            max_abs_error_time_s: self.max_abs_error_time_s,
            max_abs_error_reference: self.max_abs_error_reference,
            max_abs_error_actual: self.max_abs_error_actual,
            exceedances: self.exceedances,
        }
    }
}

fn csv_column_index(path: &Path, headers: &csv::StringRecord, column: &str) -> Result<usize> {
    headers
        .iter()
        .position(|header| header == column)
        .ok_or_else(|| SysidError::MissingCsvColumn {
            path: path.to_path_buf(),
            column: column.to_owned(),
        })
}

fn parse_csv_f64(path: &Path, column: &str, row_index: usize, value: &str) -> Result<f64> {
    let parsed = value.parse::<f64>().map_err(|_| SysidError::CsvValue {
        path: path.to_path_buf(),
        column: column.to_owned(),
        row_index,
        value: value.to_owned(),
    })?;
    if !parsed.is_finite() {
        return Err(SysidError::CsvValue {
            path: path.to_path_buf(),
            column: column.to_owned(),
            row_index,
            value: value.to_owned(),
        });
    }
    Ok(parsed)
}

fn require_non_empty(field: &'static str, value: String) -> Result<String> {
    if value.trim().is_empty() {
        Err(SysidError::EmptyField { field })
    } else {
        Ok(value)
    }
}

fn require_finite(field: &'static str, value: f64) -> Result<f64> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(SysidError::NonFinite { field, value })
    }
}

fn require_non_negative(field: &'static str, value: f64) -> Result<f64> {
    let value = require_finite(field, value)?;
    if value < 0.0 {
        Err(SysidError::NegativeTolerance { field, value })
    } else {
        Ok(value)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    fn series(id: &str, unit: &str, frame: Option<&str>, samples: &[(f64, f64)]) -> ChannelSeries {
        ChannelSeries::new(
            ChannelDescriptor::new(id, unit, frame).unwrap(),
            samples
                .iter()
                .map(|(time_s, value)| TimeSample::new(*time_s, *value).unwrap())
                .collect(),
        )
        .unwrap()
    }

    #[test]
    fn registration_and_resampling_are_exact_on_constructed_linear_signal() {
        let reference = series(
            "altitude",
            "m",
            Some("ECI"),
            &[(0.25, 1.5), (0.75, 2.5), (1.25, 3.5)],
        );
        let actual = series(
            "altitude",
            "m",
            Some("ECI"),
            &[(0.0, 1.0), (1.0, 3.0), (2.0, 5.0)],
        );
        let channel = ResidualChannel::new(
            "altitude",
            reference,
            actual,
            ResidualTolerance::new(1.0e-12, 0.0, 1.0).unwrap(),
        )
        .unwrap();
        let report =
            compare_residuals(&ResidualComparison::new(vec![channel], vec![]).unwrap()).unwrap();

        let stats = &report.channels[0].total;
        assert_eq!(stats.compared_samples, 3);
        assert_eq!(stats.skipped_samples, 0);
        assert_abs_diff_eq!(stats.max_abs_error, 0.0, epsilon = 1.0e-15);
        assert!(!report.any_out_of_family);
    }

    #[test]
    fn residual_statistics_match_hand_computed_fixture() {
        let reference = series("qbar", "Pa", None, &[(0.0, 10.0), (1.0, 20.0), (2.0, 30.0)]);
        let actual = series("qbar", "Pa", None, &[(0.0, 11.0), (1.0, 18.0), (2.0, 33.0)]);
        let channel = ResidualChannel::new(
            "qbar",
            reference,
            actual,
            ResidualTolerance::new(2.5, 0.0, 1.0).unwrap(),
        )
        .unwrap();
        let report =
            compare_residuals(&ResidualComparison::new(vec![channel], vec![]).unwrap()).unwrap();

        let stats = &report.channels[0].total;
        assert_eq!(stats.compared_samples, 3);
        assert_eq!(stats.exceedances, 1);
        assert_abs_diff_eq!(stats.bias_error, 2.0 / 3.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(stats.mean_abs_error, 2.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(
            stats.rms_abs_error,
            (14.0_f64 / 3.0).sqrt(),
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(stats.max_abs_error, 3.0, epsilon = 1.0e-15);
        assert_eq!(stats.max_abs_error_time_s, Some(2.0));
        assert!(report.any_out_of_family);
    }

    #[test]
    fn phase_segments_report_independent_out_of_family_status() {
        let reference = series(
            "velocity",
            "m/s",
            Some("ECI"),
            &[(0.0, 0.0), (1.0, 10.0), (2.0, 20.0), (3.0, 30.0)],
        );
        let actual = series(
            "velocity",
            "m/s",
            Some("ECI"),
            &[(0.0, 0.0), (1.0, 10.0), (2.0, 30.0), (3.0, 30.0)],
        );
        let channel = ResidualChannel::new(
            "velocity",
            reference,
            actual,
            ResidualTolerance::new(1.0, 0.0, 1.0).unwrap(),
        )
        .unwrap();
        let phases = vec![
            PhaseSegment::new("early", 0.0, 1.0).unwrap(),
            PhaseSegment::new("late", 2.0, 3.0).unwrap(),
        ];
        let report =
            compare_residuals(&ResidualComparison::new(vec![channel], phases).unwrap()).unwrap();

        assert!(!report.channels[0].phases[0].out_of_family);
        assert!(report.channels[0].phases[1].out_of_family);
        assert_eq!(report.channels[0].phases[1].stats.exceedances, 1);
    }

    #[test]
    fn mismatched_units_and_frames_fail_closed() {
        let reference = series("range", "m", Some("ECI"), &[(0.0, 1.0)]);
        let actual_unit = series("range", "km", Some("ECI"), &[(0.0, 1.0)]);
        let err = ResidualChannel::new(
            "range",
            reference.clone(),
            actual_unit,
            ResidualTolerance::new(1.0, 0.0, 1.0).unwrap(),
        )
        .unwrap_err();
        assert!(matches!(
            err,
            SysidError::MetadataMismatch { field: "unit", .. }
        ));

        let actual_frame = series("range", "m", Some("Body"), &[(0.0, 1.0)]);
        let err = ResidualChannel::new(
            "range",
            reference,
            actual_frame,
            ResidualTolerance::new(1.0, 0.0, 1.0).unwrap(),
        )
        .unwrap_err();
        assert!(matches!(
            err,
            SysidError::MetadataMismatch { field: "frame", .. }
        ));
    }

    #[test]
    fn invalid_time_series_inputs_fail_closed() {
        let err = ChannelSeries::new(
            ChannelDescriptor::new("bad", "m", None::<&str>).unwrap(),
            vec![
                TimeSample::new(1.0, 0.0).unwrap(),
                TimeSample::new(1.0, 1.0).unwrap(),
            ],
        )
        .unwrap_err();
        assert!(matches!(err, SysidError::NonIncreasingTime { .. }));

        let err = PhaseSegment::new("bad", 2.0, 1.0).unwrap_err();
        assert!(matches!(err, SysidError::InvalidPhaseWindow { .. }));
    }

    #[test]
    fn csv_loader_builds_declared_series() {
        let temp = tempfile::NamedTempFile::new().unwrap();
        std::fs::write(
            temp.path(),
            "time_s,altitude_m,velocity_m_s\n0,10,3\n1,12,4\n2,14,5\n",
        )
        .unwrap();
        let spec = CsvSeriesSpec::new(
            "time_s",
            vec![
                CsvChannelColumn::new("altitude", "altitude_m", "m", Some("ECI")).unwrap(),
                CsvChannelColumn::new("velocity", "velocity_m_s", "m/s", Some("ECI")).unwrap(),
            ],
        )
        .unwrap();

        let series = read_csv_series(temp.path(), &spec).unwrap();
        assert_eq!(series.len(), 2);
        assert_eq!(series[0].descriptor.name, "altitude");
        assert_abs_diff_eq!(series[0].samples[2].value, 14.0, epsilon = 0.0);
        assert_eq!(series[1].descriptor.unit, "m/s");
    }
}
