//! Deterministic trend histories, drift gates, and scoreboard generation.
//!
//! The trend layer consumes committed run records and produces plain file
//! artifacts. It intentionally has no service, database, randomness, wall-clock
//! dependency, or live CI coupling: callers opt in by declaring metrics and
//! feeding an append-only run index.

use std::collections::btree_map::Entry;
use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::verdict::ChannelMetric;
use crate::{ChannelDigest, RunIndex, RunRecord, RundataError, markdown_cell, require_non_empty};

/// Trend artifact schema version.
pub const TREND_SCHEMA_VERSION: &str = "openbmp.rundata.trend.v1";

/// Default label key used to partition run records into scenario families.
pub const FAMILY_LABEL_KEY: &str = "family";

/// Default label key used to key run records by code-id ordinal.
pub const CODE_ORDINAL_LABEL_KEY: &str = "code_ordinal";

/// Errors returned by trend-store helpers.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum TrendError {
    /// JSON serialization or parsing failed.
    #[error("trend JSON error: {message}")]
    Json {
        /// Error message.
        message: String,
    },
    /// JSONL run index parsing failed.
    #[error("trend run-index error: {message}")]
    Index {
        /// Error message.
        message: String,
    },
    /// A required string field was empty after trimming.
    #[error("{field} must not be empty")]
    EmptyField {
        /// Field name.
        field: &'static str,
    },
    /// At least one trend specification is required.
    #[error("trend generation requires at least one metric spec")]
    EmptySpecs,
    /// Trend specification ids must be unique.
    #[error("duplicate trend metric spec id {id}")]
    DuplicateSpecId {
        /// Duplicate spec id.
        id: String,
    },
    /// A detector parameter was invalid.
    #[error("invalid detector for {spec_id}: {field}")]
    InvalidDetector {
        /// Metric spec id.
        spec_id: String,
        /// Invalid field.
        field: &'static str,
    },
    /// A matching run record had no code-ordinal label.
    #[error("run {run_id} is missing code_ordinal label")]
    MissingCodeOrdinal {
        /// Run id.
        run_id: String,
    },
    /// A matching run record had a non-u64 code-ordinal label.
    #[error("run {run_id} has invalid code_ordinal label {value}")]
    InvalidCodeOrdinal {
        /// Run id.
        run_id: String,
        /// Invalid label value.
        value: String,
    },
    /// Two records disagree about the code id for the same ordinal.
    #[error("ordinal {ordinal} has conflicting code ids {first} and {second}")]
    ConflictingCodeId {
        /// Code-id ordinal.
        ordinal: u64,
        /// First code id observed.
        first: String,
        /// Conflicting code id.
        second: String,
    },
    /// A metric spec matched no records.
    #[error("trend metric spec {spec_id} matched no records")]
    EmptySeries {
        /// Metric spec id.
        spec_id: String,
    },
    /// A record did not contain the requested numeric metric.
    #[error("run {run_id} does not contain numeric metric {channel}.{metric:?}")]
    MissingMetric {
        /// Run id.
        run_id: String,
        /// Channel name.
        channel: String,
        /// Requested metric.
        metric: ChannelMetric,
    },
}

impl From<serde_json::Error> for TrendError {
    fn from(error: serde_json::Error) -> Self {
        Self::Json {
            message: error.to_string(),
        }
    }
}

impl From<RundataError> for TrendError {
    fn from(error: RundataError) -> Self {
        Self::Index {
            message: error.to_string(),
        }
    }
}

/// Declared metric to track across code-id ordinals.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TrendMetricSpec {
    /// Stable spec id used by reports and CI gates.
    pub id: String,
    /// Stable scenario/campaign family id.
    pub family_id: String,
    /// Telemetry channel name.
    pub channel: String,
    /// Numeric digest metric.
    pub metric: ChannelMetric,
    /// Deterministic detector configuration.
    pub detector: DriftDetectorConfig,
    /// Whether a drift result should fail the CI gate helper.
    pub fail_on_drift: bool,
}

impl TrendMetricSpec {
    /// Construct a validated trend metric specification.
    ///
    /// # Errors
    ///
    /// Returns [`TrendError`] when a required string is empty or a detector
    /// parameter is invalid.
    pub fn new(
        id: impl Into<String>,
        family_id: impl Into<String>,
        channel: impl Into<String>,
        metric: ChannelMetric,
        detector: DriftDetectorConfig,
        fail_on_drift: bool,
    ) -> Result<Self, TrendError> {
        let spec = Self {
            id: require_non_empty("trend.id", id.into())
                .map_err(|_| TrendError::EmptyField { field: "trend.id" })?,
            family_id: require_non_empty("trend.family_id", family_id.into()).map_err(|_| {
                TrendError::EmptyField {
                    field: "trend.family_id",
                }
            })?,
            channel: require_non_empty("trend.channel", channel.into()).map_err(|_| {
                TrendError::EmptyField {
                    field: "trend.channel",
                }
            })?,
            metric,
            detector,
            fail_on_drift,
        };
        spec.validate()?;
        Ok(spec)
    }

    fn validate(&self) -> Result<(), TrendError> {
        self.detector.validate(&self.id)
    }
}

/// Deterministic drift detector configuration.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum DriftDetectorConfig {
    /// One-sided or two-sided Page-Hinkley-class detector.
    PageHinkley {
        /// Minimum number of points before a hit can be reported.
        warmup: u64,
        /// Small drift allowance subtracted from each increment.
        delta: f64,
        /// Positive detection threshold.
        lambda: f64,
        /// Cumulative-score memory in `[0, 1]`.
        alpha: f64,
        /// Direction to detect.
        direction: DriftDirection,
    },
}

impl DriftDetectorConfig {
    fn validate(&self, spec_id: &str) -> Result<(), TrendError> {
        match self {
            Self::PageHinkley {
                warmup,
                delta,
                lambda,
                alpha,
                ..
            } => {
                if *warmup == 0 {
                    return invalid_detector(spec_id, "warmup");
                }
                if !delta.is_finite() || *delta < 0.0 {
                    return invalid_detector(spec_id, "delta");
                }
                if !lambda.is_finite() || *lambda <= 0.0 {
                    return invalid_detector(spec_id, "lambda");
                }
                if !alpha.is_finite() || !(0.0..=1.0).contains(alpha) {
                    return invalid_detector(spec_id, "alpha");
                }
                Ok(())
            }
        }
    }
}

/// Page-Hinkley detector direction.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DriftDirection {
    /// Detect upward drift.
    Upward,
    /// Detect downward drift.
    Downward,
    /// Detect either upward or downward drift.
    Both,
}

/// One aggregated trend point for a code-id ordinal.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TrendPoint {
    /// Code-id ordinal.
    pub ordinal: u64,
    /// Stable code/config id for this ordinal.
    pub code_id: String,
    /// Deterministic mean of all matching run metrics at this ordinal.
    pub value: f64,
    /// Source run ids included in the aggregate, sorted lexicographically.
    pub source_run_ids: Vec<String>,
}

/// Per-metric history over code-id ordinals.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TrendSeries {
    /// Stable spec id used by reports and CI gates.
    pub spec_id: String,
    /// Stable scenario/campaign family id.
    pub family_id: String,
    /// Telemetry channel name.
    pub channel: String,
    /// Numeric digest metric.
    pub metric: ChannelMetric,
    /// Detector configuration.
    pub detector: DriftDetectorConfig,
    /// Whether a drift result should fail the CI gate helper.
    pub fail_on_drift: bool,
    /// Aggregated points in increasing code-id ordinal order.
    pub points: Vec<TrendPoint>,
}

/// Deterministic trend-store artifact.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TrendStore {
    /// Artifact schema version.
    pub schema_version: String,
    /// Series in deterministic spec order.
    pub series: Vec<TrendSeries>,
}

impl TrendStore {
    /// Serialize as deterministic JSON bytes.
    ///
    /// # Errors
    ///
    /// Returns [`TrendError::Json`] if serialization fails.
    pub fn to_json_bytes(&self) -> Result<Vec<u8>, TrendError> {
        Ok(serde_json::to_vec(self)?)
    }

    /// Parse a JSON trend-store artifact.
    ///
    /// # Errors
    ///
    /// Returns [`TrendError::Json`] if parsing fails.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, TrendError> {
        Ok(serde_json::from_slice(bytes)?)
    }
}

/// Drift evaluation status.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DriftStatus {
    /// Not enough points have been accumulated to evaluate the detector.
    InsufficientData,
    /// Detector did not cross its threshold.
    Nominal,
    /// Detector crossed its threshold.
    Drift,
}

/// Drift evaluation for one trend series.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DriftReport {
    /// Stable spec id.
    pub spec_id: String,
    /// Stable scenario/campaign family id.
    pub family_id: String,
    /// Telemetry channel name.
    pub channel: String,
    /// Numeric digest metric.
    pub metric: ChannelMetric,
    /// Whether a drift result should fail the CI gate helper.
    pub fail_on_drift: bool,
    /// Evaluation status.
    pub status: DriftStatus,
    /// Number of trend points evaluated.
    pub point_count: u64,
    /// Last code-id ordinal in the series.
    pub last_ordinal: u64,
    /// Last code/config id in the series.
    pub last_code_id: String,
    /// Last aggregate metric value.
    pub last_value: f64,
    /// First drift ordinal, when detected.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub first_drift_ordinal: Option<u64>,
    /// First drift code id, when detected.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub first_drift_code_id: Option<String>,
    /// Final or triggering detector score.
    pub detector_score: f64,
    /// Stable human-readable detail for scoreboard and CI output.
    pub detail: String,
}

/// Generate a deterministic trend-store artifact from run records.
///
/// # Errors
///
/// Returns [`TrendError`] when specs are empty/invalid, matching records lack
/// a valid code ordinal, or any requested metric is unavailable.
pub fn generate_trend_store(
    records: &[RunRecord],
    specs: &[TrendMetricSpec],
) -> Result<TrendStore, TrendError> {
    if specs.is_empty() {
        return Err(TrendError::EmptySpecs);
    }
    validate_specs(specs)?;

    let mut sorted_specs = specs.to_vec();
    sorted_specs.sort_by(|a, b| {
        a.family_id
            .cmp(&b.family_id)
            .then_with(|| a.channel.cmp(&b.channel))
            .then_with(|| a.metric.cmp(&b.metric))
            .then_with(|| a.id.cmp(&b.id))
    });

    let mut series = Vec::with_capacity(sorted_specs.len());
    for spec in sorted_specs {
        series.push(generate_series(records, &spec)?);
    }

    Ok(TrendStore {
        schema_version: TREND_SCHEMA_VERSION.to_owned(),
        series,
    })
}

/// Generate a deterministic trend-store artifact from an append-only run
/// index.
///
/// # Errors
///
/// Returns [`TrendError`] when the index cannot be evaluated.
pub fn generate_trend_store_from_index(
    index: &RunIndex,
    specs: &[TrendMetricSpec],
) -> Result<TrendStore, TrendError> {
    generate_trend_store(index.records(), specs)
}

/// Parse a JSONL run index and generate a deterministic trend-store artifact.
///
/// # Errors
///
/// Returns [`TrendError`] when JSONL parsing or trend generation fails.
pub fn generate_trend_store_from_jsonl_index(
    jsonl: &[u8],
    specs: &[TrendMetricSpec],
) -> Result<TrendStore, TrendError> {
    let index = RunIndex::from_jsonl_bytes(jsonl)?;
    generate_trend_store_from_index(&index, specs)
}

/// Evaluate every series in a trend-store artifact.
#[must_use]
pub fn evaluate_trend_store(store: &TrendStore) -> Vec<DriftReport> {
    let mut reports = store.series.iter().map(evaluate_series).collect::<Vec<_>>();
    reports.sort_by(report_order);
    reports
}

/// Generate a byte-stable Markdown scoreboard from drift reports.
#[must_use]
pub fn generate_scoreboard_markdown(reports: &[DriftReport]) -> String {
    let mut reports = reports.to_vec();
    reports.sort_by(report_order);

    let mut markdown = String::from("# OpenBMP Run-Data Trend Scoreboard\n\n");
    markdown.push_str(
        "| family | metric | status | points | last ordinal | last code id | last value | drift ordinal | gate | detail |\n",
    );
    markdown.push_str("|---|---|---|---:|---:|---|---:|---:|---|---|\n");
    for report in reports {
        let metric = format!("{}.{}", report.channel, metric_name(report.metric));
        let drift_ordinal = report
            .first_drift_ordinal
            .map_or_else(|| "-".to_owned(), |ordinal| ordinal.to_string());
        let gate = if report.fail_on_drift {
            "fail_on_drift"
        } else {
            "observe"
        };
        markdown.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |\n",
            markdown_cell(&report.family_id),
            markdown_cell(&metric),
            status_name(report.status),
            report.point_count,
            report.last_ordinal,
            markdown_cell(&report.last_code_id),
            format_f64(report.last_value),
            drift_ordinal,
            gate,
            markdown_cell(&report.detail),
        ));
    }
    markdown
}

/// Return stable CI failure strings for drift reports whose metric spec opted
/// into fail-on-drift.
#[must_use]
pub fn ci_gate_failures(reports: &[DriftReport]) -> Vec<String> {
    let mut failures = reports
        .iter()
        .filter(|report| report.fail_on_drift && report.status == DriftStatus::Drift)
        .map(|report| {
            format!(
                "{} {}.{} drifted at ordinal {}: {}",
                report.family_id,
                report.channel,
                metric_name(report.metric),
                report
                    .first_drift_ordinal
                    .map_or_else(|| "-".to_owned(), |ordinal| ordinal.to_string()),
                report.detail
            )
        })
        .collect::<Vec<_>>();
    failures.sort();
    failures
}

fn validate_specs(specs: &[TrendMetricSpec]) -> Result<(), TrendError> {
    let mut ids = BTreeSet::new();
    for spec in specs {
        let id = require_non_empty("trend.id", spec.id.clone())
            .map_err(|_| TrendError::EmptyField { field: "trend.id" })?;
        require_non_empty("trend.family_id", spec.family_id.clone()).map_err(|_| {
            TrendError::EmptyField {
                field: "trend.family_id",
            }
        })?;
        require_non_empty("trend.channel", spec.channel.clone()).map_err(|_| {
            TrendError::EmptyField {
                field: "trend.channel",
            }
        })?;
        if !ids.insert(id.clone()) {
            return Err(TrendError::DuplicateSpecId { id });
        }
        spec.validate()?;
    }
    Ok(())
}

fn generate_series(
    records: &[RunRecord],
    spec: &TrendMetricSpec,
) -> Result<TrendSeries, TrendError> {
    let mut buckets = BTreeMap::<u64, OrdinalBucket>::new();
    for record in records {
        if label_value(record, FAMILY_LABEL_KEY) != Some(spec.family_id.as_str()) {
            continue;
        }
        let ordinal = code_ordinal(record)?;
        let value = record_metric(record, &spec.channel, spec.metric)?;
        match buckets.entry(ordinal) {
            Entry::Vacant(entry) => {
                entry.insert(OrdinalBucket {
                    code_id: record.code_id.clone(),
                    observations: vec![(record.run_id.clone(), value)],
                });
            }
            Entry::Occupied(mut entry) => {
                let bucket = entry.get_mut();
                if bucket.code_id != record.code_id {
                    return Err(TrendError::ConflictingCodeId {
                        ordinal,
                        first: bucket.code_id.clone(),
                        second: record.code_id.clone(),
                    });
                }
                bucket.observations.push((record.run_id.clone(), value));
            }
        }
    }

    if buckets.is_empty() {
        return Err(TrendError::EmptySeries {
            spec_id: spec.id.clone(),
        });
    }

    let points = buckets
        .into_iter()
        .map(|(ordinal, mut bucket)| {
            bucket
                .observations
                .sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
            let sum = bucket
                .observations
                .iter()
                .map(|(_, value)| *value)
                .sum::<f64>();
            let source_run_ids = bucket
                .observations
                .iter()
                .map(|(run_id, _)| run_id.clone())
                .collect::<Vec<_>>();
            TrendPoint {
                ordinal,
                code_id: bucket.code_id,
                value: sum / source_run_ids.len() as f64,
                source_run_ids,
            }
        })
        .collect::<Vec<_>>();

    Ok(TrendSeries {
        spec_id: spec.id.clone(),
        family_id: spec.family_id.clone(),
        channel: spec.channel.clone(),
        metric: spec.metric,
        detector: spec.detector,
        fail_on_drift: spec.fail_on_drift,
        points,
    })
}

#[derive(Debug)]
struct OrdinalBucket {
    code_id: String,
    observations: Vec<(String, f64)>,
}

fn evaluate_series(series: &TrendSeries) -> DriftReport {
    let last = series.points.last();
    let (last_ordinal, last_code_id, last_value) = last.map_or_else(
        || (0, String::new(), f64::NAN),
        |point| (point.ordinal, point.code_id.clone(), point.value),
    );
    match series.detector {
        DriftDetectorConfig::PageHinkley {
            warmup,
            delta,
            lambda,
            alpha,
            direction,
        } => {
            if series.points.len() < warmup as usize {
                return DriftReport {
                    spec_id: series.spec_id.clone(),
                    family_id: series.family_id.clone(),
                    channel: series.channel.clone(),
                    metric: series.metric,
                    fail_on_drift: series.fail_on_drift,
                    status: DriftStatus::InsufficientData,
                    point_count: series.points.len() as u64,
                    last_ordinal,
                    last_code_id,
                    last_value,
                    first_drift_ordinal: None,
                    first_drift_code_id: None,
                    detector_score: 0.0,
                    detail: format!(
                        "insufficient points: {} < warmup {warmup}",
                        series.points.len()
                    ),
                };
            }
            let outcome = page_hinkley(&series.points, warmup, delta, lambda, alpha, direction);
            let (status, first_drift_ordinal, first_drift_code_id, detail) =
                if let Some(hit) = outcome.hit {
                    (
                        DriftStatus::Drift,
                        Some(hit.ordinal),
                        Some(hit.code_id),
                        format!(
                            "page_hinkley_{} score {} exceeded lambda {}",
                            hit.direction,
                            format_f64(hit.score),
                            format_f64(lambda),
                        ),
                    )
                } else {
                    (
                        DriftStatus::Nominal,
                        None,
                        None,
                        format!(
                            "page_hinkley max_score {} <= lambda {}",
                            format_f64(outcome.score),
                            format_f64(lambda),
                        ),
                    )
                };
            DriftReport {
                spec_id: series.spec_id.clone(),
                family_id: series.family_id.clone(),
                channel: series.channel.clone(),
                metric: series.metric,
                fail_on_drift: series.fail_on_drift,
                status,
                point_count: series.points.len() as u64,
                last_ordinal,
                last_code_id,
                last_value,
                first_drift_ordinal,
                first_drift_code_id,
                detector_score: outcome.score,
                detail,
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct PageHinkleyOutcome {
    hit: Option<PageHinkleyHit>,
    score: f64,
}

#[derive(Clone, Debug, PartialEq)]
struct PageHinkleyHit {
    ordinal: u64,
    code_id: String,
    score: f64,
    direction: &'static str,
}

fn page_hinkley(
    points: &[TrendPoint],
    warmup: u64,
    delta: f64,
    lambda: f64,
    alpha: f64,
    direction: DriftDirection,
) -> PageHinkleyOutcome {
    match direction {
        DriftDirection::Upward => {
            page_hinkley_signed(points, warmup, delta, lambda, alpha, 1.0, "upward")
        }
        DriftDirection::Downward => {
            page_hinkley_signed(points, warmup, delta, lambda, alpha, -1.0, "downward")
        }
        DriftDirection::Both => {
            let upward = page_hinkley_signed(points, warmup, delta, lambda, alpha, 1.0, "upward");
            let downward =
                page_hinkley_signed(points, warmup, delta, lambda, alpha, -1.0, "downward");
            match (upward.hit, downward.hit) {
                (Some(upward_hit), Some(downward_hit)) => {
                    if upward_hit.ordinal <= downward_hit.ordinal {
                        PageHinkleyOutcome {
                            hit: Some(upward_hit),
                            score: upward.score.max(downward.score),
                        }
                    } else {
                        PageHinkleyOutcome {
                            hit: Some(downward_hit),
                            score: upward.score.max(downward.score),
                        }
                    }
                }
                (Some(hit), None) | (None, Some(hit)) => PageHinkleyOutcome {
                    hit: Some(hit),
                    score: upward.score.max(downward.score),
                },
                (None, None) => PageHinkleyOutcome {
                    hit: None,
                    score: upward.score.max(downward.score),
                },
            }
        }
    }
}

fn page_hinkley_signed(
    points: &[TrendPoint],
    warmup: u64,
    delta: f64,
    lambda: f64,
    alpha: f64,
    sign: f64,
    direction: &'static str,
) -> PageHinkleyOutcome {
    let mut mean = 0.0;
    let mut cumulative = 0.0;
    let mut min_cumulative = 0.0;
    let mut max_score = 0.0;
    for (index, point) in points.iter().enumerate() {
        let value = sign * point.value;
        let count = index as f64 + 1.0;
        mean += (value - mean) / count;
        cumulative = alpha * cumulative + value - mean - delta;
        min_cumulative = f64::min(min_cumulative, cumulative);
        let score = cumulative - min_cumulative;
        max_score = f64::max(max_score, score);
        if index + 1 >= warmup as usize && score > lambda {
            return PageHinkleyOutcome {
                hit: Some(PageHinkleyHit {
                    ordinal: point.ordinal,
                    code_id: point.code_id.clone(),
                    score,
                    direction,
                }),
                score,
            };
        }
    }
    PageHinkleyOutcome {
        hit: None,
        score: max_score,
    }
}

fn code_ordinal(record: &RunRecord) -> Result<u64, TrendError> {
    let Some(value) = label_value(record, CODE_ORDINAL_LABEL_KEY) else {
        return Err(TrendError::MissingCodeOrdinal {
            run_id: record.run_id.clone(),
        });
    };
    value
        .parse::<u64>()
        .map_err(|_| TrendError::InvalidCodeOrdinal {
            run_id: record.run_id.clone(),
            value: value.to_owned(),
        })
}

fn label_value<'a>(record: &'a RunRecord, key: &str) -> Option<&'a str> {
    record
        .labels
        .iter()
        .find(|label| label.key == key)
        .map(|label| label.value.as_str())
}

fn record_metric(
    record: &RunRecord,
    channel: &str,
    metric: ChannelMetric,
) -> Result<f64, TrendError> {
    let Some(digest) = record
        .channel_digests
        .iter()
        .find(|digest| digest.channel == channel)
    else {
        return missing_metric(record, channel, metric);
    };
    digest_metric(record, digest, channel, metric)
}

fn digest_metric(
    record: &RunRecord,
    digest: &ChannelDigest,
    channel: &str,
    metric: ChannelMetric,
) -> Result<f64, TrendError> {
    let Some(numeric) = digest.numeric else {
        return missing_metric(record, channel, metric);
    };
    Ok(match metric {
        ChannelMetric::Min => numeric.min,
        ChannelMetric::Max => numeric.max,
        ChannelMetric::Mean => numeric.mean,
        ChannelMetric::Rms => numeric.rms,
        ChannelMetric::Final => numeric.final_value,
    })
}

fn missing_metric<T>(
    record: &RunRecord,
    channel: &str,
    metric: ChannelMetric,
) -> Result<T, TrendError> {
    Err(TrendError::MissingMetric {
        run_id: record.run_id.clone(),
        channel: channel.to_owned(),
        metric,
    })
}

fn invalid_detector<T>(spec_id: &str, field: &'static str) -> Result<T, TrendError> {
    Err(TrendError::InvalidDetector {
        spec_id: spec_id.to_owned(),
        field,
    })
}

fn report_order(a: &DriftReport, b: &DriftReport) -> std::cmp::Ordering {
    a.family_id
        .cmp(&b.family_id)
        .then_with(|| a.channel.cmp(&b.channel))
        .then_with(|| a.metric.cmp(&b.metric))
        .then_with(|| a.spec_id.cmp(&b.spec_id))
}

fn metric_name(metric: ChannelMetric) -> &'static str {
    match metric {
        ChannelMetric::Min => "min",
        ChannelMetric::Max => "max",
        ChannelMetric::Mean => "mean",
        ChannelMetric::Rms => "rms",
        ChannelMetric::Final => "final",
    }
}

fn status_name(status: DriftStatus) -> &'static str {
    match status {
        DriftStatus::InsufficientData => "insufficient_data",
        DriftStatus::Nominal => "nominal",
        DriftStatus::Drift => "drift",
    }
}

fn format_f64(value: f64) -> String {
    format!("{value:.17e}")
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::{ArtifactRef, ChannelDigest, NumericChannelDigest, RunLabel, append_record_jsonl};
    use serde::Deserialize;

    #[derive(Debug, Deserialize)]
    struct DetectorExpectationFixture {
        page_hinkley_step: StepExpectation,
        flat_series: FlatExpectation,
    }

    #[derive(Debug, Deserialize)]
    struct StepExpectation {
        expected_first_drift_ordinal: u64,
        expected_detection_delay_points: u64,
    }

    #[derive(Debug, Deserialize)]
    struct FlatExpectation {
        expected_false_alarm_count: usize,
    }

    fn detector() -> DriftDetectorConfig {
        DriftDetectorConfig::PageHinkley {
            warmup: 3,
            delta: 0.0,
            lambda: 5.0,
            alpha: 1.0,
            direction: DriftDirection::Upward,
        }
    }

    fn spec(fail_on_drift: bool) -> TrendMetricSpec {
        TrendMetricSpec::new(
            "drop.altitude.final",
            "drop-family",
            "altitude_m",
            ChannelMetric::Final,
            detector(),
            fail_on_drift,
        )
        .unwrap()
    }

    fn record(seed: u64, ordinal: u64, code_id: &str, final_value: f64) -> RunRecord {
        RunRecord::new(
            "1111111111111111111111111111111111111111111111111111111111111111",
            seed,
            code_id,
            vec![ChannelDigest {
                channel: "altitude_m".to_owned(),
                value_kind: "float64".to_owned(),
                sample_count: 1,
                present_count: 1,
                numeric: Some(NumericChannelDigest {
                    min: final_value,
                    max: final_value,
                    mean: final_value,
                    rms: final_value.abs(),
                    final_value,
                }),
                bool_true_count: None,
                final_bool: None,
                final_text_sha256_hex: None,
            }],
            vec![
                ArtifactRef::new(
                    "telemetry_csv",
                    format!("runs/{seed}.csv"),
                    "2222222222222222222222222222222222222222222222222222222222222222",
                )
                .unwrap(),
            ],
            vec![
                RunLabel::new(FAMILY_LABEL_KEY, "drop-family").unwrap(),
                RunLabel::new(CODE_ORDINAL_LABEL_KEY, ordinal.to_string()).unwrap(),
            ],
            Vec::new(),
        )
        .unwrap()
    }

    fn step_records() -> Vec<RunRecord> {
        (0_u64..10)
            .map(|ordinal| {
                let value = if ordinal < 5 { 0.0 } else { 10.0 };
                record(ordinal, ordinal, &format!("code-{ordinal:02}"), value)
            })
            .collect()
    }

    fn flat_records() -> Vec<RunRecord> {
        (0_u64..10)
            .map(|ordinal| record(ordinal, ordinal, &format!("code-{ordinal:02}"), 0.0))
            .collect()
    }

    #[test]
    fn trend_store_builds_code_ordinal_history_and_aggregates_runs() {
        let records = [
            record(1, 0, "code-00", 2.0),
            record(2, 0, "code-00", 4.0),
            record(3, 1, "code-01", 8.0),
        ];

        let store = generate_trend_store(&records, &[spec(false)]).unwrap();

        assert_eq!(store.schema_version, TREND_SCHEMA_VERSION);
        assert_eq!(store.series.len(), 1);
        assert_eq!(store.series[0].points.len(), 2);
        assert_eq!(store.series[0].points[0].ordinal, 0);
        assert_eq!(store.series[0].points[0].value, 3.0);
        assert_eq!(store.series[0].points[0].source_run_ids.len(), 2);
        assert_eq!(store.series[0].points[1].value, 8.0);
    }

    #[test]
    fn trend_store_consumes_append_only_jsonl_index() {
        let first = record(1, 0, "code-00", 0.0);
        let second = record(2, 1, "code-01", 1.0);
        let bytes = append_record_jsonl(&first.to_jsonl_line().unwrap(), &second).unwrap();

        let from_jsonl = generate_trend_store_from_jsonl_index(&bytes, &[spec(false)]).unwrap();
        let mut index = RunIndex::new();
        index.append(first).unwrap();
        index.append(second).unwrap();
        let from_index = generate_trend_store_from_index(&index, &[spec(false)]).unwrap();

        assert_eq!(from_jsonl, from_index);
    }

    #[test]
    fn page_hinkley_detects_synthetic_step_at_declared_delay() {
        let fixture: DetectorExpectationFixture = toml::from_str(include_str!(
            "../tests/fixtures/trend-detector-expectations.toml"
        ))
        .unwrap();
        let reports =
            evaluate_trend_store(&generate_trend_store(&step_records(), &[spec(true)]).unwrap());

        assert_eq!(reports.len(), 1);
        assert_eq!(reports[0].status, DriftStatus::Drift);
        assert_eq!(
            reports[0].first_drift_ordinal,
            Some(fixture.page_hinkley_step.expected_first_drift_ordinal)
        );
        let delay = reports[0].first_drift_ordinal.unwrap() - 5;
        assert_eq!(
            delay,
            fixture.page_hinkley_step.expected_detection_delay_points
        );
    }

    #[test]
    fn page_hinkley_has_no_false_alarm_on_flat_series() {
        let fixture: DetectorExpectationFixture = toml::from_str(include_str!(
            "../tests/fixtures/trend-detector-expectations.toml"
        ))
        .unwrap();
        let reports =
            evaluate_trend_store(&generate_trend_store(&flat_records(), &[spec(true)]).unwrap());

        let false_alarm_count = reports
            .iter()
            .filter(|report| report.status == DriftStatus::Drift)
            .count();
        assert_eq!(
            false_alarm_count,
            fixture.flat_series.expected_false_alarm_count
        );
        assert_eq!(reports[0].status, DriftStatus::Nominal);
    }

    #[test]
    fn page_hinkley_respects_warmup_before_reporting() {
        let detector = DriftDetectorConfig::PageHinkley {
            warmup: 3,
            delta: 0.0,
            lambda: 5.0,
            alpha: 1.0,
            direction: DriftDirection::Upward,
        };
        let spec = TrendMetricSpec::new(
            "warmup.altitude.final",
            "drop-family",
            "altitude_m",
            ChannelMetric::Final,
            detector,
            true,
        )
        .unwrap();
        let records = [
            record(1, 0, "code-00", 0.0),
            record(2, 1, "code-01", 100.0),
            record(3, 2, "code-02", 100.0),
        ];

        let reports = evaluate_trend_store(&generate_trend_store(&records, &[spec]).unwrap());

        assert_eq!(reports[0].status, DriftStatus::Drift);
        assert_eq!(reports[0].first_drift_ordinal, Some(2));
    }

    #[test]
    fn scoreboard_markdown_is_byte_stable() {
        let reports =
            evaluate_trend_store(&generate_trend_store(&step_records(), &[spec(true)]).unwrap());
        let scoreboard = generate_scoreboard_markdown(&reports);
        assert_eq!(
            scoreboard,
            include_str!("../tests/fixtures/trend-scoreboard.md")
        );
    }

    #[test]
    fn ci_gate_fails_constructed_drift_fixture() {
        let reports =
            evaluate_trend_store(&generate_trend_store(&step_records(), &[spec(true)]).unwrap());
        let failures = ci_gate_failures(&reports);

        assert_eq!(failures.len(), 1);
        assert!(failures[0].contains("drop-family altitude_m.final drifted at ordinal 5"));
    }

    #[test]
    fn ci_gate_ignores_observe_only_drift() {
        let reports =
            evaluate_trend_store(&generate_trend_store(&step_records(), &[spec(false)]).unwrap());

        assert!(ci_gate_failures(&reports).is_empty());
    }

    #[test]
    fn trend_json_round_trips() {
        let store = generate_trend_store(&step_records(), &[spec(true)]).unwrap();
        let decoded = TrendStore::from_json_bytes(&store.to_json_bytes().unwrap()).unwrap();

        assert_eq!(decoded, store);
    }

    #[test]
    fn trend_generation_fails_closed_on_bad_inputs() {
        let mut missing_ordinal = record(1, 0, "code-00", 1.0);
        missing_ordinal
            .labels
            .retain(|label| label.key != CODE_ORDINAL_LABEL_KEY);
        let missing = generate_trend_store(&[missing_ordinal], &[spec(false)]).unwrap_err();
        assert!(matches!(missing, TrendError::MissingCodeOrdinal { .. }));

        let records = [
            record(1, 0, "code-00", 1.0),
            record(2, 0, "code-other", 2.0),
        ];
        let conflict = generate_trend_store(&records, &[spec(false)]).unwrap_err();
        assert!(matches!(conflict, TrendError::ConflictingCodeId { .. }));

        let duplicate =
            generate_trend_store(&step_records(), &[spec(false), spec(false)]).unwrap_err();
        assert!(matches!(duplicate, TrendError::DuplicateSpecId { .. }));
    }

    #[test]
    fn trend_generation_rejects_missing_or_nonnumeric_metric() {
        let mut run = record(1, 0, "code-00", 1.0);
        run.channel_digests[0].numeric = None;
        let err = generate_trend_store(&[run], &[spec(false)]).unwrap_err();

        assert!(matches!(err, TrendError::MissingMetric { .. }));
    }

    #[test]
    fn detector_validation_rejects_bad_parameters() {
        let bad = DriftDetectorConfig::PageHinkley {
            warmup: 0,
            delta: 0.0,
            lambda: 5.0,
            alpha: 1.0,
            direction: DriftDirection::Upward,
        };
        let err = TrendMetricSpec::new(
            "bad",
            "drop-family",
            "altitude_m",
            ChannelMetric::Final,
            bad,
            true,
        )
        .unwrap_err();

        assert!(matches!(err, TrendError::InvalidDetector { .. }));
    }
}
