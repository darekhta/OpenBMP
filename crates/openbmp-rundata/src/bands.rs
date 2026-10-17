//! Deterministic baseline-band artifacts for run families.
//!
//! Bands are generated from committed run records, not live services. The
//! artifact schema is intentionally plain JSON so downstream consumers, such
//! as the launch-commit predicate in doc 18, can consume the same file without
//! translation.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::verdict::ChannelMetric;
use crate::{ChannelDigest, RunIndex, RunRecord, RundataError, require_non_empty};

/// Baseline-band artifact schema version.
pub const BAND_SCHEMA_VERSION: &str = "openbmp.rundata.band.v1";

/// Errors returned by baseline-band helpers.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum BandError {
    /// JSON serialization or parsing failed.
    #[error("band JSON error: {message}")]
    Json {
        /// Error message.
        message: String,
    },
    /// JSONL run index parsing failed.
    #[error("band run-index error: {message}")]
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
    /// At least one run record is required.
    #[error("baseline band generation requires at least one run record")]
    EmptyRecords,
    /// At least one band specification is required.
    #[error("baseline band generation requires at least one band spec")]
    EmptySpecs,
    /// Percentile bounds were outside the supported interval or inverted.
    #[error("invalid percentile bounds lower={lower}, upper={upper}")]
    InvalidPercentiles {
        /// Lower percentile.
        lower: f64,
        /// Upper percentile.
        upper: f64,
    },
    /// A scalar parameter was non-finite or invalid.
    #[error("{field} must be finite and non-negative")]
    InvalidScalar {
        /// Field name.
        field: &'static str,
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

impl From<serde_json::Error> for BandError {
    fn from(error: serde_json::Error) -> Self {
        Self::Json {
            message: error.to_string(),
        }
    }
}

impl From<RundataError> for BandError {
    fn from(error: RundataError) -> Self {
        Self::Index {
            message: error.to_string(),
        }
    }
}

/// One requested band in a family artifact.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct BaselineBandSpec {
    /// Telemetry channel name.
    pub channel: String,
    /// Numeric digest metric.
    pub metric: ChannelMetric,
    /// Lower percentile in `[0, 1]`.
    pub lower_percentile: f64,
    /// Upper percentile in `[0, 1]`.
    pub upper_percentile: f64,
    /// Median absolute deviation scale for the robust envelope.
    pub mad_scale: f64,
}

impl BaselineBandSpec {
    /// Construct a validated band specification.
    ///
    /// # Errors
    ///
    /// Returns [`BandError`] when the channel is empty, percentiles are
    /// invalid, or `mad_scale` is non-finite/negative.
    pub fn new(
        channel: impl Into<String>,
        metric: ChannelMetric,
        lower_percentile: f64,
        upper_percentile: f64,
        mad_scale: f64,
    ) -> Result<Self, BandError> {
        let spec = Self {
            channel: require_non_empty("band.channel", channel.into()).map_err(|_| {
                BandError::EmptyField {
                    field: "band.channel",
                }
            })?,
            metric,
            lower_percentile,
            upper_percentile,
            mad_scale,
        };
        spec.validate()?;
        Ok(spec)
    }

    fn validate(&self) -> Result<(), BandError> {
        validate_percentiles(self.lower_percentile, self.upper_percentile)?;
        if !self.mad_scale.is_finite() || self.mad_scale < 0.0 {
            return Err(BandError::InvalidScalar { field: "mad_scale" });
        }
        Ok(())
    }
}

/// A deterministic baseline-band artifact for one run family.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct BaselineBandArtifact {
    /// Artifact schema version.
    pub schema_version: String,
    /// Stable scenario/campaign family id.
    pub family_id: String,
    /// Number of records used to generate each band.
    pub sample_count: u64,
    /// Bands in deterministic `(channel, metric)` order.
    pub bands: Vec<BaselineBand>,
}

impl BaselineBandArtifact {
    /// Serialize as deterministic JSON bytes.
    ///
    /// # Errors
    ///
    /// Returns [`BandError::Json`] if serialization fails.
    pub fn to_json_bytes(&self) -> Result<Vec<u8>, BandError> {
        Ok(serde_json::to_vec(self)?)
    }

    /// Parse a JSON artifact.
    ///
    /// # Errors
    ///
    /// Returns [`BandError::Json`] if parsing fails.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, BandError> {
        Ok(serde_json::from_slice(bytes)?)
    }

    /// Find a band by channel and metric.
    #[must_use]
    pub fn band(&self, channel: &str, metric: ChannelMetric) -> Option<&BaselineBand> {
        self.bands
            .iter()
            .find(|band| band.channel == channel && band.metric == metric)
    }
}

/// One robust baseline band.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct BaselineBand {
    /// Telemetry channel name.
    pub channel: String,
    /// Numeric digest metric.
    pub metric: ChannelMetric,
    /// Number of samples used.
    pub sample_count: u64,
    /// Sample median.
    pub median: f64,
    /// Median absolute deviation around [`Self::median`].
    pub mad: f64,
    /// Lower percentile in `[0, 1]`.
    pub lower_percentile: f64,
    /// Upper percentile in `[0, 1]`.
    pub upper_percentile: f64,
    /// Interpolated lower percentile value.
    pub percentile_lower: f64,
    /// Interpolated upper percentile value.
    pub percentile_upper: f64,
    /// Robust lower bound `median - mad_scale * mad`.
    pub mad_lower: f64,
    /// Robust upper bound `median + mad_scale * mad`.
    pub mad_upper: f64,
}

impl BaselineBand {
    /// Return true when `value` lies inside both percentile and MAD envelopes.
    #[must_use]
    pub fn contains(&self, value: f64) -> bool {
        value >= self.percentile_lower
            && value <= self.percentile_upper
            && value >= self.mad_lower
            && value <= self.mad_upper
    }
}

/// Result of evaluating one run metric against a baseline band.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct OutOfFamilyFinding {
    /// Telemetry channel name.
    pub channel: String,
    /// Numeric digest metric.
    pub metric: ChannelMetric,
    /// Observed value in the candidate run.
    pub value: f64,
    /// Lower effective bound.
    pub lower: f64,
    /// Upper effective bound.
    pub upper: f64,
    /// True when the value lies outside either the percentile or MAD envelope.
    pub out_of_family: bool,
}

/// Doc-18 launch-commit predicate over a shared baseline-band artifact.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct LaunchCommitOutOfFamilyPredicate {
    /// Stable predicate id.
    pub id: String,
    /// Telemetry channel name.
    pub channel: String,
    /// Numeric digest metric.
    pub metric: ChannelMetric,
}

impl LaunchCommitOutOfFamilyPredicate {
    /// Construct a validated out-of-family launch-commit predicate.
    ///
    /// # Errors
    ///
    /// Returns [`BandError::EmptyField`] when id or channel is empty.
    pub fn new(
        id: impl Into<String>,
        channel: impl Into<String>,
        metric: ChannelMetric,
    ) -> Result<Self, BandError> {
        Ok(Self {
            id: require_non_empty("lcc.id", id.into())
                .map_err(|_| BandError::EmptyField { field: "lcc.id" })?,
            channel: require_non_empty("lcc.channel", channel.into()).map_err(|_| {
                BandError::EmptyField {
                    field: "lcc.channel",
                }
            })?,
            metric,
        })
    }
}

/// Launch-commit predicate status.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LaunchCommitStatus {
    /// Predicate is green/go.
    Green,
    /// Predicate is red/no-go.
    Red,
}

/// Launch-commit out-of-family predicate report.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct LaunchCommitOutOfFamilyReport {
    /// Predicate id.
    pub id: String,
    /// Green/red status.
    pub status: LaunchCommitStatus,
    /// Out-of-family finding when the band and metric were available.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub finding: Option<OutOfFamilyFinding>,
    /// Stable detail string.
    pub detail: String,
}

/// Generate a deterministic baseline-band artifact from run records.
///
/// # Errors
///
/// Returns [`BandError`] when inputs are empty, malformed, or a record lacks a
/// requested numeric metric.
pub fn generate_baseline_band_artifact(
    family_id: impl Into<String>,
    records: &[RunRecord],
    specs: &[BaselineBandSpec],
) -> Result<BaselineBandArtifact, BandError> {
    if records.is_empty() {
        return Err(BandError::EmptyRecords);
    }
    if specs.is_empty() {
        return Err(BandError::EmptySpecs);
    }
    let family_id = require_non_empty("family_id", family_id.into())
        .map_err(|_| BandError::EmptyField { field: "family_id" })?;

    let mut specs = specs.to_vec();
    for spec in &specs {
        spec.validate()?;
    }
    specs.sort_by(|a, b| {
        a.channel
            .cmp(&b.channel)
            .then_with(|| a.metric.cmp(&b.metric))
            .then_with(|| a.lower_percentile.total_cmp(&b.lower_percentile))
            .then_with(|| a.upper_percentile.total_cmp(&b.upper_percentile))
            .then_with(|| a.mad_scale.total_cmp(&b.mad_scale))
    });

    let mut bands = Vec::with_capacity(specs.len());
    for spec in specs {
        let mut values = Vec::with_capacity(records.len());
        for record in records {
            values.push(record_metric(record, &spec.channel, spec.metric)?);
        }
        values.sort_by(f64::total_cmp);
        let median = percentile_sorted(&values, 0.5);
        let mut deviations = values
            .iter()
            .map(|value| (value - median).abs())
            .collect::<Vec<_>>();
        deviations.sort_by(f64::total_cmp);
        let mad = percentile_sorted(&deviations, 0.5);
        let percentile_lower = percentile_sorted(&values, spec.lower_percentile);
        let percentile_upper = percentile_sorted(&values, spec.upper_percentile);
        bands.push(BaselineBand {
            channel: spec.channel,
            metric: spec.metric,
            sample_count: values.len() as u64,
            median,
            mad,
            lower_percentile: spec.lower_percentile,
            upper_percentile: spec.upper_percentile,
            percentile_lower,
            percentile_upper,
            mad_lower: median - spec.mad_scale * mad,
            mad_upper: median + spec.mad_scale * mad,
        });
    }

    Ok(BaselineBandArtifact {
        schema_version: BAND_SCHEMA_VERSION.to_owned(),
        family_id,
        sample_count: records.len() as u64,
        bands,
    })
}

/// Generate a deterministic baseline-band artifact from an append-only run
/// index.
///
/// # Errors
///
/// Returns [`BandError`] when the index is empty or band generation fails.
pub fn generate_baseline_band_artifact_from_index(
    family_id: impl Into<String>,
    index: &RunIndex,
    specs: &[BaselineBandSpec],
) -> Result<BaselineBandArtifact, BandError> {
    generate_baseline_band_artifact(family_id, index.records(), specs)
}

/// Parse a JSONL run index and generate a deterministic baseline-band
/// artifact.
///
/// # Errors
///
/// Returns [`BandError`] when JSONL parsing or band generation fails.
pub fn generate_baseline_band_artifact_from_jsonl_index(
    family_id: impl Into<String>,
    jsonl: &[u8],
    specs: &[BaselineBandSpec],
) -> Result<BaselineBandArtifact, BandError> {
    let index = RunIndex::from_jsonl_bytes(jsonl)?;
    generate_baseline_band_artifact_from_index(family_id, &index, specs)
}

/// Evaluate a candidate run against every band in an artifact.
///
/// # Errors
///
/// Returns [`BandError::MissingMetric`] when the candidate lacks a requested
/// numeric metric.
pub fn evaluate_out_of_family(
    artifact: &BaselineBandArtifact,
    record: &RunRecord,
) -> Result<Vec<OutOfFamilyFinding>, BandError> {
    artifact
        .bands
        .iter()
        .map(|band| {
            let value = record_metric(record, &band.channel, band.metric)?;
            let lower = band.percentile_lower.max(band.mad_lower);
            let upper = band.percentile_upper.min(band.mad_upper);
            Ok(OutOfFamilyFinding {
                channel: band.channel.clone(),
                metric: band.metric,
                value,
                lower,
                upper,
                out_of_family: !band.contains(value),
            })
        })
        .collect()
}

/// Evaluate a doc-18-style launch-commit out-of-family predicate using the
/// same baseline-band artifact schema.
///
/// Missing bands and missing candidate metrics are red/no-go, not skipped.
#[must_use]
pub fn evaluate_launch_commit_out_of_family(
    predicate: &LaunchCommitOutOfFamilyPredicate,
    artifact: &BaselineBandArtifact,
    record: &RunRecord,
) -> LaunchCommitOutOfFamilyReport {
    let Some(band) = artifact.band(&predicate.channel, predicate.metric) else {
        return LaunchCommitOutOfFamilyReport {
            id: predicate.id.clone(),
            status: LaunchCommitStatus::Red,
            finding: None,
            detail: format!(
                "baseline band missing for {}.{:?}",
                predicate.channel, predicate.metric
            ),
        };
    };
    let value = match record_metric(record, &predicate.channel, predicate.metric) {
        Ok(value) => value,
        Err(error) => {
            return LaunchCommitOutOfFamilyReport {
                id: predicate.id.clone(),
                status: LaunchCommitStatus::Red,
                finding: None,
                detail: error.to_string(),
            };
        }
    };
    let lower = band.percentile_lower.max(band.mad_lower);
    let upper = band.percentile_upper.min(band.mad_upper);
    let finding = OutOfFamilyFinding {
        channel: predicate.channel.clone(),
        metric: predicate.metric,
        value,
        lower,
        upper,
        out_of_family: !band.contains(value),
    };
    let status = if finding.out_of_family {
        LaunchCommitStatus::Red
    } else {
        LaunchCommitStatus::Green
    };
    let detail = if finding.out_of_family {
        format!(
            "{}.{:?} {value:.17e} outside [{lower:.17e}, {upper:.17e}]",
            predicate.channel, predicate.metric
        )
    } else {
        format!(
            "{}.{:?} {value:.17e} inside [{lower:.17e}, {upper:.17e}]",
            predicate.channel, predicate.metric
        )
    };
    LaunchCommitOutOfFamilyReport {
        id: predicate.id.clone(),
        status,
        finding: Some(finding),
        detail,
    }
}

fn validate_percentiles(lower: f64, upper: f64) -> Result<(), BandError> {
    if !lower.is_finite() || !upper.is_finite() || lower < 0.0 || upper > 1.0 || lower > upper {
        Err(BandError::InvalidPercentiles { lower, upper })
    } else {
        Ok(())
    }
}

fn record_metric(
    record: &RunRecord,
    channel: &str,
    metric: ChannelMetric,
) -> Result<f64, BandError> {
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
) -> Result<f64, BandError> {
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
) -> Result<T, BandError> {
    Err(BandError::MissingMetric {
        run_id: record.run_id.clone(),
        channel: channel.to_owned(),
        metric,
    })
}

fn percentile_sorted(values: &[f64], percentile: f64) -> f64 {
    debug_assert!(!values.is_empty());
    if values.len() == 1 {
        return values[0];
    }
    let scaled = percentile * (values.len() - 1) as f64;
    let lower_index = scaled.floor() as usize;
    let upper_index = scaled.ceil() as usize;
    if lower_index == upper_index {
        values[lower_index]
    } else {
        let fraction = scaled - lower_index as f64;
        values[lower_index] * (1.0 - fraction) + values[upper_index] * fraction
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::{
        ArtifactRef, ChannelDigest, NumericChannelDigest, RunIndex, RunRecord, channel_digests,
    };
    use openbmp_core::{ChannelId, SimTime, StepIndex};
    use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
    use serde::Deserialize;

    #[derive(Debug, Deserialize)]
    struct RateFixture {
        in_family_false_alarm_rate: RateBounds,
        out_of_family_detection_rate: RateBounds,
    }

    #[derive(Debug, Deserialize)]
    struct RateBounds {
        min: f64,
        max: f64,
    }

    impl RateBounds {
        fn contains(&self, value: f64) -> bool {
            value >= self.min && value <= self.max
        }
    }

    fn record(seed: u64, final_value: f64) -> RunRecord {
        RunRecord::new(
            "1111111111111111111111111111111111111111111111111111111111111111",
            seed,
            format!("code-{seed}"),
            channel_digests(&table(&[final_value])),
            vec![
                ArtifactRef::new(
                    "scenario",
                    format!("scenario-{seed}.toml"),
                    "0000000000000000000000000000000000000000000000000000000000000000",
                )
                .unwrap(),
            ],
            Vec::new(),
            Vec::new(),
        )
        .unwrap()
    }

    fn table(values: &[f64]) -> TelemetryTable {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", None::<String>)
                .unwrap();
        let schema = TelemetrySchema::new(vec![altitude.metadata().clone()]).unwrap();
        let mut table = TelemetryTable::new(schema);
        for (index, value) in values.iter().enumerate() {
            let mut row = TelemetryRow::new(
                SimTime::from_seconds(index as f64),
                StepIndex::new(index as u64),
            )
            .unwrap();
            row.insert(&altitude, *value).unwrap();
            table.push_row(row).unwrap();
        }
        table
    }

    #[test]
    fn band_artifact_is_deterministic_and_ordered() {
        let records = [record(3, 30.0), record(1, 10.0), record(2, 20.0)];
        let specs =
            [BaselineBandSpec::new("altitude_m", ChannelMetric::Final, 0.0, 1.0, 2.0).unwrap()];

        let artifact = generate_baseline_band_artifact("drop-family", &records, &specs).unwrap();
        let bytes_a = artifact.to_json_bytes().unwrap();
        let bytes_b = generate_baseline_band_artifact("drop-family", &records, &specs)
            .unwrap()
            .to_json_bytes()
            .unwrap();

        assert_eq!(bytes_a, bytes_b);
        let golden = include_bytes!("../tests/fixtures/baseline-band.json");
        let golden = golden.strip_suffix(b"\n").unwrap_or(golden);
        assert_eq!(bytes_a, golden);
        let data_golden = include_bytes!("../../../data/rundata/baseline-band-v1.json");
        let data_golden = data_golden.strip_suffix(b"\n").unwrap_or(data_golden);
        assert_eq!(bytes_a, data_golden);
        assert_eq!(artifact.schema_version, BAND_SCHEMA_VERSION);
        assert_eq!(artifact.bands[0].median, 20.0);
        assert_eq!(artifact.bands[0].mad, 10.0);
        assert_eq!(artifact.bands[0].percentile_lower, 10.0);
        assert_eq!(artifact.bands[0].percentile_upper, 30.0);
        assert_eq!(artifact.bands[0].mad_lower, 0.0);
        assert_eq!(artifact.bands[0].mad_upper, 40.0);
    }

    #[test]
    fn band_generation_consumes_append_only_jsonl_index() {
        let mut index = RunIndex::new();
        index.append(record(3, 30.0)).unwrap();
        index.append(record(1, 10.0)).unwrap();
        index.append(record(2, 20.0)).unwrap();
        let specs =
            [BaselineBandSpec::new("altitude_m", ChannelMetric::Final, 0.0, 1.0, 2.0).unwrap()];
        let bytes = index.to_jsonl_bytes().unwrap();

        let from_index =
            generate_baseline_band_artifact_from_index("drop-family", &index, &specs).unwrap();
        let from_jsonl =
            generate_baseline_band_artifact_from_jsonl_index("drop-family", &bytes, &specs)
                .unwrap();

        assert_eq!(from_jsonl, from_index);
        assert_eq!(from_jsonl.bands[0].median, 20.0);
    }

    #[test]
    fn out_of_family_detection_uses_shared_band_schema() {
        let records = [record(1, 10.0), record(2, 20.0), record(3, 30.0)];
        let specs =
            [BaselineBandSpec::new("altitude_m", ChannelMetric::Final, 0.0, 1.0, 1.0).unwrap()];
        let artifact = generate_baseline_band_artifact("drop-family", &records, &specs).unwrap();

        let inside = evaluate_out_of_family(&artifact, &record(4, 25.0)).unwrap();
        let outside = evaluate_out_of_family(&artifact, &record(5, 35.0)).unwrap();

        assert!(!inside[0].out_of_family);
        assert!(outside[0].out_of_family);
        assert_eq!(outside[0].lower, 10.0);
        assert_eq!(outside[0].upper, 30.0);
    }

    #[test]
    fn launch_commit_predicate_consumes_band_schema_without_translation() {
        let records = [record(1, 10.0), record(2, 20.0), record(3, 30.0)];
        let specs =
            [BaselineBandSpec::new("altitude_m", ChannelMetric::Final, 0.0, 1.0, 1.0).unwrap()];
        let artifact = generate_baseline_band_artifact("drop-family", &records, &specs).unwrap();
        let predicate = LaunchCommitOutOfFamilyPredicate::new(
            "lcc.altitude",
            "altitude_m",
            ChannelMetric::Final,
        )
        .unwrap();

        let green = evaluate_launch_commit_out_of_family(&predicate, &artifact, &record(4, 20.0));
        let red = evaluate_launch_commit_out_of_family(&predicate, &artifact, &record(5, 35.0));
        let missing = evaluate_launch_commit_out_of_family(
            &LaunchCommitOutOfFamilyPredicate::new(
                "lcc.missing",
                "missing_m",
                ChannelMetric::Final,
            )
            .unwrap(),
            &artifact,
            &record(6, 20.0),
        );

        assert_eq!(green.status, LaunchCommitStatus::Green);
        assert_eq!(red.status, LaunchCommitStatus::Red);
        assert_eq!(missing.status, LaunchCommitStatus::Red);
        assert!(missing.detail.contains("baseline band missing"));
    }

    #[test]
    fn generation_rejects_missing_or_nonnumeric_metric() {
        let mut run = record(1, 10.0);
        run.channel_digests = vec![ChannelDigest {
            channel: "mode".to_owned(),
            value_kind: "text".to_owned(),
            sample_count: 1,
            present_count: 1,
            numeric: None,
            bool_true_count: None,
            final_bool: None,
            final_text_sha256_hex: Some(
                "0000000000000000000000000000000000000000000000000000000000000000".to_owned(),
            ),
        }];
        let spec = BaselineBandSpec::new("mode", ChannelMetric::Final, 0.0, 1.0, 1.0).unwrap();
        let err = generate_baseline_band_artifact("family", &[run], &[spec]).unwrap_err();
        assert!(matches!(err, BandError::MissingMetric { .. }));
    }

    #[test]
    fn spec_validation_rejects_bad_percentiles() {
        let err =
            BaselineBandSpec::new("altitude_m", ChannelMetric::Final, 0.9, 0.1, 1.0).unwrap_err();
        assert!(matches!(err, BandError::InvalidPercentiles { .. }));
    }

    #[test]
    fn band_json_round_trips() {
        let records = [record(1, 10.0), record(2, 20.0), record(3, 30.0)];
        let specs =
            [BaselineBandSpec::new("altitude_m", ChannelMetric::Final, 0.1, 0.9, 1.5).unwrap()];
        let artifact = generate_baseline_band_artifact("drop-family", &records, &specs).unwrap();
        let decoded =
            BaselineBandArtifact::from_json_bytes(&artifact.to_json_bytes().unwrap()).unwrap();
        assert_eq!(decoded, artifact);
    }

    #[test]
    fn percentile_interpolation_is_linear() {
        let values = [0.0, 10.0, 20.0];
        assert_eq!(percentile_sorted(&values, 0.25), 5.0);
        assert_eq!(percentile_sorted(&values, 0.75), 15.0);
    }

    #[test]
    fn direct_digest_metric_can_build_band_without_table() {
        let mut run = record(1, 10.0);
        run.channel_digests[0].numeric = Some(NumericChannelDigest {
            min: 1.0,
            max: 5.0,
            mean: 3.0,
            rms: 3.5,
            final_value: 4.0,
        });
        let spec = BaselineBandSpec::new("altitude_m", ChannelMetric::Max, 0.0, 1.0, 1.0).unwrap();
        let artifact = generate_baseline_band_artifact("family", &[run], &[spec]).unwrap();
        assert_eq!(artifact.bands[0].median, 5.0);
    }

    #[test]
    fn synthetic_rate_fixture_bounds_detection_and_false_alarm_rates() {
        let fixture: RateFixture =
            toml::from_str(include_str!("../tests/fixtures/baseline-band-rates.toml")).unwrap();
        let records = (0_u64..100)
            .map(|seed| record(seed, seed as f64))
            .collect::<Vec<_>>();
        let specs =
            [BaselineBandSpec::new("altitude_m", ChannelMetric::Final, 0.1, 0.9, 10.0).unwrap()];
        let artifact =
            generate_baseline_band_artifact("synthetic-family", &records, &specs).unwrap();

        let in_family_false_alarms = (0_u64..100)
            .filter(|seed| {
                evaluate_out_of_family(&artifact, &record(*seed + 100, *seed as f64)).unwrap()[0]
                    .out_of_family
            })
            .count();
        let shifted_detections = (0_u64..100)
            .filter(|seed| {
                evaluate_out_of_family(&artifact, &record(*seed + 200, *seed as f64 + 100.0))
                    .unwrap()[0]
                    .out_of_family
            })
            .count();
        let false_alarm_rate = in_family_false_alarms as f64 / 100.0;
        let detection_rate = shifted_detections as f64 / 100.0;

        assert!(
            fixture
                .in_family_false_alarm_rate
                .contains(false_alarm_rate)
        );
        assert!(
            fixture
                .out_of_family_detection_rate
                .contains(detection_rate)
        );
    }
}
