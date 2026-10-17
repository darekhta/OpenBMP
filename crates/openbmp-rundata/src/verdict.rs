//! Declarative post-run verdict rules.
//!
//! Verdict rules are evaluated after a run over an already-recorded telemetry
//! table. They are intentionally fail-closed: an unknown channel, missing
//! baseline, non-numeric metric, or malformed rule yields a failure or parse
//! error, never an implicit pass.

use std::collections::BTreeSet;

use openbmp_telemetry::{TelemetryTable, TelemetryValue};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{ChannelDigest, RunRecord, channel_digests, require_non_empty};

/// Verdict report schema version.
pub const VERDICT_SCHEMA_VERSION: &str = "openbmp.rundata.verdict.v1";

/// Errors returned while parsing or validating verdict rule documents.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum VerdictError {
    /// TOML parsing failed.
    #[error("verdict TOML error: {message}")]
    Toml {
        /// Error message.
        message: String,
    },
    /// JSON serialization failed.
    #[error("verdict JSON error: {message}")]
    Json {
        /// Error message.
        message: String,
    },
    /// A rule document was empty.
    #[error("verdict rule document must contain at least one rule")]
    EmptyRules,
    /// A required string field was empty after trimming.
    #[error("{field} must not be empty")]
    EmptyField {
        /// Field name.
        field: &'static str,
    },
    /// Rule ids must be unique.
    #[error("duplicate verdict rule id {id}")]
    DuplicateRuleId {
        /// Duplicate rule id.
        id: String,
    },
    /// A rule has invalid bounds or parameters.
    #[error("invalid verdict rule {id}: {message}")]
    InvalidRule {
        /// Rule id.
        id: String,
        /// Validation message.
        message: String,
    },
}

impl From<toml::de::Error> for VerdictError {
    fn from(error: toml::de::Error) -> Self {
        Self::Toml {
            message: error.to_string(),
        }
    }
}

impl From<serde_json::Error> for VerdictError {
    fn from(error: serde_json::Error) -> Self {
        Self::Json {
            message: error.to_string(),
        }
    }
}

/// Declarative verdict rule document.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct VerdictRuleDocument {
    /// Rules evaluated in declared order.
    pub rules: Vec<VerdictRule>,
}

impl VerdictRuleDocument {
    /// Parse and validate a TOML verdict rule document.
    ///
    /// # Errors
    ///
    /// Returns [`VerdictError`] when the TOML is malformed or any rule is
    /// invalid.
    pub fn from_toml_str(input: &str) -> Result<Self, VerdictError> {
        let document: Self = toml::from_str(input)?;
        document.validate()?;
        Ok(document)
    }

    /// Validate rule ids and rule-local parameters.
    ///
    /// # Errors
    ///
    /// Returns [`VerdictError`] when the rule set is empty, has duplicate ids,
    /// or contains invalid finite bounds.
    pub fn validate(&self) -> Result<(), VerdictError> {
        if self.rules.is_empty() {
            return Err(VerdictError::EmptyRules);
        }
        let mut ids = BTreeSet::new();
        for rule in &self.rules {
            let id = require_non_empty("rule.id", rule.id().to_owned())
                .map_err(|_| VerdictError::EmptyField { field: "rule.id" })?;
            if !ids.insert(id.clone()) {
                return Err(VerdictError::DuplicateRuleId { id });
            }
            rule.validate()?;
        }
        Ok(())
    }

    /// Evaluate all rules against a telemetry table and optional baseline.
    #[must_use]
    pub fn evaluate(&self, table: &TelemetryTable, baseline: Option<&RunRecord>) -> VerdictReport {
        evaluate_rules(&self.rules, table, baseline)
    }
}

/// One declared verdict rule.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum VerdictRule {
    /// Bound a numeric digest metric.
    Limit {
        /// Stable rule id.
        id: String,
        /// Telemetry channel name.
        channel: String,
        /// Metric to evaluate.
        metric: ChannelMetric,
        /// Optional inclusive lower bound.
        min: Option<f64>,
        /// Optional inclusive upper bound.
        max: Option<f64>,
    },
    /// Compare a numeric digest metric with a scalar golden value.
    ToleranceVsGolden {
        /// Stable rule id.
        id: String,
        /// Telemetry channel name.
        channel: String,
        /// Metric to evaluate.
        metric: ChannelMetric,
        /// Expected golden value.
        expected: f64,
        /// Absolute tolerance.
        abs_tol: f64,
        /// Relative tolerance scale.
        #[serde(default)]
        rel_tol: f64,
    },
    /// Require a numeric channel to move monotonically over observed samples.
    Monotonic {
        /// Stable rule id.
        id: String,
        /// Telemetry channel name.
        channel: String,
        /// Required monotonic direction.
        direction: MonotonicDirection,
        /// Whether equal adjacent samples are accepted.
        #[serde(default = "default_allow_equal")]
        allow_equal: bool,
    },
    /// Require a bool/text event marker to occur in a time window.
    EventWindow {
        /// Stable rule id.
        id: String,
        /// Telemetry channel name.
        channel: String,
        /// For text channels, the value that marks the event. If omitted, any
        /// present text sample marks the event.
        equals: Option<String>,
        /// Inclusive window start time.
        start_s: f64,
        /// Inclusive window end time.
        end_s: f64,
    },
    /// Bound a numeric digest metric by a precomputed statistical band.
    StatisticalBand {
        /// Stable rule id.
        id: String,
        /// Telemetry channel name.
        channel: String,
        /// Metric to evaluate.
        metric: ChannelMetric,
        /// Inclusive lower band.
        lower: f64,
        /// Inclusive upper band.
        upper: f64,
    },
    /// Compare a numeric digest metric with the same metric in a baseline run.
    CrossRunDelta {
        /// Stable rule id.
        id: String,
        /// Telemetry channel name.
        channel: String,
        /// Metric to evaluate.
        metric: ChannelMetric,
        /// Maximum absolute delta from the baseline metric.
        max_abs_delta: f64,
    },
}

impl VerdictRule {
    /// Stable rule id.
    #[must_use]
    pub fn id(&self) -> &str {
        match self {
            Self::Limit { id, .. }
            | Self::ToleranceVsGolden { id, .. }
            | Self::Monotonic { id, .. }
            | Self::EventWindow { id, .. }
            | Self::StatisticalBand { id, .. }
            | Self::CrossRunDelta { id, .. } => id,
        }
    }

    fn validate(&self) -> Result<(), VerdictError> {
        match self {
            Self::Limit {
                id,
                channel,
                min,
                max,
                ..
            } => {
                require_channel(id, channel)?;
                if min.is_none() && max.is_none() {
                    return invalid(id, "limit rule must declare min, max, or both");
                }
                if let (Some(min), Some(max)) = (min, max) {
                    finite_bound(id, "min", *min)?;
                    finite_bound(id, "max", *max)?;
                    if min > max {
                        return invalid(id, "min must be <= max");
                    }
                }
                if let Some(min) = min {
                    finite_bound(id, "min", *min)?;
                }
                if let Some(max) = max {
                    finite_bound(id, "max", *max)?;
                }
                Ok(())
            }
            Self::ToleranceVsGolden {
                id,
                channel,
                expected,
                abs_tol,
                rel_tol,
                ..
            } => {
                require_channel(id, channel)?;
                finite_bound(id, "expected", *expected)?;
                nonnegative_bound(id, "abs_tol", *abs_tol)?;
                nonnegative_bound(id, "rel_tol", *rel_tol)?;
                Ok(())
            }
            Self::Monotonic { id, channel, .. } => require_channel(id, channel),
            Self::EventWindow {
                id,
                channel,
                equals,
                start_s,
                end_s,
            } => {
                require_channel(id, channel)?;
                if let Some(value) = equals {
                    require_non_empty("rule.equals", value.clone()).map_err(|_| {
                        VerdictError::EmptyField {
                            field: "rule.equals",
                        }
                    })?;
                }
                finite_bound(id, "start_s", *start_s)?;
                finite_bound(id, "end_s", *end_s)?;
                if start_s > end_s {
                    return invalid(id, "start_s must be <= end_s");
                }
                Ok(())
            }
            Self::StatisticalBand {
                id,
                channel,
                lower,
                upper,
                ..
            } => {
                require_channel(id, channel)?;
                finite_bound(id, "lower", *lower)?;
                finite_bound(id, "upper", *upper)?;
                if lower > upper {
                    return invalid(id, "lower must be <= upper");
                }
                Ok(())
            }
            Self::CrossRunDelta {
                id,
                channel,
                max_abs_delta,
                ..
            } => {
                require_channel(id, channel)?;
                nonnegative_bound(id, "max_abs_delta", *max_abs_delta)
            }
        }
    }

    fn evaluate(
        &self,
        digests: &[ChannelDigest],
        table: &TelemetryTable,
        baseline: Option<&RunRecord>,
    ) -> VerdictRecord {
        match self {
            Self::Limit {
                id,
                channel,
                metric,
                min,
                max,
            } => match metric_value(digests, channel, *metric) {
                Ok(value) => bounded_record(id, value, *min, *max),
                Err(detail) => fail(id, None, None, None, detail),
            },
            Self::ToleranceVsGolden {
                id,
                channel,
                metric,
                expected,
                abs_tol,
                rel_tol,
            } => match metric_value(digests, channel, *metric) {
                Ok(value) => {
                    let tolerance = abs_tol.max(rel_tol * expected.abs());
                    let lower = expected - tolerance;
                    let upper = expected + tolerance;
                    bounded_record(id, value, Some(lower), Some(upper))
                }
                Err(detail) => fail(id, None, Some(*expected), None, detail),
            },
            Self::Monotonic {
                id,
                channel,
                direction,
                allow_equal,
            } => monotonic_record(id, table, channel, *direction, *allow_equal),
            Self::EventWindow {
                id,
                channel,
                equals,
                start_s,
                end_s,
            } => event_window_record(id, table, channel, equals.as_deref(), *start_s, *end_s),
            Self::StatisticalBand {
                id,
                channel,
                metric,
                lower,
                upper,
            } => match metric_value(digests, channel, *metric) {
                Ok(value) => bounded_record(id, value, Some(*lower), Some(*upper)),
                Err(detail) => fail(id, None, None, None, detail),
            },
            Self::CrossRunDelta {
                id,
                channel,
                metric,
                max_abs_delta,
            } => cross_run_delta_record(id, digests, baseline, channel, *metric, *max_abs_delta),
        }
    }
}

/// Numeric channel metric used by verdict rules.
#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ChannelMetric {
    /// Minimum value.
    Min,
    /// Maximum value.
    Max,
    /// Mean value.
    Mean,
    /// RMS value.
    Rms,
    /// Final observed value.
    Final,
}

/// Required monotonic direction.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MonotonicDirection {
    /// Nondecreasing/increasing.
    Increasing,
    /// Nonincreasing/decreasing.
    Decreasing,
}

/// Verdict status.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum VerdictStatus {
    /// Rule passed.
    Pass,
    /// Rule failed.
    Fail,
    /// Rule was intentionally skipped. Current fail-closed rules do not emit
    /// this status, but it is reserved for future opt-in skip semantics.
    Skip,
}

/// One verdict result.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct VerdictRecord {
    /// Rule id.
    pub id: String,
    /// Pass/fail/skip status.
    pub status: VerdictStatus,
    /// Measured scalar value, if the rule produced one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub measured: Option<f64>,
    /// Expected scalar value, if applicable.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expected: Option<f64>,
    /// Inclusive lower bound, if applicable.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lower: Option<f64>,
    /// Inclusive upper bound, if applicable.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub upper: Option<f64>,
    /// Stable human-readable detail.
    pub detail: String,
}

/// Stable verdict report.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct VerdictReport {
    /// Verdict report schema version.
    pub schema_version: String,
    /// Verdicts in rule declaration order.
    pub verdicts: Vec<VerdictRecord>,
}

impl VerdictReport {
    /// Return true when every verdict passed.
    #[must_use]
    pub fn passed(&self) -> bool {
        self.verdicts
            .iter()
            .all(|verdict| verdict.status == VerdictStatus::Pass)
    }

    /// Serialize the verdict report as deterministic JSON bytes.
    ///
    /// # Errors
    ///
    /// Returns [`VerdictError::Json`] if serialization fails.
    pub fn to_json_bytes(&self) -> Result<Vec<u8>, VerdictError> {
        Ok(serde_json::to_vec(self)?)
    }
}

/// Evaluate verdict rules in declaration order.
#[must_use]
pub fn evaluate_rules(
    rules: &[VerdictRule],
    table: &TelemetryTable,
    baseline: Option<&RunRecord>,
) -> VerdictReport {
    let digests = channel_digests(table);
    VerdictReport {
        schema_version: VERDICT_SCHEMA_VERSION.to_owned(),
        verdicts: rules
            .iter()
            .map(|rule| rule.evaluate(&digests, table, baseline))
            .collect(),
    }
}

fn default_allow_equal() -> bool {
    true
}

fn require_channel(id: &str, channel: &str) -> Result<(), VerdictError> {
    require_non_empty("rule.channel", channel.to_owned()).map_err(|_| {
        VerdictError::EmptyField {
            field: "rule.channel",
        }
    })?;
    if id.trim().is_empty() {
        return Err(VerdictError::EmptyField { field: "rule.id" });
    }
    Ok(())
}

fn finite_bound(id: &str, field: &'static str, value: f64) -> Result<(), VerdictError> {
    if value.is_finite() {
        Ok(())
    } else {
        invalid(id, &format!("{field} must be finite"))
    }
}

fn nonnegative_bound(id: &str, field: &'static str, value: f64) -> Result<(), VerdictError> {
    finite_bound(id, field, value)?;
    if value >= 0.0 {
        Ok(())
    } else {
        invalid(id, &format!("{field} must be >= 0"))
    }
}

fn invalid<T>(id: &str, message: &str) -> Result<T, VerdictError> {
    Err(VerdictError::InvalidRule {
        id: id.to_owned(),
        message: message.to_owned(),
    })
}

fn metric_value(
    digests: &[ChannelDigest],
    channel: &str,
    metric: ChannelMetric,
) -> Result<f64, String> {
    let Some(digest) = digests.iter().find(|digest| digest.channel == channel) else {
        return Err(format!("channel `{channel}` was not found"));
    };
    let Some(numeric) = digest.numeric else {
        return Err(format!("channel `{channel}` is not numeric"));
    };
    if digest.present_count == 0 {
        return Err(format!("channel `{channel}` has no observed samples"));
    }
    Ok(match metric {
        ChannelMetric::Min => numeric.min,
        ChannelMetric::Max => numeric.max,
        ChannelMetric::Mean => numeric.mean,
        ChannelMetric::Rms => numeric.rms,
        ChannelMetric::Final => numeric.final_value,
    })
}

fn bounded_record(
    id: &str,
    measured: f64,
    lower: Option<f64>,
    upper: Option<f64>,
) -> VerdictRecord {
    let lower_pass = lower.is_none_or(|bound| measured >= bound);
    let upper_pass = upper.is_none_or(|bound| measured <= bound);
    let status = if lower_pass && upper_pass {
        VerdictStatus::Pass
    } else {
        VerdictStatus::Fail
    };
    let detail = match (lower, upper) {
        (Some(lower), Some(upper)) => {
            format!("{measured:.17e} in [{lower:.17e}, {upper:.17e}]")
        }
        (Some(lower), None) => format!("{measured:.17e} >= {lower:.17e}"),
        (None, Some(upper)) => format!("{measured:.17e} <= {upper:.17e}"),
        (None, None) => format!("{measured:.17e} evaluated without bounds"),
    };
    VerdictRecord {
        id: id.to_owned(),
        status,
        measured: Some(measured),
        expected: None,
        lower,
        upper,
        detail,
    }
}

fn fail(
    id: &str,
    measured: Option<f64>,
    expected: Option<f64>,
    bounds: Option<(f64, f64)>,
    detail: String,
) -> VerdictRecord {
    VerdictRecord {
        id: id.to_owned(),
        status: VerdictStatus::Fail,
        measured,
        expected,
        lower: bounds.map(|(lower, _)| lower),
        upper: bounds.map(|(_, upper)| upper),
        detail,
    }
}

fn monotonic_record(
    id: &str,
    table: &TelemetryTable,
    channel: &str,
    direction: MonotonicDirection,
    allow_equal: bool,
) -> VerdictRecord {
    let Some(channel_id) = channel_id(table, channel) else {
        return fail(
            id,
            None,
            None,
            None,
            format!("channel `{channel}` was not found"),
        );
    };
    let mut previous = None;
    let mut observed = 0_u64;
    for row in table.rows() {
        let Some(value) = row.get(channel_id) else {
            continue;
        };
        let Some(current) = numeric_value(value) else {
            return fail(
                id,
                None,
                None,
                None,
                format!("channel `{channel}` is not numeric"),
            );
        };
        if let Some(previous) = previous
            && !ordered(previous, current, direction, allow_equal)
        {
            return fail(
                id,
                Some(current),
                None,
                None,
                format!(
                    "channel `{channel}` violated {:?}: previous={previous:.17e}, current={current:.17e}",
                    direction
                ),
            );
        }
        previous = Some(current);
        observed += 1;
    }
    if observed == 0 {
        return fail(
            id,
            None,
            None,
            None,
            format!("channel `{channel}` has no observed samples"),
        );
    }
    VerdictRecord {
        id: id.to_owned(),
        status: VerdictStatus::Pass,
        measured: Some(observed as f64),
        expected: None,
        lower: None,
        upper: None,
        detail: format!(
            "channel `{channel}` satisfied {:?} over {observed} samples",
            direction
        ),
    }
}

fn event_window_record(
    id: &str,
    table: &TelemetryTable,
    channel: &str,
    equals: Option<&str>,
    start_s: f64,
    end_s: f64,
) -> VerdictRecord {
    let Some(channel_id) = channel_id(table, channel) else {
        return fail(
            id,
            None,
            None,
            None,
            format!("channel `{channel}` was not found"),
        );
    };
    for row in table.rows() {
        let Some(value) = row.get(channel_id) else {
            continue;
        };
        if event_matches(value, equals) {
            let event_time = row.time.as_seconds();
            return bounded_record(id, event_time, Some(start_s), Some(end_s));
        }
    }
    fail(
        id,
        None,
        None,
        Some((start_s, end_s)),
        format!("event channel `{channel}` did not fire"),
    )
}

fn cross_run_delta_record(
    id: &str,
    digests: &[ChannelDigest],
    baseline: Option<&RunRecord>,
    channel: &str,
    metric: ChannelMetric,
    max_abs_delta: f64,
) -> VerdictRecord {
    let Ok(current) = metric_value(digests, channel, metric) else {
        return fail(
            id,
            None,
            None,
            None,
            format!("channel `{channel}` was not found"),
        );
    };
    let Some(baseline) = baseline else {
        return fail(
            id,
            Some(current),
            None,
            None,
            "baseline run record is missing".to_owned(),
        );
    };
    let baseline_value = match metric_value(&baseline.channel_digests, channel, metric) {
        Ok(value) => value,
        Err(detail) => return fail(id, Some(current), None, None, detail),
    };
    let delta = (current - baseline_value).abs();
    let status = if delta <= max_abs_delta {
        VerdictStatus::Pass
    } else {
        VerdictStatus::Fail
    };
    VerdictRecord {
        id: id.to_owned(),
        status,
        measured: Some(delta),
        expected: Some(baseline_value),
        lower: Some(0.0),
        upper: Some(max_abs_delta),
        detail: format!(
            "abs(delta) {delta:.17e} from baseline {baseline_value:.17e} <= {max_abs_delta:.17e}"
        ),
    }
}

fn ordered(previous: f64, current: f64, direction: MonotonicDirection, allow_equal: bool) -> bool {
    match (direction, allow_equal) {
        (MonotonicDirection::Increasing, true) => current >= previous,
        (MonotonicDirection::Increasing, false) => current > previous,
        (MonotonicDirection::Decreasing, true) => current <= previous,
        (MonotonicDirection::Decreasing, false) => current < previous,
    }
}

fn channel_id(table: &TelemetryTable, channel: &str) -> Option<openbmp_core::ChannelId> {
    table
        .schema()
        .channels()
        .iter()
        .find(|metadata| metadata.name == channel)
        .map(|metadata| metadata.id)
}

fn numeric_value(value: &TelemetryValue) -> Option<f64> {
    match value {
        TelemetryValue::Float64(value) => Some(*value),
        TelemetryValue::Int64(value) => Some(*value as f64),
        TelemetryValue::Bool(_) | TelemetryValue::Text(_) => None,
    }
}

fn event_matches(value: &TelemetryValue, equals: Option<&str>) -> bool {
    match value {
        TelemetryValue::Bool(value) => *value,
        TelemetryValue::Text(value) => equals.is_none_or(|expected| value == expected),
        TelemetryValue::Float64(_) | TelemetryValue::Int64(_) => false,
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::unwrap_used
)]
mod tests {
    use openbmp_core::{ChannelId, SimTime, StepIndex};
    use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema};
    use proptest::prelude::*;

    use super::*;
    use crate::ArtifactRef;

    fn table(values: &[f64], events: &[(f64, bool)]) -> TelemetryTable {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", None::<String>)
                .unwrap();
        let phase =
            TelemetryChannel::<String>::new(ChannelId::new(2), "phase", "1", None::<String>)
                .unwrap();
        let cutoff =
            TelemetryChannel::<bool>::new(ChannelId::new(3), "cutoff", "1", None::<String>)
                .unwrap();
        let schema = TelemetrySchema::new(vec![
            altitude.metadata().clone(),
            phase.metadata().clone(),
            cutoff.metadata().clone(),
        ])
        .unwrap();
        let mut table = TelemetryTable::new(schema);
        for (index, value) in values.iter().enumerate() {
            let time = SimTime::from_seconds(index as f64);
            let mut row = TelemetryRow::new(time, StepIndex::new(index as u64)).unwrap();
            row.insert(&altitude, *value).unwrap();
            row.insert(&phase, format!("phase-{index}")).unwrap();
            if events.iter().any(|(event_time, fired)| {
                (*event_time - time.as_seconds()).abs() <= f64::EPSILON && *fired
            }) {
                row.insert(&cutoff, true).unwrap();
            } else {
                row.insert(&cutoff, false).unwrap();
            }
            table.push_row(row).unwrap();
        }
        table
    }

    fn baseline_record(values: &[f64]) -> RunRecord {
        let artifact = ArtifactRef::new(
            "scenario",
            "scenario.toml",
            "0000000000000000000000000000000000000000000000000000000000000000",
        )
        .unwrap();
        RunRecord::new(
            "1111111111111111111111111111111111111111111111111111111111111111",
            7,
            "verdict-test",
            channel_digests(&table(values, &[])),
            vec![artifact],
            Vec::new(),
            Vec::new(),
        )
        .unwrap()
    }

    #[test]
    fn limit_rule_passes_and_fails_exactly() {
        let rules = [
            VerdictRule::Limit {
                id: "alt-max-pass".to_owned(),
                channel: "altitude_m".to_owned(),
                metric: ChannelMetric::Max,
                min: None,
                max: Some(3.0),
            },
            VerdictRule::Limit {
                id: "alt-max-fail".to_owned(),
                channel: "altitude_m".to_owned(),
                metric: ChannelMetric::Max,
                min: None,
                max: Some(2.0),
            },
        ];
        let report = evaluate_rules(&rules, &table(&[1.0, 2.0, 3.0], &[]), None);
        assert_eq!(report.verdicts[0].status, VerdictStatus::Pass);
        assert_eq!(report.verdicts[1].status, VerdictStatus::Fail);
        assert!(!report.passed());
    }

    #[test]
    fn tolerance_and_band_rules_use_inclusive_bounds() {
        let rules = [
            VerdictRule::ToleranceVsGolden {
                id: "final-golden".to_owned(),
                channel: "altitude_m".to_owned(),
                metric: ChannelMetric::Final,
                expected: 3.0,
                abs_tol: 0.0,
                rel_tol: 0.0,
            },
            VerdictRule::StatisticalBand {
                id: "mean-band".to_owned(),
                channel: "altitude_m".to_owned(),
                metric: ChannelMetric::Mean,
                lower: 1.9,
                upper: 2.1,
            },
        ];
        let report = evaluate_rules(&rules, &table(&[1.0, 2.0, 3.0], &[]), None);
        assert!(report.passed());
    }

    #[test]
    fn monotonic_rule_detects_order_violation() {
        let rules = [VerdictRule::Monotonic {
            id: "alt-increasing".to_owned(),
            channel: "altitude_m".to_owned(),
            direction: MonotonicDirection::Increasing,
            allow_equal: true,
        }];
        let pass = evaluate_rules(&rules, &table(&[1.0, 1.0, 2.0], &[]), None);
        let fail = evaluate_rules(&rules, &table(&[1.0, 3.0, 2.0], &[]), None);
        assert_eq!(pass.verdicts[0].status, VerdictStatus::Pass);
        assert_eq!(fail.verdicts[0].status, VerdictStatus::Fail);
    }

    #[test]
    fn event_window_rule_uses_first_matching_event_time() {
        let rules = [
            VerdictRule::EventWindow {
                id: "cutoff-pass".to_owned(),
                channel: "cutoff".to_owned(),
                equals: None,
                start_s: 1.0,
                end_s: 2.0,
            },
            VerdictRule::EventWindow {
                id: "cutoff-fail".to_owned(),
                channel: "cutoff".to_owned(),
                equals: None,
                start_s: 2.5,
                end_s: 3.0,
            },
        ];
        let report = evaluate_rules(&rules, &table(&[1.0, 2.0, 3.0], &[(2.0, true)]), None);
        assert_eq!(report.verdicts[0].status, VerdictStatus::Pass);
        assert_eq!(report.verdicts[0].measured, Some(2.0));
        assert_eq!(report.verdicts[1].status, VerdictStatus::Fail);
    }

    #[test]
    fn cross_run_delta_uses_baseline_record_and_fails_closed_without_one() {
        let rules = [VerdictRule::CrossRunDelta {
            id: "final-delta".to_owned(),
            channel: "altitude_m".to_owned(),
            metric: ChannelMetric::Final,
            max_abs_delta: 1.0,
        }];
        let current = table(&[1.0, 2.0, 3.0], &[]);
        let baseline = baseline_record(&[1.0, 2.0, 2.0]);
        let pass = evaluate_rules(&rules, &current, Some(&baseline));
        let fail = evaluate_rules(&rules, &current, None);
        assert_eq!(pass.verdicts[0].status, VerdictStatus::Pass);
        assert_eq!(pass.verdicts[0].measured, Some(1.0));
        assert_eq!(fail.verdicts[0].status, VerdictStatus::Fail);
    }

    #[test]
    fn unknown_channel_fails_closed() {
        let rules = [VerdictRule::Limit {
            id: "missing".to_owned(),
            channel: "missing_m".to_owned(),
            metric: ChannelMetric::Final,
            min: None,
            max: Some(1.0),
        }];
        let report = evaluate_rules(&rules, &table(&[1.0], &[]), None);
        assert_eq!(report.verdicts[0].status, VerdictStatus::Fail);
        assert!(report.verdicts[0].detail.contains("was not found"));
    }

    #[test]
    fn toml_document_is_closed_and_report_json_is_stable() {
        let document = VerdictRuleDocument::from_toml_str(
            r#"
                [[rules]]
                kind = "limit"
                id = "alt-max"
                channel = "altitude_m"
                metric = "max"
                max = 3.0
            "#,
        )
        .unwrap();
        let table = table(&[1.0, 2.0, 3.0], &[]);
        let a = document.evaluate(&table, None).to_json_bytes().unwrap();
        let b = document.evaluate(&table, None).to_json_bytes().unwrap();
        assert_eq!(a, b);

        let err = VerdictRuleDocument::from_toml_str(
            r#"
                [[rules]]
                kind = "limit"
                id = "alt-max"
                channel = "altitude_m"
                metric = "max"
                max = 3.0
                unexpected = true
            "#,
        )
        .unwrap_err();
        assert!(matches!(err, VerdictError::Toml { .. }));
    }

    #[test]
    fn fixture_document_covers_all_rule_forms() {
        let document = VerdictRuleDocument::from_toml_str(include_str!(
            "../tests/fixtures/verdict-rules.toml"
        ))
        .unwrap();
        let current = table(&[1.0, 2.0, 3.0], &[(2.0, true)]);
        let baseline = baseline_record(&[1.0, 2.0, 2.0]);
        let report = document.evaluate(&current, Some(&baseline));
        assert_eq!(report.verdicts.len(), 6);
        assert!(report.passed());
        let golden = include_bytes!("../tests/fixtures/verdict-report.json");
        let golden = golden.strip_suffix(b"\n").unwrap_or(golden);
        assert_eq!(report.to_json_bytes().unwrap(), golden);
    }

    #[test]
    fn validation_rejects_duplicate_ids_and_bad_bounds() {
        let duplicate = VerdictRuleDocument::from_toml_str(
            r#"
                [[rules]]
                kind = "limit"
                id = "same"
                channel = "altitude_m"
                metric = "max"
                max = 3.0

                [[rules]]
                kind = "limit"
                id = "same"
                channel = "altitude_m"
                metric = "max"
                max = 4.0
            "#,
        )
        .unwrap_err();
        assert!(matches!(duplicate, VerdictError::DuplicateRuleId { .. }));

        let bad_bounds = VerdictRuleDocument::from_toml_str(
            r#"
                [[rules]]
                kind = "statistical_band"
                id = "bad-band"
                channel = "altitude_m"
                metric = "mean"
                lower = 2.0
                upper = 1.0
            "#,
        )
        .unwrap_err();
        assert!(matches!(bad_bounds, VerdictError::InvalidRule { .. }));
    }

    proptest! {
        #[test]
        fn limit_rule_matches_inclusive_upper_bound(value in -1.0e6_f64..1.0e6, max in -1.0e6_f64..1.0e6) {
            let rules = [VerdictRule::Limit {
                id: "limit".to_owned(),
                channel: "altitude_m".to_owned(),
                metric: ChannelMetric::Final,
                min: None,
                max: Some(max),
            }];
            let report = evaluate_rules(&rules, &table(&[value], &[]), None);
            let expected = if value <= max {
                VerdictStatus::Pass
            } else {
                VerdictStatus::Fail
            };
            prop_assert_eq!(report.verdicts[0].status, expected);
            prop_assert_eq!(report.verdicts[0].measured, Some(value));
        }

        #[test]
        fn monotonic_increasing_rule_accepts_sorted_samples(mut values in prop::collection::vec(-1.0e6_f64..1.0e6, 1..32)) {
            values.sort_by(f64::total_cmp);
            let rules = [VerdictRule::Monotonic {
                id: "monotonic".to_owned(),
                channel: "altitude_m".to_owned(),
                direction: MonotonicDirection::Increasing,
                allow_equal: true,
            }];
            let report = evaluate_rules(&rules, &table(&values, &[]), None);
            prop_assert_eq!(report.verdicts[0].status, VerdictStatus::Pass);
        }

        #[test]
        fn unknown_numeric_channel_fails_closed(channel in "[a-z_]{1,12}", value in -1.0e6_f64..1.0e6) {
            prop_assume!(channel != "altitude_m");
            let rules = [VerdictRule::Limit {
                id: "missing".to_owned(),
                channel,
                metric: ChannelMetric::Final,
                min: None,
                max: Some(value),
            }];
            let report = evaluate_rules(&rules, &table(&[value], &[]), None);
            prop_assert_eq!(report.verdicts[0].status, VerdictStatus::Fail);
        }
    }
}
