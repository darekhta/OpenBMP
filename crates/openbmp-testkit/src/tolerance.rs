//! `expected.toml` parser and check helpers.
//!
//! Per `docs/verification.md § Tolerance Tables`, every analytic-toy
//! and public-benchmark validation case ships an `expected.toml` file
//! with the canonical reference values and tolerances. This module
//! parses that file and provides per-metric checks.

use std::collections::BTreeSet;
use std::path::Path;

use openbmp_core::ValidationStatus;
use serde::Deserialize;

use crate::error::TestkitError;

/// Floor used when computing relative error for an expected value near
/// zero.
pub const RELATIVE_DENOMINATOR_FLOOR: f64 = f64::MIN_POSITIVE;

/// Top-level tolerance-table document.
#[derive(Clone, Debug, Deserialize)]
pub struct ToleranceTable {
    /// Validation case name (matches the scenario name).
    pub case: String,
    /// Source citation for the reference values.
    pub source: String,
    /// Validation label this table backs.
    pub validation: ValidationStatus,
    /// One entry per checked metric.
    #[serde(rename = "metric")]
    pub metrics: Vec<MetricTolerance>,
}

/// One metric entry in a tolerance table.
#[derive(Clone, Debug, Deserialize)]
pub struct MetricTolerance {
    /// Metric name (e.g., `final_position_z_m`).
    pub name: String,
    /// Reference value.
    pub expected: f64,
    /// Maximum allowed `|expected - actual|`.
    pub absolute_tolerance: f64,
    /// Maximum allowed `|expected - actual| / max(|expected|, eps)`.
    pub relative_tolerance: f64,
}

impl ToleranceTable {
    /// Parse a tolerance table from a TOML string.
    ///
    /// # Errors
    ///
    /// Returns [`TestkitError::ParseToml`] on parser failure.
    pub fn from_toml_str(s: &str) -> Result<Self, TestkitError> {
        let table: Self = toml::from_str(s).map_err(|source| TestkitError::ParseToml { source })?;
        table.require_valid()?;
        Ok(table)
    }

    /// Read and parse a tolerance table from `path`.
    ///
    /// # Errors
    ///
    /// Returns [`TestkitError::ReadFile`] on IO failure or
    /// [`TestkitError::ParseToml`] on parser failure.
    pub fn from_path(path: impl AsRef<Path>) -> Result<Self, TestkitError> {
        let path_ref = path.as_ref();
        let content =
            std::fs::read_to_string(path_ref).map_err(|source| TestkitError::ReadFile {
                path: path_ref.display().to_string(),
                source,
            })?;
        Self::from_toml_str(&content)
    }

    /// Look up a metric by name.
    #[must_use]
    pub fn metric(&self, name: &str) -> Option<&MetricTolerance> {
        self.metrics.iter().find(|m| m.name == name)
    }

    /// Validate table-level and metric-level invariants.
    ///
    /// # Errors
    ///
    /// Returns a [`TestkitError`] if there are no metrics, a metric
    /// name is empty or duplicated, an expected value is not finite, or
    /// either tolerance is negative or not finite.
    pub fn require_valid(&self) -> Result<(), TestkitError> {
        if self.metrics.is_empty() {
            return Err(TestkitError::EmptyToleranceTable);
        }
        let mut names = BTreeSet::new();
        for metric in &self.metrics {
            if metric.name.trim().is_empty() {
                return Err(TestkitError::EmptyMetricName);
            }
            if !names.insert(metric.name.as_str()) {
                return Err(TestkitError::DuplicateMetric {
                    name: metric.name.clone(),
                });
            }
            metric.require_valid()?;
        }
        Ok(())
    }

    /// Check that an actual value is within tolerance for the named
    /// metric.
    ///
    /// # Errors
    ///
    /// Returns [`TestkitError::UnknownMetric`] if the metric name is
    /// not declared, [`TestkitError::MetricActualNotFinite`] if the
    /// actual value is not finite, or
    /// [`TestkitError::MetricOutOfTolerance`] if the actual value
    /// exceeds the absolute *and* relative envelopes.
    pub fn check_metric(&self, name: &str, actual: f64) -> Result<(), TestkitError> {
        let metric = self
            .metric(name)
            .ok_or_else(|| TestkitError::UnknownMetric {
                name: name.to_string(),
            })?;
        metric.check(actual)
    }
}

impl MetricTolerance {
    /// Validate metric invariants.
    ///
    /// # Errors
    ///
    /// Returns a [`TestkitError`] if the expected value is not finite or
    /// a tolerance is negative or not finite.
    pub fn require_valid(&self) -> Result<(), TestkitError> {
        if !self.expected.is_finite() {
            return Err(TestkitError::InvalidMetricValue {
                name: self.name.clone(),
                field: "expected",
                value: self.expected,
            });
        }
        if !self.absolute_tolerance.is_finite() || self.absolute_tolerance < 0.0 {
            return Err(TestkitError::InvalidMetricValue {
                name: self.name.clone(),
                field: "absolute_tolerance",
                value: self.absolute_tolerance,
            });
        }
        if !self.relative_tolerance.is_finite() || self.relative_tolerance < 0.0 {
            return Err(TestkitError::InvalidMetricValue {
                name: self.name.clone(),
                field: "relative_tolerance",
                value: self.relative_tolerance,
            });
        }
        Ok(())
    }

    /// Check an actual value against this metric's tolerance envelope.
    ///
    /// # Errors
    ///
    /// Returns [`TestkitError::MetricActualNotFinite`] if `actual` is
    /// not finite, or [`TestkitError::MetricOutOfTolerance`] if it
    /// exceeds the absolute and relative envelopes.
    pub fn check(&self, actual: f64) -> Result<(), TestkitError> {
        if !actual.is_finite() {
            return Err(TestkitError::MetricActualNotFinite {
                name: self.name.clone(),
                actual,
            });
        }
        let abs_diff = (self.expected - actual).abs();
        let rel_denominator = self.expected.abs().max(RELATIVE_DENOMINATOR_FLOOR);
        let rel_diff = abs_diff / rel_denominator;
        let within_abs = abs_diff <= self.absolute_tolerance;
        let within_rel = rel_diff <= self.relative_tolerance;
        if within_abs || within_rel {
            Ok(())
        } else {
            Err(TestkitError::MetricOutOfTolerance {
                name: self.name.clone(),
                expected: self.expected,
                actual,
                abs_diff,
                rel_diff,
                absolute_tolerance: self.absolute_tolerance,
                relative_tolerance: self.relative_tolerance,
            })
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    // Parser-test fixture. The `case`, `source`, metric names, and
    // numerical values are intentionally synthetic — clean integers
    // chosen so the abs/rel tolerance arithmetic in the assertions
    // below is easy to verify by hand. This fixture does NOT
    // represent any real benchmark; real validation cases ship as
    // their own files under `data/scenarios/` (or
    // `crates/<crate>/tests/expected/`) with full provenance per
    // `docs/data-provenance.md`.
    const SAMPLE: &str = r#"
        case = "test-tolerance-fixture"
        source = "test fixture"
        validation = "validated-toy"

        [[metric]]
        name = "metric_a"
        expected = 10.0
        absolute_tolerance = 0.05
        relative_tolerance = 0.01

        [[metric]]
        name = "metric_b"
        expected = 100.0
        absolute_tolerance = 0.5
        relative_tolerance = 0.01
    "#;

    #[test]
    fn parses_valid_document() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        assert_eq!(table.case, "test-tolerance-fixture");
        assert_eq!(table.metrics.len(), 2);
        assert_eq!(table.validation, ValidationStatus::ValidatedToy);
    }

    #[test]
    fn check_metric_passes_within_tolerance() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        // 10.01 vs expected 10.0, abs_diff 0.01 < 0.05 → pass.
        assert!(table.check_metric("metric_a", 10.01).is_ok());
    }

    #[test]
    fn check_metric_fails_outside_tolerance() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        // 11.0 vs 10.0, abs 1.0 > 0.05 and rel 0.1 > 0.01 → fail.
        let err = table.check_metric("metric_a", 11.0).unwrap_err();
        assert!(matches!(err, TestkitError::MetricOutOfTolerance { .. }));
    }

    #[test]
    fn check_metric_unknown_returns_error() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        let err = table.check_metric("nonexistent_metric", 0.0).unwrap_err();
        assert!(matches!(err, TestkitError::UnknownMetric { .. }));
    }

    #[test]
    fn duplicate_metric_is_rejected_at_parse_time() {
        let duplicate = r#"
            case = "case"
            source = "source"
            validation = "experimental"

            [[metric]]
            name = "x"
            expected = 1.0
            absolute_tolerance = 0.1
            relative_tolerance = 0.1

            [[metric]]
            name = "x"
            expected = 1.0
            absolute_tolerance = 0.1
            relative_tolerance = 0.1
        "#;
        let err = ToleranceTable::from_toml_str(duplicate).unwrap_err();
        assert!(matches!(err, TestkitError::DuplicateMetric { .. }));
    }

    #[test]
    fn invalid_tolerance_is_rejected_at_parse_time() {
        let invalid = r#"
            case = "case"
            source = "source"
            validation = "experimental"

            [[metric]]
            name = "x"
            expected = 1.0
            absolute_tolerance = -0.1
            relative_tolerance = 0.1
        "#;
        let err = ToleranceTable::from_toml_str(invalid).unwrap_err();
        assert!(matches!(err, TestkitError::InvalidMetricValue { .. }));
    }

    #[test]
    fn check_metric_rejects_nan_actual() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        let err = table.check_metric("metric_a", f64::NAN).unwrap_err();
        assert!(matches!(err, TestkitError::MetricActualNotFinite { .. }));
    }

    #[test]
    fn check_metric_passes_when_relative_tolerance_satisfies() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        // 10.06 vs 10.0 → abs_diff 0.06 > 0.05 (abs fails),
        // but 0.06/10.0 = 0.006 < 0.01 (rel passes). Either-or
        // semantics: pass.
        assert!(table.check_metric("metric_a", 10.06).is_ok());
    }
}
