//! `expected.toml` parser and check helpers.
//!
//! Per `docs/verification.md § Tolerance Tables`, every analytic-toy
//! and public-benchmark validation case ships an `expected.toml` file
//! with the canonical reference values and tolerances. This module
//! parses that file and provides per-metric checks.

use std::path::Path;

use openbmp_core::ValidationStatus;
use serde::Deserialize;

use crate::error::TestkitError;

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
    /// Metric name (e.g., `peak_deceleration_g`).
    pub name: String,
    /// Reference value.
    pub expected: f64,
    /// Maximum allowed `|expected - actual|`.
    pub absolute_tolerance: f64,
    /// Maximum allowed `|expected - actual| / max(|expected|, ε)`.
    pub relative_tolerance: f64,
}

impl ToleranceTable {
    /// Parse a tolerance table from a TOML string.
    ///
    /// # Errors
    ///
    /// Returns [`TestkitError::ParseToml`] on parser failure.
    pub fn from_toml_str(s: &str) -> Result<Self, TestkitError> {
        toml::from_str(s).map_err(|source| TestkitError::ParseToml { source })
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

    /// Check that an actual value is within tolerance for the named
    /// metric.
    ///
    /// # Errors
    ///
    /// Returns [`TestkitError::UnknownMetric`] if the metric name is
    /// not declared, or [`TestkitError::MetricOutOfTolerance`] if the
    /// actual value exceeds the absolute *and* relative envelopes.
    pub fn check_metric(&self, name: &str, actual: f64) -> Result<(), TestkitError> {
        let metric = self
            .metric(name)
            .ok_or_else(|| TestkitError::UnknownMetric {
                name: name.to_string(),
            })?;
        let abs_diff = (metric.expected - actual).abs();
        let rel_denominator = metric.expected.abs().max(f64::MIN_POSITIVE);
        let within_abs = abs_diff <= metric.absolute_tolerance;
        let within_rel = (abs_diff / rel_denominator) <= metric.relative_tolerance;
        if within_abs || within_rel {
            Ok(())
        } else {
            Err(TestkitError::MetricOutOfTolerance {
                name: name.to_string(),
                expected: metric.expected,
                actual,
                abs_diff,
                absolute_tolerance: metric.absolute_tolerance,
                relative_tolerance: metric.relative_tolerance,
            })
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"
        case = "allen-eggers-ballistic-entry"
        source = "NACA Report 1381"
        validation = "validated-toy"

        [[metric]]
        name = "peak_deceleration_g"
        expected = 12.3
        absolute_tolerance = 0.05
        relative_tolerance = 0.01

        [[metric]]
        name = "peak_altitude_km"
        expected = 50.0
        absolute_tolerance = 0.5
        relative_tolerance = 0.01
    "#;

    #[test]
    fn parses_valid_document() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        assert_eq!(table.case, "allen-eggers-ballistic-entry");
        assert_eq!(table.metrics.len(), 2);
        assert_eq!(table.validation, ValidationStatus::ValidatedToy);
    }

    #[test]
    fn check_metric_passes_within_tolerance() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        // 12.31 vs expected 12.3, abs_diff 0.01 < 0.05 → pass.
        assert!(table.check_metric("peak_deceleration_g", 12.31).is_ok());
    }

    #[test]
    fn check_metric_fails_outside_tolerance() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        // 13.0 vs 12.3, abs 0.7 > 0.05 and rel 0.0569 > 0.01 → fail.
        let err = table.check_metric("peak_deceleration_g", 13.0).unwrap_err();
        assert!(matches!(err, TestkitError::MetricOutOfTolerance { .. }));
    }

    #[test]
    fn check_metric_unknown_returns_error() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        let err = table.check_metric("nonexistent_metric", 0.0).unwrap_err();
        assert!(matches!(err, TestkitError::UnknownMetric { .. }));
    }

    #[test]
    fn check_metric_passes_when_relative_tolerance_satisfies() {
        let table = ToleranceTable::from_toml_str(SAMPLE).unwrap();
        // 12.42 vs 12.3 → abs_diff 0.12 > 0.05 (abs fails),
        // but 0.12/12.3 = 0.0098 < 0.01 (rel passes). Either-or
        // semantics: pass.
        assert!(table.check_metric("peak_deceleration_g", 12.42).is_ok());
    }
}
