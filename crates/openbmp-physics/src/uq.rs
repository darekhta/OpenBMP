//! Phase-6.13 hypersonic UQ and credibility reporting.
//!
//! Scenario-level error-budget machinery: per-model uncertainty
//! contributions, root-sum-square propagation to a top-level
//! aggregate, and NASA-STD-7009B-style evidence summaries. The
//! reporting surface intentionally **does not** claim operational
//! suitability — every aggregate carries a documented validity
//! label and an inline reminder that the simulator is
//! academic-scope.

use crate::error::PhysicsError;

/// Validation status label per NASA-STD-7009B / AIAA G-077 vocabulary.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ValidationStatus {
    /// Implementation exists; no validation evidence yet.
    Experimental,
    /// Unit tests / property tests pass; no cross-tool comparison.
    Checked,
    /// Closed-form analytic-toy agreement documented.
    ValidatedToy,
    /// Public-benchmark cross-validation against textbook reference.
    Research,
}

impl ValidationStatus {
    /// Short label for telemetry / report rendering.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::Experimental => "experimental",
            Self::Checked => "checked",
            Self::ValidatedToy => "validated-toy",
            Self::Research => "research",
        }
    }
}

/// One model's uncertainty contribution to a scenario aggregate.
#[derive(Clone, Debug, PartialEq)]
pub struct UncertaintyContribution {
    /// Model identifier (e.g. "atmosphere", "aero-deck",
    /// "stagnation-heating").
    pub model_id: String,
    /// 1-σ uncertainty (units of the model's primary output).
    pub one_sigma: f64,
    /// Validation status label.
    pub status: ValidationStatus,
    /// Free-text justification — points at the validation case
    /// or external reference that produced the 1-σ estimate.
    pub justification: String,
}

impl UncertaintyContribution {
    /// Validate the contribution: non-negative σ, non-empty fields.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] on bad input.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        if self.model_id.is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "uncertainty contribution: model_id required",
            });
        }
        if !self.one_sigma.is_finite() || self.one_sigma < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "uncertainty contribution: one_sigma must be ≥ 0 and finite",
            });
        }
        if self.justification.is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "uncertainty contribution: justification required",
            });
        }
        Ok(())
    }
}

/// Scenario-level error budget. Aggregates per-model contributions
/// by root-sum-square (assuming independent contributors) plus a
/// monolithic correlated additive bias.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct ErrorBudget {
    /// Per-model independent contributions.
    pub contributions: Vec<UncertaintyContribution>,
    /// Additional correlated bias (units of the primary output)
    /// that should be added linearly (not RSS) to the aggregate.
    pub correlated_bias: f64,
}

impl ErrorBudget {
    /// Aggregate uncertainty = `sqrt(sum(sigma_i^2)) + correlated_bias`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when any
    /// contribution is malformed.
    pub fn aggregate_one_sigma(&self) -> Result<f64, PhysicsError> {
        for c in &self.contributions {
            c.validate()?;
        }
        if !self.correlated_bias.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "correlated bias must be finite",
            });
        }
        let sum_sq: f64 = self
            .contributions
            .iter()
            .map(|c| c.one_sigma * c.one_sigma)
            .sum();
        Ok(sum_sq.sqrt() + self.correlated_bias.abs())
    }

    /// Minimum validation status across all contributions. Returns
    /// `None` for an empty budget.
    #[must_use]
    pub fn minimum_status(&self) -> Option<ValidationStatus> {
        self.contributions
            .iter()
            .map(|c| c.status)
            .min_by_key(|s| match s {
                ValidationStatus::Experimental => 0,
                ValidationStatus::Checked => 1,
                ValidationStatus::ValidatedToy => 2,
                ValidationStatus::Research => 3,
            })
    }

    /// Render a short Markdown credibility report. Caller-provided
    /// `scenario_id` is included verbatim for traceability.
    #[must_use]
    pub fn render_markdown(&self, scenario_id: &str) -> String {
        use std::fmt::Write;
        let mut out = String::new();
        let _ = writeln!(out, "# Credibility report — {scenario_id}");
        let _ = writeln!(out);
        let _ = writeln!(
            out,
            "> Academic simulation; not validated for operational flight."
        );
        let _ = writeln!(out);
        let _ = writeln!(out, "| Model | 1-σ | Status | Justification |");
        let _ = writeln!(out, "|---|---|---|---|");
        for c in &self.contributions {
            let _ = writeln!(
                out,
                "| {} | {:.4e} | {} | {} |",
                c.model_id,
                c.one_sigma,
                c.status.label(),
                c.justification
            );
        }
        let _ = writeln!(out);
        if let Ok(agg) = self.aggregate_one_sigma() {
            let _ = writeln!(out, "**Aggregate 1-σ (RSS + correlated bias):** {agg:.4e}");
        }
        if let Some(s) = self.minimum_status() {
            let _ = writeln!(out, "**Minimum validation status:** {}", s.label());
        }
        out
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::missing_panics_doc,
    clippy::similar_names
)]
mod tests {
    use super::*;

    fn ctrib(id: &str, sigma: f64, status: ValidationStatus) -> UncertaintyContribution {
        UncertaintyContribution {
            model_id: id.into(),
            one_sigma: sigma,
            status,
            justification: "textbook reference".into(),
        }
    }

    #[test]
    fn well_formed_contribution_validates() {
        ctrib("atmos", 1.0e-5, ValidationStatus::ValidatedToy)
            .validate()
            .unwrap();
    }

    #[test]
    fn empty_id_rejected() {
        let mut c = ctrib("ok", 1.0, ValidationStatus::Checked);
        c.model_id = String::new();
        assert!(matches!(
            c.validate(),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn negative_sigma_rejected() {
        let mut c = ctrib("ok", -1.0, ValidationStatus::Checked);
        c.one_sigma = -1.0;
        assert!(matches!(
            c.validate(),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn aggregate_root_sum_square_plus_bias() {
        let budget = ErrorBudget {
            contributions: vec![
                ctrib("a", 3.0, ValidationStatus::Research),
                ctrib("b", 4.0, ValidationStatus::Research),
            ],
            correlated_bias: 1.0,
        };
        // √(3² + 4²) + 1 = 5 + 1 = 6.
        let agg = budget.aggregate_one_sigma().unwrap();
        assert!((agg - 6.0).abs() < 1e-12);
    }

    #[test]
    fn minimum_status_is_worst() {
        let budget = ErrorBudget {
            contributions: vec![
                ctrib("a", 1.0, ValidationStatus::Research),
                ctrib("b", 2.0, ValidationStatus::Checked),
                ctrib("c", 3.0, ValidationStatus::Experimental),
            ],
            correlated_bias: 0.0,
        };
        assert_eq!(
            budget.minimum_status(),
            Some(ValidationStatus::Experimental)
        );
    }

    #[test]
    fn empty_budget_aggregates_to_zero() {
        let budget = ErrorBudget::default();
        assert_eq!(budget.aggregate_one_sigma().unwrap(), 0.0);
        assert_eq!(budget.minimum_status(), None);
    }

    #[test]
    fn markdown_report_contains_aggregate_and_status() {
        let budget = ErrorBudget {
            contributions: vec![ctrib("atmos", 1.5e-2, ValidationStatus::ValidatedToy)],
            correlated_bias: 0.0,
        };
        let md = budget.render_markdown("test-scenario");
        assert!(md.contains("test-scenario"));
        assert!(md.contains("atmos"));
        assert!(md.contains("validated-toy"));
        assert!(md.contains("Aggregate"));
        assert!(md.contains("Academic simulation"));
    }
}
