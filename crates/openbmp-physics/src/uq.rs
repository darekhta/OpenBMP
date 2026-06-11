//! Compatibility re-export for OpenBMP uncertainty quantification.
//!
//! The UQ primitives live in `openbmp-uq` (L1) so physics, model, and campaign
//! crates can share one source-tagged error-budget and credibility API without
//! duplicating validation-status labels.

pub use openbmp_uq::{
    CorrelatedErrorBudget, CorrelationMatrix, CredibilityFactor, CredibilityLevel,
    CredibilityRecord, ErrorBudget, UncertaintyClass, UncertaintyContribution, UncertaintySource,
    UqError, ValidationStatus,
};

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn physics_uq_module_reexports_core_validation_status() {
        assert_eq!(ValidationStatus::ValidatedToy.as_label(), "validated-toy");
    }

    #[test]
    fn physics_uq_module_reexports_legacy_budget() {
        let budget = ErrorBudget {
            contributions: vec![UncertaintyContribution {
                model_id: "atmos".to_owned(),
                one_sigma: 3.0,
                status: ValidationStatus::Checked,
                justification: "unit test".to_owned(),
            }],
            correlated_bias: 4.0,
        };
        let aggregate = budget.aggregate_one_sigma().expect("aggregate");
        assert!((aggregate - 7.0).abs() < 1.0e-12);
    }
}
