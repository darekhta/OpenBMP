//! Code-verification helpers: observed order of accuracy and
//! Richardson/GCI summaries.

use crate::TestkitError;

/// One endpoint-error sample at a fixed step size.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepError {
    /// Fixed step size in seconds.
    pub step_s: f64,
    /// Absolute endpoint error at this step size.
    pub error: f64,
}

/// Order-of-accuracy and Richardson/GCI report for a halving study.
#[derive(Clone, Debug, PartialEq)]
pub struct OrderVerificationReport {
    /// Integrator or method label.
    pub method: String,
    /// Formal method order used for acceptance context.
    pub formal_order: f64,
    /// Refinement ratio. WP-11.5 uses `2`.
    pub refinement_ratio: f64,
    /// Roache GCI safety factor. WP-11.5 uses `1.25`.
    pub safety_factor: f64,
    /// Coarse-grid endpoint error.
    pub coarse: StepError,
    /// Medium-grid endpoint error.
    pub medium: StepError,
    /// Fine-grid endpoint error.
    pub fine: StepError,
    /// Observed order from `coarse/medium`.
    pub observed_order_coarse_medium: f64,
    /// Observed order from `medium/fine`.
    pub observed_order_medium_fine: f64,
    /// Richardson-estimated fine-grid discretization error.
    pub richardson_error_fine: f64,
    /// Fine-grid GCI bar, `Fs * richardson_error_fine`.
    pub gci_fine: f64,
    /// Medium-grid GCI bar.
    pub gci_medium: f64,
    /// Whether the GCI bar decreases under refinement.
    pub gci_monotone: bool,
    /// Whether the GCI bar covers the Richardson-estimated error.
    pub gci_brackets_richardson_error: bool,
}

impl OrderVerificationReport {
    /// Returns `true` when both observed-order estimates fall within
    /// the supplied inclusive band.
    #[must_use]
    pub fn observed_order_in_band(&self, min: f64, max: f64) -> bool {
        (min..=max).contains(&self.observed_order_coarse_medium)
            && (min..=max).contains(&self.observed_order_medium_fine)
    }
}

/// Build an order-verification report from a three-level step-halving
/// study.
///
/// # Errors
///
/// Returns [`TestkitError`] when any step or error is non-finite,
/// non-positive, or when the supplied step sizes are not an exact
/// halving sequence within a tight floating-point tolerance.
pub fn order_verification_report(
    method: impl Into<String>,
    formal_order: f64,
    coarse: StepError,
    medium: StepError,
    fine: StepError,
) -> Result<OrderVerificationReport, TestkitError> {
    validate_step_error("coarse", coarse)?;
    validate_step_error("medium", medium)?;
    validate_step_error("fine", fine)?;
    require_positive_finite("formal_order", formal_order)?;

    let ratio_cm = coarse.step_s / medium.step_s;
    let ratio_mf = medium.step_s / fine.step_s;
    if (ratio_cm - ratio_mf).abs() > 1.0e-12 || (ratio_cm - 2.0).abs() > 1.0e-12 {
        return Err(TestkitError::InvalidVerificationInput {
            field: "step_refinement",
            value: ratio_cm,
            rule: "step sizes must form a 2:1 halving sequence",
        });
    }
    if coarse.error <= medium.error || medium.error <= fine.error {
        return Err(TestkitError::InvalidVerificationInput {
            field: "endpoint_error",
            value: fine.error,
            rule: "errors must decrease monotonically under refinement",
        });
    }

    let observed_order_coarse_medium = (coarse.error / medium.error).ln() / ratio_cm.ln();
    let observed_order_medium_fine = (medium.error / fine.error).ln() / ratio_mf.ln();
    let richardson_denominator = ratio_mf.powf(observed_order_medium_fine) - 1.0;
    if !richardson_denominator.is_finite() || richardson_denominator <= 0.0 {
        return Err(TestkitError::InvalidVerificationInput {
            field: "richardson_denominator",
            value: richardson_denominator,
            rule: "must be positive and finite",
        });
    }
    let safety_factor = 1.25;
    let richardson_error_fine = fine.error / richardson_denominator;
    let richardson_error_medium = medium.error / richardson_denominator;
    let gci_fine = safety_factor * richardson_error_fine;
    let gci_medium = safety_factor * richardson_error_medium;
    Ok(OrderVerificationReport {
        method: method.into(),
        formal_order,
        refinement_ratio: ratio_mf,
        safety_factor,
        coarse,
        medium,
        fine,
        observed_order_coarse_medium,
        observed_order_medium_fine,
        richardson_error_fine,
        gci_fine,
        gci_medium,
        gci_monotone: gci_fine < gci_medium,
        gci_brackets_richardson_error: gci_fine >= richardson_error_fine,
    })
}

fn validate_step_error(label: &'static str, sample: StepError) -> Result<(), TestkitError> {
    require_positive_finite(label, sample.step_s)?;
    require_positive_finite(label, sample.error)
}

fn require_positive_finite(field: &'static str, value: f64) -> Result<(), TestkitError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(TestkitError::InvalidVerificationInput {
            field,
            value,
            rule: "must be positive and finite",
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn order_report_recovers_fourth_order_and_monotone_gci() -> Result<(), TestkitError> {
        let report = order_verification_report(
            "synthetic-rk4",
            4.0,
            StepError {
                step_s: 0.2,
                error: 16.0e-6,
            },
            StepError {
                step_s: 0.1,
                error: 1.0e-6,
            },
            StepError {
                step_s: 0.05,
                error: 6.25e-8,
            },
        )?;
        assert!(report.observed_order_in_band(3.8, 4.2));
        assert!(report.gci_monotone);
        assert!(report.gci_brackets_richardson_error);
        Ok(())
    }

    #[test]
    fn order_report_rejects_non_halving_steps() -> Result<(), Box<dyn std::error::Error>> {
        let result = order_verification_report(
            "bad",
            4.0,
            StepError {
                step_s: 0.2,
                error: 16.0e-6,
            },
            StepError {
                step_s: 0.12,
                error: 1.0e-6,
            },
            StepError {
                step_s: 0.05,
                error: 6.25e-8,
            },
        );
        match result {
            Err(TestkitError::InvalidVerificationInput {
                field: "step_refinement",
                ..
            }) => Ok(()),
            Err(err) => Err(std::io::Error::other(format!("unexpected error: {err}")).into()),
            Ok(report) => Err(std::io::Error::other(format!(
                "expected non-halving error, got {report:?}"
            ))
            .into()),
        }
    }
}
