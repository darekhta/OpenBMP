//! Declarative SIL acceptance checks.
//!
//! A [`SilCheck`] is a closed, tap-backed acceptance criterion evaluated over
//! a run's estimate-vs-truth observations. Closed by design: every variant
//! names exactly one scalar bound, exact mask, or tick count. There is no
//! free-form / `String` kind and no surface / range / aimpoint / impact /
//! miss-distance parameter, so a targeting requirement cannot be expressed.
//!
//! Forward-only by construction: [`SilCheck::evaluate`] is a pure reduction
//! run AFTER the simulation over already-recorded observations. It can never
//! feed back into the loop, and the only observables it can read are
//! estimate-vs-truth residual norms and vehicle-intrinsic / FDIR flags.

use serde::{Deserialize, Serialize};

use crate::monitor::SilObservationReport;
use crate::{RequirementVerdict, stop_label_is_nominal};

/// A closed declarative SIL acceptance check.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum SilCheck {
    /// The peak estimate-vs-truth position error must stay below `max_m`.
    NavPositionErrorBelow {
        /// Maximum allowed position residual norm (m).
        max_m: f64,
    },
    /// The peak estimate-vs-truth velocity error must stay below `max_m_s`.
    NavVelocityErrorBelow {
        /// Maximum allowed velocity residual norm (m/s).
        max_m_s: f64,
    },
    /// The peak attitude tracking error must stay below `max_rad`.
    AttitudeTrackingErrorBelow {
        /// Maximum allowed attitude error angle (rad).
        max_rad: f64,
    },
    /// The peak GNSS NEES `‖ν̃‖²` must stay at or below `max` (estimator
    /// consistency: an over-confident or diverging filter exceeds it).
    EstimatorConsistencyNeesWithin {
        /// Maximum allowed normalised estimation error squared.
        max: f64,
    },
    /// The estimator must not reject innovations on more than
    /// `max_rejected_ticks` ticks (a fault that storms the gate fails).
    NoInnovationRejectionStorm {
        /// Maximum allowed count of innovation-gate-rejection ticks.
        max_rejected_ticks: u64,
    },
    /// The union of FDIR tripped-detector masks over the run must equal
    /// `bits` exactly (fails on both under- and over-isolation).
    FdirIsolatesExactly {
        /// Expected tripped-detector bitmask.
        bits: u64,
    },
    /// The run must terminate on a nominal stop reason (not a divergence,
    /// ground impact, or guard-rail truncation).
    StopReasonIsNominal,
}

impl SilCheck {
    /// Stable requirement id for this check, used in evidence verdicts.
    #[must_use]
    pub const fn requirement_id(&self) -> &'static str {
        match self {
            Self::NavPositionErrorBelow { .. } => "SIL-NAV-POSITION-ERROR",
            Self::NavVelocityErrorBelow { .. } => "SIL-NAV-VELOCITY-ERROR",
            Self::AttitudeTrackingErrorBelow { .. } => "SIL-ATTITUDE-TRACKING-ERROR",
            Self::EstimatorConsistencyNeesWithin { .. } => "SIL-ESTIMATOR-CONSISTENCY-NEES",
            Self::NoInnovationRejectionStorm { .. } => "SIL-NO-INNOVATION-REJECTION-STORM",
            Self::FdirIsolatesExactly { .. } => "SIL-FDIR-ISOLATES-EXACTLY",
            Self::StopReasonIsNominal => "SIL-STOP-REASON-NOMINAL",
        }
    }

    /// Evaluate this check against a run's observation report and stop
    /// label, yielding a `pass` / `fail` / `skip` requirement verdict.
    ///
    /// All bound checks read the full-fidelity summary (every tick), so they
    /// are honest regardless of the sample decimation factor. A check whose
    /// observable was never produced (e.g. no position estimate on a
    /// non-`[fc]` run) is `skip`, never a silent `pass`.
    #[must_use]
    pub fn evaluate(
        &self,
        observations: &SilObservationReport,
        stop_label: &str,
    ) -> RequirementVerdict {
        let summary = &observations.summary;
        let (verdict, detail) = match self {
            Self::NavPositionErrorBelow { max_m } => {
                bound_below(summary.max_pos_err_m, *max_m, "max position error", "m")
            }
            Self::NavVelocityErrorBelow { max_m_s } => {
                bound_below(summary.max_vel_err_m_s, *max_m_s, "max velocity error", "m/s")
            }
            Self::AttitudeTrackingErrorBelow { max_rad } => {
                bound_below(summary.max_att_err_rad, *max_rad, "max attitude error", "rad")
            }
            Self::EstimatorConsistencyNeesWithin { max } => {
                bound_below(summary.max_nees, *max, "max NEES", "")
            }
            Self::NoInnovationRejectionStorm { max_rejected_ticks } => {
                let actual = summary.innovation_rejected_ticks;
                if actual <= *max_rejected_ticks {
                    ("pass", format!("innovation-rejected ticks {actual} <= {max_rejected_ticks}"))
                } else {
                    ("fail", format!("innovation-rejected ticks {actual} > {max_rejected_ticks}"))
                }
            }
            Self::FdirIsolatesExactly { bits } => {
                let actual = summary.fdir_tripped_mask_union;
                if actual == *bits {
                    ("pass", format!("FDIR tripped mask 0x{actual:016x} == expected 0x{bits:016x}"))
                } else {
                    ("fail", format!("FDIR tripped mask 0x{actual:016x} != expected 0x{bits:016x}"))
                }
            }
            Self::StopReasonIsNominal => {
                if stop_label_is_nominal(stop_label) {
                    ("pass", format!("stop reason `{stop_label}` is nominal"))
                } else {
                    ("fail", format!("stop reason `{stop_label}` is non-nominal"))
                }
            }
        };
        RequirementVerdict {
            id: self.requirement_id().to_owned(),
            verdict: verdict.to_owned(),
            summary: detail,
        }
    }
}

/// Evaluate an "observed value at or below a bound" check, skipping when the
/// observable was never produced.
fn bound_below(actual: Option<f64>, max: f64, label: &str, unit: &str) -> (&'static str, String) {
    match actual {
        None => ("skip", format!("{label}: not observed (estimate absent)")),
        Some(value) if value <= max => {
            ("pass", format!("{label} {value:.6} {unit} <= {max:.6} {unit}"))
        }
        Some(value) => ("fail", format!("{label} {value:.6} {unit} > {max:.6} {unit}")),
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::monitor::SilObservationSummary;

    fn report_with(summary: SilObservationSummary) -> SilObservationReport {
        SilObservationReport {
            decimation: 1,
            summary,
            samples: Vec::new(),
        }
    }

    #[test]
    fn bound_check_passes_under_limit_and_fails_over() {
        let pass = report_with(SilObservationSummary {
            max_pos_err_m: Some(5.0),
            ..SilObservationSummary::default()
        });
        let check = SilCheck::NavPositionErrorBelow { max_m: 10.0 };
        assert_eq!(check.evaluate(&pass, "end-time").verdict, "pass");

        let fail = report_with(SilObservationSummary {
            max_pos_err_m: Some(25.0),
            ..SilObservationSummary::default()
        });
        assert_eq!(check.evaluate(&fail, "end-time").verdict, "fail");
    }

    #[test]
    fn bound_check_skips_when_observable_absent() {
        // No position estimate was observed (max_pos_err_m is None): the
        // check must SKIP, never silently pass.
        let report = report_with(SilObservationSummary::default());
        let check = SilCheck::NavPositionErrorBelow { max_m: 1.0 };
        assert_eq!(check.evaluate(&report, "end-time").verdict, "skip");
    }

    #[test]
    fn stop_reason_nominal_check_reads_label() {
        let report = report_with(SilObservationSummary::default());
        let check = SilCheck::StopReasonIsNominal;
        assert_eq!(check.evaluate(&report, "end-time").verdict, "pass");
        assert_eq!(check.evaluate(&report, "ground-impact").verdict, "fail");
    }

    #[test]
    fn fdir_isolation_check_requires_exact_mask() {
        let report = report_with(SilObservationSummary {
            fdir_tripped_mask_union: 0b0010,
            ..SilObservationSummary::default()
        });
        assert_eq!(
            SilCheck::FdirIsolatesExactly { bits: 0b0010 }
                .evaluate(&report, "end-time")
                .verdict,
            "pass"
        );
        assert_eq!(
            SilCheck::FdirIsolatesExactly { bits: 0b0110 }
                .evaluate(&report, "end-time")
                .verdict,
            "fail"
        );
    }
}
