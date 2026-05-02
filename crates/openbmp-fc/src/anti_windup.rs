//! PID anti-windup strategies (Phase 5.A.3.A).
//!
//! Saturating actuators interact badly with classical PID integral
//! action: when the commanded torque exceeds the effector limit, the
//! integrator continues to accumulate error even though the plant
//! cannot respond, leading to large overshoot once the saturation
//! clears. Two textbook remedies are exposed here:
//!
//! - [`AntiWindupKind::BackCalculation`] — the original Åström &
//!   Wittenmark (1984) construction. The integrator is bled at a rate
//!   proportional to how far the unsaturated command exceeds the
//!   limit. The proportionality constant is a unitless **gain** picked
//!   by the integrator's designer.
//!
//! - [`AntiWindupKind::ObserverForm`] — the SISO tracking-time
//!   reduction of the Åström & Rundqwist (1989) observer
//!   interpretation. The integrator is driven by a first-order
//!   tracking error with **tracking time constant** `T_t` (seconds).
//!   For this scalar PID implementation it collapses to the same
//!   bleed equation as back-calculation with `gain = 1/T_t`, but the
//!   parameter is units-aware and chosen by observer pole placement
//!   rather than trial-and-error.
//!
//! The two parameterisations are mathematically equivalent on a SISO
//! PID; the user-facing distinction is a design intent the scenario
//! declares. Full state-space observer injection for MIMO controllers
//! is a richer construction and remains outside this scalar helper.
//!
//! # References
//!
//! - Åström K.J., Wittenmark B. (1984). *Computer-Controlled Systems:
//!   Theory and Design*. Prentice-Hall.
//! - Åström K.J., Rundqwist L. (1989). *Integrator Windup and How to
//!   Avoid It.* American Control Conference, pp. 1693–1698.

use thiserror::Error;

/// Strategy used to bleed the PID integrator when the command
/// saturates.
///
/// The default is [`AntiWindupKind::BackCalculation`] with `gain =
/// 1.0`, matching Phase-4 behaviour.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AntiWindupKind {
    /// Åström-Wittenmark 1984 back-calculation. The integrator is
    /// bled by `gain * excess * dt` whenever the unsaturated command
    /// exceeds the actuator limit. `gain` is unitless and tuned
    /// empirically.
    BackCalculation {
        /// Back-calculation gain `k_aw`. Must be `> 0`.
        gain: f64,
    },
    /// SISO tracking-time reduction of Åström-Rundqwist 1989
    /// observer-form anti-windup. The integrator is bled by
    /// `excess * dt / tracking_time_s`, where `tracking_time_s` is
    /// the observer time constant (seconds) chosen by pole placement.
    /// Mathematically equivalent to back-calculation with
    /// `gain = 1/tracking_time_s` for this scalar PID helper.
    ObserverForm {
        /// Observer tracking time constant `T_t` (seconds). Must be
        /// `> 0`.
        tracking_time_s: f64,
    },
}

impl Default for AntiWindupKind {
    /// Phase-4 default: back-calculation with unit gain.
    fn default() -> Self {
        Self::BackCalculation { gain: 1.0 }
    }
}

/// Validation errors raised by [`AntiWindupKind::validate`].
#[derive(Copy, Clone, Debug, Error, PartialEq)]
pub enum AntiWindupError {
    /// `BackCalculation { gain }` with non-positive `gain`.
    #[error("anti-windup back-calculation gain must be > 0; got {gain}")]
    NonPositiveGain {
        /// Offending value.
        gain: f64,
    },
    /// `ObserverForm { tracking_time_s }` with non-positive
    /// `tracking_time_s`.
    #[error("anti-windup observer-form tracking time must be > 0 s; got {tracking_time_s}")]
    NonPositiveTrackingTime {
        /// Offending value.
        tracking_time_s: f64,
    },
    /// A non-finite parameter (NaN or infinite) was passed.
    #[error("anti-windup parameter must be finite; got {value}")]
    NonFiniteParameter {
        /// Offending value.
        value: f64,
    },
}

impl AntiWindupKind {
    /// Validate the parameters of this anti-windup kind.
    ///
    /// # Errors
    ///
    /// Returns [`AntiWindupError::NonPositiveGain`] /
    /// [`AntiWindupError::NonPositiveTrackingTime`] when the relevant
    /// parameter is non-positive, or
    /// [`AntiWindupError::NonFiniteParameter`] when it is NaN or
    /// infinite.
    pub fn validate(&self) -> Result<(), AntiWindupError> {
        match *self {
            Self::BackCalculation { gain } => {
                if !gain.is_finite() {
                    return Err(AntiWindupError::NonFiniteParameter { value: gain });
                }
                if gain <= 0.0 {
                    return Err(AntiWindupError::NonPositiveGain { gain });
                }
            }
            Self::ObserverForm { tracking_time_s } => {
                if !tracking_time_s.is_finite() {
                    return Err(AntiWindupError::NonFiniteParameter {
                        value: tracking_time_s,
                    });
                }
                if tracking_time_s <= 0.0 {
                    return Err(AntiWindupError::NonPositiveTrackingTime { tracking_time_s });
                }
            }
        }
        Ok(())
    }

    /// The effective bleed rate `r` such that the integrator update
    /// is `I -= r * excess * dt`. For back-calculation `r = gain`;
    /// for observer-form `r = 1 / tracking_time_s`.
    #[must_use]
    pub fn bleed_rate(&self) -> f64 {
        match *self {
            Self::BackCalculation { gain } => gain,
            Self::ObserverForm { tracking_time_s } => 1.0 / tracking_time_s,
        }
    }

    /// Apply the bleed to the integrator state in-place. `excess` is
    /// `raw - clamped` (positive when the command saturated above the
    /// limit, negative when below). `dt` is the loop step in seconds.
    /// No-op when `excess == 0` (no saturation).
    pub fn apply(&self, integral: &mut f64, excess: f64, dt: f64) {
        if excess == 0.0 || dt <= 0.0 {
            return;
        }
        *integral -= self.bleed_rate() * excess * dt;
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn default_is_back_calculation_unit_gain() {
        let aw = AntiWindupKind::default();
        assert_eq!(aw, AntiWindupKind::BackCalculation { gain: 1.0 });
        assert_eq!(aw.bleed_rate(), 1.0);
    }

    #[test]
    fn back_calculation_validate_rejects_non_positive_gain() {
        for gain in [0.0, -1.0, -1.0e-12] {
            let err = AntiWindupKind::BackCalculation { gain }
                .validate()
                .expect_err("non-positive gain rejected");
            assert!(matches!(err, AntiWindupError::NonPositiveGain { .. }));
        }
    }

    #[test]
    fn observer_form_validate_rejects_non_positive_time_constant() {
        for tracking_time_s in [0.0, -1.0, -1.0e-12] {
            let err = AntiWindupKind::ObserverForm { tracking_time_s }
                .validate()
                .expect_err("non-positive tracking time rejected");
            assert!(matches!(
                err,
                AntiWindupError::NonPositiveTrackingTime { .. }
            ));
        }
    }

    #[test]
    fn validate_rejects_non_finite_parameters() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(matches!(
                AntiWindupKind::BackCalculation { gain: bad }.validate(),
                Err(AntiWindupError::NonFiniteParameter { .. })
            ));
            assert!(matches!(
                AntiWindupKind::ObserverForm {
                    tracking_time_s: bad
                }
                .validate(),
                Err(AntiWindupError::NonFiniteParameter { .. })
            ));
        }
    }

    #[test]
    fn back_calc_and_observer_form_collapse_to_same_bleed_when_gain_matches_inverse_time() {
        let bc = AntiWindupKind::BackCalculation { gain: 4.0 };
        let of = AntiWindupKind::ObserverForm {
            tracking_time_s: 0.25,
        };
        assert_abs_diff_eq!(bc.bleed_rate(), of.bleed_rate(), epsilon = 1.0e-15);
    }

    #[test]
    fn apply_is_noop_when_excess_is_zero() {
        let aw = AntiWindupKind::BackCalculation { gain: 10.0 };
        let mut i = 1.5;
        aw.apply(&mut i, 0.0, 0.001);
        assert_eq!(i, 1.5);
    }

    #[test]
    fn apply_is_noop_when_dt_is_non_positive() {
        let aw = AntiWindupKind::BackCalculation { gain: 10.0 };
        let mut i = 1.5;
        aw.apply(&mut i, 0.5, 0.0);
        assert_eq!(i, 1.5);
        aw.apply(&mut i, 0.5, -0.001);
        assert_eq!(i, 1.5);
    }

    #[test]
    fn apply_bleeds_proportionally_to_excess() {
        let aw = AntiWindupKind::BackCalculation { gain: 2.0 };
        let mut i = 0.0;
        aw.apply(&mut i, 0.5, 0.1);
        // I -= 2.0 * 0.5 * 0.1 = 0.1
        assert_abs_diff_eq!(i, -0.1, epsilon = 1.0e-15);
    }

    #[test]
    fn apply_bleeds_negative_excess_back_into_integrator() {
        let aw = AntiWindupKind::ObserverForm {
            tracking_time_s: 0.5,
        };
        let mut i = 1.0;
        aw.apply(&mut i, -0.25, 0.05);
        // I -= (1/0.5) * (-0.25) * 0.05 = -0.025  →  I = 1.025
        assert_abs_diff_eq!(i, 1.025, epsilon = 1.0e-15);
    }

    #[test]
    fn observer_form_bleeds_faster_when_tracking_time_shrinks() {
        let mut i_slow = 1.0;
        let mut i_fast = 1.0;
        let slow = AntiWindupKind::ObserverForm {
            tracking_time_s: 1.0,
        };
        let fast = AntiWindupKind::ObserverForm {
            tracking_time_s: 0.05,
        };
        slow.apply(&mut i_slow, 0.5, 0.001);
        fast.apply(&mut i_fast, 0.5, 0.001);
        // |Δfast| = 20 |Δslow|
        let drop_slow = 1.0 - i_slow;
        let drop_fast = 1.0 - i_fast;
        // Subtraction-then-scaling FP noise on the order of 1e-14;
        // exact equality unobtainable for 0.99 etc.
        assert_abs_diff_eq!(drop_fast, 20.0 * drop_slow, epsilon = 1.0e-14);
    }

    #[test]
    fn under_sustained_saturation_integrator_remains_bounded() {
        // Steady-state check: with excess = 1 N·m held constant and
        // back-calc gain = 10, the integrator is driven toward
        // -∞ by the bleed term but bounded in any finite time. After
        // 1000 steps at dt = 0.001, integrator should be around
        // -10 (= -gain * excess * total_time).
        let aw = AntiWindupKind::BackCalculation { gain: 10.0 };
        let mut i = 0.0;
        for _ in 0..1000 {
            aw.apply(&mut i, 1.0, 0.001);
        }
        assert_abs_diff_eq!(i, -10.0, epsilon = 1.0e-12);
        assert!(i.is_finite());
    }
}
