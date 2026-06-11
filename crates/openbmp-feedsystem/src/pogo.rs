//! Reduced POGO closed-loop stability primitive.
//!
//! This module provides a deterministic frequency-domain check for the feed
//! half of POGO. It intentionally starts with a small analytic model: one
//! longitudinal structural mode coupled to a first-order feed response whose
//! effective gain is attenuated by accumulator compliance.

use openbmp_core::ValidationStatus;

use crate::error::FeedSystemError;

/// Stability verdict for the reduced POGO loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PogoStabilityVerdict {
    /// The cubic closed-loop characteristic is asymptotically stable by the
    /// Routh-Hurwitz conditions.
    Stable,
    /// The effective feed gain overwhelms modal stiffness (`a0 <= 0`).
    StaticDivergence,
    /// The modal stiffness remains positive, but damping/phase margin is lost.
    DynamicInstability,
}

/// One longitudinal structural mode consumed by the POGO feed-half model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoModeConfig {
    /// Modal natural frequency in rad/s.
    pub natural_frequency_rad_s: f64,
    /// Modal damping ratio.
    pub damping_ratio: f64,
}

impl PogoModeConfig {
    /// Validate modal parameters.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite, non-positive frequency or a
    /// negative damping ratio.
    pub fn require_valid(self) -> Result<(), FeedSystemError> {
        require_positive_finite(
            self.natural_frequency_rad_s,
            "POGO mode natural frequency must be positive",
        )?;
        require_nonnegative_finite(
            self.damping_ratio,
            "POGO mode damping ratio must be finite and non-negative",
        )
    }
}

/// Feed-loop parameters for the reduced POGO stability check.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoFeedCouplingConfig {
    /// Open-loop modal feedback gain in rad^2/s^2 before accumulator
    /// attenuation.
    pub open_loop_gain_rad2: f64,
    /// First-order feed response time constant in seconds.
    pub feed_time_constant_s: f64,
    /// Effective feed zero in seconds representing mass-flow-gain phase lead.
    pub mass_flow_gain_time_s: f64,
    /// Cavitation compliance in m^3/Pa. This is the reference compliance that
    /// couples feed pressure back into the modal equation.
    pub cavitation_compliance_m3_per_pa: f64,
    /// Accumulator compliance in m^3/Pa. Larger values attenuate the effective
    /// feed feedback gain.
    pub accumulator_compliance_m3_per_pa: f64,
}

impl PogoFeedCouplingConfig {
    /// Validate feed-loop parameters.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite, negative, or unsupported
    /// feed-loop parameters.
    pub fn require_valid(self) -> Result<(), FeedSystemError> {
        require_nonnegative_finite(
            self.open_loop_gain_rad2,
            "POGO feed open-loop gain must be finite and non-negative",
        )?;
        require_positive_finite(
            self.feed_time_constant_s,
            "POGO feed time constant must be positive",
        )?;
        require_nonnegative_finite(
            self.mass_flow_gain_time_s,
            "POGO mass-flow-gain time must be finite and non-negative",
        )?;
        require_positive_finite(
            self.cavitation_compliance_m3_per_pa,
            "POGO cavitation compliance must be positive",
        )?;
        require_nonnegative_finite(
            self.accumulator_compliance_m3_per_pa,
            "POGO accumulator compliance must be finite and non-negative",
        )
    }

    fn accumulator_attenuation(self) -> Result<f64, FeedSystemError> {
        let attenuation = self.cavitation_compliance_m3_per_pa
            / (self.cavitation_compliance_m3_per_pa + self.accumulator_compliance_m3_per_pa);
        if !attenuation.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "POGO accumulator attenuation is non-finite",
            });
        }
        Ok(attenuation)
    }
}

/// Configuration for the reduced POGO stability primitive.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoStabilityConfig {
    /// Longitudinal structural mode.
    pub mode: PogoModeConfig,
    /// Linearized feed-loop coupling.
    pub feed: PogoFeedCouplingConfig,
}

impl PogoStabilityConfig {
    /// Validate the full POGO configuration.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when any modal or feed-loop parameter is
    /// invalid.
    pub fn require_valid(self) -> Result<(), FeedSystemError> {
        self.mode.require_valid()?;
        self.feed.require_valid()
    }
}

/// Result of one reduced POGO stability analysis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoStabilitySnapshot {
    /// Accumulator attenuation factor applied to the open-loop gain.
    pub accumulator_attenuation: f64,
    /// Effective closed-loop feed gain after accumulator attenuation.
    pub effective_open_loop_gain_rad2: f64,
    /// Characteristic coefficients `[a3, a2, a1, a0]` for
    /// `a3*s^3 + a2*s^2 + a1*s + a0`.
    pub characteristic_coefficients: [f64; 4],
    /// Stiffness margin `a0 = omega^2 - gain`.
    pub static_margin_rad2: f64,
    /// Cubic Routh-Hurwitz margin `a2*a1 - a3*a0`.
    pub routh_hurwitz_margin: f64,
    /// Critical effective gain at the neutral stability boundary.
    pub critical_effective_gain_rad2: f64,
    /// Exact accumulator compliance that would place the current open-loop
    /// gain on the neutral boundary. `None` means no finite gain boundary is
    /// present for this reduced model.
    pub neutral_accumulator_compliance_m3_per_pa: Option<f64>,
    /// Stability verdict.
    pub verdict: PogoStabilityVerdict,
    /// Validation posture of this reduced model.
    pub validation: ValidationStatus,
}

/// Reduced POGO stability analyzer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoStability {
    config: PogoStabilityConfig,
}

impl PogoStability {
    /// Construct a validated POGO stability analyzer.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the configuration is invalid.
    pub fn new(config: PogoStabilityConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self { config })
    }

    /// Borrow the validated configuration.
    #[must_use]
    pub const fn config(&self) -> PogoStabilityConfig {
        self.config
    }

    /// Analyze the cubic closed-loop characteristic.
    ///
    /// The reduced characteristic is
    /// `(tau*s + 1) * (s^2 + 2*zeta*omega*s + omega^2)
    /// - gain * (1 + zero*s) = 0`, where `gain` is attenuated by
    /// `C_cav / (C_cav + C_acc)`.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] if any intermediate coefficient is
    /// non-finite.
    pub fn analyze(&self) -> Result<PogoStabilitySnapshot, FeedSystemError> {
        let mode = self.config.mode;
        let feed = self.config.feed;
        let omega = mode.natural_frequency_rad_s;
        let damping = mode.damping_ratio;
        let tau = feed.feed_time_constant_s;
        let zero = feed.mass_flow_gain_time_s;
        let omega_sq = omega * omega;

        let attenuation = feed.accumulator_attenuation()?;
        let gain = feed.open_loop_gain_rad2 * attenuation;
        let a3 = tau;
        let a2 = 1.0 + 2.0 * damping * omega * tau;
        let a1 = 2.0 * damping * omega + tau * omega_sq - gain * zero;
        let a0 = omega_sq - gain;
        let routh_hurwitz_margin = a2 * a1 - a3 * a0;

        for value in [gain, a3, a2, a1, a0, routh_hurwitz_margin] {
            if !value.is_finite() {
                return Err(FeedSystemError::NonFinite {
                    reason: "POGO characteristic coefficient is non-finite",
                });
            }
        }

        let verdict = if a0 <= 0.0 {
            PogoStabilityVerdict::StaticDivergence
        } else if a1 <= 0.0 || routh_hurwitz_margin <= 0.0 {
            PogoStabilityVerdict::DynamicInstability
        } else {
            PogoStabilityVerdict::Stable
        };
        let critical_effective_gain_rad2 = critical_effective_gain_rad2(mode, feed);
        let neutral_accumulator_compliance_m3_per_pa =
            neutral_accumulator_compliance(feed, critical_effective_gain_rad2);

        Ok(PogoStabilitySnapshot {
            accumulator_attenuation: attenuation,
            effective_open_loop_gain_rad2: gain,
            characteristic_coefficients: [a3, a2, a1, a0],
            static_margin_rad2: a0,
            routh_hurwitz_margin,
            critical_effective_gain_rad2,
            neutral_accumulator_compliance_m3_per_pa,
            verdict,
            validation: ValidationStatus::ValidatedToy,
        })
    }
}

fn critical_effective_gain_rad2(mode: PogoModeConfig, feed: PogoFeedCouplingConfig) -> f64 {
    let omega = mode.natural_frequency_rad_s;
    let damping = mode.damping_ratio;
    let tau = feed.feed_time_constant_s;
    let zero = feed.mass_flow_gain_time_s;
    let omega_sq = omega * omega;
    let a2_without_gain = 1.0 + 2.0 * damping * omega * tau;
    let a1_without_gain = 2.0 * damping * omega + tau * omega_sq;

    let mut critical = omega_sq;
    if zero > 0.0 {
        critical = critical.min(a1_without_gain / zero);
    }

    let denominator = a2_without_gain * zero - tau;
    if denominator > 0.0 {
        let numerator = a2_without_gain * a1_without_gain - tau * omega_sq;
        critical = critical.min(numerator.max(0.0) / denominator);
    }
    critical
}

fn neutral_accumulator_compliance(
    feed: PogoFeedCouplingConfig,
    critical_effective_gain_rad2: f64,
) -> Option<f64> {
    if !critical_effective_gain_rad2.is_finite() || critical_effective_gain_rad2 <= 0.0 {
        return None;
    }
    if feed.open_loop_gain_rad2 <= critical_effective_gain_rad2 {
        return Some(0.0);
    }
    Some(
        feed.cavitation_compliance_m3_per_pa
            * (feed.open_loop_gain_rad2 / critical_effective_gain_rad2 - 1.0),
    )
}

fn require_positive_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    if !value.is_finite() {
        return Err(FeedSystemError::NonFinite { reason });
    }
    if value <= 0.0 {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

fn require_nonnegative_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    if !value.is_finite() {
        return Err(FeedSystemError::NonFinite { reason });
    }
    if value < 0.0 {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    fn config(
        open_loop_gain_rad2: f64,
        accumulator_compliance_m3_per_pa: f64,
    ) -> PogoStabilityConfig {
        PogoStabilityConfig {
            mode: PogoModeConfig {
                natural_frequency_rad_s: 60.0,
                damping_ratio: 0.04,
            },
            feed: PogoFeedCouplingConfig {
                open_loop_gain_rad2,
                feed_time_constant_s: 0.02,
                mass_flow_gain_time_s: 0.004,
                cavitation_compliance_m3_per_pa: 1.0e-9,
                accumulator_compliance_m3_per_pa,
            },
        }
    }

    #[test]
    fn pogo_stability_reports_stable_low_gain() {
        let snapshot = PogoStability::new(config(500.0, 0.0))
            .unwrap()
            .analyze()
            .unwrap();

        assert_eq!(snapshot.verdict, PogoStabilityVerdict::Stable);
        assert!(snapshot.static_margin_rad2 > 0.0);
        assert!(snapshot.routh_hurwitz_margin > 0.0);
        assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
    }

    #[test]
    fn pogo_stability_detects_static_divergence() {
        let snapshot = PogoStability::new(config(4_000.0, 0.0))
            .unwrap()
            .analyze()
            .unwrap();

        assert_eq!(snapshot.verdict, PogoStabilityVerdict::StaticDivergence);
        assert!(snapshot.static_margin_rad2 < 0.0);
    }

    #[test]
    fn pogo_stability_detects_dynamic_instability() {
        let mut unstable = config(500.0, 0.0);
        unstable.feed.mass_flow_gain_time_s = 0.50;
        let snapshot = PogoStability::new(unstable).unwrap().analyze().unwrap();

        assert_eq!(snapshot.verdict, PogoStabilityVerdict::DynamicInstability);
        assert!(snapshot.static_margin_rad2 > 0.0);
        assert!(snapshot.routh_hurwitz_margin <= 0.0);
    }

    #[test]
    fn pogo_accumulator_compliance_detunes_unstable_case() {
        let baseline = PogoStability::new(config(4_000.0, 0.0))
            .unwrap()
            .analyze()
            .unwrap();
        let neutral = baseline
            .neutral_accumulator_compliance_m3_per_pa
            .expect("unstable case has finite accumulator boundary");
        let detuned = PogoStability::new(config(4_000.0, neutral * 1.1))
            .unwrap()
            .analyze()
            .unwrap();

        assert!(neutral > 0.0);
        assert_eq!(detuned.verdict, PogoStabilityVerdict::Stable);
        assert!(detuned.effective_open_loop_gain_rad2 < baseline.critical_effective_gain_rad2);
    }

    #[test]
    fn pogo_accumulator_attenuation_uses_cavitation_reference_compliance() {
        let snapshot = PogoStability::new(config(1_000.0, 3.0e-9))
            .unwrap()
            .analyze()
            .unwrap();

        assert_abs_diff_eq!(snapshot.accumulator_attenuation, 0.25, epsilon = 1.0e-15);
        assert_abs_diff_eq!(
            snapshot.effective_open_loop_gain_rad2,
            250.0,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn pogo_stability_rejects_invalid_parameters() {
        let mut invalid = config(500.0, 0.0);
        invalid.mode.natural_frequency_rad_s = 0.0;
        assert!(matches!(
            PogoStability::new(invalid),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        invalid = config(500.0, 0.0);
        invalid.feed.cavitation_compliance_m3_per_pa = 0.0;
        assert!(matches!(
            PogoStability::new(invalid),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }
}
