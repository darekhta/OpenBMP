//! Runner-side adapter for reduced POGO stability checks.

use openbmp_feedsystem::{
    PogoFeedCouplingConfig, PogoModeConfig, PogoStability, PogoStabilityConfig,
    PogoStabilitySnapshot, PogoStabilityVerdict,
};
use openbmp_scenario::{PropulsionPogoConfig, ScenarioDocument};

use crate::error::RunnerError;

/// Runner-side optional POGO stability evaluation.
#[derive(Clone, Debug, Default)]
pub struct PogoStabilityRack {
    snapshot: Option<PogoStabilitySnapshot>,
}

impl PogoStabilityRack {
    /// Build the optional POGO rack from `[propulsion.pogo]`.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when feed-system validation fails or when
    /// `require_stable = true` and the reduced stability verdict is not stable.
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let Some(config) = document
            .propulsion
            .as_ref()
            .and_then(|propulsion| propulsion.pogo.as_ref())
        else {
            return Ok(Self::default());
        };
        Self::from_config(config)
    }

    fn from_config(config: &PropulsionPogoConfig) -> Result<Self, RunnerError> {
        let snapshot = PogoStability::new(PogoStabilityConfig {
            mode: PogoModeConfig {
                natural_frequency_rad_s: config.mode_natural_frequency_rad_s,
                damping_ratio: config.mode_damping_ratio,
            },
            feed: PogoFeedCouplingConfig {
                open_loop_gain_rad2: config.open_loop_gain_rad2_s2,
                feed_time_constant_s: config.feed_time_constant_s,
                mass_flow_gain_time_s: config.mass_flow_gain_time_s,
                cavitation_compliance_m3_per_pa: config.cavitation_compliance_m3_per_pa,
                accumulator_compliance_m3_per_pa: config.accumulator_compliance_m3_per_pa,
            },
        })?
        .analyze()?;
        if config.require_stable && snapshot.verdict != PogoStabilityVerdict::Stable {
            return Err(openbmp_feedsystem::FeedSystemError::InvalidParameter {
                reason: "propulsion POGO stability gate rejected an unstable verdict",
            }
            .into());
        }
        Ok(Self {
            snapshot: Some(snapshot),
        })
    }

    /// `true` when no `[propulsion.pogo]` block was configured.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.snapshot.is_none()
    }

    /// Latest startup stability result.
    #[must_use]
    pub const fn snapshot(&self) -> Option<PogoStabilitySnapshot> {
        self.snapshot
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    fn config(open_loop_gain_rad2: f64, require_stable: bool) -> PropulsionPogoConfig {
        PropulsionPogoConfig {
            mode_natural_frequency_rad_s: 60.0,
            mode_damping_ratio: 0.04,
            open_loop_gain_rad2_s2: open_loop_gain_rad2,
            feed_time_constant_s: 0.02,
            mass_flow_gain_time_s: 0.004,
            cavitation_compliance_m3_per_pa: 1.0e-9,
            accumulator_compliance_m3_per_pa: 0.0,
            require_stable,
        }
    }

    #[test]
    fn pogo_rack_accepts_stable_required_config() {
        let rack = PogoStabilityRack::from_config(&config(500.0, true)).unwrap();

        assert!(!rack.is_empty());
        assert_eq!(
            rack.snapshot().unwrap().verdict,
            PogoStabilityVerdict::Stable
        );
    }

    #[test]
    fn pogo_rack_allows_unstable_when_not_required() {
        let rack = PogoStabilityRack::from_config(&config(4_000.0, false)).unwrap();

        assert_eq!(
            rack.snapshot().unwrap().verdict,
            PogoStabilityVerdict::StaticDivergence
        );
    }

    #[test]
    fn pogo_rack_rejects_required_unstable_config() {
        assert!(PogoStabilityRack::from_config(&config(4_000.0, true)).is_err());
    }
}
