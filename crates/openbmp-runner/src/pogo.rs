//! Runner-side adapter for reduced POGO stability checks.

use openbmp_feedsystem::{
    PogoFeedCouplingConfig, PogoModeConfig, PogoStability, PogoStabilityConfig,
    PogoStabilitySnapshot, PogoStabilityVerdict,
};
use openbmp_scenario::ScenarioDocument;
use openbmp_scenario::document::{BendingConfig, PropulsionPogoConfig, PropulsionPogoModeSource};

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
        Self::from_config(config, document.vehicle.bending.as_ref())
    }

    fn from_config(
        config: &PropulsionPogoConfig,
        bending: Option<&BendingConfig>,
    ) -> Result<Self, RunnerError> {
        let snapshot = PogoStability::new(PogoStabilityConfig {
            mode: pogo_mode_config(config, bending)?,
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

fn pogo_mode_config(
    config: &PropulsionPogoConfig,
    bending: Option<&BendingConfig>,
) -> Result<PogoModeConfig, RunnerError> {
    match config.mode_source {
        PropulsionPogoModeSource::Explicit => Ok(PogoModeConfig {
            natural_frequency_rad_s: config.mode_natural_frequency_rad_s.ok_or(
                openbmp_feedsystem::FeedSystemError::InvalidParameter {
                    reason: "propulsion POGO explicit mode is missing natural frequency",
                },
            )?,
            damping_ratio: config.mode_damping_ratio.ok_or(
                openbmp_feedsystem::FeedSystemError::InvalidParameter {
                    reason: "propulsion POGO explicit mode is missing damping ratio",
                },
            )?,
        }),
        PropulsionPogoModeSource::VehicleBending => {
            let bending = bending.ok_or(openbmp_feedsystem::FeedSystemError::InvalidParameter {
                reason: "propulsion POGO mode_source=vehicle_bending requires [vehicle.bending]",
            })?;
            Ok(PogoModeConfig {
                natural_frequency_rad_s: std::f64::consts::TAU * bending.frequency_hz,
                damping_ratio: bending.damping_ratio,
            })
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    const RIGID_BODY_ATMOSPHERE_SCENARIO: &str =
        include_str!("../tests/fixtures/rigid-body-geocentric-atmosphere-telemetry.toml");

    fn config(open_loop_gain_rad2: f64, require_stable: bool) -> PropulsionPogoConfig {
        PropulsionPogoConfig {
            mode_source: PropulsionPogoModeSource::Explicit,
            mode_natural_frequency_rad_s: Some(60.0),
            mode_damping_ratio: Some(0.04),
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
        let rack = PogoStabilityRack::from_config(&config(500.0, true), None).unwrap();

        assert!(!rack.is_empty());
        assert_eq!(
            rack.snapshot().unwrap().verdict,
            PogoStabilityVerdict::Stable
        );
    }

    #[test]
    fn pogo_rack_allows_unstable_when_not_required() {
        let rack = PogoStabilityRack::from_config(&config(4_000.0, false), None).unwrap();

        assert_eq!(
            rack.snapshot().unwrap().verdict,
            PogoStabilityVerdict::StaticDivergence
        );
    }

    #[test]
    fn pogo_rack_rejects_required_unstable_config() {
        assert!(PogoStabilityRack::from_config(&config(4_000.0, true), None).is_err());
    }

    fn vehicle_bending_pogo_block() -> &'static str {
        "\n[propulsion.pogo]\n\
         mode_source = \"vehicle_bending\"\n\
         open_loop_gain_rad2_s2 = 0.0\n\
         feed_time_constant_s = 0.02\n\
         mass_flow_gain_time_s = 0.004\n\
         cavitation_compliance_m3_per_pa = 0.000000001\n\
         accumulator_compliance_m3_per_pa = 0.0\n\
         require_stable = true\n"
    }

    #[test]
    fn pogo_rack_consumes_vehicle_bending_mode_source() {
        let toml = RIGID_BODY_ATMOSPHERE_SCENARIO.replace(
            "[vehicle.assembly]\n",
            "[vehicle.bending]\n\
             frequency_hz = 9.5\n\
             damping_ratio = 0.035\n\
             modal_mass_kg = 1200.0\n\
             slope_at_engine = 0.2\n\
             slope_at_gyro = 0.05\n\
             \n\
             [vehicle.assembly]\n",
        ) + vehicle_bending_pogo_block();
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("pogo scenario parses");
        let rack = PogoStabilityRack::build(&scenario.document).expect("pogo rack builds");
        let snapshot = rack.snapshot().expect("pogo snapshot");
        let expected_omega_sq = (std::f64::consts::TAU * 9.5_f64).powi(2);

        assert_eq!(snapshot.verdict, PogoStabilityVerdict::Stable);
        assert_eq!(
            snapshot.static_margin_rad2.to_bits(),
            expected_omega_sq.to_bits()
        );
    }

    #[test]
    fn pogo_rack_rejects_vehicle_bending_mode_source_without_bending_block() {
        let toml = format!(
            "{}{}",
            RIGID_BODY_ATMOSPHERE_SCENARIO,
            vehicle_bending_pogo_block()
        );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("pogo scenario parses");
        let err = PogoStabilityRack::build(&scenario.document).expect_err("missing bending");

        assert!(matches!(
            err,
            RunnerError::FeedSystem(openbmp_feedsystem::FeedSystemError::InvalidParameter {
                reason
            }) if reason.contains("requires [vehicle.bending]")
        ));
    }
}
