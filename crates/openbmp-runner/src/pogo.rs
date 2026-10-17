//! Runner-side adapter for reduced POGO stability checks.

use std::collections::BTreeMap;

use num_complex::Complex64;
use openbmp_feedsystem::{
    PogoFeedCouplingConfig, PogoModeConfig, PogoStability, PogoStabilityConfig,
    PogoStabilitySnapshot, PogoStabilityVerdict,
};
use openbmp_scenario::ScenarioDocument;
use openbmp_scenario::document::{
    BendingConfig, PropulsionLineConfig, PropulsionPogoConfig, PropulsionPogoDispersionBoxConfig,
    PropulsionPogoModeSource,
};
use openbmp_structdyn::pogo::{
    FeedTransferSample, LongitudinalPogoMode, PogoClosedLoopEigenReport,
    PogoFeedlineCoupledEigenReport, PogoFeedlineSegment, PogoFirstOrderFeed, PogoLoopMargin,
    PogoLumpedFeedlineModel, PogoNyquistReport, PogoNyquistVerdict, PogoPumpDispersionBox,
    PogoPumpDispersionPoint, PogoPumpDispersionReport, analyze_first_order_closed_loop,
    analyze_lumped_feedline_coupled_eigenproblem, analyze_pump_dispersion_box,
};

use crate::error::RunnerError;

/// Runner-side POGO report for `[propulsion.pogo]` scenarios.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoRunReport {
    /// Reduced feed-system stability snapshot from `openbmp-feedsystem`.
    pub feed_snapshot: PogoStabilitySnapshot,
    /// Closed-loop eigenvalue/RHP report from `openbmp-structdyn`.
    pub closed_loop: PogoClosedLoopEigenReport,
    /// Optional pump Cb/Mb dispersion-box report from `openbmp-structdyn`.
    pub pump_dispersion: Option<PogoPumpDispersionReport>,
    /// Provenance pointer for the optional pump Cb/Mb dispersion-box input.
    pub pump_dispersion_provenance: Option<String>,
    /// Optional lumped feedline-coupled eigen report built from referenced
    /// standalone `[[propulsion.lines]]` declarations.
    pub feedline_coupled: Option<PogoFeedlineCoupledEigenReport>,
    /// Structural sampled Nyquist report from `openbmp-structdyn`.
    pub structural_nyquist: PogoNyquistReport,
    /// Deterministic CSV artifact for the structural sampled open-loop points.
    pub structural_margin_csv: Vec<u8>,
}

/// Runner-side optional POGO stability evaluation.
#[derive(Clone, Debug, Default)]
pub struct PogoStabilityRack {
    report: Option<PogoRunReport>,
}

impl PogoStabilityRack {
    /// Build the optional POGO rack from `[propulsion.pogo]`.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when feed-system validation fails or when
    /// `require_stable = true` and the reduced stability verdict is not stable.
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let Some(propulsion) = document.propulsion.as_ref() else {
            return Ok(Self::default());
        };
        let Some(config) = propulsion.pogo.as_ref() else {
            return Ok(Self::default());
        };
        Self::from_config(config, document.vehicle.bending.as_ref(), &propulsion.lines)
    }

    fn from_config(
        config: &PropulsionPogoConfig,
        bending: Option<&BendingConfig>,
        lines: &[PropulsionLineConfig],
    ) -> Result<Self, RunnerError> {
        let mode = pogo_mode_config(config, bending)?;
        let feed = PogoFeedCouplingConfig {
            open_loop_gain_rad2: config.open_loop_gain_rad2_s2,
            feed_time_constant_s: config.feed_time_constant_s,
            mass_flow_gain_time_s: config.mass_flow_gain_time_s,
            cavitation_compliance_m3_per_pa: config.cavitation_compliance_m3_per_pa,
            accumulator_compliance_m3_per_pa: config.accumulator_compliance_m3_per_pa,
        };
        let snapshot = PogoStability::new(PogoStabilityConfig { mode, feed })?.analyze()?;
        if config.require_stable && snapshot.verdict != PogoStabilityVerdict::Stable {
            return Err(openbmp_feedsystem::FeedSystemError::InvalidParameter {
                reason: "propulsion POGO stability gate rejected an unstable verdict",
            }
            .into());
        }
        let closed_loop = closed_loop_eigen_report(mode, feed, snapshot)?;
        let pump_dispersion = config
            .dispersion_box
            .as_ref()
            .map(|dispersion_box| pump_dispersion_report(mode, feed, dispersion_box))
            .transpose()?;
        let pump_dispersion_provenance = config
            .dispersion_box
            .as_ref()
            .map(|dispersion_box| dispersion_box.provenance.clone());
        let feedline_coupled = feedline_coupled_eigen_report(config, mode, feed, lines)?;
        let structural_nyquist =
            structural_loop_margin(mode, feed, snapshot, config.structural_nyquist_clearance_1)?
                .analyze()
                .map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!("propulsion POGO structural Nyquist analysis failed: {err}"),
                })?;
        if config.require_structural_nyquist_clearance
            && structural_nyquist.verdict != PogoNyquistVerdict::Clear
        {
            return Err(RunnerError::UnsupportedScenario {
                what: "propulsion POGO structural Nyquist clearance gate rejected a below-clearance verdict".to_owned(),
            });
        }
        let structural_margin_csv = pogo_structural_margin_csv(&structural_nyquist);
        Ok(Self {
            report: Some(PogoRunReport {
                feed_snapshot: snapshot,
                closed_loop,
                pump_dispersion,
                pump_dispersion_provenance,
                feedline_coupled,
                structural_nyquist,
                structural_margin_csv,
            }),
        })
    }

    /// `true` when no `[propulsion.pogo]` block was configured.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.report.is_none()
    }

    /// Latest startup stability result.
    #[must_use]
    pub const fn snapshot(&self) -> Option<PogoStabilitySnapshot> {
        match &self.report {
            Some(report) => Some(report.feed_snapshot),
            None => None,
        }
    }

    /// Complete runner POGO report.
    #[must_use]
    pub fn report(&self) -> Option<&PogoRunReport> {
        self.report.as_ref()
    }

    /// Consume the rack and return the report for [`crate::RunOutcome`].
    #[must_use]
    pub fn into_report(self) -> Option<PogoRunReport> {
        self.report
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

fn structural_loop_margin(
    mode: PogoModeConfig,
    feed: PogoFeedCouplingConfig,
    snapshot: PogoStabilitySnapshot,
    structural_nyquist_clearance_1: f64,
) -> Result<PogoLoopMargin, RunnerError> {
    let structural_mode = normalized_structural_mode(mode)?;
    let feed_samples = structural_frequency_grid(mode, feed)
        .into_iter()
        .map(|omega| {
            FeedTransferSample::new(
                omega,
                reduced_feed_transfer_at(feed, snapshot.effective_open_loop_gain_rad2, omega),
            )
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("propulsion POGO feed transfer sample is invalid: {err}"),
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    PogoLoopMargin::new(
        vec![structural_mode],
        feed_samples,
        structural_nyquist_clearance_1,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("propulsion POGO structural loop margin is invalid: {err}"),
    })
}

fn closed_loop_eigen_report(
    mode: PogoModeConfig,
    feed: PogoFeedCouplingConfig,
    snapshot: PogoStabilitySnapshot,
) -> Result<PogoClosedLoopEigenReport, RunnerError> {
    analyze_first_order_closed_loop(
        normalized_structural_mode(mode)?,
        PogoFirstOrderFeed::new(
            snapshot.effective_open_loop_gain_rad2,
            feed.feed_time_constant_s,
            feed.mass_flow_gain_time_s,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("propulsion POGO first-order feed model is invalid: {err}"),
        })?,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("propulsion POGO closed-loop eigen analysis failed: {err}"),
    })
}

fn pump_dispersion_report(
    mode: PogoModeConfig,
    feed: PogoFeedCouplingConfig,
    dispersion_box: &PropulsionPogoDispersionBoxConfig,
) -> Result<PogoPumpDispersionReport, RunnerError> {
    analyze_pump_dispersion_box(
        normalized_structural_mode(mode)?,
        feed.open_loop_gain_rad2,
        feed.feed_time_constant_s,
        feed.accumulator_compliance_m3_per_pa,
        PogoPumpDispersionBox::new(
            PogoPumpDispersionPoint::new(
                dispersion_box.cavitation_compliance_min_m3_per_pa,
                dispersion_box.mass_flow_gain_time_min_s,
            )
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("propulsion POGO pump-dispersion minimum is invalid: {err}"),
            })?,
            PogoPumpDispersionPoint::new(
                feed.cavitation_compliance_m3_per_pa,
                feed.mass_flow_gain_time_s,
            )
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("propulsion POGO pump-dispersion nominal is invalid: {err}"),
            })?,
            PogoPumpDispersionPoint::new(
                dispersion_box.cavitation_compliance_max_m3_per_pa,
                dispersion_box.mass_flow_gain_time_max_s,
            )
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("propulsion POGO pump-dispersion maximum is invalid: {err}"),
            })?,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("propulsion POGO pump-dispersion box is invalid: {err}"),
        })?,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("propulsion POGO pump-dispersion analysis failed: {err}"),
    })
}

fn feedline_coupled_eigen_report(
    config: &PropulsionPogoConfig,
    mode: PogoModeConfig,
    feed: PogoFeedCouplingConfig,
    lines: &[PropulsionLineConfig],
) -> Result<Option<PogoFeedlineCoupledEigenReport>, RunnerError> {
    let Some(feedline) = config.feedline.as_ref() else {
        return Ok(None);
    };
    let line_by_id = lines
        .iter()
        .map(|line| (line.id.as_str(), line))
        .collect::<BTreeMap<_, _>>();
    let segments = feedline
        .line_ids
        .iter()
        .map(|line_id| {
            let line = line_by_id.get(line_id.as_str()).ok_or_else(|| {
                RunnerError::UnsupportedScenario {
                    what: format!("propulsion POGO feedline references unknown line '{line_id}'"),
                }
            })?;
            PogoFeedlineSegment::new(
                line.length_m,
                line.wave_speed_m_s,
                line.density_kg_m3,
                line.cross_section_area_m2,
            )
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("propulsion POGO feedline segment '{line_id}' is invalid: {err}"),
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    let pump = PogoPumpDispersionPoint::new(
        feed.cavitation_compliance_m3_per_pa,
        feed.mass_flow_gain_time_s,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("propulsion POGO feedline pump point is invalid: {err}"),
    })?;
    let model = PogoLumpedFeedlineModel::new(
        segments,
        pump,
        feed.accumulator_compliance_m3_per_pa,
        feed.open_loop_gain_rad2,
        feed.feed_time_constant_s,
        feedline.feedline_damping_ratio_1,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("propulsion POGO lumped feedline model is invalid: {err}"),
    })?;
    analyze_lumped_feedline_coupled_eigenproblem(normalized_structural_mode(mode)?, &model)
        .map(Some)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("propulsion POGO lumped feedline eigen analysis failed: {err}"),
        })
}

fn normalized_structural_mode(mode: PogoModeConfig) -> Result<LongitudinalPogoMode, RunnerError> {
    LongitudinalPogoMode::new(mode.natural_frequency_rad_s, mode.damping_ratio, 1.0, 1.0).map_err(
        |err| RunnerError::UnsupportedScenario {
            what: format!("propulsion POGO structural mode is invalid: {err}"),
        },
    )
}

fn structural_frequency_grid(mode: PogoModeConfig, feed: PogoFeedCouplingConfig) -> Vec<f64> {
    let omega = mode.natural_frequency_rad_s;
    let feed_pole = 1.0 / feed.feed_time_constant_s;
    let mut grid = [
        0.0,
        0.25 * omega,
        0.5 * omega,
        omega,
        2.0 * omega,
        feed_pole,
        2.0 * feed_pole,
    ]
    .into_iter()
    .filter(|value| value.is_finite() && *value >= 0.0)
    .collect::<Vec<_>>();
    grid.sort_by(f64::total_cmp);
    grid.dedup_by(|a, b| (*a - *b).abs() <= f64::EPSILON * a.abs().max(b.abs()).max(1.0));
    grid
}

fn reduced_feed_transfer_at(
    feed: PogoFeedCouplingConfig,
    effective_gain_rad2: f64,
    omega_rad_s: f64,
) -> Complex64 {
    let s = Complex64::new(0.0, omega_rad_s);
    -effective_gain_rad2 * (Complex64::new(1.0, 0.0) + s * feed.mass_flow_gain_time_s)
        / (Complex64::new(1.0, 0.0) + s * feed.feed_time_constant_s)
}

fn pogo_structural_margin_csv(report: &PogoNyquistReport) -> Vec<u8> {
    let mut csv = String::from(
        "omega_rad_s,structural_receptance_re_m_per_n,structural_receptance_im_m_per_n,feed_transfer_re_n_per_m,feed_transfer_im_n_per_m,open_loop_re,open_loop_im,distance_to_minus_one,gain,phase_deg\n",
    );
    for sample in report.samples() {
        csv.push_str(&format!(
            "{},{},{},{},{},{},{},{},{},{}\n",
            sample.omega_rad_s,
            sample.structural_receptance_m_per_n.re,
            sample.structural_receptance_m_per_n.im,
            sample.feed_transfer_n_per_m.re,
            sample.feed_transfer_n_per_m.im,
            sample.open_loop.re,
            sample.open_loop.im,
            sample.distance_to_minus_one,
            sample.gain,
            sample.phase_deg
        ));
    }
    csv.into_bytes()
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
            dispersion_box: None,
            feedline: None,
            structural_nyquist_clearance_1: 0.1,
            require_structural_nyquist_clearance: false,
            require_stable,
        }
    }

    #[test]
    fn pogo_rack_accepts_stable_required_config() {
        let rack = PogoStabilityRack::from_config(&config(500.0, true), None, &[]).unwrap();

        assert!(!rack.is_empty());
        assert_eq!(
            rack.snapshot().unwrap().verdict,
            PogoStabilityVerdict::Stable
        );
    }

    #[test]
    fn pogo_rack_emits_structural_nyquist_report() {
        let rack = PogoStabilityRack::from_config(&config(500.0, true), None, &[]).unwrap();
        let report = rack.report().unwrap();
        let samples = report.structural_nyquist.samples();

        assert_eq!(samples.len(), 7);
        assert_eq!(samples[0].omega_rad_s.to_bits(), 0.0_f64.to_bits());
        assert_eq!(
            samples[0].feed_transfer_n_per_m.re.to_bits(),
            (-report.feed_snapshot.effective_open_loop_gain_rad2).to_bits()
        );
        assert_eq!(
            samples[0].feed_transfer_n_per_m.im.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            report.closed_loop.verdict,
            openbmp_structdyn::pogo::PogoClosedLoopVerdict::AsymptoticallyStable
        );
        assert!(report.closed_loop.max_real_part < 0.0);
        assert!(
            report
                .structural_nyquist
                .min_distance_to_minus_one
                .is_finite()
        );
        assert!(report.structural_nyquist.peak_gain.is_finite());
        let csv = std::str::from_utf8(&report.structural_margin_csv).unwrap();
        assert!(csv.starts_with(
            "omega_rad_s,structural_receptance_re_m_per_n,structural_receptance_im_m_per_n"
        ));
        assert_eq!(csv.lines().count(), samples.len() + 1);
    }

    #[test]
    fn pogo_rack_allows_unstable_when_not_required() {
        let rack = PogoStabilityRack::from_config(&config(4_000.0, false), None, &[]).unwrap();

        assert_eq!(
            rack.snapshot().unwrap().verdict,
            PogoStabilityVerdict::StaticDivergence
        );
    }

    #[test]
    fn pogo_rack_rejects_required_unstable_config() {
        assert!(PogoStabilityRack::from_config(&config(4_000.0, true), None, &[]).is_err());
    }

    #[test]
    fn pogo_rack_uses_declared_structural_nyquist_clearance() {
        let mut config = config(500.0, true);
        config.structural_nyquist_clearance_1 = 2.0;

        let rack = PogoStabilityRack::from_config(&config, None, &[]).unwrap();

        assert_eq!(
            rack.report().unwrap().structural_nyquist.verdict,
            PogoNyquistVerdict::BelowClearance
        );
    }

    #[test]
    fn pogo_rack_rejects_required_structural_nyquist_below_clearance() {
        let mut config = config(500.0, true);
        config.structural_nyquist_clearance_1 = 2.0;
        config.require_structural_nyquist_clearance = true;

        let err = PogoStabilityRack::from_config(&config, None, &[])
            .expect_err("structural gate rejects");

        assert!(matches!(
            err,
            RunnerError::UnsupportedScenario { what }
                if what.contains("structural Nyquist clearance gate")
        ));
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

    fn standalone_pogo_feedline_block(line_id: &str) -> String {
        format!(
            "\n[[propulsion.lines]]\n\
             id = \"{line_id}\"\n\
             length_m = 100.0\n\
             wave_speed_m_s = 1000.0\n\
             density_kg_m3 = 1000.0\n\
             cross_section_area_m2 = 0.01\n\
             segment_count = 1\n\
             initial_head_m = 100.0\n\
             initial_velocity_m_s = 2.0\n\
             upstream = {{ kind = \"head\", head_m = 100.0 }}\n\
             downstream = {{ kind = \"velocity\", velocity_m_s = 0.0 }}\n"
        )
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

    #[test]
    fn run_outcome_carries_pogo_report_when_configured() {
        let toml = format!(
            "{}\n[propulsion.pogo]\n\
             mode_natural_frequency_rad_s = 60.0\n\
             mode_damping_ratio = 0.04\n\
             open_loop_gain_rad2_s2 = 500.0\n\
             feed_time_constant_s = 0.02\n\
             mass_flow_gain_time_s = 0.004\n\
             cavitation_compliance_m3_per_pa = 0.000000001\n\
             accumulator_compliance_m3_per_pa = 0.0\n\
             require_stable = true\n",
            RIGID_BODY_ATMOSPHERE_SCENARIO
        );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("pogo scenario parses");
        let outcome = crate::run(&scenario).expect("pogo scenario runs");
        let report = outcome.pogo.expect("POGO report emitted");

        assert_eq!(report.feed_snapshot.verdict, PogoStabilityVerdict::Stable);
        assert_eq!(
            report.closed_loop.verdict,
            openbmp_structdyn::pogo::PogoClosedLoopVerdict::AsymptoticallyStable
        );
        assert_eq!(report.structural_nyquist.samples().len(), 7);
        assert!(!report.structural_margin_csv.is_empty());
    }

    #[test]
    fn run_outcome_carries_pogo_pump_dispersion_report_when_configured() {
        let toml = format!(
            "{}\n[propulsion.pogo]\n\
             mode_natural_frequency_rad_s = 60.0\n\
             mode_damping_ratio = 0.04\n\
             open_loop_gain_rad2_s2 = 500.0\n\
             feed_time_constant_s = 0.02\n\
             mass_flow_gain_time_s = 0.004\n\
             cavitation_compliance_m3_per_pa = 0.000000001\n\
             accumulator_compliance_m3_per_pa = 0.0\n\
             require_stable = true\n\
             \n\
             [propulsion.pogo.dispersion_box]\n\
             provenance = \"data/feed_system/provenance.md#openbmp.feed_system.generic_pogo_stability.v1\"\n\
             cavitation_compliance_min_m3_per_pa = 0.0000000005\n\
             cavitation_compliance_max_m3_per_pa = 0.000000004\n\
             mass_flow_gain_time_min_s = 0.002\n\
             mass_flow_gain_time_max_s = 0.006\n",
            RIGID_BODY_ATMOSPHERE_SCENARIO
        );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("pogo scenario parses");
        let outcome = crate::run(&scenario).expect("pogo scenario runs");
        let report = outcome.pogo.expect("POGO report emitted");
        let dispersion = report
            .pump_dispersion
            .expect("pump-dispersion report emitted");

        assert_eq!(
            report.pump_dispersion_provenance.as_deref(),
            Some("data/feed_system/provenance.md#openbmp.feed_system.generic_pogo_stability.v1")
        );
        assert_eq!(dispersion.samples().len(), 3);
        assert_eq!(
            dispersion.samples()[0]
                .point
                .cavitation_compliance_m3_per_pa
                .to_bits(),
            0.5e-9_f64.to_bits()
        );
        assert_eq!(
            dispersion.samples()[1]
                .point
                .mass_flow_gain_time_s
                .to_bits(),
            0.004_f64.to_bits()
        );
        assert_eq!(
            dispersion.worst_verdict,
            openbmp_structdyn::pogo::PogoClosedLoopVerdict::AsymptoticallyStable
        );
        assert!(dispersion.worst_max_real_part < 0.0);
    }

    #[test]
    fn run_outcome_carries_pogo_feedline_coupled_report_when_configured() {
        let toml = format!(
            "{}{}\n[propulsion.pogo]\n\
             mode_natural_frequency_rad_s = 60.0\n\
             mode_damping_ratio = 0.04\n\
             open_loop_gain_rad2_s2 = 0.0\n\
             feed_time_constant_s = 0.02\n\
             mass_flow_gain_time_s = 0.004\n\
             cavitation_compliance_m3_per_pa = 0.000000001\n\
             accumulator_compliance_m3_per_pa = 0.0\n\
             require_stable = true\n\
             \n\
             [propulsion.pogo.feedline]\n\
             line_ids = [\"oxidizer-main\"]\n\
             feedline_damping_ratio_1 = 0.02\n",
            RIGID_BODY_ATMOSPHERE_SCENARIO,
            standalone_pogo_feedline_block("oxidizer-main")
        );
        let scenario =
            openbmp_scenario::Scenario::from_toml_str(&toml).expect("pogo scenario parses");
        let outcome = crate::run(&scenario).expect("pogo scenario runs");
        let report = outcome.pogo.expect("POGO report emitted");
        let feedline = report
            .feedline_coupled
            .expect("feedline-coupled report emitted");

        assert_eq!(
            feedline.verdict,
            openbmp_structdyn::pogo::PogoClosedLoopVerdict::AsymptoticallyStable
        );
        assert_eq!(feedline.characteristic_coefficients.len(), 6);
        assert_eq!(feedline.eigenvalues().len(), 5);
        assert!(feedline.feedline_inertance_pa_s2_per_m3.is_finite());
        assert!(feedline.feedline_inertance_pa_s2_per_m3 > 0.0);
        assert!(feedline.feedline_compliance_m3_per_pa.is_finite());
        assert!(feedline.feedline_compliance_m3_per_pa > 0.0);
    }
}
