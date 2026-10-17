//! Runner-side control-structure-interaction report wiring.
//!
//! The actual margin math lives in `openbmp-fc::csi` to preserve the
//! flight-controller portability boundary. This module only maps the typed
//! scenario `[csi]` block into those helpers and stores deterministic
//! out-of-band evidence on [`crate::RunOutcome`].

use std::fmt::Write as _;

use openbmp_fc::csi::{
    CsiMarginCriteria, CsiMarginReport, CsiMarginVerdict, CsiTrajectoryReport, FrozenPlant,
    FrozenTrajectoryPoint, ModalCsiMode, analyze_frozen_trajectory,
    assemble_modal_csi_plant_coefficients, design_bending_notch,
};
use openbmp_fc::filters::NotchConfig;
use openbmp_scenario::ScenarioDocument;
use openbmp_scenario::document::{CsiConfig, CsiModalAssemblySensorConfig};
use openbmp_vehicle::structural::ModeParams;

use crate::error::RunnerError;

/// Runner-side CSI report for a parsed `[csi]` sweep.
#[derive(Clone, Debug, PartialEq)]
pub struct CsiRunReport {
    /// Frequencies used for sampled `L(j omega)` evaluation, rad/s.
    pub frequency_grid_rad_s: Vec<f64>,
    /// Required sampled gain margin, dB.
    pub min_gain_margin_db: f64,
    /// Required sampled phase margin, deg.
    pub min_phase_margin_deg: f64,
    /// Worst-case trajectory summary from `openbmp-fc::csi`.
    pub trajectory: CsiTrajectoryReport,
    /// Per-frozen-point margin reports in scenario declaration order.
    pub frozen_points: Vec<CsiFrozenPointReport>,
    /// Optional CSI-designed gyro notch to install in the FC autopilot.
    pub gyro_notch: Option<[NotchConfig; 3]>,
    /// Deterministic per-frozen-point margin table CSV bytes.
    pub margin_csv: Vec<u8>,
}

/// One runner-side CSI frozen-point report.
#[derive(Clone, Debug, PartialEq)]
pub struct CsiFrozenPointReport {
    /// Frozen-point id from the selected CSI plant source.
    pub id: String,
    /// Frozen trajectory time, s.
    pub time_s: f64,
    /// Sampled margin report from `openbmp-fc::csi`.
    pub margins: CsiMarginReport,
}

/// Evaluate the optional scenario `[csi]` block.
///
/// # Errors
///
/// Returns [`RunnerError`] when `[csi]` is declared without an active
/// structural modal rack or when the FC-portable CSI helper rejects a plant,
/// frequency grid, criteria, or output buffer.
pub fn evaluate_document(
    document: &ScenarioDocument,
    structural_rack: &crate::structural::StructuralRack,
) -> Result<Option<CsiRunReport>, RunnerError> {
    let Some(config) = document.csi.as_ref() else {
        return Ok(None);
    };
    if structural_rack.is_inactive() {
        return Err(RunnerError::UnsupportedScenario {
            what: "[csi] requires an active structural modal rack".to_owned(),
        });
    }

    let criteria = CsiMarginCriteria::new(
        config.criteria.min_gain_margin_db,
        config.criteria.min_phase_margin_deg,
    )
    .map_err(|err| csi_error("csi.criteria", err))?;
    let frozen_plants = build_frozen_plants(config, structural_rack)?;
    let trajectory = frozen_plants
        .iter()
        .map(|point| {
            let plant = FrozenPlant::new(
                &point.numerator_coefficients,
                &point.denominator_coefficients,
            )
            .map_err(|err| csi_error(&point.source_path, err))?;
            FrozenTrajectoryPoint::new(point.time_s, plant)
                .map_err(|err| csi_error(&point.source_path, err))
        })
        .collect::<Result<Vec<_>, _>>()?;

    let mut margin_reports = vec![empty_margin_report(); trajectory.len()];
    let trajectory_report = analyze_frozen_trajectory(
        &trajectory,
        &config.frequency_grid_rad_s,
        criteria,
        &mut margin_reports,
    )
    .map_err(|err| csi_error("csi", err))?;

    let frozen_points = frozen_plants
        .iter()
        .zip(margin_reports.iter())
        .map(|(point, margins)| CsiFrozenPointReport {
            id: point.id.clone(),
            time_s: point.time_s,
            margins: *margins,
        })
        .collect::<Vec<_>>();
    let margin_csv = csi_margin_csv(&frozen_points).into_bytes();
    let gyro_notch = config
        .filter_insertion
        .as_ref()
        .map(|filter| csi_gyro_notch_from_structural(filter, structural_rack))
        .transpose()?;

    Ok(Some(CsiRunReport {
        frequency_grid_rad_s: config.frequency_grid_rad_s.clone(),
        min_gain_margin_db: config.criteria.min_gain_margin_db,
        min_phase_margin_deg: config.criteria.min_phase_margin_deg,
        trajectory: trajectory_report,
        frozen_points,
        gyro_notch,
        margin_csv,
    }))
}

#[derive(Clone, Debug, PartialEq)]
struct FrozenPlantData {
    id: String,
    source_path: String,
    time_s: f64,
    numerator_coefficients: Vec<f64>,
    denominator_coefficients: Vec<f64>,
}

fn build_frozen_plants(
    config: &CsiConfig,
    structural_rack: &crate::structural::StructuralRack,
) -> Result<Vec<FrozenPlantData>, RunnerError> {
    if let Some(modal_assembly) = config.modal_assembly.as_ref() {
        let modes = structural_rack.mode_params();
        if modes.is_empty() {
            return Err(RunnerError::UnsupportedScenario {
                what: "[csi.modal_assembly] requires structural modes".to_owned(),
            });
        }
        let modal_modes = modes
            .iter()
            .map(|mode| modal_csi_mode_from_structural(*mode, modal_assembly.sensor))
            .collect::<Result<Vec<_>, _>>()?;
        let coefficient_count = (2 * modal_modes.len()) + 1;
        let mut plants = Vec::with_capacity(modal_assembly.frozen_points.len());
        for point in &modal_assembly.frozen_points {
            let mut numerator = vec![0.0; coefficient_count];
            let mut denominator = vec![0.0; coefficient_count];
            let mut scratch = vec![0.0; coefficient_count];
            let active_count = assemble_modal_csi_plant_coefficients(
                &modal_modes,
                modal_assembly.loop_gain * point.loop_gain_scale,
                &mut numerator,
                &mut denominator,
                &mut scratch,
            )
            .map_err(|err| {
                csi_error(
                    &format!("csi.modal_assembly.frozen_points.{}", point.id),
                    err,
                )
            })?;
            numerator.truncate(active_count);
            denominator.truncate(active_count);
            plants.push(FrozenPlantData {
                id: point.id.clone(),
                source_path: format!("csi.modal_assembly.frozen_points.{}", point.id),
                time_s: point.time_s,
                numerator_coefficients: numerator,
                denominator_coefficients: denominator,
            });
        }
        Ok(plants)
    } else {
        Ok(config
            .frozen_plants
            .iter()
            .map(|point| FrozenPlantData {
                id: point.id.clone(),
                source_path: format!("csi.frozen_plants.{}", point.id),
                time_s: point.time_s,
                numerator_coefficients: point.numerator_coefficients.clone(),
                denominator_coefficients: point.denominator_coefficients.clone(),
            })
            .collect())
    }
}

fn modal_csi_mode_from_structural(
    mode: ModeParams,
    sensor: CsiModalAssemblySensorConfig,
) -> Result<ModalCsiMode, RunnerError> {
    let (sensor_rate_coupling, sensor_acceleration_coupling) = match sensor {
        CsiModalAssemblySensorConfig::GyroRate => (mode.slope_at_gyro, 0.0),
        CsiModalAssemblySensorConfig::Accelerometer => (0.0, mode.shape_at_accelerometer),
    };
    ModalCsiMode::new(
        mode.omega_rad_s,
        mode.zeta,
        mode.modal_mass_kg,
        mode.slope_at_engine,
        0.0,
        sensor_rate_coupling,
        sensor_acceleration_coupling,
    )
    .map_err(|err| csi_error("csi.modal_assembly.mode", err))
}

fn csi_gyro_notch_from_structural(
    config: &openbmp_scenario::document::CsiFilterInsertionConfig,
    structural_rack: &crate::structural::StructuralRack,
) -> Result<[NotchConfig; 3], RunnerError> {
    let mode = structural_rack
        .mode_params()
        .get(config.mode_index)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!(
                "[csi.filter_insertion] mode_index {} is out of range for {} structural modes",
                config.mode_index,
                structural_rack.mode_params().len()
            ),
        })?;
    let notch = design_bending_notch(
        mode.omega_rad_s,
        config.zero_damping_ratio,
        config.pole_damping_ratio,
    )
    .map_err(|err| csi_error("csi.filter_insertion", err))?
    .notch_config();
    Ok([notch; 3])
}

fn csi_error(field: &str, err: openbmp_fc::csi::CsiError) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("{field} CSI evaluation failed: {err:?}"),
    }
}

fn empty_margin_report() -> CsiMarginReport {
    CsiMarginReport {
        gain_margin_db: None,
        phase_margin_deg: None,
        gain_crossover_frequency_rad_s: None,
        phase_crossover_frequency_rad_s: None,
        min_distance_to_minus_one: 0.0,
        min_distance_frequency_rad_s: 0.0,
        peak_gain_db: 0.0,
        peak_gain_frequency_rad_s: 0.0,
        verdict: CsiMarginVerdict::BelowCriteria,
    }
}

fn csi_margin_csv(points: &[CsiFrozenPointReport]) -> String {
    let mut out = String::from(
        "frozen_point_id,time_s,gain_margin_db,phase_margin_deg,\
         gain_crossover_frequency_rad_s,phase_crossover_frequency_rad_s,\
         min_distance_to_minus_one,min_distance_frequency_rad_s,peak_gain_db,\
         peak_gain_frequency_rad_s,verdict\n",
    );
    for point in points {
        let margins = point.margins;
        let _ = writeln!(
            out,
            "{},{},{},{},{},{},{},{},{},{},{}",
            point.id,
            point.time_s,
            optional_f64(margins.gain_margin_db),
            optional_f64(margins.phase_margin_deg),
            optional_f64(margins.gain_crossover_frequency_rad_s),
            optional_f64(margins.phase_crossover_frequency_rad_s),
            margins.min_distance_to_minus_one,
            margins.min_distance_frequency_rad_s,
            margins.peak_gain_db,
            margins.peak_gain_frequency_rad_s,
            verdict_str(margins.verdict)
        );
    }
    out
}

fn optional_f64(value: Option<f64>) -> String {
    value.map_or_else(String::new, |value| value.to_string())
}

const fn verdict_str(verdict: CsiMarginVerdict) -> &'static str {
    match verdict {
        CsiMarginVerdict::MeetsCriteria => "meets_criteria",
        CsiMarginVerdict::BelowCriteria => "below_criteria",
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    const BASE_SCENARIO: &str = include_str!("../tests/fixtures/structural-rack-base.toml");
    const FC_BLOCK: &str = r#"
[fc]
estimator = "ekf"
autopilot = "three_loop"
guidance = "attitude_hold"
reference_q_xyzw = [0.0, 0.0, 0.0, 1.0]
base_rate_hz = 1000
frame_budget_us = 2000

[fc.ekf]
mag_field = "earth_dipole"
sigma_w_gyro = 0.01
sigma_w_accel_bias = 1.0e-4
sigma_w_gyro_bias = 1.0e-5
sigma_gnss_pos_m = 5.0
sigma_gnss_vel_m_s = 0.5
sigma_baro_alt_m = 2.0
sigma_mag_nt = 100.0
innovation_false_alarm_rate = 0.01
dead_reckon_timeout_s = 1.5

[fc.autopilot_params]
rate_deadband_rad_s = 0.001
trajectory_loop_enabled = false
trajectory_kind = "pid"

[fc.health]
imu_stale_after_s = 0.05
gnss_stale_after_s = 0.5
baro_stale_after_s = 10.0
mag_stale_after_s = 0.2
overrun_burst_count = 5

[fc.gain_schedule."mission.phases.ascent"]
rate_kp = [0.5, 0.5, 0.5]
rate_kd = [0.05, 0.05, 0.05]
attitude_kp = [2.0, 2.0, 1.0]
elevator_limit_rad = 0.35
aileron_limit_rad = 0.35
rudder_limit_rad = 0.35
throttle_baseline = 0.0
"#;

    fn insert_before_assembly(insert: &str) -> String {
        BASE_SCENARIO.replace(
            "[vehicle.assembly]\n",
            &format!("{insert}\n[vehicle.assembly]\n"),
        )
    }

    #[test]
    fn csi_block_emits_run_outcome_margin_report() {
        let gain_0 = 2.5_f64;
        let gain_1 = 3.0_f64;
        let omega_gc_0 = (gain_0.powf(2.0 / 3.0) - 1.0).sqrt();
        let omega_gc_1 = (gain_1.powf(2.0 / 3.0) - 1.0).sqrt();
        let toml = insert_before_assembly(&format!(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1

[csi]
frequency_grid_rad_s = [0.25, {omega_gc_0}, {omega_gc_1}, 1.7320508075688772, 4.0]

[csi.criteria]
min_gain_margin_db = 6.0
min_phase_margin_deg = 30.0

[[csi.frozen_plants]]
id = "liftoff"
time_s = 0.0
numerator_coefficients = [{gain_0}]
denominator_coefficients = [1.0, 3.0, 3.0, 1.0]

[[csi.frozen_plants]]
id = "max_q"
time_s = 5.0
numerator_coefficients = [{gain_1}]
denominator_coefficients = [1.0, 3.0, 3.0, 1.0]
"#
        ));
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let outcome = crate::run(&scenario).unwrap();
        let report = outcome.csi.expect("CSI report is emitted");

        assert_eq!(report.frequency_grid_rad_s.len(), 5);
        assert_eq!(report.trajectory.point_count, 2);
        assert_eq!(report.trajectory.verdict, CsiMarginVerdict::MeetsCriteria);
        assert_eq!(report.frozen_points.len(), 2);
        assert_eq!(report.frozen_points[0].id, "liftoff");
        assert_eq!(report.frozen_points[1].id, "max_q");
        assert_eq!(
            report
                .trajectory
                .worst_gain_margin_time_s
                .unwrap()
                .to_bits(),
            5.0_f64.to_bits()
        );
        assert_eq!(
            report
                .trajectory
                .worst_phase_margin_time_s
                .unwrap()
                .to_bits(),
            5.0_f64.to_bits()
        );
        assert_eq!(
            report.frozen_points[0].margins.verdict,
            CsiMarginVerdict::MeetsCriteria
        );
        assert_eq!(
            report.frozen_points[1].margins.verdict,
            CsiMarginVerdict::MeetsCriteria
        );

        let csv = std::str::from_utf8(&report.margin_csv).unwrap();
        assert!(csv.starts_with("frozen_point_id,time_s,gain_margin_db,phase_margin_deg,"));
        assert!(csv.contains("liftoff,0,"));
        assert!(csv.contains("max_q,5,"));
        assert!(csv.contains("meets_criteria"));
    }

    #[test]
    fn csi_report_is_absent_when_block_is_omitted() {
        let scenario = openbmp_scenario::Scenario::from_toml_str(BASE_SCENARIO).unwrap();
        let outcome = crate::run(&scenario).unwrap();
        assert!(outcome.csi.is_none());
    }

    #[test]
    fn csi_modal_assembly_uses_structural_modes() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.04
modal_mass_kg = 10.0
slope_at_engine = 2.0
slope_at_gyro = 0.5
shape_at_accelerometer = 0.25

[csi]
frequency_grid_rad_s = [1.0, 6.283185307179586, 20.0]

[csi.criteria]
min_gain_margin_db = 0.0
min_phase_margin_deg = 0.0

[csi.modal_assembly]
sensor = "gyro_rate"
loop_gain = 10.0

[[csi.modal_assembly.frozen_points]]
id = "low_gain"
time_s = 0.0

[[csi.modal_assembly.frozen_points]]
id = "high_gain"
time_s = 5.0
loop_gain_scale = 2.0
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let outcome = crate::run(&scenario).unwrap();
        let report = outcome.csi.expect("CSI report is emitted");

        assert_eq!(report.trajectory.point_count, 2);
        assert_eq!(report.frozen_points.len(), 2);
        assert_eq!(report.frozen_points[0].id, "low_gain");
        assert_eq!(report.frozen_points[1].id, "high_gain");
        assert!(
            report.frozen_points[1].margins.peak_gain_db
                > report.frozen_points[0].margins.peak_gain_db
        );

        let csv = std::str::from_utf8(&report.margin_csv).unwrap();
        assert!(csv.contains("low_gain,0,"));
        assert!(csv.contains("high_gain,5,"));
    }

    #[test]
    fn csi_filter_insertion_designs_fc_gyro_notch() {
        let toml = insert_before_assembly(&format!(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.04
modal_mass_kg = 10.0
slope_at_engine = 2.0
slope_at_gyro = 0.5
shape_at_accelerometer = 0.25

{FC_BLOCK}

[csi]
frequency_grid_rad_s = [1.0, 6.283185307179586, 20.0]

[csi.criteria]
min_gain_margin_db = 0.0
min_phase_margin_deg = 0.0

[csi.filter_insertion]
mode_index = 0
zero_damping_ratio = 0.02
pole_damping_ratio = 0.20

[[csi.frozen_plants]]
id = "liftoff"
time_s = 0.0
numerator_coefficients = [1.0]
denominator_coefficients = [1.0, 1.0]
"#
        ));
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let structural_rack = crate::structural::StructuralRack::build(&scenario.document).unwrap();
        let report = evaluate_document(&scenario.document, &structural_rack)
            .unwrap()
            .expect("CSI report is emitted");
        let notch = report.gyro_notch.expect("CSI notch is designed");

        for axis_notch in notch {
            assert!((axis_notch.center_hz - 1.0).abs() < 1.0e-14);
            assert!((axis_notch.bandwidth_hz - 0.4).abs() < 1.0e-14);
            assert!((axis_notch.depth_db - 20.0).abs() < 1.0e-12);
        }
    }
}
