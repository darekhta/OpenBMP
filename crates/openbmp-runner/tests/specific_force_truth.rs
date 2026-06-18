//! WP-09.1 runner coverage for force-accumulator IMU truth.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::Path;

use openbmp_core::StepIndex;
use openbmp_physics::{WGS84_A_M, WGS84_MU_M3_S2, WGS84_OMEGA_RAD_S};
use openbmp_runner::sil::{FcObservation, SilMonitor};
use openbmp_scenario::Scenario;
use openbmp_sensors::SensorTruth;
use openbmp_telemetry::TelemetryValue;

const SCENARIO_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
);

const INLINE_MOTOR: &str = r#"
[propulsion.motor]
variant = "solid"
ignite_at_s = 0.0
mounted_to = "main"

[propulsion.motor.grain]
geometry = "end_burner"
cross_section_area_m2 = 0.02
length_m = 0.05
throat_radius_m = 0.005641895835477563
expansion_ratio = 20.0
dry_mass_kg = 0.2
name = "specific-force-truth-grain"
provenance = "synthetic WP-09.1 force-accumulator truth regression"

[propulsion.motor.grain.propellant]
label = "synthetic_textbook"
density_kg_m3 = 1700.0
burn_rate_a = 0.00004
burn_rate_n = 0.32
c_star_m_s = 1400.0
gamma = 1.2
web_steps = 64

[propulsion.nozzle]
ambient_pressure_correction = "pressure_thrust"
"#;

fn f64_column(outcome: &openbmp_runner::RunOutcome, name: &str) -> Vec<f64> {
    let channel = outcome
        .table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .unwrap_or_else(|| panic!("telemetry channel `{name}` must exist"))
        .id;

    outcome
        .table
        .rows()
        .iter()
        .map(|row| match row.get(channel) {
            Some(TelemetryValue::Float64(value)) => *value,
            other => panic!("unexpected `{name}` telemetry value: {other:?}"),
        })
        .collect()
}

fn load_force_accumulator_scenario() -> Scenario {
    let base = std::fs::read_to_string(SCENARIO_PATH).expect("read scenario");
    let toml = base
        .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
        .replace("stop_s  = 1.0", "stop_s  = 0.003")
        .replace(
            "[forces]\nmodels = [\"gravity\"]",
            "[forces]\nmodels = [\"gravity\", \"thrust\"]",
        )
        .replace(
            "[sensors.imu]\nkind = \"imu\"\nfile = \"../../data/sensors/imu-consumer-mems.toml\"",
            "[sensors.imu]\nkind = \"imu\"\nfile = \"../../data/sensors/imu-consumer-mems.toml\"\nspecific_force_source = \"force_accumulator\"",
        );
    let toml = format!("{toml}\n{INLINE_MOTOR}");
    let source_dir = Path::new(SCENARIO_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

fn load_stationary_rotating_frame_scenario() -> Scenario {
    let base = std::fs::read_to_string(SCENARIO_PATH).expect("read scenario");
    let inertial_velocity_y_m_s = WGS84_OMEGA_RAD_S * WGS84_A_M;
    let toml = base
        .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
        .replace("stop_s  = 1.0", "stop_s  = 0.001")
        .replace(
            "kind                     = \"point_mass\"",
            "kind                     = \"rigid_body\"",
        )
        .replace(
            "initial_position_eci_m   = [0.0, 0.0, 0.0]",
            &format!("initial_position_eci_m   = [{WGS84_A_M}, 0.0, 0.0]"),
        )
        .replace(
            "initial_velocity_eci_m_s = [0.0, 0.0, 0.0]",
            &format!(
                "initial_velocity_eci_m_s = [0.0, {inertial_velocity_y_m_s}, 0.0]\n\
                 initial_quaternion_body_to_eci_xyzw = [0.0, -0.7071067811865476, 0.0, 0.7071067811865476]\n\
                 initial_angular_velocity_body_rad_s = [{WGS84_OMEGA_RAD_S}, 0.0, 0.0]"
            ),
        )
        .replace(
            "dry_cg_body_m = [0.0, 0.0, 0.0]",
            "dry_cg_body_m = [0.0, 0.0, 0.0]\n\
             dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]",
        )
        .replace(
            "frame_profile = \"toy-fixed-earth\"\ngravity       = \"constant\"\ngravity_m_s2  = 9.80665",
            &format!(
                "frame_profile = \"wgs84-uniform-rotation\"\n\
                 gravity       = \"point_mass\"\n\
                 mu_m3_s2      = {WGS84_MU_M3_S2}"
            ),
        )
        .replace(
            "[sensors.imu]\nkind = \"imu\"\nfile = \"../../data/sensors/imu-consumer-mems.toml\"",
            "[sensors.imu]\nkind = \"imu\"\nfile = \"../../data/sensors/imu-consumer-mems.toml\"\nspecific_force_source = \"stationary_rotating_frame\"",
        );
    let source_dir = Path::new(SCENARIO_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

#[derive(Default)]
struct SpecificForceMonitor {
    ticks: usize,
    max_abs_z_m_s2: Option<f64>,
}

impl SilMonitor for SpecificForceMonitor {
    fn observe(&mut self, _step: StepIndex, truth: &SensorTruth, _observation: &FcObservation) {
        self.ticks += 1;
        let z = truth.specific_force_body_m_s2.z;
        if self
            .max_abs_z_m_s2
            .is_none_or(|current| z.abs() > current.abs())
        {
            self.max_abs_z_m_s2 = Some(z);
        }
    }
}

#[derive(Default)]
struct FirstTruthMonitor {
    ticks: usize,
    first_truth: Option<SensorTruth>,
}

impl SilMonitor for FirstTruthMonitor {
    fn observe(&mut self, _step: StepIndex, truth: &SensorTruth, _observation: &FcObservation) {
        self.ticks += 1;
        if self.first_truth.is_none() {
            self.first_truth = Some(*truth);
        }
    }
}

#[test]
fn force_accumulator_specific_force_truth_uses_thrust_over_mass() {
    let scenario = load_force_accumulator_scenario();
    let mut monitor = SpecificForceMonitor::default();
    let outcome = openbmp_runner::run_with_monitor(&scenario, &mut monitor)
        .expect("force-accumulator scenario runs");

    assert!(monitor.ticks > 0, "monitor should observe bridge truth");
    let observed = monitor
        .max_abs_z_m_s2
        .expect("specific-force truth should be observed");

    let thrust_z = f64_column(&outcome, "force.thrust.z_n");
    let mass_kg = f64_column(&outcome, "mass_kg");
    let expected = thrust_z
        .iter()
        .zip(mass_kg.iter())
        .take(monitor.ticks)
        .map(|(thrust, mass)| thrust / mass)
        .max_by(|a, b| a.abs().total_cmp(&b.abs()))
        .expect("thrust telemetry should have samples");

    assert!(
        (observed - expected).abs() <= 1.0e-9,
        "force-accumulator IMU truth must use non-gravity thrust/mass: observed {observed}, expected {expected}"
    );
    assert!(
        expected.abs() > 1.0,
        "fixture must produce a nontrivial powered specific force: {expected}"
    );
    assert!(
        (observed - (expected - 9.80665)).abs() > 1.0,
        "specific-force truth should exclude the gravity force component"
    );
}

#[test]
fn stationary_rotating_frame_truth_projects_wgs84_equator_to_body_ned() {
    let scenario = load_stationary_rotating_frame_scenario();
    let mut monitor = FirstTruthMonitor::default();
    openbmp_runner::run_with_monitor(&scenario, &mut monitor)
        .expect("stationary rotating-frame scenario runs");

    assert!(monitor.ticks > 0, "monitor should observe bridge truth");
    let truth = monitor
        .first_truth
        .expect("stationary rotating-frame truth should be observed");
    let expected_support_m_s2 =
        WGS84_MU_M3_S2 / (WGS84_A_M * WGS84_A_M) - WGS84_OMEGA_RAD_S.powi(2) * WGS84_A_M;

    assert!(
        truth.specific_force_body_m_s2.x.abs() <= 1.0e-9,
        "local-NED north specific force should be zero: {:?}",
        truth.specific_force_body_m_s2
    );
    assert!(
        truth.specific_force_body_m_s2.y.abs() <= 1.0e-9,
        "local-NED east specific force should be zero: {:?}",
        truth.specific_force_body_m_s2
    );
    assert!(
        (truth.specific_force_body_m_s2.z + expected_support_m_s2).abs() <= 1.0e-9,
        "local-NED down specific force should be -g + centrifugal: observed {:?}, expected z {}",
        truth.specific_force_body_m_s2,
        -expected_support_m_s2
    );
    assert!(
        (truth.angular_velocity_body_rad_s.x - WGS84_OMEGA_RAD_S).abs() <= 1.0e-12,
        "local-NED gyro north component should be Earth rate: {:?}",
        truth.angular_velocity_body_rad_s
    );
    assert!(
        truth.angular_velocity_body_rad_s.y.abs() <= 1.0e-12,
        "local-NED gyro east component should be zero: {:?}",
        truth.angular_velocity_body_rad_s
    );
    assert!(
        truth.angular_velocity_body_rad_s.z.abs() <= 1.0e-12,
        "local-NED gyro down component should be zero at equator: {:?}",
        truth.angular_velocity_body_rad_s
    );
}
