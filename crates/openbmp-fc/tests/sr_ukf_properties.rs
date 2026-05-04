//! SR-UKF property checks that compare the new filter family against
//! the EKF reference surface.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::missing_panics_doc
)]

use std::path::PathBuf;

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_core::SimTime;
use openbmp_fc::estimator::{Ekf, EkfParams, Estimator};
use openbmp_fc::sr_ukf::{SquareRootUkf, SquareRootUkfParams};
use openbmp_fc::topics::{BarometerSample, GnssSample, ImuSample};
use openbmp_physics::atmosphere::USSA76_SEA_LEVEL_PRESSURE_PA;
use openbmp_physics::gravity::STANDARD_GRAVITY_M_S2;
use openbmp_testkit::tolerance::ToleranceTable;
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};

fn tolerance_table_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/expected/ukf-vs-ekf-tolerance.toml")
}

fn deterministic_pos_noise(k: u64) -> Vector3<f64> {
    let t = k as f64;
    Vector3::new(
        0.35 * (0.17 * t).sin(),
        -0.25 * (0.11 * t).cos(),
        0.20 * (0.07 * t).sin(),
    )
}

fn deterministic_vel_noise(k: u64) -> Vector3<f64> {
    let t = k as f64;
    Vector3::new(
        0.030 * (0.13 * t).cos(),
        -0.020 * (0.19 * t).sin(),
        0.015 * (0.05 * t).cos(),
    )
}

fn quaternion_angle_delta_rad(a_xyzw: [f64; 4], b_xyzw: [f64; 4]) -> f64 {
    let dot = a_xyzw[0] * b_xyzw[0]
        + a_xyzw[1] * b_xyzw[1]
        + a_xyzw[2] * b_xyzw[2]
        + a_xyzw[3] * b_xyzw[3];
    2.0 * dot.abs().clamp(-1.0, 1.0).acos()
}

fn sample_normal(rng: &mut StdRng, mean: f64, sigma: f64) -> f64 {
    let u1: f64 = rng.random_range(1.0e-12_f64..=1.0_f64);
    let u2: f64 = rng.random_range(0.0_f64..1.0_f64);
    let z = (-2.0_f64 * u1.ln()).sqrt() * (2.0_f64 * std::f64::consts::PI * u2).cos();
    mean + sigma * z
}

fn step_static_sr_ukf(filter: &mut SquareRootUkf, now: SimTime, dt_s: f64) {
    filter
        .update_imu(&ImuSample {
            time: now,
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::new(0.0, 0.0, STANDARD_GRAVITY_M_S2),
            healthy: true,
        })
        .unwrap();
    filter.predict(dt_s).unwrap();
}

fn autocorrelation(samples: &[f64], lag: usize) -> f64 {
    let mean = samples.iter().sum::<f64>() / samples.len() as f64;
    let var = samples
        .iter()
        .map(|sample| {
            let centered = sample - mean;
            centered * centered
        })
        .sum::<f64>()
        / samples.len() as f64;
    let cov = samples
        .iter()
        .zip(samples.iter().skip(lag))
        .map(|(a, b)| (a - mean) * (b - mean))
        .sum::<f64>()
        / (samples.len() - lag) as f64;
    cov / var
}

#[test]
fn sr_ukf_and_ekf_agree_on_synthetic_sensor_trajectory() {
    let table = ToleranceTable::from_path(tolerance_table_path()).expect("tolerance table");
    let mut ekf = Ekf::new(EkfParams::default());
    let mut sr = SquareRootUkf::new(SquareRootUkfParams::default());
    let mut truth_pos = Vector3::new(0.0, 0.0, 0.0);
    let mut truth_vel = Vector3::new(0.4, -0.2, 0.0);
    ekf.seed(truth_pos, truth_vel, UnitQuaternion::identity());
    sr.seed(truth_pos, truth_vel, UnitQuaternion::identity());

    let dt_s = 0.01;
    let mut max_position_delta_m: f64 = 0.0;
    let mut max_velocity_delta_m_s: f64 = 0.0;
    let mut max_attitude_delta_rad: f64 = 0.0;

    for k in 0..500_u64 {
        let now = SimTime::from_seconds(k as f64 * dt_s);
        let lateral_accel = Vector3::new(
            0.03 * (0.03 * k as f64).sin(),
            0.02 * (0.05 * k as f64).cos(),
            0.0,
        );
        let imu = ImuSample {
            time: now,
            gyro_rad_s: Vector3::new(0.0, 0.0, 0.002),
            accel_m_s2: Vector3::new(
                lateral_accel.x,
                lateral_accel.y,
                STANDARD_GRAVITY_M_S2 + lateral_accel.z,
            ),
            healthy: true,
        };
        ekf.update_imu(&imu).unwrap();
        sr.update_imu(&imu).unwrap();
        ekf.predict(dt_s).unwrap();
        sr.predict(dt_s).unwrap();

        truth_vel += lateral_accel * dt_s;
        truth_pos += truth_vel * dt_s;

        if k % 10 == 0 {
            let gnss = GnssSample {
                time: now,
                position_eci_m: truth_pos + deterministic_pos_noise(k),
                velocity_eci_m_s: truth_vel + deterministic_vel_noise(k),
                position_bias_eci_m: Vector3::zeros(),
                healthy: true,
            };
            ekf.update_gnss(&gnss).unwrap();
            sr.update_gnss(&gnss).unwrap();
        }
        if k % 25 == 0 {
            let baro = BarometerSample {
                time: now,
                pressure_pa: USSA76_SEA_LEVEL_PRESSURE_PA,
                bias_pa: 0.0,
                healthy: true,
            };
            ekf.update_baro(&baro).unwrap();
            sr.update_baro(&baro).unwrap();
        }

        let ekf_pos = ekf.position();
        let sr_pos = sr.position();
        max_position_delta_m =
            max_position_delta_m.max((ekf_pos.position_eci_m - sr_pos.position_eci_m).norm());
        max_velocity_delta_m_s =
            max_velocity_delta_m_s.max((ekf_pos.velocity_eci_m_s - sr_pos.velocity_eci_m_s).norm());
        max_attitude_delta_rad = max_attitude_delta_rad.max(quaternion_angle_delta_rad(
            ekf.attitude().q_body_to_eci_xyzw,
            sr.attitude().q_body_to_eci_xyzw,
        ));
    }

    table
        .check_metric("max_position_delta_m", max_position_delta_m)
        .unwrap();
    table
        .check_metric("max_velocity_delta_m_s", max_velocity_delta_m_s)
        .unwrap();
    table
        .check_metric("max_attitude_delta_rad", max_attitude_delta_rad)
        .unwrap();
}

#[test]
fn sr_ukf_gnss_innovation_lags_1_to_10_autocorrelation_are_small() {
    let params = SquareRootUkfParams {
        sigma_gnss_pos_m: 0.5,
        sigma_gnss_vel_m_s: 0.05,
        innovation_gate: 1.0e12,
        ..SquareRootUkfParams::default()
    };
    let mut filter = SquareRootUkf::new(params);
    filter.seed(
        Vector3::zeros(),
        Vector3::zeros(),
        UnitQuaternion::identity(),
    );
    let mut rng = StdRng::seed_from_u64(0x5B1);
    let dt_s = 0.01;
    let mut whitened_x = Vec::with_capacity(300);

    for k in 0..300_u64 {
        let now = SimTime::from_seconds(k as f64 * dt_s);
        step_static_sr_ukf(&mut filter, now, dt_s);
        let predicted = filter.position();
        let gnss = GnssSample {
            time: now,
            position_eci_m: predicted.position_eci_m
                + Vector3::new(
                    sample_normal(&mut rng, 0.0, 0.5),
                    sample_normal(&mut rng, 0.0, 0.5),
                    sample_normal(&mut rng, 0.0, 0.5),
                ),
            velocity_eci_m_s: predicted.velocity_eci_m_s
                + Vector3::new(
                    sample_normal(&mut rng, 0.0, 0.05),
                    sample_normal(&mut rng, 0.0, 0.05),
                    sample_normal(&mut rng, 0.0, 0.05),
                ),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        };
        filter.update_gnss(&gnss).unwrap();
        whitened_x.push(filter.status().gnss_innovation_whitened[0]);
    }

    for lag in 1..=10 {
        let autocorr = autocorrelation(&whitened_x, lag);
        assert!(
            autocorr.abs() < 0.2,
            "lag-{lag} autocorrelation {autocorr} indicates non-white SR-UKF GNSS innovations"
        );
    }
}

#[test]
fn sr_ukf_sixty_second_direct_synthetic_truth_run_stays_bounded() {
    let params = SquareRootUkfParams {
        innovation_gate: 1.0e12,
        ..SquareRootUkfParams::default()
    };
    let mut filter = SquareRootUkf::new(params);
    filter.seed(
        Vector3::zeros(),
        Vector3::zeros(),
        UnitQuaternion::identity(),
    );
    let initial_cov = filter.covariance_max_diag();
    let dt_s = 0.01;

    for k in 0..6_000_u64 {
        let now = SimTime::from_seconds(k as f64 * dt_s);
        step_static_sr_ukf(&mut filter, now, dt_s);
        if k % 10 == 0 {
            filter
                .update_gnss(&GnssSample {
                    time: now,
                    position_eci_m: Vector3::zeros(),
                    velocity_eci_m_s: Vector3::zeros(),
                    position_bias_eci_m: Vector3::zeros(),
                    healthy: true,
                })
                .unwrap();
            filter
                .update_baro(&BarometerSample {
                    time: now,
                    pressure_pa: USSA76_SEA_LEVEL_PRESSURE_PA,
                    bias_pa: 0.0,
                    healthy: true,
                })
                .unwrap();
        }

        let position = filter.position();
        assert!(position.position_eci_m.iter().all(|v| v.is_finite()));
        assert!(position.velocity_eci_m_s.iter().all(|v| v.is_finite()));
        assert!(
            filter.covariance_max_diag() <= initial_cov * 100.0,
            "SR-UKF covariance became unbounded at tick {k}: max diag {}",
            filter.covariance_max_diag()
        );
    }

    let attitude = filter.attitude();
    let attitude_error_rad = 2.0
        * (attitude.q_body_to_eci_xyzw[0] * attitude.q_body_to_eci_xyzw[0]
            + attitude.q_body_to_eci_xyzw[1] * attitude.q_body_to_eci_xyzw[1]
            + attitude.q_body_to_eci_xyzw[2] * attitude.q_body_to_eci_xyzw[2])
            .sqrt();
    assert!(attitude_error_rad < 0.1);
}
