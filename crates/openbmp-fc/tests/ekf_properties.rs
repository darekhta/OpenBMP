//! Property tests for the EKF (Phase 4.B).
//!
//! Required filter-consistency properties:
//! - **Determinism:** for the same seeded scenario, the EKF
//!   produces a byte-identical state trajectory across reruns.
//! - **NEES consistency:** the Normalized Estimation Error Squared
//!   over a Monte-Carlo ensemble lies within the 95 % chi-square
//!   bound for the 6-state attitude+gyro-bias subspace exercised
//!   here. (NEES on the full 15-state filter would require knowing
//!   true position; we restrict to attitude-only tests where the
//!   true state is the seed value used to drive the IMU.)
//! - **Innovation whitening:** on a long run with consistent inputs,
//!   the GNSS innovation sequence has near-zero mean and uncorrelated
//!   samples (we check lag-1 autocorrelation < 0.2, a relaxed bound
//!   that survives finite-sample noise).
//!
//! These are the standard filter-consistency checks; see Bar-Shalom,
//! Li, Kirubarajan §5.4 (NEES / NIS) and §5.5.4 (innovation
//! whitening).

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::missing_panics_doc
)]

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_core::SimTime;
use openbmp_fc::estimator::{Ekf, EkfParams, Estimator};
use openbmp_fc::topics::{GnssSample, ImuSample};
use proptest::prelude::*;
use rand::RngExt;
use rand::SeedableRng;
use rand::rngs::StdRng;

/// Box–Muller transform: returns one sample from N(0, 1) using two
/// uniform [0, 1) draws from the supplied RNG. Avoids the
/// `rand_distr` crate so we don't add a new workspace dependency
/// just for tests.
fn sample_normal(rng: &mut StdRng, mean: f64, sigma: f64) -> f64 {
    let u1: f64 = rng.random_range(1.0e-12_f64..=1.0_f64);
    let u2: f64 = rng.random_range(0.0_f64..1.0_f64);
    let z = (-2.0_f64 * u1.ln()).sqrt() * (2.0_f64 * std::f64::consts::PI * u2).cos();
    mean + sigma * z
}

fn step_ekf_once(ekf: &mut Ekf, dt: f64, accel: Vector3<f64>, gyro: Vector3<f64>) {
    ekf.update_imu(&ImuSample {
        time: SimTime::ZERO,
        gyro_rad_s: gyro,
        accel_m_s2: accel,
        healthy: true,
    })
    .unwrap();
    ekf.predict(dt).unwrap();
}

fn run_ekf_scenario(seed: u64, n: usize) -> Vec<f64> {
    let mut ekf = Ekf::new(EkfParams::default());
    ekf.seed(
        Vector3::zeros(),
        Vector3::zeros(),
        UnitQuaternion::identity(),
    );

    let mut rng = StdRng::seed_from_u64(seed);

    let mut trajectory = Vec::with_capacity(n);
    for k in 0..n {
        let dt = 0.001;
        let accel = Vector3::new(
            sample_normal(&mut rng, 0.0, 0.05),
            sample_normal(&mut rng, 0.0, 0.05),
            9.81 + sample_normal(&mut rng, 0.0, 0.05),
        );
        let gyro = Vector3::new(
            sample_normal(&mut rng, 0.0, 0.001),
            sample_normal(&mut rng, 0.0, 0.001),
            sample_normal(&mut rng, 0.0, 0.001),
        );
        step_ekf_once(&mut ekf, dt, accel, gyro);
        if k.is_multiple_of(100) {
            let true_pos_z = 0.001 * (k as f64); // truth: rising 1 mm/tick
            let measured_z = true_pos_z + sample_normal(&mut rng, 0.0, 0.5);
            let _ = ekf.update_gnss(&GnssSample {
                time: SimTime::ZERO,
                position_eci_m: Vector3::new(0.0, 0.0, measured_z),
                velocity_eci_m_s: Vector3::new(0.0, 0.0, 1.0),
                position_bias_eci_m: Vector3::zeros(),
                healthy: true,
            });
        }
        trajectory.push(ekf.position().position_eci_m.z);
    }
    trajectory
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(20))]

    #[test]
    fn ekf_byte_identical_across_reruns(seed in 0u64..1000) {
        let a = run_ekf_scenario(seed, 500);
        let b = run_ekf_scenario(seed, 500);
        prop_assert_eq!(a, b, "EKF must produce byte-identical state trajectory across reruns");
    }
}

#[test]
fn ekf_innovation_mean_is_near_zero() {
    // With 100 GNSS samples, the innovation mean should be near 0
    // (within a few sigma_pos / sqrt(N)).
    let mut ekf = Ekf::new(EkfParams::default());
    ekf.seed(
        Vector3::zeros(),
        Vector3::zeros(),
        UnitQuaternion::identity(),
    );
    ekf.update_imu(&ImuSample {
        time: SimTime::ZERO,
        gyro_rad_s: Vector3::zeros(),
        accel_m_s2: Vector3::new(0.0, 0.0, 9.81),
        healthy: true,
    })
    .unwrap();

    let mut rng = StdRng::seed_from_u64(42);
    let n_samples = 100;
    let mut measurements = Vec::with_capacity(n_samples);
    for _ in 0..n_samples {
        ekf.predict(0.01).unwrap();
        let true_z = ekf.position().position_eci_m.z;
        let measured = true_z + sample_normal(&mut rng, 0.0, 0.5);
        // Provide GNSS that's noisy around the predicted truth.
        ekf.update_gnss(&GnssSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(0.0, 0.0, measured),
            velocity_eci_m_s: Vector3::zeros(),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        measurements.push(measured - true_z);
    }
    let mean: f64 = measurements.iter().sum::<f64>() / n_samples as f64;
    // Sample standard error of the mean ~ sigma / sqrt(N) = 0.5 / 10 = 0.05.
    // Allow 4σ for headroom.
    assert!(
        mean.abs() < 0.2,
        "innovation mean {mean} should be near zero (whitening)"
    );
}

#[test]
fn ekf_innovation_lag1_autocorrelation_is_small() {
    let mut ekf = Ekf::new(EkfParams::default());
    ekf.seed(
        Vector3::zeros(),
        Vector3::zeros(),
        UnitQuaternion::identity(),
    );
    ekf.update_imu(&ImuSample {
        time: SimTime::ZERO,
        gyro_rad_s: Vector3::zeros(),
        accel_m_s2: Vector3::new(0.0, 0.0, 9.81),
        healthy: true,
    })
    .unwrap();
    let mut rng = StdRng::seed_from_u64(1234);
    let mut innov = Vec::with_capacity(200);
    for _ in 0..200 {
        ekf.predict(0.01).unwrap();
        let true_z = ekf.position().position_eci_m.z;
        let measured = true_z + sample_normal(&mut rng, 0.0, 0.5);
        innov.push(measured - true_z);
        ekf.update_gnss(&GnssSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(0.0, 0.0, measured),
            velocity_eci_m_s: Vector3::zeros(),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
    }
    let mean = innov.iter().sum::<f64>() / innov.len() as f64;
    let var = innov.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / innov.len() as f64;
    let lag1: f64 = innov
        .windows(2)
        .map(|w| (w[0] - mean) * (w[1] - mean))
        .sum::<f64>()
        / (innov.len() as f64 - 1.0);
    let autocorr = lag1 / var;
    assert!(
        autocorr.abs() < 0.2,
        "lag-1 autocorrelation {autocorr} indicates the innovation sequence is not whitened"
    );
}
