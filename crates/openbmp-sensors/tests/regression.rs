//! Phase 2.7.D regression — IMU noise-budget pins + Allan-variance
//! slope check.
//!
//! Three classes of check:
//!
//! First, both shipped IMU noise budgets
//! (`data/sensors/imu-tactical.toml` and
//! `data/sensors/imu-consumer-mems.toml`) parse against Schema 1 and
//! their numerical fields round-trip bit-equally through
//! `ImuNoiseBudget::load_from_str`.
//!
//! Second, an IMU model parameterised with an ARW-only sub-budget
//! produces a per-axis gyro stream whose Allan-deviation log-log
//! slope is close to -1/2 over the 1 ms – 100 ms averaging-time
//! decade, matching the IEEE 952-2020 ARW signature.
//!
//! Third, the same IMU rerun from the same scenario seed produces a
//! bit-identical measurement stream — the determinism contract
//! exercised end-to-end.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::panic
)]

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_core::{Position3, SensorId, SimTime, StepIndex, Velocity3};
use openbmp_sensors::{
    ImuNoiseBudget, SensorMeasurement, SensorTruth, SyntheticImu, SyntheticSensor,
    TriaxialNoiseBudget,
};

const TACTICAL_BUDGET: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/sensors/imu-tactical.toml"
));

const CONSUMER_MEMS_BUDGET: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/sensors/imu-consumer-mems.toml"
));

fn zero_truth() -> SensorTruth {
    SensorTruth {
        position_eci: Position3::new(0.0, 0.0, 0.0),
        velocity_eci: Velocity3::new(0.0, 0.0, 0.0),
        attitude_eci_to_body: UnitQuaternion::identity(),
        angular_velocity_body_rad_s: Vector3::zeros(),
        specific_force_body_m_s2: Vector3::zeros(),
        static_pressure_pa: 0.0,
        altitude_geometric_m: 0.0,
        time: SimTime::ZERO,
    }
}

#[test]
fn tactical_budget_parses_and_carries_expected_fields() {
    let b = ImuNoiseBudget::load_from_str(TACTICAL_BUDGET).unwrap();
    assert_eq!(b.dt_s.to_bits(), 0.005_f64.to_bits());
    assert_eq!(b.gyro.arw_per_sqrt_s.to_bits(), 2.4e-7_f64.to_bits());
    assert_eq!(b.accel.arw_per_sqrt_s.to_bits(), 1.0e-4_f64.to_bits());
}

#[test]
fn consumer_mems_budget_parses_and_carries_expected_fields() {
    let b = ImuNoiseBudget::load_from_str(CONSUMER_MEMS_BUDGET).unwrap();
    assert_eq!(b.dt_s.to_bits(), 0.01_f64.to_bits());
    assert_eq!(b.gyro.arw_per_sqrt_s.to_bits(), 4.8e-6_f64.to_bits());
    assert_eq!(b.accel.arw_per_sqrt_s.to_bits(), 1.0e-2_f64.to_bits());
}

/// Compute the overlapping Allan variance of a sample sequence at
/// averaging time `tau = m * dt`, in canonical Allan-variance form:
///
/// ```text
/// σ²_A(τ) = 1 / (2(N - 2m)) Σ (ȳ_{i+m} - ȳ_i)²
/// ```
///
/// Returns the Allan deviation `σ_A(τ)`.
fn allan_deviation(samples: &[f64], dt: f64, m: usize) -> f64 {
    assert!(m >= 1);
    assert!(samples.len() > 2 * m);
    let n = samples.len();
    // Compute m-sample averages (overlapping).
    let n_avgs = n - m + 1;
    let mut avgs = Vec::with_capacity(n_avgs);
    let inv_m = 1.0 / (m as f64);
    let mut acc: f64 = samples.iter().take(m).sum();
    avgs.push(acc * inv_m);
    for i in m..n {
        acc += samples[i];
        acc -= samples[i - m];
        avgs.push(acc * inv_m);
    }
    // Allan variance from differences of m-spaced averages.
    let mut sum_sq = 0.0_f64;
    let mut count = 0usize;
    let mut i = 0;
    while i + m < avgs.len() {
        let diff = avgs[i + m] - avgs[i];
        sum_sq += diff * diff;
        count += 1;
        i += 1;
    }
    let variance = sum_sq / (2.0 * (count as f64));
    let _ = dt; // dt is informational; the slope check uses tau values directly.
    variance.sqrt()
}

/// Linear regression in log-log space: returns the slope of
/// log10(y) vs log10(x).
fn log_log_slope(xs: &[f64], ys: &[f64]) -> f64 {
    assert_eq!(xs.len(), ys.len());
    assert!(xs.len() >= 2);
    let n = xs.len() as f64;
    let lx: Vec<f64> = xs.iter().map(|x| x.log10()).collect();
    let ly: Vec<f64> = ys.iter().map(|y| y.log10()).collect();
    let mean_x: f64 = lx.iter().sum::<f64>() / n;
    let mean_y: f64 = ly.iter().sum::<f64>() / n;
    let mut num = 0.0_f64;
    let mut den = 0.0_f64;
    for (x, y) in lx.iter().zip(ly.iter()) {
        num += (x - mean_x) * (y - mean_y);
        den += (x - mean_x).powi(2);
    }
    num / den
}

/// Run an ARW-only IMU for `n_steps` and return the gyro-x stream.
fn arw_only_gyro_x_stream(
    arw_per_sqrt_s: f64,
    dt_s: f64,
    n_steps: usize,
    scenario_seed: u64,
) -> Vec<f64> {
    // Pure ARW: zero everything else.
    let triax = TriaxialNoiseBudget::new(
        arw_per_sqrt_s,
        1.0, // theta — irrelevant when sigma = 0
        0.0, // bias_ou_sigma off
        0.0, // rrw off
        0.0, // scale factor off
        0.0, // quantisation off
    )
    .unwrap();
    let budget = ImuNoiseBudget::new(triax, triax, dt_s).unwrap();
    let mut imu = SyntheticImu::new(SensorId::from_path("sensors.imu"), budget).unwrap();
    let truth = zero_truth();
    let mut stream = Vec::with_capacity(n_steps);
    for k in 0..n_steps {
        let m = imu
            .measure(&truth, StepIndex::new(k as u64), scenario_seed)
            .unwrap();
        match m {
            SensorMeasurement::Imu { gyro_rad_s, .. } => stream.push(gyro_rad_s.x),
            _ => panic!("expected Imu variant"),
        }
    }
    stream
}

/// Allan-deviation log-log slope on a long ARW-only stream must be
/// close to -1/2 over the chosen averaging-time decade per IEEE
/// 952-2020.
#[test]
fn arw_only_stream_yields_allan_deviation_slope_near_minus_half() {
    let arw = 1.0e-3_f64; // rad/√s — large enough to dominate any numerical noise floor.
    let dt = 0.001_f64; // 1 kHz
    let n_steps: usize = 60_000; // 60 seconds
    let stream = arw_only_gyro_x_stream(arw, dt, n_steps, 0xA11A);

    // Sample Allan deviation at decade-spaced m values inside the
    // 1 ms .. 100 ms averaging-time decade.
    let m_values: Vec<usize> = vec![1, 2, 4, 8, 16, 32, 64, 128];
    let taus: Vec<f64> = m_values.iter().map(|m| (*m as f64) * dt).collect();
    let devs: Vec<f64> = m_values
        .iter()
        .map(|m| allan_deviation(&stream, dt, *m))
        .collect();

    let slope = log_log_slope(&taus, &devs);
    // Theoretical -0.5; tolerance ±0.1 to cover finite-N sampling.
    assert!(
        (slope - (-0.5)).abs() < 0.1,
        "ARW-only Allan-deviation slope {slope} not within 0.1 of -0.5",
    );
}

/// Bit-stable replay across two reruns of the same IMU + same
/// scenario seed. End-to-end determinism check.
#[test]
fn imu_measurement_stream_is_bit_stable_across_two_runs() {
    let arw = 1.0e-4_f64;
    let dt = 0.01_f64;
    let n_steps: usize = 500;
    let stream_a = arw_only_gyro_x_stream(arw, dt, n_steps, 0xBEEF);
    let stream_b = arw_only_gyro_x_stream(arw, dt, n_steps, 0xBEEF);
    assert_eq!(stream_a.len(), stream_b.len());
    for (a, b) in stream_a.iter().zip(stream_b.iter()) {
        assert_eq!(a.to_bits(), b.to_bits());
    }
}
