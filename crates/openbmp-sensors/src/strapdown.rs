//! High-rate strapdown IMU increment helpers.
//!
//! This module is the WP-09.4 substrate: deterministic inertial
//! increment records, quantization, constant-truth window generation,
//! and a two-sample coning/sculling reference. [`crate::SyntheticImu`]
//! can publish generated windows when configured by the scenario schema.

use nalgebra::{UnitQuaternion, Vector3};

use crate::error::SensorError;

/// One high-rate inertial increment pair, matching a strapdown IMU
/// wire-format sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct InertialIncrement {
    /// Integrated body angular rate over the sub-interval (rad).
    pub delta_theta_rad: Vector3<f64>,
    /// Integrated body specific force over the sub-interval (m/s).
    pub delta_v_m_s: Vector3<f64>,
    /// Sub-interval length (s).
    pub dt_s: f64,
    /// High-rate sample sequence number.
    pub seq: u64,
}

/// Independent quantization LSBs for generated inertial increments.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IncrementQuantization {
    /// Delta-theta LSB (rad). `0` disables angular quantization.
    pub delta_theta_lsb_rad: f64,
    /// Delta-v LSB (m/s). `0` disables velocity quantization.
    pub delta_v_lsb_m_s: f64,
}

impl IncrementQuantization {
    /// Construct after validating finite non-negative LSB values.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::NonFinite`] for NaN/Inf values and
    /// [`SensorError::InvalidParameter`] for negative LSBs.
    pub fn new(delta_theta_lsb_rad: f64, delta_v_lsb_m_s: f64) -> Result<Self, SensorError> {
        if !delta_theta_lsb_rad.is_finite() || !delta_v_lsb_m_s.is_finite() {
            return Err(SensorError::NonFinite {
                reason: "strapdown increment quantization LSB is non-finite",
            });
        }
        if delta_theta_lsb_rad < 0.0 || delta_v_lsb_m_s < 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "strapdown increment quantization LSB must be non-negative",
            });
        }
        Ok(Self {
            delta_theta_lsb_rad,
            delta_v_lsb_m_s,
        })
    }

    /// No quantization.
    #[must_use]
    pub const fn none() -> Self {
        Self {
            delta_theta_lsb_rad: 0.0,
            delta_v_lsb_m_s: 0.0,
        }
    }
}

/// Configuration for a synthetic IMU high-rate increment window.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct HighRateImuConfig {
    /// Number of high-rate sub-samples generated per IMU sample.
    pub sub_samples: u32,
    /// Delta-theta / delta-v quantization applied to each sub-sample.
    pub quantization: IncrementQuantization,
}

impl HighRateImuConfig {
    /// Construct after validating a positive sub-sample count and
    /// finite non-negative quantization LSBs.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidParameter`] when `sub_samples`
    /// is zero. See [`IncrementQuantization::new`] for LSB failures.
    pub fn new(
        sub_samples: u32,
        delta_theta_lsb_rad: f64,
        delta_v_lsb_m_s: f64,
    ) -> Result<Self, SensorError> {
        if sub_samples == 0 {
            return Err(SensorError::InvalidParameter {
                reason: "strapdown high-rate sub-samples must be > 0",
            });
        }
        Ok(Self {
            sub_samples,
            quantization: IncrementQuantization::new(delta_theta_lsb_rad, delta_v_lsb_m_s)?,
        })
    }
}

/// Instantaneous body-frame truth used by high-rate strapdown integration.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct InertialTruthSample {
    /// Body-frame angular velocity (rad/s).
    pub angular_velocity_body_rad_s: Vector3<f64>,
    /// Body-frame specific force (m/s²).
    pub specific_force_body_m_s2: Vector3<f64>,
}

/// Coning/sculling reference algorithm selector.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConingScullingAlgo {
    /// Adjacent-pair two-sample correction.
    TwoSample,
}

/// Moderate-rate coning/sculling accumulation over a high-rate window.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ConingScullingWindow {
    /// Sum of input delta-theta increments before coning correction.
    pub summed_delta_theta_rad: Vector3<f64>,
    /// Sum of input delta-v increments before rotation/sculling correction.
    pub summed_delta_v_m_s: Vector3<f64>,
    /// Two-sample coning correction (rad).
    pub coning_correction_rad: Vector3<f64>,
    /// Body-frame velocity rotation correction (m/s).
    pub velocity_rotation_correction_m_s: Vector3<f64>,
    /// Body-frame sculling correction (m/s).
    pub sculling_correction_m_s: Vector3<f64>,
    /// Rotation vector applied over the moderate-rate window (rad).
    pub rotation_vector_rad: Vector3<f64>,
    /// Corrected body-frame delta-v over the moderate-rate window (m/s).
    pub body_delta_v_m_s: Vector3<f64>,
    /// Sum of sub-interval durations (s).
    pub dt_s: f64,
    /// First sequence number in the window.
    pub first_seq: u64,
    /// Number of increments consumed.
    pub sample_count: usize,
}

fn require_finite_vector(v: &Vector3<f64>, reason: &'static str) -> Result<(), SensorError> {
    if v.iter().all(|component| component.is_finite()) {
        Ok(())
    } else {
        Err(SensorError::NonFinite { reason })
    }
}

fn require_positive_dt(dt_s: f64) -> Result<(), SensorError> {
    if !dt_s.is_finite() {
        return Err(SensorError::NonFinite {
            reason: "strapdown increment dt is non-finite",
        });
    }
    if dt_s <= 0.0 {
        return Err(SensorError::InvalidParameter {
            reason: "strapdown increment dt must be > 0",
        });
    }
    Ok(())
}

fn require_finite_time(time_s: f64) -> Result<(), SensorError> {
    if time_s.is_finite() {
        Ok(())
    } else {
        Err(SensorError::NonFinite {
            reason: "strapdown truth sample time is non-finite",
        })
    }
}

fn require_finite_truth(truth: &InertialTruthSample) -> Result<(), SensorError> {
    require_finite_vector(
        &truth.angular_velocity_body_rad_s,
        "strapdown angular velocity is non-finite",
    )?;
    require_finite_vector(
        &truth.specific_force_body_m_s2,
        "strapdown specific force is non-finite",
    )
}

fn quantize_scalar(value: f64, lsb: f64) -> Result<f64, SensorError> {
    if !value.is_finite() {
        return Err(SensorError::NonFinite {
            reason: "strapdown increment component is non-finite",
        });
    }
    if lsb == 0.0 {
        return Ok(value);
    }
    let quantized = (value / lsb).round() * lsb;
    if quantized.is_finite() {
        Ok(quantized)
    } else {
        Err(SensorError::NonFinite {
            reason: "strapdown increment quantization produced a non-finite value",
        })
    }
}

/// Quantize each vector component with a deterministic round-to-nearest
/// LSB. `lsb = 0` disables quantization.
///
/// # Errors
///
/// Returns [`SensorError::NonFinite`] for non-finite vector
/// components or quantized results.
pub fn quantize_increment_vector(
    value: Vector3<f64>,
    lsb: f64,
) -> Result<Vector3<f64>, SensorError> {
    if lsb < 0.0 {
        return Err(SensorError::InvalidParameter {
            reason: "strapdown increment vector LSB must be non-negative",
        });
    }
    Ok(Vector3::new(
        quantize_scalar(value.x, lsb)?,
        quantize_scalar(value.y, lsb)?,
        quantize_scalar(value.z, lsb)?,
    ))
}

/// Generate one inertial increment from constant body-rate and
/// constant body-frame specific-force truth.
///
/// # Errors
///
/// Returns [`SensorError::NonFinite`] / [`SensorError::InvalidParameter`]
/// for invalid inputs.
pub fn inertial_increment_from_constant_truth(
    angular_velocity_body_rad_s: Vector3<f64>,
    specific_force_body_m_s2: Vector3<f64>,
    dt_s: f64,
    seq: u64,
    quantization: IncrementQuantization,
) -> Result<InertialIncrement, SensorError> {
    require_positive_dt(dt_s)?;
    require_finite_vector(
        &angular_velocity_body_rad_s,
        "strapdown angular velocity is non-finite",
    )?;
    require_finite_vector(
        &specific_force_body_m_s2,
        "strapdown specific force is non-finite",
    )?;
    let delta_theta_rad = quantize_increment_vector(
        angular_velocity_body_rad_s * dt_s,
        quantization.delta_theta_lsb_rad,
    )?;
    let delta_v_m_s = quantize_increment_vector(
        specific_force_body_m_s2 * dt_s,
        quantization.delta_v_lsb_m_s,
    )?;
    Ok(InertialIncrement {
        delta_theta_rad,
        delta_v_m_s,
        dt_s,
        seq,
    })
}

/// Generate one inertial increment by RK4 quadrature over
/// time-varying body-rate and specific-force truth.
///
/// The sampler is called in fixed order at `t0`, `t0 + dt/2`,
/// `t0 + dt/2`, and `t0 + dt`. This intentionally exposes the same
/// operand order to deterministic validation harnesses that compare
/// generated increments byte-for-byte.
///
/// # Errors
///
/// Returns [`SensorError::NonFinite`] / [`SensorError::InvalidParameter`]
/// for invalid time, truth, or increment inputs.
pub fn inertial_increment_from_rk4_truth<F>(
    mut truth_at: F,
    start_time_s: f64,
    dt_s: f64,
    seq: u64,
    quantization: IncrementQuantization,
) -> Result<InertialIncrement, SensorError>
where
    F: FnMut(f64) -> InertialTruthSample,
{
    require_finite_time(start_time_s)?;
    require_positive_dt(dt_s)?;
    let half_dt_s = 0.5 * dt_s;
    let mid_time_s = start_time_s + half_dt_s;
    let end_time_s = start_time_s + dt_s;
    require_finite_time(mid_time_s)?;
    require_finite_time(end_time_s)?;

    let k1 = truth_at(start_time_s);
    require_finite_truth(&k1)?;
    let k2 = truth_at(mid_time_s);
    require_finite_truth(&k2)?;
    let k3 = truth_at(mid_time_s);
    require_finite_truth(&k3)?;
    let k4 = truth_at(end_time_s);
    require_finite_truth(&k4)?;

    let delta_theta_rad = (dt_s / 6.0)
        * (k1.angular_velocity_body_rad_s
            + 2.0 * k2.angular_velocity_body_rad_s
            + 2.0 * k3.angular_velocity_body_rad_s
            + k4.angular_velocity_body_rad_s);
    let delta_v_m_s = (dt_s / 6.0)
        * (k1.specific_force_body_m_s2
            + 2.0 * k2.specific_force_body_m_s2
            + 2.0 * k3.specific_force_body_m_s2
            + k4.specific_force_body_m_s2);

    Ok(InertialIncrement {
        delta_theta_rad: quantize_increment_vector(
            delta_theta_rad,
            quantization.delta_theta_lsb_rad,
        )?,
        delta_v_m_s: quantize_increment_vector(delta_v_m_s, quantization.delta_v_lsb_m_s)?,
        dt_s,
        seq,
    })
}

/// Generate a fixed-length high-rate window from constant truth.
///
/// This is a deterministic substrate for tests and flight-software
/// validation harnesses; the full WP-09.4 synthetic IMU path will use
/// time-varying truth and scenario wiring.
///
/// # Errors
///
/// Returns [`SensorError::InvalidParameter`] when `sub_samples == 0`
/// or any sub-interval is invalid.
pub fn integrate_constant_truth_window(
    angular_velocity_body_rad_s: Vector3<f64>,
    specific_force_body_m_s2: Vector3<f64>,
    sub_dt_s: f64,
    sub_samples: u32,
    first_seq: u64,
    quantization: IncrementQuantization,
) -> Result<Vec<InertialIncrement>, SensorError> {
    if sub_samples == 0 {
        return Err(SensorError::InvalidParameter {
            reason: "strapdown window requires at least one sub-sample",
        });
    }
    let capacity = usize::try_from(sub_samples).map_err(|_| SensorError::InvalidParameter {
        reason: "strapdown sub-sample count does not fit usize",
    })?;
    let mut increments = Vec::with_capacity(capacity);
    for i in 0..sub_samples {
        increments.push(inertial_increment_from_constant_truth(
            angular_velocity_body_rad_s,
            specific_force_body_m_s2,
            sub_dt_s,
            first_seq + u64::from(i),
            quantization,
        )?);
    }
    Ok(increments)
}

/// Generate a fixed-length high-rate window from time-varying truth
/// using RK4 quadrature per sub-interval.
///
/// # Errors
///
/// Returns [`SensorError::InvalidParameter`] when `sub_samples == 0`
/// or any sub-interval is invalid.
pub fn integrate_rk4_truth_window<F>(
    mut truth_at: F,
    start_time_s: f64,
    sub_dt_s: f64,
    sub_samples: u32,
    first_seq: u64,
    quantization: IncrementQuantization,
) -> Result<Vec<InertialIncrement>, SensorError>
where
    F: FnMut(f64) -> InertialTruthSample,
{
    if sub_samples == 0 {
        return Err(SensorError::InvalidParameter {
            reason: "strapdown window requires at least one sub-sample",
        });
    }
    require_finite_time(start_time_s)?;
    require_positive_dt(sub_dt_s)?;
    let capacity = usize::try_from(sub_samples).map_err(|_| SensorError::InvalidParameter {
        reason: "strapdown sub-sample count does not fit usize",
    })?;
    let mut increments = Vec::with_capacity(capacity);
    for i in 0..sub_samples {
        let offset_s = f64::from(i) * sub_dt_s;
        let sub_start_s = start_time_s + offset_s;
        require_finite_time(sub_start_s)?;
        increments.push(inertial_increment_from_rk4_truth(
            &mut truth_at,
            sub_start_s,
            sub_dt_s,
            first_seq + u64::from(i),
            quantization,
        )?);
    }
    Ok(increments)
}

/// Accumulate a moderate-rate two-sample coning/sculling window.
///
/// # Errors
///
/// Returns [`SensorError::InvalidParameter`] for an empty window or
/// non-positive increment `dt_s`; [`SensorError::NonFinite`] for
/// non-finite arithmetic.
pub fn coning_sculling_window(
    increments: &[InertialIncrement],
    algo: ConingScullingAlgo,
) -> Result<ConingScullingWindow, SensorError> {
    if increments.is_empty() {
        return Err(SensorError::InvalidParameter {
            reason: "strapdown coning/sculling window is empty",
        });
    }
    let mut alpha_prev = Vector3::zeros();
    let mut delta_v_prev = Vector3::zeros();
    let mut summed_delta_theta_rad = Vector3::zeros();
    let mut summed_delta_v_m_s = Vector3::zeros();
    let mut coning_correction_rad = Vector3::zeros();
    let mut sculling_correction_m_s = Vector3::zeros();
    let mut dt_s = 0.0;

    for (index, increment) in increments.iter().enumerate() {
        require_positive_dt(increment.dt_s)?;
        require_finite_vector(
            &increment.delta_theta_rad,
            "strapdown delta-theta is non-finite",
        )?;
        require_finite_vector(&increment.delta_v_m_s, "strapdown delta-v is non-finite")?;

        if matches!(algo, ConingScullingAlgo::TwoSample) && index > 0 {
            coning_correction_rad += (2.0 / 3.0)
                * increments[index - 1]
                    .delta_theta_rad
                    .cross(&increment.delta_theta_rad);
        }

        let alpha_i = summed_delta_theta_rad + increment.delta_theta_rad;
        sculling_correction_m_s +=
            0.5 * (alpha_prev.cross(&increment.delta_v_m_s) + delta_v_prev.cross(&alpha_i));
        summed_delta_theta_rad = alpha_i;
        summed_delta_v_m_s += increment.delta_v_m_s;
        alpha_prev = alpha_i;
        delta_v_prev = increment.delta_v_m_s;
        dt_s += increment.dt_s;
    }

    let velocity_rotation_correction_m_s = 0.5 * summed_delta_theta_rad.cross(&summed_delta_v_m_s);
    let rotation_vector_rad = summed_delta_theta_rad + coning_correction_rad;
    let body_delta_v_m_s =
        summed_delta_v_m_s + velocity_rotation_correction_m_s + sculling_correction_m_s;

    for (value, reason) in [
        (
            &coning_correction_rad,
            "strapdown coning correction is non-finite",
        ),
        (
            &velocity_rotation_correction_m_s,
            "strapdown velocity-rotation correction is non-finite",
        ),
        (
            &sculling_correction_m_s,
            "strapdown sculling correction is non-finite",
        ),
        (
            &rotation_vector_rad,
            "strapdown rotation vector is non-finite",
        ),
        (&body_delta_v_m_s, "strapdown body delta-v is non-finite"),
    ] {
        require_finite_vector(value, reason)?;
    }

    Ok(ConingScullingWindow {
        summed_delta_theta_rad,
        summed_delta_v_m_s,
        coning_correction_rad,
        velocity_rotation_correction_m_s,
        sculling_correction_m_s,
        rotation_vector_rad,
        body_delta_v_m_s,
        dt_s,
        first_seq: increments[0].seq,
        sample_count: increments.len(),
    })
}

/// Apply the coning/sculling reference update to an attitude quaternion.
///
/// The returned delta-v is still body-frame; navigation-frame gravity,
/// Coriolis, and transport-rate compensation belong to the downstream
/// navigation mechanization.
///
/// # Errors
///
/// See [`coning_sculling_window`].
pub fn coning_sculling_update(
    q_in: UnitQuaternion<f64>,
    increments: &[InertialIncrement],
    algo: ConingScullingAlgo,
) -> Result<(UnitQuaternion<f64>, Vector3<f64>), SensorError> {
    let window = coning_sculling_window(increments, algo)?;
    let dq = UnitQuaternion::from_scaled_axis(window.rotation_vector_rad);
    Ok((q_in * dq, window.body_delta_v_m_s))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn increment(
        delta_theta_rad: Vector3<f64>,
        delta_v_m_s: Vector3<f64>,
        seq: u64,
    ) -> InertialIncrement {
        InertialIncrement {
            delta_theta_rad,
            delta_v_m_s,
            dt_s: 0.001,
            seq,
        }
    }

    #[test]
    fn increment_quantization_rejects_invalid_lsb() {
        assert!(matches!(
            IncrementQuantization::new(-1.0, 0.0),
            Err(SensorError::InvalidParameter { .. })
        ));
        assert!(matches!(
            IncrementQuantization::new(f64::NAN, 0.0),
            Err(SensorError::NonFinite { .. })
        ));
    }

    #[test]
    fn high_rate_config_rejects_zero_sub_samples() {
        assert!(matches!(
            HighRateImuConfig::new(0, 0.0, 0.0),
            Err(SensorError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn constant_truth_increment_quantizes_delta_theta_and_delta_v() {
        let quantization = IncrementQuantization::new(0.05, 0.25).unwrap();
        let increment = inertial_increment_from_constant_truth(
            Vector3::new(12.6, -7.4, 0.0),
            Vector3::new(60.0, -20.0, 0.0),
            0.01,
            7,
            quantization,
        )
        .unwrap();

        assert_eq!(increment.seq, 7);
        assert_abs_diff_eq!(increment.delta_theta_rad.x, 0.15, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increment.delta_theta_rad.y, -0.05, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increment.delta_v_m_s.x, 0.50, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increment.delta_v_m_s.y, -0.25, epsilon = 1.0e-15);
    }

    #[test]
    fn constant_truth_window_assigns_locked_sequence_numbers() {
        let increments = integrate_constant_truth_window(
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
            0.002,
            3,
            100,
            IncrementQuantization::none(),
        )
        .unwrap();

        assert_eq!(increments.len(), 3);
        assert_eq!(increments[0].seq, 100);
        assert_eq!(increments[2].seq, 102);
        assert_abs_diff_eq!(increments[1].delta_theta_rad.y, 0.004, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increments[1].delta_v_m_s.z, 0.012, epsilon = 1.0e-15);
    }

    #[test]
    fn rk4_truth_increment_integrates_linear_truth_exactly() {
        let increment = inertial_increment_from_rk4_truth(
            |time_s| InertialTruthSample {
                angular_velocity_body_rad_s: Vector3::new(1.0 + 2.0 * time_s, -2.0, 0.5),
                specific_force_body_m_s2: Vector3::new(3.0, -4.0 + time_s, 2.0 * time_s),
            },
            0.25,
            0.5,
            17,
            IncrementQuantization::none(),
        )
        .unwrap();

        assert_eq!(increment.seq, 17);
        assert_abs_diff_eq!(increment.delta_theta_rad.x, 1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increment.delta_theta_rad.y, -1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increment.delta_theta_rad.z, 0.25, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increment.delta_v_m_s.x, 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increment.delta_v_m_s.y, -1.75, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increment.delta_v_m_s.z, 0.5, epsilon = 1.0e-15);
    }

    #[test]
    fn rk4_truth_window_assigns_locked_sequence_numbers() {
        let increments = integrate_rk4_truth_window(
            |time_s| InertialTruthSample {
                angular_velocity_body_rad_s: Vector3::new(time_s, 0.0, 0.0),
                specific_force_body_m_s2: Vector3::new(0.0, 2.0 * time_s, 0.0),
            },
            0.0,
            0.1,
            2,
            90,
            IncrementQuantization::none(),
        )
        .unwrap();

        assert_eq!(increments.len(), 2);
        assert_eq!(increments[0].seq, 90);
        assert_eq!(increments[1].seq, 91);
        assert_abs_diff_eq!(increments[0].delta_theta_rad.x, 0.005, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increments[1].delta_theta_rad.x, 0.015, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increments[0].delta_v_m_s.y, 0.010, epsilon = 1.0e-15);
        assert_abs_diff_eq!(increments[1].delta_v_m_s.y, 0.030, epsilon = 1.0e-15);
    }

    #[test]
    fn rk4_truth_increment_rejects_non_finite_truth() {
        let err = inertial_increment_from_rk4_truth(
            |_| InertialTruthSample {
                angular_velocity_body_rad_s: Vector3::new(f64::NAN, 0.0, 0.0),
                specific_force_body_m_s2: Vector3::zeros(),
            },
            0.0,
            0.1,
            0,
            IncrementQuantization::none(),
        )
        .unwrap_err();

        assert!(matches!(err, SensorError::NonFinite { .. }));
    }

    #[test]
    fn two_sample_coning_matches_closed_form_cross_product() {
        let a = 1.0e-3;
        let b = 2.0e-3;
        let increments = [
            increment(Vector3::new(a, 0.0, 0.0), Vector3::zeros(), 0),
            increment(Vector3::new(0.0, b, 0.0), Vector3::zeros(), 1),
        ];

        let window = coning_sculling_window(&increments, ConingScullingAlgo::TwoSample).unwrap();
        let expected_coning = Vector3::new(0.0, 0.0, (2.0 / 3.0) * a * b);
        let expected_rotation = Vector3::new(a, b, expected_coning.z);

        assert_abs_diff_eq!(
            window.coning_correction_rad,
            expected_coning,
            epsilon = 1.0e-18
        );
        assert_abs_diff_eq!(
            window.rotation_vector_rad,
            expected_rotation,
            epsilon = 1.0e-18
        );

        let (q, body_delta_v) = coning_sculling_update(
            UnitQuaternion::identity(),
            &increments,
            ConingScullingAlgo::TwoSample,
        )
        .unwrap();
        assert_abs_diff_eq!(q.scaled_axis(), expected_rotation, epsilon = 1.0e-15);
        assert_abs_diff_eq!(body_delta_v, Vector3::zeros(), epsilon = 1.0e-18);
    }

    #[test]
    fn two_sample_sculling_matches_manufactured_cross_product() {
        let a = 2.0e-3;
        let c = 5.0e-3;
        let increments = [
            increment(Vector3::new(a, 0.0, 0.0), Vector3::zeros(), 10),
            increment(Vector3::zeros(), Vector3::new(0.0, c, 0.0), 11),
        ];

        let window = coning_sculling_window(&increments, ConingScullingAlgo::TwoSample).unwrap();
        let expected = Vector3::new(0.0, c, a * c);

        assert_abs_diff_eq!(
            window.velocity_rotation_correction_m_s,
            Vector3::new(0.0, 0.0, 0.5 * a * c),
            epsilon = 1.0e-18
        );
        assert_abs_diff_eq!(
            window.sculling_correction_m_s,
            Vector3::new(0.0, 0.0, 0.5 * a * c),
            epsilon = 1.0e-18
        );
        assert_abs_diff_eq!(window.body_delta_v_m_s, expected, epsilon = 1.0e-18);
    }

    #[test]
    fn coning_sculling_rejects_empty_or_bad_increment_window() {
        assert!(matches!(
            coning_sculling_window(&[], ConingScullingAlgo::TwoSample),
            Err(SensorError::InvalidParameter { .. })
        ));

        let bad = [InertialIncrement {
            delta_theta_rad: Vector3::zeros(),
            delta_v_m_s: Vector3::zeros(),
            dt_s: 0.0,
            seq: 0,
        }];
        assert!(matches!(
            coning_sculling_window(&bad, ConingScullingAlgo::TwoSample),
            Err(SensorError::InvalidParameter { .. })
        ));
    }
}
