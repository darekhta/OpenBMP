//! Load-relief helpers for ascent guidance.
//!
//! The module stays inside `openbmp-fc` and consumes only controller-side
//! estimates: inertial velocity, modeled wind, attitude, and density. It
//! does not import scenario, runner, or target/aimpoint vocabulary.

use std::format;
use std::string::{String, ToString};
use std::vec::Vec;

use nalgebra::{UnitQuaternion, Vector3};
#[cfg(not(feature = "std"))]
use num_traits::Float;

const MIN_FORWARD_SPEED_M_S: f64 = 1.0e-9;
const BLEND_SUM_TOLERANCE: f64 = 1.0e-9;

/// Blended drift-minimum / load-minimum ascent load-relief parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadReliefParams {
    /// Rigid-body attitude proportional gain carried with the I-load.
    pub a0: f64,
    /// Rigid-body rate gain carried with the I-load.
    pub a1: f64,
    /// Maximum angle-of-attack feedback gain at full load-min blend.
    pub g_alpha_max: f64,
    /// Lateral-acceleration feedback gain.
    pub g_acc: f64,
    /// Strictly ascending `(schedule_value, blend)` nodes, with blend in `[0, 1]`.
    pub blend_schedule: Vec<(f64, f64)>,
    /// Structural `q * alpha` ceiling, Pa-rad.
    pub q_alpha_limit_pa_rad: f64,
}

impl LoadReliefParams {
    /// Validate load-relief parameters before runtime use.
    ///
    /// # Errors
    ///
    /// Returns a validation string when gains, schedule nodes, or the structural limit are
    /// non-finite or malformed.
    pub fn validate(&self) -> Result<(), String> {
        require_finite("a0", self.a0)?;
        require_finite("a1", self.a1)?;
        require_finite("g_alpha_max", self.g_alpha_max)?;
        require_finite("g_acc", self.g_acc)?;
        require_positive_finite("q_alpha_limit_pa_rad", self.q_alpha_limit_pa_rad)?;
        if self.blend_schedule.is_empty() {
            return Err("blend_schedule must contain at least one node".to_string());
        }
        let mut previous = f64::NEG_INFINITY;
        for (idx, &(schedule_value, blend)) in self.blend_schedule.iter().enumerate() {
            require_finite("blend_schedule.schedule_value", schedule_value)?;
            require_finite("blend_schedule.blend", blend)?;
            if !(0.0..=1.0).contains(&blend) {
                return Err(format!("blend_schedule[{idx}].blend must be in [0, 1]"));
            }
            if schedule_value <= previous {
                return Err("blend_schedule values must be strictly ascending".to_string());
            }
            previous = schedule_value;
        }
        Ok(())
    }

    /// Interpolate the load-minimum blend coefficient at `schedule_value`.
    ///
    /// Values outside the table clamp to the nearest endpoint.
    ///
    /// # Errors
    ///
    /// Returns a validation string when the table or query is invalid.
    pub fn blend_at(&self, schedule_value: f64) -> Result<f64, String> {
        self.validate()?;
        require_finite("schedule_value", schedule_value)?;
        if schedule_value <= self.blend_schedule[0].0 {
            return Ok(self.blend_schedule[0].1);
        }
        let last = self.blend_schedule[self.blend_schedule.len() - 1];
        if schedule_value >= last.0 {
            return Ok(last.1);
        }
        for window in self.blend_schedule.windows(2) {
            let (x0, y0) = window[0];
            let (x1, y1) = window[1];
            if schedule_value >= x0 && schedule_value <= x1 {
                let t = (schedule_value - x0) / (x1 - x0);
                return Ok(y0 + t * (y1 - y0));
            }
        }
        Err("blend_schedule interpolation failed".to_string())
    }
}

/// Estimated aerodynamic angles and dynamic pressure.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AlphaEstimate {
    /// Angle of attack, `atan2(w_rel_body, u_rel_body)`, rad.
    pub alpha_rad: f64,
    /// Sideslip angle, `atan2(v_rel_body, u_rel_body)`, rad.
    pub beta_rad: f64,
    /// Dynamic pressure from density and relative speed, Pa.
    pub qbar_pa: f64,
    /// Air-relative velocity in body axes, m/s.
    pub air_relative_body_m_s: Vector3<f64>,
}

/// Result of evaluating the load-relief law at one tick.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LoadReliefCommand {
    /// Raw aerodynamic estimate before limiting.
    pub estimate: AlphaEstimate,
    /// Interpolated load-minimum blend coefficient.
    pub blend: f64,
    /// Effective alpha after the structural `q * alpha` limiter.
    pub limited_alpha_rad: f64,
    /// Effective beta after the structural `q * beta` limiter.
    pub limited_beta_rad: f64,
    /// Body-axis attitude correction to add before the attitude loop.
    pub attitude_correction_body_rad: Vector3<f64>,
}

/// Estimate alpha/beta from inertial velocity, modeled wind, attitude, and density.
///
/// `q_body_from_inertial` rotates inertial-frame vectors into body axes.
///
/// # Errors
///
/// Returns a validation string when any input is non-finite or density is negative.
pub fn estimate_alpha_beta(
    v_inertial_m_s: Vector3<f64>,
    v_wind_eci_m_s: Vector3<f64>,
    q_body_from_inertial: UnitQuaternion<f64>,
    density_kg_m3: f64,
) -> Result<AlphaEstimate, String> {
    require_vector_finite("v_inertial_m_s", v_inertial_m_s)?;
    require_vector_finite("v_wind_eci_m_s", v_wind_eci_m_s)?;
    require_finite("density_kg_m3", density_kg_m3)?;
    if density_kg_m3 < 0.0 {
        return Err("density_kg_m3 must be non-negative".to_string());
    }
    let v_rel_inertial = v_inertial_m_s - v_wind_eci_m_s;
    let v_rel_body = q_body_from_inertial * v_rel_inertial;
    let qbar_pa = 0.5 * density_kg_m3 * v_rel_inertial.norm_squared();
    let forward = v_rel_body.x;
    let (alpha_rad, beta_rad) = if forward.abs() <= MIN_FORWARD_SPEED_M_S {
        (0.0, 0.0)
    } else {
        (v_rel_body.z.atan2(forward), v_rel_body.y.atan2(forward))
    };
    Ok(AlphaEstimate {
        alpha_rad,
        beta_rad,
        qbar_pa,
        air_relative_body_m_s: v_rel_body,
    })
}

/// Evaluate the blended load-relief command for one controller tick.
///
/// `lateral_accel_body_m_s2` is optional accelerometer feedback expressed in body axes. The
/// current autopilot insertion passes zero until a dedicated lateral-accelerometer topic lands.
///
/// # Errors
///
/// Returns a validation string when parameters or inputs are malformed.
#[allow(clippy::too_many_arguments)]
pub fn evaluate_load_relief(
    params: &LoadReliefParams,
    schedule_value: f64,
    v_inertial_m_s: Vector3<f64>,
    v_wind_eci_m_s: Vector3<f64>,
    q_body_from_inertial: UnitQuaternion<f64>,
    density_kg_m3: f64,
    lateral_accel_body_m_s2: Vector3<f64>,
) -> Result<LoadReliefCommand, String> {
    params.validate()?;
    require_vector_finite("lateral_accel_body_m_s2", lateral_accel_body_m_s2)?;
    let estimate = estimate_alpha_beta(
        v_inertial_m_s,
        v_wind_eci_m_s,
        q_body_from_inertial,
        density_kg_m3,
    )?;
    let blend = params.blend_at(schedule_value)?;
    let alpha_limit = if estimate.qbar_pa > 0.0 {
        params.q_alpha_limit_pa_rad / estimate.qbar_pa
    } else {
        f64::INFINITY
    };
    let limited_alpha_rad = estimate.alpha_rad.clamp(-alpha_limit, alpha_limit);
    let limited_beta_rad = estimate.beta_rad.clamp(-alpha_limit, alpha_limit);
    let alpha_gain = blend * params.g_alpha_max;
    let pitch_correction =
        -alpha_gain * limited_alpha_rad + params.g_acc * lateral_accel_body_m_s2.z;
    let yaw_correction = alpha_gain * limited_beta_rad - params.g_acc * lateral_accel_body_m_s2.y;
    Ok(LoadReliefCommand {
        estimate,
        blend,
        limited_alpha_rad,
        limited_beta_rad,
        attitude_correction_body_rad: Vector3::new(0.0, pitch_correction, yaw_correction),
    })
}

/// Offline-computed multi-station gyro blender.
#[derive(Clone, Debug, PartialEq)]
pub struct GyroBlender {
    weights: Vec<f64>,
}

impl GyroBlender {
    /// Construct a blender from normalized station weights.
    ///
    /// # Errors
    ///
    /// Returns a validation string when weights are empty, non-finite, negative, or do not sum to 1.
    pub fn new(weights: Vec<f64>) -> Result<Self, String> {
        if weights.is_empty() {
            return Err("gyro blender requires at least one station weight".to_string());
        }
        let mut sum = 0.0;
        for (idx, weight) in weights.iter().enumerate() {
            require_finite("gyro_blender.weight", *weight)?;
            if *weight < 0.0 {
                return Err(format!("gyro blender weight {idx} must be non-negative"));
            }
            sum += *weight;
        }
        if (sum - 1.0).abs() > BLEND_SUM_TOLERANCE {
            return Err("gyro blender weights must sum to 1".to_string());
        }
        Ok(Self { weights })
    }

    /// Number of station weights.
    #[must_use]
    pub fn len(&self) -> usize {
        self.weights.len()
    }

    /// Return `true` when no station weights are present.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.weights.is_empty()
    }

    /// Blend station rate measurements in fixed declaration order.
    ///
    /// # Errors
    ///
    /// Returns a validation string when the station count or a station value is invalid.
    pub fn blend(&self, station_rates_body_rad_s: &[Vector3<f64>]) -> Result<Vector3<f64>, String> {
        if station_rates_body_rad_s.len() != self.weights.len() {
            return Err("gyro blender station count does not match weights".to_string());
        }
        let mut out = Vector3::zeros();
        for (idx, (weight, rate)) in self
            .weights
            .iter()
            .zip(station_rates_body_rad_s)
            .enumerate()
        {
            require_vector_finite("gyro_blender.station_rate", *rate)
                .map_err(|reason| format!("station {idx}: {reason}"))?;
            out += *rate * *weight;
        }
        Ok(out)
    }
}

fn require_finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn require_positive_finite(name: &str, value: f64) -> Result<(), String> {
    require_finite(name, value)?;
    if value > 0.0 {
        Ok(())
    } else {
        Err(format!("{name} must be positive"))
    }
}

fn require_vector_finite(name: &str, value: Vector3<f64>) -> Result<(), String> {
    if value.x.is_finite() && value.y.is_finite() && value.z.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use toml::Value;

    fn fixture() -> Value {
        toml::from_str(include_str!(
            "../../../data/gnc/load-relief-alpha-beta-tolerance-v1.toml"
        ))
        .expect("load-relief fixture parses")
    }

    #[test]
    fn load_relief_alpha_beta_fixture_matches_geometry() {
        let value = fixture();
        let tolerances = value["tolerances"].as_table().expect("tolerances table");
        let alpha_tol = tolerances["alpha_abs_rad"]
            .as_float()
            .expect("alpha tolerance");
        let beta_tol = tolerances["beta_abs_rad"]
            .as_float()
            .expect("beta tolerance");
        let qbar_tol = tolerances["qbar_abs_pa"]
            .as_float()
            .expect("qbar tolerance");
        let cases = value["alpha_beta_cases"]
            .as_array()
            .expect("alpha_beta_cases array");
        for case in cases {
            let velocity = vec3(case, "velocity_eci_m_s");
            let wind = vec3(case, "wind_eci_m_s");
            let density = case["density_kg_m3"].as_float().expect("density");
            let estimate = estimate_alpha_beta(velocity, wind, UnitQuaternion::identity(), density)
                .expect("estimate alpha/beta");
            assert_abs_diff_eq!(
                estimate.alpha_rad,
                case["expected_alpha_rad"]
                    .as_float()
                    .expect("expected alpha"),
                epsilon = alpha_tol
            );
            assert_abs_diff_eq!(
                estimate.beta_rad,
                case["expected_beta_rad"].as_float().expect("expected beta"),
                epsilon = beta_tol
            );
            assert_abs_diff_eq!(
                estimate.qbar_pa,
                case["expected_qbar_pa"].as_float().expect("expected qbar"),
                epsilon = qbar_tol
            );
        }
    }

    #[test]
    fn load_relief_qalpha_limiter_reduces_peak_toy_gust() {
        let value = fixture();
        let scenario = &value["qalpha_limiter_case"];
        let params = fixture_params(scenario);
        let density = scenario["density_kg_m3"].as_float().expect("density");
        let speed = scenario["speed_m_s"].as_float().expect("speed");
        let limit = params.q_alpha_limit_pa_rad;
        let mut raw_peak = 0.0_f64;
        let mut limited_peak = 0.0_f64;
        for sample in scenario["alpha_samples_rad"]
            .as_array()
            .expect("alpha samples")
        {
            let alpha = sample.as_float().expect("alpha sample");
            let velocity = Vector3::new(speed, 0.0, speed * alpha.tan());
            let output = evaluate_load_relief(
                &params,
                1.0,
                velocity,
                Vector3::zeros(),
                UnitQuaternion::identity(),
                density,
                Vector3::zeros(),
            )
            .expect("load relief output");
            raw_peak = raw_peak.max((output.estimate.qbar_pa * output.estimate.alpha_rad).abs());
            limited_peak =
                limited_peak.max((output.estimate.qbar_pa * output.limited_alpha_rad).abs());
        }
        assert!(raw_peak > limit * 1.5, "raw toy gust must breach the limit");
        assert!(
            limited_peak <= limit + 1.0e-9,
            "limited q-alpha {limited_peak} should stay below {limit}"
        );
        assert!(limited_peak < raw_peak);
    }

    #[test]
    fn blend_schedule_interpolates_and_validates_target_free_params() {
        let params = LoadReliefParams {
            a0: 1.0,
            a1: 0.2,
            g_alpha_max: 0.5,
            g_acc: 0.0,
            blend_schedule: vec![(0.0, 0.0), (10.0, 1.0)],
            q_alpha_limit_pa_rad: 5_000.0,
        };
        assert_abs_diff_eq!(params.blend_at(5.0).unwrap(), 0.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(params.blend_at(-1.0).unwrap(), 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(params.blend_at(11.0).unwrap(), 1.0, epsilon = 1.0e-15);

        let bad = LoadReliefParams {
            blend_schedule: vec![(0.0, 0.0), (0.0, 1.0)],
            ..params
        };
        assert!(bad.validate().is_err());
    }

    #[test]
    fn gyro_blender_weights_stations_in_locked_order() {
        let blender = GyroBlender::new(vec![0.25, 0.75]).expect("valid blender");
        let out = blender
            .blend(&[Vector3::new(1.0, 10.0, -2.0), Vector3::new(3.0, 2.0, 6.0)])
            .expect("blend stations");
        assert_abs_diff_eq!(out.x, 2.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(out.y, 4.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(out.z, 4.0, epsilon = 1.0e-15);
        assert!(GyroBlender::new(vec![0.2, 0.2]).is_err());
    }

    fn fixture_params(value: &Value) -> LoadReliefParams {
        let params = &value["params"];
        let schedule = params["blend_schedule"]
            .as_array()
            .expect("blend schedule")
            .iter()
            .map(|entry| {
                let pair = entry.as_array().expect("blend pair");
                (
                    pair[0].as_float().expect("schedule value"),
                    pair[1].as_float().expect("blend value"),
                )
            })
            .collect();
        LoadReliefParams {
            a0: params["a0"].as_float().expect("a0"),
            a1: params["a1"].as_float().expect("a1"),
            g_alpha_max: params["g_alpha_max"].as_float().expect("g_alpha_max"),
            g_acc: params["g_acc"].as_float().expect("g_acc"),
            blend_schedule: schedule,
            q_alpha_limit_pa_rad: params["q_alpha_limit_pa_rad"]
                .as_float()
                .expect("q alpha limit"),
        }
    }

    fn vec3(value: &Value, key: &str) -> Vector3<f64> {
        let array = value[key].as_array().expect("vector array");
        Vector3::new(
            array[0].as_float().expect("x"),
            array[1].as_float().expect("y"),
            array[2].as_float().expect("z"),
        )
    }
}
