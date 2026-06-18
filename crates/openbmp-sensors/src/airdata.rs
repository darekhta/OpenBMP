//! Synthetic Pitot-static and angle-vane air-data sensor.
//!
//! The forward pressure model uses the compressible Pitot relation
//! for subsonic flow and the Rayleigh Pitot formula behind a normal
//! shock for supersonic flow. Angle vanes recover alpha and beta from
//! the body-frame air-relative velocity convention used by the aero
//! deck: `v_body = |V| [cos(alpha) cos(beta), sin(beta),
//! sin(alpha) cos(beta)]`.

use nalgebra::Vector3;
use openbmp_core::{DeterministicRng, Duration, SensorId, SimTime, StepIndex, ValidationStatus};
use serde::Deserialize;

use crate::error::SensorError;
use crate::noise::BoxMullerGaussian;
use crate::sensor::{SensorMeasurement, SensorTruth, SyntheticSensor, require_truth_finite};

const COMPONENT_ID_STATIC_PRESSURE_NOISE: u32 = 0;
const COMPONENT_ID_IMPACT_PRESSURE_NOISE: u32 = 1;
const COMPONENT_ID_ALPHA_NOISE: u32 = 2;
const COMPONENT_ID_BETA_NOISE: u32 = 3;

const GAMMA_AIR: f64 = 1.4;
const ISA_SEA_LEVEL_PRESSURE_PA: f64 = 101_325.0;
const ISA_SEA_LEVEL_SPEED_OF_SOUND_M_S: f64 = 340.294_101_758_449_9;
const ISA_SEA_LEVEL_TEMPERATURE_K: f64 = 288.15;
const ISA_TROPOSPHERE_LAPSE_K_M: f64 = 0.0065;
const ISA_G0_M_S2: f64 = 9.806_65;
const ISA_AIR_GAS_CONSTANT_J_KG_K: f64 = 287.052_87;
const SCHEMA_VERSION: u32 = 1;

/// Noise and timing budget for [`SyntheticAirData`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AirDataNoiseBudget {
    /// Gaussian stddev applied independently to static and impact
    /// pressure channels (Pa).
    pub pressure_stddev_pa: f64,
    /// Gaussian stddev applied independently to alpha and beta
    /// vane channels (rad).
    pub angle_stddev_rad: f64,
    /// Fixed measurement latency (s). The synthetic adapter reports
    /// `truth.time - fixed_latency_s` as the capture timestamp.
    pub fixed_latency_s: f64,
}

impl AirDataNoiseBudget {
    /// Construct after validating the parameters.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::NonFinite`] for NaN/Inf values and
    /// [`SensorError::InvalidParameter`] for negative stddevs or
    /// latency.
    pub fn new(
        pressure_stddev_pa: f64,
        angle_stddev_rad: f64,
        fixed_latency_s: f64,
    ) -> Result<Self, SensorError> {
        for value in [pressure_stddev_pa, angle_stddev_rad, fixed_latency_s] {
            if !value.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "air-data noise budget contains a NaN or infinite value",
                });
            }
        }
        if pressure_stddev_pa < 0.0 || angle_stddev_rad < 0.0 || fixed_latency_s < 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "air-data noise stddevs and latency must be non-negative",
            });
        }
        Ok(Self {
            pressure_stddev_pa,
            angle_stddev_rad,
            fixed_latency_s,
        })
    }

    /// Parse a Schema-1 air-data noise budget from TOML.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::MalformedBudget`] for structural parse
    /// failures and value-level errors from [`Self::new`] otherwise.
    pub fn load_from_str(s: &str) -> Result<Self, SensorError> {
        let parsed: AirDataBudgetFile =
            toml::from_str(s).map_err(|_e| SensorError::MalformedBudget {
                reason: "air-data noise-budget TOML did not parse against Schema-1",
            })?;
        if parsed.openbmp.airdata_noise_budget != SCHEMA_VERSION {
            return Err(SensorError::MalformedBudget {
                reason: "openbmp.airdata_noise_budget schema version is not 1",
            });
        }
        validate_metadata(
            &parsed.meta.name,
            &parsed.meta.provenance,
            parsed.meta.validation,
        )?;
        Self::new(
            parsed.budget.pressure_stddev_pa,
            parsed.budget.angle_stddev_rad,
            parsed.budget.fixed_latency_s,
        )
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct AirDataBudgetFile {
    openbmp: SchemaMarker,
    meta: MetaSection,
    budget: BudgetSection,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SchemaMarker {
    airdata_noise_budget: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MetaSection {
    name: String,
    provenance: String,
    validation: ValidationStatus,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct BudgetSection {
    pressure_stddev_pa: f64,
    angle_stddev_rad: f64,
    #[serde(default)]
    fixed_latency_s: f64,
}

fn validate_metadata(
    name: &str,
    provenance: &str,
    _validation: ValidationStatus,
) -> Result<(), SensorError> {
    if name.trim().is_empty() {
        return Err(SensorError::MalformedBudget {
            reason: "air-data noise-budget `meta.name` must not be blank",
        });
    }
    if provenance.trim().is_empty() {
        return Err(SensorError::MalformedBudget {
            reason: "air-data noise-budget `meta.provenance` must not be blank",
        });
    }
    Ok(())
}

/// Synthetic Pitot-static plus angle-vane sensor.
#[derive(Clone, Debug)]
pub struct SyntheticAirData {
    sensor_id: SensorId,
    budget: AirDataNoiseBudget,
    gauss: BoxMullerGaussian,
}

impl SyntheticAirData {
    /// Construct from a sensor id and fully validated budget.
    #[must_use]
    pub const fn new(sensor_id: SensorId, budget: AirDataNoiseBudget) -> Self {
        Self {
            sensor_id,
            budget,
            gauss: BoxMullerGaussian::new(),
        }
    }

    /// Read-only access to the budget.
    #[must_use]
    pub const fn budget(&self) -> &AirDataNoiseBudget {
        &self.budget
    }
}

impl SyntheticSensor for SyntheticAirData {
    fn sensor_id(&self) -> SensorId {
        self.sensor_id
    }

    fn measure(
        &mut self,
        truth: &SensorTruth,
        step: StepIndex,
        scenario_seed: u64,
    ) -> Result<SensorMeasurement, SensorError> {
        require_truth_finite(truth)?;
        validate_airdata_truth(truth)?;

        let true_airspeed_m_s = truth.air_relative_velocity_body_m_s.norm();
        let true_mach =
            if true_airspeed_m_s <= f64::EPSILON || truth.speed_of_sound_m_s <= f64::EPSILON {
                0.0
            } else {
                true_airspeed_m_s / truth.speed_of_sound_m_s
            };
        let impact_pressure_pa = pitot_impact_pressure_pa(truth.static_pressure_pa, true_mach)?;
        let (angle_of_attack_rad, sideslip_rad) =
            vane_angles_from_body_velocity(truth.air_relative_velocity_body_m_s);

        let static_pressure_pa = add_pressure_noise(
            truth.static_pressure_pa,
            self.budget.pressure_stddev_pa,
            scenario_seed,
            step,
            self.sensor_id,
            COMPONENT_ID_STATIC_PRESSURE_NOISE,
            &self.gauss,
        )?
        .max(f64::MIN_POSITIVE);
        let impact_pressure_pa = add_pressure_noise(
            impact_pressure_pa,
            self.budget.pressure_stddev_pa,
            scenario_seed,
            step,
            self.sensor_id,
            COMPONENT_ID_IMPACT_PRESSURE_NOISE,
            &self.gauss,
        )?;
        let mach = mach_from_pitot_impact_pressure(static_pressure_pa, impact_pressure_pa)?;
        let calibrated_airspeed_m_s = calibrated_airspeed_from_impact_pressure(impact_pressure_pa)?;
        let angle_of_attack_rad = add_angle_noise(
            angle_of_attack_rad,
            self.budget.angle_stddev_rad,
            scenario_seed,
            step,
            self.sensor_id,
            COMPONENT_ID_ALPHA_NOISE,
            &self.gauss,
        )?;
        let sideslip_rad = add_angle_noise(
            sideslip_rad,
            self.budget.angle_stddev_rad,
            scenario_seed,
            step,
            self.sensor_id,
            COMPONENT_ID_BETA_NOISE,
            &self.gauss,
        )?;

        Ok(SensorMeasurement::AirData {
            static_pressure_pa,
            impact_pressure_pa,
            mach,
            calibrated_airspeed_m_s,
            true_airspeed_m_s,
            angle_of_attack_rad,
            sideslip_rad,
            pressure_altitude_m: pressure_altitude_from_static_pressure(static_pressure_pa)?,
        })
    }

    fn capture_time(&self, truth_time: SimTime) -> SimTime {
        if self.budget.fixed_latency_s <= 0.0 {
            return truth_time;
        }
        let delayed_s = (truth_time - Duration::from_seconds(self.budget.fixed_latency_s))
            .as_seconds()
            .max(0.0);
        SimTime::from_seconds(delayed_s)
    }
}

/// Compute Pitot impact pressure from static pressure and Mach.
///
/// # Errors
///
/// Returns [`SensorError`] when inputs are non-finite, negative, or
/// when the supersonic Rayleigh relation leaves its physical domain.
pub fn pitot_impact_pressure_pa(static_pressure_pa: f64, mach: f64) -> Result<f64, SensorError> {
    if !static_pressure_pa.is_finite() || !mach.is_finite() {
        return Err(SensorError::NonFinite {
            reason: "Pitot pressure input is NaN or infinite",
        });
    }
    if static_pressure_pa < 0.0 || mach < 0.0 {
        return Err(SensorError::InvalidParameter {
            reason: "Pitot static pressure and Mach must be non-negative",
        });
    }
    if static_pressure_pa == 0.0 || mach == 0.0 {
        return Ok(0.0);
    }
    let total_over_static = if mach <= 1.0 {
        isentropic_total_over_static(mach)
    } else {
        rayleigh_pitot_total_over_static(mach)?
    };
    let impact = static_pressure_pa * (total_over_static - 1.0);
    if impact.is_finite() && impact >= 0.0 {
        Ok(impact)
    } else {
        Err(SensorError::NonFinite {
            reason: "Pitot impact pressure is non-finite",
        })
    }
}

/// Invert the compressible Pitot relation and recover Mach.
///
/// The bisection bound expands deterministically until the requested
/// pressure ratio is bracketed, then performs a fixed 96 iterations.
///
/// # Errors
///
/// Returns [`SensorError`] for invalid pressure inputs.
pub fn mach_from_pitot_impact_pressure(
    static_pressure_pa: f64,
    impact_pressure_pa: f64,
) -> Result<f64, SensorError> {
    if !static_pressure_pa.is_finite() || !impact_pressure_pa.is_finite() {
        return Err(SensorError::NonFinite {
            reason: "Pitot Mach inversion input is NaN or infinite",
        });
    }
    if static_pressure_pa <= 0.0 || impact_pressure_pa < 0.0 {
        return Err(SensorError::InvalidParameter {
            reason: "Pitot Mach inversion requires p_static > 0 and q_c >= 0",
        });
    }
    if impact_pressure_pa == 0.0 {
        return Ok(0.0);
    }

    let mut high = 1.0_f64;
    while pitot_impact_pressure_pa(static_pressure_pa, high)? < impact_pressure_pa {
        high *= 2.0;
        if high > 256.0 {
            return Err(SensorError::InvalidParameter {
                reason: "Pitot Mach inversion could not bracket the pressure ratio",
            });
        }
    }
    let mut low = 0.0_f64;
    for _ in 0..96 {
        let mid = 0.5 * (low + high);
        if pitot_impact_pressure_pa(static_pressure_pa, mid)? < impact_pressure_pa {
            low = mid;
        } else {
            high = mid;
        }
    }
    Ok(0.5 * (low + high))
}

/// Convert impact pressure to calibrated airspeed using the ISA
/// sea-level subsonic relation.
///
/// # Errors
///
/// Returns [`SensorError`] when impact pressure is invalid.
pub fn calibrated_airspeed_from_impact_pressure(
    impact_pressure_pa: f64,
) -> Result<f64, SensorError> {
    if !impact_pressure_pa.is_finite() {
        return Err(SensorError::NonFinite {
            reason: "calibrated-airspeed input is NaN or infinite",
        });
    }
    if impact_pressure_pa < 0.0 {
        return Err(SensorError::InvalidParameter {
            reason: "calibrated-airspeed impact pressure must be non-negative",
        });
    }
    if impact_pressure_pa == 0.0 {
        return Ok(0.0);
    }
    let term = (impact_pressure_pa / ISA_SEA_LEVEL_PRESSURE_PA + 1.0)
        .powf((GAMMA_AIR - 1.0) / GAMMA_AIR)
        - 1.0;
    Ok(ISA_SEA_LEVEL_SPEED_OF_SOUND_M_S * (2.0 * term / (GAMMA_AIR - 1.0)).sqrt())
}

/// Convert static pressure to ISA tropospheric pressure altitude.
///
/// # Errors
///
/// Returns [`SensorError`] when static pressure is invalid.
pub fn pressure_altitude_from_static_pressure(static_pressure_pa: f64) -> Result<f64, SensorError> {
    if !static_pressure_pa.is_finite() {
        return Err(SensorError::NonFinite {
            reason: "pressure-altitude input is NaN or infinite",
        });
    }
    if static_pressure_pa <= 0.0 {
        return Err(SensorError::InvalidParameter {
            reason: "pressure-altitude static pressure must be positive",
        });
    }
    let exponent = ISA_AIR_GAS_CONSTANT_J_KG_K * ISA_TROPOSPHERE_LAPSE_K_M / ISA_G0_M_S2;
    Ok(ISA_SEA_LEVEL_TEMPERATURE_K / ISA_TROPOSPHERE_LAPSE_K_M
        * (1.0 - (static_pressure_pa / ISA_SEA_LEVEL_PRESSURE_PA).powf(exponent)))
}

/// Recover vane angles from body-frame air-relative velocity.
#[must_use]
pub fn vane_angles_from_body_velocity(velocity_body_m_s: Vector3<f64>) -> (f64, f64) {
    let speed = velocity_body_m_s.norm();
    if speed <= f64::EPSILON {
        return (0.0, 0.0);
    }
    let alpha = velocity_body_m_s.z.atan2(velocity_body_m_s.x);
    let beta = (velocity_body_m_s.y / speed).clamp(-1.0, 1.0).asin();
    (alpha, beta)
}

fn isentropic_total_over_static(mach: f64) -> f64 {
    (1.0 + 0.5 * (GAMMA_AIR - 1.0) * mach * mach).powf(GAMMA_AIR / (GAMMA_AIR - 1.0))
}

fn rayleigh_pitot_total_over_static(mach: f64) -> Result<f64, SensorError> {
    let mach2 = mach * mach;
    let numerator = (GAMMA_AIR + 1.0) * (GAMMA_AIR + 1.0) * mach2;
    let denominator = 4.0 * GAMMA_AIR * mach2 - 2.0 * (GAMMA_AIR - 1.0);
    let shock_static_ratio = (1.0 - GAMMA_AIR + 2.0 * GAMMA_AIR * mach2) / (GAMMA_AIR + 1.0);
    if denominator <= 0.0 || shock_static_ratio <= 0.0 {
        return Err(SensorError::InvalidParameter {
            reason: "Rayleigh Pitot relation left its physical domain",
        });
    }
    Ok((numerator / denominator).powf(GAMMA_AIR / (GAMMA_AIR - 1.0)) * shock_static_ratio)
}

fn validate_airdata_truth(truth: &SensorTruth) -> Result<(), SensorError> {
    if truth.static_pressure_pa < 0.0
        || truth.atmosphere_density_kg_m3 < 0.0
        || truth.speed_of_sound_m_s < 0.0
    {
        return Err(SensorError::InvalidParameter {
            reason: "air-data truth pressure, density, and speed of sound must be non-negative",
        });
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_pressure_noise(
    value: f64,
    stddev: f64,
    scenario_seed: u64,
    step: StepIndex,
    sensor_id: SensorId,
    component_id: u32,
    gauss: &BoxMullerGaussian,
) -> Result<f64, SensorError> {
    let mut rng =
        DeterministicRng::for_sensor_component(scenario_seed, step, sensor_id, component_id);
    let noisy = value + gauss.sample(&mut rng, 0.0, stddev)?;
    Ok(noisy.max(0.0))
}

#[allow(clippy::too_many_arguments)]
fn add_angle_noise(
    value: f64,
    stddev: f64,
    scenario_seed: u64,
    step: StepIndex,
    sensor_id: SensorId,
    component_id: u32,
    gauss: &BoxMullerGaussian,
) -> Result<f64, SensorError> {
    let mut rng =
        DeterministicRng::for_sensor_component(scenario_seed, step, sensor_id, component_id);
    gauss.sample(&mut rng, value, stddev)
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]
mod tests {
    use approx::assert_abs_diff_eq;
    use nalgebra::{UnitQuaternion, Vector3};
    use openbmp_core::{Position3, SimTime, Velocity3};

    use super::*;
    use crate::sensor::{Sensor, SyntheticSensorAdapter, Timestamped};

    fn budget_toml() -> &'static str {
        r#"
[openbmp]
airdata_noise_budget = 1

[meta]
name = "airdata-test"
provenance = "unit test"
validation = "validated-toy"

[budget]
pressure_stddev_pa = 0.0
angle_stddev_rad = 0.0
fixed_latency_s = 0.25
"#
    }

    fn truth_at(time_s: f64, velocity_body_m_s: Vector3<f64>) -> SensorTruth {
        SensorTruth {
            position_eci: Position3::new(0.0, 0.0, 0.0),
            velocity_eci: Velocity3::new(
                velocity_body_m_s.x,
                velocity_body_m_s.y,
                velocity_body_m_s.z,
            ),
            attitude_eci_to_body: UnitQuaternion::identity(),
            angular_velocity_body_rad_s: Vector3::zeros(),
            angular_acceleration_body_rad_s2: Vector3::zeros(),
            specific_force_body_m_s2: Vector3::zeros(),
            static_pressure_pa: ISA_SEA_LEVEL_PRESSURE_PA,
            atmosphere_density_kg_m3: 1.225,
            speed_of_sound_m_s: ISA_SEA_LEVEL_SPEED_OF_SOUND_M_S,
            air_relative_velocity_body_m_s: velocity_body_m_s,
            altitude_geometric_m: 0.0,
            magnetic_field_body_nt: Vector3::zeros(),
            time: SimTime::from_seconds(time_s),
        }
    }

    #[test]
    fn subsonic_pitot_pressure_matches_closed_form() {
        let mach = 0.5_f64;
        let expected = ISA_SEA_LEVEL_PRESSURE_PA * ((1.0 + 0.2 * mach * mach).powf(3.5) - 1.0);

        let actual = pitot_impact_pressure_pa(ISA_SEA_LEVEL_PRESSURE_PA, mach).unwrap();

        assert_abs_diff_eq!(actual, expected, epsilon = 1.0e-10);
    }

    #[test]
    fn supersonic_pitot_pressure_inverts_to_original_mach() {
        let mach = 2.0;
        let qc = pitot_impact_pressure_pa(70_000.0, mach).unwrap();

        let recovered = mach_from_pitot_impact_pressure(70_000.0, qc).unwrap();

        assert_abs_diff_eq!(recovered, mach, epsilon = 1.0e-12);
    }

    #[test]
    fn calibrated_airspeed_uses_sea_level_reference() {
        let mach = 0.3;
        let qc = pitot_impact_pressure_pa(ISA_SEA_LEVEL_PRESSURE_PA, mach).unwrap();
        let cas = calibrated_airspeed_from_impact_pressure(qc).unwrap();

        assert_abs_diff_eq!(
            cas,
            mach * ISA_SEA_LEVEL_SPEED_OF_SOUND_M_S,
            epsilon = 1.0e-10
        );
    }

    #[test]
    fn pressure_altitude_is_zero_at_isa_sea_level() {
        let altitude = pressure_altitude_from_static_pressure(ISA_SEA_LEVEL_PRESSURE_PA).unwrap();

        assert_abs_diff_eq!(altitude, 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn vanes_recover_alpha_beta_from_body_velocity() {
        let alpha = 8.0_f64.to_radians();
        let beta = -3.0_f64.to_radians();
        let speed = 250.0;
        let velocity = Vector3::new(
            speed * alpha.cos() * beta.cos(),
            speed * beta.sin(),
            speed * alpha.sin() * beta.cos(),
        );

        let (actual_alpha, actual_beta) = vane_angles_from_body_velocity(velocity);

        assert_abs_diff_eq!(actual_alpha, alpha, epsilon = 1.0e-14);
        assert_abs_diff_eq!(actual_beta, beta, epsilon = 1.0e-14);
    }

    #[test]
    fn zero_noise_measurement_reports_expected_airdata() {
        let budget = AirDataNoiseBudget::new(0.0, 0.0, 0.0).unwrap();
        let mut sensor = SyntheticAirData::new(SensorId::from_path("sensors.airdata"), budget);
        let truth = truth_at(
            3.0,
            Vector3::new(0.5 * ISA_SEA_LEVEL_SPEED_OF_SOUND_M_S, 0.0, 0.0),
        );

        let measurement = sensor.measure(&truth, StepIndex::new(0), 0).unwrap();

        let SensorMeasurement::AirData {
            static_pressure_pa,
            impact_pressure_pa,
            mach,
            calibrated_airspeed_m_s,
            true_airspeed_m_s,
            angle_of_attack_rad,
            sideslip_rad,
            pressure_altitude_m,
        } = measurement
        else {
            panic!("expected AirData variant");
        };
        assert_eq!(
            static_pressure_pa.to_bits(),
            truth.static_pressure_pa.to_bits()
        );
        assert_abs_diff_eq!(
            impact_pressure_pa,
            pitot_impact_pressure_pa(truth.static_pressure_pa, 0.5).unwrap(),
            epsilon = 1.0e-10
        );
        assert_abs_diff_eq!(mach, 0.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(
            calibrated_airspeed_m_s,
            true_airspeed_m_s,
            epsilon = 1.0e-10
        );
        assert_eq!(angle_of_attack_rad.to_bits(), 0.0_f64.to_bits());
        assert_eq!(sideslip_rad.to_bits(), 0.0_f64.to_bits());
        assert_abs_diff_eq!(pressure_altitude_m, 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn budget_parser_rejects_wrong_schema_version() {
        let text = budget_toml().replace("airdata_noise_budget = 1", "airdata_noise_budget = 2");

        assert!(matches!(
            AirDataNoiseBudget::load_from_str(&text),
            Err(SensorError::MalformedBudget { .. })
        ));
    }

    #[test]
    fn budget_rejects_negative_latency() {
        assert!(matches!(
            AirDataNoiseBudget::new(0.0, 0.0, -1.0),
            Err(SensorError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn fixed_latency_reports_delayed_timestamp_on_adapter() {
        let budget = AirDataNoiseBudget::load_from_str(budget_toml()).unwrap();
        let sensor = SyntheticAirData::new(SensorId::from_path("sensors.airdata"), budget);
        let truth = truth_at(10.0, Vector3::new(100.0, 0.0, 0.0));
        let mut adapter = SyntheticSensorAdapter::new(sensor);

        adapter.prime(truth, StepIndex::new(5), 0x1234);
        let Timestamped { time, value } = adapter.read().unwrap();

        assert_eq!(time, SimTime::from_seconds(9.75));
        assert!(matches!(value, SensorMeasurement::AirData { .. }));
    }
}
