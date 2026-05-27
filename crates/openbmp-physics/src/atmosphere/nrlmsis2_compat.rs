//! NRLMSIS 2.x compatibility atmosphere profile.
//!
//! This module provides an OpenBMP-local compatibility profile for
//! scenarios that need an NRLMSIS-2-facing atmosphere selector without
//! importing the official NRLMSIS 2.x source or coefficient packages.
//! The baseline is the in-repository [`Nrlmsise00Full`] evaluator. A
//! deterministic upper-atmosphere correction and nitric-oxide proxy are
//! layered on top so the model responds to the same date, location,
//! F10.7, and Ap inputs exposed by the MSIS family.
//!
//! This is not the official NRLMSIS 2.0 or 2.1 distribution. The
//! official packages remain excluded from this repository because the
//! inspected terms are not compatible with direct vendoring here.

use openbmp_core::SimTime;

use super::{AtmosphereModel, AtmosphereSample};
use crate::atmosphere::nrlmsise00::{Nrlmsise00Full, Nrlmsise00Inputs, Nrlmsise00Outputs};
use crate::error::PhysicsError;

const BOLTZMANN_J_K: f64 = 1.380_649e-23;
const COMPAT_GAMMA: f64 = 1.4;
const ATOMIC_MASS_UNIT_KG: f64 = 1.660_539_066_60e-27;
const NO_MOLECULAR_MASS_KG: f64 = 30.0061 * ATOMIC_MASS_UNIT_KG;

/// NRLMSIS 2.x compatibility outputs.
///
/// `base` preserves the direct NRLMSISE-00 coefficient output. The
/// remaining fields are OpenBMP compatibility additions used by
/// [`Nrlmsis2Compat`] when producing an [`AtmosphereSample`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Nrlmsis2CompatOutputs {
    /// Underlying NRLMSISE-00 output before compatibility corrections.
    pub base: Nrlmsise00Outputs,
    /// Nitric-oxide number density proxy (1/m^3).
    pub n_no: f64,
    /// Corrected total neutral mass density (kg/m^3).
    pub mass_density_kg_m3: f64,
    /// Corrected neutral temperature at altitude (K).
    pub neutral_temperature_k: f64,
    /// Corrected exospheric temperature (K).
    pub exospheric_temperature_k: f64,
    /// Multiplicative density correction applied to the NRLMSISE-00
    /// baseline before the nitric-oxide mass contribution is added.
    pub density_scale: f64,
    /// Additive neutral-temperature correction (K).
    pub temperature_delta_k: f64,
}

impl Nrlmsis2CompatOutputs {
    /// Convert corrected compatibility outputs to the shared
    /// [`AtmosphereSample`] surface.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::NonFinite`] for non-finite fields and
    /// [`PhysicsError::InvalidParameter`] for physically invalid output
    /// components.
    pub fn to_sample(self) -> Result<AtmosphereSample, PhysicsError> {
        self.validate()?;
        let total_number_density = total_neutral_number_density(self.base) + self.n_no;
        let pressure = total_number_density * BOLTZMANN_J_K * self.neutral_temperature_k;
        let speed_of_sound =
            (COMPAT_GAMMA * pressure / self.mass_density_kg_m3.max(f64::MIN_POSITIVE)).sqrt();
        AtmosphereSample::new(
            self.mass_density_kg_m3,
            pressure,
            self.neutral_temperature_k,
            speed_of_sound,
        )
    }

    fn validate(self) -> Result<(), PhysicsError> {
        let values = [
            self.n_no,
            self.mass_density_kg_m3,
            self.neutral_temperature_k,
            self.exospheric_temperature_k,
            self.density_scale,
            self.temperature_delta_k,
        ];
        if values.iter().any(|value| !value.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "nrlmsis2_compat output field is NaN or Inf",
            });
        }
        if self.n_no < 0.0 || self.density_scale <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "nrlmsis2_compat number densities and scale must be non-negative",
            });
        }
        if self.mass_density_kg_m3 <= 0.0
            || self.neutral_temperature_k <= 0.0
            || self.exospheric_temperature_k <= 0.0
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "nrlmsis2_compat density and temperatures must be positive",
            });
        }
        Ok(())
    }
}

/// NRLMSIS 2.x compatibility atmosphere profile.
///
/// The model keeps the NRLMSISE-00 full coefficient path as its
/// deterministic baseline and applies a bounded upper-atmosphere
/// correction plus a nitric-oxide proxy. Use this when a scenario needs
/// an NRLMSIS-2-family selector but cannot import the official 2.x
/// coefficient packages.
#[derive(Copy, Clone, Debug)]
pub struct Nrlmsis2Compat {
    base: Nrlmsise00Full,
}

impl Default for Nrlmsis2Compat {
    fn default() -> Self {
        Self::mid_conditions()
    }
}

impl Nrlmsis2Compat {
    /// Construct with mid-condition defaults for all non-altitude
    /// inputs.
    #[must_use]
    pub const fn mid_conditions() -> Self {
        Self {
            base: Nrlmsise00Full::mid_conditions(),
        }
    }

    /// Construct from scenario-provided default inputs. The altitude
    /// in `base_inputs` is ignored by [`AtmosphereModel::sample`] and
    /// replaced by the sampled altitude.
    ///
    /// # Errors
    ///
    /// Returns the same validation errors as
    /// [`Nrlmsise00Inputs::validate_full_path`].
    pub fn new(base_inputs: Nrlmsise00Inputs) -> Result<Self, PhysicsError> {
        Ok(Self {
            base: Nrlmsise00Full::new(base_inputs)?,
        })
    }

    /// Scenario/default inputs used by the trait-based sampler.
    #[must_use]
    pub const fn base_inputs(self) -> Nrlmsise00Inputs {
        self.base.base_inputs()
    }

    /// Evaluate the compatibility profile at a full MSIS query.
    ///
    /// # Errors
    ///
    /// Returns input-validation errors from the NRLMSISE-00 baseline
    /// and output-validation errors if the compatibility corrections
    /// produce non-finite or physically invalid values.
    pub fn evaluate(self, inputs: Nrlmsise00Inputs) -> Result<Nrlmsis2CompatOutputs, PhysicsError> {
        let base = self.base.evaluate(inputs)?;
        let density_scale = density_scale(inputs);
        let temperature_delta_k = temperature_delta_k(inputs);
        let n_no = no_number_density(inputs, base);
        let mass_density_kg_m3 =
            base.mass_density_kg_m3 * density_scale + n_no * NO_MOLECULAR_MASS_KG;
        let neutral_temperature_k = (base.neutral_temperature_k + temperature_delta_k).max(1.0);
        let exospheric_temperature_k =
            (base.exospheric_temperature_k + 0.55 * temperature_delta_k).max(neutral_temperature_k);
        let outputs = Nrlmsis2CompatOutputs {
            base,
            n_no,
            mass_density_kg_m3,
            neutral_temperature_k,
            exospheric_temperature_k,
            density_scale,
            temperature_delta_k,
        };
        outputs.validate()?;
        Ok(outputs)
    }
}

impl AtmosphereModel for Nrlmsis2Compat {
    fn sample(
        &self,
        altitude_geometric_m: f64,
        _time: SimTime,
    ) -> Result<AtmosphereSample, PhysicsError> {
        let inputs = self.base_inputs().with_altitude_m(altitude_geometric_m);
        self.evaluate(inputs)?.to_sample()
    }
}

fn density_scale(inputs: Nrlmsise00Inputs) -> f64 {
    let altitude_weight = smoothstep(80_000.0, 170_000.0, inputs.altitude_m);
    let driver = 0.060 * solar_driver(inputs)
        + 0.040 * ap_driver(inputs)
        + 0.018 * latitude_driver(inputs) * (1.0 + 0.25 * seasonal_driver(inputs))
        + 0.018 * local_time_driver(inputs);
    (altitude_weight * driver).clamp(-0.18, 0.22).exp()
}

fn temperature_delta_k(inputs: Nrlmsise00Inputs) -> f64 {
    let altitude_weight = smoothstep(90_000.0, 190_000.0, inputs.altitude_m);
    let driver = 42.0 * solar_driver(inputs)
        + 24.0 * ap_driver(inputs)
        + 7.5 * latitude_driver(inputs) * seasonal_driver(inputs)
        + 8.0 * local_time_driver(inputs);
    (altitude_weight * driver).clamp(-65.0, 95.0)
}

fn no_number_density(inputs: Nrlmsise00Inputs, base: Nrlmsise00Outputs) -> f64 {
    let lower = smoothstep(70_000.0, 95_000.0, inputs.altitude_m);
    let upper = 1.0 - smoothstep(260_000.0, 360_000.0, inputs.altitude_m);
    let altitude_weight = lower * upper;
    if altitude_weight <= 0.0 {
        return 0.0;
    }
    let altitude_km = inputs.altitude_m / 1000.0;
    let main_peak = gaussian(altitude_km, 112.0, 24.0);
    let shoulder = 0.28 * gaussian(altitude_km, 170.0, 58.0);
    let activity = (1.0 + 0.85 * solar_driver(inputs) + 0.45 * ap_driver(inputs)).clamp(0.20, 2.35);
    let geometry = 0.72 + 0.28 * latitude_driver(inputs);
    let day_wave = 0.85 + 0.15 * seasonal_driver(inputs);
    let fraction =
        (9.0e-4 * altitude_weight * (main_peak + shoulder) * activity * geometry * day_wave)
            .clamp(0.0, 4.5e-3);
    total_neutral_number_density(base) * fraction
}

fn total_neutral_number_density(outputs: Nrlmsise00Outputs) -> f64 {
    outputs.n_he
        + outputs.n_o
        + outputs.n_n2
        + outputs.n_o2
        + outputs.n_ar
        + outputs.n_h
        + outputs.n_n
}

fn solar_driver(inputs: Nrlmsise00Inputs) -> f64 {
    let mean_f107 = 0.5 * (inputs.f107_average_81day + inputs.f107_yesterday);
    ((mean_f107 - 150.0) / 150.0).clamp(-0.60, 1.15)
}

fn ap_driver(inputs: Nrlmsise00Inputs) -> f64 {
    let quiet = (1.0_f64 + 4.0).ln();
    let active = (1.0_f64 + 100.0).ln();
    (((1.0 + inputs.ap_average).ln() - quiet) / (active - quiet)).clamp(-0.45, 1.20)
}

fn local_time_driver(inputs: Nrlmsise00Inputs) -> f64 {
    let phase = std::f64::consts::TAU * (inputs.local_apparent_solar_time_hours - 14.0) / 24.0;
    let baseline = std::f64::consts::TAU * (12.0 - 14.0) / 24.0;
    phase.cos() - baseline.cos()
}

fn seasonal_driver(inputs: Nrlmsise00Inputs) -> f64 {
    let phase = std::f64::consts::TAU * (f64::from(inputs.day_of_year) - 80.0) / 365.25;
    phase.sin()
}

fn latitude_driver(inputs: Nrlmsise00Inputs) -> f64 {
    inputs.latitude_rad.sin().abs()
}

fn smoothstep(edge0: f64, edge1: f64, x: f64) -> f64 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn gaussian(x: f64, mean: f64, sigma: f64) -> f64 {
    let z = (x - mean) / sigma;
    (-0.5 * z * z).exp()
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::missing_panics_doc
)]
mod tests {
    use super::*;

    #[test]
    fn compat_matches_baseline_below_blend_region() {
        let inputs = Nrlmsise00Inputs::mid_conditions(50_000.0);
        let base = Nrlmsise00Full::default().evaluate(inputs).unwrap();
        let compat = Nrlmsis2Compat::default().evaluate(inputs).unwrap();

        assert_eq!(compat.n_no, 0.0);
        assert_eq!(compat.density_scale, 1.0);
        assert_eq!(
            compat.mass_density_kg_m3.to_bits(),
            base.mass_density_kg_m3.to_bits()
        );
        assert_eq!(
            compat.neutral_temperature_k.to_bits(),
            base.neutral_temperature_k.to_bits()
        );
    }

    #[test]
    fn compat_exposes_no_proxy_near_lower_thermosphere() {
        let model = Nrlmsis2Compat::default();
        let outputs = model
            .evaluate(Nrlmsise00Inputs::mid_conditions(115_000.0))
            .unwrap();
        let sample = outputs.to_sample().unwrap();

        assert!(outputs.n_no > 0.0);
        assert!(outputs.mass_density_kg_m3 > outputs.base.mass_density_kg_m3);
        assert!(sample.pressure_pa > 0.0);
    }

    #[test]
    fn compat_activity_increases_adjusted_density() {
        let model = Nrlmsis2Compat::default();
        let mut quiet = Nrlmsise00Inputs::mid_conditions(220_000.0);
        quiet.f107_average_81day = 70.0;
        quiet.f107_yesterday = 70.0;
        quiet.ap_average = 2.0;

        let mut active = quiet;
        active.f107_average_81day = 250.0;
        active.f107_yesterday = 250.0;
        active.ap_average = 80.0;

        let quiet_outputs = model.evaluate(quiet).unwrap();
        let active_outputs = model.evaluate(active).unwrap();

        assert!(active_outputs.density_scale > quiet_outputs.density_scale);
        assert!(active_outputs.mass_density_kg_m3 > quiet_outputs.mass_density_kg_m3);
    }

    #[test]
    fn sample_returns_finite_atmosphere_at_orbital_altitudes() {
        let model = Nrlmsis2Compat::default();
        for altitude_m in [100_000.0_f64, 200_000.0, 400_000.0, 800_000.0] {
            let sample = model.sample(altitude_m, SimTime::ZERO).unwrap();
            assert!(sample.density_kg_m3 > 0.0);
            assert!(sample.pressure_pa > 0.0);
            assert!(sample.temperature_k > 0.0);
            assert!(sample.speed_of_sound_m_s > 0.0);
        }
    }
}
