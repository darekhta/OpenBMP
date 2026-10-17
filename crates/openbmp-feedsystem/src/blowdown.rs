//! Isentropic tank blowdown evidence primitive.

use openbmp_core::ValidationStatus;

use crate::error::FeedSystemError;

/// Constant-geometry, choked-orifice tank blowdown configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IsentropicTankBlowdownConfig {
    /// Tank free volume in m^3.
    pub tank_volume_m3: f64,
    /// Initial tank pressure in Pa.
    pub initial_pressure_pa: f64,
    /// Initial tank temperature in K.
    pub initial_temperature_k: f64,
    /// Downstream/static sink pressure in Pa.
    pub downstream_pressure_pa: f64,
    /// Gas ratio of specific heats.
    pub gas_gamma: f64,
    /// Gas constant in J/(kg*K).
    pub gas_constant_j_per_kg_k: f64,
    /// Orifice flow area in m^2.
    pub orifice_area_m2: f64,
    /// Discharge coefficient applied to the choked orifice area.
    pub discharge_coefficient: f64,
}

impl IsentropicTankBlowdownConfig {
    /// Validate tank and gas parameters.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite, non-positive, or
    /// out-of-envelope blowdown parameters.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_positive_finite(self.tank_volume_m3, "blowdown tank volume must be positive")?;
        require_positive_finite(
            self.initial_pressure_pa,
            "blowdown initial pressure must be positive",
        )?;
        require_positive_finite(
            self.initial_temperature_k,
            "blowdown initial temperature must be positive",
        )?;
        require_nonnegative_finite(
            self.downstream_pressure_pa,
            "blowdown downstream pressure must be finite and non-negative",
        )?;
        require_positive_finite(self.gas_gamma, "blowdown gas gamma must be positive")?;
        if self.gas_gamma <= 1.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "blowdown gas gamma must exceed 1",
            });
        }
        require_positive_finite(
            self.gas_constant_j_per_kg_k,
            "blowdown gas constant must be positive",
        )?;
        require_positive_finite(
            self.orifice_area_m2,
            "blowdown orifice area must be positive",
        )?;
        require_positive_finite(
            self.discharge_coefficient,
            "blowdown discharge coefficient must be positive",
        )?;
        if self.discharge_coefficient > 1.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "blowdown discharge coefficient must not exceed 1",
            });
        }
        if self.downstream_pressure_pa >= self.initial_pressure_pa {
            return Err(FeedSystemError::InvalidParameter {
                reason: "blowdown downstream pressure must be below initial pressure",
            });
        }
        Ok(())
    }

    fn initial_density_kg_m3(self) -> f64 {
        self.initial_pressure_pa / (self.gas_constant_j_per_kg_k * self.initial_temperature_k)
    }

    fn initial_mass_kg(self) -> f64 {
        self.initial_density_kg_m3() * self.tank_volume_m3
    }

    fn critical_flow_function(self) -> f64 {
        let gamma = self.gas_gamma;
        (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (2.0 * (gamma - 1.0)))
    }

    fn critical_pressure_ratio(self) -> f64 {
        let gamma = self.gas_gamma;
        (2.0 / (gamma + 1.0)).powf(gamma / (gamma - 1.0))
    }

    fn decay_coefficient_per_s(self) -> f64 {
        0.5 * (self.gas_gamma - 1.0) * self.discharge_coefficient * self.orifice_area_m2
            / self.tank_volume_m3
            * (self.gas_gamma * self.gas_constant_j_per_kg_k * self.initial_temperature_k).sqrt()
            * self.critical_flow_function()
    }
}

/// State sampled from the closed-form choked blowdown solution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IsentropicTankBlowdownSample {
    /// Sample time in seconds from the initial state.
    pub time_s: f64,
    /// Tank pressure in Pa.
    pub pressure_pa: f64,
    /// Pressure normalized by the initial pressure.
    pub pressure_ratio: f64,
    /// Tank temperature in K.
    pub temperature_k: f64,
    /// Gas mass remaining in the tank in kg.
    pub mass_kg: f64,
    /// Choked-orifice outflow at this sample in kg/s.
    pub mass_flow_kg_per_s: f64,
    /// Validation posture of this analytic blowdown primitive.
    pub validation: ValidationStatus,
}

/// Closed-form isentropic tank blowdown sampler.
///
/// The solution is the Moody choked-blowdown expression used by GFSSP Version 6
/// Example 8 for comparison to GFSSP's unsteady tank model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IsentropicTankBlowdown {
    config: IsentropicTankBlowdownConfig,
}

impl IsentropicTankBlowdown {
    /// Construct a validated blowdown sampler.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the configuration is invalid.
    pub fn new(config: IsentropicTankBlowdownConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self { config })
    }

    /// Borrow the validated configuration.
    #[must_use]
    pub const fn config(&self) -> IsentropicTankBlowdownConfig {
        self.config
    }

    /// Sample the choked isentropic blowdown solution at `time_s`.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid time values, non-finite
    /// intermediate values, or samples outside the choked-flow pressure-ratio
    /// envelope.
    pub fn sample_at(&self, time_s: f64) -> Result<IsentropicTankBlowdownSample, FeedSystemError> {
        require_nonnegative_finite(
            time_s,
            "blowdown sample time must be finite and non-negative",
        )?;

        let gamma = self.config.gas_gamma;
        let argument = 1.0 + self.config.decay_coefficient_per_s() * time_s;
        if !argument.is_finite() || argument <= 0.0 {
            return Err(FeedSystemError::NonFinite {
                reason: "blowdown pressure-ratio argument is non-finite",
            });
        }
        let pressure_ratio = argument.powf(-2.0 * gamma / (gamma - 1.0));
        let pressure_pa = self.config.initial_pressure_pa * pressure_ratio;
        let downstream_ratio = self.config.downstream_pressure_pa / pressure_pa;
        if downstream_ratio >= self.config.critical_pressure_ratio() {
            return Err(FeedSystemError::InvalidParameter {
                reason: "blowdown sample is outside the choked-flow envelope",
            });
        }
        let temperature_ratio = pressure_ratio.powf((gamma - 1.0) / gamma);
        let temperature_k = self.config.initial_temperature_k * temperature_ratio;
        let mass_kg = self.config.initial_mass_kg() * pressure_ratio.powf(1.0 / gamma);
        let mass_flow_kg_per_s = self.config.discharge_coefficient
            * self.config.orifice_area_m2
            * pressure_pa
            * (gamma / (self.config.gas_constant_j_per_kg_k * temperature_k)).sqrt()
            * self.config.critical_flow_function();

        if !pressure_ratio.is_finite()
            || !pressure_pa.is_finite()
            || !temperature_k.is_finite()
            || !mass_kg.is_finite()
            || !mass_flow_kg_per_s.is_finite()
        {
            return Err(FeedSystemError::NonFinite {
                reason: "blowdown sample contains a non-finite value",
            });
        }

        Ok(IsentropicTankBlowdownSample {
            time_s,
            pressure_pa,
            pressure_ratio,
            temperature_k,
            mass_kg,
            mass_flow_kg_per_s,
            validation: ValidationStatus::Research,
        })
    }
}

fn require_positive_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    if !value.is_finite() {
        return Err(FeedSystemError::NonFinite { reason });
    }
    if value <= 0.0 {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

fn require_nonnegative_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    if !value.is_finite() {
        return Err(FeedSystemError::NonFinite { reason });
    }
    if value < 0.0 {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use approx::assert_abs_diff_eq;
    use toml::value::Table;

    use super::*;

    fn config() -> IsentropicTankBlowdownConfig {
        IsentropicTankBlowdownConfig {
            tank_volume_m3: 0.28316846592,
            initial_pressure_pa: 689_475.729_316_8,
            initial_temperature_k: 299.816_666_666_666_66,
            downstream_pressure_pa: 101_352.932_209_569_6,
            gas_gamma: 1.4,
            gas_constant_j_per_kg_k: 287.058,
            orifice_area_m2: 5.067_074_790_974_978e-6,
            discharge_coefficient: 1.0,
        }
    }

    #[test]
    fn isentropic_tank_blowdown_decays_pressure_and_mass() {
        let blowdown = IsentropicTankBlowdown::new(config()).unwrap();

        let start = blowdown.sample_at(0.0).unwrap();
        let later = blowdown.sample_at(100.0).unwrap();

        assert_abs_diff_eq!(start.pressure_ratio, 1.0, epsilon = 1.0e-15);
        assert!(later.pressure_pa < start.pressure_pa);
        assert!(later.temperature_k < start.temperature_k);
        assert!(later.mass_kg < start.mass_kg);
        assert!(later.mass_flow_kg_per_s < start.mass_flow_kg_per_s);
        assert_eq!(later.validation, ValidationStatus::Research);
    }

    #[test]
    fn isentropic_tank_blowdown_matches_gfssp_example8_moody_solution() {
        let data: toml::Value = toml::from_str(include_str!(
            "../../../data/feed_system/gfssp-tank-blowdown-example8-v1.toml"
        ))
        .unwrap();
        assert_eq!(
            table("openbmp", &data)["feed_system_gfssp_tank_blowdown_example8"]
                .as_integer()
                .unwrap(),
            1
        );
        let config_table = table("config", &data);
        let blowdown = IsentropicTankBlowdown::new(IsentropicTankBlowdownConfig {
            tank_volume_m3: float(config_table, "tank_volume_m3"),
            initial_pressure_pa: float(config_table, "initial_pressure_pa"),
            initial_temperature_k: float(config_table, "initial_temperature_k"),
            downstream_pressure_pa: float(config_table, "downstream_pressure_pa"),
            gas_gamma: float(config_table, "gas_gamma"),
            gas_constant_j_per_kg_k: float(config_table, "gas_constant_j_per_kg_k"),
            orifice_area_m2: float(config_table, "orifice_area_m2"),
            discharge_coefficient: float(config_table, "discharge_coefficient"),
        })
        .unwrap();
        let tolerances = table("tolerances", &data);

        for sample in data
            .get("sample")
            .and_then(toml::Value::as_array)
            .expect("sample array")
        {
            let sample = sample.as_table().unwrap();
            let actual = blowdown.sample_at(float(sample, "time_s")).unwrap();

            assert_abs_diff_eq!(
                actual.pressure_pa,
                float(sample, "expected_pressure_pa"),
                epsilon = float(tolerances, "absolute_pressure_pa")
            );
            assert_abs_diff_eq!(
                actual.pressure_ratio,
                float(sample, "expected_pressure_ratio"),
                epsilon = float(tolerances, "absolute_pressure_ratio")
            );
            assert_abs_diff_eq!(
                actual.temperature_k,
                float(sample, "expected_temperature_k"),
                epsilon = float(tolerances, "absolute_temperature_k")
            );
            assert_abs_diff_eq!(
                actual.mass_kg,
                float(sample, "expected_mass_kg"),
                epsilon = float(tolerances, "absolute_mass_kg")
            );
            assert_abs_diff_eq!(
                actual.mass_flow_kg_per_s,
                float(sample, "expected_mass_flow_kg_per_s"),
                epsilon = float(tolerances, "absolute_mass_flow_kg_per_s")
            );
            assert_eq!(
                string(sample, "expected_validation"),
                actual.validation.as_label()
            );
        }
    }

    #[test]
    fn isentropic_tank_blowdown_rejects_invalid_values() {
        let mut bad = config();
        bad.gas_gamma = 1.0;
        assert!(matches!(
            IsentropicTankBlowdown::new(bad),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        let blowdown = IsentropicTankBlowdown::new(config()).unwrap();
        assert!(matches!(
            blowdown.sample_at(-1.0),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
        assert!(matches!(
            blowdown.sample_at(10_000.0),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    fn table<'a>(key: &str, value: &'a toml::Value) -> &'a Table {
        value
            .get(key)
            .and_then(toml::Value::as_table)
            .unwrap_or_else(|| panic!("missing table {key}"))
    }

    fn float(table: &Table, key: &str) -> f64 {
        table
            .get(key)
            .and_then(toml::Value::as_float)
            .unwrap_or_else(|| panic!("missing float {key}"))
    }

    fn string<'a>(table: &'a Table, key: &str) -> &'a str {
        table
            .get(key)
            .and_then(toml::Value::as_str)
            .unwrap_or_else(|| panic!("missing string {key}"))
    }
}
