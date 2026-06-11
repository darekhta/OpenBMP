//! Generic normalized turbopump map with affinity scaling and NPSH gate.

use openbmp_core::ValidationStatus;

use crate::error::FeedSystemError;

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;

/// Pump cavitation state derived from available and required NPSH.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PumpCavitationState {
    /// Available NPSH is at or above the speed-scaled requirement.
    Nominal,
    /// Available NPSH is below the speed-scaled requirement.
    Cavitating,
}

/// Normalized quadratic pump map.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalizedPumpMap {
    /// Head-ratio polynomial coefficients in increasing order:
    /// `a0 + a1*phi + a2*phi^2`.
    pub head_coefficients: [f64; 3],
    /// Efficiency-ratio polynomial coefficients in increasing order:
    /// `a0 + a1*phi + a2*phi^2`.
    pub efficiency_coefficients: [f64; 3],
    /// Head multiplier applied when the NPSH gate is cavitating.
    pub cavitation_head_multiplier: f64,
}

impl NormalizedPumpMap {
    /// Validate map coefficients.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite coefficients, non-positive
    /// design-point head/efficiency, or an invalid cavitation multiplier.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        for coefficient in self.head_coefficients {
            require_finite(coefficient, "pump head-map coefficient must be finite")?;
        }
        for coefficient in self.efficiency_coefficients {
            require_finite(
                coefficient,
                "pump efficiency-map coefficient must be finite",
            )?;
        }
        require_positive_finite(
            evaluate_quadratic(self.head_coefficients, 1.0),
            "pump head map must be positive at the design flow coefficient",
        )?;
        require_positive_finite(
            evaluate_quadratic(self.efficiency_coefficients, 1.0),
            "pump efficiency map must be positive at the design flow coefficient",
        )?;
        require_unit_interval(
            self.cavitation_head_multiplier,
            "pump cavitation head multiplier must lie in [0, 1]",
        )
    }

    fn head_ratio(self, flow_coefficient: f64) -> Result<f64, FeedSystemError> {
        normalized_ratio(
            self.head_coefficients,
            flow_coefficient,
            "pump head ratio is non-finite",
        )
    }

    fn efficiency_ratio(self, flow_coefficient: f64) -> Result<f64, FeedSystemError> {
        normalized_ratio(
            self.efficiency_coefficients,
            flow_coefficient,
            "pump efficiency ratio is non-finite",
        )
    }
}

/// Turbopump design point used as the affinity-law anchor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TurbopumpDesignPoint {
    /// Design volumetric flow in m^3/s.
    pub volumetric_flow_m3_per_s: f64,
    /// Design pressure rise in Pa.
    pub pressure_rise_pa: f64,
    /// Design shaft speed in rad/s.
    pub shaft_speed_rad_per_s: f64,
    /// Pumped-fluid density in kg/m^3.
    pub fluid_density_kg_m3: f64,
    /// Hydraulic efficiency at the design point in `(0, 1]`.
    pub efficiency: f64,
    /// Required NPSH at the design speed in m.
    pub required_npsh_m: f64,
    /// Published/generic specific-speed family marker for this map.
    pub specific_speed: f64,
}

impl TurbopumpDesignPoint {
    /// Validate the design point.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite, non-positive, or
    /// out-of-envelope design-point values.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_positive_finite(
            self.volumetric_flow_m3_per_s,
            "pump design flow must be positive",
        )?;
        require_positive_finite(
            self.pressure_rise_pa,
            "pump design pressure rise must be positive",
        )?;
        require_positive_finite(
            self.shaft_speed_rad_per_s,
            "pump design shaft speed must be positive",
        )?;
        require_positive_finite(
            self.fluid_density_kg_m3,
            "pump fluid density must be positive",
        )?;
        require_positive_finite(self.efficiency, "pump design efficiency must be positive")?;
        if self.efficiency > 1.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "pump design efficiency must not exceed 1",
            });
        }
        require_nonnegative_finite(
            self.required_npsh_m,
            "pump required NPSH must be finite and non-negative",
        )?;
        require_positive_finite(self.specific_speed, "pump specific speed must be positive")
    }
}

/// Turbopump configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TurbopumpConfig {
    /// Design point used by the affinity laws.
    pub design: TurbopumpDesignPoint,
    /// Normalized generic map shape.
    pub map: NormalizedPumpMap,
}

impl TurbopumpConfig {
    /// Validate pump configuration.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid design point or map values.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        self.design.require_valid()?;
        self.map.require_valid()
    }
}

/// One turbopump operating point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TurbopumpOperatingPoint {
    /// Current volumetric flow in m^3/s.
    pub volumetric_flow_m3_per_s: f64,
    /// Current shaft speed in rad/s.
    pub shaft_speed_rad_per_s: f64,
    /// Pump inlet pressure in Pa.
    pub suction_pressure_pa: f64,
    /// Fluid vapor pressure in Pa.
    pub vapor_pressure_pa: f64,
}

impl TurbopumpOperatingPoint {
    fn require_valid(self) -> Result<(), FeedSystemError> {
        require_nonnegative_finite(
            self.volumetric_flow_m3_per_s,
            "pump operating flow must be finite and non-negative",
        )?;
        require_positive_finite(
            self.shaft_speed_rad_per_s,
            "pump operating shaft speed must be positive",
        )?;
        require_nonnegative_finite(
            self.suction_pressure_pa,
            "pump suction pressure must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.vapor_pressure_pa,
            "pump vapor pressure must be finite and non-negative",
        )
    }
}

/// Solved turbopump operating state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TurbopumpSnapshot {
    /// Normalized flow coefficient after affinity scaling.
    pub flow_coefficient: f64,
    /// Current shaft-speed ratio `N/N_design`.
    pub speed_ratio: f64,
    /// Pump pressure rise in Pa.
    pub pressure_rise_pa: f64,
    /// Pump discharge pressure in Pa.
    pub discharge_pressure_pa: f64,
    /// Available NPSH in m.
    pub available_npsh_m: f64,
    /// Speed-scaled required NPSH in m.
    pub required_npsh_m: f64,
    /// Cavitation verdict.
    pub cavitation: PumpCavitationState,
    /// Pump hydraulic efficiency after map scaling.
    pub efficiency: f64,
    /// Hydraulic power in W.
    pub hydraulic_power_w: f64,
    /// Shaft power in W.
    pub shaft_power_w: f64,
    /// Validation posture of the pump model.
    pub validation: ValidationStatus,
}

/// Generic normalized turbopump model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Turbopump {
    config: TurbopumpConfig,
}

impl Turbopump {
    /// Construct a validated turbopump model.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the configuration is invalid.
    pub fn new(config: TurbopumpConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self { config })
    }

    /// Borrow the validated pump configuration.
    #[must_use]
    pub const fn config(&self) -> TurbopumpConfig {
        self.config
    }

    /// Solve the pump at one operating point.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid operating values or non-finite
    /// map/affinity-law outputs.
    pub fn solve(
        &self,
        operating: TurbopumpOperatingPoint,
    ) -> Result<TurbopumpSnapshot, FeedSystemError> {
        operating.require_valid()?;

        let speed_ratio =
            operating.shaft_speed_rad_per_s / self.config.design.shaft_speed_rad_per_s;
        let flow_coefficient = (operating.volumetric_flow_m3_per_s
            / self.config.design.volumetric_flow_m3_per_s)
            / speed_ratio;
        if !flow_coefficient.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "pump flow coefficient is non-finite",
            });
        }

        let head_ratio = self.config.map.head_ratio(flow_coefficient)?.max(0.0);
        let efficiency_ratio = self.config.map.efficiency_ratio(flow_coefficient)?;
        let efficiency = self.config.design.efficiency * efficiency_ratio;
        if !efficiency.is_finite() || efficiency <= 0.0 || efficiency > 1.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "pump efficiency map produced an out-of-envelope value",
            });
        }

        let required_npsh_m = self.config.design.required_npsh_m * speed_ratio * speed_ratio;
        let available_npsh_m = (operating.suction_pressure_pa - operating.vapor_pressure_pa)
            / (self.config.design.fluid_density_kg_m3 * STANDARD_GRAVITY_M_S2);
        if !required_npsh_m.is_finite() || !available_npsh_m.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "pump NPSH calculation is non-finite",
            });
        }

        let cavitation = if available_npsh_m < required_npsh_m {
            PumpCavitationState::Cavitating
        } else {
            PumpCavitationState::Nominal
        };
        let cavitation_multiplier = match cavitation {
            PumpCavitationState::Nominal => 1.0,
            PumpCavitationState::Cavitating => self.config.map.cavitation_head_multiplier,
        };
        let pressure_rise_pa = self.config.design.pressure_rise_pa
            * speed_ratio
            * speed_ratio
            * head_ratio
            * cavitation_multiplier;
        let discharge_pressure_pa = operating.suction_pressure_pa + pressure_rise_pa;
        let hydraulic_power_w = pressure_rise_pa * operating.volumetric_flow_m3_per_s;
        let shaft_power_w = hydraulic_power_w / efficiency;
        for value in [
            pressure_rise_pa,
            discharge_pressure_pa,
            hydraulic_power_w,
            shaft_power_w,
        ] {
            if !value.is_finite() {
                return Err(FeedSystemError::NonFinite {
                    reason: "pump output contains a non-finite value",
                });
            }
        }

        Ok(TurbopumpSnapshot {
            flow_coefficient,
            speed_ratio,
            pressure_rise_pa,
            discharge_pressure_pa,
            available_npsh_m,
            required_npsh_m,
            cavitation,
            efficiency,
            hydraulic_power_w,
            shaft_power_w,
            validation: ValidationStatus::ValidatedToy,
        })
    }
}

fn evaluate_quadratic(coefficients: [f64; 3], x: f64) -> f64 {
    coefficients[0] + coefficients[1] * x + coefficients[2] * x * x
}

fn normalized_ratio(
    coefficients: [f64; 3],
    flow_coefficient: f64,
    non_finite_reason: &'static str,
) -> Result<f64, FeedSystemError> {
    let design_value = evaluate_quadratic(coefficients, 1.0);
    let value = evaluate_quadratic(coefficients, flow_coefficient) / design_value;
    if !value.is_finite() {
        return Err(FeedSystemError::NonFinite {
            reason: non_finite_reason,
        });
    }
    Ok(value)
}

fn require_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    if !value.is_finite() {
        return Err(FeedSystemError::NonFinite { reason });
    }
    Ok(())
}

fn require_positive_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    require_finite(value, reason)?;
    if value <= 0.0 {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

fn require_nonnegative_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    require_finite(value, reason)?;
    if value < 0.0 {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

fn require_unit_interval(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    require_finite(value, reason)?;
    if !(0.0..=1.0).contains(&value) {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    fn map() -> NormalizedPumpMap {
        NormalizedPumpMap {
            head_coefficients: [1.2, -0.2, 0.0],
            efficiency_coefficients: [0.8, 0.4, -0.2],
            cavitation_head_multiplier: 0.25,
        }
    }

    fn config() -> TurbopumpConfig {
        TurbopumpConfig {
            design: TurbopumpDesignPoint {
                volumetric_flow_m3_per_s: 0.05,
                pressure_rise_pa: 6.0e6,
                shaft_speed_rad_per_s: 3_000.0,
                fluid_density_kg_m3: 810.0,
                efficiency: 0.70,
                required_npsh_m: 20.0,
                specific_speed: 0.8,
            },
            map: map(),
        }
    }

    fn operating(flow: f64, speed: f64, suction_pressure_pa: f64) -> TurbopumpOperatingPoint {
        TurbopumpOperatingPoint {
            volumetric_flow_m3_per_s: flow,
            shaft_speed_rad_per_s: speed,
            suction_pressure_pa,
            vapor_pressure_pa: 30_000.0,
        }
    }

    #[test]
    fn turbopump_affinity_scales_head_with_speed_squared() {
        let pump = Turbopump::new(config()).unwrap();
        let design = pump.solve(operating(0.05, 3_000.0, 1.0e6)).unwrap();
        let scaled = pump.solve(operating(0.10, 6_000.0, 1.0e6)).unwrap();

        assert_abs_diff_eq!(design.flow_coefficient, scaled.flow_coefficient);
        assert_abs_diff_eq!(
            scaled.pressure_rise_pa,
            4.0 * design.pressure_rise_pa,
            epsilon = 1.0e-8
        );
        assert_eq!(scaled.cavitation, PumpCavitationState::Nominal);
    }

    #[test]
    fn turbopump_head_drops_at_higher_flow_coefficient() {
        let pump = Turbopump::new(config()).unwrap();
        let design = pump.solve(operating(0.05, 3_000.0, 1.0e6)).unwrap();
        let high_flow = pump.solve(operating(0.075, 3_000.0, 1.0e6)).unwrap();

        assert!(high_flow.flow_coefficient > design.flow_coefficient);
        assert!(high_flow.pressure_rise_pa < design.pressure_rise_pa);
    }

    #[test]
    fn turbopump_npsh_gate_applies_cavitation_multiplier() {
        let pump = Turbopump::new(config()).unwrap();
        let nominal = pump.solve(operating(0.05, 3_000.0, 1.0e6)).unwrap();
        let cavitating = pump.solve(operating(0.05, 3_000.0, 100_000.0)).unwrap();

        assert_eq!(nominal.cavitation, PumpCavitationState::Nominal);
        assert_eq!(cavitating.cavitation, PumpCavitationState::Cavitating);
        assert_abs_diff_eq!(
            cavitating.pressure_rise_pa,
            nominal.pressure_rise_pa * map().cavitation_head_multiplier,
            epsilon = 1.0e-8
        );
    }

    #[test]
    fn turbopump_reports_power_and_efficiency() {
        let snapshot = Turbopump::new(config())
            .unwrap()
            .solve(operating(0.05, 3_000.0, 1.0e6))
            .unwrap();

        assert_abs_diff_eq!(snapshot.efficiency, 0.70, epsilon = 1.0e-12);
        assert_abs_diff_eq!(snapshot.hydraulic_power_w, 6.0e6 * 0.05);
        assert_abs_diff_eq!(snapshot.shaft_power_w, snapshot.hydraulic_power_w / 0.70);
        assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
    }

    #[test]
    fn turbopump_rejects_invalid_map_and_operating_point() {
        let mut bad = config();
        bad.map.cavitation_head_multiplier = 1.2;
        assert!(matches!(
            Turbopump::new(bad),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        let pump = Turbopump::new(config()).unwrap();
        assert!(matches!(
            pump.solve(operating(0.05, 0.0, 1.0e6)),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }
}
