//! Deterministic throttle and mixture-ratio control primitives.

use openbmp_core::ValidationStatus;

use crate::error::FeedSystemError;

/// Paired oxidizer/fuel valve opening commands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ValveCommandPair {
    /// Oxidizer valve opening fraction in `[0, 1]`.
    pub oxidizer_open_fraction: f64,
    /// Fuel valve opening fraction in `[0, 1]`.
    pub fuel_open_fraction: f64,
}

impl ValveCommandPair {
    /// Validate valve command bounds.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when either command is non-finite or outside
    /// `[0, 1]`.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_unit_interval(
            self.oxidizer_open_fraction,
            "oxidizer valve command must lie in [0, 1]",
        )?;
        require_unit_interval(
            self.fuel_open_fraction,
            "fuel valve command must lie in [0, 1]",
        )
    }
}

/// Chamber-pressure and optional mixture-ratio target.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrottleMixtureSetpoint {
    /// Chamber pressure setpoint in Pa.
    pub chamber_pressure_pa: f64,
    /// Optional oxidizer/fuel mixture-ratio setpoint.
    pub mixture_ratio: Option<f64>,
}

impl ThrottleMixtureSetpoint {
    fn require_valid(self) -> Result<(), FeedSystemError> {
        require_nonnegative_finite(
            self.chamber_pressure_pa,
            "chamber pressure setpoint must be finite and non-negative",
        )?;
        if let Some(mixture_ratio) = self.mixture_ratio {
            require_positive_finite(mixture_ratio, "mixture-ratio setpoint must be positive")?;
        }
        Ok(())
    }
}

/// Chamber-pressure and optional mixture-ratio measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrottleMixtureMeasurement {
    /// Measured chamber pressure in Pa.
    pub chamber_pressure_pa: f64,
    /// Optional measured oxidizer/fuel mixture ratio.
    pub mixture_ratio: Option<f64>,
}

impl ThrottleMixtureMeasurement {
    fn require_valid(self) -> Result<(), FeedSystemError> {
        require_nonnegative_finite(
            self.chamber_pressure_pa,
            "measured chamber pressure must be finite and non-negative",
        )?;
        if let Some(mixture_ratio) = self.mixture_ratio {
            require_positive_finite(mixture_ratio, "measured mixture ratio must be positive")?;
        }
        Ok(())
    }
}

/// Deterministic pressure/MR controller tuning and actuator limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrottleMixtureControllerConfig {
    /// Proportional gain applied to chamber-pressure error.
    pub pressure_proportional_gain_per_pa: f64,
    /// Integral gain applied to accumulated chamber-pressure error.
    pub pressure_integral_gain_per_pa_s: f64,
    /// Absolute clamp for pressure integral state in Pa*s.
    pub pressure_integral_limit_pa_s: f64,
    /// Proportional gain applied to mixture-ratio error.
    pub mixture_proportional_gain: f64,
    /// Integral gain applied to accumulated mixture-ratio error.
    pub mixture_integral_gain_per_s: f64,
    /// Absolute clamp for mixture-ratio integral state in seconds.
    pub mixture_integral_limit_s: f64,
    /// Minimum valve opening fraction.
    pub min_open_fraction: f64,
    /// Maximum valve opening fraction.
    pub max_open_fraction: f64,
    /// Maximum absolute valve-command slew rate in opening-fraction per second.
    pub max_open_fraction_slew_per_s: f64,
}

impl ThrottleMixtureControllerConfig {
    /// Validate controller gains and actuator limits.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite, negative, or inconsistent
    /// controller parameters.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_nonnegative_finite(
            self.pressure_proportional_gain_per_pa,
            "pressure proportional gain must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.pressure_integral_gain_per_pa_s,
            "pressure integral gain must be finite and non-negative",
        )?;
        require_positive_finite(
            self.pressure_integral_limit_pa_s,
            "pressure integral limit must be positive",
        )?;
        require_nonnegative_finite(
            self.mixture_proportional_gain,
            "mixture proportional gain must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.mixture_integral_gain_per_s,
            "mixture integral gain must be finite and non-negative",
        )?;
        require_positive_finite(
            self.mixture_integral_limit_s,
            "mixture integral limit must be positive",
        )?;
        require_unit_interval(
            self.min_open_fraction,
            "minimum valve opening must lie in [0, 1]",
        )?;
        require_unit_interval(
            self.max_open_fraction,
            "maximum valve opening must lie in [0, 1]",
        )?;
        if self.min_open_fraction > self.max_open_fraction {
            return Err(FeedSystemError::InvalidParameter {
                reason: "minimum valve opening must not exceed maximum valve opening",
            });
        }
        require_nonnegative_finite(
            self.max_open_fraction_slew_per_s,
            "maximum valve slew rate must be finite and non-negative",
        )
    }
}

/// Controller state carried between samples.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrottleMixtureControllerState {
    /// Most recent bounded valve commands.
    pub valve_commands: ValveCommandPair,
    /// Accumulated chamber-pressure error in Pa*s.
    pub pressure_integral_pa_s: f64,
    /// Accumulated mixture-ratio error in seconds.
    pub mixture_integral_s: f64,
}

impl ThrottleMixtureControllerState {
    /// Build a state with zero integral terms.
    #[must_use]
    pub const fn new(valve_commands: ValveCommandPair) -> Self {
        Self {
            valve_commands,
            pressure_integral_pa_s: 0.0,
            mixture_integral_s: 0.0,
        }
    }

    fn require_valid(self) -> Result<(), FeedSystemError> {
        self.valve_commands.require_valid()?;
        require_finite(
            self.pressure_integral_pa_s,
            "pressure integral state must be finite",
        )?;
        require_finite(
            self.mixture_integral_s,
            "mixture integral state must be finite",
        )
    }
}

/// Controller result for one sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrottleMixtureControllerSnapshot {
    /// Updated controller state.
    pub state: ThrottleMixtureControllerState,
    /// Chamber-pressure setpoint error in Pa.
    pub pressure_error_pa: f64,
    /// Mixture-ratio setpoint error when the MR loop is active.
    pub mixture_ratio_error: Option<f64>,
    /// Unslewed and unclamped oxidizer command requested by the PI law.
    pub raw_oxidizer_open_fraction: f64,
    /// Unslewed and unclamped fuel command requested by the PI law.
    pub raw_fuel_open_fraction: f64,
    /// Validation posture of the controller model.
    pub validation: ValidationStatus,
}

/// PI throttle/MR controller for dual-valve feed networks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrottleMixtureController {
    config: ThrottleMixtureControllerConfig,
}

impl ThrottleMixtureController {
    /// Construct a validated controller.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the configuration is invalid.
    pub fn new(config: ThrottleMixtureControllerConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self { config })
    }

    /// Borrow the validated controller configuration.
    #[must_use]
    pub const fn config(&self) -> ThrottleMixtureControllerConfig {
        self.config
    }

    /// Advance the controller by one deterministic sample.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid state, setpoint, measurement, or
    /// time-step values.
    pub fn step(
        &self,
        state: ThrottleMixtureControllerState,
        setpoint: ThrottleMixtureSetpoint,
        measurement: ThrottleMixtureMeasurement,
        dt_s: f64,
    ) -> Result<ThrottleMixtureControllerSnapshot, FeedSystemError> {
        state.require_valid()?;
        setpoint.require_valid()?;
        measurement.require_valid()?;
        require_nonnegative_finite(dt_s, "controller time step must be finite and non-negative")?;

        let pressure_error_pa = setpoint.chamber_pressure_pa - measurement.chamber_pressure_pa;
        let pressure_integral_pa_s = clamp_symmetric(
            state.pressure_integral_pa_s + pressure_error_pa * dt_s,
            self.config.pressure_integral_limit_pa_s,
        );
        let pressure_delta = self.config.pressure_proportional_gain_per_pa * pressure_error_pa
            + self.config.pressure_integral_gain_per_pa_s * pressure_integral_pa_s;

        let (mixture_ratio_error, mixture_integral_s, mixture_delta) =
            self.mixture_terms(state, setpoint, measurement, dt_s)?;

        let raw_oxidizer_open_fraction =
            state.valve_commands.oxidizer_open_fraction + pressure_delta + mixture_delta;
        let raw_fuel_open_fraction =
            state.valve_commands.fuel_open_fraction + pressure_delta - mixture_delta;
        if !raw_oxidizer_open_fraction.is_finite() || !raw_fuel_open_fraction.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "controller raw valve command is non-finite",
            });
        }

        let oxidizer_open_fraction = self.limit_command(
            state.valve_commands.oxidizer_open_fraction,
            raw_oxidizer_open_fraction,
            dt_s,
        );
        let fuel_open_fraction = self.limit_command(
            state.valve_commands.fuel_open_fraction,
            raw_fuel_open_fraction,
            dt_s,
        );

        Ok(ThrottleMixtureControllerSnapshot {
            state: ThrottleMixtureControllerState {
                valve_commands: ValveCommandPair {
                    oxidizer_open_fraction,
                    fuel_open_fraction,
                },
                pressure_integral_pa_s,
                mixture_integral_s,
            },
            pressure_error_pa,
            mixture_ratio_error,
            raw_oxidizer_open_fraction,
            raw_fuel_open_fraction,
            validation: ValidationStatus::ValidatedToy,
        })
    }

    fn mixture_terms(
        self,
        state: ThrottleMixtureControllerState,
        setpoint: ThrottleMixtureSetpoint,
        measurement: ThrottleMixtureMeasurement,
        dt_s: f64,
    ) -> Result<(Option<f64>, f64, f64), FeedSystemError> {
        let Some(target_mixture_ratio) = setpoint.mixture_ratio else {
            return Ok((None, state.mixture_integral_s, 0.0));
        };
        let Some(measured_mixture_ratio) = measurement.mixture_ratio else {
            return Err(FeedSystemError::InvalidParameter {
                reason: "mixture-ratio setpoint requires a mixture-ratio measurement",
            });
        };
        let mixture_ratio_error = target_mixture_ratio - measured_mixture_ratio;
        let mixture_integral_s = clamp_symmetric(
            state.mixture_integral_s + mixture_ratio_error * dt_s,
            self.config.mixture_integral_limit_s,
        );
        let mixture_delta = self.config.mixture_proportional_gain * mixture_ratio_error
            + self.config.mixture_integral_gain_per_s * mixture_integral_s;
        if !mixture_delta.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "controller mixture correction is non-finite",
            });
        }
        Ok((Some(mixture_ratio_error), mixture_integral_s, mixture_delta))
    }

    fn limit_command(self, previous: f64, raw: f64, dt_s: f64) -> f64 {
        let clamped = raw.clamp(self.config.min_open_fraction, self.config.max_open_fraction);
        let max_step = self.config.max_open_fraction_slew_per_s * dt_s;
        if max_step == 0.0 {
            return previous;
        }
        let delta = (clamped - previous).clamp(-max_step, max_step);
        previous + delta
    }
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

fn clamp_symmetric(value: f64, limit: f64) -> f64 {
    value.clamp(-limit, limit)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    fn config() -> ThrottleMixtureControllerConfig {
        ThrottleMixtureControllerConfig {
            pressure_proportional_gain_per_pa: 1.0e-7,
            pressure_integral_gain_per_pa_s: 0.0,
            pressure_integral_limit_pa_s: 1.0e7,
            mixture_proportional_gain: 0.1,
            mixture_integral_gain_per_s: 0.0,
            mixture_integral_limit_s: 10.0,
            min_open_fraction: 0.0,
            max_open_fraction: 1.0,
            max_open_fraction_slew_per_s: 10.0,
        }
    }

    fn state() -> ThrottleMixtureControllerState {
        ThrottleMixtureControllerState::new(ValveCommandPair {
            oxidizer_open_fraction: 0.5,
            fuel_open_fraction: 0.5,
        })
    }

    #[test]
    fn controller_opens_both_valves_for_low_chamber_pressure() {
        let controller = ThrottleMixtureController::new(config()).unwrap();
        let snapshot = controller
            .step(
                state(),
                ThrottleMixtureSetpoint {
                    chamber_pressure_pa: 4.0e6,
                    mixture_ratio: None,
                },
                ThrottleMixtureMeasurement {
                    chamber_pressure_pa: 3.0e6,
                    mixture_ratio: None,
                },
                0.1,
            )
            .unwrap();

        assert_abs_diff_eq!(
            snapshot.state.valve_commands.oxidizer_open_fraction,
            0.6,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.state.valve_commands.fuel_open_fraction,
            0.6,
            epsilon = 1.0e-12
        );
        assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
    }

    #[test]
    fn controller_biases_valves_for_mixture_ratio_error() {
        let controller = ThrottleMixtureController::new(config()).unwrap();
        let snapshot = controller
            .step(
                state(),
                ThrottleMixtureSetpoint {
                    chamber_pressure_pa: 4.0e6,
                    mixture_ratio: Some(2.5),
                },
                ThrottleMixtureMeasurement {
                    chamber_pressure_pa: 4.0e6,
                    mixture_ratio: Some(2.0),
                },
                0.1,
            )
            .unwrap();

        assert_abs_diff_eq!(
            snapshot.state.valve_commands.oxidizer_open_fraction,
            0.55,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.state.valve_commands.fuel_open_fraction,
            0.45,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.mixture_ratio_error.unwrap(),
            0.5,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn controller_applies_clamp_and_slew_limit() {
        let mut limited = config();
        limited.max_open_fraction = 0.9;
        limited.max_open_fraction_slew_per_s = 0.2;
        let controller = ThrottleMixtureController::new(limited).unwrap();
        let snapshot = controller
            .step(
                state(),
                ThrottleMixtureSetpoint {
                    chamber_pressure_pa: 10.0e6,
                    mixture_ratio: None,
                },
                ThrottleMixtureMeasurement {
                    chamber_pressure_pa: 1.0e6,
                    mixture_ratio: None,
                },
                0.5,
            )
            .unwrap();

        assert!(snapshot.raw_oxidizer_open_fraction > limited.max_open_fraction);
        assert_abs_diff_eq!(
            snapshot.state.valve_commands.oxidizer_open_fraction,
            0.6,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.state.valve_commands.fuel_open_fraction,
            0.6,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn controller_rejects_missing_mixture_measurement() {
        let controller = ThrottleMixtureController::new(config()).unwrap();

        assert!(matches!(
            controller.step(
                state(),
                ThrottleMixtureSetpoint {
                    chamber_pressure_pa: 4.0e6,
                    mixture_ratio: Some(2.5),
                },
                ThrottleMixtureMeasurement {
                    chamber_pressure_pa: 4.0e6,
                    mixture_ratio: None,
                },
                0.1,
            ),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn controller_rejects_invalid_limits() {
        let mut bad = config();
        bad.min_open_fraction = 0.8;
        bad.max_open_fraction = 0.2;

        assert!(matches!(
            ThrottleMixtureController::new(bad),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }
}
