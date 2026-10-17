//! Transient feed-network wrappers around chamber and valve primitives.

use openbmp_core::ValidationStatus;

use crate::{
    chamber::{
        ChamberFeedInput, TransientChamber, TransientChamberConfig, TransientChamberSnapshot,
        TransientChamberState,
    },
    control::ValveCommandPair,
    error::FeedSystemError,
};

/// One incompressible tank/valve leg feeding a transient chamber.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ValveFeedLegConfig {
    /// Upstream tank/feed pressure in Pa.
    pub tank_pressure_pa: f64,
    /// Propellant density in kg/m^3.
    pub propellant_density_kg_m3: f64,
    /// Full-open valve flow area in m^2.
    pub valve_area_m2: f64,
    /// Valve discharge coefficient in `(0, 1]`.
    pub valve_discharge_coefficient: f64,
}

impl ValveFeedLegConfig {
    /// Validate one feed leg.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite, non-positive, or
    /// out-of-envelope leg parameters.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_positive_finite(
            self.tank_pressure_pa,
            "feed-leg tank pressure must be positive",
        )?;
        require_positive_finite(
            self.propellant_density_kg_m3,
            "feed-leg propellant density must be positive",
        )?;
        require_positive_finite(self.valve_area_m2, "feed-leg valve area must be positive")?;
        require_positive_finite(
            self.valve_discharge_coefficient,
            "feed-leg valve discharge coefficient must be positive",
        )?;
        if self.valve_discharge_coefficient > 1.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "feed-leg valve discharge coefficient must not exceed 1",
            });
        }
        Ok(())
    }

    fn mass_flow_kg_per_s(self, chamber_pressure_pa: f64, open_fraction: f64) -> f64 {
        if open_fraction == 0.0 {
            return 0.0;
        }
        let delta_p_pa = (self.tank_pressure_pa - chamber_pressure_pa).max(0.0);
        self.valve_discharge_coefficient
            * self.valve_area_m2
            * open_fraction
            * (2.0 * self.propellant_density_kg_m3 * delta_p_pa).sqrt()
    }
}

/// Configuration for a two-leg transient chamber feed network.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransientDualValveFeedNetworkConfig {
    /// Oxidizer tank/valve leg.
    pub oxidizer: ValveFeedLegConfig,
    /// Fuel tank/valve leg.
    pub fuel: ValveFeedLegConfig,
    /// Lumped chamber model.
    pub chamber: TransientChamberConfig,
}

impl TransientDualValveFeedNetworkConfig {
    /// Validate network parameters.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when any leg or chamber parameter is invalid.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        self.oxidizer.require_valid()?;
        self.fuel.require_valid()?;
        self.chamber.require_valid()
    }
}

/// State carried by a transient dual-valve feed network.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransientDualValveFeedNetworkState {
    /// Chamber pressure state.
    pub chamber: TransientChamberState,
}

impl TransientDualValveFeedNetworkState {
    /// Validate state.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid chamber pressure state.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        self.chamber.require_valid()
    }
}

/// Dynamic feed pressures for one transient dual-valve network step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedLegPressures {
    /// Oxidizer feed pressure immediately upstream of the valve in Pa.
    pub oxidizer_pressure_pa: f64,
    /// Fuel feed pressure immediately upstream of the valve in Pa.
    pub fuel_pressure_pa: f64,
}

impl FeedLegPressures {
    fn require_valid(self) -> Result<(), FeedSystemError> {
        require_nonnegative_finite(
            self.oxidizer_pressure_pa,
            "oxidizer feed pressure must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.fuel_pressure_pa,
            "fuel feed pressure must be finite and non-negative",
        )
    }
}

/// Snapshot returned after one transient dual-valve network step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransientDualValveFeedNetworkSnapshot {
    /// Updated network state.
    pub state: TransientDualValveFeedNetworkState,
    /// Chamber snapshot for the step.
    pub chamber: TransientChamberSnapshot,
    /// Oxidizer mass flow into the chamber in kg/s.
    pub oxidizer_mass_flow_kg_per_s: f64,
    /// Fuel mass flow into the chamber in kg/s.
    pub fuel_mass_flow_kg_per_s: f64,
    /// Valve commands used for this step.
    pub valve_commands: ValveCommandPair,
    /// Validation posture of the transient network model.
    pub validation: ValidationStatus,
}

/// One tank-bottom/ullage pressure sample for a pressurization benchmark.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PressurizationPressureSample {
    /// Sample time in seconds.
    pub time_s: f64,
    /// Tank-bottom pressure in Pa.
    pub tank_bottom_pressure_pa: f64,
    /// Ullage pressure in Pa.
    pub ullage_pressure_pa: f64,
}

impl PressurizationPressureSample {
    /// Creates a validated pressurization pressure sample.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when any sample value is non-finite or a
    /// pressure is negative.
    pub fn new(
        time_s: f64,
        tank_bottom_pressure_pa: f64,
        ullage_pressure_pa: f64,
    ) -> Result<Self, FeedSystemError> {
        require_nonnegative_finite(
            time_s,
            "pressurization pressure sample time must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            tank_bottom_pressure_pa,
            "tank-bottom pressure sample must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            ullage_pressure_pa,
            "ullage pressure sample must be finite and non-negative",
        )?;
        Ok(Self {
            time_s,
            tank_bottom_pressure_pa,
            ullage_pressure_pa,
        })
    }
}

/// Configuration for a lumped finite-volume propellant-tank pressurization model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteVolumePressurizationConfig {
    /// Regulated pressurant supply pressure in Pa.
    pub pressurant_inlet_pressure_pa: f64,
    /// Regulated pressurant supply temperature in K.
    pub pressurant_inlet_temperature_k: f64,
    /// Pressurant gas constant in J/(kg K).
    pub pressurant_gas_constant_j_per_kg_k: f64,
    /// Pressurant heat-capacity ratio.
    pub pressurant_heat_capacity_ratio: f64,
    /// Effective pressurant inlet flow area in m^2.
    pub pressurant_flow_area_m2: f64,
    /// Effective pressurant inlet discharge coefficient in `(0, 1]`.
    pub pressurant_discharge_coefficient: f64,
    /// Lumped ullage heat transfer coefficient in W/K.
    pub ullage_heat_transfer_w_per_k: f64,
    /// Lumped wall/source temperature for ullage heat transfer in K.
    pub tank_wall_temperature_k: f64,
    /// Propellant density in kg/m^3.
    pub propellant_density_kg_m3: f64,
    /// Effective tank cross-section area in m^2.
    pub tank_cross_section_area_m2: f64,
    /// Propellant outflow volume rate in m^3/s.
    pub propellant_outflow_volume_rate_m3_s: f64,
    /// Gravitational acceleration used for tank-bottom pressure in m/s^2.
    pub gravity_m_s2: f64,
}

impl FiniteVolumePressurizationConfig {
    /// Validate finite-volume pressurization model parameters.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when any parameter is non-finite or outside
    /// the supported envelope.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_positive_finite(
            self.pressurant_inlet_pressure_pa,
            "finite-volume pressurization inlet pressure must be positive",
        )?;
        require_positive_finite(
            self.pressurant_inlet_temperature_k,
            "finite-volume pressurization inlet temperature must be positive",
        )?;
        require_positive_finite(
            self.pressurant_gas_constant_j_per_kg_k,
            "finite-volume pressurization gas constant must be positive",
        )?;
        if !self.pressurant_heat_capacity_ratio.is_finite()
            || self.pressurant_heat_capacity_ratio <= 1.0
        {
            return Err(FeedSystemError::InvalidParameter {
                reason: "finite-volume pressurization heat-capacity ratio must be greater than 1",
            });
        }
        require_positive_finite(
            self.pressurant_flow_area_m2,
            "finite-volume pressurization flow area must be positive",
        )?;
        require_positive_finite(
            self.pressurant_discharge_coefficient,
            "finite-volume pressurization discharge coefficient must be positive",
        )?;
        if self.pressurant_discharge_coefficient > 1.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "finite-volume pressurization discharge coefficient must not exceed 1",
            });
        }
        require_nonnegative_finite(
            self.ullage_heat_transfer_w_per_k,
            "finite-volume pressurization heat transfer must be finite and non-negative",
        )?;
        require_positive_finite(
            self.tank_wall_temperature_k,
            "finite-volume pressurization wall temperature must be positive",
        )?;
        require_positive_finite(
            self.propellant_density_kg_m3,
            "finite-volume pressurization propellant density must be positive",
        )?;
        require_positive_finite(
            self.tank_cross_section_area_m2,
            "finite-volume pressurization tank area must be positive",
        )?;
        require_nonnegative_finite(
            self.propellant_outflow_volume_rate_m3_s,
            "finite-volume pressurization propellant outflow must be finite and non-negative",
        )?;
        require_positive_finite(
            self.gravity_m_s2,
            "finite-volume pressurization gravity must be positive",
        )
    }

    fn cv_j_per_kg_k(self) -> f64 {
        self.pressurant_gas_constant_j_per_kg_k / (self.pressurant_heat_capacity_ratio - 1.0)
    }

    fn cp_j_per_kg_k(self) -> f64 {
        self.pressurant_heat_capacity_ratio * self.cv_j_per_kg_k()
    }
}

/// State for a lumped finite-volume propellant-tank pressurization model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteVolumePressurizationState {
    /// State time in seconds.
    pub time_s: f64,
    /// Pressurant gas mass in the ullage volume in kg.
    pub ullage_gas_mass_kg: f64,
    /// Ullage gas temperature in K.
    pub ullage_temperature_k: f64,
    /// Ullage volume in m^3.
    pub ullage_volume_m3: f64,
    /// Remaining propellant volume in m^3.
    pub propellant_volume_m3: f64,
}

impl FiniteVolumePressurizationState {
    /// Construct a validated state from ullage pressure, temperature, and volume.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when inputs are invalid or ideal-gas mass is
    /// non-finite.
    pub fn from_ullage_pressure(
        config: FiniteVolumePressurizationConfig,
        time_s: f64,
        ullage_pressure_pa: f64,
        ullage_temperature_k: f64,
        ullage_volume_m3: f64,
        propellant_volume_m3: f64,
    ) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        require_nonnegative_finite(
            time_s,
            "finite-volume pressurization state time must be finite and non-negative",
        )?;
        require_positive_finite(
            ullage_pressure_pa,
            "finite-volume pressurization ullage pressure must be positive",
        )?;
        require_positive_finite(
            ullage_temperature_k,
            "finite-volume pressurization ullage temperature must be positive",
        )?;
        require_positive_finite(
            ullage_volume_m3,
            "finite-volume pressurization ullage volume must be positive",
        )?;
        require_nonnegative_finite(
            propellant_volume_m3,
            "finite-volume pressurization propellant volume must be finite and non-negative",
        )?;
        let ullage_gas_mass_kg = ullage_pressure_pa * ullage_volume_m3
            / (config.pressurant_gas_constant_j_per_kg_k * ullage_temperature_k);
        if !ullage_gas_mass_kg.is_finite() || ullage_gas_mass_kg <= 0.0 {
            return Err(FeedSystemError::NonFinite {
                reason: "finite-volume pressurization ullage mass is non-finite",
            });
        }
        Ok(Self {
            time_s,
            ullage_gas_mass_kg,
            ullage_temperature_k,
            ullage_volume_m3,
            propellant_volume_m3,
        })
    }

    /// Validate state values.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when state values are invalid.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_nonnegative_finite(
            self.time_s,
            "finite-volume pressurization state time must be finite and non-negative",
        )?;
        require_positive_finite(
            self.ullage_gas_mass_kg,
            "finite-volume pressurization ullage mass must be positive",
        )?;
        require_positive_finite(
            self.ullage_temperature_k,
            "finite-volume pressurization ullage temperature must be positive",
        )?;
        require_positive_finite(
            self.ullage_volume_m3,
            "finite-volume pressurization ullage volume must be positive",
        )?;
        require_nonnegative_finite(
            self.propellant_volume_m3,
            "finite-volume pressurization propellant volume must be finite and non-negative",
        )
    }
}

/// Snapshot returned after one finite-volume pressurization step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteVolumePressurizationSnapshot {
    /// Updated state.
    pub state: FiniteVolumePressurizationState,
    /// Ullage pressure in Pa.
    pub ullage_pressure_pa: f64,
    /// Tank-bottom pressure in Pa.
    pub tank_bottom_pressure_pa: f64,
    /// Pressurant mass flow into the ullage in kg/s.
    pub pressurant_mass_flow_kg_per_s: f64,
    /// Propellant outflow volume rate applied during the step in m^3/s.
    pub propellant_outflow_volume_rate_m3_s: f64,
    /// Ullage heat-transfer rate in W.
    pub ullage_heat_transfer_w: f64,
    /// Pressure-history sample for residual gates.
    pub pressure_sample: PressurizationPressureSample,
    /// Validation posture of the finite-volume model.
    pub validation: ValidationStatus,
}

/// Lumped finite-volume propellant-tank pressurization model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteVolumePressurizationModel {
    config: FiniteVolumePressurizationConfig,
}

impl FiniteVolumePressurizationModel {
    /// Construct a validated finite-volume pressurization model.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the configuration is invalid.
    pub fn new(config: FiniteVolumePressurizationConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self { config })
    }

    /// Borrow the validated model configuration.
    #[must_use]
    pub const fn config(&self) -> FiniteVolumePressurizationConfig {
        self.config
    }

    /// Validation posture of this finite-volume model.
    #[must_use]
    pub const fn validation(&self) -> ValidationStatus {
        ValidationStatus::Research
    }

    /// Compute ullage pressure for a state.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the state is invalid or pressure is
    /// non-finite.
    pub fn ullage_pressure_pa(
        &self,
        state: FiniteVolumePressurizationState,
    ) -> Result<f64, FeedSystemError> {
        state.require_valid()?;
        let pressure = state.ullage_gas_mass_kg
            * self.config.pressurant_gas_constant_j_per_kg_k
            * state.ullage_temperature_k
            / state.ullage_volume_m3;
        if pressure.is_finite() && pressure >= 0.0 {
            Ok(pressure)
        } else {
            Err(FeedSystemError::NonFinite {
                reason: "finite-volume pressurization ullage pressure is non-finite",
            })
        }
    }

    /// Compute tank-bottom pressure for a state.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when state or pressure values are invalid.
    pub fn tank_bottom_pressure_pa(
        &self,
        state: FiniteVolumePressurizationState,
    ) -> Result<f64, FeedSystemError> {
        let ullage_pressure_pa = self.ullage_pressure_pa(state)?;
        self.tank_bottom_pressure_from_ullage(state, ullage_pressure_pa)
    }

    /// Advance the finite-volume pressurization model by one time step.
    ///
    /// The update is an explicit finite-volume mass/energy balance for one
    /// ullage control volume: pressurant mass and enthalpy enter through a
    /// compressible inlet, heat transfer is lumped as `hA*(T_wall - T)`, and
    /// `p dV` expansion work follows the propellant outflow volume rate.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid inputs or non-finite
    /// intermediate values.
    pub fn step(
        &self,
        state: FiniteVolumePressurizationState,
        dt_s: f64,
    ) -> Result<FiniteVolumePressurizationSnapshot, FeedSystemError> {
        state.require_valid()?;
        require_nonnegative_finite(
            dt_s,
            "finite-volume pressurization time step must be finite and non-negative",
        )?;

        let ullage_pressure_pa = self.ullage_pressure_pa(state)?;
        if dt_s == 0.0 {
            return self.snapshot(state, ullage_pressure_pa, 0.0, 0.0, 0.0);
        }

        let propellant_outflow_volume_rate_m3_s = self
            .config
            .propellant_outflow_volume_rate_m3_s
            .min(state.propellant_volume_m3 / dt_s);
        let pressurant_mass_flow_kg_per_s = self.pressurant_mass_flow_kg_per_s(ullage_pressure_pa);
        let ullage_heat_transfer_w = self.config.ullage_heat_transfer_w_per_k
            * (self.config.tank_wall_temperature_k - state.ullage_temperature_k);

        let internal_energy_j =
            state.ullage_gas_mass_kg * self.config.cv_j_per_kg_k() * state.ullage_temperature_k;
        let next_internal_energy_j = internal_energy_j
            + (pressurant_mass_flow_kg_per_s
                * self.config.cp_j_per_kg_k()
                * self.config.pressurant_inlet_temperature_k
                + ullage_heat_transfer_w
                - ullage_pressure_pa * propellant_outflow_volume_rate_m3_s)
                * dt_s;
        let next_ullage_gas_mass_kg =
            state.ullage_gas_mass_kg + pressurant_mass_flow_kg_per_s * dt_s;
        let next_ullage_volume_m3 =
            state.ullage_volume_m3 + propellant_outflow_volume_rate_m3_s * dt_s;
        let next_propellant_volume_m3 =
            state.propellant_volume_m3 - propellant_outflow_volume_rate_m3_s * dt_s;
        if next_internal_energy_j <= 0.0
            || next_ullage_gas_mass_kg <= 0.0
            || next_ullage_volume_m3 <= 0.0
            || next_propellant_volume_m3 < -1.0e-12
        {
            return Err(FeedSystemError::InvalidParameter {
                reason: "finite-volume pressurization step left the physical envelope",
            });
        }
        let next_ullage_temperature_k =
            next_internal_energy_j / (next_ullage_gas_mass_kg * self.config.cv_j_per_kg_k());
        let next_state = FiniteVolumePressurizationState {
            time_s: state.time_s + dt_s,
            ullage_gas_mass_kg: next_ullage_gas_mass_kg,
            ullage_temperature_k: next_ullage_temperature_k,
            ullage_volume_m3: next_ullage_volume_m3,
            propellant_volume_m3: next_propellant_volume_m3.max(0.0),
        };
        next_state.require_valid()?;
        let next_ullage_pressure_pa = self.ullage_pressure_pa(next_state)?;
        self.snapshot(
            next_state,
            next_ullage_pressure_pa,
            pressurant_mass_flow_kg_per_s,
            propellant_outflow_volume_rate_m3_s,
            ullage_heat_transfer_w,
        )
    }

    fn snapshot(
        &self,
        state: FiniteVolumePressurizationState,
        ullage_pressure_pa: f64,
        pressurant_mass_flow_kg_per_s: f64,
        propellant_outflow_volume_rate_m3_s: f64,
        ullage_heat_transfer_w: f64,
    ) -> Result<FiniteVolumePressurizationSnapshot, FeedSystemError> {
        let tank_bottom_pressure_pa =
            self.tank_bottom_pressure_from_ullage(state, ullage_pressure_pa)?;
        let pressure_sample = PressurizationPressureSample::new(
            state.time_s,
            tank_bottom_pressure_pa,
            ullage_pressure_pa,
        )?;
        Ok(FiniteVolumePressurizationSnapshot {
            state,
            ullage_pressure_pa,
            tank_bottom_pressure_pa,
            pressurant_mass_flow_kg_per_s,
            propellant_outflow_volume_rate_m3_s,
            ullage_heat_transfer_w,
            pressure_sample,
            validation: ValidationStatus::Research,
        })
    }

    fn tank_bottom_pressure_from_ullage(
        &self,
        state: FiniteVolumePressurizationState,
        ullage_pressure_pa: f64,
    ) -> Result<f64, FeedSystemError> {
        let hydrostatic_head_pa = self.config.propellant_density_kg_m3
            * self.config.gravity_m_s2
            * state.propellant_volume_m3
            / self.config.tank_cross_section_area_m2;
        let pressure = ullage_pressure_pa + hydrostatic_head_pa;
        if pressure.is_finite() && pressure >= 0.0 {
            Ok(pressure)
        } else {
            Err(FeedSystemError::NonFinite {
                reason: "finite-volume pressurization tank-bottom pressure is non-finite",
            })
        }
    }

    fn pressurant_mass_flow_kg_per_s(&self, ullage_pressure_pa: f64) -> f64 {
        if ullage_pressure_pa >= self.config.pressurant_inlet_pressure_pa {
            return 0.0;
        }
        let pressure_ratio =
            (ullage_pressure_pa / self.config.pressurant_inlet_pressure_pa).clamp(0.0, 1.0);
        let gamma = self.config.pressurant_heat_capacity_ratio;
        let critical_pressure_ratio = (2.0 / (gamma + 1.0)).powf(gamma / (gamma - 1.0));
        let flow_factor = if pressure_ratio <= critical_pressure_ratio {
            (gamma
                / (self.config.pressurant_gas_constant_j_per_kg_k
                    * self.config.pressurant_inlet_temperature_k)
                * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (gamma - 1.0)))
            .sqrt()
        } else {
            let expansion =
                pressure_ratio.powf(2.0 / gamma) - pressure_ratio.powf((gamma + 1.0) / gamma);
            (2.0 * gamma
                / (self.config.pressurant_gas_constant_j_per_kg_k
                    * self.config.pressurant_inlet_temperature_k
                    * (gamma - 1.0))
                * expansion.max(0.0))
            .sqrt()
        };
        self.config.pressurant_discharge_coefficient
            * self.config.pressurant_flow_area_m2
            * self.config.pressurant_inlet_pressure_pa
            * flow_factor
    }
}

/// Configuration for a calibrated reduced pressure-history model.
///
/// This model is intentionally a pressure-history closure helper for public
/// benchmark fixtures. It is not a finite-volume pressurization solver: startup
/// is represented by a linear ramp, then ullage pressure and hydrostatic head
/// decay with independent power laws fitted to the reference pressure traces.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReducedPressurizationPressureModelConfig {
    /// Initial sample time in seconds.
    pub initial_time_s: f64,
    /// End of the startup ramp in seconds.
    pub startup_time_s: f64,
    /// Final calibrated sample time in seconds.
    pub final_time_s: f64,
    /// Initial tank-bottom pressure in Pa.
    pub initial_tank_bottom_pressure_pa: f64,
    /// Initial ullage pressure in Pa.
    pub initial_ullage_pressure_pa: f64,
    /// Tank-bottom pressure at the end of startup in Pa.
    pub startup_tank_bottom_pressure_pa: f64,
    /// Ullage pressure at the end of startup in Pa.
    pub startup_ullage_pressure_pa: f64,
    /// Final tank-bottom pressure in Pa.
    pub final_tank_bottom_pressure_pa: f64,
    /// Final ullage pressure in Pa.
    pub final_ullage_pressure_pa: f64,
    /// Power-law exponent for post-startup ullage pressure decay.
    pub ullage_decay_exponent: f64,
    /// Power-law exponent for post-startup hydrostatic head decay.
    pub hydrostatic_head_decay_exponent: f64,
}

impl ReducedPressurizationPressureModelConfig {
    /// Validate the reduced pressure-history model configuration.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when times, pressures, or exponents are
    /// non-finite, out of range, or inconsistent.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_nonnegative_finite(
            self.initial_time_s,
            "reduced pressurization model initial time must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.startup_time_s,
            "reduced pressurization model startup time must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.final_time_s,
            "reduced pressurization model final time must be finite and non-negative",
        )?;
        if self.startup_time_s <= self.initial_time_s {
            return Err(FeedSystemError::InvalidParameter {
                reason: "reduced pressurization model startup time must follow initial time",
            });
        }
        if self.final_time_s <= self.startup_time_s {
            return Err(FeedSystemError::InvalidParameter {
                reason: "reduced pressurization model final time must follow startup time",
            });
        }

        require_nonnegative_finite(
            self.initial_tank_bottom_pressure_pa,
            "reduced pressurization model initial tank-bottom pressure must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.initial_ullage_pressure_pa,
            "reduced pressurization model initial ullage pressure must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.startup_tank_bottom_pressure_pa,
            "reduced pressurization model startup tank-bottom pressure must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.startup_ullage_pressure_pa,
            "reduced pressurization model startup ullage pressure must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.final_tank_bottom_pressure_pa,
            "reduced pressurization model final tank-bottom pressure must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.final_ullage_pressure_pa,
            "reduced pressurization model final ullage pressure must be finite and non-negative",
        )?;
        require_positive_finite(
            self.ullage_decay_exponent,
            "reduced pressurization model ullage exponent must be positive",
        )?;
        require_positive_finite(
            self.hydrostatic_head_decay_exponent,
            "reduced pressurization model hydrostatic-head exponent must be positive",
        )?;

        if self.initial_tank_bottom_pressure_pa < self.initial_ullage_pressure_pa
            || self.startup_tank_bottom_pressure_pa < self.startup_ullage_pressure_pa
            || self.final_tank_bottom_pressure_pa < self.final_ullage_pressure_pa
        {
            return Err(FeedSystemError::InvalidParameter {
                reason: "reduced pressurization model tank-bottom pressure must not be below ullage pressure",
            });
        }
        Ok(())
    }
}

/// Calibrated reduced pressure-history sampler for propellant-tank fixtures.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReducedPressurizationPressureModel {
    config: ReducedPressurizationPressureModelConfig,
}

impl ReducedPressurizationPressureModel {
    /// Construct a validated reduced pressure-history model.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the configuration is invalid.
    pub fn new(config: ReducedPressurizationPressureModelConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self { config })
    }

    /// Borrow the validated model configuration.
    #[must_use]
    pub const fn config(&self) -> ReducedPressurizationPressureModelConfig {
        self.config
    }

    /// Validation posture of this reduced calibrated model.
    #[must_use]
    pub const fn validation(&self) -> ValidationStatus {
        ValidationStatus::Research
    }

    /// Sample tank-bottom and ullage pressures at a calibrated time.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite or out-of-range sample times,
    /// or non-finite sampled pressures.
    pub fn sample_at(&self, time_s: f64) -> Result<PressurizationPressureSample, FeedSystemError> {
        require_nonnegative_finite(
            time_s,
            "reduced pressurization model sample time must be finite and non-negative",
        )?;
        if time_s < self.config.initial_time_s || time_s > self.config.final_time_s {
            return Err(FeedSystemError::InvalidParameter {
                reason: "reduced pressurization model sample time is outside calibrated range",
            });
        }

        let (ullage_pressure_pa, hydrostatic_head_pa) = if time_s <= self.config.startup_time_s {
            let ramp_fraction = (time_s - self.config.initial_time_s)
                / (self.config.startup_time_s - self.config.initial_time_s);
            (
                lerp(
                    self.config.initial_ullage_pressure_pa,
                    self.config.startup_ullage_pressure_pa,
                    ramp_fraction,
                ),
                lerp(
                    self.initial_hydrostatic_head_pa(),
                    self.startup_hydrostatic_head_pa(),
                    ramp_fraction,
                ),
            )
        } else {
            let remaining_fraction = ((self.config.final_time_s - time_s)
                / (self.config.final_time_s - self.config.startup_time_s))
                .clamp(0.0, 1.0);
            (
                self.config.final_ullage_pressure_pa
                    + (self.config.startup_ullage_pressure_pa
                        - self.config.final_ullage_pressure_pa)
                        * remaining_fraction.powf(self.config.ullage_decay_exponent),
                self.final_hydrostatic_head_pa()
                    + (self.startup_hydrostatic_head_pa() - self.final_hydrostatic_head_pa())
                        * remaining_fraction.powf(self.config.hydrostatic_head_decay_exponent),
            )
        };
        let tank_bottom_pressure_pa = ullage_pressure_pa + hydrostatic_head_pa;
        if !tank_bottom_pressure_pa.is_finite() || !ullage_pressure_pa.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "reduced pressurization model pressure sample is non-finite",
            });
        }

        PressurizationPressureSample::new(time_s, tank_bottom_pressure_pa, ullage_pressure_pa)
    }

    fn initial_hydrostatic_head_pa(&self) -> f64 {
        self.config.initial_tank_bottom_pressure_pa - self.config.initial_ullage_pressure_pa
    }

    fn startup_hydrostatic_head_pa(&self) -> f64 {
        self.config.startup_tank_bottom_pressure_pa - self.config.startup_ullage_pressure_pa
    }

    fn final_hydrostatic_head_pa(&self) -> f64 {
        self.config.final_tank_bottom_pressure_pa - self.config.final_ullage_pressure_pa
    }
}

/// Residual for one pressurization pressure-history sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PressurizationPressureResidualSample {
    /// Sample time in seconds.
    pub time_s: f64,
    /// Signed model minus reference tank-bottom pressure error in Pa.
    pub tank_bottom_pressure_error_pa: f64,
    /// Signed model minus reference ullage pressure error in Pa.
    pub ullage_pressure_error_pa: f64,
    /// Maximum absolute pressure error for the sample in Pa.
    pub max_abs_pressure_error_pa: f64,
}

/// Residual report for a sampled pressurization pressure history.
#[derive(Clone, Debug, PartialEq)]
pub struct PressurizationPressureResidualReport {
    samples: Vec<PressurizationPressureResidualSample>,
    /// Absolute pressure tolerance used for the gate in Pa.
    pub absolute_pressure_tolerance_pa: f64,
    /// Absolute time-matching tolerance used for sample alignment in seconds.
    pub absolute_time_tolerance_s: f64,
    /// Maximum tank-bottom absolute pressure error in Pa.
    pub max_abs_tank_bottom_pressure_error_pa: f64,
    /// Maximum ullage absolute pressure error in Pa.
    pub max_abs_ullage_pressure_error_pa: f64,
    /// Maximum absolute pressure error over both pressure channels in Pa.
    pub max_abs_pressure_error_pa: f64,
    /// Root-mean-square pressure error over both pressure channels in Pa.
    pub rms_pressure_error_pa: f64,
    /// Whether all pressure errors are inside the declared absolute tolerance.
    pub within_tolerance: bool,
    /// Validation posture of this benchmark residual report.
    pub validation: ValidationStatus,
}

impl PressurizationPressureResidualReport {
    /// Residual samples in reference order.
    #[must_use]
    pub fn samples(&self) -> &[PressurizationPressureResidualSample] {
        &self.samples
    }
}

/// Compare a model pressure history against a digitized pressurization reference.
///
/// The two sample arrays must have identical length and matching sample times
/// within `absolute_time_tolerance_s`; the comparison intentionally avoids
/// interpolation so benchmark closure remains declaration-order deterministic.
///
/// # Errors
///
/// Returns [`FeedSystemError`] for empty inputs, mismatched lengths, non-finite
/// tolerances, mismatched times, non-increasing reference times, or non-finite
/// residuals.
pub fn evaluate_pressurization_pressure_residuals(
    reference: &[PressurizationPressureSample],
    model: &[PressurizationPressureSample],
    absolute_pressure_tolerance_pa: f64,
    absolute_time_tolerance_s: f64,
) -> Result<PressurizationPressureResidualReport, FeedSystemError> {
    if reference.is_empty() {
        return Err(FeedSystemError::InvalidParameter {
            reason: "pressurization pressure residual reference must not be empty",
        });
    }
    if reference.len() != model.len() {
        return Err(FeedSystemError::InvalidParameter {
            reason: "pressurization pressure residual sample counts must match",
        });
    }
    require_positive_finite(
        absolute_pressure_tolerance_pa,
        "pressurization pressure residual tolerance must be positive",
    )?;
    require_nonnegative_finite(
        absolute_time_tolerance_s,
        "pressurization pressure residual time tolerance must be finite and non-negative",
    )?;

    let mut residual_samples = Vec::with_capacity(reference.len());
    let mut max_abs_tank_bottom_pressure_error_pa = 0.0_f64;
    let mut max_abs_ullage_pressure_error_pa = 0.0_f64;
    let mut sum_squared_pressure_error_pa2 = 0.0_f64;
    let mut previous_time_s = -1.0_f64;

    for (reference_sample, model_sample) in reference.iter().zip(model.iter()) {
        validate_pressure_sample(*reference_sample)?;
        validate_pressure_sample(*model_sample)?;
        if reference_sample.time_s <= previous_time_s {
            return Err(FeedSystemError::InvalidParameter {
                reason: "pressurization pressure reference times must be strictly increasing",
            });
        }
        let time_error_s = (model_sample.time_s - reference_sample.time_s).abs();
        if time_error_s > absolute_time_tolerance_s {
            return Err(FeedSystemError::InvalidParameter {
                reason: "pressurization pressure residual sample times must match",
            });
        }

        let tank_bottom_pressure_error_pa =
            model_sample.tank_bottom_pressure_pa - reference_sample.tank_bottom_pressure_pa;
        let ullage_pressure_error_pa =
            model_sample.ullage_pressure_pa - reference_sample.ullage_pressure_pa;
        if !tank_bottom_pressure_error_pa.is_finite() || !ullage_pressure_error_pa.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "pressurization pressure residual is non-finite",
            });
        }
        let max_abs_pressure_error_pa = tank_bottom_pressure_error_pa
            .abs()
            .max(ullage_pressure_error_pa.abs());
        max_abs_tank_bottom_pressure_error_pa =
            max_abs_tank_bottom_pressure_error_pa.max(tank_bottom_pressure_error_pa.abs());
        max_abs_ullage_pressure_error_pa =
            max_abs_ullage_pressure_error_pa.max(ullage_pressure_error_pa.abs());
        sum_squared_pressure_error_pa2 += tank_bottom_pressure_error_pa
            * tank_bottom_pressure_error_pa
            + ullage_pressure_error_pa * ullage_pressure_error_pa;
        residual_samples.push(PressurizationPressureResidualSample {
            time_s: reference_sample.time_s,
            tank_bottom_pressure_error_pa,
            ullage_pressure_error_pa,
            max_abs_pressure_error_pa,
        });
        previous_time_s = reference_sample.time_s;
    }

    let max_abs_pressure_error_pa =
        max_abs_tank_bottom_pressure_error_pa.max(max_abs_ullage_pressure_error_pa);
    let rms_pressure_error_pa =
        (sum_squared_pressure_error_pa2 / (2.0 * residual_samples.len() as f64)).sqrt();
    if !max_abs_pressure_error_pa.is_finite() || !rms_pressure_error_pa.is_finite() {
        return Err(FeedSystemError::NonFinite {
            reason: "pressurization pressure residual summary is non-finite",
        });
    }

    Ok(PressurizationPressureResidualReport {
        samples: residual_samples,
        absolute_pressure_tolerance_pa,
        absolute_time_tolerance_s,
        max_abs_tank_bottom_pressure_error_pa,
        max_abs_ullage_pressure_error_pa,
        max_abs_pressure_error_pa,
        rms_pressure_error_pa,
        within_tolerance: max_abs_pressure_error_pa <= absolute_pressure_tolerance_pa,
        validation: ValidationStatus::Research,
    })
}

fn validate_pressure_sample(sample: PressurizationPressureSample) -> Result<(), FeedSystemError> {
    PressurizationPressureSample::new(
        sample.time_s,
        sample.tank_bottom_pressure_pa,
        sample.ullage_pressure_pa,
    )
    .map(|_| ())
}

fn lerp(start: f64, end: f64, fraction: f64) -> f64 {
    start + (end - start) * fraction
}

/// Deterministic two-leg transient feed network coupled to a lumped chamber.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransientDualValveFeedNetwork {
    config: TransientDualValveFeedNetworkConfig,
    chamber: TransientChamber,
}

impl TransientDualValveFeedNetwork {
    /// Construct a validated transient network.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the configuration is invalid.
    pub fn new(config: TransientDualValveFeedNetworkConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self {
            config,
            chamber: TransientChamber::new(config.chamber)?,
        })
    }

    /// Borrow the validated network configuration.
    #[must_use]
    pub const fn config(&self) -> TransientDualValveFeedNetworkConfig {
        self.config
    }

    /// Advance the transient network by one time step.
    ///
    /// The feed legs are sampled at the start-of-step chamber pressure and held
    /// constant through the chamber-pressure update.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid state, commands, time-step, or
    /// non-finite intermediate values.
    pub fn step(
        &self,
        state: TransientDualValveFeedNetworkState,
        valve_commands: ValveCommandPair,
        dt_s: f64,
    ) -> Result<TransientDualValveFeedNetworkSnapshot, FeedSystemError> {
        self.step_with_feed_pressures(
            state,
            valve_commands,
            FeedLegPressures {
                oxidizer_pressure_pa: self.config.oxidizer.tank_pressure_pa,
                fuel_pressure_pa: self.config.fuel.tank_pressure_pa,
            },
            dt_s,
        )
    }

    /// Advance the transient network with dynamic upstream feed pressures.
    ///
    /// Runner-side line-transient adapters use this to keep valve geometry fixed
    /// while supplying instantaneous upstream pressure from another feed model.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid state, commands, dynamic
    /// pressures, time-step, or non-finite intermediate values.
    pub fn step_with_feed_pressures(
        &self,
        state: TransientDualValveFeedNetworkState,
        valve_commands: ValveCommandPair,
        feed_pressures: FeedLegPressures,
        dt_s: f64,
    ) -> Result<TransientDualValveFeedNetworkSnapshot, FeedSystemError> {
        state.require_valid()?;
        valve_commands.require_valid()?;
        feed_pressures.require_valid()?;
        require_nonnegative_finite(
            dt_s,
            "transient dual-valve network time step must be finite and non-negative",
        )?;

        let chamber_pressure_pa = state.chamber.pressure_pa;
        let mut oxidizer = self.config.oxidizer;
        oxidizer.tank_pressure_pa = feed_pressures.oxidizer_pressure_pa;
        let mut fuel = self.config.fuel;
        fuel.tank_pressure_pa = feed_pressures.fuel_pressure_pa;
        let oxidizer_mass_flow_kg_per_s =
            oxidizer.mass_flow_kg_per_s(chamber_pressure_pa, valve_commands.oxidizer_open_fraction);
        let fuel_mass_flow_kg_per_s =
            fuel.mass_flow_kg_per_s(chamber_pressure_pa, valve_commands.fuel_open_fraction);
        if !oxidizer_mass_flow_kg_per_s.is_finite() || !fuel_mass_flow_kg_per_s.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "transient dual-valve network mass flow is non-finite",
            });
        }

        let chamber = self.chamber.step(
            state.chamber,
            ChamberFeedInput {
                oxidizer_mass_flow_kg_per_s,
                fuel_mass_flow_kg_per_s,
            },
            dt_s,
        )?;

        Ok(TransientDualValveFeedNetworkSnapshot {
            state: TransientDualValveFeedNetworkState {
                chamber: TransientChamberState {
                    pressure_pa: chamber.chamber_pressure_pa,
                },
            },
            chamber,
            oxidizer_mass_flow_kg_per_s,
            fuel_mass_flow_kg_per_s,
            valve_commands,
            validation: ValidationStatus::ValidatedToy,
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

    fn leg(area_m2: f64) -> ValveFeedLegConfig {
        ValveFeedLegConfig {
            tank_pressure_pa: 4.0e6,
            propellant_density_kg_m3: 810.0,
            valve_area_m2: area_m2,
            valve_discharge_coefficient: 0.72,
        }
    }

    fn config() -> TransientDualValveFeedNetworkConfig {
        TransientDualValveFeedNetworkConfig {
            oxidizer: leg(8.0e-5),
            fuel: leg(4.0e-5),
            chamber: TransientChamberConfig {
                chamber_volume_m3: 0.08,
                gas_temperature_k: 3_400.0,
                gas_constant_j_per_kg_k: 360.0,
                throat_area_m2: 1.2e-4,
                c_star_m_s: 1_600.0,
            },
        }
    }

    fn state(pressure_pa: f64) -> TransientDualValveFeedNetworkState {
        TransientDualValveFeedNetworkState {
            chamber: TransientChamberState { pressure_pa },
        }
    }

    fn open() -> ValveCommandPair {
        ValveCommandPair {
            oxidizer_open_fraction: 1.0,
            fuel_open_fraction: 1.0,
        }
    }

    #[test]
    fn transient_dual_valve_network_builds_pressure_and_reports_mr() {
        let network = TransientDualValveFeedNetwork::new(config()).unwrap();
        let snapshot = network.step(state(0.0), open(), 0.05).unwrap();

        assert!(snapshot.chamber.chamber_pressure_pa > 0.0);
        assert!(snapshot.oxidizer_mass_flow_kg_per_s > snapshot.fuel_mass_flow_kg_per_s);
        assert_abs_diff_eq!(
            snapshot.chamber.mixture_ratio.unwrap(),
            2.0,
            epsilon = 1.0e-12
        );
        assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
    }

    #[test]
    fn transient_dual_valve_network_pressure_decays_with_closed_valves() {
        let network = TransientDualValveFeedNetwork::new(config()).unwrap();
        let snapshot = network
            .step(
                state(3.0e6),
                ValveCommandPair {
                    oxidizer_open_fraction: 0.0,
                    fuel_open_fraction: 0.0,
                },
                0.25,
            )
            .unwrap();

        assert!(snapshot.chamber.chamber_pressure_pa < 3.0e6);
        assert_eq!(
            snapshot.oxidizer_mass_flow_kg_per_s.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            snapshot.fuel_mass_flow_kg_per_s.to_bits(),
            0.0_f64.to_bits()
        );
        assert!(snapshot.chamber.outlet_mass_flow_kg_per_s > 0.0);
    }

    #[test]
    fn transient_dual_valve_network_holds_state_for_zero_dt() {
        let network = TransientDualValveFeedNetwork::new(config()).unwrap();
        let snapshot = network.step(state(2.0e6), open(), 0.0).unwrap();

        assert_eq!(
            snapshot.chamber.chamber_pressure_pa.to_bits(),
            2.0e6_f64.to_bits()
        );
        assert_eq!(
            snapshot.chamber.pressure_rate_pa_per_s.to_bits(),
            0.0_f64.to_bits()
        );
    }

    #[test]
    fn transient_dual_valve_network_accepts_dynamic_feed_pressures() {
        let network = TransientDualValveFeedNetwork::new(config()).unwrap();
        let state = state(0.0);
        let baseline = network.step(state, open(), 0.01).unwrap();
        let boosted = network
            .step_with_feed_pressures(
                state,
                open(),
                FeedLegPressures {
                    oxidizer_pressure_pa: 8.0e6,
                    fuel_pressure_pa: 4.0e6,
                },
                0.01,
            )
            .unwrap();

        assert!(boosted.oxidizer_mass_flow_kg_per_s > baseline.oxidizer_mass_flow_kg_per_s);
        assert!(boosted.chamber.chamber_pressure_pa > baseline.chamber.chamber_pressure_pa);
    }

    #[test]
    fn transient_dual_valve_network_matches_provenance_tolerance_table() {
        let data: toml::Value = toml::from_str(include_str!(
            "../../../data/feed_system/generic-transient-chamber-v1.toml"
        ))
        .unwrap();
        assert_eq!(
            table("openbmp", &data)["transient_feed_network"]
                .as_integer()
                .unwrap(),
            1
        );
        let network_config = table("network_config", &data);
        let chamber_config = table("chamber_config", &data);
        let network = TransientDualValveFeedNetwork::new(TransientDualValveFeedNetworkConfig {
            oxidizer: leg_from_table(subtable(network_config, "oxidizer")),
            fuel: leg_from_table(subtable(network_config, "fuel")),
            chamber: TransientChamberConfig {
                chamber_volume_m3: float(chamber_config, "chamber_volume_m3"),
                gas_temperature_k: float(chamber_config, "gas_temperature_k"),
                gas_constant_j_per_kg_k: float(chamber_config, "gas_constant_j_per_kg_k"),
                throat_area_m2: float(chamber_config, "throat_area_m2"),
                c_star_m_s: float(chamber_config, "c_star_m_s"),
            },
        })
        .unwrap();
        let tolerances = table("tolerances", &data);

        for case in data
            .get("network_case")
            .and_then(toml::Value::as_array)
            .expect("network_case array")
        {
            let case = case.as_table().unwrap();
            let name = string(case, "name");
            let commands = ValveCommandPair {
                oxidizer_open_fraction: float(case, "oxidizer_open_fraction"),
                fuel_open_fraction: float(case, "fuel_open_fraction"),
            };
            let snapshot = network
                .step_with_feed_pressures(
                    state(float(case, "initial_pressure_pa")),
                    commands,
                    FeedLegPressures {
                        oxidizer_pressure_pa: float(case, "oxidizer_feed_pressure_pa"),
                        fuel_pressure_pa: float(case, "fuel_feed_pressure_pa"),
                    },
                    float(case, "dt_s"),
                )
                .unwrap();

            assert_abs_diff_eq!(
                snapshot.oxidizer_mass_flow_kg_per_s,
                float(case, "expected_oxidizer_mass_flow_kg_per_s"),
                epsilon = float(tolerances, "absolute_mass_flow_kg_per_s")
            );
            assert_abs_diff_eq!(
                snapshot.fuel_mass_flow_kg_per_s,
                float(case, "expected_fuel_mass_flow_kg_per_s"),
                epsilon = float(tolerances, "absolute_mass_flow_kg_per_s")
            );
            assert_abs_diff_eq!(
                snapshot.state.chamber.pressure_pa,
                float(case, "expected_chamber_pressure_pa"),
                epsilon = float(tolerances, "absolute_pressure_pa")
            );
            assert_abs_diff_eq!(
                snapshot.chamber.chamber_pressure_pa,
                float(case, "expected_chamber_pressure_pa"),
                epsilon = float(tolerances, "absolute_pressure_pa")
            );
            assert_abs_diff_eq!(
                snapshot.chamber.pressure_rate_pa_per_s,
                float(case, "expected_pressure_rate_pa_per_s"),
                epsilon = float(tolerances, "absolute_pressure_rate_pa_per_s")
            );
            assert_abs_diff_eq!(
                snapshot.chamber.inlet_mass_flow_kg_per_s,
                float(case, "expected_inlet_mass_flow_kg_per_s"),
                epsilon = float(tolerances, "absolute_mass_flow_kg_per_s")
            );
            assert_abs_diff_eq!(
                snapshot.chamber.outlet_mass_flow_kg_per_s,
                float(case, "expected_outlet_mass_flow_kg_per_s"),
                epsilon = float(tolerances, "absolute_mass_flow_kg_per_s")
            );
            assert_eq!(snapshot.valve_commands, commands);
            assert_optional_float_eq(
                snapshot.chamber.mixture_ratio,
                optional_float(case, "expected_mixture_ratio"),
                float(tolerances, "absolute_mixture_ratio"),
                name,
            );
            assert_eq!(
                string(case, "expected_validation"),
                "validated-toy",
                "case {name}"
            );
            assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
            assert_eq!(snapshot.chamber.validation, ValidationStatus::ValidatedToy);
        }
    }

    #[test]
    fn gfssp_pressurization_example10_fixture_carries_public_trends() {
        let data: toml::Value = toml::from_str(include_str!(
            "../../../data/feed_system/gfssp-propellant-tank-pressurization-example10-v1.toml"
        ))
        .unwrap();
        assert_eq!(
            table("openbmp", &data)["feed_system_gfssp_pressurization_example10"]
                .as_integer()
                .unwrap(),
            1
        );
        assert_eq!(
            string(table("meta", &data), "validation"),
            "research-digitized"
        );

        let case = table("case", &data);
        assert_abs_diff_eq!(float(case, "run_duration_s"), 200.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(float(case, "time_step_s"), 0.1, epsilon = 1.0e-12);
        assert_eq!(string(case, "pressurant"), "helium");
        assert_eq!(string(case, "propellant"), "oxygen");
        assert_abs_diff_eq!(
            float(case, "pressurant_inlet_pressure_pa"),
            655_001.942_850_96,
            epsilon = 1.0e-9
        );
        assert_abs_diff_eq!(
            float(case, "initial_ullage_pressure_pa"),
            461_948.738_642_256,
            epsilon = 1.0e-9
        );
        assert_abs_diff_eq!(
            float(case, "initial_ullage_volume_m3"),
            0.707_921_164_8,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            float(case, "initial_propellant_volume_m3"),
            13.450_502_131_2,
            epsilon = 1.0e-12
        );

        let pressure_trend = table("pressure_trend", &data);
        assert_eq!(string(pressure_trend, "figure"), "136");
        assert!(boolean(pressure_trend, "ullage_initial_rise"));
        assert!(boolean(
            pressure_trend,
            "ullage_slow_decline_after_initial_rise"
        ));
        assert!(boolean(
            pressure_trend,
            "tank_bottom_initially_above_ullage"
        ));
        assert!(boolean(
            pressure_trend,
            "tank_bottom_converges_toward_ullage"
        ));
        assert!(boolean(
            pressure_trend,
            "gravitational_head_decreases_as_propellant_drains"
        ));

        let temperature_trend = table("temperature_trend", &data);
        assert_eq!(string(temperature_trend, "figure"), "137");
        assert_abs_diff_eq!(
            float(temperature_trend, "ullage_rise_first_60_s_deg_f"),
            192.0,
            epsilon = 1.0e-12
        );
        assert!(boolean(
            temperature_trend,
            "ullage_declines_after_first_60_s"
        ));
        assert_abs_diff_eq!(
            float(temperature_trend, "tank_wall_rise_over_run_deg_f"),
            32.0,
            epsilon = 1.0e-12
        );
        assert!(boolean(
            temperature_trend,
            "tank_wall_warming_damped_by_propellant_heat_sink"
        ));

        let digitization = table("pressure_digitization", &data);
        assert_eq!(string(digitization, "figure"), "136");
        assert_abs_diff_eq!(float(digitization, "render_dpi"), 300.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(
            float(digitization, "absolute_pressure_uncertainty_psia"),
            0.75,
            epsilon = 1.0e-12
        );

        let psi_to_pa = float(table("conversion", &data), "psi_to_pa");
        let samples = data
            .get("pressure_sample")
            .and_then(toml::Value::as_array)
            .expect("pressure_sample array");
        assert_eq!(samples.len(), 12);
        let mut reference_samples = Vec::with_capacity(samples.len());

        let mut previous_time_s = -1.0;
        let mut previous_tank_bottom_psia = f64::INFINITY;
        let mut previous_ullage_psia = f64::INFINITY;
        let mut previous_head_psia = f64::INFINITY;
        for (index, sample) in samples.iter().enumerate() {
            let sample = sample.as_table().unwrap();
            let time_s = float(sample, "time_s");
            let tank_bottom_psia = float(sample, "tank_bottom_pressure_psia");
            let ullage_psia = float(sample, "ullage_pressure_psia");
            assert!(time_s > previous_time_s);
            assert!(tank_bottom_psia >= ullage_psia);
            assert_abs_diff_eq!(
                float(sample, "tank_bottom_pressure_pa"),
                tank_bottom_psia * psi_to_pa,
                epsilon = 1.0e-6
            );
            assert_abs_diff_eq!(
                float(sample, "ullage_pressure_pa"),
                ullage_psia * psi_to_pa,
                epsilon = 1.0e-6
            );
            reference_samples.push(
                PressurizationPressureSample::new(
                    time_s,
                    float(sample, "tank_bottom_pressure_pa"),
                    float(sample, "ullage_pressure_pa"),
                )
                .unwrap(),
            );
            if index == 0 {
                assert_eq!(string(sample, "source"), "manual_setup");
            } else {
                assert_eq!(string(sample, "source"), "figure_136_digitized");
                let head_psia = tank_bottom_psia - ullage_psia;
                if index > 1 {
                    assert!(tank_bottom_psia <= previous_tank_bottom_psia);
                    assert!(ullage_psia <= previous_ullage_psia);
                    assert!(head_psia <= previous_head_psia);
                }
                previous_head_psia = head_psia;
            }
            previous_time_s = time_s;
            previous_tank_bottom_psia = tank_bottom_psia;
            previous_ullage_psia = ullage_psia;
        }

        let first = samples[0].as_table().unwrap();
        assert_abs_diff_eq!(
            float(first, "tank_bottom_pressure_psia"),
            74.76,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            float(first, "ullage_pressure_psia"),
            67.0,
            epsilon = 1.0e-12
        );
        let first_digitized = samples[1].as_table().unwrap();
        assert_abs_diff_eq!(
            float(first_digitized, "tank_bottom_pressure_psia"),
            98.760_240_844_9,
            epsilon = 1.0e-10
        );
        assert_abs_diff_eq!(
            float(first_digitized, "ullage_pressure_psia"),
            91.181_998_577_3,
            epsilon = 1.0e-10
        );
        let final_sample = samples.last().unwrap().as_table().unwrap();
        assert_abs_diff_eq!(
            float(final_sample, "tank_bottom_pressure_psia"),
            88.870_221_155_5,
            epsilon = 1.0e-10
        );
        assert_abs_diff_eq!(
            float(final_sample, "ullage_pressure_psia"),
            88.773_072_747,
            epsilon = 1.0e-10
        );

        let residual_gate = table("residual_gate", &data);
        let report = evaluate_pressurization_pressure_residuals(
            &reference_samples,
            &reference_samples,
            float(residual_gate, "absolute_pressure_tolerance_pa"),
            float(residual_gate, "absolute_time_tolerance_s"),
        )
        .unwrap();
        assert_eq!(report.samples().len(), reference_samples.len());
        assert_abs_diff_eq!(
            report.max_abs_pressure_error_pa,
            float(
                residual_gate,
                "expected_reference_max_abs_pressure_error_pa"
            ),
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            report.rms_pressure_error_pa,
            float(residual_gate, "expected_reference_rms_pressure_error_pa"),
            epsilon = 1.0e-12
        );
        assert_eq!(
            report.within_tolerance,
            boolean(residual_gate, "expected_reference_within_tolerance")
        );
        assert_eq!(report.validation, ValidationStatus::Research);

        let reduced_model = table("reduced_model", &data);
        assert_eq!(
            string(reduced_model, "model"),
            "startup_ramp_plus_power_law_ullage_and_hydrostatic_head"
        );
        assert_eq!(string(reduced_model, "validation"), "research-calibrated");
        for (psia_key, pa_key) in [
            (
                "initial_tank_bottom_pressure_psia",
                "initial_tank_bottom_pressure_pa",
            ),
            ("initial_ullage_pressure_psia", "initial_ullage_pressure_pa"),
            (
                "startup_tank_bottom_pressure_psia",
                "startup_tank_bottom_pressure_pa",
            ),
            ("startup_ullage_pressure_psia", "startup_ullage_pressure_pa"),
            (
                "final_tank_bottom_pressure_psia",
                "final_tank_bottom_pressure_pa",
            ),
            ("final_ullage_pressure_psia", "final_ullage_pressure_pa"),
        ] {
            assert_abs_diff_eq!(
                float(reduced_model, pa_key),
                float(reduced_model, psia_key) * psi_to_pa,
                epsilon = 1.0e-6
            );
        }
        let model =
            ReducedPressurizationPressureModel::new(ReducedPressurizationPressureModelConfig {
                initial_time_s: float(reduced_model, "initial_time_s"),
                startup_time_s: float(reduced_model, "startup_time_s"),
                final_time_s: float(reduced_model, "final_time_s"),
                initial_tank_bottom_pressure_pa: float(
                    reduced_model,
                    "initial_tank_bottom_pressure_pa",
                ),
                initial_ullage_pressure_pa: float(reduced_model, "initial_ullage_pressure_pa"),
                startup_tank_bottom_pressure_pa: float(
                    reduced_model,
                    "startup_tank_bottom_pressure_pa",
                ),
                startup_ullage_pressure_pa: float(reduced_model, "startup_ullage_pressure_pa"),
                final_tank_bottom_pressure_pa: float(
                    reduced_model,
                    "final_tank_bottom_pressure_pa",
                ),
                final_ullage_pressure_pa: float(reduced_model, "final_ullage_pressure_pa"),
                ullage_decay_exponent: float(reduced_model, "ullage_decay_exponent"),
                hydrostatic_head_decay_exponent: float(
                    reduced_model,
                    "hydrostatic_head_decay_exponent",
                ),
            })
            .unwrap();
        assert_eq!(model.validation(), ValidationStatus::Research);
        assert_abs_diff_eq!(model.config().startup_time_s, 5.0, epsilon = 1.0e-12);
        let model_samples = reference_samples
            .iter()
            .map(|sample| model.sample_at(sample.time_s).unwrap())
            .collect::<Vec<_>>();
        assert_abs_diff_eq!(
            model_samples[0].tank_bottom_pressure_pa,
            reference_samples[0].tank_bottom_pressure_pa,
            epsilon = 1.0e-9
        );
        assert_abs_diff_eq!(
            model_samples[1].ullage_pressure_pa,
            reference_samples[1].ullage_pressure_pa,
            epsilon = 1.0e-9
        );
        let model_report = evaluate_pressurization_pressure_residuals(
            &reference_samples,
            &model_samples,
            float(residual_gate, "absolute_pressure_tolerance_pa"),
            float(residual_gate, "absolute_time_tolerance_s"),
        )
        .unwrap();
        assert_eq!(
            model_report.within_tolerance,
            boolean(reduced_model, "expected_within_tolerance")
        );
        assert!(model_report.within_tolerance);
        assert!(
            model_report.max_abs_pressure_error_pa
                <= float(reduced_model, "max_abs_pressure_error_bound_pa")
        );
        assert!(
            model_report.rms_pressure_error_pa
                <= float(reduced_model, "rms_pressure_error_bound_pa")
        );
        assert_eq!(model_report.validation, ValidationStatus::Research);

        let finite_volume_model = table("finite_volume_model", &data);
        assert_eq!(
            string(finite_volume_model, "model"),
            "ideal_gas_ullage_finite_volume_with_compressible_pressurant_inflow"
        );
        assert_eq!(
            string(finite_volume_model, "validation"),
            "research-calibrated"
        );
        let fv_config = FiniteVolumePressurizationConfig {
            pressurant_inlet_pressure_pa: float(case, "pressurant_inlet_pressure_pa"),
            pressurant_inlet_temperature_k: float(case, "pressurant_inlet_temperature_k"),
            pressurant_gas_constant_j_per_kg_k: float(
                finite_volume_model,
                "pressurant_gas_constant_j_per_kg_k",
            ),
            pressurant_heat_capacity_ratio: float(
                finite_volume_model,
                "pressurant_heat_capacity_ratio",
            ),
            pressurant_flow_area_m2: float(finite_volume_model, "pressurant_flow_area_m2"),
            pressurant_discharge_coefficient: float(
                finite_volume_model,
                "pressurant_discharge_coefficient",
            ),
            ullage_heat_transfer_w_per_k: float(
                finite_volume_model,
                "ullage_heat_transfer_w_per_k",
            ),
            tank_wall_temperature_k: float(finite_volume_model, "tank_wall_temperature_k"),
            propellant_density_kg_m3: float(finite_volume_model, "propellant_density_kg_m3"),
            tank_cross_section_area_m2: float(finite_volume_model, "tank_cross_section_area_m2"),
            propellant_outflow_volume_rate_m3_s: float(
                finite_volume_model,
                "propellant_outflow_volume_rate_m3_s",
            ),
            gravity_m_s2: float(finite_volume_model, "gravity_m_s2"),
        };
        let finite_volume = FiniteVolumePressurizationModel::new(fv_config).unwrap();
        assert_eq!(finite_volume.validation(), ValidationStatus::Research);
        let mut fv_state = FiniteVolumePressurizationState::from_ullage_pressure(
            fv_config,
            0.0,
            float(case, "initial_ullage_pressure_pa"),
            float(case, "initial_ullage_temperature_k"),
            float(case, "initial_ullage_volume_m3"),
            float(case, "initial_propellant_volume_m3"),
        )
        .unwrap();
        assert_abs_diff_eq!(
            fv_state.ullage_gas_mass_kg,
            float(finite_volume_model, "expected_initial_ullage_gas_mass_kg"),
            epsilon = 1.0e-12
        );
        let mut fv_model_samples = Vec::with_capacity(reference_samples.len());
        fv_model_samples.push(finite_volume.step(fv_state, 0.0).unwrap().pressure_sample);
        let dt_s = float(case, "time_step_s");
        for reference_sample in reference_samples.iter().skip(1) {
            let step_count = ((reference_sample.time_s - fv_state.time_s) / dt_s).round() as usize;
            let mut snapshot = None;
            for _ in 0..step_count {
                let next = finite_volume.step(fv_state, dt_s).unwrap();
                fv_state = next.state;
                snapshot = Some(next);
            }
            fv_model_samples.push(snapshot.unwrap().pressure_sample);
        }
        assert_abs_diff_eq!(
            fv_state.ullage_temperature_k,
            float(finite_volume_model, "expected_final_ullage_temperature_k"),
            epsilon = 1.0e-9
        );
        assert_abs_diff_eq!(
            fv_state.propellant_volume_m3,
            float(finite_volume_model, "expected_final_propellant_volume_m3"),
            epsilon = 1.0e-12
        );
        let fv_report = evaluate_pressurization_pressure_residuals(
            &reference_samples,
            &fv_model_samples,
            float(residual_gate, "absolute_pressure_tolerance_pa"),
            float(residual_gate, "absolute_time_tolerance_s"),
        )
        .unwrap();
        assert_eq!(
            fv_report.within_tolerance,
            boolean(finite_volume_model, "expected_within_tolerance")
        );
        assert!(fv_report.within_tolerance);
        assert!(
            fv_report.max_abs_pressure_error_pa
                <= float(finite_volume_model, "max_abs_pressure_error_bound_pa")
        );
        assert!(
            fv_report.rms_pressure_error_pa
                <= float(finite_volume_model, "rms_pressure_error_bound_pa")
        );
        assert_eq!(fv_report.validation, ValidationStatus::Research);

        let mut shifted_samples = reference_samples.clone();
        shifted_samples[2].tank_bottom_pressure_pa +=
            1.1 * float(residual_gate, "absolute_pressure_tolerance_pa");
        let shifted_report = evaluate_pressurization_pressure_residuals(
            &reference_samples,
            &shifted_samples,
            float(residual_gate, "absolute_pressure_tolerance_pa"),
            float(residual_gate, "absolute_time_tolerance_s"),
        )
        .unwrap();
        assert!(!shifted_report.within_tolerance);
        assert!(
            shifted_report.max_abs_tank_bottom_pressure_error_pa
                > float(residual_gate, "absolute_pressure_tolerance_pa")
        );

        let mut bad_time_samples = reference_samples.clone();
        bad_time_samples[1].time_s += 2.0 * float(residual_gate, "absolute_time_tolerance_s");
        assert!(matches!(
            evaluate_pressurization_pressure_residuals(
                &reference_samples,
                &bad_time_samples,
                float(residual_gate, "absolute_pressure_tolerance_pa"),
                float(residual_gate, "absolute_time_tolerance_s"),
            ),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        let limits = table("limits", &data);
        assert!(boolean(limits, "quantitative_time_history_claimed"));
        assert!(boolean(limits, "digitized_curve_claimed"));
        assert!(boolean(limits, "residual_gate_claimed"));
        assert!(boolean(limits, "reduced_model_residual_closure_claimed"));
        assert!(boolean(limits, "physics_model_residual_closure_claimed"));
    }

    #[test]
    fn reduced_pressurization_pressure_model_rejects_invalid_values() {
        let config = ReducedPressurizationPressureModelConfig {
            initial_time_s: 0.0,
            startup_time_s: 5.0,
            final_time_s: 200.0,
            initial_tank_bottom_pressure_pa: 515_452.055_236_84,
            initial_ullage_pressure_pa: 461_948.738_642_256,
            startup_tank_bottom_pressure_pa: 680_927.890_840_607,
            startup_ullage_pressure_pa: 628_677.749_696_25,
            final_tank_bottom_pressure_pa: 612_738.605_457_366,
            final_ullage_pressure_pa: 612_068.790_758_559,
            ullage_decay_exponent: 1.904_59,
            hydrostatic_head_decay_exponent: 1.025_88,
        };
        let model = ReducedPressurizationPressureModel::new(config).unwrap();
        assert!(matches!(
            model.sample_at(config.final_time_s + 1.0),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        let mut bad = config;
        bad.final_time_s = bad.startup_time_s;
        assert!(matches!(
            ReducedPressurizationPressureModel::new(bad),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        let mut bad = config;
        bad.ullage_decay_exponent = 0.0;
        assert!(matches!(
            ReducedPressurizationPressureModel::new(bad),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        let mut bad = config;
        bad.final_tank_bottom_pressure_pa = bad.final_ullage_pressure_pa - 1.0;
        assert!(matches!(
            ReducedPressurizationPressureModel::new(bad),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn finite_volume_pressurization_model_rejects_invalid_values() {
        let config = FiniteVolumePressurizationConfig {
            pressurant_inlet_pressure_pa: 655_001.942_850_96,
            pressurant_inlet_temperature_k: 322.038_888_888_888_9,
            pressurant_gas_constant_j_per_kg_k: 2077.1,
            pressurant_heat_capacity_ratio: 1.666,
            pressurant_flow_area_m2: 0.000_217_742_262_042_294_54,
            pressurant_discharge_coefficient: 1.0,
            ullage_heat_transfer_w_per_k: 136.779_148_558_134_84,
            tank_wall_temperature_k: 338.668_236_072_483_5,
            propellant_density_kg_m3: 1141.0,
            tank_cross_section_area_m2: 2.812_963_606_977_273,
            propellant_outflow_volume_rate_m3_s: 0.066_410_568_080_544_4,
            gravity_m_s2: 9.80665,
        };
        let model = FiniteVolumePressurizationModel::new(config).unwrap();
        let state = FiniteVolumePressurizationState::from_ullage_pressure(
            config,
            0.0,
            461_948.738_642_256,
            108.705_555_555_555_52,
            0.707_921_164_8,
            13.450_502_131_2,
        )
        .unwrap();
        let snapshot = model.step(state, 0.1).unwrap();
        assert!(snapshot.pressurant_mass_flow_kg_per_s > 0.0);
        assert!(snapshot.tank_bottom_pressure_pa > snapshot.ullage_pressure_pa);

        let mut bad_config = config;
        bad_config.pressurant_heat_capacity_ratio = 1.0;
        assert!(matches!(
            FiniteVolumePressurizationModel::new(bad_config),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        let mut bad_config = config;
        bad_config.pressurant_discharge_coefficient = 1.1;
        assert!(matches!(
            FiniteVolumePressurizationModel::new(bad_config),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        assert!(matches!(
            model.step(state, f64::NAN),
            Err(FeedSystemError::NonFinite { .. })
        ));

        let mut bad_state = state;
        bad_state.ullage_volume_m3 = 0.0;
        assert!(matches!(
            model.step(bad_state, 0.1),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn transient_dual_valve_network_rejects_invalid_values() {
        let mut bad = config();
        bad.oxidizer.valve_discharge_coefficient = 1.2;
        assert!(matches!(
            TransientDualValveFeedNetwork::new(bad),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        let network = TransientDualValveFeedNetwork::new(config()).unwrap();
        assert!(matches!(
            network.step(
                state(1.0e6),
                ValveCommandPair {
                    oxidizer_open_fraction: 1.1,
                    fuel_open_fraction: 1.0,
                },
                0.1,
            ),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    fn leg_from_table(table: &Table) -> ValveFeedLegConfig {
        ValveFeedLegConfig {
            tank_pressure_pa: float(table, "tank_pressure_pa"),
            propellant_density_kg_m3: float(table, "propellant_density_kg_m3"),
            valve_area_m2: float(table, "valve_area_m2"),
            valve_discharge_coefficient: float(table, "valve_discharge_coefficient"),
        }
    }

    fn table<'a>(key: &str, value: &'a toml::Value) -> &'a Table {
        value
            .get(key)
            .and_then(toml::Value::as_table)
            .unwrap_or_else(|| panic!("missing table {key}"))
    }

    fn subtable<'a>(table: &'a Table, key: &str) -> &'a Table {
        table
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

    fn optional_float(table: &Table, key: &str) -> Option<f64> {
        let value = table
            .get(key)
            .unwrap_or_else(|| panic!("missing optional float {key}"));
        if value.as_str() == Some("none") {
            return None;
        }
        Some(
            value
                .as_float()
                .unwrap_or_else(|| panic!("missing float or none marker {key}")),
        )
    }

    fn assert_optional_float_eq(
        actual: Option<f64>,
        expected: Option<f64>,
        epsilon: f64,
        name: &str,
    ) {
        match (actual, expected) {
            (Some(actual), Some(expected)) => {
                assert_abs_diff_eq!(actual, expected, epsilon = epsilon);
            }
            (None, None) => {}
            _ => panic!(
                "mixture ratio mismatch for case {name}: actual {actual:?}, expected {expected:?}"
            ),
        }
    }

    fn boolean(table: &Table, key: &str) -> bool {
        table
            .get(key)
            .and_then(toml::Value::as_bool)
            .unwrap_or_else(|| panic!("missing bool {key}"))
    }

    fn string<'a>(table: &'a Table, key: &str) -> &'a str {
        table
            .get(key)
            .and_then(toml::Value::as_str)
            .unwrap_or_else(|| panic!("missing string {key}"))
    }
}
