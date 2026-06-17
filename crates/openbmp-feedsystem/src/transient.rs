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

    fn string<'a>(table: &'a Table, key: &str) -> &'a str {
        table
            .get(key)
            .and_then(toml::Value::as_str)
            .unwrap_or_else(|| panic!("missing string {key}"))
    }
}
