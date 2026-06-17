//! Deterministic feed-network model primitives.

use openbmp_core::ValidationStatus;

use crate::error::FeedSystemError;

const BISECTION_STEPS: usize = 96;
const MASS_FLOW_RESIDUAL_REL_TOL: f64 = 1.0e-12;

/// Feed-network command inputs for a single solve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedCommand {
    /// Valve opening fraction in `[0, 1]`.
    pub valve_open_fraction: f64,
}

impl FeedCommand {
    /// Fully open valve command.
    #[must_use]
    pub const fn fully_open() -> Self {
        Self {
            valve_open_fraction: 1.0,
        }
    }

    fn require_valid(self) -> Result<Self, FeedSystemError> {
        if !self.valve_open_fraction.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "valve open fraction is not finite",
            });
        }
        if !(0.0..=1.0).contains(&self.valve_open_fraction) {
            return Err(FeedSystemError::InvalidParameter {
                reason: "valve open fraction must lie in [0, 1]",
            });
        }
        Ok(self)
    }
}

/// Feed-network state returned by a solve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedNetworkSnapshot {
    /// Equilibrium chamber pressure in Pa.
    pub chamber_pressure_pa: f64,
    /// Inflow/outflow mass flow at equilibrium in kg/s.
    pub mass_flow_kg_per_s: f64,
    /// Absolute mass-flow residual `|m_dot_in - m_dot_out|`, in kg/s.
    pub residual_kg_per_s: f64,
    /// Validation posture of the model that produced this snapshot.
    pub validation: ValidationStatus,
}

/// Deterministic feed-network solver boundary.
pub trait FeedNetwork {
    /// Solve the network for one command/state sample.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid commands or non-convergent
    /// algebraic solves.
    fn solve(&self, command: FeedCommand) -> Result<FeedNetworkSnapshot, FeedSystemError>;

    /// Validation-status declaration for downstream adapters.
    fn validation(&self) -> ValidationStatus;
}

/// Configuration for a reduced tank-valve-chamber ladder.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TankValveChamberConfig {
    /// Upstream tank/feed pressure in Pa.
    pub tank_pressure_pa: f64,
    /// Propellant density in kg/m^3 for the incompressible valve relation.
    pub propellant_density_kg_m3: f64,
    /// Full-open valve flow area in m^2.
    pub valve_area_m2: f64,
    /// Valve discharge coefficient. Must lie in `(0, 1]`.
    pub valve_discharge_coefficient: f64,
    /// Chamber throat area in m^2.
    pub throat_area_m2: f64,
    /// Characteristic velocity in m/s for the choked throat closure.
    pub c_star_m_s: f64,
}

impl TankValveChamberConfig {
    /// Validate the reduced feed-network parameters.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite, non-positive, or
    /// out-of-envelope values.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        for value in [
            self.tank_pressure_pa,
            self.propellant_density_kg_m3,
            self.valve_area_m2,
            self.valve_discharge_coefficient,
            self.throat_area_m2,
            self.c_star_m_s,
        ] {
            if !value.is_finite() {
                return Err(FeedSystemError::NonFinite {
                    reason: "tank-valve-chamber config contains NaN or infinity",
                });
            }
        }
        if self.tank_pressure_pa <= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "tank pressure must be positive",
            });
        }
        if self.propellant_density_kg_m3 <= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "propellant density must be positive",
            });
        }
        if self.valve_area_m2 <= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "valve area must be positive",
            });
        }
        if self.valve_discharge_coefficient <= 0.0 || self.valve_discharge_coefficient > 1.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "valve discharge coefficient must lie in (0, 1]",
            });
        }
        if self.throat_area_m2 <= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "throat area must be positive",
            });
        }
        if self.c_star_m_s <= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "characteristic velocity must be positive",
            });
        }
        Ok(())
    }
}

/// Reduced single-fluid tank-valve-chamber feed network.
///
/// The model solves `m_dot_valve(pc) = m_dot_throat(pc)` with:
///
/// - `m_dot_valve = Cd A_v sqrt(2 rho max(p_tank - pc, 0))`
/// - `m_dot_throat = pc A_t / c*`
///
/// The residual is monotone on `pc in [0, p_tank]`, so a fixed bisection solve
/// is deterministic and fail-closed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TankValveChamberNetwork {
    config: TankValveChamberConfig,
}

impl TankValveChamberNetwork {
    /// Construct a validated reduced feed network.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the configuration is invalid.
    pub fn new(config: TankValveChamberConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self { config })
    }

    /// Borrow the validated configuration.
    #[must_use]
    pub const fn config(&self) -> TankValveChamberConfig {
        self.config
    }

    fn valve_mass_flow_kg_per_s(&self, chamber_pressure_pa: f64, open_fraction: f64) -> f64 {
        let delta_p_pa = (self.config.tank_pressure_pa - chamber_pressure_pa).max(0.0);
        let effective_area_m2 = self.config.valve_area_m2 * open_fraction;
        self.config.valve_discharge_coefficient
            * effective_area_m2
            * (2.0 * self.config.propellant_density_kg_m3 * delta_p_pa).sqrt()
    }

    fn throat_mass_flow_kg_per_s(&self, chamber_pressure_pa: f64) -> f64 {
        chamber_pressure_pa * self.config.throat_area_m2 / self.config.c_star_m_s
    }

    fn residual_kg_per_s(&self, chamber_pressure_pa: f64, open_fraction: f64) -> f64 {
        self.valve_mass_flow_kg_per_s(chamber_pressure_pa, open_fraction)
            - self.throat_mass_flow_kg_per_s(chamber_pressure_pa)
    }
}

impl FeedNetwork for TankValveChamberNetwork {
    fn solve(&self, command: FeedCommand) -> Result<FeedNetworkSnapshot, FeedSystemError> {
        let command = command.require_valid()?;
        if command.valve_open_fraction == 0.0 {
            return Ok(FeedNetworkSnapshot {
                chamber_pressure_pa: 0.0,
                mass_flow_kg_per_s: 0.0,
                residual_kg_per_s: 0.0,
                validation: self.validation(),
            });
        }

        let mut low_pa = 0.0;
        let mut high_pa = self.config.tank_pressure_pa;
        for _ in 0..BISECTION_STEPS {
            let mid_pa = 0.5 * (low_pa + high_pa);
            let residual = self.residual_kg_per_s(mid_pa, command.valve_open_fraction);
            if !residual.is_finite() {
                return Err(FeedSystemError::NonFinite {
                    reason: "feed-network residual became non-finite",
                });
            }
            if residual > 0.0 {
                low_pa = mid_pa;
            } else {
                high_pa = mid_pa;
            }
        }

        let chamber_pressure_pa = 0.5 * (low_pa + high_pa);
        let inflow =
            self.valve_mass_flow_kg_per_s(chamber_pressure_pa, command.valve_open_fraction);
        let outflow = self.throat_mass_flow_kg_per_s(chamber_pressure_pa);
        let residual_kg_per_s = (inflow - outflow).abs();
        let mass_flow_kg_per_s = 0.5 * (inflow + outflow);
        let tolerance = MASS_FLOW_RESIDUAL_REL_TOL * mass_flow_kg_per_s.max(1.0);
        if residual_kg_per_s > tolerance {
            return Err(FeedSystemError::NonConverged {
                reason: "tank-valve-chamber residual exceeded tolerance after fixed bisection",
            });
        }

        Ok(FeedNetworkSnapshot {
            chamber_pressure_pa,
            mass_flow_kg_per_s,
            residual_kg_per_s,
            validation: self.validation(),
        })
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::ValidatedToy
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use approx::assert_abs_diff_eq;
    use toml::value::Table;

    use super::*;

    fn config() -> TankValveChamberConfig {
        TankValveChamberConfig {
            tank_pressure_pa: 4.0e6,
            propellant_density_kg_m3: 810.0,
            valve_area_m2: 8.0e-5,
            valve_discharge_coefficient: 0.72,
            throat_area_m2: 1.2e-4,
            c_star_m_s: 1_600.0,
        }
    }

    fn analytic_chamber_pressure_pa(config: TankValveChamberConfig, open_fraction: f64) -> f64 {
        let valve_coeff = config.valve_discharge_coefficient
            * config.valve_area_m2
            * open_fraction
            * (2.0 * config.propellant_density_kg_m3).sqrt();
        let throat_coeff = config.throat_area_m2 / config.c_star_m_s;
        let valve_coeff_sq = valve_coeff * valve_coeff;
        (-valve_coeff_sq
            + (valve_coeff_sq * valve_coeff_sq
                + 4.0 * throat_coeff * throat_coeff * valve_coeff_sq * config.tank_pressure_pa)
                .sqrt())
            / (2.0 * throat_coeff * throat_coeff)
    }

    #[test]
    fn tank_valve_chamber_balances_valve_and_choked_throat() {
        let network = TankValveChamberNetwork::new(config()).unwrap();
        let snapshot = network.solve(FeedCommand::fully_open()).unwrap();
        let expected_pressure = analytic_chamber_pressure_pa(config(), 1.0);

        assert_abs_diff_eq!(
            snapshot.chamber_pressure_pa,
            expected_pressure,
            epsilon = 1.0e-7
        );
        assert_abs_diff_eq!(snapshot.residual_kg_per_s, 0.0, epsilon = 1.0e-12);
        assert!(snapshot.mass_flow_kg_per_s > 0.0);
        assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
    }

    #[test]
    fn lower_valve_command_reduces_pressure_and_flow() {
        let network = TankValveChamberNetwork::new(config()).unwrap();
        let full = network.solve(FeedCommand::fully_open()).unwrap();
        let half = network
            .solve(FeedCommand {
                valve_open_fraction: 0.5,
            })
            .unwrap();

        assert!(half.chamber_pressure_pa < full.chamber_pressure_pa);
        assert!(half.mass_flow_kg_per_s < full.mass_flow_kg_per_s);
    }

    #[test]
    fn closed_valve_returns_zero_flow() {
        let network = TankValveChamberNetwork::new(config()).unwrap();
        let snapshot = network
            .solve(FeedCommand {
                valve_open_fraction: 0.0,
            })
            .unwrap();

        assert_eq!(snapshot.chamber_pressure_pa.to_bits(), 0.0_f64.to_bits());
        assert_eq!(snapshot.mass_flow_kg_per_s.to_bits(), 0.0_f64.to_bits());
        assert_eq!(snapshot.residual_kg_per_s.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn invalid_config_fails_closed() {
        let mut bad = config();
        bad.valve_discharge_coefficient = 1.2;

        assert!(matches!(
            TankValveChamberNetwork::new(bad),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn invalid_command_fails_closed() {
        let network = TankValveChamberNetwork::new(config()).unwrap();

        assert!(matches!(
            network.solve(FeedCommand {
                valve_open_fraction: 1.1,
            }),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn steady_feed_network_matches_provenance_tolerance_table() {
        let data: toml::Value = toml::from_str(include_str!(
            "../../../data/feed_system/generic-steady-feed-network-v1.toml"
        ))
        .unwrap();
        assert_eq!(
            table("openbmp", &data)["feed_network_steady"]
                .as_integer()
                .unwrap(),
            1
        );
        let network = TankValveChamberNetwork::new(TankValveChamberConfig {
            tank_pressure_pa: float(table("reduced_config", &data), "tank_pressure_pa"),
            propellant_density_kg_m3: float(
                table("reduced_config", &data),
                "propellant_density_kg_m3",
            ),
            valve_area_m2: float(table("reduced_config", &data), "valve_area_m2"),
            valve_discharge_coefficient: float(
                table("reduced_config", &data),
                "valve_discharge_coefficient",
            ),
            throat_area_m2: float(table("reduced_config", &data), "throat_area_m2"),
            c_star_m_s: float(table("reduced_config", &data), "c_star_m_s"),
        })
        .unwrap();
        let tolerances = table("tolerances", &data);

        for case in data
            .get("reduced_case")
            .and_then(toml::Value::as_array)
            .expect("reduced_case array")
        {
            let case = case.as_table().unwrap();
            let snapshot = network
                .solve(FeedCommand {
                    valve_open_fraction: float(case, "valve_open_fraction"),
                })
                .unwrap();
            let name = string(case, "name");

            assert_abs_diff_eq!(
                snapshot.chamber_pressure_pa,
                float(case, "expected_chamber_pressure_pa"),
                epsilon = float(tolerances, "absolute_pressure_pa")
            );
            assert_abs_diff_eq!(
                snapshot.mass_flow_kg_per_s,
                float(case, "expected_mass_flow_kg_per_s"),
                epsilon = float(tolerances, "absolute_mass_flow_kg_per_s")
            );
            assert_abs_diff_eq!(
                snapshot.residual_kg_per_s,
                float(case, "expected_residual_kg_per_s"),
                epsilon = float(tolerances, "absolute_residual_kg_per_s")
            );
            assert_eq!(
                string(case, "expected_validation"),
                "validated-toy",
                "case {name}"
            );
            assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
        }
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
