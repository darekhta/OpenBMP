//! Lumped transient chamber-pressure model.

use openbmp_core::ValidationStatus;

use crate::error::FeedSystemError;

/// Constant-geometry chamber configuration for transient pressure integration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransientChamberConfig {
    /// Lumped chamber free volume in m^3.
    pub chamber_volume_m3: f64,
    /// Effective chamber gas temperature in K.
    pub gas_temperature_k: f64,
    /// Effective combustion-gas specific gas constant in J/(kg*K).
    pub gas_constant_j_per_kg_k: f64,
    /// Chamber throat area in m^2.
    pub throat_area_m2: f64,
    /// Characteristic velocity in m/s for the choked throat closure.
    pub c_star_m_s: f64,
}

impl TransientChamberConfig {
    /// Validate chamber scalar parameters.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite or non-positive values.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_positive_finite(
            self.chamber_volume_m3,
            "transient chamber volume must be positive",
        )?;
        require_positive_finite(
            self.gas_temperature_k,
            "transient chamber gas temperature must be positive",
        )?;
        require_positive_finite(
            self.gas_constant_j_per_kg_k,
            "transient chamber gas constant must be positive",
        )?;
        require_positive_finite(
            self.throat_area_m2,
            "transient chamber throat area must be positive",
        )?;
        require_positive_finite(self.c_star_m_s, "transient chamber c_star must be positive")
    }

    fn pressure_gain_pa_per_kg(self) -> f64 {
        self.gas_constant_j_per_kg_k * self.gas_temperature_k / self.chamber_volume_m3
    }

    fn throat_coeff_kg_per_s_pa(self) -> f64 {
        self.throat_area_m2 / self.c_star_m_s
    }
}

/// Pressure state for a lumped transient chamber.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransientChamberState {
    /// Chamber pressure in Pa.
    pub pressure_pa: f64,
}

impl TransientChamberState {
    /// Validate chamber state.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite or negative pressure.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_nonnegative_finite(
            self.pressure_pa,
            "transient chamber pressure must be finite and non-negative",
        )
    }
}

/// Feed mass-flow input for one chamber-pressure step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChamberFeedInput {
    /// Oxidizer-side mass flow into the chamber in kg/s.
    pub oxidizer_mass_flow_kg_per_s: f64,
    /// Fuel-side mass flow into the chamber in kg/s.
    pub fuel_mass_flow_kg_per_s: f64,
}

impl ChamberFeedInput {
    /// Build a single-fluid input. The mixture-ratio field in the snapshot will
    /// be `None` because no fuel-side reference flow exists.
    #[must_use]
    pub const fn single_fluid(mass_flow_kg_per_s: f64) -> Self {
        Self {
            oxidizer_mass_flow_kg_per_s: mass_flow_kg_per_s,
            fuel_mass_flow_kg_per_s: 0.0,
        }
    }

    /// Total chamber inlet mass flow in kg/s.
    #[must_use]
    pub fn total_mass_flow_kg_per_s(self) -> f64 {
        self.oxidizer_mass_flow_kg_per_s + self.fuel_mass_flow_kg_per_s
    }

    fn mixture_ratio(self) -> Result<Option<f64>, FeedSystemError> {
        if self.fuel_mass_flow_kg_per_s == 0.0 {
            return Ok(None);
        }
        let ratio = self.oxidizer_mass_flow_kg_per_s / self.fuel_mass_flow_kg_per_s;
        if !ratio.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "transient chamber mixture ratio is non-finite",
            });
        }
        Ok(Some(ratio))
    }

    fn require_valid(self) -> Result<(), FeedSystemError> {
        require_nonnegative_finite(
            self.oxidizer_mass_flow_kg_per_s,
            "oxidizer chamber feed mass flow must be finite and non-negative",
        )?;
        require_nonnegative_finite(
            self.fuel_mass_flow_kg_per_s,
            "fuel chamber feed mass flow must be finite and non-negative",
        )?;
        let total = self.total_mass_flow_kg_per_s();
        if !total.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "total chamber feed mass flow is non-finite",
            });
        }
        Ok(())
    }
}

/// Solved chamber state after one transient step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransientChamberSnapshot {
    /// Chamber pressure at the start of the step in Pa.
    pub previous_pressure_pa: f64,
    /// Chamber pressure at the end of the step in Pa.
    pub chamber_pressure_pa: f64,
    /// Average pressure rate across the step in Pa/s.
    pub pressure_rate_pa_per_s: f64,
    /// Total inlet mass flow in kg/s.
    pub inlet_mass_flow_kg_per_s: f64,
    /// Oxidizer-side inlet mass flow in kg/s.
    pub oxidizer_mass_flow_kg_per_s: f64,
    /// Fuel-side inlet mass flow in kg/s.
    pub fuel_mass_flow_kg_per_s: f64,
    /// Choked throat outflow at the end-of-step pressure in kg/s.
    pub outlet_mass_flow_kg_per_s: f64,
    /// Oxidizer/fuel ratio when a fuel-side flow is present.
    pub mixture_ratio: Option<f64>,
    /// Validation posture of the chamber model.
    pub validation: ValidationStatus,
}

/// Deterministic lumped-volume chamber-pressure integrator.
///
/// The model advances `dp/dt = (R*T/V) * (m_dot_in - p*At/c*)` with the exact
/// constant-input solution for one time step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransientChamber {
    config: TransientChamberConfig,
}

impl TransientChamber {
    /// Construct a validated transient chamber.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the configuration is invalid.
    pub fn new(config: TransientChamberConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self { config })
    }

    /// Borrow the validated chamber configuration.
    #[must_use]
    pub const fn config(&self) -> TransientChamberConfig {
        self.config
    }

    /// Advance the chamber pressure by `dt_s` for a constant feed input.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid state/input/time-step values or
    /// non-finite intermediate pressure values.
    pub fn step(
        &self,
        state: TransientChamberState,
        input: ChamberFeedInput,
        dt_s: f64,
    ) -> Result<TransientChamberSnapshot, FeedSystemError> {
        state.require_valid()?;
        input.require_valid()?;
        require_nonnegative_finite(
            dt_s,
            "transient chamber time step must be finite and non-negative",
        )?;

        let inlet_mass_flow_kg_per_s = input.total_mass_flow_kg_per_s();
        let throat_coeff = self.config.throat_coeff_kg_per_s_pa();
        let steady_pressure_pa = inlet_mass_flow_kg_per_s / throat_coeff;
        if !steady_pressure_pa.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "transient chamber steady pressure is non-finite",
            });
        }

        let decay_rate_per_s = self.config.pressure_gain_pa_per_kg() * throat_coeff;
        let decay_product = decay_rate_per_s * dt_s;
        let decay_factor = if decay_product.is_finite() {
            (-decay_product).exp()
        } else if decay_product.is_sign_positive() {
            0.0
        } else {
            return Err(FeedSystemError::NonFinite {
                reason: "transient chamber decay factor is non-finite",
            });
        };

        let chamber_pressure_pa =
            steady_pressure_pa + (state.pressure_pa - steady_pressure_pa) * decay_factor;
        if !chamber_pressure_pa.is_finite() || chamber_pressure_pa < 0.0 {
            return Err(FeedSystemError::NonFinite {
                reason: "transient chamber pressure update is non-finite",
            });
        }

        let pressure_rate_pa_per_s = if dt_s == 0.0 {
            0.0
        } else {
            (chamber_pressure_pa - state.pressure_pa) / dt_s
        };
        if !pressure_rate_pa_per_s.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "transient chamber pressure rate is non-finite",
            });
        }

        Ok(TransientChamberSnapshot {
            previous_pressure_pa: state.pressure_pa,
            chamber_pressure_pa,
            pressure_rate_pa_per_s,
            inlet_mass_flow_kg_per_s,
            oxidizer_mass_flow_kg_per_s: input.oxidizer_mass_flow_kg_per_s,
            fuel_mass_flow_kg_per_s: input.fuel_mass_flow_kg_per_s,
            outlet_mass_flow_kg_per_s: chamber_pressure_pa * throat_coeff,
            mixture_ratio: input.mixture_ratio()?,
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

    use crate::graph::{
        FeedGraphNode, FeedGraphNodeKind, SteadyFeedGraph, SteadyFeedGraphConfig, ValveBranch,
    };

    use super::*;

    fn chamber() -> TransientChamber {
        TransientChamber::new(TransientChamberConfig {
            chamber_volume_m3: 0.08,
            gas_temperature_k: 3_400.0,
            gas_constant_j_per_kg_k: 360.0,
            throat_area_m2: 1.2e-4,
            c_star_m_s: 1_600.0,
        })
        .unwrap()
    }

    #[test]
    fn transient_chamber_exact_step_matches_closed_form() {
        let chamber = chamber();
        let state = TransientChamberState { pressure_pa: 1.0e6 };
        let input = ChamberFeedInput::single_fluid(0.3);
        let snapshot = chamber.step(state, input, 0.25).unwrap();

        let throat_coeff = 1.2e-4 / 1_600.0;
        let steady_pressure_pa = 0.3 / throat_coeff;
        let decay_rate_per_s = (360.0 * 3_400.0 / 0.08) * throat_coeff;
        let decay_argument: f64 = -decay_rate_per_s * 0.25;
        let expected_pressure =
            steady_pressure_pa + (1.0e6 - steady_pressure_pa) * decay_argument.exp();

        assert_abs_diff_eq!(
            snapshot.chamber_pressure_pa,
            expected_pressure,
            epsilon = 1.0e-8
        );
        assert_abs_diff_eq!(
            snapshot.outlet_mass_flow_kg_per_s,
            snapshot.chamber_pressure_pa * throat_coeff
        );
        assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
    }

    #[test]
    fn transient_chamber_approaches_constant_inflow_equilibrium() {
        let chamber = chamber();
        let mut state = TransientChamberState { pressure_pa: 0.0 };

        for _ in 0..256 {
            let snapshot = chamber
                .step(state, ChamberFeedInput::single_fluid(0.3), 0.1)
                .unwrap();
            state.pressure_pa = snapshot.chamber_pressure_pa;
        }

        let expected_pressure = 0.3 / (1.2e-4 / 1_600.0);
        assert_abs_diff_eq!(state.pressure_pa, expected_pressure, epsilon = 1.0e-6);
    }

    #[test]
    fn transient_chamber_can_hold_graph_equilibrium() {
        let graph = SteadyFeedGraph::new(SteadyFeedGraphConfig::new(
            vec![
                FeedGraphNode {
                    kind: FeedGraphNodeKind::PressureBoundary { pressure_pa: 4.0e6 },
                },
                FeedGraphNode {
                    kind: FeedGraphNodeKind::Chamber {
                        initial_pressure_pa: 3.0e6,
                        throat_area_m2: 1.2e-4,
                        c_star_m_s: 1_600.0,
                    },
                },
            ],
            vec![ValveBranch {
                from: 0,
                to: 1,
                area_m2: 8.0e-5,
                discharge_coefficient: 0.72,
                density_kg_m3: 810.0,
                open_fraction: 1.0,
            }],
        ))
        .unwrap();
        let graph_snapshot = graph.solve().unwrap();
        let chamber = chamber();
        let pressure_pa = graph_snapshot.node_pressures_pa[1];
        let feed_flow = graph_snapshot.valve_branch_mass_flow_kg_per_s[0];

        let snapshot = chamber
            .step(
                TransientChamberState { pressure_pa },
                ChamberFeedInput::single_fluid(feed_flow),
                0.5,
            )
            .unwrap();

        assert_abs_diff_eq!(snapshot.chamber_pressure_pa, pressure_pa, epsilon = 1.0e-2);
        assert_abs_diff_eq!(
            snapshot.outlet_mass_flow_kg_per_s,
            feed_flow,
            epsilon = 1.0e-9
        );
        assert_eq!(snapshot.mixture_ratio, None);
    }

    #[test]
    fn transient_chamber_reports_mixture_ratio() {
        let snapshot = chamber()
            .step(
                TransientChamberState { pressure_pa: 2.0e6 },
                ChamberFeedInput {
                    oxidizer_mass_flow_kg_per_s: 2.4,
                    fuel_mass_flow_kg_per_s: 0.8,
                },
                0.02,
            )
            .unwrap();

        assert_abs_diff_eq!(snapshot.inlet_mass_flow_kg_per_s, 3.2);
        assert_abs_diff_eq!(snapshot.mixture_ratio.unwrap(), 3.0, epsilon = 1.0e-12);
    }

    #[test]
    fn transient_chamber_matches_provenance_tolerance_table() {
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
        let config = table("chamber_config", &data);
        let chamber = TransientChamber::new(TransientChamberConfig {
            chamber_volume_m3: float(config, "chamber_volume_m3"),
            gas_temperature_k: float(config, "gas_temperature_k"),
            gas_constant_j_per_kg_k: float(config, "gas_constant_j_per_kg_k"),
            throat_area_m2: float(config, "throat_area_m2"),
            c_star_m_s: float(config, "c_star_m_s"),
        })
        .unwrap();
        let tolerances = table("tolerances", &data);

        for case in data
            .get("chamber_case")
            .and_then(toml::Value::as_array)
            .expect("chamber_case array")
        {
            let case = case.as_table().unwrap();
            let name = string(case, "name");
            let snapshot = chamber
                .step(
                    TransientChamberState {
                        pressure_pa: float(case, "initial_pressure_pa"),
                    },
                    ChamberFeedInput {
                        oxidizer_mass_flow_kg_per_s: float(case, "oxidizer_mass_flow_kg_per_s"),
                        fuel_mass_flow_kg_per_s: float(case, "fuel_mass_flow_kg_per_s"),
                    },
                    float(case, "dt_s"),
                )
                .unwrap();

            assert_abs_diff_eq!(
                snapshot.previous_pressure_pa,
                float(case, "initial_pressure_pa"),
                epsilon = float(tolerances, "absolute_pressure_pa")
            );
            assert_abs_diff_eq!(
                snapshot.chamber_pressure_pa,
                float(case, "expected_chamber_pressure_pa"),
                epsilon = float(tolerances, "absolute_pressure_pa")
            );
            assert_abs_diff_eq!(
                snapshot.pressure_rate_pa_per_s,
                float(case, "expected_pressure_rate_pa_per_s"),
                epsilon = float(tolerances, "absolute_pressure_rate_pa_per_s")
            );
            assert_abs_diff_eq!(
                snapshot.inlet_mass_flow_kg_per_s,
                float(case, "expected_inlet_mass_flow_kg_per_s"),
                epsilon = float(tolerances, "absolute_mass_flow_kg_per_s")
            );
            assert_abs_diff_eq!(
                snapshot.outlet_mass_flow_kg_per_s,
                float(case, "expected_outlet_mass_flow_kg_per_s"),
                epsilon = float(tolerances, "absolute_mass_flow_kg_per_s")
            );
            assert_optional_float_eq(
                snapshot.mixture_ratio,
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
        }
    }

    #[test]
    fn transient_chamber_rejects_invalid_values() {
        let mut bad_config = chamber().config();
        bad_config.chamber_volume_m3 = 0.0;
        assert!(matches!(
            TransientChamber::new(bad_config),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        assert!(matches!(
            chamber().step(
                TransientChamberState { pressure_pa: -1.0 },
                ChamberFeedInput::single_fluid(0.3),
                0.1,
            ),
            Err(FeedSystemError::InvalidParameter { .. })
        ));

        assert!(matches!(
            chamber().step(
                TransientChamberState { pressure_pa: 1.0 },
                ChamberFeedInput::single_fluid(-0.3),
                0.1,
            ),
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
