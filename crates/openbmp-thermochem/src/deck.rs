//! Deterministic thermochemistry deck and interpolation.

use crate::error::ThermochemError;

/// Universal molar gas constant in J/(mol K).
pub const UNIVERSAL_GAS_CONSTANT_J_PER_MOL_K: f64 = 8.314_462_618_153_24;
const C_STAR_REL_TOL: f64 = 1.0e-6;

/// Query point for a thermochemistry deck.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ThermochemQuery {
    /// Chamber pressure in Pa.
    pub chamber_pressure_pa: f64,
    /// Propellant mixture ratio, oxidizer mass flow over fuel mass flow.
    pub mixture_ratio: f64,
}

/// Thermochemical state returned by a deck lookup.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ThermochemState {
    /// Chamber temperature in K.
    pub chamber_temperature_k: f64,
    /// Ratio of specific heats.
    pub gamma: f64,
    /// Mean molecular weight in kg/mol.
    pub molecular_weight_kg_per_mol: f64,
    /// Ideal characteristic velocity in m/s.
    pub c_star_m_s: f64,
    /// Empirical `c*` efficiency band applied by runtime consumers.
    ///
    /// `c_star_m_s` remains the ideal value validated from temperature,
    /// gamma, and molecular weight; consumers use this band when they need an
    /// effective `c* = c*_ideal * eta_c*`.
    pub c_star_efficiency: CStarEfficiencyBand,
}

/// Empirical characteristic-velocity efficiency band.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CStarEfficiencyBand {
    /// Lower efficiency bound.
    pub min: f64,
    /// Nominal efficiency used for deterministic runtime propagation.
    pub nominal: f64,
    /// Upper efficiency bound.
    pub max: f64,
}

impl Default for CStarEfficiencyBand {
    fn default() -> Self {
        Self {
            min: 1.0,
            nominal: 1.0,
            max: 1.0,
        }
    }
}

impl CStarEfficiencyBand {
    /// Validate a finite physical efficiency band.
    ///
    /// # Errors
    ///
    /// Returns [`ThermochemError`] if any scalar is non-finite, non-positive,
    /// greater than one, or if the band is not ordered as
    /// `min <= nominal <= max`.
    pub fn validate(self) -> Result<Self, ThermochemError> {
        for value in [self.min, self.nominal, self.max] {
            if !value.is_finite() {
                return Err(ThermochemError::NonFinite {
                    reason: "c_star efficiency band contains NaN or infinity",
                });
            }
            if value <= 0.0 || value > 1.0 {
                return Err(ThermochemError::InvalidParameter {
                    reason: "c_star efficiency values must lie in (0, 1]",
                });
            }
        }
        if self.min > self.nominal || self.nominal > self.max {
            return Err(ThermochemError::InvalidParameter {
                reason: "c_star efficiency band must satisfy min <= nominal <= max",
            });
        }
        Ok(self)
    }

    fn lerp(a: Self, b: Self, f: f64) -> Self {
        let omf = 1.0 - f;
        Self {
            min: omf * a.min + f * b.min,
            nominal: omf * a.nominal + f * b.nominal,
            max: omf * a.max + f * b.max,
        }
    }
}

impl ThermochemState {
    /// Validate finite positive state fields.
    ///
    /// # Errors
    ///
    /// Returns [`ThermochemError`] when any field is non-finite or outside the
    /// ideal-gas choked-flow envelope.
    pub fn validate(self) -> Result<Self, ThermochemError> {
        for value in [
            self.chamber_temperature_k,
            self.gamma,
            self.molecular_weight_kg_per_mol,
            self.c_star_m_s,
        ] {
            if !value.is_finite() {
                return Err(ThermochemError::NonFinite {
                    reason: "thermochemistry state contains NaN or infinity",
                });
            }
        }
        if self.chamber_temperature_k <= 0.0 {
            return Err(ThermochemError::InvalidParameter {
                reason: "chamber temperature must be positive",
            });
        }
        if self.gamma <= 1.0 {
            return Err(ThermochemError::InvalidParameter {
                reason: "gamma must be greater than 1",
            });
        }
        if self.molecular_weight_kg_per_mol <= 0.0 {
            return Err(ThermochemError::InvalidParameter {
                reason: "molecular weight must be positive",
            });
        }
        if self.c_star_m_s <= 0.0 {
            return Err(ThermochemError::InvalidParameter {
                reason: "characteristic velocity must be positive",
            });
        }
        self.c_star_efficiency.validate()?;
        Ok(self)
    }

    fn lerp(a: Self, b: Self, f: f64) -> Self {
        let omf = 1.0 - f;
        Self {
            chamber_temperature_k: omf * a.chamber_temperature_k + f * b.chamber_temperature_k,
            gamma: omf * a.gamma + f * b.gamma,
            molecular_weight_kg_per_mol: omf * a.molecular_weight_kg_per_mol
                + f * b.molecular_weight_kg_per_mol,
            c_star_m_s: omf * a.c_star_m_s + f * b.c_star_m_s,
            c_star_efficiency: CStarEfficiencyBand::lerp(
                a.c_star_efficiency,
                b.c_star_efficiency,
                f,
            ),
        }
    }
}

/// Deterministic thermochemistry lookup surface.
pub trait ThermochemDeck {
    /// Look up the thermochemical state at a chamber pressure and mixture ratio.
    ///
    /// # Errors
    ///
    /// Returns [`ThermochemError::OutOfEnvelope`] for off-grid queries and
    /// validation errors for non-finite inputs.
    fn lookup(&self, query: ThermochemQuery) -> Result<ThermochemState, ThermochemError>;
}

/// Row-major thermochemistry table over chamber pressure and mixture ratio.
#[derive(Clone, Debug, PartialEq)]
pub struct ThermochemTable {
    chamber_pressure_pa: Vec<f64>,
    log_chamber_pressure: Vec<f64>,
    mixture_ratio: Vec<f64>,
    states: Vec<ThermochemState>,
}

impl ThermochemTable {
    /// Construct a validated thermochemistry table.
    ///
    /// `states` is row-major with mixture ratio varying fastest:
    /// `index(i_pc, i_mr) = i_pc * n_mr + i_mr`.
    ///
    /// # Errors
    ///
    /// Returns [`ThermochemError`] when axes are non-monotone, values are
    /// non-finite, the table length is wrong, or a table point's `c*` is not
    /// consistent with `Tc`, `gamma`, and molecular weight.
    pub fn new(
        chamber_pressure_pa: Vec<f64>,
        mixture_ratio: Vec<f64>,
        states: Vec<ThermochemState>,
    ) -> Result<Self, ThermochemError> {
        validate_axis(&chamber_pressure_pa, "chamber pressure")?;
        validate_axis(&mixture_ratio, "mixture ratio")?;
        let expected_len = chamber_pressure_pa
            .len()
            .checked_mul(mixture_ratio.len())
            .ok_or(ThermochemError::MalformedDeck {
                reason: "thermochemistry table shape overflowed",
            })?;
        if states.len() != expected_len {
            return Err(ThermochemError::MalformedDeck {
                reason: "thermochemistry state count does not match axes",
            });
        }
        for state in &states {
            state.validate()?;
            validate_c_star_consistency(*state)?;
        }
        let log_chamber_pressure = chamber_pressure_pa
            .iter()
            .map(|pressure| pressure.ln())
            .collect();
        Ok(Self {
            chamber_pressure_pa,
            log_chamber_pressure,
            mixture_ratio,
            states,
        })
    }

    /// Chamber-pressure axis in Pa.
    #[must_use]
    pub fn chamber_pressure_axis_pa(&self) -> &[f64] {
        &self.chamber_pressure_pa
    }

    /// Mixture-ratio axis.
    #[must_use]
    pub fn mixture_ratio_axis(&self) -> &[f64] {
        &self.mixture_ratio
    }

    /// Row-major state table.
    #[must_use]
    pub fn states(&self) -> &[ThermochemState] {
        &self.states
    }

    fn state(&self, pressure_index: usize, mixture_index: usize) -> ThermochemState {
        self.states[pressure_index * self.mixture_ratio.len() + mixture_index]
    }
}

impl ThermochemDeck for ThermochemTable {
    fn lookup(&self, query: ThermochemQuery) -> Result<ThermochemState, ThermochemError> {
        if !query.chamber_pressure_pa.is_finite() || !query.mixture_ratio.is_finite() {
            return Err(ThermochemError::NonFinite {
                reason: "thermochemistry query contains NaN or infinity",
            });
        }
        if query.chamber_pressure_pa <= 0.0 || query.mixture_ratio <= 0.0 {
            return Err(ThermochemError::InvalidParameter {
                reason: "thermochemistry query values must be positive",
            });
        }
        let (p0, p1, fp) = bracket(
            &self.log_chamber_pressure,
            query.chamber_pressure_pa.ln(),
            "pressure",
        )?;
        let (m0, m1, fm) = bracket(&self.mixture_ratio, query.mixture_ratio, "mixture ratio")?;

        let low_mr = ThermochemState::lerp(self.state(p0, m0), self.state(p0, m1), fm);
        let high_mr = ThermochemState::lerp(self.state(p1, m0), self.state(p1, m1), fm);
        ThermochemState::lerp(low_mr, high_mr, fp).validate()
    }
}

/// Choked ideal-gas mass-flux function `Gamma(gamma)`.
///
/// `c* = sqrt(R_specific * T_c) / Gamma(gamma)`.
///
/// # Errors
///
/// Returns [`ThermochemError`] when `gamma <= 1` or the result is non-finite.
pub fn choked_mass_flux_gamma(gamma: f64) -> Result<f64, ThermochemError> {
    if !gamma.is_finite() || gamma <= 1.0 {
        return Err(ThermochemError::InvalidParameter {
            reason: "gamma must be finite and greater than 1",
        });
    }
    let exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0));
    let value = gamma.sqrt() * (2.0 / (gamma + 1.0)).powf(exponent);
    if !value.is_finite() || value <= 0.0 {
        return Err(ThermochemError::NonFinite {
            reason: "choked mass-flux gamma function is non-finite",
        });
    }
    Ok(value)
}

/// Ideal characteristic velocity from chamber temperature, gamma, and molecular weight.
///
/// # Errors
///
/// Returns [`ThermochemError`] when inputs are non-finite or outside the ideal
/// choked-flow envelope.
pub fn characteristic_velocity_m_s(
    chamber_temperature_k: f64,
    gamma: f64,
    molecular_weight_kg_per_mol: f64,
) -> Result<f64, ThermochemError> {
    if !chamber_temperature_k.is_finite()
        || !molecular_weight_kg_per_mol.is_finite()
        || chamber_temperature_k <= 0.0
        || molecular_weight_kg_per_mol <= 0.0
    {
        return Err(ThermochemError::InvalidParameter {
            reason: "temperature and molecular weight must be finite and positive",
        });
    }
    let specific_gas_constant = UNIVERSAL_GAS_CONSTANT_J_PER_MOL_K / molecular_weight_kg_per_mol;
    let c_star =
        (specific_gas_constant * chamber_temperature_k).sqrt() / choked_mass_flux_gamma(gamma)?;
    if !c_star.is_finite() || c_star <= 0.0 {
        return Err(ThermochemError::NonFinite {
            reason: "characteristic velocity is non-finite",
        });
    }
    Ok(c_star)
}

fn validate_axis(axis: &[f64], name: &'static str) -> Result<(), ThermochemError> {
    if axis.len() < 2 {
        return Err(ThermochemError::MalformedDeck {
            reason: "thermochemistry axes must contain at least two points",
        });
    }
    for value in axis {
        if !value.is_finite() || *value <= 0.0 {
            return Err(ThermochemError::InvalidParameter {
                reason: "thermochemistry axes must be finite and positive",
            });
        }
    }
    for pair in axis.windows(2) {
        if pair[1] <= pair[0] {
            return Err(ThermochemError::MalformedDeck { reason: name });
        }
    }
    Ok(())
}

fn validate_c_star_consistency(state: ThermochemState) -> Result<(), ThermochemError> {
    let expected = characteristic_velocity_m_s(
        state.chamber_temperature_k,
        state.gamma,
        state.molecular_weight_kg_per_mol,
    )?;
    let rel = (state.c_star_m_s - expected).abs() / expected;
    if rel > C_STAR_REL_TOL {
        return Err(ThermochemError::InvalidParameter {
            reason: "deck c_star_m_s is inconsistent with temperature, gamma, and molecular weight",
        });
    }
    Ok(())
}

fn bracket(
    axis: &[f64],
    query: f64,
    name: &'static str,
) -> Result<(usize, usize, f64), ThermochemError> {
    if query < axis[0] || query > axis[axis.len() - 1] {
        return Err(ThermochemError::OutOfEnvelope { reason: name });
    }
    if query <= axis[0] {
        return Ok((0, 0, 0.0));
    }
    let last = axis.len() - 1;
    if query >= axis[last] {
        return Ok((last, last, 0.0));
    }
    let upper = axis.partition_point(|value| *value <= query);
    let lo = upper - 1;
    let hi = upper;
    let fraction = (query - axis[lo]) / (axis[hi] - axis[lo]);
    Ok((lo, hi, fraction))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn state(temperature: f64, gamma: f64, mw: f64) -> ThermochemState {
        ThermochemState {
            chamber_temperature_k: temperature,
            gamma,
            molecular_weight_kg_per_mol: mw,
            c_star_m_s: characteristic_velocity_m_s(temperature, gamma, mw).unwrap(),
            c_star_efficiency: CStarEfficiencyBand::default(),
        }
    }

    #[test]
    fn characteristic_velocity_matches_closed_form() {
        let c_star = characteristic_velocity_m_s(3_500.0, 1.22, 0.022).unwrap();
        assert_abs_diff_eq!(c_star, 1_762.929_269_397_344_1, epsilon = 1.0e-9);
    }

    #[test]
    fn lookup_interpolates_in_log_pressure_then_mixture_ratio() {
        let deck = ThermochemTable::new(
            vec![1.0e6, 4.0e6],
            vec![2.0, 3.0],
            vec![
                state(3_000.0, 1.20, 0.024),
                state(3_100.0, 1.21, 0.023),
                state(3_200.0, 1.22, 0.022),
                state(3_300.0, 1.23, 0.021),
            ],
        )
        .unwrap();
        let got = deck
            .lookup(ThermochemQuery {
                chamber_pressure_pa: 2.0e6,
                mixture_ratio: 2.5,
            })
            .unwrap();
        let fp = (2.0e6_f64.ln() - 1.0e6_f64.ln()) / (4.0e6_f64.ln() - 1.0e6_f64.ln());
        let fm = 0.5;
        let low = ThermochemState::lerp(deck.state(0, 0), deck.state(0, 1), fm);
        let high = ThermochemState::lerp(deck.state(1, 0), deck.state(1, 1), fm);
        let expected = ThermochemState::lerp(low, high, fp);

        assert_abs_diff_eq!(
            got.chamber_temperature_k,
            expected.chamber_temperature_k,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(got.gamma, expected.gamma, epsilon = 1.0e-12);
        assert_abs_diff_eq!(
            got.molecular_weight_kg_per_mol,
            expected.molecular_weight_kg_per_mol,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn lookup_interpolates_c_star_efficiency_band() {
        let mut states = vec![
            state(3_000.0, 1.20, 0.024),
            state(3_100.0, 1.21, 0.023),
            state(3_200.0, 1.22, 0.022),
            state(3_300.0, 1.23, 0.021),
        ];
        states[0].c_star_efficiency = CStarEfficiencyBand {
            min: 0.94,
            nominal: 0.96,
            max: 0.98,
        };
        states[1].c_star_efficiency = CStarEfficiencyBand {
            min: 0.95,
            nominal: 0.97,
            max: 0.99,
        };
        states[2].c_star_efficiency = CStarEfficiencyBand {
            min: 0.96,
            nominal: 0.98,
            max: 1.0,
        };
        states[3].c_star_efficiency = CStarEfficiencyBand {
            min: 0.97,
            nominal: 0.985,
            max: 1.0,
        };
        let deck = ThermochemTable::new(vec![1.0e6, 4.0e6], vec![2.0, 3.0], states).unwrap();

        let got = deck
            .lookup(ThermochemQuery {
                chamber_pressure_pa: 2.0e6,
                mixture_ratio: 2.5,
            })
            .unwrap();

        assert!(got.c_star_efficiency.min > 0.95);
        assert!(got.c_star_efficiency.nominal > got.c_star_efficiency.min);
        assert!(got.c_star_efficiency.max >= got.c_star_efficiency.nominal);
    }

    #[test]
    fn lookup_rejects_out_of_envelope_queries() {
        let deck = ThermochemTable::new(
            vec![1.0e6, 4.0e6],
            vec![2.0, 3.0],
            vec![
                state(3_000.0, 1.20, 0.024),
                state(3_100.0, 1.21, 0.023),
                state(3_200.0, 1.22, 0.022),
                state(3_300.0, 1.23, 0.021),
            ],
        )
        .unwrap();
        assert!(matches!(
            deck.lookup(ThermochemQuery {
                chamber_pressure_pa: 0.5e6,
                mixture_ratio: 2.5,
            }),
            Err(ThermochemError::OutOfEnvelope { .. })
        ));
    }
}
