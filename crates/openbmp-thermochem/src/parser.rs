//! TOML parser for Schema-1 thermochemistry decks.

use serde::Deserialize;

use crate::deck::{CStarEfficiencyBand, ThermochemState, ThermochemTable};
use crate::error::ThermochemError;
use openbmp_core::ValidationStatus;

const SCHEMA_VERSION: u32 = 1;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ThermochemFile {
    openbmp: SchemaMarker,
    meta: MetaSection,
    axes: AxesSection,
    state: Vec<StateRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SchemaMarker {
    thermochem_deck: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MetaSection {
    propellant_pair: String,
    provenance: String,
    validation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AxesSection {
    chamber_pressure_pa: Vec<f64>,
    mixture_ratio: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StateRow {
    chamber_pressure_pa: f64,
    mixture_ratio: f64,
    chamber_temperature_k: f64,
    gamma: f64,
    molecular_weight_kg_per_mol: f64,
    c_star_m_s: f64,
    #[serde(default)]
    c_star_efficiency: CStarEfficiencyBandRow,
}

#[derive(Copy, Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CStarEfficiencyBandRow {
    min: f64,
    nominal: f64,
    max: f64,
}

impl Default for CStarEfficiencyBandRow {
    fn default() -> Self {
        Self {
            min: 1.0,
            nominal: 1.0,
            max: 1.0,
        }
    }
}

impl From<CStarEfficiencyBandRow> for CStarEfficiencyBand {
    fn from(row: CStarEfficiencyBandRow) -> Self {
        Self {
            min: row.min,
            nominal: row.nominal,
            max: row.max,
        }
    }
}

impl ThermochemTable {
    /// Parse a Schema-1 thermochemistry deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`ThermochemError`] for TOML syntax failures, unsupported schema
    /// versions, malformed axes, missing grid points, duplicated grid points, or
    /// invalid thermochemical states.
    pub fn load_from_str(s: &str) -> Result<Self, ThermochemError> {
        let parsed: ThermochemFile = toml::from_str(s)?;
        if parsed.openbmp.thermochem_deck != SCHEMA_VERSION {
            return Err(ThermochemError::MalformedDeck {
                reason: "openbmp.thermochem_deck schema version is not 1",
            });
        }
        validate_meta(&parsed.meta)?;
        let axis_len = parsed
            .axes
            .chamber_pressure_pa
            .len()
            .checked_mul(parsed.axes.mixture_ratio.len())
            .ok_or(ThermochemError::MalformedDeck {
                reason: "thermochemistry deck axis shape overflowed",
            })?;
        let mut states: Vec<Option<ThermochemState>> = vec![None; axis_len];
        for row in parsed.state {
            let pressure_index = axis_index(
                &parsed.axes.chamber_pressure_pa,
                row.chamber_pressure_pa,
                "state chamber pressure is not on the pressure axis",
            )?;
            let mixture_index = axis_index(
                &parsed.axes.mixture_ratio,
                row.mixture_ratio,
                "state mixture ratio is not on the mixture-ratio axis",
            )?;
            let index = pressure_index * parsed.axes.mixture_ratio.len() + mixture_index;
            if states[index].is_some() {
                return Err(ThermochemError::MalformedDeck {
                    reason: "duplicate thermochemistry state grid point",
                });
            }
            states[index] = Some(ThermochemState {
                chamber_temperature_k: row.chamber_temperature_k,
                gamma: row.gamma,
                molecular_weight_kg_per_mol: row.molecular_weight_kg_per_mol,
                c_star_m_s: row.c_star_m_s,
                c_star_efficiency: row.c_star_efficiency.into(),
            });
        }
        let states = states.into_iter().collect::<Option<Vec<_>>>().ok_or(
            ThermochemError::MalformedDeck {
                reason: "thermochemistry deck is missing one or more state grid points",
            },
        )?;
        Self::new(
            parsed.axes.chamber_pressure_pa,
            parsed.axes.mixture_ratio,
            states,
        )
    }
}

fn validate_meta(meta: &MetaSection) -> Result<(), ThermochemError> {
    if meta.propellant_pair.is_empty() || meta.provenance.is_empty() || meta.validation.is_empty() {
        return Err(ThermochemError::MalformedDeck {
            reason: "thermochemistry deck meta fields must not be empty",
        });
    }
    match meta.validation.as_str() {
        "experimental" => {
            let _status = ValidationStatus::Experimental;
            Ok(())
        }
        "checked" => {
            let _status = ValidationStatus::Checked;
            Ok(())
        }
        "validated-toy" => {
            let _status = ValidationStatus::ValidatedToy;
            Ok(())
        }
        "research" => {
            let _status = ValidationStatus::Research;
            Ok(())
        }
        _ => Err(ThermochemError::MalformedDeck {
            reason: "thermochemistry deck validation label is unsupported",
        }),
    }
}

fn axis_index(axis: &[f64], value: f64, reason: &'static str) -> Result<usize, ThermochemError> {
    axis.iter()
        .position(|axis_value| axis_value.to_bits() == value.to_bits())
        .ok_or(ThermochemError::MalformedDeck { reason })
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::deck::{ThermochemDeck, ThermochemQuery};

    fn fixture() -> &'static str {
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/synthetic-thermochem.toml"
        ))
    }

    #[test]
    fn parser_loads_schema_one_deck() {
        let deck = ThermochemTable::load_from_str(fixture()).unwrap();
        let state = deck
            .lookup(ThermochemQuery {
                chamber_pressure_pa: 2.0e6,
                mixture_ratio: 2.5,
            })
            .unwrap();
        assert!(state.c_star_m_s > 1_500.0);
        assert!(state.c_star_efficiency.nominal < 1.0);
    }

    #[test]
    fn parser_rejects_duplicate_grid_point() {
        let toml = fixture().replacen("mixture_ratio = 3.0", "mixture_ratio = 2.0", 1);
        assert!(matches!(
            ThermochemTable::load_from_str(&toml),
            Err(ThermochemError::MalformedDeck { .. })
        ));
    }

    #[test]
    fn parser_rejects_malformed_efficiency_band() {
        let toml = fixture().replacen(
            "c_star_efficiency = { min = 0.955, nominal = 0.975, max = 0.995 }",
            "c_star_efficiency = { min = 0.995, nominal = 0.975, max = 0.955 }",
            1,
        );
        assert!(matches!(
            ThermochemTable::load_from_str(&toml),
            Err(ThermochemError::InvalidParameter { .. })
        ));
    }
}
