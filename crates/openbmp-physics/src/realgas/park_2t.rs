//! Phase-6.10 Park two-temperature nonequilibrium thermochemistry.
//!
//! The Park two-temperature formulation is the intended nonequilibrium
//! air model for OpenBMP, but the first Phase-6 implementation carried
//! a hand-entered five-reaction proxy while the design document called
//! for a published Park87 reaction set with roughly seventeen
//! reactions and species-specific Millikan-White constants. The audit
//! could not verify those coefficients against a public table in the
//! repository.
//!
//! # Honest Scope
//!
//! This module now keeps only the typed API surface and fails closed
//! for every reaction-set query. A follow-on slice must add
//! provenance-backed Park87/Park90/Park93 tables, species-pair
//! vibrational relaxation constants, and the Mach-15 shock-layer
//! public benchmark before this model can return reaction rates.
//!
//! The Park87 and Park93 forward Arrhenius coefficients for the neutral
//! five-species subset are pinned below from Zhang et al. (2022),
//! *A review of the mathematical modeling of equilibrium and
//! nonequilibrium hypersonic flows*, Table 2. That public table does
//! not by itself provide the backward/equilibrium-constant path or the
//! species-pair vibrational relaxation constants needed for a complete
//! two-temperature source model, so [`ParkTwoTemperatureModel`] still
//! fails closed.

use super::AirComposition;
use crate::error::PhysicsError;

const CM3_PER_M3: f64 = 1.0e6;

/// Arrhenius forward-rate coefficient as published for air chemistry.
///
/// The public review table reports coefficients in its own CGS mole
/// units. OpenBMP stores those values verbatim, exposes an
/// `as_published` evaluator, and provides a narrow SI conversion for
/// the table's stated `cm^3 mole^-1 s^-1` forward-rate units. The full
/// source-term model is still reserved because backward rates and
/// relaxation constants are not pinned.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ArrheniusForwardCoefficient {
    /// Pre-exponential factor `A_f` as published.
    pub a_as_published: f64,
    /// Temperature exponent `B_f`.
    pub b: f64,
    /// Activation temperature `T_a` (K).
    pub activation_temperature_k: f64,
}

impl ArrheniusForwardCoefficient {
    /// Evaluate `k_f = A_f T^B_f exp(-T_a / T)` in the source
    /// table's published unit system.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `temperature_k`
    /// is not positive and finite.
    pub fn rate_as_published(self, temperature_k: f64) -> Result<f64, PhysicsError> {
        if !temperature_k.is_finite() || temperature_k <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Park forward-rate temperature must be positive and finite",
            });
        }
        Ok(self.a_as_published
            * temperature_k.powf(self.b)
            * (-self.activation_temperature_k / temperature_k).exp())
    }

    /// Evaluate the forward rate in SI units (`m^3 mol^-1 s^-1`) for
    /// the published neutral-air table rows.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `temperature_k`
    /// is not positive and finite.
    pub fn rate_m3_per_mol_s(self, temperature_k: f64) -> Result<f64, PhysicsError> {
        Ok(self.rate_as_published(temperature_k)? / CM3_PER_M3)
    }
}

/// One Park forward reaction row.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ParkForwardReaction {
    /// Row number in Zhang et al. (2022), Table 2.
    pub source_row: u8,
    /// ASCII reaction equation in source-table order.
    pub equation: &'static str,
    /// Forward Arrhenius coefficient.
    pub coefficient: ArrheniusForwardCoefficient,
}

/// Forward and backward reaction rate constants.
#[derive(Clone, Debug, PartialEq)]
pub struct ReactionRates {
    /// Reaction-set selector that defines reaction ordering.
    pub reaction_set: ParkReactionSet,
    /// Forward rate constants for each reaction in the selected table.
    pub k_forward: Vec<f64>,
    /// Backward rate constants for each reaction in the selected table.
    pub k_backward: Vec<f64>,
}

impl ReactionRates {
    /// Construct rate constants for a selected reaction table.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when forward and
    /// backward arrays have different lengths, or when a Park87 table
    /// does not carry its expected 17 reactions.
    pub fn new(
        reaction_set: ParkReactionSet,
        k_forward: Vec<f64>,
        k_backward: Vec<f64>,
    ) -> Result<Self, PhysicsError> {
        if k_forward.len() != k_backward.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "Park reaction-rate arrays must have equal length",
            });
        }
        if reaction_set == ParkReactionSet::Park87
            && k_forward.len() != ParkReactionSet::PARK87_REACTIONS
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "Park87 reaction-rate table must contain 17 reactions",
            });
        }
        Ok(Self {
            reaction_set,
            k_forward,
            k_backward,
        })
    }

    /// Number of reactions in the stored table.
    #[must_use]
    pub fn reaction_count(&self) -> usize {
        self.k_forward.len()
    }
}

/// Composition rate of change (mole fractions / second).
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct CompositionDerivative {
    /// d[N2] / dt (mole fraction / s).
    pub dn2_dt: f64,
    /// d[O2] / dt.
    pub do2_dt: f64,
    /// d[NO] / dt.
    pub dno_dt: f64,
    /// d[N] / dt.
    pub dn_dt: f64,
    /// d[O] / dt.
    pub do_dt: f64,
}

/// Vibrational energy derivative (J / (kg · s)).
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct VibrationalEnergyDerivative {
    /// `d(E_v / m) / dt` — vibrational energy per unit mass per second.
    pub de_v_per_mass_dt: f64,
}

/// Diagnostic flow context for Damköhler-number queries.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FlowContext {
    /// Characteristic flow timescale (s).
    pub tau_flow_s: f64,
    /// Characteristic chemistry timescale (s).
    pub tau_chemistry_s: f64,
}

/// Trait for nonequilibrium air models.
pub trait NonequilibriumAir {
    /// Forward / backward reaction rate constants at `(T, T_v)`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the selected
    /// reaction set is reserved or the query is outside the model
    /// envelope.
    fn reaction_rates(&self, t_tr_k: f64, t_v_k: f64) -> Result<ReactionRates, PhysicsError>;

    /// Species production rates given current composition and rates.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the model is
    /// reserved.
    fn species_derivative(
        &self,
        composition: &AirComposition,
        rates: &ReactionRates,
        density_kg_m3: f64,
    ) -> Result<CompositionDerivative, PhysicsError>;

    /// Vibrational energy relaxation rate (Landau-Teller form).
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the model is
    /// reserved.
    fn vibrational_relaxation(
        &self,
        t_tr_k: f64,
        t_v_k: f64,
        composition: &AirComposition,
        density_kg_m3: f64,
    ) -> Result<VibrationalEnergyDerivative, PhysicsError>;

    /// Damköhler number `Da = τ_flow / τ_chemistry`.
    #[must_use]
    fn damkohler(&self, ctx: &FlowContext) -> f64 {
        ctx.tau_flow_s / ctx.tau_chemistry_s.max(1.0e-30)
    }
}

/// Park reaction-set selector.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ParkReactionSet {
    /// Park 1987 — 5-species (`N2`, `O2`, `NO`, `N`, `O`). Forward
    /// coefficients are pinned as published-unit reference data; the
    /// executable source-term model remains reserved.
    Park87,
    /// Park 1990 — 11-species (adds ions, electrons). Reserved.
    Park90,
    /// Park 1993 — updated rate coefficients. Forward coefficients
    /// are pinned as published-unit reference data for the neutral
    /// five-species subset; the executable source-term model remains
    /// reserved.
    Park93,
}

impl ParkReactionSet {
    /// Park87 5-species reaction count: 15 dissociation reactions
    /// (`N2`, `O2`, `NO` with five collision partners each) plus
    /// the two Zeldovich exchange reactions.
    pub const PARK87_REACTIONS: usize = 17;

    /// Forward Arrhenius coefficients for the reaction set, when
    /// publicly pinned.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] for reserved reaction
    /// sets whose public coefficient tables have not landed.
    pub fn forward_coefficients_as_published(
        self,
    ) -> Result<&'static [ParkForwardReaction], PhysicsError> {
        match self {
            Self::Park87 => Ok(PARK87_FORWARD_REACTIONS_AS_PUBLISHED),
            Self::Park93 => Ok(PARK93_FORWARD_REACTIONS_AS_PUBLISHED),
            Self::Park90 => Err(PhysicsError::OutOfEnvelope {
                reason: "Park90 forward coefficients are reserved pending verified public tables",
            }),
        }
    }
}

/// Park87 neutral five-species forward coefficients from Zhang et al.
/// (2022), Table 2, Park1987 column.
///
/// Rows are the 15 neutral dissociation reactions for `O2`, `N2`, and
/// `NO` with neutral collision partners, followed by the two
/// Zeldovich exchange reactions. Values are stored as published in the
/// table, not converted to SI source-term units.
pub const PARK87_FORWARD_REACTIONS_AS_PUBLISHED: &[ParkForwardReaction] = &[
    ParkForwardReaction {
        source_row: 1,
        equation: "O2 + N <=> O + O + N",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 2.90e23,
            b: -2.0,
            activation_temperature_k: 59_750.0,
        },
    },
    ParkForwardReaction {
        source_row: 2,
        equation: "O2 + O <=> O + O + O",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 2.90e23,
            b: -2.0,
            activation_temperature_k: 59_750.0,
        },
    },
    ParkForwardReaction {
        source_row: 3,
        equation: "O2 + O2 <=> O + O + O2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 9.68e22,
            b: -2.0,
            activation_temperature_k: 59_750.0,
        },
    },
    ParkForwardReaction {
        source_row: 4,
        equation: "O2 + N2 <=> O + O + N2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 9.68e22,
            b: -2.0,
            activation_temperature_k: 59_750.0,
        },
    },
    ParkForwardReaction {
        source_row: 5,
        equation: "O2 + NO <=> O + O + NO",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 9.68e22,
            b: -2.0,
            activation_temperature_k: 59_750.0,
        },
    },
    ParkForwardReaction {
        source_row: 12,
        equation: "N2 + N <=> N + N + N",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 1.60e22,
            b: -1.6,
            activation_temperature_k: 113_200.0,
        },
    },
    ParkForwardReaction {
        source_row: 13,
        equation: "N2 + O <=> N + N + O",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 4.98e22,
            b: -1.6,
            activation_temperature_k: 113_200.0,
        },
    },
    ParkForwardReaction {
        source_row: 14,
        equation: "N2 + O2 <=> N + N + O2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 3.70e21,
            b: -1.6,
            activation_temperature_k: 113_200.0,
        },
    },
    ParkForwardReaction {
        source_row: 15,
        equation: "N2 + N2 <=> N + N + N2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 3.70e21,
            b: -1.6,
            activation_temperature_k: 113_200.0,
        },
    },
    ParkForwardReaction {
        source_row: 16,
        equation: "N2 + NO <=> N + N + NO",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 4.98e21,
            b: -1.6,
            activation_temperature_k: 113_200.0,
        },
    },
    ParkForwardReaction {
        source_row: 22,
        equation: "NO + N <=> N + O + N",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 7.95e23,
            b: -2.0,
            activation_temperature_k: 75_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 23,
        equation: "NO + O <=> N + O + O",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 7.95e23,
            b: -2.0,
            activation_temperature_k: 75_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 24,
        equation: "NO + N2 <=> N + O + N2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 7.95e23,
            b: -2.0,
            activation_temperature_k: 75_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 25,
        equation: "NO + O2 <=> N + O + O2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 7.95e23,
            b: -2.0,
            activation_temperature_k: 75_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 26,
        equation: "NO + NO <=> N + O + NO",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 7.95e23,
            b: -2.0,
            activation_temperature_k: 75_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 33,
        equation: "NO + O <=> N + O2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 8.37e12,
            b: 0.0,
            activation_temperature_k: 19_450.0,
        },
    },
    ParkForwardReaction {
        source_row: 34,
        equation: "N2 + O <=> NO + N",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 6.44e17,
            b: -1.0,
            activation_temperature_k: 38_370.0,
        },
    },
];

/// Park93 neutral five-species forward coefficients from Zhang et al.
/// (2022), Table 2, Park1993 column.
///
/// Rows are the same neutral 17-row subset used by
/// [`PARK87_FORWARD_REACTIONS_AS_PUBLISHED`]. Values are stored as
/// published in the table, not converted to SI source-term units.
pub const PARK93_FORWARD_REACTIONS_AS_PUBLISHED: &[ParkForwardReaction] = &[
    ParkForwardReaction {
        source_row: 1,
        equation: "O2 + N <=> O + O + N",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 1.00e22,
            b: -1.5,
            activation_temperature_k: 59_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 2,
        equation: "O2 + O <=> O + O + O",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 1.00e22,
            b: -1.5,
            activation_temperature_k: 59_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 3,
        equation: "O2 + O2 <=> O + O + O2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 2.00e21,
            b: -1.5,
            activation_temperature_k: 59_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 4,
        equation: "O2 + N2 <=> O + O + N2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 2.00e21,
            b: -1.5,
            activation_temperature_k: 59_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 5,
        equation: "O2 + NO <=> O + O + NO",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 2.00e21,
            b: -1.5,
            activation_temperature_k: 59_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 12,
        equation: "N2 + N <=> N + N + N",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 3.00e22,
            b: -1.6,
            activation_temperature_k: 113_200.0,
        },
    },
    ParkForwardReaction {
        source_row: 13,
        equation: "N2 + O <=> N + N + O",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 3.00e22,
            b: -1.6,
            activation_temperature_k: 113_200.0,
        },
    },
    ParkForwardReaction {
        source_row: 14,
        equation: "N2 + O2 <=> N + N + O2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 7.00e21,
            b: -1.6,
            activation_temperature_k: 113_200.0,
        },
    },
    ParkForwardReaction {
        source_row: 15,
        equation: "N2 + N2 <=> N + N + N2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 7.00e21,
            b: -1.6,
            activation_temperature_k: 113_200.0,
        },
    },
    ParkForwardReaction {
        source_row: 16,
        equation: "N2 + NO <=> N + N + NO",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 7.00e21,
            b: -1.6,
            activation_temperature_k: 113_200.0,
        },
    },
    ParkForwardReaction {
        source_row: 22,
        equation: "NO + N <=> N + O + N",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 1.10e17,
            b: 0.0,
            activation_temperature_k: 75_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 23,
        equation: "NO + O <=> N + O + O",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 1.10e17,
            b: 0.0,
            activation_temperature_k: 75_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 24,
        equation: "NO + N2 <=> N + O + N2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 5.00e15,
            b: 0.0,
            activation_temperature_k: 75_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 25,
        equation: "NO + O2 <=> N + O + O2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 5.00e15,
            b: 0.0,
            activation_temperature_k: 75_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 26,
        equation: "NO + NO <=> N + O + NO",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 1.10e17,
            b: 0.0,
            activation_temperature_k: 75_500.0,
        },
    },
    ParkForwardReaction {
        source_row: 33,
        equation: "NO + O <=> N + O2",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 8.40e12,
            b: 0.0,
            activation_temperature_k: 19_450.0,
        },
    },
    ParkForwardReaction {
        source_row: 34,
        equation: "N2 + O <=> NO + N",
        coefficient: ArrheniusForwardCoefficient {
            a_as_published: 6.40e17,
            b: -1.0,
            activation_temperature_k: 38_400.0,
        },
    },
];

/// Vibrational-relaxation model.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum VibrationalRelaxationModel {
    /// Classical Landau-Teller using Millikan-White correlation.
    MillikanWhite,
    /// Park's high-temperature correction on top of MW.
    MillikanWhitePark,
}

/// Park two-temperature model.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct ParkTwoTemperatureModel {
    /// Reaction-set selector.
    pub reaction_set: ParkReactionSet,
    /// Vibrational-relaxation model.
    pub vibrational_relaxation_model: VibrationalRelaxationModel,
}

impl Default for ParkTwoTemperatureModel {
    fn default() -> Self {
        Self {
            reaction_set: ParkReactionSet::Park87,
            vibrational_relaxation_model: VibrationalRelaxationModel::MillikanWhitePark,
        }
    }
}

impl ParkTwoTemperatureModel {
    /// Construct a Park-2T model.
    ///
    /// # Errors
    ///
    /// Always returns [`PhysicsError::OutOfEnvelope`] until verified
    /// Park reaction tables and Millikan-White constants land.
    pub fn new(
        _reaction_set: ParkReactionSet,
        _vrelax: VibrationalRelaxationModel,
    ) -> Result<Self, PhysicsError> {
        Err(Self::deferred())
    }

    /// Geometric mean temperature `T_a = sqrt(T * T_v)` for
    /// dissociation-rate evaluation.
    #[must_use]
    pub fn ta_geometric_mean(t_tr_k: f64, t_v_k: f64) -> f64 {
        (t_tr_k.max(0.0) * t_v_k.max(0.0)).sqrt()
    }

    fn deferred() -> PhysicsError {
        PhysicsError::OutOfEnvelope {
            reason: "Park two-temperature reaction sets are deferred pending verified public coefficients",
        }
    }
}

impl NonequilibriumAir for ParkTwoTemperatureModel {
    fn reaction_rates(&self, _t_tr_k: f64, _t_v_k: f64) -> Result<ReactionRates, PhysicsError> {
        Err(Self::deferred())
    }

    fn species_derivative(
        &self,
        _composition: &AirComposition,
        _rates: &ReactionRates,
        _density_kg_m3: f64,
    ) -> Result<CompositionDerivative, PhysicsError> {
        Err(Self::deferred())
    }

    fn vibrational_relaxation(
        &self,
        _t_tr_k: f64,
        _t_v_k: f64,
        _composition: &AirComposition,
        _density_kg_m3: f64,
    ) -> Result<VibrationalEnergyDerivative, PhysicsError> {
        Err(Self::deferred())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    fn cold_air() -> AirComposition {
        AirComposition {
            n2: 0.78,
            o2: 0.21,
            n_atomic: 0.0,
            o_atomic: 0.0,
            no: 0.01,
            argon: 0.0,
            n2_ion: 0.0,
            o2_ion: 0.0,
            no_ion: 0.0,
            n_ion: 0.0,
            o_ion: 0.0,
            electrons: 0.0,
        }
    }

    #[test]
    fn ta_geometric_mean_equals_t_when_t_eq_tv() {
        let ta = ParkTwoTemperatureModel::ta_geometric_mean(5000.0, 5000.0);
        assert!((ta - 5000.0).abs() < 1e-9);
    }

    #[test]
    fn ta_geometric_mean_lies_between_t_and_tv() {
        let ta = ParkTwoTemperatureModel::ta_geometric_mean(4000.0, 9000.0);
        assert!(ta > 4000.0 && ta < 9000.0);
    }

    #[test]
    fn park87_construction_is_reserved() {
        let model = ParkTwoTemperatureModel::new(
            ParkReactionSet::Park87,
            VibrationalRelaxationModel::MillikanWhitePark,
        );
        assert!(matches!(model, Err(PhysicsError::OutOfEnvelope { .. })));
    }

    #[test]
    fn manual_park_model_queries_fail_closed() {
        let model = ParkTwoTemperatureModel::default();
        assert!(matches!(
            model.reaction_rates(8000.0, 4000.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        let rates =
            ReactionRates::new(ParkReactionSet::Park87, vec![0.0; 17], vec![0.0; 17]).unwrap();
        assert!(matches!(
            model.species_derivative(&cold_air(), &rates, 1.0e-3),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            model.vibrational_relaxation(8000.0, 4000.0, &cold_air(), 1.0e-3),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn damkohler_basic_arithmetic_remains_available() {
        let model = ParkTwoTemperatureModel::default();
        let da = model.damkohler(&FlowContext {
            tau_flow_s: 1.0e-3,
            tau_chemistry_s: 1.0e-4,
        });
        assert!((da - 10.0).abs() < 1e-9);
    }

    #[test]
    fn park87_rate_container_requires_seventeen_reactions() {
        let short = ReactionRates::new(ParkReactionSet::Park87, vec![0.0; 5], vec![0.0; 5]);
        assert!(matches!(short, Err(PhysicsError::InvalidParameter { .. })));
        let full =
            ReactionRates::new(ParkReactionSet::Park87, vec![0.0; 17], vec![0.0; 17]).unwrap();
        assert_eq!(full.reaction_count(), 17);
    }

    #[test]
    fn park87_forward_table_pins_public_row_order() {
        let table = ParkReactionSet::Park87
            .forward_coefficients_as_published()
            .unwrap();
        assert_eq!(table.len(), ParkReactionSet::PARK87_REACTIONS);
        let rows: Vec<u8> = table.iter().map(|reaction| reaction.source_row).collect();
        assert_eq!(
            rows,
            vec![
                1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 22, 23, 24, 25, 26, 33, 34
            ]
        );
    }

    #[test]
    fn park93_forward_table_pins_public_row_order() {
        let table = ParkReactionSet::Park93
            .forward_coefficients_as_published()
            .unwrap();
        assert_eq!(table.len(), ParkReactionSet::PARK87_REACTIONS);
        let rows: Vec<u8> = table.iter().map(|reaction| reaction.source_row).collect();
        assert_eq!(
            rows,
            vec![
                1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 22, 23, 24, 25, 26, 33, 34
            ]
        );
    }

    #[test]
    fn park87_forward_table_pins_representative_coefficients() {
        let table = PARK87_FORWARD_REACTIONS_AS_PUBLISHED;
        assert_eq!(table[0].equation, "O2 + N <=> O + O + N");
        assert_eq!(table[0].coefficient.a_as_published, 2.90e23);
        assert_eq!(table[0].coefficient.b, -2.0);
        assert_eq!(table[0].coefficient.activation_temperature_k, 59_750.0);

        let exchange = table[16];
        assert_eq!(exchange.source_row, 34);
        assert_eq!(exchange.equation, "N2 + O <=> NO + N");
        assert_eq!(exchange.coefficient.a_as_published, 6.44e17);
        assert_eq!(exchange.coefficient.b, -1.0);
        assert_eq!(exchange.coefficient.activation_temperature_k, 38_370.0);
    }

    #[test]
    fn park93_forward_table_pins_representative_coefficients() {
        let table = PARK93_FORWARD_REACTIONS_AS_PUBLISHED;
        assert_eq!(table[0].equation, "O2 + N <=> O + O + N");
        assert_eq!(table[0].coefficient.a_as_published, 1.00e22);
        assert_eq!(table[0].coefficient.b, -1.5);
        assert_eq!(table[0].coefficient.activation_temperature_k, 59_500.0);

        let exchange = table[16];
        assert_eq!(exchange.source_row, 34);
        assert_eq!(exchange.equation, "N2 + O <=> NO + N");
        assert_eq!(exchange.coefficient.a_as_published, 6.40e17);
        assert_eq!(exchange.coefficient.b, -1.0);
        assert_eq!(exchange.coefficient.activation_temperature_k, 38_400.0);
    }

    #[test]
    fn park_forward_rate_evaluator_uses_published_arrhenius_form() {
        let coefficient = PARK87_FORWARD_REACTIONS_AS_PUBLISHED[15].coefficient;
        let t_k = 10_000.0;
        let rate = coefficient.rate_as_published(t_k).unwrap();
        let expected = coefficient.a_as_published
            * t_k.powf(coefficient.b)
            * (-coefficient.activation_temperature_k / t_k).exp();
        assert!((rate - expected).abs() / expected < 1.0e-12);
    }

    #[test]
    fn park_forward_rate_si_conversion_uses_table_units() {
        let coefficient = PARK93_FORWARD_REACTIONS_AS_PUBLISHED[16].coefficient;
        let t_k = 12_000.0;
        let published = coefficient.rate_as_published(t_k).unwrap();
        let si = coefficient.rate_m3_per_mol_s(t_k).unwrap();
        assert_eq!(si.to_bits(), (published / CM3_PER_M3).to_bits());
    }

    #[test]
    fn reserved_park90_still_fails_closed_for_forward_tables() {
        assert!(matches!(
            ParkReactionSet::Park90.forward_coefficients_as_published(),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }
}
