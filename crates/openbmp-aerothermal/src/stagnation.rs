//! Stagnation-point heat-transfer models.
//!
//! [`FayRiddell`] exposes both the cold-gas `HeatTransferModel` scaffold
//! and a caller-supplied edge-state assembly path for the 1958
//! equilibrium-air catalytic-wall axisymmetric formula. [`SuttonGraves`]
//! is the engineering simplification that needs only freestream density,
//! nose radius, and velocity; [`SuttonGravesGas`] provides the NASA
//! TR R-376 base-gas / Earth-air coefficient presets normalized to the
//! existing SI Earth-air constant. [`DetraKempRiddell`] exposes the
//! classic DKR imperial-reference stagnation heating cross-check with
//! an SI wrapper. [`TauberSuttonRadiative`] is off by default, and can
//! be explicitly constructed with the provenance-pinned Earth-air
//! Tauber-Sutton velocity-function package.

pub use openbmp_physics::sutherland_viscosity;
use openbmp_physics::{
    AirComposition, AllenEggers, AtmosphereSample, EnvelopeBounds, ExternalReferencePackage,
    PhysicsError, ProvenanceBlock, ReferencePackageKind,
};

use crate::error::AerothermalError;

/// Body station identifier used by distributed-heating consumers.
/// The stagnation models only need station == stagnation point;
/// the type is exposed here for trait completeness.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Default)]
pub struct BodyStation {
    /// Arc-length coordinate from stagnation point (m).
    pub arc_length_m_x10: i64,
}

impl BodyStation {
    /// Stagnation-point station (arc length 0).
    #[must_use]
    pub const fn stagnation() -> Self {
        Self {
            arc_length_m_x10: 0,
        }
    }
}

/// Wall catalysis policy used by [`FayRiddell`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum WallCatalysis {
    /// Fully catalytic wall (recombination efficiency 1.0). Lewis
    /// exponent `a = 0.52` per Fay-Riddell.
    FullyCatalytic,
    /// Non-catalytic wall (recombination efficiency 0). Lewis
    /// exponent `a = 0.63`.
    NonCatalytic,
    /// Partial catalysis with declared recombination efficiency
    /// `η ∈ [0, 1]`. The Lewis exponent is interpolated linearly
    /// between the two endpoints.
    Partial(f64),
}

impl WallCatalysis {
    fn lewis_exponent(self) -> f64 {
        match self {
            Self::FullyCatalytic => 0.52,
            Self::NonCatalytic => 0.63,
            Self::Partial(eta) => {
                let eta_clamped = eta.clamp(0.0, 1.0);
                0.63 - (0.63 - 0.52) * eta_clamped
            }
        }
    }
}

/// Inputs assembled for an aerothermal-model evaluation.
#[derive(Copy, Clone, Debug)]
pub struct AerothermalContext {
    /// Freestream atmospheric sample (ρ_∞, p_∞, T_∞, a_∞).
    pub freestream: AtmosphereSample,
    /// Freestream airspeed (m/s).
    pub airspeed_m_s: f64,
    /// Mach number.
    pub mach: f64,
    /// Nose radius for stagnation-point correlations (m).
    pub nose_radius_m: f64,
    /// Wall temperature (K).
    pub wall_temperature_k: f64,
    /// Wall catalysis policy.
    pub wall_catalysis: WallCatalysis,
}

/// Stagnation-point heat-flux outputs.
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct StagnationHeating {
    /// Convective heat flux (W/m²).
    pub q_conv_w_m2: f64,
    /// Radiative heat flux (W/m²).
    pub q_rad_w_m2: f64,
    /// Adiabatic-wall enthalpy (J/kg).
    pub h_aw_j_kg: f64,
    /// Wall enthalpy (J/kg).
    pub h_w_j_kg: f64,
    /// Recovery temperature (K).
    pub recovery_temperature_k: f64,
}

/// Closed-form convective heating estimate for an Allen-Eggers
/// ballistic entry profile.
///
/// The estimate integrates the Sutton-Graves stagnation correlation
/// over the Allen-Eggers exponential-atmosphere velocity profile. It is
/// useful as a trajectory-level thermal diagnostic: peak deceleration
/// comes from [`AllenEggers`], while this struct reports the
/// corresponding stagnation heating scale.
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct BallisticEntryHeating {
    /// Peak stagnation-point convective heat flux (W/m²).
    pub peak_convective_heat_flux_w_m2: f64,
    /// Altitude where the peak convective heat flux occurs (m).
    pub peak_heat_flux_altitude_m: f64,
    /// Integrated stagnation-point convective heat load (J/m²).
    pub convective_heat_load_j_m2: f64,
}

/// Fay-Riddell boundary-layer edge and wall properties.
///
/// This is the real-gas coupling point for the Fay-Riddell
/// assembly: an upstream equilibrium-air solver, CFD deck, or external
/// reference package can provide the post-shock edge state and wall
/// thermodynamics directly. OpenBMP does not infer these values here.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FayRiddellEdgeState {
    /// Boundary-layer edge density `ρ_e` (kg/m³).
    pub edge_density_kg_m3: f64,
    /// Boundary-layer edge dynamic viscosity `μ_e` (Pa·s).
    pub edge_viscosity_pa_s: f64,
    /// Wall density `ρ_w` evaluated at wall temperature and edge
    /// pressure (kg/m³).
    pub wall_density_kg_m3: f64,
    /// Wall dynamic viscosity `μ_w` (Pa·s).
    pub wall_viscosity_pa_s: f64,
    /// Stagnation velocity gradient `du_e/dx` (1/s).
    pub velocity_gradient_s_inv: f64,
    /// Adiabatic-wall enthalpy `h_aw` (J/kg).
    pub adiabatic_wall_enthalpy_j_kg: f64,
    /// Wall enthalpy `h_w` (J/kg).
    pub wall_enthalpy_j_kg: f64,
    /// Recovery temperature for diagnostics (K).
    pub recovery_temperature_k: f64,
}

impl FayRiddellEdgeState {
    fn validate(&self) -> Result<(), AerothermalError> {
        if !self.edge_density_kg_m3.is_finite()
            || !self.edge_viscosity_pa_s.is_finite()
            || !self.wall_density_kg_m3.is_finite()
            || !self.wall_viscosity_pa_s.is_finite()
            || !self.velocity_gradient_s_inv.is_finite()
            || !self.adiabatic_wall_enthalpy_j_kg.is_finite()
            || !self.wall_enthalpy_j_kg.is_finite()
            || !self.recovery_temperature_k.is_finite()
        {
            return Err(AerothermalError::NonFinite {
                reason: "Fay-Riddell edge-state input is NaN or Inf",
            });
        }
        if self.edge_density_kg_m3 <= 0.0
            || self.edge_viscosity_pa_s <= 0.0
            || self.wall_density_kg_m3 <= 0.0
            || self.wall_viscosity_pa_s <= 0.0
            || self.velocity_gradient_s_inv <= 0.0
            || self.recovery_temperature_k <= 0.0
        {
            return Err(AerothermalError::InvalidParameter {
                reason: "Fay-Riddell edge state requires positive density, viscosity, velocity gradient, and recovery temperature",
            });
        }
        Ok(())
    }
}

/// Distributed surface-station heat-flux outputs.
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct SurfaceHeating {
    /// Local heat flux (W/m²).
    pub q_w_m2: f64,
    /// Local Stanton number.
    pub stanton: f64,
    /// Local skin-friction coefficient.
    pub skin_friction: f64,
}

/// Unified heat-transfer trait.
pub trait HeatTransferModel {
    /// Stagnation-point heating for this model.
    ///
    /// # Errors
    ///
    /// Out-of-envelope when inputs violate the model's documented
    /// validity range (e.g. zero nose radius, sub-sonic airspeed,
    /// vacuum density).
    fn stagnation(&self, ctx: &AerothermalContext) -> Result<StagnationHeating, AerothermalError>;
}

/// Composite stagnation model that sums a convective and radiative model.
///
/// The convective model owns the thermodynamic diagnostics (`h_aw`, `h_w`,
/// recovery temperature); the radiative model contributes `q_rad_w_m2`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TotalStagnationHeating<C, R> {
    convective: C,
    radiative: R,
}

impl<C, R> TotalStagnationHeating<C, R> {
    /// Construct a total-heating model from separate convective and radiative models.
    #[must_use]
    pub const fn new(convective: C, radiative: R) -> Self {
        Self {
            convective,
            radiative,
        }
    }
}

impl<C, R> HeatTransferModel for TotalStagnationHeating<C, R>
where
    C: HeatTransferModel,
    R: HeatTransferModel,
{
    fn stagnation(&self, ctx: &AerothermalContext) -> Result<StagnationHeating, AerothermalError> {
        let convective = self.convective.stagnation(ctx)?;
        let radiative = self.radiative.stagnation(ctx)?;
        let q_conv_w_m2 = convective.q_conv_w_m2 + radiative.q_conv_w_m2;
        let q_rad_w_m2 = convective.q_rad_w_m2 + radiative.q_rad_w_m2;
        if !q_conv_w_m2.is_finite() || !q_rad_w_m2.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "total stagnation heat flux non-finite",
            });
        }
        Ok(StagnationHeating {
            q_conv_w_m2,
            q_rad_w_m2,
            h_aw_j_kg: convective.h_aw_j_kg,
            h_w_j_kg: convective.h_w_j_kg,
            recovery_temperature_k: convective.recovery_temperature_k,
        })
    }
}

/// Reference air heat capacity (perfect-gas, cold-air) used for
/// `h = c_p · T` shortcuts. Real-gas h(T,p) is the
/// [`openbmp_physics::EquilibriumAir`] consumer's job; the
/// engineering Fay-Riddell shipped here uses the cold-gas value to
/// keep the standalone module honest.
const C_P_AIR_J_KG_K: f64 = 1005.0;

/// Sutton-Graves engineering stagnation-point convective heating
/// constant for Earth atmosphere (SI units).
pub const SUTTON_GRAVES_K_EARTH_SI: f64 = 1.7415e-4;

/// DKR reference heat flux coefficient in `Btu/(ft²·s)`.
pub const DETRA_KEMP_RIDDELL_REFERENCE_HEAT_FLUX_BTU_FT2_S: f64 = 17_600.0;

/// DKR reference velocity in `ft/s`.
pub const DETRA_KEMP_RIDDELL_REFERENCE_VELOCITY_FT_S: f64 = 26_000.0;

/// DKR velocity exponent.
pub const DETRA_KEMP_RIDDELL_VELOCITY_EXPONENT: f64 = 3.15;

/// DKR hot-wall correction reference wall temperature.
pub const DETRA_KEMP_RIDDELL_REFERENCE_WALL_TEMPERATURE_K: f64 = 300.0;

/// Sea-level density used by the SI DKR wrapper.
pub const DETRA_KEMP_RIDDELL_SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;

/// SHA-256 hash of `data/aerothermal/radiative/tauber-sutton-earth-v1.toml`.
pub const TAUBER_SUTTON_EARTH_AIR_PAYLOAD_SHA256_HEX: &str =
    "06d66cbd2ee2fa70ebcfbf9cf7f0aedee727dc06bc79d90d2c0a01936342b870";

/// Tauber-Sutton Earth-air radiative coefficient in `W/(cm²)` formula units.
pub const TAUBER_SUTTON_EARTH_AIR_COEFFICIENT_W_CM2: f64 = 47_360.0;

/// Tauber-Sutton Earth-air density exponent.
pub const TAUBER_SUTTON_EARTH_AIR_DENSITY_EXPONENT: f64 = 1.22;

/// Tauber-Sutton Earth-air radius-exponent coefficient.
pub const TAUBER_SUTTON_EARTH_AIR_RADIUS_EXPONENT_COEFFICIENT: f64 = 1_072_000.0;

/// Tauber-Sutton Earth-air velocity power in the radius exponent.
pub const TAUBER_SUTTON_EARTH_AIR_RADIUS_EXPONENT_VELOCITY_POWER: f64 = -1.88;

/// Tauber-Sutton Earth-air density power in the radius exponent.
pub const TAUBER_SUTTON_EARTH_AIR_RADIUS_EXPONENT_DENSITY_POWER: f64 = -0.325;

/// Tabulated Tauber-Sutton Earth-air velocity function, including the
/// model-floor zero sentinel at 9 km/s.
pub static TAUBER_SUTTON_EARTH_AIR_VELOCITY_FUNCTION: [TauberSuttonVelocityFunctionPoint; 16] = [
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 9_000.0,
        value: 0.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 10_000.0,
        value: 35.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 10_250.0,
        value: 55.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 10_500.0,
        value: 81.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 10_750.0,
        value: 115.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 11_000.0,
        value: 151.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 11_500.0,
        value: 238.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 12_000.0,
        value: 359.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 12_500.0,
        value: 495.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 13_000.0,
        value: 660.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 13_500.0,
        value: 850.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 14_000.0,
        value: 1_065.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 14_500.0,
        value: 1_313.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 15_000.0,
        value: 1_550.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 15_500.0,
        value: 1_780.0,
    },
    TauberSuttonVelocityFunctionPoint {
        velocity_m_s: 16_000.0,
        value: 2_040.0,
    },
];

const M_PER_FT: f64 = 0.3048;
const BTU_FT2_S_TO_W_M2: f64 = 1_055.055_852_62 / (M_PER_FT * M_PER_FT);
const W_CM2_TO_W_M2: f64 = 10_000.0;

/// NASA TR R-376 tabulated Sutton-Graves heat-transfer coefficient for
/// Earth air, used only as a dimensionless normalization anchor for
/// converting the public base-gas table into the existing OpenBMP SI
/// constant scale.
pub const SUTTON_GRAVES_TABLE_K_EARTH_AIR: f64 = 0.1113;

/// Gas preset for the Sutton-Graves stagnation heating coefficient.
///
/// The table values are from Sutton & Graves, NASA TR R-376, Table II
/// ("Heat-transfer coefficients from correlation of present computer
/// results"). OpenBMP keeps the historical SI Earth-air constant as
/// the absolute anchor and scales other gases by `K_table(gas) /
/// K_table(earth_air)`. This avoids silently changing legacy Earth-air
/// heating while exposing the report's per-gas trend.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SuttonGravesGas {
    /// Earth air, `0.232 O2 + 0.768 N2` by mass in TR R-376.
    EarthAir,
    /// Nitrogen, N2.
    Nitrogen,
    /// Oxygen, O2.
    Oxygen,
    /// Hydrogen, H2.
    Hydrogen,
    /// Helium, He.
    Helium,
    /// Neon, Ne.
    Neon,
    /// Argon, Ar.
    Argon,
    /// Carbon dioxide, CO2.
    CarbonDioxide,
    /// Ammonia, NH3.
    Ammonia,
    /// Methane, CH4.
    Methane,
}

impl SuttonGravesGas {
    /// TR R-376 Table II heat-transfer coefficient in the report's
    /// tabulated units.
    #[must_use]
    pub const fn table_heat_transfer_coefficient(self) -> f64 {
        match self {
            Self::EarthAir => SUTTON_GRAVES_TABLE_K_EARTH_AIR,
            Self::Nitrogen => 0.1112,
            Self::Oxygen => 0.1201,
            Self::Hydrogen => 0.0395,
            Self::Helium => 0.0797,
            Self::Neon => 0.1474,
            Self::Argon => 0.1495,
            Self::CarbonDioxide => 0.1210,
            Self::Ammonia => 0.0990,
            Self::Methane => 0.0807,
        }
    }

    /// Sutton-Graves coefficient on OpenBMP's SI Earth-air scale.
    #[must_use]
    pub fn coefficient_si(self) -> f64 {
        SUTTON_GRAVES_K_EARTH_SI * self.table_heat_transfer_coefficient()
            / SUTTON_GRAVES_TABLE_K_EARTH_AIR
    }
}

/// Sutton-Graves engineering correlation.
///
/// `q_w = K · √(ρ_∞ / R_n) · V_∞³` — quick conservative bound and
/// sanity check; no real-gas iteration, no boundary-layer state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SuttonGraves {
    /// Earth-atmosphere constant. Use [`SUTTON_GRAVES_K_EARTH_SI`]
    /// unless the scenario opts into a different planetary body
    /// (heat-transfer models are Earth-only, so this is the value
    /// most callers want).
    pub k_earth_si: f64,
}

impl Default for SuttonGraves {
    fn default() -> Self {
        Self {
            k_earth_si: SUTTON_GRAVES_K_EARTH_SI,
        }
    }
}

impl SuttonGraves {
    /// Construct a Sutton-Graves model from an explicit SI coefficient.
    ///
    /// This preserves the legacy public field while giving callers a
    /// named constructor for non-default coefficient values.
    #[must_use]
    pub const fn with_coefficient_si(k_si: f64) -> Self {
        Self { k_earth_si: k_si }
    }

    /// Construct a Sutton-Graves model using a TR R-376 gas preset.
    #[must_use]
    pub fn for_gas(gas: SuttonGravesGas) -> Self {
        Self::with_coefficient_si(gas.coefficient_si())
    }

    /// Evaluate the Sutton-Graves correlation and apply a finite wall
    /// enthalpy correction, `max(0, 1 - h_w / h_aw)`.
    ///
    /// The default [`HeatTransferModel`] implementation returns the
    /// cold-wall engineering value. This method is the higher-fidelity
    /// finite-wall-temperature variant for trajectory thermal budgets.
    ///
    /// # Errors
    ///
    /// Returns the same validation errors as [`Self::stagnation`], plus
    /// an invalid-parameter error if the adiabatic-wall enthalpy is not
    /// positive.
    pub fn stagnation_with_wall_enthalpy_correction(
        &self,
        ctx: &AerothermalContext,
    ) -> Result<StagnationHeating, AerothermalError> {
        let mut heating = self.stagnation(ctx)?;
        heating.q_conv_w_m2 *= wall_enthalpy_correction(heating.h_aw_j_kg, heating.h_w_j_kg)?;
        Ok(heating)
    }

    /// Closed-form Sutton-Graves heating over an Allen-Eggers
    /// ballistic entry profile.
    ///
    /// Assumptions: non-lifting entry, constant flight-path angle,
    /// exponential atmosphere, constant inverse ballistic parameter
    /// `B = C_D A / m`, and Sutton-Graves cold-wall convective
    /// heating. Radiation and real-gas edge-state effects are not
    /// included; use [`FayRiddell::stagnation_from_edge_state`] when a
    /// research-grade edge state is available.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError`] when the entry profile, nose radius,
    /// or Sutton-Graves constant is invalid.
    pub fn allen_eggers_heating(
        &self,
        entry: &AllenEggers,
        nose_radius_m: f64,
    ) -> Result<BallisticEntryHeating, AerothermalError> {
        entry
            .validate()
            .map_err(|_| AerothermalError::InvalidParameter {
                reason: "Allen-Eggers heating requires a valid ballistic-entry profile",
            })?;
        if !nose_radius_m.is_finite() || nose_radius_m <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Allen-Eggers heating requires positive finite nose radius",
            });
        }
        if !self.k_earth_si.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Sutton-Graves constant is NaN or Inf",
            });
        }
        if self.k_earth_si < 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Sutton-Graves constant must be non-negative",
            });
        }

        let beta = entry.beta_inv_m;
        let sin_gamma = entry.flight_path_angle_rad.sin().abs();
        let inverse_ballistic_parameter = entry.ballistic_coefficient_m2_kg;
        let v_e = entry.entry_velocity_m_s;
        let peak_altitude_inner =
            3.0 * entry.rho_s_kg_m3 * inverse_ballistic_parameter / (beta * sin_gamma);
        let peak_heat_flux_altitude_m = peak_altitude_inner.ln() / beta;
        let peak_convective_heat_flux_w_m2 = self.k_earth_si
            * v_e.powi(3)
            * (beta * sin_gamma / (3.0 * inverse_ballistic_parameter * nose_radius_m)).sqrt()
            * (-0.5_f64).exp();
        let convective_heat_load_j_m2 = self.k_earth_si
            * v_e.powi(2)
            * (std::f64::consts::PI
                / (beta * sin_gamma * inverse_ballistic_parameter * nose_radius_m))
                .sqrt();

        let estimate = BallisticEntryHeating {
            peak_convective_heat_flux_w_m2,
            peak_heat_flux_altitude_m,
            convective_heat_load_j_m2,
        };
        validate_ballistic_heating(estimate)?;
        Ok(estimate)
    }
}

impl HeatTransferModel for SuttonGraves {
    fn stagnation(&self, ctx: &AerothermalContext) -> Result<StagnationHeating, AerothermalError> {
        validate_context_for_stagnation(ctx)?;
        if !self.k_earth_si.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Sutton-Graves constant is NaN or Inf",
            });
        }
        if self.k_earth_si < 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Sutton-Graves constant must be non-negative",
            });
        }
        let rho = ctx.freestream.density_kg_m3;
        let r_n = ctx.nose_radius_m;
        let v = ctx.airspeed_m_s;
        let q = self.k_earth_si * (rho / r_n).sqrt() * v.powi(3);
        let adiabatic_wall_enthalpy = 0.5 * v * v + C_P_AIR_J_KG_K * ctx.freestream.temperature_k;
        let wall_enthalpy = C_P_AIR_J_KG_K * ctx.wall_temperature_k;
        let recovery = ctx.freestream.temperature_k + 0.5 * v * v / C_P_AIR_J_KG_K;
        if !q.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Sutton-Graves stagnation heat flux non-finite",
            });
        }
        Ok(StagnationHeating {
            q_conv_w_m2: q,
            q_rad_w_m2: 0.0,
            h_aw_j_kg: adiabatic_wall_enthalpy,
            h_w_j_kg: wall_enthalpy,
            recovery_temperature_k: recovery,
        })
    }
}

/// Detra-Kemp-Riddell stagnation-point convective heating cross-check.
///
/// The implementation wraps the classic engineering form
/// `q = 17600 / sqrt(R_n_ft) * sqrt(rho_inf / rho_sl)
/// * (V_inf_ft_s / 26000)^3.15 * wall_factor`, returning SI `W/m²`.
/// It is intended as a code-to-code/handbook cross-check alongside
/// [`SuttonGraves`], not as a replacement for real-gas Fay-Riddell or CFD.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DetraKempRiddell {
    /// Sea-level density normalization for the density ratio.
    pub sea_level_density_kg_m3: f64,
}

impl Default for DetraKempRiddell {
    fn default() -> Self {
        Self {
            sea_level_density_kg_m3: DETRA_KEMP_RIDDELL_SEA_LEVEL_DENSITY_KG_M3,
        }
    }
}

impl DetraKempRiddell {
    /// Construct a DKR model with an explicit sea-level density
    /// normalization.
    #[must_use]
    pub const fn with_sea_level_density(sea_level_density_kg_m3: f64) -> Self {
        Self {
            sea_level_density_kg_m3,
        }
    }

    /// Evaluate the DKR cold-reference heat flux before SI conversion and
    /// wall-enthalpy correction.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError`] if the context or density normalization
    /// is non-finite or outside the correlation envelope.
    pub fn cold_reference_heat_flux_btu_ft2_s(
        &self,
        ctx: &AerothermalContext,
    ) -> Result<f64, AerothermalError> {
        validate_context_for_stagnation(ctx)?;
        self.validate()?;

        let radius_ft = ctx.nose_radius_m / M_PER_FT;
        let velocity_ft_s = ctx.airspeed_m_s / M_PER_FT;
        let density_ratio = ctx.freestream.density_kg_m3 / self.sea_level_density_kg_m3;
        let q = DETRA_KEMP_RIDDELL_REFERENCE_HEAT_FLUX_BTU_FT2_S / radius_ft.sqrt()
            * density_ratio.sqrt()
            * (velocity_ft_s / DETRA_KEMP_RIDDELL_REFERENCE_VELOCITY_FT_S)
                .powf(DETRA_KEMP_RIDDELL_VELOCITY_EXPONENT);
        if !q.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Detra-Kemp-Riddell heat flux is non-finite",
            });
        }
        Ok(q)
    }

    fn validate(&self) -> Result<(), AerothermalError> {
        if !self.sea_level_density_kg_m3.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Detra-Kemp-Riddell density normalization is NaN or Inf",
            });
        }
        if self.sea_level_density_kg_m3 <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Detra-Kemp-Riddell density normalization must be positive",
            });
        }
        Ok(())
    }
}

impl HeatTransferModel for DetraKempRiddell {
    fn stagnation(&self, ctx: &AerothermalContext) -> Result<StagnationHeating, AerothermalError> {
        let q_cold_reference = self.cold_reference_heat_flux_btu_ft2_s(ctx)? * BTU_FT2_S_TO_W_M2;
        let adiabatic_wall_enthalpy = 0.5 * ctx.airspeed_m_s * ctx.airspeed_m_s
            + C_P_AIR_J_KG_K * ctx.freestream.temperature_k;
        let wall_enthalpy = C_P_AIR_J_KG_K * ctx.wall_temperature_k;
        let reference_wall_enthalpy =
            C_P_AIR_J_KG_K * DETRA_KEMP_RIDDELL_REFERENCE_WALL_TEMPERATURE_K;
        let correction = detra_kemp_riddell_wall_enthalpy_correction(
            adiabatic_wall_enthalpy,
            wall_enthalpy,
            reference_wall_enthalpy,
        )?;
        let q = q_cold_reference * correction;
        let recovery = ctx.freestream.temperature_k
            + 0.5 * ctx.airspeed_m_s * ctx.airspeed_m_s / C_P_AIR_J_KG_K;
        if !q.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Detra-Kemp-Riddell corrected heat flux is non-finite",
            });
        }
        Ok(StagnationHeating {
            q_conv_w_m2: q,
            q_rad_w_m2: 0.0,
            h_aw_j_kg: adiabatic_wall_enthalpy,
            h_w_j_kg: wall_enthalpy,
            recovery_temperature_k: recovery,
        })
    }
}

fn detra_kemp_riddell_wall_enthalpy_correction(
    adiabatic_enthalpy_j_kg: f64,
    wall_enthalpy_j_kg: f64,
    reference_wall_enthalpy_j_kg: f64,
) -> Result<f64, AerothermalError> {
    if !adiabatic_enthalpy_j_kg.is_finite()
        || !wall_enthalpy_j_kg.is_finite()
        || !reference_wall_enthalpy_j_kg.is_finite()
    {
        return Err(AerothermalError::NonFinite {
            reason: "Detra-Kemp-Riddell wall correction input is NaN or Inf",
        });
    }
    let denominator = adiabatic_enthalpy_j_kg - reference_wall_enthalpy_j_kg;
    if denominator <= 0.0 {
        return Err(AerothermalError::InvalidParameter {
            reason: "Detra-Kemp-Riddell wall correction requires stagnation enthalpy above reference wall enthalpy",
        });
    }
    Ok(((adiabatic_enthalpy_j_kg - wall_enthalpy_j_kg) / denominator).max(0.0))
}

fn wall_enthalpy_correction(
    adiabatic_enthalpy_j_kg: f64,
    wall_enthalpy_j_kg: f64,
) -> Result<f64, AerothermalError> {
    if !adiabatic_enthalpy_j_kg.is_finite() || !wall_enthalpy_j_kg.is_finite() {
        return Err(AerothermalError::NonFinite {
            reason: "wall enthalpy correction input is NaN or Inf",
        });
    }
    if adiabatic_enthalpy_j_kg <= 0.0 {
        return Err(AerothermalError::InvalidParameter {
            reason: "wall enthalpy correction requires positive adiabatic-wall enthalpy",
        });
    }
    Ok((1.0 - wall_enthalpy_j_kg / adiabatic_enthalpy_j_kg).clamp(0.0, 1.0))
}

fn validate_ballistic_heating(estimate: BallisticEntryHeating) -> Result<(), AerothermalError> {
    if !estimate.peak_convective_heat_flux_w_m2.is_finite()
        || !estimate.peak_heat_flux_altitude_m.is_finite()
        || !estimate.convective_heat_load_j_m2.is_finite()
    {
        return Err(AerothermalError::NonFinite {
            reason: "Allen-Eggers heating estimate is NaN or Inf",
        });
    }
    if estimate.peak_convective_heat_flux_w_m2 < 0.0 || estimate.convective_heat_load_j_m2 < 0.0 {
        return Err(AerothermalError::InvalidParameter {
            reason: "Allen-Eggers heating estimate must be non-negative",
        });
    }
    Ok(())
}

/// Fay-Riddell 1958 stagnation-point convective heating.
///
/// ```text
///   q_w = 0.94 · (ρ_w μ_w)^0.1 · (ρ_e μ_e)^0.4 · (du_e/dx)^0.5
///           · (h_aw - h_w) · [ 1 + (Le^a - 1) · (h_D / h_aw) ]
/// ```
///
/// # Honest Scope
///
/// The [`HeatTransferModel`] implementation uses cold-gas thermodynamics
/// with a fixed Sutherland viscosity law. The
/// [`Self::stagnation_from_edge_state`] method is the real-gas coupling
/// point: callers can provide edge and wall properties from an
/// equilibrium-air solver, CFD deck, or external reference package. The
/// in-crate real-gas shock-layer solver remains deferred until verified
/// equilibrium-air data land.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FayRiddell {
    /// Lewis number for the boundary layer (default 1.0 — the
    /// "near-unity-Le" simplification; equilibrium-air variants
    /// land in the follow-on slice).
    pub lewis_number: f64,
    /// Dissociation-enthalpy contribution `h_D` (J/kg). The cold-gas
    /// path ships the degenerate value `0.0`; the real-gas
    /// follow-on slice will compute this from the
    /// [`openbmp_physics::EquilibriumAir`] composition.
    pub h_dissociation_j_kg: f64,
}

impl Default for FayRiddell {
    fn default() -> Self {
        Self {
            lewis_number: 1.0,
            h_dissociation_j_kg: 0.0,
        }
    }
}

impl FayRiddell {
    /// Return a copy with `h_D` filled from a neutral dissociated-air
    /// composition.
    ///
    /// This is a narrow real-gas coupling helper for callers that
    /// already have an equilibrium or CFD composition at the boundary-
    /// layer edge but do not want to hand-compute the species formation
    /// enthalpy term. Ionised-air states should pass an explicitly
    /// evaluated `h_dissociation_j_kg` instead.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError`] when the composition contains
    /// non-finite charged fractions, has any ion/electron content, or
    /// cannot produce a finite neutral formation enthalpy.
    pub fn with_neutral_composition_enthalpy(
        self,
        composition: &AirComposition,
    ) -> Result<Self, AerothermalError> {
        validate_neutral_composition_for_fay_riddell(composition)?;
        let h_dissociation_j_kg = composition
            .neutral_formation_enthalpy_j_kg()
            .map_err(|error| map_air_composition_error(&error))?;
        if !h_dissociation_j_kg.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "neutral composition formation enthalpy is NaN or Inf",
            });
        }
        if h_dissociation_j_kg < 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "neutral composition formation enthalpy must be non-negative",
            });
        }
        Ok(Self {
            h_dissociation_j_kg,
            ..self
        })
    }

    /// Assemble Fay-Riddell heating from caller-supplied edge-state
    /// and wall properties.
    ///
    /// This method does not assume a perfect-gas shock or cold-air
    /// enthalpy model. All real-gas thermodynamic quantities enter
    /// through [`FayRiddellEdgeState`].
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError`] when model parameters or edge-state
    /// inputs are non-finite, non-positive where required, or produce
    /// a non-physical Lewis correction.
    pub fn stagnation_from_edge_state(
        &self,
        edge: &FayRiddellEdgeState,
        wall_catalysis: WallCatalysis,
    ) -> Result<StagnationHeating, AerothermalError> {
        edge.validate()?;
        if !self.lewis_number.is_finite() || self.lewis_number <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Fay-Riddell Lewis number must be positive and finite",
            });
        }
        if !self.h_dissociation_j_kg.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Fay-Riddell dissociation enthalpy is NaN or Inf",
            });
        }
        if matches!(wall_catalysis, WallCatalysis::Partial(eta) if !eta.is_finite()) {
            return Err(AerothermalError::NonFinite {
                reason: "Fay-Riddell wall-catalysis efficiency is NaN or Inf",
            });
        }

        let dh = (edge.adiabatic_wall_enthalpy_j_kg - edge.wall_enthalpy_j_kg).max(0.0);
        let a = wall_catalysis.lewis_exponent();
        let le_a = self.lewis_number.powf(a);
        let bracket = if edge.adiabatic_wall_enthalpy_j_kg > 0.0 {
            1.0 + (le_a - 1.0) * (self.h_dissociation_j_kg / edge.adiabatic_wall_enthalpy_j_kg)
        } else {
            1.0
        };
        if !bracket.is_finite() || bracket < 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Fay-Riddell Lewis correction must be finite and non-negative",
            });
        }

        let q_conv = 0.94
            * (edge.wall_density_kg_m3 * edge.wall_viscosity_pa_s).powf(0.1)
            * (edge.edge_density_kg_m3 * edge.edge_viscosity_pa_s).powf(0.4)
            * edge.velocity_gradient_s_inv.sqrt()
            * dh
            * bracket;
        if !q_conv.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Fay-Riddell heat flux is non-finite (check inputs)",
            });
        }
        Ok(StagnationHeating {
            q_conv_w_m2: q_conv,
            q_rad_w_m2: 0.0,
            h_aw_j_kg: edge.adiabatic_wall_enthalpy_j_kg,
            h_w_j_kg: edge.wall_enthalpy_j_kg,
            recovery_temperature_k: edge.recovery_temperature_k,
        })
    }
}

fn validate_neutral_composition_for_fay_riddell(
    composition: &AirComposition,
) -> Result<(), AerothermalError> {
    let ion_sum = composition.ion_mole_fraction_sum();
    let electrons = composition.electrons;
    if !ion_sum.is_finite() || !electrons.is_finite() {
        return Err(AerothermalError::NonFinite {
            reason: "Fay-Riddell neutral composition charged fraction is NaN or Inf",
        });
    }
    if ion_sum != 0.0 || electrons != 0.0 {
        return Err(AerothermalError::InvalidParameter {
            reason: "Fay-Riddell neutral composition helper does not accept ionised air",
        });
    }
    Ok(())
}

fn map_air_composition_error(error: &PhysicsError) -> AerothermalError {
    match error {
        PhysicsError::NonFinite { .. } => AerothermalError::NonFinite {
            reason: "neutral air-composition enthalpy input is NaN or Inf",
        },
        PhysicsError::InvalidParameter { .. }
        | PhysicsError::OutOfEnvelope { .. }
        | PhysicsError::Frame(_) => AerothermalError::InvalidParameter {
            reason: "neutral air-composition enthalpy input is invalid",
        },
    }
}

impl HeatTransferModel for FayRiddell {
    fn stagnation(&self, ctx: &AerothermalContext) -> Result<StagnationHeating, AerothermalError> {
        validate_context_for_stagnation(ctx)?;
        // Post-shock edge conditions for a normal shock in perfect-gas
        // air at γ = 1.4. The closed-form Rankine-Hugoniot relations
        // give edge density, temperature, and velocity from freestream
        // conditions. This is the cold-gas engineering form — real-gas
        // post-shock conditions land in the follow-on slice.
        let mach = ctx.mach;
        if mach <= 1.0 {
            return Err(AerothermalError::OutOfEnvelope {
                reason: "Fay-Riddell needs M > 1 (shock layer required)",
            });
        }
        let gamma = 1.4;
        let m2 = mach * mach;
        let rho_ratio = ((gamma + 1.0) * m2) / ((gamma - 1.0) * m2 + 2.0);
        let temp_ratio = (((2.0 * gamma) * m2 - (gamma - 1.0)) * ((gamma - 1.0) * m2 + 2.0))
            / ((gamma + 1.0) * (gamma + 1.0) * m2);
        let p_ratio = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (m2 - 1.0);
        let p_e = ctx.freestream.pressure_pa * p_ratio;
        let rho_e = ctx.freestream.density_kg_m3 * rho_ratio;
        let t_e = ctx.freestream.temperature_k * temp_ratio;
        // Stagnation velocity gradient for an axisymmetric blunt body:
        //   du_e/dx |_stag = (1 / R_n) · √(2 · (p_e − p_∞) / ρ_e)
        let dp = (p_e - ctx.freestream.pressure_pa).max(0.0);
        let dudx = (1.0 / ctx.nose_radius_m) * (2.0 * dp / rho_e.max(1.0e-30)).sqrt();
        // Wall properties: take wall temperature from the context;
        // density via ideal-gas at wall pressure.
        let rho_w = p_e / (287.05 * ctx.wall_temperature_k.max(50.0));
        let mu_w = sutherland_viscosity(ctx.wall_temperature_k);
        // Enthalpies.
        let v = ctx.airspeed_m_s;
        let adiabatic_wall_enthalpy = 0.5 * v * v + C_P_AIR_J_KG_K * ctx.freestream.temperature_k;
        let wall_enthalpy = C_P_AIR_J_KG_K * ctx.wall_temperature_k;
        let recovery = ctx.freestream.temperature_k + 0.5 * v * v / C_P_AIR_J_KG_K;
        let edge = FayRiddellEdgeState {
            edge_density_kg_m3: rho_e,
            edge_viscosity_pa_s: sutherland_viscosity(t_e),
            wall_density_kg_m3: rho_w,
            wall_viscosity_pa_s: mu_w,
            velocity_gradient_s_inv: dudx,
            adiabatic_wall_enthalpy_j_kg: adiabatic_wall_enthalpy,
            wall_enthalpy_j_kg: wall_enthalpy,
            recovery_temperature_k: recovery,
        };
        self.stagnation_from_edge_state(&edge, ctx.wall_catalysis)
    }
}

/// One point in the Tauber-Sutton tabulated velocity function.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TauberSuttonVelocityFunctionPoint {
    /// Freestream velocity (m/s).
    pub velocity_m_s: f64,
    /// Dimensionless Tauber-Sutton `f(V)` value.
    pub value: f64,
}

/// Provenance-pinned Tauber-Sutton coefficient and velocity-function package.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TauberSuttonPackage {
    /// Stable package identifier.
    pub source_id: &'static str,
    /// Payload SHA-256 hash as a lowercase hexadecimal string.
    pub payload_sha256_hex: &'static str,
    /// Retrieval path for the public table used by this package.
    pub retrieval_path: &'static str,
    /// Radiative coefficient in the Tauber-Sutton `W/cm²` formula units.
    pub coefficient_w_cm2: f64,
    /// Density exponent in `rho_inf^b`.
    pub density_exponent: f64,
    /// Coefficient in the uncapped radius-exponent law.
    pub radius_exponent_coefficient: f64,
    /// Velocity power in the uncapped radius-exponent law.
    pub radius_exponent_velocity_power: f64,
    /// Density power in the uncapped radius-exponent law.
    pub radius_exponent_density_power: f64,
    /// Ordered, monotone velocity function table.
    pub velocity_function: &'static [TauberSuttonVelocityFunctionPoint],
}

impl TauberSuttonPackage {
    /// Validate coefficient, exponent, provenance, and velocity-function shape.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError::InvalidParameter`] when the package is
    /// malformed.
    pub fn validate(&self) -> Result<(), AerothermalError> {
        if self.source_id.trim().is_empty()
            || self.payload_sha256_hex.trim().is_empty()
            || self.retrieval_path.trim().is_empty()
        {
            return Err(AerothermalError::InvalidParameter {
                reason: "Tauber-Sutton package provenance fields must be non-empty",
            });
        }
        if self.payload_sha256_hex.len() != 64
            || !self
                .payload_sha256_hex
                .chars()
                .all(|c| c.is_ascii_hexdigit())
        {
            return Err(AerothermalError::InvalidParameter {
                reason: "Tauber-Sutton payload hash must be 64 hex chars",
            });
        }
        if self.velocity_function.len() < 2 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Tauber-Sutton velocity function needs at least two points",
            });
        }
        if !self.coefficient_w_cm2.is_finite()
            || !self.density_exponent.is_finite()
            || !self.radius_exponent_coefficient.is_finite()
            || !self.radius_exponent_velocity_power.is_finite()
            || !self.radius_exponent_density_power.is_finite()
        {
            return Err(AerothermalError::InvalidParameter {
                reason: "Tauber-Sutton coefficient or exponent is NaN or Inf",
            });
        }
        if self.coefficient_w_cm2 <= 0.0 || self.radius_exponent_coefficient <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Tauber-Sutton coefficient and radius-exponent coefficient must be positive",
            });
        }

        let mut previous_velocity = f64::NEG_INFINITY;
        let mut previous_value = 0.0;
        for (index, point) in self.velocity_function.iter().enumerate() {
            if !point.velocity_m_s.is_finite() || !point.value.is_finite() {
                return Err(AerothermalError::InvalidParameter {
                    reason: "Tauber-Sutton velocity-function point is NaN or Inf",
                });
            }
            if point.velocity_m_s <= previous_velocity {
                return Err(AerothermalError::InvalidParameter {
                    reason: "Tauber-Sutton velocity function must be strictly increasing",
                });
            }
            if point.value < 0.0 {
                return Err(AerothermalError::InvalidParameter {
                    reason: "Tauber-Sutton velocity-function values must be non-negative",
                });
            }
            if index == 0 && point.value.abs() > f64::EPSILON {
                return Err(AerothermalError::InvalidParameter {
                    reason: "Tauber-Sutton velocity-function floor must be zero",
                });
            }
            if index > 0 && point.value + f64::EPSILON < previous_value {
                return Err(AerothermalError::InvalidParameter {
                    reason: "Tauber-Sutton velocity function must be monotone nondecreasing",
                });
            }
            previous_velocity = point.velocity_m_s;
            previous_value = point.value;
        }
        Ok(())
    }

    /// Return a provenance package carrying the payload hash and radiation-reference kind.
    #[must_use]
    pub fn reference_package(&self) -> ExternalReferencePackage {
        ExternalReferencePackage {
            kind: ReferencePackageKind::RadiationReference,
            envelope: EnvelopeBounds {
                mach_lo: 0.0,
                mach_hi: 100.0,
                altitude_lo_m: 0.0,
                altitude_hi_m: 250_000.0,
                alpha_lo_rad: -core::f64::consts::PI,
                alpha_hi_rad: core::f64::consts::PI,
            },
            provenance: ProvenanceBlock {
                solver_name: String::from(self.source_id),
                solver_version: String::from("table-v1"),
                license: String::from("public bibliographic reproduction"),
                retrieval_path: String::from(self.retrieval_path),
                governing_equations: String::from(
                    "Tauber-Sutton Earth-air stagnation-point radiative heating relation",
                ),
                content_hash_sha256_hex: String::from(self.payload_sha256_hex),
            },
        }
    }

    /// Interpolate the Tauber-Sutton velocity function in locked table order.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError::OutOfEnvelope`] outside the tabulated
    /// velocity range and [`AerothermalError::InvalidParameter`] for malformed
    /// package data.
    pub fn velocity_function_value(&self, velocity_m_s: f64) -> Result<f64, AerothermalError> {
        self.validate()?;
        if !velocity_m_s.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Tauber-Sutton velocity query is NaN or Inf",
            });
        }
        let first = self.velocity_function[0];
        let last = self.velocity_function[self.velocity_function.len() - 1];
        if velocity_m_s < first.velocity_m_s || velocity_m_s > last.velocity_m_s {
            return Err(AerothermalError::OutOfEnvelope {
                reason: "Tauber-Sutton velocity outside f(V) table",
            });
        }
        if (velocity_m_s - first.velocity_m_s).abs() <= f64::EPSILON {
            return Ok(first.value);
        }
        for window in self.velocity_function.windows(2) {
            let lo = window[0];
            let hi = window[1];
            if velocity_m_s <= hi.velocity_m_s {
                let span = hi.velocity_m_s - lo.velocity_m_s;
                let fraction = (velocity_m_s - lo.velocity_m_s) / span;
                return Ok(lo.value + fraction * (hi.value - lo.value));
            }
        }
        Ok(last.value)
    }

    /// Evaluate the capped Tauber-Sutton nose-radius exponent `a`.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError::InvalidParameter`] for non-finite or
    /// non-positive velocity, density, or nose radius.
    pub fn radius_exponent(
        &self,
        velocity_m_s: f64,
        density_kg_m3: f64,
        nose_radius_m: f64,
    ) -> Result<f64, AerothermalError> {
        self.validate()?;
        if !velocity_m_s.is_finite() || !density_kg_m3.is_finite() || !nose_radius_m.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Tauber-Sutton radius-exponent input is NaN or Inf",
            });
        }
        if velocity_m_s <= 0.0 || density_kg_m3 <= 0.0 || nose_radius_m <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Tauber-Sutton radius exponent requires positive velocity, density, and nose radius",
            });
        }
        let uncapped = self.radius_exponent_coefficient
            * velocity_m_s.powf(self.radius_exponent_velocity_power)
            * density_kg_m3.powf(self.radius_exponent_density_power);
        if !uncapped.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Tauber-Sutton radius exponent non-finite",
            });
        }
        let cap = if nose_radius_m < 1.0 {
            1.0
        } else if nose_radius_m <= 2.0 {
            0.6
        } else {
            0.5
        };
        Ok(uncapped.min(cap))
    }
}

/// Return the built-in Tauber-Sutton Earth-air package.
#[must_use]
pub const fn tauber_sutton_earth_air_package() -> TauberSuttonPackage {
    TauberSuttonPackage {
        source_id: "tauber-sutton-earth-air-radiative-v1",
        payload_sha256_hex: TAUBER_SUTTON_EARTH_AIR_PAYLOAD_SHA256_HEX,
        retrieval_path: "https://sciencejournals.ru/view-article/?a=MekhZhG1904003Brykina&j=mekhzhg&n=4&v=0&y=2019",
        coefficient_w_cm2: TAUBER_SUTTON_EARTH_AIR_COEFFICIENT_W_CM2,
        density_exponent: TAUBER_SUTTON_EARTH_AIR_DENSITY_EXPONENT,
        radius_exponent_coefficient: TAUBER_SUTTON_EARTH_AIR_RADIUS_EXPONENT_COEFFICIENT,
        radius_exponent_velocity_power: TAUBER_SUTTON_EARTH_AIR_RADIUS_EXPONENT_VELOCITY_POWER,
        radius_exponent_density_power: TAUBER_SUTTON_EARTH_AIR_RADIUS_EXPONENT_DENSITY_POWER,
        velocity_function: &TAUBER_SUTTON_EARTH_AIR_VELOCITY_FUNCTION,
    }
}

/// Tauber-Sutton radiative heating model.
///
/// The default constructor is intentionally inert and returns the same
/// fail-closed deferred error as the old typed-reserved surface. Call
/// [`Self::earth_air`] or [`Self::with_package`] to opt into an explicit
/// provenance-pinned package.
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct TauberSuttonRadiative {
    package: Option<TauberSuttonPackage>,
}

impl TauberSuttonRadiative {
    /// Construct the off-by-default deferred surface.
    #[must_use]
    pub const fn deferred() -> Self {
        Self { package: None }
    }

    /// Construct the built-in Earth-air Tauber-Sutton radiative model.
    #[must_use]
    pub const fn earth_air() -> Self {
        Self {
            package: Some(tauber_sutton_earth_air_package()),
        }
    }

    /// Construct a Tauber-Sutton model from an explicit package.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError::InvalidParameter`] when the package is malformed.
    pub fn with_package(package: TauberSuttonPackage) -> Result<Self, AerothermalError> {
        package.validate()?;
        Ok(Self {
            package: Some(package),
        })
    }

    /// Return the configured package, if this model is enabled.
    #[must_use]
    pub const fn package(&self) -> Option<TauberSuttonPackage> {
        self.package
    }
}

impl HeatTransferModel for TauberSuttonRadiative {
    fn stagnation(&self, ctx: &AerothermalContext) -> Result<StagnationHeating, AerothermalError> {
        validate_context_for_stagnation(ctx)?;
        let Some(package) = self.package else {
            return Err(AerothermalError::OutOfEnvelope {
                reason: "Tauber-Sutton radiative heating is deferred pending explicit package selection",
            });
        };
        if ctx.freestream.density_kg_m3 <= 0.0 {
            return Err(AerothermalError::OutOfEnvelope {
                reason: "Tauber-Sutton radiative heating requires positive freestream density",
            });
        }
        let f_v = package.velocity_function_value(ctx.airspeed_m_s)?;
        let radius_exponent = package.radius_exponent(
            ctx.airspeed_m_s,
            ctx.freestream.density_kg_m3,
            ctx.nose_radius_m,
        )?;
        let q_rad_w_m2 = package.coefficient_w_cm2
            * W_CM2_TO_W_M2
            * ctx.nose_radius_m.powf(radius_exponent)
            * ctx.freestream.density_kg_m3.powf(package.density_exponent)
            * f_v;
        if !q_rad_w_m2.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Tauber-Sutton radiative heat flux non-finite",
            });
        }
        Ok(StagnationHeating {
            q_conv_w_m2: 0.0,
            q_rad_w_m2,
            h_aw_j_kg: 0.0,
            h_w_j_kg: 0.0,
            recovery_temperature_k: 0.0,
        })
    }
}

fn validate_context_for_stagnation(ctx: &AerothermalContext) -> Result<(), AerothermalError> {
    if !ctx.freestream.pressure_pa.is_finite()
        || !ctx.freestream.temperature_k.is_finite()
        || !ctx.freestream.speed_of_sound_m_s.is_finite()
    {
        return Err(AerothermalError::NonFinite {
            reason: "freestream atmosphere component is NaN or Inf",
        });
    }
    if !ctx.freestream.density_kg_m3.is_finite()
        || !ctx.airspeed_m_s.is_finite()
        || !ctx.mach.is_finite()
        || !ctx.nose_radius_m.is_finite()
        || !ctx.wall_temperature_k.is_finite()
    {
        return Err(AerothermalError::NonFinite {
            reason: "stagnation context input is NaN or Inf",
        });
    }
    if ctx.freestream.density_kg_m3 < 0.0 {
        return Err(AerothermalError::InvalidParameter {
            reason: "freestream density must be ≥ 0",
        });
    }
    if ctx.freestream.pressure_pa < 0.0
        || ctx.freestream.temperature_k < 0.0
        || ctx.freestream.speed_of_sound_m_s < 0.0
    {
        return Err(AerothermalError::InvalidParameter {
            reason: "freestream atmosphere components must be non-negative",
        });
    }
    if ctx.mach < 0.0 {
        return Err(AerothermalError::InvalidParameter {
            reason: "Mach number must be non-negative",
        });
    }
    if ctx.nose_radius_m <= 0.0 {
        return Err(AerothermalError::InvalidParameter {
            reason: "nose radius must be > 0",
        });
    }
    if ctx.airspeed_m_s <= 0.0 {
        return Err(AerothermalError::InvalidParameter {
            reason: "airspeed must be > 0 for stagnation correlation",
        });
    }
    if ctx.wall_temperature_k <= 0.0 {
        return Err(AerothermalError::InvalidParameter {
            reason: "wall temperature must be > 0",
        });
    }
    if matches!(ctx.wall_catalysis, WallCatalysis::Partial(eta) if !eta.is_finite()) {
        return Err(AerothermalError::NonFinite {
            reason: "wall-catalysis efficiency is NaN or Inf",
        });
    }
    Ok(())
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::missing_panics_doc,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use sha2::{Digest, Sha256};

    fn sample_atmos(rho: f64, p: f64, t: f64) -> AtmosphereSample {
        AtmosphereSample::new(rho, p, t, (1.4 * 287.05 * t).sqrt()).unwrap()
    }

    fn ctx(rho: f64, v: f64, r_n: f64, wall_t: f64) -> AerothermalContext {
        let t_inf = 226.5; // ~70 km altitude reference
        let p_inf = rho * 287.05 * t_inf;
        let a_inf = (1.4 * 287.05 * t_inf).sqrt();
        AerothermalContext {
            freestream: sample_atmos(rho, p_inf, t_inf),
            airspeed_m_s: v,
            mach: v / a_inf,
            nose_radius_m: r_n,
            wall_temperature_k: wall_t,
            wall_catalysis: WallCatalysis::FullyCatalytic,
        }
    }

    fn dkr_fixture_value(key: &str) -> f64 {
        include_str!("../../../data/aerothermal/stagnation/detra-kemp-riddell-v1.toml")
            .lines()
            .find_map(|line| {
                let (name, value) = line.split_once('=')?;
                if name.trim() == key {
                    value.trim().parse::<f64>().ok()
                } else {
                    None
                }
            })
            .unwrap_or_else(|| panic!("missing DKR fixture value {key}"))
    }

    const TAUBER_SUTTON_FIXTURE: &str =
        include_str!("../../../data/aerothermal/radiative/tauber-sutton-earth-v1.toml");

    fn tauber_sutton_fixture_value(key: &str) -> f64 {
        TAUBER_SUTTON_FIXTURE
            .lines()
            .find_map(|line| {
                let (name, value) = line.split_once('=')?;
                if name.trim() == key {
                    value.trim().parse::<f64>().ok()
                } else {
                    None
                }
            })
            .unwrap_or_else(|| panic!("missing Tauber-Sutton fixture value {key}"))
    }

    fn tauber_sutton_fixture_velocity_function() -> Vec<(f64, f64)> {
        let mut points = Vec::new();
        let mut velocity_m_s = None;
        for line in TAUBER_SUTTON_FIXTURE.lines() {
            let Some((name, value)) = line.split_once('=') else {
                continue;
            };
            match name.trim() {
                "velocity_m_s" => {
                    velocity_m_s = value.trim().parse::<f64>().ok();
                }
                "f_v" => {
                    if let Some(velocity) = velocity_m_s.take() {
                        let f_v = value
                            .trim()
                            .parse::<f64>()
                            .unwrap_or_else(|_| panic!("malformed f_v value {value}"));
                        points.push((velocity, f_v));
                    }
                }
                _ => {}
            }
        }
        points
    }

    #[test]
    fn sutton_graves_at_textbook_apollo_point() {
        // Textbook Apollo-class re-entry sanity point:
        //   ρ ≈ 1e-4 kg/m³, V ≈ 11000 m/s, R_n = 1.5 m
        //   q_w = K · √(ρ/R_n) · V³
        //       = 1.7415e-4 · √(1e-4/1.5) · (11000)³
        //       ≈ 1.7415e-4 · 0.00816 · 1.331e12
        //       ≈ 1.89 MW/m²
        let s = SuttonGraves::default();
        let h = s.stagnation(&ctx(1.0e-4, 11_000.0, 1.5, 1500.0)).unwrap();
        assert!(
            (h.q_conv_w_m2 - 1.89e6).abs() / 1.89e6 < 0.05,
            "q_conv = {} W/m²",
            h.q_conv_w_m2
        );
    }

    #[test]
    fn sutton_graves_earth_air_preset_preserves_legacy_constant() {
        let default = SuttonGraves::default();
        let preset = SuttonGraves::for_gas(SuttonGravesGas::EarthAir);
        assert_eq!(
            preset.k_earth_si.to_bits(),
            SUTTON_GRAVES_K_EARTH_SI.to_bits()
        );

        let context = ctx(1.0e-4, 8_000.0, 1.0, 1500.0);
        let q_default = default.stagnation(&context).unwrap().q_conv_w_m2;
        let q_preset = preset.stagnation(&context).unwrap().q_conv_w_m2;
        assert_eq!(q_default.to_bits(), q_preset.to_bits());
    }

    #[test]
    fn sutton_graves_gas_presets_scale_from_tr_r376_table() {
        let co2 = SuttonGravesGas::CarbonDioxide.coefficient_si();
        let h2 = SuttonGravesGas::Hydrogen.coefficient_si();
        let ar = SuttonGravesGas::Argon.coefficient_si();

        assert_relative_eq!(
            co2 / SUTTON_GRAVES_K_EARTH_SI,
            0.1210 / SUTTON_GRAVES_TABLE_K_EARTH_AIR,
            max_relative = 1.0e-15
        );
        assert_relative_eq!(
            h2 / SUTTON_GRAVES_K_EARTH_SI,
            0.0395 / SUTTON_GRAVES_TABLE_K_EARTH_AIR,
            max_relative = 1.0e-15
        );
        assert_relative_eq!(
            ar / SUTTON_GRAVES_K_EARTH_SI,
            0.1495 / SUTTON_GRAVES_TABLE_K_EARTH_AIR,
            max_relative = 1.0e-15
        );
    }

    #[test]
    fn sutton_graves_for_gas_changes_heat_flux_by_coefficient_ratio() {
        let context = ctx(1.0e-4, 8_000.0, 1.0, 1500.0);
        let air = SuttonGraves::for_gas(SuttonGravesGas::EarthAir)
            .stagnation(&context)
            .unwrap()
            .q_conv_w_m2;
        let co2 = SuttonGraves::for_gas(SuttonGravesGas::CarbonDioxide)
            .stagnation(&context)
            .unwrap()
            .q_conv_w_m2;

        assert_relative_eq!(
            co2 / air,
            SuttonGravesGas::CarbonDioxide.table_heat_transfer_coefficient()
                / SuttonGravesGas::EarthAir.table_heat_transfer_coefficient(),
            max_relative = 1.0e-15
        );
    }

    #[test]
    fn detra_kemp_riddell_constants_match_fixture() {
        assert_eq!(
            DETRA_KEMP_RIDDELL_REFERENCE_HEAT_FLUX_BTU_FT2_S,
            dkr_fixture_value("reference_heat_flux_btu_ft2_s")
        );
        assert_eq!(
            DETRA_KEMP_RIDDELL_REFERENCE_VELOCITY_FT_S,
            dkr_fixture_value("reference_velocity_ft_s")
        );
        assert_eq!(
            DETRA_KEMP_RIDDELL_VELOCITY_EXPONENT,
            dkr_fixture_value("velocity_exponent")
        );
        assert_eq!(
            DETRA_KEMP_RIDDELL_SEA_LEVEL_DENSITY_KG_M3,
            dkr_fixture_value("sea_level_density_kg_m3")
        );
        assert_eq!(
            DETRA_KEMP_RIDDELL_REFERENCE_WALL_TEMPERATURE_K,
            dkr_fixture_value("reference_wall_temperature_k")
        );
    }

    #[test]
    fn detra_kemp_riddell_matches_synthetic_conversion_fixture() {
        let model = DetraKempRiddell::default();
        let context = ctx(
            dkr_fixture_value("synthetic_density_kg_m3"),
            dkr_fixture_value("synthetic_velocity_m_s"),
            dkr_fixture_value("synthetic_nose_radius_m"),
            dkr_fixture_value("synthetic_wall_temperature_k"),
        );

        let q_btu = model.cold_reference_heat_flux_btu_ft2_s(&context).unwrap();
        assert_relative_eq!(
            q_btu,
            dkr_fixture_value("synthetic_cold_reference_heat_flux_btu_ft2_s"),
            max_relative = 1.0e-14
        );

        let heating = model.stagnation(&context).unwrap();
        assert_relative_eq!(
            heating.q_conv_w_m2,
            dkr_fixture_value("synthetic_cold_reference_heat_flux_w_m2"),
            max_relative = 1.0e-14
        );
    }

    #[test]
    fn detra_kemp_riddell_hot_wall_correction_reduces_flux() {
        let model = DetraKempRiddell::default();
        let cold_context = ctx(
            dkr_fixture_value("synthetic_density_kg_m3"),
            dkr_fixture_value("synthetic_velocity_m_s"),
            dkr_fixture_value("synthetic_nose_radius_m"),
            dkr_fixture_value("synthetic_wall_temperature_k"),
        );
        let hot_context = ctx(
            dkr_fixture_value("synthetic_density_kg_m3"),
            dkr_fixture_value("synthetic_velocity_m_s"),
            dkr_fixture_value("synthetic_nose_radius_m"),
            dkr_fixture_value("synthetic_hot_wall_temperature_k"),
        );

        let cold = model.stagnation(&cold_context).unwrap();
        let hot = model.stagnation(&hot_context).unwrap();
        assert!(hot.q_conv_w_m2 < cold.q_conv_w_m2);
        assert_relative_eq!(
            hot.q_conv_w_m2 / cold.q_conv_w_m2,
            dkr_fixture_value("synthetic_hot_wall_correction"),
            max_relative = 1.0e-14
        );
        assert_relative_eq!(
            hot.q_conv_w_m2,
            dkr_fixture_value("synthetic_hot_wall_heat_flux_w_m2"),
            max_relative = 1.0e-14
        );
    }

    #[test]
    fn detra_kemp_riddell_rejects_invalid_inputs() {
        let context = ctx(1.0e-4, 8_000.0, 1.0, 300.0);
        assert!(matches!(
            DetraKempRiddell::with_sea_level_density(0.0).stagnation(&context),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        assert!(matches!(
            DetraKempRiddell::with_sea_level_density(f64::NAN).stagnation(&context),
            Err(AerothermalError::NonFinite { .. })
        ));
        assert!(matches!(
            DetraKempRiddell::default().stagnation(&ctx(1.0e-4, 0.0, 1.0, 300.0)),
            Err(AerothermalError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn sutton_graves_scales_with_velocity_cubed() {
        let s = SuttonGraves::default();
        let q1 = s
            .stagnation(&ctx(1.0e-4, 5_000.0, 1.0, 1500.0))
            .unwrap()
            .q_conv_w_m2;
        let q2 = s
            .stagnation(&ctx(1.0e-4, 10_000.0, 1.0, 1500.0))
            .unwrap()
            .q_conv_w_m2;
        // q ∝ V³ → doubling V → 8× q
        assert_relative_eq!(q2 / q1, 8.0, max_relative = 1e-9);
    }

    #[test]
    fn sutton_graves_scales_with_sqrt_density_over_radius() {
        let s = SuttonGraves::default();
        let q_small = s
            .stagnation(&ctx(1.0e-4, 5000.0, 1.0, 1500.0))
            .unwrap()
            .q_conv_w_m2;
        let q_big_radius = s
            .stagnation(&ctx(1.0e-4, 5000.0, 4.0, 1500.0))
            .unwrap()
            .q_conv_w_m2;
        // q ∝ 1/√R → q(R=4) / q(R=1) = 1/2
        assert_relative_eq!(q_big_radius / q_small, 0.5, max_relative = 1e-9);
    }

    #[test]
    fn sutton_graves_wall_enthalpy_correction_reduces_hot_wall_flux() {
        let s = SuttonGraves::default();
        let context = ctx(1.0e-4, 5_000.0, 1.0, 3_500.0);
        let cold_wall = s.stagnation(&context).unwrap();
        let corrected = s
            .stagnation_with_wall_enthalpy_correction(&context)
            .unwrap();

        assert!(corrected.q_conv_w_m2 < cold_wall.q_conv_w_m2);
        assert_eq!(corrected.h_aw_j_kg.to_bits(), cold_wall.h_aw_j_kg.to_bits());
        assert_eq!(corrected.h_w_j_kg.to_bits(), cold_wall.h_w_j_kg.to_bits());
    }

    #[test]
    fn allen_eggers_heating_peak_is_above_peak_deceleration() {
        let entry = AllenEggers {
            rho_s_kg_m3: 1.225,
            beta_inv_m: 1.0 / 7_000.0,
            entry_velocity_m_s: 7_800.0,
            flight_path_angle_rad: 5.0_f64.to_radians(),
            ballistic_coefficient_m2_kg: 0.001,
        };
        let estimate = SuttonGraves::default()
            .allen_eggers_heating(&entry, 1.0)
            .unwrap();

        assert!(estimate.peak_convective_heat_flux_w_m2 > 0.0);
        assert!(estimate.convective_heat_load_j_m2 > 0.0);
        assert!(estimate.peak_heat_flux_altitude_m > entry.peak_decel_altitude_m());
        assert_relative_eq!(
            estimate.peak_heat_flux_altitude_m - entry.peak_decel_altitude_m(),
            3.0_f64.ln() / entry.beta_inv_m,
            max_relative = 1.0e-12
        );
    }

    #[test]
    fn allen_eggers_heating_closed_form_matches_profile_quadrature() {
        let entry = AllenEggers {
            rho_s_kg_m3: 1.225,
            beta_inv_m: 1.0 / 7_000.0,
            entry_velocity_m_s: 7_800.0,
            flight_path_angle_rad: 5.0_f64.to_radians(),
            ballistic_coefficient_m2_kg: 0.001,
        };
        let nose_radius_m = 1.0;
        let model = SuttonGraves::default();
        let estimate = model.allen_eggers_heating(&entry, nose_radius_m).unwrap();
        let sin_gamma = entry.flight_path_angle_rad.sin().abs();
        let mut heat_load = 0.0;
        let mut peak_flux = 0.0;
        let mut peak_altitude_m = 0.0;
        let step_m = 10.0;
        let mut previous_q = 0.0;
        let mut previous_dt_dh = 0.0;
        let mut first = true;

        for i in 0..=20_000 {
            let altitude_m = 200_000.0 - f64::from(i) * step_m;
            let rho = entry.rho_s_kg_m3 * (-entry.beta_inv_m * altitude_m).exp();
            let velocity = entry.velocity_at_altitude_m_s(altitude_m);
            let q = model.k_earth_si * (rho / nose_radius_m).sqrt() * velocity.powi(3);
            let dt_dh = 1.0 / (velocity * sin_gamma);
            if q > peak_flux {
                peak_flux = q;
                peak_altitude_m = altitude_m;
            }
            if first {
                first = false;
            } else {
                heat_load += 0.5 * (previous_q * previous_dt_dh + q * dt_dh) * step_m;
            }
            previous_q = q;
            previous_dt_dh = dt_dh;
        }

        assert_relative_eq!(
            peak_flux,
            estimate.peak_convective_heat_flux_w_m2,
            max_relative = 1.0e-5
        );
        assert!(
            (peak_altitude_m - estimate.peak_heat_flux_altitude_m).abs() <= step_m,
            "quadrature peak h={peak_altitude_m}, closed form h={}",
            estimate.peak_heat_flux_altitude_m
        );
        assert_relative_eq!(
            heat_load,
            estimate.convective_heat_load_j_m2,
            max_relative = 1.0e-4
        );
    }

    #[test]
    fn allen_eggers_heating_brackets_public_stardust_traj_case() {
        use openbmp_physics::{STARDUST_SRC_TABLE19_TRAJ_INPUT, STARDUST_SRC_TABLE20_TRAJ_OUTPUT};

        let input = STARDUST_SRC_TABLE19_TRAJ_INPUT;
        let output = STARDUST_SRC_TABLE20_TRAJ_OUTPUT;
        let entry = AllenEggers {
            rho_s_kg_m3: 1.225,
            beta_inv_m: 1.0 / 7_000.0,
            entry_velocity_m_s: input.relative_velocity_m_s,
            flight_path_angle_rad: input.relative_entry_angle_below_horizon_rad,
            ballistic_coefficient_m2_kg: 1.0 / input.ballistic_parameter_kg_m2,
        };
        let estimate = SuttonGraves::default()
            .allen_eggers_heating(&entry, input.nose_radius_m)
            .unwrap();

        assert!(
            (estimate.peak_convective_heat_flux_w_m2 - output.peak_convective_heat_flux.value_si)
                .abs()
                / output.peak_convective_heat_flux.value_si
                < 0.40,
            "Sutton-Graves / Allen-Eggers q_peak={} W/m², Stardust table20={} W/m²",
            estimate.peak_convective_heat_flux_w_m2,
            output.peak_convective_heat_flux.value_si
        );
        assert!(
            (estimate.peak_heat_flux_altitude_m - output.peak_convective_heat_flux.altitude_m)
                .abs()
                < 2_000.0,
            "Sutton-Graves / Allen-Eggers h_peak={} m, Stardust table20={} m",
            estimate.peak_heat_flux_altitude_m,
            output.peak_convective_heat_flux.altitude_m
        );
        assert!(
            (estimate.convective_heat_load_j_m2 - output.convective_heat_load_j_m2).abs()
                / output.convective_heat_load_j_m2
                < 0.20,
            "Sutton-Graves / Allen-Eggers Q={} J/m², Stardust table20={} J/m²",
            estimate.convective_heat_load_j_m2,
            output.convective_heat_load_j_m2
        );
    }

    #[test]
    fn fay_riddell_positive_and_finite_at_textbook_point() {
        let m = FayRiddell::default();
        let result = m.stagnation(&ctx(1.0e-4, 5000.0, 1.0, 1500.0));
        let h = result.unwrap();
        assert!(h.q_conv_w_m2 > 0.0);
        assert!(h.q_conv_w_m2.is_finite());
    }

    fn cold_gas_edge_for(ctx: &AerothermalContext) -> FayRiddellEdgeState {
        let gamma = 1.4;
        let mach = ctx.mach;
        let m2 = mach * mach;
        let rho_ratio = ((gamma + 1.0) * m2) / ((gamma - 1.0) * m2 + 2.0);
        let temp_ratio = (((2.0 * gamma) * m2 - (gamma - 1.0)) * ((gamma - 1.0) * m2 + 2.0))
            / ((gamma + 1.0) * (gamma + 1.0) * m2);
        let p_ratio = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (m2 - 1.0);
        let p_e = ctx.freestream.pressure_pa * p_ratio;
        let rho_e = ctx.freestream.density_kg_m3 * rho_ratio;
        let t_e = ctx.freestream.temperature_k * temp_ratio;
        let dp = (p_e - ctx.freestream.pressure_pa).max(0.0);
        let dudx = (1.0 / ctx.nose_radius_m) * (2.0 * dp / rho_e.max(1.0e-30)).sqrt();
        let v = ctx.airspeed_m_s;
        FayRiddellEdgeState {
            edge_density_kg_m3: rho_e,
            edge_viscosity_pa_s: sutherland_viscosity(t_e),
            wall_density_kg_m3: p_e / (287.05 * ctx.wall_temperature_k.max(50.0)),
            wall_viscosity_pa_s: sutherland_viscosity(ctx.wall_temperature_k),
            velocity_gradient_s_inv: dudx,
            adiabatic_wall_enthalpy_j_kg: 0.5 * v * v
                + C_P_AIR_J_KG_K * ctx.freestream.temperature_k,
            wall_enthalpy_j_kg: C_P_AIR_J_KG_K * ctx.wall_temperature_k,
            recovery_temperature_k: ctx.freestream.temperature_k + 0.5 * v * v / C_P_AIR_J_KG_K,
        }
    }

    #[test]
    fn fay_riddell_edge_state_matches_cold_gas_trait_path() {
        let m = FayRiddell::default();
        let context = ctx(1.0e-4, 5000.0, 1.0, 1500.0);
        let cold = m.stagnation(&context).unwrap();
        let edge = cold_gas_edge_for(&context);
        let via_edge = m
            .stagnation_from_edge_state(&edge, context.wall_catalysis)
            .unwrap();
        assert_eq!(cold.q_conv_w_m2.to_bits(), via_edge.q_conv_w_m2.to_bits());
        assert_eq!(cold.h_aw_j_kg.to_bits(), via_edge.h_aw_j_kg.to_bits());
    }

    #[test]
    fn fay_riddell_edge_state_accepts_caller_supplied_real_gas_enthalpy() {
        let m = FayRiddell {
            lewis_number: 1.2,
            h_dissociation_j_kg: 2.0e6,
        };
        let edge = FayRiddellEdgeState {
            edge_density_kg_m3: 2.0e-4,
            edge_viscosity_pa_s: 8.0e-5,
            wall_density_kg_m3: 5.0e-4,
            wall_viscosity_pa_s: 6.0e-5,
            velocity_gradient_s_inv: 8.0e4,
            adiabatic_wall_enthalpy_j_kg: 2.8e7,
            wall_enthalpy_j_kg: 1.2e6,
            recovery_temperature_k: 3_000.0,
        };
        let base = m
            .stagnation_from_edge_state(&edge, WallCatalysis::FullyCatalytic)
            .unwrap();
        let hotter = m
            .stagnation_from_edge_state(
                &FayRiddellEdgeState {
                    adiabatic_wall_enthalpy_j_kg: 3.2e7,
                    ..edge
                },
                WallCatalysis::FullyCatalytic,
            )
            .unwrap();
        assert!(base.q_conv_w_m2.is_finite() && base.q_conv_w_m2 > 0.0);
        assert!(hotter.q_conv_w_m2 > base.q_conv_w_m2);
    }

    #[test]
    fn fay_riddell_uses_neutral_composition_enthalpy() {
        let composition = AirComposition {
            n2: 0.70,
            o2: 0.18,
            o_atomic: 0.08,
            no: 0.04,
            ..AirComposition::default()
        };
        let base = FayRiddell {
            lewis_number: 1.2,
            h_dissociation_j_kg: 0.0,
        };
        let enriched = base
            .with_neutral_composition_enthalpy(&composition)
            .unwrap();
        let edge = FayRiddellEdgeState {
            edge_density_kg_m3: 2.0e-4,
            edge_viscosity_pa_s: 8.0e-5,
            wall_density_kg_m3: 5.0e-4,
            wall_viscosity_pa_s: 6.0e-5,
            velocity_gradient_s_inv: 8.0e4,
            adiabatic_wall_enthalpy_j_kg: 2.8e7,
            wall_enthalpy_j_kg: 1.2e6,
            recovery_temperature_k: 3_000.0,
        };

        assert!(enriched.h_dissociation_j_kg > 0.0);
        let cold = base
            .stagnation_from_edge_state(&edge, WallCatalysis::FullyCatalytic)
            .unwrap();
        let real_gas = enriched
            .stagnation_from_edge_state(&edge, WallCatalysis::FullyCatalytic)
            .unwrap();
        assert!(real_gas.q_conv_w_m2 > cold.q_conv_w_m2);
    }

    #[test]
    fn fay_riddell_neutral_composition_helper_rejects_ionised_air() {
        let composition = AirComposition {
            n2: 0.90,
            n_ion: 0.05,
            electrons: 0.05,
            ..AirComposition::default()
        };
        assert!(matches!(
            FayRiddell::default().with_neutral_composition_enthalpy(&composition),
            Err(AerothermalError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn fay_riddell_edge_state_rejects_non_physical_inputs() {
        let m = FayRiddell::default();
        let edge = FayRiddellEdgeState {
            edge_density_kg_m3: 0.0,
            edge_viscosity_pa_s: 8.0e-5,
            wall_density_kg_m3: 5.0e-4,
            wall_viscosity_pa_s: 6.0e-5,
            velocity_gradient_s_inv: 8.0e4,
            adiabatic_wall_enthalpy_j_kg: 2.8e7,
            wall_enthalpy_j_kg: 1.2e6,
            recovery_temperature_k: 3_000.0,
        };
        assert!(matches!(
            m.stagnation_from_edge_state(&edge, WallCatalysis::FullyCatalytic),
            Err(AerothermalError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn fay_riddell_rejects_subsonic() {
        let m = FayRiddell::default();
        let result = m.stagnation(&ctx(1.225, 100.0, 1.0, 300.0));
        assert!(matches!(
            result,
            Err(AerothermalError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn tauber_sutton_default_is_off_by_default() {
        let t = TauberSuttonRadiative::default();
        assert!(matches!(
            t.stagnation(&ctx(1.0e-4, 11_000.0, 1.5, 1500.0)),
            Err(AerothermalError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn tauber_sutton_validates_context_before_deferred_error() {
        let t = TauberSuttonRadiative::default();
        let mut context = ctx(1.0e-4, 11_000.0, 1.5, 1500.0);
        context.mach = f64::NAN;
        assert!(matches!(
            t.stagnation(&context),
            Err(AerothermalError::NonFinite { .. })
        ));
    }

    #[test]
    fn tauber_sutton_payload_hash_matches_fixture() {
        let digest = Sha256::digest(TAUBER_SUTTON_FIXTURE.as_bytes());
        let digest_hex = format!("{digest:x}");
        assert_eq!(digest_hex, TAUBER_SUTTON_EARTH_AIR_PAYLOAD_SHA256_HEX);

        let package = tauber_sutton_earth_air_package();
        package
            .reference_package()
            .validate_payload_hash_hex(TAUBER_SUTTON_EARTH_AIR_PAYLOAD_SHA256_HEX)
            .unwrap();
    }

    #[test]
    fn tauber_sutton_package_matches_fixture_table() {
        let package = tauber_sutton_earth_air_package();
        assert_relative_eq!(
            package.coefficient_w_cm2,
            tauber_sutton_fixture_value("coefficient_w_cm2"),
            max_relative = 1.0e-15
        );
        assert_relative_eq!(
            package.density_exponent,
            tauber_sutton_fixture_value("density_exponent"),
            max_relative = 1.0e-15
        );
        assert_relative_eq!(
            package.radius_exponent_coefficient,
            tauber_sutton_fixture_value("radius_exponent_coefficient"),
            max_relative = 1.0e-15
        );

        let table = tauber_sutton_fixture_velocity_function();
        assert_eq!(table.len(), TAUBER_SUTTON_EARTH_AIR_VELOCITY_FUNCTION.len());
        for (point, (velocity_m_s, f_v)) in
            TAUBER_SUTTON_EARTH_AIR_VELOCITY_FUNCTION.iter().zip(table)
        {
            assert_relative_eq!(point.velocity_m_s, velocity_m_s, max_relative = 1.0e-15);
            assert_relative_eq!(point.value, f_v, max_relative = 1.0e-15);
        }
    }

    #[test]
    fn tauber_sutton_velocity_function_interpolates_and_refuses_envelope() {
        let package = tauber_sutton_earth_air_package();
        assert_relative_eq!(
            package.velocity_function_value(9_000.0).unwrap(),
            0.0,
            epsilon = 0.0
        );
        assert_relative_eq!(
            package.velocity_function_value(10_125.0).unwrap(),
            45.0,
            max_relative = 1.0e-15
        );
        assert_relative_eq!(
            package.velocity_function_value(16_000.0).unwrap(),
            2_040.0,
            max_relative = 1.0e-15
        );
        assert!(matches!(
            package.velocity_function_value(8_999.0),
            Err(AerothermalError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            package.velocity_function_value(16_001.0),
            Err(AerothermalError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn tauber_sutton_earth_air_matches_reference_case() {
        let package = tauber_sutton_earth_air_package();
        let density = tauber_sutton_fixture_value("density_kg_m3");
        let velocity = tauber_sutton_fixture_value("velocity_m_s");
        let nose_radius = tauber_sutton_fixture_value("nose_radius_m");
        let wall_temperature = tauber_sutton_fixture_value("wall_temperature_k");
        let tolerance = tauber_sutton_fixture_value("relative_tolerance");

        assert_relative_eq!(
            package
                .radius_exponent(velocity, density, nose_radius)
                .unwrap(),
            tauber_sutton_fixture_value("reference_radius_exponent"),
            max_relative = tolerance
        );
        assert_relative_eq!(
            package.velocity_function_value(velocity).unwrap(),
            tauber_sutton_fixture_value("reference_f_v"),
            max_relative = tolerance
        );

        let heating = TauberSuttonRadiative::earth_air()
            .stagnation(&ctx(density, velocity, nose_radius, wall_temperature))
            .unwrap();
        assert_eq!(heating.q_conv_w_m2.to_bits(), 0.0_f64.to_bits());
        assert_relative_eq!(
            heating.q_rad_w_m2,
            tauber_sutton_fixture_value("reference_q_rad_w_m2"),
            max_relative = tolerance
        );
    }

    #[test]
    fn tauber_sutton_rejects_malformed_package() {
        static NONMONOTONE_TABLE: [TauberSuttonVelocityFunctionPoint; 2] = [
            TauberSuttonVelocityFunctionPoint {
                velocity_m_s: 9_000.0,
                value: 0.0,
            },
            TauberSuttonVelocityFunctionPoint {
                velocity_m_s: 8_500.0,
                value: 35.0,
            },
        ];
        let mut package = tauber_sutton_earth_air_package();
        package.velocity_function = &NONMONOTONE_TABLE;
        assert!(matches!(
            TauberSuttonRadiative::with_package(package),
            Err(AerothermalError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tauber_sutton_total_stagnation_heating_sums_convective_and_radiative_terms() {
        let context = ctx(1.0e-4, 11_000.0, 1.5, 1500.0);
        let convective = SuttonGraves::default().stagnation(&context).unwrap();
        let radiative = TauberSuttonRadiative::earth_air()
            .stagnation(&context)
            .unwrap();
        let total = TotalStagnationHeating::new(
            SuttonGraves::default(),
            TauberSuttonRadiative::earth_air(),
        )
        .stagnation(&context)
        .unwrap();
        assert_relative_eq!(
            total.q_conv_w_m2,
            convective.q_conv_w_m2,
            max_relative = 1.0e-15
        );
        assert_relative_eq!(
            total.q_rad_w_m2,
            radiative.q_rad_w_m2,
            max_relative = 1.0e-15
        );
        assert_eq!(total.h_aw_j_kg.to_bits(), convective.h_aw_j_kg.to_bits());
        assert_eq!(total.h_w_j_kg.to_bits(), convective.h_w_j_kg.to_bits());
    }

    #[test]
    fn sutherland_viscosity_at_273k_matches_reference() {
        let mu = sutherland_viscosity(273.15);
        assert_relative_eq!(mu, 1.716e-5, max_relative = 1e-6);
    }

    #[test]
    fn determinism_two_runs_byte_identical() {
        let s = SuttonGraves::default();
        let a = s.stagnation(&ctx(1.0e-4, 8000.0, 1.0, 1500.0)).unwrap();
        let b = s.stagnation(&ctx(1.0e-4, 8000.0, 1.0, 1500.0)).unwrap();
        assert_eq!(a.q_conv_w_m2.to_bits(), b.q_conv_w_m2.to_bits());
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        let s = SuttonGraves::default();
        assert!(matches!(
            s.stagnation(&ctx(1.0e-4, 0.0, 1.0, 1500.0)),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        assert!(matches!(
            s.stagnation(&ctx(1.0e-4, 5000.0, 0.0, 1500.0)),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        assert!(matches!(
            s.stagnation(&ctx(1.0e-4, 5000.0, 1.0, 0.0)),
            Err(AerothermalError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn invalid_model_parameters_are_rejected() {
        let s = SuttonGraves { k_earth_si: -1.0 };
        assert!(matches!(
            s.stagnation(&ctx(1.0e-4, 5000.0, 1.0, 1500.0)),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        let s = SuttonGraves {
            k_earth_si: f64::NAN,
        };
        assert!(matches!(
            s.stagnation(&ctx(1.0e-4, 5000.0, 1.0, 1500.0)),
            Err(AerothermalError::NonFinite { .. })
        ));
    }

    #[test]
    fn non_finite_wall_catalysis_is_rejected() {
        let mut context = ctx(1.0e-4, 5000.0, 1.0, 1500.0);
        context.wall_catalysis = WallCatalysis::Partial(f64::NAN);
        let s = SuttonGraves::default();
        assert!(matches!(
            s.stagnation(&context),
            Err(AerothermalError::NonFinite { .. })
        ));
    }
}
