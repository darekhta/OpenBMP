//! Phase-6.4 stagnation-point heat-transfer models.
//!
//! [`FayRiddell`] exposes both the cold-gas `HeatTransferModel` scaffold
//! and a caller-supplied edge-state assembly path for the 1958
//! equilibrium-air catalytic-wall axisymmetric formula. [`SuttonGraves`]
//! is the engineering simplification that needs only freestream density,
//! nose radius, and velocity. [`TauberSuttonRadiative`] is typed-reserved
//! until the published piecewise-polynomial Tauber-Sutton coefficients
//! are imported with provenance.

use openbmp_physics::AtmosphereSample;

use crate::error::AerothermalError;

/// Body station identifier used by distributed-heating consumers.
/// Phase-6.4 stagnation models only need station == stagnation point;
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

/// Fay-Riddell boundary-layer edge and wall properties.
///
/// This is the real-gas coupling point for the Phase-6.4 Fay-Riddell
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

/// Reference air heat capacity (perfect-gas, cold-air) used for
/// `h = c_p · T` shortcuts. Real-gas h(T,p) is the
/// [`openbmp_physics::EquilibriumAir`] consumer's job; the
/// engineering Fay-Riddell shipped here uses the cold-gas value to
/// keep the standalone module honest.
const C_P_AIR_J_KG_K: f64 = 1005.0;

/// Sutton-Graves engineering stagnation-point convective heating
/// constant for Earth atmosphere (SI units).
pub const SUTTON_GRAVES_K_EARTH_SI: f64 = 1.7415e-4;

/// Sutton-Graves engineering correlation.
///
/// `q_w = K · √(ρ_∞ / R_n) · V_∞³` — quick conservative bound and
/// sanity check; no real-gas iteration, no boundary-layer state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SuttonGraves {
    /// Earth-atmosphere constant. Use [`SUTTON_GRAVES_K_EARTH_SI`]
    /// unless the scenario opts into a different planetary body
    /// (Phase 6 is Earth-only, so this is the value most callers
    /// want).
    pub k_earth_si: f64,
}

impl Default for SuttonGraves {
    fn default() -> Self {
        Self {
            k_earth_si: SUTTON_GRAVES_K_EARTH_SI,
        }
    }
}

impl HeatTransferModel for SuttonGraves {
    fn stagnation(&self, ctx: &AerothermalContext) -> Result<StagnationHeating, AerothermalError> {
        validate_context_for_stagnation(ctx)?;
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
    /// Lewis number for the boundary layer (default 1.0 — Phase-6
    /// "near-unity-Le" simplification; equilibrium-air variants
    /// land in the follow-on slice).
    pub lewis_number: f64,
    /// Dissociation-enthalpy contribution `h_D` (J/kg). Phase-6.4
    /// ships the cold-gas degenerate value `0.0`; the real-gas
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

/// Tauber-Sutton radiative heating typed-reserved surface.
///
/// # Honest Scope
///
/// Reserved until the published Earth-entry piecewise-polynomial
/// velocity function and coefficients are imported with provenance.
/// The initial Phase-6 implementation used a tuned power law; the
/// audit removed that executable approximation rather than shipping
/// an unverifiable radiative-heating value.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TauberSuttonRadiative;

impl Default for TauberSuttonRadiative {
    fn default() -> Self {
        Self
    }
}

impl HeatTransferModel for TauberSuttonRadiative {
    fn stagnation(&self, ctx: &AerothermalContext) -> Result<StagnationHeating, AerothermalError> {
        validate_context_for_stagnation(ctx)?;
        Err(AerothermalError::OutOfEnvelope {
            reason: "Tauber-Sutton radiative heating is deferred pending published coefficients",
        })
    }
}

/// Sutherland viscosity law for air.
///
/// `μ(T) = μ_ref · (T / T_ref)^1.5 · (T_ref + S) / (T + S)` with
/// `T_ref = 273.15`, `μ_ref = 1.716e-5 Pa·s`, `S = 110.4 K`. Pure
/// `f64::powf`. State-stable across platforms.
#[must_use]
pub fn sutherland_viscosity(temperature_k: f64) -> f64 {
    const MU_REF: f64 = 1.716e-5;
    const T_REF: f64 = 273.15;
    const S_K: f64 = 110.4;
    if !temperature_k.is_finite() || temperature_k <= 0.0 {
        return 0.0;
    }
    let t_ratio = temperature_k / T_REF;
    let pow_15 = t_ratio.powf(1.5);
    MU_REF * pow_15 * (T_REF + S_K) / (temperature_k + S_K)
}

fn validate_context_for_stagnation(ctx: &AerothermalContext) -> Result<(), AerothermalError> {
    if !ctx.freestream.density_kg_m3.is_finite()
        || !ctx.airspeed_m_s.is_finite()
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
    fn tauber_sutton_is_deferred_until_coefficients_land() {
        let t = TauberSuttonRadiative;
        assert!(matches!(
            t.stagnation(&ctx(1.0e-4, 11_000.0, 1.5, 1500.0)),
            Err(AerothermalError::OutOfEnvelope { .. })
        ));
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
}
