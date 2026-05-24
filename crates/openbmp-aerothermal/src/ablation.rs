//! Phase-6.11 generic ablation toy.
//!
//! Trait-level surface for ablation models with two implementations:
//!
//! * [`SteadyStateAblator`] — quasi-steady-state surface energy
//!   balance. Useful for back-of-the-envelope studies and early-stage
//!   trajectory analysis.
//! * [`CharringAblator`] — charring ablator with pyrolysis zone.
//!   Tracks virgin / char composition.
//!
//! No real fielded TPS material parameters ship — only generic
//! textbook archetypes.

use crate::error::AerothermalError;
use crate::stagnation::{AerothermalContext, BodyStation};

/// Surface state at a body station.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SurfaceState {
    /// Virgin (un-ablated) material.
    Virgin,
    /// Active pyrolysis — `progress ∈ [0, 1]`, 0 = virgin, 1 = char.
    Pyrolyzing {
        /// Pyrolysis progress.
        progress: f64,
    },
    /// Fully charred surface.
    Char,
    /// Sublimating ablator.
    Sublimating {
        /// Current recession rate (m/s).
        recession_rate_m_s: f64,
    },
    /// Steady-state ablating equilibrium.
    SteadyAblating {
        /// Current recession rate (m/s).
        recession_rate_m_s: f64,
    },
}

/// Per-station recession rate (m/s).
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct RecessionRate {
    /// Recession rate (m/s) — surface moves into the body at this
    /// rate when positive.
    pub m_per_s: f64,
}

/// Generic toy ablator material — textbook scope only.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ToyAblator {
    /// Textbook material name (e.g. "graphite-toy",
    /// "generic-charring-1").
    pub name: &'static str,
    /// Density (kg/m³).
    pub density_kg_m3: f64,
    /// Specific heat capacity (J/(kg·K)).
    pub specific_heat_j_kg_k: f64,
    /// Thermal conductivity (W/(m·K)).
    pub thermal_conductivity_w_m_k: f64,
    /// Heat of ablation `h_v` (J/kg).
    pub heat_of_ablation_j_kg: f64,
    /// Vaporisation / decomposition temperature (K).
    pub vaporisation_temperature_k: f64,
    /// Effective surface emissivity (0..=1).
    pub surface_emissivity: f64,
}

impl ToyAblator {
    /// Textbook graphite sublimator.
    #[must_use]
    pub const fn graphite_toy() -> Self {
        Self {
            name: "graphite-toy",
            density_kg_m3: 1800.0,
            specific_heat_j_kg_k: 1700.0,
            thermal_conductivity_w_m_k: 30.0,
            heat_of_ablation_j_kg: 4.0e7,
            vaporisation_temperature_k: 3900.0,
            surface_emissivity: 0.85,
        }
    }

    /// Textbook generic charring ablator (typical academic range).
    #[must_use]
    pub const fn generic_charring_1() -> Self {
        Self {
            name: "generic-charring-1",
            density_kg_m3: 1450.0,
            specific_heat_j_kg_k: 1500.0,
            thermal_conductivity_w_m_k: 0.4,
            heat_of_ablation_j_kg: 1.5e7,
            vaporisation_temperature_k: 1200.0,
            surface_emissivity: 0.8,
        }
    }
}

/// Blowing-correction correlation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BlowingCorrelation {
    /// `λ` constant blowing-reduction parameter.
    FixedLambda {
        /// `λ ∈ [0, 1]`.
        lambda: f64,
    },
    /// Lees (1958) blowing correction.
    Lees,
    /// Spalding-Chi blowing correction.
    SpaldingChi,
}

impl BlowingCorrelation {
    /// Apply correction: `q_blow = q_no_blow · (1 - λ · B)`.
    #[must_use]
    pub fn apply(self, q_no_blowing_w_m2: f64, blowing_parameter: f64) -> f64 {
        let lambda = match self {
            Self::FixedLambda { lambda } => lambda.clamp(0.0, 1.0),
            // Lees: λ ≈ 0.5 for laminar.
            Self::Lees => 0.5,
            // Spalding-Chi: λ ≈ 0.4 for turbulent.
            Self::SpaldingChi => 0.4,
        };
        let reduction = 1.0 - lambda * blowing_parameter;
        q_no_blowing_w_m2 * reduction.max(0.0)
    }
}

/// Trait for ablation models.
pub trait AblationModel {
    /// Recession rate at this station.
    ///
    /// # Errors
    ///
    /// Out-of-envelope on bad inputs.
    fn recession_rate(
        &self,
        ctx: &AerothermalContext,
        station: BodyStation,
    ) -> Result<RecessionRate, AerothermalError>;

    /// Convective heat flux corrected for blowing.
    ///
    /// # Errors
    ///
    /// Out-of-envelope when the blowing parameter is malformed.
    fn blowing_correction(
        &self,
        q_no_blowing_w_m2: f64,
        m_dot_per_area_kg_m2_s: f64,
        edge_density_kg_m3: f64,
        edge_velocity_m_s: f64,
        stanton_no_blowing: f64,
    ) -> Result<f64, AerothermalError>;

    /// Surface state at this station.
    fn surface_state(&self, station: BodyStation) -> SurfaceState;
}

/// Quasi-steady-state ablator.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SteadyStateAblator {
    /// Ablator material.
    pub material: ToyAblator,
    /// Blowing correlation.
    pub blowing: BlowingCorrelation,
}

impl AblationModel for SteadyStateAblator {
    fn recession_rate(
        &self,
        ctx: &AerothermalContext,
        _station: BodyStation,
    ) -> Result<RecessionRate, AerothermalError> {
        // Surface energy balance (steady-state, no conduction):
        //   q_conv + q_rad - σ ε T_w⁴ - m_dot · h_v = 0
        //   ⇒ m_dot = (q_conv + q_rad - σ ε T_w⁴) / h_v
        // For Phase-6.11 baseline, q_rad is taken as 0; the caller
        // supplies the full q_conv via the [`AerothermalContext`].
        const SIGMA_SB: f64 = 5.670_374_419e-8;
        let t_w = ctx.wall_temperature_k;
        if t_w < self.material.vaporisation_temperature_k {
            // No ablation below vaporisation temperature.
            return Ok(RecessionRate { m_per_s: 0.0 });
        }
        // Use Sutton-Graves estimate for q_conv as the steady-state
        // driver when the caller hasn't supplied a richer field; the
        // module is library code so we use the context inputs verbatim.
        // The ablator's recession depends on the *net* surface flux
        // after re-radiation, which the caller computes from the
        // upstream heat-transfer model. Here we approximate
        // q_conv ≈ Sutton-Graves(ctx) for the standalone trait surface.
        let rho = ctx.freestream.density_kg_m3;
        let v = ctx.airspeed_m_s;
        let r_n = ctx.nose_radius_m;
        let q_conv = crate::stagnation::SUTTON_GRAVES_K_EARTH_SI
            * (rho / r_n).sqrt()
            * v.powi(3);
        let q_rerad = SIGMA_SB * self.material.surface_emissivity * t_w.powi(4);
        let net = (q_conv - q_rerad).max(0.0);
        let m_dot_kg_m2_s = net / self.material.heat_of_ablation_j_kg;
        let recession_m_s = m_dot_kg_m2_s / self.material.density_kg_m3;
        Ok(RecessionRate {
            m_per_s: recession_m_s,
        })
    }

    fn blowing_correction(
        &self,
        q_no_blowing_w_m2: f64,
        m_dot_per_area_kg_m2_s: f64,
        edge_density_kg_m3: f64,
        edge_velocity_m_s: f64,
        stanton_no_blowing: f64,
    ) -> Result<f64, AerothermalError> {
        if stanton_no_blowing <= 0.0
            || edge_density_kg_m3 <= 0.0
            || edge_velocity_m_s <= 0.0
        {
            return Err(AerothermalError::InvalidParameter {
                reason: "blowing-correction edge state requires positive ρ_e, V_e, St",
            });
        }
        let b =
            m_dot_per_area_kg_m2_s / (edge_density_kg_m3 * edge_velocity_m_s * stanton_no_blowing);
        Ok(self.blowing.apply(q_no_blowing_w_m2, b))
    }

    fn surface_state(&self, _station: BodyStation) -> SurfaceState {
        SurfaceState::SteadyAblating { recession_rate_m_s: 0.0 }
    }
}

/// Charring ablator with pyrolysis zone. Phase-6.11 baseline ships
/// only the trait wiring and a "constant-progress" pyrolysis tracker
/// — full virgin / char depth-resolved evolution lands in the
/// follow-on slice.
#[derive(Clone, Debug, PartialEq)]
pub struct CharringAblator {
    /// Virgin (unreacted) material.
    pub virgin: ToyAblator,
    /// Char (post-pyrolysis) material.
    pub char_material: ToyAblator,
    /// Pyrolysis temperature range (K).
    pub pyrolysis_temp_range_k: [f64; 2],
    /// Pyrolysis enthalpy (J/kg).
    pub pyrolysis_enthalpy_j_kg: f64,
    /// Pyrolysis-gas injection factor (dimensionless).
    pub gas_injection_factor: f64,
    /// Pyrolysis progress per station (length tracks consumer).
    pub progress: f64,
}

impl AblationModel for CharringAblator {
    fn recession_rate(
        &self,
        ctx: &AerothermalContext,
        _station: BodyStation,
    ) -> Result<RecessionRate, AerothermalError> {
        // For the baseline trait impl, behave like a steady-state
        // ablator using the char-material vaporisation enthalpy plus
        // a pyrolysis-enthalpy contribution proportional to current
        // progress. The full depth-resolved pyrolysis-front model
        // lands in the follow-on slice.
        let proxy = SteadyStateAblator {
            material: ToyAblator {
                heat_of_ablation_j_kg: self.char_material.heat_of_ablation_j_kg
                    + (1.0 - self.progress) * self.pyrolysis_enthalpy_j_kg,
                ..self.char_material
            },
            blowing: BlowingCorrelation::Lees,
        };
        proxy.recession_rate(ctx, BodyStation::stagnation())
    }

    fn blowing_correction(
        &self,
        q_no_blowing_w_m2: f64,
        m_dot_per_area_kg_m2_s: f64,
        edge_density_kg_m3: f64,
        edge_velocity_m_s: f64,
        stanton_no_blowing: f64,
    ) -> Result<f64, AerothermalError> {
        // Charring ablators eject pyrolysis gas → scale the effective
        // m_dot by the gas-injection factor.
        let effective_mdot = m_dot_per_area_kg_m2_s * self.gas_injection_factor;
        let proxy = SteadyStateAblator {
            material: self.virgin,
            blowing: BlowingCorrelation::Lees,
        };
        proxy.blowing_correction(
            q_no_blowing_w_m2,
            effective_mdot,
            edge_density_kg_m3,
            edge_velocity_m_s,
            stanton_no_blowing,
        )
    }

    fn surface_state(&self, _station: BodyStation) -> SurfaceState {
        if self.progress <= 0.0 {
            SurfaceState::Virgin
        } else if self.progress >= 1.0 {
            SurfaceState::Char
        } else {
            SurfaceState::Pyrolyzing {
                progress: self.progress,
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp, clippy::missing_panics_doc, clippy::similar_names)]
mod tests {
    use super::*;
    use crate::stagnation::WallCatalysis;
    use openbmp_physics::AtmosphereSample;

    fn ctx(rho: f64, v: f64, wall_t: f64) -> AerothermalContext {
        let t = 226.5;
        let p = rho * 287.05 * t;
        AerothermalContext {
            freestream: AtmosphereSample::new(rho, p, t, (1.4 * 287.05 * t).sqrt()).unwrap(),
            airspeed_m_s: v,
            mach: v / (1.4 * 287.05 * t).sqrt(),
            nose_radius_m: 1.0,
            wall_temperature_k: wall_t,
            wall_catalysis: WallCatalysis::FullyCatalytic,
        }
    }

    #[test]
    fn graphite_below_vap_temp_no_recession() {
        let m = SteadyStateAblator {
            material: ToyAblator::graphite_toy(),
            blowing: BlowingCorrelation::Lees,
        };
        let r = m
            .recession_rate(&ctx(1.0e-4, 5000.0, 1500.0), BodyStation::stagnation())
            .unwrap();
        assert_eq!(r.m_per_s, 0.0);
    }

    #[test]
    fn graphite_above_vap_temp_positive_recession() {
        // High dynamic-pressure point where Sutton-Graves convective
        // heating outruns re-radiation at the graphite-toy
        // vaporisation temperature (4000 K).
        let m = SteadyStateAblator {
            material: ToyAblator::graphite_toy(),
            blowing: BlowingCorrelation::Lees,
        };
        let high_q_ctx = AerothermalContext {
            freestream: AtmosphereSample::new(
                5.0e-3,
                5.0e-3 * 287.05 * 226.5,
                226.5,
                (1.4_f64 * 287.05 * 226.5).sqrt(),
            )
            .unwrap(),
            airspeed_m_s: 11_000.0,
            mach: 11_000.0 / (1.4_f64 * 287.05 * 226.5).sqrt(),
            nose_radius_m: 0.5,
            wall_temperature_k: 4000.0,
            wall_catalysis: WallCatalysis::FullyCatalytic,
        };
        let r = m
            .recession_rate(&high_q_ctx, BodyStation::stagnation())
            .unwrap();
        assert!(
            r.m_per_s > 0.0,
            "graphite at 4000K with peak-heating density should recess; got {} m/s",
            r.m_per_s
        );
    }

    #[test]
    fn blowing_correction_zero_at_b_zero() {
        let m = SteadyStateAblator {
            material: ToyAblator::graphite_toy(),
            blowing: BlowingCorrelation::FixedLambda { lambda: 0.4 },
        };
        let q = m
            .blowing_correction(1.0e6, 0.0, 1.0e-4, 5000.0, 1.0e-3)
            .unwrap();
        assert!((q - 1.0e6).abs() < 1.0); // B = 0 → q_blow = q_no_blow
    }

    #[test]
    fn blowing_correction_decreases_with_mdot() {
        let m = SteadyStateAblator {
            material: ToyAblator::graphite_toy(),
            blowing: BlowingCorrelation::FixedLambda { lambda: 0.4 },
        };
        let q_zero = m.blowing_correction(1.0e6, 0.0, 1.0e-4, 5000.0, 1.0e-3).unwrap();
        let q_mid = m
            .blowing_correction(1.0e6, 1.0e-3, 1.0e-4, 5000.0, 1.0e-3)
            .unwrap();
        assert!(q_mid < q_zero);
    }

    #[test]
    fn charring_progress_state_transitions() {
        let mut a = CharringAblator {
            virgin: ToyAblator::generic_charring_1(),
            char_material: ToyAblator::graphite_toy(),
            pyrolysis_temp_range_k: [800.0, 1400.0],
            pyrolysis_enthalpy_j_kg: 3.0e6,
            gas_injection_factor: 0.8,
            progress: 0.0,
        };
        assert_eq!(a.surface_state(BodyStation::stagnation()), SurfaceState::Virgin);
        a.progress = 0.5;
        assert!(matches!(
            a.surface_state(BodyStation::stagnation()),
            SurfaceState::Pyrolyzing { progress } if (progress - 0.5).abs() < 1e-9
        ));
        a.progress = 1.0;
        assert_eq!(a.surface_state(BodyStation::stagnation()), SurfaceState::Char);
    }

    #[test]
    fn determinism_two_runs_byte_identical() {
        let m = SteadyStateAblator {
            material: ToyAblator::graphite_toy(),
            blowing: BlowingCorrelation::Lees,
        };
        let a = m
            .recession_rate(&ctx(1.0e-4, 11_000.0, 4500.0), BodyStation::stagnation())
            .unwrap();
        let b = m
            .recession_rate(&ctx(1.0e-4, 11_000.0, 4500.0), BodyStation::stagnation())
            .unwrap();
        assert_eq!(a.m_per_s.to_bits(), b.m_per_s.to_bits());
    }
}
