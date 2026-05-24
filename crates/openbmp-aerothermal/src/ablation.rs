//! Phase-6.11 generic ablation toy.
//!
//! Trait-level surface for ablation models with two implementations:
//!
//! * [`SteadyStateAblator`] — quasi-steady-state surface energy
//!   balance. Useful for back-of-the-envelope studies and early-stage
//!   trajectory analysis.
//! * [`CharringAblator`] — charring ablator with pyrolysis zone.
//!   Tracks virgin / char composition.
//! * [`DepthResolvedCharringAblator`] — deterministic 1-D
//!   energy-limited pyrolysis-front toy for generic charring studies.
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

/// Depth-resolved pyrolysis-front update.
#[derive(Clone, Debug, PartialEq)]
pub struct PyrolysisFrontUpdate {
    /// New front depth from the original surface (m).
    pub front_depth_m: f64,
    /// Front velocity over the update (m/s).
    pub front_velocity_m_s: f64,
    /// Pyrolysis-gas mass flux (kg/(m²·s)).
    pub gas_mdot_kg_m2_s: f64,
    /// Per-depth-node char progress: 0 = virgin, 1 = fully charred.
    pub progress_by_node: Vec<f64>,
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
    /// Recession rate at this station for caller-supplied heat fluxes.
    ///
    /// `q_conv_w_m2` should come from the selected aerothermal model
    /// (Sutton-Graves, Fay-Riddell, CFD deck, etc.). The ablator does
    /// not choose a heating correlation internally; that keeps material
    /// response composable with external references and higher-fidelity
    /// stagnation models.
    ///
    /// # Errors
    ///
    /// Out-of-envelope on bad inputs.
    fn recession_rate(
        &self,
        ctx: &AerothermalContext,
        station: BodyStation,
        q_conv_w_m2: f64,
        q_rad_w_m2: f64,
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
        q_conv_w_m2: f64,
        q_rad_w_m2: f64,
    ) -> Result<RecessionRate, AerothermalError> {
        // Surface energy balance (steady-state, no conduction):
        //   q_conv + q_rad - σ ε T_w⁴ - m_dot · h_v = 0
        //   ⇒ m_dot = (q_conv + q_rad - σ ε T_w⁴) / h_v
        const SIGMA_SB: f64 = 5.670_374_419e-8;
        let t_w = ctx.wall_temperature_k;
        if !q_conv_w_m2.is_finite() || !q_rad_w_m2.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "ablation heat flux input is NaN or Inf",
            });
        }
        if q_conv_w_m2 < 0.0 || q_rad_w_m2 < 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "ablation heat flux inputs must be non-negative",
            });
        }
        if t_w < self.material.vaporisation_temperature_k {
            // No ablation below vaporisation temperature.
            return Ok(RecessionRate { m_per_s: 0.0 });
        }
        let q_rerad = SIGMA_SB * self.material.surface_emissivity * t_w.powi(4);
        let net = (q_conv_w_m2 + q_rad_w_m2 - q_rerad).max(0.0);
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
        if stanton_no_blowing <= 0.0 || edge_density_kg_m3 <= 0.0 || edge_velocity_m_s <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "blowing-correction edge state requires positive ρ_e, V_e, St",
            });
        }
        let b =
            m_dot_per_area_kg_m2_s / (edge_density_kg_m3 * edge_velocity_m_s * stanton_no_blowing);
        Ok(self.blowing.apply(q_no_blowing_w_m2, b))
    }

    fn surface_state(&self, _station: BodyStation) -> SurfaceState {
        SurfaceState::SteadyAblating {
            recession_rate_m_s: 0.0,
        }
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

/// Depth-resolved generic charring ablator.
///
/// This is an energy-limited 1-D academic toy: after re-radiation,
/// the remaining heat flux advances a sharp pyrolysis front at
/// `v_f = q_net / (ρ_virgin h_pyrolysis)`. The state is projected
/// onto fixed depth nodes so telemetry can expose
/// `pyrolysis_progress[i]` and `pyrolysis_gas_mdot[i]` without
/// shipping any fielded TPS material parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct DepthResolvedCharringAblator {
    /// Virgin (unreacted) material.
    pub virgin: ToyAblator,
    /// Char (post-pyrolysis) material.
    pub char_material: ToyAblator,
    /// Total modeled slab thickness (m).
    pub thickness_m: f64,
    /// Fixed node depths measured from the original surface (m).
    pub node_depths_m: Vec<f64>,
    /// Current sharp-front depth from the original surface (m).
    pub front_depth_m: f64,
    /// Pyrolysis enthalpy (J/kg).
    pub pyrolysis_enthalpy_j_kg: f64,
    /// Fraction of pyrolyzed mass emitted as gas.
    pub gas_yield_fraction: f64,
}

impl DepthResolvedCharringAblator {
    /// Construct a uniformly-spaced depth-resolved charring toy.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError::InvalidParameter`] for invalid
    /// material, thickness, node count, enthalpy, or gas yield.
    pub fn new_uniform(
        virgin: ToyAblator,
        char_material: ToyAblator,
        thickness_m: f64,
        n_depth_nodes: usize,
        pyrolysis_enthalpy_j_kg: f64,
        gas_yield_fraction: f64,
    ) -> Result<Self, AerothermalError> {
        if !(thickness_m.is_finite() && thickness_m > 0.0) {
            return Err(AerothermalError::InvalidParameter {
                reason: "depth-resolved charring thickness must be positive",
            });
        }
        if n_depth_nodes < 2 {
            return Err(AerothermalError::InvalidParameter {
                reason: "depth-resolved charring needs at least two nodes",
            });
        }
        let n_depth_nodes_u32 =
            u32::try_from(n_depth_nodes).map_err(|_| AerothermalError::InvalidParameter {
                reason: "depth-resolved charring node count exceeds u32::MAX",
            })?;
        if !(pyrolysis_enthalpy_j_kg.is_finite() && pyrolysis_enthalpy_j_kg > 0.0) {
            return Err(AerothermalError::InvalidParameter {
                reason: "pyrolysis enthalpy must be positive",
            });
        }
        if !(gas_yield_fraction.is_finite() && (0.0..=1.0).contains(&gas_yield_fraction)) {
            return Err(AerothermalError::InvalidParameter {
                reason: "gas yield fraction must be finite and in [0, 1]",
            });
        }
        validate_toy_material(virgin)?;
        validate_toy_material(char_material)?;

        let denom = f64::from(n_depth_nodes_u32 - 1);
        let mut node_depths_m = Vec::with_capacity(n_depth_nodes);
        for i in 0..n_depth_nodes_u32 {
            node_depths_m.push(thickness_m * f64::from(i) / denom);
        }
        Ok(Self {
            virgin,
            char_material,
            thickness_m,
            node_depths_m,
            front_depth_m: 0.0,
            pyrolysis_enthalpy_j_kg,
            gas_yield_fraction,
        })
    }

    /// Current per-node char progress.
    #[must_use]
    pub fn progress_by_node(&self) -> Vec<f64> {
        self.node_depths_m
            .iter()
            .map(|&depth| {
                if depth <= self.front_depth_m {
                    1.0
                } else {
                    0.0
                }
            })
            .collect()
    }

    /// Advance the pyrolysis front for a supplied net heat flux.
    ///
    /// `q_net_w_m2` is the heat flux available after surface
    /// re-radiation and recession losses. Negative values fail
    /// closed; zero leaves the state unchanged.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError`] on malformed flux or time step.
    pub fn advance_front(
        &mut self,
        q_net_w_m2: f64,
        dt_s: f64,
    ) -> Result<PyrolysisFrontUpdate, AerothermalError> {
        if !(q_net_w_m2.is_finite() && dt_s.is_finite()) {
            return Err(AerothermalError::NonFinite {
                reason: "pyrolysis-front heat flux or dt is NaN or Inf",
            });
        }
        if q_net_w_m2 < 0.0 || dt_s < 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "pyrolysis-front heat flux and dt must be non-negative",
            });
        }
        let front_velocity_m_s = if q_net_w_m2 == 0.0 {
            0.0
        } else {
            q_net_w_m2 / (self.virgin.density_kg_m3 * self.pyrolysis_enthalpy_j_kg)
        };
        let old_front = self.front_depth_m;
        let new_front = (old_front + front_velocity_m_s * dt_s).min(self.thickness_m);
        self.front_depth_m = new_front;
        let realised_velocity = if dt_s > 0.0 {
            (new_front - old_front) / dt_s
        } else {
            0.0
        };
        let gas_mdot_kg_m2_s =
            realised_velocity * self.virgin.density_kg_m3 * self.gas_yield_fraction;
        Ok(PyrolysisFrontUpdate {
            front_depth_m: new_front,
            front_velocity_m_s: realised_velocity,
            gas_mdot_kg_m2_s,
            progress_by_node: self.progress_by_node(),
        })
    }
}

impl AblationModel for CharringAblator {
    fn recession_rate(
        &self,
        ctx: &AerothermalContext,
        _station: BodyStation,
        q_conv_w_m2: f64,
        q_rad_w_m2: f64,
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
        proxy.recession_rate(ctx, BodyStation::stagnation(), q_conv_w_m2, q_rad_w_m2)
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

fn validate_toy_material(material: ToyAblator) -> Result<(), AerothermalError> {
    if !(material.density_kg_m3.is_finite() && material.density_kg_m3 > 0.0) {
        return Err(AerothermalError::InvalidParameter {
            reason: "toy ablator density must be positive",
        });
    }
    if !(material.heat_of_ablation_j_kg.is_finite() && material.heat_of_ablation_j_kg > 0.0) {
        return Err(AerothermalError::InvalidParameter {
            reason: "toy ablator heat of ablation must be positive",
        });
    }
    if !(material.surface_emissivity.is_finite()
        && (0.0..=1.0).contains(&material.surface_emissivity))
    {
        return Err(AerothermalError::InvalidParameter {
            reason: "toy ablator emissivity must be finite and in [0, 1]",
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
            .recession_rate(
                &ctx(1.0e-4, 5000.0, 1500.0),
                BodyStation::stagnation(),
                1.0e7,
                0.0,
            )
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
            .recession_rate(&high_q_ctx, BodyStation::stagnation(), 3.0e7, 0.0)
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
        let q_zero = m
            .blowing_correction(1.0e6, 0.0, 1.0e-4, 5000.0, 1.0e-3)
            .unwrap();
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
        assert_eq!(
            a.surface_state(BodyStation::stagnation()),
            SurfaceState::Virgin
        );
        a.progress = 0.5;
        assert!(matches!(
            a.surface_state(BodyStation::stagnation()),
            SurfaceState::Pyrolyzing { progress } if (progress - 0.5).abs() < 1e-9
        ));
        a.progress = 1.0;
        assert_eq!(
            a.surface_state(BodyStation::stagnation()),
            SurfaceState::Char
        );
    }

    #[test]
    fn depth_resolved_pyrolysis_front_advances_by_energy_balance() {
        let mut a = DepthResolvedCharringAblator::new_uniform(
            ToyAblator::generic_charring_1(),
            ToyAblator::graphite_toy(),
            0.10,
            6,
            2.0e6,
            0.25,
        )
        .unwrap();
        let update = a.advance_front(2.9e6, 10.0).unwrap();
        let expected_v = 2.9e6 / (1450.0 * 2.0e6);
        assert!((update.front_velocity_m_s - expected_v).abs() < 1.0e-15);
        assert!((update.front_depth_m - expected_v * 10.0).abs() < 1.0e-15);
        assert!(update.progress_by_node[0] == 1.0);
        assert!(update.progress_by_node[1] == 0.0);
        assert!((update.gas_mdot_kg_m2_s - expected_v * 1450.0 * 0.25).abs() < 1.0e-12);
    }

    #[test]
    fn depth_resolved_pyrolysis_front_saturates_at_back_face() {
        let mut a = DepthResolvedCharringAblator::new_uniform(
            ToyAblator::generic_charring_1(),
            ToyAblator::graphite_toy(),
            0.01,
            3,
            1.0e6,
            1.0,
        )
        .unwrap();
        let update = a.advance_front(1.45e9, 10.0).unwrap();
        assert!((update.front_depth_m - 0.01).abs() < 1.0e-15);
        assert!(
            update
                .progress_by_node
                .iter()
                .all(|&progress| progress == 1.0)
        );
    }

    #[test]
    fn determinism_two_runs_byte_identical() {
        let m = SteadyStateAblator {
            material: ToyAblator::graphite_toy(),
            blowing: BlowingCorrelation::Lees,
        };
        let a = m
            .recession_rate(
                &ctx(1.0e-4, 11_000.0, 4500.0),
                BodyStation::stagnation(),
                5.0e7,
                0.0,
            )
            .unwrap();
        let b = m
            .recession_rate(
                &ctx(1.0e-4, 11_000.0, 4500.0),
                BodyStation::stagnation(),
                5.0e7,
                0.0,
            )
            .unwrap();
        assert_eq!(a.m_per_s.to_bits(), b.m_per_s.to_bits());
    }
}
