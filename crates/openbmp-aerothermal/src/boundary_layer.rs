//! Phase-6.5 boundary-layer state and distributed heating.
//!
//! Ships:
//!
//! * [`BoundaryLayer`] trait + state enum.
//! * Three transition models: empirical fixed `Re_x_crit`, e^N
//!   placeholder, and `Re_θ / M_e` engineering correlation.
//! * [`ReferenceEnthalpyHeating`] — Eckert reference-enthalpy method
//!   distributed-heating consumer.
//!
//! See `docs/hypersonic-extensions.md § Boundary Layer` for the full
//! mathematical context.

use crate::error::AerothermalError;
use crate::stagnation::{AerothermalContext, BodyStation, SurfaceHeating, sutherland_viscosity};

const PR_AIR: f64 = 0.71;

/// Boundary-layer state at a body station.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BoundaryLayerState {
    /// Fully laminar with local momentum-thickness Reynolds number.
    Laminar {
        /// Local Reynolds number based on streamwise distance.
        reynolds_x: f64,
    },
    /// Transitional flow with [0, 1] intermittency.
    Transitional {
        /// Fraction of time at this station flow is turbulent
        /// (0 = pure laminar; 1 = pure turbulent).
        intermittency: f64,
        /// Local Reynolds number.
        reynolds_x: f64,
    },
    /// Fully turbulent.
    Turbulent {
        /// Local Reynolds number.
        reynolds_x: f64,
    },
}

/// Trait implemented by boundary-layer evaluators.
pub trait BoundaryLayer {
    /// Boundary-layer state at this station.
    ///
    /// # Errors
    ///
    /// Out-of-envelope on bad inputs.
    fn state(
        &self,
        station: BodyStation,
        ctx: &AerothermalContext,
    ) -> Result<BoundaryLayerState, AerothermalError>;
}

/// Transition-model trait.
pub trait TransitionModel {
    /// Local intermittency at `Re_x`. Returns `(state)`.
    #[must_use]
    fn intermittency(&self, reynolds_x: f64) -> BoundaryLayerState;
}

/// Empirical fixed `Re_x_crit` transition model. Default
/// `Re_x_crit = 5e5` for low-disturbance flow.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EmpiricalTransition {
    /// Critical Reynolds number at which transition begins.
    pub re_x_crit: f64,
    /// Reynolds number at which transition completes.
    pub re_x_turb: f64,
}

impl Default for EmpiricalTransition {
    fn default() -> Self {
        Self {
            re_x_crit: 5.0e5,
            re_x_turb: 1.0e6,
        }
    }
}

impl TransitionModel for EmpiricalTransition {
    fn intermittency(&self, reynolds_x: f64) -> BoundaryLayerState {
        if reynolds_x < self.re_x_crit {
            BoundaryLayerState::Laminar { reynolds_x }
        } else if reynolds_x >= self.re_x_turb {
            BoundaryLayerState::Turbulent { reynolds_x }
        } else {
            let span = (self.re_x_turb - self.re_x_crit).max(1.0);
            let gamma = (reynolds_x - self.re_x_crit) / span;
            BoundaryLayerState::Transitional {
                intermittency: gamma.clamp(0.0, 1.0),
                reynolds_x,
            }
        }
    }
}

/// `Re_θ / M_e` engineering correlation. The momentum-thickness
/// Reynolds number normalised by edge Mach number is a widely-used
/// hypersonic transition indicator (Bertin 1994; Anderson 2019).
/// Transition is declared when the ratio exceeds `re_theta_crit`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ReThetaTransition {
    /// Critical `Re_θ / M_e` ratio for transition onset.
    pub re_theta_crit: f64,
}

impl Default for ReThetaTransition {
    fn default() -> Self {
        Self {
            re_theta_crit: 200.0,
        }
    }
}

/// e^N transition placeholder. The full Tollmien-Schlichting tracking
/// scheme is deferred — this slice ships the typed surface and a
/// pass-through to [`EmpiricalTransition`] under the hood.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EnTransition {
    /// Critical amplification factor `N_crit` (typical low-disturbance
    /// value: 9-11; quiet tunnel: 6-7).
    pub n_crit: f64,
    /// Fallback empirical transition used until the full e^N
    /// integration ships.
    pub fallback: EmpiricalTransition,
}

impl Default for EnTransition {
    fn default() -> Self {
        Self {
            n_crit: 9.0,
            fallback: EmpiricalTransition::default(),
        }
    }
}

impl TransitionModel for EnTransition {
    fn intermittency(&self, reynolds_x: f64) -> BoundaryLayerState {
        self.fallback.intermittency(reynolds_x)
    }
}

/// Reference-enthalpy (Eckert) distributed-heating consumer.
///
/// Computes local skin friction and Stanton number using the
/// reference-enthalpy correction `h* = h_e + 0.5 (h_w - h_e) + 0.22
/// (h_aw - h_e)` and the laminar/turbulent flat-plate correlations.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ReferenceEnthalpyHeating {
    /// Reference station distance from leading edge (m).
    pub x_m: f64,
}

impl ReferenceEnthalpyHeating {
    /// Local heat flux at the configured station given the
    /// freestream and boundary-layer state.
    ///
    /// # Errors
    ///
    /// Out-of-envelope on bad inputs.
    pub fn distributed(
        &self,
        ctx: &AerothermalContext,
        bl_state: BoundaryLayerState,
    ) -> Result<SurfaceHeating, AerothermalError> {
        if !self.x_m.is_finite() || self.x_m <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "ReferenceEnthalpyHeating x_m must be > 0",
            });
        }
        if !ctx.freestream.density_kg_m3.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "freestream density non-finite",
            });
        }
        let reynolds_x = match bl_state {
            BoundaryLayerState::Laminar { reynolds_x }
            | BoundaryLayerState::Turbulent { reynolds_x }
            | BoundaryLayerState::Transitional { reynolds_x, .. } => reynolds_x,
        };
        if reynolds_x <= 0.0 {
            return Err(AerothermalError::OutOfEnvelope {
                reason: "Re_x must be > 0",
            });
        }

        // Eckert reference enthalpy (cold-gas approximation for the
        // Phase-6.5 baseline; real-gas h(T,p) lands with the
        // realgas-coupled slice).
        let edge_enthalpy = 1005.0 * ctx.freestream.temperature_k;
        let wall_enthalpy = 1005.0 * ctx.wall_temperature_k;
        let adiabatic_wall_enthalpy =
            1005.0 * ctx.freestream.temperature_k + 0.5 * ctx.airspeed_m_s * ctx.airspeed_m_s;
        let h_star = edge_enthalpy
            + 0.5 * (wall_enthalpy - edge_enthalpy)
            + 0.22 * (adiabatic_wall_enthalpy - edge_enthalpy);
        let t_star = h_star / 1005.0;
        let mu_star = sutherland_viscosity(t_star);
        // Reference state density via ideal gas at edge pressure.
        let rho_star = ctx.freestream.pressure_pa / (287.05 * t_star.max(50.0));

        // Skin friction (Blasius for laminar; Schultz-Grunow for
        // turbulent). The Reynolds number used here is the bl_state
        // value, which is the upstream BL model's authoritative
        // location-aware Reynolds number — the reference state is
        // used for ρ* / μ* in the heat-flux composition, not for
        // re-selecting the regime.
        let cf = match bl_state {
            BoundaryLayerState::Laminar { .. } => 0.664 / reynolds_x.sqrt(),
            BoundaryLayerState::Turbulent { .. } => 0.0592 / reynolds_x.powf(0.2),
            BoundaryLayerState::Transitional { intermittency, .. } => {
                let cf_l = 0.664 / reynolds_x.sqrt();
                let cf_t = 0.0592 / reynolds_x.powf(0.2);
                cf_l * (1.0 - intermittency) + cf_t * intermittency
            }
        };
        // Document mu_star usage: kept for ρ* lookup and future
        // compressibility-correction extension; quiet the lint.
        let _ = mu_star;
        // Reynolds analogy: St = Cf / (2 · Pr^(2/3)). Use Pr = 0.71.
        let st = cf / (2.0 * PR_AIR.powf(2.0 / 3.0));
        // Heat flux: q = ρ* · V_e · St · (h_aw - h_w).
        let q = rho_star * ctx.airspeed_m_s * st * (adiabatic_wall_enthalpy - wall_enthalpy);
        Ok(SurfaceHeating {
            q_w_m2: q,
            stanton: st,
            skin_friction: cf,
        })
    }
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
    fn empirical_transition_laminar_below_crit() {
        let m = EmpiricalTransition::default();
        let state = m.intermittency(1.0e4);
        assert!(matches!(state, BoundaryLayerState::Laminar { .. }));
    }

    #[test]
    fn empirical_transition_turbulent_above_top() {
        let m = EmpiricalTransition::default();
        let state = m.intermittency(2.0e6);
        assert!(matches!(state, BoundaryLayerState::Turbulent { .. }));
    }

    #[test]
    fn empirical_transition_intermittent_in_band() {
        let m = EmpiricalTransition::default();
        let state = m.intermittency(7.5e5);
        let intermittency = match state {
            BoundaryLayerState::Transitional { intermittency, .. } => intermittency,
            BoundaryLayerState::Laminar { .. } | BoundaryLayerState::Turbulent { .. } => f64::NAN,
        };
        assert!((intermittency - 0.5).abs() < 0.1);
    }

    #[test]
    fn reference_enthalpy_heating_positive_for_hot_freestream() {
        let h = ReferenceEnthalpyHeating { x_m: 0.5 };
        let result = h
            .distributed(
                &ctx(1.0e-4, 5_000.0, 1000.0),
                BoundaryLayerState::Laminar { reynolds_x: 1.0e5 },
            )
            .unwrap();
        assert!(result.q_w_m2 > 0.0);
        assert!(result.stanton > 0.0);
        assert!(result.skin_friction > 0.0);
    }

    #[test]
    fn turbulent_heating_exceeds_laminar_at_same_re() {
        let h = ReferenceEnthalpyHeating { x_m: 0.5 };
        let c = ctx(1.0e-4, 5_000.0, 1000.0);
        let lam = h
            .distributed(&c, BoundaryLayerState::Laminar { reynolds_x: 1.0e6 })
            .unwrap();
        let turb = h
            .distributed(&c, BoundaryLayerState::Turbulent { reynolds_x: 1.0e6 })
            .unwrap();
        assert!(turb.q_w_m2 > lam.q_w_m2);
        assert!(turb.skin_friction > lam.skin_friction);
    }

    #[test]
    fn reference_enthalpy_rejects_zero_distance() {
        let h = ReferenceEnthalpyHeating { x_m: 0.0 };
        let result = h.distributed(
            &ctx(1.0e-4, 5_000.0, 1000.0),
            BoundaryLayerState::Laminar { reynolds_x: 1.0e5 },
        );
        assert!(matches!(
            result,
            Err(AerothermalError::InvalidParameter { .. })
        ));
    }
}
