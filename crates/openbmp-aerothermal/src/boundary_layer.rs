//! Boundary-layer state and distributed heating.
//!
//! Ships:
//!
//! * [`BoundaryLayer`] trait + state enum.
//! * Transition models: empirical fixed `Re_x_crit`, e^N
//!   placeholder, smooth-wall `Re_θ / M_e`, and PANT/Reda rough-wall
//!   engineering correlations.
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

const RE_THETA_TRANSITION_COMPLETION_RATIO: f64 = 1.25;

/// Default PANT/Reda rough-wall transition indicator threshold.
///
/// The indicator used here is `Re_θ * (T_e / T_w) * (k / θ)`.
pub const PANT_REDA_ROUGHNESS_CRITICAL_INDICATOR: f64 = 215.0;

impl ReThetaTransition {
    /// Construct a smooth-wall `Re_θ / M_e` transition model from an
    /// explicit onset threshold.
    #[must_use]
    pub const fn with_critical_ratio(re_theta_crit: f64) -> Self {
        Self { re_theta_crit }
    }

    /// Compute the smooth-wall transition indicator `Re_θ / M_e`.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError`] if the supplied momentum-thickness
    /// Reynolds number or edge Mach number is non-finite, negative, or
    /// otherwise outside the algebraic correlation envelope.
    pub fn ratio(
        momentum_thickness_reynolds: f64,
        edge_mach: f64,
    ) -> Result<f64, AerothermalError> {
        if !momentum_thickness_reynolds.is_finite() || !edge_mach.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Re_theta transition inputs are NaN or Inf",
            });
        }
        if momentum_thickness_reynolds < 0.0 || edge_mach <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Re_theta transition requires non-negative Re_theta and positive edge Mach",
            });
        }
        Ok(momentum_thickness_reynolds / edge_mach)
    }

    /// Evaluate laminar/transitional/turbulent state from a local
    /// Reynolds number, momentum-thickness Reynolds number, and edge
    /// Mach number.
    ///
    /// The transition onset is `Re_θ / M_e == re_theta_crit`; the
    /// built-in smooth engineering ramp reaches fully turbulent at
    /// `1.25 * re_theta_crit`. Rough-wall PANT/Reda and e^N stability
    /// methods are intentionally separate models.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError`] if model or state inputs are
    /// non-finite, non-positive where required, or outside the
    /// correlation envelope.
    pub fn state_from_re_theta(
        &self,
        reynolds_x: f64,
        momentum_thickness_reynolds: f64,
        edge_mach: f64,
    ) -> Result<BoundaryLayerState, AerothermalError> {
        if !self.re_theta_crit.is_finite() || !reynolds_x.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Re_theta transition model input is NaN or Inf",
            });
        }
        if self.re_theta_crit <= 0.0 || reynolds_x <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "Re_theta transition requires positive critical ratio and local Reynolds number",
            });
        }

        let ratio = Self::ratio(momentum_thickness_reynolds, edge_mach)?;
        if ratio < self.re_theta_crit {
            Ok(BoundaryLayerState::Laminar { reynolds_x })
        } else {
            let complete = self.re_theta_crit * RE_THETA_TRANSITION_COMPLETION_RATIO;
            if ratio >= complete {
                Ok(BoundaryLayerState::Turbulent { reynolds_x })
            } else {
                Ok(BoundaryLayerState::Transitional {
                    intermittency: ((ratio - self.re_theta_crit) / (complete - self.re_theta_crit))
                        .clamp(0.0, 1.0),
                    reynolds_x,
                })
            }
        }
    }
}

/// Combined smooth-wall plus PANT/Reda rough-wall transition selector.
///
/// Smooth-wall onset is still controlled by [`ReThetaTransition`]. If
/// `roughness_height_m > 0`, the rough-wall indicator
/// `Re_θ * (T_e / T_w) * (k / θ)` is evaluated as a second onset path. The
/// larger normalized indicator controls the same deterministic linear ramp
/// from onset to fully turbulent at `1.25` times threshold.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PantRedaRoughnessTransition {
    /// Smooth-wall transition model.
    pub smooth: ReThetaTransition,
    /// Equivalent roughness height `k` in meters.
    pub roughness_height_m: f64,
    /// Critical PANT/Reda roughness indicator for transition onset.
    pub roughness_critical_indicator: f64,
}

impl Default for PantRedaRoughnessTransition {
    fn default() -> Self {
        Self {
            smooth: ReThetaTransition::default(),
            roughness_height_m: 0.0,
            roughness_critical_indicator: PANT_REDA_ROUGHNESS_CRITICAL_INDICATOR,
        }
    }
}

impl PantRedaRoughnessTransition {
    /// Construct a PANT/Reda selector with default thresholds and a supplied
    /// roughness height.
    #[must_use]
    pub const fn with_roughness_height(roughness_height_m: f64) -> Self {
        Self {
            smooth: ReThetaTransition::with_critical_ratio(200.0),
            roughness_height_m,
            roughness_critical_indicator: PANT_REDA_ROUGHNESS_CRITICAL_INDICATOR,
        }
    }

    /// Construct a PANT/Reda selector from explicit smooth-wall and rough-wall
    /// onset thresholds.
    #[must_use]
    pub const fn with_thresholds(
        smooth_re_theta_crit: f64,
        roughness_height_m: f64,
        roughness_critical_indicator: f64,
    ) -> Self {
        Self {
            smooth: ReThetaTransition::with_critical_ratio(smooth_re_theta_crit),
            roughness_height_m,
            roughness_critical_indicator,
        }
    }

    /// Compute the PANT/Reda rough-wall indicator
    /// `Re_θ * (T_e / T_w) * (k / θ)`.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError`] if any input is non-finite, if `Re_θ` or
    /// roughness height is negative, or if momentum thickness / temperatures
    /// are non-positive.
    pub fn roughness_indicator(
        momentum_thickness_reynolds: f64,
        momentum_thickness_m: f64,
        roughness_height_m: f64,
        edge_temperature_k: f64,
        wall_temperature_k: f64,
    ) -> Result<f64, AerothermalError> {
        if !momentum_thickness_reynolds.is_finite()
            || !momentum_thickness_m.is_finite()
            || !roughness_height_m.is_finite()
            || !edge_temperature_k.is_finite()
            || !wall_temperature_k.is_finite()
        {
            return Err(AerothermalError::NonFinite {
                reason: "PANT/Reda transition inputs are NaN or Inf",
            });
        }
        if momentum_thickness_reynolds < 0.0
            || momentum_thickness_m <= 0.0
            || roughness_height_m < 0.0
            || edge_temperature_k <= 0.0
            || wall_temperature_k <= 0.0
        {
            return Err(AerothermalError::InvalidParameter {
                reason: "PANT/Reda transition requires non-negative Re_theta/roughness and positive theta/temperatures",
            });
        }
        Ok(momentum_thickness_reynolds
            * (edge_temperature_k / wall_temperature_k)
            * (roughness_height_m / momentum_thickness_m))
    }

    /// Evaluate laminar/transitional/turbulent state from smooth-wall and
    /// rough-wall transition indicators.
    ///
    /// If roughness height is zero, this path is equivalent to
    /// [`ReThetaTransition::state_from_re_theta`] and does not require valid
    /// momentum-thickness or temperature inputs.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError`] if model thresholds or supplied state
    /// inputs are non-finite, non-positive where required, or outside the
    /// correlation envelope.
    #[allow(clippy::too_many_arguments)]
    pub fn state_from_boundary_layer(
        &self,
        reynolds_x: f64,
        momentum_thickness_reynolds: f64,
        edge_mach: f64,
        momentum_thickness_m: f64,
        edge_temperature_k: f64,
        wall_temperature_k: f64,
    ) -> Result<BoundaryLayerState, AerothermalError> {
        if !self.smooth.re_theta_crit.is_finite()
            || !self.roughness_height_m.is_finite()
            || !self.roughness_critical_indicator.is_finite()
            || !reynolds_x.is_finite()
        {
            return Err(AerothermalError::NonFinite {
                reason: "PANT/Reda transition model input is NaN or Inf",
            });
        }
        if self.smooth.re_theta_crit <= 0.0
            || self.roughness_height_m < 0.0
            || self.roughness_critical_indicator <= 0.0
            || reynolds_x <= 0.0
        {
            return Err(AerothermalError::InvalidParameter {
                reason: "PANT/Reda transition requires positive thresholds/local Reynolds and non-negative roughness",
            });
        }

        let smooth_ratio = ReThetaTransition::ratio(momentum_thickness_reynolds, edge_mach)?
            / self.smooth.re_theta_crit;
        let roughness_ratio = if self.roughness_height_m > 0.0 {
            Self::roughness_indicator(
                momentum_thickness_reynolds,
                momentum_thickness_m,
                self.roughness_height_m,
                edge_temperature_k,
                wall_temperature_k,
            )? / self.roughness_critical_indicator
        } else {
            0.0
        };
        let governing_ratio = smooth_ratio.max(roughness_ratio);
        if governing_ratio < 1.0 {
            Ok(BoundaryLayerState::Laminar { reynolds_x })
        } else if governing_ratio >= RE_THETA_TRANSITION_COMPLETION_RATIO {
            Ok(BoundaryLayerState::Turbulent { reynolds_x })
        } else {
            Ok(BoundaryLayerState::Transitional {
                intermittency: ((governing_ratio - 1.0)
                    / (RE_THETA_TRANSITION_COMPLETION_RATIO - 1.0))
                    .clamp(0.0, 1.0),
                reynolds_x,
            })
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
        validate_heating_context(ctx)?;
        if let BoundaryLayerState::Transitional { intermittency, .. } = bl_state
            && (!intermittency.is_finite() || !(0.0..=1.0).contains(&intermittency))
        {
            return Err(AerothermalError::InvalidParameter {
                reason: "boundary-layer intermittency must be finite and in [0, 1]",
            });
        }
        let reynolds_x = match bl_state {
            BoundaryLayerState::Laminar { reynolds_x }
            | BoundaryLayerState::Turbulent { reynolds_x }
            | BoundaryLayerState::Transitional { reynolds_x, .. } => reynolds_x,
        };
        if !reynolds_x.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "Re_x is NaN or Inf",
            });
        }
        if reynolds_x <= 0.0 {
            return Err(AerothermalError::OutOfEnvelope {
                reason: "Re_x must be > 0",
            });
        }

        // Eckert reference enthalpy (cold-gas approximation for the
        // baseline; real-gas h(T,p) lands with the
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
        if !(q.is_finite() && st.is_finite() && cf.is_finite()) {
            return Err(AerothermalError::NonFinite {
                reason: "reference-enthalpy heating output is NaN or Inf",
            });
        }
        Ok(SurfaceHeating {
            q_w_m2: q,
            stanton: st,
            skin_friction: cf,
        })
    }
}

fn validate_heating_context(ctx: &AerothermalContext) -> Result<(), AerothermalError> {
    if !ctx.freestream.density_kg_m3.is_finite()
        || !ctx.freestream.pressure_pa.is_finite()
        || !ctx.freestream.temperature_k.is_finite()
        || !ctx.airspeed_m_s.is_finite()
        || !ctx.wall_temperature_k.is_finite()
    {
        return Err(AerothermalError::NonFinite {
            reason: "reference-enthalpy context input is NaN or Inf",
        });
    }
    if ctx.freestream.density_kg_m3 < 0.0
        || ctx.freestream.pressure_pa < 0.0
        || ctx.freestream.temperature_k <= 0.0
        || ctx.airspeed_m_s < 0.0
        || ctx.wall_temperature_k <= 0.0
    {
        return Err(AerothermalError::InvalidParameter {
            reason: "reference-enthalpy context inputs outside physical bounds",
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
    fn re_theta_transition_uses_mach_normalized_onset() {
        let model = ReThetaTransition::with_critical_ratio(200.0);
        assert_eq!(ReThetaTransition::ratio(1_000.0, 5.0).unwrap(), 200.0);

        let below = model.state_from_re_theta(1.0e6, 990.0, 5.0).unwrap();
        assert!(matches!(below, BoundaryLayerState::Laminar { .. }));

        let onset = model.state_from_re_theta(1.0e6, 1_000.0, 5.0).unwrap();
        assert!(matches!(
            onset,
            BoundaryLayerState::Transitional {
                intermittency,
                ..
            } if intermittency == 0.0
        ));
    }

    #[test]
    fn re_theta_transition_blends_to_turbulent() {
        let model = ReThetaTransition::with_critical_ratio(200.0);
        let mid = model.state_from_re_theta(1.0e6, 1_125.0, 5.0).unwrap();
        let intermittency = match mid {
            BoundaryLayerState::Transitional { intermittency, .. } => intermittency,
            BoundaryLayerState::Laminar { .. } | BoundaryLayerState::Turbulent { .. } => f64::NAN,
        };
        assert!((intermittency - 0.5).abs() <= f64::EPSILON);

        let complete = model.state_from_re_theta(1.0e6, 1_250.0, 5.0).unwrap();
        assert!(matches!(complete, BoundaryLayerState::Turbulent { .. }));
    }

    #[test]
    fn re_theta_transition_heating_is_continuous_at_ramp_endpoints() {
        let model = ReThetaTransition::with_critical_ratio(200.0);
        let heating = ReferenceEnthalpyHeating { x_m: 0.5 };
        let context = ctx(1.0e-4, 5_000.0, 1000.0);

        let onset = model.state_from_re_theta(1.0e6, 1_000.0, 5.0).unwrap();
        let onset_flux = heating.distributed(&context, onset).unwrap();
        let laminar_flux = heating
            .distributed(&context, BoundaryLayerState::Laminar { reynolds_x: 1.0e6 })
            .unwrap();
        assert!((onset_flux.q_w_m2 - laminar_flux.q_w_m2).abs() <= 1.0e-9);

        let complete = model.state_from_re_theta(1.0e6, 1_250.0, 5.0).unwrap();
        let complete_flux = heating.distributed(&context, complete).unwrap();
        let turbulent_flux = heating
            .distributed(
                &context,
                BoundaryLayerState::Turbulent { reynolds_x: 1.0e6 },
            )
            .unwrap();
        assert!((complete_flux.q_w_m2 - turbulent_flux.q_w_m2).abs() <= 1.0e-9);
    }

    #[test]
    fn re_theta_transition_rejects_invalid_inputs() {
        let model = ReThetaTransition::default();
        assert!(matches!(
            ReThetaTransition::ratio(f64::NAN, 5.0),
            Err(AerothermalError::NonFinite { .. })
        ));
        assert!(matches!(
            ReThetaTransition::ratio(1_000.0, 0.0),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        assert!(matches!(
            model.state_from_re_theta(0.0, 1_000.0, 5.0),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        assert!(matches!(
            ReThetaTransition::with_critical_ratio(-1.0).state_from_re_theta(1.0e6, 1_000.0, 5.0),
            Err(AerothermalError::InvalidParameter { .. })
        ));
    }

    fn fixture_value(fixture: &str, key: &str) -> f64 {
        fixture
            .lines()
            .find_map(|line| {
                let (name, value) = line.split_once('=')?;
                if name.trim() == key {
                    value.trim().parse::<f64>().ok()
                } else {
                    None
                }
            })
            .unwrap_or_else(|| panic!("missing transition fixture value {key}"))
    }

    fn transition_fixture_value(key: &str) -> f64 {
        fixture_value(
            include_str!("../../../data/aerothermal/transition/pant-reda-thresholds-v1.toml"),
            key,
        )
    }

    fn sharp_cone_fixture_value(key: &str) -> f64 {
        fixture_value(
            include_str!("../../../data/aerothermal/transition/sharp-cone-transition-v1.toml"),
            key,
        )
    }

    fn sharp_cone_re_theta(unit_reynolds_per_m: f64, x_m: f64, coefficient: f64) -> f64 {
        coefficient * (unit_reynolds_per_m * x_m).sqrt()
    }

    #[test]
    fn re_theta_transition_matches_sharp_cone_location_fixture() {
        let edge_mach = sharp_cone_fixture_value("edge_mach");
        let unit_reynolds_per_m = sharp_cone_fixture_value("unit_reynolds_per_m");
        let coefficient = sharp_cone_fixture_value("momentum_thickness_coefficient");
        let critical_ratio = sharp_cone_fixture_value("smooth_re_theta_crit");
        let completion_ratio = sharp_cone_fixture_value("completion_ratio");
        let reference_x_m = sharp_cone_fixture_value("reference_transition_x_m");
        let completion_x_m = sharp_cone_fixture_value("reference_completion_x_m");
        let tolerance_x_m = sharp_cone_fixture_value("tolerance_x_m");
        let model = ReThetaTransition::with_critical_ratio(critical_ratio);
        let reconstructed_x_m =
            (critical_ratio * edge_mach / coefficient).powi(2) / unit_reynolds_per_m;
        let reconstructed_completion_x_m =
            (critical_ratio * completion_ratio * edge_mach / coefficient).powi(2)
                / unit_reynolds_per_m;

        assert!((reconstructed_x_m - reference_x_m).abs() <= tolerance_x_m);
        assert!((reconstructed_completion_x_m - completion_x_m).abs() <= tolerance_x_m);

        let below_x_m = reference_x_m - tolerance_x_m;
        let below = model
            .state_from_re_theta(
                unit_reynolds_per_m * below_x_m,
                sharp_cone_re_theta(unit_reynolds_per_m, below_x_m, coefficient),
                edge_mach,
            )
            .unwrap();
        assert!(matches!(below, BoundaryLayerState::Laminar { .. }));

        let onset = model
            .state_from_re_theta(
                unit_reynolds_per_m * reference_x_m,
                sharp_cone_re_theta(unit_reynolds_per_m, reference_x_m, coefficient),
                edge_mach,
            )
            .unwrap();
        assert!(matches!(
            onset,
            BoundaryLayerState::Transitional {
                intermittency,
                ..
            } if intermittency.abs() <= 1.0e-12
        ));

        let complete = model
            .state_from_re_theta(
                unit_reynolds_per_m * completion_x_m,
                sharp_cone_re_theta(unit_reynolds_per_m, completion_x_m, coefficient),
                edge_mach,
            )
            .unwrap();
        assert!(matches!(complete, BoundaryLayerState::Turbulent { .. }));
    }

    #[test]
    fn pant_reda_roughness_indicator_matches_fixture_threshold() {
        let re_theta = transition_fixture_value("synthetic_onset_re_theta");
        let theta_m = transition_fixture_value("synthetic_momentum_thickness_m");
        let roughness_m = transition_fixture_value("synthetic_roughness_height_m");
        let edge_t = transition_fixture_value("synthetic_edge_temperature_k");
        let wall_t = transition_fixture_value("synthetic_wall_temperature_k");

        assert_eq!(
            transition_fixture_value("pant_reda_roughness_critical_indicator"),
            PANT_REDA_ROUGHNESS_CRITICAL_INDICATOR
        );
        assert_eq!(
            PantRedaRoughnessTransition::roughness_indicator(
                re_theta,
                theta_m,
                roughness_m,
                edge_t,
                wall_t
            )
            .unwrap(),
            PANT_REDA_ROUGHNESS_CRITICAL_INDICATOR
        );
    }

    #[test]
    fn pant_reda_roughness_transition_can_trigger_before_smooth_wall() {
        let model = PantRedaRoughnessTransition::with_thresholds(
            200.0,
            0.000_1,
            PANT_REDA_ROUGHNESS_CRITICAL_INDICATOR,
        );

        let below = model
            .state_from_boundary_layer(1.0e6, 990.0, 10.0, 0.001, 2_150.0, 1_000.0)
            .unwrap();
        assert!(matches!(below, BoundaryLayerState::Laminar { .. }));

        let onset = model
            .state_from_boundary_layer(1.0e6, 1_000.0, 10.0, 0.001, 2_150.0, 1_000.0)
            .unwrap();
        assert!(matches!(
            onset,
            BoundaryLayerState::Transitional {
                intermittency,
                ..
            } if intermittency == 0.0
        ));

        let mid = model
            .state_from_boundary_layer(1.0e6, 1_125.0, 10.0, 0.001, 2_150.0, 1_000.0)
            .unwrap();
        let intermittency = match mid {
            BoundaryLayerState::Transitional { intermittency, .. } => intermittency,
            BoundaryLayerState::Laminar { .. } | BoundaryLayerState::Turbulent { .. } => f64::NAN,
        };
        assert!((intermittency - 0.5).abs() <= f64::EPSILON);

        let complete = model
            .state_from_boundary_layer(1.0e6, 1_250.0, 10.0, 0.001, 2_150.0, 1_000.0)
            .unwrap();
        assert!(matches!(complete, BoundaryLayerState::Turbulent { .. }));
    }

    #[test]
    fn pant_reda_zero_roughness_matches_smooth_wall_transition() {
        let model = PantRedaRoughnessTransition::default();
        let smooth = ReThetaTransition::default()
            .state_from_re_theta(1.0e6, 1_125.0, 5.0)
            .unwrap();
        let combined = model
            .state_from_boundary_layer(1.0e6, 1_125.0, 5.0, f64::NAN, 0.0, 0.0)
            .unwrap();
        assert_eq!(combined, smooth);
    }

    #[test]
    fn pant_reda_transition_heating_is_continuous_at_ramp_endpoints() {
        let model = PantRedaRoughnessTransition::with_roughness_height(0.000_1);
        let heating = ReferenceEnthalpyHeating { x_m: 0.5 };
        let context = ctx(1.0e-4, 5_000.0, 1000.0);

        let onset = model
            .state_from_boundary_layer(1.0e6, 1_000.0, 10.0, 0.001, 2_150.0, 1_000.0)
            .unwrap();
        let onset_flux = heating.distributed(&context, onset).unwrap();
        let laminar_flux = heating
            .distributed(&context, BoundaryLayerState::Laminar { reynolds_x: 1.0e6 })
            .unwrap();
        assert!((onset_flux.q_w_m2 - laminar_flux.q_w_m2).abs() <= 1.0e-9);

        let complete = model
            .state_from_boundary_layer(1.0e6, 1_250.0, 10.0, 0.001, 2_150.0, 1_000.0)
            .unwrap();
        let complete_flux = heating.distributed(&context, complete).unwrap();
        let turbulent_flux = heating
            .distributed(
                &context,
                BoundaryLayerState::Turbulent { reynolds_x: 1.0e6 },
            )
            .unwrap();
        assert!((complete_flux.q_w_m2 - turbulent_flux.q_w_m2).abs() <= 1.0e-9);
    }

    #[test]
    fn pant_reda_transition_rejects_invalid_inputs() {
        assert!(matches!(
            PantRedaRoughnessTransition::roughness_indicator(
                f64::NAN,
                0.001,
                0.000_1,
                2_150.0,
                1_000.0
            ),
            Err(AerothermalError::NonFinite { .. })
        ));
        assert!(matches!(
            PantRedaRoughnessTransition::roughness_indicator(
                1_000.0, 0.0, 0.000_1, 2_150.0, 1_000.0
            ),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        assert!(matches!(
            PantRedaRoughnessTransition::with_thresholds(200.0, -1.0, 215.0)
                .state_from_boundary_layer(1.0e6, 1_000.0, 5.0, 0.001, 2_150.0, 1_000.0),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        assert!(matches!(
            PantRedaRoughnessTransition::with_thresholds(200.0, 0.000_1, 0.0)
                .state_from_boundary_layer(1.0e6, 1_000.0, 5.0, 0.001, 2_150.0, 1_000.0),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        assert!(matches!(
            PantRedaRoughnessTransition::with_roughness_height(0.000_1)
                .state_from_boundary_layer(1.0e6, 1_000.0, 5.0, 0.0, 2_150.0, 1_000.0),
            Err(AerothermalError::InvalidParameter { .. })
        ));
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

    #[test]
    fn reference_enthalpy_rejects_malformed_context() {
        let h = ReferenceEnthalpyHeating { x_m: 0.5 };
        let mut c = ctx(1.0e-4, 5_000.0, 1000.0);
        c.freestream.pressure_pa = f64::NAN;
        assert!(matches!(
            h.distributed(&c, BoundaryLayerState::Laminar { reynolds_x: 1.0e5 }),
            Err(AerothermalError::NonFinite { .. })
        ));

        let mut c = ctx(1.0e-4, 5_000.0, 1000.0);
        c.wall_temperature_k = -1.0;
        assert!(matches!(
            h.distributed(&c, BoundaryLayerState::Laminar { reynolds_x: 1.0e5 }),
            Err(AerothermalError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn reference_enthalpy_rejects_invalid_boundary_layer_state() {
        let h = ReferenceEnthalpyHeating { x_m: 0.5 };
        let c = ctx(1.0e-4, 5_000.0, 1000.0);
        assert!(matches!(
            h.distributed(
                &c,
                BoundaryLayerState::Transitional {
                    intermittency: f64::NAN,
                    reynolds_x: 1.0e5
                }
            ),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        assert!(matches!(
            h.distributed(
                &c,
                BoundaryLayerState::Laminar {
                    reynolds_x: f64::NAN
                }
            ),
            Err(AerothermalError::NonFinite { .. })
        ));
    }
}
