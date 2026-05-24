//! Phase-6.6 continuum-to-rarefied bridging.
//!
//! Knudsen number computation, three bridge-function variants
//! (Cheng, complementary-error-function, linear smoothstep),
//! free-molecular aero per Schaaf & Chambré, and a
//! `HybridAeroMethod` that dispatches across the regimes.

use std::f64::consts::PI;

use nalgebra::Vector3;

use crate::error::AeroError;
use crate::method::{AeroContext, AeroForceMomentBody, AeroMethod};

/// Boltzmann constant `k_B` (J/K).
const BOLTZMANN_J_K: f64 = 1.380_649e-23;

/// Effective collision diameter for dry air (m). The standard textbook
/// value for `N₂`-equivalent collisions.
const D_AIR_M: f64 = 3.65e-10;

/// Compute mean free path `λ = k_B · T / (√2 · π · d² · p)` (m).
#[must_use]
pub fn mean_free_path_m(temperature_k: f64, pressure_pa: f64) -> f64 {
    if !temperature_k.is_finite()
        || !pressure_pa.is_finite()
        || temperature_k <= 0.0
        || pressure_pa <= 0.0
    {
        return f64::INFINITY;
    }
    BOLTZMANN_J_K * temperature_k / ((2.0_f64).sqrt() * PI * D_AIR_M * D_AIR_M * pressure_pa)
}

/// Compute Knudsen number `Kn = λ / L_ref` (dimensionless).
#[must_use]
pub fn knudsen_number(mean_free_path_m_val: f64, characteristic_length_m: f64) -> f64 {
    if !mean_free_path_m_val.is_finite()
        || mean_free_path_m_val < 0.0
        || !characteristic_length_m.is_finite()
        || characteristic_length_m <= 0.0
    {
        return f64::INFINITY;
    }
    mean_free_path_m_val / characteristic_length_m
}

/// Gas regime classification by Knudsen number.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GasRegime {
    /// Kn < 0.01 — continuum.
    Continuum,
    /// 0.01 ≤ Kn < 0.1 — slip.
    Slip,
    /// 0.1 ≤ Kn < 10 — transition.
    Transition,
    /// Kn ≥ 10 — free molecular.
    FreeMolecular,
}

impl GasRegime {
    /// Classify by Knudsen number.
    #[must_use]
    pub fn from_knudsen(kn: f64) -> Self {
        if !kn.is_finite() {
            return Self::FreeMolecular;
        }
        if kn < 0.01 {
            Self::Continuum
        } else if kn < 0.1 {
            Self::Slip
        } else if kn < 10.0 {
            Self::Transition
        } else {
            Self::FreeMolecular
        }
    }
}

/// Trait for continuum-to-free-molecular bridge functions.
pub trait BridgeFunction {
    /// Bridge weight `α ∈ [0, 1]`. `α = 0` → continuum; `α = 1` → FM.
    fn alpha(&self, knudsen: f64) -> f64;
}

/// Cheng bridge: `α(Kn) = exp(−π / (2 · Kn))`.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct ChengBridge;

impl BridgeFunction for ChengBridge {
    fn alpha(&self, knudsen: f64) -> f64 {
        if !knudsen.is_finite() {
            return 1.0;
        }
        if knudsen <= 0.0 {
            return 0.0;
        }
        (-PI / (2.0 * knudsen)).exp()
    }
}

/// Complementary-error-function bridge:
/// `α(Kn) = 0.5 · erfc(log10(Kn) / σ)` mapped so the centre is at
/// `Kn = 1`. Pure `f64` arithmetic (uses [`erfc_approx`] for a
/// state-stable closed-form approximation).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ErfcBridge {
    /// Width parameter `σ`. Smaller → sharper transition.
    pub sigma: f64,
}

impl Default for ErfcBridge {
    fn default() -> Self {
        Self { sigma: 1.5 }
    }
}

impl BridgeFunction for ErfcBridge {
    fn alpha(&self, knudsen: f64) -> f64 {
        if !knudsen.is_finite() {
            return 1.0;
        }
        if knudsen <= 0.0 {
            return 0.0;
        }
        let arg = knudsen.log10() / self.sigma.max(1.0e-6);
        // erfc-based bridge: monotone Kn → 0: continuum (α ≈ 0);
        //                              Kn → ∞: FM (α ≈ 1).
        // We use the property erfc(-x) = 2 - erfc(x), so
        //   α = 0.5 * (1 + erf(arg)) = 0.5 * (2 - erfc(arg)) for arg > 0
        // and we want α increasing with Kn (so use the standard form
        // with erfc(-arg)).
        0.5 * erfc_approx(-arg)
    }
}

/// Linear smoothstep bridge over `[Kn_lo, Kn_hi]`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LinearKnudsenBridge {
    /// Lower Knudsen-number end of the transition band.
    pub kn_lo: f64,
    /// Upper Knudsen-number end of the transition band.
    pub kn_hi: f64,
}

impl Default for LinearKnudsenBridge {
    fn default() -> Self {
        Self {
            kn_lo: 0.01,
            kn_hi: 10.0,
        }
    }
}

impl BridgeFunction for LinearKnudsenBridge {
    fn alpha(&self, knudsen: f64) -> f64 {
        if !knudsen.is_finite() {
            return 1.0;
        }
        if knudsen <= self.kn_lo {
            0.0
        } else if knudsen >= self.kn_hi {
            1.0
        } else {
            let span = (self.kn_hi - self.kn_lo).max(1.0e-30);
            (knudsen - self.kn_lo) / span
        }
    }
}

/// Closed-form Abramowitz-Stegun 7.1.26 erfc approximation
/// (max relative error ≈ 1.5e-7). Pure arithmetic on `f64`;
/// no FMA. Used by [`ErfcBridge::alpha`] for state-stability.
#[must_use]
pub fn erfc_approx(x: f64) -> f64 {
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let a1 = 0.254_829_592;
    let a2 = -0.284_496_736;
    let a3 = 1.421_413_741;
    let a4 = -1.453_152_027;
    let a5 = 1.061_405_429;
    let p = 0.327_591_1;
    let xa = x.abs();
    let t = 1.0 / (1.0 + p * xa);
    let y = (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * (-xa * xa).exp();
    if sign > 0.0 { y } else { 2.0 - y }
}

/// Accommodation coefficients for [`FreeMolecularAero`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AccommodationCoeffs {
    /// Tangential momentum accommodation `σ_t ∈ [0, 1]`.
    pub tangential: f64,
    /// Normal momentum accommodation `σ_n ∈ [0, 1]`.
    pub normal: f64,
}

impl Default for AccommodationCoeffs {
    fn default() -> Self {
        Self {
            tangential: 1.0,
            normal: 1.0,
        }
    }
}

/// Free-molecular drag on a flat plate at angle `α` to freestream
/// (Schaaf & Chambré high-speed-ratio limit). `α = 0` is edge-on
/// and `α = π/2` is broadside. Representative-panel approximation
/// per the rest of the Phase-6 aero family.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FreeMolecularAero {
    /// Accommodation coefficients.
    pub accommodation: AccommodationCoeffs,
    /// Reference area (m²).
    pub reference_area_m2: f64,
}

impl FreeMolecularAero {
    /// Schaaf-Chambré high-speed-ratio representative drag coefficient.
    ///
    /// The normal accommodation term gives the broadside pressure
    /// contribution `2 * σ_n * sin²(α)`. The tangential accommodation
    /// term is retained as a shear proxy `2 * σ_t * sin(α) * cos(α)`.
    /// This keeps the correct limits: zero drag edge-on and
    /// `2 * σ_n` for a fully broadside diffuse plate.
    #[must_use]
    pub fn cd(&self, alpha_rad: f64) -> f64 {
        if !alpha_rad.is_finite() {
            return 0.0;
        }
        let s = alpha_rad.sin().abs();
        let c = alpha_rad.cos().abs();
        let normal = 2.0 * self.accommodation.normal * s * s;
        let shear = 2.0 * self.accommodation.tangential * s * c;
        normal + shear
    }
}

impl AeroMethod for FreeMolecularAero {
    fn aero_force_moment_body(&self, ctx: &AeroContext) -> Result<AeroForceMomentBody, AeroError> {
        if !ctx.mach.is_finite()
            || !ctx.alpha_deg.is_finite()
            || !ctx.beta_deg.is_finite()
            || !ctx.dynamic_pressure_pa.is_finite()
        {
            return Err(AeroError::NonFinite {
                reason: "FreeMolecularAero context input is NaN or Inf",
            });
        }
        if ctx.dynamic_pressure_pa < 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "FreeMolecularAero dynamic_pressure_pa must be non-negative",
            });
        }
        if !(0.0..=1.0).contains(&self.accommodation.normal)
            || !(0.0..=1.0).contains(&self.accommodation.tangential)
            || !self.reference_area_m2.is_finite()
            || self.reference_area_m2 < 0.0
        {
            return Err(AeroError::InvalidParameter {
                reason: "FreeMolecularAero requires accommodation in [0, 1] and non-negative area",
            });
        }
        let alpha_rad = ctx.alpha_deg.to_radians();
        let cd = self.cd(alpha_rad);
        // FM drag scales with q and reference area; sign in body -x.
        let drag = cd * ctx.dynamic_pressure_pa * self.reference_area_m2;
        Ok(AeroForceMomentBody {
            force_n_body: Vector3::new(-drag, 0.0, 0.0),
            moment_n_m_body: Vector3::zeros(),
        })
    }
}

/// Hybrid continuum-to-free-molecular dispatcher.
///
/// Blends three aero methods based on Mach (low/high handoff) and
/// Knudsen number (continuum/FM bridging). The blend is
/// deterministic; the scenario declares all three sub-methods and
/// the bridge function at scenario load.
pub struct HybridAeroMethod {
    /// Continuum, low-Mach (subsonic/transonic/supersonic) method —
    /// typically a [`crate::method::DeckLookup`].
    pub continuum_low_mach: Box<dyn AeroMethod>,
    /// Continuum, high-Mach method — typically
    /// [`crate::hypersonic::ModifiedNewtonian`] or
    /// [`crate::hypersonic::TangentCone`].
    pub continuum_high_mach: Box<dyn AeroMethod>,
    /// Free-molecular method.
    pub free_molecular: Box<dyn AeroMethod>,
    /// Mach number at which low/high handoff occurs (default 4.0).
    pub mach_handoff: f64,
    /// Knudsen-number bridge.
    pub bridge: Box<dyn BridgeFunction>,
    /// Knudsen number to feed the bridge — set per-call by the
    /// caller via [`Self::with_knudsen`] before evaluation.
    pub knudsen: f64,
}

impl std::fmt::Debug for HybridAeroMethod {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HybridAeroMethod")
            .field("mach_handoff", &self.mach_handoff)
            .field("knudsen", &self.knudsen)
            .finish_non_exhaustive()
    }
}

impl HybridAeroMethod {
    /// Stamp the Knudsen number for the next evaluation.
    #[must_use]
    pub fn with_knudsen(mut self, kn: f64) -> Self {
        self.knudsen = kn;
        self
    }
}

impl AeroMethod for HybridAeroMethod {
    fn aero_force_moment_body(&self, ctx: &AeroContext) -> Result<AeroForceMomentBody, AeroError> {
        let continuum = if ctx.mach >= self.mach_handoff {
            self.continuum_high_mach.aero_force_moment_body(ctx)?
        } else {
            self.continuum_low_mach.aero_force_moment_body(ctx)?
        };
        let fm = self.free_molecular.aero_force_moment_body(ctx)?;
        let alpha = self.bridge.alpha(self.knudsen).clamp(0.0, 1.0);
        let blend = |c: f64, f: f64| (1.0 - alpha) * c + alpha * f;
        Ok(AeroForceMomentBody {
            force_n_body: Vector3::new(
                blend(continuum.force_n_body.x, fm.force_n_body.x),
                blend(continuum.force_n_body.y, fm.force_n_body.y),
                blend(continuum.force_n_body.z, fm.force_n_body.z),
            ),
            moment_n_m_body: Vector3::new(
                blend(continuum.moment_n_m_body.x, fm.moment_n_m_body.x),
                blend(continuum.moment_n_m_body.y, fm.moment_n_m_body.y),
                blend(continuum.moment_n_m_body.z, fm.moment_n_m_body.z),
            ),
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
    use crate::hypersonic::ModifiedNewtonian;
    use approx::assert_relative_eq;

    #[test]
    fn mean_free_path_at_sea_level_is_microns() {
        // At T = 288 K, p = 101_325 Pa: λ ≈ 6.6e-8 m (~66 nm).
        let lam = mean_free_path_m(288.15, 101_325.0);
        assert!(lam > 1.0e-8 && lam < 1.0e-6, "λ = {lam}");
    }

    #[test]
    fn mean_free_path_at_orbital_altitude_is_meters() {
        // At T = 1000 K, p = 1e-5 Pa (~400 km): λ ≈ 100s of meters.
        let lam = mean_free_path_m(1000.0, 1.0e-5);
        assert!(lam > 1.0, "λ = {lam}");
    }

    #[test]
    fn invalid_mean_free_path_inputs_fail_closed_to_infinity() {
        assert!(mean_free_path_m(-1.0, 101_325.0).is_infinite());
        assert!(mean_free_path_m(288.15, -1.0).is_infinite());
        assert!(knudsen_number(-1.0, 1.0).is_infinite());
        assert!(knudsen_number(f64::NAN, 1.0).is_infinite());
    }

    #[test]
    fn knudsen_classification() {
        assert_eq!(GasRegime::from_knudsen(1.0e-4), GasRegime::Continuum);
        assert_eq!(GasRegime::from_knudsen(0.05), GasRegime::Slip);
        assert_eq!(GasRegime::from_knudsen(1.0), GasRegime::Transition);
        assert_eq!(GasRegime::from_knudsen(1.0e4), GasRegime::FreeMolecular);
    }

    #[test]
    fn cheng_bridge_limits() {
        let b = ChengBridge;
        // At very small Kn → α ≈ 0.
        assert!(b.alpha(1.0e-6) < 1.0e-6);
        // At very large Kn → α ≈ 1.
        assert!(b.alpha(1.0e6) > 0.99);
        assert_relative_eq!(b.alpha(f64::INFINITY), 1.0, epsilon = 0.0);
        // Monotonic.
        let mut prev = -1.0;
        for kn in [0.01_f64, 0.1, 1.0, 10.0, 100.0] {
            let a = b.alpha(kn);
            assert!(a >= prev, "non-monotone at kn={kn}: prev={prev}, a={a}");
            prev = a;
        }
    }

    #[test]
    fn linear_bridge_limits() {
        let b = LinearKnudsenBridge::default();
        assert_eq!(b.alpha(1.0e-5), 0.0);
        assert_eq!(b.alpha(1.0e5), 1.0);
        assert_relative_eq!(b.alpha(f64::INFINITY), 1.0, epsilon = 0.0);
        // Midpoint of the band.
        let mid_kn = 0.5 * (b.kn_lo + b.kn_hi);
        let a_mid = b.alpha(mid_kn);
        assert_relative_eq!(a_mid, 0.5, max_relative = 1e-9);
    }

    #[test]
    fn erfc_bridge_limits() {
        let b = ErfcBridge::default();
        assert!(b.alpha(1.0e-6) < 0.05);
        assert!(b.alpha(1.0e6) > 0.95);
        assert_relative_eq!(b.alpha(f64::INFINITY), 1.0, epsilon = 0.0);
    }

    #[test]
    fn bridge_nan_inputs_fail_closed_to_free_molecular() {
        assert_relative_eq!(ChengBridge.alpha(f64::NAN), 1.0, epsilon = 0.0);
        assert_relative_eq!(ErfcBridge::default().alpha(f64::NAN), 1.0, epsilon = 0.0);
        assert_relative_eq!(
            LinearKnudsenBridge::default().alpha(f64::NAN),
            1.0,
            epsilon = 0.0
        );
    }

    #[test]
    fn erfc_approx_matches_known_values() {
        // erfc(0) = 1, erfc(1) ≈ 0.157, erfc(-1) ≈ 1.843
        assert!((erfc_approx(0.0) - 1.0).abs() < 1.0e-7);
        assert!((erfc_approx(1.0) - 0.157_299).abs() < 1.0e-4);
        assert!((erfc_approx(-1.0) - 1.842_701).abs() < 1.0e-4);
    }

    #[test]
    fn free_molecular_cd_at_zero_alpha_is_zero() {
        let fm = FreeMolecularAero {
            accommodation: AccommodationCoeffs::default(),
            reference_area_m2: 1.0,
        };
        assert_relative_eq!(fm.cd(0.0), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn free_molecular_force_at_zero_alpha_is_zero() {
        let fm = FreeMolecularAero {
            accommodation: AccommodationCoeffs::default(),
            reference_area_m2: 1.0,
        };
        let ctx = AeroContext {
            mach: 20.0,
            alpha_deg: 0.0,
            beta_deg: 0.0,
            dynamic_pressure_pa: 1.0e-3,
        };
        let force = fm.aero_force_moment_body(&ctx).unwrap();
        assert_relative_eq!(force.force_n_body.x, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn free_molecular_rejects_non_finite_angle() {
        let fm = FreeMolecularAero {
            accommodation: AccommodationCoeffs::default(),
            reference_area_m2: 1.0,
        };
        let ctx = AeroContext {
            mach: 20.0,
            alpha_deg: f64::NAN,
            beta_deg: 0.0,
            dynamic_pressure_pa: 1.0e-3,
        };
        assert!(matches!(
            fm.aero_force_moment_body(&ctx),
            Err(AeroError::NonFinite { .. })
        ));
    }

    #[test]
    fn free_molecular_rejects_negative_dynamic_pressure() {
        let fm = FreeMolecularAero {
            accommodation: AccommodationCoeffs::default(),
            reference_area_m2: 1.0,
        };
        let ctx = AeroContext {
            mach: 20.0,
            alpha_deg: 45.0,
            beta_deg: 0.0,
            dynamic_pressure_pa: -1.0e-3,
        };
        assert!(matches!(
            fm.aero_force_moment_body(&ctx),
            Err(AeroError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn free_molecular_cd_broadside_is_pressure_limit() {
        let fm = FreeMolecularAero {
            accommodation: AccommodationCoeffs {
                tangential: 1.0,
                normal: 1.0,
            },
            reference_area_m2: 1.0,
        };
        assert_relative_eq!(
            fm.cd(std::f64::consts::FRAC_PI_2),
            2.0,
            max_relative = 1e-12
        );
    }

    #[test]
    fn free_molecular_cd_increases_with_alpha() {
        let fm = FreeMolecularAero {
            accommodation: AccommodationCoeffs::default(),
            reference_area_m2: 1.0,
        };
        let small = fm.cd(0.1);
        let big = fm.cd(0.5);
        assert!(big > small);
    }

    #[test]
    fn hybrid_continuum_limit_at_zero_kn() {
        let hybrid = HybridAeroMethod {
            continuum_low_mach: Box::new(ModifiedNewtonian {
                cp_max: 1.0,
                reference_area_m2: 1.0,
                reference_length_m: 1.0,
            }),
            continuum_high_mach: Box::new(ModifiedNewtonian {
                cp_max: 1.0,
                reference_area_m2: 1.0,
                reference_length_m: 1.0,
            }),
            free_molecular: Box::new(FreeMolecularAero {
                accommodation: AccommodationCoeffs::default(),
                reference_area_m2: 1.0,
            }),
            mach_handoff: 4.0,
            bridge: Box::new(ChengBridge),
            knudsen: 1.0e-6,
        };
        let ctx = AeroContext {
            mach: 10.0,
            alpha_deg: 5.0,
            beta_deg: 0.0,
            dynamic_pressure_pa: 1.0e4,
        };
        let f = hybrid.aero_force_moment_body(&ctx).unwrap();
        // At Kn ≈ 0, force should be dominated by the continuum
        // contribution. The Newtonian Cp = 1 panel at α = 5° gives
        // a finite negative drag along body -x.
        assert!(f.force_n_body.x < 0.0);
    }

    #[test]
    fn hybrid_free_molecular_limit_at_high_kn() {
        let hybrid = HybridAeroMethod {
            continuum_low_mach: Box::new(ModifiedNewtonian {
                cp_max: 1.0,
                reference_area_m2: 1.0,
                reference_length_m: 1.0,
            }),
            continuum_high_mach: Box::new(ModifiedNewtonian {
                cp_max: 1.0,
                reference_area_m2: 1.0,
                reference_length_m: 1.0,
            }),
            free_molecular: Box::new(FreeMolecularAero {
                accommodation: AccommodationCoeffs::default(),
                reference_area_m2: 1.0,
            }),
            mach_handoff: 4.0,
            bridge: Box::new(LinearKnudsenBridge {
                kn_lo: 0.01,
                kn_hi: 1.0,
            }),
            knudsen: 100.0,
        };
        let ctx = AeroContext {
            mach: 10.0,
            alpha_deg: 5.0,
            beta_deg: 0.0,
            dynamic_pressure_pa: 1.0e4,
        };
        let f = hybrid.aero_force_moment_body(&ctx).unwrap();
        // FM limit: pure FM contribution, no continuum blending.
        assert!(f.force_n_body.x < 0.0);
    }

    #[test]
    fn hybrid_cheng_bridge_uses_free_molecular_limit_at_infinite_kn() {
        let fm = FreeMolecularAero {
            accommodation: AccommodationCoeffs::default(),
            reference_area_m2: 1.0,
        };
        let hybrid = HybridAeroMethod {
            continuum_low_mach: Box::new(ModifiedNewtonian {
                cp_max: 2.0,
                reference_area_m2: 1.0,
                reference_length_m: 1.0,
            }),
            continuum_high_mach: Box::new(ModifiedNewtonian {
                cp_max: 2.0,
                reference_area_m2: 1.0,
                reference_length_m: 1.0,
            }),
            free_molecular: Box::new(fm),
            mach_handoff: 4.0,
            bridge: Box::new(ChengBridge),
            knudsen: f64::INFINITY,
        };
        let ctx = AeroContext {
            mach: 10.0,
            alpha_deg: 5.0,
            beta_deg: 0.0,
            dynamic_pressure_pa: 1.0e4,
        };
        let f = hybrid.aero_force_moment_body(&ctx).unwrap();
        let fm_only = fm.aero_force_moment_body(&ctx).unwrap();
        assert_relative_eq!(
            f.force_n_body.x,
            fm_only.force_n_body.x,
            max_relative = 1.0e-12
        );
    }

    #[test]
    fn determinism_two_runs_byte_identical() {
        let b = ChengBridge;
        let a = b.alpha(0.5);
        let bv = b.alpha(0.5);
        assert_eq!(a.to_bits(), bv.to_bits());
    }
}
