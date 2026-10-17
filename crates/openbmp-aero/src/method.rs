//! Aerodynamics method abstraction and the Schema-1 deck-lookup
//! implementation.
//!
//! [`AeroMethod`] is the pluggable seam the kernel adapts into
//! its `ForceModel` / `MomentModel` chain at a higher layer. The
//! base tabulated implementation is [`DeckLookup`], which wraps an
//! [`crate::AeroDeck`] and produces body-frame force and moment from
//! `(mach, alpha_deg, beta_deg, dynamic_pressure_pa)`.
//!
//! # Body-frame mapping
//!
//! If a deck carries explicit body-axis six-coefficient tables,
//! `(CX, CY, CZ, Cl, Cm, Cn)` are composed directly as
//! `F_body = q · S · (CX, CY, CZ)` and
//! `M_body = q · S · L · (Cl, Cm, Cn)`.
//!
//! Otherwise the deck's `(CN, CD, CM)` are interpreted as axisymmetric
//! **reduced** coefficients. The body-frame force / moment composition
//! is the small-angle / axisymmetric mapping documented in
//! `docs/software-architecture.md#deck-format-in-house-toml`:
//!
//! * `CD` opposes the relative wind. In body frame this is
//!   approximated as `F_body_x = -CD · q · S` (drag along
//!   body-`-x̂`, where body-`+x̂` is nose-forward at ignition).
//! * `CN` is the normal-force magnitude in the wind / body
//!   longitudinal plane. In body frame this is taken as
//!   `F_body_z = -CN · q · S` — i.e., the deck designer encodes any
//!   side-force from non-zero `β` into the deck's CN axis values
//!   rather than splitting out a separate y-channel.
//! * `CM` is the pitching-moment coefficient about body-`ŷ`,
//!   non-dimensionalised by reference length: `M_body_y = CM · q · S · L`.
//! * Roll moment is identically zero by axisymmetry. Yaw moment is
//!   zero in Schema-1.
//!
//! `q = dynamic_pressure_pa = 0.5 · ρ · V_∞²` is supplied by the
//! caller from the env / state slice. `S = reference_area_m2` and
//! `L = reference_length_m` are deck-side reference quantities. If a
//! deck carries dynamic derivative tables, `DeckLookup` also adds
//! `q·S·L·D·(p̂, q̂, r̂)`, where `D` maps body-rate inputs into
//! `(Cl, Cm, Cn)` and `(p̂, q̂, r̂) = (p, q, r)·L/(2V)`.

#[cfg(not(feature = "std"))]
use alloc::{boxed::Box, collections::BTreeMap};
#[cfg(feature = "std")]
use std::collections::BTreeMap;

use nalgebra::Vector3;

#[cfg(test)]
use crate::deck::AeroDynamicDerivativeTableSet;
use crate::deck::{AeroCoefficients, AeroDeck, AeroSixCoefficients};
use crate::error::AeroError;

// ---------------------------------------------------------------------
// AeroContext / AeroForceMomentBody
// ---------------------------------------------------------------------

/// Inputs for an aerodynamics-method evaluation. All quantities are
/// raw `f64` in SI units, named with their unit suffix.
///
/// `mach` is dimensionless. `alpha_deg` is the angle of attack in
/// the body x-z plane in **degrees**. `beta_deg` is the side-slip
/// angle in the body x-y plane in **degrees**.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AeroContext {
    /// Free-stream Mach number.
    pub mach: f64,
    /// Angle of attack (deg).
    pub alpha_deg: f64,
    /// Side-slip angle (deg).
    pub beta_deg: f64,
    /// Dynamic pressure `q = 0.5 · ρ · V_∞²` (Pa).
    pub dynamic_pressure_pa: f64,
    /// Freestream speed magnitude `V_∞` (m/s).
    pub airspeed_m_s: f64,
    /// Body-frame angular rates `(p, q, r)` (rad/s) used by optional
    /// dynamic derivative tables.
    pub body_angular_rates_rad_s: Vector3<f64>,
    /// Knudsen number based on the method's reference length.
    pub knudsen: f64,
    /// Reynolds number based on the method's reference length.
    pub reynolds_length: f64,
}

/// Aerodynamic force and moment in the body frame.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AeroForceMomentBody {
    /// Force in body frame (N).
    pub force_n_body: Vector3<f64>,
    /// Moment in body frame about the body origin (N·m).
    pub moment_n_m_body: Vector3<f64>,
}

// ---------------------------------------------------------------------
// AeroMethod trait
// ---------------------------------------------------------------------

/// Trait implemented by aerodynamics-method providers.
///
/// [`DeckLookup`] is the tabulated provider. Hypersonic methods
/// (`ModifiedNewtonian`, `TangentCone`, `LocalInclinationPanels`,
/// `FreeMolecular`) and a `HybridAeroMethod` that dispatches between
/// them based on Mach and Knudsen number round out the family.
pub trait AeroMethod {
    /// Body-frame force and moment from the supplied flight-condition
    /// context.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::OutOfEnvelope`] when the `(mach, α, β)`
    /// query falls outside the method's validity envelope and the
    /// method is configured fail-closed. Returns
    /// [`AeroError::NonFinite`] when an input is `NaN` / `Inf` or an
    /// arithmetic step produces a non-finite value.
    fn aero_force_moment_body(&self, ctx: &AeroContext) -> Result<AeroForceMomentBody, AeroError>;
}

impl<T: AeroMethod + ?Sized> AeroMethod for Box<T> {
    fn aero_force_moment_body(&self, ctx: &AeroContext) -> Result<AeroForceMomentBody, AeroError> {
        self.as_ref().aero_force_moment_body(ctx)
    }
}

// ---------------------------------------------------------------------
// DeckLookup
// ---------------------------------------------------------------------

/// Aerodynamics method backed by a tabulated [`AeroDeck`].
///
/// Composes explicit six-coefficient decks directly, or reduced
/// `(CN, CD, CM)` decks via the Schema-1 mapping documented at the top of this
/// module.
#[derive(Clone, Debug, PartialEq)]
pub struct DeckLookup {
    deck: AeroDeck,
}

impl DeckLookup {
    /// Wrap an [`AeroDeck`] for use as an aerodynamics method.
    #[must_use]
    pub const fn new(deck: AeroDeck) -> Self {
        Self { deck }
    }

    /// Read-only access to the underlying deck.
    #[must_use]
    pub const fn deck(&self) -> &AeroDeck {
        &self.deck
    }
}

impl AeroMethod for DeckLookup {
    fn aero_force_moment_body(&self, ctx: &AeroContext) -> Result<AeroForceMomentBody, AeroError> {
        if !ctx.dynamic_pressure_pa.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "dynamic pressure is NaN or infinite",
            });
        }
        if ctx.dynamic_pressure_pa < 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "dynamic pressure is negative",
            });
        }

        // Schema-1 decks ignore the deflections map. Schema-2
        // consumers thread the live `EffectorActualsView` from the kernel
        // through a higher-level adapter; this method-level
        // entry point keeps a Schema-1-only signature for now.
        let deflections = BTreeMap::<&str, f64>::new();
        let six = self
            .deck
            .lookup_six(ctx.mach, ctx.alpha_deg, ctx.beta_deg, &deflections)?;
        let dynamic_derivatives = self.deck.lookup_dynamic_derivatives(
            ctx.mach,
            ctx.alpha_deg,
            ctx.beta_deg,
            &deflections,
        )?;
        let reduced = if six.is_none() {
            Some(
                self.deck
                    .lookup(ctx.mach, ctx.alpha_deg, ctx.beta_deg, &deflections)?,
            )
        } else {
            None
        };

        let q_s = ctx.dynamic_pressure_pa * self.deck.reference_area_m2();
        if !q_s.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "dynamic pressure times reference area is non-finite",
            });
        }
        let q_s_l = q_s * self.deck.reference_length_m();
        if !q_s_l.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "dynamic pressure times reference area and length is non-finite",
            });
        }

        let (force_n_body, mut moment_n_m_body) = if let Some(AeroSixCoefficients {
            cx,
            cy,
            cz,
            cl,
            cm,
            cn,
        }) = six
        {
            // Explicit body-axis six-coefficient mapping:
            //   F_body = q·S · (CX, CY, CZ)
            //   M_body = q·S·L · (Cl, Cm, Cn)
            (
                Vector3::new(cx * q_s, cy * q_s, cz * q_s),
                Vector3::new(cl * q_s_l, cm * q_s_l, cn * q_s_l),
            )
        } else {
            let Some(AeroCoefficients { cn, cd, cm }) = reduced else {
                return Err(AeroError::MalformedDeck {
                    reason: "deck lookup did not return reduced or six coefficients",
                });
            };
            // Schema-1 reduced mapping (small-angle / axisymmetric):
            //   F_body = q·S · (-CD, 0, -CN)
            //   M_body = q·S·L · (0, CM, 0)
            // Locked operand order: deck-side coefficient × q × S
            // (and × L for the moment).
            (
                Vector3::new(-cd * q_s, 0.0, -cn * q_s),
                Vector3::new(0.0, cm * q_s_l, 0.0),
            )
        };
        if let Some(derivatives) = dynamic_derivatives
            && ctx.dynamic_pressure_pa > 0.0
        {
            if !(ctx.body_angular_rates_rad_s.x.is_finite()
                && ctx.body_angular_rates_rad_s.y.is_finite()
                && ctx.body_angular_rates_rad_s.z.is_finite())
            {
                return Err(AeroError::NonFinite {
                    reason: "body angular rates are NaN or infinite",
                });
            }
            let rates_norm_sq = ctx.body_angular_rates_rad_s.norm_squared();
            if rates_norm_sq > 0.0 {
                if !ctx.airspeed_m_s.is_finite() || ctx.airspeed_m_s <= 0.0 {
                    return Err(AeroError::InvalidParameter {
                        reason: "airspeed must be positive when dynamic derivatives are active",
                    });
                }
                let rate_scale = self.deck.reference_length_m() / (2.0 * ctx.airspeed_m_s);
                let p_hat = ctx.body_angular_rates_rad_s.x * rate_scale;
                let q_hat = ctx.body_angular_rates_rad_s.y * rate_scale;
                let r_hat = ctx.body_angular_rates_rad_s.z * rate_scale;
                moment_n_m_body += q_s_l
                    * Vector3::new(
                        derivatives.cl_p * p_hat
                            + derivatives.cl_q * q_hat
                            + derivatives.cl_r * r_hat,
                        derivatives.cm_p * p_hat
                            + derivatives.cm_q * q_hat
                            + derivatives.cm_r * r_hat,
                        derivatives.cn_p * p_hat
                            + derivatives.cn_q * q_hat
                            + derivatives.cn_r * r_hat,
                    );
            }
        }
        if !(force_n_body.x.is_finite()
            && force_n_body.y.is_finite()
            && force_n_body.z.is_finite()
            && moment_n_m_body.x.is_finite()
            && moment_n_m_body.y.is_finite()
            && moment_n_m_body.z.is_finite())
        {
            return Err(AeroError::NonFinite {
                reason: "body-frame aero force or moment is non-finite",
            });
        }

        Ok(AeroForceMomentBody {
            force_n_body,
            moment_n_m_body,
        })
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    /// A 2 x 2 x 1 deck where (CN, CD, CM) at corner (im, ia, 0)
    /// equals `(im + ia + 0.1, 0.5, im - ia)`. Reference area
    /// 0.196 m² (≈ 80 mm body diameter), reference length 0.5 m.
    fn fixture_deck() -> AeroDeck {
        let mach = vec![0.0, 1.0];
        let alpha = vec![-1.0, 1.0];
        let beta = vec![0.0];
        let mut cn = Vec::new();
        let mut cd = Vec::new();
        let mut cm = Vec::new();
        for &m in &mach {
            for &a in &alpha {
                cn.push(m + a + 0.1);
                cd.push(0.5);
                cm.push(m - a);
            }
        }
        AeroDeck::new(mach, alpha, beta, cn, cd, cm, 0.196, 0.5).unwrap()
    }

    fn six_fixture_deck() -> AeroDeck {
        AeroDeck::new(
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![99.0],
            vec![99.0],
            vec![99.0],
            0.196,
            0.5,
        )
        .unwrap()
        .with_six_coefficient_tables(
            vec![0.10],
            vec![-0.20],
            vec![0.30],
            vec![-0.010],
            vec![0.020],
            vec![-0.030],
        )
        .unwrap()
    }

    fn dynamic_derivative_fixture_deck() -> AeroDeck {
        AeroDeck::new(
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.02],
            2.0,
            4.0,
        )
        .unwrap()
        .with_dynamic_derivative_table_set(AeroDynamicDerivativeTableSet {
            cl_p: vec![-7.0],
            cl_q: vec![-0.5],
            cl_r: vec![-0.25],
            cm_p: vec![-0.1],
            cm_q: vec![-3.0],
            cm_r: vec![-0.2],
            cn_p: vec![-0.3],
            cn_q: vec![-0.4],
            cn_r: vec![-5.0],
        })
        .unwrap()
    }

    #[test]
    fn zero_dynamic_pressure_yields_zero_force_and_moment() {
        let method = DeckLookup::new(fixture_deck());
        let result = method
            .aero_force_moment_body(&AeroContext {
                mach: 0.5,
                alpha_deg: 0.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: 0.0,
                airspeed_m_s: 0.0,
                body_angular_rates_rad_s: Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap();
        assert_eq!(result.force_n_body, Vector3::new(0.0, 0.0, 0.0));
        assert_eq!(result.moment_n_m_body, Vector3::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn corner_lookup_produces_expected_body_force_and_moment() {
        let deck = fixture_deck();
        let area = deck.reference_area_m2();
        let length = deck.reference_length_m();
        let q_pa = 1000.0;

        let method = DeckLookup::new(deck);

        // Corner (mach=1, alpha=1, beta=0): CN = 1 + 1 + 0.1 = 2.1,
        // CD = 0.5, CM = 1 - 1 = 0.
        let result = method
            .aero_force_moment_body(&AeroContext {
                mach: 1.0,
                alpha_deg: 1.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: q_pa,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap();
        assert_abs_diff_eq!(result.force_n_body.x, -0.5 * q_pa * area);
        assert_abs_diff_eq!(result.force_n_body.y, 0.0);
        assert_abs_diff_eq!(result.force_n_body.z, -2.1 * q_pa * area);
        assert_abs_diff_eq!(result.moment_n_m_body.x, 0.0);
        assert_abs_diff_eq!(result.moment_n_m_body.y, 0.0 * q_pa * area * length);
        assert_abs_diff_eq!(result.moment_n_m_body.z, 0.0);
    }

    #[test]
    fn six_coefficient_deck_lookup_maps_body_axes_directly() {
        let deck = six_fixture_deck();
        let area = deck.reference_area_m2();
        let length = deck.reference_length_m();
        let method = DeckLookup::new(deck);
        let q_pa = 1000.0;

        let result = method
            .aero_force_moment_body(&AeroContext {
                mach: 0.0,
                alpha_deg: 0.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: q_pa,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap();

        assert_abs_diff_eq!(result.force_n_body.x, 0.10 * q_pa * area, epsilon = 1e-12);
        assert_abs_diff_eq!(result.force_n_body.y, -0.20 * q_pa * area, epsilon = 1e-12);
        assert_abs_diff_eq!(result.force_n_body.z, 0.30 * q_pa * area, epsilon = 1e-12);
        assert_abs_diff_eq!(
            result.moment_n_m_body.x,
            -0.010 * q_pa * area * length,
            epsilon = 1e-12
        );
        assert_abs_diff_eq!(
            result.moment_n_m_body.y,
            0.020 * q_pa * area * length,
            epsilon = 1e-12
        );
        assert_abs_diff_eq!(
            result.moment_n_m_body.z,
            -0.030 * q_pa * area * length,
            epsilon = 1e-12
        );
    }

    #[test]
    fn deck_lookup_applies_dynamic_derivative_rate_moments() {
        let method = DeckLookup::new(dynamic_derivative_fixture_deck());
        let result = method
            .aero_force_moment_body(&AeroContext {
                mach: 0.0,
                alpha_deg: 0.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: 10.0,
                airspeed_m_s: 20.0,
                body_angular_rates_rad_s: Vector3::new(2.0, 3.0, 4.0),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap();

        assert_eq!(result.force_n_body, Vector3::zeros());
        assert_abs_diff_eq!(result.moment_n_m_body.x, -132.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(result.moment_n_m_body.y, -78.4, epsilon = 1.0e-12);
        assert_abs_diff_eq!(result.moment_n_m_body.z, -174.4, epsilon = 1.0e-12);
    }

    #[test]
    fn alpha_sign_flips_normal_force_when_cn_is_odd_in_alpha() {
        // The fixture deck has CN(α=-1) = m - 1 + 0.1, CN(α=+1) = m + 1 + 0.1.
        // So CN flips sign across α = 0 only when m = -0.1, but the
        // fixture has m ≥ 0. Instead, we verify the body-z force
        // tracks the deck's CN sign at fixed mach as α moves.
        let method = DeckLookup::new(fixture_deck());
        let q_pa = 100.0;
        let r_minus = method
            .aero_force_moment_body(&AeroContext {
                mach: 0.0,
                alpha_deg: -1.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: q_pa,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap();
        let r_plus = method
            .aero_force_moment_body(&AeroContext {
                mach: 0.0,
                alpha_deg: 1.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: q_pa,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap();
        // CN at (m=0, α=-1) = -0.9; at (m=0, α=+1) = +1.1.
        // Body-z force = -CN · q · S, so r_minus.z > 0 and r_plus.z < 0.
        assert!(r_minus.force_n_body.z > 0.0);
        assert!(r_plus.force_n_body.z < 0.0);
    }

    #[test]
    fn out_of_envelope_query_propagates_from_deck() {
        let method = DeckLookup::new(fixture_deck());
        // Mach grid is [0, 1]; query at 5 with default fail-closed.
        let err = method
            .aero_force_moment_body(&AeroContext {
                mach: 5.0,
                alpha_deg: 0.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: 1000.0,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap_err();
        assert!(matches!(err, AeroError::OutOfEnvelope { .. }));
    }

    #[test]
    fn non_finite_dynamic_pressure_rejected() {
        let method = DeckLookup::new(fixture_deck());
        let err = method
            .aero_force_moment_body(&AeroContext {
                mach: 0.5,
                alpha_deg: 0.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: f64::NAN,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap_err();
        assert!(matches!(err, AeroError::NonFinite { .. }));
    }

    #[test]
    fn negative_dynamic_pressure_rejected() {
        let method = DeckLookup::new(fixture_deck());
        let err = method
            .aero_force_moment_body(&AeroContext {
                mach: 0.5,
                alpha_deg: 0.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: -1.0,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap_err();
        assert!(matches!(err, AeroError::InvalidParameter { .. }));
    }

    #[test]
    fn overflowing_force_component_is_rejected() {
        let deck = AeroDeck::new(
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![2.0],
            vec![2.0],
            vec![0.0],
            1.0,
            1.0,
        )
        .unwrap();
        let method = DeckLookup::new(deck);
        let err = method
            .aero_force_moment_body(&AeroContext {
                mach: 0.0,
                alpha_deg: 0.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: f64::MAX,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap_err();
        assert!(matches!(err, AeroError::NonFinite { .. }));
    }

    #[test]
    fn overflowing_moment_scale_is_rejected() {
        let deck = AeroDeck::new(
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![1.0],
            1.0,
            2.0,
        )
        .unwrap();
        let method = DeckLookup::new(deck);
        let err = method
            .aero_force_moment_body(&AeroContext {
                mach: 0.0,
                alpha_deg: 0.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: f64::MAX,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            })
            .unwrap_err();
        assert!(matches!(err, AeroError::NonFinite { .. }));
    }

    #[test]
    fn body_frame_force_and_moment_are_bit_stable_across_two_evaluations() {
        let method = DeckLookup::new(fixture_deck());
        let ctx = AeroContext {
            mach: 0.5,
            alpha_deg: 0.5,
            beta_deg: 0.0,
            dynamic_pressure_pa: 1234.5,
            airspeed_m_s: 100.0,
            body_angular_rates_rad_s: Vector3::zeros(),
            knudsen: 0.0,
            reynolds_length: 0.0,
        };
        let first = method.aero_force_moment_body(&ctx).unwrap();
        let second = method.aero_force_moment_body(&ctx).unwrap();
        assert_eq!(
            first.force_n_body.x.to_bits(),
            second.force_n_body.x.to_bits()
        );
        assert_eq!(
            first.force_n_body.y.to_bits(),
            second.force_n_body.y.to_bits()
        );
        assert_eq!(
            first.force_n_body.z.to_bits(),
            second.force_n_body.z.to_bits()
        );
        assert_eq!(
            first.moment_n_m_body.x.to_bits(),
            second.moment_n_m_body.x.to_bits()
        );
        assert_eq!(
            first.moment_n_m_body.y.to_bits(),
            second.moment_n_m_body.y.to_bits()
        );
        assert_eq!(
            first.moment_n_m_body.z.to_bits(),
            second.moment_n_m_body.z.to_bits()
        );
    }
}
