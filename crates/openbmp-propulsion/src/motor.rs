//! Motor trait and the [`SolidMotor`] implementation.
//!
//! # Trait surface
//!
//! [`Motor`] is the crate-local trait the propulsion crate exposes.
//! The architecture's long-term intent is `Motor: ForceModel +
//! MassModel`, but the kernel's `ForceModel` / `MassModel` traits
//! live in `openbmp-sim` (L1) and `openbmp-propulsion` is L2, so
//! the kernel-side wrapper lives in `openbmp-vehicle` alongside the
//! gravity / atmosphere / wind / aero adapters. This trait
//! exposes the underlying capabilities directly:
//!
//! * [`Motor::thrust_n_at`] — instantaneous thrust at
//!   `t_since_ignition_s` (N).
//! * [`Motor::mass_kg`] — instantaneous total motor mass at
//!   `t_since_ignition_s` (kg).
//! * [`Motor::mass_rate_kg_s`] — instantaneous mass-flow rate at
//!   `t_since_ignition_s` (kg/s; ≤ 0 everywhere).
//!
//! # Solid-motor mass model
//!
//! Mass is impulse-weighted: `dm/dt = -F(t) · m_p / I_total`, so
//! `m(t) = m_dry + m_p · (1 − I_consumed(t) / I_total)`. This
//! matches the RASP / OpenRocket convention (mass loss tracks
//! instantaneous thrust, not elapsed time) and ensures
//! `m(burn_duration) = m_dry` exactly. The constructor rejects decks
//! whose declared `total_impulse_n_s` differs from the trapezoidal
//! integral of the thrust curve by more than 1e-6 relative.
//!
//! Outside the burn window (`t < 0` or `t > burn_duration`) thrust
//! is identically zero and mass holds at its boundary value
//! (`m_dry + m_p` before ignition, `m_dry` after burnout). These
//! boundary cases are *not* errors.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; locked operand order on the trapezoidal
//! cumulative-impulse table built at construction; no FMA, no
//! wall-clock, no system RNG, no network, no file I/O on the hot path.

#[cfg(not(feature = "std"))]
use alloc::{string::String, vec::Vec};

use crate::error::MotorError;

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
const TOTAL_IMPULSE_REL_TOL: f64 = 1.0e-6;
const SPECIFIC_IMPULSE_REL_TOL: f64 = 1.0e-6;

// ---------------------------------------------------------------------
// MotorVariant
// ---------------------------------------------------------------------

/// Motor variants the propulsion crate distinguishes. Only
/// [`MotorVariant::Solid`] is implemented; liquid / hybrid / cold-gas
/// remain future work.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum MotorVariant {
    /// Solid-propellant motor with a pre-tabulated thrust curve.
    Solid,
}

// ---------------------------------------------------------------------
// Motor trait
// ---------------------------------------------------------------------

/// Trait implemented by motor models in this crate.
///
/// The kernel-side `ForceModel` / `MassModel` adapters live in
/// `openbmp-vehicle`; this trait surfaces the underlying
/// capabilities directly so consumers can integrate motor effects
/// without the kernel wiring.
pub trait Motor {
    /// Instantaneous thrust at `t_since_ignition_s`, in N.
    ///
    /// Returns `0.0` for `t < 0` (pre-ignition) and
    /// `t > burn_duration_s()` (post-burnout). These boundary cases
    /// are *not* errors.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError::NonFinite`] when `t_since_ignition_s`
    /// is `NaN` or `Inf`.
    fn thrust_n_at(&self, t_since_ignition_s: f64) -> Result<f64, MotorError>;

    /// Total motor mass at `t_since_ignition_s`, in kg.
    ///
    /// Equals `dry_mass + propellant_mass` for `t ≤ 0` and `dry_mass`
    /// for `t ≥ burn_duration_s()`.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError::NonFinite`] when `t_since_ignition_s`
    /// is `NaN` or `Inf`.
    fn mass_kg(&self, t_since_ignition_s: f64) -> Result<f64, MotorError>;

    /// Mass-flow rate at `t_since_ignition_s`, in kg/s.
    ///
    /// Always ≤ 0 (motors lose mass). Identically `0.0` outside the
    /// burn window.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError::NonFinite`] when `t_since_ignition_s`
    /// is `NaN` or `Inf`.
    fn mass_rate_kg_s(&self, t_since_ignition_s: f64) -> Result<f64, MotorError>;

    /// Burn duration in seconds.
    fn burn_duration_s(&self) -> f64;

    /// Declared total impulse in N·s.
    fn total_impulse_n_s(&self) -> f64;

    /// Dry mass (motor case + nozzle, no propellant) in kg.
    fn dry_mass_kg(&self) -> f64;

    /// Propellant mass at ignition in kg.
    fn propellant_mass_kg(&self) -> f64;

    /// Motor variant tag.
    fn variant(&self) -> MotorVariant;
}

// ---------------------------------------------------------------------
// Sub-structs (carried in SolidMotor and produced by the parser)
// ---------------------------------------------------------------------

/// Validation status flag carried in the motor file's `[meta]`
/// block. Mirrors the project-wide validation labels used by the
/// aero deck and provenance records.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub enum Validation {
    /// Implemented or drafted, not independently checked.
    Experimental,
    /// Internally consistent against unit/property tests.
    Checked,
    /// Compared against analytic or simple public examples.
    #[default]
    ValidatedToy,
    /// Compared with public academic benchmark cases.
    Research,
}

/// `[meta]` block of the motor file.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MotorMeta {
    /// Display name of the motor (e.g. `"synthetic-solid-A"`).
    pub name: String,
    /// One-line provenance citation.
    pub provenance: String,
    /// Validation status.
    pub validation: Validation,
}

/// `[burn]` block of the motor file.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BurnSpec {
    /// Total burn duration (s).
    pub duration_s: f64,
    /// Declared total impulse (N·s).
    pub total_impulse_n_s: f64,
    /// Specific impulse (s); checked against total impulse and propellant mass.
    pub specific_impulse_s: f64,
    /// Propellant mass at ignition (kg).
    pub propellant_mass_kg: f64,
    /// Dry-case mass (kg).
    pub dry_mass_kg: f64,
}

/// Tabulated piecewise-linear thrust curve.
///
/// Internally stores the `(t, F)` pairs plus a precomputed
/// cumulative-impulse table `I_consumed(t_i)` so the mass model can
/// look up the consumed impulse at any `t` with one binary search
/// and one trapezoidal partial term, in locked operand order.
#[derive(Clone, Debug, PartialEq)]
pub struct ThrustCurve {
    points: Vec<[f64; 2]>,
    cumulative_impulse_n_s: Vec<f64>,
}

impl ThrustCurve {
    /// Construct from `(t_s, F_n)` pairs in time-ascending order.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError::MalformedMotor`] when the curve has
    /// fewer than two points, the time grid is not strictly
    /// monotone-increasing, or the first point is not `t = 0`.
    /// Returns [`MotorError::NonFinite`] for any non-finite value.
    /// Returns [`MotorError::InvalidParameter`] for a negative thrust
    /// value (negative thrust is rejected). The first and final thrust
    /// values are also required to be
    /// zero, matching the in-house RASP-shaped schema contract.
    pub fn new(points: Vec<[f64; 2]>) -> Result<Self, MotorError> {
        if points.len() < 2 {
            return Err(MotorError::MalformedMotor {
                reason: "thrust curve must have at least two (t, F) points",
            });
        }
        for p in &points {
            if !p[0].is_finite() || !p[1].is_finite() {
                return Err(MotorError::NonFinite {
                    reason: "thrust-curve point is NaN or infinite",
                });
            }
            if p[1] < 0.0 {
                return Err(MotorError::InvalidParameter {
                    reason: "thrust-curve thrust value is negative",
                });
            }
        }
        for w in points.windows(2) {
            if w[1][0] <= w[0][0] {
                return Err(MotorError::MalformedMotor {
                    reason: "thrust-curve time grid must be strictly monotone-increasing",
                });
            }
        }
        if points[0][0] != 0.0 {
            return Err(MotorError::MalformedMotor {
                reason: "thrust curve must start at t = 0",
            });
        }
        if points[0][1] != 0.0 {
            return Err(MotorError::MalformedMotor {
                reason: "thrust curve must start at zero thrust",
            });
        }
        if points[points.len() - 1][1] != 0.0 {
            return Err(MotorError::MalformedMotor {
                reason: "thrust curve final point must have zero thrust",
            });
        }

        // Precompute cumulative trapezoidal impulse at each grid
        // point. Locked operand order: `0.5 * (F_i + F_{i+1}) *
        // (t_{i+1} - t_i)` summed left-to-right.
        let mut cumulative = Vec::with_capacity(points.len());
        cumulative.push(0.0);
        for i in 0..points.len() - 1 {
            let dt = points[i + 1][0] - points[i][0];
            let avg_thrust = 0.5 * (points[i][1] + points[i + 1][1]);
            let segment_impulse = avg_thrust * dt;
            let next = cumulative[i] + segment_impulse;
            if !segment_impulse.is_finite() || !next.is_finite() {
                return Err(MotorError::NonFinite {
                    reason: "thrust-curve impulse integral is NaN or infinite",
                });
            }
            cumulative.push(next);
        }

        Ok(Self {
            points,
            cumulative_impulse_n_s: cumulative,
        })
    }

    /// Last time on the curve (the burnout time per the deck).
    #[must_use]
    pub fn last_time_s(&self) -> f64 {
        self.points[self.points.len() - 1][0]
    }

    /// Trapezoidal integral of the curve from 0 to `last_time_s()`.
    /// Equals the last entry of the precomputed cumulative-impulse
    /// table.
    #[must_use]
    pub fn integrated_impulse_n_s(&self) -> f64 {
        self.cumulative_impulse_n_s[self.cumulative_impulse_n_s.len() - 1]
    }

    /// Read-only access to the underlying `(t, F)` pairs.
    #[must_use]
    pub fn points(&self) -> &[[f64; 2]] {
        &self.points
    }

    fn thrust_at(&self, t: f64) -> f64 {
        let last = self.last_time_s();
        if t <= 0.0 {
            return self.points[0][1];
        }
        if t >= last {
            return self.points[self.points.len() - 1][1];
        }
        let upper = self.points.partition_point(|p| p[0] <= t);
        let i = upper.saturating_sub(1).min(self.points.len() - 2);
        let p0 = self.points[i];
        let p1 = self.points[i + 1];
        let f = (t - p0[0]) / (p1[0] - p0[0]);
        // Locked order: (1 - f) * F_i + f * F_{i+1}, no FMA.
        (1.0 - f) * p0[1] + f * p1[1]
    }

    fn cumulative_impulse_at(&self, t: f64) -> f64 {
        let last = self.last_time_s();
        if t <= 0.0 {
            return 0.0;
        }
        if t >= last {
            return self.integrated_impulse_n_s();
        }
        let upper = self.points.partition_point(|p| p[0] <= t);
        let i = upper.saturating_sub(1).min(self.points.len() - 2);
        let p0 = self.points[i];
        let thrust_t = self.thrust_at(t);
        // Partial trapezoidal segment from t_i to t. Locked order.
        let partial = 0.5 * (p0[1] + thrust_t) * (t - p0[0]);
        self.cumulative_impulse_n_s[i] + partial
    }
}

/// `[geometry]` block of the motor file.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AmbientPressureCorrection {
    /// Simplified model: assume sea-level ambient pressure correction
    /// is already baked into the thrust curve. No altitude correction.
    Constant,
}

/// `[geometry]` block of the motor file.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MotorGeometry {
    /// Nozzle exit area (m²).
    pub exit_area_m2: f64,
    /// Ambient-pressure correction strategy.
    pub ambient_pressure_correction: AmbientPressureCorrection,
}

// ---------------------------------------------------------------------
// SolidMotor
// ---------------------------------------------------------------------

/// Solid motor. Thrust and mass are derived from a
/// pre-tabulated curve plus the declared burn parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct SolidMotor {
    meta: MotorMeta,
    burn: BurnSpec,
    thrust_curve: ThrustCurve,
    geometry: MotorGeometry,
}

impl SolidMotor {
    /// Construct an in-memory solid motor and validate its
    /// structural and finiteness invariants.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError::MalformedMotor`] when the thrust curve's
    /// final time differs from `burn.duration_s` or when `burn.duration_s`
    /// is not positive. Returns [`MotorError::NonFinite`] for any
    /// non-finite value in `burn` or `geometry`. Returns
    /// [`MotorError::InvalidParameter`] for negative masses,
    /// non-positive total impulse, non-positive specific impulse, or
    /// non-positive exit area.
    pub fn new(
        meta: MotorMeta,
        burn: BurnSpec,
        thrust_curve: ThrustCurve,
        geometry: MotorGeometry,
    ) -> Result<Self, MotorError> {
        validate_burn(&burn)?;
        validate_geometry(geometry)?;
        // Bit-exact equality is intentional: the deck file declares
        // `burn.duration_s` and the curve's last `t` separately, and
        // we require they match without rounding so a typo in either
        // surface fails at construction.
        #[allow(clippy::float_cmp)]
        let duration_matches = thrust_curve.last_time_s() == burn.duration_s;
        if !duration_matches {
            return Err(MotorError::MalformedMotor {
                reason: "thrust-curve last point time must equal burn.duration_s exactly",
            });
        }
        validate_total_impulse_matches_curve(&burn, &thrust_curve)?;
        validate_specific_impulse_consistency(&burn)?;
        Ok(Self {
            meta,
            burn,
            thrust_curve,
            geometry,
        })
    }

    /// Read-only access to the meta block.
    #[must_use]
    pub fn meta(&self) -> &MotorMeta {
        &self.meta
    }

    /// Read-only access to the burn block.
    #[must_use]
    pub const fn burn(&self) -> &BurnSpec {
        &self.burn
    }

    /// Read-only access to the thrust curve.
    #[must_use]
    pub const fn thrust_curve(&self) -> &ThrustCurve {
        &self.thrust_curve
    }

    /// Read-only access to the geometry block.
    #[must_use]
    pub const fn geometry(&self) -> &MotorGeometry {
        &self.geometry
    }

    /// Initial total mass at ignition: `dry_mass + propellant_mass`.
    #[must_use]
    pub fn initial_mass_kg(&self) -> f64 {
        self.burn.dry_mass_kg + self.burn.propellant_mass_kg
    }
}

impl Motor for SolidMotor {
    fn thrust_n_at(&self, t_since_ignition_s: f64) -> Result<f64, MotorError> {
        if !t_since_ignition_s.is_finite() {
            return Err(MotorError::NonFinite {
                reason: "thrust query time is NaN or infinite",
            });
        }
        if t_since_ignition_s < 0.0 || t_since_ignition_s > self.burn.duration_s {
            return Ok(0.0);
        }
        Ok(self.thrust_curve.thrust_at(t_since_ignition_s))
    }

    fn mass_kg(&self, t_since_ignition_s: f64) -> Result<f64, MotorError> {
        if !t_since_ignition_s.is_finite() {
            return Err(MotorError::NonFinite {
                reason: "mass query time is NaN or infinite",
            });
        }
        if t_since_ignition_s <= 0.0 {
            return Ok(self.initial_mass_kg());
        }
        if t_since_ignition_s >= self.burn.duration_s {
            return Ok(self.burn.dry_mass_kg);
        }
        // Impulse-weighted mass model:
        //   m(t) = m_dry + m_p · (1 - I_consumed(t) / I_total)
        // where I_total is the declared `burn.total_impulse_n_s` so
        // m(burn_duration) = m_dry exactly when the deck integrates
        // to its declared total impulse. Locked operand order.
        let consumed_fraction = self.thrust_curve.cumulative_impulse_at(t_since_ignition_s)
            / self.burn.total_impulse_n_s;
        let propellant_remaining = self.burn.propellant_mass_kg * (1.0 - consumed_fraction);
        Ok(self.burn.dry_mass_kg + propellant_remaining)
    }

    fn mass_rate_kg_s(&self, t_since_ignition_s: f64) -> Result<f64, MotorError> {
        if !t_since_ignition_s.is_finite() {
            return Err(MotorError::NonFinite {
                reason: "mass-rate query time is NaN or infinite",
            });
        }
        if t_since_ignition_s < 0.0 || t_since_ignition_s > self.burn.duration_s {
            return Ok(0.0);
        }
        // dm/dt = -F(t) · m_p / I_total. Locked order.
        let thrust = self.thrust_curve.thrust_at(t_since_ignition_s);
        let rate = -thrust * self.burn.propellant_mass_kg / self.burn.total_impulse_n_s;
        Ok(rate)
    }

    fn burn_duration_s(&self) -> f64 {
        self.burn.duration_s
    }

    fn total_impulse_n_s(&self) -> f64 {
        self.burn.total_impulse_n_s
    }

    fn dry_mass_kg(&self) -> f64 {
        self.burn.dry_mass_kg
    }

    fn propellant_mass_kg(&self) -> f64 {
        self.burn.propellant_mass_kg
    }

    fn variant(&self) -> MotorVariant {
        MotorVariant::Solid
    }
}

// ---------------------------------------------------------------------
// Internal validators
// ---------------------------------------------------------------------

fn validate_burn(burn: &BurnSpec) -> Result<(), MotorError> {
    for v in [
        burn.duration_s,
        burn.total_impulse_n_s,
        burn.specific_impulse_s,
        burn.propellant_mass_kg,
        burn.dry_mass_kg,
    ] {
        if !v.is_finite() {
            return Err(MotorError::NonFinite {
                reason: "burn parameter is NaN or infinite",
            });
        }
    }
    if burn.duration_s <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "burn duration must be positive",
        });
    }
    if burn.total_impulse_n_s <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "total impulse must be positive",
        });
    }
    if burn.specific_impulse_s <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "specific impulse must be positive",
        });
    }
    if burn.propellant_mass_kg < 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "propellant mass must be non-negative",
        });
    }
    if burn.dry_mass_kg < 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "dry mass must be non-negative",
        });
    }
    if burn.propellant_mass_kg + burn.dry_mass_kg <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "total motor mass must be positive",
        });
    }
    Ok(())
}

fn validate_total_impulse_matches_curve(
    burn: &BurnSpec,
    thrust_curve: &ThrustCurve,
) -> Result<(), MotorError> {
    let integrated = thrust_curve.integrated_impulse_n_s();
    if !integrated.is_finite() {
        return Err(MotorError::NonFinite {
            reason: "integrated thrust-curve impulse is NaN or infinite",
        });
    }
    let rel = (integrated - burn.total_impulse_n_s).abs() / burn.total_impulse_n_s.abs();
    if rel > TOTAL_IMPULSE_REL_TOL {
        return Err(MotorError::InvalidParameter {
            reason: "declared total impulse does not match thrust-curve integral",
        });
    }
    Ok(())
}

fn validate_specific_impulse_consistency(burn: &BurnSpec) -> Result<(), MotorError> {
    let expected_total_impulse =
        burn.propellant_mass_kg * STANDARD_GRAVITY_M_S2 * burn.specific_impulse_s;
    if !expected_total_impulse.is_finite() {
        return Err(MotorError::NonFinite {
            reason: "specific impulse consistency check produced NaN or infinity",
        });
    }
    let rel =
        (expected_total_impulse - burn.total_impulse_n_s).abs() / burn.total_impulse_n_s.abs();
    if rel > SPECIFIC_IMPULSE_REL_TOL {
        return Err(MotorError::InvalidParameter {
            reason: "propellant mass, standard gravity, and specific impulse do not match total impulse",
        });
    }
    Ok(())
}

fn validate_geometry(geometry: MotorGeometry) -> Result<(), MotorError> {
    if !geometry.exit_area_m2.is_finite() {
        return Err(MotorError::NonFinite {
            reason: "exit area is NaN or infinite",
        });
    }
    if geometry.exit_area_m2 <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "exit area must be positive",
        });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    /// Trapezoidal motor: 4 s burn, total impulse exactly 3500 N·s
    /// from the closed-form integral of a (0, 0.5, 3.5, 4) →
    /// (0, 1000, 1000, 0) trapezoidal curve.
    fn trapezoidal_motor() -> SolidMotor {
        let meta = MotorMeta {
            name: "test-trapezoidal".into(),
            provenance: "synthetic test fixture".into(),
            validation: Validation::ValidatedToy,
        };
        let burn = BurnSpec {
            duration_s: 4.0,
            total_impulse_n_s: 3500.0,
            specific_impulse_s: 356.900_674_542_274_9,
            propellant_mass_kg: 1.0,
            dry_mass_kg: 0.5,
        };
        let curve =
            ThrustCurve::new(vec![[0.0, 0.0], [0.5, 1000.0], [3.5, 1000.0], [4.0, 0.0]]).unwrap();
        let geom = MotorGeometry {
            exit_area_m2: 0.0019,
            ambient_pressure_correction: AmbientPressureCorrection::Constant,
        };
        SolidMotor::new(meta, burn, curve, geom).unwrap()
    }

    // -----------------------------------------------------------------
    // Thrust curve interpolation
    // -----------------------------------------------------------------

    #[test]
    fn thrust_at_t_zero_returns_first_point() {
        let m = trapezoidal_motor();
        assert_eq!(m.thrust_n_at(0.0).unwrap().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn thrust_at_burn_duration_returns_last_point() {
        let m = trapezoidal_motor();
        assert_eq!(m.thrust_n_at(4.0).unwrap().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn thrust_at_grid_points_returns_stored_values() {
        let m = trapezoidal_motor();
        assert_eq!(m.thrust_n_at(0.5).unwrap().to_bits(), 1000.0_f64.to_bits());
        assert_eq!(m.thrust_n_at(3.5).unwrap().to_bits(), 1000.0_f64.to_bits());
    }

    #[test]
    fn thrust_at_midpoint_of_segment_interpolates() {
        let m = trapezoidal_motor();
        // Halfway between (0.0, 0.0) and (0.5, 1000.0) at t=0.25
        // gives 500.0 exactly.
        assert_eq!(m.thrust_n_at(0.25).unwrap().to_bits(), 500.0_f64.to_bits());
        // Halfway between (3.5, 1000.0) and (4.0, 0.0) at t=3.75
        // gives 500.0 exactly.
        assert_eq!(m.thrust_n_at(3.75).unwrap().to_bits(), 500.0_f64.to_bits());
        // Anywhere inside the (0.5, 3.5) flat-top segment gives 1000.
        assert_eq!(m.thrust_n_at(2.0).unwrap().to_bits(), 1000.0_f64.to_bits());
    }

    #[test]
    fn thrust_before_ignition_is_zero() {
        let m = trapezoidal_motor();
        assert_eq!(m.thrust_n_at(-1.0).unwrap().to_bits(), 0.0_f64.to_bits());
        assert_eq!(m.thrust_n_at(-0.001).unwrap().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn thrust_after_burnout_is_zero() {
        let m = trapezoidal_motor();
        assert_eq!(m.thrust_n_at(4.001).unwrap().to_bits(), 0.0_f64.to_bits());
        assert_eq!(m.thrust_n_at(100.0).unwrap().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn thrust_rejects_non_finite_time() {
        let m = trapezoidal_motor();
        assert!(matches!(
            m.thrust_n_at(f64::NAN),
            Err(MotorError::NonFinite { .. })
        ));
        assert!(matches!(
            m.thrust_n_at(f64::INFINITY),
            Err(MotorError::NonFinite { .. })
        ));
    }

    // -----------------------------------------------------------------
    // Total impulse identity
    // -----------------------------------------------------------------

    #[test]
    fn integrated_impulse_matches_declared_total_impulse() {
        let m = trapezoidal_motor();
        // Trapezoidal integral closed-form: 0.5·1000·0.5 + 1000·3 + 0.5·1000·0.5 = 3500.
        assert_eq!(
            m.thrust_curve().integrated_impulse_n_s().to_bits(),
            3500.0_f64.to_bits(),
        );
        assert_eq!(
            m.total_impulse_n_s().to_bits(),
            m.thrust_curve().integrated_impulse_n_s().to_bits(),
        );
    }

    // -----------------------------------------------------------------
    // Mass model
    // -----------------------------------------------------------------

    #[test]
    fn mass_at_zero_equals_total_initial_mass() {
        let m = trapezoidal_motor();
        // dry + propellant = 0.5 + 1.0 = 1.5
        assert_eq!(m.mass_kg(0.0).unwrap().to_bits(), 1.5_f64.to_bits());
        assert_eq!(m.mass_kg(-1.0).unwrap().to_bits(), 1.5_f64.to_bits());
    }

    #[test]
    fn mass_at_burnout_equals_dry_mass() {
        let m = trapezoidal_motor();
        assert_eq!(m.mass_kg(4.0).unwrap().to_bits(), 0.5_f64.to_bits());
        assert_eq!(m.mass_kg(100.0).unwrap().to_bits(), 0.5_f64.to_bits());
    }

    #[test]
    fn mass_is_monotone_decreasing_through_burn() {
        let m = trapezoidal_motor();
        let mut last = m.mass_kg(0.0).unwrap();
        let mut t = 0.05_f64;
        while t <= 4.0 {
            let now = m.mass_kg(t).unwrap();
            assert!(now <= last, "mass must be monotone-decreasing, t = {t}");
            last = now;
            t += 0.05;
        }
    }

    #[test]
    fn mass_rate_is_non_positive_throughout_burn() {
        let m = trapezoidal_motor();
        let mut t = 0.0_f64;
        while t <= 4.0 {
            let r = m.mass_rate_kg_s(t).unwrap();
            assert!(r <= 0.0, "mass rate must be ≤ 0 at t = {t}, got {r}");
            t += 0.05;
        }
    }

    #[test]
    fn mass_rate_is_zero_outside_burn() {
        let m = trapezoidal_motor();
        assert_eq!(m.mass_rate_kg_s(-1.0).unwrap().to_bits(), 0.0_f64.to_bits());
        assert_eq!(
            m.mass_rate_kg_s(4.001).unwrap().to_bits(),
            0.0_f64.to_bits()
        );
    }

    // -----------------------------------------------------------------
    // Determinism
    // -----------------------------------------------------------------

    #[test]
    fn thrust_and_mass_lookups_are_bit_stable_across_two_evaluations() {
        let m = trapezoidal_motor();
        for t in [0.0, 0.25, 1.0, 2.0, 3.75, 4.0] {
            let t1 = m.thrust_n_at(t).unwrap();
            let t2 = m.thrust_n_at(t).unwrap();
            assert_eq!(t1.to_bits(), t2.to_bits());
            let m1 = m.mass_kg(t).unwrap();
            let m2 = m.mass_kg(t).unwrap();
            assert_eq!(m1.to_bits(), m2.to_bits());
            let r1 = m.mass_rate_kg_s(t).unwrap();
            let r2 = m.mass_rate_kg_s(t).unwrap();
            assert_eq!(r1.to_bits(), r2.to_bits());
        }
    }

    #[test]
    fn variant_is_solid() {
        let m = trapezoidal_motor();
        assert_eq!(m.variant(), MotorVariant::Solid);
    }

    // -----------------------------------------------------------------
    // ThrustCurve constructor validation
    // -----------------------------------------------------------------

    #[test]
    fn thrust_curve_rejects_fewer_than_two_points() {
        assert!(matches!(
            ThrustCurve::new(vec![]),
            Err(MotorError::MalformedMotor { .. })
        ));
        assert!(matches!(
            ThrustCurve::new(vec![[0.0, 0.0]]),
            Err(MotorError::MalformedMotor { .. })
        ));
    }

    #[test]
    fn thrust_curve_rejects_non_zero_first_time() {
        assert!(matches!(
            ThrustCurve::new(vec![[0.1, 0.0], [1.0, 100.0]]),
            Err(MotorError::MalformedMotor { .. })
        ));
    }

    #[test]
    fn thrust_curve_rejects_non_zero_first_thrust() {
        assert!(matches!(
            ThrustCurve::new(vec![[0.0, 1.0], [1.0, 0.0]]),
            Err(MotorError::MalformedMotor { .. })
        ));
    }

    #[test]
    fn thrust_curve_rejects_non_zero_final_thrust() {
        assert!(matches!(
            ThrustCurve::new(vec![[0.0, 0.0], [1.0, 1.0]]),
            Err(MotorError::MalformedMotor { .. })
        ));
    }

    #[test]
    fn thrust_curve_rejects_non_monotone_time_grid() {
        assert!(matches!(
            ThrustCurve::new(vec![[0.0, 0.0], [1.0, 100.0], [0.5, 50.0]]),
            Err(MotorError::MalformedMotor { .. })
        ));
        assert!(matches!(
            ThrustCurve::new(vec![[0.0, 0.0], [1.0, 100.0], [1.0, 50.0]]),
            Err(MotorError::MalformedMotor { .. })
        ));
    }

    #[test]
    fn thrust_curve_rejects_non_finite_value() {
        assert!(matches!(
            ThrustCurve::new(vec![[0.0, 0.0], [f64::NAN, 100.0]]),
            Err(MotorError::NonFinite { .. })
        ));
        assert!(matches!(
            ThrustCurve::new(vec![[0.0, 0.0], [1.0, f64::INFINITY]]),
            Err(MotorError::NonFinite { .. })
        ));
    }

    #[test]
    fn thrust_curve_rejects_negative_thrust_value() {
        assert!(matches!(
            ThrustCurve::new(vec![[0.0, 0.0], [1.0, -100.0]]),
            Err(MotorError::InvalidParameter { .. })
        ));
    }

    // -----------------------------------------------------------------
    // SolidMotor constructor validation
    // -----------------------------------------------------------------

    fn valid_meta() -> MotorMeta {
        MotorMeta {
            name: "m".into(),
            provenance: "p".into(),
            validation: Validation::ValidatedToy,
        }
    }

    fn valid_burn() -> BurnSpec {
        BurnSpec {
            duration_s: 1.0,
            total_impulse_n_s: 50.0,
            specific_impulse_s: 101.971_621_297_792_84,
            propellant_mass_kg: 0.05,
            dry_mass_kg: 0.05,
        }
    }

    fn valid_curve() -> ThrustCurve {
        ThrustCurve::new(vec![[0.0, 0.0], [0.5, 100.0], [1.0, 0.0]]).unwrap()
    }

    fn valid_geom() -> MotorGeometry {
        MotorGeometry {
            exit_area_m2: 1.0e-4,
            ambient_pressure_correction: AmbientPressureCorrection::Constant,
        }
    }

    #[test]
    fn solid_motor_constructor_accepts_consistent_inputs() {
        SolidMotor::new(valid_meta(), valid_burn(), valid_curve(), valid_geom()).unwrap();
    }

    #[test]
    fn solid_motor_rejects_burn_duration_curve_mismatch() {
        let mut burn = valid_burn();
        burn.duration_s = 2.0; // curve still ends at t = 1.0
        let err = SolidMotor::new(valid_meta(), burn, valid_curve(), valid_geom()).unwrap_err();
        assert!(matches!(err, MotorError::MalformedMotor { .. }));
    }

    #[test]
    fn solid_motor_rejects_non_positive_burn_duration() {
        let mut burn = valid_burn();
        burn.duration_s = 0.0;
        let err = SolidMotor::new(valid_meta(), burn, valid_curve(), valid_geom()).unwrap_err();
        assert!(matches!(err, MotorError::InvalidParameter { .. }));
    }

    #[test]
    fn solid_motor_rejects_non_positive_total_impulse() {
        let mut burn = valid_burn();
        burn.total_impulse_n_s = 0.0;
        let err = SolidMotor::new(valid_meta(), burn, valid_curve(), valid_geom()).unwrap_err();
        assert!(matches!(err, MotorError::InvalidParameter { .. }));
    }

    #[test]
    fn solid_motor_rejects_total_impulse_curve_mismatch() {
        let mut burn = valid_burn();
        burn.total_impulse_n_s = 49.0;
        burn.specific_impulse_s = 49.0 / (burn.propellant_mass_kg * STANDARD_GRAVITY_M_S2);
        let err = SolidMotor::new(valid_meta(), burn, valid_curve(), valid_geom()).unwrap_err();
        assert!(matches!(err, MotorError::InvalidParameter { .. }));
    }

    #[test]
    fn solid_motor_rejects_specific_impulse_inconsistency() {
        let mut burn = valid_burn();
        burn.specific_impulse_s *= 0.5;
        let err = SolidMotor::new(valid_meta(), burn, valid_curve(), valid_geom()).unwrap_err();
        assert!(matches!(err, MotorError::InvalidParameter { .. }));
    }

    #[test]
    fn solid_motor_rejects_negative_propellant_mass() {
        let mut burn = valid_burn();
        burn.propellant_mass_kg = -1.0;
        let err = SolidMotor::new(valid_meta(), burn, valid_curve(), valid_geom()).unwrap_err();
        assert!(matches!(err, MotorError::InvalidParameter { .. }));
    }

    #[test]
    fn solid_motor_rejects_zero_total_mass() {
        let mut burn = valid_burn();
        burn.propellant_mass_kg = 0.0;
        burn.dry_mass_kg = 0.0;
        let err = SolidMotor::new(valid_meta(), burn, valid_curve(), valid_geom()).unwrap_err();
        assert!(matches!(err, MotorError::InvalidParameter { .. }));
    }

    #[test]
    fn solid_motor_rejects_non_positive_exit_area() {
        let mut geom = valid_geom();
        geom.exit_area_m2 = 0.0;
        let err = SolidMotor::new(valid_meta(), valid_burn(), valid_curve(), geom).unwrap_err();
        assert!(matches!(err, MotorError::InvalidParameter { .. }));
    }

    #[test]
    fn solid_motor_rejects_non_finite_burn_value() {
        let mut burn = valid_burn();
        burn.specific_impulse_s = f64::NAN;
        let err = SolidMotor::new(valid_meta(), burn, valid_curve(), valid_geom()).unwrap_err();
        assert!(matches!(err, MotorError::NonFinite { .. }));
    }

    // -----------------------------------------------------------------
    // Mass-flow conservation: ∫ -mass_rate dt = propellant_mass.
    // -----------------------------------------------------------------

    #[test]
    fn mass_loss_at_burnout_equals_propellant_mass() {
        let m = trapezoidal_motor();
        let initial = m.mass_kg(0.0).unwrap();
        let final_mass = m.mass_kg(m.burn_duration_s()).unwrap();
        let consumed = initial - final_mass;
        assert_abs_diff_eq!(consumed, m.propellant_mass_kg(), epsilon = 1.0e-12);
    }
}
