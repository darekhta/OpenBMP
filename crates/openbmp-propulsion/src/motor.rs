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

#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::error::MotorError;

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
const TOTAL_IMPULSE_REL_TOL: f64 = 1.0e-6;
const SPECIFIC_IMPULSE_REL_TOL: f64 = 1.0e-6;
const MIN_POSITIVE: f64 = 1.0e-15;
const NOZZLE_MACH_BISECTION_ITERS: usize = 96;

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

    /// Instantaneous thrust at `t_since_ignition_s`, corrected for
    /// local ambient static pressure in Pa when the motor's nozzle
    /// model opts into it.
    ///
    /// The default implementation preserves the legacy time-only
    /// thrust curve. Implementations with an ambient-aware nozzle
    /// override this method.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError::NonFinite`] when
    /// `ambient_pressure_pa` is non-finite and otherwise returns the
    /// same errors as [`Motor::thrust_n_at`].
    fn thrust_n_at_ambient_pressure(
        &self,
        t_since_ignition_s: f64,
        ambient_pressure_pa: f64,
    ) -> Result<f64, MotorError> {
        if !ambient_pressure_pa.is_finite() || ambient_pressure_pa < 0.0 {
            return Err(MotorError::NonFinite {
                reason: "ambient pressure is NaN, infinite, or negative",
            });
        }
        self.thrust_n_at(t_since_ignition_s)
    }

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
    /// Ambient-aware ideal nozzle: the stored thrust curve is the
    /// momentum-thrust curve at the optimum point (`p_a = p_e`), and
    /// runtime thrust adds `(p_e - p_a) A_e`.
    PressureThrust,
}

/// Opt-in overexpanded-nozzle flow separation criterion.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum NozzleSeparationCriterion {
    /// Disable separation clipping and use the full geometric exit area.
    #[default]
    Off,
    /// Summerfield fixed-ratio criterion, `p_sep / p_a = 0.4`.
    Summerfield,
    /// Schmucker Mach-dependent criterion for free-shock separation.
    Schmucker,
}

/// `[geometry]` block of the motor file.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MotorGeometry {
    /// Nozzle exit area (m²).
    pub exit_area_m2: f64,
    /// Nozzle throat area (m²), required for pressure-thrust
    /// correction and optional for legacy constant correction.
    pub throat_area_m2: Option<f64>,
    /// Nozzle gas specific-heat ratio, required for pressure-thrust
    /// correction and optional for legacy constant correction.
    pub gamma: Option<f64>,
    /// Ambient-pressure correction strategy.
    pub ambient_pressure_correction: AmbientPressureCorrection,
    /// Optional overexpanded-nozzle separation clipping criterion.
    pub separation: NozzleSeparationCriterion,
}

/// Chamber state supplied to a nozzle-performance model.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ChamberState {
    /// Chamber pressure in Pa.
    pub chamber_pressure_pa: f64,
    /// Exhaust mass flow in kg/s.
    pub mass_flow_kg_s: f64,
    /// Gas specific-heat ratio.
    pub gamma: f64,
    /// Nozzle throat area in m².
    pub throat_area_m2: f64,
    /// Nozzle exit area in m².
    pub exit_area_m2: f64,
}

/// Ideal-nozzle solution at one chamber state and ambient pressure.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NozzleSolution {
    /// Exit Mach number on the supersonic branch.
    pub exit_mach: f64,
    /// Exit static pressure in Pa.
    pub exit_pressure_pa: f64,
    /// Whether an overexpanded-flow separation criterion clipped the nozzle.
    pub separated: bool,
    /// Effective expansion ratio used for thrust calculation.
    pub effective_expansion_ratio: f64,
    /// Effective exit area used for pressure thrust, in m².
    pub effective_exit_area_m2: f64,
    /// Momentum thrust `mdot * Ve`, in N.
    pub momentum_thrust_n: f64,
    /// Pressure thrust `(pe - pa) * Ae`, in N.
    pub pressure_thrust_n: f64,
    /// Total thrust in N.
    pub total_thrust_n: f64,
    /// Effective specific impulse at this ambient pressure, in s.
    pub effective_isp_s: f64,
}

/// Deterministic nozzle-performance surface.
pub trait NozzlePerformance {
    /// Solve the nozzle state at a chamber condition and ambient pressure.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] when the chamber/nozzle state is outside
    /// the ideal-nozzle envelope or produces non-finite output.
    fn solve(
        &self,
        chamber: ChamberState,
        ambient_pressure_pa: f64,
    ) -> Result<NozzleSolution, MotorError>;
}

/// Isentropic ideal-nozzle performance model.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct IdealNozzlePerformance {
    /// Separation criterion used by the solve.
    pub separation: NozzleSeparationCriterion,
}

impl IdealNozzlePerformance {
    /// Construct an ideal-nozzle solver with the requested separation mode.
    #[must_use]
    pub const fn new(separation: NozzleSeparationCriterion) -> Self {
        Self { separation }
    }
}

impl NozzlePerformance for IdealNozzlePerformance {
    fn solve(
        &self,
        chamber: ChamberState,
        ambient_pressure_pa: f64,
    ) -> Result<NozzleSolution, MotorError> {
        validate_chamber_state(chamber)?;
        if !ambient_pressure_pa.is_finite() || ambient_pressure_pa < 0.0 {
            return Err(MotorError::NonFinite {
                reason: "ambient pressure is NaN, infinite, or negative",
            });
        }
        let geometric_expansion_ratio = chamber.exit_area_m2 / chamber.throat_area_m2;
        let geometric_exit_mach =
            supersonic_mach_for_area_ratio(chamber.gamma, geometric_expansion_ratio)?;
        let geometric_exit_pressure_pa =
            chamber.chamber_pressure_pa * exit_pressure_ratio(chamber.gamma, geometric_exit_mach);
        let (
            separated,
            exit_mach,
            exit_pressure_pa,
            effective_expansion_ratio,
            effective_exit_area_m2,
        ) = effective_nozzle_state(
            self.separation,
            chamber,
            ambient_pressure_pa,
            geometric_exit_mach,
            geometric_exit_pressure_pa,
        )?;
        let cf_momentum =
            ideal_momentum_thrust_coefficient(chamber.gamma, effective_expansion_ratio)?;
        let momentum_thrust_n = cf_momentum * chamber.throat_area_m2 * chamber.chamber_pressure_pa;
        let pressure_thrust_n = (exit_pressure_pa - ambient_pressure_pa) * effective_exit_area_m2;
        let total_thrust_n = momentum_thrust_n + pressure_thrust_n;
        let effective_isp_s = if chamber.mass_flow_kg_s > 0.0 {
            total_thrust_n / (STANDARD_GRAVITY_M_S2 * chamber.mass_flow_kg_s)
        } else {
            0.0
        };
        for value in [
            exit_mach,
            exit_pressure_pa,
            momentum_thrust_n,
            pressure_thrust_n,
            total_thrust_n,
            effective_isp_s,
            effective_expansion_ratio,
            effective_exit_area_m2,
        ] {
            if !value.is_finite() {
                return Err(MotorError::NonFinite {
                    reason: "nozzle solution is non-finite",
                });
            }
        }
        Ok(NozzleSolution {
            exit_mach,
            exit_pressure_pa,
            separated,
            effective_expansion_ratio,
            effective_exit_area_m2,
            momentum_thrust_n,
            pressure_thrust_n,
            total_thrust_n,
            effective_isp_s,
        })
    }
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

    /// Return a copy with a different ambient-pressure correction
    /// strategy.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] when the requested correction requires
    /// nozzle geometry the motor does not carry.
    pub fn with_ambient_pressure_correction(
        mut self,
        correction: AmbientPressureCorrection,
    ) -> Result<Self, MotorError> {
        self.geometry.ambient_pressure_correction = correction;
        validate_geometry(self.geometry)?;
        Ok(self)
    }

    /// Return a copy with a different overexpanded-nozzle separation mode.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] when separation is enabled without the
    /// pressure-thrust nozzle geometry needed to solve it.
    pub fn with_nozzle_separation(
        mut self,
        separation: NozzleSeparationCriterion,
    ) -> Result<Self, MotorError> {
        self.geometry.separation = separation;
        validate_geometry(self.geometry)?;
        Ok(self)
    }

    /// Solve the ambient-aware nozzle at one motor time.
    ///
    /// Returns `Ok(None)` outside the burn window or at zero thrust.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] when the motor does not carry the
    /// nozzle parameters required for pressure-thrust correction or
    /// when the ideal-nozzle solve fails.
    pub fn nozzle_solution_at(
        &self,
        t_since_ignition_s: f64,
        ambient_pressure_pa: f64,
    ) -> Result<Option<NozzleSolution>, MotorError> {
        let Some(chamber) = self.chamber_state_at(t_since_ignition_s)? else {
            return Ok(None);
        };
        IdealNozzlePerformance::new(self.geometry.separation)
            .solve(chamber, ambient_pressure_pa)
            .map(Some)
    }

    /// Reconstruct the ideal chamber state implied by the stored
    /// momentum-thrust curve at one motor time.
    ///
    /// Returns `Ok(None)` outside the burn window or at zero thrust.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] when the motor does not carry the throat area
    /// and gas specific-heat ratio required for ideal-nozzle reconstruction.
    pub fn chamber_state_at(
        &self,
        t_since_ignition_s: f64,
    ) -> Result<Option<ChamberState>, MotorError> {
        let momentum_thrust_n = self.thrust_n_at(t_since_ignition_s)?;
        if momentum_thrust_n == 0.0 {
            return Ok(None);
        }
        self.chamber_state_from_momentum_thrust(
            momentum_thrust_n,
            -self.mass_rate_kg_s(t_since_ignition_s)?,
        )
        .map(Some)
    }

    fn chamber_state_from_momentum_thrust(
        &self,
        momentum_thrust_n: f64,
        mass_flow_kg_s: f64,
    ) -> Result<ChamberState, MotorError> {
        let throat_area_m2 = self
            .geometry
            .throat_area_m2
            .ok_or(MotorError::InvalidParameter {
                reason: "pressure-thrust correction requires geometry.throat_area_m2",
            })?;
        let gamma = self.geometry.gamma.ok_or(MotorError::InvalidParameter {
            reason: "pressure-thrust correction requires geometry.gamma",
        })?;
        let expansion_ratio = self.geometry.exit_area_m2 / throat_area_m2;
        let cf_momentum = ideal_momentum_thrust_coefficient(gamma, expansion_ratio)?;
        let chamber_pressure_pa = momentum_thrust_n / (cf_momentum * throat_area_m2);
        let chamber = ChamberState {
            chamber_pressure_pa,
            mass_flow_kg_s,
            gamma,
            throat_area_m2,
            exit_area_m2: self.geometry.exit_area_m2,
        };
        validate_chamber_state(chamber)?;
        Ok(chamber)
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

    fn thrust_n_at_ambient_pressure(
        &self,
        t_since_ignition_s: f64,
        ambient_pressure_pa: f64,
    ) -> Result<f64, MotorError> {
        if !ambient_pressure_pa.is_finite() || ambient_pressure_pa < 0.0 {
            return Err(MotorError::NonFinite {
                reason: "ambient pressure is NaN, infinite, or negative",
            });
        }
        match self.geometry.ambient_pressure_correction {
            AmbientPressureCorrection::Constant => self.thrust_n_at(t_since_ignition_s),
            AmbientPressureCorrection::PressureThrust => Ok(self
                .nozzle_solution_at(t_since_ignition_s, ambient_pressure_pa)?
                .map_or(0.0, |solution| solution.total_thrust_n)),
        }
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
    if let Some(throat_area_m2) = geometry.throat_area_m2 {
        if !throat_area_m2.is_finite() || throat_area_m2 <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "throat area must be finite and positive",
            });
        }
        if throat_area_m2 > geometry.exit_area_m2 {
            return Err(MotorError::InvalidParameter {
                reason: "throat area must not exceed exit area",
            });
        }
    }
    if let Some(gamma) = geometry.gamma
        && (!gamma.is_finite() || gamma <= 1.0)
    {
        return Err(MotorError::InvalidParameter {
            reason: "nozzle gamma must be finite and greater than 1",
        });
    }
    if matches!(
        geometry.ambient_pressure_correction,
        AmbientPressureCorrection::PressureThrust
    ) && (geometry.throat_area_m2.is_none() || geometry.gamma.is_none())
    {
        return Err(MotorError::InvalidParameter {
            reason: "pressure-thrust correction requires throat_area_m2 and gamma",
        });
    }
    if geometry.separation != NozzleSeparationCriterion::Off
        && geometry.ambient_pressure_correction != AmbientPressureCorrection::PressureThrust
    {
        return Err(MotorError::InvalidParameter {
            reason: "nozzle separation requires pressure-thrust correction",
        });
    }
    Ok(())
}

fn validate_chamber_state(chamber: ChamberState) -> Result<(), MotorError> {
    for value in [
        chamber.chamber_pressure_pa,
        chamber.mass_flow_kg_s,
        chamber.gamma,
        chamber.throat_area_m2,
        chamber.exit_area_m2,
    ] {
        if !value.is_finite() {
            return Err(MotorError::NonFinite {
                reason: "chamber state is non-finite",
            });
        }
    }
    if chamber.chamber_pressure_pa <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "chamber pressure must be positive",
        });
    }
    if chamber.mass_flow_kg_s < 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "mass flow must be non-negative",
        });
    }
    if chamber.gamma <= 1.0 {
        return Err(MotorError::InvalidParameter {
            reason: "nozzle gamma must be greater than 1",
        });
    }
    if chamber.throat_area_m2 <= 0.0 || chamber.exit_area_m2 < chamber.throat_area_m2 {
        return Err(MotorError::InvalidParameter {
            reason: "nozzle areas must satisfy exit_area >= throat_area > 0",
        });
    }
    Ok(())
}

pub(crate) fn ideal_momentum_thrust_coefficient(
    gamma: f64,
    expansion_ratio: f64,
) -> Result<f64, MotorError> {
    if !gamma.is_finite() || gamma <= 1.0 || !expansion_ratio.is_finite() || expansion_ratio < 1.0 {
        return Err(MotorError::InvalidParameter {
            reason: "nozzle gamma and expansion_ratio are outside envelope",
        });
    }
    let exit_mach = supersonic_mach_for_area_ratio(gamma, expansion_ratio)?;
    let pe_pc = exit_pressure_ratio(gamma, exit_mach);
    let term = 1.0 - pe_pc.powf((gamma - 1.0) / gamma);
    let coeff = ((2.0 * gamma * gamma / (gamma - 1.0))
        * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (gamma - 1.0))
        * term)
        .sqrt();
    if !coeff.is_finite() || coeff <= 0.0 {
        return Err(MotorError::NonFinite {
            reason: "nozzle thrust coefficient is non-finite",
        });
    }
    Ok(coeff)
}

pub(crate) fn exit_pressure_ratio(gamma: f64, exit_mach: f64) -> f64 {
    (1.0 + 0.5 * (gamma - 1.0) * exit_mach * exit_mach).powf(-gamma / (gamma - 1.0))
}

fn effective_nozzle_state(
    separation: NozzleSeparationCriterion,
    chamber: ChamberState,
    ambient_pressure_pa: f64,
    geometric_exit_mach: f64,
    geometric_exit_pressure_pa: f64,
) -> Result<(bool, f64, f64, f64, f64), MotorError> {
    let geometric_expansion_ratio = chamber.exit_area_m2 / chamber.throat_area_m2;
    if separation == NozzleSeparationCriterion::Off || ambient_pressure_pa == 0.0 {
        return Ok((
            false,
            geometric_exit_mach,
            geometric_exit_pressure_pa,
            geometric_expansion_ratio,
            chamber.exit_area_m2,
        ));
    }

    let full_residual = separation_residual(
        separation,
        chamber.gamma,
        chamber.chamber_pressure_pa,
        ambient_pressure_pa,
        geometric_exit_mach,
    )?;
    if full_residual >= 0.0 {
        return Ok((
            false,
            geometric_exit_mach,
            geometric_exit_pressure_pa,
            geometric_expansion_ratio,
            chamber.exit_area_m2,
        ));
    }

    let mut lo = 1.0;
    let mut hi = geometric_exit_mach;
    if separation_residual(
        separation,
        chamber.gamma,
        chamber.chamber_pressure_pa,
        ambient_pressure_pa,
        lo,
    )? <= 0.0
    {
        hi = lo;
    } else {
        for _ in 0..NOZZLE_MACH_BISECTION_ITERS {
            let mid = 0.5 * (lo + hi);
            if separation_residual(
                separation,
                chamber.gamma,
                chamber.chamber_pressure_pa,
                ambient_pressure_pa,
                mid,
            )? > 0.0
            {
                lo = mid;
            } else {
                hi = mid;
            }
        }
    }

    let exit_mach = hi;
    let exit_pressure_pa =
        chamber.chamber_pressure_pa * exit_pressure_ratio(chamber.gamma, exit_mach);
    let effective_expansion_ratio = nozzle_area_ratio(chamber.gamma, exit_mach);
    let effective_exit_area_m2 = chamber.throat_area_m2 * effective_expansion_ratio;
    Ok((
        true,
        exit_mach,
        exit_pressure_pa,
        effective_expansion_ratio,
        effective_exit_area_m2,
    ))
}

fn separation_residual(
    separation: NozzleSeparationCriterion,
    gamma: f64,
    chamber_pressure_pa: f64,
    ambient_pressure_pa: f64,
    mach: f64,
) -> Result<f64, MotorError> {
    let Some(separation_ratio) = separation_pressure_ratio(separation, mach) else {
        return Err(MotorError::OutOfEnvelope {
            reason: "nozzle separation criterion is outside the fixed Mach bracket",
        });
    };
    let residual = chamber_pressure_pa * exit_pressure_ratio(gamma, mach)
        - ambient_pressure_pa * separation_ratio;
    if !residual.is_finite() {
        return Err(MotorError::NonFinite {
            reason: "nozzle separation residual is non-finite",
        });
    }
    Ok(residual)
}

fn separation_pressure_ratio(separation: NozzleSeparationCriterion, mach: f64) -> Option<f64> {
    let ratio = match separation {
        NozzleSeparationCriterion::Off => return None,
        NozzleSeparationCriterion::Summerfield => 0.4,
        NozzleSeparationCriterion::Schmucker => {
            let base = 1.88 * mach - 1.0;
            if base <= 0.0 {
                return None;
            }
            base.powf(-0.64)
        }
    };
    ratio.is_finite().then_some(ratio)
}

pub(crate) fn supersonic_mach_for_area_ratio(
    gamma: f64,
    expansion_ratio: f64,
) -> Result<f64, MotorError> {
    if (expansion_ratio - 1.0).abs() <= MIN_POSITIVE {
        return Ok(1.0);
    }
    let mut lo = 1.0;
    let mut hi = 50.0;
    if nozzle_area_ratio(gamma, hi) < expansion_ratio {
        return Err(MotorError::OutOfEnvelope {
            reason: "nozzle expansion_ratio is outside the fixed Mach bracket",
        });
    }
    for _ in 0..NOZZLE_MACH_BISECTION_ITERS {
        let mid = 0.5 * (lo + hi);
        if nozzle_area_ratio(gamma, mid) < expansion_ratio {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Ok(0.5 * (lo + hi))
}

fn nozzle_area_ratio(gamma: f64, mach: f64) -> f64 {
    let gm1 = gamma - 1.0;
    let gp1 = gamma + 1.0;
    let bracket = (2.0 / gp1) * (1.0 + 0.5 * gm1 * mach * mach);
    (1.0 / mach) * bracket.powf(gp1 / (2.0 * gm1))
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
            throat_area_m2: None,
            gamma: None,
            ambient_pressure_correction: AmbientPressureCorrection::Constant,
            separation: NozzleSeparationCriterion::Off,
        };
        SolidMotor::new(meta, burn, curve, geom).unwrap()
    }

    fn pressure_thrust_motor() -> SolidMotor {
        let mut motor = trapezoidal_motor();
        motor.geometry = MotorGeometry {
            exit_area_m2: 2.0e-3,
            throat_area_m2: Some(1.0e-4),
            gamma: Some(1.2),
            ambient_pressure_correction: AmbientPressureCorrection::PressureThrust,
            separation: NozzleSeparationCriterion::Off,
        };
        SolidMotor::new(
            motor.meta().clone(),
            *motor.burn(),
            motor.thrust_curve().clone(),
            motor.geometry,
        )
        .unwrap()
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

    #[test]
    fn pressure_thrust_requires_nozzle_state() {
        assert!(matches!(
            trapezoidal_motor()
                .with_ambient_pressure_correction(AmbientPressureCorrection::PressureThrust),
            Err(MotorError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn pressure_thrust_matches_optimum_curve_when_ambient_equals_exit_pressure() {
        let m = pressure_thrust_motor();
        let optimum = m.thrust_n_at(2.0).unwrap();
        let solution_at_vacuum = m.nozzle_solution_at(2.0, 0.0).unwrap().unwrap();
        let corrected = m
            .thrust_n_at_ambient_pressure(2.0, solution_at_vacuum.exit_pressure_pa)
            .unwrap();
        assert_abs_diff_eq!(corrected, optimum, epsilon = 1.0e-12);
    }

    #[test]
    fn chamber_state_at_exposes_pressure_thrust_nozzle_inputs() {
        let m = pressure_thrust_motor();
        let chamber = m.chamber_state_at(2.0).unwrap().unwrap();
        assert!(chamber.chamber_pressure_pa > 0.0);
        assert!(chamber.mass_flow_kg_s > 0.0);
        assert_eq!(chamber.gamma.to_bits(), 1.2_f64.to_bits());
        assert_eq!(chamber.throat_area_m2.to_bits(), 1.0e-4_f64.to_bits());
        assert_eq!(chamber.exit_area_m2.to_bits(), 2.0e-3_f64.to_bits());
        assert!(m.chamber_state_at(4.5).unwrap().is_none());
    }

    #[test]
    fn pressure_thrust_lifts_vacuum_and_reduces_sea_level_by_area_term() {
        let m = pressure_thrust_motor();
        let optimum = m.thrust_n_at(2.0).unwrap();
        let solution_at_vacuum = m.nozzle_solution_at(2.0, 0.0).unwrap().unwrap();
        let sea_level = m.thrust_n_at_ambient_pressure(2.0, 101_325.0).unwrap();
        let vacuum = m.thrust_n_at_ambient_pressure(2.0, 0.0).unwrap();
        assert!(vacuum > optimum);
        assert!(sea_level < optimum);
        assert_abs_diff_eq!(
            vacuum - sea_level,
            101_325.0 * m.geometry().exit_area_m2,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            vacuum - optimum,
            solution_at_vacuum.exit_pressure_pa * m.geometry().exit_area_m2,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn separation_off_preserves_ideal_pressure_thrust_solution() {
        let m = pressure_thrust_motor();
        let chamber = m
            .chamber_state_from_momentum_thrust(1000.0, -m.mass_rate_kg_s(2.0).unwrap())
            .unwrap();
        let off = IdealNozzlePerformance::default()
            .solve(chamber, 101_325.0)
            .unwrap();
        let explicit_off = IdealNozzlePerformance::new(NozzleSeparationCriterion::Off)
            .solve(chamber, 101_325.0)
            .unwrap();

        assert_eq!(off, explicit_off);
        assert!(!off.separated);
        assert_abs_diff_eq!(
            off.effective_expansion_ratio,
            m.geometry().exit_area_m2 / m.geometry().throat_area_m2.unwrap(),
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            off.effective_exit_area_m2,
            m.geometry().exit_area_m2,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn summerfield_separation_clips_heavily_overexpanded_nozzle() {
        let baseline = pressure_thrust_motor();
        let separated_motor = baseline
            .clone()
            .with_nozzle_separation(NozzleSeparationCriterion::Summerfield)
            .unwrap();

        let full = baseline
            .nozzle_solution_at(2.0, 101_325.0)
            .unwrap()
            .unwrap();
        let separated = separated_motor
            .nozzle_solution_at(2.0, 101_325.0)
            .unwrap()
            .unwrap();

        assert!(!full.separated);
        assert!(separated.separated);
        assert!(separated.effective_expansion_ratio < full.effective_expansion_ratio);
        assert!(separated.effective_exit_area_m2 < full.effective_exit_area_m2);
        assert!(separated.total_thrust_n > full.total_thrust_n);
        assert_abs_diff_eq!(
            separated.exit_pressure_pa / 101_325.0,
            0.4,
            epsilon = 1.0e-10
        );
    }

    #[test]
    fn schmucker_separation_clips_heavily_overexpanded_nozzle() {
        let baseline = pressure_thrust_motor();
        let separated_motor = baseline
            .clone()
            .with_nozzle_separation(NozzleSeparationCriterion::Schmucker)
            .unwrap();

        let full = baseline
            .nozzle_solution_at(2.0, 101_325.0)
            .unwrap()
            .unwrap();
        let separated = separated_motor
            .nozzle_solution_at(2.0, 101_325.0)
            .unwrap()
            .unwrap();
        let expected_ratio =
            separation_pressure_ratio(NozzleSeparationCriterion::Schmucker, separated.exit_mach)
                .unwrap();

        assert!(separated.separated);
        assert!(separated.effective_expansion_ratio < full.effective_expansion_ratio);
        assert!(separated.total_thrust_n > full.total_thrust_n);
        assert_abs_diff_eq!(
            separated.exit_pressure_pa / 101_325.0,
            expected_ratio,
            epsilon = 1.0e-10
        );
    }

    #[test]
    fn nozzle_separation_requires_pressure_thrust_correction() {
        assert!(matches!(
            trapezoidal_motor().with_nozzle_separation(NozzleSeparationCriterion::Summerfield),
            Err(MotorError::InvalidParameter { .. })
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
            throat_area_m2: None,
            gamma: None,
            ambient_pressure_correction: AmbientPressureCorrection::Constant,
            separation: NozzleSeparationCriterion::Off,
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
