//! Phase-3.4 control effectors.
//!
//! Effectors are the controller-physics interface: a scalar `cmd`
//! flows in (from a controller, a scenario-declared schedule, or a
//! scenario-script effector override event), and a [`EffectorState`]
//! comes out carrying the actual deflection, saturation flag,
//! rate-limit flag, and active fault. Phase 3.4 ships:
//!
//! - [`ControlEffector`] trait — `step(cmd, dt)`, `limits()`,
//!   `inject_fault(fault)`, `current_state()`, `id()`.
//! - [`EffectorLimits`] — position min/max, max rate, deadband,
//!   pure-delay latency.
//! - [`EffectorState`] — per-step output snapshot.
//! - [`EffectorFault`] — four canonical fault modes (`Jam`,
//!   `Runaway`, `ReducedRate`, `Hardover`).
//! - [`linear::LinearActuator`] — first-order linear actuator with
//!   rate clamp + position saturation + deadband + fixed-depth
//!   circular-buffer pure delay.
//!
//! # Determinism
//!
//! - The `LinearActuator` impl is fully deterministic in 3.4 — no
//!   RNG draws. The future stochastic-fault story will pull from
//!   [`openbmp_core::DeterministicRng::for_effector_component`]
//!   with the pinned `b"EFFC"` domain tag.
//! - The trait is scalar-only (not generic over `S: SimState`):
//!   effectors operate on `f64` commands, deflections, and rates.
//!   This stays orthogonal to the kernel's `S`-axis.
//! - The pure-delay buffer is sized at construction from a fixed
//!   `dt`. Sub-`dt` latency is rejected at construction.
//!
//! See `docs/scenario-format.md § Control effectors (Phase 3.4)` and
//! `docs/software-architecture.md § ControlEffectors` for the
//! contract.

pub mod linear;

pub use linear::LinearActuator;

use openbmp_core::{Duration, EffectorId};
use thiserror::Error;

/// Position / rate / latency limits for a [`ControlEffector`].
///
/// Validated finite at construction. Limit types are scalars with
/// the `_per_s` suffix carrying the SI rate convention; the
/// `latency` is a [`Duration`] so it composes with
/// [`openbmp_core::SimTime`] / [`openbmp_core::StepIndex`]
/// arithmetic without unit drift.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EffectorLimits {
    /// Minimum permitted deflection (rad, m, or whatever the
    /// effector axis represents).
    pub min: f64,
    /// Maximum permitted deflection. Must be `> min`.
    pub max: f64,
    /// Maximum slew rate magnitude (units / s). Strictly positive.
    pub max_rate_per_s: f64,
    /// Deadband: commands within `actual ± deadband` of the current
    /// deflection are held. Must be non-negative and `≤ (max - min)`.
    pub deadband: f64,
    /// Pure-delay latency. Must be either zero or at least `dt`
    /// (sub-`dt` latency is rejected at construction).
    pub latency: Duration,
}

impl EffectorLimits {
    /// Validate the limits against the configured kernel `dt`.
    ///
    /// # Errors
    ///
    /// Returns [`EffectorError`] for non-finite values, `min ≥ max`,
    /// `max_rate_per_s ≤ 0`, negative `deadband`, `deadband > (max -
    /// min)`, negative `latency`, or `0 < latency < dt`.
    pub fn require_valid(&self, dt: Duration) -> Result<(), EffectorError> {
        if !self.min.is_finite() || !self.max.is_finite() {
            return Err(EffectorError::InvalidLimits {
                reason: "min and max must be finite",
            });
        }
        if self.min >= self.max {
            return Err(EffectorError::InvalidLimits {
                reason: "min must be strictly less than max",
            });
        }
        if !self.max_rate_per_s.is_finite() || self.max_rate_per_s <= 0.0 {
            return Err(EffectorError::InvalidLimits {
                reason: "max_rate_per_s must be finite and strictly positive",
            });
        }
        if !self.deadband.is_finite() || self.deadband < 0.0 {
            return Err(EffectorError::InvalidLimits {
                reason: "deadband must be finite and non-negative",
            });
        }
        if self.deadband > (self.max - self.min) {
            return Err(EffectorError::InvalidLimits {
                reason: "deadband must be at most (max - min)",
            });
        }
        let latency_s = self.latency.as_seconds();
        let dt_s = dt.as_seconds();
        if !latency_s.is_finite() || latency_s < 0.0 {
            return Err(EffectorError::InvalidLimits {
                reason: "latency must be finite and non-negative",
            });
        }
        if latency_s > 0.0 && latency_s + 1.0e-12 < dt_s {
            return Err(EffectorError::SubStepLatency);
        }
        Ok(())
    }
}

/// Per-step output snapshot from a [`ControlEffector`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EffectorState {
    /// Command observed this step (post-fault, pre-clamp).
    pub commanded: f64,
    /// Actual deflection committed this step.
    pub actual: f64,
    /// `true` if the position clamp fired this step.
    pub saturated: bool,
    /// `true` if the rate clamp fired this step.
    pub rate_limited: bool,
    /// Active fault, if any.
    pub fault: Option<EffectorFault>,
}

impl EffectorState {
    /// Construct the at-rest state at `position` with no flags set.
    #[must_use]
    pub const fn at_rest(position: f64) -> Self {
        Self {
            commanded: position,
            actual: position,
            saturated: false,
            rate_limited: false,
            fault: None,
        }
    }
}

/// Canonical fault modes drawn from Patton, Frank & Clark 1989
/// *Fault Diagnosis in Dynamic Systems*.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EffectorFault {
    /// Effector locked at `at`, ignoring all subsequent commands.
    Jam {
        /// Locked position.
        at: f64,
    },
    /// Effector slews at the specified rate regardless of command.
    Runaway {
        /// Signed slew rate (units / s).
        rate_per_s: f64,
    },
    /// Effector tracks the command but at a reduced max rate.
    /// `factor ∈ [0, 1]` scales `max_rate_per_s`.
    ReducedRate {
        /// Multiplier on the max rate. Clamped to `[0, 1]` at the
        /// construction validator.
        factor: f64,
    },
    /// Effector steps to `to` (saturation-clamped) and then jams
    /// there for the remainder of the run.
    Hardover {
        /// Target deflection.
        to: f64,
    },
}

/// Errors produced by effector construction or step evaluation.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum EffectorError {
    /// One or more [`EffectorLimits`] components are invalid.
    #[error("effector limits invalid: {reason}")]
    InvalidLimits {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// Latency is non-zero but smaller than the kernel `dt`. The
    /// fixed-step pure-delay buffer requires `latency ≥ dt`.
    #[error("effector latency must be either zero or ≥ dt for fixed-step pure delay")]
    SubStepLatency,
    /// `step()` was called with a `dt` that disagrees with the
    /// effector's construction-time `dt`. Phase 3 uses fixed-step
    /// integration; mismatched `dt` is a programmer error.
    #[error("effector step dt mismatch: configured {configured_s} s, got {got_s} s")]
    DtMismatch {
        /// `dt` configured at construction.
        configured_s: f64,
        /// `dt` passed to `step`.
        got_s: f64,
    },
    /// `step()` received a non-finite command.
    #[error("effector command is not finite: {value}")]
    NonFiniteCommand {
        /// Offending command.
        value: f64,
    },
    /// A fault payload is invalid (e.g. `Jam{at}` outside `[min, max]`,
    /// `ReducedRate{factor}` outside `[0, 1]`, `Runaway{rate}` not
    /// finite).
    #[error("effector fault invalid: {reason}")]
    InvalidFault {
        /// Human-readable reason.
        reason: &'static str,
    },
}

/// Phase-3.4 control-effector trait.
///
/// Effectors are stateful (latency buffer, integrator state for
/// the first-order lag, current `EffectorState` snapshot, fault
/// mode) and therefore take `&mut self` on `step`. The trait is
/// scalar-only — effectors operate on `f64` commands and produce
/// `f64` deflections.
pub trait ControlEffector: std::fmt::Debug + Send + Sync {
    /// Stable identifier for telemetry and event-action targeting.
    fn id(&self) -> EffectorId;

    /// Advance the effector by one kernel base tick.
    ///
    /// `cmd` is the controller-or-scenario-declared command.
    /// `dt` must equal the effector's construction-time `dt` within
    /// 1e-12.
    ///
    /// # Errors
    ///
    /// Returns [`EffectorError::DtMismatch`] when `dt` differs from
    /// the construction-time `dt`, or
    /// [`EffectorError::NonFiniteCommand`] when `cmd` is not finite.
    fn step(&mut self, cmd: f64, dt: Duration) -> Result<EffectorState, EffectorError>;

    /// Return the configured limits.
    fn limits(&self) -> EffectorLimits;

    /// Inject a fault. Replaces any prior fault after validating the
    /// fault payload against this effector's limits.
    ///
    /// # Errors
    ///
    /// Returns [`EffectorError::InvalidFault`] when the fault payload
    /// is non-finite or outside the effector's authority envelope.
    fn inject_fault(&mut self, fault: EffectorFault) -> Result<(), EffectorError>;

    /// Return the most recent [`EffectorState`] snapshot. Useful for
    /// telemetry without re-stepping.
    fn current_state(&self) -> EffectorState;
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn limits_reject_min_geq_max() {
        let limits = EffectorLimits {
            min: 1.0,
            max: 1.0,
            max_rate_per_s: 1.0,
            deadband: 0.0,
            latency: Duration::from_seconds(0.0),
        };
        assert!(matches!(
            limits.require_valid(Duration::from_seconds(0.001)),
            Err(EffectorError::InvalidLimits { .. })
        ));
    }

    #[test]
    fn limits_reject_non_positive_max_rate() {
        let limits = EffectorLimits {
            min: -1.0,
            max: 1.0,
            max_rate_per_s: 0.0,
            deadband: 0.0,
            latency: Duration::from_seconds(0.0),
        };
        assert!(matches!(
            limits.require_valid(Duration::from_seconds(0.001)),
            Err(EffectorError::InvalidLimits { .. })
        ));
    }

    #[test]
    fn limits_reject_oversized_deadband() {
        let limits = EffectorLimits {
            min: -1.0,
            max: 1.0,
            max_rate_per_s: 1.0,
            deadband: 3.0,
            latency: Duration::from_seconds(0.0),
        };
        assert!(matches!(
            limits.require_valid(Duration::from_seconds(0.001)),
            Err(EffectorError::InvalidLimits { .. })
        ));
    }

    #[test]
    fn limits_reject_sub_dt_latency() {
        let limits = EffectorLimits {
            min: -1.0,
            max: 1.0,
            max_rate_per_s: 1.0,
            deadband: 0.0,
            latency: Duration::from_seconds(0.0001),
        };
        assert!(matches!(
            limits.require_valid(Duration::from_seconds(0.001)),
            Err(EffectorError::SubStepLatency)
        ));
    }

    #[test]
    fn limits_accept_zero_latency() {
        let limits = EffectorLimits {
            min: -1.0,
            max: 1.0,
            max_rate_per_s: 1.0,
            deadband: 0.0,
            latency: Duration::from_seconds(0.0),
        };
        assert!(limits.require_valid(Duration::from_seconds(0.001)).is_ok());
    }

    #[test]
    fn limits_accept_exact_dt_latency() {
        let dt = Duration::from_seconds(0.001);
        let limits = EffectorLimits {
            min: -1.0,
            max: 1.0,
            max_rate_per_s: 1.0,
            deadband: 0.0,
            latency: dt,
        };
        assert!(limits.require_valid(dt).is_ok());
    }

    #[test]
    fn effector_state_at_rest() {
        let state = EffectorState::at_rest(0.5);
        assert!((state.actual - 0.5).abs() < 1e-12);
        assert!((state.commanded - 0.5).abs() < 1e-12);
        assert!(!state.saturated);
        assert!(!state.rate_limited);
        assert!(state.fault.is_none());
    }
}
