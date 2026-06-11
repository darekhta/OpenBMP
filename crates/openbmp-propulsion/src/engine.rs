//! Engine models.
//!
//! Engines are the controller-physics interface for liquid (and
//! later hybrid / cold-gas) propulsion. The trait surface
//! exposes per-engine throttle, gimbal, and ignition / shutdown
//! lifecycle commands; per-step thrust + mass-flow snapshots come
//! out. Provides:
//!
//! - [`EngineModel`] trait — `apply_command(cmd)`, `step(dt)`,
//!   `limits()`, `inject_fault(fault)`, `current_state()`,
//!   `current_snapshot()`, `id()`, `validation()`.
//! - [`EngineCommand`] — controller-or-mission-event command
//!   payload: throttle ∈ [0, 1], gimbal pitch / yaw, ignite /
//!   shutdown booleans.
//! - [`EngineState`] enum — `Idle`, `Igniting`, `Burning`,
//!   `Shutdown`, `Failed`. Lifecycle is one-shot: `Shutdown` is
//!   terminal for nominal operation; `Failed` is terminal for
//!   load-time or scheduled runtime faults.
//! - [`EngineSnapshot`] — per-step output: gimbal-applied
//!   body-frame thrust vector, mass-flow rate, integrated
//!   propellant consumption, current state.
//! - [`EngineFault`] — canonical fault modes (`Stuck`, `HardOff`,
//!   `OverThrust`, `HardStartOverpressure`, `CavitationThrustLoss`,
//!   `GimbalLocked`).
//! - [`LiquidEngine`] — reference impl: linear ignition transient
//!   (0 → commanded thrust over `ignition_transient_s`), rate-limited
//!   throttle tracking in burn, linear shutdown transient (current → 0
//!   over `shutdown_transient_s`). Mass flow is
//!   `thrust / (g0 · effective_isp)`.
//!   Gimbal applied as a locked-order pitch-then-yaw rotation
//!   around the engine's nominal +z axis.
//!
//! # Determinism
//!
//! - Pure arithmetic on `f64`; locked operand order on the gimbal
//!   rotation; no FMA; no wall-clock; no system RNG; no I/O on the
//!   hot path.
//! - All fault modes are deterministic.
//! - The state machine advances deterministically based on
//!   `elapsed_in_state_s`, accumulated step-by-step from `dt`.
//!
//! # Crate layering
//!
//! `EngineModel` lives in `openbmp-propulsion` (L2) — no kernel
//! dependency. The kernel-side adapter trio (`EngineClusterForceAdapter`
//! et al.) lives in `openbmp-vehicle` and consumes a per-step
//! `EngineSnapshot` map via the kernel's `EngineSnapshotView` (the
//! runner-snapshot path).
//!
//! See `docs/software-architecture.md § Propulsion: EngineModel and
//! EngineCluster` and `docs/scenario-format.md § Engine clusters`
//! for the contract.

use core::fmt::Debug;

use nalgebra::Vector3;
#[cfg(not(feature = "std"))]
use num_traits::Float;
use openbmp_core::{Duration, EngineId, ValidationStatus};

use crate::error::EngineError;
use crate::motor::{
    ChamberState, IdealNozzlePerformance, NozzlePerformance, NozzleSeparationCriterion,
    NozzleSolution,
};

/// Standard gravity used for `Isp` → mass-flow conversion.
/// Matches the [`crate::motor`] convention.
const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;

// ---------------------------------------------------------------------
// EngineLimits
// ---------------------------------------------------------------------

/// Authority envelope for an [`EngineModel`].
///
/// The trait clamps `EngineCommand.throttle_unit` to
/// `[0, 1]` and `gimbal_*_rad` to `[-max_gimbal_rad, max_gimbal_rad]`.
/// These limits are validated at construction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineLimits {
    /// Maximum thrust at full throttle, in Newtons. Strictly positive.
    pub max_thrust_n: f64,
    /// Specific impulse, in seconds. Strictly positive.
    pub isp_s: f64,
    /// Linear ignition transient duration, in seconds. Non-negative.
    /// `0.0` → instantaneous ignition.
    pub ignition_transient_s: f64,
    /// Linear shutdown transient duration, in seconds. Non-negative.
    /// `0.0` → instantaneous shutdown.
    pub shutdown_transient_s: f64,
    /// Maximum absolute gimbal angle on either axis, in radians.
    /// Non-negative. `0.0` → fixed-axis engine (no gimbal).
    pub max_gimbal_rad: f64,
    /// Maximum absolute gimbal slew rate, in radians per second.
    /// `+∞` preserves the pre-existing instantaneous gimbal latch; a
    /// finite rate bounds the per-step gimbal change, which prevents a
    /// step-frequency bang-bang limit cycle when an autopilot commands
    /// large gimbal swings every tick.
    pub gimbal_slew_rad_per_s: f64,
    /// Maximum absolute throttle slew rate, in throttle units per
    /// second. `+∞` preserves the pre-existing instantaneous latch.
    pub throttle_slew_per_s: f64,
    /// Deep-throttle floor. Non-zero commands below this value clamp
    /// to this value; zero remains shutdown.
    pub min_throttle_unit: f64,
    /// Linear specific-impulse derate coefficient:
    /// `Isp(τ) = Isp · (1 - k · (1 - τ))`.
    pub isp_throttle_falloff: f64,
}

impl EngineLimits {
    /// Validate the limits. Construction-time check; rejects
    /// non-finite or out-of-range values.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidLimits`] for non-finite values,
    /// `max_thrust_n ≤ 0`, `isp_s ≤ 0`, negative `ignition_transient_s`
    /// or `shutdown_transient_s`, or negative `max_gimbal_rad`.
    pub fn require_valid(&self) -> Result<(), EngineError> {
        if !self.max_thrust_n.is_finite() || self.max_thrust_n <= 0.0 {
            return Err(EngineError::InvalidLimits {
                reason: "max_thrust_n must be finite and strictly positive",
            });
        }
        if !self.isp_s.is_finite() || self.isp_s <= 0.0 {
            return Err(EngineError::InvalidLimits {
                reason: "isp_s must be finite and strictly positive",
            });
        }
        if !self.ignition_transient_s.is_finite() || self.ignition_transient_s < 0.0 {
            return Err(EngineError::InvalidLimits {
                reason: "ignition_transient_s must be finite and non-negative",
            });
        }
        if !self.shutdown_transient_s.is_finite() || self.shutdown_transient_s < 0.0 {
            return Err(EngineError::InvalidLimits {
                reason: "shutdown_transient_s must be finite and non-negative",
            });
        }
        if !self.max_gimbal_rad.is_finite() || self.max_gimbal_rad < 0.0 {
            return Err(EngineError::InvalidLimits {
                reason: "max_gimbal_rad must be finite and non-negative",
            });
        }
        if self.gimbal_slew_rad_per_s.is_nan() || self.gimbal_slew_rad_per_s < 0.0 {
            return Err(EngineError::InvalidLimits {
                reason: "gimbal_slew_rad_per_s must be non-negative or +infinity",
            });
        }
        if self.throttle_slew_per_s.is_nan() || self.throttle_slew_per_s < 0.0 {
            return Err(EngineError::InvalidLimits {
                reason: "throttle_slew_per_s must be non-negative or +infinity",
            });
        }
        if self.throttle_slew_per_s.is_infinite() && self.throttle_slew_per_s.is_sign_negative() {
            return Err(EngineError::InvalidLimits {
                reason: "throttle_slew_per_s must be non-negative or +infinity",
            });
        }
        if !self.min_throttle_unit.is_finite() || !(0.0..=1.0).contains(&self.min_throttle_unit) {
            return Err(EngineError::InvalidLimits {
                reason: "min_throttle_unit must be finite and lie in [0, 1]",
            });
        }
        if !self.isp_throttle_falloff.is_finite()
            || !(0.0..1.0).contains(&self.isp_throttle_falloff)
        {
            return Err(EngineError::InvalidLimits {
                reason: "isp_throttle_falloff must be finite and lie in [0, 1)",
            });
        }
        Ok(())
    }

    /// Return a copy of these limits with thermochemical liquid-engine
    /// performance applied to `max_thrust_n` and `isp_s`.
    ///
    /// The remaining control/lifecycle limits are preserved. This is the
    /// construction-time bridge from a CEA/Cantera deck lookup plus nozzle
    /// geometry into the existing [`LiquidEngine`] model.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError`] if the resulting limits fail validation.
    pub fn with_liquid_performance(
        mut self,
        performance: LiquidEnginePerformance,
    ) -> Result<Self, EngineError> {
        self.max_thrust_n = performance.max_thrust_n;
        self.isp_s = performance.isp_s;
        self.require_valid()?;
        Ok(self)
    }
}

// ---------------------------------------------------------------------
// LiquidEnginePerformance
// ---------------------------------------------------------------------

/// Thermochemical state consumed by a liquid-engine performance solve.
///
/// These are the fields produced by a Schema-1 thermochemistry deck lookup.
/// The propulsion crate accepts the already-looked-up values to avoid adding
/// a dependency edge from `openbmp-propulsion` to `openbmp-thermochem`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiquidEngineThermochemistry {
    /// Chamber pressure in Pa.
    pub chamber_pressure_pa: f64,
    /// Ideal characteristic velocity in m/s.
    pub c_star_m_s: f64,
    /// Equilibrium gas specific-heat ratio.
    pub gamma: f64,
}

/// Empirical `c*` efficiency band for thermochemical performance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiquidEngineCStarEfficiencyBand {
    /// Lower efficiency bound.
    pub min: f64,
    /// Nominal efficiency used for deterministic runtime propagation.
    pub nominal: f64,
    /// Upper efficiency bound.
    pub max: f64,
}

impl Default for LiquidEngineCStarEfficiencyBand {
    fn default() -> Self {
        Self {
            min: 1.0,
            nominal: 1.0,
            max: 1.0,
        }
    }
}

/// Scalar nominal value plus lower/upper envelope.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiquidEngineScalarBand {
    /// Lower bound.
    pub min: f64,
    /// Nominal value used by the deterministic engine model.
    pub nominal: f64,
    /// Upper bound.
    pub max: f64,
}

/// Nozzle and ambient inputs for a liquid-engine performance solve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiquidEngineNozzle {
    /// Nozzle throat area in m².
    pub throat_area_m2: f64,
    /// Nozzle exit area in m².
    pub exit_area_m2: f64,
    /// Ambient static pressure in Pa for the design/performance point.
    pub ambient_pressure_pa: f64,
    /// Optional overexpanded-flow separation criterion.
    pub separation: NozzleSeparationCriterion,
}

/// Liquid-engine thrust, mass-flow, and `Isp` derived from thermochemistry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiquidEnginePerformance {
    /// Full-throttle thrust in N at the declared chamber/nozzle/ambient point.
    pub max_thrust_n: f64,
    /// Effective specific impulse in seconds at the declared point.
    pub isp_s: f64,
    /// Choked throat mass flow in kg/s, computed as `pc * At / c*`.
    pub mass_flow_kg_per_s: f64,
    /// Underlying ideal-nozzle solution used to derive thrust and `Isp`.
    pub nozzle: NozzleSolution,
    /// Empirical `c*` efficiency band used to derive the nominal/envelope values.
    pub c_star_efficiency: LiquidEngineCStarEfficiencyBand,
    /// Choked throat mass-flow envelope in kg/s.
    pub mass_flow_band_kg_per_s: LiquidEngineScalarBand,
    /// Effective specific-impulse envelope in seconds.
    pub isp_band_s: LiquidEngineScalarBand,
}

impl LiquidEnginePerformance {
    /// Solve thermochemistry-derived liquid-engine performance.
    ///
    /// `max_thrust_n` comes from the same ideal-nozzle pressure-thrust solve
    /// used by pressure-thrust solid motors. `isp_s` is then tied to the
    /// thermochemical `c*` through the choked throat mass flow, so liquid
    /// engine propellant consumption can move with deck-derived `c*`/`gamma`
    /// instead of a hardcoded `Isp` constant.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError`] if any scalar is non-finite or outside the
    /// nozzle/thermochemistry envelope.
    pub fn from_thermochemistry(
        thermochemistry: LiquidEngineThermochemistry,
        nozzle: LiquidEngineNozzle,
    ) -> Result<Self, EngineError> {
        Self::from_thermochemistry_with_efficiency(
            thermochemistry,
            nozzle,
            LiquidEngineCStarEfficiencyBand::default(),
        )
    }

    /// Solve thermochemistry-derived liquid-engine performance with an
    /// empirical `c*` efficiency band.
    ///
    /// The deterministic engine model uses the nominal efficiency. The
    /// returned [`LiquidEnginePerformance`] also carries min/max mass-flow and
    /// `Isp` bands so validation and UQ layers can retain the documented
    /// empirical performance envelope.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError`] if any scalar is non-finite or outside the
    /// nozzle/thermochemistry/efficiency envelope.
    pub fn from_thermochemistry_with_efficiency(
        thermochemistry: LiquidEngineThermochemistry,
        nozzle: LiquidEngineNozzle,
        efficiency: LiquidEngineCStarEfficiencyBand,
    ) -> Result<Self, EngineError> {
        validate_liquid_thermochemistry(thermochemistry)?;
        validate_liquid_nozzle(nozzle)?;
        validate_liquid_efficiency(efficiency)?;

        let mass_flow_kg_per_s =
            liquid_mass_flow_for_efficiency(thermochemistry, nozzle, efficiency.nominal)?;
        if !mass_flow_kg_per_s.is_finite() || mass_flow_kg_per_s <= 0.0 {
            return Err(EngineError::InvalidParameter {
                reason: "thermochemical liquid-engine mass flow must be finite and positive",
            });
        }

        let solution = IdealNozzlePerformance::new(nozzle.separation)
            .solve(
                ChamberState {
                    chamber_pressure_pa: thermochemistry.chamber_pressure_pa,
                    mass_flow_kg_s: mass_flow_kg_per_s,
                    gamma: thermochemistry.gamma,
                    throat_area_m2: nozzle.throat_area_m2,
                    exit_area_m2: nozzle.exit_area_m2,
                },
                nozzle.ambient_pressure_pa,
            )
            .map_err(|_| EngineError::InvalidParameter {
                reason: "thermochemical liquid-engine nozzle solve failed",
            })?;
        if solution.total_thrust_n <= 0.0 || solution.effective_isp_s <= 0.0 {
            return Err(EngineError::InvalidParameter {
                reason: "thermochemical liquid-engine performance must produce positive thrust and Isp",
            });
        }
        let mass_flow_band_kg_per_s = LiquidEngineScalarBand {
            min: liquid_mass_flow_for_efficiency(thermochemistry, nozzle, efficiency.max)?,
            nominal: mass_flow_kg_per_s,
            max: liquid_mass_flow_for_efficiency(thermochemistry, nozzle, efficiency.min)?,
        };
        let isp_band_s = LiquidEngineScalarBand {
            min: isp_for_mass_flow(solution.total_thrust_n, mass_flow_band_kg_per_s.max)?,
            nominal: solution.effective_isp_s,
            max: isp_for_mass_flow(solution.total_thrust_n, mass_flow_band_kg_per_s.min)?,
        };
        validate_scalar_band(mass_flow_band_kg_per_s, "mass-flow")?;
        validate_scalar_band(isp_band_s, "Isp")?;
        Ok(Self {
            max_thrust_n: solution.total_thrust_n,
            isp_s: solution.effective_isp_s,
            mass_flow_kg_per_s,
            nozzle: solution,
            c_star_efficiency: efficiency,
            mass_flow_band_kg_per_s,
            isp_band_s,
        })
    }
}

// ---------------------------------------------------------------------
// EngineCommand
// ---------------------------------------------------------------------

/// Per-engine command payload. Carried by scenario-script engine
/// command events and by the runner-side rack's per-step bookkeeping.
///
/// When both `ignite` and `shutdown` are `true` in the same command,
/// **shutdown wins**: from `Igniting` / `Burning`, the engine
/// transitions to `Shutdown`; from `Idle`, the command is a no-op
/// (no transition). The trait does not reject ambiguous commands at
/// the trait level — the scenario's parse-time validation is the
/// right place if the operator wants strictness.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineCommand {
    /// Throttle setting in `[0, 1]`. Values outside the range are
    /// clamped to the envelope at `apply_command` time, not rejected.
    pub throttle_unit: f64,
    /// Gimbal pitch angle in radians. Clamped to
    /// `[-max_gimbal_rad, max_gimbal_rad]` at `apply_command`.
    pub gimbal_pitch_rad: f64,
    /// Gimbal yaw angle in radians. Clamped likewise.
    pub gimbal_yaw_rad: f64,
    /// Ignition request. Honoured only from `Idle`.
    pub ignite: bool,
    /// Shutdown request. Honoured only from `Igniting` or `Burning`.
    pub shutdown: bool,
}

impl EngineCommand {
    /// Construct a no-op command (no ignite, no shutdown, throttle 0,
    /// no gimbal). Useful as the initial latch before any real
    /// command fires.
    #[must_use]
    pub const fn idle() -> Self {
        Self {
            throttle_unit: 0.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
            shutdown: false,
        }
    }
}

// ---------------------------------------------------------------------
// EngineState
// ---------------------------------------------------------------------

/// Engine lifecycle state. One-shot: `Shutdown` and `Failed` are
/// terminal — no re-ignition.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EngineState {
    /// Pre-ignition. Awaiting an `ignite=true` command.
    Idle,
    /// Ignition transient: thrust ramping linearly from 0.
    Igniting,
    /// Steady-state burn at the latched throttle.
    Burning,
    /// Shutdown transient: thrust ramping linearly to 0.
    Shutdown,
    /// Hard failure (load-time fault rejected at construction or
    /// runtime fault that takes the engine out of service). Terminal.
    Failed,
}

// ---------------------------------------------------------------------
// EngineSnapshot
// ---------------------------------------------------------------------

/// Per-step output snapshot from an [`EngineModel`].
///
/// `thrust_body` carries the gimbal-applied thrust vector in the
/// engine's body frame: nominal direction is body `+z`; gimbal pitch
/// rotates around body `y`, gimbal yaw around body `x` (locked
/// operand order).
///
/// `consumed_kg` is the cumulative propellant mass deficit since
/// construction. The cluster-side mass adapter sums this across
/// engines and subtracts from `initial_propellant_per_engine` to
/// report the live cluster mass. Forward-Euler integration of
/// `mass_flow_kg_per_s` on the kernel side is intentionally NOT
/// used — it would drift relative to the rack and break determinism.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineSnapshot {
    /// Body-frame thrust vector in Newtons. Gimbal applied.
    pub thrust_body: Vector3<f64>,
    /// Mass-flow rate at this step, in kg/s. Always non-negative.
    pub mass_flow_kg_per_s: f64,
    /// Cumulative propellant consumed since construction, in kg.
    /// Non-decreasing.
    pub consumed_kg: f64,
    /// Engine state at this step.
    pub state: EngineState,
}

impl EngineSnapshot {
    /// Construct the at-rest snapshot (zero thrust, zero mass-flow,
    /// zero consumed, `Idle`). Used as the initial cache before the
    /// first step.
    #[must_use]
    pub fn idle() -> Self {
        Self {
            thrust_body: Vector3::zeros(),
            mass_flow_kg_per_s: 0.0,
            consumed_kg: 0.0,
            state: EngineState::Idle,
        }
    }
}

// ---------------------------------------------------------------------
// EngineFault
// ---------------------------------------------------------------------

/// Canonical fault modes for an engine. Mirrors the
/// effector-fault taxonomy (Patton, Frank & Clark 1989 *Fault
/// Diagnosis in Dynamic Systems*).
///
/// Faults may be injected at construction or through runner-scheduled
/// runtime rules. A `Failed` engine never recovers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EngineFault {
    /// Throttle stuck at `at_throttle`; engine ignores command
    /// throttle but still honours ignite / shutdown lifecycle.
    Stuck {
        /// Stuck throttle value in `[0, 1]`.
        at_throttle: f64,
    },
    /// Engine commanded off and never restarts. Terminal — sets
    /// state to `Failed` on first step.
    HardOff,
    /// Thrust scaled by `factor` (commanded or uncommanded).
    /// `factor ≥ 1` → over-thrust; `factor < 1` → under-thrust.
    OverThrust {
        /// Thrust multiplier. Non-negative finite.
        factor: f64,
    },
    /// Ignition transient over-pressure. Scales thrust only while the engine is
    /// igniting and `elapsed_in_state_s <= duration_s`.
    HardStartOverpressure {
        /// Ignition over-pressure multiplier. Must be finite and at least 1.
        factor: f64,
        /// Duration in seconds from ignition start. Must be finite and positive.
        duration_s: f64,
    },
    /// Pump-cavitation thrust loss. Scales thrust by `factor` once injected.
    CavitationThrustLoss {
        /// Thrust multiplier after cavitation onset. Must be finite in `[0, 1]`.
        factor: f64,
    },
    /// Gimbal frozen at the given angles, regardless of command.
    GimbalLocked {
        /// Locked pitch angle in radians.
        pitch_rad: f64,
        /// Locked yaw angle in radians.
        yaw_rad: f64,
    },
}

// ---------------------------------------------------------------------
// EngineModel trait
// ---------------------------------------------------------------------

/// Engine trait.
///
/// Engines are stateful (lifecycle state machine, latched command,
/// integrated propellant deficit, fault mode) and therefore take
/// `&mut self`. The trait is scalar/vector at the engine boundary;
/// kernel coupling lives in the runner-side rack and the
/// vehicle-side cluster adapters.
pub trait EngineModel: Debug + Send + Sync {
    /// Stable identifier for telemetry and event-action addressing.
    fn id(&self) -> EngineId;

    /// Latch a controller-or-event command for the next `step()`.
    /// Lifecycle transitions (`ignite`, `shutdown`) are decided here
    /// based on the current state. Throttle / gimbal values are
    /// clamped to the engine's limits.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::NonFiniteCommand`] when any scalar in
    /// the command is `NaN` or `Inf`.
    fn apply_command(&mut self, cmd: EngineCommand) -> Result<(), EngineError>;

    /// Advance the engine state by one kernel base tick. Returns the
    /// fresh [`EngineSnapshot`] (gimbal-applied thrust, mass-flow,
    /// integrated consumption, state).
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidDt`] when `dt` is non-finite or
    /// non-positive.
    fn step(&mut self, dt: Duration) -> Result<EngineSnapshot, EngineError>;

    /// Return the configured limits.
    fn limits(&self) -> EngineLimits;

    /// Inject a fault. Replaces any prior fault after validating the
    /// payload against this engine's authority envelope.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidFault`] when the payload is
    /// non-finite or out of range.
    fn inject_fault(&mut self, fault: EngineFault) -> Result<(), EngineError>;

    /// Return the current lifecycle state.
    fn current_state(&self) -> EngineState;

    /// Return the most recent snapshot. Useful for telemetry without
    /// re-stepping.
    fn current_snapshot(&self) -> EngineSnapshot;

    /// Apply a feed-pressure scale to the next step. `1.0` is the
    /// regulated/default path; lower values model blowdown thrust and
    /// Isp decay. Implementations that do not support feed coupling
    /// should validate the scalar and otherwise ignore it.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidParameter`] when `scale` is
    /// non-finite or negative.
    fn set_feed_pressure_scale(&mut self, scale: f64) -> Result<(), EngineError> {
        if !scale.is_finite() || scale < 0.0 {
            return Err(EngineError::InvalidParameter {
                reason: "feed pressure scale must be finite and non-negative",
            });
        }
        Ok(())
    }

    /// Validation-status declaration. Matches the kernel's
    /// `ForceModel::validation()` / `MassModel::validation()` shape
    /// so adapters can forward through.
    fn validation(&self) -> ValidationStatus;
}

// ---------------------------------------------------------------------
// LiquidEngine reference impl
// ---------------------------------------------------------------------

/// Reference [`EngineModel`] implementation: linear
/// ignition transient, rate-limited throttle burn, linear shutdown
/// transient. Mass flow `mdot = thrust / (g0 · effective_isp)`. Gimbal applied
/// as locked-order pitch-around-body-y then yaw-around-body-x
/// rotation of nominal body-`+z` thrust.
#[derive(Debug)]
pub struct LiquidEngine {
    id: EngineId,
    limits: EngineLimits,
    state: EngineState,
    /// Seconds elapsed since entering the current state. Reset on
    /// transition.
    elapsed_in_state_s: f64,
    /// Commanded throttle target, clamped to `[0, 1]` at apply time.
    commanded_throttle: f64,
    /// Current delivered throttle after slew and floor handling.
    latched_throttle: f64,
    /// Commanded gimbal pitch target (rad), clamped to `±max_gimbal_rad`
    /// at apply time. The delivered `latched_pitch_rad` slews toward
    /// this at the actuator slew rate.
    commanded_pitch_rad: f64,
    /// Commanded gimbal yaw target (rad), clamped to `±max_gimbal_rad`.
    commanded_yaw_rad: f64,
    /// Latched (delivered) gimbal pitch (rad). Slews toward the
    /// commanded target each step at the gimbal slew rate.
    latched_pitch_rad: f64,
    /// Latched (delivered) gimbal yaw (rad).
    latched_yaw_rad: f64,
    /// Thrust at the moment shutdown was commanded; the linear
    /// shutdown transient ramps from this value to 0.
    shutdown_start_thrust_n: f64,
    /// Cumulative propellant consumed in kg.
    consumed_kg: f64,
    /// Feed-pressure multiplier supplied by the vehicle-side
    /// propellant budget. `1.0` is regulated/default.
    feed_pressure_scale: f64,
    /// Last computed snapshot. Returned by `current_snapshot()`.
    last_snapshot: EngineSnapshot,
    /// Active fault, if any.
    fault: Option<EngineFault>,
    /// When `true`, `Shutdown` is NOT terminal: once the shutdown
    /// transient completes the engine re-arms to `Idle` and a later
    /// `ignite` re-ignites it (e.g. an upper-stage restart for a second
    /// burn). Default `false` keeps the one-shot lifecycle.
    restartable: bool,
}

impl LiquidEngine {
    /// Construct a fresh `LiquidEngine` in the `Idle` state.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidLimits`] when the limits fail
    /// validation.
    pub fn new(id: EngineId, limits: EngineLimits) -> Result<Self, EngineError> {
        limits.require_valid()?;
        Ok(Self {
            id,
            limits,
            state: EngineState::Idle,
            elapsed_in_state_s: 0.0,
            commanded_throttle: 0.0,
            latched_throttle: 0.0,
            commanded_pitch_rad: 0.0,
            commanded_yaw_rad: 0.0,
            latched_pitch_rad: 0.0,
            latched_yaw_rad: 0.0,
            shutdown_start_thrust_n: 0.0,
            consumed_kg: 0.0,
            feed_pressure_scale: 1.0,
            last_snapshot: EngineSnapshot::idle(),
            fault: None,
            restartable: false,
        })
    }

    /// Construct a liquid engine after applying thermochemical performance to
    /// the supplied limits.
    ///
    /// Control and lifecycle limits remain those supplied by `limits`; only
    /// full-throttle thrust and nominal `Isp` are replaced.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError`] if the performance-updated limits are invalid.
    pub fn new_with_performance(
        id: EngineId,
        limits: EngineLimits,
        performance: LiquidEnginePerformance,
    ) -> Result<Self, EngineError> {
        Self::new(id, limits.with_liquid_performance(performance)?)
    }

    /// Set the restart policy. When `restartable` is `true`, the engine
    /// re-arms to `Idle` after a completed shutdown transient instead of
    /// latching `Shutdown` terminally, so a later `ignite` command starts a
    /// new burn. Builder-style; defaults to `false` (one-shot).
    #[must_use]
    pub fn with_restart_policy(mut self, restartable: bool) -> Self {
        self.restartable = restartable;
        self
    }

    /// Construction-time fault validator. Rejects non-finite,
    /// out-of-range payloads against the engine's limits.
    fn validate_fault(&self, fault: EngineFault) -> Result<(), EngineError> {
        match fault {
            EngineFault::Stuck { at_throttle } => {
                if !at_throttle.is_finite() {
                    return Err(EngineError::InvalidFault {
                        reason: "Stuck.at_throttle must be finite",
                    });
                }
                if !(0.0..=1.0).contains(&at_throttle) {
                    return Err(EngineError::InvalidFault {
                        reason: "Stuck.at_throttle must lie in [0, 1]",
                    });
                }
            }
            EngineFault::HardOff => {}
            EngineFault::OverThrust { factor } => {
                if !factor.is_finite() {
                    return Err(EngineError::InvalidFault {
                        reason: "OverThrust.factor must be finite",
                    });
                }
                if factor < 0.0 {
                    return Err(EngineError::InvalidFault {
                        reason: "OverThrust.factor must be non-negative",
                    });
                }
            }
            EngineFault::HardStartOverpressure { factor, duration_s } => {
                if !factor.is_finite() {
                    return Err(EngineError::InvalidFault {
                        reason: "HardStartOverpressure.factor must be finite",
                    });
                }
                if factor < 1.0 {
                    return Err(EngineError::InvalidFault {
                        reason: "HardStartOverpressure.factor must be at least 1",
                    });
                }
                if !duration_s.is_finite() {
                    return Err(EngineError::InvalidFault {
                        reason: "HardStartOverpressure.duration_s must be finite",
                    });
                }
                if duration_s <= 0.0 {
                    return Err(EngineError::InvalidFault {
                        reason: "HardStartOverpressure.duration_s must be positive",
                    });
                }
            }
            EngineFault::CavitationThrustLoss { factor } => {
                if !factor.is_finite() {
                    return Err(EngineError::InvalidFault {
                        reason: "CavitationThrustLoss.factor must be finite",
                    });
                }
                if !(0.0..=1.0).contains(&factor) {
                    return Err(EngineError::InvalidFault {
                        reason: "CavitationThrustLoss.factor must lie in [0, 1]",
                    });
                }
            }
            EngineFault::GimbalLocked { pitch_rad, yaw_rad } => {
                if !pitch_rad.is_finite() || !yaw_rad.is_finite() {
                    return Err(EngineError::InvalidFault {
                        reason: "GimbalLocked angles must be finite",
                    });
                }
                if pitch_rad.abs() > self.limits.max_gimbal_rad
                    || yaw_rad.abs() > self.limits.max_gimbal_rad
                {
                    return Err(EngineError::InvalidFault {
                        reason: "GimbalLocked angles must lie within ±max_gimbal_rad",
                    });
                }
            }
        }
        Ok(())
    }
}

fn validate_liquid_thermochemistry(
    thermochemistry: LiquidEngineThermochemistry,
) -> Result<(), EngineError> {
    for value in [
        thermochemistry.chamber_pressure_pa,
        thermochemistry.c_star_m_s,
        thermochemistry.gamma,
    ] {
        if !value.is_finite() {
            return Err(EngineError::InvalidParameter {
                reason: "thermochemical liquid-engine state must be finite",
            });
        }
    }
    if thermochemistry.chamber_pressure_pa <= 0.0 {
        return Err(EngineError::InvalidParameter {
            reason: "thermochemical liquid-engine chamber pressure must be positive",
        });
    }
    if thermochemistry.c_star_m_s <= 0.0 {
        return Err(EngineError::InvalidParameter {
            reason: "thermochemical liquid-engine c_star_m_s must be positive",
        });
    }
    if thermochemistry.gamma <= 1.0 {
        return Err(EngineError::InvalidParameter {
            reason: "thermochemical liquid-engine gamma must be greater than 1",
        });
    }
    Ok(())
}

fn validate_liquid_nozzle(nozzle: LiquidEngineNozzle) -> Result<(), EngineError> {
    for value in [
        nozzle.throat_area_m2,
        nozzle.exit_area_m2,
        nozzle.ambient_pressure_pa,
    ] {
        if !value.is_finite() {
            return Err(EngineError::InvalidParameter {
                reason: "thermochemical liquid-engine nozzle values must be finite",
            });
        }
    }
    if nozzle.throat_area_m2 <= 0.0 || nozzle.exit_area_m2 < nozzle.throat_area_m2 {
        return Err(EngineError::InvalidParameter {
            reason: "thermochemical liquid-engine nozzle areas must satisfy exit >= throat > 0",
        });
    }
    if nozzle.ambient_pressure_pa < 0.0 {
        return Err(EngineError::InvalidParameter {
            reason: "thermochemical liquid-engine ambient pressure must be non-negative",
        });
    }
    Ok(())
}

fn validate_liquid_efficiency(
    efficiency: LiquidEngineCStarEfficiencyBand,
) -> Result<(), EngineError> {
    for value in [efficiency.min, efficiency.nominal, efficiency.max] {
        if !value.is_finite() {
            return Err(EngineError::InvalidParameter {
                reason: "thermochemical liquid-engine c_star efficiency must be finite",
            });
        }
        if value <= 0.0 || value > 1.0 {
            return Err(EngineError::InvalidParameter {
                reason: "thermochemical liquid-engine c_star efficiency must lie in (0, 1]",
            });
        }
    }
    if efficiency.min > efficiency.nominal || efficiency.nominal > efficiency.max {
        return Err(EngineError::InvalidParameter {
            reason: "thermochemical liquid-engine c_star efficiency must satisfy min <= nominal <= max",
        });
    }
    Ok(())
}

fn liquid_mass_flow_for_efficiency(
    thermochemistry: LiquidEngineThermochemistry,
    nozzle: LiquidEngineNozzle,
    efficiency: f64,
) -> Result<f64, EngineError> {
    let effective_c_star_m_s = thermochemistry.c_star_m_s * efficiency;
    if !effective_c_star_m_s.is_finite() || effective_c_star_m_s <= 0.0 {
        return Err(EngineError::InvalidParameter {
            reason: "thermochemical liquid-engine effective c_star must be finite and positive",
        });
    }
    let mass_flow_kg_per_s =
        thermochemistry.chamber_pressure_pa * nozzle.throat_area_m2 / effective_c_star_m_s;
    if !mass_flow_kg_per_s.is_finite() || mass_flow_kg_per_s <= 0.0 {
        return Err(EngineError::InvalidParameter {
            reason: "thermochemical liquid-engine mass flow must be finite and positive",
        });
    }
    Ok(mass_flow_kg_per_s)
}

fn isp_for_mass_flow(total_thrust_n: f64, mass_flow_kg_per_s: f64) -> Result<f64, EngineError> {
    if !total_thrust_n.is_finite()
        || !mass_flow_kg_per_s.is_finite()
        || total_thrust_n <= 0.0
        || mass_flow_kg_per_s <= 0.0
    {
        return Err(EngineError::InvalidParameter {
            reason: "thermochemical liquid-engine Isp inputs must be finite and positive",
        });
    }
    let isp_s = total_thrust_n / (STANDARD_GRAVITY_M_S2 * mass_flow_kg_per_s);
    if !isp_s.is_finite() || isp_s <= 0.0 {
        return Err(EngineError::InvalidParameter {
            reason: "thermochemical liquid-engine Isp must be finite and positive",
        });
    }
    Ok(isp_s)
}

fn validate_scalar_band(
    band: LiquidEngineScalarBand,
    field: &'static str,
) -> Result<(), EngineError> {
    for value in [band.min, band.nominal, band.max] {
        if !value.is_finite() || value <= 0.0 {
            return Err(EngineError::InvalidParameter { reason: field });
        }
    }
    if band.min > band.nominal || band.nominal > band.max {
        return Err(EngineError::InvalidParameter { reason: field });
    }
    Ok(())
}

/// Apply gimbal rotation to a nominal body-`+z` thrust scalar.
///
/// Locked operand order: rotate by pitch around body-`y` first, then
/// by yaw around body-`x`. FMA disabled — each multiplication is its
/// own `fmul`. Returns the thrust vector in body frame.
fn apply_gimbal(thrust_z_n: f64, pitch_rad: f64, yaw_rad: f64) -> Vector3<f64> {
    let cp = pitch_rad.cos();
    let sp = pitch_rad.sin();
    let cy = yaw_rad.cos();
    let sy = yaw_rad.sin();
    Vector3::new(
        thrust_z_n * sp,
        -(thrust_z_n * cp) * sy,
        (thrust_z_n * cp) * cy,
    )
}

impl EngineModel for LiquidEngine {
    fn id(&self) -> EngineId {
        self.id
    }

    fn apply_command(&mut self, cmd: EngineCommand) -> Result<(), EngineError> {
        if !cmd.throttle_unit.is_finite() {
            return Err(EngineError::NonFiniteCommand {
                reason: "throttle_unit",
            });
        }
        if !cmd.gimbal_pitch_rad.is_finite() {
            return Err(EngineError::NonFiniteCommand {
                reason: "gimbal_pitch_rad",
            });
        }
        if !cmd.gimbal_yaw_rad.is_finite() {
            return Err(EngineError::NonFiniteCommand {
                reason: "gimbal_yaw_rad",
            });
        }

        // Failed and Shutdown are terminal — ignore commands.
        if matches!(self.state, EngineState::Failed | EngineState::Shutdown) {
            return Ok(());
        }

        // Lifecycle transitions: ignite from Idle, shutdown from
        // Igniting / Burning. Shutdown takes precedence if both flags
        // are set in the same command (rare but defined behaviour).
        match self.state {
            EngineState::Idle => {
                if cmd.ignite && !cmd.shutdown {
                    self.state = EngineState::Igniting;
                    self.elapsed_in_state_s = 0.0;
                }
            }
            EngineState::Igniting | EngineState::Burning => {
                if cmd.shutdown {
                    // Capture current thrust as the shutdown ramp's
                    // start value. Use the last snapshot's thrust
                    // magnitude (already gimbal-applied) — but for the
                    // ramp we want the un-gimballed scalar magnitude
                    // along the engine's nominal axis. Reconstruct:
                    // last_snapshot.thrust_body's norm is
                    // |scalar_thrust|; sign is implicitly positive.
                    self.shutdown_start_thrust_n = self.last_snapshot.thrust_body.norm();
                    self.state = EngineState::Shutdown;
                    self.elapsed_in_state_s = 0.0;
                }
            }
            EngineState::Shutdown | EngineState::Failed => {}
        }

        // Latch throttle target (clamped to [0, 1]) and gimbal angles
        // (clamped to ±max_gimbal_rad).
        self.commanded_throttle = cmd.throttle_unit.clamp(0.0, 1.0);
        let g = self.limits.max_gimbal_rad;
        self.commanded_pitch_rad = cmd.gimbal_pitch_rad.clamp(-g, g);
        self.commanded_yaw_rad = cmd.gimbal_yaw_rad.clamp(-g, g);

        Ok(())
    }

    fn step(&mut self, dt: Duration) -> Result<EngineSnapshot, EngineError> {
        let dt_s = dt.as_seconds();
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(EngineError::InvalidDt { got_s: dt_s });
        }

        // HardOff fault forces the engine into Failed before any
        // computation. Terminal.
        if matches!(self.fault, Some(EngineFault::HardOff)) {
            self.state = EngineState::Failed;
        }

        // Advance the in-state timer.
        self.elapsed_in_state_s += dt_s;
        self.latched_throttle = next_throttle(
            self.latched_throttle,
            self.commanded_throttle,
            self.limits.throttle_slew_per_s,
            self.limits.min_throttle_unit,
            dt_s,
        );

        // Gimbal actuator slew. Real TVC gimbals move at a finite rate;
        // an instantaneous latch lets the autopilot slam the gimbal
        // ±max each step, producing a step-frequency bang-bang limit
        // cycle. Slewing the delivered angle toward the command bounds
        // the per-step change and stabilises the loop. `+∞` (the
        // default) reproduces the instantaneous latch.
        let max_gimbal_step = self.limits.gimbal_slew_rad_per_s * dt_s;
        self.latched_pitch_rad += (self.commanded_pitch_rad - self.latched_pitch_rad)
            .clamp(-max_gimbal_step, max_gimbal_step);
        self.latched_yaw_rad += (self.commanded_yaw_rad - self.latched_yaw_rad)
            .clamp(-max_gimbal_step, max_gimbal_step);

        // Determine the un-gimballed scalar thrust along the engine
        // nominal axis based on state + elapsed.
        let commanded_thrust_n =
            self.latched_throttle * self.limits.max_thrust_n * self.feed_pressure_scale;
        let mut thrust_z_n = match self.state {
            EngineState::Idle | EngineState::Failed => 0.0,
            EngineState::Igniting => {
                let t_ig = self.limits.ignition_transient_s;
                if t_ig <= 0.0 {
                    // Instantaneous ignition: jump to Burning at this
                    // step.
                    commanded_thrust_n
                } else {
                    let ratio = (self.elapsed_in_state_s / t_ig).min(1.0);
                    ratio * commanded_thrust_n
                }
            }
            EngineState::Burning => commanded_thrust_n,
            EngineState::Shutdown => {
                let t_sd = self.limits.shutdown_transient_s;
                if t_sd <= 0.0 || self.elapsed_in_state_s >= t_sd {
                    0.0
                } else {
                    let ratio = 1.0 - (self.elapsed_in_state_s / t_sd);
                    ratio * self.shutdown_start_thrust_n
                }
            }
        };

        // Apply faults (other than HardOff which is handled at the top).
        let mut effective_pitch_rad = self.latched_pitch_rad;
        let mut effective_yaw_rad = self.latched_yaw_rad;
        match self.fault {
            None | Some(EngineFault::HardOff) => {}
            Some(EngineFault::Stuck { at_throttle }) => {
                // Stuck overrides throttle but respects state-based
                // ramps (e.g. during Shutdown the engine still tapers
                // off).
                let stuck_throttle = floor_throttle(at_throttle, self.limits.min_throttle_unit);
                let stuck_thrust =
                    stuck_throttle * self.limits.max_thrust_n * self.feed_pressure_scale;
                thrust_z_n = match self.state {
                    EngineState::Idle | EngineState::Failed => 0.0,
                    EngineState::Igniting => {
                        let t_ig = self.limits.ignition_transient_s;
                        if t_ig <= 0.0 {
                            stuck_thrust
                        } else {
                            let ratio = (self.elapsed_in_state_s / t_ig).min(1.0);
                            ratio * stuck_thrust
                        }
                    }
                    EngineState::Burning => stuck_thrust,
                    EngineState::Shutdown => {
                        // Shutdown ramp uses captured shutdown_start_thrust_n
                        // — leave thrust_z_n as already computed.
                        thrust_z_n
                    }
                };
            }
            Some(EngineFault::OverThrust { factor }) => {
                thrust_z_n *= factor;
            }
            Some(EngineFault::HardStartOverpressure { factor, duration_s }) => {
                if matches!(self.state, EngineState::Igniting)
                    && self.elapsed_in_state_s <= duration_s
                {
                    thrust_z_n *= factor;
                }
            }
            Some(EngineFault::CavitationThrustLoss { factor }) => {
                thrust_z_n *= factor;
            }
            Some(EngineFault::GimbalLocked { pitch_rad, yaw_rad }) => {
                effective_pitch_rad = pitch_rad;
                effective_yaw_rad = yaw_rad;
            }
        }

        // Apply gimbal rotation to produce body-frame thrust vector.
        let thrust_body = apply_gimbal(thrust_z_n, effective_pitch_rad, effective_yaw_rad);

        // Mass flow from current scalar thrust magnitude. Always
        // non-negative.
        let scalar_thrust = thrust_z_n.abs();
        let effective_isp_s = effective_isp_s(
            self.limits.isp_s,
            self.limits.isp_throttle_falloff,
            self.latched_throttle,
            self.feed_pressure_scale,
        );
        let mass_flow_kg_per_s = if scalar_thrust == 0.0 || effective_isp_s <= 0.0 {
            0.0
        } else {
            scalar_thrust / (STANDARD_GRAVITY_M_S2 * effective_isp_s)
        };
        // Locked operand order: rate · dt → integrated kg this step.
        self.consumed_kg += mass_flow_kg_per_s * dt_s;

        // Lifecycle transitions AFTER computing this step's thrust.
        // This makes the ignition ramp end exactly at
        // commanded_thrust_n on the step where elapsed crosses
        // ignition_transient_s, then transitions to Burning for the
        // next step.
        if matches!(self.state, EngineState::Igniting)
            && (self.limits.ignition_transient_s <= 0.0
                || self.elapsed_in_state_s >= self.limits.ignition_transient_s)
        {
            self.state = EngineState::Burning;
            self.elapsed_in_state_s = 0.0;
        }

        // Restartable engines re-arm to Idle once the shutdown transient has
        // fully tapered thrust to zero, so a later `ignite` begins a new
        // burn (e.g. an upper-stage restart). Non-restartable engines (the
        // default) leave `Shutdown` terminal — byte-identical to before.
        if self.restartable
            && matches!(self.state, EngineState::Shutdown)
            && (self.limits.shutdown_transient_s <= 0.0
                || self.elapsed_in_state_s >= self.limits.shutdown_transient_s)
        {
            self.state = EngineState::Idle;
            self.elapsed_in_state_s = 0.0;
            self.shutdown_start_thrust_n = 0.0;
            self.commanded_throttle = 0.0;
            self.latched_throttle = 0.0;
        }

        let snapshot = EngineSnapshot {
            thrust_body,
            mass_flow_kg_per_s,
            consumed_kg: self.consumed_kg,
            state: self.state,
        };
        self.last_snapshot = snapshot;
        Ok(snapshot)
    }

    fn limits(&self) -> EngineLimits {
        self.limits
    }

    fn inject_fault(&mut self, fault: EngineFault) -> Result<(), EngineError> {
        self.validate_fault(fault)?;
        self.fault = Some(fault);
        Ok(())
    }

    fn current_state(&self) -> EngineState {
        self.state
    }

    fn current_snapshot(&self) -> EngineSnapshot {
        self.last_snapshot
    }

    fn set_feed_pressure_scale(&mut self, scale: f64) -> Result<(), EngineError> {
        if !scale.is_finite() || scale < 0.0 {
            return Err(EngineError::InvalidParameter {
                reason: "feed pressure scale must be finite and non-negative",
            });
        }
        self.feed_pressure_scale = scale;
        Ok(())
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

fn floor_throttle(throttle_unit: f64, min_throttle_unit: f64) -> f64 {
    if throttle_unit <= 0.0 {
        0.0
    } else {
        throttle_unit.max(min_throttle_unit)
    }
}

fn next_throttle(
    current: f64,
    commanded: f64,
    slew_per_s: f64,
    min_throttle_unit: f64,
    dt_s: f64,
) -> f64 {
    let target = floor_throttle(commanded.clamp(0.0, 1.0), min_throttle_unit);
    let raw = if slew_per_s.is_infinite() {
        target
    } else {
        let max_delta = slew_per_s * dt_s;
        let delta = (target - current).clamp(-max_delta, max_delta);
        current + delta
    };
    floor_throttle(raw.clamp(0.0, 1.0), min_throttle_unit)
}

fn effective_isp_s(
    base_isp_s: f64,
    throttle_falloff: f64,
    throttle_unit: f64,
    feed_pressure_scale: f64,
) -> f64 {
    if throttle_unit <= 0.0 || feed_pressure_scale <= 0.0 {
        return 0.0;
    }
    let throttle_factor = 1.0 - throttle_falloff * (1.0 - throttle_unit);
    base_isp_s * throttle_factor * feed_pressure_scale
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::approx_constant
)]
mod tests {
    use super::*;
    use openbmp_core::EngineId;

    fn dt() -> Duration {
        Duration::from_seconds(0.001)
    }

    fn make_limits() -> EngineLimits {
        EngineLimits {
            max_thrust_n: 1000.0,
            isp_s: 250.0,
            ignition_transient_s: 0.1,
            shutdown_transient_s: 0.1,
            max_gimbal_rad: 0.1,
            gimbal_slew_rad_per_s: f64::INFINITY,
            throttle_slew_per_s: f64::INFINITY,
            min_throttle_unit: 0.0,
            isp_throttle_falloff: 0.0,
        }
    }

    fn fresh_engine() -> LiquidEngine {
        LiquidEngine::new(EngineId::from_path("test.engine"), make_limits()).unwrap()
    }

    fn ignite_to_burning(engine: &mut LiquidEngine) {
        engine
            .apply_command(EngineCommand {
                throttle_unit: 1.0,
                gimbal_pitch_rad: 0.0,
                gimbal_yaw_rad: 0.0,
                ignite: true,
                shutdown: false,
            })
            .unwrap();
        // Step through the ignition transient (0.1 s = 100 dts).
        for _ in 0..101 {
            engine.step(dt()).unwrap();
        }
        assert_eq!(engine.current_state(), EngineState::Burning);
    }

    // -----------------------------------------------------------------
    // Limits validation
    // -----------------------------------------------------------------

    #[test]
    fn limits_reject_zero_thrust() {
        let limits = EngineLimits {
            max_thrust_n: 0.0,
            ..make_limits()
        };
        assert!(matches!(
            limits.require_valid(),
            Err(EngineError::InvalidLimits { .. })
        ));
    }

    #[test]
    fn limits_reject_negative_isp() {
        let limits = EngineLimits {
            isp_s: -1.0,
            ..make_limits()
        };
        assert!(matches!(
            limits.require_valid(),
            Err(EngineError::InvalidLimits { .. })
        ));
    }

    #[test]
    fn limits_reject_negative_transient() {
        let limits = EngineLimits {
            ignition_transient_s: -0.1,
            ..make_limits()
        };
        assert!(matches!(
            limits.require_valid(),
            Err(EngineError::InvalidLimits { .. })
        ));
    }

    #[test]
    fn limits_reject_negative_max_gimbal() {
        let limits = EngineLimits {
            max_gimbal_rad: -0.1,
            ..make_limits()
        };
        assert!(matches!(
            limits.require_valid(),
            Err(EngineError::InvalidLimits { .. })
        ));
    }

    // -----------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------

    #[test]
    fn liquid_engine_idle_returns_zero_thrust() {
        let mut e = fresh_engine();
        let snap = e.step(dt()).unwrap();
        assert_eq!(snap.thrust_body.norm().to_bits(), 0.0_f64.to_bits());
        assert_eq!(snap.mass_flow_kg_per_s.to_bits(), 0.0_f64.to_bits());
        assert_eq!(snap.state, EngineState::Idle);
    }

    #[test]
    fn liquid_engine_ignite_from_idle_transitions_to_igniting() {
        let mut e = fresh_engine();
        e.apply_command(EngineCommand {
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: true,
            shutdown: false,
        })
        .unwrap();
        assert_eq!(e.current_state(), EngineState::Igniting);
    }

    #[test]
    fn liquid_engine_ignition_transient_is_linear_rise_over_t_ignite_s() {
        let mut e = fresh_engine();
        e.apply_command(EngineCommand {
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: true,
            shutdown: false,
        })
        .unwrap();
        // After 50 ms (half the 100 ms transient), thrust should be
        // ~0.5 of max.
        for _ in 0..50 {
            e.step(dt()).unwrap();
        }
        let snap = e.current_snapshot();
        let expected = 0.5 * make_limits().max_thrust_n;
        assert!((snap.thrust_body.z - expected).abs() / expected < 0.02);
    }

    #[test]
    fn liquid_engine_burn_phase_returns_throttle_times_max_thrust() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        e.apply_command(EngineCommand {
            throttle_unit: 0.5,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
            shutdown: false,
        })
        .unwrap();
        let snap = e.step(dt()).unwrap();
        let expected = 0.5 * make_limits().max_thrust_n;
        assert_eq!(snap.thrust_body.z.to_bits(), expected.to_bits());
    }

    #[test]
    fn liquid_engine_shutdown_transient_is_linear_fall_over_t_shutdown_s() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        e.apply_command(EngineCommand {
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
            shutdown: true,
        })
        .unwrap();
        assert_eq!(e.current_state(), EngineState::Shutdown);
        // After 50 ms (half the 100 ms transient), thrust should be
        // ~0.5 of max.
        for _ in 0..50 {
            e.step(dt()).unwrap();
        }
        let snap = e.current_snapshot();
        let expected = 0.5 * make_limits().max_thrust_n;
        assert!((snap.thrust_body.z - expected).abs() / expected < 0.02);
    }

    #[test]
    fn liquid_engine_post_shutdown_returns_zero_thrust() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        e.apply_command(EngineCommand {
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
            shutdown: true,
        })
        .unwrap();
        // Step through the shutdown transient + a bit more.
        for _ in 0..150 {
            e.step(dt()).unwrap();
        }
        let snap = e.current_snapshot();
        assert_eq!(snap.thrust_body.norm().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn liquid_engine_cannot_re_ignite_after_shutdown() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        e.apply_command(EngineCommand {
            shutdown: true,
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
        })
        .unwrap();
        for _ in 0..150 {
            e.step(dt()).unwrap();
        }
        // Try to re-ignite.
        e.apply_command(EngineCommand {
            ignite: true,
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            shutdown: false,
        })
        .unwrap();
        assert_eq!(e.current_state(), EngineState::Shutdown);
    }

    #[test]
    fn liquid_engine_restartable_re_ignites_after_shutdown() {
        let mut e = fresh_engine().with_restart_policy(true);
        ignite_to_burning(&mut e);
        // First-burn propellant consumed.
        let consumed_after_first = e.current_snapshot().consumed_kg;
        assert!(consumed_after_first > 0.0);

        // Shut down and step past the shutdown transient: a restartable
        // engine re-arms to Idle (vs the terminal Shutdown of the default).
        e.apply_command(EngineCommand {
            shutdown: true,
            throttle_unit: 0.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
        })
        .unwrap();
        for _ in 0..150 {
            e.step(dt()).unwrap();
        }
        assert_eq!(
            e.current_state(),
            EngineState::Idle,
            "restartable engine must re-arm to Idle"
        );
        assert_eq!(
            e.current_snapshot().thrust_body.norm().to_bits(),
            0.0_f64.to_bits()
        );

        // Re-ignite for a second burn.
        e.apply_command(EngineCommand {
            ignite: true,
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            shutdown: false,
        })
        .unwrap();
        for _ in 0..101 {
            e.step(dt()).unwrap();
        }
        assert_eq!(
            e.current_state(),
            EngineState::Burning,
            "must re-ignite to Burning"
        );
        assert!(
            e.current_snapshot().thrust_body.norm() > 0.0,
            "second burn must produce thrust"
        );
        // The second burn consumes additional propellant.
        assert!(e.current_snapshot().consumed_kg > consumed_after_first);
    }

    // -----------------------------------------------------------------
    // Throttle clamping & gimbal
    // -----------------------------------------------------------------

    #[test]
    fn liquid_engine_throttle_clamped_to_zero_one() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        // Above-1 throttle: clamped to 1.
        e.apply_command(EngineCommand {
            throttle_unit: 2.5,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
            shutdown: false,
        })
        .unwrap();
        let snap = e.step(dt()).unwrap();
        assert_eq!(snap.thrust_body.z.to_bits(), 1000.0_f64.to_bits());

        // Below-0 throttle: clamped to 0.
        e.apply_command(EngineCommand {
            throttle_unit: -0.5,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
            shutdown: false,
        })
        .unwrap();
        let snap = e.step(dt()).unwrap();
        assert_eq!(snap.thrust_body.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn liquid_engine_throttle_slew_limits_commanded_step() {
        let limits = EngineLimits {
            ignition_transient_s: 0.0,
            throttle_slew_per_s: 2.0,
            ..make_limits()
        };
        let mut e = LiquidEngine::new(EngineId::from_path("test.engine"), limits).unwrap();
        e.apply_command(EngineCommand {
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: true,
            shutdown: false,
        })
        .unwrap();
        let snap = e.step(Duration::from_seconds(0.25)).unwrap();
        assert_eq!(snap.state, EngineState::Burning);
        assert_eq!(snap.thrust_body.z.to_bits(), 500.0_f64.to_bits());
    }

    #[test]
    fn liquid_engine_min_throttle_floor_derates_isp() {
        let limits = EngineLimits {
            ignition_transient_s: 0.0,
            min_throttle_unit: 0.4,
            isp_throttle_falloff: 0.5,
            ..make_limits()
        };
        let mut e = LiquidEngine::new(EngineId::from_path("test.engine"), limits).unwrap();
        e.apply_command(EngineCommand {
            throttle_unit: 0.1,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: true,
            shutdown: false,
        })
        .unwrap();
        let snap = e.step(dt()).unwrap();
        let expected_isp_s = 250.0 * (1.0 - 0.5 * (1.0 - 0.4));
        let expected_mdot = 400.0 / (STANDARD_GRAVITY_M_S2 * expected_isp_s);
        assert_eq!(snap.thrust_body.z.to_bits(), 400.0_f64.to_bits());
        assert!((snap.mass_flow_kg_per_s - expected_mdot).abs() < 1e-12);
    }

    #[test]
    fn liquid_engine_gimbal_clamped_to_max_gimbal_rad() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        // Demand 1.0 rad pitch (limit is 0.1 rad).
        e.apply_command(EngineCommand {
            throttle_unit: 1.0,
            gimbal_pitch_rad: 1.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
            shutdown: false,
        })
        .unwrap();
        let snap = e.step(dt()).unwrap();
        // Pitch was clamped to 0.1 rad, so x = 1000 * sin(0.1) ≈ 99.83.
        let expected_x = 1000.0_f64 * 0.1_f64.sin();
        assert!((snap.thrust_body.x - expected_x).abs() < 1e-9);
    }

    #[test]
    fn liquid_engine_gimbal_zero_returns_pure_z_thrust() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        let snap = e.step(dt()).unwrap();
        // `assert_eq!(_, 0.0)` treats +0.0 and -0.0 as equal under
        // PartialEq, which is the right semantics for "no lateral
        // thrust at zero gimbal". The locked-order rotation can
        // produce -0.0 in y from the `-(t * cp) * sy` term when
        // sy = 0.0; that's mathematically zero and byte-stable across
        // runs.
        assert_eq!(snap.thrust_body.x, 0.0);
        assert_eq!(snap.thrust_body.y, 0.0);
        assert_eq!(snap.thrust_body.z.to_bits(), 1000.0_f64.to_bits());
    }

    // -----------------------------------------------------------------
    // Mass flow
    // -----------------------------------------------------------------

    #[test]
    fn liquid_engine_mass_flow_equals_thrust_over_isp_g0() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        let snap = e.step(dt()).unwrap();
        let expected = 1000.0 / (STANDARD_GRAVITY_M_S2 * 250.0);
        assert!((snap.mass_flow_kg_per_s - expected).abs() < 1e-12);
    }

    fn liquid_thermochemistry() -> LiquidEngineThermochemistry {
        LiquidEngineThermochemistry {
            chamber_pressure_pa: 3_000_000.0,
            c_star_m_s: 1_700.0,
            gamma: 1.22,
        }
    }

    fn liquid_nozzle() -> LiquidEngineNozzle {
        LiquidEngineNozzle {
            throat_area_m2: 0.02,
            exit_area_m2: 0.24,
            ambient_pressure_pa: 101_325.0,
            separation: NozzleSeparationCriterion::Off,
        }
    }

    #[test]
    fn liquid_engine_performance_derives_thrust_and_isp_from_thermochemistry() {
        let performance = LiquidEnginePerformance::from_thermochemistry(
            liquid_thermochemistry(),
            liquid_nozzle(),
        )
        .unwrap();
        let expected_mass_flow = liquid_thermochemistry().chamber_pressure_pa
            * liquid_nozzle().throat_area_m2
            / liquid_thermochemistry().c_star_m_s;
        let expected_isp = performance.max_thrust_n / (STANDARD_GRAVITY_M_S2 * expected_mass_flow);
        assert!((performance.mass_flow_kg_per_s - expected_mass_flow).abs() < 1e-12);
        assert_eq!(
            performance.max_thrust_n.to_bits(),
            performance.nozzle.total_thrust_n.to_bits()
        );
        assert!((performance.isp_s - expected_isp).abs() < 1e-12);
        assert!(performance.isp_s > 0.0);
        assert_eq!(performance.c_star_efficiency, Default::default());
    }

    #[test]
    fn liquid_engine_performance_propagates_c_star_efficiency_band() {
        let efficiency = LiquidEngineCStarEfficiencyBand {
            min: 0.96,
            nominal: 0.98,
            max: 1.0,
        };
        let performance = LiquidEnginePerformance::from_thermochemistry_with_efficiency(
            liquid_thermochemistry(),
            liquid_nozzle(),
            efficiency,
        )
        .unwrap();
        let ideal_mass_flow = liquid_thermochemistry().chamber_pressure_pa
            * liquid_nozzle().throat_area_m2
            / liquid_thermochemistry().c_star_m_s;

        assert!((performance.mass_flow_kg_per_s - ideal_mass_flow / 0.98).abs() < 1e-12);
        assert!((performance.mass_flow_band_kg_per_s.min - ideal_mass_flow).abs() < 1e-12);
        assert!((performance.mass_flow_band_kg_per_s.max - ideal_mass_flow / 0.96).abs() < 1e-12);
        assert!(performance.isp_band_s.min < performance.isp_s);
        assert!(performance.isp_s < performance.isp_band_s.max);
    }

    #[test]
    fn liquid_engine_new_with_performance_uses_thermochemical_mass_flow() {
        let performance = LiquidEnginePerformance::from_thermochemistry(
            liquid_thermochemistry(),
            liquid_nozzle(),
        )
        .unwrap();
        let limits = EngineLimits {
            max_thrust_n: 10.0,
            isp_s: 1.0,
            ignition_transient_s: 0.0,
            ..make_limits()
        };
        let mut engine = LiquidEngine::new_with_performance(
            EngineId::from_path("test.engine"),
            limits,
            performance,
        )
        .unwrap();
        engine
            .apply_command(EngineCommand {
                throttle_unit: 1.0,
                gimbal_pitch_rad: 0.0,
                gimbal_yaw_rad: 0.0,
                ignite: true,
                shutdown: false,
            })
            .unwrap();

        let snap = engine.step(dt()).unwrap();

        assert_eq!(snap.state, EngineState::Burning);
        assert_eq!(
            snap.thrust_body.z.to_bits(),
            performance.max_thrust_n.to_bits()
        );
        assert!((snap.mass_flow_kg_per_s - performance.mass_flow_kg_per_s).abs() < 1e-12);
    }

    #[test]
    fn liquid_engine_performance_rejects_invalid_thermochemistry() {
        let invalid = LiquidEngineThermochemistry {
            c_star_m_s: 0.0,
            ..liquid_thermochemistry()
        };
        assert!(matches!(
            LiquidEnginePerformance::from_thermochemistry(invalid, liquid_nozzle()),
            Err(EngineError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn liquid_engine_consumed_kg_is_monotone_non_decreasing() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        let mut last = e.current_snapshot().consumed_kg;
        for _ in 0..200 {
            let snap = e.step(dt()).unwrap();
            assert!(snap.consumed_kg >= last);
            last = snap.consumed_kg;
        }
    }

    // -----------------------------------------------------------------
    // Faults
    // -----------------------------------------------------------------

    #[test]
    fn liquid_engine_fault_stuck_ignores_command_throttle() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        e.inject_fault(EngineFault::Stuck { at_throttle: 0.3 })
            .unwrap();
        // Command full throttle — fault should override to 0.3.
        e.apply_command(EngineCommand {
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
            shutdown: false,
        })
        .unwrap();
        let snap = e.step(dt()).unwrap();
        assert_eq!(snap.thrust_body.z.to_bits(), 300.0_f64.to_bits());
    }

    #[test]
    fn liquid_engine_fault_hard_off_transitions_to_failed() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        e.inject_fault(EngineFault::HardOff).unwrap();
        let snap = e.step(dt()).unwrap();
        assert_eq!(snap.state, EngineState::Failed);
        assert_eq!(snap.thrust_body.norm().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn liquid_engine_fault_over_thrust_scales_thrust_by_factor() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        e.inject_fault(EngineFault::OverThrust { factor: 1.2 })
            .unwrap();
        let snap = e.step(dt()).unwrap();
        assert_eq!(snap.thrust_body.z.to_bits(), 1200.0_f64.to_bits());
    }

    #[test]
    fn liquid_engine_fault_hard_start_overpressure_boosts_ignition_only() {
        let mut e = fresh_engine();
        e.inject_fault(EngineFault::HardStartOverpressure {
            factor: 2.0,
            duration_s: 0.05,
        })
        .unwrap();
        e.apply_command(EngineCommand {
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: true,
            shutdown: false,
        })
        .unwrap();
        let first = e.step(Duration::from_seconds(0.025)).unwrap();
        let second = e.step(Duration::from_seconds(0.025)).unwrap();
        let after = e.step(Duration::from_seconds(0.025)).unwrap();

        assert_eq!(first.thrust_body.z.to_bits(), 500.0_f64.to_bits());
        assert_eq!(second.thrust_body.z.to_bits(), 1000.0_f64.to_bits());
        assert!((after.thrust_body.z - 750.0).abs() < 1.0e-9);
    }

    #[test]
    fn liquid_engine_fault_cavitation_thrust_loss_scales_thrust_by_factor() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        e.inject_fault(EngineFault::CavitationThrustLoss { factor: 0.4 })
            .unwrap();

        let snap = e.step(dt()).unwrap();

        assert_eq!(snap.thrust_body.z.to_bits(), 400.0_f64.to_bits());
    }

    #[test]
    fn liquid_engine_fault_gimbal_locked_freezes_pitch_yaw() {
        let mut e = fresh_engine();
        ignite_to_burning(&mut e);
        e.inject_fault(EngineFault::GimbalLocked {
            pitch_rad: 0.05,
            yaw_rad: -0.02,
        })
        .unwrap();
        // Command zero gimbal — fault should override.
        e.apply_command(EngineCommand {
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
            shutdown: false,
        })
        .unwrap();
        let snap = e.step(dt()).unwrap();
        let expected = apply_gimbal(1000.0, 0.05, -0.02);
        assert_eq!(snap.thrust_body.x.to_bits(), expected.x.to_bits());
        assert_eq!(snap.thrust_body.y.to_bits(), expected.y.to_bits());
        assert_eq!(snap.thrust_body.z.to_bits(), expected.z.to_bits());
    }

    #[test]
    fn invalid_fault_payload_rejected_without_mutating_existing_fault() {
        let mut e = fresh_engine();
        e.inject_fault(EngineFault::OverThrust { factor: 1.5 })
            .unwrap();
        let err = e
            .inject_fault(EngineFault::Stuck { at_throttle: 5.0 })
            .unwrap_err();
        assert!(matches!(err, EngineError::InvalidFault { .. }));
        // Original fault preserved.
        ignite_to_burning(&mut e);
        let snap = e.step(dt()).unwrap();
        assert_eq!(snap.thrust_body.z.to_bits(), 1500.0_f64.to_bits());
    }

    #[test]
    fn invalid_hard_start_fault_payload_rejected() {
        let mut e = fresh_engine();
        assert!(matches!(
            e.inject_fault(EngineFault::HardStartOverpressure {
                factor: 0.9,
                duration_s: 0.05,
            }),
            Err(EngineError::InvalidFault { .. })
        ));
        assert!(matches!(
            e.inject_fault(EngineFault::HardStartOverpressure {
                factor: 1.5,
                duration_s: 0.0,
            }),
            Err(EngineError::InvalidFault { .. })
        ));
    }

    #[test]
    fn invalid_cavitation_fault_payload_rejected() {
        let mut e = fresh_engine();
        assert!(matches!(
            e.inject_fault(EngineFault::CavitationThrustLoss { factor: -0.1 }),
            Err(EngineError::InvalidFault { .. })
        ));
        assert!(matches!(
            e.inject_fault(EngineFault::CavitationThrustLoss { factor: 1.1 }),
            Err(EngineError::InvalidFault { .. })
        ));
    }

    // -----------------------------------------------------------------
    // Determinism / bit-stability
    // -----------------------------------------------------------------

    #[test]
    fn liquid_engine_step_is_bit_stable_across_two_invocations() {
        let mut a = fresh_engine();
        let mut b = fresh_engine();
        let cmd = EngineCommand {
            throttle_unit: 0.7,
            gimbal_pitch_rad: 0.03,
            gimbal_yaw_rad: -0.02,
            ignite: true,
            shutdown: false,
        };
        a.apply_command(cmd).unwrap();
        b.apply_command(cmd).unwrap();
        for _ in 0..150 {
            let sa = a.step(dt()).unwrap();
            let sb = b.step(dt()).unwrap();
            assert_eq!(sa.thrust_body.x.to_bits(), sb.thrust_body.x.to_bits());
            assert_eq!(sa.thrust_body.y.to_bits(), sb.thrust_body.y.to_bits());
            assert_eq!(sa.thrust_body.z.to_bits(), sb.thrust_body.z.to_bits());
            assert_eq!(
                sa.mass_flow_kg_per_s.to_bits(),
                sb.mass_flow_kg_per_s.to_bits()
            );
            assert_eq!(sa.consumed_kg.to_bits(), sb.consumed_kg.to_bits());
        }
    }

    // -----------------------------------------------------------------
    // dt validation
    // -----------------------------------------------------------------

    #[test]
    fn step_rejects_zero_dt() {
        let mut e = fresh_engine();
        let err = e.step(Duration::from_seconds(0.0)).unwrap_err();
        assert!(matches!(err, EngineError::InvalidDt { .. }));
    }

    #[test]
    fn step_rejects_negative_dt() {
        let mut e = fresh_engine();
        let err = e.step(Duration::from_seconds(-0.001)).unwrap_err();
        assert!(matches!(err, EngineError::InvalidDt { .. }));
    }

    #[test]
    fn apply_command_rejects_non_finite_throttle() {
        let mut e = fresh_engine();
        let err = e
            .apply_command(EngineCommand {
                throttle_unit: f64::NAN,
                gimbal_pitch_rad: 0.0,
                gimbal_yaw_rad: 0.0,
                ignite: false,
                shutdown: false,
            })
            .unwrap_err();
        assert!(matches!(err, EngineError::NonFiniteCommand { .. }));
    }

    #[test]
    fn apply_command_rejects_non_finite_gimbal() {
        let mut e = fresh_engine();
        let err = e
            .apply_command(EngineCommand {
                throttle_unit: 0.5,
                gimbal_pitch_rad: f64::INFINITY,
                gimbal_yaw_rad: 0.0,
                ignite: false,
                shutdown: false,
            })
            .unwrap_err();
        assert!(matches!(err, EngineError::NonFiniteCommand { .. }));
    }
}
