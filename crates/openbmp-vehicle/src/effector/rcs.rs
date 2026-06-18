//! Reaction-control-system pulse primitives.
//!
//! This module is the WP-09.2 RCS substrate: a deterministic
//! minimum-impulse-bit quantizer plus a pulse-width pulse-frequency
//! Schmitt prefilter, scalar direct-torque pulse effector, and direct-torque
//! thruster-bank effector with per-thruster geometry allocation plus simple
//! blowdown scaling.

use nalgebra::Vector3;
use openbmp_core::{Duration, EffectorId};
use thiserror::Error;

use super::{ControlEffector, EffectorError, EffectorFault, EffectorLimits, EffectorState};

/// Pulse sign emitted by an RCS prefilter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PulsePolarity {
    /// Fire the negative-direction thruster.
    Negative,
    /// No pulse.
    Off,
    /// Fire the positive-direction thruster.
    Positive,
}

impl PulsePolarity {
    /// Signed scalar for this polarity.
    #[must_use]
    pub const fn sign(self) -> f64 {
        match self {
            Self::Negative => -1.0,
            Self::Off => 0.0,
            Self::Positive => 1.0,
        }
    }

    /// `true` when this polarity commands a thruster on.
    #[must_use]
    pub const fn is_firing(self) -> bool {
        !matches!(self, Self::Off)
    }

    fn from_signed(value: f64) -> Self {
        if value > 0.0 {
            Self::Positive
        } else if value < 0.0 {
            Self::Negative
        } else {
            Self::Off
        }
    }
}

/// Minimum impulse bit plus nominal thruster level.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcsMinimumImpulseBit {
    /// Minimum non-zero impulse, N*s.
    pub minimum_impulse_n_s: f64,
    /// Nominal thruster force used to convert impulse to fixed-step duty.
    pub nominal_thrust_n: f64,
}

impl RcsMinimumImpulseBit {
    /// Construct a minimum-impulse-bit quantizer.
    ///
    /// # Errors
    ///
    /// Returns [`RcsError::InvalidParameter`] when either scalar is not finite
    /// and strictly positive.
    pub fn new(minimum_impulse_n_s: f64, nominal_thrust_n: f64) -> Result<Self, RcsError> {
        require_positive_finite("minimum_impulse_n_s", minimum_impulse_n_s)?;
        require_positive_finite("nominal_thrust_n", nominal_thrust_n)?;
        Ok(Self {
            minimum_impulse_n_s,
            nominal_thrust_n,
        })
    }

    /// Quantize a signed requested impulse into a fixed-step pulse.
    ///
    /// Zero request produces zero output. Any non-zero request with magnitude
    /// below `minimum_impulse_n_s` is floored to the MIB with preserved sign;
    /// requests above the MIB pass through linearly. The emitted average
    /// thrust is bounded by `nominal_thrust_n`, so a pulse that cannot fit into
    /// the current fixed step fails closed.
    ///
    /// # Errors
    ///
    /// Returns [`RcsError::NonFiniteImpulseRequest`] for non-finite requests,
    /// [`RcsError::InvalidDt`] for invalid `dt`, or
    /// [`RcsError::StepCapacityExceeded`] when the requested/floored impulse
    /// exceeds `nominal_thrust_n * dt`.
    pub fn pulse_for_requested_impulse(
        self,
        requested_impulse_n_s: f64,
        dt: Duration,
    ) -> Result<RcsPulse, RcsError> {
        if !requested_impulse_n_s.is_finite() {
            return Err(RcsError::NonFiniteImpulseRequest {
                value: requested_impulse_n_s,
            });
        }
        let dt_s = require_valid_dt(dt)?;
        if requested_impulse_n_s == 0.0 {
            return Ok(RcsPulse::zero(requested_impulse_n_s));
        }

        let capacity_n_s = self.nominal_thrust_n * dt_s;
        let requested_abs = requested_impulse_n_s.abs();
        let actual_abs = requested_abs.max(self.minimum_impulse_n_s);
        if actual_abs > capacity_n_s + 1.0e-12 * capacity_n_s.max(1.0) {
            return Err(RcsError::StepCapacityExceeded {
                requested_impulse_n_s: actual_abs.copysign(requested_impulse_n_s),
                capacity_n_s,
            });
        }

        let actual_impulse_n_s = actual_abs.copysign(requested_impulse_n_s);
        Ok(RcsPulse {
            requested_impulse_n_s,
            actual_impulse_n_s,
            average_thrust_n: actual_impulse_n_s / dt_s,
            on_time_s: actual_abs / self.nominal_thrust_n,
            duty_cycle: actual_abs / capacity_n_s,
            polarity: PulsePolarity::from_signed(requested_impulse_n_s),
        })
    }
}

/// One fixed-step pulse returned by [`RcsMinimumImpulseBit`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcsPulse {
    /// Signed requested impulse before quantization, N*s.
    pub requested_impulse_n_s: f64,
    /// Signed impulse emitted after MIB quantization, N*s.
    pub actual_impulse_n_s: f64,
    /// Signed average thrust over the fixed step, N.
    pub average_thrust_n: f64,
    /// Thruster on-time inside the fixed step, s.
    pub on_time_s: f64,
    /// Fixed-step duty cycle in `[0, 1]`.
    pub duty_cycle: f64,
    /// Pulse polarity.
    pub polarity: PulsePolarity,
}

impl RcsPulse {
    fn zero(requested_impulse_n_s: f64) -> Self {
        Self {
            requested_impulse_n_s,
            actual_impulse_n_s: 0.0,
            average_thrust_n: 0.0,
            on_time_s: 0.0,
            duty_cycle: 0.0,
            polarity: PulsePolarity::Off,
        }
    }
}

/// Pulse-width pulse-frequency prefilter parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PwpfParams {
    /// Prefilter gain.
    pub gain: f64,
    /// First-order prefilter time constant, s.
    pub time_constant_s: f64,
    /// Schmitt turn-on threshold, positive command units.
    pub on_threshold: f64,
    /// Schmitt turn-off threshold, non-negative and smaller than
    /// `on_threshold`.
    pub off_threshold: f64,
}

impl PwpfParams {
    fn require_valid(self) -> Result<(), RcsError> {
        require_positive_finite("gain", self.gain)?;
        require_positive_finite("time_constant_s", self.time_constant_s)?;
        require_positive_finite("on_threshold", self.on_threshold)?;
        if !self.off_threshold.is_finite() || self.off_threshold < 0.0 {
            return Err(RcsError::InvalidParameter {
                field: "off_threshold",
                reason: "must be finite and non-negative",
            });
        }
        if self.off_threshold >= self.on_threshold {
            return Err(RcsError::InvalidParameter {
                field: "off_threshold",
                reason: "must be smaller than on_threshold",
            });
        }
        Ok(())
    }
}

/// Deterministic PWPF Schmitt prefilter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PwpfModulator {
    params: PwpfParams,
    filtered_command: f64,
    polarity: PulsePolarity,
}

impl PwpfModulator {
    /// Construct a PWPF modulator at rest.
    ///
    /// # Errors
    ///
    /// Returns [`RcsError::InvalidParameter`] when the parameters are invalid.
    pub fn new(params: PwpfParams) -> Result<Self, RcsError> {
        params.require_valid()?;
        Ok(Self {
            params,
            filtered_command: 0.0,
            polarity: PulsePolarity::Off,
        })
    }

    /// Current filtered command.
    #[must_use]
    pub const fn filtered_command(&self) -> f64 {
        self.filtered_command
    }

    /// Current Schmitt polarity.
    #[must_use]
    pub const fn polarity(&self) -> PulsePolarity {
        self.polarity
    }

    /// Advance the PWPF prefilter by one fixed step.
    ///
    /// The first-order filter uses the exact zero-order-hold update:
    /// `x <- K_m*u + (x - K_m*u)*exp(-dt/tau_m)`, followed by a symmetric
    /// Schmitt trigger with hysteresis thresholds.
    ///
    /// # Errors
    ///
    /// Returns [`RcsError::NonFiniteCommand`] for non-finite commands or
    /// [`RcsError::InvalidDt`] for invalid `dt`.
    pub fn step(&mut self, command: f64, dt: Duration) -> Result<PwpfStep, RcsError> {
        if !command.is_finite() {
            return Err(RcsError::NonFiniteCommand { value: command });
        }
        let dt_s = require_valid_dt(dt)?;
        let target = self.params.gain * command;
        if !target.is_finite() {
            return Err(RcsError::NonFiniteCommand { value: command });
        }
        let alpha = (-dt_s / self.params.time_constant_s).exp();
        self.filtered_command = target + (self.filtered_command - target) * alpha;
        self.polarity = schmitt(self.polarity, self.filtered_command, self.params);
        Ok(PwpfStep {
            command,
            filtered_command: self.filtered_command,
            polarity: self.polarity,
        })
    }
}

/// One PWPF prefilter step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PwpfStep {
    /// Raw command passed to the prefilter.
    pub command: f64,
    /// Filtered command after the ZOH update.
    pub filtered_command: f64,
    /// Schmitt output polarity after hysteresis.
    pub polarity: PulsePolarity,
}

/// Parameters for a scalar RCS pulse effector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcsPulseEffectorParams {
    /// Minimum non-zero impulse, command-units*s.
    pub minimum_impulse_n_s: f64,
    /// Nominal on-pulse command magnitude.
    pub nominal_thrust_n: f64,
    /// Optional PWPF prefilter. When absent, the raw command is directly
    /// quantized by the minimum-impulse-bit rule.
    pub pwpf: Option<PwpfParams>,
}

/// Scalar RCS pulse effector implementing [`ControlEffector`].
///
/// The input command and output actual share the same scalar command units as
/// the existing effector path. For `direct_torque` effectors, the runner's
/// existing moment adapter still multiplies `actual` by the configured
/// effectiveness.
#[derive(Clone, Debug, PartialEq)]
pub struct RcsPulseEffector {
    id: EffectorId,
    limits: EffectorLimits,
    dt: Duration,
    mib: RcsMinimumImpulseBit,
    pwpf: Option<PwpfModulator>,
    actual: f64,
    last_state: EffectorState,
    fault: Option<EffectorFault>,
    fault_elapsed_s: f64,
}

/// Pressure-fed blowdown model for an RCS thruster.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcsBlowdownParams {
    /// Initial feed pressure, Pa.
    pub initial_pressure_pa: f64,
    /// Pressure floor, Pa.
    pub minimum_pressure_pa: f64,
    /// Usable cumulative thruster impulse from initial to floor pressure,
    /// N*s.
    pub usable_impulse_n_s: f64,
    /// Shape exponent applied to remaining impulse fraction.
    pub pressure_exponent: f64,
}

impl RcsBlowdownParams {
    fn require_valid(self) -> Result<(), RcsError> {
        require_positive_finite("initial_pressure_pa", self.initial_pressure_pa)?;
        require_positive_finite("minimum_pressure_pa", self.minimum_pressure_pa)?;
        if self.minimum_pressure_pa > self.initial_pressure_pa {
            return Err(RcsError::InvalidParameter {
                field: "minimum_pressure_pa",
                reason: "must be less than or equal to initial_pressure_pa",
            });
        }
        require_positive_finite("usable_impulse_n_s", self.usable_impulse_n_s)?;
        require_positive_finite("pressure_exponent", self.pressure_exponent)?;
        Ok(())
    }

    fn scale_for_cumulative_impulse(self, cumulative_impulse_n_s: f64) -> Result<f64, RcsError> {
        if !cumulative_impulse_n_s.is_finite() || cumulative_impulse_n_s < 0.0 {
            return Err(RcsError::InvalidParameter {
                field: "cumulative_impulse_n_s",
                reason: "must be finite and non-negative",
            });
        }
        let remaining = (1.0 - cumulative_impulse_n_s / self.usable_impulse_n_s).clamp(0.0, 1.0);
        let pressure_pa = self.minimum_pressure_pa
            + (self.initial_pressure_pa - self.minimum_pressure_pa)
                * remaining.powf(self.pressure_exponent);
        Ok(pressure_pa / self.initial_pressure_pa)
    }

    fn minimum_scale(self) -> f64 {
        self.minimum_pressure_pa / self.initial_pressure_pa
    }
}

/// One physical thruster in a direct-torque RCS bank.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcsThrusterConfig {
    /// Thruster mount position in body axes, m.
    pub position_body_m: [f64; 3],
    /// Thruster force direction in body axes. Normalized at construction.
    pub direction_body: [f64; 3],
    /// Minimum non-zero impulse, N*s.
    pub minimum_impulse_n_s: f64,
    /// Initial nominal thrust, N.
    pub nominal_thrust_n: f64,
    /// Optional pressure-fed blowdown model.
    pub blowdown: Option<RcsBlowdownParams>,
}

/// Parameters for a direct-torque physical RCS bank.
#[derive(Clone, Debug, PartialEq)]
pub struct RcsThrusterBankEffectorParams {
    /// Body-axis index the enclosing direct-torque effector drives.
    pub axis_index: usize,
    /// Direct-torque adapter effectiveness, N*m per command unit.
    pub command_effectiveness_n_m_per_unit: f64,
    /// Bank thrusters in deterministic allocation order.
    pub thrusters: Vec<RcsThrusterConfig>,
    /// Optional bank-level PWPF prefilter.
    pub pwpf: Option<PwpfParams>,
}

/// Coupled multi-axis RCS allocation result for one thruster.
#[derive(Clone, Debug, PartialEq)]
pub struct RcsCoupledThrusterPulse {
    /// Thruster index in declaration order.
    pub thruster_index: usize,
    /// Requested force impulse before MIB quantization, N*s.
    pub requested_impulse_n_s: f64,
    /// Actual force impulse after MIB quantization, N*s.
    pub actual_impulse_n_s: f64,
    /// Body torque impulse contributed by this pulse, N*m*s.
    pub torque_impulse_body_n_m_s: [f64; 3],
    /// Fixed-step duty cycle in `[0, 1]`.
    pub duty_cycle: f64,
    /// Thruster on-time inside the fixed step, s.
    pub on_time_s: f64,
}

/// Direct-torque RCS bank pulse telemetry for one thruster.
#[derive(Clone, Debug, PartialEq)]
pub struct RcsThrusterBankPulse {
    /// Thruster index in declaration order.
    pub thruster_index: usize,
    /// Requested force impulse before MIB quantization, N*s.
    pub requested_impulse_n_s: f64,
    /// Actual force impulse after MIB quantization, N*s.
    pub actual_impulse_n_s: f64,
    /// Fixed-step duty cycle in `[0, 1]`.
    pub duty_cycle: f64,
    /// Thruster on-time inside the fixed step, s.
    pub on_time_s: f64,
}

/// Coupled multi-axis RCS allocation summary.
#[derive(Clone, Debug, PartialEq)]
pub struct RcsCoupledAllocation {
    /// Requested body torque impulse, N*m*s.
    pub requested_torque_impulse_body_n_m_s: [f64; 3],
    /// Body torque impulse actually emitted after allocation and MIB
    /// quantization, N*m*s.
    pub actual_torque_impulse_body_n_m_s: [f64; 3],
    /// Remaining unallocated request, N*m*s.
    pub residual_torque_impulse_body_n_m_s: [f64; 3],
    /// Per-thruster pulse list in deterministic declaration order.
    pub pulses: Vec<RcsCoupledThrusterPulse>,
    /// `true` when declared thruster authority could not cover the requested
    /// vector.
    pub saturated: bool,
    /// `true` when at least one emitted pulse differs from the requested
    /// projection because of MIB flooring.
    pub quantized: bool,
}

/// Deterministic coupled multi-axis RCS allocator.
///
/// The allocator greedily projects the residual requested body torque impulse
/// onto each declared thruster's physical torque direction `r x F`, applies
/// that thruster's MIB and optional blowdown state, and subtracts the emitted
/// torque impulse from the residual. It is intentionally deterministic and
/// allocation-method explicit; it does not claim optimal fuel use.
#[derive(Clone, Debug, PartialEq)]
pub struct RcsCoupledAllocator {
    dt: Duration,
    thrusters: Vec<RcsCoupledThruster>,
}

#[derive(Clone, Debug, PartialEq)]
struct RcsBankThruster {
    torque_per_newton_axis: f64,
    minimum_impulse_n_s: f64,
    nominal_thrust_n: f64,
    feed_pressure_scale: f64,
    blowdown: Option<RcsBlowdownParams>,
    cumulative_impulse_n_s: f64,
}

#[derive(Clone, Debug, PartialEq)]
struct RcsCoupledThruster {
    torque_per_newton_body: Vector3<f64>,
    minimum_impulse_n_s: f64,
    nominal_thrust_n: f64,
    feed_pressure_scale: f64,
    blowdown: Option<RcsBlowdownParams>,
    cumulative_impulse_n_s: f64,
}

impl RcsCoupledThruster {
    fn current_nominal_thrust_n(&self) -> Result<f64, EffectorError> {
        let scale = match self.blowdown {
            Some(blowdown) => blowdown
                .scale_for_cumulative_impulse(self.cumulative_impulse_n_s)
                .map_err(|_| EffectorError::InvalidLimits {
                    reason: "RCS coupled allocator blowdown state is invalid",
                })?,
            None => 1.0,
        };
        Ok(self.nominal_thrust_n * scale * self.feed_pressure_scale)
    }

    fn torque_per_newton_norm(&self) -> f64 {
        self.torque_per_newton_body.norm()
    }

    fn torque_direction(&self) -> Vector3<f64> {
        self.torque_per_newton_body / self.torque_per_newton_norm()
    }

    fn capacity_torque_impulse_n_m_s(&self, dt_s: f64) -> Result<f64, EffectorError> {
        Ok(self.torque_per_newton_norm() * self.current_nominal_thrust_n()? * dt_s)
    }

    fn pulse_for_torque_request(
        &mut self,
        requested_torque_impulse_n_m_s: f64,
        dt: Duration,
    ) -> Result<RcsPulse, EffectorError> {
        let force_impulse_n_s = requested_torque_impulse_n_m_s / self.torque_per_newton_norm();
        let nominal_thrust_n = self.current_nominal_thrust_n()?;
        let pulse = RcsMinimumImpulseBit::new(self.minimum_impulse_n_s, nominal_thrust_n)
            .map_err(|_| EffectorError::InvalidLimits {
                reason: "RCS coupled allocator MIB parameters are invalid",
            })?
            .pulse_for_requested_impulse(force_impulse_n_s, dt)
            .map_err(|_| EffectorError::InvalidLimits {
                reason: "RCS coupled allocator pulse exceeded fixed-step capacity",
            })?;
        self.cumulative_impulse_n_s += pulse.actual_impulse_n_s.abs();
        Ok(pulse)
    }
}

impl RcsCoupledAllocator {
    /// Construct a coupled multi-axis allocator.
    ///
    /// # Errors
    ///
    /// Returns [`EffectorError`] when `dt`, thruster geometry, MIB, thrust, or
    /// blowdown parameters are invalid.
    pub fn new(thrusters: Vec<RcsThrusterConfig>, dt: Duration) -> Result<Self, EffectorError> {
        let dt_s = require_effector_dt(dt)?;
        if thrusters.is_empty() {
            return Err(EffectorError::InvalidLimits {
                reason: "RCS coupled allocator requires at least one thruster",
            });
        }
        let mut built = Vec::with_capacity(thrusters.len());
        for config in thrusters {
            built.push(build_coupled_thruster(config, dt_s)?);
        }
        Ok(Self {
            dt,
            thrusters: built,
        })
    }

    /// Allocate a requested body torque impulse across the thruster bank.
    ///
    /// # Errors
    ///
    /// Returns [`EffectorError`] when any request component is non-finite or
    /// internal pulse evaluation fails closed.
    pub fn allocate(
        &mut self,
        requested_torque_impulse_body_n_m_s: [f64; 3],
    ) -> Result<RcsCoupledAllocation, EffectorError> {
        require_vec3_finite(
            requested_torque_impulse_body_n_m_s,
            "RCS coupled allocator requested torque impulse must be finite",
        )?;
        let requested = Vector3::from(requested_torque_impulse_body_n_m_s);
        let mut requested_residual = requested;
        let mut actual = Vector3::zeros();
        let mut pulses = Vec::new();
        let mut quantized = false;
        let dt_s = self.dt.as_seconds();

        for (thruster_index, thruster) in self.thrusters.iter_mut().enumerate() {
            if requested_residual.norm() <= 1.0e-12 {
                break;
            }
            let direction = thruster.torque_direction();
            let projected = requested_residual.dot(&direction);
            if projected <= 0.0 {
                continue;
            }
            let capacity = thruster.capacity_torque_impulse_n_m_s(dt_s)?;
            if capacity <= 0.0 {
                continue;
            }
            let requested_torque = projected.min(capacity);
            requested_residual -= direction * requested_torque;
            let pulse = thruster.pulse_for_torque_request(requested_torque, self.dt)?;
            let actual_torque_magnitude =
                pulse.actual_impulse_n_s * thruster.torque_per_newton_norm();
            let contribution = direction * actual_torque_magnitude;
            actual += contribution;
            quantized |= (actual_torque_magnitude - requested_torque).abs()
                > 1.0e-12 * requested_torque.max(1.0);
            pulses.push(RcsCoupledThrusterPulse {
                thruster_index,
                requested_impulse_n_s: pulse.requested_impulse_n_s,
                actual_impulse_n_s: pulse.actual_impulse_n_s,
                torque_impulse_body_n_m_s: [contribution.x, contribution.y, contribution.z],
                duty_cycle: pulse.duty_cycle,
                on_time_s: pulse.on_time_s,
            });
        }

        let residual = requested - actual;
        Ok(RcsCoupledAllocation {
            requested_torque_impulse_body_n_m_s,
            actual_torque_impulse_body_n_m_s: [actual.x, actual.y, actual.z],
            residual_torque_impulse_body_n_m_s: [residual.x, residual.y, residual.z],
            pulses,
            saturated: requested_residual.norm() > 1.0e-9,
            quantized,
        })
    }

    /// Apply live per-thruster feed pressure scales.
    ///
    /// # Errors
    ///
    /// Returns [`EffectorError`] when the scale slice length does not match
    /// the bank or any scale is non-finite/negative.
    pub fn set_thruster_feed_pressure_scales(
        &mut self,
        scales: &[f64],
    ) -> Result<(), EffectorError> {
        if scales.len() != self.thrusters.len() {
            return Err(EffectorError::InvalidLimits {
                reason: "RCS coupled allocator feed scale count must match thruster count",
            });
        }
        for (thruster, scale) in self.thrusters.iter_mut().zip(scales.iter().copied()) {
            if !scale.is_finite() || scale < 0.0 {
                return Err(EffectorError::InvalidLimits {
                    reason: "RCS coupled allocator feed scales must be finite and non-negative",
                });
            }
            thruster.feed_pressure_scale = scale;
        }
        Ok(())
    }
}

impl RcsBankThruster {
    fn current_nominal_thrust_n(&self) -> Result<f64, EffectorError> {
        let scale = match self.blowdown {
            Some(blowdown) => blowdown
                .scale_for_cumulative_impulse(self.cumulative_impulse_n_s)
                .map_err(|_| EffectorError::InvalidLimits {
                    reason: "RCS blowdown state is invalid",
                })?,
            None => 1.0,
        };
        Ok(self.nominal_thrust_n * scale * self.feed_pressure_scale)
    }

    fn axis_sign(&self) -> f64 {
        self.torque_per_newton_axis.signum()
    }

    fn torque_per_newton_abs(&self) -> f64 {
        self.torque_per_newton_axis.abs()
    }

    fn capacity_torque_impulse_n_m_s(&self, dt_s: f64) -> Result<f64, EffectorError> {
        Ok(self.torque_per_newton_abs() * self.current_nominal_thrust_n()? * dt_s)
    }

    fn pulse_for_torque_request(
        &mut self,
        requested_torque_impulse_n_m_s: f64,
        dt: Duration,
    ) -> Result<RcsPulse, EffectorError> {
        let force_impulse_n_s = requested_torque_impulse_n_m_s / self.torque_per_newton_abs();
        let nominal_thrust_n = self.current_nominal_thrust_n()?;
        let pulse = RcsMinimumImpulseBit::new(self.minimum_impulse_n_s, nominal_thrust_n)
            .map_err(|_| EffectorError::InvalidLimits {
                reason: "RCS bank MIB parameters are invalid",
            })?
            .pulse_for_requested_impulse(force_impulse_n_s, dt)
            .map_err(|_| EffectorError::InvalidLimits {
                reason: "RCS bank pulse exceeded fixed-step capacity",
            })?;
        self.cumulative_impulse_n_s += pulse.actual_impulse_n_s.abs();
        Ok(pulse)
    }
}

/// Direct-torque RCS bank implementing [`ControlEffector`].
///
/// The bank maps scalar direct-torque commands into a requested body-axis
/// torque using `command_effectiveness_n_m_per_unit`, allocates that request to
/// declared physical thrusters by deterministic priority order, applies each
/// thruster's MIB and optional blowdown scaling, then reports the average
/// torque back as equivalent direct-torque command units. The existing
/// [`crate::DirectTorqueMomentAdapter`] can therefore consume the output through
/// the same scalar snapshot channel as continuous direct-torque effectors.
#[derive(Clone, Debug, PartialEq)]
pub struct RcsThrusterBankEffector {
    id: EffectorId,
    limits: EffectorLimits,
    dt: Duration,
    command_effectiveness_n_m_per_unit: f64,
    thrusters: Vec<RcsBankThruster>,
    pwpf: Option<PwpfModulator>,
    actual: f64,
    last_state: EffectorState,
    last_pulses: Vec<RcsThrusterBankPulse>,
    fault: Option<EffectorFault>,
    fault_elapsed_s: f64,
}

impl RcsThrusterBankEffector {
    /// Construct a direct-torque RCS bank effector.
    ///
    /// # Errors
    ///
    /// Returns [`EffectorError`] when limits, parameters, `dt`, geometry, or
    /// initial position are invalid.
    pub fn new(
        id: EffectorId,
        limits: EffectorLimits,
        params: RcsThrusterBankEffectorParams,
        dt: Duration,
        initial_position: f64,
    ) -> Result<Self, EffectorError> {
        let dt_s = require_effector_dt(dt)?;
        limits.require_valid(dt)?;
        validate_initial_position(initial_position, limits)?;
        if params.axis_index >= 3 {
            return Err(EffectorError::InvalidLimits {
                reason: "RCS bank axis_index must be 0, 1, or 2",
            });
        }
        if !params.command_effectiveness_n_m_per_unit.is_finite()
            || params.command_effectiveness_n_m_per_unit <= 0.0
        {
            return Err(EffectorError::InvalidLimits {
                reason: "RCS bank command effectiveness must be finite and strictly positive",
            });
        }
        if params.thrusters.is_empty() {
            return Err(EffectorError::InvalidLimits {
                reason: "RCS bank requires at least one thruster",
            });
        }

        let mut thrusters = Vec::with_capacity(params.thrusters.len());
        for config in params.thrusters {
            thrusters.push(build_bank_thruster(config, params.axis_index, dt_s)?);
        }
        validate_bank_authority(&thrusters, limits)?;

        let pwpf = params
            .pwpf
            .map(PwpfModulator::new)
            .transpose()
            .map_err(|_| EffectorError::InvalidLimits {
                reason: "PWPF parameters are invalid",
            })?;
        let last_state = EffectorState::at_rest(initial_position);
        Ok(Self {
            id,
            limits,
            dt,
            command_effectiveness_n_m_per_unit: params.command_effectiveness_n_m_per_unit,
            thrusters,
            pwpf,
            actual: initial_position,
            last_state,
            last_pulses: Vec::new(),
            fault: None,
            fault_elapsed_s: 0.0,
        })
    }

    fn validate_fault(&self, fault: EffectorFault) -> Result<(), EffectorError> {
        validate_fault_against_limits(fault, self.limits)
    }

    fn allocate_torque_impulse(
        &mut self,
        requested_torque_impulse_n_m_s: f64,
        dt: Duration,
    ) -> Result<RcsBankAllocation, EffectorError> {
        if requested_torque_impulse_n_m_s == 0.0 {
            return Ok(RcsBankAllocation::zero());
        }
        let dt_s = dt.as_seconds();
        let sign = requested_torque_impulse_n_m_s.signum();
        let target_abs = requested_torque_impulse_n_m_s.abs();
        let capacity_abs = self.bank_capacity_torque_impulse_n_m_s(sign, dt_s)?;
        let saturated = target_abs > capacity_abs + 1.0e-12 * capacity_abs.max(1.0);
        let target_abs = target_abs.min(capacity_abs);

        let mut actual_abs = 0.0;
        let mut pulses = Vec::new();
        for (thruster_index, thruster) in self.thrusters.iter_mut().enumerate() {
            if !same_nonzero_sign(thruster.axis_sign(), sign) || actual_abs >= target_abs {
                continue;
            }
            let remaining = (target_abs - actual_abs).max(0.0);
            let capacity = thruster.capacity_torque_impulse_n_m_s(dt_s)?;
            if capacity <= 0.0 {
                continue;
            }
            let requested = remaining.min(capacity);
            let pulse = thruster.pulse_for_torque_request(requested, dt)?;
            actual_abs += pulse.actual_impulse_n_s.abs() * thruster.torque_per_newton_abs();
            pulses.push(RcsThrusterBankPulse {
                thruster_index,
                requested_impulse_n_s: pulse.requested_impulse_n_s,
                actual_impulse_n_s: pulse.actual_impulse_n_s,
                duty_cycle: pulse.duty_cycle,
                on_time_s: pulse.on_time_s,
            });
        }

        Ok(RcsBankAllocation {
            actual_torque_impulse_n_m_s: sign * actual_abs,
            quantized: (actual_abs - target_abs).abs() > 1.0e-12 * target_abs.max(1.0),
            saturated,
            pulses,
        })
    }

    /// Apply live per-thruster feed pressure scales.
    ///
    /// # Errors
    ///
    /// Returns [`EffectorError`] when the scale slice length does not match
    /// the bank or any scale is non-finite/negative.
    pub fn set_thruster_feed_pressure_scales(
        &mut self,
        scales: &[f64],
    ) -> Result<(), EffectorError> {
        if scales.len() != self.thrusters.len() {
            return Err(EffectorError::InvalidLimits {
                reason: "RCS bank feed scale count must match thruster count",
            });
        }
        for (thruster, scale) in self.thrusters.iter_mut().zip(scales.iter().copied()) {
            if !scale.is_finite() || scale < 0.0 {
                return Err(EffectorError::InvalidLimits {
                    reason: "RCS bank feed scales must be finite and non-negative",
                });
            }
            thruster.feed_pressure_scale = scale;
        }
        Ok(())
    }

    /// Per-thruster pulses emitted by the most recent step.
    #[must_use]
    pub fn last_thruster_pulses(&self) -> &[RcsThrusterBankPulse] {
        &self.last_pulses
    }

    fn bank_capacity_torque_impulse_n_m_s(
        &self,
        sign: f64,
        dt_s: f64,
    ) -> Result<f64, EffectorError> {
        let mut capacity = 0.0;
        for thruster in &self.thrusters {
            if same_nonzero_sign(thruster.axis_sign(), sign) {
                capacity += thruster.capacity_torque_impulse_n_m_s(dt_s)?;
            }
        }
        Ok(capacity)
    }

    fn fault_state(&mut self, cmd: f64, fault: EffectorFault, dt_s: f64) -> Option<EffectorState> {
        match fault {
            EffectorFault::Jam { at } => {
                self.actual = at;
            }
            EffectorFault::Hardover { to } => {
                self.actual = to.clamp(self.limits.min, self.limits.max);
            }
            EffectorFault::Runaway { rate_per_s } => {
                let proposed = self.actual + rate_per_s * dt_s;
                self.actual = proposed.clamp(self.limits.min, self.limits.max);
                let state = EffectorState {
                    commanded: cmd,
                    actual: self.actual,
                    saturated: proposed < self.limits.min || proposed > self.limits.max,
                    rate_limited: false,
                    fault: Some(fault),
                };
                self.last_state = state;
                return Some(state);
            }
            EffectorFault::ReducedRate { .. } => return None,
            EffectorFault::Oscillatory { .. } => return None,
        }
        let state = EffectorState {
            commanded: cmd,
            actual: self.actual,
            saturated: self.actual <= self.limits.min || self.actual >= self.limits.max,
            rate_limited: true,
            fault: Some(fault),
        };
        self.last_state = state;
        Some(state)
    }
}

#[derive(Clone, Debug, PartialEq)]
struct RcsBankAllocation {
    actual_torque_impulse_n_m_s: f64,
    quantized: bool,
    saturated: bool,
    pulses: Vec<RcsThrusterBankPulse>,
}

impl RcsBankAllocation {
    fn zero() -> Self {
        Self {
            actual_torque_impulse_n_m_s: 0.0,
            quantized: false,
            saturated: false,
            pulses: Vec::new(),
        }
    }
}

fn require_effector_dt(dt: Duration) -> Result<f64, EffectorError> {
    let dt_s = dt.as_seconds();
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(EffectorError::InvalidLimits {
            reason: "dt must be finite and strictly positive",
        });
    }
    Ok(dt_s)
}

fn validate_initial_position(
    initial_position: f64,
    limits: EffectorLimits,
) -> Result<(), EffectorError> {
    if !initial_position.is_finite() {
        return Err(EffectorError::NonFiniteCommand {
            value: initial_position,
        });
    }
    if initial_position < limits.min || initial_position > limits.max {
        return Err(EffectorError::InvalidLimits {
            reason: "initial_position must be within [min, max]",
        });
    }
    Ok(())
}

fn validate_fault_against_limits(
    fault: EffectorFault,
    limits: EffectorLimits,
) -> Result<(), EffectorError> {
    match fault {
        EffectorFault::Jam { at } => {
            if !at.is_finite() {
                return Err(EffectorError::InvalidFault {
                    reason: "Jam.at must be finite",
                });
            }
            if at < limits.min || at > limits.max {
                return Err(EffectorError::InvalidFault {
                    reason: "Jam.at must lie within [min, max]",
                });
            }
        }
        EffectorFault::Runaway { rate_per_s } => {
            if !rate_per_s.is_finite() {
                return Err(EffectorError::InvalidFault {
                    reason: "Runaway.rate_per_s must be finite",
                });
            }
        }
        EffectorFault::ReducedRate { factor } => {
            if !factor.is_finite() {
                return Err(EffectorError::InvalidFault {
                    reason: "ReducedRate.factor must be finite",
                });
            }
            if !(0.0..=1.0).contains(&factor) {
                return Err(EffectorError::InvalidFault {
                    reason: "ReducedRate.factor must lie in [0, 1]",
                });
            }
        }
        EffectorFault::Hardover { to } => {
            if !to.is_finite() {
                return Err(EffectorError::InvalidFault {
                    reason: "Hardover.to must be finite",
                });
            }
            if to < limits.min || to > limits.max {
                return Err(EffectorError::InvalidFault {
                    reason: "Hardover.to must lie within [min, max]",
                });
            }
        }
        EffectorFault::Oscillatory {
            amplitude,
            frequency_hz,
            phase_rad,
        } => {
            if !amplitude.is_finite() || amplitude < 0.0 {
                return Err(EffectorError::InvalidFault {
                    reason: "Oscillatory.amplitude must be finite and non-negative",
                });
            }
            if !frequency_hz.is_finite() || frequency_hz <= 0.0 {
                return Err(EffectorError::InvalidFault {
                    reason: "Oscillatory.frequency_hz must be finite and strictly positive",
                });
            }
            if !phase_rad.is_finite() {
                return Err(EffectorError::InvalidFault {
                    reason: "Oscillatory.phase_rad must be finite",
                });
            }
        }
    }
    Ok(())
}

fn build_bank_thruster(
    config: RcsThrusterConfig,
    axis_index: usize,
    dt_s: f64,
) -> Result<RcsBankThruster, EffectorError> {
    require_vec3_finite(config.position_body_m, "RCS bank position_body_m")?;
    require_vec3_finite(config.direction_body, "RCS bank direction_body")?;
    RcsMinimumImpulseBit::new(config.minimum_impulse_n_s, config.nominal_thrust_n).map_err(
        |_| EffectorError::InvalidLimits {
            reason: "RCS bank MIB parameters must be finite and strictly positive",
        },
    )?;
    if let Some(blowdown) = config.blowdown {
        blowdown
            .require_valid()
            .map_err(|_| EffectorError::InvalidLimits {
                reason: "RCS bank blowdown parameters are invalid",
            })?;
        let minimum_nominal = config.nominal_thrust_n * blowdown.minimum_scale();
        if config.minimum_impulse_n_s > minimum_nominal * dt_s {
            return Err(EffectorError::InvalidLimits {
                reason: "RCS bank MIB must fit within one dt at minimum blowdown pressure",
            });
        }
    } else if config.minimum_impulse_n_s > config.nominal_thrust_n * dt_s {
        return Err(EffectorError::InvalidLimits {
            reason: "RCS bank MIB must fit within one dt at nominal thrust",
        });
    }

    let position = Vector3::from(config.position_body_m);
    let direction = Vector3::from(config.direction_body);
    let direction_norm = direction.norm();
    if direction_norm <= 1.0e-12 {
        return Err(EffectorError::InvalidLimits {
            reason: "RCS bank direction_body must have non-zero norm",
        });
    }
    let torque_per_newton_axis = position.cross(&(direction / direction_norm))[axis_index];
    if torque_per_newton_axis.abs() <= 1.0e-12 {
        return Err(EffectorError::InvalidLimits {
            reason: "RCS bank thruster must produce non-zero torque about the declared axis",
        });
    }
    Ok(RcsBankThruster {
        torque_per_newton_axis,
        minimum_impulse_n_s: config.minimum_impulse_n_s,
        nominal_thrust_n: config.nominal_thrust_n,
        feed_pressure_scale: 1.0,
        blowdown: config.blowdown,
        cumulative_impulse_n_s: 0.0,
    })
}

fn build_coupled_thruster(
    config: RcsThrusterConfig,
    dt_s: f64,
) -> Result<RcsCoupledThruster, EffectorError> {
    require_vec3_finite(
        config.position_body_m,
        "RCS coupled allocator position_body_m",
    )?;
    require_vec3_finite(
        config.direction_body,
        "RCS coupled allocator direction_body",
    )?;
    RcsMinimumImpulseBit::new(config.minimum_impulse_n_s, config.nominal_thrust_n).map_err(
        |_| EffectorError::InvalidLimits {
            reason: "RCS coupled allocator MIB parameters must be finite and strictly positive",
        },
    )?;
    if let Some(blowdown) = config.blowdown {
        blowdown
            .require_valid()
            .map_err(|_| EffectorError::InvalidLimits {
                reason: "RCS coupled allocator blowdown parameters are invalid",
            })?;
        let minimum_nominal = config.nominal_thrust_n * blowdown.minimum_scale();
        if config.minimum_impulse_n_s > minimum_nominal * dt_s {
            return Err(EffectorError::InvalidLimits {
                reason: "RCS coupled allocator MIB must fit within one dt at minimum blowdown pressure",
            });
        }
    } else if config.minimum_impulse_n_s > config.nominal_thrust_n * dt_s {
        return Err(EffectorError::InvalidLimits {
            reason: "RCS coupled allocator MIB must fit within one dt at nominal thrust",
        });
    }

    let position = Vector3::from(config.position_body_m);
    let direction = Vector3::from(config.direction_body);
    let direction_norm = direction.norm();
    if direction_norm <= 1.0e-12 {
        return Err(EffectorError::InvalidLimits {
            reason: "RCS coupled allocator direction_body must have non-zero norm",
        });
    }
    let torque_per_newton_body = position.cross(&(direction / direction_norm));
    if torque_per_newton_body.norm() <= 1.0e-12 {
        return Err(EffectorError::InvalidLimits {
            reason: "RCS coupled allocator thruster must produce non-zero body torque",
        });
    }
    Ok(RcsCoupledThruster {
        torque_per_newton_body,
        minimum_impulse_n_s: config.minimum_impulse_n_s,
        nominal_thrust_n: config.nominal_thrust_n,
        feed_pressure_scale: 1.0,
        blowdown: config.blowdown,
        cumulative_impulse_n_s: 0.0,
    })
}

fn require_vec3_finite(values: [f64; 3], reason: &'static str) -> Result<(), EffectorError> {
    if values.iter().any(|value| !value.is_finite()) {
        return Err(EffectorError::InvalidLimits { reason });
    }
    Ok(())
}

fn validate_bank_authority(
    thrusters: &[RcsBankThruster],
    limits: EffectorLimits,
) -> Result<(), EffectorError> {
    let has_positive = thrusters
        .iter()
        .any(|thruster| thruster.torque_per_newton_axis > 0.0);
    let has_negative = thrusters
        .iter()
        .any(|thruster| thruster.torque_per_newton_axis < 0.0);
    if limits.max > 0.0 && !has_positive {
        return Err(EffectorError::InvalidLimits {
            reason: "RCS bank lacks positive-axis authority required by limits.max",
        });
    }
    if limits.min < 0.0 && !has_negative {
        return Err(EffectorError::InvalidLimits {
            reason: "RCS bank lacks negative-axis authority required by limits.min",
        });
    }
    Ok(())
}

fn same_nonzero_sign(a: f64, b: f64) -> bool {
    a * b > 0.0
}

impl ControlEffector for RcsThrusterBankEffector {
    fn id(&self) -> EffectorId {
        self.id
    }

    fn step(&mut self, cmd: f64, dt: Duration) -> Result<EffectorState, EffectorError> {
        let configured_s = self.dt.as_seconds();
        let got_s = dt.as_seconds();
        if (configured_s - got_s).abs() > 1.0e-12 {
            return Err(EffectorError::DtMismatch {
                configured_s,
                got_s,
            });
        }
        if !cmd.is_finite() {
            return Err(EffectorError::NonFiniteCommand { value: cmd });
        }
        let effective_cmd = if let Some(fault) = self.fault {
            if let Some(offset) = fault.oscillatory_offset(self.fault_elapsed_s) {
                let value = cmd + offset;
                if !value.is_finite() {
                    return Err(EffectorError::NonFiniteCommand { value });
                }
                self.fault_elapsed_s += got_s;
                value
            } else {
                cmd
            }
        } else {
            cmd
        };

        if let Some(fault) = self.fault
            && let Some(state) = self.fault_state(cmd, fault, got_s)
        {
            return Ok(state);
        }

        let mut saturated = effective_cmd < self.limits.min || effective_cmd > self.limits.max;
        let mut command = effective_cmd.clamp(self.limits.min, self.limits.max);
        let reduced_factor = match self.fault {
            Some(EffectorFault::ReducedRate { factor }) => factor.clamp(0.0, 1.0),
            _ => 1.0,
        };
        command *= reduced_factor;

        let mut pwpf_gated = false;
        if let Some(pwpf) = &mut self.pwpf {
            let polarity = pwpf
                .step(command, dt)
                .map_err(|_| EffectorError::InvalidLimits {
                    reason: "PWPF step failed",
                })?
                .polarity;
            let gated_command = if polarity.is_firing() {
                polarity.sign() * command.abs()
            } else {
                0.0
            };
            pwpf_gated = (gated_command - command).abs() > 1.0e-12 * command.abs().max(1.0);
            command = gated_command;
        }

        let requested_torque_impulse_n_m_s =
            command * self.command_effectiveness_n_m_per_unit * got_s;
        let allocation = self.allocate_torque_impulse(requested_torque_impulse_n_m_s, dt)?;
        self.last_pulses = allocation.pulses.clone();
        let actual_torque_n_m = allocation.actual_torque_impulse_n_m_s / got_s;
        let raw_actual = actual_torque_n_m / self.command_effectiveness_n_m_per_unit;
        self.actual = raw_actual.clamp(self.limits.min, self.limits.max);
        saturated = saturated
            || allocation.saturated
            || (raw_actual - self.actual).abs() > 1.0e-12 * raw_actual.abs().max(1.0);
        let state = EffectorState {
            commanded: effective_cmd,
            actual: self.actual,
            saturated,
            rate_limited: allocation.quantized || allocation.saturated || pwpf_gated,
            fault: self.fault,
        };
        self.last_state = state;
        Ok(state)
    }

    fn limits(&self) -> EffectorLimits {
        self.limits
    }

    fn set_rcs_thruster_feed_pressure_scales(
        &mut self,
        scales: &[f64],
    ) -> Result<(), EffectorError> {
        self.set_thruster_feed_pressure_scales(scales)
    }

    fn rcs_last_thruster_pulses(&self) -> &[RcsThrusterBankPulse] {
        self.last_thruster_pulses()
    }

    fn inject_fault(&mut self, fault: EffectorFault) -> Result<(), EffectorError> {
        self.validate_fault(fault)?;
        self.fault = Some(fault);
        Ok(())
    }

    fn current_state(&self) -> EffectorState {
        self.last_state
    }
}

impl RcsPulseEffector {
    /// Construct a scalar RCS pulse effector.
    ///
    /// # Errors
    ///
    /// Returns [`EffectorError`] when limits, parameters, `dt`, or the initial
    /// position are invalid.
    pub fn new(
        id: EffectorId,
        limits: EffectorLimits,
        params: RcsPulseEffectorParams,
        dt: Duration,
        initial_position: f64,
    ) -> Result<Self, EffectorError> {
        let dt_s = dt.as_seconds();
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(EffectorError::InvalidLimits {
                reason: "dt must be finite and strictly positive",
            });
        }
        limits.require_valid(dt)?;
        if !initial_position.is_finite() {
            return Err(EffectorError::NonFiniteCommand {
                value: initial_position,
            });
        }
        if initial_position < limits.min || initial_position > limits.max {
            return Err(EffectorError::InvalidLimits {
                reason: "initial_position must be within [min, max]",
            });
        }
        let mib = RcsMinimumImpulseBit::new(params.minimum_impulse_n_s, params.nominal_thrust_n)
            .map_err(|_| EffectorError::InvalidLimits {
                reason: "RCS MIB parameters must be finite and strictly positive",
            })?;
        if mib.nominal_thrust_n > limits.max || -mib.nominal_thrust_n < limits.min {
            return Err(EffectorError::InvalidLimits {
                reason: "nominal_thrust_n must fit within [min, max]",
            });
        }
        if mib.minimum_impulse_n_s > mib.nominal_thrust_n * dt_s {
            return Err(EffectorError::InvalidLimits {
                reason: "minimum_impulse_n_s must fit within one dt at nominal_thrust_n",
            });
        }
        let pwpf = params
            .pwpf
            .map(PwpfModulator::new)
            .transpose()
            .map_err(|_| EffectorError::InvalidLimits {
                reason: "PWPF parameters are invalid",
            })?;
        let last_state = EffectorState::at_rest(initial_position);
        Ok(Self {
            id,
            limits,
            dt,
            mib,
            pwpf,
            actual: initial_position,
            last_state,
            fault: None,
            fault_elapsed_s: 0.0,
        })
    }

    fn validate_fault(&self, fault: EffectorFault) -> Result<(), EffectorError> {
        match fault {
            EffectorFault::Jam { at } => {
                if !at.is_finite() {
                    return Err(EffectorError::InvalidFault {
                        reason: "Jam.at must be finite",
                    });
                }
                if at < self.limits.min || at > self.limits.max {
                    return Err(EffectorError::InvalidFault {
                        reason: "Jam.at must lie within [min, max]",
                    });
                }
            }
            EffectorFault::Runaway { rate_per_s } => {
                if !rate_per_s.is_finite() {
                    return Err(EffectorError::InvalidFault {
                        reason: "Runaway.rate_per_s must be finite",
                    });
                }
            }
            EffectorFault::ReducedRate { factor } => {
                if !factor.is_finite() {
                    return Err(EffectorError::InvalidFault {
                        reason: "ReducedRate.factor must be finite",
                    });
                }
                if !(0.0..=1.0).contains(&factor) {
                    return Err(EffectorError::InvalidFault {
                        reason: "ReducedRate.factor must lie in [0, 1]",
                    });
                }
            }
            EffectorFault::Hardover { to } => {
                if !to.is_finite() {
                    return Err(EffectorError::InvalidFault {
                        reason: "Hardover.to must be finite",
                    });
                }
                if to < self.limits.min || to > self.limits.max {
                    return Err(EffectorError::InvalidFault {
                        reason: "Hardover.to must lie within [min, max]",
                    });
                }
            }
            EffectorFault::Oscillatory {
                amplitude,
                frequency_hz,
                phase_rad,
            } => {
                if !amplitude.is_finite() || amplitude < 0.0 {
                    return Err(EffectorError::InvalidFault {
                        reason: "Oscillatory.amplitude must be finite and non-negative",
                    });
                }
                if !frequency_hz.is_finite() || frequency_hz <= 0.0 {
                    return Err(EffectorError::InvalidFault {
                        reason: "Oscillatory.frequency_hz must be finite and strictly positive",
                    });
                }
                if !phase_rad.is_finite() {
                    return Err(EffectorError::InvalidFault {
                        reason: "Oscillatory.phase_rad must be finite",
                    });
                }
            }
        }
        Ok(())
    }
}

impl ControlEffector for RcsPulseEffector {
    fn id(&self) -> EffectorId {
        self.id
    }

    fn step(&mut self, cmd: f64, dt: Duration) -> Result<EffectorState, EffectorError> {
        let configured_s = self.dt.as_seconds();
        let got_s = dt.as_seconds();
        if (configured_s - got_s).abs() > 1.0e-12 {
            return Err(EffectorError::DtMismatch {
                configured_s,
                got_s,
            });
        }
        if !cmd.is_finite() {
            return Err(EffectorError::NonFiniteCommand { value: cmd });
        }
        let effective_cmd = if let Some(fault) = self.fault {
            if let Some(offset) = fault.oscillatory_offset(self.fault_elapsed_s) {
                let value = cmd + offset;
                if !value.is_finite() {
                    return Err(EffectorError::NonFiniteCommand { value });
                }
                self.fault_elapsed_s += got_s;
                value
            } else {
                cmd
            }
        } else {
            cmd
        };

        if let Some(fault) = self.fault {
            match fault {
                EffectorFault::Jam { at } => {
                    self.actual = at;
                    let state = EffectorState {
                        commanded: cmd,
                        actual: self.actual,
                        saturated: self.actual <= self.limits.min || self.actual >= self.limits.max,
                        rate_limited: true,
                        fault: Some(fault),
                    };
                    self.last_state = state;
                    return Ok(state);
                }
                EffectorFault::Hardover { to } => {
                    self.actual = to.clamp(self.limits.min, self.limits.max);
                    let state = EffectorState {
                        commanded: cmd,
                        actual: self.actual,
                        saturated: self.actual <= self.limits.min || self.actual >= self.limits.max,
                        rate_limited: true,
                        fault: Some(fault),
                    };
                    self.last_state = state;
                    return Ok(state);
                }
                EffectorFault::Runaway { rate_per_s } => {
                    let proposed = self.actual + rate_per_s * got_s;
                    self.actual = proposed.clamp(self.limits.min, self.limits.max);
                    let state = EffectorState {
                        commanded: cmd,
                        actual: self.actual,
                        saturated: proposed < self.limits.min || proposed > self.limits.max,
                        rate_limited: false,
                        fault: Some(fault),
                    };
                    self.last_state = state;
                    return Ok(state);
                }
                EffectorFault::ReducedRate { .. } => {}
                EffectorFault::Oscillatory { .. } => {}
            }
        }

        let mut saturated = effective_cmd < self.limits.min || effective_cmd > self.limits.max;
        let command = effective_cmd.clamp(self.limits.min, self.limits.max);
        let reduced_factor = match self.fault {
            Some(EffectorFault::ReducedRate { factor }) => factor.clamp(0.0, 1.0),
            _ => 1.0,
        };
        let nominal = self.mib.nominal_thrust_n * reduced_factor;
        if nominal <= 0.0 {
            self.actual = 0.0;
            let state = EffectorState {
                commanded: effective_cmd,
                actual: self.actual,
                saturated,
                rate_limited: command != 0.0,
                fault: self.fault,
            };
            self.last_state = state;
            return Ok(state);
        }

        let pulse_command = if let Some(pwpf) = &mut self.pwpf {
            let step = pwpf
                .step(command, dt)
                .map_err(|_| EffectorError::InvalidLimits {
                    reason: "PWPF step failed",
                })?;
            step.polarity.sign() * nominal
        } else {
            command.clamp(-nominal, nominal)
        };
        let pulse_command_differs =
            (pulse_command - command).abs() > 1.0e-12 * command.abs().max(1.0);
        if pulse_command_differs {
            saturated = saturated || command.abs() > nominal;
        }

        let quantizer = RcsMinimumImpulseBit {
            minimum_impulse_n_s: self.mib.minimum_impulse_n_s,
            nominal_thrust_n: nominal,
        };
        let requested_impulse_n_s = pulse_command * got_s;
        let pulse = quantizer
            .pulse_for_requested_impulse(requested_impulse_n_s, dt)
            .map_err(|_| EffectorError::InvalidLimits {
                reason: "RCS pulse exceeded fixed-step capacity",
            })?;
        self.actual = pulse
            .average_thrust_n
            .clamp(self.limits.min, self.limits.max);
        let quantized = (pulse.actual_impulse_n_s - requested_impulse_n_s).abs() > 1.0e-12;
        let state = EffectorState {
            commanded: effective_cmd,
            actual: self.actual,
            saturated,
            rate_limited: quantized || pulse_command_differs,
            fault: self.fault,
        };
        self.last_state = state;
        Ok(state)
    }

    fn limits(&self) -> EffectorLimits {
        self.limits
    }

    fn inject_fault(&mut self, fault: EffectorFault) -> Result<(), EffectorError> {
        self.validate_fault(fault)?;
        self.fault = Some(fault);
        Ok(())
    }

    fn current_state(&self) -> EffectorState {
        self.last_state
    }
}

/// Errors produced by RCS pulse primitives.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum RcsError {
    /// Static parameter validation failed.
    #[error("RCS parameter {field} invalid: {reason}")]
    InvalidParameter {
        /// Parameter field.
        field: &'static str,
        /// Human-readable rule.
        reason: &'static str,
    },
    /// A PWPF command was not finite.
    #[error("RCS command is not finite: {value}")]
    NonFiniteCommand {
        /// Offending command.
        value: f64,
    },
    /// A requested impulse was not finite.
    #[error("RCS requested impulse is not finite: {value}")]
    NonFiniteImpulseRequest {
        /// Offending impulse.
        value: f64,
    },
    /// Invalid fixed-step duration.
    #[error("RCS dt must be finite and strictly positive, got {dt_s}")]
    InvalidDt {
        /// Offending step duration in seconds.
        dt_s: f64,
    },
    /// The requested or MIB-floored impulse cannot fit into one fixed step at
    /// the configured nominal thrust.
    #[error(
        "RCS requested impulse {requested_impulse_n_s} N*s exceeds fixed-step capacity {capacity_n_s} N*s"
    )]
    StepCapacityExceeded {
        /// Requested or MIB-floored signed impulse.
        requested_impulse_n_s: f64,
        /// Available impulse capacity over the step.
        capacity_n_s: f64,
    },
}

fn require_positive_finite(field: &'static str, value: f64) -> Result<(), RcsError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(RcsError::InvalidParameter {
            field,
            reason: "must be finite and strictly positive",
        });
    }
    Ok(())
}

fn require_valid_dt(dt: Duration) -> Result<f64, RcsError> {
    let dt_s = dt.as_seconds();
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(RcsError::InvalidDt { dt_s });
    }
    Ok(dt_s)
}

fn schmitt(previous: PulsePolarity, filtered: f64, params: PwpfParams) -> PulsePolarity {
    match previous {
        PulsePolarity::Off => {
            if filtered >= params.on_threshold {
                PulsePolarity::Positive
            } else if filtered <= -params.on_threshold {
                PulsePolarity::Negative
            } else {
                PulsePolarity::Off
            }
        }
        PulsePolarity::Positive => {
            if filtered <= -params.on_threshold {
                PulsePolarity::Negative
            } else if filtered <= params.off_threshold {
                PulsePolarity::Off
            } else {
                PulsePolarity::Positive
            }
        }
        PulsePolarity::Negative => {
            if filtered >= params.on_threshold {
                PulsePolarity::Positive
            } else if filtered >= -params.off_threshold {
                PulsePolarity::Off
            } else {
                PulsePolarity::Negative
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;

    fn dt() -> Duration {
        Duration::from_seconds(0.01)
    }

    #[test]
    fn mib_quantizer_zero_floor_and_linear_regions() {
        let mib = RcsMinimumImpulseBit::new(0.02, 10.0).unwrap();

        let zero = mib.pulse_for_requested_impulse(0.0, dt()).unwrap();
        assert_eq!(zero.polarity, PulsePolarity::Off);
        assert!(zero.actual_impulse_n_s.abs() <= 1.0e-12);

        let floored = mib.pulse_for_requested_impulse(0.005, dt()).unwrap();
        assert_eq!(floored.polarity, PulsePolarity::Positive);
        assert!((floored.actual_impulse_n_s - 0.02).abs() <= 1.0e-12);
        assert!((floored.on_time_s - 0.002).abs() <= 1.0e-12);
        assert!((floored.duty_cycle - 0.2).abs() <= 1.0e-12);
        assert!((floored.average_thrust_n - 2.0).abs() <= 1.0e-12);

        let linear = mib.pulse_for_requested_impulse(0.06, dt()).unwrap();
        assert!((linear.actual_impulse_n_s - 0.06).abs() <= 1.0e-12);
        assert!((linear.duty_cycle - 0.6).abs() <= 1.0e-12);

        let negative = mib.pulse_for_requested_impulse(-0.001, dt()).unwrap();
        assert_eq!(negative.polarity, PulsePolarity::Negative);
        assert!((negative.actual_impulse_n_s + 0.02).abs() <= 1.0e-12);
    }

    #[test]
    fn mib_quantizer_rejects_invalid_inputs_and_step_capacity() {
        assert!(matches!(
            RcsMinimumImpulseBit::new(0.0, 10.0),
            Err(RcsError::InvalidParameter { .. })
        ));
        let mib = RcsMinimumImpulseBit::new(0.2, 10.0).unwrap();
        assert!(matches!(
            mib.pulse_for_requested_impulse(0.01, dt()),
            Err(RcsError::StepCapacityExceeded { .. })
        ));
        assert!(matches!(
            mib.pulse_for_requested_impulse(f64::NAN, dt()),
            Err(RcsError::NonFiniteImpulseRequest { .. })
        ));
    }

    #[test]
    fn pwpf_schmitt_hysteresis_tracks_positive_command() {
        let params = PwpfParams {
            gain: 1.0,
            time_constant_s: 0.1,
            on_threshold: 0.5,
            off_threshold: 0.2,
        };
        let mut pwpf = PwpfModulator::new(params).unwrap();
        let step = pwpf.step(1.0, Duration::from_seconds(0.1)).unwrap();
        let expected_first = 1.0 - (-1.0_f64).exp();
        assert!((step.filtered_command - expected_first).abs() <= 1.0e-12);
        assert_eq!(step.polarity, PulsePolarity::Positive);

        let still_on = pwpf.step(0.0, Duration::from_seconds(0.1)).unwrap();
        assert_eq!(still_on.polarity, PulsePolarity::Positive);
        assert!(still_on.filtered_command > params.off_threshold);

        let off = pwpf.step(0.0, Duration::from_seconds(0.1)).unwrap();
        assert_eq!(off.polarity, PulsePolarity::Off);
        assert!(off.filtered_command < params.off_threshold);
    }

    #[test]
    fn pwpf_schmitt_supports_negative_polarity() {
        let params = PwpfParams {
            gain: 1.0,
            time_constant_s: 0.1,
            on_threshold: 0.5,
            off_threshold: 0.2,
        };
        let mut pwpf = PwpfModulator::new(params).unwrap();
        let step = pwpf.step(-1.0, Duration::from_seconds(0.1)).unwrap();
        assert_eq!(step.polarity, PulsePolarity::Negative);
        assert!(step.filtered_command < -params.on_threshold);
        assert!(pwpf.polarity().is_firing());
    }

    fn pulse_limits() -> EffectorLimits {
        EffectorLimits {
            min: -10.0,
            max: 10.0,
            max_rate_per_s: 1_000.0,
            deadband: 0.0,
            latency: Duration::from_seconds(0.0),
        }
    }

    #[test]
    fn rcs_pulse_effector_quantizes_direct_command_to_average_output() {
        let mut effector = RcsPulseEffector::new(
            EffectorId::from_path("test.rcs"),
            pulse_limits(),
            RcsPulseEffectorParams {
                minimum_impulse_n_s: 0.02,
                nominal_thrust_n: 10.0,
                pwpf: None,
            },
            dt(),
            0.0,
        )
        .unwrap();

        let floored = effector.step(0.5, dt()).unwrap();
        assert!((floored.actual - 2.0).abs() <= 1.0e-12);
        assert!(floored.rate_limited);

        let linear = effector.step(6.0, dt()).unwrap();
        assert!((linear.actual - 6.0).abs() <= 1.0e-12);
        assert!(!linear.rate_limited);
    }

    #[test]
    fn rcs_pulse_effector_pwpf_gates_nominal_pulses() {
        let step = Duration::from_seconds(0.1);
        let mut effector = RcsPulseEffector::new(
            EffectorId::from_path("test.rcs_pwpf"),
            pulse_limits(),
            RcsPulseEffectorParams {
                minimum_impulse_n_s: 0.02,
                nominal_thrust_n: 10.0,
                pwpf: Some(PwpfParams {
                    gain: 1.0,
                    time_constant_s: 0.1,
                    on_threshold: 0.5,
                    off_threshold: 0.2,
                }),
            },
            step,
            0.0,
        )
        .unwrap();

        let on = effector.step(1.0, step).unwrap();
        assert!((on.actual - 10.0).abs() <= 1.0e-12);
        assert!(on.rate_limited);

        let mut negative_effector = RcsPulseEffector::new(
            EffectorId::from_path("test.rcs_pwpf_negative"),
            pulse_limits(),
            RcsPulseEffectorParams {
                minimum_impulse_n_s: 0.02,
                nominal_thrust_n: 10.0,
                pwpf: Some(PwpfParams {
                    gain: 1.0,
                    time_constant_s: 0.1,
                    on_threshold: 0.5,
                    off_threshold: 0.2,
                }),
            },
            step,
            0.0,
        )
        .unwrap();
        let negative = negative_effector.step(-1.0, step).unwrap();
        assert!((negative.actual + 10.0).abs() <= 1.0e-12);
    }

    #[test]
    fn rcs_pulse_effector_oscillatory_fault_is_quantized() {
        let mut effector = RcsPulseEffector::new(
            EffectorId::from_path("test.rcs_osc"),
            pulse_limits(),
            RcsPulseEffectorParams {
                minimum_impulse_n_s: 0.02,
                nominal_thrust_n: 10.0,
                pwpf: None,
            },
            dt(),
            0.0,
        )
        .unwrap();
        effector
            .inject_fault(EffectorFault::Oscillatory {
                amplitude: 0.01,
                frequency_hz: 2.0,
                phase_rad: std::f64::consts::FRAC_PI_2,
            })
            .unwrap();

        let state = effector.step(0.0, dt()).unwrap();
        assert!((state.commanded - 0.01).abs() <= 1.0e-12);
        assert!((state.actual - 2.0).abs() <= 1.0e-12);
        assert!(state.rate_limited);
    }

    fn bank_pair() -> Vec<RcsThrusterConfig> {
        vec![
            RcsThrusterConfig {
                position_body_m: [1.0, 0.0, 0.0],
                direction_body: [0.0, 0.0, -1.0],
                minimum_impulse_n_s: 0.02,
                nominal_thrust_n: 10.0,
                blowdown: None,
            },
            RcsThrusterConfig {
                position_body_m: [1.0, 0.0, 0.0],
                direction_body: [0.0, 0.0, 1.0],
                minimum_impulse_n_s: 0.02,
                nominal_thrust_n: 10.0,
                blowdown: None,
            },
        ]
    }

    #[test]
    fn rcs_thruster_bank_allocates_geometry_to_equivalent_command() {
        let mut bank = RcsThrusterBankEffector::new(
            EffectorId::from_path("test.rcs_bank"),
            pulse_limits(),
            RcsThrusterBankEffectorParams {
                axis_index: 1,
                command_effectiveness_n_m_per_unit: 2.0,
                thrusters: bank_pair(),
                pwpf: None,
            },
            dt(),
            0.0,
        )
        .unwrap();

        let floored = bank.step(0.5, dt()).unwrap();
        assert!((floored.actual - 1.0).abs() <= 1.0e-12);
        assert!(floored.rate_limited);

        let negative = bank.step(-0.5, dt()).unwrap();
        assert!((negative.actual + 1.0).abs() <= 1.0e-12);
        assert!(negative.rate_limited);

        let linear = bank.step(3.0, dt()).unwrap();
        assert!((linear.actual - 3.0).abs() <= 1.0e-12);
        assert!(!linear.rate_limited);
    }

    #[test]
    fn rcs_thruster_bank_blowdown_reduces_later_capacity() {
        let mut bank = RcsThrusterBankEffector::new(
            EffectorId::from_path("test.rcs_bank_blowdown"),
            EffectorLimits {
                min: 0.0,
                max: 20.0,
                max_rate_per_s: 1_000.0,
                deadband: 0.0,
                latency: Duration::from_seconds(0.0),
            },
            RcsThrusterBankEffectorParams {
                axis_index: 1,
                command_effectiveness_n_m_per_unit: 1.0,
                thrusters: vec![RcsThrusterConfig {
                    position_body_m: [1.0, 0.0, 0.0],
                    direction_body: [0.0, 0.0, -1.0],
                    minimum_impulse_n_s: 0.01,
                    nominal_thrust_n: 10.0,
                    blowdown: Some(RcsBlowdownParams {
                        initial_pressure_pa: 1_000_000.0,
                        minimum_pressure_pa: 500_000.0,
                        usable_impulse_n_s: 0.1,
                        pressure_exponent: 1.0,
                    }),
                }],
                pwpf: None,
            },
            dt(),
            0.0,
        )
        .unwrap();

        let initial = bank.step(10.0, dt()).unwrap();
        assert!((initial.actual - 10.0).abs() <= 1.0e-12);

        let depleted = bank.step(10.0, dt()).unwrap();
        assert!((depleted.actual - 5.0).abs() <= 1.0e-12);
        assert!(depleted.saturated);
    }

    fn coupled_diagonal_thruster(minimum_impulse_n_s: f64) -> RcsThrusterConfig {
        RcsThrusterConfig {
            position_body_m: [0.0, 0.0, 1.0],
            direction_body: [1.0, -1.0, 0.0],
            minimum_impulse_n_s,
            nominal_thrust_n: 10.0,
            blowdown: None,
        }
    }

    #[test]
    fn rcs_coupled_allocator_allocates_cross_axis_thruster_geometry() {
        let mut allocator =
            RcsCoupledAllocator::new(vec![coupled_diagonal_thruster(0.001)], dt()).unwrap();

        let allocation = allocator.allocate([0.04, 0.04, 0.0]).unwrap();

        assert_eq!(allocation.pulses.len(), 1);
        assert!(!allocation.saturated);
        assert!(!allocation.quantized);
        assert!((allocation.actual_torque_impulse_body_n_m_s[0] - 0.04).abs() <= 1.0e-12);
        assert!((allocation.actual_torque_impulse_body_n_m_s[1] - 0.04).abs() <= 1.0e-12);
        assert!(allocation.actual_torque_impulse_body_n_m_s[2].abs() <= 1.0e-12);
        assert!(Vector3::from(allocation.residual_torque_impulse_body_n_m_s).norm() <= 1.0e-12);

        let pulse = &allocation.pulses[0];
        let expected_impulse_n_s = 0.04 * 2.0_f64.sqrt();
        assert_eq!(pulse.thruster_index, 0);
        assert!((pulse.requested_impulse_n_s - expected_impulse_n_s).abs() <= 1.0e-12);
        assert!((pulse.actual_impulse_n_s - expected_impulse_n_s).abs() <= 1.0e-12);
        assert!((pulse.on_time_s - expected_impulse_n_s / 10.0).abs() <= 1.0e-12);
        assert!((pulse.duty_cycle - expected_impulse_n_s / 0.1).abs() <= 1.0e-12);
    }

    #[test]
    fn rcs_coupled_allocator_reports_mib_quantization_without_saturation() {
        let mut allocator =
            RcsCoupledAllocator::new(vec![coupled_diagonal_thruster(0.02)], dt()).unwrap();

        let allocation = allocator.allocate([0.0001, 0.0001, 0.0]).unwrap();

        assert_eq!(allocation.pulses.len(), 1);
        assert!(!allocation.saturated);
        assert!(allocation.quantized);
        let expected_component = 0.02 * std::f64::consts::FRAC_1_SQRT_2;
        assert!(
            (allocation.actual_torque_impulse_body_n_m_s[0] - expected_component).abs() <= 1.0e-12
        );
        assert!(
            (allocation.actual_torque_impulse_body_n_m_s[1] - expected_component).abs() <= 1.0e-12
        );
        let pulse = &allocation.pulses[0];
        assert!((pulse.requested_impulse_n_s - 0.0001 * 2.0_f64.sqrt()).abs() <= 1.0e-12);
        assert!((pulse.actual_impulse_n_s - 0.02).abs() <= 1.0e-12);
    }

    #[test]
    fn rcs_coupled_allocator_rejects_invalid_geometry_or_request() {
        assert!(matches!(
            RcsCoupledAllocator::new(
                vec![RcsThrusterConfig {
                    position_body_m: [0.0, 0.0, 1.0],
                    direction_body: [0.0, 0.0, 1.0],
                    minimum_impulse_n_s: 0.001,
                    nominal_thrust_n: 10.0,
                    blowdown: None,
                }],
                dt(),
            ),
            Err(EffectorError::InvalidLimits { .. })
        ));

        let mut allocator =
            RcsCoupledAllocator::new(vec![coupled_diagonal_thruster(0.001)], dt()).unwrap();
        assert!(matches!(
            allocator.allocate([f64::NAN, 0.0, 0.0]),
            Err(EffectorError::InvalidLimits { .. })
        ));
    }
}
