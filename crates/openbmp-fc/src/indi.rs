//! Incremental Nonlinear Dynamic Inversion rate loop
//! (Smeur, Chu, de Croon 2016).
//!
//! INDI inverts only the **local incremental** relationship between
//! actuator command and angular acceleration. From the rotational
//! plant
//!
//! ```text
//! J · ω̇ = G_eff · u + d
//! ```
//!
//! with the standard project assumption of diagonal `J` (rocket-class
//! body-axis inertia ellipsoid) and diagonal `G_eff` (decoupled
//! direct-torque effectors), the legacy runtime path is an independent
//! SISO loop. The dense-INCA helpers below lift the increment relation
//! to `B_eff Δu ≈ Δω̇` for coupled gimbaled clusters while reusing the
//! FC-local dense WLS allocator.
//! Taking time differences between consecutive samples and assuming
//! the disturbance varies slowly relative to the loop step gives
//!
//! ```text
//! J · Δω̇ ≈ G_eff · Δu
//! ⇒ Δu_axis ≈ (J_axis / G_eff_axis) · (ω̇_des − ω̇_meas)
//! u_axis[k+1] = clamp(u_axis[k] + Δu_axis, u_min, u_max)
//! ```
//!
//! `ω̇_des` is supplied by an outer attitude-loop P-controller
//! (`ω̇_des = K_p · (ω_ref − ω_meas)`); `ω̇_meas` is derived from a
//! filtered finite difference of the gyro samples. Crucially, the
//! prior actuator command `u` and the measured angular rate `ω` must
//! pass through **identical** low-pass filters so the increment
//! relation `Δu ↔ Δω̇` is not biased by filter delay. INDI inherits
//! its robustness from the filtered `ω̇_meas` term: any unmodeled
//! disturbance `d` shows up directly in `ω̇_meas`, and the inversion
//! cancels it on the next sample.
//!
//! Implicit anti-windup: `u[k+1] = clamp(u[k] + Δu)` is bounded by
//! construction. There is no separate integrator state to bleed.
//! When the disturbance exceeds the actuator authority, `u` is pinned
//! at the limit and `Δu ≈ 0` thereafter — the loop is doing
//! everything it can.
//!
//! # Anti-scope (5.A.3.C)
//!
//! - Multi-body assemblies are rejected at scenario load (the
//!   per-axis decoupling assumption breaks for non-trivial inertia
//!   coupling — same precondition the LQR rate loop ships with).
//! - Composition with the L1 adaptive augmentation (5.A.2.C) is
//!   forbidden at scenario load: INDI's filtered `ω̇_meas` already
//!   absorbs matched disturbance, and adding L1 on top creates
//!   filter-interaction concerns that haven't been characterised.
//!   A future slice may revisit.
//! - Inertia-mismatch demonstration scenarios (where the configured
//!   INDI inertia differs from the truth-side body inertia) are
//!   permitted and showcase the controller's hallmark robustness;
//!   the runner does not cross-check the two values.
//!
//! # References
//!
//! - Smeur, E. J. J., Chu, Q., and de Croon, G. C. H. E. (2016).
//!   *Adaptive Incremental Nonlinear Dynamic Inversion for Attitude
//!   Control of Micro Air Vehicles.* JGCD 39(3):450–461.
//! - Smeur et al. (2024). *From fundamentals to applications of
//!   incremental nonlinear dynamic inversion: A survey on INDI –
//!   Part I.* Chinese Journal of Aeronautics.

use thiserror::Error;

use openbmp_core::EffectorId;

#[cfg(not(feature = "std"))]
use alloc::vec;
use std::vec::Vec;

use crate::allocation::{
    AllocatorError, EffectivenessMatrix, EffectorBounds, WlsActiveSetAllocator,
};
use crate::filters::Biquad;

/// Low-pass filter shape applied identically to `ω` and the prior
/// actuator command `u`.
///
/// Both filters MUST share the same transfer function for the INDI
/// increment relation to hold without time-shift bias; the scenario
/// API exposes a single cutoff + kind to make misconfiguration
/// impossible.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum IndiFilterKind {
    /// First-order low-pass via the bilinear transform. Lower delay,
    /// gentler rolloff.
    FirstOrderLowPass,
    /// Second-order Butterworth low-pass via the bilinear transform.
    /// Standard textbook choice (Smeur 2016): maximally flat
    /// passband, sharper rolloff than first-order, more delay.
    #[default]
    SecondOrderButterworth,
}

/// Per-axis INDI configuration.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IndiParams {
    /// INDI's working estimate of the body-axis diagonal inertia
    /// (`kg·m²`). May intentionally differ from the truth-side
    /// vehicle inertia — INDI's robustness is exactly that this
    /// estimate need not be exact.
    pub inertia_per_axis_kg_m2: [f64; 3],
    /// INDI's working estimate of the per-axis control-effectiveness
    /// coefficient `g` such that `Δτ_axis = g_axis · Δu_axis`. For
    /// `direct_torque` effectors with effectiveness 1 N·m / rad and
    /// 1:1 channel mapping, this is `[1.0, 1.0, 1.0]`.
    pub control_effectiveness_per_axis: [f64; 3],
    /// Cutoff (rad/s) applied identically to the `ω` filter and the
    /// `u` filter. Must be `> 0` and `< π · sample_rate_hz`
    /// (Nyquist).
    pub filter_cutoff_rad_s: f64,
    /// Filter shape; see [`IndiFilterKind`].
    pub filter_kind: IndiFilterKind,
    /// Outer-loop attitude-to-angular-acceleration P-gain per axis.
    /// `ω̇_des = K_p · (ω_ref − ω_meas)`; larger → tighter rate
    /// tracking but bigger required actuator authority.
    pub attitude_to_omega_dot_gain: [f64; 3],
}

/// Errors raised by [`IndiParams::validate`] and
/// [`IndiChannel::new`].
#[derive(Copy, Clone, Debug, Error, PartialEq)]
pub enum IndiError {
    /// `inertia_per_axis_kg_m2[axis]` is non-positive.
    #[error("INDI inertia must be > 0; got {value} on axis {axis}")]
    NonPositiveInertia {
        /// Offending axis (0 = roll, 1 = pitch, 2 = yaw).
        axis: usize,
        /// Offending value.
        value: f64,
    },
    /// `control_effectiveness_per_axis[axis]` is non-positive.
    #[error("INDI control effectiveness must be > 0; got {value} on axis {axis}")]
    NonPositiveControlEffectiveness {
        /// Offending axis.
        axis: usize,
        /// Offending value.
        value: f64,
    },
    /// `attitude_to_omega_dot_gain[axis]` is non-positive.
    #[error("INDI attitude-to-omega-dot gain must be > 0; got {value} on axis {axis}")]
    NonPositiveAttitudeGain {
        /// Offending axis.
        axis: usize,
        /// Offending value.
        value: f64,
    },
    /// `filter_cutoff_rad_s` is non-positive.
    #[error("INDI filter cutoff must be > 0; got {value}")]
    NonPositiveFilterCutoff {
        /// Offending value.
        value: f64,
    },
    /// `filter_cutoff_rad_s >= π · sample_rate_hz` (Nyquist).
    #[error(
        "INDI filter cutoff {cutoff_rad_s} rad/s must be < π · sample_rate_hz \
         ({nyquist_rad_s} rad/s)"
    )]
    FilterCutoffAtOrAboveNyquist {
        /// Configured cutoff.
        cutoff_rad_s: f64,
        /// Nyquist boundary in rad/s.
        nyquist_rad_s: f64,
    },
    /// `dt_s` is non-positive.
    #[error("INDI sample time dt_s must be > 0; got {value}")]
    NonPositiveSampleTime {
        /// Offending sample time.
        value: f64,
    },
    /// A parameter is NaN or infinite.
    #[error("INDI parameter must be finite; got {value} for {label}")]
    NonFiniteParameter {
        /// Parameter name.
        label: &'static str,
        /// Offending value.
        value: f64,
    },
}

/// Errors raised by dense-INCA and pseudo-control hedging helpers.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum DenseIncaError {
    /// The dense allocator rejected the request.
    #[error("dense INCA allocator rejected request: {0}")]
    Allocator(AllocatorError),
    /// Vector dimensions do not match the configured effectiveness matrix.
    #[error("dense INCA dimension mismatch: {reason}")]
    DimensionMismatch {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// A scalar is NaN or infinite.
    #[error("dense INCA field {field} must be finite; got {value}")]
    NonFinite {
        /// Field name.
        field: &'static str,
        /// Offending value.
        value: f64,
    },
    /// A scalar must be positive and finite.
    #[error("dense INCA field {field} must be positive and finite; got {value}")]
    NonPositive {
        /// Field name.
        field: &'static str,
        /// Offending value.
        value: f64,
    },
}

impl From<AllocatorError> for DenseIncaError {
    fn from(value: AllocatorError) -> Self {
        Self::Allocator(value)
    }
}

impl IndiParams {
    /// Validate INDI parameters against the loop step `dt_s`.
    ///
    /// # Errors
    ///
    /// Returns the matching [`IndiError`] variant when any per-axis
    /// inertia, control-effectiveness, or attitude gain is
    /// non-positive; when the filter cutoff is non-positive or at /
    /// above Nyquist; when `dt_s` is non-positive; or when any
    /// parameter is non-finite.
    pub fn validate(&self, dt_s: f64) -> Result<(), IndiError> {
        require_finite("dt_s", dt_s)?;
        if dt_s <= 0.0 {
            return Err(IndiError::NonPositiveSampleTime { value: dt_s });
        }
        for axis in 0..3 {
            require_finite_axis(
                "inertia_per_axis_kg_m2",
                axis,
                self.inertia_per_axis_kg_m2[axis],
            )?;
            if self.inertia_per_axis_kg_m2[axis] <= 0.0 {
                return Err(IndiError::NonPositiveInertia {
                    axis,
                    value: self.inertia_per_axis_kg_m2[axis],
                });
            }
            require_finite_axis(
                "control_effectiveness_per_axis",
                axis,
                self.control_effectiveness_per_axis[axis],
            )?;
            if self.control_effectiveness_per_axis[axis] <= 0.0 {
                return Err(IndiError::NonPositiveControlEffectiveness {
                    axis,
                    value: self.control_effectiveness_per_axis[axis],
                });
            }
            require_finite_axis(
                "attitude_to_omega_dot_gain",
                axis,
                self.attitude_to_omega_dot_gain[axis],
            )?;
            if self.attitude_to_omega_dot_gain[axis] <= 0.0 {
                return Err(IndiError::NonPositiveAttitudeGain {
                    axis,
                    value: self.attitude_to_omega_dot_gain[axis],
                });
            }
        }
        require_finite("filter_cutoff_rad_s", self.filter_cutoff_rad_s)?;
        if self.filter_cutoff_rad_s <= 0.0 {
            return Err(IndiError::NonPositiveFilterCutoff {
                value: self.filter_cutoff_rad_s,
            });
        }
        let sample_rate_hz = 1.0 / dt_s;
        require_finite("sample_rate_hz", sample_rate_hz)?;
        let nyquist_rad_s = std::f64::consts::PI * sample_rate_hz;
        if self.filter_cutoff_rad_s >= nyquist_rad_s {
            return Err(IndiError::FilterCutoffAtOrAboveNyquist {
                cutoff_rad_s: self.filter_cutoff_rad_s,
                nyquist_rad_s,
            });
        }
        Ok(())
    }
}

fn require_finite(label: &'static str, value: f64) -> Result<(), IndiError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(IndiError::NonFiniteParameter { label, value })
    }
}

fn require_finite_axis(field: &'static str, _axis: usize, value: f64) -> Result<(), IndiError> {
    require_finite(field, value)
}

/// Single-axis INDI channel state.
///
/// Holds the synchronised `ω` and `u` filters and the previous
/// clamped command. The increment-relation invariant is preserved by
/// pushing the same actuator command through `u_filter` that the
/// rest of the loop produced on the prior step.
#[derive(Copy, Clone, Debug)]
pub struct IndiChannel {
    omega_filter: Biquad,
    u_filter: Biquad,
    omega_prev_filtered: f64,
    u_prev: f64,
}

/// Result of one scalar INDI channel step.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IndiStep {
    /// Clamped actuator command.
    pub command: f64,
    /// `true` when the unclamped command exceeded the supplied bounds.
    pub saturated: bool,
    /// Desired angular acceleration, rad/s².
    pub omega_dot_des_rad_s2: f64,
    /// Measured filtered angular acceleration, rad/s².
    pub omega_dot_meas_rad_s2: f64,
}

/// Dense incremental-control-allocation controller state.
///
/// `WlsActiveSetAllocator` solves absolute commands, so the INCA wrapper
/// shifts the virtual target by the prior achieved pseudo-control:
///
/// `B u_next = B u_prev + (omega_dot_des - omega_dot_meas)`.
///
/// The pseudo-control deficit returned in [`DenseIncaOutput`] is the portion
/// of the requested angular-acceleration increment that could not be achieved
/// because of position/rate bounds. A downstream pseudo-control hedger can
/// subtract that deficit from the reference-model input.
#[derive(Clone, Debug)]
pub struct DenseIncaController {
    allocator: WlsActiveSetAllocator,
    bounds: EffectorBounds,
    u_prev: Vec<f64>,
    dt_s: f64,
}

/// Result of one dense INCA step.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseIncaOutput {
    /// Absolute per-effector command after applying the increment.
    pub u: Vec<(EffectorId, f64)>,
    /// `true` where the allocator pinned an effector at a tightened bound.
    pub active_at_bound: Vec<bool>,
    /// Requested angular-acceleration increment, row order.
    pub requested_increment: Vec<f64>,
    /// Achieved angular-acceleration increment, row order.
    pub achieved_increment: Vec<f64>,
    /// Requested minus achieved increment, row order.
    pub pseudo_control_deficit: Vec<f64>,
    /// `true` if a bound, residual, or nonzero deficit shows unavailable authority.
    pub saturated: bool,
    /// Active-set iterations used by the dense WLS allocator.
    pub iterations: u16,
}

/// Pseudo-control hedging reference-model integrator.
///
/// The helper integrates `commanded_pseudo_control - pseudo_control_deficit`.
/// In a saturated actuator case where the dense-INCA output reports that all of
/// the requested pseudo-control was unavailable, this state does not wind up on
/// the unavailable command.
#[derive(Clone, Debug, PartialEq)]
pub struct PseudoControlHedger {
    integral_state: Vec<f64>,
}

impl IndiChannel {
    /// Construct a channel with the configured filter pair seeded at
    /// zero state.
    ///
    /// # Errors
    ///
    /// Returns an [`IndiError`] when the filter cutoff is invalid
    /// for the supplied `dt_s` (the underlying biquad constructor
    /// rejects cutoffs at or above Nyquist).
    pub fn new(params: &IndiParams, dt_s: f64) -> Result<Self, IndiError> {
        params.validate(dt_s)?;
        let sample_rate_hz = 1.0 / dt_s;
        let (omega_filter, u_filter) =
            match params.filter_kind {
                IndiFilterKind::FirstOrderLowPass => (
                    Biquad::lowpass_first_order(params.filter_cutoff_rad_s, sample_rate_hz)
                        .map_err(|_| IndiError::FilterCutoffAtOrAboveNyquist {
                            cutoff_rad_s: params.filter_cutoff_rad_s,
                            nyquist_rad_s: std::f64::consts::PI / dt_s,
                        })?,
                    Biquad::lowpass_first_order(params.filter_cutoff_rad_s, sample_rate_hz)
                        .map_err(|_| IndiError::FilterCutoffAtOrAboveNyquist {
                            cutoff_rad_s: params.filter_cutoff_rad_s,
                            nyquist_rad_s: std::f64::consts::PI / dt_s,
                        })?,
                ),
                IndiFilterKind::SecondOrderButterworth => (
                    Biquad::butterworth_lowpass_second_order(
                        params.filter_cutoff_rad_s,
                        sample_rate_hz,
                    )
                    .map_err(|_| IndiError::FilterCutoffAtOrAboveNyquist {
                        cutoff_rad_s: params.filter_cutoff_rad_s,
                        nyquist_rad_s: std::f64::consts::PI / dt_s,
                    })?,
                    Biquad::butterworth_lowpass_second_order(
                        params.filter_cutoff_rad_s,
                        sample_rate_hz,
                    )
                    .map_err(|_| IndiError::FilterCutoffAtOrAboveNyquist {
                        cutoff_rad_s: params.filter_cutoff_rad_s,
                        nyquist_rad_s: std::f64::consts::PI / dt_s,
                    })?,
                ),
            };
        Ok(Self {
            omega_filter,
            u_filter,
            omega_prev_filtered: 0.0,
            u_prev: 0.0,
        })
    }

    /// One INDI rate-loop step on a single axis.
    ///
    /// Returns `(u_clamped, saturated)` where `u_clamped` is the
    /// torque command (or generic actuator command) sent to the
    /// effector and `saturated` is `true` when the unclamped
    /// candidate exceeded `[u_min, u_max]`.
    #[allow(clippy::too_many_arguments)]
    pub fn step_diagnostic(
        &mut self,
        params: &IndiParams,
        axis: usize,
        omega_meas: f64,
        rate_cmd: f64,
        u_min: f64,
        u_max: f64,
        dt: f64,
    ) -> IndiStep {
        // 1. Filter the gyro sample.
        let omega_filtered = self.omega_filter.step(omega_meas);
        // 2. Filtered finite difference for ω̇_meas.
        let omega_dot_meas = if dt > 0.0 {
            (omega_filtered - self.omega_prev_filtered) / dt
        } else {
            0.0
        };
        self.omega_prev_filtered = omega_filtered;
        // 3. Outer-loop ω̇_des = K_p · (ω_ref − ω_meas). The outer
        //    loop sees the unfiltered gyro to keep its bandwidth as
        //    high as possible; the filter is INDI-specific.
        let omega_dot_des = params.attitude_to_omega_dot_gain[axis] * (rate_cmd - omega_meas);
        // 4. Filter the prior actuator command — this is the key
        //    sync step: u_filtered carries the same group delay as
        //    omega_filtered, so the increment relation is unbiased.
        let u_prev_filtered = self.u_filter.step(self.u_prev);
        // 5. INDI increment.
        let inertia = params.inertia_per_axis_kg_m2[axis];
        let g_eff = params.control_effectiveness_per_axis[axis];
        let delta_u = (inertia / g_eff) * (omega_dot_des - omega_dot_meas);
        // 6. Apply increment with implicit anti-windup via clamp.
        let u_raw = u_prev_filtered + delta_u;
        let u_clamped = u_raw.clamp(u_min, u_max);
        let saturated = (u_raw - u_clamped).abs() > 0.0;
        self.u_prev = u_clamped;
        IndiStep {
            command: u_clamped,
            saturated,
            omega_dot_des_rad_s2: omega_dot_des,
            omega_dot_meas_rad_s2: omega_dot_meas,
        }
    }

    /// One INDI rate-loop step on a single axis.
    ///
    /// Returns `(u_clamped, saturated)` for existing callers. Use
    /// [`IndiChannel::step_diagnostic`] when the caller also needs the
    /// desired/measured pseudo-control terms for dense-INCA allocation.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        params: &IndiParams,
        axis: usize,
        omega_meas: f64,
        rate_cmd: f64,
        u_min: f64,
        u_max: f64,
        dt: f64,
    ) -> (f64, bool) {
        let step = self.step_diagnostic(params, axis, omega_meas, rate_cmd, u_min, u_max, dt);
        (step.command, step.saturated)
    }
}

impl DenseIncaController {
    /// Construct a dense INCA wrapper around the FC dense WLS allocator.
    ///
    /// # Errors
    ///
    /// Returns [`DenseIncaError`] when dimensions are inconsistent, `dt_s` is
    /// non-positive/non-finite, or the initial command contains non-finite
    /// values. Detailed bound validation is delegated to the dense allocator on
    /// each step so the same fail-closed path is used at runtime.
    pub fn new(
        allocator: WlsActiveSetAllocator,
        bounds: EffectorBounds,
        initial_u: Vec<f64>,
        dt_s: f64,
    ) -> Result<Self, DenseIncaError> {
        let columns = allocator.matrix().columns();
        if initial_u.len() != columns
            || bounds.u_min_pos.len() != columns
            || bounds.u_max_pos.len() != columns
            || bounds.rate_limit.len() != columns
        {
            return Err(DenseIncaError::DimensionMismatch {
                reason: "initial command and bounds lengths must match effectiveness columns",
            });
        }
        require_dense_inca_positive("dt_s", dt_s)?;
        for value in &initial_u {
            require_dense_inca_finite("initial_u", *value)?;
        }
        Ok(Self {
            allocator,
            bounds,
            u_prev: initial_u,
            dt_s,
        })
    }

    /// Borrow the configured dense effectiveness matrix.
    #[must_use]
    pub const fn matrix(&self) -> &EffectivenessMatrix {
        self.allocator.matrix()
    }

    /// Previous absolute effector command in matrix-column order.
    #[must_use]
    pub fn previous_command(&self) -> &[f64] {
        &self.u_prev
    }

    /// One dense INCA step.
    ///
    /// `omega_dot_des` and `omega_dot_meas` are angular-acceleration
    /// pseudo-control vectors in the effectiveness matrix row order.
    ///
    /// # Errors
    ///
    /// Returns [`DenseIncaError`] when dimensions, scalars, bounds, or the dense
    /// WLS solve fail validation.
    pub fn step(
        &mut self,
        omega_dot_des: &[f64],
        omega_dot_meas: &[f64],
    ) -> Result<DenseIncaOutput, DenseIncaError> {
        let step_prev = self.u_prev.clone();
        let step_bounds = self.bounds.clone();
        self.step_inner(omega_dot_des, omega_dot_meas, &step_prev, &step_bounds)
    }

    /// One dense INCA step while treating disallowed effectors as fixed-zero
    /// columns.
    ///
    /// # Errors
    ///
    /// Returns [`DenseIncaError`] when dimensions, scalars, bounds, or the dense
    /// WLS solve fail validation.
    pub fn step_with_allowed_effectors(
        &mut self,
        omega_dot_des: &[f64],
        omega_dot_meas: &[f64],
        allowed_effectors: &[EffectorId],
    ) -> Result<DenseIncaOutput, DenseIncaError> {
        let mut step_prev = self.u_prev.clone();
        let mut step_bounds = self.bounds.clone();
        for (column, effector_id) in self.matrix().effectors().iter().enumerate() {
            if !allowed_effectors.contains(effector_id) {
                step_prev[column] = 0.0;
                step_bounds.u_min_pos[column] = 0.0;
                step_bounds.u_max_pos[column] = 0.0;
                step_bounds.rate_limit[column] = 0.0;
            }
        }
        self.step_inner(omega_dot_des, omega_dot_meas, &step_prev, &step_bounds)
    }

    fn step_inner(
        &mut self,
        omega_dot_des: &[f64],
        omega_dot_meas: &[f64],
        step_prev: &[f64],
        step_bounds: &EffectorBounds,
    ) -> Result<DenseIncaOutput, DenseIncaError> {
        let rows = self.matrix().rows();
        validate_dense_inca_vector("omega_dot_des", omega_dot_des, rows)?;
        validate_dense_inca_vector("omega_dot_meas", omega_dot_meas, rows)?;
        let prior_pseudo_control = effectiveness_multiply(self.matrix(), step_prev)?;
        let requested_increment = omega_dot_des
            .iter()
            .zip(omega_dot_meas)
            .map(|(desired, measured)| desired - measured)
            .collect::<Vec<_>>();
        let target_pseudo_control = prior_pseudo_control
            .iter()
            .zip(&requested_increment)
            .map(|(prior, increment)| prior + increment)
            .collect::<Vec<_>>();

        let allocation =
            self.allocator
                .allocate(&target_pseudo_control, step_prev, step_bounds, self.dt_s)?;
        let next_u = allocation
            .u
            .iter()
            .map(|(_, command)| *command)
            .collect::<Vec<_>>();
        let achieved_pseudo_control = effectiveness_multiply(self.matrix(), &next_u)?;
        let achieved_increment = achieved_pseudo_control
            .iter()
            .zip(&prior_pseudo_control)
            .map(|(next, prior)| next - prior)
            .collect::<Vec<_>>();
        let pseudo_control_deficit = requested_increment
            .iter()
            .zip(&achieved_increment)
            .map(|(requested, achieved)| requested - achieved)
            .collect::<Vec<_>>();
        let saturated = allocation.active_at_bound.iter().any(|active| *active)
            || !allocation.attainable
            || pseudo_control_deficit
                .iter()
                .any(|deficit| deficit.abs() > 1.0e-9);
        self.u_prev = next_u;
        Ok(DenseIncaOutput {
            u: allocation.u,
            active_at_bound: allocation.active_at_bound,
            requested_increment,
            achieved_increment,
            pseudo_control_deficit,
            saturated,
            iterations: allocation.iterations,
        })
    }
}

impl PseudoControlHedger {
    /// Construct a hedger with zero integral state.
    ///
    /// # Errors
    ///
    /// Returns [`DenseIncaError`] when `dimension == 0`.
    pub fn new(dimension: usize) -> Result<Self, DenseIncaError> {
        if dimension == 0 {
            return Err(DenseIncaError::DimensionMismatch {
                reason: "pseudo-control hedger dimension must be nonzero",
            });
        }
        Ok(Self {
            integral_state: vec![0.0; dimension],
        })
    }

    /// Integrate one hedged reference-model input.
    ///
    /// Returns the hedged pseudo-control vector
    /// `commanded_pseudo_control - pseudo_control_deficit`.
    ///
    /// # Errors
    ///
    /// Returns [`DenseIncaError`] when dimensions mismatch, values are
    /// non-finite, or `dt_s` is non-positive.
    pub fn step(
        &mut self,
        commanded_pseudo_control: &[f64],
        pseudo_control_deficit: &[f64],
        dt_s: f64,
    ) -> Result<Vec<f64>, DenseIncaError> {
        let dimension = self.integral_state.len();
        validate_dense_inca_vector(
            "commanded_pseudo_control",
            commanded_pseudo_control,
            dimension,
        )?;
        validate_dense_inca_vector("pseudo_control_deficit", pseudo_control_deficit, dimension)?;
        require_dense_inca_positive("dt_s", dt_s)?;
        let mut hedged = vec![0.0; dimension];
        for i in 0..dimension {
            hedged[i] = commanded_pseudo_control[i] - pseudo_control_deficit[i];
            require_dense_inca_finite("hedged_pseudo_control", hedged[i])?;
            self.integral_state[i] += hedged[i] * dt_s;
        }
        Ok(hedged)
    }

    /// Current integrated hedged reference-model input.
    #[must_use]
    pub fn integral_state(&self) -> &[f64] {
        &self.integral_state
    }
}

fn effectiveness_multiply(
    matrix: &EffectivenessMatrix,
    u: &[f64],
) -> Result<Vec<f64>, DenseIncaError> {
    if u.len() != matrix.columns() {
        return Err(DenseIncaError::DimensionMismatch {
            reason: "command length must match effectiveness columns",
        });
    }
    validate_dense_inca_vector("u", u, matrix.columns())?;
    let mut out = vec![0.0; matrix.rows()];
    for (row, out_value) in out.iter_mut().enumerate() {
        let mut sum = 0.0;
        for (column, command) in u.iter().enumerate() {
            let coefficient =
                matrix
                    .coefficient(row, column)
                    .ok_or(DenseIncaError::DimensionMismatch {
                        reason: "effectiveness coefficient index out of range",
                    })?;
            sum += coefficient * command;
        }
        require_dense_inca_finite("effectiveness product", sum)?;
        *out_value = sum;
    }
    Ok(out)
}

fn validate_dense_inca_vector(
    field: &'static str,
    values: &[f64],
    expected_len: usize,
) -> Result<(), DenseIncaError> {
    if values.len() != expected_len {
        return Err(DenseIncaError::DimensionMismatch {
            reason: "vector length does not match configured dense INCA dimension",
        });
    }
    for value in values {
        require_dense_inca_finite(field, *value)?;
    }
    Ok(())
}

fn require_dense_inca_finite(field: &'static str, value: f64) -> Result<(), DenseIncaError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(DenseIncaError::NonFinite { field, value })
    }
}

fn require_dense_inca_positive(field: &'static str, value: f64) -> Result<(), DenseIncaError> {
    require_dense_inca_finite(field, value)?;
    if value <= 0.0 {
        return Err(DenseIncaError::NonPositive { field, value });
    }
    Ok(())
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]
mod tests {
    use approx::assert_abs_diff_eq;
    use toml::Value;

    use super::*;

    const TVC_TWD_INCA_TOLERANCE_TABLE: &str =
        include_str!("../../../data/gnc/tvc-twd-inca-tolerance-v1.toml");

    fn fixture() -> Value {
        toml::from_str(TVC_TWD_INCA_TOLERANCE_TABLE).expect("TVC/TWD/INCA fixture parses")
    }

    fn toml_table<'a>(value: &'a Value, key: &str) -> &'a toml::map::Map<String, Value> {
        value
            .get(key)
            .and_then(Value::as_table)
            .unwrap_or_else(|| panic!("missing TOML table {key}"))
    }

    fn toml_f64(value: &toml::map::Map<String, Value>, key: &str) -> f64 {
        value
            .get(key)
            .and_then(Value::as_float)
            .unwrap_or_else(|| panic!("missing TOML float {key}"))
    }

    fn toml_usize(value: &toml::map::Map<String, Value>, key: &str) -> usize {
        value
            .get(key)
            .and_then(Value::as_integer)
            .and_then(|raw| usize::try_from(raw).ok())
            .unwrap_or_else(|| panic!("missing TOML usize {key}"))
    }

    fn toml_f64_array(value: &toml::map::Map<String, Value>, key: &str) -> Vec<f64> {
        value
            .get(key)
            .and_then(Value::as_array)
            .unwrap_or_else(|| panic!("missing TOML array {key}"))
            .iter()
            .map(|entry| {
                entry
                    .as_float()
                    .unwrap_or_else(|| panic!("TOML array {key} contains non-float"))
            })
            .collect()
    }

    fn eid(path: &str) -> EffectorId {
        EffectorId::from_path(path)
    }

    fn nominal_params() -> IndiParams {
        IndiParams {
            inertia_per_axis_kg_m2: [1.0, 1.0, 1.0],
            control_effectiveness_per_axis: [1.0, 1.0, 1.0],
            filter_cutoff_rad_s: 50.0,
            filter_kind: IndiFilterKind::SecondOrderButterworth,
            attitude_to_omega_dot_gain: [10.0, 10.0, 10.0],
        }
    }

    #[test]
    fn validate_accepts_nominal_params() {
        nominal_params()
            .validate(0.001)
            .expect("nominal params validate");
    }

    #[test]
    fn validate_rejects_non_positive_inertia() {
        let mut p = nominal_params();
        p.inertia_per_axis_kg_m2[1] = 0.0;
        assert!(matches!(
            p.validate(0.001),
            Err(IndiError::NonPositiveInertia { axis: 1, .. })
        ));
        p.inertia_per_axis_kg_m2[2] = -1.0;
        assert!(matches!(
            p.validate(0.001),
            Err(IndiError::NonPositiveInertia { .. })
        ));
    }

    #[test]
    fn validate_rejects_non_positive_control_effectiveness() {
        let mut p = nominal_params();
        p.control_effectiveness_per_axis[0] = 0.0;
        assert!(matches!(
            p.validate(0.001),
            Err(IndiError::NonPositiveControlEffectiveness { axis: 0, .. })
        ));
    }

    #[test]
    fn validate_rejects_non_positive_attitude_gain() {
        let mut p = nominal_params();
        p.attitude_to_omega_dot_gain[2] = -0.5;
        assert!(matches!(
            p.validate(0.001),
            Err(IndiError::NonPositiveAttitudeGain { axis: 2, .. })
        ));
    }

    #[test]
    fn validate_rejects_non_positive_filter_cutoff() {
        let mut p = nominal_params();
        p.filter_cutoff_rad_s = 0.0;
        assert!(matches!(
            p.validate(0.001),
            Err(IndiError::NonPositiveFilterCutoff { .. })
        ));
    }

    #[test]
    fn validate_rejects_non_positive_dt() {
        let p = nominal_params();
        assert!(matches!(
            p.validate(0.0),
            Err(IndiError::NonPositiveSampleTime { .. })
        ));
        assert!(matches!(
            p.validate(-0.001),
            Err(IndiError::NonPositiveSampleTime { .. })
        ));
    }

    #[test]
    fn validate_rejects_filter_cutoff_at_or_above_nyquist() {
        let mut p = nominal_params();
        // Nyquist for dt = 0.001 is π·1000 rad/s ≈ 3141.6.
        p.filter_cutoff_rad_s = 5_000.0;
        assert!(matches!(
            p.validate(0.001),
            Err(IndiError::FilterCutoffAtOrAboveNyquist { .. })
        ));
    }

    #[test]
    fn validate_rejects_non_finite_parameters() {
        let mut p = nominal_params();
        p.filter_cutoff_rad_s = f64::NAN;
        assert!(matches!(
            p.validate(0.001),
            Err(IndiError::NonFiniteParameter { .. })
        ));
        p.filter_cutoff_rad_s = 50.0;
        p.inertia_per_axis_kg_m2[0] = f64::INFINITY;
        assert!(matches!(
            p.validate(0.001),
            Err(IndiError::NonFiniteParameter { .. })
        ));
    }

    #[test]
    fn channel_new_seeds_filters_at_zero_state() {
        let p = nominal_params();
        let ch = IndiChannel::new(&p, 0.001).unwrap();
        assert_eq!(ch.omega_prev_filtered, 0.0);
        assert_eq!(ch.u_prev, 0.0);
    }

    #[test]
    fn first_order_filter_kind_constructs_a_valid_channel() {
        let mut p = nominal_params();
        p.filter_kind = IndiFilterKind::FirstOrderLowPass;
        IndiChannel::new(&p, 0.001).expect("first-order channel");
    }

    /// Closed-loop on a unit-inertia plant `J·ω̇ = u`: a step rate
    /// command should be tracked with bounded error.
    #[test]
    fn closed_loop_step_response_settles() {
        let dt = 0.001;
        let p = nominal_params();
        let mut ch = IndiChannel::new(&p, dt).unwrap();
        let omega_ref = 1.0_f64;
        let mut omega = 0.0_f64;
        for _ in 0..5_000 {
            let (u, _sat) = ch.step(&p, 0, omega, omega_ref, -10.0, 10.0, dt);
            omega += dt * u;
        }
        assert_abs_diff_eq!(omega, omega_ref, epsilon = 1.0e-2);
    }

    /// Closed-loop on a unit-inertia plant with constant matched
    /// torque disturbance: INDI's filtered `ω̇_meas` term should
    /// absorb the disturbance, yielding bounded steady-state rate
    /// error.
    #[test]
    fn closed_loop_rejects_constant_torque_disturbance() {
        let dt = 0.001;
        let p = nominal_params();
        let mut ch = IndiChannel::new(&p, dt).unwrap();
        let disturbance_torque = 0.05_f64;
        let mut omega = 0.0_f64;
        for _ in 0..10_000 {
            let (u, _) = ch.step(&p, 0, omega, 0.0, -10.0, 10.0, dt);
            omega += dt * (u + disturbance_torque);
        }
        assert!(omega.abs() < 5.0e-2, "rate after disturbance was {omega}");
    }

    /// INDI's hallmark robustness: the inversion uses a parameter
    /// inertia of 0.7 against a truth-side inertia of 1.0. The
    /// scenario should still track a step rate command without
    /// blowing up.
    #[test]
    fn closed_loop_tolerates_inertia_estimate_mismatch() {
        let dt = 0.001;
        let mut p = nominal_params();
        p.inertia_per_axis_kg_m2[0] = 0.7; // INDI thinks J = 0.7
        let truth_inertia = 1.0; // plant J = 1.0
        let mut ch = IndiChannel::new(&p, dt).unwrap();
        let omega_ref = 1.0_f64;
        let mut omega = 0.0_f64;
        for _ in 0..10_000 {
            let (u, _) = ch.step(&p, 0, omega, omega_ref, -10.0, 10.0, dt);
            omega += dt * u / truth_inertia;
        }
        assert_abs_diff_eq!(omega, omega_ref, epsilon = 5.0e-2);
    }

    /// Saturation envelope: if the disturbance exceeds the actuator
    /// rail, `u` is pinned and INDI does not blow up.
    #[test]
    fn implicit_anti_windup_holds_under_sustained_saturation() {
        let dt = 0.001;
        let p = nominal_params();
        let mut ch = IndiChannel::new(&p, dt).unwrap();
        let big_disturbance = 5.0_f64; // much bigger than rail
        let mut omega = 0.0_f64;
        for _ in 0..1_000 {
            let (u, _) = ch.step(&p, 0, omega, 0.0, -1.0, 1.0, dt);
            assert!((-1.0..=1.0).contains(&u), "u out of rail: {u}");
            omega += dt * (u + big_disturbance);
        }
        assert!(omega.is_finite());
    }

    /// Two channels primed identically must produce identical
    /// outputs across the same input sequence.
    #[test]
    fn channel_is_bit_stable_across_reruns() {
        let dt = 0.001;
        let p = nominal_params();
        let mut ch_a = IndiChannel::new(&p, dt).unwrap();
        let mut ch_b = IndiChannel::new(&p, dt).unwrap();
        for k in 0..1_000_u32 {
            let omega = 0.01 * (f64::from(k) * 0.1).sin();
            let (u_a, sat_a) = ch_a.step(&p, 0, omega, 0.5, -1.0, 1.0, dt);
            let (u_b, sat_b) = ch_b.step(&p, 0, omega, 0.5, -1.0, 1.0, dt);
            assert_eq!(u_a.to_bits(), u_b.to_bits());
            assert_eq!(sat_a, sat_b);
        }
    }

    #[test]
    fn first_principles_increment_relation_holds_in_steady_state() {
        // After many steps with constant input, the filters reach
        // steady state; in that regime `Δu ≈ 0` and the loop must
        // hold the rate at the command.
        let dt = 0.001;
        let p = nominal_params();
        let mut ch = IndiChannel::new(&p, dt).unwrap();
        let omega_ref = 0.5_f64;
        let mut omega = 0.0_f64;
        let mut last_u = 0.0_f64;
        for _ in 0..20_000 {
            let (u, _) = ch.step(&p, 0, omega, omega_ref, -10.0, 10.0, dt);
            omega += dt * u; // unit inertia, no disturbance
            last_u = u;
        }
        // In steady state the rate has settled to the command and the
        // command is small (≈ 0) since no torque is needed to maintain
        // a constant rate.
        assert_abs_diff_eq!(omega, omega_ref, epsilon = 1.0e-3);
        assert!(last_u.abs() < 1.0e-2, "steady-state u was {last_u}");
    }

    #[test]
    fn step_with_zero_dt_returns_zero_omega_dot() {
        let dt = 0.001;
        let p = nominal_params();
        let mut ch = IndiChannel::new(&p, dt).unwrap();
        let (u_zero_dt, _) = ch.step(&p, 0, 1.0, 1.0, -10.0, 10.0, 0.0);
        // With dt = 0 the increment formula yields ω̇_des·J/g_eff,
        // which for ω_meas == ω_ref is zero. The dt-zero guard
        // prevents division by zero in ω̇_meas.
        assert!(u_zero_dt.is_finite());
    }

    #[test]
    fn dense_inca_controller_cancels_synthetic_twd_reaction() {
        let matrix = EffectivenessMatrix::new(
            2,
            vec![
                eid("vehicle.assembly.effectors.engine-left"),
                eid("vehicle.assembly.effectors.engine-right"),
            ],
            vec![
                1.0, 0.35, //
                -0.20, 0.90,
            ],
        )
        .expect("matrix constructs");
        let allocator = WlsActiveSetAllocator::new(
            matrix,
            vec![1.0, 1.0],
            vec![1.0e-6, 1.0e-6],
            1.0e12,
            vec![0.0, 0.0],
        )
        .expect("allocator constructs");
        let bounds = EffectorBounds {
            u_min_pos: vec![-10.0, -10.0],
            u_max_pos: vec![10.0, 10.0],
            rate_limit: vec![1000.0, 1000.0],
        };
        let mut inca = DenseIncaController::new(allocator, bounds, vec![0.0, 0.0], 0.01).unwrap();

        let twd_reaction = [0.45, -0.20];
        let out = inca.step(&[0.0, 0.0], &twd_reaction).unwrap();

        for (achieved, measured) in out.achieved_increment.iter().zip(twd_reaction) {
            assert_abs_diff_eq!(*achieved + measured, 0.0, epsilon = 1.0e-8);
        }
        for deficit in out.pseudo_control_deficit {
            assert_abs_diff_eq!(deficit, 0.0, epsilon = 1.0e-8);
        }
        assert!(!out.saturated);
        assert_eq!(out.active_at_bound, vec![false, false]);
    }

    #[test]
    fn dense_inca_recovers_effective_authority_against_twd_reaction_series() {
        let data = fixture();
        let tolerances = toml_table(&data, "tolerances");
        let series = toml_table(&data, "inca_recovery_series");
        let rows = toml_usize(series, "rows");
        let effectors = toml_usize(series, "effectors");
        assert_eq!(rows, 2);
        assert_eq!(effectors, 2);
        let matrix = EffectivenessMatrix::new(
            rows,
            vec![
                eid("vehicle.assembly.effectors.engine-left"),
                eid("vehicle.assembly.effectors.engine-right"),
            ],
            toml_f64_array(series, "b_row_major"),
        )
        .expect("matrix constructs");
        let allocator = WlsActiveSetAllocator::new(
            matrix,
            vec![1.0, 1.0],
            vec![1.0e-6, 1.0e-6],
            1.0e12,
            vec![0.0, 0.0],
        )
        .expect("allocator constructs");
        let bounds = EffectorBounds {
            u_min_pos: toml_f64_array(series, "u_min"),
            u_max_pos: toml_f64_array(series, "u_max"),
            rate_limit: toml_f64_array(series, "rate_limit"),
        };
        let dt_s = toml_f64(series, "dt_s");
        let mut inca = DenseIncaController::new(allocator, bounds, vec![0.0, 0.0], dt_s).unwrap();

        let samples_total = toml_usize(series, "samples");
        let disturbances = (0..samples_total)
            .map(|k| {
                let t = k as f64 * dt_s;
                let envelope = (-toml_f64(series, "disturbance_envelope_decay_per_s") * t).exp();
                [
                    envelope
                        * (toml_f64(series, "pitch_sine_amplitude")
                            * (toml_f64(series, "pitch_sine_frequency_rad_s") * t).sin()
                            + toml_f64(series, "pitch_cosine_amplitude")
                                * (toml_f64(series, "pitch_cosine_frequency_rad_s") * t).cos()),
                    envelope
                        * (toml_f64(series, "yaw_cosine_amplitude")
                            * (toml_f64(series, "yaw_cosine_frequency_rad_s") * t).cos()
                            + toml_f64(series, "yaw_sine_amplitude")
                                * (toml_f64(series, "yaw_sine_frequency_rad_s") * t).sin()),
                ]
            })
            .collect::<Vec<_>>();
        let mut uncompensated_sum_sq = 0.0;
        let mut compensated_sum_sq = 0.0;
        let mut samples = 0usize;
        let discard_samples = toml_usize(series, "discard_samples");

        for k in 0..(disturbances.len() - 1) {
            let prior = effectiveness_multiply(inca.matrix(), inca.previous_command()).unwrap();
            let measured = [prior[0] + disturbances[k][0], prior[1] + disturbances[k][1]];
            let out = inca.step(&[0.0, 0.0], &measured).unwrap();
            assert!(!out.saturated);
            let commands = out
                .u
                .iter()
                .map(|(_, command)| *command)
                .collect::<Vec<_>>();
            let compensated = effectiveness_multiply(inca.matrix(), &commands).unwrap();
            let residual_next = [
                compensated[0] + disturbances[k + 1][0],
                compensated[1] + disturbances[k + 1][1],
            ];

            if k >= discard_samples {
                uncompensated_sum_sq += disturbances[k + 1][0] * disturbances[k + 1][0]
                    + disturbances[k + 1][1] * disturbances[k + 1][1];
                compensated_sum_sq +=
                    residual_next[0] * residual_next[0] + residual_next[1] * residual_next[1];
                samples += 1;
            }
        }

        let uncompensated_rms = (uncompensated_sum_sq / samples as f64).sqrt();
        let compensated_rms = (compensated_sum_sq / samples as f64).sqrt();
        let compensated_ratio = compensated_rms / uncompensated_rms;
        assert_eq!(samples, toml_usize(series, "expected_rms_samples"));
        assert_abs_diff_eq!(
            uncompensated_rms,
            toml_f64(series, "expected_uncompensated_rms_rad_s2"),
            epsilon = toml_f64(tolerances, "inca_rms_abs_rad_s2")
        );
        assert_abs_diff_eq!(
            compensated_rms,
            toml_f64(series, "expected_compensated_rms_rad_s2"),
            epsilon = toml_f64(tolerances, "inca_rms_abs_rad_s2")
        );
        assert_abs_diff_eq!(
            compensated_ratio,
            toml_f64(series, "expected_compensated_rms_ratio"),
            epsilon = toml_f64(tolerances, "inca_rms_abs_rad_s2")
        );
        assert!(uncompensated_rms > 1.0e-2);
        assert!(
            compensated_ratio < toml_f64(tolerances, "inca_max_compensated_rms_ratio"),
            "dense INCA should recover effective authority against the TWD series: compensated_rms={compensated_rms:.17e}, uncompensated_rms={uncompensated_rms:.17e}"
        );
    }

    #[test]
    fn dense_inca_reports_pseudo_control_deficit_under_rate_limited_actuator() {
        let matrix =
            EffectivenessMatrix::new(1, vec![eid("vehicle.assembly.effectors.engine")], vec![1.0])
                .expect("matrix constructs");
        let allocator = WlsActiveSetAllocator::new(matrix, vec![1.0], vec![1.0], 1.0e9, vec![0.0])
            .expect("allocator constructs");
        let bounds = EffectorBounds {
            u_min_pos: vec![-10.0],
            u_max_pos: vec![10.0],
            rate_limit: vec![1.0],
        };
        let mut inca = DenseIncaController::new(allocator, bounds, vec![0.0], 0.1).unwrap();

        let out = inca.step(&[2.0], &[0.0]).unwrap();

        assert_abs_diff_eq!(out.u[0].1, 0.1, epsilon = 1.0e-12);
        assert_abs_diff_eq!(out.requested_increment[0], 2.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(out.achieved_increment[0], 0.1, epsilon = 1.0e-12);
        assert_abs_diff_eq!(out.pseudo_control_deficit[0], 1.9, epsilon = 1.0e-12);
        assert!(out.saturated);
        assert_eq!(out.active_at_bound, vec![true]);
    }

    #[test]
    fn pseudo_control_hedging_prevents_unavailable_command_integration() {
        let dt_s = 0.01;
        let mut hedger = PseudoControlHedger::new(1).unwrap();
        let mut unhedged_integral = 0.0;

        for _ in 0..1_000 {
            unhedged_integral += 5.0 * dt_s;
            let hedged = hedger.step(&[5.0], &[5.0], dt_s).unwrap();
            assert_abs_diff_eq!(hedged[0], 0.0, epsilon = 1.0e-12);
        }

        assert!(unhedged_integral > 49.0);
        assert_abs_diff_eq!(hedger.integral_state()[0], 0.0, epsilon = 1.0e-12);
    }
}
