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
//! direct-torque effectors), each axis is an independent SISO loop.
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
        (u_clamped, saturated)
    }
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

    use super::*;

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
}
