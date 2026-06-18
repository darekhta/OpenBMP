//! [`LinearActuator`] reference impl.
//!
//! First-order linear actuator with rate clamp, position
//! saturation, deadband, and fixed-depth circular-buffer pure
//! delay. The canonical [`EffectorFault`] modes are honoured
//! per Stevens & Lewis 2015 §10.4.2 *Actuator Models* and Patton,
//! Frank & Clark 1989 *Fault Diagnosis in Dynamic Systems*.
//!
//! # Step order (each `step()` call)
//!
//! 1. Validate `dt` matches the construction-time `dt` and `cmd`
//!    is finite.
//! 2. Apply the active fault, if any:
//!    - `Jam{at}` → `actual = at`; ignore command.
//!    - `Hardover{to}` → step toward `to` at full max-rate; once
//!      reached, jam there.
//!    - `Runaway{rate}` → `actual += rate * dt` (still saturated to
//!      `[min, max]`).
//!    - `ReducedRate{factor}` → use a scaled `max_rate_per_s` for
//!      the rate clamp this step.
//!    - `Oscillatory{...}` → add a deterministic sinusoidal command
//!      offset before the normal actuator path.
//! 3. Push the latest command on the latency buffer back; pop the
//!    front to get the `delayed_cmd` fed into the actuator.
//! 4. Apply deadband: if `|delayed_cmd - actual| < deadband`, hold.
//! 5. Apply position saturation to the command target.
//! 6. Apply rate clamp on the increment.
//! 7. Apply first-order lag: `actual <- actual + (1 - exp(-dt/tau)) *
//!    (target - actual)`. For `tau = 0`, the rate-limited increment
//!    is committed directly.
//! 8. Pack and cache the [`EffectorState`].

use std::collections::VecDeque;

use openbmp_core::{Duration, EffectorId};

use super::{ControlEffector, EffectorError, EffectorFault, EffectorLimits, EffectorState};

/// First-order linear actuator with optional pure-delay buffer.
#[derive(Clone, Debug)]
pub struct LinearActuator {
    id: EffectorId,
    limits: EffectorLimits,
    dt: Duration,
    /// Per-step latency buffer. `latency_depth` entries; pre-filled
    /// with `initial_position` so the first `latency_depth` calls
    /// of `step()` see the initial position as the delayed command.
    latency_buffer: VecDeque<f64>,
    /// Current actual deflection.
    actual: f64,
    /// Time constant for the first-order lag (s). `0.0` collapses
    /// to a pure rate-clamped tracker.
    tau_s: f64,
    /// Cached most-recent [`EffectorState`].
    last_state: EffectorState,
    /// Active fault.
    fault: Option<EffectorFault>,
    /// Local elapsed time for deterministic oscillatory faults.
    fault_elapsed_s: f64,
}

impl LinearActuator {
    /// Construct an actuator.
    ///
    /// `tau_s = 0.0` collapses to a discrete rate-clamped tracker
    /// (the rate-limited increment is committed exactly each step).
    /// Positive `tau_s` adds a first-order lag with the standard
    /// `1 - exp(-dt/tau)` discretisation.
    ///
    /// # Errors
    ///
    /// Returns [`EffectorError::InvalidLimits`] when the limits do
    /// not validate against `dt`, or [`EffectorError::SubStepLatency`]
    /// when `0 < latency < dt`. Also rejects non-finite `tau_s` or
    /// `initial_position`, and rejects `initial_position` outside
    /// `[min, max]`.
    pub fn new(
        id: EffectorId,
        limits: EffectorLimits,
        dt: Duration,
        initial_position: f64,
        tau_s: f64,
    ) -> Result<Self, EffectorError> {
        limits.require_valid(dt)?;
        if !tau_s.is_finite() || tau_s < 0.0 {
            return Err(EffectorError::InvalidLimits {
                reason: "tau_s must be finite and non-negative",
            });
        }
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
        let latency_depth = latency_buffer_depth(limits.latency, dt)?;
        let latency_buffer = VecDeque::from(vec![initial_position; latency_depth]);
        let last_state = EffectorState::at_rest(initial_position);
        Ok(Self {
            id,
            limits,
            dt,
            latency_buffer,
            actual: initial_position,
            tau_s,
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

/// Compute the fixed-step pure-delay buffer depth from the
/// scenario-declared latency and kernel `dt`. Rounds to the nearest
/// integer step count after asserting the latency is a whole-step
/// multiple within 1e-9 relative tolerance.
fn latency_buffer_depth(latency: Duration, dt: Duration) -> Result<usize, EffectorError> {
    let latency_s = latency.as_seconds();
    let dt_s = dt.as_seconds();
    if latency_s == 0.0 {
        return Ok(0);
    }
    let ratio = latency_s / dt_s;
    let rounded = ratio.round();
    if (ratio - rounded).abs() > 1.0e-9 * ratio {
        return Err(EffectorError::InvalidLimits {
            reason: "latency must be an integer multiple of dt",
        });
    }
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    Ok(rounded as usize)
}

impl ControlEffector for LinearActuator {
    fn id(&self) -> EffectorId {
        self.id
    }

    #[allow(clippy::too_many_lines)]
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

        // Step 2: fault dispatch — handle Jam / Hardover / Runaway
        // up-front because they short-circuit the linear-actuator
        // path. ReducedRate and Oscillatory fall through.
        if let Some(fault) = self.fault {
            match fault {
                EffectorFault::Jam { at } => {
                    self.actual = at;
                    let saturated =
                        self.actual <= self.limits.min || self.actual >= self.limits.max;
                    let state = EffectorState {
                        commanded: cmd,
                        actual: self.actual,
                        saturated,
                        rate_limited: true,
                        fault: Some(fault),
                    };
                    self.last_state = state;
                    return Ok(state);
                }
                EffectorFault::Hardover { to } => {
                    let max_rate = self.limits.max_rate_per_s;
                    let step_limit = max_rate * dt.as_seconds();
                    let raw_delta = to - self.actual;
                    let delta = raw_delta.clamp(-step_limit, step_limit);
                    self.actual = (self.actual + delta).clamp(self.limits.min, self.limits.max);
                    let saturated =
                        self.actual <= self.limits.min || self.actual >= self.limits.max;
                    let state = EffectorState {
                        commanded: cmd,
                        actual: self.actual,
                        saturated,
                        rate_limited: true,
                        fault: Some(fault),
                    };
                    self.last_state = state;
                    if raw_delta.abs() <= step_limit + 1.0e-12 {
                        self.fault = Some(EffectorFault::Jam { at: self.actual });
                    }
                    return Ok(state);
                }
                EffectorFault::Runaway { rate_per_s } => {
                    let proposed = self.actual + rate_per_s * dt.as_seconds();
                    let saturated = proposed < self.limits.min || proposed > self.limits.max;
                    self.actual = proposed.clamp(self.limits.min, self.limits.max);
                    let state = EffectorState {
                        commanded: cmd,
                        actual: self.actual,
                        saturated,
                        rate_limited: false,
                        fault: Some(fault),
                    };
                    self.last_state = state;
                    return Ok(state);
                }
                EffectorFault::ReducedRate { .. } => {
                    // Falls through to the linear-actuator path with
                    // a scaled max-rate.
                }
                EffectorFault::Oscillatory { .. } => {
                    // Falls through with the sinusoidal command offset applied.
                }
            }
        }

        // Step 3: latency buffer.
        let delayed_cmd = if self.latency_buffer.is_empty() {
            effective_cmd
        } else {
            self.latency_buffer.push_back(effective_cmd);
            // SAFETY: we just pushed to a non-empty (capacity-pinned)
            // buffer; pop is guaranteed to return Some.
            self.latency_buffer.pop_front().unwrap_or(effective_cmd)
        };

        // Step 4: deadband.
        let target_pre_clamp = if (delayed_cmd - self.actual).abs() < self.limits.deadband {
            self.actual
        } else {
            delayed_cmd
        };

        // Step 5: position saturation.
        let target = target_pre_clamp.clamp(self.limits.min, self.limits.max);
        let saturated = target_pre_clamp < self.limits.min || target_pre_clamp > self.limits.max;

        // Step 6: rate clamp (with optional ReducedRate scaling).
        let max_rate_unmodified = self.limits.max_rate_per_s;
        let max_rate = match self.fault {
            Some(EffectorFault::ReducedRate { factor }) => {
                max_rate_unmodified * factor.clamp(0.0, 1.0)
            }
            _ => max_rate_unmodified,
        };
        let step_limit = max_rate * dt.as_seconds();
        let raw_delta = target - self.actual;
        let clamped_delta = raw_delta.clamp(-step_limit, step_limit);
        let rate_limited = (raw_delta - clamped_delta).abs() > 0.0;

        // Step 7: first-order lag (or pure rate-clamped tracker
        // when tau_s == 0).
        let dt_s = dt.as_seconds();
        let actual_step = if self.tau_s > 0.0 {
            let target_after_rate = self.actual + clamped_delta;
            let alpha = (-dt_s / self.tau_s).exp();
            (1.0 - alpha) * (target_after_rate - self.actual)
        } else {
            clamped_delta
        };
        self.actual += actual_step;
        // Re-saturate after first-order lag (lag never overshoots
        // bounded targets, but defence-in-depth).
        self.actual = self.actual.clamp(self.limits.min, self.limits.max);

        let state = EffectorState {
            commanded: effective_cmd,
            actual: self.actual,
            saturated,
            rate_limited,
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

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::unwrap_used
)]
mod tests {
    use super::*;
    use openbmp_core::EffectorId;
    use proptest::prelude::*;

    fn dt() -> Duration {
        Duration::from_seconds(0.001)
    }

    fn make_limits() -> EffectorLimits {
        EffectorLimits {
            min: -0.349,
            max: 0.349,
            max_rate_per_s: 5.236,
            deadband: 0.0,
            latency: Duration::from_seconds(0.0),
        }
    }

    fn fresh_actuator() -> LinearActuator {
        LinearActuator::new(
            EffectorId::from_path("test.actuator"),
            make_limits(),
            dt(),
            0.0,
            0.0,
        )
        .expect("actuator construction must succeed")
    }

    #[test]
    fn at_rest_with_zero_command_holds_position() {
        let mut a = fresh_actuator();
        let state = a.step(0.0, dt()).unwrap();
        assert!((state.actual - 0.0).abs() < 1e-12);
    }

    #[test]
    fn ramps_toward_command_at_max_rate() {
        let mut a = fresh_actuator();
        // commanded position 0.087 (5°), max rate 5.236 rad/s, dt 0.001
        // → first step covers 5.236*0.001 ≈ 0.005236 rad.
        let state = a.step(0.087, dt()).unwrap();
        assert!((state.actual - 0.005_236).abs() < 1e-9);
        assert!(state.rate_limited);
    }

    #[test]
    fn reaches_command_after_enough_steps() {
        let mut a = fresh_actuator();
        let target = 0.087;
        // 0.087 / 0.005236 ≈ 16.62 → 17 steps to reach.
        for _ in 0..17 {
            a.step(target, dt()).unwrap();
        }
        assert!((a.current_state().actual - target).abs() < 1e-3);
    }

    #[test]
    fn jam_locks_position_independent_of_command() {
        let mut a = fresh_actuator();
        a.inject_fault(EffectorFault::Jam { at: 0.05 }).unwrap();
        let state = a.step(0.087, dt()).unwrap();
        assert!((state.actual - 0.05).abs() < 1e-12);
        // Subsequent steps stay at 0.05 regardless of command.
        let state = a.step(-0.349, dt()).unwrap();
        assert!((state.actual - 0.05).abs() < 1e-12);
    }

    #[test]
    fn runaway_drives_at_fault_rate_ignoring_command() {
        let mut a = fresh_actuator();
        a.inject_fault(EffectorFault::Runaway { rate_per_s: 0.5 })
            .unwrap();
        let state = a.step(0.0, dt()).unwrap();
        // Expect actual = 0 + 0.5*0.001 = 0.0005
        assert!((state.actual - 0.000_5).abs() < 1e-12);
        let state = a.step(-0.349, dt()).unwrap();
        // Continues at fault rate regardless of command.
        assert!((state.actual - 0.001_0).abs() < 1e-12);
    }

    #[test]
    fn reduced_rate_scales_max_rate() {
        let mut a = fresh_actuator();
        a.inject_fault(EffectorFault::ReducedRate { factor: 0.5 })
            .unwrap();
        // Half the slew rate: first step covers 0.5236*0.001*0.5 ≈ 0.002618.
        let state = a.step(0.087, dt()).unwrap();
        assert!((state.actual - 0.002_618).abs() < 1e-9);
    }

    #[test]
    fn hardover_steps_to_extreme_then_jams() {
        let mut a = fresh_actuator();
        a.inject_fault(EffectorFault::Hardover { to: 0.349 })
            .unwrap();
        // Hard-rail toward 0.349 at full max rate; reaches in ~67 steps.
        for _ in 0..70 {
            a.step(0.0, dt()).unwrap();
        }
        let state = a.current_state();
        assert!((state.actual - 0.349).abs() < 1e-9);
        assert!(state.saturated);
        assert!(matches!(state.fault, Some(EffectorFault::Jam { .. })));
    }

    #[test]
    fn oscillatory_fault_adds_deterministic_command_offset() {
        let limits = EffectorLimits {
            max_rate_per_s: 1_000.0,
            ..make_limits()
        };
        let mut a =
            LinearActuator::new(EffectorId::from_path("test.osc"), limits, dt(), 0.0, 0.0).unwrap();
        a.inject_fault(EffectorFault::Oscillatory {
            amplitude: 0.1,
            frequency_hz: 2.0,
            phase_rad: std::f64::consts::FRAC_PI_2,
        })
        .unwrap();
        let state = a.step(0.0, dt()).unwrap();
        assert!((state.commanded - 0.1).abs() <= 1.0e-12);
        assert!((state.actual - 0.1).abs() <= 1.0e-12);
        assert!(matches!(
            state.fault,
            Some(EffectorFault::Oscillatory { .. })
        ));
    }

    #[test]
    fn latency_zero_passes_through() {
        // Already covered by `ramps_toward_command_at_max_rate`
        // (latency=0 means the command is consumed immediately).
        let mut a = fresh_actuator();
        let state = a.step(0.005, dt()).unwrap();
        assert!((state.actual - 0.005).abs() < 1e-9);
    }

    #[test]
    fn latency_two_dt_produces_two_step_delay() {
        let limits = EffectorLimits {
            latency: Duration::from_seconds(0.002),
            ..make_limits()
        };
        let mut a =
            LinearActuator::new(EffectorId::from_path("test.delay"), limits, dt(), 0.0, 0.0)
                .unwrap();
        // First two steps see initial_position via the buffer.
        let s1 = a.step(0.087, dt()).unwrap();
        assert!(s1.actual.abs() < 1e-12);
        let s2 = a.step(0.087, dt()).unwrap();
        assert!(s2.actual.abs() < 1e-12);
        // Third step sees the first commanded value (0.087).
        let s3 = a.step(0.087, dt()).unwrap();
        assert!(s3.actual > 0.0);
    }

    #[test]
    fn latency_one_dt_produces_one_step_delay() {
        let limits = EffectorLimits {
            latency: Duration::from_seconds(0.001),
            ..make_limits()
        };
        let mut a =
            LinearActuator::new(EffectorId::from_path("test.delay"), limits, dt(), 0.0, 0.0)
                .unwrap();
        let s1 = a.step(0.087, dt()).unwrap();
        assert!(s1.actual.abs() < 1e-12);
        let s2 = a.step(0.087, dt()).unwrap();
        assert!(s2.actual > 0.0);
    }

    #[test]
    fn latency_non_integer_multiple_rejected() {
        let limits = EffectorLimits {
            latency: Duration::from_seconds(0.0015),
            ..make_limits()
        };
        let err = LinearActuator::new(EffectorId::from_path("test.delay"), limits, dt(), 0.0, 0.0)
            .unwrap_err();
        assert!(matches!(err, EffectorError::InvalidLimits { .. }));
    }

    #[test]
    fn deadband_blocks_micro_commands() {
        let limits = EffectorLimits {
            deadband: 0.01,
            ..make_limits()
        };
        let mut a = LinearActuator::new(
            EffectorId::from_path("test.deadband"),
            limits,
            dt(),
            0.0,
            0.0,
        )
        .unwrap();
        let state = a.step(0.005, dt()).unwrap();
        assert!(state.actual.abs() < 1e-12);
    }

    #[test]
    fn dt_mismatch_rejected() {
        let mut a = fresh_actuator();
        let err = a.step(0.0, Duration::from_seconds(0.002)).unwrap_err();
        assert!(matches!(err, EffectorError::DtMismatch { .. }));
    }

    #[test]
    fn non_finite_command_rejected() {
        let mut a = fresh_actuator();
        let err = a.step(f64::NAN, dt()).unwrap_err();
        assert!(matches!(err, EffectorError::NonFiniteCommand { .. }));
    }

    #[test]
    fn byte_stable_replay_two_runs() {
        let mut a = fresh_actuator();
        let mut b = fresh_actuator();
        let cmds = [0.0, 0.05, 0.1, -0.1, 0.0];
        for &c in &cmds {
            let sa = a.step(c, dt()).unwrap();
            let sb = b.step(c, dt()).unwrap();
            assert_eq!(sa.actual.to_bits(), sb.actual.to_bits());
            assert_eq!(sa.commanded.to_bits(), sb.commanded.to_bits());
            assert_eq!(sa.saturated, sb.saturated);
            assert_eq!(sa.rate_limited, sb.rate_limited);
        }
    }

    #[test]
    fn invalid_initial_position_rejected() {
        let err = LinearActuator::new(
            EffectorId::from_path("test"),
            make_limits(),
            dt(),
            10.0, // way above max=0.349
            0.0,
        )
        .unwrap_err();
        assert!(matches!(err, EffectorError::InvalidLimits { .. }));
    }

    #[test]
    fn invalid_runtime_fault_rejected_without_mutating_existing_fault() {
        let mut a = fresh_actuator();
        a.inject_fault(EffectorFault::ReducedRate { factor: 0.5 })
            .unwrap();
        let err = a.inject_fault(EffectorFault::Jam { at: 10.0 }).unwrap_err();
        assert!(matches!(err, EffectorError::InvalidFault { .. }));
        let state = a.step(0.087, dt()).unwrap();
        assert!(matches!(
            state.fault,
            Some(EffectorFault::ReducedRate { factor }) if (factor - 0.5).abs() < 1e-12
        ));
    }

    proptest! {
        #[test]
        fn property_nominal_never_exits_limits_and_respects_rate(
            commands in proptest::collection::vec(-2.0_f64..2.0, 1..200)
        ) {
            let mut a = fresh_actuator();
            let mut previous = a.current_state().actual;
            let max_step = a.limits().max_rate_per_s * dt().as_seconds();
            for command in commands {
                let state = a.step(command, dt()).unwrap();
                prop_assert!(state.actual >= a.limits().min - 1.0e-12);
                prop_assert!(state.actual <= a.limits().max + 1.0e-12);
                prop_assert!((state.actual - previous).abs() <= max_step + 1.0e-12);
                previous = state.actual;
            }
        }

        #[test]
        fn property_reduced_rate_never_exits_limits_and_respects_scaled_rate(
            commands in proptest::collection::vec(-2.0_f64..2.0, 1..200),
            factor in 0.0_f64..1.0
        ) {
            let mut a = fresh_actuator();
            a.inject_fault(EffectorFault::ReducedRate { factor }).unwrap();
            let mut previous = a.current_state().actual;
            let max_step = a.limits().max_rate_per_s * factor * dt().as_seconds();
            for command in commands {
                let state = a.step(command, dt()).unwrap();
                prop_assert!(state.actual >= a.limits().min - 1.0e-12);
                prop_assert!(state.actual <= a.limits().max + 1.0e-12);
                prop_assert!((state.actual - previous).abs() <= max_step + 1.0e-12);
                previous = state.actual;
            }
        }

        #[test]
        fn property_fault_modes_never_exit_limits(
            commands in proptest::collection::vec(-2.0_f64..2.0, 1..200),
            fault_case in 0_u8..4
        ) {
            let mut a = fresh_actuator();
            let fault = match fault_case {
                0 => EffectorFault::Jam { at: 0.123 },
                1 => EffectorFault::Runaway { rate_per_s: 50.0 },
                2 => EffectorFault::ReducedRate { factor: 0.25 },
                _ => EffectorFault::Hardover { to: -0.349 },
            };
            a.inject_fault(fault).unwrap();
            for command in commands {
                let state = a.step(command, dt()).unwrap();
                prop_assert!(state.actual >= a.limits().min - 1.0e-12);
                prop_assert!(state.actual <= a.limits().max + 1.0e-12);
            }
        }
    }
}
