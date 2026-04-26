//! The lockstep simulation kernel.
//!
//! [`SimulationKernel`] owns the integrator, the model trio
//! ([`ForceModel`], [`MassModel`], [`EnvironmentModel`]), the
//! [`StopCondition`], the current state, and the step counter. It
//! runs a deterministic step loop until the stop condition fires.
//!
//! # Determinism contract
//!
//! 1. Time advances by `start + step * dt` (canonical multiplication)
//!    — not by accumulating `t += dt` — to avoid O(N · ε) drift.
//! 2. Floating-point environment is asserted clean at construction
//!    (FTZ / DAZ flushed to zero, round-to-nearest-ties-to-even
//!    rounding mode) on x86_64.
//! 3. The integrator's locked weighted-sum order plus the FMA-disabled
//!    target features make every step bit-stable across reruns on the
//!    same platform profile.
//! 4. Every model is evaluated at deterministic, well-defined sub-step
//!    times.
//! 5. Tracing-emitted bytes are not part of the kernel's deterministic
//!    output; the determinism CI gate runs once with the subscriber
//!    redirected to verify this.
//!
//! See `docs/phase-1-plan.md § 1.3` for the full Phase-1.3 contract.

use openbmp_core::{Duration, SimTime, StepIndex};
use openbmp_state::PointMassState;
use uom::si::mass::kilogram;

use crate::derivative::PointMassDerivative;
use crate::error::{IntegratorError, SimulationError, StopReason};
use crate::integrator::{Integrator, SimState};
use crate::models::{EnvironmentModel, EnvironmentQuery, ForceContext, ForceModel, MassModel};
use crate::stop::StopCondition;

/// Configuration for [`SimulationKernel`].
///
/// Generic over the integrator, force model, mass model, environment
/// model, and stop condition. Phase 1.3's state type is fixed to
/// [`PointMassState`]; rigid-body lands in a follow-on sub-phase.
#[derive(Debug)]
pub struct SimulationConfig<I, F, MM, E, SC>
where
    I: Integrator<PointMassState>,
    F: ForceModel,
    MM: MassModel,
    E: EnvironmentModel,
    SC: StopCondition,
{
    /// Initial point-mass state. Must satisfy
    /// `PointMassState::require_valid()`.
    pub initial_state: PointMassState,
    /// Integrator instance.
    pub integrator: I,
    /// Force model.
    pub force_model: F,
    /// Mass model.
    pub mass_model: MM,
    /// Environment model.
    pub environment: E,
    /// Stop condition.
    pub stop_condition: SC,
    /// Fixed time step.
    pub dt: Duration,
    /// Scenario seed (TigerBeetle VOPR pattern). Stored for downstream
    /// RNG-driven models (sensors, fault models) to derive
    /// deterministic streams.
    pub scenario_seed: u64,
}

/// The lockstep simulation kernel.
#[derive(Debug)]
pub struct SimulationKernel<I, F, MM, E, SC>
where
    I: Integrator<PointMassState>,
    F: ForceModel,
    MM: MassModel,
    E: EnvironmentModel,
    SC: StopCondition,
{
    state: PointMassState,
    initial_state: PointMassState,
    initial_time_s: f64,
    step_index: StepIndex,
    integrator: I,
    force_model: F,
    mass_model: MM,
    environment: E,
    stop_condition: SC,
    dt: Duration,
    dt_s: f64,
    scenario_seed: u64,
    stopped: Option<StopReason>,
}

impl<I, F, MM, E, SC> SimulationKernel<I, F, MM, E, SC>
where
    I: Integrator<PointMassState>,
    F: ForceModel,
    MM: MassModel,
    E: EnvironmentModel,
    SC: StopCondition,
{
    /// Construct from a [`SimulationConfig`].
    ///
    /// # Errors
    ///
    /// Returns [`SimulationError::InvalidConfig`] if `dt` is not
    /// strictly positive and finite, [`SimulationError::State`] if the
    /// initial state fails validation, or
    /// [`SimulationError::FpEnvironmentDirty`] on x86_64 if the
    /// MXCSR register is not in the canonical (round-to-nearest,
    /// FTZ/DAZ off) state required for bit-stable replay.
    pub fn new(config: SimulationConfig<I, F, MM, E, SC>) -> Result<Self, SimulationError> {
        let dt_s = config.dt.as_seconds();
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(SimulationError::InvalidConfig {
                reason: format!("dt must be strictly positive and finite, got {dt_s} s"),
            });
        }
        config.initial_state.require_valid()?;
        assert_clean_mxcsr()?;
        let initial_time_s = config.initial_state.time.as_seconds();
        Ok(Self {
            state: config.initial_state,
            initial_state: config.initial_state,
            initial_time_s,
            step_index: StepIndex::ZERO,
            integrator: config.integrator,
            force_model: config.force_model,
            mass_model: config.mass_model,
            environment: config.environment,
            stop_condition: config.stop_condition,
            dt: config.dt,
            dt_s,
            scenario_seed: config.scenario_seed,
            stopped: None,
        })
    }

    /// Advance the kernel by exactly one step.
    ///
    /// Order of operations:
    ///
    /// 1. If already stopped, return Ok(()) — `step()` is idempotent
    ///    after a stop.
    /// 2. Evaluate the stop condition against the **current** state.
    ///    If it fires, record the reason and return.
    /// 3. Advance the step counter candidate (overflow-checked).
    /// 4. Build the derivative closure capturing the models and
    ///    environment.
    /// 5. Call `integrator.advance` (which evaluates the closure four
    ///    times for RK4).
    /// 6. Overwrite the new state's time with the canonical
    ///    `start + step * dt` value.
    /// 7. Validate post-step state.
    /// 8. Emit a `tracing::trace!` event.
    ///
    /// # Errors
    ///
    /// Returns [`SimulationError::Integrator`] if the integrator
    /// fails, [`SimulationError::Time`] if the step counter overflows,
    /// or [`SimulationError::InvalidPostStepState`] if the integrated
    /// state fails post-step validation.
    #[allow(clippy::cast_precision_loss)] // step values stay well under 2^52
    pub fn step(&mut self) -> Result<(), SimulationError> {
        if self.stopped.is_some() {
            return Ok(());
        }

        if let Some(reason) = self.stop_condition.evaluate(&self.state, self.step_index) {
            tracing::debug!(
                step = self.step_index.value(),
                stop = reason.label(),
                "stop condition fired"
            );
            self.stopped = Some(reason);
            return Ok(());
        }

        // Advance the step counter candidate before model evaluation;
        // if it cannot advance, no derivative call for an uncommittable
        // step is allowed to run.
        let next_step = match self.step_index.checked_next() {
            Ok(step) => step,
            Err(err) => {
                self.stopped = Some(StopReason::StepOverflow {
                    step: self.step_index,
                });
                return Err(SimulationError::Time(err));
            }
        };

        // Capture refs into locals so the closure does not borrow
        // `&self` and we can still mutate `self.state` after the call.
        let force_model = &self.force_model;
        let mass_model = &self.mass_model;
        let environment = &self.environment;

        let derive = |s: &PointMassState, t: SimTime| -> PointMassDerivative {
            let env = environment.sample(EnvironmentQuery {
                time: t,
                position_eci: s.position,
            });
            let force_n_eci = force_model.force_n_eci(ForceContext {
                state: s,
                environment: &env,
                time: t,
            });
            let mass_kg = s.mass.get::<kilogram>();
            let mass_rate_kg_s = mass_model.mass_rate_kg_s(t);
            // Locked order: (force / mass) gives acceleration. We
            // tolerate a non-positive intermediate mass producing
            // NaN/Inf — the integrator's per-stage finite check
            // catches it.
            PointMassDerivative {
                velocity_m_s: s.velocity.vector,
                acceleration_m_s2: force_n_eci / mass_kg,
                mass_rate_kg_s,
            }
        };

        let raw_new = match self.integrator.advance(&self.state, derive, self.dt) {
            Ok(state) => state,
            Err(err @ (IntegratorError::NonFiniteDerivative | IntegratorError::NonFiniteState)) => {
                self.stopped = Some(StopReason::NonFiniteState { step: next_step });
                return Err(SimulationError::Integrator(err));
            }
            Err(err) => return Err(SimulationError::Integrator(err)),
        };

        // Overwrite the integrated time with the canonical
        // `start + step * dt` (single multiplication, no accumulation
        // drift over long runs).
        let canonical_time_s = self.initial_time_s + (next_step.value() as f64) * self.dt_s;
        let new_state = raw_new.with_time(SimTime::from_seconds(canonical_time_s));

        // Post-step validation after canonical time assignment.
        if let Err(source) = new_state.require_valid() {
            self.stopped = Some(StopReason::NonFiniteState { step: next_step });
            return Err(SimulationError::InvalidPostStepState {
                step: next_step,
                source,
            });
        }

        self.state = new_state;
        self.step_index = next_step;

        tracing::trace!(
            step = next_step.value(),
            time_s = canonical_time_s,
            "kernel step complete"
        );

        Ok(())
    }

    /// Run the kernel until the stop condition fires.
    ///
    /// # Errors
    ///
    /// Propagates the first [`SimulationError`] from any `step()` call.
    /// On success, returns the [`StopReason`] that terminated the run.
    pub fn run(&mut self) -> Result<&StopReason, SimulationError> {
        while self.stopped.is_none() {
            self.step()?;
        }
        // Loop exit ⇒ self.stopped is Some. The match is a defensive
        // alternative to `.unwrap()` to satisfy the workspace's
        // `clippy::unwrap_used` lint.
        match self.stopped.as_ref() {
            Some(reason) => Ok(reason),
            None => Err(SimulationError::InvalidConfig {
                reason: "internal invariant: run() loop exited with no stop reason".into(),
            }),
        }
    }

    /// Current state.
    #[must_use]
    pub const fn current_state(&self) -> &PointMassState {
        &self.state
    }

    /// Current step counter.
    #[must_use]
    pub const fn current_step(&self) -> StepIndex {
        self.step_index
    }

    /// Current simulation time.
    #[must_use]
    pub const fn current_time(&self) -> SimTime {
        self.state.time
    }

    /// Configured time step.
    #[must_use]
    pub const fn dt(&self) -> Duration {
        self.dt
    }

    /// Scenario seed (TigerBeetle VOPR pattern). Downstream RNG-driven
    /// models derive deterministic streams from this.
    #[must_use]
    pub const fn scenario_seed(&self) -> u64 {
        self.scenario_seed
    }

    /// Stop reason (`Some` after the kernel has halted).
    #[must_use]
    pub const fn stop_reason(&self) -> Option<&StopReason> {
        self.stopped.as_ref()
    }

    /// Initial state (preserved across the run for replay / reset).
    #[must_use]
    pub const fn initial_state(&self) -> &PointMassState {
        &self.initial_state
    }
}

// ---------------------------------------------------------------------
// Floating-point environment guard
// ---------------------------------------------------------------------

#[cfg(target_arch = "x86_64")]
#[allow(unsafe_code)]
fn assert_clean_mxcsr() -> Result<(), SimulationError> {
    let mut csr = 0_u32;
    let csr_ptr = core::ptr::addr_of_mut!(csr);
    // SAFETY: `stmxcsr` stores the MXCSR control register into the
    // provided 32-bit memory location. `csr_ptr` points to a live local
    // `u32`, is properly aligned, and is valid for this single write.
    unsafe {
        core::arch::asm!(
            "stmxcsr [{0}]",
            in(reg) csr_ptr,
            options(nostack, preserves_flags),
        );
    }
    let ftz = (csr >> 15) & 1;
    let daz = (csr >> 6) & 1;
    let rounding_mode = (csr >> 13) & 0b11;
    if ftz != 0 || daz != 0 || rounding_mode != 0 {
        return Err(SimulationError::FpEnvironmentDirty {
            ftz: ftz != 0,
            daz: daz != 0,
            rounding_mode,
        });
    }
    Ok(())
}

#[cfg(not(target_arch = "x86_64"))]
#[allow(clippy::unnecessary_wraps)] // signature parity with x86_64 path
fn assert_clean_mxcsr() -> Result<(), SimulationError> {
    // No portable equivalent for non-x86_64. The kernel trusts the
    // OS/runtime FP environment on aarch64, etc. Documented in the
    // determinism profile.
    Ok(())
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;
    use crate::integrator::{IntegratorDeterminism, Rk4FixedStep};
    use crate::models::{
        ConstantGravityForce, ConstantMass, LinearBurnMass, NullEnvironment, ZeroForce,
    };
    use crate::stop::{AlwaysContinue, EndTime, MaxSteps};
    use approx::assert_abs_diff_eq;
    use openbmp_core::{Position3, SimTime, Velocity3};
    use openbmp_testkit::strategies;
    use proptest::prelude::*;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    fn one_kg_drop_kernel(
        dt_s: f64,
        stop_s: f64,
    ) -> SimulationKernel<Rk4FixedStep, ConstantGravityForce, ConstantMass, NullEnvironment, EndTime>
    {
        let config = SimulationConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Mass::new::<kilogram>(1.0),
            ),
            integrator: Rk4FixedStep,
            force_model: ConstantGravityForce::down_z(9.80665),
            mass_model: ConstantMass::new(1.0),
            environment: NullEnvironment,
            stop_condition: EndTime::new(SimTime::from_seconds(stop_s)),
            dt: Duration::from_seconds(dt_s),
            scenario_seed: 0xdead_beef,
        };
        SimulationKernel::new(config).expect("valid config must construct")
    }

    #[test]
    fn new_rejects_zero_dt() {
        let config = SimulationConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Mass::new::<kilogram>(1.0),
            ),
            integrator: Rk4FixedStep,
            force_model: ConstantGravityForce::down_z(9.81),
            mass_model: ConstantMass::new(1.0),
            environment: NullEnvironment,
            stop_condition: AlwaysContinue,
            dt: Duration::from_seconds(0.0),
            scenario_seed: 0,
        };
        let result = SimulationKernel::new(config);
        assert!(matches!(result, Err(SimulationError::InvalidConfig { .. })));
    }

    #[test]
    fn new_rejects_invalid_initial_state() {
        let config = SimulationConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::new(f64::NAN, 0.0, 0.0),
                Velocity3::zero(),
                Mass::new::<kilogram>(1.0),
            ),
            integrator: Rk4FixedStep,
            force_model: ConstantGravityForce::down_z(9.81),
            mass_model: ConstantMass::new(1.0),
            environment: NullEnvironment,
            stop_condition: AlwaysContinue,
            dt: Duration::from_seconds(0.01),
            scenario_seed: 0,
        };
        let result = SimulationKernel::new(config);
        assert!(matches!(result, Err(SimulationError::State(_))));
    }

    #[test]
    fn step_advances_step_index_and_time() {
        let mut kernel = one_kg_drop_kernel(0.01, 1.0);
        kernel.step().expect("first step");
        assert_eq!(kernel.current_step().value(), 1);
        assert_abs_diff_eq!(kernel.current_time().as_seconds(), 0.01, epsilon = 1.0e-15);
    }

    #[test]
    fn run_terminates_at_end_time() {
        let mut kernel = one_kg_drop_kernel(0.01, 0.10);
        let reason = kernel.run().expect("run must succeed");
        match reason {
            StopReason::EndTime { reached_s } => {
                assert!((reached_s - 0.10).abs() < 1.0e-12);
            }
            other => panic!("unexpected stop reason: {other:?}"),
        }
        // 10 steps to reach 0.10 s at dt=0.01.
        assert_eq!(kernel.current_step().value(), 10);
    }

    #[test]
    fn run_overshoots_end_time_to_next_step_boundary() {
        let mut kernel = one_kg_drop_kernel(0.01, 0.015);
        let reason = kernel.run().expect("run must succeed");
        match reason {
            StopReason::EndTime { reached_s } => {
                assert_abs_diff_eq!(*reached_s, 0.02, epsilon = 1.0e-15);
            }
            other => panic!("unexpected stop reason: {other:?}"),
        }
        assert_eq!(kernel.current_step().value(), 2);
    }

    #[test]
    fn run_with_max_steps_terminates_on_count() {
        let config = SimulationConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Mass::new::<kilogram>(1.0),
            ),
            integrator: Rk4FixedStep,
            force_model: ConstantGravityForce::down_z(9.81),
            mass_model: ConstantMass::new(1.0),
            environment: NullEnvironment,
            stop_condition: MaxSteps::new(50),
            dt: Duration::from_seconds(0.01),
            scenario_seed: 1,
        };
        let mut kernel = SimulationKernel::new(config).expect("construct");
        let reason = kernel.run().expect("run");
        assert!(matches!(reason, StopReason::UserRequested { .. }));
        assert_eq!(kernel.current_step().value(), 50);
    }

    #[test]
    fn always_continue_allows_manual_steps() {
        let config = SimulationConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Mass::new::<kilogram>(1.0),
            ),
            integrator: Rk4FixedStep,
            force_model: ZeroForce,
            mass_model: ConstantMass::new(1.0),
            environment: NullEnvironment,
            stop_condition: AlwaysContinue,
            dt: Duration::from_seconds(0.01),
            scenario_seed: 1,
        };
        let mut kernel = SimulationKernel::new(config).expect("construct");
        kernel.step().expect("manual step");
        assert_eq!(kernel.current_step().value(), 1);
        assert!(kernel.stop_reason().is_none());
    }

    #[test]
    fn linear_burn_mass_integrates_linearly() {
        let config = SimulationConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Mass::new::<kilogram>(10.0),
            ),
            integrator: Rk4FixedStep,
            force_model: ZeroForce,
            mass_model: LinearBurnMass::new(0.0, 10.0, -0.5),
            environment: NullEnvironment,
            stop_condition: EndTime::new(SimTime::from_seconds(1.0)),
            dt: Duration::from_seconds(0.1),
            scenario_seed: 1,
        };
        let mut kernel = SimulationKernel::new(config).expect("construct");
        kernel.run().expect("run");
        assert_abs_diff_eq!(
            kernel.current_state().mass.get::<kilogram>(),
            9.5,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn step_after_stop_is_no_op() {
        let mut kernel = one_kg_drop_kernel(0.01, 0.05);
        kernel.run().expect("run");
        let stopped_step = kernel.current_step();
        kernel.step().expect("idempotent");
        assert_eq!(kernel.current_step(), stopped_step);
    }

    #[test]
    fn step_overflow_sets_stop_reason() {
        let mut kernel = one_kg_drop_kernel(0.01, 1.0);
        kernel.step_index = StepIndex::new(u64::MAX);
        let err = kernel.step().unwrap_err();
        assert!(matches!(err, SimulationError::Time(_)));
        assert!(matches!(
            kernel.stop_reason(),
            Some(StopReason::StepOverflow { .. })
        ));
    }

    #[test]
    fn invalid_integrated_state_sets_stop_reason() {
        let config = SimulationConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Mass::new::<kilogram>(1.0),
            ),
            integrator: Rk4FixedStep,
            force_model: ZeroForce,
            mass_model: LinearBurnMass::new(0.0, 1.0, -10.0),
            environment: NullEnvironment,
            stop_condition: AlwaysContinue,
            dt: Duration::from_seconds(1.0),
            scenario_seed: 1,
        };
        let mut kernel = SimulationKernel::new(config).expect("construct");
        let err = kernel.step().unwrap_err();
        assert!(matches!(err, SimulationError::Integrator(_)));
        assert!(matches!(
            kernel.stop_reason(),
            Some(StopReason::NonFiniteState { .. })
        ));
        let step = kernel.current_step();
        kernel.step().expect("stopped step is no-op");
        assert_eq!(kernel.current_step(), step);
    }

    #[derive(Copy, Clone, Debug)]
    struct BadIntegrator;

    impl Integrator<PointMassState> for BadIntegrator {
        fn determinism(&self) -> IntegratorDeterminism {
            IntegratorDeterminism::BitStable
        }

        fn advance<DF>(
            &self,
            state: &PointMassState,
            _derive_fn: DF,
            _dt: Duration,
        ) -> Result<PointMassState, IntegratorError>
        where
            DF: Fn(&PointMassState, SimTime) -> PointMassDerivative,
        {
            Ok(PointMassState::new(
                state.time,
                Position3::new(f64::NAN, 0.0, 0.0),
                state.velocity,
                state.mass,
            ))
        }
    }

    #[test]
    fn invalid_post_step_state_sets_stop_reason() {
        let config = SimulationConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Mass::new::<kilogram>(1.0),
            ),
            integrator: BadIntegrator,
            force_model: ZeroForce,
            mass_model: ConstantMass::new(1.0),
            environment: NullEnvironment,
            stop_condition: AlwaysContinue,
            dt: Duration::from_seconds(0.01),
            scenario_seed: 1,
        };
        let mut kernel = SimulationKernel::new(config).expect("construct");
        let err = kernel.step().unwrap_err();
        assert!(matches!(err, SimulationError::InvalidPostStepState { .. }));
        assert!(matches!(
            kernel.stop_reason(),
            Some(StopReason::NonFiniteState { .. })
        ));
    }

    #[test]
    fn time_uses_canonical_multiplication_not_accumulation() {
        // Run for many steps and verify time is exactly start + step * dt
        // (modulo a single ULP from one multiplication), not the
        // O(N · ε) drift naïve accumulation produces.
        let mut kernel = one_kg_drop_kernel(0.01, 10.0);
        kernel.run().expect("run");
        let n = kernel.current_step().value();
        let expected = (n as f64) * 0.01;
        assert_abs_diff_eq!(
            kernel.current_time().as_seconds(),
            expected,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn drop_under_gravity_matches_analytic_for_short_run() {
        // Sanity check; the headline validation lives in
        // tests/analytic_toy.rs.
        let mut kernel = one_kg_drop_kernel(0.01, 1.0);
        kernel.run().expect("run");
        let t = kernel.current_time().as_seconds();
        // Analytic: x(t) = 0 + 0 * t - 0.5 * 9.80665 * t² = -4.903325
        let expected_z = -0.5 * 9.80665 * t * t;
        let actual_z = kernel.current_state().position.vector.z;
        assert_abs_diff_eq!(actual_z, expected_z, epsilon = 1.0e-12);
    }

    #[test]
    fn replay_byte_stable_across_two_runs() {
        // Run the same configuration twice and verify the final state's
        // f64 fields are bit-equal.
        let mut k1 = one_kg_drop_kernel(0.01, 1.0);
        let mut k2 = one_kg_drop_kernel(0.01, 1.0);
        k1.run().expect("run 1");
        k2.run().expect("run 2");
        assert_eq!(
            k1.current_state().position.vector.z.to_bits(),
            k2.current_state().position.vector.z.to_bits()
        );
        assert_eq!(
            k1.current_state().velocity.vector.z.to_bits(),
            k2.current_state().velocity.vector.z.to_bits()
        );
        assert_eq!(
            k1.current_time().as_seconds().to_bits(),
            k2.current_time().as_seconds().to_bits()
        );
    }

    #[test]
    fn scenario_seed_is_preserved() {
        let kernel = one_kg_drop_kernel(0.01, 1.0);
        assert_eq!(kernel.scenario_seed(), 0xdead_beef);
    }

    #[test]
    fn initial_state_is_preserved_across_run() {
        let mut kernel = one_kg_drop_kernel(0.01, 0.05);
        let initial = *kernel.initial_state();
        kernel.run().expect("run");
        // Initial state unchanged after run.
        assert_abs_diff_eq!(
            kernel.initial_state().time.as_seconds(),
            initial.time.as_seconds()
        );
        assert_abs_diff_eq!(
            kernel.initial_state().position.vector.z,
            initial.position.vector.z
        );
    }

    /// Some build configurations (notably `cargo nextest run` on x86_64
    /// macOS or Linux) leave MXCSR in the default round-to-nearest /
    /// FTZ-off / DAZ-off state. This test confirms the kernel
    /// constructs successfully against that baseline. If a future test
    /// run fails here, look for an upstream library mutating MXCSR.
    #[test]
    #[cfg(target_arch = "x86_64")]
    fn mxcsr_is_clean_under_default_test_runner() {
        let kernel = one_kg_drop_kernel(0.01, 0.01);
        // Simply constructing did not error.
        let _ = kernel.current_step();
    }

    proptest! {
        #[test]
        fn one_zero_force_step_preserves_valid_generated_state(
            initial in strategies::point_mass_state(60.0, 1.0e6, 1.0e3, 1.0, 1.0e3),
            dt_s in 1.0e-6_f64..1.0,
        ) {
            let initial_time_s = initial.time.as_seconds();
            let config = SimulationConfig {
                initial_state: initial,
                integrator: Rk4FixedStep,
                force_model: ZeroForce,
                mass_model: ConstantMass::new(initial.mass_kg()),
                environment: NullEnvironment,
                stop_condition: AlwaysContinue,
                dt: Duration::from_seconds(dt_s),
                scenario_seed: 0,
            };
            let mut kernel = SimulationKernel::new(config).expect("construct");
            kernel.step().expect("step");
            prop_assert_eq!(kernel.current_step().value(), 1);
            prop_assert!(kernel.current_state().require_valid().is_ok());
            prop_assert_eq!(
                kernel.current_time().as_seconds().to_bits(),
                (initial_time_s + dt_s).to_bits()
            );
        }
    }
}
