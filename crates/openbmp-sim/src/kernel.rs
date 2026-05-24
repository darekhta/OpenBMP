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
//! See `docs/software-architecture.md § Simulation Kernel` for the
//! full kernel contract.

use std::borrow::Cow;

use openbmp_core::{Duration, SimTime, StepIndex};
use openbmp_state::PointMassState;
use uom::si::mass::kilogram;

use openbmp_models::{SimState, VehicleState};

use crate::derivative::PointMassDerivative;
use crate::error::{IntegratorError, SimulationError, StopReason};
use crate::integrator::Integrator;
use crate::models::{
    EffectorActualsView, EngineSnapshotView, EnvironmentModel, EnvironmentQuery, EnvironmentSample,
    ForceContext, ForceModel, MassContext, MassModel, RecoverySnapshot, RecoverySnapshotView,
    TankSnapshot, TankSnapshotView,
};
use crate::solver_profile::{ProfiledIntegrator, SolverProfile, SolverProfileError};
use crate::stop::StopCondition;

/// Configuration for [`SimulationKernel`].
///
/// Generic over the integrated state type `S`, the integrator, force
/// model, mass model, environment model, and stop condition. Phase
/// 2.1.A introduces the `S: SimState` parameter; Phase 2.1.C adds the
/// rigid-body `step()` impl alongside the existing point-mass one.
///
/// `MM` is intentionally unconstrained at the struct level so the same
/// kernel struct can carry both Phase-1 [`MassModel`] (point-mass
/// scalar mass) and Phase-2.1.C [`crate::models::RigidMassModel`]
/// (full mass / inertia / CG). Each impl block constrains `MM`
/// per the integrated state type.
#[derive(Debug)]
pub struct SimulationConfig<S, I, F, MM, E, SC>
where
    S: SimState,
    I: Integrator<S>,
    F: ForceModel<S>,
    E: EnvironmentModel,
    SC: StopCondition<S>,
{
    /// Initial state. Must satisfy the state type's structural
    /// validation (e.g. positive mass, finite components).
    pub initial_state: S,
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

/// Named-field input for [`SimulationConfig::from_trajectory_profile`].
#[derive(Debug)]
pub struct TrajectoryProfileConfig<S, F, MM, E, SC>
where
    S: SimState,
    F: ForceModel<S>,
    E: EnvironmentModel,
    SC: StopCondition<S>,
{
    /// Initial state.
    pub initial_state: S,
    /// Trajectory solver profile to dispatch.
    pub solver_profile: SolverProfile,
    /// Force model.
    pub force_model: F,
    /// Mass model.
    pub mass_model: MM,
    /// Environment model.
    pub environment: E,
    /// Stop condition.
    pub stop_condition: SC,
    /// Kernel macro-step.
    pub dt: Duration,
    /// Scenario seed.
    pub scenario_seed: u64,
}

impl<S, F, MM, E, SC> SimulationConfig<S, ProfiledIntegrator, F, MM, E, SC>
where
    S: SimState,
    F: ForceModel<S>,
    E: EnvironmentModel,
    SC: StopCondition<S>,
{
    /// Construct a kernel config from a trajectory [`SolverProfile`].
    ///
    /// This is the sim-owned dispatch path for Phase 6.0: callers
    /// supply the physics models and stop condition exactly as before,
    /// while `openbmp-sim` builds the concrete [`ProfiledIntegrator`]
    /// consumed by [`SimulationKernel`]. Fixed-step profiles must
    /// declare the same `dt` as the kernel macro-step.
    ///
    /// # Errors
    ///
    /// Returns [`SolverProfileError`] when the profile is malformed,
    /// source-term-only, reserved, or declares a fixed-step `dt` that
    /// disagrees with `dt`.
    pub fn from_trajectory_profile(
        config: TrajectoryProfileConfig<S, F, MM, E, SC>,
    ) -> Result<Self, SolverProfileError> {
        let integrator = config
            .solver_profile
            .build_kernel_trajectory_integrator(config.dt)?;
        Ok(Self {
            initial_state: config.initial_state,
            integrator,
            force_model: config.force_model,
            mass_model: config.mass_model,
            environment: config.environment,
            stop_condition: config.stop_condition,
            dt: config.dt,
            scenario_seed: config.scenario_seed,
        })
    }
}

/// The lockstep simulation kernel.
///
/// Generic over `S: SimState`. Phase-1 / Phase-2.1.A impls `step()`
/// for `S = PointMassState` (`MM: MassModel`); Phase 2.1.C adds the
/// `S = RigidBodyState` impl (`MM = RigidModels<MOM, RigidMassModel>`).
#[derive(Debug)]
pub struct SimulationKernel<S, I, F, MM, E, SC>
where
    S: SimState,
    I: Integrator<S>,
    F: ForceModel<S>,
    E: EnvironmentModel,
    SC: StopCondition<S>,
{
    state: S,
    initial_state: S,
    initial_time_s: f64,
    initial_mass_kg: f64,
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
    /// Mission-action bindings. Empty when no `[mission]` block is
    /// declared; the kernel hot path early-exits in that case so
    /// legacy scenarios stay byte-stable.
    mission_events_typed: Vec<crate::events::EventBinding<crate::events::MissionAction>>,
    /// Simulator-only scenario-script action bindings.
    script_events_typed: Vec<crate::events::EventBinding<crate::events::ScenarioScriptAction>>,
    /// Phase-3.2 mission graph. `None` when no `[mission]` block is
    /// declared.
    mission_graph: Option<crate::events::MissionPhaseGraph>,
    /// Lifted HSM used for transition entry / exit / active actions.
    mission_hsm: Option<crate::MissionStateMachine>,
    /// Active mission phase. Initialised to `mission_graph.initial`
    /// when a graph is wired, else `None`.
    current_phase: Option<crate::events::PhaseId>,
    /// Current authority for `current_phase`.
    mission_state_authority: MissionStateAuthority,
    /// Per-step queue of fired mission-action events.
    pending_mission_fired: Vec<crate::events::FiredEvent<crate::events::MissionAction>>,
    /// Per-step queue of fired scenario-script-action events.
    pending_script_fired: Vec<crate::events::FiredEvent<crate::events::ScenarioScriptAction>>,
    /// Set of binding ids that have fired and are flagged `once: true`.
    /// `BTreeSet` (not `HashSet`) defeats macOS `SipHash` randomisation.
    fired_once_events: std::collections::BTreeSet<crate::events::EventId>,
    /// Previous-step `EventScalars`, fed into the trigger evaluator
    /// for crossing detection. `None` on step 0.
    previous_event_scalars: Option<crate::events::EventScalars>,
    /// Phase-3.5: kernel-owned snapshot of effector-actuals values
    /// keyed by deck-axis name. The runner refreshes this map via
    /// [`Self::set_effector_actuals`] before each `step()` call so
    /// every RK4 stage sees the same snapshot. Empty `BTreeMap` for
    /// legacy / Schema-1 scenarios — schema-1 decks ignore the view
    /// the closure passes through, so legacy code paths produce
    /// byte-identical Parquet to pre-3.5.
    effector_actuals: std::collections::BTreeMap<String, f64>,
    /// Phase-3.6: kernel-owned snapshot of per-engine state, keyed
    /// by `EngineId`. The runner refreshes this map via
    /// [`Self::set_engine_snapshot`] before each `step()` call so
    /// every RK4 stage sees the same snapshot. Empty `BTreeMap` for
    /// legacy single-motor scenarios — the cluster adapters
    /// short-circuit on the empty view, so legacy code paths
    /// produce byte-identical Parquet.
    engine_snapshot:
        std::collections::BTreeMap<openbmp_core::EngineId, openbmp_propulsion::EngineSnapshot>,
    /// Phase-3.7: kernel-owned snapshot of per-tank state, keyed by
    /// [`openbmp_core::TankId`]. Refreshed via
    /// [`Self::set_tank_snapshot`] before each `step()` call so
    /// every RK4 stage sees the same snapshot. Empty `BTreeMap` for
    /// scenarios without `[[vehicle.assembly.tanks]]` — tank-rack
    /// adapters short-circuit on the empty view, preserving pre-3.7
    /// byte output.
    tank_snapshot: std::collections::BTreeMap<openbmp_core::TankId, TankSnapshot>,
    /// Phase-3.8: optional kernel-owned NED wind sample pushed by the
    /// runner-side `WindRack`. When `Some`, the kernel splices it
    /// into the `EnvironmentSample.wind_ned_m_s` field at every RK4
    /// stage so all four stages see the same wind. When `None` (no
    /// `[wind]` block, or `kind = "none"`), the environment sample's
    /// default-zero wind flows through, preserving pre-3.8 byte
    /// output.
    wind_sample_override: Option<nalgebra::Vector3<f64>>,
    /// Phase-3.9: kernel-owned snapshot of per-recovery-device state,
    /// keyed by [`openbmp_core::RecoveryId`]. Refreshed via
    /// [`Self::set_recovery_snapshot`] before each `step()` call so
    /// every RK4 stage sees the same snapshot. Empty `BTreeMap` for
    /// scenarios without `[[vehicle.assembly.recovery]]` — the
    /// recovery-rack adapter short-circuits on the empty view,
    /// preserving pre-3.9 byte output.
    recovery_snapshot: std::collections::BTreeMap<openbmp_core::RecoveryId, RecoverySnapshot>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum MissionStateAuthority {
    Kernel,
    FlightController,
}

/// Type alias for the point-mass kernel shape: a [`SimulationKernel`]
/// over [`PointMassState`], generic in integrator, force, mass,
/// environment, and stop-condition models.
pub type PointMassKernel<I, F, MM, E, SC> = SimulationKernel<PointMassState, I, F, MM, E, SC>;

/// Point-mass kernel with sim-owned [`SolverProfile`] dispatch.
pub type ProfiledPointMassKernel<F, MM, E, SC> =
    SimulationKernel<PointMassState, ProfiledIntegrator, F, MM, E, SC>;

impl<I, F, MM, E, SC> SimulationKernel<PointMassState, I, F, MM, E, SC>
where
    I: Integrator<PointMassState>,
    F: ForceModel<PointMassState>,
    MM: MassModel,
    E: EnvironmentModel,
    SC: StopCondition<PointMassState>,
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
    pub fn new(
        config: SimulationConfig<PointMassState, I, F, MM, E, SC>,
    ) -> Result<Self, SimulationError> {
        let dt_s = config.dt.as_seconds();
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(SimulationError::InvalidConfig {
                reason: format!("dt must be strictly positive and finite, got {dt_s} s"),
            });
        }
        config.initial_state.require_valid()?;
        assert_clean_mxcsr()?;
        let initial_time_s = config.initial_state.time.as_seconds();
        let initial_mass_kg = config.initial_state.mass.get::<kilogram>();
        Ok(Self {
            state: config.initial_state,
            initial_state: config.initial_state,
            initial_time_s,
            initial_mass_kg,
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
            mission_events_typed: Vec::new(),
            script_events_typed: Vec::new(),
            mission_graph: None,
            mission_hsm: None,
            current_phase: None,
            mission_state_authority: MissionStateAuthority::Kernel,
            pending_mission_fired: Vec::new(),
            pending_script_fired: Vec::new(),
            fired_once_events: std::collections::BTreeSet::new(),
            previous_event_scalars: None,
            effector_actuals: std::collections::BTreeMap::new(),
            engine_snapshot: std::collections::BTreeMap::new(),
            tank_snapshot: std::collections::BTreeMap::new(),
            wind_sample_override: None,
            recovery_snapshot: std::collections::BTreeMap::new(),
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
    #[allow(clippy::cast_precision_loss, clippy::too_many_lines)] // step values stay well under 2^52; Phase 3.8 added wind splice
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
        let effector_actuals = &self.effector_actuals;
        let engine_snapshot = &self.engine_snapshot;
        let tank_snapshot = &self.tank_snapshot;
        let recovery_snapshot = &self.recovery_snapshot;
        let wind_override = self.wind_sample_override;

        let derive = |s: &PointMassState,
                      t: SimTime|
         -> Result<PointMassDerivative, crate::error::ModelEvalError> {
            let mut env = environment.sample(EnvironmentQuery {
                time: t,
                position_eci: s.position,
            })?;
            if let Some(wind) = wind_override {
                env.wind_ned_m_s = wind;
            }
            let mass_kg = s.mass.get::<kilogram>();
            let force_n_eci = force_model.force_n_eci(ForceContext {
                state: s,
                environment: &env,
                mass_kg,
                time: t,
                effector_actuals: EffectorActualsView::new(effector_actuals),
                engine_snapshot: EngineSnapshotView::new(engine_snapshot),
                tank_snapshot: TankSnapshotView::new(tank_snapshot),
                recovery_snapshot: RecoverySnapshotView::new(recovery_snapshot),
            })?;
            let mass_rate_kg_s = mass_model.mass_rate_kg_s_at(MassContext {
                time: t,
                engine_snapshot: EngineSnapshotView::new(engine_snapshot),
                tank_snapshot: TankSnapshotView::new(tank_snapshot),
            })?;
            // Locked order: (force / mass) gives acceleration. We
            // tolerate a non-positive intermediate mass producing
            // NaN/Inf — the integrator's per-stage finite check
            // catches it.
            Ok(PointMassDerivative {
                velocity_m_s: s.velocity.vector,
                acceleration_m_s2: force_n_eci / mass_kg,
                mass_rate_kg_s,
            })
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

        // Phase-3.2 event evaluation. Early-exit when no events are
        // declared so legacy scenarios stay bit-stable.
        if self.has_event_bindings() {
            if self.previous_event_scalars.is_none() {
                self.previous_event_scalars = Some(crate::events::EventScalars {
                    time_s: self.state.time.as_seconds(),
                    altitude_m: self.state.position.vector.z,
                    vertical_velocity_m_s: self.state.velocity.vector.z,
                    mass_fraction: self.state.mass.get::<kilogram>() / self.initial_mass_kg,
                    dynamic_pressure_pa: 0.0,
                });
            }
            let scalars = crate::events::EventScalars {
                time_s: canonical_time_s,
                altitude_m: new_state.position.vector.z,
                vertical_velocity_m_s: new_state.velocity.vector.z,
                mass_fraction: new_state.mass.get::<kilogram>() / self.initial_mass_kg,
                // Phase-3.2: kernel does not yet wire atmosphere into
                // the trigger eval; dynamic pressure is reported as
                // 0.0 regardless of altitude. Phase 3.4 will route
                // the atmosphere model here.
                dynamic_pressure_pa: 0.0,
            };
            self.evaluate_events(scalars, next_step, SimTime::from_seconds(canonical_time_s));
        }

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

    /// Environment sample at the current state and time.
    ///
    /// This mirrors the sample shape passed to force models during a
    /// kernel derivative evaluation. Runner-side telemetry uses it when
    /// evaluating post-step force breakdowns so those evaluations do not
    /// accidentally drift from the kernel's environment semantics.
    ///
    /// # Errors
    ///
    /// Returns [`SimulationError::ModelEval`] if the environment model
    /// rejects the current state/time query.
    pub fn current_environment_sample(&self) -> Result<EnvironmentSample, SimulationError> {
        let mut env = self.environment.sample(EnvironmentQuery {
            time: self.state.time,
            position_eci: self.state.position,
        })?;
        if let Some(wind) = self.wind_sample_override {
            env.wind_ned_m_s = wind;
        }
        Ok(env)
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
// Shared event-evaluation helper (Phase 3.2)
// ---------------------------------------------------------------------

impl<S, I, F, MM, E, SC> SimulationKernel<S, I, F, MM, E, SC>
where
    S: SimState,
    I: Integrator<S>,
    F: ForceModel<S>,
    E: EnvironmentModel,
    SC: StopCondition<S>,
{
    /// Active mission phase id.
    #[must_use]
    pub const fn current_phase(&self) -> Option<crate::events::PhaseId> {
        self.current_phase
    }

    /// HAL-portable mission-action bindings view. Returns the
    /// canonical id-sorted mission bindings populated by
    /// [`Self::with_mission_split`]. Empty when no `[mission]` block
    /// is declared.
    #[must_use]
    pub fn mission_bindings(&self) -> &[crate::events::EventBinding<crate::events::MissionAction>] {
        &self.mission_events_typed
    }

    /// Phase 5.X.A: simulator-only scenario-script bindings view.
    #[must_use]
    pub fn script_bindings(
        &self,
    ) -> &[crate::events::EventBinding<crate::events::ScenarioScriptAction>] {
        &self.script_events_typed
    }

    /// Drain the per-step queue of fired mission-action events.
    pub fn drain_mission_fired_events(
        &mut self,
    ) -> Vec<crate::events::FiredEvent<crate::events::MissionAction>> {
        std::mem::take(&mut self.pending_mission_fired)
    }

    /// Drain the per-step queue of fired scenario-script-action events.
    pub fn drain_script_fired_events(
        &mut self,
    ) -> Vec<crate::events::FiredEvent<crate::events::ScenarioScriptAction>> {
        std::mem::take(&mut self.pending_script_fired)
    }

    /// Inject the externally-owned mission state published by the FC
    /// commander.
    pub fn set_external_mission_state(&mut self, phase: Option<crate::events::PhaseId>) {
        if let Some(phase) = phase {
            self.current_phase = Some(phase);
            self.mission_state_authority = MissionStateAuthority::FlightController;
        } else {
            self.mission_state_authority = MissionStateAuthority::Kernel;
        }
    }

    /// Read the externally-supplied mission state when the FC owns it.
    #[must_use]
    pub fn external_mission_state(&self) -> Option<crate::events::PhaseId> {
        if self.mission_state_authority == MissionStateAuthority::FlightController {
            self.current_phase
        } else {
            None
        }
    }

    /// Typed split-binding wiring. Accepts the FC-owned mission
    /// bindings and the simulator-owned scenario-script bindings
    /// separately and stores them as the kernel's only evaluation
    /// lists.
    ///
    /// # Errors
    ///
    /// Returns [`SimulationError::MissionGraph`] if any binding's id
    /// is duplicated across the combined list, or if the graph
    /// references an event not present in either binding list.
    pub fn with_mission_split(
        mut self,
        mut mission_events: Vec<crate::events::EventBinding<crate::events::MissionAction>>,
        mut script_events: Vec<crate::events::EventBinding<crate::events::ScenarioScriptAction>>,
        mission_graph: Option<crate::events::MissionPhaseGraph>,
        mission_hsm: Option<crate::MissionStateMachine>,
    ) -> Result<Self, SimulationError> {
        mission_events.sort_by_key(|e| e.id.value());
        script_events.sort_by_key(|e| e.id.value());

        let mut event_ids = std::collections::BTreeSet::new();
        for event in mission_events
            .iter()
            .map(|e| e.id)
            .chain(script_events.iter().map(|e| e.id))
        {
            if !event_ids.insert(event) {
                return Err(SimulationError::MissionGraph(
                    crate::events::MissionGraphError::DuplicateEvent { event },
                ));
            }
        }

        if let Some(graph) = &mission_graph {
            for (i, transition) in graph.transitions.iter().enumerate() {
                if !event_ids.contains(&transition.event) {
                    return Err(SimulationError::MissionGraph(
                        crate::events::MissionGraphError::UnknownEvent {
                            event: transition.event,
                            in_transition: i,
                        },
                    ));
                }
            }
            self.current_phase = Some(graph.initial);
        }
        if let (Some(graph), Some(hsm)) = (&mission_graph, &mission_hsm) {
            for phase in &graph.phases {
                if !hsm.contains_state(phase.id) {
                    return Err(SimulationError::MissionGraph(
                        crate::events::MissionGraphError::UnknownPhaseId {
                            phase: phase.id,
                            in_field: Cow::Borrowed("mission_hsm.states"),
                        },
                    ));
                }
            }
        }

        self.mission_events_typed = mission_events;
        self.script_events_typed = script_events;
        self.mission_graph = mission_graph;
        self.mission_hsm = mission_hsm;
        Ok(self)
    }

    /// Replace the effector-actuals snapshot consumed by per-step
    /// force / moment evaluation. Phase-3.5 contract: the runner
    /// calls this **before** every `step()` invocation, with a map
    /// keyed by deck-axis name and valued by the rack's
    /// `EffectorState.actual` for the matching effector. The
    /// snapshot is held for the entire `step()` call, so all four
    /// RK4 stages see the same value.
    ///
    /// Legacy / Schema-1 scenarios skip the call entirely; the
    /// internal map starts empty and stays empty.
    pub fn set_effector_actuals(&mut self, snapshot: std::collections::BTreeMap<String, f64>) {
        self.effector_actuals = snapshot;
    }

    /// Read-only access to the current effector-actuals snapshot.
    /// Mainly useful for tests; production callers consume the
    /// snapshot via [`crate::models::EffectorActualsView`] inside
    /// `ForceContext` / `MomentContext`.
    #[must_use]
    pub fn effector_actuals(&self) -> &std::collections::BTreeMap<String, f64> {
        &self.effector_actuals
    }

    /// Replace the per-engine snapshot consumed by force / moment /
    /// mass evaluation. Phase-3.6 contract: the runner's
    /// `EngineRack` calls this before every `step()` invocation,
    /// keyed by `EngineId` and valued by the engine's
    /// `EngineSnapshot` at the time the snapshot was taken. Held
    /// for the entire `step()` call, so all four RK4 stages see the
    /// same snapshot.
    ///
    /// Legacy single-motor scenarios skip the call; the internal
    /// map stays empty.
    pub fn set_engine_snapshot(
        &mut self,
        snapshot: std::collections::BTreeMap<
            openbmp_core::EngineId,
            openbmp_propulsion::EngineSnapshot,
        >,
    ) {
        self.engine_snapshot = snapshot;
    }

    /// Read-only access to the current per-engine snapshot.
    /// Production consumers read via [`crate::models::EngineSnapshotView`]
    /// inside `ForceContext` / `MomentContext` / `MassContext`.
    #[must_use]
    pub fn engine_snapshot(
        &self,
    ) -> &std::collections::BTreeMap<openbmp_core::EngineId, openbmp_propulsion::EngineSnapshot>
    {
        &self.engine_snapshot
    }

    /// Replace the per-tank snapshot consumed by force / moment /
    /// mass evaluation. Phase-3.7 contract: the runner's `TankRack`
    /// calls this before every `step()` invocation, keyed by
    /// [`openbmp_core::TankId`] and valued by a [`TankSnapshot`]
    /// with the tank's `mass_contribution()` and `reaction_body()`
    /// observations at the time the snapshot was taken. Held for
    /// the entire `step()` call, so all four RK4 stages see the
    /// same snapshot.
    ///
    /// Scenarios without `[[vehicle.assembly.tanks]]` skip the call;
    /// the internal map stays empty.
    pub fn set_tank_snapshot(
        &mut self,
        snapshot: std::collections::BTreeMap<openbmp_core::TankId, TankSnapshot>,
    ) {
        self.tank_snapshot = snapshot;
    }

    /// Read-only access to the current per-tank snapshot.
    #[must_use]
    pub fn tank_snapshot(&self) -> &std::collections::BTreeMap<openbmp_core::TankId, TankSnapshot> {
        &self.tank_snapshot
    }

    /// Replace the per-recovery-device snapshot consumed by force
    /// evaluation. Phase-3.9 contract: the runner's `RecoveryRack`
    /// calls this before every `step()` invocation, keyed by
    /// [`openbmp_core::RecoveryId`] and valued by a
    /// [`RecoverySnapshot`] with the device's current phase, drag
    /// area, and drag coefficient. Held for the entire `step()` call,
    /// so all four RK4 stages see the same snapshot.
    ///
    /// Scenarios without `[[vehicle.assembly.recovery]]` skip the call;
    /// the internal map stays empty and the kernel-side recovery-rack
    /// adapter short-circuits on the empty view.
    pub fn set_recovery_snapshot(
        &mut self,
        snapshot: std::collections::BTreeMap<openbmp_core::RecoveryId, RecoverySnapshot>,
    ) {
        self.recovery_snapshot = snapshot;
    }

    /// Read-only access to the current per-recovery snapshot.
    #[must_use]
    pub fn recovery_snapshot(
        &self,
    ) -> &std::collections::BTreeMap<openbmp_core::RecoveryId, RecoverySnapshot> {
        &self.recovery_snapshot
    }

    /// Replace the kernel-spliced NED wind sample (Phase 3.8). The
    /// runner's `WindRack` calls this before every `step()` so all
    /// four RK4 stages observe the same wind. Scenarios without a
    /// non-`none` `[wind]` block skip the call; the underlying
    /// environment sample's default-zero wind flows through and pre-3.8
    /// outputs stay byte-identical.
    pub fn set_wind_sample(&mut self, wind_ned_m_s: nalgebra::Vector3<f64>) {
        self.wind_sample_override = Some(wind_ned_m_s);
    }

    /// Read-only access to the current wind override.
    #[must_use]
    pub fn wind_sample(&self) -> Option<nalgebra::Vector3<f64>> {
        self.wind_sample_override
    }

    fn has_event_bindings(&self) -> bool {
        !self.mission_events_typed.is_empty() || !self.script_events_typed.is_empty()
    }

    /// Evaluate every declared event binding against a post-step
    /// `EventScalars` snapshot. Mission bindings are evaluated first,
    /// followed by simulator-only script bindings. Fired bindings are
    /// recorded into their typed drain queues, graph transitions are
    /// applied only on the pure-sim path, and the once-fired set is
    /// updated after each firing.
    #[allow(clippy::match_same_arms)] // Phase-3.2 deferred actions vs. runner-side markers
    fn evaluate_events(
        &mut self,
        scalars: crate::events::EventScalars,
        step: StepIndex,
        time: SimTime,
    ) {
        use crate::events::{EventEvalState, EventTrigger};
        let eval_state = EventEvalState {
            current: scalars,
            previous: self.previous_event_scalars,
            current_phase: self.current_phase,
        };
        let fc_owned = self.mission_state_authority == MissionStateAuthority::FlightController;
        let mut transitioned = false;
        let mission_bindings = self.mission_events_typed.clone();
        for binding in &mission_bindings {
            if binding.once && self.fired_once_events.contains(&binding.id) {
                continue;
            }
            if !binding.trigger.fired(&eval_state, time, step) {
                continue;
            }
            self.pending_mission_fired.push(crate::events::FiredEvent {
                binding_id: binding.id,
                step,
                time,
                action: binding.action.clone(),
            });
            let graph_transitioned =
                self.apply_graph_transition_for_event(binding.id, fc_owned, step, time);
            transitioned |= graph_transitioned;
            match &binding.action {
                crate::events::MissionAction::EnterState(phase) => {
                    // Phase 5.X.B: when FC owns mission state,
                    // ignore in-binding phase entries — the commander
                    // already applied them. When pure-sim, the graph
                    // transition table takes precedence; direct
                    // EnterState is only the legacy fallback for an
                    // event with no graph edge from the current state.
                    if !fc_owned && !graph_transitioned {
                        self.current_phase = Some(*phase);
                    }
                }
                crate::events::MissionAction::EmitTelemetryMarker { .. } => {
                    // Runner-side fan-out; kernel records the fire.
                }
                crate::events::MissionAction::Stop { label } => {
                    self.stopped = Some(StopReason::MissionEnded {
                        phase: self.current_phase,
                        label: label.clone(),
                    });
                }
                crate::events::MissionAction::RaiseHealthAlarm { .. }
                | crate::events::MissionAction::RequestSafeState { .. } => {
                    // Health-region demotion is the FC commander's
                    // job — the kernel records the fire in the typed
                    // pending queue (above) and the commander reads
                    // it via `drain_mission_fired_events`. Pure-sim
                    // scenarios without an FC observe the fire but
                    // do not act on it because the simulator owns no
                    // region set.
                }
            }
            if binding.once {
                self.fired_once_events.insert(binding.id);
            }
        }
        let script_bindings = self.script_events_typed.clone();
        for binding in &script_bindings {
            if binding.once && self.fired_once_events.contains(&binding.id) {
                continue;
            }
            if !binding.trigger.fired(&eval_state, time, step) {
                continue;
            }
            self.pending_script_fired.push(crate::events::FiredEvent {
                binding_id: binding.id,
                step,
                time,
                action: binding.action.clone(),
            });
            transitioned |= self.apply_graph_transition_for_event(binding.id, fc_owned, step, time);
            if binding.once {
                self.fired_once_events.insert(binding.id);
            }
        }
        if !transitioned {
            self.fire_active_state_actions(step, time);
        }
        self.previous_event_scalars = Some(scalars);
    }

    fn apply_graph_transition_for_event(
        &mut self,
        event: crate::events::EventId,
        fc_owned: bool,
        step: StepIndex,
        time: SimTime,
    ) -> bool {
        if fc_owned {
            return false;
        }
        let graph_transition_to = self.mission_graph.as_ref().and_then(|graph| {
            self.current_phase.and_then(|current_phase| {
                graph
                    .transitions
                    .iter()
                    .find(|transition| {
                        transition.from == current_phase && transition.event == event
                    })
                    .map(|transition| transition.to)
            })
        });
        if let Some(phase) = graph_transition_to {
            if let Some(current) = self.current_phase {
                self.fire_transition_chain_actions(current, phase, event, step, time);
            }
            self.current_phase = Some(phase);
            return true;
        }
        false
    }

    fn fire_transition_chain_actions(
        &mut self,
        from: crate::events::PhaseId,
        to: crate::events::PhaseId,
        cause: crate::events::EventId,
        step: StepIndex,
        time: SimTime,
    ) {
        let Some(hsm) = self.mission_hsm.clone() else {
            return;
        };
        for state in hsm.exit_chain(from, to) {
            for action in hsm.on_exit_actions(state) {
                self.fire_hsm_action(cause, step, time, action.clone());
            }
        }
        for state in hsm.enter_chain(from, to) {
            for action in hsm.on_entry_actions(state) {
                self.fire_hsm_action(cause, step, time, action.clone());
            }
        }
    }

    fn fire_active_state_actions(&mut self, step: StepIndex, time: SimTime) {
        let (Some(hsm), Some(current)) = (self.mission_hsm.clone(), self.current_phase) else {
            return;
        };
        for action in hsm.on_active_actions(current) {
            self.fire_hsm_action(crate::events::EventId::new(0), step, time, action.clone());
        }
    }

    fn fire_hsm_action(
        &mut self,
        cause: crate::events::EventId,
        step: StepIndex,
        time: SimTime,
        action: crate::events::MissionAction,
    ) {
        if let crate::events::MissionAction::Stop { label } = &action {
            self.stopped = Some(StopReason::MissionEnded {
                phase: self.current_phase,
                label: label.clone(),
            });
        }
        self.pending_mission_fired.push(crate::events::FiredEvent {
            binding_id: cause,
            step,
            time,
            action,
        });
    }
}

// ---------------------------------------------------------------------
// Rigid-body kernel `step()` (Phase 2.1.C)
//
// Parallel to the point-mass impl above. Differences:
//
//   - state type is `RigidBodyState`
//   - mass model is `RigidMassModel` (full MassProperties, not scalar)
//   - the derivative closure builds `RigidBodyDerivative` from
//     ForceModel + MomentModel + RigidMassModel + EnvironmentModel
//   - quaternion kinematics: `q_dot = 0.5 · q ⊗ [0, ω_body]`
//   - Euler equation: `ω_dot = I⁻¹ (M − ω × Iω − I_dot · ω)`
// ---------------------------------------------------------------------

/// Phase-2 type alias for the rigid-body kernel shape.
pub type RigidBodyKernel<I, F, MOM, MM, E, SC> =
    SimulationKernel<openbmp_state::RigidBodyState, I, F, RigidModels<MOM, MM>, E, SC>;

/// Rigid-body kernel with sim-owned [`SolverProfile`] dispatch.
pub type ProfiledRigidBodyKernel<F, MOM, MM, E, SC> = SimulationKernel<
    openbmp_state::RigidBodyState,
    ProfiledIntegrator,
    F,
    RigidModels<MOM, MM>,
    E,
    SC,
>;

/// Quaternion-magnitude tolerance for post-step validation in the
/// rigid-body kernel. Loose enough to accept the post-`project()`
/// renormalisation residue, tight enough to flag a divergent state.
const POST_STEP_QUATERNION_TOL: f64 = 1.0e-9;

/// Inertia-tensor symmetry tolerance for post-step validation.
const POST_STEP_INERTIA_TOL: f64 = 1.0e-9;

/// Sentinel model id for kernel-internal rigid-body equation checks.
/// Real Phase-3 vehicle models will supply stable model ids; this
/// avoids overloading `ModelId::default()` for an internal algebraic
/// failure.
const RIGID_BODY_EQUATIONS_MODEL_ID: openbmp_core::ModelId = openbmp_core::ModelId::new(u64::MAX);

fn mass_properties_bits_equal(
    a: &openbmp_state::MassProperties,
    b: &openbmp_state::MassProperties,
) -> bool {
    if a.mass.get::<kilogram>().to_bits() != b.mass.get::<kilogram>().to_bits() {
        return false;
    }
    for axis in 0..3 {
        if a.center_of_mass_body.vector[axis].to_bits()
            != b.center_of_mass_body.vector[axis].to_bits()
        {
            return false;
        }
    }
    for row in 0..3 {
        for col in 0..3 {
            if a.inertia_body[(row, col)].to_bits() != b.inertia_body[(row, col)].to_bits() {
                return false;
            }
        }
    }
    true
}

/// Wrapper bundle so the rigid-body kernel can carry a moment model
/// and a rigid mass model in the single `MM` slot of
/// `SimulationKernel`. Phase 3's `VehicleAssembly` work removes this
/// wrapping; for Phase 2 it keeps the kernel struct's six type
/// parameters stable.
///
/// Implementing nothing on its own — the rigid-body impl block uses
/// the bundled types directly.
#[derive(Copy, Clone, Debug)]
pub struct RigidModels<MOM, MM> {
    /// Moment model.
    pub moment_model: MOM,
    /// Rigid-body mass model.
    pub mass_model: MM,
}

impl<MOM, MM> RigidModels<MOM, MM> {
    /// Construct from a moment model and a rigid-body mass model.
    pub const fn new(moment_model: MOM, mass_model: MM) -> Self {
        Self {
            moment_model,
            mass_model,
        }
    }
}

impl<I, F, MOM, MM, E, SC>
    SimulationKernel<openbmp_state::RigidBodyState, I, F, RigidModels<MOM, MM>, E, SC>
where
    I: Integrator<openbmp_state::RigidBodyState>,
    F: ForceModel<openbmp_state::RigidBodyState>,
    MOM: crate::models::MomentModel<openbmp_state::RigidBodyState>,
    MM: crate::models::RigidMassModel,
    E: EnvironmentModel,
    SC: StopCondition<openbmp_state::RigidBodyState>,
{
    /// Construct.
    ///
    /// # Errors
    ///
    /// Same shape as the point-mass `new`: rejects non-positive `dt`,
    /// invalid initial state, or dirty MXCSR.
    pub fn new_rigid(
        config: SimulationConfig<openbmp_state::RigidBodyState, I, F, RigidModels<MOM, MM>, E, SC>,
    ) -> Result<Self, SimulationError> {
        let dt_s = config.dt.as_seconds();
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(SimulationError::InvalidConfig {
                reason: format!("dt must be strictly positive and finite, got {dt_s} s"),
            });
        }
        config
            .initial_state
            .require_valid(POST_STEP_QUATERNION_TOL, POST_STEP_INERTIA_TOL)?;
        let model_initial_props = config
            .mass_model
            .mass_model
            .mass_properties(config.initial_state.time)?;
        model_initial_props.require_valid(POST_STEP_INERTIA_TOL)?;
        if !mass_properties_bits_equal(&config.initial_state.mass_props, &model_initial_props) {
            return Err(SimulationError::InvalidConfig {
                reason: "initial rigid-body mass properties must match the rigid mass model at initial time"
                    .into(),
            });
        }
        assert_clean_mxcsr()?;
        let initial_time_s = config.initial_state.time.as_seconds();
        let initial_mass_kg = config.initial_state.mass_props.mass.get::<kilogram>();
        Ok(Self {
            state: config.initial_state,
            initial_state: config.initial_state,
            initial_time_s,
            initial_mass_kg,
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
            mission_events_typed: Vec::new(),
            script_events_typed: Vec::new(),
            mission_graph: None,
            mission_hsm: None,
            current_phase: None,
            mission_state_authority: MissionStateAuthority::Kernel,
            pending_mission_fired: Vec::new(),
            pending_script_fired: Vec::new(),
            fired_once_events: std::collections::BTreeSet::new(),
            previous_event_scalars: None,
            effector_actuals: std::collections::BTreeMap::new(),
            engine_snapshot: std::collections::BTreeMap::new(),
            tank_snapshot: std::collections::BTreeMap::new(),
            wind_sample_override: None,
            recovery_snapshot: std::collections::BTreeMap::new(),
        })
    }

    /// Advance the rigid-body kernel by one step.
    ///
    /// # Errors
    ///
    /// Same shape as the point-mass `step()`.
    #[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
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
        let next_step = match self.step_index.checked_next() {
            Ok(step) => step,
            Err(err) => {
                self.stopped = Some(StopReason::StepOverflow {
                    step: self.step_index,
                });
                return Err(SimulationError::Time(err));
            }
        };

        let force_model = &self.force_model;
        let moment_model = &self.mass_model.moment_model;
        let mass_model = &self.mass_model.mass_model;
        let environment = &self.environment;
        let effector_actuals = &self.effector_actuals;
        let engine_snapshot = &self.engine_snapshot;
        let tank_snapshot = &self.tank_snapshot;
        let recovery_snapshot = &self.recovery_snapshot;
        let wind_override = self.wind_sample_override;

        let derive = |s: &openbmp_state::RigidBodyState,
                      t: SimTime|
         -> Result<
            crate::derivative::RigidBodyDerivative,
            crate::error::ModelEvalError,
        > {
            let mut env = environment.sample(EnvironmentQuery {
                time: t,
                position_eci: s.position,
            })?;
            if let Some(wind) = wind_override {
                env.wind_ned_m_s = wind;
            }
            let mass_kg = s.mass_props.mass.get::<kilogram>();
            let force_n_eci = force_model.force_n_eci(ForceContext {
                state: s,
                environment: &env,
                mass_kg,
                time: t,
                effector_actuals: EffectorActualsView::new(effector_actuals),
                engine_snapshot: EngineSnapshotView::new(engine_snapshot),
                tank_snapshot: TankSnapshotView::new(tank_snapshot),
                recovery_snapshot: RecoverySnapshotView::new(recovery_snapshot),
            })?;
            let moment_n_m_body = moment_model.moment_n_m_body(crate::models::MomentContext {
                state: s,
                environment: &env,
                time: t,
                effector_actuals: EffectorActualsView::new(effector_actuals),
                engine_snapshot: EngineSnapshotView::new(engine_snapshot),
                tank_snapshot: TankSnapshotView::new(tank_snapshot),
            })?;
            let rate = mass_model.mass_properties_rate(t)?;

            // Quaternion kinematics: q_dot = 0.5 · q ⊗ [0, ω_body].
            let q = s.orientation.q.into_inner();
            let omega_quat = nalgebra::Quaternion::new(
                0.0,
                s.angular_velocity.vector.x,
                s.angular_velocity.vector.y,
                s.angular_velocity.vector.z,
            );
            let q_dot = q * omega_quat * 0.5;

            // Euler equation: ω_dot = I⁻¹ (M − ω × I ω − I_dot · ω).
            let inertia = s.mass_props.inertia_body;
            let i_omega = inertia * s.angular_velocity.vector;
            let omega_cross_iomega = s.angular_velocity.vector.cross(&i_omega);
            let i_dot_omega = rate.inertia_rate_body * s.angular_velocity.vector;
            let net = moment_n_m_body - omega_cross_iomega - i_dot_omega;
            let inv_inertia = inertia.try_inverse().ok_or_else(|| {
                crate::error::ModelEvalError::InvalidState {
                    model: RIGID_BODY_EQUATIONS_MODEL_ID,
                    reason: "inertia tensor is not invertible".into(),
                }
            })?;
            let omega_dot = inv_inertia * net;

            Ok(crate::derivative::RigidBodyDerivative {
                velocity_m_s_eci: s.velocity.vector,
                acceleration_m_s2_eci: force_n_eci / mass_kg,
                quaternion_rate: q_dot,
                angular_acceleration_rad_s2_body: omega_dot,
                mass_rate_kg_s: rate.mass_rate_kg_s,
                center_of_mass_rate_body_m_s: rate.center_of_mass_rate_body_m_s,
                inertia_rate_body: rate.inertia_rate_body,
            })
        };

        let raw_new = match self.integrator.advance(&self.state, derive, self.dt) {
            Ok(state) => state,
            Err(err @ (IntegratorError::NonFiniteDerivative | IntegratorError::NonFiniteState)) => {
                self.stopped = Some(StopReason::NonFiniteState { step: next_step });
                return Err(SimulationError::Integrator(err));
            }
            Err(err) => return Err(SimulationError::Integrator(err)),
        };

        let canonical_time_s = self.initial_time_s + (next_step.value() as f64) * self.dt_s;
        let new_state = raw_new.with_time(SimTime::from_seconds(canonical_time_s));

        // Phase-3.2 event evaluation. Early-exit when no events are
        // declared so legacy byte-stability is preserved.
        if self.has_event_bindings() {
            if self.previous_event_scalars.is_none() {
                self.previous_event_scalars = Some(crate::events::EventScalars {
                    time_s: self.state.time.as_seconds(),
                    altitude_m: self.state.position.vector.z,
                    vertical_velocity_m_s: self.state.velocity.vector.z,
                    mass_fraction: self.state.mass_props.mass.get::<kilogram>()
                        / self.initial_mass_kg,
                    dynamic_pressure_pa: 0.0,
                });
            }
            let scalars = crate::events::EventScalars {
                time_s: canonical_time_s,
                altitude_m: new_state.position.vector.z,
                vertical_velocity_m_s: new_state.velocity.vector.z,
                mass_fraction: new_state.mass_props.mass.get::<kilogram>() / self.initial_mass_kg,
                // Phase-3.2: see point-mass kernel comment — atmosphere
                // is not yet wired into the trigger eval.
                dynamic_pressure_pa: 0.0,
            };
            self.evaluate_events(scalars, next_step, SimTime::from_seconds(canonical_time_s));
        }

        if let Err(source) =
            new_state.require_valid(POST_STEP_QUATERNION_TOL, POST_STEP_INERTIA_TOL)
        {
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
            "rigid-body kernel step complete"
        );

        Ok(())
    }

    /// Run until the stop condition fires.
    ///
    /// # Errors
    ///
    /// Propagates the first [`SimulationError`] from any `step()` call.
    pub fn run(&mut self) -> Result<&StopReason, SimulationError> {
        while self.stopped.is_none() {
            self.step()?;
        }
        match self.stopped.as_ref() {
            Some(reason) => Ok(reason),
            None => Err(SimulationError::InvalidConfig {
                reason: "internal invariant: run() loop exited with no stop reason".into(),
            }),
        }
    }

    /// Current state.
    #[must_use]
    pub const fn current_state(&self) -> &openbmp_state::RigidBodyState {
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

    /// Environment sample at the current state and time.
    ///
    /// Mirrors the sample shape passed to force and moment models during
    /// a kernel derivative evaluation, including the Phase-3.8 wind
    /// override splice.
    ///
    /// # Errors
    ///
    /// Returns [`SimulationError::ModelEval`] if the environment model
    /// rejects the current state/time query.
    pub fn current_environment_sample(&self) -> Result<EnvironmentSample, SimulationError> {
        let mut env = self.environment.sample(EnvironmentQuery {
            time: self.state.time,
            position_eci: self.state.position,
        })?;
        if let Some(wind) = self.wind_sample_override {
            env.wind_ned_m_s = wind;
        }
        Ok(env)
    }

    /// Configured time step.
    #[must_use]
    pub const fn dt(&self) -> Duration {
        self.dt
    }

    /// Scenario seed. Downstream RNG-driven models derive deterministic
    /// streams from this.
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
    pub const fn initial_state(&self) -> &openbmp_state::RigidBodyState {
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
    use openbmp_mission::MissionState;
    use openbmp_testkit::strategies;
    use proptest::prelude::*;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    fn one_kg_drop_kernel(
        dt_s: f64,
        stop_s: f64,
    ) -> SimulationKernel<
        PointMassState,
        Rk4FixedStep,
        ConstantGravityForce,
        ConstantMass,
        NullEnvironment,
        EndTime,
    > {
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
    fn profiled_config_wires_solver_profile_into_kernel() {
        let dt = Duration::from_seconds(0.01);
        let config = SimulationConfig::from_trajectory_profile(TrajectoryProfileConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Mass::new::<kilogram>(1.0),
            ),
            solver_profile: SolverProfile::FixedStepExplicit {
                method: crate::solver_profile::ExplicitMethod::DormandPrince54,
                dt,
            },
            force_model: ZeroForce,
            mass_model: ConstantMass::new(1.0),
            environment: NullEnvironment,
            stop_condition: AlwaysContinue,
            dt,
            scenario_seed: 0x6_000,
        })
        .expect("profiled config");
        assert!(matches!(
            config.integrator,
            ProfiledIntegrator::Dopri54Fixed(_)
        ));

        let mut kernel = SimulationKernel::new(config).expect("profiled kernel");
        kernel.step().expect("profiled step");
        assert_eq!(kernel.current_step().value(), 1);
    }

    #[test]
    fn profiled_config_rejects_source_term_only_profile() {
        let err = SimulationConfig::<
            PointMassState,
            ProfiledIntegrator,
            ZeroForce,
            ConstantMass,
            NullEnvironment,
            AlwaysContinue,
        >::from_trajectory_profile(TrajectoryProfileConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Mass::new::<kilogram>(1.0),
            ),
            solver_profile: SolverProfile::ImplicitSourceTerm {
                method: crate::solver_profile::ImplicitMethod::ImplicitEuler,
                substeps: 2,
                nonlinear_tolerance: 1.0e-9,
                nonlinear_max_iter: 8,
            },
            force_model: ZeroForce,
            mass_model: ConstantMass::new(1.0),
            environment: NullEnvironment,
            stop_condition: AlwaysContinue,
            dt: Duration::from_seconds(0.01),
            scenario_seed: 0x6_001,
        })
        .unwrap_err();
        assert!(matches!(
            err,
            SolverProfileError::ProfileRoleMismatch { .. }
        ));
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

    fn zero_force_always_continue_kernel(
        dt_s: f64,
    ) -> SimulationKernel<
        PointMassState,
        Rk4FixedStep,
        ZeroForce,
        ConstantMass,
        NullEnvironment,
        AlwaysContinue,
    > {
        let config = SimulationConfig {
            initial_state: PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::new(0.0, 0.0, 1.0),
                Mass::new::<kilogram>(1.0),
            ),
            integrator: Rk4FixedStep,
            force_model: ZeroForce,
            mass_model: ConstantMass::new(1.0),
            environment: NullEnvironment,
            stop_condition: AlwaysContinue,
            dt: Duration::from_seconds(dt_s),
            scenario_seed: 1,
        };
        SimulationKernel::new(config).expect("construct")
    }

    #[test]
    fn event_crossing_between_initial_state_and_first_step_fires() {
        let event_id = crate::events::EventId::from_path("mission.events.stop_half_second");
        let events = vec![crate::events::EventBinding {
            id: event_id,
            trigger: crate::events::BuiltInEventTrigger::AtTime { time_s: 0.5 },
            action: crate::events::MissionAction::Stop {
                label: "half-second".to_owned(),
            },
            once: true,
        }];
        let mut kernel = zero_force_always_continue_kernel(1.0)
            .with_mission_split(events, Vec::new(), None, None)
            .expect("mission wiring");

        kernel.step().expect("step");

        assert!(matches!(
            kernel.stop_reason(),
            Some(StopReason::MissionEnded { label, .. }) if label == "half-second"
        ));
    }

    #[test]
    fn external_mission_state_authority_reuses_current_phase_storage() {
        let mut kernel = zero_force_always_continue_kernel(1.0);
        let fc_phase = crate::events::PhaseId::from_path("mission.phases.fc_owned");

        assert_eq!(kernel.current_phase(), None);
        assert_eq!(kernel.external_mission_state(), None);

        kernel.set_external_mission_state(Some(fc_phase));
        assert_eq!(kernel.current_phase(), Some(fc_phase));
        assert_eq!(kernel.external_mission_state(), Some(fc_phase));

        kernel.set_external_mission_state(None);
        assert_eq!(kernel.current_phase(), Some(fc_phase));
        assert_eq!(kernel.external_mission_state(), None);
    }

    #[test]
    fn mission_graph_transition_applies_when_event_fires_in_current_phase() {
        let ascent = crate::events::PhaseId::from_path("mission.phases.ascent");
        let descent = crate::events::PhaseId::from_path("mission.phases.descent");
        let event_id = crate::events::EventId::from_path("mission.events.at_half_second");
        let phases = vec![
            crate::events::Phase {
                id: ascent,
                label: "ascent".to_owned(),
                allowed_effectors: Vec::new(),
                allowed_engines: Vec::new(),
            },
            crate::events::Phase {
                id: descent,
                label: "descent".to_owned(),
                allowed_effectors: Vec::new(),
                allowed_engines: Vec::new(),
            },
        ];
        let transitions = vec![crate::events::PhaseTransition {
            from: ascent,
            to: descent,
            event: event_id,
        }];
        let graph = crate::events::MissionPhaseGraph::new(phases, transitions, ascent, &[event_id])
            .expect("valid graph");
        let events = vec![crate::events::EventBinding {
            id: event_id,
            trigger: crate::events::BuiltInEventTrigger::AtTime { time_s: 0.5 },
            action: crate::events::MissionAction::EmitTelemetryMarker {
                tag: "at_half_second".to_owned(),
            },
            once: true,
        }];
        let mut kernel = zero_force_always_continue_kernel(1.0)
            .with_mission_split(events, Vec::new(), Some(graph), None)
            .expect("mission wiring");

        assert_eq!(kernel.current_phase(), Some(ascent));
        kernel.step().expect("step");

        assert_eq!(kernel.current_phase(), Some(descent));
        assert_eq!(kernel.drain_mission_fired_events().len(), 1);
    }

    #[test]
    fn hsm_exit_and_entry_actions_fire_on_graph_transition() {
        let ascent = crate::events::PhaseId::from_path("mission.phases.ascent");
        let descent = crate::events::PhaseId::from_path("mission.phases.descent");
        let event_id = crate::events::EventId::from_path("mission.events.at_half_second");
        let phases = vec![
            crate::events::Phase {
                id: ascent,
                label: "ascent".to_owned(),
                allowed_effectors: Vec::new(),
                allowed_engines: Vec::new(),
            },
            crate::events::Phase {
                id: descent,
                label: "descent".to_owned(),
                allowed_effectors: Vec::new(),
                allowed_engines: Vec::new(),
            },
        ];
        let transitions = vec![crate::events::PhaseTransition {
            from: ascent,
            to: descent,
            event: event_id,
        }];
        let graph = crate::events::MissionPhaseGraph::new(phases, transitions, ascent, &[event_id])
            .expect("valid graph");
        let hsm = crate::MissionStateMachine::new(
            vec![
                MissionState {
                    id: ascent,
                    label: "ascent".to_owned(),
                    parent: None,
                    on_entry: Vec::new(),
                    on_exit: vec![crate::events::MissionAction::EmitTelemetryMarker {
                        tag: "exit_ascent".to_owned(),
                    }],
                    on_active: Vec::new(),
                    allowed_effectors: Vec::new(),
                    allowed_engines: Vec::new(),
                },
                MissionState {
                    id: descent,
                    label: "descent".to_owned(),
                    parent: None,
                    on_entry: vec![crate::events::MissionAction::EmitTelemetryMarker {
                        tag: "enter_descent".to_owned(),
                    }],
                    on_exit: Vec::new(),
                    on_active: Vec::new(),
                    allowed_effectors: Vec::new(),
                    allowed_engines: Vec::new(),
                },
            ],
            ascent,
        )
        .expect("valid hsm");
        let events = vec![crate::events::EventBinding {
            id: event_id,
            trigger: crate::events::BuiltInEventTrigger::AtTime { time_s: 0.5 },
            action: crate::events::MissionAction::EmitTelemetryMarker {
                tag: "at_half_second".to_owned(),
            },
            once: true,
        }];
        let mut kernel = zero_force_always_continue_kernel(1.0)
            .with_mission_split(events, Vec::new(), Some(graph), Some(hsm))
            .expect("mission wiring");

        kernel.step().expect("step");

        assert_eq!(kernel.current_phase(), Some(descent));
        let fired = kernel.drain_mission_fired_events();
        let tags: Vec<&str> = fired
            .iter()
            .filter_map(|event| match &event.action {
                crate::events::MissionAction::EmitTelemetryMarker { tag } => Some(tag.as_str()),
                crate::events::MissionAction::EnterState(_)
                | crate::events::MissionAction::Stop { .. }
                | crate::events::MissionAction::RaiseHealthAlarm { .. }
                | crate::events::MissionAction::RequestSafeState { .. } => None,
            })
            .collect();
        assert_eq!(tags, vec!["at_half_second", "exit_ascent", "enter_descent"]);
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
            DF: Fn(
                &PointMassState,
                SimTime,
            ) -> Result<PointMassDerivative, crate::error::ModelEvalError>,
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
