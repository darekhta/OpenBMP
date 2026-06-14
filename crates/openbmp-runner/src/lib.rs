//! `openbmp-runner` — scenario → kernel → telemetry orchestration.
//!
//! Takes a parsed [`Scenario`] and builds, drives, and records a
//! simulation. The runner owns the wiring between the declarative
//! scenario format and the kernel, model, controller, and telemetry
//! crates; the `openbmp` CLI is a thin argument-parsing shell over
//! [`run`].
//!
//! Two runner paths are selected by vehicle kind:
//!
//! - [`point_mass`] — three-degree-of-freedom point-mass translation
//!   with structured environment, aerodynamics, and propulsion blocks.
//!   Force-model evaluation order follows the scenario-declared
//!   [`forces.models`](openbmp_scenario::ForcesConfig) order, which is
//!   part of the determinism contract. The analytic-toy drop (constant
//!   gravity, gravity-only forces) is the degenerate case of this path.
//! - [`rigid_body`] — six-degree-of-freedom rigid-body path with the
//!   full assembly tree, engine cluster, control effectors, tanks,
//!   recovery devices, mission graph, and flight controller.
//!
//! Pin verification: when the scenario references external files
//! (`[aero].deck`, `[propulsion.motor].file`, `[sensors.<name>].file`),
//! the dispatcher calls [`Scenario::resolved_files`] before any kernel
//! state advances. A malformed pin, missing file, or mismatch fails
//! closed with a [`RunnerError::Scenario`].

pub mod error;

pub mod aero;
pub mod aero_effector_match;
pub mod aerothermal;
pub mod assembly;
pub mod atmosphere;
pub mod celestial;
pub mod contact;
pub mod determinism;
pub mod effectors;
pub mod engines;
pub mod entry;
pub mod fc;
pub mod fc_bridge;
pub mod feed_network;
pub mod footprint;
pub mod frames;
pub mod integrator;
pub mod landing_gear;
pub mod mission;
pub mod plume;
pub mod pogo;
pub mod point_mass;
pub mod propulsion;
pub mod recovery;
pub mod rigid_body;
pub mod rt;
pub mod separated_attitude;
pub mod separated_landing;
pub mod sil;
pub mod structural;
pub mod tanks;
pub mod wind;

use std::collections::BTreeMap;

use openbmp_scenario::{ResolvedFile, Scenario, ScenarioDocument};
use openbmp_sim::{MissionAction, PhaseId, RegionId, StopReason};
use openbmp_telemetry::TelemetryTable;

pub use crate::error::RunnerError;
pub use openbmp_propulsion::EngineFault;

pub(crate) fn phase_force_overrides(document: &ScenarioDocument) -> BTreeMap<u64, Vec<String>> {
    let mut overrides = BTreeMap::new();
    let Some(forces) = &document.forces else {
        return overrides;
    };
    for override_config in &forces.phase_override {
        let phase = crate::mission::phase_id_from_scenario_text(&override_config.phase).value();
        let models = override_config.models.clone();
        overrides.insert(phase, models);
    }
    overrides
}

pub(crate) fn default_active_force_models(document: &ScenarioDocument) -> Vec<String> {
    let mut models = document.resolved_force_models();
    if !document.vehicle.assembly.tanks.is_empty()
        && !models.iter().any(|model| model == "tank_reaction")
    {
        models.push("tank_reaction".to_owned());
    }
    if !document.vehicle.assembly.recovery.is_empty()
        && !models.iter().any(|model| model == "recovery_drag")
    {
        models.push("recovery_drag".to_owned());
    }
    models
}

pub(crate) fn active_model_label(document: &ScenarioDocument, phase: Option<u64>) -> String {
    let Some(forces) = &document.forces else {
        return default_active_force_models(document).join(",");
    };
    let active = phase.and_then(|phase| {
        forces
            .phase_override
            .iter()
            .find(|override_config| {
                crate::mission::phase_id_from_scenario_text(&override_config.phase).value() == phase
            })
            .map(|override_config| override_config.models.as_slice())
    });
    active.map_or_else(
        || default_active_force_models(document).join(","),
        |models| models.join(","),
    )
}

pub(crate) fn mission_phase_label(document: &ScenarioDocument, phase: Option<u64>) -> String {
    let Some(phase) = phase else {
        return "none".to_owned();
    };
    if let Some(mission) = &document.mission {
        let phase_id = |id: &str| crate::mission::phase_id_from_scenario_text(id).value();
        for declared in &mission.phases {
            if phase_id(&declared.id) == phase {
                return declared.id.clone();
            }
        }
        for declared in &mission.states {
            if phase_id(&declared.id) == phase {
                return declared.id.clone();
            }
        }
    }
    format!("0x{phase:016x}")
}

#[derive(Clone, Debug)]
pub(crate) struct MissionRegionDeclaration {
    pub region_id: u64,
    pub channel_suffix: String,
    pub initial_state: u64,
    pub state_labels: BTreeMap<u64, String>,
}

#[derive(Clone, Debug)]
pub(crate) struct MissionRegionTraceState {
    current: BTreeMap<u64, u64>,
}

impl MissionRegionTraceState {
    pub(crate) fn new(declarations: &[MissionRegionDeclaration]) -> Self {
        Self {
            current: declarations
                .iter()
                .map(|declaration| (declaration.region_id, declaration.initial_state))
                .collect(),
        }
    }

    pub(crate) fn apply_fired_events(
        &mut self,
        fired_events: &[openbmp_sim::FiredEvent<MissionAction>],
    ) {
        for fired in fired_events {
            if let MissionAction::SetRegionState { region, state } = fired.action {
                self.current.insert(region.value(), state.value());
            }
        }
    }

    pub(crate) fn label(&self, declaration: &MissionRegionDeclaration) -> String {
        let state = self
            .current
            .get(&declaration.region_id)
            .copied()
            .unwrap_or(declaration.initial_state);
        declaration
            .state_labels
            .get(&state)
            .cloned()
            .unwrap_or_else(|| format!("0x{state:016x}"))
    }
}

pub(crate) fn mission_region_declarations(
    document: &ScenarioDocument,
) -> Vec<MissionRegionDeclaration> {
    let Some(mission) = &document.mission else {
        return Vec::new();
    };
    mission
        .regions
        .iter()
        .map(|region| {
            let region_path = mission_region_path(&region.id);
            let mut state_labels = BTreeMap::new();
            for state in &region.states {
                let state_path = mission_region_state_path(&region.id, &state.id);
                state_labels.insert(PhaseId::from_path(&state_path).value(), state_path);
            }
            let initial_path = mission_region_state_path(&region.id, &region.initial_state);
            MissionRegionDeclaration {
                region_id: RegionId::from_path(&region_path).value(),
                channel_suffix: telemetry_suffix(
                    region_path.trim_start_matches("mission.regions."),
                ),
                initial_state: PhaseId::from_path(&initial_path).value(),
                state_labels,
            }
        })
        .collect()
}

fn mission_region_path(region: &str) -> String {
    if region.starts_with("mission.regions.") {
        region.to_owned()
    } else {
        format!("mission.regions.{region}")
    }
}

fn mission_region_state_path(region: &str, state: &str) -> String {
    if state.starts_with("mission.regions.") {
        state.to_owned()
    } else if region.starts_with("mission.regions.") {
        format!("{region}.{state}")
    } else {
        format!("mission.regions.{region}.{state}")
    }
}

fn telemetry_suffix(value: &str) -> String {
    value
        .chars()
        .map(|ch| {
            if ch.is_ascii_alphanumeric() || ch == '_' || ch == '.' || ch == '-' {
                ch
            } else {
                '_'
            }
        })
        .collect()
}

/// Outcome of a scenario run.
#[derive(Debug)]
pub struct RunOutcome {
    /// Telemetry table populated step-by-step.
    pub table: TelemetryTable,
    /// Stop reason reported by the kernel.
    pub stop_reason: StopReason,
    /// Final step index.
    pub final_step: u64,
    /// Final simulation time in seconds.
    pub final_time_s: f64,
    /// Optional host realtime pacing report.
    pub realtime: Option<rt::RealtimeRunReport>,
    /// Optional FC actuator command-stream digest report.
    pub actuator_stream: Option<determinism::ActuatorStreamReport>,
    /// Optional contact diagnostics report for `[contact]` scenarios.
    pub contact: Option<contact::ContactRunReport>,
    /// Optional landing-gear touchdown report for `[vehicle.landing_gear]`
    /// scenarios.
    pub landing_gear: Option<landing_gear::LandingGearRunReport>,
}

/// Run a scenario through the appropriate kernel path.
///
/// `rigid_body` vehicles route through the six-degree-of-freedom
/// [`rigid_body`] path; every other vehicle routes through the
/// point-mass [`point_mass`] path (the analytic-toy drop is its
/// degenerate case).
///
/// # Errors
///
/// Returns the wrapped error from whichever runner path matches:
///
/// - [`RunnerError::Scenario`] for parse / SHA-256 pin failures (raised
///   before kernel construction).
/// - [`RunnerError::UnsupportedScenario`] when the scenario shape does
///   not match any wired runner path.
/// - [`RunnerError::Aero`], [`RunnerError::Motor`], [`RunnerError::Env`] for
///   loader-side failures.
/// - [`RunnerError::Simulation`] / [`RunnerError::Telemetry`] for kernel- or
///   telemetry-side failures.
pub fn run(scenario: &Scenario) -> Result<RunOutcome, RunnerError> {
    run_dispatch(scenario, None)
}

/// Run a scenario while observing the in-loop flight controller each tick.
///
/// Identical to [`run`] except that, when the scenario wires a flight
/// controller, `monitor` is invoked once per kernel tick (after the
/// controller steps, before its commands reach the racks) with a read-only
/// [`sil::FcObservation`] and the matching [`openbmp_sensors::SensorTruth`].
/// For a scenario without an `[fc]` block the monitor is never called.
///
/// This shares [`run`]'s exact code path with `monitor = None`; an installed
/// monitor only adds a read-only observation tap and cannot change the
/// telemetry bytes a plain [`run`] would produce.
///
/// # Errors
///
/// Returns the same errors as [`run`].
pub fn run_with_monitor(
    scenario: &Scenario,
    monitor: &mut dyn sil::SilMonitor,
) -> Result<RunOutcome, RunnerError> {
    run_dispatch(scenario, Some(monitor))
}

fn run_dispatch(
    scenario: &Scenario,
    monitor: Option<&mut (dyn sil::SilMonitor + '_)>,
) -> Result<RunOutcome, RunnerError> {
    // Pin verification fires before kernel construction so a bad
    // SHA-256 cannot reach the integrator. Resolved digests are then
    // threaded into the telemetry header so a downstream Parquet
    // reader can verify the same pins at replay time.
    let resolved_files = scenario.resolved_files()?;

    if scenario.document.vehicle.kind == "rigid_body" {
        return rigid_body::run(scenario, &resolved_files, monitor);
    }

    point_mass::run(scenario, &resolved_files, monitor)
}

/// A prepared rigid-body scenario run the caller drives one kernel tick
/// at a time.
///
/// [`run`] is implemented as `prepare → step* → finish` over the same
/// internals, so a session-stepped run produces a byte-identical
/// [`RunOutcome`] to a one-shot [`run`] of the same scenario — the
/// equivalence is by construction, and
/// `crates/openbmp-runner/tests/session_equivalence.rs` pins it.
///
/// Between ticks the caller may read truth state, separated bodies,
/// per-engine actuation, and the flight controller's published
/// estimates (the same read-only assembly a [`sil::SilMonitor`]
/// receives), and may inject deterministic malfunction stimuli:
/// scheduled engine faults, sensor outage windows, and a wind override.
/// Stimuli are scheduled against the kernel step clock, so a host that
/// replays the same stimulus timeline reproduces the run byte-for-byte
/// — interactive divergence is always explicit, never ambient.
///
/// The session is the simulator-side host surface for interactive and
/// non-filesystem consumers (the `openbmp-web` WebAssembly demo host
/// drives one in a browser worker). It adds no authority the one-shot
/// runner did not have: stimuli go through the same scheduled-fault and
/// stimulus seams scenario files already declare, and observation goes
/// through the same forward-only SIL boundary.
#[derive(Debug)]
pub struct Session {
    scenario: Scenario,
    inner: rigid_body::RigidBodySession,
}

impl Session {
    /// Prepare a session, resolving scenario-referenced files from the
    /// filesystem (the [`run`] resolution path).
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::UnsupportedScenario`] for non-rigid-body
    /// scenarios (the point-mass path has no session host yet) and the
    /// same scenario / loader / kernel errors as [`run`].
    pub fn prepare(scenario: Scenario) -> Result<Self, RunnerError> {
        let resolved_files = scenario.resolved_files()?;
        Self::prepare_with_files(scenario, &resolved_files)
    }

    /// Prepare a session from an already-resolved file map, for hosts
    /// without a filesystem (embedded assets, WebAssembly).
    ///
    /// `resolved_files` must come from
    /// [`Scenario::resolved_files`] or
    /// [`Scenario::resolved_files_with_reader`] so SHA-256 pin
    /// verification has already fired; both fail closed on a missing
    /// file or digest mismatch.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::UnsupportedScenario`] for non-rigid-body
    /// scenarios and the same loader / kernel errors as [`run`].
    pub fn prepare_with_files(
        scenario: Scenario,
        resolved_files: &BTreeMap<String, ResolvedFile>,
    ) -> Result<Self, RunnerError> {
        if scenario.document.vehicle.kind != "rigid_body" {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "Session::prepare supports vehicle.kind = \"rigid_body\" (got \"{}\"); \
                     use `run` for point-mass scenarios",
                    scenario.document.vehicle.kind
                ),
            });
        }
        let inner = rigid_body::RigidBodySession::prepare(&scenario, resolved_files)?;
        Ok(Self { scenario, inner })
    }

    /// Advance the run by one kernel tick and report whether the kernel
    /// has stopped. Stepping a finished session is a no-op returning
    /// `true`.
    ///
    /// # Errors
    ///
    /// Returns the same errors as the [`run`] loop body.
    pub fn step(&mut self) -> Result<bool, RunnerError> {
        if self.inner.is_finished() {
            return Ok(true);
        }
        self.inner.step_once(&self.scenario.document, None)?;
        Ok(self.inner.is_finished())
    }

    /// Whether the kernel has reported a stop reason.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.inner.is_finished()
    }

    /// Current simulation time in seconds.
    #[must_use]
    pub fn time_s(&self) -> f64 {
        self.inner.time_s()
    }

    /// Current kernel step index.
    #[must_use]
    pub fn step_index(&self) -> u64 {
        self.inner.step_index()
    }

    /// Truth rigid-body state of the continuing stack.
    #[must_use]
    pub fn state(&self) -> &openbmp_state::RigidBodyState {
        self.inner.state()
    }

    /// Separated bodies currently propagated alongside the stack.
    #[must_use]
    pub fn separated_bodies(&self) -> &[openbmp_sim::SeparatedRigidBody] {
        self.inner.separated_bodies()
    }

    /// Per-engine actuation snapshots keyed by engine id.
    #[must_use]
    pub fn engine_snapshots(
        &self,
    ) -> BTreeMap<openbmp_core::EngineId, openbmp_sim::EngineSnapshot> {
        self.inner.engine_snapshots()
    }

    /// Environment sample (atmosphere, gravity, wind) at the current
    /// state.
    ///
    /// # Errors
    ///
    /// Propagates kernel environment-evaluation failures.
    pub fn environment_sample(&self) -> Result<openbmp_sim::EnvironmentSample, RunnerError> {
        self.inner.environment_sample()
    }

    /// Scenario-declared label of the current mission phase, or
    /// `"none"` when no mission graph is wired.
    #[must_use]
    pub fn mission_phase(&self) -> String {
        mission_phase_label(
            &self.scenario.document,
            self.inner.current_phase().map(|phase| phase.value()),
        )
    }

    /// Latest flight-controller observation (estimates, estimator
    /// health, FDIR, guidance reference), when the scenario wires a
    /// controller — the same read-only assembly a [`sil::SilMonitor`]
    /// receives.
    #[must_use]
    pub fn fc_observation(&self) -> Option<sil::FcObservation> {
        self.inner.fc_observation()
    }

    /// Latest guidance cutoff estimate published by the FC.
    #[must_use]
    pub fn latest_guidance_cutoff(&self) -> Option<openbmp_fc::topics::GuidanceCutoff> {
        self.inner.latest_guidance_cutoff()
    }

    /// The scenario this session was prepared from.
    #[must_use]
    pub fn scenario(&self) -> &Scenario {
        &self.scenario
    }

    /// Drain recorded telemetry rows, bounding table memory for
    /// streaming hosts. A drained session's [`RunOutcome`] table exports
    /// an empty body — hosts that want the complete archive must not
    /// drain.
    #[must_use]
    pub fn take_recorded_rows(&mut self) -> Vec<openbmp_telemetry::TelemetryRow> {
        self.inner.take_recorded_rows()
    }

    /// Telemetry schema for interpreting drained rows.
    #[must_use]
    pub fn telemetry_schema(&self) -> &openbmp_telemetry::TelemetrySchema {
        self.inner.telemetry_schema()
    }

    /// Schedule a deterministic engine malfunction by scenario engine
    /// name, taking effect on the next kernel tick. Returns the
    /// scheduled fault id.
    ///
    /// This is the runtime twin of the scenario's `[propulsion.faults]`
    /// rules — a simulated fault-injection stimulus
    /// (`docs/safety-boundaries.md`), not a flight-controller command;
    /// the controller sees only the physical consequence.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Engine`] when the name matches no declared
    /// engine.
    pub fn inject_engine_fault(
        &mut self,
        engine: &str,
        fault: EngineFault,
    ) -> Result<String, RunnerError> {
        self.inner.schedule_engine_fault(engine, fault)
    }

    /// Arm a sensor outage window by `[sensors.<name>]` key: the sensor
    /// keeps being read (its deterministic noise stream is unchanged)
    /// but its measurements are withheld from the flight controller
    /// while `time ∈ [start_s, stop_s)`.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::UnsupportedScenario`] when the scenario
    /// has no `[fc]` block, the sensor is unknown, or the window is
    /// empty.
    pub fn inject_sensor_outage(
        &mut self,
        sensor: &str,
        start_s: f64,
        stop_s: f64,
    ) -> Result<(), RunnerError> {
        self.inner.arm_sensor_outage(sensor, start_s, stop_s)
    }

    /// Set or clear the wind override (NED metres per second). While
    /// set, it replaces the scenario wind sample each tick; clearing
    /// restores the scenario wind (zero for scenarios without a
    /// `[wind]` block).
    pub fn set_wind_override(&mut self, wind_ned_m_s: Option<[f64; 3]>) {
        self.inner
            .set_wind_override(wind_ned_m_s.map(nalgebra::Vector3::from));
    }

    /// Consume the session and assemble the [`RunOutcome`].
    ///
    /// # Errors
    ///
    /// Returns the same errors as the one-shot [`run`] tail.
    pub fn finish(self) -> Result<RunOutcome, RunnerError> {
        self.inner.finish()
    }
}

fn append_solver_metadata(document: &ScenarioDocument, metadata: &mut BTreeMap<String, String>) {
    let Some(solver) = &document.solver else {
        return;
    };
    metadata.insert(
        "openbmp.solver.profile".to_owned(),
        solver
            .profile
            .clone()
            .unwrap_or_else(|| "fixed-step-explicit".to_owned()),
    );
    metadata.insert(
        "openbmp.solver.trajectory_method".to_owned(),
        solver
            .trajectory_method
            .clone()
            .unwrap_or_else(|| "rk4".to_owned()),
    );
    metadata.insert(
        "openbmp.solver.determinism".to_owned(),
        solver
            .determinism
            .clone()
            .unwrap_or_else(|| "bit-stable".to_owned()),
    );
    if let Some(source_terms) = &solver.source_terms {
        metadata.insert(
            "openbmp.solver.source_terms.chemistry_method".to_owned(),
            source_terms.chemistry_method.clone(),
        );
        metadata.insert(
            "openbmp.solver.source_terms.chemistry_substeps".to_owned(),
            source_terms.chemistry_substeps.to_string(),
        );
        metadata.insert(
            "openbmp.solver.source_terms.material_method".to_owned(),
            source_terms.material_method.clone(),
        );
        metadata.insert(
            "openbmp.solver.source_terms.material_substeps".to_owned(),
            source_terms.material_substeps.to_string(),
        );
        metadata.insert(
            "openbmp.solver.source_terms.nonlinear_tolerance".to_owned(),
            format!("{:.17e}", source_terms.nonlinear_tolerance),
        );
        metadata.insert(
            "openbmp.solver.source_terms.nonlinear_max_iter".to_owned(),
            source_terms.nonlinear_max_iter.to_string(),
        );
    }
}

fn append_frame_time_metadata(
    document: &ScenarioDocument,
    metadata: &mut BTreeMap<String, String>,
) {
    metadata.insert(
        "openbmp.frame.profile".to_owned(),
        document.environment.frame_profile.clone(),
    );
    if let Some(epoch) = &document.epoch {
        metadata.insert("openbmp.epoch.scale".to_owned(), epoch.scale.clone());
        metadata.insert("openbmp.epoch.iso8601".to_owned(), epoch.iso8601.clone());
    }
}
