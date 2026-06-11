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

use openbmp_scenario::{Scenario, ScenarioDocument};
use openbmp_sim::{MissionAction, PhaseId, RegionId, StopReason};
use openbmp_telemetry::TelemetryTable;

pub use crate::error::RunnerError;

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
