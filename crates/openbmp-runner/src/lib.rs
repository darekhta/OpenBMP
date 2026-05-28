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
pub mod effectors;
pub mod engines;
pub mod entry;
pub mod fc;
pub mod fc_bridge;
pub mod footprint;
pub mod integrator;
pub mod mission;
pub mod point_mass;
pub mod propulsion;
pub mod recovery;
pub mod rigid_body;
pub mod tanks;
pub mod wind;

use std::collections::BTreeMap;

use openbmp_scenario::{Scenario, ScenarioDocument};
use openbmp_sim::StopReason;
use openbmp_telemetry::TelemetryTable;

pub use crate::error::RunnerError;

pub(crate) fn phase_force_overrides(document: &ScenarioDocument) -> BTreeMap<u64, Vec<String>> {
    let mut overrides = BTreeMap::new();
    let Some(forces) = &document.forces else {
        return overrides;
    };
    for override_config in &forces.phase_override {
        let phase = crate::mission::phase_id_from_scenario_text(&override_config.phase).value();
        let models = override_config
            .models
            .iter()
            .filter(|model| model.as_str() != "aerothermal_diagnostics")
            .cloned()
            .collect();
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
    // Pin verification fires before kernel construction so a bad
    // SHA-256 cannot reach the integrator. Resolved digests are then
    // threaded into the telemetry header so a downstream Parquet
    // reader can verify the same pins at replay time.
    let resolved_files = scenario.resolved_files()?;

    if scenario.document.vehicle.kind == "rigid_body" {
        return rigid_body::run(scenario, &resolved_files);
    }

    point_mass::run(scenario, &resolved_files)
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
