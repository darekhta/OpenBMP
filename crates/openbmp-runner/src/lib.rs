//! `openbmp-runner` — scenario → kernel → telemetry orchestration.
//!
//! Takes a parsed [`Scenario`] and builds, drives, and records a
//! simulation. The runner owns the wiring between the declarative
//! scenario format and the kernel, model, controller, and telemetry
//! crates; the `openbmp` CLI is a thin argument-parsing shell over
//! [`run`].
//!
//! Three runner paths are selected by scenario shape:
//!
//! - [`phase1`] — byte-stable analytic-toy: `point_mass` vehicle,
//!   constant gravity, gravity-only forces, no structured blocks.
//!   Emits the minimal seven-channel telemetry schema.
//! - [`phase2_point_mass`] — point-mass with structured environment,
//!   aerodynamics, and propulsion blocks. Force-model evaluation order
//!   follows the scenario-declared
//!   [`forces.models`](openbmp_scenario::ForcesConfig) order, which is
//!   part of the determinism contract.
//! - [`phase2_rigid_body`] — six-degree-of-freedom rigid-body path with
//!   the full assembly tree, engine cluster, control effectors, tanks,
//!   recovery devices, mission graph, and flight controller.
//!
//! Pin verification: when the scenario references external files
//! (`[aero].deck`, `[propulsion.motor].file`, `[sensors.<name>].file`),
//! the dispatcher calls [`Scenario::resolved_files`] before any kernel
//! state advances. A malformed pin, missing file, or mismatch fails
//! closed with a [`RunnerError::Scenario`].

pub mod error;

pub mod aero_effector_match;
pub mod assembly;
pub mod atmosphere;
pub mod effectors;
pub mod engines;
pub mod fc;
pub mod fc_bridge;
pub mod integrator;
pub mod mission;
pub mod phase1;
pub mod phase2_point_mass;
pub mod phase2_rigid_body;
pub mod recovery;
pub mod tanks;
pub mod wind;

use std::collections::BTreeMap;

use openbmp_scenario::{Scenario, ScenarioDocument};
use openbmp_sim::StopReason;
use openbmp_telemetry::TelemetryTable;

pub use crate::error::RunnerError;
pub use phase1::Phase1Kernel;

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
/// # Errors
///
/// Returns the wrapped error from whichever runner path matches:
///
/// - [`RunnerError::Scenario`] for parse / SHA-256 pin failures (raised
///   before kernel construction).
/// - [`RunnerError::UnsupportedScenario`] when the scenario shape does
///   not match any wired runner path.
/// - [`RunnerError::Aero`], [`RunnerError::Motor`], [`RunnerError::Env`] for
///   loader-side failures inside the Phase-2 path.
/// - [`RunnerError::Simulation`] / [`RunnerError::Telemetry`] for kernel- or
///   telemetry-side failures.
pub fn run(scenario: &Scenario) -> Result<RunOutcome, RunnerError> {
    // Pin verification fires before kernel construction so a bad
    // SHA-256 cannot reach the integrator. Resolved digests are then
    // threaded into the Phase-2 telemetry header so a downstream
    // Parquet reader can verify the same pins at replay time.
    let resolved_files = scenario.resolved_files()?;

    let document = &scenario.document;

    if document.vehicle.kind == "rigid_body" {
        return phase2_rigid_body::run(scenario, &resolved_files);
    }

    if is_phase1_byte_stable_shape(scenario) {
        let outcome = phase1::run(scenario)?;
        return Ok(outcome);
    }

    phase2_point_mass::run(scenario, &resolved_files)
}

/// Phase-1 byte-stable analytic-toy shape detector.
///
/// True iff the scenario uses only the byte-stable analytic-toy
/// surface: `point_mass` vehicle, `constant` gravity, `none` for
/// atmosphere/wind, single-element `["gravity"]` force list, and no
/// Phase-2 structured blocks. Anything else routes through
/// [`phase2_point_mass`] (or fails closed for `rigid_body`).
///
/// Phase-3.4 carve-out: scenarios that declare
/// `[[vehicle.assembly.effectors]]` route through
/// [`phase2_point_mass`] even when the rest of the surface is
/// phase-1-shaped, so the runner-side `EffectorRack` and the
/// effector telemetry channels are wired. The phase-1 path has no
/// effector rack — routing an effector scenario through phase-1
/// would silently drop the deflection telemetry.
fn is_phase1_byte_stable_shape(scenario: &Scenario) -> bool {
    let document = &scenario.document;
    let has_effectors = !document.vehicle.assembly.effectors.is_empty();
    let has_engines = !document.vehicle.assembly.engines.is_empty();
    document.vehicle.kind == "point_mass"
        && document.solver.is_none()
        && document.environment.gravity == "constant"
        && document.environment.atmosphere == "none"
        && document.environment.wind == "none"
        && document.force_models().len() == 1
        && document.force_models()[0] == "gravity"
        && document.aero.is_none()
        && document.propulsion.is_none()
        && document.wind.is_none()
        && document.atmosphere.is_none()
        && !has_effectors
        && !has_engines
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
