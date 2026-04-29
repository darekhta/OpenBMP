//! Scenario → kernel → telemetry dispatcher.
//!
//! Three runner paths, selected by scenario shape:
//!
//! - [`phase1`] — byte-stable analytic-toy: `vehicle.kind = "point_mass"`
//!   with `gravity = "constant"`, no Phase-2 structured blocks, and
//!   `forces = ["gravity"]`. Telemetry layout is the seven-channel
//!   schema pinned by `crates/openbmp-cli/tests/expected/`.
//! - [`phase2_point_mass`] — Phase-2 point-mass with Phase-2.10
//!   structured blocks: `[aero]`, `[propulsion.motor]`, USSA76
//!   atmosphere. Force-model order is the scenario-declared
//!   [`forces.models`](openbmp_scenario::ForcesConfig) order, which is
//!   the determinism contract.
//! - [`phase2_rigid_body`] — Phase-3.1 rigid-body path. Same scenario
//!   shape as `phase2_point_mass` but with
//!   `vehicle.kind = "rigid_body"` plus the parser-required
//!   `initial_quaternion_body_to_eci_xyzw`,
//!   `initial_angular_velocity_body_rad_s`, and
//!   `inertia_tensor_body_kg_m2` fields. Wires the rigid-body
//!   adapter family (`GravityForceAdapter` /
//!   `MotorThrustForceAdapter` / `AxialDragForceAdapter` over
//!   `RigidBodyState`, plus `RigidMotorMassAdapter`) into a
//!   `RigidBodyKernel` with `ZeroMoment`. Wind, body-frame moments,
//!   and rigid-body aero side-force / pitching moment land in
//!   Phase 3.4 / 3.5 / 3.8.
//!
//! Pin verification: when the scenario references external files
//! (`[aero].deck`, `[propulsion.motor].file`,
//! `[sensors.<name>].file`), the dispatcher calls
//! [`Scenario::resolved_files`] before any kernel state advances. A
//! malformed pin, missing file, or mismatch fails closed with a
//! [`CliError::Scenario`] (exit code 2).

pub mod aero_effector_match;
pub mod assembly;
pub mod effectors;
pub mod engines;
pub mod mission;
pub mod phase1;
pub mod phase2_point_mass;
pub mod phase2_rigid_body;
pub mod recovery;
pub mod tanks;
pub mod wind;

use openbmp_scenario::Scenario;
use openbmp_sim::StopReason;
use openbmp_telemetry::TelemetryTable;

use crate::error::CliError;

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
/// - [`CliError::Scenario`] for parse / SHA-256 pin failures (raised
///   before kernel construction).
/// - [`CliError::UnsupportedScenario`] when the scenario shape does
///   not match any wired runner path.
/// - [`CliError::Aero`], [`CliError::Motor`], [`CliError::Env`] for
///   loader-side failures inside the Phase-2 path.
/// - [`CliError::Simulation`] / [`CliError::Telemetry`] for kernel- or
///   telemetry-side failures.
pub fn run(scenario: &Scenario) -> Result<RunOutcome, CliError> {
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
        && document.environment.gravity == "constant"
        && document.environment.atmosphere == "none"
        && document.environment.wind == "none"
        && document.forces.models.len() == 1
        && document.forces.models[0] == "gravity"
        && document.aero.is_none()
        && document.propulsion.is_none()
        && document.wind.is_none()
        && document.atmosphere.is_none()
        && !has_effectors
        && !has_engines
}
