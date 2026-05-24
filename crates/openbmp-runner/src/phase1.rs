//! Scenario → kernel → telemetry adapter for the byte-stable Phase-1
//! analytic-toy path.
//!
//! Accepted scenario shape:
//!
//! - `vehicle.kind = "point_mass"`
//! - `environment.gravity = "constant"`
//! - `environment.atmosphere = "none"`
//! - `environment.wind = "none"`
//! - `forces = ["gravity"]`
//! - no `[aero]` / `[propulsion]` / `[wind]` / `[atmosphere]` blocks
//!
//! Phase-2 scenarios that declare any of the structured Phase-2.10
//! blocks (or use `vehicle.kind = "rigid_body"`) are routed through
//! [`crate::phase2_point_mass`] (or rejected with
//! `UnsupportedScenario` for `rigid_body`). This module is the
//! byte-stable analytic-toy runner; do not extend it without
//! updating the Phase-1 byte-stability baseline.
//!
//! Records every kernel step into a [`TelemetryTable`] with seven
//! channels: `position_x_m`, `position_y_m`, `position_z_m` (frame
//! `ECI`), `velocity_x_m_s`, `velocity_y_m_s`, `velocity_z_m_s`
//! (frame `ECI`), and `mass_kg`. Step `0` is the initial state.
//!
//! Determinism: the runner only feeds the kernel; it does not reorder
//! channels or mutate floats. Telemetry channel ordering is fixed in
//! `Phase1TelemetryChannels::schema`.

use openbmp_core::{ChannelId, Duration, Position3, SimTime, Velocity3};
use openbmp_scenario::Scenario;
use openbmp_sim::{
    ConstantGravityForce, ConstantMass, EndTime, NullEnvironment, Rk4FixedStep, SimulationConfig,
    SimulationKernel, StopReason,
};
use openbmp_state::PointMassState;
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::RunnerError;
use crate::RunOutcome;
use crate::assembly::dry_mass_kg_at;
use openbmp_vehicle::Assembly;

/// Concrete kernel type assembled by the Phase-1 runner.
pub type Phase1Kernel = SimulationKernel<
    PointMassState,
    Rk4FixedStep,
    ConstantGravityForce,
    ConstantMass,
    NullEnvironment,
    EndTime,
>;

#[derive(Debug)]
struct Phase1TelemetryChannels {
    position_x: TelemetryChannel<f64>,
    position_y: TelemetryChannel<f64>,
    position_z: TelemetryChannel<f64>,
    velocity_x: TelemetryChannel<f64>,
    velocity_y: TelemetryChannel<f64>,
    velocity_z: TelemetryChannel<f64>,
    mass: TelemetryChannel<f64>,
}

impl Phase1TelemetryChannels {
    fn new() -> Result<Self, RunnerError> {
        Ok(Self {
            position_x: TelemetryChannel::<f64>::new(
                ChannelId::new(1),
                "position_x_m",
                "m",
                Some("ECI"),
            )?,
            position_y: TelemetryChannel::<f64>::new(
                ChannelId::new(2),
                "position_y_m",
                "m",
                Some("ECI"),
            )?,
            position_z: TelemetryChannel::<f64>::new(
                ChannelId::new(3),
                "position_z_m",
                "m",
                Some("ECI"),
            )?,
            velocity_x: TelemetryChannel::<f64>::new(
                ChannelId::new(4),
                "velocity_x_m_s",
                "m/s",
                Some("ECI"),
            )?,
            velocity_y: TelemetryChannel::<f64>::new(
                ChannelId::new(5),
                "velocity_y_m_s",
                "m/s",
                Some("ECI"),
            )?,
            velocity_z: TelemetryChannel::<f64>::new(
                ChannelId::new(6),
                "velocity_z_m_s",
                "m/s",
                Some("ECI"),
            )?,
            mass: TelemetryChannel::<f64>::new(ChannelId::new(7), "mass_kg", "kg", None::<&str>)?,
        })
    }

    /// Build the Phase-1 telemetry schema (fixed channel order).
    fn schema(&self) -> Result<TelemetrySchema, RunnerError> {
        Ok(TelemetrySchema::new(vec![
            self.position_x.metadata().clone(),
            self.position_y.metadata().clone(),
            self.position_z.metadata().clone(),
            self.velocity_x.metadata().clone(),
            self.velocity_y.metadata().clone(),
            self.velocity_z.metadata().clone(),
            self.mass.metadata().clone(),
        ])?)
    }
}

/// Build the Phase-1 kernel from a scenario document.
///
/// # Errors
///
/// Returns [`RunnerError::UnsupportedScenario`] when the scenario shape
/// does not match the byte-stable analytic-toy contract documented in
/// the module header. Returns [`RunnerError::Simulation`] when the kernel
/// rejects the configuration (invalid `dt`, invalid initial state,
/// dirty FP env).
pub fn build_kernel(scenario: &Scenario) -> Result<Phase1Kernel, RunnerError> {
    let assembly = crate::assembly::synthesize_assembly(&scenario.document)?;
    build_kernel_with_assembly(scenario, &assembly)
}

fn build_kernel_with_assembly(
    scenario: &Scenario,
    assembly: &Assembly,
) -> Result<Phase1Kernel, RunnerError> {
    let document = &scenario.document;

    // Vehicle: only `point_mass` is wired by the analytic-toy runner.
    if document.vehicle.kind != "point_mass" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("vehicle.kind = {}", document.vehicle.kind),
        });
    }

    // Environment: only the analytic-toy combination.
    if document.environment.gravity != "constant" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("environment.gravity = {}", document.environment.gravity),
        });
    }
    if document.environment.atmosphere != "none" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "environment.atmosphere = {}",
                document.environment.atmosphere
            ),
        });
    }
    if document.environment.wind != "none" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("environment.wind = {}", document.environment.wind),
        });
    }

    // Forces: only `["gravity"]`.
    if document.force_models().len() != 1 || document.force_models()[0] != "gravity" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("forces.models = {:?}", document.force_models()),
        });
    }

    // Phase-2 structured blocks must be absent on the analytic-toy
    // path. Any of them present routes through Phase-2 dispatch.
    if document.aero.is_some() {
        return Err(RunnerError::UnsupportedScenario {
            what: "[aero] block present (Phase-2 path)".to_owned(),
        });
    }
    if document.propulsion.is_some() {
        return Err(RunnerError::UnsupportedScenario {
            what: "[propulsion] block present (Phase-2 path)".to_owned(),
        });
    }
    if document.wind.is_some() || document.atmosphere.is_some() {
        return Err(RunnerError::UnsupportedScenario {
            what: "[wind] / [atmosphere] block present (Phase-2 path)".to_owned(),
        });
    }

    let g = document
        .environment
        .gravity_m_s2
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "environment.gravity_m_s2 missing for constant gravity".to_owned(),
        })?;
    if g < 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "environment.gravity_m_s2 must be a non-negative magnitude; Phase-1 constant gravity is down_z".to_owned(),
        });
    }

    let start_time = SimTime::from_seconds(document.time.start_s);
    let dry_mass_kg = dry_mass_kg_at(assembly, start_time, "vehicle.assembly")?;

    let initial_position = document.vehicle.initial_position_eci_m;
    let initial_velocity = document.vehicle.initial_velocity_eci_m_s;
    let initial_state = PointMassState::new(
        start_time,
        Position3::new(
            initial_position[0],
            initial_position[1],
            initial_position[2],
        ),
        Velocity3::new(
            initial_velocity[0],
            initial_velocity[1],
            initial_velocity[2],
        ),
        Mass::new::<kilogram>(dry_mass_kg),
    );

    let stop = EndTime::new(SimTime::from_seconds(document.time.stop_s));

    let config = SimulationConfig {
        initial_state,
        integrator: Rk4FixedStep,
        force_model: ConstantGravityForce::down_z(g),
        mass_model: ConstantMass::new(dry_mass_kg),
        environment: NullEnvironment,
        stop_condition: stop,
        dt: Duration::from_seconds(document.time.dt_s),
        scenario_seed: document.time.seed,
    };

    Ok(SimulationKernel::new(config)?)
}

/// Run a scenario through the kernel and return the populated table.
///
/// # Errors
///
/// Returns [`RunnerError::UnsupportedScenario`] if the scenario uses a
/// model the Phase-1 runner cannot wire, [`RunnerError::Simulation`] if a
/// kernel step fails, and [`RunnerError::Telemetry`] if a row cannot be
/// added.
pub fn run(scenario: &Scenario) -> Result<RunOutcome, RunnerError> {
    // Phase-3.3: resolve the scenario's vehicle composition into a
    // `Assembly` and use its dry mass for kernel construction.
    let assembly = crate::assembly::synthesize_assembly(&scenario.document)?;

    let mut kernel = build_kernel_with_assembly(scenario, &assembly)?;
    let channels = Phase1TelemetryChannels::new()?;
    let mut table = TelemetryTable::new(channels.schema()?);

    record_step(&mut table, &kernel, &channels)?;
    while kernel.stop_reason().is_none() {
        kernel.step()?;
        record_step(&mut table, &kernel, &channels)?;
    }

    let stop_reason = kernel
        .stop_reason()
        .cloned()
        .unwrap_or(StopReason::EndTime { reached_s: 0.0 });

    Ok(RunOutcome {
        final_step: kernel.current_step().value(),
        final_time_s: kernel.current_time().as_seconds(),
        stop_reason,
        table,
    })
}

fn record_step(
    table: &mut TelemetryTable,
    kernel: &Phase1Kernel,
    channels: &Phase1TelemetryChannels,
) -> Result<(), RunnerError> {
    let state = kernel.current_state();
    let mut row = TelemetryRow::new(state.time, kernel.current_step())?;

    row.insert(&channels.position_x, state.position.vector.x)?;
    row.insert(&channels.position_y, state.position.vector.y)?;
    row.insert(&channels.position_z, state.position.vector.z)?;
    row.insert(&channels.velocity_x, state.velocity.vector.x)?;
    row.insert(&channels.velocity_y, state.velocity.vector.y)?;
    row.insert(&channels.velocity_z, state.velocity.vector.z)?;
    row.insert(&channels.mass, state.mass.get::<kilogram>())?;

    table.push_row(row)?;
    Ok(())
}
