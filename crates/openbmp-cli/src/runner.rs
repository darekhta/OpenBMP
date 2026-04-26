//! Scenario → kernel → telemetry adapter for the Phase-1 CLI.
//!
//! This module is the only place that knows how to assemble a
//! [`SimulationKernel`] from a parsed [`Scenario`]. The Phase-1.7
//! adapter is intentionally narrow.
//!
//! Accepted scenario shape:
//!
//! - `vehicle.kind = "point_mass"`
//! - `environment.gravity = "constant"`
//! - `environment.atmosphere = "none"`
//! - `environment.wind = "none"`
//! - `forces = ["gravity"]`
//!
//! Anything else is rejected with [`CliError::UnsupportedScenario`].
//! Phase-2 broadens the surface.
//!
//! The runner records every kernel step into a [`TelemetryTable`] with
//! seven channels: `position_x_m`, `position_y_m`, `position_z_m`
//! (frame `ECI`), `velocity_x_m_s`, `velocity_y_m_s`, `velocity_z_m_s`
//! (frame `ECI`), and `mass_kg`. Step `0` is the initial state.
//!
//! Determinism: the runner only feeds the kernel; it does not reorder
//! channels or mutate floats. Telemetry channel ordering is fixed in
//! [`build_schema`].

use openbmp_core::{ChannelId, Duration, Position3, SimTime, Velocity3};
use openbmp_scenario::Scenario;
use openbmp_sim::{
    AlwaysContinue, ConstantGravityForce, ConstantMass, EndTime, NullEnvironment, Rk4FixedStep,
    SimulationConfig, SimulationKernel, StopReason,
};
use openbmp_state::PointMassState;
use openbmp_telemetry::{
    ChannelMetadata, TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable,
    TelemetryValueKind,
};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::CliError;

/// Concrete kernel type assembled by the Phase-1 runner.
pub type Phase1Kernel = SimulationKernel<
    Rk4FixedStep,
    ConstantGravityForce,
    ConstantMass,
    NullEnvironment,
    Phase1StopCondition,
>;

/// One of the two stop-condition shapes the Phase-1 runner produces:
/// scenario-driven `EndTime`, or `AlwaysContinue` when the test driver
/// will stop the kernel manually. Modelled as an enum (rather than two
/// kernel monomorphisations) to keep the public CLI types stable.
#[derive(Copy, Clone, Debug)]
pub enum Phase1StopCondition {
    /// Kernel halts at or after this simulation time.
    EndTime(EndTime),
    /// Kernel never halts on its own.
    AlwaysContinue(AlwaysContinue),
}

impl openbmp_sim::StopCondition for Phase1StopCondition {
    fn evaluate(
        &self,
        state: &PointMassState,
        step: openbmp_core::StepIndex,
    ) -> Option<StopReason> {
        match self {
            Self::EndTime(condition) => condition.evaluate(state, step),
            Self::AlwaysContinue(condition) => condition.evaluate(state, step),
        }
    }
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

/// Build the Phase-1 telemetry schema (fixed channel order).
fn build_schema() -> Result<TelemetrySchema, CliError> {
    let channels = vec![
        ChannelMetadata::new(
            ChannelId::new(1),
            "position_x_m",
            "m",
            Some("ECI"),
            TelemetryValueKind::Float64,
        )?,
        ChannelMetadata::new(
            ChannelId::new(2),
            "position_y_m",
            "m",
            Some("ECI"),
            TelemetryValueKind::Float64,
        )?,
        ChannelMetadata::new(
            ChannelId::new(3),
            "position_z_m",
            "m",
            Some("ECI"),
            TelemetryValueKind::Float64,
        )?,
        ChannelMetadata::new(
            ChannelId::new(4),
            "velocity_x_m_s",
            "m/s",
            Some("ECI"),
            TelemetryValueKind::Float64,
        )?,
        ChannelMetadata::new(
            ChannelId::new(5),
            "velocity_y_m_s",
            "m/s",
            Some("ECI"),
            TelemetryValueKind::Float64,
        )?,
        ChannelMetadata::new(
            ChannelId::new(6),
            "velocity_z_m_s",
            "m/s",
            Some("ECI"),
            TelemetryValueKind::Float64,
        )?,
        ChannelMetadata::new(
            ChannelId::new(7),
            "mass_kg",
            "kg",
            None::<&str>,
            TelemetryValueKind::Float64,
        )?,
    ];
    Ok(TelemetrySchema::new(channels)?)
}

/// Build the Phase-1 kernel from a scenario document.
///
/// # Errors
///
/// Returns [`CliError::UnsupportedScenario`] when the scenario uses a
/// model the Phase-1 runner does not yet wire (any vehicle other than
/// `point_mass`, any non-`constant` gravity, anything but `none` for
/// atmosphere or wind, or any force list other than `["gravity"]`).
/// Returns [`CliError::Simulation`] when the kernel rejects the
/// configuration (invalid `dt`, invalid initial state, dirty FP env).
pub fn build_kernel(scenario: &Scenario) -> Result<Phase1Kernel, CliError> {
    let document = &scenario.document;

    // Vehicle: only `point_mass` is wired in Phase 1.
    if document.vehicle.kind != "point_mass" {
        return Err(CliError::UnsupportedScenario {
            what: format!("vehicle.kind = {}", document.vehicle.kind),
        });
    }

    // Environment: only the analytic-toy combination.
    if document.environment.gravity != "constant" {
        return Err(CliError::UnsupportedScenario {
            what: format!("environment.gravity = {}", document.environment.gravity),
        });
    }
    if document.environment.atmosphere != "none" {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "environment.atmosphere = {}",
                document.environment.atmosphere
            ),
        });
    }
    if document.environment.wind != "none" {
        return Err(CliError::UnsupportedScenario {
            what: format!("environment.wind = {}", document.environment.wind),
        });
    }

    // Forces: only `["gravity"]`.
    if document.forces.models.as_slice() != ["gravity".to_owned()].as_slice() {
        return Err(CliError::UnsupportedScenario {
            what: format!("forces.models = {:?}", document.forces.models),
        });
    }

    let g = document
        .environment
        .gravity_m_s2
        .ok_or_else(|| CliError::UnsupportedScenario {
            what: "environment.gravity_m_s2 missing for constant gravity".to_owned(),
        })?;

    let initial_position = document.vehicle.initial_position_eci_m;
    let initial_velocity = document.vehicle.initial_velocity_eci_m_s;
    let initial_state = PointMassState::new(
        SimTime::from_seconds(document.time.start_s),
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
        Mass::new::<kilogram>(document.vehicle.mass_kg),
    );

    let stop =
        Phase1StopCondition::EndTime(EndTime::new(SimTime::from_seconds(document.time.stop_s)));

    let config = SimulationConfig {
        initial_state,
        integrator: Rk4FixedStep,
        force_model: ConstantGravityForce::down_z(g),
        mass_model: ConstantMass::new(document.vehicle.mass_kg),
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
/// Returns [`CliError::UnsupportedScenario`] if the scenario uses a
/// model the Phase-1 runner cannot wire, [`CliError::Simulation`] if a
/// kernel step fails, and [`CliError::Telemetry`] if a row cannot be
/// added.
pub fn run(scenario: &Scenario) -> Result<RunOutcome, CliError> {
    let mut kernel = build_kernel(scenario)?;
    let schema = build_schema()?;
    let mut table = TelemetryTable::new(schema);

    record_step(&mut table, &kernel)?;
    while kernel.stop_reason().is_none() {
        kernel.step()?;
        record_step(&mut table, &kernel)?;
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

fn record_step(table: &mut TelemetryTable, kernel: &Phase1Kernel) -> Result<(), CliError> {
    let state = kernel.current_state();
    let mut row = TelemetryRow::new(state.time, kernel.current_step())?;

    let position_x =
        TelemetryChannel::<f64>::new(ChannelId::new(1), "position_x_m", "m", Some("ECI"))?;
    let position_y =
        TelemetryChannel::<f64>::new(ChannelId::new(2), "position_y_m", "m", Some("ECI"))?;
    let position_z =
        TelemetryChannel::<f64>::new(ChannelId::new(3), "position_z_m", "m", Some("ECI"))?;
    let velocity_x =
        TelemetryChannel::<f64>::new(ChannelId::new(4), "velocity_x_m_s", "m/s", Some("ECI"))?;
    let velocity_y =
        TelemetryChannel::<f64>::new(ChannelId::new(5), "velocity_y_m_s", "m/s", Some("ECI"))?;
    let velocity_z =
        TelemetryChannel::<f64>::new(ChannelId::new(6), "velocity_z_m_s", "m/s", Some("ECI"))?;
    let mass = TelemetryChannel::<f64>::new(ChannelId::new(7), "mass_kg", "kg", None::<&str>)?;

    row.insert(&position_x, state.position.vector.x)?;
    row.insert(&position_y, state.position.vector.y)?;
    row.insert(&position_z, state.position.vector.z)?;
    row.insert(&velocity_x, state.velocity.vector.x)?;
    row.insert(&velocity_y, state.velocity.vector.y)?;
    row.insert(&velocity_z, state.velocity.vector.z)?;
    row.insert(&mass, state.mass.get::<kilogram>())?;

    table.push_row(row)?;
    Ok(())
}
