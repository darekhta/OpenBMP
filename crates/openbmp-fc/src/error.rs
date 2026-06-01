//! Typed errors for the flight controller.
//!
//! Errors are partitioned by subsystem: [`BusError`] for pub/sub
//! topic operations, [`SchedulerError`] for cyclic dispatch issues,
//! [`ParamError`] for parameter registry round-trips, [`TableError`]
//! for validated-then-activated tables, and [`ControllerError`] as the
//! top-level union returned by the [`FlightController`](crate::FlightController).
//! All variants are non-panicking; the controller fails closed by
//! propagating an error rather than aborting the binary.

use thiserror::Error;

use std::string::String;

/// Errors raised by the internal pub/sub bus.
///
/// The bus is single-writer-many-reader by construction; the variants
/// below represent contract violations that the controller catches at
/// runtime so a buggy module does not silently corrupt the topic
/// registry.
#[derive(Debug, Error)]
pub enum BusError {
    /// A subscriber attempted to read a topic that has never been
    /// registered. Returned by
    /// [`Bus::latest`](crate::bus::Bus::latest) when the topic's type
    /// is unknown to the bus.
    #[error("bus: topic `{topic_name}` is not registered")]
    UnknownTopic {
        /// Canonical topic name (`Topic::NAME`).
        topic_name: &'static str,
    },
    /// A publisher attempted to publish a topic before registering it.
    /// Returned by [`Bus::publish`](crate::bus::Bus::publish) when the
    /// topic's type is unknown to the bus.
    #[error("bus: topic `{topic_name}` cannot be published before registration")]
    UnregisteredPublish {
        /// Canonical topic name (`Topic::NAME`).
        topic_name: &'static str,
    },
    /// A topic was registered twice. The bus rejects re-registration so
    /// scenarios cannot accidentally clobber a topic's metadata at
    /// runtime.
    #[error("bus: topic `{topic_name}` is already registered")]
    DuplicateTopic {
        /// Canonical topic name (`Topic::NAME`).
        topic_name: &'static str,
    },
}

/// Errors raised by the cyclic scheduler.
#[derive(Debug, Error)]
pub enum SchedulerError {
    /// A job was registered with a non-positive period.
    #[error("scheduler: job `{job_name}` declared period_ticks = {period_ticks}, must be >= 1")]
    NonPositivePeriod {
        /// Job name as registered.
        job_name: &'static str,
        /// Declared period in ticks.
        period_ticks: u64,
    },
    /// A job was registered with a non-positive budget.
    #[error("scheduler: job `{job_name}` declared budget_us = {budget_us}, must be >= 1")]
    NonPositiveBudget {
        /// Job name as registered.
        job_name: &'static str,
        /// Declared budget in microseconds.
        budget_us: u64,
    },
    /// A job was registered with the same name as an existing job.
    #[error("scheduler: duplicate job name `{job_name}`")]
    DuplicateJob {
        /// Job name as registered.
        job_name: &'static str,
    },
}

/// Errors raised by the parameter registry.
#[derive(Debug, Error)]
pub enum ParamError {
    /// A parameter section was queried that has not been declared.
    #[error("params: section `{section}` is not declared")]
    UnknownSection {
        /// Canonical section name.
        section: &'static str,
    },
    /// A parameter section was declared twice.
    #[error("params: section `{section}` is already declared")]
    DuplicateSection {
        /// Canonical section name.
        section: &'static str,
    },
    /// A parameter section was declared with an incompatible type at
    /// runtime — invariably a programmer error.
    #[error("params: section `{section}` has incompatible type")]
    TypeMismatch {
        /// Canonical section name.
        section: &'static str,
    },
}

/// Errors raised by the table registry.
#[derive(Debug, Error)]
pub enum TableError {
    /// A table was queried that has not been declared.
    #[error("tables: table `{table}` is not declared")]
    UnknownTable {
        /// Canonical table name.
        table: &'static str,
    },
    /// A table was declared twice.
    #[error("tables: table `{table}` is already declared")]
    DuplicateTable {
        /// Canonical table name.
        table: &'static str,
    },
    /// A pending-buffer activation failed validation.
    #[error("tables: table `{table}` failed validation: {reason}")]
    ValidationFailed {
        /// Canonical table name.
        table: &'static str,
        /// Human-readable validation reason.
        reason: String,
    },
}

/// Errors raised by an [`Estimator`](crate::estimator::Estimator) implementation.
#[derive(Debug, Error)]
pub enum EstimatorError {
    /// A measurement update failed because the innovation gate
    /// rejected the measurement.
    #[error("estimator: innovation gate rejected `{measurement}` (chi^2 = {chi2:.3} > {gate:.3})")]
    InnovationGateRejected {
        /// Measurement source name.
        measurement: &'static str,
        /// Computed chi-square value.
        chi2: f64,
        /// Configured chi-square gate threshold.
        gate: f64,
    },
    /// An internal numerical step produced a non-finite value.
    #[error("estimator: non-finite value in stage `{stage}`")]
    NonFiniteState {
        /// Filter stage where the non-finite value was produced.
        stage: &'static str,
    },
    /// Filter configuration was inconsistent (e.g. negative
    /// covariance, non-PD process noise).
    #[error("estimator: invalid configuration: {reason}")]
    InvalidConfig {
        /// Human-readable reason.
        reason: String,
    },
}

/// Errors raised by an autopilot implementation.
#[derive(Debug, Error)]
pub enum AutopilotError {
    /// A gain schedule lookup did not find an entry for the active
    /// phase.
    #[error("autopilot: no gain table for phase id 0x{phase_id:016x}")]
    MissingGainSchedule {
        /// `PhaseId::value()` of the active phase.
        phase_id: u64,
    },
    /// A non-finite value was produced inside the autopilot loop.
    #[error("autopilot: non-finite value in stage `{stage}`")]
    NonFinite {
        /// Stage where the non-finite was detected.
        stage: &'static str,
    },
    /// The trajectory loop selected a `DifferentialFlatness`
    /// trajectory kind without an installed minimum-snap trajectory,
    /// or the configured trajectory failed to evaluate.
    #[error("autopilot: trajectory loop error: {reason}")]
    Trajectory {
        /// Human-readable reason from the trajectory layer.
        reason: String,
    },
}

/// Errors raised by a guidance implementation.
#[derive(Debug, Error)]
pub enum GuidanceError {
    /// Guidance configuration was inconsistent after scenario
    /// validation or direct construction.
    #[error("guidance: invalid configuration: {reason}")]
    InvalidConfig {
        /// Human-readable reason.
        reason: String,
    },
    /// A guidance law rejected the current state.
    #[error("guidance: reference generation failed: {reason}")]
    ReferenceGeneration {
        /// Human-readable reason.
        reason: String,
    },
}

/// Errors raised by the [`Commander`](crate::commander::Commander).
#[derive(Debug, Error)]
pub enum CommanderError {
    /// The mission graph rejected a transition request that did not
    /// satisfy a per-phase precondition.
    #[error("commander: transition from 0x{from:016x} to 0x{to:016x} blocked: {reason}")]
    BlockedTransition {
        /// `PhaseId` of the source phase.
        from: u64,
        /// `PhaseId` of the destination phase.
        to: u64,
        /// Human-readable reason the transition was blocked.
        reason: &'static str,
    },
    /// The commander was asked to evaluate a graph that referenced an
    /// unknown phase.
    #[error("commander: unknown phase id 0x{phase_id:016x}")]
    UnknownPhase {
        /// Unknown phase id.
        phase_id: u64,
    },
}

/// Top-level error returned by the [`FlightController::step`](crate::FlightController::step) entry point.
#[derive(Debug, Error)]
pub enum ControllerError {
    /// A bus operation failed.
    #[error(transparent)]
    Bus(#[from] BusError),
    /// A scheduler operation failed.
    #[error(transparent)]
    Scheduler(#[from] SchedulerError),
    /// A parameter registry operation failed.
    #[error(transparent)]
    Params(#[from] ParamError),
    /// A table registry operation failed.
    #[error(transparent)]
    Tables(#[from] TableError),
    /// An estimator failed.
    #[error(transparent)]
    Estimator(#[from] EstimatorError),
    /// An autopilot failed.
    #[error(transparent)]
    Autopilot(#[from] AutopilotError),
    /// A guidance job failed.
    #[error(transparent)]
    Guidance(#[from] GuidanceError),
    /// A commander failed.
    #[error(transparent)]
    Commander(#[from] CommanderError),
}
