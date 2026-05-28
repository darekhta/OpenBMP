//! Runner error type.
//!
//! Wraps every model- and kernel-side failure the runner can surface
//! while building or driving a simulation from a parsed scenario. The
//! CLI wraps this in its own error so the binary can map any failure
//! to a stable exit code.

use openbmp_aero::AeroError;
use openbmp_aerothermal::AerothermalError;
use openbmp_physics::PhysicsError;
use openbmp_propulsion::MotorError;
use openbmp_scenario::ScenarioError;
use openbmp_sim::SimulationError;
use openbmp_telemetry::TelemetryError;
use thiserror::Error;

/// Errors surfaced while building or running a scenario.
#[derive(Debug, Error)]
pub enum RunnerError {
    /// The scenario file could not be resolved (SHA-256 pin failure,
    /// missing referenced file, or re-validation failure).
    #[error("scenario error")]
    Scenario(#[from] ScenarioError),
    /// The parsed scenario could not be resolved into a vehicle assembly.
    #[error("vehicle assembly error at {field}: {reason}")]
    Assembly {
        /// Scenario field path where resolution failed.
        field: String,
        /// Human-readable assembly construction failure.
        reason: String,
    },
    /// A control effector failed to construct or step.
    #[error("effector error at {field}: {reason}")]
    Effector {
        /// Scenario field path where the failure occurred.
        field: String,
        /// Human-readable effector failure.
        reason: String,
    },
    /// An engine failed to construct, command, or step.
    #[error("engine error at {field}: {reason}")]
    Engine {
        /// Scenario field path where the failure occurred.
        field: String,
        /// Human-readable engine failure.
        reason: String,
    },
    /// A tank failed to construct, drain, or step.
    #[error("tank error at {field}: {reason}")]
    Tank {
        /// Scenario field path where the failure occurred.
        field: String,
        /// Human-readable tank failure.
        reason: String,
    },
    /// A recovery device failed to construct or accept a deploy /
    /// stow command.
    #[error("recovery error at {field}: {reason}")]
    Recovery {
        /// Scenario field path where the failure occurred.
        field: String,
        /// Human-readable recovery failure.
        reason: String,
    },
    /// The kernel could not be constructed or stepped.
    #[error("simulation error")]
    Simulation(#[from] SimulationError),
    /// A telemetry channel, schema, or exporter failed.
    #[error("telemetry error")]
    Telemetry(#[from] TelemetryError),
    /// The scenario uses a model or shape the runner does not support.
    #[error("unsupported scenario: {what}")]
    UnsupportedScenario {
        /// Human-readable description of the unsupported feature.
        what: String,
    },
    /// An aerodynamic-deck loader or sample evaluation failed.
    #[error("aerodynamic deck error")]
    Aero(#[from] AeroError),
    /// A live aerothermal model construction or evaluation failed.
    #[error("aerothermal error")]
    Aerothermal(#[from] AerothermalError),
    /// A schema-2 aero deck axis could not be matched against a
    /// scenario effector at runner build time.
    #[error("aero/effector mismatch at {field}: {reason}")]
    AeroEffectorMismatch {
        /// Scenario or deck field path where the mismatch was
        /// detected.
        field: String,
        /// Human-readable description of the mismatch (missing
        /// effector, unit suffix mismatch, etc.).
        reason: String,
    },
    /// A motor-file loader or thrust-curve evaluation failed.
    #[error("motor error")]
    Motor(#[from] MotorError),
    /// A physics-model construction or evaluation failed.
    #[error("physics error")]
    Env(#[from] PhysicsError),
}

impl RunnerError {
    /// Stable exit code contribution for the binary.
    ///
    /// Configuration / model / scenario failures map to `2`; kernel,
    /// telemetry, and everything else map to `4`. The CLI delegates to
    /// this for the wrapped-runner case.
    #[must_use]
    pub const fn exit_code(&self) -> u8 {
        match self {
            Self::Scenario(_)
            | Self::Assembly { .. }
            | Self::Effector { .. }
            | Self::Engine { .. }
            | Self::Tank { .. }
            | Self::Recovery { .. }
            | Self::UnsupportedScenario { .. }
            | Self::Aero(_)
            | Self::Aerothermal(_)
            | Self::AeroEffectorMismatch { .. }
            | Self::Motor(_)
            | Self::Env(_) => 2,
            Self::Simulation(_) | Self::Telemetry(_) => 4,
        }
    }
}
