//! `openbmp-scenario` — OpenBMP scenario format and parser.
//!
//! Phase 1.5 ships a strict in-house TOML scenario parser. The parser
//! rejects unknown top-level tables and unknown fields in the Phase-1
//! schema, validates model names through a compile-time
//! [`ModelRegistry`], checks dimensional field suffixes and 3-vector
//! frame suffixes, rejects safety-limited operational vocabulary, and
//! resolves relative paths against the scenario file directory.
//!
//! # Module map
//!
//! - [`error`] — [`ScenarioError`].
//! - [`registry`] — [`ModelRole`], [`ModelDescriptor`], [`ModelRegistry`].
//! - [`solver`] — optional solver-profile config.
//! - [`document`] — [`ScenarioDocument`] and per-section config structs.
//! - [`scenario`] — [`Scenario`] entry point.
//!
//! Linting and tiny `require_*` helpers are crate-private.

pub mod checks;
pub mod document;
pub mod error;
pub mod files;
mod lint;
pub mod registry;
pub mod scenario;
pub mod solver;

pub use document::{
    AeroConfig, AssemblyBodyConfig, AssemblyConfig, AtmosphereConfig, BaffleModelConfig,
    BatchConfig, BodyGeometryConfig, ClusterLayoutConfig, EffectorCommandScheduleConfig,
    EffectorConfig, EffectorFaultConfig, EffectorKindConfig, EffectorLimitsConfig,
    EngineCommandConfig, EngineConfig, EngineFaultConfig, EngineKindConfig, EngineLimitsConfig,
    EnvironmentConfig, EpochConfig, EventActionConfig, EventConfig, EventTriggerConfig,
    ForcesConfig, FramesConfig, InitialSloshConfig, LocalOriginConfig, MetaConfig, MissionConfig,
    MotorConfig, MovingMassKindConfig, OpenBmpHeader, PhaseConfig, PhaseTransitionConfig,
    PropellantSpecConfig, PropulsionConfig, SUPPORTED_SCENARIO_VERSION, ScenarioDocument,
    SensorConfig, TankConfig, TankGeometryConfig, TelemetryConfig, TelemetryOutputConfig,
    TimeConfig, ValidationConfig, VehicleConfig, WGS84_J2_DEFAULT, WindConfig,
};
pub use error::ScenarioError;
pub use files::ResolvedFile;
pub use registry::{ModelDescriptor, ModelRegistry, ModelRole};
pub use scenario::Scenario;
pub use solver::{AdaptiveSolverConfig, SolverConfig, SourceTermSolverConfig};
