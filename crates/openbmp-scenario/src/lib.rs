//! `openbmp-scenario` — OpenBMP scenario format and parser.
//!
//! A strict in-house TOML scenario parser. The parser
//! rejects unknown top-level tables and unknown fields in the
//! schema, validates model names through a compile-time
//! [`ModelRegistry`], checks dimensional field suffixes and 3-vector
//! frame suffixes, and resolves relative paths against the scenario
//! file directory.
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
    AeroBuildupAfterbodyConfig, AeroBuildupConfig, AeroBuildupFinsConfig, AeroBuildupGridConfig,
    AeroBuildupNoseConfig, AeroConfig, AeroFreeMolecularConfig, AeroHybridConfig,
    AeroKnudsenBridgeConfig, AeroLinearMachBridgeConfig, AeroMethodConfig,
    AeroModifiedNewtonianConfig, AeroPlumeConfig, AeroTangentConeConfig, AeroTangentWedgeConfig,
    AerothermalAblationConfig, AerothermalBackwallConfig, AerothermalConfig,
    AerothermalFayRiddellConfig, AerothermalThermalToyConfig, AssemblyBodyConfig, AssemblyConfig,
    AtmosphereConfig, BaffleModelConfig, BatchConfig, BodyGeometryConfig, ClusterLayoutConfig,
    ContactConfig, ContactFrictionLawConfig, ContactGeometryConfig, ContactNormalLawConfig,
    EffectorCommandScheduleConfig, EffectorConfig, EffectorFaultConfig, EffectorKindConfig,
    EffectorLimitsConfig, EngineCommandConfig, EngineConfig, EngineFaultConfig, EngineKindConfig,
    EngineLimitsConfig, EnginePropellantConfig, EngineThermochemicalPerformanceConfig,
    EntryCorridorConfig, EntryProfileConfig, EntryProfileMode, EnvironmentConfig, EpochConfig,
    EventConfig, EventTriggerConfig, FcActuatorChannelsConfig, FcAntiWindupConfig,
    FcAscentReferenceConfig, FcAscentReferenceMethod, FcAttitudeLoopKind, FcAttitudeMpcConfig,
    FcAutopilotAllocationConfig, FcAutopilotAllocationKind, FcAutopilotKind, FcAutopilotParams,
    FcConfig, FcEkfConfig, FcEstimatorKind, FcEstimatorLaneConfig, FcEstimatorLanesConfig,
    FcEstimatorVoterKind, FcFdirConfig, FcFdirDetectorConfig, FcFdirDetectorKind,
    FcFdirDetectorKindV5, FcFdirRedlinesConfig, FcGainsConfig, FcGravityModelKind, FcGuidanceKind,
    FcHealthConfig, FcImmConfig, FcImmModeConfig, FcIndiConfig, FcIndiFilterKind,
    FcL1AdaptiveConfig, FcLqrConfig, FcMagFieldKind, FcMekfConfig, FcPhaseAuthorityConfig,
    FcRateLoopKind, FcSchedulerConfig, FcSilFaultConfig, FcSilFaultKind, FcSilStimulusConfig,
    FcTrajectoryConfig, FcTrajectoryConfigKind, FcTrajectoryKind, FcTrajectoryWaypointConfig,
    FcTransportConfig, FcTransportFaultRuleConfig, FcTransportFaultSignalConfig,
    FcTransportFaultTransformConfig, FcTransportFaultsConfig, FcTransportModeConfig,
    FcTransportPacketDirectionConfig, FcTransportPacketFaultRuleConfig,
    FcTransportPacketTransformConfig, FcTransportQuaternionAxisConfig, FcTransportVectorAxisConfig,
    FeedModeConfig, ForcePhaseOverrideConfig, ForcesConfig, FramesConfig, FreefallRestoringConfig,
    GrainGeometryConfig, GrainPropellantConfig, GrainRegressionModeConfig, InitialSloshConfig,
    LATEST_SCENARIO_VERSION, LandingFootprintConfig, LandingFootprintDispersionConfig,
    LandingFootprintMethod, LandingFootprintMonteCarloBallisticCoefficientConfig,
    LandingFootprintMonteCarloBurnoutStateConfig, LandingFootprintMonteCarloConfig,
    LandingFootprintMonteCarloDistribution, LandingFootprintMonteCarloNestedConfig,
    LandingFootprintMonteCarloNestedMetric, LandingFootprintMonteCarloOutputConfig,
    LandingFootprintMonteCarloUncertaintyClass, LandingFootprintMonteCarloUqConfig,
    LandingFootprintMonteCarloWindConfig, LandingFootprintMonteCarloWindKind, LandingGearConfig,
    LandingGearCrushConfig, LandingGearLegConfig, LandingGearOleoConfig, LocalOriginConfig,
    MetaConfig, MissionConfig, MissionScope, MissionScopeConfig, MissionScopeKind,
    MonteCarloConfig, MonteCarloCrossEntropyConfig, MonteCarloLimitStateConfig,
    MonteCarloSubsetSimulationConfig, MotorConfig, MotorGrainConfig, MovingMassKindConfig,
    MultiBodyAttitudeTargetConfig, MultiBodyAttitudeTargetKindConfig, MultiBodyConfig,
    MultiBodyGimbalJointConfig, MultiBodyInitialLaneConfig, MultiBodyLandingControllerConfig,
    MultiBodyPropagationAuthorityConfig, MultiBodySeparationConfig,
    NozzleAmbientPressureCorrectionConfig, NozzleSeparationConfig, OpenBmpHeader, PhaseConfig,
    PhaseTransitionConfig, PropellantSpecConfig, PropulsionCavitationFaultLegConfig,
    PropulsionCavitationFaultRuleConfig, PropulsionConfig, PropulsionFaultRuleConfig,
    PropulsionFaultsConfig, PropulsionFeedNetworkConfig, PropulsionFeedNetworkControllerConfig,
    PropulsionFeedNetworkLineConfig, PropulsionFeedNetworkTurbopumpConfig,
    PropulsionMixtureRatioRunawayRuleConfig, PropulsionNozzleConfig, PropulsionPogoConfig,
    PropulsionThermochemConfig, RealtimeConfig, RealtimeModeConfig, RealtimeTaskConfig,
    RecoveryConfig, RecoveryKindConfig, RegionConfig, RegionStateConfig, SCENARIO_VERSION_V2,
    SCENARIO_VERSION_V3, SUPPORTED_SCENARIO_VERSIONS, ScenarioActionConfig, ScenarioDocument,
    ScenarioScriptConfig, ScheduleConfig, ScheduleGroupConfig, SensorConfig, StagingAnalysisConfig,
    StagingAnalysisMode, StagingAnalysisStageConfig, StateConfig, TankConfig, TankGeometryConfig,
    TankUllageConfig, TelemetryConfig, TelemetryOutputConfig, TimeConfig, TorqueAxis,
    ValidationConfig, VehicleConfig, WGS84_J2_DEFAULT, WindConfig, WindLayerConfig,
};
pub use error::ScenarioError;
pub use files::ResolvedFile;
pub use registry::{ModelDescriptor, ModelRegistry, ModelRole};
pub use scenario::{Scenario, ScenarioFileRead};
pub use solver::{AdaptiveSolverConfig, SolverConfig, SourceTermSolverConfig};
