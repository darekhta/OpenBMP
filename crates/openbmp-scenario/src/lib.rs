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
    AcousticsAscentClassSelectionConfig, AcousticsAscentConfig, AcousticsAscentSampleConfig,
    AcousticsConfig, AcousticsDsmConfig, AcousticsDsmLiftoffStepConfig, AcousticsDsmSourceConfig,
    AcousticsFairingInternalConfig, AcousticsFairingInternalSampleConfig, AcousticsIopConfig,
    AcousticsResponseConfig, AcousticsResponseSampleConfig, AcousticsResponseSourceConfig,
    AeroBuildupAfterbodyConfig, AeroBuildupConfig, AeroBuildupFinsConfig, AeroBuildupGridConfig,
    AeroBuildupNoseConfig, AeroBuildupProtuberancesConfig, AeroConfig, AeroDeckDrawConfig,
    AeroFreeMolecularConfig, AeroHybridConfig, AeroKnudsenBridgeConfig, AeroLinearMachBridgeConfig,
    AeroMethodConfig, AeroModifiedNewtonianConfig, AeroPlumeConfig, AeroRunMatrixConfig,
    AeroTangentConeConfig, AeroTangentWedgeConfig, AerothermalAblationConfig,
    AerothermalBackwallConfig, AerothermalConfig, AerothermalElectronDensityProxyConfig,
    AerothermalFayRiddellConfig, AerothermalThermalToyConfig, AftsConfig, AftsCorridorMetricConfig,
    AftsCorridorRuleConfig, AftsGateDirectionConfig, AftsGateRuleConfig, AftsKeepInsideRuleConfig,
    AftsKeepOutRuleConfig, AftsPropagatorConfig, AftsPropagatorDragConfig,
    AftsPropagatorEarthRotationConfig, AftsPropagatorSurfaceConfig, AftsVertexConfig,
    AftsZoneColorConfig, AftsZoneRuleConfig, AssemblyBodyConfig, AssemblyConfig, AtmosphereConfig,
    BaffleModelConfig, BatchConfig, BodyGeometryConfig, ClusterLayoutConfig, CommAntennaConfig,
    CommBlackoutElectronDensitySourceConfig, CommBridgeLinkSelectionConfig,
    CommBridgePassPlanConfig, CommConfig, CommGroundSiteConfig, CommLinkConfig,
    CommPacketErrorActionConfig, CommPacketLossRateGateConfig, CommRelayAvailabilityWindowConfig,
    CommRelayConfig, CommTerrainMaskBinConfig, CommTrackingConfig, ContactConfig,
    ContactFrictionLawConfig, ContactGeometryConfig, ContactMechanismConfig,
    ContactMechanismKindConfig, ContactMechanismStopSideConfig, ContactNormalLawConfig,
    CountdownConfig, CountdownStepConfig, CountdownStepFailureConfig, DayOfLaunchConfig,
    DayOfLaunchMartinGrahamFilterConfig, DayOfLaunchMeasuredWindProfileConfig,
    DayOfLaunchSyntheticWindFamilyConfig, DayOfLaunchWindConfig, DayOfLaunchWindQcConfig,
    DayOfLaunchWindSampleConfig, DayOfLaunchWindSpliceConfig, DirectTorqueRcsConfig,
    DirectTorqueRcsMode, EffectorCommandScheduleConfig, EffectorConfig, EffectorFaultConfig,
    EffectorKindConfig, EffectorLimitsConfig, EngineCommandConfig, EngineConfig, EngineFaultConfig,
    EngineKindConfig, EngineLimitsConfig, EnginePropellantConfig,
    EngineThermochemicalPerformanceConfig, EngineTvcActuatorConfig, EntryCorridorConfig,
    EntryProfileConfig, EntryProfileMode, EnvironmentConfig, EpochConfig, EventConfig,
    EventTriggerConfig, FcActuatorChannelsConfig, FcAntiWindupConfig, FcAscentReferenceConfig,
    FcAscentReferenceMethod, FcAttitudeLoopKind, FcAttitudeMpcConfig, FcAutopilotAllocationConfig,
    FcAutopilotAllocationKind, FcAutopilotDenseEffectorConfig, FcAutopilotKind, FcAutopilotParams,
    FcConfig, FcEkfConfig, FcEngineOutTrimAutoConfig, FcEngineOutTrimCommandConfig,
    FcEngineOutTrimConfig, FcEstimatorKind, FcEstimatorLaneConfig, FcEstimatorLanesConfig,
    FcEstimatorVoterKind, FcFdirConfig, FcFdirDetectorConfig, FcFdirDetectorKind,
    FcFdirDetectorKindV5, FcFdirRedlinesConfig, FcGainsConfig, FcGravityModelKind, FcGuidanceKind,
    FcHealthConfig, FcImmConfig, FcImmModeConfig, FcIndiConfig, FcIndiFilterKind,
    FcL1AdaptiveConfig, FcLoadReliefBlendNodeConfig, FcLoadReliefConfig, FcLqrConfig,
    FcMagFieldKind, FcMekfConfig, FcOptimizedReferenceConfig, FcOptimizedReferenceSampleConfig,
    FcPhaseAuthorityConfig, FcRateLoopKind, FcRobustControllerConfig, FcSchedulerConfig,
    FcSilFaultConfig, FcSilFaultKind, FcSilStimulusConfig, FcTrajectoryConfig,
    FcTrajectoryConfigKind, FcTrajectoryKind, FcTrajectoryWaypointConfig, FcTransportConfig,
    FcTransportFaultRuleConfig, FcTransportFaultSignalConfig, FcTransportFaultTransformConfig,
    FcTransportFaultsConfig, FcTransportModeConfig, FcTransportPacketDirectionConfig,
    FcTransportPacketFaultRuleConfig, FcTransportPacketTransformConfig,
    FcTransportQuaternionAxisConfig, FcTransportVectorAxisConfig, FeedModeConfig,
    ForcePhaseOverrideConfig, ForcesConfig, FramesConfig, FreefallRestoringConfig,
    GrainGeometryConfig, GrainPropellantConfig, GrainRegressionModeConfig, InitialSloshConfig,
    LATEST_SCENARIO_VERSION, LandingFootprintConfig, LandingFootprintDispersionConfig,
    LandingFootprintMethod, LandingFootprintMonteCarloBallisticCoefficientConfig,
    LandingFootprintMonteCarloBurnoutStateConfig, LandingFootprintMonteCarloConfig,
    LandingFootprintMonteCarloDistribution, LandingFootprintMonteCarloNestedConfig,
    LandingFootprintMonteCarloNestedMetric, LandingFootprintMonteCarloOutputConfig,
    LandingFootprintMonteCarloUncertaintyClass, LandingFootprintMonteCarloUqConfig,
    LandingFootprintMonteCarloWindConfig, LandingFootprintMonteCarloWindKind, LandingGearConfig,
    LandingGearCrushConfig, LandingGearLegConfig, LandingGearOleoConfig,
    LandingGearPrismaticDofConfig, LoadsConfig, LoadsEventFlightKindConfig, LoadsEventForceConfig,
    LoadsEventProfileKindConfig, LoadsEventsConfig, LoadsRecoveryConfig, LoadsRecoveryOutputConfig,
    LoadsRecoveryOutputIdConfig, LoadsRecoveryStationConfig, LocalOriginConfig, MetaConfig,
    MissionConfig, MissionScope, MissionScopeConfig, MissionScopeKind, MonteCarloConfig,
    MonteCarloCrossEntropyConfig, MonteCarloLimitStateConfig, MonteCarloSubsetSimulationConfig,
    MotorConfig, MotorGrainConfig, MovingMassKindConfig, MultiBodyAttitudeTargetConfig,
    MultiBodyAttitudeTargetKindConfig, MultiBodyCfeGimbalLockConfig, MultiBodyCfeLoopConfig,
    MultiBodyCfeLoopJacobianEntryConfig, MultiBodyCfePlanarFourBarConfig, MultiBodyConfig,
    MultiBodyGimbalJointConfig, MultiBodyGimbalMechanismAxisConfig, MultiBodyInitialLaneConfig,
    MultiBodyLandingControllerConfig, MultiBodyPropagationAuthorityConfig,
    MultiBodySeparationConfig, NozzleAmbientPressureCorrectionConfig, NozzleSeparationConfig,
    OpenBmpHeader, PhaseConfig, PhaseTransitionConfig, PropellantSpecConfig,
    PropulsionCavitationFaultLegConfig, PropulsionCavitationFaultRuleConfig, PropulsionConfig,
    PropulsionFaultRuleConfig, PropulsionFaultsConfig, PropulsionFeedNetworkConfig,
    PropulsionFeedNetworkControllerConfig, PropulsionFeedNetworkLineConfig,
    PropulsionFeedNetworkTurbopumpConfig, PropulsionLineConfig,
    PropulsionLineDownstreamBoundaryConfig, PropulsionLineEndpointConfig,
    PropulsionLineJunctionConfig, PropulsionLineJunctionConnectionConfig,
    PropulsionLineUpstreamBoundaryConfig, PropulsionMixtureRatioRunawayRuleConfig,
    PropulsionNozzleConfig, PropulsionPogoConfig, PropulsionPogoFeedlineConfig,
    PropulsionThermochemConfig, PwpfConfig, RcsBlowdownConfig, RcsTankFeedConfig,
    RcsThrusterConfig, RealtimeConfig, RealtimeModeConfig, RealtimeTaskConfig, RecoveryConfig,
    RecoveryInflationConfig, RecoveryInflationGrowthKindConfig, RecoveryKindConfig,
    RecoveryPflanzReductionConfig, RecoveryRiserCanopyConfig, RecoveryRiserConfig,
    RecoverySequenceConfig, RegionConfig, RegionStateConfig, SCENARIO_VERSION_V2,
    SCENARIO_VERSION_V3, SUPPORTED_SCENARIO_VERSIONS, ScenarioActionConfig, ScenarioDocument,
    ScenarioScriptConfig, ScheduleConfig, ScheduleGroupConfig, SensorConfig,
    SensorHighRateImuConfig, SpecificForceSourceConfig, StagingAnalysisConfig, StagingAnalysisMode,
    StagingAnalysisStageConfig, StateConfig, TankConfig, TankGeometryConfig,
    TankPressurizationConfig, TankSafeDumpBackpressureRampConfig,
    TankSafeDumpBackpressureSourceConfig, TankSafeDumpOrificeConfig, TankSafeDumpVentConfig,
    TankSafeDumpVentThermalModelConfig, TankUllageConfig, TelemetryConfig, TelemetryOutputConfig,
    TimeConfig, TorqueAxis, ValidationConfig, VehicleConfig, VehicleModeConfig,
    VehicleModesAssetConfig, VehicleModesAssetFormatConfig, VehicleModesConfig,
    VehicleReactionConfig, VehicleTransientConfig, VehicleTransientSchemeConfig, WGS84_J2_DEFAULT,
    WindConfig, WindLayerConfig,
};
pub use error::ScenarioError;
pub use files::ResolvedFile;
pub use registry::{ModelDescriptor, ModelRegistry, ModelRole};
pub use scenario::{Scenario, ScenarioFileRead};
pub use solver::{AdaptiveSolverConfig, SolverConfig, SourceTermSolverConfig};
