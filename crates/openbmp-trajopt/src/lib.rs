//! Offline trajectory optimization and I-load synthesis scaffolding.
//!
//! This crate is deliberately L4 / host-side. It packages the
//! forward-only terminal-condition vocabulary from
//! `openbmp-physics`, provides residual calculations for orbital /
//! inertial terminal conditions, and emits versioned postcard I-load
//! payloads for a flight controller to validate before use. It is not
//! linked by `openbmp-fc`.

#![forbid(unsafe_code)]
#![deny(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

extern crate alloc;

pub mod collocation;
pub mod corrector;
pub mod dol;
pub mod driver;
pub mod iload;
#[cfg(feature = "sqp")]
pub mod lcvx;
pub mod nlp;
pub mod pmp;
pub mod pseudospectral;
#[cfg(feature = "sqp")]
pub mod scvx;
pub mod shooting;
#[cfg(feature = "sqp")]
pub mod sqp;
pub mod stm;
pub mod target;

pub use collocation::{
    CollocationAutoDiffDynamics, CollocationAutoDiffDynamicsAdapter, CollocationAutoDiffScalar,
    CollocationDynamics, CollocationDynamicsJacobian, CollocationDynamicsJacobianSample,
    CollocationLagrange, CollocationPathFunction, HermiteSimpsonBoundaryConstraint,
    HermiteSimpsonColoredSparseResidualJacobianReport, HermiteSimpsonDecisionBoundsReport,
    HermiteSimpsonDecisionLayout, HermiteSimpsonFixedMeshOcp,
    HermiteSimpsonFixedMeshResidualReport, HermiteSimpsonFixedMeshSparseGaussNewtonStepReport,
    HermiteSimpsonFixedMeshSparseKktSqpReport, HermiteSimpsonFixedMeshSparseKktStepReport,
    HermiteSimpsonHardPathConstraint, HermiteSimpsonKktCollocationMultiplierSample,
    HermiteSimpsonKktCostateEstimateReport, HermiteSimpsonKktHamiltonianReport,
    HermiteSimpsonKktHamiltonianSample, HermiteSimpsonKktNodeCostateEstimate,
    HermiteSimpsonKktResidualMultiplierReport, HermiteSimpsonNonlinearHardPathConstraint,
    HermiteSimpsonNonlinearPathInequalityReport, HermiteSimpsonPathInequalityReport,
    HermiteSimpsonQuadraticControlCost, HermiteSimpsonReferenceProfileMapping,
    HermiteSimpsonReferenceProfileReport, HermiteSimpsonSamplePoint, HermiteSimpsonScalarVariable,
    HermiteSimpsonSegment, HermiteSimpsonSegmentReport, HermiteSimpsonSoftPathConstraint,
    HermiteSimpsonSparseAugmentedKktReport, HermiteSimpsonSparseGaussNewtonHessianReport,
    HermiteSimpsonSparseKktSolveReport, HermiteSimpsonSparseNormalSolveReport,
    HermiteSimpsonSparsePathInequalityJacobianReport, HermiteSimpsonSparseResidualJacobianReport,
    HermiteSimpsonTranscriptionReport, SparseKktSqpIterationReport, SparseKktSqpSettings,
    evaluate_hermite_simpson_segment, evaluate_hermite_simpson_transcription,
};
#[cfg(feature = "sqp")]
pub use collocation::{
    HermiteSimpsonFixedMeshDenseConstrainedSqpReport,
    HermiteSimpsonFixedMeshDenseQuadraticQpReport,
    HermiteSimpsonFixedMeshDenseSqpPmpKktCertificateReport, HermiteSimpsonFixedMeshDenseSqpReport,
    HermiteSimpsonFixedMeshFreeTimeDenseSqpReport,
    HermiteSimpsonFixedMeshFreeTimeObjectiveDenseSqpReport,
};
pub use corrector::{DifferentialCorrection, DifferentialCorrector};
pub use dol::{
    DOL_COLA_SCREENING_SCHEMA_VERSION, DOL_COMMIT_READINESS_SCHEMA_VERSION,
    DOL_LOAD_INDICATOR_SURROGATE_SCHEMA_VERSION, DOL_PERSISTENCE_KNOCKDOWN_SCHEMA_VERSION,
    DOL_REPORT_CARD_SCHEMA_VERSION, DOL_UPDATE_SCHEMA_VERSION, DolColaCatalogObject,
    DolColaEncounterReport, DolColaScreeningInput, DolColaScreeningReport, DolColaSlotReport,
    DolColaTrajectorySample, DolColaWindowSlot, DolCommitGateReport, DolCommitReadinessInput,
    DolCommitReadinessReport, DolLoadWindow, DolPersistenceBand, DolPersistenceDeck,
    DolPersistenceIndicatorInput, DolPersistenceIndicatorReport, DolPersistenceKnockdownInput,
    DolPersistenceKnockdownReport, DolPersistenceTruthSample, DolPersistenceTruthValidationInput,
    DolPersistenceTruthValidationReport, DolRedesignInput, DolReportCard, DolReportCardInput,
    DolReportVerdict, DolVerificationIndicator, DolVerificationIndicatorReport, ILoadUpdate,
    ILoadUpdateArtifact, LoadIndicatorFitInput, LoadIndicatorFitResult, LoadIndicatorSample,
    LoadIndicatorSurrogate, LoadIndicatorValidationReport, LoadIndicatorVector,
    build_dol_commit_readiness, build_dol_report_card, design_dol_iload_update,
    dol_commit_readiness_csv, dol_report_card_csv, evaluate_load_indicator_surrogate,
    evaluate_persistence_knockdowns, fit_load_indicator_surrogate, load_indicator_vector,
    screen_launch_cola, validate_persistence_truth_samples,
};
pub use driver::{TwoBodyApogeeReport, TwoBodyApogeeTargeting, correct_two_body_apogee};
pub use iload::{
    GainAxis, GainTable, ILoadHeader, ILoadObjectiveClass, ILoadOptimalityCertificate,
    ILoadOptimalityEvidenceKind, ILoadPayload, ILoadSchemaVersion, ReferenceProfileSample,
    SynthesisMetadata, TrajoptError, decode_iload_payload, encode_iload_payload,
};
#[cfg(feature = "sqp")]
pub use lcvx::{
    LcvxControlDiscretization, LcvxPoweredDescentProblem, LcvxPoweredDescentSolution,
    LcvxTaylorThrustBounds, LcvxTerminalStateTarget, LcvxTimeOfFlightSearchReport,
    LcvxTimeOfFlightSearchSettings, solve_lcvx_powered_descent_fixed_time,
    solve_lcvx_powered_descent_time_of_flight_search,
};
pub use nlp::{
    CscMatrix, DecisionLayout, NlpAssemblyReport, NlpCheckedSolveReport, NlpResidual, NlpSolution,
    NlpSolveValidationSettings, NlpSolver, OptimalControlProblem, ProblemDims, SparseVec,
    Transcription, assemble_sparse_nlp, solve_sparse_nlp_checked,
    solve_sparse_nlp_checked_with_settings,
};
#[cfg(feature = "sqp")]
pub use nlp::{DenseSqpNlpSettings, DenseSqpNlpSolver};
pub use openbmp_physics::profile::TerminalCondition;
pub use pmp::{
    PmpCostateComparisonReport, PmpCostateComparisonSample, PmpCostateComparisonSettings,
    PmpDirectCostateConvention, PmpDirectCostateConventionReport, PmpDirectCostateConventionSample,
    PmpHamiltonianControlStationarityReport, PmpHamiltonianControlStationaritySample,
    PmpHamiltonianControlStationaritySettings, PmpHamiltonianProfile, PmpHamiltonianReport,
    PmpHamiltonianSample, PmpScalarMinEnergyIteration, PmpScalarMinEnergyProblem,
    PmpScalarMinEnergyShootingSettings, PmpScalarMinEnergySolution,
    apply_pmp_direct_costate_convention, compare_pmp_costate_profiles,
    evaluate_pmp_hamiltonian_control_stationarity,
    evaluate_pmp_hamiltonian_control_stationarity_from_report, evaluate_pmp_hamiltonian_profile,
    solve_pmp_scalar_min_energy_shooting,
};
pub use pseudospectral::{
    LegendreGaussRadauGrid, LgrPseudospectralBoundaryConstraint,
    LgrPseudospectralColoredSparseResidualJacobianReport, LgrPseudospectralDecisionBoundsReport,
    LgrPseudospectralDecisionLayout, LgrPseudospectralDecisionReport,
    LgrPseudospectralFixedMeshOcp, LgrPseudospectralFixedMeshResidualReport,
    LgrPseudospectralFixedMeshSparseGaussNewtonStepReport,
    LgrPseudospectralFixedMeshSparseKktSqpReport, LgrPseudospectralFixedMeshSparseKktStepReport,
    LgrPseudospectralHardPathConstraint, LgrPseudospectralKktCollocationCostateEstimate,
    LgrPseudospectralKktCollocationMultiplierSample, LgrPseudospectralKktCostateEstimateReport,
    LgrPseudospectralKktHamiltonianReport, LgrPseudospectralKktHamiltonianSample,
    LgrPseudospectralKktResidualMultiplierReport, LgrPseudospectralMayerCost,
    LgrPseudospectralMeshRefinementAction, LgrPseudospectralMeshRefinementApplication,
    LgrPseudospectralMeshRefinementIterationReport, LgrPseudospectralMeshRefinementLoopReport,
    LgrPseudospectralMeshRefinementReport, LgrPseudospectralMeshRefinementSettings,
    LgrPseudospectralNonlinearHardPathConstraint, LgrPseudospectralNonlinearPathInequalityReport,
    LgrPseudospectralObjectiveReport, LgrPseudospectralPathInequalityReport,
    LgrPseudospectralQuadraticControlCost, LgrPseudospectralReferenceProfileMapping,
    LgrPseudospectralReferenceProfileReport, LgrPseudospectralRefinedSegmentMapping,
    LgrPseudospectralScalarVariable, LgrPseudospectralSegment,
    LgrPseudospectralSegmentErrorEstimate, LgrPseudospectralSegmentRefinementRecommendation,
    LgrPseudospectralSegmentReport, LgrPseudospectralSoftPathConstraint,
    LgrPseudospectralSparseAugmentedKktReport, LgrPseudospectralSparseGaussNewtonHessianReport,
    LgrPseudospectralSparseKktSolveReport, LgrPseudospectralSparseNormalSolveReport,
    LgrPseudospectralSparsePathInequalityJacobianReport,
    LgrPseudospectralSparseResidualJacobianReport, LgrPseudospectralTranscription,
    LgrPseudospectralTranscriptionReport, apply_lgr_pseudospectral_mesh_refinement,
    estimate_lgr_pseudospectral_mesh_refinement, evaluate_lgr_pseudospectral_segment,
    evaluate_lgr_pseudospectral_transcription, legendre_gauss_radau_grid,
    refine_lgr_pseudospectral_mesh_with_reference,
};
#[cfg(feature = "sqp")]
pub use pseudospectral::{
    LgrPseudospectralFixedMeshDenseConstrainedSqpReport,
    LgrPseudospectralFixedMeshDenseQuadraticQpReport,
    LgrPseudospectralFixedMeshDenseSqpPmpKktCertificateReport,
    LgrPseudospectralFixedMeshDenseSqpReport,
};
#[cfg(all(feature = "sqp", feature = "aero"))]
pub use scvx::{
    ScvxAeroMethodReferenceProfile, build_scvx_aero_method_powered_landing_foh_problem,
    build_scvx_aero_method_rigid_body_foh_problem,
    evaluate_scvx_aero_method_coupled_rigid_body_dynamics, evaluate_scvx_aero_method_force_moment,
    evaluate_scvx_aero_method_powered_landing_dynamics,
};
#[cfg(feature = "sqp")]
pub use scvx::{
    ScvxAffineDynamicsSegment, ScvxContinuousFohIterationReport, ScvxContinuousFohSettings,
    ScvxContinuousFohSolution, ScvxContinuousFohTimeOfFlightCandidateReport,
    ScvxContinuousFohTimeOfFlightSearchReport, ScvxContinuousFohTimeOfFlightSearchSettings,
    ScvxContinuousFohTimeOfFlightSweepReport, ScvxContinuousFohTimeOfFlightSweepSettings,
    ScvxContinuousFohTimeOfFlightSweepWindowReport, ScvxDiscreteAffineProblem,
    ScvxForwardEulerFohProblemDefinition, ScvxForwardEulerLinearizationSettings,
    ScvxIterationReport, ScvxLinearControlConstraint, ScvxLinearControlFunction,
    ScvxLinearStateConstraint, ScvxLinearStateFunction, ScvxPoweredLandingConicEnvelopeConstraints,
    ScvxPoweredLandingConicEnvelopeSettings, ScvxPoweredLandingControlLayout,
    ScvxPoweredLandingDynamicsParameters, ScvxPoweredLandingEnvelopeLimits,
    ScvxPoweredLandingEnvelopeReport, ScvxPoweredLandingEnvelopeSample,
    ScvxPoweredLandingLinearEnvelopeConstraints, ScvxPoweredLandingLinearizedEnvelopeSettings,
    ScvxPoweredLandingStateLayout, ScvxReducedAeroCoefficients, ScvxReducedAeroForceMoment,
    ScvxReducedAeroParameters, ScvxReducedAeroReferenceProfile, ScvxRigidBodyControlLayout,
    ScvxRigidBodyDynamicsParameters, ScvxRigidBodyDynamicsSettings, ScvxRigidBodyStateLayout,
    ScvxSecondOrderConeControlConstraint, ScvxSecondOrderConeStateConstraint, ScvxSettings,
    ScvxSolution, ScvxStateTriggeredConstraint, ScvxTerminalStateTarget, ScvxTerminationStatus,
    ScvxTimeOfFlightCandidateReport, ScvxTimeOfFlightSearchReport, ScvxTimeOfFlightSearchSettings,
    ScvxTimeOfFlightSweepReport, ScvxTimeOfFlightSweepSettings, ScvxTimeOfFlightSweepWindowReport,
    build_scvx_forward_euler_foh_problem, build_scvx_powered_landing_conic_envelope_constraints,
    build_scvx_powered_landing_foh_problem, build_scvx_powered_landing_linear_envelope_constraints,
    build_scvx_powered_landing_linearized_envelope_constraints,
    build_scvx_reduced_aero_powered_landing_foh_problem,
    build_scvx_reduced_aero_rigid_body_foh_problem, build_scvx_rigid_body_foh_problem,
    evaluate_scvx_aero_coupled_rigid_body_dynamics,
    evaluate_scvx_continuous_foh_time_of_flight_candidate, evaluate_scvx_powered_landing_dynamics,
    evaluate_scvx_powered_landing_envelope, evaluate_scvx_reduced_aero_force_moment,
    evaluate_scvx_reduced_aero_powered_landing_dynamics, evaluate_scvx_rigid_body_dynamics,
    evaluate_scvx_time_of_flight_candidate, linearize_scvx_forward_euler_foh,
    normalize_scvx_powered_landing_quaternion_reference_states,
    solve_scvx_continuous_foh_time_of_flight_search,
    solve_scvx_continuous_foh_time_of_flight_sweep, solve_scvx_discrete_affine,
    solve_scvx_forward_euler_foh, solve_scvx_forward_euler_foh_with_reference_projection,
    solve_scvx_forward_euler_foh_with_reference_update, solve_scvx_time_of_flight_search,
    solve_scvx_time_of_flight_sweep,
};
pub use shooting::{
    MultipleShootingContinuityReport, MultipleShootingControl,
    MultipleShootingControlledContinuityReport, MultipleShootingControlledTargetCorrection,
    MultipleShootingCorrection, MultipleShootingCorrector,
    MultipleShootingFreeDurationTargetCorrection, MultipleShootingNode,
    MultipleShootingSoftConstraint, MultipleShootingSoftConstraintKind,
    MultipleShootingSoftConstraintReport, MultipleShootingTargetCorrection,
    evaluate_multiple_shooting_soft_constraints, evaluate_two_body_controlled_multiple_shooting,
    evaluate_two_body_multiple_shooting, seed_two_body_controlled_multiple_shooting_nodes,
    seed_two_body_multiple_shooting_nodes,
};
#[cfg(feature = "sqp")]
pub use sqp::{
    ClarabelQpSolution, DenseConstrainedSqpSolution, DenseQpProblem, DenseSqpObjectiveSolution,
    DenseSqpSolution, DenseSqpSolver, LinearizedConstrainedLeastSquaresStepProblem,
    LinearizedElasticEqualityObjectiveStepProblem, LinearizedEqualityObjectiveStepProblem,
    LinearizedLeastSquaresObjectiveStepProblem, LinearizedLeastSquaresStepProblem,
    SqpInequalityModel, SqpResidualModel, TrajoptClarabelSettings,
    constrained_least_squares_step_qp, deterministic_sqp_settings,
    elastic_equality_objective_step_qp, equality_objective_step_qp,
    least_squares_objective_step_qp, least_squares_step_qp, solve_dense_qp,
    solve_linearized_constrained_least_squares_step,
    solve_linearized_elastic_equality_objective_step, solve_linearized_equality_objective_step,
    solve_linearized_least_squares_objective_step, solve_linearized_least_squares_step,
};
pub use stm::{
    StateTransitionMatrix, TwoBodyCartesianState, TwoBodyVariationalPropagation,
    propagate_two_body_variational,
};
pub use target::{TerminalResidual, terminal_residual};
