//! Regression coverage for exported flight-controller reference telemetry.

#![allow(clippy::expect_used, clippy::panic)]

use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use openbmp_core::ChannelId;
use openbmp_telemetry::TelemetryValue;
use openbmp_trajopt::{
    DenseSqpSolver, HermiteSimpsonBoundaryConstraint, HermiteSimpsonFixedMeshOcp,
    HermiteSimpsonFixedMeshSparseKktSqpReport, HermiteSimpsonHardPathConstraint,
    HermiteSimpsonReferenceProfileMapping, HermiteSimpsonSamplePoint, HermiteSimpsonScalarVariable,
    ILoadHeader, ILoadObjectiveClass, ILoadOptimalityCertificate, ILoadOptimalityEvidenceKind,
    ILoadPayload, PmpCostateComparisonSettings, PmpDirectCostateConvention,
    PmpHamiltonianControlStationaritySettings, PmpScalarMinEnergyProblem,
    PmpScalarMinEnergyShootingSettings, PmpScalarMinEnergySolution, ReferenceProfileSample,
    SparseKktSqpSettings, SynthesisMetadata, TerminalCondition, TrajoptError, encode_iload_payload,
    solve_pmp_scalar_min_energy_shooting,
};

struct TempScenarioDir {
    path: PathBuf,
}

impl TempScenarioDir {
    fn new(label: &str) -> Self {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system clock is after Unix epoch")
            .as_nanos();
        let path =
            std::env::temp_dir().join(format!("openbmp-{label}-{}-{nonce}", std::process::id()));
        fs::create_dir_all(&path).expect("temporary scenario directory is created");
        Self { path }
    }

    fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for TempScenarioDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.path);
    }
}

fn channel_id(outcome: &openbmp_runner::RunOutcome, name: &str) -> ChannelId {
    outcome
        .table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .unwrap_or_else(|| panic!("channel `{name}` must exist"))
        .id
}

fn pmp_min_energy_reference_iload() -> (ILoadPayload, PmpScalarMinEnergySolution) {
    let terminal = TerminalCondition::ApogeeRadius { radius_m: 102.0 };
    let problem = PmpScalarMinEnergyProblem {
        initial_time_s: 0.0,
        final_time_s: 0.2,
        initial_state: 100.0,
        terminal_state: 102.0,
        control_weight: 1.0,
    };
    let solution = solve_pmp_scalar_min_energy_shooting(
        problem,
        PmpScalarMinEnergyShootingSettings {
            sample_count: 5,
            ..Default::default()
        },
    )
    .expect("PMP scalar minimum-energy witness solves");
    assert!(solution.converged);
    assert!(solution.terminal_residual.abs() < 1.0e-12);
    assert!(solution.control > 0.0);
    assert!(solution.hamiltonian_report.max_abs_hamiltonian_drift < 1.0e-12);

    let reference_profile = solution
        .hamiltonian_report
        .samples
        .iter()
        .map(|sample| ReferenceProfileSample {
            time_s: sample.time_s,
            radius_m: sample.state[0],
            speed_m_s: solution.control,
            flight_path_angle_rad: 0.1,
        })
        .collect::<Vec<_>>();
    let optimality_certificate = ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
        ILoadOptimalityEvidenceKind::PmpScalarMinEnergyShooting,
        &solution.hamiltonian_report,
        1.0e-12,
    )
    .expect("PMP scalar optimality certificate validates");

    (
        ILoadPayload {
            header: ILoadHeader::new_with_objective(
                &terminal,
                ILoadObjectiveClass::OptimizedReference,
                7,
                "openbmp-runner-pmp-test".to_owned(),
            ),
            metadata: SynthesisMetadata {
                scenario_digest: "sha256:pmp-min-energy-file-backed-runner".to_owned(),
                source_revision: "test".to_owned(),
                method: "pmp-scalar-min-energy-shooting".to_owned(),
            },
            event_bindings_postcard: Vec::new(),
            gain_tables: Vec::new(),
            reference_profile,
            optimality_certificate: Some(optimality_certificate),
        },
        solution,
    )
}

fn hs_dense_sqp_sparse_kkt_reference_iload() -> ILoadPayload {
    let terminal = TerminalCondition::ApogeeRadius { radius_m: 102.0 };
    let ocp = HermiteSimpsonFixedMeshOcp::new(
        vec![0.0, 0.2],
        3,
        1,
        vec![
            HermiteSimpsonBoundaryConstraint::new(0, 0, 100.0).expect("initial radius boundary"),
            HermiteSimpsonBoundaryConstraint::new(1, 0, 102.0).expect("terminal radius boundary"),
            HermiteSimpsonBoundaryConstraint::new(0, 1, 10.0).expect("initial speed boundary"),
            HermiteSimpsonBoundaryConstraint::new(1, 1, 10.0).expect("terminal speed boundary"),
            HermiteSimpsonBoundaryConstraint::new(0, 2, 0.1).expect("initial gamma boundary"),
            HermiteSimpsonBoundaryConstraint::new(1, 2, 0.1).expect("terminal gamma boundary"),
        ],
    )
    .expect("Hermite-Simpson OCP builds");
    let exact = ocp
        .layout
        .pack_decision_vector(
            &[100.0, 10.0, 0.1, 102.0, 10.0, 0.1],
            &[101.0, 10.0, 0.1],
            &[10.0, 10.0],
            &[10.0],
        )
        .expect("HS dense-SQP exact decision packs");
    let mut initial = exact.clone();
    initial[3] = 101.9;
    initial[7] = 10.05;
    let mut lower = vec![-200.0; ocp.layout.variable_count];
    let mut upper = vec![200.0; ocp.layout.variable_count];
    for index in ocp.layout.node_control_offset..ocp.layout.variable_count {
        lower[index] = 10.0;
        upper[index] = 10.0;
    }
    let solver = DenseSqpSolver {
        residual_tolerance: 1.0e-10,
        step_tolerance: 1.0e-12,
        regularization: 1.0e-12,
        ..DenseSqpSolver::default()
    };
    let sqp_report = ocp
        .solve_dense_sqp(
            &initial,
            &lower,
            &upper,
            &solver,
            hs_dense_sqp_reference_dynamics,
        )
        .expect("HS dense SQP solves certified reference");
    assert!(sqp_report.sqp_solution.converged);
    assert!(sqp_report.residual_report.residual_norm < 1.0e-10);

    let mut certificate_dynamics = hs_dense_sqp_reference_dynamics;
    let mut certificate_lagrange = zero_lagrange;
    let certificate_report = ocp
        .dense_sqp_pmp_kkt_certificate_report(
            &sqp_report,
            PmpDirectCostateConvention::default(),
            1.0e-9,
            &mut certificate_dynamics,
            &mut certificate_lagrange,
            1.0e-6,
            1.0e-12,
            2.5e-1,
            1.0e-12,
        )
        .expect("HS dense-SQP final sparse-KKT certificate validates");
    assert_eq!(
        certificate_report.optimality_certificate.evidence_kind,
        ILoadOptimalityEvidenceKind::HermiteSimpsonPmpKkt
    );

    ocp.optimized_reference_iload_payload_with_optimality_certificate(
        &sqp_report.sqp_solution.variables,
        HermiteSimpsonReferenceProfileMapping::new(0, 1, 2),
        &terminal,
        8,
        "openbmp-runner-hs-dense-sqp-test".to_owned(),
        SynthesisMetadata {
            scenario_digest: "sha256:hs-dense-sqp-file-backed-runner".to_owned(),
            source_revision: "test".to_owned(),
            method: "hermite_simpson_dense_sqp_sparse_kkt_pmp_certificate".to_owned(),
        },
        Vec::new(),
        Vec::new(),
        certificate_report.optimality_certificate,
    )
    .expect("HS dense-SQP certified optimized-reference I-load emits")
}

fn hs_sparse_sqp_sparse_kkt_reference_iload() -> ILoadPayload {
    let terminal = TerminalCondition::ApogeeRadius { radius_m: 102.0 };
    let ocp = HermiteSimpsonFixedMeshOcp::new(
        vec![0.0, 0.2],
        3,
        1,
        vec![
            HermiteSimpsonBoundaryConstraint::new(0, 0, 100.0).expect("initial radius boundary"),
            HermiteSimpsonBoundaryConstraint::new(1, 0, 102.0).expect("terminal radius boundary"),
            HermiteSimpsonBoundaryConstraint::new(0, 1, 10.0).expect("initial speed boundary"),
            HermiteSimpsonBoundaryConstraint::new(1, 1, 10.0).expect("terminal speed boundary"),
            HermiteSimpsonBoundaryConstraint::new(0, 2, 0.1).expect("initial gamma boundary"),
            HermiteSimpsonBoundaryConstraint::new(1, 2, 0.1).expect("terminal gamma boundary"),
        ],
    )
    .expect("Hermite-Simpson OCP builds")
    .with_hard_path_constraints(vec![
        HermiteSimpsonHardPathConstraint::new(
            HermiteSimpsonSamplePoint::Midpoint(0),
            HermiteSimpsonScalarVariable::StateComponent { component: 0 },
            None,
            Some(101.0),
        )
        .expect("active midpoint hard path builds"),
    ])
    .expect("hard-path constrained OCP builds");
    let exact = ocp
        .layout
        .pack_decision_vector(
            &[100.0, 10.0, 0.1, 102.0, 10.0, 0.1],
            &[101.0, 10.0, 0.1],
            &[10.0, 10.0],
            &[10.0],
        )
        .expect("HS sparse-SQP exact decision packs");
    let mut lower = vec![-200.0; ocp.layout.variable_count];
    let mut upper = vec![200.0; ocp.layout.variable_count];
    for index in ocp.layout.node_control_offset..ocp.layout.variable_count {
        lower[index] = 10.0;
        upper[index] = 10.0;
    }
    let settings = SparseKktSqpSettings {
        max_iterations: 12,
        residual_tolerance: 1.0e-10,
        step_tolerance: 1.0e-12,
        kkt_regularization: 2.5e-1,
        ..SparseKktSqpSettings::default()
    };
    let decision_bounds = ocp
        .decision_bounds_with_hard_path_constraints(&lower, &upper)
        .expect("hard-path decision bounds build");
    let max_decision_bound_violation = decision_bounds
        .max_violation(&exact)
        .expect("decision-bound violation evaluates");
    let hard_path_inequalities = ocp
        .hard_path_inequality_report()
        .expect("hard-path inequalities build");
    let max_hard_path_violation = hard_path_inequalities
        .max_violation(&exact)
        .expect("hard-path violation evaluates");
    let mut residual_dynamics = hs_dense_sqp_reference_dynamics;
    let residual_report = ocp
        .residual_report(&exact, &mut residual_dynamics)
        .expect("sparse-SQP residual report evaluates");
    let mut kkt_dynamics = hs_dense_sqp_reference_dynamics;
    let final_kkt_step_report = ocp
        .sparse_augmented_kkt_step_report(
            &exact,
            &mut kkt_dynamics,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
        )
        .expect("final sparse KKT step report evaluates");
    let mut constrained_dynamics = hs_dense_sqp_reference_dynamics;
    let final_constrained_kkt_step_report = ocp
        .sparse_constrained_kkt_step_report(
            &exact,
            &mut constrained_dynamics,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
            settings.residual_tolerance,
        )
        .expect("final constrained sparse KKT step report evaluates");
    let sqp_report = HermiteSimpsonFixedMeshSparseKktSqpReport {
        decision: exact,
        residual_report,
        final_kkt_step_report,
        final_constrained_kkt_step_report: Some(final_constrained_kkt_step_report),
        decision_bounds,
        max_decision_bound_violation,
        max_hard_path_violation,
        nonlinear_hard_path_report: None,
        max_nonlinear_hard_path_violation: 0.0,
        iterations: Vec::new(),
        converged: true,
    };
    assert!(sqp_report.converged);
    assert!(sqp_report.residual_report.residual_norm < settings.residual_tolerance);

    let mut certificate_dynamics = hs_dense_sqp_reference_dynamics;
    let mut certificate_lagrange = zero_lagrange;
    let certificate_report = ocp
        .sparse_kkt_sqp_pmp_kkt_certificate_report(
            &sqp_report,
            PmpDirectCostateConvention::default(),
            1.0e-9,
            &mut certificate_dynamics,
            &mut certificate_lagrange,
        )
        .expect("HS sparse-SQP final sparse-KKT certificate validates");
    assert!(certificate_report.used_constrained_kkt_step_report);
    let pmp_times_s = certificate_report.costate_convention_report.times_s.clone();
    let pmp_costates = certificate_report
        .costate_convention_report
        .pmp_costates
        .clone();
    let certificate_report = certificate_report
        .with_pmp_costate_comparison(
            &pmp_times_s,
            &pmp_costates,
            PmpCostateComparisonSettings {
                costate_tolerance: 1.0e-12,
                ..Default::default()
            },
        )
        .expect("HS sparse-SQP PMP/KKT costate comparison attaches");
    let mut stationarity_dynamics = hs_dense_sqp_reference_dynamics;
    let mut stationarity_lagrange = zero_lagrange;
    let certificate_report = certificate_report
        .with_pmp_control_stationarity(
            PmpHamiltonianControlStationaritySettings {
                gradient_tolerance: 1.0e-9,
                ..Default::default()
            },
            &mut stationarity_dynamics,
            &mut stationarity_lagrange,
        )
        .expect("HS sparse-SQP PMP control stationarity attaches");
    assert!(
        certificate_report
            .control_stationarity_report
            .as_ref()
            .expect("control stationarity report")
            .within_tolerance
    );

    ocp.optimized_reference_iload_payload_with_optimality_certificate(
        &sqp_report.decision,
        HermiteSimpsonReferenceProfileMapping::new(0, 1, 2),
        &terminal,
        9,
        "openbmp-runner-hs-sparse-sqp-test".to_owned(),
        SynthesisMetadata {
            scenario_digest: "sha256:hs-sparse-sqp-file-backed-runner".to_owned(),
            source_revision: "test".to_owned(),
            method: "hermite_simpson_sparse_sqp_pmp_kkt_certificate".to_owned(),
        },
        Vec::new(),
        Vec::new(),
        certificate_report.optimality_certificate,
    )
    .expect("HS sparse-SQP certified optimized-reference I-load emits")
}

fn hs_dense_sqp_reference_dynamics(
    _time_s: f64,
    _state: &[f64],
    control: &[f64],
) -> Result<Vec<f64>, TrajoptError> {
    Ok(vec![control[0], 0.0, 0.0])
}

fn zero_lagrange(_time_s: f64, _state: &[f64], _control: &[f64]) -> Result<f64, TrajoptError> {
    Ok(0.0)
}

#[test]
fn fc_scenarios_export_guidance_reference_quaternion() {
    let scenario = openbmp_scenario::Scenario::from_toml_str(include_str!(
        "fixtures/fc-reference-telemetry-attitude-hold.toml"
    ))
    .expect("scenario parses");

    let outcome = openbmp_runner::run(&scenario).expect("scenario runs");

    let valid = channel_id(&outcome, "guidance.reference.valid");
    let q_x = channel_id(&outcome, "guidance.reference.q_x");
    let q_y = channel_id(&outcome, "guidance.reference.q_y");
    let q_z = channel_id(&outcome, "guidance.reference.q_z");
    let q_w = channel_id(&outcome, "guidance.reference.q_w");

    let valid_quaternion_rows = outcome
        .table
        .rows()
        .iter()
        .filter(|row| matches!(row.get(valid), Some(TelemetryValue::Bool(true))))
        .map(|row| {
            [q_x, q_y, q_z, q_w].map(|channel| match row.get(channel) {
                Some(TelemetryValue::Float64(value)) => *value,
                other => panic!("unexpected reference quaternion value: {other:?}"),
            })
        })
        .collect::<Vec<_>>();

    assert!(
        !valid_quaternion_rows.is_empty(),
        "FC guidance should publish at least one valid reference row"
    );
    for q in valid_quaternion_rows {
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!(
            (norm - 1.0).abs() < 1.0e-12,
            "reference quaternion must stay unit length, got {q:?} with norm {norm}"
        );
    }

    assert!(
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .all(|channel| channel.name != "guidance.cutoff.time_to_go_s"),
        "attitude-hold guidance should not export ascent cutoff telemetry"
    );

    let ascent_reference = openbmp_scenario::Scenario::from_toml_str(include_str!(
        "fixtures/fc-reference-telemetry-ascent-reference.toml"
    ))
    .expect("ascent-reference scenario parses");
    let ascent_outcome =
        openbmp_runner::run(&ascent_reference).expect("ascent-reference scenario runs");
    let _cutoff_valid = channel_id(&ascent_outcome, "guidance.cutoff.valid");
    let _cutoff_time = channel_id(&ascent_outcome, "guidance.cutoff.time_to_go_s");
}

#[test]
fn optimized_reference_scenario_exports_reference_and_actuator_stream() {
    let scenario = openbmp_scenario::Scenario::from_toml_str(include_str!(
        "fixtures/fc-reference-telemetry-optimized-reference.toml"
    ))
    .expect("optimized-reference scenario parses");

    let outcome = openbmp_runner::run(&scenario).expect("optimized-reference scenario runs");

    let valid = channel_id(&outcome, "guidance.reference.valid");
    let q_x = channel_id(&outcome, "guidance.reference.q_x");
    let q_y = channel_id(&outcome, "guidance.reference.q_y");
    let q_z = channel_id(&outcome, "guidance.reference.q_z");
    let q_w = channel_id(&outcome, "guidance.reference.q_w");

    let valid_quaternion_rows = outcome
        .table
        .rows()
        .iter()
        .filter(|row| matches!(row.get(valid), Some(TelemetryValue::Bool(true))))
        .map(|row| {
            [q_x, q_y, q_z, q_w].map(|channel| match row.get(channel) {
                Some(TelemetryValue::Float64(value)) => *value,
                other => panic!("unexpected optimized reference quaternion value: {other:?}"),
            })
        })
        .collect::<Vec<_>>();

    assert!(
        !valid_quaternion_rows.is_empty(),
        "optimized-reference guidance should publish valid reference rows"
    );
    for q in &valid_quaternion_rows {
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!(
            (norm - 1.0).abs() < 1.0e-12,
            "optimized-reference quaternion must stay unit length, got {q:?} with norm {norm}"
        );
    }
    assert!(
        valid_quaternion_rows
            .iter()
            .any(|q| q[1].abs() > 1.0e-2 && q[3] > 0.99),
        "optimized-reference scenario should export the declared pitched reference attitude"
    );
    assert!(
        outcome
            .actuator_stream
            .as_ref()
            .is_some_and(|report| report.packet_count > 0),
        "optimized-reference closed-loop run should emit FC actuator packets"
    );
    assert!(
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .all(|channel| channel.name != "guidance.cutoff.time_to_go_s"),
        "optimized-reference guidance should not export ascent cutoff telemetry"
    );
}

#[test]
fn optimized_reference_iload_file_scenario_exports_pmp_reference_and_actuator_stream() {
    let (payload, solution) = pmp_min_energy_reference_iload();
    assert_eq!(
        payload.reference_profile.len(),
        solution.hamiltonian_report.sample_count
    );
    assert_eq!(
        payload.header.objective_class,
        ILoadObjectiveClass::OptimizedReference
    );
    assert_eq!(payload.metadata.method, "pmp-scalar-min-energy-shooting");
    let certificate = payload
        .optimality_certificate
        .as_ref()
        .expect("PMP reference I-load carries optimality certificate");
    assert_eq!(
        certificate.evidence_kind,
        ILoadOptimalityEvidenceKind::PmpScalarMinEnergyShooting
    );
    assert_eq!(
        certificate.sample_count,
        solution.hamiltonian_report.sample_count
    );
    assert!(
        certificate.max_abs_hamiltonian_drift <= certificate.hamiltonian_drift_tolerance,
        "PMP reference I-load certificate must satisfy Hamiltonian drift tolerance"
    );

    let encoded = encode_iload_payload(&payload).expect("PMP reference I-load encodes");
    let temp_dir = TempScenarioDir::new("optimized-reference-iload");
    fs::write(
        temp_dir
            .path()
            .join("pmp-min-energy-optimized-reference.iload"),
        encoded,
    )
    .expect("PMP reference I-load fixture is written");
    let scenario = openbmp_scenario::Scenario::from_toml_str_with_source_dir(
        include_str!("fixtures/fc-reference-telemetry-optimized-reference-iload.toml"),
        Some(temp_dir.path().to_path_buf()),
    )
    .expect("file-backed optimized-reference scenario parses");

    let outcome = openbmp_runner::run(&scenario)
        .expect("file-backed optimized-reference scenario runs through runner");

    let valid = channel_id(&outcome, "guidance.reference.valid");
    let q_x = channel_id(&outcome, "guidance.reference.q_x");
    let q_y = channel_id(&outcome, "guidance.reference.q_y");
    let q_z = channel_id(&outcome, "guidance.reference.q_z");
    let q_w = channel_id(&outcome, "guidance.reference.q_w");

    let valid_quaternion_rows = outcome
        .table
        .rows()
        .iter()
        .filter(|row| matches!(row.get(valid), Some(TelemetryValue::Bool(true))))
        .map(|row| {
            [q_x, q_y, q_z, q_w].map(|channel| match row.get(channel) {
                Some(TelemetryValue::Float64(value)) => *value,
                other => panic!("unexpected file-backed reference quaternion value: {other:?}"),
            })
        })
        .collect::<Vec<_>>();

    assert!(
        !valid_quaternion_rows.is_empty(),
        "file-backed optimized-reference guidance should publish valid reference rows"
    );
    assert!(
        valid_quaternion_rows.iter().any(|q| {
            let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
            (norm - 1.0).abs() < 1.0e-12 && q[1].abs() > 1.0e-2 && q[3] > 0.99
        }),
        "file-backed optimized-reference scenario should export the declared pitched reference attitude"
    );
    assert!(
        outcome
            .actuator_stream
            .as_ref()
            .is_some_and(|report| report.packet_count > 0),
        "file-backed optimized-reference closed-loop run should emit FC actuator packets"
    );
    assert!(
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .all(|channel| channel.name != "guidance.cutoff.time_to_go_s"),
        "file-backed optimized-reference guidance should not export ascent cutoff telemetry"
    );
}

#[test]
fn optimized_reference_iload_file_scenario_exports_dense_sqp_pmp_kkt_reference_and_actuator_stream()
{
    let payload = hs_dense_sqp_sparse_kkt_reference_iload();
    assert_eq!(
        payload.header.objective_class,
        ILoadObjectiveClass::OptimizedReference
    );
    assert_eq!(
        payload.metadata.method,
        "hermite_simpson_dense_sqp_sparse_kkt_pmp_certificate"
    );
    let certificate = payload
        .optimality_certificate
        .as_ref()
        .expect("dense-SQP reference I-load carries optimality certificate");
    assert_eq!(
        certificate.evidence_kind,
        ILoadOptimalityEvidenceKind::HermiteSimpsonPmpKkt
    );
    assert!(
        certificate.max_abs_hamiltonian_drift <= certificate.hamiltonian_drift_tolerance,
        "dense-SQP reference I-load certificate must satisfy Hamiltonian drift tolerance"
    );

    let encoded = encode_iload_payload(&payload).expect("dense-SQP reference I-load encodes");
    let temp_dir = TempScenarioDir::new("optimized-reference-dense-sqp-iload");
    fs::write(
        temp_dir
            .path()
            .join("pmp-min-energy-optimized-reference.iload"),
        encoded,
    )
    .expect("dense-SQP reference I-load fixture is written");
    let scenario = openbmp_scenario::Scenario::from_toml_str_with_source_dir(
        include_str!("fixtures/fc-reference-telemetry-optimized-reference-iload.toml"),
        Some(temp_dir.path().to_path_buf()),
    )
    .expect("dense-SQP file-backed optimized-reference scenario parses");

    let outcome = openbmp_runner::run(&scenario)
        .expect("dense-SQP file-backed optimized-reference scenario runs through runner");

    let valid = channel_id(&outcome, "guidance.reference.valid");
    let q_x = channel_id(&outcome, "guidance.reference.q_x");
    let q_y = channel_id(&outcome, "guidance.reference.q_y");
    let q_z = channel_id(&outcome, "guidance.reference.q_z");
    let q_w = channel_id(&outcome, "guidance.reference.q_w");

    let valid_quaternion_rows = outcome
        .table
        .rows()
        .iter()
        .filter(|row| matches!(row.get(valid), Some(TelemetryValue::Bool(true))))
        .map(|row| {
            [q_x, q_y, q_z, q_w].map(|channel| match row.get(channel) {
                Some(TelemetryValue::Float64(value)) => *value,
                other => {
                    panic!("unexpected dense-SQP file-backed reference quaternion value: {other:?}")
                }
            })
        })
        .collect::<Vec<_>>();

    assert!(
        !valid_quaternion_rows.is_empty(),
        "dense-SQP file-backed optimized-reference guidance should publish valid reference rows"
    );
    assert!(
        valid_quaternion_rows.iter().any(|q| {
            let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
            (norm - 1.0).abs() < 1.0e-12 && q[1].abs() > 1.0e-2 && q[3] > 0.99
        }),
        "dense-SQP file-backed optimized-reference scenario should export the declared pitched reference attitude"
    );
    assert!(
        outcome
            .actuator_stream
            .as_ref()
            .is_some_and(|report| report.packet_count > 0),
        "dense-SQP file-backed optimized-reference closed-loop run should emit FC actuator packets"
    );
    assert!(
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .all(|channel| channel.name != "guidance.cutoff.time_to_go_s"),
        "dense-SQP file-backed optimized-reference guidance should not export ascent cutoff telemetry"
    );
}

#[test]
fn optimized_reference_iload_file_scenario_exports_sparse_sqp_pmp_kkt_reference_and_actuator_stream()
 {
    let payload = hs_sparse_sqp_sparse_kkt_reference_iload();
    assert_eq!(
        payload.header.objective_class,
        ILoadObjectiveClass::OptimizedReference
    );
    assert_eq!(
        payload.metadata.method,
        "hermite_simpson_sparse_sqp_pmp_kkt_certificate"
    );
    let certificate = payload
        .optimality_certificate
        .as_ref()
        .expect("sparse-SQP reference I-load carries optimality certificate");
    assert_eq!(
        certificate.evidence_kind,
        ILoadOptimalityEvidenceKind::HermiteSimpsonPmpKkt
    );
    assert!(
        certificate.max_abs_hamiltonian_drift <= certificate.hamiltonian_drift_tolerance,
        "sparse-SQP reference I-load certificate must satisfy Hamiltonian drift tolerance"
    );
    assert_eq!(certificate.max_costate_norm_difference, Some(0.0));
    let max_control_stationarity = certificate
        .max_control_stationarity_gradient_norm
        .expect("sparse-SQP certificate carries control stationarity");
    let control_stationarity_tolerance = certificate
        .control_stationarity_tolerance
        .expect("sparse-SQP certificate carries control-stationarity tolerance");
    assert!(
        max_control_stationarity <= control_stationarity_tolerance,
        "sparse-SQP reference I-load certificate must satisfy control-stationarity tolerance"
    );

    let encoded = encode_iload_payload(&payload).expect("sparse-SQP reference I-load encodes");
    let temp_dir = TempScenarioDir::new("optimized-reference-sparse-sqp-iload");
    fs::write(
        temp_dir
            .path()
            .join("pmp-min-energy-optimized-reference.iload"),
        encoded,
    )
    .expect("sparse-SQP reference I-load fixture is written");
    let scenario = openbmp_scenario::Scenario::from_toml_str_with_source_dir(
        include_str!("fixtures/fc-reference-telemetry-optimized-reference-iload.toml"),
        Some(temp_dir.path().to_path_buf()),
    )
    .expect("sparse-SQP file-backed optimized-reference scenario parses");

    let outcome = openbmp_runner::run(&scenario)
        .expect("sparse-SQP file-backed optimized-reference scenario runs through runner");

    let valid = channel_id(&outcome, "guidance.reference.valid");
    let q_x = channel_id(&outcome, "guidance.reference.q_x");
    let q_y = channel_id(&outcome, "guidance.reference.q_y");
    let q_z = channel_id(&outcome, "guidance.reference.q_z");
    let q_w = channel_id(&outcome, "guidance.reference.q_w");

    let valid_quaternion_rows = outcome
        .table
        .rows()
        .iter()
        .filter(|row| matches!(row.get(valid), Some(TelemetryValue::Bool(true))))
        .map(|row| {
            [q_x, q_y, q_z, q_w].map(|channel| match row.get(channel) {
                Some(TelemetryValue::Float64(value)) => *value,
                other => {
                    panic!(
                        "unexpected sparse-SQP file-backed reference quaternion value: {other:?}"
                    )
                }
            })
        })
        .collect::<Vec<_>>();

    assert!(
        !valid_quaternion_rows.is_empty(),
        "sparse-SQP file-backed optimized-reference guidance should publish valid reference rows"
    );
    assert!(
        valid_quaternion_rows.iter().any(|q| {
            let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
            (norm - 1.0).abs() < 1.0e-12 && q[1].abs() > 1.0e-2 && q[3] > 0.99
        }),
        "sparse-SQP file-backed optimized-reference scenario should export the declared pitched reference attitude"
    );
    assert!(
        outcome
            .actuator_stream
            .as_ref()
            .is_some_and(|report| report.packet_count > 0),
        "sparse-SQP file-backed optimized-reference closed-loop run should emit FC actuator packets"
    );
    assert!(
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .all(|channel| channel.name != "guidance.cutoff.time_to_go_s"),
        "sparse-SQP file-backed optimized-reference guidance should not export ascent cutoff telemetry"
    );
}
