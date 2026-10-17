//! Pontryagin Minimum Principle witness helpers.
//!
//! This module is a host-side optimality diagnostic substrate. It evaluates
//! Hamiltonian constancy and same-mesh costate agreement for local state-vector
//! trajectories; it deliberately does not introduce any target vocabulary.

use alloc::vec;
use alloc::vec::Vec;

use crate::iload::TrajoptError;

/// One Hamiltonian sample along a PMP witness trajectory.
#[derive(Clone, Debug, PartialEq)]
pub struct PmpHamiltonianSample {
    /// Physical sample time.
    pub time_s: f64,
    /// State vector at the sample.
    pub state: Vec<f64>,
    /// Control vector at the sample.
    pub control: Vec<f64>,
    /// Costate vector at the sample.
    pub costate: Vec<f64>,
    /// Dynamics value `f(x, u, t)` at the sample.
    pub dynamics: Vec<f64>,
    /// Running cost `L(x, u, t)` at the sample.
    pub lagrange_value: f64,
    /// Inner product `lambda^T f`.
    pub costate_dot_dynamics: f64,
    /// Hamiltonian `H = L + lambda^T f`.
    pub hamiltonian: f64,
    /// Difference from the first sample's Hamiltonian.
    pub hamiltonian_drift: f64,
}

/// Hamiltonian constancy report for an autonomous-arc optimality witness.
#[derive(Clone, Debug, PartialEq)]
pub struct PmpHamiltonianReport {
    /// State dimension.
    pub state_dimension: usize,
    /// Control dimension.
    pub control_dimension: usize,
    /// Number of samples.
    pub sample_count: usize,
    /// Hamiltonian at the first sample.
    pub initial_hamiltonian: f64,
    /// Maximum absolute Hamiltonian value.
    pub max_abs_hamiltonian: f64,
    /// Maximum absolute Hamiltonian drift from the first sample.
    pub max_abs_hamiltonian_drift: f64,
    /// Per-sample diagnostics.
    pub samples: Vec<PmpHamiltonianSample>,
}

/// Borrowed local state/control/costate profile for Hamiltonian evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PmpHamiltonianProfile<'a> {
    /// State dimension.
    pub state_dimension: usize,
    /// Control dimension.
    pub control_dimension: usize,
    /// Strictly increasing physical sample times.
    pub times_s: &'a [f64],
    /// Row-major state samples.
    pub states: &'a [f64],
    /// Row-major control samples.
    pub controls: &'a [f64],
    /// Row-major costate samples.
    pub costates: &'a [f64],
}

/// Settings for same-mesh PMP-vs-direct-method costate comparisons.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PmpCostateComparisonSettings {
    /// Maximum permitted absolute time mismatch for corresponding samples.
    pub time_tolerance_s: f64,
    /// Maximum permitted costate-vector norm difference.
    pub costate_tolerance: f64,
}

impl Default for PmpCostateComparisonSettings {
    fn default() -> Self {
        Self {
            time_tolerance_s: 1.0e-12,
            costate_tolerance: 1.0e-4,
        }
    }
}

/// One same-mesh costate comparison sample.
#[derive(Clone, Debug, PartialEq)]
pub struct PmpCostateComparisonSample {
    /// Physical sample time.
    pub time_s: f64,
    /// PMP-shooting costate vector.
    pub pmp_costate: Vec<f64>,
    /// Direct-method/KKT costate estimate vector.
    pub kkt_costate: Vec<f64>,
    /// Componentwise `pmp_costate - kkt_costate`.
    pub difference: Vec<f64>,
    /// Maximum absolute component difference.
    pub max_abs_difference: f64,
    /// Euclidean norm of the component difference.
    pub norm_difference: f64,
}

/// Same-mesh PMP-vs-KKT costate comparison report.
#[derive(Clone, Debug, PartialEq)]
pub struct PmpCostateComparisonReport {
    /// State/costate dimension.
    pub state_dimension: usize,
    /// Number of compared samples.
    pub sample_count: usize,
    /// Tolerance used for the verdict.
    pub costate_tolerance: f64,
    /// Maximum absolute component difference across all samples.
    pub max_abs_costate_difference: f64,
    /// Maximum Euclidean costate-vector difference across all samples.
    pub max_costate_norm_difference: f64,
    /// True when `max_costate_norm_difference <= costate_tolerance`.
    pub within_tolerance: bool,
    /// Per-sample comparisons.
    pub samples: Vec<PmpCostateComparisonSample>,
}

/// Settings for Hamiltonian control-stationarity checks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PmpHamiltonianControlStationaritySettings {
    /// Central finite-difference perturbation applied to each control component.
    pub finite_difference_step: f64,
    /// Maximum permitted Euclidean norm of `dH/du` at each sample.
    pub gradient_tolerance: f64,
}

impl Default for PmpHamiltonianControlStationaritySettings {
    fn default() -> Self {
        Self {
            finite_difference_step: 1.0e-6,
            gradient_tolerance: 1.0e-6,
        }
    }
}

/// One Hamiltonian control-stationarity sample.
#[derive(Clone, Debug, PartialEq)]
pub struct PmpHamiltonianControlStationaritySample {
    /// Physical sample time.
    pub time_s: f64,
    /// Control vector at the sample.
    pub control: Vec<f64>,
    /// Central finite-difference Hamiltonian gradient with respect to control.
    pub gradient: Vec<f64>,
    /// Maximum absolute gradient component.
    pub max_abs_gradient: f64,
    /// Euclidean norm of the gradient vector.
    pub gradient_norm: f64,
}

/// Hamiltonian control-stationarity report for a PMP witness profile.
#[derive(Clone, Debug, PartialEq)]
pub struct PmpHamiltonianControlStationarityReport {
    /// State dimension.
    pub state_dimension: usize,
    /// Control dimension.
    pub control_dimension: usize,
    /// Number of samples.
    pub sample_count: usize,
    /// Central finite-difference perturbation used for `dH/du`.
    pub finite_difference_step: f64,
    /// Tolerance used for the verdict.
    pub gradient_tolerance: f64,
    /// Maximum absolute gradient component across all samples.
    pub max_abs_gradient: f64,
    /// Maximum Euclidean gradient norm across all samples.
    pub max_gradient_norm: f64,
    /// True when `max_gradient_norm <= gradient_tolerance`.
    pub within_tolerance: bool,
    /// Per-sample control-stationarity diagnostics.
    pub samples: Vec<PmpHamiltonianControlStationaritySample>,
}

/// Explicit sign/scale convention for direct-method costate estimates.
///
/// Direct transcription residual multipliers are row-family diagnostics until a
/// solver-specific covector convention is selected. This scale maps each raw
/// direct-method costate component into the PMP convention used by
/// [`compare_pmp_costate_profiles`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PmpDirectCostateConvention {
    /// Multiplicative scale applied to raw direct-method costates.
    pub raw_to_pmp_scale: f64,
}

impl Default for PmpDirectCostateConvention {
    fn default() -> Self {
        Self {
            raw_to_pmp_scale: 1.0,
        }
    }
}

/// One direct-method costate sample after applying a PMP convention.
#[derive(Clone, Debug, PartialEq)]
pub struct PmpDirectCostateConventionSample {
    /// Sample index in the supplied profile.
    pub sample_index: usize,
    /// Physical sample time.
    pub time_s: f64,
    /// Raw direct-method costate vector.
    pub raw_costate: Vec<f64>,
    /// Costate vector after applying [`PmpDirectCostateConvention`].
    pub pmp_costate: Vec<f64>,
}

/// Direct-method costate profile mapped into PMP sign/scale convention.
#[derive(Clone, Debug, PartialEq)]
pub struct PmpDirectCostateConventionReport {
    /// State/costate dimension.
    pub state_dimension: usize,
    /// Number of samples.
    pub sample_count: usize,
    /// Applied raw-to-PMP scale.
    pub raw_to_pmp_scale: f64,
    /// Strictly increasing sample times.
    pub times_s: Vec<f64>,
    /// Row-major raw direct-method costates.
    pub raw_costates: Vec<f64>,
    /// Row-major PMP-convention costates.
    pub pmp_costates: Vec<f64>,
    /// Maximum absolute raw costate component.
    pub max_abs_raw_costate: f64,
    /// Maximum absolute PMP-convention costate component.
    pub max_abs_pmp_costate: f64,
    /// Per-sample conversion diagnostics.
    pub samples: Vec<PmpDirectCostateConventionSample>,
}

/// Scalar fixed-time minimum-energy integrator witness problem.
///
/// The OCP is `min integral 0.5 * control_weight * u^2 dt` subject to
/// `x_dot = u`, fixed `x(t0)`, fixed `x(tf)`, and fixed final time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PmpScalarMinEnergyProblem {
    /// Initial time.
    pub initial_time_s: f64,
    /// Final time.
    pub final_time_s: f64,
    /// Initial scalar state.
    pub initial_state: f64,
    /// Terminal scalar state.
    pub terminal_state: f64,
    /// Positive scalar running-cost control weight.
    pub control_weight: f64,
}

/// Settings for the deterministic scalar PMP shooting witness.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PmpScalarMinEnergyShootingSettings {
    /// Maximum Newton/Gauss-Newton shooting iterations.
    pub max_iterations: usize,
    /// Terminal residual convergence tolerance.
    pub terminal_tolerance: f64,
    /// Central finite-difference step for the shooting residual derivative.
    pub finite_difference_step: f64,
    /// Number of Hamiltonian samples emitted in the solution report.
    pub sample_count: usize,
    /// Initial costate guess.
    pub initial_costate_guess: f64,
}

impl Default for PmpScalarMinEnergyShootingSettings {
    fn default() -> Self {
        Self {
            max_iterations: 8,
            terminal_tolerance: 1.0e-12,
            finite_difference_step: 1.0e-6,
            sample_count: 5,
            initial_costate_guess: 0.0,
        }
    }
}

/// One scalar PMP shooting iteration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PmpScalarMinEnergyIteration {
    /// Zero-based iteration index.
    pub iteration: usize,
    /// Costate guess at the start of the iteration.
    pub costate_guess: f64,
    /// Propagated terminal state for this costate.
    pub terminal_state: f64,
    /// Terminal residual `x(tf) - target`.
    pub terminal_residual: f64,
    /// Finite-difference derivative of the terminal residual wrt costate.
    pub residual_derivative: f64,
    /// Newton step applied to the costate.
    pub costate_step: f64,
}

/// Scalar fixed-time minimum-energy PMP shooting solution.
#[derive(Clone, Debug, PartialEq)]
pub struct PmpScalarMinEnergySolution {
    /// Whether the terminal residual reached tolerance.
    pub converged: bool,
    /// Number of completed shooting iterations.
    pub iterations: usize,
    /// Solved initial costate.
    pub initial_costate: f64,
    /// Constant optimal control.
    pub control: f64,
    /// Propagated final state.
    pub final_state: f64,
    /// Final terminal residual `x(tf) - target`.
    pub terminal_residual: f64,
    /// Integral objective value.
    pub objective: f64,
    /// Hamiltonian constancy report for the solved arc.
    pub hamiltonian_report: PmpHamiltonianReport,
    /// Per-iteration shooting diagnostics.
    pub iteration_reports: Vec<PmpScalarMinEnergyIteration>,
}

/// Evaluate Hamiltonian samples for a local state/control/costate profile.
pub fn evaluate_pmp_hamiltonian_profile<D, L>(
    profile: PmpHamiltonianProfile<'_>,
    mut dynamics: D,
    mut lagrange: L,
) -> Result<PmpHamiltonianReport, TrajoptError>
where
    D: FnMut(f64, &[f64], &[f64]) -> Result<Vec<f64>, TrajoptError>,
    L: FnMut(f64, &[f64], &[f64]) -> Result<f64, TrajoptError>,
{
    let state_dimension = profile.state_dimension;
    let control_dimension = profile.control_dimension;
    let times_s = profile.times_s;
    let states = profile.states;
    let controls = profile.controls;
    let costates = profile.costates;
    validate_dimensions(state_dimension, control_dimension)?;
    validate_time_grid(times_s)?;
    let sample_count = times_s.len();
    validate_len(
        "PMP state trajectory dimensions are inconsistent",
        states,
        sample_count * state_dimension,
    )?;
    validate_len(
        "PMP control trajectory dimensions are inconsistent",
        controls,
        sample_count * control_dimension,
    )?;
    validate_len(
        "PMP costate trajectory dimensions are inconsistent",
        costates,
        sample_count * state_dimension,
    )?;
    validate_finite("PMP state trajectory must be finite", states)?;
    validate_finite("PMP control trajectory must be finite", controls)?;
    validate_finite("PMP costate trajectory must be finite", costates)?;

    let mut samples = Vec::with_capacity(sample_count);
    let mut initial_hamiltonian = 0.0;
    let mut max_abs_hamiltonian = 0.0_f64;
    let mut max_abs_hamiltonian_drift = 0.0_f64;

    for (sample_index, &time_s) in times_s.iter().enumerate() {
        let state = block(states, state_dimension, sample_index);
        let control = block(controls, control_dimension, sample_index);
        let costate = block(costates, state_dimension, sample_index);
        let dynamics_value = dynamics(time_s, state, control)?;
        validate_len(
            "PMP dynamics output dimensions are inconsistent",
            &dynamics_value,
            state_dimension,
        )?;
        validate_finite("PMP dynamics output must be finite", &dynamics_value)?;
        let lagrange_value = lagrange(time_s, state, control)?;
        if !lagrange_value.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "PMP Lagrange output must be finite",
            });
        }
        let costate_dot_dynamics = costate
            .iter()
            .zip(&dynamics_value)
            .map(|(lambda, value)| lambda * value)
            .sum::<f64>();
        let hamiltonian = lagrange_value + costate_dot_dynamics;
        if sample_index == 0 {
            initial_hamiltonian = hamiltonian;
        }
        let hamiltonian_drift = hamiltonian - initial_hamiltonian;
        max_abs_hamiltonian = max_abs_hamiltonian.max(hamiltonian.abs());
        max_abs_hamiltonian_drift = max_abs_hamiltonian_drift.max(hamiltonian_drift.abs());
        samples.push(PmpHamiltonianSample {
            time_s,
            state: state.to_vec(),
            control: control.to_vec(),
            costate: costate.to_vec(),
            dynamics: dynamics_value,
            lagrange_value,
            costate_dot_dynamics,
            hamiltonian,
            hamiltonian_drift,
        });
    }

    Ok(PmpHamiltonianReport {
        state_dimension,
        control_dimension,
        sample_count,
        initial_hamiltonian,
        max_abs_hamiltonian,
        max_abs_hamiltonian_drift,
        samples,
    })
}

/// Compare same-mesh PMP and direct-method/KKT costate profiles.
pub fn compare_pmp_costate_profiles(
    state_dimension: usize,
    pmp_times_s: &[f64],
    pmp_costates: &[f64],
    kkt_times_s: &[f64],
    kkt_costates: &[f64],
    settings: PmpCostateComparisonSettings,
) -> Result<PmpCostateComparisonReport, TrajoptError> {
    validate_costate_comparison_settings(settings)?;
    if state_dimension == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP costate dimension must be nonzero",
        });
    }
    validate_time_grid(pmp_times_s)?;
    validate_time_grid(kkt_times_s)?;
    if pmp_times_s.len() != kkt_times_s.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP and KKT costate sample counts must match",
        });
    }
    let sample_count = pmp_times_s.len();
    validate_len(
        "PMP costate comparison PMP dimensions are inconsistent",
        pmp_costates,
        sample_count * state_dimension,
    )?;
    validate_len(
        "PMP costate comparison KKT dimensions are inconsistent",
        kkt_costates,
        sample_count * state_dimension,
    )?;
    validate_finite("PMP costates must be finite", pmp_costates)?;
    validate_finite("PMP KKT costates must be finite", kkt_costates)?;

    let mut samples = Vec::with_capacity(sample_count);
    let mut max_abs_costate_difference = 0.0_f64;
    let mut max_costate_norm_difference = 0.0_f64;

    for sample_index in 0..sample_count {
        if (pmp_times_s[sample_index] - kkt_times_s[sample_index]).abs() > settings.time_tolerance_s
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "PMP and KKT costate sample times do not match",
            });
        }
        let pmp_costate = block(pmp_costates, state_dimension, sample_index);
        let kkt_costate = block(kkt_costates, state_dimension, sample_index);
        let mut difference = Vec::with_capacity(state_dimension);
        let mut sum_sq = 0.0;
        let mut max_abs_difference = 0.0_f64;
        for component in 0..state_dimension {
            let value = pmp_costate[component] - kkt_costate[component];
            max_abs_difference = max_abs_difference.max(value.abs());
            sum_sq += value * value;
            difference.push(value);
        }
        let norm_difference = sum_sq.sqrt();
        max_abs_costate_difference = max_abs_costate_difference.max(max_abs_difference);
        max_costate_norm_difference = max_costate_norm_difference.max(norm_difference);
        samples.push(PmpCostateComparisonSample {
            time_s: pmp_times_s[sample_index],
            pmp_costate: pmp_costate.to_vec(),
            kkt_costate: kkt_costate.to_vec(),
            difference,
            max_abs_difference,
            norm_difference,
        });
    }

    Ok(PmpCostateComparisonReport {
        state_dimension,
        sample_count,
        costate_tolerance: settings.costate_tolerance,
        max_abs_costate_difference,
        max_costate_norm_difference,
        within_tolerance: max_costate_norm_difference <= settings.costate_tolerance,
        samples,
    })
}

/// Evaluate central finite-difference `dH/du` stationarity for a PMP profile.
pub fn evaluate_pmp_hamiltonian_control_stationarity<D, L>(
    profile: PmpHamiltonianProfile<'_>,
    settings: PmpHamiltonianControlStationaritySettings,
    mut dynamics: D,
    mut lagrange: L,
) -> Result<PmpHamiltonianControlStationarityReport, TrajoptError>
where
    D: FnMut(f64, &[f64], &[f64]) -> Result<Vec<f64>, TrajoptError>,
    L: FnMut(f64, &[f64], &[f64]) -> Result<f64, TrajoptError>,
{
    validate_hamiltonian_control_stationarity_settings(settings)?;
    let state_dimension = profile.state_dimension;
    let control_dimension = profile.control_dimension;
    let times_s = profile.times_s;
    let states = profile.states;
    let controls = profile.controls;
    let costates = profile.costates;
    validate_dimensions(state_dimension, control_dimension)?;
    validate_time_grid(times_s)?;
    let sample_count = times_s.len();
    validate_len(
        "PMP stationarity state trajectory dimensions are inconsistent",
        states,
        sample_count * state_dimension,
    )?;
    validate_len(
        "PMP stationarity control trajectory dimensions are inconsistent",
        controls,
        sample_count * control_dimension,
    )?;
    validate_len(
        "PMP stationarity costate trajectory dimensions are inconsistent",
        costates,
        sample_count * state_dimension,
    )?;
    validate_finite("PMP stationarity states must be finite", states)?;
    validate_finite("PMP stationarity controls must be finite", controls)?;
    validate_finite("PMP stationarity costates must be finite", costates)?;

    let mut samples = Vec::with_capacity(sample_count);
    let mut max_abs_gradient = 0.0_f64;
    let mut max_gradient_norm = 0.0_f64;
    let step = settings.finite_difference_step;

    for (sample_index, &time_s) in times_s.iter().enumerate() {
        let state = block(states, state_dimension, sample_index);
        let control = block(controls, control_dimension, sample_index);
        let costate = block(costates, state_dimension, sample_index);
        let mut gradient = Vec::with_capacity(control_dimension);
        let mut sample_max_abs_gradient = 0.0_f64;
        let mut sum_sq = 0.0;
        for component in 0..control_dimension {
            let mut plus_control = control.to_vec();
            let mut minus_control = control.to_vec();
            plus_control[component] += step;
            minus_control[component] -= step;
            let h_plus = evaluate_hamiltonian_value(
                time_s,
                state,
                &plus_control,
                costate,
                state_dimension,
                &mut dynamics,
                &mut lagrange,
            )?;
            let h_minus = evaluate_hamiltonian_value(
                time_s,
                state,
                &minus_control,
                costate,
                state_dimension,
                &mut dynamics,
                &mut lagrange,
            )?;
            let value = (h_plus - h_minus) / (2.0 * step);
            if !value.is_finite() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "PMP stationarity gradient must be finite",
                });
            }
            sample_max_abs_gradient = sample_max_abs_gradient.max(value.abs());
            sum_sq += value * value;
            gradient.push(value);
        }
        let gradient_norm = sum_sq.sqrt();
        max_abs_gradient = max_abs_gradient.max(sample_max_abs_gradient);
        max_gradient_norm = max_gradient_norm.max(gradient_norm);
        samples.push(PmpHamiltonianControlStationaritySample {
            time_s,
            control: control.to_vec(),
            gradient,
            max_abs_gradient: sample_max_abs_gradient,
            gradient_norm,
        });
    }

    Ok(PmpHamiltonianControlStationarityReport {
        state_dimension,
        control_dimension,
        sample_count,
        finite_difference_step: settings.finite_difference_step,
        gradient_tolerance: settings.gradient_tolerance,
        max_abs_gradient,
        max_gradient_norm,
        within_tolerance: max_gradient_norm <= settings.gradient_tolerance,
        samples,
    })
}

/// Evaluate Hamiltonian control stationarity from an existing Hamiltonian report.
///
/// This reuses the state/control/costate samples already audited by
/// [`evaluate_pmp_hamiltonian_profile`] and central-differences `H` with
/// respect to each control component through
/// [`evaluate_pmp_hamiltonian_control_stationarity`].
///
/// # Errors
///
/// Returns [`TrajoptError`] when the report sample count is inconsistent or the
/// generated profile, stationarity settings, dynamics callback, or Lagrange
/// callback are malformed.
pub fn evaluate_pmp_hamiltonian_control_stationarity_from_report<D, L>(
    report: &PmpHamiltonianReport,
    settings: PmpHamiltonianControlStationaritySettings,
    dynamics: D,
    lagrange: L,
) -> Result<PmpHamiltonianControlStationarityReport, TrajoptError>
where
    D: FnMut(f64, &[f64], &[f64]) -> Result<Vec<f64>, TrajoptError>,
    L: FnMut(f64, &[f64], &[f64]) -> Result<f64, TrajoptError>,
{
    if report.sample_count != report.samples.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP stationarity Hamiltonian report sample count is inconsistent",
        });
    }
    let mut times_s = Vec::with_capacity(report.sample_count);
    let mut states = Vec::with_capacity(report.sample_count * report.state_dimension);
    let mut controls = Vec::with_capacity(report.sample_count * report.control_dimension);
    let mut costates = Vec::with_capacity(report.sample_count * report.state_dimension);
    for sample in &report.samples {
        times_s.push(sample.time_s);
        states.extend_from_slice(&sample.state);
        controls.extend_from_slice(&sample.control);
        costates.extend_from_slice(&sample.costate);
    }
    evaluate_pmp_hamiltonian_control_stationarity(
        PmpHamiltonianProfile {
            state_dimension: report.state_dimension,
            control_dimension: report.control_dimension,
            times_s: &times_s,
            states: &states,
            controls: &controls,
            costates: &costates,
        },
        settings,
        dynamics,
        lagrange,
    )
}

/// Apply an explicit direct-method-to-PMP costate sign/scale convention.
///
/// This helper intentionally does not infer a convention. Callers must pass the
/// scale that maps their direct-method residual-multiplier convention into the
/// PMP costate convention they want to compare.
pub fn apply_pmp_direct_costate_convention(
    state_dimension: usize,
    times_s: &[f64],
    raw_costates: &[f64],
    convention: PmpDirectCostateConvention,
) -> Result<PmpDirectCostateConventionReport, TrajoptError> {
    validate_direct_costate_convention(convention)?;
    if state_dimension == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP direct costate convention dimension must be nonzero",
        });
    }
    validate_time_grid(times_s)?;
    let sample_count = times_s.len();
    validate_len(
        "PMP direct costate convention dimensions are inconsistent",
        raw_costates,
        sample_count * state_dimension,
    )?;
    validate_finite("PMP direct costates must be finite", raw_costates)?;

    let mut samples = Vec::with_capacity(sample_count);
    let mut pmp_costates = Vec::with_capacity(raw_costates.len());
    let mut max_abs_raw_costate = 0.0_f64;
    let mut max_abs_pmp_costate = 0.0_f64;

    for (sample_index, time_s) in times_s.iter().copied().enumerate() {
        let raw_costate = block(raw_costates, state_dimension, sample_index);
        let mut pmp_costate = Vec::with_capacity(state_dimension);
        for &value in raw_costate {
            max_abs_raw_costate = max_abs_raw_costate.max(value.abs());
            let scaled = value * convention.raw_to_pmp_scale;
            max_abs_pmp_costate = max_abs_pmp_costate.max(scaled.abs());
            pmp_costate.push(scaled);
            pmp_costates.push(scaled);
        }
        samples.push(PmpDirectCostateConventionSample {
            sample_index,
            time_s,
            raw_costate: raw_costate.to_vec(),
            pmp_costate,
        });
    }

    Ok(PmpDirectCostateConventionReport {
        state_dimension,
        sample_count,
        raw_to_pmp_scale: convention.raw_to_pmp_scale,
        times_s: times_s.to_vec(),
        raw_costates: raw_costates.to_vec(),
        pmp_costates,
        max_abs_raw_costate,
        max_abs_pmp_costate,
        samples,
    })
}

/// Solve the scalar fixed-time minimum-energy integrator by costate shooting.
pub fn solve_pmp_scalar_min_energy_shooting(
    problem: PmpScalarMinEnergyProblem,
    settings: PmpScalarMinEnergyShootingSettings,
) -> Result<PmpScalarMinEnergySolution, TrajoptError> {
    validate_scalar_min_energy_problem(problem)?;
    validate_scalar_min_energy_settings(settings)?;

    let mut costate = settings.initial_costate_guess;
    let mut iteration_reports = Vec::new();
    let mut converged = false;

    for iteration in 0..settings.max_iterations {
        let terminal_state = scalar_min_energy_terminal_state(problem, costate);
        let terminal_residual = terminal_state - problem.terminal_state;
        if terminal_residual.abs() <= settings.terminal_tolerance {
            converged = true;
            break;
        }
        let residual_plus =
            scalar_min_energy_terminal_state(problem, costate + settings.finite_difference_step)
                - problem.terminal_state;
        let residual_minus =
            scalar_min_energy_terminal_state(problem, costate - settings.finite_difference_step)
                - problem.terminal_state;
        let residual_derivative =
            (residual_plus - residual_minus) / (2.0 * settings.finite_difference_step);
        if !residual_derivative.is_finite() || residual_derivative.abs() <= f64::EPSILON {
            return Err(TrajoptError::InvalidPayload {
                reason: "PMP scalar shooting residual derivative is singular",
            });
        }
        let costate_step = -terminal_residual / residual_derivative;
        if !costate_step.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "PMP scalar shooting step is non-finite",
            });
        }
        iteration_reports.push(PmpScalarMinEnergyIteration {
            iteration,
            costate_guess: costate,
            terminal_state,
            terminal_residual,
            residual_derivative,
            costate_step,
        });
        costate += costate_step;
    }

    let final_state = scalar_min_energy_terminal_state(problem, costate);
    let terminal_residual = final_state - problem.terminal_state;
    converged = converged || terminal_residual.abs() <= settings.terminal_tolerance;
    let control = scalar_min_energy_control(problem, costate);
    let duration_s = problem.final_time_s - problem.initial_time_s;
    let objective = 0.5 * problem.control_weight * control * control * duration_s;
    let hamiltonian_report =
        scalar_min_energy_hamiltonian_report(problem, settings.sample_count, costate)?;

    Ok(PmpScalarMinEnergySolution {
        converged,
        iterations: iteration_reports.len(),
        initial_costate: costate,
        control,
        final_state,
        terminal_residual,
        objective,
        hamiltonian_report,
        iteration_reports,
    })
}

fn scalar_min_energy_hamiltonian_report(
    problem: PmpScalarMinEnergyProblem,
    sample_count: usize,
    costate: f64,
) -> Result<PmpHamiltonianReport, TrajoptError> {
    let duration_s = problem.final_time_s - problem.initial_time_s;
    let control = scalar_min_energy_control(problem, costate);
    let mut times_s = Vec::with_capacity(sample_count);
    let mut states = Vec::with_capacity(sample_count);
    let mut controls = Vec::with_capacity(sample_count);
    let mut costates = Vec::with_capacity(sample_count);
    for sample_index in 0..sample_count {
        let fraction = sample_index as f64 / (sample_count - 1) as f64;
        let time_s = problem.initial_time_s + fraction * duration_s;
        times_s.push(time_s);
        states.push(problem.initial_state + control * (time_s - problem.initial_time_s));
        controls.push(control);
        costates.push(costate);
    }
    evaluate_pmp_hamiltonian_profile(
        PmpHamiltonianProfile {
            state_dimension: 1,
            control_dimension: 1,
            times_s: &times_s,
            states: &states,
            controls: &controls,
            costates: &costates,
        },
        |_, _, control| Ok(vec![control[0]]),
        |_, _, control| Ok(0.5 * problem.control_weight * control[0] * control[0]),
    )
}

fn scalar_min_energy_terminal_state(problem: PmpScalarMinEnergyProblem, costate: f64) -> f64 {
    let duration_s = problem.final_time_s - problem.initial_time_s;
    problem.initial_state + scalar_min_energy_control(problem, costate) * duration_s
}

fn scalar_min_energy_control(problem: PmpScalarMinEnergyProblem, costate: f64) -> f64 {
    -costate / problem.control_weight
}

fn validate_dimensions(
    state_dimension: usize,
    control_dimension: usize,
) -> Result<(), TrajoptError> {
    if state_dimension == 0 || control_dimension == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP state and control dimensions must be nonzero",
        });
    }
    Ok(())
}

fn validate_time_grid(times_s: &[f64]) -> Result<(), TrajoptError> {
    if times_s.len() < 2 {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP time grid must contain at least two samples",
        });
    }
    validate_finite("PMP time grid must be finite", times_s)?;
    for window in times_s.windows(2) {
        if window[1] <= window[0] {
            return Err(TrajoptError::InvalidPayload {
                reason: "PMP time grid must be strictly increasing",
            });
        }
    }
    Ok(())
}

fn validate_costate_comparison_settings(
    settings: PmpCostateComparisonSettings,
) -> Result<(), TrajoptError> {
    validate_nonnegative_finite(
        "PMP costate comparison time_tolerance_s must be finite and non-negative",
        settings.time_tolerance_s,
    )?;
    validate_nonnegative_finite(
        "PMP costate comparison costate_tolerance must be finite and non-negative",
        settings.costate_tolerance,
    )
}

fn validate_hamiltonian_control_stationarity_settings(
    settings: PmpHamiltonianControlStationaritySettings,
) -> Result<(), TrajoptError> {
    validate_positive_finite(
        "PMP stationarity finite_difference_step must be positive and finite",
        settings.finite_difference_step,
    )?;
    validate_nonnegative_finite(
        "PMP stationarity gradient_tolerance must be finite and non-negative",
        settings.gradient_tolerance,
    )
}

fn validate_direct_costate_convention(
    convention: PmpDirectCostateConvention,
) -> Result<(), TrajoptError> {
    if !convention.raw_to_pmp_scale.is_finite() || convention.raw_to_pmp_scale == 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP direct costate raw_to_pmp_scale must be finite and nonzero",
        });
    }
    Ok(())
}

fn validate_scalar_min_energy_problem(
    problem: PmpScalarMinEnergyProblem,
) -> Result<(), TrajoptError> {
    if !problem.initial_time_s.is_finite()
        || !problem.final_time_s.is_finite()
        || problem.final_time_s <= problem.initial_time_s
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP scalar shooting time bounds must be finite and ordered",
        });
    }
    if !problem.initial_state.is_finite() || !problem.terminal_state.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP scalar shooting states must be finite",
        });
    }
    validate_positive_finite(
        "PMP scalar shooting control_weight must be positive and finite",
        problem.control_weight,
    )
}

fn validate_scalar_min_energy_settings(
    settings: PmpScalarMinEnergyShootingSettings,
) -> Result<(), TrajoptError> {
    if settings.max_iterations == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP scalar shooting max_iterations must be nonzero",
        });
    }
    validate_positive_finite(
        "PMP scalar shooting terminal_tolerance must be positive and finite",
        settings.terminal_tolerance,
    )?;
    validate_positive_finite(
        "PMP scalar shooting finite_difference_step must be positive and finite",
        settings.finite_difference_step,
    )?;
    if settings.sample_count < 2 {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP scalar shooting sample_count must be at least two",
        });
    }
    if !settings.initial_costate_guess.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP scalar shooting initial_costate_guess must be finite",
        });
    }
    Ok(())
}

fn validate_len<T>(
    reason: &'static str,
    values: &[T],
    expected: usize,
) -> Result<(), TrajoptError> {
    if values.len() != expected {
        return Err(TrajoptError::InvalidPayload { reason });
    }
    Ok(())
}

fn validate_finite(reason: &'static str, values: &[f64]) -> Result<(), TrajoptError> {
    if values.iter().any(|value| !value.is_finite()) {
        return Err(TrajoptError::InvalidPayload { reason });
    }
    Ok(())
}

fn validate_positive_finite(reason: &'static str, value: f64) -> Result<(), TrajoptError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(TrajoptError::InvalidPayload { reason });
    }
    Ok(())
}

fn validate_nonnegative_finite(reason: &'static str, value: f64) -> Result<(), TrajoptError> {
    if !value.is_finite() || value < 0.0 {
        return Err(TrajoptError::InvalidPayload { reason });
    }
    Ok(())
}

fn block(values: &[f64], dimension: usize, index: usize) -> &[f64] {
    let start = index * dimension;
    &values[start..start + dimension]
}

fn evaluate_hamiltonian_value<D, L>(
    time_s: f64,
    state: &[f64],
    control: &[f64],
    costate: &[f64],
    state_dimension: usize,
    dynamics: &mut D,
    lagrange: &mut L,
) -> Result<f64, TrajoptError>
where
    D: FnMut(f64, &[f64], &[f64]) -> Result<Vec<f64>, TrajoptError>,
    L: FnMut(f64, &[f64], &[f64]) -> Result<f64, TrajoptError>,
{
    let dynamics_value = dynamics(time_s, state, control)?;
    validate_len(
        "PMP stationarity dynamics output dimensions are inconsistent",
        &dynamics_value,
        state_dimension,
    )?;
    validate_finite(
        "PMP stationarity dynamics output must be finite",
        &dynamics_value,
    )?;
    let lagrange_value = lagrange(time_s, state, control)?;
    if !lagrange_value.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP stationarity Lagrange output must be finite",
        });
    }
    let hamiltonian = lagrange_value
        + costate
            .iter()
            .zip(&dynamics_value)
            .map(|(a, b)| a * b)
            .sum::<f64>();
    if !hamiltonian.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "PMP stationarity Hamiltonian value must be finite",
        });
    }
    Ok(hamiltonian)
}

#[cfg(test)]
// Exact comparisons verify copied boundaries, sparse coefficients, and clamped values.
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn pmp_hamiltonian_profile_reports_constant_autonomous_arc() -> Result<(), TrajoptError> {
        let times_s = [0.0, 0.5, 1.0];
        let states = [0.0, 0.5, 1.0];
        let controls = [1.0, 1.0, 1.0];
        let costates = [-1.0, -1.0, -1.0];

        let report = evaluate_pmp_hamiltonian_profile(
            PmpHamiltonianProfile {
                state_dimension: 1,
                control_dimension: 1,
                times_s: &times_s,
                states: &states,
                controls: &controls,
                costates: &costates,
            },
            |_, _, control| Ok(vec![control[0]]),
            |_, _, control| Ok(0.5 * control[0] * control[0]),
        )?;

        assert_eq!(report.sample_count, 3);
        assert_eq!(report.initial_hamiltonian, -0.5);
        assert_eq!(report.max_abs_hamiltonian_drift, 0.0);
        for sample in &report.samples {
            assert_eq!(sample.hamiltonian, -0.5);
            assert_eq!(sample.hamiltonian_drift, 0.0);
        }
        Ok(())
    }

    #[test]
    fn pmp_scalar_min_energy_shooting_solves_and_compares_costates() -> Result<(), TrajoptError> {
        let problem = PmpScalarMinEnergyProblem {
            initial_time_s: 0.0,
            final_time_s: 4.0,
            initial_state: 0.0,
            terminal_state: 2.0,
            control_weight: 2.0,
        };
        let solution = solve_pmp_scalar_min_energy_shooting(problem, Default::default())?;

        assert!(solution.converged);
        assert!(solution.iterations <= 2);
        assert!((solution.initial_costate + 1.0).abs() < 1.0e-10);
        assert!((solution.control - 0.5).abs() < 1.0e-10);
        assert!(solution.terminal_residual.abs() < 1.0e-12);
        assert!((solution.objective - 1.0).abs() < 1.0e-10);
        assert!(solution.hamiltonian_report.max_abs_hamiltonian_drift < 1.0e-12);

        let times_s: Vec<_> = solution
            .hamiltonian_report
            .samples
            .iter()
            .map(|sample| sample.time_s)
            .collect();
        let pmp_costates: Vec<_> = solution
            .hamiltonian_report
            .samples
            .iter()
            .flat_map(|sample| sample.costate.iter().copied())
            .collect();
        let kkt_costates = pmp_costates.clone();
        let comparison = compare_pmp_costate_profiles(
            1,
            &times_s,
            &pmp_costates,
            &times_s,
            &kkt_costates,
            Default::default(),
        )?;
        assert!(comparison.within_tolerance);
        assert_eq!(comparison.max_costate_norm_difference, 0.0);
        Ok(())
    }

    #[test]
    fn pmp_control_stationarity_reports_scalar_min_energy_optimum() -> Result<(), TrajoptError> {
        let problem = PmpScalarMinEnergyProblem {
            initial_time_s: 0.0,
            final_time_s: 4.0,
            initial_state: 0.0,
            terminal_state: 2.0,
            control_weight: 2.0,
        };
        let solution = solve_pmp_scalar_min_energy_shooting(
            problem,
            PmpScalarMinEnergyShootingSettings {
                sample_count: 5,
                ..Default::default()
            },
        )?;
        assert!(solution.converged);

        let times_s: Vec<_> = solution
            .hamiltonian_report
            .samples
            .iter()
            .map(|sample| sample.time_s)
            .collect();
        let states: Vec<_> = solution
            .hamiltonian_report
            .samples
            .iter()
            .flat_map(|sample| sample.state.iter().copied())
            .collect();
        let controls: Vec<_> = solution
            .hamiltonian_report
            .samples
            .iter()
            .flat_map(|sample| sample.control.iter().copied())
            .collect();
        let costates: Vec<_> = solution
            .hamiltonian_report
            .samples
            .iter()
            .flat_map(|sample| sample.costate.iter().copied())
            .collect();

        let report = evaluate_pmp_hamiltonian_control_stationarity(
            PmpHamiltonianProfile {
                state_dimension: 1,
                control_dimension: 1,
                times_s: &times_s,
                states: &states,
                controls: &controls,
                costates: &costates,
            },
            PmpHamiltonianControlStationaritySettings {
                gradient_tolerance: 1.0e-9,
                ..Default::default()
            },
            |_, _, control| Ok(vec![control[0]]),
            |_, _, control| Ok(0.5 * problem.control_weight * control[0] * control[0]),
        )?;

        assert_eq!(report.sample_count, 5);
        assert!(report.within_tolerance);
        assert!(report.max_gradient_norm <= 1.0e-9);
        for sample in &report.samples {
            assert!(sample.max_abs_gradient <= 1.0e-9);
            assert!(sample.gradient_norm <= 1.0e-9);
        }
        Ok(())
    }

    #[test]
    fn pmp_control_stationarity_reports_off_optimum_control() -> Result<(), TrajoptError> {
        let times_s = [0.0, 1.0];
        let states = [0.0, 0.5];
        let controls = [0.25, 0.25];
        let costates = [-1.0, -1.0];

        let report = evaluate_pmp_hamiltonian_control_stationarity(
            PmpHamiltonianProfile {
                state_dimension: 1,
                control_dimension: 1,
                times_s: &times_s,
                states: &states,
                controls: &controls,
                costates: &costates,
            },
            PmpHamiltonianControlStationaritySettings {
                gradient_tolerance: 1.0e-9,
                ..Default::default()
            },
            |_, _, control| Ok(vec![control[0]]),
            |_, _, control| Ok(control[0] * control[0]),
        )?;

        assert!(!report.within_tolerance);
        assert!((report.max_gradient_norm - 0.5).abs() < 1.0e-10);
        for sample in &report.samples {
            assert!((sample.gradient[0] + 0.5).abs() < 1.0e-10);
        }
        Ok(())
    }

    #[test]
    fn pmp_scalar_min_energy_shooting_is_repeat_deterministic() -> Result<(), TrajoptError> {
        let problem = PmpScalarMinEnergyProblem {
            initial_time_s: 1.0,
            final_time_s: 3.5,
            initial_state: -2.0,
            terminal_state: 1.0,
            control_weight: 3.0,
        };
        let settings = PmpScalarMinEnergyShootingSettings {
            initial_costate_guess: 2.0,
            sample_count: 6,
            ..Default::default()
        };
        let first = solve_pmp_scalar_min_energy_shooting(problem, settings)?;
        let second = solve_pmp_scalar_min_energy_shooting(problem, settings)?;

        assert_eq!(first.converged, second.converged);
        assert_eq!(first.iterations, second.iterations);
        assert_eq!(
            first.initial_costate.to_bits(),
            second.initial_costate.to_bits()
        );
        assert_eq!(first.control.to_bits(), second.control.to_bits());
        assert_eq!(first.final_state.to_bits(), second.final_state.to_bits());
        assert_eq!(first.objective.to_bits(), second.objective.to_bits());
        assert_eq!(first.hamiltonian_report, second.hamiltonian_report);
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn pmp_scalar_min_energy_dense_qp_dual_matches_costate() -> Result<(), TrajoptError> {
        let problem = PmpScalarMinEnergyProblem {
            initial_time_s: 0.0,
            final_time_s: 4.0,
            initial_state: 0.0,
            terminal_state: 2.0,
            control_weight: 2.0,
        };
        let pmp_solution = solve_pmp_scalar_min_energy_shooting(problem, Default::default())?;
        let duration_s = problem.final_time_s - problem.initial_time_s;
        let dense_qp = crate::sqp::DenseQpProblem::new(
            1,
            vec![problem.control_weight * duration_s],
            vec![0.0],
            vec![duration_s],
            vec![problem.terminal_state - problem.initial_state],
            Vec::new(),
            Vec::new(),
        )?;
        let qp_solution = crate::sqp::solve_dense_qp(&dense_qp)?;

        assert_eq!(qp_solution.equality_dual.len(), 1);
        assert!(qp_solution.inequality_dual.is_empty());
        assert!((qp_solution.primal[0] - pmp_solution.control).abs() < 1.0e-9);
        assert!(
            (problem.control_weight * duration_s * qp_solution.primal[0]
                + duration_s * qp_solution.equality_dual[0])
                .abs()
                < 1.0e-7
        );

        let times_s: Vec<_> = pmp_solution
            .hamiltonian_report
            .samples
            .iter()
            .map(|sample| sample.time_s)
            .collect();
        let pmp_costates: Vec<_> = pmp_solution
            .hamiltonian_report
            .samples
            .iter()
            .flat_map(|sample| sample.costate.iter().copied())
            .collect();
        let qp_costates = vec![qp_solution.equality_dual[0]; pmp_costates.len()];
        let comparison = compare_pmp_costate_profiles(
            1,
            &times_s,
            &pmp_costates,
            &times_s,
            &qp_costates,
            PmpCostateComparisonSettings {
                costate_tolerance: 1.0e-7,
                ..Default::default()
            },
        )?;
        assert!(comparison.within_tolerance);
        assert!(comparison.max_costate_norm_difference < 1.0e-7);
        Ok(())
    }

    #[test]
    fn pmp_direct_costate_convention_scales_raw_profile_for_comparison() -> Result<(), TrajoptError>
    {
        let problem = PmpScalarMinEnergyProblem {
            initial_time_s: 0.0,
            final_time_s: 4.0,
            initial_state: 0.0,
            terminal_state: 2.0,
            control_weight: 2.0,
        };
        let pmp_solution = solve_pmp_scalar_min_energy_shooting(problem, Default::default())?;
        let times_s: Vec<_> = pmp_solution
            .hamiltonian_report
            .samples
            .iter()
            .map(|sample| sample.time_s)
            .collect();
        let pmp_costates: Vec<_> = pmp_solution
            .hamiltonian_report
            .samples
            .iter()
            .flat_map(|sample| sample.costate.iter().copied())
            .collect();
        let raw_to_pmp_scale = -0.5;
        let raw_costates = pmp_costates
            .iter()
            .map(|costate| costate / raw_to_pmp_scale)
            .collect::<Vec<_>>();

        let convention_report = apply_pmp_direct_costate_convention(
            1,
            &times_s,
            &raw_costates,
            PmpDirectCostateConvention { raw_to_pmp_scale },
        )?;

        assert_eq!(convention_report.state_dimension, 1);
        assert_eq!(convention_report.sample_count, times_s.len());
        assert_eq!(convention_report.times_s, times_s);
        assert_eq!(convention_report.pmp_costates, pmp_costates);
        assert!((convention_report.max_abs_raw_costate - 2.0).abs() < 1.0e-12);
        assert!((convention_report.max_abs_pmp_costate - 1.0).abs() < 1.0e-12);

        let comparison = compare_pmp_costate_profiles(
            1,
            &convention_report.times_s,
            &pmp_costates,
            &convention_report.times_s,
            &convention_report.pmp_costates,
            PmpCostateComparisonSettings {
                costate_tolerance: 1.0e-12,
                ..Default::default()
            },
        )?;
        assert!(comparison.within_tolerance);
        assert_eq!(comparison.max_costate_norm_difference, 0.0);
        Ok(())
    }

    #[test]
    fn pmp_rejects_malformed_inputs() {
        assert!(matches!(
            evaluate_pmp_hamiltonian_profile(
                PmpHamiltonianProfile {
                    state_dimension: 0,
                    control_dimension: 1,
                    times_s: &[0.0, 1.0],
                    states: &[0.0, 1.0],
                    controls: &[0.0, 1.0],
                    costates: &[0.0, 1.0],
                },
                |_, _, _| Ok(vec![0.0]),
                |_, _, _| Ok(0.0),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            evaluate_pmp_hamiltonian_profile(
                PmpHamiltonianProfile {
                    state_dimension: 1,
                    control_dimension: 1,
                    times_s: &[0.0, 1.0],
                    states: &[0.0],
                    controls: &[0.0, 1.0],
                    costates: &[0.0, 1.0],
                },
                |_, _, _| Ok(vec![0.0]),
                |_, _, _| Ok(0.0),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            evaluate_pmp_hamiltonian_profile(
                PmpHamiltonianProfile {
                    state_dimension: 1,
                    control_dimension: 1,
                    times_s: &[0.0, 1.0],
                    states: &[0.0, 1.0],
                    controls: &[0.0, 1.0],
                    costates: &[0.0, 1.0],
                },
                |_, _, _| Ok(vec![0.0, 0.0]),
                |_, _, _| Ok(0.0),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            compare_pmp_costate_profiles(
                1,
                &[0.0, 1.0],
                &[0.0, 0.0],
                &[0.0, 1.1],
                &[0.0, 0.0],
                Default::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            evaluate_pmp_hamiltonian_control_stationarity(
                PmpHamiltonianProfile {
                    state_dimension: 1,
                    control_dimension: 1,
                    times_s: &[0.0, 1.0],
                    states: &[0.0, 1.0],
                    controls: &[0.0],
                    costates: &[0.0, 0.0],
                },
                Default::default(),
                |_, _, control| Ok(vec![control[0]]),
                |_, _, _| Ok(0.0),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            evaluate_pmp_hamiltonian_control_stationarity(
                PmpHamiltonianProfile {
                    state_dimension: 1,
                    control_dimension: 1,
                    times_s: &[0.0, 1.0],
                    states: &[0.0, 1.0],
                    controls: &[0.0, 1.0],
                    costates: &[0.0, 0.0],
                },
                PmpHamiltonianControlStationaritySettings {
                    finite_difference_step: 0.0,
                    ..Default::default()
                },
                |_, _, control| Ok(vec![control[0]]),
                |_, _, _| Ok(0.0),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            apply_pmp_direct_costate_convention(
                1,
                &[0.0, 1.0],
                &[1.0, 2.0],
                PmpDirectCostateConvention {
                    raw_to_pmp_scale: 0.0,
                },
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            apply_pmp_direct_costate_convention(
                1,
                &[0.0, 1.0],
                &[1.0],
                PmpDirectCostateConvention::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            solve_pmp_scalar_min_energy_shooting(
                PmpScalarMinEnergyProblem {
                    initial_time_s: 1.0,
                    final_time_s: 1.0,
                    initial_state: 0.0,
                    terminal_state: 1.0,
                    control_weight: 1.0,
                },
                Default::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            solve_pmp_scalar_min_energy_shooting(
                PmpScalarMinEnergyProblem {
                    initial_time_s: 0.0,
                    final_time_s: 1.0,
                    initial_state: 0.0,
                    terminal_state: 1.0,
                    control_weight: 0.0,
                },
                Default::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }
}
