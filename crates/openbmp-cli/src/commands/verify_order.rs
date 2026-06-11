//! Observed-order and Richardson/GCI verification command.

use std::fmt::Write as _;
use std::fs;
use std::ops::{Add, Mul};
use std::path::{Path, PathBuf};

use openbmp_core::{Duration, SimTime};
use openbmp_sim::{
    Dopri853FixedStep, Integratable, Integrator, ModelEvalError, Rk4FixedStep, SimStateDerivative,
    VehicleState,
};
use openbmp_testkit::analytic::ManufacturedScalarOde;
use openbmp_testkit::verification::{
    OrderVerificationReport, StepError, order_verification_report,
};

use crate::CliError;
use crate::cli::VerifyOrderMethod;

const STOP_S: f64 = 4.0;

/// CLI report for one `openbmp verify-order` invocation.
#[derive(Clone, Debug, PartialEq)]
pub struct VerifyOrderCliReport {
    /// User-requested method selection.
    pub requested_method: VerifyOrderMethod,
    /// Per-method order/GCI reports.
    pub methods: Vec<VerifyOrderMethodReport>,
    /// Evidence TOML path, when requested and written.
    pub output_toml: Option<PathBuf>,
}

/// Order/GCI report plus command acceptance metadata for one method.
#[derive(Clone, Debug, PartialEq)]
pub struct VerifyOrderMethodReport {
    /// Observed-order and GCI calculations.
    pub report: OrderVerificationReport,
    /// Inclusive lower bound for both observed-order estimates.
    pub min_observed_order: f64,
    /// Inclusive upper bound for both observed-order estimates.
    pub max_observed_order: f64,
    /// Whether the method passed the command's acceptance gate.
    pub passed: bool,
}

/// Run the observed-order verification gate.
///
/// # Errors
///
/// Returns [`CliError`] when the real integrator run fails, the
/// observed-order reducer rejects the samples, the acceptance gate
/// fails, or the optional evidence TOML cannot be written.
pub fn run(
    requested_method: VerifyOrderMethod,
    output_toml: Option<&Path>,
) -> Result<VerifyOrderCliReport, CliError> {
    let mut report = VerifyOrderCliReport {
        requested_method,
        methods: Vec::new(),
        output_toml: None,
    };
    for method in selected_methods(requested_method) {
        report.methods.push(run_one(method)?);
    }

    if let Some(path) = output_toml {
        write_report_toml(path, &report)?;
        report.output_toml = Some(path.to_path_buf());
    }

    if let Some(method) = report.methods.iter().find(|method| !method.passed) {
        return Err(CliError::CodeVerification {
            summary: format!(
                "{} observed order outside [{:.3}, {:.3}] or GCI acceptance failed \
                 (p_cm={:.6}, p_mf={:.6}, gci_monotone={}, gci_brackets={})",
                method.report.method,
                method.min_observed_order,
                method.max_observed_order,
                method.report.observed_order_coarse_medium,
                method.report.observed_order_medium_fine,
                method.report.gci_monotone,
                method.report.gci_brackets_richardson_error,
            ),
        });
    }

    Ok(report)
}

fn selected_methods(requested_method: VerifyOrderMethod) -> Vec<IntegratorUnderTest> {
    match requested_method {
        VerifyOrderMethod::All => vec![IntegratorUnderTest::Rk4, IntegratorUnderTest::Dop853],
        VerifyOrderMethod::Rk4 => vec![IntegratorUnderTest::Rk4],
        VerifyOrderMethod::Dop853 => vec![IntegratorUnderTest::Dop853],
    }
}

fn run_one(method: IntegratorUnderTest) -> Result<VerifyOrderMethodReport, CliError> {
    let (coarse_h, medium_h, fine_h) = method.step_sizes_s();
    let ode = ManufacturedScalarOde::STANDARD;
    let coarse = StepError {
        step_s: coarse_h,
        error: integrate(method, ode, coarse_h)?,
    };
    let medium = StepError {
        step_s: medium_h,
        error: integrate(method, ode, medium_h)?,
    };
    let fine = StepError {
        step_s: fine_h,
        error: integrate(method, ode, fine_h)?,
    };
    let report =
        order_verification_report(method.label(), method.formal_order(), coarse, medium, fine)
            .map_err(|err| CliError::CodeVerification {
                summary: err.to_string(),
            })?;
    let (min_observed_order, max_observed_order) = method.order_band();
    let passed = report.observed_order_in_band(min_observed_order, max_observed_order)
        && report.gci_monotone
        && report.gci_brackets_richardson_error;
    Ok(VerifyOrderMethodReport {
        report,
        min_observed_order,
        max_observed_order,
        passed,
    })
}

fn integrate(
    method: IntegratorUnderTest,
    ode: ManufacturedScalarOde,
    step_s: f64,
) -> Result<f64, CliError> {
    match method {
        IntegratorUnderTest::Rk4 => integrate_fixed_step(&Rk4FixedStep, method, ode, step_s),
        IntegratorUnderTest::Dop853 => {
            integrate_fixed_step(&Dopri853FixedStep, method, ode, step_s)
        }
    }
}

fn integrate_fixed_step<I>(
    integrator: &I,
    method: IntegratorUnderTest,
    ode: ManufacturedScalarOde,
    step_s: f64,
) -> Result<f64, CliError>
where
    I: Integrator<ScalarState>,
{
    if !step_s.is_finite() || step_s <= 0.0 {
        return Err(CliError::CodeVerification {
            summary: format!("{} step size must be positive and finite", method.label()),
        });
    }
    let step_count = step_count(step_s)?;
    let dt = Duration::from_seconds(step_s);
    let mut state = ScalarState {
        time: SimTime::ZERO,
        value: ode.exact_at(0.0),
    };
    for step_index in 0..step_count {
        state = integrator
            .advance(
                &state,
                |stage, time| -> Result<ScalarDerivative, ModelEvalError> {
                    Ok(ScalarDerivative {
                        value_rate: ode.rhs(time.as_seconds(), stage.value),
                    })
                },
                dt,
            )
            .map_err(|source| CliError::CodeVerification {
                summary: format!(
                    "{} integration failed at step {} with h={:.6e}: {}",
                    method.label(),
                    step_index,
                    step_s,
                    source
                ),
            })?;
        let canonical_time = SimTime::from_seconds((step_index + 1) as f64 * step_s);
        state = state.with_time(canonical_time);
    }
    let exact = ode.exact_at(STOP_S);
    Ok((state.value - exact).abs())
}

fn step_count(step_s: f64) -> Result<u64, CliError> {
    let raw = STOP_S / step_s;
    let rounded = raw.round();
    if !raw.is_finite() || (raw - rounded).abs() > 1.0e-10 {
        return Err(CliError::CodeVerification {
            summary: format!("stop time {STOP_S:.6e} is not an integer multiple of h={step_s:.6e}"),
        });
    }
    Ok(rounded as u64)
}

fn write_report_toml(path: &Path, report: &VerifyOrderCliReport) -> Result<(), CliError> {
    ensure_parent_dir(path)?;
    let mut out = String::new();
    let ode = ManufacturedScalarOde::STANDARD;
    let _ = writeln!(out, "[verify_order]");
    let _ = writeln!(
        out,
        "requested_method = {}",
        toml_string(selection_label(report.requested_method))
    );
    let _ = writeln!(out, "case = \"manufactured_scalar_ode\"");
    let _ = writeln!(out, "lambda_per_s = {:.17e}", ode.lambda_per_s);
    let _ = writeln!(out, "stop_s = {:.17e}", STOP_S);
    let _ = writeln!(out, "exact_state = \"y(t) = 0.75 + sin(0.3 t) + 0.05 t^3\"");
    let _ = writeln!(
        out,
        "source_term = \"source(t) = y_exact'(t) + lambda * y_exact(t)\""
    );
    let _ = writeln!(out, "method_count = {}", report.methods.len());
    for method in &report.methods {
        let report = &method.report;
        let _ = writeln!(out);
        let _ = writeln!(out, "[[verify_order.methods]]");
        let _ = writeln!(out, "method = {}", toml_string(&report.method));
        let _ = writeln!(out, "formal_order = {:.17e}", report.formal_order);
        let _ = writeln!(
            out,
            "min_observed_order = {:.17e}",
            method.min_observed_order
        );
        let _ = writeln!(
            out,
            "max_observed_order = {:.17e}",
            method.max_observed_order
        );
        let _ = writeln!(out, "passed = {}", method.passed);
        let _ = writeln!(out, "refinement_ratio = {:.17e}", report.refinement_ratio);
        let _ = writeln!(out, "safety_factor = {:.17e}", report.safety_factor);
        let _ = writeln!(out, "coarse_step_s = {:.17e}", report.coarse.step_s);
        let _ = writeln!(out, "coarse_error = {:.17e}", report.coarse.error);
        let _ = writeln!(out, "medium_step_s = {:.17e}", report.medium.step_s);
        let _ = writeln!(out, "medium_error = {:.17e}", report.medium.error);
        let _ = writeln!(out, "fine_step_s = {:.17e}", report.fine.step_s);
        let _ = writeln!(out, "fine_error = {:.17e}", report.fine.error);
        let _ = writeln!(
            out,
            "observed_order_coarse_medium = {:.17e}",
            report.observed_order_coarse_medium
        );
        let _ = writeln!(
            out,
            "observed_order_medium_fine = {:.17e}",
            report.observed_order_medium_fine
        );
        let _ = writeln!(
            out,
            "richardson_error_fine = {:.17e}",
            report.richardson_error_fine
        );
        let _ = writeln!(out, "gci_fine = {:.17e}", report.gci_fine);
        let _ = writeln!(out, "gci_medium = {:.17e}", report.gci_medium);
        let _ = writeln!(out, "gci_monotone = {}", report.gci_monotone);
        let _ = writeln!(
            out,
            "gci_brackets_richardson_error = {}",
            report.gci_brackets_richardson_error
        );
    }

    fs::write(path, out).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn ensure_parent_dir(path: &Path) -> Result<(), CliError> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent).map_err(|source| CliError::Io {
            path: parent.to_path_buf(),
            source,
        })?;
    }
    Ok(())
}

fn toml_string(value: &str) -> String {
    let escaped = value.replace('\\', "\\\\").replace('"', "\\\"");
    format!("\"{escaped}\"")
}

fn selection_label(method: VerifyOrderMethod) -> &'static str {
    match method {
        VerifyOrderMethod::All => "all",
        VerifyOrderMethod::Rk4 => "rk4",
        VerifyOrderMethod::Dop853 => "dop853",
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum IntegratorUnderTest {
    Rk4,
    Dop853,
}

impl IntegratorUnderTest {
    const fn label(self) -> &'static str {
        match self {
            Self::Rk4 => "rk4",
            Self::Dop853 => "dop853",
        }
    }

    const fn formal_order(self) -> f64 {
        match self {
            Self::Rk4 => 4.0,
            Self::Dop853 => 8.0,
        }
    }

    const fn order_band(self) -> (f64, f64) {
        match self {
            Self::Rk4 => (3.8, 4.2),
            Self::Dop853 => (7.5, 8.5),
        }
    }

    const fn step_sizes_s(self) -> (f64, f64, f64) {
        match self {
            Self::Rk4 => (0.2, 0.1, 0.05),
            Self::Dop853 => (1.0, 0.5, 0.25),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ScalarState {
    time: SimTime,
    value: f64,
}

impl VehicleState for ScalarState {
    fn time(&self) -> SimTime {
        self.time
    }

    fn is_finite(&self) -> bool {
        self.time.is_valid() && self.value.is_finite()
    }

    fn with_time(mut self, t: SimTime) -> Self {
        self.time = t;
        self
    }
}

impl Integratable for ScalarState {
    type Derivative = ScalarDerivative;

    fn advance_by(&self, h_seconds: f64, derivative: &Self::Derivative) -> Self {
        Self {
            time: SimTime::from_seconds(self.time.as_seconds() + h_seconds),
            value: self.value + h_seconds * derivative.value_rate,
        }
    }

    fn scalar_state_size(&self) -> f64 {
        self.value.abs()
    }

    fn weighted_error_norm(
        &self,
        prev_state: &Self,
        error_deriv: &Self::Derivative,
        h: f64,
        atol: f64,
        rtol: f64,
    ) -> f64 {
        let scale = atol + rtol * self.value.abs().max(prev_state.value.abs());
        (h * error_deriv.value_rate / scale).abs()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ScalarDerivative {
    value_rate: f64,
}

impl Add for ScalarDerivative {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self {
            value_rate: self.value_rate + rhs.value_rate,
        }
    }
}

impl Mul<f64> for ScalarDerivative {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Self {
            value_rate: self.value_rate * rhs,
        }
    }
}

impl SimStateDerivative for ScalarDerivative {
    fn is_finite(&self) -> bool {
        self.value_rate.is_finite()
    }

    fn l2_norm(&self) -> f64 {
        self.value_rate.abs()
    }

    fn dimension(&self) -> usize {
        1
    }
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use super::*;

    #[test]
    fn verify_order_runs_rk4_and_dop853() -> Result<(), Box<dyn Error>> {
        let report = run(VerifyOrderMethod::All, None)?;

        assert_eq!(report.methods.len(), 2);
        assert!(report.methods.iter().all(|method| method.passed));
        assert!(
            report
                .methods
                .iter()
                .any(|method| method.report.method == "rk4")
        );
        assert!(
            report
                .methods
                .iter()
                .any(|method| method.report.method == "dop853")
        );
        Ok(())
    }

    #[test]
    fn verify_order_writes_toml() -> Result<(), Box<dyn Error>> {
        let temp = tempfile::tempdir()?;
        let output = temp.path().join("evidence").join("verify-order.toml");

        let report = run(VerifyOrderMethod::Rk4, Some(&output))?;

        assert_eq!(report.output_toml.as_deref(), Some(output.as_path()));
        let text = fs::read_to_string(&output)?;
        let _: toml::Value = toml::from_str(&text)?;
        assert!(text.contains("[[verify_order.methods]]"));
        assert!(text.contains("method = \"rk4\""));
        assert!(text.contains("exact_state"));
        assert!(text.contains("source_term"));
        Ok(())
    }
}
