//! Hermite-Simpson direct-collocation transcription primitives.
//!
//! This module is the fixed-mesh transcription substrate for WP-07.2. It
//! evaluates Hermite-Simpson dynamics defects, Hermite midpoint defects, and
//! Simpson Lagrange cost for caller-supplied dynamics and integrand closures.
//! It does not add target vocabulary and it does not solve the SQP subproblem.

use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use core::ops::{Add, Div, Mul, Neg, Sub};

use crate::iload::ILoadOptimalityEvidenceKind;
use crate::iload::{
    GainTable, ILoadHeader, ILoadObjectiveClass, ILoadOptimalityCertificate, ILoadPayload,
    ReferenceProfileSample, SynthesisMetadata, TrajoptError,
};
use openbmp_physics::profile::TerminalCondition;

/// Dynamics callback used by the Hermite-Simpson transcription.
pub trait CollocationDynamics {
    /// Evaluate `x_dot = f(t, x, u)`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dynamics cannot be evaluated.
    fn eval(
        &mut self,
        time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError>;
}

/// Dynamics value plus first derivatives for analytic fixed-mesh Jacobians.
#[derive(Clone, Debug, PartialEq)]
pub struct CollocationDynamicsJacobianSample {
    /// Dynamics derivative `f(t, x, u)`.
    pub derivative: Vec<f64>,
    /// Row-major state Jacobian `df/dx`, with one row per state derivative.
    pub state_jacobian: Vec<f64>,
    /// Row-major control Jacobian `df/du`, with one row per state derivative.
    pub control_jacobian: Vec<f64>,
}

impl CollocationDynamicsJacobianSample {
    /// Build a dynamics derivative sample with row-major Jacobian blocks.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when dimensions are inconsistent or any value is
    /// non-finite.
    pub fn new(
        derivative: Vec<f64>,
        state_jacobian: Vec<f64>,
        control_jacobian: Vec<f64>,
        state_dimension: usize,
        control_dimension: usize,
    ) -> Result<Self, TrajoptError> {
        let sample = Self {
            derivative,
            state_jacobian,
            control_jacobian,
        };
        validate_dynamics_jacobian_sample(&sample, state_dimension, control_dimension)?;
        Ok(sample)
    }
}

/// Dynamics callback with caller-supplied first derivatives.
///
/// This is the analytic derivative injection path for WP-07.2 fixed meshes.
/// Automatic differentiation can implement this trait later; this trait itself
/// does not require a specific AD backend.
pub trait CollocationDynamicsJacobian {
    /// Evaluate `x_dot = f(t, x, u)` and row-major Jacobian blocks.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dynamics or Jacobian cannot be
    /// evaluated.
    fn eval_with_jacobian(
        &mut self,
        time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<CollocationDynamicsJacobianSample, TrajoptError>;
}

/// Forward-mode automatic-differentiation scalar for fixed-mesh dynamics.
///
/// The `derivatives` vector stores partial derivatives with respect to the
/// seeded state variables followed by control variables. Arithmetic treats an
/// empty derivative vector as a constant, which lets dynamics return constants
/// without knowing the active derivative dimension.
#[derive(Clone, Debug, PartialEq)]
pub struct CollocationAutoDiffScalar {
    /// Primal scalar value.
    pub value: f64,
    /// Partial derivatives in state-then-control variable order.
    pub derivatives: Vec<f64>,
}

impl CollocationAutoDiffScalar {
    /// Build an automatic-differentiation scalar with explicit derivatives.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the value or any derivative is non-finite.
    pub fn new(value: f64, derivatives: Vec<f64>) -> Result<Self, TrajoptError> {
        let scalar = Self { value, derivatives };
        validate_autodiff_scalar(&scalar)?;
        Ok(scalar)
    }

    /// Build a scalar constant with no active derivatives.
    #[must_use]
    pub fn constant(value: f64) -> Self {
        Self {
            value,
            derivatives: Vec::new(),
        }
    }

    /// Build a scalar constant with a fixed derivative vector length.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the value is non-finite.
    pub fn constant_with_derivatives(
        value: f64,
        derivative_count: usize,
    ) -> Result<Self, TrajoptError> {
        Self::new(value, vec![0.0; derivative_count])
    }

    /// Build a seeded independent variable.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the value is non-finite or `derivative_index`
    /// is outside `derivative_count`.
    pub fn variable(
        value: f64,
        derivative_count: usize,
        derivative_index: usize,
    ) -> Result<Self, TrajoptError> {
        if derivative_index >= derivative_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson autodiff variable derivative index is out of range",
            });
        }
        let mut derivatives = vec![0.0; derivative_count];
        derivatives[derivative_index] = 1.0;
        Self::new(value, derivatives)
    }

    /// Return `sin(self)` with propagated derivatives.
    #[must_use]
    pub fn sin(&self) -> Self {
        self.map_unary(self.value.sin(), self.value.cos())
    }

    /// Return `cos(self)` with propagated derivatives.
    #[must_use]
    pub fn cos(&self) -> Self {
        self.map_unary(self.value.cos(), -self.value.sin())
    }

    /// Return `exp(self)` with propagated derivatives.
    #[must_use]
    pub fn exp(&self) -> Self {
        let value = self.value.exp();
        self.map_unary(value, value)
    }

    /// Return `ln(self)` with propagated derivatives.
    #[must_use]
    pub fn ln(&self) -> Self {
        self.map_unary(self.value.ln(), 1.0 / self.value)
    }

    /// Return `sqrt(self)` with propagated derivatives.
    #[must_use]
    pub fn sqrt(&self) -> Self {
        let value = self.value.sqrt();
        self.map_unary(value, 0.5 / value)
    }

    /// Return `self.powi(power)` with propagated derivatives.
    #[must_use]
    pub fn powi(&self, power: i32) -> Self {
        let value = self.value.powi(power);
        let derivative_scale = if power == 0 {
            0.0
        } else {
            f64::from(power) * self.value.powi(power - 1)
        };
        self.map_unary(value, derivative_scale)
    }

    fn map_unary(&self, value: f64, derivative_scale: f64) -> Self {
        Self {
            value,
            derivatives: self
                .derivatives
                .iter()
                .map(|derivative| derivative_scale * derivative)
                .collect(),
        }
    }
}

impl Add for CollocationAutoDiffScalar {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value + rhs.value,
            derivatives: binary_derivatives(&self.derivatives, &rhs.derivatives, |left, right| {
                left + right
            }),
        }
    }
}

impl Add<f64> for CollocationAutoDiffScalar {
    type Output = Self;

    fn add(self, rhs: f64) -> Self::Output {
        Self {
            value: self.value + rhs,
            derivatives: self.derivatives,
        }
    }
}

impl Sub for CollocationAutoDiffScalar {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value - rhs.value,
            derivatives: binary_derivatives(&self.derivatives, &rhs.derivatives, |left, right| {
                left - right
            }),
        }
    }
}

impl Sub<f64> for CollocationAutoDiffScalar {
    type Output = Self;

    fn sub(self, rhs: f64) -> Self::Output {
        Self {
            value: self.value - rhs,
            derivatives: self.derivatives,
        }
    }
}

#[allow(clippy::suspicious_arithmetic_impl)]
impl Mul for CollocationAutoDiffScalar {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        let derivatives = binary_derivatives(&self.derivatives, &rhs.derivatives, |left, right| {
            left * rhs.value + right * self.value
        });
        Self {
            value: self.value * rhs.value,
            derivatives,
        }
    }
}

impl Mul<f64> for CollocationAutoDiffScalar {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            value: self.value * rhs,
            derivatives: self
                .derivatives
                .iter()
                .map(|derivative| derivative * rhs)
                .collect(),
        }
    }
}

impl Div for CollocationAutoDiffScalar {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        let denominator = rhs.value * rhs.value;
        let derivatives = binary_derivatives(&self.derivatives, &rhs.derivatives, |left, right| {
            (left * rhs.value - self.value * right) / denominator
        });
        Self {
            value: self.value / rhs.value,
            derivatives,
        }
    }
}

impl Div<f64> for CollocationAutoDiffScalar {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self {
            value: self.value / rhs,
            derivatives: self
                .derivatives
                .iter()
                .map(|derivative| derivative / rhs)
                .collect(),
        }
    }
}

impl Neg for CollocationAutoDiffScalar {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            value: -self.value,
            derivatives: self
                .derivatives
                .iter()
                .map(|derivative| -derivative)
                .collect(),
        }
    }
}

/// Dynamics callback that evaluates over forward-mode AD scalars.
pub trait CollocationAutoDiffDynamics {
    /// Evaluate `x_dot = f(t, x, u)` using AD scalar inputs.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dynamics cannot be evaluated.
    fn eval_dual(
        &mut self,
        time_s: f64,
        state: &[CollocationAutoDiffScalar],
        control: &[CollocationAutoDiffScalar],
    ) -> Result<Vec<CollocationAutoDiffScalar>, TrajoptError>;
}

impl<F> CollocationAutoDiffDynamics for F
where
    F: for<'state, 'control> FnMut(
        f64,
        &'state [CollocationAutoDiffScalar],
        &'control [CollocationAutoDiffScalar],
    ) -> Result<Vec<CollocationAutoDiffScalar>, TrajoptError>,
{
    fn eval_dual(
        &mut self,
        time_s: f64,
        state: &[CollocationAutoDiffScalar],
        control: &[CollocationAutoDiffScalar],
    ) -> Result<Vec<CollocationAutoDiffScalar>, TrajoptError> {
        self(time_s, state, control)
    }
}

struct BorrowedAutoDiffDynamics<'a, D> {
    dynamics: &'a mut D,
}

impl<D> CollocationAutoDiffDynamics for BorrowedAutoDiffDynamics<'_, D>
where
    D: CollocationAutoDiffDynamics,
{
    fn eval_dual(
        &mut self,
        time_s: f64,
        state: &[CollocationAutoDiffScalar],
        control: &[CollocationAutoDiffScalar],
    ) -> Result<Vec<CollocationAutoDiffScalar>, TrajoptError> {
        self.dynamics.eval_dual(time_s, state, control)
    }
}

/// Adapter that exposes AD dynamics through the analytic-Jacobian callback.
#[derive(Clone, Debug, PartialEq)]
pub struct CollocationAutoDiffDynamicsAdapter<D> {
    dynamics: D,
    state_dimension: usize,
    control_dimension: usize,
}

impl<D> CollocationAutoDiffDynamicsAdapter<D> {
    /// Build an AD adapter for fixed state/control dimensions.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when either dimension is zero.
    pub fn new(
        dynamics: D,
        state_dimension: usize,
        control_dimension: usize,
    ) -> Result<Self, TrajoptError> {
        validate_autodiff_dimensions(state_dimension, control_dimension)?;
        Ok(Self {
            dynamics,
            state_dimension,
            control_dimension,
        })
    }

    /// Return the wrapped dynamics object.
    pub fn into_inner(self) -> D {
        self.dynamics
    }
}

impl<D> CollocationDynamics for CollocationAutoDiffDynamicsAdapter<D>
where
    D: CollocationAutoDiffDynamics,
{
    fn eval(
        &mut self,
        time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        validate_autodiff_input_dimensions(
            self.state_dimension,
            self.control_dimension,
            state,
            control,
        )?;
        let dual_state = state
            .iter()
            .map(|value| CollocationAutoDiffScalar::constant(*value))
            .collect::<Vec<_>>();
        let dual_control = control
            .iter()
            .map(|value| CollocationAutoDiffScalar::constant(*value))
            .collect::<Vec<_>>();
        let output = self
            .dynamics
            .eval_dual(time_s, &dual_state, &dual_control)?;
        validate_autodiff_output_dimension(&output, self.state_dimension)?;
        Ok(output.iter().map(|sample| sample.value).collect())
    }
}

impl<D> CollocationDynamicsJacobian for CollocationAutoDiffDynamicsAdapter<D>
where
    D: CollocationAutoDiffDynamics,
{
    fn eval_with_jacobian(
        &mut self,
        time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<CollocationDynamicsJacobianSample, TrajoptError> {
        validate_autodiff_input_dimensions(
            self.state_dimension,
            self.control_dimension,
            state,
            control,
        )?;
        let derivative_count = self.state_dimension + self.control_dimension;
        let dual_state = state
            .iter()
            .enumerate()
            .map(|(index, value)| {
                CollocationAutoDiffScalar::variable(*value, derivative_count, index)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let dual_control = control
            .iter()
            .enumerate()
            .map(|(index, value)| {
                CollocationAutoDiffScalar::variable(
                    *value,
                    derivative_count,
                    self.state_dimension + index,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let output = self
            .dynamics
            .eval_dual(time_s, &dual_state, &dual_control)?;
        validate_autodiff_output_dimension(&output, self.state_dimension)?;

        let mut derivative = Vec::with_capacity(self.state_dimension);
        let mut state_jacobian = vec![0.0; self.state_dimension * self.state_dimension];
        let mut control_jacobian = vec![0.0; self.state_dimension * self.control_dimension];
        for (row, sample) in output.iter().enumerate() {
            derivative.push(sample.value);
            for column in 0..self.state_dimension {
                state_jacobian[row * self.state_dimension + column] =
                    autodiff_derivative_at(sample, derivative_count, column)?;
            }
            for column in 0..self.control_dimension {
                control_jacobian[row * self.control_dimension + column] = autodiff_derivative_at(
                    sample,
                    derivative_count,
                    self.state_dimension + column,
                )?;
            }
        }

        CollocationDynamicsJacobianSample::new(
            derivative,
            state_jacobian,
            control_jacobian,
            self.state_dimension,
            self.control_dimension,
        )
    }
}

impl<F> CollocationDynamics for F
where
    F: for<'state, 'control> FnMut(
        f64,
        &'state [f64],
        &'control [f64],
    ) -> Result<Vec<f64>, TrajoptError>,
{
    fn eval(
        &mut self,
        time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        self(time_s, state, control)
    }
}

/// Lagrange-cost callback used by the Hermite-Simpson transcription.
pub trait CollocationLagrange {
    /// Evaluate the running-cost integrand `L(t, x, u)`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the integrand cannot be evaluated.
    fn eval(&mut self, time_s: f64, state: &[f64], control: &[f64]) -> Result<f64, TrajoptError>;
}

impl<F> CollocationLagrange for F
where
    F: for<'state, 'control> FnMut(
        f64,
        &'state [f64],
        &'control [f64],
    ) -> Result<f64, TrajoptError>,
{
    fn eval(&mut self, time_s: f64, state: &[f64], control: &[f64]) -> Result<f64, TrajoptError> {
        self(time_s, state, control)
    }
}

/// Path-function callback used by nonlinear hard path inequalities.
pub trait CollocationPathFunction {
    /// Evaluate a vector path function `c(t, x, u)`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the path function cannot be evaluated.
    fn eval(
        &mut self,
        time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError>;
}

impl<F> CollocationPathFunction for F
where
    F: for<'state, 'control> FnMut(
        f64,
        &'state [f64],
        &'control [f64],
    ) -> Result<Vec<f64>, TrajoptError>,
{
    fn eval(
        &mut self,
        time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        self(time_s, state, control)
    }
}

/// One Hermite-Simpson segment with endpoint and midpoint decision variables.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonSegment {
    /// Segment start time, s.
    pub t0_s: f64,
    /// Segment midpoint time, s.
    pub tm_s: f64,
    /// Segment end time, s.
    pub t1_s: f64,
    /// Segment start state.
    pub x0: Vec<f64>,
    /// Segment midpoint state.
    pub xm: Vec<f64>,
    /// Segment end state.
    pub x1: Vec<f64>,
    /// Segment start control.
    pub u0: Vec<f64>,
    /// Segment midpoint control.
    pub um: Vec<f64>,
    /// Segment end control.
    pub u1: Vec<f64>,
}

impl HermiteSimpsonSegment {
    /// Build a segment from endpoint and midpoint slices.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when times are invalid, dimensions disagree, or
    /// any state/control entry is non-finite.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        t0_s: f64,
        tm_s: f64,
        t1_s: f64,
        x0: &[f64],
        xm: &[f64],
        x1: &[f64],
        u0: &[f64],
        um: &[f64],
        u1: &[f64],
    ) -> Result<Self, TrajoptError> {
        let segment = Self {
            t0_s,
            tm_s,
            t1_s,
            x0: x0.to_vec(),
            xm: xm.to_vec(),
            x1: x1.to_vec(),
            u0: u0.to_vec(),
            um: um.to_vec(),
            u1: u1.to_vec(),
        };
        validate_segment(&segment)?;
        Ok(segment)
    }

    /// Segment duration, s.
    #[must_use]
    pub fn step_s(&self) -> f64 {
        self.t1_s - self.t0_s
    }

    /// State dimension.
    #[must_use]
    pub fn state_dimension(&self) -> usize {
        self.x0.len()
    }

    /// Control dimension.
    #[must_use]
    pub fn control_dimension(&self) -> usize {
        self.u0.len()
    }
}

/// Defect and running-cost report for one Hermite-Simpson segment.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonSegmentReport {
    /// Segment duration, s.
    pub step_s: f64,
    /// Dynamics defects:
    /// `x1 - x0 - h/6 * (f0 + 4*fm + f1)`.
    pub dynamics_defect: Vec<f64>,
    /// Hermite midpoint defects:
    /// `xm - 0.5*(x0+x1) - h/8*(f0-f1)`.
    pub midpoint_defect: Vec<f64>,
    /// Simpson running-cost contribution for the segment.
    pub simpson_lagrange_cost: f64,
    /// Euclidean norm of dynamics plus midpoint defects.
    pub defect_norm: f64,
    /// Maximum absolute dynamics or midpoint defect.
    pub max_abs_defect: f64,
}

/// Fixed-mesh Hermite-Simpson transcription report.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonTranscriptionReport {
    /// Stacked defects, segment-major with dynamics rows followed by midpoint
    /// rows.
    pub defects: Vec<f64>,
    /// Sum of segment Simpson running-cost contributions.
    pub simpson_lagrange_cost: f64,
    /// Euclidean norm of all dynamics defects.
    pub dynamics_defect_norm: f64,
    /// Euclidean norm of all midpoint defects.
    pub midpoint_defect_norm: f64,
    /// Euclidean norm of all dynamics and midpoint defects.
    pub total_defect_norm: f64,
    /// Maximum absolute dynamics or midpoint defect over all segments.
    pub max_abs_defect: f64,
    /// Number of segments.
    pub segment_count: usize,
    /// State dimension.
    pub state_dimension: usize,
    /// Control dimension.
    pub control_dimension: usize,
}

/// Fixed-state boundary residual applied to a Hermite-Simpson node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HermiteSimpsonBoundaryConstraint {
    /// Node index in `[0, segment_count]`.
    pub node_index: usize,
    /// State component index in `[0, state_dimension)`.
    pub component: usize,
    /// Target state value.
    pub target: f64,
}

impl HermiteSimpsonBoundaryConstraint {
    /// Build a finite node-state boundary constraint.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when `target` is non-finite.
    pub fn new(node_index: usize, component: usize, target: f64) -> Result<Self, TrajoptError> {
        if !target.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson boundary target must be finite",
            });
        }
        Ok(Self {
            node_index,
            component,
            target,
        })
    }
}

/// Fixed-mesh Hermite-Simpson sample point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HermiteSimpsonSamplePoint {
    /// Mesh node index in `[0, segment_count]`.
    Node(usize),
    /// Segment midpoint index in `[0, segment_count)`.
    Midpoint(usize),
}

/// Scalar variable selected from a fixed-mesh Hermite-Simpson sample point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HermiteSimpsonScalarVariable {
    /// State component at the sample point.
    StateComponent {
        /// State component index.
        component: usize,
    },
    /// Control component at the sample point.
    ControlComponent {
        /// Control component index.
        component: usize,
    },
}

/// Soft scalar path bound represented as an exterior-penalty residual.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HermiteSimpsonSoftPathConstraint {
    /// Node or segment-midpoint sample.
    pub sample: HermiteSimpsonSamplePoint,
    /// Scalar state/control component.
    pub variable: HermiteSimpsonScalarVariable,
    /// Optional lower bound.
    pub lower: Option<f64>,
    /// Optional upper bound.
    pub upper: Option<f64>,
    /// Positive residual weight. The reported residual is
    /// `sqrt(weight) * violation`.
    pub weight: f64,
}

impl HermiteSimpsonSoftPathConstraint {
    /// Build a soft path bound.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the bounds or weight are invalid.
    pub fn new(
        sample: HermiteSimpsonSamplePoint,
        variable: HermiteSimpsonScalarVariable,
        lower: Option<f64>,
        upper: Option<f64>,
        weight: f64,
    ) -> Result<Self, TrajoptError> {
        if !weight.is_finite() || weight <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson soft path weight must be finite and positive",
            });
        }
        validate_path_bounds(lower, upper, "Hermite-Simpson soft path")?;
        Ok(Self {
            sample,
            variable,
            lower,
            upper,
            weight,
        })
    }
}

/// Hard scalar path bound transcribed as a linear inequality row.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HermiteSimpsonHardPathConstraint {
    /// Node or segment-midpoint sample.
    pub sample: HermiteSimpsonSamplePoint,
    /// Scalar state/control component.
    pub variable: HermiteSimpsonScalarVariable,
    /// Optional lower bound.
    pub lower: Option<f64>,
    /// Optional upper bound.
    pub upper: Option<f64>,
}

impl HermiteSimpsonHardPathConstraint {
    /// Build a hard path bound.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the bounds are invalid.
    pub fn new(
        sample: HermiteSimpsonSamplePoint,
        variable: HermiteSimpsonScalarVariable,
        lower: Option<f64>,
        upper: Option<f64>,
    ) -> Result<Self, TrajoptError> {
        validate_path_bounds(lower, upper, "Hermite-Simpson hard path")?;
        Ok(Self {
            sample,
            variable,
            lower,
            upper,
        })
    }

    /// Number of dense inequality rows emitted by this bound.
    #[must_use]
    pub fn inequality_count(&self) -> usize {
        usize::from(self.upper.is_some()) + usize::from(self.lower.is_some())
    }
}

/// Nonlinear vector path bound transcribed as hard inequality rows.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonNonlinearHardPathConstraint {
    /// Node or segment-midpoint sample.
    pub sample: HermiteSimpsonSamplePoint,
    /// Optional lower bounds for each path-function component.
    pub lower_bounds: Vec<Option<f64>>,
    /// Optional upper bounds for each path-function component.
    pub upper_bounds: Vec<Option<f64>>,
}

impl HermiteSimpsonNonlinearHardPathConstraint {
    /// Build a nonlinear hard path constraint descriptor.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when dimensions or bounds are invalid.
    pub fn new(
        sample: HermiteSimpsonSamplePoint,
        lower_bounds: Vec<Option<f64>>,
        upper_bounds: Vec<Option<f64>>,
    ) -> Result<Self, TrajoptError> {
        let constraint = Self {
            sample,
            lower_bounds,
            upper_bounds,
        };
        validate_nonlinear_path_bounds(&constraint)?;
        Ok(constraint)
    }

    /// Number of path-function components.
    #[must_use]
    pub fn output_dimension(&self) -> usize {
        self.lower_bounds.len()
    }

    /// Number of scalar inequality rows emitted by this descriptor.
    #[must_use]
    pub fn inequality_count(&self) -> usize {
        self.lower_bounds
            .iter()
            .zip(self.upper_bounds.iter())
            .map(|(lower, upper)| usize::from(upper.is_some()) + usize::from(lower.is_some()))
            .sum()
    }
}

/// State-component mapping used when emitting an optimized reference profile
/// from a fixed-mesh Hermite-Simpson decision vector.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HermiteSimpsonReferenceProfileMapping {
    /// State component containing inertial reference radius, m.
    pub radius_component: usize,
    /// State component containing inertial reference speed, m/s.
    pub speed_component: usize,
    /// State component containing flight-path angle, rad.
    pub flight_path_angle_component: usize,
}

impl HermiteSimpsonReferenceProfileMapping {
    /// Build a reference-profile state-component mapping.
    #[must_use]
    pub const fn new(
        radius_component: usize,
        speed_component: usize,
        flight_path_angle_component: usize,
    ) -> Self {
        Self {
            radius_component,
            speed_component,
            flight_path_angle_component,
        }
    }
}

/// Deterministic optimized reference profile emitted from a fixed mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonReferenceProfileReport {
    /// Reference samples in strictly increasing time order.
    pub samples: Vec<ReferenceProfileSample>,
    /// Number of emitted samples.
    pub sample_count: usize,
    /// First sample time, s.
    pub start_time_s: f64,
    /// Last sample time, s.
    pub end_time_s: f64,
}

/// Simpson-weighted quadratic control objective residual.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HermiteSimpsonQuadraticControlCost {
    /// Control component index.
    pub component: usize,
    /// Positive control-effort weight.
    pub weight: f64,
}

impl HermiteSimpsonQuadraticControlCost {
    /// Build a quadratic control-effort objective term.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when `weight` is not finite and positive.
    pub fn new(component: usize, weight: f64) -> Result<Self, TrajoptError> {
        if !weight.is_finite() || weight <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson control-cost weight must be finite and positive",
            });
        }
        Ok(Self { component, weight })
    }
}

/// Flat decision-vector layout for a fixed-mesh Hermite-Simpson OCP.
///
/// Decision vectors are ordered as:
///
/// ```text
/// [node states, midpoint states, node controls, midpoint controls]
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonDecisionLayout {
    /// Number of Hermite-Simpson segments.
    pub segment_count: usize,
    /// State dimension.
    pub state_dimension: usize,
    /// Control dimension.
    pub control_dimension: usize,
    /// Offset of node states in the flat decision vector.
    pub node_state_offset: usize,
    /// Offset of midpoint states in the flat decision vector.
    pub midpoint_state_offset: usize,
    /// Offset of node controls in the flat decision vector.
    pub node_control_offset: usize,
    /// Offset of midpoint controls in the flat decision vector.
    pub midpoint_control_offset: usize,
    /// Total number of decision variables.
    pub variable_count: usize,
}

impl HermiteSimpsonDecisionLayout {
    /// Build a fixed-mesh decision layout.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when there are no segments or no state
    /// variables.
    pub fn new(
        segment_count: usize,
        state_dimension: usize,
        control_dimension: usize,
    ) -> Result<Self, TrajoptError> {
        if segment_count == 0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson fixed mesh requires at least one segment",
            });
        }
        if state_dimension == 0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson fixed mesh state dimension must be nonzero",
            });
        }
        let node_state_offset = 0;
        let midpoint_state_offset = (segment_count + 1) * state_dimension;
        let node_control_offset = midpoint_state_offset + segment_count * state_dimension;
        let midpoint_control_offset = node_control_offset + (segment_count + 1) * control_dimension;
        let variable_count = midpoint_control_offset + segment_count * control_dimension;
        Ok(Self {
            segment_count,
            state_dimension,
            control_dimension,
            node_state_offset,
            midpoint_state_offset,
            node_control_offset,
            midpoint_control_offset,
            variable_count,
        })
    }

    /// Pack row-major node/midpoint states and controls into this layout.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when slice lengths disagree with the layout or
    /// any value is non-finite.
    pub fn pack_decision_vector(
        &self,
        node_states: &[f64],
        midpoint_states: &[f64],
        node_controls: &[f64],
        midpoint_controls: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        if node_states.len() != (self.segment_count + 1) * self.state_dimension
            || midpoint_states.len() != self.segment_count * self.state_dimension
            || node_controls.len() != (self.segment_count + 1) * self.control_dimension
            || midpoint_controls.len() != self.segment_count * self.control_dimension
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson decision block dimensions are inconsistent",
            });
        }
        validate_finite_slice(node_states)?;
        validate_finite_slice(midpoint_states)?;
        validate_finite_slice(node_controls)?;
        validate_finite_slice(midpoint_controls)?;

        let mut decision = Vec::with_capacity(self.variable_count);
        decision.extend_from_slice(node_states);
        decision.extend_from_slice(midpoint_states);
        decision.extend_from_slice(node_controls);
        decision.extend_from_slice(midpoint_controls);
        Ok(decision)
    }

    /// Node state slice from a flat decision vector.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the vector length or node index is invalid.
    pub fn node_state<'a>(
        &self,
        decision: &'a [f64],
        node_index: usize,
    ) -> Result<&'a [f64], TrajoptError> {
        validate_decision_vector(self, decision)?;
        if node_index > self.segment_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson node index is out of range",
            });
        }
        Ok(slice(
            decision,
            self.node_state_offset + node_index * self.state_dimension,
            self.state_dimension,
        ))
    }

    /// Decode a flat decision vector into Hermite-Simpson segments.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when mesh times, decision dimensions, or segment
    /// contents are invalid.
    pub fn segments_from_decision(
        &self,
        time_nodes_s: &[f64],
        decision: &[f64],
    ) -> Result<Vec<HermiteSimpsonSegment>, TrajoptError> {
        validate_time_nodes(time_nodes_s, self.segment_count)?;
        validate_decision_vector(self, decision)?;
        let mut segments = Vec::with_capacity(self.segment_count);
        for segment_index in 0..self.segment_count {
            let t0_s = time_nodes_s[segment_index];
            let t1_s = time_nodes_s[segment_index + 1];
            let tm_s = 0.5 * (t0_s + t1_s);
            segments.push(HermiteSimpsonSegment::new(
                t0_s,
                tm_s,
                t1_s,
                self.node_state(decision, segment_index)?,
                slice(
                    decision,
                    self.midpoint_state_offset + segment_index * self.state_dimension,
                    self.state_dimension,
                ),
                self.node_state(decision, segment_index + 1)?,
                slice(
                    decision,
                    self.node_control_offset + segment_index * self.control_dimension,
                    self.control_dimension,
                ),
                slice(
                    decision,
                    self.midpoint_control_offset + segment_index * self.control_dimension,
                    self.control_dimension,
                ),
                slice(
                    decision,
                    self.node_control_offset + (segment_index + 1) * self.control_dimension,
                    self.control_dimension,
                ),
            )?);
        }
        Ok(segments)
    }
}

/// Fixed-mesh Hermite-Simpson residual report.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshResidualReport {
    /// Residual vector: collocation defects followed by boundary residuals.
    pub residuals: Vec<f64>,
    /// Number of collocation residual rows.
    pub collocation_residual_count: usize,
    /// Number of boundary residual rows.
    pub boundary_residual_count: usize,
    /// Number of soft path residual rows.
    pub path_residual_count: usize,
    /// Number of objective residual rows.
    pub objective_residual_count: usize,
    /// Euclidean norm of all residual rows.
    pub residual_norm: f64,
    /// Simpson running-cost contribution over the mesh.
    pub simpson_lagrange_cost: f64,
    /// Simpson quadrature of configured quadratic control costs.
    pub quadratic_control_cost: f64,
}

/// Dense row-major inequality transcription for hard path bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonPathInequalityReport {
    /// Number of flat Hermite-Simpson decision variables.
    pub variable_count: usize,
    /// Number of emitted inequality rows.
    pub inequality_count: usize,
    /// Row-major dense `Aineq` such that `Aineq * decision <= inequality_rhs`.
    pub inequality_matrix: Vec<f64>,
    /// Dense inequality right-hand side.
    pub inequality_rhs: Vec<f64>,
}

impl HermiteSimpsonPathInequalityReport {
    /// Evaluate `Aineq * decision`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or decision vector is invalid.
    pub fn lhs(&self, decision: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_path_inequality_report(self)?;
        validate_dense_decision_vector(decision, self.variable_count)?;

        let mut lhs = Vec::with_capacity(self.inequality_count);
        for row in 0..self.inequality_count {
            let offset = row * self.variable_count;
            let value = self.inequality_matrix[offset..offset + self.variable_count]
                .iter()
                .zip(decision.iter())
                .map(|(coefficient, value)| coefficient * value)
                .sum();
            lhs.push(value);
        }
        Ok(lhs)
    }

    /// Evaluate non-negative hard-inequality violations.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or decision vector is invalid.
    pub fn violations(&self, decision: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        let lhs = self.lhs(decision)?;
        Ok(lhs
            .iter()
            .zip(self.inequality_rhs.iter())
            .map(|(lhs, rhs)| (lhs - rhs).max(0.0))
            .collect())
    }

    /// Maximum non-negative hard-inequality violation.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or decision vector is invalid.
    pub fn max_violation(&self, decision: &[f64]) -> Result<f64, TrajoptError> {
        Ok(self
            .violations(decision)?
            .into_iter()
            .fold(0.0_f64, f64::max))
    }
}

/// Nonlinear hard path inequalities evaluated as `g(z) <= 0` rows.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonNonlinearPathInequalityReport {
    /// Number of flat Hermite-Simpson decision variables.
    pub variable_count: usize,
    /// Number of path-function components evaluated before bound splitting.
    pub path_value_count: usize,
    /// Number of emitted scalar inequality rows.
    pub inequality_count: usize,
    /// Raw path-function values, constraint-major and component-major.
    pub path_values: Vec<f64>,
    /// Inequality row values. Feasible rows are `<= 0`.
    pub inequality_values: Vec<f64>,
    /// Maximum non-negative inequality-row violation.
    pub max_violation: f64,
}

impl HermiteSimpsonNonlinearPathInequalityReport {
    /// Evaluate non-negative hard-inequality violations.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report shape is invalid.
    pub fn violations(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_nonlinear_path_inequality_report(self)?;
        Ok(self
            .inequality_values
            .iter()
            .map(|value| value.max(0.0))
            .collect())
    }
}

/// Sparse CSC Jacobian for nonlinear hard path inequalities.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonSparsePathInequalityJacobianReport {
    /// Number of inequality rows.
    pub row_count: usize,
    /// Number of decision-variable columns.
    pub column_count: usize,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
    /// Central finite-difference step used for each column scale.
    pub finite_difference_step: f64,
    /// Absolute coefficient threshold used to drop structural zeros.
    pub drop_tolerance: f64,
}

impl HermiteSimpsonSparsePathInequalityJacobianReport {
    /// Return a single Jacobian coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape or requested index
    /// is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_sparse_path_inequality_jacobian_report(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse path inequality Jacobian index is out of range",
            });
        }
        let start = self.column_offsets[column];
        let end = self.column_offsets[column + 1];
        for index in start..end {
            if self.row_indices[index] == row {
                return Ok(self.values[index]);
            }
        }
        Ok(0.0)
    }

    /// Expand the CSC Jacobian to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_path_inequality_jacobian_report(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }
}

/// Sparse CSC residual Jacobian for a fixed-mesh residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonSparseResidualJacobianReport {
    /// Number of residual rows.
    pub row_count: usize,
    /// Number of decision-variable columns.
    pub column_count: usize,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
    /// Central finite-difference step used for each column scale, or `0.0`
    /// when the report was assembled from analytic derivative callbacks.
    pub finite_difference_step: f64,
    /// Absolute coefficient threshold used to drop structural zeros.
    pub drop_tolerance: f64,
}

/// Sparse CSC Gauss-Newton Hessian / KKT normal block for a fixed-mesh
/// residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonSparseGaussNewtonHessianReport {
    /// Number of Hessian rows.
    pub row_count: usize,
    /// Number of Hessian columns.
    pub column_count: usize,
    /// Number of residual rows in the source Jacobian.
    pub residual_row_count: usize,
    /// Number of nonzeros in the source residual Jacobian.
    pub source_jacobian_nonzero_count: usize,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
    /// Non-negative diagonal regularization added to `J'J`.
    pub regularization: f64,
    /// Absolute coefficient threshold used to drop structural zeros.
    pub drop_tolerance: f64,
}

/// Sparse CSC augmented KKT system for a linearized least-squares step.
///
/// The system is the symmetric quasi-definite saddle matrix
/// `[regularization * I, J'; J, -I]`, where `J` is the sparse residual
/// Jacobian. Solving it with right-hand side `[0, -r]` yields the same
/// primal step as the regularized normal equation
/// `(J'J + regularization I) * step = -J'r` while preserving the KKT
/// block structure needed by larger sparse SQP backends.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonSparseAugmentedKktReport {
    /// Number of KKT rows.
    pub row_count: usize,
    /// Number of KKT columns.
    pub column_count: usize,
    /// Number of primal decision variables.
    pub variable_count: usize,
    /// Number of residual rows in the source Jacobian.
    pub residual_row_count: usize,
    /// Number of nonzeros in the source residual Jacobian.
    pub source_jacobian_nonzero_count: usize,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
    /// Positive diagonal regularization in the primal block.
    pub regularization: f64,
    /// Absolute coefficient threshold used to drop structural zeros.
    pub drop_tolerance: f64,
}

/// Sparse CSC constrained KKT system for an active-hard-inequality
/// linearized least-squares step.
///
/// The system is `[regularization * I, J', A'; J, -I, 0; A, 0, 0]`, where `J`
/// is the residual Jacobian and `A` contains active linearized hard-path rows.
/// Solving it with right-hand side `[0, -r, -g_active]` yields a primal step,
/// residual-space multipliers, and active inequality multipliers.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonSparseConstrainedKktReport {
    /// Number of KKT rows.
    pub row_count: usize,
    /// Number of KKT columns.
    pub column_count: usize,
    /// Number of primal decision variables.
    pub variable_count: usize,
    /// Number of residual rows in the source Jacobian.
    pub residual_row_count: usize,
    /// Number of active hard-inequality rows.
    pub active_inequality_row_count: usize,
    /// Number of nonzeros in the source residual Jacobian.
    pub source_jacobian_nonzero_count: usize,
    /// Number of nonzeros in the active hard-inequality Jacobian.
    pub source_inequality_nonzero_count: usize,
    /// Active hard-path row indices in the source inequality report.
    pub active_inequality_rows: Vec<usize>,
    /// Active inequality values `g(z) = A*z - b` at the linearization point.
    pub active_inequality_values: Vec<f64>,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
    /// Positive diagonal regularization in the primal block.
    pub regularization: f64,
    /// Absolute coefficient threshold used to drop structural zeros.
    pub drop_tolerance: f64,
    /// Active-row threshold used to select rows with `g(z) >= -tolerance`.
    pub active_tolerance: f64,
}

/// Conjugate-gradient solve report for a sparse normal-equation block.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonSparseNormalSolveReport {
    /// Final solution vector.
    pub solution: Vec<f64>,
    /// Final linear residual `A * solution - rhs`.
    pub residual: Vec<f64>,
    /// Euclidean norm of the initial linear residual.
    pub initial_residual_norm: f64,
    /// Euclidean norm of the final linear residual.
    pub residual_norm: f64,
    /// Number of conjugate-gradient iterations executed.
    pub iterations: usize,
    /// Whether `residual_norm <= residual_tolerance`.
    pub converged: bool,
}

/// Deterministic LDL solve report for a sparse augmented KKT matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonSparseKktSolveReport {
    /// Final solution vector.
    pub solution: Vec<f64>,
    /// Final linear residual `A * solution - rhs`.
    pub residual: Vec<f64>,
    /// Euclidean norm of [`Self::residual`].
    pub residual_norm: f64,
    /// Diagonal entries of the no-pivot LDL factorization.
    pub factor_diagonal: Vec<f64>,
    /// Frobenius norm of `L * D * L' - A` for the dense factor workspace.
    pub factorization_residual_norm: f64,
}

/// Per-segment KKT multipliers for Hermite-Simpson collocation residual rows.
///
/// These are raw residual-space multipliers from the augmented KKT least-squares
/// system, split into dynamics-defect and midpoint-defect rows. They are a
/// deterministic substrate for later PMP/costate cross-checks, not a complete
/// covector mapping by themselves.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonKktCollocationMultiplierSample {
    /// Segment index in `[0, segment_count)`.
    pub segment_index: usize,
    /// Multipliers associated with the segment dynamics-defect rows.
    pub dynamics_defect_multipliers: Vec<f64>,
    /// Multipliers associated with the segment midpoint-defect rows.
    pub midpoint_defect_multipliers: Vec<f64>,
}

/// KKT residual multipliers split by fixed-mesh residual row family.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonKktResidualMultiplierReport {
    /// Number of Hermite-Simpson segments in the source OCP.
    pub segment_count: usize,
    /// State dimension in the source OCP.
    pub state_dimension: usize,
    /// Total number of residual multipliers.
    pub residual_multiplier_count: usize,
    /// Number of collocation residual multipliers.
    pub collocation_multiplier_count: usize,
    /// Number of boundary residual multipliers.
    pub boundary_multiplier_count: usize,
    /// Number of soft path residual multipliers.
    pub path_multiplier_count: usize,
    /// Number of objective residual multipliers.
    pub objective_multiplier_count: usize,
    /// Per-segment collocation multiplier slices.
    pub collocation_samples: Vec<HermiteSimpsonKktCollocationMultiplierSample>,
    /// Boundary residual multipliers.
    pub boundary_multipliers: Vec<f64>,
    /// Soft path residual multipliers.
    pub path_multipliers: Vec<f64>,
    /// Objective residual multipliers.
    pub objective_multipliers: Vec<f64>,
    /// Maximum absolute dynamics-defect multiplier.
    pub max_abs_dynamics_multiplier: f64,
    /// Maximum absolute midpoint-defect multiplier.
    pub max_abs_midpoint_multiplier: f64,
    /// Maximum absolute boundary multiplier.
    pub max_abs_boundary_multiplier: f64,
}

/// Node-level raw costate estimate derived from KKT dynamics-defect multipliers.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonKktNodeCostateEstimate {
    /// Mesh node index in `[0, segment_count]`.
    pub node_index: usize,
    /// Mesh node time, s.
    pub time_s: f64,
    /// Raw costate estimate in state-component order.
    ///
    /// The estimate averages adjacent segment dynamics-defect multipliers for
    /// interior nodes and uses the single adjacent segment at endpoints. No
    /// sign flip or mesh scaling is applied here.
    pub costate: Vec<f64>,
    /// Number of adjacent segment dynamics-defect multiplier vectors averaged.
    pub contributing_segment_count: usize,
}

/// Raw node-costate estimate report derived from KKT residual multipliers.
///
/// This is a deterministic mesh diagnostic for later PMP witnesses. It is not a
/// complete Betts-style covector mapping and does not apply final sign/scaling
/// conventions.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonKktCostateEstimateReport {
    /// Number of Hermite-Simpson segments in the source OCP.
    pub segment_count: usize,
    /// State dimension in the source OCP.
    pub state_dimension: usize,
    /// Total number of residual multipliers consumed.
    pub source_residual_multiplier_count: usize,
    /// Node-level raw costate estimates.
    pub node_samples: Vec<HermiteSimpsonKktNodeCostateEstimate>,
    /// Maximum absolute node costate component.
    pub max_abs_costate: f64,
}

impl HermiteSimpsonKktCostateEstimateReport {
    /// Apply an explicit direct-method-to-PMP costate convention.
    ///
    /// The raw Hermite-Simpson node estimates remain unscaled diagnostics. This
    /// adapter only packages them into the common PMP comparison shape after the
    /// caller selects a sign/scale convention.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the raw node report is malformed or the
    /// selected PMP convention is invalid.
    pub fn pmp_costate_convention_report(
        &self,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        if self.state_dimension == 0 || self.node_samples.len() != self.segment_count + 1 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson PMP costate convention report dimensions are inconsistent",
            });
        }
        let mut times_s = Vec::with_capacity(self.node_samples.len());
        let mut raw_costates = Vec::with_capacity(self.node_samples.len() * self.state_dimension);
        for (node_index, sample) in self.node_samples.iter().enumerate() {
            if sample.node_index != node_index
                || sample.costate.len() != self.state_dimension
                || sample.contributing_segment_count == 0
            {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson PMP costate convention sample is inconsistent",
                });
            }
            times_s.push(sample.time_s);
            raw_costates.extend(sample.costate.iter().copied());
        }
        crate::pmp::apply_pmp_direct_costate_convention(
            self.state_dimension,
            &times_s,
            &raw_costates,
            convention,
        )
    }
}

/// Node-level Hamiltonian diagnostic from raw KKT costate estimates.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonKktHamiltonianSample {
    /// Mesh node index in `[0, segment_count]`.
    pub node_index: usize,
    /// Mesh node time, s.
    pub time_s: f64,
    /// Raw node-costate estimate used for this sample.
    pub costate: Vec<f64>,
    /// Dynamics derivative `f(t, x, u)` at this sample.
    pub dynamics: Vec<f64>,
    /// Lagrange integrand `L(t, x, u)` at this sample.
    pub lagrange_value: f64,
    /// Inner product `lambda' * f`.
    pub costate_dot_dynamics: f64,
    /// Raw Hamiltonian diagnostic `L + lambda' * f`.
    pub hamiltonian: f64,
}

/// Raw Hamiltonian diagnostic report for later PMP witnesses.
///
/// This report evaluates `H = L + lambda' f` at mesh nodes using the unscaled
/// node-costate estimates. It exposes deterministic diagnostic evidence only;
/// it is not a PMP constancy proof or final covector convention.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonKktHamiltonianReport {
    /// Number of Hermite-Simpson segments in the source OCP.
    pub segment_count: usize,
    /// State dimension in the source OCP.
    pub state_dimension: usize,
    /// Number of node Hamiltonian samples.
    pub sample_count: usize,
    /// Node-level Hamiltonian diagnostic samples.
    pub samples: Vec<HermiteSimpsonKktHamiltonianSample>,
    /// Maximum absolute Hamiltonian value.
    pub max_abs_hamiltonian: f64,
    /// Maximum absolute Hamiltonian drift relative to the first sample.
    pub max_abs_hamiltonian_drift: f64,
}

/// Sparse Gauss-Newton step report for a fixed-mesh residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshSparseGaussNewtonStepReport {
    /// Residual report evaluated at the linearization point.
    pub residual_report: HermiteSimpsonFixedMeshResidualReport,
    /// Sparse residual Jacobian at the linearization point.
    pub sparse_jacobian: HermiteSimpsonSparseResidualJacobianReport,
    /// Sparse normal-equation block `J'J + regularization I`.
    pub normal_hessian: HermiteSimpsonSparseGaussNewtonHessianReport,
    /// Sparse normal-equation right-hand side `-J'r`.
    pub rhs: Vec<f64>,
    /// Conjugate-gradient solve report for the sparse normal equation.
    pub solve_report: HermiteSimpsonSparseNormalSolveReport,
    /// Linearized residual `r + J * step`.
    pub linearized_residual: Vec<f64>,
    /// Euclidean norm of [`Self::linearized_residual`].
    pub linearized_residual_norm: f64,
    /// Euclidean norm of the solved step.
    pub step_norm: f64,
}

/// Sparse augmented-KKT step report for a fixed-mesh residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshSparseKktStepReport {
    /// Residual report evaluated at the linearization point.
    pub residual_report: HermiteSimpsonFixedMeshResidualReport,
    /// Sparse residual Jacobian at the linearization point.
    pub sparse_jacobian: HermiteSimpsonSparseResidualJacobianReport,
    /// Sparse augmented KKT matrix `[regularization * I, J'; J, -I]`.
    pub augmented_kkt: HermiteSimpsonSparseAugmentedKktReport,
    /// Augmented KKT right-hand side `[0, -r]`.
    pub rhs: Vec<f64>,
    /// LDL solve report for the augmented KKT system.
    pub solve_report: HermiteSimpsonSparseKktSolveReport,
    /// Primal decision step.
    pub step: Vec<f64>,
    /// Residual-space KKT variables, equal to the linearized residual.
    pub residual_multipliers: Vec<f64>,
    /// Linearized residual `r + J * step`.
    pub linearized_residual: Vec<f64>,
    /// Euclidean norm of [`Self::linearized_residual`].
    pub linearized_residual_norm: f64,
    /// Euclidean norm of the primal step.
    pub step_norm: f64,
}

/// Sparse constrained-KKT step report for active scalar hard-path rows.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshSparseConstrainedKktStepReport {
    /// Residual report evaluated at the linearization point.
    pub residual_report: HermiteSimpsonFixedMeshResidualReport,
    /// Sparse residual Jacobian at the linearization point.
    pub sparse_jacobian: HermiteSimpsonSparseResidualJacobianReport,
    /// Hard scalar path inequality rows evaluated at the linearization point.
    pub hard_path_inequality_report: HermiteSimpsonPathInequalityReport,
    /// Sparse constrained KKT matrix with active inequality rows.
    pub constrained_kkt: HermiteSimpsonSparseConstrainedKktReport,
    /// Constrained KKT right-hand side `[0, -r, -g_active]`.
    pub rhs: Vec<f64>,
    /// LDL solve report for the constrained KKT system.
    pub solve_report: HermiteSimpsonSparseKktSolveReport,
    /// Primal decision step.
    pub step: Vec<f64>,
    /// Residual-space KKT variables, equal to the linearized residual.
    pub residual_multipliers: Vec<f64>,
    /// Active hard-inequality multipliers in active-row order.
    pub active_inequality_multipliers: Vec<f64>,
    /// Linearized residual `r + J * step`.
    pub linearized_residual: Vec<f64>,
    /// Euclidean norm of [`Self::linearized_residual`].
    pub linearized_residual_norm: f64,
    /// Linearized active hard-inequality values `g_active + A_active * step`.
    pub linearized_active_inequality_values: Vec<f64>,
    /// Maximum non-negative linearized active-inequality violation.
    pub max_linearized_active_inequality_violation: f64,
    /// Euclidean norm of the primal step.
    pub step_norm: f64,
}

/// Settings for sparse augmented-KKT SQP outer loops.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SparseKktSqpSettings {
    /// Maximum accepted SQP iterations.
    pub max_iterations: usize,
    /// Residual norm required for convergence.
    pub residual_tolerance: f64,
    /// Minimum step norm before the loop reports non-convergence.
    pub step_tolerance: f64,
    /// Central finite-difference step for sparse residual Jacobians.
    pub finite_difference_step: f64,
    /// Drop tolerance for sparse residual Jacobian coefficients.
    pub jacobian_drop_tolerance: f64,
    /// Positive primal regularization in the augmented KKT block.
    pub kkt_regularization: f64,
    /// Drop tolerance for sparse augmented-KKT coefficients.
    pub kkt_drop_tolerance: f64,
    /// Maximum line-search backtracks for an accepted KKT step.
    pub max_line_search_backtracks: usize,
    /// Smallest accepted line-search scale.
    pub min_line_search_scale: f64,
}

impl Default for SparseKktSqpSettings {
    fn default() -> Self {
        Self {
            max_iterations: 16,
            residual_tolerance: 1.0e-9,
            step_tolerance: 1.0e-12,
            finite_difference_step: 1.0e-6,
            jacobian_drop_tolerance: 1.0e-12,
            kkt_regularization: 2.5e-1,
            kkt_drop_tolerance: 1.0e-12,
            max_line_search_backtracks: 12,
            min_line_search_scale: 1.0e-8,
        }
    }
}

impl SparseKktSqpSettings {
    pub(crate) fn validate(&self) -> Result<(), TrajoptError> {
        if self.max_iterations == 0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse KKT SQP requires at least one iteration",
            });
        }
        if !self.residual_tolerance.is_finite() || self.residual_tolerance <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse KKT SQP residual tolerance must be finite and positive",
            });
        }
        if !self.step_tolerance.is_finite() || self.step_tolerance < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse KKT SQP step tolerance must be finite and non-negative",
            });
        }
        if !self.finite_difference_step.is_finite() || self.finite_difference_step <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse KKT SQP finite-difference step must be finite and positive",
            });
        }
        if !self.jacobian_drop_tolerance.is_finite() || self.jacobian_drop_tolerance < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse KKT SQP Jacobian drop tolerance must be finite and non-negative",
            });
        }
        if !self.kkt_regularization.is_finite() || self.kkt_regularization <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse KKT SQP regularization must be finite and positive",
            });
        }
        if !self.kkt_drop_tolerance.is_finite() || self.kkt_drop_tolerance < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse KKT SQP KKT drop tolerance must be finite and non-negative",
            });
        }
        if self.max_line_search_backtracks == 0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse KKT SQP line search requires at least one backtrack",
            });
        }
        if !self.min_line_search_scale.is_finite()
            || self.min_line_search_scale <= 0.0
            || self.min_line_search_scale > 1.0
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse KKT SQP minimum line-search scale must be in (0, 1]",
            });
        }
        Ok(())
    }
}

/// One accepted sparse augmented-KKT SQP iteration.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseKktSqpIterationReport {
    /// Zero-based accepted iteration index.
    pub iteration: usize,
    /// Residual norm at the linearization point.
    pub residual_norm: f64,
    /// Linearized residual norm predicted by the sparse KKT step.
    pub linearized_residual_norm: f64,
    /// Full sparse KKT step norm before line search scaling.
    pub step_norm: f64,
    /// Accepted line-search scale applied to the step.
    pub accepted_step_scale: f64,
    /// Residual norm after applying the accepted scaled step.
    pub accepted_residual_norm: f64,
}

/// Sparse augmented-KKT SQP solve report for a fixed-mesh residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshSparseKktSqpReport {
    /// Accepted final decision vector.
    pub decision: Vec<f64>,
    /// Final residual report evaluated at [`Self::decision`].
    pub residual_report: HermiteSimpsonFixedMeshResidualReport,
    /// Final sparse augmented-KKT report assembled at [`Self::decision`].
    pub final_kkt_step_report: HermiteSimpsonFixedMeshSparseKktStepReport,
    /// Final sparse constrained-KKT report when a scalar hard-path row is active.
    pub final_constrained_kkt_step_report:
        Option<HermiteSimpsonFixedMeshSparseConstrainedKktStepReport>,
    /// Decision bounds after merging scalar hard-path constraints.
    pub decision_bounds: HermiteSimpsonDecisionBoundsReport,
    /// Maximum final violation of [`Self::decision_bounds`].
    pub max_decision_bound_violation: f64,
    /// Maximum final violation of scalar hard-path inequalities.
    pub max_hard_path_violation: f64,
    /// Final nonlinear hard-path inequality report, when evaluated by a sparse
    /// SQP entry point with a path-function callback.
    pub nonlinear_hard_path_report: Option<HermiteSimpsonNonlinearPathInequalityReport>,
    /// Maximum final violation of nonlinear hard-path inequalities.
    pub max_nonlinear_hard_path_violation: f64,
    /// Accepted iteration diagnostics.
    pub iterations: Vec<SparseKktSqpIterationReport>,
    /// True when the final residual norm satisfies the requested tolerance.
    pub converged: bool,
}

/// PMP/KKT certificate artifacts derived from one active constrained-KKT step.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshSparseConstrainedKktPmpKktCertificateReport {
    /// Sparse constrained-KKT report consumed by the PMP witness.
    pub constrained_kkt_step_report: HermiteSimpsonFixedMeshSparseConstrainedKktStepReport,
    /// Raw node costates mapped through the caller-selected PMP convention.
    pub costate_convention_report: crate::pmp::PmpDirectCostateConventionReport,
    /// PMP Hamiltonian profile evaluated from those convention-mapped costates.
    pub hamiltonian_report: crate::pmp::PmpHamiltonianReport,
    /// Optional Hamiltonian control-stationarity report attached to the certificate.
    pub control_stationarity_report: Option<crate::pmp::PmpHamiltonianControlStationarityReport>,
    /// Optional same-mesh PMP-vs-KKT costate comparison attached to the certificate.
    pub costate_comparison_report: Option<crate::pmp::PmpCostateComparisonReport>,
    /// Validated I-load optimality certificate for the Hamiltonian profile plus
    /// optional control-stationarity and costate-comparison summaries.
    pub optimality_certificate: ILoadOptimalityCertificate,
}

impl HermiteSimpsonFixedMeshSparseConstrainedKktPmpKktCertificateReport {
    /// Attach a validated same-mesh PMP-vs-KKT costate comparison.
    ///
    /// `pmp_times_s` and `pmp_costates` are the caller-provided PMP/oracle
    /// profile in the same PMP convention selected when this certificate report
    /// was built.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the supplied PMP profile is malformed, does
    /// not match this report's mesh/dimensions, or exceeds the declared
    /// comparison tolerance.
    pub fn with_pmp_costate_comparison(
        mut self,
        pmp_times_s: &[f64],
        pmp_costates: &[f64],
        settings: crate::pmp::PmpCostateComparisonSettings,
    ) -> Result<Self, TrajoptError> {
        attach_pmp_costate_comparison_to_certificate(
            &self.costate_convention_report,
            pmp_times_s,
            pmp_costates,
            settings,
            &mut self.costate_comparison_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }

    /// Attach a validated Hamiltonian control-stationarity summary.
    ///
    /// This evaluates `dH/du` from the certificate's solved PMP Hamiltonian
    /// profile and records the resulting tolerance-checked summary in the I-load
    /// optimality certificate.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the stationarity settings, callbacks, or
    /// resulting gradient summary are malformed or outside tolerance.
    pub fn with_pmp_control_stationarity<D, L>(
        mut self,
        settings: crate::pmp::PmpHamiltonianControlStationaritySettings,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<Self, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        attach_pmp_control_stationarity_to_certificate(
            &self.hamiltonian_report,
            settings,
            dynamics,
            lagrange,
            &mut self.control_stationarity_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }
}

/// PMP/KKT certificate artifacts derived from an accepted sparse-KKT SQP report.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshSparseKktSqpPmpKktCertificateReport {
    /// Final sparse augmented-KKT report from the accepted sparse SQP decision.
    pub sparse_kkt_step_report: HermiteSimpsonFixedMeshSparseKktStepReport,
    /// Final constrained sparse-KKT report, when a scalar hard-path row is active.
    pub final_constrained_kkt_step_report:
        Option<HermiteSimpsonFixedMeshSparseConstrainedKktStepReport>,
    /// True when the PMP witness used [`Self::final_constrained_kkt_step_report`].
    pub used_constrained_kkt_step_report: bool,
    /// Raw node costates mapped through the caller-selected PMP convention.
    pub costate_convention_report: crate::pmp::PmpDirectCostateConventionReport,
    /// PMP Hamiltonian profile evaluated from those convention-mapped costates.
    pub hamiltonian_report: crate::pmp::PmpHamiltonianReport,
    /// Optional Hamiltonian control-stationarity report attached to the certificate.
    pub control_stationarity_report: Option<crate::pmp::PmpHamiltonianControlStationarityReport>,
    /// Optional same-mesh PMP-vs-KKT costate comparison attached to the certificate.
    pub costate_comparison_report: Option<crate::pmp::PmpCostateComparisonReport>,
    /// Validated I-load optimality certificate for the Hamiltonian profile plus
    /// optional control-stationarity and costate-comparison summaries.
    pub optimality_certificate: ILoadOptimalityCertificate,
}

impl HermiteSimpsonFixedMeshSparseKktSqpPmpKktCertificateReport {
    /// Attach a validated same-mesh PMP-vs-KKT costate comparison.
    ///
    /// `pmp_times_s` and `pmp_costates` are the caller-provided PMP/oracle
    /// profile in the same PMP convention selected when this certificate report
    /// was built.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the supplied PMP profile is malformed, does
    /// not match this report's mesh/dimensions, or exceeds the declared
    /// comparison tolerance.
    pub fn with_pmp_costate_comparison(
        mut self,
        pmp_times_s: &[f64],
        pmp_costates: &[f64],
        settings: crate::pmp::PmpCostateComparisonSettings,
    ) -> Result<Self, TrajoptError> {
        attach_pmp_costate_comparison_to_certificate(
            &self.costate_convention_report,
            pmp_times_s,
            pmp_costates,
            settings,
            &mut self.costate_comparison_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }

    /// Attach a validated Hamiltonian control-stationarity summary.
    ///
    /// This evaluates `dH/du` from the certificate's solved PMP Hamiltonian
    /// profile and records the resulting tolerance-checked summary in the I-load
    /// optimality certificate.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the stationarity settings, callbacks, or
    /// resulting gradient summary are malformed or outside tolerance.
    pub fn with_pmp_control_stationarity<D, L>(
        mut self,
        settings: crate::pmp::PmpHamiltonianControlStationaritySettings,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<Self, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        attach_pmp_control_stationarity_to_certificate(
            &self.hamiltonian_report,
            settings,
            dynamics,
            lagrange,
            &mut self.control_stationarity_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }
}

fn attach_pmp_costate_comparison_to_certificate(
    convention_report: &crate::pmp::PmpDirectCostateConventionReport,
    pmp_times_s: &[f64],
    pmp_costates: &[f64],
    settings: crate::pmp::PmpCostateComparisonSettings,
    costate_comparison_report: &mut Option<crate::pmp::PmpCostateComparisonReport>,
    optimality_certificate: &mut ILoadOptimalityCertificate,
) -> Result<(), TrajoptError> {
    let comparison = crate::pmp::compare_pmp_costate_profiles(
        convention_report.state_dimension,
        pmp_times_s,
        pmp_costates,
        &convention_report.times_s,
        &convention_report.pmp_costates,
        settings,
    )?;
    let certificate = optimality_certificate
        .clone()
        .with_costate_comparison(&comparison)?;
    *costate_comparison_report = Some(comparison);
    *optimality_certificate = certificate;
    Ok(())
}

fn attach_pmp_control_stationarity_to_certificate<D, L>(
    hamiltonian_report: &crate::pmp::PmpHamiltonianReport,
    settings: crate::pmp::PmpHamiltonianControlStationaritySettings,
    dynamics: &mut D,
    lagrange: &mut L,
    control_stationarity_report: &mut Option<crate::pmp::PmpHamiltonianControlStationarityReport>,
    optimality_certificate: &mut ILoadOptimalityCertificate,
) -> Result<(), TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    let stationarity = crate::pmp::evaluate_pmp_hamiltonian_control_stationarity_from_report(
        hamiltonian_report,
        settings,
        |time_s, state, control| dynamics.eval(time_s, state, control),
        |time_s, state, control| lagrange.eval(time_s, state, control),
    )?;
    let certificate = optimality_certificate
        .clone()
        .with_control_stationarity(&stationarity)?;
    *control_stationarity_report = Some(stationarity);
    *optimality_certificate = certificate;
    Ok(())
}

/// Colored finite-difference sparse residual Jacobian report.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonColoredSparseResidualJacobianReport {
    /// Sparse residual Jacobian evaluated with colored finite differences.
    pub sparse_jacobian: HermiteSimpsonSparseResidualJacobianReport,
    /// Greedy deterministic color for each decision-variable column.
    pub column_colors: Vec<usize>,
    /// Number of column colors used.
    pub color_count: usize,
    /// Number of residual evaluations used: one base evaluation plus two per
    /// color.
    pub residual_evaluation_count: usize,
    /// Conservative structural nonzero count before numeric drop tolerance.
    pub structural_nonzero_count: usize,
}

impl HermiteSimpsonColoredSparseResidualJacobianReport {
    /// Expand the CSC Jacobian to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_colored_sparse_residual_jacobian_report(self)?;
        self.sparse_jacobian.to_dense_row_major()
    }
}

impl HermiteSimpsonSparseResidualJacobianReport {
    /// Return a single Jacobian coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape or requested index
    /// is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_sparse_residual_jacobian_report(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse residual Jacobian index is out of range",
            });
        }
        let start = self.column_offsets[column];
        let end = self.column_offsets[column + 1];
        for index in start..end {
            if self.row_indices[index] == row {
                return Ok(self.values[index]);
            }
        }
        Ok(0.0)
    }

    /// Expand the CSC Jacobian to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_residual_jacobian_report(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }

    /// Multiply this CSC sparse Jacobian by `vector`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report is malformed, the vector
    /// length does not match the Jacobian column count, or any vector value is
    /// non-finite.
    pub fn multiply_vector(&self, vector: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_residual_jacobian_report(self)?;
        if vector.len() != self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse residual Jacobian vector length is inconsistent",
            });
        }
        validate_finite_slice(vector)?;

        let mut product = vec![0.0; self.row_count];
        for (column, vector_value) in vector.iter().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                product[self.row_indices[index]] += self.values[index] * *vector_value;
            }
        }
        Ok(product)
    }

    /// Assemble the sparse Gauss-Newton step right-hand side `-J' * r`.
    ///
    /// This is the vector paired with
    /// [`Self::sparse_gauss_newton_hessian_report`] for normal-equation
    /// least-squares steps.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the Jacobian report is malformed, the
    /// residual length does not match the Jacobian row count, or any numeric
    /// value is non-finite.
    pub fn sparse_gauss_newton_step_rhs(
        &self,
        residuals: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_residual_jacobian_report(self)?;
        if residuals.len() != self.row_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse Gauss-Newton RHS residual length is inconsistent",
            });
        }
        validate_finite_slice(residuals)?;

        let mut rhs = vec![0.0; self.column_count];
        for (column, rhs_column) in rhs.iter_mut().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                *rhs_column -= self.values[index] * residuals[self.row_indices[index]];
            }
        }
        Ok(rhs)
    }

    /// Assemble the sparse Gauss-Newton Hessian block `J'J + regularization I`.
    ///
    /// The output is the symmetric normal-equation block used by sparse KKT
    /// least-squares SQP solvers. This routine keeps the assembly in sparse
    /// row/CSC form; it does not factor or solve the KKT system.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the source sparse Jacobian is malformed or
    /// the assembly settings are invalid.
    pub fn sparse_gauss_newton_hessian_report(
        &self,
        regularization: f64,
        drop_tolerance: f64,
    ) -> Result<HermiteSimpsonSparseGaussNewtonHessianReport, TrajoptError> {
        validate_sparse_residual_jacobian_report(self)?;
        validate_sparse_hessian_settings(regularization, drop_tolerance)?;

        let mut residual_rows = vec![Vec::new(); self.row_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                residual_rows[self.row_indices[index]].push((column, self.values[index]));
            }
        }

        let mut coefficients = BTreeMap::<(usize, usize), f64>::new();
        for row_entries in &residual_rows {
            for (hessian_row, row_value) in row_entries {
                for (hessian_column, column_value) in row_entries {
                    *coefficients
                        .entry((*hessian_column, *hessian_row))
                        .or_insert(0.0) += row_value * column_value;
                }
            }
        }
        if regularization > 0.0 {
            for diagonal in 0..self.column_count {
                *coefficients.entry((diagonal, diagonal)).or_insert(0.0) += regularization;
            }
        }

        let mut column_offsets = Vec::with_capacity(self.column_count + 1);
        let mut row_indices = Vec::new();
        let mut values = Vec::new();
        let mut current_column = 0;
        column_offsets.push(0);
        for ((column, row), value) in coefficients {
            while current_column < column {
                column_offsets.push(values.len());
                current_column += 1;
            }
            if value.abs() > drop_tolerance {
                row_indices.push(row);
                values.push(value);
            }
        }
        while current_column < self.column_count {
            column_offsets.push(values.len());
            current_column += 1;
        }

        let report = HermiteSimpsonSparseGaussNewtonHessianReport {
            row_count: self.column_count,
            column_count: self.column_count,
            residual_row_count: self.row_count,
            source_jacobian_nonzero_count: self.values.len(),
            column_offsets,
            row_indices,
            values,
            regularization,
            drop_tolerance,
        };
        validate_sparse_gauss_newton_hessian_report(&report)?;
        Ok(report)
    }

    /// Assemble the sparse augmented KKT matrix `[regularization * I, J'; J, -I]`.
    ///
    /// This is the quasi-definite saddle system paired with
    /// [`Self::sparse_gauss_newton_step_rhs`]. It exposes the KKT block structure
    /// directly instead of forming only `J'J`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the source sparse Jacobian is malformed or
    /// the assembly settings are invalid.
    pub fn sparse_augmented_kkt_report(
        &self,
        regularization: f64,
        drop_tolerance: f64,
    ) -> Result<HermiteSimpsonSparseAugmentedKktReport, TrajoptError> {
        validate_sparse_residual_jacobian_report(self)?;
        validate_sparse_augmented_kkt_settings(regularization, drop_tolerance)?;

        let variable_count = self.column_count;
        let residual_row_count = self.row_count;
        let dimension = variable_count + residual_row_count;
        let mut coefficients = BTreeMap::<(usize, usize), f64>::new();

        for variable in 0..variable_count {
            coefficients.insert((variable, variable), regularization);
        }
        for residual in 0..residual_row_count {
            let index = variable_count + residual;
            coefficients.insert((index, index), -1.0);
        }
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let residual_row = self.row_indices[index];
                let value = self.values[index];
                let kkt_residual_index = variable_count + residual_row;
                *coefficients
                    .entry((column, kkt_residual_index))
                    .or_insert(0.0) += value;
                *coefficients
                    .entry((kkt_residual_index, column))
                    .or_insert(0.0) += value;
            }
        }

        let mut column_offsets = Vec::with_capacity(dimension + 1);
        let mut row_indices = Vec::new();
        let mut values = Vec::new();
        let mut current_column = 0;
        column_offsets.push(0);
        for ((column, row), value) in coefficients {
            while current_column < column {
                column_offsets.push(values.len());
                current_column += 1;
            }
            if value.abs() > drop_tolerance {
                row_indices.push(row);
                values.push(value);
            }
        }
        while current_column < dimension {
            column_offsets.push(values.len());
            current_column += 1;
        }

        let report = HermiteSimpsonSparseAugmentedKktReport {
            row_count: dimension,
            column_count: dimension,
            variable_count,
            residual_row_count,
            source_jacobian_nonzero_count: self.values.len(),
            column_offsets,
            row_indices,
            values,
            regularization,
            drop_tolerance,
        };
        validate_sparse_augmented_kkt_report(&report)?;
        Ok(report)
    }
}

impl HermiteSimpsonSparseGaussNewtonHessianReport {
    /// Return a single Hessian coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape or requested index
    /// is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_sparse_gauss_newton_hessian_report(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse Gauss-Newton Hessian index is out of range",
            });
        }
        let start = self.column_offsets[column];
        let end = self.column_offsets[column + 1];
        for index in start..end {
            if self.row_indices[index] == row {
                return Ok(self.values[index]);
            }
        }
        Ok(0.0)
    }

    /// Expand the CSC Hessian to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_gauss_newton_hessian_report(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }

    /// Multiply this CSC sparse normal-equation block by `vector`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report is malformed, the vector
    /// length does not match the Hessian dimension, or any vector value is
    /// non-finite.
    pub fn multiply_vector(&self, vector: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_gauss_newton_hessian_report(self)?;
        if vector.len() != self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse Gauss-Newton Hessian vector length is inconsistent",
            });
        }
        validate_finite_slice(vector)?;

        let mut product = vec![0.0; self.row_count];
        for (column, vector_value) in vector.iter().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                product[self.row_indices[index]] += self.values[index] * *vector_value;
            }
        }
        Ok(product)
    }

    /// Solve `A * x = rhs` with deterministic conjugate gradients.
    ///
    /// The matrix `A` is this sparse Gauss-Newton normal block. This is a
    /// lightweight large-sparse exploitation path; it is not a sparse KKT
    /// factorization and it assumes the normal block is positive definite for
    /// the supplied problem.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when dimensions/settings are invalid or the CG
    /// iteration detects non-positive curvature.
    pub fn solve_conjugate_gradient(
        &self,
        rhs: &[f64],
        initial_solution: &[f64],
        max_iterations: usize,
        residual_tolerance: f64,
    ) -> Result<HermiteSimpsonSparseNormalSolveReport, TrajoptError> {
        validate_sparse_cg_settings(
            self,
            rhs,
            initial_solution,
            max_iterations,
            residual_tolerance,
        )?;

        let mut solution = initial_solution.to_vec();
        let mut residual = rhs.to_vec();
        let product = self.multiply_vector(&solution)?;
        for index in 0..residual.len() {
            residual[index] -= product[index];
        }
        let initial_residual_norm = residual_norm(&residual);
        if initial_residual_norm <= residual_tolerance {
            return Ok(HermiteSimpsonSparseNormalSolveReport {
                solution,
                residual: residual.iter().map(|value| -value).collect(),
                initial_residual_norm,
                residual_norm: initial_residual_norm,
                iterations: 0,
                converged: true,
            });
        }

        let mut direction = residual.clone();
        let mut residual_dot = dot(&residual, &residual);
        let mut iterations = 0;
        let mut converged = false;

        for iteration in 0..max_iterations {
            let product = self.multiply_vector(&direction)?;
            let curvature = dot(&direction, &product);
            if !curvature.is_finite() || curvature <= 0.0 {
                return Err(TrajoptError::SingularSystem {
                    reason: "Hermite-Simpson sparse Gauss-Newton CG detected non-positive curvature",
                });
            }
            let alpha = residual_dot / curvature;
            for index in 0..solution.len() {
                solution[index] += alpha * direction[index];
                residual[index] -= alpha * product[index];
            }
            iterations = iteration + 1;
            let new_residual_dot = dot(&residual, &residual);
            if !new_residual_dot.is_finite() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson sparse Gauss-Newton CG residual is non-finite",
                });
            }
            if new_residual_dot.sqrt() <= residual_tolerance {
                converged = true;
                break;
            }
            let beta = new_residual_dot / residual_dot;
            for index in 0..direction.len() {
                direction[index] = residual[index] + beta * direction[index];
            }
            residual_dot = new_residual_dot;
        }

        let final_residual = self.multiply_vector(&solution)?;
        let final_residual = final_residual
            .iter()
            .zip(rhs.iter())
            .map(|(lhs, rhs_value)| lhs - rhs_value)
            .collect::<Vec<_>>();
        let residual_norm = residual_norm(&final_residual);

        Ok(HermiteSimpsonSparseNormalSolveReport {
            solution,
            residual: final_residual,
            initial_residual_norm,
            residual_norm,
            iterations,
            converged: converged || residual_norm <= residual_tolerance,
        })
    }
}

impl HermiteSimpsonSparseAugmentedKktReport {
    /// Return a single augmented KKT coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape or requested index
    /// is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_sparse_augmented_kkt_report(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse augmented KKT index is out of range",
            });
        }
        let start = self.column_offsets[column];
        let end = self.column_offsets[column + 1];
        for index in start..end {
            if self.row_indices[index] == row {
                return Ok(self.values[index]);
            }
        }
        Ok(0.0)
    }

    /// Expand the CSC augmented KKT matrix to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_augmented_kkt_report(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }

    /// Multiply this CSC sparse augmented KKT matrix by `vector`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report is malformed, the vector
    /// length does not match the KKT dimension, or any vector value is
    /// non-finite.
    pub fn multiply_vector(&self, vector: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_augmented_kkt_report(self)?;
        if vector.len() != self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse augmented KKT vector length is inconsistent",
            });
        }
        validate_finite_slice(vector)?;

        let mut product = vec![0.0; self.row_count];
        for (column, vector_value) in vector.iter().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                product[self.row_indices[index]] += self.values[index] * *vector_value;
            }
        }
        Ok(product)
    }

    /// Solve the augmented KKT system with a deterministic no-pivot LDL factorization.
    ///
    /// The matrix is quasi-definite when `regularization > 0`, so the no-pivot
    /// factorization is deterministic for the supported least-squares KKT
    /// system. This is the first direct KKT factorization path; it is still a
    /// fixed-mesh substrate, not a full large-sparse NLP backend.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or right-hand side is malformed,
    /// or when the LDL factorization detects a singular pivot.
    pub fn solve_ldlt(
        &self,
        rhs: &[f64],
    ) -> Result<HermiteSimpsonSparseKktSolveReport, TrajoptError> {
        validate_sparse_augmented_kkt_report(self)?;
        if rhs.len() != self.row_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse augmented KKT RHS length is inconsistent",
            });
        }
        validate_finite_slice(rhs)?;

        let dense = self.to_dense_row_major()?;
        let factorization = dense_ldlt_factorization(&dense, self.row_count)?;
        let solution = factorization.solve(rhs)?;
        let product = self.multiply_vector(&solution)?;
        let residual = product
            .iter()
            .zip(rhs.iter())
            .map(|(lhs, rhs_value)| lhs - rhs_value)
            .collect::<Vec<_>>();
        let residual_norm = residual_norm(&residual);

        Ok(HermiteSimpsonSparseKktSolveReport {
            solution,
            residual,
            residual_norm,
            factor_diagonal: factorization.diagonal,
            factorization_residual_norm: factorization.factorization_residual_norm,
        })
    }
}

impl HermiteSimpsonSparseConstrainedKktReport {
    /// Return a single constrained KKT coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape or requested index
    /// is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_sparse_constrained_kkt_report(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse constrained KKT index is out of range",
            });
        }
        let start = self.column_offsets[column];
        let end = self.column_offsets[column + 1];
        for index in start..end {
            if self.row_indices[index] == row {
                return Ok(self.values[index]);
            }
        }
        Ok(0.0)
    }

    /// Expand the CSC constrained KKT matrix to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_constrained_kkt_report(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }

    /// Multiply this CSC sparse constrained KKT matrix by `vector`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report is malformed, the vector
    /// length does not match the KKT dimension, or any vector value is
    /// non-finite.
    pub fn multiply_vector(&self, vector: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_constrained_kkt_report(self)?;
        if vector.len() != self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse constrained KKT vector length is inconsistent",
            });
        }
        validate_finite_slice(vector)?;

        let mut product = vec![0.0; self.row_count];
        for (column, vector_value) in vector.iter().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                product[self.row_indices[index]] += self.values[index] * *vector_value;
            }
        }
        Ok(product)
    }

    /// Solve the constrained KKT system with deterministic no-pivot LDL.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or right-hand side is malformed,
    /// or when the LDL factorization detects a singular pivot.
    pub fn solve_ldlt(
        &self,
        rhs: &[f64],
    ) -> Result<HermiteSimpsonSparseKktSolveReport, TrajoptError> {
        validate_sparse_constrained_kkt_report(self)?;
        if rhs.len() != self.row_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse constrained KKT RHS length is inconsistent",
            });
        }
        validate_finite_slice(rhs)?;

        let dense = self.to_dense_row_major()?;
        let factorization = dense_ldlt_factorization(&dense, self.row_count)?;
        let solution = factorization.solve(rhs)?;
        let product = self.multiply_vector(&solution)?;
        let residual = product
            .iter()
            .zip(rhs.iter())
            .map(|(lhs, rhs_value)| lhs - rhs_value)
            .collect::<Vec<_>>();
        let residual_norm = residual_norm(&residual);

        Ok(HermiteSimpsonSparseKktSolveReport {
            solution,
            residual,
            residual_norm,
            factor_diagonal: factorization.diagonal,
            factorization_residual_norm: factorization.factorization_residual_norm,
        })
    }
}

/// Dense decision bounds after applying hard scalar path constraints.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonDecisionBoundsReport {
    /// Number of flat Hermite-Simpson decision variables.
    pub variable_count: usize,
    /// Lower decision bounds.
    pub lower_bounds: Vec<f64>,
    /// Upper decision bounds.
    pub upper_bounds: Vec<f64>,
    /// Number of lower/upper hard-path bounds applied.
    pub applied_bound_count: usize,
    /// Number of distinct decision variables tightened by hard-path bounds.
    pub constrained_variable_count: usize,
}

impl HermiteSimpsonDecisionBoundsReport {
    /// Evaluate the maximum non-negative bound violation for `decision`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or decision vector is invalid.
    pub fn max_violation(&self, decision: &[f64]) -> Result<f64, TrajoptError> {
        validate_decision_bounds_report(self)?;
        validate_dense_decision_vector(decision, self.variable_count)?;
        let mut max_violation = 0.0_f64;
        for ((value, lower), upper) in decision
            .iter()
            .zip(self.lower_bounds.iter())
            .zip(self.upper_bounds.iter())
        {
            max_violation = max_violation.max((lower - value).max(0.0));
            max_violation = max_violation.max((value - upper).max(0.0));
        }
        Ok(max_violation)
    }
}

/// Result from the feature-gated dense SQP fixed-mesh solve wrapper.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshDenseSqpReport {
    /// Dense SQP solution returned by the feature-gated solver lane.
    pub sqp_solution: crate::sqp::DenseSqpSolution,
    /// Final residual report evaluated at the accepted SQP variables.
    pub residual_report: HermiteSimpsonFixedMeshResidualReport,
    /// Decision bounds consumed by the dense SQP solver after hard path merges.
    pub decision_bounds: HermiteSimpsonDecisionBoundsReport,
    /// Maximum violation over the merged decision bounds.
    pub max_decision_bound_violation: f64,
    /// Maximum violation over hard path inequality rows.
    pub max_hard_path_violation: f64,
}

/// PMP/KKT certificate artifacts derived from an accepted dense SQP solution.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshDenseSqpPmpKktCertificateReport {
    /// Sparse augmented-KKT report assembled at the accepted dense SQP decision.
    pub sparse_kkt_step_report: HermiteSimpsonFixedMeshSparseKktStepReport,
    /// Raw node costates mapped through the caller-selected PMP convention.
    pub costate_convention_report: crate::pmp::PmpDirectCostateConventionReport,
    /// PMP Hamiltonian profile evaluated from those convention-mapped costates.
    pub hamiltonian_report: crate::pmp::PmpHamiltonianReport,
    /// Optional Hamiltonian control-stationarity report attached to the certificate.
    pub control_stationarity_report: Option<crate::pmp::PmpHamiltonianControlStationarityReport>,
    /// Optional same-mesh PMP-vs-KKT costate comparison attached to the certificate.
    pub costate_comparison_report: Option<crate::pmp::PmpCostateComparisonReport>,
    /// Validated I-load optimality certificate for the Hamiltonian profile plus
    /// optional control-stationarity and costate-comparison summaries.
    pub optimality_certificate: ILoadOptimalityCertificate,
}

#[cfg(feature = "sqp")]
impl HermiteSimpsonFixedMeshDenseSqpPmpKktCertificateReport {
    /// Attach a validated same-mesh PMP-vs-KKT costate comparison.
    ///
    /// `pmp_times_s` and `pmp_costates` are the caller-provided PMP/oracle
    /// profile in the same PMP convention selected when this certificate report
    /// was built.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the supplied PMP profile is malformed, does
    /// not match this report's mesh/dimensions, or exceeds the declared
    /// comparison tolerance.
    pub fn with_pmp_costate_comparison(
        mut self,
        pmp_times_s: &[f64],
        pmp_costates: &[f64],
        settings: crate::pmp::PmpCostateComparisonSettings,
    ) -> Result<Self, TrajoptError> {
        attach_pmp_costate_comparison_to_certificate(
            &self.costate_convention_report,
            pmp_times_s,
            pmp_costates,
            settings,
            &mut self.costate_comparison_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }

    /// Attach a validated Hamiltonian control-stationarity summary.
    ///
    /// This evaluates `dH/du` from the certificate's solved PMP Hamiltonian
    /// profile and records the resulting tolerance-checked summary in the I-load
    /// optimality certificate.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the stationarity settings, callbacks, or
    /// resulting gradient summary are malformed or outside tolerance.
    pub fn with_pmp_control_stationarity<D, L>(
        mut self,
        settings: crate::pmp::PmpHamiltonianControlStationaritySettings,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<Self, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        attach_pmp_control_stationarity_to_certificate(
            &self.hamiltonian_report,
            settings,
            dynamics,
            lagrange,
            &mut self.control_stationarity_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }
}

/// Result from the feature-gated dense SQP solve wrapper with final time as a
/// scalar decision variable.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshFreeTimeDenseSqpReport {
    /// Dense SQP solution over `[final_time_s, decision...]`.
    pub sqp_solution: crate::sqp::DenseSqpSolution,
    /// Accepted final mesh time, s.
    pub final_time_s: f64,
    /// Accepted Hermite-Simpson decision vector without the final-time scalar.
    pub decision: Vec<f64>,
    /// Final residual report evaluated on the accepted uniform final-time mesh.
    pub residual_report: HermiteSimpsonFixedMeshResidualReport,
    /// Decision bounds consumed by the dense SQP solver after hard path merges.
    pub decision_bounds: HermiteSimpsonDecisionBoundsReport,
    /// Maximum violation over the final-time scalar bounds.
    pub max_final_time_bound_violation: f64,
    /// Maximum violation over the merged decision bounds.
    pub max_decision_bound_violation: f64,
    /// Maximum violation over hard path inequality rows.
    pub max_hard_path_violation: f64,
}

/// Result from the feature-gated dense SQP solve wrapper with final time as a
/// scalar decision variable and a linear final-time objective term.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshFreeTimeObjectiveDenseSqpReport {
    /// Dense SQP objective solution over `[final_time_s, decision...]`.
    pub sqp_solution: crate::sqp::DenseSqpObjectiveSolution,
    /// Positive linear objective weight applied to final time.
    pub final_time_objective_weight: f64,
    /// Accepted final mesh time, s.
    pub final_time_s: f64,
    /// Accepted Hermite-Simpson decision vector without the final-time scalar.
    pub decision: Vec<f64>,
    /// Final residual report evaluated on the accepted uniform final-time mesh.
    pub residual_report: HermiteSimpsonFixedMeshResidualReport,
    /// Decision bounds consumed by the dense SQP solver after hard path merges.
    pub decision_bounds: HermiteSimpsonDecisionBoundsReport,
    /// Maximum violation over the final-time scalar bounds.
    pub max_final_time_bound_violation: f64,
    /// Maximum violation over the merged decision bounds.
    pub max_decision_bound_violation: f64,
    /// Maximum violation over hard path inequality rows.
    pub max_hard_path_violation: f64,
}

/// Result from the feature-gated dense QP fixed-mesh solve wrapper for
/// quadratic-control OCPs with affine equality residuals.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshDenseQuadraticQpReport {
    /// Dense QP solution returned by Clarabel.
    pub qp_solution: crate::sqp::ClarabelQpSolution,
    /// Final residual report evaluated at the QP solution.
    pub residual_report: HermiteSimpsonFixedMeshResidualReport,
    /// Hard path inequalities consumed by the dense QP.
    pub hard_path_inequality_report: HermiteSimpsonPathInequalityReport,
    /// Number of equality rows sent to the dense QP.
    pub equality_row_count: usize,
    /// Euclidean norm of collocation plus boundary equality residuals.
    pub equality_residual_norm: f64,
    /// Maximum violation over scalar hard path inequality rows.
    pub max_hard_path_violation: f64,
}

/// Result from the feature-gated dense SQP fixed-mesh solve wrapper with
/// nonlinear hard path inequalities.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshDenseConstrainedSqpReport {
    /// Dense constrained SQP solution returned by the feature-gated solver lane.
    pub sqp_solution: crate::sqp::DenseConstrainedSqpSolution,
    /// Final residual report evaluated at the accepted SQP variables.
    pub residual_report: HermiteSimpsonFixedMeshResidualReport,
    /// Decision bounds consumed by the dense SQP solver after hard path merges.
    pub decision_bounds: HermiteSimpsonDecisionBoundsReport,
    /// Final nonlinear hard path inequality report.
    pub nonlinear_hard_path_report: HermiteSimpsonNonlinearPathInequalityReport,
    /// Maximum violation over the merged decision bounds.
    pub max_decision_bound_violation: f64,
    /// Maximum violation over scalar hard path inequality rows.
    pub max_hard_path_violation: f64,
    /// Maximum violation over nonlinear hard path inequality rows.
    pub max_nonlinear_hard_path_violation: f64,
}

/// Fixed-mesh Hermite-Simpson OCP residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct HermiteSimpsonFixedMeshOcp {
    /// Decision vector layout.
    pub layout: HermiteSimpsonDecisionLayout,
    /// Strictly increasing mesh node times, s.
    pub time_nodes_s: Vec<f64>,
    /// Node-state boundary residuals.
    pub boundary_constraints: Vec<HermiteSimpsonBoundaryConstraint>,
    /// Hard path bounds transcribed as linear inequality rows.
    pub hard_path_constraints: Vec<HermiteSimpsonHardPathConstraint>,
    /// Nonlinear hard path bounds evaluated through a caller-supplied path
    /// function and transcribed as `g(z) <= 0` rows.
    pub nonlinear_hard_path_constraints: Vec<HermiteSimpsonNonlinearHardPathConstraint>,
    /// Soft path bound residuals.
    pub path_constraints: Vec<HermiteSimpsonSoftPathConstraint>,
    /// Quadratic control objective residuals.
    pub quadratic_control_costs: Vec<HermiteSimpsonQuadraticControlCost>,
}

impl HermiteSimpsonFixedMeshOcp {
    /// Build a fixed-mesh Hermite-Simpson OCP residual model.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when times, dimensions, or boundary constraints
    /// are invalid.
    pub fn new(
        time_nodes_s: Vec<f64>,
        state_dimension: usize,
        control_dimension: usize,
        boundary_constraints: Vec<HermiteSimpsonBoundaryConstraint>,
    ) -> Result<Self, TrajoptError> {
        let segment_count =
            time_nodes_s
                .len()
                .checked_sub(1)
                .ok_or(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson fixed mesh requires at least two time nodes",
                })?;
        let layout =
            HermiteSimpsonDecisionLayout::new(segment_count, state_dimension, control_dimension)?;
        validate_time_nodes(&time_nodes_s, segment_count)?;
        for constraint in &boundary_constraints {
            validate_boundary_constraint(&layout, constraint)?;
        }
        Ok(Self {
            layout,
            time_nodes_s,
            boundary_constraints,
            hard_path_constraints: Vec::new(),
            nonlinear_hard_path_constraints: Vec::new(),
            path_constraints: Vec::new(),
            quadratic_control_costs: Vec::new(),
        })
    }

    /// Add hard path constraints to this fixed mesh.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when a path constraint references an invalid
    /// sample or component.
    pub fn with_hard_path_constraints(
        mut self,
        hard_path_constraints: Vec<HermiteSimpsonHardPathConstraint>,
    ) -> Result<Self, TrajoptError> {
        for constraint in &hard_path_constraints {
            validate_hard_path_constraint(&self.layout, constraint)?;
        }
        self.hard_path_constraints = hard_path_constraints;
        Ok(self)
    }

    /// Add nonlinear hard path constraints to this fixed mesh.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when a path constraint references an invalid
    /// sample or has malformed bounds.
    pub fn with_nonlinear_hard_path_constraints(
        mut self,
        nonlinear_hard_path_constraints: Vec<HermiteSimpsonNonlinearHardPathConstraint>,
    ) -> Result<Self, TrajoptError> {
        for constraint in &nonlinear_hard_path_constraints {
            validate_nonlinear_hard_path_constraint(&self.layout, constraint)?;
        }
        self.nonlinear_hard_path_constraints = nonlinear_hard_path_constraints;
        Ok(self)
    }

    /// Add soft path constraints to this fixed mesh.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when a path constraint references an invalid
    /// sample or component.
    pub fn with_path_constraints(
        mut self,
        path_constraints: Vec<HermiteSimpsonSoftPathConstraint>,
    ) -> Result<Self, TrajoptError> {
        for constraint in &path_constraints {
            validate_path_constraint(&self.layout, constraint)?;
        }
        self.path_constraints = path_constraints;
        Ok(self)
    }

    /// Add Simpson-weighted quadratic control costs to this fixed mesh.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when a control-cost component is invalid.
    pub fn with_quadratic_control_costs(
        mut self,
        quadratic_control_costs: Vec<HermiteSimpsonQuadraticControlCost>,
    ) -> Result<Self, TrajoptError> {
        for cost in &quadratic_control_costs {
            validate_quadratic_control_cost(&self.layout, cost)?;
        }
        self.quadratic_control_costs = quadratic_control_costs;
        Ok(self)
    }

    #[cfg(feature = "sqp")]
    fn with_uniform_final_time(&self, final_time_s: f64) -> Result<Self, TrajoptError> {
        if !final_time_s.is_finite() || final_time_s <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson free-final-time mesh must be positive",
            });
        }
        let mut time_nodes_s = Vec::with_capacity(self.layout.segment_count + 1);
        for index in 0..=self.layout.segment_count {
            time_nodes_s.push(final_time_s * (index as f64) / (self.layout.segment_count as f64));
        }
        validate_time_nodes(&time_nodes_s, self.layout.segment_count)?;
        let mut next = self.clone();
        next.time_nodes_s = time_nodes_s;
        Ok(next)
    }

    /// Decode `decision` into Hermite-Simpson segments for this fixed mesh.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector is invalid.
    pub fn segments_from_decision(
        &self,
        decision: &[f64],
    ) -> Result<Vec<HermiteSimpsonSegment>, TrajoptError> {
        self.layout
            .segments_from_decision(&self.time_nodes_s, decision)
    }

    /// Emit a deterministic optimized reference profile from node and midpoint
    /// state samples.
    ///
    /// Samples are ordered as `node_0, midpoint_0, node_1, ...,
    /// midpoint_{N-1}, node_N`. This is an output formatting surface only; it
    /// adds no target vocabulary.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector or state-component
    /// mapping is invalid, or emitted samples are malformed.
    pub fn optimized_reference_profile(
        &self,
        decision: &[f64],
        mapping: HermiteSimpsonReferenceProfileMapping,
    ) -> Result<HermiteSimpsonReferenceProfileReport, TrajoptError> {
        validate_decision_vector(&self.layout, decision)?;
        validate_reference_profile_mapping(&self.layout, mapping)?;

        let mut samples = Vec::with_capacity(2 * self.layout.segment_count + 1);
        samples.push(reference_profile_sample_from_state(
            self.time_nodes_s[0],
            self.layout.node_state(decision, 0)?,
            mapping,
        )?);
        for segment_index in 0..self.layout.segment_count {
            let midpoint_time_s =
                0.5 * (self.time_nodes_s[segment_index] + self.time_nodes_s[segment_index + 1]);
            let midpoint_state = slice(
                decision,
                self.layout.midpoint_state_offset + segment_index * self.layout.state_dimension,
                self.layout.state_dimension,
            );
            samples.push(reference_profile_sample_from_state(
                midpoint_time_s,
                midpoint_state,
                mapping,
            )?);
            samples.push(reference_profile_sample_from_state(
                self.time_nodes_s[segment_index + 1],
                self.layout.node_state(decision, segment_index + 1)?,
                mapping,
            )?);
        }

        let sample_count = samples.len();
        let start_time_s = samples
            .first()
            .ok_or(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson reference profile must contain samples",
            })?
            .time_s;
        let end_time_s = samples
            .last()
            .ok_or(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson reference profile must contain samples",
            })?
            .time_s;
        let report = HermiteSimpsonReferenceProfileReport {
            samples,
            sample_count,
            start_time_s,
            end_time_s,
        };
        validate_reference_profile_report(&report)?;
        Ok(report)
    }

    /// Package an optimized fixed-mesh reference profile as an I-load payload.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when reference-profile extraction or I-load
    /// payload validation fails.
    #[allow(clippy::too_many_arguments)]
    pub fn optimized_reference_iload_payload(
        &self,
        decision: &[f64],
        mapping: HermiteSimpsonReferenceProfileMapping,
        terminal_condition: &TerminalCondition,
        synthesis_seed: u64,
        producer: String,
        metadata: SynthesisMetadata,
        event_bindings_postcard: Vec<u8>,
        gain_tables: Vec<GainTable>,
    ) -> Result<ILoadPayload, TrajoptError> {
        self.optimized_reference_iload_payload_with_optional_certificate(
            decision,
            mapping,
            terminal_condition,
            synthesis_seed,
            producer,
            metadata,
            event_bindings_postcard,
            gain_tables,
            None,
        )
    }

    /// Package an optimized fixed-mesh reference profile as an I-load payload
    /// with attached optimality evidence.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when reference-profile extraction, certificate
    /// validation, or I-load payload validation fails.
    #[allow(clippy::too_many_arguments)]
    pub fn optimized_reference_iload_payload_with_optimality_certificate(
        &self,
        decision: &[f64],
        mapping: HermiteSimpsonReferenceProfileMapping,
        terminal_condition: &TerminalCondition,
        synthesis_seed: u64,
        producer: String,
        metadata: SynthesisMetadata,
        event_bindings_postcard: Vec<u8>,
        gain_tables: Vec<GainTable>,
        optimality_certificate: ILoadOptimalityCertificate,
    ) -> Result<ILoadPayload, TrajoptError> {
        self.optimized_reference_iload_payload_with_optional_certificate(
            decision,
            mapping,
            terminal_condition,
            synthesis_seed,
            producer,
            metadata,
            event_bindings_postcard,
            gain_tables,
            Some(optimality_certificate),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn optimized_reference_iload_payload_with_optional_certificate(
        &self,
        decision: &[f64],
        mapping: HermiteSimpsonReferenceProfileMapping,
        terminal_condition: &TerminalCondition,
        synthesis_seed: u64,
        producer: String,
        metadata: SynthesisMetadata,
        event_bindings_postcard: Vec<u8>,
        gain_tables: Vec<GainTable>,
        optimality_certificate: Option<ILoadOptimalityCertificate>,
    ) -> Result<ILoadPayload, TrajoptError> {
        let reference_profile = self.optimized_reference_profile(decision, mapping)?.samples;
        let payload = ILoadPayload {
            header: ILoadHeader::new_with_objective(
                terminal_condition,
                ILoadObjectiveClass::OptimizedReference,
                synthesis_seed,
                producer,
            ),
            metadata,
            event_bindings_postcard,
            gain_tables,
            reference_profile,
            optimality_certificate,
        };
        payload.validate()?;
        Ok(payload)
    }

    /// Evaluate collocation and boundary residuals for a flat decision vector.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when decision decoding, dynamics evaluation, or
    /// residual assembly fails.
    pub fn residual_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
    ) -> Result<HermiteSimpsonFixedMeshResidualReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        let segments = self.segments_from_decision(decision)?;
        let mut zero_lagrange = |_time_s: f64, _state: &[f64], _control: &[f64]| Ok(0.0);
        let mut residuals = Vec::with_capacity(segments.len() * self.layout.state_dimension * 2);
        let mut simpson_lagrange_cost = 0.0;
        for segment in &segments {
            let report = evaluate_hermite_simpson_segment(segment, dynamics, &mut zero_lagrange)?;
            simpson_lagrange_cost += report.simpson_lagrange_cost;
            residuals.extend(report.dynamics_defect);
            residuals.extend(report.midpoint_defect);
        }
        let collocation_residual_count = residuals.len();
        for constraint in &self.boundary_constraints {
            let state = self.layout.node_state(decision, constraint.node_index)?;
            residuals.push(state[constraint.component] - constraint.target);
        }
        let path_residual_count = append_path_residuals(self, decision, &mut residuals)?;
        let quadratic_control_cost =
            append_quadratic_control_cost_residuals(self, decision, &mut residuals)?;
        let residual_norm = residual_norm(&residuals);
        Ok(HermiteSimpsonFixedMeshResidualReport {
            residuals,
            collocation_residual_count,
            boundary_residual_count: self.boundary_constraints.len(),
            path_residual_count,
            objective_residual_count: self.layout.segment_count
                * 3
                * self.quadratic_control_costs.len(),
            residual_norm,
            simpson_lagrange_cost,
            quadratic_control_cost,
        })
    }

    /// Evaluate only the residual vector for a flat decision vector.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when residual assembly fails.
    pub fn residuals<D>(&self, decision: &[f64], dynamics: &mut D) -> Result<Vec<f64>, TrajoptError>
    where
        D: CollocationDynamics,
    {
        Ok(self.residual_report(decision, dynamics)?.residuals)
    }

    /// Evaluate the fixed-mesh residual Jacobian as CSC sparse storage.
    ///
    /// The derivative is computed with deterministic central finite
    /// differences and drops coefficients whose absolute value is at or below
    /// `drop_tolerance`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, finite-difference
    /// settings, residual evaluation, or residual dimension stability is
    /// invalid.
    pub fn sparse_residual_jacobian_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        finite_difference_step: f64,
        drop_tolerance: f64,
    ) -> Result<HermiteSimpsonSparseResidualJacobianReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        validate_finite_difference_settings(finite_difference_step, drop_tolerance)?;
        validate_decision_vector(&self.layout, decision)?;

        let base_residual = self.residuals(decision, dynamics)?;
        let row_count = base_residual.len();
        let column_count = self.layout.variable_count;
        let mut column_offsets = Vec::with_capacity(column_count + 1);
        let mut row_indices = Vec::new();
        let mut values = Vec::new();
        column_offsets.push(0);

        for column in 0..column_count {
            let step = finite_difference_step * decision[column].abs().max(1.0);
            let mut plus = decision.to_vec();
            let mut minus = decision.to_vec();
            plus[column] += step;
            minus[column] -= step;
            validate_decision_vector(&self.layout, &plus)?;
            validate_decision_vector(&self.layout, &minus)?;

            let plus_residual = self.residuals(&plus, dynamics)?;
            validate_residual_dimension(&plus_residual, row_count)?;
            let minus_residual = self.residuals(&minus, dynamics)?;
            validate_residual_dimension(&minus_residual, row_count)?;

            let scale = 2.0 * step;
            for row in 0..row_count {
                let coefficient = (plus_residual[row] - minus_residual[row]) / scale;
                if coefficient.abs() > drop_tolerance {
                    row_indices.push(row);
                    values.push(coefficient);
                }
            }
            column_offsets.push(values.len());
        }

        Ok(HermiteSimpsonSparseResidualJacobianReport {
            row_count,
            column_count,
            column_offsets,
            row_indices,
            values,
            finite_difference_step,
            drop_tolerance,
        })
    }

    /// Assemble and solve one sparse Gauss-Newton normal-equation step.
    ///
    /// This evaluates the fixed-mesh residuals and deterministic central-
    /// finite-difference sparse Jacobian at `decision`, forms
    /// `J'J + regularization I`, assembles `-J'r`, and solves the normal system
    /// with conjugate gradients. The report exposes the sparse matrix artifacts
    /// and the linearized residual norm. This is a sparse normal-equation step;
    /// it is not a sparse KKT factorization.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when residual/Jacobian assembly, sparse normal
    /// block assembly, or the CG solve fails.
    #[allow(clippy::too_many_arguments)]
    pub fn sparse_gauss_newton_step_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        finite_difference_step: f64,
        jacobian_drop_tolerance: f64,
        hessian_regularization: f64,
        hessian_drop_tolerance: f64,
        cg_max_iterations: usize,
        cg_residual_tolerance: f64,
    ) -> Result<HermiteSimpsonFixedMeshSparseGaussNewtonStepReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        let residual_report = self.residual_report(decision, dynamics)?;
        let sparse_jacobian = self.sparse_residual_jacobian_report(
            decision,
            dynamics,
            finite_difference_step,
            jacobian_drop_tolerance,
        )?;
        assemble_sparse_gauss_newton_step_report(
            residual_report,
            sparse_jacobian,
            hessian_regularization,
            hessian_drop_tolerance,
            cg_max_iterations,
            cg_residual_tolerance,
        )
    }

    /// Assemble and solve one sparse augmented-KKT linearized least-squares step.
    ///
    /// This evaluates residuals and a sparse finite-difference Jacobian at
    /// `decision`, builds `[regularization * I, J'; J, -I]`, and solves the
    /// KKT system with deterministic LDL factorization. The primal step matches
    /// the regularized sparse Gauss-Newton normal-equation step while exposing
    /// KKT multipliers/residual variables for later costate and large-sparse SQP
    /// work.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when residual/Jacobian assembly, KKT assembly, or
    /// the LDL solve fails.
    #[allow(clippy::too_many_arguments)]
    pub fn sparse_augmented_kkt_step_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        finite_difference_step: f64,
        jacobian_drop_tolerance: f64,
        kkt_regularization: f64,
        kkt_drop_tolerance: f64,
    ) -> Result<HermiteSimpsonFixedMeshSparseKktStepReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        let residual_report = self.residual_report(decision, dynamics)?;
        let sparse_jacobian = self.sparse_residual_jacobian_report(
            decision,
            dynamics,
            finite_difference_step,
            jacobian_drop_tolerance,
        )?;
        assemble_sparse_augmented_kkt_step_report(
            residual_report,
            sparse_jacobian,
            kkt_regularization,
            kkt_drop_tolerance,
        )
    }

    /// Assemble and solve one sparse constrained-KKT linearized least-squares
    /// step using active scalar hard-path inequalities.
    ///
    /// This evaluates residuals, a sparse residual Jacobian, and dense hard
    /// scalar path inequalities at `decision`. Rows with `Aineq * decision -
    /// rhs >= -active_tolerance` are inserted into the KKT system as active
    /// linearized equalities. The report exposes active row indices, active
    /// inequality multipliers, and linearized active-row closure. This is an
    /// active-set constrained sparse-KKT step substrate, not a full inequality
    /// globalization or production large-sparse NLP backend.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when residual/Jacobian/inequality assembly, KKT
    /// assembly, active-row selection, or the LDL solve fails.
    #[allow(clippy::too_many_arguments)]
    pub fn sparse_constrained_kkt_step_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        finite_difference_step: f64,
        jacobian_drop_tolerance: f64,
        kkt_regularization: f64,
        kkt_drop_tolerance: f64,
        active_tolerance: f64,
    ) -> Result<HermiteSimpsonFixedMeshSparseConstrainedKktStepReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        if !active_tolerance.is_finite() || active_tolerance < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse constrained KKT active tolerance must be finite and non-negative",
            });
        }
        let residual_report = self.residual_report(decision, dynamics)?;
        let sparse_jacobian = self.sparse_residual_jacobian_report(
            decision,
            dynamics,
            finite_difference_step,
            jacobian_drop_tolerance,
        )?;
        let hard_path_inequality_report = self.hard_path_inequality_report()?;
        assemble_sparse_constrained_kkt_step_report(
            residual_report,
            sparse_jacobian,
            hard_path_inequality_report,
            decision,
            kkt_regularization,
            kkt_drop_tolerance,
            active_tolerance,
        )
    }

    /// Solve the fixed-mesh residual model with a sparse augmented-KKT SQP loop.
    ///
    /// Each accepted iteration evaluates the residual/Jacobian at the current
    /// decision, solves the sparse augmented KKT system, and applies a
    /// backtracking line search on residual norm. The final report includes a
    /// sparse KKT solve assembled at the accepted final decision so residual
    /// multipliers come from the final sparse loop state. This convenience
    /// method uses a wide finite default decision-bound envelope before merging
    /// scalar hard-path constraints.
    ///
    /// This is a residual-only sparse SQP substrate with scalar hard-path bound
    /// enforcement; full inequality multiplier and constrained large-sparse NLP
    /// features remain separate work.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when settings, residual/Jacobian/KKT assembly,
    /// line-search evaluations, or final multiplier assembly fail.
    pub fn solve_sparse_kkt_sqp<D>(
        &self,
        initial_decision: &[f64],
        dynamics: &mut D,
        settings: SparseKktSqpSettings,
    ) -> Result<HermiteSimpsonFixedMeshSparseKktSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        let (default_lower_bounds, default_upper_bounds) =
            sparse_kkt_sqp_default_decision_bounds(self.layout.variable_count);
        self.solve_sparse_kkt_sqp_with_decision_bounds(
            initial_decision,
            &default_lower_bounds,
            &default_upper_bounds,
            dynamics,
            settings,
        )
    }

    /// Solve the fixed-mesh residual model with a bounded sparse augmented-KKT
    /// SQP loop.
    ///
    /// Scalar hard-path constraints are merged into the supplied finite default
    /// decision bounds. Initial decisions and line-search trial decisions must
    /// satisfy those merged bounds within the residual tolerance, so hard scalar
    /// path bounds are enforced even though the sparse KKT system itself still
    /// contains only residual rows.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when settings, decision bounds, residual/Jacobian
    /// assembly, line-search evaluations, or final multiplier assembly fail.
    pub fn solve_sparse_kkt_sqp_with_decision_bounds<D>(
        &self,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        dynamics: &mut D,
        settings: SparseKktSqpSettings,
    ) -> Result<HermiteSimpsonFixedMeshSparseKktSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        settings.validate()?;
        validate_decision_vector(&self.layout, initial_decision)?;
        if !self.nonlinear_hard_path_constraints.is_empty() {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse KKT SQP with nonlinear hard path constraints requires a path callback",
            });
        }
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        if decision_bounds.max_violation(initial_decision)? > settings.residual_tolerance {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse KKT SQP initial decision violates bounds",
            });
        }

        let mut decision = initial_decision.to_vec();
        let mut residual_report = self.residual_report(&decision, dynamics)?;
        let mut iterations = Vec::new();
        let mut converged = residual_report.residual_norm <= settings.residual_tolerance;

        for iteration in 0..settings.max_iterations {
            if converged {
                break;
            }
            let step_report = self.sparse_augmented_kkt_step_report(
                &decision,
                dynamics,
                settings.finite_difference_step,
                settings.jacobian_drop_tolerance,
                settings.kkt_regularization,
                settings.kkt_drop_tolerance,
            )?;
            if step_report.step_norm <= settings.step_tolerance {
                break;
            }

            let mut scale = 1.0_f64;
            let mut accepted = None;
            for _ in 0..settings.max_line_search_backtracks {
                let trial = scaled_decision_step(&decision, &step_report.step, scale)?;
                validate_decision_vector(&self.layout, &trial)?;
                if decision_bounds.max_violation(&trial)? > settings.residual_tolerance {
                    scale *= 0.5;
                    if scale < settings.min_line_search_scale {
                        break;
                    }
                    continue;
                }
                let trial_report = self.residual_report(&trial, dynamics)?;
                if trial_report.residual_norm < residual_report.residual_norm
                    || trial_report.residual_norm <= settings.residual_tolerance
                {
                    accepted = Some((trial, trial_report, scale));
                    break;
                }
                scale *= 0.5;
                if scale < settings.min_line_search_scale {
                    break;
                }
            }
            let Some((accepted_decision, accepted_residual_report, accepted_scale)) = accepted
            else {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson sparse KKT SQP line search failed to reduce residual",
                });
            };
            iterations.push(SparseKktSqpIterationReport {
                iteration,
                residual_norm: residual_report.residual_norm,
                linearized_residual_norm: step_report.linearized_residual_norm,
                step_norm: step_report.step_norm,
                accepted_step_scale: accepted_scale,
                accepted_residual_norm: accepted_residual_report.residual_norm,
            });
            decision = accepted_decision;
            residual_report = accepted_residual_report;
            converged = residual_report.residual_norm <= settings.residual_tolerance;
        }

        let final_kkt_step_report = self.sparse_augmented_kkt_step_report(
            &decision,
            dynamics,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
        )?;
        let max_decision_bound_violation = decision_bounds.max_violation(&decision)?;
        let hard_path_inequalities = self.hard_path_inequality_report()?;
        let final_constrained_kkt_step_report = sparse_constrained_kkt_step_report_if_active(
            &decision,
            settings,
            &hard_path_inequalities,
            &final_kkt_step_report,
        )?;
        let max_hard_path_violation = hard_path_inequalities.max_violation(&decision)?;
        Ok(HermiteSimpsonFixedMeshSparseKktSqpReport {
            decision,
            residual_report,
            final_kkt_step_report,
            final_constrained_kkt_step_report,
            decision_bounds,
            max_decision_bound_violation,
            max_hard_path_violation,
            nonlinear_hard_path_report: None,
            max_nonlinear_hard_path_violation: 0.0,
            iterations,
            converged,
        })
    }

    /// Solve the fixed-mesh residual model with a sparse augmented-KKT SQP loop
    /// while preserving nonlinear hard-path feasibility.
    ///
    /// Scalar hard-path constraints are merged into finite decision bounds.
    /// Nonlinear hard-path constraints are evaluated through `path_function` at
    /// the initial decision and every line-search trial. The sparse KKT system
    /// still contains only residual rows, so this is a feasibility-preserving
    /// constrained sparse-SQP substrate rather than a full inequality-multiplier
    /// large-sparse NLP solve.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when settings, decision bounds, residual/Jacobian
    /// assembly, nonlinear hard-path evaluation, line-search evaluations, or
    /// final multiplier assembly fail.
    pub fn solve_sparse_kkt_sqp_with_nonlinear_hard_path_constraints<D, P>(
        &self,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        dynamics: &mut D,
        path_function: &mut P,
        settings: SparseKktSqpSettings,
    ) -> Result<HermiteSimpsonFixedMeshSparseKktSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
        P: CollocationPathFunction,
    {
        settings.validate()?;
        validate_decision_vector(&self.layout, initial_decision)?;
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        if decision_bounds.max_violation(initial_decision)? > settings.residual_tolerance {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse KKT SQP initial decision violates bounds",
            });
        }
        let mut nonlinear_hard_path_report =
            self.nonlinear_hard_path_inequality_report(initial_decision, path_function)?;
        if nonlinear_hard_path_report.max_violation > settings.residual_tolerance {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse KKT SQP initial decision violates nonlinear hard path constraints",
            });
        }

        let mut decision = initial_decision.to_vec();
        let mut residual_report = self.residual_report(&decision, dynamics)?;
        let mut iterations = Vec::new();
        let mut converged = residual_report.residual_norm <= settings.residual_tolerance
            && nonlinear_hard_path_report.max_violation <= settings.residual_tolerance;

        for iteration in 0..settings.max_iterations {
            if converged {
                break;
            }
            let step_report = self.sparse_augmented_kkt_step_report(
                &decision,
                dynamics,
                settings.finite_difference_step,
                settings.jacobian_drop_tolerance,
                settings.kkt_regularization,
                settings.kkt_drop_tolerance,
            )?;
            if step_report.step_norm <= settings.step_tolerance {
                break;
            }

            let mut scale = 1.0_f64;
            let mut accepted = None;
            for _ in 0..settings.max_line_search_backtracks {
                let trial = scaled_decision_step(&decision, &step_report.step, scale)?;
                validate_decision_vector(&self.layout, &trial)?;
                if decision_bounds.max_violation(&trial)? > settings.residual_tolerance {
                    scale *= 0.5;
                    if scale < settings.min_line_search_scale {
                        break;
                    }
                    continue;
                }
                let trial_nonlinear_report =
                    self.nonlinear_hard_path_inequality_report(&trial, path_function)?;
                if trial_nonlinear_report.max_violation > settings.residual_tolerance {
                    scale *= 0.5;
                    if scale < settings.min_line_search_scale {
                        break;
                    }
                    continue;
                }
                let trial_report = self.residual_report(&trial, dynamics)?;
                if trial_report.residual_norm < residual_report.residual_norm
                    || trial_report.residual_norm <= settings.residual_tolerance
                {
                    accepted = Some((trial, trial_report, trial_nonlinear_report, scale));
                    break;
                }
                scale *= 0.5;
                if scale < settings.min_line_search_scale {
                    break;
                }
            }
            let Some((
                accepted_decision,
                accepted_residual_report,
                accepted_nonlinear_report,
                accepted_scale,
            )) = accepted
            else {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson sparse KKT SQP line search failed to reduce residual while preserving nonlinear hard path constraints",
                });
            };
            iterations.push(SparseKktSqpIterationReport {
                iteration,
                residual_norm: residual_report.residual_norm,
                linearized_residual_norm: step_report.linearized_residual_norm,
                step_norm: step_report.step_norm,
                accepted_step_scale: accepted_scale,
                accepted_residual_norm: accepted_residual_report.residual_norm,
            });
            decision = accepted_decision;
            residual_report = accepted_residual_report;
            nonlinear_hard_path_report = accepted_nonlinear_report;
            converged = residual_report.residual_norm <= settings.residual_tolerance
                && nonlinear_hard_path_report.max_violation <= settings.residual_tolerance;
        }

        let final_kkt_step_report = self.sparse_augmented_kkt_step_report(
            &decision,
            dynamics,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
        )?;
        let max_decision_bound_violation = decision_bounds.max_violation(&decision)?;
        let hard_path_inequalities = self.hard_path_inequality_report()?;
        let final_constrained_kkt_step_report = sparse_constrained_kkt_step_report_if_active(
            &decision,
            settings,
            &hard_path_inequalities,
            &final_kkt_step_report,
        )?;
        let max_hard_path_violation = hard_path_inequalities.max_violation(&decision)?;
        let max_nonlinear_hard_path_violation = nonlinear_hard_path_report.max_violation;
        Ok(HermiteSimpsonFixedMeshSparseKktSqpReport {
            decision,
            residual_report,
            final_kkt_step_report,
            final_constrained_kkt_step_report,
            decision_bounds,
            max_decision_bound_violation,
            max_hard_path_violation,
            nonlinear_hard_path_report: Some(nonlinear_hard_path_report),
            max_nonlinear_hard_path_violation,
            iterations,
            converged,
        })
    }

    /// Split sparse augmented-KKT residual multipliers by fixed-mesh row family.
    ///
    /// The collocation slices are segment-major, with dynamics-defect
    /// multipliers followed by midpoint-defect multipliers for each segment.
    /// This exposes deterministic raw multiplier evidence for later
    /// costate/PMP witnesses while preserving the exact residual-row ordering.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report does not match this OCP layout
    /// or contains malformed multiplier dimensions.
    pub fn kkt_residual_multiplier_report(
        &self,
        step_report: &HermiteSimpsonFixedMeshSparseKktStepReport,
    ) -> Result<HermiteSimpsonKktResidualMultiplierReport, TrajoptError> {
        assemble_kkt_residual_multiplier_report(&self.layout, step_report)
    }

    /// Split dense-QP equality duals by fixed-mesh KKT row family.
    ///
    /// Dense quadratic-control QPs include collocation and boundary residuals
    /// as equality rows while quadratic-control costs live in the QP Hessian.
    /// This adapter zero-fills non-equality residual families and feeds the
    /// solver-produced equality duals through the same raw multiplier splitter
    /// used by sparse augmented-KKT diagnostics.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense-QP report does not match this
    /// OCP layout or its equality dual vector is malformed.
    #[cfg(feature = "sqp")]
    pub fn dense_qp_kkt_residual_multiplier_report(
        &self,
        report: &HermiteSimpsonFixedMeshDenseQuadraticQpReport,
    ) -> Result<HermiteSimpsonKktResidualMultiplierReport, TrajoptError> {
        let residual_multipliers = dense_qp_residual_multipliers_from_equality_duals(report)?;
        assemble_kkt_residual_multiplier_report_from_residuals(
            &self.layout,
            &report.residual_report,
            &residual_multipliers,
        )
    }

    /// Estimate raw node costates from dense-QP equality duals.
    ///
    /// Returned values remain raw direct-method diagnostics; callers must choose
    /// an explicit PMP sign/scale convention before comparing them with an
    /// indirect solution.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense-QP multiplier report is malformed
    /// or does not match this OCP layout.
    #[cfg(feature = "sqp")]
    pub fn dense_qp_kkt_node_costate_estimate_report(
        &self,
        report: &HermiteSimpsonFixedMeshDenseQuadraticQpReport,
    ) -> Result<HermiteSimpsonKktCostateEstimateReport, TrajoptError> {
        let multiplier_report = self.dense_qp_kkt_residual_multiplier_report(report)?;
        assemble_kkt_node_costate_estimate_report(&self.time_nodes_s, &multiplier_report)
    }

    /// Apply an explicit PMP convention to dense-QP-derived raw node costates.
    ///
    /// This is a bridge from solver-produced equality duals to the common PMP
    /// comparison shape. It does not infer or certify a production covector
    /// convention.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense-QP report is malformed or the PMP
    /// convention is invalid.
    #[cfg(feature = "sqp")]
    pub fn dense_qp_pmp_costate_convention_report(
        &self,
        report: &HermiteSimpsonFixedMeshDenseQuadraticQpReport,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        self.dense_qp_kkt_node_costate_estimate_report(report)?
            .pmp_costate_convention_report(convention)
    }

    /// Evaluate a PMP Hamiltonian profile from dense-QP-derived node costates.
    ///
    /// This maps solver-produced equality duals through the caller-selected PMP
    /// costate convention, samples the solved node states and controls, and
    /// evaluates `H = L + lambda' f` through the common PMP witness helper.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense-QP report is malformed, the PMP
    /// convention is invalid, the solved decision vector is malformed, or the
    /// dynamics/Lagrange callbacks emit invalid values.
    #[cfg(feature = "sqp")]
    pub fn dense_qp_pmp_hamiltonian_report<D, L>(
        &self,
        report: &HermiteSimpsonFixedMeshDenseQuadraticQpReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<crate::pmp::PmpHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        let costate_report = self.dense_qp_kkt_node_costate_estimate_report(report)?;
        let convention_report = costate_report.pmp_costate_convention_report(convention)?;
        let mut states =
            Vec::with_capacity(convention_report.sample_count * self.layout.state_dimension);
        let mut controls =
            Vec::with_capacity(convention_report.sample_count * self.layout.control_dimension);
        for node_index in 0..=self.layout.segment_count {
            let (time_s, state, control) = sample_state_control(
                &self.layout,
                &self.time_nodes_s,
                &report.qp_solution.primal,
                HermiteSimpsonSamplePoint::Node(node_index),
            )?;
            if time_s.to_bits() != convention_report.times_s[node_index].to_bits() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson dense-QP PMP Hamiltonian sample time is inconsistent",
                });
            }
            states.extend_from_slice(state);
            controls.extend_from_slice(control);
        }
        crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: self.layout.state_dimension,
                control_dimension: self.layout.control_dimension,
                times_s: &convention_report.times_s,
                states: &states,
                controls: &controls,
                costates: &convention_report.pmp_costates,
            },
            |time_s, state, control| dynamics.eval(time_s, state, control),
            |time_s, state, control| lagrange.eval(time_s, state, control),
        )
    }

    /// Estimate node costates from sparse augmented-KKT dynamics multipliers.
    ///
    /// Interior node samples average the adjacent segment dynamics-defect
    /// multipliers; endpoint samples use their single adjacent segment. The
    /// returned values are raw and unscaled so a later PMP verifier can apply
    /// the selected covector sign/scaling convention explicitly.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the KKT multiplier report is malformed or
    /// does not match this OCP layout.
    pub fn kkt_node_costate_estimate_report(
        &self,
        step_report: &HermiteSimpsonFixedMeshSparseKktStepReport,
    ) -> Result<HermiteSimpsonKktCostateEstimateReport, TrajoptError> {
        let multiplier_report = self.kkt_residual_multiplier_report(step_report)?;
        assemble_kkt_node_costate_estimate_report(&self.time_nodes_s, &multiplier_report)
    }

    /// Apply an explicit PMP convention to sparse-KKT-derived raw node
    /// costates.
    ///
    /// This is the sparse augmented-KKT analogue of the dense-QP PMP costate
    /// adapter. It packages residual-multiplier-derived node costates into the
    /// common PMP comparison shape after the caller selects the sign/scale
    /// convention explicitly.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse KKT report is malformed or the
    /// PMP convention is invalid.
    pub fn sparse_kkt_pmp_costate_convention_report(
        &self,
        step_report: &HermiteSimpsonFixedMeshSparseKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        self.kkt_node_costate_estimate_report(step_report)?
            .pmp_costate_convention_report(convention)
    }

    /// Evaluate raw node Hamiltonian diagnostics from KKT costate estimates.
    ///
    /// This computes `H = L + lambda' f` at mesh nodes using the unscaled
    /// node-costate estimates produced from the sparse augmented-KKT report.
    /// It is a deterministic diagnostic substrate for a later PMP witness, not
    /// a proof that Hamiltonian constancy or a final covector convention holds.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, KKT report, dynamics,
    /// Lagrange callback, or generated dimensions are invalid.
    pub fn kkt_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &HermiteSimpsonFixedMeshSparseKktStepReport,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<HermiteSimpsonKktHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_decision_vector(&self.layout, decision)?;
        let costate_report = self.kkt_node_costate_estimate_report(step_report)?;
        assemble_kkt_hamiltonian_report(
            &self.layout,
            &self.time_nodes_s,
            decision,
            &costate_report,
            dynamics,
            lagrange,
        )
    }

    /// Evaluate a PMP-convention Hamiltonian profile from sparse-KKT node
    /// costates.
    ///
    /// This maps sparse augmented-KKT residual multipliers through the
    /// caller-selected PMP costate convention, samples node states and controls
    /// from `decision`, and evaluates `H = L + lambda' f` through the common PMP
    /// witness helper.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, sparse KKT report,
    /// PMP convention, dynamics callback, or Lagrange callback is malformed.
    pub fn sparse_kkt_pmp_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &HermiteSimpsonFixedMeshSparseKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<crate::pmp::PmpHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_decision_vector(&self.layout, decision)?;
        let costate_report = self.kkt_node_costate_estimate_report(step_report)?;
        let convention_report = costate_report.pmp_costate_convention_report(convention)?;
        let mut states =
            Vec::with_capacity(convention_report.sample_count * self.layout.state_dimension);
        let mut controls =
            Vec::with_capacity(convention_report.sample_count * self.layout.control_dimension);
        for node_index in 0..=self.layout.segment_count {
            let (time_s, state, control) = sample_state_control(
                &self.layout,
                &self.time_nodes_s,
                decision,
                HermiteSimpsonSamplePoint::Node(node_index),
            )?;
            if time_s.to_bits() != convention_report.times_s[node_index].to_bits() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson sparse-KKT PMP Hamiltonian sample time is inconsistent",
                });
            }
            states.extend_from_slice(state);
            controls.extend_from_slice(control);
        }
        crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: self.layout.state_dimension,
                control_dimension: self.layout.control_dimension,
                times_s: &convention_report.times_s,
                states: &states,
                controls: &controls,
                costates: &convention_report.pmp_costates,
            },
            |time_s, state, control| dynamics.eval(time_s, state, control),
            |time_s, state, control| lagrange.eval(time_s, state, control),
        )
    }

    /// Split constrained sparse-KKT residual multipliers by fixed-mesh row family.
    ///
    /// Active hard-inequality multipliers are retained on the constrained step
    /// report; this adapter consumes only the residual-space block so downstream
    /// costate diagnostics preserve the existing Hermite-Simpson row convention.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the constrained step report does not match
    /// this OCP layout or contains malformed multiplier dimensions.
    pub fn sparse_constrained_kkt_residual_multiplier_report(
        &self,
        step_report: &HermiteSimpsonFixedMeshSparseConstrainedKktStepReport,
    ) -> Result<HermiteSimpsonKktResidualMultiplierReport, TrajoptError> {
        validate_sparse_constrained_kkt_step_report_for_ocp(&self.layout, step_report)?;
        assemble_kkt_residual_multiplier_report_from_residuals(
            &self.layout,
            &step_report.residual_report,
            &step_report.residual_multipliers,
        )
    }

    /// Estimate raw node costates from constrained sparse-KKT residual multipliers.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the constrained KKT multiplier report is
    /// malformed or does not match this OCP layout.
    pub fn sparse_constrained_kkt_node_costate_estimate_report(
        &self,
        step_report: &HermiteSimpsonFixedMeshSparseConstrainedKktStepReport,
    ) -> Result<HermiteSimpsonKktCostateEstimateReport, TrajoptError> {
        let multiplier_report =
            self.sparse_constrained_kkt_residual_multiplier_report(step_report)?;
        assemble_kkt_node_costate_estimate_report(&self.time_nodes_s, &multiplier_report)
    }

    /// Apply an explicit PMP convention to constrained sparse-KKT raw node costates.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the constrained KKT report is malformed or the
    /// PMP convention is invalid.
    pub fn sparse_constrained_kkt_pmp_costate_convention_report(
        &self,
        step_report: &HermiteSimpsonFixedMeshSparseConstrainedKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        self.sparse_constrained_kkt_node_costate_estimate_report(step_report)?
            .pmp_costate_convention_report(convention)
    }

    /// Evaluate raw node Hamiltonian diagnostics from constrained sparse-KKT costates.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, constrained KKT report,
    /// dynamics, Lagrange callback, or generated dimensions are invalid.
    pub fn sparse_constrained_kkt_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &HermiteSimpsonFixedMeshSparseConstrainedKktStepReport,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<HermiteSimpsonKktHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_decision_vector(&self.layout, decision)?;
        let costate_report =
            self.sparse_constrained_kkt_node_costate_estimate_report(step_report)?;
        assemble_kkt_hamiltonian_report(
            &self.layout,
            &self.time_nodes_s,
            decision,
            &costate_report,
            dynamics,
            lagrange,
        )
    }

    /// Evaluate a PMP-convention Hamiltonian profile from constrained sparse-KKT
    /// node costates.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, constrained KKT report,
    /// PMP convention, dynamics callback, or Lagrange callback is malformed.
    pub fn sparse_constrained_kkt_pmp_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &HermiteSimpsonFixedMeshSparseConstrainedKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<crate::pmp::PmpHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_decision_vector(&self.layout, decision)?;
        let costate_report =
            self.sparse_constrained_kkt_node_costate_estimate_report(step_report)?;
        let convention_report = costate_report.pmp_costate_convention_report(convention)?;
        let mut states =
            Vec::with_capacity(convention_report.sample_count * self.layout.state_dimension);
        let mut controls =
            Vec::with_capacity(convention_report.sample_count * self.layout.control_dimension);
        for node_index in 0..=self.layout.segment_count {
            let (time_s, state, control) = sample_state_control(
                &self.layout,
                &self.time_nodes_s,
                decision,
                HermiteSimpsonSamplePoint::Node(node_index),
            )?;
            if time_s.to_bits() != convention_report.times_s[node_index].to_bits() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson constrained sparse-KKT PMP Hamiltonian sample time is inconsistent",
                });
            }
            states.extend_from_slice(state);
            controls.extend_from_slice(control);
        }
        crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: self.layout.state_dimension,
                control_dimension: self.layout.control_dimension,
                times_s: &convention_report.times_s,
                states: &states,
                controls: &controls,
                costates: &convention_report.pmp_costates,
            },
            |time_s, state, control| dynamics.eval(time_s, state, control),
            |time_s, state, control| lagrange.eval(time_s, state, control),
        )
    }

    /// Build PMP/KKT certificate artifacts from one active constrained-KKT step.
    ///
    /// This consumes an already assembled sparse constrained-KKT report, applies
    /// the caller-selected PMP costate convention, evaluates the common
    /// Hamiltonian witness, and packages a validated I-load certificate. The
    /// helper makes the active inequality multiplier handoff auditable without
    /// claiming a full production large-sparse NLP solve.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, constrained KKT
    /// report, PMP convention, Hamiltonian callbacks, or declared certificate
    /// tolerance are malformed.
    pub fn sparse_constrained_kkt_pmp_kkt_certificate_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &HermiteSimpsonFixedMeshSparseConstrainedKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        hamiltonian_drift_tolerance: f64,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<HermiteSimpsonFixedMeshSparseConstrainedKktPmpKktCertificateReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_decision_vector(&self.layout, decision)?;
        validate_sparse_constrained_kkt_step_report_for_ocp(&self.layout, step_report)?;
        let costate_convention_report =
            self.sparse_constrained_kkt_pmp_costate_convention_report(step_report, convention)?;
        let hamiltonian_report = self.sparse_constrained_kkt_pmp_hamiltonian_report(
            decision,
            step_report,
            convention,
            dynamics,
            lagrange,
        )?;
        let optimality_certificate = ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            ILoadOptimalityEvidenceKind::HermiteSimpsonPmpKkt,
            &hamiltonian_report,
            hamiltonian_drift_tolerance,
        )?;
        Ok(
            HermiteSimpsonFixedMeshSparseConstrainedKktPmpKktCertificateReport {
                constrained_kkt_step_report: step_report.clone(),
                costate_convention_report,
                hamiltonian_report,
                control_stationarity_report: None,
                costate_comparison_report: None,
                optimality_certificate,
            },
        )
    }

    /// Build PMP/KKT certificate artifacts from an accepted dense SQP solution.
    ///
    /// This assembles a sparse augmented-KKT report at the final dense SQP
    /// decision, applies the caller-selected PMP costate convention to the
    /// residual-multiplier costate estimates, evaluates the common Hamiltonian
    /// witness, and packages a validated I-load certificate. The helper is a
    /// final-solution audit surface for the dense SQP lane; it does not infer
    /// a covector sign convention or replace a production large-sparse NLP
    /// backend.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense SQP report, sparse KKT assembly,
    /// PMP convention, Hamiltonian callbacks, or declared certificate tolerance
    /// are malformed.
    #[cfg(feature = "sqp")]
    #[allow(clippy::too_many_arguments)]
    pub fn dense_sqp_pmp_kkt_certificate_report<D, L>(
        &self,
        report: &HermiteSimpsonFixedMeshDenseSqpReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        hamiltonian_drift_tolerance: f64,
        dynamics: &mut D,
        lagrange: &mut L,
        finite_difference_step: f64,
        jacobian_drop_tolerance: f64,
        kkt_regularization: f64,
        kkt_drop_tolerance: f64,
    ) -> Result<HermiteSimpsonFixedMeshDenseSqpPmpKktCertificateReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_decision_vector(&self.layout, &report.sqp_solution.variables)?;
        let sparse_kkt_step_report = self.sparse_augmented_kkt_step_report(
            &report.sqp_solution.variables,
            dynamics,
            finite_difference_step,
            jacobian_drop_tolerance,
            kkt_regularization,
            kkt_drop_tolerance,
        )?;
        let costate_convention_report =
            self.sparse_kkt_pmp_costate_convention_report(&sparse_kkt_step_report, convention)?;
        let hamiltonian_report = self.sparse_kkt_pmp_hamiltonian_report(
            &report.sqp_solution.variables,
            &sparse_kkt_step_report,
            convention,
            dynamics,
            lagrange,
        )?;
        let optimality_certificate = ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            ILoadOptimalityEvidenceKind::HermiteSimpsonPmpKkt,
            &hamiltonian_report,
            hamiltonian_drift_tolerance,
        )?;
        Ok(HermiteSimpsonFixedMeshDenseSqpPmpKktCertificateReport {
            sparse_kkt_step_report,
            costate_convention_report,
            hamiltonian_report,
            control_stationarity_report: None,
            costate_comparison_report: None,
            optimality_certificate,
        })
    }

    /// Build PMP/KKT certificate artifacts from an accepted sparse-KKT SQP report.
    ///
    /// This consumes the final sparse-KKT multipliers stored by the sparse SQP
    /// outer loop, applies the caller-selected PMP costate convention, evaluates
    /// the common Hamiltonian witness, and packages a validated I-load
    /// certificate. If the sparse SQP report carries a final constrained-KKT
    /// active-row report, the PMP witness uses those constrained multipliers;
    /// otherwise it uses the final augmented-KKT multipliers.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse SQP report is malformed, does not
    /// match this OCP layout, the PMP convention or callbacks are invalid, or the
    /// declared certificate tolerance is not met.
    pub fn sparse_kkt_sqp_pmp_kkt_certificate_report<D, L>(
        &self,
        report: &HermiteSimpsonFixedMeshSparseKktSqpReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        hamiltonian_drift_tolerance: f64,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<HermiteSimpsonFixedMeshSparseKktSqpPmpKktCertificateReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_decision_vector(&self.layout, &report.decision)?;
        if report.final_kkt_step_report.residual_report != report.residual_report {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse SQP final KKT report does not match the final residual report",
            });
        }

        let (costate_convention_report, hamiltonian_report, used_constrained_kkt_step_report) =
            if let Some(constrained_report) = &report.final_constrained_kkt_step_report {
                validate_sparse_constrained_kkt_step_report_for_ocp(
                    &self.layout,
                    constrained_report,
                )?;
                if constrained_report.residual_report != report.residual_report {
                    return Err(TrajoptError::InvalidPayload {
                        reason: "Hermite-Simpson sparse SQP final constrained KKT report does not match the final residual report",
                    });
                }
                (
                    self.sparse_constrained_kkt_pmp_costate_convention_report(
                        constrained_report,
                        convention,
                    )?,
                    self.sparse_constrained_kkt_pmp_hamiltonian_report(
                        &report.decision,
                        constrained_report,
                        convention,
                        dynamics,
                        lagrange,
                    )?,
                    true,
                )
            } else {
                (
                    self.sparse_kkt_pmp_costate_convention_report(
                        &report.final_kkt_step_report,
                        convention,
                    )?,
                    self.sparse_kkt_pmp_hamiltonian_report(
                        &report.decision,
                        &report.final_kkt_step_report,
                        convention,
                        dynamics,
                        lagrange,
                    )?,
                    false,
                )
            };

        let optimality_certificate = ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            ILoadOptimalityEvidenceKind::HermiteSimpsonPmpKkt,
            &hamiltonian_report,
            hamiltonian_drift_tolerance,
        )?;
        Ok(HermiteSimpsonFixedMeshSparseKktSqpPmpKktCertificateReport {
            sparse_kkt_step_report: report.final_kkt_step_report.clone(),
            final_constrained_kkt_step_report: report.final_constrained_kkt_step_report.clone(),
            used_constrained_kkt_step_report,
            costate_convention_report,
            hamiltonian_report,
            control_stationarity_report: None,
            costate_comparison_report: None,
            optimality_certificate,
        })
    }

    /// Assemble and solve one sparse Gauss-Newton step using forward-mode AD.
    ///
    /// This is the same sparse normal-equation path as
    /// [`Self::sparse_gauss_newton_step_report`], but the residual Jacobian is
    /// produced through [`Self::autodiff_sparse_residual_jacobian_report`].
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when residual/Jacobian assembly, sparse normal
    /// block assembly, or the CG solve fails.
    #[allow(clippy::too_many_arguments)]
    pub fn autodiff_sparse_gauss_newton_step_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        jacobian_drop_tolerance: f64,
        hessian_regularization: f64,
        hessian_drop_tolerance: f64,
        cg_max_iterations: usize,
        cg_residual_tolerance: f64,
    ) -> Result<HermiteSimpsonFixedMeshSparseGaussNewtonStepReport, TrajoptError>
    where
        D: CollocationAutoDiffDynamics,
    {
        let residual_report = {
            let mut residual_dynamics = CollocationAutoDiffDynamicsAdapter::new(
                BorrowedAutoDiffDynamics { dynamics },
                self.layout.state_dimension,
                self.layout.control_dimension,
            )?;
            self.residual_report(decision, &mut residual_dynamics)?
        };
        let sparse_jacobian = self.autodiff_sparse_residual_jacobian_report(
            decision,
            dynamics,
            jacobian_drop_tolerance,
        )?;
        assemble_sparse_gauss_newton_step_report(
            residual_report,
            sparse_jacobian,
            hessian_regularization,
            hessian_drop_tolerance,
            cg_max_iterations,
            cg_residual_tolerance,
        )
    }

    /// Evaluate the fixed-mesh residual Jacobian as CSC sparse storage using
    /// caller-supplied analytic dynamics Jacobians.
    ///
    /// The assembled rows match [`Self::residual_report`]: Hermite-Simpson
    /// collocation defects, boundary residuals, soft path residuals, and
    /// quadratic control objective residuals. The returned sparse report uses
    /// `finite_difference_step = 0.0` to identify analytic derivative assembly.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, dynamics Jacobian
    /// samples, residual shape, or sparse report are invalid.
    pub fn analytic_sparse_residual_jacobian_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        drop_tolerance: f64,
    ) -> Result<HermiteSimpsonSparseResidualJacobianReport, TrajoptError>
    where
        D: CollocationDynamicsJacobian,
    {
        if !drop_tolerance.is_finite() || drop_tolerance < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson analytic residual Jacobian drop tolerance must be finite and non-negative",
            });
        }
        validate_decision_vector(&self.layout, decision)?;

        let row_count = self.layout.segment_count * self.layout.state_dimension * 2
            + self.boundary_constraints.len()
            + self.path_constraints.len()
            + self.layout.segment_count * 3 * self.quadratic_control_costs.len();
        let column_count = self.layout.variable_count;
        let mut dense = vec![0.0; row_count * column_count];

        for segment_index in 0..self.layout.segment_count {
            let t0_s = self.time_nodes_s[segment_index];
            let t1_s = self.time_nodes_s[segment_index + 1];
            let tm_s = 0.5 * (t0_s + t1_s);
            let h = t1_s - t0_s;
            let x0 = self.layout.node_state(decision, segment_index)?;
            let xm = slice(
                decision,
                self.layout.midpoint_state_offset + segment_index * self.layout.state_dimension,
                self.layout.state_dimension,
            );
            let x1 = self.layout.node_state(decision, segment_index + 1)?;
            let u0 = slice(
                decision,
                self.layout.node_control_offset + segment_index * self.layout.control_dimension,
                self.layout.control_dimension,
            );
            let um = slice(
                decision,
                self.layout.midpoint_control_offset + segment_index * self.layout.control_dimension,
                self.layout.control_dimension,
            );
            let u1 = slice(
                decision,
                self.layout.node_control_offset
                    + (segment_index + 1) * self.layout.control_dimension,
                self.layout.control_dimension,
            );
            let f0 = dynamics.eval_with_jacobian(t0_s, x0, u0)?;
            let fm = dynamics.eval_with_jacobian(tm_s, xm, um)?;
            let f1 = dynamics.eval_with_jacobian(t1_s, x1, u1)?;
            validate_dynamics_jacobian_sample(
                &f0,
                self.layout.state_dimension,
                self.layout.control_dimension,
            )?;
            validate_dynamics_jacobian_sample(
                &fm,
                self.layout.state_dimension,
                self.layout.control_dimension,
            )?;
            validate_dynamics_jacobian_sample(
                &f1,
                self.layout.state_dimension,
                self.layout.control_dimension,
            )?;

            assemble_segment_analytic_jacobian(
                &self.layout,
                segment_index,
                h,
                &f0,
                &fm,
                &f1,
                &mut dense,
            )?;
        }

        let mut row = self.layout.segment_count * self.layout.state_dimension * 2;
        for constraint in &self.boundary_constraints {
            validate_boundary_constraint(&self.layout, constraint)?;
            let column = self.layout.node_state_offset
                + constraint.node_index * self.layout.state_dimension
                + constraint.component;
            dense[row * column_count + column] = 1.0;
            row += 1;
        }

        for constraint in &self.path_constraints {
            validate_path_constraint(&self.layout, constraint)?;
            let value = sample_scalar(
                &self.layout,
                decision,
                constraint.sample,
                constraint.variable,
            )?;
            let active = constraint.lower.is_some_and(|lower| value < lower)
                || constraint.upper.is_some_and(|upper| value > upper);
            if active {
                let column =
                    scalar_decision_index(&self.layout, constraint.sample, constraint.variable)?;
                dense[row * column_count + column] = constraint.weight.sqrt();
            }
            row += 1;
        }

        for segment_index in 0..self.layout.segment_count {
            let h = self.time_nodes_s[segment_index + 1] - self.time_nodes_s[segment_index];
            for cost in &self.quadratic_control_costs {
                validate_quadratic_control_cost(&self.layout, cost)?;
                let endpoint_scale = (cost.weight * h / 6.0).sqrt();
                let midpoint_scale = (cost.weight * 4.0 * h / 6.0).sqrt();
                let node0 = scalar_decision_index(
                    &self.layout,
                    HermiteSimpsonSamplePoint::Node(segment_index),
                    HermiteSimpsonScalarVariable::ControlComponent {
                        component: cost.component,
                    },
                )?;
                let midpoint = scalar_decision_index(
                    &self.layout,
                    HermiteSimpsonSamplePoint::Midpoint(segment_index),
                    HermiteSimpsonScalarVariable::ControlComponent {
                        component: cost.component,
                    },
                )?;
                let node1 = scalar_decision_index(
                    &self.layout,
                    HermiteSimpsonSamplePoint::Node(segment_index + 1),
                    HermiteSimpsonScalarVariable::ControlComponent {
                        component: cost.component,
                    },
                )?;
                dense[row * column_count + node0] = endpoint_scale;
                dense[(row + 1) * column_count + midpoint] = midpoint_scale;
                dense[(row + 2) * column_count + node1] = endpoint_scale;
                row += 3;
            }
        }

        if row != row_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson analytic residual Jacobian row count is inconsistent",
            });
        }

        sparse_report_from_dense(row_count, column_count, &dense, 0.0, drop_tolerance)
    }

    /// Evaluate the fixed-mesh residual Jacobian with forward-mode autodiff.
    ///
    /// The dynamics callback receives [`CollocationAutoDiffScalar`] inputs at
    /// each node and midpoint sample. This method seeds state/control variables
    /// automatically and reuses the analytic sparse-Jacobian assembly path.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, AD dynamics output, or
    /// sparse report are invalid.
    pub fn autodiff_sparse_residual_jacobian_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        drop_tolerance: f64,
    ) -> Result<HermiteSimpsonSparseResidualJacobianReport, TrajoptError>
    where
        D: CollocationAutoDiffDynamics,
    {
        let borrowed = BorrowedAutoDiffDynamics { dynamics };
        let mut adapter = CollocationAutoDiffDynamicsAdapter::new(
            borrowed,
            self.layout.state_dimension,
            self.layout.control_dimension,
        )?;
        self.analytic_sparse_residual_jacobian_report(decision, &mut adapter, drop_tolerance)
    }

    /// Evaluate the fixed-mesh residual Jacobian as CSC sparse storage using a
    /// deterministic greedy column coloring.
    ///
    /// The structural pattern is conservative for Hermite-Simpson local
    /// dynamics: each segment residual row is assumed to depend on all state and
    /// control variables at that segment's endpoint and midpoint samples.
    /// Columns sharing a color never share a structural residual row, so one
    /// central finite-difference pair per color recovers all colored columns.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, finite-difference
    /// settings, structural pattern, residual evaluation, or residual dimension
    /// stability is invalid.
    pub fn colored_sparse_residual_jacobian_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        finite_difference_step: f64,
        drop_tolerance: f64,
    ) -> Result<HermiteSimpsonColoredSparseResidualJacobianReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        validate_finite_difference_settings(finite_difference_step, drop_tolerance)?;
        validate_decision_vector(&self.layout, decision)?;

        let base_residual = self.residuals(decision, dynamics)?;
        let row_count = base_residual.len();
        let column_count = self.layout.variable_count;
        let column_rows = self.residual_jacobian_structural_column_rows(row_count)?;
        let structural_nonzero_count = column_rows.iter().map(Vec::len).sum::<usize>();
        let column_colors = greedy_column_colors(&column_rows, row_count)?;
        let color_count = column_colors
            .iter()
            .copied()
            .max()
            .map_or(0, |color| color + 1);
        let mut columns_by_color = vec![Vec::new(); color_count];
        for (column, color) in column_colors.iter().copied().enumerate() {
            columns_by_color[color].push(column);
        }

        let mut dense = vec![0.0; row_count * column_count];
        for columns in &columns_by_color {
            let mut plus = decision.to_vec();
            let mut minus = decision.to_vec();
            let mut steps = Vec::with_capacity(columns.len());
            for column in columns {
                let step = finite_difference_step * decision[*column].abs().max(1.0);
                plus[*column] += step;
                minus[*column] -= step;
                steps.push(step);
            }
            validate_decision_vector(&self.layout, &plus)?;
            validate_decision_vector(&self.layout, &minus)?;

            let plus_residual = self.residuals(&plus, dynamics)?;
            validate_residual_dimension(&plus_residual, row_count)?;
            let minus_residual = self.residuals(&minus, dynamics)?;
            validate_residual_dimension(&minus_residual, row_count)?;

            for (column, step) in columns.iter().zip(steps.iter()) {
                let scale = 2.0 * step;
                for row in &column_rows[*column] {
                    dense[*row * column_count + *column] =
                        (plus_residual[*row] - minus_residual[*row]) / scale;
                }
            }
        }

        let sparse_jacobian = sparse_report_from_dense(
            row_count,
            column_count,
            &dense,
            finite_difference_step,
            drop_tolerance,
        )?;
        let report = HermiteSimpsonColoredSparseResidualJacobianReport {
            sparse_jacobian,
            column_colors,
            color_count,
            residual_evaluation_count: 1 + 2 * color_count,
            structural_nonzero_count,
        };
        validate_colored_sparse_residual_jacobian_report(&report)?;
        Ok(report)
    }

    fn residual_jacobian_structural_column_rows(
        &self,
        row_count: usize,
    ) -> Result<Vec<Vec<usize>>, TrajoptError> {
        let mut column_rows = vec![Vec::new(); self.layout.variable_count];
        let collocation_rows = self.layout.segment_count * self.layout.state_dimension * 2;
        let expected_row_count = collocation_rows
            + self.boundary_constraints.len()
            + self.path_constraints.len()
            + self.layout.segment_count * 3 * self.quadratic_control_costs.len();
        if row_count != expected_row_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson residual structural pattern row count is inconsistent",
            });
        }

        for segment_index in 0..self.layout.segment_count {
            let columns = segment_structural_columns(&self.layout, segment_index);
            let row_start = segment_index * self.layout.state_dimension * 2;
            for row in row_start..row_start + self.layout.state_dimension * 2 {
                push_structural_row(&mut column_rows, row, &columns)?;
            }
        }

        let mut row = collocation_rows;
        for constraint in &self.boundary_constraints {
            validate_boundary_constraint(&self.layout, constraint)?;
            let column = self.layout.node_state_offset
                + constraint.node_index * self.layout.state_dimension
                + constraint.component;
            push_structural_row(&mut column_rows, row, &[column])?;
            row += 1;
        }

        for constraint in &self.path_constraints {
            validate_path_constraint(&self.layout, constraint)?;
            let column =
                scalar_decision_index(&self.layout, constraint.sample, constraint.variable)?;
            push_structural_row(&mut column_rows, row, &[column])?;
            row += 1;
        }

        for segment_index in 0..self.layout.segment_count {
            for cost in &self.quadratic_control_costs {
                validate_quadratic_control_cost(&self.layout, cost)?;
                let node0 = scalar_decision_index(
                    &self.layout,
                    HermiteSimpsonSamplePoint::Node(segment_index),
                    HermiteSimpsonScalarVariable::ControlComponent {
                        component: cost.component,
                    },
                )?;
                let midpoint = scalar_decision_index(
                    &self.layout,
                    HermiteSimpsonSamplePoint::Midpoint(segment_index),
                    HermiteSimpsonScalarVariable::ControlComponent {
                        component: cost.component,
                    },
                )?;
                let node1 = scalar_decision_index(
                    &self.layout,
                    HermiteSimpsonSamplePoint::Node(segment_index + 1),
                    HermiteSimpsonScalarVariable::ControlComponent {
                        component: cost.component,
                    },
                )?;
                push_structural_row(&mut column_rows, row, &[node0])?;
                push_structural_row(&mut column_rows, row + 1, &[midpoint])?;
                push_structural_row(&mut column_rows, row + 2, &[node1])?;
                row += 3;
            }
        }

        Ok(column_rows)
    }

    /// Merge hard path constraints into finite decision-variable bounds.
    ///
    /// The returned bounds are directly consumable by the dense SQP lane. This
    /// is possible because current hard path constraints are scalar state or
    /// control bounds at node/midpoint decision variables.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the default bounds are malformed, any hard
    /// path constraint is invalid, or the merged bounds are contradictory.
    pub fn decision_bounds_with_hard_path_constraints(
        &self,
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
    ) -> Result<HermiteSimpsonDecisionBoundsReport, TrajoptError> {
        validate_decision_bounds(&self.layout, default_lower_bounds, default_upper_bounds)?;
        let variable_count = self.layout.variable_count;
        let mut lower_bounds = default_lower_bounds.to_vec();
        let mut upper_bounds = default_upper_bounds.to_vec();
        let mut constrained_variables = vec![false; variable_count];
        let mut applied_bound_count = 0;

        for constraint in &self.hard_path_constraints {
            validate_hard_path_constraint(&self.layout, constraint)?;
            let column =
                scalar_decision_index(&self.layout, constraint.sample, constraint.variable)?;
            if let Some(lower) = constraint.lower {
                lower_bounds[column] = lower_bounds[column].max(lower);
                constrained_variables[column] = true;
                applied_bound_count += 1;
            }
            if let Some(upper) = constraint.upper {
                upper_bounds[column] = upper_bounds[column].min(upper);
                constrained_variables[column] = true;
                applied_bound_count += 1;
            }
            if lower_bounds[column] > upper_bounds[column] {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson hard path bounds conflict with decision bounds",
                });
            }
        }

        Ok(HermiteSimpsonDecisionBoundsReport {
            variable_count,
            lower_bounds,
            upper_bounds,
            applied_bound_count,
            constrained_variable_count: constrained_variables
                .iter()
                .filter(|constrained| **constrained)
                .count(),
        })
    }

    /// Solve the fixed-mesh equality-constrained quadratic-control QP.
    ///
    /// This wrapper forms a dense Clarabel QP from the collocation and boundary
    /// residual rows, Simpson-weighted quadratic control costs, and scalar hard
    /// path inequalities. The equality rows are linearized at the zero decision
    /// vector, so this is exact for affine dynamics/residual models and is not
    /// a general nonlinear NLP solve.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when quadratic controls are absent, unsupported
    /// soft/nonlinear path constraints are present, finite-difference Jacobian
    /// assembly fails, or Clarabel cannot solve the dense QP.
    #[cfg(feature = "sqp")]
    pub fn solve_dense_linearized_quadratic_control_qp<D>(
        &self,
        dynamics: &mut D,
        finite_difference_step: f64,
        drop_tolerance: f64,
    ) -> Result<HermiteSimpsonFixedMeshDenseQuadraticQpReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        if self.quadratic_control_costs.is_empty() {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson dense quadratic QP requires at least one control cost",
            });
        }
        if !self.path_constraints.is_empty() || !self.nonlinear_hard_path_constraints.is_empty() {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson dense quadratic QP supports only scalar hard path constraints",
            });
        }

        let zero_decision = vec![0.0; self.layout.variable_count];
        let zero_residual_report = self.residual_report(&zero_decision, dynamics)?;
        let equality_row_count = zero_residual_report.collocation_residual_count
            + zero_residual_report.boundary_residual_count;
        let equality_rhs = zero_residual_report.residuals[..equality_row_count]
            .iter()
            .map(|residual| -residual)
            .collect::<Vec<_>>();
        let jacobian = self.sparse_residual_jacobian_report(
            &zero_decision,
            dynamics,
            finite_difference_step,
            drop_tolerance,
        )?;
        let dense_jacobian = jacobian.to_dense_row_major()?;
        let equality_matrix =
            dense_jacobian[..equality_row_count * self.layout.variable_count].to_vec();
        let hard_path_inequality_report = self.hard_path_inequality_report()?;
        let problem = crate::sqp::DenseQpProblem::new(
            self.layout.variable_count,
            dense_quadratic_control_hessian(self)?,
            vec![0.0; self.layout.variable_count],
            equality_matrix,
            equality_rhs,
            hard_path_inequality_report.inequality_matrix.clone(),
            hard_path_inequality_report.inequality_rhs.clone(),
        )?;
        let qp_solution = crate::sqp::solve_dense_qp(&problem)?;
        let residual_report = self.residual_report(&qp_solution.primal, dynamics)?;
        let equality_residual_norm =
            residual_norm(&residual_report.residuals[..equality_row_count]);
        let max_hard_path_violation =
            hard_path_inequality_report.max_violation(&qp_solution.primal)?;

        Ok(HermiteSimpsonFixedMeshDenseQuadraticQpReport {
            qp_solution,
            residual_report,
            hard_path_inequality_report,
            equality_row_count,
            equality_residual_norm,
            max_hard_path_violation,
        })
    }

    /// Solve this fixed-mesh residual model with the dense SQP lane.
    ///
    /// Hard scalar path constraints are merged into the supplied default
    /// decision bounds before calling the solver. The returned report includes
    /// both the solver result and final residual/bound violation evidence.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when bound merging, residual evaluation, hard
    /// path evaluation, or the dense SQP solve fails.
    #[cfg(feature = "sqp")]
    pub fn solve_dense_sqp<D>(
        &self,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        solver: &crate::sqp::DenseSqpSolver,
        mut dynamics: D,
    ) -> Result<HermiteSimpsonFixedMeshDenseSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        let sqp_solution = solver.solve(
            initial_decision,
            &decision_bounds.lower_bounds,
            &decision_bounds.upper_bounds,
            |decision: &[f64]| self.residuals(decision, &mut dynamics),
        )?;
        let residual_report = self.residual_report(&sqp_solution.variables, &mut dynamics)?;
        let max_decision_bound_violation =
            decision_bounds.max_violation(&sqp_solution.variables)?;
        let hard_path_inequalities = self.hard_path_inequality_report()?;
        let max_hard_path_violation =
            hard_path_inequalities.max_violation(&sqp_solution.variables)?;

        Ok(HermiteSimpsonFixedMeshDenseSqpReport {
            sqp_solution,
            residual_report,
            decision_bounds,
            max_decision_bound_violation,
            max_hard_path_violation,
        })
    }

    /// Solve this fixed-mesh residual model with dense SQP while treating the
    /// final mesh time as a scalar decision variable.
    ///
    /// The mesh remains uniformly distributed between `t = 0` and the accepted
    /// final time. Hard scalar path bounds are still merged into decision
    /// bounds; the returned SQP variables are `[final_time_s, decision...]`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when final-time bounds are malformed, decision
    /// bounds cannot be merged, residual evaluation fails, or the dense SQP
    /// solve fails.
    #[cfg(feature = "sqp")]
    #[allow(clippy::too_many_arguments)]
    pub fn solve_dense_sqp_with_free_final_time<D>(
        &self,
        initial_final_time_s: f64,
        final_time_lower_s: f64,
        final_time_upper_s: f64,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        solver: &crate::sqp::DenseSqpSolver,
        dynamics: D,
    ) -> Result<HermiteSimpsonFixedMeshFreeTimeDenseSqpReport, TrajoptError>
    where
        D: CollocationDynamics + Clone,
    {
        validate_final_time_bounds(initial_final_time_s, final_time_lower_s, final_time_upper_s)?;
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        validate_dense_decision_vector(initial_decision, self.layout.variable_count)?;
        let mut initial = Vec::with_capacity(self.layout.variable_count + 1);
        initial.push(initial_final_time_s);
        initial.extend_from_slice(initial_decision);
        let mut lower = Vec::with_capacity(self.layout.variable_count + 1);
        lower.push(final_time_lower_s);
        lower.extend_from_slice(&decision_bounds.lower_bounds);
        let mut upper = Vec::with_capacity(self.layout.variable_count + 1);
        upper.push(final_time_upper_s);
        upper.extend_from_slice(&decision_bounds.upper_bounds);

        let final_dynamics = dynamics.clone();
        let sqp_solution = solver.solve(
            &initial,
            &lower,
            &upper,
            HermiteSimpsonFreeFinalTimeResidualModel {
                template: self.clone(),
                dynamics,
            },
        )?;
        let final_time_s = sqp_solution.variables[0];
        let decision = sqp_solution.variables[1..].to_vec();
        let accepted_ocp = self.with_uniform_final_time(final_time_s)?;
        let mut final_dynamics = final_dynamics;
        let residual_report = accepted_ocp.residual_report(&decision, &mut final_dynamics)?;
        let max_decision_bound_violation = decision_bounds.max_violation(&decision)?;
        let max_hard_path_violation = accepted_ocp
            .hard_path_inequality_report()?
            .max_violation(&decision)?;
        Ok(HermiteSimpsonFixedMeshFreeTimeDenseSqpReport {
            sqp_solution,
            final_time_s,
            decision,
            residual_report,
            decision_bounds,
            max_final_time_bound_violation: scalar_bound_violation(
                final_time_s,
                final_time_lower_s,
                final_time_upper_s,
            ),
            max_decision_bound_violation,
            max_hard_path_violation,
        })
    }

    /// Solve this fixed-mesh residual model with dense SQP while treating the
    /// final mesh time as both a scalar decision variable and a positive linear
    /// objective term.
    ///
    /// The mesh remains uniformly distributed between `t = 0` and the accepted
    /// final time. Hard scalar path bounds are still merged into decision
    /// bounds; the returned SQP variables are `[final_time_s, decision...]`.
    /// This is a linearized-equality-plus-linear-objective lane, not a full
    /// large-sparse constrained NLP backend.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when final-time bounds or objective weight are
    /// malformed, decision bounds cannot be merged, residual evaluation fails,
    /// or the dense SQP objective solve fails.
    #[cfg(feature = "sqp")]
    #[allow(clippy::too_many_arguments)]
    pub fn solve_dense_sqp_with_free_final_time_objective<D>(
        &self,
        initial_final_time_s: f64,
        final_time_lower_s: f64,
        final_time_upper_s: f64,
        final_time_objective_weight: f64,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        solver: &crate::sqp::DenseSqpSolver,
        dynamics: D,
    ) -> Result<HermiteSimpsonFixedMeshFreeTimeObjectiveDenseSqpReport, TrajoptError>
    where
        D: CollocationDynamics + Clone,
    {
        validate_final_time_bounds(initial_final_time_s, final_time_lower_s, final_time_upper_s)?;
        validate_final_time_objective_weight(final_time_objective_weight)?;
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        validate_dense_decision_vector(initial_decision, self.layout.variable_count)?;
        let mut initial = Vec::with_capacity(self.layout.variable_count + 1);
        initial.push(initial_final_time_s);
        initial.extend_from_slice(initial_decision);
        let mut lower = Vec::with_capacity(self.layout.variable_count + 1);
        lower.push(final_time_lower_s);
        lower.extend_from_slice(&decision_bounds.lower_bounds);
        let mut upper = Vec::with_capacity(self.layout.variable_count + 1);
        upper.push(final_time_upper_s);
        upper.extend_from_slice(&decision_bounds.upper_bounds);
        let mut linear_objective = vec![0.0; self.layout.variable_count + 1];
        linear_objective[0] = final_time_objective_weight;

        let final_dynamics = dynamics.clone();
        let sqp_solution = solver.solve_equalities_with_linear_objective(
            &initial,
            &lower,
            &upper,
            &linear_objective,
            HermiteSimpsonFreeFinalTimeResidualModel {
                template: self.clone(),
                dynamics,
            },
        )?;
        let final_time_s = sqp_solution.variables[0];
        let decision = sqp_solution.variables[1..].to_vec();
        let accepted_ocp = self.with_uniform_final_time(final_time_s)?;
        let mut final_dynamics = final_dynamics;
        let residual_report = accepted_ocp.residual_report(&decision, &mut final_dynamics)?;
        let max_decision_bound_violation = decision_bounds.max_violation(&decision)?;
        let max_hard_path_violation = accepted_ocp
            .hard_path_inequality_report()?
            .max_violation(&decision)?;
        Ok(HermiteSimpsonFixedMeshFreeTimeObjectiveDenseSqpReport {
            sqp_solution,
            final_time_objective_weight,
            final_time_s,
            decision,
            residual_report,
            decision_bounds,
            max_final_time_bound_violation: scalar_bound_violation(
                final_time_s,
                final_time_lower_s,
                final_time_upper_s,
            ),
            max_decision_bound_violation,
            max_hard_path_violation,
        })
    }

    /// Solve this fixed-mesh residual model with dense SQP and nonlinear hard
    /// path inequalities.
    ///
    /// Scalar hard path constraints are still merged into decision bounds.
    /// Nonlinear hard path constraints are evaluated as `g(z) <= 0`, linearized
    /// into each SQP QP subproblem, and reported at the accepted solution.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when bound merging, residual evaluation,
    /// nonlinear hard path evaluation, or the constrained dense SQP solve fails.
    #[cfg(feature = "sqp")]
    pub fn solve_dense_sqp_with_nonlinear_hard_path_constraints<D, P>(
        &self,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        solver: &crate::sqp::DenseSqpSolver,
        mut dynamics: D,
        mut path_function: P,
    ) -> Result<HermiteSimpsonFixedMeshDenseConstrainedSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
        P: CollocationPathFunction,
    {
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        let sqp_solution = solver.solve_with_inequalities(
            initial_decision,
            &decision_bounds.lower_bounds,
            &decision_bounds.upper_bounds,
            |decision: &[f64]| self.residuals(decision, &mut dynamics),
            |decision: &[f64]| {
                Ok(self
                    .nonlinear_hard_path_inequality_report(decision, &mut path_function)?
                    .inequality_values)
            },
        )?;
        let residual_report = self.residual_report(&sqp_solution.variables, &mut dynamics)?;
        let nonlinear_hard_path_report = self
            .nonlinear_hard_path_inequality_report(&sqp_solution.variables, &mut path_function)?;
        let max_decision_bound_violation =
            decision_bounds.max_violation(&sqp_solution.variables)?;
        let hard_path_inequalities = self.hard_path_inequality_report()?;
        let max_hard_path_violation =
            hard_path_inequalities.max_violation(&sqp_solution.variables)?;
        let max_nonlinear_hard_path_violation = nonlinear_hard_path_report.max_violation;

        Ok(HermiteSimpsonFixedMeshDenseConstrainedSqpReport {
            sqp_solution,
            residual_report,
            decision_bounds,
            nonlinear_hard_path_report,
            max_decision_bound_violation,
            max_hard_path_violation,
            max_nonlinear_hard_path_violation,
        })
    }

    /// Transcribe hard scalar path bounds to dense `Aineq * z <= bineq` rows.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when any hard path constraint is invalid.
    pub fn hard_path_inequality_report(
        &self,
    ) -> Result<HermiteSimpsonPathInequalityReport, TrajoptError> {
        let inequality_count = self
            .hard_path_constraints
            .iter()
            .map(HermiteSimpsonHardPathConstraint::inequality_count)
            .sum();
        let variable_count = self.layout.variable_count;
        let mut inequality_matrix = vec![0.0; inequality_count * variable_count];
        let mut inequality_rhs = Vec::with_capacity(inequality_count);
        let mut row = 0;

        for constraint in &self.hard_path_constraints {
            validate_hard_path_constraint(&self.layout, constraint)?;
            let column =
                scalar_decision_index(&self.layout, constraint.sample, constraint.variable)?;
            if let Some(upper) = constraint.upper {
                inequality_matrix[row * variable_count + column] = 1.0;
                inequality_rhs.push(upper);
                row += 1;
            }
            if let Some(lower) = constraint.lower {
                inequality_matrix[row * variable_count + column] = -1.0;
                inequality_rhs.push(-lower);
                row += 1;
            }
        }

        Ok(HermiteSimpsonPathInequalityReport {
            variable_count,
            inequality_count,
            inequality_matrix,
            inequality_rhs,
        })
    }

    /// Evaluate nonlinear hard path bounds as `g(z) <= 0` rows.
    ///
    /// For each path-function component, upper bounds emit `value - upper`
    /// rows and lower bounds emit `lower - value` rows. Constrained dense SQP
    /// and downstream sparse solvers consume this same row convention.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, constraint
    /// descriptors, path-function outputs, or report shape are invalid.
    pub fn nonlinear_hard_path_inequality_report<P>(
        &self,
        decision: &[f64],
        path_function: &mut P,
    ) -> Result<HermiteSimpsonNonlinearPathInequalityReport, TrajoptError>
    where
        P: CollocationPathFunction,
    {
        validate_decision_vector(&self.layout, decision)?;
        let mut path_values = Vec::new();
        let inequality_count = self
            .nonlinear_hard_path_constraints
            .iter()
            .map(HermiteSimpsonNonlinearHardPathConstraint::inequality_count)
            .sum();
        let mut inequality_values = Vec::with_capacity(inequality_count);

        for constraint in &self.nonlinear_hard_path_constraints {
            validate_nonlinear_hard_path_constraint(&self.layout, constraint)?;
            let (time_s, state, control) = sample_state_control(
                &self.layout,
                &self.time_nodes_s,
                decision,
                constraint.sample,
            )?;
            let values = path_function.eval(time_s, state, control)?;
            if values.len() != constraint.output_dimension() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson nonlinear path output dimension is inconsistent",
                });
            }
            validate_finite_slice(&values)?;
            for ((value, lower), upper) in values
                .iter()
                .zip(constraint.lower_bounds.iter())
                .zip(constraint.upper_bounds.iter())
            {
                if let Some(upper) = upper {
                    inequality_values.push(value - upper);
                }
                if let Some(lower) = lower {
                    inequality_values.push(lower - value);
                }
            }
            path_values.extend(values);
        }

        let max_violation = inequality_values
            .iter()
            .map(|value| value.max(0.0))
            .fold(0.0_f64, f64::max);
        let report = HermiteSimpsonNonlinearPathInequalityReport {
            variable_count: self.layout.variable_count,
            path_value_count: path_values.len(),
            inequality_count,
            path_values,
            inequality_values,
            max_violation,
        };
        validate_nonlinear_path_inequality_report(&report)?;
        Ok(report)
    }

    /// Evaluate the nonlinear hard path inequality Jacobian as CSC sparse
    /// storage.
    ///
    /// The derivative is computed with deterministic central finite
    /// differences and drops coefficients whose absolute value is at or below
    /// `drop_tolerance`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, finite-difference
    /// settings, path function, or inequality-row dimension stability is
    /// invalid.
    pub fn sparse_nonlinear_hard_path_inequality_jacobian_report<P>(
        &self,
        decision: &[f64],
        path_function: &mut P,
        finite_difference_step: f64,
        drop_tolerance: f64,
    ) -> Result<HermiteSimpsonSparsePathInequalityJacobianReport, TrajoptError>
    where
        P: CollocationPathFunction,
    {
        validate_finite_difference_settings(finite_difference_step, drop_tolerance)?;
        validate_decision_vector(&self.layout, decision)?;

        let base_report = self.nonlinear_hard_path_inequality_report(decision, path_function)?;
        let row_count = base_report.inequality_count;
        let column_count = self.layout.variable_count;
        let mut column_offsets = Vec::with_capacity(column_count + 1);
        let mut row_indices = Vec::new();
        let mut values = Vec::new();
        column_offsets.push(0);

        for column in 0..column_count {
            let step = finite_difference_step * decision[column].abs().max(1.0);
            let mut plus = decision.to_vec();
            let mut minus = decision.to_vec();
            plus[column] += step;
            minus[column] -= step;
            validate_decision_vector(&self.layout, &plus)?;
            validate_decision_vector(&self.layout, &minus)?;

            let plus_report = self.nonlinear_hard_path_inequality_report(&plus, path_function)?;
            validate_inequality_dimension(&plus_report.inequality_values, row_count)?;
            let minus_report = self.nonlinear_hard_path_inequality_report(&minus, path_function)?;
            validate_inequality_dimension(&minus_report.inequality_values, row_count)?;

            let scale = 2.0 * step;
            for row in 0..row_count {
                let coefficient = (plus_report.inequality_values[row]
                    - minus_report.inequality_values[row])
                    / scale;
                if coefficient.abs() > drop_tolerance {
                    row_indices.push(row);
                    values.push(coefficient);
                }
            }
            column_offsets.push(values.len());
        }

        let report = HermiteSimpsonSparsePathInequalityJacobianReport {
            row_count,
            column_count,
            column_offsets,
            row_indices,
            values,
            finite_difference_step,
            drop_tolerance,
        };
        validate_sparse_path_inequality_jacobian_report(&report)?;
        Ok(report)
    }
}

/// Evaluate one Hermite-Simpson segment.
///
/// # Errors
///
/// Returns [`TrajoptError`] when segment validation fails, dynamics returns an
/// invalid derivative, or the Lagrange callback returns a non-finite value.
pub fn evaluate_hermite_simpson_segment<D, L>(
    segment: &HermiteSimpsonSegment,
    dynamics: &mut D,
    lagrange: &mut L,
) -> Result<HermiteSimpsonSegmentReport, TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    validate_segment(segment)?;
    let h = segment.step_s();
    let f0 = dynamics.eval(segment.t0_s, &segment.x0, &segment.u0)?;
    let fm = dynamics.eval(segment.tm_s, &segment.xm, &segment.um)?;
    let f1 = dynamics.eval(segment.t1_s, &segment.x1, &segment.u1)?;
    validate_derivative(&f0, segment.state_dimension())?;
    validate_derivative(&fm, segment.state_dimension())?;
    validate_derivative(&f1, segment.state_dimension())?;

    let l0 = lagrange.eval(segment.t0_s, &segment.x0, &segment.u0)?;
    let lm = lagrange.eval(segment.tm_s, &segment.xm, &segment.um)?;
    let l1 = lagrange.eval(segment.t1_s, &segment.x1, &segment.u1)?;
    if !l0.is_finite() || !lm.is_finite() || !l1.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson Lagrange values must be finite",
        });
    }

    let mut dynamics_defect = vec![0.0; segment.state_dimension()];
    let mut midpoint_defect = vec![0.0; segment.state_dimension()];
    let mut sum_sq = 0.0;
    let mut max_abs = 0.0_f64;
    for index in 0..segment.state_dimension() {
        let dyn_defect = segment.x1[index]
            - segment.x0[index]
            - h * (f0[index] + 4.0 * fm[index] + f1[index]) / 6.0;
        let mid_defect = segment.xm[index]
            - 0.5 * (segment.x0[index] + segment.x1[index])
            - h * (f0[index] - f1[index]) / 8.0;
        dynamics_defect[index] = dyn_defect;
        midpoint_defect[index] = mid_defect;
        sum_sq += dyn_defect * dyn_defect + mid_defect * mid_defect;
        max_abs = max_abs.max(dyn_defect.abs()).max(mid_defect.abs());
    }

    Ok(HermiteSimpsonSegmentReport {
        step_s: h,
        dynamics_defect,
        midpoint_defect,
        simpson_lagrange_cost: h * (l0 + 4.0 * lm + l1) / 6.0,
        defect_norm: sum_sq.sqrt(),
        max_abs_defect: max_abs,
    })
}

/// Evaluate a fixed-mesh Hermite-Simpson transcription.
///
/// # Errors
///
/// Returns [`TrajoptError`] when no segments are supplied, dimensions disagree
/// across segments, or any segment evaluation fails.
pub fn evaluate_hermite_simpson_transcription<D, L>(
    segments: &[HermiteSimpsonSegment],
    mut dynamics: D,
    mut lagrange: L,
) -> Result<HermiteSimpsonTranscriptionReport, TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    let first = segments.first().ok_or(TrajoptError::InvalidPayload {
        reason: "Hermite-Simpson transcription requires at least one segment",
    })?;
    let state_dimension = first.state_dimension();
    let control_dimension = first.control_dimension();
    let mut defects = Vec::with_capacity(segments.len() * state_dimension * 2);
    let mut cost = 0.0;
    let mut dynamics_sum_sq = 0.0;
    let mut midpoint_sum_sq = 0.0;
    let mut max_abs = 0.0_f64;
    for segment in segments {
        if segment.state_dimension() != state_dimension
            || segment.control_dimension() != control_dimension
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson segment dimensions must be consistent",
            });
        }
        let report = evaluate_hermite_simpson_segment(segment, &mut dynamics, &mut lagrange)?;
        dynamics_sum_sq += report
            .dynamics_defect
            .iter()
            .map(|value| value * value)
            .sum::<f64>();
        midpoint_sum_sq += report
            .midpoint_defect
            .iter()
            .map(|value| value * value)
            .sum::<f64>();
        max_abs = max_abs.max(report.max_abs_defect);
        cost += report.simpson_lagrange_cost;
        defects.extend(report.dynamics_defect);
        defects.extend(report.midpoint_defect);
    }
    let dynamics_defect_norm = dynamics_sum_sq.sqrt();
    let midpoint_defect_norm = midpoint_sum_sq.sqrt();
    let total_defect_norm = (dynamics_sum_sq + midpoint_sum_sq).sqrt();
    Ok(HermiteSimpsonTranscriptionReport {
        defects,
        simpson_lagrange_cost: cost,
        dynamics_defect_norm,
        midpoint_defect_norm,
        total_defect_norm,
        max_abs_defect: max_abs,
        segment_count: segments.len(),
        state_dimension,
        control_dimension,
    })
}

fn validate_segment(segment: &HermiteSimpsonSegment) -> Result<(), TrajoptError> {
    if !segment.t0_s.is_finite() || !segment.tm_s.is_finite() || !segment.t1_s.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson segment times must be finite",
        });
    }
    if segment.t1_s <= segment.t0_s {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson segment end time must be greater than start time",
        });
    }
    let midpoint = 0.5 * (segment.t0_s + segment.t1_s);
    let midpoint_tolerance = 1.0e-12 * segment.t0_s.abs().max(segment.t1_s.abs()).max(1.0);
    if (segment.tm_s - midpoint).abs() > midpoint_tolerance {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson segment midpoint time must bisect the segment",
        });
    }
    if segment.x0.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson state dimension must be nonzero",
        });
    }
    if segment.xm.len() != segment.x0.len() || segment.x1.len() != segment.x0.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson state dimensions must agree",
        });
    }
    if segment.um.len() != segment.u0.len() || segment.u1.len() != segment.u0.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson control dimensions must agree",
        });
    }
    validate_finite_slice(&segment.x0)?;
    validate_finite_slice(&segment.xm)?;
    validate_finite_slice(&segment.x1)?;
    validate_finite_slice(&segment.u0)?;
    validate_finite_slice(&segment.um)?;
    validate_finite_slice(&segment.u1)
}

fn validate_derivative(derivative: &[f64], state_dimension: usize) -> Result<(), TrajoptError> {
    if derivative.len() != state_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson dynamics derivative dimension must match state dimension",
        });
    }
    validate_finite_slice(derivative)
}

fn validate_dynamics_jacobian_sample(
    sample: &CollocationDynamicsJacobianSample,
    state_dimension: usize,
    control_dimension: usize,
) -> Result<(), TrajoptError> {
    validate_derivative(&sample.derivative, state_dimension)?;
    if sample.state_jacobian.len() != state_dimension * state_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson dynamics state Jacobian dimensions are inconsistent",
        });
    }
    if sample.control_jacobian.len() != state_dimension * control_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson dynamics control Jacobian dimensions are inconsistent",
        });
    }
    validate_finite_slice(&sample.state_jacobian)?;
    validate_finite_slice(&sample.control_jacobian)
}

fn validate_finite_slice(values: &[f64]) -> Result<(), TrajoptError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson state/control values must be finite",
        })
    }
}

fn validate_autodiff_dimensions(
    state_dimension: usize,
    control_dimension: usize,
) -> Result<(), TrajoptError> {
    if state_dimension == 0 || control_dimension == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson autodiff dimensions must be nonzero",
        });
    }
    Ok(())
}

fn validate_autodiff_input_dimensions(
    state_dimension: usize,
    control_dimension: usize,
    state: &[f64],
    control: &[f64],
) -> Result<(), TrajoptError> {
    if state.len() != state_dimension || control.len() != control_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson autodiff input dimensions are inconsistent",
        });
    }
    validate_finite_slice(state)?;
    validate_finite_slice(control)
}

fn validate_autodiff_scalar(scalar: &CollocationAutoDiffScalar) -> Result<(), TrajoptError> {
    if !scalar.value.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson autodiff scalar value must be finite",
        });
    }
    if !scalar
        .derivatives
        .iter()
        .all(|derivative| derivative.is_finite())
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson autodiff scalar derivatives must be finite",
        });
    }
    Ok(())
}

fn validate_autodiff_output_dimension(
    output: &[CollocationAutoDiffScalar],
    state_dimension: usize,
) -> Result<(), TrajoptError> {
    if output.len() != state_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson autodiff dynamics output dimension must match state dimension",
        });
    }
    for scalar in output {
        validate_autodiff_scalar(scalar)?;
    }
    Ok(())
}

fn autodiff_derivative_at(
    scalar: &CollocationAutoDiffScalar,
    derivative_count: usize,
    index: usize,
) -> Result<f64, TrajoptError> {
    if scalar.derivatives.is_empty() {
        return Ok(0.0);
    }
    if scalar.derivatives.len() != derivative_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson autodiff derivative dimension is inconsistent",
        });
    }
    Ok(scalar.derivatives[index])
}

#[cfg(feature = "sqp")]
struct HermiteSimpsonFreeFinalTimeResidualModel<D> {
    template: HermiteSimpsonFixedMeshOcp,
    dynamics: D,
}

#[cfg(feature = "sqp")]
impl<D> crate::sqp::SqpResidualModel for HermiteSimpsonFreeFinalTimeResidualModel<D>
where
    D: CollocationDynamics,
{
    fn residual(&mut self, variables: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        if variables.len() != self.template.layout.variable_count + 1 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson free-final-time SQP vector length is inconsistent",
            });
        }
        let final_time_s = variables[0];
        let ocp = self.template.with_uniform_final_time(final_time_s)?;
        ocp.residuals(&variables[1..], &mut self.dynamics)
    }
}

fn validate_time_nodes(time_nodes_s: &[f64], segment_count: usize) -> Result<(), TrajoptError> {
    if time_nodes_s.len() != segment_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson fixed mesh time-node count is inconsistent",
        });
    }
    if !time_nodes_s.iter().all(|time| time.is_finite()) {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson fixed mesh times must be finite",
        });
    }
    if !time_nodes_s.windows(2).all(|window| window[0] < window[1]) {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson fixed mesh times must be strictly increasing",
        });
    }
    Ok(())
}

fn validate_decision_vector(
    layout: &HermiteSimpsonDecisionLayout,
    decision: &[f64],
) -> Result<(), TrajoptError> {
    if decision.len() != layout.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson decision vector length is inconsistent",
        });
    }
    validate_finite_slice(decision)
}

fn validate_dense_decision_vector(
    decision: &[f64],
    variable_count: usize,
) -> Result<(), TrajoptError> {
    if decision.len() != variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson dense inequality decision vector length is inconsistent",
        });
    }
    validate_finite_slice(decision)
}

#[cfg(feature = "sqp")]
fn validate_final_time_bounds(
    initial_final_time_s: f64,
    final_time_lower_s: f64,
    final_time_upper_s: f64,
) -> Result<(), TrajoptError> {
    if !initial_final_time_s.is_finite()
        || !final_time_lower_s.is_finite()
        || !final_time_upper_s.is_finite()
        || initial_final_time_s <= 0.0
        || final_time_lower_s <= 0.0
        || final_time_upper_s <= 0.0
        || final_time_lower_s > final_time_upper_s
        || initial_final_time_s < final_time_lower_s
        || initial_final_time_s > final_time_upper_s
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson free-final-time bounds are inconsistent",
        });
    }
    Ok(())
}

#[cfg(feature = "sqp")]
fn validate_final_time_objective_weight(weight: f64) -> Result<(), TrajoptError> {
    if !weight.is_finite() || weight <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson final-time objective weight must be finite and positive",
        });
    }
    Ok(())
}

#[cfg(feature = "sqp")]
fn scalar_bound_violation(value: f64, lower: f64, upper: f64) -> f64 {
    (lower - value).max(0.0).max((value - upper).max(0.0))
}

fn validate_decision_bounds(
    layout: &HermiteSimpsonDecisionLayout,
    lower_bounds: &[f64],
    upper_bounds: &[f64],
) -> Result<(), TrajoptError> {
    if lower_bounds.len() != layout.variable_count || upper_bounds.len() != layout.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson decision bounds must match variable count",
        });
    }
    validate_finite_slice(lower_bounds)?;
    validate_finite_slice(upper_bounds)?;
    if !lower_bounds
        .iter()
        .zip(upper_bounds.iter())
        .all(|(lower, upper)| lower <= upper)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson decision lower bounds must not exceed upper bounds",
        });
    }
    Ok(())
}

fn validate_boundary_constraint(
    layout: &HermiteSimpsonDecisionLayout,
    constraint: &HermiteSimpsonBoundaryConstraint,
) -> Result<(), TrajoptError> {
    if !constraint.target.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson boundary target must be finite",
        });
    }
    if constraint.node_index > layout.segment_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson boundary node is out of range",
        });
    }
    if constraint.component >= layout.state_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson boundary component is out of range",
        });
    }
    Ok(())
}

fn validate_path_bounds(
    lower: Option<f64>,
    upper: Option<f64>,
    label: &'static str,
) -> Result<(), TrajoptError> {
    if lower.is_none() && upper.is_none() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson path constraint requires at least one bound",
        });
    }
    if let Some(lower) = lower
        && !lower.is_finite()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: match label {
                "Hermite-Simpson hard path" => {
                    "Hermite-Simpson hard path lower bound must be finite"
                }
                _ => "Hermite-Simpson soft path lower bound must be finite",
            },
        });
    }
    if let Some(upper) = upper
        && !upper.is_finite()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: match label {
                "Hermite-Simpson hard path" => {
                    "Hermite-Simpson hard path upper bound must be finite"
                }
                _ => "Hermite-Simpson soft path upper bound must be finite",
            },
        });
    }
    if let (Some(lower), Some(upper)) = (lower, upper)
        && lower > upper
    {
        return Err(TrajoptError::InvalidPayload {
            reason: match label {
                "Hermite-Simpson hard path" => {
                    "Hermite-Simpson hard path lower bound must not exceed upper bound"
                }
                _ => "Hermite-Simpson soft path lower bound must not exceed upper bound",
            },
        });
    }
    Ok(())
}

fn validate_nonlinear_path_bounds(
    constraint: &HermiteSimpsonNonlinearHardPathConstraint,
) -> Result<(), TrajoptError> {
    if constraint.lower_bounds.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson nonlinear hard path dimension must be nonzero",
        });
    }
    if constraint.lower_bounds.len() != constraint.upper_bounds.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson nonlinear hard path bound dimensions are inconsistent",
        });
    }
    let mut has_bound = false;
    for (lower, upper) in constraint
        .lower_bounds
        .iter()
        .zip(constraint.upper_bounds.iter())
    {
        if let Some(lower) = lower {
            has_bound = true;
            if !lower.is_finite() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson nonlinear hard path lower bound must be finite",
                });
            }
        }
        if let Some(upper) = upper {
            has_bound = true;
            if !upper.is_finite() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson nonlinear hard path upper bound must be finite",
                });
            }
        }
        if let (Some(lower), Some(upper)) = (lower, upper)
            && lower > upper
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson nonlinear hard path lower bound must not exceed upper bound",
            });
        }
    }
    if !has_bound {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson nonlinear hard path requires at least one bound",
        });
    }
    Ok(())
}

fn validate_hard_path_constraint(
    layout: &HermiteSimpsonDecisionLayout,
    constraint: &HermiteSimpsonHardPathConstraint,
) -> Result<(), TrajoptError> {
    validate_path_bounds(
        constraint.lower,
        constraint.upper,
        "Hermite-Simpson hard path",
    )?;
    validate_scalar_sample(layout, constraint.sample, constraint.variable)
}

fn validate_nonlinear_hard_path_constraint(
    layout: &HermiteSimpsonDecisionLayout,
    constraint: &HermiteSimpsonNonlinearHardPathConstraint,
) -> Result<(), TrajoptError> {
    validate_nonlinear_path_bounds(constraint)?;
    validate_sample(layout, constraint.sample)
}

fn validate_path_constraint(
    layout: &HermiteSimpsonDecisionLayout,
    constraint: &HermiteSimpsonSoftPathConstraint,
) -> Result<(), TrajoptError> {
    validate_path_bounds(
        constraint.lower,
        constraint.upper,
        "Hermite-Simpson soft path",
    )?;
    validate_scalar_sample(layout, constraint.sample, constraint.variable)
}

fn validate_quadratic_control_cost(
    layout: &HermiteSimpsonDecisionLayout,
    cost: &HermiteSimpsonQuadraticControlCost,
) -> Result<(), TrajoptError> {
    if cost.component >= layout.control_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson control-cost component is out of range",
        });
    }
    if !cost.weight.is_finite() || cost.weight <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson control-cost weight must be finite and positive",
        });
    }
    Ok(())
}

fn validate_scalar_variable(
    layout: &HermiteSimpsonDecisionLayout,
    variable: HermiteSimpsonScalarVariable,
) -> Result<(), TrajoptError> {
    match variable {
        HermiteSimpsonScalarVariable::StateComponent { component } => {
            if component >= layout.state_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson soft path state component is out of range",
                });
            }
        }
        HermiteSimpsonScalarVariable::ControlComponent { component } => {
            if component >= layout.control_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson soft path control component is out of range",
                });
            }
        }
    }
    Ok(())
}

fn validate_sample(
    layout: &HermiteSimpsonDecisionLayout,
    sample: HermiteSimpsonSamplePoint,
) -> Result<(), TrajoptError> {
    match sample {
        HermiteSimpsonSamplePoint::Node(index) => {
            if index > layout.segment_count {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson path node is out of range",
                });
            }
        }
        HermiteSimpsonSamplePoint::Midpoint(index) => {
            if index >= layout.segment_count {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson path midpoint is out of range",
                });
            }
        }
    }
    Ok(())
}

fn validate_scalar_sample(
    layout: &HermiteSimpsonDecisionLayout,
    sample: HermiteSimpsonSamplePoint,
    variable: HermiteSimpsonScalarVariable,
) -> Result<(), TrajoptError> {
    validate_sample(layout, sample)?;
    validate_scalar_variable(layout, variable)
}

fn validate_reference_profile_mapping(
    layout: &HermiteSimpsonDecisionLayout,
    mapping: HermiteSimpsonReferenceProfileMapping,
) -> Result<(), TrajoptError> {
    if mapping.radius_component >= layout.state_dimension
        || mapping.speed_component >= layout.state_dimension
        || mapping.flight_path_angle_component >= layout.state_dimension
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson reference profile mapping component is out of range",
        });
    }
    Ok(())
}

fn validate_reference_profile_report(
    report: &HermiteSimpsonReferenceProfileReport,
) -> Result<(), TrajoptError> {
    if report.sample_count == 0 || report.samples.len() != report.sample_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson reference profile sample count is inconsistent",
        });
    }
    if report.start_time_s.to_bits() != report.samples[0].time_s.to_bits()
        || report.end_time_s.to_bits() != report.samples[report.samples.len() - 1].time_s.to_bits()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson reference profile time endpoints are inconsistent",
        });
    }
    if !report
        .samples
        .windows(2)
        .all(|window| window[0].time_s < window[1].time_s)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson reference profile times must be strictly increasing",
        });
    }
    for sample in &report.samples {
        validate_reference_profile_sample(sample)?;
    }
    Ok(())
}

fn validate_path_inequality_report(
    report: &HermiteSimpsonPathInequalityReport,
) -> Result<(), TrajoptError> {
    if report.variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson path inequality variable count must be nonzero",
        });
    }
    if report.inequality_matrix.len() != report.inequality_count * report.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson path inequality matrix dimensions are inconsistent",
        });
    }
    if report.inequality_rhs.len() != report.inequality_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson path inequality RHS dimensions are inconsistent",
        });
    }
    validate_finite_slice(&report.inequality_matrix)?;
    validate_finite_slice(&report.inequality_rhs)
}

fn validate_nonlinear_path_inequality_report(
    report: &HermiteSimpsonNonlinearPathInequalityReport,
) -> Result<(), TrajoptError> {
    if report.variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson nonlinear path inequality variable count must be nonzero",
        });
    }
    if report.path_values.len() != report.path_value_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson nonlinear path value count is inconsistent",
        });
    }
    if report.inequality_values.len() != report.inequality_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson nonlinear path inequality count is inconsistent",
        });
    }
    validate_finite_slice(&report.path_values)?;
    validate_finite_slice(&report.inequality_values)?;
    let max_violation = report
        .inequality_values
        .iter()
        .map(|value| value.max(0.0))
        .fold(0.0_f64, f64::max);
    if !report.max_violation.is_finite()
        || (report.max_violation - max_violation).abs() > 1.0e-12 * max_violation.abs().max(1.0)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson nonlinear path max violation is inconsistent",
        });
    }
    Ok(())
}

fn validate_finite_difference_settings(
    finite_difference_step: f64,
    drop_tolerance: f64,
) -> Result<(), TrajoptError> {
    if !finite_difference_step.is_finite() || finite_difference_step <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse residual Jacobian step must be finite and positive",
        });
    }
    if !drop_tolerance.is_finite() || drop_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse residual Jacobian drop tolerance must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_residual_dimension(residuals: &[f64], row_count: usize) -> Result<(), TrajoptError> {
    if residuals.len() != row_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse residual Jacobian residual dimension changed",
        });
    }
    validate_finite_slice(residuals)
}

fn validate_inequality_dimension(
    inequality_values: &[f64],
    row_count: usize,
) -> Result<(), TrajoptError> {
    if inequality_values.len() != row_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse path inequality Jacobian row dimension changed",
        });
    }
    validate_finite_slice(inequality_values)
}

fn validate_sparse_residual_jacobian_report(
    report: &HermiteSimpsonSparseResidualJacobianReport,
) -> Result<(), TrajoptError> {
    if report.column_count == 0 || report.row_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse residual Jacobian dimensions must be nonzero",
        });
    }
    if report.column_offsets.len() != report.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse residual Jacobian column offsets are inconsistent",
        });
    }
    if report.column_offsets.first() != Some(&0)
        || report.column_offsets.last() != Some(&report.values.len())
        || !report
            .column_offsets
            .windows(2)
            .all(|window| window[0] <= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse residual Jacobian column offsets are invalid",
        });
    }
    if report.row_indices.len() != report.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse residual Jacobian row/value lengths differ",
        });
    }
    if !report.row_indices.iter().all(|row| *row < report.row_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse residual Jacobian row index is out of range",
        });
    }
    validate_sparse_residual_jacobian_metadata(
        report.finite_difference_step,
        report.drop_tolerance,
    )?;
    validate_finite_slice(&report.values)
}

fn validate_sparse_residual_jacobian_metadata(
    finite_difference_step: f64,
    drop_tolerance: f64,
) -> Result<(), TrajoptError> {
    if !finite_difference_step.is_finite() || finite_difference_step < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse residual Jacobian step metadata must be finite and non-negative",
        });
    }
    if !drop_tolerance.is_finite() || drop_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse residual Jacobian drop tolerance must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_sparse_hessian_settings(
    regularization: f64,
    drop_tolerance: f64,
) -> Result<(), TrajoptError> {
    if !regularization.is_finite() || regularization < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton Hessian regularization must be finite and non-negative",
        });
    }
    if !drop_tolerance.is_finite() || drop_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton Hessian drop tolerance must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_sparse_augmented_kkt_settings(
    regularization: f64,
    drop_tolerance: f64,
) -> Result<(), TrajoptError> {
    if !regularization.is_finite() || regularization <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse augmented KKT regularization must be finite and positive",
        });
    }
    if !drop_tolerance.is_finite() || drop_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse augmented KKT drop tolerance must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_sparse_gauss_newton_hessian_report(
    report: &HermiteSimpsonSparseGaussNewtonHessianReport,
) -> Result<(), TrajoptError> {
    if report.row_count == 0 || report.column_count == 0 || report.row_count != report.column_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton Hessian dimensions must be square and nonzero",
        });
    }
    if report.residual_row_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton Hessian source residual count must be nonzero",
        });
    }
    if report.column_offsets.len() != report.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton Hessian column offsets are inconsistent",
        });
    }
    if report.column_offsets.first() != Some(&0)
        || report.column_offsets.last() != Some(&report.values.len())
        || !report
            .column_offsets
            .windows(2)
            .all(|window| window[0] <= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton Hessian column offsets are invalid",
        });
    }
    if report.row_indices.len() != report.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton Hessian row/value lengths differ",
        });
    }
    if !report.row_indices.iter().all(|row| *row < report.row_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton Hessian row index is out of range",
        });
    }
    validate_sparse_hessian_settings(report.regularization, report.drop_tolerance)?;
    validate_finite_slice(&report.values)
}

fn validate_sparse_augmented_kkt_report(
    report: &HermiteSimpsonSparseAugmentedKktReport,
) -> Result<(), TrajoptError> {
    if report.row_count == 0 || report.column_count == 0 || report.row_count != report.column_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse augmented KKT dimensions must be square and nonzero",
        });
    }
    if report.variable_count == 0 || report.residual_row_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse augmented KKT source dimensions must be nonzero",
        });
    }
    if report.row_count != report.variable_count + report.residual_row_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse augmented KKT dimensions are inconsistent",
        });
    }
    if report.column_offsets.len() != report.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse augmented KKT column offsets are inconsistent",
        });
    }
    if report.column_offsets.first() != Some(&0)
        || report.column_offsets.last() != Some(&report.values.len())
        || !report
            .column_offsets
            .windows(2)
            .all(|window| window[0] <= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse augmented KKT column offsets are invalid",
        });
    }
    if report.row_indices.len() != report.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse augmented KKT row/value lengths differ",
        });
    }
    if !report.row_indices.iter().all(|row| *row < report.row_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse augmented KKT row index is out of range",
        });
    }
    validate_sparse_augmented_kkt_settings(report.regularization, report.drop_tolerance)?;
    validate_finite_slice(&report.values)
}

fn validate_sparse_constrained_kkt_report(
    report: &HermiteSimpsonSparseConstrainedKktReport,
) -> Result<(), TrajoptError> {
    if report.row_count == 0 || report.column_count == 0 || report.row_count != report.column_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT dimensions must be square and nonzero",
        });
    }
    if report.variable_count == 0
        || report.residual_row_count == 0
        || report.active_inequality_row_count == 0
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT source dimensions must be nonzero",
        });
    }
    if report.row_count
        != report.variable_count + report.residual_row_count + report.active_inequality_row_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT dimensions are inconsistent",
        });
    }
    if report.active_inequality_rows.len() != report.active_inequality_row_count
        || report.active_inequality_values.len() != report.active_inequality_row_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT active rows are inconsistent",
        });
    }
    if report
        .active_inequality_rows
        .windows(2)
        .any(|window| window[0] >= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT active rows must be strictly increasing",
        });
    }
    if report.source_jacobian_nonzero_count == 0 || report.source_inequality_nonzero_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT source nonzero counts must be nonzero",
        });
    }
    if report.column_offsets.len() != report.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT column offsets are inconsistent",
        });
    }
    if report.column_offsets.first() != Some(&0)
        || report.column_offsets.last() != Some(&report.values.len())
        || !report
            .column_offsets
            .windows(2)
            .all(|window| window[0] <= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT column offsets are invalid",
        });
    }
    if report.row_indices.len() != report.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT row/value lengths differ",
        });
    }
    if !report.row_indices.iter().all(|row| *row < report.row_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT row index is out of range",
        });
    }
    validate_sparse_augmented_kkt_settings(report.regularization, report.drop_tolerance)?;
    if !report.active_tolerance.is_finite() || report.active_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT active tolerance must be finite and non-negative",
        });
    }
    validate_finite_slice(&report.active_inequality_values)?;
    validate_finite_slice(&report.values)
}

fn validate_sparse_constrained_kkt_step_report_for_ocp(
    layout: &HermiteSimpsonDecisionLayout,
    step_report: &HermiteSimpsonFixedMeshSparseConstrainedKktStepReport,
) -> Result<(), TrajoptError> {
    validate_sparse_residual_jacobian_report(&step_report.sparse_jacobian)?;
    validate_sparse_constrained_kkt_report(&step_report.constrained_kkt)?;
    validate_path_inequality_report(&step_report.hard_path_inequality_report)?;
    if step_report.sparse_jacobian.column_count != layout.variable_count
        || step_report.constrained_kkt.variable_count != layout.variable_count
        || step_report.hard_path_inequality_report.variable_count != layout.variable_count
        || step_report.constrained_kkt.residual_row_count != step_report.sparse_jacobian.row_count
        || step_report.sparse_jacobian.row_count != step_report.residual_report.residuals.len()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson constrained sparse KKT step report does not match the OCP layout",
        });
    }
    if step_report
        .constrained_kkt
        .active_inequality_rows
        .iter()
        .any(|row| *row >= step_report.hard_path_inequality_report.inequality_count)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson constrained sparse KKT active row index is invalid",
        });
    }
    if step_report.rhs.len() != step_report.constrained_kkt.row_count
        || step_report.solve_report.solution.len() != step_report.constrained_kkt.row_count
        || step_report.step.len() != step_report.sparse_jacobian.column_count
        || step_report.residual_multipliers.len() != step_report.residual_report.residuals.len()
        || step_report.active_inequality_multipliers.len()
            != step_report.constrained_kkt.active_inequality_row_count
        || step_report.linearized_residual.len() != step_report.residual_report.residuals.len()
        || step_report.linearized_active_inequality_values.len()
            != step_report.constrained_kkt.active_inequality_row_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson constrained sparse KKT step report dimensions are inconsistent",
        });
    }
    validate_finite_slice(&step_report.rhs)?;
    validate_finite_slice(&step_report.step)?;
    validate_finite_slice(&step_report.residual_multipliers)?;
    validate_finite_slice(&step_report.active_inequality_multipliers)?;
    validate_finite_slice(&step_report.linearized_residual)?;
    validate_finite_slice(&step_report.linearized_active_inequality_values)
}

fn validate_sparse_cg_settings(
    report: &HermiteSimpsonSparseGaussNewtonHessianReport,
    rhs: &[f64],
    initial_solution: &[f64],
    max_iterations: usize,
    residual_tolerance: f64,
) -> Result<(), TrajoptError> {
    validate_sparse_gauss_newton_hessian_report(report)?;
    if rhs.len() != report.row_count || initial_solution.len() != report.column_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton CG dimensions are inconsistent",
        });
    }
    if max_iterations == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton CG iteration limit must be positive",
        });
    }
    if !residual_tolerance.is_finite() || residual_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse Gauss-Newton CG tolerance must be finite and non-negative",
        });
    }
    validate_finite_slice(rhs)?;
    validate_finite_slice(initial_solution)
}

fn validate_colored_sparse_residual_jacobian_report(
    report: &HermiteSimpsonColoredSparseResidualJacobianReport,
) -> Result<(), TrajoptError> {
    validate_sparse_residual_jacobian_report(&report.sparse_jacobian)?;
    if report.column_colors.len() != report.sparse_jacobian.column_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson colored sparse residual Jacobian colors must match columns",
        });
    }
    if report.color_count == 0
        || report
            .column_colors
            .iter()
            .any(|color| *color >= report.color_count)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson colored sparse residual Jacobian colors are inconsistent",
        });
    }
    if report.residual_evaluation_count != 1 + 2 * report.color_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson colored sparse residual Jacobian evaluation count is inconsistent",
        });
    }
    if report.structural_nonzero_count < report.sparse_jacobian.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson colored sparse residual Jacobian structural count is inconsistent",
        });
    }
    Ok(())
}

fn validate_sparse_path_inequality_jacobian_report(
    report: &HermiteSimpsonSparsePathInequalityJacobianReport,
) -> Result<(), TrajoptError> {
    if report.column_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse path inequality Jacobian column count must be nonzero",
        });
    }
    if report.column_offsets.len() != report.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse path inequality Jacobian column offsets are inconsistent",
        });
    }
    if report.column_offsets.first() != Some(&0)
        || report.column_offsets.last() != Some(&report.values.len())
        || !report
            .column_offsets
            .windows(2)
            .all(|window| window[0] <= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse path inequality Jacobian column offsets are invalid",
        });
    }
    if report.row_indices.len() != report.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse path inequality Jacobian row/value lengths differ",
        });
    }
    if !report.row_indices.iter().all(|row| *row < report.row_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse path inequality Jacobian row index is out of range",
        });
    }
    validate_finite_difference_settings(report.finite_difference_step, report.drop_tolerance)?;
    validate_finite_slice(&report.values)
}

#[derive(Clone, Debug)]
struct DenseLdltFactorization {
    unit_lower: Vec<f64>,
    diagonal: Vec<f64>,
    factorization_residual_norm: f64,
}

impl DenseLdltFactorization {
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        let dimension = self.diagonal.len();
        if rhs.len() != dimension {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson LDL RHS length is inconsistent",
            });
        }
        validate_finite_slice(rhs)?;

        let mut y = vec![0.0; dimension];
        for row in 0..dimension {
            let mut sum = rhs[row];
            for (column, y_column) in y.iter().enumerate().take(row) {
                sum -= self.unit_lower[row * dimension + column] * *y_column;
            }
            y[row] = sum;
        }

        let mut z = vec![0.0; dimension];
        for row in 0..dimension {
            let diagonal = self.diagonal[row];
            if !diagonal.is_finite() || diagonal.abs() <= 1.0e-14 {
                return Err(TrajoptError::SingularSystem {
                    reason: "Hermite-Simpson LDL solve detected a singular diagonal",
                });
            }
            z[row] = y[row] / diagonal;
        }

        let mut solution = vec![0.0; dimension];
        for row in (0..dimension).rev() {
            let mut sum = z[row];
            for (column, solution_column) in
                solution.iter().enumerate().take(dimension).skip(row + 1)
            {
                sum -= self.unit_lower[column * dimension + row] * *solution_column;
            }
            solution[row] = sum;
        }
        validate_finite_slice(&solution)?;
        Ok(solution)
    }
}

fn dense_ldlt_factorization(
    dense: &[f64],
    dimension: usize,
) -> Result<DenseLdltFactorization, TrajoptError> {
    if dense.len() != dimension * dimension || dimension == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson LDL factorization matrix dimensions are inconsistent",
        });
    }
    validate_finite_slice(dense)?;

    let mut unit_lower = vec![0.0; dimension * dimension];
    let mut diagonal = vec![0.0; dimension];
    for index in 0..dimension {
        unit_lower[index * dimension + index] = 1.0;
    }

    for pivot in 0..dimension {
        for column in 0..pivot {
            let mut value = dense[pivot * dimension + column];
            for inner in 0..column {
                value -= unit_lower[pivot * dimension + inner]
                    * diagonal[inner]
                    * unit_lower[column * dimension + inner];
            }
            let column_diagonal = diagonal[column];
            if !column_diagonal.is_finite() || column_diagonal.abs() <= 1.0e-14 {
                return Err(TrajoptError::SingularSystem {
                    reason: "Hermite-Simpson LDL factorization detected a singular pivot",
                });
            }
            unit_lower[pivot * dimension + column] = value / column_diagonal;
        }

        let mut pivot_diagonal = dense[pivot * dimension + pivot];
        for inner in 0..pivot {
            let lower = unit_lower[pivot * dimension + inner];
            pivot_diagonal -= lower * lower * diagonal[inner];
        }
        if !pivot_diagonal.is_finite() || pivot_diagonal.abs() <= 1.0e-14 {
            return Err(TrajoptError::SingularSystem {
                reason: "Hermite-Simpson LDL factorization detected a singular pivot",
            });
        }
        diagonal[pivot] = pivot_diagonal;
    }

    let factorization_residual_norm =
        dense_ldlt_factorization_residual_norm(dense, dimension, &unit_lower, &diagonal);
    if !factorization_residual_norm.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson LDL factorization residual is non-finite",
        });
    }

    Ok(DenseLdltFactorization {
        unit_lower,
        diagonal,
        factorization_residual_norm,
    })
}

fn dense_ldlt_factorization_residual_norm(
    dense: &[f64],
    dimension: usize,
    unit_lower: &[f64],
    diagonal: &[f64],
) -> f64 {
    let mut squared_norm = 0.0;
    for row in 0..dimension {
        for column in 0..dimension {
            let mut reconstructed = 0.0;
            let shared = row.min(column);
            for inner in 0..=shared {
                reconstructed += unit_lower[row * dimension + inner]
                    * diagonal[inner]
                    * unit_lower[column * dimension + inner];
            }
            let residual = reconstructed - dense[row * dimension + column];
            squared_norm += residual * residual;
        }
    }
    squared_norm.sqrt()
}

fn sparse_report_from_dense(
    row_count: usize,
    column_count: usize,
    dense: &[f64],
    finite_difference_step: f64,
    drop_tolerance: f64,
) -> Result<HermiteSimpsonSparseResidualJacobianReport, TrajoptError> {
    if dense.len() != row_count * column_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson dense residual Jacobian dimensions are inconsistent",
        });
    }
    validate_finite_slice(dense)?;
    let mut column_offsets = Vec::with_capacity(column_count + 1);
    let mut row_indices = Vec::new();
    let mut values = Vec::new();
    column_offsets.push(0);
    for column in 0..column_count {
        for row in 0..row_count {
            let coefficient = dense[row * column_count + column];
            if coefficient.abs() > drop_tolerance {
                row_indices.push(row);
                values.push(coefficient);
            }
        }
        column_offsets.push(values.len());
    }
    let report = HermiteSimpsonSparseResidualJacobianReport {
        row_count,
        column_count,
        column_offsets,
        row_indices,
        values,
        finite_difference_step,
        drop_tolerance,
    };
    validate_sparse_residual_jacobian_report(&report)?;
    Ok(report)
}

fn assemble_sparse_gauss_newton_step_report(
    residual_report: HermiteSimpsonFixedMeshResidualReport,
    sparse_jacobian: HermiteSimpsonSparseResidualJacobianReport,
    hessian_regularization: f64,
    hessian_drop_tolerance: f64,
    cg_max_iterations: usize,
    cg_residual_tolerance: f64,
) -> Result<HermiteSimpsonFixedMeshSparseGaussNewtonStepReport, TrajoptError> {
    validate_sparse_residual_jacobian_report(&sparse_jacobian)?;
    validate_residual_dimension(&residual_report.residuals, sparse_jacobian.row_count)?;
    let normal_hessian = sparse_jacobian
        .sparse_gauss_newton_hessian_report(hessian_regularization, hessian_drop_tolerance)?;
    let rhs = sparse_jacobian.sparse_gauss_newton_step_rhs(&residual_report.residuals)?;
    let initial_step = vec![0.0; sparse_jacobian.column_count];
    let solve_report = normal_hessian.solve_conjugate_gradient(
        &rhs,
        &initial_step,
        cg_max_iterations,
        cg_residual_tolerance,
    )?;
    let jacobian_step = sparse_jacobian.multiply_vector(&solve_report.solution)?;
    let linearized_residual = residual_report
        .residuals
        .iter()
        .zip(jacobian_step.iter())
        .map(|(residual, jacobian_step)| residual + jacobian_step)
        .collect::<Vec<_>>();
    let linearized_residual_norm = residual_norm(&linearized_residual);
    let step_norm = residual_norm(&solve_report.solution);

    Ok(HermiteSimpsonFixedMeshSparseGaussNewtonStepReport {
        residual_report,
        sparse_jacobian,
        normal_hessian,
        rhs,
        solve_report,
        linearized_residual,
        linearized_residual_norm,
        step_norm,
    })
}

fn assemble_sparse_augmented_kkt_step_report(
    residual_report: HermiteSimpsonFixedMeshResidualReport,
    sparse_jacobian: HermiteSimpsonSparseResidualJacobianReport,
    kkt_regularization: f64,
    kkt_drop_tolerance: f64,
) -> Result<HermiteSimpsonFixedMeshSparseKktStepReport, TrajoptError> {
    validate_sparse_residual_jacobian_report(&sparse_jacobian)?;
    validate_residual_dimension(&residual_report.residuals, sparse_jacobian.row_count)?;
    let augmented_kkt =
        sparse_jacobian.sparse_augmented_kkt_report(kkt_regularization, kkt_drop_tolerance)?;
    let mut rhs = vec![0.0; augmented_kkt.row_count];
    for (row, residual) in residual_report.residuals.iter().enumerate() {
        rhs[sparse_jacobian.column_count + row] = -*residual;
    }
    let solve_report = augmented_kkt.solve_ldlt(&rhs)?;
    let step = solve_report.solution[..sparse_jacobian.column_count].to_vec();
    let residual_multipliers = solve_report.solution[sparse_jacobian.column_count..].to_vec();
    let jacobian_step = sparse_jacobian.multiply_vector(&step)?;
    let linearized_residual = residual_report
        .residuals
        .iter()
        .zip(jacobian_step.iter())
        .map(|(residual, jacobian_step)| residual + jacobian_step)
        .collect::<Vec<_>>();
    let linearized_residual_norm = residual_norm(&linearized_residual);
    let step_norm = residual_norm(&step);

    Ok(HermiteSimpsonFixedMeshSparseKktStepReport {
        residual_report,
        sparse_jacobian,
        augmented_kkt,
        rhs,
        solve_report,
        step,
        residual_multipliers,
        linearized_residual,
        linearized_residual_norm,
        step_norm,
    })
}

fn assemble_sparse_constrained_kkt_step_report(
    residual_report: HermiteSimpsonFixedMeshResidualReport,
    sparse_jacobian: HermiteSimpsonSparseResidualJacobianReport,
    hard_path_inequality_report: HermiteSimpsonPathInequalityReport,
    decision: &[f64],
    kkt_regularization: f64,
    kkt_drop_tolerance: f64,
    active_tolerance: f64,
) -> Result<HermiteSimpsonFixedMeshSparseConstrainedKktStepReport, TrajoptError> {
    validate_sparse_residual_jacobian_report(&sparse_jacobian)?;
    validate_residual_dimension(&residual_report.residuals, sparse_jacobian.row_count)?;
    validate_path_inequality_report(&hard_path_inequality_report)?;
    validate_dense_decision_vector(decision, hard_path_inequality_report.variable_count)?;
    validate_sparse_augmented_kkt_settings(kkt_regularization, kkt_drop_tolerance)?;
    if !active_tolerance.is_finite() || active_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT active tolerance must be finite and non-negative",
        });
    }
    if hard_path_inequality_report.variable_count != sparse_jacobian.column_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT inequality dimensions are inconsistent",
        });
    }

    let inequality_values = hard_path_inequality_values(&hard_path_inequality_report, decision)?;
    let mut active_inequality_rows = Vec::new();
    let mut active_inequality_values = Vec::new();
    for (row, value) in inequality_values.iter().enumerate() {
        if *value >= -active_tolerance {
            active_inequality_rows.push(row);
            active_inequality_values.push(*value);
        }
    }
    if active_inequality_rows.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT requires at least one active hard path row",
        });
    }

    let constrained_kkt = assemble_sparse_constrained_kkt_report(
        &sparse_jacobian,
        &hard_path_inequality_report,
        active_inequality_rows,
        active_inequality_values,
        kkt_regularization,
        kkt_drop_tolerance,
        active_tolerance,
    )?;
    let mut rhs = vec![0.0; constrained_kkt.row_count];
    for (row, residual) in residual_report.residuals.iter().enumerate() {
        rhs[sparse_jacobian.column_count + row] = -*residual;
    }
    let active_offset = sparse_jacobian.column_count + sparse_jacobian.row_count;
    for (index, value) in constrained_kkt.active_inequality_values.iter().enumerate() {
        rhs[active_offset + index] = -*value;
    }

    let solve_report = constrained_kkt.solve_ldlt(&rhs)?;
    let variable_count = sparse_jacobian.column_count;
    let residual_end = variable_count + sparse_jacobian.row_count;
    let step = solve_report.solution[..variable_count].to_vec();
    let residual_multipliers = solve_report.solution[variable_count..residual_end].to_vec();
    let active_inequality_multipliers = solve_report.solution[residual_end..].to_vec();
    let jacobian_step = sparse_jacobian.multiply_vector(&step)?;
    let linearized_residual = residual_report
        .residuals
        .iter()
        .zip(jacobian_step.iter())
        .map(|(residual, jacobian_step)| residual + jacobian_step)
        .collect::<Vec<_>>();
    let linearized_residual_norm = residual_norm(&linearized_residual);
    let linearized_active_inequality_values = linearized_active_inequality_values(
        &hard_path_inequality_report,
        &constrained_kkt.active_inequality_rows,
        &constrained_kkt.active_inequality_values,
        &step,
    )?;
    let max_linearized_active_inequality_violation = linearized_active_inequality_values
        .iter()
        .map(|value| value.max(0.0))
        .fold(0.0_f64, f64::max);
    let step_norm = residual_norm(&step);

    Ok(HermiteSimpsonFixedMeshSparseConstrainedKktStepReport {
        residual_report,
        sparse_jacobian,
        hard_path_inequality_report,
        constrained_kkt,
        rhs,
        solve_report,
        step,
        residual_multipliers,
        active_inequality_multipliers,
        linearized_residual,
        linearized_residual_norm,
        linearized_active_inequality_values,
        max_linearized_active_inequality_violation,
        step_norm,
    })
}

fn assemble_sparse_constrained_kkt_report(
    sparse_jacobian: &HermiteSimpsonSparseResidualJacobianReport,
    hard_path_inequality_report: &HermiteSimpsonPathInequalityReport,
    active_inequality_rows: Vec<usize>,
    active_inequality_values: Vec<f64>,
    regularization: f64,
    drop_tolerance: f64,
    active_tolerance: f64,
) -> Result<HermiteSimpsonSparseConstrainedKktReport, TrajoptError> {
    validate_sparse_residual_jacobian_report(sparse_jacobian)?;
    validate_path_inequality_report(hard_path_inequality_report)?;
    validate_sparse_augmented_kkt_settings(regularization, drop_tolerance)?;
    if !active_tolerance.is_finite() || active_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT active tolerance must be finite and non-negative",
        });
    }
    if active_inequality_rows.is_empty()
        || active_inequality_rows.len() != active_inequality_values.len()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT active rows are inconsistent",
        });
    }
    if active_inequality_rows
        .windows(2)
        .any(|window| window[0] >= window[1])
        || active_inequality_rows
            .iter()
            .any(|row| *row >= hard_path_inequality_report.inequality_count)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT active row index is invalid",
        });
    }
    validate_finite_slice(&active_inequality_values)?;
    if hard_path_inequality_report.variable_count != sparse_jacobian.column_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT inequality dimensions are inconsistent",
        });
    }

    let variable_count = sparse_jacobian.column_count;
    let residual_row_count = sparse_jacobian.row_count;
    let active_inequality_row_count = active_inequality_rows.len();
    let dimension = variable_count + residual_row_count + active_inequality_row_count;
    let mut coefficients = BTreeMap::<(usize, usize), f64>::new();

    for variable in 0..variable_count {
        coefficients.insert((variable, variable), regularization);
    }
    for residual in 0..residual_row_count {
        let index = variable_count + residual;
        coefficients.insert((index, index), -1.0);
    }
    for column in 0..sparse_jacobian.column_count {
        for index in
            sparse_jacobian.column_offsets[column]..sparse_jacobian.column_offsets[column + 1]
        {
            let residual_row = sparse_jacobian.row_indices[index];
            let value = sparse_jacobian.values[index];
            let kkt_residual_index = variable_count + residual_row;
            *coefficients
                .entry((column, kkt_residual_index))
                .or_insert(0.0) += value;
            *coefficients
                .entry((kkt_residual_index, column))
                .or_insert(0.0) += value;
        }
    }

    let mut source_inequality_nonzero_count = 0usize;
    for (active_index, source_row) in active_inequality_rows.iter().enumerate() {
        let active_kkt_index = variable_count + residual_row_count + active_index;
        let row_offset = *source_row * variable_count;
        for variable in 0..variable_count {
            let value = hard_path_inequality_report.inequality_matrix[row_offset + variable];
            if value.abs() > drop_tolerance {
                source_inequality_nonzero_count += 1;
                *coefficients
                    .entry((variable, active_kkt_index))
                    .or_insert(0.0) += value;
                *coefficients
                    .entry((active_kkt_index, variable))
                    .or_insert(0.0) += value;
            }
        }
    }
    if source_inequality_nonzero_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT active rows have no coefficients",
        });
    }

    let mut column_offsets = Vec::with_capacity(dimension + 1);
    let mut row_indices = Vec::new();
    let mut values = Vec::new();
    let mut current_column = 0;
    column_offsets.push(0);
    for ((column, row), value) in coefficients {
        while current_column < column {
            column_offsets.push(values.len());
            current_column += 1;
        }
        if value.abs() > drop_tolerance {
            row_indices.push(row);
            values.push(value);
        }
    }
    while current_column < dimension {
        column_offsets.push(values.len());
        current_column += 1;
    }

    let report = HermiteSimpsonSparseConstrainedKktReport {
        row_count: dimension,
        column_count: dimension,
        variable_count,
        residual_row_count,
        active_inequality_row_count,
        source_jacobian_nonzero_count: sparse_jacobian.values.len(),
        source_inequality_nonzero_count,
        active_inequality_rows,
        active_inequality_values,
        column_offsets,
        row_indices,
        values,
        regularization,
        drop_tolerance,
        active_tolerance,
    };
    validate_sparse_constrained_kkt_report(&report)?;
    Ok(report)
}

fn hard_path_inequality_values(
    report: &HermiteSimpsonPathInequalityReport,
    decision: &[f64],
) -> Result<Vec<f64>, TrajoptError> {
    let lhs = report.lhs(decision)?;
    Ok(lhs
        .iter()
        .zip(report.inequality_rhs.iter())
        .map(|(lhs, rhs)| lhs - rhs)
        .collect())
}

fn sparse_constrained_kkt_step_report_if_active(
    decision: &[f64],
    settings: SparseKktSqpSettings,
    hard_path_inequality_report: &HermiteSimpsonPathInequalityReport,
    final_kkt_step_report: &HermiteSimpsonFixedMeshSparseKktStepReport,
) -> Result<Option<HermiteSimpsonFixedMeshSparseConstrainedKktStepReport>, TrajoptError> {
    if hard_path_inequality_report.inequality_count == 0 {
        return Ok(None);
    }
    let inequality_values = hard_path_inequality_values(hard_path_inequality_report, decision)?;
    if !inequality_values
        .iter()
        .any(|value| *value >= -settings.residual_tolerance)
    {
        return Ok(None);
    }

    assemble_sparse_constrained_kkt_step_report(
        final_kkt_step_report.residual_report.clone(),
        final_kkt_step_report.sparse_jacobian.clone(),
        hard_path_inequality_report.clone(),
        decision,
        settings.kkt_regularization,
        settings.kkt_drop_tolerance,
        settings.residual_tolerance,
    )
    .map(Some)
}

fn linearized_active_inequality_values(
    report: &HermiteSimpsonPathInequalityReport,
    active_rows: &[usize],
    active_values: &[f64],
    step: &[f64],
) -> Result<Vec<f64>, TrajoptError> {
    validate_path_inequality_report(report)?;
    validate_dense_decision_vector(step, report.variable_count)?;
    if active_rows.len() != active_values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson sparse constrained KKT active values are inconsistent",
        });
    }
    validate_finite_slice(active_values)?;
    let mut linearized = Vec::with_capacity(active_rows.len());
    for (active_index, source_row) in active_rows.iter().enumerate() {
        if *source_row >= report.inequality_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson sparse constrained KKT active row index is invalid",
            });
        }
        let row_offset = *source_row * report.variable_count;
        let step_contribution = report.inequality_matrix
            [row_offset..row_offset + report.variable_count]
            .iter()
            .zip(step.iter())
            .map(|(coefficient, step)| coefficient * step)
            .sum::<f64>();
        linearized.push(active_values[active_index] + step_contribution);
    }
    Ok(linearized)
}

fn assemble_kkt_residual_multiplier_report(
    layout: &HermiteSimpsonDecisionLayout,
    step_report: &HermiteSimpsonFixedMeshSparseKktStepReport,
) -> Result<HermiteSimpsonKktResidualMultiplierReport, TrajoptError> {
    validate_sparse_residual_jacobian_report(&step_report.sparse_jacobian)?;
    if step_report.sparse_jacobian.column_count != layout.variable_count
        || step_report.augmented_kkt.variable_count != layout.variable_count
        || step_report.augmented_kkt.residual_row_count != step_report.sparse_jacobian.row_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson KKT multiplier report does not match the OCP layout",
        });
    }
    validate_residual_dimension(
        &step_report.residual_report.residuals,
        step_report.sparse_jacobian.row_count,
    )?;
    if step_report.residual_multipliers.len() != step_report.residual_report.residuals.len()
        || step_report.linearized_residual.len() != step_report.residual_report.residuals.len()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson KKT residual multiplier dimensions are inconsistent",
        });
    }
    assemble_kkt_residual_multiplier_report_from_residuals(
        layout,
        &step_report.residual_report,
        &step_report.residual_multipliers,
    )
}

fn assemble_kkt_residual_multiplier_report_from_residuals(
    layout: &HermiteSimpsonDecisionLayout,
    residual_report: &HermiteSimpsonFixedMeshResidualReport,
    residual_multipliers: &[f64],
) -> Result<HermiteSimpsonKktResidualMultiplierReport, TrajoptError> {
    if residual_multipliers.len() != residual_report.residuals.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson KKT residual multiplier dimensions are inconsistent",
        });
    }
    validate_finite_slice(residual_multipliers)?;
    let expected_collocation_count = layout.segment_count * layout.state_dimension * 2;
    let expected_total_count = residual_report.collocation_residual_count
        + residual_report.boundary_residual_count
        + residual_report.path_residual_count
        + residual_report.objective_residual_count;
    if residual_report.collocation_residual_count != expected_collocation_count
        || expected_total_count != residual_report.residuals.len()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson residual row families do not match the OCP layout",
        });
    }

    let mut offset = 0usize;
    let mut collocation_samples = Vec::with_capacity(layout.segment_count);
    let mut max_abs_dynamics_multiplier = 0.0_f64;
    let mut max_abs_midpoint_multiplier = 0.0_f64;
    for segment_index in 0..layout.segment_count {
        let dynamics_end = offset + layout.state_dimension;
        let dynamics_defect_multipliers = residual_multipliers[offset..dynamics_end].to_vec();
        max_abs_dynamics_multiplier = max_abs_dynamics_multiplier.max(
            dynamics_defect_multipliers
                .iter()
                .map(|value| value.abs())
                .fold(0.0_f64, f64::max),
        );
        offset = dynamics_end;

        let midpoint_end = offset + layout.state_dimension;
        let midpoint_defect_multipliers = residual_multipliers[offset..midpoint_end].to_vec();
        max_abs_midpoint_multiplier = max_abs_midpoint_multiplier.max(
            midpoint_defect_multipliers
                .iter()
                .map(|value| value.abs())
                .fold(0.0_f64, f64::max),
        );
        offset = midpoint_end;

        collocation_samples.push(HermiteSimpsonKktCollocationMultiplierSample {
            segment_index,
            dynamics_defect_multipliers,
            midpoint_defect_multipliers,
        });
    }

    let boundary_end = offset + residual_report.boundary_residual_count;
    let boundary_multipliers = residual_multipliers[offset..boundary_end].to_vec();
    let max_abs_boundary_multiplier = boundary_multipliers
        .iter()
        .map(|value| value.abs())
        .fold(0.0_f64, f64::max);
    offset = boundary_end;

    let path_end = offset + residual_report.path_residual_count;
    let path_multipliers = residual_multipliers[offset..path_end].to_vec();
    offset = path_end;

    let objective_end = offset + residual_report.objective_residual_count;
    let objective_multipliers = residual_multipliers[offset..objective_end].to_vec();
    if objective_end != residual_multipliers.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson KKT residual multiplier family split is inconsistent",
        });
    }

    Ok(HermiteSimpsonKktResidualMultiplierReport {
        segment_count: layout.segment_count,
        state_dimension: layout.state_dimension,
        residual_multiplier_count: residual_multipliers.len(),
        collocation_multiplier_count: residual_report.collocation_residual_count,
        boundary_multiplier_count: residual_report.boundary_residual_count,
        path_multiplier_count: residual_report.path_residual_count,
        objective_multiplier_count: residual_report.objective_residual_count,
        collocation_samples,
        boundary_multipliers,
        path_multipliers,
        objective_multipliers,
        max_abs_dynamics_multiplier,
        max_abs_midpoint_multiplier,
        max_abs_boundary_multiplier,
    })
}

#[cfg(feature = "sqp")]
fn dense_qp_residual_multipliers_from_equality_duals(
    report: &HermiteSimpsonFixedMeshDenseQuadraticQpReport,
) -> Result<Vec<f64>, TrajoptError> {
    let expected_equality_count = report.residual_report.collocation_residual_count
        + report.residual_report.boundary_residual_count;
    if report.equality_row_count != expected_equality_count
        || report.qp_solution.equality_dual.len() != report.equality_row_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson dense-QP equality dual dimensions are inconsistent",
        });
    }
    validate_finite_slice(&report.qp_solution.equality_dual)?;
    let mut residual_multipliers = vec![0.0; report.residual_report.residuals.len()];
    residual_multipliers[..report.equality_row_count]
        .copy_from_slice(&report.qp_solution.equality_dual);
    Ok(residual_multipliers)
}

fn assemble_kkt_node_costate_estimate_report(
    time_nodes_s: &[f64],
    multiplier_report: &HermiteSimpsonKktResidualMultiplierReport,
) -> Result<HermiteSimpsonKktCostateEstimateReport, TrajoptError> {
    if time_nodes_s.len() != multiplier_report.segment_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson KKT costate estimate time nodes are inconsistent",
        });
    }
    validate_time_nodes(time_nodes_s, multiplier_report.segment_count)?;
    if multiplier_report.state_dimension == 0
        || multiplier_report.collocation_samples.len() != multiplier_report.segment_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson KKT costate estimate multiplier dimensions are inconsistent",
        });
    }
    for (segment_index, sample) in multiplier_report.collocation_samples.iter().enumerate() {
        if sample.segment_index != segment_index
            || sample.dynamics_defect_multipliers.len() != multiplier_report.state_dimension
            || sample.midpoint_defect_multipliers.len() != multiplier_report.state_dimension
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson KKT costate estimate collocation sample is inconsistent",
            });
        }
        validate_finite_slice(&sample.dynamics_defect_multipliers)?;
        validate_finite_slice(&sample.midpoint_defect_multipliers)?;
    }

    let mut node_samples = Vec::with_capacity(multiplier_report.segment_count + 1);
    let mut max_abs_costate = 0.0_f64;
    for (node_index, time_s) in time_nodes_s
        .iter()
        .enumerate()
        .take(multiplier_report.segment_count + 1)
    {
        let mut costate = vec![0.0; multiplier_report.state_dimension];
        let mut contributing_segment_count = 0usize;
        if node_index > 0 {
            let previous =
                &multiplier_report.collocation_samples[node_index - 1].dynamics_defect_multipliers;
            for (component, value) in previous.iter().enumerate() {
                costate[component] += *value;
            }
            contributing_segment_count += 1;
        }
        if node_index < multiplier_report.segment_count {
            let next =
                &multiplier_report.collocation_samples[node_index].dynamics_defect_multipliers;
            for (component, value) in next.iter().enumerate() {
                costate[component] += *value;
            }
            contributing_segment_count += 1;
        }
        if contributing_segment_count == 0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson KKT costate estimate has no contributing segments",
            });
        }
        let scale = 1.0 / (contributing_segment_count as f64);
        for value in &mut costate {
            *value *= scale;
            max_abs_costate = max_abs_costate.max(value.abs());
        }
        validate_finite_slice(&costate)?;
        node_samples.push(HermiteSimpsonKktNodeCostateEstimate {
            node_index,
            time_s: *time_s,
            costate,
            contributing_segment_count,
        });
    }

    Ok(HermiteSimpsonKktCostateEstimateReport {
        segment_count: multiplier_report.segment_count,
        state_dimension: multiplier_report.state_dimension,
        source_residual_multiplier_count: multiplier_report.residual_multiplier_count,
        node_samples,
        max_abs_costate,
    })
}

fn assemble_kkt_hamiltonian_report<D, L>(
    layout: &HermiteSimpsonDecisionLayout,
    time_nodes_s: &[f64],
    decision: &[f64],
    costate_report: &HermiteSimpsonKktCostateEstimateReport,
    dynamics: &mut D,
    lagrange: &mut L,
) -> Result<HermiteSimpsonKktHamiltonianReport, TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    validate_decision_vector(layout, decision)?;
    validate_time_nodes(time_nodes_s, layout.segment_count)?;
    if costate_report.segment_count != layout.segment_count
        || costate_report.state_dimension != layout.state_dimension
        || costate_report.node_samples.len() != layout.segment_count + 1
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson KKT Hamiltonian costate report dimensions are inconsistent",
        });
    }

    let mut samples = Vec::with_capacity(layout.segment_count + 1);
    let mut max_abs_hamiltonian = 0.0_f64;
    let mut max_abs_hamiltonian_drift = 0.0_f64;
    let mut reference_hamiltonian = None;
    for (node_index, costate_sample) in costate_report.node_samples.iter().enumerate() {
        if costate_sample.node_index != node_index
            || costate_sample.time_s.to_bits() != time_nodes_s[node_index].to_bits()
            || costate_sample.costate.len() != layout.state_dimension
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson KKT Hamiltonian costate sample is inconsistent",
            });
        }
        validate_finite_slice(&costate_sample.costate)?;

        let state = layout.node_state(decision, node_index)?;
        let control = slice(
            decision,
            layout.node_control_offset + node_index * layout.control_dimension,
            layout.control_dimension,
        );
        let dynamics_value = dynamics.eval(time_nodes_s[node_index], state, control)?;
        validate_derivative(&dynamics_value, layout.state_dimension)?;
        let lagrange_value = lagrange.eval(time_nodes_s[node_index], state, control)?;
        if !lagrange_value.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson KKT Hamiltonian Lagrange value must be finite",
            });
        }
        let costate_dot_dynamics = dot(&costate_sample.costate, &dynamics_value);
        let hamiltonian = lagrange_value + costate_dot_dynamics;
        if !costate_dot_dynamics.is_finite() || !hamiltonian.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson KKT Hamiltonian value must be finite",
            });
        }
        max_abs_hamiltonian = max_abs_hamiltonian.max(hamiltonian.abs());
        let first_hamiltonian = *reference_hamiltonian.get_or_insert(hamiltonian);
        max_abs_hamiltonian_drift =
            max_abs_hamiltonian_drift.max((hamiltonian - first_hamiltonian).abs());

        samples.push(HermiteSimpsonKktHamiltonianSample {
            node_index,
            time_s: time_nodes_s[node_index],
            costate: costate_sample.costate.clone(),
            dynamics: dynamics_value,
            lagrange_value,
            costate_dot_dynamics,
            hamiltonian,
        });
    }

    Ok(HermiteSimpsonKktHamiltonianReport {
        segment_count: layout.segment_count,
        state_dimension: layout.state_dimension,
        sample_count: samples.len(),
        samples,
        max_abs_hamiltonian,
        max_abs_hamiltonian_drift,
    })
}

fn segment_structural_columns(
    layout: &HermiteSimpsonDecisionLayout,
    segment_index: usize,
) -> Vec<usize> {
    let mut columns = Vec::with_capacity(3 * (layout.state_dimension + layout.control_dimension));
    for component in 0..layout.state_dimension {
        columns.push(layout.node_state_offset + segment_index * layout.state_dimension + component);
    }
    for component in 0..layout.state_dimension {
        columns.push(
            layout.midpoint_state_offset + segment_index * layout.state_dimension + component,
        );
    }
    for component in 0..layout.state_dimension {
        columns.push(
            layout.node_state_offset + (segment_index + 1) * layout.state_dimension + component,
        );
    }
    for component in 0..layout.control_dimension {
        columns.push(
            layout.node_control_offset + segment_index * layout.control_dimension + component,
        );
    }
    for component in 0..layout.control_dimension {
        columns.push(
            layout.midpoint_control_offset + segment_index * layout.control_dimension + component,
        );
    }
    for component in 0..layout.control_dimension {
        columns.push(
            layout.node_control_offset + (segment_index + 1) * layout.control_dimension + component,
        );
    }
    columns
}

fn assemble_segment_analytic_jacobian(
    layout: &HermiteSimpsonDecisionLayout,
    segment_index: usize,
    step_s: f64,
    f0: &CollocationDynamicsJacobianSample,
    fm: &CollocationDynamicsJacobianSample,
    f1: &CollocationDynamicsJacobianSample,
    dense: &mut [f64],
) -> Result<(), TrajoptError> {
    let column_count = layout.variable_count;
    if column_count == 0 || !dense.len().is_multiple_of(column_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson analytic residual Jacobian dense shape is inconsistent",
        });
    }
    let row_count = dense.len() / column_count;
    validate_dynamics_jacobian_sample(f0, layout.state_dimension, layout.control_dimension)?;
    validate_dynamics_jacobian_sample(fm, layout.state_dimension, layout.control_dimension)?;
    validate_dynamics_jacobian_sample(f1, layout.state_dimension, layout.control_dimension)?;
    if !step_s.is_finite() || step_s <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson analytic residual Jacobian step must be finite and positive",
        });
    }

    let state_dimension = layout.state_dimension;
    let control_dimension = layout.control_dimension;
    let dynamics_row_start = segment_index * state_dimension * 2;
    let midpoint_row_start = dynamics_row_start + state_dimension;
    if midpoint_row_start + state_dimension > row_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson analytic residual Jacobian row range is inconsistent",
        });
    }
    for state_row in 0..state_dimension {
        let dynamics_row = dynamics_row_start + state_row;
        let midpoint_row = midpoint_row_start + state_row;
        for state_col in 0..state_dimension {
            let identity = if state_row == state_col { 1.0 } else { 0.0 };
            let x0_col = layout.node_state_offset + segment_index * state_dimension + state_col;
            let xm_col = layout.midpoint_state_offset + segment_index * state_dimension + state_col;
            let x1_col =
                layout.node_state_offset + (segment_index + 1) * state_dimension + state_col;
            let a0 = f0.state_jacobian[state_row * state_dimension + state_col];
            let am = fm.state_jacobian[state_row * state_dimension + state_col];
            let a1 = f1.state_jacobian[state_row * state_dimension + state_col];

            dense[dynamics_row * column_count + x0_col] += -identity - step_s * a0 / 6.0;
            dense[dynamics_row * column_count + xm_col] += -4.0 * step_s * am / 6.0;
            dense[dynamics_row * column_count + x1_col] += identity - step_s * a1 / 6.0;

            dense[midpoint_row * column_count + x0_col] += -0.5 * identity - step_s * a0 / 8.0;
            dense[midpoint_row * column_count + xm_col] += identity;
            dense[midpoint_row * column_count + x1_col] += -0.5 * identity + step_s * a1 / 8.0;
        }
        for control_col in 0..control_dimension {
            let u0_col =
                layout.node_control_offset + segment_index * control_dimension + control_col;
            let um_col =
                layout.midpoint_control_offset + segment_index * control_dimension + control_col;
            let u1_col =
                layout.node_control_offset + (segment_index + 1) * control_dimension + control_col;
            let b0 = f0.control_jacobian[state_row * control_dimension + control_col];
            let bm = fm.control_jacobian[state_row * control_dimension + control_col];
            let b1 = f1.control_jacobian[state_row * control_dimension + control_col];

            dense[dynamics_row * column_count + u0_col] += -step_s * b0 / 6.0;
            dense[dynamics_row * column_count + um_col] += -4.0 * step_s * bm / 6.0;
            dense[dynamics_row * column_count + u1_col] += -step_s * b1 / 6.0;

            dense[midpoint_row * column_count + u0_col] += -step_s * b0 / 8.0;
            dense[midpoint_row * column_count + u1_col] += step_s * b1 / 8.0;
        }
    }
    Ok(())
}

fn push_structural_row(
    column_rows: &mut [Vec<usize>],
    row: usize,
    columns: &[usize],
) -> Result<(), TrajoptError> {
    for column in columns {
        let rows = column_rows
            .get_mut(*column)
            .ok_or(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson residual structural pattern column is out of range",
            })?;
        if rows.last() != Some(&row) {
            rows.push(row);
        }
    }
    Ok(())
}

fn greedy_column_colors(
    column_rows: &[Vec<usize>],
    row_count: usize,
) -> Result<Vec<usize>, TrajoptError> {
    let mut row_colors = vec![Vec::<usize>::new(); row_count];
    let mut column_colors = Vec::with_capacity(column_rows.len());
    for rows in column_rows {
        if rows.iter().any(|row| *row >= row_count) {
            return Err(TrajoptError::InvalidPayload {
                reason: "Hermite-Simpson residual structural pattern row is out of range",
            });
        }
        let mut color = 0;
        while rows.iter().any(|row| row_colors[*row].contains(&color)) {
            color += 1;
        }
        for row in rows {
            row_colors[*row].push(color);
        }
        column_colors.push(color);
    }
    Ok(column_colors)
}

fn validate_decision_bounds_report(
    report: &HermiteSimpsonDecisionBoundsReport,
) -> Result<(), TrajoptError> {
    if report.variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson decision bounds variable count must be nonzero",
        });
    }
    if report.lower_bounds.len() != report.variable_count
        || report.upper_bounds.len() != report.variable_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson decision bounds report dimensions are inconsistent",
        });
    }
    validate_finite_slice(&report.lower_bounds)?;
    validate_finite_slice(&report.upper_bounds)?;
    if !report
        .lower_bounds
        .iter()
        .zip(report.upper_bounds.iter())
        .all(|(lower, upper)| lower <= upper)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson decision bounds report is contradictory",
        });
    }
    if report.constrained_variable_count > report.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson decision bounds constrained count is inconsistent",
        });
    }
    Ok(())
}

fn append_path_residuals(
    ocp: &HermiteSimpsonFixedMeshOcp,
    decision: &[f64],
    residuals: &mut Vec<f64>,
) -> Result<usize, TrajoptError> {
    for constraint in &ocp.path_constraints {
        let value = sample_scalar(
            &ocp.layout,
            decision,
            constraint.sample,
            constraint.variable,
        )?;
        let mut violation = 0.0_f64;
        if let Some(lower) = constraint.lower
            && value < lower
        {
            violation = value - lower;
        }
        if let Some(upper) = constraint.upper
            && value > upper
        {
            violation = value - upper;
        }
        residuals.push(constraint.weight.sqrt() * violation);
    }
    Ok(ocp.path_constraints.len())
}

fn append_quadratic_control_cost_residuals(
    ocp: &HermiteSimpsonFixedMeshOcp,
    decision: &[f64],
    residuals: &mut Vec<f64>,
) -> Result<f64, TrajoptError> {
    let mut cost_sum = 0.0;
    for segment_index in 0..ocp.layout.segment_count {
        let h = ocp.time_nodes_s[segment_index + 1] - ocp.time_nodes_s[segment_index];
        for cost in &ocp.quadratic_control_costs {
            let u0 = sample_scalar(
                &ocp.layout,
                decision,
                HermiteSimpsonSamplePoint::Node(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent {
                    component: cost.component,
                },
            )?;
            let um = sample_scalar(
                &ocp.layout,
                decision,
                HermiteSimpsonSamplePoint::Midpoint(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent {
                    component: cost.component,
                },
            )?;
            let u1 = sample_scalar(
                &ocp.layout,
                decision,
                HermiteSimpsonSamplePoint::Node(segment_index + 1),
                HermiteSimpsonScalarVariable::ControlComponent {
                    component: cost.component,
                },
            )?;
            let endpoint_scale = (cost.weight * h / 6.0).sqrt();
            let midpoint_scale = (cost.weight * 4.0 * h / 6.0).sqrt();
            residuals.push(endpoint_scale * u0);
            residuals.push(midpoint_scale * um);
            residuals.push(endpoint_scale * u1);
            cost_sum += cost.weight * h * (u0 * u0 + 4.0 * um * um + u1 * u1) / 6.0;
        }
    }
    Ok(cost_sum)
}

#[cfg(feature = "sqp")]
fn dense_quadratic_control_hessian(
    ocp: &HermiteSimpsonFixedMeshOcp,
) -> Result<Vec<f64>, TrajoptError> {
    if ocp.quadratic_control_costs.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson dense quadratic QP requires at least one control cost",
        });
    }
    let n = ocp.layout.variable_count;
    let mut hessian = vec![0.0; n * n];
    for segment_index in 0..ocp.layout.segment_count {
        let h = ocp.time_nodes_s[segment_index + 1] - ocp.time_nodes_s[segment_index];
        for cost in &ocp.quadratic_control_costs {
            validate_quadratic_control_cost(&ocp.layout, cost)?;
            let node0 = scalar_decision_index(
                &ocp.layout,
                HermiteSimpsonSamplePoint::Node(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent {
                    component: cost.component,
                },
            )?;
            let midpoint = scalar_decision_index(
                &ocp.layout,
                HermiteSimpsonSamplePoint::Midpoint(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent {
                    component: cost.component,
                },
            )?;
            let node1 = scalar_decision_index(
                &ocp.layout,
                HermiteSimpsonSamplePoint::Node(segment_index + 1),
                HermiteSimpsonScalarVariable::ControlComponent {
                    component: cost.component,
                },
            )?;
            hessian[node0 * n + node0] += cost.weight * h / 6.0;
            hessian[midpoint * n + midpoint] += cost.weight * 2.0 * h / 3.0;
            hessian[node1 * n + node1] += cost.weight * h / 6.0;
        }
    }
    Ok(hessian)
}

fn sample_state_control<'a>(
    layout: &HermiteSimpsonDecisionLayout,
    time_nodes_s: &[f64],
    decision: &'a [f64],
    sample: HermiteSimpsonSamplePoint,
) -> Result<(f64, &'a [f64], &'a [f64]), TrajoptError> {
    validate_time_nodes(time_nodes_s, layout.segment_count)?;
    validate_decision_vector(layout, decision)?;
    validate_sample(layout, sample)?;
    match sample {
        HermiteSimpsonSamplePoint::Node(index) => {
            let time_s = time_nodes_s[index];
            let state = slice(
                decision,
                layout.node_state_offset + index * layout.state_dimension,
                layout.state_dimension,
            );
            let control = slice(
                decision,
                layout.node_control_offset + index * layout.control_dimension,
                layout.control_dimension,
            );
            Ok((time_s, state, control))
        }
        HermiteSimpsonSamplePoint::Midpoint(index) => {
            let time_s = 0.5 * (time_nodes_s[index] + time_nodes_s[index + 1]);
            let state = slice(
                decision,
                layout.midpoint_state_offset + index * layout.state_dimension,
                layout.state_dimension,
            );
            let control = slice(
                decision,
                layout.midpoint_control_offset + index * layout.control_dimension,
                layout.control_dimension,
            );
            Ok((time_s, state, control))
        }
    }
}

fn reference_profile_sample_from_state(
    time_s: f64,
    state: &[f64],
    mapping: HermiteSimpsonReferenceProfileMapping,
) -> Result<ReferenceProfileSample, TrajoptError> {
    let sample = ReferenceProfileSample {
        time_s,
        radius_m: state[mapping.radius_component],
        speed_m_s: state[mapping.speed_component],
        flight_path_angle_rad: state[mapping.flight_path_angle_component],
    };
    validate_reference_profile_sample(&sample)?;
    Ok(sample)
}

fn validate_reference_profile_sample(sample: &ReferenceProfileSample) -> Result<(), TrajoptError> {
    if sample.time_s.is_finite()
        && sample.radius_m.is_finite()
        && sample.speed_m_s.is_finite()
        && sample.flight_path_angle_rad.is_finite()
    {
        Ok(())
    } else {
        Err(TrajoptError::InvalidPayload {
            reason: "Hermite-Simpson reference profile samples must be finite",
        })
    }
}

fn sample_scalar(
    layout: &HermiteSimpsonDecisionLayout,
    decision: &[f64],
    sample: HermiteSimpsonSamplePoint,
    variable: HermiteSimpsonScalarVariable,
) -> Result<f64, TrajoptError> {
    validate_decision_vector(layout, decision)?;
    Ok(decision[scalar_decision_index(layout, sample, variable)?])
}

fn scalar_decision_index(
    layout: &HermiteSimpsonDecisionLayout,
    sample: HermiteSimpsonSamplePoint,
    variable: HermiteSimpsonScalarVariable,
) -> Result<usize, TrajoptError> {
    match (sample, variable) {
        (
            HermiteSimpsonSamplePoint::Node(index),
            HermiteSimpsonScalarVariable::StateComponent { component },
        ) => {
            if index > layout.segment_count || component >= layout.state_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson node state sample is out of range",
                });
            }
            Ok(layout.node_state_offset + index * layout.state_dimension + component)
        }
        (
            HermiteSimpsonSamplePoint::Midpoint(index),
            HermiteSimpsonScalarVariable::StateComponent { component },
        ) => {
            if index >= layout.segment_count || component >= layout.state_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson midpoint state sample is out of range",
                });
            }
            Ok(layout.midpoint_state_offset + index * layout.state_dimension + component)
        }
        (
            HermiteSimpsonSamplePoint::Node(index),
            HermiteSimpsonScalarVariable::ControlComponent { component },
        ) => {
            if index > layout.segment_count || component >= layout.control_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson node control sample is out of range",
                });
            }
            Ok(layout.node_control_offset + index * layout.control_dimension + component)
        }
        (
            HermiteSimpsonSamplePoint::Midpoint(index),
            HermiteSimpsonScalarVariable::ControlComponent { component },
        ) => {
            if index >= layout.segment_count || component >= layout.control_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "Hermite-Simpson midpoint control sample is out of range",
                });
            }
            Ok(layout.midpoint_control_offset + index * layout.control_dimension + component)
        }
    }
}

fn slice(values: &[f64], offset: usize, len: usize) -> &[f64] {
    &values[offset..offset + len]
}

fn residual_norm(values: &[f64]) -> f64 {
    values.iter().map(|value| value * value).sum::<f64>().sqrt()
}

const SPARSE_KKT_SQP_DEFAULT_DECISION_BOUND_ABS: f64 = 1.0e30;

pub(crate) fn sparse_kkt_sqp_default_decision_bounds(
    variable_count: usize,
) -> (Vec<f64>, Vec<f64>) {
    (
        vec![-SPARSE_KKT_SQP_DEFAULT_DECISION_BOUND_ABS; variable_count],
        vec![SPARSE_KKT_SQP_DEFAULT_DECISION_BOUND_ABS; variable_count],
    )
}

pub(crate) fn scaled_decision_step(
    decision: &[f64],
    step: &[f64],
    scale: f64,
) -> Result<Vec<f64>, TrajoptError> {
    if decision.len() != step.len() || !scale.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "scaled sparse KKT SQP step dimensions are inconsistent",
        });
    }
    let trial = decision
        .iter()
        .zip(step.iter())
        .map(|(value, delta)| value + scale * delta)
        .collect::<Vec<_>>();
    validate_finite_slice(&trial)?;
    Ok(trial)
}

fn binary_derivatives<F>(left: &[f64], right: &[f64], mut combine: F) -> Vec<f64>
where
    F: FnMut(f64, f64) -> f64,
{
    let derivative_count = left.len().max(right.len());
    let mut derivatives = Vec::with_capacity(derivative_count);
    for index in 0..derivative_count {
        derivatives.push(combine(
            left.get(index).copied().unwrap_or(0.0),
            right.get(index).copied().unwrap_or(0.0),
        ));
    }
    derivatives
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter()
        .zip(right.iter())
        .map(|(left_value, right_value)| left_value * right_value)
        .sum()
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use serde::Deserialize;
    use std::io::{Error, ErrorKind};

    use super::*;
    use crate::corrector::DifferentialCorrector;
    use crate::driver::{TwoBodyApogeeTargeting, correct_two_body_apogee};
    use crate::shooting::{
        MultipleShootingCorrector, MultipleShootingNode, seed_two_body_multiple_shooting_nodes,
    };
    use crate::stm::{TwoBodyCartesianState, propagate_two_body_variational};
    use openbmp_physics::WGS84_MU_M3_S2;

    const MMS_TOLERANCE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/trajopt-two-body-apogee/hermite-simpson-mms-tolerance.toml"
    ));
    const CROSS_TIER_TOLERANCE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/trajopt-two-body-apogee/multiple-shooting-tolerance.toml"
    ));
    const BRACHISTOCHRONE_BENCHMARK: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/trajopt-brachistochrone/brachistochrone-cycloid-benchmark.toml"
    ));
    #[cfg(feature = "sqp")]
    const SCALAR_INTEGRATOR_BENCHMARK: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/trajopt-two-body-apogee/hermite-simpson-scalar-integrator-benchmark.toml"
    ));
    #[cfg(feature = "sqp")]
    const BRYSON_DENHAM_BENCHMARK: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/trajopt-bryson-denham/bryson-denham-state-constrained-benchmark.toml"
    ));

    #[derive(Debug, Deserialize)]
    struct MmsToleranceTable {
        case: String,
        validation: String,
        mesh_segment_counts: Vec<usize>,
        expected_midpoint_slope: f64,
        slope_tolerance: f64,
        max_finest_midpoint_defect: f64,
    }

    impl MmsToleranceTable {
        fn parse() -> Result<Self, Box<dyn std::error::Error>> {
            let table: Self = toml::from_str(MMS_TOLERANCE)?;
            table.require_valid()?;
            Ok(table)
        }

        fn require_valid(&self) -> Result<(), Error> {
            if self.case != "hermite-simpson-sine-mms" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "unexpected Hermite-Simpson MMS tolerance case",
                ));
            }
            if self.validation != "validated-toy" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Hermite-Simpson MMS validation must be validated-toy",
                ));
            }
            if self.mesh_segment_counts.len() < 3
                || !self
                    .mesh_segment_counts
                    .windows(2)
                    .all(|window| window[0] > 0 && window[1] == window[0] * 2)
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Hermite-Simpson MMS meshes must be positive powers-of-two refinements",
                ));
            }
            if !self.expected_midpoint_slope.is_finite()
                || !self.slope_tolerance.is_finite()
                || self.slope_tolerance <= 0.0
                || !self.max_finest_midpoint_defect.is_finite()
                || self.max_finest_midpoint_defect <= 0.0
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Hermite-Simpson MMS tolerances must be finite and positive",
                ));
            }
            Ok(())
        }
    }

    #[derive(Debug, Deserialize)]
    struct CrossTierToleranceTable {
        case: String,
        source: String,
        validation: String,
        problem: CrossTierProblem,
        metric: Vec<CrossTierMetric>,
    }

    #[derive(Debug, Deserialize)]
    struct CrossTierProblem {
        initial_radius_m: f64,
        target_apogee_radius_m: f64,
        initial_tangential_speed_m_s: f64,
        coast_duration_s: f64,
        step_s: f64,
        segment_durations_s: Vec<f64>,
        residual_tolerance: f64,
        max_iterations: usize,
    }

    #[derive(Debug, Deserialize)]
    struct CrossTierMetric {
        name: String,
        expected: f64,
        absolute_tolerance: f64,
        relative_tolerance: f64,
    }

    impl CrossTierToleranceTable {
        fn parse() -> Result<Self, Box<dyn std::error::Error>> {
            let table: Self = toml::from_str(CROSS_TIER_TOLERANCE)?;
            table.require_valid()?;
            Ok(table)
        }

        fn require_valid(&self) -> Result<(), Error> {
            if self.case != "trajopt-two-body-apogee-cross-tier" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "unexpected cross-tier tolerance case",
                ));
            }
            if !self
                .source
                .contains("Synthetic WGS84 point-mass cross-tier regression")
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "cross-tier tolerance source must describe the synthetic reference",
                ));
            }
            if self.validation != "validated-toy" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "cross-tier tolerance validation must be validated-toy",
                ));
            }
            if self.metric.is_empty() {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "cross-tier tolerance table must declare metrics",
                ));
            }
            self.problem.require_valid()?;
            for metric in &self.metric {
                metric.require_valid()?;
            }
            Ok(())
        }

        fn check_metric(&self, name: &str, actual: f64) -> Result<(), Error> {
            let metric = self
                .metric
                .iter()
                .find(|metric| metric.name == name)
                .ok_or_else(|| {
                    Error::new(
                        ErrorKind::InvalidData,
                        format!("missing cross-tier tolerance metric {name}"),
                    )
                })?;
            metric.check(actual)
        }
    }

    impl CrossTierProblem {
        fn require_valid(&self) -> Result<(), Error> {
            let positive_values = [
                self.initial_radius_m,
                self.target_apogee_radius_m,
                self.initial_tangential_speed_m_s,
                self.step_s,
                self.residual_tolerance,
            ];
            if !positive_values
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "cross-tier problem positive values must be finite and positive",
                ));
            }
            if !self.coast_duration_s.is_finite() || self.coast_duration_s < 0.0 {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "cross-tier coast duration must be finite and non-negative",
                ));
            }
            if self.segment_durations_s.is_empty()
                || !self
                    .segment_durations_s
                    .iter()
                    .all(|duration_s| duration_s.is_finite() && *duration_s >= 0.0)
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "cross-tier segment durations must be finite and non-negative",
                ));
            }
            let duration_sum = self.segment_durations_s.iter().sum::<f64>();
            if (duration_sum - self.coast_duration_s).abs() > 1.0e-12 {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "cross-tier segment durations must sum to coast duration",
                ));
            }
            if self.max_iterations == 0 {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "cross-tier max_iterations must be nonzero",
                ));
            }
            Ok(())
        }
    }

    impl CrossTierMetric {
        fn require_valid(&self) -> Result<(), Error> {
            if self.name.trim().is_empty()
                || !self.expected.is_finite()
                || !self.absolute_tolerance.is_finite()
                || self.absolute_tolerance < 0.0
                || !self.relative_tolerance.is_finite()
                || self.relative_tolerance < 0.0
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    format!("invalid cross-tier metric {}", self.name),
                ));
            }
            Ok(())
        }

        fn check(&self, actual: f64) -> Result<(), Error> {
            if !actual.is_finite() {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    format!("metric {} actual is not finite", self.name),
                ));
            }
            let absolute_error = (actual - self.expected).abs();
            let relative_denominator = self.expected.abs().max(f64::MIN_POSITIVE);
            let relative_error = absolute_error / relative_denominator;
            if absolute_error <= self.absolute_tolerance
                || relative_error <= self.relative_tolerance
            {
                Ok(())
            } else {
                Err(Error::other(format!(
                    "metric {} actual {actual} outside tolerance: expected {}, \
                     absolute error {absolute_error} > {}, relative error {relative_error} > {}",
                    self.name, self.expected, self.absolute_tolerance, self.relative_tolerance
                )))
            }
        }
    }

    #[derive(Debug, Deserialize)]
    struct BrachistochroneBenchmarkTable {
        case: String,
        source: String,
        validation: String,
        problem: BrachistochroneBenchmarkProblem,
    }

    #[derive(Clone, Copy, Debug, Deserialize)]
    struct BrachistochroneBenchmarkProblem {
        endpoint_x_m: f64,
        endpoint_y_down_m: f64,
        gravity_m_s2: f64,
        theta_final_rad: f64,
        cycloid_scale_m: f64,
        optimal_time_s: f64,
        segment_count: usize,
        constant_tolerance: f64,
        max_terminal_error_m: f64,
        max_residual_norm: f64,
        relative_time_tolerance: f64,
        dense_sqp_max_residual_norm: f64,
        dense_sqp_max_solution_delta: f64,
        free_time_dense_sqp_max_residual_norm: f64,
        free_time_dense_sqp_relative_time_tolerance: f64,
    }

    impl BrachistochroneBenchmarkTable {
        fn parse() -> Result<Self, Box<dyn std::error::Error>> {
            let table: Self = toml::from_str(BRACHISTOCHRONE_BENCHMARK)?;
            table.require_valid()?;
            Ok(table)
        }

        fn require_valid(&self) -> Result<(), Error> {
            if self.case != "brachistochrone-cycloid-closed-form" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "unexpected brachistochrone benchmark case",
                ));
            }
            if !self.source.contains("Public closed-form brachistochrone") {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "brachistochrone benchmark source must describe the public closed form",
                ));
            }
            if self.validation != "research" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "brachistochrone benchmark validation must be research",
                ));
            }
            self.problem.require_valid()
        }
    }

    impl BrachistochroneBenchmarkProblem {
        fn require_valid(self) -> Result<(), Error> {
            let positive_values = [
                self.endpoint_x_m,
                self.endpoint_y_down_m,
                self.gravity_m_s2,
                self.theta_final_rad,
                self.cycloid_scale_m,
                self.optimal_time_s,
                self.constant_tolerance,
                self.max_terminal_error_m,
                self.max_residual_norm,
                self.relative_time_tolerance,
                self.dense_sqp_max_residual_norm,
                self.dense_sqp_max_solution_delta,
                self.free_time_dense_sqp_max_residual_norm,
                self.free_time_dense_sqp_relative_time_tolerance,
            ];
            if !positive_values
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
                || self.segment_count == 0
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "brachistochrone benchmark values must be finite and positive",
                ));
            }
            let endpoint_x =
                self.cycloid_scale_m * (self.theta_final_rad - self.theta_final_rad.sin());
            let endpoint_y = self.cycloid_scale_m * (1.0 - self.theta_final_rad.cos());
            let optimal_time =
                (self.cycloid_scale_m / self.gravity_m_s2).sqrt() * self.theta_final_rad;
            if (endpoint_x - self.endpoint_x_m).abs() > self.constant_tolerance
                || (endpoint_y - self.endpoint_y_down_m).abs() > self.constant_tolerance
                || (optimal_time - self.optimal_time_s).abs() > self.constant_tolerance
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "brachistochrone benchmark constants are inconsistent",
                ));
            }
            Ok(())
        }
    }

    #[cfg(feature = "sqp")]
    #[derive(Debug, Deserialize)]
    struct BrysonDenhamBenchmarkTable {
        case: String,
        source: String,
        validation: String,
        problem: BrysonDenhamBenchmarkProblem,
    }

    #[cfg(feature = "sqp")]
    #[derive(Clone, Copy, Debug, Deserialize)]
    struct BrysonDenhamBenchmarkProblem {
        segment_count: usize,
        state_bound: f64,
        entry_time_s: f64,
        exit_time_s: f64,
        expected_cost: f64,
        cost_tolerance: f64,
        max_residual_norm: f64,
        max_path_violation: f64,
        max_state_error: f64,
        max_control_error: f64,
    }

    #[cfg(feature = "sqp")]
    impl BrysonDenhamBenchmarkTable {
        fn parse() -> Result<Self, Box<dyn std::error::Error>> {
            let table: Self = toml::from_str(BRYSON_DENHAM_BENCHMARK)?;
            table.require_valid()?;
            Ok(table)
        }

        fn require_valid(&self) -> Result<(), Error> {
            if self.case != "bryson-denham-state-constrained" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "unexpected Bryson-Denham benchmark case",
                ));
            }
            if !self.source.contains("Public Bryson-Denham") {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Bryson-Denham benchmark source must describe the public benchmark",
                ));
            }
            if self.validation != "research" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Bryson-Denham benchmark validation must be research",
                ));
            }
            self.problem.require_valid()
        }
    }

    #[cfg(feature = "sqp")]
    impl BrysonDenhamBenchmarkProblem {
        fn require_valid(self) -> Result<(), Error> {
            let positive_values = [
                self.state_bound,
                self.entry_time_s,
                self.exit_time_s,
                self.expected_cost,
                self.cost_tolerance,
                self.max_residual_norm,
                self.max_state_error,
                self.max_control_error,
            ];
            if self.segment_count == 0
                || !positive_values
                    .iter()
                    .all(|value| value.is_finite() && *value > 0.0)
                || !self.max_path_violation.is_finite()
                || self.max_path_violation < 0.0
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Bryson-Denham benchmark values must be finite and positive",
                ));
            }
            if (3.0 * self.entry_time_s - 1.0).abs() > 1.0e-12
                || (3.0 * self.exit_time_s - 2.0).abs() > 1.0e-12
                || (9.0 * self.state_bound - 1.0).abs() > 1.0e-12
                || (self.expected_cost - 4.0).abs() > 1.0e-12
                || self.segment_count % 3 != 0
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Bryson-Denham benchmark constants are inconsistent",
                ));
            }
            Ok(())
        }
    }

    #[cfg(feature = "sqp")]
    #[derive(Debug, Deserialize)]
    struct ScalarIntegratorBenchmarkTable {
        case: String,
        validation: String,
        segment_count: usize,
        expected_cost: f64,
        cost_tolerance: f64,
        max_residual_norm: f64,
        max_state_error: f64,
        max_control_error: f64,
    }

    #[cfg(feature = "sqp")]
    impl ScalarIntegratorBenchmarkTable {
        fn parse() -> Result<Self, Box<dyn std::error::Error>> {
            let table: Self = toml::from_str(SCALAR_INTEGRATOR_BENCHMARK)?;
            table.require_valid()?;
            Ok(table)
        }

        fn require_valid(&self) -> Result<(), Error> {
            if self.case != "hermite-simpson-scalar-integrator-min-energy" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "unexpected Hermite-Simpson scalar-integrator benchmark case",
                ));
            }
            if self.validation != "validated-toy" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "scalar-integrator benchmark validation must be validated-toy",
                ));
            }
            if self.segment_count == 0 {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "scalar-integrator benchmark segment count must be nonzero",
                ));
            }
            if !self.expected_cost.is_finite()
                || !self.cost_tolerance.is_finite()
                || self.cost_tolerance <= 0.0
                || !self.max_residual_norm.is_finite()
                || self.max_residual_norm <= 0.0
                || !self.max_state_error.is_finite()
                || self.max_state_error <= 0.0
                || !self.max_control_error.is_finite()
                || self.max_control_error <= 0.0
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "scalar-integrator benchmark tolerances must be finite and positive",
                ));
            }
            Ok(())
        }
    }

    #[test]
    fn hermite_simpson_segment_is_exact_for_cubic_manufactured_state() {
        let segment = cubic_segment(0.0, 0.5);
        let report =
            evaluate_hermite_simpson_transcription(&[segment], cubic_dynamics, zero_lagrange)
                .expect("cubic segment evaluates");

        assert!(report.max_abs_defect < 1.0e-14);
        assert!(report.total_defect_norm < 1.0e-14);
        assert_eq!(report.segment_count, 1);
        assert_eq!(report.state_dimension, 1);
        assert_eq!(report.control_dimension, 0);
    }

    #[test]
    fn hermite_simpson_transcription_accumulates_simpson_lagrange_cost() {
        let segments = sine_segments(16);
        let report =
            evaluate_hermite_simpson_transcription(&segments, sine_dynamics, time_squared_lagrange)
                .expect("sine transcription evaluates");

        assert!((report.simpson_lagrange_cost - 1.0 / 3.0).abs() < 1.0e-14);
    }

    #[test]
    fn hermite_simpson_midpoint_mms_residual_is_fourth_order() {
        let table = MmsToleranceTable::parse().expect("MMS tolerance parses");
        let mut steps = Vec::new();
        let mut defects = Vec::new();
        for segment_count in &table.mesh_segment_counts {
            let segments = sine_segments(*segment_count);
            let report =
                evaluate_hermite_simpson_transcription(&segments, sine_dynamics, zero_lagrange)
                    .expect("sine transcription evaluates");
            steps.push(1.0 / (*segment_count as f64));
            defects.push(report.max_abs_defect);
        }

        let slope = log_log_slope(&steps, &defects);
        assert!(
            (slope - table.expected_midpoint_slope).abs() <= table.slope_tolerance,
            "expected slope {} +/- {}, got {slope}",
            table.expected_midpoint_slope,
            table.slope_tolerance
        );
        let finest = *defects.last().expect("nonempty defects");
        assert!(
            finest <= table.max_finest_midpoint_defect,
            "finest midpoint defect {finest:e} exceeded {}",
            table.max_finest_midpoint_defect
        );
    }

    #[test]
    fn hermite_simpson_brachistochrone_closed_form_benchmark_matches_cycloid()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = BrachistochroneBenchmarkTable::parse()?;
        let problem = table.problem;
        let (node_states, midpoint_states, node_controls, midpoint_controls) =
            brachistochrone_decision_blocks(problem);
        let ocp = brachistochrone_ocp(problem, problem.optimal_time_s)?;
        let decision = ocp.layout.pack_decision_vector(
            &node_states,
            &midpoint_states,
            &node_controls,
            &midpoint_controls,
        )?;
        let mut residual_dynamics = move |_: f64, state: &[f64], control: &[f64]| {
            brachistochrone_dynamics(problem.gravity_m_s2, state, control)
        };
        let residual_report = ocp.residual_report(&decision, &mut residual_dynamics)?;
        let collocation_norm =
            residual_norm(&residual_report.residuals[..residual_report.collocation_residual_count]);
        assert!(
            collocation_norm <= problem.max_residual_norm,
            "brachistochrone HS residual norm {collocation_norm:e} exceeded {}",
            problem.max_residual_norm
        );
        let boundary_norm = residual_norm(
            &residual_report.residuals[residual_report.collocation_residual_count
                ..residual_report.collocation_residual_count
                    + residual_report.boundary_residual_count],
        );
        assert!(
            boundary_norm <= problem.max_terminal_error_m,
            "brachistochrone boundary residual {boundary_norm:e} exceeded {}",
            problem.max_terminal_error_m
        );

        let mut transcription_dynamics = move |_: f64, state: &[f64], control: &[f64]| {
            brachistochrone_dynamics(problem.gravity_m_s2, state, control)
        };
        let segments = ocp.segments_from_decision(&decision)?;
        let transcription = evaluate_hermite_simpson_transcription(
            &segments,
            &mut transcription_dynamics,
            unit_lagrange,
        )?;
        let relative_time_error = ((transcription.simpson_lagrange_cost - problem.optimal_time_s)
            / problem.optimal_time_s)
            .abs();
        assert!(
            relative_time_error <= problem.relative_time_tolerance,
            "brachistochrone time relative error {relative_time_error:e} exceeded {}",
            problem.relative_time_tolerance
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_brachistochrone_dense_sqp_restores_perturbed_cycloid()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = BrachistochroneBenchmarkTable::parse()?;
        let problem = table.problem;
        let ocp = brachistochrone_ocp(problem, problem.optimal_time_s)?;
        let (node_states, midpoint_states, node_controls, midpoint_controls) =
            brachistochrone_decision_blocks(problem);
        let exact = ocp.layout.pack_decision_vector(
            &node_states,
            &midpoint_states,
            &node_controls,
            &midpoint_controls,
        )?;
        let initial = perturb_brachistochrone_state_decision(&ocp.layout, &exact);
        let (lower, upper) = brachistochrone_state_restoration_bounds(&ocp.layout, &exact);
        let solver = crate::sqp::DenseSqpSolver {
            residual_tolerance: problem.dense_sqp_max_residual_norm,
            step_tolerance: 1.0e-14,
            regularization: 1.0e-12,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let report = ocp.solve_dense_sqp(
            &initial,
            &lower,
            &upper,
            &solver,
            BrachistochroneDynamics {
                gravity_m_s2: problem.gravity_m_s2,
            },
        )?;

        assert!(
            report.sqp_solution.converged,
            "brachistochrone dense SQP did not converge: norm = {:e}, iterations = {}",
            report.sqp_solution.residual_norm, report.sqp_solution.iterations
        );
        assert!(
            report.residual_report.residual_norm <= problem.dense_sqp_max_residual_norm,
            "brachistochrone dense SQP residual norm {:e} exceeded {}",
            report.residual_report.residual_norm,
            problem.dense_sqp_max_residual_norm
        );
        assert!(report.max_decision_bound_violation <= 1.0e-10);
        let max_solution_delta = exact
            .iter()
            .zip(report.sqp_solution.variables.iter())
            .map(|(expected, actual)| (expected - actual).abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_solution_delta <= problem.dense_sqp_max_solution_delta,
            "brachistochrone dense SQP solution delta {max_solution_delta:e} exceeded {}",
            problem.dense_sqp_max_solution_delta
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_brachistochrone_dense_sqp_restores_free_duration()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = BrachistochroneBenchmarkTable::parse()?;
        let problem = table.problem;
        let ocp = brachistochrone_ocp(problem, problem.optimal_time_s)?;
        let (node_states, midpoint_states, node_controls, midpoint_controls) =
            brachistochrone_decision_blocks(problem);
        let exact_decision = ocp.layout.pack_decision_vector(
            &node_states,
            &midpoint_states,
            &node_controls,
            &midpoint_controls,
        )?;
        let initial_decision = perturb_brachistochrone_state_decision(&ocp.layout, &exact_decision);
        let (decision_lower, decision_upper) =
            brachistochrone_state_restoration_bounds(&ocp.layout, &exact_decision);
        let solver = crate::sqp::DenseSqpSolver {
            residual_tolerance: problem.free_time_dense_sqp_max_residual_norm,
            step_tolerance: 1.0e-14,
            regularization: 1.0e-12,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let report = ocp.solve_dense_sqp_with_free_final_time(
            problem.optimal_time_s * 1.02,
            problem.optimal_time_s * 0.8,
            problem.optimal_time_s * 1.2,
            &initial_decision,
            &decision_lower,
            &decision_upper,
            &solver,
            BrachistochroneDynamics {
                gravity_m_s2: problem.gravity_m_s2,
            },
        )?;

        assert!(
            report.sqp_solution.converged,
            "free-duration brachistochrone SQP did not converge: norm = {:e}, iterations = {}",
            report.sqp_solution.residual_norm, report.sqp_solution.iterations
        );
        let relative_time_error =
            ((report.final_time_s - problem.optimal_time_s) / problem.optimal_time_s).abs();
        assert!(
            relative_time_error <= problem.free_time_dense_sqp_relative_time_tolerance,
            "free-duration brachistochrone time relative error {relative_time_error:e} exceeded {}",
            problem.free_time_dense_sqp_relative_time_tolerance
        );
        assert!(
            report.residual_report.residual_norm <= problem.free_time_dense_sqp_max_residual_norm,
            "free-duration brachistochrone residual norm {:e} exceeded {}",
            report.residual_report.residual_norm,
            problem.free_time_dense_sqp_max_residual_norm
        );
        assert!(report.max_final_time_bound_violation <= 1.0e-12);
        assert!(report.max_decision_bound_violation <= 1.0e-10);
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_brachistochrone_free_control_time_objective_recovers_cycloid()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = BrachistochroneBenchmarkTable::parse()?;
        let problem = table.problem;
        let ocp = brachistochrone_ocp(problem, problem.optimal_time_s)?;
        let (node_states, midpoint_states, node_controls, midpoint_controls) =
            brachistochrone_decision_blocks(problem);
        let exact_decision = ocp.layout.pack_decision_vector(
            &node_states,
            &midpoint_states,
            &node_controls,
            &midpoint_controls,
        )?;
        let initial_decision =
            perturb_brachistochrone_free_control_decision(&ocp.layout, &exact_decision);
        let (decision_lower, decision_upper) = brachistochrone_free_control_bounds(&ocp.layout);
        let solver = crate::sqp::DenseSqpSolver {
            max_iterations: 120,
            residual_tolerance: problem.free_time_dense_sqp_max_residual_norm,
            step_tolerance: 1.0e-6,
            regularization: 1.0e-3,
            objective_residual_penalty: 100.0,
            max_line_search_backtracks: 24,
            min_line_search_scale: 1.0e-10,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let report = ocp.solve_dense_sqp_with_free_final_time_objective(
            problem.optimal_time_s * 1.02,
            problem.optimal_time_s * 0.8,
            problem.optimal_time_s * 1.2,
            1.0,
            &initial_decision,
            &decision_lower,
            &decision_upper,
            &solver,
            BrachistochroneDynamics {
                gravity_m_s2: problem.gravity_m_s2,
            },
        )?;

        let relative_time_error =
            ((report.final_time_s - problem.optimal_time_s) / problem.optimal_time_s).abs();
        assert!(
            relative_time_error <= problem.free_time_dense_sqp_relative_time_tolerance,
            "free-control brachistochrone time relative error {relative_time_error:e} exceeded {}",
            problem.free_time_dense_sqp_relative_time_tolerance
        );
        assert!(
            report.residual_report.residual_norm <= problem.free_time_dense_sqp_max_residual_norm,
            "free-control brachistochrone residual norm {:e} exceeded {}",
            report.residual_report.residual_norm,
            problem.free_time_dense_sqp_max_residual_norm
        );
        assert!(report.max_final_time_bound_violation <= 1.0e-12);
        assert!(report.max_decision_bound_violation <= 1.0e-10);
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_free_final_time_objective_drives_scalar_integrator_to_bound() {
        let ocp = unit_interval_integrator_ocp(1).expect("OCP builds");
        let initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.5], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let mut lower = vec![-10.0; ocp.layout.variable_count];
        let mut upper = vec![10.0; ocp.layout.variable_count];
        for index in ocp.layout.node_control_offset..ocp.layout.variable_count {
            lower[index] = 0.0;
            upper[index] = 2.0;
        }
        let solver = crate::sqp::DenseSqpSolver {
            residual_tolerance: 1.0e-10,
            step_tolerance: 1.0e-8,
            regularization: 1.0e-10,
            max_iterations: 40,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let report = ocp
            .solve_dense_sqp_with_free_final_time_objective(
                1.0,
                0.5,
                1.5,
                1.0,
                &initial,
                &lower,
                &upper,
                &solver,
                integrator_control_dynamics,
            )
            .expect("free-final-time objective wrapper solves");

        assert!(
            report.sqp_solution.converged,
            "objective SQP did not converge: merit {:e}, residual {:e}, iterations {}",
            report.sqp_solution.merit,
            report.sqp_solution.residual_norm,
            report.sqp_solution.iterations
        );
        assert!(
            report.residual_report.residual_norm < 2.0e-5,
            "residual norm {:e}, final time {:e}, merit {:e}",
            report.residual_report.residual_norm,
            report.final_time_s,
            report.sqp_solution.merit
        );
        assert!(
            (report.final_time_s - 0.5).abs() < 1.0e-7,
            "final time {:e}",
            report.final_time_s
        );
        assert!(report.max_final_time_bound_violation < 1.0e-10);
        assert!(report.max_decision_bound_violation < 1.0e-8);
        assert!(report.max_hard_path_violation < 1.0e-8);
        let node_controls =
            &report.decision[ocp.layout.node_control_offset..ocp.layout.midpoint_control_offset];
        let midpoint_controls = &report.decision[ocp.layout.midpoint_control_offset..];
        assert!(
            node_controls
                .iter()
                .all(|control| (*control - 2.0).abs() < 1.0e-4)
        );
        assert!(
            midpoint_controls
                .iter()
                .all(|control| (*control - 2.0).abs() < 1.0e-4)
        );
    }

    #[test]
    fn hermite_simpson_cross_checks_t1_shared_ascent_case() -> Result<(), Box<dyn std::error::Error>>
    {
        let table = CrossTierToleranceTable::parse()?;
        let problem = &table.problem;
        let mut config = TwoBodyApogeeTargeting::wgs84(
            problem.initial_radius_m,
            problem.target_apogee_radius_m,
            problem.initial_tangential_speed_m_s,
        );
        config.coast_duration_s = problem.coast_duration_s;
        config.step_s = problem.step_s;
        config.corrector = DifferentialCorrector {
            max_iterations: problem.max_iterations,
            residual_tolerance: problem.residual_tolerance,
            ..DifferentialCorrector::default()
        };

        let t0_report = correct_two_body_apogee(&config)?;
        let initial_state = TwoBodyCartesianState::new(
            [problem.initial_radius_m, 0.0, 0.0],
            [0.0, t0_report.correction.free_variables[0], 0.0],
        )?;
        let nodes = seed_two_body_multiple_shooting_nodes(
            initial_state,
            &problem.segment_durations_s,
            problem.step_s,
            WGS84_MU_M3_S2,
        )?;
        let t0_terminal_position = t0_report.terminal_state.position_eci_m();
        let t0_terminal_velocity = t0_report.terminal_state.velocity_eci_m_s();
        let condition = TerminalCondition::RendezvousState {
            position_eci_m: t0_terminal_position,
            velocity_eci_m_s: t0_terminal_velocity,
        };
        let mut perturbed = nodes.clone();
        perturbed[1].state.position_eci_m[0] += 25.0;
        perturbed[1].state.velocity_eci_m_s[1] -= 0.025;
        perturbed[2].state.position_eci_m[1] -= 40.0;
        perturbed[2].state.velocity_eci_m_s[0] += 0.03;
        perturbed[3].state.position_eci_m[0] += 10.0;
        perturbed[3].state.velocity_eci_m_s[1] -= 0.01;
        let corrector = MultipleShootingCorrector {
            max_iterations: problem.max_iterations,
            defect_tolerance: problem.residual_tolerance,
            max_step_norm: 100.0,
            ..MultipleShootingCorrector::default()
        };
        let t1_correction = corrector.solve_two_body_terminal_condition(
            &condition,
            &perturbed,
            &problem.segment_durations_s,
            problem.step_s,
            WGS84_MU_M3_S2,
        )?;
        assert!(t0_report.correction.converged, "{t0_report:?}");
        assert!(t1_correction.converged, "{t1_correction:?}");

        let time_nodes_s = time_nodes_from_durations(&problem.segment_durations_s);
        let node_states = flattened_node_states(&t1_correction.nodes);
        let midpoint_states = two_body_midpoint_states(
            &t1_correction.nodes,
            &problem.segment_durations_s,
            problem.step_s,
            WGS84_MU_M3_S2,
        )?;
        let terminal_target = [
            t0_terminal_position[0],
            t0_terminal_position[1],
            t0_terminal_position[2],
            t0_terminal_velocity[0],
            t0_terminal_velocity[1],
            t0_terminal_velocity[2],
        ];
        let ocp = HermiteSimpsonFixedMeshOcp::new(
            time_nodes_s,
            6,
            0,
            terminal_boundary_constraints(problem.segment_durations_s.len(), terminal_target)?,
        )?;
        let decision = ocp
            .layout
            .pack_decision_vector(&node_states, &midpoint_states, &[], &[])?;
        let mut dynamics =
            |_: f64, state: &[f64], control: &[f64]| two_body_dynamics(state, control);
        let report = ocp.residual_report(&decision, &mut dynamics)?;

        let terminal = ocp
            .layout
            .node_state(&decision, problem.segment_durations_s.len())?;
        table.check_metric(
            "t2_terminal_position_delta_norm_m",
            norm3([
                terminal[0] - t0_terminal_position[0],
                terminal[1] - t0_terminal_position[1],
                terminal[2] - t0_terminal_position[2],
            ]),
        )?;
        table.check_metric(
            "t2_terminal_velocity_delta_norm_m_s",
            norm3([
                terminal[3] - t0_terminal_velocity[0],
                terminal[4] - t0_terminal_velocity[1],
                terminal[5] - t0_terminal_velocity[2],
            ]),
        )?;
        table.check_metric(
            "t2_hs_collocation_residual_norm",
            residual_norm(&report.residuals[..report.collocation_residual_count]),
        )?;
        assert_eq!(report.boundary_residual_count, 6);
        Ok(())
    }

    #[test]
    fn hermite_simpson_rejects_bad_dimensions_and_times() {
        let bad_time =
            HermiteSimpsonSegment::new(0.0, 0.25, 1.0, &[0.0], &[0.0], &[0.0], &[], &[], &[]);
        assert!(bad_time.is_err());

        let bad_state =
            HermiteSimpsonSegment::new(0.0, 0.5, 1.0, &[0.0], &[0.0, 1.0], &[0.0], &[], &[], &[]);
        assert!(bad_state.is_err());
    }

    #[test]
    fn fixed_mesh_ocp_layout_assembles_exact_collocation_residuals() {
        let ocp = unit_interval_integrator_ocp(2).expect("OCP builds");
        let layout = &ocp.layout;
        let decision = layout
            .pack_decision_vector(
                &[0.0, 0.25, 1.0],
                &[0.0625, 0.5625],
                &[0.0, 1.0, 2.0],
                &[0.5, 1.5],
            )
            .expect("decision packs");
        let mut dynamics = integrator_control_dynamics;
        let report = ocp
            .residual_report(&decision, &mut dynamics)
            .expect("residuals evaluate");

        assert_eq!(report.collocation_residual_count, 4);
        assert_eq!(report.boundary_residual_count, 2);
        assert!(report.residual_norm < 1.0e-14);
        assert_eq!(
            ocp.segments_from_decision(&decision)
                .expect("segments")
                .len(),
            2
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_residual_model_feeds_dense_sqp_solver() {
        let ocp = unit_interval_integrator_ocp(1).expect("OCP builds");
        let layout = &ocp.layout;
        let initial = layout
            .pack_decision_vector(&[0.0, 0.25], &[0.2], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let lower = vec![-10.0, -10.0, -10.0, 1.0, 1.0, 1.0];
        let upper = vec![10.0, 10.0, 10.0, 1.0, 1.0, 1.0];
        let solver = crate::sqp::DenseSqpSolver {
            residual_tolerance: 1.0e-10,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let solution = solver
            .solve(&initial, &lower, &upper, |decision: &[f64]| {
                let mut dynamics = integrator_control_dynamics;
                ocp.residuals(decision, &mut dynamics)
            })
            .expect("SQP solves fixed mesh");

        assert!(solution.converged);
        assert!(solution.residual_norm < 1.0e-10);
        assert!((solution.variables[0] - 0.0).abs() < 1.0e-8);
        assert!((solution.variables[1] - 1.0).abs() < 1.0e-8);
        assert!((solution.variables[2] - 0.5).abs() < 1.0e-8);
    }

    #[test]
    fn fixed_mesh_ocp_rejects_malformed_layout_and_boundaries() {
        let bad_times = HermiteSimpsonFixedMeshOcp::new(vec![0.0, 0.0], 1, 0, Vec::new());
        assert!(bad_times.is_err());

        let bad_boundary = HermiteSimpsonFixedMeshOcp::new(
            vec![0.0, 1.0],
            1,
            0,
            vec![HermiteSimpsonBoundaryConstraint::new(2, 0, 0.0).expect("boundary")],
        );
        assert!(bad_boundary.is_err());

        let layout = HermiteSimpsonDecisionLayout::new(1, 1, 0).expect("layout");
        let bad_decision = layout.pack_decision_vector(&[0.0], &[0.5], &[], &[]);
        assert!(bad_decision.is_err());
    }

    #[test]
    fn fixed_mesh_ocp_appends_path_and_quadratic_control_objective_residuals() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_path_constraints(vec![
                HermiteSimpsonSoftPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    Some(0.6),
                    Some(0.8),
                    4.0,
                )
                .expect("path constraint"),
            ])
            .expect("path constraints validate")
            .with_quadratic_control_costs(vec![
                HermiteSimpsonQuadraticControlCost::new(0, 2.0).expect("control cost"),
            ])
            .expect("control costs validate");
        let decision = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.5], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let mut dynamics = integrator_control_dynamics;
        let report = ocp
            .residual_report(&decision, &mut dynamics)
            .expect("residuals evaluate");

        assert_eq!(report.collocation_residual_count, 2);
        assert_eq!(report.boundary_residual_count, 2);
        assert_eq!(report.path_residual_count, 1);
        assert_eq!(report.objective_residual_count, 3);
        assert!((report.residuals[4] + 0.2).abs() < 1.0e-14);
        assert!((report.quadratic_control_cost - 2.0).abs() < 1.0e-14);
        assert!((report.residual_norm * report.residual_norm - 2.04).abs() < 1.0e-12);
    }

    #[test]
    fn fixed_mesh_ocp_transcribes_hard_path_bounds_to_dense_inequalities() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_hard_path_constraints(vec![
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    Some(0.25),
                    Some(0.75),
                )
                .expect("hard path bound"),
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Node(1),
                    HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
                    None,
                    Some(1.5),
                )
                .expect("hard path bound"),
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
                    Some(0.5),
                    None,
                )
                .expect("hard path bound"),
            ])
            .expect("hard path constraints validate");
        let report = ocp
            .hard_path_inequality_report()
            .expect("hard inequalities transcribe");

        assert_eq!(report.variable_count, 6);
        assert_eq!(report.inequality_count, 4);
        assert_eq!(report.inequality_rhs, vec![0.75, -0.25, 1.5, -0.5]);
        assert_eq!(report.inequality_matrix[2], 1.0);
        assert_eq!(report.inequality_matrix[8], -1.0);
        assert_eq!(report.inequality_matrix[16], 1.0);
        assert_eq!(report.inequality_matrix[23], -1.0);

        let decision = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.8], &[1.0, 2.0], &[0.25])
            .expect("decision packs");
        assert_eq!(
            report.lhs(&decision).expect("lhs"),
            vec![0.8, -0.8, 2.0, -0.25]
        );
        assert_eq!(
            report.violations(&decision).expect("violations"),
            vec![0.050000000000000044, 0.0, 0.5, 0.25]
        );
        assert!((report.max_violation(&decision).expect("max violation") - 0.5).abs() < 1.0e-14);
    }

    #[test]
    fn fixed_mesh_ocp_transcribes_nonlinear_hard_path_inequalities_with_sparse_jacobian() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_nonlinear_hard_path_constraints(vec![
                HermiteSimpsonNonlinearHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    vec![Some(0.5), None],
                    vec![Some(0.75), Some(1.0)],
                )
                .expect("nonlinear hard path bound"),
            ])
            .expect("nonlinear hard path constraints validate");
        let decision = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.5], &[1.0, 1.0], &[1.0])
            .expect("decision packs");

        let mut path_function = quadratic_path_function;
        let report = ocp
            .nonlinear_hard_path_inequality_report(&decision, &mut path_function)
            .expect("nonlinear hard inequalities evaluate");

        assert_eq!(report.variable_count, 6);
        assert_eq!(report.path_value_count, 2);
        assert_eq!(report.inequality_count, 3);
        assert_eq!(report.path_values, vec![1.25, 0.5]);
        assert_eq!(report.inequality_values, vec![0.5, -0.75, -0.5]);
        assert_eq!(
            report.violations().expect("violations"),
            vec![0.5, 0.0, 0.0]
        );
        assert!((report.max_violation - 0.5).abs() < 1.0e-14);

        let mut path_function = quadratic_path_function;
        let jacobian = ocp
            .sparse_nonlinear_hard_path_inequality_jacobian_report(
                &decision,
                &mut path_function,
                1.0e-6,
                1.0e-10,
            )
            .expect("nonlinear hard path Jacobian evaluates");

        assert_eq!(jacobian.row_count, 3);
        assert_eq!(jacobian.column_count, 6);
        assert_path_jacobian_entry(&jacobian, 0, 2, 1.0);
        assert_path_jacobian_entry(&jacobian, 0, 5, 1.0);
        assert_path_jacobian_entry(&jacobian, 1, 2, -1.0);
        assert_path_jacobian_entry(&jacobian, 1, 5, -1.0);
        assert_path_jacobian_entry(&jacobian, 2, 2, 1.0);
        assert_path_jacobian_entry(&jacobian, 2, 5, 0.5);
        assert_path_jacobian_entry(&jacobian, 0, 0, 0.0);

        let dense = jacobian.to_dense_row_major().expect("dense expansion");
        assert!((dense[2 * jacobian.column_count + 5] - 0.5).abs() < 1.0e-8);
    }

    #[test]
    fn fixed_mesh_ocp_emits_optimized_reference_profile_and_iload_payload() {
        let ocp =
            HermiteSimpsonFixedMeshOcp::new(vec![0.0, 2.0], 3, 0, Vec::new()).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[100.0, 10.0, 0.0, 120.0, 12.0, 0.2],
                &[110.0, 11.0, 0.1],
                &[],
                &[],
            )
            .expect("decision packs");
        let mapping = HermiteSimpsonReferenceProfileMapping::new(0, 1, 2);

        let report = ocp
            .optimized_reference_profile(&decision, mapping)
            .expect("reference profile emits");

        assert_eq!(report.sample_count, 3);
        assert_eq!(report.start_time_s, 0.0);
        assert_eq!(report.end_time_s, 2.0);
        assert_eq!(report.samples[0].time_s, 0.0);
        assert_eq!(report.samples[1].time_s, 1.0);
        assert_eq!(report.samples[2].time_s, 2.0);
        assert_eq!(report.samples[1].radius_m, 110.0);
        assert_eq!(report.samples[1].speed_m_s, 11.0);
        assert_eq!(report.samples[1].flight_path_angle_rad, 0.1);

        let payload = ocp
            .optimized_reference_iload_payload(
                &decision,
                mapping,
                &TerminalCondition::ApogeeRadius { radius_m: 120.0 },
                42,
                "openbmp-trajopt-test".to_string(),
                SynthesisMetadata {
                    scenario_digest: "scenario-sha256".to_string(),
                    source_revision: "test-revision".to_string(),
                    method: "hermite_simpson_fixed_mesh".to_string(),
                },
                Vec::new(),
                Vec::new(),
            )
            .expect("I-load payload emits");
        let encoded = crate::iload::encode_iload_payload(&payload).expect("payload encodes");
        let decoded = crate::iload::decode_iload_payload(&encoded).expect("payload decodes");

        assert_eq!(decoded.header.synthesis_seed, 42);
        assert_eq!(decoded.metadata.method, "hermite_simpson_fixed_mesh");
        assert_eq!(decoded.reference_profile, report.samples);

        let certificate_hamiltonian = crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: 3,
                control_dimension: 1,
                times_s: &[0.0, 1.0, 2.0],
                states: &[0.0; 9],
                controls: &[0.0; 3],
                costates: &[0.0; 9],
            },
            |_time_s, _state, _control| Ok(vec![0.0, 0.0, 0.0]),
            |_time_s, _state, _control| Ok(0.0),
        )
        .expect("certificate Hamiltonian evaluates");
        let certificate = crate::iload::ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            crate::iload::ILoadOptimalityEvidenceKind::HermiteSimpsonPmpKkt,
            &certificate_hamiltonian,
            1.0e-12,
        )
        .expect("certificate validates");
        let certified_payload = ocp
            .optimized_reference_iload_payload_with_optimality_certificate(
                &decision,
                mapping,
                &TerminalCondition::ApogeeRadius { radius_m: 120.0 },
                43,
                "openbmp-trajopt-test".to_string(),
                SynthesisMetadata {
                    scenario_digest: "scenario-sha256".to_string(),
                    source_revision: "test-revision".to_string(),
                    method: "hermite_simpson_fixed_mesh_with_pmp_certificate".to_string(),
                },
                Vec::new(),
                Vec::new(),
                certificate,
            )
            .expect("certified I-load payload emits");
        let decoded_certified = crate::iload::decode_iload_payload(
            &crate::iload::encode_iload_payload(&certified_payload)
                .expect("certified payload encodes"),
        )
        .expect("certified payload decodes");
        assert_eq!(
            decoded_certified
                .optimality_certificate
                .as_ref()
                .expect("certificate present")
                .evidence_kind,
            crate::iload::ILoadOptimalityEvidenceKind::HermiteSimpsonPmpKkt
        );
    }

    #[test]
    fn fixed_mesh_ocp_reference_profile_rejects_bad_mapping_and_nonfinite_samples() {
        let ocp =
            HermiteSimpsonFixedMeshOcp::new(vec![0.0, 2.0], 3, 0, Vec::new()).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[100.0, 10.0, 0.0, 120.0, 12.0, 0.2],
                &[110.0, 11.0, 0.1],
                &[],
                &[],
            )
            .expect("decision packs");

        assert!(
            ocp.optimized_reference_profile(
                &decision,
                HermiteSimpsonReferenceProfileMapping::new(0, 1, 3),
            )
            .is_err()
        );

        let nonfinite = ocp.layout.pack_decision_vector(
            &[100.0, 10.0, 0.0, 120.0, 12.0, 0.2],
            &[110.0, f64::NAN, 0.1],
            &[],
            &[],
        );
        assert!(nonfinite.is_err());

        let bad_payload = ocp.optimized_reference_iload_payload(
            &decision,
            HermiteSimpsonReferenceProfileMapping::new(0, 1, 2),
            &TerminalCondition::ApogeeRadius { radius_m: 120.0 },
            42,
            " ".to_string(),
            SynthesisMetadata {
                scenario_digest: "scenario-sha256".to_string(),
                source_revision: "test-revision".to_string(),
                method: "hermite_simpson_fixed_mesh".to_string(),
            },
            Vec::new(),
            Vec::new(),
        );
        assert!(bad_payload.is_err());
    }

    #[test]
    fn fixed_mesh_ocp_merges_hard_path_constraints_into_decision_bounds() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_hard_path_constraints(vec![
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    Some(0.25),
                    Some(0.75),
                )
                .expect("hard path bound"),
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Node(1),
                    HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
                    None,
                    Some(1.5),
                )
                .expect("hard path bound"),
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
                    Some(0.5),
                    None,
                )
                .expect("hard path bound"),
            ])
            .expect("hard path constraints validate");
        let lower = vec![-10.0; ocp.layout.variable_count];
        let upper = vec![10.0; ocp.layout.variable_count];
        let bounds = ocp
            .decision_bounds_with_hard_path_constraints(&lower, &upper)
            .expect("bounds merge");

        assert_eq!(bounds.variable_count, 6);
        assert_eq!(bounds.applied_bound_count, 4);
        assert_eq!(bounds.constrained_variable_count, 3);
        assert_eq!(bounds.lower_bounds[2], 0.25);
        assert_eq!(bounds.upper_bounds[2], 0.75);
        assert_eq!(bounds.upper_bounds[4], 1.5);
        assert_eq!(bounds.lower_bounds[5], 0.5);

        let violating = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.8], &[1.0, 2.0], &[0.25])
            .expect("decision packs");
        assert!((bounds.max_violation(&violating).expect("violation") - 0.5).abs() < 1.0e-14);
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_hard_path_bounds_feed_dense_sqp_solver() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_hard_path_constraints(vec![
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    Some(0.45),
                    Some(0.6),
                )
                .expect("hard path bound"),
            ])
            .expect("hard path constraints validate");
        let initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.55], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let mut lower = vec![-10.0; ocp.layout.variable_count];
        let mut upper = vec![10.0; ocp.layout.variable_count];
        lower[3] = 1.0;
        lower[4] = 1.0;
        lower[5] = 1.0;
        upper[3] = 1.0;
        upper[4] = 1.0;
        upper[5] = 1.0;
        let bounds = ocp
            .decision_bounds_with_hard_path_constraints(&lower, &upper)
            .expect("bounds merge");
        let solver = crate::sqp::DenseSqpSolver {
            residual_tolerance: 1.0e-8,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let solution = solver
            .solve(
                &initial,
                &bounds.lower_bounds,
                &bounds.upper_bounds,
                |decision: &[f64]| {
                    let mut dynamics = integrator_control_dynamics;
                    ocp.residuals(decision, &mut dynamics)
                },
            )
            .expect("SQP solves fixed mesh with hard bounds");

        assert!(solution.converged);
        assert!(solution.residual_norm < 1.0e-8);
        assert!(bounds.max_violation(&solution.variables).expect("bounds") < 1.0e-8);
        assert!((solution.variables[2] - 0.5).abs() < 1.0e-8);
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_dense_sqp_wrapper_reports_residuals_and_bound_violations() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_hard_path_constraints(vec![
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    Some(0.45),
                    Some(0.6),
                )
                .expect("hard path bound"),
            ])
            .expect("hard path constraints validate");
        let initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.55], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let mut lower = vec![-10.0; ocp.layout.variable_count];
        let mut upper = vec![10.0; ocp.layout.variable_count];
        lower[3] = 1.0;
        lower[4] = 1.0;
        lower[5] = 1.0;
        upper[3] = 1.0;
        upper[4] = 1.0;
        upper[5] = 1.0;
        let solver = crate::sqp::DenseSqpSolver {
            residual_tolerance: 1.0e-8,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let report = ocp
            .solve_dense_sqp(
                &initial,
                &lower,
                &upper,
                &solver,
                integrator_control_dynamics,
            )
            .expect("fixed-mesh dense SQP wrapper solves");

        assert!(report.sqp_solution.converged);
        assert!(report.residual_report.residual_norm < 1.0e-8);
        assert!(report.max_decision_bound_violation < 1.0e-8);
        assert!(report.max_hard_path_violation < 1.0e-8);
        assert_eq!(report.decision_bounds.applied_bound_count, 2);
        assert_eq!(report.residual_report.collocation_residual_count, 2);
        assert_eq!(report.residual_report.boundary_residual_count, 2);
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_dense_sqp_wrapper_is_repeat_deterministic() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_hard_path_constraints(vec![
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    Some(0.45),
                    Some(0.6),
                )
                .expect("hard path bound"),
            ])
            .expect("hard path constraints validate");
        let initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.55], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let mut lower = vec![-10.0; ocp.layout.variable_count];
        let mut upper = vec![10.0; ocp.layout.variable_count];
        lower[3] = 1.0;
        lower[4] = 1.0;
        lower[5] = 1.0;
        upper[3] = 1.0;
        upper[4] = 1.0;
        upper[5] = 1.0;
        let solver = crate::sqp::DenseSqpSolver {
            residual_tolerance: 1.0e-8,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let first = ocp
            .solve_dense_sqp(
                &initial,
                &lower,
                &upper,
                &solver,
                integrator_control_dynamics,
            )
            .expect("first fixed-mesh dense SQP wrapper solve succeeds");
        let second = ocp
            .solve_dense_sqp(
                &initial,
                &lower,
                &upper,
                &solver,
                integrator_control_dynamics,
            )
            .expect("second fixed-mesh dense SQP wrapper solve succeeds");

        assert!(first.sqp_solution.converged);
        assert_eq!(first.sqp_solution.variables, second.sqp_solution.variables);
        assert_eq!(first.sqp_solution.residual, second.sqp_solution.residual);
        assert_eq!(
            first.sqp_solution.residual_norm.to_bits(),
            second.sqp_solution.residual_norm.to_bits()
        );
        assert_eq!(
            first.sqp_solution.iterations,
            second.sqp_solution.iterations
        );
        assert_eq!(first.residual_report, second.residual_report);
        assert_eq!(first.decision_bounds, second.decision_bounds);
        assert_eq!(
            first.max_decision_bound_violation.to_bits(),
            second.max_decision_bound_violation.to_bits()
        );
        assert_eq!(
            first.max_hard_path_violation.to_bits(),
            second.max_hard_path_violation.to_bits()
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn dense_sqp_sparse_kkt_certificate_attaches_to_optimized_reference_iload()
    -> Result<(), TrajoptError> {
        let ocp = HermiteSimpsonFixedMeshOcp::new(
            vec![0.0, 1.0],
            3,
            1,
            vec![
                HermiteSimpsonBoundaryConstraint::new(0, 0, 0.0)?,
                HermiteSimpsonBoundaryConstraint::new(1, 0, 1.0)?,
                HermiteSimpsonBoundaryConstraint::new(0, 1, 10.0)?,
                HermiteSimpsonBoundaryConstraint::new(1, 1, 10.0)?,
                HermiteSimpsonBoundaryConstraint::new(0, 2, 0.1)?,
                HermiteSimpsonBoundaryConstraint::new(1, 2, 0.1)?,
            ],
        )?;
        let exact = ocp.layout.pack_decision_vector(
            &[0.0, 10.0, 0.1, 1.0, 10.0, 0.1],
            &[0.5, 10.0, 0.1],
            &[1.0, 1.0],
            &[1.0],
        )?;
        let mut initial = exact.clone();
        initial[3] = 0.95;
        initial[7] = 10.05;
        let mut lower = vec![-20.0; ocp.layout.variable_count];
        let mut upper = vec![20.0; ocp.layout.variable_count];
        for index in ocp.layout.node_control_offset..ocp.layout.variable_count {
            lower[index] = 1.0;
            upper[index] = 1.0;
        }
        let solver = crate::sqp::DenseSqpSolver {
            residual_tolerance: 1.0e-10,
            step_tolerance: 1.0e-12,
            regularization: 1.0e-12,
            ..crate::sqp::DenseSqpSolver::default()
        };
        let sqp_report = ocp.solve_dense_sqp(
            &initial,
            &lower,
            &upper,
            &solver,
            three_state_reference_dynamics,
        )?;
        let mut certificate_dynamics = three_state_reference_dynamics;
        let mut certificate_lagrange = zero_lagrange;
        let certificate_report = ocp.dense_sqp_pmp_kkt_certificate_report(
            &sqp_report,
            crate::pmp::PmpDirectCostateConvention::default(),
            1.0e-9,
            &mut certificate_dynamics,
            &mut certificate_lagrange,
            1.0e-6,
            1.0e-12,
            2.5e-1,
            1.0e-12,
        )?;
        let pmp_times_s = certificate_report.costate_convention_report.times_s.clone();
        let pmp_costates = certificate_report
            .costate_convention_report
            .pmp_costates
            .clone();
        let certificate_report = certificate_report.with_pmp_costate_comparison(
            &pmp_times_s,
            &pmp_costates,
            crate::pmp::PmpCostateComparisonSettings {
                costate_tolerance: 1.0e-12,
                ..Default::default()
            },
        )?;
        let mut stationarity_dynamics = three_state_reference_dynamics;
        let mut stationarity_lagrange = zero_lagrange;
        let certificate_report = certificate_report.with_pmp_control_stationarity(
            crate::pmp::PmpHamiltonianControlStationaritySettings {
                gradient_tolerance: 1.0e-9,
                ..Default::default()
            },
            &mut stationarity_dynamics,
            &mut stationarity_lagrange,
        )?;

        assert!(sqp_report.sqp_solution.converged);
        assert!(sqp_report.residual_report.residual_norm < 1.0e-10);
        assert_eq!(
            certificate_report
                .sparse_kkt_step_report
                .residual_multipliers
                .len(),
            sqp_report.residual_report.residuals.len()
        );
        assert_eq!(certificate_report.costate_convention_report.sample_count, 2);
        assert_eq!(certificate_report.hamiltonian_report.sample_count, 2);
        assert!(
            certificate_report
                .costate_comparison_report
                .as_ref()
                .expect("costate comparison attached")
                .within_tolerance
        );
        assert!(
            certificate_report
                .control_stationarity_report
                .as_ref()
                .expect("stationarity attached")
                .within_tolerance
        );
        assert_eq!(
            certificate_report
                .optimality_certificate
                .max_costate_norm_difference,
            Some(0.0)
        );

        let payload = ocp.optimized_reference_iload_payload_with_optimality_certificate(
            &sqp_report.sqp_solution.variables,
            HermiteSimpsonReferenceProfileMapping::new(0, 1, 2),
            &TerminalCondition::ApogeeRadius { radius_m: 1.0 },
            201,
            "openbmp-trajopt-hs-dense-sqp-test".to_owned(),
            SynthesisMetadata {
                scenario_digest: "sha256:hs-dense-sqp-pmp-kkt-certificate".to_owned(),
                source_revision: "test".to_owned(),
                method: "hermite_simpson_dense_sqp_sparse_kkt_pmp_certificate".to_owned(),
            },
            Vec::new(),
            Vec::new(),
            certificate_report.optimality_certificate.clone(),
        )?;
        let decoded =
            crate::iload::decode_iload_payload(&crate::iload::encode_iload_payload(&payload)?)?;
        assert_eq!(decoded.metadata.method, payload.metadata.method);
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present"),
            &certificate_report.optimality_certificate
        );
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present")
                .max_control_stationarity_gradient_norm,
            Some(
                certificate_report
                    .control_stationarity_report
                    .as_ref()
                    .expect("stationarity attached")
                    .max_gradient_norm
            )
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn sparse_kkt_sqp_pmp_kkt_certificate_attaches_to_optimized_reference_iload()
    -> Result<(), TrajoptError> {
        let ocp = HermiteSimpsonFixedMeshOcp::new(
            vec![0.0, 1.0],
            3,
            1,
            vec![
                HermiteSimpsonBoundaryConstraint::new(0, 0, 0.0)?,
                HermiteSimpsonBoundaryConstraint::new(1, 0, 1.0)?,
                HermiteSimpsonBoundaryConstraint::new(0, 1, 10.0)?,
                HermiteSimpsonBoundaryConstraint::new(1, 1, 10.0)?,
                HermiteSimpsonBoundaryConstraint::new(0, 2, 0.1)?,
                HermiteSimpsonBoundaryConstraint::new(1, 2, 0.1)?,
            ],
        )?
        .with_hard_path_constraints(vec![HermiteSimpsonHardPathConstraint::new(
            HermiteSimpsonSamplePoint::Midpoint(0),
            HermiteSimpsonScalarVariable::StateComponent { component: 0 },
            None,
            Some(0.5),
        )?])?;
        let exact = ocp.layout.pack_decision_vector(
            &[0.0, 10.0, 0.1, 1.0, 10.0, 0.1],
            &[0.5, 10.0, 0.1],
            &[1.0, 1.0],
            &[1.0],
        )?;
        let mut lower = vec![-20.0; ocp.layout.variable_count];
        let mut upper = vec![20.0; ocp.layout.variable_count];
        for index in ocp.layout.node_control_offset..ocp.layout.variable_count {
            lower[index] = 1.0;
            upper[index] = 1.0;
        }
        let settings = SparseKktSqpSettings {
            max_iterations: 12,
            residual_tolerance: 1.0e-10,
            step_tolerance: 1.0e-12,
            kkt_regularization: 2.5e-1,
            ..SparseKktSqpSettings::default()
        };
        let decision_bounds = ocp.decision_bounds_with_hard_path_constraints(&lower, &upper)?;
        let max_decision_bound_violation = decision_bounds.max_violation(&exact)?;
        let hard_path_inequalities = ocp.hard_path_inequality_report()?;
        let max_hard_path_violation = hard_path_inequalities.max_violation(&exact)?;
        let mut residual_dynamics = three_state_reference_dynamics;
        let residual_report = ocp.residual_report(&exact, &mut residual_dynamics)?;
        let mut kkt_dynamics = three_state_reference_dynamics;
        let final_kkt_step_report = ocp.sparse_augmented_kkt_step_report(
            &exact,
            &mut kkt_dynamics,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
        )?;
        let mut constrained_dynamics = three_state_reference_dynamics;
        let final_constrained_kkt_step_report = ocp.sparse_constrained_kkt_step_report(
            &exact,
            &mut constrained_dynamics,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
            settings.residual_tolerance,
        )?;
        let sqp_report = HermiteSimpsonFixedMeshSparseKktSqpReport {
            decision: exact.clone(),
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
        let mut certificate_dynamics = three_state_reference_dynamics;
        let mut certificate_lagrange = zero_lagrange;
        let certificate_report = ocp.sparse_kkt_sqp_pmp_kkt_certificate_report(
            &sqp_report,
            crate::pmp::PmpDirectCostateConvention::default(),
            1.0e-9,
            &mut certificate_dynamics,
            &mut certificate_lagrange,
        )?;
        let pmp_times_s = certificate_report.costate_convention_report.times_s.clone();
        let pmp_costates = certificate_report
            .costate_convention_report
            .pmp_costates
            .clone();
        let certificate_report = certificate_report.with_pmp_costate_comparison(
            &pmp_times_s,
            &pmp_costates,
            crate::pmp::PmpCostateComparisonSettings {
                costate_tolerance: 1.0e-12,
                ..Default::default()
            },
        )?;
        let mut stationarity_dynamics = three_state_reference_dynamics;
        let mut stationarity_lagrange = zero_lagrange;
        let certificate_report = certificate_report.with_pmp_control_stationarity(
            crate::pmp::PmpHamiltonianControlStationaritySettings {
                gradient_tolerance: 1.0e-9,
                ..Default::default()
            },
            &mut stationarity_dynamics,
            &mut stationarity_lagrange,
        )?;

        assert!(sqp_report.converged);
        assert!(sqp_report.residual_report.residual_norm < settings.residual_tolerance);
        assert!(sqp_report.final_constrained_kkt_step_report.is_some());
        assert!(certificate_report.used_constrained_kkt_step_report);
        assert!(
            certificate_report
                .final_constrained_kkt_step_report
                .as_ref()
                .expect("constrained report")
                .constrained_kkt
                .active_inequality_row_count
                > 0
        );
        assert_eq!(certificate_report.costate_convention_report.sample_count, 2);
        assert_eq!(certificate_report.hamiltonian_report.sample_count, 2);
        assert!(
            certificate_report
                .costate_comparison_report
                .as_ref()
                .expect("costate comparison attached")
                .within_tolerance
        );
        assert!(
            certificate_report
                .control_stationarity_report
                .as_ref()
                .expect("stationarity attached")
                .within_tolerance
        );
        assert_eq!(
            certificate_report
                .optimality_certificate
                .max_costate_norm_difference,
            Some(0.0)
        );

        let payload = ocp.optimized_reference_iload_payload_with_optimality_certificate(
            &sqp_report.decision,
            HermiteSimpsonReferenceProfileMapping::new(0, 1, 2),
            &TerminalCondition::ApogeeRadius { radius_m: 1.0 },
            211,
            "openbmp-trajopt-hs-sparse-sqp-test".to_owned(),
            SynthesisMetadata {
                scenario_digest: "sha256:hs-sparse-sqp-pmp-kkt-certificate".to_owned(),
                source_revision: "test".to_owned(),
                method: "hermite_simpson_sparse_sqp_pmp_kkt_certificate".to_owned(),
            },
            Vec::new(),
            Vec::new(),
            certificate_report.optimality_certificate.clone(),
        )?;
        let decoded =
            crate::iload::decode_iload_payload(&crate::iload::encode_iload_payload(&payload)?)?;
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present"),
            &certificate_report.optimality_certificate
        );
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present")
                .max_control_stationarity_gradient_norm,
            Some(
                certificate_report
                    .control_stationarity_report
                    .as_ref()
                    .expect("stationarity attached")
                    .max_gradient_norm
            )
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_dense_constrained_sqp_wrapper_consumes_nonlinear_hard_path_rows() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_nonlinear_hard_path_constraints(vec![
                HermiteSimpsonNonlinearHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    vec![None],
                    vec![Some(0.25)],
                )
                .expect("nonlinear hard path bound"),
            ])
            .expect("nonlinear hard path constraints validate");
        let initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.5], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let lower = vec![-10.0; ocp.layout.variable_count];
        let upper = vec![10.0; ocp.layout.variable_count];
        let solver = crate::sqp::DenseSqpSolver {
            max_iterations: 12,
            residual_tolerance: 1.0e-8,
            inequality_tolerance: 1.0e-8,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let report = ocp
            .solve_dense_sqp_with_nonlinear_hard_path_constraints(
                &initial,
                &lower,
                &upper,
                &solver,
                integrator_control_dynamics,
                control_square_path_function,
            )
            .expect("fixed-mesh constrained dense SQP wrapper solves");

        assert!(report.sqp_solution.converged);
        assert!(report.residual_report.residual_norm < 1.0e-8);
        assert!(report.max_decision_bound_violation < 1.0e-8);
        assert!(report.max_hard_path_violation < 1.0e-8);
        assert!(report.max_nonlinear_hard_path_violation < 1.0e-8);
        assert!(report.nonlinear_hard_path_report.max_violation < 1.0e-8);
        let midpoint_control = sample_scalar(
            &ocp.layout,
            &report.sqp_solution.variables,
            HermiteSimpsonSamplePoint::Midpoint(0),
            HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
        )
        .expect("midpoint control");
        assert!(midpoint_control.abs() <= 0.5 + 1.0e-7);
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_dense_sqp_wrapper_rejects_start_outside_merged_bounds() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_hard_path_constraints(vec![
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    Some(0.5),
                    Some(0.6),
                )
                .expect("hard path bound"),
            ])
            .expect("hard path constraints validate");
        let initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.4], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let lower = vec![-10.0; ocp.layout.variable_count];
        let upper = vec![10.0; ocp.layout.variable_count];

        assert!(
            ocp.solve_dense_sqp(
                &initial,
                &lower,
                &upper,
                &crate::sqp::DenseSqpSolver::default(),
                integrator_control_dynamics,
            )
            .is_err()
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_scalar_integrator_min_energy_benchmark_matches_analytic_solution() {
        let table = ScalarIntegratorBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = unit_interval_integrator_ocp(table.segment_count).expect("OCP builds");
        let zero_decision = vec![0.0; ocp.layout.variable_count];
        let mut dynamics = integrator_control_dynamics;
        let residual_at_zero = ocp
            .residuals(&zero_decision, &mut dynamics)
            .expect("zero residual evaluates");
        let mut dynamics = integrator_control_dynamics;
        let jacobian = ocp
            .sparse_residual_jacobian_report(&zero_decision, &mut dynamics, 1.0e-6, 1.0e-12)
            .expect("Jacobian evaluates");
        let equality_matrix = jacobian.to_dense_row_major().expect("dense Jacobian");
        let equality_rhs = residual_at_zero
            .iter()
            .map(|residual| -residual)
            .collect::<Vec<_>>();
        let problem = crate::sqp::DenseQpProblem::new(
            ocp.layout.variable_count,
            scalar_integrator_min_energy_hessian(&ocp),
            vec![0.0; ocp.layout.variable_count],
            equality_matrix,
            equality_rhs,
            Vec::new(),
            Vec::new(),
        )
        .expect("benchmark QP validates");

        let solution = crate::sqp::solve_dense_qp(&problem).expect("benchmark QP solves");
        let mut dynamics = integrator_control_dynamics;
        let residual_report = ocp
            .residual_report(&solution.primal, &mut dynamics)
            .expect("benchmark residuals evaluate");
        let cost = scalar_integrator_control_cost(&ocp, &solution.primal).expect("cost evaluates");

        assert!(residual_report.residual_norm <= table.max_residual_norm);
        assert!((cost - table.expected_cost).abs() <= table.cost_tolerance);
        assert_scalar_integrator_analytic_solution(&ocp, &solution.primal, &table);
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn hermite_simpson_dense_qp_duals_feed_pmp_costate_convention_bridge()
    -> Result<(), TrajoptError> {
        let table = ScalarIntegratorBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = unit_interval_integrator_ocp(table.segment_count)?
            .with_quadratic_control_costs(vec![HermiteSimpsonQuadraticControlCost::new(0, 1.0)?])?;
        let mut dynamics = integrator_control_dynamics;

        let report =
            ocp.solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)?;
        let multiplier_report = ocp.dense_qp_kkt_residual_multiplier_report(&report)?;
        let costate_report = ocp.dense_qp_kkt_node_costate_estimate_report(&report)?;
        let pmp_problem = crate::pmp::PmpScalarMinEnergyProblem {
            initial_time_s: 0.0,
            final_time_s: 1.0,
            initial_state: 0.0,
            terminal_state: 1.0,
            control_weight: 2.0,
        };
        let pmp_solution = crate::pmp::solve_pmp_scalar_min_energy_shooting(
            pmp_problem,
            crate::pmp::PmpScalarMinEnergyShootingSettings {
                sample_count: ocp.layout.segment_count + 1,
                ..Default::default()
            },
        )?;
        let pmp_convention = crate::pmp::PmpDirectCostateConvention {
            raw_to_pmp_scale: -2.0,
        };
        let convention_report =
            ocp.dense_qp_pmp_costate_convention_report(&report, pmp_convention)?;
        let pmp_times_s: Vec<_> = pmp_solution
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
        let comparison = crate::pmp::compare_pmp_costate_profiles(
            costate_report.state_dimension,
            &pmp_times_s,
            &pmp_costates,
            &convention_report.times_s,
            &convention_report.pmp_costates,
            crate::pmp::PmpCostateComparisonSettings {
                costate_tolerance: 1.0e-7,
                ..Default::default()
            },
        )?;
        let mut hamiltonian_dynamics = integrator_control_dynamics;
        let mut hamiltonian_lagrange =
            |_: f64, _: &[f64], control: &[f64]| Ok(control[0] * control[0]);
        let hamiltonian_report = ocp.dense_qp_pmp_hamiltonian_report(
            &report,
            pmp_convention,
            &mut hamiltonian_dynamics,
            &mut hamiltonian_lagrange,
        )?;

        assert!(pmp_solution.converged);
        assert_eq!(
            report.equality_row_count,
            report.residual_report.collocation_residual_count
                + report.residual_report.boundary_residual_count
        );
        assert_eq!(
            multiplier_report.residual_multiplier_count,
            report.residual_report.residuals.len()
        );
        assert_eq!(multiplier_report.path_multiplier_count, 0);
        assert_eq!(
            multiplier_report.objective_multipliers,
            vec![0.0; multiplier_report.objective_multiplier_count]
        );
        assert_eq!(
            costate_report.node_samples.len(),
            ocp.layout.segment_count + 1
        );
        assert_eq!(convention_report.sample_count, ocp.layout.segment_count + 1);
        assert!(convention_report.max_abs_raw_costate > 0.0);
        assert!(comparison.within_tolerance);
        assert!(hamiltonian_report.max_abs_hamiltonian_drift < 1.0e-7);
        let certificate = crate::iload::ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            crate::iload::ILoadOptimalityEvidenceKind::HermiteSimpsonPmpKkt,
            &hamiltonian_report,
            1.0e-7,
        )?
        .with_costate_comparison(&comparison)?;
        assert_eq!(certificate.sample_count, convention_report.sample_count);
        assert_eq!(
            certificate.max_costate_norm_difference,
            Some(comparison.max_costate_norm_difference)
        );
        let certified_payload = crate::iload::ILoadPayload {
            header: crate::iload::ILoadHeader::new_with_objective(
                &TerminalCondition::ApogeeRadius { radius_m: 1.0 },
                crate::iload::ILoadObjectiveClass::OptimizedReference,
                101,
                "openbmp-trajopt-hs-dense-qp-test".to_owned(),
            ),
            metadata: crate::iload::SynthesisMetadata {
                scenario_digest: "sha256:hs-dense-qp-pmp-certificate".to_owned(),
                source_revision: "test".to_owned(),
                method: "hermite_simpson_dense_qp_pmp_certificate".to_owned(),
            },
            event_bindings_postcard: Vec::new(),
            gain_tables: Vec::new(),
            reference_profile: hamiltonian_report
                .samples
                .iter()
                .map(|sample| crate::iload::ReferenceProfileSample {
                    time_s: sample.time_s,
                    radius_m: sample.state[0],
                    speed_m_s: sample.control[0],
                    flight_path_angle_rad: 0.0,
                })
                .collect(),
            optimality_certificate: Some(certificate.clone()),
        };
        let decoded_payload = crate::iload::decode_iload_payload(
            &crate::iload::encode_iload_payload(&certified_payload)?,
        )?;
        assert_eq!(
            decoded_payload
                .optimality_certificate
                .as_ref()
                .expect("certificate present"),
            &certificate
        );
        assert!(
            (hamiltonian_report.initial_hamiltonian
                - pmp_solution.hamiltonian_report.initial_hamiltonian)
                .abs()
                < 1.0e-7
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_bryson_denham_state_constrained_benchmark_matches_active_arc() {
        let table = BrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = bryson_denham_ocp(table.problem).expect("OCP builds");
        let (problem, inequalities) =
            bryson_denham_benchmark_qp(&ocp).expect("benchmark QP validates");

        let solution = crate::sqp::solve_dense_qp(&problem).expect("benchmark QP solves");
        let mut residual_dynamics = bryson_denham_dynamics;
        let residual_report = ocp
            .residual_report(&solution.primal, &mut residual_dynamics)
            .expect("benchmark residuals evaluate");
        let max_path_violation = inequalities
            .max_violation(&solution.primal)
            .expect("path violation evaluates");

        assert!(
            residual_report.residual_norm <= table.problem.max_residual_norm,
            "Bryson-Denham residual norm {} exceeded {}",
            residual_report.residual_norm,
            table.problem.max_residual_norm
        );
        assert!(
            max_path_violation <= table.problem.max_path_violation,
            "Bryson-Denham path violation {max_path_violation:e} exceeded {}",
            table.problem.max_path_violation
        );
        assert!(
            (solution.objective - table.problem.expected_cost).abs()
                <= table.problem.cost_tolerance,
            "Bryson-Denham cost {} expected {}",
            solution.objective,
            table.problem.expected_cost
        );
        assert_bryson_denham_active_arc_solution(&ocp, &solution.primal, table.problem);
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_bryson_denham_dense_quadratic_qp_wrapper_matches_active_arc() {
        let table = BrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = bryson_denham_quadratic_ocp(table.problem).expect("OCP builds");
        let mut dynamics = bryson_denham_dynamics;

        let report = ocp
            .solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)
            .expect("dense quadratic QP wrapper solves");

        assert!(
            report.equality_residual_norm <= table.problem.max_residual_norm,
            "Bryson-Denham wrapper equality norm {} exceeded {}",
            report.equality_residual_norm,
            table.problem.max_residual_norm
        );
        assert!(
            report.max_hard_path_violation <= table.problem.max_path_violation,
            "Bryson-Denham wrapper path violation {} exceeded {}",
            report.max_hard_path_violation,
            table.problem.max_path_violation
        );
        assert!(
            (report.qp_solution.objective - table.problem.expected_cost).abs()
                <= table.problem.cost_tolerance,
            "Bryson-Denham wrapper cost {} expected {}",
            report.qp_solution.objective,
            table.problem.expected_cost
        );
        assert_eq!(
            report.equality_row_count,
            report.residual_report.collocation_residual_count
                + report.residual_report.boundary_residual_count
        );
        assert_eq!(
            report.hard_path_inequality_report.inequality_count,
            2 * table.problem.segment_count + 1
        );
        assert_bryson_denham_active_arc_solution(&ocp, &report.qp_solution.primal, table.problem);
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_bryson_denham_dense_solution_feeds_sparse_constrained_kkt_report() {
        let table = BrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = bryson_denham_quadratic_ocp(table.problem).expect("OCP builds");
        let mut dynamics = bryson_denham_dynamics;
        let dense_report = ocp
            .solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)
            .expect("dense quadratic QP wrapper solves");
        let settings = SparseKktSqpSettings {
            residual_tolerance: table.problem.max_path_violation,
            ..SparseKktSqpSettings::default()
        };
        let mut sparse_dynamics = bryson_denham_dynamics;

        let sparse_report = ocp
            .sparse_constrained_kkt_step_report(
                &dense_report.qp_solution.primal,
                &mut sparse_dynamics,
                settings.finite_difference_step,
                settings.jacobian_drop_tolerance,
                settings.kkt_regularization,
                settings.kkt_drop_tolerance,
                settings.residual_tolerance,
            )
            .expect("Bryson-Denham sparse constrained KKT report assembles");
        let equality_row_count = sparse_report.residual_report.collocation_residual_count
            + sparse_report.residual_report.boundary_residual_count;
        let equality_residual_norm =
            residual_norm(&sparse_report.residual_report.residuals[..equality_row_count]);
        let constrained_multiplier_report = ocp
            .sparse_constrained_kkt_residual_multiplier_report(&sparse_report)
            .expect("constrained sparse KKT residual multipliers slice");

        assert!(
            equality_residual_norm <= table.problem.max_residual_norm,
            "Bryson-Denham sparse-KKT equality norm {equality_residual_norm} exceeded {}",
            table.problem.max_residual_norm
        );
        assert!(
            sparse_report
                .hard_path_inequality_report
                .max_violation(&dense_report.qp_solution.primal)
                .expect("path violation evaluates")
                <= table.problem.max_path_violation
        );
        assert!(
            (dense_report.qp_solution.objective - table.problem.expected_cost).abs()
                <= table.problem.cost_tolerance,
            "Bryson-Denham sparse-KKT cost {} expected {}",
            dense_report.qp_solution.objective,
            table.problem.expected_cost
        );
        assert!(
            sparse_report.constrained_kkt.active_inequality_row_count > 0,
            "Bryson-Denham sparse-KKT report should retain active path rows"
        );
        assert_eq!(
            sparse_report.active_inequality_multipliers.len(),
            sparse_report.constrained_kkt.active_inequality_row_count
        );
        assert!(
            sparse_report.max_linearized_active_inequality_violation
                <= table.problem.max_path_violation
        );
        assert_eq!(
            constrained_multiplier_report.residual_multiplier_count,
            sparse_report.residual_report.residuals.len()
        );
        assert_eq!(
            constrained_multiplier_report.objective_multiplier_count,
            sparse_report.residual_report.objective_residual_count
        );
        assert_bryson_denham_active_arc_solution(
            &ocp,
            &dense_report.qp_solution.primal,
            table.problem,
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_bryson_denham_sparse_constrained_kkt_feeds_pmp_diagnostics() {
        let table = BrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = bryson_denham_quadratic_ocp(table.problem).expect("OCP builds");
        let mut dynamics = bryson_denham_dynamics;
        let dense_report = ocp
            .solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)
            .expect("dense quadratic QP wrapper solves");
        let settings = SparseKktSqpSettings {
            residual_tolerance: table.problem.max_path_violation,
            ..SparseKktSqpSettings::default()
        };
        let mut sparse_dynamics = bryson_denham_dynamics;
        let sparse_report = ocp
            .sparse_constrained_kkt_step_report(
                &dense_report.qp_solution.primal,
                &mut sparse_dynamics,
                settings.finite_difference_step,
                settings.jacobian_drop_tolerance,
                settings.kkt_regularization,
                settings.kkt_drop_tolerance,
                settings.residual_tolerance,
            )
            .expect("Bryson-Denham sparse constrained KKT report assembles");
        let pmp_convention = crate::pmp::PmpDirectCostateConvention {
            raw_to_pmp_scale: 1.0,
        };

        let costate_report = ocp
            .sparse_constrained_kkt_node_costate_estimate_report(&sparse_report)
            .expect("Bryson-Denham constrained sparse-KKT costates evaluate");
        let pmp_costate_report = ocp
            .sparse_constrained_kkt_pmp_costate_convention_report(&sparse_report, pmp_convention)
            .expect("Bryson-Denham constrained sparse-KKT PMP costates evaluate");
        let mut hamiltonian_dynamics = bryson_denham_dynamics;
        let mut hamiltonian_lagrange = zero_lagrange;
        let hamiltonian_report = ocp
            .sparse_constrained_kkt_pmp_hamiltonian_report(
                &dense_report.qp_solution.primal,
                &sparse_report,
                pmp_convention,
                &mut hamiltonian_dynamics,
                &mut hamiltonian_lagrange,
            )
            .expect("Bryson-Denham constrained sparse-KKT PMP Hamiltonian evaluates");

        assert!(sparse_report.constrained_kkt.active_inequality_row_count > 0);
        assert_eq!(
            costate_report.node_samples.len(),
            table.problem.segment_count + 1
        );
        assert_eq!(
            pmp_costate_report.sample_count,
            costate_report.node_samples.len()
        );
        assert_eq!(pmp_costate_report.state_dimension, 2);
        assert_eq!(
            hamiltonian_report.sample_count,
            pmp_costate_report.sample_count
        );
        assert_eq!(hamiltonian_report.state_dimension, 2);
        assert_eq!(hamiltonian_report.control_dimension, 1);
        assert!(hamiltonian_report.max_abs_hamiltonian.is_finite());
        assert!(hamiltonian_report.max_abs_hamiltonian_drift.is_finite());
        assert!(
            pmp_costate_report
                .pmp_costates
                .iter()
                .all(|value| value.is_finite())
        );
        assert_bryson_denham_active_arc_solution(
            &ocp,
            &dense_report.qp_solution.primal,
            table.problem,
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_bryson_denham_sparse_constrained_kkt_certificate_attaches_to_iload() {
        let table = BrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = bryson_denham_quadratic_ocp(table.problem).expect("OCP builds");
        let mut dynamics = bryson_denham_dynamics;
        let dense_report = ocp
            .solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)
            .expect("dense quadratic QP wrapper solves");
        let settings = SparseKktSqpSettings {
            residual_tolerance: table.problem.max_path_violation,
            ..SparseKktSqpSettings::default()
        };
        let mut sparse_dynamics = bryson_denham_dynamics;
        let sparse_report = ocp
            .sparse_constrained_kkt_step_report(
                &dense_report.qp_solution.primal,
                &mut sparse_dynamics,
                settings.finite_difference_step,
                settings.jacobian_drop_tolerance,
                settings.kkt_regularization,
                settings.kkt_drop_tolerance,
                settings.residual_tolerance,
            )
            .expect("Bryson-Denham sparse constrained KKT report assembles");
        let pmp_convention = crate::pmp::PmpDirectCostateConvention {
            raw_to_pmp_scale: 1.0,
        };
        let mut certificate_dynamics = bryson_denham_dynamics;
        let mut certificate_lagrange = zero_lagrange;
        let certificate_report = ocp
            .sparse_constrained_kkt_pmp_kkt_certificate_report(
                &dense_report.qp_solution.primal,
                &sparse_report,
                pmp_convention,
                1.0e12,
                &mut certificate_dynamics,
                &mut certificate_lagrange,
            )
            .expect("Bryson-Denham constrained KKT PMP/KKT certificate builds");
        let pmp_times_s = certificate_report.costate_convention_report.times_s.clone();
        let pmp_costates = certificate_report
            .costate_convention_report
            .pmp_costates
            .clone();
        let certificate_report = certificate_report
            .with_pmp_costate_comparison(
                &pmp_times_s,
                &pmp_costates,
                crate::pmp::PmpCostateComparisonSettings::default(),
            )
            .expect("self costate-comparison summary attaches");
        let payload = ocp
            .optimized_reference_iload_payload_with_optimality_certificate(
                &dense_report.qp_solution.primal,
                HermiteSimpsonReferenceProfileMapping::new(0, 1, 1),
                &TerminalCondition::ApogeeRadius { radius_m: 1.0 },
                77,
                "openbmp-trajopt-bryson-denham-test".to_string(),
                SynthesisMetadata {
                    scenario_digest: "sha256:hs-bryson-denham-constrained-kkt-certificate"
                        .to_string(),
                    source_revision: "test-revision".to_string(),
                    method: "hermite_simpson_bryson_denham_constrained_kkt_pmp_certificate"
                        .to_string(),
                },
                Vec::new(),
                Vec::new(),
                certificate_report.optimality_certificate.clone(),
            )
            .expect("certified Bryson-Denham I-load payload emits");
        let decoded = crate::iload::decode_iload_payload(
            &crate::iload::encode_iload_payload(&payload)
                .expect("certified Bryson-Denham payload encodes"),
        )
        .expect("certified Bryson-Denham payload decodes");

        assert!(
            certificate_report
                .constrained_kkt_step_report
                .constrained_kkt
                .active_inequality_row_count
                > 0
        );
        assert_eq!(
            certificate_report.costate_convention_report.sample_count,
            table.problem.segment_count + 1
        );
        assert_eq!(
            certificate_report.hamiltonian_report.sample_count,
            certificate_report.costate_convention_report.sample_count
        );
        assert_eq!(
            certificate_report
                .costate_comparison_report
                .as_ref()
                .expect("comparison present")
                .max_costate_norm_difference,
            0.0
        );
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present"),
            &certificate_report.optimality_certificate
        );
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present")
                .evidence_kind,
            crate::iload::ILoadOptimalityEvidenceKind::HermiteSimpsonPmpKkt
        );
        assert_bryson_denham_active_arc_solution(
            &ocp,
            &dense_report.qp_solution.primal,
            table.problem,
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn fixed_mesh_ocp_bryson_denham_benchmark_qp_is_repeat_deterministic() {
        let table = BrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = bryson_denham_ocp(table.problem).expect("OCP builds");
        let (problem, inequalities) =
            bryson_denham_benchmark_qp(&ocp).expect("benchmark QP validates");

        let first = crate::sqp::solve_dense_qp(&problem).expect("first benchmark QP solves");
        let second = crate::sqp::solve_dense_qp(&problem).expect("second benchmark QP solves");

        assert_eq!(first.status, second.status);
        assert_eq!(first.primal, second.primal);
        assert_eq!(first.objective.to_bits(), second.objective.to_bits());

        let mut first_dynamics = bryson_denham_dynamics;
        let first_residual = ocp
            .residual_report(&first.primal, &mut first_dynamics)
            .expect("first residual evaluates");
        let mut second_dynamics = bryson_denham_dynamics;
        let second_residual = ocp
            .residual_report(&second.primal, &mut second_dynamics)
            .expect("second residual evaluates");
        let first_violation = inequalities
            .max_violation(&first.primal)
            .expect("first path violation evaluates");
        let second_violation = inequalities
            .max_violation(&second.primal)
            .expect("second path violation evaluates");

        assert_eq!(first_residual, second_residual);
        assert_eq!(first_violation.to_bits(), second_violation.to_bits());
        assert_bryson_denham_active_arc_solution(&ocp, &first.primal, table.problem);
    }

    #[test]
    fn fixed_mesh_ocp_rejects_conflicting_hard_path_decision_bounds() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_hard_path_constraints(vec![
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    Some(0.25),
                    Some(0.75),
                )
                .expect("hard path bound"),
            ])
            .expect("hard path constraints validate");
        let lower = vec![-10.0; ocp.layout.variable_count];
        let mut upper = vec![10.0; ocp.layout.variable_count];
        upper[2] = 0.2;

        assert!(
            ocp.decision_bounds_with_hard_path_constraints(&lower, &upper)
                .is_err()
        );
        assert!(
            ocp.decision_bounds_with_hard_path_constraints(&lower[..5], &upper)
                .is_err()
        );

        let malformed = HermiteSimpsonDecisionBoundsReport {
            variable_count: 6,
            lower_bounds: vec![0.0],
            upper_bounds: vec![1.0],
            applied_bound_count: 1,
            constrained_variable_count: 1,
        };
        let decision = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.5], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        assert!(malformed.max_violation(&decision).is_err());
    }

    #[test]
    fn fixed_mesh_ocp_sparse_jacobian_matches_scalar_integrator_residuals() {
        let ocp = unit_interval_integrator_ocp(1).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.5], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let mut dynamics = integrator_control_dynamics;
        let jacobian = ocp
            .sparse_residual_jacobian_report(&decision, &mut dynamics, 1.0e-6, 1.0e-9)
            .expect("Jacobian evaluates");

        assert_eq!(jacobian.row_count, 4);
        assert_eq!(jacobian.column_count, 6);
        assert_eq!(jacobian.column_offsets.len(), 7);
        assert_eq!(jacobian.values.len(), 12);
        assert_jacobian_entry(&jacobian, 0, 0, -1.0);
        assert_jacobian_entry(&jacobian, 0, 1, 1.0);
        assert_jacobian_entry(&jacobian, 0, 3, -1.0 / 6.0);
        assert_jacobian_entry(&jacobian, 0, 4, -1.0 / 6.0);
        assert_jacobian_entry(&jacobian, 0, 5, -4.0 / 6.0);
        assert_jacobian_entry(&jacobian, 1, 0, -0.5);
        assert_jacobian_entry(&jacobian, 1, 1, -0.5);
        assert_jacobian_entry(&jacobian, 1, 2, 1.0);
        assert_jacobian_entry(&jacobian, 1, 3, -1.0 / 8.0);
        assert_jacobian_entry(&jacobian, 1, 4, 1.0 / 8.0);
        assert_jacobian_entry(&jacobian, 2, 0, 1.0);
        assert_jacobian_entry(&jacobian, 3, 1, 1.0);
        assert_jacobian_entry(&jacobian, 2, 1, 0.0);

        let dense = jacobian.to_dense_row_major().expect("dense expansion");
        assert!((dense[3 * jacobian.column_count + 1] - 1.0).abs() < 1.0e-8);
    }

    #[test]
    fn fixed_mesh_ocp_analytic_sparse_jacobian_matches_finite_difference_report() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_path_constraints(vec![
                HermiteSimpsonSoftPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    Some(0.6),
                    Some(0.8),
                    4.0,
                )
                .expect("path constraint"),
            ])
            .expect("path constraints validate")
            .with_quadratic_control_costs(vec![
                HermiteSimpsonQuadraticControlCost::new(0, 2.0).expect("control cost"),
            ])
            .expect("control costs validate");
        let decision = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.4], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let mut finite_difference_dynamics = integrator_control_dynamics;
        let finite_difference = ocp
            .sparse_residual_jacobian_report(
                &decision,
                &mut finite_difference_dynamics,
                1.0e-6,
                1.0e-10,
            )
            .expect("finite-difference Jacobian evaluates");
        let mut analytic_dynamics = IntegratorControlDynamicsJacobian;
        let analytic = ocp
            .analytic_sparse_residual_jacobian_report(&decision, &mut analytic_dynamics, 1.0e-10)
            .expect("analytic Jacobian evaluates");

        assert_eq!(analytic.finite_difference_step.to_bits(), 0.0_f64.to_bits());
        assert_eq!(analytic.row_count, finite_difference.row_count);
        assert_eq!(analytic.column_count, finite_difference.column_count);
        let dense_finite_difference = finite_difference
            .to_dense_row_major()
            .expect("finite-difference dense Jacobian");
        let dense_analytic = analytic
            .to_dense_row_major()
            .expect("analytic dense Jacobian");
        assert_eq!(dense_analytic.len(), dense_finite_difference.len());
        for (index, (actual, expected)) in dense_analytic
            .iter()
            .zip(dense_finite_difference.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "analytic Jacobian index {index} expected {expected}, got {actual}"
            );
        }
        assert_jacobian_entry(&analytic, 0, 0, -1.0);
        assert_jacobian_entry(&analytic, 4, 2, 2.0);
    }

    #[test]
    fn fixed_mesh_ocp_analytic_sparse_jacobian_rejects_bad_dynamics_jacobian() {
        let ocp = unit_interval_integrator_ocp(1).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.5], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let mut dynamics = MalformedIntegratorControlDynamicsJacobian;

        assert!(
            ocp.analytic_sparse_residual_jacobian_report(&decision, &mut dynamics, 1.0e-10)
                .is_err()
        );
        let mut dynamics = IntegratorControlDynamicsJacobian;
        assert!(
            ocp.analytic_sparse_residual_jacobian_report(&decision[..5], &mut dynamics, 1.0e-10)
                .is_err()
        );
        assert!(
            ocp.analytic_sparse_residual_jacobian_report(&decision, &mut dynamics, -1.0)
                .is_err()
        );
    }

    #[test]
    fn fixed_mesh_ocp_autodiff_sparse_jacobian_matches_finite_difference_report() {
        let ocp = nonlinear_autodiff_ocp().expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[0.2, -0.1, 0.4, 0.3, -0.2, 0.1],
                &[0.31, 0.05, 0.08, 0.2],
                &[0.7, -0.4, 0.2],
                &[0.1, -0.3],
            )
            .expect("decision packs");
        let mut finite_difference_dynamics =
            CollocationAutoDiffDynamicsAdapter::new(NonlinearAutoDiffDynamics, 2, 1)
                .expect("AD adapter builds");
        let finite_difference = ocp
            .sparse_residual_jacobian_report(
                &decision,
                &mut finite_difference_dynamics,
                1.0e-6,
                1.0e-10,
            )
            .expect("finite-difference Jacobian evaluates");
        let mut autodiff_dynamics = NonlinearAutoDiffDynamics;
        let autodiff = ocp
            .autodiff_sparse_residual_jacobian_report(&decision, &mut autodiff_dynamics, 1.0e-12)
            .expect("autodiff Jacobian evaluates");

        assert_eq!(autodiff.finite_difference_step, 0.0);
        assert_eq!(autodiff.row_count, finite_difference.row_count);
        assert_eq!(autodiff.column_count, finite_difference.column_count);
        let dense_finite_difference = finite_difference
            .to_dense_row_major()
            .expect("finite-difference dense Jacobian");
        let dense_autodiff = autodiff
            .to_dense_row_major()
            .expect("autodiff dense Jacobian");
        for (index, (actual, expected)) in dense_autodiff
            .iter()
            .zip(dense_finite_difference.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 2.0e-6,
                "autodiff Jacobian index {index} expected {expected}, got {actual}"
            );
        }
    }

    #[test]
    fn fixed_mesh_ocp_autodiff_sparse_jacobian_rejects_bad_dynamics_output() {
        let ocp = nonlinear_autodiff_ocp().expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[0.2, -0.1, 0.4, 0.3, -0.2, 0.1],
                &[0.31, 0.05, 0.08, 0.2],
                &[0.7, -0.4, 0.2],
                &[0.1, -0.3],
            )
            .expect("decision packs");
        let mut short_output = ShortAutoDiffDynamicsOutput;
        assert!(
            ocp.autodiff_sparse_residual_jacobian_report(&decision, &mut short_output, 1.0e-12)
                .is_err()
        );
        let mut bad_derivative_dimension = BadAutoDiffDerivativeDimension;
        assert!(
            ocp.autodiff_sparse_residual_jacobian_report(
                &decision,
                &mut bad_derivative_dimension,
                1.0e-12,
            )
            .is_err()
        );
        assert!(CollocationAutoDiffScalar::new(f64::NAN, Vec::new()).is_err());
        assert!(CollocationAutoDiffScalar::variable(1.0, 1, 1).is_err());
        assert!(CollocationAutoDiffDynamicsAdapter::new(NonlinearAutoDiffDynamics, 0, 1).is_err());
    }

    #[test]
    fn fixed_mesh_ocp_colored_sparse_jacobian_matches_columnwise_report() {
        let ocp = unit_interval_integrator_ocp(2).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[0.0, 0.5, 1.0],
                &[0.25, 0.75],
                &[1.0, 1.0, 1.0],
                &[1.0, 1.0],
            )
            .expect("decision packs");

        let mut dynamics = integrator_control_dynamics;
        let columnwise = ocp
            .sparse_residual_jacobian_report(&decision, &mut dynamics, 1.0e-6, 1.0e-9)
            .expect("columnwise Jacobian evaluates");
        let mut dynamics = integrator_control_dynamics;
        let colored = ocp
            .colored_sparse_residual_jacobian_report(&decision, &mut dynamics, 1.0e-6, 1.0e-9)
            .expect("colored Jacobian evaluates");

        assert_eq!(colored.sparse_jacobian.row_count, columnwise.row_count);
        assert_eq!(
            colored.sparse_jacobian.column_count,
            columnwise.column_count
        );
        assert!(colored.color_count < ocp.layout.variable_count);
        assert!(colored.residual_evaluation_count < 1 + 2 * ocp.layout.variable_count);
        assert_eq!(colored.column_colors.len(), ocp.layout.variable_count);
        assert!(colored.structural_nonzero_count >= colored.sparse_jacobian.values.len());

        let dense_columnwise = columnwise.to_dense_row_major().expect("dense columnwise");
        let dense_colored = colored.to_dense_row_major().expect("dense colored");
        assert_eq!(dense_colored.len(), dense_columnwise.len());
        for (index, (colored_value, columnwise_value)) in dense_colored
            .iter()
            .zip(dense_columnwise.iter())
            .enumerate()
        {
            assert!(
                (colored_value - columnwise_value).abs() < 1.0e-8,
                "dense Jacobian index {index} expected {columnwise_value}, got {colored_value}"
            );
        }
    }

    #[test]
    fn fixed_mesh_ocp_sparse_gauss_newton_hessian_matches_dense_normal_matrix() {
        let ocp = unit_interval_integrator_ocp(2).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[0.0, 0.25, 1.0],
                &[0.0625, 0.5625],
                &[0.0, 1.0, 2.0],
                &[0.5, 1.5],
            )
            .expect("decision packs");
        let mut dynamics = integrator_control_dynamics;
        let jacobian = ocp
            .sparse_residual_jacobian_report(&decision, &mut dynamics, 1.0e-6, 1.0e-9)
            .expect("Jacobian evaluates");
        let regularization = 1.0e-3;
        let hessian = jacobian
            .sparse_gauss_newton_hessian_report(regularization, 1.0e-12)
            .expect("sparse Gauss-Newton Hessian assembles");
        let dense_jacobian = jacobian.to_dense_row_major().expect("dense Jacobian");
        let expected = dense_gauss_newton_hessian(
            jacobian.row_count,
            jacobian.column_count,
            &dense_jacobian,
            regularization,
        );
        let dense_hessian = hessian.to_dense_row_major().expect("dense Hessian");

        assert_eq!(hessian.row_count, jacobian.column_count);
        assert_eq!(hessian.column_count, jacobian.column_count);
        assert_eq!(hessian.residual_row_count, jacobian.row_count);
        assert_eq!(hessian.source_jacobian_nonzero_count, jacobian.values.len());
        assert_eq!(dense_hessian.len(), expected.len());
        for row in 0..hessian.row_count {
            for column in 0..hessian.column_count {
                let index = row * hessian.column_count + column;
                assert!(
                    (dense_hessian[index] - expected[index]).abs() < 1.0e-8,
                    "H({row}, {column}) expected {}, got {}",
                    expected[index],
                    dense_hessian[index]
                );
                assert!(
                    (hessian.coefficient(row, column).expect("coefficient") - expected[index])
                        .abs()
                        < 1.0e-8,
                    "coefficient H({row}, {column}) mismatch"
                );
            }
        }
    }

    #[test]
    fn fixed_mesh_ocp_sparse_gauss_newton_rhs_and_cg_match_dense_normal_system() {
        let ocp = unit_interval_integrator_ocp(2).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[0.0, 0.25, 1.0],
                &[0.0625, 0.5625],
                &[0.0, 1.0, 2.0],
                &[0.5, 1.5],
            )
            .expect("decision packs");
        let mut dynamics = integrator_control_dynamics;
        let residuals = ocp.residuals(&decision, &mut dynamics).expect("residuals");
        let jacobian = ocp
            .sparse_residual_jacobian_report(&decision, &mut dynamics, 1.0e-6, 1.0e-9)
            .expect("Jacobian evaluates");
        let dense_jacobian = jacobian.to_dense_row_major().expect("dense Jacobian");
        let rhs = jacobian
            .sparse_gauss_newton_step_rhs(&residuals)
            .expect("sparse RHS assembles");
        let expected_rhs = dense_gauss_newton_step_rhs(
            jacobian.row_count,
            jacobian.column_count,
            &dense_jacobian,
            &residuals,
        );
        for (index, (actual, expected)) in rhs.iter().zip(expected_rhs.iter()).enumerate() {
            assert!(
                (actual - expected).abs() < 1.0e-10,
                "rhs index {index} expected {expected}, got {actual}"
            );
        }

        let regularization = 2.5e-1;
        let hessian = jacobian
            .sparse_gauss_newton_hessian_report(regularization, 1.0e-12)
            .expect("sparse Hessian assembles");
        let dense_hessian = dense_gauss_newton_hessian(
            jacobian.row_count,
            jacobian.column_count,
            &dense_jacobian,
            regularization,
        );
        let known_solution = (0..jacobian.column_count)
            .map(|index| {
                if index % 2 == 0 {
                    0.05 * (index as f64 + 1.0)
                } else {
                    -0.03 * (index as f64 + 1.0)
                }
            })
            .collect::<Vec<_>>();
        let sparse_product = hessian
            .multiply_vector(&known_solution)
            .expect("sparse matvec");
        let dense_product =
            dense_matrix_vector(jacobian.column_count, &dense_hessian, &known_solution);
        for (index, (actual, expected)) in
            sparse_product.iter().zip(dense_product.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-10,
                "matvec index {index} expected {expected}, got {actual}"
            );
        }

        let zero = vec![0.0; jacobian.column_count];
        let solve = hessian
            .solve_conjugate_gradient(&sparse_product, &zero, jacobian.column_count * 4, 1.0e-11)
            .expect("CG solves sparse normal system");
        assert!(solve.converged);
        assert!(solve.iterations <= jacobian.column_count);
        assert!(solve.initial_residual_norm > solve.residual_norm);
        assert!(solve.residual_norm < 1.0e-9);
        for (index, (actual, expected)) in
            solve.solution.iter().zip(known_solution.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "CG solution index {index} expected {expected}, got {actual}"
            );
        }
    }

    #[test]
    fn fixed_mesh_ocp_sparse_augmented_kkt_matches_dense_system() {
        let ocp = unit_interval_integrator_ocp(2).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[0.0, 0.25, 1.0],
                &[0.0625, 0.5625],
                &[0.0, 1.0, 2.0],
                &[0.5, 1.5],
            )
            .expect("decision packs");
        let mut dynamics = integrator_control_dynamics;
        let jacobian = ocp
            .sparse_residual_jacobian_report(&decision, &mut dynamics, 1.0e-6, 1.0e-9)
            .expect("Jacobian evaluates");
        let dense_jacobian = jacobian.to_dense_row_major().expect("dense Jacobian");
        let regularization = 2.5e-1;
        let kkt = jacobian
            .sparse_augmented_kkt_report(regularization, 1.0e-12)
            .expect("sparse augmented KKT assembles");
        let expected = dense_augmented_kkt(
            jacobian.row_count,
            jacobian.column_count,
            &dense_jacobian,
            regularization,
        );
        let dense_kkt = kkt.to_dense_row_major().expect("dense KKT");

        assert_eq!(kkt.row_count, jacobian.column_count + jacobian.row_count);
        assert_eq!(kkt.column_count, kkt.row_count);
        assert_eq!(kkt.variable_count, jacobian.column_count);
        assert_eq!(kkt.residual_row_count, jacobian.row_count);
        assert_eq!(kkt.source_jacobian_nonzero_count, jacobian.values.len());
        for row in 0..kkt.row_count {
            for column in 0..kkt.column_count {
                let index = row * kkt.column_count + column;
                assert!(
                    (dense_kkt[index] - expected[index]).abs() < 1.0e-10,
                    "KKT({row}, {column}) expected {}, got {}",
                    expected[index],
                    dense_kkt[index]
                );
                assert!(
                    (kkt.coefficient(row, column).expect("coefficient") - expected[index]).abs()
                        < 1.0e-10,
                    "coefficient KKT({row}, {column}) mismatch"
                );
            }
        }

        let known_solution = (0..kkt.column_count)
            .map(|index| {
                if index % 2 == 0 {
                    0.04 * (index as f64 + 1.0)
                } else {
                    -0.02 * (index as f64 + 1.0)
                }
            })
            .collect::<Vec<_>>();
        let sparse_product = kkt.multiply_vector(&known_solution).expect("KKT matvec");
        let dense_product = dense_matrix_vector(kkt.column_count, &expected, &known_solution);
        for (index, (actual, expected)) in
            sparse_product.iter().zip(dense_product.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-10,
                "KKT matvec index {index} expected {expected}, got {actual}"
            );
        }

        let solve = kkt.solve_ldlt(&dense_product).expect("LDL solve");
        assert!(solve.residual_norm < 1.0e-9);
        assert!(solve.factorization_residual_norm < 1.0e-9);
        assert_eq!(solve.factor_diagonal.len(), kkt.column_count);
        for (index, (actual, expected)) in
            solve.solution.iter().zip(known_solution.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "LDL solution index {index} expected {expected}, got {actual}"
            );
        }
    }

    #[test]
    fn fixed_mesh_ocp_sparse_gauss_newton_step_report_matches_dense_normal_step() {
        let ocp = unit_interval_integrator_ocp(2).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[0.0, 0.25, 1.0],
                &[0.0625, 0.5625],
                &[0.0, 1.0, 2.0],
                &[0.5, 1.5],
            )
            .expect("decision packs");
        let mut dynamics = integrator_control_dynamics;
        let regularization = 2.5e-1;
        let report = ocp
            .sparse_gauss_newton_step_report(
                &decision,
                &mut dynamics,
                1.0e-6,
                1.0e-9,
                regularization,
                1.0e-12,
                ocp.layout.variable_count * 4,
                1.0e-12,
            )
            .expect("sparse Gauss-Newton step assembles");
        let dense_jacobian = report
            .sparse_jacobian
            .to_dense_row_major()
            .expect("dense Jacobian");
        let expected_hessian = dense_gauss_newton_hessian(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            regularization,
        );
        let expected_rhs = dense_gauss_newton_step_rhs(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            &report.residual_report.residuals,
        );
        let expected_step = crate::corrector::solve_linear_system(
            expected_hessian,
            expected_rhs.clone(),
            report.sparse_jacobian.column_count,
        )
        .expect("dense normal system solves");
        let expected_linearized_residual = dense_rectangular_matrix_vector(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            &expected_step,
        )
        .iter()
        .zip(report.residual_report.residuals.iter())
        .map(|(jacobian_step, residual)| residual + jacobian_step)
        .collect::<Vec<_>>();

        assert!(report.solve_report.converged);
        assert!(report.solve_report.residual_norm < 1.0e-9);
        assert!((report.step_norm - residual_norm(&expected_step)).abs() < 1.0e-9);
        for (index, (actual, expected)) in report.rhs.iter().zip(expected_rhs.iter()).enumerate() {
            assert!(
                (actual - expected).abs() < 1.0e-10,
                "rhs index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .solve_report
            .solution
            .iter()
            .zip(expected_step.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "step index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .linearized_residual
            .iter()
            .zip(expected_linearized_residual.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "linearized residual index {index} expected {expected}, got {actual}"
            );
        }
    }

    #[test]
    fn fixed_mesh_ocp_sparse_augmented_kkt_step_matches_dense_normal_step() {
        let ocp = unit_interval_integrator_ocp(2).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[0.0, 0.25, 1.0],
                &[0.0625, 0.5625],
                &[0.0, 1.0, 2.0],
                &[0.5, 1.5],
            )
            .expect("decision packs");
        let mut dynamics = integrator_control_dynamics;
        let regularization = 2.5e-1;
        let report = ocp
            .sparse_augmented_kkt_step_report(
                &decision,
                &mut dynamics,
                1.0e-6,
                1.0e-9,
                regularization,
                1.0e-12,
            )
            .expect("sparse augmented KKT step assembles");
        let dense_jacobian = report
            .sparse_jacobian
            .to_dense_row_major()
            .expect("dense Jacobian");
        let expected_hessian = dense_gauss_newton_hessian(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            regularization,
        );
        let expected_rhs = dense_gauss_newton_step_rhs(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            &report.residual_report.residuals,
        );
        let expected_step = crate::corrector::solve_linear_system(
            expected_hessian,
            expected_rhs,
            report.sparse_jacobian.column_count,
        )
        .expect("dense normal system solves");
        let expected_linearized_residual = dense_rectangular_matrix_vector(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            &expected_step,
        )
        .iter()
        .zip(report.residual_report.residuals.iter())
        .map(|(jacobian_step, residual)| residual + jacobian_step)
        .collect::<Vec<_>>();

        assert!(report.solve_report.residual_norm < 1.0e-9);
        assert!(report.solve_report.factorization_residual_norm < 1.0e-9);
        assert!((report.step_norm - residual_norm(&expected_step)).abs() < 1.0e-9);
        assert_eq!(report.rhs.len(), report.augmented_kkt.row_count);
        for rhs in &report.rhs[..report.sparse_jacobian.column_count] {
            assert!(rhs.abs() < 1.0e-15);
        }
        for (index, residual) in report.residual_report.residuals.iter().enumerate() {
            assert!(
                (report.rhs[report.sparse_jacobian.column_count + index] + residual).abs()
                    < 1.0e-15,
                "KKT RHS residual block mismatch at {index}"
            );
        }
        for (index, (actual, expected)) in report.step.iter().zip(expected_step.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "KKT step index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .linearized_residual
            .iter()
            .zip(expected_linearized_residual.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "linearized residual index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .residual_multipliers
            .iter()
            .zip(expected_linearized_residual.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "KKT residual multiplier index {index} expected {expected}, got {actual}"
            );
        }

        let multiplier_report = ocp
            .kkt_residual_multiplier_report(&report)
            .expect("KKT multiplier report slices");
        assert_eq!(multiplier_report.segment_count, 2);
        assert_eq!(multiplier_report.state_dimension, 1);
        assert_eq!(
            multiplier_report.collocation_multiplier_count,
            report.residual_report.collocation_residual_count
        );
        assert_eq!(
            multiplier_report.boundary_multiplier_count,
            report.residual_report.boundary_residual_count
        );
        assert_eq!(multiplier_report.collocation_samples.len(), 2);
        assert_eq!(
            multiplier_report.collocation_samples[0].dynamics_defect_multipliers,
            vec![report.residual_multipliers[0]]
        );
        assert_eq!(
            multiplier_report.collocation_samples[0].midpoint_defect_multipliers,
            vec![report.residual_multipliers[1]]
        );
        assert_eq!(
            multiplier_report.boundary_multipliers,
            report.residual_multipliers[report.residual_report.collocation_residual_count
                ..report.residual_report.collocation_residual_count
                    + report.residual_report.boundary_residual_count]
        );
        assert!(multiplier_report.max_abs_dynamics_multiplier >= 0.0);
        assert!(multiplier_report.max_abs_midpoint_multiplier >= 0.0);
        assert!(multiplier_report.max_abs_boundary_multiplier >= 0.0);

        let costate_report = ocp
            .kkt_node_costate_estimate_report(&report)
            .expect("KKT node costate estimate reports");
        assert_eq!(costate_report.segment_count, 2);
        assert_eq!(costate_report.state_dimension, 1);
        assert_eq!(costate_report.node_samples.len(), 3);
        assert_eq!(costate_report.node_samples[0].node_index, 0);
        assert_eq!(costate_report.node_samples[0].time_s, ocp.time_nodes_s[0]);
        assert_eq!(costate_report.node_samples[0].contributing_segment_count, 1);
        assert_eq!(
            costate_report.node_samples[0].costate,
            multiplier_report.collocation_samples[0].dynamics_defect_multipliers
        );
        assert_eq!(costate_report.node_samples[1].contributing_segment_count, 2);
        let expected_interior_costate = 0.5
            * (multiplier_report.collocation_samples[0].dynamics_defect_multipliers[0]
                + multiplier_report.collocation_samples[1].dynamics_defect_multipliers[0]);
        assert!(
            (costate_report.node_samples[1].costate[0] - expected_interior_costate).abs() < 1.0e-14
        );
        assert_eq!(costate_report.node_samples[2].contributing_segment_count, 1);
        assert_eq!(
            costate_report.node_samples[2].costate,
            multiplier_report.collocation_samples[1].dynamics_defect_multipliers
        );
        assert!(costate_report.max_abs_costate >= 0.0);
        let pmp_costate_report = ocp
            .sparse_kkt_pmp_costate_convention_report(
                &report,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
            )
            .expect("sparse KKT PMP costate convention report evaluates");
        assert_eq!(
            pmp_costate_report.sample_count,
            costate_report.node_samples.len()
        );
        assert_eq!(
            pmp_costate_report.state_dimension,
            costate_report.state_dimension
        );
        assert_eq!(pmp_costate_report.raw_to_pmp_scale, 1.0);
        for (pmp_sample, raw_sample) in pmp_costate_report
            .samples
            .iter()
            .zip(costate_report.node_samples.iter())
        {
            assert_eq!(pmp_sample.time_s.to_bits(), raw_sample.time_s.to_bits());
            assert_eq!(pmp_sample.raw_costate, raw_sample.costate);
            assert_eq!(pmp_sample.pmp_costate, raw_sample.costate);
        }

        let mut hamiltonian_dynamics = integrator_control_dynamics;
        let mut hamiltonian_lagrange = time_squared_lagrange;
        let hamiltonian_report = ocp
            .kkt_hamiltonian_report(
                &decision,
                &report,
                &mut hamiltonian_dynamics,
                &mut hamiltonian_lagrange,
            )
            .expect("KKT Hamiltonian report evaluates");
        assert_eq!(hamiltonian_report.segment_count, 2);
        assert_eq!(hamiltonian_report.state_dimension, 1);
        assert_eq!(hamiltonian_report.sample_count, 3);
        assert_eq!(hamiltonian_report.samples.len(), 3);
        let mut expected_max_abs_hamiltonian = 0.0_f64;
        let mut expected_max_abs_hamiltonian_drift = 0.0_f64;
        let first_hamiltonian = hamiltonian_report.samples[0].hamiltonian;
        for sample in &hamiltonian_report.samples {
            let expected_dynamics = decision[ocp.layout.node_control_offset + sample.node_index];
            assert_eq!(sample.dynamics, vec![expected_dynamics]);
            assert!((sample.lagrange_value - sample.time_s * sample.time_s).abs() < 1.0e-15);
            let expected_dot = sample.costate[0] * expected_dynamics;
            assert!((sample.costate_dot_dynamics - expected_dot).abs() < 1.0e-14);
            assert!((sample.hamiltonian - (sample.lagrange_value + expected_dot)).abs() < 1.0e-14);
            expected_max_abs_hamiltonian =
                expected_max_abs_hamiltonian.max(sample.hamiltonian.abs());
            expected_max_abs_hamiltonian_drift = expected_max_abs_hamiltonian_drift
                .max((sample.hamiltonian - first_hamiltonian).abs());
        }
        assert!(
            (hamiltonian_report.max_abs_hamiltonian - expected_max_abs_hamiltonian).abs() < 1.0e-14
        );
        assert!(
            (hamiltonian_report.max_abs_hamiltonian_drift - expected_max_abs_hamiltonian_drift)
                .abs()
                < 1.0e-14
        );
        let mut pmp_hamiltonian_dynamics = integrator_control_dynamics;
        let mut pmp_hamiltonian_lagrange = time_squared_lagrange;
        let pmp_hamiltonian_report = ocp
            .sparse_kkt_pmp_hamiltonian_report(
                &decision,
                &report,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
                &mut pmp_hamiltonian_dynamics,
                &mut pmp_hamiltonian_lagrange,
            )
            .expect("sparse KKT PMP Hamiltonian report evaluates");
        assert_eq!(
            pmp_hamiltonian_report.sample_count,
            hamiltonian_report.sample_count
        );
        assert_eq!(
            pmp_hamiltonian_report.state_dimension,
            hamiltonian_report.state_dimension
        );
        assert_eq!(
            pmp_hamiltonian_report.control_dimension,
            ocp.layout.control_dimension
        );
        assert!(
            (pmp_hamiltonian_report.max_abs_hamiltonian - hamiltonian_report.max_abs_hamiltonian)
                .abs()
                < 1.0e-14
        );
        assert!(
            (pmp_hamiltonian_report.max_abs_hamiltonian_drift
                - hamiltonian_report.max_abs_hamiltonian_drift)
                .abs()
                < 1.0e-14
        );
        for (pmp_sample, raw_sample) in pmp_hamiltonian_report
            .samples
            .iter()
            .zip(hamiltonian_report.samples.iter())
        {
            assert_eq!(pmp_sample.time_s.to_bits(), raw_sample.time_s.to_bits());
            assert!((pmp_sample.hamiltonian - raw_sample.hamiltonian).abs() < 1.0e-14);
        }

        let mut malformed = report.clone();
        malformed.residual_multipliers.pop();
        assert!(ocp.kkt_residual_multiplier_report(&malformed).is_err());
        assert!(ocp.kkt_node_costate_estimate_report(&malformed).is_err());
        assert!(
            ocp.sparse_kkt_pmp_costate_convention_report(
                &malformed,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
            )
            .is_err()
        );
        let mut malformed_dynamics = integrator_control_dynamics;
        let mut malformed_lagrange = time_squared_lagrange;
        assert!(
            ocp.kkt_hamiltonian_report(
                &decision,
                &malformed,
                &mut malformed_dynamics,
                &mut malformed_lagrange
            )
            .is_err()
        );
        let mut malformed_pmp_dynamics = integrator_control_dynamics;
        let mut malformed_pmp_lagrange = time_squared_lagrange;
        assert!(
            ocp.sparse_kkt_pmp_hamiltonian_report(
                &decision,
                &malformed,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
                &mut malformed_pmp_dynamics,
                &mut malformed_pmp_lagrange
            )
            .is_err()
        );
    }

    #[test]
    fn fixed_mesh_ocp_sparse_constrained_kkt_step_reports_active_hard_path_multipliers() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_hard_path_constraints(vec![
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    None,
                    Some(0.45),
                )
                .expect("hard path upper bound"),
            ])
            .expect("hard path constraints validate");
        let decision = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.45], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let mut dynamics = integrator_control_dynamics;
        let regularization = 2.5e-1;
        let report = ocp
            .sparse_constrained_kkt_step_report(
                &decision,
                &mut dynamics,
                1.0e-6,
                1.0e-9,
                regularization,
                1.0e-12,
                1.0e-10,
            )
            .expect("sparse constrained KKT step assembles");

        let dense_jacobian = report
            .sparse_jacobian
            .to_dense_row_major()
            .expect("dense residual Jacobian");
        let dense_kkt = dense_constrained_augmented_kkt(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            &report.hard_path_inequality_report,
            &report.constrained_kkt.active_inequality_rows,
            regularization,
        );
        let sparse_dense_kkt = report
            .constrained_kkt
            .to_dense_row_major()
            .expect("dense constrained KKT");
        assert_eq!(report.constrained_kkt.active_inequality_row_count, 1);
        assert_eq!(report.constrained_kkt.active_inequality_rows, vec![0]);
        assert!(report.constrained_kkt.active_inequality_values[0].abs() < 1.0e-15);
        assert_eq!(
            report.active_inequality_multipliers.len(),
            report.constrained_kkt.active_inequality_row_count
        );
        for (index, (actual, expected)) in sparse_dense_kkt.iter().zip(dense_kkt.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-10,
                "constrained KKT entry {index} expected {expected}, got {actual}"
            );
        }

        let expected_solution = crate::corrector::solve_linear_system(
            dense_kkt,
            report.rhs.clone(),
            report.constrained_kkt.column_count,
        )
        .expect("dense constrained KKT solves");
        assert!(report.solve_report.residual_norm < 1.0e-9);
        assert!(report.solve_report.factorization_residual_norm < 1.0e-9);
        for rhs in &report.rhs[..report.sparse_jacobian.column_count] {
            assert!(rhs.abs() < 1.0e-15);
        }
        for (index, residual) in report.residual_report.residuals.iter().enumerate() {
            assert!(
                (report.rhs[report.sparse_jacobian.column_count + index] + residual).abs()
                    < 1.0e-15,
                "constrained KKT RHS residual block mismatch at {index}"
            );
        }
        let active_rhs_offset =
            report.sparse_jacobian.column_count + report.sparse_jacobian.row_count;
        assert!(
            (report.rhs[active_rhs_offset] + report.constrained_kkt.active_inequality_values[0])
                .abs()
                < 1.0e-15
        );
        for (index, (actual, expected)) in report
            .solve_report
            .solution
            .iter()
            .zip(expected_solution.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "constrained KKT solution index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .residual_multipliers
            .iter()
            .zip(report.linearized_residual.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "constrained KKT residual multiplier index {index} expected {expected}, got {actual}"
            );
        }
        assert!(
            report.max_linearized_active_inequality_violation < 1.0e-9,
            "active hard-path row did not close: {:?}",
            report.linearized_active_inequality_values
        );

        let constrained_multiplier_report = ocp
            .sparse_constrained_kkt_residual_multiplier_report(&report)
            .expect("constrained sparse KKT residual multipliers slice");
        assert_eq!(
            constrained_multiplier_report.residual_multiplier_count,
            report.residual_multipliers.len()
        );
        assert_eq!(constrained_multiplier_report.collocation_samples.len(), 1);
        let constrained_costate_report = ocp
            .sparse_constrained_kkt_node_costate_estimate_report(&report)
            .expect("constrained sparse KKT costates evaluate");
        assert_eq!(constrained_costate_report.node_samples.len(), 2);
        let pmp_convention = crate::pmp::PmpDirectCostateConvention {
            raw_to_pmp_scale: 1.0,
        };
        let constrained_pmp_costate_report = ocp
            .sparse_constrained_kkt_pmp_costate_convention_report(&report, pmp_convention)
            .expect("constrained sparse KKT PMP costates evaluate");
        assert_eq!(constrained_pmp_costate_report.sample_count, 2);

        let mut constrained_hamiltonian_dynamics = integrator_control_dynamics;
        let mut constrained_hamiltonian_lagrange = zero_lagrange;
        let constrained_hamiltonian_report = ocp
            .sparse_constrained_kkt_hamiltonian_report(
                &decision,
                &report,
                &mut constrained_hamiltonian_dynamics,
                &mut constrained_hamiltonian_lagrange,
            )
            .expect("constrained sparse KKT Hamiltonian diagnostics evaluate");
        assert_eq!(constrained_hamiltonian_report.sample_count, 2);
        let mut constrained_pmp_hamiltonian_dynamics = integrator_control_dynamics;
        let mut constrained_pmp_hamiltonian_lagrange = zero_lagrange;
        let constrained_pmp_hamiltonian_report = ocp
            .sparse_constrained_kkt_pmp_hamiltonian_report(
                &decision,
                &report,
                pmp_convention,
                &mut constrained_pmp_hamiltonian_dynamics,
                &mut constrained_pmp_hamiltonian_lagrange,
            )
            .expect("constrained sparse KKT PMP Hamiltonian evaluates");
        assert_eq!(constrained_pmp_hamiltonian_report.sample_count, 2);

        let mut malformed_report = report.clone();
        malformed_report.residual_multipliers.pop();
        assert!(
            ocp.sparse_constrained_kkt_residual_multiplier_report(&malformed_report)
                .is_err()
        );
        assert!(
            ocp.sparse_constrained_kkt_node_costate_estimate_report(&malformed_report)
                .is_err()
        );
        assert!(
            ocp.sparse_constrained_kkt_pmp_costate_convention_report(
                &malformed_report,
                pmp_convention,
            )
            .is_err()
        );
        let mut malformed_hamiltonian_dynamics = integrator_control_dynamics;
        let mut malformed_hamiltonian_lagrange = zero_lagrange;
        assert!(
            ocp.sparse_constrained_kkt_hamiltonian_report(
                &decision,
                &malformed_report,
                &mut malformed_hamiltonian_dynamics,
                &mut malformed_hamiltonian_lagrange,
            )
            .is_err()
        );

        let inactive = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.3], &[1.0, 1.0], &[1.0])
            .expect("inactive decision packs");
        let mut inactive_dynamics = integrator_control_dynamics;
        assert!(
            ocp.sparse_constrained_kkt_step_report(
                &inactive,
                &mut inactive_dynamics,
                1.0e-6,
                1.0e-9,
                regularization,
                1.0e-12,
                1.0e-10,
            )
            .is_err()
        );
        assert!(report.constrained_kkt.multiply_vector(&[0.0]).is_err());
        assert!(report.constrained_kkt.solve_ldlt(&[0.0]).is_err());
    }

    #[test]
    fn fixed_mesh_ocp_sparse_kkt_sqp_solves_scalar_integrator_and_reports_final_multipliers() {
        let ocp = unit_interval_integrator_ocp(2).expect("OCP builds");
        let initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 0.4, 1.0], &[0.1, 0.6], &[0.8, 1.1, 0.9], &[1.2, 0.7])
            .expect("initial decision packs");
        let mut dynamics = integrator_control_dynamics;
        let settings = SparseKktSqpSettings {
            max_iterations: 12,
            residual_tolerance: 1.0e-8,
            step_tolerance: 1.0e-12,
            kkt_regularization: 1.0e-3,
            ..SparseKktSqpSettings::default()
        };

        let report = ocp
            .solve_sparse_kkt_sqp(&initial, &mut dynamics, settings)
            .expect("sparse KKT SQP solves scalar integrator");

        assert!(report.converged);
        assert!(!report.iterations.is_empty());
        assert!(report.residual_report.residual_norm <= settings.residual_tolerance);
        assert_eq!(report.decision_bounds.applied_bound_count, 0);
        assert!(report.max_decision_bound_violation <= settings.residual_tolerance);
        assert!(report.max_hard_path_violation <= settings.residual_tolerance);
        assert_eq!(
            report.final_kkt_step_report.residual_report,
            report.residual_report
        );
        assert_eq!(
            report.final_kkt_step_report.residual_multipliers.len(),
            report.residual_report.residuals.len()
        );
        assert!(report.final_constrained_kkt_step_report.is_none());
        let multiplier_report = ocp
            .kkt_residual_multiplier_report(&report.final_kkt_step_report)
            .expect("final sparse-SQP multipliers slice");
        assert_eq!(
            multiplier_report.residual_multiplier_count,
            report.residual_report.residuals.len()
        );
        assert!(
            report
                .iterations
                .windows(2)
                .all(|window| window[1].accepted_residual_norm <= window[0].accepted_residual_norm)
        );
    }

    #[test]
    fn fixed_mesh_ocp_sparse_kkt_sqp_enforces_hard_path_decision_bounds() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_hard_path_constraints(vec![
                HermiteSimpsonHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                    Some(0.5),
                    Some(0.5),
                )
                .expect("hard path bound"),
            ])
            .expect("hard path constraints validate");
        let initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.5], &[1.0, 1.0], &[1.0])
            .expect("initial decision packs");
        let settings = SparseKktSqpSettings {
            max_iterations: 12,
            residual_tolerance: 1.0e-8,
            step_tolerance: 1.0e-12,
            kkt_regularization: 1.0e-3,
            ..SparseKktSqpSettings::default()
        };
        let mut dynamics = integrator_control_dynamics;

        let report = ocp
            .solve_sparse_kkt_sqp(&initial, &mut dynamics, settings)
            .expect("bounded sparse KKT SQP solves");

        assert!(report.converged);
        assert_eq!(report.decision_bounds.applied_bound_count, 2);
        assert_eq!(report.decision_bounds.constrained_variable_count, 1);
        assert!(report.max_decision_bound_violation <= settings.residual_tolerance);
        assert!(report.max_hard_path_violation <= settings.residual_tolerance);
        let constrained_report = report
            .final_constrained_kkt_step_report
            .as_ref()
            .expect("final constrained sparse-KKT step report");
        assert_eq!(constrained_report.residual_report, report.residual_report);
        assert_eq!(
            constrained_report
                .constrained_kkt
                .active_inequality_row_count,
            2
        );
        assert_eq!(constrained_report.active_inequality_multipliers.len(), 2);
        assert!(
            constrained_report.max_linearized_active_inequality_violation
                <= settings.residual_tolerance
        );
        let constrained_multiplier_report = ocp
            .sparse_constrained_kkt_residual_multiplier_report(constrained_report)
            .expect("final constrained multipliers slice");
        assert_eq!(
            constrained_multiplier_report.residual_multiplier_count,
            report.residual_report.residuals.len()
        );

        let outside_initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.7], &[1.0, 1.0], &[1.0])
            .expect("outside decision packs");
        let mut dynamics = integrator_control_dynamics;
        assert!(
            ocp.solve_sparse_kkt_sqp(&outside_initial, &mut dynamics, settings)
                .is_err()
        );
    }

    #[test]
    fn fixed_mesh_ocp_sparse_kkt_sqp_preserves_nonlinear_hard_path_feasibility() {
        let ocp = unit_interval_integrator_ocp(1)
            .expect("OCP builds")
            .with_nonlinear_hard_path_constraints(vec![
                HermiteSimpsonNonlinearHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    vec![Some(0.4)],
                    vec![Some(0.6)],
                )
                .expect("nonlinear hard path bound"),
            ])
            .expect("nonlinear hard path constraints validate");
        let initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.45], &[1.0, 1.0], &[1.0])
            .expect("initial decision packs");
        let settings = SparseKktSqpSettings {
            max_iterations: 12,
            residual_tolerance: 1.0e-8,
            step_tolerance: 1.0e-12,
            kkt_regularization: 1.0e-3,
            ..SparseKktSqpSettings::default()
        };
        let lower = vec![-10.0; ocp.layout.variable_count];
        let upper = vec![10.0; ocp.layout.variable_count];
        let mut dynamics = integrator_control_dynamics;
        let mut path_function = |_time_s: f64, state: &[f64], _control: &[f64]| {
            Ok::<Vec<f64>, TrajoptError>(vec![state[0]])
        };

        let report = ocp
            .solve_sparse_kkt_sqp_with_nonlinear_hard_path_constraints(
                &initial,
                &lower,
                &upper,
                &mut dynamics,
                &mut path_function,
                settings,
            )
            .expect("nonlinear-feasible sparse KKT SQP solves");

        assert!(report.converged);
        assert!(report.max_decision_bound_violation <= settings.residual_tolerance);
        assert!(report.max_hard_path_violation <= settings.residual_tolerance);
        assert!(report.final_constrained_kkt_step_report.is_none());
        assert!(report.max_nonlinear_hard_path_violation <= settings.residual_tolerance);
        assert!(
            report
                .nonlinear_hard_path_report
                .as_ref()
                .expect("nonlinear report")
                .max_violation
                <= settings.residual_tolerance
        );

        let mut dynamics = integrator_control_dynamics;
        assert!(
            ocp.solve_sparse_kkt_sqp(&initial, &mut dynamics, settings)
                .is_err()
        );

        let outside_initial = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.7], &[1.0, 1.0], &[1.0])
            .expect("outside decision packs");
        let mut dynamics = integrator_control_dynamics;
        let mut path_function = |_time_s: f64, state: &[f64], _control: &[f64]| {
            Ok::<Vec<f64>, TrajoptError>(vec![state[0]])
        };
        assert!(
            ocp.solve_sparse_kkt_sqp_with_nonlinear_hard_path_constraints(
                &outside_initial,
                &lower,
                &upper,
                &mut dynamics,
                &mut path_function,
                settings,
            )
            .is_err()
        );
    }

    #[test]
    fn hermite_simpson_pmp_costate_convention_bridge_matches_scalar_pmp_arc()
    -> Result<(), TrajoptError> {
        let problem = crate::pmp::PmpScalarMinEnergyProblem {
            initial_time_s: 0.0,
            final_time_s: 4.0,
            initial_state: 0.0,
            terminal_state: 2.0,
            control_weight: 2.0,
        };
        let pmp_solution =
            crate::pmp::solve_pmp_scalar_min_energy_shooting(problem, Default::default())?;
        let times_s = pmp_solution
            .hamiltonian_report
            .samples
            .iter()
            .map(|sample| sample.time_s)
            .collect::<Vec<_>>();
        let pmp_costates = pmp_solution
            .hamiltonian_report
            .samples
            .iter()
            .flat_map(|sample| sample.costate.iter().copied())
            .collect::<Vec<_>>();
        let raw_to_pmp_scale = -0.5;
        let raw_costate = pmp_solution.initial_costate / raw_to_pmp_scale;
        let last_index = times_s.len() - 1;
        let node_samples = times_s
            .iter()
            .enumerate()
            .map(
                |(node_index, time_s)| HermiteSimpsonKktNodeCostateEstimate {
                    node_index,
                    time_s: *time_s,
                    costate: vec![raw_costate],
                    contributing_segment_count: if node_index == 0 || node_index == last_index {
                        1
                    } else {
                        2
                    },
                },
            )
            .collect::<Vec<_>>();
        let hs_costate_report = HermiteSimpsonKktCostateEstimateReport {
            segment_count: last_index,
            state_dimension: 1,
            source_residual_multiplier_count: last_index,
            node_samples,
            max_abs_costate: raw_costate.abs(),
        };

        let convention_report = hs_costate_report.pmp_costate_convention_report(
            crate::pmp::PmpDirectCostateConvention { raw_to_pmp_scale },
        )?;
        let comparison = crate::pmp::compare_pmp_costate_profiles(
            1,
            &times_s,
            &pmp_costates,
            &convention_report.times_s,
            &convention_report.pmp_costates,
            crate::pmp::PmpCostateComparisonSettings {
                costate_tolerance: 1.0e-12,
                ..Default::default()
            },
        )?;

        assert_eq!(convention_report.sample_count, times_s.len());
        assert_eq!(convention_report.raw_to_pmp_scale, raw_to_pmp_scale);
        assert!(comparison.within_tolerance);
        assert_eq!(comparison.max_costate_norm_difference, 0.0);
        Ok(())
    }

    #[test]
    fn fixed_mesh_ocp_autodiff_sparse_gauss_newton_step_matches_finite_difference_step() {
        let ocp = nonlinear_autodiff_ocp().expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(
                &[0.2, -0.1, 0.4, 0.3, -0.2, 0.1],
                &[0.31, 0.05, 0.08, 0.2],
                &[0.7, -0.4, 0.2],
                &[0.1, -0.3],
            )
            .expect("decision packs");
        let regularization = 1.0;
        let mut finite_difference_dynamics =
            CollocationAutoDiffDynamicsAdapter::new(NonlinearAutoDiffDynamics, 2, 1)
                .expect("AD adapter builds");
        let finite_difference = ocp
            .sparse_gauss_newton_step_report(
                &decision,
                &mut finite_difference_dynamics,
                1.0e-6,
                1.0e-10,
                regularization,
                1.0e-12,
                ocp.layout.variable_count * 4,
                1.0e-12,
            )
            .expect("finite-difference sparse step assembles");
        let mut autodiff_dynamics = NonlinearAutoDiffDynamics;
        let autodiff = ocp
            .autodiff_sparse_gauss_newton_step_report(
                &decision,
                &mut autodiff_dynamics,
                1.0e-12,
                regularization,
                1.0e-12,
                ocp.layout.variable_count * 4,
                1.0e-12,
            )
            .expect("autodiff sparse step assembles");

        assert_eq!(autodiff.sparse_jacobian.finite_difference_step, 0.0);
        assert!(autodiff.solve_report.converged);
        assert!(autodiff.linearized_residual_norm < autodiff.residual_report.residual_norm);
        for (index, (actual, expected)) in autodiff
            .solve_report
            .solution
            .iter()
            .zip(finite_difference.solve_report.solution.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 5.0e-6,
                "autodiff step index {index} expected {expected}, got {actual}"
            );
        }
        assert!(
            (autodiff.linearized_residual_norm - finite_difference.linearized_residual_norm).abs()
                < 5.0e-6
        );
    }

    #[test]
    fn fixed_mesh_ocp_sparse_jacobian_rejects_bad_settings_and_indices() {
        let ocp = unit_interval_integrator_ocp(1).expect("OCP builds");
        let decision = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.5], &[1.0, 1.0], &[1.0])
            .expect("decision packs");
        let mut dynamics = integrator_control_dynamics;

        assert!(
            ocp.sparse_residual_jacobian_report(&decision, &mut dynamics, 0.0, 1.0e-9)
                .is_err()
        );
        assert!(
            ocp.sparse_residual_jacobian_report(&decision, &mut dynamics, 1.0e-6, -1.0)
                .is_err()
        );
        assert!(
            ocp.sparse_residual_jacobian_report(&decision[..5], &mut dynamics, 1.0e-6, 1.0e-9)
                .is_err()
        );
        assert!(
            ocp.colored_sparse_residual_jacobian_report(&decision, &mut dynamics, 0.0, 1.0e-9)
                .is_err()
        );
        assert!(
            ocp.colored_sparse_residual_jacobian_report(
                &decision[..5],
                &mut dynamics,
                1.0e-6,
                1.0e-9,
            )
            .is_err()
        );

        let jacobian = ocp
            .sparse_residual_jacobian_report(&decision, &mut dynamics, 1.0e-6, 1.0e-9)
            .expect("Jacobian evaluates");
        assert!(jacobian.coefficient(4, 0).is_err());
        assert!(jacobian.coefficient(0, 6).is_err());
        assert!(
            jacobian
                .sparse_gauss_newton_hessian_report(-1.0, 1.0e-9)
                .is_err()
        );
        assert!(
            jacobian
                .sparse_gauss_newton_hessian_report(0.0, -1.0)
                .is_err()
        );
        assert!(jacobian.sparse_augmented_kkt_report(0.0, 1.0e-12).is_err());
        assert!(jacobian.sparse_augmented_kkt_report(1.0e-3, -1.0).is_err());
        let hessian = jacobian
            .sparse_gauss_newton_hessian_report(0.0, 1.0e-12)
            .expect("sparse Hessian assembles");
        let kkt = jacobian
            .sparse_augmented_kkt_report(1.0e-3, 1.0e-12)
            .expect("sparse augmented KKT assembles");
        assert!(hessian.coefficient(6, 0).is_err());
        assert!(hessian.coefficient(0, 6).is_err());
        assert!(kkt.coefficient(kkt.row_count, 0).is_err());
        assert!(kkt.coefficient(0, kkt.column_count).is_err());
        assert!(
            ocp.sparse_gauss_newton_step_report(
                &decision,
                &mut dynamics,
                1.0e-6,
                1.0e-9,
                1.0e-3,
                1.0e-12,
                0,
                1.0e-9,
            )
            .is_err()
        );
        assert!(
            ocp.sparse_augmented_kkt_step_report(
                &decision,
                &mut dynamics,
                1.0e-6,
                1.0e-9,
                0.0,
                1.0e-12,
            )
            .is_err()
        );
        assert!(jacobian.sparse_gauss_newton_step_rhs(&[0.0]).is_err());
        assert!(hessian.multiply_vector(&[0.0]).is_err());
        assert!(kkt.multiply_vector(&[0.0]).is_err());
        assert!(kkt.solve_ldlt(&[0.0]).is_err());
        assert!(
            hessian
                .solve_conjugate_gradient(&vec![0.0; 6], &vec![0.0; 6], 0, 1.0e-9)
                .is_err()
        );
        assert!(
            hessian
                .solve_conjugate_gradient(&vec![0.0; 6], &vec![0.0; 6], 4, -1.0)
                .is_err()
        );

        let malformed = HermiteSimpsonSparseResidualJacobianReport {
            row_count: 4,
            column_count: 6,
            column_offsets: vec![0, 1],
            row_indices: vec![0],
            values: vec![1.0],
            finite_difference_step: 1.0e-6,
            drop_tolerance: 1.0e-9,
        };
        assert!(malformed.to_dense_row_major().is_err());
        assert!(
            malformed
                .sparse_gauss_newton_hessian_report(0.0, 1.0e-9)
                .is_err()
        );
        assert!(
            malformed
                .sparse_augmented_kkt_report(1.0e-3, 1.0e-9)
                .is_err()
        );

        let malformed_hessian = HermiteSimpsonSparseGaussNewtonHessianReport {
            row_count: 6,
            column_count: 6,
            residual_row_count: 4,
            source_jacobian_nonzero_count: 1,
            column_offsets: vec![0, 1],
            row_indices: vec![0],
            values: vec![1.0],
            regularization: 0.0,
            drop_tolerance: 1.0e-9,
        };
        assert!(malformed_hessian.to_dense_row_major().is_err());

        let malformed_kkt = HermiteSimpsonSparseAugmentedKktReport {
            row_count: 10,
            column_count: 10,
            variable_count: 6,
            residual_row_count: 4,
            source_jacobian_nonzero_count: 1,
            column_offsets: vec![0, 1],
            row_indices: vec![0],
            values: vec![1.0],
            regularization: 1.0e-3,
            drop_tolerance: 1.0e-9,
        };
        assert!(malformed_kkt.to_dense_row_major().is_err());

        let non_positive_curvature = HermiteSimpsonSparseGaussNewtonHessianReport {
            row_count: 1,
            column_count: 1,
            residual_row_count: 1,
            source_jacobian_nonzero_count: 1,
            column_offsets: vec![0, 1],
            row_indices: vec![0],
            values: vec![-1.0],
            regularization: 0.0,
            drop_tolerance: 0.0,
        };
        assert!(
            non_positive_curvature
                .solve_conjugate_gradient(&[1.0], &[0.0], 4, 1.0e-9)
                .is_err()
        );

        let malformed_colored = HermiteSimpsonColoredSparseResidualJacobianReport {
            sparse_jacobian: jacobian,
            column_colors: vec![0],
            color_count: 1,
            residual_evaluation_count: 3,
            structural_nonzero_count: 12,
        };
        assert!(malformed_colored.to_dense_row_major().is_err());
    }

    #[test]
    fn fixed_mesh_ocp_rejects_malformed_nonlinear_hard_path_terms() {
        let ocp = unit_interval_integrator_ocp(1).expect("OCP builds");

        assert!(
            HermiteSimpsonNonlinearHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Node(0),
                Vec::new(),
                Vec::new(),
            )
            .is_err()
        );
        assert!(
            HermiteSimpsonNonlinearHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Node(0),
                vec![Some(0.0)],
                vec![Some(1.0), None],
            )
            .is_err()
        );
        assert!(
            HermiteSimpsonNonlinearHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Node(0),
                vec![None],
                vec![None],
            )
            .is_err()
        );
        assert!(
            HermiteSimpsonNonlinearHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Node(0),
                vec![Some(2.0)],
                vec![Some(1.0)],
            )
            .is_err()
        );

        let bad_sample = ocp.clone().with_nonlinear_hard_path_constraints(vec![
            HermiteSimpsonNonlinearHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Midpoint(1),
                vec![Some(0.0)],
                vec![Some(1.0)],
            )
            .expect("nonlinear hard path"),
        ]);
        assert!(bad_sample.is_err());

        let ocp = ocp
            .with_nonlinear_hard_path_constraints(vec![
                HermiteSimpsonNonlinearHardPathConstraint::new(
                    HermiteSimpsonSamplePoint::Midpoint(0),
                    vec![Some(0.0), None],
                    vec![Some(1.0), Some(1.0)],
                )
                .expect("nonlinear hard path"),
            ])
            .expect("nonlinear hard path constraints validate");
        let decision = ocp
            .layout
            .pack_decision_vector(&[0.0, 1.0], &[0.5], &[1.0, 1.0], &[1.0])
            .expect("decision packs");

        let mut wrong_dimension = |_time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![0.0]);
        assert!(
            ocp.nonlinear_hard_path_inequality_report(&decision, &mut wrong_dimension)
                .is_err()
        );
        let mut nonfinite =
            |_time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![0.0, f64::NAN]);
        assert!(
            ocp.nonlinear_hard_path_inequality_report(&decision, &mut nonfinite)
                .is_err()
        );

        let mut path_function = quadratic_path_function;
        assert!(
            ocp.sparse_nonlinear_hard_path_inequality_jacobian_report(
                &decision,
                &mut path_function,
                0.0,
                1.0e-10,
            )
            .is_err()
        );
        let mut path_function = quadratic_path_function;
        assert!(
            ocp.sparse_nonlinear_hard_path_inequality_jacobian_report(
                &decision[..5],
                &mut path_function,
                1.0e-6,
                1.0e-10,
            )
            .is_err()
        );

        let malformed_report = HermiteSimpsonNonlinearPathInequalityReport {
            variable_count: 6,
            path_value_count: 1,
            inequality_count: 1,
            path_values: vec![0.0],
            inequality_values: vec![0.0],
            max_violation: 1.0,
        };
        assert!(malformed_report.violations().is_err());

        let malformed_jacobian = HermiteSimpsonSparsePathInequalityJacobianReport {
            row_count: 1,
            column_count: 6,
            column_offsets: vec![0, 1],
            row_indices: vec![1],
            values: vec![1.0],
            finite_difference_step: 1.0e-6,
            drop_tolerance: 1.0e-10,
        };
        assert!(malformed_jacobian.to_dense_row_major().is_err());
    }

    #[test]
    fn fixed_mesh_ocp_rejects_malformed_path_and_objective_terms() {
        let ocp = unit_interval_integrator_ocp(1).expect("OCP builds");
        let bad_component = ocp.clone().with_path_constraints(vec![
            HermiteSimpsonSoftPathConstraint::new(
                HermiteSimpsonSamplePoint::Node(0),
                HermiteSimpsonScalarVariable::StateComponent { component: 4 },
                Some(0.0),
                None,
                1.0,
            )
            .expect("path constraint"),
        ]);
        assert!(bad_component.is_err());

        let bad_sample = ocp.clone().with_path_constraints(vec![
            HermiteSimpsonSoftPathConstraint::new(
                HermiteSimpsonSamplePoint::Midpoint(1),
                HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                Some(0.0),
                None,
                1.0,
            )
            .expect("path constraint"),
        ]);
        assert!(bad_sample.is_err());

        let bad_hard_component = ocp.clone().with_hard_path_constraints(vec![
            HermiteSimpsonHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Node(0),
                HermiteSimpsonScalarVariable::StateComponent { component: 4 },
                Some(0.0),
                None,
            )
            .expect("hard path constraint"),
        ]);
        assert!(bad_hard_component.is_err());

        let bad_hard_sample = ocp.clone().with_hard_path_constraints(vec![
            HermiteSimpsonHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Midpoint(1),
                HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                Some(0.0),
                None,
            )
            .expect("hard path constraint"),
        ]);
        assert!(bad_hard_sample.is_err());

        let no_control =
            HermiteSimpsonFixedMeshOcp::new(vec![0.0, 1.0], 1, 0, Vec::new()).expect("OCP builds");
        let bad_cost = no_control.with_quadratic_control_costs(vec![
            HermiteSimpsonQuadraticControlCost::new(0, 1.0).expect("control cost"),
        ]);
        assert!(bad_cost.is_err());

        assert!(
            HermiteSimpsonSoftPathConstraint::new(
                HermiteSimpsonSamplePoint::Node(0),
                HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                None,
                None,
                1.0,
            )
            .is_err()
        );
        assert!(
            HermiteSimpsonHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Node(0),
                HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                None,
                None,
            )
            .is_err()
        );
        assert!(
            HermiteSimpsonHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Node(0),
                HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                Some(2.0),
                Some(1.0),
            )
            .is_err()
        );
        assert!(HermiteSimpsonQuadraticControlCost::new(0, 0.0).is_err());
    }

    fn cubic_segment(t0: f64, t1: f64) -> HermiteSimpsonSegment {
        let tm = 0.5 * (t0 + t1);
        HermiteSimpsonSegment::new(
            t0,
            tm,
            t1,
            &[t0 * t0 * t0],
            &[tm * tm * tm],
            &[t1 * t1 * t1],
            &[],
            &[],
            &[],
        )
        .expect("cubic segment")
    }

    fn sine_segments(segment_count: usize) -> Vec<HermiteSimpsonSegment> {
        let h = 1.0 / (segment_count as f64);
        (0..segment_count)
            .map(|index| {
                let t0 = (index as f64) * h;
                let t1 = ((index + 1) as f64) * h;
                let tm = 0.5 * (t0 + t1);
                HermiteSimpsonSegment::new(
                    t0,
                    tm,
                    t1,
                    &[t0.sin()],
                    &[tm.sin()],
                    &[t1.sin()],
                    &[],
                    &[],
                    &[],
                )
                .expect("sine segment")
            })
            .collect()
    }

    fn cubic_dynamics(
        time_s: f64,
        _state: &[f64],
        _control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![3.0 * time_s * time_s])
    }

    fn sine_dynamics(
        time_s: f64,
        _state: &[f64],
        _control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![time_s.cos()])
    }

    fn zero_lagrange(_time_s: f64, _state: &[f64], _control: &[f64]) -> Result<f64, TrajoptError> {
        Ok(0.0)
    }

    fn time_squared_lagrange(
        time_s: f64,
        _state: &[f64],
        _control: &[f64],
    ) -> Result<f64, TrajoptError> {
        Ok(time_s * time_s)
    }

    fn unit_lagrange(_time_s: f64, _state: &[f64], _control: &[f64]) -> Result<f64, TrajoptError> {
        Ok(1.0)
    }

    fn unit_interval_integrator_ocp(
        segment_count: usize,
    ) -> Result<HermiteSimpsonFixedMeshOcp, TrajoptError> {
        let mut time_nodes_s = Vec::with_capacity(segment_count + 1);
        for index in 0..=segment_count {
            time_nodes_s.push((index as f64) / (segment_count as f64));
        }
        HermiteSimpsonFixedMeshOcp::new(
            time_nodes_s,
            1,
            1,
            vec![
                HermiteSimpsonBoundaryConstraint::new(0, 0, 0.0)?,
                HermiteSimpsonBoundaryConstraint::new(segment_count, 0, 1.0)?,
            ],
        )
    }

    fn nonlinear_autodiff_ocp() -> Result<HermiteSimpsonFixedMeshOcp, TrajoptError> {
        HermiteSimpsonFixedMeshOcp::new(
            vec![0.0, 0.4, 1.0],
            2,
            1,
            vec![
                HermiteSimpsonBoundaryConstraint::new(0, 0, 0.2)?,
                HermiteSimpsonBoundaryConstraint::new(2, 1, 0.1)?,
            ],
        )
    }

    fn integrator_control_dynamics(
        _time_s: f64,
        _state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![control[0]])
    }

    #[cfg(feature = "sqp")]
    fn three_state_reference_dynamics(
        _time_s: f64,
        _state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![control[0], 0.0, 0.0])
    }

    struct NonlinearAutoDiffDynamics;

    impl CollocationAutoDiffDynamics for NonlinearAutoDiffDynamics {
        fn eval_dual(
            &mut self,
            time_s: f64,
            state: &[CollocationAutoDiffScalar],
            control: &[CollocationAutoDiffScalar],
        ) -> Result<Vec<CollocationAutoDiffScalar>, TrajoptError> {
            if state.len() != 2 || control.len() != 1 {
                return Err(TrajoptError::InvalidPayload {
                    reason: "nonlinear autodiff dynamics dimensions are inconsistent",
                });
            }
            Ok(vec![
                state[1].clone() + control[0].sin(),
                state[0].clone() * control[0].clone()
                    + state[1].powi(2)
                    + CollocationAutoDiffScalar::constant(time_s).cos(),
            ])
        }
    }

    struct ShortAutoDiffDynamicsOutput;

    impl CollocationAutoDiffDynamics for ShortAutoDiffDynamicsOutput {
        fn eval_dual(
            &mut self,
            _time_s: f64,
            _state: &[CollocationAutoDiffScalar],
            control: &[CollocationAutoDiffScalar],
        ) -> Result<Vec<CollocationAutoDiffScalar>, TrajoptError> {
            Ok(vec![control[0].clone()])
        }
    }

    struct BadAutoDiffDerivativeDimension;

    impl CollocationAutoDiffDynamics for BadAutoDiffDerivativeDimension {
        fn eval_dual(
            &mut self,
            _time_s: f64,
            _state: &[CollocationAutoDiffScalar],
            _control: &[CollocationAutoDiffScalar],
        ) -> Result<Vec<CollocationAutoDiffScalar>, TrajoptError> {
            Ok(vec![
                CollocationAutoDiffScalar::new(1.0, vec![1.0])?,
                CollocationAutoDiffScalar::new(2.0, Vec::new())?,
            ])
        }
    }

    struct IntegratorControlDynamicsJacobian;

    impl CollocationDynamicsJacobian for IntegratorControlDynamicsJacobian {
        fn eval_with_jacobian(
            &mut self,
            _time_s: f64,
            state: &[f64],
            control: &[f64],
        ) -> Result<CollocationDynamicsJacobianSample, TrajoptError> {
            CollocationDynamicsJacobianSample::new(
                vec![control[0]],
                vec![0.0],
                vec![1.0],
                state.len(),
                control.len(),
            )
        }
    }

    struct MalformedIntegratorControlDynamicsJacobian;

    impl CollocationDynamicsJacobian for MalformedIntegratorControlDynamicsJacobian {
        fn eval_with_jacobian(
            &mut self,
            _time_s: f64,
            _state: &[f64],
            control: &[f64],
        ) -> Result<CollocationDynamicsJacobianSample, TrajoptError> {
            Ok(CollocationDynamicsJacobianSample {
                derivative: vec![control[0]],
                state_jacobian: vec![0.0],
                control_jacobian: Vec::new(),
            })
        }
    }

    #[cfg(feature = "sqp")]
    fn bryson_denham_ocp(
        problem: BrysonDenhamBenchmarkProblem,
    ) -> Result<HermiteSimpsonFixedMeshOcp, TrajoptError> {
        let mut hard_path_constraints = Vec::with_capacity(2 * problem.segment_count + 1);
        for node_index in 0..=problem.segment_count {
            hard_path_constraints.push(HermiteSimpsonHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Node(node_index),
                HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                None,
                Some(problem.state_bound),
            )?);
        }
        for segment_index in 0..problem.segment_count {
            hard_path_constraints.push(HermiteSimpsonHardPathConstraint::new(
                HermiteSimpsonSamplePoint::Midpoint(segment_index),
                HermiteSimpsonScalarVariable::StateComponent { component: 0 },
                None,
                Some(problem.state_bound),
            )?);
        }
        HermiteSimpsonFixedMeshOcp::new(
            uniform_time_nodes(problem.segment_count, 1.0),
            2,
            1,
            vec![
                HermiteSimpsonBoundaryConstraint::new(0, 0, 0.0)?,
                HermiteSimpsonBoundaryConstraint::new(0, 1, 1.0)?,
                HermiteSimpsonBoundaryConstraint::new(problem.segment_count, 0, 0.0)?,
                HermiteSimpsonBoundaryConstraint::new(problem.segment_count, 1, -1.0)?,
            ],
        )?
        .with_hard_path_constraints(hard_path_constraints)
    }

    #[cfg(feature = "sqp")]
    fn bryson_denham_quadratic_ocp(
        problem: BrysonDenhamBenchmarkProblem,
    ) -> Result<HermiteSimpsonFixedMeshOcp, TrajoptError> {
        bryson_denham_ocp(problem)?
            .with_quadratic_control_costs(vec![HermiteSimpsonQuadraticControlCost::new(0, 1.0)?])
    }

    #[cfg(feature = "sqp")]
    fn bryson_denham_dynamics(
        _time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        if state.len() != 2 || control.len() != 1 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Bryson-Denham dynamics dimensions are inconsistent",
            });
        }
        Ok(vec![state[1], control[0]])
    }

    #[cfg(feature = "sqp")]
    fn bryson_denham_benchmark_qp(
        ocp: &HermiteSimpsonFixedMeshOcp,
    ) -> Result<
        (
            crate::sqp::DenseQpProblem,
            HermiteSimpsonPathInequalityReport,
        ),
        TrajoptError,
    > {
        let zero_decision = vec![0.0; ocp.layout.variable_count];
        let mut base_dynamics = bryson_denham_dynamics;
        let residual_at_zero = ocp.residuals(&zero_decision, &mut base_dynamics)?;
        let mut jacobian_dynamics = bryson_denham_dynamics;
        let equality_matrix = ocp
            .sparse_residual_jacobian_report(
                &zero_decision,
                &mut jacobian_dynamics,
                1.0e-6,
                1.0e-12,
            )?
            .to_dense_row_major()?;
        let equality_rhs = residual_at_zero
            .iter()
            .map(|residual| -residual)
            .collect::<Vec<_>>();
        let inequalities = ocp.hard_path_inequality_report()?;
        let problem = crate::sqp::DenseQpProblem::new(
            ocp.layout.variable_count,
            bryson_denham_control_hessian(ocp),
            vec![0.0; ocp.layout.variable_count],
            equality_matrix,
            equality_rhs,
            inequalities.inequality_matrix.clone(),
            inequalities.inequality_rhs.clone(),
        )?;
        Ok((problem, inequalities))
    }

    fn quadratic_path_function(
        _time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![
            state[0] * state[0] + control[0],
            state[0] * control[0],
        ])
    }

    #[cfg(feature = "sqp")]
    fn control_square_path_function(
        _time_s: f64,
        _state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![control[0] * control[0]])
    }

    fn uniform_time_nodes(segment_count: usize, final_time_s: f64) -> Vec<f64> {
        let mut time_nodes_s = Vec::with_capacity(segment_count + 1);
        for index in 0..=segment_count {
            time_nodes_s.push(final_time_s * (index as f64) / (segment_count as f64));
        }
        time_nodes_s
    }

    fn brachistochrone_decision_blocks(
        problem: BrachistochroneBenchmarkProblem,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut node_states = Vec::with_capacity((problem.segment_count + 1) * 3);
        let mut midpoint_states = Vec::with_capacity(problem.segment_count * 3);
        let mut node_controls = Vec::with_capacity(problem.segment_count + 1);
        let mut midpoint_controls = Vec::with_capacity(problem.segment_count);

        for node_index in 0..=problem.segment_count {
            let theta =
                problem.theta_final_rad * (node_index as f64) / (problem.segment_count as f64);
            let (state, control) = brachistochrone_sample(problem, theta);
            node_states.extend(state);
            node_controls.push(control);
        }
        for segment_index in 0..problem.segment_count {
            let theta = problem.theta_final_rad * ((segment_index as f64) + 0.5)
                / (problem.segment_count as f64);
            let (state, control) = brachistochrone_sample(problem, theta);
            midpoint_states.extend(state);
            midpoint_controls.push(control);
        }
        (
            node_states,
            midpoint_states,
            node_controls,
            midpoint_controls,
        )
    }

    fn brachistochrone_ocp(
        problem: BrachistochroneBenchmarkProblem,
        final_time_s: f64,
    ) -> Result<HermiteSimpsonFixedMeshOcp, TrajoptError> {
        let time_nodes_s = uniform_time_nodes(problem.segment_count, final_time_s);
        let boundary_constraints = vec![
            HermiteSimpsonBoundaryConstraint::new(0, 0, 0.0)?,
            HermiteSimpsonBoundaryConstraint::new(0, 1, 0.0)?,
            HermiteSimpsonBoundaryConstraint::new(0, 2, 0.0)?,
            HermiteSimpsonBoundaryConstraint::new(problem.segment_count, 0, problem.endpoint_x_m)?,
            HermiteSimpsonBoundaryConstraint::new(
                problem.segment_count,
                1,
                problem.endpoint_y_down_m,
            )?,
        ];
        HermiteSimpsonFixedMeshOcp::new(time_nodes_s, 3, 1, boundary_constraints)
    }

    #[cfg(feature = "sqp")]
    fn perturb_brachistochrone_state_decision(
        layout: &HermiteSimpsonDecisionLayout,
        exact: &[f64],
    ) -> Vec<f64> {
        let mut perturbed = exact.to_vec();
        for node_index in 1..layout.segment_count {
            let offset = layout.node_state_offset + node_index * layout.state_dimension;
            let phase = node_index as f64;
            perturbed[offset] *= 1.0 + 0.005 * phase.sin();
            perturbed[offset + 1] *= 1.0 - 0.004 * phase.cos();
            perturbed[offset + 2] *= 1.0 + 0.003 * (0.5 * phase).sin();
        }
        for segment_index in 0..layout.segment_count {
            let state_offset =
                layout.midpoint_state_offset + segment_index * layout.state_dimension;
            let phase = (segment_index as f64) + 0.5;
            perturbed[state_offset] *= 1.0 - 0.004 * phase.sin();
            perturbed[state_offset + 1] *= 1.0 + 0.003 * phase.cos();
            perturbed[state_offset + 2] *= 1.0 - 0.003 * (0.25 * phase).cos();
        }
        perturbed
    }

    #[cfg(feature = "sqp")]
    fn brachistochrone_state_restoration_bounds(
        layout: &HermiteSimpsonDecisionLayout,
        exact: &[f64],
    ) -> (Vec<f64>, Vec<f64>) {
        let mut lower = vec![-10.0; layout.variable_count];
        let mut upper = vec![10.0; layout.variable_count];
        for node_index in 0..=layout.segment_count {
            let offset = layout.node_state_offset + node_index * layout.state_dimension;
            lower[offset] = -0.1;
            upper[offset] = 1.2;
            lower[offset + 1] = -0.1;
            upper[offset + 1] = 1.2;
            lower[offset + 2] = 0.0;
            upper[offset + 2] = 10.0;
        }
        for segment_index in 0..layout.segment_count {
            let offset = layout.midpoint_state_offset + segment_index * layout.state_dimension;
            lower[offset] = -0.1;
            upper[offset] = 1.2;
            lower[offset + 1] = -0.1;
            upper[offset + 1] = 1.2;
            lower[offset + 2] = 0.0;
            upper[offset + 2] = 10.0;
        }
        for node_index in 0..=layout.segment_count {
            let offset = layout.node_control_offset + node_index * layout.control_dimension;
            lower[offset] = exact[offset];
            upper[offset] = exact[offset];
        }
        for segment_index in 0..layout.segment_count {
            let offset = layout.midpoint_control_offset + segment_index * layout.control_dimension;
            lower[offset] = exact[offset];
            upper[offset] = exact[offset];
        }
        (lower, upper)
    }

    #[cfg(feature = "sqp")]
    fn perturb_brachistochrone_free_control_decision(
        layout: &HermiteSimpsonDecisionLayout,
        exact: &[f64],
    ) -> Vec<f64> {
        let mut perturbed = perturb_brachistochrone_state_decision(layout, exact);
        for node_index in 0..=layout.segment_count {
            let offset = layout.node_control_offset + node_index * layout.control_dimension;
            let phase = (node_index as f64) + 1.0;
            perturbed[offset] += 0.01 * phase.sin();
        }
        for segment_index in 0..layout.segment_count {
            let offset = layout.midpoint_control_offset + segment_index * layout.control_dimension;
            let phase = (segment_index as f64) + 0.5;
            perturbed[offset] += 0.01 * phase.cos();
        }
        perturbed
    }

    #[cfg(feature = "sqp")]
    fn brachistochrone_free_control_bounds(
        layout: &HermiteSimpsonDecisionLayout,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut lower = vec![-10.0; layout.variable_count];
        let mut upper = vec![10.0; layout.variable_count];
        for node_index in 0..=layout.segment_count {
            let offset = layout.node_state_offset + node_index * layout.state_dimension;
            lower[offset] = -0.1;
            upper[offset] = 1.2;
            lower[offset + 1] = -0.1;
            upper[offset + 1] = 1.2;
            lower[offset + 2] = 0.0;
            upper[offset + 2] = 10.0;
        }
        for segment_index in 0..layout.segment_count {
            let offset = layout.midpoint_state_offset + segment_index * layout.state_dimension;
            lower[offset] = -0.1;
            upper[offset] = 1.2;
            lower[offset + 1] = -0.1;
            upper[offset + 1] = 1.2;
            lower[offset + 2] = 0.0;
            upper[offset + 2] = 10.0;
        }
        for node_index in 0..=layout.segment_count {
            let offset = layout.node_control_offset + node_index * layout.control_dimension;
            lower[offset] = 0.0;
            upper[offset] = core::f64::consts::PI;
        }
        for segment_index in 0..layout.segment_count {
            let offset = layout.midpoint_control_offset + segment_index * layout.control_dimension;
            lower[offset] = 0.0;
            upper[offset] = core::f64::consts::PI;
        }
        (lower, upper)
    }

    #[cfg(feature = "sqp")]
    #[derive(Clone, Copy)]
    struct BrachistochroneDynamics {
        gravity_m_s2: f64,
    }

    #[cfg(feature = "sqp")]
    impl CollocationDynamics for BrachistochroneDynamics {
        fn eval(
            &mut self,
            _time_s: f64,
            state: &[f64],
            control: &[f64],
        ) -> Result<Vec<f64>, TrajoptError> {
            brachistochrone_dynamics(self.gravity_m_s2, state, control)
        }
    }

    fn brachistochrone_sample(
        problem: BrachistochroneBenchmarkProblem,
        theta: f64,
    ) -> ([f64; 3], f64) {
        let x = problem.cycloid_scale_m * (theta - theta.sin());
        let y_down = problem.cycloid_scale_m * (1.0 - theta.cos());
        let speed = (2.0 * problem.gravity_m_s2 * y_down).sqrt();
        let control_angle_rad = core::f64::consts::FRAC_PI_2 - 0.5 * theta;
        ([x, y_down, speed], control_angle_rad)
    }

    fn brachistochrone_dynamics(
        gravity_m_s2: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        if state.len() != 3 || control.len() != 1 {
            return Err(TrajoptError::InvalidPayload {
                reason: "brachistochrone dynamics dimensions are inconsistent",
            });
        }
        let speed = state[2];
        let control_angle_rad = control[0];
        if !gravity_m_s2.is_finite()
            || gravity_m_s2 <= 0.0
            || !speed.is_finite()
            || speed < 0.0
            || !control_angle_rad.is_finite()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "brachistochrone dynamics values are invalid",
            });
        }
        Ok(vec![
            speed * control_angle_rad.cos(),
            speed * control_angle_rad.sin(),
            gravity_m_s2 * control_angle_rad.sin(),
        ])
    }

    fn time_nodes_from_durations(durations_s: &[f64]) -> Vec<f64> {
        let mut elapsed_s = 0.0;
        let mut time_nodes_s = Vec::with_capacity(durations_s.len() + 1);
        time_nodes_s.push(elapsed_s);
        for duration_s in durations_s {
            elapsed_s += duration_s;
            time_nodes_s.push(elapsed_s);
        }
        time_nodes_s
    }

    fn flattened_node_states(nodes: &[MultipleShootingNode]) -> Vec<f64> {
        let mut states = Vec::with_capacity(nodes.len() * 6);
        for node in nodes {
            states.extend(node.state.to_array());
        }
        states
    }

    fn two_body_midpoint_states(
        nodes: &[MultipleShootingNode],
        durations_s: &[f64],
        step_s: f64,
        mu_m3_s2: f64,
    ) -> Result<Vec<f64>, TrajoptError> {
        let mut states = Vec::with_capacity(durations_s.len() * 6);
        for (node, duration_s) in nodes.iter().zip(durations_s) {
            let midpoint =
                propagate_two_body_variational(node.state, 0.5 * duration_s, step_s, mu_m3_s2)?
                    .terminal_state;
            states.extend(midpoint.to_array());
        }
        Ok(states)
    }

    fn terminal_boundary_constraints(
        node_index: usize,
        terminal_target: [f64; 6],
    ) -> Result<Vec<HermiteSimpsonBoundaryConstraint>, TrajoptError> {
        let mut constraints = Vec::with_capacity(terminal_target.len());
        for (component, target) in terminal_target.iter().copied().enumerate() {
            constraints.push(HermiteSimpsonBoundaryConstraint::new(
                node_index, component, target,
            )?);
        }
        Ok(constraints)
    }

    fn two_body_dynamics(state: &[f64], control: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        if state.len() != 6 || !control.is_empty() {
            return Err(TrajoptError::InvalidPayload {
                reason: "two-body Hermite-Simpson cross-check dimensions are inconsistent",
            });
        }
        let radius_squared = state[0] * state[0] + state[1] * state[1] + state[2] * state[2];
        if !radius_squared.is_finite() || radius_squared <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "two-body Hermite-Simpson cross-check radius is invalid",
            });
        }
        let radius = radius_squared.sqrt();
        let acceleration_scale = -WGS84_MU_M3_S2 / (radius_squared * radius);
        Ok(vec![
            state[3],
            state[4],
            state[5],
            acceleration_scale * state[0],
            acceleration_scale * state[1],
            acceleration_scale * state[2],
        ])
    }

    fn norm3(value: [f64; 3]) -> f64 {
        value
            .iter()
            .map(|component| component * component)
            .sum::<f64>()
            .sqrt()
    }

    fn dense_gauss_newton_hessian(
        row_count: usize,
        column_count: usize,
        dense_jacobian: &[f64],
        regularization: f64,
    ) -> Vec<f64> {
        assert_eq!(dense_jacobian.len(), row_count * column_count);
        let mut hessian = vec![0.0; column_count * column_count];
        for residual_row in 0..row_count {
            for row in 0..column_count {
                let row_value = dense_jacobian[residual_row * column_count + row];
                for column in 0..column_count {
                    hessian[row * column_count + column] +=
                        row_value * dense_jacobian[residual_row * column_count + column];
                }
            }
        }
        for diagonal in 0..column_count {
            hessian[diagonal * column_count + diagonal] += regularization;
        }
        hessian
    }

    fn dense_gauss_newton_step_rhs(
        row_count: usize,
        column_count: usize,
        dense_jacobian: &[f64],
        residuals: &[f64],
    ) -> Vec<f64> {
        assert_eq!(dense_jacobian.len(), row_count * column_count);
        assert_eq!(residuals.len(), row_count);
        let mut rhs = vec![0.0; column_count];
        for row in 0..row_count {
            for column in 0..column_count {
                rhs[column] -= dense_jacobian[row * column_count + column] * residuals[row];
            }
        }
        rhs
    }

    fn dense_augmented_kkt(
        row_count: usize,
        column_count: usize,
        dense_jacobian: &[f64],
        regularization: f64,
    ) -> Vec<f64> {
        assert_eq!(dense_jacobian.len(), row_count * column_count);
        let dimension = row_count + column_count;
        let mut kkt = vec![0.0; dimension * dimension];
        for variable in 0..column_count {
            kkt[variable * dimension + variable] = regularization;
        }
        for residual in 0..row_count {
            let index = column_count + residual;
            kkt[index * dimension + index] = -1.0;
        }
        for residual in 0..row_count {
            for variable in 0..column_count {
                let value = dense_jacobian[residual * column_count + variable];
                let residual_index = column_count + residual;
                kkt[variable * dimension + residual_index] = value;
                kkt[residual_index * dimension + variable] = value;
            }
        }
        kkt
    }

    fn dense_constrained_augmented_kkt(
        row_count: usize,
        column_count: usize,
        dense_jacobian: &[f64],
        hard_path_inequality_report: &HermiteSimpsonPathInequalityReport,
        active_rows: &[usize],
        regularization: f64,
    ) -> Vec<f64> {
        assert_eq!(dense_jacobian.len(), row_count * column_count);
        assert_eq!(hard_path_inequality_report.variable_count, column_count);
        let active_count = active_rows.len();
        let dimension = row_count + column_count + active_count;
        let mut kkt = vec![0.0; dimension * dimension];
        for variable in 0..column_count {
            kkt[variable * dimension + variable] = regularization;
        }
        for residual in 0..row_count {
            let index = column_count + residual;
            kkt[index * dimension + index] = -1.0;
        }
        for residual in 0..row_count {
            for variable in 0..column_count {
                let value = dense_jacobian[residual * column_count + variable];
                let residual_index = column_count + residual;
                kkt[variable * dimension + residual_index] = value;
                kkt[residual_index * dimension + variable] = value;
            }
        }
        for (active_index, source_row) in active_rows.iter().enumerate() {
            let active_kkt_index = column_count + row_count + active_index;
            let source_offset = *source_row * column_count;
            for variable in 0..column_count {
                let value = hard_path_inequality_report.inequality_matrix[source_offset + variable];
                kkt[variable * dimension + active_kkt_index] = value;
                kkt[active_kkt_index * dimension + variable] = value;
            }
        }
        kkt
    }

    fn dense_matrix_vector(dimension: usize, dense_matrix: &[f64], vector: &[f64]) -> Vec<f64> {
        assert_eq!(dense_matrix.len(), dimension * dimension);
        assert_eq!(vector.len(), dimension);
        let mut product = vec![0.0; dimension];
        for row in 0..dimension {
            for column in 0..dimension {
                product[row] += dense_matrix[row * dimension + column] * vector[column];
            }
        }
        product
    }

    fn dense_rectangular_matrix_vector(
        row_count: usize,
        column_count: usize,
        dense_matrix: &[f64],
        vector: &[f64],
    ) -> Vec<f64> {
        assert_eq!(dense_matrix.len(), row_count * column_count);
        assert_eq!(vector.len(), column_count);
        let mut product = vec![0.0; row_count];
        for row in 0..row_count {
            for column in 0..column_count {
                product[row] += dense_matrix[row * column_count + column] * vector[column];
            }
        }
        product
    }

    #[cfg(feature = "sqp")]
    fn bryson_denham_control_hessian(ocp: &HermiteSimpsonFixedMeshOcp) -> Vec<f64> {
        let n = ocp.layout.variable_count;
        let mut hessian = vec![0.0; n * n];
        for segment_index in 0..ocp.layout.segment_count {
            let h = ocp.time_nodes_s[segment_index + 1] - ocp.time_nodes_s[segment_index];
            let node0 = scalar_decision_index(
                &ocp.layout,
                HermiteSimpsonSamplePoint::Node(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )
            .expect("node control index");
            let midpoint = scalar_decision_index(
                &ocp.layout,
                HermiteSimpsonSamplePoint::Midpoint(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )
            .expect("midpoint control index");
            let node1 = scalar_decision_index(
                &ocp.layout,
                HermiteSimpsonSamplePoint::Node(segment_index + 1),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )
            .expect("node control index");
            hessian[node0 * n + node0] += h / 6.0;
            hessian[midpoint * n + midpoint] += 2.0 * h / 3.0;
            hessian[node1 * n + node1] += h / 6.0;
        }
        hessian
    }

    #[cfg(feature = "sqp")]
    fn assert_bryson_denham_active_arc_solution(
        ocp: &HermiteSimpsonFixedMeshOcp,
        decision: &[f64],
        problem: BrysonDenhamBenchmarkProblem,
    ) {
        for node_index in 0..=ocp.layout.segment_count {
            let time_s = ocp.time_nodes_s[node_index];
            assert_bryson_denham_sample(
                ocp,
                decision,
                HermiteSimpsonSamplePoint::Node(node_index),
                time_s,
                problem,
            );
        }
        for segment_index in 0..ocp.layout.segment_count {
            let time_s =
                0.5 * (ocp.time_nodes_s[segment_index] + ocp.time_nodes_s[segment_index + 1]);
            assert_bryson_denham_sample(
                ocp,
                decision,
                HermiteSimpsonSamplePoint::Midpoint(segment_index),
                time_s,
                problem,
            );
        }
    }

    #[cfg(feature = "sqp")]
    fn assert_bryson_denham_sample(
        ocp: &HermiteSimpsonFixedMeshOcp,
        decision: &[f64],
        sample: HermiteSimpsonSamplePoint,
        time_s: f64,
        problem: BrysonDenhamBenchmarkProblem,
    ) {
        let (expected_x, expected_v, expected_u) = bryson_denham_active_arc_sample(time_s);
        let x = sample_scalar(
            &ocp.layout,
            decision,
            sample,
            HermiteSimpsonScalarVariable::StateComponent { component: 0 },
        )
        .expect("state x sample");
        let v = sample_scalar(
            &ocp.layout,
            decision,
            sample,
            HermiteSimpsonScalarVariable::StateComponent { component: 1 },
        )
        .expect("state v sample");
        let u = sample_scalar(
            &ocp.layout,
            decision,
            sample,
            HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
        )
        .expect("control sample");
        assert!(
            (x - expected_x).abs() <= problem.max_state_error,
            "Bryson-Denham x({time_s}) expected {expected_x}, got {x}"
        );
        assert!(
            (v - expected_v).abs() <= problem.max_state_error,
            "Bryson-Denham v({time_s}) expected {expected_v}, got {v}"
        );
        assert!(
            (u - expected_u).abs() <= problem.max_control_error,
            "Bryson-Denham u({time_s}) expected {expected_u}, got {u}"
        );
    }

    #[cfg(feature = "sqp")]
    fn bryson_denham_active_arc_sample(time_s: f64) -> (f64, f64, f64) {
        if time_s <= (1.0 / 3.0) {
            let x = time_s - 3.0 * time_s * time_s + 3.0 * time_s * time_s * time_s;
            let v = 1.0 - 6.0 * time_s + 9.0 * time_s * time_s;
            let u = 18.0 * time_s - 6.0;
            (x, v, u)
        } else if time_s <= (2.0 / 3.0) {
            (1.0 / 9.0, 0.0, 0.0)
        } else {
            let tau = time_s - (2.0 / 3.0);
            let x = 1.0 / 9.0 - 3.0 * tau * tau * tau;
            let v = -9.0 * tau * tau;
            let u = -18.0 * tau;
            (x, v, u)
        }
    }

    #[cfg(feature = "sqp")]
    fn scalar_integrator_min_energy_hessian(ocp: &HermiteSimpsonFixedMeshOcp) -> Vec<f64> {
        let n = ocp.layout.variable_count;
        let mut hessian = vec![0.0; n * n];
        for segment_index in 0..ocp.layout.segment_count {
            let h = ocp.time_nodes_s[segment_index + 1] - ocp.time_nodes_s[segment_index];
            let node0 = scalar_decision_index(
                &ocp.layout,
                HermiteSimpsonSamplePoint::Node(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )
            .expect("node control index");
            let midpoint = scalar_decision_index(
                &ocp.layout,
                HermiteSimpsonSamplePoint::Midpoint(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )
            .expect("midpoint control index");
            let node1 = scalar_decision_index(
                &ocp.layout,
                HermiteSimpsonSamplePoint::Node(segment_index + 1),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )
            .expect("node control index");
            hessian[node0 * n + node0] += h / 3.0;
            hessian[midpoint * n + midpoint] += 4.0 * h / 3.0;
            hessian[node1 * n + node1] += h / 3.0;
        }
        hessian
    }

    #[cfg(feature = "sqp")]
    fn scalar_integrator_control_cost(
        ocp: &HermiteSimpsonFixedMeshOcp,
        decision: &[f64],
    ) -> Result<f64, TrajoptError> {
        let mut cost = 0.0;
        for segment_index in 0..ocp.layout.segment_count {
            let h = ocp.time_nodes_s[segment_index + 1] - ocp.time_nodes_s[segment_index];
            let u0 = sample_scalar(
                &ocp.layout,
                decision,
                HermiteSimpsonSamplePoint::Node(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )?;
            let um = sample_scalar(
                &ocp.layout,
                decision,
                HermiteSimpsonSamplePoint::Midpoint(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )?;
            let u1 = sample_scalar(
                &ocp.layout,
                decision,
                HermiteSimpsonSamplePoint::Node(segment_index + 1),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )?;
            cost += h * (u0 * u0 + 4.0 * um * um + u1 * u1) / 6.0;
        }
        Ok(cost)
    }

    #[cfg(feature = "sqp")]
    fn assert_scalar_integrator_analytic_solution(
        ocp: &HermiteSimpsonFixedMeshOcp,
        decision: &[f64],
        table: &ScalarIntegratorBenchmarkTable,
    ) {
        for node_index in 0..=ocp.layout.segment_count {
            let expected_time = ocp.time_nodes_s[node_index];
            let state = sample_scalar(
                &ocp.layout,
                decision,
                HermiteSimpsonSamplePoint::Node(node_index),
                HermiteSimpsonScalarVariable::StateComponent { component: 0 },
            )
            .expect("node state");
            let control = sample_scalar(
                &ocp.layout,
                decision,
                HermiteSimpsonSamplePoint::Node(node_index),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )
            .expect("node control");
            assert!(
                (state - expected_time).abs() <= table.max_state_error,
                "node state {node_index} expected {expected_time}, got {state}"
            );
            assert!(
                (control - 1.0).abs() <= table.max_control_error,
                "node control {node_index} expected 1, got {control}"
            );
        }
        for segment_index in 0..ocp.layout.segment_count {
            let expected_time =
                0.5 * (ocp.time_nodes_s[segment_index] + ocp.time_nodes_s[segment_index + 1]);
            let state = sample_scalar(
                &ocp.layout,
                decision,
                HermiteSimpsonSamplePoint::Midpoint(segment_index),
                HermiteSimpsonScalarVariable::StateComponent { component: 0 },
            )
            .expect("midpoint state");
            let control = sample_scalar(
                &ocp.layout,
                decision,
                HermiteSimpsonSamplePoint::Midpoint(segment_index),
                HermiteSimpsonScalarVariable::ControlComponent { component: 0 },
            )
            .expect("midpoint control");
            assert!(
                (state - expected_time).abs() <= table.max_state_error,
                "midpoint state {segment_index} expected {expected_time}, got {state}"
            );
            assert!(
                (control - 1.0).abs() <= table.max_control_error,
                "midpoint control {segment_index} expected 1, got {control}"
            );
        }
    }

    fn assert_jacobian_entry(
        jacobian: &HermiteSimpsonSparseResidualJacobianReport,
        row: usize,
        column: usize,
        expected: f64,
    ) {
        let actual = jacobian
            .coefficient(row, column)
            .expect("Jacobian coefficient exists");
        assert!(
            (actual - expected).abs() < 1.0e-8,
            "Jacobian({row}, {column}) expected {expected}, got {actual}"
        );
    }

    fn assert_path_jacobian_entry(
        jacobian: &HermiteSimpsonSparsePathInequalityJacobianReport,
        row: usize,
        column: usize,
        expected: f64,
    ) {
        let actual = jacobian
            .coefficient(row, column)
            .expect("path Jacobian coefficient exists");
        assert!(
            (actual - expected).abs() < 1.0e-8,
            "path Jacobian({row}, {column}) expected {expected}, got {actual}"
        );
    }

    fn log_log_slope(steps: &[f64], values: &[f64]) -> f64 {
        assert_eq!(steps.len(), values.len());
        let n = steps.len() as f64;
        let mean_x = steps.iter().map(|step| step.ln()).sum::<f64>() / n;
        let mean_y = values.iter().map(|value| value.ln()).sum::<f64>() / n;
        let numerator = steps
            .iter()
            .zip(values)
            .map(|(step, value)| (step.ln() - mean_x) * (value.ln() - mean_y))
            .sum::<f64>();
        let denominator = steps
            .iter()
            .map(|step| {
                let centered = step.ln() - mean_x;
                centered * centered
            })
            .sum::<f64>();
        numerator / denominator
    }
}
