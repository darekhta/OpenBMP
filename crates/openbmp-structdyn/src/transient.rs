//! Native modal transient integration.
//!
//! The shipped integrators are Newmark-β and generalized-α with constant time
//! step and precomputed effective-stiffness Cholesky factors for the default
//! constant-system path. They also expose explicit one-step alternate-system
//! methods for caller-owned variable-stiffness modal transients.

use crate::{DenseCholesky, DenseMatrix, StructDynError, add_scaled_matrix, mul_vec_unchecked};

/// Newmark-β parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NewmarkParameters {
    /// Newmark beta parameter.
    pub beta: f64,
    /// Newmark gamma parameter.
    pub gamma: f64,
}

impl NewmarkParameters {
    /// Constant-average-acceleration Newmark parameters (`β = 1/4`, `γ = 1/2`).
    pub const AVERAGE_ACCELERATION: Self = Self {
        beta: 0.25,
        gamma: 0.5,
    };

    /// Validate parameters.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when either parameter is non-finite or not
    /// positive.
    pub fn validate(self) -> Result<Self, StructDynError> {
        if !self.beta.is_finite() || self.beta <= 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "newmark.beta",
            });
        }
        if !self.gamma.is_finite() || self.gamma <= 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "newmark.gamma",
            });
        }
        Ok(self)
    }
}

/// Generalized-α parameter set derived from a pinned spectral radius.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeneralizedAlphaParameters {
    /// High-frequency spectral radius.
    pub rho_inf: f64,
    /// Generalized-α mass weighting parameter.
    pub alpha_m: f64,
    /// Generalized-α force/stiffness weighting parameter.
    pub alpha_f: f64,
    /// Implied beta parameter.
    pub beta: f64,
    /// Implied gamma parameter.
    pub gamma: f64,
}

impl GeneralizedAlphaParameters {
    /// Derive generalized-α parameters from `rho_inf`.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when `rho_inf` is outside `[0, 1]`.
    pub fn from_spectral_radius(rho_inf: f64) -> Result<Self, StructDynError> {
        if !rho_inf.is_finite() || !(0.0..=1.0).contains(&rho_inf) {
            return Err(StructDynError::InvalidNumber {
                field: "generalized_alpha.rho_inf",
            });
        }
        let alpha_m = (2.0 * rho_inf - 1.0) / (rho_inf + 1.0);
        let alpha_f = rho_inf / (rho_inf + 1.0);
        let gamma = 0.5 - alpha_m + alpha_f;
        let beta = 0.25 * (1.0 - alpha_m + alpha_f) * (1.0 - alpha_m + alpha_f);
        Ok(Self {
            rho_inf,
            alpha_m,
            alpha_f,
            beta,
            gamma,
        })
    }

    /// Validate an explicitly constructed parameter set.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any parameter is non-finite, `rho_inf`
    /// is outside `[0, 1]`, or beta/gamma are not positive.
    pub fn validate(self) -> Result<Self, StructDynError> {
        if !self.rho_inf.is_finite() || !(0.0..=1.0).contains(&self.rho_inf) {
            return Err(StructDynError::InvalidNumber {
                field: "generalized_alpha.rho_inf",
            });
        }
        if !self.alpha_m.is_finite() {
            return Err(StructDynError::InvalidNumber {
                field: "generalized_alpha.alpha_m",
            });
        }
        if !self.alpha_f.is_finite() {
            return Err(StructDynError::InvalidNumber {
                field: "generalized_alpha.alpha_f",
            });
        }
        if !self.beta.is_finite() || self.beta <= 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "generalized_alpha.beta",
            });
        }
        if !self.gamma.is_finite() || self.gamma <= 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "generalized_alpha.gamma",
            });
        }
        Ok(self)
    }
}

/// Linear modal transient system `M q_ddot + C q_dot + K q = f`.
#[derive(Clone, Debug, PartialEq)]
pub struct ModalTransientSystem {
    mass: DenseMatrix,
    damping: DenseMatrix,
    stiffness: DenseMatrix,
}

impl ModalTransientSystem {
    /// Construct and validate a modal transient system.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when matrices are not conforming square
    /// matrices or the mass matrix is not positive definite.
    pub fn new(
        mass: DenseMatrix,
        damping: DenseMatrix,
        stiffness: DenseMatrix,
    ) -> Result<Self, StructDynError> {
        validate_square_same_size("damping", &mass, &damping)?;
        validate_square_same_size("stiffness", &mass, &stiffness)?;
        DenseCholesky::factor(&mass, "modal.mass")?;
        Ok(Self {
            mass,
            damping,
            stiffness,
        })
    }

    /// Number of modal coordinates.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.mass.rows()
    }

    /// Whether the system has no coordinates. A constructed system is never empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.mass.rows() == 0
    }

    /// Mass matrix.
    #[must_use]
    pub const fn mass(&self) -> &DenseMatrix {
        &self.mass
    }

    /// Damping matrix.
    #[must_use]
    pub const fn damping(&self) -> &DenseMatrix {
        &self.damping
    }

    /// Stiffness matrix.
    #[must_use]
    pub const fn stiffness(&self) -> &DenseMatrix {
        &self.stiffness
    }
}

/// Modal transient state.
#[derive(Clone, Debug, PartialEq)]
pub struct ModalTransientState {
    /// Modal coordinates.
    pub q: Vec<f64>,
    /// Modal velocities.
    pub q_dot: Vec<f64>,
    /// Modal accelerations.
    pub q_ddot: Vec<f64>,
}

impl ModalTransientState {
    /// Construct and validate a modal transient state.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when vector dimensions differ from
    /// `coordinate_count` or contain non-finite values.
    pub fn new(
        q: Vec<f64>,
        q_dot: Vec<f64>,
        q_ddot: Vec<f64>,
        coordinate_count: usize,
    ) -> Result<Self, StructDynError> {
        validate_vector_len("modal.q", &q, coordinate_count)?;
        validate_vector_len("modal.q_dot", &q_dot, coordinate_count)?;
        validate_vector_len("modal.q_ddot", &q_ddot, coordinate_count)?;
        Ok(Self { q, q_dot, q_ddot })
    }
}

/// Constant-step Newmark-β transient integrator.
#[derive(Clone, Debug, PartialEq)]
pub struct NewmarkIntegrator {
    system: ModalTransientSystem,
    dt_s: f64,
    parameters: NewmarkParameters,
    effective_factor: DenseCholesky,
}

/// Constant-step generalized-α transient integrator.
#[derive(Clone, Debug, PartialEq)]
pub struct GeneralizedAlphaIntegrator {
    system: ModalTransientSystem,
    dt_s: f64,
    parameters: GeneralizedAlphaParameters,
    effective_factor: DenseCholesky,
}

impl GeneralizedAlphaIntegrator {
    /// Construct a generalized-α integrator and precompute the effective
    /// stiffness Cholesky factor for the constant time step.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when parameters are invalid or the effective
    /// stiffness is not positive definite.
    pub fn new(
        system: ModalTransientSystem,
        dt_s: f64,
        parameters: GeneralizedAlphaParameters,
    ) -> Result<Self, StructDynError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "generalized_alpha.dt_s",
            });
        }
        let parameters = parameters.validate()?;
        let a0 = 1.0 / (parameters.beta * dt_s * dt_s);
        let a1 = parameters.gamma / (parameters.beta * dt_s);
        let mut effective = DenseMatrix::zeros(system.len(), system.len())?;
        add_scaled_matrix(
            &mut effective,
            &system.mass,
            (1.0 - parameters.alpha_m) * a0,
        )?;
        add_scaled_matrix(
            &mut effective,
            &system.damping,
            (1.0 - parameters.alpha_f) * a1,
        )?;
        add_scaled_matrix(&mut effective, &system.stiffness, 1.0 - parameters.alpha_f)?;
        let effective_factor = DenseCholesky::factor(&effective, "generalized_alpha.k_eff")?;
        Ok(Self {
            system,
            dt_s,
            parameters,
            effective_factor,
        })
    }

    /// Time step, seconds.
    #[must_use]
    pub const fn dt_s(&self) -> f64 {
        self.dt_s
    }

    /// Pinned generalized-α parameters.
    #[must_use]
    pub const fn parameters(&self) -> GeneralizedAlphaParameters {
        self.parameters
    }

    /// Advance one generalized-α step using forces at `t_n` and `t_{n+1}`.
    ///
    /// The equilibrium is evaluated at the generalized-α weighted state:
    /// `(1-α_m) a_{n+1} + α_m a_n` for inertia and
    /// `(1-α_f) {q,v,f}_{n+1} + α_f {q,v,f}_n` for stiffness, damping, and
    /// load.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when dimensions do not match or the effective
    /// solve fails.
    pub fn step(
        &self,
        state: &ModalTransientState,
        force_current: &[f64],
        force_next: &[f64],
    ) -> Result<ModalTransientState, StructDynError> {
        generalized_alpha_step_with_factor(GeneralizedAlphaStepWork {
            current_system: &self.system,
            next_system: &self.system,
            dt_s: self.dt_s,
            parameters: self.parameters,
            effective_factor: &self.effective_factor,
            state,
            force_current,
            force_next,
        })
    }

    /// Advance one generalized-α step with caller-provided current and next
    /// transient systems.
    ///
    /// This path factors the next-step effective stiffness for this single
    /// step. It is intended for variable-stiffness modal systems whose mass,
    /// damping, or stiffness matrices change between steps while preserving the
    /// integrator time step and parameters.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when dimensions do not match or the effective
    /// solve fails.
    pub fn step_with_systems(
        &self,
        current_system: &ModalTransientSystem,
        next_system: &ModalTransientSystem,
        state: &ModalTransientState,
        force_current: &[f64],
        force_next: &[f64],
    ) -> Result<ModalTransientState, StructDynError> {
        validate_system_len(
            "generalized_alpha.current_system",
            current_system,
            self.system.len(),
        )?;
        validate_system_len(
            "generalized_alpha.next_system",
            next_system,
            self.system.len(),
        )?;
        let effective_factor =
            generalized_alpha_effective_factor(next_system, self.dt_s, self.parameters)?;
        generalized_alpha_step_with_factor(GeneralizedAlphaStepWork {
            current_system,
            next_system,
            dt_s: self.dt_s,
            parameters: self.parameters,
            effective_factor: &effective_factor,
            state,
            force_current,
            force_next,
        })
    }
}

impl NewmarkIntegrator {
    /// Construct a Newmark integrator and precompute the effective stiffness
    /// Cholesky factor for the constant time step.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when parameters are invalid or the effective
    /// stiffness is not positive definite.
    pub fn new(
        system: ModalTransientSystem,
        dt_s: f64,
        parameters: NewmarkParameters,
    ) -> Result<Self, StructDynError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "newmark.dt_s",
            });
        }
        let parameters = parameters.validate()?;
        let a0 = 1.0 / (parameters.beta * dt_s * dt_s);
        let a1 = parameters.gamma / (parameters.beta * dt_s);
        let mut effective = system.stiffness.clone();
        add_scaled_matrix(&mut effective, &system.mass, a0)?;
        add_scaled_matrix(&mut effective, &system.damping, a1)?;
        let effective_factor = DenseCholesky::factor(&effective, "newmark.k_eff")?;
        Ok(Self {
            system,
            dt_s,
            parameters,
            effective_factor,
        })
    }

    /// Time step, seconds.
    #[must_use]
    pub const fn dt_s(&self) -> f64 {
        self.dt_s
    }

    /// Pinned Newmark parameters.
    #[must_use]
    pub const fn parameters(&self) -> NewmarkParameters {
        self.parameters
    }

    /// Compute acceleration from the equation of motion at a known state.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when dimensions do not match or the mass solve
    /// fails.
    pub fn acceleration_from_force(
        &self,
        q: &[f64],
        q_dot: &[f64],
        force: &[f64],
    ) -> Result<Vec<f64>, StructDynError> {
        let n = self.system.len();
        validate_vector_len("modal.q", q, n)?;
        validate_vector_len("modal.q_dot", q_dot, n)?;
        validate_vector_len("modal.force", force, n)?;
        let damping_force = self.system.damping.mul_vec(q_dot)?;
        let stiffness_force = self.system.stiffness.mul_vec(q)?;
        let rhs = force
            .iter()
            .zip(damping_force.iter().zip(stiffness_force.iter()))
            .map(|(force_i, (damping_i, stiffness_i))| *force_i - *damping_i - *stiffness_i)
            .collect::<Vec<_>>();
        DenseCholesky::factor(&self.system.mass, "modal.mass")?.solve(&rhs)
    }

    /// Advance one Newmark step using force at `t_{n+1}`.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when dimensions do not match or the effective
    /// solve fails.
    pub fn step(
        &self,
        state: &ModalTransientState,
        force_next: &[f64],
    ) -> Result<ModalTransientState, StructDynError> {
        newmark_step_with_factor(
            &self.system,
            self.dt_s,
            self.parameters,
            &self.effective_factor,
            state,
            force_next,
        )
    }

    /// Advance one Newmark step with a caller-provided next-step transient
    /// system.
    ///
    /// This path factors the effective stiffness for this single step. It is
    /// intended for variable-stiffness modal systems whose mass, damping, or
    /// stiffness matrices change between steps while preserving the integrator
    /// time step and parameters.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when dimensions do not match or the effective
    /// solve fails.
    pub fn step_with_system(
        &self,
        next_system: &ModalTransientSystem,
        state: &ModalTransientState,
        force_next: &[f64],
    ) -> Result<ModalTransientState, StructDynError> {
        validate_system_len("newmark.next_system", next_system, self.system.len())?;
        let effective_factor = newmark_effective_factor(next_system, self.dt_s, self.parameters)?;
        newmark_step_with_factor(
            next_system,
            self.dt_s,
            self.parameters,
            &effective_factor,
            state,
            force_next,
        )
    }
}

fn generalized_alpha_effective_factor(
    system: &ModalTransientSystem,
    dt_s: f64,
    parameters: GeneralizedAlphaParameters,
) -> Result<DenseCholesky, StructDynError> {
    let a0 = 1.0 / (parameters.beta * dt_s * dt_s);
    let a1 = parameters.gamma / (parameters.beta * dt_s);
    let mut effective = DenseMatrix::zeros(system.len(), system.len())?;
    add_scaled_matrix(
        &mut effective,
        &system.mass,
        (1.0 - parameters.alpha_m) * a0,
    )?;
    add_scaled_matrix(
        &mut effective,
        &system.damping,
        (1.0 - parameters.alpha_f) * a1,
    )?;
    add_scaled_matrix(&mut effective, &system.stiffness, 1.0 - parameters.alpha_f)?;
    DenseCholesky::factor(&effective, "generalized_alpha.k_eff")
}

struct GeneralizedAlphaStepWork<'a> {
    current_system: &'a ModalTransientSystem,
    next_system: &'a ModalTransientSystem,
    dt_s: f64,
    parameters: GeneralizedAlphaParameters,
    effective_factor: &'a DenseCholesky,
    state: &'a ModalTransientState,
    force_current: &'a [f64],
    force_next: &'a [f64],
}

fn generalized_alpha_step_with_factor(
    work: GeneralizedAlphaStepWork<'_>,
) -> Result<ModalTransientState, StructDynError> {
    let n = work.next_system.len();
    validate_system_len("generalized_alpha.current_system", work.current_system, n)?;
    validate_vector_len("modal.force_current", work.force_current, n)?;
    validate_vector_len("modal.force_next", work.force_next, n)?;
    validate_vector_len("modal.q", &work.state.q, n)?;
    validate_vector_len("modal.q_dot", &work.state.q_dot, n)?;
    validate_vector_len("modal.q_ddot", &work.state.q_ddot, n)?;

    let beta = work.parameters.beta;
    let gamma = work.parameters.gamma;
    let alpha_m = work.parameters.alpha_m;
    let alpha_f = work.parameters.alpha_f;
    let dt = work.dt_s;
    let a0 = 1.0 / (beta * dt * dt);
    let a2 = 1.0 / (beta * dt);
    let a3 = (1.0 / (2.0 * beta)) - 1.0;
    let a1 = gamma / (beta * dt);
    let a4 = (gamma / beta) - 1.0;
    let a5 = dt * ((gamma / (2.0 * beta)) - 1.0);

    let mass_predictor = (0..n)
        .map(|i| a0 * work.state.q[i] + a2 * work.state.q_dot[i] + a3 * work.state.q_ddot[i])
        .collect::<Vec<_>>();
    let damping_predictor = (0..n)
        .map(|i| a1 * work.state.q[i] + a4 * work.state.q_dot[i] + a5 * work.state.q_ddot[i])
        .collect::<Vec<_>>();
    let mass_rhs = mul_vec_unchecked(&work.next_system.mass, &mass_predictor);
    let damping_rhs = mul_vec_unchecked(&work.next_system.damping, &damping_predictor);
    let current_inertia = mul_vec_unchecked(&work.current_system.mass, &work.state.q_ddot);
    let current_damping = mul_vec_unchecked(&work.current_system.damping, &work.state.q_dot);
    let current_stiffness = mul_vec_unchecked(&work.current_system.stiffness, &work.state.q);
    let rhs = (0..n)
        .map(|i| {
            (1.0 - alpha_f) * work.force_next[i]
                + alpha_f * work.force_current[i]
                + (1.0 - alpha_m) * mass_rhs[i]
                + (1.0 - alpha_f) * damping_rhs[i]
                - alpha_m * current_inertia[i]
                - alpha_f * current_damping[i]
                - alpha_f * current_stiffness[i]
        })
        .collect::<Vec<_>>();

    let q_next = work.effective_factor.solve(&rhs)?;
    let q_ddot_next = (0..n)
        .map(|i| {
            a0 * (q_next[i] - work.state.q[i])
                - a2 * work.state.q_dot[i]
                - a3 * work.state.q_ddot[i]
        })
        .collect::<Vec<_>>();
    let q_dot_next = (0..n)
        .map(|i| {
            work.state.q_dot[i]
                + dt * ((1.0 - gamma) * work.state.q_ddot[i] + gamma * q_ddot_next[i])
        })
        .collect::<Vec<_>>();
    ModalTransientState::new(q_next, q_dot_next, q_ddot_next, n)
}

fn newmark_effective_factor(
    system: &ModalTransientSystem,
    dt_s: f64,
    parameters: NewmarkParameters,
) -> Result<DenseCholesky, StructDynError> {
    let a0 = 1.0 / (parameters.beta * dt_s * dt_s);
    let a1 = parameters.gamma / (parameters.beta * dt_s);
    let mut effective = system.stiffness.clone();
    add_scaled_matrix(&mut effective, &system.mass, a0)?;
    add_scaled_matrix(&mut effective, &system.damping, a1)?;
    DenseCholesky::factor(&effective, "newmark.k_eff")
}

fn newmark_step_with_factor(
    system: &ModalTransientSystem,
    dt_s: f64,
    parameters: NewmarkParameters,
    effective_factor: &DenseCholesky,
    state: &ModalTransientState,
    force_next: &[f64],
) -> Result<ModalTransientState, StructDynError> {
    let n = system.len();
    validate_vector_len("modal.force_next", force_next, n)?;
    validate_vector_len("modal.q", &state.q, n)?;
    validate_vector_len("modal.q_dot", &state.q_dot, n)?;
    validate_vector_len("modal.q_ddot", &state.q_ddot, n)?;

    let beta = parameters.beta;
    let gamma = parameters.gamma;
    let dt = dt_s;
    let a0 = 1.0 / (beta * dt * dt);
    let a2 = 1.0 / (beta * dt);
    let a3 = (1.0 / (2.0 * beta)) - 1.0;
    let a1 = gamma / (beta * dt);
    let a4 = (gamma / beta) - 1.0;
    let a5 = dt * ((gamma / (2.0 * beta)) - 1.0);

    let mass_predictor = (0..n)
        .map(|i| a0 * state.q[i] + a2 * state.q_dot[i] + a3 * state.q_ddot[i])
        .collect::<Vec<_>>();
    let damping_predictor = (0..n)
        .map(|i| a1 * state.q[i] + a4 * state.q_dot[i] + a5 * state.q_ddot[i])
        .collect::<Vec<_>>();
    let mass_rhs = mul_vec_unchecked(&system.mass, &mass_predictor);
    let damping_rhs = mul_vec_unchecked(&system.damping, &damping_predictor);
    let rhs = force_next
        .iter()
        .zip(mass_rhs.iter().zip(damping_rhs.iter()))
        .map(|(force_i, (mass_i, damping_i))| *force_i + *mass_i + *damping_i)
        .collect::<Vec<_>>();

    let q_next = effective_factor.solve(&rhs)?;
    let q_ddot_next = (0..n)
        .map(|i| a0 * (q_next[i] - state.q[i]) - a2 * state.q_dot[i] - a3 * state.q_ddot[i])
        .collect::<Vec<_>>();
    let q_dot_next = (0..n)
        .map(|i| state.q_dot[i] + dt * ((1.0 - gamma) * state.q_ddot[i] + gamma * q_ddot_next[i]))
        .collect::<Vec<_>>();
    ModalTransientState::new(q_next, q_dot_next, q_ddot_next, n)
}

fn validate_square_same_size(
    field: &'static str,
    reference: &DenseMatrix,
    matrix: &DenseMatrix,
) -> Result<(), StructDynError> {
    if reference.rows() != reference.cols() {
        return Err(StructDynError::DimensionMismatch {
            field: "mass.cols",
            expected: reference.rows(),
            actual: reference.cols(),
        });
    }
    if matrix.rows() != matrix.cols() {
        return Err(StructDynError::DimensionMismatch {
            field,
            expected: matrix.rows(),
            actual: matrix.cols(),
        });
    }
    if matrix.rows() != reference.rows() {
        return Err(StructDynError::DimensionMismatch {
            field,
            expected: reference.rows(),
            actual: matrix.rows(),
        });
    }
    Ok(())
}

fn validate_vector_len(
    field: &'static str,
    values: &[f64],
    expected: usize,
) -> Result<(), StructDynError> {
    if values.len() != expected {
        return Err(StructDynError::DimensionMismatch {
            field,
            expected,
            actual: values.len(),
        });
    }
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(StructDynError::InvalidNumber { field })
    }
}

fn validate_system_len(
    field: &'static str,
    system: &ModalTransientSystem,
    expected: usize,
) -> Result<(), StructDynError> {
    if system.len() == expected {
        Ok(())
    } else {
        Err(StructDynError::DimensionMismatch {
            field,
            expected,
            actual: system.len(),
        })
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    fn scalar_system() -> ModalTransientSystem {
        ModalTransientSystem::new(
            DenseMatrix::from_row_major(1, 1, vec![2.0]).unwrap(),
            DenseMatrix::from_row_major(1, 1, vec![3.0]).unwrap(),
            DenseMatrix::from_row_major(1, 1, vec![4.0]).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn newmark_average_acceleration_recovers_quadratic_mms() {
        let integrator = NewmarkIntegrator::new(
            scalar_system(),
            0.01,
            NewmarkParameters::AVERAGE_ACCELERATION,
        )
        .unwrap();
        let q0 = 0.2;
        let v0 = -0.3;
        let acceleration = 0.7;
        let mut state =
            ModalTransientState::new(vec![q0], vec![v0], vec![acceleration], 1).unwrap();
        let mut max_error = 0.0_f64;

        for step in 1..=100 {
            let t = f64::from(step) * integrator.dt_s();
            let q_exact = q0 + v0 * t + 0.5 * acceleration * t * t;
            let v_exact = v0 + acceleration * t;
            let force = 2.0 * acceleration + 3.0 * v_exact + 4.0 * q_exact;
            state = integrator.step(&state, &[force]).unwrap();
            max_error = max_error.max((state.q[0] - q_exact).abs());
            max_error = max_error.max((state.q_dot[0] - v_exact).abs());
            max_error = max_error.max((state.q_ddot[0] - acceleration).abs());
        }

        assert!(
            max_error < 1.0e-10,
            "quadratic manufactured solution drifted by {max_error}"
        );
    }

    #[test]
    fn acceleration_from_force_satisfies_equation_residual() {
        let integrator = NewmarkIntegrator::new(
            scalar_system(),
            0.01,
            NewmarkParameters::AVERAGE_ACCELERATION,
        )
        .unwrap();
        let q = [0.4];
        let q_dot = [-0.2];
        let q_ddot_expected = 1.25;
        let force = [2.0 * q_ddot_expected + 3.0 * q_dot[0] + 4.0 * q[0]];
        let q_ddot = integrator
            .acceleration_from_force(&q, &q_dot, &force)
            .unwrap();
        assert!((q_ddot[0] - q_ddot_expected).abs() < 1.0e-14);
    }

    #[test]
    fn newmark_step_with_system_matches_integrator_built_for_that_system() {
        let base = NewmarkIntegrator::new(
            scalar_system(),
            0.01,
            NewmarkParameters::AVERAGE_ACCELERATION,
        )
        .unwrap();
        let scaled_system = ModalTransientSystem::new(
            DenseMatrix::from_row_major(1, 1, vec![2.0]).unwrap(),
            DenseMatrix::from_row_major(1, 1, vec![1.5]).unwrap(),
            DenseMatrix::from_row_major(1, 1, vec![16.0]).unwrap(),
        )
        .unwrap();
        let scaled_integrator = NewmarkIntegrator::new(
            scaled_system.clone(),
            0.01,
            NewmarkParameters::AVERAGE_ACCELERATION,
        )
        .unwrap();
        let state = ModalTransientState::new(vec![0.4], vec![-0.2], vec![0.8], 1).unwrap();
        let force_next = [1.7];

        let variable_next = base
            .step_with_system(&scaled_system, &state, &force_next)
            .unwrap();
        let rebuilt_next = scaled_integrator.step(&state, &force_next).unwrap();

        assert_eq!(variable_next, rebuilt_next);
    }

    #[test]
    fn generalized_alpha_parameters_are_pinned_from_spectral_radius() {
        let params = GeneralizedAlphaParameters::from_spectral_radius(1.0).unwrap();
        assert_eq!(params.rho_inf.to_bits(), 1.0_f64.to_bits());
        assert_eq!(params.alpha_m.to_bits(), 0.5_f64.to_bits());
        assert_eq!(params.alpha_f.to_bits(), 0.5_f64.to_bits());
        assert_eq!(params.beta.to_bits(), 0.25_f64.to_bits());
        assert_eq!(params.gamma.to_bits(), 0.5_f64.to_bits());
        assert!(GeneralizedAlphaParameters::from_spectral_radius(-0.1).is_err());
        assert!(GeneralizedAlphaParameters::from_spectral_radius(1.1).is_err());
    }

    #[test]
    fn generalized_alpha_with_zero_weights_matches_newmark_step() {
        let system = scalar_system();
        let newmark = NewmarkIntegrator::new(
            system.clone(),
            0.02,
            NewmarkParameters::AVERAGE_ACCELERATION,
        )
        .unwrap();
        let generalized_alpha = GeneralizedAlphaIntegrator::new(
            system,
            0.02,
            GeneralizedAlphaParameters {
                rho_inf: 1.0,
                alpha_m: 0.0,
                alpha_f: 0.0,
                beta: 0.25,
                gamma: 0.5,
            },
        )
        .unwrap();
        let state = ModalTransientState::new(vec![0.4], vec![-0.2], vec![0.8], 1).unwrap();
        let force_next = [1.7];

        let newmark_next = newmark.step(&state, &force_next).unwrap();
        let generalized_alpha_next = generalized_alpha
            .step(&state, &[999.0], &force_next)
            .unwrap();

        assert!((newmark_next.q[0] - generalized_alpha_next.q[0]).abs() < 1.0e-14);
        assert!((newmark_next.q_dot[0] - generalized_alpha_next.q_dot[0]).abs() < 1.0e-14);
        assert!((newmark_next.q_ddot[0] - generalized_alpha_next.q_ddot[0]).abs() < 1.0e-14);
    }

    #[test]
    fn generalized_alpha_step_with_systems_matches_constant_path_when_unchanged() {
        let system = scalar_system();
        let integrator = GeneralizedAlphaIntegrator::new(
            system.clone(),
            0.02,
            GeneralizedAlphaParameters::from_spectral_radius(0.5).unwrap(),
        )
        .unwrap();
        let state = ModalTransientState::new(vec![0.4], vec![-0.2], vec![0.8], 1).unwrap();
        let force_current = [1.2];
        let force_next = [1.7];

        let variable_next = integrator
            .step_with_systems(&system, &system, &state, &force_current, &force_next)
            .unwrap();
        let constant_next = integrator
            .step(&state, &force_current, &force_next)
            .unwrap();

        assert_eq!(variable_next, constant_next);
    }

    #[test]
    fn generalized_alpha_recovers_quadratic_mms() {
        let integrator = GeneralizedAlphaIntegrator::new(
            scalar_system(),
            0.01,
            GeneralizedAlphaParameters::from_spectral_radius(0.5).unwrap(),
        )
        .unwrap();
        let q0 = 0.2;
        let v0 = -0.3;
        let acceleration = 0.7;
        let mut state =
            ModalTransientState::new(vec![q0], vec![v0], vec![acceleration], 1).unwrap();
        let mut max_error = 0.0_f64;

        for step in 1..=100 {
            let t_current = f64::from(step - 1) * integrator.dt_s();
            let t_next = f64::from(step) * integrator.dt_s();
            let q_current = q0 + v0 * t_current + 0.5 * acceleration * t_current * t_current;
            let v_current = v0 + acceleration * t_current;
            let q_next = q0 + v0 * t_next + 0.5 * acceleration * t_next * t_next;
            let v_next = v0 + acceleration * t_next;
            let force_current = 2.0 * acceleration + 3.0 * v_current + 4.0 * q_current;
            let force_next = 2.0 * acceleration + 3.0 * v_next + 4.0 * q_next;
            state = integrator
                .step(&state, &[force_current], &[force_next])
                .unwrap();
            max_error = max_error.max((state.q[0] - q_next).abs());
            max_error = max_error.max((state.q_dot[0] - v_next).abs());
            max_error = max_error.max((state.q_ddot[0] - acceleration).abs());
        }

        assert!(
            max_error < 1.0e-10,
            "quadratic manufactured solution drifted by {max_error}"
        );
    }

    #[test]
    fn newmark_rejects_bad_parameters_and_non_spd_effective_matrix() {
        assert!(
            NewmarkIntegrator::new(
                scalar_system(),
                0.01,
                NewmarkParameters {
                    beta: 0.0,
                    gamma: 0.5,
                },
            )
            .is_err()
        );
        assert!(
            NewmarkIntegrator::new(
                scalar_system(),
                0.0,
                NewmarkParameters::AVERAGE_ACCELERATION
            )
            .is_err()
        );

        let bad_system = ModalTransientSystem::new(
            DenseMatrix::from_row_major(1, 1, vec![1.0]).unwrap(),
            DenseMatrix::from_row_major(1, 1, vec![0.0]).unwrap(),
            DenseMatrix::from_row_major(1, 1, vec![-1.0e8]).unwrap(),
        )
        .unwrap();
        assert!(
            NewmarkIntegrator::new(bad_system, 0.01, NewmarkParameters::AVERAGE_ACCELERATION)
                .is_err()
        );
    }

    #[test]
    fn generalized_alpha_rejects_bad_parameters_and_non_spd_effective_matrix() {
        assert!(
            GeneralizedAlphaIntegrator::new(
                scalar_system(),
                0.01,
                GeneralizedAlphaParameters {
                    rho_inf: 1.0,
                    alpha_m: 0.0,
                    alpha_f: 0.0,
                    beta: 0.0,
                    gamma: 0.5,
                },
            )
            .is_err()
        );
        assert!(
            GeneralizedAlphaIntegrator::new(
                scalar_system(),
                0.0,
                GeneralizedAlphaParameters::from_spectral_radius(0.5).unwrap(),
            )
            .is_err()
        );

        let bad_system = ModalTransientSystem::new(
            DenseMatrix::from_row_major(1, 1, vec![1.0]).unwrap(),
            DenseMatrix::from_row_major(1, 1, vec![0.0]).unwrap(),
            DenseMatrix::from_row_major(1, 1, vec![-1.0e8]).unwrap(),
        )
        .unwrap();
        assert!(
            GeneralizedAlphaIntegrator::new(
                bad_system,
                0.01,
                GeneralizedAlphaParameters::from_spectral_radius(0.5).unwrap(),
            )
            .is_err()
        );
    }
}
