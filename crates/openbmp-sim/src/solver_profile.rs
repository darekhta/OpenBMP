//! Phase 6.0 — Hypersonic solver profile.
//!
//! Wraps the existing integrator family ([`Rk4FixedStep`],
//! [`Dopri54FixedStep`], [`Dopri54Adaptive`], [`Dopri853FixedStep`],
//! [`Dopri853Adaptive`]) plus an implicit source-term sub-stepper
//! for stiff chemistry / material response into a single
//! profile-aware selector that scenarios declare and that becomes
//! part of the determinism hash.
//!
//! The profile is intentionally separate from the [`Integrator`]
//! trait — the kernel still drives [`Integrator`] instances directly
//! for the rigid-body trajectory, but a hypersonic scenario carries
//! the profile through the telemetry header so a reproduced run
//! pins the same `(method, dt | tolerance)` selection.
//!
//! Implicit source-term integration is intended for nonequilibrium
//! thermochemistry (Park-2T sub-stepping inside a single rigid-body
//! kernel step) and 1-D thermal-response (`OneDThermalToy` interior
//! conduction). Both use cases land in Phases 6.10 / 6.9. The
//! [`implicit_euler_step`] primitive is a fixed-point iteration that
//! solves `y_{n+1} = y_n + h * f(t_{n+1}, y_{n+1})` to a declared
//! tolerance and maximum iteration count; both bounds are scenario-
//! declared and part of the determinism profile.

use openbmp_core::Duration;

use crate::integrator::IntegratorDeterminism;

/// Phase 6.0 solver profile declared by a hypersonic scenario.
///
/// Selects the trajectory integrator and (optionally) the implicit
/// sub-stepper used by chemistry / material-response source terms.
/// The full profile is part of the scenario hash and is recorded in
/// the telemetry header.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SolverProfile {
    /// Fixed-step explicit Runge-Kutta. Used for byte-stable golden
    /// tests and deterministic replay. `dt` must be strictly positive
    /// and finite.
    FixedStepExplicit {
        /// Explicit-RK method to run.
        method: ExplicitMethod,
        /// Fixed integration step size.
        dt: Duration,
    },

    /// Adaptive explicit Runge-Kutta with embedded error estimate.
    /// Used for smooth trajectories and event-gradient accuracy.
    AdaptiveExplicit {
        /// Adaptive method (must be one of the embedded pairs:
        /// `DormandPrince54`, `DormandPrince853`, `RungeKuttaFehlberg78`).
        method: ExplicitMethod,
        /// Relative tolerance per component.
        rtol: f64,
        /// Absolute tolerance per component.
        atol: f64,
        /// Lower bound on `dt`. The PI step controller refuses to go
        /// below this value (fails closed).
        min_dt: Duration,
        /// Upper bound on `dt`. The PI step controller clamps to this
        /// value when the error estimate would otherwise grow the step
        /// further.
        max_dt: Duration,
    },

    /// Stiff source-term integration for chemistry / thermal response.
    /// Used as a sub-stepper inside a trajectory step or by a research
    /// profile that needs implicit stability on a small slow manifold
    /// (e.g. Park-2T species evolution behind a normal shock).
    ImplicitSourceTerm {
        /// Implicit method to run.
        method: ImplicitMethod,
        /// Fixed sub-step count per trajectory step. Part of the
        /// determinism profile.
        substeps: usize,
        /// Nonlinear iteration tolerance (fixed-point or Newton residual).
        nonlinear_tolerance: f64,
        /// Maximum nonlinear iterations per sub-step. A scenario that
        /// declares a tolerance unreachable in this many iterations
        /// fails closed via [`ImplicitSolveError::DidNotConverge`].
        nonlinear_max_iter: usize,
    },
}

/// Explicit Runge-Kutta methods exposed by the profile.
///
/// Mapping to existing integrators in this crate:
///
/// | Variant                   | Implementation         | Use         |
/// |---------------------------|------------------------|-------------|
/// | `Rk4`                     | `Rk4FixedStep`         | byte-stable |
/// | `DormandPrince54`         | `Dopri54FixedStep` / `Dopri54Adaptive` | adaptive smooth |
/// | `DormandPrince853`        | `Dopri853FixedStep` / `Dopri853Adaptive` | high-order smooth |
/// | `RungeKuttaFehlberg78`    | reserved (deferred)    | adaptive high-order |
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ExplicitMethod {
    /// 4th-order, 4-stage, classical Runge-Kutta.
    Rk4,
    /// Dormand-Prince 5(4) embedded pair (7 stages, FSAL).
    DormandPrince54,
    /// Dormand-Prince 8(5,3) embedded pair (13 stages).
    DormandPrince853,
    /// Runge-Kutta-Fehlberg 7(8) embedded pair. Reserved for a
    /// follow-on slice; declaring this variant in a scenario is
    /// rejected by the profile selector until the integrator lands.
    RungeKuttaFehlberg78,
}

impl ExplicitMethod {
    /// Determinism class of the underlying integrator.
    ///
    /// Fixed-step methods are bit-stable. Adaptive methods are
    /// state-stable: the PI step controller calls `pow()` / `log()`
    /// which round per the platform libm, so cross-platform byte
    /// equality is not guaranteed (per-platform replay still matches).
    #[must_use]
    pub const fn fixed_step_determinism(self) -> IntegratorDeterminism {
        match self {
            Self::Rk4 | Self::DormandPrince54 | Self::DormandPrince853 => {
                IntegratorDeterminism::BitStable
            }
            // RKF78 reserved.
            Self::RungeKuttaFehlberg78 => IntegratorDeterminism::BitStable,
        }
    }
}

/// Implicit methods exposed by the [`SolverProfile::ImplicitSourceTerm`]
/// variant.
///
/// `ImplicitEuler` is the 6.0 deliverable; `RosenbrockWanner` and
/// `Bdf` are scheduled for the 6.10 stiff-chemistry path and stand
/// here as the type surface ahead of their solver implementations.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ImplicitMethod {
    /// First-order A-stable implicit Euler (backward Euler).
    /// Bit-stable when paired with a fixed-iteration solver and a
    /// fixed maximum-iteration cap.
    ImplicitEuler,
    /// Rosenbrock-Wanner linearised stage method (placeholder type;
    /// solver lands in 6.10).
    RosenbrockWanner,
    /// Backward-differentiation formula multi-step method (placeholder
    /// type; solver lands in 6.10).
    Bdf,
}

/// Coupling-edge declarations for the partitioned hypersonic stack.
///
/// The architecture chain
/// `environment -> aero -> aerothermal -> material_response -> mass/geometry -> dynamics`
/// is partitioned and explicit: each edge declares whether feedback
/// is disabled, lagged one kernel step, sub-iterated to a fixed
/// count, or solved with a profile-gated implicit coupling method.
/// The research-safe default is one-step lagged feedback.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub enum CouplingEdge {
    /// No feedback from the downstream model to the upstream model
    /// at this edge. Used when the upstream model is independent of
    /// the downstream state (e.g. environment → aero, when the aero
    /// only consumes the freestream sample and never writes back).
    Disabled,
    /// Downstream state is read using its value from the previous
    /// kernel step. Default research-safe path; telemetry reports
    /// the one-step lag.
    #[default]
    LaggedOneStep,
    /// Downstream state is sub-iterated to a fixed count per
    /// trajectory step. Used when the coupled response is significant
    /// enough to warrant inner iteration but stiff enough to avoid
    /// implicit coupling.
    SubIterated {
        /// Fixed inner-iteration count. Part of the determinism
        /// profile.
        iterations: usize,
    },
    /// Profile-gated implicit coupling: the edge resolves via an
    /// implicit sub-stepper declared on the [`SolverProfile`]. Reserved
    /// for the 6.10 chemistry / 6.11 ablation stages; rejected at
    /// scenario load when the profile does not carry an implicit
    /// sub-stepper.
    ImplicitProfile,
}

/// Profile-construction error returned by [`SolverProfile::validate`].
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SolverProfileError {
    /// Fixed-step `dt` was not strictly positive and finite.
    #[error("solver profile fixed-step dt must be strictly positive and finite; got {dt_seconds}")]
    InvalidFixedStep {
        /// The offending step size in seconds.
        dt_seconds: f64,
    },
    /// Adaptive bounds were not consistent: `min_dt <= max_dt`, both
    /// strictly positive and finite, and tolerances strictly positive.
    #[error("solver profile adaptive bounds invalid: {reason}")]
    InvalidAdaptiveBounds {
        /// Short reason.
        reason: &'static str,
    },
    /// Implicit sub-step count was zero, tolerance was non-positive,
    /// or `max_iter` was zero.
    #[error("solver profile implicit parameters invalid: {reason}")]
    InvalidImplicitParameters {
        /// Short reason.
        reason: &'static str,
    },
    /// A reserved variant ([`ExplicitMethod::RungeKuttaFehlberg78`])
    /// was selected before its solver landed.
    #[error("solver profile method is reserved and not yet implemented")]
    ReservedMethod,
}

/// Solver error returned by the implicit-Euler sub-stepper.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ImplicitSolveError {
    /// The nonlinear iteration did not converge within `max_iter`
    /// iterations at the declared tolerance.
    #[error("implicit-Euler did not converge: residual {residual:e}, max_iter {max_iter}")]
    DidNotConverge {
        /// Final residual at exit (max-norm of the per-component
        /// fixed-point gap).
        residual: f64,
        /// Maximum iteration count declared on the profile.
        max_iter: usize,
    },
    /// The right-hand side produced a non-finite value during
    /// iteration.
    #[error("implicit-Euler right-hand side produced non-finite value")]
    NonFiniteRhs,
}

impl SolverProfile {
    /// Validate the profile's internal parameters at scenario load.
    ///
    /// Returns `Ok(())` for any well-formed profile. The validator
    /// is intentionally cheap — it does not run any integration step.
    ///
    /// # Errors
    ///
    /// See [`SolverProfileError`] for the rejection cases.
    pub fn validate(&self) -> Result<(), SolverProfileError> {
        match self {
            Self::FixedStepExplicit { method, dt } => {
                if matches!(method, ExplicitMethod::RungeKuttaFehlberg78) {
                    return Err(SolverProfileError::ReservedMethod);
                }
                let dt_s = dt.as_seconds();
                if !dt_s.is_finite() || dt_s <= 0.0 {
                    return Err(SolverProfileError::InvalidFixedStep { dt_seconds: dt_s });
                }
                Ok(())
            }
            Self::AdaptiveExplicit {
                method,
                rtol,
                atol,
                min_dt,
                max_dt,
            } => {
                if matches!(method, ExplicitMethod::Rk4) {
                    return Err(SolverProfileError::InvalidAdaptiveBounds {
                        reason: "RK4 has no embedded error estimate; use a Dormand-Prince pair",
                    });
                }
                if matches!(method, ExplicitMethod::RungeKuttaFehlberg78) {
                    return Err(SolverProfileError::ReservedMethod);
                }
                let (min_s, max_s) = (min_dt.as_seconds(), max_dt.as_seconds());
                if !rtol.is_finite() || *rtol <= 0.0 {
                    return Err(SolverProfileError::InvalidAdaptiveBounds {
                        reason: "rtol must be strictly positive and finite",
                    });
                }
                if !atol.is_finite() || *atol <= 0.0 {
                    return Err(SolverProfileError::InvalidAdaptiveBounds {
                        reason: "atol must be strictly positive and finite",
                    });
                }
                if !min_s.is_finite() || min_s <= 0.0 {
                    return Err(SolverProfileError::InvalidAdaptiveBounds {
                        reason: "min_dt must be strictly positive and finite",
                    });
                }
                if !max_s.is_finite() || max_s < min_s {
                    return Err(SolverProfileError::InvalidAdaptiveBounds {
                        reason: "max_dt must be finite and ≥ min_dt",
                    });
                }
                Ok(())
            }
            Self::ImplicitSourceTerm {
                substeps,
                nonlinear_tolerance,
                nonlinear_max_iter,
                method: _,
            } => {
                if *substeps == 0 {
                    return Err(SolverProfileError::InvalidImplicitParameters {
                        reason: "substeps must be ≥ 1",
                    });
                }
                if !nonlinear_tolerance.is_finite() || *nonlinear_tolerance <= 0.0 {
                    return Err(SolverProfileError::InvalidImplicitParameters {
                        reason: "nonlinear tolerance must be strictly positive and finite",
                    });
                }
                if *nonlinear_max_iter == 0 {
                    return Err(SolverProfileError::InvalidImplicitParameters {
                        reason: "nonlinear max_iter must be ≥ 1",
                    });
                }
                Ok(())
            }
        }
    }

    /// Determinism class implied by the profile's selected method.
    ///
    /// Fixed-step profiles inherit the integrator's bit-stable class.
    /// Adaptive profiles are state-stable. Implicit profiles are
    /// bit-stable when paired with `ImplicitEuler` and a fixed-cap
    /// iteration solver (`Rosenbrock-Wanner` / `Bdf` placeholders
    /// inherit the same class once their solvers land).
    #[must_use]
    pub const fn determinism(&self) -> IntegratorDeterminism {
        match self {
            Self::FixedStepExplicit { .. } | Self::ImplicitSourceTerm { .. } => {
                IntegratorDeterminism::BitStable
            }
            Self::AdaptiveExplicit { .. } => IntegratorDeterminism::StateStable,
        }
    }
}

/// One scalar implicit-Euler sub-step with Newton iteration:
/// solves `G(y) = y - y_n - h * f(t_{n+1}, y) = 0` using Newton's
/// method with the supplied analytic slope `df/dy`.
///
/// The Newton step is
///
/// ```text
/// y_{k+1} = y_k - (y_k - y_n - h f(t,y_k)) / (1 - h f_y(t,y_k))
/// ```
///
/// For a scalar linear stiff RHS `f(t,y) = -k·y` (with
/// `df/dy = -k`), Newton converges in **one** iteration to the
/// analytic implicit-Euler answer `y_1 = y_n / (1 + k·h)` — that is
/// the stiff-stable property the profile needs for Phase-6.10
/// Park-2T vibrational relaxation and Phase-6.11 surface energy
/// balance.
///
/// The residual reported on `DidNotConverge` is the max-norm of the
/// final step `|y_{k+1} - y_k|`. The Jacobian denominator is clamped
/// away from zero with a fixed regularisation `eps_J = 1e-30` to
/// avoid division-by-zero in non-stiff regions; the regularisation
/// is small enough not to perturb the converged value at this
/// crate's working tolerances.
///
/// Designed for scalar-valued stiff source terms (per-mode
/// vibrational energy, per-species production rate after composition
/// projection, per-node thermal-conduction node value). Vector source
/// terms loop over components externally to keep the cross-platform
/// call graph bit-identical.
///
/// # Errors
///
/// Returns [`ImplicitSolveError::NonFiniteRhs`] if any call to `rhs`
/// or `dfdy` returns a non-finite value, or
/// [`ImplicitSolveError::DidNotConverge`] if Newton exceeds
/// `max_iter` without reaching `tol`.
pub fn implicit_euler_step<F, J>(
    y_n: f64,
    t_next_seconds: f64,
    h_seconds: f64,
    tol: f64,
    max_iter: usize,
    rhs: F,
    dfdy: J,
) -> Result<f64, ImplicitSolveError>
where
    F: Fn(f64, f64) -> f64,
    J: Fn(f64, f64) -> f64,
{
    const EPS_J: f64 = 1.0e-30;
    let mut y = y_n;
    for _ in 0..max_iter {
        let f_val = rhs(t_next_seconds, y);
        if !f_val.is_finite() {
            return Err(ImplicitSolveError::NonFiniteRhs);
        }
        let g = y - y_n - h_seconds * f_val;
        let dfdy_val = dfdy(t_next_seconds, y);
        if !dfdy_val.is_finite() {
            return Err(ImplicitSolveError::NonFiniteRhs);
        }
        let denom = 1.0 - h_seconds * dfdy_val;
        let denom = if denom.abs() < EPS_J {
            if denom.is_sign_negative() {
                -EPS_J
            } else {
                EPS_J
            }
        } else {
            denom
        };
        let delta = -g / denom;
        let y_next = y + delta;
        y = y_next;
        if delta.abs() <= tol {
            return Ok(y);
        }
    }
    let f_final = rhs(t_next_seconds, y);
    if !f_final.is_finite() {
        return Err(ImplicitSolveError::NonFiniteRhs);
    }
    Err(ImplicitSolveError::DidNotConverge {
        residual: (y - y_n - h_seconds * f_final).abs(),
        max_iter,
    })
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::missing_panics_doc,
    clippy::similar_names
)]
mod tests {
    use super::*;

    fn fixed(dt_s: f64, method: ExplicitMethod) -> SolverProfile {
        SolverProfile::FixedStepExplicit {
            method,
            dt: Duration::from_seconds(dt_s),
        }
    }

    #[test]
    fn fixed_step_rk4_validates() {
        let p = fixed(1e-3, ExplicitMethod::Rk4);
        p.validate().unwrap();
        assert_eq!(p.determinism(), IntegratorDeterminism::BitStable);
    }

    #[test]
    fn fixed_step_rejects_non_positive_dt() {
        assert!(matches!(
            fixed(0.0, ExplicitMethod::Rk4).validate(),
            Err(SolverProfileError::InvalidFixedStep { .. })
        ));
        assert!(matches!(
            fixed(-1.0e-3, ExplicitMethod::Rk4).validate(),
            Err(SolverProfileError::InvalidFixedStep { .. })
        ));
    }

    #[test]
    fn fixed_step_rejects_reserved_method() {
        let p = fixed(1e-3, ExplicitMethod::RungeKuttaFehlberg78);
        assert!(matches!(
            p.validate(),
            Err(SolverProfileError::ReservedMethod)
        ));
    }

    #[test]
    fn adaptive_dopri54_validates() {
        let p = SolverProfile::AdaptiveExplicit {
            method: ExplicitMethod::DormandPrince54,
            rtol: 1e-7,
            atol: 1e-9,
            min_dt: Duration::from_seconds(1e-6),
            max_dt: Duration::from_seconds(1e-2),
        };
        p.validate().unwrap();
        assert_eq!(p.determinism(), IntegratorDeterminism::StateStable);
    }

    #[test]
    fn adaptive_rejects_rk4() {
        let p = SolverProfile::AdaptiveExplicit {
            method: ExplicitMethod::Rk4,
            rtol: 1e-7,
            atol: 1e-9,
            min_dt: Duration::from_seconds(1e-6),
            max_dt: Duration::from_seconds(1e-2),
        };
        assert!(matches!(
            p.validate(),
            Err(SolverProfileError::InvalidAdaptiveBounds { .. })
        ));
    }

    #[test]
    fn adaptive_rejects_inverted_bounds() {
        let p = SolverProfile::AdaptiveExplicit {
            method: ExplicitMethod::DormandPrince54,
            rtol: 1e-7,
            atol: 1e-9,
            min_dt: Duration::from_seconds(1e-2),
            max_dt: Duration::from_seconds(1e-6),
        };
        assert!(matches!(
            p.validate(),
            Err(SolverProfileError::InvalidAdaptiveBounds { .. })
        ));
    }

    #[test]
    fn implicit_validates_well_formed() {
        let p = SolverProfile::ImplicitSourceTerm {
            method: ImplicitMethod::ImplicitEuler,
            substeps: 4,
            nonlinear_tolerance: 1e-9,
            nonlinear_max_iter: 32,
        };
        p.validate().unwrap();
        assert_eq!(p.determinism(), IntegratorDeterminism::BitStable);
    }

    #[test]
    fn implicit_rejects_zero_substeps() {
        let p = SolverProfile::ImplicitSourceTerm {
            method: ImplicitMethod::ImplicitEuler,
            substeps: 0,
            nonlinear_tolerance: 1e-9,
            nonlinear_max_iter: 32,
        };
        assert!(matches!(
            p.validate(),
            Err(SolverProfileError::InvalidImplicitParameters { .. })
        ));
    }

    #[test]
    fn coupling_edge_default_is_lagged_one_step() {
        assert_eq!(CouplingEdge::default(), CouplingEdge::LaggedOneStep);
    }

    /// Scalar test: integrate `dy/dt = -100 y` (stiff scalar decay)
    /// from `y(0) = 1` for `h = 0.01` using implicit Euler. The exact
    /// implicit-Euler solution is `y_1 = y_0 / (1 + 100 h) = 1 / 2`,
    /// and Newton converges in one iteration for the linear case.
    #[test]
    fn implicit_euler_scalar_stiff_decay() {
        let y = implicit_euler_step(1.0, 0.01, 0.01, 1e-12, 64, |_, y| -100.0 * y, |_, _| -100.0)
            .unwrap();
        assert!((y - 0.5).abs() < 1e-12, "got {y}");
    }

    /// Linear stiffness with `dy/dt = -k y` for various k. Implicit
    /// Euler stays stable for any h > 0 (A-stable) and Newton
    /// converges exactly in one iteration for the linear case.
    #[test]
    fn implicit_euler_handles_high_stiffness() {
        for k in [10.0_f64, 100.0, 1.0e4, 1.0e6] {
            let y = implicit_euler_step(1.0, 1.0e-3, 1.0e-3, 1e-12, 64, |_, y| -k * y, |_, _| -k)
                .unwrap();
            let analytic = 1.0 / (1.0 + k * 1.0e-3);
            let scale = analytic.max(1.0e-12);
            assert!(
                (y - analytic).abs() <= 1e-9 * scale,
                "k={k}, y={y}, expected≈{analytic}"
            );
        }
    }

    /// Convergence failure path: a `max_iter = 0` budget guarantees
    /// the iteration cannot complete a single Newton step and surfaces
    /// `ImplicitSolveError::DidNotConverge`.
    #[test]
    fn implicit_euler_reports_non_convergence() {
        let result =
            implicit_euler_step(1.0, 0.01, 0.01, 1e-12, 0, |_, y| -100.0 * y, |_, _| -100.0);
        assert!(
            matches!(result, Err(ImplicitSolveError::DidNotConverge { .. })),
            "got {result:?}"
        );
    }

    /// Non-finite rhs is rejected explicitly so the determinism
    /// profile doesn't quietly accept a NaN-poisoned source term.
    #[test]
    fn implicit_euler_rejects_nan_rhs() {
        let result = implicit_euler_step(1.0, 0.01, 0.01, 1e-12, 8, |_, _| f64::NAN, |_, _| 0.0);
        assert!(matches!(result, Err(ImplicitSolveError::NonFiniteRhs)));
    }

    /// Determinism class: implicit Euler at a fixed iteration cap is
    /// bit-stable when the rhs is pure arithmetic.
    #[test]
    fn implicit_euler_is_bit_stable_across_runs() {
        let a = implicit_euler_step(1.0, 0.01, 0.01, 1e-12, 64, |_, y| -100.0 * y, |_, _| -100.0)
            .unwrap();
        let b = implicit_euler_step(1.0, 0.01, 0.01, 1e-12, 64, |_, y| -100.0 * y, |_, _| -100.0)
            .unwrap();
        assert_eq!(a.to_bits(), b.to_bits());
    }

    /// Nonlinear test: `dy/dt = -y^3` from `y(0) = 1` with `h = 0.5`.
    /// Implicit Euler: `y - 1 + 0.5 y^3 = 0`. Newton must converge
    /// to the real root.
    #[test]
    fn implicit_euler_solves_nonlinear_cubic() {
        let y = implicit_euler_step(
            1.0,
            0.5,
            0.5,
            1e-12,
            64,
            |_, y| -y.powi(3),
            |_, y| -3.0 * y * y,
        )
        .unwrap();
        let residual = y - 1.0 + 0.5 * y.powi(3);
        assert!(residual.abs() < 1e-10, "y={y}, residual={residual}");
    }

    /// Non-stiff constant source term: Newton denominator is one, so
    /// the implicit step must match the explicit affine update exactly
    /// at normal tolerances. This guards against Jacobian
    /// regularisation leaking into ordinary smooth source terms.
    #[test]
    fn implicit_euler_constant_source_has_no_regularisation_bias() {
        let y = implicit_euler_step(2.0, 0.25, 0.25, 1e-12, 8, |_, _| 3.0, |_, _| 0.0).unwrap();
        assert!((y - 2.75).abs() < 1e-12, "got {y}");
    }
}
