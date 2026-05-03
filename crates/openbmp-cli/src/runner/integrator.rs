//! Phase-5.D.4 runner-side integrator dispatch.
//!
//! The kernel ([`openbmp_sim::SimulationKernel`]) is generic over the
//! `Integrator<S>` type, which means the integrator selection
//! propagates into the kernel's concrete type. To let the runner pick
//! between [`Rk4FixedStep`], [`Dopri54FixedStep`], and
//! [`Dopri54Adaptive`] based on the scenario's `[solver]` block
//! without duplicating the entire run-loop body per integrator
//! variant, we wrap the three concretes in a single enum that itself
//! implements `Integrator<S>` and delegates to the active variant.
//!
//! The dispatch overhead is one match arm per [`Integrator::advance`]
//! call. The compiler inlines the per-variant body, so the IEEE 754
//! arithmetic in each branch is identical to the standalone integrator.
//!
//! Honest scope (Phase 5.D.4):
//!
//! - The `Adaptive` variant is wired only on the
//!   [`crate::runner::phase2_point_mass`] runner. The rigid-body
//!   runner continues to hardcode [`Rk4FixedStep`] until § 5.D.5
//!   ships.
//! - The DOPRI8(7) trajectory method named in the `[solver]` block
//!   parses but the runner rejects it with `UnsupportedScenario`.

use openbmp_scenario::{ScenarioDocument, SolverConfig};
use openbmp_sim::{
    AdaptiveIntegratorError, Dopri54Adaptive, Dopri54FixedStep, Integrator, IntegratorDeterminism,
    IntegratorError, ModelEvalError, Rk4FixedStep, SimState,
};

use crate::error::CliError;

/// Runner-side integrator selector. Implements [`Integrator<S>`] by
/// dispatching to the wrapped variant on every step.
pub enum RuntimeIntegrator {
    /// Default: classical RK4 fixed-step.
    Rk4(Rk4FixedStep),
    /// Dormand-Prince 5(4) fixed-step (5th-order solution).
    Dopri54Fixed(Dopri54FixedStep),
    /// Dormand-Prince 5(4) adaptive with PI step controller.
    Dopri54Adaptive(Box<Dopri54Adaptive>),
}

impl std::fmt::Debug for RuntimeIntegrator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Rk4(_) => f.write_str("RuntimeIntegrator::Rk4"),
            Self::Dopri54Fixed(_) => f.write_str("RuntimeIntegrator::Dopri54Fixed"),
            Self::Dopri54Adaptive(_) => f.write_str("RuntimeIntegrator::Dopri54Adaptive"),
        }
    }
}

impl<S: SimState> Integrator<S> for RuntimeIntegrator {
    fn determinism(&self) -> IntegratorDeterminism {
        match self {
            Self::Rk4(i) => <Rk4FixedStep as Integrator<S>>::determinism(i),
            Self::Dopri54Fixed(i) => <Dopri54FixedStep as Integrator<S>>::determinism(i),
            Self::Dopri54Adaptive(i) => <Dopri54Adaptive as Integrator<S>>::determinism(i),
        }
    }

    fn advance<F>(
        &self,
        state: &S,
        derive_fn: F,
        dt: openbmp_core::Duration,
    ) -> Result<S, IntegratorError>
    where
        F: Fn(&S, openbmp_core::SimTime) -> Result<S::Derivative, ModelEvalError>,
    {
        match self {
            Self::Rk4(i) => i.advance(state, derive_fn, dt),
            Self::Dopri54Fixed(i) => i.advance(state, derive_fn, dt),
            Self::Dopri54Adaptive(i) => i.advance(state, derive_fn, dt),
        }
    }
}

/// Build a [`RuntimeIntegrator`] from the scenario's `[solver]`
/// block. When the block is absent, defaults to
/// [`RuntimeIntegrator::Rk4`] — preserving the byte-stable Phase-1
/// contract for every existing scenario.
///
/// # Errors
///
/// Returns [`CliError::UnsupportedScenario`] when the
/// `(profile, trajectory_method, determinism)` combination names a
/// solver that has not been wired in the runner yet (e.g.,
/// `dopri853`, `rkf78`, `implicit-source-term`,
/// `partitioned-hypersonic`). The scenario validator already enforces
/// `bit-stable + adaptive-explicit` is rejected at parse time, so the
/// runner only handles the cross-product of valid wired combos.
pub fn build_runtime_integrator(
    document: &ScenarioDocument,
) -> Result<RuntimeIntegrator, CliError> {
    build_runtime_integrator_from_solver(document.solver.as_ref())
}

/// Inner dispatch: maps an `Option<&SolverConfig>` to a
/// [`RuntimeIntegrator`]. Factored out of
/// [`build_runtime_integrator`] so unit tests can exercise the
/// `(profile, method, determinism)` cross-product without
/// constructing a full [`ScenarioDocument`].
fn build_runtime_integrator_from_solver(
    solver: Option<&SolverConfig>,
) -> Result<RuntimeIntegrator, CliError> {
    let Some(solver) = solver else {
        return Ok(RuntimeIntegrator::Rk4(Rk4FixedStep));
    };
    let profile = solver.profile.as_deref().unwrap_or("fixed-step-explicit");
    let method = solver.trajectory_method.as_deref().unwrap_or("rk4");
    let determinism = solver.determinism.as_deref().unwrap_or("bit-stable");

    match (profile, method, determinism) {
        ("fixed-step-explicit", "rk4", "bit-stable") => Ok(RuntimeIntegrator::Rk4(Rk4FixedStep)),
        ("fixed-step-explicit", "dopri54", "bit-stable") => {
            Ok(RuntimeIntegrator::Dopri54Fixed(Dopri54FixedStep))
        }
        ("adaptive-explicit", "dopri54", "state-stable") => {
            let adaptive = solver.adaptive.as_ref().ok_or_else(|| {
                // Defensive: scenario validator already enforces
                // [solver.adaptive] is present for adaptive-explicit.
                CliError::UnsupportedScenario {
                    what: "[solver.adaptive] block missing for adaptive-explicit profile"
                        .to_owned(),
                }
            })?;
            let integrator = Dopri54Adaptive::new(
                adaptive.atol,
                adaptive.rtol,
                adaptive.min_dt_s,
                adaptive.max_dt_s,
            )
            .map_err(|e: AdaptiveIntegratorError| CliError::UnsupportedScenario {
                what: format!("[solver.adaptive] params rejected by Dopri54Adaptive: {e:?}"),
            })?;
            Ok(RuntimeIntegrator::Dopri54Adaptive(Box::new(integrator)))
        }
        ("fixed-step-explicit", method, _) if method == "dopri853" || method == "rkf78" => {
            Err(CliError::UnsupportedScenario {
                what: format!(
                    "solver.trajectory_method = {method:?} parses but is not wired \
                     in the runner; deferred to a future slice"
                ),
            })
        }
        ("adaptive-explicit", method, _) if method != "dopri54" => {
            Err(CliError::UnsupportedScenario {
                what: format!(
                    "adaptive-explicit + {method:?} parses but is not wired in the \
                     runner; only dopri54 is wired (Phase 5.D.4). DOPRI8(7) is \
                     deferred to § 5.D.5"
                ),
            })
        }
        ("implicit-source-term" | "partitioned-hypersonic", _, _) => {
            Err(CliError::UnsupportedScenario {
                what: format!("solver.profile = {profile:?} is parser-only; not wired"),
            })
        }
        _ => Err(CliError::UnsupportedScenario {
            what: format!(
                "unsupported (profile, method, determinism) combination: \
                 ({profile:?}, {method:?}, {determinism:?})"
            ),
        }),
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use openbmp_scenario::{AdaptiveSolverConfig, SolverConfig};

    use super::*;

    fn solver(
        profile: &str,
        method: &str,
        determinism: &str,
        adaptive: Option<AdaptiveSolverConfig>,
    ) -> SolverConfig {
        SolverConfig {
            profile: Some(profile.to_owned()),
            trajectory_method: Some(method.to_owned()),
            determinism: Some(determinism.to_owned()),
            adaptive,
            source_terms: None,
        }
    }

    fn well_formed_adaptive() -> AdaptiveSolverConfig {
        AdaptiveSolverConfig {
            rtol: 1.0e-9,
            atol: 1.0e-12,
            min_dt_s: 1.0e-6,
            max_dt_s: 1.0,
            dense_output: false,
        }
    }

    #[test]
    fn missing_solver_block_defaults_to_rk4() {
        let result = build_runtime_integrator_from_solver(None).unwrap();
        assert!(matches!(result, RuntimeIntegrator::Rk4(_)));
    }

    #[test]
    fn fixed_step_rk4_bit_stable_selects_rk4() {
        let s = solver("fixed-step-explicit", "rk4", "bit-stable", None);
        let result = build_runtime_integrator_from_solver(Some(&s)).unwrap();
        assert!(matches!(result, RuntimeIntegrator::Rk4(_)));
    }

    #[test]
    fn fixed_step_dopri54_bit_stable_selects_dopri54_fixed() {
        let s = solver("fixed-step-explicit", "dopri54", "bit-stable", None);
        let result = build_runtime_integrator_from_solver(Some(&s)).unwrap();
        assert!(matches!(result, RuntimeIntegrator::Dopri54Fixed(_)));
    }

    #[test]
    fn adaptive_explicit_dopri54_selects_adaptive_with_state_stable_determinism() {
        let s = solver(
            "adaptive-explicit",
            "dopri54",
            "state-stable",
            Some(well_formed_adaptive()),
        );
        let result = build_runtime_integrator_from_solver(Some(&s)).unwrap();
        let RuntimeIntegrator::Dopri54Adaptive(_) = &result else {
            panic!("expected Dopri54Adaptive, got {result:?}");
        };
        assert_eq!(
            <RuntimeIntegrator as Integrator<openbmp_state::PointMassState>>::determinism(&result),
            IntegratorDeterminism::StateStable
        );
    }

    #[test]
    fn fixed_step_dopri853_is_rejected_as_unwired() {
        let s = solver("fixed-step-explicit", "dopri853", "bit-stable", None);
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let CliError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(
            what.contains("dopri853"),
            "error message should name dopri853: {what}"
        );
    }

    #[test]
    fn fixed_step_rkf78_is_rejected_as_unwired() {
        let s = solver("fixed-step-explicit", "rkf78", "bit-stable", None);
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let CliError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(
            what.contains("rkf78"),
            "error message should name rkf78: {what}"
        );
    }

    #[test]
    fn adaptive_explicit_dopri853_is_rejected_with_dopri54_only_message() {
        let s = solver(
            "adaptive-explicit",
            "dopri853",
            "state-stable",
            Some(well_formed_adaptive()),
        );
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let CliError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(
            what.contains("dopri54 is wired"),
            "error message should explain only dopri54 is wired: {what}"
        );
    }

    #[test]
    fn implicit_source_term_profile_is_rejected_as_parser_only() {
        let s = solver("implicit-source-term", "rk4", "state-stable", None);
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let CliError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(
            what.contains("implicit-source-term"),
            "error message should name implicit-source-term: {what}"
        );
        assert!(
            what.contains("parser-only"),
            "error message should explain parser-only: {what}"
        );
    }

    #[test]
    fn partitioned_hypersonic_profile_is_rejected_as_parser_only() {
        let s = solver("partitioned-hypersonic", "rk4", "state-stable", None);
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let CliError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(
            what.contains("partitioned-hypersonic"),
            "error message should name partitioned-hypersonic: {what}"
        );
    }

    #[test]
    fn adaptive_explicit_without_adaptive_block_returns_defensive_error() {
        let s = solver("adaptive-explicit", "dopri54", "state-stable", None);
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let CliError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(
            what.contains("[solver.adaptive] block missing"),
            "error should call out missing adaptive block: {what}"
        );
    }
}
