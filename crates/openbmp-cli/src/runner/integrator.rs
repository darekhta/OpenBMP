//! Phase-5.D.4 / 5.D.5 / 5.D.6 runner-side integrator dispatch.
//!
//! The kernel ([`openbmp_sim::SimulationKernel`]) is generic over the
//! `Integrator<S>` type, which means the integrator selection
//! propagates into the kernel's concrete type. To let the runner pick
//! between [`Rk4FixedStep`], [`Dopri54FixedStep`],
//! [`Dopri54Adaptive`], [`Dopri853FixedStep`], and
//! [`Dopri853Adaptive`] based on the scenario's `[solver]` block
//! without duplicating the entire run-loop body per integrator
//! variant, we wrap the five concretes in a single enum that itself
//! implements `Integrator<S>` and delegates to the active variant.
//!
//! The dispatch overhead is one match arm per [`Integrator::advance`]
//! call. The compiler inlines the per-variant body, so the IEEE 754
//! arithmetic in each branch is identical to the standalone integrator.
//!
//! Wired triples (positive selection):
//!
//! - `(fixed-step-explicit, rk4, bit-stable)` → [`Rk4FixedStep`]
//!   (the no-`[solver]` default — preserves the byte-stable Phase-1
//!   contract).
//! - `(fixed-step-explicit, dopri54, bit-stable)` →
//!   [`Dopri54FixedStep`] (§ 5.D.3).
//! - `(adaptive-explicit, dopri54, state-stable)` →
//!   [`Dopri54Adaptive`] (§ 5.D.4 — point-mass runner; § 5.D.5
//!   wires the rigid-body runner through the same enum dispatch).
//! - `(fixed-step-explicit, dopri853, bit-stable)` →
//!   [`Dopri853FixedStep`] (§ 5.D.6).
//! - `(adaptive-explicit, dopri853, state-stable)` →
//!   [`Dopri853Adaptive`] (§ 5.D.6).
//!
//! Still rejected as unwired: `rkf78`, `implicit-source-term`,
//! `partitioned-hypersonic`. Both runners (point-mass and rigid-body)
//! dispatch through this module today.

use openbmp_scenario::{ScenarioDocument, SolverConfig};
use openbmp_sim::{
    AdaptiveIntegratorError, Dopri54Adaptive, Dopri54FixedStep, Dopri853Adaptive,
    Dopri853FixedStep, Integrator, IntegratorDeterminism, IntegratorError, ModelEvalError,
    Rk4FixedStep, SimState,
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
    /// Dormand-Prince 8(5,3) (DOP853) fixed-step (8th-order solution).
    Dopri853Fixed(Dopri853FixedStep),
    /// DOP853 adaptive with err5/err3 stabilised error norm and
    /// I-controller.
    Dopri853Adaptive(Box<Dopri853Adaptive>),
}

impl std::fmt::Debug for RuntimeIntegrator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Rk4(_) => f.write_str("RuntimeIntegrator::Rk4"),
            Self::Dopri54Fixed(_) => f.write_str("RuntimeIntegrator::Dopri54Fixed"),
            Self::Dopri54Adaptive(_) => f.write_str("RuntimeIntegrator::Dopri54Adaptive"),
            Self::Dopri853Fixed(_) => f.write_str("RuntimeIntegrator::Dopri853Fixed"),
            Self::Dopri853Adaptive(_) => f.write_str("RuntimeIntegrator::Dopri853Adaptive"),
        }
    }
}

impl<S: SimState> Integrator<S> for RuntimeIntegrator {
    fn determinism(&self) -> IntegratorDeterminism {
        match self {
            Self::Rk4(i) => <Rk4FixedStep as Integrator<S>>::determinism(i),
            Self::Dopri54Fixed(i) => <Dopri54FixedStep as Integrator<S>>::determinism(i),
            Self::Dopri54Adaptive(i) => <Dopri54Adaptive as Integrator<S>>::determinism(i),
            Self::Dopri853Fixed(i) => <Dopri853FixedStep as Integrator<S>>::determinism(i),
            Self::Dopri853Adaptive(i) => <Dopri853Adaptive as Integrator<S>>::determinism(i),
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
            Self::Dopri853Fixed(i) => i.advance(state, derive_fn, dt),
            Self::Dopri853Adaptive(i) => i.advance(state, derive_fn, dt),
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
/// `rkf78`, `implicit-source-term`, `partitioned-hypersonic`). The
/// scenario validator already enforces
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
        ("fixed-step-explicit", "dopri853", "bit-stable") => {
            Ok(RuntimeIntegrator::Dopri853Fixed(Dopri853FixedStep))
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
        ("adaptive-explicit", "dopri853", "state-stable") => {
            let adaptive = solver.adaptive.as_ref().ok_or_else(|| {
                // Defensive: scenario validator already enforces
                // [solver.adaptive] is present for adaptive-explicit.
                CliError::UnsupportedScenario {
                    what: "[solver.adaptive] block missing for adaptive-explicit profile"
                        .to_owned(),
                }
            })?;
            let integrator = Dopri853Adaptive::new(
                adaptive.atol,
                adaptive.rtol,
                adaptive.min_dt_s,
                adaptive.max_dt_s,
            )
            .map_err(|e: AdaptiveIntegratorError| CliError::UnsupportedScenario {
                what: format!("[solver.adaptive] params rejected by Dopri853Adaptive: {e:?}"),
            })?;
            Ok(RuntimeIntegrator::Dopri853Adaptive(Box::new(integrator)))
        }
        ("fixed-step-explicit", "rkf78", _) => Err(CliError::UnsupportedScenario {
            what: "solver.trajectory_method = \"rkf78\" parses but is not wired in the runner; \
                 deferred to a future slice"
                .to_owned(),
        }),
        ("adaptive-explicit", method, _) if method != "dopri54" && method != "dopri853" => {
            Err(CliError::UnsupportedScenario {
                what: format!(
                    "adaptive-explicit + {method:?} parses but is not wired in the \
                     runner; wired methods are dopri54 (§ 5.D.4) and dopri853 (§ 5.D.6)"
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
    fn fixed_step_dopri853_bit_stable_selects_dopri853_fixed() {
        let s = solver("fixed-step-explicit", "dopri853", "bit-stable", None);
        let result = build_runtime_integrator_from_solver(Some(&s)).unwrap();
        let RuntimeIntegrator::Dopri853Fixed(_) = &result else {
            panic!("expected Dopri853Fixed, got {result:?}");
        };
        assert_eq!(
            <RuntimeIntegrator as Integrator<openbmp_state::PointMassState>>::determinism(&result),
            IntegratorDeterminism::BitStable
        );
    }

    #[test]
    fn adaptive_explicit_dopri853_selects_dopri853_adaptive() {
        let s = solver(
            "adaptive-explicit",
            "dopri853",
            "state-stable",
            Some(well_formed_adaptive()),
        );
        let result = build_runtime_integrator_from_solver(Some(&s)).unwrap();
        let RuntimeIntegrator::Dopri853Adaptive(_) = &result else {
            panic!("expected Dopri853Adaptive, got {result:?}");
        };
        assert_eq!(
            <RuntimeIntegrator as Integrator<openbmp_state::PointMassState>>::determinism(&result),
            IntegratorDeterminism::StateStable
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
    fn adaptive_explicit_rkf78_is_rejected_as_unwired() {
        let s = solver(
            "adaptive-explicit",
            "rkf78",
            "state-stable",
            Some(well_formed_adaptive()),
        );
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let CliError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(
            what.contains("rkf78") && what.contains("dopri853"),
            "error message should name rkf78 and reference wired dopri54/dopri853: {what}"
        );
    }

    #[test]
    fn adaptive_explicit_dopri853_with_missing_adaptive_block_returns_defensive_error() {
        let s = solver("adaptive-explicit", "dopri853", "state-stable", None);
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let CliError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(
            what.contains("[solver.adaptive] block missing"),
            "error should call out missing adaptive block: {what}"
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

    #[test]
    fn solver_cross_product_accepts_only_wired_triples() {
        let profiles = [
            "fixed-step-explicit",
            "adaptive-explicit",
            "implicit-source-term",
            "partitioned-hypersonic",
        ];
        let methods = ["rk4", "dopri54", "dopri853", "rkf78"];
        let determinisms = ["bit-stable", "state-stable"];

        for profile in profiles {
            for method in methods {
                for determinism in determinisms {
                    let adaptive = (profile == "adaptive-explicit").then(well_formed_adaptive);
                    let s = solver(profile, method, determinism, adaptive);
                    let result = build_runtime_integrator_from_solver(Some(&s));
                    let should_accept = matches!(
                        (profile, method, determinism),
                        (
                            "fixed-step-explicit",
                            "rk4" | "dopri54" | "dopri853",
                            "bit-stable"
                        ) | ("adaptive-explicit", "dopri54" | "dopri853", "state-stable")
                    );

                    assert_eq!(
                        result.is_ok(),
                        should_accept,
                        "unexpected dispatch result for ({profile:?}, {method:?}, {determinism:?}): {result:?}"
                    );
                }
            }
        }
    }
}
