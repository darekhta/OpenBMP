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
//! Still rejected as unwired: `rkf78`, Rosenbrock-Wanner, and BDF.
//! Phase 6.0 source-term profiles now dispatch the declared
//! trajectory method and pin the source-term profile; source-term
//! consumers can call the implicit-Euler primitive through their own
//! state adapters.

use openbmp_scenario::{ScenarioDocument, SolverConfig, SourceTermSolverConfig};
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
    /// Phase-6 source-term profile: delegates trajectory integration
    /// to the wrapped explicit/adaptive integrator, while recording
    /// the source-term sub-step controls as part of the runtime
    /// solver profile.
    ProfiledSourceTerm {
        /// Profile name (`implicit-source-term` or
        /// `partitioned-hypersonic`).
        profile: &'static str,
        /// Source-term controls.
        source_terms: SourceTermRuntimeProfile,
        /// Trajectory integrator.
        trajectory: Box<RuntimeIntegrator>,
    },
}

/// Runtime source-term profile accepted by the Phase-6 dispatcher.
#[derive(Clone, Debug, PartialEq)]
pub struct SourceTermRuntimeProfile {
    /// Chemistry substeps per trajectory step.
    pub chemistry_substeps: u32,
    /// Material substeps per trajectory step.
    pub material_substeps: u32,
    /// Nonlinear solve tolerance.
    pub nonlinear_tolerance: f64,
    /// Nonlinear iteration cap.
    pub nonlinear_max_iter: u32,
}

impl std::fmt::Debug for RuntimeIntegrator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Rk4(_) => f.write_str("RuntimeIntegrator::Rk4"),
            Self::Dopri54Fixed(_) => f.write_str("RuntimeIntegrator::Dopri54Fixed"),
            Self::Dopri54Adaptive(_) => f.write_str("RuntimeIntegrator::Dopri54Adaptive"),
            Self::Dopri853Fixed(_) => f.write_str("RuntimeIntegrator::Dopri853Fixed"),
            Self::Dopri853Adaptive(_) => f.write_str("RuntimeIntegrator::Dopri853Adaptive"),
            Self::ProfiledSourceTerm {
                profile,
                source_terms: _,
                trajectory,
            } => f
                .debug_struct("RuntimeIntegrator::ProfiledSourceTerm")
                .field("profile", profile)
                .field("trajectory", trajectory)
                .finish(),
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
            Self::ProfiledSourceTerm { .. } => IntegratorDeterminism::StateStable,
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
            Self::ProfiledSourceTerm { trajectory, .. } => trajectory.advance(state, derive_fn, dt),
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
        ("implicit-source-term" | "partitioned-hypersonic", "rk4" | "dopri54" | "dopri853", "state-stable") => {
            let source_terms = source_term_runtime_profile(solver.source_terms.as_ref())?;
            let trajectory = build_source_profile_trajectory(method)?;
            let profile = if profile == "implicit-source-term" {
                "implicit-source-term"
            } else {
                "partitioned-hypersonic"
            };
            Ok(RuntimeIntegrator::ProfiledSourceTerm {
                profile,
                source_terms,
                trajectory: Box::new(trajectory),
            })
        }
        ("implicit-source-term" | "partitioned-hypersonic", "rkf78", _) => Err(CliError::UnsupportedScenario {
            what: "source-term solver profiles do not support solver.trajectory_method = \"rkf78\"; \
                 wired methods are rk4, dopri54, and dopri853"
                .to_owned(),
        }),
        _ => Err(CliError::UnsupportedScenario {
            what: format!(
                "unsupported (profile, method, determinism) combination: \
                 ({profile:?}, {method:?}, {determinism:?})"
            ),
        }),
    }
}

fn build_source_profile_trajectory(method: &str) -> Result<RuntimeIntegrator, CliError> {
    match method {
        "rk4" => Ok(RuntimeIntegrator::Rk4(Rk4FixedStep)),
        "dopri54" => Ok(RuntimeIntegrator::Dopri54Fixed(Dopri54FixedStep)),
        "dopri853" => Ok(RuntimeIntegrator::Dopri853Fixed(Dopri853FixedStep)),
        _ => Err(CliError::UnsupportedScenario {
            what: format!("source-term profile trajectory method {method:?} is not wired"),
        }),
    }
}

fn source_term_runtime_profile(
    source_terms: Option<&SourceTermSolverConfig>,
) -> Result<SourceTermRuntimeProfile, CliError> {
    let source_terms = source_terms.ok_or_else(|| CliError::UnsupportedScenario {
        what: "[solver.source_terms] block missing for source-term solver profile".to_owned(),
    })?;
    if source_terms.chemistry_method != "implicit-euler" {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "solver.source_terms.chemistry_method = {:?} parses but is not wired; \
                 wired method is implicit-euler",
                source_terms.chemistry_method
            ),
        });
    }
    if source_terms.material_method != "implicit-euler" {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "solver.source_terms.material_method = {:?} parses but is not wired; \
                 wired method is implicit-euler",
                source_terms.material_method
            ),
        });
    }
    Ok(SourceTermRuntimeProfile {
        chemistry_substeps: source_terms.chemistry_substeps,
        material_substeps: source_terms.material_substeps,
        nonlinear_tolerance: source_terms.nonlinear_tolerance,
        nonlinear_max_iter: source_terms.nonlinear_max_iter,
    })
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

    fn solver_with_source_terms(
        profile: &str,
        method: &str,
        source_terms: SourceTermSolverConfig,
    ) -> SolverConfig {
        SolverConfig {
            profile: Some(profile.to_owned()),
            trajectory_method: Some(method.to_owned()),
            determinism: Some("state-stable".to_owned()),
            adaptive: None,
            source_terms: Some(source_terms),
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

    fn well_formed_source_terms() -> SourceTermSolverConfig {
        SourceTermSolverConfig {
            chemistry_method: "implicit-euler".to_owned(),
            chemistry_substeps: 4,
            material_method: "implicit-euler".to_owned(),
            material_substeps: 2,
            nonlinear_tolerance: 1.0e-10,
            nonlinear_max_iter: 12,
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
    fn implicit_source_term_profile_wraps_trajectory_integrator() {
        let s = solver_with_source_terms("implicit-source-term", "rk4", well_formed_source_terms());
        let result = build_runtime_integrator_from_solver(Some(&s)).unwrap();
        let RuntimeIntegrator::ProfiledSourceTerm {
            profile,
            source_terms,
            trajectory,
        } = result
        else {
            panic!("expected ProfiledSourceTerm");
        };
        assert_eq!(profile, "implicit-source-term");
        assert_eq!(source_terms.chemistry_substeps, 4);
        assert!(matches!(*trajectory, RuntimeIntegrator::Rk4(_)));
    }

    #[test]
    fn partitioned_hypersonic_profile_wraps_dopri853() {
        let s = solver_with_source_terms(
            "partitioned-hypersonic",
            "dopri853",
            well_formed_source_terms(),
        );
        let result = build_runtime_integrator_from_solver(Some(&s)).unwrap();
        let RuntimeIntegrator::ProfiledSourceTerm {
            profile,
            trajectory,
            ..
        } = result
        else {
            panic!("expected ProfiledSourceTerm");
        };
        assert_eq!(profile, "partitioned-hypersonic");
        assert!(matches!(*trajectory, RuntimeIntegrator::Dopri853Fixed(_)));
    }

    #[test]
    fn source_profile_rejects_unwired_chemistry_method() {
        let mut source_terms = well_formed_source_terms();
        source_terms.chemistry_method = "bdf".to_owned();
        let s = solver_with_source_terms("implicit-source-term", "rk4", source_terms);
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let CliError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(what.contains("chemistry_method") && what.contains("implicit-euler"));
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
                    let mut s = solver(profile, method, determinism, adaptive);
                    if matches!(profile, "implicit-source-term" | "partitioned-hypersonic") {
                        s.source_terms = Some(well_formed_source_terms());
                    }
                    let result = build_runtime_integrator_from_solver(Some(&s));
                    let should_accept = matches!(
                        (profile, method, determinism),
                        (
                            "fixed-step-explicit",
                            "rk4" | "dopri54" | "dopri853",
                            "bit-stable"
                        ) | ("adaptive-explicit", "dopri54" | "dopri853", "state-stable")
                            | (
                                "implicit-source-term" | "partitioned-hypersonic",
                                "rk4" | "dopri54" | "dopri853",
                                "state-stable"
                            )
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
