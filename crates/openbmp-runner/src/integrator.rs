//! Runner-side integrator dispatch.
//!
//! The kernel ([`openbmp_sim::SimulationKernel`]) is generic over the
//! `Integrator<S>` type, which means integrator selection propagates
//! into the kernel's concrete type. The runner translates the
//! scenario's `[solver]` block into [`openbmp_sim::ProfiledIntegrator`],
//! the sim-owned enum that wraps [`openbmp_sim::Rk4FixedStep`],
//! [`openbmp_sim::Dopri54FixedStep`], [`openbmp_sim::Dopri54Adaptive`],
//! [`openbmp_sim::Dopri853FixedStep`], and
//! [`openbmp_sim::Dopri853Adaptive`] without duplicating the run loop.
//!
//! The dispatch overhead is one match arm per `Integrator::advance`
//! call. The compiler inlines the per-variant body, so the IEEE 754
//! arithmetic in each branch is identical to the standalone integrator.
//!
//! Wired triples (positive selection):
//!
//! - `(fixed-step-explicit, rk4, bit-stable)` →
//!   [`openbmp_sim::Rk4FixedStep`]
//!   (the no-`[solver]` default — preserves the byte-stable
//!   contract).
//! - `(fixed-step-explicit, dopri54, bit-stable)` →
//!   [`openbmp_sim::Dopri54FixedStep`].
//! - `(adaptive-explicit, dopri54, state-stable)` →
//!   [`openbmp_sim::Dopri54Adaptive`] (point-mass and rigid-body runners
//!   share the same enum dispatch).
//! - `(fixed-step-explicit, dopri853, bit-stable)` →
//!   [`openbmp_sim::Dopri853FixedStep`].
//! - `(adaptive-explicit, dopri853, state-stable)` →
//!   [`openbmp_sim::Dopri853Adaptive`].
//!
//! Still rejected as unwired: `rkf78`, Rosenbrock-Wanner, and BDF.
//! Source-term profiles dispatch the declared
//! trajectory method and pin the source-term profile; source-term
//! consumers can call the implicit-Euler primitive through their own
//! state adapters.

use openbmp_scenario::{ScenarioDocument, SolverConfig, SourceTermSolverConfig};
use openbmp_sim::{ExplicitMethod, SolverProfileError, SourceTermCouplingProfile};

use crate::error::RunnerError;

pub use openbmp_sim::ProfiledIntegrator as RuntimeIntegrator;
pub use openbmp_sim::SourceTermProfile as SourceTermRuntimeProfile;

/// Build a [`RuntimeIntegrator`] from the scenario's `[solver]`
/// block. When the block is absent, defaults to
/// [`RuntimeIntegrator::Rk4`] — preserving the byte-stable
/// contract for every existing scenario.
///
/// # Errors
///
/// Returns [`RunnerError::UnsupportedScenario`] when the
/// `(profile, trajectory_method, determinism)` combination names a
/// solver that has not been wired in the runner yet (e.g.,
/// `rkf78`, `implicit-source-term`, `partitioned-hypersonic`). The
/// scenario validator already enforces
/// `bit-stable + adaptive-explicit` is rejected at parse time, so the
/// runner only handles the cross-product of valid wired combos.
pub fn build_runtime_integrator(
    document: &ScenarioDocument,
) -> Result<RuntimeIntegrator, RunnerError> {
    build_runtime_integrator_from_solver(document.solver.as_ref())
}

/// Inner dispatch: maps an `Option<&SolverConfig>` to a
/// [`RuntimeIntegrator`]. Factored out of
/// [`build_runtime_integrator`] so unit tests can exercise the
/// `(profile, method, determinism)` cross-product without
/// constructing a full [`ScenarioDocument`].
fn build_runtime_integrator_from_solver(
    solver: Option<&SolverConfig>,
) -> Result<RuntimeIntegrator, RunnerError> {
    let Some(solver) = solver else {
        return RuntimeIntegrator::from_fixed_method(ExplicitMethod::Rk4)
            .map_err(|error| solver_profile_error(&error));
    };
    let profile = solver.profile.as_deref().unwrap_or("fixed-step-explicit");
    let method = solver.trajectory_method.as_deref().unwrap_or("rk4");
    let determinism = solver.determinism.as_deref().unwrap_or("bit-stable");
    if let Some(adaptive) = solver.adaptive.as_ref()
        && adaptive.dense_output
        && profile != "adaptive-explicit"
    {
        return Err(RunnerError::UnsupportedScenario {
            what: "solver.adaptive.dense_output = true requires \
                 solver.profile = \"adaptive-explicit\""
                .to_owned(),
        });
    }

    match (profile, method, determinism) {
        ("fixed-step-explicit", "rk4" | "dopri54" | "dopri853", "bit-stable") => {
            RuntimeIntegrator::from_fixed_method(explicit_method(method)?)
                .map_err(|error| solver_profile_error(&error))
        }
        ("adaptive-explicit", "dopri54", "state-stable") => {
            let adaptive = solver.adaptive.as_ref().ok_or_else(|| {
                // Defensive: scenario validator already enforces
                // [solver.adaptive] is present for adaptive-explicit.
                RunnerError::UnsupportedScenario {
                    what: "[solver.adaptive] block missing for adaptive-explicit profile"
                        .to_owned(),
                }
            })?;
            validate_dense_output_support(method, adaptive)?;
            RuntimeIntegrator::from_adaptive_method(
                ExplicitMethod::DormandPrince54,
                adaptive.rtol,
                adaptive.atol,
                openbmp_core::Duration::from_seconds(adaptive.min_dt_s),
                openbmp_core::Duration::from_seconds(adaptive.max_dt_s),
            )
            .map_err(|error| solver_profile_error(&error))
        }
        ("adaptive-explicit", "dopri853", "state-stable") => {
            let adaptive = solver.adaptive.as_ref().ok_or_else(|| {
                // Defensive: scenario validator already enforces
                // [solver.adaptive] is present for adaptive-explicit.
                RunnerError::UnsupportedScenario {
                    what: "[solver.adaptive] block missing for adaptive-explicit profile"
                        .to_owned(),
                }
            })?;
            validate_dense_output_support(method, adaptive)?;
            RuntimeIntegrator::from_adaptive_method(
                ExplicitMethod::DormandPrince853,
                adaptive.rtol,
                adaptive.atol,
                openbmp_core::Duration::from_seconds(adaptive.min_dt_s),
                openbmp_core::Duration::from_seconds(adaptive.max_dt_s),
            )
            .map_err(|error| solver_profile_error(&error))
        }
        ("fixed-step-explicit", "rkf78", _) => Err(RunnerError::UnsupportedScenario {
            what: "solver.trajectory_method = \"rkf78\" parses but is not wired in the runner; \
                 deferred to a future slice"
                .to_owned(),
        }),
        ("adaptive-explicit", method, _) if method != "dopri54" && method != "dopri853" => {
            Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "adaptive-explicit + {method:?} parses but is not wired in the \
                     runner; wired adaptive methods are dopri54 and dopri853"
                ),
            })
        }
        ("implicit-source-term" | "partitioned-hypersonic", "rk4" | "dopri54" | "dopri853", "state-stable") => {
            let source_terms = source_term_runtime_profile(solver.source_terms.as_ref())?;
            let trajectory = build_source_profile_trajectory(method)?;
            let coupling = if profile == "implicit-source-term" {
                SourceTermCouplingProfile::ImplicitSourceTerm
            } else {
                SourceTermCouplingProfile::PartitionedHypersonic
            };
            Ok(RuntimeIntegrator::with_source_terms(
                coupling,
                trajectory,
                source_terms,
            ))
        }
        ("implicit-source-term" | "partitioned-hypersonic", "rkf78", _) => Err(RunnerError::UnsupportedScenario {
            what: "source-term solver profiles do not support solver.trajectory_method = \"rkf78\"; \
                 wired methods are rk4, dopri54, and dopri853"
                .to_owned(),
        }),
        _ => Err(RunnerError::UnsupportedScenario {
            what: format!(
                "unsupported (profile, method, determinism) combination: \
                 ({profile:?}, {method:?}, {determinism:?})"
            ),
        }),
    }
}

fn build_source_profile_trajectory(method: &str) -> Result<RuntimeIntegrator, RunnerError> {
    RuntimeIntegrator::from_fixed_method(explicit_method(method)?)
        .map_err(|error| solver_profile_error(&error))
}

fn explicit_method(method: &str) -> Result<ExplicitMethod, RunnerError> {
    match method {
        "rk4" => Ok(ExplicitMethod::Rk4),
        "dopri54" => Ok(ExplicitMethod::DormandPrince54),
        "dopri853" => Ok(ExplicitMethod::DormandPrince853),
        "rkf78" => Ok(ExplicitMethod::RungeKuttaFehlberg78),
        _ => Err(RunnerError::UnsupportedScenario {
            what: format!("solver.trajectory_method = {method:?} is not supported"),
        }),
    }
}

fn validate_dense_output_support(
    method: &str,
    adaptive: &openbmp_scenario::AdaptiveSolverConfig,
) -> Result<(), RunnerError> {
    if adaptive.dense_output && !matches!(method, "dopri54" | "dopri853") {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "solver.adaptive.dense_output = true is wired only for adaptive-explicit \
                 + dopri54 or dopri853; method {method:?} still requires its \
                 method-specific dense-output interpolant"
            ),
        });
    }
    Ok(())
}

fn solver_profile_error(error: &SolverProfileError) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("solver profile rejected by openbmp-sim dispatcher: {error}"),
    }
}

fn source_term_runtime_profile(
    source_terms: Option<&SourceTermSolverConfig>,
) -> Result<SourceTermRuntimeProfile, RunnerError> {
    let source_terms = source_terms.ok_or_else(|| RunnerError::UnsupportedScenario {
        what: "[solver.source_terms] block missing for source-term solver profile".to_owned(),
    })?;
    if source_terms.chemistry_method != "implicit-euler" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "solver.source_terms.chemistry_method = {:?} parses but is not wired; \
                 wired method is implicit-euler",
                source_terms.chemistry_method
            ),
        });
    }
    if source_terms.material_method != "implicit-euler" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "solver.source_terms.material_method = {:?} parses but is not wired; \
                 wired method is implicit-euler",
                source_terms.material_method
            ),
        });
    }
    let chemistry_substeps = usize::try_from(source_terms.chemistry_substeps).map_err(|_| {
        RunnerError::UnsupportedScenario {
            what: "solver.source_terms.chemistry_substeps does not fit usize".to_owned(),
        }
    })?;
    let material_substeps = usize::try_from(source_terms.material_substeps).map_err(|_| {
        RunnerError::UnsupportedScenario {
            what: "solver.source_terms.material_substeps does not fit usize".to_owned(),
        }
    })?;
    let nonlinear_max_iter = usize::try_from(source_terms.nonlinear_max_iter).map_err(|_| {
        RunnerError::UnsupportedScenario {
            what: "solver.source_terms.nonlinear_max_iter does not fit usize".to_owned(),
        }
    })?;
    SourceTermRuntimeProfile::new(
        chemistry_substeps,
        material_substeps,
        source_terms.nonlinear_tolerance,
        nonlinear_max_iter,
    )
    .map_err(|error| solver_profile_error(&error))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use openbmp_scenario::{AdaptiveSolverConfig, SolverConfig};
    use openbmp_sim::{Integrator, IntegratorDeterminism};

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
    fn adaptive_explicit_dopri54_accepts_dense_output_flag() {
        let mut adaptive = well_formed_adaptive();
        adaptive.dense_output = true;
        let s = solver(
            "adaptive-explicit",
            "dopri54",
            "state-stable",
            Some(adaptive),
        );
        let result = build_runtime_integrator_from_solver(Some(&s)).unwrap();
        assert!(matches!(result, RuntimeIntegrator::Dopri54Adaptive(_)));
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
    fn adaptive_explicit_dopri853_accepts_dense_output_flag() {
        let mut adaptive = well_formed_adaptive();
        adaptive.dense_output = true;
        let s = solver(
            "adaptive-explicit",
            "dopri853",
            "state-stable",
            Some(adaptive),
        );
        let result = build_runtime_integrator_from_solver(Some(&s)).unwrap();
        assert!(matches!(result, RuntimeIntegrator::Dopri853Adaptive(_)));
    }

    #[test]
    fn fixed_step_rejects_dense_output_flag() {
        let mut adaptive = well_formed_adaptive();
        adaptive.dense_output = true;
        let s = solver(
            "fixed-step-explicit",
            "dopri54",
            "bit-stable",
            Some(adaptive),
        );
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let RunnerError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(what.contains("adaptive-explicit"));
    }

    #[test]
    fn fixed_step_rkf78_is_rejected_as_unwired() {
        let s = solver("fixed-step-explicit", "rkf78", "bit-stable", None);
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let RunnerError::UnsupportedScenario { what } = err else {
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
        let RunnerError::UnsupportedScenario { what } = err else {
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
        let RunnerError::UnsupportedScenario { what } = err else {
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
        assert_eq!(profile, SourceTermCouplingProfile::ImplicitSourceTerm);
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
        assert_eq!(profile, SourceTermCouplingProfile::PartitionedHypersonic);
        assert!(matches!(*trajectory, RuntimeIntegrator::Dopri853Fixed(_)));
    }

    #[test]
    fn source_profile_rejects_unwired_chemistry_method() {
        let mut source_terms = well_formed_source_terms();
        source_terms.chemistry_method = "bdf".to_owned();
        let s = solver_with_source_terms("implicit-source-term", "rk4", source_terms);
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let RunnerError::UnsupportedScenario { what } = err else {
            panic!("expected UnsupportedScenario, got {err:?}");
        };
        assert!(what.contains("chemistry_method") && what.contains("implicit-euler"));
    }

    #[test]
    fn adaptive_explicit_without_adaptive_block_returns_defensive_error() {
        let s = solver("adaptive-explicit", "dopri54", "state-stable", None);
        let err = build_runtime_integrator_from_solver(Some(&s)).unwrap_err();
        let RunnerError::UnsupportedScenario { what } = err else {
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
