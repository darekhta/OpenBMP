//! Differential corrector (Gauss-Newton shooting).
//!
//! [`DifferentialCorrector`] drives a caller-supplied forward
//! "shooting" map — a set of free control variables mapped to a
//! terminal [`BallisticState`] — until the orbital / inertial
//! [`TerminalCondition`] residual ([`terminal_residual`]) is nulled.
//!
//! This is the differential-correction backend in the GMAT sense: vary
//! the free variables, finite-difference the Jacobian, take a
//! Gauss-Newton step, repeat. It is scoped to the closed, forward-only
//! terminal-condition set and reduces a free-flight state to orbital
//! elements only. It targets orbital / inertial / vehicle-intrinsic
//! conditions exclusively — never a surface coordinate, and its outputs
//! are residuals to those conditions, never a surface-relative error.
//! The crate is offline / L4 and is not linked by `openbmp-fc`, so the
//! corrector is unreachable from the flight-control loop.
//!
//! The forward map is supplied by the caller as a closure, so the
//! corrector is independent of any particular propagator; here it is
//! exercised against analytic two-body shooting maps.

use alloc::vec;
use alloc::vec::Vec;

use openbmp_physics::profile::{BallisticState, TerminalCondition};

use crate::iload::TrajoptError;
use crate::target::{TerminalResidual, terminal_residual};

/// Configuration for a [`DifferentialCorrector`] run.
///
/// The residual tolerance is expressed in the terminal condition's
/// native (and, for [`TerminalCondition::OrbitalElements`], mixed)
/// units, so callers should choose it to match the condition's scale.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DifferentialCorrector {
    /// Maximum Gauss-Newton iterations before reporting non-convergence.
    pub max_iterations: usize,
    /// Convergence threshold on the residual root-sum-square norm.
    pub residual_tolerance: f64,
    /// Relative forward finite-difference step used to build the Jacobian.
    pub finite_difference_step: f64,
    /// Levenberg-Marquardt damping added to the normal-equation diagonal.
    ///
    /// Zero gives a plain Gauss-Newton step; a small positive value
    /// regularizes an ill-conditioned or rank-deficient Jacobian.
    pub levenberg_marquardt_damping: f64,
    /// Cap on the Euclidean norm of a single correction step.
    ///
    /// Use [`f64::INFINITY`] to disable step limiting.
    pub max_step_norm: f64,
}

impl Default for DifferentialCorrector {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            residual_tolerance: 1.0e-6,
            finite_difference_step: 1.0e-6,
            levenberg_marquardt_damping: 0.0,
            max_step_norm: f64::INFINITY,
        }
    }
}

/// Result of a [`DifferentialCorrector::solve`] run.
#[derive(Clone, Debug, PartialEq)]
pub struct DifferentialCorrection {
    /// Free-variable vector at the final iterate.
    pub free_variables: Vec<f64>,
    /// Terminal residual at the final iterate.
    pub residual: TerminalResidual,
    /// Number of Gauss-Newton iterations taken.
    pub iterations: usize,
    /// Whether the residual norm reached the configured tolerance.
    ///
    /// Callers must inspect this: a `false` value means the iterate is
    /// the best effort within the iteration budget, not a converged
    /// solution.
    pub converged: bool,
}

impl DifferentialCorrector {
    /// Drive `forward_map` so the [`TerminalCondition`] residual is
    /// nulled, starting from `initial_free_variables`.
    ///
    /// `forward_map` maps a free-variable slice to the terminal
    /// free-flight [`BallisticState`] it produces. `mu_m3_s2` is the
    /// central-body gravitational parameter used to reduce that state
    /// to orbital elements for the residual.
    ///
    /// Targets orbital / inertial conditions only. The objective-style
    /// [`TerminalCondition::MaximizePayloadMass`] is rejected, because
    /// differential correction nulls constraints rather than optimizing
    /// an objective.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the configuration or inputs are
    /// invalid, the forward map fails, the residual dimension is not
    /// stable across the finite-difference stencil, or a correction
    /// step's normal equations are singular.
    pub fn solve<F>(
        &self,
        condition: &TerminalCondition,
        mu_m3_s2: f64,
        initial_free_variables: &[f64],
        mut forward_map: F,
    ) -> Result<DifferentialCorrection, TrajoptError>
    where
        F: FnMut(&[f64]) -> Result<BallisticState, TrajoptError>,
    {
        if matches!(condition, TerminalCondition::MaximizePayloadMass) {
            return Err(TrajoptError::InvalidPayload {
                reason: "differential correction targets constraint terminal conditions, \
                         not the payload-mass objective",
            });
        }
        self.validate_config()?;

        let free_count = initial_free_variables.len();
        if free_count == 0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "differential correction requires at least one free variable",
            });
        }
        for &value in initial_free_variables {
            if !value.is_finite() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "initial free variables must be finite",
                });
            }
        }

        let mut free = initial_free_variables.to_vec();
        let mut residual = residual_at(condition, mu_m3_s2, &free, &mut forward_map)?;
        let residual_count = residual.components.len();
        let mut iterations = 0;

        while residual.norm > self.residual_tolerance {
            if iterations >= self.max_iterations {
                return Ok(DifferentialCorrection {
                    free_variables: free,
                    residual,
                    iterations,
                    converged: false,
                });
            }

            let jacobian = self.finite_difference_jacobian(
                condition,
                mu_m3_s2,
                &free,
                &residual.components,
                residual_count,
                free_count,
                &mut forward_map,
            )?;
            let step = self.gauss_newton_step(
                &jacobian,
                &residual.components,
                residual_count,
                free_count,
            )?;
            apply_step(&mut free, &step, self.max_step_norm)?;

            iterations += 1;
            residual = residual_at(condition, mu_m3_s2, &free, &mut forward_map)?;
            if residual.components.len() != residual_count {
                return Err(TrajoptError::InvalidPayload {
                    reason: "terminal residual dimension changed during correction",
                });
            }
        }

        Ok(DifferentialCorrection {
            free_variables: free,
            residual,
            iterations,
            converged: true,
        })
    }

    fn validate_config(&self) -> Result<(), TrajoptError> {
        if !self.residual_tolerance.is_finite() || self.residual_tolerance < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "residual tolerance must be finite and non-negative",
            });
        }
        if !self.finite_difference_step.is_finite() || self.finite_difference_step <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "finite-difference step must be finite and positive",
            });
        }
        if !self.levenberg_marquardt_damping.is_finite() || self.levenberg_marquardt_damping < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Levenberg-Marquardt damping must be finite and non-negative",
            });
        }
        if self.max_step_norm.is_nan() || self.max_step_norm <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "maximum step norm must be positive",
            });
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn finite_difference_jacobian<F>(
        &self,
        condition: &TerminalCondition,
        mu_m3_s2: f64,
        free: &[f64],
        base_components: &[f64],
        residual_count: usize,
        free_count: usize,
        forward_map: &mut F,
    ) -> Result<Vec<f64>, TrajoptError>
    where
        F: FnMut(&[f64]) -> Result<BallisticState, TrajoptError>,
    {
        // Row-major `residual_count x free_count` Jacobian.
        let mut jacobian = vec![0.0_f64; residual_count * free_count];
        let mut perturbed = free.to_vec();
        for column in 0..free_count {
            let step = self.finite_difference_step * (1.0 + free[column].abs());
            perturbed[column] = free[column] + step;
            let forward = residual_at(condition, mu_m3_s2, &perturbed, forward_map)?;
            perturbed[column] = free[column];
            if forward.components.len() != residual_count {
                return Err(TrajoptError::InvalidPayload {
                    reason: "terminal residual dimension changed across the \
                             finite-difference stencil",
                });
            }
            for row in 0..residual_count {
                jacobian[row * free_count + column] =
                    (forward.components[row] - base_components[row]) / step;
            }
        }
        Ok(jacobian)
    }

    fn gauss_newton_step(
        &self,
        jacobian: &[f64],
        residual: &[f64],
        residual_count: usize,
        free_count: usize,
    ) -> Result<Vec<f64>, TrajoptError> {
        // Normal equations: `(Jᵀ J + lambda I) dx = -Jᵀ r`.
        let mut normal = vec![0.0_f64; free_count * free_count];
        let mut rhs = vec![0.0_f64; free_count];
        for column in 0..free_count {
            for other in 0..free_count {
                let mut accumulator = 0.0_f64;
                for row in 0..residual_count {
                    accumulator +=
                        jacobian[row * free_count + column] * jacobian[row * free_count + other];
                }
                normal[column * free_count + other] = accumulator;
            }
            normal[column * free_count + column] += self.levenberg_marquardt_damping;
            let mut gradient = 0.0_f64;
            for row in 0..residual_count {
                gradient += jacobian[row * free_count + column] * residual[row];
            }
            rhs[column] = -gradient;
        }
        solve_linear_system(normal, rhs, free_count)
    }
}

fn residual_at<F>(
    condition: &TerminalCondition,
    mu_m3_s2: f64,
    free: &[f64],
    forward_map: &mut F,
) -> Result<TerminalResidual, TrajoptError>
where
    F: FnMut(&[f64]) -> Result<BallisticState, TrajoptError>,
{
    let state = forward_map(free)?;
    terminal_residual(condition, &state, mu_m3_s2)
}

fn apply_step(free: &mut [f64], step: &[f64], max_step_norm: f64) -> Result<(), TrajoptError> {
    let mut norm_squared = 0.0_f64;
    for &value in step {
        norm_squared += value * value;
    }
    let norm = norm_squared.sqrt();
    let scale = if max_step_norm.is_finite() && norm > max_step_norm && norm > 0.0 {
        max_step_norm / norm
    } else {
        1.0
    };
    for (variable, &delta) in free.iter_mut().zip(step.iter()) {
        *variable += scale * delta;
        if !variable.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "free variable became non-finite during correction",
            });
        }
    }
    Ok(())
}

/// Solve a dense `size x size` system `a x = b` by Gaussian
/// elimination with partial pivoting. `a` is row-major and both `a`
/// and `b` are consumed in place.
fn solve_linear_system(
    mut a: Vec<f64>,
    mut b: Vec<f64>,
    size: usize,
) -> Result<Vec<f64>, TrajoptError> {
    for column in 0..size {
        let mut pivot_row = column;
        let mut pivot_magnitude = a[column * size + column].abs();
        for row in (column + 1)..size {
            let magnitude = a[row * size + column].abs();
            if magnitude > pivot_magnitude {
                pivot_magnitude = magnitude;
                pivot_row = row;
            }
        }
        if pivot_magnitude <= f64::EPSILON {
            return Err(TrajoptError::SingularSystem {
                reason: "rank-deficient Jacobian; add Levenberg-Marquardt damping or \
                         supply an independent free variable",
            });
        }
        if pivot_row != column {
            for k in 0..size {
                a.swap(column * size + k, pivot_row * size + k);
            }
            b.swap(column, pivot_row);
        }
        let diagonal = a[column * size + column];
        for row in (column + 1)..size {
            let factor = a[row * size + column] / diagonal;
            if factor != 0.0 {
                for k in column..size {
                    a[row * size + k] -= factor * a[column * size + k];
                }
                b[row] -= factor * b[column];
            }
        }
    }

    let mut solution = vec![0.0_f64; size];
    for column in (0..size).rev() {
        let mut accumulator = b[column];
        for k in (column + 1)..size {
            accumulator -= a[column * size + k] * solution[k];
        }
        let diagonal = a[column * size + column];
        if diagonal.abs() <= f64::EPSILON {
            return Err(TrajoptError::SingularSystem {
                reason: "rank-deficient Jacobian; add Levenberg-Marquardt damping or \
                         supply an independent free variable",
            });
        }
        solution[column] = accumulator / diagonal;
    }
    Ok(solution)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openbmp_core::SimTime;
    use openbmp_physics::WGS84_MU_M3_S2;
    use openbmp_physics::profile::{ForwardSimulationProvenance, ForwardSimulationSource};

    /// Two-body forward map: a state at `radius_m` on the x-axis with a
    /// purely tangential (y-axis) velocity of `speed_m_s`. Tangential
    /// velocity makes the radius an apsis of the resulting conic.
    fn tangential_state(radius_m: f64, speed_m_s: f64) -> Result<BallisticState, TrajoptError> {
        let position = [radius_m, 0.0, 0.0];
        let velocity = [0.0, speed_m_s, 0.0];
        let token = ForwardSimulationProvenance::for_state(
            position,
            velocity,
            0.0,
            SimTime::ZERO,
            ForwardSimulationSource::RunnerTelemetryState,
        );
        BallisticState::from_forward_simulation(position, velocity, 0.0, SimTime::ZERO, token)
            .map_err(|_| TrajoptError::InvalidPayload {
                reason: "test forward map produced an invalid state",
            })
    }

    fn meters_tolerance() -> DifferentialCorrector {
        DifferentialCorrector {
            residual_tolerance: 1.0e-2,
            max_iterations: 100,
            ..DifferentialCorrector::default()
        }
    }

    #[test]
    fn corrects_tangential_speed_to_target_apogee() -> Result<(), TrajoptError> {
        let perigee_m = 6_778_000.0_f64;
        let target_apogee_m = 7_578_000.0_f64;
        let circular_speed = (WGS84_MU_M3_S2 / perigee_m).sqrt();
        let condition = TerminalCondition::ApogeeRadius {
            radius_m: target_apogee_m,
        };

        let result =
            meters_tolerance().solve(&condition, WGS84_MU_M3_S2, &[circular_speed], |free| {
                tangential_state(perigee_m, free[0])
            })?;

        assert!(result.converged, "{result:?}");
        assert!(result.residual.norm < 1.0e-2, "{result:?}");
        // A higher apogee than the (circular) perigee needs supercircular speed.
        assert!(result.free_variables[0] > circular_speed, "{result:?}");
        Ok(())
    }

    #[test]
    fn corrects_overdetermined_circular_orbit() -> Result<(), TrajoptError> {
        // One free variable (tangential speed) against a three-component
        // residual (a, e, i). The circular speed nulls all three at once.
        let radius_m = 7_000_000.0_f64;
        let circular_speed = (WGS84_MU_M3_S2 / radius_m).sqrt();
        let condition = TerminalCondition::OrbitalElements {
            semi_major_axis_m: radius_m,
            eccentricity: 0.0,
            inclination_rad: 0.0,
        };

        // Start 5 % fast so the corrector has to work.
        let result = meters_tolerance().solve(
            &condition,
            WGS84_MU_M3_S2,
            &[circular_speed * 1.05],
            |free| tangential_state(radius_m, free[0]),
        )?;

        assert!(result.converged, "{result:?}");
        assert!(
            (result.free_variables[0] - circular_speed).abs() < 1.0e-2,
            "{result:?}"
        );
        Ok(())
    }

    #[test]
    fn rejects_payload_mass_objective() {
        let result = DifferentialCorrector::default().solve(
            &TerminalCondition::MaximizePayloadMass,
            WGS84_MU_M3_S2,
            &[7_500.0],
            |_free| tangential_state(7_000_000.0, 7_500.0),
        );
        assert!(
            matches!(result, Err(TrajoptError::InvalidPayload { .. })),
            "{result:?}"
        );
    }

    #[test]
    fn rejects_empty_free_variables() {
        let condition = TerminalCondition::ApogeeRadius {
            radius_m: 7_000_000.0,
        };
        let result =
            DifferentialCorrector::default().solve(&condition, WGS84_MU_M3_S2, &[], |_free| {
                tangential_state(7_000_000.0, 7_500.0)
            });
        assert!(
            matches!(result, Err(TrajoptError::InvalidPayload { .. })),
            "{result:?}"
        );
    }

    #[test]
    fn reports_non_convergence_within_budget() -> Result<(), TrajoptError> {
        let perigee_m = 6_778_000.0_f64;
        let circular_speed = (WGS84_MU_M3_S2 / perigee_m).sqrt();
        let condition = TerminalCondition::ApogeeRadius {
            radius_m: 7_578_000.0,
        };
        let corrector = DifferentialCorrector {
            max_iterations: 0,
            residual_tolerance: 1.0e-3,
            ..DifferentialCorrector::default()
        };

        // Initial guess is circular, so the apogee residual is large and
        // a zero-iteration budget cannot close it.
        let result = corrector.solve(&condition, WGS84_MU_M3_S2, &[circular_speed], |free| {
            tangential_state(perigee_m, free[0])
        })?;

        assert!(!result.converged, "{result:?}");
        assert_eq!(result.iterations, 0, "{result:?}");
        Ok(())
    }

    #[test]
    fn detects_singular_jacobian() {
        // Forward map ignores the free variable, so the Jacobian column
        // is zero and the undamped normal equations are singular.
        let condition = TerminalCondition::ApogeeRadius {
            radius_m: 9_000_000.0,
        };
        let result = DifferentialCorrector::default().solve(
            &condition,
            WGS84_MU_M3_S2,
            &[7_500.0],
            |_free| tangential_state(7_000_000.0, 7_500.0),
        );
        assert!(
            matches!(result, Err(TrajoptError::SingularSystem { .. })),
            "{result:?}"
        );
    }
}
