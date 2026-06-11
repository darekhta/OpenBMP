//! State-transition matrix propagation for T1 shooting methods.
//!
//! This module keeps the first STM surface deliberately narrow: a deterministic
//! two-body Cartesian variational equation integrated with the same fixed-step
//! RK4 pattern used by the T0 apogee driver. It is the sensitivity substrate
//! for multiple shooting; it does not introduce any target vocabulary.

use crate::iload::TrajoptError;

/// Cartesian two-body state `[r, v]` in an inertial frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TwoBodyCartesianState {
    /// Inertial position, in m.
    pub position_eci_m: [f64; 3],
    /// Inertial velocity, in m/s.
    pub velocity_eci_m_s: [f64; 3],
}

impl TwoBodyCartesianState {
    /// Construct a finite Cartesian state.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when any state component is non-finite.
    pub fn new(position_eci_m: [f64; 3], velocity_eci_m_s: [f64; 3]) -> Result<Self, TrajoptError> {
        let state = Self {
            position_eci_m,
            velocity_eci_m_s,
        };
        validate_state(state)?;
        Ok(state)
    }

    /// Flatten to `[x, y, z, vx, vy, vz]`.
    #[must_use]
    pub fn to_array(self) -> [f64; 6] {
        [
            self.position_eci_m[0],
            self.position_eci_m[1],
            self.position_eci_m[2],
            self.velocity_eci_m_s[0],
            self.velocity_eci_m_s[1],
            self.velocity_eci_m_s[2],
        ]
    }

    /// Construct from `[x, y, z, vx, vy, vz]`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when any state component is non-finite.
    pub fn from_array(value: [f64; 6]) -> Result<Self, TrajoptError> {
        Self::new(
            [value[0], value[1], value[2]],
            [value[3], value[4], value[5]],
        )
    }
}

/// Row-major 6x6 state-transition matrix `d x(t_f) / d x(t_0)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateTransitionMatrix {
    /// Row-major values.
    pub row_major: [f64; 36],
}

impl StateTransitionMatrix {
    /// Identity STM.
    #[must_use]
    pub const fn identity() -> Self {
        let mut row_major = [0.0_f64; 36];
        row_major[0] = 1.0;
        row_major[7] = 1.0;
        row_major[14] = 1.0;
        row_major[21] = 1.0;
        row_major[28] = 1.0;
        row_major[35] = 1.0;
        Self { row_major }
    }

    /// Matrix element at `row, column`.
    #[must_use]
    pub const fn get(self, row: usize, column: usize) -> f64 {
        self.row_major[row * 6 + column]
    }
}

/// Result of propagating state plus STM.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TwoBodyVariationalPropagation {
    /// Propagated terminal state.
    pub terminal_state: TwoBodyCartesianState,
    /// State-transition matrix from initial state to terminal state.
    pub stm: StateTransitionMatrix,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct AugmentedState {
    state: [f64; 6],
    stm: [f64; 36],
}

/// Propagate a two-body Cartesian state and its STM with fixed-step RK4.
///
/// # Errors
///
/// Returns [`TrajoptError`] when inputs are malformed or propagation produces a
/// non-finite/degenerate state.
pub fn propagate_two_body_variational(
    initial_state: TwoBodyCartesianState,
    duration_s: f64,
    step_s: f64,
    mu_m3_s2: f64,
) -> Result<TwoBodyVariationalPropagation, TrajoptError> {
    validate_problem(duration_s, step_s, mu_m3_s2)?;
    let mut augmented = AugmentedState {
        state: initial_state.to_array(),
        stm: StateTransitionMatrix::identity().row_major,
    };
    let mut elapsed_s = 0.0_f64;
    while elapsed_s < duration_s {
        let remaining_s = duration_s - elapsed_s;
        let dt_s = remaining_s.min(step_s);
        augmented = rk4_augmented_step(augmented, dt_s, mu_m3_s2)?;
        elapsed_s += dt_s;
    }
    Ok(TwoBodyVariationalPropagation {
        terminal_state: TwoBodyCartesianState::from_array(augmented.state)?,
        stm: StateTransitionMatrix {
            row_major: augmented.stm,
        },
    })
}

fn validate_problem(duration_s: f64, step_s: f64, mu_m3_s2: f64) -> Result<(), TrajoptError> {
    if !duration_s.is_finite() || duration_s < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "STM propagation duration must be finite and non-negative",
        });
    }
    if !step_s.is_finite() || step_s <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "STM propagation step must be finite and positive",
        });
    }
    if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "STM propagation gravity parameter must be finite and positive",
        });
    }
    Ok(())
}

fn rk4_augmented_step(
    state: AugmentedState,
    dt_s: f64,
    mu_m3_s2: f64,
) -> Result<AugmentedState, TrajoptError> {
    let k1 = augmented_derivative(state, mu_m3_s2)?;
    let k2 = augmented_derivative(offset_augmented(state, k1, 0.5 * dt_s)?, mu_m3_s2)?;
    let k3 = augmented_derivative(offset_augmented(state, k2, 0.5 * dt_s)?, mu_m3_s2)?;
    let k4 = augmented_derivative(offset_augmented(state, k3, dt_s)?, mu_m3_s2)?;
    let mut next = state;
    let one_sixth_dt = dt_s / 6.0;
    for i in 0..6 {
        next.state[i] +=
            (k1.state[i] + 2.0 * k2.state[i] + 2.0 * k3.state[i] + k4.state[i]) * one_sixth_dt;
    }
    for i in 0..36 {
        next.stm[i] += (k1.stm[i] + 2.0 * k2.stm[i] + 2.0 * k3.stm[i] + k4.stm[i]) * one_sixth_dt;
    }
    validate_augmented(next)?;
    Ok(next)
}

fn offset_augmented(
    state: AugmentedState,
    derivative: AugmentedState,
    dt_s: f64,
) -> Result<AugmentedState, TrajoptError> {
    let mut offset = state;
    for i in 0..6 {
        offset.state[i] += derivative.state[i] * dt_s;
    }
    for i in 0..36 {
        offset.stm[i] += derivative.stm[i] * dt_s;
    }
    validate_augmented(offset)?;
    Ok(offset)
}

fn augmented_derivative(
    state: AugmentedState,
    mu_m3_s2: f64,
) -> Result<AugmentedState, TrajoptError> {
    validate_augmented(state)?;
    let state_derivative = two_body_state_derivative(state.state, mu_m3_s2)?;
    let dynamics_jacobian = two_body_dynamics_jacobian(state.state, mu_m3_s2)?;
    let mut stm_derivative = [0.0_f64; 36];
    for row in 0..6 {
        for column in 0..6 {
            let mut sum = 0.0_f64;
            for inner in 0..6 {
                sum += dynamics_jacobian[row * 6 + inner] * state.stm[inner * 6 + column];
            }
            stm_derivative[row * 6 + column] = sum;
        }
    }
    Ok(AugmentedState {
        state: state_derivative,
        stm: stm_derivative,
    })
}

fn two_body_state_derivative(state: [f64; 6], mu_m3_s2: f64) -> Result<[f64; 6], TrajoptError> {
    let position = [state[0], state[1], state[2]];
    let radius_m = norm3(position);
    if radius_m <= f64::EPSILON {
        return Err(TrajoptError::InvalidPayload {
            reason: "STM propagation state radius is degenerate",
        });
    }
    let inv_r3 = 1.0 / (radius_m * radius_m * radius_m);
    Ok([
        state[3],
        state[4],
        state[5],
        -mu_m3_s2 * state[0] * inv_r3,
        -mu_m3_s2 * state[1] * inv_r3,
        -mu_m3_s2 * state[2] * inv_r3,
    ])
}

fn two_body_dynamics_jacobian(state: [f64; 6], mu_m3_s2: f64) -> Result<[f64; 36], TrajoptError> {
    let position = [state[0], state[1], state[2]];
    let radius_m = norm3(position);
    if radius_m <= f64::EPSILON {
        return Err(TrajoptError::InvalidPayload {
            reason: "STM propagation state radius is degenerate",
        });
    }
    let r2 = radius_m * radius_m;
    let r3 = r2 * radius_m;
    let r5 = r3 * r2;
    let mut jacobian = [0.0_f64; 36];
    jacobian[3] = 1.0;
    jacobian[10] = 1.0;
    jacobian[17] = 1.0;
    for row in 0..3 {
        for column in 0..3 {
            let identity = if row == column { 1.0 } else { 0.0 };
            jacobian[(row + 3) * 6 + column] =
                -mu_m3_s2 * (identity / r3 - 3.0 * position[row] * position[column] / r5);
        }
    }
    Ok(jacobian)
}

fn validate_augmented(state: AugmentedState) -> Result<(), TrajoptError> {
    if !state.state.iter().all(|value| value.is_finite())
        || !state.stm.iter().all(|value| value.is_finite())
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "STM propagation produced non-finite state",
        });
    }
    TwoBodyCartesianState::from_array(state.state)?;
    Ok(())
}

fn validate_state(state: TwoBodyCartesianState) -> Result<(), TrajoptError> {
    if !state
        .position_eci_m
        .iter()
        .chain(state.velocity_eci_m_s.iter())
        .all(|value| value.is_finite())
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "two-body Cartesian state must be finite",
        });
    }
    Ok(())
}

fn norm3(value: [f64; 3]) -> f64 {
    (value[0] * value[0] + value[1] * value[1] + value[2] * value[2]).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use num_complex::Complex64;
    use openbmp_physics::WGS84_MU_M3_S2;

    #[test]
    fn stm_identity_for_zero_duration() -> Result<(), TrajoptError> {
        let state = TwoBodyCartesianState::new([6_778_000.0, 0.0, 0.0], [0.0, 7_668.635_675, 0.0])?;

        let propagated = propagate_two_body_variational(state, 0.0, 10.0, WGS84_MU_M3_S2)?;

        assert_eq!(propagated.terminal_state, state);
        assert_eq!(propagated.stm, StateTransitionMatrix::identity());
        Ok(())
    }

    #[test]
    fn stm_matches_central_difference_columns() -> Result<(), TrajoptError> {
        let state = TwoBodyCartesianState::new([6_778_000.0, 0.0, 0.0], [0.0, 7_668.635_675, 0.0])?;
        let duration_s = 120.0;
        let step_s = 10.0;
        let propagated = propagate_two_body_variational(state, duration_s, step_s, WGS84_MU_M3_S2)?;
        let base = state.to_array();
        let perturbation = 1.0e-3;

        for column in 0..6 {
            let mut plus = base;
            plus[column] += perturbation;
            let plus_state = TwoBodyCartesianState::from_array(plus)?;
            let plus_terminal =
                propagate_two_body_variational(plus_state, duration_s, step_s, WGS84_MU_M3_S2)?
                    .terminal_state
                    .to_array();

            let mut minus = base;
            minus[column] -= perturbation;
            let minus_state = TwoBodyCartesianState::from_array(minus)?;
            let minus_terminal =
                propagate_two_body_variational(minus_state, duration_s, step_s, WGS84_MU_M3_S2)?
                    .terminal_state
                    .to_array();

            for row in 0..6 {
                let finite_difference =
                    (plus_terminal[row] - minus_terminal[row]) / (2.0 * perturbation);
                let stm_value = propagated.stm.get(row, column);
                assert!(
                    (finite_difference - stm_value).abs() < 5.0e-4,
                    "row={row} column={column} finite_difference={finite_difference} stm={stm_value}"
                );
            }
        }
        Ok(())
    }

    #[test]
    fn stm_matches_complex_step_columns() -> Result<(), TrajoptError> {
        let state = TwoBodyCartesianState::new([6_778_000.0, 0.0, 0.0], [0.0, 7_668.635_675, 0.0])?;
        let duration_s = 120.0;
        let step_s = 10.0;
        let propagated = propagate_two_body_variational(state, duration_s, step_s, WGS84_MU_M3_S2)?;
        let base = state.to_array();
        let imaginary_step = 1.0e-30;

        for column in 0..6 {
            let mut complex_state = [Complex64::new(0.0, 0.0); 6];
            for i in 0..6 {
                complex_state[i] = Complex64::new(base[i], 0.0);
            }
            complex_state[column].im = imaginary_step;
            let terminal =
                propagate_two_body_complex(complex_state, duration_s, step_s, WGS84_MU_M3_S2)?;

            for (row, terminal_component) in terminal.iter().enumerate() {
                let complex_step = terminal_component.im / imaginary_step;
                let stm_value = propagated.stm.get(row, column);
                assert!(
                    (complex_step - stm_value).abs() < 1.0e-8,
                    "row={row} column={column} complex_step={complex_step} stm={stm_value}"
                );
            }
        }
        Ok(())
    }

    fn propagate_two_body_complex(
        mut state: [Complex64; 6],
        duration_s: f64,
        step_s: f64,
        mu_m3_s2: f64,
    ) -> Result<[Complex64; 6], TrajoptError> {
        let mut elapsed_s = 0.0_f64;
        while elapsed_s < duration_s {
            let dt_s = (duration_s - elapsed_s).min(step_s);
            state = rk4_complex_step(state, dt_s, mu_m3_s2)?;
            elapsed_s += dt_s;
        }
        Ok(state)
    }

    fn rk4_complex_step(
        state: [Complex64; 6],
        dt_s: f64,
        mu_m3_s2: f64,
    ) -> Result<[Complex64; 6], TrajoptError> {
        let k1 = complex_state_derivative(state, mu_m3_s2)?;
        let k2 = complex_state_derivative(offset_complex_state(state, k1, 0.5 * dt_s), mu_m3_s2)?;
        let k3 = complex_state_derivative(offset_complex_state(state, k2, 0.5 * dt_s), mu_m3_s2)?;
        let k4 = complex_state_derivative(offset_complex_state(state, k3, dt_s), mu_m3_s2)?;
        let mut next = state;
        let one_sixth_dt = dt_s / 6.0;
        for i in 0..6 {
            next[i] += (k1[i] + k2[i] * 2.0 + k3[i] * 2.0 + k4[i]) * one_sixth_dt;
            if !next[i].re.is_finite() || !next[i].im.is_finite() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "complex-step propagation produced non-finite state",
                });
            }
        }
        Ok(next)
    }

    fn offset_complex_state(
        state: [Complex64; 6],
        derivative: [Complex64; 6],
        dt_s: f64,
    ) -> [Complex64; 6] {
        let mut offset = state;
        for i in 0..6 {
            offset[i] += derivative[i] * dt_s;
        }
        offset
    }

    fn complex_state_derivative(
        state: [Complex64; 6],
        mu_m3_s2: f64,
    ) -> Result<[Complex64; 6], TrajoptError> {
        let radius_squared = state[0] * state[0] + state[1] * state[1] + state[2] * state[2];
        let radius = radius_squared.sqrt();
        if radius.norm() <= f64::EPSILON {
            return Err(TrajoptError::InvalidPayload {
                reason: "complex-step state radius is degenerate",
            });
        }
        let inv_r3 = Complex64::new(1.0, 0.0) / (radius_squared * radius);
        Ok([
            state[3],
            state[4],
            state[5],
            -mu_m3_s2 * state[0] * inv_r3,
            -mu_m3_s2 * state[1] * inv_r3,
            -mu_m3_s2 * state[2] * inv_r3,
        ])
    }
}
