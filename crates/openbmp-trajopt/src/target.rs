//! Forward-only terminal-condition residuals.

use openbmp_physics::profile::{BallisticState, TerminalCondition};

use crate::iload::TrajoptError;

/// Residual vector for an orbital / inertial terminal condition.
#[derive(Clone, Debug, PartialEq)]
pub struct TerminalResidual {
    /// Residual components in the terminal condition's native units.
    pub components: alloc::vec::Vec<f64>,
    /// Root-sum-square norm of [`Self::components`].
    pub norm: f64,
}

/// Compute residuals for the closed, forward-only
/// [`TerminalCondition`] enum.
///
/// The residual is always relative to an orbital / inertial /
/// vehicle-intrinsic condition. It never scores miss distance to a
/// surface aimpoint.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the state, gravitational parameter,
/// or requested terminal condition is invalid.
pub fn terminal_residual(
    condition: &TerminalCondition,
    state: &BallisticState,
    mu_m3_s2: f64,
) -> Result<TerminalResidual, TrajoptError> {
    state.validate().map_err(|_| TrajoptError::InvalidPayload {
        reason: "terminal residual state is invalid",
    })?;
    condition
        .validate()
        .map_err(|_| TrajoptError::InvalidPayload {
            reason: "terminal condition is invalid",
        })?;
    if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "terminal residual gravity parameter must be finite and positive",
        });
    }
    let elements = orbital_elements(state, mu_m3_s2)?;
    let components = match *condition {
        TerminalCondition::OrbitalElements {
            semi_major_axis_m,
            eccentricity,
            inclination_rad,
        } => alloc::vec![
            elements.semi_major_axis_m - semi_major_axis_m,
            elements.eccentricity - eccentricity,
            elements.inclination_rad - inclination_rad,
        ],
        TerminalCondition::ApogeeRadius { radius_m } => {
            alloc::vec![elements.apogee_radius_m - radius_m]
        }
        TerminalCondition::FlightPathAngleAtBurnout { angle_rad } => {
            alloc::vec![elements.flight_path_angle_rad - angle_rad]
        }
        TerminalCondition::RendezvousState {
            position_eci_m,
            velocity_eci_m_s,
        } => {
            let position = state.position_eci_m();
            let velocity = state.velocity_eci_m_s();
            alloc::vec![
                position[0] - position_eci_m[0],
                position[1] - position_eci_m[1],
                position[2] - position_eci_m[2],
                velocity[0] - velocity_eci_m_s[0],
                velocity[1] - velocity_eci_m_s[1],
                velocity[2] - velocity_eci_m_s[2],
            ]
        }
        TerminalCondition::MaximizePayloadMass => alloc::vec![0.0],
    };
    let norm = components
        .iter()
        .fold(0.0_f64, |sum, value| sum + value * value)
        .sqrt();
    Ok(TerminalResidual { components, norm })
}

#[derive(Clone, Copy, Debug)]
struct OrbitalElements {
    semi_major_axis_m: f64,
    eccentricity: f64,
    inclination_rad: f64,
    apogee_radius_m: f64,
    flight_path_angle_rad: f64,
}

fn orbital_elements(
    state: &BallisticState,
    mu_m3_s2: f64,
) -> Result<OrbitalElements, TrajoptError> {
    let r = state.position_eci_m();
    let v = state.velocity_eci_m_s();
    let r_norm = norm3(r);
    let v_norm = norm3(v);
    if r_norm <= f64::EPSILON || v_norm <= f64::EPSILON {
        return Err(TrajoptError::InvalidPayload {
            reason: "terminal residual requires non-degenerate position and velocity",
        });
    }
    let h = cross(r, v);
    let h_norm = norm3(h);
    if h_norm <= f64::EPSILON {
        return Err(TrajoptError::InvalidPayload {
            reason: "terminal residual angular momentum is degenerate",
        });
    }
    let energy = 0.5 * v_norm * v_norm - mu_m3_s2 / r_norm;
    if energy >= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "terminal residual orbital elements require a bound conic",
        });
    }
    let semi_major_axis_m = -mu_m3_s2 / (2.0 * energy);
    let vxh = cross(v, h);
    let e_vec = [
        vxh[0] / mu_m3_s2 - r[0] / r_norm,
        vxh[1] / mu_m3_s2 - r[1] / r_norm,
        vxh[2] / mu_m3_s2 - r[2] / r_norm,
    ];
    let eccentricity = norm3(e_vec);
    let inclination_rad = (h[2] / h_norm).clamp(-1.0, 1.0).acos();
    let apogee_radius_m = semi_major_axis_m * (1.0 + eccentricity);
    let radial_velocity_m_s = dot(r, v) / r_norm;
    let flight_path_angle_rad = (radial_velocity_m_s / v_norm).clamp(-1.0, 1.0).asin();
    Ok(OrbitalElements {
        semi_major_axis_m,
        eccentricity,
        inclination_rad,
        apogee_radius_m,
        flight_path_angle_rad,
    })
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm3(value: [f64; 3]) -> f64 {
    dot(value, value).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use openbmp_core::SimTime;
    use openbmp_physics::WGS84_MU_M3_S2;
    use openbmp_physics::profile::{ForwardSimulationProvenance, ForwardSimulationSource};

    fn test_state(
        position_eci_m: [f64; 3],
        velocity_eci_m_s: [f64; 3],
    ) -> Result<BallisticState, openbmp_physics::PhysicsError> {
        let provenance = ForwardSimulationProvenance::for_state(
            position_eci_m,
            velocity_eci_m_s,
            0.0,
            SimTime::ZERO,
            ForwardSimulationSource::RunnerTelemetryState,
        );
        BallisticState::from_forward_simulation(
            position_eci_m,
            velocity_eci_m_s,
            0.0,
            SimTime::ZERO,
            provenance,
        )
    }

    #[test]
    fn circular_orbit_residual_is_zero_for_matching_elements() {
        let mu = WGS84_MU_M3_S2;
        let radius = 7_000_000.0_f64;
        let speed = (mu / radius).sqrt();
        let state = test_state([radius, 0.0, 0.0], [0.0, speed, 0.0]).unwrap();
        let condition = TerminalCondition::OrbitalElements {
            semi_major_axis_m: radius,
            eccentricity: 0.0,
            inclination_rad: 0.0,
        };
        let residual = terminal_residual(&condition, &state, mu).unwrap();
        assert!(residual.norm < 1.0e-8, "{residual:?}");
    }

    #[test]
    fn rendezvous_residual_reports_inertial_state_difference() {
        let state = test_state([1.0, 2.0, 3.0], [0.0, 7_800.0, 1.0]).unwrap();
        let condition = TerminalCondition::RendezvousState {
            position_eci_m: [1.0, 4.0, 3.0],
            velocity_eci_m_s: [0.0, 7_799.0, 1.0],
        };
        let residual = terminal_residual(&condition, &state, WGS84_MU_M3_S2).unwrap();
        assert_eq!(
            residual.components,
            alloc::vec![0.0, -2.0, 0.0, 0.0, 1.0, 0.0]
        );
    }
}
