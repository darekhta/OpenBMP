//! Phase-6.7 re-entry trajectory infrastructure.
//!
//! Closed-form analytic-toy propagators and entry-interface
//! convenience builders for hypersonic re-entry studies. All
//! algorithms reproduce textbook reference solutions; no operational
//! re-entry profiles or targeting logic are shipped.
//!
//! * [`EntryInterfaceBuilder`] — build a re-entry state at the
//!   entry-interface altitude (typically 122 km for Earth).
//! * [`AllenEggers`] — closed-form ballistic re-entry profile
//!   (non-rotating Earth, exponential atmosphere). Peak-deceleration
//!   altitude and magnitude have analytic forms.
//! * [`Vinh`] — Vinh's 1981 dimensionless lifting-entry equations,
//!   provided as a state-evolution helper for the 6-state spherical-
//!   Earth lifting-entry case.

use crate::error::PhysicsError;
use crate::frames::WGS84_MU_M3_S2;

/// Standard gravitational acceleration at Earth surface (m/s²).
const G0_M_S2: f64 = 9.806_65;

/// Earth radius used by the toy propagators (m).
const R_EARTH_M: f64 = 6.371_0e6;

/// Entry-interface state convenience builder.
///
/// Returns a `(position, velocity, flight-path-angle, heading)`
/// tuple ready to seed a rigid-body integrator. Phase-6.7 ships the
/// scalar tuple; integration into [`openbmp_state::RigidBodyState`]
/// happens at the scenario layer.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EntryInterfaceBuilder {
    /// Entry-interface geometric altitude (m). Default 122 km.
    pub entry_altitude_m: f64,
    /// Entry-interface velocity magnitude (m/s).
    pub entry_velocity_m_s: f64,
    /// Flight-path angle below local horizon (rad). Positive for
    /// descending flight.
    pub flight_path_angle_below_horizon_rad: f64,
    /// Heading angle (rad), measured clockwise from north.
    pub heading_rad: f64,
}

impl Default for EntryInterfaceBuilder {
    fn default() -> Self {
        Self {
            entry_altitude_m: 122_000.0,
            entry_velocity_m_s: 7_800.0,
            flight_path_angle_below_horizon_rad: 0.05,
            heading_rad: 0.0,
        }
    }
}

impl EntryInterfaceBuilder {
    /// Position magnitude `r = R_E + h` (m).
    #[must_use]
    pub fn radius_m(&self) -> f64 {
        R_EARTH_M + self.entry_altitude_m
    }

    /// Validate the build: positive altitude, velocity, and
    /// flight-path angle in `[-π/2, π/2]`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] on a bad config.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        use std::f64::consts::FRAC_PI_2;
        if !self.entry_altitude_m.is_finite() || self.entry_altitude_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry altitude must be > 0",
            });
        }
        if !self.entry_velocity_m_s.is_finite() || self.entry_velocity_m_s <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry velocity must be > 0",
            });
        }
        if self.flight_path_angle_below_horizon_rad.abs() >= FRAC_PI_2 {
            return Err(PhysicsError::InvalidParameter {
                reason: "flight path angle must be in (-π/2, π/2)",
            });
        }
        Ok(())
    }
}

/// Allen-Eggers (1958) ballistic-entry closed form.
///
/// For non-rotating Earth, exponential atmosphere
/// `ρ(h) = ρ_s · exp(-β · h)`, ballistic-entry at constant ballistic
/// coefficient `B = C_D · A / m`, and constant entry flight-path
/// angle `γ_e`, the velocity profile is
///
/// ```text
///   V(h) / V_e = exp[ -ρ_s · B / (2 · β · |sin γ_e|) · exp(-β · h) ]
/// ```
///
/// Peak deceleration occurs at altitude
///
/// ```text
///   h_max_decel = (1/β) · ln[ ρ_s · B / (β · |sin γ_e|) ]
/// ```
///
/// with peak deceleration (in `g`'s):
///
/// ```text
///   n_max = V_e² · β · |sin γ_e| / (2 · g₀ · e)
/// ```
///
/// where `e = exp(1)`. This is the highest-value analytic re-entry
/// validation case in OpenBMP; the Phase-6.8 e2e test confronts an
/// integrated trajectory against these closed forms.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AllenEggers {
    /// Surface (`h = 0`) reference density (kg/m³).
    pub rho_s_kg_m3: f64,
    /// Inverse scale height `β = 1 / H` (1/m). Default 1/7000.
    pub beta_inv_m: f64,
    /// Entry velocity (m/s).
    pub entry_velocity_m_s: f64,
    /// Entry flight-path angle below horizon (rad), strictly positive.
    pub flight_path_angle_rad: f64,
    /// Ballistic coefficient `B = C_D · A / m` (m²/kg).
    pub ballistic_coefficient_m2_kg: f64,
}

impl AllenEggers {
    /// Velocity at altitude `h` (m).
    #[must_use]
    pub fn velocity_at_altitude_m_s(&self, altitude_m: f64) -> f64 {
        let exponent = -self.rho_s_kg_m3 * self.ballistic_coefficient_m2_kg
            / (2.0 * self.beta_inv_m * self.flight_path_angle_rad.sin().abs())
            * (-self.beta_inv_m * altitude_m).exp();
        self.entry_velocity_m_s * exponent.exp()
    }

    /// Altitude of peak deceleration (m).
    #[must_use]
    pub fn peak_decel_altitude_m(&self) -> f64 {
        let inner = self.rho_s_kg_m3 * self.ballistic_coefficient_m2_kg
            / (self.beta_inv_m * self.flight_path_angle_rad.sin().abs());
        inner.ln() / self.beta_inv_m
    }

    /// Peak deceleration magnitude (m/s²).
    #[must_use]
    pub fn peak_deceleration_m_s2(&self) -> f64 {
        let e = std::f64::consts::E;
        let v_e = self.entry_velocity_m_s;
        v_e * v_e * self.beta_inv_m * self.flight_path_angle_rad.sin().abs() / (2.0 * e)
    }

    /// Peak deceleration in earth `g`'s.
    #[must_use]
    pub fn peak_deceleration_g(&self) -> f64 {
        self.peak_deceleration_m_s2() / G0_M_S2
    }

    /// Validate input ranges.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] on malformed inputs.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        if !self.rho_s_kg_m3.is_finite()
            || self.rho_s_kg_m3 <= 0.0
            || !self.beta_inv_m.is_finite()
            || self.beta_inv_m <= 0.0
            || !self.entry_velocity_m_s.is_finite()
            || self.entry_velocity_m_s <= 0.0
            || !self.flight_path_angle_rad.is_finite()
            || self.flight_path_angle_rad <= 0.0
            || !self.ballistic_coefficient_m2_kg.is_finite()
            || self.ballistic_coefficient_m2_kg <= 0.0
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "AllenEggers requires positive finite parameters and γ > 0",
            });
        }
        Ok(())
    }
}

/// Vinh lifting-entry equations (1981) — 6-state spherical-Earth
/// formulation.
///
/// State: `[r, θ, φ, V, γ, ψ]` where
/// * `r` is radial distance from Earth centre (m),
/// * `θ` is longitude (rad),
/// * `φ` is geocentric latitude (rad),
/// * `V` is inertial speed magnitude (m/s),
/// * `γ` is flight-path angle (rad, positive up),
/// * `ψ` is heading angle (rad, measured east from north).
///
/// Atmospheric drag is parameterised by ballistic coefficient `B` and
/// lift by `L/D`. The right-hand side is the textbook Vinh form
/// (Vinh, *Hypersonic and Planetary Entry Flight Mechanics*, 1980).
/// OpenBMP uses this as a reference propagator for the lifting-entry
/// validation case; the production rigid-body integrator consumes
/// the same RHS via the kernel.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Vinh {
    /// Ballistic coefficient (m²/kg).
    pub ballistic_coefficient_m2_kg: f64,
    /// Lift-to-drag ratio.
    pub lift_to_drag_ratio: f64,
    /// Bank angle (rad). Default 0 (no out-of-plane lift).
    pub bank_angle_rad: f64,
    /// Reference surface density (kg/m³).
    pub rho_s_kg_m3: f64,
    /// Inverse scale height (1/m).
    pub beta_inv_m: f64,
    /// Gravitational parameter `μ = G · M` (m³/s²).
    pub mu_m3_s2: f64,
}

impl Default for Vinh {
    fn default() -> Self {
        Self {
            ballistic_coefficient_m2_kg: 0.001,
            lift_to_drag_ratio: 0.3,
            bank_angle_rad: 0.0,
            rho_s_kg_m3: 1.225,
            beta_inv_m: 1.0 / 7_000.0,
            mu_m3_s2: WGS84_MU_M3_S2,
        }
    }
}

/// Vinh state vector.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VinhState {
    /// Radial distance (m).
    pub r_m: f64,
    /// Longitude (rad).
    pub theta_rad: f64,
    /// Latitude (rad).
    pub phi_rad: f64,
    /// Velocity magnitude (m/s).
    pub velocity_m_s: f64,
    /// Flight-path angle (rad, positive up).
    pub flight_path_angle_rad: f64,
    /// Heading (rad, east from north).
    pub heading_rad: f64,
}

/// Vinh state-derivative.
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct VinhStateDerivative {
    /// dr/dt (m/s).
    pub dr_dt: f64,
    /// dθ/dt (rad/s).
    pub dtheta_dt: f64,
    /// dφ/dt (rad/s).
    pub dphi_dt: f64,
    /// dV/dt (m/s²).
    pub dv_dt: f64,
    /// dγ/dt (rad/s).
    pub dgamma_dt: f64,
    /// dψ/dt (rad/s).
    pub dpsi_dt: f64,
}

impl Vinh {
    /// Right-hand side of the Vinh lifting-entry equations.
    #[must_use]
    pub fn derivative(&self, state: VinhState) -> VinhStateDerivative {
        let r = state.r_m;
        let phi = state.phi_rad;
        let v = state.velocity_m_s;
        let gamma = state.flight_path_angle_rad;
        let psi = state.heading_rad;
        let sigma = self.bank_angle_rad;
        // Altitude over surface.
        let altitude = (r - R_EARTH_M).max(0.0);
        // Density from exponential atmosphere.
        let rho = self.rho_s_kg_m3 * (-self.beta_inv_m * altitude).exp();
        // Gravity.
        let g = self.mu_m3_s2 / (r * r);
        // Aerodynamic accelerations.
        let q_dyn = 0.5 * rho * v * v;
        let drag_per_mass = q_dyn * self.ballistic_coefficient_m2_kg;
        let lift_per_mass = drag_per_mass * self.lift_to_drag_ratio;
        // Vinh equations (Anderson 2019, §13.2).
        let dr_dt = v * gamma.sin();
        let dtheta_dt = v * gamma.cos() * psi.sin() / (r * phi.cos().max(1.0e-9));
        let dphi_dt = v * gamma.cos() * psi.cos() / r;
        let dv_dt = -drag_per_mass - g * gamma.sin();
        let dgamma_dt = (lift_per_mass * sigma.cos()) / v
            + (v / r - g / v) * gamma.cos();
        let dpsi_dt = lift_per_mass * sigma.sin() / (v * gamma.cos().max(1.0e-9))
            - v * gamma.cos() * psi.sin() * phi.tan() / r;
        VinhStateDerivative {
            dr_dt,
            dtheta_dt,
            dphi_dt,
            dv_dt,
            dgamma_dt,
            dpsi_dt,
        }
    }

    /// One explicit-Euler integration step.
    #[must_use]
    pub fn step_explicit_euler(&self, state: VinhState, dt_s: f64) -> VinhState {
        let d = self.derivative(state);
        VinhState {
            r_m: state.r_m + d.dr_dt * dt_s,
            theta_rad: state.theta_rad + d.dtheta_dt * dt_s,
            phi_rad: state.phi_rad + d.dphi_dt * dt_s,
            velocity_m_s: state.velocity_m_s + d.dv_dt * dt_s,
            flight_path_angle_rad: state.flight_path_angle_rad + d.dgamma_dt * dt_s,
            heading_rad: state.heading_rad + d.dpsi_dt * dt_s,
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp, clippy::missing_panics_doc, clippy::similar_names)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn entry_interface_default_validates() {
        EntryInterfaceBuilder::default().validate().unwrap();
    }

    #[test]
    fn entry_interface_rejects_zero_altitude() {
        let mut b = EntryInterfaceBuilder::default();
        b.entry_altitude_m = 0.0;
        assert!(matches!(
            b.validate(),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn allen_eggers_peak_decel_classical_value() {
        // Allen-Eggers classical example (Anderson 2019 §13.4):
        //   V_e = 7800 m/s, γ_e = 5°, β = 1/7000 m⁻¹
        //   ⇒ n_max ≈ V_e² · β · sin(γ_e) / (2 g₀ e)
        //          = 7800² · (1/7000) · sin(5°) / (2 · 9.80665 · 2.71828)
        //          ≈ 60_840_000 · 0.0001429 · 0.0872 / 53.32
        //          ≈ 14.2 g
        let ae = AllenEggers {
            rho_s_kg_m3: 1.225,
            beta_inv_m: 1.0 / 7_000.0,
            entry_velocity_m_s: 7_800.0,
            flight_path_angle_rad: 5.0_f64.to_radians(),
            ballistic_coefficient_m2_kg: 0.001,
        };
        let n = ae.peak_deceleration_g();
        // Tolerance ± 5 % on the classical analytic value.
        assert!((n - 14.2).abs() / 14.2 < 0.05, "n_max = {n} g");
    }

    #[test]
    fn allen_eggers_peak_altitude_is_finite_positive() {
        let ae = AllenEggers {
            rho_s_kg_m3: 1.225,
            beta_inv_m: 1.0 / 7_000.0,
            entry_velocity_m_s: 7_800.0,
            flight_path_angle_rad: 5.0_f64.to_radians(),
            ballistic_coefficient_m2_kg: 0.001,
        };
        let h = ae.peak_decel_altitude_m();
        assert!(h.is_finite() && h > 0.0, "h_max_decel = {h}");
        // Engineering expectation: 30-50 km for these parameters.
        assert!((20_000.0..=80_000.0).contains(&h), "h = {h}");
    }

    #[test]
    fn allen_eggers_velocity_decays_with_altitude() {
        let ae = AllenEggers {
            rho_s_kg_m3: 1.225,
            beta_inv_m: 1.0 / 7_000.0,
            entry_velocity_m_s: 7_800.0,
            flight_path_angle_rad: 5.0_f64.to_radians(),
            ballistic_coefficient_m2_kg: 0.001,
        };
        let v_top = ae.velocity_at_altitude_m_s(120_000.0);
        let v_bot = ae.velocity_at_altitude_m_s(30_000.0);
        assert!(v_top > v_bot, "v_top={v_top}, v_bot={v_bot}");
        // At very high altitude, V ≈ V_e.
        assert_relative_eq!(v_top, 7_800.0, max_relative = 0.01);
    }

    #[test]
    fn allen_eggers_rejects_zero_angle() {
        let ae = AllenEggers {
            rho_s_kg_m3: 1.225,
            beta_inv_m: 1.0 / 7_000.0,
            entry_velocity_m_s: 7_800.0,
            flight_path_angle_rad: 0.0,
            ballistic_coefficient_m2_kg: 0.001,
        };
        assert!(matches!(
            ae.validate(),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn vinh_derivative_dr_dt_matches_v_sin_gamma() {
        let v = Vinh::default();
        let state = VinhState {
            r_m: R_EARTH_M + 100_000.0,
            theta_rad: 0.0,
            phi_rad: 0.0,
            velocity_m_s: 7_000.0,
            flight_path_angle_rad: (-3.0_f64).to_radians(),
            heading_rad: 0.0,
        };
        let d = v.derivative(state);
        let expected = state.velocity_m_s * state.flight_path_angle_rad.sin();
        assert_relative_eq!(d.dr_dt, expected, max_relative = 1e-12);
    }

    #[test]
    fn vinh_explicit_euler_steps_state_in_descent() {
        let v = Vinh::default();
        let state = VinhState {
            r_m: R_EARTH_M + 100_000.0,
            theta_rad: 0.0,
            phi_rad: 0.0,
            velocity_m_s: 7_000.0,
            flight_path_angle_rad: (-3.0_f64).to_radians(),
            heading_rad: 0.0,
        };
        let s_next = v.step_explicit_euler(state, 1.0);
        assert!(s_next.r_m < state.r_m, "descending step should reduce r");
    }

    #[test]
    fn determinism_two_runs_byte_identical() {
        let v = Vinh::default();
        let state = VinhState {
            r_m: R_EARTH_M + 100_000.0,
            theta_rad: 0.0,
            phi_rad: 0.5,
            velocity_m_s: 7_500.0,
            flight_path_angle_rad: (-5.0_f64).to_radians(),
            heading_rad: 0.5,
        };
        let a = v.step_explicit_euler(state, 1.0);
        let b = v.step_explicit_euler(state, 1.0);
        assert_eq!(a.r_m.to_bits(), b.r_m.to_bits());
        assert_eq!(a.velocity_m_s.to_bits(), b.velocity_m_s.to_bits());
        assert_eq!(a.flight_path_angle_rad.to_bits(), b.flight_path_angle_rad.to_bits());
    }
}
