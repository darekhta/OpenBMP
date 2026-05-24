//! Phase-6.9 1-D thermal-conduction toy.
//!
//! Explicit forward-time / centred-space (FTCS) integrator for the
//! 1-D heat-conduction equation through a slab of textbook material
//! under prescribed surface heat flux. Fourier-number stability is
//! enforced (Fo < 0.5) — out-of-bound steps fail closed.

use crate::error::AerothermalError;

/// Material properties for the 1-D thermal toy.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ToyMaterial {
    /// Textbook material name (e.g. "generic-ablator-1").
    pub name: &'static str,
    /// Mass density (kg/m³).
    pub density_kg_m3: f64,
    /// Specific heat capacity (J/(kg·K)).
    pub specific_heat_j_kg_k: f64,
    /// Thermal conductivity (W/(m·K)).
    pub thermal_conductivity_w_m_k: f64,
    /// Surface emissivity (0..=1).
    pub emissivity: f64,
}

/// Backwall boundary condition for the 1-D thermal-toy slab.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BackwallCondition {
    /// `∂T/∂x = 0` at the backwall (insulated).
    Adiabatic,
    /// Prescribed temperature at the backwall.
    PrescribedTemperature {
        /// Backwall temperature (K).
        t_k: f64,
    },
    /// Convective backwall: `−k · ∂T/∂x = h · (T_back − T_∞)`.
    Convective {
        /// Heat-transfer coefficient (W/(m²·K)).
        h_w_m2_k: f64,
        /// Backwall sink temperature (K).
        t_inf_k: f64,
    },
}

/// 1-D thermal-conduction toy slab.
///
/// Internal state: temperature at each interior node + the two
/// boundary nodes (surface and backwall). The default initialiser
/// fills every node at the supplied initial temperature.
#[derive(Clone, Debug, PartialEq)]
pub struct OneDThermalToy {
    /// Slab material.
    pub material: ToyMaterial,
    /// Slab thickness (m).
    pub thickness_m: f64,
    /// Number of interior nodes used for finite-difference integration.
    pub n_nodes: usize,
    /// Backwall boundary condition.
    pub backwall: BackwallCondition,
    /// Per-node temperature (K). Length = `n_nodes + 2`
    /// (surface node + interior + backwall).
    pub temperature_k: Vec<f64>,
}

impl OneDThermalToy {
    /// Construct with uniform initial temperature.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError::InvalidParameter`] on
    /// pathological dimensions or non-physical material.
    pub fn with_uniform_temperature(
        material: ToyMaterial,
        thickness_m: f64,
        n_nodes: usize,
        backwall: BackwallCondition,
        initial_temperature_k: f64,
    ) -> Result<Self, AerothermalError> {
        if !thickness_m.is_finite() || thickness_m <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "thickness must be > 0",
            });
        }
        if n_nodes == 0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "n_nodes must be ≥ 1",
            });
        }
        if !(material.density_kg_m3 > 0.0
            && material.specific_heat_j_kg_k > 0.0
            && material.thermal_conductivity_w_m_k > 0.0)
        {
            return Err(AerothermalError::InvalidParameter {
                reason: "material density / cp / k must be > 0",
            });
        }
        if !initial_temperature_k.is_finite() || initial_temperature_k <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "initial temperature must be finite and > 0",
            });
        }
        Ok(Self {
            material,
            thickness_m,
            n_nodes,
            backwall,
            temperature_k: vec![initial_temperature_k; n_nodes + 2],
        })
    }

    /// Thermal diffusivity `α = k / (ρ · c_p)` (m²/s).
    #[must_use]
    pub fn diffusivity_m2_s(&self) -> f64 {
        self.material.thermal_conductivity_w_m_k
            / (self.material.density_kg_m3 * self.material.specific_heat_j_kg_k)
    }

    /// Node spacing `dx = L / (N+1)` (m).
    #[must_use]
    pub fn dx_m(&self) -> f64 {
        self.thickness_m / ((self.n_nodes + 1) as f64)
    }

    /// Maximum stable explicit-FTCS step size for this slab
    /// (`Fo = α · dt / dx² < 0.5`).
    #[must_use]
    pub fn max_stable_dt_s(&self) -> f64 {
        let dx = self.dx_m();
        0.5 * dx * dx / self.diffusivity_m2_s()
    }

    /// Advance one explicit-FTCS step with the given surface heat
    /// flux applied at node 0.
    ///
    /// # Errors
    ///
    /// Returns [`AerothermalError::OutOfEnvelope`] when the supplied
    /// step exceeds the Fourier-number stability bound.
    pub fn step(&mut self, dt_s: f64, q_surface_w_m2: f64) -> Result<(), AerothermalError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(AerothermalError::InvalidParameter {
                reason: "dt must be > 0",
            });
        }
        if !q_surface_w_m2.is_finite() {
            return Err(AerothermalError::NonFinite {
                reason: "surface heat flux non-finite",
            });
        }
        let dx = self.dx_m();
        let alpha = self.diffusivity_m2_s();
        let fo = alpha * dt_s / (dx * dx);
        if fo >= 0.5 {
            return Err(AerothermalError::OutOfEnvelope {
                reason: "1-D thermal toy: Fo ≥ 0.5 (stability bound)",
            });
        }
        let k = self.material.thermal_conductivity_w_m_k;
        let n_total = self.temperature_k.len();
        let prev = self.temperature_k.clone();
        // Interior nodes: forward-time / centred-space.
        for i in 1..(n_total - 1) {
            let lap = prev[i + 1] - 2.0 * prev[i] + prev[i - 1];
            self.temperature_k[i] = prev[i] + fo * lap;
        }
        // Surface node: −k · ∂T/∂x = q_surface ⇒ ghost node
        // approximation.
        //   T_-1 ≈ T_1 + 2 · dx · q / k
        let ghost = prev[1] + 2.0 * dx * q_surface_w_m2 / k;
        let lap_surface = ghost - 2.0 * prev[0] + prev[1];
        self.temperature_k[0] = prev[0] + fo * lap_surface;
        // Backwall node.
        let backwall = match self.backwall {
            BackwallCondition::Adiabatic => {
                // Ghost mirrors interior: T_{N+1} = T_{N-1}.
                let lap_back = prev[n_total - 2] - 2.0 * prev[n_total - 1] + prev[n_total - 2];
                prev[n_total - 1] + fo * lap_back
            }
            BackwallCondition::PrescribedTemperature { t_k } => t_k,
            BackwallCondition::Convective { h_w_m2_k, t_inf_k } => {
                // −k · ∂T/∂x = h · (T_back − T_∞)
                let ghost = prev[n_total - 2] - 2.0 * dx * h_w_m2_k * (prev[n_total - 1] - t_inf_k)
                    / k;
                let lap_back = ghost - 2.0 * prev[n_total - 1] + prev[n_total - 2];
                prev[n_total - 1] + fo * lap_back
            }
        };
        self.temperature_k[n_total - 1] = backwall;
        Ok(())
    }

    /// Surface temperature (K).
    #[must_use]
    pub fn surface_temperature_k(&self) -> f64 {
        self.temperature_k[0]
    }

    /// Backwall temperature (K).
    #[must_use]
    pub fn backwall_temperature_k(&self) -> f64 {
        *self.temperature_k.last().expect("temperature array non-empty by construction")
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp, clippy::missing_panics_doc, clippy::similar_names)]
mod tests {
    use super::*;

    fn mat() -> ToyMaterial {
        ToyMaterial {
            name: "generic-insulator-1",
            density_kg_m3: 1500.0,
            specific_heat_j_kg_k: 1200.0,
            thermal_conductivity_w_m_k: 0.5,
            emissivity: 0.85,
        }
    }

    #[test]
    fn constructor_validates_geometry() {
        assert!(matches!(
            OneDThermalToy::with_uniform_temperature(
                mat(),
                0.0,
                10,
                BackwallCondition::Adiabatic,
                300.0
            ),
            Err(AerothermalError::InvalidParameter { .. })
        ));
        assert!(matches!(
            OneDThermalToy::with_uniform_temperature(
                mat(),
                0.01,
                0,
                BackwallCondition::Adiabatic,
                300.0
            ),
            Err(AerothermalError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn diffusivity_is_finite_positive() {
        let toy =
            OneDThermalToy::with_uniform_temperature(mat(), 0.01, 10, BackwallCondition::Adiabatic, 300.0)
                .unwrap();
        let alpha = toy.diffusivity_m2_s();
        assert!(alpha > 0.0 && alpha.is_finite());
    }

    #[test]
    fn rejects_fourier_violating_step() {
        let mut toy =
            OneDThermalToy::with_uniform_temperature(mat(), 0.01, 10, BackwallCondition::Adiabatic, 300.0)
                .unwrap();
        let too_big = toy.max_stable_dt_s() * 2.0;
        assert!(matches!(
            toy.step(too_big, 1.0e5),
            Err(AerothermalError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn accepts_stable_step_and_heats_surface() {
        let mut toy = OneDThermalToy::with_uniform_temperature(
            mat(),
            0.01,
            20,
            BackwallCondition::PrescribedTemperature { t_k: 300.0 },
            300.0,
        )
        .unwrap();
        let dt = 0.4 * toy.max_stable_dt_s();
        // Apply 100 kW/m² for 100 steps — surface should heat up.
        for _ in 0..100 {
            toy.step(dt, 1.0e5).unwrap();
        }
        assert!(toy.surface_temperature_k() > 300.0);
        // Backwall is pinned at 300 K.
        assert!((toy.backwall_temperature_k() - 300.0).abs() < 1.0e-6);
    }

    #[test]
    fn adiabatic_backwall_eventually_heats() {
        let mut toy = OneDThermalToy::with_uniform_temperature(
            mat(),
            0.01,
            20,
            BackwallCondition::Adiabatic,
            300.0,
        )
        .unwrap();
        let dt = 0.4 * toy.max_stable_dt_s();
        for _ in 0..5000 {
            toy.step(dt, 1.0e5).unwrap();
        }
        // With adiabatic backwall and sustained heating, backwall
        // temperature must rise above the initial 300 K.
        assert!(toy.backwall_temperature_k() > 300.0);
    }

    #[test]
    fn surface_temperature_is_monotone_under_constant_heating() {
        let mut toy = OneDThermalToy::with_uniform_temperature(
            mat(),
            0.01,
            20,
            BackwallCondition::Adiabatic,
            300.0,
        )
        .unwrap();
        let dt = 0.4 * toy.max_stable_dt_s();
        let mut prev = toy.surface_temperature_k();
        for _ in 0..100 {
            toy.step(dt, 1.0e5).unwrap();
            let now = toy.surface_temperature_k();
            assert!(now >= prev, "surface temperature should not decrease");
            prev = now;
        }
    }

    #[test]
    fn determinism_two_runs_byte_identical() {
        let make = || -> OneDThermalToy {
            OneDThermalToy::with_uniform_temperature(
                mat(),
                0.01,
                20,
                BackwallCondition::Adiabatic,
                300.0,
            )
            .unwrap()
        };
        let mut a = make();
        let mut b = make();
        let dt = 0.4 * a.max_stable_dt_s();
        for _ in 0..50 {
            a.step(dt, 1.0e5).unwrap();
            b.step(dt, 1.0e5).unwrap();
        }
        for (ai, bi) in a.temperature_k.iter().zip(b.temperature_k.iter()) {
            assert_eq!(ai.to_bits(), bi.to_bits());
        }
    }
}
