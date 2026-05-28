//! Live aerothermal driver wiring for scenario runs.
//!
//! The physics crate owns the stagnation, conduction, and ablation
//! models. This module owns runtime coupling: sample the scenario
//! atmosphere at the current trajectory state, evaluate heating,
//! advance optional material state, and expose diagnostics plus an
//! optional rigid-body mass-loss feedback handle.

use std::f64::consts::PI;
use std::sync::{Arc, Mutex};

use nalgebra::Vector3;
use openbmp_aero::{knudsen_number, mean_free_path_m};
use openbmp_aerothermal::{
    AerothermalContext, BackwallCondition, DepthResolvedCharringAblator, FayRiddell,
    HeatTransferModel, OneDThermalToy, SuttonGraves, ToyAblator, ToyMaterial, WallCatalysis,
};
use openbmp_core::{ModelId, ValidationStatus};
use openbmp_physics::AtmosphereModel;
use openbmp_scenario::{
    AerothermalAblationConfig, AerothermalBackwallConfig, AerothermalConfig,
    AerothermalThermalToyConfig, ScenarioDocument,
};
use openbmp_sim::{EnvironmentSample, ForceContext, ForceModel, ModelEvalError};
use openbmp_state::{PointMassState, RigidBodyState};

use crate::atmosphere::{RuntimeAtmosphere, build_document_runtime_atmosphere};
use crate::error::RunnerError;

const SIGMA_SB_W_M2_K4: f64 = 5.670_374_419e-8;
const MAX_LIVE_WALL_TEMPERATURE_K: f64 = 5000.0;

/// Shared mass-loss rate produced by the live aerothermal driver.
#[derive(Clone, Debug, Default)]
pub struct AerothermalMassFeedback {
    inner: Arc<Mutex<f64>>,
}

impl AerothermalMassFeedback {
    /// Current mass-loss rate in kg/s. Positive means mass is being
    /// removed from the vehicle.
    #[must_use]
    pub fn mass_loss_kg_s(&self) -> f64 {
        self.inner.lock().map_or(0.0, |guard| *guard)
    }

    fn set_mass_loss_kg_s(&self, rate_kg_s: f64) {
        if let Ok(mut guard) = self.inner.lock() {
            *guard = if rate_kg_s.is_finite() && rate_kg_s > 0.0 {
                rate_kg_s
            } else {
                0.0
            };
        }
    }
}

/// Live aerothermal telemetry sample at a trajectory state.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct LiveAerothermalOutput {
    /// Convective stagnation heat flux (W/m²).
    pub q_conv_w_m2: f64,
    /// Radiative stagnation heat flux (W/m²).
    pub q_rad_w_m2: f64,
    /// Adiabatic-wall enthalpy (J/kg).
    pub h_aw_j_kg: f64,
    /// Recovery temperature (K).
    pub recovery_temperature_k: f64,
    /// Stagnation/total temperature (K) used by live thermal
    /// diagnostics. For the currently wired engineering correlations
    /// this matches `recovery_temperature_k`.
    pub stagnation_temperature_k: f64,
    /// Knudsen number based on nose radius.
    pub knudsen: f64,
    /// Current wall/surface temperature (K).
    pub wall_temperature_k: f64,
    /// Current backwall temperature (K), or the wall temperature when
    /// no thermal toy is active.
    pub backwall_temperature_k: f64,
    /// Depth-resolved pyrolysis-front depth (m).
    pub recession_depth_m: f64,
    /// Pyrolysis gas mass flux (kg/(m²*s)).
    pub gas_mdot_kg_m2_s: f64,
    /// Vehicle-level mass-loss rate fed back to rigid mass dynamics
    /// when `aerothermal.ablation.feedback = "mass"`.
    pub mass_loss_kg_s: f64,
}

/// Shared latest aerothermal output for force-stack diagnostics and
/// telemetry row writers.
#[derive(Clone, Debug, Default)]
pub struct LiveAerothermalSink {
    inner: Arc<Mutex<LiveAerothermalOutput>>,
}

impl LiveAerothermalSink {
    #[must_use]
    fn new(output: LiveAerothermalOutput) -> Self {
        Self {
            inner: Arc::new(Mutex::new(output)),
        }
    }

    /// Most recent live aerothermal diagnostic sample.
    #[must_use]
    pub fn output(&self) -> LiveAerothermalOutput {
        match self.inner.lock() {
            Ok(guard) => *guard,
            Err(_) => LiveAerothermalOutput::default(),
        }
    }

    fn set_output(&self, output: LiveAerothermalOutput) {
        if let Ok(mut guard) = self.inner.lock() {
            *guard = output;
        }
    }
}

/// Zero-force force-stack hook for live stagnation heating,
/// conduction, and ablation diagnostics.
#[derive(Clone, Debug)]
pub struct StagnationHeatingForceAdapter {
    sink: LiveAerothermalSink,
    model_id: ModelId,
}

impl StagnationHeatingForceAdapter {
    /// Construct a zero-force diagnostic adapter backed by the live
    /// aerothermal sink owned by [`LiveAerothermalDriver`].
    #[must_use]
    pub const fn new(sink: LiveAerothermalSink, model_id: ModelId) -> Self {
        Self { sink, model_id }
    }

    /// Most recent output that this adapter exposes to telemetry.
    #[must_use]
    pub fn output(&self) -> LiveAerothermalOutput {
        self.sink.output()
    }

    fn validate_output(&self) -> Result<(), ModelEvalError> {
        let output = self.sink.output();
        let all_finite = [
            output.q_conv_w_m2,
            output.q_rad_w_m2,
            output.h_aw_j_kg,
            output.recovery_temperature_k,
            output.stagnation_temperature_k,
            output.knudsen,
            output.wall_temperature_k,
            output.backwall_temperature_k,
            output.recession_depth_m,
            output.gas_mdot_kg_m2_s,
            output.mass_loss_kg_s,
        ]
        .into_iter()
        .all(f64::is_finite);
        if all_finite {
            Ok(())
        } else {
            Err(ModelEvalError::NonFinite {
                model: self.model_id,
            })
        }
    }
}

impl ForceModel<PointMassState> for StagnationHeatingForceAdapter {
    fn force_n_eci(
        &self,
        _ctx: ForceContext<'_, PointMassState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        self.validate_output()?;
        Ok(Vector3::zeros())
    }

    fn supports_separated_body_propagation(&self) -> bool {
        true
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

impl ForceModel<RigidBodyState> for StagnationHeatingForceAdapter {
    fn force_n_eci(
        &self,
        _ctx: ForceContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        self.validate_output()?;
        Ok(Vector3::zeros())
    }

    fn supports_separated_body_propagation(&self) -> bool {
        true
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

/// Stateful live aerothermal driver.
#[derive(Clone, Debug)]
pub struct LiveAerothermalDriver {
    config: AerothermalConfig,
    atmosphere: RuntimeAtmosphere,
    thermal: Option<OneDThermalToy>,
    ablator: Option<DepthResolvedCharringAblator>,
    feedback_enabled: bool,
    mass_feedback: AerothermalMassFeedback,
    sink: LiveAerothermalSink,
    output: LiveAerothermalOutput,
}

impl LiveAerothermalDriver {
    /// Build the driver when `[aerothermal]` is declared.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when the scenario atmosphere or
    /// material-state models cannot be constructed.
    pub fn maybe_new(document: &ScenarioDocument) -> Result<Option<Self>, RunnerError> {
        let Some(config) = &document.aerothermal else {
            return Ok(None);
        };
        let atmosphere = build_document_runtime_atmosphere(document)?;
        let thermal = config
            .thermal_toy
            .as_ref()
            .map(build_thermal_toy)
            .transpose()?;
        let ablator = config.ablation.as_ref().map(build_ablator).transpose()?;
        let feedback_enabled = config
            .ablation
            .as_ref()
            .is_some_and(|ablation| ablation.feedback == "mass");
        let wall_temperature_k = thermal.as_ref().map_or(
            config.wall_temperature_k,
            OneDThermalToy::surface_temperature_k,
        );
        let backwall_temperature_k = thermal.as_ref().map_or(
            config.wall_temperature_k,
            OneDThermalToy::backwall_temperature_k,
        );
        let output = LiveAerothermalOutput {
            wall_temperature_k,
            backwall_temperature_k,
            ..LiveAerothermalOutput::default()
        };
        Ok(Some(Self {
            config: config.clone(),
            atmosphere,
            thermal,
            ablator,
            feedback_enabled,
            mass_feedback: AerothermalMassFeedback::default(),
            sink: LiveAerothermalSink::new(output),
            output,
        }))
    }

    /// Mass-feedback handle for rigid-body mass models.
    #[must_use]
    pub fn mass_feedback(&self) -> Option<AerothermalMassFeedback> {
        self.feedback_enabled.then(|| self.mass_feedback.clone())
    }

    /// Shared diagnostic sink for zero-force force-stack adapters.
    #[must_use]
    pub fn sink(&self) -> LiveAerothermalSink {
        self.sink.clone()
    }

    /// Most recent diagnostic output.
    #[must_use]
    pub const fn output(&self) -> &LiveAerothermalOutput {
        &self.output
    }

    /// Evaluate at a point-mass state.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] if atmosphere, heating, thermal, or
    /// ablation model evaluation fails.
    pub fn evaluate_point_mass(
        &mut self,
        state: &PointMassState,
        environment: &EnvironmentSample,
        dt_s: f64,
    ) -> Result<&LiveAerothermalOutput, RunnerError> {
        self.evaluate_common(
            state.position.vector.z,
            environment
                .air_relative_velocity_eci_m_s(state.velocity.vector)
                .norm(),
            state.time,
            dt_s,
        )
    }

    /// Evaluate at a rigid-body state.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] if atmosphere, heating, thermal, or
    /// ablation model evaluation fails.
    pub fn evaluate_rigid_body(
        &mut self,
        state: &RigidBodyState,
        environment: &EnvironmentSample,
        dt_s: f64,
    ) -> Result<&LiveAerothermalOutput, RunnerError> {
        self.evaluate_common(
            state.position.vector.z,
            environment
                .air_relative_velocity_eci_m_s(state.velocity.vector)
                .norm(),
            state.time,
            dt_s,
        )
    }

    fn evaluate_common(
        &mut self,
        altitude_m: f64,
        airspeed_m_s: f64,
        time: openbmp_core::SimTime,
        dt_s: f64,
    ) -> Result<&LiveAerothermalOutput, RunnerError> {
        let altitude_m = altitude_m.max(0.0);
        let sample = self.atmosphere.sample(altitude_m, time)?;
        let wall_temperature_k = self.thermal.as_ref().map_or(
            self.config.wall_temperature_k,
            OneDThermalToy::surface_temperature_k,
        );
        let backwall_temperature_k = self
            .thermal
            .as_ref()
            .map_or(wall_temperature_k, OneDThermalToy::backwall_temperature_k);
        let mean_free_path = mean_free_path_m(sample.temperature_k, sample.pressure_pa);
        let knudsen = knudsen_number(mean_free_path, self.config.nose_radius_m);

        if airspeed_m_s <= 0.0 || sample.speed_of_sound_m_s <= 0.0 || sample.density_kg_m3 <= 0.0 {
            self.mass_feedback.set_mass_loss_kg_s(0.0);
            self.output = LiveAerothermalOutput {
                stagnation_temperature_k: sample.temperature_k,
                knudsen,
                wall_temperature_k,
                backwall_temperature_k,
                ..LiveAerothermalOutput::default()
            };
            self.sink.set_output(self.output);
            return Ok(&self.output);
        }

        let mach = airspeed_m_s / sample.speed_of_sound_m_s;
        if self.config.stagnation_kind == "fay_riddell" && mach <= 1.0 {
            self.mass_feedback.set_mass_loss_kg_s(0.0);
            self.output = LiveAerothermalOutput {
                stagnation_temperature_k: sample.temperature_k,
                knudsen,
                wall_temperature_k,
                backwall_temperature_k,
                ..LiveAerothermalOutput::default()
            };
            self.sink.set_output(self.output);
            return Ok(&self.output);
        }

        let ctx = AerothermalContext {
            freestream: sample,
            airspeed_m_s,
            mach,
            nose_radius_m: self.config.nose_radius_m,
            wall_temperature_k,
            wall_catalysis: wall_catalysis(&self.config.wall_catalysis),
        };
        let heating = match self.config.stagnation_kind.as_str() {
            "sutton_graves" => {
                SuttonGraves::default().stagnation_with_wall_enthalpy_correction(&ctx)?
            }
            "fay_riddell" => {
                let fay = FayRiddell {
                    lewis_number: self
                        .config
                        .fay_riddell
                        .as_ref()
                        .map_or(1.0, |cfg| cfg.lewis_number),
                    h_dissociation_j_kg: 0.0,
                };
                fay.stagnation(&ctx)?
            }
            other => {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!("unsupported aerothermal.stagnation_kind `{other}`"),
                });
            }
        };

        let q_in_w_m2 = (heating.q_conv_w_m2 + heating.q_rad_w_m2).max(0.0);
        if let Some(thermal) = &mut self.thermal
            && dt_s > 0.0
        {
            thermal.step(dt_s, q_in_w_m2)?;
        }

        let wall_temperature_k = self
            .thermal
            .as_ref()
            .map_or(wall_temperature_k, OneDThermalToy::surface_temperature_k);
        if wall_temperature_k > MAX_LIVE_WALL_TEMPERATURE_K {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "live aerothermal wall temperature exceeded {MAX_LIVE_WALL_TEMPERATURE_K} K"
                ),
            });
        }
        let backwall_temperature_k = self.thermal.as_ref().map_or(
            backwall_temperature_k,
            OneDThermalToy::backwall_temperature_k,
        );

        let mut recession_depth_m = self.ablator.as_ref().map_or(0.0, |a| a.front_depth_m);
        let mut gas_mdot_kg_m2_s = 0.0;
        if let Some(ablator) = &mut self.ablator {
            let emissivity = ablator.char_material.surface_emissivity;
            let reradiation_w_m2 = SIGMA_SB_W_M2_K4 * emissivity * wall_temperature_k.powi(4);
            let q_net_w_m2 = (q_in_w_m2 - reradiation_w_m2).max(0.0);
            if dt_s > 0.0 {
                let update = ablator.advance_front(q_net_w_m2, dt_s)?;
                recession_depth_m = update.front_depth_m;
                gas_mdot_kg_m2_s = update.gas_mdot_kg_m2_s;
            }
        }
        let reference_area_m2 = PI * self.config.nose_radius_m * self.config.nose_radius_m;
        let mass_loss_kg_s = if self.feedback_enabled {
            gas_mdot_kg_m2_s * reference_area_m2
        } else {
            0.0
        };
        self.mass_feedback.set_mass_loss_kg_s(mass_loss_kg_s);
        self.output = LiveAerothermalOutput {
            q_conv_w_m2: heating.q_conv_w_m2,
            q_rad_w_m2: heating.q_rad_w_m2,
            h_aw_j_kg: heating.h_aw_j_kg,
            recovery_temperature_k: heating.recovery_temperature_k,
            stagnation_temperature_k: heating.recovery_temperature_k,
            knudsen,
            wall_temperature_k,
            backwall_temperature_k,
            recession_depth_m,
            gas_mdot_kg_m2_s,
            mass_loss_kg_s,
        };
        self.sink.set_output(self.output);
        Ok(&self.output)
    }
}

fn build_thermal_toy(config: &AerothermalThermalToyConfig) -> Result<OneDThermalToy, RunnerError> {
    let n_nodes =
        usize::try_from(config.n_nodes).map_err(|_| RunnerError::UnsupportedScenario {
            what: "aerothermal.thermal_toy.n_nodes overflows usize".to_owned(),
        })?;
    Ok(OneDThermalToy::with_uniform_temperature(
        thermal_material(&config.material),
        config.thickness_m,
        n_nodes,
        backwall_condition(config.backwall.as_ref()),
        config.initial_temperature_k,
    )?)
}

fn build_ablator(
    config: &AerothermalAblationConfig,
) -> Result<DepthResolvedCharringAblator, RunnerError> {
    let n_nodes =
        usize::try_from(config.n_nodes).map_err(|_| RunnerError::UnsupportedScenario {
            what: "aerothermal.ablation.n_nodes overflows usize".to_owned(),
        })?;
    Ok(DepthResolvedCharringAblator::new_uniform(
        ablator_material(&config.virgin_material),
        ablator_material(&config.char_material),
        config.thickness_m,
        n_nodes,
        config.pyrolysis_enthalpy_j_kg,
        config.gas_yield_fraction,
    )?)
}

fn wall_catalysis(kind: &str) -> WallCatalysis {
    match kind {
        "non_catalytic" => WallCatalysis::NonCatalytic,
        _ => WallCatalysis::FullyCatalytic,
    }
}

fn backwall_condition(config: Option<&AerothermalBackwallConfig>) -> BackwallCondition {
    let Some(config) = config else {
        return BackwallCondition::Adiabatic;
    };
    match config.kind.as_str() {
        "prescribed" => BackwallCondition::PrescribedTemperature {
            t_k: config.t_k.unwrap_or(300.0),
        },
        "convective" => BackwallCondition::Convective {
            h_w_m2_k: config.h_w_m2_k.unwrap_or(0.0),
            t_inf_k: config.t_inf_k.unwrap_or(300.0),
        },
        _ => BackwallCondition::Adiabatic,
    }
}

fn thermal_material(name: &str) -> ToyMaterial {
    match name {
        "textbook_avcoat_like" => ToyMaterial {
            name: "textbook-avcoat-like",
            density_kg_m3: 512.0,
            specific_heat_j_kg_k: 1250.0,
            thermal_conductivity_w_m_k: 0.16,
            emissivity: 0.88,
        },
        _ => ToyMaterial {
            name: "textbook-pica-like",
            density_kg_m3: 270.0,
            specific_heat_j_kg_k: 1200.0,
            thermal_conductivity_w_m_k: 0.08,
            emissivity: 0.85,
        },
    }
}

fn ablator_material(name: &str) -> ToyAblator {
    match name {
        "textbook_graphite" => ToyAblator::graphite_toy(),
        "textbook_avcoat_like" => ToyAblator {
            name: "textbook-avcoat-like",
            density_kg_m3: 512.0,
            specific_heat_j_kg_k: 1250.0,
            thermal_conductivity_w_m_k: 0.16,
            heat_of_ablation_j_kg: 2.0e7,
            vaporisation_temperature_k: 1250.0,
            surface_emissivity: 0.88,
        },
        "textbook_char" => ToyAblator {
            name: "textbook-char",
            density_kg_m3: 320.0,
            specific_heat_j_kg_k: 1100.0,
            thermal_conductivity_w_m_k: 0.25,
            heat_of_ablation_j_kg: 1.0e7,
            vaporisation_temperature_k: 1800.0,
            surface_emissivity: 0.86,
        },
        _ => ToyAblator {
            name: "textbook-pica-like",
            density_kg_m3: 270.0,
            specific_heat_j_kg_k: 1200.0,
            thermal_conductivity_w_m_k: 0.08,
            heat_of_ablation_j_kg: 2.6e7,
            vaporisation_temperature_k: 1200.0,
            surface_emissivity: 0.85,
        },
    }
}
