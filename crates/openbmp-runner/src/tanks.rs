//! Runner-side tank rack.
//!
//! The rack owns a `BTreeMap<TankId, openbmp_vehicle::Tank>` resolved
//! from the scenario's `[[vehicle.assembly.tanks]]` block, plus the
//! construction-time `dt` that's passed to each tank's `step(...)`
//! call and a cached pair `(specific_force_body, omega_body)` from
//! the prior kernel step's solution.
//!
//! Each kernel base tick the runner:
//!
//! 1. Drains every tank by its scenario-declared
//!    `drain_rate_kg_per_s` (drain is decoupled from
//!    engine-cluster mdot).
//! 2. Steps every tank using the **prior step's** cached
//!    `(specific_force_body, omega_body)`. The runner caches the
//!    freshly computed drivers from the step that just completed via
//!    [`TankRack::update_drivers`]. The first step uses zeros.
//! 3. Packs the resulting `mass_contribution()` and
//!    `reaction_body()` observations into a `BTreeMap<TankId,
//!    TankSnapshot>` via [`TankRack::snapshot_map`] and pushes it
//!    into the kernel via `set_tank_snapshot(...)`.
//!
//! The rack leaves the kernel's force / moment / mass evaluation
//! untouched when it is empty — legacy scenarios produce
//! byte-identical Parquet because the runner short-circuits every
//! rack-related operation on `is_empty()`.
//!
//! # Determinism
//!
//! - Tanks are stored in a `BTreeMap<TankId, Tank>` (defeats macOS
//!   `SipHash` randomisation); iteration is in `TankId` order which
//!   for `from_path`-derived ids is deterministic across reruns.
//! - The kernel-pushed snapshot map is also `BTreeMap<TankId,
//!   TankSnapshot>`.
//! - One-step lag on `(specific_force_body, omega_body)` is the only
//!   defensible bit-stable solution to the circular dependency
//!   between tank reaction force / mass contribution and the
//!   kernel's per-step force / mass evaluation. Initial step: zeros.

use std::collections::BTreeMap;

use nalgebra::Vector3;
use openbmp_core::{Duration, TankId};
use openbmp_fc::topics::PropellantSafingCommand;
use openbmp_feedsystem::{
    FiniteVolumePressurizationConfig, FiniteVolumePressurizationModel,
    FiniteVolumePressurizationSnapshot, FiniteVolumePressurizationState,
};
use openbmp_scenario::{
    FreefallRestoringConfig, InitialSloshConfig, MovingMassKindConfig, PropellantSpecConfig,
    ScenarioDocument, TankConfig, TankGeometryConfig, TankSafeDumpBackpressureRampConfig,
    TankSafeDumpBackpressureSourceConfig, TankSafeDumpVentThermalModelConfig,
};
use openbmp_sim::TankSnapshot;
use openbmp_vehicle::{
    AbramsonMultiMass, BaffleModel, BaffledPendulum, EquivalentPendulum, EquivalentSpringMass,
    MovingMassModel, PropellantSpec, PropellantTankState, RigidLiquid, Tank, TankGeometry,
};

use crate::error::RunnerError;

/// Runner-side tank rack. Built once per `openbmp run` invocation;
/// consumed by the per-step kernel loop.
pub struct TankRack {
    tanks: BTreeMap<TankId, Tank>,
    dt: Duration,
    /// Elapsed rack time used for deterministic SAFE dump schedules.
    elapsed_s: f64,
    /// Latest runner environment pressure for ambient SAFE dump backpressure.
    safe_dump_ambient_backpressure_pa: Option<f64>,
    /// Per-tank scenario-declared drain rates (kg/s). Drain is
    /// decoupled from engines.
    drain_rates_kg_per_s: BTreeMap<TankId, f64>,
    /// Per-tank scenario-declared SAFE dump rates (kg/s). This is a
    /// separate dump path from ordinary drains.
    safe_dump_rates_kg_per_s: BTreeMap<TankId, f64>,
    /// Per-tank pressure-coupled SAFE dump orifices.
    safe_dump_orifices: BTreeMap<TankId, SafeDumpOrificeRuntime>,
    /// Per-tank choked ullage-gas SAFE vents.
    safe_dump_vents: BTreeMap<TankId, SafeDumpVentRuntime>,
    /// Per-tank finite-volume pressurization hardware substrates.
    pressurization: BTreeMap<TankId, PressurizationRuntime>,
    /// Per-tank engine-coupled drain rates (kg/s) produced by the
    /// propellant budget from the prior engine snapshot.
    propellant_budget_drain_rates_kg_per_s: BTreeMap<TankId, f64>,
    /// Per-tank RCS feed drain rates (kg/s) produced by the effector rack.
    rcs_feed_drain_rates_kg_per_s: BTreeMap<TankId, f64>,
    /// Per-tank realized mass rates from the most recent tank step.
    latest_mass_rates_kg_per_s: BTreeMap<TankId, f64>,
    /// `false` after SAFE asks tank/feed drains to be disabled.
    drain_paths_enabled: bool,
    /// `true` after SAFE asks configured propellant dump paths to open.
    propellant_dump_active: bool,
    /// `false` after SAFE asks pressurization to be disabled.
    pressurization_enabled: bool,
    /// Cached `(specific_force_body_m_s2, omega_body_rad_s)` from the
    /// prior kernel step. Initialised to zeros at construction.
    last_drivers: (Vector3<f64>, Vector3<f64>),
}

impl std::fmt::Debug for TankRack {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TankRack")
            .field("tank_count", &self.tanks.len())
            .field("dt", &self.dt)
            .field("elapsed_s", &self.elapsed_s)
            .field(
                "safe_dump_ambient_backpressure_pa",
                &self.safe_dump_ambient_backpressure_pa,
            )
            .field("drain_rates_kg_per_s", &self.drain_rates_kg_per_s)
            .field("safe_dump_rates_kg_per_s", &self.safe_dump_rates_kg_per_s)
            .field("safe_dump_orifices", &self.safe_dump_orifices)
            .field("safe_dump_vents", &self.safe_dump_vents)
            .field("pressurization", &self.pressurization)
            .field(
                "propellant_budget_drain_rates_kg_per_s",
                &self.propellant_budget_drain_rates_kg_per_s,
            )
            .field(
                "rcs_feed_drain_rates_kg_per_s",
                &self.rcs_feed_drain_rates_kg_per_s,
            )
            .field(
                "latest_mass_rates_kg_per_s",
                &self.latest_mass_rates_kg_per_s,
            )
            .field("drain_paths_enabled", &self.drain_paths_enabled)
            .field("propellant_dump_active", &self.propellant_dump_active)
            .field("pressurization_enabled", &self.pressurization_enabled)
            .field("last_drivers", &self.last_drivers)
            .finish()
    }
}

impl TankRack {
    /// Build the rack from a parsed scenario document. Returns an
    /// empty rack when no `[[vehicle.assembly.tanks]]` block is
    /// declared.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Tank`] when a tank config fails one of
    /// the moving-mass model constructors (invalid geometry,
    /// damping, fill fraction, etc.).
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let dt = Duration::from_seconds(document.time.dt_s);
        let mut tanks: BTreeMap<TankId, Tank> = BTreeMap::new();
        let mut drain_rates: BTreeMap<TankId, f64> = BTreeMap::new();
        let mut safe_dump_rates: BTreeMap<TankId, f64> = BTreeMap::new();
        let mut safe_dump_orifices: BTreeMap<TankId, SafeDumpOrificeRuntime> = BTreeMap::new();
        let mut safe_dump_vents: BTreeMap<TankId, SafeDumpVentRuntime> = BTreeMap::new();
        let mut pressurization: BTreeMap<TankId, PressurizationRuntime> = BTreeMap::new();

        for config in &document.vehicle.assembly.tanks {
            let (id, tank) = build_tank(config)?;
            drain_rates.insert(id, config.drain_rate_kg_per_s.unwrap_or(0.0));
            safe_dump_rates.insert(id, config.safe_dump_rate_kg_per_s.unwrap_or(0.0));
            if let Some(orifice) = safe_dump_orifice_runtime(config) {
                safe_dump_orifices.insert(id, orifice);
            }
            if let Some(vent) = safe_dump_vent_runtime(config) {
                safe_dump_vents.insert(id, vent);
            }
            if let Some(runtime) =
                pressurization_runtime(config).map_err(|err| RunnerError::Tank {
                    field: format!("vehicle.assembly.tanks.{id}.pressurization", id = config.id),
                    reason: err,
                })?
            {
                pressurization.insert(id, runtime);
            }
            if tanks.insert(id, tank).is_some() {
                return Err(RunnerError::Tank {
                    field: format!("vehicle.assembly.tanks.{id}", id = config.id),
                    reason: "duplicate tank id (collision in fnv1a-64 hash)".to_owned(),
                });
            }
        }

        Ok(Self {
            tanks,
            dt,
            elapsed_s: 0.0,
            safe_dump_ambient_backpressure_pa: None,
            drain_rates_kg_per_s: drain_rates,
            safe_dump_rates_kg_per_s: safe_dump_rates,
            safe_dump_orifices,
            safe_dump_vents,
            pressurization,
            propellant_budget_drain_rates_kg_per_s: BTreeMap::new(),
            rcs_feed_drain_rates_kg_per_s: BTreeMap::new(),
            latest_mass_rates_kg_per_s: BTreeMap::new(),
            drain_paths_enabled: true,
            propellant_dump_active: false,
            pressurization_enabled: true,
            last_drivers: (Vector3::zeros(), Vector3::zeros()),
        })
    }

    /// `true` when the rack carries no tanks. The runner gates
    /// every per-step rack operation on this so legacy scenarios
    /// never touch the rack code path.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tanks.is_empty()
    }

    /// Number of tanks in the rack.
    #[must_use]
    pub fn len(&self) -> usize {
        self.tanks.len()
    }

    /// Tank ids in `BTreeMap` (sorted) order.
    #[must_use]
    pub fn tank_ids(&self) -> Vec<TankId> {
        self.tanks.keys().copied().collect()
    }

    /// Replace the cached prior-step `(specific_force_body, omega_body)`.
    /// The runner calls this with the freshly-computed body-frame
    /// translational acceleration and angular rate after the kernel
    /// step that just completed; the next [`Self::step`] uses these
    /// as the slosh-dynamics drivers.
    pub fn update_drivers(
        &mut self,
        accel_body_m_s2: Vector3<f64>,
        omega_body_rad_s: Vector3<f64>,
    ) {
        self.last_drivers = (accel_body_m_s2, omega_body_rad_s);
    }

    /// Replace engine-coupled drain rates for the next
    /// [`Self::step`]. Scenario-declared drain remains additive.
    pub fn set_propellant_budget_drain_rates(&mut self, rates: BTreeMap<TankId, f64>) {
        self.propellant_budget_drain_rates_kg_per_s = rates;
    }

    /// Replace RCS feed-coupled drain rates for the next [`Self::step`].
    /// Scenario-declared and engine-budget drain remain additive.
    pub fn set_rcs_feed_drain_rates(&mut self, rates: BTreeMap<TankId, f64>) {
        self.rcs_feed_drain_rates_kg_per_s = rates;
    }

    /// `true` when at least one SAFE dump path needs runner ambient pressure.
    #[must_use]
    pub fn uses_safe_dump_ambient_backpressure(&self) -> bool {
        self.safe_dump_orifices.values().any(|orifice| {
            orifice.backpressure_source == TankSafeDumpBackpressureSourceConfig::AmbientAtmosphere
        }) || self.safe_dump_vents.values().any(|vent| {
            vent.backpressure_source == TankSafeDumpBackpressureSourceConfig::AmbientAtmosphere
        })
    }

    /// Set current ambient pressure used by SAFE dump paths whose
    /// `backpressure_source` is `ambient_atmosphere`.
    pub fn set_safe_dump_ambient_backpressure_pa(
        &mut self,
        pressure_pa: f64,
    ) -> Result<(), RunnerError> {
        if !pressure_pa.is_finite() || pressure_pa < 0.0 {
            return Err(RunnerError::Tank {
                field: "environment.atmosphere_pressure_pa".to_owned(),
                reason: "SAFE dump ambient backpressure must be finite and non-negative".to_owned(),
            });
        }
        self.safe_dump_ambient_backpressure_pa = Some(pressure_pa);
        Ok(())
    }

    /// Latch SAFE tank/feed drain isolation.
    pub fn apply_propellant_safing_command(&mut self, command: Option<PropellantSafingCommand>) {
        let Some(command) = command else {
            return;
        };
        if command.safed && !command.drain_enabled {
            self.drain_paths_enabled = false;
        }
        if command.safed && command.dump_enabled {
            self.propellant_dump_active = true;
        }
        if command.safed && !command.pressurization_enabled {
            self.pressurization_enabled = false;
        }
    }

    /// Live tank states consumed by the vehicle-side propellant
    /// budget.
    #[must_use]
    pub fn propellant_tank_states(
        &self,
        document: &ScenarioDocument,
    ) -> BTreeMap<TankId, PropellantTankState> {
        let mut out = BTreeMap::new();
        for config in &document.vehicle.assembly.tanks {
            let id = TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = config.id));
            if let Some(tank) = self.tanks.get(&id) {
                out.insert(
                    id,
                    PropellantTankState {
                        fluid_remaining_kg: tank.fluid_remaining_kg(),
                        initial_fluid_mass_kg: tank.initial_fluid_mass_kg(),
                        volume_m3: tank.geometry().volume_m3(),
                        density_kg_m3: tank.propellant().density_kg_m3,
                        has_ullage: config.ullage.is_some(),
                        ullage_gamma: config.ullage.map_or(1.4, |ullage| ullage.gas_gamma),
                    },
                );
            }
        }
        out
    }

    /// Drain every tank by its scenario-declared rate, then advance
    /// every tank's slosh state using the cached `(accel, omega)`
    /// from the prior step (one-step lag).
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Tank`] when a tank's `drain` or `step`
    /// rejects (non-finite rate, non-finite drivers).
    pub fn step(&mut self) -> Result<(), RunnerError> {
        let (accel, omega) = self.last_drivers;
        let elapsed_s = self.elapsed_s;
        let ambient_backpressure_pa = self.safe_dump_ambient_backpressure_pa;
        self.latest_mass_rates_kg_per_s.clear();
        for (id, tank) in &mut self.tanks {
            let drain_rate = if self.drain_paths_enabled {
                self.drain_rates_kg_per_s.get(id).copied().unwrap_or(0.0)
                    + self
                        .propellant_budget_drain_rates_kg_per_s
                        .get(id)
                        .copied()
                        .unwrap_or(0.0)
                    + self
                        .rcs_feed_drain_rates_kg_per_s
                        .get(id)
                        .copied()
                        .unwrap_or(0.0)
            } else {
                0.0
            };
            let dump_rate = if self.propellant_dump_active {
                let fixed_rate = self
                    .safe_dump_rates_kg_per_s
                    .get(id)
                    .copied()
                    .unwrap_or(0.0);
                let vented_pressure_pa = match self.safe_dump_vents.get(id) {
                    Some(vent) => {
                        Some(vent.pressure_pa(tank).map_err(|err| RunnerError::Tank {
                            field: format!(
                                "vehicle.assembly.tanks.{id_v}.safe_dump_vent",
                                id_v = id.value()
                            ),
                            reason: err,
                        })?)
                    }
                    None => None,
                };
                let pressurized_pressure_pa =
                    if self.pressurization_enabled {
                        match self.pressurization.get(id) {
                            Some(runtime) => Some(runtime.ullage_pressure_pa().map_err(|err| {
                                RunnerError::Tank {
                                    field: format!(
                                        "vehicle.assembly.tanks.{id_v}.pressurization",
                                        id_v = id.value()
                                    ),
                                    reason: err,
                                }
                            })?),
                            None => None,
                        }
                    } else {
                        None
                    };
                let orifice_rate = match self.safe_dump_orifices.get(id) {
                    Some(orifice) => orifice
                        .dump_rate_kg_per_s(
                            tank,
                            vented_pressure_pa.or(pressurized_pressure_pa),
                            elapsed_s,
                            ambient_backpressure_pa,
                        )
                        .map_err(|err| RunnerError::Tank {
                            field: format!(
                                "vehicle.assembly.tanks.{id_v}.safe_dump_orifice",
                                id_v = id.value()
                            ),
                            reason: err,
                        })?,
                    None => 0.0,
                };
                if let Some(vent) = self.safe_dump_vents.get_mut(id) {
                    vent.step(
                        tank,
                        self.dt.as_seconds(),
                        elapsed_s,
                        ambient_backpressure_pa,
                    )
                    .map_err(|err| RunnerError::Tank {
                        field: format!(
                            "vehicle.assembly.tanks.{id_v}.safe_dump_vent",
                            id_v = id.value()
                        ),
                        reason: err,
                    })?;
                }
                fixed_rate + orifice_rate
            } else {
                0.0
            };
            let rate = drain_rate + dump_rate;
            let before_mass_kg = tank.fluid_remaining_kg();
            tank.drain(rate).map_err(|err| RunnerError::Tank {
                field: format!("vehicle.assembly.tanks.{id_v}", id_v = id.value()),
                reason: err.to_string(),
            })?;
            if self.pressurization_enabled
                && let Some(runtime) = self.pressurization.get_mut(id)
            {
                let propellant_outflow_volume_rate_m3_s = rate / tank.propellant().density_kg_m3;
                runtime
                    .step(self.dt.as_seconds(), propellant_outflow_volume_rate_m3_s)
                    .map_err(|err| RunnerError::Tank {
                        field: format!(
                            "vehicle.assembly.tanks.{id_v}.pressurization",
                            id_v = id.value()
                        ),
                        reason: err,
                    })?;
            }
            tank.step(accel, omega, self.dt)
                .map_err(|err| RunnerError::Tank {
                    field: format!("vehicle.assembly.tanks.{id_v}", id_v = id.value()),
                    reason: err.to_string(),
                })?;
            let after_mass_kg = tank.fluid_remaining_kg();
            let dt_s = self.dt.as_seconds();
            let mass_rate_kg_s = if dt_s > 0.0 {
                (after_mass_kg - before_mass_kg) / dt_s
            } else {
                0.0
            };
            self.latest_mass_rates_kg_per_s.insert(*id, mass_rate_kg_s);
        }
        self.elapsed_s += self.dt.as_seconds();
        Ok(())
    }

    /// Produce the per-tank snapshot map the kernel consumes via
    /// `set_tank_snapshot`.
    #[must_use]
    pub fn snapshot_map(&self) -> BTreeMap<TankId, TankSnapshot> {
        let mut out = BTreeMap::new();
        for (id, tank) in &self.tanks {
            let mass = tank.mass_contribution();
            let reaction = tank.reaction_body();
            out.insert(
                *id,
                TankSnapshot {
                    mass_kg: mass.mass_kg,
                    cg_offset_body_m: mass.cg_offset_body_m,
                    inertia_delta_body_kg_m2: mass.inertia_delta_body_kg_m2,
                    reaction_force_body_n: reaction.force_body_n,
                    reaction_moment_body_n_m: reaction.moment_body_n_m,
                    fluid_remaining_kg: tank.fluid_remaining_kg(),
                    mass_rate_kg_s: self
                        .latest_mass_rates_kg_per_s
                        .get(id)
                        .copied()
                        .unwrap_or(0.0),
                },
            );
        }
        out
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PressurizationRuntime {
    config: FiniteVolumePressurizationConfig,
    model: FiniteVolumePressurizationModel,
    state: FiniteVolumePressurizationState,
    latest_snapshot: FiniteVolumePressurizationSnapshot,
}

impl PressurizationRuntime {
    fn ullage_pressure_pa(self) -> Result<f64, String> {
        self.model
            .ullage_pressure_pa(self.state)
            .map_err(|err| err.to_string())
    }

    fn step(&mut self, dt_s: f64, propellant_outflow_volume_rate_m3_s: f64) -> Result<(), String> {
        if !propellant_outflow_volume_rate_m3_s.is_finite()
            || propellant_outflow_volume_rate_m3_s < 0.0
        {
            return Err(
                "finite-volume tank pressurization requires finite non-negative outflow".to_owned(),
            );
        }
        let mut config = self.config;
        config.propellant_outflow_volume_rate_m3_s = propellant_outflow_volume_rate_m3_s;
        let model = FiniteVolumePressurizationModel::new(config).map_err(|err| err.to_string())?;
        let snapshot = model
            .step(self.state, dt_s)
            .map_err(|err| err.to_string())?;
        self.config = config;
        self.model = model;
        self.state = snapshot.state;
        self.latest_snapshot = snapshot;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct SafeDumpOrificeRuntime {
    discharge_coefficient: f64,
    area_m2: f64,
    backpressure_pa: f64,
    backpressure_source: TankSafeDumpBackpressureSourceConfig,
    backpressure_ramp: Option<TankSafeDumpBackpressureRampConfig>,
    initial_pressure_pa: f64,
    initial_ullage_m3: f64,
    density_kg_m3: f64,
    gas_gamma: f64,
}

impl SafeDumpOrificeRuntime {
    fn dump_rate_kg_per_s(
        self,
        tank: &Tank,
        pressure_override_pa: Option<f64>,
        elapsed_s: f64,
        ambient_backpressure_pa: Option<f64>,
    ) -> Result<f64, String> {
        let pressure_pa = match pressure_override_pa {
            Some(pressure) => pressure,
            None => self.pressure_pa(tank)?,
        };
        let backpressure_pa = safe_dump_backpressure_pa(
            self.backpressure_pa,
            self.backpressure_ramp,
            self.backpressure_source,
            elapsed_s,
            ambient_backpressure_pa,
        )?;
        let delta_p_pa = (pressure_pa - backpressure_pa).max(0.0);
        let rate = self.discharge_coefficient
            * self.area_m2
            * (2.0 * self.density_kg_m3 * delta_p_pa).sqrt();
        if !rate.is_finite() || rate < 0.0 {
            return Err("safe dump orifice produced a non-finite or negative rate".to_owned());
        }
        Ok(rate)
    }

    fn pressure_pa(self, tank: &Tank) -> Result<f64, String> {
        let current_propellant_volume_m3 = tank.fluid_remaining_kg() / self.density_kg_m3;
        let current_ullage_m3 = tank.geometry().volume_m3() - current_propellant_volume_m3;
        if current_ullage_m3 <= 0.0 {
            return Err("safe dump orifice requires positive current ullage volume".to_owned());
        }
        let pressure_pa = self.initial_pressure_pa
            * (self.initial_ullage_m3 / current_ullage_m3).powf(self.gas_gamma);
        if !pressure_pa.is_finite() || pressure_pa < 0.0 {
            return Err("safe dump orifice produced a non-finite or negative pressure".to_owned());
        }
        Ok(pressure_pa)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct SafeDumpVentRuntime {
    discharge_coefficient: f64,
    area_m2: f64,
    backpressure_pa: f64,
    backpressure_source: TankSafeDumpBackpressureSourceConfig,
    backpressure_ramp: Option<TankSafeDumpBackpressureRampConfig>,
    initial_gas_temperature_k: f64,
    gas_temperature_k: f64,
    wall_temperature_k: Option<f64>,
    wall_thermal_relaxation_tau_s: Option<f64>,
    gas_specific_gas_constant_j_kg_k: f64,
    density_kg_m3: f64,
    gas_gamma: f64,
    initial_ullage_m3: f64,
    initial_gas_mass_kg: f64,
    gas_mass_kg: f64,
    thermal_model: TankSafeDumpVentThermalModelConfig,
}

impl SafeDumpVentRuntime {
    fn current_ullage_m3(self, tank: &Tank) -> Result<f64, String> {
        let current_propellant_volume_m3 = tank.fluid_remaining_kg() / self.density_kg_m3;
        let current_ullage_m3 = tank.geometry().volume_m3() - current_propellant_volume_m3;
        if current_ullage_m3 <= 0.0 {
            return Err("safe dump vent requires positive current ullage volume".to_owned());
        }
        Ok(current_ullage_m3)
    }

    fn adiabatic_temperature_k(self, current_ullage_m3: f64) -> Result<f64, String> {
        if self.initial_gas_mass_kg <= 0.0
            || self.gas_mass_kg <= 0.0
            || self.initial_ullage_m3 <= 0.0
            || current_ullage_m3 <= 0.0
        {
            return Err("safe dump vent adiabatic model requires positive gas state".to_owned());
        }
        let density_ratio = (self.gas_mass_kg / self.initial_gas_mass_kg)
            * (self.initial_ullage_m3 / current_ullage_m3);
        Ok(self.initial_gas_temperature_k * density_ratio.powf(self.gas_gamma - 1.0))
    }

    fn effective_gas_temperature_k(self, current_ullage_m3: f64) -> Result<f64, String> {
        let temperature = match self.thermal_model {
            TankSafeDumpVentThermalModelConfig::FixedTemperature => self.initial_gas_temperature_k,
            TankSafeDumpVentThermalModelConfig::AdiabaticUllage => {
                self.adiabatic_temperature_k(current_ullage_m3)?
            }
            TankSafeDumpVentThermalModelConfig::AdiabaticWallRelaxation => self.gas_temperature_k,
        };
        if !temperature.is_finite() || temperature <= 0.0 {
            return Err(
                "safe dump vent produced a non-finite or non-positive gas temperature".to_owned(),
            );
        }
        Ok(temperature)
    }

    fn pressure_pa(self, tank: &Tank) -> Result<f64, String> {
        let current_ullage_m3 = self.current_ullage_m3(tank)?;
        let gas_temperature_k = self.effective_gas_temperature_k(current_ullage_m3)?;
        let pressure_pa =
            self.gas_mass_kg * self.gas_specific_gas_constant_j_kg_k * gas_temperature_k
                / current_ullage_m3;
        if !pressure_pa.is_finite() || pressure_pa < 0.0 {
            return Err("safe dump vent produced a non-finite or negative pressure".to_owned());
        }
        Ok(pressure_pa)
    }

    fn vent_rate_kg_per_s(
        self,
        tank: &Tank,
        elapsed_s: f64,
        ambient_backpressure_pa: Option<f64>,
    ) -> Result<f64, String> {
        let current_ullage_m3 = self.current_ullage_m3(tank)?;
        let gas_temperature_k = self.effective_gas_temperature_k(current_ullage_m3)?;
        let pressure_pa = self.pressure_pa(tank)?;
        let backpressure_pa = safe_dump_backpressure_pa(
            self.backpressure_pa,
            self.backpressure_ramp,
            self.backpressure_source,
            elapsed_s,
            ambient_backpressure_pa,
        )?;
        if pressure_pa <= backpressure_pa {
            return Ok(0.0);
        }
        let pressure_ratio = (backpressure_pa / pressure_pa).clamp(0.0, 1.0);
        let critical_pressure_ratio =
            (2.0 / (self.gas_gamma + 1.0)).powf(self.gas_gamma / (self.gas_gamma - 1.0));
        let mass_flux_factor = if pressure_ratio <= critical_pressure_ratio {
            (self.gas_gamma / (self.gas_specific_gas_constant_j_kg_k * gas_temperature_k)).sqrt()
                * (2.0 / (self.gas_gamma + 1.0))
                    .powf((self.gas_gamma + 1.0) / (2.0 * (self.gas_gamma - 1.0)))
        } else {
            let expansion = pressure_ratio.powf(2.0 / self.gas_gamma)
                - pressure_ratio.powf((self.gas_gamma + 1.0) / self.gas_gamma);
            (2.0 * self.gas_gamma
                / (self.gas_specific_gas_constant_j_kg_k
                    * gas_temperature_k
                    * (self.gas_gamma - 1.0))
                * expansion.max(0.0))
            .sqrt()
        };
        let rate = self.discharge_coefficient * self.area_m2 * pressure_pa * mass_flux_factor;
        if !rate.is_finite() || rate < 0.0 {
            return Err("safe dump vent produced a non-finite or negative rate".to_owned());
        }
        Ok(rate)
    }

    fn step(
        &mut self,
        tank: &Tank,
        dt_s: f64,
        elapsed_s: f64,
        ambient_backpressure_pa: Option<f64>,
    ) -> Result<(), String> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err("safe dump vent requires a positive finite time step".to_owned());
        }
        let vented_kg = self.vent_rate_kg_per_s(tank, elapsed_s, ambient_backpressure_pa)? * dt_s;
        if vented_kg > self.gas_mass_kg {
            return Err("safe dump vent would empty ullage gas in one step".to_owned());
        }
        self.gas_mass_kg -= vented_kg;
        if !self.gas_mass_kg.is_finite() || self.gas_mass_kg < 0.0 {
            return Err("safe dump vent produced a non-finite or negative gas mass".to_owned());
        }
        if self.thermal_model == TankSafeDumpVentThermalModelConfig::AdiabaticWallRelaxation {
            self.apply_wall_relaxation(tank, dt_s)?;
        }
        Ok(())
    }

    fn apply_wall_relaxation(&mut self, tank: &Tank, dt_s: f64) -> Result<(), String> {
        let wall_temperature_k = self.wall_temperature_k.ok_or_else(|| {
            "safe dump vent wall relaxation requires a configured wall temperature".to_owned()
        })?;
        let tau_s = self.wall_thermal_relaxation_tau_s.ok_or_else(|| {
            "safe dump vent wall relaxation requires a configured relaxation time constant"
                .to_owned()
        })?;
        if !wall_temperature_k.is_finite() || wall_temperature_k <= 0.0 {
            return Err(
                "safe dump vent wall relaxation requires a positive finite wall temperature"
                    .to_owned(),
            );
        }
        if !tau_s.is_finite() || tau_s <= 0.0 {
            return Err(
                "safe dump vent wall relaxation requires a positive finite time constant"
                    .to_owned(),
            );
        }
        let current_ullage_m3 = self.current_ullage_m3(tank)?;
        let adiabatic_temperature_k = self.adiabatic_temperature_k(current_ullage_m3)?;
        let relaxation = (-dt_s / tau_s).exp();
        let gas_temperature_k =
            wall_temperature_k + (adiabatic_temperature_k - wall_temperature_k) * relaxation;
        if !gas_temperature_k.is_finite() || gas_temperature_k <= 0.0 {
            return Err(
                "safe dump vent wall relaxation produced a non-finite or non-positive gas temperature"
                    .to_owned(),
            );
        }
        self.gas_temperature_k = gas_temperature_k;
        Ok(())
    }
}

fn safe_dump_orifice_runtime(config: &TankConfig) -> Option<SafeDumpOrificeRuntime> {
    let orifice = config.safe_dump_orifice?;
    let ullage = config.ullage?;
    let geometry = build_geometry(&config.geometry);
    let density_kg_m3 = config.propellant.density_kg_m3;
    let initial_propellant_volume_m3 = geometry.volume_m3() * config.initial_fill_fraction;
    let initial_ullage_m3 = geometry.volume_m3() - initial_propellant_volume_m3;
    Some(SafeDumpOrificeRuntime {
        discharge_coefficient: orifice.discharge_coefficient,
        area_m2: orifice.area_m2,
        backpressure_pa: orifice.backpressure_pa,
        backpressure_source: orifice.backpressure_source,
        backpressure_ramp: orifice.backpressure_ramp,
        initial_pressure_pa: ullage.initial_pressure_pa,
        initial_ullage_m3,
        density_kg_m3,
        gas_gamma: ullage.gas_gamma,
    })
}

fn safe_dump_vent_runtime(config: &TankConfig) -> Option<SafeDumpVentRuntime> {
    let vent = config.safe_dump_vent?;
    let ullage = config.ullage?;
    let geometry = build_geometry(&config.geometry);
    let density_kg_m3 = config.propellant.density_kg_m3;
    let initial_propellant_volume_m3 = geometry.volume_m3() * config.initial_fill_fraction;
    let initial_ullage_m3 = geometry.volume_m3() - initial_propellant_volume_m3;
    let gas_mass_kg = ullage.initial_pressure_pa * initial_ullage_m3
        / (vent.gas_specific_gas_constant_j_kg_k * vent.gas_temperature_k);
    Some(SafeDumpVentRuntime {
        discharge_coefficient: vent.discharge_coefficient,
        area_m2: vent.area_m2,
        backpressure_pa: vent.backpressure_pa,
        backpressure_source: vent.backpressure_source,
        backpressure_ramp: vent.backpressure_ramp,
        initial_gas_temperature_k: vent.gas_temperature_k,
        gas_temperature_k: vent.gas_temperature_k,
        wall_temperature_k: vent.wall_temperature_k,
        wall_thermal_relaxation_tau_s: vent.wall_thermal_relaxation_tau_s,
        gas_specific_gas_constant_j_kg_k: vent.gas_specific_gas_constant_j_kg_k,
        density_kg_m3,
        gas_gamma: ullage.gas_gamma,
        initial_ullage_m3,
        initial_gas_mass_kg: gas_mass_kg,
        gas_mass_kg,
        thermal_model: vent.thermal_model,
    })
}

fn pressurization_runtime(config: &TankConfig) -> Result<Option<PressurizationRuntime>, String> {
    let Some(pressurization) = config.pressurization else {
        return Ok(None);
    };
    let Some(ullage) = config.ullage else {
        return Ok(None);
    };
    let geometry = build_geometry(&config.geometry);
    let tank_volume_m3 = geometry.volume_m3();
    let initial_propellant_volume_m3 = tank_volume_m3 * config.initial_fill_fraction;
    let initial_ullage_m3 = tank_volume_m3 - initial_propellant_volume_m3;
    let model_config = FiniteVolumePressurizationConfig {
        pressurant_inlet_pressure_pa: pressurization.pressurant_inlet_pressure_pa,
        pressurant_inlet_temperature_k: pressurization.pressurant_inlet_temperature_k,
        pressurant_gas_constant_j_per_kg_k: pressurization.pressurant_gas_constant_j_per_kg_k,
        pressurant_heat_capacity_ratio: pressurization.pressurant_heat_capacity_ratio,
        pressurant_flow_area_m2: pressurization.pressurant_flow_area_m2,
        pressurant_discharge_coefficient: pressurization.pressurant_discharge_coefficient,
        ullage_heat_transfer_w_per_k: pressurization.ullage_heat_transfer_w_per_k,
        tank_wall_temperature_k: pressurization.tank_wall_temperature_k,
        propellant_density_kg_m3: config.propellant.density_kg_m3,
        tank_cross_section_area_m2: pressurization.tank_cross_section_area_m2,
        propellant_outflow_volume_rate_m3_s: 0.0,
        gravity_m_s2: pressurization.gravity_m_s2,
    };
    let model =
        FiniteVolumePressurizationModel::new(model_config).map_err(|err| err.to_string())?;
    let state = FiniteVolumePressurizationState::from_ullage_pressure(
        model_config,
        0.0,
        ullage.initial_pressure_pa,
        pressurization.initial_ullage_temperature_k,
        initial_ullage_m3,
        initial_propellant_volume_m3,
    )
    .map_err(|err| err.to_string())?;
    let latest_snapshot = model.step(state, 0.0).map_err(|err| err.to_string())?;
    Ok(Some(PressurizationRuntime {
        config: model_config,
        model,
        state,
        latest_snapshot,
    }))
}

fn safe_dump_backpressure_pa(
    scalar_pa: f64,
    ramp: Option<TankSafeDumpBackpressureRampConfig>,
    source: TankSafeDumpBackpressureSourceConfig,
    elapsed_s: f64,
    ambient_backpressure_pa: Option<f64>,
) -> Result<f64, String> {
    let pressure_pa = match source {
        TankSafeDumpBackpressureSourceConfig::Configured => {
            let Some(ramp) = ramp else {
                return Ok(scalar_pa);
            };
            if elapsed_s <= ramp.start_time_s {
                ramp.start_pressure_pa
            } else if elapsed_s >= ramp.end_time_s {
                ramp.end_pressure_pa
            } else {
                let fraction =
                    (elapsed_s - ramp.start_time_s) / (ramp.end_time_s - ramp.start_time_s);
                ramp.start_pressure_pa + fraction * (ramp.end_pressure_pa - ramp.start_pressure_pa)
            }
        }
        TankSafeDumpBackpressureSourceConfig::AmbientAtmosphere => ambient_backpressure_pa
            .ok_or_else(|| {
                "safe dump ambient backpressure source requires an environment pressure sample"
                    .to_owned()
            })?,
    };
    if !pressure_pa.is_finite() || pressure_pa < 0.0 {
        return Err(
            "safe dump backpressure source produced a non-finite or negative pressure".to_owned(),
        );
    }
    Ok(pressure_pa)
}

/// Tank resolver: scenario `TankConfig` → openbmp-vehicle
/// `Tank` with the appropriate `Box<dyn MovingMassModel>` inner.
fn build_tank(config: &TankConfig) -> Result<(TankId, Tank), RunnerError> {
    let path = format!("vehicle.assembly.tanks.{id}", id = config.id);
    let id = TankId::from_path(&path);
    let geometry = build_geometry(&config.geometry);
    let propellant = build_propellant(&config.propellant);
    let mount = Vector3::new(
        config.mount_point_body_m[0],
        config.mount_point_body_m[1],
        config.mount_point_body_m[2],
    );
    let body_id = openbmp_core::BodyId::from_path(&format!(
        "vehicle.assembly.bodies.{body}",
        body = config.mounted_to
    ));

    let baffle = config.baffle_model.map(|b| BaffleModel {
        damping_increment_zeta: b.damping_increment_zeta,
    });

    let moving_mass: Box<dyn MovingMassModel> = match config.moving_mass {
        MovingMassKindConfig::RigidLiquid => Box::new(
            RigidLiquid::new(geometry, propellant, config.initial_fill_fraction, mount).map_err(
                |err| RunnerError::Tank {
                    field: path.clone(),
                    reason: err.to_string(),
                },
            )?,
        ),
        MovingMassKindConfig::EquivalentPendulum {
            damping_ratio_zeta,
            freefall_restoring,
        } => {
            let mut model = EquivalentPendulum::new(
                geometry,
                propellant,
                config.initial_fill_fraction,
                mount,
                damping_ratio_zeta,
            )
            .map_err(|err| RunnerError::Tank {
                field: path.clone(),
                reason: err.to_string(),
            })?;
            if let Some(FreefallRestoringConfig::CapillarySurfaceWave {
                surface_tension_n_m,
                damping_ratio_zeta,
            }) = freefall_restoring
            {
                model = model
                    .with_capillary_freefall_restoring(surface_tension_n_m, damping_ratio_zeta)
                    .map_err(|err| RunnerError::Tank {
                        field: path.clone(),
                        reason: err.to_string(),
                    })?;
            }
            apply_initial_slosh_pendulum(&mut model, config.initial_slosh.as_ref())?;
            Box::new(model)
        }
        MovingMassKindConfig::EquivalentSpringMass { damping_ratio_zeta } => {
            let mut model = EquivalentSpringMass::new(
                geometry,
                propellant,
                config.initial_fill_fraction,
                mount,
                damping_ratio_zeta,
            )
            .map_err(|err| RunnerError::Tank {
                field: path.clone(),
                reason: err.to_string(),
            })?;
            apply_initial_slosh_spring_mass(&mut model, config.initial_slosh.as_ref())?;
            Box::new(model)
        }
        MovingMassKindConfig::AbramsonMultiMass {
            damping_ratio_zeta,
            mode_count,
        } => {
            let mut model = AbramsonMultiMass::new(
                geometry,
                propellant,
                config.initial_fill_fraction,
                mount,
                damping_ratio_zeta,
                mode_count,
            )
            .map_err(|err| RunnerError::Tank {
                field: path.clone(),
                reason: err.to_string(),
            })?;
            apply_initial_slosh_abramson_multi_mass(&mut model, config.initial_slosh.as_ref())?;
            Box::new(model)
        }
        MovingMassKindConfig::BaffledPendulum {
            base_damping_ratio_zeta,
        } => {
            let baffle_model = baffle.unwrap_or(BaffleModel {
                damping_increment_zeta: 0.0,
            });
            let mut model = BaffledPendulum::new(
                geometry,
                propellant,
                config.initial_fill_fraction,
                mount,
                base_damping_ratio_zeta,
                baffle_model,
            )
            .map_err(|err| RunnerError::Tank {
                field: path.clone(),
                reason: err.to_string(),
            })?;
            apply_initial_slosh_baffled_pendulum(&mut model, config.initial_slosh.as_ref())?;
            Box::new(model)
        }
    };

    let tank = Tank::new(
        id,
        geometry,
        body_id,
        mount,
        propellant,
        config.initial_fill_fraction,
        baffle,
        moving_mass,
    )
    .map_err(|err| RunnerError::Tank {
        field: path.clone(),
        reason: err.to_string(),
    })?;

    Ok((id, tank))
}

fn build_geometry(config: &TankGeometryConfig) -> TankGeometry {
    match *config {
        TankGeometryConfig::Cylinder { radius_m, height_m } => {
            TankGeometry::Cylinder { radius_m, height_m }
        }
        TankGeometryConfig::Sphere { radius_m } => TankGeometry::Sphere { radius_m },
        TankGeometryConfig::EllipsoidTextbook { a_m, b_m, c_m } => {
            TankGeometry::EllipsoidTextbook { a_m, b_m, c_m }
        }
    }
}

fn build_propellant(config: &PropellantSpecConfig) -> PropellantSpec {
    PropellantSpec {
        density_kg_m3: config.density_kg_m3,
        // Leak the scenario-supplied label into a static string so
        // `PropellantSpec` (which holds `&'static str`) can carry it
        // verbatim. One-time leak per scenario load, ≈ 30 bytes per
        // tank — negligible.
        label: Box::leak(config.label.clone().into_boxed_str()),
    }
}

fn apply_initial_slosh_pendulum(
    model: &mut EquivalentPendulum,
    initial: Option<&InitialSloshConfig>,
) -> Result<(), RunnerError> {
    let Some(initial) = initial else {
        return Ok(());
    };
    let angles = initial.angles_rad.ok_or_else(|| RunnerError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.angles_rad".to_owned(),
        reason: "required for equivalent_pendulum".to_owned(),
    })?;
    let rates = initial.rates_rad_s.ok_or_else(|| RunnerError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.rates_rad_s".to_owned(),
        reason: "required for equivalent_pendulum".to_owned(),
    })?;
    model
        .set_initial_slosh((angles[0], angles[1]), (rates[0], rates[1]))
        .map_err(|err| RunnerError::Tank {
            field: "vehicle.assembly.tanks[*].initial_slosh".to_owned(),
            reason: err.to_string(),
        })
}

fn apply_initial_slosh_spring_mass(
    model: &mut EquivalentSpringMass,
    initial: Option<&InitialSloshConfig>,
) -> Result<(), RunnerError> {
    let Some(initial) = initial else {
        return Ok(());
    };
    let displacement = initial
        .displacement_body_m
        .ok_or_else(|| RunnerError::Tank {
            field: "vehicle.assembly.tanks[*].initial_slosh.displacement_body_m".to_owned(),
            reason: "required for equivalent_spring_mass".to_owned(),
        })?;
    let velocity = initial.velocity_body_m_s.ok_or_else(|| RunnerError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.velocity_body_m_s".to_owned(),
        reason: "required for equivalent_spring_mass".to_owned(),
    })?;
    model
        .set_initial_slosh(
            (displacement[0], displacement[1]),
            (velocity[0], velocity[1]),
        )
        .map_err(|err| RunnerError::Tank {
            field: "vehicle.assembly.tanks[*].initial_slosh".to_owned(),
            reason: err.to_string(),
        })
}

fn apply_initial_slosh_abramson_multi_mass(
    model: &mut AbramsonMultiMass,
    initial: Option<&InitialSloshConfig>,
) -> Result<(), RunnerError> {
    let Some(initial) = initial else {
        return Ok(());
    };
    let displacement = initial
        .displacement_body_m
        .ok_or_else(|| RunnerError::Tank {
            field: "vehicle.assembly.tanks[*].initial_slosh.displacement_body_m".to_owned(),
            reason: "required for abramson_multi_mass".to_owned(),
        })?;
    let velocity = initial.velocity_body_m_s.ok_or_else(|| RunnerError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.velocity_body_m_s".to_owned(),
        reason: "required for abramson_multi_mass".to_owned(),
    })?;
    model
        .set_initial_slosh(
            (displacement[0], displacement[1]),
            (velocity[0], velocity[1]),
        )
        .map_err(|err| RunnerError::Tank {
            field: "vehicle.assembly.tanks[*].initial_slosh".to_owned(),
            reason: err.to_string(),
        })
}

fn apply_initial_slosh_baffled_pendulum(
    model: &mut BaffledPendulum,
    initial: Option<&InitialSloshConfig>,
) -> Result<(), RunnerError> {
    let Some(initial) = initial else {
        return Ok(());
    };
    let angles = initial.angles_rad.ok_or_else(|| RunnerError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.angles_rad".to_owned(),
        reason: "required for baffled_pendulum".to_owned(),
    })?;
    let rates = initial.rates_rad_s.ok_or_else(|| RunnerError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.rates_rad_s".to_owned(),
        reason: "required for baffled_pendulum".to_owned(),
    })?;
    model
        .set_initial_slosh((angles[0], angles[1]), (rates[0], rates[1]))
        .map_err(|err| RunnerError::Tank {
            field: "vehicle.assembly.tanks[*].initial_slosh".to_owned(),
            reason: err.to_string(),
        })
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::SimTime;
    use openbmp_scenario::Scenario;

    const SLOSHING_TANK: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/sloshing-tank/sloshing-tank.toml"
    ));

    fn abramson_multi_mass_scenario_toml() -> String {
        SLOSHING_TANK
            .replace(
                r#"moving_mass              = { kind = "equivalent_pendulum", damping_ratio_zeta = 0.005 }"#,
                r#"moving_mass              = { kind = "abramson_multi_mass", damping_ratio_zeta = 0.005, mode_count = 3 }"#,
            )
            .replace(
                "initial_slosh            = { angles_rad = [0.05, 0.0], rates_rad_s = [0.0, 0.0] }",
                "initial_slosh            = { displacement_body_m = [0.05, 0.0], velocity_body_m_s = [0.0, 0.0] }",
            )
    }

    fn draining_slosh_tank_toml() -> String {
        SLOSHING_TANK.replace(
            "drain_rate_kg_per_s      = 0.0",
            "drain_rate_kg_per_s      = 5.0",
        )
    }

    fn safe_dump_slosh_tank_toml() -> String {
        SLOSHING_TANK.replace(
            "drain_rate_kg_per_s      = 0.0",
            "drain_rate_kg_per_s      = 5.0\nsafe_dump_rate_kg_per_s  = 11.0",
        )
    }

    fn safe_dump_orifice_slosh_tank_toml() -> String {
        SLOSHING_TANK.replace(
            "drain_rate_kg_per_s      = 0.0",
            "drain_rate_kg_per_s      = 5.0\n\
             ullage = { initial_pressure_pa = 2000000.0, gas_gamma = 1.4 }\n\
             safe_dump_orifice = { discharge_coefficient = 0.5, area_m2 = 0.00001, backpressure_pa = 500000.0 }",
        )
    }

    fn safe_dump_orifice_backpressure_ramp_slosh_tank_toml() -> String {
        SLOSHING_TANK.replace(
            "drain_rate_kg_per_s      = 0.0",
            "drain_rate_kg_per_s      = 5.0\n\
             ullage = { initial_pressure_pa = 2000000.0, gas_gamma = 1.4 }\n\
             safe_dump_orifice = { discharge_coefficient = 0.5, area_m2 = 0.00001, backpressure_ramp = { start_time_s = 0.0, end_time_s = 0.001, start_pressure_pa = 500000.0, end_pressure_pa = 1900000.0 } }",
        )
    }

    fn pressurized_safe_dump_orifice_slosh_tank_toml() -> String {
        safe_dump_orifice_slosh_tank_toml().replace(
            "safe_dump_orifice = { discharge_coefficient = 0.5, area_m2 = 0.00001, backpressure_pa = 500000.0 }",
            "pressurization = { initial_ullage_temperature_k = 300.0, pressurant_inlet_pressure_pa = 5000000.0, pressurant_inlet_temperature_k = 320.0, pressurant_gas_constant_j_per_kg_k = 287.0, pressurant_heat_capacity_ratio = 1.4, pressurant_flow_area_m2 = 0.001, pressurant_discharge_coefficient = 0.8, ullage_heat_transfer_w_per_k = 10.0, tank_wall_temperature_k = 315.0, tank_cross_section_area_m2 = 1.0, gravity_m_s2 = 9.80665 }\n\
             safe_dump_orifice = { discharge_coefficient = 0.5, area_m2 = 0.00001, backpressure_pa = 500000.0 }",
        )
    }

    fn safe_dump_vented_orifice_slosh_tank_toml() -> String {
        SLOSHING_TANK.replace(
            "drain_rate_kg_per_s      = 0.0",
            "drain_rate_kg_per_s      = 5.0\n\
             ullage = { initial_pressure_pa = 2000000.0, gas_gamma = 1.4 }\n\
             safe_dump_orifice = { discharge_coefficient = 0.5, area_m2 = 0.00001, backpressure_pa = 500000.0 }\n\
             safe_dump_vent = { discharge_coefficient = 0.5, area_m2 = 0.001, gas_temperature_k = 300.0, gas_specific_gas_constant_j_kg_k = 287.0, backpressure_pa = 0.0 }",
        )
    }

    fn safing_command(safed: bool, drain_enabled: bool) -> PropellantSafingCommand {
        PropellantSafingCommand {
            time: SimTime::ZERO,
            safed,
            isolate_feeds: false,
            close_engine_valves: false,
            pressurization_enabled: false,
            drain_enabled,
            dump_enabled: false,
        }
    }

    fn single_tank_fluid_mass(rack: &TankRack, document: &ScenarioDocument) -> f64 {
        rack.propellant_tank_states(document)
            .values()
            .next()
            .expect("one tank state")
            .fluid_remaining_kg
    }

    #[test]
    fn builds_abramson_multi_mass_tank_rack() {
        let toml = abramson_multi_mass_scenario_toml();
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = TankRack::build(&scenario.document).expect("tank rack builds");
        assert_eq!(rack.len(), 1);

        rack.update_drivers(Vector3::new(1.0, 0.0, 9.81), Vector3::zeros());
        rack.step().expect("tank rack steps");
        let snapshots = rack.snapshot_map();
        let snapshot = snapshots.values().next().unwrap();
        assert!(snapshot.mass_kg > 0.0);
        assert!(snapshot.cg_offset_body_m.x.is_finite());
        assert!(snapshot.cg_offset_body_m.x > 0.0);
        assert!(snapshot.cg_offset_body_m.x < 0.05);
        assert!(snapshot.reaction_force_body_n.x.is_finite());
        assert_ne!(
            snapshot.reaction_force_body_n.x.to_bits(),
            0.0_f64.to_bits()
        );
    }

    #[test]
    fn propellant_safing_disables_declared_and_rcs_tank_drains() {
        let toml = draining_slosh_tank_toml();
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = TankRack::build(&scenario.document).expect("tank rack builds");
        let tank_id = rack.tank_ids()[0];
        let mut rcs_drains = BTreeMap::new();
        rcs_drains.insert(tank_id, 7.0);
        rack.set_rcs_feed_drain_rates(rcs_drains);
        let initial = single_tank_fluid_mass(&rack, &scenario.document);

        rack.apply_propellant_safing_command(Some(safing_command(true, false)));
        rack.step().expect("tank rack steps");

        let after = single_tank_fluid_mass(&rack, &scenario.document);
        assert_eq!(after.to_bits(), initial.to_bits());
    }

    #[test]
    fn inactive_propellant_safing_keeps_tank_drains_enabled() {
        let toml = draining_slosh_tank_toml();
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = TankRack::build(&scenario.document).expect("tank rack builds");
        let tank_id = rack.tank_ids()[0];
        let mut rcs_drains = BTreeMap::new();
        rcs_drains.insert(tank_id, 7.0);
        rack.set_rcs_feed_drain_rates(rcs_drains);
        let initial = single_tank_fluid_mass(&rack, &scenario.document);

        rack.apply_propellant_safing_command(Some(safing_command(false, false)));
        rack.step().expect("tank rack steps");

        let after = single_tank_fluid_mass(&rack, &scenario.document);
        let expected_decrement = (5.0 + 7.0) * scenario.document.time.dt_s;
        assert!((initial - after - expected_decrement).abs() < 1.0e-12);
    }

    #[test]
    fn propellant_safing_dump_uses_declared_safe_dump_rate_when_drains_disabled() {
        let toml = safe_dump_slosh_tank_toml();
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = TankRack::build(&scenario.document).expect("tank rack builds");
        let tank_id = rack.tank_ids()[0];
        let mut rcs_drains = BTreeMap::new();
        rcs_drains.insert(tank_id, 7.0);
        rack.set_rcs_feed_drain_rates(rcs_drains);
        let initial = single_tank_fluid_mass(&rack, &scenario.document);

        let mut command = safing_command(true, false);
        command.dump_enabled = true;
        rack.apply_propellant_safing_command(Some(command));
        rack.step().expect("tank rack steps");

        let after = single_tank_fluid_mass(&rack, &scenario.document);
        let expected_decrement = 11.0 * scenario.document.time.dt_s;
        assert!((initial - after - expected_decrement).abs() < 1.0e-12);
    }

    #[test]
    fn propellant_safing_dump_orifice_uses_live_ullage_pressure() {
        let toml = safe_dump_orifice_slosh_tank_toml();
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = TankRack::build(&scenario.document).expect("tank rack builds");
        let initial = single_tank_fluid_mass(&rack, &scenario.document);

        let mut command = safing_command(true, false);
        command.dump_enabled = true;
        rack.apply_propellant_safing_command(Some(command));
        rack.step().expect("tank rack steps");

        let after = single_tank_fluid_mass(&rack, &scenario.document);
        let expected_rate = 0.5_f64 * 0.00001_f64 * (2.0_f64 * 250.0_f64 * 1_500_000.0_f64).sqrt();
        let expected_decrement = expected_rate * scenario.document.time.dt_s;
        assert!((initial - after - expected_decrement).abs() < 1.0e-12);
    }

    #[test]
    fn tank_pressurization_raises_orifice_flow_until_safe_disables_it() {
        let toml = pressurized_safe_dump_orifice_slosh_tank_toml();
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut enabled_rack = TankRack::build(&scenario.document).expect("enabled rack builds");
        let mut disabled_rack = TankRack::build(&scenario.document).expect("disabled rack builds");
        let tank_id = enabled_rack.tank_ids()[0];
        let initial_pressure = enabled_rack.pressurization[&tank_id]
            .ullage_pressure_pa()
            .expect("initial pressure");

        let mut enabled_command = safing_command(true, false);
        enabled_command.dump_enabled = true;
        enabled_command.pressurization_enabled = true;
        enabled_rack.apply_propellant_safing_command(Some(enabled_command));

        let mut disabled_command = safing_command(true, false);
        disabled_command.dump_enabled = true;
        disabled_rack.apply_propellant_safing_command(Some(disabled_command));

        let initial_enabled_mass = single_tank_fluid_mass(&enabled_rack, &scenario.document);
        let initial_disabled_mass = single_tank_fluid_mass(&disabled_rack, &scenario.document);
        enabled_rack.step().expect("enabled first tank rack step");
        disabled_rack.step().expect("disabled first tank rack step");
        let enabled_after_first = single_tank_fluid_mass(&enabled_rack, &scenario.document);
        let disabled_after_first = single_tank_fluid_mass(&disabled_rack, &scenario.document);
        let enabled_pressure = enabled_rack.pressurization[&tank_id]
            .ullage_pressure_pa()
            .expect("enabled pressure");
        let disabled_pressure = disabled_rack.pressurization[&tank_id]
            .ullage_pressure_pa()
            .expect("disabled pressure");
        assert!(
            enabled_pressure > initial_pressure,
            "pressurant inlet should raise ullage pressure after one step: \
             initial={initial_pressure:.17e}, enabled={enabled_pressure:.17e}"
        );
        assert_eq!(disabled_pressure.to_bits(), initial_pressure.to_bits());

        enabled_rack.step().expect("enabled second tank rack step");
        disabled_rack
            .step()
            .expect("disabled second tank rack step");
        let enabled_after_second = single_tank_fluid_mass(&enabled_rack, &scenario.document);
        let disabled_after_second = single_tank_fluid_mass(&disabled_rack, &scenario.document);
        let enabled_second_decrement = enabled_after_first - enabled_after_second;
        let disabled_second_decrement = disabled_after_first - disabled_after_second;
        assert!(
            enabled_second_decrement > disabled_second_decrement,
            "enabled pressurization should increase next-step pressure-coupled dump flow: \
             enabled={enabled_second_decrement:.17e}, disabled={disabled_second_decrement:.17e}"
        );
        assert!(
            (initial_enabled_mass
                - enabled_after_first
                - (initial_disabled_mass - disabled_after_first))
                .abs()
                < 1.0e-12,
            "first step uses the same initial pressure before pressurization advances"
        );
    }

    #[test]
    fn propellant_safing_dump_orifice_backpressure_ramp_reduces_later_flow() {
        let toml = safe_dump_orifice_backpressure_ramp_slosh_tank_toml();
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = TankRack::build(&scenario.document).expect("tank rack builds");
        let initial = single_tank_fluid_mass(&rack, &scenario.document);

        let mut command = safing_command(true, false);
        command.dump_enabled = true;
        rack.apply_propellant_safing_command(Some(command));
        rack.step().expect("first tank rack step");
        let after_first = single_tank_fluid_mass(&rack, &scenario.document);
        rack.step().expect("second tank rack step");
        let after_second = single_tank_fluid_mass(&rack, &scenario.document);

        let first_decrement = initial - after_first;
        let second_decrement = after_first - after_second;
        let expected_first_rate =
            0.5_f64 * 0.00001_f64 * (2.0_f64 * 250.0_f64 * 1_500_000.0_f64).sqrt();
        assert!(
            (first_decrement - expected_first_rate * scenario.document.time.dt_s).abs() < 1.0e-12
        );
        assert!(
            second_decrement < first_decrement * 0.35,
            "rising backpressure should reduce later liquid-orifice dump: \
             first={first_decrement:.17e}, second={second_decrement:.17e}"
        );
    }

    #[test]
    fn propellant_safing_dump_orifice_ambient_backpressure_reduces_flow() {
        let toml = safe_dump_orifice_slosh_tank_toml().replace(
            "backpressure_pa = 500000.0",
            "backpressure_source = \"ambient_atmosphere\", backpressure_pa = 0.0",
        );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut low_backpressure_rack = TankRack::build(&scenario.document).expect("rack builds");
        let mut high_backpressure_rack = TankRack::build(&scenario.document).expect("rack builds");
        let initial = single_tank_fluid_mass(&low_backpressure_rack, &scenario.document);

        let mut command = safing_command(true, false);
        command.dump_enabled = true;
        low_backpressure_rack.apply_propellant_safing_command(Some(command));
        high_backpressure_rack.apply_propellant_safing_command(Some(command));
        low_backpressure_rack
            .set_safe_dump_ambient_backpressure_pa(0.0)
            .expect("low ambient pressure accepted");
        high_backpressure_rack
            .set_safe_dump_ambient_backpressure_pa(1_900_000.0)
            .expect("high ambient pressure accepted");
        low_backpressure_rack
            .step()
            .expect("low-backpressure tank rack step");
        high_backpressure_rack
            .step()
            .expect("high-backpressure tank rack step");

        let low_decrement =
            initial - single_tank_fluid_mass(&low_backpressure_rack, &scenario.document);
        let high_decrement =
            initial - single_tank_fluid_mass(&high_backpressure_rack, &scenario.document);
        assert!(
            high_decrement < low_decrement * 0.3,
            "ambient backpressure should reduce liquid-orifice dump: \
             low={low_decrement:.17e}, high={high_decrement:.17e}"
        );
    }

    #[test]
    fn propellant_safing_dump_vent_lowers_next_step_orifice_rate() {
        let toml = safe_dump_vented_orifice_slosh_tank_toml();
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = TankRack::build(&scenario.document).expect("tank rack builds");
        let tank_id = rack.tank_ids()[0];
        let initial = single_tank_fluid_mass(&rack, &scenario.document);
        let initial_gas_mass = rack
            .safe_dump_vents
            .get(&tank_id)
            .expect("vent runtime present")
            .gas_mass_kg;

        let mut command = safing_command(true, false);
        command.dump_enabled = true;
        rack.apply_propellant_safing_command(Some(command));
        rack.step().expect("first tank rack step");
        let after_first = single_tank_fluid_mass(&rack, &scenario.document);
        let first_gas_mass = rack
            .safe_dump_vents
            .get(&tank_id)
            .expect("vent runtime present")
            .gas_mass_kg;

        rack.step().expect("second tank rack step");
        let after_second = single_tank_fluid_mass(&rack, &scenario.document);

        let expected_first_rate =
            0.5_f64 * 0.00001_f64 * (2.0_f64 * 250.0_f64 * 1_500_000.0_f64).sqrt();
        let first_decrement = initial - after_first;
        let second_decrement = after_first - after_second;
        assert!(
            (first_decrement - expected_first_rate * scenario.document.time.dt_s).abs() < 1.0e-12
        );
        assert!(
            first_gas_mass < initial_gas_mass,
            "SAFE vent should reduce ullage gas mass"
        );
        assert!(
            second_decrement < first_decrement,
            "vented ullage pressure should lower the next-step liquid orifice rate: first={first_decrement:.17e}, second={second_decrement:.17e}"
        );
    }

    #[test]
    fn propellant_safing_dump_vent_backpressure_ramp_reduces_later_gas_loss() {
        let toml = safe_dump_vented_orifice_slosh_tank_toml().replace(
            "gas_specific_gas_constant_j_kg_k = 287.0, backpressure_pa = 0.0",
            "gas_specific_gas_constant_j_kg_k = 287.0, backpressure_ramp = { start_time_s = 0.0, end_time_s = 0.001, start_pressure_pa = 0.0, end_pressure_pa = 1990000.0 }",
        );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = TankRack::build(&scenario.document).expect("tank rack builds");
        let tank_id = rack.tank_ids()[0];
        let initial_gas_mass = rack
            .safe_dump_vents
            .get(&tank_id)
            .expect("vent runtime present")
            .gas_mass_kg;

        let mut command = safing_command(true, false);
        command.dump_enabled = true;
        rack.apply_propellant_safing_command(Some(command));
        rack.step().expect("first tank rack step");
        let after_first_gas_mass = rack
            .safe_dump_vents
            .get(&tank_id)
            .expect("vent runtime present")
            .gas_mass_kg;
        rack.step().expect("second tank rack step");
        let after_second_gas_mass = rack
            .safe_dump_vents
            .get(&tank_id)
            .expect("vent runtime present")
            .gas_mass_kg;

        let first_gas_loss = initial_gas_mass - after_first_gas_mass;
        let second_gas_loss = after_first_gas_mass - after_second_gas_mass;
        assert!(first_gas_loss > 0.0);
        assert!(
            second_gas_loss < first_gas_loss * 0.25,
            "rising backpressure should reduce later gas venting: \
             first={first_gas_loss:.17e}, second={second_gas_loss:.17e}"
        );
    }

    #[test]
    fn propellant_safing_dump_vent_ambient_backpressure_reduces_gas_loss() {
        let toml = safe_dump_vented_orifice_slosh_tank_toml().replace(
            "gas_specific_gas_constant_j_kg_k = 287.0, backpressure_pa = 0.0",
            "gas_specific_gas_constant_j_kg_k = 287.0, backpressure_source = \"ambient_atmosphere\", backpressure_pa = 0.0",
        );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut low_backpressure_rack = TankRack::build(&scenario.document).expect("rack builds");
        let mut high_backpressure_rack = TankRack::build(&scenario.document).expect("rack builds");
        let tank_id = low_backpressure_rack.tank_ids()[0];
        let initial_gas_mass = low_backpressure_rack
            .safe_dump_vents
            .get(&tank_id)
            .expect("vent runtime present")
            .gas_mass_kg;

        let mut command = safing_command(true, false);
        command.dump_enabled = true;
        low_backpressure_rack.apply_propellant_safing_command(Some(command));
        high_backpressure_rack.apply_propellant_safing_command(Some(command));
        low_backpressure_rack
            .set_safe_dump_ambient_backpressure_pa(0.0)
            .expect("low ambient pressure accepted");
        high_backpressure_rack
            .set_safe_dump_ambient_backpressure_pa(1_990_000.0)
            .expect("high ambient pressure accepted");
        low_backpressure_rack
            .step()
            .expect("low-backpressure tank rack step");
        high_backpressure_rack
            .step()
            .expect("high-backpressure tank rack step");

        let low_gas_loss =
            initial_gas_mass - low_backpressure_rack.safe_dump_vents[&tank_id].gas_mass_kg;
        let high_gas_loss =
            initial_gas_mass - high_backpressure_rack.safe_dump_vents[&tank_id].gas_mass_kg;
        assert!(
            high_gas_loss < low_gas_loss * 0.2,
            "ambient backpressure should reduce gas venting: \
             low={low_gas_loss:.17e}, high={high_gas_loss:.17e}"
        );
    }

    #[test]
    fn propellant_safing_dump_adiabatic_vent_lowers_pressure_below_fixed_temperature() {
        let fixed_toml = safe_dump_vented_orifice_slosh_tank_toml();
        let adiabatic_toml = fixed_toml.replace(
            "gas_specific_gas_constant_j_kg_k = 287.0, backpressure_pa = 0.0",
            "gas_specific_gas_constant_j_kg_k = 287.0, thermal_model = \"adiabatic_ullage\", backpressure_pa = 0.0",
        );
        let fixed_scenario = Scenario::from_toml_str(&fixed_toml).expect("fixed scenario parses");
        let adiabatic_scenario =
            Scenario::from_toml_str(&adiabatic_toml).expect("adiabatic scenario parses");
        let mut fixed_rack = TankRack::build(&fixed_scenario.document).expect("fixed rack builds");
        let mut adiabatic_rack =
            TankRack::build(&adiabatic_scenario.document).expect("adiabatic rack builds");
        let fixed_tank_id = fixed_rack.tank_ids()[0];
        let adiabatic_tank_id = adiabatic_rack.tank_ids()[0];

        let mut command = safing_command(true, false);
        command.dump_enabled = true;
        fixed_rack.apply_propellant_safing_command(Some(command));
        adiabatic_rack.apply_propellant_safing_command(Some(command));
        fixed_rack.step().expect("fixed first tank rack step");
        adiabatic_rack
            .step()
            .expect("adiabatic first tank rack step");

        let fixed_after_first = single_tank_fluid_mass(&fixed_rack, &fixed_scenario.document);
        let adiabatic_after_first =
            single_tank_fluid_mass(&adiabatic_rack, &adiabatic_scenario.document);
        assert!(
            (fixed_after_first - adiabatic_after_first).abs() < 1.0e-12,
            "first step should be identical because both models start at the declared temperature"
        );

        let fixed_pressure = fixed_rack.safe_dump_vents[&fixed_tank_id]
            .pressure_pa(&fixed_rack.tanks[&fixed_tank_id])
            .expect("fixed pressure");
        let adiabatic_pressure = adiabatic_rack.safe_dump_vents[&adiabatic_tank_id]
            .pressure_pa(&adiabatic_rack.tanks[&adiabatic_tank_id])
            .expect("adiabatic pressure");
        assert!(
            adiabatic_pressure < fixed_pressure,
            "adiabatic ullage pressure should fall below fixed-temperature pressure: \
             adiabatic={adiabatic_pressure:.17e}, fixed={fixed_pressure:.17e}"
        );

        fixed_rack.step().expect("fixed second tank rack step");
        adiabatic_rack
            .step()
            .expect("adiabatic second tank rack step");
        let fixed_after_second = single_tank_fluid_mass(&fixed_rack, &fixed_scenario.document);
        let adiabatic_after_second =
            single_tank_fluid_mass(&adiabatic_rack, &adiabatic_scenario.document);
        let fixed_second_decrement = fixed_after_first - fixed_after_second;
        let adiabatic_second_decrement = adiabatic_after_first - adiabatic_after_second;
        assert!(
            adiabatic_second_decrement < fixed_second_decrement,
            "adiabatic pressure should reduce the next liquid-orifice dump decrement: \
            adiabatic={adiabatic_second_decrement:.17e}, fixed={fixed_second_decrement:.17e}"
        );
    }

    #[test]
    fn propellant_safing_dump_wall_relaxed_vent_warms_adiabatic_pressure() {
        let fixed_toml = safe_dump_vented_orifice_slosh_tank_toml();
        let adiabatic_toml = fixed_toml.replace(
            "gas_specific_gas_constant_j_kg_k = 287.0, backpressure_pa = 0.0",
            "gas_specific_gas_constant_j_kg_k = 287.0, thermal_model = \"adiabatic_ullage\", backpressure_pa = 0.0",
        );
        let wall_relaxed_toml = fixed_toml.replace(
            "gas_specific_gas_constant_j_kg_k = 287.0, backpressure_pa = 0.0",
            "gas_specific_gas_constant_j_kg_k = 287.0, thermal_model = \"adiabatic_wall_relaxation\", wall_temperature_k = 450.0, wall_thermal_relaxation_tau_s = 0.0001, backpressure_pa = 0.0",
        );
        let adiabatic_scenario =
            Scenario::from_toml_str(&adiabatic_toml).expect("adiabatic scenario parses");
        let wall_relaxed_scenario =
            Scenario::from_toml_str(&wall_relaxed_toml).expect("wall-relaxed scenario parses");
        let mut adiabatic_rack =
            TankRack::build(&adiabatic_scenario.document).expect("adiabatic rack builds");
        let mut wall_relaxed_rack =
            TankRack::build(&wall_relaxed_scenario.document).expect("wall-relaxed rack builds");
        let adiabatic_tank_id = adiabatic_rack.tank_ids()[0];
        let wall_relaxed_tank_id = wall_relaxed_rack.tank_ids()[0];

        let mut command = safing_command(true, false);
        command.dump_enabled = true;
        adiabatic_rack.apply_propellant_safing_command(Some(command));
        wall_relaxed_rack.apply_propellant_safing_command(Some(command));
        adiabatic_rack
            .step()
            .expect("adiabatic first tank rack step");
        wall_relaxed_rack
            .step()
            .expect("wall-relaxed first tank rack step");

        let adiabatic_pressure = adiabatic_rack.safe_dump_vents[&adiabatic_tank_id]
            .pressure_pa(&adiabatic_rack.tanks[&adiabatic_tank_id])
            .expect("adiabatic pressure");
        let wall_relaxed_pressure = wall_relaxed_rack.safe_dump_vents[&wall_relaxed_tank_id]
            .pressure_pa(&wall_relaxed_rack.tanks[&wall_relaxed_tank_id])
            .expect("wall-relaxed pressure");
        assert!(
            wall_relaxed_pressure > adiabatic_pressure,
            "warm-wall relaxation should raise post-vent pressure above pure adiabatic: \
             wall_relaxed={wall_relaxed_pressure:.17e}, adiabatic={adiabatic_pressure:.17e}"
        );
    }
}
