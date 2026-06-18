//! Vehicle-side engine-to-tank propellant budget.
//!
//! The propulsion crate owns engine physics and the tank module owns
//! moving-mass dynamics; this adapter closes the budget between them
//! without adding a tank dependency to `openbmp-propulsion`.

use std::collections::{BTreeMap, BTreeSet};

use openbmp_core::{EngineId, TankId};
use openbmp_propulsion::{EngineSnapshot, EngineState};
use thiserror::Error;

/// Feed-system mode for a propellant-budget binding.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FeedMode {
    /// Regulator holds feed pressure; scale is always `1.0` until
    /// depletion.
    Regulated,
    /// Pressure-fed blowdown using isentropic ullage expansion.
    Blowdown,
}

/// Per-engine propellant ownership declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct EnginePropellantBinding {
    /// Engine id.
    pub engine_id: EngineId,
    /// Oxidizer/fuel mass ratio. `0.0` denotes monopropellant.
    pub oxidizer_fuel_ratio: f64,
    /// Fuel or monopropellant tank.
    pub fuel_tank: TankId,
    /// Oxidizer tank, required when `oxidizer_fuel_ratio > 0`.
    pub oxidizer_tank: Option<TankId>,
    /// Feed-pressure model.
    pub feed: FeedMode,
    /// Per-tank residual reserve (kg). Depletion at or below this
    /// mass triggers engine shutdown on the next step.
    pub residual_reserve_kg: f64,
}

impl EnginePropellantBinding {
    /// Validate scalar and tank-reference shape.
    ///
    /// # Errors
    ///
    /// Returns [`PropellantBudgetError`] when the declaration is
    /// inconsistent.
    pub fn require_valid(&self) -> Result<(), PropellantBudgetError> {
        if !self.oxidizer_fuel_ratio.is_finite() || self.oxidizer_fuel_ratio < 0.0 {
            return Err(PropellantBudgetError::InvalidBinding {
                reason: "oxidizer_fuel_ratio must be finite and non-negative",
            });
        }
        if self.oxidizer_fuel_ratio > 0.0 && self.oxidizer_tank.is_none() {
            return Err(PropellantBudgetError::InvalidBinding {
                reason: "oxidizer_tank is required when oxidizer_fuel_ratio > 0",
            });
        }
        if self.oxidizer_fuel_ratio == 0.0 && self.oxidizer_tank.is_some() {
            return Err(PropellantBudgetError::InvalidBinding {
                reason: "oxidizer_tank is not accepted for monopropellant binding",
            });
        }
        if !self.residual_reserve_kg.is_finite() || self.residual_reserve_kg < 0.0 {
            return Err(PropellantBudgetError::InvalidBinding {
                reason: "residual_reserve_kg must be finite and non-negative",
            });
        }
        Ok(())
    }
}

/// Tank state needed by the budget. Runner-side tank racks can derive
/// this from live [`crate::tank::Tank`] instances.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PropellantTankState {
    /// Current fluid mass (kg).
    pub fluid_remaining_kg: f64,
    /// Initial fluid mass (kg).
    pub initial_fluid_mass_kg: f64,
    /// Tank internal volume (m³).
    pub volume_m3: f64,
    /// Propellant density (kg/m³).
    pub density_kg_m3: f64,
    /// Whether ullage/pressurant state was declared for blowdown.
    pub has_ullage: bool,
    /// Pressurant heat-capacity ratio.
    pub ullage_gamma: f64,
}

impl PropellantTankState {
    /// Validate the state.
    ///
    /// # Errors
    ///
    /// Returns [`PropellantBudgetError`] for invalid tank state.
    pub fn require_valid(&self) -> Result<(), PropellantBudgetError> {
        for value in [
            self.fluid_remaining_kg,
            self.initial_fluid_mass_kg,
            self.volume_m3,
            self.density_kg_m3,
            self.ullage_gamma,
        ] {
            if !value.is_finite() {
                return Err(PropellantBudgetError::InvalidTankState {
                    reason: "tank budget state contains a non-finite value",
                });
            }
        }
        if self.fluid_remaining_kg < 0.0
            || self.initial_fluid_mass_kg < 0.0
            || self.volume_m3 <= 0.0
            || self.density_kg_m3 <= 0.0
        {
            return Err(PropellantBudgetError::InvalidTankState {
                reason: "tank budget masses, volume, and density must be physically valid",
            });
        }
        if self.has_ullage && self.ullage_gamma <= 1.0 {
            return Err(PropellantBudgetError::InvalidTankState {
                reason: "blowdown ullage_gamma must be greater than 1",
            });
        }
        Ok(())
    }

    /// Normalized blowdown feed-pressure scale from isentropic ullage
    /// expansion.
    ///
    /// # Errors
    ///
    /// Returns [`PropellantBudgetError`] when the tank lacks an ullage
    /// declaration or contains invalid volume/state data.
    pub fn blowdown_pressure_scale(&self) -> Result<f64, PropellantBudgetError> {
        blowdown_tank_scale(self)
    }
}

/// Per-engine residual/utilization output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EnginePropellantUtilization {
    /// Engine id.
    pub engine_id: EngineId,
    /// Fuel/monopropellant residual at evaluation time (kg).
    pub fuel_residual_kg: f64,
    /// Oxidizer residual at evaluation time (kg), or zero for
    /// monopropellant.
    pub oxidizer_residual_kg: f64,
    /// Fraction of declared initial propellant mass consumed.
    pub utilization_fraction: f64,
}

/// Budget result for the next tank/engine step.
#[derive(Clone, Debug, PartialEq)]
pub struct PropellantBudgetReport {
    /// Tank drain rates to apply on the next tank step.
    pub tank_drain_rates_kg_per_s: BTreeMap<TankId, f64>,
    /// Engines that must be shutdown because a bound tank is at or
    /// below reserve.
    pub shutdown_engines: BTreeSet<EngineId>,
    /// Feed-pressure scale per engine for the next engine step.
    pub feed_pressure_scales: BTreeMap<EngineId, f64>,
    /// Residual/utilization diagnostics.
    pub utilization: Vec<EnginePropellantUtilization>,
}

/// Propellant-budget adapter.
#[derive(Clone, Debug, PartialEq)]
pub struct PropellantBudget {
    bindings: Vec<EnginePropellantBinding>,
}

impl PropellantBudget {
    /// Construct from engine bindings.
    ///
    /// # Errors
    ///
    /// Returns [`PropellantBudgetError`] for duplicate engines or
    /// invalid binding values.
    pub fn new(bindings: Vec<EnginePropellantBinding>) -> Result<Self, PropellantBudgetError> {
        let mut seen = BTreeSet::new();
        for binding in &bindings {
            binding.require_valid()?;
            if !seen.insert(binding.engine_id) {
                return Err(PropellantBudgetError::InvalidBinding {
                    reason: "duplicate engine propellant binding",
                });
            }
        }
        Ok(Self { bindings })
    }

    /// `true` when no engine declares a propellant budget.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.bindings.is_empty()
    }

    /// Evaluate tank drain, feed scale, and depletion shutdown for
    /// the next step from current engine snapshots and tank states.
    ///
    /// # Errors
    ///
    /// Returns [`PropellantBudgetError`] when a referenced engine or
    /// tank is missing, or when a scalar is invalid.
    pub fn evaluate(
        &self,
        engines: &BTreeMap<EngineId, EngineSnapshot>,
        tanks: &BTreeMap<TankId, PropellantTankState>,
    ) -> Result<PropellantBudgetReport, PropellantBudgetError> {
        let mut drains = BTreeMap::new();
        let mut shutdown = BTreeSet::new();
        let mut feed_scales = BTreeMap::new();
        let mut utilization = Vec::with_capacity(self.bindings.len());
        for binding in &self.bindings {
            let snapshot =
                engines
                    .get(&binding.engine_id)
                    .ok_or(PropellantBudgetError::MissingEngine {
                        engine_id: binding.engine_id,
                    })?;
            let fuel = tank_state(tanks, binding.fuel_tank)?;
            let oxidizer = match binding.oxidizer_tank {
                Some(id) => Some(tank_state(tanks, id)?),
                None => None,
            };
            let depleted = fuel.fluid_remaining_kg <= binding.residual_reserve_kg
                || oxidizer
                    .is_some_and(|tank| tank.fluid_remaining_kg <= binding.residual_reserve_kg);
            if depleted {
                shutdown.insert(binding.engine_id);
            }
            let feed_scale = if depleted {
                0.0
            } else {
                feed_pressure_scale(binding.feed, fuel, oxidizer)?
            };
            feed_scales.insert(binding.engine_id, feed_scale);

            if snapshot.mass_flow_kg_per_s.is_finite()
                && snapshot.mass_flow_kg_per_s > 0.0
                && matches!(snapshot.state, EngineState::Igniting | EngineState::Burning)
                && !depleted
            {
                let mdot = snapshot.mass_flow_kg_per_s;
                let fuel_rate = mdot / (1.0 + binding.oxidizer_fuel_ratio);
                add_drain(&mut drains, binding.fuel_tank, fuel_rate);
                if let Some(oxidizer_tank) = binding.oxidizer_tank {
                    let oxidizer_rate =
                        mdot * binding.oxidizer_fuel_ratio / (1.0 + binding.oxidizer_fuel_ratio);
                    add_drain(&mut drains, oxidizer_tank, oxidizer_rate);
                }
            }

            let ox_remaining = oxidizer.map_or(0.0, |tank| tank.fluid_remaining_kg);
            let initial_total = fuel.initial_fluid_mass_kg
                + oxidizer.map_or(0.0, |tank| tank.initial_fluid_mass_kg);
            let remaining_total = fuel.fluid_remaining_kg + ox_remaining;
            let utilization_fraction = if initial_total > 0.0 {
                ((initial_total - remaining_total) / initial_total).clamp(0.0, 1.0)
            } else {
                0.0
            };
            utilization.push(EnginePropellantUtilization {
                engine_id: binding.engine_id,
                fuel_residual_kg: fuel.fluid_remaining_kg,
                oxidizer_residual_kg: ox_remaining,
                utilization_fraction,
            });
        }
        Ok(PropellantBudgetReport {
            tank_drain_rates_kg_per_s: drains,
            shutdown_engines: shutdown,
            feed_pressure_scales: feed_scales,
            utilization,
        })
    }
}

fn tank_state(
    tanks: &BTreeMap<TankId, PropellantTankState>,
    id: TankId,
) -> Result<&PropellantTankState, PropellantBudgetError> {
    let tank = tanks
        .get(&id)
        .ok_or(PropellantBudgetError::MissingTank { tank_id: id })?;
    tank.require_valid()?;
    Ok(tank)
}

fn add_drain(drains: &mut BTreeMap<TankId, f64>, tank: TankId, rate: f64) {
    let entry = drains.entry(tank).or_insert(0.0);
    *entry += rate;
}

fn feed_pressure_scale(
    mode: FeedMode,
    fuel: &PropellantTankState,
    oxidizer: Option<&PropellantTankState>,
) -> Result<f64, PropellantBudgetError> {
    match mode {
        FeedMode::Regulated => Ok(1.0),
        FeedMode::Blowdown => {
            let fuel_scale = blowdown_tank_scale(fuel)?;
            let oxidizer_scale = match oxidizer {
                Some(tank) => blowdown_tank_scale(tank)?,
                None => fuel_scale,
            };
            Ok(fuel_scale.min(oxidizer_scale))
        }
    }
}

fn blowdown_tank_scale(tank: &PropellantTankState) -> Result<f64, PropellantBudgetError> {
    if !tank.has_ullage {
        return Err(PropellantBudgetError::InvalidTankState {
            reason: "blowdown feed requires a tank ullage declaration",
        });
    }
    let initial_propellant_volume_m3 = tank.initial_fluid_mass_kg / tank.density_kg_m3;
    let current_propellant_volume_m3 = tank.fluid_remaining_kg / tank.density_kg_m3;
    let initial_ullage_m3 = tank.volume_m3 - initial_propellant_volume_m3;
    let current_ullage_m3 = tank.volume_m3 - current_propellant_volume_m3;
    if initial_ullage_m3 <= 0.0 || current_ullage_m3 <= 0.0 {
        return Err(PropellantBudgetError::InvalidTankState {
            reason: "blowdown feed requires positive initial and current ullage volume",
        });
    }
    let scale = (initial_ullage_m3 / current_ullage_m3).powf(tank.ullage_gamma);
    if !scale.is_finite() || scale < 0.0 {
        return Err(PropellantBudgetError::InvalidTankState {
            reason: "blowdown feed pressure scale is non-finite",
        });
    }
    Ok(scale.min(1.0))
}

/// Errors raised by propellant-budget coupling.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum PropellantBudgetError {
    /// Binding is malformed.
    #[error("propellant budget binding invalid: {reason}")]
    InvalidBinding {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// Referenced engine snapshot is missing.
    #[error("propellant budget references missing engine id {engine_id:?}")]
    MissingEngine {
        /// Missing engine id.
        engine_id: EngineId,
    },
    /// Referenced tank state is missing.
    #[error("propellant budget references missing tank id {tank_id:?}")]
    MissingTank {
        /// Missing tank id.
        tank_id: TankId,
    },
    /// Tank state is invalid.
    #[error("propellant budget tank state invalid: {reason}")]
    InvalidTankState {
        /// Human-readable reason.
        reason: &'static str,
    },
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    fn engine_id() -> EngineId {
        EngineId::from_path("vehicle.assembly.engines.main")
    }

    fn tank_id(name: &str) -> TankId {
        TankId::from_path(&format!("vehicle.assembly.tanks.{name}"))
    }

    fn snapshot(mdot: f64) -> EngineSnapshot {
        EngineSnapshot {
            thrust_body: Vector3::new(0.0, 0.0, 1000.0),
            mass_flow_kg_per_s: mdot,
            consumed_kg: 0.0,
            state: EngineState::Burning,
        }
    }

    fn tank(mass: f64, initial: f64) -> PropellantTankState {
        PropellantTankState {
            fluid_remaining_kg: mass,
            initial_fluid_mass_kg: initial,
            volume_m3: 2.0,
            density_kg_m3: 1000.0,
            has_ullage: true,
            ullage_gamma: 1.2,
        }
    }

    #[test]
    fn mixture_ratio_splits_mass_flow() {
        let fuel = tank_id("fuel");
        let ox = tank_id("ox");
        let budget = PropellantBudget::new(vec![EnginePropellantBinding {
            engine_id: engine_id(),
            oxidizer_fuel_ratio: 2.0,
            fuel_tank: fuel,
            oxidizer_tank: Some(ox),
            feed: FeedMode::Regulated,
            residual_reserve_kg: 1.0,
        }])
        .unwrap();
        let engines = BTreeMap::from([(engine_id(), snapshot(3.0))]);
        let tanks = BTreeMap::from([(fuel, tank(10.0, 10.0)), (ox, tank(20.0, 20.0))]);
        let report = budget.evaluate(&engines, &tanks).unwrap();
        assert_eq!(
            report.tank_drain_rates_kg_per_s[&fuel].to_bits(),
            1.0_f64.to_bits()
        );
        assert_eq!(
            report.tank_drain_rates_kg_per_s[&ox].to_bits(),
            2.0_f64.to_bits()
        );
    }

    #[test]
    fn depletion_triggers_shutdown_and_zero_drain() {
        let fuel = tank_id("fuel");
        let budget = PropellantBudget::new(vec![EnginePropellantBinding {
            engine_id: engine_id(),
            oxidizer_fuel_ratio: 0.0,
            fuel_tank: fuel,
            oxidizer_tank: None,
            feed: FeedMode::Regulated,
            residual_reserve_kg: 1.0,
        }])
        .unwrap();
        let engines = BTreeMap::from([(engine_id(), snapshot(1.0))]);
        let tanks = BTreeMap::from([(fuel, tank(1.0, 10.0))]);
        let report = budget.evaluate(&engines, &tanks).unwrap();
        assert!(report.shutdown_engines.contains(&engine_id()));
        assert!(report.tank_drain_rates_kg_per_s.is_empty());
    }

    #[test]
    fn blowdown_scale_falls_as_ullage_expands() {
        let fuel = tank_id("fuel");
        let budget = PropellantBudget::new(vec![EnginePropellantBinding {
            engine_id: engine_id(),
            oxidizer_fuel_ratio: 0.0,
            fuel_tank: fuel,
            oxidizer_tank: None,
            feed: FeedMode::Blowdown,
            residual_reserve_kg: 0.0,
        }])
        .unwrap();
        let engines = BTreeMap::from([(engine_id(), snapshot(0.0))]);
        let tanks = BTreeMap::from([(fuel, tank(500.0, 1000.0))]);
        let report = budget.evaluate(&engines, &tanks).unwrap();
        let scale = report.feed_pressure_scales[&engine_id()];
        assert!(scale > 0.0 && scale < 1.0);
    }
}
