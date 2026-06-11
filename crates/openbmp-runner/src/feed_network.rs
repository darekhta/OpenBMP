//! Runner-side feed-network adapters.
//!
//! The feed-system crate produces reduced chamber pressure and mass-flow
//! snapshots. The current liquid-engine model consumes only a scalar
//! feed-pressure multiplier, so this adapter maps each configured network's
//! `pc / reference_pc` onto the existing propellant-budget report.

use std::collections::BTreeMap;

use openbmp_core::{EngineId, StepIndex};
use openbmp_feedsystem::{
    FeedCommand, FeedLegPressures, FeedNetwork, MocLine, MocLineBoundary, MocLineConfig,
    MocLineState, NormalizedPumpMap, PumpCavitationState, TankValveChamberConfig,
    TankValveChamberNetwork, ThrottleMixtureController, ThrottleMixtureControllerConfig,
    ThrottleMixtureControllerState, ThrottleMixtureMeasurement, ThrottleMixtureSetpoint,
    TransientChamberConfig, TransientChamberState, TransientDualValveFeedNetwork,
    TransientDualValveFeedNetworkConfig, TransientDualValveFeedNetworkState, Turbopump,
    TurbopumpConfig, TurbopumpDesignPoint, TurbopumpOperatingPoint, ValveCommandPair,
    ValveFeedLegConfig,
};
use openbmp_scenario::{
    PropulsionFeedNetworkConfig, PropulsionFeedNetworkControllerConfig,
    PropulsionFeedNetworkLineConfig, PropulsionFeedNetworkTurbopumpConfig,
    PropulsionMixtureRatioRunawayRuleConfig, ScenarioDocument,
};
use openbmp_vehicle::PropellantBudgetReport;

use crate::error::RunnerError;

/// Pump leg associated with a detected feed-network cavitation event.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FeedPumpLeg {
    /// Oxidizer-side pump leg.
    Oxidizer,
    /// Fuel-side pump leg.
    Fuel,
}

/// Deterministic pump-cavitation event exposed to the engine fault rack.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FeedPumpCavitationEvent {
    /// Engine whose feed-network pump is cavitating.
    pub engine_id: EngineId,
    /// Pump leg that is cavitating.
    pub leg: FeedPumpLeg,
}

#[derive(Debug)]
enum FeedNetworkEntry {
    TankValveChamber {
        engine_id: EngineId,
        network: TankValveChamberNetwork,
        reference_chamber_pressure_pa: f64,
        valve_open_fraction: f64,
    },
    TransientDualValveChamber {
        engine_id: EngineId,
        network: Box<TransientDualValveFeedNetwork>,
        state: TransientDualValveFeedNetworkState,
        base_oxidizer_pressure_pa: f64,
        base_fuel_pressure_pa: f64,
        reference_chamber_pressure_pa: f64,
        valve_commands: ValveCommandPair,
        pump_cavitation: FeedPumpCavitation,
        controller: Option<Box<FeedNetworkControllerRuntime>>,
        oxidizer_line: Option<Box<FeedLineRuntime>>,
        fuel_line: Option<Box<FeedLineRuntime>>,
    },
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct FeedPumpCavitation {
    oxidizer: Option<PumpCavitationState>,
    fuel: Option<PumpCavitationState>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct ValveCommandBias {
    oxidizer_open_fraction: f64,
    fuel_open_fraction: f64,
}

impl ValveCommandBias {
    fn apply_to(self, commands: &mut ValveCommandPair) {
        commands.oxidizer_open_fraction =
            (commands.oxidizer_open_fraction + self.oxidizer_open_fraction).clamp(0.0, 1.0);
        commands.fuel_open_fraction =
            (commands.fuel_open_fraction + self.fuel_open_fraction).clamp(0.0, 1.0);
    }
}

#[derive(Clone, Debug, PartialEq)]
struct MixtureRatioRunawayFaultRuntime {
    engine_id: EngineId,
    start_step: u64,
    oxidizer_open_fraction_rate_per_s: f64,
    fuel_open_fraction_rate_per_s: f64,
}

impl MixtureRatioRunawayFaultRuntime {
    fn from_config(config: &PropulsionMixtureRatioRunawayRuleConfig) -> Self {
        Self {
            engine_id: EngineId::from_path(&format!(
                "vehicle.assembly.engines.{engine_id}",
                engine_id = config.engine_id
            )),
            start_step: config.start_step,
            oxidizer_open_fraction_rate_per_s: config.oxidizer_open_fraction_rate_per_s,
            fuel_open_fraction_rate_per_s: config.fuel_open_fraction_rate_per_s,
        }
    }

    fn step_bias(&mut self, engine_id: EngineId, step: StepIndex, dt_s: f64) -> ValveCommandBias {
        if self.engine_id != engine_id || step.value() < self.start_step {
            return ValveCommandBias::default();
        }
        ValveCommandBias {
            oxidizer_open_fraction: self.oxidizer_open_fraction_rate_per_s * dt_s,
            fuel_open_fraction: self.fuel_open_fraction_rate_per_s * dt_s,
        }
    }
}

fn update_mixture_ratio_runaway_bias(
    faults: &mut [MixtureRatioRunawayFaultRuntime],
    engine_id: EngineId,
    step: StepIndex,
    dt_s: f64,
) -> ValveCommandBias {
    faults
        .iter_mut()
        .fold(ValveCommandBias::default(), |mut combined, fault| {
            let bias = fault.step_bias(engine_id, step, dt_s);
            combined.oxidizer_open_fraction += bias.oxidizer_open_fraction;
            combined.fuel_open_fraction += bias.fuel_open_fraction;
            combined
        })
}

#[derive(Debug)]
struct FeedLineRuntime {
    line: MocLine,
    state: MocLineState,
    boundary: MocLineBoundary,
    baseline_downstream_pressure_pa: f64,
}

impl FeedLineRuntime {
    fn new(config: &PropulsionFeedNetworkLineConfig, dt_s: f64) -> Result<Self, RunnerError> {
        let line_config = MocLineConfig {
            length_m: config.length_m,
            wave_speed_m_s: config.wave_speed_m_s,
            density_kg_m3: config.density_kg_m3,
            cross_section_area_m2: config.cross_section_area_m2,
            segment_count: config.segment_count,
        };
        let line = MocLine::new(line_config)?;
        let courant_dt_s = line.courant_dt_s();
        let tolerance = 1.0e-12_f64.max(courant_dt_s.abs() * 1.0e-9);
        if (dt_s - courant_dt_s).abs() > tolerance {
            return Err(openbmp_feedsystem::FeedSystemError::InvalidParameter {
                reason: "MOC feed-line Courant time step must match scenario time.dt_s",
            }
            .into());
        }
        let baseline_downstream_pressure_pa =
            line_config.pressure_from_head_pa(config.initial_head_m);
        if !baseline_downstream_pressure_pa.is_finite() {
            return Err(openbmp_feedsystem::FeedSystemError::NonFinite {
                reason: "MOC feed-line baseline pressure is non-finite",
            }
            .into());
        }
        Ok(Self {
            line,
            state: MocLineState::uniform(
                line_config,
                config.initial_head_m,
                config.initial_velocity_m_s,
            )?,
            boundary: MocLineBoundary {
                upstream_head_m: config.upstream_head_m,
                downstream_velocity_m_s: config.downstream_velocity_m_s,
            },
            baseline_downstream_pressure_pa,
        })
    }

    fn feed_pressure_pa(&mut self, base_pressure_pa: f64) -> Result<f64, RunnerError> {
        let snapshot = self.line.step(&self.state, self.boundary)?;
        self.state = snapshot.state;
        let pressure_pa = base_pressure_pa + snapshot.downstream_pressure_pa
            - self.baseline_downstream_pressure_pa;
        if !pressure_pa.is_finite() {
            return Err(openbmp_feedsystem::FeedSystemError::NonFinite {
                reason: "line-coupled feed pressure is non-finite",
            }
            .into());
        }
        if pressure_pa < 0.0 {
            return Err(openbmp_feedsystem::FeedSystemError::InvalidParameter {
                reason: "line-coupled feed pressure became negative",
            }
            .into());
        }
        Ok(pressure_pa)
    }
}

#[derive(Debug)]
struct FeedNetworkControllerRuntime {
    controller: ThrottleMixtureController,
    state: ThrottleMixtureControllerState,
    setpoint: ThrottleMixtureSetpoint,
    latest_mixture_ratio: Option<f64>,
}

impl FeedNetworkControllerRuntime {
    fn new(
        config: &PropulsionFeedNetworkControllerConfig,
        initial_commands: ValveCommandPair,
    ) -> Result<Self, RunnerError> {
        Ok(Self {
            controller: ThrottleMixtureController::new(ThrottleMixtureControllerConfig {
                pressure_proportional_gain_per_pa: config.pressure_proportional_gain_per_pa,
                pressure_integral_gain_per_pa_s: config.pressure_integral_gain_per_pa_s,
                pressure_integral_limit_pa_s: config.pressure_integral_limit_pa_s,
                mixture_proportional_gain: config.mixture_proportional_gain,
                mixture_integral_gain_per_s: config.mixture_integral_gain_per_s,
                mixture_integral_limit_s: config.mixture_integral_limit_s,
                min_open_fraction: config.min_open_fraction,
                max_open_fraction: config.max_open_fraction,
                max_open_fraction_slew_per_s: config.max_open_fraction_slew_per_s,
            })?,
            state: ThrottleMixtureControllerState::new(initial_commands),
            setpoint: ThrottleMixtureSetpoint {
                chamber_pressure_pa: config.target_chamber_pressure_pa,
                mixture_ratio: config.target_mixture_ratio,
            },
            latest_mixture_ratio: config.target_mixture_ratio,
        })
    }

    fn update_commands(
        &mut self,
        chamber_pressure_pa: f64,
        dt_s: f64,
    ) -> Result<ValveCommandPair, RunnerError> {
        let snapshot = self.controller.step(
            self.state,
            self.setpoint,
            ThrottleMixtureMeasurement {
                chamber_pressure_pa,
                mixture_ratio: self.latest_mixture_ratio,
            },
            dt_s,
        )?;
        self.state = snapshot.state;
        Ok(snapshot.state.valve_commands)
    }

    fn update_mixture_ratio(&mut self, mixture_ratio: Option<f64>) {
        if mixture_ratio.is_some() {
            self.latest_mixture_ratio = mixture_ratio;
        }
    }
}

fn pump_augmented_pressure_pa(
    base_pressure_pa: f64,
    pump: Option<&PropulsionFeedNetworkTurbopumpConfig>,
) -> Result<(f64, Option<PumpCavitationState>), RunnerError> {
    let Some(pump) = pump else {
        return Ok((base_pressure_pa, None));
    };
    let snapshot = Turbopump::new(TurbopumpConfig {
        design: TurbopumpDesignPoint {
            volumetric_flow_m3_per_s: pump.design_volumetric_flow_m3_per_s,
            pressure_rise_pa: pump.design_pressure_rise_pa,
            shaft_speed_rad_per_s: pump.design_shaft_speed_rad_per_s,
            fluid_density_kg_m3: pump.fluid_density_kg_m3,
            efficiency: pump.design_efficiency,
            required_npsh_m: pump.required_npsh_m,
            specific_speed: pump.specific_speed,
        },
        map: NormalizedPumpMap {
            head_coefficients: pump.head_coefficients,
            efficiency_coefficients: pump.efficiency_coefficients,
            cavitation_head_multiplier: pump.cavitation_head_multiplier,
        },
    })?
    .solve(TurbopumpOperatingPoint {
        volumetric_flow_m3_per_s: pump.operating_volumetric_flow_m3_per_s,
        shaft_speed_rad_per_s: pump.operating_shaft_speed_rad_per_s,
        suction_pressure_pa: pump.suction_pressure_pa,
        vapor_pressure_pa: pump.vapor_pressure_pa,
    })?;
    let pressure_pa = base_pressure_pa + snapshot.pressure_rise_pa;
    if !pressure_pa.is_finite() {
        return Err(openbmp_feedsystem::FeedSystemError::NonFinite {
            reason: "pump-augmented feed pressure is non-finite",
        }
        .into());
    }
    Ok((pressure_pa, Some(snapshot.cavitation)))
}

impl FeedNetworkEntry {
    fn engine_id(&self) -> EngineId {
        match self {
            Self::TankValveChamber { engine_id, .. } => *engine_id,
            Self::TransientDualValveChamber { engine_id, .. } => *engine_id,
        }
    }

    fn append_cavitation_events(&self, events: &mut Vec<FeedPumpCavitationEvent>) {
        let Self::TransientDualValveChamber {
            engine_id,
            pump_cavitation,
            ..
        } = self
        else {
            return;
        };
        if matches!(
            pump_cavitation.oxidizer,
            Some(PumpCavitationState::Cavitating)
        ) {
            events.push(FeedPumpCavitationEvent {
                engine_id: *engine_id,
                leg: FeedPumpLeg::Oxidizer,
            });
        }
        if matches!(pump_cavitation.fuel, Some(PumpCavitationState::Cavitating)) {
            events.push(FeedPumpCavitationEvent {
                engine_id: *engine_id,
                leg: FeedPumpLeg::Fuel,
            });
        }
    }

    fn feed_scale(&mut self, dt_s: f64, valve_bias: ValveCommandBias) -> Result<f64, RunnerError> {
        match self {
            Self::TankValveChamber {
                network,
                reference_chamber_pressure_pa,
                valve_open_fraction,
                ..
            } => {
                let snapshot = network.solve(FeedCommand {
                    valve_open_fraction: *valve_open_fraction,
                })?;
                Ok(snapshot.chamber_pressure_pa / *reference_chamber_pressure_pa)
            }
            Self::TransientDualValveChamber {
                network,
                state,
                base_oxidizer_pressure_pa,
                base_fuel_pressure_pa,
                reference_chamber_pressure_pa,
                valve_commands,
                controller,
                oxidizer_line,
                fuel_line,
                ..
            } => {
                if let Some(controller) = controller {
                    *valve_commands =
                        controller.update_commands(state.chamber.pressure_pa, dt_s)?;
                }
                valve_bias.apply_to(valve_commands);
                if let Some(controller) = controller {
                    controller.state = ThrottleMixtureControllerState::new(*valve_commands);
                }
                let oxidizer_pressure_pa = if let Some(line) = oxidizer_line {
                    line.feed_pressure_pa(*base_oxidizer_pressure_pa)?
                } else {
                    *base_oxidizer_pressure_pa
                };
                let fuel_pressure_pa = if let Some(line) = fuel_line {
                    line.feed_pressure_pa(*base_fuel_pressure_pa)?
                } else {
                    *base_fuel_pressure_pa
                };
                let snapshot = network.step_with_feed_pressures(
                    *state,
                    *valve_commands,
                    FeedLegPressures {
                        oxidizer_pressure_pa,
                        fuel_pressure_pa,
                    },
                    dt_s,
                )?;
                *state = snapshot.state;
                if let Some(controller) = controller {
                    controller.update_mixture_ratio(snapshot.chamber.mixture_ratio);
                }
                Ok(snapshot.chamber.chamber_pressure_pa / *reference_chamber_pressure_pa)
            }
        }
    }
}

/// Runner-side collection of opt-in feed-network overrides.
#[derive(Debug, Default)]
pub struct FeedNetworkRack {
    entries: Vec<FeedNetworkEntry>,
    mixture_ratio_runaway_faults: Vec<MixtureRatioRunawayFaultRuntime>,
}

impl FeedNetworkRack {
    /// Build from `[propulsion.feed_network]` declarations.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when the reduced feed-network config fails
    /// validation in `openbmp-feedsystem`.
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let Some(propulsion) = &document.propulsion else {
            return Ok(Self::default());
        };
        let mut entries = Vec::with_capacity(propulsion.feed_networks.len());
        for config in &propulsion.feed_networks {
            match config {
                PropulsionFeedNetworkConfig::TankValveChamber {
                    engine_id,
                    tank_pressure_pa,
                    propellant_density_kg_m3,
                    valve_area_m2,
                    valve_discharge_coefficient,
                    throat_area_m2,
                    c_star_m_s,
                    reference_chamber_pressure_pa,
                    valve_open_fraction,
                } => {
                    let network = TankValveChamberNetwork::new(TankValveChamberConfig {
                        tank_pressure_pa: *tank_pressure_pa,
                        propellant_density_kg_m3: *propellant_density_kg_m3,
                        valve_area_m2: *valve_area_m2,
                        valve_discharge_coefficient: *valve_discharge_coefficient,
                        throat_area_m2: *throat_area_m2,
                        c_star_m_s: *c_star_m_s,
                    })?;
                    entries.push(FeedNetworkEntry::TankValveChamber {
                        engine_id: EngineId::from_path(&format!(
                            "vehicle.assembly.engines.{engine_id}"
                        )),
                        network,
                        reference_chamber_pressure_pa: *reference_chamber_pressure_pa,
                        valve_open_fraction: *valve_open_fraction,
                    });
                }
                PropulsionFeedNetworkConfig::TransientDualValveChamber {
                    engine_id,
                    oxidizer_tank_pressure_pa,
                    fuel_tank_pressure_pa,
                    oxidizer_density_kg_m3,
                    fuel_density_kg_m3,
                    oxidizer_valve_area_m2,
                    fuel_valve_area_m2,
                    oxidizer_valve_discharge_coefficient,
                    fuel_valve_discharge_coefficient,
                    oxidizer_pump,
                    fuel_pump,
                    oxidizer_line,
                    fuel_line,
                    chamber_volume_m3,
                    gas_temperature_k,
                    gas_constant_j_per_kg_k,
                    throat_area_m2,
                    c_star_m_s,
                    initial_chamber_pressure_pa,
                    reference_chamber_pressure_pa,
                    oxidizer_open_fraction,
                    fuel_open_fraction,
                    controller,
                } => {
                    let (oxidizer_feed_pressure_pa, oxidizer_cavitation) =
                        pump_augmented_pressure_pa(
                            *oxidizer_tank_pressure_pa,
                            oxidizer_pump.as_ref(),
                        )?;
                    let (fuel_feed_pressure_pa, fuel_cavitation) =
                        pump_augmented_pressure_pa(*fuel_tank_pressure_pa, fuel_pump.as_ref())?;
                    let oxidizer_line = oxidizer_line
                        .as_ref()
                        .map(|config| FeedLineRuntime::new(config, document.time.dt_s))
                        .map(|runtime| runtime.map(Box::new))
                        .transpose()?;
                    let fuel_line = fuel_line
                        .as_ref()
                        .map(|config| FeedLineRuntime::new(config, document.time.dt_s))
                        .map(|runtime| runtime.map(Box::new))
                        .transpose()?;
                    let network =
                        TransientDualValveFeedNetwork::new(TransientDualValveFeedNetworkConfig {
                            oxidizer: ValveFeedLegConfig {
                                tank_pressure_pa: oxidizer_feed_pressure_pa,
                                propellant_density_kg_m3: *oxidizer_density_kg_m3,
                                valve_area_m2: *oxidizer_valve_area_m2,
                                valve_discharge_coefficient: *oxidizer_valve_discharge_coefficient,
                            },
                            fuel: ValveFeedLegConfig {
                                tank_pressure_pa: fuel_feed_pressure_pa,
                                propellant_density_kg_m3: *fuel_density_kg_m3,
                                valve_area_m2: *fuel_valve_area_m2,
                                valve_discharge_coefficient: *fuel_valve_discharge_coefficient,
                            },
                            chamber: TransientChamberConfig {
                                chamber_volume_m3: *chamber_volume_m3,
                                gas_temperature_k: *gas_temperature_k,
                                gas_constant_j_per_kg_k: *gas_constant_j_per_kg_k,
                                throat_area_m2: *throat_area_m2,
                                c_star_m_s: *c_star_m_s,
                            },
                        })?;
                    let valve_commands = ValveCommandPair {
                        oxidizer_open_fraction: *oxidizer_open_fraction,
                        fuel_open_fraction: *fuel_open_fraction,
                    };
                    entries.push(FeedNetworkEntry::TransientDualValveChamber {
                        engine_id: EngineId::from_path(&format!(
                            "vehicle.assembly.engines.{engine_id}"
                        )),
                        network: Box::new(network),
                        state: TransientDualValveFeedNetworkState {
                            chamber: TransientChamberState {
                                pressure_pa: *initial_chamber_pressure_pa,
                            },
                        },
                        base_oxidizer_pressure_pa: oxidizer_feed_pressure_pa,
                        base_fuel_pressure_pa: fuel_feed_pressure_pa,
                        reference_chamber_pressure_pa: *reference_chamber_pressure_pa,
                        valve_commands,
                        pump_cavitation: FeedPumpCavitation {
                            oxidizer: oxidizer_cavitation,
                            fuel: fuel_cavitation,
                        },
                        controller: controller
                            .as_ref()
                            .map(|config| FeedNetworkControllerRuntime::new(config, valve_commands))
                            .map(|runtime| runtime.map(Box::new))
                            .transpose()?,
                        oxidizer_line,
                        fuel_line,
                    });
                }
            }
        }
        let mixture_ratio_runaway_faults = propulsion
            .faults
            .as_ref()
            .map(|faults| {
                faults
                    .mixture_ratio_runaway_rules
                    .iter()
                    .map(MixtureRatioRunawayFaultRuntime::from_config)
                    .collect()
            })
            .unwrap_or_default();
        Ok(Self {
            entries,
            mixture_ratio_runaway_faults,
        })
    }

    /// `true` when no feed-network overrides were configured.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Current pump-cavitation events exposed by configured feed networks.
    #[must_use]
    pub fn cavitation_events(&self) -> Vec<FeedPumpCavitationEvent> {
        let mut events = Vec::new();
        for entry in &self.entries {
            entry.append_cavitation_events(&mut events);
        }
        events
    }

    /// Override matching feed-pressure scales in the propellant-budget report.
    ///
    /// Depletion shutdowns keep priority: if the budget already set a referenced
    /// engine scale to zero because a tank is depleted, the feed network leaves it
    /// at zero and the shutdown list remains authoritative.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] if a configured network fails its algebraic solve.
    pub fn apply_to_report(
        &mut self,
        report: &mut PropellantBudgetReport,
        dt_s: f64,
        step: StepIndex,
    ) -> Result<(), RunnerError> {
        let faults = &mut self.mixture_ratio_runaway_faults;
        for entry in &mut self.entries {
            let engine_id = entry.engine_id();
            if report.shutdown_engines.contains(&engine_id) {
                report.feed_pressure_scales.insert(engine_id, 0.0);
                continue;
            }
            let valve_bias = update_mixture_ratio_runaway_bias(faults, engine_id, step, dt_s);
            report
                .feed_pressure_scales
                .insert(engine_id, entry.feed_scale(dt_s, valve_bias)?);
        }
        Ok(())
    }

    /// Deterministic feed scales for diagnostics and focused tests.
    #[must_use]
    pub fn configured_engine_ids(&self) -> BTreeMap<EngineId, usize> {
        self.entries
            .iter()
            .enumerate()
            .map(|(index, entry)| (entry.engine_id(), index))
            .collect()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::collections::BTreeSet;

    use openbmp_core::TankId;

    use super::*;

    fn engine_id() -> EngineId {
        EngineId::from_path("vehicle.assembly.engines.main")
    }

    fn rack(reference_chamber_pressure_pa: f64) -> FeedNetworkRack {
        FeedNetworkRack {
            entries: vec![FeedNetworkEntry::TankValveChamber {
                engine_id: engine_id(),
                network: TankValveChamberNetwork::new(TankValveChamberConfig {
                    tank_pressure_pa: 4.0e6,
                    propellant_density_kg_m3: 810.0,
                    valve_area_m2: 8.0e-5,
                    valve_discharge_coefficient: 0.72,
                    throat_area_m2: 1.2e-4,
                    c_star_m_s: 1_600.0,
                })
                .unwrap(),
                reference_chamber_pressure_pa,
                valve_open_fraction: 1.0,
            }],
            mixture_ratio_runaway_faults: Vec::new(),
        }
    }

    fn report(scale: f64) -> PropellantBudgetReport {
        let mut feed_pressure_scales = BTreeMap::new();
        feed_pressure_scales.insert(engine_id(), scale);
        PropellantBudgetReport {
            tank_drain_rates_kg_per_s: BTreeMap::<TankId, f64>::new(),
            shutdown_engines: BTreeSet::new(),
            feed_pressure_scales,
            utilization: Vec::new(),
        }
    }

    fn pump_config() -> PropulsionFeedNetworkTurbopumpConfig {
        PropulsionFeedNetworkTurbopumpConfig {
            design_volumetric_flow_m3_per_s: 0.05,
            design_pressure_rise_pa: 6.0e6,
            design_shaft_speed_rad_per_s: 3_000.0,
            fluid_density_kg_m3: 810.0,
            design_efficiency: 0.70,
            required_npsh_m: 20.0,
            specific_speed: 0.8,
            head_coefficients: [1.2, -0.2, 0.0],
            efficiency_coefficients: [0.8, 0.4, -0.2],
            cavitation_head_multiplier: 0.25,
            operating_volumetric_flow_m3_per_s: 0.05,
            operating_shaft_speed_rad_per_s: 3_000.0,
            suction_pressure_pa: 1.0e6,
            vapor_pressure_pa: 30_000.0,
        }
    }

    fn line_config() -> PropulsionFeedNetworkLineConfig {
        PropulsionFeedNetworkLineConfig {
            length_m: 40.0,
            wave_speed_m_s: 1_000.0,
            density_kg_m3: 1_000.0,
            cross_section_area_m2: 0.01,
            segment_count: 4,
            initial_head_m: 100.0,
            initial_velocity_m_s: 2.0,
            upstream_head_m: 100.0,
            downstream_velocity_m_s: 0.0,
        }
    }

    fn transient_network() -> TransientDualValveFeedNetwork {
        TransientDualValveFeedNetwork::new(TransientDualValveFeedNetworkConfig {
            oxidizer: ValveFeedLegConfig {
                tank_pressure_pa: 4.0e6,
                propellant_density_kg_m3: 810.0,
                valve_area_m2: 8.0e-5,
                valve_discharge_coefficient: 0.72,
            },
            fuel: ValveFeedLegConfig {
                tank_pressure_pa: 4.0e6,
                propellant_density_kg_m3: 810.0,
                valve_area_m2: 4.0e-5,
                valve_discharge_coefficient: 0.72,
            },
            chamber: TransientChamberConfig {
                chamber_volume_m3: 0.08,
                gas_temperature_k: 3_400.0,
                gas_constant_j_per_kg_k: 360.0,
                throat_area_m2: 1.2e-4,
                c_star_m_s: 1_600.0,
            },
        })
        .unwrap()
    }

    #[test]
    fn feed_network_overrides_existing_feed_pressure_scale() {
        let mut report = report(1.0);
        rack(4.0e6)
            .apply_to_report(&mut report, 0.1, StepIndex::ZERO)
            .unwrap();

        let scale = report.feed_pressure_scales[&engine_id()];
        assert!(scale > 0.0);
        assert!(scale < 1.0);
    }

    #[test]
    fn depleted_engine_keeps_zero_feed_pressure_scale() {
        let mut report = report(1.0);
        report.shutdown_engines.insert(engine_id());
        rack(4.0e6)
            .apply_to_report(&mut report, 0.1, StepIndex::ZERO)
            .unwrap();

        assert_eq!(
            report.feed_pressure_scales[&engine_id()].to_bits(),
            0.0_f64.to_bits()
        );
    }

    #[test]
    fn transient_feed_network_advances_chamber_pressure_scale() {
        let mut rack = FeedNetworkRack {
            entries: vec![FeedNetworkEntry::TransientDualValveChamber {
                engine_id: engine_id(),
                network: Box::new(
                    TransientDualValveFeedNetwork::new(TransientDualValveFeedNetworkConfig {
                        oxidizer: ValveFeedLegConfig {
                            tank_pressure_pa: 4.0e6,
                            propellant_density_kg_m3: 810.0,
                            valve_area_m2: 8.0e-5,
                            valve_discharge_coefficient: 0.72,
                        },
                        fuel: ValveFeedLegConfig {
                            tank_pressure_pa: 4.0e6,
                            propellant_density_kg_m3: 810.0,
                            valve_area_m2: 4.0e-5,
                            valve_discharge_coefficient: 0.72,
                        },
                        chamber: TransientChamberConfig {
                            chamber_volume_m3: 0.08,
                            gas_temperature_k: 3_400.0,
                            gas_constant_j_per_kg_k: 360.0,
                            throat_area_m2: 1.2e-4,
                            c_star_m_s: 1_600.0,
                        },
                    })
                    .unwrap(),
                ),
                state: TransientDualValveFeedNetworkState {
                    chamber: TransientChamberState { pressure_pa: 0.0 },
                },
                base_oxidizer_pressure_pa: 4.0e6,
                base_fuel_pressure_pa: 4.0e6,
                reference_chamber_pressure_pa: 4.0e6,
                valve_commands: ValveCommandPair {
                    oxidizer_open_fraction: 1.0,
                    fuel_open_fraction: 1.0,
                },
                pump_cavitation: FeedPumpCavitation::default(),
                controller: None,
                oxidizer_line: None,
                fuel_line: None,
            }],
            mixture_ratio_runaway_faults: Vec::new(),
        };
        let mut report = report(1.0);
        rack.apply_to_report(&mut report, 0.01, StepIndex::ZERO)
            .unwrap();
        let first_scale = report.feed_pressure_scales[&engine_id()];
        rack.apply_to_report(&mut report, 0.01, StepIndex::new(1))
            .unwrap();
        let second_scale = report.feed_pressure_scales[&engine_id()];

        assert!(first_scale > 0.0);
        assert!(second_scale > first_scale);
    }

    #[test]
    fn transient_feed_network_controller_updates_valve_commands() {
        let initial_commands = ValveCommandPair {
            oxidizer_open_fraction: 0.2,
            fuel_open_fraction: 0.2,
        };
        let controller_config = PropulsionFeedNetworkControllerConfig {
            target_chamber_pressure_pa: 4.0e6,
            target_mixture_ratio: Some(2.0),
            pressure_proportional_gain_per_pa: 1.0e-7,
            pressure_integral_gain_per_pa_s: 0.0,
            pressure_integral_limit_pa_s: 1.0e7,
            mixture_proportional_gain: 0.0,
            mixture_integral_gain_per_s: 0.0,
            mixture_integral_limit_s: 10.0,
            min_open_fraction: 0.0,
            max_open_fraction: 1.0,
            max_open_fraction_slew_per_s: 10.0,
        };
        let mut rack = FeedNetworkRack {
            entries: vec![FeedNetworkEntry::TransientDualValveChamber {
                engine_id: engine_id(),
                network: Box::new(
                    TransientDualValveFeedNetwork::new(TransientDualValveFeedNetworkConfig {
                        oxidizer: ValveFeedLegConfig {
                            tank_pressure_pa: 4.0e6,
                            propellant_density_kg_m3: 810.0,
                            valve_area_m2: 8.0e-5,
                            valve_discharge_coefficient: 0.72,
                        },
                        fuel: ValveFeedLegConfig {
                            tank_pressure_pa: 4.0e6,
                            propellant_density_kg_m3: 810.0,
                            valve_area_m2: 4.0e-5,
                            valve_discharge_coefficient: 0.72,
                        },
                        chamber: TransientChamberConfig {
                            chamber_volume_m3: 0.08,
                            gas_temperature_k: 3_400.0,
                            gas_constant_j_per_kg_k: 360.0,
                            throat_area_m2: 1.2e-4,
                            c_star_m_s: 1_600.0,
                        },
                    })
                    .unwrap(),
                ),
                state: TransientDualValveFeedNetworkState {
                    chamber: TransientChamberState { pressure_pa: 0.0 },
                },
                base_oxidizer_pressure_pa: 4.0e6,
                base_fuel_pressure_pa: 4.0e6,
                reference_chamber_pressure_pa: 4.0e6,
                valve_commands: initial_commands,
                pump_cavitation: FeedPumpCavitation::default(),
                controller: Some(Box::new(
                    FeedNetworkControllerRuntime::new(&controller_config, initial_commands)
                        .unwrap(),
                )),
                oxidizer_line: None,
                fuel_line: None,
            }],
            mixture_ratio_runaway_faults: Vec::new(),
        };
        let mut report = report(1.0);
        rack.apply_to_report(&mut report, 0.1, StepIndex::ZERO)
            .unwrap();

        match &rack.entries[0] {
            FeedNetworkEntry::TransientDualValveChamber { valve_commands, .. } => {
                assert!(valve_commands.oxidizer_open_fraction > 0.2);
                assert!(valve_commands.fuel_open_fraction > 0.2);
            }
            FeedNetworkEntry::TankValveChamber { .. } => panic!("unexpected equilibrium network"),
        }
    }

    #[test]
    fn turbopump_augments_transient_feed_pressure() {
        let (pressure, cavitation) =
            pump_augmented_pressure_pa(4.0e6, Some(&pump_config())).unwrap();

        assert!(pressure > 4.0e6);
        assert_eq!(pressure.to_bits(), 10.0e6_f64.to_bits());
        assert_eq!(cavitation, Some(PumpCavitationState::Nominal));
    }

    #[test]
    fn feed_network_reports_pump_cavitation_events() {
        let rack = FeedNetworkRack {
            entries: vec![FeedNetworkEntry::TransientDualValveChamber {
                engine_id: engine_id(),
                network: Box::new(transient_network()),
                state: TransientDualValveFeedNetworkState {
                    chamber: TransientChamberState { pressure_pa: 0.0 },
                },
                base_oxidizer_pressure_pa: 4.0e6,
                base_fuel_pressure_pa: 4.0e6,
                reference_chamber_pressure_pa: 4.0e6,
                valve_commands: ValveCommandPair {
                    oxidizer_open_fraction: 1.0,
                    fuel_open_fraction: 1.0,
                },
                pump_cavitation: FeedPumpCavitation {
                    oxidizer: Some(PumpCavitationState::Cavitating),
                    fuel: Some(PumpCavitationState::Nominal),
                },
                controller: None,
                oxidizer_line: None,
                fuel_line: None,
            }],
            mixture_ratio_runaway_faults: Vec::new(),
        };

        assert_eq!(
            rack.cavitation_events(),
            vec![FeedPumpCavitationEvent {
                engine_id: engine_id(),
                leg: FeedPumpLeg::Oxidizer,
            }]
        );
    }

    #[test]
    fn mixture_ratio_runaway_drifts_valve_commands_after_start_step() {
        let mut rack = FeedNetworkRack {
            entries: vec![FeedNetworkEntry::TransientDualValveChamber {
                engine_id: engine_id(),
                network: Box::new(transient_network()),
                state: TransientDualValveFeedNetworkState {
                    chamber: TransientChamberState { pressure_pa: 0.0 },
                },
                base_oxidizer_pressure_pa: 4.0e6,
                base_fuel_pressure_pa: 4.0e6,
                reference_chamber_pressure_pa: 4.0e6,
                valve_commands: ValveCommandPair {
                    oxidizer_open_fraction: 0.5,
                    fuel_open_fraction: 0.5,
                },
                pump_cavitation: FeedPumpCavitation::default(),
                controller: None,
                oxidizer_line: None,
                fuel_line: None,
            }],
            mixture_ratio_runaway_faults: vec![MixtureRatioRunawayFaultRuntime {
                engine_id: engine_id(),
                start_step: 2,
                oxidizer_open_fraction_rate_per_s: 0.4,
                fuel_open_fraction_rate_per_s: -0.2,
            }],
        };
        let mut report = report(1.0);

        rack.apply_to_report(&mut report, 0.1, StepIndex::new(1))
            .unwrap();
        match &rack.entries[0] {
            FeedNetworkEntry::TransientDualValveChamber { valve_commands, .. } => {
                assert_eq!(
                    valve_commands.oxidizer_open_fraction.to_bits(),
                    0.5_f64.to_bits()
                );
                assert_eq!(
                    valve_commands.fuel_open_fraction.to_bits(),
                    0.5_f64.to_bits()
                );
            }
            FeedNetworkEntry::TankValveChamber { .. } => panic!("unexpected equilibrium network"),
        }

        rack.apply_to_report(&mut report, 0.1, StepIndex::new(2))
            .unwrap();
        match &rack.entries[0] {
            FeedNetworkEntry::TransientDualValveChamber { valve_commands, .. } => {
                assert!((valve_commands.oxidizer_open_fraction - 0.54).abs() < 1.0e-12);
                assert!((valve_commands.fuel_open_fraction - 0.48).abs() < 1.0e-12);
            }
            FeedNetworkEntry::TankValveChamber { .. } => panic!("unexpected equilibrium network"),
        }
    }

    #[test]
    fn moc_line_runtime_applies_joukowsky_pressure_delta() {
        let mut runtime = FeedLineRuntime::new(&line_config(), 0.01).unwrap();

        let pressure_pa = runtime.feed_pressure_pa(4.0e6).unwrap();

        assert!((pressure_pa - 6.0e6).abs() < 1.0e-6);
    }

    #[test]
    fn moc_line_runtime_rejects_non_courant_runner_dt() {
        assert!(FeedLineRuntime::new(&line_config(), 0.02).is_err());
    }

    #[test]
    fn moc_line_coupled_transient_network_builds_pressure_faster() {
        let mut base_entry = FeedNetworkEntry::TransientDualValveChamber {
            engine_id: engine_id(),
            network: Box::new(transient_network()),
            state: TransientDualValveFeedNetworkState {
                chamber: TransientChamberState { pressure_pa: 0.0 },
            },
            base_oxidizer_pressure_pa: 4.0e6,
            base_fuel_pressure_pa: 4.0e6,
            reference_chamber_pressure_pa: 4.0e6,
            valve_commands: ValveCommandPair {
                oxidizer_open_fraction: 1.0,
                fuel_open_fraction: 1.0,
            },
            pump_cavitation: FeedPumpCavitation::default(),
            controller: None,
            oxidizer_line: None,
            fuel_line: None,
        };
        let mut line_entry = FeedNetworkEntry::TransientDualValveChamber {
            engine_id: engine_id(),
            network: Box::new(transient_network()),
            state: TransientDualValveFeedNetworkState {
                chamber: TransientChamberState { pressure_pa: 0.0 },
            },
            base_oxidizer_pressure_pa: 4.0e6,
            base_fuel_pressure_pa: 4.0e6,
            reference_chamber_pressure_pa: 4.0e6,
            valve_commands: ValveCommandPair {
                oxidizer_open_fraction: 1.0,
                fuel_open_fraction: 1.0,
            },
            pump_cavitation: FeedPumpCavitation::default(),
            controller: None,
            oxidizer_line: Some(Box::new(
                FeedLineRuntime::new(&line_config(), 0.01).unwrap(),
            )),
            fuel_line: None,
        };

        let base_scale = base_entry
            .feed_scale(0.01, ValveCommandBias::default())
            .unwrap();
        let line_scale = line_entry
            .feed_scale(0.01, ValveCommandBias::default())
            .unwrap();

        assert!(line_scale > base_scale);
    }

    #[test]
    fn pump_fed_transient_network_builds_pressure_faster() {
        let (oxidizer_pressure_pa, _) =
            pump_augmented_pressure_pa(4.0e6, Some(&pump_config())).unwrap();
        let base_network =
            TransientDualValveFeedNetwork::new(TransientDualValveFeedNetworkConfig {
                oxidizer: ValveFeedLegConfig {
                    tank_pressure_pa: 4.0e6,
                    propellant_density_kg_m3: 810.0,
                    valve_area_m2: 8.0e-5,
                    valve_discharge_coefficient: 0.72,
                },
                fuel: ValveFeedLegConfig {
                    tank_pressure_pa: 4.0e6,
                    propellant_density_kg_m3: 810.0,
                    valve_area_m2: 4.0e-5,
                    valve_discharge_coefficient: 0.72,
                },
                chamber: TransientChamberConfig {
                    chamber_volume_m3: 0.08,
                    gas_temperature_k: 3_400.0,
                    gas_constant_j_per_kg_k: 360.0,
                    throat_area_m2: 1.2e-4,
                    c_star_m_s: 1_600.0,
                },
            })
            .unwrap();
        let pump_network =
            TransientDualValveFeedNetwork::new(TransientDualValveFeedNetworkConfig {
                oxidizer: ValveFeedLegConfig {
                    tank_pressure_pa: oxidizer_pressure_pa,
                    propellant_density_kg_m3: 810.0,
                    valve_area_m2: 8.0e-5,
                    valve_discharge_coefficient: 0.72,
                },
                fuel: ValveFeedLegConfig {
                    tank_pressure_pa: 4.0e6,
                    propellant_density_kg_m3: 810.0,
                    valve_area_m2: 4.0e-5,
                    valve_discharge_coefficient: 0.72,
                },
                chamber: TransientChamberConfig {
                    chamber_volume_m3: 0.08,
                    gas_temperature_k: 3_400.0,
                    gas_constant_j_per_kg_k: 360.0,
                    throat_area_m2: 1.2e-4,
                    c_star_m_s: 1_600.0,
                },
            })
            .unwrap();
        let state = TransientDualValveFeedNetworkState {
            chamber: TransientChamberState { pressure_pa: 0.0 },
        };
        let commands = ValveCommandPair {
            oxidizer_open_fraction: 1.0,
            fuel_open_fraction: 1.0,
        };
        let base = base_network.step(state, commands, 0.01).unwrap();
        let pumped = pump_network.step(state, commands, 0.01).unwrap();

        assert!(pumped.chamber.chamber_pressure_pa > base.chamber.chamber_pressure_pa);
    }
}
