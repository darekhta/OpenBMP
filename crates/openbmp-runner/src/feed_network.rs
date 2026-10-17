//! Runner-side feed-network adapters.
//!
//! The feed-system crate produces reduced chamber pressure and mass-flow
//! snapshots. The current liquid-engine model consumes only a scalar
//! feed-pressure multiplier, so this adapter maps each configured network's
//! `pc / reference_pc` onto the existing propellant-budget report.

use std::collections::BTreeMap;

use openbmp_core::{EngineId, StepIndex};
use openbmp_fc::topics::PropellantSafingCommand;
use openbmp_feedsystem::{
    FeedCommand, FeedLegPressures, FeedNetwork, MocLine, MocLineBoundary, MocLineConfig,
    MocLineDownstreamBoundary, MocLineElementConfig, MocLineEndpointBoundaryConditions,
    MocLineEndpointKind, MocLineGraphTopology, MocLineGraphTopologyConfig,
    MocLineGraphTopologySnapshot, MocLineJunctionConfig, MocLineJunctionConnection, MocLineState,
    MocLineTopology, MocLineTopologyConfig, MocLineTopologySnapshot, MocLineUpstreamBoundary,
    NormalizedPumpMap, PumpCavitationState, TankValveChamberConfig, TankValveChamberNetwork,
    ThrottleMixtureController, ThrottleMixtureControllerConfig, ThrottleMixtureControllerState,
    ThrottleMixtureMeasurement, ThrottleMixtureSetpoint, TransientChamberConfig,
    TransientChamberState, TransientDualValveFeedNetwork, TransientDualValveFeedNetworkConfig,
    TransientDualValveFeedNetworkState, Turbopump, TurbopumpConfig, TurbopumpDesignPoint,
    TurbopumpOperatingPoint, ValveCommandPair, ValveFeedLegConfig,
};
use openbmp_scenario::{
    PropulsionFeedNetworkConfig, PropulsionFeedNetworkControllerConfig,
    PropulsionFeedNetworkLineConfig, PropulsionFeedNetworkTurbopumpConfig, PropulsionLineConfig,
    PropulsionLineDownstreamBoundaryConfig, PropulsionLineEndpointConfig,
    PropulsionLineJunctionConfig, PropulsionLineUpstreamBoundaryConfig,
    PropulsionMixtureRatioRunawayRuleConfig, PropulsionThermochemConfig, ResolvedFile,
    ScenarioDocument,
};
use openbmp_thermochem::{
    ThermochemDeck, ThermochemQuery, ThermochemTable, UNIVERSAL_GAS_CONSTANT_J_PER_MOL_K,
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

#[derive(Clone, Copy, Debug, PartialEq)]
struct ThermochemFeedConstants {
    gas_temperature_k: f64,
    gas_constant_j_per_kg_k: f64,
    c_star_m_s: f64,
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
        oxidizer_line_reference: Option<FeedLineReferenceRuntime>,
        fuel_line: Option<Box<FeedLineRuntime>>,
        fuel_line_reference: Option<FeedLineReferenceRuntime>,
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
struct FeedLineReferenceRuntime {
    id: String,
    baseline_downstream_pressure_pa: f64,
}

impl FeedLineReferenceRuntime {
    fn new(config: &PropulsionLineConfig) -> Result<Self, RunnerError> {
        let line_config = MocLineConfig {
            length_m: config.length_m,
            wave_speed_m_s: config.wave_speed_m_s,
            density_kg_m3: config.density_kg_m3,
            cross_section_area_m2: config.cross_section_area_m2,
            segment_count: config.segment_count,
        };
        line_config.require_valid()?;
        let baseline_downstream_pressure_pa =
            line_config.pressure_from_head_pa(config.initial_head_m);
        if !baseline_downstream_pressure_pa.is_finite() {
            return Err(openbmp_feedsystem::FeedSystemError::NonFinite {
                reason: "referenced MOC feed-line baseline pressure is non-finite",
            }
            .into());
        }
        Ok(Self {
            id: config.id.clone(),
            baseline_downstream_pressure_pa,
        })
    }

    fn feed_pressure_pa(
        &self,
        base_pressure_pa: f64,
        topology_snapshot: Option<&MocLineTopologySnapshot>,
    ) -> Result<f64, RunnerError> {
        let snapshot = topology_snapshot
            .and_then(|topology_snapshot| topology_snapshot.line(&self.id))
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "standalone MOC line `{}` was referenced before a topology snapshot existed",
                    self.id
                ),
            })?;
        let pressure_pa = base_pressure_pa + snapshot.downstream_pressure_pa
            - self.baseline_downstream_pressure_pa;
        if !pressure_pa.is_finite() {
            return Err(openbmp_feedsystem::FeedSystemError::NonFinite {
                reason: "referenced line-coupled feed pressure is non-finite",
            }
            .into());
        }
        if pressure_pa < 0.0 {
            return Err(openbmp_feedsystem::FeedSystemError::InvalidParameter {
                reason: "referenced line-coupled feed pressure became negative",
            }
            .into());
        }
        Ok(pressure_pa)
    }
}

#[cfg(test)]
fn build_standalone_line_topology(
    configs: &[PropulsionLineConfig],
    dt_s: f64,
) -> Result<Option<StandaloneLineTopologyRuntime>, RunnerError> {
    build_standalone_line_topology_with_junctions(configs, &[], dt_s)
}

fn build_standalone_line_topology_with_junctions(
    configs: &[PropulsionLineConfig],
    junctions: &[PropulsionLineJunctionConfig],
    dt_s: f64,
) -> Result<Option<StandaloneLineTopologyRuntime>, RunnerError> {
    if configs.is_empty() {
        return Ok(None);
    }
    let elements = configs
        .iter()
        .map(standalone_line_element)
        .collect::<Result<Vec<_>, _>>()?;
    let topology = if junctions.is_empty() {
        StandaloneLineTopologyRuntime::Independent(MocLineTopology::new(MocLineTopologyConfig {
            lines: elements,
        })?)
    } else {
        StandaloneLineTopologyRuntime::Graph(MocLineGraphTopology::new(
            MocLineGraphTopologyConfig {
                lines: elements,
                junctions: junctions
                    .iter()
                    .map(standalone_line_junction_config)
                    .collect(),
            },
        )?)
    };
    let courant_dt_s = match &topology {
        StandaloneLineTopologyRuntime::Independent(topology) => topology.courant_dt_s(),
        StandaloneLineTopologyRuntime::Graph(topology) => topology.courant_dt_s(),
    };
    let tolerance = 1.0e-12_f64.max(courant_dt_s.abs() * 1.0e-9);
    if (dt_s - courant_dt_s).abs() > tolerance {
        return Err(openbmp_feedsystem::FeedSystemError::InvalidParameter {
            reason: "standalone MOC line topology Courant time step must match scenario time.dt_s",
        }
        .into());
    }
    Ok(Some(topology))
}

fn standalone_line_junction_config(config: &PropulsionLineJunctionConfig) -> MocLineJunctionConfig {
    MocLineJunctionConfig {
        id: config.id.clone(),
        connections: config
            .connections
            .iter()
            .map(|connection| MocLineJunctionConnection {
                line_id: connection.line_id.clone(),
                endpoint: standalone_line_endpoint(connection.endpoint),
            })
            .collect(),
    }
}

fn standalone_line_endpoint(config: PropulsionLineEndpointConfig) -> MocLineEndpointKind {
    match config {
        PropulsionLineEndpointConfig::Upstream => MocLineEndpointKind::Upstream,
        PropulsionLineEndpointConfig::Downstream => MocLineEndpointKind::Downstream,
    }
}

fn standalone_line_element(
    config: &PropulsionLineConfig,
) -> Result<MocLineElementConfig, RunnerError> {
    MocLineElementConfig::uniform(
        config.id.clone(),
        MocLineConfig {
            length_m: config.length_m,
            wave_speed_m_s: config.wave_speed_m_s,
            density_kg_m3: config.density_kg_m3,
            cross_section_area_m2: config.cross_section_area_m2,
            segment_count: config.segment_count,
        },
        config.initial_head_m,
        config.initial_velocity_m_s,
        MocLineEndpointBoundaryConditions {
            upstream: standalone_upstream_boundary(&config.upstream),
            downstream: standalone_downstream_boundary(&config.downstream),
        },
    )
    .map_err(Into::into)
}

fn standalone_upstream_boundary(
    config: &PropulsionLineUpstreamBoundaryConfig,
) -> MocLineUpstreamBoundary {
    match config {
        PropulsionLineUpstreamBoundaryConfig::Head { head_m } => {
            MocLineUpstreamBoundary::Head { head_m: *head_m }
        }
        PropulsionLineUpstreamBoundaryConfig::Velocity { velocity_m_s } => {
            MocLineUpstreamBoundary::Velocity {
                velocity_m_s: *velocity_m_s,
            }
        }
        PropulsionLineUpstreamBoundaryConfig::GasSource {
            datum_head_m,
            gas_reference_volume_m3,
            gas_precharge_pressure_pa,
            delivered_liquid_volume_delta_m3,
            polytropic_exponent,
        } => MocLineUpstreamBoundary::GasSource {
            datum_head_m: *datum_head_m,
            gas_reference_volume_m3: *gas_reference_volume_m3,
            gas_precharge_pressure_pa: *gas_precharge_pressure_pa,
            delivered_liquid_volume_delta_m3: *delivered_liquid_volume_delta_m3,
            polytropic_exponent: *polytropic_exponent,
        },
    }
}

fn standalone_downstream_boundary(
    config: &PropulsionLineDownstreamBoundaryConfig,
) -> MocLineDownstreamBoundary {
    match config {
        PropulsionLineDownstreamBoundaryConfig::Velocity { velocity_m_s } => {
            MocLineDownstreamBoundary::Velocity {
                velocity_m_s: *velocity_m_s,
            }
        }
        PropulsionLineDownstreamBoundaryConfig::Head { head_m } => {
            MocLineDownstreamBoundary::Head { head_m: *head_m }
        }
        PropulsionLineDownstreamBoundaryConfig::Accumulator {
            reference_head_m,
            stored_volume_delta_m3,
            compliance_m3_per_m,
        } => MocLineDownstreamBoundary::Accumulator {
            reference_head_m: *reference_head_m,
            stored_volume_delta_m3: *stored_volume_delta_m3,
            compliance_m3_per_m: *compliance_m3_per_m,
        },
        PropulsionLineDownstreamBoundaryConfig::GasAccumulator {
            datum_head_m,
            gas_reference_volume_m3,
            gas_precharge_pressure_pa,
            stored_liquid_volume_delta_m3,
            polytropic_exponent,
        } => MocLineDownstreamBoundary::GasAccumulator {
            datum_head_m: *datum_head_m,
            gas_reference_volume_m3: *gas_reference_volume_m3,
            gas_precharge_pressure_pa: *gas_precharge_pressure_pa,
            stored_liquid_volume_delta_m3: *stored_liquid_volume_delta_m3,
            polytropic_exponent: *polytropic_exponent,
        },
    }
}

fn feed_line_reference_runtime(
    id: &str,
    configs: &BTreeMap<&str, &PropulsionLineConfig>,
) -> Result<FeedLineReferenceRuntime, RunnerError> {
    let config = configs
        .get(id)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("standalone MOC line `{id}` was referenced but not declared"),
        })?;
    FeedLineReferenceRuntime::new(config)
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

fn load_thermochem_feed_constants(
    thermochem: Option<&PropulsionThermochemConfig>,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Option<ThermochemFeedConstants>, RunnerError> {
    let Some(thermochem) = thermochem else {
        return Ok(None);
    };
    let resolved = resolved_files
        .get("propulsion.thermochem.file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `propulsion.thermochem.file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|e| RunnerError::UnsupportedScenario {
            what: format!(
                "could not read thermochemistry deck {} as UTF-8: {e}",
                resolved.path.display()
            ),
        })?;
    let deck = ThermochemTable::load_from_str(text)?;
    let state = deck.lookup(ThermochemQuery {
        chamber_pressure_pa: thermochem.chamber_pressure_pa,
        mixture_ratio: thermochem.mixture_ratio,
    })?;
    Ok(Some(ThermochemFeedConstants {
        gas_temperature_k: state.chamber_temperature_k,
        gas_constant_j_per_kg_k: UNIVERSAL_GAS_CONSTANT_J_PER_MOL_K
            / state.molecular_weight_kg_per_mol,
        c_star_m_s: state.c_star_m_s,
    }))
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

    fn apply_propellant_safing_command(&mut self) {
        match self {
            Self::TankValveChamber {
                valve_open_fraction,
                ..
            } => {
                *valve_open_fraction = 0.0;
            }
            Self::TransientDualValveChamber {
                valve_commands,
                controller,
                ..
            } => {
                *valve_commands = ValveCommandPair {
                    oxidizer_open_fraction: 0.0,
                    fuel_open_fraction: 0.0,
                };
                if let Some(controller) = controller {
                    controller.state = ThrottleMixtureControllerState::new(*valve_commands);
                }
            }
        }
    }

    fn feed_scale(
        &mut self,
        dt_s: f64,
        valve_bias: ValveCommandBias,
        standalone_line_snapshot: Option<&MocLineTopologySnapshot>,
    ) -> Result<f64, RunnerError> {
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
                oxidizer_line_reference,
                fuel_line,
                fuel_line_reference,
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
                } else if let Some(line_reference) = oxidizer_line_reference {
                    line_reference
                        .feed_pressure_pa(*base_oxidizer_pressure_pa, standalone_line_snapshot)?
                } else {
                    *base_oxidizer_pressure_pa
                };
                let fuel_pressure_pa = if let Some(line) = fuel_line {
                    line.feed_pressure_pa(*base_fuel_pressure_pa)?
                } else if let Some(line_reference) = fuel_line_reference {
                    line_reference
                        .feed_pressure_pa(*base_fuel_pressure_pa, standalone_line_snapshot)?
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

#[derive(Debug)]
enum StandaloneLineTopologyRuntime {
    Independent(MocLineTopology),
    Graph(MocLineGraphTopology),
}

impl StandaloneLineTopologyRuntime {
    fn line_ids(&self) -> Box<dyn Iterator<Item = &str> + '_> {
        match self {
            Self::Independent(topology) => Box::new(topology.line_ids()),
            Self::Graph(topology) => Box::new(topology.line_ids()),
        }
    }

    fn step(&mut self) -> Result<MocLineTopologySnapshot, RunnerError> {
        match self {
            Self::Independent(topology) => topology.step().map_err(Into::into),
            Self::Graph(topology) => {
                let snapshot = topology.step()?;
                Ok(graph_snapshot_into_line_snapshot(snapshot))
            }
        }
    }
}

fn graph_snapshot_into_line_snapshot(
    snapshot: MocLineGraphTopologySnapshot,
) -> MocLineTopologySnapshot {
    MocLineTopologySnapshot {
        lines: snapshot.lines,
        dt_s: snapshot.dt_s,
        validation: snapshot.validation,
    }
}

/// Runner-side collection of opt-in feed-network overrides.
#[derive(Debug, Default)]
pub struct FeedNetworkRack {
    entries: Vec<FeedNetworkEntry>,
    standalone_lines: Option<StandaloneLineTopologyRuntime>,
    latest_standalone_line_snapshot: Option<MocLineTopologySnapshot>,
    mixture_ratio_runaway_faults: Vec<MixtureRatioRunawayFaultRuntime>,
    propellant_safing_active: bool,
}

impl FeedNetworkRack {
    /// Build from `[propulsion.feed_network]` declarations.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when the reduced feed-network config fails
    /// validation in `openbmp-feedsystem`.
    pub fn build(
        document: &ScenarioDocument,
        resolved_files: &BTreeMap<String, ResolvedFile>,
    ) -> Result<Self, RunnerError> {
        let Some(propulsion) = &document.propulsion else {
            return Ok(Self::default());
        };
        if propulsion.feed_networks.is_empty()
            && propulsion.lines.is_empty()
            && propulsion.line_junctions.is_empty()
        {
            return Ok(Self::default());
        }
        let thermochem_constants = if propulsion.feed_networks.is_empty() {
            None
        } else {
            load_thermochem_feed_constants(propulsion.thermochem.as_ref(), resolved_files)?
        };
        let standalone_line_configs: BTreeMap<&str, &PropulsionLineConfig> = propulsion
            .lines
            .iter()
            .map(|config| (config.id.as_str(), config))
            .collect();
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
                    let c_star_m_s =
                        thermochem_constants.map_or(*c_star_m_s, |state| state.c_star_m_s);
                    let network = TankValveChamberNetwork::new(TankValveChamberConfig {
                        tank_pressure_pa: *tank_pressure_pa,
                        propellant_density_kg_m3: *propellant_density_kg_m3,
                        valve_area_m2: *valve_area_m2,
                        valve_discharge_coefficient: *valve_discharge_coefficient,
                        throat_area_m2: *throat_area_m2,
                        c_star_m_s,
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
                    oxidizer_line_id,
                    fuel_line,
                    fuel_line_id,
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
                    let gas_temperature_k = thermochem_constants
                        .map_or(*gas_temperature_k, |state| state.gas_temperature_k);
                    let gas_constant_j_per_kg_k = thermochem_constants
                        .map_or(*gas_constant_j_per_kg_k, |state| {
                            state.gas_constant_j_per_kg_k
                        });
                    let c_star_m_s =
                        thermochem_constants.map_or(*c_star_m_s, |state| state.c_star_m_s);
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
                    let oxidizer_line_reference = oxidizer_line_id
                        .as_deref()
                        .map(|id| feed_line_reference_runtime(id, &standalone_line_configs))
                        .transpose()?;
                    let fuel_line_reference = fuel_line_id
                        .as_deref()
                        .map(|id| feed_line_reference_runtime(id, &standalone_line_configs))
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
                                gas_temperature_k,
                                gas_constant_j_per_kg_k,
                                throat_area_m2: *throat_area_m2,
                                c_star_m_s,
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
                        oxidizer_line_reference,
                        fuel_line,
                        fuel_line_reference,
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
        let standalone_lines = build_standalone_line_topology_with_junctions(
            &propulsion.lines,
            &propulsion.line_junctions,
            document.time.dt_s,
        )?;
        Ok(Self {
            entries,
            standalone_lines,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults,
            propellant_safing_active: false,
        })
    }

    /// `true` when no feed-network overrides were configured.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty() && self.standalone_lines.is_none()
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

    /// Drive configured feed-network valves to their safed state.
    pub fn apply_propellant_safing_command(&mut self, command: Option<PropellantSafingCommand>) {
        let Some(command) = command else {
            return;
        };
        if !command.safed || !(command.close_engine_valves || command.isolate_feeds) {
            return;
        }

        self.propellant_safing_active = true;
        for entry in &mut self.entries {
            entry.apply_propellant_safing_command();
        }
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
        if let Some(standalone_lines) = &mut self.standalone_lines {
            self.latest_standalone_line_snapshot = Some(standalone_lines.step()?);
        }
        for entry in &mut self.entries {
            let engine_id = entry.engine_id();
            if self.propellant_safing_active {
                report.shutdown_engines.insert(engine_id);
                report.feed_pressure_scales.insert(engine_id, 0.0);
                continue;
            }
            if report.shutdown_engines.contains(&engine_id) {
                report.feed_pressure_scales.insert(engine_id, 0.0);
                continue;
            }
            let valve_bias = update_mixture_ratio_runaway_bias(faults, engine_id, step, dt_s);
            report.feed_pressure_scales.insert(
                engine_id,
                entry.feed_scale(
                    dt_s,
                    valve_bias,
                    self.latest_standalone_line_snapshot.as_ref(),
                )?,
            );
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

    /// Standalone line ids in scenario declaration order.
    #[must_use]
    pub fn standalone_line_ids(&self) -> Vec<String> {
        self.standalone_lines
            .as_ref()
            .map(|topology| topology.line_ids().map(str::to_owned).collect())
            .unwrap_or_default()
    }

    /// Latest standalone line topology snapshot, if the rack has stepped it.
    #[must_use]
    pub fn latest_standalone_line_snapshot(&self) -> Option<&MocLineTopologySnapshot> {
        self.latest_standalone_line_snapshot.as_ref()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::{collections::BTreeSet, path::PathBuf};

    use approx::assert_abs_diff_eq;
    use openbmp_core::{SimTime, TankId};

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
            standalone_lines: None,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults: Vec::new(),
            propellant_safing_active: false,
        }
    }

    fn safing_command(safed: bool) -> PropellantSafingCommand {
        PropellantSafingCommand {
            time: SimTime::ZERO,
            safed,
            isolate_feeds: true,
            close_engine_valves: true,
            pressurization_enabled: false,
            drain_enabled: false,
            dump_enabled: true,
        }
    }

    fn report(scale: f64) -> PropellantBudgetReport {
        let mut feed_pressure_scales = BTreeMap::new();
        feed_pressure_scales.insert(engine_id(), scale);
        PropellantBudgetReport {
            tank_drain_rates_kg_per_s: BTreeMap::<TankId, f64>::new(),
            pressurization_enabled: true,
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

    fn standalone_line_config(id: &str) -> PropulsionLineConfig {
        PropulsionLineConfig {
            id: id.to_owned(),
            length_m: 40.0,
            wave_speed_m_s: 1_000.0,
            density_kg_m3: 1_000.0,
            cross_section_area_m2: 0.01,
            segment_count: 4,
            initial_head_m: 100.0,
            initial_velocity_m_s: 2.0,
            upstream: PropulsionLineUpstreamBoundaryConfig::Head { head_m: 100.0 },
            downstream: PropulsionLineDownstreamBoundaryConfig::Velocity { velocity_m_s: 0.0 },
        }
    }

    fn standalone_gas_accumulator_line_config(id: &str) -> PropulsionLineConfig {
        let mut config = standalone_line_config(id);
        config.downstream = PropulsionLineDownstreamBoundaryConfig::GasAccumulator {
            datum_head_m: 0.0,
            gas_reference_volume_m3: 0.02,
            gas_precharge_pressure_pa: 1_000_000.0,
            stored_liquid_volume_delta_m3: 0.0,
            polytropic_exponent: 1.2,
        };
        config
    }

    fn standalone_gas_source_line_config(id: &str) -> PropulsionLineConfig {
        let mut config = standalone_line_config(id);
        config.upstream = PropulsionLineUpstreamBoundaryConfig::GasSource {
            datum_head_m: 0.0,
            gas_reference_volume_m3: 0.02,
            gas_precharge_pressure_pa: 1_000_000.0,
            delivered_liquid_volume_delta_m3: 0.0,
            polytropic_exponent: 1.2,
        };
        config
    }

    fn standalone_graph_branch_line_config(id: &str) -> PropulsionLineConfig {
        let mut config = standalone_line_config(id);
        config.initial_velocity_m_s = 0.0;
        config.downstream = PropulsionLineDownstreamBoundaryConfig::Velocity { velocity_m_s: 0.0 };
        config
    }

    fn standalone_graph_junction_config(id: &str) -> PropulsionLineJunctionConfig {
        PropulsionLineJunctionConfig {
            id: id.to_owned(),
            connections: vec![
                openbmp_scenario::PropulsionLineJunctionConnectionConfig {
                    line_id: "inlet".to_owned(),
                    endpoint: PropulsionLineEndpointConfig::Downstream,
                },
                openbmp_scenario::PropulsionLineJunctionConnectionConfig {
                    line_id: "branch-a".to_owned(),
                    endpoint: PropulsionLineEndpointConfig::Upstream,
                },
                openbmp_scenario::PropulsionLineJunctionConnectionConfig {
                    line_id: "branch-b".to_owned(),
                    endpoint: PropulsionLineEndpointConfig::Upstream,
                },
            ],
        }
    }

    #[test]
    fn feed_network_thermochem_deck_overrides_chamber_constants() {
        let bytes = include_bytes!("../tests/fixtures/thermochem/synthetic-grain.toml").to_vec();
        let mut resolved_files = BTreeMap::new();
        resolved_files.insert(
            "propulsion.thermochem.file".to_owned(),
            ResolvedFile {
                path: PathBuf::from("tests/fixtures/thermochem/synthetic-grain.toml"),
                sha256_hex: String::new(),
                bytes,
            },
        );
        let constants = load_thermochem_feed_constants(
            Some(&PropulsionThermochemConfig {
                file: PathBuf::from("tests/fixtures/thermochem/synthetic-grain.toml"),
                file_sha256: None,
                chamber_pressure_pa: 2_000_000.0,
                mixture_ratio: 2.5,
            }),
            &resolved_files,
        )
        .unwrap()
        .expect("thermochem constants");

        assert!((constants.gas_temperature_k - 3_150.0).abs() < 1.0e-12);
        assert!((constants.c_star_m_s - 1_658.024_003_983_371_8).abs() < 1.0e-12);
        assert!((constants.gas_constant_j_per_kg_k - 369.531_671_917_921_76).abs() < 1.0e-12);
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
    fn feed_network_safing_closes_tank_valve_chamber() {
        let mut rack = rack(4.0e6);
        rack.apply_propellant_safing_command(Some(safing_command(true)));
        let mut report = report(1.0);

        rack.apply_to_report(&mut report, 0.1, StepIndex::ZERO)
            .unwrap();

        assert!(report.shutdown_engines.contains(&engine_id()));
        assert_eq!(
            report.feed_pressure_scales[&engine_id()].to_bits(),
            0.0_f64.to_bits()
        );
        match &rack.entries[0] {
            FeedNetworkEntry::TankValveChamber {
                valve_open_fraction,
                ..
            } => assert_eq!(valve_open_fraction.to_bits(), 0.0_f64.to_bits()),
            FeedNetworkEntry::TransientDualValveChamber { .. } => {
                panic!("unexpected transient network")
            }
        }
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
                oxidizer_line_reference: None,
                fuel_line: None,
                fuel_line_reference: None,
            }],
            standalone_lines: None,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults: Vec::new(),
            propellant_safing_active: false,
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
                oxidizer_line_reference: None,
                fuel_line: None,
                fuel_line_reference: None,
            }],
            standalone_lines: None,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults: Vec::new(),
            propellant_safing_active: false,
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
                oxidizer_line_reference: None,
                fuel_line: None,
                fuel_line_reference: None,
            }],
            standalone_lines: None,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults: Vec::new(),
            propellant_safing_active: false,
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
                oxidizer_line_reference: None,
                fuel_line: None,
                fuel_line_reference: None,
            }],
            standalone_lines: None,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults: vec![MixtureRatioRunawayFaultRuntime {
                engine_id: engine_id(),
                start_step: 2,
                oxidizer_open_fraction_rate_per_s: 0.4,
                fuel_open_fraction_rate_per_s: -0.2,
            }],
            propellant_safing_active: false,
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
    fn standalone_line_topology_runtime_steps_with_rack_tick() {
        let standalone_lines =
            build_standalone_line_topology(&[standalone_line_config("oxidizer-main")], 0.01)
                .unwrap();
        let mut rack = FeedNetworkRack {
            entries: Vec::new(),
            standalone_lines,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults: Vec::new(),
            propellant_safing_active: false,
        };

        assert!(!rack.is_empty());
        assert_eq!(rack.standalone_line_ids(), vec!["oxidizer-main"]);
        rack.apply_to_report(&mut report(1.0), 0.01, StepIndex::ZERO)
            .unwrap();

        let snapshot = rack.latest_standalone_line_snapshot().unwrap();
        let line_snapshot = snapshot.line("oxidizer-main").unwrap();
        let expected_pressure_pa = 1_000.0 * 9.806_65 * 100.0 + 1_000.0 * 1_000.0 * 2.0;
        assert!((line_snapshot.downstream_pressure_pa - expected_pressure_pa).abs() < 1.0e-6);
    }

    #[test]
    fn standalone_line_topology_runtime_rejects_non_courant_dt() {
        assert!(build_standalone_line_topology(&[standalone_line_config("line")], 0.02).is_err());
    }

    #[test]
    fn standalone_line_topology_runtime_maps_gas_accumulator_boundary() {
        let standalone_lines = build_standalone_line_topology(
            &[standalone_gas_accumulator_line_config("gas-main")],
            0.01,
        )
        .unwrap();
        let mut rack = FeedNetworkRack {
            entries: Vec::new(),
            standalone_lines,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults: Vec::new(),
            propellant_safing_active: false,
        };

        rack.apply_to_report(&mut report(1.0), 0.01, StepIndex::ZERO)
            .unwrap();

        let line_snapshot = rack
            .latest_standalone_line_snapshot()
            .and_then(|snapshot| snapshot.line("gas-main"))
            .unwrap();
        let precharge_head_m = 1_000_000.0 / (1_000.0 * 9.806_65);
        assert!(line_snapshot.downstream_head_m > precharge_head_m);
    }

    #[test]
    fn standalone_line_topology_runtime_maps_gas_source_boundary() {
        let standalone_lines = build_standalone_line_topology(
            &[standalone_gas_source_line_config("source-main")],
            0.01,
        )
        .unwrap();
        let mut rack = FeedNetworkRack {
            entries: Vec::new(),
            standalone_lines,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults: Vec::new(),
            propellant_safing_active: false,
        };

        rack.apply_to_report(&mut report(1.0), 0.01, StepIndex::ZERO)
            .unwrap();

        let line_snapshot = rack
            .latest_standalone_line_snapshot()
            .and_then(|snapshot| snapshot.line("source-main"))
            .unwrap();
        let precharge_head_m = 1_000_000.0 / (1_000.0 * 9.806_65);
        assert!(line_snapshot.state.head_m[0] < precharge_head_m);
    }

    #[test]
    fn standalone_line_topology_runtime_maps_junction_graph() {
        let standalone_lines = build_standalone_line_topology_with_junctions(
            &[
                standalone_line_config("inlet"),
                standalone_graph_branch_line_config("branch-a"),
                standalone_graph_branch_line_config("branch-b"),
            ],
            &[standalone_graph_junction_config("tee")],
            0.01,
        )
        .unwrap();
        let mut rack = FeedNetworkRack {
            entries: Vec::new(),
            standalone_lines,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults: Vec::new(),
            propellant_safing_active: false,
        };

        rack.apply_to_report(&mut report(1.0), 0.01, StepIndex::ZERO)
            .unwrap();

        let snapshot = rack.latest_standalone_line_snapshot().unwrap();
        let wave_over_gravity = 1_000.0 / 9.806_65;
        let expected_head_m = 100.0 + (2.0 * wave_over_gravity) / 3.0;
        assert_eq!(
            rack.standalone_line_ids(),
            vec!["inlet", "branch-a", "branch-b"]
        );
        assert_abs_diff_eq!(
            snapshot.line("inlet").unwrap().downstream_head_m,
            expected_head_m,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.line("branch-a").unwrap().state.velocity_m_s[0],
            2.0 / 3.0,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.line("branch-b").unwrap().state.velocity_m_s[0],
            2.0 / 3.0,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn referenced_standalone_line_couples_transient_feed_pressure() {
        let standalone_line = standalone_line_config("oxidizer-main");
        let standalone_lines =
            build_standalone_line_topology(std::slice::from_ref(&standalone_line), 0.01).unwrap();
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
                    oxidizer_open_fraction: 1.0,
                    fuel_open_fraction: 1.0,
                },
                pump_cavitation: FeedPumpCavitation::default(),
                controller: None,
                oxidizer_line: None,
                oxidizer_line_reference: Some(
                    FeedLineReferenceRuntime::new(&standalone_line).unwrap(),
                ),
                fuel_line: None,
                fuel_line_reference: None,
            }],
            standalone_lines,
            latest_standalone_line_snapshot: None,
            mixture_ratio_runaway_faults: Vec::new(),
            propellant_safing_active: false,
        };
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
            oxidizer_line_reference: None,
            fuel_line: None,
            fuel_line_reference: None,
        };

        let base_scale = base_entry
            .feed_scale(0.01, ValveCommandBias::default(), None)
            .unwrap();
        let mut report = report(1.0);
        rack.apply_to_report(&mut report, 0.01, StepIndex::ZERO)
            .unwrap();
        let referenced_scale = report.feed_pressure_scales[&engine_id()];

        assert!(referenced_scale > base_scale);
        assert!(
            rack.latest_standalone_line_snapshot()
                .and_then(|snapshot| snapshot.line("oxidizer-main"))
                .is_some()
        );
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
            oxidizer_line_reference: None,
            fuel_line: None,
            fuel_line_reference: None,
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
            oxidizer_line_reference: None,
            fuel_line: None,
            fuel_line_reference: None,
        };

        let base_scale = base_entry
            .feed_scale(0.01, ValveCommandBias::default(), None)
            .unwrap();
        let line_scale = line_entry
            .feed_scale(0.01, ValveCommandBias::default(), None)
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
