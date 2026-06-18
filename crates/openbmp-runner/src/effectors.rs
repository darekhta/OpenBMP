//! Runner-side effector rack.
//!
//! The rack owns the `Vec<Box<dyn ControlEffector>>` resolved from
//! the scenario's `[[vehicle.assembly.effectors]]` block, plus the
//! per-effector deterministic command schedules and one-shot
//! override map. It is stepped once per kernel base tick **before**
//! the kernel's `step()` so the effector telemetry observable in
//! the Parquet matches the kernel's view of the world.
//!
//! The rack leaves the kernel's force / moment evaluation untouched
//! when it is empty — legacy scenarios produce byte-identical
//! Parquet because the runner short-circuits every rack-related
//! operation on `is_empty()`.
//!
//! Aero-deck schema-2 scenarios consume effector deflections; for
//! decks without effector axes the rack snapshot is read only by the
//! telemetry layer.

use std::collections::BTreeMap;

use openbmp_core::{Duration, EffectorId, SimTime, TankId};
use openbmp_scenario::{
    DirectTorqueRcsMode, EffectorCommandScheduleConfig, EffectorConfig, EffectorFaultConfig,
    EffectorKindConfig, ScenarioDocument,
};
use openbmp_sim::{FiredEvent, ScenarioScriptAction};
use openbmp_vehicle::{
    ControlEffector, EffectorFault, EffectorLimits, EffectorState, LinearActuator,
    PropellantTankState, PwpfModulator, PwpfParams, RcsBlowdownParams, RcsCoupledAllocator,
    RcsCoupledThrusterPulse, RcsPulseEffector, RcsPulseEffectorParams, RcsThrusterBankEffector,
    RcsThrusterBankEffectorParams, RcsThrusterBankPulse, RcsThrusterConfig, SecondOrderServo,
    SecondOrderServoParams,
};

use crate::error::RunnerError;

const STANDARD_GRAVITY_M_S2: f64 = 9.80665;

/// Runner-side effector container. Built once per `openbmp run`
/// invocation; consumed by the per-step kernel loop.
pub struct EffectorRack {
    /// Effectors in scenario-declared order.
    effectors: Vec<Box<dyn ControlEffector>>,
    /// Telemetry-friendly id-string copies of each effector's
    /// scenario-declared id. Same iteration order as `effectors`.
    string_ids: Vec<String>,
    /// Lookup from `EffectorId` to the `effectors` index.
    /// `BTreeMap`, not `HashMap`, to defeat macOS `SipHash`
    /// randomisation when the runner iterates the override map.
    id_index: BTreeMap<EffectorId, usize>,
    /// Per-effector command schedules. Indexed by effector index;
    /// `None` when the scenario declared no schedule for that
    /// effector (defaults to commanding the initial position
    /// indefinitely).
    schedules: Vec<Option<EffectorSchedule>>,
    /// Baseline hold command for effectors without a schedule. This is
    /// the configured initial position, not the most recent commanded
    /// value, so one-shot overrides do not become sticky.
    hold_commands: Vec<f64>,
    /// One-shot command overrides drained from scenario-script
    /// effector override events. Applied on the next rack tick and
    /// cleared after each `step()`.
    overrides: BTreeMap<EffectorId, f64>,
    /// Shared coupled RCS allocator groups keyed from direct-torque
    /// effectors with `mode = "coupled_bank"`.
    coupled_rcs_groups: Vec<CoupledRcsGroup>,
    /// Per-effector bank-mode RCS tank feeds, indexed by effector index.
    bank_rcs_feeds: BTreeMap<usize, Vec<Option<RcsTankFeedBinding>>>,
    /// Per-effector allocated state overrides produced by coupled RCS groups.
    coupled_allocated_states: Vec<Option<EffectorState>>,
    /// RCS propellant drain rates emitted by the most recent step.
    rcs_feed_drain_rates_kg_per_s: BTreeMap<TankId, f64>,
    /// Construction-time `dt`; stepped at exactly this rate.
    dt: Duration,
}

impl std::fmt::Debug for EffectorRack {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EffectorRack")
            .field("effector_count", &self.effectors.len())
            .field(
                "schedule_count",
                &self.schedules.iter().filter(|s| s.is_some()).count(),
            )
            .field("override_count", &self.overrides.len())
            .field("coupled_rcs_group_count", &self.coupled_rcs_groups.len())
            .field("bank_rcs_feed_count", &self.bank_rcs_feeds.len())
            .finish_non_exhaustive()
    }
}

/// Per-effector deterministic command schedule.
#[derive(Clone, Debug)]
enum EffectorSchedule {
    Constant(f64),
    StepAt {
        time_s: f64,
        before: f64,
        after: f64,
    },
    LinearRamp {
        start_time_s: f64,
        end_time_s: f64,
        start: f64,
        end: f64,
    },
    PiecewiseLinear {
        points: Vec<(f64, f64)>,
    },
}

impl EffectorSchedule {
    fn from_config(config: &EffectorCommandScheduleConfig) -> Self {
        match config {
            EffectorCommandScheduleConfig::Constant { value } => Self::Constant(*value),
            EffectorCommandScheduleConfig::StepAt {
                time_s,
                before,
                after,
            } => Self::StepAt {
                time_s: *time_s,
                before: *before,
                after: *after,
            },
            EffectorCommandScheduleConfig::LinearRamp {
                start_time_s,
                end_time_s,
                start,
                end,
            } => Self::LinearRamp {
                start_time_s: *start_time_s,
                end_time_s: *end_time_s,
                start: *start,
                end: *end,
            },
            EffectorCommandScheduleConfig::PiecewiseLinear { points } => Self::PiecewiseLinear {
                points: points
                    .iter()
                    .map(|point| (point.time_s, point.value))
                    .collect(),
            },
        }
    }

    fn command_at(&self, time: SimTime) -> f64 {
        let t = time.as_seconds();
        match *self {
            Self::Constant(value) => value,
            Self::StepAt {
                time_s,
                before,
                after,
            } => {
                if t < time_s {
                    before
                } else {
                    after
                }
            }
            Self::LinearRamp {
                start_time_s,
                end_time_s,
                start,
                end,
            } => {
                if t <= start_time_s {
                    start
                } else if t >= end_time_s {
                    end
                } else {
                    let alpha = (t - start_time_s) / (end_time_s - start_time_s);
                    start + alpha * (end - start)
                }
            }
            Self::PiecewiseLinear { ref points } => {
                let Some(first) = points.first() else {
                    return 0.0;
                };
                if t <= first.0 {
                    return first.1;
                }
                for window in points.windows(2) {
                    let (t0, value0) = window[0];
                    let (t1, value1) = window[1];
                    if t <= t1 {
                        let alpha = (t - t0) / (t1 - t0);
                        return value0 + alpha * (value1 - value0);
                    }
                }
                points.last().map_or(first.1, |point| point.1)
            }
        }
    }
}

#[derive(Debug)]
struct CoupledRcsGroup {
    group_id: String,
    allocator: RcsCoupledAllocator,
    members: Vec<CoupledRcsMember>,
    feeds: Vec<Option<RcsTankFeedBinding>>,
}

#[derive(Debug)]
struct CoupledRcsMember {
    effector_index: usize,
    axis_index: usize,
    effectiveness_n_m_per_unit: f64,
    pwpf: Option<PwpfModulator>,
}

struct CoupledRcsStep {
    updates: Vec<(usize, EffectorState)>,
    drains: BTreeMap<TankId, f64>,
}

#[derive(Clone, Debug, PartialEq)]
struct RcsTankFeedBinding {
    tank_id: TankId,
    specific_impulse_s: f64,
    pressure_exponent: f64,
}

impl CoupledRcsGroup {
    fn allocate(
        &mut self,
        effectors: &[Box<dyn ControlEffector>],
        dt: Duration,
        tank_states: Option<&BTreeMap<TankId, PropellantTankState>>,
    ) -> Result<CoupledRcsStep, RunnerError> {
        let dt_s = dt.as_seconds();
        let feed_scales = rcs_feed_pressure_scales(&self.feeds, tank_states, &self.group_id)?;
        self.allocator
            .set_thruster_feed_pressure_scales(&feed_scales)
            .map_err(|err| RunnerError::Effector {
                field: format!(
                    "vehicle.assembly.effectors[*].kind.rcs.group={}",
                    self.group_id
                ),
                reason: err.to_string(),
            })?;
        let mut requested_torque_impulse_body_n_m_s = [0.0; 3];
        let mut pwpf_gated = BTreeMap::new();
        for member in &mut self.members {
            let state = effectors[member.effector_index].current_state();
            let mut command = state.actual;
            let mut gated = false;
            if let Some(pwpf) = &mut member.pwpf {
                let polarity = pwpf
                    .step(command, dt)
                    .map_err(|err| RunnerError::Effector {
                        field: format!(
                            "vehicle.assembly.effectors[{}].kind.rcs.pwpf",
                            member.effector_index
                        ),
                        reason: err.to_string(),
                    })?
                    .polarity;
                let gated_command = if polarity.is_firing() {
                    polarity.sign() * command.abs()
                } else {
                    0.0
                };
                gated = (gated_command - command).abs() > 1.0e-12 * command.abs().max(1.0);
                command = gated_command;
            }
            pwpf_gated.insert(member.effector_index, gated);
            requested_torque_impulse_body_n_m_s[member.axis_index] +=
                command * member.effectiveness_n_m_per_unit * dt_s;
        }

        let allocation = self
            .allocator
            .allocate(requested_torque_impulse_body_n_m_s)
            .map_err(|err| RunnerError::Effector {
                field: format!(
                    "vehicle.assembly.effectors[*].kind.rcs.group={}",
                    self.group_id
                ),
                reason: err.to_string(),
            })?;

        let mut updates = Vec::with_capacity(self.members.len());
        for member in &self.members {
            let mut state = effectors[member.effector_index].current_state();
            let actual_torque_impulse =
                allocation.actual_torque_impulse_body_n_m_s[member.axis_index];
            state.actual = actual_torque_impulse / dt_s / member.effectiveness_n_m_per_unit;
            state.saturated |= allocation.saturated;
            state.rate_limited |= allocation.quantized
                || pwpf_gated
                    .get(&member.effector_index)
                    .copied()
                    .unwrap_or(false);
            updates.push((member.effector_index, state));
        }
        Ok(CoupledRcsStep {
            updates,
            drains: rcs_feed_drain_rates(&self.feeds, &allocation.pulses, dt_s),
        })
    }
}

impl EffectorRack {
    /// Build the rack from the parsed scenario document. Returns an
    /// empty rack when no `[[vehicle.assembly.effectors]]` blocks
    /// are declared.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Assembly`] when effector construction rejects
    /// the scenario-declared limits, initial position, latency, or per-kind
    /// dynamics parameters.
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let dt = Duration::from_seconds(document.time.dt_s);
        let mut effectors: Vec<Box<dyn ControlEffector>> = Vec::new();
        let mut string_ids: Vec<String> = Vec::new();
        let mut id_index: BTreeMap<EffectorId, usize> = BTreeMap::new();
        let mut schedules: Vec<Option<EffectorSchedule>> = Vec::new();
        let mut hold_commands: Vec<f64> = Vec::new();

        for (index, config) in document.vehicle.assembly.effectors.iter().enumerate() {
            let actuator = build_effector(index, config, dt)?;
            let id = actuator.id();
            effectors.push(actuator);
            string_ids.push(config.id.clone());
            id_index.insert(id, index);
            schedules.push(
                config
                    .command_schedule
                    .as_ref()
                    .map(EffectorSchedule::from_config),
            );
            hold_commands.push(config.initial_position.unwrap_or(0.0));
        }
        let coupled_rcs_groups = build_coupled_rcs_groups(document, dt)?;
        let bank_rcs_feeds = build_bank_rcs_feeds(document)?;
        let coupled_allocated_states = vec![None; effectors.len()];

        Ok(Self {
            effectors,
            string_ids,
            id_index,
            schedules,
            hold_commands,
            overrides: BTreeMap::new(),
            coupled_rcs_groups,
            bank_rcs_feeds,
            coupled_allocated_states,
            rcs_feed_drain_rates_kg_per_s: BTreeMap::new(),
            dt,
        })
    }

    /// Number of effectors in the rack.
    #[must_use]
    pub fn len(&self) -> usize {
        self.effectors.len()
    }

    /// `true` when the rack carries no effectors. The runner gates
    /// every per-step operation on this so legacy scenarios never
    /// touch the rack code path.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.effectors.is_empty()
    }

    /// Scenario-declared id strings, in declared order.
    #[must_use]
    pub fn scenario_ids(&self) -> &[String] {
        &self.string_ids
    }

    /// Apply any scenario-script effector override actions drained
    /// from the kernel's per-step fired-event queue. Override values
    /// take precedence over the schedule for the next rack tick only.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Effector`] when an override references an
    /// effector id that is not present in this rack.
    pub fn apply_overrides(
        &mut self,
        fired: &[FiredEvent<ScenarioScriptAction>],
    ) -> Result<(), RunnerError> {
        for event in fired {
            if let ScenarioScriptAction::EffectorOverride { id, command } = event.action {
                if !self.id_index.contains_key(&id) {
                    return Err(RunnerError::Effector {
                        field: "mission.events[*].action.id".to_owned(),
                        reason: format!(
                            "effector_override references unknown effector id value {}",
                            id.value()
                        ),
                    });
                }
                self.overrides.insert(id, command);
            }
        }
        Ok(())
    }

    /// Apply one-tick FC commands keyed by `EffectorId::value()`.
    ///
    /// These commands share the same one-shot override path as
    /// mission events, so the existing per-effector schedule remains
    /// the fallback when the FC publishes no command for a lane.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Effector`] when the FC references an
    /// effector id that is not present in this rack.
    pub fn apply_fc_commands(
        &mut self,
        commands: &openbmp_fc::topics::EffectorCommandSet,
    ) -> Result<(), RunnerError> {
        for command in commands.commands.iter().take(usize::from(commands.count)) {
            let id = EffectorId::new(command.effector_id);
            if !self.id_index.contains_key(&id) {
                return Err(RunnerError::Effector {
                    field: "fc.actuator.effector_cmds".to_owned(),
                    reason: format!(
                        "FC command references unknown effector id value {}",
                        command.effector_id
                    ),
                });
            }
            self.overrides.insert(id, command.command);
        }
        Ok(())
    }

    /// Step every effector by one kernel base tick.
    ///
    /// Command resolution: override (if present) wins; else the
    /// schedule's command at `time` (if a schedule is declared);
    /// else the configured initial-position hold command.
    /// Overrides are cleared after the step.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Effector`] when `LinearActuator::step`
    /// fails (dt mismatch, non-finite command, etc.).
    pub fn step(&mut self, time: SimTime) -> Result<(), RunnerError> {
        self.step_inner(time, None)
    }

    /// Step every effector with live tank states available for RCS feed
    /// coupling.
    pub fn step_with_tanks(
        &mut self,
        time: SimTime,
        tank_states: &BTreeMap<TankId, PropellantTankState>,
    ) -> Result<(), RunnerError> {
        self.step_inner(time, Some(tank_states))
    }

    fn step_inner(
        &mut self,
        time: SimTime,
        tank_states: Option<&BTreeMap<TankId, PropellantTankState>>,
    ) -> Result<(), RunnerError> {
        self.coupled_allocated_states.fill(None);
        self.rcs_feed_drain_rates_kg_per_s.clear();
        for (index, effector) in self.effectors.iter_mut().enumerate() {
            let id = effector.id();
            let cmd = if let Some(&override_cmd) = self.overrides.get(&id) {
                override_cmd
            } else if let Some(schedule) = &self.schedules[index] {
                schedule.command_at(time)
            } else {
                self.hold_commands[index]
            };
            if let Some(feeds) = self.bank_rcs_feeds.get(&index) {
                let scales = rcs_feed_pressure_scales(
                    feeds,
                    tank_states,
                    &format!("vehicle.assembly.effectors[{index}]"),
                )?;
                effector
                    .set_rcs_thruster_feed_pressure_scales(&scales)
                    .map_err(|err| RunnerError::Effector {
                        field: format!("vehicle.assembly.effectors[{index}].kind.rcs.thrusters"),
                        reason: err.to_string(),
                    })?;
            }
            effector
                .step(cmd, self.dt)
                .map_err(|err| RunnerError::Effector {
                    field: format!("vehicle.assembly.effectors[{index}]"),
                    reason: err.to_string(),
                })?;
            if let Some(feeds) = self.bank_rcs_feeds.get(&index) {
                merge_drain_rates(
                    &mut self.rcs_feed_drain_rates_kg_per_s,
                    rcs_feed_drain_rates(
                        feeds,
                        effector.rcs_last_thruster_pulses(),
                        self.dt.as_seconds(),
                    ),
                );
            }
        }
        for group in &mut self.coupled_rcs_groups {
            let step = group.allocate(&self.effectors, self.dt, tank_states)?;
            for (index, state) in step.updates {
                self.coupled_allocated_states[index] = Some(state);
            }
            merge_drain_rates(&mut self.rcs_feed_drain_rates_kg_per_s, step.drains);
        }
        self.overrides.clear();
        Ok(())
    }

    /// RCS feed drain rates emitted by the most recent step.
    #[must_use]
    pub fn rcs_feed_drain_rates(&self) -> BTreeMap<TankId, f64> {
        self.rcs_feed_drain_rates_kg_per_s.clone()
    }

    /// Snapshot every effector's most recent `EffectorState` for
    /// telemetry, in declared order.
    #[must_use]
    pub fn snapshot(&self) -> Vec<EffectorState> {
        self.effectors
            .iter()
            .enumerate()
            .map(|(index, effector)| {
                self.coupled_allocated_states[index].unwrap_or_else(|| effector.current_state())
            })
            .collect()
    }
}

/// Effector resolver: scenario `EffectorConfig` → concrete
/// `ControlEffector`. Mounts the optional load-time fault.
fn build_effector(
    index: usize,
    config: &EffectorConfig,
    dt: Duration,
) -> Result<Box<dyn ControlEffector>, RunnerError> {
    let id = EffectorId::from_path(&format!("vehicle.assembly.effectors.{id}", id = config.id));
    let limits = EffectorLimits {
        min: config.limits.min,
        max: config.limits.max,
        max_rate_per_s: config.limits.max_rate_per_s,
        deadband: config.limits.deadband,
        latency: Duration::from_seconds(config.limits.latency_s),
    };
    let initial_position = config.initial_position.unwrap_or(0.0);
    let mut actuator: Box<dyn ControlEffector> = match &config.kind {
        EffectorKindConfig::LinearActuator { tau_s } => {
            let tau_s = tau_s.unwrap_or(0.0);
            Box::new(
                LinearActuator::new(id, limits, dt, initial_position, tau_s).map_err(|err| {
                    RunnerError::Assembly {
                        field: format!("vehicle.assembly.effectors[{index}]"),
                        reason: err.to_string(),
                    }
                })?,
            )
        }
        EffectorKindConfig::SecondOrderServo {
            natural_frequency_rad_s,
            damping_ratio,
            max_accel_per_s2,
            backlash_half_width,
        } => Box::new(
            SecondOrderServo::new(
                id,
                limits,
                SecondOrderServoParams {
                    natural_frequency_rad_s: *natural_frequency_rad_s,
                    damping_ratio: *damping_ratio,
                    max_accel_per_s2: *max_accel_per_s2,
                    backlash_half_width: *backlash_half_width,
                },
                dt,
                initial_position,
            )
            .map_err(|err| RunnerError::Assembly {
                field: format!("vehicle.assembly.effectors[{index}]"),
                reason: err.to_string(),
            })?,
        ),
        // Direct-torque effectors default to the first-order linear-actuator
        // dynamics; the kind tag tells the moment-model layer to interpret the
        // deflection as a body torque command rather than feed it to an aero
        // deck. When `rcs` is declared, the same scalar command path is
        // quantized through either scalar MIB/PWPF or a physical RCS bank
        // before the existing direct-torque moment adapter consumes the
        // average output.
        EffectorKindConfig::DirectTorque {
            axis,
            effectiveness_n_m_per_rad,
            rcs,
        } => {
            if let Some(rcs) = rcs {
                match rcs.resolved_mode() {
                    DirectTorqueRcsMode::Scalar => Box::new(
                        RcsPulseEffector::new(
                            id,
                            limits,
                            RcsPulseEffectorParams {
                                minimum_impulse_n_s: rcs.minimum_impulse_n_s.ok_or_else(|| {
                                    RunnerError::Assembly {
                                        field: format!(
                                            "vehicle.assembly.effectors[{index}].kind.rcs.minimum_impulse_n_s"
                                        ),
                                        reason: "missing scalar RCS minimum impulse".to_owned(),
                                    }
                                })?,
                                nominal_thrust_n: rcs.nominal_thrust_n.ok_or_else(|| {
                                    RunnerError::Assembly {
                                        field: format!(
                                            "vehicle.assembly.effectors[{index}].kind.rcs.nominal_thrust_n"
                                        ),
                                        reason: "missing scalar RCS nominal thrust".to_owned(),
                                    }
                                })?,
                                pwpf: rcs.pwpf.as_ref().map(map_pwpf_params),
                            },
                            dt,
                            initial_position,
                        )
                        .map_err(|err| RunnerError::Assembly {
                            field: format!("vehicle.assembly.effectors[{index}]"),
                            reason: err.to_string(),
                        })?,
                    ),
                    DirectTorqueRcsMode::CoupledBank => Box::new(
                        LinearActuator::new(id, limits, dt, initial_position, 0.0).map_err(
                            |err| RunnerError::Assembly {
                                field: format!("vehicle.assembly.effectors[{index}]"),
                                reason: err.to_string(),
                            },
                        )?,
                    ),
                    DirectTorqueRcsMode::Bank => Box::new(
                        RcsThrusterBankEffector::new(
                            id,
                            limits,
                            RcsThrusterBankEffectorParams {
                                axis_index: axis.body_axis_index(),
                                command_effectiveness_n_m_per_unit: *effectiveness_n_m_per_rad,
                                thrusters: rcs
                                    .thrusters
                                    .iter()
                                    .map(map_rcs_thruster_config)
                                    .collect(),
                                pwpf: rcs.pwpf.as_ref().map(map_pwpf_params),
                            },
                            dt,
                            initial_position,
                        )
                        .map_err(|err| RunnerError::Assembly {
                            field: format!("vehicle.assembly.effectors[{index}]"),
                            reason: err.to_string(),
                        })?,
                    ),
                }
            } else {
                Box::new(
                    LinearActuator::new(id, limits, dt, initial_position, 0.0).map_err(|err| {
                        RunnerError::Assembly {
                            field: format!("vehicle.assembly.effectors[{index}]"),
                            reason: err.to_string(),
                        }
                    })?,
                )
            }
        }
    };
    if let Some(fault_config) = &config.fault {
        let fault = match *fault_config {
            EffectorFaultConfig::Jam { at } => EffectorFault::Jam { at },
            EffectorFaultConfig::Runaway { rate_per_s } => EffectorFault::Runaway { rate_per_s },
            EffectorFaultConfig::ReducedRate { factor } => EffectorFault::ReducedRate { factor },
            EffectorFaultConfig::Hardover { to } => EffectorFault::Hardover { to },
            EffectorFaultConfig::Oscillatory {
                amplitude,
                frequency_hz,
                phase_rad,
            } => EffectorFault::Oscillatory {
                amplitude,
                frequency_hz,
                phase_rad,
            },
        };
        actuator
            .inject_fault(fault)
            .map_err(|err| RunnerError::Assembly {
                field: format!("vehicle.assembly.effectors[{index}].fault"),
                reason: err.to_string(),
            })?;
    }
    Ok(actuator)
}

#[derive(Debug)]
struct CoupledRcsGroupBuilder {
    group_id: String,
    thrusters: Vec<RcsThrusterConfig>,
    members: Vec<CoupledRcsMember>,
    feeds: Vec<Option<RcsTankFeedBinding>>,
}

fn build_bank_rcs_feeds(
    document: &ScenarioDocument,
) -> Result<BTreeMap<usize, Vec<Option<RcsTankFeedBinding>>>, RunnerError> {
    let mut feeds = BTreeMap::new();
    for (index, config) in document.vehicle.assembly.effectors.iter().enumerate() {
        let EffectorKindConfig::DirectTorque { rcs: Some(rcs), .. } = &config.kind else {
            continue;
        };
        if rcs.resolved_mode() != DirectTorqueRcsMode::Bank {
            continue;
        }
        let bindings = rcs
            .thrusters
            .iter()
            .map(map_rcs_tank_feed_binding)
            .collect::<Vec<_>>();
        if bindings.iter().any(Option::is_some) {
            feeds.insert(index, bindings);
        }
    }
    Ok(feeds)
}

fn build_coupled_rcs_groups(
    document: &ScenarioDocument,
    dt: Duration,
) -> Result<Vec<CoupledRcsGroup>, RunnerError> {
    let mut builders: BTreeMap<String, CoupledRcsGroupBuilder> = BTreeMap::new();

    for (index, config) in document.vehicle.assembly.effectors.iter().enumerate() {
        let EffectorKindConfig::DirectTorque {
            axis,
            effectiveness_n_m_per_rad,
            rcs: Some(rcs),
        } = &config.kind
        else {
            continue;
        };
        if rcs.resolved_mode() != DirectTorqueRcsMode::CoupledBank {
            continue;
        }
        let group_id = rcs.group.as_ref().ok_or_else(|| RunnerError::Assembly {
            field: format!("vehicle.assembly.effectors[{index}].kind.rcs.group"),
            reason: "missing coupled_bank RCS group".to_owned(),
        })?;
        let member = CoupledRcsMember {
            effector_index: index,
            axis_index: axis.body_axis_index(),
            effectiveness_n_m_per_unit: *effectiveness_n_m_per_rad,
            pwpf: rcs
                .pwpf
                .as_ref()
                .map(map_pwpf_params)
                .map(PwpfModulator::new)
                .transpose()
                .map_err(|err| RunnerError::Assembly {
                    field: format!("vehicle.assembly.effectors[{index}].kind.rcs.pwpf"),
                    reason: err.to_string(),
                })?,
        };
        if let Some(builder) = builders.get_mut(group_id) {
            builder.members.push(member);
        } else {
            builders.insert(
                group_id.clone(),
                CoupledRcsGroupBuilder {
                    group_id: group_id.clone(),
                    thrusters: rcs.thrusters.iter().map(map_rcs_thruster_config).collect(),
                    feeds: rcs
                        .thrusters
                        .iter()
                        .map(map_rcs_tank_feed_binding)
                        .collect::<Vec<_>>(),
                    members: vec![member],
                },
            );
        }
    }

    builders
        .into_values()
        .map(|builder| {
            let allocator = RcsCoupledAllocator::new(builder.thrusters, dt).map_err(|err| {
                RunnerError::Assembly {
                    field: format!(
                        "vehicle.assembly.effectors[*].kind.rcs.group={}",
                        builder.group_id
                    ),
                    reason: err.to_string(),
                }
            })?;
            Ok(CoupledRcsGroup {
                group_id: builder.group_id,
                allocator,
                members: builder.members,
                feeds: builder.feeds,
            })
        })
        .collect()
}

fn map_rcs_tank_feed_binding(
    thruster: &openbmp_scenario::RcsThrusterConfig,
) -> Option<RcsTankFeedBinding> {
    thruster.feed.as_ref().map(|feed| RcsTankFeedBinding {
        tank_id: TankId::from_path(&format!("vehicle.assembly.tanks.{id}", id = feed.tank)),
        specific_impulse_s: feed.specific_impulse_s,
        pressure_exponent: feed.pressure_exponent,
    })
}

trait RcsFeedPulseView {
    fn thruster_index(&self) -> usize;
    fn actual_impulse_n_s(&self) -> f64;
}

impl RcsFeedPulseView for RcsThrusterBankPulse {
    fn thruster_index(&self) -> usize {
        self.thruster_index
    }

    fn actual_impulse_n_s(&self) -> f64 {
        self.actual_impulse_n_s
    }
}

impl RcsFeedPulseView for RcsCoupledThrusterPulse {
    fn thruster_index(&self) -> usize {
        self.thruster_index
    }

    fn actual_impulse_n_s(&self) -> f64 {
        self.actual_impulse_n_s
    }
}

fn rcs_feed_pressure_scales(
    feeds: &[Option<RcsTankFeedBinding>],
    tank_states: Option<&BTreeMap<TankId, PropellantTankState>>,
    field: &str,
) -> Result<Vec<f64>, RunnerError> {
    let mut scales = Vec::with_capacity(feeds.len());
    for feed in feeds {
        let Some(feed) = feed else {
            scales.push(1.0);
            continue;
        };
        let tank_states = tank_states.ok_or_else(|| RunnerError::Effector {
            field: field.to_owned(),
            reason: "RCS tank feed requires live tank states".to_owned(),
        })?;
        let tank = tank_states
            .get(&feed.tank_id)
            .ok_or_else(|| RunnerError::Effector {
                field: field.to_owned(),
                reason: format!(
                    "RCS tank feed references missing tank {}",
                    feed.tank_id.value()
                ),
            })?;
        let scale = if tank.fluid_remaining_kg <= 0.0 {
            0.0
        } else {
            tank.blowdown_pressure_scale()
                .map_err(|err| RunnerError::Effector {
                    field: field.to_owned(),
                    reason: err.to_string(),
                })?
                .powf(feed.pressure_exponent)
        };
        if !scale.is_finite() || scale < 0.0 {
            return Err(RunnerError::Effector {
                field: field.to_owned(),
                reason: "RCS tank feed pressure scale is non-finite or negative".to_owned(),
            });
        }
        scales.push(scale);
    }
    Ok(scales)
}

fn rcs_feed_drain_rates<P: RcsFeedPulseView>(
    feeds: &[Option<RcsTankFeedBinding>],
    pulses: &[P],
    dt_s: f64,
) -> BTreeMap<TankId, f64> {
    let mut drains = BTreeMap::new();
    if dt_s <= 0.0 {
        return drains;
    }
    for pulse in pulses {
        let Some(feed) = feeds.get(pulse.thruster_index()).and_then(Option::as_ref) else {
            continue;
        };
        let mass_kg =
            pulse.actual_impulse_n_s().abs() / (feed.specific_impulse_s * STANDARD_GRAVITY_M_S2);
        if mass_kg > 0.0 && mass_kg.is_finite() {
            *drains.entry(feed.tank_id).or_insert(0.0) += mass_kg / dt_s;
        }
    }
    drains
}

fn merge_drain_rates(target: &mut BTreeMap<TankId, f64>, source: BTreeMap<TankId, f64>) {
    for (tank, rate) in source {
        *target.entry(tank).or_insert(0.0) += rate;
    }
}

fn map_rcs_thruster_config(thruster: &openbmp_scenario::RcsThrusterConfig) -> RcsThrusterConfig {
    RcsThrusterConfig {
        position_body_m: thruster.position_body_m,
        direction_body: thruster.direction_body,
        minimum_impulse_n_s: thruster.minimum_impulse_n_s,
        nominal_thrust_n: thruster.nominal_thrust_n,
        blowdown: thruster
            .blowdown
            .as_ref()
            .map(|blowdown| RcsBlowdownParams {
                initial_pressure_pa: blowdown.initial_pressure_pa,
                minimum_pressure_pa: blowdown.minimum_pressure_pa,
                usable_impulse_n_s: blowdown.usable_impulse_n_s,
                pressure_exponent: blowdown.pressure_exponent,
            }),
    }
}

fn map_pwpf_params(pwpf: &openbmp_scenario::PwpfConfig) -> PwpfParams {
    PwpfParams {
        gain: pwpf.gain,
        time_constant_s: pwpf.time_constant_s,
        on_threshold: pwpf.on_threshold,
        off_threshold: pwpf.off_threshold,
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use openbmp_core::{EffectorId, SimTime, StepIndex, TankId};
    use openbmp_scenario::Scenario;
    use openbmp_sim::{EventId, FiredEvent, ScenarioScriptAction};

    use super::*;

    const ASSEMBLY_WITH_EFFECTOR: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../crates/openbmp-scenario/tests/fixtures/assembly-with-effector.toml"
    ));

    fn scenario_without_schedule() -> Scenario {
        let toml = ASSEMBLY_WITH_EFFECTOR.replace(
            "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
            "",
        );
        Scenario::from_toml_str(&toml).expect("scenario parses")
    }

    fn rcs_feed_tank_toml() -> &'static str {
        "[[vehicle.assembly.tanks]]\n\
         id = \"rcs_prop\"\n\
         mounted_to = \"main\"\n\
         mount_point_body_m = [0.0, 0.0, 0.0]\n\
         geometry = { kind = \"cylinder\", radius_m = 0.5, height_m = 1.0 }\n\
         propellant = { density_kg_m3 = 1000.0, label = \"mono_textbook\" }\n\
         initial_fill_fraction = 0.5\n\
         moving_mass = { kind = \"rigid_liquid\" }\n\
         ullage = { initial_pressure_pa = 2000000.0, gas_gamma = 2.0 }\n"
    }

    fn with_rcs_feed_tank(toml: &str) -> String {
        toml.replace(
            "dry_mass_kg   = 1.0\n\n[[vehicle.assembly.effectors]]",
            &format!(
                "dry_mass_kg   = 1.0\n\n{}\n[[vehicle.assembly.effectors]]",
                rcs_feed_tank_toml()
            ),
        )
    }

    fn rcs_prop_tank_id() -> TankId {
        TankId::from_path("vehicle.assembly.tanks.rcs_prop")
    }

    fn half_pressure_tank_states() -> std::collections::BTreeMap<TankId, PropellantTankState> {
        let mut states = std::collections::BTreeMap::new();
        let current_ullage_m3 = 0.5 / 0.5_f64.sqrt();
        states.insert(
            rcs_prop_tank_id(),
            PropellantTankState {
                fluid_remaining_kg: (1.0 - current_ullage_m3) * 1000.0,
                initial_fluid_mass_kg: 500.0,
                volume_m3: 1.0,
                density_kg_m3: 1000.0,
                has_ullage: true,
                ullage_gamma: 2.0,
            },
        );
        states
    }

    fn expected_drain_rate(actual_impulse_n_s: f64) -> f64 {
        actual_impulse_n_s / (100.0 * STANDARD_GRAVITY_M_S2) / 0.001
    }

    fn coupled_rcs_thrusters_toml() -> &'static str {
        "{ id = \"roll_pos\", position_body_m = [0.0, 1.0, 0.0], direction_body = [0.0, 0.0, 1.0], minimum_impulse_n_s = 0.0001, nominal_thrust_n = 0.2 }, \
         { id = \"roll_neg\", position_body_m = [0.0, 1.0, 0.0], direction_body = [0.0, 0.0, -1.0], minimum_impulse_n_s = 0.0001, nominal_thrust_n = 0.2 }, \
         { id = \"pitch_pos\", position_body_m = [0.0, 0.0, 1.0], direction_body = [1.0, 0.0, 0.0], minimum_impulse_n_s = 0.0001, nominal_thrust_n = 0.2 }, \
         { id = \"pitch_neg\", position_body_m = [0.0, 0.0, 1.0], direction_body = [-1.0, 0.0, 0.0], minimum_impulse_n_s = 0.0001, nominal_thrust_n = 0.2 }, \
         { id = \"yaw_pos\", position_body_m = [1.0, 0.0, 0.0], direction_body = [0.0, 1.0, 0.0], minimum_impulse_n_s = 0.0001, nominal_thrust_n = 0.2 }, \
         { id = \"yaw_neg\", position_body_m = [1.0, 0.0, 0.0], direction_body = [0.0, -1.0, 0.0], minimum_impulse_n_s = 0.0001, nominal_thrust_n = 0.2 }"
    }

    fn coupled_rcs_three_axis_toml() -> String {
        let thrusters = coupled_rcs_thrusters_toml();
        let effectors = format!(
            "[[vehicle.assembly.effectors]]\n\
             id = \"roll_rcs\"\n\
             kind = {{ kind = \"direct_torque\", axis = \"roll\", effectiveness_n_m_per_rad = 1.0, rcs = {{ mode = \"coupled_bank\", group = \"acs\", thrusters = [ {thrusters} ] }} }}\n\
             limits = {{ min = -0.349, max = 0.349, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }}\n\
             initial_position = 0.0\n\
             unit = \"rad\"\n\
             command_schedule = {{ kind = \"constant\", value = 0.01 }}\n\n\
             [[vehicle.assembly.effectors]]\n\
             id = \"pitch_rcs\"\n\
             kind = {{ kind = \"direct_torque\", axis = \"pitch\", effectiveness_n_m_per_rad = 1.0, rcs = {{ mode = \"coupled_bank\", group = \"acs\", thrusters = [ {thrusters} ] }} }}\n\
             limits = {{ min = -0.349, max = 0.349, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }}\n\
             initial_position = 0.0\n\
             unit = \"rad\"\n\
             command_schedule = {{ kind = \"constant\", value = 0.02 }}\n\n\
             [[vehicle.assembly.effectors]]\n\
             id = \"yaw_rcs\"\n\
             kind = {{ kind = \"direct_torque\", axis = \"yaw\", effectiveness_n_m_per_rad = 1.0, rcs = {{ mode = \"coupled_bank\", group = \"acs\", thrusters = [ {thrusters} ] }} }}\n\
             limits = {{ min = -0.349, max = 0.349, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }}\n\
             initial_position = 0.0\n\
             unit = \"rad\"\n\
             command_schedule = {{ kind = \"constant\", value = -0.01 }}"
        );
        ASSEMBLY_WITH_EFFECTOR
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "[[vehicle.assembly.effectors]]\nid               = \"delta_e\"\nkind             = { kind = \"linear_actuator\", tau_s = 0.05 }\nlimits           = { min = -0.349, max = 0.349, max_rate_per_s = 5.236, deadband = 0.0, latency_s = 0.020 }\ninitial_position = 0.0\nunit             = \"rad\"\ncommand_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
                &effectors,
            )
    }

    fn override_event(id: EffectorId, command: f64) -> FiredEvent<ScenarioScriptAction> {
        FiredEvent {
            binding_id: EventId::from_path("mission.events.override"),
            step: StepIndex::new(1),
            time: SimTime::from_seconds(0.0),
            action: ScenarioScriptAction::EffectorOverride { id, command },
        }
    }

    #[test]
    fn unscheduled_override_is_one_step_only() {
        let scenario = scenario_without_schedule();
        let mut rack = EffectorRack::build(&scenario.document).unwrap();
        let id = EffectorId::from_path("vehicle.assembly.effectors.delta_e");

        rack.apply_overrides(&[override_event(id, 0.087)]).unwrap();
        rack.step(SimTime::from_seconds(0.0)).unwrap();
        assert!((rack.snapshot()[0].commanded - 0.087).abs() < 1.0e-12);

        rack.step(SimTime::from_seconds(0.001)).unwrap();
        assert!(rack.snapshot()[0].commanded.abs() < 1.0e-12);
    }

    #[test]
    fn piecewise_linear_schedule_interpolates_and_holds_endpoints() {
        let toml = ASSEMBLY_WITH_EFFECTOR.replace(
            "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
            "command_schedule = { kind = \"piecewise_linear\", points = [ \
             { time_s = 0.25, value = 0.0 }, \
             { time_s = 0.50, value = 0.10 }, \
             { time_s = 1.00, value = 0.20 } ] }",
        );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = EffectorRack::build(&scenario.document).unwrap();

        rack.step(SimTime::from_seconds(0.0)).unwrap();
        assert!(rack.snapshot()[0].commanded.abs() < 1.0e-12);

        rack.step(SimTime::from_seconds(0.375)).unwrap();
        assert!((rack.snapshot()[0].commanded - 0.05).abs() < 1.0e-12);

        rack.step(SimTime::from_seconds(0.75)).unwrap();
        assert!((rack.snapshot()[0].commanded - 0.15).abs() < 1.0e-12);

        rack.step(SimTime::from_seconds(2.0)).unwrap();
        assert!((rack.snapshot()[0].commanded - 0.20).abs() < 1.0e-12);
    }

    #[test]
    fn runtime_override_unknown_id_fails_closed() {
        let scenario = scenario_without_schedule();
        let mut rack = EffectorRack::build(&scenario.document).unwrap();
        let err = rack
            .apply_overrides(&[override_event(EffectorId::from_path("unknown"), 0.087)])
            .unwrap_err();
        assert!(matches!(err, RunnerError::Effector { .. }));
    }

    #[test]
    fn second_order_servo_kind_builds_and_steps_from_scenario() {
        let toml = ASSEMBLY_WITH_EFFECTOR
            .replace(
                "kind             = { kind = \"linear_actuator\", tau_s = 0.05 }",
                "kind             = { kind = \"second_order_servo\", \
                 natural_frequency_rad_s = 50.0, damping_ratio = 1.0, \
                 max_accel_per_s2 = 1000000.0, backlash_half_width = 0.0 }",
            )
            .replace("latency_s = 0.020", "latency_s = 0.0")
            .replace(
                "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
                "command_schedule = { kind = \"constant\", value = 0.1 }",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = EffectorRack::build(&scenario.document).unwrap();

        rack.step(SimTime::from_seconds(0.0)).unwrap();
        let state = rack.snapshot()[0];
        assert!((state.commanded - 0.1).abs() < 1.0e-12);
        assert!(
            state.actual > 0.0 && state.actual < 0.1,
            "second-order servo should move toward, not jump to, the command: {state:?}"
        );
    }

    #[test]
    fn oscillatory_fault_builds_and_offsets_command_from_scenario() {
        let toml = ASSEMBLY_WITH_EFFECTOR
            .replace("latency_s = 0.020", "latency_s = 0.0")
            .replace(
                "initial_position = 0.0",
                "initial_position = 0.0\nfault = { kind = \"oscillatory\", amplitude = 0.1, frequency_hz = 2.0, phase_rad = 1.5707963267948966 }",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = EffectorRack::build(&scenario.document).unwrap();

        rack.step(SimTime::from_seconds(0.0)).unwrap();
        let state = rack.snapshot()[0];
        assert!((state.commanded - 0.1).abs() < 1.0e-12);
        assert!(state.actual > 0.0);
        assert!(matches!(
            state.fault,
            Some(EffectorFault::Oscillatory { .. })
        ));
    }

    #[test]
    fn direct_torque_rcs_pulse_kind_builds_and_quantizes_from_scenario() {
        let toml = ASSEMBLY_WITH_EFFECTOR
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "kind             = { kind = \"linear_actuator\", tau_s = 0.05 }",
                "kind             = { kind = \"direct_torque\", axis = \"pitch\", \
                 effectiveness_n_m_per_rad = 1.0, rcs = { minimum_impulse_n_s = 0.0001, \
                 nominal_thrust_n = 0.2 } }",
            )
            .replace("latency_s = 0.020", "latency_s = 0.0")
            .replace(
                "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
                "command_schedule = { kind = \"constant\", value = 0.01 }",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = EffectorRack::build(&scenario.document).unwrap();

        rack.step(SimTime::from_seconds(0.0)).unwrap();
        let state = rack.snapshot()[0];
        assert!((state.commanded - 0.01).abs() < 1.0e-12);
        assert!((state.actual - 0.1).abs() < 1.0e-12);
        assert!(
            state.rate_limited,
            "MIB flooring should be reported as rate-limited/quantized"
        );
    }

    #[test]
    fn direct_torque_rcs_bank_kind_builds_and_quantizes_from_scenario() {
        let toml = ASSEMBLY_WITH_EFFECTOR
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "kind             = { kind = \"linear_actuator\", tau_s = 0.05 }",
                "kind             = { kind = \"direct_torque\", axis = \"pitch\", \
                 effectiveness_n_m_per_rad = 2.0, rcs = { mode = \"bank\", \
                 thrusters = [ \
                 { id = \"pitch_pos\", position_body_m = [1.0, 0.0, 0.0], \
                 direction_body = [0.0, 0.0, -1.0], minimum_impulse_n_s = 0.0001, \
                 nominal_thrust_n = 0.2 }, \
                 { id = \"pitch_neg\", position_body_m = [1.0, 0.0, 0.0], \
                 direction_body = [0.0, 0.0, 1.0], minimum_impulse_n_s = 0.0001, \
                 nominal_thrust_n = 0.2 } ] } }",
            )
            .replace("latency_s = 0.020", "latency_s = 0.0")
            .replace(
                "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
                "command_schedule = { kind = \"constant\", value = 0.01 }",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = EffectorRack::build(&scenario.document).unwrap();

        rack.step(SimTime::from_seconds(0.0)).unwrap();
        let state = rack.snapshot()[0];
        assert!((state.commanded - 0.01).abs() < 1.0e-12);
        assert!((state.actual - 0.05).abs() < 1.0e-12);
        assert!(
            state.rate_limited,
            "bank MIB flooring should be reported as rate-limited/quantized"
        );
    }

    #[test]
    fn direct_torque_rcs_bank_tank_feed_derates_thrust_and_reports_drain() {
        let toml = ASSEMBLY_WITH_EFFECTOR
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "kind             = { kind = \"linear_actuator\", tau_s = 0.05 }",
                "kind             = { kind = \"direct_torque\", axis = \"pitch\", \
                 effectiveness_n_m_per_rad = 1.0, rcs = { mode = \"bank\", \
                 thrusters = [ \
                 { id = \"pitch_pos\", position_body_m = [1.0, 0.0, 0.0], \
                 direction_body = [0.0, 0.0, -1.0], minimum_impulse_n_s = 0.0001, \
                 nominal_thrust_n = 1.0, feed = { tank = \"rcs_prop\", \
                 specific_impulse_s = 100.0 } }, \
                 { id = \"pitch_neg\", position_body_m = [1.0, 0.0, 0.0], \
                 direction_body = [0.0, 0.0, 1.0], minimum_impulse_n_s = 0.0001, \
                 nominal_thrust_n = 1.0 } ] } }",
            )
            .replace(
                "limits           = { min = -0.349, max = 0.349, max_rate_per_s = 5.236, deadband = 0.0, latency_s = 0.020 }",
                "limits           = { min = -2.0, max = 2.0, max_rate_per_s = 1000.0, deadband = 0.0, latency_s = 0.0 }",
            )
            .replace(
                "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
                "command_schedule = { kind = \"constant\", value = 1.0 }",
            );
        let scenario =
            Scenario::from_toml_str(&with_rcs_feed_tank(&toml)).expect("scenario parses");
        let mut rack = EffectorRack::build(&scenario.document).unwrap();

        rack.step_with_tanks(SimTime::from_seconds(0.0), &half_pressure_tank_states())
            .unwrap();
        let state = rack.snapshot()[0];
        assert!((state.commanded - 1.0).abs() < 1e-12);
        assert!((state.actual - 0.5).abs() < 1e-12);
        assert!(state.saturated);
        assert!(state.rate_limited);

        let drains = rack.rcs_feed_drain_rates();
        let drain = drains.get(&rcs_prop_tank_id()).copied().unwrap_or(0.0);
        assert!((drain - expected_drain_rate(0.0005)).abs() < 1e-12);
    }

    #[test]
    fn direct_torque_rcs_coupled_bank_group_builds_and_quantizes_from_scenario() {
        let toml = coupled_rcs_three_axis_toml();
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = EffectorRack::build(&scenario.document).unwrap();

        rack.step(SimTime::from_seconds(0.0)).unwrap();
        let snapshot = rack.snapshot();
        assert_eq!(snapshot.len(), 3);

        assert!((snapshot[0].commanded - 0.01).abs() < 1.0e-12);
        assert!((snapshot[1].commanded - 0.02).abs() < 1.0e-12);
        assert!((snapshot[2].commanded + 0.01).abs() < 1.0e-12);
        assert!((snapshot[0].actual - 0.1).abs() < 1.0e-12);
        assert!((snapshot[1].actual - 0.1).abs() < 1.0e-12);
        assert!((snapshot[2].actual + 0.1).abs() < 1.0e-12);
        assert!(snapshot.iter().all(|state| state.rate_limited));
        assert!(snapshot.iter().all(|state| !state.saturated));
    }

    #[test]
    fn direct_torque_rcs_coupled_bank_tank_feed_reports_shared_drain() {
        let toml = coupled_rcs_three_axis_toml()
            .replace(
                "nominal_thrust_n = 0.2",
                "nominal_thrust_n = 0.2, feed = { tank = \"rcs_prop\", specific_impulse_s = 100.0 }",
            )
            .replace("max_rate_per_s = 100.0", "max_rate_per_s = 1000.0")
            .replace(
                "command_schedule = { kind = \"constant\", value = 0.01 }",
                "command_schedule = { kind = \"constant\", value = 0.2 }",
            )
            .replace(
                "command_schedule = { kind = \"constant\", value = 0.02 }",
                "command_schedule = { kind = \"constant\", value = 0.0 }",
            )
            .replace(
                "command_schedule = { kind = \"constant\", value = -0.01 }",
                "command_schedule = { kind = \"constant\", value = 0.0 }",
            );
        let scenario =
            Scenario::from_toml_str(&with_rcs_feed_tank(&toml)).expect("scenario parses");
        let mut rack = EffectorRack::build(&scenario.document).unwrap();

        rack.step_with_tanks(SimTime::from_seconds(0.0), &half_pressure_tank_states())
            .unwrap();
        let snapshot = rack.snapshot();
        assert!((snapshot[0].commanded - 0.2).abs() < 1e-12);
        assert!((snapshot[0].actual - 0.1).abs() < 1e-12);
        assert!(snapshot[0].saturated);

        let drains = rack.rcs_feed_drain_rates();
        let drain = drains.get(&rcs_prop_tank_id()).copied().unwrap_or(0.0);
        assert!((drain - expected_drain_rate(0.0001)).abs() < 1e-12);
    }

    #[test]
    fn direct_torque_rcs_coupled_bank_pwpf_gates_group_axes_from_scenario() {
        let toml = coupled_rcs_three_axis_toml().replace(
            "group = \"acs\", thrusters",
            "group = \"acs\", pwpf = { gain = 1.0, time_constant_s = 0.01, on_threshold = 10.0, off_threshold = 0.0 }, thrusters",
        );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let mut rack = EffectorRack::build(&scenario.document).unwrap();

        rack.step(SimTime::from_seconds(0.0)).unwrap();
        let snapshot = rack.snapshot();

        assert!((snapshot[0].commanded - 0.01).abs() < 1.0e-12);
        assert!((snapshot[1].commanded - 0.02).abs() < 1.0e-12);
        assert!((snapshot[2].commanded + 0.01).abs() < 1.0e-12);
        assert!(snapshot.iter().all(|state| state.actual.abs() < 1.0e-12));
        assert!(snapshot.iter().all(|state| state.rate_limited));
        assert!(snapshot.iter().all(|state| !state.saturated));
    }
}
