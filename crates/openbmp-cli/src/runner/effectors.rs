//! Phase-3.4 runner-side effector rack.
//!
//! The rack owns the `Vec<Box<dyn ControlEffector>>` resolved from
//! the scenario's `[[vehicle.assembly.effectors]]` block, plus the
//! per-effector deterministic command schedules and one-shot
//! override map. It is stepped once per kernel base tick **before**
//! the kernel's `step()` so the effector telemetry observable in
//! the Parquet matches the kernel's view of the world.
//!
//! Phase-3.4 leaves the kernel's force / moment evaluation untouched
//! when the rack is empty — legacy scenarios produce byte-identical
//! Parquet because the runner short-circuits every rack-related
//! operation on `is_empty()`.
//!
//! Aero-deck schema-2 consumption of effector deflections lands in
//! Phase 3.5; in 3.4 the rack snapshot is read only by the
//! telemetry layer.

use std::collections::BTreeMap;

use openbmp_core::{Duration, EffectorId, SimTime};
use openbmp_scenario::{
    EffectorCommandScheduleConfig, EffectorConfig, EffectorFaultConfig, EffectorKindConfig,
    ScenarioDocument,
};
use openbmp_sim::{FiredEvent, ScenarioScriptAction};
use openbmp_vehicle::{
    ControlEffector, EffectorFault, EffectorLimits, EffectorState, LinearActuator,
};

use crate::error::CliError;

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
    /// One-shot command overrides drained from the kernel's
    /// `EventAction::EffectorOverride` events. Applied on the next
    /// rack tick and cleared after each `step()`.
    overrides: BTreeMap<EffectorId, f64>,
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
}

impl EffectorSchedule {
    fn from_config(config: &EffectorCommandScheduleConfig) -> Self {
        match *config {
            EffectorCommandScheduleConfig::Constant { value } => Self::Constant(value),
            EffectorCommandScheduleConfig::StepAt {
                time_s,
                before,
                after,
            } => Self::StepAt {
                time_s,
                before,
                after,
            },
            EffectorCommandScheduleConfig::LinearRamp {
                start_time_s,
                end_time_s,
                start,
                end,
            } => Self::LinearRamp {
                start_time_s,
                end_time_s,
                start,
                end,
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
        }
    }
}

impl EffectorRack {
    /// Build the rack from the parsed scenario document. Returns an
    /// empty rack when no `[[vehicle.assembly.effectors]]` blocks
    /// are declared.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Assembly`] when a `LinearActuator::new`
    /// call rejects the scenario-declared limits / initial position
    /// / latency / `tau_s`.
    pub fn build(document: &ScenarioDocument) -> Result<Self, CliError> {
        let dt = Duration::from_seconds(document.time.dt_s);
        let mut effectors: Vec<Box<dyn ControlEffector>> = Vec::new();
        let mut string_ids: Vec<String> = Vec::new();
        let mut id_index: BTreeMap<EffectorId, usize> = BTreeMap::new();
        let mut schedules: Vec<Option<EffectorSchedule>> = Vec::new();
        let mut hold_commands: Vec<f64> = Vec::new();

        for (index, config) in document.vehicle.assembly.effectors.iter().enumerate() {
            let actuator = build_effector(index, config, dt)?;
            let id = actuator.id();
            effectors.push(Box::new(actuator));
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

        Ok(Self {
            effectors,
            string_ids,
            id_index,
            schedules,
            hold_commands,
            overrides: BTreeMap::new(),
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

    /// Apply any `EventAction::EffectorOverride` actions drained
    /// from the kernel's per-step fired-event queue. Override values
    /// take precedence over the schedule for the next rack tick only.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Effector`] when an override references an
    /// effector id that is not present in this rack.
    pub fn apply_overrides(
        &mut self,
        fired: &[FiredEvent<ScenarioScriptAction>],
    ) -> Result<(), CliError> {
        for event in fired {
            if let ScenarioScriptAction::EffectorOverride { id, command } = event.action {
                if !self.id_index.contains_key(&id) {
                    return Err(CliError::Effector {
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
    /// Returns [`CliError::Effector`] when the FC references an
    /// effector id that is not present in this rack.
    pub fn apply_fc_commands(
        &mut self,
        commands: &openbmp_fc::topics::EffectorCommandSet,
    ) -> Result<(), CliError> {
        for command in commands.commands.iter().take(usize::from(commands.count)) {
            let id = EffectorId::new(command.effector_id);
            if !self.id_index.contains_key(&id) {
                return Err(CliError::Effector {
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
    /// Returns [`CliError::Effector`] when `LinearActuator::step`
    /// fails (dt mismatch, non-finite command, etc.).
    pub fn step(&mut self, time: SimTime) -> Result<(), CliError> {
        for (index, effector) in self.effectors.iter_mut().enumerate() {
            let id = effector.id();
            let cmd = if let Some(&override_cmd) = self.overrides.get(&id) {
                override_cmd
            } else if let Some(schedule) = &self.schedules[index] {
                schedule.command_at(time)
            } else {
                self.hold_commands[index]
            };
            effector
                .step(cmd, self.dt)
                .map_err(|err| CliError::Effector {
                    field: format!("vehicle.assembly.effectors[{index}]"),
                    reason: err.to_string(),
                })?;
        }
        self.overrides.clear();
        Ok(())
    }

    /// Snapshot every effector's most recent `EffectorState` for
    /// telemetry, in declared order.
    #[must_use]
    pub fn snapshot(&self) -> Vec<EffectorState> {
        self.effectors.iter().map(|e| e.current_state()).collect()
    }
}

/// Phase-3.4 effector resolver: scenario `EffectorConfig` →
/// `LinearActuator` (the only kind shipped in 3.4). Mounts the
/// optional load-time fault.
fn build_effector(
    index: usize,
    config: &EffectorConfig,
    dt: Duration,
) -> Result<LinearActuator, CliError> {
    let id = EffectorId::from_path(&format!("vehicle.assembly.effectors.{id}", id = config.id));
    let limits = EffectorLimits {
        min: config.limits.min,
        max: config.limits.max,
        max_rate_per_s: config.limits.max_rate_per_s,
        deadband: config.limits.deadband,
        latency: Duration::from_seconds(config.limits.latency_s),
    };
    let initial_position = config.initial_position.unwrap_or(0.0);
    let tau_s = match config.kind {
        EffectorKindConfig::LinearActuator { tau_s } => tau_s.unwrap_or(0.0),
        // Phase-5.A.2.A: direct-torque effectors share the
        // first-order linear-actuator dynamics; the kind tag tells the
        // moment-model layer to interpret the deflection as a body
        // torque command rather than feed it to an aero deck. Default
        // tau = 0 (pure rate-clamped tracker) so the autopilot's
        // commanded torque reaches the kernel within one base tick.
        EffectorKindConfig::DirectTorque { .. } => 0.0,
    };
    let mut actuator =
        LinearActuator::new(id, limits, dt, initial_position, tau_s).map_err(|err| {
            CliError::Assembly {
                field: format!("vehicle.assembly.effectors[{index}]"),
                reason: err.to_string(),
            }
        })?;
    if let Some(fault_config) = &config.fault {
        let fault = match *fault_config {
            EffectorFaultConfig::Jam { at } => EffectorFault::Jam { at },
            EffectorFaultConfig::Runaway { rate_per_s } => EffectorFault::Runaway { rate_per_s },
            EffectorFaultConfig::ReducedRate { factor } => EffectorFault::ReducedRate { factor },
            EffectorFaultConfig::Hardover { to } => EffectorFault::Hardover { to },
        };
        actuator
            .inject_fault(fault)
            .map_err(|err| CliError::Assembly {
                field: format!("vehicle.assembly.effectors[{index}].fault"),
                reason: err.to_string(),
            })?;
    }
    Ok(actuator)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use openbmp_core::{EffectorId, SimTime, StepIndex};
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
    fn runtime_override_unknown_id_fails_closed() {
        let scenario = scenario_without_schedule();
        let mut rack = EffectorRack::build(&scenario.document).unwrap();
        let err = rack
            .apply_overrides(&[override_event(EffectorId::from_path("unknown"), 0.087)])
            .unwrap_err();
        assert!(matches!(err, CliError::Effector { .. }));
    }
}
