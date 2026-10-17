//! Phase-gated actuator mixer.
//!
//! The mixer is the only path between an autopilot's commanded
//! deflection and the actuator topic an effector consumer reads. If
//! the commander has not armed the vehicle, or the active phase's
//! `allowed_effectors` mask does not authorise the channel, the mixer
//! zero-mixes the demand and emits a warning event. This prevents a
//! controller bug in one phase from actuating in the wrong phase.
//!
//! Holds a static channel allowlist per `PhaseId`
//! in a [`crate::tables`] entry; future revisions can add more
//! sophisticated allocation matrices.

use std::collections::BTreeMap;
use std::string::{String, ToString};
use std::vec::Vec;

use openbmp_core::{EffectorId, EngineId};

use crate::allocation::{AllocatorError, ControlAllocator};
use crate::error::ControllerError;
use crate::scheduler::{Job, JobContext};
use crate::tables::Table;
use crate::topics::{
    ActuatorCommand, EffectorAvailability, EffectorCommand, EffectorCommandSet, EngineCommand,
    EngineCommandSet, EngineDemand, MAX_EFFECTOR_COMMANDS, MAX_ENGINE_COMMANDS,
    PropellantSafingCommand, PseudoControlCommand, VehicleStatus,
};

/// Mapping from semantic FC actuator channels to scenario-declared
/// control-effector ids. When no mapping is installed, the mixer keeps
/// the legacy semantic command topic only; when a mapping is present,
/// it also publishes an [`EffectorCommandSet`] keyed by `EffectorId`.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct ActuatorChannelMap {
    /// Effector that consumes aileron-equivalent command.
    pub aileron: Option<EffectorId>,
    /// Effector that consumes elevator-equivalent command.
    pub elevator: Option<EffectorId>,
    /// Effector that consumes rudder-equivalent command.
    pub rudder: Option<EffectorId>,
    /// Effector that consumes body-flap command.
    pub body_flap: Option<EffectorId>,
}

impl ActuatorChannelMap {
    fn has_any_mapping(self) -> bool {
        self.aileron.is_some()
            || self.elevator.is_some()
            || self.rudder.is_some()
            || self.body_flap.is_some()
    }
}

/// Per-phase mask of which effectors / engines are allowed authority.
///
/// `lookup` returns the entry for a known `PhaseId.value()`, falling
/// through to `default` when no per-phase entry exists. The default
/// is permissive (autopilot + engines allowed) so an
/// uninitialised mixer doesn't block ascent before the runner has
/// installed a real authority schedule.
#[derive(Clone, Debug)]
pub struct PhaseAuthorityTable {
    /// Mapping from `PhaseId.value()` to the (`allowed_effectors`, `allowed_engines`) pair.
    pub allowed: BTreeMap<u64, PhaseAuthority>,
    /// Fall-through entry used when no per-phase entry exists.
    pub default: PhaseAuthority,
}

impl Default for PhaseAuthorityTable {
    fn default() -> Self {
        Self {
            allowed: BTreeMap::new(),
            default: PhaseAuthority {
                effectors: Vec::new(),
                engines: Vec::new(),
                autopilot_allowed: true,
                engines_allowed: true,
            },
        }
    }
}

impl PhaseAuthorityTable {
    /// Returns the authority record for the given phase, falling
    /// through to [`PhaseAuthorityTable::default`] when the phase has
    /// no entry.
    #[must_use]
    pub fn lookup(&self, phase_id: u64) -> &PhaseAuthority {
        self.allowed.get(&phase_id).unwrap_or(&self.default)
    }
}

/// Authority record for one mission phase.
#[derive(Clone, Debug, Default)]
pub struct PhaseAuthority {
    /// Allowed effector ids for this phase. Empty = no allowance.
    pub effectors: Vec<EffectorId>,
    /// Allowed engine ids for this phase. Empty = no allowance.
    pub engines: Vec<openbmp_core::EngineId>,
    /// `true` if the phase permits autopilot actuator commands.
    /// Even if the effector list is non-empty, the autopilot can't
    /// drive surfaces unless this is set.
    pub autopilot_allowed: bool,
    /// `true` if the phase permits autopilot engine commands.
    pub engines_allowed: bool,
}

impl Table for PhaseAuthorityTable {
    const NAME: &'static str = "mixer.phase_authority";
    fn validate(&self) -> Result<(), String> {
        // The default-only table is valid: the fall-through is
        // permissive, so an empty `allowed` map still authorises.
        Ok(())
    }
}

/// Per-survivor gimbal trim to apply when a named engine is unavailable.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EngineOutTrimCommand {
    /// Surviving engine that receives the bias.
    pub engine: EngineId,
    /// Additive pitch-gimbal bias, rad.
    pub gimbal_pitch_bias_rad: f64,
    /// Additive yaw-gimbal bias, rad.
    pub gimbal_yaw_bias_rad: f64,
}

/// Optional flight-calibrated engine-out trim table.
///
/// This is a deterministic substrate for off-CG retrim in the scalar
/// `EngineDemand` fan-out path. It does not infer vehicle geometry; the
/// runner or scenario provides calibrated per-survivor gimbal biases keyed by
/// the failed engine id.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EngineOutTrimTable {
    /// Trim rows keyed by unavailable engine id.
    pub by_failed_engine: BTreeMap<EngineId, Vec<EngineOutTrimCommand>>,
}

impl EngineOutTrimTable {
    fn bias_for(&self, engine: EngineId, unavailable_engines: &[EngineId]) -> (f64, f64) {
        let mut pitch = 0.0;
        let mut yaw = 0.0;
        for failed_engine in unavailable_engines {
            let Some(commands) = self.by_failed_engine.get(failed_engine) else {
                continue;
            };
            for command in commands {
                if command.engine == engine {
                    pitch += command.gimbal_pitch_bias_rad;
                    yaw += command.gimbal_yaw_bias_rad;
                }
            }
        }
        (pitch, yaw)
    }
}

impl Table for EngineOutTrimTable {
    const NAME: &'static str = "mixer.engine_out_trim";

    fn validate(&self) -> Result<(), String> {
        for (failed_engine, commands) in &self.by_failed_engine {
            if commands.is_empty() {
                return Err("engine-out trim row must contain at least one command".to_string());
            }
            let mut seen = Vec::new();
            for command in commands {
                if command.engine == *failed_engine {
                    return Err(
                        "engine-out trim command cannot target the failed engine".to_string()
                    );
                }
                if seen.contains(&command.engine) {
                    return Err(
                        "engine-out trim row has duplicate survivor engine commands".to_string()
                    );
                }
                if !command.gimbal_pitch_bias_rad.is_finite()
                    || !command.gimbal_yaw_bias_rad.is_finite()
                {
                    return Err("engine-out trim gimbal biases must be finite".to_string());
                }
                seen.push(command.engine);
            }
        }
        Ok(())
    }
}

/// Mixer job — runs after the autopilot, applies phase-gating, and
/// republishes gated actuator + engine commands on the same topics.
///
/// The phase-authority mask is consulted on every tick: a phase that
/// declares `autopilot_allowed = false` zero-mixes the actuator
/// command even when the vehicle is armed and in flight, and an
/// equivalent rule applies to engine demand.
///
/// Two effector dispatch paths coexist:
/// - **Direct channel map** — `with_actuator_channel_map(...)` —
///   1:1 routing from the autopilot's `aileron / elevator / rudder /
///   body_flap` semantic channels to a single effector each.
/// - **Control allocator** — `with_allocator(...)` — distributes
///   the per-axis torque demand (`aileron_rad → roll`, `elevator_rad
///   → pitch`, `rudder_rad → yaw`) across all effectors assigned to
///   that axis via [`crate::allocation::ControlAllocator`].
///
/// When an allocator is installed, it supersedes the channel map for
/// the [`EffectorCommandSet`] publish path. The legacy
/// [`ActuatorCommand`] semantic-topic publish is always honoured for
/// downstream consumers that read it directly.
#[derive(Debug)]
pub struct Mixer {
    name: &'static str,
    last_seen_actuator: u64,
    last_seen_engine: u64,
    last_commanded_engines: Vec<EngineId>,
    authority: PhaseAuthorityTable,
    engine_out_trim: EngineOutTrimTable,
    channel_map: ActuatorChannelMap,
    allocator: Option<ControlAllocator>,
}

impl Mixer {
    /// Constructs the mixer with a permissive default phase-authority
    /// table — every phase's autopilot/engines are allowed. The runner
    /// installs a real per-phase mask via [`Mixer::with_authority`].
    #[must_use]
    pub fn new() -> Self {
        Self {
            name: "mixer.tick",
            last_seen_actuator: 0,
            last_seen_engine: 0,
            last_commanded_engines: Vec::new(),
            authority: PhaseAuthorityTable::default(),
            engine_out_trim: EngineOutTrimTable::default(),
            channel_map: ActuatorChannelMap::default(),
            allocator: None,
        }
    }

    /// Replaces the phase-authority table.
    #[must_use]
    pub fn with_authority(mut self, authority: PhaseAuthorityTable) -> Self {
        self.authority = authority;
        self
    }

    /// Replaces the optional engine-out trim table.
    #[must_use]
    pub fn with_engine_out_trim(mut self, trim: EngineOutTrimTable) -> Self {
        self.engine_out_trim = trim;
        self
    }

    /// Replaces the semantic-channel to effector-id map.
    #[must_use]
    pub fn with_actuator_channel_map(mut self, channel_map: ActuatorChannelMap) -> Self {
        self.channel_map = channel_map;
        self
    }

    /// Installs a control allocator. When set, the
    /// allocator supersedes the channel map for the
    /// [`EffectorCommandSet`] publish path; per-effector commands
    /// are gated against the active phase's `allowed_effectors`
    /// list before the proportional split so disallowed effectors do
    /// not contribute capacity. Disallowed effectors are still emitted
    /// with zero commands to withdraw authority explicitly.
    #[must_use]
    pub fn with_allocator<A>(mut self, allocator: A) -> Self
    where
        A: Into<ControlAllocator>,
    {
        self.allocator = Some(allocator.into());
        self
    }

    fn channel_allowed(&self, authority: &PhaseAuthority, channel: Option<EffectorId>) -> bool {
        if !authority.autopilot_allowed {
            return false;
        }
        if !self.channel_map.has_any_mapping() {
            return true;
        }
        channel.is_some_and(|id| authority.effectors.contains(&id))
    }

    fn gate_actuator_command(
        &self,
        cmd: ActuatorCommand,
        armed_in_flight: bool,
        authority: &PhaseAuthority,
    ) -> ActuatorCommand {
        if !armed_in_flight {
            return ActuatorCommand {
                time: cmd.time,
                saturated: cmd.saturated,
                ..ActuatorCommand::default()
            };
        }
        ActuatorCommand {
            time: cmd.time,
            elevator_rad: if self.channel_allowed(authority, self.channel_map.elevator) {
                cmd.elevator_rad
            } else {
                0.0
            },
            aileron_rad: if self.channel_allowed(authority, self.channel_map.aileron) {
                cmd.aileron_rad
            } else {
                0.0
            },
            rudder_rad: if self.channel_allowed(authority, self.channel_map.rudder) {
                cmd.rudder_rad
            } else {
                0.0
            },
            body_flap_rad: if self.channel_allowed(authority, self.channel_map.body_flap) {
                cmd.body_flap_rad
            } else {
                0.0
            },
            saturated: cmd.saturated,
        }
    }

    fn effector_command_set(
        &mut self,
        cmd: ActuatorCommand,
        pseudo_control: Option<PseudoControlCommand>,
        authority: &PhaseAuthority,
    ) -> Result<EffectorCommandSet, ControllerError> {
        let mut set = EffectorCommandSet {
            time: cmd.time,
            saturated: cmd.saturated,
            ..EffectorCommandSet::default()
        };
        if let Some(allocator) = self.allocator.as_mut() {
            // Allocator-driven dispatch. The autopilot's
            // semantic channels carry per-axis torque demand:
            //   aileron_rad  → roll
            //   elevator_rad → pitch
            //   rudder_rad   → yaw
            // Body-flap is not part of the rotational allocator
            // surface in 5.A.5 (it's a translational / aero
            // surface). The allocator distributes the three
            // rotational demands across the effectors assigned to
            // each axis. Phase authority is applied before the
            // capacity calculation so disallowed effectors do not
            // dilute or inflate the split.
            let allocation = if allocator.uses_pseudo_control() {
                let pseudo_control = pseudo_control.ok_or_else(|| {
                    ControllerError::from(AllocatorError::DenseDimensionMismatch {
                        reason: "dense_inca allocator requires autopilot.pseudo_control_cmd",
                    })
                })?;
                allocator.allocate_pseudo_control_with_allowed_effectors(
                    [
                        pseudo_control.omega_dot_des_rad_s2.x,
                        pseudo_control.omega_dot_des_rad_s2.y,
                        pseudo_control.omega_dot_des_rad_s2.z,
                    ],
                    [
                        pseudo_control.omega_dot_meas_rad_s2.x,
                        pseudo_control.omega_dot_meas_rad_s2.y,
                        pseudo_control.omega_dot_meas_rad_s2.z,
                    ],
                    &authority.effectors,
                )?
            } else {
                allocator.allocate_with_allowed_effectors(
                    [cmd.aileron_rad, cmd.elevator_rad, cmd.rudder_rad],
                    &authority.effectors,
                )?
            };
            let mut any_saturated = cmd.saturated;
            for sat in allocation.saturated_axes {
                any_saturated |= sat;
            }
            for (effector_id, command) in allocation.commands {
                let index = usize::from(set.count);
                if index >= MAX_EFFECTOR_COMMANDS {
                    return Err(AllocatorError::DenseDimensionMismatch {
                        reason: "effector command output exceeds MAX_EFFECTOR_COMMANDS",
                    }
                    .into());
                }
                set.commands[index] = EffectorCommand {
                    effector_id: effector_id.value(),
                    command,
                    saturated: any_saturated,
                };
                set.count = set.count.saturating_add(1);
            }
            set.saturated = any_saturated;
            return Ok(set);
        }
        let channels = [
            (self.channel_map.aileron, cmd.aileron_rad),
            (self.channel_map.elevator, cmd.elevator_rad),
            (self.channel_map.rudder, cmd.rudder_rad),
            (self.channel_map.body_flap, cmd.body_flap_rad),
        ];
        for (id, command) in channels {
            let Some(id) = id else { continue };
            let index = usize::from(set.count);
            if index >= MAX_EFFECTOR_COMMANDS {
                return Err(AllocatorError::DenseDimensionMismatch {
                    reason: "semantic effector command output exceeds MAX_EFFECTOR_COMMANDS",
                }
                .into());
            }
            set.commands[index] = EffectorCommand {
                effector_id: id.value(),
                command,
                saturated: cmd.saturated,
            };
            set.count = set.count.saturating_add(1);
        }
        Ok(set)
    }

    fn engine_command_set(
        &mut self,
        cmd: EngineDemand,
        authority: &PhaseAuthority,
        engine_allowed: bool,
        unavailable_engines: &[EngineId],
    ) -> EngineCommandSet {
        let mut set = EngineCommandSet {
            time: cmd.time,
            ..EngineCommandSet::default()
        };
        let zero = EngineCommand {
            throttle_unit: 0.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: false,
            shutdown: true,
            ..EngineCommand::default()
        };
        let authorized_count = authority.engines.len();
        let active_count = authority
            .engines
            .iter()
            .filter(|id| !unavailable_engines.contains(id))
            .count();
        let throttle_unit = if engine_allowed
            && !cmd.shutdown
            && active_count > 0
            && active_count < authorized_count
        {
            (cmd.throttle_unit * authorized_count as f64 / active_count as f64).clamp(0.0, 1.0)
        } else {
            cmd.throttle_unit
        };
        for id in &authority.engines {
            let index = usize::from(set.count);
            if index >= MAX_ENGINE_COMMANDS {
                break;
            }
            let engine_available = engine_allowed && !unavailable_engines.contains(id);
            let (trim_pitch_rad, trim_yaw_rad) = if engine_available && !cmd.shutdown {
                self.engine_out_trim.bias_for(*id, unavailable_engines)
            } else {
                (0.0, 0.0)
            };
            set.commands[index] = EngineCommand {
                engine_id: id.value(),
                throttle_unit: if engine_available {
                    throttle_unit
                } else {
                    zero.throttle_unit
                },
                gimbal_pitch_rad: if engine_available {
                    cmd.gimbal_pitch_rad + trim_pitch_rad
                } else {
                    zero.gimbal_pitch_rad
                },
                gimbal_yaw_rad: if engine_available {
                    cmd.gimbal_yaw_rad + trim_yaw_rad
                } else {
                    zero.gimbal_yaw_rad
                },
                ignite: engine_available && cmd.ignite,
                shutdown: if engine_available {
                    cmd.shutdown
                } else {
                    zero.shutdown
                },
            };
            set.count = set.count.saturating_add(1);
        }
        for id in &self.last_commanded_engines {
            if authority.engines.contains(id) {
                continue;
            }
            let index = usize::from(set.count);
            if index >= MAX_ENGINE_COMMANDS {
                break;
            }
            set.commands[index] = EngineCommand {
                engine_id: id.value(),
                ..zero
            };
            set.count = set.count.saturating_add(1);
        }
        self.last_commanded_engines = set
            .commands
            .iter()
            .take(usize::from(set.count))
            .map(|command| EngineId::new(command.engine_id))
            .collect();
        set
    }
}

impl Default for Mixer {
    fn default() -> Self {
        Self::new()
    }
}

impl Job for Mixer {
    fn name(&self) -> &'static str {
        self.name
    }

    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        let status = ctx.bus.latest::<VehicleStatus>()?.map(|(s, _)| s);
        let armed_in_flight = matches!(status, Some(s) if s.armed && s.in_flight);
        let phase_id = status.as_ref().map_or(0, |s| s.phase_id);
        let mut authority = self.authority.lookup(phase_id).clone();
        let mut unavailable_engines = Vec::new();
        if let Some((availability, _)) = ctx.bus.latest::<EffectorAvailability>()? {
            authority
                .effectors
                .retain(|id| !availability.is_unavailable(id.value()));
            unavailable_engines.extend(
                authority
                    .engines
                    .iter()
                    .copied()
                    .filter(|id| availability.is_engine_unavailable(id.value())),
            );
        }
        let safe_state_requested = status
            .as_ref()
            .is_some_and(|status| status.safe_state_requested);
        let actuator_allowed = armed_in_flight && authority.autopilot_allowed;
        let engine_allowed = armed_in_flight && authority.engines_allowed;

        if let Ok(Some((cmd, seq))) = ctx.bus.latest::<ActuatorCommand>()
            && seq.value() > self.last_seen_actuator
        {
            self.last_seen_actuator = seq.value();
            let gated = if actuator_allowed {
                self.gate_actuator_command(cmd, true, &authority)
            } else {
                self.gate_actuator_command(cmd, false, &authority)
            };
            if let Ok(new_seq) = ctx.bus.publish(gated) {
                self.last_seen_actuator = new_seq.value();
            }
            if self.channel_map.has_any_mapping() || self.allocator.is_some() {
                let effector_source = if self.allocator.is_some() && actuator_allowed {
                    cmd
                } else {
                    gated
                };
                let pseudo_control = if self
                    .allocator
                    .as_ref()
                    .is_some_and(ControlAllocator::uses_pseudo_control)
                {
                    ctx.bus
                        .latest::<PseudoControlCommand>()?
                        .map(|(pseudo_control, _)| pseudo_control)
                } else {
                    None
                };
                let effector_set =
                    self.effector_command_set(effector_source, pseudo_control, &authority)?;
                let _ = ctx.bus.publish(effector_set);
            }
        }

        if safe_state_requested {
            let _ = ctx.bus.publish(PropellantSafingCommand {
                time: ctx.clock.now(),
                safed: true,
                isolate_feeds: true,
                close_engine_valves: true,
                pressurization_enabled: false,
                drain_enabled: false,
                dump_enabled: true,
            });
            let shutdown = EngineDemand {
                time: ctx.clock.now(),
                throttle_unit: 0.0,
                gimbal_pitch_rad: 0.0,
                gimbal_yaw_rad: 0.0,
                ignite: false,
                shutdown: true,
            };
            if let Ok(new_seq) = ctx.bus.publish(shutdown) {
                self.last_seen_engine = new_seq.value();
            }
            let engine_set =
                self.engine_command_set(shutdown, &authority, false, &unavailable_engines);
            let _ = ctx.bus.publish(engine_set);
        } else if let Ok(Some((cmd, seq))) = ctx.bus.latest::<EngineDemand>()
            && seq.value() > self.last_seen_engine
        {
            self.last_seen_engine = seq.value();
            let gated = if engine_allowed {
                cmd
            } else {
                EngineDemand {
                    time: cmd.time,
                    throttle_unit: 0.0,
                    gimbal_pitch_rad: 0.0,
                    gimbal_yaw_rad: 0.0,
                    ignite: false,
                    shutdown: true,
                }
            };
            if let Ok(new_seq) = ctx.bus.publish(gated) {
                self.last_seen_engine = new_seq.value();
            }
            let engine_set =
                self.engine_command_set(gated, &authority, engine_allowed, &unavailable_engines);
            let _ = ctx.bus.publish(engine_set);
        }

        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use approx::assert_abs_diff_eq;
    use openbmp_core::{SimTime, StepIndex};

    use super::*;
    use crate::bus::Bus;
    use crate::clock::SimulatedClock;

    fn build_bus() -> Bus {
        let bus = Bus::new();
        bus.register::<ActuatorCommand>().unwrap();
        bus.register::<PseudoControlCommand>().unwrap();
        bus.register::<EffectorAvailability>().unwrap();
        bus.register::<EffectorCommandSet>().unwrap();
        bus.register::<EngineDemand>().unwrap();
        bus.register::<EngineCommandSet>().unwrap();
        bus.register::<PropellantSafingCommand>().unwrap();
        bus.register::<VehicleStatus>().unwrap();
        bus
    }

    fn publish_status(bus: &Bus, armed: bool, in_flight: bool, phase_id: u64) {
        publish_status_with_safe(bus, armed, in_flight, phase_id, false);
    }

    fn publish_status_with_safe(
        bus: &Bus,
        armed: bool,
        in_flight: bool,
        phase_id: u64,
        safe_state_requested: bool,
    ) {
        bus.publish(VehicleStatus {
            armed,
            in_flight,
            phase_id,
            safe_state_requested,
        })
        .unwrap();
    }

    #[test]
    fn phase_disallowed_zero_mixes_actuator_even_when_armed() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let mut allowed = BTreeMap::new();
        allowed.insert(
            42,
            PhaseAuthority {
                effectors: Vec::new(),
                engines: Vec::new(),
                autopilot_allowed: false,
                engines_allowed: true,
            },
        );
        let mut mixer = Mixer::new().with_authority(PhaseAuthorityTable {
            allowed,
            default: PhaseAuthority::default(),
        });

        publish_status(&bus, true, true, 42);
        bus.publish(ActuatorCommand {
            time: SimTime::ZERO,
            elevator_rad: 0.5,
            aileron_rad: 0.3,
            rudder_rad: 0.2,
            body_flap_rad: 0.0,
            saturated: false,
        })
        .unwrap();

        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (latest, _) = bus.latest::<ActuatorCommand>().unwrap().unwrap();
        assert_eq!(latest.elevator_rad, 0.0);
        assert_eq!(latest.aileron_rad, 0.0);
        assert_eq!(latest.rudder_rad, 0.0);
    }

    #[test]
    fn phase_allowed_passes_through_when_armed() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let mut allowed = BTreeMap::new();
        allowed.insert(
            7,
            PhaseAuthority {
                effectors: Vec::new(),
                engines: Vec::new(),
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        let mut mixer = Mixer::new().with_authority(PhaseAuthorityTable {
            allowed,
            default: PhaseAuthority::default(),
        });

        publish_status(&bus, true, true, 7);
        bus.publish(ActuatorCommand {
            time: SimTime::ZERO,
            elevator_rad: 0.4,
            aileron_rad: 0.0,
            rudder_rad: 0.0,
            body_flap_rad: 0.0,
            saturated: false,
        })
        .unwrap();

        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (latest, _) = bus.latest::<ActuatorCommand>().unwrap().unwrap();
        assert!((latest.elevator_rad - 0.4).abs() < 1e-12);
    }

    #[test]
    fn phase_disallowed_shutdowns_stale_engine_commands() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let engine = EngineId::from_path("vehicle.assembly.engines.main");
        let mut allowed = BTreeMap::new();
        allowed.insert(
            1,
            PhaseAuthority {
                effectors: Vec::new(),
                engines: vec![engine],
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        allowed.insert(
            2,
            PhaseAuthority {
                effectors: Vec::new(),
                engines: Vec::new(),
                autopilot_allowed: true,
                engines_allowed: false,
            },
        );
        let mut mixer = Mixer::new().with_authority(PhaseAuthorityTable {
            allowed,
            default: PhaseAuthority::default(),
        });

        publish_status(&bus, true, true, 1);
        bus.publish(EngineDemand {
            time: SimTime::ZERO,
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.1,
            gimbal_yaw_rad: -0.1,
            ignite: true,
            shutdown: false,
        })
        .unwrap();
        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();
        let (burning, _) = bus.latest::<EngineCommandSet>().unwrap().unwrap();
        assert_eq!(burning.count, 1);
        assert_eq!(burning.commands[0].engine_id, engine.value());
        assert_eq!(burning.commands[0].throttle_unit, 1.0);
        assert!(burning.commands[0].ignite);
        assert!(!burning.commands[0].shutdown);

        publish_status(&bus, true, true, 2);
        bus.publish(EngineDemand {
            time: SimTime::from_seconds(0.02),
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.1,
            gimbal_yaw_rad: -0.1,
            ignite: true,
            shutdown: false,
        })
        .unwrap();
        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (gated, _) = bus.latest::<EngineDemand>().unwrap().unwrap();
        assert_eq!(gated.throttle_unit, 0.0);
        assert!(!gated.ignite);
        assert!(gated.shutdown);

        let (shutdown, _) = bus.latest::<EngineCommandSet>().unwrap().unwrap();
        assert_eq!(shutdown.count, 1);
        assert_eq!(shutdown.commands[0].engine_id, engine.value());
        assert_eq!(shutdown.commands[0].throttle_unit, 0.0);
        assert!(!shutdown.commands[0].ignite);
        assert!(shutdown.commands[0].shutdown);
    }

    #[test]
    fn safe_state_request_publishes_engine_shutdown_commands_without_fresh_demand() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let left_engine = EngineId::from_path("vehicle.assembly.engines.left");
        let right_engine = EngineId::from_path("vehicle.assembly.engines.right");
        let mut allowed = BTreeMap::new();
        allowed.insert(
            7,
            PhaseAuthority {
                effectors: Vec::new(),
                engines: vec![left_engine, right_engine],
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        let mut mixer = Mixer::new().with_authority(PhaseAuthorityTable {
            allowed,
            default: PhaseAuthority::default(),
        });

        publish_status(&bus, true, true, 7);
        bus.publish(EngineDemand {
            time: SimTime::ZERO,
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.1,
            gimbal_yaw_rad: -0.1,
            ignite: true,
            shutdown: false,
        })
        .unwrap();
        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (burning, _) = bus.latest::<EngineCommandSet>().unwrap().unwrap();
        assert_eq!(burning.count, 2);
        assert_eq!(burning.commands[0].throttle_unit, 1.0);
        assert!(burning.commands[0].ignite);
        assert!(!burning.commands[0].shutdown);

        clock.set(SimTime::from_seconds(0.25), StepIndex::new(25));
        publish_status_with_safe(&bus, true, true, 7, true);
        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (gated, _) = bus.latest::<EngineDemand>().unwrap().unwrap();
        assert_eq!(gated.time, SimTime::from_seconds(0.25));
        assert_eq!(gated.throttle_unit, 0.0);
        assert_eq!(gated.gimbal_pitch_rad, 0.0);
        assert_eq!(gated.gimbal_yaw_rad, 0.0);
        assert!(!gated.ignite);
        assert!(gated.shutdown);

        let (safing, _) = bus.latest::<PropellantSafingCommand>().unwrap().unwrap();
        assert_eq!(safing.time, SimTime::from_seconds(0.25));
        assert!(safing.safed);
        assert!(safing.isolate_feeds);
        assert!(safing.close_engine_valves);
        assert!(!safing.pressurization_enabled);
        assert!(!safing.drain_enabled);
        assert!(safing.dump_enabled);

        let (shutdown, _) = bus.latest::<EngineCommandSet>().unwrap().unwrap();
        assert_eq!(shutdown.count, 2);
        assert_eq!(shutdown.time, SimTime::from_seconds(0.25));
        let mut by_id = BTreeMap::new();
        for command in shutdown.commands.iter().take(usize::from(shutdown.count)) {
            by_id.insert(command.engine_id, *command);
        }
        for engine in [left_engine, right_engine] {
            let command = by_id[&engine.value()];
            assert_eq!(command.throttle_unit, 0.0);
            assert_eq!(command.gimbal_pitch_rad, 0.0);
            assert_eq!(command.gimbal_yaw_rad, 0.0);
            assert!(!command.ignite);
            assert!(command.shutdown);
        }
    }

    #[test]
    fn engine_availability_retrims_throttle_and_shutdowns_failed_engine() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let left_engine = EngineId::from_path("vehicle.assembly.engines.left");
        let center_engine = EngineId::from_path("vehicle.assembly.engines.center");
        let right_engine = EngineId::from_path("vehicle.assembly.engines.right");
        let mut allowed = BTreeMap::new();
        allowed.insert(
            7,
            PhaseAuthority {
                effectors: Vec::new(),
                engines: vec![left_engine, center_engine, right_engine],
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        let mut mixer = Mixer::new().with_authority(PhaseAuthorityTable {
            allowed,
            default: PhaseAuthority::default(),
        });

        let mut unavailable_engines = [0; MAX_ENGINE_COMMANDS];
        unavailable_engines[0] = center_engine.value();
        bus.publish(EffectorAvailability {
            time: SimTime::ZERO,
            unavailable_engine_count: 1,
            unavailable_engines,
            ..EffectorAvailability::default()
        })
        .unwrap();
        publish_status(&bus, true, true, 7);
        bus.publish(EngineDemand {
            time: SimTime::ZERO,
            throttle_unit: 0.6,
            gimbal_pitch_rad: 0.1,
            gimbal_yaw_rad: -0.1,
            ignite: true,
            shutdown: false,
        })
        .unwrap();

        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (commands, _) = bus.latest::<EngineCommandSet>().unwrap().unwrap();
        assert_eq!(commands.count, 3);
        let mut by_id = BTreeMap::new();
        for command in commands.commands.iter().take(usize::from(commands.count)) {
            by_id.insert(command.engine_id, *command);
        }

        let failed = by_id[&center_engine.value()];
        assert_eq!(failed.throttle_unit, 0.0);
        assert_eq!(failed.gimbal_pitch_rad, 0.0);
        assert_eq!(failed.gimbal_yaw_rad, 0.0);
        assert!(!failed.ignite);
        assert!(failed.shutdown);

        for engine in [left_engine, right_engine] {
            let command = by_id[&engine.value()];
            assert_abs_diff_eq!(command.throttle_unit, 0.9, epsilon = 1.0e-12);
            assert_abs_diff_eq!(command.gimbal_pitch_rad, 0.1, epsilon = 1.0e-12);
            assert_abs_diff_eq!(command.gimbal_yaw_rad, -0.1, epsilon = 1.0e-12);
            assert!(command.ignite);
            assert!(!command.shutdown);
        }

        bus.publish(EngineDemand {
            time: SimTime::from_seconds(0.1),
            throttle_unit: 0.8,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: true,
            shutdown: false,
        })
        .unwrap();
        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (commands, _) = bus.latest::<EngineCommandSet>().unwrap().unwrap();
        let mut by_id = BTreeMap::new();
        for command in commands.commands.iter().take(usize::from(commands.count)) {
            by_id.insert(command.engine_id, *command);
        }
        assert_eq!(by_id[&center_engine.value()].throttle_unit, 0.0);
        for engine in [left_engine, right_engine] {
            assert_abs_diff_eq!(by_id[&engine.value()].throttle_unit, 1.0, epsilon = 1.0e-12);
        }
    }

    #[test]
    fn engine_out_trim_table_adds_survivor_gimbal_biases_after_retrim() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let left_engine = EngineId::from_path("vehicle.assembly.engines.left");
        let center_engine = EngineId::from_path("vehicle.assembly.engines.center");
        let right_engine = EngineId::from_path("vehicle.assembly.engines.right");
        let mut allowed = BTreeMap::new();
        allowed.insert(
            7,
            PhaseAuthority {
                effectors: Vec::new(),
                engines: vec![left_engine, center_engine, right_engine],
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        let mut trim_rows = BTreeMap::new();
        trim_rows.insert(
            center_engine,
            vec![
                EngineOutTrimCommand {
                    engine: left_engine,
                    gimbal_pitch_bias_rad: 0.005,
                    gimbal_yaw_bias_rad: -0.010,
                },
                EngineOutTrimCommand {
                    engine: right_engine,
                    gimbal_pitch_bias_rad: -0.010,
                    gimbal_yaw_bias_rad: 0.015,
                },
            ],
        );
        let mut mixer = Mixer::new()
            .with_authority(PhaseAuthorityTable {
                allowed,
                default: PhaseAuthority::default(),
            })
            .with_engine_out_trim(EngineOutTrimTable {
                by_failed_engine: trim_rows,
            });

        let mut unavailable_engines = [0; MAX_ENGINE_COMMANDS];
        unavailable_engines[0] = center_engine.value();
        bus.publish(EffectorAvailability {
            time: SimTime::ZERO,
            unavailable_engine_count: 1,
            unavailable_engines,
            ..EffectorAvailability::default()
        })
        .unwrap();
        publish_status(&bus, true, true, 7);
        bus.publish(EngineDemand {
            time: SimTime::ZERO,
            throttle_unit: 0.6,
            gimbal_pitch_rad: 0.020,
            gimbal_yaw_rad: -0.030,
            ignite: true,
            shutdown: false,
        })
        .unwrap();

        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (commands, _) = bus.latest::<EngineCommandSet>().unwrap().unwrap();
        let mut by_id = BTreeMap::new();
        for command in commands.commands.iter().take(usize::from(commands.count)) {
            by_id.insert(command.engine_id, *command);
        }

        assert_eq!(by_id[&center_engine.value()].throttle_unit, 0.0);
        assert_eq!(by_id[&center_engine.value()].gimbal_pitch_rad, 0.0);
        assert_eq!(by_id[&center_engine.value()].gimbal_yaw_rad, 0.0);
        assert!(by_id[&center_engine.value()].shutdown);

        let left = by_id[&left_engine.value()];
        assert_abs_diff_eq!(left.throttle_unit, 0.9, epsilon = 1.0e-12);
        assert_abs_diff_eq!(left.gimbal_pitch_rad, 0.025, epsilon = 1.0e-12);
        assert_abs_diff_eq!(left.gimbal_yaw_rad, -0.040, epsilon = 1.0e-12);

        let right = by_id[&right_engine.value()];
        assert_abs_diff_eq!(right.throttle_unit, 0.9, epsilon = 1.0e-12);
        assert_abs_diff_eq!(right.gimbal_pitch_rad, 0.010, epsilon = 1.0e-12);
        assert_abs_diff_eq!(right.gimbal_yaw_rad, -0.015, epsilon = 1.0e-12);
    }

    #[test]
    fn channel_map_publishes_effector_id_commands() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let elevator = EffectorId::from_path("vehicle.assembly.effectors.delta_e");
        let aileron = EffectorId::from_path("vehicle.assembly.effectors.delta_a");
        let mut allowed = BTreeMap::new();
        allowed.insert(
            9,
            PhaseAuthority {
                effectors: vec![elevator],
                engines: Vec::new(),
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        let mut mixer = Mixer::new()
            .with_authority(PhaseAuthorityTable {
                allowed,
                default: PhaseAuthority::default(),
            })
            .with_actuator_channel_map(ActuatorChannelMap {
                elevator: Some(elevator),
                aileron: Some(aileron),
                rudder: None,
                body_flap: None,
            });

        publish_status(&bus, true, true, 9);
        bus.publish(ActuatorCommand {
            time: SimTime::ZERO,
            elevator_rad: 0.4,
            aileron_rad: 0.2,
            rudder_rad: 0.0,
            body_flap_rad: 0.0,
            saturated: false,
        })
        .unwrap();

        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (semantic, _) = bus.latest::<ActuatorCommand>().unwrap().unwrap();
        assert_eq!(semantic.elevator_rad, 0.4);
        assert_eq!(semantic.aileron_rad, 0.0);

        let (mapped, _) = bus.latest::<EffectorCommandSet>().unwrap().unwrap();
        assert_eq!(mapped.count, 2);
        assert_eq!(mapped.commands[0].effector_id, aileron.value());
        assert_eq!(mapped.commands[0].command, 0.0);
        assert_eq!(mapped.commands[1].effector_id, elevator.value());
        assert_eq!(mapped.commands[1].command, 0.4);
    }

    #[test]
    fn allocator_uses_raw_axis_demand_and_pre_gates_capacity() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let roll_a = EffectorId::from_path("vehicle.assembly.effectors.roll-a");
        let roll_b = EffectorId::from_path("vehicle.assembly.effectors.roll-b");
        let pitch = EffectorId::from_path("vehicle.assembly.effectors.pitch");
        let yaw = EffectorId::from_path("vehicle.assembly.effectors.yaw");
        let allocator = crate::allocation::PrioritisedRedistributedAllocator::new(
            [
                crate::allocation::BodyAxis::Roll,
                crate::allocation::BodyAxis::Pitch,
                crate::allocation::BodyAxis::Yaw,
            ],
            vec![
                crate::allocation::EffectorAxisAssignment {
                    effector_id: roll_a,
                    axis: crate::allocation::BodyAxis::Roll,
                    max_abs: 0.2,
                },
                crate::allocation::EffectorAxisAssignment {
                    effector_id: roll_b,
                    axis: crate::allocation::BodyAxis::Roll,
                    max_abs: 0.2,
                },
                crate::allocation::EffectorAxisAssignment {
                    effector_id: pitch,
                    axis: crate::allocation::BodyAxis::Pitch,
                    max_abs: 0.35,
                },
                crate::allocation::EffectorAxisAssignment {
                    effector_id: yaw,
                    axis: crate::allocation::BodyAxis::Yaw,
                    max_abs: 0.35,
                },
            ],
        )
        .expect("allocator builds");
        let mut allowed = BTreeMap::new();
        allowed.insert(
            11,
            PhaseAuthority {
                effectors: vec![roll_b, pitch, yaw],
                engines: Vec::new(),
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        let mut mixer = Mixer::new()
            .with_authority(PhaseAuthorityTable {
                allowed,
                default: PhaseAuthority::default(),
            })
            .with_actuator_channel_map(ActuatorChannelMap {
                aileron: Some(roll_a),
                elevator: Some(pitch),
                rudder: Some(yaw),
                body_flap: None,
            })
            .with_allocator(allocator);

        publish_status(&bus, true, true, 11);
        bus.publish(ActuatorCommand {
            time: SimTime::ZERO,
            elevator_rad: 0.0,
            aileron_rad: 0.3,
            rudder_rad: 0.0,
            body_flap_rad: 0.0,
            saturated: false,
        })
        .unwrap();

        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (semantic, _) = bus.latest::<ActuatorCommand>().unwrap().unwrap();
        assert_eq!(
            semantic.aileron_rad, 0.0,
            "legacy semantic publish remains channel-map gated"
        );

        let (mapped, _) = bus.latest::<EffectorCommandSet>().unwrap().unwrap();
        let mut by_id = BTreeMap::new();
        for command in mapped.commands.iter().take(usize::from(mapped.count)) {
            by_id.insert(command.effector_id, command.command);
        }
        assert_eq!(by_id[&roll_a.value()].to_bits(), 0.0_f64.to_bits());
        assert_eq!(by_id[&roll_b.value()].to_bits(), 0.2_f64.to_bits());
        assert!(
            mapped.saturated,
            "allowed roll capacity is 0.2, so a 0.3 demand must saturate"
        );
    }

    #[test]
    fn dense_wls_allocator_publishes_effector_command_set() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let roll = EffectorId::from_path("vehicle.assembly.effectors.roll");
        let pitch = EffectorId::from_path("vehicle.assembly.effectors.pitch");
        let yaw = EffectorId::from_path("vehicle.assembly.effectors.yaw");
        let matrix = crate::allocation::EffectivenessMatrix::new(
            3,
            vec![roll, pitch, yaw],
            vec![
                1.0, 0.0, 0.0, //
                0.0, 1.0, 0.0, //
                0.0, 0.0, 1.0,
            ],
        )
        .expect("dense matrix builds");
        let wls = crate::allocation::WlsActiveSetAllocator::new(
            matrix,
            vec![1.0, 1.0, 1.0],
            vec![1.0, 1.0, 1.0],
            1.0e9,
            vec![0.0, 0.0, 0.0],
        )
        .expect("dense wls builds");
        let allocator = crate::allocation::DenseWlsControlAllocator::new(
            wls,
            crate::allocation::EffectorBounds {
                u_min_pos: vec![-1.0, -1.0, -1.0],
                u_max_pos: vec![1.0, 1.0, 1.0],
                rate_limit: vec![10.0, 10.0, 10.0],
            },
            vec![0.0, 0.0, 0.0],
            0.1,
        )
        .expect("dense control allocator builds");
        let mut allowed = BTreeMap::new();
        allowed.insert(
            7,
            PhaseAuthority {
                effectors: vec![roll, pitch, yaw],
                engines: Vec::new(),
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        let mut mixer = Mixer::new()
            .with_authority(PhaseAuthorityTable {
                allowed,
                default: PhaseAuthority::default(),
            })
            .with_allocator(allocator);

        publish_status(&bus, true, true, 7);
        bus.publish(ActuatorCommand {
            time: SimTime::ZERO,
            elevator_rad: 0.2,
            aileron_rad: 0.4,
            rudder_rad: -0.1,
            body_flap_rad: 0.0,
            saturated: false,
        })
        .unwrap();

        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (mapped, _) = bus.latest::<EffectorCommandSet>().unwrap().unwrap();
        let mut by_id = BTreeMap::new();
        for command in mapped.commands.iter().take(usize::from(mapped.count)) {
            by_id.insert(command.effector_id, command.command);
        }
        assert_abs_diff_eq!(by_id[&roll.value()], 0.4, epsilon = 1.0e-8);
        assert_abs_diff_eq!(by_id[&pitch.value()], 0.2, epsilon = 1.0e-8);
        assert_abs_diff_eq!(by_id[&yaw.value()], -0.1, epsilon = 1.0e-8);
        assert!(!mapped.saturated);
    }

    #[test]
    fn effector_availability_removes_dense_column_before_allocation() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let roll_a = EffectorId::from_path("vehicle.assembly.effectors.roll-a");
        let roll_b = EffectorId::from_path("vehicle.assembly.effectors.roll-b");
        let pitch = EffectorId::from_path("vehicle.assembly.effectors.pitch");
        let yaw = EffectorId::from_path("vehicle.assembly.effectors.yaw");
        let matrix = crate::allocation::EffectivenessMatrix::new(
            3,
            vec![roll_a, roll_b, pitch, yaw],
            vec![
                1.0, 1.0, 0.0, 0.0, //
                0.0, 0.0, 1.0, 0.0, //
                0.0, 0.0, 0.0, 1.0,
            ],
        )
        .expect("dense matrix builds");
        let wls = crate::allocation::WlsActiveSetAllocator::new(
            matrix,
            vec![1.0, 1.0, 1.0],
            vec![1.0, 1.0, 1.0, 1.0],
            1.0e9,
            vec![0.0, 0.0, 0.0, 0.0],
        )
        .expect("dense wls builds");
        let allocator = crate::allocation::DenseWlsControlAllocator::new(
            wls,
            crate::allocation::EffectorBounds {
                u_min_pos: vec![-1.0, -1.0, -1.0, -1.0],
                u_max_pos: vec![1.0, 1.0, 1.0, 1.0],
                rate_limit: vec![10.0, 10.0, 10.0, 10.0],
            },
            vec![0.0, 0.0, 0.0, 0.0],
            0.1,
        )
        .expect("dense control allocator builds");
        let mut allowed = BTreeMap::new();
        allowed.insert(
            7,
            PhaseAuthority {
                effectors: vec![roll_a, roll_b, pitch, yaw],
                engines: Vec::new(),
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        let mut mixer = Mixer::new()
            .with_authority(PhaseAuthorityTable {
                allowed,
                default: PhaseAuthority::default(),
            })
            .with_allocator(allocator);

        let mut unavailable_effectors = [0; MAX_EFFECTOR_COMMANDS];
        unavailable_effectors[0] = roll_b.value();
        bus.publish(EffectorAvailability {
            time: SimTime::ZERO,
            unavailable_count: 1,
            unavailable_effectors,
            ..EffectorAvailability::default()
        })
        .unwrap();
        publish_status(&bus, true, true, 7);
        bus.publish(ActuatorCommand {
            time: SimTime::ZERO,
            elevator_rad: 0.0,
            aileron_rad: 0.6,
            rudder_rad: 0.0,
            body_flap_rad: 0.0,
            saturated: false,
        })
        .unwrap();

        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (mapped, _) = bus.latest::<EffectorCommandSet>().unwrap().unwrap();
        let mut by_id = BTreeMap::new();
        for command in mapped.commands.iter().take(usize::from(mapped.count)) {
            by_id.insert(command.effector_id, command.command);
        }
        assert_abs_diff_eq!(by_id[&roll_a.value()], 0.6, epsilon = 1.0e-8);
        assert_eq!(by_id[&roll_b.value()].to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn effector_availability_reports_saturation_when_remaining_dense_authority_is_insufficient() {
        let bus = build_bus();
        let clock = SimulatedClock::new();
        let roll_a = EffectorId::from_path("vehicle.assembly.effectors.roll-a");
        let roll_b = EffectorId::from_path("vehicle.assembly.effectors.roll-b");
        let pitch = EffectorId::from_path("vehicle.assembly.effectors.pitch");
        let yaw = EffectorId::from_path("vehicle.assembly.effectors.yaw");
        let matrix = crate::allocation::EffectivenessMatrix::new(
            3,
            vec![roll_a, roll_b, pitch, yaw],
            vec![
                1.0, 1.0, 0.0, 0.0, //
                0.0, 0.0, 1.0, 0.0, //
                0.0, 0.0, 0.0, 1.0,
            ],
        )
        .expect("dense matrix builds");
        let wls = crate::allocation::WlsActiveSetAllocator::new(
            matrix,
            vec![1.0, 1.0, 1.0],
            vec![1.0, 1.0, 1.0, 1.0],
            1.0e9,
            vec![0.0, 0.0, 0.0, 0.0],
        )
        .expect("dense wls builds");
        let allocator = crate::allocation::DenseWlsControlAllocator::new(
            wls,
            crate::allocation::EffectorBounds {
                u_min_pos: vec![-0.5, -1.0, -1.0, -1.0],
                u_max_pos: vec![0.5, 1.0, 1.0, 1.0],
                rate_limit: vec![10.0, 10.0, 10.0, 10.0],
            },
            vec![0.0, 0.0, 0.0, 0.0],
            0.1,
        )
        .expect("dense control allocator builds");
        let mut allowed = BTreeMap::new();
        allowed.insert(
            7,
            PhaseAuthority {
                effectors: vec![roll_a, roll_b, pitch, yaw],
                engines: Vec::new(),
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        let mut mixer = Mixer::new()
            .with_authority(PhaseAuthorityTable {
                allowed,
                default: PhaseAuthority::default(),
            })
            .with_allocator(allocator);

        let mut unavailable_effectors = [0; MAX_EFFECTOR_COMMANDS];
        unavailable_effectors[0] = roll_b.value();
        bus.publish(EffectorAvailability {
            time: SimTime::ZERO,
            unavailable_count: 1,
            unavailable_effectors,
            ..EffectorAvailability::default()
        })
        .unwrap();
        publish_status(&bus, true, true, 7);
        bus.publish(ActuatorCommand {
            time: SimTime::ZERO,
            elevator_rad: 0.0,
            aileron_rad: 0.8,
            rudder_rad: 0.0,
            body_flap_rad: 0.0,
            saturated: false,
        })
        .unwrap();

        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (mapped, _) = bus.latest::<EffectorCommandSet>().unwrap().unwrap();
        let mut by_id = BTreeMap::new();
        for command in mapped.commands.iter().take(usize::from(mapped.count)) {
            by_id.insert(command.effector_id, command.command);
        }
        assert_abs_diff_eq!(by_id[&roll_a.value()], 0.5, epsilon = 1.0e-8);
        assert_eq!(by_id[&roll_b.value()].to_bits(), 0.0_f64.to_bits());
        assert!(
            mapped.saturated,
            "remaining roll authority cannot attain the 0.8 demand after roll_b is unavailable"
        );
    }

    #[cfg(feature = "indi")]
    #[test]
    fn dense_inca_allocator_consumes_pseudo_control_topic() {
        use nalgebra::Vector3;

        let bus = build_bus();
        let clock = SimulatedClock::new();
        let roll = EffectorId::from_path("vehicle.assembly.effectors.roll");
        let pitch = EffectorId::from_path("vehicle.assembly.effectors.pitch");
        let yaw = EffectorId::from_path("vehicle.assembly.effectors.yaw");
        let matrix = crate::allocation::EffectivenessMatrix::new(
            3,
            vec![roll, pitch, yaw],
            vec![
                1.0, 0.0, 0.0, //
                0.0, 1.0, 0.0, //
                0.0, 0.0, 1.0,
            ],
        )
        .expect("dense matrix builds");
        let wls = crate::allocation::WlsActiveSetAllocator::new(
            matrix,
            vec![1.0, 1.0, 1.0],
            vec![1.0, 1.0, 1.0],
            1.0e9,
            vec![0.0, 0.0, 0.0],
        )
        .expect("dense wls builds");
        let allocator = crate::allocation::DenseIncaControlAllocator::new(
            wls,
            crate::allocation::EffectorBounds {
                u_min_pos: vec![-1.0, -1.0, -1.0],
                u_max_pos: vec![1.0, 1.0, 1.0],
                rate_limit: vec![10.0, 10.0, 10.0],
            },
            vec![0.0, 0.0, 0.0],
            0.1,
        )
        .expect("dense INCA allocator builds");
        let mut allowed = BTreeMap::new();
        allowed.insert(
            7,
            PhaseAuthority {
                effectors: vec![roll, pitch, yaw],
                engines: Vec::new(),
                autopilot_allowed: true,
                engines_allowed: true,
            },
        );
        let mut mixer = Mixer::new()
            .with_authority(PhaseAuthorityTable {
                allowed,
                default: PhaseAuthority::default(),
            })
            .with_allocator(allocator);

        publish_status(&bus, true, true, 7);
        bus.publish(PseudoControlCommand {
            time: SimTime::ZERO,
            omega_dot_des_rad_s2: Vector3::new(0.4, 0.2, -0.1),
            omega_dot_meas_rad_s2: Vector3::zeros(),
        })
        .unwrap();
        bus.publish(ActuatorCommand {
            time: SimTime::ZERO,
            elevator_rad: 0.0,
            aileron_rad: 0.0,
            rudder_rad: 0.0,
            body_flap_rad: 0.0,
            saturated: false,
        })
        .unwrap();

        mixer
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (mapped, _) = bus.latest::<EffectorCommandSet>().unwrap().unwrap();
        let mut by_id = BTreeMap::new();
        for command in mapped.commands.iter().take(usize::from(mapped.count)) {
            by_id.insert(command.effector_id, command.command);
        }
        assert_abs_diff_eq!(by_id[&roll.value()], 0.4, epsilon = 1.0e-8);
        assert_abs_diff_eq!(by_id[&pitch.value()], 0.2, epsilon = 1.0e-8);
        assert_abs_diff_eq!(by_id[&yaw.value()], -0.1, epsilon = 1.0e-8);
        assert!(!mapped.saturated);
    }
}
