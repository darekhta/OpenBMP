//! Phase-gated actuator mixer.
//!
//! The mixer is the only path between an autopilot's commanded
//! deflection and the actuator topic an effector consumer reads. If
//! the commander has not armed the vehicle, or the active phase's
//! `allowed_effectors` mask does not authorise the channel, the mixer
//! zero-mixes the demand and emits a warning event. This prevents a
//! controller bug in one phase from actuating in the wrong phase.
//!
//! Phase 4.4 ships a static channel allowlist per `PhaseId` held
//! in a [`crate::tables`] entry; future revisions can add more
//! sophisticated allocation matrices.

use std::collections::BTreeMap;

use openbmp_core::EffectorId;

use crate::error::ControllerError;
use crate::scheduler::{Job, JobContext};
use crate::tables::Table;
use crate::topics::{
    ActuatorCommand, EffectorCommand, EffectorCommandSet, EngineCommand, EngineCommandSet,
    EngineDemand, MAX_EFFECTOR_COMMANDS, MAX_ENGINE_COMMANDS, VehicleStatus,
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

/// Mixer job — runs after the autopilot, applies phase-gating, and
/// republishes gated actuator + engine commands on the same topics.
///
/// The phase-authority mask is consulted on every tick: a phase that
/// declares `autopilot_allowed = false` zero-mixes the actuator
/// command even when the vehicle is armed and in flight, and an
/// equivalent rule applies to engine demand.
#[derive(Debug)]
pub struct Mixer {
    name: &'static str,
    last_seen_actuator: u64,
    last_seen_engine: u64,
    authority: PhaseAuthorityTable,
    channel_map: ActuatorChannelMap,
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
            authority: PhaseAuthorityTable::default(),
            channel_map: ActuatorChannelMap::default(),
        }
    }

    /// Replaces the phase-authority table.
    #[must_use]
    pub fn with_authority(mut self, authority: PhaseAuthorityTable) -> Self {
        self.authority = authority;
        self
    }

    /// Replaces the semantic-channel to effector-id map.
    #[must_use]
    pub fn with_actuator_channel_map(mut self, channel_map: ActuatorChannelMap) -> Self {
        self.channel_map = channel_map;
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

    fn effector_command_set(&self, cmd: ActuatorCommand) -> EffectorCommandSet {
        let mut set = EffectorCommandSet {
            time: cmd.time,
            saturated: cmd.saturated,
            ..EffectorCommandSet::default()
        };
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
                break;
            }
            set.commands[index] = EffectorCommand {
                effector_id: id.value(),
                command,
                saturated: cmd.saturated,
            };
            set.count = set.count.saturating_add(1);
        }
        set
    }

    fn engine_command_set(cmd: EngineDemand, authority: &PhaseAuthority) -> EngineCommandSet {
        let mut set = EngineCommandSet {
            time: cmd.time,
            ..EngineCommandSet::default()
        };
        for id in &authority.engines {
            let index = usize::from(set.count);
            if index >= MAX_ENGINE_COMMANDS {
                break;
            }
            set.commands[index] = EngineCommand {
                engine_id: id.value(),
                throttle_unit: cmd.throttle_unit,
                gimbal_pitch_rad: cmd.gimbal_pitch_rad,
                gimbal_yaw_rad: cmd.gimbal_yaw_rad,
                ignite: cmd.ignite,
                shutdown: cmd.shutdown,
            };
            set.count = set.count.saturating_add(1);
        }
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
        let authority = self.authority.lookup(phase_id);
        let actuator_allowed = armed_in_flight && authority.autopilot_allowed;
        let engine_allowed = armed_in_flight && authority.engines_allowed;

        if let Ok(Some((cmd, seq))) = ctx.bus.latest::<ActuatorCommand>()
            && seq.value() > self.last_seen_actuator
        {
            self.last_seen_actuator = seq.value();
            let gated = if actuator_allowed {
                self.gate_actuator_command(cmd, true, authority)
            } else {
                self.gate_actuator_command(cmd, false, authority)
            };
            if let Ok(new_seq) = ctx.bus.publish(gated) {
                self.last_seen_actuator = new_seq.value();
            }
            if self.channel_map.has_any_mapping() {
                let _ = ctx.bus.publish(self.effector_command_set(gated));
            }
        }

        if let Ok(Some((cmd, seq))) = ctx.bus.latest::<EngineDemand>()
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
                    shutdown: cmd.shutdown,
                }
            };
            if let Ok(new_seq) = ctx.bus.publish(gated) {
                self.last_seen_engine = new_seq.value();
            }
            if !authority.engines.is_empty() {
                let _ = ctx.bus.publish(Self::engine_command_set(gated, authority));
            }
        }

        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use openbmp_core::SimTime;

    use super::*;
    use crate::bus::Bus;
    use crate::clock::SimulatedClock;

    fn build_bus() -> Bus {
        let bus = Bus::new();
        bus.register::<ActuatorCommand>().unwrap();
        bus.register::<EffectorCommandSet>().unwrap();
        bus.register::<EngineDemand>().unwrap();
        bus.register::<EngineCommandSet>().unwrap();
        bus.register::<VehicleStatus>().unwrap();
        bus
    }

    fn publish_status(bus: &Bus, armed: bool, in_flight: bool, phase_id: u64) {
        bus.publish(VehicleStatus {
            armed,
            in_flight,
            phase_id,
            safe_state_requested: false,
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
}
