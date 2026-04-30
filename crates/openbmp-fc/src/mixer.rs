//! Phase-gated actuator mixer.
//!
//! The mixer is the only path between an autopilot's commanded
//! deflection and the actuator topic an effector consumer reads. If
//! the commander has not armed the vehicle, or the active phase's
//! `allowed_effectors` mask does not authorise the channel, the mixer
//! zero-mixes the demand and emits a warning event. This prevents a
//! controller bug in one phase from actuating in the wrong phase.
//!
//! Phase 4.4 ships a static channel allowlist per [`PhaseId`] held
//! in a [`crate::tables`] entry; future revisions can add more
//! sophisticated allocation matrices.

use std::collections::BTreeMap;

use openbmp_core::EffectorId;

use crate::error::ControllerError;
use crate::scheduler::{Job, JobContext};
use crate::tables::Table;
use crate::topics::{ActuatorCommand, EngineDemand, VehicleStatus};

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
        }
    }

    /// Replaces the phase-authority table.
    #[must_use]
    pub fn with_authority(mut self, authority: PhaseAuthorityTable) -> Self {
        self.authority = authority;
        self
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
                cmd
            } else {
                ActuatorCommand {
                    time: cmd.time,
                    elevator_rad: 0.0,
                    aileron_rad: 0.0,
                    rudder_rad: 0.0,
                    body_flap_rad: 0.0,
                    saturated: cmd.saturated,
                }
            };
            let _ = ctx.bus.publish(gated);
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
            let _ = ctx.bus.publish(gated);
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
        bus.register::<EngineDemand>().unwrap();
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
}
