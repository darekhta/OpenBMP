//! Academic guidance laws.
//!
//! Provides:
//! - Attitude tracking against a scripted reference state.
//! - Scenario-waypoint navigation in inertial space.
//! - Simple terminal-state regulation (constant attitude / position).
//!
//! The safety-boundaries `Reject` list applies categorically — no
//! proportional navigation, no terminal-homing, no targeting, no
//! real-world-location guidance.

use nalgebra::Vector3;
use openbmp_physics::profile::{AscentReferenceGenerator, AscentState};

use crate::error::{ControllerError, GuidanceError};
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::topics::{PositionEstimate, ReferenceState, VehicleStatus};

/// Guidance configuration.
#[derive(Clone, Debug)]
pub struct GuidanceParams {
    /// Acceptance radius (m) for considering a waypoint reached.
    pub waypoint_acceptance_radius_m: f64,
    /// Default reference quaternion (x, y, z, w) when no waypoint
    /// guidance is active.
    pub default_reference_q_xyzw: [f64; 4],
}

impl Default for GuidanceParams {
    fn default() -> Self {
        Self {
            waypoint_acceptance_radius_m: 5.0,
            default_reference_q_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

impl ParamSection for GuidanceParams {
    const NAME: &'static str = "guidance";
}

/// One inertial-space waypoint.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct InertialWaypoint {
    /// Target ECI position (m).
    pub position_eci_m: Vector3<f64>,
    /// Target ECI velocity (m/s) — `[0;3]` for position-hold semantics.
    pub velocity_eci_m_s: Vector3<f64>,
    /// Reference quaternion at this waypoint (x, y, z, w).
    pub reference_q_xyzw: [f64; 4],
}

/// Scripted-waypoint sequence consumed by [`WaypointGuidance`].
#[derive(Clone, Debug, Default)]
pub struct WaypointSequence {
    /// Ordered list of waypoints. Guidance advances when each is
    /// inside [`GuidanceParams::waypoint_acceptance_radius_m`].
    pub waypoints: Vec<InertialWaypoint>,
}

/// Waypoint-tracking guidance job.
#[derive(Debug)]
pub struct WaypointGuidance {
    name: &'static str,
    params: GuidanceParams,
    sequence: WaypointSequence,
    cursor: usize,
}

impl WaypointGuidance {
    /// Constructs the guidance job around the given waypoint sequence.
    #[must_use]
    pub fn new(sequence: WaypointSequence, params: GuidanceParams) -> Self {
        Self {
            name: "guidance.waypoint",
            params,
            sequence,
            cursor: 0,
        }
    }

    /// Returns the index of the active waypoint.
    #[must_use]
    pub fn cursor(&self) -> usize {
        self.cursor
    }

    /// Returns `true` if all waypoints have been visited.
    #[must_use]
    pub fn complete(&self) -> bool {
        self.cursor >= self.sequence.waypoints.len()
    }
}

impl Job for WaypointGuidance {
    fn name(&self) -> &'static str {
        self.name
    }

    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        if self.complete() {
            return Ok(());
        }
        let armed_in_flight = ctx
            .bus
            .latest::<VehicleStatus>()?
            .is_some_and(|(s, _)| s.armed && s.in_flight);
        if !armed_in_flight {
            return Ok(());
        }

        let pos_estimate = ctx.bus.latest::<PositionEstimate>()?.map(|(p, _)| p);
        if let Some(pe) = pos_estimate {
            // Defensive .get() so an over-incremented cursor never panics.
            if let Some(active) = self.sequence.waypoints.get(self.cursor) {
                let delta = active.position_eci_m - pe.position_eci_m;
                if delta.norm() <= self.params.waypoint_acceptance_radius_m {
                    self.cursor = self.cursor.saturating_add(1);
                }
            }
        }

        let active = self.sequence.waypoints.get(
            self.cursor
                .min(self.sequence.waypoints.len().saturating_sub(1)),
        );

        let reference = if let Some(wp) = active {
            ReferenceState {
                time: ctx.clock.now(),
                q_body_to_eci_xyzw: wp.reference_q_xyzw,
                omega_body_rad_s: Vector3::zeros(),
                position_eci_m: wp.position_eci_m,
                velocity_eci_m_s: wp.velocity_eci_m_s,
            }
        } else {
            ReferenceState {
                time: ctx.clock.now(),
                q_body_to_eci_xyzw: self.params.default_reference_q_xyzw,
                omega_body_rad_s: Vector3::zeros(),
                position_eci_m: Vector3::zeros(),
                velocity_eci_m_s: Vector3::zeros(),
            }
        };
        let _ = ctx.bus.publish(reference);
        Ok(())
    }
}

/// Static-attitude-hold guidance: publishes a constant reference
/// state. Useful for academic attitude-tracking experiments where a
/// waypoint sequence is not appropriate.
#[derive(Debug)]
pub struct AttitudeHoldGuidance {
    name: &'static str,
    reference_q_xyzw: [f64; 4],
}

impl AttitudeHoldGuidance {
    /// Constructs an attitude-hold guidance with the given reference.
    #[must_use]
    pub fn new(reference_q_xyzw: [f64; 4]) -> Self {
        Self {
            name: "guidance.attitude_hold",
            reference_q_xyzw,
        }
    }
}

impl Job for AttitudeHoldGuidance {
    fn name(&self) -> &'static str {
        self.name
    }

    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        let reference = ReferenceState {
            time: ctx.clock.now(),
            q_body_to_eci_xyzw: self.reference_q_xyzw,
            omega_body_rad_s: Vector3::zeros(),
            position_eci_m: Vector3::zeros(),
            velocity_eci_m_s: Vector3::zeros(),
        };
        let _ = ctx.bus.publish(reference);
        Ok(())
    }
}

/// Powered-ascent reference guidance.
///
/// The job samples the estimator's translational state, delegates the
/// reference attitude to an [`AscentReferenceGenerator`], and publishes
/// a [`ReferenceState`] for the autopilot. It is phase-gated by
/// mission phase id when constructed with active phase ids.
pub struct AscentReferenceGuidance {
    name: &'static str,
    generator: Box<dyn AscentReferenceGenerator + Send>,
    active_phase_ids: Vec<u64>,
}

impl std::fmt::Debug for AscentReferenceGuidance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AscentReferenceGuidance")
            .field("name", &self.name)
            .field("active_phase_ids", &self.active_phase_ids)
            .finish_non_exhaustive()
    }
}

impl AscentReferenceGuidance {
    /// Constructs an ascent-reference guidance job.
    #[must_use]
    pub fn new(generator: Box<dyn AscentReferenceGenerator + Send>) -> Self {
        Self {
            name: "guidance.ascent_reference",
            generator,
            active_phase_ids: Vec::new(),
        }
    }

    /// Restrict reference publication to the listed mission phase ids.
    #[must_use]
    pub fn with_active_phase_ids(mut self, phase_ids: Vec<u64>) -> Self {
        self.active_phase_ids = phase_ids;
        self
    }

    fn should_run_for_status(&self, status: VehicleStatus) -> bool {
        status.armed
            && status.in_flight
            && (self.active_phase_ids.is_empty()
                || self.active_phase_ids.contains(&status.phase_id))
    }
}

impl Job for AscentReferenceGuidance {
    fn name(&self) -> &'static str {
        self.name
    }

    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        let Some((status, _)) = ctx.bus.latest::<VehicleStatus>()? else {
            return Ok(());
        };
        if !self.should_run_for_status(status) {
            return Ok(());
        }
        let Some((position, _)) = ctx.bus.latest::<PositionEstimate>()? else {
            return Ok(());
        };
        let speed = position.velocity_eci_m_s.norm();
        let flight_path_angle_rad = if speed > 0.0 {
            (position.velocity_eci_m_s.z / speed)
                .clamp(-1.0, 1.0)
                .asin()
        } else {
            0.0
        };
        let state = AscentState {
            position_eci_m: [
                position.position_eci_m.x,
                position.position_eci_m.y,
                position.position_eci_m.z,
            ],
            velocity_eci_m_s: [
                position.velocity_eci_m_s.x,
                position.velocity_eci_m_s.y,
                position.velocity_eci_m_s.z,
            ],
            altitude_m: position.position_eci_m.z,
            inertial_speed_m_s: speed,
            flight_path_angle_rad,
            dynamic_pressure_pa: 0.0,
            mass_fraction: 1.0,
        };
        let reference = self
            .generator
            .ascent_reference(&state, ctx.clock.now())
            .map_err(|err| GuidanceError::ReferenceGeneration {
                reason: err.to_string(),
            })?;
        ctx.bus.publish(ReferenceState {
            time: ctx.clock.now(),
            q_body_to_eci_xyzw: reference.q_body_to_eci_xyzw,
            omega_body_rad_s: reference
                .body_rate_rad_s
                .map_or_else(Vector3::zeros, |omega| {
                    Vector3::new(omega[0], omega[1], omega[2])
                }),
            position_eci_m: position.position_eci_m,
            velocity_eci_m_s: position.velocity_eci_m_s,
        })?;
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use openbmp_core::{SimTime, StepIndex};
    use openbmp_physics::profile::PitchProgramAscentReference;

    use super::*;
    use crate::bus::Bus;
    use crate::clock::SimulatedClock;

    #[test]
    fn ascent_reference_guidance_publishes_pitch_program_reference() {
        let bus = Bus::new();
        bus.register::<VehicleStatus>().unwrap();
        bus.register::<PositionEstimate>().unwrap();
        bus.register::<ReferenceState>().unwrap();
        bus.publish(VehicleStatus {
            phase_id: 42,
            armed: true,
            in_flight: true,
            safe_state_requested: false,
        })
        .unwrap();
        bus.publish(PositionEstimate {
            time: SimTime::from_seconds(5.0),
            position_eci_m: Vector3::new(0.0, 0.0, 100.0),
            velocity_eci_m_s: Vector3::new(10.0, 0.0, 100.0),
            accel_bias_body_m_s2: Vector3::zeros(),
        })
        .unwrap();
        let clock = SimulatedClock::at(SimTime::from_seconds(5.0), StepIndex::new(5));
        let mut job = AscentReferenceGuidance::new(Box::new(
            PitchProgramAscentReference::new(vec![0.0, 10.0], vec![0.0, 0.4]).unwrap(),
        ))
        .with_active_phase_ids(vec![42]);

        job.run(&JobContext {
            bus: &bus,
            clock: &clock,
        })
        .unwrap();

        let (reference, _) = bus.latest::<ReferenceState>().unwrap().unwrap();
        assert_eq!(reference.time, SimTime::from_seconds(5.0));
        assert!(reference.q_body_to_eci_xyzw.iter().all(|v| v.is_finite()));
        assert_eq!(reference.position_eci_m, Vector3::new(0.0, 0.0, 100.0));
        assert_eq!(reference.velocity_eci_m_s, Vector3::new(10.0, 0.0, 100.0));
    }

    #[test]
    fn ascent_reference_guidance_is_phase_gated() {
        let bus = Bus::new();
        bus.register::<VehicleStatus>().unwrap();
        bus.register::<PositionEstimate>().unwrap();
        bus.register::<ReferenceState>().unwrap();
        bus.publish(VehicleStatus {
            phase_id: 7,
            armed: true,
            in_flight: true,
            safe_state_requested: false,
        })
        .unwrap();
        bus.publish(PositionEstimate {
            time: SimTime::from_seconds(5.0),
            position_eci_m: Vector3::new(0.0, 0.0, 100.0),
            velocity_eci_m_s: Vector3::new(10.0, 0.0, 100.0),
            accel_bias_body_m_s2: Vector3::zeros(),
        })
        .unwrap();
        let clock = SimulatedClock::at(SimTime::from_seconds(5.0), StepIndex::new(5));
        let mut job = AscentReferenceGuidance::new(Box::new(
            PitchProgramAscentReference::new(vec![0.0, 10.0], vec![0.0, 0.4]).unwrap(),
        ))
        .with_active_phase_ids(vec![42]);

        job.run(&JobContext {
            bus: &bus,
            clock: &clock,
        })
        .unwrap();

        assert!(bus.latest::<ReferenceState>().unwrap().is_none());
    }
}
