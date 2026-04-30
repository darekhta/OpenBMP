//! Academic guidance laws.
//!
//! Phase 4.7 ships:
//! - Attitude tracking against a scripted reference state.
//! - Scenario-waypoint navigation in inertial space.
//! - Simple terminal-state regulation (constant attitude / position).
//!
//! The safety-boundaries `Reject` list applies categorically — no
//! proportional navigation, no terminal-homing, no targeting, no
//! real-world-location guidance.

use nalgebra::Vector3;

use crate::error::ControllerError;
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
