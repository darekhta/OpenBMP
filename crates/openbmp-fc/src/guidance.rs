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
use crate::nav_metrics;
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::topics::{
    EnvironmentEstimate, GuidanceCutoff, ImuSample, PositionEstimate, ReferenceState, VehicleStatus,
};

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

    /// Override the scheduler job name. Required when more than one
    /// ascent-reference guidance job is registered (per-phase guidance),
    /// since the scheduler rejects duplicate job names.
    #[must_use]
    pub fn with_name(mut self, name: &'static str) -> Self {
        self.name = name;
        self
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
        // Sensed thrust acceleration: the IMU specific-force magnitude
        // (≈ thrust/mass in flight). PEG uses it to track the burn-time
        // constant from the live state.
        let thrust_accel_m_s2 = ctx
            .bus
            .latest::<ImuSample>()
            .ok()
            .flatten()
            .map_or(0.0, |(s, _)| s.accel_m_s2.norm());
        let environment = ctx
            .bus
            .latest::<EnvironmentEstimate>()
            .ok()
            .flatten()
            .map(|(e, _)| e);
        let surface_relative_velocity = nav_metrics::air_relative_velocity_eci_m_s(&position);
        let speed = position.velocity_eci_m_s.norm();
        let dynamic_pressure_pa = environment.map_or_else(
            || nav_metrics::dynamic_pressure_air_relative(&position),
            |env| {
                nav_metrics::dynamic_pressure_air_relative_with_density(
                    &position,
                    env.density_kg_m3,
                )
            },
        );
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
            surface_relative_velocity_eci_m_s: [
                surface_relative_velocity.x,
                surface_relative_velocity.y,
                surface_relative_velocity.z,
            ],
            altitude_m: nav_metrics::altitude_m(position.position_eci_m),
            inertial_speed_m_s: speed,
            surface_relative_speed_m_s: surface_relative_velocity.norm(),
            flight_path_angle_rad: nav_metrics::flight_path_angle_rad(
                position.position_eci_m,
                surface_relative_velocity,
            ),
            dynamic_pressure_pa,
            mass_fraction: 1.0,
            thrust_accel_m_s2,
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
        // Publish the guidance time-to-go (if the active method computes
        // one) so the commander can schedule engine cutoff at insertion.
        let _ = ctx.bus.publish(GuidanceCutoff {
            time: ctx.clock.now(),
            time_to_go_s: self.generator.time_to_go_s().unwrap_or(f64::INFINITY),
        });
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::sync::{Arc, Mutex};

    use openbmp_core::{SimTime, StepIndex};
    use openbmp_physics::profile::{
        AscentReference, AscentReferenceGenerator, AscentState, PitchProgramAscentReference,
    };

    use super::*;
    use crate::bus::Bus;
    use crate::clock::SimulatedClock;

    #[derive(Debug)]
    struct RecordingReference {
        state: Arc<Mutex<Option<AscentState>>>,
    }

    impl AscentReferenceGenerator for RecordingReference {
        fn ascent_reference(
            &self,
            state: &AscentState,
            _time: SimTime,
        ) -> Result<AscentReference, openbmp_physics::PhysicsError> {
            *self.state.lock().unwrap() = Some(*state);
            Ok(AscentReference {
                q_body_to_eci_xyzw: [0.0, 0.0, 0.0, 1.0],
                body_rate_rad_s: None,
            })
        }
    }

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

    #[test]
    fn ascent_state_uses_geocentric_altitude_and_surface_relative_flight_path_angle() {
        let bus = Bus::new();
        bus.register::<VehicleStatus>().unwrap();
        bus.register::<PositionEstimate>().unwrap();
        bus.register::<ReferenceState>().unwrap();
        bus.register::<GuidanceCutoff>().unwrap();
        bus.publish(VehicleStatus {
            phase_id: 42,
            armed: true,
            in_flight: true,
            safe_state_requested: false,
        })
        .unwrap();
        let position = PositionEstimate {
            time: SimTime::from_seconds(5.0),
            position_eci_m: Vector3::new(6_471_000.0, 0.0, 0.0),
            velocity_eci_m_s: Vector3::new(100.0, 100.0, 0.0),
            accel_bias_body_m_s2: Vector3::zeros(),
        };
        let expected_surface_relative_velocity =
            nav_metrics::air_relative_velocity_eci_m_s(&position);
        bus.publish(position).unwrap();
        let clock = SimulatedClock::at(SimTime::from_seconds(5.0), StepIndex::new(5));
        let captured = Arc::new(Mutex::new(None));
        let mut job = AscentReferenceGuidance::new(Box::new(RecordingReference {
            state: Arc::clone(&captured),
        }))
        .with_active_phase_ids(vec![42]);

        job.run(&JobContext {
            bus: &bus,
            clock: &clock,
        })
        .unwrap();

        let state = captured
            .lock()
            .unwrap()
            .expect("guidance should pass ascent state to generator");
        assert!((state.altitude_m - 100_000.0).abs() < 1.0e-9);
        assert_eq!(state.velocity_eci_m_s, [100.0, 100.0, 0.0]);
        assert!(
            (Vector3::from(state.surface_relative_velocity_eci_m_s)
                - expected_surface_relative_velocity)
                .norm()
                < 1.0e-12
        );
        assert!(
            (state.surface_relative_speed_m_s - expected_surface_relative_velocity.norm()).abs()
                < 1.0e-12
        );
        let expected_gamma = nav_metrics::flight_path_angle_rad(
            position.position_eci_m,
            expected_surface_relative_velocity,
        );
        assert!((state.flight_path_angle_rad - expected_gamma).abs() < 1.0e-12);
    }

    #[test]
    fn ascent_state_dynamic_pressure_uses_published_environment_density() {
        let bus = Bus::new();
        bus.register::<VehicleStatus>().unwrap();
        bus.register::<PositionEstimate>().unwrap();
        bus.register::<EnvironmentEstimate>().unwrap();
        bus.register::<ReferenceState>().unwrap();
        bus.register::<GuidanceCutoff>().unwrap();
        bus.publish(VehicleStatus {
            phase_id: 42,
            armed: true,
            in_flight: true,
            safe_state_requested: false,
        })
        .unwrap();
        let surface_radius_m = 6_371_000.0;
        let omega = openbmp_physics::frames::WGS84_OMEGA_RAD_S;
        bus.publish(PositionEstimate {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(surface_radius_m, 0.0, 0.0),
            velocity_eci_m_s: Vector3::new(0.0, omega * surface_radius_m + 300.0, 0.0),
            accel_bias_body_m_s2: Vector3::zeros(),
        })
        .unwrap();
        bus.publish(EnvironmentEstimate {
            time: SimTime::ZERO,
            density_kg_m3: 0.01,
        })
        .unwrap();

        let clock = SimulatedClock::at(SimTime::ZERO, StepIndex::ZERO);
        let captured = Arc::new(Mutex::new(None));
        let mut job = AscentReferenceGuidance::new(Box::new(RecordingReference {
            state: Arc::clone(&captured),
        }))
        .with_active_phase_ids(vec![42]);

        job.run(&JobContext {
            bus: &bus,
            clock: &clock,
        })
        .unwrap();

        let state = captured
            .lock()
            .unwrap()
            .expect("guidance should pass ascent state to generator");
        assert!(
            (state.dynamic_pressure_pa - 450.0).abs() < 1.0e-9,
            "ascent guidance should derive q from the runtime atmosphere density: {}",
            state.dynamic_pressure_pa
        );
    }
}
