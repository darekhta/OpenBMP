//! Plain-Rust web session: an [`openbmp_runner::Session`] over an
//! embedded scenario, a flat numeric snapshot for render hosts, and a
//! logged, replayable malfunction-stimulus timeline.
//!
//! Everything here is target-independent and natively testable; the
//! WebAssembly bindings in `lib.rs` are a thin marshalling shell over
//! this module.

use std::collections::VecDeque;

use openbmp_core::{BodyId, EngineId};
use openbmp_runner::{EngineFault, Session};
use openbmp_scenario::{
    BodyGeometryConfig, FcEstimatorKind, ScenarioDocument, TankGeometryConfig,
};
use serde::{Deserialize, Serialize};

use crate::assets;
use crate::snapshot::SnapshotLayout;

/// One stimulus command, tagged for JSON logging and replay.
///
/// Commands address simulator-side malfunction seams only (engine
/// faults, sensor outages, wind) — the same simulated fault-injection
/// scope `docs/safety-boundaries.md` places in bounds. There is no
/// command that steers the vehicle: the flight controller flies, the
/// host can only disturb and observe.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum StimulusCommand {
    /// Schedule an engine malfunction (see [`WebSession::inject_engine_fault`]).
    EngineFault {
        /// Scenario engine name.
        engine: String,
        /// Fault kind: `hard_off`, `thrust_loss`, `stuck_throttle`, or
        /// `gimbal_lock`.
        kind: String,
        /// Kind-specific magnitude (unused for `hard_off`).
        magnitude: f64,
    },
    /// Withhold a sensor's measurements over an absolute time window.
    SensorOutage {
        /// `[sensors.<name>]` key.
        sensor: String,
        /// Window start (simulation seconds, inclusive).
        start_s: f64,
        /// Window stop (simulation seconds, exclusive).
        stop_s: f64,
    },
    /// Set or clear the wind override (NED, m/s).
    Wind {
        /// NED wind vector (m/s).
        wind_ned_m_s: [f64; 3],
        /// `false` clears the override regardless of the vector.
        enabled: bool,
    },
}

/// A stimulus command pinned to the kernel step it applies at.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct TimedCommand {
    /// Kernel step index the command applies before.
    pub step: u64,
    /// The command itself.
    #[serde(flatten)]
    pub command: StimulusCommand,
}

/// Serialised stimulus timeline: scenario key, seed, and the ordered
/// command list. Two sessions constructed from the same timeline
/// reproduce byte-identical runs (the determinism contract of
/// [`openbmp_runner::Session`]).
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct StimulusTimeline {
    /// Timeline schema version.
    pub version: u32,
    /// Embedded scenario key the timeline applies to.
    pub scenario: String,
    /// Scenario seed the run was constructed with.
    pub seed: u64,
    /// Ordered command list (non-decreasing `step`).
    pub commands: Vec<TimedCommand>,
}

/// A stepping web session over an embedded scenario.
#[derive(Debug)]
pub struct WebSession {
    session: Session,
    layout: SnapshotLayout,
    scenario_key: String,
    seed: u64,
    log: Vec<TimedCommand>,
    replay: VecDeque<TimedCommand>,
    snapshot_buffer: Vec<f64>,
    engine_ids: Vec<(String, EngineId)>,
    body_ids: Vec<(String, BodyId)>,
}

/// Apply a "flight-software config" selection to the parsed scenario before
/// the controller is built. This is the demo's reset-with-config path: the
/// vehicle and mission are unchanged; only the onboard NAVIGATION FILTER is
/// swapped, so the EKF and the square-root UKF can be compared on the same
/// flight. (The FlightController stays immutable once built — a determinism
/// property — so a mode change re-runs the mission rather than hot-swapping
/// mid-flight; and the LQR/INDI control laws need a single-body assembly, so
/// they don't apply to this two-stage vehicle.)
fn apply_fc_mode(document: &mut ScenarioDocument, fc_mode: Option<&str>) -> Result<(), String> {
    let mode = match fc_mode {
        Some(mode) if !mode.is_empty() && mode != "default" => mode,
        _ => return Ok(()), // scenario default (EKF)
    };
    let fc = document.fc.as_mut().ok_or_else(|| {
        format!("scenario has no [fc] block; cannot apply flight-computer mode `{mode}`")
    })?;
    if fc.estimator_lanes.is_some() {
        return Err(
            "scenario uses [fc.estimator_lanes]; single-estimator override unavailable".to_owned(),
        );
    }
    match mode {
        "ekf" => fc.estimator = FcEstimatorKind::Ekf,
        // SR-UKF reuses the [fc.ekf] parameter set — no extra config needed.
        "sr_ukf" => fc.estimator = FcEstimatorKind::SrUkf,
        other => return Err(format!("unknown flight-computer mode `{other}`")),
    }
    Ok(())
}

impl WebSession {
    /// Construct a session over the embedded scenario `key`.
    ///
    /// `seed_override` replaces the scenario's declared seed before the
    /// kernel is built (vary it for run-to-run sensor-noise diversity;
    /// omit it for the canonical run). `timeline_json`, when given,
    /// must be a [`StimulusTimeline`] for the same scenario key; its
    /// seed wins over both the scenario seed and `seed_override`, and
    /// its commands are re-applied at their recorded steps so a shared
    /// timeline reproduces the original run exactly.
    ///
    /// # Errors
    ///
    /// Returns a message for an unknown key, a parse/pin failure in the
    /// embedded assets, a timeline for a different scenario, or a
    /// kernel construction failure.
    pub fn new(
        key: &str,
        seed_override: Option<u64>,
        timeline_json: Option<&str>,
        fc_mode: Option<&str>,
    ) -> Result<Self, String> {
        let embedded = assets::find(key)
            .ok_or_else(|| format!("unknown embedded scenario `{key}` (see scenario list)"))?;
        let timeline: Option<StimulusTimeline> = timeline_json
            .map(serde_json::from_str)
            .transpose()
            .map_err(|err| format!("stimulus timeline does not parse: {err}"))?;
        if let Some(timeline) = &timeline {
            if timeline.scenario != key {
                return Err(format!(
                    "stimulus timeline is for scenario `{}`, not `{key}`",
                    timeline.scenario
                ));
            }
            if !timeline.commands.windows(2).all(|w| w[0].step <= w[1].step) {
                return Err("stimulus timeline commands must be ordered by step".to_owned());
            }
        }

        let (mut scenario, resolved_files) = embedded.load().map_err(|err| err.to_string())?;
        let seed = timeline
            .as_ref()
            .map(|timeline| timeline.seed)
            .or(seed_override)
            .unwrap_or(scenario.document.time.seed);
        scenario.document.time.seed = seed;
        apply_fc_mode(&mut scenario.document, fc_mode)?;

        let layout = SnapshotLayout::from_document(key, embedded.title, &scenario.document);
        let engine_ids = scenario
            .document
            .vehicle
            .assembly
            .engines
            .iter()
            .map(|engine| {
                (
                    engine.id.clone(),
                    EngineId::from_path(&format!("vehicle.assembly.engines.{}", engine.id)),
                )
            })
            .collect();
        let body_ids = scenario
            .document
            .vehicle
            .assembly
            .bodies
            .iter()
            .map(|body| {
                (
                    body.id.clone(),
                    BodyId::from_path(&format!("vehicle.assembly.bodies.{}", body.id)),
                )
            })
            .collect();

        let session = Session::prepare_with_files(scenario, &resolved_files)
            .map_err(|err| err.to_string())?;
        let stride = layout.stride;
        Ok(Self {
            session,
            layout,
            scenario_key: key.to_owned(),
            seed,
            log: Vec::new(),
            replay: timeline
                .map(|timeline| timeline.commands.into())
                .unwrap_or_default(),
            snapshot_buffer: vec![0.0; stride],
            engine_ids,
            body_ids,
        })
    }

    /// JSON list of the embedded scenarios (`[{key, title}]`).
    #[must_use]
    pub fn scenario_list_json() -> String {
        let list: Vec<serde_json::Value> = assets::SCENARIOS
            .iter()
            .map(|scenario| serde_json::json!({ "key": scenario.key, "title": scenario.title }))
            .collect();
        serde_json::Value::Array(list).to_string()
    }

    /// Advance up to `n` kernel ticks, applying any replayed commands
    /// at their recorded steps. Returns `true` once the run is
    /// finished. Recorded telemetry rows are drained each call so a
    /// long-running host's memory stays bounded.
    ///
    /// # Errors
    ///
    /// Returns the runner's error message; the session is not usable
    /// after an error.
    pub fn step_many(&mut self, n: u32) -> Result<bool, String> {
        for _ in 0..n {
            if self.session.is_finished() {
                break;
            }
            let step = self.session.step_index();
            while self
                .replay
                .front()
                .is_some_and(|command| command.step <= step)
            {
                let timed = self
                    .replay
                    .pop_front()
                    .unwrap_or_else(|| unreachable!("front checked above"));
                self.apply(timed.command.clone())
                    .map_err(|err| format!("replayed command failed: {err}"))?;
            }
            self.session.step().map_err(|err| err.to_string())?;
        }
        let _ = self.session.take_recorded_rows();
        Ok(self.session.is_finished())
    }

    /// Whether the kernel has reported a stop reason.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.session.is_finished()
    }

    /// Current simulation time in seconds.
    #[must_use]
    pub fn time_s(&self) -> f64 {
        self.session.time_s()
    }

    /// Current kernel step index.
    #[must_use]
    pub fn step_index(&self) -> u64 {
        self.session.step_index()
    }

    /// Scenario-declared label of the current mission phase.
    #[must_use]
    pub fn mission_phase(&self) -> String {
        self.session.mission_phase()
    }

    /// Snapshot layout as JSON (field indices, engines, tanks, bodies,
    /// phases — everything a renderer needs to interpret
    /// [`Self::snapshot`] and build the vehicle from the scenario).
    #[must_use]
    pub fn layout_json(&self) -> String {
        self.layout.to_json()
    }

    /// Encode the current state into the flat numeric snapshot
    /// described by [`Self::layout_json`].
    #[must_use]
    pub fn snapshot(&mut self) -> Vec<f64> {
        self.layout.encode(
            &self.session,
            &self.engine_ids,
            &self.body_ids,
            &mut self.snapshot_buffer,
        );
        self.snapshot_buffer.clone()
    }

    /// Schedule an engine malfunction by scenario engine name, logged
    /// into the stimulus timeline. Kinds: `hard_off` (terminal
    /// shutdown; magnitude unused), `thrust_loss` (thrust multiplier
    /// `magnitude ∈ [0, 1)`), `stuck_throttle` (throttle stuck at
    /// `magnitude ∈ [0, 1]`), `gimbal_lock` (both gimbal axes frozen at
    /// `magnitude` radians).
    ///
    /// # Errors
    ///
    /// Returns a message for an unknown kind, a non-finite or
    /// out-of-range magnitude, or an unknown engine (fail-closed).
    pub fn inject_engine_fault(
        &mut self,
        engine: &str,
        kind: &str,
        magnitude: f64,
    ) -> Result<(), String> {
        let command = StimulusCommand::EngineFault {
            engine: engine.to_owned(),
            kind: kind.to_owned(),
            magnitude,
        };
        self.apply_logged(command)
    }

    /// Withhold a sensor's measurements for `duration_s` starting now,
    /// logged into the stimulus timeline.
    ///
    /// # Errors
    ///
    /// Returns a message for an unknown sensor or non-positive
    /// duration (fail-closed).
    pub fn inject_sensor_outage(&mut self, sensor: &str, duration_s: f64) -> Result<(), String> {
        if !duration_s.is_finite() || duration_s <= 0.0 {
            return Err(format!(
                "sensor outage duration must be a positive number of seconds (got {duration_s})"
            ));
        }
        let start_s = self.session.time_s();
        let command = StimulusCommand::SensorOutage {
            sensor: sensor.to_owned(),
            start_s,
            stop_s: start_s + duration_s,
        };
        self.apply_logged(command)
    }

    /// Set or clear the wind override (NED, m/s), logged into the
    /// stimulus timeline.
    ///
    /// # Errors
    ///
    /// Returns a message when a component is non-finite.
    pub fn set_wind(&mut self, wind_ned_m_s: [f64; 3], enabled: bool) -> Result<(), String> {
        if !wind_ned_m_s.iter().all(|c| c.is_finite()) {
            return Err("wind override components must be finite".to_owned());
        }
        let command = StimulusCommand::Wind {
            wind_ned_m_s,
            enabled,
        };
        self.apply_logged(command)
    }

    /// The stimulus timeline so far, as shareable JSON.
    #[must_use]
    pub fn timeline_json(&self) -> String {
        let timeline = StimulusTimeline {
            version: 1,
            scenario: self.scenario_key.clone(),
            seed: self.seed,
            commands: self.log.clone(),
        };
        serde_json::to_string(&timeline)
            .unwrap_or_else(|_| unreachable!("timeline serialisation is infallible"))
    }

    fn apply_logged(&mut self, command: StimulusCommand) -> Result<(), String> {
        self.apply(command.clone())?;
        self.log.push(TimedCommand {
            step: self.session.step_index(),
            command,
        });
        Ok(())
    }

    fn apply(&mut self, command: StimulusCommand) -> Result<(), String> {
        match command {
            StimulusCommand::EngineFault {
                engine,
                kind,
                magnitude,
            } => {
                let fault = engine_fault_from_parts(&kind, magnitude)?;
                self.session
                    .inject_engine_fault(&engine, fault)
                    .map(|_| ())
                    .map_err(|err| err.to_string())
            }
            StimulusCommand::SensorOutage {
                sensor,
                start_s,
                stop_s,
            } => self
                .session
                .inject_sensor_outage(&sensor, start_s, stop_s)
                .map_err(|err| err.to_string()),
            StimulusCommand::Wind {
                wind_ned_m_s,
                enabled,
            } => {
                self.session
                    .set_wind_override(enabled.then_some(wind_ned_m_s));
                Ok(())
            }
        }
    }
}

/// Map a (kind, magnitude) pair onto a typed [`EngineFault`],
/// validating magnitude ranges fail-closed.
fn engine_fault_from_parts(kind: &str, magnitude: f64) -> Result<EngineFault, String> {
    match kind {
        "hard_off" => Ok(EngineFault::HardOff),
        "thrust_loss" => {
            if !magnitude.is_finite() || !(0.0..1.0).contains(&magnitude) {
                return Err(format!(
                    "thrust_loss magnitude is the post-fault thrust multiplier and must be in [0, 1) (got {magnitude})"
                ));
            }
            Ok(EngineFault::OverThrust { factor: magnitude })
        }
        "stuck_throttle" => {
            if !magnitude.is_finite() || !(0.0..=1.0).contains(&magnitude) {
                return Err(format!(
                    "stuck_throttle magnitude is the stuck throttle setting and must be in [0, 1] (got {magnitude})"
                ));
            }
            Ok(EngineFault::Stuck {
                at_throttle: magnitude,
            })
        }
        "gimbal_lock" => {
            if !magnitude.is_finite() {
                return Err("gimbal_lock magnitude must be finite radians".to_owned());
            }
            Ok(EngineFault::GimbalLocked {
                pitch_rad: magnitude,
                yaw_rad: magnitude,
            })
        }
        other => Err(format!(
            "unknown engine fault kind `{other}` (expected hard_off, thrust_loss, stuck_throttle, or gimbal_lock)"
        )),
    }
}

/// Internal volume of a tank geometry (m³) — mirrors the scenario
/// crate's private volume computation for the closed-form kinds.
pub(crate) fn tank_volume_m3(geometry: &TankGeometryConfig) -> f64 {
    match *geometry {
        TankGeometryConfig::Cylinder { radius_m, height_m } => {
            std::f64::consts::PI * radius_m * radius_m * height_m
        }
        TankGeometryConfig::Sphere { radius_m } => {
            (4.0 / 3.0) * std::f64::consts::PI * radius_m.powi(3)
        }
        TankGeometryConfig::EllipsoidTextbook { a_m, b_m, c_m } => {
            (4.0 / 3.0) * std::f64::consts::PI * a_m * b_m * c_m
        }
    }
}

/// Renderable proxy dimensions of a body geometry: `(length_m,
/// diameter_m)`.
pub(crate) fn body_render_dimensions(geometry: &BodyGeometryConfig) -> (f64, f64) {
    match *geometry {
        BodyGeometryConfig::Cylinder {
            length_m,
            diameter_m,
        } => (length_m, diameter_m),
        BodyGeometryConfig::Cone {
            length_m,
            base_diameter_m,
        } => (length_m, base_diameter_m),
        BodyGeometryConfig::Reference { length_m, area_m2 } => {
            (length_m, 2.0 * (area_m2 / std::f64::consts::PI).sqrt())
        }
    }
}
