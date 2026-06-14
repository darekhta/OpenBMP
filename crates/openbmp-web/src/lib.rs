//! `openbmp-web` — WebAssembly host for the OpenBMP runner session.
//!
//! This crate packages the steppable [`openbmp_runner::Session`] with
//! embedded Phalcon-9 scenarios (and every data file they pin) behind a
//! small JavaScript-facing API, so a browser can drive the actual
//! simulation — kernel, physics, sensors, and the in-loop flight
//! controller — inside a Web Worker with no server and no filesystem.
//!
//! Surface (see `web/ascent/` for the reference front-end):
//!
//! - [`AscentSim::new`] — construct over an embedded scenario, with an
//!   optional seed override and an optional recorded stimulus timeline
//!   to replay.
//! - [`AscentSim::step_many`] — advance N kernel ticks.
//! - [`AscentSim::snapshot`] / [`AscentSim::layout_json`] — flat `f64`
//!   state array plus the JSON layout/render-constants that interpret
//!   it.
//! - `inject_*` / [`AscentSim::set_wind`] — deterministic malfunction
//!   stimuli (engine faults, sensor outages, wind), logged into a
//!   shareable timeline ([`AscentSim::timeline_json`]).
//!
//! The host can disturb and observe; it cannot steer. Guidance,
//! navigation, and control stay onboard — the same forward-only
//! observation boundary the SIL bench uses (`openbmp_runner::sil`) —
//! and the stimuli reuse the simulator's scenario-declarable fault
//! seams (`docs/safety-boundaries.md`, "simulated fault-injection
//! paths").
//!
//! Everything except the thin `#[wasm_bindgen]` shell is plain,
//! natively-tested Rust ([`session::WebSession`]); the bindings only
//! marshal strings and `f64` arrays.

pub mod assets;
pub mod session;
pub mod snapshot;

use wasm_bindgen::prelude::*;

use crate::session::WebSession;

/// JSON list of the embedded scenarios: `[{ "key", "title" }]`.
#[wasm_bindgen]
#[must_use]
pub fn scenario_list() -> String {
    WebSession::scenario_list_json()
}

/// Install the panic-to-console hook so a wasm panic surfaces as a
/// readable browser-console error instead of an opaque `unreachable`.
#[wasm_bindgen(start)]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

/// A stepping simulation of one embedded scenario, driven from
/// JavaScript.
#[wasm_bindgen]
#[derive(Debug)]
pub struct AscentSim {
    inner: WebSession,
}

#[wasm_bindgen]
impl AscentSim {
    /// Construct a session over the embedded scenario `key` (see
    /// [`scenario_list`]).
    ///
    /// `seed` overrides the scenario's declared RNG seed (sensor-noise
    /// diversity; omit for the canonical run). `timeline_json` replays
    /// a previously recorded stimulus timeline
    /// ([`Self::timeline_json`]); its seed wins when present.
    ///
    /// # Errors
    ///
    /// Rejects unknown keys, malformed timelines, timelines recorded
    /// for a different scenario, and scenario/kernel construction
    /// failures.
    #[wasm_bindgen(constructor)]
    pub fn new(
        key: &str,
        seed: Option<u64>,
        timeline_json: Option<String>,
        fc_mode: Option<String>,
    ) -> Result<AscentSim, JsError> {
        WebSession::new(key, seed, timeline_json.as_deref(), fc_mode.as_deref())
            .map(|inner| Self { inner })
            .map_err(|message| JsError::new(&message))
    }

    /// Advance up to `n` kernel ticks; returns `true` once the run is
    /// finished.
    ///
    /// # Errors
    ///
    /// Propagates runner errors; the session is not usable afterwards.
    pub fn step_many(&mut self, n: u32) -> Result<bool, JsError> {
        self.inner
            .step_many(n)
            .map_err(|message| JsError::new(&message))
    }

    /// Whether the run has finished.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.inner.is_finished()
    }

    /// Current simulation time in seconds.
    #[must_use]
    pub fn time_s(&self) -> f64 {
        self.inner.time_s()
    }

    /// Current kernel step index.
    #[must_use]
    pub fn step_index(&self) -> u64 {
        self.inner.step_index()
    }

    /// Scenario-declared label of the current mission phase.
    #[must_use]
    pub fn mission_phase(&self) -> String {
        self.inner.mission_phase()
    }

    /// Snapshot layout and render constants as JSON (interprets
    /// [`Self::snapshot`]).
    #[must_use]
    pub fn layout_json(&self) -> String {
        self.inner.layout_json()
    }

    /// Flat `f64` state snapshot (a `Float64Array` on the JS side),
    /// laid out per [`Self::layout_json`].
    #[must_use]
    pub fn snapshot(&mut self) -> Vec<f64> {
        self.inner.snapshot()
    }

    /// Schedule an engine malfunction (`hard_off`, `thrust_loss`,
    /// `stuck_throttle`, or `gimbal_lock`) by scenario engine name.
    ///
    /// # Errors
    ///
    /// Rejects unknown kinds, out-of-range magnitudes, and unknown
    /// engines (fail-closed).
    pub fn inject_engine_fault(
        &mut self,
        engine: &str,
        kind: &str,
        magnitude: f64,
    ) -> Result<(), JsError> {
        self.inner
            .inject_engine_fault(engine, kind, magnitude)
            .map_err(|message| JsError::new(&message))
    }

    /// Withhold a sensor's measurements for `duration_s` starting now.
    ///
    /// # Errors
    ///
    /// Rejects unknown sensors and non-positive durations
    /// (fail-closed).
    pub fn inject_sensor_outage(&mut self, sensor: &str, duration_s: f64) -> Result<(), JsError> {
        self.inner
            .inject_sensor_outage(sensor, duration_s)
            .map_err(|message| JsError::new(&message))
    }

    /// Set (`enabled = true`) or clear the wind override, NED m/s.
    ///
    /// # Errors
    ///
    /// Rejects non-finite components.
    pub fn set_wind(
        &mut self,
        north_m_s: f64,
        east_m_s: f64,
        down_m_s: f64,
        enabled: bool,
    ) -> Result<(), JsError> {
        self.inner
            .set_wind([north_m_s, east_m_s, down_m_s], enabled)
            .map_err(|message| JsError::new(&message))
    }

    /// The stimulus timeline so far, as shareable JSON — feed it back
    /// to [`Self::new`] to reproduce this run byte-for-byte.
    #[must_use]
    pub fn timeline_json(&self) -> String {
        self.inner.timeline_json()
    }
}
