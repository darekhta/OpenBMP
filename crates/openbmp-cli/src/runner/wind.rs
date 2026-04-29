//! Phase-3.8 runner-side wind rack.
//!
//! The rack owns the scenario-resolved wind model (one of `NoWind`,
//! `ConstantWind`, `LayeredWind`, or `GustWind`) and orchestrates
//! per-step state updates plus kernel snapshot pushes. Mirrors the
//! Phase-3.6 `EngineRack` and Phase-3.7 `TankRack` patterns.
//!
//! Each kernel base tick the runner:
//!
//! 1. Calls [`WindRack::advance`] to step the gust filter (only
//!    meaningful for `GustWind`; the time-only models are no-ops).
//! 2. Calls [`WindRack::sample`] to query the current wind value
//!    given the kernel's pre-step state.
//! 3. Pushes the result into the kernel via
//!    `kernel.set_wind_sample(...)` so all four RK4 stages observe
//!    the same wind.
//!
//! Phase-3.8 leaves the kernel hot path's force/moment/mass
//! evaluation byte-identical for legacy scenarios: when no `[wind]`
//! block is declared (or `kind = "none"`), the runner builds an
//! `Inactive` rack, the per-step setter is never called, and the
//! kernel's `wind_sample_override` stays `None` — the kernel keeps
//! the `EnvironmentSample::default()` zero wind.
//!
//! # Determinism
//!
//! - The Dryden filter state lives inside the [`GustWind`]'s
//!   `Cell<f64>` per-axis cells; the rack's `advance` writes them
//!   exactly once per kernel base tick. The kernel's RK4 stages
//!   read via [`openbmp_env::WindModel::wind_ned_m_s`] and all see
//!   the same value.
//! - `WindRack::reset` is called once at scenario start so reruns
//!   produce bit-identical sample streams from step 0.
//! - The wind sample uses the kernel's pre-step state (position +
//!   frame + time); the rack does not query the integrator's
//!   sub-step intermediates — matches the rack-pattern convention
//!   where one kernel base tick = one rack tick.

use nalgebra::Vector3;
use openbmp_core::{Eci, FrameContext, Position3, SimTime, StepIndex};
use openbmp_env::{ConstantWind, GustWind, GustWindParams, LayerEntry, LayeredWind, WindModel};
use openbmp_scenario::{ScenarioDocument, WindConfig};

use crate::error::CliError;

/// Runner-side wind rack. Built once per `openbmp run` invocation;
/// consumed by the per-step kernel loop.
#[derive(Debug)]
pub enum WindRack {
    /// No wind active — `[wind]` block missing or `kind = "none"`.
    /// The runner skips every per-step rack operation; the kernel's
    /// `wind_sample_override` stays `None` and `EnvironmentSample`
    /// reports zero wind.
    Inactive,
    /// Phase-2.4 constant-everywhere wind.
    Constant(ConstantWind),
    /// Phase-3.8.B per-altitude NED wind table.
    Layered(LayeredWind),
    /// Phase-3.8.C Dryden gust filter. Carries internal filter
    /// state advanced per-step.
    Gust(GustWind),
}

impl WindRack {
    /// Build the rack from a parsed scenario document.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::UnsupportedScenario`] when the wind
    /// kind is unknown (the scenario validator should have caught
    /// this earlier — this is a defensive arm) or when a model
    /// constructor rejects the parsed parameters.
    pub fn build(document: &ScenarioDocument) -> Result<Self, CliError> {
        let Some(wind) = document.wind.as_ref() else {
            return Ok(Self::Inactive);
        };
        match wind.kind.as_str() {
            "none" => Ok(Self::Inactive),
            "constant" => Self::build_constant(wind),
            "layered" => Self::build_layered(wind),
            "gust" => Self::build_gust(wind, document.time.dt_s, document.time.seed),
            other => Err(CliError::UnsupportedScenario {
                what: format!("unsupported [wind].kind = \"{other}\""),
            }),
        }
    }

    fn build_constant(wind: &WindConfig) -> Result<Self, CliError> {
        let v = wind
            .wind_ned_m_s
            .ok_or_else(|| CliError::UnsupportedScenario {
                what: "[wind].kind = \"constant\" requires wind_ned_m_s".to_owned(),
            })?;
        let model =
            ConstantWind::new(v[0], v[1], v[2]).map_err(|err| CliError::UnsupportedScenario {
                what: format!("ConstantWind construction failed: {err}"),
            })?;
        Ok(Self::Constant(model))
    }

    fn build_layered(wind: &WindConfig) -> Result<Self, CliError> {
        let layers = wind
            .layers
            .as_ref()
            .ok_or_else(|| CliError::UnsupportedScenario {
                what: "[wind].kind = \"layered\" requires layers".to_owned(),
            })?;
        let mut entries: Vec<LayerEntry> = Vec::with_capacity(layers.len());
        for layer in layers {
            entries.push(LayerEntry::new(
                layer.altitude_m,
                layer.wind_ned_m_s[0],
                layer.wind_ned_m_s[1],
                layer.wind_ned_m_s[2],
            ));
        }
        let model = LayeredWind::new(entries).map_err(|err| CliError::UnsupportedScenario {
            what: format!("LayeredWind construction failed: {err}"),
        })?;
        Ok(Self::Layered(model))
    }

    fn build_gust(wind: &WindConfig, dt_s: f64, scenario_seed: u64) -> Result<Self, CliError> {
        let intensity = wind
            .intensity_m_s
            .ok_or_else(|| CliError::UnsupportedScenario {
                what: "[wind].kind = \"gust\" requires intensity_m_s".to_owned(),
            })?;
        let length_scale = wind
            .length_scale_m
            .ok_or_else(|| CliError::UnsupportedScenario {
                what: "[wind].kind = \"gust\" requires length_scale_m".to_owned(),
            })?;
        let airspeed_m_s = wind
            .airspeed_m_s
            .ok_or_else(|| CliError::UnsupportedScenario {
                what: "[wind].kind = \"gust\" requires airspeed_m_s".to_owned(),
            })?;
        let mean_wind_ned_m_s = wind.mean_wind_ned_m_s.unwrap_or([0.0, 0.0, 0.0]);
        let params = GustWindParams {
            intensity_m_s: intensity,
            length_scale_m: length_scale,
            airspeed_m_s,
            mean_wind_ned_m_s,
            dt_s,
            scenario_seed,
        };
        let model = GustWind::new(params).map_err(|err| CliError::UnsupportedScenario {
            what: format!("GustWind construction failed: {err}"),
        })?;
        Ok(Self::Gust(model))
    }

    /// `true` when the rack is inactive (legacy / `none` scenarios).
    /// The runner short-circuits every per-step rack op on this so
    /// pre-3.8 byte output is preserved.
    #[must_use]
    pub fn is_inactive(&self) -> bool {
        matches!(self, Self::Inactive)
    }

    /// Reset rack-internal state to a fresh start. Called once at
    /// scenario start so reruns produce bit-identical streams.
    pub fn reset(&self) {
        if let Self::Gust(gust) = self {
            gust.reset();
        }
    }

    /// Advance per-step rack state for the given step index. For
    /// `GustWind` this advances the Dryden filter; the time-only
    /// models are no-ops.
    pub fn advance(&self, step: StepIndex) {
        if let Self::Gust(gust) = self {
            gust.advance(step);
        }
    }

    /// Query the current wind sample at the kernel's pre-step state.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Env`] when the underlying wind model
    /// rejects the query (non-finite altitude for `LayeredWind`,
    /// non-finite filter output for `GustWind`).
    pub fn sample(
        &self,
        position_eci: Position3<Eci>,
        frame: &FrameContext,
        time: SimTime,
    ) -> Result<Vector3<f64>, CliError> {
        match self {
            Self::Inactive => Ok(Vector3::zeros()),
            Self::Constant(c) => Ok(c.wind_ned_m_s(position_eci, frame, time)?.vector),
            Self::Layered(l) => Ok(l.wind_ned_m_s(position_eci, frame, time)?.vector),
            Self::Gust(g) => Ok(g.wind_ned_m_s(position_eci, frame, time)?.vector),
        }
    }
}
