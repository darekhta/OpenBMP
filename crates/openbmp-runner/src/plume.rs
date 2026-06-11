//! Runner-side plume-similarity assembly and telemetry helpers.

use openbmp_core::ChannelId;
use openbmp_physics::atmosphere::AtmosphereSample;
use openbmp_plume::{PlumeClusterGeometry, PlumeFreestream, PlumeNozzle, PlumeState};
use openbmp_propulsion::SolidMotor;
use openbmp_scenario::{AeroPlumeConfig, ScenarioDocument};
use openbmp_state::PointMassState;
use openbmp_telemetry::{ChannelMetadata, TelemetryChannel, TelemetryRow};

use crate::RunnerError;
use crate::atmosphere::{is_runtime_atmosphere_kind, scenario_atmosphere_kind};

/// Opt-in telemetry channels for live plume similarity state.
#[derive(Debug)]
pub(crate) struct PlumeTelemetryChannels {
    nozzle_pressure_ratio: TelemetryChannel<f64>,
    exit_pressure_ratio: TelemetryChannel<f64>,
    thrust_coefficient: TelemetryChannel<f64>,
    momentum_flux_ratio: TelemetryChannel<f64>,
    initial_turn_angle: TelemetryChannel<f64>,
    merge_distance: TelemetryChannel<f64>,
    cluster_merged: TelemetryChannel<bool>,
    pifs_onset: TelemetryChannel<bool>,
}

impl PlumeTelemetryChannels {
    pub(crate) fn new<F>(alloc: &mut F) -> Result<Self, RunnerError>
    where
        F: FnMut() -> ChannelId,
    {
        Ok(Self {
            nozzle_pressure_ratio: TelemetryChannel::<f64>::new(
                alloc(),
                "plume.nozzle_pressure_ratio",
                "1",
                None::<&str>,
            )?,
            exit_pressure_ratio: TelemetryChannel::<f64>::new(
                alloc(),
                "plume.exit_pressure_ratio",
                "1",
                None::<&str>,
            )?,
            thrust_coefficient: TelemetryChannel::<f64>::new(
                alloc(),
                "plume.thrust_coefficient",
                "1",
                None::<&str>,
            )?,
            momentum_flux_ratio: TelemetryChannel::<f64>::new(
                alloc(),
                "plume.momentum_flux_ratio",
                "1",
                None::<&str>,
            )?,
            initial_turn_angle: TelemetryChannel::<f64>::new(
                alloc(),
                "plume.initial_turn_angle_rad",
                "rad",
                None::<&str>,
            )?,
            merge_distance: TelemetryChannel::<f64>::new(
                alloc(),
                "plume.merge_distance_m",
                "m",
                None::<&str>,
            )?,
            cluster_merged: TelemetryChannel::<bool>::new(
                alloc(),
                "plume.cluster_merged",
                "bool",
                None::<&str>,
            )?,
            pifs_onset: TelemetryChannel::<bool>::new(
                alloc(),
                "plume.pifs_onset",
                "bool",
                None::<&str>,
            )?,
        })
    }

    pub(crate) fn push_metadata(&self, channels: &mut Vec<ChannelMetadata>) {
        channels.push(self.nozzle_pressure_ratio.metadata().clone());
        channels.push(self.exit_pressure_ratio.metadata().clone());
        channels.push(self.thrust_coefficient.metadata().clone());
        channels.push(self.momentum_flux_ratio.metadata().clone());
        channels.push(self.initial_turn_angle.metadata().clone());
        channels.push(self.merge_distance.metadata().clone());
        channels.push(self.cluster_merged.metadata().clone());
        channels.push(self.pifs_onset.metadata().clone());
    }

    pub(crate) fn insert(
        &self,
        row: &mut TelemetryRow,
        state: Option<PlumeState>,
    ) -> Result<(), RunnerError> {
        row.insert(
            &self.nozzle_pressure_ratio,
            state.map_or(0.0, |state| state.nozzle_pressure_ratio),
        )?;
        row.insert(
            &self.exit_pressure_ratio,
            state.map_or(0.0, |state| state.exit_pressure_ratio),
        )?;
        row.insert(
            &self.thrust_coefficient,
            state.map_or(0.0, |state| state.thrust_coefficient),
        )?;
        row.insert(
            &self.momentum_flux_ratio,
            state.map_or(0.0, |state| state.momentum_flux_ratio),
        )?;
        row.insert(
            &self.initial_turn_angle,
            state.map_or(0.0, |state| state.initial_turn_angle_rad),
        )?;
        row.insert(
            &self.merge_distance,
            state
                .and_then(|state| state.merge_distance_m)
                .unwrap_or(0.0),
        )?;
        row.insert(
            &self.cluster_merged,
            state.is_some_and(|state| state.cluster_merged),
        )?;
        row.insert(
            &self.pifs_onset,
            state.is_some_and(|state| state.pifs_onset),
        )?;
        Ok(())
    }
}

/// Point-mass plume evaluator for the legacy single solid-motor path.
#[derive(Debug)]
pub(crate) struct PointMassPlumeEvaluator<'a> {
    motor: &'a SolidMotor,
    ignition_time_s: f64,
    reference_area_m2: f64,
    geometry: PlumeClusterGeometry,
}

impl<'a> PointMassPlumeEvaluator<'a> {
    pub(crate) fn maybe_new(
        document: &ScenarioDocument,
        motor: Option<&'a SolidMotor>,
    ) -> Result<Option<Self>, RunnerError> {
        let Some(config) = document.aero.as_ref().and_then(|aero| aero.plume.as_ref()) else {
            return Ok(None);
        };
        if !is_runtime_atmosphere_kind(scenario_atmosphere_kind(document)) {
            return Err(RunnerError::UnsupportedScenario {
                what: "[aero.plume] requires a runner-sampled atmosphere".to_owned(),
            });
        }
        let motor = motor.ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "[aero.plume] point-mass telemetry currently requires [propulsion.motor]"
                .to_owned(),
        })?;
        let ignition_time_s = document.time.start_s
            + document
                .propulsion
                .as_ref()
                .and_then(|propulsion| propulsion.motor.as_ref())
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "[aero.plume] point-mass telemetry currently requires [propulsion.motor]"
                        .to_owned(),
                })?
                .ignite_at_s;
        Ok(Some(Self {
            motor,
            ignition_time_s,
            reference_area_m2: config.reference_area_m2,
            geometry: plume_geometry(config),
        }))
    }

    pub(crate) fn evaluate(
        &self,
        state: &PointMassState,
        atmosphere: Option<AtmosphereSample>,
    ) -> Result<Option<PlumeState>, RunnerError> {
        let Some(atmosphere) = atmosphere else {
            return Ok(None);
        };
        if atmosphere.pressure_pa <= 0.0 || atmosphere.density_kg_m3 <= 0.0 {
            return Ok(None);
        }
        let speed_m_s = state.velocity.vector.norm();
        let dynamic_pressure_pa = 0.5 * atmosphere.density_kg_m3 * speed_m_s * speed_m_s;
        if !dynamic_pressure_pa.is_finite() || dynamic_pressure_pa <= 0.0 {
            return Ok(None);
        }
        let t_since_ignition_s = state.time.as_seconds() - self.ignition_time_s;
        let Some(chamber) = self.motor.chamber_state_at(t_since_ignition_s)? else {
            return Ok(None);
        };
        let Some(solution) = self
            .motor
            .nozzle_solution_at(t_since_ignition_s, atmosphere.pressure_pa)?
        else {
            return Ok(None);
        };
        let freestream = PlumeFreestream {
            ambient_pressure_pa: atmosphere.pressure_pa,
            dynamic_pressure_pa,
            reference_area_m2: self.reference_area_m2,
        };
        let nozzle =
            PlumeNozzle::from_nozzle_solution(chamber.chamber_pressure_pa, chamber.gamma, solution);
        PlumeState::from_inputs(freestream, nozzle, self.geometry)
            .map(Some)
            .map_err(RunnerError::from)
    }
}

fn plume_geometry(config: &AeroPlumeConfig) -> PlumeClusterGeometry {
    PlumeClusterGeometry {
        engine_count: config.engine_count,
        exit_area_total_m2: config.exit_area_total_m2,
        base_area_m2: config.base_area_m2,
        center_spacing_m: config.center_spacing_m,
        merge_evaluation_distance_m: config.merge_evaluation_distance_m,
        pifs_onset_angle_rad: config.pifs_onset_angle_rad,
    }
}
