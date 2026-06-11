//! Runner-side plume-similarity assembly and telemetry helpers.

use std::collections::BTreeMap;

use openbmp_core::{Body, ChannelId, EngineId, Position3};
use openbmp_physics::atmosphere::AtmosphereSample;
use openbmp_plume::{PlumeClusterGeometry, PlumeFreestream, PlumeNozzle, PlumeState};
use openbmp_propulsion::{
    ChamberState, IdealNozzlePerformance, NozzlePerformance, NozzleSeparationCriterion, SolidMotor,
};
use openbmp_scenario::{AeroPlumeConfig, ScenarioDocument};
use openbmp_state::{PointMassState, RigidBodyState};
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

/// Static plume metadata retained for thermochemical liquid engines.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct LiquidPlumeEngine {
    /// Nominal chamber pressure in Pa at the thermochemistry lookup point.
    pub(crate) chamber_pressure_pa: f64,
    /// Exhaust specific-heat ratio.
    pub(crate) gamma: f64,
    /// Nozzle throat area in m^2.
    pub(crate) throat_area_m2: f64,
    /// Nozzle exit area in m^2.
    pub(crate) exit_area_m2: f64,
    /// Nominal full-throttle mass flow in kg/s.
    pub(crate) nominal_mass_flow_kg_per_s: f64,
    /// Optional overexpanded-flow separation criterion.
    pub(crate) separation: NozzleSeparationCriterion,
}

impl LiquidPlumeEngine {
    fn solve_at_snapshot(
        self,
        snapshot: &openbmp_sim::EngineSnapshot,
        ambient_pressure_pa: f64,
    ) -> Result<Option<NozzleSolutionSample>, RunnerError> {
        let mass_flow_kg_s = snapshot.mass_flow_kg_per_s;
        if mass_flow_kg_s <= 0.0 || snapshot.thrust_body.norm() <= 0.0 {
            return Ok(None);
        }
        let pressure_scale = mass_flow_kg_s / self.nominal_mass_flow_kg_per_s;
        if !pressure_scale.is_finite() || pressure_scale <= 0.0 {
            return Ok(None);
        }
        let chamber = ChamberState {
            chamber_pressure_pa: self.chamber_pressure_pa * pressure_scale,
            mass_flow_kg_s,
            gamma: self.gamma,
            throat_area_m2: self.throat_area_m2,
            exit_area_m2: self.exit_area_m2,
        };
        let solution = IdealNozzlePerformance::new(self.separation)
            .solve(chamber, ambient_pressure_pa)
            .map_err(RunnerError::from)?;
        Ok(Some(NozzleSolutionSample {
            chamber_pressure_pa: chamber.chamber_pressure_pa,
            gamma: chamber.gamma,
            exit_area_m2: chamber.exit_area_m2,
            total_thrust_n: solution.total_thrust_n,
            momentum_thrust_n: solution.momentum_thrust_n,
            exit_pressure_pa: solution.exit_pressure_pa,
            exit_mach: solution.exit_mach,
        }))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct NozzleSolutionSample {
    chamber_pressure_pa: f64,
    gamma: f64,
    exit_area_m2: f64,
    total_thrust_n: f64,
    momentum_thrust_n: f64,
    exit_pressure_pa: f64,
    exit_mach: f64,
}

/// Rigid-body plume evaluator for thermochemical liquid-engine clusters.
#[derive(Debug)]
pub(crate) struct RigidPlumeEvaluator {
    engines: BTreeMap<EngineId, RigidPlumeEngine>,
    reference_area_m2: f64,
    base_area_m2: f64,
    merge_evaluation_distance_m: f64,
    pifs_onset_angle_rad: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct RigidPlumeEngine {
    nozzle: LiquidPlumeEngine,
    mount_point_body_m: [f64; 3],
}

impl RigidPlumeEvaluator {
    pub(crate) fn maybe_new(
        document: &ScenarioDocument,
        engines: &BTreeMap<EngineId, LiquidPlumeEngine>,
        engine_ids: &[EngineId],
        mount_points_body: &[Position3<Body>],
    ) -> Result<Option<Self>, RunnerError> {
        let Some(config) = document.aero.as_ref().and_then(|aero| aero.plume.as_ref()) else {
            return Ok(None);
        };
        if engine_ids.len() != mount_points_body.len() {
            return Err(RunnerError::UnsupportedScenario {
                what: "[aero.plume] rigid-body telemetry requires aligned engine ids and mount \
                       points"
                    .to_owned(),
            });
        }
        if !is_runtime_atmosphere_kind(scenario_atmosphere_kind(document)) {
            return Err(RunnerError::UnsupportedScenario {
                what: "[aero.plume] requires a runner-sampled atmosphere".to_owned(),
            });
        }
        if document.vehicle.assembly.engines.is_empty() {
            return Err(RunnerError::UnsupportedScenario {
                what: "[aero.plume] rigid-body telemetry requires liquid engines".to_owned(),
            });
        }
        for config in &document.vehicle.assembly.engines {
            let id = EngineId::from_path(&format!("vehicle.assembly.engines.{id}", id = config.id));
            if !engines.contains_key(&id) {
                return Err(RunnerError::UnsupportedScenario {
                    what:
                        "[aero.plume] rigid-body telemetry currently requires every liquid engine \
                           to declare thermochemical_performance"
                            .to_owned(),
                });
            }
        }
        let mount_points_by_id: BTreeMap<EngineId, [f64; 3]> = engine_ids
            .iter()
            .copied()
            .zip(mount_points_body.iter().map(position_to_array))
            .collect();
        let mut rigid_engines = BTreeMap::new();
        for config in &document.vehicle.assembly.engines {
            let id = EngineId::from_path(&format!("vehicle.assembly.engines.{id}", id = config.id));
            let Some(nozzle) = engines.get(&id).copied() else {
                return Err(RunnerError::UnsupportedScenario {
                    what:
                        "[aero.plume] rigid-body telemetry currently requires every liquid engine \
                           to declare thermochemical_performance"
                            .to_owned(),
                });
            };
            let mount_point_body_m = mount_points_by_id.get(&id).copied().ok_or_else(|| {
                RunnerError::UnsupportedScenario {
                    what: "[aero.plume] rigid-body telemetry could not resolve an engine mount \
                               point"
                        .to_owned(),
                }
            })?;
            rigid_engines.insert(
                id,
                RigidPlumeEngine {
                    nozzle,
                    mount_point_body_m,
                },
            );
        }
        Ok(Some(Self {
            engines: rigid_engines,
            reference_area_m2: config.reference_area_m2,
            base_area_m2: config.base_area_m2,
            merge_evaluation_distance_m: config.merge_evaluation_distance_m,
            pifs_onset_angle_rad: config.pifs_onset_angle_rad,
        }))
    }

    pub(crate) fn evaluate(
        &self,
        state: &RigidBodyState,
        atmosphere: Option<AtmosphereSample>,
        engine_snapshot: &BTreeMap<EngineId, openbmp_sim::EngineSnapshot>,
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

        let mut aggregate = AggregateNozzle::default();
        for (id, plume) in &self.engines {
            let Some(snapshot) = engine_snapshot.get(id) else {
                continue;
            };
            if let Some(sample) = plume
                .nozzle
                .solve_at_snapshot(snapshot, atmosphere.pressure_pa)?
            {
                aggregate.add(sample, plume.mount_point_body_m);
            }
        }
        let Some((nozzle, geometry)) = aggregate.finish(self)? else {
            return Ok(None);
        };
        let freestream = PlumeFreestream {
            ambient_pressure_pa: atmosphere.pressure_pa,
            dynamic_pressure_pa,
            reference_area_m2: self.reference_area_m2,
        };
        PlumeState::from_inputs(freestream, nozzle, geometry)
            .map(Some)
            .map_err(RunnerError::from)
    }
}

#[derive(Debug, Default)]
struct AggregateNozzle {
    active_count: u32,
    total_weight_n: f64,
    total_thrust_n: f64,
    momentum_thrust_n: f64,
    exit_area_total_m2: f64,
    mount_points_body_m: Vec<[f64; 3]>,
    chamber_pressure_weighted_pa_n: f64,
    exit_pressure_weighted_pa_n: f64,
    exit_mach_weighted_n: f64,
    gamma_weighted_n: f64,
}

impl AggregateNozzle {
    fn add(&mut self, sample: NozzleSolutionSample, mount_point_body_m: [f64; 3]) {
        let weight = sample.total_thrust_n.max(0.0);
        if weight == 0.0 {
            return;
        }
        self.active_count = self.active_count.saturating_add(1);
        self.total_weight_n += weight;
        self.total_thrust_n += sample.total_thrust_n;
        self.momentum_thrust_n += sample.momentum_thrust_n;
        self.exit_area_total_m2 += sample.exit_area_m2;
        self.mount_points_body_m.push(mount_point_body_m);
        self.chamber_pressure_weighted_pa_n += sample.chamber_pressure_pa * weight;
        self.exit_pressure_weighted_pa_n += sample.exit_pressure_pa * weight;
        self.exit_mach_weighted_n += sample.exit_mach * weight;
        self.gamma_weighted_n += sample.gamma * weight;
    }

    fn finish(
        self,
        evaluator: &RigidPlumeEvaluator,
    ) -> Result<Option<(PlumeNozzle, PlumeClusterGeometry)>, RunnerError> {
        if self.active_count == 0 || self.total_weight_n <= 0.0 {
            return Ok(None);
        }
        let nozzle = PlumeNozzle {
            chamber_pressure_pa: self.chamber_pressure_weighted_pa_n / self.total_weight_n,
            exit_pressure_pa: self.exit_pressure_weighted_pa_n / self.total_weight_n,
            exit_mach: self.exit_mach_weighted_n / self.total_weight_n,
            gamma: self.gamma_weighted_n / self.total_weight_n,
            total_thrust_n: self.total_thrust_n,
            momentum_thrust_n: self.momentum_thrust_n,
        };
        let center_spacing_m = active_center_spacing_m(&self.mount_points_body_m).unwrap_or(0.0);
        let geometry = PlumeClusterGeometry {
            engine_count: self.active_count,
            exit_area_total_m2: self.exit_area_total_m2,
            base_area_m2: evaluator.base_area_m2,
            center_spacing_m,
            merge_evaluation_distance_m: evaluator.merge_evaluation_distance_m,
            pifs_onset_angle_rad: evaluator.pifs_onset_angle_rad,
        };
        nozzle.validate()?;
        geometry.validate()?;
        Ok(Some((nozzle, geometry)))
    }
}

fn position_to_array(position: &Position3<Body>) -> [f64; 3] {
    [position.vector.x, position.vector.y, position.vector.z]
}

fn active_center_spacing_m(mount_points_body_m: &[[f64; 3]]) -> Option<f64> {
    if mount_points_body_m.len() < 2 {
        return None;
    }
    let mut min_spacing_m = f64::INFINITY;
    for (index, a) in mount_points_body_m.iter().enumerate() {
        for b in mount_points_body_m.iter().skip(index + 1) {
            let dx = a[0] - b[0];
            let dy = a[1] - b[1];
            let dz = a[2] - b[2];
            let spacing_m = (dx * dx + dy * dy + dz * dz).sqrt();
            min_spacing_m = min_spacing_m.min(spacing_m);
        }
    }
    min_spacing_m.is_finite().then_some(min_spacing_m)
}
