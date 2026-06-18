//! Runner-side AFTS rule-table execution and evidence reporting.
//!
//! The runner treats `[afts]` as an observer over truth state. It samples the
//! same post-step states that telemetry records, evaluates the forward IIP
//! containment rules in `openbmp-afts`, and publishes a sidecar report in
//! [`crate::RunOutcome`]. It does not mutate kernel state or replace the
//! kernel-owned [`openbmp_sim::StopReason`].

use openbmp_afts::{
    AftsError, AftsMonitor, ContainmentPolygon, ContainmentRule, ContainmentRuleKind,
    CorridorMetric, CorridorRule, ExponentialAtmosphericDrag, FlightCorridorSample,
    GateCrossingDirection, GateRule, GateSegment, IipImpactSurface, IipPropagator, ImpactPoint,
    LatLon, ZoneColor, ZoneRule, evaluate_zone_rules,
};
use openbmp_core::{Eci, Position3, Velocity3};
use openbmp_physics::{WGS84_MU_M3_S2, WGS84_OMEGA_RAD_S};
use openbmp_scenario::{
    AftsCorridorMetricConfig, AftsCorridorRuleConfig, AftsGateDirectionConfig, AftsGateRuleConfig,
    AftsKeepInsideRuleConfig, AftsPropagatorConfig, AftsPropagatorEarthRotationConfig,
    AftsPropagatorSurfaceConfig, AftsZoneColorConfig, AftsZoneRuleConfig, ScenarioDocument,
};
use openbmp_state::{PointMassState, RigidBodyState};
use serde::{Deserialize, Serialize};

/// One scenario-declared AFTS rule and its provenance label.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct AftsRuleReport {
    /// Rule identifier from `[[afts.keep_inside]]` or `[[afts.keep_out]]`.
    pub id: String,
    /// Rule kind: `keep_inside` or `keep_out`.
    pub kind: String,
    /// Scenario-declared evidence/provenance string for the rule table.
    pub evidence: String,
    /// Optional scenario-resolved evidence file path as declared by the rule.
    pub evidence_file: Option<String>,
    /// Optional scenario-declared SHA-256 pin for the evidence file.
    pub evidence_file_sha256: Option<String>,
    /// Corridor metric name for scalar corridor rules.
    pub metric: Option<String>,
    /// Inclusive lower bound for scalar corridor rules.
    pub min_value: Option<f64>,
    /// Inclusive upper bound for scalar corridor rules.
    pub max_value: Option<f64>,
    /// Direction filter for geospatial gate rules.
    pub direction: Option<String>,
    /// Zone color for green/red zone rules.
    pub zone_color: Option<String>,
}

/// AFTS sidecar report emitted by a runner outcome.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct AftsRunReport {
    /// Number of telemetry-aligned state samples observed by the monitor.
    pub samples: u64,
    /// Whether the forward containment monitor latched termination.
    pub terminate: bool,
    /// First firing rule id, if termination latched.
    pub rule_id: Option<String>,
    /// Evidence string associated with the first firing rule.
    pub rule_evidence: Option<String>,
    /// Step at which the first rule fired.
    pub first_trigger_step: Option<u64>,
    /// Simulation time at which the first rule fired.
    pub first_trigger_time_s: Option<f64>,
    /// Latest sampled IIP latitude, in radians.
    pub impact_latitude_rad: Option<f64>,
    /// Latest sampled IIP longitude, in radians.
    pub impact_longitude_rad: Option<f64>,
    /// Latest sampled forward time to impact, in seconds.
    pub time_to_impact_s: Option<f64>,
    /// Latest sampled geocentric altitude above the AFTS surface radius.
    pub altitude_m: Option<f64>,
    /// Latest sampled inertial speed magnitude.
    pub speed_m_s: Option<f64>,
    /// Latest sampled flight-path angle from local horizontal.
    pub flight_path_angle_rad: Option<f64>,
    /// Rule-table evidence copied from the scenario.
    pub rule_table: Vec<AftsRuleReport>,
}

#[derive(Clone, Debug)]
struct RunnerAftsRule {
    id: String,
    kind: ContainmentRuleKind,
    evidence: String,
    vertices: Vec<LatLon>,
}

#[derive(Clone, Debug)]
struct RunnerAftsCorridorRule {
    id: String,
    evidence: String,
    metric: CorridorMetric,
    min_value: Option<f64>,
    max_value: Option<f64>,
}

#[derive(Clone, Debug)]
struct RunnerAftsGateRule {
    id: String,
    evidence: String,
    start: LatLon,
    end: LatLon,
    direction: GateCrossingDirection,
}

#[derive(Clone, Debug)]
struct RunnerAftsZoneRule {
    id: String,
    evidence: String,
    color: ZoneColor,
    vertices: Vec<LatLon>,
}

/// Runtime AFTS observer owned by the runner loop.
#[derive(Clone, Debug)]
pub(crate) struct RunnerAftsMonitor {
    propagator: IipPropagator,
    rules: Vec<RunnerAftsRule>,
    corridor_rules: Vec<RunnerAftsCorridorRule>,
    gate_rules: Vec<RunnerAftsGateRule>,
    zone_rules: Vec<RunnerAftsZoneRule>,
    previous_impact: Option<LatLon>,
    report: AftsRunReport,
}

impl RunnerAftsMonitor {
    /// Builds an AFTS observer from the scenario, returning `None` when the
    /// optional block is absent or explicitly disabled.
    pub(crate) fn maybe_new(document: &ScenarioDocument) -> Result<Option<Self>, AftsError> {
        let Some(config) = &document.afts else {
            return Ok(None);
        };
        if !config.enabled {
            return Ok(None);
        }

        let total_rules = config.keep_inside.len()
            + config.keep_out.len()
            + config.corridor.len()
            + config.gate.len()
            + config.zone.len();
        let mut rules = Vec::with_capacity(config.keep_inside.len() + config.keep_out.len());
        let mut corridor_rules = Vec::with_capacity(config.corridor.len());
        let mut gate_rules = Vec::with_capacity(config.gate.len());
        let mut zone_rules = Vec::with_capacity(config.zone.len());
        let mut rule_table = Vec::with_capacity(total_rules);
        for rule in &config.keep_inside {
            push_rule(
                rule,
                ContainmentRuleKind::KeepInside,
                &mut rules,
                &mut rule_table,
            )?;
        }
        for rule in &config.keep_out {
            push_rule(
                rule,
                ContainmentRuleKind::KeepOut,
                &mut rules,
                &mut rule_table,
            )?;
        }
        for rule in &config.corridor {
            push_corridor_rule(rule, &mut corridor_rules, &mut rule_table)?;
        }
        for rule in &config.gate {
            push_gate_rule(rule, &mut gate_rules, &mut rule_table)?;
        }
        for rule in &config.zone {
            push_zone_rule(rule, &mut zone_rules, &mut rule_table)?;
        }
        if rules.is_empty()
            && corridor_rules.is_empty()
            && gate_rules.is_empty()
            && zone_rules.is_empty()
        {
            return Err(AftsError::InvalidPolygon {
                field: "rules",
                reason: "must contain at least one containment rule",
            });
        }

        Ok(Some(Self {
            propagator: propagator_for_document(document, &config.propagator)?,
            rules,
            corridor_rules,
            gate_rules,
            zone_rules,
            previous_impact: None,
            report: AftsRunReport {
                samples: 0,
                terminate: false,
                rule_id: None,
                rule_evidence: None,
                first_trigger_step: None,
                first_trigger_time_s: None,
                impact_latitude_rad: None,
                impact_longitude_rad: None,
                time_to_impact_s: None,
                altitude_m: None,
                speed_m_s: None,
                flight_path_angle_rad: None,
                rule_table,
            },
        }))
    }

    /// Observes a point-mass truth state.
    pub(crate) fn observe_point_mass(
        &mut self,
        step: u64,
        time_s: f64,
        state: &PointMassState,
    ) -> Result<(), AftsError> {
        if self.report.terminate {
            self.report.samples += 1;
            return Ok(());
        }
        self.observe_position_velocity(step, time_s, state.position, state.velocity)
    }

    /// Observes a rigid-body truth state projected to position/velocity.
    pub(crate) fn observe_rigid_body(
        &mut self,
        step: u64,
        time_s: f64,
        state: &RigidBodyState,
    ) -> Result<(), AftsError> {
        if self.report.terminate {
            self.report.samples += 1;
            return Ok(());
        }
        self.observe_position_velocity(step, time_s, state.position, state.velocity)
    }

    fn observe_position_velocity(
        &mut self,
        step: u64,
        time_s: f64,
        position: Position3<Eci>,
        velocity: Velocity3<Eci>,
    ) -> Result<(), AftsError> {
        let impact =
            if self.rules.is_empty() && self.gate_rules.is_empty() && self.zone_rules.is_empty() {
                None
            } else {
                Some(
                    self.propagator
                        .impact_point_from_position_velocity(position, velocity)?,
                )
            };
        let corridor_sample = if self.corridor_rules.is_empty() {
            None
        } else {
            Some(self.propagator.flight_corridor_sample(position, velocity)?)
        };
        self.observe_sample(step, time_s, impact, corridor_sample)
    }

    fn observe_sample(
        &mut self,
        step: u64,
        time_s: f64,
        impact: Option<ImpactPoint>,
        corridor_sample: Option<FlightCorridorSample>,
    ) -> Result<(), AftsError> {
        self.report.samples += 1;
        if let Some(impact) = impact {
            self.report.impact_latitude_rad = Some(impact.point.latitude_rad);
            self.report.impact_longitude_rad = Some(impact.point.longitude_rad);
            self.report.time_to_impact_s = Some(impact.time_to_impact_s);
        }

        if let Some(impact) = impact
            && !self.rules.is_empty()
        {
            let mut rule_views = Vec::with_capacity(self.rules.len());
            for rule in &self.rules {
                let polygon = ContainmentPolygon::new(&rule.id, &rule.vertices)?;
                rule_views.push(match rule.kind {
                    ContainmentRuleKind::KeepInside => {
                        ContainmentRule::keep_inside(&rule.id, polygon)?
                    }
                    ContainmentRuleKind::KeepOut => ContainmentRule::keep_out(&rule.id, polygon)?,
                });
            }
            let mut monitor = AftsMonitor::new(&rule_views)?;
            let decision = monitor.evaluate_iip(impact);
            if decision.terminate {
                let Some(rule_id) = decision.rule_id.map(str::to_owned) else {
                    return Err(AftsError::InvalidPolygon {
                        field: "rules",
                        reason: "fired rule did not report an id",
                    });
                };
                self.latch_rule(step, time_s, &rule_id);
                return Ok(());
            }
        }

        if let Some(impact) = impact
            && !self.zone_rules.is_empty()
        {
            let mut zone_views = Vec::with_capacity(self.zone_rules.len());
            for rule in &self.zone_rules {
                let polygon = ContainmentPolygon::new(&rule.id, &rule.vertices)?;
                zone_views.push(ZoneRule::new(&rule.id, rule.color, polygon)?);
            }
            if let Some(rule_id) = evaluate_zone_rules(&zone_views, impact.point).map(str::to_owned)
            {
                self.latch_rule(step, time_s, &rule_id);
                return Ok(());
            }
        }

        if let Some(impact) = impact
            && !self.gate_rules.is_empty()
        {
            if let Some(previous) = self.previous_impact {
                for rule in &self.gate_rules {
                    let gate = GateSegment::new(&rule.id, rule.start, rule.end)?;
                    let gate_rule = GateRule::new(&rule.id, gate, rule.direction)?;
                    if gate_rule.fires(previous, impact.point) {
                        let rule_id = rule.id.clone();
                        self.previous_impact = Some(impact.point);
                        self.latch_rule(step, time_s, &rule_id);
                        return Ok(());
                    }
                }
            }
            self.previous_impact = Some(impact.point);
        }

        if let Some(sample) = corridor_sample {
            self.report.altitude_m = Some(sample.altitude_m);
            self.report.speed_m_s = Some(sample.speed_m_s);
            self.report.flight_path_angle_rad = Some(sample.flight_path_angle_rad);

            for rule in &self.corridor_rules {
                let corridor =
                    CorridorRule::new(&rule.id, rule.metric, rule.min_value, rule.max_value)?;
                if corridor.fires(sample) {
                    let rule_id = rule.id.clone();
                    self.latch_rule(step, time_s, &rule_id);
                    return Ok(());
                }
            }
        }
        Ok(())
    }

    fn latch_rule(&mut self, step: u64, time_s: f64, rule_id: &str) {
        self.report.terminate = true;
        self.report.rule_id = Some(rule_id.to_owned());
        self.report.rule_evidence = self.rule_evidence(rule_id);
        self.report.first_trigger_step = Some(step);
        self.report.first_trigger_time_s = Some(time_s);
    }

    fn rule_evidence(&self, rule_id: &str) -> Option<String> {
        self.rules
            .iter()
            .find(|rule| rule.id == rule_id)
            .map(|rule| rule.evidence.clone())
            .or_else(|| {
                self.corridor_rules
                    .iter()
                    .find(|rule| rule.id == rule_id)
                    .map(|rule| rule.evidence.clone())
            })
            .or_else(|| {
                self.gate_rules
                    .iter()
                    .find(|rule| rule.id == rule_id)
                    .map(|rule| rule.evidence.clone())
            })
            .or_else(|| {
                self.zone_rules
                    .iter()
                    .find(|rule| rule.id == rule_id)
                    .map(|rule| rule.evidence.clone())
            })
    }

    /// Consumes the observer and returns its report.
    #[must_use]
    pub(crate) fn finish(self) -> AftsRunReport {
        self.report
    }
}

fn push_rule(
    rule: &AftsKeepInsideRuleConfig,
    kind: ContainmentRuleKind,
    rules: &mut Vec<RunnerAftsRule>,
    rule_table: &mut Vec<AftsRuleReport>,
) -> Result<(), AftsError> {
    let mut vertices = Vec::with_capacity(rule.vertices.len());
    for vertex in &rule.vertices {
        vertices.push(LatLon::new_degrees(
            vertex.latitude_deg,
            vertex.longitude_deg,
        )?);
    }
    ContainmentPolygon::new(&rule.id, &vertices)?;
    rules.push(RunnerAftsRule {
        id: rule.id.clone(),
        kind,
        evidence: rule.evidence.clone(),
        vertices,
    });
    rule_table.push(AftsRuleReport {
        id: rule.id.clone(),
        kind: rule_kind_label(kind).to_owned(),
        evidence: rule.evidence.clone(),
        evidence_file: rule
            .evidence_file
            .as_ref()
            .map(|path| path.display().to_string()),
        evidence_file_sha256: rule.evidence_file_sha256.clone(),
        metric: None,
        min_value: None,
        max_value: None,
        direction: None,
        zone_color: None,
    });
    Ok(())
}

fn push_corridor_rule(
    rule: &AftsCorridorRuleConfig,
    rules: &mut Vec<RunnerAftsCorridorRule>,
    rule_table: &mut Vec<AftsRuleReport>,
) -> Result<(), AftsError> {
    let metric = corridor_metric(rule.metric);
    let (min_value, max_value) = corridor_rule_bounds(rule);
    CorridorRule::new(&rule.id, metric, min_value, max_value)?;
    rules.push(RunnerAftsCorridorRule {
        id: rule.id.clone(),
        evidence: rule.evidence.clone(),
        metric,
        min_value,
        max_value,
    });
    rule_table.push(AftsRuleReport {
        id: rule.id.clone(),
        kind: "corridor".to_owned(),
        evidence: rule.evidence.clone(),
        evidence_file: rule
            .evidence_file
            .as_ref()
            .map(|path| path.display().to_string()),
        evidence_file_sha256: rule.evidence_file_sha256.clone(),
        metric: Some(corridor_metric_label(metric).to_owned()),
        min_value,
        max_value,
        direction: None,
        zone_color: None,
    });
    Ok(())
}

fn push_gate_rule(
    rule: &AftsGateRuleConfig,
    rules: &mut Vec<RunnerAftsGateRule>,
    rule_table: &mut Vec<AftsRuleReport>,
) -> Result<(), AftsError> {
    let start = LatLon::new_degrees(rule.start.latitude_deg, rule.start.longitude_deg)?;
    let end = LatLon::new_degrees(rule.end.latitude_deg, rule.end.longitude_deg)?;
    GateSegment::new(&rule.id, start, end)?;
    let direction = gate_direction(rule.direction);
    rules.push(RunnerAftsGateRule {
        id: rule.id.clone(),
        evidence: rule.evidence.clone(),
        start,
        end,
        direction,
    });
    rule_table.push(AftsRuleReport {
        id: rule.id.clone(),
        kind: "gate".to_owned(),
        evidence: rule.evidence.clone(),
        evidence_file: rule
            .evidence_file
            .as_ref()
            .map(|path| path.display().to_string()),
        evidence_file_sha256: rule.evidence_file_sha256.clone(),
        metric: None,
        min_value: None,
        max_value: None,
        direction: Some(gate_direction_label(direction).to_owned()),
        zone_color: None,
    });
    Ok(())
}

fn push_zone_rule(
    rule: &AftsZoneRuleConfig,
    rules: &mut Vec<RunnerAftsZoneRule>,
    rule_table: &mut Vec<AftsRuleReport>,
) -> Result<(), AftsError> {
    let color = zone_color(rule.color);
    let mut vertices = Vec::with_capacity(rule.vertices.len());
    for vertex in &rule.vertices {
        vertices.push(LatLon::new_degrees(
            vertex.latitude_deg,
            vertex.longitude_deg,
        )?);
    }
    ContainmentPolygon::new(&rule.id, &vertices)?;
    rules.push(RunnerAftsZoneRule {
        id: rule.id.clone(),
        evidence: rule.evidence.clone(),
        color,
        vertices,
    });
    rule_table.push(AftsRuleReport {
        id: rule.id.clone(),
        kind: "zone".to_owned(),
        evidence: rule.evidence.clone(),
        evidence_file: rule
            .evidence_file
            .as_ref()
            .map(|path| path.display().to_string()),
        evidence_file_sha256: rule.evidence_file_sha256.clone(),
        metric: None,
        min_value: None,
        max_value: None,
        direction: None,
        zone_color: Some(zone_color_label(color).to_owned()),
    });
    Ok(())
}

fn rule_kind_label(kind: ContainmentRuleKind) -> &'static str {
    match kind {
        ContainmentRuleKind::KeepInside => "keep_inside",
        ContainmentRuleKind::KeepOut => "keep_out",
    }
}

fn corridor_metric(metric: AftsCorridorMetricConfig) -> CorridorMetric {
    match metric {
        AftsCorridorMetricConfig::AltitudeM => CorridorMetric::AltitudeM,
        AftsCorridorMetricConfig::SpeedMS => CorridorMetric::SpeedMS,
        AftsCorridorMetricConfig::FlightPathAngleRad => CorridorMetric::FlightPathAngleRad,
    }
}

fn corridor_metric_label(metric: CorridorMetric) -> &'static str {
    match metric {
        CorridorMetric::AltitudeM => "altitude_m",
        CorridorMetric::SpeedMS => "speed_m_s",
        CorridorMetric::FlightPathAngleRad => "flight_path_angle_rad",
    }
}

fn gate_direction(direction: AftsGateDirectionConfig) -> GateCrossingDirection {
    match direction {
        AftsGateDirectionConfig::Any => GateCrossingDirection::Any,
        AftsGateDirectionConfig::NegativeToPositive => GateCrossingDirection::NegativeToPositive,
        AftsGateDirectionConfig::PositiveToNegative => GateCrossingDirection::PositiveToNegative,
    }
}

fn gate_direction_label(direction: GateCrossingDirection) -> &'static str {
    match direction {
        GateCrossingDirection::Any => "any",
        GateCrossingDirection::NegativeToPositive => "negative_to_positive",
        GateCrossingDirection::PositiveToNegative => "positive_to_negative",
    }
}

fn zone_color(color: AftsZoneColorConfig) -> ZoneColor {
    match color {
        AftsZoneColorConfig::Green => ZoneColor::Green,
        AftsZoneColorConfig::Red => ZoneColor::Red,
    }
}

fn zone_color_label(color: ZoneColor) -> &'static str {
    match color {
        ZoneColor::Green => "green",
        ZoneColor::Red => "red",
    }
}

fn corridor_rule_bounds(rule: &AftsCorridorRuleConfig) -> (Option<f64>, Option<f64>) {
    match rule.metric {
        AftsCorridorMetricConfig::AltitudeM => (rule.min_altitude_m, rule.max_altitude_m),
        AftsCorridorMetricConfig::SpeedMS => (rule.min_speed_m_s, rule.max_speed_m_s),
        AftsCorridorMetricConfig::FlightPathAngleRad => (
            rule.min_flight_path_angle_rad,
            rule.max_flight_path_angle_rad,
        ),
    }
}

fn propagator_for_document(
    document: &ScenarioDocument,
    config: &AftsPropagatorConfig,
) -> Result<IipPropagator, AftsError> {
    let surface = match config.surface {
        AftsPropagatorSurfaceConfig::Wgs84Sphere => IipImpactSurface::wgs84_sphere(),
        AftsPropagatorSurfaceConfig::Wgs84Ellipsoid => IipImpactSurface::wgs84_ellipsoid(),
    };
    let earth_rotation_rate_rad_s = match config.earth_rotation {
        AftsPropagatorEarthRotationConfig::FrameProfile => {
            if document.environment.frame_profile == "toy-fixed-earth" {
                0.0
            } else {
                WGS84_OMEGA_RAD_S
            }
        }
        AftsPropagatorEarthRotationConfig::Disabled => 0.0,
        AftsPropagatorEarthRotationConfig::Wgs84Uniform => WGS84_OMEGA_RAD_S,
    };
    let drag_model = config
        .drag
        .map(|drag| {
            ExponentialAtmosphericDrag::new(
                surface.equatorial_radius_m(),
                drag.reference_density_kg_m3,
                drag.scale_height_m,
                drag.ballistic_coefficient_kg_m2,
            )
        })
        .transpose()?;
    IipPropagator::new_with_surface_and_drag(
        surface,
        WGS84_MU_M3_S2,
        config.step_s.unwrap_or(0.25),
        config.max_time_s.unwrap_or(7_200.0),
        config.radius_tolerance_m.unwrap_or(1.0e-3),
        earth_rotation_rate_rad_s,
        drag_model,
    )
}
