//! Fixed-grid Method-of-Characteristics line transient primitive.

use std::collections::{HashMap, HashSet};

use openbmp_core::ValidationStatus;

use crate::error::FeedSystemError;

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
const GAS_ACCUMULATOR_SOLVE_MAX_ITERS: usize = 96;
const GAS_ACCUMULATOR_BRACKET_MAX_ITERS: usize = 96;

/// Frictionless fixed-grid MOC line configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MocLineConfig {
    /// Pipe length in m.
    pub length_m: f64,
    /// Acoustic wave speed in m/s.
    pub wave_speed_m_s: f64,
    /// Fluid density in kg/m^3.
    pub density_kg_m3: f64,
    /// Pipe cross-sectional flow area in m^2.
    pub cross_section_area_m2: f64,
    /// Number of equal pipe segments. The state has `segment_count + 1` nodes.
    pub segment_count: usize,
}

impl MocLineConfig {
    /// Validate line configuration.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite, non-positive, or unsupported
    /// line parameters.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_positive_finite(self.length_m, "MOC line length must be positive")?;
        require_positive_finite(self.wave_speed_m_s, "MOC line wave speed must be positive")?;
        require_positive_finite(self.density_kg_m3, "MOC line density must be positive")?;
        require_positive_finite(
            self.cross_section_area_m2,
            "MOC line cross-section area must be positive",
        )?;
        if self.segment_count == 0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line segment count must be positive",
            });
        }
        Ok(())
    }

    /// Number of pressure/velocity nodes.
    #[must_use]
    pub const fn node_count(self) -> usize {
        self.segment_count + 1
    }

    /// Courant-exact time step `dx/a` for this fixed grid.
    #[must_use]
    pub fn courant_dt_s(self) -> f64 {
        self.length_m / self.segment_count as f64 / self.wave_speed_m_s
    }

    /// Convert piezometric head in m to gauge pressure in Pa for this line.
    #[must_use]
    pub fn pressure_from_head_pa(self, head_m: f64) -> f64 {
        self.density_kg_m3 * STANDARD_GRAVITY_M_S2 * head_m
    }

    fn wave_over_gravity(self) -> f64 {
        self.wave_speed_m_s / STANDARD_GRAVITY_M_S2
    }
}

/// MOC line state: piezometric head and axial velocity at each node.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineState {
    /// Piezometric head in m at each node.
    pub head_m: Vec<f64>,
    /// Axial velocity in m/s at each node.
    pub velocity_m_s: Vec<f64>,
}

impl MocLineState {
    /// Build a uniform line state.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid config or non-finite state values.
    pub fn uniform(
        config: MocLineConfig,
        head_m: f64,
        velocity_m_s: f64,
    ) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        require_finite(head_m, "MOC line uniform head must be finite")?;
        require_finite(velocity_m_s, "MOC line uniform velocity must be finite")?;
        Ok(Self {
            head_m: vec![head_m; config.node_count()],
            velocity_m_s: vec![velocity_m_s; config.node_count()],
        })
    }

    fn require_valid(&self, config: MocLineConfig) -> Result<(), FeedSystemError> {
        let node_count = config.node_count();
        if self.head_m.len() != node_count || self.velocity_m_s.len() != node_count {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line state length must match segment_count + 1",
            });
        }
        for value in &self.head_m {
            require_finite(*value, "MOC line state head must be finite")?;
        }
        for value in &self.velocity_m_s {
            require_finite(*value, "MOC line state velocity must be finite")?;
        }
        Ok(())
    }
}

/// One named line element in a standalone MOC topology.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineElementConfig {
    /// Stable caller-facing line identifier.
    pub id: String,
    /// Fixed-grid line configuration.
    pub config: MocLineConfig,
    /// Initial head/velocity state for this line.
    pub initial_state: MocLineState,
    /// Endpoint boundary conditions applied at the first step.
    pub boundary: MocLineEndpointBoundaryConditions,
}

impl MocLineElementConfig {
    /// Build a named topology element with a uniform initial state.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the line configuration, initial state,
    /// id, or boundary values are invalid.
    pub fn uniform(
        id: impl Into<String>,
        config: MocLineConfig,
        initial_head_m: f64,
        initial_velocity_m_s: f64,
        boundary: MocLineEndpointBoundaryConditions,
    ) -> Result<Self, FeedSystemError> {
        let element = Self {
            id: id.into(),
            config,
            initial_state: MocLineState::uniform(config, initial_head_m, initial_velocity_m_s)?,
            boundary,
        };
        element.require_valid()?;
        Ok(element)
    }

    fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_non_empty_id(&self.id)?;
        self.config.require_valid()?;
        self.initial_state.require_valid(self.config)?;
        self.boundary.require_valid()
    }
}

/// Standalone collection of named fixed-grid MOC lines.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineTopologyConfig {
    /// Named MOC line elements.
    pub lines: Vec<MocLineElementConfig>,
}

impl MocLineTopologyConfig {
    /// Validate topology shape, per-line inputs, and duplicate ids.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for an empty topology, duplicate ids, or
    /// invalid per-line configuration/state/boundary values.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        if self.lines.is_empty() {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line topology must contain at least one line",
            });
        }
        let mut ids = HashSet::with_capacity(self.lines.len());
        let mut reference_dt_s: Option<f64> = None;
        for element in &self.lines {
            element.require_valid()?;
            if !ids.insert(element.id.as_str()) {
                return Err(FeedSystemError::InvalidParameter {
                    reason: "MOC line topology ids must be unique",
                });
            }
            let dt_s = element.config.courant_dt_s();
            if let Some(reference_dt_s) = reference_dt_s {
                let tolerance = 1.0e-12_f64.max(reference_dt_s.abs() * 1.0e-9);
                if (dt_s - reference_dt_s).abs() > tolerance {
                    return Err(FeedSystemError::InvalidParameter {
                        reason: "MOC line topology Courant time steps must match",
                    });
                }
            } else {
                reference_dt_s = Some(dt_s);
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
struct MocLineTopologyElement {
    id: String,
    line: MocLine,
    state: MocLineState,
    boundary: MocLineEndpointBoundaryConditions,
}

/// Snapshot for one named line in a topology step.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineTopologyLineSnapshot {
    /// Stable caller-facing line identifier.
    pub id: String,
    /// Snapshot from the line step.
    pub snapshot: MocLineSnapshot,
}

/// Snapshot from one standalone MOC topology step.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineTopologySnapshot {
    /// Per-line snapshots in topology declaration order.
    pub lines: Vec<MocLineTopologyLineSnapshot>,
    /// Common Courant-exact topology step size in seconds.
    pub dt_s: f64,
    /// Validation posture of the topology model.
    pub validation: ValidationStatus,
}

impl MocLineTopologySnapshot {
    /// Return a line snapshot by id.
    #[must_use]
    pub fn line(&self, id: &str) -> Option<&MocLineSnapshot> {
        self.lines
            .iter()
            .find(|line| line.id == id)
            .map(|line| &line.snapshot)
    }
}

/// Endpoint selector for a named MOC line in a graph junction.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum MocLineEndpointKind {
    /// The upstream end of the line.
    Upstream,
    /// The downstream end of the line.
    Downstream,
}

/// One line endpoint attached to a graph junction.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct MocLineJunctionConnection {
    /// Referenced line id.
    pub line_id: String,
    /// Referenced endpoint on the line.
    pub endpoint: MocLineEndpointKind,
}

/// A zero-storage junction connecting two or more MOC line endpoints.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineJunctionConfig {
    /// Stable caller-facing junction identifier.
    pub id: String,
    /// Attached line endpoints.
    pub connections: Vec<MocLineJunctionConnection>,
}

/// Standalone MOC line graph with zero-storage junction semantics.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineGraphTopologyConfig {
    /// Named MOC line elements.
    pub lines: Vec<MocLineElementConfig>,
    /// Zero-storage junctions connecting line endpoints.
    pub junctions: Vec<MocLineJunctionConfig>,
}

impl MocLineGraphTopologyConfig {
    /// Validate graph shape, line configs, junction references, and endpoint use.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the line topology is malformed, a
    /// junction is invalid, a junction references an unknown line id, or a line
    /// endpoint is attached to more than one junction.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        MocLineTopologyConfig {
            lines: self.lines.clone(),
        }
        .require_valid()?;
        if self.junctions.is_empty() {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line graph topology must contain at least one junction",
            });
        }

        let line_ids: HashSet<&str> = self.lines.iter().map(|line| line.id.as_str()).collect();
        let mut junction_ids = HashSet::with_capacity(self.junctions.len());
        let mut connected_endpoints = HashSet::new();
        for junction in &self.junctions {
            require_non_empty_id(&junction.id)?;
            if !junction_ids.insert(junction.id.as_str()) {
                return Err(FeedSystemError::InvalidParameter {
                    reason: "MOC line graph junction ids must be unique",
                });
            }
            if junction.connections.len() < 2 {
                return Err(FeedSystemError::InvalidParameter {
                    reason: "MOC line graph junctions must connect at least two endpoints",
                });
            }
            let mut junction_line_ids = HashSet::with_capacity(junction.connections.len());
            for connection in &junction.connections {
                require_non_empty_id(&connection.line_id)?;
                if !line_ids.contains(connection.line_id.as_str()) {
                    return Err(FeedSystemError::InvalidParameter {
                        reason: "MOC line graph junction references an unknown line id",
                    });
                }
                if !junction_line_ids.insert(connection.line_id.as_str()) {
                    return Err(FeedSystemError::InvalidParameter {
                        reason: "MOC line graph junction cannot connect multiple endpoints of one line",
                    });
                }
                if !connected_endpoints.insert((connection.line_id.as_str(), connection.endpoint)) {
                    return Err(FeedSystemError::InvalidParameter {
                        reason: "MOC line graph endpoint can belong to only one junction",
                    });
                }
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct MocLineJunctionRuntimeConnection {
    line_index: usize,
    endpoint: MocLineEndpointKind,
}

#[derive(Clone, Debug, PartialEq)]
struct MocLineJunctionRuntime {
    id: String,
    connections: Vec<MocLineJunctionRuntimeConnection>,
}

/// Runtime diagnostic for one line endpoint attached to a junction.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineJunctionConnectionSnapshot {
    /// Referenced line id.
    pub line_id: String,
    /// Referenced endpoint on the line.
    pub endpoint: MocLineEndpointKind,
    /// Endpoint velocity solved from the common junction head and line
    /// characteristic.
    pub velocity_m_s: f64,
    /// Signed volume flow into the junction in m^3/s.
    pub signed_flow_into_junction_m3_s: f64,
}

/// Runtime diagnostic for one zero-storage graph junction.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineJunctionSnapshot {
    /// Stable caller-facing junction id.
    pub id: String,
    /// Common junction head in m.
    pub head_m: f64,
    /// Sum of signed volume flows into the junction in m^3/s.
    pub net_flow_m3_s: f64,
    /// Per-connection endpoint diagnostics.
    pub connections: Vec<MocLineJunctionConnectionSnapshot>,
}

/// Snapshot from one standalone MOC graph step.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineGraphTopologySnapshot {
    /// Per-line snapshots in topology declaration order.
    pub lines: Vec<MocLineTopologyLineSnapshot>,
    /// Junction diagnostics in topology declaration order.
    pub junctions: Vec<MocLineJunctionSnapshot>,
    /// Common Courant-exact topology step size in seconds.
    pub dt_s: f64,
    /// Validation posture of the graph model.
    pub validation: ValidationStatus,
}

impl MocLineGraphTopologySnapshot {
    /// Return a line snapshot by id.
    #[must_use]
    pub fn line(&self, id: &str) -> Option<&MocLineSnapshot> {
        self.lines
            .iter()
            .find(|line| line.id == id)
            .map(|line| &line.snapshot)
    }

    /// Return a junction snapshot by id.
    #[must_use]
    pub fn junction(&self, id: &str) -> Option<&MocLineJunctionSnapshot> {
        self.junctions.iter().find(|junction| junction.id == id)
    }
}

/// Runtime state for a standalone named-line MOC topology.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineTopology {
    elements: Vec<MocLineTopologyElement>,
}

impl MocLineTopology {
    /// Construct a standalone MOC topology from validated named line elements.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the topology is malformed.
    pub fn new(config: MocLineTopologyConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        let mut elements = Vec::with_capacity(config.lines.len());
        for element in config.lines {
            elements.push(MocLineTopologyElement {
                id: element.id,
                line: MocLine::new(element.config)?,
                state: element.initial_state,
                boundary: element.boundary,
            });
        }
        Ok(Self { elements })
    }

    /// Number of named line elements.
    #[must_use]
    pub fn line_count(&self) -> usize {
        self.elements.len()
    }

    /// Common Courant-exact topology step size.
    #[must_use]
    pub fn courant_dt_s(&self) -> f64 {
        self.elements[0].line.courant_dt_s()
    }

    /// Iterate line ids in topology declaration order.
    pub fn line_ids(&self) -> impl Iterator<Item = &str> {
        self.elements.iter().map(|element| element.id.as_str())
    }

    /// Return the current state for a named line.
    #[must_use]
    pub fn state(&self, id: &str) -> Option<&MocLineState> {
        self.elements
            .iter()
            .find(|element| element.id == id)
            .map(|element| &element.state)
    }

    /// Return the current endpoint boundary for a named line.
    #[must_use]
    pub fn boundary(&self, id: &str) -> Option<MocLineEndpointBoundaryConditions> {
        self.elements
            .iter()
            .find(|element| element.id == id)
            .map(|element| element.boundary)
    }

    /// Replace endpoint boundary conditions for a named line.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the id is unknown or the boundary is
    /// invalid.
    pub fn set_boundary(
        &mut self,
        id: &str,
        boundary: MocLineEndpointBoundaryConditions,
    ) -> Result<(), FeedSystemError> {
        let element = self
            .elements
            .iter_mut()
            .find(|element| element.id == id)
            .ok_or(FeedSystemError::InvalidParameter {
                reason: "MOC line topology id was not found",
            })?;
        boundary.require_valid()?;
        element.boundary = boundary;
        Ok(())
    }

    /// Advance every named line by one Courant-exact MOC step.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid line state, invalid boundary
    /// values, or non-finite characteristic updates. Runtime state is updated
    /// only after every line step has succeeded.
    pub fn step(&mut self) -> Result<MocLineTopologySnapshot, FeedSystemError> {
        let mut line_snapshots = Vec::with_capacity(self.elements.len());
        for element in &self.elements {
            let snapshot = element
                .line
                .step_with_endpoint_boundaries(&element.state, element.boundary)?;
            line_snapshots.push(MocLineTopologyLineSnapshot {
                id: element.id.clone(),
                snapshot,
            });
        }

        for (element, line_snapshot) in self.elements.iter_mut().zip(&line_snapshots) {
            element.state = line_snapshot.snapshot.state.clone();
        }

        Ok(MocLineTopologySnapshot {
            lines: line_snapshots,
            dt_s: self.courant_dt_s(),
            validation: ValidationStatus::ValidatedToy,
        })
    }
}

/// Runtime state for a standalone named-line MOC graph topology.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineGraphTopology {
    elements: Vec<MocLineTopologyElement>,
    junctions: Vec<MocLineJunctionRuntime>,
}

impl MocLineGraphTopology {
    /// Construct a standalone MOC graph topology from named line elements and
    /// zero-storage junction declarations.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the topology is malformed.
    pub fn new(config: MocLineGraphTopologyConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        let mut elements = Vec::with_capacity(config.lines.len());
        let mut line_indices = HashMap::with_capacity(config.lines.len());
        for (index, element) in config.lines.into_iter().enumerate() {
            line_indices.insert(element.id.clone(), index);
            elements.push(MocLineTopologyElement {
                id: element.id,
                line: MocLine::new(element.config)?,
                state: element.initial_state,
                boundary: element.boundary,
            });
        }

        let mut junctions = Vec::with_capacity(config.junctions.len());
        for junction in config.junctions {
            let mut connections = Vec::with_capacity(junction.connections.len());
            for connection in junction.connections {
                let line_index = *line_indices.get(&connection.line_id).ok_or(
                    FeedSystemError::InvalidParameter {
                        reason: "MOC line graph junction references an unknown line id",
                    },
                )?;
                connections.push(MocLineJunctionRuntimeConnection {
                    line_index,
                    endpoint: connection.endpoint,
                });
            }
            junctions.push(MocLineJunctionRuntime {
                id: junction.id,
                connections,
            });
        }

        Ok(Self {
            elements,
            junctions,
        })
    }

    /// Number of named line elements.
    #[must_use]
    pub fn line_count(&self) -> usize {
        self.elements.len()
    }

    /// Number of zero-storage junctions.
    #[must_use]
    pub fn junction_count(&self) -> usize {
        self.junctions.len()
    }

    /// Common Courant-exact topology step size.
    #[must_use]
    pub fn courant_dt_s(&self) -> f64 {
        self.elements[0].line.courant_dt_s()
    }

    /// Iterate line ids in topology declaration order.
    pub fn line_ids(&self) -> impl Iterator<Item = &str> {
        self.elements.iter().map(|element| element.id.as_str())
    }

    /// Return the current state for a named line.
    #[must_use]
    pub fn state(&self, id: &str) -> Option<&MocLineState> {
        self.elements
            .iter()
            .find(|element| element.id == id)
            .map(|element| &element.state)
    }

    /// Advance every named line and graph junction by one Courant-exact MOC
    /// step.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid line state, invalid boundary
    /// values, non-finite junction solves, or non-finite line updates. Runtime
    /// state is updated only after every junction and line step has succeeded.
    pub fn step(&mut self) -> Result<MocLineGraphTopologySnapshot, FeedSystemError> {
        let mut boundaries: Vec<MocLineEndpointBoundaryConditions> = self
            .elements
            .iter()
            .map(|element| element.boundary)
            .collect();
        let mut junction_snapshots = Vec::with_capacity(self.junctions.len());

        for junction in &self.junctions {
            let snapshot = self.solve_junction(junction)?;
            for connection in &junction.connections {
                match connection.endpoint {
                    MocLineEndpointKind::Upstream => {
                        boundaries[connection.line_index].upstream =
                            MocLineUpstreamBoundary::Head {
                                head_m: snapshot.head_m,
                            };
                    }
                    MocLineEndpointKind::Downstream => {
                        boundaries[connection.line_index].downstream =
                            MocLineDownstreamBoundary::Head {
                                head_m: snapshot.head_m,
                            };
                    }
                }
            }
            junction_snapshots.push(snapshot);
        }

        let mut line_snapshots = Vec::with_capacity(self.elements.len());
        for (element, boundary) in self.elements.iter().zip(boundaries) {
            let snapshot = element
                .line
                .step_with_endpoint_boundaries(&element.state, boundary)?;
            line_snapshots.push(MocLineTopologyLineSnapshot {
                id: element.id.clone(),
                snapshot,
            });
        }

        for (element, line_snapshot) in self.elements.iter_mut().zip(&line_snapshots) {
            element.state = line_snapshot.snapshot.state.clone();
        }

        Ok(MocLineGraphTopologySnapshot {
            lines: line_snapshots,
            junctions: junction_snapshots,
            dt_s: self.courant_dt_s(),
            validation: ValidationStatus::ValidatedToy,
        })
    }

    fn solve_junction(
        &self,
        junction: &MocLineJunctionRuntime,
    ) -> Result<MocLineJunctionSnapshot, FeedSystemError> {
        let mut weighted_characteristic_sum = 0.0;
        let mut weight_sum = 0.0;
        for connection in &junction.connections {
            let element = &self.elements[connection.line_index];
            element.state.require_valid(element.line.config())?;
            let weight = junction_weight(element.line.config());
            let characteristic =
                junction_characteristic(element.line.config(), &element.state, connection.endpoint);
            weighted_characteristic_sum += weight * characteristic;
            weight_sum += weight;
        }
        if !weighted_characteristic_sum.is_finite() || !weight_sum.is_finite() || weight_sum <= 0.0
        {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line graph junction solve is non-finite",
            });
        }
        let head_m = weighted_characteristic_sum / weight_sum;
        if !head_m.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line graph junction head is non-finite",
            });
        }

        let mut net_flow_m3_s = 0.0;
        let mut snapshots = Vec::with_capacity(junction.connections.len());
        for connection in &junction.connections {
            let element = &self.elements[connection.line_index];
            let velocity_m_s = junction_endpoint_velocity(
                element.line.config(),
                &element.state,
                connection.endpoint,
                head_m,
            );
            let signed_flow_into_junction_m3_s = match connection.endpoint {
                MocLineEndpointKind::Upstream => {
                    -element.line.config().cross_section_area_m2 * velocity_m_s
                }
                MocLineEndpointKind::Downstream => {
                    element.line.config().cross_section_area_m2 * velocity_m_s
                }
            };
            if !velocity_m_s.is_finite() || !signed_flow_into_junction_m3_s.is_finite() {
                return Err(FeedSystemError::NonFinite {
                    reason: "MOC line graph junction flow is non-finite",
                });
            }
            net_flow_m3_s += signed_flow_into_junction_m3_s;
            snapshots.push(MocLineJunctionConnectionSnapshot {
                line_id: element.id.clone(),
                endpoint: connection.endpoint,
                velocity_m_s,
                signed_flow_into_junction_m3_s,
            });
        }
        if !net_flow_m3_s.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line graph junction net flow is non-finite",
            });
        }

        Ok(MocLineJunctionSnapshot {
            id: junction.id.clone(),
            head_m,
            net_flow_m3_s,
            connections: snapshots,
        })
    }
}

fn junction_weight(config: MocLineConfig) -> f64 {
    config.cross_section_area_m2 / config.wave_over_gravity()
}

fn junction_characteristic(
    config: MocLineConfig,
    state: &MocLineState,
    endpoint: MocLineEndpointKind,
) -> f64 {
    match endpoint {
        MocLineEndpointKind::Upstream => {
            state.head_m[1] - config.wave_over_gravity() * state.velocity_m_s[1]
        }
        MocLineEndpointKind::Downstream => {
            let interior_index = config.node_count() - 2;
            state.head_m[interior_index]
                + config.wave_over_gravity() * state.velocity_m_s[interior_index]
        }
    }
}

fn junction_endpoint_velocity(
    config: MocLineConfig,
    state: &MocLineState,
    endpoint: MocLineEndpointKind,
    head_m: f64,
) -> f64 {
    match endpoint {
        MocLineEndpointKind::Upstream => {
            (head_m - junction_characteristic(config, state, endpoint)) / config.wave_over_gravity()
        }
        MocLineEndpointKind::Downstream => {
            (junction_characteristic(config, state, endpoint) - head_m) / config.wave_over_gravity()
        }
    }
}

/// Backward-compatible boundary conditions for one MOC line step.
///
/// This is the original valve-velocity boundary shape. New callers that need
/// additional boundary types should use [`MocLineBoundaryConditions`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MocLineBoundary {
    /// Fixed upstream reservoir/source head in m.
    pub upstream_head_m: f64,
    /// Downstream valve velocity in m/s.
    pub downstream_velocity_m_s: f64,
}

impl MocLineBoundary {
    fn into_boundary_conditions(self) -> MocLineBoundaryConditions {
        MocLineBoundaryConditions {
            upstream_head_m: self.upstream_head_m,
            downstream: MocLineDownstreamBoundary::Velocity {
                velocity_m_s: self.downstream_velocity_m_s,
            },
        }
    }
}

/// Upstream boundary option for a fixed-grid MOC line step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MocLineUpstreamBoundary {
    /// Prescribed upstream reservoir/source head in m.
    Head {
        /// Upstream piezometric head in m.
        head_m: f64,
    },
    /// Prescribed upstream source/pump velocity in m/s.
    Velocity {
        /// Upstream axial velocity in m/s.
        velocity_m_s: f64,
    },
    /// Nonlinear gas-pressurized upstream source/tank endpoint.
    ///
    /// The endpoint solves the incoming characteristic together with a
    /// polytropic source gas:
    /// `H = H_datum + P0/(rho*g) * (V0 / (V0 + delivered_delta + A*V*dt))^n`.
    GasSource {
        /// Datum head added to the gas pressure head in m.
        datum_head_m: f64,
        /// Gas volume at precharge/reference state in m^3.
        gas_reference_volume_m3: f64,
        /// Gas precharge/reference pressure in Pa.
        gas_precharge_pressure_pa: f64,
        /// Liquid volume already delivered beyond the reference state in m^3.
        delivered_liquid_volume_delta_m3: f64,
        /// Polytropic exponent.
        polytropic_exponent: f64,
    },
}

/// Downstream boundary option for a fixed-grid MOC line step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MocLineDownstreamBoundary {
    /// Prescribed downstream valve/end velocity in m/s.
    Velocity {
        /// Downstream axial velocity in m/s.
        velocity_m_s: f64,
    },
    /// Prescribed downstream reservoir/end head in m.
    Head {
        /// Downstream piezometric head in m.
        head_m: f64,
    },
    /// Linear accumulator/compliance endpoint.
    ///
    /// The endpoint solves the incoming characteristic together with
    /// `H = H0 + (stored_volume_delta + A * V * dt) / compliance`.
    Accumulator {
        /// Accumulator head at zero stored-volume offset in m.
        reference_head_m: f64,
        /// Current stored-volume offset in m^3 before this line step.
        stored_volume_delta_m3: f64,
        /// Linear volume compliance `dV/dH` in m^3 per m head.
        compliance_m3_per_m: f64,
    },
    /// Nonlinear gas-charged accumulator endpoint.
    ///
    /// The endpoint solves the incoming characteristic together with a
    /// polytropic gas spring:
    /// `H = H_datum + P0/(rho*g) * (V0 / (V0 - stored_delta - A*V*dt))^n`.
    GasAccumulator {
        /// Datum head added to the gas pressure head in m.
        datum_head_m: f64,
        /// Gas volume at precharge/reference state in m^3.
        gas_reference_volume_m3: f64,
        /// Gas precharge/reference pressure in Pa.
        gas_precharge_pressure_pa: f64,
        /// Current liquid volume stored beyond the reference state in m^3.
        stored_liquid_volume_delta_m3: f64,
        /// Polytropic exponent.
        polytropic_exponent: f64,
    },
}

/// Boundary conditions for one generalized MOC line step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MocLineBoundaryConditions {
    /// Fixed upstream reservoir/source head in m.
    pub upstream_head_m: f64,
    /// Downstream boundary condition.
    pub downstream: MocLineDownstreamBoundary,
}

impl MocLineBoundaryConditions {
    fn into_endpoint_boundaries(self) -> MocLineEndpointBoundaryConditions {
        MocLineEndpointBoundaryConditions {
            upstream: MocLineUpstreamBoundary::Head {
                head_m: self.upstream_head_m,
            },
            downstream: self.downstream,
        }
    }
}

/// Full endpoint boundary conditions for one generalized MOC line step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MocLineEndpointBoundaryConditions {
    /// Upstream endpoint boundary condition.
    pub upstream: MocLineUpstreamBoundary,
    /// Downstream endpoint boundary condition.
    pub downstream: MocLineDownstreamBoundary,
}

impl MocLineEndpointBoundaryConditions {
    fn require_valid(self) -> Result<(), FeedSystemError> {
        match self.upstream {
            MocLineUpstreamBoundary::Head { head_m } => {
                require_finite(head_m, "MOC line upstream boundary head must be finite")
            }
            MocLineUpstreamBoundary::Velocity { velocity_m_s } => require_finite(
                velocity_m_s,
                "MOC line upstream boundary velocity must be finite",
            ),
            MocLineUpstreamBoundary::GasSource {
                datum_head_m,
                gas_reference_volume_m3,
                gas_precharge_pressure_pa,
                delivered_liquid_volume_delta_m3,
                polytropic_exponent,
            } => GasSourceBoundary {
                datum_head_m,
                gas_reference_volume_m3,
                gas_precharge_pressure_pa,
                delivered_liquid_volume_delta_m3,
                polytropic_exponent,
            }
            .require_valid(),
        }?;
        match self.downstream {
            MocLineDownstreamBoundary::Velocity { velocity_m_s } => require_finite(
                velocity_m_s,
                "MOC line downstream boundary velocity must be finite",
            ),
            MocLineDownstreamBoundary::Head { head_m } => {
                require_finite(head_m, "MOC line downstream boundary head must be finite")
            }
            MocLineDownstreamBoundary::Accumulator {
                reference_head_m,
                stored_volume_delta_m3,
                compliance_m3_per_m,
            } => {
                require_finite(
                    reference_head_m,
                    "MOC line downstream accumulator reference head must be finite",
                )?;
                require_finite(
                    stored_volume_delta_m3,
                    "MOC line downstream accumulator stored volume must be finite",
                )?;
                require_positive_finite(
                    compliance_m3_per_m,
                    "MOC line downstream accumulator compliance must be positive",
                )
            }
            MocLineDownstreamBoundary::GasAccumulator {
                datum_head_m,
                gas_reference_volume_m3,
                gas_precharge_pressure_pa,
                stored_liquid_volume_delta_m3,
                polytropic_exponent,
            } => GasAccumulatorBoundary {
                datum_head_m,
                gas_reference_volume_m3,
                gas_precharge_pressure_pa,
                stored_liquid_volume_delta_m3,
                polytropic_exponent,
            }
            .require_valid(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct GasSourceBoundary {
    datum_head_m: f64,
    gas_reference_volume_m3: f64,
    gas_precharge_pressure_pa: f64,
    delivered_liquid_volume_delta_m3: f64,
    polytropic_exponent: f64,
}

impl GasSourceBoundary {
    fn require_valid(self) -> Result<(), FeedSystemError> {
        require_finite(
            self.datum_head_m,
            "MOC line upstream gas source datum head must be finite",
        )?;
        require_positive_finite(
            self.gas_reference_volume_m3,
            "MOC line upstream gas source reference volume must be positive",
        )?;
        require_positive_finite(
            self.gas_precharge_pressure_pa,
            "MOC line upstream gas source precharge pressure must be positive",
        )?;
        require_finite(
            self.delivered_liquid_volume_delta_m3,
            "MOC line upstream gas source delivered volume must be finite",
        )?;
        require_positive_finite(
            self.polytropic_exponent,
            "MOC line upstream gas source polytropic exponent must be positive",
        )?;
        if self.available_gas_volume_m3() <= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line upstream gas source gas volume must remain positive",
            });
        }
        Ok(())
    }

    fn available_gas_volume_m3(self) -> f64 {
        self.gas_reference_volume_m3 + self.delivered_liquid_volume_delta_m3
    }

    fn head_m(self, config: MocLineConfig, velocity_m_s: f64) -> Result<f64, FeedSystemError> {
        let gas_volume_m3 = self.available_gas_volume_m3()
            + config.cross_section_area_m2 * config.courant_dt_s() * velocity_m_s;
        if gas_volume_m3 <= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line upstream gas source gas volume must remain positive",
            });
        }
        let pressure_pa = self.gas_precharge_pressure_pa
            * (self.gas_reference_volume_m3 / gas_volume_m3).powf(self.polytropic_exponent);
        let head_m =
            self.datum_head_m + pressure_pa / (config.density_kg_m3 * STANDARD_GRAVITY_M_S2);
        if !head_m.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line upstream gas source head is non-finite",
            });
        }
        Ok(head_m)
    }

    fn residual(
        self,
        config: MocLineConfig,
        upstream_compat_minus: f64,
        wave_over_gravity: f64,
        velocity_m_s: f64,
    ) -> Result<f64, FeedSystemError> {
        let residual = self.head_m(config, velocity_m_s)?
            - wave_over_gravity * velocity_m_s
            - upstream_compat_minus;
        if !residual.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line upstream gas source residual is non-finite",
            });
        }
        Ok(residual)
    }

    fn solve_velocity(
        self,
        config: MocLineConfig,
        upstream_compat_minus: f64,
        wave_over_gravity: f64,
    ) -> Result<f64, FeedSystemError> {
        let lower_velocity_limit_m_s = -self.available_gas_volume_m3()
            / (config.cross_section_area_m2 * config.courant_dt_s());
        if !lower_velocity_limit_m_s.is_finite() || lower_velocity_limit_m_s >= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line upstream gas source velocity limit is invalid",
            });
        }

        let mut low_velocity_m_s = lower_velocity_limit_m_s * 0.5;
        let mut low_residual = self.residual(
            config,
            upstream_compat_minus,
            wave_over_gravity,
            low_velocity_m_s,
        )?;
        for _ in 0..GAS_ACCUMULATOR_BRACKET_MAX_ITERS {
            if low_residual > 0.0 {
                break;
            }
            low_velocity_m_s = 0.5 * (low_velocity_m_s + lower_velocity_limit_m_s);
            low_residual = self.residual(
                config,
                upstream_compat_minus,
                wave_over_gravity,
                low_velocity_m_s,
            )?;
        }
        if low_residual <= 0.0 {
            return Err(FeedSystemError::NonConverged {
                reason: "MOC line upstream gas source could not bracket low velocity",
            });
        }

        let mut high_velocity_m_s = 0.0;
        let mut high_residual = self.residual(
            config,
            upstream_compat_minus,
            wave_over_gravity,
            high_velocity_m_s,
        )?;
        for _ in 0..GAS_ACCUMULATOR_BRACKET_MAX_ITERS {
            if high_residual <= 0.0 {
                break;
            }
            high_velocity_m_s = if high_velocity_m_s == 0.0 {
                1.0
            } else {
                high_velocity_m_s * 2.0
            };
            high_residual = self.residual(
                config,
                upstream_compat_minus,
                wave_over_gravity,
                high_velocity_m_s,
            )?;
        }
        if high_residual > 0.0 {
            return Err(FeedSystemError::NonConverged {
                reason: "MOC line upstream gas source could not bracket high velocity",
            });
        }

        for _ in 0..GAS_ACCUMULATOR_SOLVE_MAX_ITERS {
            let mid_velocity_m_s = 0.5 * (low_velocity_m_s + high_velocity_m_s);
            let mid_residual = self.residual(
                config,
                upstream_compat_minus,
                wave_over_gravity,
                mid_velocity_m_s,
            )?;
            if mid_residual > 0.0 {
                low_velocity_m_s = mid_velocity_m_s;
            } else {
                high_velocity_m_s = mid_velocity_m_s;
            }
        }

        let velocity_m_s = 0.5 * (low_velocity_m_s + high_velocity_m_s);
        if !velocity_m_s.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line upstream gas source velocity is non-finite",
            });
        }
        Ok(velocity_m_s)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct GasAccumulatorBoundary {
    datum_head_m: f64,
    gas_reference_volume_m3: f64,
    gas_precharge_pressure_pa: f64,
    stored_liquid_volume_delta_m3: f64,
    polytropic_exponent: f64,
}

impl GasAccumulatorBoundary {
    fn require_valid(self) -> Result<(), FeedSystemError> {
        require_finite(
            self.datum_head_m,
            "MOC line downstream gas accumulator datum head must be finite",
        )?;
        require_positive_finite(
            self.gas_reference_volume_m3,
            "MOC line downstream gas accumulator reference volume must be positive",
        )?;
        require_positive_finite(
            self.gas_precharge_pressure_pa,
            "MOC line downstream gas accumulator precharge pressure must be positive",
        )?;
        require_finite(
            self.stored_liquid_volume_delta_m3,
            "MOC line downstream gas accumulator stored volume must be finite",
        )?;
        require_positive_finite(
            self.polytropic_exponent,
            "MOC line downstream gas accumulator polytropic exponent must be positive",
        )?;
        if self.available_gas_volume_m3() <= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line downstream gas accumulator gas volume must remain positive",
            });
        }
        Ok(())
    }

    fn available_gas_volume_m3(self) -> f64 {
        self.gas_reference_volume_m3 - self.stored_liquid_volume_delta_m3
    }

    fn head_m(self, config: MocLineConfig, velocity_m_s: f64) -> Result<f64, FeedSystemError> {
        let gas_volume_m3 = self.available_gas_volume_m3()
            - config.cross_section_area_m2 * config.courant_dt_s() * velocity_m_s;
        if gas_volume_m3 <= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line downstream gas accumulator gas volume must remain positive",
            });
        }
        let pressure_pa = self.gas_precharge_pressure_pa
            * (self.gas_reference_volume_m3 / gas_volume_m3).powf(self.polytropic_exponent);
        let head_m =
            self.datum_head_m + pressure_pa / (config.density_kg_m3 * STANDARD_GRAVITY_M_S2);
        if !head_m.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line downstream gas accumulator head is non-finite",
            });
        }
        Ok(head_m)
    }

    fn residual(
        self,
        config: MocLineConfig,
        downstream_compat_plus: f64,
        wave_over_gravity: f64,
        velocity_m_s: f64,
    ) -> Result<f64, FeedSystemError> {
        let residual = self.head_m(config, velocity_m_s)? + wave_over_gravity * velocity_m_s
            - downstream_compat_plus;
        if !residual.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line downstream gas accumulator residual is non-finite",
            });
        }
        Ok(residual)
    }

    fn solve_velocity(
        self,
        config: MocLineConfig,
        downstream_compat_plus: f64,
        wave_over_gravity: f64,
    ) -> Result<f64, FeedSystemError> {
        let upper_velocity_m_s =
            self.available_gas_volume_m3() / (config.cross_section_area_m2 * config.courant_dt_s());
        if !upper_velocity_m_s.is_finite() || upper_velocity_m_s <= 0.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line downstream gas accumulator velocity limit is invalid",
            });
        }

        let mut high_velocity_m_s = upper_velocity_m_s * 0.5;
        let mut high_residual = self.residual(
            config,
            downstream_compat_plus,
            wave_over_gravity,
            high_velocity_m_s,
        )?;
        for _ in 0..GAS_ACCUMULATOR_BRACKET_MAX_ITERS {
            if high_residual > 0.0 {
                break;
            }
            high_velocity_m_s = 0.5 * (high_velocity_m_s + upper_velocity_m_s);
            high_residual = self.residual(
                config,
                downstream_compat_plus,
                wave_over_gravity,
                high_velocity_m_s,
            )?;
        }
        if high_residual <= 0.0 {
            return Err(FeedSystemError::NonConverged {
                reason: "MOC line downstream gas accumulator could not bracket high velocity",
            });
        }

        let mut low_velocity_m_s = high_velocity_m_s - 1.0;
        let mut low_residual = self.residual(
            config,
            downstream_compat_plus,
            wave_over_gravity,
            low_velocity_m_s,
        )?;
        for _ in 0..GAS_ACCUMULATOR_BRACKET_MAX_ITERS {
            if low_residual <= 0.0 {
                break;
            }
            low_velocity_m_s = 2.0 * low_velocity_m_s - high_velocity_m_s;
            low_residual = self.residual(
                config,
                downstream_compat_plus,
                wave_over_gravity,
                low_velocity_m_s,
            )?;
        }
        if low_residual > 0.0 {
            return Err(FeedSystemError::NonConverged {
                reason: "MOC line downstream gas accumulator could not bracket low velocity",
            });
        }

        for _ in 0..GAS_ACCUMULATOR_SOLVE_MAX_ITERS {
            let mid_velocity_m_s = 0.5 * (low_velocity_m_s + high_velocity_m_s);
            let mid_residual = self.residual(
                config,
                downstream_compat_plus,
                wave_over_gravity,
                mid_velocity_m_s,
            )?;
            if mid_residual <= 0.0 {
                low_velocity_m_s = mid_velocity_m_s;
            } else {
                high_velocity_m_s = mid_velocity_m_s;
            }
        }

        let velocity_m_s = 0.5 * (low_velocity_m_s + high_velocity_m_s);
        if !velocity_m_s.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line downstream gas accumulator velocity is non-finite",
            });
        }
        Ok(velocity_m_s)
    }
}

/// Snapshot from one MOC line step.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineSnapshot {
    /// Updated line state.
    pub state: MocLineState,
    /// Downstream valve-node head in m.
    pub downstream_head_m: f64,
    /// Downstream valve-node gauge pressure in Pa.
    pub downstream_pressure_pa: f64,
    /// Minimum head across the updated line in m.
    pub min_head_m: f64,
    /// Maximum head across the updated line in m.
    pub max_head_m: f64,
    /// Courant-exact step size in seconds for this update.
    pub dt_s: f64,
    /// Validation posture of the line model.
    pub validation: ValidationStatus,
}

/// Frictionless MOC line with fixed Courant number 1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MocLine {
    config: MocLineConfig,
}

impl MocLine {
    /// Construct a validated MOC line.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the line configuration is invalid.
    pub fn new(config: MocLineConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self { config })
    }

    /// Borrow the validated line configuration.
    #[must_use]
    pub const fn config(&self) -> MocLineConfig {
        self.config
    }

    /// Courant-exact time step `dx/a`.
    #[must_use]
    pub fn courant_dt_s(&self) -> f64 {
        self.config.courant_dt_s()
    }

    /// Advance the line by one Courant-exact MOC step.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid state/boundary values or
    /// non-finite characteristic updates.
    pub fn step(
        &self,
        state: &MocLineState,
        boundary: MocLineBoundary,
    ) -> Result<MocLineSnapshot, FeedSystemError> {
        self.step_with_boundaries(state, boundary.into_boundary_conditions())
    }

    /// Advance the line by one Courant-exact MOC step with generalized
    /// boundary conditions.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid state/boundary values or
    /// non-finite characteristic updates.
    pub fn step_with_boundaries(
        &self,
        state: &MocLineState,
        boundary: MocLineBoundaryConditions,
    ) -> Result<MocLineSnapshot, FeedSystemError> {
        self.step_with_endpoint_boundaries(state, boundary.into_endpoint_boundaries())
    }

    /// Advance the line by one Courant-exact MOC step with generalized
    /// upstream and downstream endpoint boundary conditions.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid state/boundary values or
    /// non-finite characteristic updates.
    pub fn step_with_endpoint_boundaries(
        &self,
        state: &MocLineState,
        boundary: MocLineEndpointBoundaryConditions,
    ) -> Result<MocLineSnapshot, FeedSystemError> {
        state.require_valid(self.config)?;
        boundary.require_valid()?;

        let node_count = self.config.node_count();
        let mut next_head_m = vec![0.0; node_count];
        let mut next_velocity_m_s = vec![0.0; node_count];
        let wave_over_gravity = self.config.wave_over_gravity();
        let gravity_over_wave = STANDARD_GRAVITY_M_S2 / self.config.wave_speed_m_s;

        let upstream_compat_minus = state.head_m[1] - wave_over_gravity * state.velocity_m_s[1];
        match boundary.upstream {
            MocLineUpstreamBoundary::Head { head_m } => {
                next_head_m[0] = head_m;
                next_velocity_m_s[0] = (head_m - upstream_compat_minus) * gravity_over_wave;
            }
            MocLineUpstreamBoundary::Velocity { velocity_m_s } => {
                next_velocity_m_s[0] = velocity_m_s;
                next_head_m[0] = upstream_compat_minus + wave_over_gravity * velocity_m_s;
            }
            MocLineUpstreamBoundary::GasSource {
                datum_head_m,
                gas_reference_volume_m3,
                gas_precharge_pressure_pa,
                delivered_liquid_volume_delta_m3,
                polytropic_exponent,
            } => {
                let boundary = GasSourceBoundary {
                    datum_head_m,
                    gas_reference_volume_m3,
                    gas_precharge_pressure_pa,
                    delivered_liquid_volume_delta_m3,
                    polytropic_exponent,
                };
                let velocity_m_s = boundary.solve_velocity(
                    self.config,
                    upstream_compat_minus,
                    wave_over_gravity,
                )?;
                next_velocity_m_s[0] = velocity_m_s;
                next_head_m[0] = boundary.head_m(self.config, velocity_m_s)?;
            }
        }

        for index in 1..(node_count - 1) {
            let compat_plus =
                state.head_m[index - 1] + wave_over_gravity * state.velocity_m_s[index - 1];
            let compat_minus =
                state.head_m[index + 1] - wave_over_gravity * state.velocity_m_s[index + 1];
            next_head_m[index] = 0.5 * (compat_plus + compat_minus);
            next_velocity_m_s[index] = 0.5 * (compat_plus - compat_minus) * gravity_over_wave;
        }

        let downstream_index = node_count - 1;
        let downstream_compat_plus = state.head_m[downstream_index - 1]
            + wave_over_gravity * state.velocity_m_s[downstream_index - 1];
        match boundary.downstream {
            MocLineDownstreamBoundary::Velocity { velocity_m_s } => {
                next_velocity_m_s[downstream_index] = velocity_m_s;
                next_head_m[downstream_index] =
                    downstream_compat_plus - wave_over_gravity * velocity_m_s;
            }
            MocLineDownstreamBoundary::Head { head_m } => {
                next_head_m[downstream_index] = head_m;
                next_velocity_m_s[downstream_index] =
                    (downstream_compat_plus - head_m) * gravity_over_wave;
            }
            MocLineDownstreamBoundary::Accumulator {
                reference_head_m,
                stored_volume_delta_m3,
                compliance_m3_per_m,
            } => {
                let dynamic_compliance_m =
                    self.config.cross_section_area_m2 * self.courant_dt_s() / compliance_m3_per_m;
                let static_head_m = reference_head_m + stored_volume_delta_m3 / compliance_m3_per_m;
                let velocity_m_s = (downstream_compat_plus - static_head_m)
                    / (wave_over_gravity + dynamic_compliance_m);
                next_velocity_m_s[downstream_index] = velocity_m_s;
                next_head_m[downstream_index] = static_head_m + dynamic_compliance_m * velocity_m_s;
            }
            MocLineDownstreamBoundary::GasAccumulator {
                datum_head_m,
                gas_reference_volume_m3,
                gas_precharge_pressure_pa,
                stored_liquid_volume_delta_m3,
                polytropic_exponent,
            } => {
                let boundary = GasAccumulatorBoundary {
                    datum_head_m,
                    gas_reference_volume_m3,
                    gas_precharge_pressure_pa,
                    stored_liquid_volume_delta_m3,
                    polytropic_exponent,
                };
                let velocity_m_s = boundary.solve_velocity(
                    self.config,
                    downstream_compat_plus,
                    wave_over_gravity,
                )?;
                next_velocity_m_s[downstream_index] = velocity_m_s;
                next_head_m[downstream_index] = boundary.head_m(self.config, velocity_m_s)?;
            }
        }

        for value in next_head_m.iter().chain(next_velocity_m_s.iter()) {
            if !value.is_finite() {
                return Err(FeedSystemError::NonFinite {
                    reason: "MOC line update produced a non-finite value",
                });
            }
        }

        let downstream_head_m = next_head_m[downstream_index];
        let downstream_pressure_pa = self.config.pressure_from_head_pa(downstream_head_m);
        if !downstream_pressure_pa.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line downstream pressure is non-finite",
            });
        }

        let min_head_m = next_head_m
            .iter()
            .fold(f64::INFINITY, |acc, value| acc.min(*value));
        let max_head_m = next_head_m
            .iter()
            .fold(f64::NEG_INFINITY, |acc, value| acc.max(*value));

        Ok(MocLineSnapshot {
            state: MocLineState {
                head_m: next_head_m,
                velocity_m_s: next_velocity_m_s,
            },
            downstream_head_m,
            downstream_pressure_pa,
            min_head_m,
            max_head_m,
            dt_s: self.courant_dt_s(),
            validation: ValidationStatus::ValidatedToy,
        })
    }
}

fn require_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    if !value.is_finite() {
        return Err(FeedSystemError::NonFinite { reason });
    }
    Ok(())
}

fn require_positive_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    require_finite(value, reason)?;
    if value <= 0.0 {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

fn require_non_empty_id(value: &str) -> Result<(), FeedSystemError> {
    if value.trim().is_empty() {
        return Err(FeedSystemError::InvalidParameter {
            reason: "MOC line topology id must be non-empty",
        });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use approx::assert_abs_diff_eq;
    use toml::value::Table;

    use super::*;

    fn config() -> MocLineConfig {
        MocLineConfig {
            length_m: 40.0,
            wave_speed_m_s: 1_000.0,
            density_kg_m3: 1_000.0,
            cross_section_area_m2: 0.01,
            segment_count: 4,
        }
    }

    #[test]
    fn moc_line_courant_dt_is_dx_over_wave_speed() {
        let line = MocLine::new(config()).unwrap();

        assert_abs_diff_eq!(line.courant_dt_s(), 0.01, epsilon = 1.0e-15);
    }

    #[test]
    fn moc_line_valve_closure_matches_joukowsky_bound() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();
        let snapshot = line
            .step(
                &state,
                MocLineBoundary {
                    upstream_head_m: 100.0,
                    downstream_velocity_m_s: 0.0,
                },
            )
            .unwrap();

        let expected_surge_head_m = config().wave_speed_m_s * 2.0 / STANDARD_GRAVITY_M_S2;
        assert_abs_diff_eq!(
            snapshot.downstream_head_m,
            100.0 + expected_surge_head_m,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.downstream_pressure_pa - 1_000.0 * STANDARD_GRAVITY_M_S2 * 100.0,
            1_000.0 * config().wave_speed_m_s * 2.0,
            epsilon = 1.0e-6
        );
        assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
    }

    #[test]
    fn moc_line_interior_node_uses_neighbor_characteristics() {
        let line = MocLine::new(config()).unwrap();
        let mut state = MocLineState::uniform(config(), 100.0, 1.0).unwrap();
        state.head_m[0] = 110.0;
        state.head_m[2] = 90.0;

        let snapshot = line
            .step(
                &state,
                MocLineBoundary {
                    upstream_head_m: 100.0,
                    downstream_velocity_m_s: 1.0,
                },
            )
            .unwrap();

        let wave_over_gravity = config().wave_speed_m_s / STANDARD_GRAVITY_M_S2;
        let compat_plus = 110.0 + wave_over_gravity;
        let compat_minus = 90.0 - wave_over_gravity;
        assert_abs_diff_eq!(
            snapshot.state.head_m[1],
            0.5 * (compat_plus + compat_minus),
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn moc_line_fixed_downstream_head_solves_compatible_velocity() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();
        let snapshot = line
            .step_with_boundaries(
                &state,
                MocLineBoundaryConditions {
                    upstream_head_m: 100.0,
                    downstream: MocLineDownstreamBoundary::Head { head_m: 125.0 },
                },
            )
            .unwrap();

        let wave_over_gravity = config().wave_speed_m_s / STANDARD_GRAVITY_M_S2;
        let expected_velocity_m_s = (100.0 + wave_over_gravity * 2.0 - 125.0)
            * STANDARD_GRAVITY_M_S2
            / config().wave_speed_m_s;
        assert_abs_diff_eq!(snapshot.downstream_head_m, 125.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(
            snapshot.state.velocity_m_s[config().segment_count],
            expected_velocity_m_s,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn moc_line_fixed_upstream_velocity_solves_compatible_head() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();
        let snapshot = line
            .step_with_endpoint_boundaries(
                &state,
                MocLineEndpointBoundaryConditions {
                    upstream: MocLineUpstreamBoundary::Velocity { velocity_m_s: 0.5 },
                    downstream: MocLineDownstreamBoundary::Velocity { velocity_m_s: 2.0 },
                },
            )
            .unwrap();

        let wave_over_gravity = config().wave_speed_m_s / STANDARD_GRAVITY_M_S2;
        let expected_head_m = 100.0 - wave_over_gravity * 2.0 + wave_over_gravity * 0.5;
        assert_abs_diff_eq!(snapshot.state.velocity_m_s[0], 0.5, epsilon = 1.0e-12);
        assert_abs_diff_eq!(snapshot.state.head_m[0], expected_head_m, epsilon = 1.0e-12);
    }

    #[test]
    fn moc_line_upstream_gas_source_solves_polytropic_boundary() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();
        let gas_reference_volume_m3 = 0.02;
        let gas_precharge_pressure_pa = 1_000_000.0;
        let polytropic_exponent = 1.2;
        let snapshot = line
            .step_with_endpoint_boundaries(
                &state,
                MocLineEndpointBoundaryConditions {
                    upstream: MocLineUpstreamBoundary::GasSource {
                        datum_head_m: 0.0,
                        gas_reference_volume_m3,
                        gas_precharge_pressure_pa,
                        delivered_liquid_volume_delta_m3: 0.0,
                        polytropic_exponent,
                    },
                    downstream: MocLineDownstreamBoundary::Velocity { velocity_m_s: 2.0 },
                },
            )
            .unwrap();

        let upstream_velocity_m_s = snapshot.state.velocity_m_s[0];
        let gas_volume_m3 = gas_reference_volume_m3
            + config().cross_section_area_m2 * line.courant_dt_s() * upstream_velocity_m_s;
        let expected_head_m = gas_precharge_pressure_pa
            * (gas_reference_volume_m3 / gas_volume_m3).powf(polytropic_exponent)
            / (config().density_kg_m3 * STANDARD_GRAVITY_M_S2);
        let wave_over_gravity = config().wave_speed_m_s / STANDARD_GRAVITY_M_S2;
        let characteristic_head_m =
            100.0 - wave_over_gravity * 2.0 + wave_over_gravity * upstream_velocity_m_s;

        assert!(gas_volume_m3 > gas_reference_volume_m3);
        assert_abs_diff_eq!(snapshot.state.head_m[0], expected_head_m, epsilon = 1.0e-10);
        assert_abs_diff_eq!(
            snapshot.state.head_m[0],
            characteristic_head_m,
            epsilon = 1.0e-10
        );
        assert!(
            snapshot.state.head_m[0]
                < gas_precharge_pressure_pa / (config().density_kg_m3 * STANDARD_GRAVITY_M_S2),
            "positive outflow should expand source gas and lower head"
        );
    }

    #[test]
    fn moc_line_downstream_accumulator_solves_characteristic_compliance() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();
        let compliance_m3_per_m = 1.0e-5;
        let snapshot = line
            .step_with_endpoint_boundaries(
                &state,
                MocLineEndpointBoundaryConditions {
                    upstream: MocLineUpstreamBoundary::Head { head_m: 100.0 },
                    downstream: MocLineDownstreamBoundary::Accumulator {
                        reference_head_m: 100.0,
                        stored_volume_delta_m3: 0.0,
                        compliance_m3_per_m,
                    },
                },
            )
            .unwrap();

        let wave_over_gravity = config().wave_speed_m_s / STANDARD_GRAVITY_M_S2;
        let dynamic_compliance_m =
            config().cross_section_area_m2 * line.courant_dt_s() / compliance_m3_per_m;
        let compat_plus = 100.0 + wave_over_gravity * 2.0;
        let expected_velocity_m_s =
            (compat_plus - 100.0) / (wave_over_gravity + dynamic_compliance_m);
        let expected_head_m = 100.0 + dynamic_compliance_m * expected_velocity_m_s;

        assert_abs_diff_eq!(
            snapshot.state.velocity_m_s[config().segment_count],
            expected_velocity_m_s,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.downstream_head_m,
            expected_head_m,
            epsilon = 1.0e-12
        );
        assert!(
            snapshot.downstream_head_m > 100.0,
            "positive inflow should charge accumulator head"
        );
    }

    #[test]
    fn moc_line_downstream_gas_accumulator_solves_polytropic_boundary() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();
        let gas_reference_volume_m3 = 0.02;
        let gas_precharge_pressure_pa = 1_000_000.0;
        let polytropic_exponent = 1.2;
        let snapshot = line
            .step_with_endpoint_boundaries(
                &state,
                MocLineEndpointBoundaryConditions {
                    upstream: MocLineUpstreamBoundary::Head { head_m: 100.0 },
                    downstream: MocLineDownstreamBoundary::GasAccumulator {
                        datum_head_m: 0.0,
                        gas_reference_volume_m3,
                        gas_precharge_pressure_pa,
                        stored_liquid_volume_delta_m3: 0.0,
                        polytropic_exponent,
                    },
                },
            )
            .unwrap();

        let downstream_velocity_m_s = snapshot.state.velocity_m_s[config().segment_count];
        let gas_volume_m3 = gas_reference_volume_m3
            - config().cross_section_area_m2 * line.courant_dt_s() * downstream_velocity_m_s;
        let expected_head_m = gas_precharge_pressure_pa
            * (gas_reference_volume_m3 / gas_volume_m3).powf(polytropic_exponent)
            / (config().density_kg_m3 * STANDARD_GRAVITY_M_S2);
        let wave_over_gravity = config().wave_speed_m_s / STANDARD_GRAVITY_M_S2;
        let characteristic_head_m =
            100.0 + wave_over_gravity * 2.0 - wave_over_gravity * downstream_velocity_m_s;

        assert!(gas_volume_m3 > 0.0);
        assert_abs_diff_eq!(
            snapshot.downstream_head_m,
            expected_head_m,
            epsilon = 1.0e-10
        );
        assert_abs_diff_eq!(
            snapshot.downstream_head_m,
            characteristic_head_m,
            epsilon = 1.0e-10
        );
        assert!(
            snapshot.downstream_head_m
                > gas_precharge_pressure_pa / (config().density_kg_m3 * STANDARD_GRAVITY_M_S2),
            "positive inflow should compress gas and raise head"
        );
    }

    #[test]
    fn moc_line_generalized_boundary_rejects_non_finite_fixed_head() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();

        assert!(matches!(
            line.step_with_boundaries(
                &state,
                MocLineBoundaryConditions {
                    upstream_head_m: 100.0,
                    downstream: MocLineDownstreamBoundary::Head { head_m: f64::NAN },
                },
            ),
            Err(FeedSystemError::NonFinite { .. })
        ));
    }

    #[test]
    fn moc_line_endpoint_boundary_rejects_invalid_accumulator() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();

        assert!(matches!(
            line.step_with_endpoint_boundaries(
                &state,
                MocLineEndpointBoundaryConditions {
                    upstream: MocLineUpstreamBoundary::Head { head_m: 100.0 },
                    downstream: MocLineDownstreamBoundary::Accumulator {
                        reference_head_m: 100.0,
                        stored_volume_delta_m3: 0.0,
                        compliance_m3_per_m: 0.0,
                    },
                },
            ),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn moc_line_endpoint_boundary_rejects_invalid_gas_accumulator() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();

        assert!(matches!(
            line.step_with_endpoint_boundaries(
                &state,
                MocLineEndpointBoundaryConditions {
                    upstream: MocLineUpstreamBoundary::Head { head_m: 100.0 },
                    downstream: MocLineDownstreamBoundary::GasAccumulator {
                        datum_head_m: 0.0,
                        gas_reference_volume_m3: 0.01,
                        gas_precharge_pressure_pa: 1_000_000.0,
                        stored_liquid_volume_delta_m3: 0.01,
                        polytropic_exponent: 1.2,
                    },
                },
            ),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn moc_line_endpoint_boundary_rejects_invalid_gas_source() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();

        assert!(matches!(
            line.step_with_endpoint_boundaries(
                &state,
                MocLineEndpointBoundaryConditions {
                    upstream: MocLineUpstreamBoundary::GasSource {
                        datum_head_m: 0.0,
                        gas_reference_volume_m3: 0.01,
                        gas_precharge_pressure_pa: 1_000_000.0,
                        delivered_liquid_volume_delta_m3: -0.01,
                        polytropic_exponent: 1.2,
                    },
                    downstream: MocLineDownstreamBoundary::Velocity { velocity_m_s: 2.0 },
                },
            ),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn moc_line_endpoint_boundary_rejects_non_finite_upstream_velocity() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();

        assert!(matches!(
            line.step_with_endpoint_boundaries(
                &state,
                MocLineEndpointBoundaryConditions {
                    upstream: MocLineUpstreamBoundary::Velocity {
                        velocity_m_s: f64::NAN
                    },
                    downstream: MocLineDownstreamBoundary::Velocity { velocity_m_s: 2.0 },
                },
            ),
            Err(FeedSystemError::NonFinite { .. })
        ));
    }

    #[test]
    fn moc_line_topology_steps_named_lines_independently() {
        let oxidizer = topology_element("oxidizer", config(), 100.0, 2.0, 0.0);
        let fuel_config = MocLineConfig {
            length_m: 18.0,
            wave_speed_m_s: 900.0,
            density_kg_m3: 720.0,
            cross_section_area_m2: 0.008,
            segment_count: 2,
        };
        let fuel = topology_element("fuel", fuel_config, 80.0, 1.5, 0.5);
        let mut topology = MocLineTopology::new(MocLineTopologyConfig {
            lines: vec![oxidizer, fuel],
        })
        .unwrap();

        assert_eq!(topology.line_count(), 2);
        assert_abs_diff_eq!(topology.courant_dt_s(), 0.01, epsilon = 1.0e-15);
        assert_eq!(
            topology.line_ids().collect::<Vec<_>>(),
            vec!["oxidizer", "fuel"]
        );
        topology
            .set_boundary(
                "fuel",
                MocLineEndpointBoundaryConditions {
                    upstream: MocLineUpstreamBoundary::Head { head_m: 80.0 },
                    downstream: MocLineDownstreamBoundary::Velocity { velocity_m_s: 0.25 },
                },
            )
            .unwrap();

        let snapshot = topology.step().unwrap();
        let oxidizer_snapshot = snapshot.line("oxidizer").unwrap();
        let fuel_snapshot = snapshot.line("fuel").unwrap();
        let expected_oxidizer_head_m =
            100.0 + config().wave_speed_m_s * 2.0 / STANDARD_GRAVITY_M_S2;
        let expected_fuel_head_m =
            80.0 + fuel_config.wave_speed_m_s * (1.5 - 0.25) / STANDARD_GRAVITY_M_S2;

        assert_eq!(snapshot.lines.len(), 2);
        assert_abs_diff_eq!(snapshot.dt_s, 0.01, epsilon = 1.0e-15);
        assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
        assert_abs_diff_eq!(
            oxidizer_snapshot.downstream_head_m,
            expected_oxidizer_head_m,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            fuel_snapshot.downstream_head_m,
            expected_fuel_head_m,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            topology.state("fuel").unwrap().head_m[fuel_config.segment_count],
            expected_fuel_head_m,
            epsilon = 1.0e-12
        );
        assert_eq!(
            topology.boundary("fuel").unwrap(),
            MocLineEndpointBoundaryConditions {
                upstream: MocLineUpstreamBoundary::Head { head_m: 80.0 },
                downstream: MocLineDownstreamBoundary::Velocity { velocity_m_s: 0.25 },
            }
        );
    }

    #[test]
    fn moc_line_topology_rejects_duplicate_ids() {
        let element = topology_element("shared", config(), 100.0, 2.0, 0.0);

        assert!(matches!(
            MocLineTopology::new(MocLineTopologyConfig {
                lines: vec![element.clone(), element],
            }),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn moc_line_topology_rejects_invalid_initial_state() {
        let mut element = topology_element("line", config(), 100.0, 2.0, 0.0);
        element.initial_state.head_m.pop();

        assert!(matches!(
            MocLineTopology::new(MocLineTopologyConfig {
                lines: vec![element],
            }),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn moc_line_topology_rejects_mismatched_courant_steps() {
        let slow_config = MocLineConfig {
            length_m: 20.0,
            wave_speed_m_s: 900.0,
            density_kg_m3: 720.0,
            cross_section_area_m2: 0.008,
            segment_count: 2,
        };

        assert!(matches!(
            MocLineTopology::new(MocLineTopologyConfig {
                lines: vec![
                    topology_element("oxidizer", config(), 100.0, 2.0, 0.0),
                    topology_element("fuel", slow_config, 80.0, 1.5, 0.5),
                ],
            }),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn moc_line_topology_rejects_unknown_boundary_update() {
        let mut topology = MocLineTopology::new(MocLineTopologyConfig {
            lines: vec![topology_element("line", config(), 100.0, 2.0, 0.0)],
        })
        .unwrap();

        assert!(matches!(
            topology.set_boundary(
                "missing",
                MocLineEndpointBoundaryConditions {
                    upstream: MocLineUpstreamBoundary::Head { head_m: 100.0 },
                    downstream: MocLineDownstreamBoundary::Velocity { velocity_m_s: 0.0 },
                },
            ),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn moc_line_graph_topology_solves_three_line_junction_mass_balance() {
        let mut graph = MocLineGraphTopology::new(MocLineGraphTopologyConfig {
            lines: vec![
                topology_element("inlet", config(), 100.0, 2.0, 2.0),
                topology_element("branch-a", config(), 100.0, 0.0, 0.0),
                topology_element("branch-b", config(), 100.0, 0.0, 0.0),
            ],
            junctions: vec![MocLineJunctionConfig {
                id: "tee".to_owned(),
                connections: vec![
                    MocLineJunctionConnection {
                        line_id: "inlet".to_owned(),
                        endpoint: MocLineEndpointKind::Downstream,
                    },
                    MocLineJunctionConnection {
                        line_id: "branch-a".to_owned(),
                        endpoint: MocLineEndpointKind::Upstream,
                    },
                    MocLineJunctionConnection {
                        line_id: "branch-b".to_owned(),
                        endpoint: MocLineEndpointKind::Upstream,
                    },
                ],
            }],
        })
        .unwrap();

        assert_eq!(graph.line_count(), 3);
        assert_eq!(graph.junction_count(), 1);
        assert_eq!(
            graph.line_ids().collect::<Vec<_>>(),
            vec!["inlet", "branch-a", "branch-b"]
        );

        let snapshot = graph.step().unwrap();
        let junction = snapshot.junction("tee").unwrap();
        let wave_over_gravity = config().wave_speed_m_s / STANDARD_GRAVITY_M_S2;
        let expected_head_m = 100.0 + (2.0 * wave_over_gravity) / 3.0;

        assert_eq!(snapshot.lines.len(), 3);
        assert_eq!(snapshot.junctions.len(), 1);
        assert_abs_diff_eq!(snapshot.dt_s, 0.01, epsilon = 1.0e-15);
        assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
        assert_abs_diff_eq!(junction.head_m, expected_head_m, epsilon = 1.0e-12);
        assert_abs_diff_eq!(junction.net_flow_m3_s, 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(
            snapshot.line("inlet").unwrap().downstream_head_m,
            expected_head_m,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.line("branch-a").unwrap().state.head_m[0],
            expected_head_m,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.line("inlet").unwrap().state.velocity_m_s[config().segment_count],
            4.0 / 3.0,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.line("branch-a").unwrap().state.velocity_m_s[0],
            2.0 / 3.0,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            graph.state("branch-b").unwrap().velocity_m_s[0],
            2.0 / 3.0,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn moc_line_graph_topology_rejects_unknown_junction_line() {
        assert!(matches!(
            MocLineGraphTopology::new(MocLineGraphTopologyConfig {
                lines: vec![topology_element("line", config(), 100.0, 2.0, 0.0)],
                junctions: vec![MocLineJunctionConfig {
                    id: "bad".to_owned(),
                    connections: vec![
                        MocLineJunctionConnection {
                            line_id: "line".to_owned(),
                            endpoint: MocLineEndpointKind::Downstream,
                        },
                        MocLineJunctionConnection {
                            line_id: "missing".to_owned(),
                            endpoint: MocLineEndpointKind::Upstream,
                        },
                    ],
                }],
            }),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn moc_line_graph_topology_rejects_reused_endpoint() {
        assert!(matches!(
            MocLineGraphTopology::new(MocLineGraphTopologyConfig {
                lines: vec![
                    topology_element("a", config(), 100.0, 2.0, 0.0),
                    topology_element("b", config(), 100.0, 0.0, 0.0),
                    topology_element("c", config(), 100.0, 0.0, 0.0),
                ],
                junctions: vec![
                    MocLineJunctionConfig {
                        id: "j1".to_owned(),
                        connections: vec![
                            MocLineJunctionConnection {
                                line_id: "a".to_owned(),
                                endpoint: MocLineEndpointKind::Downstream,
                            },
                            MocLineJunctionConnection {
                                line_id: "b".to_owned(),
                                endpoint: MocLineEndpointKind::Upstream,
                            },
                        ],
                    },
                    MocLineJunctionConfig {
                        id: "j2".to_owned(),
                        connections: vec![
                            MocLineJunctionConnection {
                                line_id: "a".to_owned(),
                                endpoint: MocLineEndpointKind::Downstream,
                            },
                            MocLineJunctionConnection {
                                line_id: "c".to_owned(),
                                endpoint: MocLineEndpointKind::Upstream,
                            },
                        ],
                    },
                ],
            }),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn moc_line_matches_provenance_tolerance_table() {
        let data: toml::Value = toml::from_str(include_str!(
            "../../../data/feed_system/generic-moc-line-v1.toml"
        ))
        .unwrap();
        assert_eq!(
            table("openbmp", &data)["feed_moc_line"]
                .as_integer()
                .unwrap(),
            1
        );
        let config = MocLineConfig {
            length_m: float(table("config", &data), "length_m"),
            wave_speed_m_s: float(table("config", &data), "wave_speed_m_s"),
            density_kg_m3: float(table("config", &data), "density_kg_m3"),
            cross_section_area_m2: float(table("config", &data), "cross_section_area_m2"),
            segment_count: integer(table("config", &data), "segment_count") as usize,
        };
        let line = MocLine::new(config).unwrap();
        let tolerances = table("tolerances", &data);

        for case in data
            .get("case")
            .and_then(toml::Value::as_array)
            .expect("case array")
        {
            let case = case.as_table().unwrap();
            let state = MocLineState::uniform(
                config,
                float(case, "initial_head_m"),
                float(case, "initial_velocity_m_s"),
            )
            .unwrap();
            let snapshot = line
                .step(
                    &state,
                    MocLineBoundary {
                        upstream_head_m: float(case, "upstream_head_m"),
                        downstream_velocity_m_s: float(case, "downstream_velocity_m_s"),
                    },
                )
                .unwrap();

            assert_abs_diff_eq!(
                snapshot.dt_s,
                float(case, "expected_dt_s"),
                epsilon = float(tolerances, "absolute_dt_s")
            );
            assert_abs_diff_eq!(
                snapshot.downstream_head_m,
                float(case, "expected_downstream_head_m"),
                epsilon = float(tolerances, "absolute_head_m")
            );
            assert_abs_diff_eq!(
                snapshot.downstream_pressure_pa,
                float(case, "expected_downstream_pressure_pa"),
                epsilon = float(tolerances, "absolute_pressure_pa")
            );
            assert_abs_diff_eq!(
                snapshot.downstream_pressure_pa
                    - config.pressure_from_head_pa(float(case, "initial_head_m")),
                float(case, "expected_pressure_delta_pa"),
                epsilon = float(tolerances, "absolute_pressure_pa")
            );
            assert_abs_diff_eq!(
                snapshot.downstream_head_m - float(case, "initial_head_m"),
                float(case, "expected_joukowsky_head_delta_m"),
                epsilon = float(tolerances, "absolute_head_m")
            );
            assert_abs_diff_eq!(
                snapshot.min_head_m,
                float(case, "expected_min_head_m"),
                epsilon = float(tolerances, "absolute_head_m")
            );
            assert_abs_diff_eq!(
                snapshot.max_head_m,
                float(case, "expected_max_head_m"),
                epsilon = float(tolerances, "absolute_head_m")
            );
            assert_eq!(string(case, "expected_validation"), "validated-toy");
            assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
        }
    }

    #[test]
    fn moc_line_matches_public_joukowsky_valve_closure_example() {
        let data: toml::Value = toml::from_str(include_str!(
            "../../../data/feed_system/public-joukowsky-valve-closure-v1.toml"
        ))
        .unwrap();
        assert_eq!(
            table("openbmp", &data)["feed_moc_line_public_joukowsky"]
                .as_integer()
                .unwrap(),
            1
        );
        let config = MocLineConfig {
            length_m: float(table("config", &data), "length_m"),
            wave_speed_m_s: float(table("config", &data), "wave_speed_m_s"),
            density_kg_m3: float(table("config", &data), "density_kg_m3"),
            cross_section_area_m2: float(table("config", &data), "cross_section_area_m2"),
            segment_count: integer(table("config", &data), "segment_count") as usize,
        };
        let line = MocLine::new(config).unwrap();
        let tolerances = table("tolerances", &data);
        let source_example = table("source_example", &data);

        for case in data
            .get("case")
            .and_then(toml::Value::as_array)
            .expect("case array")
        {
            let case = case.as_table().unwrap();
            let initial_head_m = float(case, "initial_head_m");
            let state =
                MocLineState::uniform(config, initial_head_m, float(case, "initial_velocity_m_s"))
                    .unwrap();
            let snapshot = line
                .step(
                    &state,
                    MocLineBoundary {
                        upstream_head_m: float(case, "upstream_head_m"),
                        downstream_velocity_m_s: float(case, "downstream_velocity_m_s"),
                    },
                )
                .unwrap();

            assert_abs_diff_eq!(
                snapshot.dt_s,
                float(case, "expected_dt_s"),
                epsilon = float(tolerances, "absolute_dt_s")
            );
            assert_abs_diff_eq!(
                snapshot.downstream_head_m,
                float(case, "expected_downstream_head_m"),
                epsilon = float(tolerances, "absolute_head_m")
            );
            assert_abs_diff_eq!(
                snapshot.downstream_pressure_pa,
                float(case, "expected_downstream_pressure_pa"),
                epsilon = float(tolerances, "absolute_pressure_pa")
            );
            let pressure_delta_pa =
                snapshot.downstream_pressure_pa - config.pressure_from_head_pa(initial_head_m);
            let head_delta_m = snapshot.downstream_head_m - initial_head_m;
            assert_abs_diff_eq!(
                pressure_delta_pa,
                float(case, "expected_pressure_delta_pa"),
                epsilon = float(tolerances, "absolute_pressure_pa")
            );
            assert_abs_diff_eq!(
                head_delta_m,
                float(case, "expected_joukowsky_head_delta_m"),
                epsilon = float(tolerances, "absolute_head_m")
            );
            assert_abs_diff_eq!(
                head_delta_m,
                float(source_example, "reported_head_rise_m"),
                epsilon = float(tolerances, "published_absolute_head_m")
            );
            assert_abs_diff_eq!(
                pressure_delta_pa / 1000.0,
                float(source_example, "reported_pressure_rise_kpa"),
                epsilon = float(tolerances, "published_absolute_pressure_kpa")
            );
            assert_abs_diff_eq!(
                snapshot.min_head_m,
                float(case, "expected_min_head_m"),
                epsilon = float(tolerances, "absolute_head_m")
            );
            assert_abs_diff_eq!(
                snapshot.max_head_m,
                float(case, "expected_max_head_m"),
                epsilon = float(tolerances, "absolute_head_m")
            );
            assert_eq!(string(case, "expected_validation"), "validated-toy");
            assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
        }
    }

    #[test]
    fn moc_line_rejects_invalid_state_shape() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState {
            head_m: vec![100.0],
            velocity_m_s: vec![1.0],
        };

        assert!(matches!(
            line.step(
                &state,
                MocLineBoundary {
                    upstream_head_m: 100.0,
                    downstream_velocity_m_s: 0.0,
                },
            ),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    fn table<'a>(key: &str, value: &'a toml::Value) -> &'a Table {
        value.get(key).and_then(toml::Value::as_table).unwrap()
    }

    fn float(table: &Table, key: &str) -> f64 {
        table.get(key).and_then(toml::Value::as_float).unwrap()
    }

    fn integer(table: &Table, key: &str) -> i64 {
        table.get(key).and_then(toml::Value::as_integer).unwrap()
    }

    fn string<'a>(table: &'a Table, key: &str) -> &'a str {
        table.get(key).and_then(toml::Value::as_str).unwrap()
    }

    fn topology_element(
        id: &str,
        config: MocLineConfig,
        initial_head_m: f64,
        initial_velocity_m_s: f64,
        downstream_velocity_m_s: f64,
    ) -> MocLineElementConfig {
        MocLineElementConfig::uniform(
            id,
            config,
            initial_head_m,
            initial_velocity_m_s,
            MocLineEndpointBoundaryConditions {
                upstream: MocLineUpstreamBoundary::Head {
                    head_m: initial_head_m,
                },
                downstream: MocLineDownstreamBoundary::Velocity {
                    velocity_m_s: downstream_velocity_m_s,
                },
            },
        )
        .unwrap()
    }
}
