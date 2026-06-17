//! Steady node/branch feed-network graph.

use openbmp_core::ValidationStatus;

use crate::error::FeedSystemError;

const DEFAULT_MAX_NEWTON_ITERATIONS: usize = 64;
const DEFAULT_RESIDUAL_TOLERANCE_KG_PER_S: f64 = 1.0e-8;
const DERIVATIVE_DELTA_P_FLOOR_PA: f64 = 1.0;
const MIN_DAMPING_STEPS: usize = 32;

/// Node declaration for a steady feed-network graph.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedGraphNode {
    /// Node kind and closure.
    pub kind: FeedGraphNodeKind,
}

/// Supported steady graph node kinds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FeedGraphNodeKind {
    /// Fixed-pressure boundary, e.g. a regulated tank outlet.
    PressureBoundary {
        /// Fixed node pressure in Pa.
        pressure_pa: f64,
    },
    /// Algebraic internal junction with zero net mass accumulation.
    Junction {
        /// Initial Newton guess in Pa.
        initial_pressure_pa: f64,
    },
    /// Algebraic chamber node with choked throat outflow `pc * At / c*`.
    Chamber {
        /// Initial Newton guess in Pa.
        initial_pressure_pa: f64,
        /// Throat area in m^2.
        throat_area_m2: f64,
        /// Characteristic velocity in m/s.
        c_star_m_s: f64,
    },
}

impl FeedGraphNodeKind {
    fn require_valid(self) -> Result<(), FeedSystemError> {
        match self {
            Self::PressureBoundary { pressure_pa } => require_positive_finite(
                pressure_pa,
                "pressure-boundary node pressure must be positive and finite",
            ),
            Self::Junction {
                initial_pressure_pa,
            } => require_nonnegative_finite(
                initial_pressure_pa,
                "junction initial pressure must be finite and non-negative",
            ),
            Self::Chamber {
                initial_pressure_pa,
                throat_area_m2,
                c_star_m_s,
            } => {
                require_nonnegative_finite(
                    initial_pressure_pa,
                    "chamber initial pressure must be finite and non-negative",
                )?;
                require_positive_finite(throat_area_m2, "chamber throat area must be positive")?;
                require_positive_finite(c_star_m_s, "chamber c_star must be positive")
            }
        }
    }

    fn is_unknown(self) -> bool {
        !matches!(self, Self::PressureBoundary { .. })
    }

    fn initial_pressure_pa(self) -> f64 {
        match self {
            Self::PressureBoundary { pressure_pa } => pressure_pa,
            Self::Junction {
                initial_pressure_pa,
            }
            | Self::Chamber {
                initial_pressure_pa,
                ..
            } => initial_pressure_pa,
        }
    }
}

/// Incompressible valve branch between two pressure nodes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ValveBranch {
    /// Upstream endpoint index for positive flow.
    pub from: usize,
    /// Downstream endpoint index for positive flow.
    pub to: usize,
    /// Full-open flow area in m^2.
    pub area_m2: f64,
    /// Discharge coefficient in `(0, 1]`.
    pub discharge_coefficient: f64,
    /// Single-fluid density in kg/m^3.
    pub density_kg_m3: f64,
    /// Fixed valve opening fraction in `[0, 1]`.
    pub open_fraction: f64,
}

impl ValveBranch {
    fn require_valid(self, node_count: usize) -> Result<(), FeedSystemError> {
        if self.from >= node_count || self.to >= node_count {
            return Err(FeedSystemError::InvalidParameter {
                reason: "valve branch endpoint index is outside the node list",
            });
        }
        if self.from == self.to {
            return Err(FeedSystemError::InvalidParameter {
                reason: "valve branch endpoints must be distinct",
            });
        }
        require_positive_finite(self.area_m2, "valve branch area must be positive")?;
        require_positive_finite(self.density_kg_m3, "valve branch density must be positive")?;
        require_positive_finite(
            self.discharge_coefficient,
            "valve discharge coefficient must be positive",
        )?;
        if self.discharge_coefficient > 1.0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "valve discharge coefficient must not exceed 1",
            });
        }
        if !self.open_fraction.is_finite() || !(0.0..=1.0).contains(&self.open_fraction) {
            return Err(FeedSystemError::InvalidParameter {
                reason: "valve open fraction must lie in [0, 1]",
            });
        }
        Ok(())
    }

    fn coefficient(self) -> f64 {
        self.discharge_coefficient
            * self.area_m2
            * self.open_fraction
            * (2.0 * self.density_kg_m3).sqrt()
    }
}

/// Validated steady feed-network graph configuration.
#[derive(Clone, Debug, PartialEq)]
pub struct SteadyFeedGraphConfig {
    /// Pressure nodes.
    pub nodes: Vec<FeedGraphNode>,
    /// Valve branches.
    pub valve_branches: Vec<ValveBranch>,
    /// Fixed Newton iteration cap.
    pub max_newton_iterations: usize,
    /// Absolute mass-balance residual tolerance in kg/s.
    pub residual_tolerance_kg_per_s: f64,
}

impl SteadyFeedGraphConfig {
    /// Construct with default Newton controls.
    #[must_use]
    pub fn new(nodes: Vec<FeedGraphNode>, valve_branches: Vec<ValveBranch>) -> Self {
        Self {
            nodes,
            valve_branches,
            max_newton_iterations: DEFAULT_MAX_NEWTON_ITERATIONS,
            residual_tolerance_kg_per_s: DEFAULT_RESIDUAL_TOLERANCE_KG_PER_S,
        }
    }

    /// Validate graph shape and scalar envelopes.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for malformed graph topology, non-finite
    /// parameters, or unsupported solver controls.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        if self.nodes.is_empty() {
            return Err(FeedSystemError::InvalidParameter {
                reason: "feed graph must declare at least one node",
            });
        }
        if self.valve_branches.is_empty() {
            return Err(FeedSystemError::InvalidParameter {
                reason: "feed graph must declare at least one valve branch",
            });
        }
        if self.max_newton_iterations == 0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "feed graph Newton iteration cap must be positive",
            });
        }
        require_positive_finite(
            self.residual_tolerance_kg_per_s,
            "feed graph residual tolerance must be positive",
        )?;

        let mut has_boundary = false;
        let mut has_unknown = false;
        for node in &self.nodes {
            node.kind.require_valid()?;
            has_boundary |= matches!(node.kind, FeedGraphNodeKind::PressureBoundary { .. });
            has_unknown |= node.kind.is_unknown();
        }
        if !has_boundary || !has_unknown {
            return Err(FeedSystemError::InvalidParameter {
                reason: "feed graph requires at least one pressure boundary and one unknown node",
            });
        }
        for branch in &self.valve_branches {
            branch.require_valid(self.nodes.len())?;
        }
        Ok(())
    }
}

/// Steady feed-network graph with valve branches and chamber throat closures.
#[derive(Clone, Debug, PartialEq)]
pub struct SteadyFeedGraph {
    config: SteadyFeedGraphConfig,
    unknown_nodes: Vec<usize>,
    unknown_by_node: Vec<Option<usize>>,
}

impl SteadyFeedGraph {
    /// Build a validated steady feed graph.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the graph is malformed.
    pub fn new(config: SteadyFeedGraphConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        let mut unknown_nodes = Vec::new();
        let mut unknown_by_node = vec![None; config.nodes.len()];
        for (node_index, node) in config.nodes.iter().enumerate() {
            if node.kind.is_unknown() {
                let unknown_index = unknown_nodes.len();
                unknown_nodes.push(node_index);
                unknown_by_node[node_index] = Some(unknown_index);
            }
        }
        Ok(Self {
            config,
            unknown_nodes,
            unknown_by_node,
        })
    }

    /// Borrow the validated graph config.
    #[must_use]
    pub fn config(&self) -> &SteadyFeedGraphConfig {
        &self.config
    }

    /// Solve the steady graph.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite residuals, singular Newton
    /// systems, or non-convergence inside the fixed iteration cap.
    pub fn solve(&self) -> Result<SteadyFeedGraphSnapshot, FeedSystemError> {
        let mut unknown_pressures_pa: Vec<f64> = self
            .unknown_nodes
            .iter()
            .map(|node_index| self.config.nodes[*node_index].kind.initial_pressure_pa())
            .collect();

        for _ in 0..self.config.max_newton_iterations {
            let (residuals, jacobian, max_residual) =
                self.residual_and_jacobian(&unknown_pressures_pa)?;
            if max_residual <= self.config.residual_tolerance_kg_per_s {
                return self.snapshot(&unknown_pressures_pa, max_residual);
            }
            let rhs: Vec<f64> = residuals.iter().map(|value| -*value).collect();
            let delta = solve_dense_linear(jacobian, rhs)?;
            damped_update(&mut unknown_pressures_pa, &delta)?;
        }

        let (residuals, _, max_residual) = self.residual_and_jacobian(&unknown_pressures_pa)?;
        if max_residual <= self.config.residual_tolerance_kg_per_s {
            self.snapshot(&unknown_pressures_pa, max_abs(&residuals))
        } else {
            Err(FeedSystemError::NonConverged {
                reason: "steady feed graph exceeded the Newton residual tolerance",
            })
        }
    }

    fn node_pressures(&self, unknown_pressures_pa: &[f64]) -> Vec<f64> {
        self.config
            .nodes
            .iter()
            .enumerate()
            .map(
                |(node_index, node)| match self.unknown_by_node[node_index] {
                    Some(unknown_index) => unknown_pressures_pa[unknown_index],
                    None => node.kind.initial_pressure_pa(),
                },
            )
            .collect()
    }

    fn residual_and_jacobian(
        &self,
        unknown_pressures_pa: &[f64],
    ) -> Result<(Vec<f64>, Vec<f64>, f64), FeedSystemError> {
        let unknown_count = self.unknown_nodes.len();
        let node_pressures_pa = self.node_pressures(unknown_pressures_pa);
        let mut residuals = vec![0.0; unknown_count];
        let mut jacobian = vec![0.0; unknown_count * unknown_count];

        for branch in &self.config.valve_branches {
            let pressure_delta_pa = node_pressures_pa[branch.from] - node_pressures_pa[branch.to];
            let flow_kg_per_s = signed_valve_flow(*branch, pressure_delta_pa);
            let derivative = valve_flow_derivative(*branch, pressure_delta_pa);
            if !flow_kg_per_s.is_finite() || !derivative.is_finite() {
                return Err(FeedSystemError::NonFinite {
                    reason: "steady feed graph valve branch produced a non-finite value",
                });
            }
            accumulate_branch(
                &mut residuals,
                &mut jacobian,
                &self.unknown_by_node,
                *branch,
                flow_kg_per_s,
                derivative,
                unknown_count,
            );
        }

        for (node_index, node) in self.config.nodes.iter().enumerate() {
            let Some(unknown_index) = self.unknown_by_node[node_index] else {
                continue;
            };
            if let FeedGraphNodeKind::Chamber {
                throat_area_m2,
                c_star_m_s,
                ..
            } = node.kind
            {
                let throat_coeff = throat_area_m2 / c_star_m_s;
                residuals[unknown_index] -= node_pressures_pa[node_index] * throat_coeff;
                jacobian[unknown_index * unknown_count + unknown_index] -= throat_coeff;
            }
        }

        let max_residual = max_abs(&residuals);
        if !max_residual.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "steady feed graph residual became non-finite",
            });
        }
        Ok((residuals, jacobian, max_residual))
    }

    fn snapshot(
        &self,
        unknown_pressures_pa: &[f64],
        max_residual_kg_per_s: f64,
    ) -> Result<SteadyFeedGraphSnapshot, FeedSystemError> {
        let node_pressures_pa = self.node_pressures(unknown_pressures_pa);
        let mut valve_branch_mass_flow_kg_per_s =
            Vec::with_capacity(self.config.valve_branches.len());
        for branch in &self.config.valve_branches {
            let flow = signed_valve_flow(
                *branch,
                node_pressures_pa[branch.from] - node_pressures_pa[branch.to],
            );
            if !flow.is_finite() {
                return Err(FeedSystemError::NonFinite {
                    reason: "steady feed graph snapshot contains non-finite branch flow",
                });
            }
            valve_branch_mass_flow_kg_per_s.push(flow);
        }
        Ok(SteadyFeedGraphSnapshot {
            node_pressures_pa,
            valve_branch_mass_flow_kg_per_s,
            max_residual_kg_per_s,
            validation: ValidationStatus::ValidatedToy,
        })
    }
}

/// Solved steady feed-network state.
#[derive(Clone, Debug, PartialEq)]
pub struct SteadyFeedGraphSnapshot {
    /// Node pressures in declaration order.
    pub node_pressures_pa: Vec<f64>,
    /// Signed branch mass flows in declaration order. Positive is `from -> to`.
    pub valve_branch_mass_flow_kg_per_s: Vec<f64>,
    /// Maximum absolute unknown-node mass residual in kg/s.
    pub max_residual_kg_per_s: f64,
    /// Validation posture of the graph model.
    pub validation: ValidationStatus,
}

fn signed_valve_flow(branch: ValveBranch, pressure_delta_pa: f64) -> f64 {
    if branch.open_fraction == 0.0 || pressure_delta_pa == 0.0 {
        return 0.0;
    }
    let magnitude = branch.coefficient() * pressure_delta_pa.abs().sqrt();
    if pressure_delta_pa.is_sign_positive() {
        magnitude
    } else {
        -magnitude
    }
}

fn valve_flow_derivative(branch: ValveBranch, pressure_delta_pa: f64) -> f64 {
    if branch.open_fraction == 0.0 {
        return 0.0;
    }
    0.5 * branch.coefficient()
        / pressure_delta_pa
            .abs()
            .max(DERIVATIVE_DELTA_P_FLOOR_PA)
            .sqrt()
}

fn accumulate_branch(
    residuals: &mut [f64],
    jacobian: &mut [f64],
    unknown_by_node: &[Option<usize>],
    branch: ValveBranch,
    flow_kg_per_s: f64,
    derivative: f64,
    unknown_count: usize,
) {
    if let Some(from_unknown) = unknown_by_node[branch.from] {
        residuals[from_unknown] -= flow_kg_per_s;
        add_jacobian(
            jacobian,
            unknown_count,
            from_unknown,
            branch.from,
            -derivative,
            unknown_by_node,
        );
        add_jacobian(
            jacobian,
            unknown_count,
            from_unknown,
            branch.to,
            derivative,
            unknown_by_node,
        );
    }
    if let Some(to_unknown) = unknown_by_node[branch.to] {
        residuals[to_unknown] += flow_kg_per_s;
        add_jacobian(
            jacobian,
            unknown_count,
            to_unknown,
            branch.from,
            derivative,
            unknown_by_node,
        );
        add_jacobian(
            jacobian,
            unknown_count,
            to_unknown,
            branch.to,
            -derivative,
            unknown_by_node,
        );
    }
}

fn add_jacobian(
    jacobian: &mut [f64],
    unknown_count: usize,
    row: usize,
    node_index: usize,
    value: f64,
    unknown_by_node: &[Option<usize>],
) {
    if let Some(column) = unknown_by_node[node_index] {
        jacobian[row * unknown_count + column] += value;
    }
}

fn solve_dense_linear(
    mut matrix: Vec<f64>,
    mut rhs: Vec<f64>,
) -> Result<Vec<f64>, FeedSystemError> {
    let n = rhs.len();
    for pivot in 0..n {
        let mut pivot_row = pivot;
        let mut pivot_abs = matrix[pivot * n + pivot].abs();
        for row in (pivot + 1)..n {
            let candidate_abs = matrix[row * n + pivot].abs();
            if candidate_abs > pivot_abs {
                pivot_row = row;
                pivot_abs = candidate_abs;
            }
        }
        if !pivot_abs.is_finite() || pivot_abs <= f64::EPSILON {
            return Err(FeedSystemError::NonConverged {
                reason: "steady feed graph Newton matrix is singular",
            });
        }
        if pivot_row != pivot {
            for column in 0..n {
                matrix.swap(pivot * n + column, pivot_row * n + column);
            }
            rhs.swap(pivot, pivot_row);
        }
        for row in (pivot + 1)..n {
            let factor = matrix[row * n + pivot] / matrix[pivot * n + pivot];
            matrix[row * n + pivot] = 0.0;
            for column in (pivot + 1)..n {
                matrix[row * n + column] -= factor * matrix[pivot * n + column];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let mut sum = rhs[row];
        for column in (row + 1)..n {
            sum -= matrix[row * n + column] * x[column];
        }
        let diagonal = matrix[row * n + row];
        if !diagonal.is_finite() || diagonal.abs() <= f64::EPSILON {
            return Err(FeedSystemError::NonConverged {
                reason: "steady feed graph Newton matrix is singular",
            });
        }
        x[row] = sum / diagonal;
        if !x[row].is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "steady feed graph Newton update is non-finite",
            });
        }
    }
    Ok(x)
}

fn damped_update(values: &mut [f64], delta: &[f64]) -> Result<(), FeedSystemError> {
    let mut scale = 1.0;
    for _ in 0..MIN_DAMPING_STEPS {
        let candidate_valid = values.iter().zip(delta).all(|(value, step)| {
            (*value + scale * *step).is_finite() && *value + scale * *step >= 0.0
        });
        if candidate_valid {
            for (value, step) in values.iter_mut().zip(delta) {
                *value += scale * *step;
            }
            return Ok(());
        }
        scale *= 0.5;
    }
    Err(FeedSystemError::NonConverged {
        reason: "steady feed graph Newton damping could not keep pressures non-negative",
    })
}

fn max_abs(values: &[f64]) -> f64 {
    values.iter().fold(0.0, |acc, value| acc.max(value.abs()))
}

fn require_positive_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    if !value.is_finite() {
        return Err(FeedSystemError::NonFinite { reason });
    }
    if value <= 0.0 {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

fn require_nonnegative_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    if !value.is_finite() {
        return Err(FeedSystemError::NonFinite { reason });
    }
    if value < 0.0 {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use approx::assert_abs_diff_eq;
    use toml::value::Table;

    use crate::network::{
        FeedCommand, FeedNetwork, TankValveChamberConfig, TankValveChamberNetwork,
    };

    use super::*;

    fn valve(from: usize, to: usize, area_m2: f64) -> ValveBranch {
        ValveBranch {
            from,
            to,
            area_m2,
            discharge_coefficient: 0.72,
            density_kg_m3: 810.0,
            open_fraction: 1.0,
        }
    }

    fn chamber_node(initial_pressure_pa: f64) -> FeedGraphNode {
        FeedGraphNode {
            kind: FeedGraphNodeKind::Chamber {
                initial_pressure_pa,
                throat_area_m2: 1.2e-4,
                c_star_m_s: 1_600.0,
            },
        }
    }

    #[test]
    fn graph_matches_reduced_tank_valve_chamber_solution() {
        let reduced = TankValveChamberNetwork::new(TankValveChamberConfig {
            tank_pressure_pa: 4.0e6,
            propellant_density_kg_m3: 810.0,
            valve_area_m2: 8.0e-5,
            valve_discharge_coefficient: 0.72,
            throat_area_m2: 1.2e-4,
            c_star_m_s: 1_600.0,
        })
        .unwrap()
        .solve(FeedCommand::fully_open())
        .unwrap();

        let graph = SteadyFeedGraph::new(SteadyFeedGraphConfig::new(
            vec![
                FeedGraphNode {
                    kind: FeedGraphNodeKind::PressureBoundary { pressure_pa: 4.0e6 },
                },
                chamber_node(3.0e6),
            ],
            vec![valve(0, 1, 8.0e-5)],
        ))
        .unwrap();
        let snapshot = graph.solve().unwrap();

        assert_abs_diff_eq!(
            snapshot.node_pressures_pa[1],
            reduced.chamber_pressure_pa,
            epsilon = 1.0e-3
        );
        assert_abs_diff_eq!(
            snapshot.valve_branch_mass_flow_kg_per_s[0],
            reduced.mass_flow_kg_per_s,
            epsilon = DEFAULT_RESIDUAL_TOLERANCE_KG_PER_S
        );
        assert!(snapshot.max_residual_kg_per_s <= DEFAULT_RESIDUAL_TOLERANCE_KG_PER_S);
    }

    #[test]
    fn graph_solves_two_valve_junction_chamber_ladder() {
        let graph = SteadyFeedGraph::new(SteadyFeedGraphConfig::new(
            vec![
                FeedGraphNode {
                    kind: FeedGraphNodeKind::PressureBoundary { pressure_pa: 4.0e6 },
                },
                FeedGraphNode {
                    kind: FeedGraphNodeKind::Junction {
                        initial_pressure_pa: 3.5e6,
                    },
                },
                chamber_node(3.0e6),
            ],
            vec![valve(0, 1, 1.2e-4), valve(1, 2, 1.0e-4)],
        ))
        .unwrap();

        let snapshot = graph.solve().unwrap();

        assert!(snapshot.node_pressures_pa[0] > snapshot.node_pressures_pa[1]);
        assert!(snapshot.node_pressures_pa[1] > snapshot.node_pressures_pa[2]);
        assert!(snapshot.node_pressures_pa[2] > 0.0);
        assert_abs_diff_eq!(
            snapshot.valve_branch_mass_flow_kg_per_s[0],
            snapshot.valve_branch_mass_flow_kg_per_s[1],
            epsilon = DEFAULT_RESIDUAL_TOLERANCE_KG_PER_S
        );
        let throat_flow = snapshot.node_pressures_pa[2] * 1.2e-4 / 1_600.0;
        assert_abs_diff_eq!(
            snapshot.valve_branch_mass_flow_kg_per_s[1],
            throat_flow,
            epsilon = DEFAULT_RESIDUAL_TOLERANCE_KG_PER_S
        );
        assert!(snapshot.max_residual_kg_per_s <= DEFAULT_RESIDUAL_TOLERANCE_KG_PER_S);
    }

    #[test]
    fn graph_rejects_invalid_branch_endpoint() {
        let config = SteadyFeedGraphConfig::new(
            vec![
                FeedGraphNode {
                    kind: FeedGraphNodeKind::PressureBoundary { pressure_pa: 4.0e6 },
                },
                chamber_node(3.0e6),
            ],
            vec![valve(0, 2, 8.0e-5)],
        );

        assert!(matches!(
            SteadyFeedGraph::new(config),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn graph_matches_provenance_tolerance_table() {
        let data: toml::Value = toml::from_str(include_str!(
            "../../../data/feed_system/generic-steady-feed-network-v1.toml"
        ))
        .unwrap();
        assert_eq!(
            table("openbmp", &data)["feed_network_steady"]
                .as_integer()
                .unwrap(),
            1
        );
        let graph_config = table("graph_config", &data);
        let graph = SteadyFeedGraph::new(SteadyFeedGraphConfig::new(
            vec![
                FeedGraphNode {
                    kind: FeedGraphNodeKind::PressureBoundary {
                        pressure_pa: float(graph_config, "boundary_pressure_pa"),
                    },
                },
                FeedGraphNode {
                    kind: FeedGraphNodeKind::Junction {
                        initial_pressure_pa: float(graph_config, "junction_initial_pressure_pa"),
                    },
                },
                FeedGraphNode {
                    kind: FeedGraphNodeKind::Chamber {
                        initial_pressure_pa: float(graph_config, "chamber_initial_pressure_pa"),
                        throat_area_m2: float(graph_config, "throat_area_m2"),
                        c_star_m_s: float(graph_config, "c_star_m_s"),
                    },
                },
            ],
            vec![
                ValveBranch {
                    from: 0,
                    to: 1,
                    area_m2: float(graph_config, "upstream_valve_area_m2"),
                    discharge_coefficient: float(graph_config, "valve_discharge_coefficient"),
                    density_kg_m3: float(graph_config, "density_kg_m3"),
                    open_fraction: float(graph_config, "open_fraction"),
                },
                ValveBranch {
                    from: 1,
                    to: 2,
                    area_m2: float(graph_config, "downstream_valve_area_m2"),
                    discharge_coefficient: float(graph_config, "valve_discharge_coefficient"),
                    density_kg_m3: float(graph_config, "density_kg_m3"),
                    open_fraction: float(graph_config, "open_fraction"),
                },
            ],
        ))
        .unwrap();
        let tolerances = table("tolerances", &data);

        for case in data
            .get("graph_case")
            .and_then(toml::Value::as_array)
            .expect("graph_case array")
        {
            let case = case.as_table().unwrap();
            let snapshot = graph.solve().unwrap();
            let expected_node_pressures = float_array(case, "expected_node_pressures_pa");
            let expected_branch_flows =
                float_array(case, "expected_valve_branch_mass_flow_kg_per_s");
            let name = string(case, "name");

            assert_eq!(
                snapshot.node_pressures_pa.len(),
                expected_node_pressures.len()
            );
            for (actual, expected) in snapshot
                .node_pressures_pa
                .iter()
                .zip(expected_node_pressures.iter())
            {
                assert_abs_diff_eq!(
                    actual,
                    expected,
                    epsilon = float(tolerances, "absolute_pressure_pa")
                );
            }
            assert_eq!(
                snapshot.valve_branch_mass_flow_kg_per_s.len(),
                expected_branch_flows.len()
            );
            for (actual, expected) in snapshot
                .valve_branch_mass_flow_kg_per_s
                .iter()
                .zip(expected_branch_flows.iter())
            {
                assert_abs_diff_eq!(
                    actual,
                    expected,
                    epsilon = float(tolerances, "absolute_mass_flow_kg_per_s")
                );
            }
            let chamber_flow = snapshot.node_pressures_pa[2]
                * float(graph_config, "throat_area_m2")
                / float(graph_config, "c_star_m_s");
            assert_abs_diff_eq!(
                chamber_flow,
                float(case, "expected_chamber_throat_mass_flow_kg_per_s"),
                epsilon = float(tolerances, "absolute_mass_flow_kg_per_s")
            );
            assert_abs_diff_eq!(
                snapshot.max_residual_kg_per_s,
                float(case, "expected_max_residual_kg_per_s"),
                epsilon = float(tolerances, "absolute_residual_kg_per_s")
            );
            assert_eq!(
                string(case, "expected_validation"),
                "validated-toy",
                "case {name}"
            );
            assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
        }
    }

    fn table<'a>(key: &str, value: &'a toml::Value) -> &'a Table {
        value
            .get(key)
            .and_then(toml::Value::as_table)
            .unwrap_or_else(|| panic!("missing table {key}"))
    }

    fn float(table: &Table, key: &str) -> f64 {
        table
            .get(key)
            .and_then(toml::Value::as_float)
            .unwrap_or_else(|| panic!("missing float {key}"))
    }

    fn float_array(table: &Table, key: &str) -> Vec<f64> {
        table
            .get(key)
            .and_then(toml::Value::as_array)
            .unwrap_or_else(|| panic!("missing array {key}"))
            .iter()
            .map(|value| {
                value
                    .as_float()
                    .unwrap_or_else(|| panic!("array {key} contains a non-float"))
            })
            .collect()
    }

    fn string<'a>(table: &'a Table, key: &str) -> &'a str {
        table
            .get(key)
            .and_then(toml::Value::as_str)
            .unwrap_or_else(|| panic!("missing string {key}"))
    }
}
