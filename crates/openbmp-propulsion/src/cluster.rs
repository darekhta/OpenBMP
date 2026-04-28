//! Phase-3.6 [`EngineCluster`]: a propulsion-side container of
//! [`EngineModel`] instances plus their body-frame mount geometry.
//!
//! `EngineCluster` is **not** a kernel-side `ForceModel` /
//! `MomentModel` / `MassModel`; the kernel-side adapters live in
//! `openbmp-vehicle::adapters` and consume per-step
//! [`EngineSnapshot`]s via the runner-pushed kernel snapshot
//! (Phase-3.6.C). Splitting the architecture-spec'd
//! "cluster-as-force-model" into a propulsion-side container plus
//! vehicle-side adapter trio keeps `openbmp-propulsion` L2 and
//! avoids the `&mut self` problem that would arise if engines lived
//! inside the kernel's force-model chain.
//!
//! This struct exists so the runner's `EngineRack` (Phase-3.6.C)
//! has a typed home for the engines plus the parallel
//! `mount_points_body` / `engine_ids` arrays that the kernel-side
//! cluster adapter needs at construction time.
//!
//! # Determinism
//!
//! - Engines are stored in a `Vec` and iterated in scenario-declared
//!   order (the order the runner passed at construction time). No
//!   sorting, no parallelism.
//! - `apply_commands` and `step(dt)` walk the engines in declared
//!   order.
//! - `snapshot()` returns a `Vec<EngineSnapshot>` in the same order.

use openbmp_core::{Body, Duration, EngineId, Position3};

use crate::engine::{EngineCommand, EngineModel, EngineSnapshot};
use crate::error::EngineError;

/// Cluster-layout tag, carried for telemetry / docs. Phase 3.6 has
/// no behavioural use for this — it informs neither the cluster's
/// summed force / moment / mass-flow nor the adapter trio. Future
/// sub-phases may use the layout to drive symmetry-aware fault
/// scenarios or controller-side allocation tables.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClusterLayout {
    /// Single axial engine (1 mount along body `+z`).
    Axial,
    /// Ring of N engines around the body `+z` axis.
    Ring,
    /// Octaweb-style: 1 axial + N ring-mounted.
    Octaweb,
    /// Custom geometry; runner consumes `mount_points_body` directly.
    Custom,
}

/// Propulsion-side cluster container.
///
/// Holds the engines, their body-frame mount points, and a layout
/// tag. The runner's per-step loop walks `engines` in declared order
/// applying commands and stepping; the kernel-side adapter trio
/// consumes a snapshot map keyed by [`EngineId`].
pub struct EngineCluster {
    engines: Vec<Box<dyn EngineModel>>,
    mount_points_body: Vec<Position3<Body>>,
    engine_ids: Vec<EngineId>,
    layout: ClusterLayout,
}

impl std::fmt::Debug for EngineCluster {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EngineCluster")
            .field("engine_count", &self.engines.len())
            .field("layout", &self.layout)
            .field("engine_ids", &self.engine_ids)
            .finish_non_exhaustive()
    }
}

impl EngineCluster {
    /// Construct a cluster from parallel arrays. Validates that the
    /// arrays are the same length.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidParameter`] when `engines`,
    /// `mount_points_body`, and `engine_ids` are not all the same
    /// length, or when any engine's id does not match the
    /// corresponding `engine_ids` entry.
    pub fn new(
        engines: Vec<Box<dyn EngineModel>>,
        mount_points_body: Vec<Position3<Body>>,
        engine_ids: Vec<EngineId>,
        layout: ClusterLayout,
    ) -> Result<Self, EngineError> {
        if engines.len() != mount_points_body.len() || engines.len() != engine_ids.len() {
            return Err(EngineError::InvalidParameter {
                reason: "engines, mount_points_body, and engine_ids must have the same length",
            });
        }
        for (engine, declared_id) in engines.iter().zip(engine_ids.iter()) {
            if engine.id() != *declared_id {
                return Err(EngineError::InvalidParameter {
                    reason: "engine.id() does not match the declared engine_ids entry",
                });
            }
        }
        Ok(Self {
            engines,
            mount_points_body,
            engine_ids,
            layout,
        })
    }

    /// Number of engines in the cluster.
    #[must_use]
    pub fn len(&self) -> usize {
        self.engines.len()
    }

    /// `true` when the cluster carries no engines.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.engines.is_empty()
    }

    /// Stable engine ids in scenario-declared order.
    #[must_use]
    pub fn engine_ids(&self) -> &[EngineId] {
        &self.engine_ids
    }

    /// Body-frame mount points in scenario-declared order.
    #[must_use]
    pub fn mount_points_body(&self) -> &[Position3<Body>] {
        &self.mount_points_body
    }

    /// Cluster layout tag.
    #[must_use]
    pub const fn layout(&self) -> ClusterLayout {
        self.layout
    }

    /// Apply a command to a specific engine by id. The command is
    /// latched on the engine; the next `step(dt)` call advances the
    /// engine's state machine using the latched command.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidParameter`] when no engine in
    /// the cluster has the given id. Forwards
    /// [`EngineError::NonFiniteCommand`] from the engine.
    pub fn apply_command(&mut self, id: EngineId, cmd: EngineCommand) -> Result<(), EngineError> {
        let index =
            self.engine_ids
                .iter()
                .position(|x| *x == id)
                .ok_or(EngineError::InvalidParameter {
                    reason: "engine id not found in cluster",
                })?;
        self.engines[index].apply_command(cmd)
    }

    /// Step every engine in the cluster by one kernel base tick. The
    /// returned `Vec<EngineSnapshot>` is in scenario-declared order.
    ///
    /// # Errors
    ///
    /// Forwards [`EngineError::InvalidDt`] from the first engine that
    /// rejects `dt`.
    pub fn step(&mut self, dt: Duration) -> Result<Vec<EngineSnapshot>, EngineError> {
        let mut snapshots = Vec::with_capacity(self.engines.len());
        for engine in &mut self.engines {
            snapshots.push(engine.step(dt)?);
        }
        Ok(snapshots)
    }

    /// Snapshot every engine without stepping. Useful for telemetry
    /// at step 0 (before any rack tick).
    #[must_use]
    pub fn current_snapshot(&self) -> Vec<EngineSnapshot> {
        self.engines.iter().map(|e| e.current_snapshot()).collect()
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::engine::{EngineCommand, EngineLimits, EngineState, LiquidEngine};
    use openbmp_core::EngineId;

    fn make_limits() -> EngineLimits {
        EngineLimits {
            max_thrust_n: 1000.0,
            isp_s: 250.0,
            ignition_transient_s: 0.1,
            shutdown_transient_s: 0.1,
            max_gimbal_rad: 0.1,
        }
    }

    fn fresh_two_engine_cluster() -> EngineCluster {
        let id_a = EngineId::from_path("test.engine_a");
        let id_b = EngineId::from_path("test.engine_b");
        let engines: Vec<Box<dyn EngineModel>> = vec![
            Box::new(LiquidEngine::new(id_a, make_limits()).unwrap()),
            Box::new(LiquidEngine::new(id_b, make_limits()).unwrap()),
        ];
        let mount_points = vec![
            Position3::<Body>::new(0.0, 0.0, 0.0),
            Position3::<Body>::new(0.5, 0.0, 0.0),
        ];
        let engine_ids = vec![id_a, id_b];
        EngineCluster::new(engines, mount_points, engine_ids, ClusterLayout::Custom).unwrap()
    }

    #[test]
    fn cluster_rejects_mismatched_array_lengths() {
        let id_a = EngineId::from_path("test.engine_a");
        let engines: Vec<Box<dyn EngineModel>> =
            vec![Box::new(LiquidEngine::new(id_a, make_limits()).unwrap())];
        let mount_points = vec![
            Position3::<Body>::new(0.0, 0.0, 0.0),
            Position3::<Body>::new(0.5, 0.0, 0.0),
        ];
        let engine_ids = vec![id_a];
        let err = EngineCluster::new(engines, mount_points, engine_ids, ClusterLayout::Custom)
            .unwrap_err();
        assert!(matches!(err, EngineError::InvalidParameter { .. }));
    }

    #[test]
    fn cluster_rejects_id_mismatch_between_engine_and_engine_ids() {
        let id_a = EngineId::from_path("test.engine_a");
        let id_b = EngineId::from_path("test.engine_b");
        let engines: Vec<Box<dyn EngineModel>> =
            vec![Box::new(LiquidEngine::new(id_a, make_limits()).unwrap())];
        let mount_points = vec![Position3::<Body>::new(0.0, 0.0, 0.0)];
        let engine_ids = vec![id_b]; // mismatch
        let err = EngineCluster::new(engines, mount_points, engine_ids, ClusterLayout::Custom)
            .unwrap_err();
        assert!(matches!(err, EngineError::InvalidParameter { .. }));
    }

    #[test]
    fn cluster_apply_command_routes_to_correct_engine() {
        let mut c = fresh_two_engine_cluster();
        let id_a = c.engine_ids()[0];
        c.apply_command(
            id_a,
            EngineCommand {
                throttle_unit: 1.0,
                gimbal_pitch_rad: 0.0,
                gimbal_yaw_rad: 0.0,
                ignite: true,
                shutdown: false,
            },
        )
        .unwrap();
        let snaps = c.step(Duration::from_seconds(0.001)).unwrap();
        assert_eq!(snaps[0].state, EngineState::Igniting);
        assert_eq!(snaps[1].state, EngineState::Idle);
    }

    #[test]
    fn cluster_apply_command_unknown_id_fails_closed() {
        let mut c = fresh_two_engine_cluster();
        let unknown = EngineId::from_path("test.nonexistent");
        let err = c.apply_command(unknown, EngineCommand::idle()).unwrap_err();
        assert!(matches!(err, EngineError::InvalidParameter { .. }));
    }

    #[test]
    fn cluster_step_iterates_in_declared_order() {
        let mut c = fresh_two_engine_cluster();
        let snaps = c.step(Duration::from_seconds(0.001)).unwrap();
        assert_eq!(snaps.len(), 2);
        assert_eq!(snaps[0].state, EngineState::Idle);
        assert_eq!(snaps[1].state, EngineState::Idle);
    }

    #[test]
    fn cluster_current_snapshot_iterates_in_declared_order() {
        let c = fresh_two_engine_cluster();
        let snaps = c.current_snapshot();
        assert_eq!(snaps.len(), 2);
        for s in snaps {
            assert_eq!(s.state, EngineState::Idle);
        }
    }

    #[test]
    fn cluster_summed_thrust_equals_scalar_sum_of_per_engine_thrust() {
        let mut c = fresh_two_engine_cluster();
        let id_a = c.engine_ids()[0];
        let id_b = c.engine_ids()[1];
        let cmd = EngineCommand {
            throttle_unit: 1.0,
            gimbal_pitch_rad: 0.0,
            gimbal_yaw_rad: 0.0,
            ignite: true,
            shutdown: false,
        };
        c.apply_command(id_a, cmd).unwrap();
        c.apply_command(id_b, cmd).unwrap();
        // Step through ignition transient.
        for _ in 0..101 {
            c.step(Duration::from_seconds(0.001)).unwrap();
        }
        let snaps = c.current_snapshot();
        // At full throttle, no gimbal: each engine produces 1000 N
        // along body +z.
        let summed_z: f64 = snaps.iter().map(|s| s.thrust_body.z).sum();
        assert_eq!(summed_z.to_bits(), 2000.0_f64.to_bits());
    }

    #[test]
    fn cluster_length_and_emptiness() {
        let c = fresh_two_engine_cluster();
        assert_eq!(c.len(), 2);
        assert!(!c.is_empty());

        let empty =
            EngineCluster::new(Vec::new(), Vec::new(), Vec::new(), ClusterLayout::Axial).unwrap();
        assert_eq!(empty.len(), 0);
        assert!(empty.is_empty());
    }
}
