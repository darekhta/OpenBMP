//! `openbmp-scenario-script` — simulator-only scenario-script actions.
//!
//! The action taxonomy is split: actions that drive
//! the in-house *physics-only* scenario script — engine throttle /
//! gimbal commands, effector overrides, scripted separation, recovery
//! deploy — live in this crate, separate from
//! `openbmp_mission::MissionAction` (which is HAL-portable).
//!
//! This crate is **simulator-only**: it depends on
//! `openbmp-core` for id types while staying separate from
//! `openbmp-mission`. Its [`ScenarioScriptAction`] enum is consumed only by the
//! simulator kernel. A real-hardware HAL adopter does not link this
//! crate.
//!
//! See [`docs/mission-graph-architecture.md`](../../docs/mission-graph-architecture.md)
//! for the architectural rationale: the load-bearing HAL-portability
//! rule for `openbmp-mission` and the action-taxonomy split that keeps
//! these simulator-only physics overrides out of the HAL-portable
//! mission crate.

use openbmp_core::{BodyId, EffectorId, EngineId, RecoveryId};

/// Action taken when a scenario-script-side event fires.
///
/// Split out of the unified mission-event action
/// enum. Every variant here is a *simulator-only physics override* —
/// the simulator kernel records the firing each tick and the runner-side
/// rack (`EngineRack` / `EffectorRack` / `RecoveryRack`) drains it on
/// the next rack tick. These actions cannot fire in a HAL deployment
/// because the script-action binding list is not compiled into the
/// HAL crate. See
/// [`docs/mission-graph-architecture.md § Action taxonomy`](../../docs/mission-graph-architecture.md).
#[derive(Clone, Debug, PartialEq)]
pub enum ScenarioScriptAction {
    /// Per-engine command targeting a declared engine by
    /// [`EngineId`]. The event consumer records the firing; the
    /// runner-side `EngineRack::apply_commands` drains it and applies
    /// the command to the engine on the next rack tick.
    ///
    /// The field shape is engine-domain-shaped
    /// but this crate does *not* depend on `openbmp-propulsion`. The
    /// runner translates these scalar fields into a typed
    /// `openbmp_propulsion::EngineCommand` at apply time — same pattern
    /// as [`Self::DeployRecovery`].
    EngineCommand {
        /// Target engine id.
        id: EngineId,
        /// Throttle setting in `[0, 1]`. Clamped at apply time.
        throttle_unit: f64,
        /// Gimbal pitch angle in radians. Clamped at apply time.
        gimbal_pitch_rad: f64,
        /// Gimbal yaw angle in radians. Clamped at apply time.
        gimbal_yaw_rad: f64,
        /// Ignition request. Honoured only from `Idle`.
        ignite: bool,
        /// Shutdown request. Honoured only from `Igniting` or
        /// `Burning`. When both `ignite` and `shutdown` are `true`,
        /// shutdown wins.
        shutdown: bool,
    },
    /// Scenario-driven effector command override. Targets
    /// a declared effector by [`EffectorId`]; the runner-side
    /// `EffectorRack::apply_overrides` consumes the fired event and
    /// stores the override into the rack's per-effector override map
    /// for the next rack tick.
    EffectorOverride {
        /// Target effector id.
        id: EffectorId,
        /// Command value.
        command: f64,
    },
    /// Legacy stage-separation event without a body id. Kept for
    /// defensive compatibility; scenario validation rejects the bare
    /// action because executed separation needs an explicit body.
    Separation,
    /// Commanded stage-separation event for a declared assembly body.
    ///
    /// This is simulator-only: the kernel / runner use it to split a
    /// rigid-body scenario into independent propagated bodies. It is
    /// not part of the HAL-portable mission-action vocabulary.
    JettisonStage {
        /// Body to detach from the continuing stack.
        body: BodyId,
    },
    /// Commanded batch separation event for multiple declared
    /// assembly bodies.
    ///
    /// Every listed body is partitioned from the same pre-separation
    /// rigid-body state. This is the coordinated deployment path for
    /// one bus releasing multiple independent bodies on the same
    /// event tick.
    JettisonBodies {
        /// Bodies to detach from the continuing stack.
        bodies: Vec<BodyId>,
    },
    /// Deploy / stow a recovery device. Targets a declared
    /// recovery device by [`RecoveryId`]; the runner-side
    /// `RecoveryRack::apply_deploys` consumes the fired event and
    /// applies the command to the device's state machine.
    DeployRecovery {
        /// Target recovery-device id.
        id: RecoveryId,
        /// Command name (one of `"deploy"`, `"deploy_drogue"`,
        /// `"deploy_main"`, `"stow"`). The string is the rack-side
        /// canonical name; the rack maps it to the typed
        /// `openbmp_vehicle::RecoveryCommand` enum at apply time.
        command: String,
    },
}
