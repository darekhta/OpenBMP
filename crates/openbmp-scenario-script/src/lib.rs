//! `openbmp-scenario-script` — simulator-only scenario-script actions.
//!
//! Phase 5.X.A landed the action-taxonomy split: actions that drive
//! the in-house *physics-only* scenario script — engine throttle /
//! gimbal commands, effector overrides, scripted separation, recovery
//! deploy — live in this crate, separate from
//! [`openbmp_mission::MissionAction`] (which is HAL-portable).
//!
//! This crate is **simulator-only**: it depends on
//! `openbmp-mission` and `openbmp-core` for trigger / id types but
//! its [`ScenarioScriptAction`] enum is consumed only by the
//! simulator kernel. A real-hardware HAL adopter does not link this
//! crate.
//!
//! See [`docs/mission-graph-architecture.md`](../../docs/mission-graph-architecture.md)
//! for the architectural rationale (the load-bearing HAL portability
//! rule for `openbmp-mission`) and
//! [`docs/phase-5x-plan.md`](../../docs/phase-5x-plan.md#5xa--action-taxonomy-split)
//! for the migration sequence.

use openbmp_core::{EffectorId, EngineId, RecoveryId};

/// Action taken when a scenario-script-side event fires.
///
/// Phase 5.X.A: split out of the legacy `openbmp_mission::EventAction`
/// enum. Every variant here is a *simulator-only physics override* —
/// the simulator kernel records the firing each tick and the runner-side
/// rack (`EngineRack` / `EffectorRack` / `RecoveryRack`) drains it on
/// the next rack tick. These actions cannot fire in a HAL deployment
/// because the script-action binding list is not compiled into the
/// HAL crate. See
/// [`docs/mission-graph-architecture.md § Action taxonomy`](../../docs/mission-graph-architecture.md).
#[derive(Clone, Debug, PartialEq)]
pub enum ScenarioScriptAction {
    /// Phase-3.6: per-engine command targeting a declared engine by
    /// [`EngineId`]. The event consumer records the firing; the
    /// runner-side `EngineRack::apply_commands` drains it and applies
    /// the command to the engine on the next rack tick.
    ///
    /// Phase-3.15.C / Phase-5.X.A: the field shape is engine-domain-shaped
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
    /// Phase-3.4: scenario-driven effector command override. Targets
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
    /// Phase-3.6 / 3.7 deferred: stage-separation event.
    Separation,
    /// Phase-3.9: deploy / stow a recovery device. Targets a declared
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
