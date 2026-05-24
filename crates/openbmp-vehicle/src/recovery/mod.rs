//! Recovery and drag-device models.
//!
//! Recovery devices are scenario-deployable drag elements: parachutes,
//! drogue / main two-stage chutes, and generic airbrakes. They produce
//! an aerodynamic drag force opposing the body's ECI velocity once
//! deployed; before deployment the force is zero. They are driven by
//! mission-phase graph events through deploy-recovery actions.
//!
//! Three implementations are provided:
//!
//! - [`parachute_drag::ParachuteDrag`] — single-stage parachute. One
//!   deploy event toggles `Stowed → Main`.
//! - [`drogue_main::DrogueMainRecovery`] — two-stage academic
//!   descent. First deploy → `Drogue`, second deploy → `Main`.
//! - [`drag_device::DragDevice`] — generic airbrake. Cycles
//!   `Stowed ↔ Main` via deploy / stow commands.
//!
//! All three publish a [`openbmp_models::RecoverySnapshot`] each kernel base tick
//! through the runner-side [`crate::recovery`]-rack adapter. Drag is
//! evaluated by the [`crate::adapters::RecoveryRackForceAdapter`]
//! using the kernel's atmosphere sample (density) and the body's ECI
//! velocity.
//!
//! # Force model
//!
//! `F_drag = -½ ρ |v|² C_D · A · v̂` per Knacke 1992 *Parachute
//! Recovery Systems Design Manual* Chapter 5. Drag opposes the body's
//! ECI velocity; the academic formulation ignores wind-relative
//! velocity (matches the [`crate::adapters::DeckDragForceAdapter`]
//! convention). Drag is applied at the body CG; recovery devices
//! contribute zero moment (long risers are assumed to
//! decouple body rotation from drag direction).
//!
//! # Determinism
//!
//! Pure `f64` arithmetic; locked operand order on the drag formula;
//! no FMA, no wall-clock, no system RNG. Recovery state machines
//! transition only on event triggers; per-step evaluation is a pure
//! observer.
//!
//! # Crate layering
//!
//! Recovery models live in `openbmp-vehicle` (L1) alongside tanks /
//! engines / effectors. The runner-side rack is part of the
//! runner integration; the kernel-side adapter ships in
//! [`crate::adapters`] and consumes the kernel's
//! [`openbmp_models::RecoverySnapshotView`] view.
//!
//! See `docs/scenario-format.md § Recovery and descent`
//! and `docs/software-architecture.md § Recovery and Descent Models`
//! for the contract.

pub mod drag_device;
pub mod drogue_main;
pub mod parachute_drag;

pub use drag_device::DragDevice;
pub use drogue_main::DrogueMainRecovery;
pub use parachute_drag::ParachuteDrag;

use openbmp_core::RecoveryId;
use thiserror::Error;

// ---------------------------------------------------------------------
// Phase + command enums
// ---------------------------------------------------------------------

/// Recovery-device deployment phase.
///
/// `Stowed` is the pre-deploy state shared by every recovery model.
/// `Drogue` is reached only by [`DrogueMainRecovery`] after its first
/// deploy. `Main` is the fully deployed state for every model
/// (or the post-drogue state for [`DrogueMainRecovery`]).
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
#[repr(u8)]
pub enum RecoveryPhase {
    /// Not deployed. Drag area is zero.
    #[default]
    Stowed = 0,
    /// Drogue stage open ([`DrogueMainRecovery`] only).
    Drogue = 1,
    /// Main stage open (or single-stage device fully deployed).
    Main = 2,
}

impl RecoveryPhase {
    /// Stable index used in telemetry channels.
    #[must_use]
    pub const fn index(self) -> u8 {
        self as u8
    }

    /// `true` for any non-`Stowed` phase.
    #[must_use]
    pub const fn is_deployed(self) -> bool {
        !matches!(self, Self::Stowed)
    }
}

/// Command targeting one recovery device, dispatched by a deploy-recovery
/// event firing.
///
/// Each command's effect depends on the receiving model's kind:
///
/// | Command | [`ParachuteDrag`] | [`DrogueMainRecovery`] | [`DragDevice`] |
/// |---|---|---|---|
/// | `Deploy` | `Stowed → Main` | rejected | `Stowed → Main` |
/// | `DeployDrogue` | rejected | `Stowed → Drogue` | rejected |
/// | `DeployMain` | rejected | `Drogue → Main` | rejected |
/// | `Stow` | rejected | rejected | `Main → Stowed` |
///
/// Mismatched commands surface as
/// [`RecoveryError::UnsupportedCommand`].
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum RecoveryCommand {
    /// Open the device (single-stage).
    Deploy,
    /// Open the drogue stage (two-stage only).
    DeployDrogue,
    /// Open the main stage (two-stage only).
    DeployMain,
    /// Close the device (cycles allowed; airbrake only).
    Stow,
}

impl RecoveryCommand {
    /// Stable canonical-name string used in scenario TOML and
    /// telemetry tags.
    #[must_use]
    pub const fn canonical_name(self) -> &'static str {
        match self {
            Self::Deploy => "deploy",
            Self::DeployDrogue => "deploy_drogue",
            Self::DeployMain => "deploy_main",
            Self::Stow => "stow",
        }
    }
}

// ---------------------------------------------------------------------
// RecoveryModel trait
// ---------------------------------------------------------------------

/// Recovery-device trait.
///
/// Implementors hold their own internal state ([`RecoveryPhase`])
/// and expose the four primitive operations the runner-side rack
/// needs: a stable id, the current phase, the current drag area and
/// drag coefficient at that phase, and a typed command-application
/// path that mutates the state machine.
///
/// Implementors must be deterministic: pure `f64` arithmetic, no FMA,
/// no system RNG, no I/O. The implementations have no
/// internal numerical integration — phase transitions are
/// instantaneous on the firing event — so [`RecoveryModel::step`] is
/// a no-op for the shipped models. The trait reserves the method for
/// future phases that may add inflation transients or canopy-area
/// blends.
pub trait RecoveryModel: Send + std::fmt::Debug {
    /// Stable identifier (FNV-1a-64 hash of the canonical scenario
    /// path).
    fn id(&self) -> RecoveryId;

    /// Current deployment phase.
    fn phase(&self) -> RecoveryPhase;

    /// Drag coefficient `C_D` at the current phase. Zero in
    /// [`RecoveryPhase::Stowed`]; otherwise the phase-specific value.
    fn current_c_d(&self) -> f64;

    /// Drag area `A` (m²) at the current phase. Zero in
    /// [`RecoveryPhase::Stowed`]; otherwise the phase-specific value.
    fn current_drag_area_m2(&self) -> f64;

    /// Apply a deploy / stow command. Implementations validate the
    /// command against their state machine and reject mismatches with
    /// [`RecoveryError::UnsupportedCommand`].
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError`] when the command is not legal for
    /// the current state machine or kind.
    fn apply_command(&mut self, command: RecoveryCommand) -> Result<(), RecoveryError>;

    /// Advance per-step internal state.
    ///
    /// The three instantaneous-deploy models make this a
    /// no-op. Reserved for extensions that may add canopy
    /// inflation transients.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError`] for non-finite inputs (the shipped
    /// implementations never raise).
    fn step(&mut self, dt_s: f64) -> Result<(), RecoveryError>;
}

// ---------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------

/// Errors raised by recovery-device construction or command handling.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum RecoveryError {
    /// `c_d` or a drag area is non-finite or non-positive.
    #[error("recovery parameter invalid: {reason}")]
    InvalidParameter {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// Step `dt_s` is non-finite or non-positive.
    #[error("recovery step dt_s must be finite and strictly positive, got {dt_s}")]
    InvalidStep {
        /// Offending value.
        dt_s: f64,
    },
    /// The command does not apply to this device kind / current phase.
    #[error("recovery {device:?} does not support command {command:?} from phase {from_phase:?}")]
    UnsupportedCommand {
        /// Device id.
        device: RecoveryId,
        /// Offending command.
        command: RecoveryCommand,
        /// State the device was in when the command fired.
        from_phase: RecoveryPhase,
    },
}

// ---------------------------------------------------------------------
// Helpers shared by impls
// ---------------------------------------------------------------------

/// Validate that `c_d` is finite and strictly positive.
///
/// # Errors
///
/// Returns [`RecoveryError::InvalidParameter`] otherwise.
pub(crate) fn require_positive_c_d(c_d: f64) -> Result<(), RecoveryError> {
    if !c_d.is_finite() || c_d <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "c_d must be finite and strictly positive",
        });
    }
    Ok(())
}

/// Validate that an area is finite and strictly positive.
///
/// # Errors
///
/// Returns [`RecoveryError::InvalidParameter`] otherwise.
pub(crate) fn require_positive_area(area_m2: f64) -> Result<(), RecoveryError> {
    if !area_m2.is_finite() || area_m2 <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "drag area must be finite and strictly positive",
        });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn phase_index_matches_repr() {
        assert_eq!(RecoveryPhase::Stowed.index(), 0);
        assert_eq!(RecoveryPhase::Drogue.index(), 1);
        assert_eq!(RecoveryPhase::Main.index(), 2);
    }

    #[test]
    fn stowed_is_not_deployed() {
        assert!(!RecoveryPhase::Stowed.is_deployed());
        assert!(RecoveryPhase::Drogue.is_deployed());
        assert!(RecoveryPhase::Main.is_deployed());
    }

    #[test]
    fn command_canonical_names_are_stable() {
        assert_eq!(RecoveryCommand::Deploy.canonical_name(), "deploy");
        assert_eq!(
            RecoveryCommand::DeployDrogue.canonical_name(),
            "deploy_drogue"
        );
        assert_eq!(RecoveryCommand::DeployMain.canonical_name(), "deploy_main");
        assert_eq!(RecoveryCommand::Stow.canonical_name(), "stow");
    }

    #[test]
    fn require_positive_c_d_rejects_zero_and_negative_and_non_finite() {
        assert!(require_positive_c_d(1.0).is_ok());
        assert!(require_positive_c_d(0.0).is_err());
        assert!(require_positive_c_d(-1.0).is_err());
        assert!(require_positive_c_d(f64::NAN).is_err());
        assert!(require_positive_c_d(f64::INFINITY).is_err());
    }

    #[test]
    fn require_positive_area_rejects_zero_and_negative_and_non_finite() {
        assert!(require_positive_area(0.5).is_ok());
        assert!(require_positive_area(0.0).is_err());
        assert!(require_positive_area(-0.1).is_err());
        assert!(require_positive_area(f64::NAN).is_err());
    }
}
