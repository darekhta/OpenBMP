//! Phase-3.9 generic drag device (airbrake).
//!
//! [`DragDevice`] is a cycle-able airbrake or speed-brake. Unlike
//! [`crate::recovery::ParachuteDrag`] (one-shot deploy) or
//! [`crate::recovery::DrogueMainRecovery`] (one-way two-stage), the
//! drag device accepts repeated [`RecoveryCommand::Deploy`] /
//! [`RecoveryCommand::Stow`] cycles. State machine:
//! `Stowed ↔ Main`.
//!
//! Drag area swap is instantaneous on the firing event — Phase-3.9
//! does not model deployment transients (rate-limited brake travel,
//! hinge dynamics, etc.). The brake is "on" or "off"; no intermediate
//! position.

use openbmp_core::RecoveryId;

use super::{
    RecoveryCommand, RecoveryError, RecoveryModel, RecoveryPhase, require_positive_area,
    require_positive_c_d,
};

/// Generic airbrake / drag device. Cycles [`RecoveryPhase::Stowed`]
/// ↔ [`RecoveryPhase::Main`] under `Deploy` / `Stow` commands.
#[derive(Clone, Debug)]
pub struct DragDevice {
    id: RecoveryId,
    c_d: f64,
    area_deployed_m2: f64,
    phase: RecoveryPhase,
}

impl DragDevice {
    /// Construct a stowed drag device.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] when `c_d` or
    /// `area_deployed_m2` is non-finite or non-positive.
    pub fn new(id: RecoveryId, c_d: f64, area_deployed_m2: f64) -> Result<Self, RecoveryError> {
        require_positive_c_d(c_d)?;
        require_positive_area(area_deployed_m2)?;
        Ok(Self {
            id,
            c_d,
            area_deployed_m2,
            phase: RecoveryPhase::Stowed,
        })
    }

    /// Drag coefficient declared at construction.
    #[must_use]
    pub fn c_d(&self) -> f64 {
        self.c_d
    }

    /// Deployed-state drag area declared at construction.
    #[must_use]
    pub fn area_deployed_m2(&self) -> f64 {
        self.area_deployed_m2
    }
}

impl RecoveryModel for DragDevice {
    fn id(&self) -> RecoveryId {
        self.id
    }

    fn phase(&self) -> RecoveryPhase {
        self.phase
    }

    fn current_c_d(&self) -> f64 {
        match self.phase {
            RecoveryPhase::Stowed => 0.0,
            RecoveryPhase::Drogue | RecoveryPhase::Main => self.c_d,
        }
    }

    fn current_drag_area_m2(&self) -> f64 {
        match self.phase {
            RecoveryPhase::Stowed => 0.0,
            RecoveryPhase::Drogue | RecoveryPhase::Main => self.area_deployed_m2,
        }
    }

    fn apply_command(&mut self, command: RecoveryCommand) -> Result<(), RecoveryError> {
        match (self.phase, command) {
            (RecoveryPhase::Stowed, RecoveryCommand::Deploy) => {
                self.phase = RecoveryPhase::Main;
                Ok(())
            }
            (RecoveryPhase::Main, RecoveryCommand::Stow) => {
                self.phase = RecoveryPhase::Stowed;
                Ok(())
            }
            (from_phase, command) => Err(RecoveryError::UnsupportedCommand {
                device: self.id,
                command,
                from_phase,
            }),
        }
    }

    fn step(&mut self, _dt_s: f64) -> Result<(), RecoveryError> {
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    fn brake() -> DragDevice {
        DragDevice::new(RecoveryId::new(3), 0.8, 0.05).unwrap()
    }

    #[test]
    fn rejects_non_positive_c_d() {
        assert!(DragDevice::new(RecoveryId::new(1), 0.0, 0.05).is_err());
    }

    #[test]
    fn rejects_non_positive_area() {
        assert!(DragDevice::new(RecoveryId::new(1), 0.8, 0.0).is_err());
    }

    #[test]
    fn starts_stowed_with_zero_drag_outputs() {
        let b = brake();
        assert_eq!(b.phase(), RecoveryPhase::Stowed);
        assert_eq!(b.current_c_d().to_bits(), 0.0_f64.to_bits());
        assert_eq!(b.current_drag_area_m2().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn deploy_then_stow_cycles_back_to_stowed() {
        let mut b = brake();
        b.apply_command(RecoveryCommand::Deploy).unwrap();
        assert_eq!(b.phase(), RecoveryPhase::Main);
        assert_eq!(b.current_c_d().to_bits(), 0.8_f64.to_bits());

        b.apply_command(RecoveryCommand::Stow).unwrap();
        assert_eq!(b.phase(), RecoveryPhase::Stowed);
        assert_eq!(b.current_c_d().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn many_cycles_remain_consistent() {
        let mut b = brake();
        for _ in 0..16 {
            b.apply_command(RecoveryCommand::Deploy).unwrap();
            assert_eq!(b.phase(), RecoveryPhase::Main);
            b.apply_command(RecoveryCommand::Stow).unwrap();
            assert_eq!(b.phase(), RecoveryPhase::Stowed);
        }
    }

    #[test]
    fn deploy_when_already_deployed_is_rejected() {
        let mut b = brake();
        b.apply_command(RecoveryCommand::Deploy).unwrap();
        let err = b.apply_command(RecoveryCommand::Deploy).unwrap_err();
        assert!(matches!(
            err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::Deploy,
                from_phase: RecoveryPhase::Main,
                ..
            }
        ));
    }

    #[test]
    fn stow_when_already_stowed_is_rejected() {
        let mut b = brake();
        let err = b.apply_command(RecoveryCommand::Stow).unwrap_err();
        assert!(matches!(
            err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::Stow,
                from_phase: RecoveryPhase::Stowed,
                ..
            }
        ));
    }

    #[test]
    fn drogue_and_main_commands_are_rejected() {
        let mut b = brake();
        let drogue_err = b.apply_command(RecoveryCommand::DeployDrogue).unwrap_err();
        assert!(matches!(
            drogue_err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::DeployDrogue,
                ..
            }
        ));
        let main_err = b.apply_command(RecoveryCommand::DeployMain).unwrap_err();
        assert!(matches!(
            main_err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::DeployMain,
                ..
            }
        ));
    }

    #[test]
    fn step_is_noop() {
        let mut b = brake();
        for _ in 0..100 {
            b.step(0.001).unwrap();
        }
        assert_eq!(b.phase(), RecoveryPhase::Stowed);
    }
}
