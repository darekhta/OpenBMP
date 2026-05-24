//! Two-stage drogue + main recovery.
//!
//! [`DrogueMainRecovery`] is an academic two-stage descent model.
//! The first deploy fires the drogue chute; the second deploy
//! transitions to the (larger) main canopy. State machine:
//! `Stowed → Drogue → Main`. No re-entry: once `Main`, subsequent
//! deploys are rejected.
//!
//! Drag area swap is instantaneous on the firing event; canopy-
//! inflation transients are not modelled. Both stages publish
//! their own `(C_D, A)` pair through [`RecoveryModel::current_c_d`] /
//! [`RecoveryModel::current_drag_area_m2`].

use openbmp_core::RecoveryId;

use super::{
    RecoveryCommand, RecoveryError, RecoveryModel, RecoveryPhase, require_positive_area,
    require_positive_c_d,
};

/// Two-stage drogue + main parachute recovery.
///
/// Scenario-declared `(drogue_c_d, drogue_area_m2)` and
/// `(main_c_d, main_area_m2)` describe the two canopy stages. The
/// state machine is one-way: `Stowed → Drogue → Main`.
#[derive(Clone, Debug)]
pub struct DrogueMainRecovery {
    id: RecoveryId,
    drogue_c_d: f64,
    drogue_area_m2: f64,
    main_c_d: f64,
    main_area_m2: f64,
    phase: RecoveryPhase,
}

impl DrogueMainRecovery {
    /// Construct a stowed two-stage recovery device.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] when any drag
    /// coefficient or area is non-finite or non-positive.
    pub fn new(
        id: RecoveryId,
        drogue_c_d: f64,
        drogue_area_m2: f64,
        main_c_d: f64,
        main_area_m2: f64,
    ) -> Result<Self, RecoveryError> {
        require_positive_c_d(drogue_c_d)?;
        require_positive_area(drogue_area_m2)?;
        require_positive_c_d(main_c_d)?;
        require_positive_area(main_area_m2)?;
        Ok(Self {
            id,
            drogue_c_d,
            drogue_area_m2,
            main_c_d,
            main_area_m2,
            phase: RecoveryPhase::Stowed,
        })
    }

    /// Drogue-stage drag coefficient.
    #[must_use]
    pub fn drogue_c_d(&self) -> f64 {
        self.drogue_c_d
    }

    /// Drogue-stage drag area.
    #[must_use]
    pub fn drogue_area_m2(&self) -> f64 {
        self.drogue_area_m2
    }

    /// Main-stage drag coefficient.
    #[must_use]
    pub fn main_c_d(&self) -> f64 {
        self.main_c_d
    }

    /// Main-stage drag area.
    #[must_use]
    pub fn main_area_m2(&self) -> f64 {
        self.main_area_m2
    }
}

impl RecoveryModel for DrogueMainRecovery {
    fn id(&self) -> RecoveryId {
        self.id
    }

    fn phase(&self) -> RecoveryPhase {
        self.phase
    }

    fn current_c_d(&self) -> f64 {
        match self.phase {
            RecoveryPhase::Stowed => 0.0,
            RecoveryPhase::Drogue => self.drogue_c_d,
            RecoveryPhase::Main => self.main_c_d,
        }
    }

    fn current_drag_area_m2(&self) -> f64 {
        match self.phase {
            RecoveryPhase::Stowed => 0.0,
            RecoveryPhase::Drogue => self.drogue_area_m2,
            RecoveryPhase::Main => self.main_area_m2,
        }
    }

    fn apply_command(&mut self, command: RecoveryCommand) -> Result<(), RecoveryError> {
        match (self.phase, command) {
            (RecoveryPhase::Stowed, RecoveryCommand::DeployDrogue) => {
                self.phase = RecoveryPhase::Drogue;
                Ok(())
            }
            (RecoveryPhase::Drogue, RecoveryCommand::DeployMain) => {
                self.phase = RecoveryPhase::Main;
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

    fn dm() -> DrogueMainRecovery {
        DrogueMainRecovery::new(RecoveryId::new(7), 1.2, 0.5, 1.5, 4.0).unwrap()
    }

    #[test]
    fn rejects_non_positive_drogue_c_d() {
        assert!(DrogueMainRecovery::new(RecoveryId::new(1), 0.0, 0.5, 1.5, 4.0).is_err());
    }

    #[test]
    fn rejects_non_positive_drogue_area() {
        assert!(DrogueMainRecovery::new(RecoveryId::new(1), 1.2, 0.0, 1.5, 4.0).is_err());
    }

    #[test]
    fn rejects_non_positive_main_c_d() {
        assert!(DrogueMainRecovery::new(RecoveryId::new(1), 1.2, 0.5, 0.0, 4.0).is_err());
    }

    #[test]
    fn rejects_non_positive_main_area() {
        assert!(DrogueMainRecovery::new(RecoveryId::new(1), 1.2, 0.5, 1.5, 0.0).is_err());
    }

    #[test]
    fn rejects_non_finite_inputs() {
        assert!(DrogueMainRecovery::new(RecoveryId::new(1), f64::NAN, 0.5, 1.5, 4.0).is_err());
        assert!(DrogueMainRecovery::new(RecoveryId::new(1), 1.2, f64::INFINITY, 1.5, 4.0).is_err());
    }

    #[test]
    fn starts_stowed_with_zero_drag_outputs() {
        let d = dm();
        assert_eq!(d.phase(), RecoveryPhase::Stowed);
        assert_eq!(d.current_c_d().to_bits(), 0.0_f64.to_bits());
        assert_eq!(d.current_drag_area_m2().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn drogue_then_main_walks_state_machine() {
        let mut d = dm();
        d.apply_command(RecoveryCommand::DeployDrogue).unwrap();
        assert_eq!(d.phase(), RecoveryPhase::Drogue);
        assert_eq!(d.current_c_d().to_bits(), 1.2_f64.to_bits());
        assert_eq!(d.current_drag_area_m2().to_bits(), 0.5_f64.to_bits());

        d.apply_command(RecoveryCommand::DeployMain).unwrap();
        assert_eq!(d.phase(), RecoveryPhase::Main);
        assert_eq!(d.current_c_d().to_bits(), 1.5_f64.to_bits());
        assert_eq!(d.current_drag_area_m2().to_bits(), 4.0_f64.to_bits());
    }

    #[test]
    fn deploy_main_before_drogue_is_rejected() {
        let mut d = dm();
        let err = d.apply_command(RecoveryCommand::DeployMain).unwrap_err();
        assert!(matches!(
            err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::DeployMain,
                from_phase: RecoveryPhase::Stowed,
                ..
            }
        ));
    }

    #[test]
    fn deploy_drogue_again_after_drogue_is_rejected() {
        let mut d = dm();
        d.apply_command(RecoveryCommand::DeployDrogue).unwrap();
        let err = d.apply_command(RecoveryCommand::DeployDrogue).unwrap_err();
        assert!(matches!(
            err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::DeployDrogue,
                from_phase: RecoveryPhase::Drogue,
                ..
            }
        ));
    }

    #[test]
    fn deploy_main_again_after_main_is_rejected() {
        let mut d = dm();
        d.apply_command(RecoveryCommand::DeployDrogue).unwrap();
        d.apply_command(RecoveryCommand::DeployMain).unwrap();
        let err = d.apply_command(RecoveryCommand::DeployMain).unwrap_err();
        assert!(matches!(
            err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::DeployMain,
                from_phase: RecoveryPhase::Main,
                ..
            }
        ));
    }

    #[test]
    fn single_stage_deploy_is_rejected_on_two_stage_device() {
        let mut d = dm();
        let err = d.apply_command(RecoveryCommand::Deploy).unwrap_err();
        assert!(matches!(
            err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::Deploy,
                ..
            }
        ));
    }

    #[test]
    fn stow_command_is_rejected() {
        let mut d = dm();
        d.apply_command(RecoveryCommand::DeployDrogue).unwrap();
        let err = d.apply_command(RecoveryCommand::Stow).unwrap_err();
        assert!(matches!(
            err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::Stow,
                ..
            }
        ));
    }

    #[test]
    fn step_is_noop_across_all_phases() {
        let mut d = dm();
        for _ in 0..100 {
            d.step(0.01).unwrap();
        }
        assert_eq!(d.phase(), RecoveryPhase::Stowed);

        d.apply_command(RecoveryCommand::DeployDrogue).unwrap();
        for _ in 0..100 {
            d.step(0.01).unwrap();
        }
        assert_eq!(d.phase(), RecoveryPhase::Drogue);

        d.apply_command(RecoveryCommand::DeployMain).unwrap();
        for _ in 0..100 {
            d.step(0.01).unwrap();
        }
        assert_eq!(d.phase(), RecoveryPhase::Main);
    }
}
