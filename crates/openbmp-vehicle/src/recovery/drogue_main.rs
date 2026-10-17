//! Two-stage drogue + main recovery.
//!
//! [`DrogueMainRecovery`] is an academic two-stage descent model.
//! The first deploy fires the drogue chute; the second deploy
//! transitions to the (larger) main canopy. State machine:
//! `Stowed → Drogue → Main`. No re-entry: once `Main`, subsequent
//! deploys are rejected.
//!
//! By default the drag-area swap is instantaneous on the firing
//! event. Opt-in stage inflation profiles ramp the effective area for
//! the drogue and/or main stage independently. Both stages publish
//! their own `(C_D, A)` pair through [`RecoveryModel::current_c_d`] /
//! [`RecoveryModel::current_drag_area_m2`].

use openbmp_core::RecoveryId;

use super::{
    CanopyInflationSample, CanopyInflationState, InflationProfile, RecoveryCommand, RecoveryError,
    RecoveryModel, RecoveryPhase, require_positive_area, require_positive_c_d,
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
    drogue_inflation: Option<CanopyInflationState>,
    main_inflation: Option<CanopyInflationState>,
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
            drogue_inflation: None,
            main_inflation: None,
        })
    }

    /// Construct a stowed two-stage recovery device with optional
    /// independent drogue/main fill-time profiles.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] when any drag
    /// coefficient or area is non-finite or non-positive.
    pub fn new_with_inflation(
        id: RecoveryId,
        drogue_c_d: f64,
        drogue_area_m2: f64,
        main_c_d: f64,
        main_area_m2: f64,
        drogue_inflation: Option<InflationProfile>,
        main_inflation: Option<InflationProfile>,
    ) -> Result<Self, RecoveryError> {
        let mut recovery = Self::new(id, drogue_c_d, drogue_area_m2, main_c_d, main_area_m2)?;
        recovery.drogue_inflation = drogue_inflation.map(CanopyInflationState::new);
        recovery.main_inflation = main_inflation.map(CanopyInflationState::new);
        Ok(recovery)
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

    /// Drogue-stage inflation state, if configured.
    #[must_use]
    pub const fn drogue_inflation(&self) -> Option<CanopyInflationState> {
        self.drogue_inflation
    }

    /// Main-stage inflation state, if configured.
    #[must_use]
    pub const fn main_inflation(&self) -> Option<CanopyInflationState> {
        self.main_inflation
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
            RecoveryPhase::Drogue => {
                self.drogue_inflation
                    .map_or(self.drogue_area_m2, |inflation| {
                        inflation
                            .effective_drag_area_m2(self.drogue_area_m2)
                            .unwrap_or(f64::NAN)
                    })
            }
            RecoveryPhase::Main => self.main_inflation.map_or(self.main_area_m2, |inflation| {
                inflation
                    .effective_drag_area_m2(self.main_area_m2)
                    .unwrap_or(f64::NAN)
            }),
        }
    }

    fn current_inflation_sample(
        &self,
        dynamic_pressure_pa: f64,
    ) -> Result<Option<CanopyInflationSample>, RecoveryError> {
        let stage = match self.phase {
            RecoveryPhase::Stowed => None,
            RecoveryPhase::Drogue => self
                .drogue_inflation
                .map(|inflation| (inflation, self.drogue_c_d, self.drogue_area_m2)),
            RecoveryPhase::Main => self
                .main_inflation
                .map(|inflation| (inflation, self.main_c_d, self.main_area_m2)),
        };
        stage
            .map(|(inflation, c_d, area_m2)| {
                if !inflation.started() {
                    return Ok(None);
                }
                inflation
                    .profile()
                    .sample(inflation.elapsed_s(), c_d, area_m2, dynamic_pressure_pa)
                    .map(Some)
            })
            .transpose()
            .map(Option::flatten)
    }

    fn apply_command(&mut self, command: RecoveryCommand) -> Result<(), RecoveryError> {
        match (self.phase, command) {
            (RecoveryPhase::Stowed, RecoveryCommand::DeployDrogue) => {
                self.phase = RecoveryPhase::Drogue;
                if let Some(inflation) = &mut self.drogue_inflation {
                    inflation.start();
                }
                Ok(())
            }
            (RecoveryPhase::Drogue, RecoveryCommand::DeployMain) => {
                self.phase = RecoveryPhase::Main;
                if let Some(inflation) = &mut self.main_inflation {
                    inflation.start();
                }
                Ok(())
            }
            (from_phase, command) => Err(RecoveryError::UnsupportedCommand {
                device: self.id,
                command,
                from_phase,
            }),
        }
    }

    fn step(&mut self, dt_s: f64) -> Result<(), RecoveryError> {
        match self.phase {
            RecoveryPhase::Stowed => {}
            RecoveryPhase::Drogue => {
                if let Some(inflation) = &mut self.drogue_inflation {
                    inflation.step(dt_s)?;
                }
            }
            RecoveryPhase::Main => {
                if let Some(inflation) = &mut self.main_inflation {
                    inflation.step(dt_s)?;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::recovery::CanopyGrowthShape;

    fn dm() -> DrogueMainRecovery {
        DrogueMainRecovery::new(RecoveryId::new(7), 1.2, 0.5, 1.5, 4.0).unwrap()
    }

    fn linear_profile(fill_time_s: f64, opening_force_coeff_c_x: f64) -> InflationProfile {
        InflationProfile::new(
            fill_time_s,
            0.0,
            CanopyGrowthShape::polynomial(1.0).unwrap(),
            opening_force_coeff_c_x,
        )
        .unwrap()
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

    #[test]
    fn staged_inflation_ramps_drogue_then_main_independently() {
        let mut d = DrogueMainRecovery::new_with_inflation(
            RecoveryId::new(7),
            1.2,
            0.5,
            1.5,
            4.0,
            Some(linear_profile(1.0, 1.1)),
            Some(linear_profile(2.0, 1.2)),
        )
        .unwrap();

        d.apply_command(RecoveryCommand::DeployDrogue).unwrap();
        assert_eq!(d.phase(), RecoveryPhase::Drogue);
        assert_eq!(d.current_drag_area_m2().to_bits(), 0.0_f64.to_bits());
        let sample = d.current_inflation_sample(100.0).unwrap().unwrap();
        assert_eq!(sample.progress.to_bits(), 0.0_f64.to_bits());
        assert!((sample.infinite_mass_peak_load_n - 66.0).abs() < 1e-12);

        d.step(0.25).unwrap();
        assert_eq!(d.current_drag_area_m2().to_bits(), 0.125_f64.to_bits());
        let sample = d.current_inflation_sample(100.0).unwrap().unwrap();
        assert_eq!(sample.progress.to_bits(), 0.25_f64.to_bits());
        assert_eq!(sample.effective_drag_area_m2.to_bits(), 0.125_f64.to_bits());
        assert!((sample.instantaneous_infinite_mass_load_n - 16.5).abs() < 1e-12);

        d.apply_command(RecoveryCommand::DeployMain).unwrap();
        assert_eq!(d.phase(), RecoveryPhase::Main);
        assert_eq!(d.current_drag_area_m2().to_bits(), 0.0_f64.to_bits());
        let sample = d.current_inflation_sample(100.0).unwrap().unwrap();
        assert_eq!(sample.progress.to_bits(), 0.0_f64.to_bits());
        assert!((sample.infinite_mass_peak_load_n - 720.0).abs() < 1e-12);

        d.step(0.5).unwrap();
        assert_eq!(d.current_drag_area_m2().to_bits(), 1.0_f64.to_bits());
        let sample = d.current_inflation_sample(100.0).unwrap().unwrap();
        assert_eq!(sample.progress.to_bits(), 0.25_f64.to_bits());
        assert_eq!(sample.effective_drag_area_m2.to_bits(), 1.0_f64.to_bits());
        assert!((sample.instantaneous_infinite_mass_load_n - 180.0).abs() < 1e-12);
    }
}
