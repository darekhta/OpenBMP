//! Single-stage parachute model.
//!
//! [`ParachuteDrag`] is the simplest recovery device: a stowed
//! parachute that opens once on a [`RecoveryCommand::Deploy`] event
//! and stays open for the rest of the scenario. Closed-form drag area
//! and drag coefficient are scenario-declared and constant after
//! deployment.
//!
//! Drag force is evaluated by the kernel-side adapter
//! [`crate::adapters::RecoveryRackForceAdapter`] using the
//! body's air-relative velocity in ECI components and the atmosphere
//! sample's density per Knacke 1992 Chapter 5:
//! `F = -½ ρ |v_air|² C_D A · v̂_air`.

use openbmp_core::RecoveryId;

use super::{
    CanopyInflationSample, CanopyInflationState, InflationProfile, RecoveryCommand, RecoveryError,
    RecoveryModel, RecoveryPhase, require_positive_area, require_positive_c_d,
};

/// Single-stage parachute. One [`RecoveryCommand::Deploy`] firing
/// transitions `Stowed → Main`; subsequent deploys are rejected
/// (one-shot semantics).
#[derive(Clone, Debug)]
pub struct ParachuteDrag {
    id: RecoveryId,
    c_d: f64,
    area_inflated_m2: f64,
    phase: RecoveryPhase,
    inflation: Option<CanopyInflationState>,
}

impl ParachuteDrag {
    /// Construct a stowed parachute.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] when `c_d` or
    /// `area_inflated_m2` is non-finite or non-positive.
    pub fn new(id: RecoveryId, c_d: f64, area_inflated_m2: f64) -> Result<Self, RecoveryError> {
        require_positive_c_d(c_d)?;
        require_positive_area(area_inflated_m2)?;
        Ok(Self {
            id,
            c_d,
            area_inflated_m2,
            phase: RecoveryPhase::Stowed,
            inflation: None,
        })
    }

    /// Construct a stowed parachute with opt-in fill-time inflation.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] when `c_d` or
    /// `area_inflated_m2` is non-finite or non-positive.
    pub fn new_with_inflation(
        id: RecoveryId,
        c_d: f64,
        area_inflated_m2: f64,
        inflation: InflationProfile,
    ) -> Result<Self, RecoveryError> {
        let mut chute = Self::new(id, c_d, area_inflated_m2)?;
        chute.inflation = Some(CanopyInflationState::new(inflation));
        Ok(chute)
    }

    /// Drag coefficient declared at construction.
    #[must_use]
    pub fn c_d(&self) -> f64 {
        self.c_d
    }

    /// Inflated drag area declared at construction.
    #[must_use]
    pub fn area_inflated_m2(&self) -> f64 {
        self.area_inflated_m2
    }

    /// Inflation state, if this parachute was configured with one.
    #[must_use]
    pub const fn inflation(&self) -> Option<CanopyInflationState> {
        self.inflation
    }
}

impl RecoveryModel for ParachuteDrag {
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
            RecoveryPhase::Drogue | RecoveryPhase::Main => {
                self.inflation.map_or(self.area_inflated_m2, |inflation| {
                    inflation
                        .effective_drag_area_m2(self.area_inflated_m2)
                        .unwrap_or(f64::NAN)
                })
            }
        }
    }

    fn current_inflation_sample(
        &self,
        dynamic_pressure_pa: f64,
    ) -> Result<Option<CanopyInflationSample>, RecoveryError> {
        if !self.phase.is_deployed() {
            return Ok(None);
        }
        self.inflation
            .map(|inflation| {
                inflation.profile().sample(
                    inflation.elapsed_s(),
                    self.c_d,
                    self.area_inflated_m2,
                    dynamic_pressure_pa,
                )
            })
            .transpose()
    }

    fn apply_command(&mut self, command: RecoveryCommand) -> Result<(), RecoveryError> {
        match (self.phase, command) {
            (RecoveryPhase::Stowed, RecoveryCommand::Deploy) => {
                self.phase = RecoveryPhase::Main;
                if let Some(inflation) = &mut self.inflation {
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
        if self.phase.is_deployed()
            && let Some(inflation) = &mut self.inflation
        {
            inflation.step(dt_s)?;
        }
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::recovery::CanopyGrowthShape;

    fn pchute() -> ParachuteDrag {
        ParachuteDrag::new(RecoveryId::new(1), 1.5, 2.0).unwrap()
    }

    #[test]
    fn rejects_non_positive_c_d() {
        assert!(ParachuteDrag::new(RecoveryId::new(1), 0.0, 1.0).is_err());
        assert!(ParachuteDrag::new(RecoveryId::new(1), -1.0, 1.0).is_err());
        assert!(ParachuteDrag::new(RecoveryId::new(1), f64::NAN, 1.0).is_err());
    }

    #[test]
    fn rejects_non_positive_area() {
        assert!(ParachuteDrag::new(RecoveryId::new(1), 1.5, 0.0).is_err());
        assert!(ParachuteDrag::new(RecoveryId::new(1), 1.5, -2.0).is_err());
        assert!(ParachuteDrag::new(RecoveryId::new(1), 1.5, f64::INFINITY).is_err());
    }

    #[test]
    fn starts_stowed_with_zero_drag_outputs() {
        let p = pchute();
        assert_eq!(p.phase(), RecoveryPhase::Stowed);
        assert_eq!(p.current_c_d().to_bits(), 0.0_f64.to_bits());
        assert_eq!(p.current_drag_area_m2().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn deploy_transitions_to_main_and_publishes_drag() {
        let mut p = pchute();
        p.apply_command(RecoveryCommand::Deploy).unwrap();
        assert_eq!(p.phase(), RecoveryPhase::Main);
        assert_eq!(p.current_c_d().to_bits(), 1.5_f64.to_bits());
        assert_eq!(p.current_drag_area_m2().to_bits(), 2.0_f64.to_bits());
    }

    #[test]
    fn second_deploy_is_rejected() {
        let mut p = pchute();
        p.apply_command(RecoveryCommand::Deploy).unwrap();
        let err = p.apply_command(RecoveryCommand::Deploy).unwrap_err();
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
    fn drogue_command_is_rejected() {
        let mut p = pchute();
        let err = p.apply_command(RecoveryCommand::DeployDrogue).unwrap_err();
        assert!(matches!(
            err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::DeployDrogue,
                ..
            }
        ));
    }

    #[test]
    fn stow_command_is_rejected_even_when_deployed() {
        let mut p = pchute();
        p.apply_command(RecoveryCommand::Deploy).unwrap();
        let err = p.apply_command(RecoveryCommand::Stow).unwrap_err();
        assert!(matches!(
            err,
            RecoveryError::UnsupportedCommand {
                command: RecoveryCommand::Stow,
                from_phase: RecoveryPhase::Main,
                ..
            }
        ));
    }

    #[test]
    fn step_is_noop_and_does_not_change_phase() {
        let mut p = pchute();
        for _ in 0..1000 {
            p.step(0.001).unwrap();
        }
        assert_eq!(p.phase(), RecoveryPhase::Stowed);
        p.apply_command(RecoveryCommand::Deploy).unwrap();
        for _ in 0..1000 {
            p.step(0.001).unwrap();
        }
        assert_eq!(p.phase(), RecoveryPhase::Main);
    }

    #[test]
    fn inflation_ramps_single_stage_drag_area_after_deploy() {
        let profile =
            InflationProfile::new(1.0, 0.0, CanopyGrowthShape::polynomial(1.0).unwrap(), 1.2)
                .unwrap();
        let mut p =
            ParachuteDrag::new_with_inflation(RecoveryId::new(1), 1.5, 8.0, profile).unwrap();
        assert_eq!(p.current_drag_area_m2().to_bits(), 0.0_f64.to_bits());
        p.apply_command(RecoveryCommand::Deploy).unwrap();
        assert_eq!(p.current_drag_area_m2().to_bits(), 0.0_f64.to_bits());
        p.step(0.25).unwrap();
        assert_eq!(p.current_drag_area_m2().to_bits(), 2.0_f64.to_bits());
        p.step(1.0).unwrap();
        assert_eq!(p.current_drag_area_m2().to_bits(), 8.0_f64.to_bits());
        assert!(p.inflation().unwrap().complete());
    }

    #[test]
    fn id_round_trips() {
        let p = ParachuteDrag::new(RecoveryId::from_path("recovery.pchute"), 1.5, 2.0).unwrap();
        assert_eq!(p.id(), RecoveryId::from_path("recovery.pchute"));
    }
}
