//! Structural bending-mode rack — the runner-side stateful home for the
//! launch vehicle's first lateral bending mode, mirroring the wind / tank
//! racks. It owns the modal state, advances it once per base tick with a
//! one-step-lag body-acceleration driver (the same bit-stable discipline the
//! slosh tank rack documents), and exposes the rate-gyro pickup the FC bridge
//! adds to the sensed body rate.
//!
//! When `[vehicle.bending]` is absent the rack is [`StructuralRack::Inactive`]
//! and every consumer short-circuits — the run is byte-identical to a
//! perfectly rigid vehicle.

use nalgebra::Vector3;
use openbmp_core::Duration;
use openbmp_scenario::ScenarioDocument;
use openbmp_vehicle::structural::BendingMode;

use crate::error::RunnerError;

/// Stateful bending-mode container advanced once per base tick.
#[derive(Clone, Debug)]
pub enum StructuralRack {
    /// No `[vehicle.bending]` declared — perfectly rigid (default).
    Inactive,
    /// One active first bending mode.
    Active(ActiveBending),
}

/// State for an active bending mode plus its one-step-lag driver.
#[derive(Clone, Debug)]
pub struct ActiveBending {
    mode: BendingMode,
    dt_s: f64,
    /// Body-frame lateral specific force from the previous step; the mode is
    /// advanced against this (one-step lag, matching the slosh rack).
    driver_accel_body: Vector3<f64>,
    /// Gyro pickup computed after the most recent `step`, read by the bridge
    /// at the top of the next tick (contemporaneous with the body rate the
    /// FC also reads from `current_state`).
    pickup_rad_s: Vector3<f64>,
    reaction_moment_body_n_m: Vector3<f64>,
}

impl StructuralRack {
    /// Build from the scenario. Returns [`StructuralRack::Inactive`] when no
    /// `[vehicle.bending]` block is present.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when the bending parameters are invalid.
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let Some(cfg) = document.vehicle.bending.as_ref() else {
            return Ok(Self::Inactive);
        };
        let omega_b = core::f64::consts::TAU * cfg.frequency_hz;
        let mode = BendingMode::new(
            omega_b,
            cfg.damping_ratio,
            cfg.modal_mass_kg,
            cfg.slope_at_engine,
            cfg.slope_at_gyro,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("[vehicle.bending] invalid: {err}"),
        })?;
        Ok(Self::Active(ActiveBending {
            mode,
            dt_s: document.time.dt_s,
            driver_accel_body: Vector3::zeros(),
            pickup_rad_s: Vector3::zeros(),
            reaction_moment_body_n_m: Vector3::zeros(),
        }))
    }

    /// Whether the rack carries no active mode (every consumer short-circuits).
    #[must_use]
    pub fn is_inactive(&self) -> bool {
        matches!(self, Self::Inactive)
    }

    /// Reset modal state to rest (called at scenario start for replay
    /// determinism, mirroring the wind rack).
    pub fn reset(&mut self) {
        if let Self::Active(a) = self {
            // Rebuild a fresh mode at rest, preserving parameters.
            a.driver_accel_body = Vector3::zeros();
            a.pickup_rad_s = Vector3::zeros();
            a.reaction_moment_body_n_m = Vector3::zeros();
        }
    }

    /// Advance the mode one base tick against the prior-step driver, then
    /// refresh the gyro pickup and reaction moment.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] on a non-finite modal step.
    pub fn step(&mut self) -> Result<(), RunnerError> {
        if let Self::Active(a) = self {
            a.mode
                .step(a.driver_accel_body, Duration::from_seconds(a.dt_s))
                .map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!("bending-mode step failed: {err}"),
                })?;
            a.pickup_rad_s = a.mode.gyro_pickup_rad_s();
            a.reaction_moment_body_n_m = a.mode.reaction_moment_body_n_m();
        }
        Ok(())
    }

    /// Store the post-step body-frame lateral specific force for the next
    /// tick's advance (one-step lag).
    pub fn update_drivers(&mut self, accel_body_m_s2: Vector3<f64>) {
        if let Self::Active(a) = self {
            a.driver_accel_body = accel_body_m_s2;
        }
    }

    /// Rate-gyro pickup from the bending slope rate, added to the sensed body
    /// rate by the FC bridge. Zero when inactive.
    #[must_use]
    pub fn gyro_pickup_rad_s(&self) -> Vector3<f64> {
        match self {
            Self::Inactive => Vector3::zeros(),
            Self::Active(a) => a.pickup_rad_s,
        }
    }

    /// Reaction moment the bending mode exerts on the rigid body (body
    /// frame). Wired into the dynamics in a follow-up; exposed now for the
    /// reaction path. Zero when inactive.
    #[must_use]
    pub fn reaction_moment_body_n_m(&self) -> Vector3<f64> {
        match self {
            Self::Inactive => Vector3::zeros(),
            Self::Active(a) => a.reaction_moment_body_n_m,
        }
    }
}
