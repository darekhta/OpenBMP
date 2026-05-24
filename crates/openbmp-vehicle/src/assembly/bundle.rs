//! [`KernelModelBundle`] — flat lists the kernel consumes.
//!
//! The full assembly-to-kernel resolver returns bundles of this
//! shape once propulsion, effectors, tanks, and sensors are represented
//! in the assembly tree. Runners consume assembly dry mass
//! properties directly and keep force / moment plumbing on the existing
//! runner paths. Two parallel bundle forms cover the point-mass /
//! rigid-body kernel split:
//!
//! - [`KernelModelBundle<S>`] for kernels parameterised over a
//!   point-mass-shaped state (`MM: MassModel`).
//! - [`KernelModelBundleRigid`] for the rigid-body kernel
//!   (`MM: RigidMassModel`).
//!
//! The `sensors: Vec<Box<dyn SyntheticSensor>>` slot is left
//! empty here; the sensor resolver populates it.

use openbmp_models::{MassModel, RigidMassModel, SimState};
use openbmp_state::RigidBodyState;

use crate::error::VehicleError;
use crate::vehicle::{KernelVehicle, NamedForceModel, NamedMomentModel};

/// Flat lists the point-mass kernel consumes.
///
/// `force_models` and `moment_models` carry scenario-declared order
/// — that is the determinism contract every existing kernel test
/// relies on. The bundle does not validate names; that happens
/// downstream in [`KernelVehicle::new`].
pub struct KernelModelBundle<S: SimState> {
    /// Force models in scenario-declared order.
    pub force_models: Vec<NamedForceModel<S>>,
    /// Moment models in scenario-declared order.
    pub moment_models: Vec<NamedMomentModel<S>>,
    /// Mass model.
    pub mass_model: Box<dyn MassModel>,
}

impl<S: SimState> KernelModelBundle<S> {
    /// Construct an empty bundle.
    #[must_use]
    pub fn empty(mass_model: Box<dyn MassModel>) -> Self {
        Self {
            force_models: Vec::new(),
            moment_models: Vec::new(),
            mass_model,
        }
    }

    /// Number of force models in the bundle.
    #[must_use]
    pub fn force_model_count(&self) -> usize {
        self.force_models.len()
    }

    /// Build a [`KernelVehicle`] from the bundle by consuming the
    /// force / moment-model lists. Returns the constructed vehicle
    /// and the mass model passed in (the runner uses the mass model
    /// separately for the kernel's `MM` slot).
    ///
    /// The caller must supply the mass model rather than reading
    /// `self.mass_model` because [`MassModel`] is not [`Clone`]; the
    /// resolver hands out fresh mass-model instances for each
    /// `KernelVehicle` (kernel + breakdown).
    ///
    /// # Errors
    ///
    /// Propagates [`VehicleError`] when the force / moment-model
    /// names fail [`KernelVehicle::new`] validation.
    pub fn into_basic_vehicle(
        self,
        vehicle_owned_mass_model: Box<dyn MassModel>,
    ) -> Result<KernelVehicle<S>, VehicleError> {
        KernelVehicle::new(
            self.force_models,
            self.moment_models,
            vehicle_owned_mass_model,
        )
    }
}

impl<S: SimState> std::fmt::Debug for KernelModelBundle<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // `Box<dyn MassModel>` is not Debug; skip via finish_non_exhaustive
        // to satisfy `clippy::missing_fields_in_debug`.
        f.debug_struct("KernelModelBundle")
            .field("force_model_count", &self.force_models.len())
            .field("moment_model_count", &self.moment_models.len())
            .finish_non_exhaustive()
    }
}

/// Flat lists the rigid-body kernel consumes. Mirrors
/// [`KernelModelBundle`] with [`RigidMassModel`] in the mass-model
/// slot.
pub struct KernelModelBundleRigid {
    /// Force models in scenario-declared order.
    pub force_models: Vec<NamedForceModel<RigidBodyState>>,
    /// Moment models in scenario-declared order.
    pub moment_models: Vec<NamedMomentModel<RigidBodyState>>,
    /// Rigid-body mass model.
    pub rigid_mass_model: Box<dyn RigidMassModel>,
}

impl KernelModelBundleRigid {
    /// Construct an empty bundle around a mass model.
    #[must_use]
    pub fn empty(rigid_mass_model: Box<dyn RigidMassModel>) -> Self {
        Self {
            force_models: Vec::new(),
            moment_models: Vec::new(),
            rigid_mass_model,
        }
    }

    /// Number of force models in the bundle.
    #[must_use]
    pub fn force_model_count(&self) -> usize {
        self.force_models.len()
    }
}

impl std::fmt::Debug for KernelModelBundleRigid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KernelModelBundleRigid")
            .field("force_model_count", &self.force_models.len())
            .field("moment_model_count", &self.moment_models.len())
            .finish_non_exhaustive()
    }
}
