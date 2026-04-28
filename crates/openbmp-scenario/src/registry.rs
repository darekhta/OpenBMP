//! Compile-time model registry for scenario validation.
//!
//! The registry resolves a `(role, name)` pair to a [`ModelDescriptor`].
//! Phase 1 ships a small fixed set; the registry is the single seam by
//! which later phases can plug in additional models without changes to
//! the parser.

use std::collections::BTreeMap;
use std::fmt;

use crate::error::ScenarioError;

/// Compile-time model role used by the Phase-1 registry.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum ModelRole {
    /// Vehicle model.
    Vehicle,
    /// Gravity model.
    Gravity,
    /// Atmosphere model.
    Atmosphere,
    /// Wind model.
    Wind,
    /// Virtual controller model.
    Controller,
    /// Force-model entry in deterministic force ordering.
    Force,
    /// Synthetic sensor model.
    Sensor,
    /// Propulsion motor model.
    Motor,
}

impl ModelRole {
    /// Canonical short label.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Vehicle => "vehicle",
            Self::Gravity => "gravity",
            Self::Atmosphere => "atmosphere",
            Self::Wind => "wind",
            Self::Controller => "controller",
            Self::Force => "force",
            Self::Sensor => "sensor",
            Self::Motor => "motor",
        }
    }
}

impl fmt::Display for ModelRole {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Registered model descriptor.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ModelDescriptor {
    /// Stable model name as used in scenario TOML.
    pub name: String,
    /// Model role.
    pub role: ModelRole,
}

impl ModelDescriptor {
    /// Construct a model descriptor.
    #[must_use]
    pub fn new(name: impl Into<String>, role: ModelRole) -> Self {
        Self {
            name: name.into(),
            role,
        }
    }
}

/// Compile-time model registry for scenario validation.
///
/// Lookup is keyed by `(role, name)` so the same model name may exist
/// under different roles without collision.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ModelRegistry {
    by_role: BTreeMap<ModelRole, BTreeMap<String, ModelDescriptor>>,
}

impl ModelRegistry {
    /// Phase-1 registry.
    #[must_use]
    pub fn phase1() -> Self {
        Self::from_descriptors([
            ModelDescriptor::new("point_mass", ModelRole::Vehicle),
            ModelDescriptor::new("constant", ModelRole::Gravity),
            ModelDescriptor::new("none", ModelRole::Atmosphere),
            ModelDescriptor::new("none", ModelRole::Wind),
            ModelDescriptor::new("noop", ModelRole::Controller),
            ModelDescriptor::new("gravity", ModelRole::Force),
        ])
    }

    /// Phase-2 registry.
    ///
    /// Extends the Phase-1 registry with the L2 physics models shipped
    /// across Phase 2.2 through 2.7: `j2` and `point_mass` gravity,
    /// `us_standard_1976` and `isothermal` atmosphere, `constant` wind,
    /// `aero` and `thrust` force terms, the `rigid_body` vehicle kind,
    /// the three synthetic sensors (`ideal_state`, `imu`, `barometer`),
    /// and the `solid` motor variant.
    #[must_use]
    pub fn phase2() -> Self {
        Self::from_descriptors([
            // Phase 1 entries.
            ModelDescriptor::new("point_mass", ModelRole::Vehicle),
            ModelDescriptor::new("constant", ModelRole::Gravity),
            ModelDescriptor::new("none", ModelRole::Atmosphere),
            ModelDescriptor::new("none", ModelRole::Wind),
            ModelDescriptor::new("noop", ModelRole::Controller),
            ModelDescriptor::new("gravity", ModelRole::Force),
            // Phase 2 vehicle.
            ModelDescriptor::new("rigid_body", ModelRole::Vehicle),
            // Phase 2.2 gravity.
            ModelDescriptor::new("point_mass", ModelRole::Gravity),
            ModelDescriptor::new("j2", ModelRole::Gravity),
            // Phase 2.3 atmosphere.
            ModelDescriptor::new("isothermal", ModelRole::Atmosphere),
            ModelDescriptor::new("us_standard_1976", ModelRole::Atmosphere),
            // Phase 2.4 wind.
            ModelDescriptor::new("constant", ModelRole::Wind),
            // Phase 3.8 wind extensions.
            ModelDescriptor::new("layered", ModelRole::Wind),
            ModelDescriptor::new("gust", ModelRole::Wind),
            // Phase 2.5/2.6 force terms.
            ModelDescriptor::new("aero", ModelRole::Force),
            ModelDescriptor::new("thrust", ModelRole::Force),
            // Phase 2.7 sensors.
            ModelDescriptor::new("ideal_state", ModelRole::Sensor),
            ModelDescriptor::new("imu", ModelRole::Sensor),
            ModelDescriptor::new("barometer", ModelRole::Sensor),
            // Phase 2.6 motor variants.
            ModelDescriptor::new("solid", ModelRole::Motor),
        ])
    }

    fn from_descriptors<I>(descriptors: I) -> Self
    where
        I: IntoIterator<Item = ModelDescriptor>,
    {
        let mut by_role = BTreeMap::new();
        for descriptor in descriptors {
            by_role
                .entry(descriptor.role)
                .or_insert_with(BTreeMap::new)
                .insert(descriptor.name.clone(), descriptor);
        }
        Self { by_role }
    }

    /// Resolve a model by role and name.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError::UnknownModel`] when the `(role, name)`
    /// pair is not registered, or [`ScenarioError::WrongModelRole`] when
    /// `name` is registered under a different role (a friendlier error
    /// for typos like "wind = constant" when "constant" is a gravity
    /// model).
    pub fn resolve(&self, role: ModelRole, name: &str) -> Result<&ModelDescriptor, ScenarioError> {
        if let Some(descriptor) = self.by_role.get(&role).and_then(|models| models.get(name)) {
            return Ok(descriptor);
        }
        let actual = self
            .by_role
            .values()
            .flat_map(|models| models.values())
            .find(|descriptor| descriptor.name == name)
            .map(|descriptor| descriptor.role);
        match actual {
            Some(actual) => Err(ScenarioError::WrongModelRole {
                name: name.to_owned(),
                expected: role,
                actual,
            }),
            None => Err(ScenarioError::UnknownModel {
                role,
                name: name.to_owned(),
            }),
        }
    }
}

impl Default for ModelRegistry {
    fn default() -> Self {
        Self::phase2()
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn resolves_known_model() {
        let registry = ModelRegistry::phase1();
        let descriptor = registry
            .resolve(ModelRole::Vehicle, "point_mass")
            .expect("known model resolves");
        assert_eq!(descriptor.role, ModelRole::Vehicle);
    }

    #[test]
    fn rejects_unknown_model_with_unknown_error() {
        let err = ModelRegistry::phase1()
            .resolve(ModelRole::Vehicle, "rigid_stick")
            .unwrap_err();
        assert!(matches!(err, ScenarioError::UnknownModel { .. }));
    }

    #[test]
    fn detects_wrong_role_for_known_name() {
        let err = ModelRegistry::phase1()
            .resolve(ModelRole::Wind, "constant")
            .unwrap_err();
        match err {
            ScenarioError::WrongModelRole {
                expected, actual, ..
            } => {
                assert_eq!(expected, ModelRole::Wind);
                assert_eq!(actual, ModelRole::Gravity);
            }
            other => panic!("unexpected error variant: {other:?}"),
        }
    }

    #[test]
    fn phase2_registers_aero_thrust_and_rigid_body() {
        let registry = ModelRegistry::phase2();
        registry
            .resolve(ModelRole::Vehicle, "rigid_body")
            .expect("rigid_body");
        registry.resolve(ModelRole::Force, "aero").expect("aero");
        registry
            .resolve(ModelRole::Force, "thrust")
            .expect("thrust");
        registry
            .resolve(ModelRole::Atmosphere, "us_standard_1976")
            .expect("us_standard_1976");
        registry
            .resolve(ModelRole::Wind, "constant")
            .expect("wind constant");
        registry.resolve(ModelRole::Sensor, "imu").expect("imu");
        registry.resolve(ModelRole::Motor, "solid").expect("solid");
    }

    #[test]
    fn phase2_resolves_same_name_under_each_registered_role() {
        // `constant` is both a gravity and a wind model in Phase 2.
        // The resolver must return the role-specific entry, not bail
        // with WrongModelRole.
        let registry = ModelRegistry::phase2();
        let gravity = registry.resolve(ModelRole::Gravity, "constant").unwrap();
        assert_eq!(gravity.role, ModelRole::Gravity);
        let wind = registry.resolve(ModelRole::Wind, "constant").unwrap();
        assert_eq!(wind.role, ModelRole::Wind);
    }

    #[test]
    fn phase2_keeps_phase1_entries() {
        let registry = ModelRegistry::phase2();
        registry
            .resolve(ModelRole::Vehicle, "point_mass")
            .expect("point_mass");
        registry
            .resolve(ModelRole::Gravity, "constant")
            .expect("constant gravity");
        registry
            .resolve(ModelRole::Force, "gravity")
            .expect("gravity force");
    }
}
