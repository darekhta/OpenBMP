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
        Self::phase1()
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
}
