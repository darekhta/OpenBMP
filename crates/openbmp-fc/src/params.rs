//! Typed parameter registry.
//!
//! Each module declares a parameter section by registering its typed
//! `Params` struct under a canonical section name. The runner loads
//! parameter values at boot (typically from a TOML file via
//! `openbmp-scenario`) and either calls [`Parameters::declare`] with
//! the resulting struct or hot-tunes individual sections via
//! [`Parameters::update`].
//!
//! For Phase 4.1 the registry stores typed sections only; round-trip
//! TOML / JSON serialisation is implemented behind the `serde`
//! feature of `openbmp-fc`. The `parameter_update` bus topic that
//! signals consumers of a fresh value is reserved for Phase 4.6 — for
//! 4.1, modules read their section once at startup.
//!
//! # Determinism
//!
//! - Section iteration order is registration order.
//! - The registry does not allocate after declaration; subsequent
//!   updates overwrite the in-place storage.

use std::any::{Any, TypeId};

use indexmap::IndexMap;

use crate::error::ParamError;

/// Marker trait implemented by every typed parameter section.
pub trait ParamSection: 'static + Clone {
    /// Canonical section name. Convention: `snake_case`, namespaced by
    /// owning module (`estimator.ekf`, `autopilot.three_loop`, etc.).
    const NAME: &'static str;
}

/// Runtime parameter registry.
#[derive(Default)]
pub struct Parameters {
    sections: IndexMap<TypeId, ParamCell>,
}

struct ParamCell {
    name: &'static str,
    value: Box<dyn Any>,
}

/// Registered section descriptor exposed via [`Parameters::sections`]
/// for dictionary generation.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ParamSectionInfo {
    /// Canonical section name (`ParamSection::NAME`).
    pub name: &'static str,
}

impl Parameters {
    /// Constructs an empty registry.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Declares a section and seeds it with its default value.
    ///
    /// # Errors
    ///
    /// Returns [`ParamError::DuplicateSection`] if the section was
    /// already declared.
    pub fn declare<T: ParamSection>(&mut self, defaults: T) -> Result<(), ParamError> {
        let id = TypeId::of::<T>();
        if self.sections.contains_key(&id) {
            return Err(ParamError::DuplicateSection { section: T::NAME });
        }
        self.sections.insert(
            id,
            ParamCell {
                name: T::NAME,
                value: Box::new(defaults),
            },
        );
        Ok(())
    }

    /// Replaces a section's value.
    ///
    /// # Errors
    ///
    /// Returns [`ParamError::UnknownSection`] if the section was not
    /// declared, [`ParamError::TypeMismatch`] if the storage cell was
    /// declared with a different concrete type.
    pub fn update<T: ParamSection>(&mut self, value: T) -> Result<(), ParamError> {
        let id = TypeId::of::<T>();
        let cell = self
            .sections
            .get_mut(&id)
            .ok_or(ParamError::UnknownSection { section: T::NAME })?;
        let storage = cell
            .value
            .downcast_mut::<T>()
            .ok_or(ParamError::TypeMismatch { section: T::NAME })?;
        *storage = value;
        Ok(())
    }

    /// Returns a reference to a section's value.
    ///
    /// # Errors
    ///
    /// Returns [`ParamError::UnknownSection`] if the section was not
    /// declared.
    pub fn get<T: ParamSection>(&self) -> Result<&T, ParamError> {
        let id = TypeId::of::<T>();
        let cell = self
            .sections
            .get(&id)
            .ok_or(ParamError::UnknownSection { section: T::NAME })?;
        cell.value
            .downcast_ref::<T>()
            .ok_or(ParamError::TypeMismatch { section: T::NAME })
    }

    /// Returns a clone of a section's value.
    ///
    /// # Errors
    ///
    /// Returns [`ParamError::UnknownSection`] if the section was not
    /// declared.
    pub fn clone_section<T: ParamSection>(&self) -> Result<T, ParamError> {
        self.get::<T>().cloned()
    }

    /// Returns descriptors for every declared section in declaration
    /// order.
    #[must_use]
    pub fn sections(&self) -> Vec<ParamSectionInfo> {
        self.sections
            .values()
            .map(|c| ParamSectionInfo { name: c.name })
            .collect()
    }
}

impl std::fmt::Debug for Parameters {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Parameters")
            .field("sections", &self.sections())
            .finish()
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[derive(Clone, Debug, PartialEq)]
    struct Demo {
        gain: f64,
    }

    impl ParamSection for Demo {
        const NAME: &'static str = "test.demo";
    }

    #[test]
    fn declare_get_update_round_trip() {
        let mut params = Parameters::new();
        params.declare(Demo { gain: 1.0 }).unwrap();
        let v = params.get::<Demo>().unwrap();
        assert!((v.gain - 1.0).abs() < f64::EPSILON);

        params.update(Demo { gain: 2.5 }).unwrap();
        let v = params.get::<Demo>().unwrap();
        assert!((v.gain - 2.5).abs() < f64::EPSILON);
    }

    #[test]
    fn declare_twice_rejected() {
        let mut params = Parameters::new();
        params.declare(Demo { gain: 1.0 }).unwrap();
        let err = params.declare(Demo { gain: 2.0 }).unwrap_err();
        assert!(matches!(err, ParamError::DuplicateSection { .. }));
    }

    #[test]
    fn get_undeclared_rejected() {
        let params = Parameters::new();
        let err = params.get::<Demo>().unwrap_err();
        assert!(matches!(err, ParamError::UnknownSection { .. }));
    }
}
