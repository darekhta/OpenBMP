//! Validated-then-activated typed table registry.
//!
//! Pattern adapted from NASA cFE TBL: a table is an immutable typed
//! blob (gain schedule, mass/CG sweep, schedule definition) that the
//! ground / runner uplinks into a *pending* slot, asks the registry
//! to validate, and then atomically activates by swapping the pending
//! and active cells.
//!
//! # Why two buffers
//!
//! Modules read from the active buffer continuously. The runner can
//! stage a candidate update in the pending buffer, run validation
//! synchronously, and only swap on success. A failed validation
//! leaves the active buffer untouched — the controller never reads a
//! half-initialised table.
//!
//! # Determinism
//!
//! Active-table iteration order is registration order. The two-buffer
//! swap is deterministic: pending becomes active, the previous active
//! is dropped. Validation runs synchronously on the calling thread.

use std::any::{Any, TypeId};

use indexmap::IndexMap;

use crate::error::TableError;

/// Marker trait implemented by every typed table.
pub trait Table: 'static + Clone {
    /// Canonical table name. Convention: `snake_case`, namespaced by
    /// owning module.
    const NAME: &'static str;

    /// Returns `Ok(())` if the candidate value is structurally valid,
    /// or `Err(reason)` if it must be rejected. Reasons are
    /// human-readable strings recorded in [`TableError::ValidationFailed`].
    ///
    /// # Errors
    ///
    /// Returns a human-readable validation failure string when the
    /// table content is rejected.
    fn validate(&self) -> Result<(), String>;
}

/// Runtime table registry.
#[derive(Default)]
pub struct Tables {
    cells: IndexMap<TypeId, TableCell>,
}

struct TableCell {
    name: &'static str,
    active: Box<dyn Any>,
    pending: Option<Box<dyn Any>>,
}

/// Registered table descriptor.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct TableInfo {
    /// Canonical table name (`Table::NAME`).
    pub name: &'static str,
    /// `true` if a pending value is staged.
    pub has_pending: bool,
}

impl Tables {
    /// Constructs an empty table registry.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Declares a table and seeds it with its initial active value.
    /// The initial value is validated before being installed.
    ///
    /// # Errors
    ///
    /// Returns [`TableError::DuplicateTable`] if the table was already
    /// declared, [`TableError::ValidationFailed`] if the initial value
    /// fails validation.
    pub fn declare<T: Table>(&mut self, initial: T) -> Result<(), TableError> {
        let id = TypeId::of::<T>();
        if self.cells.contains_key(&id) {
            return Err(TableError::DuplicateTable { table: T::NAME });
        }
        if let Err(reason) = initial.validate() {
            return Err(TableError::ValidationFailed {
                table: T::NAME,
                reason,
            });
        }
        self.cells.insert(
            id,
            TableCell {
                name: T::NAME,
                active: Box::new(initial),
                pending: None,
            },
        );
        Ok(())
    }

    /// Stages a candidate value in the pending buffer. Validation
    /// runs synchronously; on failure the pending buffer is cleared
    /// and the original error is returned.
    ///
    /// # Errors
    ///
    /// Returns [`TableError::UnknownTable`] if the table was not
    /// declared, [`TableError::ValidationFailed`] if validation fails.
    pub fn stage<T: Table>(&mut self, candidate: T) -> Result<(), TableError> {
        let id = TypeId::of::<T>();
        let cell = self
            .cells
            .get_mut(&id)
            .ok_or(TableError::UnknownTable { table: T::NAME })?;
        if let Err(reason) = candidate.validate() {
            cell.pending = None;
            return Err(TableError::ValidationFailed {
                table: T::NAME,
                reason,
            });
        }
        cell.pending = Some(Box::new(candidate));
        Ok(())
    }

    /// Atomically activates the staged pending buffer. Has no effect
    /// if no pending value is staged.
    ///
    /// # Errors
    ///
    /// Returns [`TableError::UnknownTable`] if the table was not
    /// declared.
    pub fn activate<T: Table>(&mut self) -> Result<bool, TableError> {
        let id = TypeId::of::<T>();
        let cell = self
            .cells
            .get_mut(&id)
            .ok_or(TableError::UnknownTable { table: T::NAME })?;
        if let Some(pending) = cell.pending.take() {
            cell.active = pending;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Stages and activates in one call. Convenience for boot-time
    /// loads that have no separate review step.
    ///
    /// # Errors
    ///
    /// Returns [`TableError::UnknownTable`] if the table was not
    /// declared, [`TableError::ValidationFailed`] if validation fails.
    pub fn replace<T: Table>(&mut self, candidate: T) -> Result<(), TableError> {
        self.stage::<T>(candidate)?;
        self.activate::<T>()?;
        Ok(())
    }

    /// Returns a reference to the active value of a table.
    ///
    /// # Errors
    ///
    /// Returns [`TableError::UnknownTable`] if the table was not
    /// declared.
    pub fn active<T: Table>(&self) -> Result<&T, TableError> {
        let id = TypeId::of::<T>();
        let cell = self
            .cells
            .get(&id)
            .ok_or(TableError::UnknownTable { table: T::NAME })?;
        cell.active
            .downcast_ref::<T>()
            .ok_or(TableError::UnknownTable { table: T::NAME })
    }

    /// Returns descriptors for every declared table in declaration
    /// order.
    #[must_use]
    pub fn tables(&self) -> Vec<TableInfo> {
        self.cells
            .values()
            .map(|c| TableInfo {
                name: c.name,
                has_pending: c.pending.is_some(),
            })
            .collect()
    }
}

impl std::fmt::Debug for Tables {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Tables")
            .field("tables", &self.tables())
            .finish()
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[derive(Clone, Debug, PartialEq)]
    struct GainSchedule {
        kp: Vec<f64>,
    }

    impl Table for GainSchedule {
        const NAME: &'static str = "test.gain_schedule";
        fn validate(&self) -> Result<(), String> {
            if self.kp.is_empty() {
                return Err("kp must be non-empty".to_string());
            }
            if self.kp.iter().any(|k| !k.is_finite() || *k < 0.0) {
                return Err("kp entries must be finite and non-negative".to_string());
            }
            Ok(())
        }
    }

    #[test]
    fn declare_and_read_active() {
        let mut tables = Tables::new();
        tables.declare(GainSchedule { kp: vec![1.0, 2.0] }).unwrap();
        let active = tables.active::<GainSchedule>().unwrap();
        assert_eq!(active.kp, vec![1.0, 2.0]);
    }

    #[test]
    fn stage_then_activate() {
        let mut tables = Tables::new();
        tables.declare(GainSchedule { kp: vec![1.0] }).unwrap();
        tables.stage(GainSchedule { kp: vec![3.0, 4.0] }).unwrap();
        // Active still old until activate.
        assert_eq!(tables.active::<GainSchedule>().unwrap().kp, vec![1.0]);
        let activated = tables.activate::<GainSchedule>().unwrap();
        assert!(activated);
        assert_eq!(tables.active::<GainSchedule>().unwrap().kp, vec![3.0, 4.0]);
    }

    #[test]
    fn validation_failure_keeps_active_intact() {
        let mut tables = Tables::new();
        tables.declare(GainSchedule { kp: vec![1.0] }).unwrap();
        let err = tables.stage(GainSchedule { kp: vec![] }).unwrap_err();
        assert!(matches!(err, TableError::ValidationFailed { .. }));
        // Active untouched.
        assert_eq!(tables.active::<GainSchedule>().unwrap().kp, vec![1.0]);
    }

    #[test]
    fn declare_with_invalid_initial_rejected() {
        let mut tables = Tables::new();
        let err = tables.declare(GainSchedule { kp: vec![] }).unwrap_err();
        assert!(matches!(err, TableError::ValidationFailed { .. }));
    }
}
