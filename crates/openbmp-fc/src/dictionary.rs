//! Build-time dictionary generator.
//!
//! The dictionary is a typed snapshot of every topic, parameter
//! section, table, and registered job the controller binary speaks.
//! It is dumped to JSON at boot so external consumers (ground
//! tooling, log analysers, replay tools) can parse a flight log
//! without recompiling against the controller crate.
//!
//! Pattern adapted from F Prime's autocoded dictionary.
//!
//! # Format
//!
//! ```json
//! {
//!   "schema_version": 1,
//!   "controller_name": "openbmp-fc",
//!   "topics": [{"name": "...", "version": 1}],
//!   "parameters": [{"name": "..."}],
//!   "tables": [{"name": "..."}],
//!   "jobs": [{"name": "...", "trigger": "...", "budget_us": 0, "priority": 0}]
//! }
//! ```

use thiserror::Error;

use crate::bus::Bus;
use crate::params::Parameters;
use crate::scheduler::{Scheduler, Trigger};
use crate::tables::Tables;

/// Errors emitted by [`Dictionary::dump_json`].
#[derive(Debug, Error)]
pub enum DictionaryError {
    /// JSON encoding of the dictionary failed. Returned only when the
    /// `serde` feature is enabled and the encoder reports an error.
    #[cfg(feature = "serde")]
    #[error("dictionary: failed to serialise to json: {0}")]
    Serialize(#[from] serde_json::Error),
    /// JSON dumping is unavailable because the `serde` feature is
    /// disabled.
    #[cfg(not(feature = "serde"))]
    #[error("dictionary: serde feature is disabled; rebuild with `--features serde`")]
    SerdeDisabled,
}

/// Snapshot of the controller's introspectable surface at any
/// moment.
#[derive(Debug, Clone)]
pub struct Dictionary<'a> {
    /// Bus topic descriptors.
    pub bus: &'a Bus,
    /// Parameter section descriptors.
    pub params: &'a Parameters,
    /// Table descriptors.
    pub tables: &'a Tables,
    /// Scheduled job descriptors.
    pub scheduler: &'a Scheduler,
}

impl<'a> Dictionary<'a> {
    /// Constructs a dictionary view over the given subsystems.
    #[must_use]
    pub fn new(
        bus: &'a Bus,
        params: &'a Parameters,
        tables: &'a Tables,
        scheduler: &'a Scheduler,
    ) -> Self {
        Self {
            bus,
            params,
            tables,
            scheduler,
        }
    }

    /// Returns a deterministic, human-readable text dump of the
    /// dictionary. Always available regardless of feature flags;
    /// useful for `tracing`-style diagnostic logs.
    #[must_use]
    pub fn render_text(&self) -> String {
        use std::fmt::Write as _;
        let mut out = String::new();
        out.push_str("openbmp-fc dictionary\n");
        out.push_str("=====================\n");
        out.push_str("topics:\n");
        for t in self.bus.topics() {
            let _ = writeln!(out, "  {} v{}", t.name, t.version);
        }
        out.push_str("parameters:\n");
        for s in self.params.sections() {
            let _ = writeln!(out, "  {}", s.name);
        }
        out.push_str("tables:\n");
        for t in self.tables.tables() {
            let _ = writeln!(out, "  {}", t.name);
        }
        out.push_str("jobs:\n");
        for j in self.scheduler.jobs() {
            let trigger = match j.trigger {
                Trigger::Periodic { period_ticks } => format!("periodic({period_ticks})"),
                Trigger::TopicUpdated { topic_name } => format!("on_topic({topic_name})"),
            };
            let _ = writeln!(
                out,
                "  {} trigger={} budget_us={} priority={}",
                j.name, trigger, j.budget_us, j.priority
            );
        }
        out
    }

    /// Renders the dictionary as JSON.
    ///
    /// # Errors
    ///
    /// Returns [`DictionaryError::Serialize`] on serialisation
    /// failure (only possible with the `serde` feature). With the
    /// `serde` feature disabled, returns
    /// [`DictionaryError::SerdeDisabled`].
    #[cfg(feature = "serde")]
    pub fn dump_json(&self) -> Result<String, DictionaryError> {
        let payload = serde_json::json!({
            "schema_version": 1,
            "controller_name": "openbmp-fc",
            "topics": self.bus.topics().iter().map(|t| serde_json::json!({
                "name": t.name,
                "version": t.version,
            })).collect::<Vec<_>>(),
            "parameters": self.params.sections().iter().map(|s| serde_json::json!({
                "name": s.name,
            })).collect::<Vec<_>>(),
            "tables": self.tables.tables().iter().map(|t| serde_json::json!({
                "name": t.name,
            })).collect::<Vec<_>>(),
            "jobs": self.scheduler.jobs().iter().map(|j| {
                let trigger = match j.trigger {
                    Trigger::Periodic { period_ticks } => serde_json::json!({"kind": "periodic", "period_ticks": period_ticks}),
                    Trigger::TopicUpdated { topic_name } => serde_json::json!({"kind": "on_topic", "topic": topic_name}),
                };
                serde_json::json!({
                    "name": j.name,
                    "trigger": trigger,
                    "budget_us": j.budget_us,
                    "priority": j.priority,
                })
            }).collect::<Vec<_>>(),
        });
        serde_json::to_string_pretty(&payload).map_err(DictionaryError::Serialize)
    }

    /// Renders the dictionary as JSON. Returns
    /// [`DictionaryError::SerdeDisabled`] when the `serde` feature is
    /// not active.
    ///
    /// # Errors
    ///
    /// Always returns [`DictionaryError::SerdeDisabled`] when the
    /// `serde` feature is disabled.
    #[cfg(not(feature = "serde"))]
    pub fn dump_json(&self) -> Result<String, DictionaryError> {
        Err(DictionaryError::SerdeDisabled)
    }
}
