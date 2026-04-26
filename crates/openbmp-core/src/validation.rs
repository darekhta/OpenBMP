//! Validation-status labels.
//!
//! Every model, dataset, scenario, and validation case in OpenBMP
//! declares one of these labels. See `docs/verification.md` for the
//! evidence required at each level.

use serde::{Deserialize, Serialize};

/// Validation status for an OpenBMP model, dataset, or scenario.
///
/// Labels are conservative; an artifact may claim a label only when
/// the evidence required by that label is in place.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ValidationStatus {
    /// Implemented or drafted, but not independently checked.
    #[default]
    Experimental,
    /// Internally consistent against unit/property tests.
    Checked,
    /// Compared against analytic or simple public examples.
    ValidatedToy,
    /// Compared against public academic benchmark cases. Still not
    /// operationally validated.
    Research,
}

impl ValidationStatus {
    /// Returns the canonical string label used in scenario files and
    /// telemetry metadata.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::Experimental => "experimental",
            Self::Checked => "checked",
            Self::ValidatedToy => "validated-toy",
            Self::Research => "research",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_experimental() {
        assert_eq!(ValidationStatus::default(), ValidationStatus::Experimental);
    }

    #[test]
    fn label_strings_are_canonical() {
        assert_eq!(ValidationStatus::Experimental.as_label(), "experimental");
        assert_eq!(ValidationStatus::Checked.as_label(), "checked");
        assert_eq!(ValidationStatus::ValidatedToy.as_label(), "validated-toy");
        assert_eq!(ValidationStatus::Research.as_label(), "research");
    }

    /// Compile-time check that `ValidationStatus` implements
    /// `serde::Serialize` and `serde::Deserialize`. End-to-end format
    /// round-trips are exercised in higher-level crates that pair
    /// `serde` with a concrete format implementation.
    #[test]
    fn serde_traits_implemented() {
        fn assert_serde<T: serde::Serialize + serde::de::DeserializeOwned>() {}
        assert_serde::<ValidationStatus>();
    }
}
