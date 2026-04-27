//! Tiny `require_*` validators shared across [`crate::document`] and
//! [`crate::solver`].

use std::collections::BTreeSet;

use crate::error::ScenarioError;

pub(crate) fn require_non_empty(field: &str, value: &str) -> Result<(), ScenarioError> {
    if value.trim().is_empty() {
        Err(ScenarioError::EmptyField {
            field: field.to_owned(),
        })
    } else {
        Ok(())
    }
}

pub(crate) fn require_finite(field: &str, value: f64) -> Result<(), ScenarioError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value,
            rule: "must be finite",
        })
    }
}

pub(crate) fn require_positive(field: &str, value: f64) -> Result<(), ScenarioError> {
    if !value.is_finite() {
        return Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value,
            rule: "must be finite",
        });
    }
    if value <= 0.0 {
        return Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value,
            rule: "must be positive",
        });
    }
    Ok(())
}

pub(crate) fn require_finite_array(field: &str, values: &[f64]) -> Result<(), ScenarioError> {
    for value in values {
        require_finite(field, *value)?;
    }
    Ok(())
}

pub(crate) fn require_non_empty_list(field: &str, values: &[String]) -> Result<(), ScenarioError> {
    if values.is_empty() {
        Err(ScenarioError::EmptyList {
            field: field.to_owned(),
        })
    } else {
        Ok(())
    }
}

pub(crate) fn require_unique(field: &str, values: &[String]) -> Result<(), ScenarioError> {
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(value.as_str()) {
            return Err(ScenarioError::DuplicateValue {
                field: field.to_owned(),
                value: value.clone(),
            });
        }
    }
    Ok(())
}

pub(crate) fn require_supported(
    field: &str,
    value: &str,
    supported: &[&str],
) -> Result<(), ScenarioError> {
    if supported.contains(&value) {
        Ok(())
    } else {
        Err(ScenarioError::UnsupportedValue {
            field: field.to_owned(),
            value: value.to_owned(),
        })
    }
}

pub(crate) fn require_positive_u32(field: &str, value: u32) -> Result<(), ScenarioError> {
    if value == 0 {
        Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value: 0.0,
            rule: "must be greater than zero",
        })
    } else {
        Ok(())
    }
}

pub(crate) fn require_in_range(
    field: &str,
    value: f64,
    min: f64,
    max: f64,
) -> Result<(), ScenarioError> {
    require_finite(field, value)?;
    if value < min || value > max {
        return Err(ScenarioError::InvalidNumber {
            field: field.to_owned(),
            value,
            rule: "must be within declared range",
        });
    }
    Ok(())
}

pub(crate) fn validate_frame_profile(field: &str, value: &str) -> Result<(), ScenarioError> {
    require_supported(
        field,
        value,
        &[
            "toy-fixed-earth",
            "wgs84-uniform-rotation",
            "iers-tabulated",
            "spice-reference",
        ],
    )
}
