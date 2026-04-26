//! Top-level [`Scenario`] entry point: parses TOML once, runs the
//! safety-and-units lint, deserialises into [`ScenarioDocument`], and
//! resolves relative paths against the scenario file directory.

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

use crate::document::ScenarioDocument;
use crate::error::ScenarioError;
use crate::lint;
use crate::registry::ModelRegistry;

/// Parsed scenario plus source-directory context.
#[derive(Clone, Debug, PartialEq)]
pub struct Scenario {
    /// Parsed scenario document.
    pub document: ScenarioDocument,
    source_dir: Option<PathBuf>,
}

impl Scenario {
    /// Parse and validate a scenario from a TOML string.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for TOML parse failures and validation
    /// failures.
    pub fn from_toml_str(toml: &str) -> Result<Self, ScenarioError> {
        Self::from_toml_str_with_source_dir(toml, None::<PathBuf>)
    }

    /// Parse and validate a scenario from a TOML string with an
    /// optional source directory used to resolve relative paths.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for TOML parse failures and validation
    /// failures.
    pub fn from_toml_str_with_source_dir(
        toml: &str,
        source_dir: Option<impl Into<PathBuf>>,
    ) -> Result<Self, ScenarioError> {
        // Parse TOML once. We lint the untyped representation first so
        // the user gets a unit / frame / safety-name error before the
        // less-helpful serde "unknown field" error fires.
        let value: toml::Value = toml::from_str(toml)?;
        lint::lint(&value)?;

        // Re-decode from the parsed Value, avoiding a second round of
        // tokenisation.
        let document: ScenarioDocument = value.try_into()?;

        let scenario = Self {
            document,
            source_dir: source_dir.map(Into::into),
        };
        scenario.validate_with_registry(&ModelRegistry::phase1())?;
        Ok(scenario)
    }

    /// Parse and validate a scenario file. Relative paths inside the
    /// scenario resolve against the file's parent directory.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError::ReadFile`] when the file cannot be
    /// read, plus the same parse / validation errors as
    /// [`Self::from_toml_str`].
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, ScenarioError> {
        let path = path.as_ref();
        let content = fs::read_to_string(path).map_err(|source| ScenarioError::ReadFile {
            path: path.to_path_buf(),
            source,
        })?;
        let source_dir = path.parent().map(Path::to_path_buf);
        Self::from_toml_str_with_source_dir(&content, source_dir)
    }

    /// Validate against an explicit model registry (overrides the
    /// default [`ModelRegistry::phase1`] used by the parser entry
    /// points).
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for invalid values or unknown models.
    pub fn validate_with_registry(&self, registry: &ModelRegistry) -> Result<(), ScenarioError> {
        self.document.validate(registry)
    }

    /// Resolve a path relative to the scenario file directory.
    /// Absolute paths are returned unchanged.
    #[must_use]
    pub fn resolve_path(&self, path: impl AsRef<Path>) -> PathBuf {
        let path = path.as_ref();
        if path.is_absolute() {
            return path.to_path_buf();
        }
        match &self.source_dir {
            Some(source_dir) => source_dir.join(path),
            None => path.to_path_buf(),
        }
    }

    /// Declared telemetry and data-package paths after resolution.
    ///
    /// Keys are deterministic field paths
    /// (`telemetry.output.csv`, `data_packages.<name>`); the order is
    /// fixed by `BTreeMap`.
    #[must_use]
    pub fn resolved_paths(&self) -> BTreeMap<String, PathBuf> {
        let mut paths = BTreeMap::new();
        let output = &self.document.telemetry.output;
        if let Some(path) = &output.csv {
            paths.insert("telemetry.output.csv".to_owned(), self.resolve_path(path));
        }
        if let Some(path) = &output.json {
            paths.insert("telemetry.output.json".to_owned(), self.resolve_path(path));
        }
        if let Some(path) = &output.parquet {
            paths.insert(
                "telemetry.output.parquet".to_owned(),
                self.resolve_path(path),
            );
        }
        if let Some(packages) = &self.document.data_packages {
            for (name, path) in packages {
                paths.insert(format!("data_packages.{name}"), self.resolve_path(path));
            }
        }
        paths
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::ValidationStatus;

    const MINIMAL: &str = r#"
openbmp.scenario = 1

[meta]
name = "constant-acceleration-drop"
description = "Analytic toy scenario for vertical acceleration."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 10.0
dt_s = 0.01
seed = 42

[vehicle]
kind = "point_mass"
mass_kg = 1.0
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/constant-acceleration-drop.csv"
output.parquet = "out/constant-acceleration-drop.parquet"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    #[test]
    fn parses_minimal_scenario() {
        let scenario = Scenario::from_toml_str(MINIMAL).unwrap();
        assert_eq!(scenario.document.openbmp.scenario, 1);
        assert_eq!(
            scenario.document.meta.validation,
            ValidationStatus::ValidatedToy
        );
        assert_eq!(scenario.document.vehicle.kind, "point_mass");
    }

    #[test]
    fn resolves_paths_against_source_dir() {
        let scenario =
            Scenario::from_toml_str_with_source_dir(MINIMAL, Some(PathBuf::from("/tmp/scenario")))
                .unwrap();
        let paths = scenario.resolved_paths();
        assert_eq!(
            paths.get("telemetry.output.csv").unwrap(),
            &PathBuf::from("/tmp/scenario/out/constant-acceleration-drop.csv")
        );
    }

    #[test]
    fn rejects_unknown_fields() {
        let toml = MINIMAL.replace("mass_kg = 1.0", "mass_kg = 1.0\nextra_kg = 2.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::ParseToml(_)));
    }

    #[test]
    fn rejects_unknown_models() {
        let toml = MINIMAL.replace(r#"kind = "point_mass""#, r#"kind = "rigid_stick""#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::UnknownModel { .. }));
    }

    #[test]
    fn rejects_safety_limited_names() {
        let toml = MINIMAL.replace(
            r#"name = "constant-acceleration-drop""#,
            r#"name = "target-demo""#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::SafetyName { .. }));
    }

    #[test]
    fn safety_lint_has_global_priority_over_unit_lint() {
        let toml = MINIMAL
            .replace(
                r#"name = "constant-acceleration-drop""#,
                r#"name = "target-demo""#,
            )
            .replace("[environment]\n", "[environment]\nbad = 1.0\n");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::SafetyName { .. }));
    }

    #[test]
    fn rejects_missing_unit_suffix_before_serde_unknown_field() {
        let toml = MINIMAL.replace("mass_kg = 1.0", "mass = 1.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::MissingUnitSuffix { .. }));
    }

    #[test]
    fn rejects_missing_frame_suffix_on_numeric_vector() {
        let toml = MINIMAL.replace(
            "initial_position_eci_m = [0.0, 0.0, 0.0]",
            "initial_position_m = [0.0, 0.0, 0.0]",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::MissingFrameSuffix { .. }));
    }

    #[test]
    fn rejects_empty_force_model_list() {
        let toml = MINIMAL.replace(r#"models = ["gravity"]"#, "models = []");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::EmptyList { .. }));
    }

    #[test]
    fn rejects_invalid_time_range() {
        let toml = MINIMAL.replace("stop_s = 10.0", "stop_s = 0.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::InvalidNumber { .. }));
    }

    #[test]
    fn rejects_missing_telemetry_outputs() {
        let toml = MINIMAL.replace(
            "output.csv = \"out/constant-acceleration-drop.csv\"\noutput.parquet = \"out/constant-acceleration-drop.parquet\"",
            "",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::MissingTelemetryOutput));
    }
}
