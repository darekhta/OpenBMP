//! Top-level [`Scenario`] entry point: parses TOML once, runs the
//! safety-and-units lint, deserialises into [`ScenarioDocument`], and
//! resolves relative paths against the scenario file directory.

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

use crate::document::ScenarioDocument;
use crate::error::ScenarioError;
use crate::files::ResolvedFile;
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
        scenario.validate_with_registry(&ModelRegistry::phase2())?;
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
    /// default [`ModelRegistry::phase2`] used by the parser entry
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

    /// Read every scenario-referenced external input file, compute its
    /// SHA-256 digest, and verify any declared pin.
    ///
    /// Telemetry output paths are excluded (they are written, not read,
    /// and may not exist before the run). Aero deck, motor file,
    /// per-sensor noise budget, and data-package references are all
    /// included with their declared `*_sha256` pins.
    ///
    /// Keys are deterministic field paths so repeated calls produce
    /// identical iteration order via `BTreeMap`.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError::ReferencedFileMissing`] when any file
    /// cannot be read, [`ScenarioError::Sha256Mismatch`] when a
    /// declared pin disagrees with the computed digest, or
    /// [`ScenarioError::InvalidSha256Pin`] when a pin is malformed.
    pub fn resolved_files(&self) -> Result<BTreeMap<String, ResolvedFile>, ScenarioError> {
        let mut files = BTreeMap::new();

        if let Some(aero) = &self.document.aero {
            let resolved = ResolvedFile::load(self.resolve_path(&aero.deck))?;
            resolved.verify_pin(aero.deck_sha256.as_deref())?;
            files.insert("aero.deck".to_owned(), resolved);
        }

        if let Some(motor) = self
            .document
            .propulsion
            .as_ref()
            .and_then(|prop| prop.motor.as_ref())
        {
            let resolved = ResolvedFile::load(self.resolve_path(&motor.file))?;
            resolved.verify_pin(motor.file_sha256.as_deref())?;
            files.insert("propulsion.motor.file".to_owned(), resolved);
        }

        if let Some(sensors) = &self.document.sensors {
            for (name, sensor) in sensors {
                if let Some(file) = &sensor.file {
                    let resolved = ResolvedFile::load(self.resolve_path(file))?;
                    resolved.verify_pin(sensor.file_sha256.as_deref())?;
                    files.insert(format!("sensors.{name}.file"), resolved);
                }
            }
        }

        if let Some(packages) = &self.document.data_packages {
            for (name, path) in packages {
                let resolved = ResolvedFile::load(self.resolve_path(path))?;
                files.insert(format!("data_packages.{name}"), resolved);
            }
        }

        Ok(files)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::document::WGS84_J2_DEFAULT;
    use openbmp_core::ValidationStatus;

    // Parser-test fixture loaded from the canonical Phase-1
    // analytic-toy scenario at
    // `scenarios/analytic-toy/constant-acceleration-drop.toml`.
    // This is the single source of truth for the analytic-toy
    // scenario; the parser tests below mutate this string via
    // `replace(...)` to construct negative-test variants. Real
    // validation cases live in `data/scenarios/` or `scenarios/`
    // per `docs/data-provenance.md`.
    const MINIMAL: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/analytic-toy/constant-acceleration-drop.toml"
    ));

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

    // Phase-2.10 sounding-rocket-shaped fixture. Exercises every new
    // optional block (`[aero]`, `[propulsion.motor]`, `[wind]`,
    // `[atmosphere]`, `[frames.local_origin]`, `[sensors.<name>]`)
    // plus the rigid-body initial-state fields. The Phase-2.10 parser
    // tests below mutate this fixture with `replace(...)` to construct
    // negative-test variants; the canonical Niskanen scenario lives at
    // `scenarios/sounding-rocket/niskanen-2009-chapter6.toml`.
    const SOUNDING_ROCKET: &str = r#"
openbmp.scenario = 1

[meta]
name = "sounding-rocket-fixture"
description = "Phase-2.10 parser fixture exercising rigid body, aero deck, motor, structured wind/atmosphere, frames local origin, sensors."
validation = "experimental"

[time]
start_s = 0.0
stop_s = 20.0
dt_s = 0.001
seed = 0xdeadbeef

[vehicle]
kind = "rigid_body"
mass_kg = 1.5
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[environment]
frame_profile = "wgs84-uniform-rotation"
gravity = "j2"
mu_m3_s2 = 3.986004418e14
r_e_m = 6378137.0
j2 = 1.082626683e-3
atmosphere = "us_standard_1976"
wind = "constant"

[atmosphere]
kind = "us_standard_1976"

[wind]
kind = "constant"
wind_ned_m_s = [0.0, 0.0, 0.0]

[frames]
profile = "wgs84-uniform-rotation"

[frames.local_origin]
latitude_deg = 60.18
longitude_deg = 24.83
height_m = 0.0
source = "Helsinki proxy launch site"

[aero]
deck = "../aero/synthetic-finned-cylinder.toml"

[propulsion.motor]
file = "../motors/synthetic-solid-textbook.toml"
ignite_at_s = 0.0
variant = "solid"

[forces]
models = ["gravity", "aero", "thrust"]

[sensors.imu]
kind = "imu"
file = "../sensors/imu-tactical.toml"

[sensors.barometer]
kind = "barometer"
file = "../sensors/baro.toml"

[sensors.truth]
kind = "ideal_state"

[telemetry]
output.parquet = "out/sounding-rocket.parquet"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    #[test]
    fn parses_sounding_rocket_scenario() {
        let scenario = Scenario::from_toml_str(SOUNDING_ROCKET).unwrap();
        assert_eq!(scenario.document.vehicle.kind, "rigid_body");
        assert_eq!(scenario.document.environment.gravity, "j2");
        assert!(scenario.document.aero.is_some());
        assert!(scenario.document.propulsion.is_some());
        assert!(scenario.document.wind.is_some());
        assert!(scenario.document.atmosphere.is_some());
        let frames = scenario.document.frames.as_ref().unwrap();
        assert!(frames.local_origin.is_some());
        let sensors = scenario.document.sensors.as_ref().unwrap();
        assert_eq!(sensors.len(), 3);
        assert_eq!(sensors.get("imu").unwrap().kind, "imu");
        assert_eq!(sensors.get("truth").unwrap().kind, "ideal_state");
    }

    #[test]
    fn rejects_rigid_body_without_quaternion() {
        let toml = SOUNDING_ROCKET.replace(
            "initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]\n",
            "",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "vehicle.initial_quaternion_body_to_eci_xyzw"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_point_mass_with_quaternion() {
        let toml = SOUNDING_ROCKET.replace(r#"kind = "rigid_body""#, r#"kind = "point_mass""#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "vehicle.initial_quaternion_body_to_eci_xyzw"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_non_unit_quaternion() {
        let toml = SOUNDING_ROCKET.replace(
            "initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]",
            "initial_quaternion_body_to_eci_xyzw = [1.0, 0.0, 0.0, 1.0]",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field == "vehicle.initial_quaternion_body_to_eci_xyzw"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_latitude_out_of_range() {
        let toml = SOUNDING_ROCKET.replace("latitude_deg = 60.18", "latitude_deg = 91.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field == "frames.local_origin.latitude_deg"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_atmosphere_kind_disagreement() {
        let toml = SOUNDING_ROCKET.replace(
            r#"atmosphere = "us_standard_1976""#,
            r#"atmosphere = "isothermal""#,
        );
        // The structured `[atmosphere].kind = "us_standard_1976"` now
        // conflicts with `environment.atmosphere = "isothermal"`.
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { .. }),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_wind_kind_disagreement() {
        let toml = SOUNDING_ROCKET.replace(
            "[wind]\nkind = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]",
            "[wind]\nkind = \"none\"",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { .. }),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_constant_wind_without_vector() {
        let toml = SOUNDING_ROCKET.replace("wind_ned_m_s = [0.0, 0.0, 0.0]\n", "");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "wind.wind_ned_m_s"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_flat_constant_wind_without_structured_block() {
        let toml = MINIMAL.replace(r#"wind          = "none""#, r#"wind          = "constant""#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "wind"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_isothermal_atmosphere_without_state() {
        let toml = SOUNDING_ROCKET
            .replace(
                r#"atmosphere = "us_standard_1976""#,
                r#"atmosphere = "isothermal""#,
            )
            .replace(
                r#"[atmosphere]
kind = "us_standard_1976""#,
                r#"[atmosphere]
kind = "isothermal""#,
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field.starts_with("atmosphere.")),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_flat_isothermal_atmosphere_without_structured_block() {
        let toml = MINIMAL.replace(
            r#"atmosphere    = "none""#,
            r#"atmosphere    = "isothermal""#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "atmosphere"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_ideal_state_sensor_with_file() {
        let toml = SOUNDING_ROCKET.replace(
            "[sensors.truth]\nkind = \"ideal_state\"",
            "[sensors.truth]\nkind = \"ideal_state\"\nfile = \"../sensors/truth.toml\"",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "sensors.truth.file"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_ideal_state_sensor_with_pin_only() {
        let toml = SOUNDING_ROCKET.replace(
            "[sensors.truth]\nkind = \"ideal_state\"",
            &format!(
                "[sensors.truth]\nkind = \"ideal_state\"\nfile_sha256 = \"{}\"",
                "0".repeat(64)
            ),
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "sensors.truth.file_sha256"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_imu_sensor_without_file() {
        let toml = SOUNDING_ROCKET.replace(
            "[sensors.imu]\nkind = \"imu\"\nfile = \"../sensors/imu-tactical.toml\"",
            "[sensors.imu]\nkind = \"imu\"",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "sensors.imu.file"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_constant_gravity_with_mu() {
        let toml = SOUNDING_ROCKET.replace(
            "gravity = \"j2\"\nmu_m3_s2 = 3.986004418e14\nr_e_m = 6378137.0\nj2 = 1.082626683e-3",
            "gravity = \"constant\"\ngravity_m_s2 = 9.80665\nmu_m3_s2 = 3.986004418e14",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "environment.mu_m3_s2"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_j2_gravity_without_r_e() {
        let toml = SOUNDING_ROCKET.replace("r_e_m = 6378137.0\n", "");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "environment.r_e_m"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_point_mass_gravity_with_j2_coefficients() {
        let toml = SOUNDING_ROCKET.replace(
            "gravity = \"j2\"\nmu_m3_s2 = 3.986004418e14\nr_e_m = 6378137.0\nj2 = 1.082626683e-3",
            "gravity = \"point_mass\"\nmu_m3_s2 = 3.986004418e14\nr_e_m = 6378137.0\nj2 = 1.082626683e-3",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "environment.r_e_m / environment.j2"),
            "got {err:?}",
        );
    }

    #[test]
    fn j2_gravity_uses_wgs84_default_when_j2_is_omitted() {
        let toml = SOUNDING_ROCKET.replace("j2 = 1.082626683e-3\n", "");
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        assert_eq!(
            scenario
                .document
                .environment
                .j2_or_wgs84_default()
                .unwrap()
                .to_bits(),
            WGS84_J2_DEFAULT.to_bits()
        );
    }

    #[test]
    fn rejects_unknown_motor_variant() {
        let toml = SOUNDING_ROCKET.replace(r#"variant = "solid""#, r#"variant = "liquid""#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnknownModel { .. }),
            "got {err:?}"
        );
    }

    #[test]
    fn rejects_aero_force_without_aero_block() {
        let toml = MINIMAL.replace(r#"models = ["gravity"]"#, r#"models = ["gravity", "aero"]"#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "aero"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_thrust_force_without_motor_block() {
        let toml = MINIMAL.replace(
            r#"models = ["gravity"]"#,
            r#"models = ["gravity", "thrust"]"#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "propulsion.motor"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_environment_and_frames_profile_disagreement() {
        let toml = format!("{MINIMAL}\n[frames]\nprofile = \"wgs84-uniform-rotation\"\n");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { .. }),
            "got {err:?}",
        );
    }

    #[test]
    fn phase1_scenario_continues_to_parse_under_phase2_registry() {
        // Byte-stability guard: the Phase-1 analytic-toy scenario must
        // continue to parse and validate identically under the Phase-2
        // model registry that 2.10.B installs.
        let scenario = Scenario::from_toml_str(MINIMAL).unwrap();
        assert_eq!(scenario.document.openbmp.scenario, 1);
        assert_eq!(scenario.document.vehicle.kind, "point_mass");
    }

    #[test]
    fn resolved_files_resolves_aero_motor_and_sensor_refs() {
        use std::fs;
        use tempfile::tempdir;

        let dir = tempdir().expect("tempdir");
        let scenario_dir = dir.path();
        // Stub the referenced files so ResolvedFile::load can read them.
        let aero_dir = scenario_dir.join("../aero");
        fs::create_dir_all(&aero_dir).expect("aero dir");
        fs::write(aero_dir.join("synthetic-finned-cylinder.toml"), "stub deck")
            .expect("write deck");
        let motor_dir = scenario_dir.join("../motors");
        fs::create_dir_all(&motor_dir).expect("motor dir");
        fs::write(
            motor_dir.join("synthetic-solid-textbook.toml"),
            "stub motor",
        )
        .expect("write motor");
        let sensors_dir = scenario_dir.join("../sensors");
        fs::create_dir_all(&sensors_dir).expect("sensors dir");
        fs::write(sensors_dir.join("imu-tactical.toml"), "stub imu").expect("write imu");
        fs::write(sensors_dir.join("baro.toml"), "stub baro").expect("write baro");

        let scenario = Scenario::from_toml_str_with_source_dir(
            SOUNDING_ROCKET,
            Some(scenario_dir.to_path_buf()),
        )
        .expect("parse");
        let files = scenario.resolved_files().expect("resolve");
        assert!(files.contains_key("aero.deck"));
        assert!(files.contains_key("propulsion.motor.file"));
        assert!(files.contains_key("sensors.imu.file"));
        assert!(files.contains_key("sensors.barometer.file"));
        // Three sensors but only two declare files (truth is ideal_state).
        assert!(!files.keys().any(|k| k == "sensors.truth.file"));
        // Every digest is 64 hex chars.
        for resolved in files.values() {
            assert_eq!(resolved.sha256_hex.len(), 64);
        }
    }

    #[test]
    fn resolved_files_fails_closed_on_missing_file() {
        let scenario = Scenario::from_toml_str_with_source_dir(
            SOUNDING_ROCKET,
            Some(PathBuf::from("/nonexistent/openbmp")),
        )
        .expect("parse");
        let err = scenario.resolved_files().unwrap_err();
        assert!(matches!(err, ScenarioError::ReferencedFileMissing { .. }));
    }

    #[test]
    fn resolved_files_fails_closed_on_pin_mismatch() {
        use std::fs;
        use tempfile::tempdir;

        let dir = tempdir().expect("tempdir");
        let scenario_dir = dir.path();
        let aero_dir = scenario_dir.join("../aero");
        fs::create_dir_all(&aero_dir).expect("aero dir");
        fs::write(
            aero_dir.join("synthetic-finned-cylinder.toml"),
            "actual contents",
        )
        .expect("write deck");
        let motor_dir = scenario_dir.join("../motors");
        fs::create_dir_all(&motor_dir).expect("motor dir");
        fs::write(
            motor_dir.join("synthetic-solid-textbook.toml"),
            "stub motor",
        )
        .expect("write motor");
        let sensors_dir = scenario_dir.join("../sensors");
        fs::create_dir_all(&sensors_dir).expect("sensors dir");
        fs::write(sensors_dir.join("imu-tactical.toml"), "stub imu").expect("write imu");
        fs::write(sensors_dir.join("baro.toml"), "stub baro").expect("write baro");

        let bad_pin = "0".repeat(64);
        let toml = SOUNDING_ROCKET.replace(
            r#"deck = "../aero/synthetic-finned-cylinder.toml""#,
            &format!(
                "deck = \"../aero/synthetic-finned-cylinder.toml\"\ndeck_sha256 = \"{bad_pin}\""
            ),
        );
        let scenario =
            Scenario::from_toml_str_with_source_dir(&toml, Some(scenario_dir.to_path_buf()))
                .expect("parse");
        let err = scenario.resolved_files().unwrap_err();
        assert!(matches!(err, ScenarioError::Sha256Mismatch { .. }));
    }
}
