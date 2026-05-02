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
        let mut document: ScenarioDocument = value.try_into()?;

        // Phase-3.13.E: synthesise the default `[forces]` list from the
        // assembly when the scenario does not declare one explicitly.
        // After this point the rest of the runner sees a populated
        // `Option<ForcesConfig>` and behaves as if the scenario had
        // hand-listed the derived models.
        if document.forces.is_none() {
            document.forces = Some(crate::ForcesConfig {
                models: document.resolved_force_models(),
            });
        }

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
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::document::{
        FcAntiWindupConfig, FcIndiConfig, FcIndiFilterKind, FcLqrConfig, FcRateLoopKind,
        WGS84_J2_DEFAULT,
    };
    use openbmp_core::ValidationStatus;

    // Parser-test fixture loaded from the canonical Phase-1
    // analytic-toy scenario at
    // `scenarios/analytic-toy/constant-acceleration-drop.toml`.
    // This is the single source of truth for the analytic-toy
    // scenario; the parser tests below mutate this string via
    // `replace(...)` to construct negative-test variants. Real
    // validation cases live under `scenarios/<category>/` per
    // `docs/data-provenance.md`.
    const MINIMAL: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/analytic-toy/constant-acceleration-drop.toml"
    ));

    #[test]
    fn parses_minimal_scenario() {
        let scenario = Scenario::from_toml_str(MINIMAL).unwrap();
        assert_eq!(scenario.document.openbmp.scenario, 2);
        assert_eq!(
            scenario.document.meta.validation,
            ValidationStatus::ValidatedToy
        );
        assert_eq!(scenario.document.vehicle.kind, "point_mass");
    }

    #[test]
    fn rejects_v1_schema_with_migration_pointer() {
        let legacy_version = 1_u16;
        let toml = MINIMAL.replace(
            "openbmp.scenario = 2",
            &format!("openbmp.scenario = {legacy_version}"),
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        let message = err.to_string();
        assert!(
            message.contains(&format!("found openbmp.scenario = {legacy_version}")),
            "message should report found schema version: {message}"
        );
        assert!(
            message.contains("expected one of [2, 3]"),
            "message should report supported schema version list: {message}"
        );
        assert!(
            message.contains("docs/scenario-format.md#migrating-v1-scenarios-to-v2"),
            "message should point at the migration section: {message}"
        );
    }

    #[test]
    fn parses_minimal_scenario_under_v3_header() {
        // Phase 5.0 — v3 scenarios that include only Phase-3 / Phase-4
        // fields parse and validate identically to v2; only the header
        // value changes.
        let toml = MINIMAL.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario = Scenario::from_toml_str(&toml).expect("v3 minimal should validate");
        assert_eq!(scenario.document.openbmp.scenario, 3);
    }

    fn assert_phase5_reserved_under_v2(toml: &str, expected_field: &str) {
        let err = Scenario::from_toml_str(toml).unwrap_err();
        match err {
            ScenarioError::SchemaVersionFieldReserved {
                field,
                required,
                found,
            } => {
                assert_eq!(field, expected_field);
                assert_eq!(required, 3);
                assert_eq!(found, 2);
            }
            other => {
                panic!("expected SchemaVersionFieldReserved for {expected_field}, got: {other:?}")
            }
        }
    }

    fn assert_phase5_deferred_under_v3(toml: &str, expected_field: &str, expected_phase: &str) {
        let v3 = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&v3).unwrap_err();
        match err {
            ScenarioError::ElementDeferredToFuturePhase { field, deferred_to } => {
                assert_eq!(field, expected_field);
                assert_eq!(deferred_to, expected_phase);
            }
            other => {
                panic!("expected ElementDeferredToFuturePhase for {expected_field}, got: {other:?}")
            }
        }
    }

    const SCHEDULE_BLOCK: &str = r#"
[schedule]
base_hz = 1000

[[schedule.group]]
label = "env"
hz = 100
members = ["atmosphere", "gravity", "wind"]

[[schedule.group]]
label = "fc"
hz = 50
members = ["estimator", "autopilot"]
"#;

    const MULTI_BODY_BLOCK: &str = r#"
[multi_body]

[[multi_body.separation]]
event_id = "fairing_separation"
upper_body_id = "main"
lower_body_id = "lower_stage"
upper_delta_v_body_m_s = [0.0, 0.0, 0.5]
lower_delta_v_body_m_s = [0.0, 0.0, -0.5]
"#;

    #[test]
    fn schedule_block_is_v3_only() {
        let toml_v2 = format!("{MINIMAL}{SCHEDULE_BLOCK}");
        assert_phase5_reserved_under_v2(&toml_v2, "schedule");
        assert_phase5_deferred_under_v3(&toml_v2, "schedule", "Phase 5.D.1");
    }

    #[test]
    fn multi_body_block_is_v3_only() {
        let toml_v2 = format!("{MINIMAL}{MULTI_BODY_BLOCK}");
        assert_phase5_reserved_under_v2(&toml_v2, "multi_body");
        assert_phase5_deferred_under_v3(&toml_v2, "multi_body", "Phase 5.D.2");
    }

    #[test]
    fn v2_with_multiple_phase5_blocks_reports_first_reserved_field() {
        let toml_v2 = format!("{MINIMAL}{SCHEDULE_BLOCK}{MULTI_BODY_BLOCK}");
        assert_phase5_reserved_under_v2(&toml_v2, "schedule");
    }

    const SCHEDULE_NON_DIVIDING_BLOCK: &str = r#"
[schedule]
base_hz = 1000

[[schedule.group]]
label = "broken"
hz = 333
members = ["x"]
"#;

    #[test]
    fn schedule_block_rejects_non_dividing_group_rate() {
        let toml = append(MINIMAL, SCHEDULE_NON_DIVIDING_BLOCK)
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        // The non-dividing-group-rate error fires from
        // ScheduleConfig::validate before the deferred-phase check
        // returns, because the gate runs the per-block validate first.
        match err {
            ScenarioError::InvalidNumber { field, rule, .. } => {
                assert_eq!(field, "schedule.group[0].hz");
                assert!(rule.contains("must divide"));
            }
            other => panic!("expected InvalidNumber, got {other:?}"),
        }
    }

    const SCHEDULE_EMPTY_MEMBER_BLOCK: &str = r#"
[schedule]
base_hz = 1000

[[schedule.group]]
label = "broken"
hz = 100
members = [""]
"#;

    #[test]
    fn schedule_block_rejects_empty_member_id() {
        let toml = append(MINIMAL, SCHEDULE_EMPTY_MEMBER_BLOCK)
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::EmptyField { field } => {
                assert_eq!(field, "schedule.group[0].members[0]");
            }
            other => panic!("expected EmptyField, got {other:?}"),
        }
    }

    const SCHEDULE_UNKNOWN_FIELD_BLOCK: &str = r"
[schedule]
base_hz = 1000
bogus_field = 1
";

    #[test]
    fn schedule_block_rejects_unknown_field() {
        // serde(deny_unknown_fields) on ScheduleConfig — an unknown
        // top-level key under `[schedule]` produces a TOML parse error,
        // not a deferred-phase diagnostic.
        let toml = append(MINIMAL, SCHEDULE_UNKNOWN_FIELD_BLOCK)
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::ParseToml(_)), "got {err:?}");
    }

    #[test]
    fn egm2008_gravity_is_v3_only() {
        // The Phase-5.0 v3 gate fires before the cross-field check that
        // would otherwise reject `gravity_m_s2` against a non-constant
        // gravity model, so the test only needs to flip the gravity
        // selector. The aligned-equals layout in the canonical scenario
        // (`gravity       = "constant"`) is matched verbatim.
        let toml = MINIMAL.replace(
            "gravity       = \"constant\"",
            "gravity       = \"egm2008\"",
        );
        assert_phase5_reserved_under_v2(&toml, "environment.gravity = \"egm2008\"");
        assert_phase5_deferred_under_v3(&toml, "environment.gravity = \"egm2008\"", "Phase 5.C.2");
    }

    /// Canonical Phase-4 FC scenario used as the base for the v3-only
    /// FC sub-block tests. Loaded via `include_str!` so the test
    /// remains in sync with the shipped scenario contract.
    const FC_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
    ));

    fn fc_v2_scenario() -> &'static str {
        FC_FIXTURE
    }

    fn append(base: &str, block: &str) -> String {
        format!("{base}\n{block}")
    }

    #[test]
    fn fc_estimator_lanes_block_is_v3_only() {
        let block = r#"
[fc.estimator_lanes]
voter = "best_by_covariance_trace"

[[fc.estimator_lanes.lane]]
id        = "primary"
estimator = "ekf"

[[fc.estimator_lanes.lane]]
id        = "spare"
estimator = "mekf"
"#;
        let toml = append(fc_v2_scenario(), block);
        assert_phase5_reserved_under_v2(&toml, "fc.estimator_lanes");
        assert_phase5_deferred_under_v3(&toml, "fc.estimator_lanes", "Phase 5.B.2");
    }

    #[test]
    fn fc_autopilot_allocation_block_is_v3_only() {
        let block = r#"
[fc.autopilot_allocation]
kind          = "prioritised_redistributed"
axis_priority = ["roll", "yaw", "pitch"]
"#;
        let toml = append(fc_v2_scenario(), block);
        assert_phase5_reserved_under_v2(&toml, "fc.autopilot_allocation");
        assert_phase5_deferred_under_v3(&toml, "fc.autopilot_allocation", "Phase 5.A.5");
    }

    #[test]
    fn fc_fdir_detector_block_is_v3_only() {
        let block = r#"
[fc.fdir.detector]
kind            = "windowed_glrt"
window_samples  = 32
parity_threshold = 25.0
"#;
        let toml = append(fc_v2_scenario(), block);
        assert_phase5_reserved_under_v2(&toml, "fc.fdir.detector");
        assert_phase5_deferred_under_v3(&toml, "fc.fdir.detector", "Phase 5.B.4");
    }

    #[test]
    fn fc_v3_block_round_trips_unknown_field_rejection() {
        // serde(deny_unknown_fields) on the new sub-blocks must reject
        // typos. Lint is bypassed for $.fc, so the rejection comes from
        // serde, not the unit-suffix lint.
        let block = r#"
[fc.estimator_lanes]
voter   = "simplex_pass_through"
typo_id = "unknown"
"#;
        let toml =
            append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::ParseToml(_)), "got {err:?}");
    }

    #[test]
    fn fc_trajectory_block_is_v3_only() {
        let block = r#"
[fc.trajectory]
kind    = "minimum_snap"
yaw_rad = 0.0

[[fc.trajectory.waypoint]]
position_eci_m = [0.0, 0.0, 0.0]
time_s         = 0.0

[[fc.trajectory.waypoint]]
position_eci_m = [10.0, 0.0, 0.0]
time_s         = 4.0
"#;
        let toml_v2 = append(fc_v2_scenario(), block);
        let err = Scenario::from_toml_str(&toml_v2).unwrap_err();
        match err {
            ScenarioError::SchemaVersionFieldReserved { field, .. } => {
                assert_eq!(field, "fc.trajectory");
            }
            other => panic!("expected SchemaVersionFieldReserved, got {other:?}"),
        }
    }

    #[test]
    fn fc_trajectory_block_under_v3_requires_matching_trajectory_kind() {
        // The block parses but autopilot_params.trajectory_kind is left
        // at the default `pid`; the cross-check fires.
        let block = r#"
[fc.trajectory]
kind = "minimum_snap"

[[fc.trajectory.waypoint]]
position_eci_m = [0.0, 0.0, 0.0]
time_s         = 0.0

[[fc.trajectory.waypoint]]
position_eci_m = [10.0, 0.0, 0.0]
time_s         = 4.0
"#;
        let toml =
            append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::InconsistentSection {
                field_a, field_b, ..
            } => {
                assert_eq!(field_a, "fc.autopilot_params.trajectory_kind");
                assert_eq!(field_b, "fc.trajectory.kind");
            }
            other => panic!("expected InconsistentSection, got {other:?}"),
        }
    }

    #[test]
    fn fc_trajectory_block_under_v3_validates_with_kind_minimum_snap() {
        let block = r#"
[fc.trajectory]
kind    = "minimum_snap"
yaw_rad = 0.0

[[fc.trajectory.waypoint]]
position_eci_m = [0.0, 0.0, 0.0]
time_s         = 0.0

[[fc.trajectory.waypoint]]
position_eci_m = [10.0, 0.0, 0.0]
time_s         = 4.0
"#;
        let toml = append(fc_v2_scenario(), block)
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "trajectory_kind          = \"pid\"",
                "trajectory_kind          = \"minimum_snap\"",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("v3 fc.trajectory should validate");
        let trajectory = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.trajectory.as_ref())
            .expect("fc.trajectory present");
        assert_eq!(trajectory.waypoints.len(), 2);
    }

    #[test]
    fn fc_trajectory_minimum_snap_kind_requires_block() {
        let toml = fc_v2_scenario()
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "trajectory_kind          = \"pid\"",
                "trajectory_kind          = \"minimum_snap\"",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::MissingRequiredField { field, .. } => {
                assert_eq!(field, "fc.trajectory");
            }
            other => panic!("expected MissingRequiredField, got {other:?}"),
        }
    }

    #[test]
    fn fc_l1_adaptive_block_is_v3_only() {
        let block = r"
[fc.autopilot_params.l1_adaptive]
reference_model_a_m       = -10.0
reference_model_b         =  1.0
reference_model_k_g       = 10.0
adaptation_sample_time_s  =  0.001
low_pass_cutoff_rad_s     =  5.0
lipschitz_bound           =  0.1
projection_bound          =  100.0
";
        let toml_v2 = append(fc_v2_scenario(), block);
        let err = Scenario::from_toml_str(&toml_v2).unwrap_err();
        match err {
            ScenarioError::SchemaVersionFieldReserved { field, .. } => {
                assert_eq!(field, "fc.autopilot_params.l1_adaptive");
            }
            other => panic!("expected SchemaVersionFieldReserved, got {other:?}"),
        }
    }

    #[test]
    fn fc_l1_adaptive_block_under_v3_validates_with_nominal_params() {
        let block = r"
[fc.autopilot_params.l1_adaptive]
reference_model_a_m       = -10.0
reference_model_b         =  1.0
reference_model_k_g       = 10.0
adaptation_sample_time_s  =  0.001
low_pass_cutoff_rad_s     =  5.0
lipschitz_bound           =  0.1
projection_bound          =  100.0
";
        let toml =
            append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario = Scenario::from_toml_str(&toml).expect("v3 l1_adaptive validates");
        let l1 = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.autopilot_params.as_ref())
            .and_then(|p| p.l1_adaptive.as_ref())
            .expect("l1_adaptive present");
        assert!((l1.reference_model_a_m + 10.0).abs() < 1e-12);
    }

    #[test]
    fn fc_l1_adaptive_block_rejects_bandwidth_projection_violation() {
        // ω_c · L = 5 · 0.5 = 2.5 ≥ 1 — must fail Cao-Hovakimyan.
        let block = r"
[fc.autopilot_params.l1_adaptive]
reference_model_a_m       = -10.0
reference_model_b         =  1.0
reference_model_k_g       = 10.0
adaptation_sample_time_s  =  0.001
low_pass_cutoff_rad_s     =  5.0
lipschitz_bound           =  0.5
projection_bound          =  100.0
";
        let toml =
            append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, rule, .. }
                if field.contains("low_pass_cutoff_rad_s") && rule.contains("ω_c · L < 1")),
            "expected bandwidth-projection violation, got {err:?}"
        );
    }

    #[test]
    fn fc_l1_adaptive_block_rejects_unstable_reference_model() {
        // a_m = +1.0 ≥ 0 — reference model not stable.
        let block = r"
[fc.autopilot_params.l1_adaptive]
reference_model_a_m       =  1.0
reference_model_b         =  1.0
reference_model_k_g       = 10.0
adaptation_sample_time_s  =  0.001
low_pass_cutoff_rad_s     =  5.0
lipschitz_bound           =  0.1
projection_bound          =  100.0
";
        let toml =
            append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. }
                if field.contains("reference_model_a_m")),
            "expected reference_model_a_m InvalidNumber, got {err:?}"
        );
    }

    #[test]
    fn fc_l1_adaptive_block_rejects_non_positive_required_params() {
        let nominal = r"
[fc.autopilot_params.l1_adaptive]
reference_model_a_m       = -10.0
reference_model_b         =  1.0
reference_model_k_g       = 10.0
adaptation_sample_time_s  =  0.001
low_pass_cutoff_rad_s     =  5.0
lipschitz_bound           =  0.1
projection_bound          =  100.0
";
        let cases = [
            (
                "adaptation_sample_time_s",
                "adaptation_sample_time_s  =  0.001",
                "adaptation_sample_time_s  =  0.0",
            ),
            (
                "low_pass_cutoff_rad_s",
                "low_pass_cutoff_rad_s     =  5.0",
                "low_pass_cutoff_rad_s     =  0.0",
            ),
            (
                "lipschitz_bound",
                "lipschitz_bound           =  0.1",
                "lipschitz_bound           =  0.0",
            ),
            (
                "projection_bound",
                "projection_bound          =  100.0",
                "projection_bound          =  0.0",
            ),
        ];
        for (field_name, from, to) in cases {
            let block = nominal.replace(from, to);
            let toml = append(fc_v2_scenario(), &block)
                .replace("openbmp.scenario = 2", "openbmp.scenario = 3");
            let err = Scenario::from_toml_str(&toml).unwrap_err();
            assert!(
                matches!(err, ScenarioError::InvalidNumber { ref field, .. }
                    if field.contains(field_name)),
                "expected InvalidNumber for {field_name}, got {err:?}"
            );
        }
    }

    #[test]
    fn fc_l1_adaptive_block_rejects_euler_unstable_dt() {
        let block = r"
[fc.autopilot_params.l1_adaptive]
reference_model_a_m       = -10000.0
reference_model_b         =  1.0
reference_model_k_g       = 10000.0
adaptation_sample_time_s  =  0.001
low_pass_cutoff_rad_s     =  5.0
lipschitz_bound           =  0.1
projection_bound          =  100.0
";
        let toml =
            append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, rule, .. }
                if field == "time.dt_s" && rule.contains("forward-Euler L1 stability")),
            "expected time.dt_s InvalidNumber, got {err:?}"
        );
    }

    #[test]
    fn fc_l1_inspired_old_spelling_rejects_under_v2_and_v3() {
        let block = r"
[fc.autopilot_params.l1_inspired]
bandwidth_rad_s  = 5.0
projection_bound = 1.0
";
        for header in ["openbmp.scenario = 2", "openbmp.scenario = 3"] {
            let toml = append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", header);
            let err = Scenario::from_toml_str(&toml).unwrap_err();
            match err {
                ScenarioError::ParseToml(source) => {
                    let text = source.to_string();
                    assert!(
                        text.contains("l1_inspired") && text.contains("unknown field"),
                        "unexpected l1_inspired diagnostic: {text}"
                    );
                }
                other => panic!("expected ParseToml unknown-field error, got {other:?}"),
            }
        }
    }

    #[test]
    fn fc_anti_windup_block_is_v3_only() {
        let block = r#"
[fc.autopilot_params.anti_windup]
kind = "back_calculation"
gain = 2.0
"#;
        let toml_v2 = append(fc_v2_scenario(), block);
        let err = Scenario::from_toml_str(&toml_v2).unwrap_err();
        match err {
            ScenarioError::SchemaVersionFieldReserved { field, .. } => {
                assert_eq!(field, "fc.autopilot_params.anti_windup");
            }
            other => panic!("expected SchemaVersionFieldReserved, got {other:?}"),
        }
    }

    #[test]
    fn fc_anti_windup_back_calculation_validates_under_v3() {
        let block = r#"
[fc.autopilot_params.anti_windup]
kind = "back_calculation"
gain = 1.5
"#;
        let toml =
            append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario = Scenario::from_toml_str(&toml).expect("v3 anti_windup back_calculation");
        let aw = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.autopilot_params.as_ref())
            .and_then(|p| p.anti_windup.as_ref())
            .expect("anti_windup present");
        assert_eq!(*aw, FcAntiWindupConfig::BackCalculation { gain: 1.5 });
    }

    #[test]
    fn fc_anti_windup_observer_form_validates_under_v3() {
        let block = r#"
[fc.autopilot_params.anti_windup]
kind = "observer_form"
tracking_time_s = 0.05
"#;
        let toml =
            append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario = Scenario::from_toml_str(&toml).expect("v3 anti_windup observer_form");
        let aw = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.autopilot_params.as_ref())
            .and_then(|p| p.anti_windup.as_ref())
            .expect("anti_windup present");
        assert_eq!(
            *aw,
            FcAntiWindupConfig::ObserverForm {
                tracking_time_s: 0.05
            }
        );
    }

    #[test]
    fn fc_anti_windup_rejects_non_positive_parameters() {
        let cases = [
            (
                "back_calculation gain",
                r#"
[fc.autopilot_params.anti_windup]
kind = "back_calculation"
gain = 0.0
"#,
                "gain",
            ),
            (
                "back_calculation negative gain",
                r#"
[fc.autopilot_params.anti_windup]
kind = "back_calculation"
gain = -1.0
"#,
                "gain",
            ),
            (
                "observer_form tracking_time_s",
                r#"
[fc.autopilot_params.anti_windup]
kind = "observer_form"
tracking_time_s = 0.0
"#,
                "tracking_time_s",
            ),
        ];
        for (label, block, field_needle) in cases {
            let toml = append(fc_v2_scenario(), block)
                .replace("openbmp.scenario = 2", "openbmp.scenario = 3");
            let err = Scenario::from_toml_str(&toml).unwrap_err();
            assert!(
                matches!(err, ScenarioError::InvalidNumber { ref field, .. }
                    if field.contains(field_needle)),
                "{label}: expected InvalidNumber on field containing {field_needle}, got {err:?}"
            );
        }
    }

    /// Inject `rate_loop_kind = "lqr"` into the existing
    /// `[fc.autopilot_params]` table of the v2 scenario fixture by
    /// appending the field to the end of the block (TOML allows
    /// duplicate keys to be added by inline-extension as long as they
    /// don't collide with each other). The optional `lqr_block`
    /// argument is appended as a separate sub-table.
    fn fc_v3_with_lqr(rate_loop_kind: Option<&str>, lqr_block: Option<&str>) -> String {
        let mut text = fc_v2_scenario().replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        if let Some(kind) = rate_loop_kind {
            text = text.replace(
                "trajectory_kind          = \"pid\"\n",
                &format!(
                    "trajectory_kind          = \"pid\"\nrate_loop_kind           = \"{kind}\"\n"
                ),
            );
        }
        if let Some(block) = lqr_block {
            text.push('\n');
            text.push_str(block);
        }
        text
    }

    #[test]
    fn fc_rate_loop_lqr_block_is_v3_only() {
        let block = r"
[fc.autopilot_params.lqr]
q_omega = [1.0, 1.0, 1.0]
q_int   = [0.1, 0.1, 0.1]
r       = [0.1, 0.1, 0.1]
";
        let toml_v2 = append(fc_v2_scenario(), block);
        let err = Scenario::from_toml_str(&toml_v2).unwrap_err();
        match err {
            ScenarioError::SchemaVersionFieldReserved { field, .. } => {
                assert!(
                    field == "fc.autopilot_params.rate_loop_kind"
                        || field == "fc.autopilot_params.lqr",
                    "unexpected SchemaVersionFieldReserved field: {field}"
                );
            }
            other => panic!("expected SchemaVersionFieldReserved, got {other:?}"),
        }
    }

    #[test]
    fn fc_rate_loop_lqr_block_under_v3_validates() {
        let lqr_block = r"
[fc.autopilot_params.lqr]
q_omega = [10.0, 10.0, 10.0]
q_int   = [0.5, 0.5, 0.5]
r       = [0.1, 0.1, 0.1]
";
        let toml = fc_v3_with_lqr(Some("lqr"), Some(lqr_block));
        let scenario = Scenario::from_toml_str(&toml).expect("v3 LQR validates");
        let params = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.autopilot_params.as_ref())
            .expect("autopilot_params present");
        assert_eq!(params.rate_loop_kind, Some(FcRateLoopKind::Lqr));
        let lqr = params.lqr.as_ref().expect("lqr block present");
        assert_eq!(
            *lqr,
            FcLqrConfig {
                q_omega: [10.0, 10.0, 10.0],
                q_int: [0.5, 0.5, 0.5],
                r: [0.1, 0.1, 0.1],
            }
        );
    }

    #[test]
    fn fc_rate_loop_lqr_without_lqr_block_fails_closed() {
        let toml = fc_v3_with_lqr(Some("lqr"), None);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. }
                if field == "fc.autopilot_params.lqr"),
            "expected MissingRequiredField for fc.autopilot_params.lqr, got {err:?}"
        );
    }

    #[test]
    fn fc_lqr_block_without_rate_loop_lqr_kind_fails_closed() {
        let lqr_block = r"
[fc.autopilot_params.lqr]
q_omega = [1.0, 1.0, 1.0]
q_int   = [0.1, 0.1, 0.1]
r       = [0.1, 0.1, 0.1]
";
        let toml = fc_v3_with_lqr(None, Some(lqr_block));
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, .. }
                if field_a == "fc.autopilot_params.lqr"),
            "expected InconsistentSection on fc.autopilot_params.lqr, got {err:?}"
        );
    }

    #[test]
    fn fc_lqr_block_rejects_non_positive_weights() {
        let cases = [
            (
                "q_omega zero on roll",
                "q_omega = [10.0, 10.0, 10.0]",
                "q_omega = [0.0, 10.0, 10.0]",
            ),
            (
                "q_int negative on pitch",
                "q_int   = [0.5, 0.5, 0.5]",
                "q_int   = [0.5, -0.1, 0.5]",
            ),
            (
                "r zero on yaw",
                "r       = [0.1, 0.1, 0.1]",
                "r       = [0.1, 0.1, 0.0]",
            ),
        ];
        let nominal = r"
[fc.autopilot_params.lqr]
q_omega = [10.0, 10.0, 10.0]
q_int   = [0.5, 0.5, 0.5]
r       = [0.1, 0.1, 0.1]
";
        for (label, from, to) in cases {
            let block = nominal.replace(from, to);
            let toml = fc_v3_with_lqr(Some("lqr"), Some(&block));
            let err = Scenario::from_toml_str(&toml).unwrap_err();
            assert!(
                matches!(err, ScenarioError::InvalidNumber { .. }),
                "{label}: expected InvalidNumber, got {err:?}"
            );
        }
    }

    /// Inject `rate_loop_kind = "<kind>"` into the existing
    /// `[fc.autopilot_params]` table of the v2 fixture and append the
    /// matching `[fc.autopilot_params.indi]` sub-table.
    fn fc_v3_with_indi(rate_loop_kind: Option<&str>, indi_block: Option<&str>) -> String {
        let mut text = fc_v2_scenario().replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        if let Some(kind) = rate_loop_kind {
            text = text.replace(
                "trajectory_kind          = \"pid\"\n",
                &format!(
                    "trajectory_kind          = \"pid\"\nrate_loop_kind           = \"{kind}\"\n"
                ),
            );
        }
        if let Some(block) = indi_block {
            text.push('\n');
            text.push_str(block);
        }
        text
    }

    #[test]
    fn fc_indi_block_is_v3_only() {
        let block = r"
[fc.autopilot_params.indi]
inertia_per_axis_kg_m2          = [1.0, 1.0, 1.0]
control_effectiveness_per_axis  = [1.0, 1.0, 1.0]
filter_cutoff_rad_s             = 50.0
attitude_to_omega_dot_gain      = [10.0, 10.0, 5.0]
";
        let toml_v2 = append(fc_v2_scenario(), block);
        let err = Scenario::from_toml_str(&toml_v2).unwrap_err();
        match err {
            ScenarioError::SchemaVersionFieldReserved { field, .. } => {
                assert!(
                    field == "fc.autopilot_params.rate_loop_kind"
                        || field == "fc.autopilot_params.indi",
                    "unexpected SchemaVersionFieldReserved field: {field}"
                );
            }
            other => panic!("expected SchemaVersionFieldReserved, got {other:?}"),
        }
    }

    #[test]
    fn fc_indi_block_under_v3_validates_with_default_filter_kind() {
        let indi_block = r"
[fc.autopilot_params.indi]
inertia_per_axis_kg_m2          = [1.0, 1.0, 1.0]
control_effectiveness_per_axis  = [1.0, 1.0, 1.0]
filter_cutoff_rad_s             = 50.0
attitude_to_omega_dot_gain      = [10.0, 10.0, 5.0]
";
        let toml = fc_v3_with_indi(Some("indi"), Some(indi_block));
        let scenario = Scenario::from_toml_str(&toml).expect("v3 INDI validates");
        let params = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.autopilot_params.as_ref())
            .expect("autopilot_params present");
        assert_eq!(params.rate_loop_kind, Some(FcRateLoopKind::Indi));
        let indi = params.indi.as_ref().expect("indi block present");
        assert_eq!(indi.filter_kind, FcIndiFilterKind::SecondOrderButterworth);
        assert_eq!(
            *indi,
            FcIndiConfig {
                inertia_per_axis_kg_m2: [1.0, 1.0, 1.0],
                control_effectiveness_per_axis: [1.0, 1.0, 1.0],
                filter_cutoff_rad_s: 50.0,
                filter_kind: FcIndiFilterKind::SecondOrderButterworth,
                attitude_to_omega_dot_gain: [10.0, 10.0, 5.0],
            }
        );
    }

    #[test]
    fn fc_indi_block_under_v3_accepts_first_order_filter_kind() {
        let indi_block = "
[fc.autopilot_params.indi]
inertia_per_axis_kg_m2          = [1.0, 1.0, 1.0]
control_effectiveness_per_axis  = [1.0, 1.0, 1.0]
filter_cutoff_rad_s             = 80.0
filter_kind                     = \"first_order_low_pass\"
attitude_to_omega_dot_gain      = [10.0, 10.0, 5.0]
";
        let toml = fc_v3_with_indi(Some("indi"), Some(indi_block));
        let scenario = Scenario::from_toml_str(&toml).expect("v3 INDI validates");
        let params = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.autopilot_params.as_ref())
            .expect("autopilot_params present");
        let indi = params.indi.as_ref().expect("indi block present");
        assert_eq!(indi.filter_kind, FcIndiFilterKind::FirstOrderLowPass);
    }

    #[test]
    fn fc_rate_loop_indi_without_indi_block_fails_closed() {
        let toml = fc_v3_with_indi(Some("indi"), None);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. }
                if field == "fc.autopilot_params.indi"),
            "expected MissingRequiredField for fc.autopilot_params.indi, got {err:?}"
        );
    }

    #[test]
    fn fc_indi_block_without_rate_loop_indi_kind_fails_closed() {
        let indi_block = r"
[fc.autopilot_params.indi]
inertia_per_axis_kg_m2          = [1.0, 1.0, 1.0]
control_effectiveness_per_axis  = [1.0, 1.0, 1.0]
filter_cutoff_rad_s             = 50.0
attitude_to_omega_dot_gain      = [10.0, 10.0, 5.0]
";
        let toml = fc_v3_with_indi(None, Some(indi_block));
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, .. }
                if field_a == "fc.autopilot_params.indi"),
            "expected InconsistentSection on fc.autopilot_params.indi, got {err:?}"
        );
    }

    #[test]
    fn fc_indi_block_rejects_non_positive_inertia_or_filter_cutoff() {
        let cases = [
            (
                "inertia zero on pitch",
                "inertia_per_axis_kg_m2          = [1.0, 1.0, 1.0]",
                "inertia_per_axis_kg_m2          = [1.0, 0.0, 1.0]",
            ),
            (
                "control effectiveness negative on yaw",
                "control_effectiveness_per_axis  = [1.0, 1.0, 1.0]",
                "control_effectiveness_per_axis  = [1.0, 1.0, -1.0]",
            ),
            (
                "filter cutoff at Nyquist",
                "filter_cutoff_rad_s             = 50.0",
                "filter_cutoff_rad_s             = 6000.0",
            ),
            (
                "attitude gain zero on roll",
                "attitude_to_omega_dot_gain      = [10.0, 10.0, 5.0]",
                "attitude_to_omega_dot_gain      = [0.0, 10.0, 5.0]",
            ),
        ];
        let nominal = r"
[fc.autopilot_params.indi]
inertia_per_axis_kg_m2          = [1.0, 1.0, 1.0]
control_effectiveness_per_axis  = [1.0, 1.0, 1.0]
filter_cutoff_rad_s             = 50.0
attitude_to_omega_dot_gain      = [10.0, 10.0, 5.0]
";
        for (label, from, to) in cases {
            let block = nominal.replace(from, to);
            let toml = fc_v3_with_indi(Some("indi"), Some(&block));
            let err = Scenario::from_toml_str(&toml).unwrap_err();
            assert!(
                matches!(err, ScenarioError::InvalidNumber { .. }),
                "{label}: expected InvalidNumber, got {err:?}"
            );
        }
    }

    #[test]
    fn fc_indi_with_l1_adaptive_block_fails_closed() {
        // INDI + L1 composition is rejected at scenario load.
        let combined = r"
[fc.autopilot_params.indi]
inertia_per_axis_kg_m2          = [1.0, 1.0, 1.0]
control_effectiveness_per_axis  = [1.0, 1.0, 1.0]
filter_cutoff_rad_s             = 50.0
attitude_to_omega_dot_gain      = [10.0, 10.0, 5.0]

[fc.autopilot_params.l1_adaptive]
reference_model_a_m       = -10.0
reference_model_b         =  1.0
reference_model_k_g       = 10.0
adaptation_sample_time_s  =  0.001
low_pass_cutoff_rad_s     =  5.0
lipschitz_bound           =  0.1
projection_bound          =  100.0
";
        let toml = fc_v3_with_indi(Some("indi"), Some(combined));
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, ref field_b, .. }
                if field_a == "fc.autopilot_params.rate_loop_kind"
                && field_b == "fc.autopilot_params.l1_adaptive"),
            "expected InconsistentSection rate_loop_kind ↔ l1_adaptive, got {err:?}"
        );
    }

    #[test]
    fn fc_trajectory_block_rejects_too_few_waypoints() {
        let block = r#"
[fc.trajectory]
kind = "minimum_snap"

[[fc.trajectory.waypoint]]
position_eci_m = [0.0, 0.0, 0.0]
time_s         = 0.0
"#;
        let toml = append(fc_v2_scenario(), block)
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "trajectory_kind          = \"pid\"",
                "trajectory_kind          = \"minimum_snap\"",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidFc { .. }),
            "got {err:?}"
        );
    }

    #[test]
    fn fc_trajectory_block_rejects_too_short_segment_duration() {
        let block = r#"
[fc.trajectory]
kind = "minimum_snap"

[[fc.trajectory.waypoint]]
position_eci_m = [0.0, 0.0, 0.0]
time_s         = 0.0

[[fc.trajectory.waypoint]]
position_eci_m = [10.0, 0.0, 0.0]
time_s         = 0.0005
"#;
        let toml = append(fc_v2_scenario(), block)
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "trajectory_kind          = \"pid\"",
                "trajectory_kind          = \"minimum_snap\"",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::InvalidFc { reason } => {
                assert!(reason.contains("segment 0 duration"), "got {reason}");
            }
            other => panic!("expected InvalidFc, got {other:?}"),
        }
    }

    #[test]
    fn nrlmsise00_environment_atmosphere_is_v3_only() {
        // The constant-acceleration-drop scenario uses
        // `atmosphere    = "none"`; flipping to nrlmsise00 exercises
        // the v3 gate. Aligned-equals layout matched verbatim.
        let toml = MINIMAL.replace("atmosphere    = \"none\"", "atmosphere    = \"nrlmsise00\"");
        assert_phase5_reserved_under_v2(&toml, "atmosphere.kind = \"nrlmsise00\"");
        assert_phase5_deferred_under_v3(&toml, "atmosphere.kind = \"nrlmsise00\"", "Phase 5.C.1");
    }

    const NRLMSISE00_STRUCTURED_ATMOSPHERE_BLOCK: &str = r#"
[atmosphere]
kind = "nrlmsise00"
"#;

    #[test]
    fn nrlmsise00_structured_atmosphere_kind_is_v3_only() {
        let toml = append(MINIMAL, NRLMSISE00_STRUCTURED_ATMOSPHERE_BLOCK);
        assert_phase5_reserved_under_v2(&toml, "atmosphere.kind = \"nrlmsise00\"");
        assert_phase5_deferred_under_v3(&toml, "atmosphere.kind = \"nrlmsise00\"", "Phase 5.C.1");
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
        let toml = MINIMAL.replace("dry_mass_kg = 1.0", "dry_mass_kg = 1.0\nextra_kg = 2.0");
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
        let toml = MINIMAL.replace("dry_mass_kg = 1.0", "dry_mass = 1.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::MissingUnitSuffix { .. }));
    }

    #[test]
    fn rejects_effector_dimensionless_keys_outside_effector_paths() {
        // The `min = 1.0` field is allowed inside
        // `$.vehicle.assembly.effectors` but must not pass anywhere
        // else. The `[faults.example]` extension table is open-typed
        // (BTreeMap<String, toml::Value>) and exercises the lint
        // before serde deserialisation.
        let toml = format!("{MINIMAL}\n[faults.example]\nmin = 1.0\n");
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
    fn wind_axis_vector_frame_exemption_is_scoped_to_wind_block() {
        let toml = format!("{MINIMAL}\n[wind_extra]\nintensity_m_s = [1.0, 1.0, 1.0]\n");
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

    // Phase-2.10 sounding-rocket-shaped fixture. Loaded from a sibling
    // file so the four-pillar provenance contract is unambiguous: the
    // file lives under `tests/fixtures/`, marking it as a synthetic
    // parser test artefact, not a benchmark scenario. The constants
    // inside are intentionally synthetic round numbers — see the
    // CI tripwire in
    // `crates/openbmp-testkit/tests/inline_data_tripwire.rs` and the
    // "Inline Data Tripwires" section of `docs/data-provenance.md`.
    //
    // Real validation cases live under `scenarios/sounding-rocket/...`
    // (e.g., `niskanen-2009-chapter6.toml`); reference physical
    // constants live under `data/<category>/...` with provenance.
    const SOUNDING_ROCKET: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/sounding-rocket.toml"
    ));

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
            "initial_quaternion_body_to_eci_xyzw  = [0.0, 0.0, 0.0, 1.0]\n",
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
        let toml = SOUNDING_ROCKET.replace(
            r#"kind                                 = "rigid_body""#,
            r#"kind                                 = "point_mass""#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "vehicle.initial_quaternion_body_to_eci_xyzw"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_non_unit_quaternion() {
        let toml = SOUNDING_ROCKET.replace(
            "initial_quaternion_body_to_eci_xyzw  = [0.0, 0.0, 0.0, 1.0]",
            "initial_quaternion_body_to_eci_xyzw  = [1.0, 0.0, 0.0, 1.0]",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field == "vehicle.initial_quaternion_body_to_eci_xyzw"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_latitude_out_of_range() {
        let toml = SOUNDING_ROCKET.replace("latitude_deg  = 0.0", "latitude_deg  = 91.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field == "frames.local_origin.latitude_deg"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_atmosphere_kind_disagreement() {
        let toml = SOUNDING_ROCKET.replace(
            r#"atmosphere    = "us_standard_1976""#,
            r#"atmosphere    = "isothermal""#,
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
            "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]",
            "[wind]\nkind         = \"none\"",
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
    fn rejects_flat_gust_wind_without_structured_block() {
        let toml = SOUNDING_ROCKET
            .replace(r#"wind          = "constant""#, r#"wind          = "gust""#)
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, ref name, .. } if field == "wind" && name == "gust"),
            "got {err:?}",
        );
    }

    // -----------------------------------------------------------------
    // Phase 3.8.B layered wind
    // -----------------------------------------------------------------

    #[test]
    fn accepts_layered_wind_with_well_formed_table() {
        let toml = SOUNDING_ROCKET
            .replace(r#"wind          = "constant""#, r#"wind          = "layered""#)
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"layered\"\nlayers = [\n  { altitude_m = 0.0, wind_ned_m_s = [5.0, 0.0, 0.0] },\n  { altitude_m = 3000.0, wind_ned_m_s = [12.0, 2.0, 0.0] },\n]\n",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("layered wind scenario must parse");
        let layers = scenario
            .document
            .wind
            .as_ref()
            .and_then(|w| w.layers.as_ref())
            .expect("layered wind must populate layers");
        assert_eq!(layers.len(), 2);
        assert_eq!(layers[0].altitude_m.to_bits(), 0.0_f64.to_bits());
        assert_eq!(layers[1].altitude_m.to_bits(), 3000.0_f64.to_bits());
    }

    #[test]
    fn rejects_layered_wind_without_layers() {
        let toml = SOUNDING_ROCKET
            .replace(
                r#"wind          = "constant""#,
                r#"wind          = "layered""#,
            )
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"layered\"\n",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "wind.layers"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_layered_wind_with_descending_altitudes() {
        let toml = SOUNDING_ROCKET
            .replace(r#"wind          = "constant""#, r#"wind          = "layered""#)
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"layered\"\nlayers = [\n  { altitude_m = 1000.0, wind_ned_m_s = [5.0, 0.0, 0.0] },\n  { altitude_m = 500.0, wind_ned_m_s = [10.0, 0.0, 0.0] },\n]\n",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field == "wind.layers[*].altitude_m"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_layered_wind_with_constant_field() {
        let toml = SOUNDING_ROCKET
            .replace(r#"wind          = "constant""#, r#"wind          = "layered""#)
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"layered\"\nwind_ned_m_s = [5.0, 0.0, 0.0]\nlayers = [\n  { altitude_m = 0.0, wind_ned_m_s = [5.0, 0.0, 0.0] },\n]\n",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "wind.wind_ned_m_s"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_constant_wind_with_layers_field() {
        let toml = SOUNDING_ROCKET.replace(
            "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
            "[wind]\nkind = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\nlayers = [\n  { altitude_m = 0.0, wind_ned_m_s = [5.0, 0.0, 0.0] },\n]\n",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "wind.layers"),
            "got {err:?}",
        );
    }

    // -----------------------------------------------------------------
    // Phase 3.8.C gust wind
    // -----------------------------------------------------------------

    fn gust_wind_block() -> &'static str {
        "[wind]\nkind = \"gust\"\nintensity_m_s = [2.5, 2.0, 1.5]\nlength_scale_m = [533.0, 533.0, 100.0]\nairspeed_m_s = 250.0\n"
    }

    #[test]
    fn accepts_gust_wind_with_well_formed_block() {
        let toml = SOUNDING_ROCKET
            .replace(r#"wind          = "constant""#, r#"wind          = "gust""#)
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                gust_wind_block(),
            );
        let scenario = Scenario::from_toml_str(&toml).expect("gust wind scenario must parse");
        let wind = scenario.document.wind.as_ref().expect("wind block present");
        assert_eq!(wind.kind, "gust");
        assert_eq!(wind.intensity_m_s.unwrap()[0].to_bits(), 2.5_f64.to_bits());
        assert_eq!(
            wind.length_scale_m.unwrap()[2].to_bits(),
            100.0_f64.to_bits()
        );
        assert_eq!(wind.airspeed_m_s.unwrap().to_bits(), 250.0_f64.to_bits());
    }

    #[test]
    fn rejects_gust_wind_without_intensity() {
        let toml = SOUNDING_ROCKET
            .replace(r#"wind          = "constant""#, r#"wind          = "gust""#)
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"gust\"\nlength_scale_m = [533.0, 533.0, 100.0]\nairspeed_m_s = 250.0\n",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "wind.intensity_m_s"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_gust_wind_with_negative_sigma() {
        let toml = SOUNDING_ROCKET
            .replace(r#"wind          = "constant""#, r#"wind          = "gust""#)
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"gust\"\nintensity_m_s = [-1.0, 2.0, 1.5]\nlength_scale_m = [533.0, 533.0, 100.0]\nairspeed_m_s = 250.0\n",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field == "wind.intensity_m_s[u]"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_gust_wind_with_zero_length_scale() {
        let toml = SOUNDING_ROCKET
            .replace(r#"wind          = "constant""#, r#"wind          = "gust""#)
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"gust\"\nintensity_m_s = [2.5, 2.0, 1.5]\nlength_scale_m = [0.0, 533.0, 100.0]\nairspeed_m_s = 250.0\n",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field == "wind.length_scale_m[u]"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_constant_wind_with_gust_field() {
        let toml = SOUNDING_ROCKET.replace(
            "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
            "[wind]\nkind = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\nintensity_m_s = [1.0, 1.0, 1.0]\n",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "wind.intensity_m_s"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_isothermal_atmosphere_without_state() {
        let toml = SOUNDING_ROCKET
            .replace(
                r#"atmosphere    = "us_standard_1976""#,
                r#"atmosphere    = "isothermal""#,
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
            "gravity       = \"j2\"\nmu_m3_s2      = 4.0e14\nr_e_m         = 6_400_000.0\nj2            = 1.0e-3",
            "gravity       = \"constant\"\ngravity_m_s2  = 9.80665\nmu_m3_s2      = 4.0e14",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "environment.mu_m3_s2"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_j2_gravity_without_r_e() {
        let toml = SOUNDING_ROCKET.replace("r_e_m         = 6_400_000.0\n", "");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "environment.r_e_m"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_point_mass_gravity_with_j2_coefficients() {
        let toml =
            SOUNDING_ROCKET.replace("gravity       = \"j2\"", "gravity       = \"point_mass\"");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "environment.r_e_m / environment.j2"),
            "got {err:?}",
        );
    }

    #[test]
    fn j2_gravity_uses_wgs84_default_when_j2_is_omitted() {
        let toml = SOUNDING_ROCKET.replace("j2            = 1.0e-3\n", "");
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
        let toml = SOUNDING_ROCKET.replace(r#"variant     = "solid""#, r#"variant     = "liquid""#);
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
    fn rejects_thrust_force_without_motor_block_or_engine_cluster() {
        let toml = MINIMAL.replace(
            r#"models = ["gravity"]"#,
            r#"models = ["gravity", "thrust"]"#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "propulsion.motor or vehicle.assembly.engines"),
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
        assert_eq!(scenario.document.openbmp.scenario, 2);
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

    // -----------------------------------------------------------------
    // Phase 3.2: `[mission]` block parser tests
    // -----------------------------------------------------------------

    /// Append a `[mission]` block to the analytic-toy MINIMAL scenario.
    fn with_mission(mission_block: &str) -> String {
        format!("{MINIMAL}\n{mission_block}")
    }

    const MISSION_ONCE_FALSE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/mission-once-false.toml"
    ));
    const MISSION_DUPLICATE_EVENT: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/mission-duplicate-event.toml"
    ));
    const MISSION_UNKNOWN_TRANSITION_PHASE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/mission-unknown-transition-phase.toml"
    ));
    const MISSION_CYCLIC_GRAPH: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/mission-cyclic-graph.toml"
    ));

    #[test]
    fn parses_minimal_mission_block() {
        let scenario = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "pre_launch"

[[mission.phases]]
id    = "pre_launch"
label = "pre-launch hold"

[[mission.phases]]
id    = "ascent"
label = "powered + coast ascent"

[[mission.events]]
id      = "ignition"
trigger = { kind = "at_time", time_s = 0.0 }
action  = { kind = "enter_phase", phase = "ascent" }

[[mission.transitions]]
from  = "pre_launch"
to    = "ascent"
event = "ignition"
"#,
        ))
        .expect("parse");

        let mission = scenario
            .document
            .mission
            .as_ref()
            .expect("mission block parsed");
        assert_eq!(mission.initial_phase, "pre_launch");
        assert_eq!(mission.phases.len(), 2);
        assert_eq!(mission.events.len(), 1);
        assert_eq!(mission.transitions.len(), 1);
        // `once` defaults to true.
        assert!(mission.events[0].once);
    }

    #[test]
    fn parses_mission_event_once_false() {
        let scenario = Scenario::from_toml_str(&with_mission(MISSION_ONCE_FALSE)).expect("parse");
        let mission = scenario.document.mission.as_ref().expect("mission");
        assert!(!mission.events[0].once);
    }

    #[test]
    fn rejects_unknown_field_in_mission_phase() {
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"
unknown_extra_field = true
"#,
        ))
        .unwrap_err();
        assert!(
            matches!(err, ScenarioError::ParseToml(_)),
            "expected ParseToml on deny_unknown_fields, got {err:?}",
        );
    }

    #[test]
    fn rejects_scripted_trigger_kind() {
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "scripted" }
action  = { kind = "stop", label = "scripted-stop" }
"#,
        ))
        .unwrap_err();
        match err {
            ScenarioError::UnsupportedTriggerKind { kind, .. } => {
                assert_eq!(kind, "scripted");
            }
            other => panic!("expected UnsupportedTriggerKind, got {other:?}"),
        }
    }

    #[test]
    fn accepts_engine_command_action_with_typed_payload() {
        let scenario = Scenario::from_toml_str(ASSEMBLY_ENGINE_CLUSTER_WITH_ENGINE_COMMAND)
            .expect("scenario parses");
        let mission = scenario.document.mission.as_ref().expect("mission present");
        assert_eq!(mission.events.len(), 1);
        match &mission.events[0].action {
            crate::EventActionConfig::EngineCommand { id, command } => {
                assert_eq!(id, "engine_a");
                assert!((command.throttle_unit - 0.5).abs() < 1e-12);
                assert!(command.ignite);
                assert!(!command.shutdown);
            }
            other => panic!("expected EngineCommand action, got {other:?}"),
        }
    }

    #[test]
    fn rejects_engine_command_action_referencing_unknown_engine_id() {
        let err =
            Scenario::from_toml_str(ASSEMBLY_ENGINE_CLUSTER_WITH_UNKNOWN_ENGINE_ID).unwrap_err();
        assert!(
            matches!(
                err,
                ScenarioError::UnknownEngineReference { ref id, .. } if id == "engine_typo"
            ),
            "expected UnknownEngineReference, got {err:?}",
        );
    }

    #[test]
    fn rejects_engine_command_with_throttle_above_one() {
        let err =
            Scenario::from_toml_str(ASSEMBLY_ENGINE_CLUSTER_WITH_THROTTLE_TOO_HIGH).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { .. }),
            "expected InvalidNumber, got {err:?}",
        );
    }

    #[test]
    fn rejects_scenario_with_both_motor_and_engines_blocks() {
        let err = Scenario::from_toml_str(ASSEMBLY_ENGINE_CLUSTER_AND_MOTOR).unwrap_err();
        assert!(
            matches!(err, ScenarioError::AmbiguousPropulsion),
            "expected AmbiguousPropulsion, got {err:?}",
        );
    }

    const SLOSHING_TANK: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/sloshing-tank/sloshing-tank.toml"
    ));

    #[test]
    fn parses_sloshing_tank_scenario() {
        let scenario = Scenario::from_toml_str(SLOSHING_TANK).expect("sloshing tank parses");
        let tanks = &&scenario.document.vehicle.assembly.tanks;
        assert_eq!(tanks.len(), 1);
        assert!(
            tanks[0]
                .initial_slosh
                .as_ref()
                .unwrap()
                .angles_rad
                .is_some()
        );
    }

    #[test]
    fn rejects_initial_slosh_on_rigid_liquid() {
        let toml = SLOSHING_TANK.replace(
            r#"moving_mass              = { kind = "equivalent_pendulum", damping_ratio_zeta = 0.005 }"#,
            r#"moving_mass              = { kind = "rigid_liquid" }"#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::IncompatibleAssemblyEntry { ref field, .. } if field == "vehicle.assembly.tanks[0].initial_slosh"),
            "expected IncompatibleAssemblyEntry, got {err:?}",
        );
    }

    #[test]
    fn rejects_baffle_model_without_baffled_pendulum() {
        let toml = SLOSHING_TANK.replace(
            "drain_rate_kg_per_s      = 0.0",
            "baffle_model             = { damping_increment_zeta = 0.02 }\ndrain_rate_kg_per_s      = 0.0",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::IncompatibleAssemblyEntry { ref field, .. } if field == "vehicle.assembly.tanks[0].baffle_model"),
            "expected IncompatibleAssemblyEntry, got {err:?}",
        );
    }

    #[test]
    fn rejects_tank_drain_that_empties_in_one_step() {
        let toml = SLOSHING_TANK.replace(
            "drain_rate_kg_per_s      = 0.0",
            "drain_rate_kg_per_s      = 1.0e9",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field == "vehicle.assembly.tanks[0].drain_rate_kg_per_s"),
            "expected InvalidNumber, got {err:?}",
        );
    }

    #[test]
    fn rejects_spring_mass_initial_slosh_with_angle_fields() {
        let toml = SLOSHING_TANK.replace(
            r#"moving_mass              = { kind = "equivalent_pendulum", damping_ratio_zeta = 0.005 }"#,
            r#"moving_mass              = { kind = "equivalent_spring_mass", damping_ratio_zeta = 0.005 }"#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "vehicle.assembly.tanks[0].initial_slosh.displacement_body_m"),
            "expected MissingRequiredField, got {err:?}",
        );
    }

    #[test]
    fn accepts_spring_mass_initial_slosh_with_linear_fields() {
        let toml = SLOSHING_TANK
            .replace(
                r#"moving_mass              = { kind = "equivalent_pendulum", damping_ratio_zeta = 0.005 }"#,
                r#"moving_mass              = { kind = "equivalent_spring_mass", damping_ratio_zeta = 0.005 }"#,
            )
            .replace(
                "initial_slosh            = { angles_rad = [0.05, 0.0], rates_rad_s = [0.0, 0.0] }",
                "initial_slosh            = { displacement_body_m = [0.05, 0.0], velocity_body_m_s = [0.0, 0.0] }",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("spring-mass tank parses");
        let initial = &scenario.document.vehicle.assembly.tanks[0]
            .initial_slosh
            .as_ref()
            .unwrap();
        assert!(initial.displacement_body_m.is_some());
        assert!(initial.velocity_body_m_s.is_some());
    }

    #[test]
    fn rejects_separation_action_kind() {
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "at_apogee" }
action  = { kind = "separation" }
"#,
        ))
        .unwrap_err();
        assert!(matches!(
            err,
            ScenarioError::UnsupportedActionKind { ref kind, .. } if kind == "separation"
        ));
    }

    #[test]
    fn rejects_deploy_recovery_action_missing_fields() {
        // Phase-3.9: `deploy_recovery` requires `id` and `command`
        // fields. A bare `{ kind = "deploy_recovery" }` is a serde
        // decode failure (missing fields).
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "at_apogee" }
action  = { kind = "deploy_recovery" }
"#,
        ))
        .unwrap_err();
        assert!(matches!(err, ScenarioError::ParseToml(_)));
    }

    #[test]
    fn rejects_deploy_recovery_with_unknown_command_name() {
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "at_apogee" }
action  = { kind = "deploy_recovery", id = "main", command = "unfurl" }
"#,
        ))
        .unwrap_err();
        assert!(matches!(
            err,
            ScenarioError::UnsupportedValue { ref field, ref value }
                if field.contains("command") && value == "unfurl"
        ));
    }

    #[test]
    fn accepts_effector_override_action_kind() {
        // Phase 3.4 wires `EventAction::EffectorOverride { id, command }`.
        let parse_result = Scenario::from_toml_str(&format!(
            "{ASSEMBLY_WITH_EFFECTOR}\n{}",
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "at_apogee" }
action  = { kind = "effector_override", id = "delta_e", command = 0.087 }
"#,
        ));
        let scenario = match parse_result {
            Ok(s) => s,
            Err(e) => panic!("parse with EffectorOverride action failed: {e:?}"),
        };
        let mission = scenario.document.mission.as_ref().expect("mission present");
        let action = &mission.events[0].action;
        assert!(matches!(
            action,
            crate::EventActionConfig::EffectorOverride { id, command }
                if id == "delta_e" && (command - 0.087).abs() < 1e-12
        ));
    }

    #[test]
    fn rejects_effector_override_action_with_empty_id() {
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "at_apogee" }
action  = { kind = "effector_override", id = "", command = 0.0 }
"#,
        ))
        .unwrap_err();
        assert!(matches!(err, ScenarioError::EmptyField { .. }));
    }

    #[test]
    fn rejects_effector_override_action_with_unknown_id() {
        let err = Scenario::from_toml_str(&format!(
            "{ASSEMBLY_WITH_EFFECTOR}\n{}",
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "at_apogee" }
action  = { kind = "effector_override", id = "delta_typo", command = 0.087 }
"#,
        ))
        .unwrap_err();
        assert!(matches!(
            err,
            ScenarioError::UnknownEffectorReference { ref field, ref id }
                if field.contains("mission.events") && id == "delta_typo"
        ));
    }

    #[test]
    fn rejects_empty_phase_list() {
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"
"#,
        ))
        .unwrap_err();
        assert!(matches!(err, ScenarioError::EmptyList { .. }));
    }

    #[test]
    fn rejects_mass_fraction_out_of_range() {
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "at_mass_fraction", remaining = 1.5 }
action  = { kind = "stop", label = "burnout" }
"#,
        ))
        .unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { .. }),
            "expected InvalidNumber, got {err:?}",
        );
    }

    #[test]
    fn rejects_dynamic_pressure_trigger_kind() {
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "at_dynamic_pressure", pressure_pa = 0.0, falling = false }
action  = { kind = "stop", label = "max-q" }
"#,
        ))
        .unwrap_err();
        assert!(matches!(
            err,
            ScenarioError::UnsupportedTriggerKind { ref kind, .. }
                if kind == "at_dynamic_pressure"
        ));
    }

    #[test]
    fn rejects_duplicate_mission_event_ids() {
        let err = Scenario::from_toml_str(&with_mission(MISSION_DUPLICATE_EVENT)).unwrap_err();
        assert!(matches!(
            err,
            ScenarioError::DuplicateValue { ref field, ref value }
                if field == "mission.events.id" && value == "evt"
        ));
    }

    #[test]
    fn rejects_unknown_transition_phase_at_parse_time() {
        let err =
            Scenario::from_toml_str(&with_mission(MISSION_UNKNOWN_TRANSITION_PHASE)).unwrap_err();
        assert!(matches!(err, ScenarioError::MissionGraph { .. }));
    }

    #[test]
    fn rejects_cyclic_mission_graph_at_parse_time() {
        let err = Scenario::from_toml_str(&with_mission(MISSION_CYCLIC_GRAPH)).unwrap_err();
        assert!(matches!(err, ScenarioError::MissionGraph { .. }));
    }

    #[test]
    fn legacy_scenario_without_mission_block_parses_unchanged() {
        // The Phase-3.2 mission block is optional; pre-3.2 scenarios
        // must continue to parse identically.
        let scenario = Scenario::from_toml_str(MINIMAL).expect("parse");
        assert!(scenario.document.mission.is_none());
    }

    // -----------------------------------------------------------------
    // Phase-3.3: `[vehicle.assembly]` block parser tests
    // -----------------------------------------------------------------

    const ASSEMBLY_TWO_BODY: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-two-body.toml"
    ));

    const ASSEMBLY_MASS_MISMATCH: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-mass-mismatch.toml"
    ));

    const ASSEMBLY_WITH_ENGINE_CLUSTER: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-with-engine-cluster.toml"
    ));

    const ASSEMBLY_ENGINE_CLUSTER_WITH_ENGINE_COMMAND: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-engine-cluster-with-engine-command.toml"
    ));

    const ASSEMBLY_ENGINE_CLUSTER_WITH_UNKNOWN_ENGINE_ID: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-engine-cluster-with-unknown-engine-id.toml"
    ));

    const ASSEMBLY_ENGINE_CLUSTER_WITH_THROTTLE_TOO_HIGH: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-engine-cluster-with-throttle-too-high.toml"
    ));

    const ASSEMBLY_ENGINE_CLUSTER_AND_MOTOR: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-engine-cluster-and-motor.toml"
    ));

    const ASSEMBLY_ENGINE_CLUSTER_WITHOUT_THRUST: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-engine-cluster-without-thrust.toml"
    ));

    const ASSEMBLY_WITH_EFFECTOR: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-with-effector.toml"
    ));

    const ASSEMBLY_WITH_RECOVERY: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-with-recovery.toml"
    ));

    const ASSEMBLY_RIGID_MISSING_BODY_INERTIA: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-rigid-missing-body-inertia.toml"
    ));

    const ASSEMBLY_RIGID_INERTIA_MISMATCH: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/assembly-rigid-inertia-mismatch.toml"
    ));

    fn without_forces(toml: &str) -> String {
        let start = toml.find("\n[forces]\n").expect("fixture has forces block");
        let after_header = start + "\n[forces]\n".len();
        let end = toml[after_header..]
            .find("\n[telemetry]")
            .map(|offset| after_header + offset)
            .expect("fixture has telemetry block after forces");
        let mut output = String::with_capacity(toml.len());
        output.push_str(&toml[..start]);
        output.push_str(&toml[end..]);
        output
    }

    fn force_names(scenario: &Scenario) -> Vec<&str> {
        scenario
            .document
            .force_models()
            .iter()
            .map(String::as_str)
            .collect()
    }

    #[test]
    fn parses_two_body_assembly_block() {
        let scenario = Scenario::from_toml_str(ASSEMBLY_TWO_BODY).expect("parse");
        let assembly = &scenario.document.vehicle.assembly;
        assert_eq!(assembly.bodies.len(), 2);
        assert_eq!(assembly.bodies[0].id, "main");
        assert_eq!(assembly.bodies[1].id, "fairing");
    }

    #[test]
    fn minimal_scenario_carries_single_body_assembly() {
        // Phase-3.13 v2 contract: every scenario has a non-empty
        // [vehicle.assembly] block; the analytic-toy scenario is no
        // exception.
        let scenario = Scenario::from_toml_str(MINIMAL).expect("parse");
        let assembly = &scenario.document.vehicle.assembly;
        assert_eq!(assembly.bodies.len(), 1);
        assert_eq!(assembly.bodies[0].id, "main");
        assert_eq!(assembly.bodies[0].dry_mass_kg.to_bits(), 1.0_f64.to_bits());
    }

    #[test]
    fn derives_gravity_only_when_forces_absent() {
        let scenario = Scenario::from_toml_str(&without_forces(MINIMAL)).expect("parse");
        assert_eq!(force_names(&scenario), ["gravity"]);
    }

    #[test]
    fn derives_thrust_and_aero_when_motor_and_deck_are_declared() {
        let scenario = Scenario::from_toml_str(&without_forces(SOUNDING_ROCKET)).expect("parse");
        assert_eq!(force_names(&scenario), ["gravity", "thrust", "aero"]);
    }

    #[test]
    fn derives_thrust_when_engine_cluster_is_declared() {
        let scenario =
            Scenario::from_toml_str(&without_forces(ASSEMBLY_WITH_ENGINE_CLUSTER)).expect("parse");
        assert_eq!(force_names(&scenario), ["gravity", "thrust"]);
    }

    #[test]
    fn recovery_devices_do_not_derive_force_model_names() {
        let scenario =
            Scenario::from_toml_str(&without_forces(ASSEMBLY_WITH_RECOVERY)).expect("parse");
        assert_eq!(force_names(&scenario), ["gravity"]);
    }

    #[test]
    fn explicit_forces_override_derived_order() {
        let scenario = Scenario::from_toml_str(SOUNDING_ROCKET).expect("parse");
        assert_eq!(force_names(&scenario), ["gravity", "aero", "thrust"]);
    }

    #[test]
    #[ignore = "Phase-3.13 retired flat fields; cross-consistency check is now structurally impossible"]
    fn rejects_mass_mismatch_between_flat_and_assembly() {
        let err = Scenario::from_toml_str(ASSEMBLY_MASS_MISMATCH).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { .. }),
            "expected InconsistentSection, got {err:?}",
        );
    }

    #[test]
    #[ignore = "Phase-3.13 retired flat fields; relative-tolerance cross-consistency is now structurally impossible"]
    fn rejects_small_mass_mismatch_with_relative_tolerance() {
        let toml = ASSEMBLY_TWO_BODY
            .replace(
                "mass_kg                  = 0.085",
                "mass_kg                  = 0.000000001",
            )
            .replace("dry_mass_kg  = 0.080", "dry_mass_kg  = 0.000000000001")
            .replace("dry_mass_kg  = 0.005", "dry_mass_kg  = 0.000000000001");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { .. }),
            "expected InconsistentSection, got {err:?}",
        );
    }

    #[test]
    fn rejects_rigid_assembly_body_without_inertia() {
        let err = Scenario::from_toml_str(ASSEMBLY_RIGID_MISSING_BODY_INERTIA).unwrap_err();
        assert!(
            matches!(
                err,
                ScenarioError::MissingRequiredField {
                    ref field,
                    ..
                } if field == "vehicle.assembly.bodies[0].dry_inertia_body_kg_m2"
            ),
            "expected MissingRequiredField for per-body inertia, got {err:?}",
        );
    }

    #[test]
    #[ignore = "Phase-3.13 retired flat fields; rigid inertia cross-consistency is now structurally impossible"]
    fn rejects_rigid_flat_and_assembly_inertia_mismatch() {
        let err = Scenario::from_toml_str(ASSEMBLY_RIGID_INERTIA_MISMATCH).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { .. }),
            "expected InconsistentSection, got {err:?}",
        );
    }

    #[test]
    fn parses_assembly_with_engine_cluster() {
        let scenario = match Scenario::from_toml_str(ASSEMBLY_WITH_ENGINE_CLUSTER) {
            Ok(s) => s,
            Err(e) => panic!("parse failed: {e:?}"),
        };
        let assembly = &scenario.document.vehicle.assembly;
        assert_eq!(assembly.engines.len(), 2);
        assert_eq!(assembly.engines[0].id, "engine_a");
        assert_eq!(assembly.engines[1].id, "engine_b");
        assert!((assembly.engines[0].limits.max_thrust_n - 1000.0).abs() < 1e-12);
        assert_eq!(
            assembly.cluster_layout,
            Some(crate::ClusterLayoutConfig::Ring)
        );
    }

    #[test]
    fn rejects_engine_with_zero_thrust() {
        let toml =
            ASSEMBLY_WITH_ENGINE_CLUSTER.replace("max_thrust_n = 1000.0", "max_thrust_n = 0.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { .. }),
            "expected InvalidNumber, got {err:?}",
        );
    }

    #[test]
    fn rejects_engine_with_negative_isp() {
        let toml = ASSEMBLY_WITH_ENGINE_CLUSTER.replace("isp_s = 250.0", "isp_s = -100.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { .. }),
            "expected InvalidNumber, got {err:?}",
        );
    }

    #[test]
    fn rejects_engine_with_negative_ignition_transient() {
        let toml = ASSEMBLY_WITH_ENGINE_CLUSTER
            .replace("ignition_transient_s = 0.1", "ignition_transient_s = -0.05");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { .. }),
            "expected InvalidNumber, got {err:?}",
        );
    }

    #[test]
    fn rejects_duplicate_engine_id() {
        // Replace second engine id to match the first.
        let toml = ASSEMBLY_WITH_ENGINE_CLUSTER.replace(
            "id                  = \"engine_b\"",
            "id                  = \"engine_a\"",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::DuplicateValue { ref field, .. } if field.contains("engines")),
            "expected DuplicateValue, got {err:?}",
        );
    }

    #[test]
    fn rejects_unknown_cluster_layout() {
        let toml = ASSEMBLY_WITH_ENGINE_CLUSTER
            .replace("cluster_layout = \"ring\"", "cluster_layout = \"flower\"");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::ParseToml(_)));
    }

    #[test]
    fn rejects_engine_cluster_without_thrust_force_model() {
        let err = Scenario::from_toml_str(ASSEMBLY_ENGINE_CLUSTER_WITHOUT_THRUST).unwrap_err();
        assert!(
            matches!(
                err,
                ScenarioError::InconsistentSection {
                    ref field_a,
                    ref field_b,
                    ..
                } if field_a == "vehicle.assembly.engines" && field_b == "forces.models"
            ),
            "expected InconsistentSection for missing thrust force, got {err:?}",
        );
    }

    #[test]
    fn parses_assembly_with_effector_block() {
        let scenario = match Scenario::from_toml_str(ASSEMBLY_WITH_EFFECTOR) {
            Ok(s) => s,
            Err(e) => panic!("parse failed: {e:?}"),
        };
        let assembly = &scenario.document.vehicle.assembly;
        assert_eq!(assembly.effectors.len(), 1);
        let effector = &assembly.effectors[0];
        assert_eq!(effector.id, "delta_e");
        match &effector.kind {
            crate::EffectorKindConfig::LinearActuator { tau_s } => {
                assert!((tau_s.unwrap_or(0.0) - 0.05).abs() < 1e-12);
            }
            crate::EffectorKindConfig::DirectTorque { .. } => {
                panic!("expected LinearActuator effector kind in fixture")
            }
        }
        assert!((effector.limits.max_rate_per_s - 5.236).abs() < 1e-12);
        assert_eq!(effector.unit.as_deref(), Some("rad"));
    }

    #[test]
    fn parses_direct_torque_effector_under_v3() {
        let toml = ASSEMBLY_WITH_EFFECTOR
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "kind             = { kind = \"linear_actuator\", tau_s = 0.05 }",
                "kind             = { kind = \"direct_torque\", axis = \"roll\", \
                 effectiveness_n_m_per_rad = 0.5 }",
            )
            .replace(
                "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
                "",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("v3 direct_torque should validate");
        let effector = &scenario.document.vehicle.assembly.effectors[0];
        match &effector.kind {
            crate::EffectorKindConfig::DirectTorque {
                axis,
                effectiveness_n_m_per_rad,
            } => {
                assert_eq!(*axis, crate::TorqueAxis::Roll);
                assert!((effectiveness_n_m_per_rad - 0.5).abs() < 1e-12);
            }
            crate::EffectorKindConfig::LinearActuator { .. } => {
                panic!("expected DirectTorque effector kind")
            }
        }
    }

    #[test]
    fn direct_torque_effector_is_v3_only() {
        let toml = ASSEMBLY_WITH_EFFECTOR
            .replace(
                "kind             = { kind = \"linear_actuator\", tau_s = 0.05 }",
                "kind             = { kind = \"direct_torque\", axis = \"pitch\", \
                 effectiveness_n_m_per_rad = 1.0 }",
            )
            .replace(
                "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
                "",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::SchemaVersionFieldReserved {
                field, required, ..
            } => {
                assert!(
                    field.contains("direct_torque"),
                    "expected direct_torque in field, got {field}"
                );
                assert_eq!(required, 3);
            }
            other => panic!("expected SchemaVersionFieldReserved, got {other:?}"),
        }
    }

    #[test]
    fn direct_torque_effector_rejects_non_positive_effectiveness() {
        let toml = ASSEMBLY_WITH_EFFECTOR
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "kind             = { kind = \"linear_actuator\", tau_s = 0.05 }",
                "kind             = { kind = \"direct_torque\", axis = \"yaw\", \
                 effectiveness_n_m_per_rad = 0.0 }",
            )
            .replace(
                "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
                "",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. }
                if field.contains("effectiveness_n_m_per_rad")),
            "expected InvalidNumber on effectiveness_n_m_per_rad, got {err:?}",
        );
    }

    #[test]
    fn rejects_effector_jam_at_outside_limits() {
        let toml = ASSEMBLY_WITH_EFFECTOR.replace(
            "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
            "fault = { kind = \"jam\", at = 999.0 }",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field.contains("fault.at")),
            "expected InvalidNumber on fault.at, got {err:?}",
        );
    }

    #[test]
    fn rejects_effector_min_geq_max() {
        let toml = ASSEMBLY_WITH_EFFECTOR
            .replace("min = -0.349, max = 0.349", "min = 0.349, max = -0.349");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field.contains("limits.min")),
            "expected InvalidNumber on limits.min, got {err:?}",
        );
    }

    #[test]
    fn rejects_effector_deadband_larger_than_range() {
        let toml = ASSEMBLY_WITH_EFFECTOR.replace("deadband = 0.0", "deadband = 1.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field.contains("limits.deadband")),
            "expected InvalidNumber on limits.deadband, got {err:?}",
        );
    }

    #[test]
    fn rejects_effector_latency_not_integer_multiple_of_dt() {
        let toml = ASSEMBLY_WITH_EFFECTOR.replace("latency_s = 0.020", "latency_s = 0.0015");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field.contains("limits.latency_s")),
            "expected InvalidNumber on limits.latency_s, got {err:?}",
        );
    }

    #[test]
    fn rejects_negative_effector_tau() {
        let toml = ASSEMBLY_WITH_EFFECTOR.replace("tau_s = 0.05", "tau_s = -0.05");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field.contains("kind.tau_s")),
            "expected InvalidNumber on kind.tau_s, got {err:?}",
        );
    }

    #[test]
    fn rejects_effector_reduced_rate_factor_out_of_range() {
        let toml = ASSEMBLY_WITH_EFFECTOR.replace(
            "command_schedule = { kind = \"step_at\", time_s = 0.5, before = 0.0, after = 0.087 }",
            "fault = { kind = \"reduced_rate\", factor = 2.0 }",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field.contains("fault.factor")),
            "expected InvalidNumber on fault.factor, got {err:?}",
        );
    }

    #[test]
    fn rejects_duplicate_effector_id() {
        let toml = ASSEMBLY_WITH_EFFECTOR.replace(
            "[[vehicle.assembly.effectors]]\nid               = \"delta_e\"",
            "[[vehicle.assembly.effectors]]\nid               = \"delta_e\"\nkind             = { kind = \"linear_actuator\" }\nlimits           = { min = -1.0, max = 1.0, max_rate_per_s = 1.0, deadband = 0.0, latency_s = 0.0 }\n[[vehicle.assembly.effectors]]\nid               = \"delta_e\"",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::DuplicateValue { ref field, .. } if field.contains("effectors")),
            "expected DuplicateValue on effectors id, got {err:?}",
        );
    }

    #[test]
    fn rejects_empty_bodies_list() {
        // Construct via .replace because the fixture intentionally has
        // bodies; remove them via runtime mutation.
        let toml = ASSEMBLY_TWO_BODY.replace(
            "[[vehicle.assembly.bodies]]",
            "[[vehicle.assembly.removed]]",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        // The serde-level rejection of `[[vehicle.assembly.removed]]`
        // (an unknown field) fires first; both
        // `EmptyList` and a serde parse error are acceptable.
        assert!(
            matches!(err, ScenarioError::ParseToml(_))
                || matches!(err, ScenarioError::EmptyList { .. }),
            "expected ParseToml or EmptyList, got {err:?}"
        );
    }

    #[test]
    fn rejects_duplicate_body_id() {
        let toml = ASSEMBLY_TWO_BODY.replace("\"fairing\"", "\"main\"");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::DuplicateValue { .. }),
            "expected DuplicateValue, got {err:?}"
        );
    }

    // -----------------------------------------------------------------
    // Phase-3.9 recovery
    // -----------------------------------------------------------------

    #[test]
    fn parses_assembly_with_recovery_block() {
        let scenario = match Scenario::from_toml_str(ASSEMBLY_WITH_RECOVERY) {
            Ok(s) => s,
            Err(e) => panic!("parse failed: {e:?}"),
        };
        let assembly = &scenario.document.vehicle.assembly;
        assert_eq!(assembly.recovery.len(), 3);
        assert_eq!(assembly.recovery[0].id, "main_chute");
        assert!(matches!(
            assembly.recovery[0].kind,
            crate::RecoveryKindConfig::ParachuteDrag {
                c_d: 1.5,
                area_inflated_m2: 2.0
            }
        ));
        assert!(matches!(
            assembly.recovery[1].kind,
            crate::RecoveryKindConfig::DrogueMain { .. }
        ));
        assert!(matches!(
            assembly.recovery[2].kind,
            crate::RecoveryKindConfig::DragDevice { .. }
        ));
    }

    #[test]
    fn rejects_recovery_non_positive_c_d() {
        let toml = ASSEMBLY_WITH_RECOVERY.replace("c_d = 1.5", "c_d = 0.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field.contains("c_d")),
            "expected InvalidNumber on c_d, got {err:?}"
        );
    }

    #[test]
    fn rejects_recovery_non_positive_area() {
        let toml =
            ASSEMBLY_WITH_RECOVERY.replace("area_inflated_m2 = 2.0", "area_inflated_m2 = -1.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field.contains("area")),
            "expected InvalidNumber on area, got {err:?}"
        );
    }

    #[test]
    fn rejects_duplicate_recovery_id() {
        // Replace the second declared id with the first so the
        // duplicate-id validator fires.
        let toml = ASSEMBLY_WITH_RECOVERY.replace(r#"id   = "drogue""#, r#"id   = "main_chute""#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::DuplicateValue { ref field, .. } if field.contains("recovery")),
            "expected DuplicateValue on recovery id, got {err:?}"
        );
    }

    #[test]
    fn rejects_deploy_recovery_event_referencing_unknown_id() {
        let toml = format!(
            "{ASSEMBLY_WITH_RECOVERY}{}",
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt_deploy"
trigger = { kind = "at_apogee" }
action  = { kind = "deploy_recovery", id = "no_such_device", command = "deploy" }
"#
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnknownRecoveryReference { ref id, .. } if id == "no_such_device"),
            "expected UnknownRecoveryReference, got {err:?}"
        );
    }

    #[test]
    fn rejects_deploy_drogue_against_parachute_drag_kind() {
        // The `main_chute` device is `parachute_drag`, which only
        // accepts `deploy`. `deploy_drogue` is incompatible.
        let toml = format!(
            "{ASSEMBLY_WITH_RECOVERY}{}",
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt_deploy"
trigger = { kind = "at_apogee" }
action  = { kind = "deploy_recovery", id = "main_chute", command = "deploy_drogue" }
"#
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(
                err,
                ScenarioError::IncompatibleRecoveryCommand { ref command, ref kind, .. }
                    if command == "deploy_drogue" && kind == "parachute_drag"
            ),
            "expected IncompatibleRecoveryCommand, got {err:?}"
        );
    }

    #[test]
    fn accepts_deploy_recovery_event_with_compatible_command() {
        let toml = format!(
            "{ASSEMBLY_WITH_RECOVERY}{}",
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt_deploy"
trigger = { kind = "at_apogee" }
action  = { kind = "deploy_recovery", id = "main_chute", command = "deploy" }
"#
        );
        let scenario = match Scenario::from_toml_str(&toml) {
            Ok(s) => s,
            Err(e) => panic!("parse failed: {e:?}"),
        };
        let mission = scenario.document.mission.as_ref().expect("mission");
        assert_eq!(mission.events.len(), 1);
    }

    // -----------------------------------------------------------------
    // Phase-3.10 sensors
    // -----------------------------------------------------------------

    #[test]
    fn parses_phase_3_10_sensor_kinds() {
        let toml = SOUNDING_ROCKET.replace(
            "[sensors.imu]\nkind = \"imu\"\nfile = \"../sensors/imu-tactical.toml\"",
            r#"[sensors.imu]
kind = "imu"
file = "../sensors/imu-tactical.toml"

[sensors.gnss]
kind = "gnss"
file = "../sensors/gnss-textbook.toml"

[sensors.mag]
kind = "magnetometer"
file = "../sensors/magnetometer-textbook.toml"

[sensors.star]
kind = "star_tracker"
file = "../sensors/star-tracker-textbook.toml""#,
        );
        let scenario = match Scenario::from_toml_str(&toml) {
            Ok(s) => s,
            Err(e) => panic!("parse failed: {e:?}"),
        };
        let sensors = scenario.document.sensors.as_ref().expect("sensors");
        assert_eq!(sensors.len(), 6);
        assert_eq!(sensors.get("gnss").expect("gnss").kind, "gnss");
        assert_eq!(sensors.get("mag").expect("mag").kind, "magnetometer");
        assert_eq!(sensors.get("star").expect("star").kind, "star_tracker");
    }

    #[test]
    fn rejects_phase_3_10_sensor_without_file() {
        let toml = SOUNDING_ROCKET.replace(
            "[sensors.imu]\nkind = \"imu\"\nfile = \"../sensors/imu-tactical.toml\"",
            "[sensors.imu]\nkind = \"imu\"\nfile = \"../sensors/imu-tactical.toml\"\n\n[sensors.gnss]\nkind = \"gnss\"",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, ref name, .. } if field == "sensors.gnss.file" && name == "gnss"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_unknown_sensor_kind() {
        let toml =
            SOUNDING_ROCKET.replace(r#"kind = "imu""#, r#"kind = "magnetic_anomaly_detector""#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnknownModel { ref name, .. } if name == "magnetic_anomaly_detector"),
            "got {err:?}",
        );
    }

    // Phase-4.B closed-loop scenario fixture.
    const CLOSED_LOOP_ATTITUDE_HOLD: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
    ));

    #[test]
    fn parses_closed_loop_attitude_hold_scenario() {
        let scenario = Scenario::from_toml_str(CLOSED_LOOP_ATTITUDE_HOLD).unwrap();
        let fc = scenario
            .document
            .fc
            .as_ref()
            .expect("scenario must declare [fc] block");
        assert_eq!(fc.estimator, crate::FcEstimatorKind::Ekf);
        assert_eq!(fc.guidance, crate::FcGuidanceKind::AttitudeHold);
        assert_eq!(fc.base_rate_hz, 1000);
        assert!(!fc.gain_schedule.is_empty());
        assert!(fc.phase_authority.as_ref().is_some_and(|m| m.len() == 2));
    }
}
