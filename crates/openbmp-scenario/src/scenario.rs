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

        // Synthesise the default `[forces]` list from the
        // assembly when the scenario does not declare one explicitly.
        // After this point the rest of the runner sees a populated
        // `Option<ForcesConfig>` and behaves as if the scenario had
        // hand-listed the derived models.
        if document.forces.is_none() {
            document.forces = Some(crate::ForcesConfig {
                models: document.resolved_force_models(),
                phase_override: Vec::new(),
            });
        }

        let scenario = Self {
            document,
            source_dir: source_dir.map(Into::into),
        };
        scenario.validate_with_registry(&ModelRegistry::full())?;
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
    /// default [`ModelRegistry::full`] used by the parser entry
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
        if let Some(output) = self
            .document
            .landing_footprint
            .as_ref()
            .and_then(|footprint| footprint.monte_carlo.as_ref())
            .map(|monte_carlo| &monte_carlo.output)
        {
            if let Some(path) = &output.samples_csv {
                paths.insert(
                    "landing_footprint.monte_carlo.output.samples_csv".to_owned(),
                    self.resolve_path(path),
                );
            }
            if let Some(path) = &output.samples_parquet {
                paths.insert(
                    "landing_footprint.monte_carlo.output.samples_parquet".to_owned(),
                    self.resolve_path(path),
                );
            }
            if let Some(path) = &output.summary_toml {
                paths.insert(
                    "landing_footprint.monte_carlo.output.summary_toml".to_owned(),
                    self.resolve_path(path),
                );
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
            if let Some(deck) = &aero.deck {
                let resolved = ResolvedFile::load(self.resolve_path(deck))?;
                resolved.verify_pin(aero.deck_sha256.as_deref())?;
                files.insert("aero.deck".to_owned(), resolved);
            }
        }

        if let Some(motor) = self
            .document
            .propulsion
            .as_ref()
            .and_then(|prop| prop.motor.as_ref())
            && let Some(file) = &motor.file
        {
            let resolved = ResolvedFile::load(self.resolve_path(file))?;
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

        if let Some(epoch) = &self.document.epoch
            && let Some(eop) = &epoch.eop
        {
            let resolved = ResolvedFile::load(self.resolve_path(eop))?;
            resolved.verify_pin(epoch.eop_sha256.as_deref())?;
            files.insert("epoch.eop".to_owned(), resolved);
        }
        if let Some(epoch) = &self.document.epoch
            && let Some(leap_second_table) = &epoch.leap_second_table
        {
            let resolved = ResolvedFile::load(self.resolve_path(leap_second_table))?;
            resolved.verify_pin(epoch.leap_second_table_sha256.as_deref())?;
            files.insert("epoch.leap_second_table".to_owned(), resolved);
        }

        if let Some(ephemeris_file) = &self.document.environment.ephemeris_file {
            let resolved = ResolvedFile::load(self.resolve_path(ephemeris_file))?;
            resolved.verify_pin(self.document.environment.ephemeris_file_sha256.as_deref())?;
            files.insert("environment.ephemeris_file".to_owned(), resolved);
        }
        for (index, ephemeris_file) in self.document.environment.ephemeris_files.iter().enumerate()
        {
            let resolved = ResolvedFile::load(self.resolve_path(ephemeris_file))?;
            let pin = self
                .document
                .environment
                .ephemeris_files_sha256
                .get(index)
                .map(String::as_str);
            resolved.verify_pin(pin)?;
            files.insert(format!("environment.ephemeris_files[{index}]"), resolved);
        }
        if let Some(meta_kernel) = &self.document.environment.ephemeris_meta_kernel {
            let resolved = ResolvedFile::load(self.resolve_path(meta_kernel))?;
            resolved.verify_pin(
                self.document
                    .environment
                    .ephemeris_meta_kernel_sha256
                    .as_deref(),
            )?;
            let kernel_paths = parse_spice_meta_kernel_paths(&resolved)?;
            if !self
                .document
                .environment
                .ephemeris_meta_kernel_files_sha256
                .is_empty()
                && self
                    .document
                    .environment
                    .ephemeris_meta_kernel_files_sha256
                    .len()
                    != kernel_paths.len()
            {
                return Err(ScenarioError::InconsistentSection {
                    field_a: "environment.ephemeris_meta_kernel KERNELS_TO_LOAD".to_owned(),
                    value_a: kernel_paths.len().to_string(),
                    field_b: "environment.ephemeris_meta_kernel_files_sha256".to_owned(),
                    value_b: self
                        .document
                        .environment
                        .ephemeris_meta_kernel_files_sha256
                        .len()
                        .to_string(),
                });
            }
            files.insert("environment.ephemeris_meta_kernel".to_owned(), resolved);
            for (index, kernel_path) in kernel_paths.iter().enumerate() {
                let resolved = ResolvedFile::load(kernel_path)?;
                let pin = self
                    .document
                    .environment
                    .ephemeris_meta_kernel_files_sha256
                    .get(index)
                    .map(String::as_str);
                resolved.verify_pin(pin)?;
                files.insert(
                    format!("environment.ephemeris_meta_kernel.files[{index}]"),
                    resolved,
                );
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

fn parse_spice_meta_kernel_paths(
    meta_kernel: &ResolvedFile,
) -> Result<Vec<PathBuf>, ScenarioError> {
    let text = std::str::from_utf8(&meta_kernel.bytes).map_err(|err| {
        ScenarioError::InvalidReferencedFile {
            path: meta_kernel.path.clone(),
            reason: format!("SPICE meta-kernel is not UTF-8: {err}"),
        }
    })?;
    if !text.trim_start().starts_with("KPL/MK") {
        return Err(ScenarioError::InvalidReferencedFile {
            path: meta_kernel.path.clone(),
            reason: "SPICE meta-kernel must start with KPL/MK".to_owned(),
        });
    }

    let kernels =
        spice_kernel_values(text, "KERNELS_TO_LOAD", &meta_kernel.path)?.ok_or_else(|| {
            ScenarioError::InvalidReferencedFile {
                path: meta_kernel.path.clone(),
                reason: "SPICE meta-kernel missing KERNELS_TO_LOAD".to_owned(),
            }
        })?;
    let kernels = join_spice_continuations(kernels, &meta_kernel.path, "KERNELS_TO_LOAD")?;
    if kernels.is_empty() {
        return Err(ScenarioError::InvalidReferencedFile {
            path: meta_kernel.path.clone(),
            reason: "SPICE meta-kernel KERNELS_TO_LOAD must not be empty".to_owned(),
        });
    }

    let symbols = spice_kernel_values(text, "PATH_SYMBOLS", &meta_kernel.path)?.unwrap_or_default();
    let values = spice_kernel_values(text, "PATH_VALUES", &meta_kernel.path)?.unwrap_or_default();
    if symbols.is_empty() != values.is_empty() {
        return Err(ScenarioError::InvalidReferencedFile {
            path: meta_kernel.path.clone(),
            reason: "SPICE meta-kernel PATH_SYMBOLS and PATH_VALUES must be declared together"
                .to_owned(),
        });
    }
    let values = join_spice_continuations(values, &meta_kernel.path, "PATH_VALUES")?;
    if symbols.len() != values.len() {
        return Err(ScenarioError::InvalidReferencedFile {
            path: meta_kernel.path.clone(),
            reason: "SPICE meta-kernel PATH_SYMBOLS and PATH_VALUES length mismatch".to_owned(),
        });
    }

    let mut substitutions = BTreeMap::new();
    for (symbol, value) in symbols.iter().zip(values.iter()) {
        if symbol.is_empty() || symbol.chars().any(char::is_whitespace) {
            return Err(ScenarioError::InvalidReferencedFile {
                path: meta_kernel.path.clone(),
                reason: format!("SPICE meta-kernel PATH_SYMBOL `{symbol}` is invalid"),
            });
        }
        substitutions.insert(symbol.as_str(), value.as_str());
    }

    let base_dir = meta_kernel.path.parent().unwrap_or_else(|| Path::new(""));
    kernels
        .iter()
        .map(|kernel| {
            let substituted =
                substitute_spice_path_symbols(kernel, &substitutions, &meta_kernel.path)?;
            let path = PathBuf::from(substituted);
            Ok(if path.is_absolute() {
                path
            } else {
                base_dir.join(path)
            })
        })
        .collect()
}

fn spice_kernel_values(
    text: &str,
    key: &str,
    path: &Path,
) -> Result<Option<Vec<String>>, ScenarioError> {
    let data = spice_data_text(text);
    let Some(offset) = find_spice_assignment(&data, key) else {
        return Ok(None);
    };
    let bytes = data.as_bytes();
    let mut index = offset;
    while index < bytes.len() && bytes[index].is_ascii_whitespace() {
        index += 1;
    }
    if index >= bytes.len() {
        return Err(ScenarioError::InvalidReferencedFile {
            path: path.to_path_buf(),
            reason: format!("SPICE meta-kernel {key} assignment is empty"),
        });
    }
    let slice = if bytes[index] == b'(' {
        let end = data[index + 1..]
            .find(')')
            .map(|end| index + 1 + end)
            .ok_or_else(|| ScenarioError::InvalidReferencedFile {
                path: path.to_path_buf(),
                reason: format!("SPICE meta-kernel {key} list is missing `)`"),
            })?;
        &data[index + 1..end]
    } else if bytes[index] == b'\'' {
        let end = data[index + 1..]
            .find('\'')
            .map(|end| index + 2 + end)
            .ok_or_else(|| ScenarioError::InvalidReferencedFile {
                path: path.to_path_buf(),
                reason: format!("SPICE meta-kernel {key} scalar string is unterminated"),
            })?;
        &data[index..end]
    } else {
        return Err(ScenarioError::InvalidReferencedFile {
            path: path.to_path_buf(),
            reason: format!("SPICE meta-kernel {key} must be a quoted string or string list"),
        });
    };
    collect_spice_quoted_strings(slice, path, key).map(Some)
}

fn spice_data_text(text: &str) -> String {
    let mut out = String::new();
    let mut in_data = false;
    for line in text.lines() {
        let trimmed = line.trim_start();
        if trimmed.starts_with("\\begindata") {
            in_data = true;
            continue;
        }
        if trimmed.starts_with("\\begintext") {
            in_data = false;
            continue;
        }
        if in_data {
            out.push_str(line);
            out.push('\n');
        }
    }
    out
}

fn find_spice_assignment(data: &str, key: &str) -> Option<usize> {
    let mut search_start = 0;
    while let Some(relative) = data[search_start..].find(key) {
        let start = search_start + relative;
        let end = start + key.len();
        let before_ok = start == 0
            || !data.as_bytes()[start - 1].is_ascii_alphanumeric()
                && data.as_bytes()[start - 1] != b'_';
        let after_ok = end == data.len()
            || !data.as_bytes()[end].is_ascii_alphanumeric() && data.as_bytes()[end] != b'_';
        if before_ok && after_ok {
            let mut index = end;
            let bytes = data.as_bytes();
            while index < bytes.len() && bytes[index].is_ascii_whitespace() {
                index += 1;
            }
            if index < bytes.len() && bytes[index] == b'=' {
                return Some(index + 1);
            }
        }
        search_start = end;
    }
    None
}

fn collect_spice_quoted_strings(
    text: &str,
    path: &Path,
    key: &str,
) -> Result<Vec<String>, ScenarioError> {
    let mut values = Vec::new();
    let mut chars = text.char_indices().peekable();
    while let Some((_, ch)) = chars.next() {
        if ch != '\'' {
            continue;
        }
        let mut value = String::new();
        let mut closed = false;
        for (_, ch) in chars.by_ref() {
            if ch == '\'' {
                closed = true;
                break;
            }
            value.push(ch);
        }
        if !closed {
            return Err(ScenarioError::InvalidReferencedFile {
                path: path.to_path_buf(),
                reason: format!("SPICE meta-kernel {key} contains an unterminated string"),
            });
        }
        values.push(value);
    }
    Ok(values)
}

fn join_spice_continuations(
    values: Vec<String>,
    path: &Path,
    key: &str,
) -> Result<Vec<String>, ScenarioError> {
    let mut joined = Vec::new();
    let mut pending = String::new();
    for value in values {
        if let Some(prefix) = value.strip_suffix('+') {
            pending.push_str(prefix);
        } else if pending.is_empty() {
            joined.push(value);
        } else {
            pending.push_str(&value);
            joined.push(std::mem::take(&mut pending));
        }
    }
    if !pending.is_empty() {
        return Err(ScenarioError::InvalidReferencedFile {
            path: path.to_path_buf(),
            reason: format!("SPICE meta-kernel {key} has unterminated `+` continuation"),
        });
    }
    Ok(joined)
}

fn substitute_spice_path_symbols(
    value: &str,
    substitutions: &BTreeMap<&str, &str>,
    path: &Path,
) -> Result<String, ScenarioError> {
    let mut out = String::new();
    let mut chars = value.char_indices().peekable();
    while let Some((_, ch)) = chars.next() {
        if ch != '$' {
            out.push(ch);
            continue;
        }
        let mut symbol = String::new();
        while let Some((_, next)) = chars.peek().copied() {
            if next.is_ascii_alphanumeric() || next == '_' {
                symbol.push(next);
                chars.next();
            } else {
                break;
            }
        }
        if symbol.is_empty() {
            return Err(ScenarioError::InvalidReferencedFile {
                path: path.to_path_buf(),
                reason: format!("SPICE meta-kernel path `{value}` contains an empty path symbol"),
            });
        }
        let replacement = substitutions.get(symbol.as_str()).ok_or_else(|| {
            ScenarioError::InvalidReferencedFile {
                path: path.to_path_buf(),
                reason: format!(
                    "SPICE meta-kernel path `{value}` references undefined PATH_SYMBOL `{symbol}`"
                ),
            }
        })?;
        out.push_str(replacement);
    }
    Ok(out)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::document::{
        EventTriggerConfig, FcAntiWindupConfig, FcAttitudeLoopKind, FcAttitudeMpcConfig,
        FcFdirDetectorKindV5, FcIndiConfig, FcIndiFilterKind, FcLqrConfig, FcRateLoopKind,
        GrainGeometryConfig, MissionScope, MissionScopeKind, WGS84_J2_DEFAULT,
    };
    use openbmp_core::ValidationStatus;

    // Parser-test fixture loaded from the canonical
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
        // v3 scenarios that include only v2-era
        // fields parse and validate identically to v2; only the header
        // value changes.
        let toml = MINIMAL.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario = Scenario::from_toml_str(&toml).expect("v3 minimal should validate");
        assert_eq!(scenario.document.openbmp.scenario, 3);
    }

    fn assert_v3_block_reserved_under_v2(toml: &str, expected_field: &str) {
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

    fn assert_v3_block_not_yet_supported(
        toml: &str,
        expected_field: &str,
        expected_capability: &str,
    ) {
        let v3 = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&v3).unwrap_err();
        match err {
            ScenarioError::ElementNotYetSupported {
                field,
                missing_capability,
            } => {
                assert_eq!(field, expected_field);
                assert_eq!(missing_capability, expected_capability);
            }
            other => {
                panic!("expected ElementNotYetSupported for {expected_field}, got: {other:?}")
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
        assert_v3_block_reserved_under_v2(&toml_v2, "schedule");
        assert_v3_block_not_yet_supported(&toml_v2, "schedule", "multi-rate scheduling");
    }

    #[test]
    fn multi_body_block_is_v3_only() {
        let toml_v2 = format!("{MINIMAL}{MULTI_BODY_BLOCK}");
        assert_v3_block_reserved_under_v2(&toml_v2, "multi_body");
        let v3 = toml_v2.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&v3).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, ref field_b, .. }
                if field_a == "multi_body.separation" && field_b == "mission.events"),
            "expected multi_body to require matching mission events under v3, got {err:?}",
        );
    }

    #[test]
    fn v2_with_multiple_v3_only_blocks_reports_first_reserved_field() {
        let toml_v2 = format!("{MINIMAL}{SCHEDULE_BLOCK}{MULTI_BODY_BLOCK}");
        assert_v3_block_reserved_under_v2(&toml_v2, "schedule");
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
    fn egm2008_gravity_is_v3_only_and_validates_under_v3() {
        // The v3 gate fires before the cross-field check that
        // would otherwise reject `gravity_m_s2` against a non-constant
        // gravity model, so the v2-rejection test only needs to flip
        // the gravity selector — the leftover `gravity_m_s2` line is
        // covered by the schema-version-reserved diagnostic.
        let toml_v2 = MINIMAL.replace(
            "gravity       = \"constant\"",
            "gravity       = \"egm2008\"",
        );
        assert_v3_block_reserved_under_v2(&toml_v2, "environment.gravity = \"egm2008\"");

        // Under v3, `egm2008` is consumed: drop the
        // constant-gravity-only `gravity_m_s2` line and verify that
        // the document validates clean.
        let toml_v3 = MINIMAL
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "gravity       = \"constant\"\ngravity_m_s2  = 9.80665\n",
                "gravity       = \"egm2008\"\n",
            );
        let scenario = Scenario::from_toml_str(&toml_v3).expect("egm2008 must validate under v3");
        assert_eq!(scenario.document.environment.gravity, "egm2008");
        assert!(scenario.document.environment.gravity_m_s2.is_none());
    }

    #[test]
    fn egm2008_gravity_rejects_constant_only_fields_under_v3() {
        // Leftover `gravity_m_s2` under `gravity = "egm2008"` must be
        // rejected with `UnexpectedField` — the EGM2008 zonal model
        // pins WGS84 µ / R_e and the published J_n table, so it
        // exposes no per-scenario overrides.
        let toml = MINIMAL
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "gravity       = \"constant\"",
                "gravity       = \"egm2008\"",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::UnexpectedField { field, name, .. } => {
                assert_eq!(field, "environment.gravity_m_s2");
                assert_eq!(name, "egm2008");
            }
            other => panic!("expected UnexpectedField for gravity_m_s2, got: {other:?}"),
        }
    }

    #[test]
    fn third_body_gravity_is_v3_only_and_validates_under_v3() {
        let toml_v2 = MINIMAL.replace(
            "gravity       = \"constant\"",
            "gravity       = \"third_body\"",
        );
        assert_v3_block_reserved_under_v2(&toml_v2, "environment.gravity = \"third_body\"");

        let toml_v3 = MINIMAL
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "gravity       = \"constant\"\ngravity_m_s2  = 9.80665\n",
                "gravity       = \"third_body\"\ngravity_base  = \"j2\"\nmu_m3_s2      = 3.986004418e14\nr_e_m         = 6378137.0\nthird_bodies  = [\"sun\", \"moon\"]\nephemeris     = \"low_precision_sun_moon\"\n",
            );
        let scenario =
            Scenario::from_toml_str(&toml_v3).expect("third_body must validate under v3");
        assert_eq!(scenario.document.environment.gravity, "third_body");
        assert_eq!(
            scenario.document.environment.third_bodies,
            ["sun".to_owned(), "moon".to_owned()]
        );
    }

    #[test]
    fn third_body_gravity_rejects_duplicate_bodies() {
        let toml = MINIMAL
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "gravity       = \"constant\"\ngravity_m_s2  = 9.80665\n",
                "gravity       = \"third_body\"\ngravity_base  = \"point_mass\"\nmu_m3_s2      = 3.986004418e14\nthird_bodies  = [\"moon\", \"moon\"]\n",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(
            err,
            ScenarioError::DuplicateValue { ref field, .. }
                if field == "environment.third_bodies"
        ));
    }

    /// Canonical FC scenario used as the base for the v3-only
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

    fn without_fc_ekf_block(toml: &str) -> String {
        let start = toml.find("\n[fc.ekf]\n").expect("fixture has [fc.ekf]");
        let rest_start = toml[start + 1..]
            .find("\n[fc.autopilot_params]")
            .expect("fixture has [fc.autopilot_params]")
            + start
            + 1;
        format!("{}{}", &toml[..start], &toml[rest_start..])
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
estimator = "sr_ukf"
"#;
        let toml = append(fc_v2_scenario(), block);
        assert_v3_block_reserved_under_v2(&toml, "fc.estimator_lanes");
        // This block is consumed — under v3 it parses
        // and validates rather than emitting `ElementNotYetSupported`.
        let v3 = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario = Scenario::from_toml_str(&v3).expect("v3 estimator-lanes block validates");
        let lanes = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.estimator_lanes.as_ref())
            .expect("estimator_lanes block present");
        assert_eq!(
            lanes.voter,
            crate::document::FcEstimatorVoterKind::BestByCovarianceTrace
        );
        assert_eq!(lanes.lanes.len(), 2);
        assert_eq!(lanes.lanes[0].id, "primary");
        assert_eq!(lanes.lanes[1].id, "spare");
    }

    #[test]
    fn fc_estimator_lanes_validate_lane_dependencies_not_top_level_selector() {
        let block = r#"
[fc.estimator_lanes]
voter = "simplex_pass_through"

[[fc.estimator_lanes.lane]]
id        = "primary"
estimator = "ekf"
"#;
        let v3 = append(fc_v2_scenario(), block)
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace("estimator        = \"ekf\"", "estimator        = \"mekf\"");
        Scenario::from_toml_str(&v3)
            .expect("lane EKF should validate from [fc.ekf] without requiring top-level [fc.mekf]");
    }

    #[test]
    fn fc_estimator_lanes_require_each_lane_parameter_block() {
        let block = r#"
[fc.estimator_lanes]
voter = "simplex_pass_through"

[[fc.estimator_lanes.lane]]
id        = "sr"
estimator = "sr_ukf"
"#;
        let v3 = append(fc_v2_scenario(), block)
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace("estimator        = \"ekf\"", "estimator        = \"mekf\"");
        let err = Scenario::from_toml_str(&without_fc_ekf_block(&v3)).unwrap_err();
        match err {
            ScenarioError::InvalidFc { reason } => {
                assert!(
                    reason.contains(
                        "fc.estimator_lanes.lane[0].estimator = \"sr_ukf\" requires [fc.ekf]"
                    ),
                    "lane dependency diagnostic should mention the lane selector; got: {reason}"
                );
            }
            other => panic!("expected InvalidFc for missing lane [fc.ekf], got: {other:?}"),
        }
    }

    #[test]
    fn fc_sr_ukf_estimators_are_v3_only() {
        for (name, expected_field) in [
            ("sr_ukf", "fc.estimator = \"sr_ukf\""),
            ("sr_ukf_attitude", "fc.estimator = \"sr_ukf_attitude\""),
        ] {
            let toml = fc_v2_scenario().replace(
                "estimator        = \"ekf\"",
                &format!("estimator        = \"{name}\""),
            );
            assert_v3_block_reserved_under_v2(&toml, expected_field);
        }
    }

    #[test]
    fn fc_autopilot_allocation_block_is_v3_only() {
        let block = r#"
[fc.autopilot_allocation]
kind          = "prioritised_redistributed"
axis_priority = ["roll", "yaw", "pitch"]
"#;
        let toml = append(fc_v2_scenario(), block);
        assert_v3_block_reserved_under_v2(&toml, "fc.autopilot_allocation");
        // This block is consumed — under v3 it parses
        // and validates rather than emitting `ElementNotYetSupported`.
        let v3 = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario = Scenario::from_toml_str(&v3).expect("v3 allocation block validates");
        let alloc = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.autopilot_allocation.as_ref())
            .expect("allocation block present");
        assert_eq!(
            alloc.kind,
            crate::document::FcAutopilotAllocationKind::PrioritisedRedistributed
        );
    }

    #[test]
    fn fc_autopilot_allocation_axis_priority_must_list_all_axes() {
        let block = r#"
[fc.autopilot_allocation]
kind          = "prioritised_redistributed"
axis_priority = ["roll", "yaw"]
"#;
        let base = fc_v2_scenario().replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let toml = append(&base, block);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(
                err,
                ScenarioError::InvalidNumber { ref field, .. }
                    if field == "fc.autopilot_allocation.axis_priority"
            ),
            "expected axis_priority InvalidNumber, got {err:?}"
        );
    }

    #[test]
    fn fc_fdir_detector_block_is_v3_only_and_validates_under_v3() {
        // The block ships with a typed enum
        // (`windowed_mean_shift_glrt`) and consumed
        // `window_samples` / `false_alarm_rate` fields. v2 still
        // rejects the entire block via the schema-version gate.
        let block = r#"
[fc.fdir.detector]
kind             = "windowed_mean_shift_glrt"
window_samples   = 32
false_alarm_rate = 0.001
"#;
        let toml_v2 = append(fc_v2_scenario(), block);
        assert_v3_block_reserved_under_v2(&toml_v2, "fc.fdir.detector");
        let toml_v3 = toml_v2.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario = Scenario::from_toml_str(&toml_v3)
            .expect("windowed_mean_shift_glrt block must validate under v3");
        let detector = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.fdir.as_ref())
            .and_then(|fdir| fdir.detector.as_ref())
            .expect("detector block parsed");
        assert_eq!(detector.kind, FcFdirDetectorKindV5::WindowedMeanShiftGlrt);
        assert_eq!(detector.window_samples, Some(32));
        assert_eq!(detector.false_alarm_rate, Some(0.001));
    }

    #[test]
    fn fc_fdir_detector_rejects_parity_threshold_for_glrt_under_v3() {
        // The Patton-Frank parity-space residual generator is not yet
        // wired. Mixing its reserved `parity_threshold` with
        // `windowed_mean_shift_glrt` is rejected with a pointed
        // diagnostic.
        let block = r#"
[fc.fdir.detector]
kind             = "windowed_mean_shift_glrt"
window_samples   = 32
parity_threshold = 25.0
"#;
        let toml =
            append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::InvalidFc { reason } => {
                assert!(
                    reason.contains("parity_threshold"),
                    "diagnostic should mention parity_threshold; got {reason}"
                );
            }
            other => panic!("expected InvalidFc, got: {other:?}"),
        }
    }

    #[test]
    fn fc_fdir_detector_rejects_missing_window_samples_for_glrt_under_v3() {
        let block = r#"
[fc.fdir.detector]
kind = "windowed_mean_shift_glrt"
"#;
        let toml =
            append(fc_v2_scenario(), block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::InvalidFc { reason } => {
                assert!(
                    reason.contains("window_samples"),
                    "diagnostic should mention window_samples; got {reason}"
                );
            }
            other => panic!("expected InvalidFc, got: {other:?}"),
        }
    }

    #[test]
    fn fc_fdir_detector_rejects_invalid_false_alarm_rate_for_glrt_under_v3() {
        for bad in [0.0, 1.0, -0.5, 1.5] {
            let block = format!(
                r#"
[fc.fdir.detector]
kind             = "windowed_mean_shift_glrt"
window_samples   = 32
false_alarm_rate = {bad}
"#
            );
            let toml = append(fc_v2_scenario(), &block)
                .replace("openbmp.scenario = 2", "openbmp.scenario = 3");
            let err = Scenario::from_toml_str(&toml).unwrap_err();
            assert!(
                matches!(err, ScenarioError::InvalidFc { .. }),
                "false_alarm_rate = {bad} should be rejected; got {err:?}"
            );
        }
    }

    // -----------------------------------------------------------------
    // `[fc.imm]` validator tests.
    // -----------------------------------------------------------------

    /// Returns the canonical FC v3 fixture with `estimator = "imm"`
    /// and a well-formed 2-mode `[fc.imm]` block appended. Tests
    /// derive negative-test variants by replacement.
    fn fc_imm_v3_scenario() -> String {
        let block = r"
[fc.imm]
transition_matrix          = [[0.95, 0.05], [0.10, 0.90]]
initial_mode_probabilities = [0.9, 0.1]

[[fc.imm.mode]]
sigma_w_gyro = 0.01

[[fc.imm.mode]]
sigma_w_gyro = 0.1
";
        append(fc_v2_scenario(), block)
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace("estimator        = \"ekf\"", "estimator        = \"imm\"")
    }

    #[test]
    fn fc_imm_block_validates_under_v3_with_well_formed_2_mode_bank() {
        let scenario = Scenario::from_toml_str(&fc_imm_v3_scenario())
            .expect("well-formed 2-mode IMM block must validate under v3");
        let imm = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.imm.as_ref())
            .expect("imm block parsed");
        assert_eq!(imm.transition_matrix.len(), 2);
        assert_eq!(imm.initial_mode_probabilities.len(), 2);
        assert_eq!(imm.modes.len(), 2);
    }

    #[test]
    fn fc_imm_is_v3_only() {
        let toml = fc_imm_v3_scenario().replace("openbmp.scenario = 3", "openbmp.scenario = 2");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::SchemaVersionFieldReserved { field, .. } => {
                assert_eq!(field, "fc.estimator = \"imm\"");
            }
            other => panic!("expected SchemaVersionFieldReserved, got {other:?}"),
        }
    }

    #[test]
    fn fc_imm_rejects_transition_matrix_row_sum_other_than_one() {
        let toml = fc_imm_v3_scenario().replace(
            "transition_matrix          = [[0.95, 0.05], [0.10, 0.90]]",
            "transition_matrix          = [[0.95, 0.04], [0.10, 0.90]]",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::InvalidFc { reason } => {
                assert!(
                    reason.contains("row 0 sums to"),
                    "diagnostic should mention row 0; got: {reason}"
                );
            }
            other => panic!("expected InvalidFc, got {other:?}"),
        }
    }

    #[test]
    fn fc_imm_rejects_initial_probabilities_that_do_not_sum_to_one() {
        let toml = fc_imm_v3_scenario().replace(
            "initial_mode_probabilities = [0.9, 0.1]",
            "initial_mode_probabilities = [0.9, 0.05]",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::InvalidFc { reason } => {
                assert!(
                    reason.contains("initial_mode_probabilities sum"),
                    "diagnostic should mention probability-sum; got: {reason}"
                );
            }
            other => panic!("expected InvalidFc, got {other:?}"),
        }
    }

    #[test]
    fn fc_imm_rejects_mode_count_mismatch_with_transition_matrix() {
        // 2×2 transition matrix but only 1 [[fc.imm.mode]] entry.
        let toml = fc_imm_v3_scenario().replace(
            "[[fc.imm.mode]]\nsigma_w_gyro = 0.01\n\n[[fc.imm.mode]]\nsigma_w_gyro = 0.1\n",
            "[[fc.imm.mode]]\nsigma_w_gyro = 0.01\n",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::InvalidFc { reason } => {
                assert!(
                    reason.contains("fc.imm.mode count"),
                    "diagnostic should mention mode-count mismatch; got: {reason}"
                );
            }
            other => panic!("expected InvalidFc, got {other:?}"),
        }
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

    /// Inject `attitude_loop_kind = "<kind>"` into the existing
    /// `[fc.autopilot_params]` table of the v2 fixture and append the
    /// matching `[fc.autopilot_params.attitude_mpc]` sub-table.
    fn fc_v3_with_attitude_mpc(
        attitude_loop_kind: Option<&str>,
        attitude_mpc_block: Option<&str>,
    ) -> String {
        let mut text = fc_v2_scenario().replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        if let Some(kind) = attitude_loop_kind {
            text = text.replace(
                "trajectory_kind          = \"pid\"\n",
                &format!(
                    "trajectory_kind          = \"pid\"\nattitude_loop_kind       = \"{kind}\"\n"
                ),
            );
        }
        if let Some(block) = attitude_mpc_block {
            text.push('\n');
            text.push_str(block);
        }
        text
    }

    #[test]
    fn fc_attitude_mpc_block_is_v3_only() {
        let block = r"
[fc.autopilot_params.attitude_mpc]
horizon_n         = 20
q_x               = [100.0, 100.0, 50.0]
r_u               = [0.001, 0.001, 0.001]
terminal_p        = [1000.0, 1000.0, 500.0]
rate_limit_rad_s  = [3.0, 3.0, 3.0]
";
        let toml_v2 = append(fc_v2_scenario(), block);
        let err = Scenario::from_toml_str(&toml_v2).unwrap_err();
        match err {
            ScenarioError::SchemaVersionFieldReserved { field, .. } => {
                assert!(
                    field == "fc.autopilot_params.attitude_loop_kind"
                        || field == "fc.autopilot_params.attitude_mpc",
                    "unexpected SchemaVersionFieldReserved field: {field}"
                );
            }
            other => panic!("expected SchemaVersionFieldReserved, got {other:?}"),
        }
    }

    #[test]
    fn fc_attitude_mpc_block_under_v3_validates() {
        let mpc_block = r"
[fc.autopilot_params.attitude_mpc]
horizon_n         = 20
q_x               = [100.0, 100.0, 50.0]
r_u               = [0.001, 0.001, 0.001]
terminal_p        = [1000.0, 1000.0, 500.0]
rate_limit_rad_s  = [3.0, 3.0, 3.0]
";
        let toml = fc_v3_with_attitude_mpc(Some("mpc"), Some(mpc_block));
        let scenario = Scenario::from_toml_str(&toml).expect("v3 attitude MPC validates");
        let params = scenario
            .document
            .fc
            .as_ref()
            .and_then(|fc| fc.autopilot_params.as_ref())
            .expect("autopilot_params present");
        assert_eq!(params.attitude_loop_kind, Some(FcAttitudeLoopKind::Mpc));
        let mpc = params
            .attitude_mpc
            .as_ref()
            .expect("attitude_mpc block present");
        assert_eq!(
            *mpc,
            FcAttitudeMpcConfig {
                horizon_n: 20,
                q_x: [100.0, 100.0, 50.0],
                r_u: [0.001, 0.001, 0.001],
                terminal_p: [1000.0, 1000.0, 500.0],
                rate_limit_rad_s: [3.0, 3.0, 3.0],
            }
        );
    }

    #[test]
    fn fc_attitude_loop_kind_mpc_without_block_fails_closed() {
        let toml = fc_v3_with_attitude_mpc(Some("mpc"), None);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. }
                if field == "fc.autopilot_params.attitude_mpc"),
            "expected MissingRequiredField for fc.autopilot_params.attitude_mpc, got {err:?}"
        );
    }

    #[test]
    fn fc_attitude_mpc_block_without_kind_mpc_fails_closed() {
        let mpc_block = r"
[fc.autopilot_params.attitude_mpc]
horizon_n         = 20
q_x               = [100.0, 100.0, 50.0]
r_u               = [0.001, 0.001, 0.001]
terminal_p        = [1000.0, 1000.0, 500.0]
rate_limit_rad_s  = [3.0, 3.0, 3.0]
";
        let toml = fc_v3_with_attitude_mpc(None, Some(mpc_block));
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, .. }
                if field_a == "fc.autopilot_params.attitude_mpc"),
            "expected InconsistentSection on fc.autopilot_params.attitude_mpc, got {err:?}"
        );
    }

    #[test]
    fn fc_attitude_mpc_block_rejects_invalid_parameters() {
        let cases = [
            (
                "horizon zero",
                "horizon_n         = 20",
                "horizon_n         = 0",
            ),
            (
                "q_x zero on roll",
                "q_x               = [100.0, 100.0, 50.0]",
                "q_x               = [0.0, 100.0, 50.0]",
            ),
            (
                "r_u negative on yaw",
                "r_u               = [0.001, 0.001, 0.001]",
                "r_u               = [0.001, 0.001, -0.001]",
            ),
            (
                "terminal_p zero on pitch",
                "terminal_p        = [1000.0, 1000.0, 500.0]",
                "terminal_p        = [1000.0, 0.0, 500.0]",
            ),
            (
                "rate_limit zero on roll",
                "rate_limit_rad_s  = [3.0, 3.0, 3.0]",
                "rate_limit_rad_s  = [0.0, 3.0, 3.0]",
            ),
        ];
        let nominal = r"
[fc.autopilot_params.attitude_mpc]
horizon_n         = 20
q_x               = [100.0, 100.0, 50.0]
r_u               = [0.001, 0.001, 0.001]
terminal_p        = [1000.0, 1000.0, 500.0]
rate_limit_rad_s  = [3.0, 3.0, 3.0]
";
        for (label, from, to) in cases {
            let block = nominal.replace(from, to);
            let toml = fc_v3_with_attitude_mpc(Some("mpc"), Some(&block));
            let err = Scenario::from_toml_str(&toml).unwrap_err();
            assert!(
                matches!(err, ScenarioError::InvalidNumber { .. }),
                "{label}: expected InvalidNumber, got {err:?}"
            );
        }
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
        assert_v3_block_reserved_under_v2(&toml, "atmosphere.kind = \"nrlmsise00\"");

        let v3 = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario = Scenario::from_toml_str(&v3).expect("nrlmsise00 validates under v3");
        assert_eq!(scenario.document.environment.atmosphere, "nrlmsise00");
    }

    const NRLMSISE00_STRUCTURED_ATMOSPHERE_BLOCK: &str = r#"
[atmosphere]
kind = "nrlmsise00"
"#;

    #[test]
    fn nrlmsise00_structured_atmosphere_kind_is_v3_only() {
        let toml = append(MINIMAL, NRLMSISE00_STRUCTURED_ATMOSPHERE_BLOCK);
        assert_v3_block_reserved_under_v2(&toml, "atmosphere.kind = \"nrlmsise00\"");

        let v3 = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario =
            Scenario::from_toml_str(&v3).expect("structured nrlmsise00 validates under v3");
        assert_eq!(
            scenario.document.atmosphere.as_ref().unwrap().kind,
            "nrlmsise00"
        );
    }

    #[test]
    fn nrlmsise00_structured_inputs_validate_under_v3() {
        let block = r#"
[atmosphere]
kind = "nrlmsise00"
year = 2024
day_of_year = 80
utc_s = 43200.0
latitude_deg = 0.0
longitude_deg = 0.0
local_apparent_solar_time_h = 12.0
f107_average_81day_sfu = 150.0
f107_yesterday_sfu = 150.0
ap_average = 4.0
"#;
        let toml = append(MINIMAL, block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario =
            Scenario::from_toml_str(&toml).expect("structured nrlmsise00 inputs validate");
        let atmosphere = scenario.document.atmosphere.as_ref().unwrap();
        assert_eq!(atmosphere.day_of_year, Some(80));
        assert_eq!(atmosphere.f107_average_81day_sfu, Some(150.0));
    }

    #[test]
    fn nrlmsise00_structured_inputs_reject_bad_latitude() {
        let block = r#"
[atmosphere]
kind = "nrlmsise00"
latitude_deg = 100.0
"#;
        let toml = append(MINIMAL, block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        match err {
            ScenarioError::InvalidNumber { field, .. } => {
                assert_eq!(field, "atmosphere.latitude_deg");
            }
            other => panic!("expected InvalidNumber for atmosphere.latitude_deg, got: {other:?}"),
        }
    }

    const NRLMSIS2_COMPAT_STRUCTURED_ATMOSPHERE_BLOCK: &str = r#"
[atmosphere]
kind = "nrlmsis2_compat"
"#;

    #[test]
    fn nrlmsis2_compat_environment_atmosphere_is_v3_only() {
        let toml = MINIMAL.replace(
            "atmosphere    = \"none\"",
            "atmosphere    = \"nrlmsis2_compat\"",
        );
        assert_v3_block_reserved_under_v2(&toml, "atmosphere.kind = \"nrlmsis2_compat\"");

        let v3 = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario = Scenario::from_toml_str(&v3).expect("nrlmsis2_compat validates under v3");
        assert_eq!(scenario.document.environment.atmosphere, "nrlmsis2_compat");
    }

    #[test]
    fn nrlmsis2_compat_structured_atmosphere_kind_is_v3_only() {
        let toml = append(MINIMAL, NRLMSIS2_COMPAT_STRUCTURED_ATMOSPHERE_BLOCK);
        assert_v3_block_reserved_under_v2(&toml, "atmosphere.kind = \"nrlmsis2_compat\"");

        let v3 = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario =
            Scenario::from_toml_str(&v3).expect("structured nrlmsis2_compat validates under v3");
        assert_eq!(
            scenario.document.atmosphere.as_ref().unwrap().kind,
            "nrlmsis2_compat"
        );
    }

    #[test]
    fn nrlmsis2_compat_structured_inputs_validate_under_v3() {
        let block = r#"
[atmosphere]
kind = "nrlmsis2_compat"
year = 2024
day_of_year = 172
utc_s = 29000.0
latitude_deg = 60.0
longitude_deg = -70.0
local_apparent_solar_time_h = 16.0
f107_average_81day_sfu = 150.0
f107_yesterday_sfu = 150.0
ap_average = 4.0
"#;
        let toml = append(MINIMAL, block).replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario =
            Scenario::from_toml_str(&toml).expect("structured nrlmsis2_compat inputs validate");
        let atmosphere = scenario.document.atmosphere.as_ref().unwrap();
        assert_eq!(atmosphere.kind, "nrlmsis2_compat");
        assert_eq!(atmosphere.day_of_year, Some(172));
        assert_eq!(atmosphere.latitude_deg, Some(60.0));
    }

    #[test]
    fn piecewise_exponential_environment_atmosphere_is_v3_only_and_validates_under_v3() {
        let toml_v2 = MINIMAL.replace(
            "atmosphere    = \"none\"",
            "atmosphere    = \"piecewise_exponential\"",
        );
        assert_v3_block_reserved_under_v2(&toml_v2, "atmosphere.kind = \"piecewise_exponential\"");

        let toml_v3 = toml_v2.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario =
            Scenario::from_toml_str(&toml_v3).expect("piecewise_exponential validates under v3");
        assert_eq!(
            scenario.document.environment.atmosphere,
            "piecewise_exponential"
        );
    }

    const PIECEWISE_EXP_STRUCTURED_ATMOSPHERE_BLOCK: &str = r#"
[atmosphere]
kind = "piecewise_exponential"
"#;

    #[test]
    fn piecewise_exponential_structured_atmosphere_kind_is_v3_only_and_validates_under_v3() {
        let toml_v2 = append(MINIMAL, PIECEWISE_EXP_STRUCTURED_ATMOSPHERE_BLOCK);
        assert_v3_block_reserved_under_v2(&toml_v2, "atmosphere.kind = \"piecewise_exponential\"");

        let toml_v3 = toml_v2.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let scenario =
            Scenario::from_toml_str(&toml_v3).expect("piecewise_exponential validates under v3");
        assert_eq!(
            scenario
                .document
                .atmosphere
                .expect("structured atmosphere")
                .kind,
            "piecewise_exponential"
        );
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
    fn rejects_staging_range_optimization_vocabulary() {
        let toml = MINIMAL.replace(
            r#"name = "constant-acceleration-drop""#,
            r#"name = "maxrange-demo""#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::SafetyName { term, .. } if term == "max-range"));
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
    fn parses_inline_grain_motor_schema() {
        let toml = MINIMAL
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                r#"models = ["gravity"]"#,
                r#"models = ["gravity", "thrust"]"#,
            );
        let toml = format!(
            "{toml}\n\
             [propulsion.motor]\n\
             variant = \"solid\"\n\
             ignite_at_s = 0.0\n\
             mounted_to = \"main\"\n\
             \n\
             [propulsion.motor.grain]\n\
             geometry = \"end_burner\"\n\
             cross_section_area_m2 = 0.001\n\
             length_m = 0.05\n\
             throat_radius_m = 0.003\n\
             expansion_ratio = 8.0\n\
             \n\
             [propulsion.motor.grain.propellant]\n\
             label = \"synthetic_textbook\"\n\
             density_kg_m3 = 1700.0\n\
             burn_rate_a = 0.00004\n\
             burn_rate_n = 0.32\n\
             c_star_m_s = 1400.0\n\
             gamma = 1.2\n\
             web_steps = 64\n"
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let grain = scenario
            .document
            .propulsion
            .as_ref()
            .and_then(|propulsion| propulsion.motor.as_ref())
            .and_then(|motor| motor.grain.as_ref())
            .expect("grain motor parsed");
        assert_eq!(grain.geometry, GrainGeometryConfig::EndBurner);
        assert!(grain.propellant.web_steps == 64);
    }

    #[test]
    fn parses_offline_staging_analysis_schema() {
        let toml = MINIMAL.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        let toml = format!(
            "{toml}\n\
             [staging_analysis]\n\
             mode = \"optimal\"\n\
             delta_v_budget_m_s = 6000.0\n\
             payload_mass_kg = 100.0\n\
             \n\
             [[staging_analysis.stages]]\n\
             isp_s = 300.0\n\
             structural_coefficient = 0.1\n\
             \n\
             [[staging_analysis.stages]]\n\
             isp_s = 320.0\n\
             structural_coefficient = 0.12\n"
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        assert!(scenario.document.staging_analysis.is_some());
    }

    #[test]
    fn staging_analysis_is_v3_only() {
        let toml = format!(
            "{MINIMAL}\n\
             [staging_analysis]\n\
             mode = \"optimal\"\n\
             delta_v_budget_m_s = 6000.0\n\
             payload_mass_kg = 100.0\n\
             [[staging_analysis.stages]]\n\
             isp_s = 300.0\n\
             structural_coefficient = 0.1\n"
        );
        assert_v3_block_reserved_under_v2(&toml, "staging_analysis");
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

    // Sounding-rocket-shaped fixture. Loaded from a sibling
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
    // Layered wind
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
    // HWM14 wind
    // -----------------------------------------------------------------

    #[test]
    fn accepts_hwm14_wind_without_extra_fields() {
        let toml = SOUNDING_ROCKET
            .replace(
                r#"wind          = "constant""#,
                r#"wind          = "hwm14""#,
            )
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"hwm14\"\n",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("hwm14 wind scenario must parse");
        let wind = scenario.document.wind.as_ref().expect("wind block present");
        assert_eq!(wind.kind, "hwm14");
        assert!(wind.wind_ned_m_s.is_none());
        assert!(wind.layers.is_none());
        assert!(wind.day_of_year.is_none());
        assert!(wind.utc_s.is_none());
        assert!(wind.ap_current_3h.is_none());
    }

    #[test]
    fn accepts_hwm14_wind_with_model_inputs() {
        let toml = SOUNDING_ROCKET
            .replace(r#"wind          = "constant""#, r#"wind          = "hwm14""#)
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"hwm14\"\nyear = 1995\nday_of_year = 150\nutc_s = 43200.0\nlatitude_deg = -45.0\nlongitude_deg = -85.0\nap_current_3h = 80.0\n",
            );
        let scenario = Scenario::from_toml_str(&toml).expect("hwm14 wind scenario must parse");
        let wind = scenario.document.wind.as_ref().expect("wind block present");
        assert_eq!(wind.day_of_year, Some(150));
        assert_eq!(wind.utc_s.unwrap().to_bits(), 43_200.0_f64.to_bits());
        assert_eq!(wind.ap_current_3h.unwrap().to_bits(), 80.0_f64.to_bits());
    }

    #[test]
    fn rejects_hwm14_wind_with_layered_table() {
        let toml = SOUNDING_ROCKET
            .replace(r#"wind          = "constant""#, r#"wind          = "hwm14""#)
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"hwm14\"\nlayers = [\n  { altitude_m = 0.0, wind_ned_m_s = [5.0, 0.0, 0.0] },\n]\n",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnexpectedField { ref field, .. } if field == "wind.layers"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_hwm14_wind_with_bad_latitude() {
        let toml = SOUNDING_ROCKET
            .replace(
                r#"wind          = "constant""#,
                r#"wind          = "hwm14""#,
            )
            .replace(
                "[wind]\nkind         = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                "[wind]\nkind = \"hwm14\"\nlatitude_deg = 91.0\n",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field == "wind.latitude_deg"),
            "got {err:?}",
        );
    }

    // -----------------------------------------------------------------
    // Gust wind
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

    fn minimal_with_buildup_aero() -> String {
        MINIMAL
            .replace(
                r#"atmosphere    = "none""#,
                r#"atmosphere    = "us_standard_1976""#,
            )
            .replace(r#"models = ["gravity"]"#, r#"models = ["gravity", "aero"]"#)
            + r#"

[aero]

[aero.buildup]
body_diameter_m = 0.2
body_length_m = 2.4
surface_roughness_m = 6.0e-5
reference_area_m2 = 0.031415926535897934
reference_length_m = 0.2
center_of_gravity_from_nose_m = 1.2
mach_grid = { min = 0.0, max = 2.0, steps = 5 }
alpha_grid_deg = { min = 0.0, max = 6.0, steps = 4 }
reference_altitude_m = 0.0

[aero.buildup.nose]
shape = "ogive"
fineness = 3.5

[aero.buildup.afterbody]
exit_diameter_m = 0.14
length_m = 0.25

[aero.buildup.fins]
count = 4
root_chord_m = 0.30
tip_chord_m = 0.12
span_m = 0.16
thickness_ratio = 0.06
sweep_rad = 0.52
"#
    }

    #[test]
    fn parses_aero_buildup_without_external_deck_file() {
        let scenario = Scenario::from_toml_str(&minimal_with_buildup_aero()).unwrap();
        let aero = scenario.document.aero.as_ref().expect("aero block parsed");
        assert!(aero.deck.is_none());
        assert!(aero.buildup.is_some());
        let files = scenario
            .resolved_files()
            .expect("buildup has no external deck file");
        assert!(!files.contains_key("aero.deck"));
    }

    #[test]
    fn parses_hybrid_aero_method_with_buildup_deck_source() {
        let toml = minimal_with_buildup_aero()
            + r#"

[aero.method]
kind = "hybrid"

[aero.method.hybrid]
reference_length_m = 0.2
mach_handoff = 4.0

[aero.method.hybrid.continuum_low_mach]
kind = "deck"

[aero.method.hybrid.continuum_high_mach]
kind = "modified_newtonian"

[aero.method.hybrid.continuum_high_mach.modified_newtonian]
cp_max = 2.0
reference_area_m2 = 0.031415926535897934
reference_length_m = 0.2

[aero.method.hybrid.free_molecular]
reference_area_m2 = 0.031415926535897934
accommodation_normal = 1.0
accommodation_tangential = 1.0

[aero.method.hybrid.bridge]
kind = "linear"
kn_lo = 0.01
kn_hi = 10.0
"#;
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let method = scenario
            .document
            .aero
            .as_ref()
            .and_then(|aero| aero.method.as_ref())
            .expect("hybrid method parsed");
        assert_eq!(method.kind, "hybrid");
        assert!(method.hybrid.as_ref().is_some_and(|hybrid| {
            hybrid.continuum_low_mach.kind == "deck" && hybrid.bridge.kind == "linear"
        }));
    }

    #[test]
    fn rejects_aero_deck_and_buildup_together() {
        let toml = minimal_with_buildup_aero().replace(
            "[aero]\n",
            "[aero]\ndeck = \"data/aero/synthetic-finned-cylinder.toml\"\n",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(matches!(err, ScenarioError::AmbiguousAero), "got {err:?}");
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
    fn iers_tabulated_frame_requires_epoch_eop() {
        let toml = MINIMAL.replace(
            r#"frame_profile = "toy-fixed-earth""#,
            r#"frame_profile = "iers-tabulated""#,
        ) + r#"
[epoch]
scale = "UTC"
iso8601 = "2000-01-01T12:00:00Z"

[frames]
profile = "iers-tabulated"
"#;
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "epoch.eop"),
            "got {err:?}",
        );
    }

    #[test]
    fn resolved_files_includes_epoch_eop() {
        use std::fs;
        use tempfile::tempdir;

        let dir = tempdir().expect("tempdir");
        fs::write(dir.path().join("eop.toml"), "format = \"openbmp-eop-v1\"\n").expect("write eop");
        let toml = MINIMAL.replace(
            r#"frame_profile = "toy-fixed-earth""#,
            r#"frame_profile = "iers-tabulated""#,
        ) + r#"
[epoch]
scale = "UTC"
iso8601 = "2000-01-01T12:00:00Z"
eop = "eop.toml"

[frames]
profile = "iers-tabulated"
"#;
        let scenario = Scenario::from_toml_str_with_source_dir(&toml, Some(dir.path()))
            .expect("parse iers scenario");
        let files = scenario.resolved_files().expect("resolve files");
        assert!(files.contains_key("epoch.eop"));
    }

    #[test]
    fn resolved_files_includes_epoch_leap_second_table() {
        use std::fs;
        use tempfile::tempdir;

        let dir = tempdir().expect("tempdir");
        fs::write(
            dir.path().join("leaps.toml"),
            "format = \"openbmp-leap-seconds-v1\"\n",
        )
        .expect("write leaps");
        let toml = MINIMAL.to_owned()
            + r#"
[epoch]
scale = "UTC"
iso8601 = "2017-01-01T00:00:00Z"
leap_second_table = "leaps.toml"
"#;
        let scenario = Scenario::from_toml_str_with_source_dir(&toml, Some(dir.path()))
            .expect("parse leap-second scenario");
        let files = scenario.resolved_files().expect("resolve files");
        assert!(files.contains_key("epoch.leap_second_table"));
    }

    #[test]
    fn spk_utc_epoch_requires_leap_second_table() {
        let toml = MINIMAL
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "gravity       = \"constant\"\ngravity_m_s2  = 9.80665",
                "gravity       = \"third_body\"\ngravity_base  = \"point_mass\"\nmu_m3_s2      = 3.986004418e14\nthird_bodies  = [\"sun\"]\nephemeris     = \"spk\"\nephemeris_file = \"synthetic.bsp\"",
            )
            + r#"
[epoch]
scale = "UTC"
iso8601 = "2017-01-01T00:00:00Z"
"#;
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "epoch.leap_second_table"),
            "got {err:?}",
        );
    }

    #[test]
    fn third_body_spk_ephemeris_requires_file() {
        let toml_v2 = MINIMAL
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "gravity       = \"constant\"\ngravity_m_s2  = 9.80665",
                "gravity       = \"third_body\"\ngravity_base  = \"point_mass\"\nmu_m3_s2      = 3.986004418e14\nthird_bodies  = [\"sun\"]\nephemeris     = \"spk\"",
            );
        let err = Scenario::from_toml_str(&toml_v2).unwrap_err();
        assert!(
            matches!(err, ScenarioError::MissingRequiredField { ref field, .. } if field == "environment.ephemeris_file, environment.ephemeris_files, or environment.ephemeris_meta_kernel"),
            "got {err:?}",
        );
    }

    #[test]
    fn resolved_files_includes_spk_ephemeris_file() {
        use std::fs;
        use tempfile::tempdir;

        let dir = tempdir().expect("tempdir");
        fs::write(dir.path().join("synthetic.bsp"), b"stub spk").expect("write spk");
        let toml = MINIMAL
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "gravity       = \"constant\"\ngravity_m_s2  = 9.80665",
                "gravity       = \"third_body\"\ngravity_base  = \"point_mass\"\nmu_m3_s2      = 3.986004418e14\nthird_bodies  = [\"sun\"]\nephemeris     = \"spk\"\nephemeris_file = \"synthetic.bsp\"",
            )
            + r#"
[epoch]
scale = "TDB"
iso8601 = "2000-01-01T12:00:00Z"
"#;
        let scenario = Scenario::from_toml_str_with_source_dir(&toml, Some(dir.path()))
            .expect("parse spk scenario");
        let files = scenario.resolved_files().expect("resolve files");
        assert!(files.contains_key("environment.ephemeris_file"));
    }

    #[test]
    fn resolved_files_includes_spk_ephemeris_file_list() {
        use std::fs;
        use tempfile::tempdir;

        let dir = tempdir().expect("tempdir");
        fs::write(dir.path().join("base.bsp"), b"base spk").expect("write base spk");
        fs::write(dir.path().join("override.bsp"), b"override spk").expect("write override spk");
        let toml = MINIMAL
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "gravity       = \"constant\"\ngravity_m_s2  = 9.80665",
                "gravity       = \"third_body\"\ngravity_base  = \"point_mass\"\nmu_m3_s2      = 3.986004418e14\nthird_bodies  = [\"sun\"]\nephemeris     = \"spk\"\nephemeris_files = [\"base.bsp\", \"override.bsp\"]",
            )
            + r#"
[epoch]
scale = "TDB"
iso8601 = "2000-01-01T12:00:00Z"
"#;
        let scenario = Scenario::from_toml_str_with_source_dir(&toml, Some(dir.path()))
            .expect("parse spk scenario");
        let files = scenario.resolved_files().expect("resolve files");
        assert!(files.contains_key("environment.ephemeris_files[0]"));
        assert!(files.contains_key("environment.ephemeris_files[1]"));
    }

    #[test]
    fn resolved_files_expands_spk_meta_kernel() {
        use std::fs;
        use tempfile::tempdir;

        let dir = tempdir().expect("tempdir");
        fs::create_dir_all(dir.path().join("kernels/spk")).expect("create spk dir");
        fs::create_dir_all(dir.path().join("kernels/lsk")).expect("create lsk dir");
        fs::write(dir.path().join("kernels/spk/base.bsp"), b"base spk").expect("write spk");
        fs::write(dir.path().join("kernels/lsk/naif0012.tls"), b"lsk").expect("write lsk");
        fs::write(
            dir.path().join("mission.tm"),
            r#"
KPL/MK

\begindata
PATH_SYMBOLS = ( 'SPK', 'LSK' )
PATH_VALUES  = ( 'kernels/spk', 'kernels/lsk' )
KERNELS_TO_LOAD = (
    '$SPK/base.bsp'
    '$LSK/naif0012.tls'
)
\begintext
"#,
        )
        .expect("write meta-kernel");
        let toml = MINIMAL
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "gravity       = \"constant\"\ngravity_m_s2  = 9.80665",
                "gravity       = \"third_body\"\ngravity_base  = \"point_mass\"\nmu_m3_s2      = 3.986004418e14\nthird_bodies  = [\"sun\"]\nephemeris     = \"spk\"\nephemeris_meta_kernel = \"mission.tm\"",
            )
            + r#"
[epoch]
scale = "TDB"
iso8601 = "2000-01-01T12:00:00Z"
"#;
        let scenario = Scenario::from_toml_str_with_source_dir(&toml, Some(dir.path()))
            .expect("parse spk meta-kernel scenario");
        let files = scenario.resolved_files().expect("resolve files");
        assert!(files.contains_key("environment.ephemeris_meta_kernel"));
        assert!(files.contains_key("environment.ephemeris_meta_kernel.files[0]"));
        assert!(files.contains_key("environment.ephemeris_meta_kernel.files[1]"));
        assert_eq!(
            files["environment.ephemeris_meta_kernel.files[0]"].bytes,
            b"base spk"
        );
        assert_eq!(
            files["environment.ephemeris_meta_kernel.files[1]"].bytes,
            b"lsk"
        );
    }

    #[test]
    fn base_scenario_continues_to_parse_under_full_registry() {
        // Byte-stability guard: the analytic-toy scenario must
        // continue to parse and validate identically under the full
        // model registry.
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
    // `[mission]` block parser tests
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
    fn parses_mission_v4_states_scope_regions_and_override_flag() {
        let scenario = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "pad"
test_only_state_override = true

[mission.scope]
kind = "closed_loop_test"

[[mission.states]]
id    = "pad"
label = "pad"

[[mission.states]]
id     = "ascent"
label  = "ascent"
parent = "pad"

[[mission.events]]
id      = "liftoff"
trigger = { kind = "at_time", time_s = 0.0 }
action  = { kind = "enter_phase", phase = "ascent" }

[[mission.transitions]]
from  = "pad"
to    = "ascent"
event = "liftoff"

[[mission.regions]]
id            = "health"
initial_state = "nominal"

[[mission.regions.states]]
id = "nominal"

[[mission.regions.states]]
id = "abort_requested"
"#,
        ))
        .expect("parse");

        let mission = scenario.document.mission.as_ref().expect("mission");
        assert!(mission.phases.is_empty());
        assert_eq!(mission.states.len(), 2);
        assert_eq!(mission.regions.len(), 1);
        assert!(mission.test_only_state_override);
        assert!(matches!(
            mission.scope,
            Some(MissionScope::Config(ref scope))
                if scope.kind == MissionScopeKind::ClosedLoopTest
        ));
    }

    #[test]
    fn parses_mission_event_once_false() {
        let scenario = Scenario::from_toml_str(&with_mission(MISSION_ONCE_FALSE)).expect("parse");
        let mission = scenario.document.mission.as_ref().expect("mission");
        assert!(!mission.events[0].once);
    }

    #[test]
    fn parses_at_velocity_trigger() {
        let scenario = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "burnout_velocity"
trigger = { kind = "at_velocity", velocity_m_s = 1850.0 }
action  = { kind = "stop", label = "burnout" }
"#,
        ))
        .expect("parse");
        let mission = scenario.document.mission.as_ref().expect("mission");
        assert!(matches!(
            mission.events[0].trigger,
            EventTriggerConfig::AtVelocity { velocity_m_s, falling }
                if velocity_m_s.to_bits() == 1850.0_f64.to_bits() && !falling
        ));
    }

    #[test]
    fn parses_at_velocity_falling_trigger() {
        let scenario = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "descent"

[[mission.phases]]
id    = "descent"
label = "descent"

[[mission.events]]
id      = "terminal_velocity"
trigger = { kind = "at_velocity", velocity_m_s = 250.0, falling = true }
action  = { kind = "stop", label = "terminal" }
"#,
        ))
        .expect("parse");
        let mission = scenario.document.mission.as_ref().expect("mission");
        assert!(matches!(
            mission.events[0].trigger,
            EventTriggerConfig::AtVelocity { velocity_m_s, falling }
                if velocity_m_s.to_bits() == 250.0_f64.to_bits() && falling
        ));
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
            crate::ScenarioActionConfig::EngineCommand { id, command } => {
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

    const VALID_STAGE_SEPARATION_SCENARIO: &str =
        include_str!("../tests/fixtures/stage-separation-valid.toml");

    #[test]
    fn validates_jettison_stage_with_matching_multi_body_block() {
        let scenario =
            Scenario::from_toml_str(VALID_STAGE_SEPARATION_SCENARIO).expect("scenario validates");
        let mission = scenario.document.mission.as_ref().expect("mission present");
        assert!(matches!(
            mission.events[0].action,
            crate::ScenarioActionConfig::JettisonStage { ref body } if body == "lower"
        ));
        let multi_body = scenario
            .document
            .multi_body
            .as_ref()
            .expect("multi_body present");
        assert_eq!(multi_body.separations.len(), 1);
        assert_eq!(multi_body.separations[0].lower_body_id, "lower");
    }

    #[test]
    fn accepts_relative_distance_trigger_for_multi_body_lane() {
        let relative_event = r#"
[[mission.events]]
id      = "lower_clear"
trigger = { kind = "at_relative_distance", body = "lower", distance_m = 1.0 }
action  = { kind = "emit_telemetry_marker", tag = "lower_clear" }
once    = true
"#;
        let toml = VALID_STAGE_SEPARATION_SCENARIO.replace(
            "\n[multi_body]\n",
            &format!("{relative_event}\n[multi_body]\n"),
        );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario validates");
        let mission = scenario.document.mission.as_ref().expect("mission present");
        assert!(matches!(
            mission.events[1].trigger,
            crate::EventTriggerConfig::AtRelativeDistance { ref body, distance_m, .. }
                if body == "lower" && (distance_m - 1.0).abs() < f64::EPSILON
        ));
    }

    #[test]
    fn rejects_relative_distance_trigger_unknown_body() {
        let relative_event = r#"
[[mission.events]]
id      = "lower_clear"
trigger = { kind = "at_relative_distance", body = "typo", distance_m = 1.0 }
action  = { kind = "emit_telemetry_marker", tag = "lower_clear" }
once    = true
"#;
        let toml = VALID_STAGE_SEPARATION_SCENARIO.replace(
            "\n[multi_body]\n",
            &format!("{relative_event}\n[multi_body]\n"),
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnknownBodyReference { ref field, ref value }
                if field == "mission.events[1].trigger.body" && value == "typo"),
            "expected UnknownBodyReference, got {err:?}",
        );
    }

    fn with_assembly_resource(fragment: &str) -> String {
        VALID_STAGE_SEPARATION_SCENARIO.replace(
            "\n[environment]\n",
            &format!("\n{fragment}\n[environment]\n"),
        )
    }

    fn assert_missing_multi_body_owner(fragment: &str, expected_field: &str) {
        let toml = with_assembly_resource(fragment);
        assert_missing_multi_body_owner_in(&toml, expected_field);
    }

    fn assert_missing_multi_body_owner_in(toml: &str, expected_field: &str) {
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::IncompatibleAssemblyEntry { ref field, ref reason }
                if field == expected_field && reason.contains("explicit `mounted_to` body")),
            "expected missing mounted_to for {expected_field}, got {err:?}",
        );
    }

    #[test]
    fn rejects_multi_body_resources_without_explicit_owners() {
        let aero = VALID_STAGE_SEPARATION_SCENARIO
            .replace(r#"models = ["gravity"]"#, r#"models = ["gravity", "aero"]"#)
            .replace(
                "\n[telemetry]\n",
                "\n[aero]\ndeck = \"aero.csv\"\n\n[telemetry]\n",
            );
        let err = Scenario::from_toml_str(&aero).unwrap_err();
        assert!(
            matches!(err, ScenarioError::IncompatibleAssemblyEntry { ref field, ref reason }
                if field == "aero.mounted_to" && reason.contains("explicit `mounted_to` body")),
            "expected missing aero mounted_to, got {err:?}",
        );

        let motor = VALID_STAGE_SEPARATION_SCENARIO.replace(
            "\n[forces]\n",
            "\n[propulsion.motor]\nfile = \"motor.csv\"\nignite_at_s = 0.0\nvariant = \"solid\"\n\n[forces]\n",
        );
        let err = Scenario::from_toml_str(&motor).unwrap_err();
        assert!(
            matches!(err, ScenarioError::IncompatibleAssemblyEntry { ref field, ref reason }
                if field == "propulsion.motor.mounted_to" && reason.contains("explicit `mounted_to` body")),
            "expected missing motor mounted_to, got {err:?}",
        );

        assert_missing_multi_body_owner(
            r#"[[vehicle.assembly.effectors]]
id               = "delta_e"
kind             = { kind = "linear_actuator", tau_s = 0.05 }
limits           = { min = -0.349, max = 0.349, max_rate_per_s = 5.236, deadband = 0.0, latency_s = 0.0 }
"#,
            "vehicle.assembly.effectors[0].mounted_to",
        );
        let engine = with_assembly_resource(
            r#"[[vehicle.assembly.engines]]
id                 = "engine_a"
kind               = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, -0.5]
limits             = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.1, shutdown_transient_s = 0.1, max_gimbal_rad = 0.087 }
"#)
        .replace(r#"models = ["gravity"]"#, r#"models = ["gravity", "thrust"]"#);
        assert_missing_multi_body_owner_in(&engine, "vehicle.assembly.engines[0].mounted_to");
        assert_missing_multi_body_owner(
            r#"[[vehicle.assembly.recovery]]
id   = "main_chute"
kind = { kind = "parachute_drag", c_d = 1.5, area_inflated_m2 = 2.0 }
"#,
            "vehicle.assembly.recovery[0].mounted_to",
        );
    }

    #[test]
    fn rejects_multi_body_resource_owner_unknown_body() {
        let toml = VALID_STAGE_SEPARATION_SCENARIO
            .replace(r#"models = ["gravity"]"#, r#"models = ["gravity", "aero"]"#)
            .replace(
                "\n[telemetry]\n",
                "\n[aero]\ndeck = \"aero.csv\"\nmounted_to = \"typo\"\n\n[telemetry]\n",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnknownBodyReference { ref field, ref value }
                if field == "aero.mounted_to" && value == "typo"),
            "expected unknown aero owner body, got {err:?}",
        );
    }

    #[test]
    fn rejects_jettison_stage_without_multi_body_block() {
        let toml = VALID_STAGE_SEPARATION_SCENARIO
            .split_once("\n[multi_body]\n")
            .expect("fixture has multi_body block")
            .0
            .to_owned();
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_b, .. }
                if field_b == "multi_body"),
            "expected InconsistentSection requiring multi_body, got {err:?}",
        );
    }

    #[test]
    fn rejects_jettison_stage_unknown_body() {
        let toml = VALID_STAGE_SEPARATION_SCENARIO.replace(r#"body = "lower""#, r#"body = "typo""#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::UnknownBodyReference { ref field, ref value }
                if field == "mission.events[0].action.body" && value == "typo"),
            "expected UnknownBodyReference, got {err:?}",
        );
    }

    #[test]
    fn rejects_jettison_stage_on_point_mass_vehicle() {
        let toml = VALID_STAGE_SEPARATION_SCENARIO
            .replace(r#"kind                                  = "rigid_body""#, r#"kind                                  = "point_mass""#)
            .replace(
                "initial_quaternion_body_to_eci_xyzw   = [0.0, 0.0, 0.0, 1.0]\ninitial_angular_velocity_body_rad_s   = [0.0, 0.0, 0.0]\n",
                "",
            )
            .replace(
                "dry_inertia_body_kg_m2      = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]\n",
                "",
            )
            .replace(
                "dry_inertia_body_kg_m2      = [[0.25, 0.0, 0.0], [0.0, 0.25, 0.0], [0.0, 0.0, 0.25]]\n",
                "",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::IncompatibleAssemblyEntry { ref field, .. }
                if field == "mission.events.action.kind = \"jettison_stage\""),
            "expected IncompatibleAssemblyEntry, got {err:?}",
        );
    }

    #[test]
    fn rejects_jettison_stage_without_matching_multi_body_body() {
        let toml = VALID_STAGE_SEPARATION_SCENARIO.replace(
            r#"lower_body_id             = "lower""#,
            r#"lower_body_id             = "upper""#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, .. }
                if field_a == "multi_body.separation[0].upper_body_id"),
            "expected InconsistentSection from invalid multi_body body pair, got {err:?}",
        );
    }

    #[test]
    fn rejects_multi_body_without_matching_jettison_stage() {
        let toml =
            VALID_STAGE_SEPARATION_SCENARIO.replace(r#"body = "lower""#, r#"body = "upper""#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, ref field_b, .. }
                if field_a == "mission.events[0].action.body" && field_b == "multi_body.separation"),
            "expected InconsistentSection for unmatched body, got {err:?}",
        );
    }

    #[test]
    fn rejects_duplicate_jettisoned_body() {
        let duplicate_event = r#"
[[mission.events]]
id      = "stage_separation_again"
trigger = { kind = "at_time", time_s = 0.3 }
action  = { kind = "jettison_stage", body = "lower" }
once    = true
"#;
        let toml = VALID_STAGE_SEPARATION_SCENARIO.replace(
            "\n[multi_body]\n",
            &format!("{duplicate_event}\n[multi_body]\n"),
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::DuplicateValue { ref field, ref value }
                if field == "mission.events[1].action.body" && value == "lower"),
            "expected DuplicateValue for duplicate jettison, got {err:?}",
        );
    }

    #[test]
    fn rejects_separation_momentum_mismatch() {
        let toml = VALID_STAGE_SEPARATION_SCENARIO.replace("[0.0, 0.0, -1.0]", "[0.0, 0.0, -0.5]");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::SeparationMomentumMismatch { ref field, .. }
                if field == "multi_body.separation[0]"),
            "expected SeparationMomentumMismatch, got {err:?}",
        );
    }

    #[test]
    fn rejects_deploy_recovery_action_missing_fields() {
        // `deploy_recovery` requires `id` and `command`
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
        // Scenario-script effector override actions are wired.
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
            crate::ScenarioActionConfig::EffectorOverride { id, command }
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
    fn rejects_at_velocity_non_positive_threshold() {
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "at_velocity", velocity_m_s = 0.0 }
action  = { kind = "stop", label = "burnout" }
"#,
        ))
        .unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. }
                if field == "mission.events[0].trigger.velocity_m_s"),
            "expected InvalidNumber for velocity_m_s, got {err:?}",
        );
    }

    #[test]
    fn accepts_dynamic_pressure_trigger_kind() {
        let scenario = Scenario::from_toml_str(&with_mission(
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
        .expect("dynamic pressure trigger should parse");
        assert!(matches!(
            &scenario.document.mission.as_ref().unwrap().events[0].trigger,
            EventTriggerConfig::AtDynamicPressure { pressure_pa, falling: false }
                if *pressure_pa == 0.0
        ));
    }

    #[test]
    fn rejects_negative_dynamic_pressure_trigger_threshold() {
        let err = Scenario::from_toml_str(&with_mission(
            r#"
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "ascent"

[[mission.events]]
id      = "evt"
trigger = { kind = "at_dynamic_pressure", pressure_pa = -1.0, falling = false }
action  = { kind = "stop", label = "max-q" }
"#,
        ))
        .unwrap_err();
        assert!(matches!(
            err,
            ScenarioError::InvalidNumber { ref field, .. }
                if field == "mission.events[0].trigger.pressure_pa"
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
        // The mission block is optional; scenarios without one
        // must continue to parse identically.
        let scenario = Scenario::from_toml_str(MINIMAL).expect("parse");
        assert!(scenario.document.mission.is_none());
    }

    // -----------------------------------------------------------------
    // `[vehicle.assembly]` block parser tests
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
        // v2 contract: every scenario has a non-empty
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
    #[ignore = "v1 flat fields were retired; cross-consistency check is now structurally impossible"]
    fn rejects_mass_mismatch_between_flat_and_assembly() {
        let err = Scenario::from_toml_str(ASSEMBLY_MASS_MISMATCH).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { .. }),
            "expected InconsistentSection, got {err:?}",
        );
    }

    #[test]
    #[ignore = "v1 flat fields were retired; relative-tolerance cross-consistency is now structurally impossible"]
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
    #[ignore = "v1 flat fields were retired; rigid inertia cross-consistency is now structurally impossible"]
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
    // Recovery
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
    // Sensors
    // -----------------------------------------------------------------

    #[test]
    fn parses_all_supported_sensor_kinds() {
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
    fn rejects_file_backed_sensor_without_file() {
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

    // Closed-loop scenario fixture.
    const CLOSED_LOOP_ATTITUDE_HOLD: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
    ));

    const ASCENT_REFERENCE_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/ascent-reference-valid.toml"
    ));

    const COAST_FOOTPRINT_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/coast-footprint-valid.toml"
    ));

    const COAST_FOOTPRINT_MC_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/coast-footprint-mc-valid.toml"
    ));

    const ENTRY_PROFILE_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/entry-profile-valid.toml"
    ));

    #[test]
    fn parses_ascent_reference_pitch_program() {
        let scenario = Scenario::from_toml_str(ASCENT_REFERENCE_SCENARIO).unwrap();
        let fc = scenario.document.fc.as_ref().unwrap();
        assert_eq!(fc.guidance, crate::FcGuidanceKind::AscentReference);
        let ascent = fc.ascent_reference.as_ref().unwrap();
        assert_eq!(ascent.method, crate::FcAscentReferenceMethod::PitchProgram);
        assert_eq!(ascent.schedule_s.as_ref().unwrap().len(), 3);
    }

    #[test]
    fn rejects_ascent_reference_without_monotonic_pitch_schedule() {
        let toml = ASCENT_REFERENCE_SCENARIO.replace(
            "schedule_s = [0.0, 10.0, 30.0]",
            "schedule_s = [0.0, 10.0, 10.0]",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InvalidNumber { ref field, .. } if field == "fc.ascent_reference.schedule_s[2]"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_ascent_reference_on_point_mass_vehicle() {
        let toml = ASCENT_REFERENCE_SCENARIO
            .replace(r#"kind = "rigid_body""#, r#"kind = "point_mass""#)
            .replace(
                "initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]\n",
                "",
            )
            .replace(
                "initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]\n",
                "",
            )
            .replace(
                "dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]\n",
                "",
            );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::IncompatibleAssemblyEntry { ref field, .. } if field == "fc.ascent_reference"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_ascent_reference_under_v2_schema() {
        let toml =
            ASCENT_REFERENCE_SCENARIO.replace("openbmp.scenario = 3", "openbmp.scenario = 2");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::SchemaVersionFieldReserved { ref field, .. } if field == "fc.guidance = \"ascent_reference\""),
            "got {err:?}",
        );
    }

    #[test]
    fn explicit_ascent_reference_remains_reserved() {
        let toml = ASCENT_REFERENCE_SCENARIO.replace(
            r#"method = "pitch_program""#,
            r#"method = "explicit_reference""#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::ElementNotYetSupported { ref field, .. } if field == "fc.ascent_reference.method = \"explicit_reference\""),
            "got {err:?}",
        );
    }

    #[test]
    fn parses_coast_landing_footprint_config() {
        let scenario = Scenario::from_toml_str(COAST_FOOTPRINT_SCENARIO).unwrap();
        let footprint = scenario.document.landing_footprint.as_ref().unwrap();
        assert_eq!(
            footprint.method,
            crate::LandingFootprintMethod::ConstantGravity
        );
        assert!(footprint.cull_altitude_m.abs() < f64::EPSILON);
        assert!(footprint.dispersion.is_some());
    }

    #[test]
    fn parses_coast_landing_footprint_monte_carlo_config() {
        let scenario = Scenario::from_toml_str(COAST_FOOTPRINT_MC_SCENARIO).unwrap();
        let footprint = scenario.document.landing_footprint.as_ref().unwrap();
        let monte_carlo = footprint.monte_carlo.as_ref().unwrap();
        assert_eq!(monte_carlo.samples, 32);
        assert_eq!(monte_carlo.confidence_levels, vec![0.5, 0.9, 0.99]);
        assert!(monte_carlo.wind.is_some());
        assert!(monte_carlo.ballistic_coefficient.is_some());
        assert!(monte_carlo.burnout_state.is_some());

        let paths = scenario.resolved_paths();
        assert!(paths.contains_key("landing_footprint.monte_carlo.output.samples_csv"));
        assert!(paths.contains_key("landing_footprint.monte_carlo.output.samples_parquet"));
        assert!(paths.contains_key("landing_footprint.monte_carlo.output.summary_toml"));
    }

    #[test]
    fn rejects_landing_footprint_monte_carlo_without_uncertainty_source() {
        let toml = COAST_FOOTPRINT_MC_SCENARIO
            .split("[landing_footprint.monte_carlo.wind]")
            .next()
            .unwrap();
        let err = Scenario::from_toml_str(toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, ref field_b, .. }
                if field_a == "landing_footprint.monte_carlo"
                    && field_b == "landing_footprint.monte_carlo.uncertainty_source"),
            "got {err:?}",
        );
    }

    #[test]
    fn parses_j2_and_egm2008_landing_footprint_methods() {
        let j2_toml = COAST_FOOTPRINT_SCENARIO
            .replace(r#"gravity = "constant""#, r#"gravity = "j2""#)
            .replace(
                "gravity_m_s2 = 9.80665",
                "mu_m3_s2 = 398600441800000.0\nr_e_m = 7000000.0",
            )
            .replace(r#"method = "constant_gravity""#, r#"method = "j2""#);
        let scenario = Scenario::from_toml_str(&j2_toml).unwrap();
        let footprint = scenario.document.landing_footprint.as_ref().unwrap();
        assert_eq!(footprint.method, crate::LandingFootprintMethod::J2);

        let egm_toml = COAST_FOOTPRINT_SCENARIO
            .replace(r#"gravity = "constant""#, r#"gravity = "egm2008""#)
            .replace("gravity_m_s2 = 9.80665\n", "")
            .replace(r#"method = "constant_gravity""#, r#"method = "egm2008""#);
        let scenario = Scenario::from_toml_str(&egm_toml).unwrap();
        let footprint = scenario.document.landing_footprint.as_ref().unwrap();
        assert_eq!(footprint.method, crate::LandingFootprintMethod::Egm2008);
    }

    #[test]
    fn rejects_landing_footprint_under_v2_schema() {
        let toml = COAST_FOOTPRINT_SCENARIO.replace("openbmp.scenario = 3", "openbmp.scenario = 2");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::SchemaVersionFieldReserved { ref field, .. } if field == "landing_footprint"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_landing_footprint_without_coast_phase() {
        let toml = COAST_FOOTPRINT_SCENARIO
            .replace(r#"id = "coast""#, r#"id = "ascent""#)
            .replace(r#"id = "ballistic_descent""#, r#"id = "descent""#)
            .replace(r#"initial_phase = "coast""#, r#"initial_phase = "ascent""#)
            .replace(r#"from = "coast""#, r#"from = "ascent""#)
            .replace(r#"from = "ballistic_descent""#, r#"from = "descent""#)
            .replace(r#"to = "ballistic_descent""#, r#"to = "descent""#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, ref field_b, .. } if field_a == "landing_footprint" && field_b == "mission.phase_or_state.id"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_landing_footprint_geodetic_output_without_origin() {
        let toml =
            COAST_FOOTPRINT_SCENARIO.replace("include_geodetic = false", "include_geodetic = true");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, ref field_b, .. } if field_a == "landing_footprint.include_geodetic" && field_b == "frames.local_origin"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_constant_gravity_footprint_with_nonconstant_gravity() {
        let toml = COAST_FOOTPRINT_SCENARIO
            .replace(r#"gravity = "constant""#, r#"gravity = "point_mass""#)
            .replace("gravity_m_s2 = 9.80665", "mu_m3_s2 = 398600441800000.0");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, ref field_b, .. } if field_a == "landing_footprint.method" && field_b == "environment.gravity"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_j2_footprint_with_non_j2_gravity() {
        let toml =
            COAST_FOOTPRINT_SCENARIO.replace(r#"method = "constant_gravity""#, r#"method = "j2""#);
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, ref field_b, .. } if field_a == "landing_footprint.method" && field_b == "environment.gravity"),
            "got {err:?}",
        );
    }

    #[test]
    fn parses_entry_profile_config() {
        let scenario = Scenario::from_toml_str(ENTRY_PROFILE_SCENARIO).unwrap();
        let entry = scenario.document.entry_profile.as_ref().unwrap();
        assert_eq!(entry.mode, crate::EntryProfileMode::Ballistic);
        assert!((entry.entry_interface_altitude_m - 122_000.0).abs() < f64::EPSILON);
        assert!((entry.final_descent_altitude_m.unwrap() - 5_000.0).abs() < f64::EPSILON);
        assert_eq!(entry.nose_radius_m, Some(1.0));
    }

    #[test]
    fn rejects_entry_profile_under_v2_schema() {
        let toml = ENTRY_PROFILE_SCENARIO
            .replace("openbmp.scenario = 3", "openbmp.scenario = 2")
            .replace(
                r#"atmosphere = "piecewise_exponential""#,
                r#"atmosphere = "us_standard_1976""#,
            )
            .replace("[atmosphere]\nkind = \"piecewise_exponential\"\n\n", "");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::SchemaVersionFieldReserved { ref field, .. } if field == "entry_profile"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_entry_profile_without_entry_interface_handoff() {
        let toml = ENTRY_PROFILE_SCENARIO.replace(
            r#"action = { kind = "enter_phase", phase = "entry_interface" }"#,
            r#"action = { kind = "enter_phase", phase = "final_descent" }"#,
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, ref field_b, .. } if field_a == "entry_profile.entry_interface_altitude_m" && field_b == "mission.events"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_high_entry_profile_with_ussa76_atmosphere() {
        let toml = ENTRY_PROFILE_SCENARIO
            .replace(
                r#"atmosphere = "piecewise_exponential""#,
                r#"atmosphere = "us_standard_1976""#,
            )
            .replace("[atmosphere]\nkind = \"piecewise_exponential\"\n\n", "");
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::InconsistentSection { ref field_a, ref field_b, .. } if field_a == "entry_profile.entry_interface_altitude_m" && field_b == "environment.atmosphere"),
            "got {err:?}",
        );
    }

    #[test]
    fn rejects_lifting_entry_profile_on_point_mass_vehicle() {
        let toml = ENTRY_PROFILE_SCENARIO.replace(
            "[entry_profile]\nmode = \"ballistic\"\nentry_interface_altitude_m = 122000.0\nfinal_descent_altitude_m = 5000.0\nnose_radius_m = 1.0",
            "[entry_profile]\nmode = \"lifting\"\nentry_interface_altitude_m = 122000.0\nfinal_descent_altitude_m = 5000.0\nlift_to_drag_ratio = 0.3\n\n[entry_profile.corridor]\nmax_heat_rate_w_m2 = 1000000.0\nmax_load_factor_g = 8.0\nflight_path_angle_band_rad = 0.2\nmax_bank_rad = 1.2",
        );
        let err = Scenario::from_toml_str(&toml).unwrap_err();
        assert!(
            matches!(err, ScenarioError::IncompatibleAssemblyEntry { ref field, .. } if field == "entry_profile.mode = \"lifting\""),
            "got {err:?}",
        );
    }

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
