//! Embedded scenario and data assets for hosts without a filesystem.
//!
//! The bytes are embedded from the canonical repository files at compile
//! time (`include_str!` / `include_bytes!` against `scenarios/` and
//! `data/`), so the web demo cannot drift from the scenarios the native
//! CLI and CI validate — there is one source of truth and the embedded
//! copy *is* it. `tests/native.rs` additionally pins the embedded
//! resolution byte-for-byte against the filesystem resolution.

use std::collections::BTreeMap;
use std::io;
use std::path::Path;

use openbmp_scenario::{ResolvedFile, Scenario, ScenarioError};

/// One embedded, runnable scenario with every file it references.
#[derive(Clone, Copy, Debug)]
pub struct EmbeddedScenario {
    /// Stable lookup key for the host API.
    pub key: &'static str,
    /// Human-readable title for scenario pickers.
    pub title: &'static str,
    /// Scenario TOML source (the repository file, embedded verbatim).
    pub toml: &'static str,
    /// Referenced files keyed by the path string as written in the
    /// scenario (the path `Scenario::resolve_path` yields when the
    /// scenario is parsed without a source directory).
    pub files: &'static [(&'static str, &'static [u8])],
}

/// Shared `data/` files referenced by the Phalcon-9 scenarios, keyed by
/// the scenario-relative path strings they are referenced under.
const PHALCON9_DATA_FILES: &[(&str, &[u8])] = &[
    (
        "../../data/aero/phalcon9-drag.toml",
        include_bytes!("../../../data/aero/phalcon9-drag.toml"),
    ),
    (
        "../../data/sensors/imu-tactical.toml",
        include_bytes!("../../../data/sensors/imu-tactical.toml"),
    ),
    (
        "../../data/sensors/gnss-textbook.toml",
        include_bytes!("../../../data/sensors/gnss-textbook.toml"),
    ),
    (
        "../../data/sensors/star-tracker-textbook.toml",
        include_bytes!("../../../data/sensors/star-tracker-textbook.toml"),
    ),
];

/// Every scenario the web host can run.
pub const SCENARIOS: &[EmbeddedScenario] = &[
    EmbeddedScenario {
        key: "phalcon9-orbit-boostback",
        title: "Phalcon-9: ascent to orbit + booster boostback",
        toml: include_str!("../../../scenarios/phalcon9/phalcon9-orbit-boostback.toml"),
        files: PHALCON9_DATA_FILES,
    },
    EmbeddedScenario {
        key: "phalcon9-orbit",
        title: "Phalcon-9: two-stage ascent to orbit",
        toml: include_str!("../../../scenarios/phalcon9/phalcon9-orbit.toml"),
        files: PHALCON9_DATA_FILES,
    },
    EmbeddedScenario {
        key: "phalcon9-orbit-staged",
        title: "Phalcon-9: full sequence + fairing & payload deploy",
        toml: include_str!("../../../scenarios/phalcon9/phalcon9-orbit-staged.toml"),
        files: PHALCON9_DATA_FILES,
    },
    EmbeddedScenario {
        key: "phalcon9-orbit-slosh-rcs",
        title: "Phalcon-9: propellant slosh + RCS attitude hold",
        toml: include_str!("../../../scenarios/phalcon9/phalcon9-orbit-slosh-rcs.toml"),
        files: PHALCON9_DATA_FILES,
    },
];

/// Look up an embedded scenario by key.
pub fn find(key: &str) -> Option<&'static EmbeddedScenario> {
    SCENARIOS.iter().find(|scenario| scenario.key == key)
}

impl EmbeddedScenario {
    /// Parse the scenario and resolve its referenced files from the
    /// embedded asset table, with the same digest computation and pin
    /// verification as a filesystem load.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for parse or lint failures, a missing
    /// embedded asset (fail-closed, surfaced exactly like a missing
    /// file), or a SHA-256 pin mismatch.
    pub fn load(&self) -> Result<(Scenario, BTreeMap<String, ResolvedFile>), ScenarioError> {
        let scenario = Scenario::from_toml_str(self.toml)?;
        let files = self.files;
        let read = move |path: &Path| -> io::Result<Vec<u8>> {
            let wanted = path.to_string_lossy();
            files
                .iter()
                .find(|(key, _)| *key == wanted)
                .map(|(_, bytes)| bytes.to_vec())
                .ok_or_else(|| {
                    io::Error::new(
                        io::ErrorKind::NotFound,
                        format!("no embedded asset for `{wanted}`"),
                    )
                })
        };
        let resolved = scenario.resolved_files_with_reader(&read)?;
        Ok((scenario, resolved))
    }
}
