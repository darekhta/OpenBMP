//! Tripwire helpers for `openbmp-fc` structural invariants.
//!
//! `openbmp-fc` is the simulator-local autopilot. It enforces a
//! lockstep-clock contract: every time read goes through the injected
//! [`Clock`](<https://docs.rs/openbmp-fc/latest/openbmp_fc/clock/trait.Clock.html>)
//! trait. Wall-clock APIs (`std::time::Instant::now`,
//! `std::time::SystemTime::now`) are banned crate-wide.
//!
//! This module ships a tripwire: a string-grep over the controller's
//! source tree that fails CI if any forbidden pattern reappears.
//! The contract is structural — without enforcement it rots silently
//! when a deeply-nested utility quietly imports `std::time` again.

use std::path::{Path, PathBuf};

/// Patterns considered banned in `openbmp-fc/src/` and `tests/`.
pub const BANNED_PATTERNS: &[&str] = &[
    "Instant::now",
    "SystemTime::now",
    "std::time::Instant",
    "std::time::SystemTime",
];

/// Outcome of a single tripwire scan: every banned pattern is paired
/// with the `file:line` where it appeared.
#[derive(Clone, Debug, Default)]
pub struct TripwireFindings {
    /// `(path, line_no, pattern)` triples for each match.
    pub matches: Vec<(PathBuf, usize, &'static str)>,
}

impl TripwireFindings {
    /// `true` if the scan returned any banned-pattern hits.
    #[must_use]
    pub fn has_violations(&self) -> bool {
        !self.matches.is_empty()
    }
}

/// Scans the given root directory for any banned wall-clock pattern.
/// Recurses into subdirectories; reads `*.rs` files only.
///
/// # Errors
///
/// Returns `Err` if the directory cannot be read.
pub fn scan_tree(root: &Path) -> std::io::Result<TripwireFindings> {
    let mut findings = TripwireFindings::default();
    visit_dir(root, &mut findings)?;
    Ok(findings)
}

fn visit_dir(dir: &Path, findings: &mut TripwireFindings) -> std::io::Result<()> {
    if !dir.is_dir() {
        return Ok(());
    }
    let mut entries: Vec<_> = std::fs::read_dir(dir)?.collect::<Result<Vec<_>, _>>()?;
    entries.sort_by_key(std::fs::DirEntry::file_name);
    for entry in entries {
        let path = entry.path();
        if path.is_dir() {
            visit_dir(&path, findings)?;
        } else if path.extension().is_some_and(|e| e == "rs") {
            scan_file(&path, findings)?;
        }
    }
    Ok(())
}

fn scan_file(path: &Path, findings: &mut TripwireFindings) -> std::io::Result<()> {
    let contents = std::fs::read_to_string(path)?;
    for (line_idx, line) in contents.lines().enumerate() {
        let trimmed = line.trim_start();
        // Skip pure comment lines — including doc-comments. They're
        // text, not code paths. Linting them produces false positives
        // when a module docstring mentions the banned APIs by name to
        // explain *why* they're banned.
        if trimmed.starts_with("//") {
            continue;
        }
        // Skip the tripwire's own banned-pattern declarations.
        if line.contains("BANNED_PATTERNS") {
            continue;
        }
        for &pattern in BANNED_PATTERNS {
            if line.contains(pattern) {
                findings
                    .matches
                    .push((path.to_path_buf(), line_idx + 1, pattern));
            }
        }
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;

    fn project_root() -> PathBuf {
        // testkit lives at crates/openbmp-testkit; the project root
        // is two parents up from the manifest.
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        manifest
            .parent()
            .expect("crates/")
            .parent()
            .expect("workspace root")
            .to_path_buf()
    }

    #[test]
    fn openbmp_fc_has_no_wall_clock_calls() {
        let fc_src = project_root().join("crates/openbmp-fc/src");
        let findings = scan_tree(&fc_src).expect("scan openbmp-fc/src");
        if findings.has_violations() {
            for (p, line, pat) in &findings.matches {
                eprintln!("{}:{} matches banned pattern {}", p.display(), line, pat);
            }
            panic!(
                "lockstep-clock contract violated: openbmp-fc must not reach for \
                 std::time wall-clock APIs"
            );
        }
    }

    #[test]
    fn openbmp_fc_tests_have_no_wall_clock_calls() {
        let fc_tests = project_root().join("crates/openbmp-fc/tests");
        let findings = scan_tree(&fc_tests).expect("scan openbmp-fc/tests");
        if findings.has_violations() {
            for (p, line, pat) in &findings.matches {
                eprintln!("{}:{} matches banned pattern {}", p.display(), line, pat);
            }
            panic!(
                "lockstep-clock contract violated: openbmp-fc tests must not \
                 reach for std::time wall-clock APIs"
            );
        }
    }

    #[test]
    fn openbmp_runner_fc_bridge_has_no_wall_clock_calls() {
        let bridge = project_root().join("crates/openbmp-runner/src/fc_bridge.rs");
        let mut findings = TripwireFindings::default();
        scan_file(&bridge, &mut findings).expect("scan openbmp-runner fc_bridge");
        if findings.has_violations() {
            for (p, line, pat) in &findings.matches {
                eprintln!("{}:{} matches banned pattern {}", p.display(), line, pat);
            }
            panic!(
                "lockstep-clock contract violated: openbmp-runner fc_bridge must not \
                 reach for std::time wall-clock APIs"
            );
        }
    }
}
