//! CI tripwire for rejected operational vocabulary.
//!
//! Scans the workspace's `*.rs` and `*.md` files for the rejected
//! mission-state vocabulary documented in
//! `docs/mission-states-vocabulary.md § Rejected Vocabulary`. The
//! tripwire is a string-grep — without enforcement, terms like
//! `Terminal`, `Endgame`, `Midcourse`, or `Engagement` (as
//! mission-state names) can creep back into the codebase silently.
//!
//! The tripwire deliberately:
//!
//! - Skips comment lines (doc-comments and `//`-comments). Project
//!   prose may *discuss* the rejected terms when explaining the
//!   safety boundary; only actual code identifiers / type names
//!   matter.
//! - Skips the tripwire's own pattern declarations.
//! - Skips test files that explicitly assert the rejection (the
//!   scenario lint tests reference forbidden terms by design).
//! - Skips `docs/mission-states-vocabulary.md` itself (the canonical
//!   rejection table).
//! - Skips `docs/safety-boundaries.md` (the workspace-wide naming
//!   policy that enumerates the rejected terms).
//! - Skips `docs/mission-graph-architecture.md` (it discusses the
//!   mission state machine, including the rejected terms).
//!
//! The tripwire fires on case-insensitive substring match against
//! source identifiers. New CI gates can ratchet it up to a hard fail.

use std::path::{Path, PathBuf};

/// Mission-vocabulary patterns that fail review when found
/// outside the documented allowlist.
///
/// Each pattern is matched **case-insensitively** as a substring of
/// the source line. Patterns intentionally avoid generic physics
/// terms (`terminal_velocity` is fine; only `terminal` *as a state
/// name* is rejected — the scan uses identifier-shaped neighbours
/// to filter false positives).
pub const REJECTED_MISSION_VOCABULARY: &[&str] = &[
    // From mission-states-vocabulary.md § Rejected Vocabulary.
    "Endgame",
    "midcourse",
    "Midcourse",
    "MIDCOURSE",
    "Pen_Aid",
    "PenAid",
    "BlackoutEvasion",
    // Operational-vocabulary mission states (those that don't
    // double as physics terms — `terminal_velocity` is allowed, the
    // rejection targets `TerminalDescent` / `TerminalPhase`).
    "TerminalDescent",
    "TerminalPhase",
    "TerminalMode",
    "TerminalHoming",
    "TerminalGuidance",
    "TerminalWaypoint",
];

/// Files / path-suffix patterns intentionally allowed to mention
/// rejected vocabulary because their *purpose* is to document the
/// rejection.
const ALLOWLIST_PATHS: &[&str] = &[
    "docs/mission-states-vocabulary.md",
    "docs/safety-boundaries.md",
    "docs/mission-graph-architecture.md",
    "docs/scenario-format.md",
    "docs/design-concept.md",
    "docs/software-architecture.md",
    "docs/real-rocket-integration.md",
    "docs/README.md",
    "crates/openbmp-scenario/src/lint.rs",
    "crates/openbmp-testkit/src/mission_vocabulary_lints.rs",
];

/// Outcome of a single tripwire scan.
#[derive(Clone, Debug, Default)]
pub struct VocabularyFindings {
    /// `(path, line_no, pattern)` triples for each match.
    pub matches: Vec<(PathBuf, usize, &'static str)>,
}

impl VocabularyFindings {
    /// `true` if the scan returned any rejected-vocabulary hits.
    #[must_use]
    pub fn has_violations(&self) -> bool {
        !self.matches.is_empty()
    }
}

/// Scan `root` recursively for the rejected mission vocabulary in
/// `*.rs` and `*.md` files. Skips paths matching `ALLOWLIST_PATHS`.
///
/// # Errors
///
/// Returns `Err` if the directory cannot be read.
pub fn scan_workspace(root: &Path) -> std::io::Result<VocabularyFindings> {
    let mut findings = VocabularyFindings::default();
    visit(root, root, &mut findings)?;
    Ok(findings)
}

fn visit(root: &Path, dir: &Path, findings: &mut VocabularyFindings) -> std::io::Result<()> {
    if !dir.is_dir() {
        return Ok(());
    }
    let mut entries: Vec<_> = std::fs::read_dir(dir)?.collect::<Result<Vec<_>, _>>()?;
    entries.sort_by_key(std::fs::DirEntry::file_name);
    for entry in entries {
        let path = entry.path();
        let name = path.file_name().and_then(std::ffi::OsStr::to_str);
        // Skip target / .git / similar.
        if matches!(name, Some("target" | ".git" | "node_modules")) {
            continue;
        }
        if path.is_dir() {
            visit(root, &path, findings)?;
        } else if path.extension().is_some_and(|e| e == "rs" || e == "md") {
            let rel = path
                .strip_prefix(root)
                .unwrap_or(&path)
                .to_string_lossy()
                .replace('\\', "/");
            if ALLOWLIST_PATHS.iter().any(|allowed| rel == *allowed) {
                continue;
            }
            scan_file(&path, findings)?;
        }
    }
    Ok(())
}

fn scan_file(path: &Path, findings: &mut VocabularyFindings) -> std::io::Result<()> {
    let contents = std::fs::read_to_string(path)?;
    for (line_idx, line) in contents.lines().enumerate() {
        let trimmed = line.trim_start();
        // Skip pure-comment lines and markdown blockquote prose.
        if trimmed.starts_with("//") || trimmed.starts_with('>') {
            continue;
        }
        // Skip the tripwire's own pattern table.
        if line.contains("REJECTED_MISSION_VOCABULARY") {
            continue;
        }
        for &pattern in REJECTED_MISSION_VOCABULARY {
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
        let here = env!("CARGO_MANIFEST_DIR");
        Path::new(here)
            .parent()
            .and_then(Path::parent)
            .expect("workspace root")
            .to_path_buf()
    }

    #[test]
    fn workspace_has_no_rejected_mission_vocabulary() {
        let root = project_root();
        let findings = scan_workspace(&root).expect("scan");
        if findings.has_violations() {
            let lines: Vec<String> = findings
                .matches
                .iter()
                .map(|(p, line, pat)| {
                    let rel = p.strip_prefix(&root).unwrap_or(p);
                    format!("  {}:{} ({})", rel.display(), line, pat)
                })
                .collect();
            panic!("mission-vocabulary tripwire fired:\n{}", lines.join("\n"));
        }
    }
}
