//! Tripwire helpers for `openbmp-fc` structural invariants.
//!
//! `openbmp-fc` is the simulator-local autopilot. It enforces a
//! lockstep-clock contract: every time read goes through the injected
//! [`Clock`](<https://docs.rs/openbmp-fc/latest/openbmp_fc/clock/trait.Clock.html>)
//! trait. Wall-clock APIs (`std::time::Instant::now`,
//! `std::time::SystemTime::now`) are banned crate-wide.
//!
//! This module ships tripwires: string-greps over the controller's
//! source tree that fail CI if forbidden patterns reappear.
//! The contract is structural — without enforcement it rots silently
//! when a deeply-nested utility quietly imports `std::time` again.
//! The same lint module also locks a few known no-hot-path-allocation
//! regressions that are easy to reintroduce during refactors.

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

/// Returns the subsection of `source` starting at `start_marker` and
/// ending before `end_marker`.
#[must_use]
pub fn source_between<'a>(
    source: &'a str,
    start_marker: &str,
    end_marker: &str,
) -> Option<&'a str> {
    let start = source.find(start_marker)?;
    let tail = &source[start..];
    let end = tail.find(end_marker)?;
    Some(&tail[..end])
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

    #[test]
    fn scheduler_dispatch_uses_cached_order_not_per_tick_sort_allocation() {
        let scheduler =
            std::fs::read_to_string(project_root().join("crates/openbmp-fc/src/scheduler.rs"))
                .expect("read scheduler.rs");
        let dispatch = source_between(
            &scheduler,
            "    pub fn dispatch(",
            "    /// Returns descriptors for every registered job",
        )
        .expect("scheduler dispatch section");

        for forbidden in [".collect()", "sort_by_key", "Vec::new", "Vec<("] {
            assert!(
                !dispatch.contains(forbidden),
                "scheduler dispatch must use its cached registration-time order; \
                 found `{forbidden}` in dispatch hot path"
            );
        }
        assert!(
            dispatch.contains("self.dispatch_order"),
            "scheduler dispatch should iterate the cached dispatch_order"
        );
    }

    #[test]
    fn commander_run_does_not_clone_binding_table_per_tick() {
        let commander =
            std::fs::read_to_string(project_root().join("crates/openbmp-fc/src/commander.rs"))
                .expect("read commander.rs");
        let run = source_between(
            &commander,
            "    fn run(&mut self, ctx: &JobContext<'_>)",
            "        self.previous_scalars = Some(eval_state.current);",
        )
        .expect("commander run section");

        for forbidden in [
            "bindings_snapshot",
            "self.bindings.clone()",
            "bindings.clone()",
        ] {
            assert!(
                !run.contains(forbidden),
                "commander tick must not clone the full mission binding table; \
                 found `{forbidden}` in run hot path"
            );
        }
    }

    #[test]
    fn voted_sensor_ingest_reuses_scratch_buffers_per_tick() {
        let ingest =
            std::fs::read_to_string(project_root().join("crates/openbmp-fc/src/sensor_ingest.rs"))
                .expect("read sensor_ingest.rs");

        let sections = [
            (
                "barometer",
                "impl<S, V> Job for VotedBarometerIngest",
                "/// Voted ingest job for redundant GNSS lanes.",
            ),
            (
                "gnss",
                "impl<S, V> Job for VotedGnssIngest",
                "/// Voted ingest job for redundant magnetometer lanes.",
            ),
            (
                "magnetometer",
                "impl<S, V> Job for VotedMagnetometerIngest",
                "",
            ),
        ];

        for (name, start, end) in sections {
            let section = if end.is_empty() {
                let start_idx = ingest.find(start).expect("voted sensor ingest job start");
                &ingest[start_idx..]
            } else {
                source_between(&ingest, start, end).expect("voted sensor ingest job section")
            };
            assert!(
                !section.contains("Vec::with_capacity"),
                "{name} voted ingest must allocate scratch buffers at construction, not per tick"
            );
            assert!(
                section.contains(".clear()"),
                "{name} voted ingest should clear and reuse scratch buffers each tick"
            );
        }
    }
}
