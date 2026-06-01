//! Dual-use reachability tripwires for forward-only trajectory code.
//!
//! The FC dependency graph tripwire proves `openbmp-fc` does not pull
//! simulator/tooling crates. It cannot prove the M9 property by itself,
//! because `openbmp-fc` legitimately depends on `openbmp-physics` for
//! guidance and estimator math. This test adds the separate assertion:
//! the offline footprint / free-flight propagator symbols must not
//! appear in FC control-loop sources or scenario-consumer sources.

#![allow(clippy::expect_used, clippy::panic)]

use std::fs;
use std::path::{Path, PathBuf};

const PROHIBITED_FC_SYMBOLS: &[&str] = &[
    "BallisticState",
    "FootprintEnvironment",
    "FootprintMonteCarloInput",
    "FootprintSampleInput",
    "RangeSafetyFootprint",
    "landing_footprint",
];

const FREE_FLIGHT_STATE_SYMBOLS: &[&str] = &[
    "BallisticState",
    "FootprintEnvironment",
    "FootprintMonteCarloInput",
    "FootprintSampleInput",
    "RangeSafetyFootprint",
];

const FREE_FLIGHT_SCAN_ROOTS: &[&str] = &[
    "crates/openbmp-fc/src",
    "crates/openbmp-scenario/src",
    "crates/openbmp-runner/src",
    "crates/openbmp-cli/src",
    "crates/openbmp-sim/src",
    "crates/openbmp-bridge/src",
    "crates/openbmp-physics/src",
];

const ALLOWED_FREE_FLIGHT_CONSUMERS: &[&str] = &[
    "crates/openbmp-physics/src/profile.rs",
    "crates/openbmp-runner/src/footprint.rs",
];

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root must exist")
}

#[test]
fn openbmp_fc_control_loop_sources_do_not_reference_offline_footprint_symbols() {
    let root = workspace_root();
    let files = rust_files_under(&root.join("crates/openbmp-fc/src"));
    let violations = symbol_violations(&root, &files, PROHIBITED_FC_SYMBOLS, &[]);

    assert!(
        violations.is_empty(),
        "openbmp-fc control-loop sources must not reference offline footprint/free-flight \
         symbols: {violations:#?}"
    );
}

#[test]
fn free_flight_state_symbols_are_only_consumed_by_offline_footprint_code() {
    let root = workspace_root();
    let mut files = Vec::new();
    for scan_root in FREE_FLIGHT_SCAN_ROOTS {
        files.extend(rust_files_under(&root.join(scan_root)));
    }

    let violations = symbol_violations(
        &root,
        &files,
        FREE_FLIGHT_STATE_SYMBOLS,
        ALLOWED_FREE_FLIGHT_CONSUMERS,
    );

    assert!(
        violations.is_empty(),
        "free-flight / footprint seed symbols must remain confined to the physics definition \
         and offline runner footprint path: {violations:#?}"
    );
}

#[test]
fn tripwire_fixture_catches_prohibited_fc_symbol() {
    let source = "use openbmp_physics::profile::BallisticState;";
    let matches = matching_symbols(source, PROHIBITED_FC_SYMBOLS);
    assert_eq!(matches, vec!["BallisticState"]);
}

fn rust_files_under(root: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    visit_rust_files(root, &mut out);
    out.sort();
    out
}

fn visit_rust_files(path: &Path, out: &mut Vec<PathBuf>) {
    let metadata =
        fs::metadata(path).unwrap_or_else(|err| panic!("failed to stat {}: {err}", path.display()));
    if metadata.is_file() {
        if path.extension().and_then(|ext| ext.to_str()) == Some("rs") {
            out.push(path.to_path_buf());
        }
        return;
    }

    let entries = fs::read_dir(path)
        .unwrap_or_else(|err| panic!("failed to read directory {}: {err}", path.display()));
    for entry in entries {
        let entry = entry.unwrap_or_else(|err| panic!("failed to read dir entry: {err}"));
        visit_rust_files(&entry.path(), out);
    }
}

fn symbol_violations(
    workspace_root: &Path,
    files: &[PathBuf],
    symbols: &[&str],
    allowed_paths: &[&str],
) -> Vec<String> {
    let mut violations = Vec::new();
    for file in files {
        let rel = file
            .strip_prefix(workspace_root)
            .unwrap_or(file)
            .to_string_lossy()
            .replace('\\', "/");
        if allowed_paths.contains(&rel.as_str()) {
            continue;
        }
        let source = fs::read_to_string(file)
            .unwrap_or_else(|err| panic!("failed to read {}: {err}", file.display()));
        for symbol in matching_symbols(&source, symbols) {
            violations.push(format!("{rel}: references prohibited symbol `{symbol}`"));
        }
    }
    violations
}

fn matching_symbols<'a>(source: &str, symbols: &'a [&'a str]) -> Vec<&'a str> {
    let mut matches = Vec::new();
    for &symbol in symbols {
        if source.contains(symbol) {
            matches.push(symbol);
        }
    }
    matches
}
