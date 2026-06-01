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
use std::process::Command;

const PROHIBITED_FC_SYMBOLS: &[&str] = &[
    "BallisticState",
    "DispersionSource",
    "FootprintEnvironment",
    "FootprintMonteCarloInput",
    "FootprintSampleInput",
    "RangeSafetyFootprint",
    "TerminalCondition",
    "constant_gravity_footprint_monte_carlo",
    "landing_footprint",
    "numerical_gravity_footprint_monte_carlo",
];

const FREE_FLIGHT_STATE_SYMBOLS: &[&str] = &[
    "BallisticState",
    "DispersionSource",
    "FootprintEnvironment",
    "FootprintMonteCarloInput",
    "FootprintSampleInput",
    "RangeSafetyFootprint",
    "TerminalCondition",
    "constant_gravity_footprint_monte_carlo",
    "numerical_gravity_footprint_monte_carlo",
];

const FORBIDDEN_FC_GRAPH_PACKAGES: &[&str] = &[
    "openbmp-trajopt",
    "openbmp-runner",
    "openbmp-cli",
    "openbmp-scenario",
    "openbmp-bridge",
    "openbmp-sim",
    "openbmp-telemetry",
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
fn openbmp_fc_metadata_graph_does_not_reach_offline_optimizer_or_tooling() {
    let root = workspace_root();
    let output = Command::new("cargo")
        .args(["metadata", "--format-version", "1", "--locked"])
        .current_dir(&root)
        .output()
        .expect("run cargo metadata");
    assert!(
        output.status.success(),
        "cargo metadata failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    let metadata: serde_json::Value =
        serde_json::from_slice(&output.stdout).expect("parse cargo metadata json");
    let package_ids = package_ids_by_name(&metadata);
    let fc_id = package_ids
        .iter()
        .find_map(|(name, id)| (*name == "openbmp-fc").then_some(id.clone()))
        .expect("openbmp-fc package id");
    let reachable = reachable_package_ids(&metadata, &fc_id);
    let mut violations = Vec::new();
    for forbidden in FORBIDDEN_FC_GRAPH_PACKAGES {
        if let Some(forbidden_id) = package_ids.get(*forbidden)
            && reachable.contains(forbidden_id)
        {
            violations.push((*forbidden).to_owned());
        }
    }
    assert!(
        violations.is_empty(),
        "openbmp-fc dependency graph must not reach offline optimizer/tooling packages: \
         {violations:?}"
    );
}

#[test]
fn tripwire_fixture_catches_prohibited_fc_symbol() {
    let source = "use openbmp_physics::profile::BallisticState;";
    let matches = matching_symbols(source, PROHIBITED_FC_SYMBOLS);
    assert_eq!(matches, vec!["BallisticState"]);
}

fn package_ids_by_name(metadata: &serde_json::Value) -> std::collections::BTreeMap<String, String> {
    let mut ids = std::collections::BTreeMap::new();
    let packages = metadata
        .get("packages")
        .and_then(serde_json::Value::as_array)
        .expect("metadata packages");
    for package in packages {
        let name = package
            .get("name")
            .and_then(serde_json::Value::as_str)
            .expect("package name");
        let id = package
            .get("id")
            .and_then(serde_json::Value::as_str)
            .expect("package id");
        ids.insert(name.to_owned(), id.to_owned());
    }
    ids
}

fn reachable_package_ids(
    metadata: &serde_json::Value,
    root_id: &str,
) -> std::collections::BTreeSet<String> {
    let nodes = metadata
        .get("resolve")
        .and_then(|resolve| resolve.get("nodes"))
        .and_then(serde_json::Value::as_array)
        .expect("metadata resolve nodes");
    let mut graph = std::collections::BTreeMap::<String, Vec<String>>::new();
    for node in nodes {
        let id = node
            .get("id")
            .and_then(serde_json::Value::as_str)
            .expect("node id");
        let deps = node
            .get("dependencies")
            .and_then(serde_json::Value::as_array)
            .expect("node dependencies")
            .iter()
            .filter_map(serde_json::Value::as_str)
            .map(ToOwned::to_owned)
            .collect();
        graph.insert(id.to_owned(), deps);
    }

    let mut reachable = std::collections::BTreeSet::new();
    let mut stack = vec![root_id.to_owned()];
    while let Some(id) = stack.pop() {
        if !reachable.insert(id.clone()) {
            continue;
        }
        if let Some(deps) = graph.get(&id) {
            stack.extend(deps.iter().cloned());
        }
    }
    reachable
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
