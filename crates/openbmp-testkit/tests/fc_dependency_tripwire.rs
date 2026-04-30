//! Flight-controller dependency tripwire.
//!
//! Phase 3.14 established the load-bearing Phase-4 rule:
//! `openbmp-fc` must depend only on hardware-portable crates. This
//! test fails closed if a future edit adds a forbidden repository-local
//! dependency to the controller crate.

#![allow(clippy::expect_used, clippy::panic)]

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

use toml::Value;

const FORBIDDEN_OPENBMP_FC_DEPS: &[&str] = &[
    "openbmp-sim",
    "openbmp-cli",
    "openbmp-scenario",
    "openbmp-telemetry",
    "openbmp-bridge",
    "openbmp-aerothermal",
];

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root must exist")
}

#[test]
fn openbmp_fc_does_not_depend_on_simulator_or_tooling_crates() {
    let manifest_path = workspace_root().join("crates/openbmp-fc/Cargo.toml");
    let manifest = fs::read_to_string(&manifest_path)
        .unwrap_or_else(|err| panic!("failed to read {}: {err}", manifest_path.display()));
    let parsed: Value = toml::from_str(&manifest)
        .unwrap_or_else(|err| panic!("failed to parse {}: {err}", manifest_path.display()));

    let mut violations = BTreeSet::new();
    collect_forbidden_deps(&parsed, "dependencies", "dependencies", &mut violations);
    collect_forbidden_deps(
        &parsed,
        "dev-dependencies",
        "dev-dependencies",
        &mut violations,
    );
    collect_forbidden_deps(
        &parsed,
        "build-dependencies",
        "build-dependencies",
        &mut violations,
    );

    if let Some(targets) = parsed.get("target").and_then(Value::as_table) {
        for (target_name, target_table) in targets {
            collect_forbidden_deps(
                target_table,
                "dependencies",
                &format!("target.{target_name}.dependencies"),
                &mut violations,
            );
            collect_forbidden_deps(
                target_table,
                "dev-dependencies",
                &format!("target.{target_name}.dev-dependencies"),
                &mut violations,
            );
            collect_forbidden_deps(
                target_table,
                "build-dependencies",
                &format!("target.{target_name}.build-dependencies"),
                &mut violations,
            );
        }
    }

    assert!(
        violations.is_empty(),
        "openbmp-fc must not depend on simulator/tooling crates: {violations:?}"
    );
}

fn collect_forbidden_deps(
    root: &Value,
    section: &str,
    label: &str,
    violations: &mut BTreeSet<String>,
) {
    let Some(deps) = root.get(section).and_then(Value::as_table) else {
        return;
    };
    for dep_name in deps.keys() {
        if FORBIDDEN_OPENBMP_FC_DEPS.contains(&dep_name.as_str()) {
            violations.insert(format!("{label}:{dep_name}"));
        }
    }
}
