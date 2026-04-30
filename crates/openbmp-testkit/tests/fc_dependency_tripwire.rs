//! Flight-controller dependency tripwire.
//!
//! Phase 3.14 established the load-bearing Phase-4 rule:
//! `openbmp-fc` must depend only on hardware-portable crates. This
//! test fails closed if a future edit adds a forbidden repository-local
//! dependency to the controller crate.
//!
//! Phase-3.15.A hardening: the forbidden-name check looks at *both*
//! the dep-table key and the inline `package = "..."` field, so
//! `sim = { workspace = true, package = "openbmp-sim" }` is caught
//! the same as `openbmp-sim = { workspace = true }`. The tripwire
//! also recurses into target-conditional dep tables.

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
    let violations = scan_forbidden_deps(&manifest);
    assert!(
        violations.is_empty(),
        "openbmp-fc must not depend on simulator/tooling crates: {violations:?}"
    );
}

#[test]
fn tripwire_catches_renamed_dependency_via_package_field() {
    // Negative-case fixture: a `dep-table key != package name` rename
    // that the original tripwire missed. The hardened scan must report
    // it as a violation.
    let renamed = r#"
[package]
name = "openbmp-fc"

[dependencies]
sim   = { version = "0.0.1", package = "openbmp-sim" }
clean = { version = "0.0.1", package = "openbmp-models" }
"#;
    let violations = scan_forbidden_deps(renamed);
    assert!(
        violations.iter().any(|v| v.contains("openbmp-sim")),
        "renamed dep `sim` (package = openbmp-sim) must be reported as a forbidden edge: \
         got {violations:?}",
    );
    assert!(
        violations.iter().all(|v| !v.contains("openbmp-models")),
        "openbmp-models is allowed; must not be reported: got {violations:?}",
    );
}

#[test]
fn tripwire_passes_for_clean_manifest() {
    let clean = r#"
[package]
name = "openbmp-fc"

[dependencies]
openbmp-models  = { workspace = true }
openbmp-mission = { workspace = true }
openbmp-state   = { workspace = true }
"#;
    let violations = scan_forbidden_deps(clean);
    assert!(
        violations.is_empty(),
        "clean manifest must produce no violations: {violations:?}",
    );
}

#[test]
fn tripwire_catches_string_form_dependency() {
    // The compact `name = "version"` form: no package field, key is
    // the package name. The plain key check still bites.
    let compact = r#"
[package]
name = "openbmp-fc"

[dependencies]
openbmp-cli = "0.0.1"
"#;
    let violations = scan_forbidden_deps(compact);
    assert!(
        violations.iter().any(|v| v.contains("openbmp-cli")),
        "compact dep `openbmp-cli = \"...\"` must be reported: got {violations:?}",
    );
}

#[test]
fn tripwire_catches_target_conditional_dependency() {
    let with_target = r#"
[package]
name = "openbmp-fc"

[target.'cfg(unix)'.dependencies]
openbmp-bridge = { workspace = true }
"#;
    let violations = scan_forbidden_deps(with_target);
    assert!(
        violations.iter().any(|v| v.contains("openbmp-bridge")),
        "target-conditional forbidden dep must be reported: got {violations:?}",
    );
}

fn scan_forbidden_deps(manifest: &str) -> BTreeSet<String> {
    let parsed: Value =
        toml::from_str(manifest).unwrap_or_else(|err| panic!("failed to parse manifest: {err}"));
    let mut violations = BTreeSet::new();

    for section in ["dependencies", "dev-dependencies", "build-dependencies"] {
        collect_forbidden_deps(&parsed, section, section, &mut violations);
    }

    if let Some(targets) = parsed.get("target").and_then(Value::as_table) {
        for (target_name, target_table) in targets {
            for section in ["dependencies", "dev-dependencies", "build-dependencies"] {
                collect_forbidden_deps(
                    target_table,
                    section,
                    &format!("target.{target_name}.{section}"),
                    &mut violations,
                );
            }
        }
    }

    violations
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
    for (dep_key, dep_value) in deps {
        // The effective package name: either the inline `package`
        // field (rename pattern) or the dep-table key (compact and
        // explicit forms). Phase-3.15.A: the prior tripwire only
        // checked the key, which a renamed dep silently bypassed.
        let resolved_pkg = dep_value
            .as_table()
            .and_then(|t| t.get("package"))
            .and_then(Value::as_str)
            .unwrap_or(dep_key.as_str());
        if FORBIDDEN_OPENBMP_FC_DEPS.contains(&resolved_pkg) {
            // Report the resolved package name so the failure message
            // points at the *forbidden* crate, not at the alias.
            violations.insert(format!("{label}:{resolved_pkg} (key={dep_key})"));
        }
    }
}
