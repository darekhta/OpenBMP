//! Flight-controller dependency tripwire.
//!
//! The load-bearing rule:
//! `openbmp-fc` must depend only on hardware-portable crates. This
//! test fails closed if a future edit adds a forbidden repository-local
//! dependency to the controller crate.
//!
//! The forbidden-name check looks at *both*
//! the dep-table key and the inline `package = "..."` field, so
//! `sim = { workspace = true, package = "openbmp-sim" }` is caught
//! the same as `openbmp-sim = { workspace = true }`. The tripwire
//! also recurses into target-conditional dep tables.
//!
//! Workspace-level renamed dependencies are
//! resolved through the root `[workspace.dependencies]` table, so
//! `sim = { workspace = true }` is caught when the workspace aliases
//! `sim` to `package = "openbmp-sim"`. The tripwire also scans
//! `dep:<name>` feature activations.

#![allow(clippy::expect_used, clippy::panic)]

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

use toml::Value;

const FORBIDDEN_OPENBMP_FC_DEPS: &[&str] = &[
    "openbmp-sim",
    "openbmp-cli",
    "openbmp-runner",
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
    let workspace_manifest_path = workspace_root().join("Cargo.toml");
    let manifest = fs::read_to_string(&manifest_path)
        .unwrap_or_else(|err| panic!("failed to read {}: {err}", manifest_path.display()));
    let workspace_manifest = fs::read_to_string(&workspace_manifest_path).unwrap_or_else(|err| {
        panic!(
            "failed to read {}: {err}",
            workspace_manifest_path.display()
        )
    });
    let violations = scan_forbidden_deps_with_workspace(&manifest, &workspace_manifest);
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

#[test]
fn tripwire_catches_workspace_renamed_dependency_alias() {
    let workspace = r#"
[workspace]

[workspace.dependencies]
sim    = { version = "0.0.1", package = "openbmp-sim" }
models = { version = "0.0.1", package = "openbmp-models" }
"#;
    let manifest = r#"
[package]
name = "openbmp-fc"

[dependencies]
sim    = { workspace = true }
models = { workspace = true }
"#;
    let violations = scan_forbidden_deps_with_workspace(manifest, workspace);
    assert!(
        violations
            .iter()
            .any(|v| v.contains("openbmp-sim") && v.contains("key=sim")),
        "workspace alias `sim` (package = openbmp-sim) must be reported: got {violations:?}",
    );
    assert!(
        violations.iter().all(|v| !v.contains("openbmp-models")),
        "openbmp-models is allowed; must not be reported: got {violations:?}",
    );
}

#[test]
fn tripwire_catches_dep_feature_activation() {
    let with_feature = r#"
[package]
name = "openbmp-fc"

[features]
simhack = ["dep:openbmp-sim"]
"#;
    let violations = scan_forbidden_deps(with_feature);
    assert!(
        violations
            .iter()
            .any(|v| v.contains("features.simhack:openbmp-sim")),
        "`dep:openbmp-sim` feature activation must be reported: got {violations:?}",
    );
}

#[test]
fn tripwire_catches_dep_feature_activation_through_workspace_alias() {
    let workspace = r#"
[workspace]

[workspace.dependencies]
sim = { version = "0.0.1", package = "openbmp-sim" }
"#;
    let manifest = r#"
[package]
name = "openbmp-fc"

[dependencies]
sim = { workspace = true, optional = true }

[features]
simhack = ["dep:sim"]
"#;
    let violations = scan_forbidden_deps_with_workspace(manifest, workspace);
    assert!(
        violations
            .iter()
            .any(|v| v.contains("openbmp-sim") && v.contains("alias=sim")),
        "`dep:sim` must resolve through workspace package = openbmp-sim: got {violations:?}",
    );
}

fn scan_forbidden_deps(manifest: &str) -> BTreeSet<String> {
    scan_forbidden_deps_with_workspace(manifest, "")
}

fn scan_forbidden_deps_with_workspace(
    manifest: &str,
    workspace_manifest: &str,
) -> BTreeSet<String> {
    let parsed: Value =
        toml::from_str(manifest).unwrap_or_else(|err| panic!("failed to parse manifest: {err}"));
    let workspace_deps = workspace_dependency_packages(workspace_manifest);
    let mut violations = BTreeSet::new();

    for section in ["dependencies", "dev-dependencies", "build-dependencies"] {
        collect_forbidden_deps(&parsed, section, section, &workspace_deps, &mut violations);
    }

    if let Some(targets) = parsed.get("target").and_then(Value::as_table) {
        for (target_name, target_table) in targets {
            for section in ["dependencies", "dev-dependencies", "build-dependencies"] {
                collect_forbidden_deps(
                    target_table,
                    section,
                    &format!("target.{target_name}.{section}"),
                    &workspace_deps,
                    &mut violations,
                );
            }
        }
    }

    collect_forbidden_feature_refs(&parsed, &workspace_deps, &mut violations);

    violations
}

fn workspace_dependency_packages(workspace_manifest: &str) -> BTreeMap<String, String> {
    if workspace_manifest.trim().is_empty() {
        return BTreeMap::new();
    }
    let parsed: Value = toml::from_str(workspace_manifest)
        .unwrap_or_else(|err| panic!("failed to parse workspace manifest: {err}"));
    let Some(deps) = parsed
        .get("workspace")
        .and_then(|workspace| workspace.get("dependencies"))
        .and_then(Value::as_table)
    else {
        return BTreeMap::new();
    };

    deps.iter()
        .map(|(dep_key, dep_value)| {
            let package = dep_value
                .as_table()
                .and_then(|t| t.get("package"))
                .and_then(Value::as_str)
                .unwrap_or(dep_key)
                .to_owned();
            (dep_key.to_owned(), package)
        })
        .collect()
}

fn collect_forbidden_deps(
    root: &Value,
    section: &str,
    label: &str,
    workspace_deps: &BTreeMap<String, String>,
    violations: &mut BTreeSet<String>,
) {
    let Some(deps) = root.get(section).and_then(Value::as_table) else {
        return;
    };
    for (dep_key, dep_value) in deps {
        // The effective package name: either the inline `package`
        // field (rename pattern), the workspace dependency package
        // behind `{ workspace = true }`, or the dep-table key (compact
        // and explicit forms).
        let resolved_pkg = resolve_dep_package(dep_key, dep_value, workspace_deps);
        if FORBIDDEN_OPENBMP_FC_DEPS.contains(&resolved_pkg.as_str()) {
            // Report the resolved package name so the failure message
            // points at the *forbidden* crate, not at the alias.
            violations.insert(format!("{label}:{resolved_pkg} (key={dep_key})"));
        }
    }
}

fn resolve_dep_package(
    dep_key: &str,
    dep_value: &Value,
    workspace_deps: &BTreeMap<String, String>,
) -> String {
    let Some(dep_table) = dep_value.as_table() else {
        return dep_key.to_owned();
    };
    if let Some(package) = dep_table.get("package").and_then(Value::as_str) {
        return package.to_owned();
    }
    let uses_workspace = dep_table
        .get("workspace")
        .and_then(Value::as_bool)
        .unwrap_or(false);
    if uses_workspace {
        return workspace_deps
            .get(dep_key)
            .cloned()
            .unwrap_or_else(|| dep_key.to_owned());
    }
    dep_key.to_owned()
}

fn collect_forbidden_feature_refs(
    parsed: &Value,
    workspace_deps: &BTreeMap<String, String>,
    violations: &mut BTreeSet<String>,
) {
    let Some(features) = parsed.get("features").and_then(Value::as_table) else {
        return;
    };
    for (feature_name, feature_value) in features {
        let Some(entries) = feature_value.as_array() else {
            continue;
        };
        for entry in entries.iter().filter_map(Value::as_str) {
            let Some(alias) = entry.strip_prefix("dep:") else {
                continue;
            };
            let resolved_pkg = workspace_deps.get(alias).map_or(alias, String::as_str);
            if FORBIDDEN_OPENBMP_FC_DEPS.contains(&resolved_pkg) {
                violations.insert(format!(
                    "features.{feature_name}:{resolved_pkg} (alias={alias})"
                ));
            }
        }
    }
}
