//! Inline-data tripwires.
//!
//! This file is the workspace-wide enforcement of the four-pillar
//! provenance contract documented in
//! `docs/data-provenance.md § Inline Data Tripwires`. It walks the
//! source tree at `cargo test` time and fails the build if it finds:
//!
//! 1. **Inline TOML scenario / data fixtures in `*.rs` source.** TOML
//!    fixtures must live in `tests/fixtures/<name>.toml` (synthetic
//!    test artefact) or `data/<category>/<name>.toml` (benchmark data
//!    with sibling `provenance.md` and SHA-256 pin) and be loaded via
//!    `include_str!`. Module-level `const FOO: &str = r#"…"#;` raw
//!    strings containing OpenBMP schema headers are forbidden because
//!    they are the historical vector for "Niskanen-class" benchmark
//!    smuggling.
//!
//! 2. **Real benchmark constants outside their declared
//!    source-of-truth.** The full-precision WGS84 GM, J2, and
//!    equatorial-radius values are public physical constants; the
//!    project policy is that they live in `data/gravity/wgs84-j2.toml`
//!    with provenance, plus *one* `pub const` source-of-truth in code
//!    (`crates/openbmp-scenario/src/document.rs::WGS84_J2_DEFAULT`,
//!    underscored form so it does not match the canonical-form needle
//!    by accident). Any other appearance is benchmark transcription
//!    and fails closed.
//!
//! Both checks consult per-needle allow-lists. Adding a new genuine
//! source-of-truth means editing the allow-list, which is itself
//! reviewable.

#![allow(clippy::expect_used, clippy::panic, clippy::print_stderr)]

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

/// Workspace root, derived from this crate's manifest dir at compile time.
fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root must exist")
}

/// Top-level directories the tripwire walks.
const SCAN_ROOTS: &[&str] = &["crates", "data", "docs", "scenarios"];

/// File extensions the tripwire considers.
fn is_scan_extension(path: &Path) -> bool {
    matches!(
        path.extension().and_then(|e| e.to_str()),
        Some("rs" | "toml" | "md")
    )
}

/// Schema markers used to detect OpenBMP-shaped TOML inside a raw
/// string. The tripwire flags a multi-line `r#"…"#` whose body
/// contains any of these.
const TOML_SCHEMA_MARKERS: &[&str] = &[
    "openbmp.scenario",
    "openbmp.aero_deck",
    "openbmp.motor",
    "openbmp.imu_noise_budget",
    "openbmp.benchmark",
    "[[metric]]",
];

/// `*.rs` files where module-level raw-string TOML is allowed because
/// the file's job is to detect / cite the pattern, not to instantiate
/// it.
const INLINE_TOML_ALLOW_LIST: &[&str] = &[
    // This file: contains the schema markers as needles.
    "crates/openbmp-testkit/tests/inline_data_tripwire.rs",
];

/// Real benchmark constants. The needle is the canonical
/// non-underscored form a contributor would paste from a published
/// reference; the allow-list is the set of files where the constant
/// is the documented source-of-truth or an explicit citation.
struct Tripwire {
    name: &'static str,
    needle: &'static str,
    allow_list: &'static [&'static str],
}

const TRIPWIRES: &[Tripwire] = &[
    Tripwire {
        name: "WGS84 GM (NIMA TR 8350.2)",
        needle: "3.986004418",
        allow_list: &[
            "data/gravity/wgs84-j2.toml",
            "data/gravity/provenance.md",
            "docs/phase-2-plan.md",
            "docs/data-provenance.md",
            "crates/openbmp-testkit/tests/inline_data_tripwire.rs",
        ],
    },
    Tripwire {
        name: "WGS84 J2 unnormalised (NIMA TR 8350.2)",
        needle: "1.082626683",
        allow_list: &[
            "data/gravity/wgs84-j2.toml",
            "data/gravity/provenance.md",
            "docs/phase-2-plan.md",
            "docs/scenario-format.md",
            "docs/data-provenance.md",
            "crates/openbmp-env/src/gravity.rs",
            "crates/openbmp-testkit/tests/inline_data_tripwire.rs",
        ],
    },
    Tripwire {
        name: "WGS84 equatorial radius (NIMA TR 8350.2)",
        needle: "6378137.0",
        allow_list: &[
            "data/gravity/wgs84-j2.toml",
            "data/gravity/provenance.md",
            "docs/phase-2-plan.md",
            "docs/data-provenance.md",
            "crates/openbmp-env/src/atmosphere/us_standard_1976.rs",
            "crates/openbmp-testkit/tests/inline_data_tripwire.rs",
        ],
    },
];

/// Walk `dir` recursively, returning every file with a scannable
/// extension. Skips hidden directories and `target/`.
fn collect_scan_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
            continue;
        };
        if name.starts_with('.') || name == "target" {
            continue;
        }
        let Ok(file_type) = entry.file_type() else {
            continue;
        };
        if file_type.is_dir() {
            collect_scan_files(&path, out);
        } else if file_type.is_file() && is_scan_extension(&path) {
            out.push(path);
        }
    }
}

/// Stable, forward-slash relative path from the workspace root.
fn relative_path(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .expect("path under root")
        .to_string_lossy()
        .replace('\\', "/")
}

/// Heuristic: returns `true` when the byte offset `idx` falls inside a
/// raw-string literal that opens at module scope (i.e., not inside a
/// function body). The tripwire targets shared `const` raw-string
/// fixtures, not single-use function-local strings, because
/// module-level constants are the smuggling vector.
fn is_inside_module_level_raw_string(content: &str, idx: usize) -> bool {
    // Find the most recent opening of a raw-string literal before `idx`.
    let prefix = &content[..idx];
    let Some(opener_idx) = prefix.rfind("r#\"") else {
        return false;
    };
    // Find the closing of that raw-string literal.
    let after_opener = &content[opener_idx + 3..];
    let Some(closer_rel) = after_opener.find("\"#") else {
        return false;
    };
    // Marker must lie strictly inside the body.
    if idx <= opener_idx + 3 || idx >= opener_idx + 3 + closer_rel {
        return false;
    }
    // Walk back from the opener to find the enclosing item. If we hit
    // a `const ... &str = ` on the same logical line (or one that ends
    // there with `=`), it's a module-level constant. If we hit `let `
    // or `fn ` first, it's function-local.
    let preface = &content[..opener_idx];
    let last_semicolon = preface.rfind(';').map_or(0, |i| i + 1);
    let last_brace = preface.rfind('{').map_or(0, |i| i + 1);
    let item_start = last_semicolon.max(last_brace);
    let item = &preface[item_start..];
    let trimmed = item.trim_start();
    // Module-level pattern: `const NAME: &str = ` or `pub const NAME:
    // &str = `. Function-local `let NAME = r#"..."#` is allowed.
    trimmed.starts_with("const ")
        || trimmed.starts_with("pub const ")
        || trimmed.starts_with("pub(crate) const ")
        || trimmed.starts_with("static ")
        || trimmed.starts_with("pub static ")
}

#[test]
fn no_inline_toml_scenario_data_in_source() {
    let root = workspace_root();
    let mut files = Vec::new();
    for scan_root in SCAN_ROOTS {
        let scan_path = root.join(scan_root);
        if scan_path.is_dir() {
            collect_scan_files(&scan_path, &mut files);
        }
    }

    let mut violations: BTreeMap<String, Vec<String>> = BTreeMap::new();

    for path in files
        .iter()
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("rs"))
    {
        let rel = relative_path(&root, path);
        if INLINE_TOML_ALLOW_LIST.contains(&rel.as_str()) {
            continue;
        }
        let Ok(content) = fs::read_to_string(path) else {
            continue;
        };
        for marker in TOML_SCHEMA_MARKERS {
            if let Some(marker_idx) = content.find(marker)
                && is_inside_module_level_raw_string(&content, marker_idx)
            {
                violations.entry(rel.clone()).or_default().push(format!(
                    "module-level raw-string TOML containing `{marker}`"
                ));
            }
        }
    }

    if !violations.is_empty() {
        let mut report = String::from(
            "Inline TOML scenario / data fixtures must live in `tests/fixtures/<name>.toml` \
             or `data/<category>/<name>.toml` and be loaded via `include_str!`.\n\
             See `docs/data-provenance.md § Inline Data Tripwires`.\n\nViolations:\n",
        );
        for (file, finds) in &violations {
            let _ = writeln!(report, "  {file}:");
            for find in finds {
                let _ = writeln!(report, "    - {find}");
            }
        }
        panic!("{report}");
    }
}

#[test]
fn no_inline_benchmark_constants_in_source() {
    let root = workspace_root();
    let mut files = Vec::new();
    for scan_root in SCAN_ROOTS {
        let scan_path = root.join(scan_root);
        if scan_path.is_dir() {
            collect_scan_files(&scan_path, &mut files);
        }
    }

    let mut violations: BTreeMap<String, Vec<String>> = BTreeMap::new();

    for tripwire in TRIPWIRES {
        for path in &files {
            let rel = relative_path(&root, path);
            if tripwire.allow_list.contains(&rel.as_str()) {
                continue;
            }
            let Ok(content) = fs::read_to_string(path) else {
                continue;
            };
            if content.contains(tripwire.needle) {
                violations
                    .entry(rel.clone())
                    .or_default()
                    .push(format!("`{}` ({})", tripwire.needle, tripwire.name));
            }
        }
    }

    if !violations.is_empty() {
        let mut report = String::from(
            "Real benchmark constants must live only in their declared source-of-truth files.\n\
             See `docs/data-provenance.md § Inline Data Tripwires`.\n\nViolations:\n",
        );
        for (file, finds) in &violations {
            let _ = writeln!(report, "  {file}:");
            for find in finds {
                let _ = writeln!(report, "    - {find}");
            }
        }
        report.push_str(
            "\nIf this is a legitimate new source-of-truth, add the file path to the\n\
             tripwire's `allow_list` in `crates/openbmp-testkit/tests/inline_data_tripwire.rs`.\n",
        );
        panic!("{report}");
    }
}

#[test]
fn tripwire_finds_known_examples_in_allow_listed_files() {
    // Sanity-check: the allow-listed files must actually contain the
    // needles. If a needle disappears from its source-of-truth file
    // (e.g., `data/gravity/wgs84-j2.toml` is renamed) the allow-list
    // entry becomes stale and the tripwire silently fails to enforce.
    // This regression catches that.
    let root = workspace_root();
    for tripwire in TRIPWIRES {
        let mut found_in_any_allowed = false;
        for allowed in tripwire.allow_list {
            // The tripwire test file itself is allowed but does not
            // count toward "the constant has a real source-of-truth".
            if allowed.ends_with("inline_data_tripwire.rs") {
                continue;
            }
            let path = root.join(allowed);
            let Ok(content) = fs::read_to_string(&path) else {
                continue;
            };
            if content.contains(tripwire.needle) {
                found_in_any_allowed = true;
                break;
            }
        }
        assert!(
            found_in_any_allowed,
            "tripwire `{}` (needle `{}`) has no source-of-truth in its allow-list — \
             the allow-list is stale.",
            tripwire.name, tripwire.needle,
        );
    }
}
