//! Inline-data tripwires.
//!
//! This file is the workspace-wide enforcement of the four-pillar
//! provenance contract documented in
//! `docs/data-provenance.md § Inline Data Tripwires`. It walks the
//! source tree at `cargo test` time and fails the build if it finds:
//!
//! 1. **Inline TOML scenario / data fixtures in `*.rs` source.** Any
//!    multi-line `r#"…"#` raw string in a `*.rs` file whose body
//!    contains an OpenBMP schema header (`openbmp.scenario`,
//!    `openbmp.aero_deck`, `openbmp.motor`, `openbmp.imu_noise_budget`,
//!    `openbmp.benchmark`) or a `[[metric]]` table marker is
//!    forbidden, regardless of enclosing context (module-level
//!    `const`, function-local `let`, or `fn` returning `String`). TOML
//!    fixtures must live in sibling files under `tests/fixtures/<name>.toml`
//!    (synthetic test artefact) or `data/<category>/<name>.toml`
//!    (benchmark data with sibling `provenance.md` and SHA-256 pin)
//!    and be loaded via `include_str!`.
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
//! 3. **Test TOML lives in fixture directories, not src/.** Every
//!    `include_str!(… ".toml")` in `*.rs` source must resolve to a
//!    path under `tests/fixtures/`, `data/`, or `scenarios/`. TOML
//!    files placed inside `crates/<crate>/src/` are forbidden because
//!    src is for code, not data.
//!
//! 4. **Real-data TOML lives in `data/` or `scenarios/` with
//!    provenance.** Every `*.toml` under `data/` or `scenarios/` (or
//!    their subdirectories) must have a sibling `provenance.md` in
//!    the same directory. This is the workspace-test mirror of the
//!    `openbmp check-provenance` CLI command.
//!
//! All checks consult per-needle allow-lists. Adding a new genuine
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

/// Top-level directories the tripwires walk.
const SCAN_ROOTS: &[&str] = &["crates", "data", "docs", "scenarios"];

/// File extensions the tripwires consider.
fn is_scan_extension(path: &Path) -> bool {
    matches!(
        path.extension().and_then(|e| e.to_str()),
        Some("rs" | "toml" | "md")
    )
}

/// Schema markers used to detect OpenBMP-shaped TOML inside a raw
/// string. The tripwire flags a raw string whose body contains any of
/// these.
const TOML_SCHEMA_MARKERS: &[&str] = &[
    "openbmp.scenario",
    "openbmp.aero_deck",
    "openbmp.motor",
    "openbmp.imu_noise_budget",
    "openbmp.benchmark",
    "[[metric]]",
];

/// `*.rs` files where raw-string TOML markers are allowed because the
/// file's job is to detect / cite the pattern, not to instantiate it.
const INLINE_TOML_ALLOW_LIST: &[&str] = &[
    // This file: contains the schema markers as needles.
    "crates/openbmp-testkit/tests/inline_data_tripwire.rs",
];

/// Substrings that indicate a path passed to `include_str!` is in an
/// approved TOML location: per-crate test fixtures, the workspace
/// `data/` tree (real benchmark data with provenance), or
/// `scenarios/` (canonical scenarios).
const INCLUDE_TOML_ALLOWED_SUBSTRINGS: &[&str] = &[
    "/tests/fixtures/",
    "/tests/expected/",
    "/data/",
    "/scenarios/",
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

/// Walk `dir` recursively, returning every file matching `extension`.
fn collect_files_with_extension(dir: &Path, extension: &str, out: &mut Vec<PathBuf>) {
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
            collect_files_with_extension(&path, extension, out);
        } else if file_type.is_file()
            && path.extension().and_then(|e| e.to_str()) == Some(extension)
        {
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

/// Iterate every `r#"…"#` raw-string literal in `content` and yield
/// `(opener_byte_offset, body)` pairs. The scanner tolerates any
/// number of `#` characters (`r##"…"##`, etc.).
fn for_each_raw_string<F>(content: &str, mut f: F)
where
    F: FnMut(usize, &str),
{
    let bytes = content.as_bytes();
    let mut i = 0;
    while i + 2 < bytes.len() {
        if bytes[i] == b'r' {
            // Count following `#`s.
            let mut hash_count = 0;
            while i + 1 + hash_count < bytes.len() && bytes[i + 1 + hash_count] == b'#' {
                hash_count += 1;
            }
            if hash_count > 0
                && i + 1 + hash_count < bytes.len()
                && bytes[i + 1 + hash_count] == b'"'
            {
                let body_start = i + 2 + hash_count;
                let mut needle = String::with_capacity(hash_count + 1);
                needle.push('"');
                for _ in 0..hash_count {
                    needle.push('#');
                }
                if let Some(rel_end) = content[body_start..].find(&needle) {
                    let body = &content[body_start..body_start + rel_end];
                    f(i, body);
                    i = body_start + rel_end + needle.len();
                    continue;
                }
            }
        }
        i += 1;
    }
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
        for_each_raw_string(&content, |_offset, body| {
            // Multi-line raw strings (i.e., bodies that contain a
            // newline) are the smuggling shape; single-line raw
            // strings used as `.replace(needle, …)` patterns are
            // always permitted.
            if !body.contains('\n') {
                return;
            }
            for marker in TOML_SCHEMA_MARKERS {
                if body.contains(marker) {
                    violations
                        .entry(rel.clone())
                        .or_default()
                        .push(format!("raw-string TOML containing `{marker}`"));
                }
            }
        });
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
    let root = workspace_root();
    for tripwire in TRIPWIRES {
        let mut found_in_any_allowed = false;
        for allowed in tripwire.allow_list {
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

/// Find every `include_str!(... ".toml" ...)` argument literal in a
/// Rust source file. Returns the literal substrings (everything
/// between the opening `(` and closing `)`).
fn find_include_str_arguments(content: &str) -> Vec<String> {
    let mut out = Vec::new();
    let needle = "include_str!(";
    let mut search_from = 0;
    while let Some(rel) = content[search_from..].find(needle) {
        let open = search_from + rel + needle.len();
        let mut depth = 1;
        let mut end = open;
        let bytes = content.as_bytes();
        while end < bytes.len() && depth > 0 {
            match bytes[end] {
                b'(' => depth += 1,
                b')' => depth -= 1,
                _ => {}
            }
            if depth > 0 {
                end += 1;
            }
        }
        if depth == 0 {
            let arg = &content[open..end];
            if arg.contains(".toml") {
                out.push(arg.to_string());
            }
            search_from = end + 1;
        } else {
            break;
        }
    }
    out
}

#[test]
fn test_toml_lives_in_fixture_or_data_directories() {
    let root = workspace_root();
    let crates_dir = root.join("crates");
    let mut rust_files = Vec::new();
    if crates_dir.is_dir() {
        collect_files_with_extension(&crates_dir, "rs", &mut rust_files);
    }

    let mut violations: BTreeMap<String, Vec<String>> = BTreeMap::new();

    for path in &rust_files {
        let rel = relative_path(&root, path);
        if INLINE_TOML_ALLOW_LIST.contains(&rel.as_str()) {
            continue;
        }
        let Ok(content) = fs::read_to_string(path) else {
            continue;
        };
        for arg in find_include_str_arguments(&content) {
            // A literal fragment of the form "/tests/fixtures/…/foo.toml"
            // (or `/data/…/foo.toml`, `/scenarios/…/foo.toml`) anywhere
            // inside the macro argument is enough; this works whether
            // the call uses `concat!(env!("CARGO_MANIFEST_DIR"), "/…")`
            // or a bare relative path.
            let allowed = INCLUDE_TOML_ALLOWED_SUBSTRINGS
                .iter()
                .any(|substring| arg.contains(substring));
            if !allowed {
                violations.entry(rel.clone()).or_default().push(format!(
                    "include_str!(... .toml) does not resolve under \
                     tests/fixtures/, tests/expected/, data/, or scenarios/: {}",
                    arg.trim().chars().take(120).collect::<String>(),
                ));
            }
        }
    }

    if !violations.is_empty() {
        let mut report = String::from(
            "Test TOML fixtures must live under `tests/fixtures/`, `tests/expected/`, \
             `data/`, or `scenarios/`. TOML files inside `src/` are forbidden because \
             src is for code, not data.\n\
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
fn data_and_scenario_tomls_have_sibling_provenance() {
    // Every `*.toml` under `data/` or `scenarios/` must have a
    // sibling `provenance.md` in the same directory. Mirrors the
    // `openbmp check-provenance` CLI command but enforces it under
    // `cargo test` so the contract is checked even when the CLI is
    // not run. Sub-directories may carry their own per-directory
    // provenance.md.
    let root = workspace_root();
    let mut violations: BTreeMap<String, Vec<String>> = BTreeMap::new();

    for tree in &["data", "scenarios"] {
        let tree_path = root.join(tree);
        if !tree_path.is_dir() {
            continue;
        }
        let mut tomls = Vec::new();
        collect_files_with_extension(&tree_path, "toml", &mut tomls);
        for toml_path in tomls {
            let Some(parent) = toml_path.parent() else {
                continue;
            };
            let provenance = parent.join("provenance.md");
            if !provenance.is_file() {
                let rel = relative_path(&root, &toml_path);
                violations.entry((*tree).to_owned()).or_default().push(rel);
            }
        }
    }

    if !violations.is_empty() {
        let mut report = String::from(
            "Every `*.toml` under `data/` or `scenarios/` must have a sibling \
             `provenance.md` in the same directory.\n\
             See `docs/data-provenance.md § Required Record`.\n\nViolations:\n",
        );
        for (tree, finds) in &violations {
            let _ = writeln!(report, "  {tree}/:");
            for find in finds {
                let _ = writeln!(report, "    - {find}");
            }
        }
        panic!("{report}");
    }
}
