//! Inline-data tripwires.
//!
//! This file is the workspace-wide enforcement of the four-pillar
//! provenance contract documented in
//! `docs/data-provenance.md § Inline Data Tripwires`. It walks the
//! source tree at `cargo test` time and fails the build if it finds:
//!
//! 1. **Inline TOML scenario / data fixtures in `*.rs` source.** Any
//!    multi-line Rust string literal in a `*.rs` file whose body
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
//!    with provenance, plus explicitly allow-listed Rust constants
//!    that cite the same source. The checker normalises Rust numeric
//!    separators before matching. Any other appearance is benchmark
//!    transcription and fails closed.
//!
//! 3. **Test TOML lives in fixture directories, not src/.** Every
//!    `include_str!(… ".toml")` in `*.rs` source must lexically
//!    resolve to a path under `tests/fixtures/`, `tests/expected/`,
//!    `data/`, or `scenarios/`. TOML files placed inside
//!    `crates/<crate>/src/` are forbidden because src is for code,
//!    not data.
//!
//! 4. **Real-data TOML lives in `data/` or `scenarios/` with
//!    provenance.** Every `*.toml` under `data/` or `scenarios/` (or
//!    their subdirectories) must have a sibling `provenance.md` in
//!    the same directory, and that file must list the TOML path. This
//!    is the workspace-test mirror of the `openbmp check-provenance`
//!    CLI command.
//!
//! All checks consult per-needle allow-lists. Adding a new genuine
//! source-of-truth means editing the allow-list, which is itself
//! reviewable.

#![allow(clippy::expect_used, clippy::panic, clippy::print_stderr)]

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::path::{Component, Path, PathBuf};

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

/// Schema markers used to detect OpenBMP-shaped TOML inside a Rust
/// string literal. The tripwire flags a multi-line string whose body
/// contains any of these.
const TOML_SCHEMA_MARKERS: &[&str] = &[
    "openbmp.scenario",
    "openbmp.aero_deck",
    "openbmp.motor",
    "openbmp.imu_noise_budget",
    "openbmp.benchmark",
    "[[metric]]",
];

const INCLUDE_STR_MACRO: &[u8] = b"include_str";

/// `*.rs` files where string-literal TOML markers are allowed because the
/// file's job is to detect / cite the pattern, not to instantiate it.
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
            "docs/data-provenance.md",
            "crates/openbmp-core/src/frames.rs",
            "crates/openbmp-testkit/tests/inline_data_tripwire.rs",
        ],
    },
    Tripwire {
        name: "WGS84 J2 unnormalised (NIMA TR 8350.2)",
        needle: "1.082626683",
        allow_list: &[
            "data/gravity/wgs84-j2.toml",
            "docs/scenario-format.md",
            "docs/data-provenance.md",
            "crates/openbmp-env/src/gravity.rs",
            "crates/openbmp-scenario/src/document.rs",
            "crates/openbmp-testkit/tests/inline_data_tripwire.rs",
        ],
    },
    Tripwire {
        name: "WGS84 equatorial radius (NIMA TR 8350.2)",
        needle: "6378137.0",
        allow_list: &[
            "data/gravity/wgs84-j2.toml",
            "docs/data-provenance.md",
            "crates/openbmp-core/src/frames.rs",
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

fn display_path(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .to_string_lossy()
        .replace('\\', "/")
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum StringLiteralKind {
    Cooked,
    Raw,
}

impl StringLiteralKind {
    const fn label(self) -> &'static str {
        match self {
            Self::Cooked => "string-literal",
            Self::Raw => "raw-string",
        }
    }
}

fn is_ident_continue(byte: u8) -> bool {
    byte.is_ascii_alphanumeric() || byte == b'_'
}

fn is_token_boundary(bytes: &[u8], index: usize) -> bool {
    index == 0 || !is_ident_continue(bytes[index - 1])
}

fn skip_whitespace(bytes: &[u8], mut index: usize) -> usize {
    while index < bytes.len() && bytes[index].is_ascii_whitespace() {
        index += 1;
    }
    index
}

fn skip_line_comment(bytes: &[u8], mut index: usize) -> usize {
    while index < bytes.len() && bytes[index] != b'\n' {
        index += 1;
    }
    index
}

fn skip_block_comment(bytes: &[u8], mut index: usize) -> usize {
    index += 2;
    let mut depth = 1_u32;
    while index + 1 < bytes.len() && depth > 0 {
        if bytes[index] == b'/' && bytes[index + 1] == b'*' {
            depth += 1;
            index += 2;
        } else if bytes[index] == b'*' && bytes[index + 1] == b'/' {
            depth -= 1;
            index += 2;
        } else {
            index += 1;
        }
    }
    index
}

fn skip_comment_at(bytes: &[u8], index: usize) -> Option<usize> {
    if index + 1 >= bytes.len() || bytes[index] != b'/' {
        return None;
    }
    match bytes[index + 1] {
        b'/' => Some(skip_line_comment(bytes, index + 2)),
        b'*' => Some(skip_block_comment(bytes, index)),
        _ => None,
    }
}

fn parse_raw_string_at(content: &str, index: usize) -> Option<(usize, usize, usize)> {
    let bytes = content.as_bytes();
    if !is_token_boundary(bytes, index) {
        return None;
    }

    let r_index = if bytes[index] == b'r' {
        index
    } else if index + 1 < bytes.len()
        && (bytes[index] == b'b' || bytes[index] == b'c')
        && bytes[index + 1] == b'r'
    {
        index + 1
    } else {
        return None;
    };

    let mut delimiter_quote = r_index + 1;
    while delimiter_quote < bytes.len() && bytes[delimiter_quote] == b'#' {
        delimiter_quote += 1;
    }
    if delimiter_quote >= bytes.len() || bytes[delimiter_quote] != b'"' {
        return None;
    }

    let hash_count = delimiter_quote - r_index - 1;
    let body_start = delimiter_quote + 1;
    let mut closer = String::with_capacity(hash_count + 1);
    closer.push('"');
    for _ in 0..hash_count {
        closer.push('#');
    }
    let rel_end = content[body_start..].find(&closer)?;
    let body_end = body_start + rel_end;
    Some((body_start, body_end, body_end + closer.len()))
}

fn parse_cooked_string_at(content: &str, index: usize) -> Option<(usize, usize, usize)> {
    let bytes = content.as_bytes();
    let quote_index = if bytes[index] == b'"' {
        index
    } else if is_token_boundary(bytes, index)
        && index + 1 < bytes.len()
        && (bytes[index] == b'b' || bytes[index] == b'c')
        && bytes[index + 1] == b'"'
    {
        index + 1
    } else {
        return None;
    };

    let body_start = quote_index + 1;
    let mut cursor = body_start;
    let mut escaped = false;
    while cursor < bytes.len() {
        if escaped {
            escaped = false;
        } else if bytes[cursor] == b'\\' {
            escaped = true;
        } else if bytes[cursor] == b'"' {
            return Some((body_start, cursor, cursor + 1));
        }
        cursor += 1;
    }
    None
}

/// Iterate every Rust string literal in `content` and yield
/// `(opener_byte_offset, kind, body)` tuples. Supports cooked
/// strings, raw strings with any `#` count including `r"…"`, and
/// byte/C-string prefixes used with the same delimiters.
fn for_each_rust_string_literal<F>(content: &str, mut f: F)
where
    F: FnMut(usize, StringLiteralKind, &str),
{
    let bytes = content.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if let Some(next) = skip_comment_at(bytes, i) {
            i = next;
            continue;
        }
        if let Some((body_start, body_end, end)) = parse_raw_string_at(content, i) {
            f(i, StringLiteralKind::Raw, &content[body_start..body_end]);
            i = end;
            continue;
        }
        if let Some((body_start, body_end, end)) = parse_cooked_string_at(content, i) {
            f(i, StringLiteralKind::Cooked, &content[body_start..body_end]);
            i = end;
            continue;
        }
        i += 1;
    }
}

fn string_body_is_multiline(body: &str) -> bool {
    body.contains('\n') || body.contains("\\n")
}

fn inline_toml_findings(content: &str) -> Vec<String> {
    let mut findings = Vec::new();
    for_each_rust_string_literal(content, |_offset, kind, body| {
        if !string_body_is_multiline(body) {
            return;
        }
        for marker in TOML_SCHEMA_MARKERS {
            if body.contains(marker) {
                findings.push(format!("{} TOML containing `{marker}`", kind.label()));
            }
        }
    });
    findings
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
        violations
            .entry(rel.clone())
            .or_default()
            .extend(inline_toml_findings(&content));
        if violations.get(&rel).is_some_and(Vec::is_empty) {
            violations.remove(&rel);
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

fn strip_numeric_separators(value: &str) -> String {
    value.chars().filter(|ch| *ch != '_').collect()
}

fn contains_tripwire_needle(content: &str, needle: &str) -> bool {
    content.contains(needle) || strip_numeric_separators(content).contains(needle)
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
            if contains_tripwire_needle(&content, tripwire.needle) {
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
    let mut stale_entries = Vec::new();
    for tripwire in TRIPWIRES {
        for allowed in tripwire.allow_list {
            if allowed.ends_with("inline_data_tripwire.rs") {
                continue;
            }
            let path = root.join(allowed);
            let content = match fs::read_to_string(&path) {
                Ok(content) => content,
                Err(error) => {
                    stale_entries.push(format!(
                        "`{}` allow-list entry `{allowed}` could not be read: {error}",
                        tripwire.name
                    ));
                    continue;
                }
            };
            if !contains_tripwire_needle(&content, tripwire.needle) {
                stale_entries.push(format!(
                    "`{}` allow-list entry `{allowed}` does not contain `{}`",
                    tripwire.name, tripwire.needle
                ));
            }
        }
    }

    if !stale_entries.is_empty() {
        let mut report = String::from("Tripwire allow-list entries are stale:\n");
        for entry in stale_entries {
            let _ = writeln!(report, "  - {entry}");
        }
        panic!("{report}");
    }
}

/// Find every `include_str!(... ".toml" ...)` argument literal in a
/// Rust source file. Returns the literal substrings (everything
/// between the opening `(` and closing `)`).
fn find_include_str_arguments(content: &str) -> Vec<String> {
    let mut out = Vec::new();
    let bytes = content.as_bytes();
    let mut index = 0;
    while index < bytes.len() {
        if let Some(next) = skip_comment_at(bytes, index) {
            index = next;
            continue;
        }

        if let Some((_, _, end)) = parse_raw_string_at(content, index) {
            index = end;
            continue;
        }
        if let Some((_, _, end)) = parse_cooked_string_at(content, index) {
            index = end;
            continue;
        }

        let has_name = index + INCLUDE_STR_MACRO.len() <= bytes.len()
            && &bytes[index..index + INCLUDE_STR_MACRO.len()] == INCLUDE_STR_MACRO
            && is_token_boundary(bytes, index)
            && (index + INCLUDE_STR_MACRO.len() == bytes.len()
                || !is_ident_continue(bytes[index + INCLUDE_STR_MACRO.len()]));

        if has_name {
            let mut cursor = skip_whitespace(bytes, index + INCLUDE_STR_MACRO.len());
            if cursor < bytes.len() && bytes[cursor] == b'!' {
                cursor = skip_whitespace(bytes, cursor + 1);
                if cursor < bytes.len()
                    && bytes[cursor] == b'('
                    && let Some(close) = find_matching_paren(content, cursor)
                {
                    let arg = &content[cursor + 1..close];
                    if argument_contains_toml_path(arg) {
                        out.push(arg.to_string());
                    }
                    index = close + 1;
                    continue;
                }
            }
        }

        index += 1;
    }
    out
}

fn find_matching_paren(content: &str, open_paren: usize) -> Option<usize> {
    let bytes = content.as_bytes();
    let mut depth = 1_u32;
    let mut index = open_paren + 1;
    while index < bytes.len() {
        if let Some(next) = skip_comment_at(bytes, index) {
            index = next;
            continue;
        }
        if let Some((_, _, end)) = parse_raw_string_at(content, index) {
            index = end;
            continue;
        }
        if let Some((_, _, end)) = parse_cooked_string_at(content, index) {
            index = end;
            continue;
        }

        match bytes[index] {
            b'(' => depth += 1,
            b')' => {
                depth -= 1;
                if depth == 0 {
                    return Some(index);
                }
            }
            _ => {}
        }
        index += 1;
    }
    None
}

fn string_literal_bodies(content: &str) -> Vec<String> {
    let mut out = Vec::new();
    for_each_rust_string_literal(content, |_offset, _kind, body| out.push(body.to_owned()));
    out
}

fn argument_contains_toml_path(argument: &str) -> bool {
    string_literal_bodies(argument)
        .into_iter()
        .any(|literal| literal.contains(".toml"))
}

fn argument_uses_cargo_manifest_dir(argument: &str) -> bool {
    argument.contains("env!")
        && string_literal_bodies(argument)
            .into_iter()
            .any(|literal| literal == "CARGO_MANIFEST_DIR")
}

fn crate_manifest_dir(root: &Path, source_path: &Path) -> Option<PathBuf> {
    let mut cursor = source_path.parent()?;
    loop {
        if cursor.join("Cargo.toml").is_file() {
            return Some(cursor.to_path_buf());
        }
        if cursor == root {
            return None;
        }
        cursor = cursor.parent()?;
    }
}

fn include_str_literal_path(argument: &str) -> Option<String> {
    let fragments = string_literal_bodies(argument);
    let uses_manifest_dir = argument_uses_cargo_manifest_dir(argument);
    let mut path = String::new();
    for fragment in fragments {
        if uses_manifest_dir && fragment == "CARGO_MANIFEST_DIR" {
            continue;
        }
        path.push_str(&fragment);
    }
    path.contains(".toml").then_some(path)
}

fn normalize_path(path: &Path) -> PathBuf {
    let mut normalized = PathBuf::new();
    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                normalized.pop();
            }
            Component::Prefix(_) | Component::RootDir | Component::Normal(_) => {
                normalized.push(component.as_os_str());
            }
        }
    }
    normalized
}

fn resolve_include_str_path(root: &Path, source_path: &Path, argument: &str) -> Option<PathBuf> {
    let uses_manifest_dir = argument_uses_cargo_manifest_dir(argument);
    let literal_path = include_str_literal_path(argument)?;
    let literal_path = if uses_manifest_dir {
        literal_path.trim_start_matches(['/', '\\']).to_owned()
    } else {
        literal_path
    };
    let literal_path = PathBuf::from(literal_path);
    if literal_path.is_absolute() && !uses_manifest_dir {
        return Some(normalize_path(&literal_path));
    }

    let base = if uses_manifest_dir {
        crate_manifest_dir(root, source_path)?
    } else {
        source_path.parent()?.to_path_buf()
    };

    Some(normalize_path(&base.join(literal_path)))
}

fn path_components(path: &Path) -> Vec<String> {
    path.components()
        .filter_map(|component| match component {
            Component::Normal(value) => Some(value.to_string_lossy().into_owned()),
            _ => None,
        })
        .collect()
}

fn is_allowed_toml_include(root: &Path, resolved_path: &Path) -> bool {
    let root = normalize_path(root);
    let resolved_path = normalize_path(resolved_path);
    if resolved_path.starts_with(root.join("data"))
        || resolved_path.starts_with(root.join("scenarios"))
    {
        return true;
    }

    let crates_root = root.join("crates");
    let Ok(rel) = resolved_path.strip_prefix(crates_root) else {
        return false;
    };
    let components = path_components(rel);
    components.len() >= 4
        && components[1] == "tests"
        && matches!(components[2].as_str(), "fixtures" | "expected")
}

fn provenance_mentions_toml(provenance: &str, relative_toml_path: &str) -> bool {
    provenance.contains(relative_toml_path)
}

#[test]
fn inline_toml_scanner_catches_zero_hash_raw_string() {
    let source = "fn helper() -> &'static str { r\"\nopenbmp.aero_deck = 1\n[grid]\n\" }";

    let findings = inline_toml_findings(source);

    assert!(
        findings
            .iter()
            .any(|finding| finding.contains("openbmp.aero_deck")),
        "zero-hash raw strings must not bypass inline TOML detection: {findings:?}",
    );
}

#[test]
fn inline_toml_scanner_catches_cooked_multiline_string() {
    let source = "const BAD: &str = \"\nopenbmp.scenario = 1\n[meta]\n\";";

    let findings = inline_toml_findings(source);

    assert!(
        findings
            .iter()
            .any(|finding| finding.contains("openbmp.scenario")),
        "ordinary multiline strings must not bypass inline TOML detection: {findings:?}",
    );
}

#[test]
fn inline_toml_scanner_catches_escaped_newline_string() {
    let source = "const BAD: &str = \"openbmp.motor = 1\\n[motor]\\n\";";

    let findings = inline_toml_findings(source);

    assert!(
        findings
            .iter()
            .any(|finding| finding.contains("openbmp.motor")),
        "escaped-newline strings must not smuggle TOML as one physical line: {findings:?}",
    );
}

#[test]
fn inline_toml_scanner_ignores_single_line_replace_needles() {
    let source = r#"let changed = minimal.replace("openbmp.motor = 1", "openbmp.motor = 2");"#;

    assert!(inline_toml_findings(source).is_empty());
}

#[test]
fn inline_toml_scanner_handles_nested_raw_delimiters_and_utf8() {
    let source = "const BAD: &str = r##\"π r#\"inner\"#\nopenbmp.aero_deck = 1\n\"##;";

    let findings = inline_toml_findings(source);

    assert!(
        findings
            .iter()
            .any(|finding| finding.contains("openbmp.aero_deck")),
        "nested lower-hash raw delimiters and UTF-8 must not hide TOML: {findings:?}",
    );
}

#[test]
fn benchmark_tripwire_matches_underscored_numeric_literals() {
    assert!(contains_tripwire_needle(
        "const X: f64 = 3.986_004_418e14;",
        "3.986004418",
    ));
    assert!(contains_tripwire_needle(
        "const X: f64 = 1.082_626_683e-3;",
        "1.082626683",
    ));
    assert!(contains_tripwire_needle(
        "const X: f64 = 6_378_137.0;",
        "6378137.0",
    ));
}

#[test]
fn include_str_scanner_accepts_whitespace_before_paren() {
    let source = r#"const BAD: &str = include_str! ("__bad/wrong.toml");"#;

    let args = find_include_str_arguments(source);

    assert_eq!(args.len(), 1);
    assert_eq!(
        include_str_literal_path(&args[0]).as_deref(),
        Some("__bad/wrong.toml")
    );
}

#[test]
fn include_str_resolver_rejects_src_path_traversal() {
    let root = PathBuf::from("/repo");
    let source_path = root.join("crates/openbmp-aero/src/__smoke.rs");
    let argument = r#""../tests/fixtures/../fixtures/../../src/__bad/wrong.toml""#;

    let resolved =
        resolve_include_str_path(&root, &source_path, argument).expect("literal path resolves");

    assert_eq!(
        display_path(&root, &resolved),
        "crates/openbmp-aero/src/__bad/wrong.toml",
    );
    assert!(!is_allowed_toml_include(&root, &resolved));
}

#[test]
fn include_str_resolver_accepts_manifest_dir_fixture_paths() {
    let root = workspace_root();
    let source_path = root.join("crates/openbmp-aero/src/parser.rs");
    let argument = r#"concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/minimal-deck.toml")"#;

    let resolved =
        resolve_include_str_path(&root, &source_path, argument).expect("manifest path resolves");

    assert!(is_allowed_toml_include(&root, &resolved));
}

#[test]
fn provenance_audit_requires_file_entry_not_just_sibling_file() {
    assert!(provenance_mentions_toml(
        "files:\n  - data/aero/example.toml\n",
        "data/aero/example.toml",
    ));
    assert!(!provenance_mentions_toml(
        "files:\n  - data/aero/other.toml\n",
        "data/aero/example.toml",
    ));
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
            let Some(resolved_path) = resolve_include_str_path(&root, path, &arg) else {
                violations.entry(rel.clone()).or_default().push(format!(
                    "include_str!(... .toml) could not be resolved: {}",
                    arg.trim().chars().take(120).collect::<String>(),
                ));
                continue;
            };
            if !is_allowed_toml_include(&root, &resolved_path) {
                violations.entry(rel.clone()).or_default().push(format!(
                    "include_str!(... .toml) does not resolve under \
                     tests/fixtures/, tests/expected/, data/, or scenarios/: {} -> {}",
                    arg.trim().chars().take(120).collect::<String>(),
                    display_path(&root, &resolved_path),
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
    // sibling `provenance.md` in the same directory and be listed
    // by repository-relative path in that provenance file. Mirrors
    // the `openbmp check-provenance` CLI command but enforces it
    // under `cargo test` so the contract is checked even when the
    // CLI is not run. Sub-directories may carry their own
    // per-directory provenance.md.
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
            let rel = relative_path(&root, &toml_path);
            if !provenance.is_file() {
                violations
                    .entry((*tree).to_owned())
                    .or_default()
                    .push(format!("{rel} (missing provenance.md)"));
                continue;
            }
            let Ok(provenance_content) = fs::read_to_string(&provenance) else {
                violations
                    .entry((*tree).to_owned())
                    .or_default()
                    .push(format!(
                        "{rel} (could not read {})",
                        relative_path(&root, &provenance),
                    ));
                continue;
            };
            if !provenance_mentions_toml(&provenance_content, &rel) {
                violations
                    .entry((*tree).to_owned())
                    .or_default()
                    .push(format!(
                        "{rel} (not listed in {})",
                        relative_path(&root, &provenance),
                    ));
            }
        }
    }

    if !violations.is_empty() {
        let mut report = String::from(
            "Every `*.toml` under `data/` or `scenarios/` must have a sibling \
             `provenance.md` in the same directory, and that provenance file \
             must list the TOML path.\n\
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
