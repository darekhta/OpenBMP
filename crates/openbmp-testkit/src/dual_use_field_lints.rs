//! Structural tripwires for dual-use-sensitive public field sets.
//!
//! The lexical scenario lint is intentionally broad but it cannot
//! prove the Tier-1 claim that sensitive input types cannot express a
//! target. This module locks the public field set of forward-only
//! trajectory / footprint structs behind a small allowlist. If a
//! future edit adds, removes, or renames one of these fields, CI fails
//! and the dual-use assessment must be reviewed with the code change.

use std::collections::BTreeSet;
use std::path::Path;

/// One struct whose public field set is locked by the audit.
#[derive(Clone, Debug)]
pub struct StructFieldAllowlist {
    /// Rust struct name.
    pub name: &'static str,
    /// Public field names that are allowed on the struct.
    pub fields: &'static [&'static str],
}

/// A source file and the sensitive structs it contains.
#[derive(Clone, Debug)]
pub struct FileFieldAllowlist {
    /// Workspace-relative source path.
    pub path: &'static str,
    /// Sensitive public structs in this file.
    pub structs: &'static [StructFieldAllowlist],
}

/// One enum whose public variant set is locked by the audit.
#[derive(Clone, Debug)]
pub struct EnumVariantAllowlist {
    /// Rust enum name.
    pub name: &'static str,
    /// Public variant names that are allowed on the enum.
    pub variants: &'static [&'static str],
}

/// A source file and the sensitive enums it contains.
#[derive(Clone, Debug)]
pub struct FileEnumAllowlist {
    /// Workspace-relative source path.
    pub path: &'static str,
    /// Sensitive public enums in this file.
    pub enums: &'static [EnumVariantAllowlist],
}

/// Field-set mismatch reported by the audit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FieldAuditMismatch {
    /// The expected source file could not be read.
    MissingFile {
        /// Workspace-relative source path.
        path: &'static str,
        /// I/O error text.
        error: String,
    },
    /// The expected struct declaration was not found.
    MissingStruct {
        /// Workspace-relative source path.
        path: &'static str,
        /// Rust struct name.
        struct_name: &'static str,
    },
    /// The struct exists, but its public field set differs from the
    /// sealed allowlist.
    FieldSetChanged {
        /// Workspace-relative source path.
        path: &'static str,
        /// Rust struct name.
        struct_name: &'static str,
        /// Sorted expected field names.
        expected: Vec<String>,
        /// Sorted actual field names.
        actual: Vec<String>,
    },
    /// The expected enum declaration was not found.
    MissingEnum {
        /// Workspace-relative source path.
        path: &'static str,
        /// Rust enum name.
        enum_name: &'static str,
    },
    /// The enum exists, but its public variant set differs from the
    /// sealed allowlist.
    VariantSetChanged {
        /// Workspace-relative source path.
        path: &'static str,
        /// Rust enum name.
        enum_name: &'static str,
        /// Sorted expected variant names.
        expected: Vec<String>,
        /// Sorted actual variant names.
        actual: Vec<String>,
    },
    /// A public sensitive type exists in a scanned module but is not
    /// covered by either the field or variant audit.
    UnenrolledPublicType {
        /// Workspace-relative source path.
        path: &'static str,
        /// Rust type name.
        type_name: String,
    },
}

/// Result of a structural field audit.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct FieldAuditFindings {
    /// All mismatches found by the audit.
    pub mismatches: Vec<FieldAuditMismatch>,
}

impl FieldAuditFindings {
    /// `true` if the audit found at least one mismatch.
    #[must_use]
    pub fn has_violations(&self) -> bool {
        !self.mismatches.is_empty()
    }
}

/// Dual-use-sensitive public struct field allowlists.
pub const SENSITIVE_FIELD_ALLOWLISTS: &[FileFieldAllowlist] = &[
    FileFieldAllowlist {
        path: "crates/openbmp-physics/src/profile.rs",
        structs: &[
            StructFieldAllowlist {
                name: "BallisticState",
                fields: &[],
            },
            StructFieldAllowlist {
                name: "ConstantGravityRangeSafetyFootprint",
                fields: &[],
            },
            StructFieldAllowlist {
                name: "NumericalGravityRangeSafetyFootprint",
                fields: &[],
            },
            StructFieldAllowlist {
                name: "FootprintGeodeticOrigin",
                fields: &["latitude_deg", "longitude_deg", "height_m"],
            },
            StructFieldAllowlist {
                name: "FootprintDispersionInput",
                fields: &[
                    "one_sigma_semi_major_m",
                    "one_sigma_semi_minor_m",
                    "orientation_rad",
                ],
            },
            StructFieldAllowlist {
                name: "FootprintEnvironment",
                fields: &[
                    "cull_altitude_m",
                    "gravity_m_s2",
                    "launch_origin_eci_m",
                    "geodetic_origin",
                    "dispersion",
                ],
            },
            StructFieldAllowlist {
                name: "FootprintDispersionEllipse",
                fields: &[
                    "one_sigma_semi_major_m",
                    "one_sigma_semi_minor_m",
                    "three_sigma_semi_major_m",
                    "three_sigma_semi_minor_m",
                    "orientation_rad",
                ],
            },
            StructFieldAllowlist {
                name: "LandingFootprint",
                fields: &[
                    "downrange_m",
                    "crossrange_m",
                    "bearing_rad",
                    "time_to_cull_s",
                    "latitude_deg",
                    "longitude_deg",
                    "dispersion_ellipse",
                ],
            },
            StructFieldAllowlist {
                name: "FootprintDragModel",
                fields: &["surface_density_kg_m3", "density_scale_height_m"],
            },
            StructFieldAllowlist {
                name: "FootprintSampleInput",
                fields: &["sample_index", "state", "wind_eci_m_s"],
            },
            StructFieldAllowlist {
                name: "FootprintMonteCarloInput",
                fields: &[
                    "nominal_state",
                    "samples",
                    "confidence_levels",
                    "drag",
                    "step_s",
                    "max_time_s",
                ],
            },
            StructFieldAllowlist {
                name: "FootprintSample",
                fields: &["sample_index", "landing"],
            },
            StructFieldAllowlist {
                name: "FootprintSampleFailure",
                fields: &["sample_index", "reason"],
            },
            StructFieldAllowlist {
                name: "FootprintQuantile",
                fields: &["confidence_level", "radial_distance_m"],
            },
            StructFieldAllowlist {
                name: "FootprintMonteCarloResult",
                fields: &[
                    "nominal",
                    "samples",
                    "failures",
                    "mean_downrange_m",
                    "mean_crossrange_m",
                    "radial_dispersion_p50_m",
                    "mean_offset_downrange_from_nominal_m",
                    "mean_offset_crossrange_from_nominal_m",
                    "mean_radial_offset_from_nominal_m",
                    "covariance_downrange_downrange_m2",
                    "covariance_downrange_crossrange_m2",
                    "covariance_crossrange_crossrange_m2",
                    "dispersion_ellipse",
                    "quantiles",
                    "nominal_radial_error_quantiles",
                ],
            },
        ],
    },
    FileFieldAllowlist {
        path: "crates/openbmp-scenario/src/document.rs",
        structs: &[
            StructFieldAllowlist {
                name: "ScenarioDirectorConfig",
                fields: &["mission_authority"],
            },
            StructFieldAllowlist {
                name: "LandingFootprintConfig",
                fields: &[
                    "method",
                    "cull_altitude_m",
                    "include_geodetic",
                    "dispersion",
                    "monte_carlo",
                ],
            },
            StructFieldAllowlist {
                name: "LandingFootprintDispersionConfig",
                fields: &[
                    "one_sigma_semi_major_m",
                    "one_sigma_semi_minor_m",
                    "orientation_rad",
                ],
            },
            StructFieldAllowlist {
                name: "LandingFootprintMonteCarloConfig",
                fields: &[
                    "samples",
                    "seed",
                    "confidence_levels",
                    "output",
                    "wind",
                    "ballistic_coefficient",
                    "burnout_state",
                ],
            },
            StructFieldAllowlist {
                name: "LandingFootprintMonteCarloOutputConfig",
                fields: &["samples_csv", "samples_parquet", "summary_toml"],
            },
            StructFieldAllowlist {
                name: "LandingFootprintMonteCarloWindConfig",
                fields: &[
                    "kind",
                    "sigma_ned_m_s",
                    "speed_scale_sigma",
                    "ensemble_members_ned_m_s",
                ],
            },
            StructFieldAllowlist {
                name: "LandingFootprintMonteCarloBallisticCoefficientConfig",
                fields: &[
                    "nominal_m2_kg",
                    "sigma_m2_kg",
                    "distribution",
                    "min_m2_kg",
                    "max_m2_kg",
                ],
            },
            StructFieldAllowlist {
                name: "LandingFootprintMonteCarloBurnoutStateConfig",
                fields: &[
                    "position_sigma_eci_m",
                    "velocity_sigma_eci_m_s",
                    "time_sigma_s",
                ],
            },
            StructFieldAllowlist {
                name: "StagingAnalysisConfig",
                fields: &["mode", "delta_v_budget_m_s", "payload_mass_kg", "stages"],
            },
            StructFieldAllowlist {
                name: "StagingAnalysisStageConfig",
                fields: &[
                    "isp_s",
                    "structural_coefficient",
                    "structural_mass_kg",
                    "propellant_mass_kg",
                ],
            },
        ],
    },
];

/// Dual-use-sensitive public enum variant allowlists.
pub const SENSITIVE_ENUM_ALLOWLISTS: &[FileEnumAllowlist] = &[
    FileEnumAllowlist {
        path: "crates/openbmp-physics/src/profile.rs",
        enums: &[
            EnumVariantAllowlist {
                name: "TerminalCondition",
                variants: &[
                    "OrbitalElements",
                    "ApogeeRadius",
                    "FlightPathAngleAtBurnout",
                    "RendezvousState",
                    "MaximizePayloadMass",
                ],
            },
            EnumVariantAllowlist {
                name: "DispersionSource",
                variants: &[
                    "Wind",
                    "BallisticCoefficient",
                    "VehicleMass",
                    "ThrustScale",
                    "SensorNoiseScale",
                    "ActuatorLag",
                ],
            },
        ],
    },
    FileEnumAllowlist {
        path: "crates/openbmp-scenario/src/document.rs",
        enums: &[
            EnumVariantAllowlist {
                name: "LandingFootprintMethod",
                variants: &["ConstantGravity", "J2", "Egm2008"],
            },
            EnumVariantAllowlist {
                name: "LandingFootprintMonteCarloWindKind",
                variants: &["Constant", "Layered", "Hwm14", "Ensemble"],
            },
            EnumVariantAllowlist {
                name: "LandingFootprintMonteCarloDistribution",
                variants: &["Normal", "Uniform"],
            },
            EnumVariantAllowlist {
                name: "StagingAnalysisMode",
                variants: &["Budget", "Optimal"],
            },
        ],
    },
];

/// Audit the workspace's dual-use-sensitive struct field sets.
#[must_use]
pub fn audit_workspace(root: &Path) -> FieldAuditFindings {
    let mut findings = FieldAuditFindings::default();
    for file in SENSITIVE_FIELD_ALLOWLISTS {
        let path = root.join(file.path);
        let contents = match std::fs::read_to_string(&path) {
            Ok(contents) => contents,
            Err(error) => {
                findings.mismatches.push(FieldAuditMismatch::MissingFile {
                    path: file.path,
                    error: error.to_string(),
                });
                continue;
            }
        };
        for struct_allowlist in file.structs {
            let Some(actual) = public_struct_fields(&contents, struct_allowlist.name) else {
                findings.mismatches.push(FieldAuditMismatch::MissingStruct {
                    path: file.path,
                    struct_name: struct_allowlist.name,
                });
                continue;
            };
            let expected = sorted_field_names(struct_allowlist.fields);
            if actual != expected {
                findings
                    .mismatches
                    .push(FieldAuditMismatch::FieldSetChanged {
                        path: file.path,
                        struct_name: struct_allowlist.name,
                        expected,
                        actual,
                    });
            }
        }
    }
    for file in SENSITIVE_ENUM_ALLOWLISTS {
        let path = root.join(file.path);
        let contents = match std::fs::read_to_string(&path) {
            Ok(contents) => contents,
            Err(error) => {
                findings.mismatches.push(FieldAuditMismatch::MissingFile {
                    path: file.path,
                    error: error.to_string(),
                });
                continue;
            }
        };
        for enum_allowlist in file.enums {
            let Some(actual) = public_enum_variants(&contents, enum_allowlist.name) else {
                findings.mismatches.push(FieldAuditMismatch::MissingEnum {
                    path: file.path,
                    enum_name: enum_allowlist.name,
                });
                continue;
            };
            let expected = sorted_field_names(enum_allowlist.variants);
            if actual != expected {
                findings
                    .mismatches
                    .push(FieldAuditMismatch::VariantSetChanged {
                        path: file.path,
                        enum_name: enum_allowlist.name,
                        expected,
                        actual,
                    });
            }
        }
    }
    audit_enrolled_sensitive_types(root, &mut findings);
    findings
}

fn sorted_field_names(fields: &[&str]) -> Vec<String> {
    let mut names: Vec<String> = fields.iter().map(|field| (*field).to_owned()).collect();
    names.sort();
    names
}

fn public_struct_fields(source: &str, struct_name: &str) -> Option<Vec<String>> {
    let start = find_pub_struct(source, struct_name)?;
    let after = &source[start..];
    let semicolon = after.find(';');
    let open = after.find('{')?;
    if semicolon.is_some_and(|index| index < open) {
        return Some(Vec::new());
    }
    let body_start = start + open + 1;
    let mut depth = 1usize;
    let mut body_end = None;
    for (offset, ch) in source[body_start..].char_indices() {
        match ch {
            '{' => depth = depth.saturating_add(1),
            '}' => {
                depth = depth.saturating_sub(1);
                if depth == 0 {
                    body_end = Some(body_start + offset);
                    break;
                }
            }
            _ => {}
        }
    }
    let body = &source[body_start..body_end?];
    let mut fields = Vec::new();
    for line in body.lines() {
        let trimmed = line.trim_start();
        let Some(rest) = trimmed.strip_prefix("pub ") else {
            continue;
        };
        let Some((field, _)) = rest.split_once(':') else {
            continue;
        };
        let field = field.trim();
        if !field.is_empty()
            && field
                .chars()
                .all(|ch| ch == '_' || ch.is_ascii_alphanumeric())
        {
            fields.push(field.to_owned());
        }
    }
    fields.sort();
    Some(fields)
}

fn find_pub_struct(source: &str, struct_name: &str) -> Option<usize> {
    let needle = format!("pub struct {struct_name}");
    let mut search_start = 0usize;
    while let Some(relative) = source[search_start..].find(&needle) {
        let start = search_start + relative;
        let end = start + needle.len();
        let boundary_ok = source[end..]
            .chars()
            .next()
            .is_none_or(|ch| !(ch == '_' || ch.is_ascii_alphanumeric()));
        if boundary_ok {
            return Some(start);
        }
        search_start = end;
    }
    None
}

fn public_enum_variants(source: &str, enum_name: &str) -> Option<Vec<String>> {
    let start = find_pub_enum(source, enum_name)?;
    let after = &source[start..];
    let open = after.find('{')?;
    let body_start = start + open + 1;
    let mut depth = 1usize;
    let mut body_end = None;
    for (offset, ch) in source[body_start..].char_indices() {
        match ch {
            '{' => depth = depth.saturating_add(1),
            '}' => {
                depth = depth.saturating_sub(1);
                if depth == 0 {
                    body_end = Some(body_start + offset);
                    break;
                }
            }
            _ => {}
        }
    }
    let body = &source[body_start..body_end?];
    let mut variants = Vec::new();
    for line in body.lines() {
        let trimmed = line.trim_start();
        if trimmed.is_empty()
            || trimmed.starts_with('#')
            || trimmed.starts_with("//")
            || trimmed.starts_with("///")
            || trimmed.starts_with("pub ")
        {
            continue;
        }
        let name = trimmed
            .split(|ch: char| {
                ch == '{' || ch == '(' || ch == '=' || ch == ',' || ch.is_whitespace()
            })
            .next()
            .unwrap_or_default();
        if name
            .chars()
            .next()
            .is_some_and(|ch| ch.is_ascii_uppercase())
            && name
                .chars()
                .all(|ch| ch == '_' || ch.is_ascii_alphanumeric())
        {
            variants.push(name.to_owned());
        }
    }
    variants.sort();
    Some(variants)
}

fn find_pub_enum(source: &str, enum_name: &str) -> Option<usize> {
    find_pub_type(source, "enum", enum_name)
}

fn find_pub_type(source: &str, kind: &str, type_name: &str) -> Option<usize> {
    let needle = format!("pub {kind} {type_name}");
    let mut search_start = 0usize;
    while let Some(relative) = source[search_start..].find(&needle) {
        let start = search_start + relative;
        let end = start + needle.len();
        let boundary_ok = source[end..]
            .chars()
            .next()
            .is_none_or(|ch| !(ch == '_' || ch.is_ascii_alphanumeric()));
        if boundary_ok {
            return Some(start);
        }
        search_start = end;
    }
    None
}

fn audit_enrolled_sensitive_types(root: &Path, findings: &mut FieldAuditFindings) {
    let field_names: BTreeSet<&str> = SENSITIVE_FIELD_ALLOWLISTS
        .iter()
        .flat_map(|file| file.structs.iter().map(|item| item.name))
        .collect();
    let enum_names: BTreeSet<&str> = SENSITIVE_ENUM_ALLOWLISTS
        .iter()
        .flat_map(|file| file.enums.iter().map(|item| item.name))
        .collect();
    let mut audited_paths: BTreeSet<&str> = SENSITIVE_FIELD_ALLOWLISTS
        .iter()
        .map(|file| file.path)
        .collect();
    audited_paths.extend(SENSITIVE_ENUM_ALLOWLISTS.iter().map(|file| file.path));

    for path in audited_paths {
        let file_path = root.join(path);
        let Ok(contents) = std::fs::read_to_string(&file_path) else {
            continue;
        };
        for type_name in public_type_names(&contents) {
            if !is_dual_use_sensitive_type_name(&type_name) {
                continue;
            }
            if !field_names.contains(type_name.as_str()) && !enum_names.contains(type_name.as_str())
            {
                findings
                    .mismatches
                    .push(FieldAuditMismatch::UnenrolledPublicType { path, type_name });
            }
        }
    }
}

fn public_type_names(source: &str) -> Vec<String> {
    let mut names = Vec::new();
    for kind in ["struct", "enum"] {
        let needle = format!("pub {kind} ");
        let mut search_start = 0usize;
        while let Some(relative) = source[search_start..].find(&needle) {
            let start = search_start + relative + needle.len();
            let tail = &source[start..];
            let name = tail
                .split(|ch: char| {
                    ch == '<' || ch == '{' || ch == ';' || ch == '(' || ch.is_whitespace()
                })
                .next()
                .unwrap_or_default();
            if !name.is_empty()
                && name
                    .chars()
                    .all(|ch| ch == '_' || ch.is_ascii_alphanumeric())
            {
                names.push(name.to_owned());
            }
            search_start = start + name.len();
        }
    }
    names.sort();
    names.dedup();
    names
}

fn is_dual_use_sensitive_type_name(name: &str) -> bool {
    name == "TerminalCondition"
        || name == "DispersionSource"
        || name.starts_with("Ballistic")
        || name.starts_with("Footprint")
        || name.starts_with("LandingFootprint")
        || name.starts_with("StagingAnalysis")
        || name.contains("RangeSafetyFootprint")
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn project_root() -> PathBuf {
        let here = env!("CARGO_MANIFEST_DIR");
        Path::new(here)
            .parent()
            .and_then(Path::parent)
            .expect("workspace root")
            .to_path_buf()
    }

    #[test]
    fn dual_use_sensitive_public_field_sets_match_allowlist() {
        let root = project_root();
        let findings = audit_workspace(&root);
        if findings.has_violations() {
            let mut lines = Vec::new();
            for mismatch in &findings.mismatches {
                match mismatch {
                    FieldAuditMismatch::MissingFile { path, error } => {
                        lines.push(format!("  {path}: missing file ({error})"));
                    }
                    FieldAuditMismatch::MissingStruct { path, struct_name } => {
                        lines.push(format!("  {path}: missing struct {struct_name}"));
                    }
                    FieldAuditMismatch::FieldSetChanged {
                        path,
                        struct_name,
                        expected,
                        actual,
                    } => {
                        lines.push(format!(
                            "  {path}: {struct_name} field set changed\n    expected: {expected:?}\n    actual:   {actual:?}",
                        ));
                    }
                    FieldAuditMismatch::MissingEnum { path, enum_name } => {
                        lines.push(format!("  {path}: missing enum {enum_name}"));
                    }
                    FieldAuditMismatch::VariantSetChanged {
                        path,
                        enum_name,
                        expected,
                        actual,
                    } => {
                        lines.push(format!(
                            "  {path}: {enum_name} variant set changed\n    expected: {expected:?}\n    actual:   {actual:?}",
                        ));
                    }
                    FieldAuditMismatch::UnenrolledPublicType { path, type_name } => {
                        lines.push(format!(
                            "  {path}: public sensitive type {type_name} is not enrolled in the field/variant audit",
                        ));
                    }
                }
            }
            panic!(
                "dual-use field audit failed; update the allowlist only with a \
                 dual-use assessment review:\n{}",
                lines.join("\n")
            );
        }
    }

    #[test]
    fn field_extractor_reports_public_fields() {
        let source = r"
#[derive(Clone)]
pub struct Example {
    /// visible
    pub alpha: f64,
    #[serde(default)]
    pub beta_value: Option<u32>,
    private: bool,
}
";
        let fields = public_struct_fields(source, "Example").expect("fields");
        assert_eq!(fields, vec!["alpha".to_owned(), "beta_value".to_owned()]);
    }

    #[test]
    fn enum_extractor_reports_variants() {
        let source = r#"
pub enum TerminalCondition {
    /// doc
    OrbitalElements { semi_major_axis_m: f64 },
    #[serde(rename = "apogee_radius")]
    ApogeeRadius { radius_m: f64 },
    MaximizePayloadMass,
}
"#;
        let variants = public_enum_variants(source, "TerminalCondition").expect("variants");
        assert_eq!(
            variants,
            vec![
                "ApogeeRadius".to_owned(),
                "MaximizePayloadMass".to_owned(),
                "OrbitalElements".to_owned(),
            ]
        );
    }

    #[test]
    fn sensitive_type_detector_catches_unenrolled_names() {
        assert!(is_dual_use_sensitive_type_name("TerminalCondition"));
        assert!(is_dual_use_sensitive_type_name("FootprintMonteCarloInput"));
        assert!(!is_dual_use_sensitive_type_name("AscentState"));
    }
}
