//! FMI 3 co-simulation export artifact helpers.
//!
//! These helpers generate a restricted `modelDescription.xml` and a
//! stored-entry `.fmu` package around a caller-supplied shared-library
//! payload. They intentionally stop at deterministic artifact
//! construction and in-repo metadata round-trip checks; the repository
//! schema script runs the generated point-mass metadata against the
//! official FMI 3.0 XSD set. `fmpy` execution and full FMI conformance
//! remain external conformance work.

use std::collections::BTreeSet;
use std::path::Path;

use openbmp_models::{FmiScalarVariable, FmiVariableCausality, FmiVariableType};
use thiserror::Error;

/// Restricted OpenBMP point-mass export FMI `modelName`.
pub const OPENBMP_POINT_MASS_MODEL_NAME: &str = "OpenBMP point mass";
/// Restricted OpenBMP point-mass export FMI co-simulation model identifier.
pub const OPENBMP_POINT_MASS_MODEL_IDENTIFIER: &str = "openbmp_point_mass";
/// Restricted OpenBMP point-mass export FMI instantiation token.
pub const OPENBMP_POINT_MASS_INSTANTIATION_TOKEN: &str = "openbmp-point-mass-restricted-fmi3-cs";
/// Value reference for exported point-mass simulation time.
pub const OPENBMP_POINT_MASS_VR_TIME_S: u32 = 0;
/// Value reference for exported point-mass throttle input.
pub const OPENBMP_POINT_MASS_VR_THROTTLE: u32 = 10;
/// Value reference for exported point-mass altitude output.
pub const OPENBMP_POINT_MASS_VR_ALTITUDE_M: u32 = 20;
/// Value reference for exported point-mass step counter output.
pub const OPENBMP_POINT_MASS_VR_STEP: u32 = 21;
/// Value reference for exported point-mass velocity output.
pub const OPENBMP_POINT_MASS_VR_VELOCITY_M_S: u32 = 22;

/// FMI exporter artifact construction error.
#[derive(Debug, Error)]
pub enum FmiExportError {
    /// Required string field was empty.
    #[error("FMI export field {field} must not be empty")]
    EmptyField {
        /// Field path.
        field: &'static str,
    },
    /// Two scalar variables used the same name.
    #[error("FMI export scalar variable name {name} is duplicated")]
    DuplicateVariableName {
        /// Duplicated variable name.
        name: String,
    },
    /// Two scalar variables used the same value reference.
    #[error("FMI export valueReference {value_reference} is duplicated")]
    DuplicateValueReference {
        /// Duplicated value reference.
        value_reference: u32,
    },
    /// FMU binary archive entry is unsafe or invalid.
    #[error("FMI export binary entry {entry} is invalid")]
    InvalidBinaryEntry {
        /// Archive entry path.
        entry: String,
    },
    /// Generated or supplied restricted `modelDescription.xml` did not
    /// satisfy the OpenBMP FMI 3 export contract.
    #[error("restricted FMI 3 modelDescription.xml is invalid: {reason}")]
    InvalidModelDescription {
        /// Validation failure summary.
        reason: String,
    },
    /// Current compilation target has no supported FMU binary entry.
    #[error("FMI export has no binary entry for current target {arch}-{os}")]
    UnsupportedCurrentPlatform {
        /// Rust target architecture.
        arch: &'static str,
        /// Rust target operating system.
        os: &'static str,
    },
    /// FMU archive entry path is too large for the restricted ZIP writer.
    #[error("FMI export archive entry {entry} exceeds ZIP name length limit")]
    EntryNameTooLong {
        /// Archive entry path.
        entry: String,
    },
    /// FMU archive entry payload is too large for the restricted ZIP writer.
    #[error("FMI export archive entry {entry} exceeds ZIP size limit")]
    EntryTooLarge {
        /// Archive entry path.
        entry: String,
    },
    /// Filesystem IO failed while writing the FMU artifact.
    #[error("FMI export could not write {path}: {source}")]
    Io {
        /// Destination path.
        path: String,
        /// Underlying IO error.
        #[source]
        source: std::io::Error,
    },
}

/// Metadata needed to emit an OpenBMP FMI 3 co-simulation export artifact.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Fmi3ExportModel {
    /// FMI `modelName`.
    pub model_name: String,
    /// FMI `CoSimulation modelIdentifier`.
    pub model_identifier: String,
    /// FMI root `instantiationToken`.
    pub instantiation_token: String,
    /// Typed scalar variables and value references exported by the model.
    pub scalar_variables: Vec<FmiScalarVariable>,
}

/// Parsed summary of a restricted OpenBMP FMI 3 `modelDescription.xml`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RestrictedFmi3ModelDescription {
    /// FMI `modelName`.
    pub model_name: String,
    /// FMI root `instantiationToken`.
    pub instantiation_token: String,
    /// FMI `CoSimulation modelIdentifier`.
    pub model_identifier: String,
    /// Direct typed scalar variables in `ModelVariables` order.
    pub scalar_variables: Vec<FmiScalarVariable>,
    /// Output value references in `ModelStructure` order.
    pub output_value_references: Vec<u32>,
}

impl Fmi3ExportModel {
    /// Construct an export model with no scalar variables.
    #[must_use]
    pub fn new(model_name: impl Into<String>, model_identifier: impl Into<String>) -> Self {
        let model_identifier = model_identifier.into();
        Self {
            model_name: model_name.into(),
            instantiation_token: format!("openbmp-{model_identifier}-restricted-fmi3-cs"),
            model_identifier,
            scalar_variables: Vec::new(),
        }
    }

    /// Replace the FMI root `instantiationToken`.
    #[must_use]
    pub fn with_instantiation_token(mut self, instantiation_token: impl Into<String>) -> Self {
        self.instantiation_token = instantiation_token.into();
        self
    }

    /// Append one typed scalar variable.
    #[must_use]
    pub fn with_scalar_variable(mut self, variable: FmiScalarVariable) -> Self {
        self.scalar_variables.push(variable);
        self
    }

    /// Generate `modelDescription.xml` for this export model.
    ///
    /// # Errors
    ///
    /// Returns [`FmiExportError`] when required identifiers are empty
    /// or scalar variables contain duplicate names/value references.
    pub fn model_description_xml(&self) -> Result<String, FmiExportError> {
        self.validate()?;
        let mut xml = String::new();
        xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        xml.push_str("<fmiModelDescription fmiVersion=\"3.0\" modelName=\"");
        xml.push_str(&escape_xml_attr(&self.model_name));
        xml.push_str("\" instantiationToken=\"");
        xml.push_str(&escape_xml_attr(&self.instantiation_token));
        xml.push_str("\" variableNamingConvention=\"flat\">\n");
        xml.push_str("  <CoSimulation modelIdentifier=\"");
        xml.push_str(&escape_xml_attr(&self.model_identifier));
        xml.push_str(
            "\" canHandleVariableCommunicationStepSize=\"true\" canGetAndSetFMUState=\"true\"/>\n",
        );
        xml.push_str("  <ModelVariables>\n");
        for variable in &self.scalar_variables {
            xml.push_str("    <");
            xml.push_str(variable.variable_type.as_str());
            xml.push_str(" name=\"");
            xml.push_str(&escape_xml_attr(&variable.name));
            xml.push_str("\" valueReference=\"");
            xml.push_str(&variable.value_reference.to_string());
            xml.push_str("\" causality=\"");
            xml.push_str(variable.causality.as_str());
            xml.push('"');
            append_initial_start_attrs(&mut xml, variable);
            xml.push_str("/>\n");
        }
        xml.push_str("  </ModelVariables>\n");
        xml.push_str("  <ModelStructure>\n");
        for variable in &self.scalar_variables {
            if variable.causality == FmiVariableCausality::Output {
                xml.push_str("    <Output valueReference=\"");
                xml.push_str(&variable.value_reference.to_string());
                xml.push_str("\"/>\n");
            }
        }
        xml.push_str("  </ModelStructure>\n");
        xml.push_str("</fmiModelDescription>\n");
        validate_restricted_fmi3_model_description_xml(&xml)?;
        Ok(xml)
    }

    /// Write a stored-entry `.fmu` archive containing
    /// `modelDescription.xml` and one caller-supplied binary payload.
    ///
    /// `binary_entry` should be an FMU archive path such as
    /// `binaries/x86_64-linux/libopenbmp_fmi.so`. The payload is not
    /// inspected by this helper.
    ///
    /// # Errors
    ///
    /// Returns [`FmiExportError`] when the model description is invalid,
    /// the binary entry path is unsafe, or filesystem writing fails.
    pub fn write_fmu_archive(
        &self,
        path: impl AsRef<Path>,
        binary_entry: &str,
        binary_payload: &[u8],
    ) -> Result<(), FmiExportError> {
        validate_binary_entry(binary_entry)?;
        let model_description = self.model_description_xml()?;
        let entries = [
            ("modelDescription.xml", model_description.as_bytes()),
            (binary_entry, binary_payload),
        ];
        write_stored_zip(path, &entries)
    }

    /// Write a stored-entry `.fmu` archive for the current host target.
    ///
    /// The binary payload is written under the OpenBMP-supported FMI
    /// platform entry for the current Rust target. The returned string is
    /// the archive entry path that was written, suitable for
    /// [`crate::Fmi3DynamicLibrary::open_archive_binary`].
    ///
    /// # Errors
    ///
    /// Returns [`FmiExportError`] when the current target is unsupported,
    /// the model description is invalid, or archive writing fails.
    pub fn write_current_platform_fmu_archive(
        &self,
        path: impl AsRef<Path>,
        binary_payload: &[u8],
    ) -> Result<&'static str, FmiExportError> {
        let binary_entry = openbmp_fmi_binary_entry_for_current_platform().ok_or(
            FmiExportError::UnsupportedCurrentPlatform {
                arch: std::env::consts::ARCH,
                os: std::env::consts::OS,
            },
        )?;
        self.write_fmu_archive(path, binary_entry, binary_payload)?;
        Ok(binary_entry)
    }

    fn validate(&self) -> Result<(), FmiExportError> {
        require_non_empty("model_name", &self.model_name)?;
        require_non_empty("model_identifier", &self.model_identifier)?;
        require_non_empty("instantiation_token", &self.instantiation_token)?;
        let mut names = BTreeSet::new();
        let mut value_references = BTreeSet::new();
        for variable in &self.scalar_variables {
            require_non_empty("scalar_variables.name", &variable.name)?;
            if !names.insert(variable.name.clone()) {
                return Err(FmiExportError::DuplicateVariableName {
                    name: variable.name.clone(),
                });
            }
            if !value_references.insert(variable.value_reference) {
                return Err(FmiExportError::DuplicateValueReference {
                    value_reference: variable.value_reference,
                });
            }
        }
        Ok(())
    }
}

/// Validate the restricted FMI 3 `modelDescription.xml` contract emitted
/// by OpenBMP export helpers.
///
/// This is not a complete FMI XSD validator. It checks the subset that
/// OpenBMP currently emits and consumes: FMI 3 root metadata,
/// `CoSimulation`, direct typed `Float64`/`Int32`/`UInt64` variables, unique
/// names/value references, no legacy `ScalarVariable` wrappers, and a
/// `ModelStructure` output list matching output variables.
///
/// # Errors
///
/// Returns [`FmiExportError::InvalidModelDescription`] when the XML does
/// not satisfy the restricted contract.
pub fn validate_restricted_fmi3_model_description_xml(
    xml: &str,
) -> Result<RestrictedFmi3ModelDescription, FmiExportError> {
    let root = required_xml_tag(xml, "fmiModelDescription")?;
    let fmi_version = required_xml_attr(root, "fmiVersion", "fmiModelDescription")?;
    if fmi_version != "3.0" {
        return invalid_model_description(format!(
            "fmiModelDescription fmiVersion must be 3.0, got {fmi_version}"
        ));
    }
    let model_name = required_xml_attr(root, "modelName", "fmiModelDescription")?;
    require_xml_attr_non_empty("modelName", &model_name)?;
    let instantiation_token = required_xml_attr(root, "instantiationToken", "fmiModelDescription")?;
    require_xml_attr_non_empty("instantiationToken", &instantiation_token)?;

    let co_sim = required_xml_tag(xml, "CoSimulation")?;
    let model_identifier = required_xml_attr(co_sim, "modelIdentifier", "CoSimulation")?;
    require_xml_attr_non_empty("modelIdentifier", &model_identifier)?;

    let model_variables = required_xml_element_body(xml, "ModelVariables")?;
    if model_variables.contains("<ScalarVariable") {
        return invalid_model_description(
            "ModelVariables must use direct FMI 3 typed variable elements".to_owned(),
        );
    }
    let scalar_variables = parse_restricted_typed_variables(model_variables)?;
    if scalar_variables.is_empty() {
        return invalid_model_description("ModelVariables must contain at least one variable");
    }
    validate_unique_restricted_variables(&scalar_variables)?;

    let model_structure = required_xml_element_body(xml, "ModelStructure")?;
    let output_value_references = parse_restricted_output_value_references(model_structure)?;
    validate_model_structure_outputs(&scalar_variables, &output_value_references)?;

    Ok(RestrictedFmi3ModelDescription {
        model_name,
        instantiation_token,
        model_identifier,
        scalar_variables,
        output_value_references,
    })
}

/// Construct a typed FMI scalar variable for export metadata.
#[must_use]
pub fn fmi3_export_scalar_variable(
    name: impl Into<String>,
    value_reference: u32,
    causality: FmiVariableCausality,
    variable_type: FmiVariableType,
) -> FmiScalarVariable {
    FmiScalarVariable {
        name: name.into(),
        value_reference,
        causality,
        variable_type,
    }
}

/// Build the restricted point-mass export metadata used by the
/// `openbmp-fmi` C ABI smoke surface.
///
/// This keeps generated `modelDescription.xml` value references aligned
/// with the symbols implemented in [`crate::export_abi`].
#[must_use]
pub fn openbmp_point_mass_export_model() -> Fmi3ExportModel {
    Fmi3ExportModel::new(
        OPENBMP_POINT_MASS_MODEL_NAME,
        OPENBMP_POINT_MASS_MODEL_IDENTIFIER,
    )
    .with_instantiation_token(OPENBMP_POINT_MASS_INSTANTIATION_TOKEN)
    .with_scalar_variable(fmi3_export_scalar_variable(
        "time",
        OPENBMP_POINT_MASS_VR_TIME_S,
        FmiVariableCausality::Independent,
        FmiVariableType::Float64,
    ))
    .with_scalar_variable(fmi3_export_scalar_variable(
        "throttle",
        OPENBMP_POINT_MASS_VR_THROTTLE,
        FmiVariableCausality::Input,
        FmiVariableType::Float64,
    ))
    .with_scalar_variable(fmi3_export_scalar_variable(
        "altitude",
        OPENBMP_POINT_MASS_VR_ALTITUDE_M,
        FmiVariableCausality::Output,
        FmiVariableType::Float64,
    ))
    .with_scalar_variable(fmi3_export_scalar_variable(
        "step",
        OPENBMP_POINT_MASS_VR_STEP,
        FmiVariableCausality::Output,
        FmiVariableType::UInt64,
    ))
    .with_scalar_variable(fmi3_export_scalar_variable(
        "velocity",
        OPENBMP_POINT_MASS_VR_VELOCITY_M_S,
        FmiVariableCausality::Output,
        FmiVariableType::Float64,
    ))
}

/// Return the FMU archive entry for the `openbmp-fmi` cdylib on the
/// current compilation target.
#[must_use]
pub fn openbmp_fmi_binary_entry_for_current_platform() -> Option<&'static str> {
    openbmp_fmi_binary_entry_for_target(std::env::consts::ARCH, std::env::consts::OS)
}

/// Return the FMU archive entry for the OpenBMP point-mass export
/// binary on the current compilation target.
#[must_use]
pub fn openbmp_point_mass_fmi_binary_entry_for_current_platform() -> Option<&'static str> {
    openbmp_point_mass_fmi_binary_entry_for_target(std::env::consts::ARCH, std::env::consts::OS)
}

/// Return the FMU archive entry for the `openbmp-fmi` cdylib on a
/// supported target.
///
/// The returned path follows the restricted exporter convention used by
/// OpenBMP FMU smoke artifacts:
/// `binaries/<fmi-platform>/libopenbmp_fmi.<ext>`.
#[must_use]
pub fn openbmp_fmi_binary_entry_for_target(
    target_arch: &str,
    target_os: &str,
) -> Option<&'static str> {
    match (target_arch, target_os) {
        ("x86_64", "linux") => Some("binaries/x86_64-linux/libopenbmp_fmi.so"),
        ("aarch64", "linux") => Some("binaries/aarch64-linux/libopenbmp_fmi.so"),
        ("x86_64", "macos") => Some("binaries/x86_64-darwin/libopenbmp_fmi.dylib"),
        ("aarch64", "macos") => Some("binaries/aarch64-darwin/libopenbmp_fmi.dylib"),
        ("x86_64", "windows") => Some("binaries/x86_64-windows/openbmp_fmi.dll"),
        ("aarch64", "windows") => Some("binaries/aarch64-windows/openbmp_fmi.dll"),
        _ => None,
    }
}

/// Return the FMU archive entry expected by standard FMI importers for
/// the OpenBMP point-mass export model identifier.
#[must_use]
pub fn openbmp_point_mass_fmi_binary_entry_for_target(
    target_arch: &str,
    target_os: &str,
) -> Option<&'static str> {
    match (target_arch, target_os) {
        ("x86_64", "linux") => Some("binaries/x86_64-linux/openbmp_point_mass.so"),
        ("aarch64", "linux") => Some("binaries/aarch64-linux/openbmp_point_mass.so"),
        ("x86_64", "macos") => Some("binaries/x86_64-darwin/openbmp_point_mass.dylib"),
        ("aarch64", "macos") => Some("binaries/aarch64-darwin/openbmp_point_mass.dylib"),
        ("x86_64", "windows") => Some("binaries/x86_64-windows/openbmp_point_mass.dll"),
        ("aarch64", "windows") => Some("binaries/aarch64-windows/openbmp_point_mass.dll"),
        _ => None,
    }
}

/// Write a restricted OpenBMP point-mass `.fmu` archive for the current
/// host target and return the binary entry path that was written.
///
/// # Errors
///
/// Returns [`FmiExportError`] when the current target is unsupported or
/// the archive cannot be written.
pub fn write_openbmp_point_mass_fmu_archive_for_current_platform(
    path: impl AsRef<Path>,
    binary_payload: &[u8],
) -> Result<&'static str, FmiExportError> {
    let binary_entry = openbmp_point_mass_fmi_binary_entry_for_current_platform().ok_or(
        FmiExportError::UnsupportedCurrentPlatform {
            arch: std::env::consts::ARCH,
            os: std::env::consts::OS,
        },
    )?;
    openbmp_point_mass_export_model().write_fmu_archive(path, binary_entry, binary_payload)?;
    Ok(binary_entry)
}

fn require_non_empty(field: &'static str, value: &str) -> Result<(), FmiExportError> {
    if value.is_empty() {
        Err(FmiExportError::EmptyField { field })
    } else {
        Ok(())
    }
}

fn validate_unique_restricted_variables(
    scalar_variables: &[FmiScalarVariable],
) -> Result<(), FmiExportError> {
    let mut names = BTreeSet::new();
    let mut value_references = BTreeSet::new();
    for variable in scalar_variables {
        require_xml_attr_non_empty("variable name", &variable.name)?;
        if !names.insert(variable.name.clone()) {
            return invalid_model_description(format!(
                "variable name {} is duplicated",
                variable.name
            ));
        }
        if !value_references.insert(variable.value_reference) {
            return invalid_model_description(format!(
                "valueReference {} is duplicated",
                variable.value_reference
            ));
        }
    }
    Ok(())
}

fn validate_model_structure_outputs(
    scalar_variables: &[FmiScalarVariable],
    output_value_references: &[u32],
) -> Result<(), FmiExportError> {
    let expected = scalar_variables
        .iter()
        .filter(|variable| variable.causality == FmiVariableCausality::Output)
        .map(|variable| variable.value_reference)
        .collect::<BTreeSet<_>>();
    let actual = output_value_references
        .iter()
        .copied()
        .collect::<BTreeSet<_>>();
    if expected != actual {
        return invalid_model_description(format!(
            "ModelStructure outputs {:?} do not match output variables {:?}",
            actual, expected
        ));
    }
    if actual.len() != output_value_references.len() {
        return invalid_model_description(
            "ModelStructure contains duplicate Output valueReference entries".to_owned(),
        );
    }
    Ok(())
}

fn append_initial_start_attrs(xml: &mut String, variable: &FmiScalarVariable) {
    if matches!(
        variable.causality,
        FmiVariableCausality::Input | FmiVariableCausality::Parameter
    ) {
        xml.push_str(" initial=\"exact\" start=\"");
        xml.push_str(match variable.variable_type {
            FmiVariableType::Float64 => "0.0",
            FmiVariableType::Int32 => "0",
            FmiVariableType::UInt64 => "0",
        });
        xml.push('"');
    }
}

fn parse_restricted_typed_variables(xml: &str) -> Result<Vec<FmiScalarVariable>, FmiExportError> {
    let mut variables = Vec::new();
    let mut rest = xml;
    while let Some((start, variable_type)) = next_restricted_variable(rest) {
        reject_unexpected_xml_elements(&rest[..start], "ModelVariables")?;
        rest = &rest[start..];
        let Some(end) = rest.find('>') else {
            return invalid_model_description(
                "ModelVariables contains an unterminated typed variable tag".to_owned(),
            );
        };
        let tag = &rest[..=end];
        let type_name = variable_type.as_str();
        let name = required_xml_attr(tag, "name", type_name)?;
        let value_reference_text = required_xml_attr(tag, "valueReference", type_name)?;
        let value_reference = value_reference_text.parse::<u32>().map_err(|source| {
            FmiExportError::InvalidModelDescription {
                reason: format!(
                    "{type_name} variable {name} has invalid valueReference {value_reference_text}: {source}"
                ),
            }
        })?;
        let causality = match xml_attr(tag, "causality") {
            Some(raw) => parse_restricted_causality(&raw)?,
            None => FmiVariableCausality::Local,
        };
        variables.push(FmiScalarVariable {
            name,
            value_reference,
            causality,
            variable_type,
        });
        rest = consume_xml_element(rest, end, type_name)?;
    }
    reject_unexpected_xml_elements(rest, "ModelVariables")?;
    Ok(variables)
}

fn parse_restricted_output_value_references(xml: &str) -> Result<Vec<u32>, FmiExportError> {
    let mut outputs = Vec::new();
    let mut rest = xml;
    while let Some(start) = rest.find("<Output") {
        reject_unexpected_xml_elements(&rest[..start], "ModelStructure")?;
        rest = &rest[start..];
        let Some(end) = rest.find('>') else {
            return invalid_model_description(
                "ModelStructure contains an unterminated Output tag".to_owned(),
            );
        };
        let tag = &rest[..=end];
        let value_reference_text = required_xml_attr(tag, "valueReference", "Output")?;
        let value_reference = value_reference_text.parse::<u32>().map_err(|source| {
            FmiExportError::InvalidModelDescription {
                reason: format!(
                    "Output has invalid valueReference {value_reference_text}: {source}"
                ),
            }
        })?;
        outputs.push(value_reference);
        rest = consume_xml_element(rest, end, "Output")?;
    }
    reject_unexpected_xml_elements(rest, "ModelStructure")?;
    Ok(outputs)
}

fn next_restricted_variable(xml: &str) -> Option<(usize, FmiVariableType)> {
    [
        ("Float64", FmiVariableType::Float64),
        ("Int32", FmiVariableType::Int32),
        ("UInt64", FmiVariableType::UInt64),
    ]
    .into_iter()
    .filter_map(|(tag, variable_type)| {
        xml.find(&format!("<{tag}"))
            .map(|index| (index, variable_type))
    })
    .min_by_key(|(index, _variable_type)| *index)
}

fn consume_xml_element<'a>(
    xml: &'a str,
    opening_tag_end: usize,
    tag: &str,
) -> Result<&'a str, FmiExportError> {
    let opening_tag = &xml[..=opening_tag_end];
    if opening_tag.trim_end().ends_with("/>") {
        return Ok(&xml[opening_tag_end + 1..]);
    }
    let close = format!("</{tag}>");
    let Some(close_start) = xml[opening_tag_end + 1..].find(&close) else {
        return invalid_model_description(format!("{tag} is missing closing {close}"));
    };
    Ok(&xml[opening_tag_end + 1 + close_start + close.len()..])
}

fn parse_restricted_causality(raw: &str) -> Result<FmiVariableCausality, FmiExportError> {
    match raw {
        "parameter" => Ok(FmiVariableCausality::Parameter),
        "calculatedParameter" => Ok(FmiVariableCausality::CalculatedParameter),
        "input" => Ok(FmiVariableCausality::Input),
        "output" => Ok(FmiVariableCausality::Output),
        "local" => Ok(FmiVariableCausality::Local),
        "independent" => Ok(FmiVariableCausality::Independent),
        _ => invalid_model_description(format!("unsupported variable causality {raw}")),
    }
}

fn required_xml_tag<'a>(xml: &'a str, tag: &str) -> Result<&'a str, FmiExportError> {
    let needle = format!("<{tag}");
    let Some(start) = xml.find(&needle) else {
        return invalid_model_description(format!("missing <{tag}>"));
    };
    let rest = &xml[start..];
    let Some(end) = rest.find('>') else {
        return invalid_model_description(format!("unterminated <{tag}>"));
    };
    Ok(&rest[..=end])
}

fn required_xml_element_body<'a>(xml: &'a str, tag: &str) -> Result<&'a str, FmiExportError> {
    let opening_tag = required_xml_tag(xml, tag)?;
    if opening_tag.trim_end().ends_with("/>") {
        return invalid_model_description(format!("<{tag}> must not be empty"));
    }
    let Some(open_start) = xml.find(opening_tag) else {
        return invalid_model_description(format!("missing <{tag}>"));
    };
    let body_start = open_start + opening_tag.len();
    let close = format!("</{tag}>");
    let Some(close_start) = xml[body_start..].find(&close) else {
        return invalid_model_description(format!("missing closing {close}"));
    };
    Ok(&xml[body_start..body_start + close_start])
}

fn required_xml_attr(tag: &str, attr: &str, owner: &str) -> Result<String, FmiExportError> {
    let value = xml_attr(tag, attr).ok_or_else(|| FmiExportError::InvalidModelDescription {
        reason: format!("{owner} is missing {attr}"),
    })?;
    require_xml_attr_non_empty(attr, &value)?;
    Ok(value)
}

fn require_xml_attr_non_empty(attr: &str, value: &str) -> Result<(), FmiExportError> {
    if value.is_empty() {
        invalid_model_description(format!("{attr} must not be empty"))
    } else {
        Ok(())
    }
}

fn xml_attr(tag: &str, attr: &str) -> Option<String> {
    let needle = format!("{attr}=\"");
    let start = tag.find(&needle)? + needle.len();
    let rest = &tag[start..];
    let end = rest.find('"')?;
    Some(rest[..end].to_owned())
}

fn reject_unexpected_xml_elements(text: &str, owner: &str) -> Result<(), FmiExportError> {
    if text.contains('<') {
        invalid_model_description(format!("{owner} contains unsupported XML elements"))
    } else {
        Ok(())
    }
}

fn invalid_model_description<T>(reason: impl Into<String>) -> Result<T, FmiExportError> {
    Err(FmiExportError::InvalidModelDescription {
        reason: reason.into(),
    })
}

fn validate_binary_entry(entry: &str) -> Result<(), FmiExportError> {
    if entry.is_empty()
        || entry.starts_with('/')
        || entry.starts_with('\\')
        || entry.split('/').any(|part| part.is_empty() || part == "..")
        || entry.contains('\\')
    {
        return Err(FmiExportError::InvalidBinaryEntry {
            entry: entry.to_owned(),
        });
    }
    Ok(())
}

fn escape_xml_attr(value: &str) -> String {
    let mut escaped = String::with_capacity(value.len());
    for ch in value.chars() {
        match ch {
            '&' => escaped.push_str("&amp;"),
            '"' => escaped.push_str("&quot;"),
            '<' => escaped.push_str("&lt;"),
            '>' => escaped.push_str("&gt;"),
            '\'' => escaped.push_str("&apos;"),
            _ => escaped.push(ch),
        }
    }
    escaped
}

fn write_stored_zip(
    path: impl AsRef<Path>,
    entries: &[(&str, &[u8])],
) -> Result<(), FmiExportError> {
    let mut archive = Vec::new();
    let mut central_directory = Vec::new();
    for (name, data) in entries {
        let offset = u32::try_from(archive.len()).map_err(|_| FmiExportError::EntryTooLarge {
            entry: (*name).to_owned(),
        })?;
        write_local_file_header(&mut archive, name, data)?;
        write_central_directory_header(&mut central_directory, name, data, offset)?;
    }
    let central_directory_offset =
        u32::try_from(archive.len()).map_err(|_| FmiExportError::EntryTooLarge {
            entry: "central-directory".to_owned(),
        })?;
    archive.extend_from_slice(&central_directory);
    let central_directory_size =
        u32::try_from(central_directory.len()).map_err(|_| FmiExportError::EntryTooLarge {
            entry: "central-directory".to_owned(),
        })?;
    write_end_of_central_directory(
        &mut archive,
        entries.len(),
        central_directory_size,
        central_directory_offset,
    )?;
    let path = path.as_ref();
    std::fs::write(path, archive).map_err(|source| FmiExportError::Io {
        path: path.display().to_string(),
        source,
    })
}

fn write_local_file_header(
    out: &mut Vec<u8>,
    name: &str,
    data: &[u8],
) -> Result<(), FmiExportError> {
    let name_bytes = checked_name_bytes(name)?;
    let size = checked_size(name, data)?;
    write_u32(out, 0x0403_4b50);
    write_u16(out, 20);
    write_u16(out, 0);
    write_u16(out, 0);
    write_u16(out, 0);
    write_u16(out, 0);
    write_u32(out, crc32(data));
    write_u32(out, size);
    write_u32(out, size);
    write_u16(out, name_bytes.len() as u16);
    write_u16(out, 0);
    out.extend_from_slice(name_bytes);
    out.extend_from_slice(data);
    Ok(())
}

fn write_central_directory_header(
    out: &mut Vec<u8>,
    name: &str,
    data: &[u8],
    local_header_offset: u32,
) -> Result<(), FmiExportError> {
    let name_bytes = checked_name_bytes(name)?;
    let size = checked_size(name, data)?;
    write_u32(out, 0x0201_4b50);
    write_u16(out, 20);
    write_u16(out, 20);
    write_u16(out, 0);
    write_u16(out, 0);
    write_u16(out, 0);
    write_u16(out, 0);
    write_u32(out, crc32(data));
    write_u32(out, size);
    write_u32(out, size);
    write_u16(out, name_bytes.len() as u16);
    write_u16(out, 0);
    write_u16(out, 0);
    write_u16(out, 0);
    write_u16(out, 0);
    write_u32(out, 0);
    write_u32(out, local_header_offset);
    out.extend_from_slice(name_bytes);
    Ok(())
}

fn write_end_of_central_directory(
    out: &mut Vec<u8>,
    entry_count: usize,
    central_directory_size: u32,
    central_directory_offset: u32,
) -> Result<(), FmiExportError> {
    let entry_count = u16::try_from(entry_count).map_err(|_| FmiExportError::EntryTooLarge {
        entry: "entry-count".to_owned(),
    })?;
    write_u32(out, 0x0605_4b50);
    write_u16(out, 0);
    write_u16(out, 0);
    write_u16(out, entry_count);
    write_u16(out, entry_count);
    write_u32(out, central_directory_size);
    write_u32(out, central_directory_offset);
    write_u16(out, 0);
    Ok(())
}

fn checked_name_bytes(name: &str) -> Result<&[u8], FmiExportError> {
    let bytes = name.as_bytes();
    if bytes.len() > u16::MAX as usize {
        return Err(FmiExportError::EntryNameTooLong {
            entry: name.to_owned(),
        });
    }
    Ok(bytes)
}

fn checked_size(name: &str, data: &[u8]) -> Result<u32, FmiExportError> {
    u32::try_from(data.len()).map_err(|_| FmiExportError::EntryTooLarge {
        entry: name.to_owned(),
    })
}

fn write_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn write_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff_u32;
    for byte in data {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0_u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::Fmi3VariableBindings;
    use openbmp_models::FmuArchive;

    #[test]
    fn export_model_description_round_trips_through_fmu_archive() {
        let temp = tempfile::tempdir().expect("tempdir");
        let fmu_path = temp.path().join("openbmp-export.fmu");

        openbmp_point_mass_export_model()
            .write_fmu_archive(
                &fmu_path,
                openbmp_fmi_binary_entry_for_target("x86_64", "linux").unwrap(),
                b"shared-library-placeholder",
            )
            .expect("write export fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load exported fmu metadata");
        let description = archive.model_description();

        assert_eq!(description.fmi_version, "3.0");
        assert_eq!(description.model_name, OPENBMP_POINT_MASS_MODEL_NAME);
        assert_eq!(
            description.model_identifier,
            OPENBMP_POINT_MASS_MODEL_IDENTIFIER
        );
        assert_eq!(description.input_variables, vec!["throttle"]);
        assert_eq!(
            description.output_variables,
            vec!["altitude", "step", "velocity"]
        );
        assert!(
            archive
                .entry(openbmp_fmi_binary_entry_for_target("x86_64", "linux").unwrap())
                .is_some()
        );
    }

    #[test]
    fn export_model_description_uses_restricted_fmi3_schema_shape() {
        let xml = openbmp_point_mass_export_model()
            .model_description_xml()
            .expect("modelDescription");

        assert!(xml.contains("fmiVersion=\"3.0\""));
        assert!(xml.contains("instantiationToken=\"openbmp-point-mass-restricted-fmi3-cs\""));
        assert!(xml.contains("variableNamingConvention=\"flat\""));
        assert!(xml.contains("canHandleVariableCommunicationStepSize=\"true\""));
        assert!(xml.contains("canGetAndSetFMUState=\"true\""));
        assert!(xml.contains("<ModelVariables>"));
        assert!(
            xml.contains("<Float64 name=\"time\" valueReference=\"0\" causality=\"independent\"/>")
        );
        assert!(xml.contains(
            "<Float64 name=\"throttle\" valueReference=\"10\" causality=\"input\" initial=\"exact\" start=\"0.0\"/>"
        ));
        assert!(
            xml.contains("<Float64 name=\"altitude\" valueReference=\"20\" causality=\"output\"/>")
        );
        assert!(xml.contains("<UInt64 name=\"step\" valueReference=\"21\" causality=\"output\"/>"));
        assert!(
            xml.contains("<Float64 name=\"velocity\" valueReference=\"22\" causality=\"output\"/>")
        );
        assert!(!xml.contains("<ScalarVariable"));
        assert!(xml.contains("<ModelStructure>"));
        assert!(xml.contains("<Output valueReference=\"20\"/>"));
        assert!(xml.contains("<Output valueReference=\"21\"/>"));
        assert!(xml.contains("<Output valueReference=\"22\"/>"));

        let parsed =
            validate_restricted_fmi3_model_description_xml(&xml).expect("restricted schema shape");
        assert_eq!(parsed.model_name, OPENBMP_POINT_MASS_MODEL_NAME);
        assert_eq!(
            parsed.instantiation_token,
            OPENBMP_POINT_MASS_INSTANTIATION_TOKEN
        );
        assert_eq!(parsed.model_identifier, OPENBMP_POINT_MASS_MODEL_IDENTIFIER);
        assert_eq!(
            parsed.output_value_references,
            vec![
                OPENBMP_POINT_MASS_VR_ALTITUDE_M,
                OPENBMP_POINT_MASS_VR_STEP,
                OPENBMP_POINT_MASS_VR_VELOCITY_M_S,
            ]
        );
    }

    #[test]
    fn restricted_model_description_validator_rejects_output_mismatch() {
        let xml = openbmp_point_mass_export_model()
            .model_description_xml()
            .expect("modelDescription")
            .replace("<Output valueReference=\"22\"/>\n", "");

        let err = validate_restricted_fmi3_model_description_xml(&xml)
            .expect_err("missing output structure entry should fail");

        assert!(matches!(
            err,
            FmiExportError::InvalidModelDescription { ref reason }
                if reason.contains("ModelStructure outputs")
        ));
    }

    #[test]
    fn openbmp_fmi_binary_entries_match_supported_targets() {
        for (target_arch, target_os, expected) in [
            ("x86_64", "linux", "binaries/x86_64-linux/libopenbmp_fmi.so"),
            (
                "aarch64",
                "linux",
                "binaries/aarch64-linux/libopenbmp_fmi.so",
            ),
            (
                "x86_64",
                "macos",
                "binaries/x86_64-darwin/libopenbmp_fmi.dylib",
            ),
            (
                "aarch64",
                "macos",
                "binaries/aarch64-darwin/libopenbmp_fmi.dylib",
            ),
            (
                "x86_64",
                "windows",
                "binaries/x86_64-windows/openbmp_fmi.dll",
            ),
            (
                "aarch64",
                "windows",
                "binaries/aarch64-windows/openbmp_fmi.dll",
            ),
        ] {
            let entry = openbmp_fmi_binary_entry_for_target(target_arch, target_os).unwrap();
            assert_eq!(entry, expected);
            validate_binary_entry(entry).expect("valid binary entry");
        }
        assert!(openbmp_fmi_binary_entry_for_target("wasm32", "unknown").is_none());
        if let Some(entry) = openbmp_fmi_binary_entry_for_current_platform() {
            validate_binary_entry(entry).expect("current binary entry");
        }
    }

    #[test]
    fn point_mass_fmi_binary_entries_match_supported_targets() {
        for (target_arch, target_os, expected) in [
            (
                "x86_64",
                "linux",
                "binaries/x86_64-linux/openbmp_point_mass.so",
            ),
            (
                "aarch64",
                "linux",
                "binaries/aarch64-linux/openbmp_point_mass.so",
            ),
            (
                "x86_64",
                "macos",
                "binaries/x86_64-darwin/openbmp_point_mass.dylib",
            ),
            (
                "aarch64",
                "macos",
                "binaries/aarch64-darwin/openbmp_point_mass.dylib",
            ),
            (
                "x86_64",
                "windows",
                "binaries/x86_64-windows/openbmp_point_mass.dll",
            ),
            (
                "aarch64",
                "windows",
                "binaries/aarch64-windows/openbmp_point_mass.dll",
            ),
        ] {
            let entry = openbmp_point_mass_fmi_binary_entry_for_target(target_arch, target_os)
                .expect("supported point-mass target");
            assert_eq!(entry, expected);
            validate_binary_entry(entry).expect("point-mass binary entry");
        }
        assert!(openbmp_point_mass_fmi_binary_entry_for_target("wasm32", "unknown").is_none());
    }

    #[test]
    fn point_mass_export_model_value_references_build_expected_bindings() {
        let archive = point_mass_archive();
        let description = archive.model_description();
        let bindings = Fmi3VariableBindings::from_model_description(description);

        assert_eq!(
            description.scalar_variables,
            vec![
                fmi3_export_scalar_variable(
                    "time",
                    OPENBMP_POINT_MASS_VR_TIME_S,
                    FmiVariableCausality::Independent,
                    FmiVariableType::Float64,
                ),
                fmi3_export_scalar_variable(
                    "throttle",
                    OPENBMP_POINT_MASS_VR_THROTTLE,
                    FmiVariableCausality::Input,
                    FmiVariableType::Float64,
                ),
                fmi3_export_scalar_variable(
                    "altitude",
                    OPENBMP_POINT_MASS_VR_ALTITUDE_M,
                    FmiVariableCausality::Output,
                    FmiVariableType::Float64,
                ),
                fmi3_export_scalar_variable(
                    "step",
                    OPENBMP_POINT_MASS_VR_STEP,
                    FmiVariableCausality::Output,
                    FmiVariableType::UInt64,
                ),
                fmi3_export_scalar_variable(
                    "velocity",
                    OPENBMP_POINT_MASS_VR_VELOCITY_M_S,
                    FmiVariableCausality::Output,
                    FmiVariableType::Float64,
                ),
            ]
        );
        assert_eq!(bindings.float64_inputs.len(), 1);
        assert_eq!(bindings.float64_inputs[0].name, "throttle");
        assert_eq!(
            bindings.float64_inputs[0].value_reference,
            OPENBMP_POINT_MASS_VR_THROTTLE
        );
        assert_eq!(bindings.float64_outputs.len(), 2);
        assert_eq!(bindings.float64_outputs[0].name, "altitude");
        assert_eq!(
            bindings.float64_outputs[0].value_reference,
            OPENBMP_POINT_MASS_VR_ALTITUDE_M
        );
        assert_eq!(bindings.float64_outputs[1].name, "velocity");
        assert_eq!(
            bindings.float64_outputs[1].value_reference,
            OPENBMP_POINT_MASS_VR_VELOCITY_M_S
        );
        assert_eq!(bindings.uint64_outputs.len(), 1);
        assert_eq!(bindings.uint64_outputs[0].name, "step");
        assert_eq!(
            bindings.uint64_outputs[0].value_reference,
            OPENBMP_POINT_MASS_VR_STEP
        );
    }

    #[test]
    fn export_model_description_escapes_xml_attributes() {
        let xml = Fmi3ExportModel::new("OpenBMP \"plant\" & test", "openbmp_plant")
            .with_scalar_variable(fmi3_export_scalar_variable(
                "altitude <m>",
                1,
                FmiVariableCausality::Output,
                FmiVariableType::Float64,
            ))
            .model_description_xml()
            .expect("modelDescription");

        assert!(xml.contains("OpenBMP &quot;plant&quot; &amp; test"));
        assert!(xml.contains("altitude &lt;m&gt;"));
    }

    #[test]
    fn export_model_description_rejects_duplicate_value_reference() {
        let err = Fmi3ExportModel::new("OpenBMP", "openbmp")
            .with_scalar_variable(fmi3_export_scalar_variable(
                "a",
                10,
                FmiVariableCausality::Input,
                FmiVariableType::Float64,
            ))
            .with_scalar_variable(fmi3_export_scalar_variable(
                "b",
                10,
                FmiVariableCausality::Output,
                FmiVariableType::Float64,
            ))
            .model_description_xml()
            .expect_err("duplicate valueReference should fail");

        assert!(matches!(
            err,
            FmiExportError::DuplicateValueReference {
                value_reference: 10
            }
        ));
    }

    #[test]
    fn export_fmu_archive_rejects_unsafe_binary_entry() {
        let temp = tempfile::tempdir().expect("tempdir");
        let err = openbmp_point_mass_export_model()
            .write_fmu_archive(
                temp.path().join("bad.fmu"),
                "../libopenbmp_fmi.so",
                b"binary",
            )
            .expect_err("unsafe path should fail");

        assert!(matches!(err, FmiExportError::InvalidBinaryEntry { .. }));
    }

    fn point_mass_archive() -> FmuArchive {
        let temp = tempfile::tempdir().expect("tempdir");
        let fmu_path = temp.path().join("openbmp-export.fmu");
        openbmp_point_mass_export_model()
            .write_fmu_archive(
                &fmu_path,
                openbmp_fmi_binary_entry_for_target("x86_64", "linux").unwrap(),
                b"shared-library-placeholder",
            )
            .expect("write point-mass fmu");
        FmuArchive::load(&fmu_path).expect("load point-mass fmu")
    }
}
