//! Model-interchange port abstraction.
//!
//! A model port is the smallest deterministic stepping boundary used
//! to swap native Rust submodels for external model implementations.
//! The current implementation provides a native closure-backed port,
//! a typed FMI co-simulation specification, and a small std-gated FMU
//! archive reader for deterministic import smoke tests. Full FMI
//! conformance, compressed ZIP entries, and dynamic-library calls remain
//! adapter responsibilities.

#[cfg(feature = "std")]
use alloc::collections::BTreeMap;
use alloc::string::String;
#[cfg(feature = "std")]
use alloc::string::ToString;
use alloc::vec::Vec;
use core::marker::PhantomData;
#[cfg(feature = "std")]
use core::{error::Error, fmt};
#[cfg(feature = "std")]
use std::{
    io::Read,
    path::{Path, PathBuf},
};

use crate::ModelEvalError;

/// Kind of implementation backing a model port.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ModelPortKind {
    /// Native Rust implementation.
    Native,
    /// FMI co-simulation import.
    FmiCoSimulation,
}

/// Static metadata for a model port.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ModelPortMetadata {
    /// Stable model id.
    pub id: String,
    /// Backing implementation kind.
    pub kind: ModelPortKind,
    /// Determinism profile label.
    pub determinism: String,
}

impl ModelPortMetadata {
    /// Create native-port metadata.
    #[must_use]
    pub fn native(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            kind: ModelPortKind::Native,
            determinism: "bit-stable-native".into(),
        }
    }

    /// Create FMI co-simulation metadata.
    #[must_use]
    pub fn fmi(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            kind: ModelPortKind::FmiCoSimulation,
            determinism: "adapter-declared".into(),
        }
    }
}

/// Deterministic model stepping port.
pub trait ModelPort {
    /// Input sample type.
    type Input;
    /// Output sample type.
    type Output;

    /// Port metadata.
    fn metadata(&self) -> &ModelPortMetadata;

    /// Step the model by `dt_s`.
    ///
    /// # Errors
    ///
    /// Returns [`ModelEvalError`] when the model cannot evaluate this
    /// input at the requested step.
    fn step(&mut self, dt_s: f64, input: &Self::Input) -> Result<Self::Output, ModelEvalError>;
}

/// Adapter trait for an imported FMI co-simulation slave.
///
/// A host-specific crate owns real FMU archive loading,
/// `modelDescription.xml` parsing, dynamic-library calls, and platform
/// cleanup. This trait is the deterministic stepping boundary that
/// portable OpenBMP code consumes once an adapter has instantiated the
/// FMU.
pub trait FmuCoSimulationBackend {
    /// Input sample type.
    type Input;
    /// Output sample type.
    type Output;

    /// Step the imported co-simulation slave.
    ///
    /// `current_time_s` is the OpenBMP master time at the beginning of
    /// the step, and `step_size_s` is the requested communication step
    /// size.
    ///
    /// # Errors
    ///
    /// Returns [`ModelEvalError`] when the adapter or FMU cannot produce
    /// a deterministic output for this step.
    fn do_step(
        &mut self,
        spec: &FmuCoSimulationPortSpec,
        current_time_s: f64,
        step_size_s: f64,
        input: &Self::Input,
    ) -> Result<Self::Output, ModelEvalError>;
}

/// Native closure-backed model port.
#[derive(Clone, Debug)]
pub struct NativeModelPort<I, O, F> {
    metadata: ModelPortMetadata,
    step_fn: F,
    _types: PhantomData<(I, O)>,
}

impl<I, O, F> NativeModelPort<I, O, F>
where
    F: FnMut(f64, &I) -> Result<O, ModelEvalError>,
{
    /// Construct a native model port.
    #[must_use]
    pub fn new(id: impl Into<String>, step_fn: F) -> Self {
        Self {
            metadata: ModelPortMetadata::native(id),
            step_fn,
            _types: PhantomData,
        }
    }
}

impl<I, O, F> ModelPort for NativeModelPort<I, O, F>
where
    F: FnMut(f64, &I) -> Result<O, ModelEvalError>,
{
    type Input = I;
    type Output = O;

    fn metadata(&self) -> &ModelPortMetadata {
        &self.metadata
    }

    fn step(&mut self, dt_s: f64, input: &Self::Input) -> Result<Self::Output, ModelEvalError> {
        (self.step_fn)(dt_s, input)
    }
}

/// FMI co-simulation model port backed by a host adapter.
#[derive(Clone, Debug)]
pub struct FmuCoSimulationModelPort<I, O, B> {
    spec: FmuCoSimulationPortSpec,
    metadata: ModelPortMetadata,
    backend: B,
    current_time_s: f64,
    _types: PhantomData<(I, O)>,
}

impl<I, O, B> FmuCoSimulationModelPort<I, O, B>
where
    B: FmuCoSimulationBackend<Input = I, Output = O>,
{
    /// Construct an FMI co-simulation port from an import spec and
    /// already-instantiated backend.
    #[must_use]
    pub fn new(spec: FmuCoSimulationPortSpec, backend: B) -> Self {
        Self {
            metadata: spec.metadata(),
            spec,
            backend,
            current_time_s: 0.0,
            _types: PhantomData,
        }
    }

    /// Current master time at the start of the next step.
    #[must_use]
    pub const fn current_time_s(&self) -> f64 {
        self.current_time_s
    }
}

impl<I, O, B> ModelPort for FmuCoSimulationModelPort<I, O, B>
where
    B: FmuCoSimulationBackend<Input = I, Output = O>,
{
    type Input = I;
    type Output = O;

    fn metadata(&self) -> &ModelPortMetadata {
        &self.metadata
    }

    fn step(&mut self, dt_s: f64, input: &Self::Input) -> Result<Self::Output, ModelEvalError> {
        let output = self
            .backend
            .do_step(&self.spec, self.current_time_s, dt_s, input)?;
        self.current_time_s += dt_s;
        Ok(output)
    }
}

/// FMI co-simulation import specification.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FmuCoSimulationPortSpec {
    /// Stable model id used by OpenBMP.
    pub id: String,
    /// FMU archive path or package-relative reference.
    pub fmu: String,
    /// FMI model identifier from `modelDescription.xml`.
    pub model_identifier: String,
    /// Declared FMI version, for example `"3.0"`.
    pub fmi_version: String,
    /// Input variable names consumed by the adapter.
    pub input_variables: Vec<String>,
    /// Output variable names produced by the adapter.
    pub output_variables: Vec<String>,
}

impl FmuCoSimulationPortSpec {
    /// Return model-port metadata for this FMU import.
    #[must_use]
    pub fn metadata(&self) -> ModelPortMetadata {
        ModelPortMetadata::fmi(self.id.clone())
    }

    /// Build a co-simulation import spec from a loaded FMU archive.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn from_archive(id: impl Into<String>, archive: &FmuArchive) -> Self {
        let description = archive.model_description();
        Self {
            id: id.into(),
            fmu: archive.path().display().to_string(),
            model_identifier: description.model_identifier.clone(),
            fmi_version: description.fmi_version.clone(),
            input_variables: description.input_variables.clone(),
            output_variables: description.output_variables.clone(),
        }
    }
}

/// FMI scalar-variable causality parsed from `modelDescription.xml`.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FmiVariableCausality {
    /// Tunable parameter supplied before initialization.
    Parameter,
    /// Calculated parameter supplied by the FMU.
    CalculatedParameter,
    /// Input consumed by the FMU.
    Input,
    /// Output produced by the FMU.
    Output,
    /// Local internal variable.
    Local,
    /// Independent variable, usually time.
    Independent,
}

impl FmiVariableCausality {
    /// Return the FMI spelling for this causality.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Parameter => "parameter",
            Self::CalculatedParameter => "calculatedParameter",
            Self::Input => "input",
            Self::Output => "output",
            Self::Local => "local",
            Self::Independent => "independent",
        }
    }

    #[cfg(feature = "std")]
    fn parse(raw: &str) -> Result<Self, FmuArchiveError> {
        match raw {
            "parameter" => Ok(Self::Parameter),
            "calculatedParameter" => Ok(Self::CalculatedParameter),
            "input" => Ok(Self::Input),
            "output" => Ok(Self::Output),
            "local" => Ok(Self::Local),
            "independent" => Ok(Self::Independent),
            _ => Err(FmuArchiveError::new(format!(
                "modelDescription.xml has unsupported ScalarVariable causality {raw}"
            ))),
        }
    }
}

/// FMI scalar-variable storage type supported by OpenBMP import metadata.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FmiVariableType {
    /// FMI 3 `Float64` variable.
    Float64,
    /// FMI 3 `Int32` variable.
    Int32,
    /// FMI 3 `UInt64` variable.
    UInt64,
}

impl FmiVariableType {
    /// Return the FMI type tag for this scalar variable.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Float64 => "Float64",
            Self::Int32 => "Int32",
            Self::UInt64 => "UInt64",
        }
    }
}

/// FMI 3 Clock `intervalVariability` value.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FmiClockIntervalVariability {
    /// Time-based periodic clock with constant interval.
    Constant,
    /// Time-based periodic clock with fixed interval set during initialization.
    Fixed,
    /// Time-based periodic clock with tunable interval.
    Tunable,
    /// Time-based aperiodic clock with changing interval.
    Changing,
    /// Time-based aperiodic countdown clock.
    Countdown,
    /// Triggered clock without an a-priori time interval.
    Triggered,
}

impl FmiClockIntervalVariability {
    /// Return the FMI XML spelling for this clock interval variability.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Constant => "constant",
            Self::Fixed => "fixed",
            Self::Tunable => "tunable",
            Self::Changing => "changing",
            Self::Countdown => "countdown",
            Self::Triggered => "triggered",
        }
    }

    #[cfg(feature = "std")]
    fn parse(raw: &str) -> Result<Self, FmuArchiveError> {
        match raw {
            "constant" => Ok(Self::Constant),
            "fixed" => Ok(Self::Fixed),
            "tunable" => Ok(Self::Tunable),
            "changing" => Ok(Self::Changing),
            "countdown" => Ok(Self::Countdown),
            "triggered" => Ok(Self::Triggered),
            _ => Err(FmuArchiveError::new(format!(
                "modelDescription.xml has unsupported Clock intervalVariability {raw}"
            ))),
        }
    }
}

/// Typed FMI scalar variable with its value reference.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FmiScalarVariable {
    /// Variable name.
    pub name: String,
    /// FMI value reference.
    pub value_reference: u32,
    /// Variable causality.
    pub causality: FmiVariableCausality,
    /// Variable storage type.
    pub variable_type: FmiVariableType,
}

/// FMI 3 Clock variable metadata parsed from `modelDescription.xml`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FmiClockVariable {
    /// Clock variable name.
    pub name: String,
    /// FMI value reference.
    pub value_reference: u32,
    /// Clock causality. FMI 3 restricts Clock causality to input,
    /// output, or local.
    pub causality: FmiVariableCausality,
    /// Declared clock interval variability.
    pub interval_variability: FmiClockIntervalVariability,
    /// Optional decimal interval attribute preserved as XML text.
    pub interval_decimal: Option<String>,
    /// Optional decimal shift attribute preserved as XML text.
    pub shift_decimal: Option<String>,
}

/// FMI variable that declares one or more governing Clocks.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FmiClockedVariable {
    /// Variable name.
    pub name: String,
    /// FMI value reference of the clocked variable.
    pub value_reference: u32,
    /// Clock value references listed in the variable's `clocks`
    /// attribute.
    pub clock_references: Vec<u32>,
}

/// Error raised while loading a restricted FMU archive.
#[cfg(feature = "std")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FmuArchiveError {
    summary: String,
}

#[cfg(feature = "std")]
impl FmuArchiveError {
    fn new(summary: impl Into<String>) -> Self {
        Self {
            summary: summary.into(),
        }
    }

    /// Human-readable failure summary.
    #[must_use]
    pub fn summary(&self) -> &str {
        &self.summary
    }
}

#[cfg(feature = "std")]
impl fmt::Display for FmuArchiveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.summary)
    }
}

#[cfg(feature = "std")]
impl Error for FmuArchiveError {}

/// Co-simulation metadata parsed from `modelDescription.xml`.
#[cfg(feature = "std")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FmuModelDescription {
    /// FMI version declared by the archive.
    pub fmi_version: String,
    /// FMI model name declared by the archive.
    pub model_name: String,
    /// FMI 3 instantiation token declared by the root model description.
    pub instantiation_token: Option<String>,
    /// Co-simulation model identifier.
    pub model_identifier: String,
    /// Typed scalar variables in document order.
    pub scalar_variables: Vec<FmiScalarVariable>,
    /// FMI 3 Clock variables in document order.
    pub clock_variables: Vec<FmiClockVariable>,
    /// Scalar variables that declare governing Clocks.
    pub clocked_variables: Vec<FmiClockedVariable>,
    /// Input variable names.
    pub input_variables: Vec<String>,
    /// Output variable names.
    pub output_variables: Vec<String>,
}

/// Restricted FMU archive reader used for import smoke tests.
///
/// This loader supports stored and deflated ZIP entries. It validates the
/// presence of `modelDescription.xml`, parses the FMI version,
/// co-simulation model identifier, scalar-variable causality, supported
/// scalar types, and value references, and exposes resource text for
/// deterministic adapter tests. It does not
/// call platform binaries and does not claim FMI conformance.
#[cfg(feature = "std")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FmuArchive {
    path: PathBuf,
    entries: BTreeMap<String, Vec<u8>>,
    model_description: FmuModelDescription,
}

#[cfg(feature = "std")]
impl FmuArchive {
    /// Load an FMU archive from disk.
    ///
    /// # Errors
    ///
    /// Returns [`FmuArchiveError`] when the archive cannot be read,
    /// when it is not a supported ZIP archive, or when required
    /// FMI co-simulation metadata is absent.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, FmuArchiveError> {
        let path = path.as_ref();
        let bytes = std::fs::read(path)
            .map_err(|source| FmuArchiveError::new(format!("read {}: {source}", path.display())))?;
        let entries = read_zip_entries(&bytes)?;
        let Some(model_description_bytes) = entries.get("modelDescription.xml") else {
            return Err(FmuArchiveError::new(
                "FMU archive is missing modelDescription.xml",
            ));
        };
        let model_description_xml =
            core::str::from_utf8(model_description_bytes).map_err(|source| {
                FmuArchiveError::new(format!("modelDescription.xml UTF-8: {source}"))
            })?;
        let model_description = parse_model_description(model_description_xml)?;
        Ok(Self {
            path: path.to_path_buf(),
            entries,
            model_description,
        })
    }

    /// Path loaded by this archive.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Parsed co-simulation metadata.
    #[must_use]
    pub const fn model_description(&self) -> &FmuModelDescription {
        &self.model_description
    }

    /// Borrow raw bytes for an archive entry.
    #[must_use]
    pub fn entry(&self, name: &str) -> Option<&[u8]> {
        self.entries.get(name).map(Vec::as_slice)
    }

    /// Iterate over raw archive entries in deterministic path order.
    pub fn entries(&self) -> impl Iterator<Item = (&str, &[u8])> {
        self.entries
            .iter()
            .map(|(name, bytes)| (name.as_str(), bytes.as_slice()))
    }

    /// Decode an archive entry as UTF-8 text.
    ///
    /// # Errors
    ///
    /// Returns [`FmuArchiveError`] when the entry is missing or not
    /// valid UTF-8.
    pub fn entry_text(&self, name: &str) -> Result<String, FmuArchiveError> {
        let Some(bytes) = self.entry(name) else {
            return Err(FmuArchiveError::new(format!(
                "FMU archive is missing entry {name}"
            )));
        };
        core::str::from_utf8(bytes)
            .map(str::to_owned)
            .map_err(|source| FmuArchiveError::new(format!("{name} UTF-8: {source}")))
    }
}

#[cfg(feature = "std")]
fn read_zip_entries(bytes: &[u8]) -> Result<BTreeMap<String, Vec<u8>>, FmuArchiveError> {
    let eocd = find_eocd(bytes)?;
    let entry_count = usize::from(read_u16(bytes, eocd + 10)?);
    let central_dir_size = usize::try_from(read_u32(bytes, eocd + 12)?)
        .map_err(|_| FmuArchiveError::new("central directory size overflows usize"))?;
    let central_dir_offset = usize::try_from(read_u32(bytes, eocd + 16)?)
        .map_err(|_| FmuArchiveError::new("central directory offset overflows usize"))?;
    let central_dir_end = central_dir_offset
        .checked_add(central_dir_size)
        .ok_or_else(|| FmuArchiveError::new("central directory bounds overflow"))?;
    if central_dir_end > bytes.len() {
        return Err(FmuArchiveError::new(
            "central directory extends beyond archive",
        ));
    }

    let mut cursor = central_dir_offset;
    let mut entries = BTreeMap::new();
    for _ in 0..entry_count {
        ensure_signature(bytes, cursor, 0x0201_4b50, "central directory header")?;
        let method = read_u16(bytes, cursor + 10)?;
        let general_purpose_flag = read_u16(bytes, cursor + 8)?;
        let compressed_size = usize::try_from(read_u32(bytes, cursor + 20)?)
            .map_err(|_| FmuArchiveError::new("compressed size overflows usize"))?;
        let uncompressed_size = usize::try_from(read_u32(bytes, cursor + 24)?)
            .map_err(|_| FmuArchiveError::new("uncompressed size overflows usize"))?;
        let name_len = usize::from(read_u16(bytes, cursor + 28)?);
        let extra_len = usize::from(read_u16(bytes, cursor + 30)?);
        let comment_len = usize::from(read_u16(bytes, cursor + 32)?);
        let local_header_offset = usize::try_from(read_u32(bytes, cursor + 42)?)
            .map_err(|_| FmuArchiveError::new("local header offset overflows usize"))?;
        let name_start = cursor + 46;
        let name_end = name_start
            .checked_add(name_len)
            .ok_or_else(|| FmuArchiveError::new("central-directory name bounds overflow"))?;
        if name_end > bytes.len() {
            return Err(FmuArchiveError::new(
                "central-directory name extends beyond archive",
            ));
        }
        let name = core::str::from_utf8(&bytes[name_start..name_end])
            .map_err(|source| FmuArchiveError::new(format!("entry name UTF-8: {source}")))?
            .to_owned();
        if general_purpose_flag & 0x0001 != 0 {
            return Err(FmuArchiveError::new(format!(
                "entry {name} uses ZIP encryption, which is unsupported"
            )));
        }
        if method != 0 && method != 8 {
            return Err(FmuArchiveError::new(format!(
                "entry {name} uses unsupported compression method {method}; only stored and deflated entries are supported"
            )));
        }
        let data = read_zip_entry(
            bytes,
            local_header_offset,
            compressed_size,
            uncompressed_size,
            method,
            name.as_str(),
        )?;
        entries.insert(name, data);
        cursor = name_end
            .checked_add(extra_len)
            .and_then(|next| next.checked_add(comment_len))
            .ok_or_else(|| FmuArchiveError::new("central-directory cursor overflow"))?;
    }
    Ok(entries)
}

#[cfg(feature = "std")]
fn read_zip_entry(
    bytes: &[u8],
    local_header_offset: usize,
    compressed_size: usize,
    uncompressed_size: usize,
    method: u16,
    name: &str,
) -> Result<Vec<u8>, FmuArchiveError> {
    ensure_signature(bytes, local_header_offset, 0x0403_4b50, "local file header")?;
    let local_method = read_u16(bytes, local_header_offset + 8)?;
    if local_method != method {
        return Err(FmuArchiveError::new(format!(
            "entry {name} local compression method {local_method} does not match central directory method {method}"
        )));
    }
    let name_len = usize::from(read_u16(bytes, local_header_offset + 26)?);
    let extra_len = usize::from(read_u16(bytes, local_header_offset + 28)?);
    let data_start = local_header_offset
        .checked_add(30)
        .and_then(|offset| offset.checked_add(name_len))
        .and_then(|offset| offset.checked_add(extra_len))
        .ok_or_else(|| FmuArchiveError::new("local-entry data bounds overflow"))?;
    let data_end = data_start
        .checked_add(compressed_size)
        .ok_or_else(|| FmuArchiveError::new("local-entry data size overflow"))?;
    if data_end > bytes.len() {
        return Err(FmuArchiveError::new(
            "local-entry data extends beyond archive",
        ));
    }
    let compressed = &bytes[data_start..data_end];
    let data = match method {
        0 => compressed.to_vec(),
        8 => {
            let mut decoder = flate2::read::DeflateDecoder::new(compressed);
            let mut data = Vec::with_capacity(uncompressed_size);
            decoder.read_to_end(&mut data).map_err(|source| {
                FmuArchiveError::new(format!("entry {name} deflate decode failed: {source}"))
            })?;
            data
        }
        _ => unreachable!("unsupported ZIP compression method was checked earlier"),
    };
    if data.len() != uncompressed_size {
        return Err(FmuArchiveError::new(format!(
            "entry {name} decoded to {} bytes, expected {uncompressed_size}",
            data.len()
        )));
    }
    Ok(data)
}

#[cfg(feature = "std")]
fn find_eocd(bytes: &[u8]) -> Result<usize, FmuArchiveError> {
    if bytes.len() < 22 {
        return Err(FmuArchiveError::new("archive is too small for ZIP EOCD"));
    }
    let min = bytes.len().saturating_sub(65_557);
    for offset in (min..=bytes.len() - 22).rev() {
        if read_u32(bytes, offset)? == 0x0605_4b50 {
            return Ok(offset);
        }
    }
    Err(FmuArchiveError::new(
        "archive does not contain a ZIP end-of-central-directory record",
    ))
}

#[cfg(feature = "std")]
fn ensure_signature(
    bytes: &[u8],
    offset: usize,
    expected: u32,
    label: &str,
) -> Result<(), FmuArchiveError> {
    let actual = read_u32(bytes, offset)?;
    if actual != expected {
        return Err(FmuArchiveError::new(format!(
            "{label} at offset {offset} has signature 0x{actual:08x}, expected 0x{expected:08x}"
        )));
    }
    Ok(())
}

#[cfg(feature = "std")]
fn read_u16(bytes: &[u8], offset: usize) -> Result<u16, FmuArchiveError> {
    let end = offset
        .checked_add(2)
        .ok_or_else(|| FmuArchiveError::new("u16 read offset overflow"))?;
    let Some(slice) = bytes.get(offset..end) else {
        return Err(FmuArchiveError::new("unexpected EOF while reading u16"));
    };
    Ok(u16::from_le_bytes([slice[0], slice[1]]))
}

#[cfg(feature = "std")]
fn read_u32(bytes: &[u8], offset: usize) -> Result<u32, FmuArchiveError> {
    let end = offset
        .checked_add(4)
        .ok_or_else(|| FmuArchiveError::new("u32 read offset overflow"))?;
    let Some(slice) = bytes.get(offset..end) else {
        return Err(FmuArchiveError::new("unexpected EOF while reading u32"));
    };
    Ok(u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]))
}

#[cfg(feature = "std")]
fn parse_model_description(xml: &str) -> Result<FmuModelDescription, FmuArchiveError> {
    let root = find_xml_tag(xml, "fmiModelDescription")?;
    let fmi_version = xml_attr(root, "fmiVersion")
        .ok_or_else(|| FmuArchiveError::new("modelDescription.xml missing fmiVersion"))?;
    let model_name = xml_attr(root, "modelName").unwrap_or_default();
    let instantiation_token = xml_attr(root, "instantiationToken");
    let co_sim = find_xml_tag(xml, "CoSimulation")?;
    let model_identifier = xml_attr(co_sim, "modelIdentifier").ok_or_else(|| {
        FmuArchiveError::new("modelDescription.xml CoSimulation missing modelIdentifier")
    })?;
    let variables_body = xml_element_body(xml, "ModelVariables")?;
    let (scalar_variables, clock_variables, clocked_variables) =
        if variables_body.contains("<ScalarVariable") {
            (
                parse_legacy_scalar_variables(variables_body)?,
                Vec::new(),
                Vec::new(),
            )
        } else {
            let (scalar_variables, clocked_variables) = parse_fmi3_typed_variables(variables_body)?;
            (
                scalar_variables,
                parse_fmi3_clock_variables(variables_body)?,
                clocked_variables,
            )
        };
    let mut input_variables = Vec::new();
    let mut output_variables = Vec::new();
    for variable in &scalar_variables {
        match variable.causality {
            FmiVariableCausality::Input => input_variables.push(variable.name.clone()),
            FmiVariableCausality::Output => output_variables.push(variable.name.clone()),
            _ => {}
        }
    }
    Ok(FmuModelDescription {
        fmi_version,
        model_name,
        instantiation_token,
        model_identifier,
        scalar_variables,
        clock_variables,
        clocked_variables,
        input_variables,
        output_variables,
    })
}

#[cfg(feature = "std")]
fn parse_legacy_scalar_variables(xml: &str) -> Result<Vec<FmiScalarVariable>, FmuArchiveError> {
    let mut scalar_variables = Vec::new();
    let mut rest = xml;
    while let Some(start) = rest.find("<ScalarVariable") {
        rest = &rest[start..];
        let Some(end) = rest.find('>') else {
            return Err(FmuArchiveError::new(
                "modelDescription.xml has unterminated ScalarVariable tag",
            ));
        };
        let tag = &rest[..=end];
        let name = xml_attr(tag, "name")
            .ok_or_else(|| FmuArchiveError::new("ScalarVariable is missing name"))?;
        let value_reference_text = xml_attr(tag, "valueReference").ok_or_else(|| {
            FmuArchiveError::new(format!("ScalarVariable {name} is missing valueReference"))
        })?;
        let value_reference = value_reference_text.parse::<u32>().map_err(|source| {
            FmuArchiveError::new(format!(
                "ScalarVariable {name} has invalid valueReference {value_reference_text}: {source}"
            ))
        })?;
        let causality_text = xml_attr(tag, "causality").unwrap_or_else(|| "local".to_owned());
        let causality = FmiVariableCausality::parse(&causality_text)?;
        let (body, next_rest) = scalar_variable_body(rest, end, &name)?;
        let variable_type = parse_scalar_variable_type(body, &name)?;

        scalar_variables.push(FmiScalarVariable {
            name,
            value_reference,
            causality,
            variable_type,
        });
        rest = next_rest;
    }
    Ok(scalar_variables)
}

#[cfg(feature = "std")]
fn parse_fmi3_typed_variables(
    xml: &str,
) -> Result<(Vec<FmiScalarVariable>, Vec<FmiClockedVariable>), FmuArchiveError> {
    let mut scalar_variables = Vec::new();
    let mut clocked_variables = Vec::new();
    let mut rest = xml;
    while let Some((start, variable_type)) = next_supported_fmi3_variable(rest) {
        rest = &rest[start..];
        let Some(end) = rest.find('>') else {
            return Err(FmuArchiveError::new(
                "modelDescription.xml has unterminated FMI 3 variable tag",
            ));
        };
        let tag = &rest[..=end];
        let type_name = variable_type.as_str();
        let name = xml_attr(tag, "name")
            .ok_or_else(|| FmuArchiveError::new(format!("{type_name} variable is missing name")))?;
        let value_reference_text = xml_attr(tag, "valueReference").ok_or_else(|| {
            FmuArchiveError::new(format!(
                "{type_name} variable {name} is missing valueReference"
            ))
        })?;
        let value_reference = value_reference_text.parse::<u32>().map_err(|source| {
            FmuArchiveError::new(format!(
                "{type_name} variable {name} has invalid valueReference {value_reference_text}: {source}"
            ))
        })?;
        let causality_text = xml_attr(tag, "causality").unwrap_or_else(|| "local".to_owned());
        let causality = FmiVariableCausality::parse(&causality_text)?;
        let clock_references = parse_clock_reference_list(tag, type_name, &name)?;

        scalar_variables.push(FmiScalarVariable {
            name: name.clone(),
            value_reference,
            causality,
            variable_type,
        });
        if !clock_references.is_empty() {
            clocked_variables.push(FmiClockedVariable {
                name,
                value_reference,
                clock_references,
            });
        }
        rest = if tag.trim_end().ends_with("/>") {
            &rest[end + 1..]
        } else {
            let close = format!("</{type_name}>");
            let Some(close_start) = rest[end + 1..].find(&close) else {
                return Err(FmuArchiveError::new(format!(
                    "{type_name} variable is missing closing {close}"
                )));
            };
            &rest[end + 1 + close_start + close.len()..]
        };
    }
    Ok((scalar_variables, clocked_variables))
}

#[cfg(feature = "std")]
fn parse_fmi3_clock_variables(xml: &str) -> Result<Vec<FmiClockVariable>, FmuArchiveError> {
    let mut clock_variables = Vec::new();
    let mut rest = xml;
    while let Some(start) = rest.find("<Clock") {
        rest = &rest[start..];
        let Some(end) = rest.find('>') else {
            return Err(FmuArchiveError::new(
                "modelDescription.xml has unterminated Clock variable tag",
            ));
        };
        let tag = &rest[..=end];
        let name = xml_attr(tag, "name")
            .ok_or_else(|| FmuArchiveError::new("Clock variable is missing name"))?;
        let value_reference_text = xml_attr(tag, "valueReference").ok_or_else(|| {
            FmuArchiveError::new(format!("Clock variable {name} is missing valueReference"))
        })?;
        let value_reference = value_reference_text.parse::<u32>().map_err(|source| {
            FmuArchiveError::new(format!(
                "Clock variable {name} has invalid valueReference {value_reference_text}: {source}"
            ))
        })?;
        let causality_text = xml_attr(tag, "causality").unwrap_or_else(|| "local".to_owned());
        let causality = FmiVariableCausality::parse(&causality_text)?;
        if !matches!(
            causality,
            FmiVariableCausality::Input
                | FmiVariableCausality::Output
                | FmiVariableCausality::Local
        ) {
            return Err(FmuArchiveError::new(format!(
                "Clock variable {name} has unsupported causality {}",
                causality.as_str()
            )));
        }
        let interval_variability_text = xml_attr(tag, "intervalVariability").ok_or_else(|| {
            FmuArchiveError::new(format!(
                "Clock variable {name} is missing intervalVariability"
            ))
        })?;
        let interval_variability = FmiClockIntervalVariability::parse(&interval_variability_text)?;

        clock_variables.push(FmiClockVariable {
            name,
            value_reference,
            causality,
            interval_variability,
            interval_decimal: xml_attr(tag, "intervalDecimal"),
            shift_decimal: xml_attr(tag, "shiftDecimal"),
        });
        rest = if tag.trim_end().ends_with("/>") {
            &rest[end + 1..]
        } else {
            let close = "</Clock>";
            let Some(close_start) = rest[end + 1..].find(close) else {
                return Err(FmuArchiveError::new(
                    "Clock variable is missing closing </Clock>",
                ));
            };
            &rest[end + 1 + close_start + close.len()..]
        };
    }
    Ok(clock_variables)
}

#[cfg(feature = "std")]
fn parse_clock_reference_list(
    tag: &str,
    type_name: &str,
    name: &str,
) -> Result<Vec<u32>, FmuArchiveError> {
    let Some(clocks) = xml_attr(tag, "clocks") else {
        return Ok(Vec::new());
    };
    let mut references = Vec::new();
    for raw in clocks.split_whitespace() {
        let value_reference = raw.parse::<u32>().map_err(|source| {
            FmuArchiveError::new(format!(
                "{type_name} variable {name} has invalid clocks valueReference {raw}: {source}"
            ))
        })?;
        references.push(value_reference);
    }
    if references.is_empty() {
        return Err(FmuArchiveError::new(format!(
            "{type_name} variable {name} has empty clocks attribute"
        )));
    }
    Ok(references)
}

#[cfg(feature = "std")]
fn next_supported_fmi3_variable(xml: &str) -> Option<(usize, FmiVariableType)> {
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

#[cfg(feature = "std")]
fn scalar_variable_body<'a>(
    rest: &'a str,
    opening_tag_end: usize,
    name: &str,
) -> Result<(&'a str, &'a str), FmuArchiveError> {
    let tag = &rest[..=opening_tag_end];
    let after_opening_tag = &rest[opening_tag_end + 1..];
    if tag.trim_end().ends_with("/>") {
        return Err(FmuArchiveError::new(format!(
            "ScalarVariable {name} is missing a supported type child"
        )));
    }
    let Some(close_start) = after_opening_tag.find("</ScalarVariable>") else {
        return Err(FmuArchiveError::new(format!(
            "ScalarVariable {name} is missing closing </ScalarVariable>"
        )));
    };
    let body = &after_opening_tag[..close_start];
    let next_rest = &after_opening_tag[close_start + "</ScalarVariable>".len()..];
    Ok((body, next_rest))
}

#[cfg(feature = "std")]
fn parse_scalar_variable_type(body: &str, name: &str) -> Result<FmiVariableType, FmuArchiveError> {
    let supported = [
        ("Float64", FmiVariableType::Float64),
        ("Int32", FmiVariableType::Int32),
        ("UInt64", FmiVariableType::UInt64),
    ]
    .into_iter()
    .filter_map(|(tag, variable_type)| body.contains(&format!("<{tag}")).then_some(variable_type))
    .collect::<Vec<_>>();
    match supported.as_slice() {
        [variable_type] => Ok(*variable_type),
        [] => Err(FmuArchiveError::new(format!(
            "ScalarVariable {name} is missing supported Float64, Int32, or UInt64 child type"
        ))),
        _ => Err(FmuArchiveError::new(format!(
            "ScalarVariable {name} declares more than one supported type child"
        ))),
    }
}

#[cfg(feature = "std")]
fn find_xml_tag<'a>(xml: &'a str, tag: &str) -> Result<&'a str, FmuArchiveError> {
    let needle = format!("<{tag}");
    let Some(start) = xml.find(&needle) else {
        return Err(FmuArchiveError::new(format!(
            "modelDescription.xml missing <{tag}>"
        )));
    };
    let rest = &xml[start..];
    let Some(end) = rest.find('>') else {
        return Err(FmuArchiveError::new(format!(
            "modelDescription.xml has unterminated <{tag}>"
        )));
    };
    Ok(&rest[..=end])
}

#[cfg(feature = "std")]
fn xml_element_body<'a>(xml: &'a str, tag: &str) -> Result<&'a str, FmuArchiveError> {
    let opening_tag = find_xml_tag(xml, tag)?;
    if opening_tag.trim_end().ends_with("/>") {
        return Err(FmuArchiveError::new(format!(
            "modelDescription.xml <{tag}> must not be empty"
        )));
    }
    let Some(open_start) = xml.find(opening_tag) else {
        return Err(FmuArchiveError::new(format!(
            "modelDescription.xml missing <{tag}>"
        )));
    };
    let body_start = open_start + opening_tag.len();
    let close = format!("</{tag}>");
    let Some(close_start) = xml[body_start..].find(&close) else {
        return Err(FmuArchiveError::new(format!(
            "modelDescription.xml missing closing {close}"
        )));
    };
    Ok(&xml[body_start..body_start + close_start])
}

#[cfg(feature = "std")]
fn xml_attr(tag: &str, attr: &str) -> Option<String> {
    let needle = format!("{attr}=\"");
    let start = tag.find(&needle)? + needle.len();
    let rest = &tag[start..];
    let end = rest.find('"')?;
    Some(rest[..end].to_owned())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    #[cfg(feature = "std")]
    use std::io::Write;

    #[test]
    fn native_model_port_steps_deterministically() {
        let mut port = NativeModelPort::new("gain", |dt_s, input: &f64| Ok(input + dt_s));

        assert_eq!(port.metadata().kind, ModelPortKind::Native);
        assert_eq!(port.step(0.25, &2.0).unwrap().to_bits(), 2.25_f64.to_bits());
    }

    #[test]
    fn fmu_spec_declares_fmi_metadata() {
        let spec = FmuCoSimulationPortSpec {
            id: "engine-fmu".into(),
            fmu: "engine.fmu".into(),
            model_identifier: "engine".into(),
            fmi_version: "3.0".into(),
            input_variables: vec!["throttle".into()],
            output_variables: vec!["thrust".into()],
        };

        assert_eq!(spec.metadata().kind, ModelPortKind::FmiCoSimulation);
    }

    #[derive(Clone, Debug)]
    struct MockGainFmu;

    impl FmuCoSimulationBackend for MockGainFmu {
        type Input = f64;
        type Output = f64;

        fn do_step(
            &mut self,
            spec: &FmuCoSimulationPortSpec,
            _current_time_s: f64,
            step_size_s: f64,
            input: &Self::Input,
        ) -> Result<Self::Output, ModelEvalError> {
            assert_eq!(spec.model_identifier, "gain");
            Ok(input + step_size_s)
        }
    }

    #[test]
    fn fmu_cosim_port_matches_native_toy_model() {
        let mut native = NativeModelPort::new("gain-native", |dt_s, input: &f64| Ok(input + dt_s));
        let spec = FmuCoSimulationPortSpec {
            id: "gain-fmu".into(),
            fmu: "gain.fmu".into(),
            model_identifier: "gain".into(),
            fmi_version: "3.0".into(),
            input_variables: vec!["x".into()],
            output_variables: vec!["y".into()],
        };
        let mut fmu = FmuCoSimulationModelPort::new(spec, MockGainFmu);

        assert_eq!(fmu.metadata().kind, ModelPortKind::FmiCoSimulation);
        for input in [0.0, 1.0, 2.5] {
            let native_output = native.step(0.25, &input).unwrap();
            let fmu_output = fmu.step(0.25, &input).unwrap();
            assert_eq!(native_output.to_bits(), fmu_output.to_bits());
        }
        assert_eq!(fmu.current_time_s().to_bits(), 0.75_f64.to_bits());
    }

    #[cfg(feature = "std")]
    #[derive(Clone, Debug)]
    struct ResourceGainFmu {
        gain: f64,
    }

    #[cfg(feature = "std")]
    impl ResourceGainFmu {
        fn from_archive(archive: &FmuArchive) -> Self {
            let resource = archive
                .entry_text("resources/openbmp-gain.txt")
                .expect("read resource gain");
            let gain = resource.trim().parse::<f64>().expect("parse gain");
            Self { gain }
        }
    }

    #[cfg(feature = "std")]
    impl FmuCoSimulationBackend for ResourceGainFmu {
        type Input = f64;
        type Output = f64;

        fn do_step(
            &mut self,
            spec: &FmuCoSimulationPortSpec,
            _current_time_s: f64,
            step_size_s: f64,
            input: &Self::Input,
        ) -> Result<Self::Output, ModelEvalError> {
            assert_eq!(spec.fmi_version, "3.0");
            assert_eq!(spec.model_identifier, "gain");
            Ok(*input * self.gain + step_size_s)
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn stored_fmu_archive_drives_cosim_port_like_native_model() {
        let path = temp_fmu_path("gain");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="gain">
  <CoSimulation modelIdentifier="gain"/>
  <ModelVariables>
    <ScalarVariable name="x" valueReference="1" causality="input"><Float64/></ScalarVariable>
    <ScalarVariable name="y" valueReference="2" causality="output"><Float64/></ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &path,
            &[
                ("modelDescription.xml", model_description.as_slice()),
                ("resources/openbmp-gain.txt", b"2.0"),
            ],
        )
        .expect("write fmu");

        let archive = FmuArchive::load(&path).expect("load fmu archive");
        let spec = FmuCoSimulationPortSpec::from_archive("gain-fmu", &archive);
        assert_eq!(spec.input_variables, vec!["x"]);
        assert_eq!(spec.output_variables, vec!["y"]);

        let mut native =
            NativeModelPort::new("gain-native", |dt_s, input: &f64| Ok(*input * 2.0 + dt_s));
        let mut fmu = FmuCoSimulationModelPort::new(spec, ResourceGainFmu::from_archive(&archive));
        for input in [0.0, 1.25, 3.5] {
            let native_output = native.step(0.5, &input).unwrap();
            let fmu_output = fmu.step(0.5, &input).unwrap();
            assert_eq!(native_output.to_bits(), fmu_output.to_bits());
        }
        assert_eq!(fmu.current_time_s().to_bits(), 1.5_f64.to_bits());

        std::fs::remove_file(path).ok();
    }

    #[cfg(feature = "std")]
    #[test]
    fn deflated_fmu_archive_parses_metadata_and_resources() {
        let path = temp_fmu_path("deflated-gain");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="gain" instantiationToken="gain-token">
  <CoSimulation modelIdentifier="gain"/>
  <ModelVariables>
    <Float64 name="x" valueReference="1" causality="input"/>
    <Float64 name="y" valueReference="2" causality="output"/>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_deflated_zip(
            &path,
            &[
                ("modelDescription.xml", model_description.as_slice()),
                ("resources/openbmp-gain.txt", b"3.0"),
            ],
        )
        .expect("write deflated fmu");

        let archive = FmuArchive::load(&path).expect("load deflated fmu archive");
        let description = archive.model_description();

        assert_eq!(description.instantiation_token, Some("gain-token".into()));
        assert_eq!(description.model_identifier, "gain");
        assert_eq!(description.input_variables, vec!["x"]);
        assert_eq!(description.output_variables, vec!["y"]);
        assert_eq!(
            archive
                .entry_text("resources/openbmp-gain.txt")
                .expect("read deflated resource"),
            "3.0"
        );

        std::fs::remove_file(path).ok();
    }

    #[cfg(feature = "std")]
    #[test]
    fn fmu_archive_parses_typed_value_references() {
        let path = temp_fmu_path("typed-vrs");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="engine" instantiationToken="engine-token">
  <CoSimulation modelIdentifier="engine" canHandleVariableCommunicationStepSize="true"/>
  <ModelVariables>
    <Float64 name="throttle" valueReference="10" causality="input"/>
    <Int32 name="mode" valueReference="12" causality="input"/>
    <UInt64 name="command_seq" valueReference="11" causality="input"/>
    <Float64 name="thrust" valueReference="20" causality="output"/>
    <Int32 name="status" valueReference="22" causality="output"/>
    <UInt64 name="sample_seq" valueReference="21" causality="output"/>
  </ModelVariables>
  <ModelStructure>
    <Output valueReference="20"/>
    <Output valueReference="22"/>
    <Output valueReference="21"/>
  </ModelStructure>
</fmiModelDescription>
"#;
        write_stored_zip(
            &path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fmu");

        let archive = FmuArchive::load(&path).expect("load fmu archive");
        let description = archive.model_description();

        assert_eq!(description.instantiation_token, Some("engine-token".into()));
        assert_eq!(
            description.input_variables,
            vec!["throttle", "mode", "command_seq"]
        );
        assert_eq!(
            description.output_variables,
            vec!["thrust", "status", "sample_seq"]
        );
        assert_eq!(
            description.scalar_variables,
            vec![
                FmiScalarVariable {
                    name: "throttle".into(),
                    value_reference: 10,
                    causality: FmiVariableCausality::Input,
                    variable_type: FmiVariableType::Float64,
                },
                FmiScalarVariable {
                    name: "mode".into(),
                    value_reference: 12,
                    causality: FmiVariableCausality::Input,
                    variable_type: FmiVariableType::Int32,
                },
                FmiScalarVariable {
                    name: "command_seq".into(),
                    value_reference: 11,
                    causality: FmiVariableCausality::Input,
                    variable_type: FmiVariableType::UInt64,
                },
                FmiScalarVariable {
                    name: "thrust".into(),
                    value_reference: 20,
                    causality: FmiVariableCausality::Output,
                    variable_type: FmiVariableType::Float64,
                },
                FmiScalarVariable {
                    name: "status".into(),
                    value_reference: 22,
                    causality: FmiVariableCausality::Output,
                    variable_type: FmiVariableType::Int32,
                },
                FmiScalarVariable {
                    name: "sample_seq".into(),
                    value_reference: 21,
                    causality: FmiVariableCausality::Output,
                    variable_type: FmiVariableType::UInt64,
                },
            ]
        );

        std::fs::remove_file(path).ok();
    }

    #[cfg(feature = "std")]
    #[test]
    fn fmu_archive_parses_fmi3_clock_subset_metadata() {
        let path = temp_fmu_path("clock-subset");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="clocked" instantiationToken="clock-token">
  <CoSimulation modelIdentifier="clocked"/>
  <ModelVariables>
    <Clock name="sample_clock" valueReference="100" causality="input" intervalVariability="constant" intervalDecimal="0.1" shiftDecimal="0.0"/>
    <Float64 name="sampled_thrust" valueReference="20" causality="output" clocks="100"/>
    <Clock name="done_clock" valueReference="101" causality="output" intervalVariability="triggered"/>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fmu");

        let archive = FmuArchive::load(&path).expect("load fmu archive");
        let description = archive.model_description();

        assert_eq!(
            description.clock_variables,
            vec![
                FmiClockVariable {
                    name: "sample_clock".into(),
                    value_reference: 100,
                    causality: FmiVariableCausality::Input,
                    interval_variability: FmiClockIntervalVariability::Constant,
                    interval_decimal: Some("0.1".into()),
                    shift_decimal: Some("0.0".into()),
                },
                FmiClockVariable {
                    name: "done_clock".into(),
                    value_reference: 101,
                    causality: FmiVariableCausality::Output,
                    interval_variability: FmiClockIntervalVariability::Triggered,
                    interval_decimal: None,
                    shift_decimal: None,
                },
            ]
        );
        assert_eq!(
            description.clocked_variables,
            vec![FmiClockedVariable {
                name: "sampled_thrust".into(),
                value_reference: 20,
                clock_references: vec![100],
            }]
        );
        assert_eq!(description.output_variables, vec!["sampled_thrust"]);

        std::fs::remove_file(path).ok();
    }

    #[cfg(feature = "std")]
    #[test]
    fn fmu_archive_rejects_invalid_clock_reference_list() {
        let path = temp_fmu_path("bad-clock-ref");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="clocked">
  <CoSimulation modelIdentifier="clocked"/>
  <ModelVariables>
    <Clock name="sample_clock" valueReference="100" causality="input" intervalVariability="constant" intervalDecimal="0.1"/>
    <Float64 name="sampled_thrust" valueReference="20" causality="output" clocks="sample_clock"/>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fmu");

        let err = FmuArchive::load(&path).expect_err("invalid clock reference should fail");

        assert!(
            err.summary().contains("invalid clocks valueReference"),
            "unexpected error: {err}"
        );
        std::fs::remove_file(path).ok();
    }

    #[cfg(feature = "std")]
    #[test]
    fn fmu_archive_rejects_scalar_variable_without_value_reference() {
        let path = temp_fmu_path("missing-vr");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="bad">
  <CoSimulation modelIdentifier="bad"/>
  <ModelVariables>
    <ScalarVariable name="x" causality="input"><Float64/></ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fmu");

        let err = FmuArchive::load(&path).expect_err("missing valueReference should fail");

        assert!(
            err.summary().contains("missing valueReference"),
            "unexpected error: {err}"
        );
        std::fs::remove_file(path).ok();
    }

    #[cfg(feature = "std")]
    fn temp_fmu_path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("openbmp-{name}-{}.fmu", std::process::id()))
    }

    #[cfg(feature = "std")]
    fn write_stored_zip(path: &std::path::Path, entries: &[(&str, &[u8])]) -> std::io::Result<()> {
        let mut file = std::fs::File::create(path)?;
        let mut central_directory = Vec::new();
        let mut offset = 0_u32;
        for (name, data) in entries {
            let name_bytes = name.as_bytes();
            write_u32(&mut file, 0x0403_4b50)?;
            write_u16(&mut file, 20)?;
            write_u16(&mut file, 0)?;
            write_u16(&mut file, 0)?;
            write_u16(&mut file, 0)?;
            write_u16(&mut file, 0)?;
            write_u32(&mut file, 0)?;
            write_u32(&mut file, data.len() as u32)?;
            write_u32(&mut file, data.len() as u32)?;
            write_u16(&mut file, name_bytes.len() as u16)?;
            write_u16(&mut file, 0)?;
            file.write_all(name_bytes)?;
            file.write_all(data)?;

            write_u32(&mut central_directory, 0x0201_4b50)?;
            write_u16(&mut central_directory, 20)?;
            write_u16(&mut central_directory, 20)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 0)?;
            write_u32(&mut central_directory, 0)?;
            write_u32(&mut central_directory, data.len() as u32)?;
            write_u32(&mut central_directory, data.len() as u32)?;
            write_u16(&mut central_directory, name_bytes.len() as u16)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 0)?;
            write_u32(&mut central_directory, 0)?;
            write_u32(&mut central_directory, offset)?;
            central_directory.write_all(name_bytes)?;

            offset += 30 + name_bytes.len() as u32 + data.len() as u32;
        }

        let central_directory_offset = offset;
        file.write_all(&central_directory)?;
        write_u32(&mut file, 0x0605_4b50)?;
        write_u16(&mut file, 0)?;
        write_u16(&mut file, 0)?;
        write_u16(&mut file, entries.len() as u16)?;
        write_u16(&mut file, entries.len() as u16)?;
        write_u32(&mut file, central_directory.len() as u32)?;
        write_u32(&mut file, central_directory_offset)?;
        write_u16(&mut file, 0)?;
        Ok(())
    }

    #[cfg(feature = "std")]
    fn write_deflated_zip(
        path: &std::path::Path,
        entries: &[(&str, &[u8])],
    ) -> std::io::Result<()> {
        let mut file = std::fs::File::create(path)?;
        let mut central_directory = Vec::new();
        let mut offset = 0_u32;
        for (name, data) in entries {
            let name_bytes = name.as_bytes();
            let mut encoder =
                flate2::write::DeflateEncoder::new(Vec::new(), flate2::Compression::default());
            encoder.write_all(data)?;
            let compressed = encoder.finish()?;

            write_u32(&mut file, 0x0403_4b50)?;
            write_u16(&mut file, 20)?;
            write_u16(&mut file, 0)?;
            write_u16(&mut file, 8)?;
            write_u16(&mut file, 0)?;
            write_u16(&mut file, 0)?;
            write_u32(&mut file, 0)?;
            write_u32(&mut file, compressed.len() as u32)?;
            write_u32(&mut file, data.len() as u32)?;
            write_u16(&mut file, name_bytes.len() as u16)?;
            write_u16(&mut file, 0)?;
            file.write_all(name_bytes)?;
            file.write_all(&compressed)?;

            write_u32(&mut central_directory, 0x0201_4b50)?;
            write_u16(&mut central_directory, 20)?;
            write_u16(&mut central_directory, 20)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 8)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 0)?;
            write_u32(&mut central_directory, 0)?;
            write_u32(&mut central_directory, compressed.len() as u32)?;
            write_u32(&mut central_directory, data.len() as u32)?;
            write_u16(&mut central_directory, name_bytes.len() as u16)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 0)?;
            write_u16(&mut central_directory, 0)?;
            write_u32(&mut central_directory, 0)?;
            write_u32(&mut central_directory, offset)?;
            central_directory.write_all(name_bytes)?;

            offset += 30 + name_bytes.len() as u32 + compressed.len() as u32;
        }

        let central_directory_offset = offset;
        file.write_all(&central_directory)?;
        write_u32(&mut file, 0x0605_4b50)?;
        write_u16(&mut file, 0)?;
        write_u16(&mut file, 0)?;
        write_u16(&mut file, entries.len() as u16)?;
        write_u16(&mut file, entries.len() as u16)?;
        write_u32(&mut file, central_directory.len() as u32)?;
        write_u32(&mut file, central_directory_offset)?;
        write_u16(&mut file, 0)?;
        Ok(())
    }

    #[cfg(feature = "std")]
    fn write_u16<W: Write>(writer: &mut W, value: u16) -> std::io::Result<()> {
        writer.write_all(&value.to_le_bytes())
    }

    #[cfg(feature = "std")]
    fn write_u32<W: Write>(writer: &mut W, value: u32) -> std::io::Result<()> {
        writer.write_all(&value.to_le_bytes())
    }
}
