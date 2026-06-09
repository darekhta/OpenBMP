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
use std::path::{Path, PathBuf};

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
    /// Co-simulation model identifier.
    pub model_identifier: String,
    /// Input variable names.
    pub input_variables: Vec<String>,
    /// Output variable names.
    pub output_variables: Vec<String>,
}

/// Restricted FMU archive reader used for import smoke tests.
///
/// This loader supports uncompressed ZIP entries only. It validates the
/// presence of `modelDescription.xml`, parses the FMI version,
/// co-simulation model identifier, and scalar-variable causality, and
/// exposes resource text for deterministic adapter tests. It does not
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
    /// when it is not a supported stored-entry ZIP, or when required
    /// FMI co-simulation metadata is absent.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, FmuArchiveError> {
        let path = path.as_ref();
        let bytes = std::fs::read(path)
            .map_err(|source| FmuArchiveError::new(format!("read {}: {source}", path.display())))?;
        let entries = read_stored_zip_entries(&bytes)?;
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
fn read_stored_zip_entries(bytes: &[u8]) -> Result<BTreeMap<String, Vec<u8>>, FmuArchiveError> {
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
        let compressed_size = usize::try_from(read_u32(bytes, cursor + 20)?)
            .map_err(|_| FmuArchiveError::new("compressed size overflows usize"))?;
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
        if method != 0 {
            return Err(FmuArchiveError::new(format!(
                "entry {name} uses unsupported compression method {method}; only stored entries are supported"
            )));
        }
        let data = read_stored_zip_entry(bytes, local_header_offset, compressed_size)?;
        entries.insert(name, data);
        cursor = name_end
            .checked_add(extra_len)
            .and_then(|next| next.checked_add(comment_len))
            .ok_or_else(|| FmuArchiveError::new("central-directory cursor overflow"))?;
    }
    Ok(entries)
}

#[cfg(feature = "std")]
fn read_stored_zip_entry(
    bytes: &[u8],
    local_header_offset: usize,
    compressed_size: usize,
) -> Result<Vec<u8>, FmuArchiveError> {
    ensure_signature(bytes, local_header_offset, 0x0403_4b50, "local file header")?;
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
    Ok(bytes[data_start..data_end].to_vec())
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
    let co_sim = find_xml_tag(xml, "CoSimulation")?;
    let model_identifier = xml_attr(co_sim, "modelIdentifier").ok_or_else(|| {
        FmuArchiveError::new("modelDescription.xml CoSimulation missing modelIdentifier")
    })?;
    let mut input_variables = Vec::new();
    let mut output_variables = Vec::new();
    let mut rest = xml;
    while let Some(start) = rest.find("<ScalarVariable") {
        rest = &rest[start..];
        let Some(end) = rest.find('>') else {
            return Err(FmuArchiveError::new(
                "modelDescription.xml has unterminated ScalarVariable tag",
            ));
        };
        let tag = &rest[..=end];
        if let (Some(name), Some(causality)) = (xml_attr(tag, "name"), xml_attr(tag, "causality")) {
            match causality.as_str() {
                "input" => input_variables.push(name),
                "output" => output_variables.push(name),
                _ => {}
            }
        }
        rest = &rest[end + 1..];
    }
    Ok(FmuModelDescription {
        fmi_version,
        model_name,
        model_identifier,
        input_variables,
        output_variables,
    })
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
        assert_eq!(port.step(0.25, &2.0).unwrap(), 2.25);
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
            assert_eq!(native_output, fmu_output);
        }
        assert_eq!(fmu.current_time_s(), 0.75);
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
    <ScalarVariable name="x" causality="input"><Float64/></ScalarVariable>
    <ScalarVariable name="y" causality="output"><Float64/></ScalarVariable>
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
            assert_eq!(native_output, fmu_output);
        }
        assert_eq!(fmu.current_time_s(), 1.5);

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
    fn write_u16<W: Write>(writer: &mut W, value: u16) -> std::io::Result<()> {
        writer.write_all(&value.to_le_bytes())
    }

    #[cfg(feature = "std")]
    fn write_u32<W: Write>(writer: &mut W, value: u32) -> std::io::Result<()> {
        writer.write_all(&value.to_le_bytes())
    }
}
