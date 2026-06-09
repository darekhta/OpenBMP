//! Host-only FMI adapter boundary.
//!
//! `openbmp-models` owns the portable [`openbmp_models::ModelPort`]
//! abstraction and restricted FMU archive metadata reader. This crate
//! owns the host dynamic-library boundary needed for importer smoke
//! tests. It intentionally checks only a small FMI 3 co-simulation C
//! API surface and does not claim FMI conformance.

use std::ffi::CStr;
use std::fmt;
use std::path::{Path, PathBuf};

use libloading::Library;
use openbmp_models::FmuArchive;
use thiserror::Error;

/// FMI 3 co-simulation symbols checked by the host smoke probe.
///
/// FMI shared libraries export unprefixed C function names when loaded
/// dynamically. This set is intentionally small: it proves the library
/// can be opened and exposes the lifecycle, step, and Float64 variable-access
/// hooks that an adapter would need before richer typed access is wired.
pub const FMI3_COSIMULATION_SMOKE_SYMBOLS: &[&str] = &[
    "fmi3GetVersion",
    "fmi3InstantiateCoSimulation",
    "fmi3EnterInitializationMode",
    "fmi3ExitInitializationMode",
    "fmi3SetFloat64",
    "fmi3GetFloat64",
    "fmi3DoStep",
    "fmi3Terminate",
    "fmi3FreeInstance",
];

/// Error raised by the host FMI dynamic-library probe.
#[derive(Debug, Error)]
pub enum FmiImportError {
    /// The FMU archive entry was missing.
    #[error("FMU archive is missing binary entry {entry}")]
    MissingArchiveEntry {
        /// Archive entry name.
        entry: String,
    },
    /// The archive entry did not have a safe filename.
    #[error("FMU binary entry {entry} has no materializable file name")]
    InvalidArchiveEntryName {
        /// Archive entry name.
        entry: String,
    },
    /// Filesystem IO failed while materializing a binary.
    #[error("could not materialize FMU binary {path}: {source}")]
    MaterializeIo {
        /// Destination path.
        path: PathBuf,
        /// Underlying IO error.
        #[source]
        source: std::io::Error,
    },
    /// Dynamic-library loading failed.
    #[error("could not load FMI dynamic library {path}: {source}")]
    LibraryLoad {
        /// Library path.
        path: PathBuf,
        /// Loader error.
        #[source]
        source: libloading::Error,
    },
    /// A required FMI symbol was absent.
    #[error("FMI dynamic library {path} is missing symbol {symbol}: {source}")]
    MissingSymbol {
        /// Library path.
        path: PathBuf,
        /// Symbol name.
        symbol: String,
        /// Loader error.
        #[source]
        source: libloading::Error,
    },
    /// `fmi3GetVersion` returned null.
    #[error("FMI dynamic library {path} returned null from fmi3GetVersion")]
    NullVersion {
        /// Library path.
        path: PathBuf,
    },
    /// `fmi3GetVersion` returned non-UTF-8 text.
    #[error("FMI dynamic library {path} returned non-UTF-8 fmi3GetVersion text: {source}")]
    VersionUtf8 {
        /// Library path.
        path: PathBuf,
        /// UTF-8 conversion error.
        #[source]
        source: std::str::Utf8Error,
    },
}

/// Report from an FMI 3 dynamic-library smoke check.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Fmi3LibraryReport {
    /// Library path that was loaded.
    pub path: PathBuf,
    /// Version string returned by `fmi3GetVersion`.
    pub fmi_version: String,
    /// Symbols verified in order.
    pub checked_symbols: Vec<String>,
}

/// Loaded FMI 3 dynamic library.
pub struct Fmi3DynamicLibrary {
    path: PathBuf,
    library: Library,
}

impl fmt::Debug for Fmi3DynamicLibrary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Fmi3DynamicLibrary")
            .field("path", &self.path)
            .finish_non_exhaustive()
    }
}

impl Fmi3DynamicLibrary {
    /// Open a materialized FMI shared library.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError::LibraryLoad`] when the host loader
    /// cannot open the path.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, FmiImportError> {
        let path = path.as_ref().to_path_buf();
        let library = unsafe {
            // Loading an arbitrary host dynamic library is inherently an
            // unsafe FFI boundary. This crate deliberately contains that
            // boundary so portable model crates do not need unsafe code.
            Library::new(&path)
        }
        .map_err(|source| FmiImportError::LibraryLoad {
            path: path.clone(),
            source,
        })?;
        Ok(Self { path, library })
    }

    /// Materialize one binary entry from a loaded FMU archive and open it.
    ///
    /// `entry` is an explicit FMU archive path such as
    /// `binaries/x86_64-linux/model.so`; OpenBMP does not infer
    /// platform-specific FMI packaging here.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the entry cannot be written or
    /// the host loader cannot open the resulting file.
    pub fn open_archive_binary(
        archive: &FmuArchive,
        entry: &str,
        output_dir: impl AsRef<Path>,
    ) -> Result<Self, FmiImportError> {
        let path = materialize_archive_binary(archive, entry, output_dir)?;
        Self::open(path)
    }

    /// Path loaded by this dynamic-library handle.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Return the string produced by `fmi3GetVersion`.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] if the version symbol is missing,
    /// returns null, or returns non-UTF-8 text.
    pub fn get_version(&self) -> Result<String, FmiImportError> {
        type Fmi3GetVersion = unsafe extern "C" fn() -> *const std::ffi::c_char;
        let symbol = self.symbol::<Fmi3GetVersion>("fmi3GetVersion")?;
        let ptr = unsafe {
            // The imported symbol is trusted only for this smoke call.
            // A null pointer is rejected before conversion.
            symbol()
        };
        if ptr.is_null() {
            return Err(FmiImportError::NullVersion {
                path: self.path.clone(),
            });
        }
        let version = unsafe {
            // FMI returns a NUL-terminated C string owned by the FMU.
            CStr::from_ptr(ptr)
        }
        .to_str()
        .map_err(|source| FmiImportError::VersionUtf8 {
            path: self.path.clone(),
            source,
        })?
        .to_owned();
        Ok(version)
    }

    /// Verify the FMI 3 co-simulation smoke symbol set.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when any required symbol is missing
    /// or `fmi3GetVersion` cannot be called.
    pub fn check_cosimulation_smoke_symbols(&self) -> Result<Fmi3LibraryReport, FmiImportError> {
        let version = self.get_version()?;
        let mut checked_symbols = Vec::with_capacity(FMI3_COSIMULATION_SMOKE_SYMBOLS.len());
        for symbol in FMI3_COSIMULATION_SMOKE_SYMBOLS {
            self.symbol::<*mut std::ffi::c_void>(symbol)?;
            checked_symbols.push((*symbol).to_owned());
        }
        Ok(Fmi3LibraryReport {
            path: self.path.clone(),
            fmi_version: version,
            checked_symbols,
        })
    }

    fn symbol<T>(&self, name: &str) -> Result<libloading::Symbol<'_, T>, FmiImportError> {
        let mut bytes = Vec::with_capacity(name.len() + 1);
        bytes.extend_from_slice(name.as_bytes());
        bytes.push(0);
        unsafe {
            // Symbol type safety is provided by the caller. Most smoke
            // checks use opaque pointers; `get_version` uses the FMI C
            // signature and immediately validates the returned pointer.
            self.library.get::<T>(bytes.as_slice())
        }
        .map_err(|source| FmiImportError::MissingSymbol {
            path: self.path.clone(),
            symbol: name.to_owned(),
            source,
        })
    }
}

/// Materialize one binary entry from a loaded FMU archive.
///
/// The archive entry's basename is used as the output filename to avoid
/// path traversal from archive-controlled names.
///
/// # Errors
///
/// Returns [`FmiImportError`] when the entry is absent or cannot be
/// written.
pub fn materialize_archive_binary(
    archive: &FmuArchive,
    entry: &str,
    output_dir: impl AsRef<Path>,
) -> Result<PathBuf, FmiImportError> {
    let bytes = archive
        .entry(entry)
        .ok_or_else(|| FmiImportError::MissingArchiveEntry {
            entry: entry.to_owned(),
        })?;
    let file_name = Path::new(entry)
        .file_name()
        .filter(|name| !name.is_empty())
        .ok_or_else(|| FmiImportError::InvalidArchiveEntryName {
            entry: entry.to_owned(),
        })?;
    let output_dir = output_dir.as_ref();
    std::fs::create_dir_all(output_dir).map_err(|source| FmiImportError::MaterializeIo {
        path: output_dir.to_path_buf(),
        source,
    })?;
    let path = output_dir.join(file_name);
    std::fs::write(&path, bytes).map_err(|source| FmiImportError::MaterializeIo {
        path: path.clone(),
        source,
    })?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut permissions = std::fs::metadata(&path)
            .map_err(|source| FmiImportError::MaterializeIo {
                path: path.clone(),
                source,
            })?
            .permissions();
        permissions.set_mode(0o755);
        std::fs::set_permissions(&path, permissions).map_err(|source| {
            FmiImportError::MaterializeIo {
                path: path.clone(),
                source,
            }
        })?;
    }
    Ok(path)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;
    use std::io::Write;
    use std::process::Command;

    use openbmp_models::FmuArchive;

    #[test]
    fn loads_fmi3_dynamic_library_and_checks_smoke_symbols() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());

        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");
        let report = library
            .check_cosimulation_smoke_symbols()
            .expect("check FMI smoke symbols");

        assert_eq!(report.fmi_version, "3.0");
        assert_eq!(report.checked_symbols, FMI3_COSIMULATION_SMOKE_SYMBOLS);
    }

    #[test]
    fn materializes_fmu_binary_entry_and_loads_it() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library_bytes = std::fs::read(&library_path).expect("read fixture library");
        let library_name = library_path.file_name().unwrap().to_string_lossy();
        let binary_entry = format!("binaries/openbmp-test/{library_name}");
        let fmu_path = temp.path().join("fixture.fmu");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="fixture">
  <CoSimulation modelIdentifier="fixture"/>
  <ModelVariables>
    <ScalarVariable name="u" causality="input"><Float64/></ScalarVariable>
    <ScalarVariable name="y" causality="output"><Float64/></ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &fmu_path,
            &[
                ("modelDescription.xml", model_description.as_slice()),
                (binary_entry.as_str(), library_bytes.as_slice()),
            ],
        )
        .expect("write fixture fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load fixture fmu");

        let library = Fmi3DynamicLibrary::open_archive_binary(
            &archive,
            binary_entry.as_str(),
            temp.path().join("materialized"),
        )
        .expect("open materialized fmu binary");

        assert_eq!(library.get_version().expect("version"), "3.0");
    }

    #[test]
    fn reports_missing_required_symbol() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_incomplete_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");

        let err = library
            .check_cosimulation_smoke_symbols()
            .expect_err("incomplete fixture should fail");

        assert!(
            matches!(err, FmiImportError::MissingSymbol { ref symbol, .. }
                if symbol == "fmi3InstantiateCoSimulation"),
            "unexpected error: {err}",
        );
    }

    fn compile_fixture_library(dir: &Path) -> PathBuf {
        compile_rust_cdylib(dir, "fixture", FIXTURE_LIBRARY_SOURCE)
    }

    fn compile_incomplete_fixture_library(dir: &Path) -> PathBuf {
        compile_rust_cdylib(dir, "incomplete", INCOMPLETE_LIBRARY_SOURCE)
    }

    fn compile_rust_cdylib(dir: &Path, name: &str, source: &str) -> PathBuf {
        let source_path = dir.join(format!("{name}.rs"));
        std::fs::write(&source_path, source).expect("write fixture source");
        let library_name = format!(
            "{}{name}.{}",
            std::env::consts::DLL_PREFIX,
            std::env::consts::DLL_EXTENSION
        );
        let library_path = dir.join(library_name);
        let rustc = std::env::var_os("RUSTC").unwrap_or_else(|| "rustc".into());
        let output = Command::new(rustc)
            .arg("--crate-type")
            .arg("cdylib")
            .arg("--edition")
            .arg("2024")
            .arg(&source_path)
            .arg("-o")
            .arg(&library_path)
            .output()
            .expect("run rustc");
        assert!(
            output.status.success(),
            "rustc failed\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        library_path
    }

    const FIXTURE_LIBRARY_SOURCE: &str = r#"
use std::ffi::c_char;

static VERSION: &[u8] = b"3.0\0";

#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetVersion() -> *const c_char {
    VERSION.as_ptr().cast()
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3InstantiateCoSimulation() {}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3EnterInitializationMode() {}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3ExitInitializationMode() {}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3SetFloat64() {}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetFloat64() {}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3DoStep() {}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3Terminate() {}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3FreeInstance() {}
"#;

    const INCOMPLETE_LIBRARY_SOURCE: &str = r#"
use std::ffi::c_char;

static VERSION: &[u8] = b"3.0\0";

#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetVersion() -> *const c_char {
    VERSION.as_ptr().cast()
}
"#;

    fn write_stored_zip(path: &Path, entries: &[(&str, &[u8])]) -> std::io::Result<()> {
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
        let central_offset = offset;
        file.write_all(&central_directory)?;
        write_u32(&mut file, 0x0605_4b50)?;
        write_u16(&mut file, 0)?;
        write_u16(&mut file, 0)?;
        write_u16(&mut file, entries.len() as u16)?;
        write_u16(&mut file, entries.len() as u16)?;
        write_u32(&mut file, central_directory.len() as u32)?;
        write_u32(&mut file, central_offset)?;
        write_u16(&mut file, 0)?;
        Ok(())
    }

    fn write_u16(out: &mut impl Write, value: u16) -> std::io::Result<()> {
        out.write_all(&value.to_le_bytes())
    }

    fn write_u32(out: &mut impl Write, value: u32) -> std::io::Result<()> {
        out.write_all(&value.to_le_bytes())
    }
}
