//! Host-only FMI adapter boundary.
//!
//! `openbmp-models` owns the portable [`openbmp_models::ModelPort`]
//! abstraction and restricted FMU archive metadata reader. This crate
//! owns the host dynamic-library boundary needed for importer smoke
//! tests. It intentionally checks only a small FMI 3 co-simulation C
//! API surface and does not claim FMI conformance.

use std::ffi::{CStr, CString, c_char, c_int, c_void};
use std::fmt;
use std::path::{Component, Path, PathBuf};

use libloading::Library;
use openbmp_models::{
    FmiClockIntervalVariability, FmiVariableCausality, FmiVariableType, FmuArchive,
    FmuModelDescription,
};
use thiserror::Error;

pub mod export;
pub mod export_abi;

pub use export::{
    Fmi3ExportModel, FmiExportError, fmi3_export_scalar_variable,
    openbmp_fmi_binary_entry_for_current_platform, openbmp_fmi_binary_entry_for_target,
    openbmp_point_mass_export_model, openbmp_point_mass_fmi_binary_entry_for_current_platform,
    openbmp_point_mass_fmi_binary_entry_for_target,
    write_openbmp_point_mass_fmu_archive_for_current_platform,
};

/// Return the standard FMI 3 binary archive entry for a model identifier
/// on the current compilation target.
#[must_use]
pub fn fmi3_binary_entry_for_current_platform(model_identifier: &str) -> Option<String> {
    fmi3_binary_entry_for_target(
        model_identifier,
        std::env::consts::ARCH,
        std::env::consts::OS,
    )
}

/// Return the standard FMI 3 binary archive entry for a model identifier
/// on a supported target.
///
/// The returned path follows the FMI packaging convention
/// `binaries/<fmi-platform>/<modelIdentifier>.<ext>`.
#[must_use]
pub fn fmi3_binary_entry_for_target(
    model_identifier: &str,
    target_arch: &str,
    target_os: &str,
) -> Option<String> {
    if !is_safe_model_identifier(model_identifier) {
        return None;
    }
    let (platform, extension) = match (target_arch, target_os) {
        ("x86_64", "linux") => ("x86_64-linux", "so"),
        ("aarch64", "linux") => ("aarch64-linux", "so"),
        ("x86_64", "macos") => ("x86_64-darwin", "dylib"),
        ("aarch64", "macos") => ("aarch64-darwin", "dylib"),
        ("x86_64", "windows") => ("x86_64-windows", "dll"),
        ("aarch64", "windows") => ("aarch64-windows", "dll"),
        _ => return None,
    };
    Some(format!(
        "binaries/{platform}/{model_identifier}.{extension}"
    ))
}

fn is_safe_model_identifier(model_identifier: &str) -> bool {
    !model_identifier.is_empty()
        && !model_identifier.contains("..")
        && model_identifier
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-'))
}

/// FMI 3 co-simulation symbols checked by the host smoke probe.
///
/// FMI shared libraries export unprefixed C function names when loaded
/// dynamically. This set is intentionally small: it proves the library
/// can be opened and exposes the lifecycle, step, and typed Float64/Int32/UInt64
/// variable-access hooks that an adapter needs before richer orchestration is wired.
pub const FMI3_COSIMULATION_SMOKE_SYMBOLS: &[&str] = &[
    "fmi3GetVersion",
    "fmi3InstantiateCoSimulation",
    "fmi3EnterInitializationMode",
    "fmi3ExitInitializationMode",
    "fmi3SetFloat64",
    "fmi3GetFloat64",
    "fmi3SetInt32",
    "fmi3GetInt32",
    "fmi3SetUInt64",
    "fmi3GetUInt64",
    "fmi3SetClock",
    "fmi3GetClock",
    "fmi3DoStep",
    "fmi3Terminate",
    "fmi3FreeInstance",
];

/// Optional FMI 3 FMU-state rollback symbols.
pub const FMI3_FMU_STATE_SYMBOLS: &[&str] =
    &["fmi3GetFMUState", "fmi3SetFMUState", "fmi3FreeFMUState"];

/// Opaque FMI 3 instance pointer.
pub type Fmi3Instance = *mut c_void;

/// Opaque FMI 3 FMU-state handle.
pub type Fmi3FmuState = *mut c_void;

/// Raw FMI 3 value reference.
pub type Fmi3ValueReferenceRaw = u32;

/// FMI 3 call status.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Fmi3Status {
    /// Call completed successfully.
    Ok,
    /// Call completed with a warning.
    Warning,
    /// The FMU discarded the call result.
    Discard,
    /// Recoverable FMU error.
    Error,
    /// Fatal FMU error.
    Fatal,
    /// Asynchronous operation pending.
    Pending,
}

impl Fmi3Status {
    /// Convert a raw FMI C status code.
    pub fn from_raw(raw: c_int) -> Option<Self> {
        match raw {
            0 => Some(Self::Ok),
            1 => Some(Self::Warning),
            2 => Some(Self::Discard),
            3 => Some(Self::Error),
            4 => Some(Self::Fatal),
            5 => Some(Self::Pending),
            _ => None,
        }
    }

    fn permits_continuation(self) -> bool {
        matches!(self, Self::Ok | Self::Warning)
    }
}

/// Typed FMI value reference grouped for host calls.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Fmi3ValueReference {
    /// Variable name from `modelDescription.xml`.
    pub name: String,
    /// FMI value reference.
    pub value_reference: Fmi3ValueReferenceRaw,
}

/// Typed FMI Clock binding grouped for host calls.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Fmi3ClockBinding {
    /// Clock variable name from `modelDescription.xml`.
    pub name: String,
    /// FMI value reference.
    pub value_reference: Fmi3ValueReferenceRaw,
    /// Clock causality.
    pub causality: FmiVariableCausality,
    /// Declared interval variability.
    pub interval_variability: FmiClockIntervalVariability,
    /// Optional decimal interval preserved as XML text.
    pub interval_decimal: Option<String>,
    /// Optional decimal shift preserved as XML text.
    pub shift_decimal: Option<String>,
}

/// Binding for a scalar variable governed by one or more FMI Clocks.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Fmi3ClockedVariableBinding {
    /// Variable name from `modelDescription.xml`.
    pub name: String,
    /// FMI value reference.
    pub value_reference: Fmi3ValueReferenceRaw,
    /// Governing Clock value references.
    pub clock_references: Vec<Fmi3ValueReferenceRaw>,
}

/// Typed FMI input/output binding plan.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Fmi3VariableBindings {
    /// Float64 inputs consumed by the FMU.
    pub float64_inputs: Vec<Fmi3ValueReference>,
    /// Float64 outputs produced by the FMU.
    pub float64_outputs: Vec<Fmi3ValueReference>,
    /// Int32 inputs consumed by the FMU.
    pub int32_inputs: Vec<Fmi3ValueReference>,
    /// Int32 outputs produced by the FMU.
    pub int32_outputs: Vec<Fmi3ValueReference>,
    /// UInt64 inputs consumed by the FMU.
    pub uint64_inputs: Vec<Fmi3ValueReference>,
    /// UInt64 outputs produced by the FMU.
    pub uint64_outputs: Vec<Fmi3ValueReference>,
    /// FMI 3 Clock variables declared by the FMU.
    pub clocks: Vec<Fmi3ClockBinding>,
    /// Scalar variables governed by Clock value references.
    pub clocked_variables: Vec<Fmi3ClockedVariableBinding>,
}

impl Fmi3VariableBindings {
    /// Build typed bindings from parsed FMI model metadata.
    #[must_use]
    pub fn from_model_description(description: &FmuModelDescription) -> Self {
        let mut bindings = Self::default();
        for variable in &description.scalar_variables {
            let value_reference = Fmi3ValueReference {
                name: variable.name.clone(),
                value_reference: variable.value_reference,
            };
            match (variable.variable_type, variable.causality) {
                (FmiVariableType::Float64, FmiVariableCausality::Input) => {
                    bindings.float64_inputs.push(value_reference);
                }
                (FmiVariableType::Float64, FmiVariableCausality::Output) => {
                    bindings.float64_outputs.push(value_reference);
                }
                (FmiVariableType::Int32, FmiVariableCausality::Input) => {
                    bindings.int32_inputs.push(value_reference);
                }
                (FmiVariableType::Int32, FmiVariableCausality::Output) => {
                    bindings.int32_outputs.push(value_reference);
                }
                (FmiVariableType::UInt64, FmiVariableCausality::Input) => {
                    bindings.uint64_inputs.push(value_reference);
                }
                (FmiVariableType::UInt64, FmiVariableCausality::Output) => {
                    bindings.uint64_outputs.push(value_reference);
                }
                _ => {}
            }
        }
        bindings.clocks = description
            .clock_variables
            .iter()
            .map(|clock| Fmi3ClockBinding {
                name: clock.name.clone(),
                value_reference: clock.value_reference,
                causality: clock.causality,
                interval_variability: clock.interval_variability,
                interval_decimal: clock.interval_decimal.clone(),
                shift_decimal: clock.shift_decimal.clone(),
            })
            .collect();
        bindings.clocked_variables = description
            .clocked_variables
            .iter()
            .map(|clocked| Fmi3ClockedVariableBinding {
                name: clocked.name.clone(),
                value_reference: clocked.value_reference,
                clock_references: clocked.clock_references.clone(),
            })
            .collect();
        bindings
    }

    /// Build typed bindings from a loaded FMU archive.
    #[must_use]
    pub fn from_archive(archive: &FmuArchive) -> Self {
        Self::from_model_description(archive.model_description())
    }
}

/// FMI co-simulation coupling order.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Fmi3CouplingOrder {
    /// Step FMUs sequentially and feed each output directly into the next input.
    GaussSeidel,
    /// Sample all FMU inputs at the start of the macro step.
    Jacobi,
}

/// FMI `doStep` completion result.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Fmi3DoStepOutcome {
    /// The requested communication step completed.
    Complete,
    /// The FMU returned early before the requested communication step ended.
    EarlyReturn {
        /// Last successful communication time reported by the FMU.
        last_successful_time_s: f64,
    },
}

/// Report returned by a typed `fmi3DoStep` call.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Fmi3DoStepReport {
    /// FMI status accepted by the wrapper.
    pub status: Fmi3Status,
    /// Whether the FMU requested event handling.
    pub event_handling_needed: bool,
    /// Whether the FMU requested simulation termination.
    pub terminate_simulation: bool,
    /// Step completion outcome.
    pub outcome: Fmi3DoStepOutcome,
}

/// Co-simulation master plan derived from an FMU archive.
#[derive(Clone, Debug, PartialEq)]
pub struct Fmi3MasterPlan {
    /// Co-simulation model identifier from `modelDescription.xml`.
    pub model_identifier: String,
    /// FMI version declared by the FMU.
    pub fmi_version: String,
    /// Fixed communication step requested by the OpenBMP master.
    pub communication_step_s: f64,
    /// Coupling order for multi-FMU stepping.
    pub coupling_order: Fmi3CouplingOrder,
    /// Typed value-reference bindings for supported I/O variables.
    pub variables: Fmi3VariableBindings,
}

impl Fmi3MasterPlan {
    /// Build a master plan from an archive and fixed communication step.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when `communication_step_s` is not
    /// positive and finite.
    pub fn from_archive(
        archive: &FmuArchive,
        communication_step_s: f64,
        coupling_order: Fmi3CouplingOrder,
    ) -> Result<Self, FmiImportError> {
        if !communication_step_s.is_finite() || communication_step_s <= 0.0 {
            return Err(FmiImportError::InvalidCommunicationStep {
                step_s: communication_step_s,
            });
        }
        let description = archive.model_description();
        Ok(Self {
            model_identifier: description.model_identifier.clone(),
            fmi_version: description.fmi_version.clone(),
            communication_step_s,
            coupling_order,
            variables: Fmi3VariableBindings::from_model_description(description),
        })
    }
}

/// Typed input values for one FMI communication step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Fmi3InputSample {
    /// Float64 values in `Fmi3MasterPlan::variables.float64_inputs` order.
    pub float64: Vec<f64>,
    /// Int32 values in `Fmi3MasterPlan::variables.int32_inputs` order.
    pub int32: Vec<i32>,
    /// UInt64 values in `Fmi3MasterPlan::variables.uint64_inputs` order.
    pub uint64: Vec<u64>,
}

/// Typed output values read after one FMI communication step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Fmi3OutputSample {
    /// Float64 values in `Fmi3MasterPlan::variables.float64_outputs` order.
    pub float64: Vec<f64>,
    /// Int32 values in `Fmi3MasterPlan::variables.int32_outputs` order.
    pub int32: Vec<i32>,
    /// UInt64 values in `Fmi3MasterPlan::variables.uint64_outputs` order.
    pub uint64: Vec<u64>,
}

/// Result of one single-FMU master step.
#[derive(Clone, Debug, PartialEq)]
pub struct Fmi3StepResult {
    /// Outputs read after the step.
    pub outputs: Fmi3OutputSample,
    /// `fmi3DoStep` report.
    pub do_step: Fmi3DoStepReport,
    /// Master time after applying complete-step or early-return timing.
    pub current_time_s: f64,
}

/// One FMI Clock activation published by the single-FMU master.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Fmi3ClockScheduleActivation {
    /// Clock variable name from `modelDescription.xml`.
    pub name: String,
    /// FMI Clock value reference.
    pub value_reference: Fmi3ValueReferenceRaw,
    /// Boolean activation value sent to `fmi3SetClock`.
    pub active: bool,
}

/// Report from publishing deterministic periodic FMI input clocks.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Fmi3ClockScheduleReport {
    /// Scheduled input-clock activations in modelDescription order.
    pub activations: Vec<Fmi3ClockScheduleActivation>,
}

/// FMU-state rollback policy for checkpointed master steps.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Fmi3RollbackPolicy {
    /// Keep the FMU state produced by a successful step.
    PreserveCompletedStep,
    /// Restore the pre-step FMU state when `fmi3DoStep` returns early.
    RestoreOnEarlyReturn,
}

impl Fmi3RollbackPolicy {
    fn should_restore(self, report: &Fmi3DoStepReport) -> bool {
        matches!(
            (self, report.outcome),
            (
                Self::RestoreOnEarlyReturn,
                Fmi3DoStepOutcome::EarlyReturn { .. }
            )
        )
    }
}

/// Action taken after a checkpointed master step.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Fmi3RollbackAction {
    /// The post-step FMU state was kept.
    Preserved,
    /// The pre-step FMU state was restored.
    Restored {
        /// Master time restored after rollback.
        restored_time_s: f64,
        /// Master time reported by the attempted step before rollback.
        attempted_time_s: f64,
    },
}

/// Result from a checkpointed master step.
#[derive(Clone, Debug, PartialEq)]
pub struct Fmi3RollbackStepResult {
    /// Step result produced before any optional rollback.
    pub step: Fmi3StepResult,
    /// Rollback action selected by the policy.
    pub action: Fmi3RollbackAction,
    /// Master time after applying the rollback action.
    pub current_time_s: f64,
}

/// Options used to instantiate one FMI 3 co-simulation component.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Fmi3CoSimulationInstantiation<'a> {
    /// FMI instance name supplied to `fmi3InstantiateCoSimulation`.
    pub instance_name: &'a str,
    /// FMU `instantiationToken` from `modelDescription.xml`.
    pub instantiation_token: &'a str,
    /// Resource path passed to the FMU, or an empty string when no
    /// resource directory is materialized.
    pub resource_path: &'a str,
    /// FMI `visible` flag.
    pub visible: bool,
    /// FMI `loggingOn` flag.
    pub logging_on: bool,
    /// Whether event mode is used by the importer.
    pub event_mode_used: bool,
    /// Whether early return is allowed by the importer.
    pub early_return_allowed: bool,
}

impl<'a> Fmi3CoSimulationInstantiation<'a> {
    /// Build default co-simulation instantiation options.
    #[must_use]
    pub const fn new(instance_name: &'a str, instantiation_token: &'a str) -> Self {
        Self {
            instance_name,
            instantiation_token,
            resource_path: "",
            visible: false,
            logging_on: false,
            event_mode_used: false,
            early_return_allowed: false,
        }
    }
}

/// Owned FMI 3 co-simulation instance tied to its dynamic library.
///
/// The wrapper owns only the FMI instance pointer. Call [`Self::terminate`]
/// for deterministic FMI termination; the instance is always passed to
/// `fmi3FreeInstance` when the wrapper is dropped.
pub struct Fmi3CoSimulationInstance<'a> {
    library: &'a Fmi3DynamicLibrary,
    instance: Fmi3Instance,
    terminated: bool,
}

impl fmt::Debug for Fmi3CoSimulationInstance<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Fmi3CoSimulationInstance")
            .field("library_path", &self.library.path)
            .field("instance", &self.instance)
            .field("terminated", &self.terminated)
            .finish()
    }
}

impl Drop for Fmi3CoSimulationInstance<'_> {
    fn drop(&mut self) {
        if !self.instance.is_null() {
            let _ = self.library.free_instance(self.instance);
            self.instance = std::ptr::null_mut();
        }
    }
}

impl Fmi3CoSimulationInstance<'_> {
    /// Borrow the raw FMI instance pointer for lower-level helpers.
    #[must_use]
    pub const fn instance(&self) -> Fmi3Instance {
        self.instance
    }

    /// Whether [`Self::terminate`] has completed successfully.
    #[must_use]
    pub const fn is_terminated(&self) -> bool {
        self.terminated
    }

    /// Enter FMI initialization mode.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn enter_initialization_mode(
        &self,
        start_time_s: f64,
        stop_time_s: Option<f64>,
        tolerance: Option<f64>,
    ) -> Result<Fmi3Status, FmiImportError> {
        self.library
            .enter_initialization_mode(self.instance, start_time_s, stop_time_s, tolerance)
    }

    /// Exit FMI initialization mode.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn exit_initialization_mode(&self) -> Result<Fmi3Status, FmiImportError> {
        self.library.exit_initialization_mode(self.instance)
    }

    /// Terminate the FMI instance.
    ///
    /// Calling this method more than once is a no-op after the first
    /// successful termination.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn terminate(&mut self) -> Result<Fmi3Status, FmiImportError> {
        if self.terminated {
            return Ok(Fmi3Status::Ok);
        }
        let status = self.library.terminate(self.instance)?;
        self.terminated = true;
        Ok(status)
    }

    /// Call `fmi3SetFloat64` for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the underlying typed FMI call
    /// fails.
    pub fn set_float64(
        &self,
        value_references: &[Fmi3ValueReferenceRaw],
        values: &[f64],
    ) -> Result<Fmi3Status, FmiImportError> {
        self.library
            .set_float64(self.instance, value_references, values)
    }

    /// Call `fmi3GetFloat64` for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the underlying typed FMI call
    /// fails.
    pub fn get_float64(
        &self,
        value_references: &[Fmi3ValueReferenceRaw],
    ) -> Result<Vec<f64>, FmiImportError> {
        self.library.get_float64(self.instance, value_references)
    }

    /// Call `fmi3SetInt32` for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the underlying typed FMI call
    /// fails.
    pub fn set_int32(
        &self,
        value_references: &[Fmi3ValueReferenceRaw],
        values: &[i32],
    ) -> Result<Fmi3Status, FmiImportError> {
        self.library
            .set_int32(self.instance, value_references, values)
    }

    /// Call `fmi3GetInt32` for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the underlying typed FMI call
    /// fails.
    pub fn get_int32(
        &self,
        value_references: &[Fmi3ValueReferenceRaw],
    ) -> Result<Vec<i32>, FmiImportError> {
        self.library.get_int32(self.instance, value_references)
    }

    /// Call `fmi3SetUInt64` for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the underlying typed FMI call
    /// fails.
    pub fn set_uint64(
        &self,
        value_references: &[Fmi3ValueReferenceRaw],
        values: &[u64],
    ) -> Result<Fmi3Status, FmiImportError> {
        self.library
            .set_uint64(self.instance, value_references, values)
    }

    /// Call `fmi3GetUInt64` for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the underlying typed FMI call
    /// fails.
    pub fn get_uint64(
        &self,
        value_references: &[Fmi3ValueReferenceRaw],
    ) -> Result<Vec<u64>, FmiImportError> {
        self.library.get_uint64(self.instance, value_references)
    }

    /// Call `fmi3SetClock` for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the underlying typed FMI call
    /// fails.
    pub fn set_clock(
        &self,
        value_references: &[Fmi3ValueReferenceRaw],
        values: &[bool],
    ) -> Result<Fmi3Status, FmiImportError> {
        self.library
            .set_clock(self.instance, value_references, values)
    }

    /// Call `fmi3GetClock` for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the underlying typed FMI call
    /// fails.
    pub fn get_clock(
        &self,
        value_references: &[Fmi3ValueReferenceRaw],
    ) -> Result<Vec<bool>, FmiImportError> {
        self.library.get_clock(self.instance, value_references)
    }

    /// Call `fmi3DoStep` for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the underlying `doStep` call
    /// fails.
    pub fn do_step(
        &self,
        current_time_s: f64,
        communication_step_s: f64,
        no_set_fmu_state_prior_to_current_point: bool,
    ) -> Result<Fmi3DoStepReport, FmiImportError> {
        self.library.do_step(
            self.instance,
            current_time_s,
            communication_step_s,
            no_set_fmu_state_prior_to_current_point,
        )
    }

    /// Capture an FMU-state handle for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the optional FMI-state symbol set
    /// is absent or the FMU reports failure.
    pub fn get_fmu_state(&self) -> Result<Fmi3FmuState, FmiImportError> {
        self.library.get_fmu_state(self.instance)
    }

    /// Restore an FMU-state handle for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the optional FMI-state symbol set
    /// is absent or the FMU reports failure.
    pub fn set_fmu_state(&self, state: Fmi3FmuState) -> Result<Fmi3Status, FmiImportError> {
        self.library.set_fmu_state(self.instance, state)
    }

    /// Free an FMU-state handle for this instance.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the optional FMI-state symbol set
    /// is absent or the FMU reports failure.
    pub fn free_fmu_state(&self, state: &mut Fmi3FmuState) -> Result<Fmi3Status, FmiImportError> {
        self.library.free_fmu_state(self.instance, state)
    }
}

/// Minimal single-FMU co-simulation master over an existing FMI instance.
#[derive(Debug)]
pub struct Fmi3SingleFmuMaster<'a> {
    library: &'a Fmi3DynamicLibrary,
    instance: Fmi3Instance,
    plan: Fmi3MasterPlan,
    current_time_s: f64,
}

impl<'a> Fmi3SingleFmuMaster<'a> {
    /// Construct a master over an already-instantiated FMI component.
    ///
    /// Lifecycle ownership stays with the caller; this helper only
    /// performs typed input publication, stepping, output reads, and
    /// master-time advancement.
    #[must_use]
    pub const fn new(
        library: &'a Fmi3DynamicLibrary,
        instance: Fmi3Instance,
        plan: Fmi3MasterPlan,
    ) -> Self {
        Self {
            library,
            instance,
            plan,
            current_time_s: 0.0,
        }
    }

    /// Borrow the immutable master plan.
    #[must_use]
    pub const fn plan(&self) -> &Fmi3MasterPlan {
        &self.plan
    }

    /// Current master time at the beginning of the next communication step.
    #[must_use]
    pub const fn current_time_s(&self) -> f64 {
        self.current_time_s
    }

    /// Publish deterministic periodic input clocks at a communication point.
    ///
    /// Constant, fixed, and tunable input clocks with finite positive
    /// `intervalDecimal` are scheduled at macro-step boundaries using
    /// optional `shiftDecimal`. Triggered, changing, countdown, output,
    /// and local clocks are ignored by this scoped scheduler.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the communication point is not
    /// finite, scheduled clock metadata is invalid, or `fmi3SetClock`
    /// fails.
    pub fn publish_periodic_input_clocks_at(
        &self,
        current_time_s: f64,
    ) -> Result<Fmi3ClockScheduleReport, FmiImportError> {
        if !current_time_s.is_finite() {
            return Err(invalid_clock_schedule(
                "communication point",
                format!("time is not finite: {current_time_s}"),
            ));
        }

        let mut value_references = Vec::new();
        let mut values = Vec::new();
        let mut activations = Vec::new();
        for clock in &self.plan.variables.clocks {
            let Some(active) = periodic_input_clock_activation(clock, current_time_s)? else {
                continue;
            };
            value_references.push(clock.value_reference);
            values.push(active);
            activations.push(Fmi3ClockScheduleActivation {
                name: clock.name.clone(),
                value_reference: clock.value_reference,
                active,
            });
        }

        if !value_references.is_empty() {
            self.library
                .set_clock(self.instance, &value_references, &values)?;
        }

        Ok(Fmi3ClockScheduleReport { activations })
    }

    /// Perform one typed single-FMU communication step.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when input vector lengths do not
    /// match the binding plan or any underlying typed FMI call fails.
    pub fn step(&mut self, inputs: &Fmi3InputSample) -> Result<Fmi3StepResult, FmiImportError> {
        ensure_sample_count(
            "Float64 input",
            self.plan.variables.float64_inputs.len(),
            inputs.float64.len(),
        )?;
        ensure_sample_count(
            "UInt64 input",
            self.plan.variables.uint64_inputs.len(),
            inputs.uint64.len(),
        )?;
        ensure_sample_count(
            "Int32 input",
            self.plan.variables.int32_inputs.len(),
            inputs.int32.len(),
        )?;

        self.publish_periodic_input_clocks_at(self.current_time_s)?;

        let float64_input_refs = raw_value_references(&self.plan.variables.float64_inputs);
        let int32_input_refs = raw_value_references(&self.plan.variables.int32_inputs);
        let uint64_input_refs = raw_value_references(&self.plan.variables.uint64_inputs);
        let float64_output_refs = raw_value_references(&self.plan.variables.float64_outputs);
        let int32_output_refs = raw_value_references(&self.plan.variables.int32_outputs);
        let uint64_output_refs = raw_value_references(&self.plan.variables.uint64_outputs);

        if !float64_input_refs.is_empty() {
            self.library
                .set_float64(self.instance, &float64_input_refs, &inputs.float64)?;
        }
        if !int32_input_refs.is_empty() {
            self.library
                .set_int32(self.instance, &int32_input_refs, &inputs.int32)?;
        }
        if !uint64_input_refs.is_empty() {
            self.library
                .set_uint64(self.instance, &uint64_input_refs, &inputs.uint64)?;
        }

        let report = self.library.do_step(
            self.instance,
            self.current_time_s,
            self.plan.communication_step_s,
            true,
        )?;
        self.current_time_s = match report.outcome {
            Fmi3DoStepOutcome::Complete => self.current_time_s + self.plan.communication_step_s,
            Fmi3DoStepOutcome::EarlyReturn {
                last_successful_time_s,
            } => last_successful_time_s,
        };

        let outputs = Fmi3OutputSample {
            float64: if float64_output_refs.is_empty() {
                Vec::new()
            } else {
                self.library
                    .get_float64(self.instance, &float64_output_refs)?
            },
            int32: if int32_output_refs.is_empty() {
                Vec::new()
            } else {
                self.library.get_int32(self.instance, &int32_output_refs)?
            },
            uint64: if uint64_output_refs.is_empty() {
                Vec::new()
            } else {
                self.library
                    .get_uint64(self.instance, &uint64_output_refs)?
            },
        };

        Ok(Fmi3StepResult {
            outputs,
            do_step: report,
            current_time_s: self.current_time_s,
        })
    }

    /// Perform one typed step with an FMU-state checkpoint.
    ///
    /// The checkpoint is captured before input publication. When the
    /// policy restores, the FMU and master time are returned to their
    /// pre-step state while the attempted step result is still reported
    /// for scheduling decisions.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when FMU-state snapshot/restore/free is
    /// unavailable or fails, when input vector lengths do not match the
    /// binding plan, or when any underlying typed FMI call fails.
    pub fn step_with_rollback_policy(
        &mut self,
        inputs: &Fmi3InputSample,
        policy: Fmi3RollbackPolicy,
    ) -> Result<Fmi3RollbackStepResult, FmiImportError> {
        let start_time_s = self.current_time_s;
        let mut state = self.library.get_fmu_state(self.instance)?;
        let step = match self.step(inputs) {
            Ok(step) => step,
            Err(error) => {
                let _ = self.library.free_fmu_state(self.instance, &mut state);
                return Err(error);
            }
        };

        let action = if policy.should_restore(&step.do_step) {
            let attempted_time_s = step.current_time_s;
            if let Err(error) = self.library.set_fmu_state(self.instance, state) {
                let _ = self.library.free_fmu_state(self.instance, &mut state);
                return Err(error);
            }
            self.current_time_s = start_time_s;
            Fmi3RollbackAction::Restored {
                restored_time_s: start_time_s,
                attempted_time_s,
            }
        } else {
            Fmi3RollbackAction::Preserved
        };

        self.library.free_fmu_state(self.instance, &mut state)?;
        Ok(Fmi3RollbackStepResult {
            step,
            action,
            current_time_s: self.current_time_s,
        })
    }
}

/// One node controlled by a multi-FMU co-simulation master.
#[derive(Debug)]
pub struct Fmi3FmuNode<'a> {
    /// Dynamic library loaded for this FMU.
    pub library: &'a Fmi3DynamicLibrary,
    /// FMI instance pointer for this node.
    pub instance: Fmi3Instance,
    /// Typed master plan for this node.
    pub plan: Fmi3MasterPlan,
}

impl<'a> Fmi3FmuNode<'a> {
    /// Build one multi-FMU node from an already-instantiated FMI component.
    #[must_use]
    pub const fn new(
        library: &'a Fmi3DynamicLibrary,
        instance: Fmi3Instance,
        plan: Fmi3MasterPlan,
    ) -> Self {
        Self {
            library,
            instance,
            plan,
        }
    }
}

/// Typed Float64 connection between two FMU nodes.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Fmi3Float64Connection {
    /// Source FMU node index.
    pub source_node: usize,
    /// Source Float64 output index in the source node plan.
    pub source_output: usize,
    /// Target FMU node index.
    pub target_node: usize,
    /// Target Float64 input index in the target node plan.
    pub target_input: usize,
}

/// Typed UInt64 connection between two FMU nodes.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Fmi3UInt64Connection {
    /// Source FMU node index.
    pub source_node: usize,
    /// Source UInt64 output index in the source node plan.
    pub source_output: usize,
    /// Target FMU node index.
    pub target_node: usize,
    /// Target UInt64 input index in the target node plan.
    pub target_input: usize,
}

/// Result from one multi-FMU macro step.
#[derive(Clone, Debug, PartialEq)]
pub struct Fmi3MultiFmuStepResult {
    /// Per-node step reports in node order.
    pub node_results: Vec<Fmi3StepResult>,
    /// Per-node output samples in node order.
    pub outputs: Vec<Fmi3OutputSample>,
    /// Per-node master times after the step.
    pub current_times_s: Vec<f64>,
}

/// Result from one checkpointed predictor/corrector multi-FMU macro step.
#[derive(Clone, Debug, PartialEq)]
pub struct Fmi3PredictorCorrectorStepResult {
    /// First pass from the previous-output cache.
    pub predictor: Fmi3MultiFmuStepResult,
    /// Second pass from the same macro-step start, seeded by predictor outputs.
    pub corrector: Fmi3MultiFmuStepResult,
}

/// Minimal multi-FMU co-simulation master with typed connection routing.
///
/// Jacobi routing applies connections from the previous output cache for all
/// nodes. Gauss-Seidel routing uses outputs already produced in the current
/// macro step, falling back to the previous output cache for sources that have
/// not stepped yet.
#[derive(Debug)]
pub struct Fmi3MultiFmuMaster<'a> {
    coupling_order: Fmi3CouplingOrder,
    masters: Vec<Fmi3SingleFmuMaster<'a>>,
    float64_connections: Vec<Fmi3Float64Connection>,
    uint64_connections: Vec<Fmi3UInt64Connection>,
    last_outputs: Vec<Fmi3OutputSample>,
}

impl<'a> Fmi3MultiFmuMaster<'a> {
    /// Build a multi-FMU master with zero-initialized previous outputs.
    #[must_use]
    pub fn new(nodes: Vec<Fmi3FmuNode<'a>>, coupling_order: Fmi3CouplingOrder) -> Self {
        let masters = nodes
            .into_iter()
            .map(|node| Fmi3SingleFmuMaster::new(node.library, node.instance, node.plan))
            .collect::<Vec<_>>();
        let last_outputs = masters
            .iter()
            .map(|master| default_output_sample(master.plan()))
            .collect();
        Self {
            coupling_order,
            masters,
            float64_connections: Vec::new(),
            uint64_connections: Vec::new(),
            last_outputs,
        }
    }

    /// Build a multi-FMU master with caller-supplied previous outputs.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the initial output vector length or
    /// any per-node output sample length does not match the node plans.
    pub fn with_initial_outputs(
        nodes: Vec<Fmi3FmuNode<'a>>,
        coupling_order: Fmi3CouplingOrder,
        initial_outputs: Vec<Fmi3OutputSample>,
    ) -> Result<Self, FmiImportError> {
        let mut master = Self::new(nodes, coupling_order);
        master.set_initial_outputs(initial_outputs)?;
        Ok(master)
    }

    /// Seed the previous-output cache used by the next connected step.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the output vector length or any
    /// per-node output sample length does not match the node plans.
    pub fn set_initial_outputs(
        &mut self,
        initial_outputs: Vec<Fmi3OutputSample>,
    ) -> Result<(), FmiImportError> {
        self.validate_output_samples(&initial_outputs, "initial")?;
        self.last_outputs = initial_outputs;
        Ok(())
    }

    /// Number of FMU nodes in this master.
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.masters.len()
    }

    /// Coupling order used for connection routing.
    #[must_use]
    pub const fn coupling_order(&self) -> Fmi3CouplingOrder {
        self.coupling_order
    }

    /// Previous-output cache used by the next Jacobi or lagged connection.
    #[must_use]
    pub fn last_outputs(&self) -> &[Fmi3OutputSample] {
        &self.last_outputs
    }

    /// Add a typed Float64 connection.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when any node or typed variable index is
    /// outside the configured node plans.
    pub fn add_float64_connection(
        &mut self,
        connection: Fmi3Float64Connection,
    ) -> Result<(), FmiImportError> {
        self.validate_float64_connection(connection)?;
        self.float64_connections.push(connection);
        Ok(())
    }

    /// Add a typed UInt64 connection.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when any node or typed variable index is
    /// outside the configured node plans.
    pub fn add_uint64_connection(
        &mut self,
        connection: Fmi3UInt64Connection,
    ) -> Result<(), FmiImportError> {
        self.validate_uint64_connection(connection)?;
        self.uint64_connections.push(connection);
        Ok(())
    }

    /// Perform one connected multi-FMU communication step.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the input sample vector does not
    /// match the node count, any routed sample does not match its node
    /// plan, a connection is invalid, or an underlying single-FMU step
    /// fails.
    pub fn step(
        &mut self,
        inputs: &[Fmi3InputSample],
    ) -> Result<Fmi3MultiFmuStepResult, FmiImportError> {
        ensure_sample_count("FMU node input sample", self.masters.len(), inputs.len())?;

        let previous_outputs = self.last_outputs.clone();
        let mut routed_inputs = inputs.to_vec();
        let mut node_results = Vec::with_capacity(self.masters.len());
        let mut outputs = Vec::with_capacity(self.masters.len());

        match self.coupling_order {
            Fmi3CouplingOrder::Jacobi => {
                for node_index in 0..self.masters.len() {
                    self.apply_connections_from_outputs(
                        node_index,
                        &mut routed_inputs,
                        |source_node| previous_outputs.get(source_node),
                    )?;
                    let result = self.masters[node_index].step(&routed_inputs[node_index])?;
                    outputs.push(result.outputs.clone());
                    node_results.push(result);
                }
            }
            Fmi3CouplingOrder::GaussSeidel => {
                let mut current_outputs = vec![None; self.masters.len()];
                for node_index in 0..self.masters.len() {
                    self.apply_connections_from_outputs(
                        node_index,
                        &mut routed_inputs,
                        |source_node| {
                            current_outputs
                                .get(source_node)
                                .and_then(Option::as_ref)
                                .or_else(|| previous_outputs.get(source_node))
                        },
                    )?;
                    let result = self.masters[node_index].step(&routed_inputs[node_index])?;
                    current_outputs[node_index] = Some(result.outputs.clone());
                    outputs.push(result.outputs.clone());
                    node_results.push(result);
                }
            }
        }

        let current_times_s = node_results
            .iter()
            .map(|result| result.current_time_s)
            .collect();
        self.last_outputs = outputs.clone();

        Ok(Fmi3MultiFmuStepResult {
            node_results,
            outputs,
            current_times_s,
        })
    }

    /// Perform one checkpointed predictor/corrector multi-FMU macro step.
    ///
    /// The predictor pass runs from the previous-output cache. The master
    /// then restores every FMU and master clock to the macro-step start,
    /// seeds the connection cache from predictor outputs, and runs the
    /// corrector pass. The final master state is the corrector state.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the input sample vector is invalid,
    /// when any FMU-state snapshot/restore/free operation fails, or when
    /// either underlying multi-FMU step fails.
    pub fn step_predictor_corrector(
        &mut self,
        inputs: &[Fmi3InputSample],
    ) -> Result<Fmi3PredictorCorrectorStepResult, FmiImportError> {
        ensure_sample_count("FMU node input sample", self.masters.len(), inputs.len())?;

        let start_times_s = self
            .masters
            .iter()
            .map(Fmi3SingleFmuMaster::current_time_s)
            .collect::<Vec<_>>();
        let start_outputs = self.last_outputs.clone();
        let mut start_states = self.capture_fmu_states()?;

        let predictor = match self.step(inputs) {
            Ok(result) => result,
            Err(error) => {
                let _ = self.restore_fmu_states(&start_states, &start_times_s);
                let _ = self.free_fmu_states(&mut start_states);
                self.last_outputs = start_outputs;
                return Err(error);
            }
        };

        if let Err(error) = self.restore_fmu_states(&start_states, &start_times_s) {
            let _ = self.free_fmu_states(&mut start_states);
            self.last_outputs = start_outputs;
            return Err(error);
        }
        self.free_fmu_states(&mut start_states)?;

        self.last_outputs = predictor.outputs.clone();
        let corrector = self.step(inputs)?;

        Ok(Fmi3PredictorCorrectorStepResult {
            predictor,
            corrector,
        })
    }

    fn validate_output_samples(
        &self,
        outputs: &[Fmi3OutputSample],
        field_prefix: &'static str,
    ) -> Result<(), FmiImportError> {
        ensure_sample_count("FMU node output sample", self.masters.len(), outputs.len())?;
        for (master, output) in self.masters.iter().zip(outputs) {
            ensure_sample_count(
                match field_prefix {
                    "initial" => "Float64 initial output",
                    _ => "Float64 output",
                },
                master.plan().variables.float64_outputs.len(),
                output.float64.len(),
            )?;
            ensure_sample_count(
                match field_prefix {
                    "initial" => "UInt64 initial output",
                    _ => "UInt64 output",
                },
                master.plan().variables.uint64_outputs.len(),
                output.uint64.len(),
            )?;
            ensure_sample_count(
                match field_prefix {
                    "initial" => "Int32 initial output",
                    _ => "Int32 output",
                },
                master.plan().variables.int32_outputs.len(),
                output.int32.len(),
            )?;
        }
        Ok(())
    }

    fn validate_float64_connection(
        &self,
        connection: Fmi3Float64Connection,
    ) -> Result<(), FmiImportError> {
        let source = self.node_plan(connection.source_node, "Float64 source")?;
        let target = self.node_plan(connection.target_node, "Float64 target")?;
        if connection.source_output >= source.variables.float64_outputs.len() {
            return Err(invalid_connection(format!(
                "source node {} Float64 output index {} is outside {} outputs",
                connection.source_node,
                connection.source_output,
                source.variables.float64_outputs.len()
            )));
        }
        if connection.target_input >= target.variables.float64_inputs.len() {
            return Err(invalid_connection(format!(
                "target node {} Float64 input index {} is outside {} inputs",
                connection.target_node,
                connection.target_input,
                target.variables.float64_inputs.len()
            )));
        }
        Ok(())
    }

    fn validate_uint64_connection(
        &self,
        connection: Fmi3UInt64Connection,
    ) -> Result<(), FmiImportError> {
        let source = self.node_plan(connection.source_node, "UInt64 source")?;
        let target = self.node_plan(connection.target_node, "UInt64 target")?;
        if connection.source_output >= source.variables.uint64_outputs.len() {
            return Err(invalid_connection(format!(
                "source node {} UInt64 output index {} is outside {} outputs",
                connection.source_node,
                connection.source_output,
                source.variables.uint64_outputs.len()
            )));
        }
        if connection.target_input >= target.variables.uint64_inputs.len() {
            return Err(invalid_connection(format!(
                "target node {} UInt64 input index {} is outside {} inputs",
                connection.target_node,
                connection.target_input,
                target.variables.uint64_inputs.len()
            )));
        }
        Ok(())
    }

    fn node_plan(
        &self,
        node_index: usize,
        role: &'static str,
    ) -> Result<&Fmi3MasterPlan, FmiImportError> {
        self.masters
            .get(node_index)
            .map(Fmi3SingleFmuMaster::plan)
            .ok_or_else(|| {
                invalid_connection(format!(
                    "{role} node index {node_index} is outside {} nodes",
                    self.masters.len()
                ))
            })
    }

    fn apply_connections_from_outputs<'b>(
        &self,
        target_node: usize,
        routed_inputs: &mut [Fmi3InputSample],
        mut output_for_source: impl FnMut(usize) -> Option<&'b Fmi3OutputSample>,
    ) -> Result<(), FmiImportError> {
        for connection in self
            .float64_connections
            .iter()
            .filter(|connection| connection.target_node == target_node)
        {
            self.validate_float64_connection(*connection)?;
            let source_output = output_for_source(connection.source_node)
                .and_then(|sample| sample.float64.get(connection.source_output))
                .copied()
                .ok_or_else(|| {
                    invalid_connection(format!(
                        "source node {} Float64 output index {} is not available",
                        connection.source_node, connection.source_output
                    ))
                })?;
            let expected = self.masters[target_node]
                .plan()
                .variables
                .float64_inputs
                .len();
            let target_sample = &mut routed_inputs[target_node];
            ensure_sample_count("Float64 input", expected, target_sample.float64.len())?;
            target_sample.float64[connection.target_input] = source_output;
        }

        for connection in self
            .uint64_connections
            .iter()
            .filter(|connection| connection.target_node == target_node)
        {
            self.validate_uint64_connection(*connection)?;
            let source_output = output_for_source(connection.source_node)
                .and_then(|sample| sample.uint64.get(connection.source_output))
                .copied()
                .ok_or_else(|| {
                    invalid_connection(format!(
                        "source node {} UInt64 output index {} is not available",
                        connection.source_node, connection.source_output
                    ))
                })?;
            let expected = self.masters[target_node]
                .plan()
                .variables
                .uint64_inputs
                .len();
            let target_sample = &mut routed_inputs[target_node];
            ensure_sample_count("UInt64 input", expected, target_sample.uint64.len())?;
            target_sample.uint64[connection.target_input] = source_output;
        }

        Ok(())
    }

    fn capture_fmu_states(&self) -> Result<Vec<Fmi3FmuState>, FmiImportError> {
        let mut states = Vec::with_capacity(self.masters.len());
        for master in &self.masters {
            match master.library.get_fmu_state(master.instance) {
                Ok(state) => states.push(state),
                Err(error) => {
                    for (master, state) in self.masters.iter().zip(&mut states) {
                        let _ = master.library.free_fmu_state(master.instance, state);
                    }
                    return Err(error);
                }
            }
        }
        Ok(states)
    }

    fn restore_fmu_states(
        &mut self,
        states: &[Fmi3FmuState],
        times_s: &[f64],
    ) -> Result<(), FmiImportError> {
        ensure_sample_count("FMU-state snapshot", self.masters.len(), states.len())?;
        ensure_sample_count(
            "FMU-state clock snapshot",
            self.masters.len(),
            times_s.len(),
        )?;
        for ((master, state), time_s) in self.masters.iter_mut().zip(states).zip(times_s) {
            master.library.set_fmu_state(master.instance, *state)?;
            master.current_time_s = *time_s;
        }
        Ok(())
    }

    fn free_fmu_states(&self, states: &mut [Fmi3FmuState]) -> Result<(), FmiImportError> {
        ensure_sample_count("FMU-state snapshot", self.masters.len(), states.len())?;
        for (master, state) in self.masters.iter().zip(states) {
            master.library.free_fmu_state(master.instance, state)?;
        }
        Ok(())
    }
}

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
    /// Filesystem IO failed while materializing an archive entry.
    #[error("could not materialize FMU archive entry {path}: {source}")]
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
    /// `fmi3InstantiateCoSimulation` returned null.
    #[error("FMI dynamic library {path} returned null from fmi3InstantiateCoSimulation")]
    NullInstance {
        /// Library path.
        path: PathBuf,
    },
    /// `fmi3GetFMUState` returned a null state handle.
    #[error("FMI dynamic library {path} returned null from fmi3GetFMUState")]
    NullFmuState {
        /// Library path.
        path: PathBuf,
    },
    /// A string passed through the FMI C API contained an interior NUL byte.
    #[error("FMI string argument {field} contains an interior NUL byte")]
    InvalidStringArgument {
        /// FMI argument field name.
        field: &'static str,
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
    /// A typed FMI call received mismatched reference/value array lengths.
    #[error("FMI call {function} expected {expected} values but received {got}")]
    ValueCountMismatch {
        /// FMI function name.
        function: &'static str,
        /// Number of value references supplied.
        expected: usize,
        /// Number of values supplied.
        got: usize,
    },
    /// A master input sample did not match the binding plan.
    #[error("FMI master expected {expected} {field} values but received {got}")]
    SampleCountMismatch {
        /// Sample field.
        field: &'static str,
        /// Number of variables in the binding plan.
        expected: usize,
        /// Number of values supplied.
        got: usize,
    },
    /// A multi-FMU typed connection references an invalid node or variable.
    #[error("FMI multi-FMU connection is invalid: {reason}")]
    InvalidConnection {
        /// Human-readable validation reason.
        reason: String,
    },
    /// FMI Clock metadata cannot be scheduled by the scoped master.
    #[error("FMI clock schedule for {clock} is invalid: {reason}")]
    InvalidClockSchedule {
        /// Clock name or scheduler field.
        clock: String,
        /// Human-readable validation reason.
        reason: String,
    },
    /// A typed FMI call returned an unknown status code.
    #[error("FMI call {function} returned unknown status {status}")]
    UnknownStatus {
        /// FMI function name.
        function: &'static str,
        /// Raw FMI status code.
        status: c_int,
    },
    /// A typed FMI call returned a non-continuable status.
    #[error("FMI call {function} returned status {status:?}")]
    CallFailed {
        /// FMI function name.
        function: &'static str,
        /// Returned FMI status.
        status: Fmi3Status,
    },
    /// The requested co-simulation communication step is invalid.
    #[error("FMI master plan requires a positive finite communication step, got {step_s}")]
    InvalidCommunicationStep {
        /// Invalid communication step in seconds.
        step_s: f64,
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

    /// Instantiate an owned FMI 3 co-simulation component.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when a string argument cannot be
    /// passed through the C API, the instantiation symbol is absent, or
    /// the FMU returns a null instance pointer.
    pub fn instantiate_co_simulation(
        &self,
        options: Fmi3CoSimulationInstantiation<'_>,
    ) -> Result<Fmi3CoSimulationInstance<'_>, FmiImportError> {
        type Fmi3InstantiateCoSimulation = unsafe extern "C" fn(
            *const c_char,
            *const c_char,
            *const c_char,
            c_int,
            c_int,
            c_int,
            c_int,
            *const Fmi3ValueReferenceRaw,
            usize,
            *mut c_void,
            *mut c_void,
            *mut c_void,
        ) -> Fmi3Instance;

        let instance_name = cstring_argument("instance_name", options.instance_name)?;
        let instantiation_token =
            cstring_argument("instantiation_token", options.instantiation_token)?;
        let resource_path = cstring_argument("resource_path", options.resource_path)?;
        let symbol = self.symbol::<Fmi3InstantiateCoSimulation>("fmi3InstantiateCoSimulation")?;
        let instance = unsafe {
            // The options C strings live for the duration of the call.
            // OpenBMP does not request callbacks or intermediate
            // variables in this importer smoke layer.
            symbol(
                instance_name.as_ptr(),
                instantiation_token.as_ptr(),
                resource_path.as_ptr(),
                fmi_boolean(options.visible),
                fmi_boolean(options.logging_on),
                fmi_boolean(options.event_mode_used),
                fmi_boolean(options.early_return_allowed),
                std::ptr::null(),
                0,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
        };
        if instance.is_null() {
            return Err(FmiImportError::NullInstance {
                path: self.path.clone(),
            });
        }
        Ok(Fmi3CoSimulationInstance {
            library: self,
            instance,
            terminated: false,
        })
    }

    /// Call `fmi3EnterInitializationMode`.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn enter_initialization_mode(
        &self,
        instance: Fmi3Instance,
        start_time_s: f64,
        stop_time_s: Option<f64>,
        tolerance: Option<f64>,
    ) -> Result<Fmi3Status, FmiImportError> {
        type Fmi3EnterInitializationMode =
            unsafe extern "C" fn(Fmi3Instance, c_int, f64, f64, c_int, f64) -> c_int;
        let symbol = self.symbol::<Fmi3EnterInitializationMode>("fmi3EnterInitializationMode")?;
        let (tolerance_defined, tolerance) = tolerance.map_or((false, 0.0), |value| (true, value));
        let (stop_time_defined, stop_time_s) =
            stop_time_s.map_or((false, 0.0), |value| (true, value));
        let raw_status = unsafe {
            // The signature mirrors the FMI 3 C API used by the
            // restricted exporter ABI.
            symbol(
                instance,
                fmi_boolean(tolerance_defined),
                tolerance,
                start_time_s,
                fmi_boolean(stop_time_defined),
                stop_time_s,
            )
        };
        continuable_status("fmi3EnterInitializationMode", raw_status)
    }

    /// Call `fmi3ExitInitializationMode`.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn exit_initialization_mode(
        &self,
        instance: Fmi3Instance,
    ) -> Result<Fmi3Status, FmiImportError> {
        type Fmi3ExitInitializationMode = unsafe extern "C" fn(Fmi3Instance) -> c_int;
        let symbol = self.symbol::<Fmi3ExitInitializationMode>("fmi3ExitInitializationMode")?;
        let raw_status = unsafe { symbol(instance) };
        continuable_status("fmi3ExitInitializationMode", raw_status)
    }

    /// Call `fmi3Terminate`.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn terminate(&self, instance: Fmi3Instance) -> Result<Fmi3Status, FmiImportError> {
        type Fmi3Terminate = unsafe extern "C" fn(Fmi3Instance) -> c_int;
        let symbol = self.symbol::<Fmi3Terminate>("fmi3Terminate")?;
        let raw_status = unsafe { symbol(instance) };
        continuable_status("fmi3Terminate", raw_status)
    }

    /// Call `fmi3FreeInstance`.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent.
    pub fn free_instance(&self, instance: Fmi3Instance) -> Result<(), FmiImportError> {
        type Fmi3FreeInstance = unsafe extern "C" fn(Fmi3Instance);
        let symbol = self.symbol::<Fmi3FreeInstance>("fmi3FreeInstance")?;
        unsafe {
            // FMI owns the instance allocation. This call releases that
            // opaque allocation and returns no status code.
            symbol(instance);
        }
        Ok(())
    }

    /// Call `fmi3SetFloat64` with typed value references and values.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent, the
    /// reference/value lengths differ, or the FMU returns a
    /// non-continuable status.
    pub fn set_float64(
        &self,
        instance: Fmi3Instance,
        value_references: &[Fmi3ValueReferenceRaw],
        values: &[f64],
    ) -> Result<Fmi3Status, FmiImportError> {
        type Fmi3SetFloat64 = unsafe extern "C" fn(
            Fmi3Instance,
            *const Fmi3ValueReferenceRaw,
            usize,
            *const f64,
            usize,
        ) -> c_int;
        ensure_matching_value_count("fmi3SetFloat64", value_references.len(), values.len())?;
        let symbol = self.symbol::<Fmi3SetFloat64>("fmi3SetFloat64")?;
        let raw_status = unsafe {
            // The symbol signature mirrors the FMI 3 C API. Slices
            // provide stable pointers and explicit element counts for
            // the duration of this call.
            symbol(
                instance,
                value_references.as_ptr(),
                value_references.len(),
                values.as_ptr(),
                values.len(),
            )
        };
        continuable_status("fmi3SetFloat64", raw_status)
    }

    /// Call `fmi3GetFloat64` and return values in reference order.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn get_float64(
        &self,
        instance: Fmi3Instance,
        value_references: &[Fmi3ValueReferenceRaw],
    ) -> Result<Vec<f64>, FmiImportError> {
        type Fmi3GetFloat64 = unsafe extern "C" fn(
            Fmi3Instance,
            *const Fmi3ValueReferenceRaw,
            usize,
            *mut f64,
            usize,
        ) -> c_int;
        let symbol = self.symbol::<Fmi3GetFloat64>("fmi3GetFloat64")?;
        let mut values = vec![0.0; value_references.len()];
        let raw_status = unsafe {
            // The FMU writes exactly the number of values advertised by
            // `values.len()`; the Vec remains alive for the full call.
            symbol(
                instance,
                value_references.as_ptr(),
                value_references.len(),
                values.as_mut_ptr(),
                values.len(),
            )
        };
        continuable_status("fmi3GetFloat64", raw_status)?;
        Ok(values)
    }

    /// Call `fmi3SetInt32` with typed value references and values.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent, the
    /// reference/value lengths differ, or the FMU returns a
    /// non-continuable status.
    pub fn set_int32(
        &self,
        instance: Fmi3Instance,
        value_references: &[Fmi3ValueReferenceRaw],
        values: &[i32],
    ) -> Result<Fmi3Status, FmiImportError> {
        type Fmi3SetInt32 = unsafe extern "C" fn(
            Fmi3Instance,
            *const Fmi3ValueReferenceRaw,
            usize,
            *const i32,
            usize,
        ) -> c_int;
        ensure_matching_value_count("fmi3SetInt32", value_references.len(), values.len())?;
        let symbol = self.symbol::<Fmi3SetInt32>("fmi3SetInt32")?;
        let raw_status = unsafe {
            // The symbol signature mirrors the FMI 3 C API. Slices
            // provide stable pointers and explicit element counts for
            // the duration of this call.
            symbol(
                instance,
                value_references.as_ptr(),
                value_references.len(),
                values.as_ptr(),
                values.len(),
            )
        };
        continuable_status("fmi3SetInt32", raw_status)
    }

    /// Call `fmi3GetInt32` and return values in reference order.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn get_int32(
        &self,
        instance: Fmi3Instance,
        value_references: &[Fmi3ValueReferenceRaw],
    ) -> Result<Vec<i32>, FmiImportError> {
        type Fmi3GetInt32 = unsafe extern "C" fn(
            Fmi3Instance,
            *const Fmi3ValueReferenceRaw,
            usize,
            *mut i32,
            usize,
        ) -> c_int;
        let symbol = self.symbol::<Fmi3GetInt32>("fmi3GetInt32")?;
        let mut values = vec![0; value_references.len()];
        let raw_status = unsafe {
            // The FMU writes exactly the number of values advertised by
            // `values.len()`; the Vec remains alive for the full call.
            symbol(
                instance,
                value_references.as_ptr(),
                value_references.len(),
                values.as_mut_ptr(),
                values.len(),
            )
        };
        continuable_status("fmi3GetInt32", raw_status)?;
        Ok(values)
    }

    /// Call `fmi3SetUInt64` with typed value references and values.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent, the
    /// reference/value lengths differ, or the FMU returns a
    /// non-continuable status.
    pub fn set_uint64(
        &self,
        instance: Fmi3Instance,
        value_references: &[Fmi3ValueReferenceRaw],
        values: &[u64],
    ) -> Result<Fmi3Status, FmiImportError> {
        type Fmi3SetUInt64 = unsafe extern "C" fn(
            Fmi3Instance,
            *const Fmi3ValueReferenceRaw,
            usize,
            *const u64,
            usize,
        ) -> c_int;
        ensure_matching_value_count("fmi3SetUInt64", value_references.len(), values.len())?;
        let symbol = self.symbol::<Fmi3SetUInt64>("fmi3SetUInt64")?;
        let raw_status = unsafe {
            // The symbol signature mirrors the FMI 3 C API. Slices
            // provide stable pointers and explicit element counts for
            // the duration of this call.
            symbol(
                instance,
                value_references.as_ptr(),
                value_references.len(),
                values.as_ptr(),
                values.len(),
            )
        };
        continuable_status("fmi3SetUInt64", raw_status)
    }

    /// Call `fmi3GetUInt64` and return values in reference order.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn get_uint64(
        &self,
        instance: Fmi3Instance,
        value_references: &[Fmi3ValueReferenceRaw],
    ) -> Result<Vec<u64>, FmiImportError> {
        type Fmi3GetUInt64 = unsafe extern "C" fn(
            Fmi3Instance,
            *const Fmi3ValueReferenceRaw,
            usize,
            *mut u64,
            usize,
        ) -> c_int;
        let symbol = self.symbol::<Fmi3GetUInt64>("fmi3GetUInt64")?;
        let mut values = vec![0; value_references.len()];
        let raw_status = unsafe {
            // The FMU writes exactly the number of values advertised by
            // `values.len()`; the Vec remains alive for the full call.
            symbol(
                instance,
                value_references.as_ptr(),
                value_references.len(),
                values.as_mut_ptr(),
                values.len(),
            )
        };
        continuable_status("fmi3GetUInt64", raw_status)?;
        Ok(values)
    }

    /// Call `fmi3SetClock` with typed value references and boolean values.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent, the
    /// reference/value lengths differ, or the FMU returns a
    /// non-continuable status.
    pub fn set_clock(
        &self,
        instance: Fmi3Instance,
        value_references: &[Fmi3ValueReferenceRaw],
        values: &[bool],
    ) -> Result<Fmi3Status, FmiImportError> {
        type Fmi3SetClock = unsafe extern "C" fn(
            Fmi3Instance,
            *const Fmi3ValueReferenceRaw,
            usize,
            *const c_int,
            usize,
        ) -> c_int;
        ensure_matching_value_count("fmi3SetClock", value_references.len(), values.len())?;
        let symbol = self.symbol::<Fmi3SetClock>("fmi3SetClock")?;
        let clock_values = values.iter().copied().map(fmi_boolean).collect::<Vec<_>>();
        let raw_status = unsafe {
            // Rust exposes clocks as bools. The C boundary receives FMI
            // boolean integers with explicit counts.
            symbol(
                instance,
                value_references.as_ptr(),
                value_references.len(),
                clock_values.as_ptr(),
                clock_values.len(),
            )
        };
        continuable_status("fmi3SetClock", raw_status)
    }

    /// Call `fmi3GetClock` and return booleans in reference order.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn get_clock(
        &self,
        instance: Fmi3Instance,
        value_references: &[Fmi3ValueReferenceRaw],
    ) -> Result<Vec<bool>, FmiImportError> {
        type Fmi3GetClock = unsafe extern "C" fn(
            Fmi3Instance,
            *const Fmi3ValueReferenceRaw,
            usize,
            *mut c_int,
            usize,
        ) -> c_int;
        let symbol = self.symbol::<Fmi3GetClock>("fmi3GetClock")?;
        let mut values = vec![0; value_references.len()];
        let raw_status = unsafe {
            // The FMU writes exactly the number of clock values
            // advertised by `values.len()`.
            symbol(
                instance,
                value_references.as_ptr(),
                value_references.len(),
                values.as_mut_ptr(),
                values.len(),
            )
        };
        continuable_status("fmi3GetClock", raw_status)?;
        Ok(values.into_iter().map(fmi_boolean_to_bool).collect())
    }

    /// Call `fmi3DoStep` and report early-return state.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the symbol is absent or the FMU
    /// returns a non-continuable status.
    pub fn do_step(
        &self,
        instance: Fmi3Instance,
        current_time_s: f64,
        communication_step_s: f64,
        no_set_fmu_state_prior_to_current_point: bool,
    ) -> Result<Fmi3DoStepReport, FmiImportError> {
        type Fmi3DoStep = unsafe extern "C" fn(
            Fmi3Instance,
            f64,
            f64,
            c_int,
            *mut c_int,
            *mut c_int,
            *mut c_int,
            *mut f64,
        ) -> c_int;
        let symbol = self.symbol::<Fmi3DoStep>("fmi3DoStep")?;
        let mut event_handling_needed = 0;
        let mut terminate_simulation = 0;
        let mut early_return = 0;
        let mut last_successful_time_s = current_time_s + communication_step_s;
        let raw_status = unsafe {
            // `fmi3DoStep` writes only through the out-pointers passed
            // below. They point to stack locals that live until the call
            // returns.
            symbol(
                instance,
                current_time_s,
                communication_step_s,
                fmi_boolean(no_set_fmu_state_prior_to_current_point),
                &mut event_handling_needed,
                &mut terminate_simulation,
                &mut early_return,
                &mut last_successful_time_s,
            )
        };
        let status = continuable_status("fmi3DoStep", raw_status)?;
        let outcome = if fmi_boolean_to_bool(early_return) {
            Fmi3DoStepOutcome::EarlyReturn {
                last_successful_time_s,
            }
        } else {
            Fmi3DoStepOutcome::Complete
        };
        Ok(Fmi3DoStepReport {
            status,
            event_handling_needed: fmi_boolean_to_bool(event_handling_needed),
            terminate_simulation: fmi_boolean_to_bool(terminate_simulation),
            outcome,
        })
    }

    /// Call `fmi3GetFMUState` and return an opaque state handle.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the optional symbol is absent,
    /// the FMU returns a non-continuable status, or the returned state
    /// handle is null.
    pub fn get_fmu_state(&self, instance: Fmi3Instance) -> Result<Fmi3FmuState, FmiImportError> {
        type Fmi3GetFmuState = unsafe extern "C" fn(Fmi3Instance, *mut Fmi3FmuState) -> c_int;
        let symbol = self.symbol::<Fmi3GetFmuState>("fmi3GetFMUState")?;
        let mut state = std::ptr::null_mut();
        let raw_status = unsafe {
            // The out-pointer targets a local opaque handle that lives
            // until the FMI call returns.
            symbol(instance, &mut state)
        };
        continuable_status("fmi3GetFMUState", raw_status)?;
        if state.is_null() {
            return Err(FmiImportError::NullFmuState {
                path: self.path.clone(),
            });
        }
        Ok(state)
    }

    /// Call `fmi3SetFMUState` for a previously captured state handle.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the optional symbol is absent or
    /// the FMU returns a non-continuable status.
    pub fn set_fmu_state(
        &self,
        instance: Fmi3Instance,
        state: Fmi3FmuState,
    ) -> Result<Fmi3Status, FmiImportError> {
        type Fmi3SetFmuState = unsafe extern "C" fn(Fmi3Instance, Fmi3FmuState) -> c_int;
        let symbol = self.symbol::<Fmi3SetFmuState>("fmi3SetFMUState")?;
        let raw_status = unsafe {
            // The opaque state handle is owned by the FMU API; this call
            // only passes it back to the FMU.
            symbol(instance, state)
        };
        continuable_status("fmi3SetFMUState", raw_status)
    }

    /// Call `fmi3FreeFMUState` and null the state handle on success.
    ///
    /// # Errors
    ///
    /// Returns [`FmiImportError`] when the optional symbol is absent or
    /// the FMU returns a non-continuable status.
    pub fn free_fmu_state(
        &self,
        instance: Fmi3Instance,
        state: &mut Fmi3FmuState,
    ) -> Result<Fmi3Status, FmiImportError> {
        type Fmi3FreeFmuState = unsafe extern "C" fn(Fmi3Instance, *mut Fmi3FmuState) -> c_int;
        let symbol = self.symbol::<Fmi3FreeFmuState>("fmi3FreeFMUState")?;
        let raw_status = unsafe {
            // The FMI API is responsible for freeing the pointed-to
            // state object and writing a null handle back on success.
            symbol(instance, state)
        };
        continuable_status("fmi3FreeFMUState", raw_status)
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

fn ensure_matching_value_count(
    function: &'static str,
    expected: usize,
    got: usize,
) -> Result<(), FmiImportError> {
    if expected == got {
        Ok(())
    } else {
        Err(FmiImportError::ValueCountMismatch {
            function,
            expected,
            got,
        })
    }
}

fn ensure_sample_count(
    field: &'static str,
    expected: usize,
    got: usize,
) -> Result<(), FmiImportError> {
    if expected == got {
        Ok(())
    } else {
        Err(FmiImportError::SampleCountMismatch {
            field,
            expected,
            got,
        })
    }
}

fn invalid_connection(reason: String) -> FmiImportError {
    FmiImportError::InvalidConnection { reason }
}

fn invalid_clock_schedule(clock: impl Into<String>, reason: String) -> FmiImportError {
    FmiImportError::InvalidClockSchedule {
        clock: clock.into(),
        reason,
    }
}

fn default_output_sample(plan: &Fmi3MasterPlan) -> Fmi3OutputSample {
    Fmi3OutputSample {
        float64: vec![0.0; plan.variables.float64_outputs.len()],
        int32: vec![0; plan.variables.int32_outputs.len()],
        uint64: vec![0; plan.variables.uint64_outputs.len()],
    }
}

fn raw_value_references(bindings: &[Fmi3ValueReference]) -> Vec<Fmi3ValueReferenceRaw> {
    bindings
        .iter()
        .map(|binding| binding.value_reference)
        .collect()
}

fn periodic_input_clock_activation(
    clock: &Fmi3ClockBinding,
    current_time_s: f64,
) -> Result<Option<bool>, FmiImportError> {
    if clock.causality != FmiVariableCausality::Input {
        return Ok(None);
    }

    match clock.interval_variability {
        FmiClockIntervalVariability::Constant
        | FmiClockIntervalVariability::Fixed
        | FmiClockIntervalVariability::Tunable => {
            let interval_s =
                parse_clock_decimal(clock, clock.interval_decimal.as_deref(), "intervalDecimal")?;
            if interval_s <= 0.0 {
                return Err(invalid_clock_schedule(
                    clock.name.clone(),
                    format!("intervalDecimal must be positive, got {interval_s}"),
                ));
            }
            let shift_s = match clock.shift_decimal.as_deref() {
                Some(shift) => parse_clock_decimal(clock, Some(shift), "shiftDecimal")?,
                None => 0.0,
            };
            Ok(Some(clock_is_active_at(
                current_time_s,
                interval_s,
                shift_s,
            )))
        }
        FmiClockIntervalVariability::Changing
        | FmiClockIntervalVariability::Countdown
        | FmiClockIntervalVariability::Triggered => Ok(None),
    }
}

fn parse_clock_decimal(
    clock: &Fmi3ClockBinding,
    value: Option<&str>,
    field: &'static str,
) -> Result<f64, FmiImportError> {
    let value = value.ok_or_else(|| {
        invalid_clock_schedule(clock.name.clone(), format!("{field} is required"))
    })?;
    let parsed = value.parse::<f64>().map_err(|source| {
        invalid_clock_schedule(
            clock.name.clone(),
            format!("{field} {value:?} is not a valid finite f64: {source}"),
        )
    })?;
    if !parsed.is_finite() {
        return Err(invalid_clock_schedule(
            clock.name.clone(),
            format!("{field} must be finite, got {parsed}"),
        ));
    }
    Ok(parsed)
}

fn clock_is_active_at(current_time_s: f64, interval_s: f64, shift_s: f64) -> bool {
    if current_time_s < shift_s {
        return false;
    }
    let phase = (current_time_s - shift_s) / interval_s;
    let nearest_tick = phase.round();
    (phase - nearest_tick).abs() <= 1.0e-9
}

fn continuable_status(
    function: &'static str,
    raw_status: c_int,
) -> Result<Fmi3Status, FmiImportError> {
    let status = Fmi3Status::from_raw(raw_status).ok_or(FmiImportError::UnknownStatus {
        function,
        status: raw_status,
    })?;
    if status.permits_continuation() {
        Ok(status)
    } else {
        Err(FmiImportError::CallFailed { function, status })
    }
}

fn cstring_argument(field: &'static str, value: &str) -> Result<CString, FmiImportError> {
    CString::new(value).map_err(|_| FmiImportError::InvalidStringArgument { field })
}

fn fmi_boolean(value: bool) -> c_int {
    if value { 1 } else { 0 }
}

fn fmi_boolean_to_bool(value: c_int) -> bool {
    value != 0
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

/// Materialize all FMU `resources/` entries into a resources directory.
///
/// Only archive entries below `resources/` are written, and each relative
/// component must be a normal path component. The returned path is the
/// materialized resources directory, or `None` when the FMU has no resource
/// entries.
///
/// # Errors
///
/// Returns [`FmiImportError`] when a resource entry has an unsafe path or
/// cannot be written.
pub fn materialize_archive_resources(
    archive: &FmuArchive,
    output_dir: impl AsRef<Path>,
) -> Result<Option<PathBuf>, FmiImportError> {
    let resources_dir = output_dir.as_ref().join("resources");
    let mut materialized = false;
    for (entry, bytes) in archive.entries() {
        let Some(relative) = entry.strip_prefix("resources/") else {
            continue;
        };
        if relative.is_empty() {
            std::fs::create_dir_all(&resources_dir).map_err(|source| {
                FmiImportError::MaterializeIo {
                    path: resources_dir.clone(),
                    source,
                }
            })?;
            materialized = true;
            continue;
        }
        let relative_path = safe_archive_relative_path(relative, entry)?;
        let path = resources_dir.join(relative_path);
        if entry.ends_with('/') {
            std::fs::create_dir_all(&path).map_err(|source| FmiImportError::MaterializeIo {
                path: path.clone(),
                source,
            })?;
        } else {
            if let Some(parent) = path.parent() {
                std::fs::create_dir_all(parent).map_err(|source| {
                    FmiImportError::MaterializeIo {
                        path: parent.to_path_buf(),
                        source,
                    }
                })?;
            }
            std::fs::write(&path, bytes).map_err(|source| FmiImportError::MaterializeIo {
                path: path.clone(),
                source,
            })?;
        }
        materialized = true;
    }
    Ok(materialized.then_some(resources_dir))
}

fn safe_archive_relative_path(relative: &str, entry: &str) -> Result<PathBuf, FmiImportError> {
    let mut path = PathBuf::new();
    for component in Path::new(relative).components() {
        match component {
            Component::Normal(segment) => path.push(segment),
            _ => {
                return Err(FmiImportError::InvalidArchiveEntryName {
                    entry: entry.to_owned(),
                });
            }
        }
    }
    if path.as_os_str().is_empty() {
        return Err(FmiImportError::InvalidArchiveEntryName {
            entry: entry.to_owned(),
        });
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
    <ScalarVariable name="u" valueReference="10" causality="input"><Float64/></ScalarVariable>
    <ScalarVariable name="y" valueReference="20" causality="output"><Float64/></ScalarVariable>
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
    fn materializes_fmu_resources_safely() {
        let temp = tempfile::tempdir().expect("tempdir");
        let fmu_path = temp.path().join("resource-fixture.fmu");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="resource-fixture">
  <CoSimulation modelIdentifier="resource_fixture"/>
  <ModelVariables>
    <ScalarVariable name="y" valueReference="1" causality="output"><Int32/></ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &fmu_path,
            &[
                ("modelDescription.xml", model_description.as_slice()),
                ("resources/y.txt", b"97"),
                ("resources/nested/z.txt", b"122"),
            ],
        )
        .expect("write resource fixture fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load fixture fmu");

        let resources = materialize_archive_resources(&archive, temp.path().join("materialized"))
            .expect("materialize resources")
            .expect("resource directory should exist");

        assert_eq!(
            std::fs::read_to_string(resources.join("y.txt")).expect("read y resource"),
            "97"
        );
        assert_eq!(
            std::fs::read_to_string(resources.join("nested").join("z.txt"))
                .expect("read nested resource"),
            "122"
        );
    }

    #[test]
    fn point_mass_export_archive_materializes_current_platform_binary() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library_bytes = std::fs::read(&library_path).expect("read fixture library");
        let fmu_path = temp.path().join("openbmp-point-mass.fmu");
        let binary_entry =
            write_openbmp_point_mass_fmu_archive_for_current_platform(&fmu_path, &library_bytes)
                .expect("write point-mass fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load point-mass fmu");

        assert!(archive.entry(binary_entry).is_some());
        assert_eq!(archive.model_description().model_name, "OpenBMP point mass");
        let library = Fmi3DynamicLibrary::open_archive_binary(
            &archive,
            binary_entry,
            temp.path().join("materialized-point-mass"),
        )
        .expect("open materialized point-mass fmu binary");
        let report = library
            .check_cosimulation_smoke_symbols()
            .expect("check materialized point-mass fmu symbols");

        assert_eq!(report.fmi_version, "3.0");
        assert_eq!(report.checked_symbols, FMI3_COSIMULATION_SMOKE_SYMBOLS);
    }

    #[test]
    fn fmi3_binary_entry_for_target_uses_standard_platform_path() {
        assert_eq!(
            fmi3_binary_entry_for_target("Dahlquist", "x86_64", "linux"),
            Some("binaries/x86_64-linux/Dahlquist.so".into())
        );
        assert_eq!(
            fmi3_binary_entry_for_target("Dahlquist", "aarch64", "macos"),
            Some("binaries/aarch64-darwin/Dahlquist.dylib".into())
        );
        assert_eq!(
            fmi3_binary_entry_for_target("Dahlquist", "x86_64", "windows"),
            Some("binaries/x86_64-windows/Dahlquist.dll".into())
        );
        assert!(fmi3_binary_entry_for_target("../Dahlquist", "x86_64", "linux").is_none());
        assert!(fmi3_binary_entry_for_target("Dahlquist/lib", "x86_64", "linux").is_none());
        assert!(fmi3_binary_entry_for_target("Dahlquist", "wasm32", "unknown").is_none());
    }

    #[test]
    fn builds_typed_value_reference_bindings_from_archive() {
        let temp = tempfile::tempdir().expect("tempdir");
        let fmu_path = temp.path().join("fixture.fmu");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="fixture">
  <CoSimulation modelIdentifier="fixture"/>
  <ModelVariables>
    <ScalarVariable name="u" valueReference="10" causality="input"><Float64/></ScalarVariable>
    <ScalarVariable name="mode" valueReference="12" causality="input"><Int32/></ScalarVariable>
    <ScalarVariable name="tick_in" valueReference="11" causality="input"><UInt64/></ScalarVariable>
    <ScalarVariable name="y" valueReference="20" causality="output"><Float64/></ScalarVariable>
    <ScalarVariable name="status" valueReference="22" causality="output"><Int32/></ScalarVariable>
    <ScalarVariable name="tick_out" valueReference="21" causality="output"><UInt64/></ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &fmu_path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fixture fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load fixture fmu");

        let bindings = Fmi3VariableBindings::from_archive(&archive);
        let plan = Fmi3MasterPlan::from_archive(&archive, 0.25, Fmi3CouplingOrder::GaussSeidel)
            .expect("build master plan");

        assert_eq!(
            bindings.float64_inputs,
            vec![Fmi3ValueReference {
                name: "u".into(),
                value_reference: 10,
            }]
        );
        assert_eq!(
            bindings.int32_inputs,
            vec![Fmi3ValueReference {
                name: "mode".into(),
                value_reference: 12,
            }]
        );
        assert_eq!(
            bindings.int32_outputs,
            vec![Fmi3ValueReference {
                name: "status".into(),
                value_reference: 22,
            }]
        );
        assert_eq!(
            bindings.uint64_outputs,
            vec![Fmi3ValueReference {
                name: "tick_out".into(),
                value_reference: 21,
            }]
        );
        assert_eq!(plan.model_identifier, "fixture");
        assert_eq!(plan.variables, bindings);
    }

    #[test]
    fn builds_clock_bindings_from_archive() {
        let temp = tempfile::tempdir().expect("tempdir");
        let fmu_path = temp.path().join("clocked.fmu");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="clocked">
  <CoSimulation modelIdentifier="clocked"/>
  <ModelVariables>
    <Clock name="sample_clock" valueReference="100" causality="input" intervalVariability="constant" intervalDecimal="0.25"/>
    <Float64 name="sampled_y" valueReference="20" causality="output" clocks="100"/>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &fmu_path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fixture fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load fixture fmu");

        let plan = Fmi3MasterPlan::from_archive(&archive, 0.25, Fmi3CouplingOrder::GaussSeidel)
            .expect("build master plan");

        assert_eq!(
            plan.variables.clocks,
            vec![Fmi3ClockBinding {
                name: "sample_clock".into(),
                value_reference: 100,
                causality: FmiVariableCausality::Input,
                interval_variability: FmiClockIntervalVariability::Constant,
                interval_decimal: Some("0.25".into()),
                shift_decimal: None,
            }]
        );
        assert_eq!(
            plan.variables.clocked_variables,
            vec![Fmi3ClockedVariableBinding {
                name: "sampled_y".into(),
                value_reference: 20,
                clock_references: vec![100],
            }]
        );
    }

    #[test]
    fn single_fmu_master_publishes_periodic_input_clock_schedule() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");
        let fmu_path = temp.path().join("clocked-step.fmu");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="clocked">
  <CoSimulation modelIdentifier="fixture"/>
  <ModelVariables>
    <Clock name="sample_clock" valueReference="100" causality="input" intervalVariability="constant" intervalDecimal="0.5" shiftDecimal="0.25"/>
    <Float64 name="u" valueReference="10" causality="input"/>
    <Float64 name="sampled_y" valueReference="20" causality="output" clocks="100"/>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &fmu_path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write clocked fixture fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load fixture fmu");
        let plan = Fmi3MasterPlan::from_archive(&archive, 0.5, Fmi3CouplingOrder::Jacobi)
            .expect("build master plan");
        let mut master = Fmi3SingleFmuMaster::new(&library, std::ptr::null_mut(), plan);

        let first = master
            .step(&Fmi3InputSample {
                float64: vec![2.0],
                int32: Vec::new(),
                uint64: Vec::new(),
            })
            .expect("first clocked step");
        assert_eq!(
            library
                .get_clock(std::ptr::null_mut(), &[100])
                .expect("get clock"),
            vec![false],
            "shifted clock is inactive at t=0.0",
        );
        assert_eq!(first.current_time_s.to_bits(), 0.25_f64.to_bits());

        let second = master
            .step(&Fmi3InputSample {
                float64: vec![3.0],
                int32: Vec::new(),
                uint64: Vec::new(),
            })
            .expect("second clocked step");
        assert_eq!(
            library
                .get_clock(std::ptr::null_mut(), &[100])
                .expect("get clock"),
            vec![true],
            "shifted clock activates at t=0.25",
        );
        assert_eq!(second.outputs.float64[0].to_bits(), 3.0_f64.to_bits());
    }

    #[test]
    fn master_plan_rejects_invalid_communication_step() {
        let temp = tempfile::tempdir().expect("tempdir");
        let fmu_path = temp.path().join("fixture.fmu");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="fixture">
  <CoSimulation modelIdentifier="fixture"/>
  <ModelVariables>
    <ScalarVariable name="u" valueReference="10" causality="input"><Float64/></ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &fmu_path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fixture fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load fixture fmu");

        let err = Fmi3MasterPlan::from_archive(&archive, 0.0, Fmi3CouplingOrder::Jacobi)
            .expect_err("zero step should fail");

        assert!(
            matches!(err, FmiImportError::InvalidCommunicationStep { .. }),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn calls_typed_float64_int32_uint64_and_do_step_symbols() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");
        let instance = std::ptr::null_mut();

        assert_eq!(
            library
                .set_float64(instance, &[10], &[42.5])
                .expect("set float"),
            Fmi3Status::Ok
        );
        let floats = library
            .get_float64(instance, &[20])
            .expect("get float")
            .into_iter()
            .map(f64::to_bits)
            .collect::<Vec<_>>();
        assert_eq!(floats, vec![42.5_f64.to_bits()]);

        assert_eq!(
            library.set_int32(instance, &[12], &[-7]).expect("set int"),
            Fmi3Status::Ok
        );
        assert_eq!(
            library.get_int32(instance, &[22]).expect("get int"),
            vec![-7]
        );

        assert_eq!(
            library
                .set_uint64(instance, &[11], &[123])
                .expect("set uint"),
            Fmi3Status::Ok
        );
        assert_eq!(
            library.get_uint64(instance, &[21]).expect("get uint"),
            vec![123]
        );

        assert_eq!(
            library
                .set_clock(instance, &[100], &[true])
                .expect("set clock"),
            Fmi3Status::Ok
        );
        assert_eq!(
            library.get_clock(instance, &[100]).expect("get clock"),
            vec![true]
        );

        let step_report = library.do_step(instance, 1.0, 0.5, true).expect("do step");

        assert_eq!(step_report.status, Fmi3Status::Ok);
        assert!(!step_report.event_handling_needed);
        assert!(!step_report.terminate_simulation);
        match step_report.outcome {
            Fmi3DoStepOutcome::EarlyReturn {
                last_successful_time_s,
            } => assert_eq!(last_successful_time_s.to_bits(), 1.25_f64.to_bits()),
            Fmi3DoStepOutcome::Complete => panic!("expected early return"),
        }
    }

    #[test]
    fn co_simulation_instance_initializes_steps_and_frees() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");
        let mut instance = library
            .instantiate_co_simulation(Fmi3CoSimulationInstantiation::new(
                "openbmp-fixture",
                "fixture-token",
            ))
            .expect("instantiate co-simulation");

        assert!(!instance.instance().is_null());
        assert_eq!(
            instance
                .enter_initialization_mode(0.0, Some(2.0), None)
                .expect("enter initialization"),
            Fmi3Status::Ok
        );
        assert_eq!(
            instance
                .exit_initialization_mode()
                .expect("exit initialization"),
            Fmi3Status::Ok
        );
        assert_eq!(
            instance
                .set_float64(&[10], &[9.25])
                .expect("set instance float"),
            Fmi3Status::Ok
        );
        assert_eq!(
            instance.set_int32(&[12], &[-11]).expect("set instance int"),
            Fmi3Status::Ok
        );
        assert_eq!(
            instance
                .set_uint64(&[11], &[99])
                .expect("set instance uint"),
            Fmi3Status::Ok
        );

        let report = instance.do_step(0.0, 0.5, true).expect("step instance");
        let floats = instance.get_float64(&[20]).expect("get instance float");
        let ints = instance.get_int32(&[22]).expect("get instance int");
        let uints = instance.get_uint64(&[21]).expect("get instance uint");

        assert_eq!(floats[0].to_bits(), 9.25_f64.to_bits());
        assert_eq!(ints, vec![-11]);
        assert_eq!(uints, vec![99]);
        assert!(matches!(
            report.outcome,
            Fmi3DoStepOutcome::EarlyReturn {
                last_successful_time_s
            } if last_successful_time_s.to_bits() == 0.25_f64.to_bits()
        ));
        assert_eq!(instance.terminate().expect("terminate"), Fmi3Status::Ok);
        assert!(instance.is_terminated());
        assert_eq!(
            instance.terminate().expect("terminate twice"),
            Fmi3Status::Ok
        );
    }

    #[test]
    fn instantiate_rejects_interior_nul_string_argument() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");

        let err = library
            .instantiate_co_simulation(Fmi3CoSimulationInstantiation::new(
                "openbmp\0fixture",
                "fixture-token",
            ))
            .expect_err("interior NUL should fail before calling FMU");

        assert!(
            matches!(
                err,
                FmiImportError::InvalidStringArgument {
                    field: "instance_name"
                }
            ),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn single_fmu_master_steps_typed_inputs_and_outputs() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");
        let fmu_path = temp.path().join("fixture.fmu");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="fixture">
  <CoSimulation modelIdentifier="fixture"/>
  <ModelVariables>
    <ScalarVariable name="u" valueReference="10" causality="input"><Float64/></ScalarVariable>
    <ScalarVariable name="mode" valueReference="12" causality="input"><Int32/></ScalarVariable>
    <ScalarVariable name="tick_in" valueReference="11" causality="input"><UInt64/></ScalarVariable>
    <ScalarVariable name="y" valueReference="20" causality="output"><Float64/></ScalarVariable>
    <ScalarVariable name="status" valueReference="22" causality="output"><Int32/></ScalarVariable>
    <ScalarVariable name="tick_out" valueReference="21" causality="output"><UInt64/></ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &fmu_path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fixture fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load fixture fmu");
        let plan = Fmi3MasterPlan::from_archive(&archive, 0.5, Fmi3CouplingOrder::GaussSeidel)
            .expect("build master plan");
        let mut master = Fmi3SingleFmuMaster::new(&library, std::ptr::null_mut(), plan);

        let result = master
            .step(&Fmi3InputSample {
                float64: vec![7.25],
                int32: vec![-3],
                uint64: vec![88],
            })
            .expect("step master");

        assert_eq!(result.outputs.float64[0].to_bits(), 7.25_f64.to_bits());
        assert_eq!(result.outputs.int32, vec![-3]);
        assert_eq!(result.outputs.uint64, vec![88]);
        assert_eq!(result.current_time_s.to_bits(), 0.25_f64.to_bits());
        assert_eq!(master.current_time_s().to_bits(), 0.25_f64.to_bits());
        assert!(matches!(
            result.do_step.outcome,
            Fmi3DoStepOutcome::EarlyReturn { .. }
        ));
    }

    #[test]
    fn single_fmu_master_restores_fmu_state_on_early_return_policy() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");
        let instance = std::ptr::null_mut();
        let fmu_path = temp.path().join("fixture.fmu");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="fixture">
  <CoSimulation modelIdentifier="fixture"/>
  <ModelVariables>
    <ScalarVariable name="u" valueReference="10" causality="input"><Float64/></ScalarVariable>
    <ScalarVariable name="mode" valueReference="12" causality="input"><Int32/></ScalarVariable>
    <ScalarVariable name="tick_in" valueReference="11" causality="input"><UInt64/></ScalarVariable>
    <ScalarVariable name="y" valueReference="20" causality="output"><Float64/></ScalarVariable>
    <ScalarVariable name="status" valueReference="22" causality="output"><Int32/></ScalarVariable>
    <ScalarVariable name="tick_out" valueReference="21" causality="output"><UInt64/></ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &fmu_path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fixture fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load fixture fmu");
        let plan = Fmi3MasterPlan::from_archive(&archive, 0.5, Fmi3CouplingOrder::GaussSeidel)
            .expect("build master plan");

        library
            .set_float64(instance, &[10], &[1.5])
            .expect("seed float state");
        library
            .set_int32(instance, &[12], &[4])
            .expect("seed int state");
        library
            .set_uint64(instance, &[11], &[15])
            .expect("seed uint state");
        let mut master = Fmi3SingleFmuMaster::new(&library, instance, plan);

        let result = master
            .step_with_rollback_policy(
                &Fmi3InputSample {
                    float64: vec![7.25],
                    int32: vec![-3],
                    uint64: vec![88],
                },
                Fmi3RollbackPolicy::RestoreOnEarlyReturn,
            )
            .expect("checkpointed step");

        assert_eq!(result.step.outputs.float64[0].to_bits(), 7.25_f64.to_bits());
        assert_eq!(result.step.outputs.int32, vec![-3]);
        assert_eq!(result.step.outputs.uint64, vec![88]);
        assert_eq!(result.step.current_time_s.to_bits(), 0.25_f64.to_bits());
        assert!(matches!(
            result.action,
            Fmi3RollbackAction::Restored {
                restored_time_s,
                attempted_time_s,
            } if restored_time_s.to_bits() == 0.0_f64.to_bits()
                && attempted_time_s.to_bits() == 0.25_f64.to_bits()
        ));
        assert_eq!(result.current_time_s.to_bits(), 0.0_f64.to_bits());
        assert_eq!(master.current_time_s().to_bits(), 0.0_f64.to_bits());

        let floats = library.get_float64(instance, &[20]).expect("get float");
        let ints = library.get_int32(instance, &[22]).expect("get int");
        let uints = library.get_uint64(instance, &[21]).expect("get uint");
        assert_eq!(floats[0].to_bits(), 1.5_f64.to_bits());
        assert_eq!(ints, vec![4]);
        assert_eq!(uints, vec![15]);
    }

    #[test]
    fn single_fmu_master_rejects_input_count_mismatch() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");
        let fmu_path = temp.path().join("fixture.fmu");
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="fixture">
  <CoSimulation modelIdentifier="fixture"/>
  <ModelVariables>
    <ScalarVariable name="u" valueReference="10" causality="input"><Float64/></ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            &fmu_path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fixture fmu");
        let archive = FmuArchive::load(&fmu_path).expect("load fixture fmu");
        let plan = Fmi3MasterPlan::from_archive(&archive, 0.5, Fmi3CouplingOrder::Jacobi)
            .expect("build master plan");
        let mut master = Fmi3SingleFmuMaster::new(&library, std::ptr::null_mut(), plan);

        let err = master
            .step(&Fmi3InputSample::default())
            .expect_err("missing input value should fail");

        assert!(
            matches!(
                err,
                FmiImportError::SampleCountMismatch {
                    field: "Float64 input",
                    expected: 1,
                    got: 0,
                }
            ),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn multi_fmu_master_applies_gauss_seidel_connections_in_step() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_a_path = compile_named_fixture_library(temp.path(), "fixture_gs_a");
        let library_b_path = compile_named_fixture_library(temp.path(), "fixture_gs_b");
        let library_a = Fmi3DynamicLibrary::open(&library_a_path).expect("open fixture library A");
        let library_b = Fmi3DynamicLibrary::open(&library_b_path).expect("open fixture library B");
        let fmu_path = temp.path().join("fixture.fmu");
        let archive = write_float64_fixture_fmu(&fmu_path);
        let plan = Fmi3MasterPlan::from_archive(&archive, 0.5, Fmi3CouplingOrder::GaussSeidel)
            .expect("build master plan");
        let mut master = Fmi3MultiFmuMaster::new(
            vec![
                Fmi3FmuNode::new(&library_a, std::ptr::null_mut(), plan.clone()),
                Fmi3FmuNode::new(&library_b, std::ptr::null_mut(), plan),
            ],
            Fmi3CouplingOrder::GaussSeidel,
        );
        master
            .add_float64_connection(Fmi3Float64Connection {
                source_node: 0,
                source_output: 0,
                target_node: 1,
                target_input: 0,
            })
            .expect("add connection");

        let result = master
            .step(&[
                Fmi3InputSample {
                    float64: vec![2.0],
                    int32: Vec::new(),
                    uint64: Vec::new(),
                },
                Fmi3InputSample {
                    float64: vec![9.0],
                    int32: Vec::new(),
                    uint64: Vec::new(),
                },
            ])
            .expect("step multi-FMU master");

        assert_eq!(result.outputs[0].float64[0].to_bits(), 2.0_f64.to_bits());
        assert_eq!(result.outputs[1].float64[0].to_bits(), 2.0_f64.to_bits());
        assert_eq!(
            result.current_times_s,
            vec![0.25_f64, 0.25_f64],
            "each fixture returns early halfway through the macro step",
        );
        assert_eq!(master.last_outputs(), result.outputs.as_slice());
    }

    #[test]
    fn multi_fmu_master_applies_jacobi_connections_from_previous_outputs() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_a_path = compile_named_fixture_library(temp.path(), "fixture_jacobi_a");
        let library_b_path = compile_named_fixture_library(temp.path(), "fixture_jacobi_b");
        let library_a = Fmi3DynamicLibrary::open(&library_a_path).expect("open fixture library A");
        let library_b = Fmi3DynamicLibrary::open(&library_b_path).expect("open fixture library B");
        let fmu_path = temp.path().join("fixture.fmu");
        let archive = write_float64_fixture_fmu(&fmu_path);
        let plan = Fmi3MasterPlan::from_archive(&archive, 0.5, Fmi3CouplingOrder::Jacobi)
            .expect("build master plan");
        let mut master = Fmi3MultiFmuMaster::with_initial_outputs(
            vec![
                Fmi3FmuNode::new(&library_a, std::ptr::null_mut(), plan.clone()),
                Fmi3FmuNode::new(&library_b, std::ptr::null_mut(), plan),
            ],
            Fmi3CouplingOrder::Jacobi,
            vec![
                Fmi3OutputSample {
                    float64: vec![1.0],
                    int32: Vec::new(),
                    uint64: Vec::new(),
                },
                Fmi3OutputSample {
                    float64: vec![0.0],
                    int32: Vec::new(),
                    uint64: Vec::new(),
                },
            ],
        )
        .expect("build seeded multi-FMU master");
        master
            .add_float64_connection(Fmi3Float64Connection {
                source_node: 0,
                source_output: 0,
                target_node: 1,
                target_input: 0,
            })
            .expect("add connection");

        let result = master
            .step(&[
                Fmi3InputSample {
                    float64: vec![2.0],
                    int32: Vec::new(),
                    uint64: Vec::new(),
                },
                Fmi3InputSample {
                    float64: vec![9.0],
                    int32: Vec::new(),
                    uint64: Vec::new(),
                },
            ])
            .expect("step multi-FMU master");

        assert_eq!(result.outputs[0].float64[0].to_bits(), 2.0_f64.to_bits());
        assert_eq!(result.outputs[1].float64[0].to_bits(), 1.0_f64.to_bits());
        assert_eq!(master.last_outputs(), result.outputs.as_slice());
    }

    #[test]
    fn multi_fmu_master_predictor_corrector_restores_then_corrects_jacobi_step() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_a_path = compile_named_fixture_library(temp.path(), "fixture_pc_a");
        let library_b_path = compile_named_fixture_library(temp.path(), "fixture_pc_b");
        let library_a = Fmi3DynamicLibrary::open(&library_a_path).expect("open fixture library A");
        let library_b = Fmi3DynamicLibrary::open(&library_b_path).expect("open fixture library B");
        let fmu_path = temp.path().join("fixture.fmu");
        let archive = write_float64_fixture_fmu(&fmu_path);
        let plan = Fmi3MasterPlan::from_archive(&archive, 0.5, Fmi3CouplingOrder::Jacobi)
            .expect("build master plan");
        let mut master = Fmi3MultiFmuMaster::with_initial_outputs(
            vec![
                Fmi3FmuNode::new(&library_a, std::ptr::null_mut(), plan.clone()),
                Fmi3FmuNode::new(&library_b, std::ptr::null_mut(), plan),
            ],
            Fmi3CouplingOrder::Jacobi,
            vec![
                Fmi3OutputSample {
                    float64: vec![1.0],
                    int32: Vec::new(),
                    uint64: Vec::new(),
                },
                Fmi3OutputSample {
                    float64: vec![0.0],
                    int32: Vec::new(),
                    uint64: Vec::new(),
                },
            ],
        )
        .expect("build seeded multi-FMU master");
        master
            .add_float64_connection(Fmi3Float64Connection {
                source_node: 0,
                source_output: 0,
                target_node: 1,
                target_input: 0,
            })
            .expect("add connection");

        let result = master
            .step_predictor_corrector(&[
                Fmi3InputSample {
                    float64: vec![2.0],
                    int32: Vec::new(),
                    uint64: Vec::new(),
                },
                Fmi3InputSample {
                    float64: vec![9.0],
                    int32: Vec::new(),
                    uint64: Vec::new(),
                },
            ])
            .expect("step predictor/corrector multi-FMU master");

        assert_eq!(
            result.predictor.outputs[1].float64[0].to_bits(),
            1.0_f64.to_bits(),
            "predictor uses the previous-output cache",
        );
        assert_eq!(
            result.corrector.outputs[1].float64[0].to_bits(),
            2.0_f64.to_bits(),
            "corrector uses predictor output from node 0",
        );
        assert_eq!(
            result.corrector.current_times_s,
            vec![0.25_f64, 0.25_f64],
            "the corrector is restored to the macro-step start before stepping",
        );
        assert_eq!(master.last_outputs(), result.corrector.outputs.as_slice());
    }

    #[test]
    fn multi_fmu_master_rejects_invalid_float64_connection() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");
        let fmu_path = temp.path().join("fixture.fmu");
        let archive = write_float64_fixture_fmu(&fmu_path);
        let plan = Fmi3MasterPlan::from_archive(&archive, 0.5, Fmi3CouplingOrder::GaussSeidel)
            .expect("build master plan");
        let mut master = Fmi3MultiFmuMaster::new(
            vec![Fmi3FmuNode::new(&library, std::ptr::null_mut(), plan)],
            Fmi3CouplingOrder::GaussSeidel,
        );

        let err = master
            .add_float64_connection(Fmi3Float64Connection {
                source_node: 0,
                source_output: 1,
                target_node: 0,
                target_input: 0,
            })
            .expect_err("out-of-range connection should fail");

        assert!(
            matches!(err, FmiImportError::InvalidConnection { .. }),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn calls_optional_fmu_state_snapshot_restore_symbols() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");
        let instance = std::ptr::null_mut();

        library
            .set_float64(instance, &[10], &[1.5])
            .expect("set initial float");
        library
            .set_uint64(instance, &[11], &[15])
            .expect("set initial uint");
        let mut state = library.get_fmu_state(instance).expect("snapshot state");

        library
            .set_float64(instance, &[10], &[2.5])
            .expect("set changed float");
        library
            .set_uint64(instance, &[11], &[25])
            .expect("set changed uint");
        library
            .set_fmu_state(instance, state)
            .expect("restore state");

        let floats = library.get_float64(instance, &[20]).expect("get float");
        let uints = library.get_uint64(instance, &[21]).expect("get uint");
        assert_eq!(floats[0].to_bits(), 1.5_f64.to_bits());
        assert_eq!(uints, vec![15]);

        library
            .free_fmu_state(instance, &mut state)
            .expect("free state");
        assert!(state.is_null());
    }

    #[test]
    fn set_float64_rejects_value_count_mismatch_before_calling_fmu() {
        let temp = tempfile::tempdir().expect("tempdir");
        let library_path = compile_fixture_library(temp.path());
        let library = Fmi3DynamicLibrary::open(&library_path).expect("open fixture library");

        let err = library
            .set_float64(std::ptr::null_mut(), &[10, 11], &[42.5])
            .expect_err("mismatched counts should fail");

        assert!(
            matches!(
                err,
                FmiImportError::ValueCountMismatch {
                    function: "fmi3SetFloat64",
                    expected: 2,
                    got: 1,
                }
            ),
            "unexpected error: {err}",
        );
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

    fn compile_named_fixture_library(dir: &Path, name: &str) -> PathBuf {
        compile_rust_cdylib(dir, name, FIXTURE_LIBRARY_SOURCE)
    }

    fn compile_incomplete_fixture_library(dir: &Path) -> PathBuf {
        compile_rust_cdylib(dir, "incomplete", INCOMPLETE_LIBRARY_SOURCE)
    }

    fn write_float64_fixture_fmu(path: &Path) -> FmuArchive {
        let model_description = br#"<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName="fixture">
  <CoSimulation modelIdentifier="fixture"/>
  <ModelVariables>
    <ScalarVariable name="u" valueReference="10" causality="input"><Float64/></ScalarVariable>
    <ScalarVariable name="y" valueReference="20" causality="output"><Float64/></ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"#;
        write_stored_zip(
            path,
            &[("modelDescription.xml", model_description.as_slice())],
        )
        .expect("write fixture fmu");
        FmuArchive::load(path).expect("load fixture fmu")
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
use std::ffi::{c_char, c_int, c_void};
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU64, Ordering};

static VERSION: &[u8] = b"3.0\0";
static FLOAT_VALUE_BITS: AtomicU64 = AtomicU64::new(0);
static INT_VALUE: AtomicI32 = AtomicI32::new(0);
static UINT_VALUE: AtomicU64 = AtomicU64::new(0);
static CLOCK_VALUE: AtomicBool = AtomicBool::new(false);
const FMI3_OK: c_int = 0;
const FMI3_ERROR: c_int = 3;

#[repr(C)]
struct Snapshot {
    float_value_bits: u64,
    int_value: i32,
    uint_value: u64,
    clock_value: bool,
}

struct FixtureInstance {
    terminated: bool,
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetVersion() -> *const c_char {
    VERSION.as_ptr().cast()
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3InstantiateCoSimulation(
    _instance_name: *const c_char,
    _instantiation_token: *const c_char,
    _resource_path: *const c_char,
    _visible: c_int,
    _logging_on: c_int,
    _event_mode_used: c_int,
    _early_return_allowed: c_int,
    _required_intermediate_variables: *const u32,
    _n_required_intermediate_variables: usize,
    _instance_environment: *mut c_void,
    _log_message: *mut c_void,
    _intermediate_update: *mut c_void,
) -> *mut c_void {
    Box::into_raw(Box::new(FixtureInstance { terminated: false })).cast()
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3EnterInitializationMode(
    instance: *mut c_void,
    _tolerance_defined: c_int,
    _tolerance: f64,
    start_time: f64,
    _stop_time_defined: c_int,
    _stop_time: f64,
) -> c_int {
    if instance.is_null() || !start_time.is_finite() {
        return FMI3_ERROR;
    }
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3ExitInitializationMode(instance: *mut c_void) -> c_int {
    if instance.is_null() {
        return FMI3_ERROR;
    }
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3SetFloat64(
    _instance: *mut c_void,
    value_references: *const u32,
    n_value_references: usize,
    values: *const f64,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || n_values != 1 || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let value_reference = unsafe { *value_references };
    if value_reference != 10 {
        return FMI3_ERROR;
    }
    let value = unsafe { *values };
    FLOAT_VALUE_BITS.store(value.to_bits(), Ordering::SeqCst);
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetFloat64(
    _instance: *mut c_void,
    value_references: *const u32,
    n_value_references: usize,
    values: *mut f64,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || n_values != 1 || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let value_reference = unsafe { *value_references };
    if value_reference != 20 {
        return FMI3_ERROR;
    }
    unsafe {
        *values = f64::from_bits(FLOAT_VALUE_BITS.load(Ordering::SeqCst));
    }
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3SetInt32(
    _instance: *mut c_void,
    value_references: *const u32,
    n_value_references: usize,
    values: *const i32,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || n_values != 1 || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let value_reference = unsafe { *value_references };
    if value_reference != 12 {
        return FMI3_ERROR;
    }
    let value = unsafe { *values };
    INT_VALUE.store(value, Ordering::SeqCst);
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetInt32(
    _instance: *mut c_void,
    value_references: *const u32,
    n_value_references: usize,
    values: *mut i32,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || n_values != 1 || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let value_reference = unsafe { *value_references };
    if value_reference != 22 {
        return FMI3_ERROR;
    }
    unsafe {
        *values = INT_VALUE.load(Ordering::SeqCst);
    }
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3SetUInt64(
    _instance: *mut c_void,
    value_references: *const u32,
    n_value_references: usize,
    values: *const u64,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || n_values != 1 || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let value_reference = unsafe { *value_references };
    if value_reference != 11 {
        return FMI3_ERROR;
    }
    let value = unsafe { *values };
    UINT_VALUE.store(value, Ordering::SeqCst);
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetUInt64(
    _instance: *mut c_void,
    value_references: *const u32,
    n_value_references: usize,
    values: *mut u64,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || n_values != 1 || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let value_reference = unsafe { *value_references };
    if value_reference != 21 {
        return FMI3_ERROR;
    }
    unsafe {
        *values = UINT_VALUE.load(Ordering::SeqCst);
    }
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3SetClock(
    _instance: *mut c_void,
    value_references: *const u32,
    n_value_references: usize,
    values: *const c_int,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || n_values != 1 || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let value_reference = unsafe { *value_references };
    if value_reference != 100 {
        return FMI3_ERROR;
    }
    let value = unsafe { *values };
    CLOCK_VALUE.store(value != 0, Ordering::SeqCst);
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetClock(
    _instance: *mut c_void,
    value_references: *const u32,
    n_value_references: usize,
    values: *mut c_int,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || n_values != 1 || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let value_reference = unsafe { *value_references };
    if value_reference != 100 {
        return FMI3_ERROR;
    }
    unsafe {
        *values = if CLOCK_VALUE.load(Ordering::SeqCst) { 1 } else { 0 };
    }
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3DoStep(
    _instance: *mut c_void,
    current_communication_point: f64,
    communication_step_size: f64,
    _no_set_fmu_state_prior_to_current_point: c_int,
    event_handling_needed: *mut c_int,
    terminate_simulation: *mut c_int,
    early_return: *mut c_int,
    last_successful_time: *mut f64,
) -> c_int {
    if event_handling_needed.is_null()
        || terminate_simulation.is_null()
        || early_return.is_null()
        || last_successful_time.is_null()
    {
        return FMI3_ERROR;
    }
    unsafe {
        *event_handling_needed = 0;
        *terminate_simulation = 0;
        *early_return = 1;
        *last_successful_time = current_communication_point + communication_step_size * 0.5;
    }
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetFMUState(
    _instance: *mut c_void,
    state: *mut *mut c_void,
) -> c_int {
    if state.is_null() {
        return FMI3_ERROR;
    }
    let snapshot = Box::new(Snapshot {
        float_value_bits: FLOAT_VALUE_BITS.load(Ordering::SeqCst),
        int_value: INT_VALUE.load(Ordering::SeqCst),
        uint_value: UINT_VALUE.load(Ordering::SeqCst),
        clock_value: CLOCK_VALUE.load(Ordering::SeqCst),
    });
    unsafe {
        *state = Box::into_raw(snapshot).cast();
    }
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3SetFMUState(
    _instance: *mut c_void,
    state: *mut c_void,
) -> c_int {
    if state.is_null() {
        return FMI3_ERROR;
    }
    let snapshot = unsafe { &*state.cast::<Snapshot>() };
    FLOAT_VALUE_BITS.store(snapshot.float_value_bits, Ordering::SeqCst);
    INT_VALUE.store(snapshot.int_value, Ordering::SeqCst);
    UINT_VALUE.store(snapshot.uint_value, Ordering::SeqCst);
    CLOCK_VALUE.store(snapshot.clock_value, Ordering::SeqCst);
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3FreeFMUState(
    _instance: *mut c_void,
    state: *mut *mut c_void,
) -> c_int {
    if state.is_null() {
        return FMI3_ERROR;
    }
    let snapshot = unsafe { *state };
    if !snapshot.is_null() {
        unsafe {
            drop(Box::from_raw(snapshot.cast::<Snapshot>()));
            *state = std::ptr::null_mut();
        }
    }
    FMI3_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi3Terminate(instance: *mut c_void) -> c_int {
    if instance.is_null() {
        return FMI3_ERROR;
    }
    unsafe {
        (*instance.cast::<FixtureInstance>()).terminated = true;
    }
    FMI3_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3FreeInstance(instance: *mut c_void) {
    if !instance.is_null() {
        unsafe {
            drop(Box::from_raw(instance.cast::<FixtureInstance>()));
        }
    }
}
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
