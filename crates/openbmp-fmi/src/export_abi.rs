//! Minimal OpenBMP FMI 3 export C-symbol surface.
//!
//! This module backs the `openbmp-fmi` `cdylib` artifact with a small,
//! deterministic point-mass-like plant state. It is intentionally a
//! restricted export smoke surface: it proves lifecycle, typed
//! Float64/Int32/UInt64 value access, fixed-step advancement, and FMU-state
//! snapshot/restore, plus enough fail-closed FMI 3 stubs for FMPy to
//! load the library. It still does not claim FMI conformance.

use std::ffi::{c_char, c_int, c_void};

use crate::export::{
    OPENBMP_POINT_MASS_VR_ALTITUDE_M, OPENBMP_POINT_MASS_VR_STEP, OPENBMP_POINT_MASS_VR_THROTTLE,
    OPENBMP_POINT_MASS_VR_TIME_S, OPENBMP_POINT_MASS_VR_VELOCITY_M_S,
};
use crate::{Fmi3FmuState, Fmi3Instance, Fmi3ValueReferenceRaw};

const FMI3_OK: c_int = 0;
const FMI3_ERROR: c_int = 3;
const VERSION: &[u8] = b"3.0\0";

const VR_TIME_S: Fmi3ValueReferenceRaw = OPENBMP_POINT_MASS_VR_TIME_S;
const VR_THROTTLE: Fmi3ValueReferenceRaw = OPENBMP_POINT_MASS_VR_THROTTLE;
const VR_ALTITUDE_M: Fmi3ValueReferenceRaw = OPENBMP_POINT_MASS_VR_ALTITUDE_M;
const VR_STEP: Fmi3ValueReferenceRaw = OPENBMP_POINT_MASS_VR_STEP;
const VR_VELOCITY_M_S: Fmi3ValueReferenceRaw = OPENBMP_POINT_MASS_VR_VELOCITY_M_S;

const THROTTLE_ACCEL_M_S2: f64 = 20.0;
const GRAVITY_M_S2: f64 = 9.80665;

#[derive(Clone, Debug)]
struct OpenBmpFmiPlantInstance {
    time_s: f64,
    altitude_m: f64,
    velocity_m_s: f64,
    throttle: f64,
    step: u64,
}

impl Default for OpenBmpFmiPlantInstance {
    fn default() -> Self {
        Self {
            time_s: 0.0,
            altitude_m: 0.0,
            velocity_m_s: 0.0,
            throttle: 0.0,
            step: 0,
        }
    }
}

#[derive(Clone, Debug)]
struct OpenBmpFmiPlantSnapshot {
    plant: OpenBmpFmiPlantInstance,
}

/// Return the supported FMI version string.
#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetVersion() -> *const c_char {
    VERSION.as_ptr().cast()
}

macro_rules! unsupported_fmi3_status_symbol {
    ($($name:ident),+ $(,)?) => {
        $(
            #[doc = concat!(
                "Unsupported FMI 3 symbol `",
                stringify!($name),
                "` exported for full-symbol-table loader compatibility."
            )]
            #[unsafe(no_mangle)]
            pub extern "C" fn $name() -> c_int {
                FMI3_ERROR
            }
        )+
    };
}

macro_rules! unsupported_fmi3_instance_symbol {
    ($($name:ident),+ $(,)?) => {
        $(
            #[doc = concat!(
                "Unsupported FMI 3 instantiation symbol `",
                stringify!($name),
                "` exported for full-symbol-table loader compatibility."
            )]
            #[unsafe(no_mangle)]
            pub extern "C" fn $name() -> Fmi3Instance {
                std::ptr::null_mut()
            }
        )+
    };
}

unsupported_fmi3_instance_symbol!(
    fmi3InstantiateModelExchange,
    fmi3InstantiateScheduledExecution,
);

unsupported_fmi3_status_symbol!(
    fmi3ActivateModelPartition,
    fmi3CompletedIntegratorStep,
    fmi3DeserializeFMUState,
    fmi3EnterConfigurationMode,
    fmi3EnterContinuousTimeMode,
    fmi3EnterEventMode,
    fmi3EnterStepMode,
    fmi3EvaluateDiscreteStates,
    fmi3ExitConfigurationMode,
    fmi3GetAdjointDerivative,
    fmi3GetBinary,
    fmi3GetBoolean,
    fmi3GetClock,
    fmi3GetContinuousStateDerivatives,
    fmi3GetContinuousStates,
    fmi3GetDirectionalDerivative,
    fmi3GetEventIndicators,
    fmi3GetFloat32,
    fmi3GetInt8,
    fmi3GetInt16,
    fmi3GetInt64,
    fmi3GetIntervalDecimal,
    fmi3GetIntervalFraction,
    fmi3GetNominalsOfContinuousStates,
    fmi3GetNumberOfContinuousStates,
    fmi3GetNumberOfEventIndicators,
    fmi3GetNumberOfVariableDependencies,
    fmi3GetOutputDerivatives,
    fmi3GetShiftDecimal,
    fmi3GetShiftFraction,
    fmi3GetString,
    fmi3GetUInt8,
    fmi3GetUInt16,
    fmi3GetUInt32,
    fmi3GetVariableDependencies,
    fmi3Reset,
    fmi3SerializeFMUState,
    fmi3SerializedFMUStateSize,
    fmi3SetBinary,
    fmi3SetBoolean,
    fmi3SetClock,
    fmi3SetContinuousStates,
    fmi3SetDebugLogging,
    fmi3SetFloat32,
    fmi3SetInt8,
    fmi3SetInt16,
    fmi3SetInt64,
    fmi3SetIntervalDecimal,
    fmi3SetIntervalFraction,
    fmi3SetShiftDecimal,
    fmi3SetShiftFraction,
    fmi3SetString,
    fmi3SetTime,
    fmi3SetUInt8,
    fmi3SetUInt16,
    fmi3SetUInt32,
    fmi3UpdateDiscreteStates,
);

/// Instantiate the restricted OpenBMP co-simulation export instance.
///
/// The argument list follows the FMI 3 co-simulation shape, but this
/// smoke implementation ignores metadata and callbacks.
#[allow(clippy::too_many_arguments)]
#[unsafe(no_mangle)]
pub extern "C" fn fmi3InstantiateCoSimulation(
    _instance_name: *const c_char,
    _instantiation_token: *const c_char,
    _resource_path: *const c_char,
    _visible: c_int,
    _logging_on: c_int,
    _event_mode_used: c_int,
    _early_return_allowed: c_int,
    _required_intermediate_variables: *const Fmi3ValueReferenceRaw,
    _n_required_intermediate_variables: usize,
    _instance_environment: *mut c_void,
    _log_message: *mut c_void,
    _intermediate_update: *mut c_void,
) -> Fmi3Instance {
    Box::into_raw(Box::new(OpenBmpFmiPlantInstance::default())).cast()
}

/// Enter initialization mode and set the exported plant time.
///
/// Returns FMI error when the instance is null or `start_time` is not
/// finite.
///
/// # Safety
///
/// `instance` must be either null or a pointer returned by
/// [`fmi3InstantiateCoSimulation`] that has not yet been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3EnterInitializationMode(
    instance: Fmi3Instance,
    _tolerance_defined: c_int,
    _tolerance: f64,
    start_time: f64,
    _stop_time_defined: c_int,
    _stop_time: f64,
) -> c_int {
    let Some(plant) = (unsafe { instance_mut(instance) }) else {
        return FMI3_ERROR;
    };
    if !start_time.is_finite() {
        return FMI3_ERROR;
    }
    plant.time_s = start_time;
    plant.step = 0;
    FMI3_OK
}

/// Exit initialization mode.
///
/// Returns FMI error when the instance is null.
///
/// # Safety
///
/// `instance` must be either null or a live pointer returned by
/// [`fmi3InstantiateCoSimulation`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3ExitInitializationMode(instance: Fmi3Instance) -> c_int {
    if instance.is_null() {
        FMI3_ERROR
    } else {
        FMI3_OK
    }
}

/// Set Float64 values by value reference.
///
/// Supported value references are throttle (`10`), altitude (`20`),
/// velocity (`22`), and time (`0`) for deterministic test setup.
///
/// # Safety
///
/// `instance` must be either null or a live pointer returned by
/// [`fmi3InstantiateCoSimulation`]. `value_references` and `values`
/// must each point to `n_value_references` / `n_values` initialized
/// elements when non-null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SetFloat64(
    instance: Fmi3Instance,
    value_references: *const Fmi3ValueReferenceRaw,
    n_value_references: usize,
    values: *const f64,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let Some(plant) = (unsafe { instance_mut(instance) }) else {
        return FMI3_ERROR;
    };
    let refs = unsafe { std::slice::from_raw_parts(value_references, n_value_references) };
    let values = unsafe { std::slice::from_raw_parts(values, n_values) };
    for (value_reference, value) in refs.iter().copied().zip(values.iter().copied()) {
        if !value.is_finite() {
            return FMI3_ERROR;
        }
        match value_reference {
            VR_TIME_S => plant.time_s = value,
            VR_THROTTLE => plant.throttle = value.clamp(0.0, 1.0),
            VR_ALTITUDE_M => plant.altitude_m = value,
            VR_VELOCITY_M_S => plant.velocity_m_s = value,
            _ => return FMI3_ERROR,
        }
    }
    FMI3_OK
}

/// Read Float64 values by value reference.
///
/// Supported value references are time (`0`), throttle (`10`),
/// altitude (`20`), and velocity (`22`).
///
/// # Safety
///
/// `instance` must be either null or a live pointer returned by
/// [`fmi3InstantiateCoSimulation`]. `value_references` must point to
/// `n_value_references` initialized elements and `values` must point
/// to `n_values` writable elements when non-null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3GetFloat64(
    instance: Fmi3Instance,
    value_references: *const Fmi3ValueReferenceRaw,
    n_value_references: usize,
    values: *mut f64,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let Some(plant) = (unsafe { instance_ref(instance) }) else {
        return FMI3_ERROR;
    };
    let refs = unsafe { std::slice::from_raw_parts(value_references, n_value_references) };
    let values = unsafe { std::slice::from_raw_parts_mut(values, n_values) };
    for (slot, value_reference) in values.iter_mut().zip(refs.iter().copied()) {
        *slot = match value_reference {
            VR_TIME_S => plant.time_s,
            VR_THROTTLE => plant.throttle,
            VR_ALTITUDE_M => plant.altitude_m,
            VR_VELOCITY_M_S => plant.velocity_m_s,
            _ => return FMI3_ERROR,
        };
    }
    FMI3_OK
}

/// Set Int32 values by value reference.
///
/// The restricted point-mass export currently declares no Int32 variables,
/// so every non-empty request fails closed after pointer/count validation.
///
/// # Safety
///
/// `instance` must be either null or a live pointer returned by
/// [`fmi3InstantiateCoSimulation`]. `value_references` and `values`
/// must each point to `n_value_references` / `n_values` initialized
/// elements when non-null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SetInt32(
    instance: Fmi3Instance,
    value_references: *const Fmi3ValueReferenceRaw,
    n_value_references: usize,
    values: *const i32,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let Some(_plant) = (unsafe { instance_mut(instance) }) else {
        return FMI3_ERROR;
    };
    FMI3_ERROR
}

/// Read Int32 values by value reference.
///
/// The restricted point-mass export currently declares no Int32 variables,
/// so every non-empty request fails closed after pointer/count validation.
///
/// # Safety
///
/// `instance` must be either null or a live pointer returned by
/// [`fmi3InstantiateCoSimulation`]. `value_references` must point to
/// `n_value_references` initialized elements and `values` must point
/// to `n_values` writable elements when non-null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3GetInt32(
    instance: Fmi3Instance,
    value_references: *const Fmi3ValueReferenceRaw,
    n_value_references: usize,
    values: *mut i32,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let Some(_plant) = (unsafe { instance_ref(instance) }) else {
        return FMI3_ERROR;
    };
    FMI3_ERROR
}

/// Set UInt64 values by value reference.
///
/// Supported value reference is step (`21`) for deterministic test
/// setup.
///
/// # Safety
///
/// `instance` must be either null or a live pointer returned by
/// [`fmi3InstantiateCoSimulation`]. `value_references` and `values`
/// must each point to `n_value_references` / `n_values` initialized
/// elements when non-null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SetUInt64(
    instance: Fmi3Instance,
    value_references: *const Fmi3ValueReferenceRaw,
    n_value_references: usize,
    values: *const u64,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let Some(plant) = (unsafe { instance_mut(instance) }) else {
        return FMI3_ERROR;
    };
    let refs = unsafe { std::slice::from_raw_parts(value_references, n_value_references) };
    let values = unsafe { std::slice::from_raw_parts(values, n_values) };
    for (value_reference, value) in refs.iter().copied().zip(values.iter().copied()) {
        match value_reference {
            VR_STEP => plant.step = value,
            _ => return FMI3_ERROR,
        }
    }
    FMI3_OK
}

/// Read UInt64 values by value reference.
///
/// Supported value reference is step (`21`).
///
/// # Safety
///
/// `instance` must be either null or a live pointer returned by
/// [`fmi3InstantiateCoSimulation`]. `value_references` must point to
/// `n_value_references` initialized elements and `values` must point
/// to `n_values` writable elements when non-null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3GetUInt64(
    instance: Fmi3Instance,
    value_references: *const Fmi3ValueReferenceRaw,
    n_value_references: usize,
    values: *mut u64,
    n_values: usize,
) -> c_int {
    if n_value_references != n_values || value_references.is_null() || values.is_null() {
        return FMI3_ERROR;
    }
    let Some(plant) = (unsafe { instance_ref(instance) }) else {
        return FMI3_ERROR;
    };
    let refs = unsafe { std::slice::from_raw_parts(value_references, n_value_references) };
    let values = unsafe { std::slice::from_raw_parts_mut(values, n_values) };
    for (slot, value_reference) in values.iter_mut().zip(refs.iter().copied()) {
        *slot = match value_reference {
            VR_STEP => plant.step,
            _ => return FMI3_ERROR,
        };
    }
    FMI3_OK
}

/// Advance the restricted OpenBMP export plant by one fixed
/// communication step.
///
/// The model integrates altitude and vertical velocity using a
/// deterministic constant-acceleration step:
/// `a = throttle * 20 - 9.80665`.
///
/// # Safety
///
/// `instance` must be either null or a live pointer returned by
/// [`fmi3InstantiateCoSimulation`]. Output pointers must be writable
/// when non-null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3DoStep(
    instance: Fmi3Instance,
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
        || !current_communication_point.is_finite()
        || !communication_step_size.is_finite()
        || communication_step_size <= 0.0
    {
        return FMI3_ERROR;
    }
    let Some(plant) = (unsafe { instance_mut(instance) }) else {
        return FMI3_ERROR;
    };
    let acceleration_m_s2 = plant.throttle * THROTTLE_ACCEL_M_S2 - GRAVITY_M_S2;
    plant.altitude_m += plant.velocity_m_s * communication_step_size
        + 0.5 * acceleration_m_s2 * communication_step_size * communication_step_size;
    plant.velocity_m_s += acceleration_m_s2 * communication_step_size;
    plant.time_s = current_communication_point + communication_step_size;
    plant.step = plant.step.saturating_add(1);
    unsafe {
        *event_handling_needed = 0;
        *terminate_simulation = 0;
        *early_return = 0;
        *last_successful_time = plant.time_s;
    }
    FMI3_OK
}

/// Capture a deterministic snapshot of the export plant state.
///
/// # Safety
///
/// `instance` must be either null or a live pointer returned by
/// [`fmi3InstantiateCoSimulation`]. `state` must be writable when
/// non-null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3GetFMUState(
    instance: Fmi3Instance,
    state: *mut Fmi3FmuState,
) -> c_int {
    if state.is_null() {
        return FMI3_ERROR;
    }
    let Some(plant) = (unsafe { instance_ref(instance) }) else {
        return FMI3_ERROR;
    };
    let snapshot = Box::new(OpenBmpFmiPlantSnapshot {
        plant: plant.clone(),
    });
    unsafe {
        *state = Box::into_raw(snapshot).cast();
    }
    FMI3_OK
}

/// Restore a deterministic snapshot of the export plant state.
///
/// # Safety
///
/// `instance` must be either null or a live pointer returned by
/// [`fmi3InstantiateCoSimulation`]. `state` must be null or a snapshot
/// pointer returned by [`fmi3GetFMUState`] for a compatible instance.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SetFMUState(instance: Fmi3Instance, state: Fmi3FmuState) -> c_int {
    let Some(plant) = (unsafe { instance_mut(instance) }) else {
        return FMI3_ERROR;
    };
    if state.is_null() {
        return FMI3_ERROR;
    }
    let snapshot = unsafe { &*state.cast::<OpenBmpFmiPlantSnapshot>() };
    *plant = snapshot.plant.clone();
    FMI3_OK
}

/// Free a deterministic snapshot captured by [`fmi3GetFMUState`].
///
/// # Safety
///
/// `state` must be null or point to a snapshot handle previously
/// returned by [`fmi3GetFMUState`] and not already freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3FreeFMUState(
    _instance: Fmi3Instance,
    state: *mut Fmi3FmuState,
) -> c_int {
    if state.is_null() {
        return FMI3_ERROR;
    }
    let snapshot = unsafe { *state };
    if !snapshot.is_null() {
        unsafe {
            drop(Box::from_raw(snapshot.cast::<OpenBmpFmiPlantSnapshot>()));
            *state = std::ptr::null_mut();
        }
    }
    FMI3_OK
}

/// Terminate the restricted export instance.
#[unsafe(no_mangle)]
pub extern "C" fn fmi3Terminate(instance: Fmi3Instance) -> c_int {
    if instance.is_null() {
        FMI3_ERROR
    } else {
        FMI3_OK
    }
}

/// Free the restricted export instance.
///
/// # Safety
///
/// `instance` must be null or a pointer returned by
/// [`fmi3InstantiateCoSimulation`] and not already freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3FreeInstance(instance: Fmi3Instance) {
    if !instance.is_null() {
        unsafe {
            drop(Box::from_raw(instance.cast::<OpenBmpFmiPlantInstance>()));
        }
    }
}

unsafe fn instance_mut(instance: Fmi3Instance) -> Option<&'static mut OpenBmpFmiPlantInstance> {
    if instance.is_null() {
        None
    } else {
        Some(unsafe { &mut *instance.cast::<OpenBmpFmiPlantInstance>() })
    }
}

unsafe fn instance_ref(instance: Fmi3Instance) -> Option<&'static OpenBmpFmiPlantInstance> {
    if instance.is_null() {
        None
    } else {
        Some(unsafe { &*instance.cast::<OpenBmpFmiPlantInstance>() })
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    const POINT_MASS_TRACE_TOLERANCE: &str =
        include_str!("../tests/expected/export-point-mass-trace.toml");

    #[derive(Clone, Copy, Debug)]
    struct TraceToleranceCase {
        throttle: f64,
        step_s: f64,
        steps: usize,
        absolute_tolerance: f64,
        relative_tolerance: f64,
    }

    #[derive(Clone, Copy, Debug)]
    struct NativePointMassState {
        time_s: f64,
        altitude_m: f64,
        velocity_m_s: f64,
        step: u64,
    }

    fn parse_trace_tolerance_case() -> TraceToleranceCase {
        let mut case_name = None;
        let mut throttle = None;
        let mut step_s = None;
        let mut steps = None;
        let mut absolute_tolerance = None;
        let mut relative_tolerance = None;

        for raw_line in POINT_MASS_TRACE_TOLERANCE.lines() {
            let line = raw_line
                .split_once('#')
                .map_or(raw_line, |(value, _comment)| value)
                .trim();
            if line.is_empty() {
                continue;
            }
            let (key, value) = line.split_once('=').unwrap();
            let key = key.trim();
            let value = value.trim();
            match key {
                "case" => case_name = Some(unquote_toml_string(value)),
                "throttle" => throttle = Some(value.parse::<f64>().unwrap()),
                "step_s" => step_s = Some(value.parse::<f64>().unwrap()),
                "steps" => steps = Some(value.parse::<usize>().unwrap()),
                "absolute_tolerance" => absolute_tolerance = Some(value.parse::<f64>().unwrap()),
                "relative_tolerance" => relative_tolerance = Some(value.parse::<f64>().unwrap()),
                _ => assert!(key.starts_with('_'), "unexpected tolerance key {key}"),
            }
        }

        assert_eq!(case_name.unwrap(), "exported-fmi-point-mass-trace");
        let parsed = TraceToleranceCase {
            throttle: throttle.unwrap(),
            step_s: step_s.unwrap(),
            steps: steps.unwrap(),
            absolute_tolerance: absolute_tolerance.unwrap(),
            relative_tolerance: relative_tolerance.unwrap(),
        };
        assert!((0.0..=1.0).contains(&parsed.throttle));
        assert!(parsed.step_s > 0.0);
        assert!(parsed.steps > 0);
        assert!(parsed.absolute_tolerance > 0.0);
        assert!(parsed.relative_tolerance > 0.0);
        parsed
    }

    fn unquote_toml_string(value: &str) -> &str {
        value
            .strip_prefix('"')
            .and_then(|suffix| suffix.strip_suffix('"'))
            .unwrap_or(value)
    }

    fn native_point_mass_trace(
        throttle: f64,
        step_s: f64,
        steps: usize,
    ) -> Vec<NativePointMassState> {
        let mut state = NativePointMassState {
            time_s: 0.0,
            altitude_m: 0.0,
            velocity_m_s: 0.0,
            step: 0,
        };
        let acceleration_m_s2 = throttle.clamp(0.0, 1.0) * THROTTLE_ACCEL_M_S2 - GRAVITY_M_S2;
        let mut trace = Vec::with_capacity(steps);

        for _ in 0..steps {
            state.altitude_m +=
                state.velocity_m_s * step_s + 0.5 * acceleration_m_s2 * step_s * step_s;
            state.velocity_m_s += acceleration_m_s2 * step_s;
            state.time_s += step_s;
            state.step = state.step.saturating_add(1);
            trace.push(state);
        }

        trace
    }

    fn assert_close_to_trace(
        metric: &str,
        step: u64,
        actual: f64,
        expected: f64,
        tolerance: TraceToleranceCase,
    ) {
        let diff = (actual - expected).abs();
        let tolerance_bound = tolerance
            .absolute_tolerance
            .max(tolerance.relative_tolerance * expected.abs().max(1.0));
        assert!(
            diff <= tolerance_bound,
            "{metric} step {step}: actual {actual:e}, expected {expected:e}, diff {diff:e}, bound {tolerance_bound:e}"
        );
    }

    #[test]
    fn exported_fmi_symbols_step_deterministic_point_mass_state() {
        let instance = fmi3InstantiateCoSimulation(
            std::ptr::null(),
            std::ptr::null(),
            std::ptr::null(),
            0,
            0,
            0,
            0,
            std::ptr::null(),
            0,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        );
        assert!(!instance.is_null());
        unsafe {
            assert_eq!(
                fmi3EnterInitializationMode(instance, 0, 0.0, 0.0, 0, 0.0),
                FMI3_OK
            );
            assert_eq!(fmi3ExitInitializationMode(instance), FMI3_OK);
            let throttle_ref = [VR_THROTTLE];
            let throttle = [1.0];
            assert_eq!(
                fmi3SetFloat64(
                    instance,
                    throttle_ref.as_ptr(),
                    throttle_ref.len(),
                    throttle.as_ptr(),
                    throttle.len(),
                ),
                FMI3_OK
            );
            let mut event_handling_needed = 99;
            let mut terminate_simulation = 99;
            let mut early_return = 99;
            let mut last_successful_time = -1.0;
            assert_eq!(
                fmi3DoStep(
                    instance,
                    0.0,
                    0.5,
                    1,
                    &mut event_handling_needed,
                    &mut terminate_simulation,
                    &mut early_return,
                    &mut last_successful_time,
                ),
                FMI3_OK
            );
            assert_eq!(event_handling_needed, 0);
            assert_eq!(terminate_simulation, 0);
            assert_eq!(early_return, 0);
            assert_eq!(last_successful_time.to_bits(), 0.5_f64.to_bits());

            let float_refs = [VR_TIME_S, VR_ALTITUDE_M, VR_VELOCITY_M_S];
            let mut floats = [0.0; 3];
            assert_eq!(
                fmi3GetFloat64(
                    instance,
                    float_refs.as_ptr(),
                    float_refs.len(),
                    floats.as_mut_ptr(),
                    floats.len(),
                ),
                FMI3_OK
            );
            assert_eq!(floats[0].to_bits(), 0.5_f64.to_bits());
            assert_eq!(floats[1].to_bits(), 1.274_168_75_f64.to_bits());
            assert_eq!(floats[2].to_bits(), 5.096_675_f64.to_bits());

            let step_ref = [VR_STEP];
            let mut step = [0_u64];
            assert_eq!(
                fmi3GetUInt64(
                    instance,
                    step_ref.as_ptr(),
                    step_ref.len(),
                    step.as_mut_ptr(),
                    step.len(),
                ),
                FMI3_OK
            );
            assert_eq!(step, [1]);
            assert_eq!(fmi3Terminate(instance), FMI3_OK);
            fmi3FreeInstance(instance);
        }
    }

    #[test]
    fn exported_fmi_symbols_match_native_point_mass_trace_tolerance() {
        let tolerance = parse_trace_tolerance_case();
        let expected_trace =
            native_point_mass_trace(tolerance.throttle, tolerance.step_s, tolerance.steps);
        let instance = fmi3InstantiateCoSimulation(
            std::ptr::null(),
            std::ptr::null(),
            std::ptr::null(),
            0,
            0,
            0,
            0,
            std::ptr::null(),
            0,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        );
        assert!(!instance.is_null());
        unsafe {
            assert_eq!(
                fmi3EnterInitializationMode(instance, 0, 0.0, 0.0, 0, 0.0),
                FMI3_OK
            );
            assert_eq!(fmi3ExitInitializationMode(instance), FMI3_OK);
            let throttle_ref = [VR_THROTTLE];
            let throttle = [tolerance.throttle];
            assert_eq!(
                fmi3SetFloat64(
                    instance,
                    throttle_ref.as_ptr(),
                    throttle_ref.len(),
                    throttle.as_ptr(),
                    throttle.len(),
                ),
                FMI3_OK
            );

            let float_refs = [VR_TIME_S, VR_ALTITUDE_M, VR_VELOCITY_M_S];
            let step_ref = [VR_STEP];
            let mut current_communication_point = 0.0;

            for expected in expected_trace {
                let mut event_handling_needed = 99;
                let mut terminate_simulation = 99;
                let mut early_return = 99;
                let mut last_successful_time = -1.0;
                assert_eq!(
                    fmi3DoStep(
                        instance,
                        current_communication_point,
                        tolerance.step_s,
                        1,
                        &mut event_handling_needed,
                        &mut terminate_simulation,
                        &mut early_return,
                        &mut last_successful_time,
                    ),
                    FMI3_OK
                );
                assert_eq!(event_handling_needed, 0);
                assert_eq!(terminate_simulation, 0);
                assert_eq!(early_return, 0);

                let mut floats = [0.0; 3];
                assert_eq!(
                    fmi3GetFloat64(
                        instance,
                        float_refs.as_ptr(),
                        float_refs.len(),
                        floats.as_mut_ptr(),
                        floats.len(),
                    ),
                    FMI3_OK
                );
                let mut step = [0_u64];
                assert_eq!(
                    fmi3GetUInt64(
                        instance,
                        step_ref.as_ptr(),
                        step_ref.len(),
                        step.as_mut_ptr(),
                        step.len(),
                    ),
                    FMI3_OK
                );

                assert_close_to_trace(
                    "last_successful_time_s",
                    expected.step,
                    last_successful_time,
                    expected.time_s,
                    tolerance,
                );
                assert_close_to_trace(
                    "time_s",
                    expected.step,
                    floats[0],
                    expected.time_s,
                    tolerance,
                );
                assert_close_to_trace(
                    "altitude_m",
                    expected.step,
                    floats[1],
                    expected.altitude_m,
                    tolerance,
                );
                assert_close_to_trace(
                    "velocity_m_s",
                    expected.step,
                    floats[2],
                    expected.velocity_m_s,
                    tolerance,
                );
                assert_eq!(step[0], expected.step);
                current_communication_point = expected.time_s;
            }

            assert_eq!(fmi3Terminate(instance), FMI3_OK);
            fmi3FreeInstance(instance);
        }
    }

    #[test]
    fn exported_fmi_state_snapshot_restore_round_trips() {
        let instance = fmi3InstantiateCoSimulation(
            std::ptr::null(),
            std::ptr::null(),
            std::ptr::null(),
            0,
            0,
            0,
            0,
            std::ptr::null(),
            0,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        );
        unsafe {
            let altitude_ref = [VR_ALTITUDE_M];
            let altitude = [12.0];
            assert_eq!(
                fmi3SetFloat64(
                    instance,
                    altitude_ref.as_ptr(),
                    altitude_ref.len(),
                    altitude.as_ptr(),
                    altitude.len(),
                ),
                FMI3_OK
            );
            let mut state = std::ptr::null_mut();
            assert_eq!(fmi3GetFMUState(instance, &mut state), FMI3_OK);
            assert!(!state.is_null());

            let changed = [42.0];
            assert_eq!(
                fmi3SetFloat64(
                    instance,
                    altitude_ref.as_ptr(),
                    altitude_ref.len(),
                    changed.as_ptr(),
                    changed.len(),
                ),
                FMI3_OK
            );
            assert_eq!(fmi3SetFMUState(instance, state), FMI3_OK);

            let mut restored = [0.0];
            assert_eq!(
                fmi3GetFloat64(
                    instance,
                    altitude_ref.as_ptr(),
                    altitude_ref.len(),
                    restored.as_mut_ptr(),
                    restored.len(),
                ),
                FMI3_OK
            );
            assert_eq!(restored[0].to_bits(), 12.0_f64.to_bits());
            assert_eq!(fmi3FreeFMUState(instance, &mut state), FMI3_OK);
            assert!(state.is_null());
            fmi3FreeInstance(instance);
        }
    }
}
