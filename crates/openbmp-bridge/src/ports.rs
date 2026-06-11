//! ASAM-XIL-shaped abstract port traits and lifecycle helpers.
//!
//! These types intentionally model the *shape* of an XIL bench API without
//! claiming ASAM conformance. They operate on symbolic signal and pin ids so
//! the same test case can be mapped onto an in-process bench, a socket-backed
//! SIL binary, or a downstream lab adapter.

use std::collections::BTreeMap;

use thiserror::Error;

/// Symbolic model-access signal identifier.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct SignalId(String);

impl SignalId {
    /// Construct a signal id from a caller-owned string.
    #[must_use]
    pub fn new(id: impl Into<String>) -> Self {
        Self(id.into())
    }

    /// Borrow the signal id as a string slice.
    #[must_use]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<&str> for SignalId {
    fn from(value: &str) -> Self {
        Self::new(value)
    }
}

impl From<String> for SignalId {
    fn from(value: String) -> Self {
        Self::new(value)
    }
}

/// Symbolic electrical-error-simulation pin identifier.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct PinId(String);

impl PinId {
    /// Construct a pin id from a caller-owned string.
    #[must_use]
    pub fn new(id: impl Into<String>) -> Self {
        Self(id.into())
    }

    /// Borrow the pin id as a string slice.
    #[must_use]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<&str> for PinId {
    fn from(value: &str) -> Self {
        Self::new(value)
    }
}

impl From<String> for PinId {
    fn from(value: String) -> Self {
        Self::new(value)
    }
}

/// Port value exchanged through the abstract XIL API.
#[derive(Clone, Debug, PartialEq)]
pub enum XilValue {
    /// 64-bit floating-point value.
    Float64(f64),
    /// Unsigned integer value.
    UInt64(u64),
    /// Boolean value.
    Bool(bool),
    /// Text value.
    Text(String),
    /// Opaque byte payload.
    Bytes(Vec<u8>),
}

/// Symbolic mapping from a test-case signal name to a bench address.
#[derive(Clone, Debug, PartialEq)]
pub struct SignalBinding {
    /// Symbolic test-case signal id.
    pub signal: SignalId,
    /// Backend-specific bench address.
    pub address: String,
    /// Optional unit label.
    pub unit: Option<String>,
    /// Engineering conversion scale.
    pub scale: f64,
    /// Engineering conversion offset.
    pub offset: f64,
}

impl SignalBinding {
    /// Construct a mapping entry.
    #[must_use]
    pub fn new(
        signal: SignalId,
        address: impl Into<String>,
        unit: Option<String>,
        scale: f64,
        offset: f64,
    ) -> Self {
        Self {
            signal,
            address: address.into(),
            unit,
            scale,
            offset,
        }
    }

    /// Convert a raw backend scalar into the mapped engineering scalar.
    #[must_use]
    pub fn engineering_value(&self, raw: f64) -> f64 {
        raw * self.scale + self.offset
    }
}

/// Symbolic signal-name mapping for one test bench.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SignalMapping {
    entries: BTreeMap<SignalId, SignalBinding>,
}

impl SignalMapping {
    /// Insert or replace one signal binding.
    pub fn insert(&mut self, binding: SignalBinding) -> Option<SignalBinding> {
        self.entries.insert(binding.signal.clone(), binding)
    }

    /// Resolve one symbolic signal binding.
    #[must_use]
    pub fn resolve(&self, signal: &SignalId) -> Option<&SignalBinding> {
        self.entries.get(signal)
    }

    /// Return the number of mapped signals.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Return whether the mapping is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// Capture trigger description for a model-access port.
#[derive(Clone, Debug, PartialEq)]
pub enum CaptureTrigger {
    /// Capture immediately when armed.
    Immediate,
    /// Capture when a simulation step is reached.
    Step {
        /// Trigger step.
        step: u64,
    },
    /// Capture when a signal reaches or crosses a threshold.
    Threshold {
        /// Signal to watch.
        signal: SignalId,
        /// Trigger threshold.
        threshold: f64,
    },
}

/// Signal generator description for a model-access port.
#[derive(Clone, Debug, PartialEq)]
pub enum SignalDescription {
    /// Hold a constant value.
    Constant {
        /// Constant value.
        value: XilValue,
    },
    /// Step from an initial value to a final value at a given step.
    Step {
        /// Value before `at_step`.
        initial: XilValue,
        /// Value from `at_step` onward.
        final_value: XilValue,
        /// Trigger step.
        at_step: u64,
    },
}

/// Opaque capture handle returned by a model-access port.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct CaptureHandle(u64);

impl CaptureHandle {
    /// Construct a capture handle from a stable integer.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Return the integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Opaque signal-generator handle returned by a model-access port.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct StimHandle(u64);

impl StimHandle {
    /// Construct a signal-generator handle from a stable integer.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Return the integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Active window for an electrical-error-simulation fault.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FaultWindow {
    /// First active step.
    pub start_step: u64,
    /// Last active step, inclusive. `None` means no upper bound.
    pub end_step: Option<u64>,
}

impl FaultWindow {
    /// Construct a fault window.
    ///
    /// # Errors
    ///
    /// Returns [`XilPortError::InvalidWindow`] when `end_step < start_step`.
    pub fn new(start_step: u64, end_step: Option<u64>) -> Result<Self, XilPortError> {
        if let Some(end_step) = end_step
            && end_step < start_step
        {
            return Err(XilPortError::InvalidWindow {
                start_step,
                end_step,
            });
        }
        Ok(Self {
            start_step,
            end_step,
        })
    }

    /// Return whether `step` is inside this active window.
    #[must_use]
    pub fn contains(self, step: u64) -> bool {
        step >= self.start_step && self.end_step.is_none_or(|end_step| step <= end_step)
    }
}

/// Electrical-error-simulation fault type.
#[derive(Clone, Debug, PartialEq)]
pub enum ElectricalErrorType {
    /// Open circuit.
    Open,
    /// Short to ground.
    ShortToGround,
    /// Short to battery.
    ShortToBattery,
    /// Short to another symbolic pin.
    ShortToPin(PinId),
    /// Stuck value at the abstract boundary.
    StuckValue(XilValue),
}

/// Error returned by XIL-shaped port helpers.
#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum XilPortError {
    /// Signal is unknown to the port implementation.
    #[error("unknown XIL signal `{signal}`")]
    UnknownSignal {
        /// Missing signal id.
        signal: String,
    },
    /// Pin is unknown to the port implementation.
    #[error("unknown XIL pin `{pin}`")]
    UnknownPin {
        /// Missing pin id.
        pin: String,
    },
    /// Downsampling value was zero.
    #[error("capture downsampling must be positive")]
    InvalidDownsampling,
    /// Fault window end step was before start step.
    #[error("invalid fault window: end_step {end_step} is before start_step {start_step}")]
    InvalidWindow {
        /// First active step.
        start_step: u64,
        /// Invalid end step.
        end_step: u64,
    },
    /// Testbench lifecycle transition is invalid.
    #[error("invalid testbench transition from {from:?} using {transition:?}")]
    InvalidLifecycleTransition {
        /// Current state.
        from: TestbenchState,
        /// Requested transition.
        transition: TestbenchTransition,
    },
}

/// Model-access port: symbolic signal read/write, capture, and stimulation.
pub trait MaPort {
    /// Read a symbolic signal.
    ///
    /// # Errors
    ///
    /// Returns [`XilPortError`] when the signal cannot be read.
    fn read(&self, id: &SignalId) -> Result<XilValue, XilPortError>;

    /// Write a symbolic signal or parameter.
    ///
    /// # Errors
    ///
    /// Returns [`XilPortError`] when the signal cannot be written.
    fn write(&mut self, id: SignalId, value: XilValue) -> Result<(), XilPortError>;

    /// Create a signal capture.
    ///
    /// # Errors
    ///
    /// Returns [`XilPortError`] when the request is invalid.
    fn create_capture(
        &mut self,
        signals: &[SignalId],
        trigger: CaptureTrigger,
        downsampling: u32,
    ) -> Result<CaptureHandle, XilPortError>;

    /// Create a signal generator.
    ///
    /// # Errors
    ///
    /// Returns [`XilPortError`] when the request is invalid.
    fn create_signal_generator(
        &mut self,
        id: SignalId,
        description: SignalDescription,
    ) -> Result<StimHandle, XilPortError>;
}

/// Electrical-error-simulation port.
pub trait EesPort {
    /// Arm an electrical error on a symbolic pin.
    ///
    /// # Errors
    ///
    /// Returns [`XilPortError`] when the pin or window is invalid.
    fn set_error(
        &mut self,
        pin: PinId,
        error: ElectricalErrorType,
        duration: FaultWindow,
    ) -> Result<(), XilPortError>;

    /// Clear an electrical error on a symbolic pin.
    ///
    /// # Errors
    ///
    /// Returns [`XilPortError`] when the pin is unknown.
    fn clear_error(&mut self, pin: &PinId) -> Result<(), XilPortError>;
}

/// Testbench lifecycle state.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum TestbenchState {
    /// Object was created but not initialized.
    Uninitialized,
    /// Testbench is initialized but not connected to a backend.
    Disconnected,
    /// Testbench is connected and stopped.
    Stopped,
    /// Testbench is connected and running.
    Running,
}

/// Testbench lifecycle transition.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum TestbenchTransition {
    /// Initialize the testbench.
    Initialize,
    /// Connect to a backend.
    Connect,
    /// Start execution.
    Start,
    /// Stop execution.
    Stop,
    /// Disconnect from the backend.
    Disconnect,
    /// Return to the uninitialized state.
    Reset,
}

/// Deterministic lifecycle FSM for an abstract XIL testbench.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TestbenchLifecycle {
    state: TestbenchState,
}

impl Default for TestbenchLifecycle {
    fn default() -> Self {
        Self {
            state: TestbenchState::Uninitialized,
        }
    }
}

impl TestbenchLifecycle {
    /// Return the current lifecycle state.
    #[must_use]
    pub const fn state(&self) -> TestbenchState {
        self.state
    }

    /// Apply one lifecycle transition.
    ///
    /// # Errors
    ///
    /// Returns [`XilPortError::InvalidLifecycleTransition`] when the requested
    /// transition is not valid from the current state.
    pub fn transition(
        &mut self,
        transition: TestbenchTransition,
    ) -> Result<TestbenchState, XilPortError> {
        let next = match (self.state, transition) {
            (TestbenchState::Uninitialized, TestbenchTransition::Initialize) => {
                TestbenchState::Disconnected
            }
            (TestbenchState::Disconnected, TestbenchTransition::Connect) => TestbenchState::Stopped,
            (TestbenchState::Stopped, TestbenchTransition::Start) => TestbenchState::Running,
            (TestbenchState::Running, TestbenchTransition::Stop) => TestbenchState::Stopped,
            (TestbenchState::Stopped, TestbenchTransition::Disconnect) => {
                TestbenchState::Disconnected
            }
            (TestbenchState::Disconnected, TestbenchTransition::Reset) => {
                TestbenchState::Uninitialized
            }
            (from, transition) => {
                return Err(XilPortError::InvalidLifecycleTransition { from, transition });
            }
        };
        self.state = next;
        Ok(next)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mapping_resolves_signal_and_scales_raw_value() {
        let mut mapping = SignalMapping::default();
        let signal = SignalId::from("fc.imu.ax");
        mapping.insert(SignalBinding::new(
            signal.clone(),
            "bridge.sensor.imu_accel_x",
            Some("m/s2".to_owned()),
            2.0,
            -1.0,
        ));

        let binding = mapping.resolve(&signal).expect("binding present");

        assert_eq!(binding.address, "bridge.sensor.imu_accel_x");
        assert_eq!(binding.engineering_value(3.0).to_bits(), 5.0_f64.to_bits());
    }

    #[test]
    fn fault_window_validates_order_and_contains_steps() {
        let window = FaultWindow::new(4, Some(6)).expect("valid window");

        assert!(!window.contains(3));
        assert!(window.contains(4));
        assert!(window.contains(6));
        assert!(!window.contains(7));
        assert!(matches!(
            FaultWindow::new(6, Some(4)),
            Err(XilPortError::InvalidWindow { .. })
        ));
    }

    #[test]
    fn lifecycle_rejects_invalid_transition() {
        let mut lifecycle = TestbenchLifecycle::default();

        assert!(matches!(
            lifecycle.transition(TestbenchTransition::Start),
            Err(XilPortError::InvalidLifecycleTransition { .. })
        ));
        assert_eq!(
            lifecycle.transition(TestbenchTransition::Initialize),
            Ok(TestbenchState::Disconnected)
        );
        assert_eq!(
            lifecycle.transition(TestbenchTransition::Connect),
            Ok(TestbenchState::Stopped)
        );
        assert_eq!(
            lifecycle.transition(TestbenchTransition::Start),
            Ok(TestbenchState::Running)
        );
        assert_eq!(
            lifecycle.transition(TestbenchTransition::Stop),
            Ok(TestbenchState::Stopped)
        );
    }
}
