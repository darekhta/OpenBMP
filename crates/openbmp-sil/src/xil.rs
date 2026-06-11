//! In-memory implementation of the abstract XIL-shaped bridge ports.
//!
//! This module is a native host testbench adapter: it records port operations
//! for evidence and unit tests, but it is not an ASAM-conformant implementation
//! and does not speak any physical bus protocol.

use std::collections::{BTreeMap, BTreeSet};

use openbmp_bridge::{
    CaptureHandle, CaptureTrigger, EesPort, ElectricalErrorType, FaultWindow, MaPort, PinId,
    SignalDescription, SignalId, SignalMapping, StimHandle, TestbenchLifecycle, TestbenchState,
    TestbenchTransition, XilPortError, XilValue,
};

use crate::{BusFrameEntry, StimulusRecord};

/// Capture request stored by the in-memory XIL bench.
#[derive(Clone, Debug, PartialEq)]
pub struct XilCaptureRecord {
    /// Capture handle.
    pub handle: CaptureHandle,
    /// Signals captured.
    pub signals: Vec<SignalId>,
    /// Capture trigger.
    pub trigger: CaptureTrigger,
    /// Capture downsampling factor.
    pub downsampling: u32,
}

/// Signal-generator request stored by the in-memory XIL bench.
#[derive(Clone, Debug, PartialEq)]
pub struct XilGeneratorRecord {
    /// Signal-generator handle.
    pub handle: StimHandle,
    /// Target signal.
    pub signal: SignalId,
    /// Generator description.
    pub description: SignalDescription,
}

/// Armed EES error stored by the in-memory XIL bench.
#[derive(Clone, Debug, PartialEq)]
pub struct XilErrorRecord {
    /// Target pin.
    pub pin: PinId,
    /// Armed electrical error.
    pub error: ElectricalErrorType,
    /// Active window.
    pub duration: FaultWindow,
}

/// In-memory host adapter for the abstract XIL-shaped port traits.
#[derive(Clone, Debug, Default)]
pub struct InMemoryXilBench {
    lifecycle: TestbenchLifecycle,
    mapping: SignalMapping,
    signals: BTreeMap<SignalId, XilValue>,
    pins: BTreeSet<PinId>,
    captures: BTreeMap<CaptureHandle, XilCaptureRecord>,
    generators: BTreeMap<StimHandle, XilGeneratorRecord>,
    errors: BTreeMap<PinId, XilErrorRecord>,
    lifecycle_log: Vec<TestbenchState>,
    signal_writes: Vec<(SignalId, XilValue)>,
    next_capture: u64,
    next_generator: u64,
}

impl InMemoryXilBench {
    /// Construct an empty in-memory XIL bench.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Borrow the current lifecycle state.
    #[must_use]
    pub fn state(&self) -> TestbenchState {
        self.lifecycle.state()
    }

    /// Apply one lifecycle transition and record it for evidence.
    ///
    /// # Errors
    ///
    /// Returns [`XilPortError`] when the transition is invalid.
    pub fn transition(
        &mut self,
        transition: TestbenchTransition,
    ) -> Result<TestbenchState, XilPortError> {
        let state = self.lifecycle.transition(transition)?;
        self.lifecycle_log.push(state);
        Ok(state)
    }

    /// Replace the symbolic signal mapping.
    pub fn set_mapping(&mut self, mapping: SignalMapping) {
        self.mapping = mapping;
    }

    /// Borrow the symbolic signal mapping.
    #[must_use]
    pub const fn mapping(&self) -> &SignalMapping {
        &self.mapping
    }

    /// Declare a readable/writable signal with an initial value.
    pub fn declare_signal(&mut self, id: impl Into<SignalId>, value: XilValue) {
        self.signals.insert(id.into(), value);
    }

    /// Declare an EES pin.
    pub fn declare_pin(&mut self, id: impl Into<PinId>) {
        self.pins.insert(id.into());
    }

    /// Borrow the capture records.
    #[must_use]
    pub fn captures(&self) -> &BTreeMap<CaptureHandle, XilCaptureRecord> {
        &self.captures
    }

    /// Borrow the signal-generator records.
    #[must_use]
    pub fn generators(&self) -> &BTreeMap<StimHandle, XilGeneratorRecord> {
        &self.generators
    }

    /// Borrow the armed EES errors.
    #[must_use]
    pub fn errors(&self) -> &BTreeMap<PinId, XilErrorRecord> {
        &self.errors
    }

    /// Convert XIL port actions into SIL stimulus evidence records.
    #[must_use]
    pub fn stimulus_records(&self) -> Vec<StimulusRecord> {
        let mut records = Vec::new();
        for (signal, value) in &self.signal_writes {
            records.push(StimulusRecord {
                kind: "xil.ma.write".to_owned(),
                target: signal.as_str().to_owned(),
                summary: format!("write {}", xil_value_summary(value)),
            });
        }
        for record in self.captures.values() {
            records.push(StimulusRecord {
                kind: "xil.ma.capture".to_owned(),
                target: format!(
                    "capture.{}",
                    record
                        .signals
                        .iter()
                        .map(SignalId::as_str)
                        .collect::<Vec<_>>()
                        .join(",")
                ),
                summary: format!(
                    "handle={} downsampling={}",
                    record.handle.value(),
                    record.downsampling
                ),
            });
        }
        for record in self.generators.values() {
            records.push(StimulusRecord {
                kind: "xil.ma.generator".to_owned(),
                target: record.signal.as_str().to_owned(),
                summary: format!("handle={}", record.handle.value()),
            });
        }
        for record in self.errors.values() {
            records.push(StimulusRecord {
                kind: "xil.ees.error".to_owned(),
                target: record.pin.as_str().to_owned(),
                summary: format!("{:?}", record.error),
            });
        }
        records
    }

    /// Convert XIL lifecycle and port actions into SIL bus-frame evidence.
    #[must_use]
    pub fn bus_frames(&self) -> Vec<BusFrameEntry> {
        let mut frames = Vec::new();
        for (step, state) in self.lifecycle_log.iter().enumerate() {
            frames.push(BusFrameEntry {
                stream: "xil.lifecycle".to_owned(),
                subject: "testbench".to_owned(),
                value: format!("{state:?}"),
                time_s: step as f64,
                step: u64::try_from(step).unwrap_or(u64::MAX),
            });
        }
        for (index, (signal, value)) in self.signal_writes.iter().enumerate() {
            frames.push(BusFrameEntry {
                stream: "xil.ma.write".to_owned(),
                subject: signal.as_str().to_owned(),
                value: xil_value_summary(value),
                time_s: index as f64,
                step: u64::try_from(index).unwrap_or(u64::MAX),
            });
        }
        for record in self.errors.values() {
            frames.push(BusFrameEntry {
                stream: "xil.ees.error".to_owned(),
                subject: record.pin.as_str().to_owned(),
                value: format!("{:?}", record.error),
                time_s: record.duration.start_step as f64,
                step: record.duration.start_step,
            });
        }
        frames
    }
}

impl MaPort for InMemoryXilBench {
    fn read(&self, id: &SignalId) -> Result<XilValue, XilPortError> {
        self.signals
            .get(id)
            .cloned()
            .ok_or_else(|| XilPortError::UnknownSignal {
                signal: id.as_str().to_owned(),
            })
    }

    fn write(&mut self, id: SignalId, value: XilValue) -> Result<(), XilPortError> {
        let slot = self
            .signals
            .get_mut(&id)
            .ok_or_else(|| XilPortError::UnknownSignal {
                signal: id.as_str().to_owned(),
            })?;
        *slot = value.clone();
        self.signal_writes.push((id, value));
        Ok(())
    }

    fn create_capture(
        &mut self,
        signals: &[SignalId],
        trigger: CaptureTrigger,
        downsampling: u32,
    ) -> Result<CaptureHandle, XilPortError> {
        if downsampling == 0 {
            return Err(XilPortError::InvalidDownsampling);
        }
        for signal in signals {
            if !self.signals.contains_key(signal) {
                return Err(XilPortError::UnknownSignal {
                    signal: signal.as_str().to_owned(),
                });
            }
        }
        let handle = CaptureHandle::new(self.next_capture);
        self.next_capture = self.next_capture.saturating_add(1);
        self.captures.insert(
            handle,
            XilCaptureRecord {
                handle,
                signals: signals.to_vec(),
                trigger,
                downsampling,
            },
        );
        Ok(handle)
    }

    fn create_signal_generator(
        &mut self,
        id: SignalId,
        description: SignalDescription,
    ) -> Result<StimHandle, XilPortError> {
        if !self.signals.contains_key(&id) {
            return Err(XilPortError::UnknownSignal {
                signal: id.as_str().to_owned(),
            });
        }
        let handle = StimHandle::new(self.next_generator);
        self.next_generator = self.next_generator.saturating_add(1);
        self.generators.insert(
            handle,
            XilGeneratorRecord {
                handle,
                signal: id,
                description,
            },
        );
        Ok(handle)
    }
}

impl EesPort for InMemoryXilBench {
    fn set_error(
        &mut self,
        pin: PinId,
        error: ElectricalErrorType,
        duration: FaultWindow,
    ) -> Result<(), XilPortError> {
        if !self.pins.contains(&pin) {
            return Err(XilPortError::UnknownPin {
                pin: pin.as_str().to_owned(),
            });
        }
        self.errors.insert(
            pin.clone(),
            XilErrorRecord {
                pin,
                error,
                duration,
            },
        );
        Ok(())
    }

    fn clear_error(&mut self, pin: &PinId) -> Result<(), XilPortError> {
        if !self.pins.contains(pin) {
            return Err(XilPortError::UnknownPin {
                pin: pin.as_str().to_owned(),
            });
        }
        self.errors.remove(pin);
        Ok(())
    }
}

fn xil_value_summary(value: &XilValue) -> String {
    match value {
        XilValue::Float64(value) => format!("{value:.12}"),
        XilValue::UInt64(value) => value.to_string(),
        XilValue::Bool(value) => value.to_string(),
        XilValue::Text(value) => value.clone(),
        XilValue::Bytes(value) => format!("{} byte(s)", value.len()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openbmp_bridge::{SignalBinding, TestbenchTransition};

    #[test]
    fn in_memory_xil_bench_reads_writes_captures_and_generates() {
        let mut bench = InMemoryXilBench::new();
        bench.declare_signal("fc.imu.ax", XilValue::Float64(1.0));
        let mut mapping = SignalMapping::default();
        mapping.insert(SignalBinding::new(
            SignalId::from("fc.imu.ax"),
            "bridge.sensor.imu_accel_x",
            Some("m/s2".to_owned()),
            1.0,
            0.0,
        ));
        bench.set_mapping(mapping);

        bench
            .write(SignalId::from("fc.imu.ax"), XilValue::Float64(2.0))
            .expect("write signal");
        assert_eq!(
            bench.read(&SignalId::from("fc.imu.ax")),
            Ok(XilValue::Float64(2.0))
        );
        let capture = bench
            .create_capture(&[SignalId::from("fc.imu.ax")], CaptureTrigger::Immediate, 1)
            .expect("create capture");
        let generator = bench
            .create_signal_generator(
                SignalId::from("fc.imu.ax"),
                SignalDescription::Constant {
                    value: XilValue::Float64(3.0),
                },
            )
            .expect("create generator");

        assert_eq!(capture.value(), 0);
        assert_eq!(generator.value(), 0);
        assert_eq!(bench.captures().len(), 1);
        assert_eq!(bench.generators().len(), 1);
        assert_eq!(bench.stimulus_records().len(), 3);
    }

    #[test]
    fn in_memory_xil_bench_records_ees_errors_and_lifecycle_frames() {
        let mut bench = InMemoryXilBench::new();
        bench.declare_pin("imu.vcc");

        bench
            .transition(TestbenchTransition::Initialize)
            .expect("initialize");
        bench
            .transition(TestbenchTransition::Connect)
            .expect("connect");
        bench.transition(TestbenchTransition::Start).expect("start");
        bench
            .set_error(
                PinId::from("imu.vcc"),
                ElectricalErrorType::ShortToGround,
                FaultWindow::new(10, Some(12)).expect("window"),
            )
            .expect("set EES error");

        assert_eq!(bench.state(), TestbenchState::Running);
        assert_eq!(bench.errors().len(), 1);
        assert!(
            bench
                .bus_frames()
                .iter()
                .any(|frame| frame.stream == "xil.lifecycle" && frame.value == "Running")
        );
        assert!(
            bench
                .stimulus_records()
                .iter()
                .any(|record| record.kind == "xil.ees.error")
        );
    }

    #[test]
    fn in_memory_xil_bench_rejects_unknown_signal_or_pin() {
        let mut bench = InMemoryXilBench::new();

        assert!(matches!(
            bench.write(SignalId::from("missing"), XilValue::Bool(true)),
            Err(XilPortError::UnknownSignal { .. })
        ));
        assert!(matches!(
            bench.set_error(
                PinId::from("missing"),
                ElectricalErrorType::Open,
                FaultWindow::new(0, None).expect("window")
            ),
            Err(XilPortError::UnknownPin { .. })
        ));
    }
}
