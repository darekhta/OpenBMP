//! Target-portable PIL shim for one flight-controller step.
//!
//! This crate joins the `openbmp-bridge` byte schema to
//! `openbmp-fc::fc_step` without adding any bridge, runner, scenario, or
//! transport dependency to `openbmp-fc`. Host runners and target firmware can
//! reuse this packet-level entry while the actual UART/GDB/Renode transport
//! remains outside the flight-controller crate.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![deny(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

extern crate alloc;

use alloc::vec::Vec;
use core::fmt;
use core::mem::{offset_of, size_of};

use nalgebra::Vector3;
use openbmp_bridge::{
    ActuatorCommandPacket, BridgeError, EngineCommandPacket as BridgeEngineCommandPacket,
    ImuIncrementPacket, PROTOCOL_VERSION, SensorPacket, decode, deframe, encode, encode_into,
};
use openbmp_core::{SimTime, StepIndex};
use openbmp_fc::topics::{
    BarometerSample, EffectorCommandSet, EngineCommandSet, GnssSample, ImuIncrementWindow,
    ImuInertialIncrement, ImuSample, MagnetometerSample, StarTrackerSample,
};
use openbmp_fc::{ControllerError, FcStepInput, FcStepOutput, FlightController, fc_step};

/// Recommended target symbol name for the PIL mailbox instance.
///
/// This crate does not force a linker export because the workspace forbids
/// unsafe symbol attributes. A downstream board or emulator wrapper can export
/// this name while storing a [`PilMailbox`] value with the reported layout.
pub const OPENBMP_PIL_MAILBOX_SYMBOL: &str = "OPENBMP_PIL_MAILBOX";

/// Recommended target symbol name for the mailbox step entry.
///
/// Host-side GDB/Renode helpers use this name as the default breakpoint label
/// for a downstream wrapper that calls [`PilFirmware::step_mailbox`].
pub const OPENBMP_PIL_STEP_MAILBOX_SYMBOL: &str = "openbmp_pil_step_mailbox";

/// Errors raised by the target-portable PIL step adapter.
#[derive(Debug, thiserror::Error)]
pub enum PilStepError {
    /// Incoming sensor bytes failed to decode.
    #[error("failed to decode bridge sensor packet: {0}")]
    Decode(#[source] BridgeError),
    /// Outgoing actuator command bytes failed to encode.
    #[error("failed to encode bridge actuator command packet: {0}")]
    Encode(#[source] BridgeError),
    /// The hardware-portable FC step failed.
    #[error("flight-controller step failed: {0}")]
    Controller(#[source] ControllerError),
    /// A command id cannot be represented by the bridge wire schema.
    #[error("fc {kind} id {id} exceeds bridge u32 id range")]
    CommandIdOutOfRange {
        /// Command kind, currently `effector` or `engine`.
        kind: &'static str,
        /// Out-of-range FC-side id.
        id: u64,
    },
    /// Caller-provided output storage is smaller than the encoded command.
    #[error("encoded actuator command requires {needed} bytes but output buffer holds {capacity}")]
    OutputBufferTooSmall {
        /// Encoded command length.
        needed: usize,
        /// Provided buffer length.
        capacity: usize,
    },
}

/// Summary of one framed PIL exchange.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct PilFrameStep {
    /// Bytes consumed from the incoming length-prefixed frame buffer.
    pub consumed: usize,
    /// Bytes written to the outgoing length-prefixed command frame buffer.
    pub written: usize,
}

/// Errors raised by target-side length-prefixed frame processing.
#[derive(Debug, thiserror::Error)]
pub enum PilFrameError {
    /// The incoming length-prefixed frame could not be extracted.
    #[error("failed to decode incoming PIL frame: {0}")]
    IncomingFrame(#[source] BridgeError),
    /// The outgoing command payload cannot fit in the supplied frame buffer.
    #[error("framed actuator command requires {needed} bytes but output buffer holds {capacity}")]
    OutputFrameTooSmall {
        /// Required framed output length.
        needed: usize,
        /// Provided output frame-buffer length.
        capacity: usize,
    },
    /// The encoded command payload exceeds the bridge frame-length field.
    #[error("encoded actuator command frame payload is too large: max {max} bytes, got {got}")]
    OutputPayloadTooLarge {
        /// Maximum payload bytes representable by the bridge frame prefix.
        max: usize,
        /// Produced payload bytes.
        got: usize,
    },
    /// One FC byte step failed after a valid incoming frame was extracted.
    #[error("PIL framed step failed: {0}")]
    Step(#[source] PilStepError),
}

/// Complete-frame byte I/O used by target firmware wrappers.
///
/// A UART, semihosting, or emulator-symbol adapter can implement this trait
/// by blocking until one length-prefixed bridge frame is available and by
/// writing the full response frame. This trait deliberately does not prescribe
/// interrupts, DMA, buffering, or a concrete device driver.
pub trait PilFrameIo {
    /// Transport-specific error type.
    type Error;

    /// Read one complete length-prefixed frame into `storage`.
    ///
    /// Returns the number of bytes written to `storage`.
    ///
    /// # Errors
    ///
    /// Returns the transport error if the underlying byte source fails.
    fn read_frame(&mut self, storage: &mut [u8]) -> Result<usize, Self::Error>;

    /// Write one complete length-prefixed frame.
    ///
    /// # Errors
    ///
    /// Returns the transport error if the underlying byte sink fails.
    fn write_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error>;
}

/// Chunk-oriented byte I/O used by target firmware polling loops.
///
/// A UART, semihosting channel, or emulator socket shim can implement this
/// trait by copying at most one currently available byte chunk into caller
/// storage and by writing a full length-prefixed response frame. Returning
/// `Ok(0)` from [`PilByteIo::read_bytes`] means no bytes are available in this
/// poll; it is not treated as an error.
pub trait PilByteIo {
    /// Transport-specific error type.
    type Error;

    /// Read one currently available byte chunk into `storage`.
    ///
    /// Returns the number of bytes written to `storage`, or `0` when no bytes
    /// are currently available.
    ///
    /// # Errors
    ///
    /// Returns the transport error if the underlying byte source fails.
    fn read_bytes(&mut self, storage: &mut [u8]) -> Result<usize, Self::Error>;

    /// Write one complete length-prefixed command frame.
    ///
    /// # Errors
    ///
    /// Returns the transport error if the underlying byte sink fails.
    fn write_bytes(&mut self, frame: &[u8]) -> Result<(), Self::Error>;
}

/// Errors raised by [`run_framed_endpoint_once`].
#[derive(Debug)]
pub enum PilEndpointError<E> {
    /// Reading the incoming frame failed.
    Read(E),
    /// Frame extraction, FC stepping, or response framing failed.
    Frame(PilFrameError),
    /// Writing the outgoing command frame failed.
    Write(E),
}

/// Status returned by [`poll_byte_stream_endpoint_once`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PilByteEndpointStatus {
    /// No complete incoming frame is buffered yet.
    Waiting {
        /// Bytes read from the byte source during this poll.
        read: usize,
        /// Bytes currently buffered by the frame pump.
        buffered: usize,
    },
    /// A command frame was written to the byte sink.
    CommandWritten {
        /// Bytes written to the byte sink.
        written: usize,
    },
}

/// Errors raised by [`poll_byte_stream_endpoint_once`].
#[derive(Debug)]
pub enum PilByteEndpointError<E> {
    /// Reading a byte chunk failed.
    Read(E),
    /// The byte source reported more bytes than the provided scratch storage.
    ReadOverflow {
        /// Scratch storage capacity.
        capacity: usize,
        /// Reported byte count.
        read: usize,
    },
    /// Frame pump ingestion or FC stepping failed.
    Pump(PilFramePumpError),
    /// A completed pump step did not leave an output frame to transmit.
    MissingOutputFrame {
        /// Frame byte count reported by the pump.
        written: usize,
    },
    /// Writing a completed command frame failed.
    Write(E),
}

/// Linker-symbol contract expected by host-side PIL GDB helpers.
///
/// The symbol names are advisory in this safe crate. Downstream target
/// firmware owns actual linker placement/export policy and can use this value
/// to keep mailbox layout and symbol spelling synchronized with host plans.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct PilFirmwareSymbolContract {
    /// Recommended symbol for the mailbox storage object.
    pub mailbox_symbol: &'static str,
    /// Recommended symbol for the target function that steps the mailbox.
    pub step_entry_symbol: &'static str,
    /// Concrete `repr(C)` mailbox layout for the selected capacities.
    pub mailbox_layout: PilMailboxLayout,
}

impl PilFirmwareSymbolContract {
    /// Build the default OpenBMP PIL symbol contract for a mailbox capacity.
    #[must_use]
    pub fn for_mailbox<const SENSOR_CAPACITY: usize, const COMMAND_CAPACITY: usize>() -> Self {
        Self {
            mailbox_symbol: OPENBMP_PIL_MAILBOX_SYMBOL,
            step_entry_symbol: OPENBMP_PIL_STEP_MAILBOX_SYMBOL,
            mailbox_layout: PilMailbox::<SENSOR_CAPACITY, COMMAND_CAPACITY>::layout(),
        }
    }
}

/// Safe target-firmware wrapper around one FC instance and PIL transports.
///
/// This type is the target-reachable object a downstream board wrapper can
/// own in static or startup-initialized storage. It keeps the mailbox,
/// UART-like frame pump, and read scratch in bounded caller-selected arrays,
/// and exposes only safe Rust entry points. Actual reset vectors, interrupt
/// handlers, UART register access, and linker symbol exports remain downstream
/// board concerns.
pub struct PilFirmware<
    const SENSOR_CAPACITY: usize,
    const COMMAND_CAPACITY: usize,
    const INPUT_CAPACITY: usize,
    const OUTPUT_CAPACITY: usize,
    const READ_CAPACITY: usize,
> {
    controller: FlightController,
    mailbox: PilMailbox<SENSOR_CAPACITY, COMMAND_CAPACITY>,
    pump: PilFramePump<INPUT_CAPACITY, OUTPUT_CAPACITY>,
    read_storage: [u8; READ_CAPACITY],
}

impl<
    const SENSOR_CAPACITY: usize,
    const COMMAND_CAPACITY: usize,
    const INPUT_CAPACITY: usize,
    const OUTPUT_CAPACITY: usize,
    const READ_CAPACITY: usize,
> fmt::Debug
    for PilFirmware<
        SENSOR_CAPACITY,
        COMMAND_CAPACITY,
        INPUT_CAPACITY,
        OUTPUT_CAPACITY,
        READ_CAPACITY,
    >
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PilFirmware")
            .field("mailbox", &self.mailbox)
            .field("pump", &self.pump)
            .field("read_capacity", &READ_CAPACITY)
            .finish_non_exhaustive()
    }
}

impl<
    const SENSOR_CAPACITY: usize,
    const COMMAND_CAPACITY: usize,
    const INPUT_CAPACITY: usize,
    const OUTPUT_CAPACITY: usize,
    const READ_CAPACITY: usize,
> PilFirmware<SENSOR_CAPACITY, COMMAND_CAPACITY, INPUT_CAPACITY, OUTPUT_CAPACITY, READ_CAPACITY>
{
    /// Construct a firmware wrapper around an initialized controller.
    #[must_use]
    pub fn new(controller: FlightController) -> Self {
        Self {
            controller,
            mailbox: PilMailbox::new(),
            pump: PilFramePump::new(),
            read_storage: [0; READ_CAPACITY],
        }
    }

    /// Return the symbol/layout contract for this firmware shape.
    #[must_use]
    pub fn symbol_contract(&self) -> PilFirmwareSymbolContract {
        PilFirmwareSymbolContract::for_mailbox::<SENSOR_CAPACITY, COMMAND_CAPACITY>()
    }

    /// Return the concrete mailbox layout for this firmware shape.
    #[must_use]
    pub fn mailbox_layout(&self) -> PilMailboxLayout {
        PilMailbox::<SENSOR_CAPACITY, COMMAND_CAPACITY>::layout()
    }

    /// Borrow the owned flight controller.
    #[must_use]
    pub fn controller(&self) -> &FlightController {
        &self.controller
    }

    /// Mutably borrow the owned flight controller.
    pub fn controller_mut(&mut self) -> &mut FlightController {
        &mut self.controller
    }

    /// Borrow the target-visible mailbox.
    #[must_use]
    pub fn mailbox(&self) -> &PilMailbox<SENSOR_CAPACITY, COMMAND_CAPACITY> {
        &self.mailbox
    }

    /// Mutably borrow the target-visible mailbox.
    pub fn mailbox_mut(&mut self) -> &mut PilMailbox<SENSOR_CAPACITY, COMMAND_CAPACITY> {
        &mut self.mailbox
    }

    /// Borrow the UART-like frame pump.
    #[must_use]
    pub fn pump(&self) -> &PilFramePump<INPUT_CAPACITY, OUTPUT_CAPACITY> {
        &self.pump
    }

    /// Mutably borrow the UART-like frame pump.
    pub fn pump_mut(&mut self) -> &mut PilFramePump<INPUT_CAPACITY, OUTPUT_CAPACITY> {
        &mut self.pump
    }

    /// Load encoded sensor bytes into the mailbox.
    ///
    /// # Errors
    ///
    /// Returns [`PilMailboxError`] when the payload does not fit.
    pub fn load_mailbox_sensor_payload(&mut self, payload: &[u8]) -> Result<(), PilMailboxError> {
        self.mailbox.load_sensor_payload(payload)
    }

    /// Run one mailbox-backed FC step.
    #[must_use]
    pub fn step_mailbox(&mut self) -> PilMailboxStatus {
        self.mailbox.step(&mut self.controller)
    }

    /// Run one complete-frame PIL exchange through an external byte endpoint.
    ///
    /// # Errors
    ///
    /// Returns [`PilEndpointError`] when the transport or framed PIL exchange
    /// fails.
    pub fn run_framed_once<I: PilFrameIo>(
        &mut self,
        io: &mut I,
        input_frame_storage: &mut [u8],
        output_frame_storage: &mut [u8],
    ) -> Result<PilFrameStep, PilEndpointError<I::Error>> {
        run_framed_endpoint_once(
            &mut self.controller,
            io,
            input_frame_storage,
            output_frame_storage,
        )
    }

    /// Poll one UART-like byte-stream exchange using internal fixed storage.
    ///
    /// # Errors
    ///
    /// Returns [`PilByteEndpointError`] when reading, writing, or frame-pump
    /// processing fails.
    pub fn poll_byte_stream_once<I: PilByteIo>(
        &mut self,
        io: &mut I,
    ) -> Result<PilByteEndpointStatus, PilByteEndpointError<I::Error>> {
        poll_byte_stream_endpoint_once(
            &mut self.controller,
            io,
            &mut self.pump,
            &mut self.read_storage,
        )
    }
}

/// Integer cycle-approximate PIL WCET budget.
///
/// The budget is intentionally simple and target-agnostic: a measured or
/// tool-estimated instruction count is multiplied by a fixed CPI value stored
/// in milli-cycles, then compared against the controller frame period. This is
/// evidence plumbing only; it is not a certified WCET proof.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct PilWcetBudget {
    /// Worst-case instruction count for one PIL controller step.
    pub instruction_count: u64,
    /// Worst-case cycles per instruction in milli-cycles.
    pub cycles_per_instruction_milli: u32,
    /// Target CPU frequency in Hertz.
    pub cpu_hz: u64,
    /// Controller frame period in microseconds.
    pub frame_period_us: u64,
}

impl PilWcetBudget {
    /// Construct and validate a cycle-approximate PIL WCET budget.
    ///
    /// # Errors
    ///
    /// Returns [`PilWcetBudgetError`] when any field is zero.
    pub fn new(
        instruction_count: u64,
        cycles_per_instruction_milli: u32,
        cpu_hz: u64,
        frame_period_us: u64,
    ) -> Result<Self, PilWcetBudgetError> {
        let budget = Self {
            instruction_count,
            cycles_per_instruction_milli,
            cpu_hz,
            frame_period_us,
        };
        budget.validate()?;
        Ok(budget)
    }

    /// Validate that all budget inputs are positive.
    ///
    /// # Errors
    ///
    /// Returns [`PilWcetBudgetError`] when any field is zero.
    pub fn validate(self) -> Result<(), PilWcetBudgetError> {
        if self.instruction_count == 0 {
            return Err(PilWcetBudgetError::ZeroInstructionCount);
        }
        if self.cycles_per_instruction_milli == 0 {
            return Err(PilWcetBudgetError::ZeroCyclesPerInstruction);
        }
        if self.cpu_hz == 0 {
            return Err(PilWcetBudgetError::ZeroCpuFrequency);
        }
        if self.frame_period_us == 0 {
            return Err(PilWcetBudgetError::ZeroFramePeriod);
        }
        Ok(())
    }

    /// Return `ceil(instruction_count * CPI)` cycles.
    ///
    /// # Errors
    ///
    /// Returns [`PilWcetBudgetError::CycleOverflow`] when the result cannot
    /// fit in `u64`.
    pub fn worst_case_cycles_ceil(self) -> Result<u64, PilWcetBudgetError> {
        self.validate()?;
        let milli_cycles =
            u128::from(self.instruction_count) * u128::from(self.cycles_per_instruction_milli);
        let cycles = milli_cycles.div_ceil(1_000);
        u64::try_from(cycles).map_err(|_| PilWcetBudgetError::CycleOverflow)
    }

    /// Return `ceil(worst_case_cycles / cpu_hz)` in microseconds.
    ///
    /// # Errors
    ///
    /// Returns [`PilWcetBudgetError`] when the budget is invalid or the cycle
    /// count overflows.
    pub fn worst_case_time_us_ceil(self) -> Result<u64, PilWcetBudgetError> {
        let cycles = u128::from(self.worst_case_cycles_ceil()?);
        let cpu_hz = u128::from(self.cpu_hz);
        let us = (cycles * 1_000_000).div_ceil(cpu_hz);
        u64::try_from(us).map_err(|_| PilWcetBudgetError::TimeOverflow)
    }

    /// Return signed deadline margin in microseconds.
    ///
    /// Positive values mean the estimate is below the frame period; negative
    /// values mean it overruns the frame period.
    ///
    /// # Errors
    ///
    /// Returns [`PilWcetBudgetError`] when validation or timing computation
    /// fails.
    pub fn deadline_margin_us(self) -> Result<i128, PilWcetBudgetError> {
        Ok(i128::from(self.frame_period_us) - i128::from(self.worst_case_time_us_ceil()?))
    }

    /// Return whether the estimated WCET fits within the frame period.
    ///
    /// # Errors
    ///
    /// Returns [`PilWcetBudgetError`] when validation or timing computation
    /// fails.
    pub fn meets_deadline(self) -> Result<bool, PilWcetBudgetError> {
        Ok(self.deadline_margin_us()? >= 0)
    }

    /// Build a deterministic report for traceability evidence.
    ///
    /// # Errors
    ///
    /// Returns [`PilWcetBudgetError`] when validation or timing computation
    /// fails.
    pub fn report(self) -> Result<PilWcetReport, PilWcetBudgetError> {
        let worst_case_cycles = self.worst_case_cycles_ceil()?;
        let worst_case_time_us = self.worst_case_time_us_ceil()?;
        let deadline_margin_us = i128::from(self.frame_period_us) - i128::from(worst_case_time_us);
        Ok(PilWcetReport {
            instruction_count: self.instruction_count,
            cycles_per_instruction_milli: self.cycles_per_instruction_milli,
            cpu_hz: self.cpu_hz,
            frame_period_us: self.frame_period_us,
            worst_case_cycles,
            worst_case_time_us,
            deadline_margin_us,
            meets_deadline: deadline_margin_us >= 0,
        })
    }
}

/// Deterministic PIL WCET budget report.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct PilWcetReport {
    /// Worst-case instruction count used for the estimate.
    pub instruction_count: u64,
    /// Worst-case cycles per instruction in milli-cycles.
    pub cycles_per_instruction_milli: u32,
    /// Target CPU frequency in Hertz.
    pub cpu_hz: u64,
    /// Controller frame period in microseconds.
    pub frame_period_us: u64,
    /// Estimated worst-case cycles, rounded up.
    pub worst_case_cycles: u64,
    /// Estimated worst-case time in microseconds, rounded up.
    pub worst_case_time_us: u64,
    /// Signed deadline margin in microseconds.
    pub deadline_margin_us: i128,
    /// Whether the estimated worst-case time fits within the frame period.
    pub meets_deadline: bool,
}

/// Errors raised while validating a PIL WCET budget.
#[derive(Copy, Clone, Debug, Eq, PartialEq, thiserror::Error)]
pub enum PilWcetBudgetError {
    /// Instruction count must be positive.
    #[error("PIL WCET instruction count must be positive")]
    ZeroInstructionCount,
    /// CPI must be positive.
    #[error("PIL WCET cycles-per-instruction must be positive")]
    ZeroCyclesPerInstruction,
    /// CPU frequency must be positive.
    #[error("PIL WCET CPU frequency must be positive")]
    ZeroCpuFrequency,
    /// Frame period must be positive.
    #[error("PIL WCET frame period must be positive")]
    ZeroFramePeriod,
    /// Worst-case cycle count does not fit in `u64`.
    #[error("PIL WCET worst-case cycle count overflowed")]
    CycleOverflow,
    /// Worst-case time does not fit in `u64`.
    #[error("PIL WCET worst-case time overflowed")]
    TimeOverflow,
}

/// Status returned by the fixed-buffer PIL frame pump.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PilFramePumpStatus {
    /// No complete incoming frame is buffered yet.
    Waiting {
        /// Bytes currently buffered from the incoming byte stream.
        buffered: usize,
    },
    /// A command frame is ready to transmit.
    CommandReady {
        /// Bytes available in [`PilFramePump::output_frame`].
        written: usize,
    },
}

/// Errors raised by [`PilFramePump`].
#[derive(Debug, thiserror::Error)]
pub enum PilFramePumpError {
    /// Incoming bytes would overflow the fixed receive buffer.
    #[error("PIL frame pump input overflow: capacity {capacity} bytes, need {needed}")]
    InputOverflow {
        /// Receive-buffer capacity.
        capacity: usize,
        /// Required bytes after appending the new chunk.
        needed: usize,
    },
    /// A command frame is pending and must be cleared before another step.
    #[error("PIL frame pump output frame is still pending: {len} bytes")]
    OutputPending {
        /// Pending output-frame length.
        len: usize,
    },
    /// Framed PIL processing failed.
    #[error("PIL frame pump step failed: {0}")]
    Frame(#[source] PilFrameError),
}

/// Fixed-buffer frame pump for UART-like target byte streams.
///
/// Target firmware can append arbitrary byte chunks as they arrive, call
/// [`PilFramePump::try_step`] when convenient, transmit
/// [`PilFramePump::output_frame`] when present, then clear it. The pump stores
/// only the existing length-prefixed bridge frames and never owns a concrete
/// device driver.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PilFramePump<const INPUT_CAPACITY: usize, const OUTPUT_CAPACITY: usize> {
    input_bytes: [u8; INPUT_CAPACITY],
    input_len: usize,
    output_bytes: [u8; OUTPUT_CAPACITY],
    output_len: usize,
}

impl<const INPUT_CAPACITY: usize, const OUTPUT_CAPACITY: usize>
    PilFramePump<INPUT_CAPACITY, OUTPUT_CAPACITY>
{
    /// Construct an empty fixed-buffer frame pump.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            input_bytes: [0; INPUT_CAPACITY],
            input_len: 0,
            output_bytes: [0; OUTPUT_CAPACITY],
            output_len: 0,
        }
    }

    /// Clear buffered input and pending output.
    pub fn clear(&mut self) {
        self.input_len = 0;
        self.output_len = 0;
    }

    /// Append incoming byte-stream data to the fixed receive buffer.
    ///
    /// Returns the total buffered byte count after appending.
    ///
    /// # Errors
    ///
    /// Returns [`PilFramePumpError::InputOverflow`] when the chunk does not fit
    /// in the configured receive buffer.
    pub fn ingest(&mut self, bytes: &[u8]) -> Result<usize, PilFramePumpError> {
        let needed =
            self.input_len
                .checked_add(bytes.len())
                .ok_or(PilFramePumpError::InputOverflow {
                    capacity: INPUT_CAPACITY,
                    needed: usize::MAX,
                })?;
        if needed > self.input_bytes.len() {
            return Err(PilFramePumpError::InputOverflow {
                capacity: self.input_bytes.len(),
                needed,
            });
        }
        self.input_bytes[self.input_len..needed].copy_from_slice(bytes);
        self.input_len = needed;
        Ok(self.input_len)
    }

    /// Bytes currently buffered from the incoming byte stream.
    #[must_use]
    pub const fn buffered_len(&self) -> usize {
        self.input_len
    }

    /// Pending command frame, when one has been produced.
    #[must_use]
    pub fn output_frame(&self) -> Option<&[u8]> {
        if self.output_len == 0 {
            None
        } else {
            Some(&self.output_bytes[..self.output_len])
        }
    }

    /// Clear the pending command frame after the caller has transmitted it.
    pub fn clear_output_frame(&mut self) {
        self.output_len = 0;
    }

    /// Process one complete incoming frame if enough bytes are buffered.
    ///
    /// Returns [`PilFramePumpStatus::Waiting`] when the buffered bytes do not
    /// yet contain a whole frame.
    ///
    /// # Errors
    ///
    /// Returns [`PilFramePumpError`] when a previous output frame is still
    /// pending or when framed PIL processing fails.
    pub fn try_step(
        &mut self,
        controller: &mut FlightController,
    ) -> Result<PilFramePumpStatus, PilFramePumpError> {
        if self.output_len != 0 {
            return Err(PilFramePumpError::OutputPending {
                len: self.output_len,
            });
        }

        match deframe(&self.input_bytes[..self.input_len]) {
            Ok((_payload, consumed)) => {
                let step = fc_step_from_frame_into(
                    controller,
                    &self.input_bytes[..self.input_len],
                    &mut self.output_bytes,
                )
                .map_err(PilFramePumpError::Frame)?;
                let remaining = self.input_len - consumed;
                self.input_bytes.copy_within(consumed..self.input_len, 0);
                self.input_len = remaining;
                self.output_len = step.written;
                Ok(PilFramePumpStatus::CommandReady {
                    written: step.written,
                })
            }
            Err(BridgeError::PrefixIncomplete { .. } | BridgeError::FrameTruncated { .. }) => {
                Ok(PilFramePumpStatus::Waiting {
                    buffered: self.input_len,
                })
            }
            Err(error) => Err(PilFramePumpError::Frame(PilFrameError::IncomingFrame(
                error,
            ))),
        }
    }
}

impl<const INPUT_CAPACITY: usize, const OUTPUT_CAPACITY: usize> Default
    for PilFramePump<INPUT_CAPACITY, OUTPUT_CAPACITY>
{
    fn default() -> Self {
        Self::new()
    }
}

/// Mailbox status for a target-side PIL step exchange.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PilMailboxStatus {
    /// No sensor payload is pending.
    Empty = 0,
    /// Sensor bytes are loaded and ready for one FC step.
    SensorReady = 1,
    /// Command bytes are available for the host/emulator side to read.
    CommandReady = 2,
    /// The last mailbox step failed; inspect [`PilMailbox::error`].
    Fault = 3,
}

/// Compact target-visible error code for [`PilMailbox`].
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PilMailboxError {
    /// No error is present.
    None = 0,
    /// The incoming sensor payload exceeds mailbox input capacity.
    SensorTooLarge = 1,
    /// `sensor_len` points outside mailbox input storage.
    InvalidSensorLength = 2,
    /// `command_len` points outside mailbox output storage.
    InvalidCommandLength = 3,
    /// Incoming sensor bytes could not be decoded.
    Decode = 4,
    /// Outgoing command bytes could not be encoded.
    Encode = 5,
    /// The FC scheduler or bus step failed.
    Controller = 6,
    /// An FC effector or engine id exceeded the bridge wire range.
    CommandIdOutOfRange = 7,
    /// The outgoing command payload exceeds mailbox output capacity.
    OutputBufferTooSmall = 8,
}

impl PilMailboxError {
    fn from_step_error(error: &PilStepError) -> Self {
        match error {
            PilStepError::Decode(_) => Self::Decode,
            PilStepError::Encode(_) => Self::Encode,
            PilStepError::Controller(_) => Self::Controller,
            PilStepError::CommandIdOutOfRange { .. } => Self::CommandIdOutOfRange,
            PilStepError::OutputBufferTooSmall { .. } => Self::OutputBufferTooSmall,
        }
    }
}

/// Concrete `repr(C)` memory layout for [`PilMailbox`].
///
/// Host-side GDB/Renode couplers can use this layout with a resolved target
/// symbol address to write encoded sensor bytes and read encoded command
/// bytes without duplicating Rust field-offset assumptions.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct PilMailboxLayout {
    /// Total `PilMailbox<SENSOR_CAPACITY, COMMAND_CAPACITY>` byte size.
    pub size_bytes: usize,
    /// Configured incoming sensor-payload storage capacity.
    pub sensor_capacity: usize,
    /// Configured outgoing command-payload storage capacity.
    pub command_capacity: usize,
    /// Byte offset of [`PilMailbox::protocol_version`].
    pub protocol_version_offset: usize,
    /// Byte offset of [`PilMailbox::status`].
    pub status_offset: usize,
    /// Byte offset of [`PilMailbox::error`].
    pub error_offset: usize,
    /// Byte offset of [`PilMailbox::sensor_len`].
    pub sensor_len_offset: usize,
    /// Byte offset of [`PilMailbox::command_len`].
    pub command_len_offset: usize,
    /// Byte offset of [`PilMailbox::sensor_bytes`].
    pub sensor_bytes_offset: usize,
    /// Byte offset of [`PilMailbox::command_bytes`].
    pub command_bytes_offset: usize,
}

/// Fixed-storage PIL mailbox suitable for target firmware wrappers.
///
/// A downstream target binary can expose one instance as a known symbol for a
/// GDB-stub flow, or copy UART payloads into `sensor_bytes` before calling
/// [`PilMailbox::step`]. This type intentionally stores only bridge payload
/// bytes; framing, interrupts, and emulator-specific symbol placement are
/// outside this crate.
#[repr(C)]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PilMailbox<const SENSOR_CAPACITY: usize, const COMMAND_CAPACITY: usize> {
    /// Bridge protocol version expected by this mailbox.
    pub protocol_version: u16,
    /// Current mailbox state.
    pub status: PilMailboxStatus,
    /// Compact error code for the last failed step.
    pub error: PilMailboxError,
    /// Number of valid bytes in [`PilMailbox::sensor_bytes`].
    pub sensor_len: u32,
    /// Number of valid bytes in [`PilMailbox::command_bytes`].
    pub command_len: u32,
    /// Caller-owned encoded [`SensorPacket`] bytes.
    pub sensor_bytes: [u8; SENSOR_CAPACITY],
    /// Caller-owned encoded [`ActuatorCommandPacket`] bytes.
    pub command_bytes: [u8; COMMAND_CAPACITY],
}

impl<const SENSOR_CAPACITY: usize, const COMMAND_CAPACITY: usize>
    PilMailbox<SENSOR_CAPACITY, COMMAND_CAPACITY>
{
    /// Return the concrete `repr(C)` field layout for this mailbox type.
    #[must_use]
    pub fn layout() -> PilMailboxLayout {
        PilMailboxLayout {
            size_bytes: size_of::<Self>(),
            sensor_capacity: SENSOR_CAPACITY,
            command_capacity: COMMAND_CAPACITY,
            protocol_version_offset: offset_of!(Self, protocol_version),
            status_offset: offset_of!(Self, status),
            error_offset: offset_of!(Self, error),
            sensor_len_offset: offset_of!(Self, sensor_len),
            command_len_offset: offset_of!(Self, command_len),
            sensor_bytes_offset: offset_of!(Self, sensor_bytes),
            command_bytes_offset: offset_of!(Self, command_bytes),
        }
    }

    /// Construct an empty mailbox for the current bridge protocol version.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            protocol_version: PROTOCOL_VERSION,
            status: PilMailboxStatus::Empty,
            error: PilMailboxError::None,
            sensor_len: 0,
            command_len: 0,
            sensor_bytes: [0; SENSOR_CAPACITY],
            command_bytes: [0; COMMAND_CAPACITY],
        }
    }

    /// Clear lengths, status, and error while preserving backing storage.
    pub fn clear(&mut self) {
        self.status = PilMailboxStatus::Empty;
        self.error = PilMailboxError::None;
        self.sensor_len = 0;
        self.command_len = 0;
    }

    /// Copy encoded sensor bytes into the mailbox and mark them ready.
    ///
    /// # Errors
    ///
    /// Returns [`PilMailboxError::SensorTooLarge`] when `payload` does not fit
    /// in [`PilMailbox::sensor_bytes`].
    pub fn load_sensor_payload(&mut self, payload: &[u8]) -> Result<(), PilMailboxError> {
        if payload.len() > self.sensor_bytes.len() {
            self.fail(PilMailboxError::SensorTooLarge);
            return Err(PilMailboxError::SensorTooLarge);
        }
        let len = u32::try_from(payload.len()).map_err(|_| {
            self.fail(PilMailboxError::SensorTooLarge);
            PilMailboxError::SensorTooLarge
        })?;
        self.sensor_bytes[..payload.len()].copy_from_slice(payload);
        self.sensor_len = len;
        self.command_len = 0;
        self.error = PilMailboxError::None;
        self.status = PilMailboxStatus::SensorReady;
        Ok(())
    }

    /// Return the currently loaded encoded sensor payload.
    ///
    /// # Errors
    ///
    /// Returns [`PilMailboxError::InvalidSensorLength`] when `sensor_len`
    /// points outside [`PilMailbox::sensor_bytes`].
    pub fn sensor_payload(&self) -> Result<&[u8], PilMailboxError> {
        let len = self.valid_sensor_len()?;
        Ok(&self.sensor_bytes[..len])
    }

    /// Return the currently encoded actuator command payload.
    ///
    /// # Errors
    ///
    /// Returns [`PilMailboxError::InvalidCommandLength`] when `command_len`
    /// points outside [`PilMailbox::command_bytes`].
    pub fn command_payload(&self) -> Result<&[u8], PilMailboxError> {
        let len = self.valid_command_len()?;
        Ok(&self.command_bytes[..len])
    }

    /// Run one FC step from the current sensor payload into command storage.
    ///
    /// Returns the new mailbox status. On failure the status is
    /// [`PilMailboxStatus::Fault`] and [`PilMailbox::error`] carries a compact
    /// target-visible code.
    pub fn step(&mut self, controller: &mut FlightController) -> PilMailboxStatus {
        match self.try_step(controller) {
            Ok(()) => PilMailboxStatus::CommandReady,
            Err(error) => {
                self.fail(error);
                PilMailboxStatus::Fault
            }
        }
    }

    /// Fallible form of [`PilMailbox::step`].
    ///
    /// # Errors
    ///
    /// Returns a compact mailbox error when lengths are invalid or the PIL byte
    /// step fails.
    pub fn try_step(&mut self, controller: &mut FlightController) -> Result<(), PilMailboxError> {
        let sensor_len = self.valid_sensor_len()?;
        let result = fc_step_from_wire_into(
            controller,
            &self.sensor_bytes[..sensor_len],
            &mut self.command_bytes,
        );
        match result {
            Ok(command_len) => {
                let command_len = u32::try_from(command_len)
                    .map_err(|_| PilMailboxError::OutputBufferTooSmall)?;
                self.command_len = command_len;
                self.error = PilMailboxError::None;
                self.status = PilMailboxStatus::CommandReady;
                Ok(())
            }
            Err(error) => Err(PilMailboxError::from_step_error(&error)),
        }
    }

    fn valid_sensor_len(&self) -> Result<usize, PilMailboxError> {
        let len = self.sensor_len as usize;
        if len > self.sensor_bytes.len() {
            return Err(PilMailboxError::InvalidSensorLength);
        }
        Ok(len)
    }

    fn valid_command_len(&self) -> Result<usize, PilMailboxError> {
        let len = self.command_len as usize;
        if len > self.command_bytes.len() {
            return Err(PilMailboxError::InvalidCommandLength);
        }
        Ok(len)
    }

    fn fail(&mut self, error: PilMailboxError) {
        self.error = error;
        self.status = PilMailboxStatus::Fault;
        self.command_len = 0;
    }
}

impl<const SENSOR_CAPACITY: usize, const COMMAND_CAPACITY: usize> Default
    for PilMailbox<SENSOR_CAPACITY, COMMAND_CAPACITY>
{
    fn default() -> Self {
        Self::new()
    }
}

/// Run one FC step through a target mailbox.
///
/// Returns the new mailbox status; see [`PilMailbox::step`] for error-state
/// details.
pub fn step_mailbox<const SENSOR_CAPACITY: usize, const COMMAND_CAPACITY: usize>(
    controller: &mut FlightController,
    mailbox: &mut PilMailbox<SENSOR_CAPACITY, COMMAND_CAPACITY>,
) -> PilMailboxStatus {
    mailbox.step(controller)
}

/// Decode one length-prefixed sensor frame, run one FC tick, and write one
/// length-prefixed actuator-command frame.
///
/// The input may contain more than one frame; the returned
/// [`PilFrameStep::consumed`] tells the caller how much input was consumed.
///
/// # Errors
///
/// Returns [`PilFrameError`] when the input frame is incomplete or malformed,
/// the FC byte step fails, the response payload exceeds the bridge frame
/// length, or the output buffer is too small for the framed response.
pub fn fc_step_from_frame_into(
    controller: &mut FlightController,
    framed_sensor: &[u8],
    framed_command: &mut [u8],
) -> Result<PilFrameStep, PilFrameError> {
    let (sensor_payload, consumed) =
        deframe(framed_sensor).map_err(PilFrameError::IncomingFrame)?;
    if framed_command.len() < 4 {
        return Err(PilFrameError::OutputFrameTooSmall {
            needed: 4,
            capacity: framed_command.len(),
        });
    }
    let payload_capacity = framed_command.len() - 4;
    let command_payload_len =
        fc_step_from_wire_into(controller, sensor_payload, &mut framed_command[4..]).map_err(
            |error| match error {
                PilStepError::OutputBufferTooSmall { needed, capacity } => {
                    PilFrameError::OutputFrameTooSmall {
                        needed: needed.saturating_add(4),
                        capacity: capacity.saturating_add(4),
                    }
                }
                other => PilFrameError::Step(other),
            },
        )?;
    let frame_len =
        u32::try_from(command_payload_len).map_err(|_| PilFrameError::OutputPayloadTooLarge {
            max: u32::MAX as usize,
            got: command_payload_len,
        })?;
    if command_payload_len > payload_capacity {
        return Err(PilFrameError::OutputFrameTooSmall {
            needed: command_payload_len.saturating_add(4),
            capacity: framed_command.len(),
        });
    }
    framed_command[..4].copy_from_slice(&frame_len.to_le_bytes());
    Ok(PilFrameStep {
        consumed,
        written: command_payload_len + 4,
    })
}

/// Run one complete read-step-write exchange against a target byte transport.
///
/// `input_frame_storage` and `output_frame_storage` are caller-owned scratch
/// buffers so a target wrapper can keep allocation policy outside this crate.
///
/// # Errors
///
/// Returns [`PilEndpointError`] when the byte transport fails or framed PIL
/// processing fails.
pub fn run_framed_endpoint_once<I: PilFrameIo>(
    controller: &mut FlightController,
    io: &mut I,
    input_frame_storage: &mut [u8],
    output_frame_storage: &mut [u8],
) -> Result<PilFrameStep, PilEndpointError<I::Error>> {
    let read = io
        .read_frame(input_frame_storage)
        .map_err(PilEndpointError::Read)?;
    let step = fc_step_from_frame_into(
        controller,
        &input_frame_storage[..read],
        output_frame_storage,
    )
    .map_err(PilEndpointError::Frame)?;
    io.write_frame(&output_frame_storage[..step.written])
        .map_err(PilEndpointError::Write)?;
    Ok(step)
}

/// Poll one chunk-oriented byte-stream endpoint once.
///
/// This helper is the target-side loop body for UART-like transports: it first
/// flushes any pending command frame from `pump`, otherwise reads one byte
/// chunk into `read_storage`, ingests it into the fixed frame pump, and writes
/// a completed command frame when a full sensor frame has arrived. It does not
/// own a driver, allocate storage, or prescribe blocking/interrupt behavior.
///
/// # Errors
///
/// Returns [`PilByteEndpointError`] when the byte source/sink fails, the
/// reader reports an impossible byte count, or the frame pump fails.
pub fn poll_byte_stream_endpoint_once<
    I: PilByteIo,
    const INPUT_CAPACITY: usize,
    const OUTPUT_CAPACITY: usize,
>(
    controller: &mut FlightController,
    io: &mut I,
    pump: &mut PilFramePump<INPUT_CAPACITY, OUTPUT_CAPACITY>,
    read_storage: &mut [u8],
) -> Result<PilByteEndpointStatus, PilByteEndpointError<I::Error>> {
    if let Some(frame) = pump.output_frame() {
        let written = frame.len();
        io.write_bytes(frame).map_err(PilByteEndpointError::Write)?;
        pump.clear_output_frame();
        return Ok(PilByteEndpointStatus::CommandWritten { written });
    }

    let read = io
        .read_bytes(read_storage)
        .map_err(PilByteEndpointError::Read)?;
    if read > read_storage.len() {
        return Err(PilByteEndpointError::ReadOverflow {
            capacity: read_storage.len(),
            read,
        });
    }
    if read != 0 {
        pump.ingest(&read_storage[..read])
            .map_err(PilByteEndpointError::Pump)?;
    }

    match pump
        .try_step(controller)
        .map_err(PilByteEndpointError::Pump)?
    {
        PilFramePumpStatus::Waiting { buffered } => {
            Ok(PilByteEndpointStatus::Waiting { read, buffered })
        }
        PilFramePumpStatus::CommandReady { written } => {
            let Some(frame) = pump.output_frame() else {
                return Err(PilByteEndpointError::MissingOutputFrame { written });
            };
            let written = frame.len();
            io.write_bytes(frame).map_err(PilByteEndpointError::Write)?;
            pump.clear_output_frame();
            Ok(PilByteEndpointStatus::CommandWritten { written })
        }
    }
}

/// Decode a bridge sensor payload, run one FC tick, and encode actuator bytes.
///
/// # Errors
///
/// Returns [`PilStepError`] if decoding fails, the controller step fails, a
/// command id is outside the bridge schema range, or encoding fails.
pub fn fc_step_from_wire(
    controller: &mut FlightController,
    payload: &[u8],
) -> Result<Vec<u8>, PilStepError> {
    let sensor = decode::<SensorPacket>(payload).map_err(PilStepError::Decode)?;
    let command = fc_step_from_sensor_packet(controller, &sensor)?;
    encode(&command).map_err(PilStepError::Encode)
}

/// Decode a bridge sensor payload, run one FC tick, and write actuator bytes.
///
/// Returns the number of bytes written to `output`.
///
/// # Errors
///
/// Returns [`PilStepError`] if decoding fails, the controller step fails, a
/// command id is outside the bridge schema range, encoding fails, or `output`
/// is too small for the encoded command.
pub fn fc_step_from_wire_into(
    controller: &mut FlightController,
    payload: &[u8],
    output: &mut [u8],
) -> Result<usize, PilStepError> {
    let sensor = decode::<SensorPacket>(payload).map_err(PilStepError::Decode)?;
    let command = fc_step_from_sensor_packet(controller, &sensor)?;
    match encode_into(&command, output) {
        Ok(len) => Ok(len),
        Err(err) => {
            let encoded = encode(&command).map_err(PilStepError::Encode)?;
            if encoded.len() > output.len() {
                return Err(PilStepError::OutputBufferTooSmall {
                    needed: encoded.len(),
                    capacity: output.len(),
                });
            }
            Err(PilStepError::Encode(err))
        }
    }
}

/// Adapt one bridge sensor packet through the hardware-portable FC step.
///
/// # Errors
///
/// Returns [`PilStepError`] if the controller step fails or a command id is
/// outside the bridge schema range.
pub fn fc_step_from_sensor_packet(
    controller: &mut FlightController,
    sensor: &SensorPacket,
) -> Result<ActuatorCommandPacket, PilStepError> {
    let input = fc_step_input_from_sensor_packet(sensor);
    let output = fc_step(controller, input).map_err(PilStepError::Controller)?;
    command_packet_from_step_output(sensor.sim_time_s, sensor.step, &output)
}

/// Build the topic-level FC input bundle represented by a bridge sensor packet.
#[must_use]
pub fn fc_step_input_from_sensor_packet(sensor: &SensorPacket) -> FcStepInput {
    let time = SimTime::from_seconds(sensor.sim_time_s);
    let mut input = FcStepInput::new(time, StepIndex::new(sensor.step)).with_imu(ImuSample {
        time,
        gyro_rad_s: Vector3::from(sensor.imu_gyro_body_rad_s),
        accel_m_s2: Vector3::from(sensor.imu_accel_body_m_s2),
        healthy: true,
    });
    if !sensor.imu_increments.is_empty() {
        input = input.with_imu_increment_window(imu_increment_window_from_packet(sensor, time));
    }

    if let (Some(pressure_pa), Some(bias_pa)) = (sensor.baro_pressure_pa, sensor.baro_bias_pa) {
        input = input.with_barometer(BarometerSample {
            time,
            pressure_pa,
            bias_pa,
            healthy: true,
        });
    }
    if let (Some(position_eci_m), Some(velocity_eci_m_s), Some(position_bias_eci_m)) = (
        sensor.gnss_position_eci_m,
        sensor.gnss_velocity_eci_m_s,
        sensor.gnss_position_bias_eci_m,
    ) {
        input = input.with_gnss(GnssSample {
            time,
            position_eci_m: Vector3::from(position_eci_m),
            velocity_eci_m_s: Vector3::from(velocity_eci_m_s),
            position_bias_eci_m: Vector3::from(position_bias_eci_m),
            healthy: true,
        });
    }
    let field_body_nt = sensor
        .mag_body_nt
        .or_else(|| sensor.mag_body_tesla.map(|v| v.map(|value| value * 1.0e9)));
    if let Some(field_body_nt) = field_body_nt {
        input = input.with_magnetometer(MagnetometerSample {
            time,
            field_body_nt: Vector3::from(field_body_nt),
            hard_iron_body_nt: Vector3::from(sensor.mag_hard_iron_body_nt.unwrap_or([0.0; 3])),
            healthy: true,
        });
    }
    if let Some(q) = sensor.star_tracker_attitude_eci_to_body_xyzw {
        input = input.with_star_tracker(StarTrackerSample {
            time,
            q_eci_to_body_xyzw: q,
            healthy: true,
        });
    }
    input
}

fn imu_increment_window_from_packet(sensor: &SensorPacket, time: SimTime) -> ImuIncrementWindow {
    ImuIncrementWindow {
        time,
        increments: sensor
            .imu_increments
            .iter()
            .copied()
            .map(imu_increment_from_packet)
            .collect(),
        healthy: true,
    }
}

fn imu_increment_from_packet(increment: ImuIncrementPacket) -> ImuInertialIncrement {
    ImuInertialIncrement {
        delta_theta_rad: Vector3::from(increment.delta_theta_rad),
        delta_v_m_s: Vector3::from(increment.delta_v_m_s),
        dt_s: increment.dt_s,
        seq: increment.seq,
    }
}

fn command_packet_from_step_output(
    sim_time_s: f64,
    step: u64,
    output: &FcStepOutput,
) -> Result<ActuatorCommandPacket, PilStepError> {
    command_packet_from_command_sets(
        sim_time_s,
        step,
        output.effector_commands.as_ref(),
        output.engine_commands.as_ref(),
    )
}

fn command_packet_from_command_sets(
    sim_time_s: f64,
    step: u64,
    effector_commands: Option<&EffectorCommandSet>,
    engine_commands: Option<&EngineCommandSet>,
) -> Result<ActuatorCommandPacket, PilStepError> {
    let mut effector_packet_commands = Vec::new();
    if let Some(commands) = effector_commands {
        for command in commands.commands.iter().take(usize::from(commands.count)) {
            let effector_id = u32::try_from(command.effector_id).map_err(|_| {
                PilStepError::CommandIdOutOfRange {
                    kind: "effector",
                    id: command.effector_id,
                }
            })?;
            effector_packet_commands.push((effector_id, command.command));
        }
    }

    let mut engine_throttles = Vec::new();
    let mut engine_packet_commands = Vec::new();
    if let Some(commands) = engine_commands {
        for command in commands.commands.iter().take(usize::from(commands.count)) {
            let engine_id = u32::try_from(command.engine_id).map_err(|_| {
                PilStepError::CommandIdOutOfRange {
                    kind: "engine",
                    id: command.engine_id,
                }
            })?;
            engine_throttles.push((engine_id, command.throttle_unit));
            engine_packet_commands.push(BridgeEngineCommandPacket {
                engine_id,
                throttle_unit: command.throttle_unit,
                gimbal_pitch_rad: command.gimbal_pitch_rad,
                gimbal_yaw_rad: command.gimbal_yaw_rad,
                ignite: command.ignite,
                shutdown: command.shutdown,
            });
        }
    }

    Ok(ActuatorCommandPacket {
        sim_time_s,
        step,
        effector_commands: effector_packet_commands,
        engine_throttles,
        engine_commands: engine_packet_commands,
    })
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use alloc::vec;
    use alloc::vec::Vec;

    use super::*;
    use openbmp_bridge::frame;
    use openbmp_fc::topics::{
        ActuatorCommand, EffectorCommand, EngineCommand, EngineDemand, FailsafeFlags, FdirStatus,
        VehicleStatus,
    };
    use openbmp_fc::{FlightControllerBuilder, Job, JobContext};

    struct PacketStepJob;

    impl Job for PacketStepJob {
        fn name(&self) -> &'static str {
            "test.pil_packet_step"
        }

        fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
            let (imu, _) = ctx
                .bus
                .latest::<ImuSample>()?
                .expect("PIL shim should publish IMU before dispatch");
            let (baro, _) = ctx
                .bus
                .latest::<BarometerSample>()?
                .expect("PIL shim should publish barometer before dispatch");
            let (gnss, _) = ctx
                .bus
                .latest::<GnssSample>()?
                .expect("PIL shim should publish GNSS before dispatch");
            let (mag, _) = ctx
                .bus
                .latest::<MagnetometerSample>()?
                .expect("PIL shim should publish magnetometer before dispatch");
            let (star, _) = ctx
                .bus
                .latest::<StarTrackerSample>()?
                .expect("PIL shim should publish star tracker before dispatch");

            assert_eq!(baro.pressure_pa, 91_250.0);
            assert_eq!(gnss.position_eci_m.x, 10.0);
            assert_eq!(mag.field_body_nt.x, 1_000.0);
            assert_eq!(star.q_eci_to_body_xyzw, [0.0, 0.1, 0.2, 0.97]);

            let mut effector_set = EffectorCommandSet {
                time: imu.time,
                count: 1,
                ..EffectorCommandSet::default()
            };
            effector_set.commands[0] = EffectorCommand {
                effector_id: 7,
                command: imu.accel_m_s2.z,
                saturated: false,
            };
            ctx.bus.publish(effector_set)?;

            let mut engine_set = EngineCommandSet {
                time: imu.time,
                count: 1,
                ..EngineCommandSet::default()
            };
            engine_set.commands[0] = EngineCommand {
                engine_id: 9,
                throttle_unit: 0.64,
                gimbal_pitch_rad: imu.gyro_rad_s.x,
                gimbal_yaw_rad: imu.gyro_rad_s.y,
                ignite: true,
                shutdown: false,
            };
            ctx.bus.publish(engine_set)?;
            Ok(())
        }
    }

    fn controller_for_packet_step() -> FlightController {
        let mut fc = FlightControllerBuilder::new()
            .frame_budget_us(1_000)
            .build();
        register_input_topics(&fc);
        register_output_topics(&fc);
        fc.scheduler_mut()
            .register_periodic(1, 100, 1, Box::new(PacketStepJob))
            .unwrap();
        fc
    }

    fn register_input_topics(fc: &FlightController) {
        fc.bus().register::<ImuSample>().unwrap();
        fc.bus().register::<ImuIncrementWindow>().unwrap();
        fc.bus().register::<BarometerSample>().unwrap();
        fc.bus().register::<GnssSample>().unwrap();
        fc.bus().register::<MagnetometerSample>().unwrap();
        fc.bus().register::<StarTrackerSample>().unwrap();
    }

    fn register_output_topics(fc: &FlightController) {
        fc.bus().register::<ActuatorCommand>().unwrap();
        fc.bus().register::<EffectorCommandSet>().unwrap();
        fc.bus().register::<EngineDemand>().unwrap();
        fc.bus().register::<EngineCommandSet>().unwrap();
        fc.bus().register::<FdirStatus>().unwrap();
        fc.bus().register::<FailsafeFlags>().unwrap();
        fc.bus().register::<VehicleStatus>().unwrap();
    }

    fn full_sensor_packet() -> SensorPacket {
        SensorPacket {
            sim_time_s: 0.25,
            step: 42,
            imu_accel_body_m_s2: [0.0, 0.0, 9.81],
            imu_gyro_body_rad_s: [0.01, -0.02, 0.03],
            baro_altitude_m: Some(1_250.0),
            gnss_position_eci_m: Some([10.0, 20.0, 30.0]),
            gnss_velocity_eci_m_s: Some([1.0, 2.0, 3.0]),
            gnss_position_bias_eci_m: Some([0.1, 0.2, 0.3]),
            mag_body_tesla: None,
            mag_body_nt: Some([1_000.0, 2_000.0, 3_000.0]),
            mag_hard_iron_body_nt: Some([1.0, 2.0, 3.0]),
            baro_pressure_pa: Some(91_250.0),
            baro_bias_pa: Some(12.0),
            star_tracker_attitude_eci_to_body_xyzw: Some([0.0, 0.1, 0.2, 0.97]),
            ..SensorPacket::default()
        }
    }

    fn command_packet_from_frame(frame: &[u8]) -> ActuatorCommandPacket {
        let (payload, consumed) =
            openbmp_bridge::deframe(frame).expect("command frame should deframe");
        assert_eq!(consumed, frame.len());
        decode(payload).unwrap()
    }

    #[test]
    fn pil_sensor_packet_input_carries_imu_increment_window() {
        let packet = SensorPacket {
            sim_time_s: 0.125,
            step: 42,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            imu_gyro_body_rad_s: [0.01, 0.02, 0.03],
            imu_increments: vec![ImuIncrementPacket {
                delta_theta_rad: [1.0e-4, 2.0e-4, 3.0e-4],
                delta_v_m_s: [0.001, 0.002, 0.003],
                dt_s: 0.00025,
                seq: 28,
            }],
            ..SensorPacket::default()
        };

        let input = fc_step_input_from_sensor_packet(&packet);
        let window = input
            .imu_increments
            .expect("PIL packet should carry IMU increment window");

        assert_eq!(window.time.as_seconds().to_bits(), 0.125_f64.to_bits());
        assert_eq!(window.increments.len(), 1);
        assert_eq!(window.increments[0].seq, 28);
        assert_eq!(
            window.increments[0].delta_theta_rad,
            Vector3::new(1.0e-4, 2.0e-4, 3.0e-4)
        );
        assert_eq!(
            window.increments[0].delta_v_m_s,
            Vector3::new(0.001, 0.002, 0.003)
        );
    }

    #[test]
    fn pil_mailbox_layout_reports_repr_c_offsets() {
        let layout = PilMailbox::<3, 5>::layout();

        assert_eq!(
            layout,
            PilMailboxLayout {
                size_bytes: 20,
                sensor_capacity: 3,
                command_capacity: 5,
                protocol_version_offset: 0,
                status_offset: 2,
                error_offset: 3,
                sensor_len_offset: 4,
                command_len_offset: 8,
                sensor_bytes_offset: 12,
                command_bytes_offset: 15,
            }
        );
    }

    #[test]
    fn pil_wire_step_decodes_sensor_runs_fc_and_encodes_command() {
        let mut fc = controller_for_packet_step();
        let payload = encode(&full_sensor_packet()).unwrap();
        let encoded = fc_step_from_wire(&mut fc, &payload).unwrap();
        let command: ActuatorCommandPacket = decode(&encoded).unwrap();

        assert_eq!(command.sim_time_s, 0.25);
        assert_eq!(command.step, 42);
        assert_eq!(command.effector_commands, vec![(7, 9.81)]);
        assert_eq!(command.engine_throttles, vec![(9, 0.64)]);
        assert_eq!(
            command.engine_commands,
            vec![BridgeEngineCommandPacket {
                engine_id: 9,
                throttle_unit: 0.64,
                gimbal_pitch_rad: 0.01,
                gimbal_yaw_rad: -0.02,
                ignite: true,
                shutdown: false,
            }]
        );
    }

    #[test]
    fn pil_wire_step_writes_into_caller_buffer() {
        let mut fc = controller_for_packet_step();
        let payload = encode(&full_sensor_packet()).unwrap();
        let mut output = vec![0; 256];
        let written = fc_step_from_wire_into(&mut fc, &payload, &mut output).unwrap();
        let command: ActuatorCommandPacket = decode(&output[..written]).unwrap();

        assert_eq!(command.step, 42);
        assert_eq!(command.effector_commands, vec![(7, 9.81)]);
    }

    #[test]
    fn pil_wire_step_fails_when_output_buffer_is_too_small() {
        let mut fc = controller_for_packet_step();
        let payload = encode(&full_sensor_packet()).unwrap();
        let mut output = [0u8; 1];

        let err = fc_step_from_wire_into(&mut fc, &payload, &mut output)
            .expect_err("single byte cannot hold encoded command packet");

        assert!(matches!(
            err,
            PilStepError::OutputBufferTooSmall {
                needed: _,
                capacity: 1
            }
        ));
    }

    #[test]
    fn pil_mailbox_steps_loaded_sensor_payload() {
        let mut fc = controller_for_packet_step();
        let payload = encode(&full_sensor_packet()).unwrap();
        let mut mailbox: PilMailbox<256, 256> = PilMailbox::new();

        mailbox.load_sensor_payload(&payload).unwrap();
        let status = step_mailbox(&mut fc, &mut mailbox);

        assert_eq!(status, PilMailboxStatus::CommandReady);
        assert_eq!(mailbox.status, PilMailboxStatus::CommandReady);
        assert_eq!(mailbox.error, PilMailboxError::None);
        assert_eq!(mailbox.protocol_version, PROTOCOL_VERSION);

        let command: ActuatorCommandPacket = decode(mailbox.command_payload().unwrap()).unwrap();
        assert_eq!(command.step, 42);
        assert_eq!(command.effector_commands, vec![(7, 9.81)]);
        assert_eq!(command.engine_throttles, vec![(9, 0.64)]);
    }

    #[test]
    fn pil_mailbox_fails_closed_on_small_command_storage() {
        let mut fc = controller_for_packet_step();
        let payload = encode(&full_sensor_packet()).unwrap();
        let mut mailbox: PilMailbox<256, 1> = PilMailbox::new();

        mailbox.load_sensor_payload(&payload).unwrap();
        let status = mailbox.step(&mut fc);

        assert_eq!(status, PilMailboxStatus::Fault);
        assert_eq!(mailbox.status, PilMailboxStatus::Fault);
        assert_eq!(mailbox.error, PilMailboxError::OutputBufferTooSmall);
        assert_eq!(mailbox.command_len, 0);
    }

    #[test]
    fn pil_mailbox_rejects_oversized_sensor_payload() {
        let mut mailbox: PilMailbox<1, 256> = PilMailbox::new();
        let payload = encode(&full_sensor_packet()).unwrap();

        let err = mailbox
            .load_sensor_payload(&payload)
            .expect_err("payload does not fit in mailbox input storage");

        assert_eq!(err, PilMailboxError::SensorTooLarge);
        assert_eq!(mailbox.status, PilMailboxStatus::Fault);
        assert_eq!(mailbox.error, PilMailboxError::SensorTooLarge);
    }

    #[test]
    fn pil_mailbox_invalid_sensor_length_faults_before_dispatch() {
        let mut fc = controller_for_packet_step();
        let mut mailbox: PilMailbox<1, 256> = PilMailbox::new();
        mailbox.sensor_len = 2;

        let status = mailbox.step(&mut fc);

        assert_eq!(status, PilMailboxStatus::Fault);
        assert_eq!(mailbox.status, PilMailboxStatus::Fault);
        assert_eq!(mailbox.error, PilMailboxError::InvalidSensorLength);
    }

    #[test]
    fn pil_mailbox_rejects_invalid_command_length_on_read() {
        let mut mailbox: PilMailbox<256, 1> = PilMailbox::new();
        mailbox.command_len = 2;

        let err = mailbox
            .command_payload()
            .expect_err("command length points outside mailbox storage");

        assert_eq!(err, PilMailboxError::InvalidCommandLength);
    }

    #[test]
    fn pil_framed_step_decodes_sensor_frame_and_writes_command_frame() {
        let mut fc = controller_for_packet_step();
        let sensor_payload = encode(&full_sensor_packet()).unwrap();
        let sensor_frame = frame(&sensor_payload);
        let mut command_frame = [0u8; 256];

        let step = fc_step_from_frame_into(&mut fc, &sensor_frame, &mut command_frame).unwrap();
        let (command_payload, consumed) = openbmp_bridge::deframe(&command_frame[..step.written])
            .expect("command frame should deframe");
        let command: ActuatorCommandPacket = decode(command_payload).unwrap();

        assert_eq!(
            step,
            PilFrameStep {
                consumed: sensor_frame.len(),
                written: consumed
            }
        );
        assert_eq!(command.step, 42);
        assert_eq!(command.effector_commands, vec![(7, 9.81)]);
        assert_eq!(command.engine_throttles, vec![(9, 0.64)]);
    }

    #[test]
    fn pil_framed_step_allows_trailing_input_frame_bytes() {
        let mut fc = controller_for_packet_step();
        let sensor_payload = encode(&full_sensor_packet()).unwrap();
        let mut sensor_frame = frame(&sensor_payload);
        sensor_frame.extend_from_slice(&[0xaa, 0xbb]);
        let mut command_frame = [0u8; 256];

        let step = fc_step_from_frame_into(&mut fc, &sensor_frame, &mut command_frame).unwrap();

        assert_eq!(step.consumed, sensor_frame.len() - 2);
    }

    #[test]
    fn pil_framed_step_reports_small_command_frame_buffer() {
        let mut fc = controller_for_packet_step();
        let sensor_payload = encode(&full_sensor_packet()).unwrap();
        let sensor_frame = frame(&sensor_payload);
        let mut command_frame = [0u8; 4];

        let err = fc_step_from_frame_into(&mut fc, &sensor_frame, &mut command_frame)
            .expect_err("response payload cannot fit after frame prefix");

        assert!(matches!(
            err,
            PilFrameError::OutputFrameTooSmall {
                needed: _,
                capacity: 4
            }
        ));
    }

    #[test]
    fn pil_framed_step_rejects_incomplete_sensor_frame() {
        let mut fc = controller_for_packet_step();
        let mut command_frame = [0u8; 256];

        let err = fc_step_from_frame_into(&mut fc, &[1, 0, 0], &mut command_frame)
            .expect_err("prefix is incomplete");

        assert!(matches!(
            err,
            PilFrameError::IncomingFrame(BridgeError::PrefixIncomplete { have: 3 })
        ));
    }

    #[derive(Debug)]
    enum MockIoError {
        InputTooLarge,
        ReadFailed,
        WriteFailed,
    }

    struct MockFrameIo {
        read_frame: Vec<u8>,
        written_frame: Vec<u8>,
        fail_write: bool,
    }

    impl PilFrameIo for MockFrameIo {
        type Error = MockIoError;

        fn read_frame(&mut self, storage: &mut [u8]) -> Result<usize, Self::Error> {
            if self.read_frame.len() > storage.len() {
                return Err(MockIoError::InputTooLarge);
            }
            storage[..self.read_frame.len()].copy_from_slice(&self.read_frame);
            Ok(self.read_frame.len())
        }

        fn write_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
            if self.fail_write {
                return Err(MockIoError::WriteFailed);
            }
            self.written_frame.clear();
            self.written_frame.extend_from_slice(frame);
            Ok(())
        }
    }

    struct MockByteIo {
        read_chunks: Vec<Vec<u8>>,
        read_index: usize,
        written_frames: Vec<Vec<u8>>,
        fail_read: bool,
        fail_write: bool,
    }

    impl MockByteIo {
        fn new(read_chunks: Vec<Vec<u8>>) -> Self {
            Self {
                read_chunks,
                read_index: 0,
                written_frames: Vec::new(),
                fail_read: false,
                fail_write: false,
            }
        }
    }

    impl PilByteIo for MockByteIo {
        type Error = MockIoError;

        fn read_bytes(&mut self, storage: &mut [u8]) -> Result<usize, Self::Error> {
            if self.fail_read {
                return Err(MockIoError::ReadFailed);
            }
            let Some(chunk) = self.read_chunks.get(self.read_index) else {
                return Ok(0);
            };
            if chunk.len() > storage.len() {
                return Err(MockIoError::InputTooLarge);
            }
            storage[..chunk.len()].copy_from_slice(chunk);
            self.read_index += 1;
            Ok(chunk.len())
        }

        fn write_bytes(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
            if self.fail_write {
                return Err(MockIoError::WriteFailed);
            }
            self.written_frames.push(frame.to_vec());
            Ok(())
        }
    }

    #[test]
    fn pil_endpoint_once_reads_steps_and_writes_frame() {
        let mut fc = controller_for_packet_step();
        let sensor_payload = encode(&full_sensor_packet()).unwrap();
        let sensor_frame = frame(&sensor_payload);
        let mut io = MockFrameIo {
            read_frame: sensor_frame.clone(),
            written_frame: Vec::new(),
            fail_write: false,
        };
        let mut input_storage = [0u8; 256];
        let mut output_storage = [0u8; 256];

        let step =
            run_framed_endpoint_once(&mut fc, &mut io, &mut input_storage, &mut output_storage)
                .unwrap();
        let (payload, consumed) =
            openbmp_bridge::deframe(&io.written_frame).expect("written command frame");
        let command: ActuatorCommandPacket = decode(payload).unwrap();

        assert_eq!(step.consumed, sensor_frame.len());
        assert_eq!(step.written, consumed);
        assert_eq!(command.step, 42);
        assert_eq!(command.effector_commands, vec![(7, 9.81)]);
    }

    #[test]
    fn pil_byte_stream_endpoint_reads_split_chunks_and_writes_command() {
        let mut fc = controller_for_packet_step();
        let sensor_payload = encode(&full_sensor_packet()).unwrap();
        let sensor_frame = frame(&sensor_payload);
        let mut io = MockByteIo::new(vec![sensor_frame[..3].to_vec(), sensor_frame[3..].to_vec()]);
        let mut pump: PilFramePump<256, 256> = PilFramePump::new();
        let mut read_storage = [0u8; 256];

        let first_status =
            poll_byte_stream_endpoint_once(&mut fc, &mut io, &mut pump, &mut read_storage).unwrap();
        assert_eq!(
            first_status,
            PilByteEndpointStatus::Waiting {
                read: 3,
                buffered: 3
            }
        );
        assert_eq!(io.written_frames.len(), 0);

        let second_status =
            poll_byte_stream_endpoint_once(&mut fc, &mut io, &mut pump, &mut read_storage).unwrap();
        assert_eq!(io.written_frames.len(), 1);
        assert_eq!(
            second_status,
            PilByteEndpointStatus::CommandWritten {
                written: io.written_frames[0].len()
            }
        );
        assert_eq!(pump.buffered_len(), 0);
        assert_eq!(pump.output_frame(), None);

        let command = command_packet_from_frame(&io.written_frames[0]);
        assert_eq!(command.step, 42);
        assert_eq!(command.effector_commands, vec![(7, 9.81)]);
        assert_eq!(command.engine_throttles, vec![(9, 0.64)]);
    }

    #[test]
    fn pil_byte_stream_endpoint_preserves_pending_output_after_write_failure() {
        let mut fc = controller_for_packet_step();
        let sensor_payload = encode(&full_sensor_packet()).unwrap();
        let sensor_frame = frame(&sensor_payload);
        let mut io = MockByteIo::new(vec![sensor_frame]);
        io.fail_write = true;
        let mut pump: PilFramePump<256, 256> = PilFramePump::new();
        let mut read_storage = [0u8; 256];

        let err = poll_byte_stream_endpoint_once(&mut fc, &mut io, &mut pump, &mut read_storage)
            .expect_err("write failure should be reported");
        assert!(matches!(
            err,
            PilByteEndpointError::Write(MockIoError::WriteFailed)
        ));
        assert!(pump.output_frame().is_some());
        assert_eq!(io.written_frames.len(), 0);

        io.fail_write = false;
        let status =
            poll_byte_stream_endpoint_once(&mut fc, &mut io, &mut pump, &mut read_storage).unwrap();
        assert_eq!(io.written_frames.len(), 1);
        assert_eq!(
            status,
            PilByteEndpointStatus::CommandWritten {
                written: io.written_frames[0].len()
            }
        );
        assert_eq!(pump.output_frame(), None);

        let command = command_packet_from_frame(&io.written_frames[0]);
        assert_eq!(command.step, 42);
    }

    #[test]
    fn pil_firmware_symbol_contract_reports_mailbox_layout() {
        let firmware: PilFirmware<256, 512, 256, 512, 64> =
            PilFirmware::new(controller_for_packet_step());
        let contract = firmware.symbol_contract();

        assert_eq!(contract.mailbox_symbol, OPENBMP_PIL_MAILBOX_SYMBOL);
        assert_eq!(contract.step_entry_symbol, OPENBMP_PIL_STEP_MAILBOX_SYMBOL);
        assert_eq!(contract.mailbox_layout, PilMailbox::<256, 512>::layout());
        assert_eq!(firmware.mailbox_layout(), contract.mailbox_layout);
        assert_eq!(firmware.mailbox().status, PilMailboxStatus::Empty);
        assert_eq!(firmware.pump().buffered_len(), 0);
    }

    #[test]
    fn pil_firmware_mailbox_entry_steps_loaded_payload() {
        let mut firmware: PilFirmware<256, 256, 256, 256, 64> =
            PilFirmware::new(controller_for_packet_step());
        let payload = encode(&full_sensor_packet()).unwrap();

        firmware.load_mailbox_sensor_payload(&payload).unwrap();
        let status = firmware.step_mailbox();

        assert_eq!(status, PilMailboxStatus::CommandReady);
        let command: ActuatorCommandPacket =
            decode(firmware.mailbox().command_payload().unwrap()).unwrap();
        assert_eq!(command.step, 42);
        assert_eq!(command.effector_commands, vec![(7, 9.81)]);
        assert_eq!(command.engine_throttles, vec![(9, 0.64)]);
    }

    #[test]
    fn pil_firmware_byte_poll_uses_internal_read_storage() {
        let mut firmware: PilFirmware<256, 256, 256, 256, 256> =
            PilFirmware::new(controller_for_packet_step());
        let sensor_frame = frame(&encode(&full_sensor_packet()).unwrap());
        let split_at = 6;
        let mut io = MockByteIo::new(vec![
            sensor_frame[..split_at].to_vec(),
            sensor_frame[split_at..].to_vec(),
        ]);

        let first = firmware.poll_byte_stream_once(&mut io).unwrap();
        assert_eq!(
            first,
            PilByteEndpointStatus::Waiting {
                read: split_at,
                buffered: split_at
            }
        );

        let second = firmware.poll_byte_stream_once(&mut io).unwrap();
        assert_eq!(io.written_frames.len(), 1);
        assert_eq!(
            second,
            PilByteEndpointStatus::CommandWritten {
                written: io.written_frames[0].len()
            }
        );
        let command = command_packet_from_frame(&io.written_frames[0]);
        assert_eq!(command.step, 42);
        assert_eq!(firmware.pump().buffered_len(), 0);
    }

    #[test]
    fn pil_wcet_budget_reports_integer_deadline_margin() {
        let budget = PilWcetBudget::new(20_000, 1_250, 100_000_000, 1_000).unwrap();
        let report = budget.report().unwrap();

        assert_eq!(
            report,
            PilWcetReport {
                instruction_count: 20_000,
                cycles_per_instruction_milli: 1_250,
                cpu_hz: 100_000_000,
                frame_period_us: 1_000,
                worst_case_cycles: 25_000,
                worst_case_time_us: 250,
                deadline_margin_us: 750,
                meets_deadline: true,
            }
        );
        assert_eq!(budget.deadline_margin_us().unwrap(), 750);
        assert!(budget.meets_deadline().unwrap());
    }

    #[test]
    fn pil_wcet_budget_detects_deadline_miss_and_bad_inputs() {
        let budget = PilWcetBudget::new(200_000, 2_000, 100_000_000, 1_000).unwrap();
        let report = budget.report().unwrap();

        assert_eq!(report.worst_case_cycles, 400_000);
        assert_eq!(report.worst_case_time_us, 4_000);
        assert_eq!(report.deadline_margin_us, -3_000);
        assert!(!report.meets_deadline);

        assert_eq!(
            PilWcetBudget::new(0, 1_000, 100_000_000, 1_000),
            Err(PilWcetBudgetError::ZeroInstructionCount)
        );
        assert_eq!(
            PilWcetBudget::new(1, 0, 100_000_000, 1_000),
            Err(PilWcetBudgetError::ZeroCyclesPerInstruction)
        );
        assert_eq!(
            PilWcetBudget::new(1, 1_000, 0, 1_000),
            Err(PilWcetBudgetError::ZeroCpuFrequency)
        );
        assert_eq!(
            PilWcetBudget::new(1, 1_000, 100_000_000, 0),
            Err(PilWcetBudgetError::ZeroFramePeriod)
        );
        assert_eq!(
            PilWcetBudget::new(u64::MAX, u32::MAX, 1, 1)
                .unwrap()
                .worst_case_cycles_ceil(),
            Err(PilWcetBudgetError::CycleOverflow)
        );
    }

    #[test]
    fn pil_byte_stream_endpoint_reports_read_errors() {
        let mut fc = controller_for_packet_step();
        let mut io = MockByteIo::new(Vec::new());
        io.fail_read = true;
        let mut pump: PilFramePump<256, 256> = PilFramePump::new();
        let mut read_storage = [0u8; 256];

        let err = poll_byte_stream_endpoint_once(&mut fc, &mut io, &mut pump, &mut read_storage)
            .expect_err("read failure should be reported");

        assert!(matches!(
            err,
            PilByteEndpointError::Read(MockIoError::ReadFailed)
        ));
        assert_eq!(pump.buffered_len(), 0);
        assert_eq!(pump.output_frame(), None);
    }

    #[test]
    fn pil_endpoint_once_reports_write_errors() {
        let mut fc = controller_for_packet_step();
        let sensor_payload = encode(&full_sensor_packet()).unwrap();
        let sensor_frame = frame(&sensor_payload);
        let mut io = MockFrameIo {
            read_frame: sensor_frame,
            written_frame: Vec::new(),
            fail_write: true,
        };
        let mut input_storage = [0u8; 256];
        let mut output_storage = [0u8; 256];

        let err =
            run_framed_endpoint_once(&mut fc, &mut io, &mut input_storage, &mut output_storage)
                .expect_err("write should fail");

        assert!(matches!(
            err,
            PilEndpointError::Write(MockIoError::WriteFailed)
        ));
    }

    #[test]
    fn pil_frame_pump_waits_for_split_frame_then_outputs_command() {
        let mut fc = controller_for_packet_step();
        let sensor_payload = encode(&full_sensor_packet()).unwrap();
        let sensor_frame = frame(&sensor_payload);
        let mut pump: PilFramePump<256, 256> = PilFramePump::new();
        let split_at = 3;

        assert_eq!(pump.ingest(&sensor_frame[..split_at]).unwrap(), split_at);
        assert_eq!(
            pump.try_step(&mut fc).unwrap(),
            PilFramePumpStatus::Waiting { buffered: split_at }
        );
        assert_eq!(pump.output_frame(), None);

        assert_eq!(
            pump.ingest(&sensor_frame[split_at..]).unwrap(),
            sensor_frame.len()
        );
        let status = pump.try_step(&mut fc).unwrap();
        let output_frame = pump
            .output_frame()
            .expect("command frame should be pending");
        assert_eq!(
            status,
            PilFramePumpStatus::CommandReady {
                written: output_frame.len()
            }
        );
        assert_eq!(pump.buffered_len(), 0);

        let command = command_packet_from_frame(output_frame);
        assert_eq!(command.step, 42);
        assert_eq!(command.effector_commands, vec![(7, 9.81)]);
        assert_eq!(command.engine_throttles, vec![(9, 0.64)]);
    }

    #[test]
    fn pil_frame_pump_blocks_step_until_output_is_cleared() {
        let mut fc = controller_for_packet_step();
        let sensor_payload = encode(&full_sensor_packet()).unwrap();
        let sensor_frame = frame(&sensor_payload);
        let mut pump: PilFramePump<256, 256> = PilFramePump::new();

        pump.ingest(&sensor_frame).unwrap();
        let status = pump.try_step(&mut fc).unwrap();
        let output_len = pump
            .output_frame()
            .expect("command frame should be pending")
            .len();
        assert_eq!(
            status,
            PilFramePumpStatus::CommandReady {
                written: output_len
            }
        );

        let err = pump
            .try_step(&mut fc)
            .expect_err("pending output must be transmitted or cleared first");
        assert!(matches!(
            err,
            PilFramePumpError::OutputPending { len } if len == output_len
        ));

        pump.clear_output_frame();
        assert_eq!(pump.output_frame(), None);
        assert_eq!(
            pump.try_step(&mut fc).unwrap(),
            PilFramePumpStatus::Waiting { buffered: 0 }
        );
    }

    #[test]
    fn pil_frame_pump_preserves_queued_frames_after_one_step() {
        let mut fc = controller_for_packet_step();
        let first_frame = frame(&encode(&full_sensor_packet()).unwrap());
        let mut second_packet = full_sensor_packet();
        second_packet.sim_time_s = 0.251;
        second_packet.step = 43;
        let second_frame = frame(&encode(&second_packet).unwrap());
        let mut stream = first_frame.clone();
        stream.extend_from_slice(&second_frame);
        let mut pump: PilFramePump<512, 256> = PilFramePump::new();

        pump.ingest(&stream).unwrap();
        let first_status = pump.try_step(&mut fc).unwrap();
        let first_output = pump
            .output_frame()
            .expect("first command frame should be pending");
        assert_eq!(
            first_status,
            PilFramePumpStatus::CommandReady {
                written: first_output.len()
            }
        );
        assert_eq!(pump.buffered_len(), second_frame.len());
        let first_command = command_packet_from_frame(first_output);
        assert_eq!(first_command.step, 42);

        pump.clear_output_frame();
        let second_status = pump.try_step(&mut fc).unwrap();
        let second_output = pump
            .output_frame()
            .expect("second command frame should be pending");
        assert_eq!(
            second_status,
            PilFramePumpStatus::CommandReady {
                written: second_output.len()
            }
        );
        assert_eq!(pump.buffered_len(), 0);
        let second_command = command_packet_from_frame(second_output);
        assert_eq!(second_command.sim_time_s, 0.251);
        assert_eq!(second_command.step, 43);
    }

    #[test]
    fn pil_frame_pump_reports_input_overflow() {
        let mut pump: PilFramePump<4, 256> = PilFramePump::new();

        let err = pump
            .ingest(&[0, 1, 2, 3, 4])
            .expect_err("input chunk exceeds pump receive buffer");

        assert!(matches!(
            err,
            PilFramePumpError::InputOverflow {
                capacity: 4,
                needed: 5
            }
        ));
        assert_eq!(pump.buffered_len(), 0);
        assert_eq!(pump.output_frame(), None);
    }

    #[test]
    fn pil_packet_step_rejects_out_of_range_command_ids() {
        struct OutOfRangeJob;

        impl Job for OutOfRangeJob {
            fn name(&self) -> &'static str {
                "test.pil_out_of_range"
            }

            fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
                let mut effector_set = EffectorCommandSet {
                    time: SimTime::ZERO,
                    count: 1,
                    ..EffectorCommandSet::default()
                };
                effector_set.commands[0] = EffectorCommand {
                    effector_id: u64::from(u32::MAX) + 1,
                    command: 1.0,
                    saturated: false,
                };
                ctx.bus.publish(effector_set)?;
                Ok(())
            }
        }

        let mut fc = FlightControllerBuilder::new()
            .frame_budget_us(1_000)
            .build();
        register_input_topics(&fc);
        register_output_topics(&fc);
        fc.scheduler_mut()
            .register_periodic(1, 100, 1, Box::new(OutOfRangeJob))
            .unwrap();

        let err = fc_step_from_sensor_packet(&mut fc, &full_sensor_packet())
            .expect_err("out-of-range effector id should fail closed");

        assert!(matches!(
            err,
            PilStepError::CommandIdOutOfRange {
                kind: "effector",
                id
            } if id == u64::from(u32::MAX) + 1
        ));
    }
}
