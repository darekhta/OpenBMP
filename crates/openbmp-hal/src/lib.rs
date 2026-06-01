//! Hardware abstraction traits for OpenBMP flight-core adapters.
//!
//! This crate is the L1 contract surface between reusable GNC code and
//! a board or simulator backend. It intentionally carries no concrete
//! sensor, actuator, simulator, filesystem, thread, or transport
//! implementation. A host runner can implement these traits against
//! synthetic sensors and racks; a downstream board crate can implement
//! them against real device drivers.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![deny(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use openbmp_core::SimTime;

/// HAL result alias.
pub type HalResult<T, E> = core::result::Result<T, E>;

/// Magic bytes at the start of every serialized I-load envelope.
pub const ILOAD_MAGIC: [u8; 4] = *b"OBIL";
/// Current I-load envelope schema version.
pub const ILOAD_ENVELOPE_VERSION: u16 = 1;
/// Size of the fixed I-load envelope header in bytes.
pub const ILOAD_HEADER_LEN: usize = 16;

/// A verified initialization-load payload borrowed from a storage
/// buffer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct VerifiedIload<'a> {
    /// Flight I-load payload schema version.
    pub schema_version: u16,
    /// CRC-32/IEEE checksum of [`Self::payload`].
    pub payload_crc32: u32,
    /// Compact serialized flight I-load payload.
    pub payload: &'a [u8],
}

/// Error raised while encoding or validating an I-load envelope.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IloadEnvelopeError {
    /// The caller-provided buffer is too small.
    BufferTooSmall {
        /// Required buffer length in bytes.
        required: usize,
        /// Actual buffer length in bytes.
        actual: usize,
    },
    /// Payload is larger than the envelope header can represent.
    PayloadTooLarge {
        /// Payload length in bytes.
        len: usize,
    },
    /// Envelope magic bytes did not match [`ILOAD_MAGIC`].
    InvalidMagic {
        /// Magic bytes found in the input.
        found: [u8; 4],
    },
    /// Envelope version is not supported by this decoder.
    UnsupportedEnvelopeVersion {
        /// Envelope version found in the input.
        found: u16,
        /// Envelope version supported by this decoder.
        supported: u16,
    },
    /// Payload schema version is not accepted by the caller.
    UnsupportedSchemaVersion {
        /// Payload schema version found in the envelope.
        found: u16,
    },
    /// Envelope payload length disagrees with the input length.
    LengthMismatch {
        /// Total byte length expected from the header.
        expected: usize,
        /// Actual input byte length.
        actual: usize,
    },
    /// Payload CRC did not match the header.
    CrcMismatch {
        /// CRC declared by the envelope.
        expected: u32,
        /// CRC computed over the payload.
        actual: u32,
    },
}

impl core::fmt::Display for IloadEnvelopeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BufferTooSmall { required, actual } => {
                write!(
                    f,
                    "I-load buffer too small: required {required} bytes, got {actual}"
                )
            }
            Self::PayloadTooLarge { len } => {
                write!(f, "I-load payload is too large: {len} bytes")
            }
            Self::InvalidMagic { found } => {
                write!(f, "invalid I-load magic bytes: {found:?}")
            }
            Self::UnsupportedEnvelopeVersion { found, supported } => write!(
                f,
                "unsupported I-load envelope version {found}; supported version is {supported}"
            ),
            Self::UnsupportedSchemaVersion { found } => {
                write!(f, "unsupported I-load payload schema version {found}")
            }
            Self::LengthMismatch { expected, actual } => {
                write!(
                    f,
                    "I-load length mismatch: expected {expected} bytes, got {actual}"
                )
            }
            Self::CrcMismatch { expected, actual } => write!(
                f,
                "I-load CRC mismatch: expected 0x{expected:08x}, got 0x{actual:08x}"
            ),
        }
    }
}

/// Source selected when loading a primary I-load with an optional
/// fallback image.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IloadSource {
    /// Primary I-load validated successfully.
    Primary,
    /// Primary I-load failed validation and the fallback image was
    /// selected.
    Fallback,
}

/// Successful primary/fallback I-load selection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IloadSelection<'a> {
    /// Selected storage image.
    pub source: IloadSource,
    /// Verified I-load payload.
    pub iload: VerifiedIload<'a>,
}

/// Error returned when neither the primary nor fallback I-load image
/// can be validated.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IloadLoadError {
    /// Primary-image validation error.
    pub primary: IloadEnvelopeError,
    /// Fallback-image validation error, if a fallback was supplied.
    pub fallback: Option<IloadEnvelopeError>,
}

impl core::fmt::Display for IloadLoadError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.fallback {
            Some(fallback) => write!(
                f,
                "primary I-load rejected ({primary}); fallback rejected ({fallback})",
                primary = self.primary
            ),
            None => write!(f, "primary I-load rejected ({})", self.primary),
        }
    }
}

/// Encode a compact payload into a versioned, CRC-protected I-load
/// envelope.
///
/// Header layout, little-endian:
/// `magic[4]`, `envelope_version:u16`, `schema_version:u16`,
/// `payload_len:u32`, `payload_crc32:u32`.
///
/// # Errors
///
/// Returns [`IloadEnvelopeError::BufferTooSmall`] when `out` cannot
/// hold the header and payload, or
/// [`IloadEnvelopeError::PayloadTooLarge`] when the payload length
/// cannot be represented by the envelope.
pub fn encode_iload_envelope(
    schema_version: u16,
    payload: &[u8],
    out: &mut [u8],
) -> HalResult<usize, IloadEnvelopeError> {
    let payload_len = u32::try_from(payload.len())
        .map_err(|_| IloadEnvelopeError::PayloadTooLarge { len: payload.len() })?;
    let total_len = ILOAD_HEADER_LEN
        .checked_add(payload.len())
        .ok_or(IloadEnvelopeError::PayloadTooLarge { len: payload.len() })?;
    if out.len() < total_len {
        return Err(IloadEnvelopeError::BufferTooSmall {
            required: total_len,
            actual: out.len(),
        });
    }

    out[..4].copy_from_slice(&ILOAD_MAGIC);
    out[4..6].copy_from_slice(&ILOAD_ENVELOPE_VERSION.to_le_bytes());
    out[6..8].copy_from_slice(&schema_version.to_le_bytes());
    out[8..12].copy_from_slice(&payload_len.to_le_bytes());
    let crc = crc32_ieee(payload);
    out[12..16].copy_from_slice(&crc.to_le_bytes());
    out[ILOAD_HEADER_LEN..total_len].copy_from_slice(payload);
    Ok(total_len)
}

/// Validate and borrow a serialized I-load payload from an envelope.
///
/// `supported_schema_versions` is supplied by the flight image so a
/// stale or too-new host-generated payload fails closed before any
/// deserializer runs.
///
/// # Errors
///
/// Returns [`IloadEnvelopeError`] when the header, length, schema
/// version, or payload CRC is invalid.
pub fn decode_iload_envelope<'a>(
    input: &'a [u8],
    supported_schema_versions: &[u16],
) -> HalResult<VerifiedIload<'a>, IloadEnvelopeError> {
    if input.len() < ILOAD_HEADER_LEN {
        return Err(IloadEnvelopeError::BufferTooSmall {
            required: ILOAD_HEADER_LEN,
            actual: input.len(),
        });
    }

    let magic = [input[0], input[1], input[2], input[3]];
    if magic != ILOAD_MAGIC {
        return Err(IloadEnvelopeError::InvalidMagic { found: magic });
    }
    let envelope_version = u16::from_le_bytes([input[4], input[5]]);
    if envelope_version != ILOAD_ENVELOPE_VERSION {
        return Err(IloadEnvelopeError::UnsupportedEnvelopeVersion {
            found: envelope_version,
            supported: ILOAD_ENVELOPE_VERSION,
        });
    }
    let schema_version = u16::from_le_bytes([input[6], input[7]]);
    if !supported_schema_versions.contains(&schema_version) {
        return Err(IloadEnvelopeError::UnsupportedSchemaVersion {
            found: schema_version,
        });
    }
    let payload_len_u32 = u32::from_le_bytes([input[8], input[9], input[10], input[11]]);
    let payload_len = usize::try_from(payload_len_u32)
        .map_err(|_| IloadEnvelopeError::PayloadTooLarge { len: usize::MAX })?;
    let expected_len = ILOAD_HEADER_LEN
        .checked_add(payload_len)
        .ok_or(IloadEnvelopeError::PayloadTooLarge { len: payload_len })?;
    if input.len() != expected_len {
        return Err(IloadEnvelopeError::LengthMismatch {
            expected: expected_len,
            actual: input.len(),
        });
    }

    let expected_crc = u32::from_le_bytes([input[12], input[13], input[14], input[15]]);
    let payload = &input[ILOAD_HEADER_LEN..expected_len];
    let actual_crc = crc32_ieee(payload);
    if actual_crc != expected_crc {
        return Err(IloadEnvelopeError::CrcMismatch {
            expected: expected_crc,
            actual: actual_crc,
        });
    }

    Ok(VerifiedIload {
        schema_version,
        payload_crc32: expected_crc,
        payload,
    })
}

/// Validate a primary I-load image and fall back to a secondary image
/// if the primary fails.
///
/// # Errors
///
/// Returns [`IloadLoadError`] only when the primary image is invalid
/// and no valid fallback image is supplied.
pub fn decode_iload_or_fallback<'a>(
    primary: &'a [u8],
    fallback: Option<&'a [u8]>,
    supported_schema_versions: &[u16],
) -> HalResult<IloadSelection<'a>, IloadLoadError> {
    match decode_iload_envelope(primary, supported_schema_versions) {
        Ok(iload) => Ok(IloadSelection {
            source: IloadSource::Primary,
            iload,
        }),
        Err(primary_error) => {
            if let Some(fallback) = fallback {
                match decode_iload_envelope(fallback, supported_schema_versions) {
                    Ok(iload) => Ok(IloadSelection {
                        source: IloadSource::Fallback,
                        iload,
                    }),
                    Err(fallback_error) => Err(IloadLoadError {
                        primary: primary_error,
                        fallback: Some(fallback_error),
                    }),
                }
            } else {
                Err(IloadLoadError {
                    primary: primary_error,
                    fallback: None,
                })
            }
        }
    }
}

/// Magic bytes at the start of every flight-recorder frame.
pub const FLIGHT_RECORD_MAGIC: [u8; 4] = *b"OBFR";
/// Current flight-recorder frame schema version.
pub const FLIGHT_RECORD_VERSION: u16 = 1;
/// Size of the fixed flight-recorder frame header in bytes.
pub const FLIGHT_RECORD_HEADER_LEN: usize = 32;

/// Common flight-record kind identifiers.
pub mod flight_record_kind {
    /// Encoded topic snapshot.
    pub const TOPIC_SNAPSHOT: u16 = 1;
    /// Health or FDIR event record.
    pub const HEALTH_EVENT: u16 = 2;
    /// Scheduler timing or overrun record.
    pub const SCHEDULER_EVENT: u16 = 3;
    /// Board or vendor-specific record. Payload schema is owned by
    /// the downstream board crate.
    pub const VENDOR: u16 = 0xffff;
}

/// Error raised while encoding, decoding, or buffering flight-recorder
/// frames.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FlightRecordError {
    /// The caller-provided buffer is too small.
    BufferTooSmall {
        /// Required buffer length in bytes.
        required: usize,
        /// Actual buffer length in bytes.
        actual: usize,
    },
    /// Payload is larger than the record header or fixed recorder slot
    /// can represent.
    PayloadTooLarge {
        /// Payload length in bytes.
        len: usize,
        /// Maximum accepted payload length in bytes.
        max: usize,
    },
    /// Record timestamp is NaN, infinite, or before the monotonic
    /// timeline origin.
    InvalidTimestamp {
        /// Timestamp in seconds.
        seconds: f64,
    },
    /// Recorder queue is full and configured to reject new records.
    RecorderFull {
        /// Queue capacity in records.
        capacity: usize,
    },
    /// Record magic bytes did not match [`FLIGHT_RECORD_MAGIC`].
    InvalidMagic {
        /// Magic bytes found in the input.
        found: [u8; 4],
    },
    /// Record version is not supported by this decoder.
    UnsupportedVersion {
        /// Version found in the input.
        found: u16,
        /// Version supported by this decoder.
        supported: u16,
    },
    /// Record payload length disagrees with the input length.
    LengthMismatch {
        /// Total byte length expected from the header.
        expected: usize,
        /// Actual input byte length.
        actual: usize,
    },
    /// Payload CRC did not match the header.
    CrcMismatch {
        /// CRC declared by the header.
        expected: u32,
        /// CRC computed over the payload.
        actual: u32,
    },
}

impl core::fmt::Display for FlightRecordError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BufferTooSmall { required, actual } => write!(
                f,
                "flight-record buffer too small: required {required} bytes, got {actual}"
            ),
            Self::PayloadTooLarge { len, max } => write!(
                f,
                "flight-record payload is too large: {len} bytes, max {max} bytes"
            ),
            Self::InvalidTimestamp { seconds } => {
                write!(f, "invalid flight-record timestamp: {seconds} s")
            }
            Self::RecorderFull { capacity } => {
                write!(f, "flight-recorder queue is full at {capacity} records")
            }
            Self::InvalidMagic { found } => {
                write!(f, "invalid flight-record magic bytes: {found:?}")
            }
            Self::UnsupportedVersion { found, supported } => write!(
                f,
                "unsupported flight-record version {found}; supported version is {supported}"
            ),
            Self::LengthMismatch { expected, actual } => write!(
                f,
                "flight-record length mismatch: expected {expected} bytes, got {actual}"
            ),
            Self::CrcMismatch { expected, actual } => write!(
                f,
                "flight-record CRC mismatch: expected 0x{expected:08x}, got 0x{actual:08x}"
            ),
        }
    }
}

/// Borrowed view of a decoded flight-recorder frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightRecordView<'a> {
    /// Application-defined record kind.
    pub kind: u16,
    /// Monotonic recorder sequence.
    pub sequence: u64,
    /// Monotonic mission timestamp.
    pub time: SimTime,
    /// CRC-32/IEEE checksum of [`Self::payload`].
    pub payload_crc32: u32,
    /// Encoded record payload.
    pub payload: &'a [u8],
}

/// Encode one flight-recorder frame into `out`.
///
/// Header layout, little-endian:
/// `magic[4]`, `record_version:u16`, `kind:u16`,
/// `sequence:u64`, `time_s:f64`, `payload_len:u32`,
/// `payload_crc32:u32`.
///
/// # Errors
///
/// Returns [`FlightRecordError`] when the timestamp is invalid, the
/// payload is too large, or `out` cannot hold the frame.
pub fn encode_flight_record(
    kind: u16,
    sequence: u64,
    time: SimTime,
    payload: &[u8],
    out: &mut [u8],
) -> HalResult<usize, FlightRecordError> {
    if !time.is_valid() {
        return Err(FlightRecordError::InvalidTimestamp {
            seconds: time.as_seconds(),
        });
    }
    let payload_len =
        u32::try_from(payload.len()).map_err(|_| FlightRecordError::PayloadTooLarge {
            len: payload.len(),
            max: u32::MAX as usize,
        })?;
    let total_len = FLIGHT_RECORD_HEADER_LEN.checked_add(payload.len()).ok_or(
        FlightRecordError::PayloadTooLarge {
            len: payload.len(),
            max: usize::MAX - FLIGHT_RECORD_HEADER_LEN,
        },
    )?;
    if out.len() < total_len {
        return Err(FlightRecordError::BufferTooSmall {
            required: total_len,
            actual: out.len(),
        });
    }

    out[..4].copy_from_slice(&FLIGHT_RECORD_MAGIC);
    out[4..6].copy_from_slice(&FLIGHT_RECORD_VERSION.to_le_bytes());
    out[6..8].copy_from_slice(&kind.to_le_bytes());
    out[8..16].copy_from_slice(&sequence.to_le_bytes());
    out[16..24].copy_from_slice(&time.as_seconds().to_le_bytes());
    out[24..28].copy_from_slice(&payload_len.to_le_bytes());
    let crc = crc32_ieee(payload);
    out[28..32].copy_from_slice(&crc.to_le_bytes());
    out[FLIGHT_RECORD_HEADER_LEN..total_len].copy_from_slice(payload);
    Ok(total_len)
}

/// Decode and validate a flight-recorder frame.
///
/// # Errors
///
/// Returns [`FlightRecordError`] when the header, length, timestamp,
/// version, or payload CRC is invalid.
pub fn decode_flight_record(input: &[u8]) -> HalResult<FlightRecordView<'_>, FlightRecordError> {
    if input.len() < FLIGHT_RECORD_HEADER_LEN {
        return Err(FlightRecordError::BufferTooSmall {
            required: FLIGHT_RECORD_HEADER_LEN,
            actual: input.len(),
        });
    }
    let magic = [input[0], input[1], input[2], input[3]];
    if magic != FLIGHT_RECORD_MAGIC {
        return Err(FlightRecordError::InvalidMagic { found: magic });
    }
    let version = u16::from_le_bytes([input[4], input[5]]);
    if version != FLIGHT_RECORD_VERSION {
        return Err(FlightRecordError::UnsupportedVersion {
            found: version,
            supported: FLIGHT_RECORD_VERSION,
        });
    }
    let kind = u16::from_le_bytes([input[6], input[7]]);
    let sequence = u64::from_le_bytes([
        input[8], input[9], input[10], input[11], input[12], input[13], input[14], input[15],
    ]);
    let time = SimTime::from_seconds(f64::from_le_bytes([
        input[16], input[17], input[18], input[19], input[20], input[21], input[22], input[23],
    ]));
    if !time.is_valid() {
        return Err(FlightRecordError::InvalidTimestamp {
            seconds: time.as_seconds(),
        });
    }
    let payload_len_u32 = u32::from_le_bytes([input[24], input[25], input[26], input[27]]);
    let payload_len =
        usize::try_from(payload_len_u32).map_err(|_| FlightRecordError::PayloadTooLarge {
            len: usize::MAX,
            max: u32::MAX as usize,
        })?;
    let expected_len = FLIGHT_RECORD_HEADER_LEN.checked_add(payload_len).ok_or(
        FlightRecordError::PayloadTooLarge {
            len: payload_len,
            max: usize::MAX - FLIGHT_RECORD_HEADER_LEN,
        },
    )?;
    if input.len() != expected_len {
        return Err(FlightRecordError::LengthMismatch {
            expected: expected_len,
            actual: input.len(),
        });
    }

    let expected_crc = u32::from_le_bytes([input[28], input[29], input[30], input[31]]);
    let payload = &input[FLIGHT_RECORD_HEADER_LEN..expected_len];
    let actual_crc = crc32_ieee(payload);
    if actual_crc != expected_crc {
        return Err(FlightRecordError::CrcMismatch {
            expected: expected_crc,
            actual: actual_crc,
        });
    }

    Ok(FlightRecordView {
        kind,
        sequence,
        time,
        payload_crc32: expected_crc,
        payload,
    })
}

/// Queue policy when a fixed-capacity flight recorder is full.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum FlightRecordOverflowPolicy {
    /// Reject the incoming record and retain existing queued records.
    DropNewest,
    /// Overwrite the oldest queued record with the incoming record.
    #[default]
    OverwriteOldest,
}

/// Stored flight record in a fixed-capacity recorder queue.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StoredFlightRecord<const RECORD_BYTES: usize> {
    kind: u16,
    sequence: u64,
    time: SimTime,
    payload_len: usize,
    payload_crc32: u32,
    payload: [u8; RECORD_BYTES],
}

impl<const RECORD_BYTES: usize> StoredFlightRecord<RECORD_BYTES> {
    const EMPTY: Self = Self {
        kind: 0,
        sequence: 0,
        time: SimTime::ZERO,
        payload_len: 0,
        payload_crc32: 0,
        payload: [0; RECORD_BYTES],
    };

    /// Application-defined record kind.
    #[must_use]
    pub const fn kind(&self) -> u16 {
        self.kind
    }

    /// Monotonic recorder sequence.
    #[must_use]
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Monotonic mission timestamp.
    #[must_use]
    pub const fn time(&self) -> SimTime {
        self.time
    }

    /// CRC-32/IEEE checksum of the payload.
    #[must_use]
    pub const fn payload_crc32(&self) -> u32 {
        self.payload_crc32
    }

    /// Stored payload bytes.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_len]
    }

    /// Encode this record into `out`.
    ///
    /// # Errors
    ///
    /// Returns [`FlightRecordError`] if `out` cannot hold the frame.
    pub fn encode(&self, out: &mut [u8]) -> HalResult<usize, FlightRecordError> {
        encode_flight_record(self.kind, self.sequence, self.time, self.payload(), out)
    }
}

/// Error returned by [`FlightRecorder::flush_oldest`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FlightRecordFlushError<E> {
    /// Frame encoding failed before the storage backend was called.
    Encode(FlightRecordError),
    /// The storage backend rejected the encoded record.
    Storage(E),
}

impl<E: core::fmt::Display> core::fmt::Display for FlightRecordFlushError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Encode(err) => write!(f, "flight-record encode error: {err}"),
            Self::Storage(err) => write!(f, "flight-record storage error: {err}"),
        }
    }
}

/// Fixed-capacity no-heap flight-recorder queue.
///
/// `CAPACITY` is the number of queued records. `RECORD_BYTES` is the
/// maximum payload length for each record. The recorder performs only
/// bounded array copies on the hot path; hardware storage latency is
/// isolated to [`Self::flush_oldest`].
#[derive(Clone, Debug, PartialEq)]
pub struct FlightRecorder<const CAPACITY: usize, const RECORD_BYTES: usize> {
    records: [StoredFlightRecord<RECORD_BYTES>; CAPACITY],
    head: usize,
    len: usize,
    next_sequence: u64,
    dropped_records: u64,
    overflow_policy: FlightRecordOverflowPolicy,
}

impl<const CAPACITY: usize, const RECORD_BYTES: usize> FlightRecorder<CAPACITY, RECORD_BYTES> {
    /// Construct an empty recorder with the given overflow policy.
    #[must_use]
    pub const fn new(overflow_policy: FlightRecordOverflowPolicy) -> Self {
        Self {
            records: [StoredFlightRecord::EMPTY; CAPACITY],
            head: 0,
            len: 0,
            next_sequence: 0,
            dropped_records: 0,
            overflow_policy,
        }
    }

    /// Number of queued records.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` when the queue contains no records.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Number of records dropped or overwritten due to queue pressure.
    #[must_use]
    pub const fn dropped_records(&self) -> u64 {
        self.dropped_records
    }

    /// Queue capacity in records.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        CAPACITY
    }

    /// Push a record into the queue.
    ///
    /// # Errors
    ///
    /// Returns [`FlightRecordError`] when the timestamp is invalid,
    /// the payload is larger than `RECORD_BYTES`, the payload length
    /// cannot be represented by the frame format, or the queue is full
    /// with [`FlightRecordOverflowPolicy::DropNewest`].
    pub fn push(
        &mut self,
        kind: u16,
        time: SimTime,
        payload: &[u8],
    ) -> HalResult<u64, FlightRecordError> {
        if !time.is_valid() {
            return Err(FlightRecordError::InvalidTimestamp {
                seconds: time.as_seconds(),
            });
        }
        if payload.len() > RECORD_BYTES {
            return Err(FlightRecordError::PayloadTooLarge {
                len: payload.len(),
                max: RECORD_BYTES,
            });
        }
        if payload.len() > u32::MAX as usize {
            return Err(FlightRecordError::PayloadTooLarge {
                len: payload.len(),
                max: u32::MAX as usize,
            });
        }
        if CAPACITY == 0 {
            self.dropped_records = self.dropped_records.saturating_add(1);
            return Err(FlightRecordError::RecorderFull { capacity: CAPACITY });
        }

        let index = if self.len == CAPACITY {
            match self.overflow_policy {
                FlightRecordOverflowPolicy::DropNewest => {
                    self.dropped_records = self.dropped_records.saturating_add(1);
                    return Err(FlightRecordError::RecorderFull { capacity: CAPACITY });
                }
                FlightRecordOverflowPolicy::OverwriteOldest => {
                    let index = self.head;
                    self.head = (self.head + 1) % CAPACITY;
                    self.dropped_records = self.dropped_records.saturating_add(1);
                    index
                }
            }
        } else {
            let index = (self.head + self.len) % CAPACITY;
            self.len += 1;
            index
        };

        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.saturating_add(1);
        let mut record = StoredFlightRecord::EMPTY;
        record.kind = kind;
        record.sequence = sequence;
        record.time = time;
        record.payload_len = payload.len();
        record.payload_crc32 = crc32_ieee(payload);
        record.payload[..payload.len()].copy_from_slice(payload);
        self.records[index] = record;
        Ok(sequence)
    }

    /// Return the oldest queued record without removing it.
    #[must_use]
    pub fn oldest(&self) -> Option<&StoredFlightRecord<RECORD_BYTES>> {
        if self.len == 0 {
            None
        } else {
            Some(&self.records[self.head])
        }
    }

    /// Remove and return the oldest queued record.
    pub fn pop_oldest(&mut self) -> Option<StoredFlightRecord<RECORD_BYTES>> {
        if self.len == 0 {
            return None;
        }
        let record = self.records[self.head];
        self.records[self.head] = StoredFlightRecord::EMPTY;
        self.head = (self.head + 1) % CAPACITY;
        self.len -= 1;
        if self.len == 0 {
            self.head = 0;
        }
        Some(record)
    }

    /// Encode and append the oldest record to a [`Storage`] backend,
    /// then remove it from the queue after storage succeeds.
    ///
    /// # Errors
    ///
    /// Returns [`FlightRecordFlushError::Encode`] when `out` is too
    /// small for the framed record, or
    /// [`FlightRecordFlushError::Storage`] when the backend rejects the
    /// append. The record remains queued on error.
    pub fn flush_oldest<S: Storage>(
        &mut self,
        storage: &mut S,
        out: &mut [u8],
    ) -> HalResult<Option<u64>, FlightRecordFlushError<S::Error>> {
        let Some(record) = self.oldest().copied() else {
            return Ok(None);
        };
        let len = record.encode(out).map_err(FlightRecordFlushError::Encode)?;
        storage
            .append_record(&out[..len])
            .map_err(FlightRecordFlushError::Storage)?;
        let _ = self.pop_oldest();
        Ok(Some(record.sequence))
    }
}

impl<const CAPACITY: usize, const RECORD_BYTES: usize> Default
    for FlightRecorder<CAPACITY, RECORD_BYTES>
{
    fn default() -> Self {
        Self::new(FlightRecordOverflowPolicy::OverwriteOldest)
    }
}

fn crc32_ieee(payload: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff_u32;
    for byte in payload {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0_u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320_u32 & mask);
        }
    }
    !crc
}

/// Monotonic runtime clock.
///
/// Simulators implement this with an integrator-driven clock. Board
/// crates implement it with a hardware monotonic counter normalized to
/// mission elapsed time.
pub trait Clock {
    /// Return the current monotonic mission time.
    #[must_use]
    fn now(&self) -> SimTime;
}

/// Typed bus topic identity for flight-portable backends.
///
/// Concrete topic payloads live above this crate. The L1 contract only
/// requires a stable name, version, and static table index so host and
/// board bus backends can agree on identity without referencing
/// simulator crates.
pub trait Topic {
    /// Stable topic name.
    const NAME: &'static str;
    /// Schema version for this topic payload.
    const VERSION: u16 = 1;
    /// Compile-time slot index inside the generated flight-topic
    /// table. This is the no-`TypeId` lookup key used by static buses.
    const INDEX: usize;
}

/// Error returned by topic payload encoders and decoders.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TopicCodecError {
    /// The caller-provided byte buffer was smaller than the payload's
    /// declared encoded length.
    BufferTooSmall {
        /// Required buffer length in bytes.
        required: usize,
        /// Actual buffer length in bytes.
        actual: usize,
    },
    /// The byte payload could not be decoded into the requested topic.
    InvalidPayload {
        /// Topic name whose payload failed to decode.
        topic_name: &'static str,
    },
}

impl core::fmt::Display for TopicCodecError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BufferTooSmall { required, actual } => {
                write!(
                    f,
                    "topic codec buffer too small: required {required} bytes, got {actual}"
                )
            }
            Self::InvalidPayload { topic_name } => {
                write!(f, "invalid encoded payload for topic {topic_name}")
            }
        }
    }
}

/// Topic payload that can be stored in a no-heap static bus.
///
/// The encoder/decoder requirement avoids `TypeId`, `Any`, raw pointer
/// casts, and heap allocation. Payload crates choose their own compact
/// representation and keep byte ordering explicit.
pub trait EncodedTopic: Topic + Copy {
    /// Exact encoded payload length in bytes.
    const ENCODED_LEN: usize;

    /// Encode `self` into `out`.
    ///
    /// # Errors
    ///
    /// Returns [`TopicCodecError`] if `out` is smaller than
    /// [`Self::ENCODED_LEN`] or the payload cannot be represented by
    /// its declared static wire format.
    fn encode(&self, out: &mut [u8]) -> HalResult<(), TopicCodecError>;

    /// Decode a payload from `input`.
    ///
    /// # Errors
    ///
    /// Returns [`TopicCodecError`] if `input` is smaller than
    /// [`Self::ENCODED_LEN`] or the payload bytes are invalid for this
    /// topic type.
    fn decode(input: &[u8]) -> HalResult<Self, TopicCodecError>;
}

/// Error raised by [`StaticBus`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StaticBusError {
    /// The topic's declared index is outside this bus's topic table.
    TopicIndexOutOfRange {
        /// Topic being read or published.
        topic_name: &'static str,
        /// Topic's declared index.
        index: usize,
        /// Bus topic capacity.
        capacity: usize,
    },
    /// The topic's encoded payload is larger than this bus's fixed
    /// per-topic slot size.
    SlotTooSmall {
        /// Topic being published or read.
        topic_name: &'static str,
        /// Topic's declared encoded length.
        encoded_len: usize,
        /// Bus slot size.
        slot_bytes: usize,
    },
    /// The topic's encoder or decoder rejected the payload.
    Codec {
        /// Topic being published or read.
        topic_name: &'static str,
        /// Topic codec error.
        source: TopicCodecError,
    },
}

impl core::fmt::Display for StaticBusError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TopicIndexOutOfRange {
                topic_name,
                index,
                capacity,
            } => write!(
                f,
                "topic {topic_name} index {index} is outside static bus capacity {capacity}"
            ),
            Self::SlotTooSmall {
                topic_name,
                encoded_len,
                slot_bytes,
            } => write!(
                f,
                "topic {topic_name} encodes to {encoded_len} bytes but static bus slot is {slot_bytes} bytes"
            ),
            Self::Codec { topic_name, source } => {
                write!(f, "topic {topic_name} codec error: {source}")
            }
        }
    }
}

/// Fixed-capacity latest-value topic bus.
///
/// `TOPICS` is the number of entries in the generated flight-topic
/// table. `SLOT_BYTES` is the maximum encoded payload length for any
/// topic in that table. Storage is static, fixed-size, and initialized
/// by value; no heap, `TypeId`, or `Any` is involved.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct StaticBus<const TOPICS: usize, const SLOT_BYTES: usize> {
    slots: [[u8; SLOT_BYTES]; TOPICS],
    occupied: [bool; TOPICS],
    sequences: [u64; TOPICS],
}

impl<const TOPICS: usize, const SLOT_BYTES: usize> StaticBus<TOPICS, SLOT_BYTES> {
    /// Construct an empty static bus.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            slots: [[0; SLOT_BYTES]; TOPICS],
            occupied: [false; TOPICS],
            sequences: [0; TOPICS],
        }
    }

    /// Return the current sequence counter for topic `T`.
    ///
    /// # Errors
    ///
    /// Returns [`StaticBusError`] when `T::INDEX` is outside this
    /// bus's topic table.
    pub fn sequence<T: Topic>(&self) -> HalResult<u64, StaticBusError> {
        let index = checked_topic_index::<T, TOPICS>()?;
        Ok(self.sequences[index])
    }

    fn checked_slot<T: EncodedTopic>() -> HalResult<usize, StaticBusError> {
        let index = checked_topic_index::<T, TOPICS>()?;
        if T::ENCODED_LEN > SLOT_BYTES {
            return Err(StaticBusError::SlotTooSmall {
                topic_name: T::NAME,
                encoded_len: T::ENCODED_LEN,
                slot_bytes: SLOT_BYTES,
            });
        }
        Ok(index)
    }
}

impl<const TOPICS: usize, const SLOT_BYTES: usize> Default for StaticBus<TOPICS, SLOT_BYTES> {
    fn default() -> Self {
        Self::new()
    }
}

fn checked_topic_index<T: Topic, const TOPICS: usize>() -> HalResult<usize, StaticBusError> {
    if T::INDEX >= TOPICS {
        return Err(StaticBusError::TopicIndexOutOfRange {
            topic_name: T::NAME,
            index: T::INDEX,
            capacity: TOPICS,
        });
    }
    Ok(T::INDEX)
}

/// Latest-value typed bus backend.
///
/// Implementations may be dynamic host buses or static flight buses.
/// The `&mut self` interface is deliberate: OpenBMP's current flight
/// shell is single-threaded/cooperative, so the L1 trait does not
/// require locks, atomics, or `Sync`.
pub trait Bus {
    /// Backend error type.
    type Error;

    /// Publish a topic value.
    ///
    /// # Errors
    ///
    /// Returns the backend error when the topic cannot be stored or
    /// dispatched.
    fn publish<T: EncodedTopic>(&mut self, value: T) -> HalResult<(), Self::Error>;

    /// Return the latest topic value, if one has been published.
    ///
    /// # Errors
    ///
    /// Returns the backend error when the topic cannot be read.
    fn latest<T: EncodedTopic>(&self) -> HalResult<Option<T>, Self::Error>;
}

impl<const TOPICS: usize, const SLOT_BYTES: usize> Bus for StaticBus<TOPICS, SLOT_BYTES> {
    type Error = StaticBusError;

    fn publish<T: EncodedTopic>(&mut self, value: T) -> HalResult<(), Self::Error> {
        let index = Self::checked_slot::<T>()?;
        let slot = &mut self.slots[index];
        slot.fill(0);
        value
            .encode(&mut slot[..T::ENCODED_LEN])
            .map_err(|source| StaticBusError::Codec {
                topic_name: T::NAME,
                source,
            })?;
        self.occupied[index] = true;
        self.sequences[index] = self.sequences[index].saturating_add(1);
        Ok(())
    }

    fn latest<T: EncodedTopic>(&self) -> HalResult<Option<T>, Self::Error> {
        let index = Self::checked_slot::<T>()?;
        if !self.occupied[index] {
            return Ok(None);
        }
        T::decode(&self.slots[index][..T::ENCODED_LEN])
            .map(Some)
            .map_err(|source| StaticBusError::Codec {
                topic_name: T::NAME,
                source,
            })
    }
}

/// Common pull-style sensor trait.
pub trait Sensor {
    /// Measurement payload produced by the sensor.
    type Measurement;
    /// Driver or transport error.
    type Error;

    /// Read one measurement from the sensor.
    ///
    /// # Errors
    ///
    /// Returns the driver or transport error when a measurement cannot
    /// be read.
    fn read(&mut self) -> HalResult<Self::Measurement, Self::Error>;
}

/// Inertial measurement unit.
pub trait Imu: Sensor {}

/// Global navigation satellite system receiver.
pub trait Gnss: Sensor {}

/// Magnetometer.
pub trait Magnetometer: Sensor {}

/// Barometric altimeter.
pub trait Barometer: Sensor {}

/// Star-tracker attitude sensor.
pub trait StarTracker: Sensor {}

/// Common command-style actuator trait.
pub trait Actuator {
    /// Command payload consumed by the actuator.
    type Command;
    /// Driver or transport error.
    type Error;

    /// Send one actuator command.
    ///
    /// # Errors
    ///
    /// Returns the driver or transport error when the command cannot
    /// be accepted.
    fn command(&mut self, command: Self::Command) -> HalResult<(), Self::Error>;
}

/// Thrust-vector-control actuator.
pub trait TvcActuator: Actuator {}

/// Reaction-control-system valve or valve bank.
pub trait RcsValve: Actuator {}

/// Throttle command sink.
pub trait ThrottleCommand: Actuator {}

/// Flight-recorder and I-load storage surface.
pub trait Storage {
    /// Storage driver error.
    type Error;

    /// Load the serialized initialization-load blob into `buffer` and
    /// return the populated prefix.
    ///
    /// # Errors
    ///
    /// Returns the storage error when the I-load cannot be read or the
    /// provided buffer is too small for the backend's policy.
    fn load_iload<'a>(&mut self, buffer: &'a mut [u8]) -> HalResult<&'a [u8], Self::Error>;

    /// Append one bounded flight-recorder payload.
    ///
    /// # Errors
    ///
    /// Returns the storage error when the record cannot be persisted.
    fn append_record(&mut self, record: &[u8]) -> HalResult<(), Self::Error>;
}

/// Hardware or simulator watchdog service.
pub trait Watchdog {
    /// Watchdog driver error.
    type Error;

    /// Service the watchdog.
    ///
    /// # Errors
    ///
    /// Returns the watchdog driver error when the service operation
    /// fails.
    fn pet(&mut self) -> HalResult<(), Self::Error>;
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn iload_envelope_round_trips_payload_and_crc() {
        let payload = b"fc-scheduler-and-mission-table";
        let mut storage = [0_u8; 128];

        let len = encode_iload_envelope(7, payload, &mut storage).unwrap();
        let verified = decode_iload_envelope(&storage[..len], &[7]).unwrap();

        assert_eq!(verified.schema_version, 7);
        assert_eq!(verified.payload, payload);
        assert_eq!(verified.payload_crc32, crc32_ieee(payload));
    }

    #[test]
    fn iload_envelope_rejects_crc_mismatch() {
        let payload = b"valid-payload";
        let mut storage = [0_u8; 64];
        let len = encode_iload_envelope(1, payload, &mut storage).unwrap();
        storage[len - 1] ^= 0xff;

        let err = decode_iload_envelope(&storage[..len], &[1]).unwrap_err();

        assert!(matches!(err, IloadEnvelopeError::CrcMismatch { .. }));
    }

    #[test]
    fn iload_envelope_rejects_unsupported_schema_version() {
        let mut storage = [0_u8; 64];
        let len = encode_iload_envelope(3, b"payload", &mut storage).unwrap();

        let err = decode_iload_envelope(&storage[..len], &[1, 2]).unwrap_err();

        assert_eq!(
            err,
            IloadEnvelopeError::UnsupportedSchemaVersion { found: 3 }
        );
    }

    #[test]
    fn iload_encoder_checks_output_capacity() {
        let mut storage = [0_u8; ILOAD_HEADER_LEN + 2];

        let err = encode_iload_envelope(1, b"three", &mut storage).unwrap_err();

        assert_eq!(
            err,
            IloadEnvelopeError::BufferTooSmall {
                required: ILOAD_HEADER_LEN + 5,
                actual: ILOAD_HEADER_LEN + 2,
            }
        );
    }

    #[test]
    fn iload_fallback_selects_fallback_when_primary_is_corrupt() {
        let mut primary = [0_u8; 64];
        let primary_len = encode_iload_envelope(1, b"primary", &mut primary).unwrap();
        primary[ILOAD_HEADER_LEN] ^= 0x55;
        let mut fallback = [0_u8; 64];
        let fallback_len = encode_iload_envelope(1, b"fallback", &mut fallback).unwrap();

        let selected = decode_iload_or_fallback(
            &primary[..primary_len],
            Some(&fallback[..fallback_len]),
            &[1],
        )
        .unwrap();

        assert_eq!(selected.source, IloadSource::Fallback);
        assert_eq!(selected.iload.payload, b"fallback");
    }

    #[test]
    fn flight_record_envelope_round_trips_payload() {
        let mut storage = [0_u8; 96];
        let len = encode_flight_record(
            flight_record_kind::TOPIC_SNAPSHOT,
            17,
            SimTime::from_seconds(12.5),
            b"topic-bytes",
            &mut storage,
        )
        .unwrap();

        let record = decode_flight_record(&storage[..len]).unwrap();

        assert_eq!(record.kind, flight_record_kind::TOPIC_SNAPSHOT);
        assert_eq!(record.sequence, 17);
        assert_eq!(record.time, SimTime::from_seconds(12.5));
        assert_eq!(record.payload, b"topic-bytes");
        assert_eq!(record.payload_crc32, crc32_ieee(b"topic-bytes"));
    }

    #[test]
    fn flight_record_envelope_rejects_crc_mismatch() {
        let mut storage = [0_u8; 96];
        let len = encode_flight_record(
            flight_record_kind::HEALTH_EVENT,
            1,
            SimTime::from_seconds(1.0),
            b"health-event",
            &mut storage,
        )
        .unwrap();
        storage[len - 1] ^= 0x44;

        let err = decode_flight_record(&storage[..len]).unwrap_err();

        assert!(matches!(err, FlightRecordError::CrcMismatch { .. }));
    }

    #[test]
    fn flight_recorder_overwrites_oldest_and_counts_drop() {
        let mut recorder = FlightRecorder::<2, 8>::new(FlightRecordOverflowPolicy::OverwriteOldest);

        assert_eq!(
            recorder
                .push(
                    flight_record_kind::TOPIC_SNAPSHOT,
                    SimTime::from_seconds(1.0),
                    b"one",
                )
                .unwrap(),
            0
        );
        assert_eq!(
            recorder
                .push(
                    flight_record_kind::TOPIC_SNAPSHOT,
                    SimTime::from_seconds(2.0),
                    b"two",
                )
                .unwrap(),
            1
        );
        assert_eq!(
            recorder
                .push(
                    flight_record_kind::TOPIC_SNAPSHOT,
                    SimTime::from_seconds(3.0),
                    b"three",
                )
                .unwrap(),
            2
        );

        assert_eq!(recorder.len(), 2);
        assert_eq!(recorder.dropped_records(), 1);
        assert_eq!(recorder.pop_oldest().unwrap().sequence(), 1);
        let newest = recorder.pop_oldest().unwrap();
        assert_eq!(newest.sequence(), 2);
        assert_eq!(newest.payload(), b"three");
    }

    #[test]
    fn flight_recorder_drop_newest_retains_queued_record() {
        let mut recorder = FlightRecorder::<1, 8>::new(FlightRecordOverflowPolicy::DropNewest);

        recorder
            .push(
                flight_record_kind::HEALTH_EVENT,
                SimTime::from_seconds(1.0),
                b"first",
            )
            .unwrap();
        let err = recorder
            .push(
                flight_record_kind::HEALTH_EVENT,
                SimTime::from_seconds(2.0),
                b"second",
            )
            .unwrap_err();

        assert_eq!(err, FlightRecordError::RecorderFull { capacity: 1 });
        assert_eq!(recorder.dropped_records(), 1);
        let queued = recorder.pop_oldest().unwrap();
        assert_eq!(queued.sequence(), 0);
        assert_eq!(queued.payload(), b"first");
    }

    #[derive(Debug)]
    struct MemoryStorage {
        record: [u8; 96],
        len: usize,
    }

    impl Default for MemoryStorage {
        fn default() -> Self {
            Self {
                record: [0; 96],
                len: 0,
            }
        }
    }

    impl Storage for MemoryStorage {
        type Error = core::convert::Infallible;

        fn load_iload<'a>(&mut self, _buffer: &'a mut [u8]) -> HalResult<&'a [u8], Self::Error> {
            Ok(&[])
        }

        fn append_record(&mut self, record: &[u8]) -> HalResult<(), Self::Error> {
            self.record[..record.len()].copy_from_slice(record);
            self.len = record.len();
            Ok(())
        }
    }

    #[test]
    fn flight_recorder_flush_encodes_and_pops_after_storage_success() {
        let mut recorder = FlightRecorder::<2, 16>::default();
        let mut storage = MemoryStorage::default();
        let mut frame = [0_u8; 96];
        recorder
            .push(
                flight_record_kind::SCHEDULER_EVENT,
                SimTime::from_seconds(4.0),
                b"overrun",
            )
            .unwrap();

        let flushed = recorder.flush_oldest(&mut storage, &mut frame).unwrap();

        assert_eq!(flushed, Some(0));
        assert!(recorder.is_empty());
        let decoded = decode_flight_record(&storage.record[..storage.len]).unwrap();
        assert_eq!(decoded.kind, flight_record_kind::SCHEDULER_EVENT);
        assert_eq!(decoded.payload, b"overrun");
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    struct ExampleTopic(u32);

    impl Topic for ExampleTopic {
        const NAME: &'static str = "example.topic";
        const INDEX: usize = 0;
    }

    impl EncodedTopic for ExampleTopic {
        const ENCODED_LEN: usize = 4;

        fn encode(&self, out: &mut [u8]) -> HalResult<(), TopicCodecError> {
            if out.len() < Self::ENCODED_LEN {
                return Err(TopicCodecError::BufferTooSmall {
                    required: Self::ENCODED_LEN,
                    actual: out.len(),
                });
            }
            out[..Self::ENCODED_LEN].copy_from_slice(&self.0.to_le_bytes());
            Ok(())
        }

        fn decode(input: &[u8]) -> HalResult<Self, TopicCodecError> {
            if input.len() < Self::ENCODED_LEN {
                return Err(TopicCodecError::BufferTooSmall {
                    required: Self::ENCODED_LEN,
                    actual: input.len(),
                });
            }
            Ok(Self(u32::from_le_bytes([
                input[0], input[1], input[2], input[3],
            ])))
        }
    }

    #[derive(Default)]
    struct ExampleBus {
        latest: Option<ExampleTopic>,
    }

    impl Bus for ExampleBus {
        type Error = core::convert::Infallible;

        fn publish<T: EncodedTopic>(&mut self, value: T) -> HalResult<(), Self::Error> {
            if T::NAME == ExampleTopic::NAME {
                // The trait-level smoke test only exercises the
                // shape; real backends provide typed storage.
                let _ = value;
                self.latest = Some(ExampleTopic(1));
            }
            Ok(())
        }

        fn latest<T: EncodedTopic>(&self) -> HalResult<Option<T>, Self::Error> {
            let _ = self.latest;
            Ok(None)
        }
    }

    #[test]
    fn bus_trait_accepts_topic_payloads() {
        let mut bus = ExampleBus::default();
        bus.publish(ExampleTopic(7)).unwrap();
        assert_eq!(bus.latest, Some(ExampleTopic(1)));
    }

    #[test]
    fn static_bus_round_trips_encoded_topics_without_heap() {
        let mut bus = StaticBus::<1, 4>::new();
        assert_eq!(bus.latest::<ExampleTopic>().unwrap(), None);
        bus.publish(ExampleTopic(42)).unwrap();
        assert_eq!(bus.sequence::<ExampleTopic>().unwrap(), 1);
        assert_eq!(
            bus.latest::<ExampleTopic>().unwrap(),
            Some(ExampleTopic(42))
        );
    }

    #[test]
    fn static_bus_rejects_out_of_range_topic_index() {
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        struct MissingTopic;

        impl Topic for MissingTopic {
            const NAME: &'static str = "missing.topic";
            const INDEX: usize = 7;
        }

        impl EncodedTopic for MissingTopic {
            const ENCODED_LEN: usize = 1;

            fn encode(&self, out: &mut [u8]) -> HalResult<(), TopicCodecError> {
                if out.is_empty() {
                    return Err(TopicCodecError::BufferTooSmall {
                        required: 1,
                        actual: out.len(),
                    });
                }
                out[0] = 0;
                Ok(())
            }

            fn decode(input: &[u8]) -> HalResult<Self, TopicCodecError> {
                if input.is_empty() {
                    return Err(TopicCodecError::BufferTooSmall {
                        required: 1,
                        actual: input.len(),
                    });
                }
                Ok(Self)
            }
        }

        let mut bus = StaticBus::<1, 4>::new();
        let err = bus.publish(MissingTopic).unwrap_err();
        assert_eq!(
            err,
            StaticBusError::TopicIndexOutOfRange {
                topic_name: "missing.topic",
                index: 7,
                capacity: 1,
            }
        );
    }
}
