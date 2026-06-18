//! Blocking message-boundary-preserving transports for the HIL bridge.
//!
//! These transports move whole [`BridgeMessage`] values. They are still
//! generic host-side utilities: no hardware bus, device driver, flight
//! stack adapter, retry policy, or qualification claim lives here.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, ToSocketAddrs};
#[cfg(unix)]
use std::os::unix::net::{UnixListener, UnixStream};
#[cfg(unix)]
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, Sender};

use crate::codec::{decode, encode, frame};
use crate::error::BridgeError;
use crate::packet::BridgeMessage;

const DEFAULT_MAX_PAYLOAD_LEN: usize = 1024 * 1024;
const LENGTH_PREFIX_LEN: usize = 4;

/// A bidirectional, message-boundary-preserving lockstep channel.
///
/// Implementations own framing. Callers exchange decoded
/// [`BridgeMessage`] values and keep lockstep validation separate via
/// the helpers in [`crate::lockstep`].
pub trait Transport {
    /// Block until exactly one bridge message is available.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError`] when the transport closes, a framed payload
    /// exceeds the configured size limit, I/O fails, or decoding fails.
    fn recv(&mut self) -> Result<BridgeMessage, BridgeError>;

    /// Frame and send exactly one bridge message.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError`] when the payload exceeds the configured
    /// size limit, the transport is closed, encoding fails, or I/O fails.
    fn send(&mut self, message: &BridgeMessage) -> Result<(), BridgeError>;
}

/// In-process lockstep channel backed by two `std::sync::mpsc` queues.
///
/// This is useful for deterministic host-side tests and for routing the
/// simulator and controller through the same message contract without
/// introducing a socket.
#[derive(Debug)]
pub struct InProcessTransport {
    tx: Sender<BridgeMessage>,
    rx: Receiver<BridgeMessage>,
}

impl InProcessTransport {
    fn new(tx: Sender<BridgeMessage>, rx: Receiver<BridgeMessage>) -> Self {
        Self { tx, rx }
    }
}

impl Transport for InProcessTransport {
    fn recv(&mut self) -> Result<BridgeMessage, BridgeError> {
        self.rx.recv().map_err(|_| BridgeError::TransportClosed)
    }

    fn send(&mut self, message: &BridgeMessage) -> Result<(), BridgeError> {
        self.tx
            .send(message.clone())
            .map_err(|_| BridgeError::TransportClosed)
    }
}

/// Construct a connected in-process transport pair.
///
/// Messages sent on the first endpoint are received by the second
/// endpoint, and messages sent on the second endpoint are received by
/// the first endpoint.
#[must_use]
pub fn in_process_transport_pair() -> (InProcessTransport, InProcessTransport) {
    let (a_tx, b_rx) = mpsc::channel();
    let (b_tx, a_rx) = mpsc::channel();
    (
        InProcessTransport::new(a_tx, a_rx),
        InProcessTransport::new(b_tx, b_rx),
    )
}

/// Length-prefixed `postcard` transport over a blocking byte stream.
///
/// `S` may be a TCP stream, Unix-domain stream, pipe, cursor, or any
/// other caller-owned stream implementing [`Read`] and [`Write`].
#[derive(Debug)]
pub struct StreamTransport<S> {
    stream: S,
    read_buf: Vec<u8>,
    max_payload_len: usize,
}

/// Length-prefixed `postcard` transport over separate blocking reader
/// and writer handles.
///
/// This covers child-process stdio, pipe pairs, and other host
/// transports where inbound and outbound bytes are not exposed through
/// one duplex object.
#[derive(Debug)]
pub struct SplitStreamTransport<R, W> {
    reader: R,
    writer: W,
    read_buf: Vec<u8>,
    max_payload_len: usize,
}

/// TCP listener for accepting one or more bridge stream transports.
///
/// This is a host-loopback convenience around [`TcpListener`]. It does
/// not define a bus protocol or retry/lifecycle policy for a hardware
/// bench.
#[derive(Debug)]
pub struct TcpBridgeListener {
    listener: TcpListener,
}

/// Unix-domain-socket listener for bridge stream transports.
///
/// Available on Unix platforms only. Like [`TcpBridgeListener`], this is
/// a host convenience and not a hardware bus adapter.
#[cfg(unix)]
#[derive(Debug)]
pub struct UnixBridgeListener {
    listener: UnixListener,
    path: PathBuf,
}

impl TcpBridgeListener {
    /// Bind a TCP listener for bridge stream transports.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError::Io`] if the socket cannot be bound.
    pub fn bind<A: ToSocketAddrs>(addr: A) -> Result<Self, BridgeError> {
        Ok(Self {
            listener: TcpListener::bind(addr)?,
        })
    }

    /// Return the local socket address assigned to the listener.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError::Io`] if the local address cannot be read.
    pub fn local_addr(&self) -> Result<SocketAddr, BridgeError> {
        self.listener.local_addr().map_err(BridgeError::Io)
    }

    /// Block until one peer connects and wrap it in [`StreamTransport`].
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError::Io`] if `accept` fails.
    pub fn accept(&self) -> Result<(StreamTransport<TcpStream>, SocketAddr), BridgeError> {
        let (stream, peer) = self.listener.accept()?;
        Ok((StreamTransport::new(stream), peer))
    }
}

impl StreamTransport<TcpStream> {
    /// Connect a TCP stream transport to a peer.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError::Io`] if the connection fails.
    pub fn connect_tcp<A: ToSocketAddrs>(addr: A) -> Result<Self, BridgeError> {
        Ok(Self::new(TcpStream::connect(addr)?))
    }
}

#[cfg(unix)]
impl UnixBridgeListener {
    /// Bind a Unix-domain-socket listener for bridge stream transports.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError::Io`] if the socket cannot be bound.
    pub fn bind<P: AsRef<Path>>(path: P) -> Result<Self, BridgeError> {
        let path = path.as_ref().to_path_buf();
        Ok(Self {
            listener: UnixListener::bind(&path)?,
            path,
        })
    }

    /// Return the filesystem path this listener was bound to.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Block until one peer connects and wrap it in [`StreamTransport`].
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError::Io`] if `accept` fails.
    pub fn accept(&self) -> Result<StreamTransport<UnixStream>, BridgeError> {
        let (stream, _) = self.listener.accept()?;
        Ok(StreamTransport::new(stream))
    }
}

#[cfg(unix)]
impl StreamTransport<UnixStream> {
    /// Connect a Unix-domain-socket stream transport to a peer.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError::Io`] if the connection fails.
    pub fn connect_unix<P: AsRef<Path>>(path: P) -> Result<Self, BridgeError> {
        Ok(Self::new(UnixStream::connect(path)?))
    }
}

impl<S> StreamTransport<S> {
    /// Construct a stream transport with a 1 MiB maximum payload.
    #[must_use]
    pub fn new(stream: S) -> Self {
        Self {
            stream,
            read_buf: Vec::new(),
            max_payload_len: DEFAULT_MAX_PAYLOAD_LEN,
        }
    }

    /// Construct a stream transport with an explicit maximum payload.
    #[must_use]
    pub fn with_max_payload_len(stream: S, max_payload_len: usize) -> Self {
        Self {
            stream,
            read_buf: Vec::new(),
            max_payload_len,
        }
    }

    /// Return the configured maximum decoded payload length in bytes.
    #[must_use]
    pub const fn max_payload_len(&self) -> usize {
        self.max_payload_len
    }

    /// Consume this transport and return the wrapped stream.
    #[must_use]
    pub fn into_inner(self) -> S {
        self.stream
    }
}

impl<S: Read + Write> Transport for StreamTransport<S> {
    fn recv(&mut self) -> Result<BridgeMessage, BridgeError> {
        recv_framed(&mut self.stream, &mut self.read_buf, self.max_payload_len)
    }

    fn send(&mut self, message: &BridgeMessage) -> Result<(), BridgeError> {
        send_framed(&mut self.stream, self.max_payload_len, message)
    }
}

impl<R, W> SplitStreamTransport<R, W> {
    /// Construct a split stream transport with a 1 MiB maximum payload.
    #[must_use]
    pub fn new(reader: R, writer: W) -> Self {
        Self {
            reader,
            writer,
            read_buf: Vec::new(),
            max_payload_len: DEFAULT_MAX_PAYLOAD_LEN,
        }
    }

    /// Construct a split stream transport with an explicit maximum payload.
    #[must_use]
    pub fn with_max_payload_len(reader: R, writer: W, max_payload_len: usize) -> Self {
        Self {
            reader,
            writer,
            read_buf: Vec::new(),
            max_payload_len,
        }
    }

    /// Return the configured maximum decoded payload length in bytes.
    #[must_use]
    pub const fn max_payload_len(&self) -> usize {
        self.max_payload_len
    }

    /// Consume this transport and return the wrapped reader and writer.
    #[must_use]
    pub fn into_inner(self) -> (R, W) {
        (self.reader, self.writer)
    }
}

impl<R: Read, W: Write> Transport for SplitStreamTransport<R, W> {
    fn recv(&mut self) -> Result<BridgeMessage, BridgeError> {
        recv_framed(&mut self.reader, &mut self.read_buf, self.max_payload_len)
    }

    fn send(&mut self, message: &BridgeMessage) -> Result<(), BridgeError> {
        send_framed(&mut self.writer, self.max_payload_len, message)
    }
}

fn buffered_frame_len(
    read_buf: &[u8],
    max_payload_len: usize,
) -> Result<Option<usize>, BridgeError> {
    if read_buf.len() < LENGTH_PREFIX_LEN {
        return Ok(None);
    }

    let payload_len =
        u32::from_le_bytes([read_buf[0], read_buf[1], read_buf[2], read_buf[3]]) as usize;
    if payload_len > max_payload_len {
        return Err(BridgeError::PayloadTooLarge {
            max: max_payload_len,
            got: payload_len,
        });
    }

    let frame_len = LENGTH_PREFIX_LEN + payload_len;
    if read_buf.len() < frame_len {
        return Ok(None);
    }

    Ok(Some(frame_len))
}

fn discard_read_prefix(read_buf: &mut Vec<u8>, consumed: usize) {
    let remaining = read_buf.len().saturating_sub(consumed);
    read_buf.copy_within(consumed.., 0);
    read_buf.truncate(remaining);
}

fn recv_framed<R: Read>(
    reader: &mut R,
    read_buf: &mut Vec<u8>,
    max_payload_len: usize,
) -> Result<BridgeMessage, BridgeError> {
    let mut scratch = [0u8; 4096];

    loop {
        if let Some(frame_len) = buffered_frame_len(read_buf, max_payload_len)? {
            let message = decode(&read_buf[LENGTH_PREFIX_LEN..frame_len]);
            discard_read_prefix(read_buf, frame_len);
            return message;
        }

        let read = reader.read(&mut scratch)?;
        if read == 0 {
            return Err(BridgeError::TransportClosed);
        }
        read_buf.extend_from_slice(&scratch[..read]);
    }
}

fn send_framed<W: Write>(
    writer: &mut W,
    max_payload_len: usize,
    message: &BridgeMessage,
) -> Result<(), BridgeError> {
    let payload = encode(message)?;
    if payload.len() > max_payload_len {
        return Err(BridgeError::PayloadTooLarge {
            max: max_payload_len,
            got: payload.len(),
        });
    }

    writer.write_all(&frame(&payload))?;
    writer.flush()?;
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use std::io::Cursor;
    #[cfg(unix)]
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::thread;

    use super::*;
    use crate::packet::{
        ActuatorCommandPacket, BridgeEndpointRole, BridgeHelloPacket, SensorPacket, StepAckPacket,
        StepAckStatus,
    };

    fn sensor(step: u64) -> SensorPacket {
        SensorPacket {
            sim_time_s: step as f64 * 0.01,
            step,
            imu_accel_body_m_s2: [0.0, 0.0, 9.81],
            imu_gyro_body_rad_s: [0.0; 3],
            baro_altitude_m: Some(100.0 + step as f64),
            gnss_position_eci_m: None,
            gnss_velocity_eci_m_s: None,
            gnss_position_bias_eci_m: None,
            mag_body_tesla: None,
            mag_body_nt: None,
            mag_hard_iron_body_nt: None,
            baro_pressure_pa: None,
            baro_bias_pa: None,
            star_tracker_attitude_eci_to_body_xyzw: None,
            ..SensorPacket::default()
        }
    }

    fn command_for(sensor: &SensorPacket) -> BridgeMessage {
        BridgeMessage::Command(ActuatorCommandPacket {
            sim_time_s: sensor.sim_time_s,
            step: sensor.step,
            effector_commands: vec![(0, 0.25)],
            engine_throttles: vec![(0, 0.8)],
            engine_commands: vec![],
        })
    }

    #[test]
    fn in_process_transport_pair_exchanges_bridge_messages() {
        let (mut simulator, mut controller) = in_process_transport_pair();
        let hello =
            BridgeMessage::Hello(BridgeHelloPacket::new(BridgeEndpointRole::Simulator, 4096));
        simulator.send(&hello).unwrap();
        assert_eq!(controller.recv().unwrap(), hello);

        let sensor = sensor(42);
        let command = command_for(&sensor);
        controller.send(&command).unwrap();
        assert_eq!(simulator.recv().unwrap(), command);
    }

    #[test]
    fn in_process_transport_reports_closed_peer() {
        let (mut simulator, controller) = in_process_transport_pair();
        drop(controller);

        let message = BridgeMessage::Sensor(sensor(1));
        assert!(matches!(
            simulator.send(&message),
            Err(BridgeError::TransportClosed)
        ));
    }

    #[test]
    fn stream_transport_writes_and_reads_framed_messages() {
        let mut writer = StreamTransport::new(Cursor::new(Vec::new()));
        let first = BridgeMessage::Sensor(sensor(7));
        let second = command_for(&sensor(7));

        writer.send(&first).unwrap();
        writer.send(&second).unwrap();

        let bytes = writer.into_inner().into_inner();
        let mut reader = StreamTransport::new(Cursor::new(bytes));
        assert_eq!(reader.recv().unwrap(), first);
        assert_eq!(reader.recv().unwrap(), second);
    }

    #[test]
    fn split_stream_transport_writes_and_reads_framed_messages() {
        let message = BridgeMessage::Sensor(sensor(11));
        let mut writer =
            SplitStreamTransport::new(Cursor::new(Vec::new()), Cursor::new(Vec::new()));

        writer.send(&message).unwrap();
        let (_, bytes) = writer.into_inner();

        let mut reader =
            SplitStreamTransport::new(Cursor::new(bytes.into_inner()), Cursor::new(Vec::new()));
        assert_eq!(reader.recv().unwrap(), message);
    }

    #[test]
    fn stream_transport_rejects_payloads_over_limit_on_send() {
        let mut transport = StreamTransport::with_max_payload_len(Cursor::new(Vec::new()), 1);
        let message = BridgeMessage::Sensor(sensor(2));

        assert!(matches!(
            transport.send(&message),
            Err(BridgeError::PayloadTooLarge { max: 1, got }) if got > 1
        ));
    }

    #[test]
    fn stream_transport_rejects_payloads_over_limit_on_recv() {
        let message = BridgeMessage::Ack(StepAckPacket {
            sim_time_s: 0.03,
            step: 3,
            status: StepAckStatus::Accepted,
        });
        let bytes = frame(&encode(&message).unwrap());
        let mut transport = StreamTransport::with_max_payload_len(Cursor::new(bytes), 1);

        assert!(matches!(
            transport.recv(),
            Err(BridgeError::PayloadTooLarge { max: 1, got }) if got > 1
        ));
    }

    #[test]
    fn stream_transport_reports_closed_before_full_message() {
        let partial_prefix = 8u32.to_le_bytes().to_vec();
        let mut transport = StreamTransport::new(Cursor::new(partial_prefix));

        assert!(matches!(
            transport.recv(),
            Err(BridgeError::TransportClosed)
        ));
    }

    #[test]
    fn tcp_bridge_listener_accepts_stream_transport() {
        let listener = TcpBridgeListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let server = thread::spawn(move || {
            let (mut transport, peer) = listener.accept().unwrap();
            assert!(peer.ip().is_loopback());
            let message = transport.recv().unwrap();
            transport.send(&message).unwrap();
        });

        let mut client = StreamTransport::connect_tcp(addr).unwrap();
        let message = BridgeMessage::Sensor(sensor(9));
        client.send(&message).unwrap();
        assert_eq!(client.recv().unwrap(), message);
        server.join().unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn unix_bridge_listener_accepts_stream_transport() {
        static NEXT_SOCKET_ID: AtomicUsize = AtomicUsize::new(0);
        let socket_id = NEXT_SOCKET_ID.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "openbmp_bridge_{}_{}.sock",
            std::process::id(),
            socket_id
        ));
        let _ = std::fs::remove_file(&path);

        let listener = UnixBridgeListener::bind(&path).unwrap();
        assert_eq!(listener.path(), path.as_path());
        let server = thread::spawn(move || {
            let mut transport = listener.accept().unwrap();
            let message = transport.recv().unwrap();
            transport.send(&message).unwrap();
        });

        let mut client = StreamTransport::connect_unix(&path).unwrap();
        let message = BridgeMessage::Sensor(sensor(10));
        client.send(&message).unwrap();
        assert_eq!(client.recv().unwrap(), message);
        server.join().unwrap();
        std::fs::remove_file(&path).unwrap();
    }
}
