//! Minimal external flight-controller stdio peer for runner transport tests.
//!
//! This is a protocol/lifecycle fixture, not a flight controller. It
//! validates the simulator hello, answers as a flight-controller peer,
//! then returns an empty command packet for each sensor frame.

use std::process::ExitCode;

use openbmp_bridge::{
    ActuatorCommandPacket, BridgeEndpointRole, BridgeError, BridgeFaultCode, BridgeFaultPacket,
    BridgeHelloPacket, BridgeMessage, SplitStreamTransport, Transport, validate_protocol_version,
};

const MAX_PAYLOAD_LEN: u32 = 4096;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("openbmp-fc-stdio-echo failed: {err}");
            ExitCode::from(1)
        }
    }
}

fn run() -> Result<(), BridgeError> {
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    let mut transport = SplitStreamTransport::with_max_payload_len(
        stdin.lock(),
        stdout.lock(),
        MAX_PAYLOAD_LEN as usize,
    );

    let hello = transport.recv()?;
    let BridgeMessage::Hello(sim_hello) = hello else {
        transport.send(&BridgeMessage::Fault(BridgeFaultPacket {
            step: None,
            code: BridgeFaultCode::ApplicationFault,
        }))?;
        return Err(BridgeError::UnexpectedMessage {
            expected: "Hello",
            got: message_kind(&hello),
        });
    };
    validate_protocol_version(sim_hello.protocol_version)?;
    if sim_hello.role != BridgeEndpointRole::Simulator {
        return Err(BridgeError::RoleMismatch {
            expected: BridgeEndpointRole::Simulator,
            got: sim_hello.role,
        });
    }
    transport.send(&BridgeMessage::Hello(BridgeHelloPacket::new(
        BridgeEndpointRole::FlightController,
        MAX_PAYLOAD_LEN,
    )))?;

    loop {
        let message = match transport.recv() {
            Ok(message) => message,
            Err(BridgeError::TransportClosed) => return Ok(()),
            Err(err) => return Err(err),
        };
        match message {
            BridgeMessage::Sensor(sensor) => {
                transport.send(&BridgeMessage::Command(ActuatorCommandPacket {
                    sim_time_s: sensor.sim_time_s,
                    step: sensor.step,
                    effector_commands: Vec::new(),
                    engine_throttles: Vec::new(),
                    engine_commands: Vec::new(),
                }))?;
            }
            BridgeMessage::Fault(_) => return Ok(()),
            message => {
                transport.send(&BridgeMessage::Fault(BridgeFaultPacket {
                    step: None,
                    code: BridgeFaultCode::ApplicationFault,
                }))?;
                return Err(BridgeError::UnexpectedMessage {
                    expected: "Sensor | Fault",
                    got: message_kind(&message),
                });
            }
        }
    }
}

fn message_kind(message: &BridgeMessage) -> &'static str {
    match message {
        BridgeMessage::Hello(_) => "Hello",
        BridgeMessage::Sensor(_) => "Sensor",
        BridgeMessage::Command(_) => "Command",
        BridgeMessage::Ack(_) => "Ack",
        BridgeMessage::Fault(_) => "Fault",
    }
}
