//! Host-side Processor-in-the-Loop coupling helpers.
//!
//! The runner does not execute Renode in-process. This module renders the
//! deterministic `.resc` monitor script needed to couple a future target ELF
//! over a socket-backed UART while preserving the existing bridge frame bytes.

use std::fs;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::path::Path;

use openbmp_bridge::{
    ActuatorCommandPacket, BridgeError, PROTOCOL_VERSION, SensorPacket, decode, encode, frame,
    validate_command_for_sensor,
};
use openbmp_pil::{PilMailboxError, PilMailboxLayout, PilMailboxStatus};

const DEFAULT_PIL_MAX_PAYLOAD_LEN: usize = 4096;
const FRAME_PREFIX_LEN: usize = 4;

/// Renode plan for a socket-backed UART PIL coupling.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RenodePilPlan {
    /// Renode machine name.
    pub machine_name: String,
    /// Renode platform description path, without the leading `@`.
    pub platform_repl: String,
    /// Target firmware ELF path, without the leading `@`.
    pub firmware_elf: String,
    /// UART peripheral name in the loaded platform, for example
    /// `sysbus.usart2`.
    pub uart_peripheral: String,
    /// Renode terminal object name used for the host socket.
    pub uart_socket_name: String,
    /// Host TCP port exposed by Renode for the UART terminal.
    pub uart_host_port: u16,
    /// Optional GDB server port.
    pub gdb_port: Option<u16>,
    /// Global quantum in microseconds.
    pub quantum_us: u64,
}

impl RenodePilPlan {
    /// Build the default OpenBMP PIL plan for the Renode STM32F4 Discovery
    /// board model.
    #[must_use]
    pub fn stm32f4_discovery_socket(firmware_elf: impl Into<String>) -> Self {
        Self {
            machine_name: "openbmp-pil".to_owned(),
            platform_repl: "platforms/boards/stm32f4_discovery-kit.repl".to_owned(),
            firmware_elf: firmware_elf.into(),
            uart_peripheral: "sysbus.usart2".to_owned(),
            uart_socket_name: "openbmp_pil_uart".to_owned(),
            uart_host_port: 3456,
            gdb_port: Some(3333),
            quantum_us: 1_000,
        }
    }

    /// Validate the plan before rendering.
    ///
    /// # Errors
    ///
    /// Returns [`RenodePilPlanError`] when a required name/path is empty or
    /// contains characters this renderer does not quote, a port is zero, or
    /// the quantum is zero.
    pub fn validate(&self) -> Result<(), RenodePilPlanError> {
        validate_machine_name("machine_name", &self.machine_name)?;
        validate_renode_path("platform_repl", &self.platform_repl)?;
        validate_renode_path("firmware_elf", &self.firmware_elf)?;
        validate_peripheral_name("uart_peripheral", &self.uart_peripheral)?;
        validate_terminal_name("uart_socket_name", &self.uart_socket_name)?;
        validate_port("uart_host_port", self.uart_host_port)?;
        if let Some(gdb_port) = self.gdb_port {
            validate_port("gdb_port", gdb_port)?;
        }
        if self.quantum_us == 0 {
            return Err(RenodePilPlanError::ZeroQuantum);
        }
        Ok(())
    }

    /// Render a deterministic Renode `.resc` monitor script.
    ///
    /// The socket terminal is created with telnet configuration bytes disabled
    /// so the host side sees only the existing OpenBMP bridge frame stream.
    ///
    /// # Errors
    ///
    /// Returns [`RenodePilPlanError`] if the plan is invalid.
    pub fn render_resc(&self) -> Result<String, RenodePilPlanError> {
        self.validate()?;

        let quantum_s = format_quantum_seconds(self.quantum_us);
        let mut script = String::new();
        script.push_str(":name: OpenBMP PIL UART coupling\n");
        script.push_str(
            ":description: Loads an OpenBMP target ELF and exposes the bridge UART as a host TCP socket\n",
        );
        script.push('\n');
        script.push_str(&format!("mach create \"{}\"\n", self.machine_name));
        script.push_str(&format!(
            "machine LoadPlatformDescription @{}\n",
            self.platform_repl
        ));
        script.push_str(&format!("sysbus LoadELF @{}\n", self.firmware_elf));
        script.push_str(&format!(
            "emulation CreateServerSocketTerminal {} \"{}\" false",
            self.uart_host_port, self.uart_socket_name
        ));
        script.push('\n');
        script.push_str(&format!(
            "connector Connect {} {}",
            self.uart_peripheral, self.uart_socket_name
        ));
        script.push('\n');
        script.push_str(&format!("emulation SetGlobalQuantum \"{quantum_s}\"\n"));
        if let Some(gdb_port) = self.gdb_port {
            script.push_str(&format!("machine StartGdbServer {gdb_port}\n"));
        }
        script.push_str(&format!(
            "echo \"OpenBMP PIL UART socket: 127.0.0.1:{}\"",
            self.uart_host_port
        ));
        script.push('\n');
        script.push_str("echo \"Load complete; start when the host bridge is attached.\"\n");
        Ok(script)
    }
}

/// GDB/RSP binding plan for a target-visible [`openbmp_pil::PilMailbox`].
///
/// This plan assumes a target ELF or debugger session has already resolved
/// `symbol_name` to `mailbox_address`. It then maps the `repr(C)` mailbox
/// fields into concrete target memory addresses and renders GDB Remote Serial
/// Protocol memory read/write packets. It does not start Renode, parse ELF
/// symbols, or drive breakpoints.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GdbPilMailboxPlan {
    /// Target symbol that names the mailbox instance.
    pub symbol_name: String,
    /// Resolved target address of `symbol_name`.
    pub mailbox_address: u64,
    /// Concrete mailbox layout exported by `openbmp-pil`.
    pub layout: PilMailboxLayout,
    /// Target function or breakpoint symbol that runs one mailbox step.
    pub step_entry_symbol: String,
    /// Resolved target address of [`GdbPilMailboxPlan::step_entry_symbol`],
    /// when this plan was built from an ELF symbol binding.
    pub step_entry_address: Option<u64>,
}

impl GdbPilMailboxPlan {
    /// Build a GDB mailbox plan from a resolved target symbol address.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError`] when a symbol name is empty,
    /// contains unsupported characters, or the mailbox address is zero.
    pub fn new(
        symbol_name: impl Into<String>,
        mailbox_address: u64,
        layout: PilMailboxLayout,
        step_entry_symbol: impl Into<String>,
    ) -> Result<Self, GdbPilMailboxPlanError> {
        let symbol_name = symbol_name.into();
        let step_entry_symbol = step_entry_symbol.into();
        validate_gdb_symbol_name("symbol_name", &symbol_name)?;
        validate_gdb_symbol_name("step_entry_symbol", &step_entry_symbol)?;
        if mailbox_address == 0 {
            return Err(GdbPilMailboxPlanError::ZeroMailboxAddress);
        }
        Ok(Self {
            symbol_name,
            mailbox_address,
            layout,
            step_entry_symbol,
            step_entry_address: None,
        })
    }

    /// Protocol version the host expects to read from the target mailbox.
    #[must_use]
    pub const fn expected_protocol_version(&self) -> u16 {
        PROTOCOL_VERSION
    }

    /// Attach a resolved target step-entry address to this plan.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::ZeroStepEntryAddress`] when `address`
    /// is zero.
    pub fn with_step_entry_address(mut self, address: u64) -> Result<Self, GdbPilMailboxPlanError> {
        if address == 0 {
            return Err(GdbPilMailboxPlanError::ZeroStepEntryAddress {
                symbol: self.step_entry_symbol.clone(),
            });
        }
        self.step_entry_address = Some(address);
        Ok(self)
    }

    /// Build a GDB mailbox plan for a concrete `PilMailbox` capacity pair.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError`] when symbol validation fails.
    pub fn for_mailbox<const SENSOR_CAPACITY: usize, const COMMAND_CAPACITY: usize>(
        symbol_name: impl Into<String>,
        mailbox_address: u64,
        step_entry_symbol: impl Into<String>,
    ) -> Result<Self, GdbPilMailboxPlanError> {
        Self::new(
            symbol_name,
            mailbox_address,
            openbmp_pil::PilMailbox::<SENSOR_CAPACITY, COMMAND_CAPACITY>::layout(),
            step_entry_symbol,
        )
    }

    /// Build a GDB mailbox plan from symbols resolved out of a target ELF
    /// image.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError`] when the ELF cannot be parsed, either
    /// required symbol is missing or invalid, or plan validation fails.
    pub fn from_elf_symbols(
        elf_bytes: &[u8],
        mailbox_symbol: impl Into<String>,
        layout: PilMailboxLayout,
        step_entry_symbol: impl Into<String>,
    ) -> Result<Self, GdbPilMailboxPlanError> {
        let binding = GdbPilSymbolBinding::resolve_from_elf_bytes(
            elf_bytes,
            mailbox_symbol,
            step_entry_symbol,
        )?;
        binding.mailbox_plan(layout)
    }

    /// Build a GDB mailbox plan from symbols resolved out of a target ELF file.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError`] when the file cannot be read, the ELF
    /// cannot be parsed, either required symbol is missing or invalid, or plan
    /// validation fails.
    pub fn from_elf_file(
        path: impl AsRef<Path>,
        mailbox_symbol: impl Into<String>,
        layout: PilMailboxLayout,
        step_entry_symbol: impl Into<String>,
    ) -> Result<Self, GdbPilMailboxPlanError> {
        let binding =
            GdbPilSymbolBinding::resolve_from_elf_file(path, mailbox_symbol, step_entry_symbol)?;
        binding.mailbox_plan(layout)
    }

    /// Address of the mailbox protocol-version field.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn protocol_version_address(&self) -> Result<u64, GdbPilMailboxPlanError> {
        self.field_address("protocol_version", self.layout.protocol_version_offset)
    }

    /// Address of the mailbox status byte.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn status_address(&self) -> Result<u64, GdbPilMailboxPlanError> {
        self.field_address("status", self.layout.status_offset)
    }

    /// Address of the mailbox compact error-code byte.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn error_address(&self) -> Result<u64, GdbPilMailboxPlanError> {
        self.field_address("error", self.layout.error_offset)
    }

    /// Address of the mailbox sensor-length field.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn sensor_len_address(&self) -> Result<u64, GdbPilMailboxPlanError> {
        self.field_address("sensor_len", self.layout.sensor_len_offset)
    }

    /// Address of the mailbox command-length field.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn command_len_address(&self) -> Result<u64, GdbPilMailboxPlanError> {
        self.field_address("command_len", self.layout.command_len_offset)
    }

    /// Address of the mailbox sensor-payload storage.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn sensor_bytes_address(&self) -> Result<u64, GdbPilMailboxPlanError> {
        self.field_address("sensor_bytes", self.layout.sensor_bytes_offset)
    }

    /// Address of the mailbox command-payload storage.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn command_bytes_address(&self) -> Result<u64, GdbPilMailboxPlanError> {
        self.field_address("command_bytes", self.layout.command_bytes_offset)
    }

    /// Build the deterministic GDB memory writes needed to load a sensor
    /// packet into the target mailbox.
    ///
    /// The status byte is written last so a polling target sees a ready
    /// payload only after the bytes and length are in place.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError`] when encoding fails, the encoded
    /// sensor payload exceeds the target mailbox capacity, or an address
    /// overflows.
    pub fn sensor_packet_writes(
        &self,
        sensor: &SensorPacket,
    ) -> Result<Vec<GdbMemoryWrite>, GdbPilMailboxPlanError> {
        let payload = encode(sensor)?;
        self.sensor_payload_writes(&payload)
    }

    /// Build the deterministic GDB memory writes needed to load an already
    /// encoded sensor payload into the target mailbox.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError`] when the payload exceeds the target
    /// mailbox capacity or an address overflows.
    pub fn sensor_payload_writes(
        &self,
        payload: &[u8],
    ) -> Result<Vec<GdbMemoryWrite>, GdbPilMailboxPlanError> {
        if payload.len() > self.layout.sensor_capacity {
            return Err(GdbPilMailboxPlanError::SensorPayloadTooLarge {
                capacity: self.layout.sensor_capacity,
                got: payload.len(),
            });
        }
        let len = u32::try_from(payload.len()).map_err(|_| {
            GdbPilMailboxPlanError::SensorPayloadTooLarge {
                capacity: self.layout.sensor_capacity,
                got: payload.len(),
            }
        })?;

        Ok(vec![
            GdbMemoryWrite::new(self.error_address()?, vec![PilMailboxError::None as u8]),
            GdbMemoryWrite::new(self.command_len_address()?, 0_u32.to_le_bytes().to_vec()),
            GdbMemoryWrite::new(self.sensor_bytes_address()?, payload.to_vec()),
            GdbMemoryWrite::new(self.sensor_len_address()?, len.to_le_bytes().to_vec()),
            GdbMemoryWrite::new(
                self.status_address()?,
                vec![PilMailboxStatus::SensorReady as u8],
            ),
        ])
    }

    /// Build a GDB memory read for the mailbox protocol-version field.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn protocol_version_read(&self) -> Result<GdbMemoryRead, GdbPilMailboxPlanError> {
        Ok(GdbMemoryRead::new(self.protocol_version_address()?, 2))
    }

    /// Build a GDB memory read for the mailbox status byte.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn status_read(&self) -> Result<GdbMemoryRead, GdbPilMailboxPlanError> {
        Ok(GdbMemoryRead::new(self.status_address()?, 1))
    }

    /// Build a GDB memory read for the mailbox error byte.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn error_read(&self) -> Result<GdbMemoryRead, GdbPilMailboxPlanError> {
        Ok(GdbMemoryRead::new(self.error_address()?, 1))
    }

    /// Build a GDB memory read for the mailbox command-length field.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::AddressOverflow`] if the field
    /// address cannot be represented.
    pub fn command_len_read(&self) -> Result<GdbMemoryRead, GdbPilMailboxPlanError> {
        Ok(GdbMemoryRead::new(self.command_len_address()?, 4))
    }

    /// Build a GDB memory read for the command payload after `command_len`
    /// has been read from the target.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError`] when `command_len` exceeds the
    /// mailbox capacity or an address overflows.
    pub fn command_payload_read(
        &self,
        command_len: u32,
    ) -> Result<GdbMemoryRead, GdbPilMailboxPlanError> {
        let len = command_len as usize;
        if len > self.layout.command_capacity {
            return Err(GdbPilMailboxPlanError::CommandPayloadTooLarge {
                capacity: self.layout.command_capacity,
                got: len,
            });
        }
        Ok(GdbMemoryRead::new(self.command_bytes_address()?, len))
    }

    /// Build a software-breakpoint insertion command for the resolved
    /// step-entry address.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::MissingStepEntryAddress`] when this
    /// plan was not built from an ELF symbol binding.
    pub fn step_entry_breakpoint_insert(
        &self,
    ) -> Result<GdbBreakpointCommand, GdbPilMailboxPlanError> {
        Ok(GdbBreakpointCommand::insert(
            self.resolved_step_entry_address()?,
        ))
    }

    /// Build a software-breakpoint removal command for the resolved
    /// step-entry address.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError::MissingStepEntryAddress`] when this
    /// plan was not built from an ELF symbol binding.
    pub fn step_entry_breakpoint_remove(
        &self,
    ) -> Result<GdbBreakpointCommand, GdbPilMailboxPlanError> {
        Ok(GdbBreakpointCommand::remove(
            self.resolved_step_entry_address()?,
        ))
    }

    fn resolved_step_entry_address(&self) -> Result<u64, GdbPilMailboxPlanError> {
        self.step_entry_address
            .ok_or_else(|| GdbPilMailboxPlanError::MissingStepEntryAddress {
                symbol: self.step_entry_symbol.clone(),
            })
    }

    fn field_address(
        &self,
        field: &'static str,
        offset: usize,
    ) -> Result<u64, GdbPilMailboxPlanError> {
        let offset = u64::try_from(offset)
            .map_err(|_| GdbPilMailboxPlanError::AddressOverflow { field, offset })?;
        self.mailbox_address
            .checked_add(offset)
            .ok_or(GdbPilMailboxPlanError::AddressOverflow {
                field,
                offset: usize::MAX,
            })
    }
}

/// Resolved target ELF symbols needed by the GDB mailbox coupling.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GdbPilSymbolBinding {
    /// Target symbol that names the mailbox instance.
    pub mailbox_symbol: String,
    /// Resolved target address of [`GdbPilSymbolBinding::mailbox_symbol`].
    pub mailbox_address: u64,
    /// Target function or breakpoint symbol that runs one mailbox step.
    pub step_entry_symbol: String,
    /// Resolved target address of [`GdbPilSymbolBinding::step_entry_symbol`].
    pub step_entry_address: u64,
}

impl GdbPilSymbolBinding {
    /// Resolve PIL mailbox symbols out of an ELF file.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError`] when the file cannot be read, the ELF
    /// cannot be parsed, either symbol is absent, or a symbol is undefined.
    pub fn resolve_from_elf_file(
        path: impl AsRef<Path>,
        mailbox_symbol: impl Into<String>,
        step_entry_symbol: impl Into<String>,
    ) -> Result<Self, GdbPilMailboxPlanError> {
        let path = path.as_ref();
        let bytes = fs::read(path).map_err(|source| GdbPilMailboxPlanError::ElfIo {
            path: path.display().to_string(),
            source,
        })?;
        Self::resolve_from_elf_bytes(&bytes, mailbox_symbol, step_entry_symbol)
    }

    /// Resolve PIL mailbox symbols out of ELF bytes.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError`] when the ELF cannot be parsed, either
    /// symbol is absent, or a symbol is undefined.
    pub fn resolve_from_elf_bytes(
        elf_bytes: &[u8],
        mailbox_symbol: impl Into<String>,
        step_entry_symbol: impl Into<String>,
    ) -> Result<Self, GdbPilMailboxPlanError> {
        let mailbox_symbol = mailbox_symbol.into();
        let step_entry_symbol = step_entry_symbol.into();
        validate_gdb_symbol_name("mailbox_symbol", &mailbox_symbol)?;
        validate_gdb_symbol_name("step_entry_symbol", &step_entry_symbol)?;

        let mut resolver = ElfSymbolResolver::parse(elf_bytes)?;
        let mailbox_address = resolver.resolve_defined_symbol(&mailbox_symbol)?;
        let step_entry_address = resolver.resolve_defined_symbol(&step_entry_symbol)?;

        Ok(Self {
            mailbox_symbol,
            mailbox_address,
            step_entry_symbol,
            step_entry_address,
        })
    }

    /// Build the GDB mailbox plan represented by this binding.
    ///
    /// # Errors
    ///
    /// Returns [`GdbPilMailboxPlanError`] when plan validation fails.
    pub fn mailbox_plan(
        &self,
        layout: PilMailboxLayout,
    ) -> Result<GdbPilMailboxPlan, GdbPilMailboxPlanError> {
        GdbPilMailboxPlan::new(
            self.mailbox_symbol.clone(),
            self.mailbox_address,
            layout,
            self.step_entry_symbol.clone(),
        )?
        .with_step_entry_address(self.step_entry_address)
    }
}

/// One GDB Remote Serial Protocol memory write command.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GdbMemoryWrite {
    /// Target address to write.
    pub address: u64,
    /// Bytes to write at [`GdbMemoryWrite::address`].
    pub bytes: Vec<u8>,
}

impl GdbMemoryWrite {
    /// Construct a memory write.
    #[must_use]
    pub fn new(address: u64, bytes: Vec<u8>) -> Self {
        Self { address, bytes }
    }

    /// Return the unframed RSP `M addr,length:hex` payload.
    #[must_use]
    pub fn rsp_payload(&self) -> String {
        format!(
            "M{:x},{:x}:{}",
            self.address,
            self.bytes.len(),
            hex_encode(&self.bytes)
        )
    }

    /// Return the checksum-framed RSP packet.
    #[must_use]
    pub fn rsp_packet(&self) -> String {
        gdb_rsp_packet(&self.rsp_payload())
    }
}

/// One GDB Remote Serial Protocol memory read command.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct GdbMemoryRead {
    /// Target address to read.
    pub address: u64,
    /// Number of bytes to read.
    pub len: usize,
}

impl GdbMemoryRead {
    /// Construct a memory read.
    #[must_use]
    pub const fn new(address: u64, len: usize) -> Self {
        Self { address, len }
    }

    /// Return the unframed RSP `m addr,length` payload.
    #[must_use]
    pub fn rsp_payload(self) -> String {
        format!("m{:x},{:x}", self.address, self.len)
    }

    /// Return the checksum-framed RSP packet.
    #[must_use]
    pub fn rsp_packet(self) -> String {
        gdb_rsp_packet(&self.rsp_payload())
    }
}

/// One GDB Remote Serial Protocol software-breakpoint command.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct GdbBreakpointCommand {
    /// Whether this command inserts or removes the breakpoint.
    pub action: GdbBreakpointAction,
    /// Target breakpoint address.
    pub address: u64,
    /// Target breakpoint kind byte count. Zero lets GDB/Renode infer the kind.
    pub kind: u32,
}

impl GdbBreakpointCommand {
    /// Construct a software-breakpoint insertion command.
    #[must_use]
    pub const fn insert(address: u64) -> Self {
        Self {
            action: GdbBreakpointAction::Insert,
            address,
            kind: 0,
        }
    }

    /// Construct a software-breakpoint removal command.
    #[must_use]
    pub const fn remove(address: u64) -> Self {
        Self {
            action: GdbBreakpointAction::Remove,
            address,
            kind: 0,
        }
    }

    /// Return the unframed RSP breakpoint payload.
    #[must_use]
    pub fn rsp_payload(self) -> String {
        let prefix = match self.action {
            GdbBreakpointAction::Insert => 'Z',
            GdbBreakpointAction::Remove => 'z',
        };
        format!("{prefix}0,{:x},{:x}", self.address, self.kind)
    }

    /// Return the checksum-framed RSP packet.
    #[must_use]
    pub fn rsp_packet(self) -> String {
        gdb_rsp_packet(&self.rsp_payload())
    }
}

/// GDB software-breakpoint command direction.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GdbBreakpointAction {
    /// Insert a breakpoint.
    Insert,
    /// Remove a breakpoint.
    Remove,
}

/// GDB Remote Serial Protocol session over a caller-owned byte stream.
///
/// This is the host-side exchange primitive for a future live GDB/Renode PIL
/// coupling. It owns packet framing, acknowledgement, checksums, memory
/// read/write commands, and mailbox-specific field decoding, but it does not
/// launch Renode or assert target equivalence by itself.
#[derive(Debug)]
pub struct GdbRspSession<S> {
    stream: S,
}

impl<S> GdbRspSession<S> {
    /// Construct a GDB RSP session over `stream`.
    #[must_use]
    pub const fn new(stream: S) -> Self {
        Self { stream }
    }

    /// Consume the session and return the wrapped stream.
    #[must_use]
    pub fn into_inner(self) -> S {
        self.stream
    }
}

impl<S: Read + Write> GdbRspSession<S> {
    /// Send one unframed RSP payload and return the unframed response payload.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when I/O fails, the remote rejects the
    /// packet, the response framing is malformed, or the checksum is invalid.
    pub fn exchange_payload(&mut self, payload: &str) -> Result<String, GdbRspSessionError> {
        self.stream.write_all(gdb_rsp_packet(payload).as_bytes())?;
        self.stream.flush()?;
        let ack = read_one_byte(&mut self.stream)?;
        match ack {
            b'+' => {}
            b'-' => return Err(GdbRspSessionError::NegativeAck),
            byte => return Err(GdbRspSessionError::UnexpectedAck { byte }),
        }
        let response = read_rsp_packet(&mut self.stream)?;
        self.stream.write_all(b"+")?;
        self.stream.flush()?;
        Ok(response)
    }

    /// Write target memory with one RSP `M` command.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when the exchange fails or the target
    /// does not return `OK`.
    pub fn write_memory(&mut self, write: &GdbMemoryWrite) -> Result<(), GdbRspSessionError> {
        let payload = write.rsp_payload();
        let response = self.exchange_payload(&payload)?;
        expect_ok_response(&payload, &response)
    }

    /// Read target memory with one RSP `m` command.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when the exchange fails, the target
    /// returns an error, the response is not valid hex, or the response length
    /// differs from the requested memory length.
    pub fn read_memory(&mut self, read: GdbMemoryRead) -> Result<Vec<u8>, GdbRspSessionError> {
        let payload = read.rsp_payload();
        let response = self.exchange_payload(&payload)?;
        if response.starts_with('E') {
            return Err(GdbRspSessionError::TargetError {
                command: payload,
                response,
            });
        }
        let bytes = decode_gdb_rsp_hex_bytes(&response)?;
        if bytes.len() != read.len {
            return Err(GdbRspSessionError::ReadLengthMismatch {
                expected: read.len,
                got: bytes.len(),
            });
        }
        Ok(bytes)
    }

    /// Insert a software breakpoint.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when the exchange fails or the target
    /// does not return `OK`.
    pub fn insert_breakpoint(
        &mut self,
        command: GdbBreakpointCommand,
    ) -> Result<(), GdbRspSessionError> {
        let payload = command.rsp_payload();
        let response = self.exchange_payload(&payload)?;
        expect_ok_response(&payload, &response)
    }

    /// Remove a software breakpoint.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when the exchange fails or the target
    /// does not return `OK`.
    pub fn remove_breakpoint(
        &mut self,
        command: GdbBreakpointCommand,
    ) -> Result<(), GdbRspSessionError> {
        let payload = command.rsp_payload();
        let response = self.exchange_payload(&payload)?;
        expect_ok_response(&payload, &response)
    }

    /// Continue the target and return the raw stop-reply payload.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when the exchange fails or the target
    /// returns an error reply.
    pub fn continue_target(&mut self) -> Result<String, GdbRspSessionError> {
        let response = self.exchange_payload("c")?;
        if response.starts_with('E') {
            return Err(GdbRspSessionError::TargetError {
                command: "c".to_owned(),
                response,
            });
        }
        Ok(response)
    }

    /// Load encoded sensor bytes into the target mailbox.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when plan validation, memory writes, or
    /// target responses fail.
    pub fn load_mailbox_sensor_payload(
        &mut self,
        plan: &GdbPilMailboxPlan,
        payload: &[u8],
    ) -> Result<(), GdbRspSessionError> {
        for write in plan.sensor_payload_writes(payload)? {
            self.write_memory(&write)?;
        }
        Ok(())
    }

    /// Load a bridge sensor packet into the target mailbox.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when encoding, plan validation, memory
    /// writes, or target responses fail.
    pub fn load_mailbox_sensor_packet(
        &mut self,
        plan: &GdbPilMailboxPlan,
        sensor: &SensorPacket,
    ) -> Result<(), GdbRspSessionError> {
        for write in plan.sensor_packet_writes(sensor)? {
            self.write_memory(&write)?;
        }
        Ok(())
    }

    /// Read the mailbox protocol version.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when the memory read or field decoding
    /// fails.
    pub fn read_mailbox_protocol_version(
        &mut self,
        plan: &GdbPilMailboxPlan,
    ) -> Result<u16, GdbRspSessionError> {
        let bytes = self.read_memory(plan.protocol_version_read()?)?;
        Ok(decode_gdb_mailbox_protocol_version(&bytes)?)
    }

    /// Read the mailbox status.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when the memory read or field decoding
    /// fails.
    pub fn read_mailbox_status(
        &mut self,
        plan: &GdbPilMailboxPlan,
    ) -> Result<PilMailboxStatus, GdbRspSessionError> {
        let bytes = self.read_memory(plan.status_read()?)?;
        Ok(decode_gdb_mailbox_status(&bytes)?)
    }

    /// Read the mailbox compact error code.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when the memory read or field decoding
    /// fails.
    pub fn read_mailbox_error(
        &mut self,
        plan: &GdbPilMailboxPlan,
    ) -> Result<PilMailboxError, GdbRspSessionError> {
        let bytes = self.read_memory(plan.error_read()?)?;
        Ok(decode_gdb_mailbox_error(&bytes)?)
    }

    /// Read the command payload length.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when the memory read or field decoding
    /// fails.
    pub fn read_mailbox_command_len(
        &mut self,
        plan: &GdbPilMailboxPlan,
    ) -> Result<u32, GdbRspSessionError> {
        let bytes = self.read_memory(plan.command_len_read()?)?;
        Ok(decode_gdb_mailbox_command_len(&bytes)?)
    }

    /// Read and decode the command packet currently stored in the target
    /// mailbox.
    ///
    /// # Errors
    ///
    /// Returns [`GdbRspSessionError`] when reading the command length,
    /// reading payload memory, or decoding the command packet fails.
    pub fn read_mailbox_command_packet(
        &mut self,
        plan: &GdbPilMailboxPlan,
    ) -> Result<ActuatorCommandPacket, GdbRspSessionError> {
        let command_len = self.read_mailbox_command_len(plan)?;
        let bytes = self.read_memory(plan.command_payload_read(command_len)?)?;
        Ok(decode_gdb_mailbox_command_payload(&bytes)?)
    }
}

/// Errors raised by live GDB Remote Serial Protocol exchanges.
#[derive(Debug, thiserror::Error)]
pub enum GdbRspSessionError {
    /// GDB RSP stream I/O failed.
    #[error("GDB RSP stream I/O failed: {0}")]
    Io(#[from] std::io::Error),
    /// The mailbox plan or mailbox decoder failed.
    #[error("GDB mailbox plan failed: {0}")]
    Plan(#[from] GdbPilMailboxPlanError),
    /// Remote replied with a negative acknowledgement.
    #[error("GDB RSP remote returned a negative acknowledgement")]
    NegativeAck,
    /// Remote replied with an unexpected acknowledgement byte.
    #[error("GDB RSP expected acknowledgement byte but received 0x{byte:02x}")]
    UnexpectedAck {
        /// Received byte.
        byte: u8,
    },
    /// Response packet did not start with `$`.
    #[error("GDB RSP response did not start with `$`, got 0x{byte:02x}")]
    PacketStart {
        /// Received byte.
        byte: u8,
    },
    /// Response packet ended before the checksum marker.
    #[error("GDB RSP response ended before checksum marker")]
    PacketTruncated,
    /// Response packet checksum was malformed.
    #[error("GDB RSP response checksum is malformed")]
    MalformedChecksum,
    /// Response packet checksum did not match its payload.
    #[error("GDB RSP response checksum mismatch: expected 0x{expected:02x}, got 0x{got:02x}")]
    ChecksumMismatch {
        /// Locally computed checksum.
        expected: u8,
        /// Remote-provided checksum.
        got: u8,
    },
    /// Response payload was not valid UTF-8.
    #[error("GDB RSP response payload is not valid UTF-8")]
    ResponseUtf8,
    /// Target returned an `E..` error packet.
    #[error("GDB RSP target returned `{response}` for command `{command}`")]
    TargetError {
        /// Command payload sent by the host.
        command: String,
        /// Target error response.
        response: String,
    },
    /// Target returned a non-OK response for an OK-acknowledged command.
    #[error("GDB RSP target returned unexpected `{response}` for command `{command}`")]
    UnexpectedResponse {
        /// Command payload sent by the host.
        command: String,
        /// Target response payload.
        response: String,
    },
    /// A memory read returned an unexpected number of bytes.
    #[error("GDB RSP memory read expected {expected} bytes but got {got}")]
    ReadLengthMismatch {
        /// Expected byte count.
        expected: usize,
        /// Actual byte count.
        got: usize,
    },
}

/// Decode bytes returned by an RSP memory-read payload.
///
/// The input is the hex payload bytes after the RSP `$...#cc` framing has
/// already been stripped.
///
/// # Errors
///
/// Returns [`GdbPilMailboxPlanError`] when the payload has odd length or
/// contains non-hex characters.
pub fn decode_gdb_rsp_hex_bytes(hex: &str) -> Result<Vec<u8>, GdbPilMailboxPlanError> {
    if !hex.len().is_multiple_of(2) {
        return Err(GdbPilMailboxPlanError::OddHexPayloadLength { len: hex.len() });
    }
    let mut bytes = Vec::with_capacity(hex.len() / 2);
    for (index, pair) in hex.as_bytes().chunks_exact(2).enumerate() {
        let high = hex_value(pair[0]).ok_or(GdbPilMailboxPlanError::InvalidHexDigit {
            index: index * 2,
            digit: char::from(pair[0]),
        })?;
        let low = hex_value(pair[1]).ok_or(GdbPilMailboxPlanError::InvalidHexDigit {
            index: index * 2 + 1,
            digit: char::from(pair[1]),
        })?;
        bytes.push((high << 4) | low);
    }
    Ok(bytes)
}

/// Decode a two-byte little-endian mailbox protocol version.
///
/// # Errors
///
/// Returns [`GdbPilMailboxPlanError`] when the byte count is not exactly two.
pub fn decode_gdb_mailbox_protocol_version(bytes: &[u8]) -> Result<u16, GdbPilMailboxPlanError> {
    let [lo, hi] = *bytes else {
        return Err(GdbPilMailboxPlanError::InvalidReadLength {
            field: "protocol_version",
            expected: 2,
            got: bytes.len(),
        });
    };
    Ok(u16::from_le_bytes([lo, hi]))
}

/// Decode a one-byte mailbox status.
///
/// # Errors
///
/// Returns [`GdbPilMailboxPlanError`] when the byte count is not exactly one
/// or the status code is not defined by `openbmp-pil`.
pub fn decode_gdb_mailbox_status(bytes: &[u8]) -> Result<PilMailboxStatus, GdbPilMailboxPlanError> {
    let [code] = *bytes else {
        return Err(GdbPilMailboxPlanError::InvalidReadLength {
            field: "status",
            expected: 1,
            got: bytes.len(),
        });
    };
    match code {
        0 => Ok(PilMailboxStatus::Empty),
        1 => Ok(PilMailboxStatus::SensorReady),
        2 => Ok(PilMailboxStatus::CommandReady),
        3 => Ok(PilMailboxStatus::Fault),
        other => Err(GdbPilMailboxPlanError::InvalidStatusCode { code: other }),
    }
}

/// Decode a one-byte mailbox error code.
///
/// # Errors
///
/// Returns [`GdbPilMailboxPlanError`] when the byte count is not exactly one
/// or the error code is not defined by `openbmp-pil`.
pub fn decode_gdb_mailbox_error(bytes: &[u8]) -> Result<PilMailboxError, GdbPilMailboxPlanError> {
    let [code] = *bytes else {
        return Err(GdbPilMailboxPlanError::InvalidReadLength {
            field: "error",
            expected: 1,
            got: bytes.len(),
        });
    };
    match code {
        0 => Ok(PilMailboxError::None),
        1 => Ok(PilMailboxError::SensorTooLarge),
        2 => Ok(PilMailboxError::InvalidSensorLength),
        3 => Ok(PilMailboxError::InvalidCommandLength),
        4 => Ok(PilMailboxError::Decode),
        5 => Ok(PilMailboxError::Encode),
        6 => Ok(PilMailboxError::Controller),
        7 => Ok(PilMailboxError::CommandIdOutOfRange),
        8 => Ok(PilMailboxError::OutputBufferTooSmall),
        other => Err(GdbPilMailboxPlanError::InvalidErrorCode { code: other }),
    }
}

/// Decode a four-byte little-endian command length.
///
/// # Errors
///
/// Returns [`GdbPilMailboxPlanError`] when the byte count is not exactly four.
pub fn decode_gdb_mailbox_command_len(bytes: &[u8]) -> Result<u32, GdbPilMailboxPlanError> {
    let [b0, b1, b2, b3] = *bytes else {
        return Err(GdbPilMailboxPlanError::InvalidReadLength {
            field: "command_len",
            expected: 4,
            got: bytes.len(),
        });
    };
    Ok(u32::from_le_bytes([b0, b1, b2, b3]))
}

/// Decode a mailbox command payload into an actuator-command packet.
///
/// # Errors
///
/// Returns [`GdbPilMailboxPlanError`] when postcard decoding fails.
pub fn decode_gdb_mailbox_command_payload(
    bytes: &[u8],
) -> Result<ActuatorCommandPacket, GdbPilMailboxPlanError> {
    Ok(decode(bytes)?)
}

/// Errors raised while validating or using a GDB mailbox binding plan.
#[derive(Debug, thiserror::Error)]
pub enum GdbPilMailboxPlanError {
    /// A required field was empty.
    #[error("GDB PIL mailbox plan field `{field}` must not be empty")]
    EmptyField {
        /// Field name.
        field: &'static str,
    },
    /// A field contains a character the renderer deliberately does not quote.
    #[error("GDB PIL mailbox plan field `{field}` contains unsupported character `{character}`")]
    UnsupportedCharacter {
        /// Field name.
        field: &'static str,
        /// Unsupported character.
        character: char,
    },
    /// The resolved mailbox address was zero.
    #[error("GDB PIL mailbox address must be nonzero")]
    ZeroMailboxAddress,
    /// The resolved step-entry address was zero.
    #[error("GDB PIL step-entry symbol `{symbol}` address must be nonzero")]
    ZeroStepEntryAddress {
        /// Step-entry symbol name.
        symbol: String,
    },
    /// A plan did not carry a resolved step-entry address.
    #[error("GDB PIL step-entry symbol `{symbol}` has not been resolved to an address")]
    MissingStepEntryAddress {
        /// Step-entry symbol name.
        symbol: String,
    },
    /// A mailbox field address overflowed.
    #[error("GDB PIL mailbox field `{field}` address overflow at offset {offset}")]
    AddressOverflow {
        /// Field name.
        field: &'static str,
        /// Offset that could not be added.
        offset: usize,
    },
    /// Reading a target ELF file failed.
    #[error("could not read target ELF {path}: {source}")]
    ElfIo {
        /// Target ELF path.
        path: String,
        /// I/O failure.
        source: std::io::Error,
    },
    /// Target ELF bytes are malformed.
    #[error("target ELF is malformed: {reason}")]
    ElfMalformed {
        /// Reason the parser rejected the ELF.
        reason: &'static str,
    },
    /// Target ELF uses an unsupported format.
    #[error("target ELF format is unsupported: {reason}")]
    ElfUnsupported {
        /// Unsupported format reason.
        reason: &'static str,
    },
    /// A required target ELF symbol was absent.
    #[error("target ELF is missing required symbol `{symbol}`")]
    ElfSymbolMissing {
        /// Missing symbol name.
        symbol: String,
    },
    /// A required target ELF symbol was present but undefined.
    #[error("target ELF symbol `{symbol}` is undefined")]
    ElfSymbolUndefined {
        /// Undefined symbol name.
        symbol: String,
    },
    /// A required target ELF symbol resolved to address zero.
    #[error("target ELF symbol `{symbol}` resolved to address zero")]
    ElfSymbolZeroAddress {
        /// Symbol name.
        symbol: String,
    },
    /// A required target ELF symbol has conflicting definitions.
    #[error("target ELF symbol `{symbol}` has conflicting definitions")]
    ElfSymbolConflict {
        /// Conflicting symbol name.
        symbol: String,
    },
    /// Encoded sensor bytes do not fit in target mailbox storage.
    #[error("encoded sensor payload requires {got} bytes but target mailbox holds {capacity}")]
    SensorPayloadTooLarge {
        /// Target mailbox sensor capacity.
        capacity: usize,
        /// Encoded sensor byte count.
        got: usize,
    },
    /// Encoded command bytes do not fit in target mailbox storage.
    #[error("encoded command payload requires {got} bytes but target mailbox holds {capacity}")]
    CommandPayloadTooLarge {
        /// Target mailbox command capacity.
        capacity: usize,
        /// Encoded command byte count.
        got: usize,
    },
    /// Hex payload length was odd.
    #[error("GDB RSP hex payload has odd length {len}")]
    OddHexPayloadLength {
        /// Hex character count.
        len: usize,
    },
    /// Hex payload contains a non-hex digit.
    #[error("GDB RSP hex payload contains invalid digit `{digit}` at index {index}")]
    InvalidHexDigit {
        /// Character index in the hex payload.
        index: usize,
        /// Invalid hex digit.
        digit: char,
    },
    /// A memory read returned the wrong number of bytes for a mailbox field.
    #[error("GDB mailbox field `{field}` read expected {expected} bytes but got {got}")]
    InvalidReadLength {
        /// Field name.
        field: &'static str,
        /// Expected byte count.
        expected: usize,
        /// Actual byte count.
        got: usize,
    },
    /// A status byte did not match a defined `PilMailboxStatus`.
    #[error("GDB mailbox status code {code} is not defined")]
    InvalidStatusCode {
        /// Raw target status byte.
        code: u8,
    },
    /// An error byte did not match a defined `PilMailboxError`.
    #[error("GDB mailbox error code {code} is not defined")]
    InvalidErrorCode {
        /// Raw target error byte.
        code: u8,
    },
    /// Bridge encode/decode failed.
    #[error("GDB mailbox bridge codec failed: {0}")]
    Bridge(#[from] BridgeError),
}

struct ElfSymbolResolver<'a> {
    bytes: &'a [u8],
    class: ElfClass,
    endian: ElfEndian,
    section_headers: Vec<ElfSectionHeader>,
}

impl<'a> ElfSymbolResolver<'a> {
    fn parse(bytes: &'a [u8]) -> Result<Self, GdbPilMailboxPlanError> {
        if bytes.len() < 16 || &bytes[..4] != b"\x7fELF" {
            return Err(GdbPilMailboxPlanError::ElfMalformed {
                reason: "missing ELF magic",
            });
        }
        let class = match bytes[4] {
            1 => ElfClass::Elf32,
            2 => ElfClass::Elf64,
            _ => {
                return Err(GdbPilMailboxPlanError::ElfUnsupported {
                    reason: "unsupported ELF class",
                });
            }
        };
        let endian = match bytes[5] {
            1 => ElfEndian::Little,
            2 => ElfEndian::Big,
            _ => {
                return Err(GdbPilMailboxPlanError::ElfUnsupported {
                    reason: "unsupported ELF endianness",
                });
            }
        };
        if bytes[6] != 1 {
            return Err(GdbPilMailboxPlanError::ElfUnsupported {
                reason: "unsupported ELF version",
            });
        }

        let (section_header_offset, section_header_size, section_count) = match class {
            ElfClass::Elf32 => {
                if bytes.len() < 52 {
                    return Err(GdbPilMailboxPlanError::ElfMalformed {
                        reason: "truncated ELF32 header",
                    });
                }
                (
                    read_u32(bytes, 32, endian)? as u64,
                    usize::from(read_u16(bytes, 46, endian)?),
                    usize::from(read_u16(bytes, 48, endian)?),
                )
            }
            ElfClass::Elf64 => {
                if bytes.len() < 64 {
                    return Err(GdbPilMailboxPlanError::ElfMalformed {
                        reason: "truncated ELF64 header",
                    });
                }
                (
                    read_u64(bytes, 40, endian)?,
                    usize::from(read_u16(bytes, 58, endian)?),
                    usize::from(read_u16(bytes, 60, endian)?),
                )
            }
        };
        if section_count == 0 {
            return Err(GdbPilMailboxPlanError::ElfMalformed {
                reason: "ELF has no section headers",
            });
        }
        let minimum_section_size = match class {
            ElfClass::Elf32 => 40,
            ElfClass::Elf64 => 64,
        };
        if section_header_size < minimum_section_size {
            return Err(GdbPilMailboxPlanError::ElfMalformed {
                reason: "section header entries are too small",
            });
        }
        let section_table_offset = usize::try_from(section_header_offset).map_err(|_| {
            GdbPilMailboxPlanError::ElfMalformed {
                reason: "section header offset is too large",
            }
        })?;
        let section_table_size = section_header_size.checked_mul(section_count).ok_or(
            GdbPilMailboxPlanError::ElfMalformed {
                reason: "section header table size overflow",
            },
        )?;
        checked_slice(bytes, section_table_offset, section_table_size)?;

        let mut section_headers = Vec::with_capacity(section_count);
        for index in 0..section_count {
            let offset = section_table_offset + index * section_header_size;
            section_headers.push(ElfSectionHeader::parse(bytes, offset, class, endian)?);
        }

        Ok(Self {
            bytes,
            class,
            endian,
            section_headers,
        })
    }

    fn resolve_defined_symbol(&mut self, name: &str) -> Result<u64, GdbPilMailboxPlanError> {
        let mut found: Option<u64> = None;
        let mut saw_undefined = false;

        for section in &self.section_headers {
            if !matches!(
                section.kind,
                ElfSectionKind::Symtab | ElfSectionKind::Dynsym
            ) {
                continue;
            }
            let string_table = self.section_headers.get(section.link).ok_or(
                GdbPilMailboxPlanError::ElfMalformed {
                    reason: "symbol table links outside section table",
                },
            )?;
            let strings = checked_slice(
                self.bytes,
                usize::try_from(string_table.offset).map_err(|_| {
                    GdbPilMailboxPlanError::ElfMalformed {
                        reason: "string table offset is too large",
                    }
                })?,
                usize::try_from(string_table.size).map_err(|_| {
                    GdbPilMailboxPlanError::ElfMalformed {
                        reason: "string table size is too large",
                    }
                })?,
            )?;
            let entries = self.symbol_entries(section)?;
            for symbol_offset in entries {
                let symbol = ElfSymbol::parse(self.bytes, symbol_offset, self.class, self.endian)?;
                if symbol.name_offset == 0 {
                    continue;
                }
                let symbol_name = elf_string(strings, symbol.name_offset)?;
                if symbol_name != name {
                    continue;
                }
                if symbol.section_index == 0 {
                    saw_undefined = true;
                    continue;
                }
                if symbol.value == 0 {
                    return Err(GdbPilMailboxPlanError::ElfSymbolZeroAddress {
                        symbol: name.to_owned(),
                    });
                }
                match found {
                    Some(address) if address != symbol.value => {
                        return Err(GdbPilMailboxPlanError::ElfSymbolConflict {
                            symbol: name.to_owned(),
                        });
                    }
                    Some(_) => {}
                    None => found = Some(symbol.value),
                }
            }
        }

        match found {
            Some(address) => Ok(address),
            None if saw_undefined => Err(GdbPilMailboxPlanError::ElfSymbolUndefined {
                symbol: name.to_owned(),
            }),
            None => Err(GdbPilMailboxPlanError::ElfSymbolMissing {
                symbol: name.to_owned(),
            }),
        }
    }

    fn symbol_entries(
        &self,
        section: &ElfSectionHeader,
    ) -> Result<Vec<usize>, GdbPilMailboxPlanError> {
        let minimum_symbol_size = match self.class {
            ElfClass::Elf32 => 16,
            ElfClass::Elf64 => 24,
        };
        if section.entry_size < minimum_symbol_size {
            return Err(GdbPilMailboxPlanError::ElfMalformed {
                reason: "symbol table entries are too small",
            });
        }
        if !section.size.is_multiple_of(section.entry_size as u64) {
            return Err(GdbPilMailboxPlanError::ElfMalformed {
                reason: "symbol table size is not a multiple of entry size",
            });
        }
        let offset =
            usize::try_from(section.offset).map_err(|_| GdbPilMailboxPlanError::ElfMalformed {
                reason: "symbol table offset is too large",
            })?;
        let size =
            usize::try_from(section.size).map_err(|_| GdbPilMailboxPlanError::ElfMalformed {
                reason: "symbol table size is too large",
            })?;
        checked_slice(self.bytes, offset, size)?;
        let count = size / section.entry_size;
        Ok((0..count)
            .map(|index| offset + index * section.entry_size)
            .collect())
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum ElfClass {
    Elf32,
    Elf64,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum ElfEndian {
    Little,
    Big,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum ElfSectionKind {
    Other,
    Symtab,
    Dynsym,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
struct ElfSectionHeader {
    kind: ElfSectionKind,
    offset: u64,
    size: u64,
    link: usize,
    entry_size: usize,
}

impl ElfSectionHeader {
    fn parse(
        bytes: &[u8],
        offset: usize,
        class: ElfClass,
        endian: ElfEndian,
    ) -> Result<Self, GdbPilMailboxPlanError> {
        let section_type = read_u32(bytes, offset + 4, endian)?;
        let kind = match section_type {
            2 => ElfSectionKind::Symtab,
            11 => ElfSectionKind::Dynsym,
            _ => ElfSectionKind::Other,
        };
        let (offset, size, link, entry_size) = match class {
            ElfClass::Elf32 => (
                u64::from(read_u32(bytes, offset + 16, endian)?),
                u64::from(read_u32(bytes, offset + 20, endian)?),
                read_u32(bytes, offset + 24, endian)? as usize,
                read_u32(bytes, offset + 36, endian)? as usize,
            ),
            ElfClass::Elf64 => (
                read_u64(bytes, offset + 24, endian)?,
                read_u64(bytes, offset + 32, endian)?,
                read_u32(bytes, offset + 40, endian)? as usize,
                read_u64(bytes, offset + 56, endian)? as usize,
            ),
        };
        Ok(Self {
            kind,
            offset,
            size,
            link,
            entry_size,
        })
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
struct ElfSymbol {
    name_offset: usize,
    value: u64,
    section_index: u16,
}

impl ElfSymbol {
    fn parse(
        bytes: &[u8],
        offset: usize,
        class: ElfClass,
        endian: ElfEndian,
    ) -> Result<Self, GdbPilMailboxPlanError> {
        match class {
            ElfClass::Elf32 => Ok(Self {
                name_offset: read_u32(bytes, offset, endian)? as usize,
                value: u64::from(read_u32(bytes, offset + 4, endian)?),
                section_index: read_u16(bytes, offset + 14, endian)?,
            }),
            ElfClass::Elf64 => Ok(Self {
                name_offset: read_u32(bytes, offset, endian)? as usize,
                value: read_u64(bytes, offset + 8, endian)?,
                section_index: read_u16(bytes, offset + 6, endian)?,
            }),
        }
    }
}

fn checked_slice(bytes: &[u8], offset: usize, len: usize) -> Result<&[u8], GdbPilMailboxPlanError> {
    let end = offset
        .checked_add(len)
        .ok_or(GdbPilMailboxPlanError::ElfMalformed {
            reason: "byte range overflow",
        })?;
    bytes
        .get(offset..end)
        .ok_or(GdbPilMailboxPlanError::ElfMalformed {
            reason: "byte range is outside ELF",
        })
}

fn read_u16(bytes: &[u8], offset: usize, endian: ElfEndian) -> Result<u16, GdbPilMailboxPlanError> {
    let bytes = read_array::<2>(bytes, offset)?;
    Ok(match endian {
        ElfEndian::Little => u16::from_le_bytes(bytes),
        ElfEndian::Big => u16::from_be_bytes(bytes),
    })
}

fn read_u32(bytes: &[u8], offset: usize, endian: ElfEndian) -> Result<u32, GdbPilMailboxPlanError> {
    let bytes = read_array::<4>(bytes, offset)?;
    Ok(match endian {
        ElfEndian::Little => u32::from_le_bytes(bytes),
        ElfEndian::Big => u32::from_be_bytes(bytes),
    })
}

fn read_u64(bytes: &[u8], offset: usize, endian: ElfEndian) -> Result<u64, GdbPilMailboxPlanError> {
    let bytes = read_array::<8>(bytes, offset)?;
    Ok(match endian {
        ElfEndian::Little => u64::from_le_bytes(bytes),
        ElfEndian::Big => u64::from_be_bytes(bytes),
    })
}

fn read_array<const N: usize>(
    bytes: &[u8],
    offset: usize,
) -> Result<[u8; N], GdbPilMailboxPlanError> {
    let slice = checked_slice(bytes, offset, N)?;
    let mut out = [0_u8; N];
    out.copy_from_slice(slice);
    Ok(out)
}

fn elf_string(strings: &[u8], offset: usize) -> Result<&str, GdbPilMailboxPlanError> {
    let bytes = strings
        .get(offset..)
        .ok_or(GdbPilMailboxPlanError::ElfMalformed {
            reason: "symbol name offset is outside string table",
        })?;
    let nul =
        bytes
            .iter()
            .position(|byte| *byte == 0)
            .ok_or(GdbPilMailboxPlanError::ElfMalformed {
                reason: "symbol name is not NUL-terminated",
            })?;
    std::str::from_utf8(&bytes[..nul]).map_err(|_| GdbPilMailboxPlanError::ElfMalformed {
        reason: "symbol name is not valid UTF-8",
    })
}

fn validate_gdb_symbol_name(
    field: &'static str,
    value: &str,
) -> Result<(), GdbPilMailboxPlanError> {
    if value.is_empty() {
        return Err(GdbPilMailboxPlanError::EmptyField { field });
    }
    for character in value.chars() {
        if !(character.is_ascii_alphanumeric() || matches!(character, '_' | '.' | '$')) {
            return Err(GdbPilMailboxPlanError::UnsupportedCharacter { field, character });
        }
    }
    Ok(())
}

fn expect_ok_response(command: &str, response: &str) -> Result<(), GdbRspSessionError> {
    if response == "OK" {
        return Ok(());
    }
    if response.starts_with('E') {
        return Err(GdbRspSessionError::TargetError {
            command: command.to_owned(),
            response: response.to_owned(),
        });
    }
    Err(GdbRspSessionError::UnexpectedResponse {
        command: command.to_owned(),
        response: response.to_owned(),
    })
}

fn read_rsp_packet<R: Read>(reader: &mut R) -> Result<String, GdbRspSessionError> {
    let start = read_one_byte(reader)?;
    if start != b'$' {
        return Err(GdbRspSessionError::PacketStart { byte: start });
    }

    let mut payload = Vec::new();
    loop {
        let byte = read_one_byte(reader).map_err(|error| match error {
            GdbRspSessionError::Io(io) if io.kind() == std::io::ErrorKind::UnexpectedEof => {
                GdbRspSessionError::PacketTruncated
            }
            other => other,
        })?;
        if byte == b'#' {
            break;
        }
        payload.push(byte);
    }

    let high = read_one_byte(reader)?;
    let low = read_one_byte(reader)?;
    let Some(high) = hex_value(high) else {
        return Err(GdbRspSessionError::MalformedChecksum);
    };
    let Some(low) = hex_value(low) else {
        return Err(GdbRspSessionError::MalformedChecksum);
    };
    let got = (high << 4) | low;
    let expected = payload
        .iter()
        .fold(0_u8, |sum, byte| sum.wrapping_add(*byte));
    if got != expected {
        return Err(GdbRspSessionError::ChecksumMismatch { expected, got });
    }

    String::from_utf8(payload).map_err(|_| GdbRspSessionError::ResponseUtf8)
}

fn read_one_byte<R: Read>(reader: &mut R) -> Result<u8, GdbRspSessionError> {
    let mut byte = [0_u8; 1];
    reader.read_exact(&mut byte)?;
    Ok(byte[0])
}

fn gdb_rsp_packet(payload: &str) -> String {
    let checksum = payload
        .as_bytes()
        .iter()
        .fold(0_u8, |sum, byte| sum.wrapping_add(*byte));
    format!("${payload}#{checksum:02x}")
}

fn hex_encode(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut encoded = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        encoded.push(char::from(HEX[usize::from(byte >> 4)]));
        encoded.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    encoded
}

fn hex_value(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

/// Host-side raw frame stream for socket-UART PIL coupling.
///
/// Unlike the host SIL transport, this stream intentionally exchanges raw
/// length-prefixed `SensorPacket` / `ActuatorCommandPacket` payloads because
/// that is the byte contract consumed by `openbmp-pil` target firmware shims.
#[derive(Debug)]
pub struct PilFrameStream<S> {
    stream: S,
    read_buf: Vec<u8>,
    max_payload_len: usize,
}

/// TCP-backed PIL socket client for a Renode UART terminal.
pub type RenodeSocketPilClient = PilFrameStream<TcpStream>;

impl<S> PilFrameStream<S> {
    /// Construct a PIL frame stream with the default 4096-byte payload limit.
    #[must_use]
    pub fn new(stream: S) -> Self {
        Self::with_max_payload_len(stream, DEFAULT_PIL_MAX_PAYLOAD_LEN)
    }

    /// Construct a PIL frame stream with an explicit payload limit.
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

    /// Consume this client and return the wrapped stream object.
    #[must_use]
    pub fn into_inner(self) -> S {
        self.stream
    }
}

impl RenodeSocketPilClient {
    /// Connect to the local TCP socket exposed by a rendered
    /// [`RenodePilPlan`].
    ///
    /// # Errors
    ///
    /// Returns [`PilClientError`] if the plan is invalid or the TCP
    /// connection fails.
    pub fn connect_plan(plan: &RenodePilPlan) -> Result<Self, PilClientError> {
        plan.validate()?;
        Ok(Self::new(
            TcpStream::connect(("127.0.0.1", plan.uart_host_port)).map_err(BridgeError::from)?,
        ))
    }

    /// Connect to any caller-provided socket address.
    ///
    /// # Errors
    ///
    /// Returns [`PilClientError`] if the TCP connection fails.
    pub fn connect_addr<A: ToSocketAddrs>(addr: A) -> Result<Self, PilClientError> {
        Ok(Self::new(
            TcpStream::connect(addr).map_err(BridgeError::from)?,
        ))
    }
}

impl<S: Read + Write> PilFrameStream<S> {
    /// Send one sensor frame and read one matching actuator command frame.
    ///
    /// # Errors
    ///
    /// Returns [`PilClientError`] when encoding, stream I/O, decoding, payload
    /// size checks, or lockstep command validation fails.
    pub fn step(&mut self, sensor: &SensorPacket) -> Result<ActuatorCommandPacket, PilClientError> {
        self.send_sensor(sensor)?;
        let command = self.recv_command()?;
        validate_command_for_sensor(sensor, &command)?;
        Ok(command)
    }

    /// Send one length-prefixed encoded sensor packet.
    ///
    /// # Errors
    ///
    /// Returns [`PilClientError`] when encoding, payload size validation, or
    /// stream I/O fails.
    pub fn send_sensor(&mut self, sensor: &SensorPacket) -> Result<(), PilClientError> {
        let payload = encode(sensor)?;
        if payload.len() > self.max_payload_len {
            return Err(BridgeError::PayloadTooLarge {
                max: self.max_payload_len,
                got: payload.len(),
            }
            .into());
        }
        self.stream
            .write_all(&frame(&payload))
            .map_err(BridgeError::from)?;
        self.stream.flush().map_err(BridgeError::from)?;
        Ok(())
    }

    /// Read one length-prefixed encoded actuator command packet.
    ///
    /// # Errors
    ///
    /// Returns [`PilClientError`] when the stream closes, I/O fails, the
    /// payload exceeds the configured limit, or decoding fails.
    pub fn recv_command(&mut self) -> Result<ActuatorCommandPacket, PilClientError> {
        let mut scratch = [0_u8; 1024];
        loop {
            if let Some(frame_len) = buffered_frame_len(&self.read_buf, self.max_payload_len)? {
                let command = decode(&self.read_buf[FRAME_PREFIX_LEN..frame_len])?;
                discard_read_prefix(&mut self.read_buf, frame_len);
                return Ok(command);
            }

            let read = self.stream.read(&mut scratch).map_err(BridgeError::from)?;
            if read == 0 {
                return Err(BridgeError::TransportClosed.into());
            }
            self.read_buf.extend_from_slice(&scratch[..read]);
        }
    }
}

/// Errors raised by host-side raw PIL frame streams.
#[derive(Debug, thiserror::Error)]
pub enum PilClientError {
    /// The Renode plan is not valid.
    #[error("invalid Renode PIL plan: {0}")]
    Plan(#[from] RenodePilPlanError),
    /// Encoding, decoding, framing, validation, or transport I/O failed.
    #[error("PIL frame stream failed: {0}")]
    Bridge(#[from] BridgeError),
}

fn buffered_frame_len(
    read_buf: &[u8],
    max_payload_len: usize,
) -> Result<Option<usize>, BridgeError> {
    if read_buf.len() < FRAME_PREFIX_LEN {
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
    let frame_len = FRAME_PREFIX_LEN + payload_len;
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

/// Errors raised while validating or rendering a Renode PIL plan.
#[derive(Debug, thiserror::Error, Eq, PartialEq)]
pub enum RenodePilPlanError {
    /// A required field was empty.
    #[error("Renode PIL plan field `{field}` must not be empty")]
    EmptyField {
        /// Field name.
        field: &'static str,
    },
    /// A field contains a character the renderer deliberately does not quote.
    #[error("Renode PIL plan field `{field}` contains unsupported character `{character}`")]
    UnsupportedCharacter {
        /// Field name.
        field: &'static str,
        /// Unsupported character.
        character: char,
    },
    /// A Renode TCP port was zero.
    #[error("Renode PIL plan field `{field}` must be a nonzero TCP port")]
    ZeroPort {
        /// Field name.
        field: &'static str,
    },
    /// The requested global quantum was zero.
    #[error("Renode PIL global quantum must be greater than zero")]
    ZeroQuantum,
}

fn validate_machine_name(field: &'static str, value: &str) -> Result<(), RenodePilPlanError> {
    validate_common_text(field, value)?;
    for character in value.chars() {
        if !(character.is_ascii_alphanumeric() || matches!(character, '-' | '_')) {
            return Err(RenodePilPlanError::UnsupportedCharacter { field, character });
        }
    }
    Ok(())
}

fn validate_renode_path(field: &'static str, value: &str) -> Result<(), RenodePilPlanError> {
    validate_common_text(field, value)?;
    for character in value.chars() {
        if !(character.is_ascii_alphanumeric() || matches!(character, '/' | '.' | '-' | '_')) {
            return Err(RenodePilPlanError::UnsupportedCharacter { field, character });
        }
    }
    Ok(())
}

fn validate_peripheral_name(field: &'static str, value: &str) -> Result<(), RenodePilPlanError> {
    validate_common_text(field, value)?;
    for character in value.chars() {
        if !(character.is_ascii_alphanumeric() || matches!(character, '.' | '_')) {
            return Err(RenodePilPlanError::UnsupportedCharacter { field, character });
        }
    }
    Ok(())
}

fn validate_terminal_name(field: &'static str, value: &str) -> Result<(), RenodePilPlanError> {
    validate_common_text(field, value)?;
    for character in value.chars() {
        if !(character.is_ascii_alphanumeric() || character == '_') {
            return Err(RenodePilPlanError::UnsupportedCharacter { field, character });
        }
    }
    Ok(())
}

fn validate_common_text(field: &'static str, value: &str) -> Result<(), RenodePilPlanError> {
    if value.is_empty() {
        return Err(RenodePilPlanError::EmptyField { field });
    }
    Ok(())
}

fn validate_port(field: &'static str, port: u16) -> Result<(), RenodePilPlanError> {
    if port == 0 {
        return Err(RenodePilPlanError::ZeroPort { field });
    }
    Ok(())
}

fn format_quantum_seconds(quantum_us: u64) -> String {
    let seconds = quantum_us / 1_000_000;
    let micros = quantum_us % 1_000_000;
    format!("{seconds}.{micros:06}")
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use std::io::{Cursor, Read, Write};

    use openbmp_bridge::EngineCommandPacket;
    use openbmp_pil::PilMailbox;

    use super::*;

    #[derive(Debug)]
    struct MockDuplex {
        read: Cursor<Vec<u8>>,
        written: Vec<u8>,
        max_read_chunk: usize,
    }

    impl MockDuplex {
        fn new(read_bytes: Vec<u8>) -> Self {
            Self {
                read: Cursor::new(read_bytes),
                written: Vec::new(),
                max_read_chunk: usize::MAX,
            }
        }

        fn with_max_read_chunk(mut self, max_read_chunk: usize) -> Self {
            self.max_read_chunk = max_read_chunk;
            self
        }
    }

    impl Read for MockDuplex {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let len = buf.len().min(self.max_read_chunk);
            self.read.read(&mut buf[..len])
        }
    }

    impl Write for MockDuplex {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.written.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[derive(Debug)]
    struct MockRsp {
        read: Cursor<Vec<u8>>,
        written: Vec<u8>,
    }

    impl MockRsp {
        fn new(read_bytes: Vec<u8>) -> Self {
            Self {
                read: Cursor::new(read_bytes),
                written: Vec::new(),
            }
        }
    }

    impl Read for MockRsp {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            self.read.read(buf)
        }
    }

    impl Write for MockRsp {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.written.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    fn rsp_response(payload: &str) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.push(b'+');
        bytes.extend_from_slice(gdb_rsp_packet(payload).as_bytes());
        bytes
    }

    fn rsp_responses(payloads: &[String]) -> Vec<u8> {
        let mut bytes = Vec::new();
        for payload in payloads {
            bytes.extend_from_slice(&rsp_response(payload));
        }
        bytes
    }

    fn sensor(step: u64) -> SensorPacket {
        SensorPacket {
            sim_time_s: step as f64 * 0.01,
            step,
            imu_accel_body_m_s2: [0.0, 0.0, 9.81],
            imu_gyro_body_rad_s: [0.01, -0.02, 0.03],
            baro_altitude_m: Some(1_000.0),
            gnss_position_eci_m: Some([10.0, 20.0, 30.0]),
            gnss_velocity_eci_m_s: Some([1.0, 2.0, 3.0]),
            gnss_position_bias_eci_m: Some([0.1, 0.2, 0.3]),
            mag_body_tesla: None,
            mag_body_nt: Some([1000.0, 2000.0, 3000.0]),
            mag_hard_iron_body_nt: Some([1.0, 2.0, 3.0]),
            baro_pressure_pa: Some(91_250.0),
            baro_bias_pa: Some(12.0),
            star_tracker_attitude_eci_to_body_xyzw: Some([0.0, 0.1, 0.2, 0.97]),
            ..SensorPacket::default()
        }
    }

    fn command_for(sensor: &SensorPacket) -> ActuatorCommandPacket {
        ActuatorCommandPacket {
            sim_time_s: sensor.sim_time_s,
            step: sensor.step,
            effector_commands: vec![(7, 0.5)],
            engine_throttles: vec![(9, 0.64)],
            engine_commands: vec![EngineCommandPacket {
                engine_id: 9,
                throttle_unit: 0.64,
                gimbal_pitch_rad: 0.01,
                gimbal_yaw_rad: -0.02,
                ignite: true,
                shutdown: false,
            }],
        }
    }

    fn framed_command(command: &ActuatorCommandPacket) -> Vec<u8> {
        frame(&encode(command).unwrap())
    }

    fn minimal_pil_elf32(mailbox_address: u32, step_entry_address: u32) -> Vec<u8> {
        const STRTAB_OFFSET: usize = 0x80;
        const SYMTAB_OFFSET: usize = 0xc0;
        const SHOFF: usize = 0x100;
        const SHENTSIZE: usize = 40;
        const SYMENTSIZE: usize = 16;

        let mailbox_symbol = "OPENBMP_PIL_MAILBOX";
        let step_symbol = "openbmp_pil_step_mailbox";
        let mut strtab = Vec::new();
        strtab.push(0);
        let mailbox_name_offset = u32::try_from(strtab.len()).unwrap();
        strtab.extend_from_slice(mailbox_symbol.as_bytes());
        strtab.push(0);
        let step_name_offset = u32::try_from(strtab.len()).unwrap();
        strtab.extend_from_slice(step_symbol.as_bytes());
        strtab.push(0);

        let mut elf = vec![0_u8; 0x180];
        elf[..4].copy_from_slice(b"\x7fELF");
        elf[4] = 1;
        elf[5] = 1;
        elf[6] = 1;
        put_u16_le(&mut elf, 16, 2);
        put_u16_le(&mut elf, 18, 40);
        put_u32_le(&mut elf, 20, 1);
        put_u32_le(&mut elf, 32, SHOFF as u32);
        put_u16_le(&mut elf, 40, 52);
        put_u16_le(&mut elf, 46, SHENTSIZE as u16);
        put_u16_le(&mut elf, 48, 3);

        elf[STRTAB_OFFSET..STRTAB_OFFSET + strtab.len()].copy_from_slice(&strtab);

        let mailbox_symbol_offset = SYMTAB_OFFSET + SYMENTSIZE;
        put_u32_le(&mut elf, mailbox_symbol_offset, mailbox_name_offset);
        put_u32_le(&mut elf, mailbox_symbol_offset + 4, mailbox_address);
        put_u32_le(&mut elf, mailbox_symbol_offset + 8, 512);
        elf[mailbox_symbol_offset + 12] = 0x11;
        put_u16_le(&mut elf, mailbox_symbol_offset + 14, 1);

        let step_symbol_offset = SYMTAB_OFFSET + 2 * SYMENTSIZE;
        put_u32_le(&mut elf, step_symbol_offset, step_name_offset);
        put_u32_le(&mut elf, step_symbol_offset + 4, step_entry_address);
        put_u32_le(&mut elf, step_symbol_offset + 8, 32);
        elf[step_symbol_offset + 12] = 0x12;
        put_u16_le(&mut elf, step_symbol_offset + 14, 1);

        let strtab_section = SHOFF + SHENTSIZE;
        put_u32_le(&mut elf, strtab_section + 4, 3);
        put_u32_le(&mut elf, strtab_section + 16, STRTAB_OFFSET as u32);
        put_u32_le(&mut elf, strtab_section + 20, strtab.len() as u32);
        put_u32_le(&mut elf, strtab_section + 32, 1);

        let symtab_section = SHOFF + 2 * SHENTSIZE;
        put_u32_le(&mut elf, symtab_section + 4, 2);
        put_u32_le(&mut elf, symtab_section + 16, SYMTAB_OFFSET as u32);
        put_u32_le(&mut elf, symtab_section + 20, (3 * SYMENTSIZE) as u32);
        put_u32_le(&mut elf, symtab_section + 24, 1);
        put_u32_le(&mut elf, symtab_section + 32, 4);
        put_u32_le(&mut elf, symtab_section + 36, SYMENTSIZE as u32);

        elf
    }

    fn put_u16_le(bytes: &mut [u8], offset: usize, value: u16) {
        bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
    }

    fn put_u32_le(bytes: &mut [u8], offset: usize, value: u32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    #[test]
    fn gdb_pil_symbols_resolve_from_elf32_symbol_table() {
        let elf = minimal_pil_elf32(0x2000_1000, 0x0800_1234);

        let binding = GdbPilSymbolBinding::resolve_from_elf_bytes(
            &elf,
            "OPENBMP_PIL_MAILBOX",
            "openbmp_pil_step_mailbox",
        )
        .unwrap();
        let plan = GdbPilMailboxPlan::from_elf_symbols(
            &elf,
            "OPENBMP_PIL_MAILBOX",
            PilMailbox::<256, 256>::layout(),
            "openbmp_pil_step_mailbox",
        )
        .unwrap();

        assert_eq!(
            binding,
            GdbPilSymbolBinding {
                mailbox_symbol: "OPENBMP_PIL_MAILBOX".to_owned(),
                mailbox_address: 0x2000_1000,
                step_entry_symbol: "openbmp_pil_step_mailbox".to_owned(),
                step_entry_address: 0x0800_1234,
            }
        );
        assert_eq!(plan.mailbox_address, binding.mailbox_address);
        assert_eq!(plan.step_entry_symbol, binding.step_entry_symbol);
        assert_eq!(plan.sensor_bytes_address().unwrap(), 0x2000_100c);
        assert_eq!(
            plan.step_entry_breakpoint_insert().unwrap().rsp_payload(),
            "Z0,8001234,0"
        );
        assert_eq!(
            plan.step_entry_breakpoint_remove().unwrap().rsp_payload(),
            "z0,8001234,0"
        );
    }

    #[test]
    fn gdb_pil_symbol_resolution_fails_closed_on_missing_symbol() {
        let elf = minimal_pil_elf32(0x2000_1000, 0x0800_1234);

        let err = GdbPilSymbolBinding::resolve_from_elf_bytes(
            &elf,
            "MISSING_OPENBMP_PIL_MAILBOX",
            "openbmp_pil_step_mailbox",
        )
        .expect_err("missing mailbox symbol must fail closed");

        assert!(matches!(
            err,
            GdbPilMailboxPlanError::ElfSymbolMissing { symbol }
                if symbol == "MISSING_OPENBMP_PIL_MAILBOX"
        ));
    }

    #[test]
    fn gdb_pil_symbol_resolution_fails_closed_on_zero_symbol_address() {
        let elf = minimal_pil_elf32(0, 0x0800_1234);

        let err = GdbPilSymbolBinding::resolve_from_elf_bytes(
            &elf,
            "OPENBMP_PIL_MAILBOX",
            "openbmp_pil_step_mailbox",
        )
        .expect_err("zero mailbox address must fail closed");

        assert!(matches!(
            err,
            GdbPilMailboxPlanError::ElfSymbolZeroAddress { symbol }
                if symbol == "OPENBMP_PIL_MAILBOX"
        ));
    }

    #[test]
    fn gdb_mailbox_plan_maps_payload_into_repr_c_offsets() {
        let plan = GdbPilMailboxPlan::for_mailbox::<256, 256>(
            "OPENBMP_PIL_MAILBOX",
            0x2000_1000,
            "openbmp_pil_step_mailbox",
        )
        .unwrap();

        let writes = plan.sensor_payload_writes(&[0xaa, 0xbb]).unwrap();

        assert_eq!(plan.expected_protocol_version(), PROTOCOL_VERSION);
        assert_eq!(
            plan.layout,
            PilMailbox::<256, 256>::layout(),
            "runner must consume the target crate mailbox layout"
        );
        assert_eq!(
            writes,
            vec![
                GdbMemoryWrite::new(0x2000_1003, vec![0]),
                GdbMemoryWrite::new(0x2000_1008, vec![0, 0, 0, 0]),
                GdbMemoryWrite::new(0x2000_100c, vec![0xaa, 0xbb]),
                GdbMemoryWrite::new(0x2000_1004, vec![2, 0, 0, 0]),
                GdbMemoryWrite::new(0x2000_1002, vec![1]),
            ]
        );
        assert_eq!(writes[2].rsp_payload(), "M2000100c,2:aabb");
        assert_eq!(
            plan.command_payload_read(17).unwrap().rsp_payload(),
            "m2000110c,11"
        );
        assert!(matches!(
            plan.step_entry_breakpoint_insert(),
            Err(GdbPilMailboxPlanError::MissingStepEntryAddress { symbol })
                if symbol == "openbmp_pil_step_mailbox"
        ));
    }

    #[test]
    fn gdb_rsp_packets_include_remote_protocol_checksums() {
        let read = GdbMemoryRead::new(0x2000_1004, 4);
        let write = GdbMemoryWrite::new(0x2000_1002, vec![1]);

        assert_eq!(read.rsp_packet(), "$m20001004,4#54");
        assert_eq!(write.rsp_packet(), "$M20001002,1:01#ca");
    }

    #[test]
    fn gdb_rsp_session_loads_sensor_payload_with_memory_writes() {
        let plan = GdbPilMailboxPlan::for_mailbox::<256, 256>(
            "OPENBMP_PIL_MAILBOX",
            0x2000_1000,
            "openbmp_pil_step_mailbox",
        )
        .unwrap();
        let payload = [0xaa, 0xbb];
        let expected_writes = plan.sensor_payload_writes(&payload).unwrap();
        let responses = rsp_responses(
            &expected_writes
                .iter()
                .map(|_| "OK".to_owned())
                .collect::<Vec<_>>(),
        );
        let mut session = GdbRspSession::new(MockRsp::new(responses));

        session
            .load_mailbox_sensor_payload(&plan, &payload)
            .unwrap();

        let stream = session.into_inner();
        let mut expected_bytes = Vec::new();
        for write in expected_writes {
            expected_bytes.extend_from_slice(write.rsp_packet().as_bytes());
            expected_bytes.push(b'+');
        }
        assert_eq!(stream.written, expected_bytes);
    }

    #[test]
    fn gdb_rsp_session_reads_mailbox_status_and_command_payload() {
        let plan = GdbPilMailboxPlan::for_mailbox::<256, 256>(
            "OPENBMP_PIL_MAILBOX",
            0x2000_1000,
            "openbmp_pil_step_mailbox",
        )
        .unwrap();
        let sensor = sensor(42);
        let command = command_for(&sensor);
        let command_payload = encode(&command).unwrap();
        let command_len = u32::try_from(command_payload.len()).unwrap();
        let responses = rsp_responses(&[
            "02".to_owned(),
            hex_encode(&command_len.to_le_bytes()),
            hex_encode(&command_payload),
        ]);
        let mut session = GdbRspSession::new(MockRsp::new(responses));

        let status = session.read_mailbox_status(&plan).unwrap();
        let received = session.read_mailbox_command_packet(&plan).unwrap();

        assert_eq!(status, PilMailboxStatus::CommandReady);
        assert_eq!(received, command);

        let stream = session.into_inner();
        let mut expected_bytes = Vec::new();
        for read in [
            plan.status_read().unwrap(),
            plan.command_len_read().unwrap(),
            plan.command_payload_read(command_len).unwrap(),
        ] {
            expected_bytes.extend_from_slice(read.rsp_packet().as_bytes());
            expected_bytes.push(b'+');
        }
        assert_eq!(stream.written, expected_bytes);
    }

    #[test]
    fn gdb_rsp_session_drives_breakpoint_and_continue_packets() {
        let plan = GdbPilMailboxPlan::for_mailbox::<256, 256>(
            "OPENBMP_PIL_MAILBOX",
            0x2000_1000,
            "openbmp_pil_step_mailbox",
        )
        .unwrap()
        .with_step_entry_address(0x0800_1234)
        .unwrap();
        let responses = rsp_responses(&["OK".to_owned(), "S05".to_owned(), "OK".to_owned()]);
        let mut session = GdbRspSession::new(MockRsp::new(responses));

        session
            .insert_breakpoint(plan.step_entry_breakpoint_insert().unwrap())
            .unwrap();
        let stop = session.continue_target().unwrap();
        session
            .remove_breakpoint(plan.step_entry_breakpoint_remove().unwrap())
            .unwrap();

        assert_eq!(stop, "S05");
        let stream = session.into_inner();
        let mut expected_bytes = Vec::new();
        for payload in ["Z0,8001234,0", "c", "z0,8001234,0"] {
            expected_bytes.extend_from_slice(gdb_rsp_packet(payload).as_bytes());
            expected_bytes.push(b'+');
        }
        assert_eq!(stream.written, expected_bytes);
    }

    #[test]
    fn gdb_rsp_session_fails_closed_on_bad_checksum_and_target_error() {
        let mut bad_checksum = GdbRspSession::new(MockRsp::new(b"+$00#ff".to_vec()));
        let err = bad_checksum
            .read_memory(GdbMemoryRead::new(0x2000_1002, 1))
            .expect_err("checksum mismatch should fail closed");
        assert!(matches!(
            err,
            GdbRspSessionError::ChecksumMismatch {
                expected: 0x60,
                got: 0xff
            }
        ));

        let mut target_error = GdbRspSession::new(MockRsp::new(rsp_response("E22")));
        let err = target_error
            .write_memory(&GdbMemoryWrite::new(0x2000_1002, vec![1]))
            .expect_err("target E response should fail closed");
        assert!(matches!(
            err,
            GdbRspSessionError::TargetError { response, .. } if response == "E22"
        ));
    }

    #[test]
    fn gdb_mailbox_plan_rejects_oversized_payloads_and_bad_symbols() {
        let plan = GdbPilMailboxPlan::for_mailbox::<1, 16>(
            "OPENBMP_PIL_MAILBOX",
            0x2000_1000,
            "openbmp_pil_step_mailbox",
        )
        .unwrap();

        let err = plan
            .sensor_payload_writes(&[0xaa, 0xbb])
            .expect_err("payload exceeds configured mailbox sensor storage");
        assert!(matches!(
            err,
            GdbPilMailboxPlanError::SensorPayloadTooLarge {
                capacity: 1,
                got: 2
            }
        ));

        let err = GdbPilMailboxPlan::for_mailbox::<16, 16>(
            "OPENBMP PIL MAILBOX",
            0x2000_1000,
            "openbmp_pil_step_mailbox",
        )
        .expect_err("spaces are not valid unquoted GDB symbol names");
        assert!(matches!(
            err,
            GdbPilMailboxPlanError::UnsupportedCharacter {
                field: "symbol_name",
                character: ' '
            }
        ));
    }

    #[test]
    fn gdb_mailbox_decodes_target_reads_and_command_payload() {
        let sensor = sensor(42);
        let command = command_for(&sensor);
        let command_payload = encode(&command).unwrap();
        let command_len = u32::try_from(command_payload.len()).unwrap();
        let command_len_hex = hex_encode(&command_len.to_le_bytes());
        let command_payload_hex = hex_encode(&command_payload);

        let protocol_bytes =
            decode_gdb_rsp_hex_bytes(&hex_encode(&PROTOCOL_VERSION.to_le_bytes())).unwrap();
        let status_bytes = decode_gdb_rsp_hex_bytes("02").unwrap();
        let error_bytes = decode_gdb_rsp_hex_bytes("00").unwrap();
        let len_bytes = decode_gdb_rsp_hex_bytes(&command_len_hex).unwrap();
        let payload_bytes = decode_gdb_rsp_hex_bytes(&command_payload_hex).unwrap();

        assert_eq!(
            decode_gdb_mailbox_protocol_version(&protocol_bytes).unwrap(),
            PROTOCOL_VERSION
        );
        assert_eq!(
            decode_gdb_mailbox_status(&status_bytes).unwrap(),
            PilMailboxStatus::CommandReady
        );
        assert_eq!(
            decode_gdb_mailbox_error(&error_bytes).unwrap(),
            PilMailboxError::None
        );
        assert_eq!(
            decode_gdb_mailbox_command_len(&len_bytes).unwrap(),
            command_len
        );
        assert_eq!(
            decode_gdb_mailbox_command_payload(&payload_bytes).unwrap(),
            command
        );
    }

    #[test]
    fn gdb_rsp_hex_decoder_fails_closed_on_malformed_payloads() {
        assert!(matches!(
            decode_gdb_rsp_hex_bytes("0"),
            Err(GdbPilMailboxPlanError::OddHexPayloadLength { len: 1 })
        ));
        assert!(matches!(
            decode_gdb_rsp_hex_bytes("0x"),
            Err(GdbPilMailboxPlanError::InvalidHexDigit {
                index: 1,
                digit: 'x'
            })
        ));
        assert!(matches!(
            decode_gdb_mailbox_status(&[9]),
            Err(GdbPilMailboxPlanError::InvalidStatusCode { code: 9 })
        ));
    }

    #[test]
    fn renode_pil_plan_renders_socket_uart_resc() {
        let plan = RenodePilPlan::stm32f4_discovery_socket(
            "target/thumbv7em-none-eabihf/debug/openbmp-pil-firmware",
        );

        let rendered = plan.render_resc().unwrap();

        assert_eq!(
            rendered,
            concat!(
                ":name: OpenBMP PIL UART coupling\n",
                ":description: Loads an OpenBMP target ELF and exposes the bridge UART as a host TCP socket\n",
                "\n",
                "mach create \"openbmp-pil\"\n",
                "machine LoadPlatformDescription @platforms/boards/stm32f4_discovery-kit.repl\n",
                "sysbus LoadELF @target/thumbv7em-none-eabihf/debug/openbmp-pil-firmware\n",
                "emulation CreateServerSocketTerminal 3456 \"openbmp_pil_uart\" false\n",
                "connector Connect sysbus.usart2 openbmp_pil_uart\n",
                "emulation SetGlobalQuantum \"0.001000\"\n",
                "machine StartGdbServer 3333\n",
                "echo \"OpenBMP PIL UART socket: 127.0.0.1:3456\"\n",
                "echo \"Load complete; start when the host bridge is attached.\"\n",
            )
        );
    }

    #[test]
    fn renode_pil_plan_omits_gdb_when_not_requested() {
        let mut plan = RenodePilPlan::stm32f4_discovery_socket("firmware/openbmp-pil.elf");
        plan.gdb_port = None;

        let rendered = plan.render_resc().unwrap();

        assert!(!rendered.contains("StartGdbServer"));
        assert!(rendered.contains("sysbus LoadELF @firmware/openbmp-pil.elf"));
    }

    #[test]
    fn renode_pil_plan_rejects_unquoted_paths() {
        let plan = RenodePilPlan::stm32f4_discovery_socket("target/with space/firmware.elf");

        let err = plan
            .render_resc()
            .expect_err("space requires explicit caller handling");

        assert_eq!(
            err,
            RenodePilPlanError::UnsupportedCharacter {
                field: "firmware_elf",
                character: ' '
            }
        );
    }

    #[test]
    fn renode_pil_plan_rejects_zero_quantum_and_ports() {
        let mut plan = RenodePilPlan::stm32f4_discovery_socket("firmware/openbmp-pil.elf");
        plan.quantum_us = 0;
        assert_eq!(plan.validate(), Err(RenodePilPlanError::ZeroQuantum));

        plan.quantum_us = 1_000;
        plan.uart_host_port = 0;
        assert_eq!(
            plan.validate(),
            Err(RenodePilPlanError::ZeroPort {
                field: "uart_host_port"
            })
        );
    }

    #[test]
    fn pil_frame_stream_sends_sensor_and_validates_matching_command() {
        let sensor = sensor(42);
        let command = command_for(&sensor);
        let stream = MockDuplex::new(framed_command(&command)).with_max_read_chunk(2);
        let mut client = PilFrameStream::new(stream);

        let received = client.step(&sensor).unwrap();
        let stream = client.into_inner();
        let (sensor_payload, consumed) =
            openbmp_bridge::deframe(&stream.written).expect("written sensor frame");
        let written_sensor: SensorPacket = decode(sensor_payload).unwrap();

        assert_eq!(received, command);
        assert_eq!(consumed, stream.written.len());
        assert_eq!(written_sensor, sensor);
    }

    #[test]
    fn pil_frame_stream_rejects_mismatched_command_step() {
        let sensor = sensor(42);
        let mut command = command_for(&sensor);
        command.step = 43;
        let stream = MockDuplex::new(framed_command(&command));
        let mut client = PilFrameStream::new(stream);

        let err = client
            .step(&sensor)
            .expect_err("mismatched command step must fail closed");

        assert!(matches!(
            err,
            PilClientError::Bridge(BridgeError::StepMismatch {
                expected: 42,
                got: 43
            })
        ));
    }

    #[test]
    fn pil_frame_stream_rejects_oversized_inbound_payload() {
        let framed = frame(&[0_u8; 5]);
        let stream = MockDuplex::new(framed);
        let mut client = PilFrameStream::with_max_payload_len(stream, 4);

        let err = client
            .recv_command()
            .expect_err("payload length exceeds configured PIL limit");

        assert!(matches!(
            err,
            PilClientError::Bridge(BridgeError::PayloadTooLarge { max: 4, got: 5 })
        ));
    }
}
