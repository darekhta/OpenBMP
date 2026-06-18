//! Deterministic bridge-frame fault transforms.
//!
//! These helpers mutate abstract [`SensorPacket`] and
//! [`ActuatorCommandPacket`] fields before or after transport. They are
//! deliberately signal-level and do not model any electrical bus or
//! device-specific failure mode.

use alloc::string::String;
use alloc::vec::Vec;

use openbmp_core::DeterministicRng;
use thiserror::Error;

use crate::packet::{ActuatorCommandPacket, BridgeMessage, SensorPacket};

const BRIDGE_FAULT_NOISE_DOMAIN_TAG: [u8; 4] = *b"BRGF";

/// Packet direction targeted by a bridge packet-level fault.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum BridgePacketDirection {
    /// Simulator-to-controller sensor frame.
    Sensor,
    /// Controller-to-simulator command frame.
    Command,
}

/// Cartesian vector axis.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum VectorAxis {
    /// X axis.
    X,
    /// Y axis.
    Y,
    /// Z axis.
    Z,
}

impl VectorAxis {
    const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
    }
}

/// Quaternion component in `[x, y, z, w]` order.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum QuaternionAxis {
    /// X vector component.
    X,
    /// Y vector component.
    Y,
    /// Z vector component.
    Z,
    /// W scalar component.
    W,
}

impl QuaternionAxis {
    const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
            Self::W => 3,
        }
    }
}

/// Scalar bridge signal that can be transformed by a fault rule.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum BridgeScalarSignal {
    /// IMU accelerometer axis in body frame, m/s².
    ImuAccelBodyMps2(VectorAxis),
    /// IMU gyro axis in body frame, rad/s.
    ImuGyroBodyRadS(VectorAxis),
    /// High-rate IMU delta-theta increment component, rad.
    ImuIncrementDeltaThetaRad {
        /// Zero-based increment index inside the bridge frame.
        sample_index: u32,
        /// Vector axis.
        axis: VectorAxis,
    },
    /// High-rate IMU delta-v increment component, m/s.
    ImuIncrementDeltaVMs {
        /// Zero-based increment index inside the bridge frame.
        sample_index: u32,
        /// Vector axis.
        axis: VectorAxis,
    },
    /// High-rate IMU increment duration, s.
    ImuIncrementDtS {
        /// Zero-based increment index inside the bridge frame.
        sample_index: u32,
    },
    /// Barometric altitude, m.
    BaroAltitudeM,
    /// Barometric pressure, Pa.
    BaroPressurePa,
    /// Barometer bias, Pa.
    BaroBiasPa,
    /// GNSS ECI position axis, m.
    GnssPositionEciM(VectorAxis),
    /// GNSS ECI velocity axis, m/s.
    GnssVelocityEciMS(VectorAxis),
    /// GNSS ECI position-bias axis, m.
    GnssPositionBiasEciM(VectorAxis),
    /// Magnetometer body-frame axis, tesla.
    MagBodyTesla(VectorAxis),
    /// Magnetometer body-frame axis, nT.
    MagBodyNt(VectorAxis),
    /// Magnetometer hard-iron body-frame axis, nT.
    MagHardIronBodyNt(VectorAxis),
    /// Star-tracker ECI-to-body quaternion component in `[x, y, z, w]`.
    StarTrackerAttitudeEciToBody(QuaternionAxis),
    /// Normalized effector command by scenario-assigned effector id.
    EffectorCommand {
        /// Scenario-assigned effector id.
        effector_id: u32,
    },
    /// Engine throttle command by scenario-assigned engine id.
    EngineThrottle {
        /// Scenario-assigned engine id.
        engine_id: u32,
    },
    /// Engine pitch-gimbal command by scenario-assigned engine id.
    EngineGimbalPitch {
        /// Scenario-assigned engine id.
        engine_id: u32,
    },
    /// Engine yaw-gimbal command by scenario-assigned engine id.
    EngineGimbalYaw {
        /// Scenario-assigned engine id.
        engine_id: u32,
    },
}

/// Scalar transform applied to a bridge signal.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BridgeScalarTransform {
    /// Add a fixed offset to the signal.
    AdditiveBias {
        /// Offset added to the signal.
        offset: f64,
    },
    /// Multiply the signal by a fixed factor.
    Scale {
        /// Multiplicative factor.
        factor: f64,
    },
    /// Replace the signal with a fixed value.
    Stuck {
        /// Replacement value.
        value: f64,
    },
    /// Clamp the signal to an inclusive range.
    Saturate {
        /// Inclusive lower bound.
        min: f64,
        /// Inclusive upper bound.
        max: f64,
    },
    /// Quantize the signal to the nearest multiple of `quantum`.
    Quantize {
        /// Positive finite quantization step.
        quantum: f64,
    },
    /// Add a linear time ramp relative to a reference simulation time.
    Drift {
        /// Drift rate in signal units per second.
        rate_per_s: f64,
        /// Reference simulation time where the drift contribution is zero.
        reference_time_s: f64,
    },
    /// Add bounded deterministic noise from a rule-local bridge-fault RNG stream.
    NoiseBurst {
        /// Positive maximum absolute noise contribution.
        amplitude: f64,
        /// Explicit deterministic stream seed for this noise burst.
        seed: u64,
    },
    /// Multiply the signal by `-1`.
    ReverseSign,
}

/// Packet-level bridge fault transform.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BridgePacketTransform {
    /// Drop the packet at the abstract transport boundary.
    Drop,
    /// Duplicate the packet at the abstract transport boundary.
    Duplicate,
    /// Delay the packet by a positive number of bridge steps.
    Delay {
        /// Positive bridge-step delay.
        steps: u64,
    },
    /// Corrupt the packet by applying a deterministic bit-flip mask.
    BitFlip {
        /// Non-zero bit mask applied by the downstream framed-byte fault bench.
        mask: u8,
    },
    /// Offset the packet step index before lockstep validation.
    StepOffset {
        /// Signed step offset. Zero is rejected by validation.
        offset: i64,
    },
    /// Offset the packet simulation timestamp before lockstep validation.
    TimeOffset {
        /// Signed timestamp offset in seconds. Must be finite and non-zero.
        offset_s: f64,
    },
}

/// Packet disposition after packet-level fault rules are applied.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum BridgePacketDisposition {
    /// Packet remains available for normal lockstep processing.
    Deliver,
    /// Packet was dropped by a configured fault.
    Drop,
    /// Packet was duplicated by a configured fault.
    Duplicate,
    /// Packet was delayed by the configured number of bridge steps.
    Delay {
        /// Positive bridge-step delay.
        steps: u64,
    },
    /// Packet was corrupted by a deterministic bit-flip mask.
    BitFlip {
        /// Non-zero bit mask.
        mask: u8,
    },
}

/// One active packet-level bridge fault rule.
#[derive(Clone, Debug, PartialEq)]
pub struct BridgePacketFaultRule {
    /// Stable rule id used in evidence output.
    pub id: String,
    /// First bridge step where this rule is active.
    pub start_step: u64,
    /// Last active bridge step, inclusive. `None` means no upper bound.
    pub end_step: Option<u64>,
    /// Packet direction targeted by this rule.
    pub direction: BridgePacketDirection,
    /// Packet transform applied by this rule.
    pub transform: BridgePacketTransform,
}

impl BridgePacketFaultRule {
    /// Construct a packet-level fault rule.
    #[must_use]
    pub fn new(
        id: impl Into<String>,
        start_step: u64,
        end_step: Option<u64>,
        direction: BridgePacketDirection,
        transform: BridgePacketTransform,
    ) -> Self {
        Self {
            id: id.into(),
            start_step,
            end_step,
            direction,
            transform,
        }
    }

    /// Return whether the rule is active for `step`.
    #[must_use]
    pub fn is_active(&self, step: u64) -> bool {
        step >= self.start_step && self.end_step.is_none_or(|end_step| step <= end_step)
    }
}

/// One active bridge fault rule.
#[derive(Clone, Debug, PartialEq)]
pub struct BridgeFaultRule {
    /// Stable rule id used in evidence output.
    pub id: String,
    /// First bridge step where this rule is active.
    pub start_step: u64,
    /// Last active bridge step, inclusive. `None` means no upper bound.
    pub end_step: Option<u64>,
    /// Signal transformed by this rule.
    pub signal: BridgeScalarSignal,
    /// Scalar transform applied to the signal.
    pub transform: BridgeScalarTransform,
}

impl BridgeFaultRule {
    /// Construct a fault rule.
    #[must_use]
    pub fn new(
        id: impl Into<String>,
        start_step: u64,
        end_step: Option<u64>,
        signal: BridgeScalarSignal,
        transform: BridgeScalarTransform,
    ) -> Self {
        Self {
            id: id.into(),
            start_step,
            end_step,
            signal,
            transform,
        }
    }

    /// Return whether the rule is active for `step`.
    #[must_use]
    pub fn is_active(&self, step: u64) -> bool {
        step >= self.start_step && self.end_step.is_none_or(|end_step| step <= end_step)
    }
}

/// Ordered bridge fault transform set.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BridgeFaultTransformSet {
    rules: Vec<BridgeFaultRule>,
    packet_rules: Vec<BridgePacketFaultRule>,
}

impl BridgeFaultTransformSet {
    /// Construct a transform set from ordered rules.
    #[must_use]
    pub fn new(rules: Vec<BridgeFaultRule>) -> Self {
        Self {
            rules,
            packet_rules: Vec::new(),
        }
    }

    /// Construct a transform set from scalar and packet-level rules.
    #[must_use]
    pub fn new_with_packet_rules(
        rules: Vec<BridgeFaultRule>,
        packet_rules: Vec<BridgePacketFaultRule>,
    ) -> Self {
        Self {
            rules,
            packet_rules,
        }
    }

    /// Borrow the ordered rules.
    #[must_use]
    pub fn rules(&self) -> &[BridgeFaultRule] {
        &self.rules
    }

    /// Borrow the ordered packet-level rules.
    #[must_use]
    pub fn packet_rules(&self) -> &[BridgePacketFaultRule] {
        &self.packet_rules
    }

    /// Apply active rules to a bridge message.
    ///
    /// Returns the ids of active rules that found their target signal
    /// and were applied. Rules targeting absent optional sensors or
    /// missing command ids are skipped without error.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeFaultError`] when a rule's transform is invalid.
    pub fn apply_to_message(
        &self,
        message: &mut BridgeMessage,
    ) -> Result<Vec<String>, BridgeFaultError> {
        match message {
            BridgeMessage::Sensor(packet) => self.apply_to_sensor(packet),
            BridgeMessage::Command(packet) => self.apply_to_command(packet),
            BridgeMessage::Hello(_) | BridgeMessage::Ack(_) | BridgeMessage::Fault(_) => {
                Ok(Vec::new())
            }
        }
    }

    /// Apply active sensor-side rules to a sensor packet.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeFaultError`] when a rule's transform is invalid.
    pub fn apply_to_sensor(
        &self,
        packet: &mut SensorPacket,
    ) -> Result<Vec<String>, BridgeFaultError> {
        let mut applied = Vec::new();
        for rule in &self.rules {
            if !rule.is_active(packet.step) {
                continue;
            }
            let context = ScalarTransformContext {
                step: packet.step,
                sim_time_s: packet.sim_time_s,
                rule_id: &rule.id,
                signal: rule.signal,
            };
            if apply_sensor_signal(packet, rule.signal, rule.transform, context)? {
                applied.push(rule.id.clone());
            }
        }
        Ok(applied)
    }

    /// Apply active sensor-side scalar and packet-level rules.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeFaultError`] when a rule's transform is invalid.
    pub fn apply_sensor_frame(
        &self,
        packet: &mut SensorPacket,
    ) -> Result<BridgeFaultApplication, BridgeFaultError> {
        let mut applied_rule_ids = self.apply_to_sensor(packet)?;
        let active_step = packet.step;
        let disposition = self.apply_packet_rules(
            active_step,
            &mut packet.step,
            &mut packet.sim_time_s,
            BridgePacketDirection::Sensor,
            &mut applied_rule_ids,
        )?;
        Ok(BridgeFaultApplication {
            applied_rule_ids,
            disposition,
        })
    }

    /// Apply active actuator-side rules to a command packet.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeFaultError`] when a rule's transform is invalid.
    pub fn apply_to_command(
        &self,
        packet: &mut ActuatorCommandPacket,
    ) -> Result<Vec<String>, BridgeFaultError> {
        let mut applied = Vec::new();
        for rule in &self.rules {
            if !rule.is_active(packet.step) {
                continue;
            }
            let context = ScalarTransformContext {
                step: packet.step,
                sim_time_s: packet.sim_time_s,
                rule_id: &rule.id,
                signal: rule.signal,
            };
            if apply_command_signal(packet, rule.signal, rule.transform, context)? {
                applied.push(rule.id.clone());
            }
        }
        Ok(applied)
    }

    /// Apply active command-side scalar and packet-level rules.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeFaultError`] when a rule's transform is invalid.
    pub fn apply_command_frame(
        &self,
        packet: &mut ActuatorCommandPacket,
    ) -> Result<BridgeFaultApplication, BridgeFaultError> {
        let mut applied_rule_ids = self.apply_to_command(packet)?;
        let active_step = packet.step;
        let disposition = self.apply_packet_rules(
            active_step,
            &mut packet.step,
            &mut packet.sim_time_s,
            BridgePacketDirection::Command,
            &mut applied_rule_ids,
        )?;
        Ok(BridgeFaultApplication {
            applied_rule_ids,
            disposition,
        })
    }

    fn apply_packet_rules(
        &self,
        active_step: u64,
        packet_step: &mut u64,
        packet_time_s: &mut f64,
        direction: BridgePacketDirection,
        applied_rule_ids: &mut Vec<String>,
    ) -> Result<BridgePacketDisposition, BridgeFaultError> {
        let mut disposition = BridgePacketDisposition::Deliver;
        for rule in &self.packet_rules {
            if rule.direction != direction || !rule.is_active(active_step) {
                continue;
            }
            match rule.transform {
                BridgePacketTransform::Drop => {
                    applied_rule_ids.push(rule.id.clone());
                    disposition = BridgePacketDisposition::Drop;
                }
                BridgePacketTransform::Duplicate => {
                    applied_rule_ids.push(rule.id.clone());
                    disposition = BridgePacketDisposition::Duplicate;
                }
                BridgePacketTransform::Delay { steps } => {
                    if steps == 0 {
                        return Err(BridgeFaultError::InvalidPacketDelay { steps });
                    }
                    applied_rule_ids.push(rule.id.clone());
                    disposition = BridgePacketDisposition::Delay { steps };
                }
                BridgePacketTransform::BitFlip { mask } => {
                    if mask == 0 {
                        return Err(BridgeFaultError::InvalidPacketBitFlipMask { mask });
                    }
                    applied_rule_ids.push(rule.id.clone());
                    disposition = BridgePacketDisposition::BitFlip { mask };
                }
                BridgePacketTransform::StepOffset { offset } => {
                    if offset == 0 {
                        return Err(BridgeFaultError::InvalidPacketStepOffset { offset });
                    }
                    *packet_step = offset_step_saturating(*packet_step, offset);
                    applied_rule_ids.push(rule.id.clone());
                }
                BridgePacketTransform::TimeOffset { offset_s } => {
                    if !offset_s.is_finite() || offset_s == 0.0 {
                        return Err(BridgeFaultError::InvalidPacketTimeOffset { offset_s });
                    }
                    *packet_time_s += offset_s;
                    applied_rule_ids.push(rule.id.clone());
                }
            }
        }
        Ok(disposition)
    }
}

/// Result of applying bridge fault rules to one packet frame.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BridgeFaultApplication {
    /// Rule ids that were applied in declaration order.
    pub applied_rule_ids: Vec<String>,
    /// Final packet disposition.
    pub disposition: BridgePacketDisposition,
}

/// Fault-transform validation error.
#[derive(Debug, Error)]
pub enum BridgeFaultError {
    /// Saturation bounds were not finite or not ordered.
    #[error("invalid saturation range: min {min}, max {max}")]
    InvalidSaturationRange {
        /// Lower bound.
        min: f64,
        /// Upper bound.
        max: f64,
    },
    /// Quantization step was not positive and finite.
    #[error("invalid quantization step: quantum {quantum}")]
    InvalidQuantizationStep {
        /// Quantization step.
        quantum: f64,
    },
    /// Drift transform parameters or packet time were not finite.
    #[error(
        "invalid drift transform: rate_per_s {rate_per_s}, reference_time_s {reference_time_s}, sim_time_s {sim_time_s}"
    )]
    InvalidDriftTransform {
        /// Drift rate in signal units per second.
        rate_per_s: f64,
        /// Reference simulation time where drift contribution is zero.
        reference_time_s: f64,
        /// Packet simulation time.
        sim_time_s: f64,
    },
    /// Noise-burst amplitude was not positive and finite.
    #[error("invalid noise-burst amplitude: amplitude {amplitude}")]
    InvalidNoiseBurstAmplitude {
        /// Maximum absolute noise contribution.
        amplitude: f64,
    },
    /// Packet delay was not positive.
    #[error("invalid packet delay: steps {steps}")]
    InvalidPacketDelay {
        /// Packet-delay step count.
        steps: u64,
    },
    /// Bit-flip mask was zero.
    #[error("invalid packet bit-flip mask: mask {mask}")]
    InvalidPacketBitFlipMask {
        /// Bit-flip mask.
        mask: u8,
    },
    /// Packet step offset was zero.
    #[error("invalid packet step offset: offset {offset}")]
    InvalidPacketStepOffset {
        /// Signed packet step offset.
        offset: i64,
    },
    /// Packet time offset was zero, NaN, or infinite.
    #[error("invalid packet time offset: offset_s {offset_s}")]
    InvalidPacketTimeOffset {
        /// Packet time offset in seconds.
        offset_s: f64,
    },
}

fn offset_step_saturating(step: u64, offset: i64) -> u64 {
    if offset >= 0 {
        step.saturating_add(offset as u64)
    } else {
        step.saturating_sub(offset.unsigned_abs())
    }
}

#[derive(Copy, Clone, Debug)]
struct ScalarTransformContext<'a> {
    step: u64,
    sim_time_s: f64,
    rule_id: &'a str,
    signal: BridgeScalarSignal,
}

fn apply_sensor_signal(
    packet: &mut SensorPacket,
    signal: BridgeScalarSignal,
    transform: BridgeScalarTransform,
    context: ScalarTransformContext<'_>,
) -> Result<bool, BridgeFaultError> {
    let target = match signal {
        BridgeScalarSignal::ImuAccelBodyMps2(axis) => {
            Some(&mut packet.imu_accel_body_m_s2[axis.index()])
        }
        BridgeScalarSignal::ImuGyroBodyRadS(axis) => {
            Some(&mut packet.imu_gyro_body_rad_s[axis.index()])
        }
        BridgeScalarSignal::ImuIncrementDeltaThetaRad { sample_index, axis } => packet
            .imu_increments
            .get_mut(sample_index as usize)
            .map(|increment| &mut increment.delta_theta_rad[axis.index()]),
        BridgeScalarSignal::ImuIncrementDeltaVMs { sample_index, axis } => packet
            .imu_increments
            .get_mut(sample_index as usize)
            .map(|increment| &mut increment.delta_v_m_s[axis.index()]),
        BridgeScalarSignal::ImuIncrementDtS { sample_index } => packet
            .imu_increments
            .get_mut(sample_index as usize)
            .map(|increment| &mut increment.dt_s),
        BridgeScalarSignal::BaroAltitudeM => packet.baro_altitude_m.as_mut(),
        BridgeScalarSignal::BaroPressurePa => packet.baro_pressure_pa.as_mut(),
        BridgeScalarSignal::BaroBiasPa => packet.baro_bias_pa.as_mut(),
        BridgeScalarSignal::GnssPositionEciM(axis) => {
            optional_vector_axis(packet.gnss_position_eci_m.as_mut(), axis)
        }
        BridgeScalarSignal::GnssVelocityEciMS(axis) => {
            optional_vector_axis(packet.gnss_velocity_eci_m_s.as_mut(), axis)
        }
        BridgeScalarSignal::GnssPositionBiasEciM(axis) => {
            optional_vector_axis(packet.gnss_position_bias_eci_m.as_mut(), axis)
        }
        BridgeScalarSignal::MagBodyTesla(axis) => {
            optional_vector_axis(packet.mag_body_tesla.as_mut(), axis)
        }
        BridgeScalarSignal::MagBodyNt(axis) => {
            optional_vector_axis(packet.mag_body_nt.as_mut(), axis)
        }
        BridgeScalarSignal::MagHardIronBodyNt(axis) => {
            optional_vector_axis(packet.mag_hard_iron_body_nt.as_mut(), axis)
        }
        BridgeScalarSignal::StarTrackerAttitudeEciToBody(axis) => packet
            .star_tracker_attitude_eci_to_body_xyzw
            .as_mut()
            .map(|value| &mut value[axis.index()]),
        BridgeScalarSignal::EffectorCommand { .. }
        | BridgeScalarSignal::EngineThrottle { .. }
        | BridgeScalarSignal::EngineGimbalPitch { .. }
        | BridgeScalarSignal::EngineGimbalYaw { .. } => None,
    };

    let Some(value) = target else {
        return Ok(false);
    };
    apply_scalar_transform(value, transform, context)?;
    Ok(true)
}

fn apply_command_signal(
    packet: &mut ActuatorCommandPacket,
    signal: BridgeScalarSignal,
    transform: BridgeScalarTransform,
    context: ScalarTransformContext<'_>,
) -> Result<bool, BridgeFaultError> {
    match signal {
        BridgeScalarSignal::EffectorCommand { effector_id } => {
            if let Some((_, command)) = packet
                .effector_commands
                .iter_mut()
                .find(|(id, _)| *id == effector_id)
            {
                apply_scalar_transform(command, transform, context)?;
                Ok(true)
            } else {
                Ok(false)
            }
        }
        BridgeScalarSignal::EngineThrottle { engine_id } => {
            let mut applied = false;
            for (_, throttle) in packet
                .engine_throttles
                .iter_mut()
                .filter(|(id, _)| *id == engine_id)
            {
                apply_scalar_transform(throttle, transform, context)?;
                applied = true;
            }
            for command in packet
                .engine_commands
                .iter_mut()
                .filter(|command| command.engine_id == engine_id)
            {
                apply_scalar_transform(&mut command.throttle_unit, transform, context)?;
                applied = true;
            }
            Ok(applied)
        }
        BridgeScalarSignal::EngineGimbalPitch { engine_id } => {
            apply_engine_command_field(packet, engine_id, transform, context, |command| {
                &mut command.gimbal_pitch_rad
            })
        }
        BridgeScalarSignal::EngineGimbalYaw { engine_id } => {
            apply_engine_command_field(packet, engine_id, transform, context, |command| {
                &mut command.gimbal_yaw_rad
            })
        }
        BridgeScalarSignal::ImuAccelBodyMps2(_)
        | BridgeScalarSignal::ImuGyroBodyRadS(_)
        | BridgeScalarSignal::ImuIncrementDeltaThetaRad { .. }
        | BridgeScalarSignal::ImuIncrementDeltaVMs { .. }
        | BridgeScalarSignal::ImuIncrementDtS { .. }
        | BridgeScalarSignal::BaroAltitudeM
        | BridgeScalarSignal::BaroPressurePa
        | BridgeScalarSignal::BaroBiasPa
        | BridgeScalarSignal::GnssPositionEciM(_)
        | BridgeScalarSignal::GnssVelocityEciMS(_)
        | BridgeScalarSignal::GnssPositionBiasEciM(_)
        | BridgeScalarSignal::MagBodyTesla(_)
        | BridgeScalarSignal::MagBodyNt(_)
        | BridgeScalarSignal::MagHardIronBodyNt(_)
        | BridgeScalarSignal::StarTrackerAttitudeEciToBody(_) => Ok(false),
    }
}

fn apply_engine_command_field(
    packet: &mut ActuatorCommandPacket,
    engine_id: u32,
    transform: BridgeScalarTransform,
    context: ScalarTransformContext<'_>,
    field: impl Fn(&mut crate::packet::EngineCommandPacket) -> &mut f64,
) -> Result<bool, BridgeFaultError> {
    let mut applied = false;
    for command in packet
        .engine_commands
        .iter_mut()
        .filter(|command| command.engine_id == engine_id)
    {
        apply_scalar_transform(field(command), transform, context)?;
        applied = true;
    }
    Ok(applied)
}

fn optional_vector_axis(value: Option<&mut [f64; 3]>, axis: VectorAxis) -> Option<&mut f64> {
    value.map(|value| &mut value[axis.index()])
}

fn apply_scalar_transform(
    value: &mut f64,
    transform: BridgeScalarTransform,
    context: ScalarTransformContext<'_>,
) -> Result<(), BridgeFaultError> {
    match transform {
        BridgeScalarTransform::AdditiveBias { offset } => *value += offset,
        BridgeScalarTransform::Scale { factor } => *value *= factor,
        BridgeScalarTransform::Stuck { value: stuck } => *value = stuck,
        BridgeScalarTransform::ReverseSign => *value = -*value,
        BridgeScalarTransform::Saturate { min, max } => {
            if !min.is_finite() || !max.is_finite() || min > max {
                return Err(BridgeFaultError::InvalidSaturationRange { min, max });
            }
            *value = value.clamp(min, max);
        }
        BridgeScalarTransform::Quantize { quantum } => {
            if !quantum.is_finite() || quantum <= 0.0 {
                return Err(BridgeFaultError::InvalidQuantizationStep { quantum });
            }
            *value = (*value / quantum).round() * quantum;
        }
        BridgeScalarTransform::Drift {
            rate_per_s,
            reference_time_s,
        } => {
            if !rate_per_s.is_finite()
                || !reference_time_s.is_finite()
                || !context.sim_time_s.is_finite()
            {
                return Err(BridgeFaultError::InvalidDriftTransform {
                    rate_per_s,
                    reference_time_s,
                    sim_time_s: context.sim_time_s,
                });
            }
            *value += rate_per_s * (context.sim_time_s - reference_time_s);
        }
        BridgeScalarTransform::NoiseBurst { amplitude, seed } => {
            if !amplitude.is_finite() || amplitude <= 0.0 {
                return Err(BridgeFaultError::InvalidNoiseBurstAmplitude { amplitude });
            }
            *value += amplitude * deterministic_noise_unit(seed, context);
        }
    }
    Ok(())
}

fn deterministic_noise_unit(seed: u64, context: ScalarTransformContext<'_>) -> f64 {
    let mut bytes = [0_u8; 32];
    bytes[0..8].copy_from_slice(&seed.to_le_bytes());
    bytes[8..16].copy_from_slice(&context.step.to_le_bytes());
    bytes[16..24].copy_from_slice(&stable_signal_id(context.signal).to_le_bytes());
    bytes[24..28].copy_from_slice(&stable_rule_hash(context.rule_id).to_le_bytes());
    bytes[28..32].copy_from_slice(&BRIDGE_FAULT_NOISE_DOMAIN_TAG);
    let raw = DeterministicRng::from_raw_seed(bytes).next_u64();
    let unit_open = ((raw >> 11) as f64) * (1.0 / ((1_u64 << 53) as f64));
    unit_open * 2.0 - 1.0
}

fn stable_rule_hash(rule_id: &str) -> u32 {
    let mut hash = 0x811c_9dc5_u32;
    for byte in rule_id.as_bytes() {
        hash ^= u32::from(*byte);
        hash = hash.wrapping_mul(0x0100_0193);
    }
    hash
}

const fn stable_signal_id(signal: BridgeScalarSignal) -> u64 {
    match signal {
        BridgeScalarSignal::ImuAccelBodyMps2(axis) => signal_with_axis_id(1, axis.index()),
        BridgeScalarSignal::ImuGyroBodyRadS(axis) => signal_with_axis_id(2, axis.index()),
        BridgeScalarSignal::ImuIncrementDeltaThetaRad { sample_index, axis } => {
            signal_with_index_axis_id(17, sample_index, axis.index())
        }
        BridgeScalarSignal::ImuIncrementDeltaVMs { sample_index, axis } => {
            signal_with_index_axis_id(18, sample_index, axis.index())
        }
        BridgeScalarSignal::ImuIncrementDtS { sample_index } => {
            signal_with_index_id(19, sample_index)
        }
        BridgeScalarSignal::BaroAltitudeM => 3_u64 << 32,
        BridgeScalarSignal::BaroPressurePa => 4_u64 << 32,
        BridgeScalarSignal::BaroBiasPa => 5_u64 << 32,
        BridgeScalarSignal::GnssPositionEciM(axis) => signal_with_axis_id(6, axis.index()),
        BridgeScalarSignal::GnssVelocityEciMS(axis) => signal_with_axis_id(7, axis.index()),
        BridgeScalarSignal::GnssPositionBiasEciM(axis) => signal_with_axis_id(8, axis.index()),
        BridgeScalarSignal::MagBodyTesla(axis) => signal_with_axis_id(9, axis.index()),
        BridgeScalarSignal::MagBodyNt(axis) => signal_with_axis_id(10, axis.index()),
        BridgeScalarSignal::MagHardIronBodyNt(axis) => signal_with_axis_id(11, axis.index()),
        BridgeScalarSignal::StarTrackerAttitudeEciToBody(axis) => {
            signal_with_axis_id(12, axis.index())
        }
        BridgeScalarSignal::EffectorCommand { effector_id } => {
            signal_with_index_id(13, effector_id)
        }
        BridgeScalarSignal::EngineThrottle { engine_id } => signal_with_index_id(14, engine_id),
        BridgeScalarSignal::EngineGimbalPitch { engine_id } => signal_with_index_id(15, engine_id),
        BridgeScalarSignal::EngineGimbalYaw { engine_id } => signal_with_index_id(16, engine_id),
    }
}

const fn signal_with_axis_id(tag: u64, axis: usize) -> u64 {
    (tag << 32) | axis as u64
}

const fn signal_with_index_id(tag: u64, index: u32) -> u64 {
    (tag << 32) | index as u64
}

const fn signal_with_index_axis_id(tag: u64, index: u32, axis: usize) -> u64 {
    (tag << 48) | ((index as u64) << 16) | axis as u64
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::packet::{EngineCommandPacket, ImuIncrementPacket};

    #[test]
    fn empty_fault_set_leaves_sensor_message_unchanged() {
        let mut message = BridgeMessage::Sensor(SensorPacket {
            step: 4,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        });
        let original = message.clone();

        let applied = BridgeFaultTransformSet::default()
            .apply_to_message(&mut message)
            .expect("apply empty set");

        assert!(applied.is_empty());
        assert_eq!(message, original);
    }

    #[test]
    fn sensor_bias_mutates_target_axis_only() {
        let mut packet = SensorPacket {
            step: 10,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            gnss_position_eci_m: Some([100.0, 200.0, 300.0]),
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new(vec![
            BridgeFaultRule::new(
                "imu-x-bias",
                10,
                None,
                BridgeScalarSignal::ImuAccelBodyMps2(VectorAxis::X),
                BridgeScalarTransform::AdditiveBias { offset: 0.5 },
            ),
            BridgeFaultRule::new(
                "gnss-z-scale",
                10,
                None,
                BridgeScalarSignal::GnssPositionEciM(VectorAxis::Z),
                BridgeScalarTransform::Scale { factor: 2.0 },
            ),
        ]);

        let applied = faults.apply_to_sensor(&mut packet).expect("apply faults");

        assert_eq!(applied, vec!["imu-x-bias", "gnss-z-scale"]);
        assert_eq!(
            packet.imu_accel_body_m_s2.map(f64::to_bits),
            [1.5_f64.to_bits(), 2.0_f64.to_bits(), 3.0_f64.to_bits()]
        );
        assert_eq!(
            packet.gnss_position_eci_m.unwrap().map(f64::to_bits),
            [
                100.0_f64.to_bits(),
                200.0_f64.to_bits(),
                600.0_f64.to_bits()
            ]
        );
    }

    #[test]
    fn sensor_fault_mutates_imu_increment_fields() {
        let mut packet = SensorPacket {
            step: 10,
            imu_increments: vec![
                ImuIncrementPacket {
                    delta_theta_rad: [1.0e-4, 2.0e-4, 3.0e-4],
                    delta_v_m_s: [0.001, 0.002, 0.003],
                    dt_s: 0.00025,
                    seq: 20,
                },
                ImuIncrementPacket {
                    delta_theta_rad: [4.0e-4, 5.0e-4, 6.0e-4],
                    delta_v_m_s: [0.004, 0.005, 0.006],
                    dt_s: 0.00025,
                    seq: 21,
                },
            ],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new(vec![
            BridgeFaultRule::new(
                "bias-dtheta",
                10,
                None,
                BridgeScalarSignal::ImuIncrementDeltaThetaRad {
                    sample_index: 1,
                    axis: VectorAxis::Y,
                },
                BridgeScalarTransform::AdditiveBias { offset: 1.0e-5 },
            ),
            BridgeFaultRule::new(
                "scale-dv",
                10,
                None,
                BridgeScalarSignal::ImuIncrementDeltaVMs {
                    sample_index: 0,
                    axis: VectorAxis::Z,
                },
                BridgeScalarTransform::Scale { factor: -2.0 },
            ),
            BridgeFaultRule::new(
                "stuck-dt",
                10,
                None,
                BridgeScalarSignal::ImuIncrementDtS { sample_index: 1 },
                BridgeScalarTransform::Stuck { value: 0.0005 },
            ),
            BridgeFaultRule::new(
                "missing-increment",
                10,
                None,
                BridgeScalarSignal::ImuIncrementDeltaVMs {
                    sample_index: 7,
                    axis: VectorAxis::X,
                },
                BridgeScalarTransform::Stuck { value: 99.0 },
            ),
        ]);

        let applied = faults
            .apply_to_sensor(&mut packet)
            .expect("apply increment faults");

        assert_eq!(applied, vec!["bias-dtheta", "scale-dv", "stuck-dt"]);
        assert_eq!(
            packet.imu_increments[1].delta_theta_rad[1].to_bits(),
            5.1e-4_f64.to_bits()
        );
        assert_eq!(
            packet.imu_increments[0].delta_v_m_s[2].to_bits(),
            (-0.006_f64).to_bits()
        );
        assert_eq!(
            packet.imu_increments[1].dt_s.to_bits(),
            0.0005_f64.to_bits()
        );
        assert_eq!(packet.imu_increments[1].seq, 21);
    }

    #[test]
    fn inactive_or_absent_signal_is_not_reported() {
        let mut packet = SensorPacket {
            step: 5,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new(vec![
            BridgeFaultRule::new(
                "future",
                6,
                None,
                BridgeScalarSignal::ImuAccelBodyMps2(VectorAxis::X),
                BridgeScalarTransform::Stuck { value: 42.0 },
            ),
            BridgeFaultRule::new(
                "missing-gnss",
                5,
                None,
                BridgeScalarSignal::GnssVelocityEciMS(VectorAxis::Y),
                BridgeScalarTransform::Stuck { value: 10.0 },
            ),
        ]);

        let applied = faults.apply_to_sensor(&mut packet).expect("apply faults");

        assert!(applied.is_empty());
        assert_eq!(
            packet.imu_accel_body_m_s2.map(f64::to_bits),
            [1.0_f64.to_bits(), 2.0_f64.to_bits(), 3.0_f64.to_bits()]
        );
    }

    #[test]
    fn command_fault_mutates_effector_and_engine_fields() {
        let mut packet = ActuatorCommandPacket {
            step: 12,
            effector_commands: vec![(1, 0.25), (2, -0.5)],
            engine_throttles: vec![(7, 0.8)],
            engine_commands: vec![EngineCommandPacket {
                engine_id: 7,
                throttle_unit: 0.8,
                gimbal_pitch_rad: 0.01,
                gimbal_yaw_rad: -0.02,
                ignite: true,
                shutdown: false,
            }],
            ..ActuatorCommandPacket::default()
        };
        let faults = BridgeFaultTransformSet::new(vec![
            BridgeFaultRule::new(
                "effector-reversed",
                12,
                Some(12),
                BridgeScalarSignal::EffectorCommand { effector_id: 2 },
                BridgeScalarTransform::ReverseSign,
            ),
            BridgeFaultRule::new(
                "engine-saturated",
                12,
                None,
                BridgeScalarSignal::EngineThrottle { engine_id: 7 },
                BridgeScalarTransform::Saturate { min: 0.0, max: 0.5 },
            ),
            BridgeFaultRule::new(
                "pitch-stuck",
                12,
                None,
                BridgeScalarSignal::EngineGimbalPitch { engine_id: 7 },
                BridgeScalarTransform::Stuck { value: 0.03 },
            ),
        ]);

        let applied = faults.apply_to_command(&mut packet).expect("apply faults");

        assert_eq!(
            applied,
            vec!["effector-reversed", "engine-saturated", "pitch-stuck"]
        );
        assert_eq!(packet.effector_commands[1].1.to_bits(), 0.5_f64.to_bits());
        assert_eq!(packet.engine_throttles[0].1.to_bits(), 0.5_f64.to_bits());
        assert_eq!(
            packet.engine_commands[0].throttle_unit.to_bits(),
            0.5_f64.to_bits()
        );
        assert_eq!(
            packet.engine_commands[0].gimbal_pitch_rad.to_bits(),
            0.03_f64.to_bits()
        );
        assert_eq!(
            packet.engine_commands[0].gimbal_yaw_rad.to_bits(),
            (-0.02_f64).to_bits()
        );
    }

    #[test]
    fn saturate_rejects_invalid_range() {
        let mut packet = SensorPacket {
            step: 1,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new(vec![BridgeFaultRule::new(
            "bad-range",
            1,
            None,
            BridgeScalarSignal::ImuAccelBodyMps2(VectorAxis::X),
            BridgeScalarTransform::Saturate {
                min: 10.0,
                max: 1.0,
            },
        )]);

        let err = faults
            .apply_to_sensor(&mut packet)
            .expect_err("invalid range should fail");

        assert!(
            matches!(err, BridgeFaultError::InvalidSaturationRange { .. }),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn quantize_rounds_to_nearest_step_and_rejects_invalid_quantum() {
        let mut packet = SensorPacket {
            step: 1,
            imu_accel_body_m_s2: [1.24, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new(vec![BridgeFaultRule::new(
            "quantize-x",
            1,
            None,
            BridgeScalarSignal::ImuAccelBodyMps2(VectorAxis::X),
            BridgeScalarTransform::Quantize { quantum: 0.5 },
        )]);

        let applied = faults.apply_to_sensor(&mut packet).expect("quantize");

        assert_eq!(applied, vec!["quantize-x"]);
        assert_eq!(packet.imu_accel_body_m_s2[0].to_bits(), 1.0_f64.to_bits());

        let mut packet = SensorPacket {
            step: 1,
            imu_accel_body_m_s2: [1.24, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new(vec![BridgeFaultRule::new(
            "bad-quantum",
            1,
            None,
            BridgeScalarSignal::ImuAccelBodyMps2(VectorAxis::X),
            BridgeScalarTransform::Quantize { quantum: 0.0 },
        )]);

        let err = faults
            .apply_to_sensor(&mut packet)
            .expect_err("invalid quantum should fail");

        assert!(
            matches!(err, BridgeFaultError::InvalidQuantizationStep { .. }),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn drift_adds_time_ramp_relative_to_reference_time() {
        let mut packet = SensorPacket {
            sim_time_s: 12.5,
            step: 3,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new(vec![BridgeFaultRule::new(
            "drift-x",
            3,
            None,
            BridgeScalarSignal::ImuAccelBodyMps2(VectorAxis::X),
            BridgeScalarTransform::Drift {
                rate_per_s: 0.2,
                reference_time_s: 10.0,
            },
        )]);

        let applied = faults.apply_to_sensor(&mut packet).expect("apply drift");

        assert_eq!(applied, vec!["drift-x"]);
        assert_eq!(packet.imu_accel_body_m_s2[0].to_bits(), 1.5_f64.to_bits());
        assert_eq!(packet.imu_accel_body_m_s2[1].to_bits(), 2.0_f64.to_bits());
    }

    #[test]
    fn noise_burst_is_deterministic_bounded_and_step_local() {
        let signal = BridgeScalarSignal::ImuGyroBodyRadS(VectorAxis::Z);
        let transform = BridgeScalarTransform::NoiseBurst {
            amplitude: 0.25,
            seed: 0x5eed,
        };
        let faults = BridgeFaultTransformSet::new(vec![BridgeFaultRule::new(
            "gyro-noise",
            4,
            None,
            signal,
            transform,
        )]);
        let mut packet = SensorPacket {
            sim_time_s: 0.004,
            step: 4,
            imu_gyro_body_rad_s: [0.0, 0.0, 1.0],
            ..SensorPacket::default()
        };
        let mut repeat = packet.clone();
        let context = ScalarTransformContext {
            step: 4,
            sim_time_s: 0.004,
            rule_id: "gyro-noise",
            signal,
        };
        let expected = 1.0 + 0.25 * deterministic_noise_unit(0x5eed, context);

        let applied = faults.apply_to_sensor(&mut packet).expect("apply noise");
        let repeat_applied = faults
            .apply_to_sensor(&mut repeat)
            .expect("repeat noise application");

        assert_eq!(applied, vec!["gyro-noise"]);
        assert_eq!(repeat_applied, vec!["gyro-noise"]);
        assert_eq!(packet, repeat);
        assert_eq!(packet.imu_gyro_body_rad_s[2].to_bits(), expected.to_bits());
        assert!((packet.imu_gyro_body_rad_s[2] - 1.0).abs() <= 0.25);

        let mut next_step = SensorPacket {
            sim_time_s: 0.005,
            step: 5,
            imu_gyro_body_rad_s: [0.0, 0.0, 1.0],
            ..SensorPacket::default()
        };
        faults
            .apply_to_sensor(&mut next_step)
            .expect("next step noise");
        assert_ne!(
            packet.imu_gyro_body_rad_s[2].to_bits(),
            next_step.imu_gyro_body_rad_s[2].to_bits()
        );
    }

    #[test]
    fn drift_and_noise_burst_reject_invalid_parameters() {
        let mut packet = SensorPacket {
            sim_time_s: 1.0,
            step: 5,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new(vec![BridgeFaultRule::new(
            "bad-drift",
            5,
            None,
            BridgeScalarSignal::ImuAccelBodyMps2(VectorAxis::X),
            BridgeScalarTransform::Drift {
                rate_per_s: f64::INFINITY,
                reference_time_s: 0.0,
            },
        )]);

        let err = faults
            .apply_to_sensor(&mut packet)
            .expect_err("invalid drift should fail");
        assert!(
            matches!(err, BridgeFaultError::InvalidDriftTransform { .. }),
            "unexpected error: {err}",
        );

        let mut packet = SensorPacket {
            sim_time_s: 1.0,
            step: 5,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new(vec![BridgeFaultRule::new(
            "bad-noise",
            5,
            None,
            BridgeScalarSignal::ImuAccelBodyMps2(VectorAxis::X),
            BridgeScalarTransform::NoiseBurst {
                amplitude: 0.0,
                seed: 1,
            },
        )]);

        let err = faults
            .apply_to_sensor(&mut packet)
            .expect_err("invalid noise amplitude should fail");
        assert!(
            matches!(err, BridgeFaultError::InvalidNoiseBurstAmplitude { .. }),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn packet_drop_marks_sensor_frame_disposition_after_scalar_rules() {
        let mut packet = SensorPacket {
            step: 7,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            vec![BridgeFaultRule::new(
                "imu-x-bias",
                7,
                None,
                BridgeScalarSignal::ImuAccelBodyMps2(VectorAxis::X),
                BridgeScalarTransform::AdditiveBias { offset: 1.0 },
            )],
            vec![BridgePacketFaultRule::new(
                "drop-sensor",
                7,
                Some(7),
                BridgePacketDirection::Sensor,
                BridgePacketTransform::Drop,
            )],
        );

        let application = faults
            .apply_sensor_frame(&mut packet)
            .expect("apply sensor frame faults");

        assert_eq!(
            application.applied_rule_ids,
            vec!["imu-x-bias", "drop-sensor"]
        );
        assert_eq!(application.disposition, BridgePacketDisposition::Drop);
        assert_eq!(packet.imu_accel_body_m_s2[0].to_bits(), 2.0_f64.to_bits());
    }

    #[test]
    fn inactive_packet_drop_delivers_command_frame() {
        let mut packet = ActuatorCommandPacket {
            step: 10,
            effector_commands: vec![(3, 0.4)],
            ..ActuatorCommandPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "drop-command",
                11,
                None,
                BridgePacketDirection::Command,
                BridgePacketTransform::Drop,
            )],
        );

        let application = faults
            .apply_command_frame(&mut packet)
            .expect("apply command frame faults");

        assert!(application.applied_rule_ids.is_empty());
        assert_eq!(application.disposition, BridgePacketDisposition::Deliver);
    }

    #[test]
    fn packet_duplicate_marks_command_frame_disposition() {
        let mut packet = ActuatorCommandPacket {
            step: 13,
            effector_commands: vec![(1, 0.1)],
            ..ActuatorCommandPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "duplicate-command",
                13,
                None,
                BridgePacketDirection::Command,
                BridgePacketTransform::Duplicate,
            )],
        );

        let application = faults
            .apply_command_frame(&mut packet)
            .expect("apply command frame faults");

        assert_eq!(application.applied_rule_ids, vec!["duplicate-command"]);
        assert_eq!(application.disposition, BridgePacketDisposition::Duplicate);
    }

    #[test]
    fn packet_delay_marks_sensor_frame_disposition() {
        let mut packet = SensorPacket {
            step: 14,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "delay-sensor",
                14,
                None,
                BridgePacketDirection::Sensor,
                BridgePacketTransform::Delay { steps: 2 },
            )],
        );

        let application = faults
            .apply_sensor_frame(&mut packet)
            .expect("apply sensor frame faults");

        assert_eq!(application.applied_rule_ids, vec!["delay-sensor"]);
        assert_eq!(
            application.disposition,
            BridgePacketDisposition::Delay { steps: 2 }
        );
    }

    #[test]
    fn packet_bit_flip_marks_command_frame_disposition() {
        let mut packet = ActuatorCommandPacket {
            step: 15,
            effector_commands: vec![(1, 0.1)],
            ..ActuatorCommandPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "bit-flip-command",
                15,
                None,
                BridgePacketDirection::Command,
                BridgePacketTransform::BitFlip { mask: 0b0000_0101 },
            )],
        );

        let application = faults
            .apply_command_frame(&mut packet)
            .expect("apply command frame faults");

        assert_eq!(application.applied_rule_ids, vec!["bit-flip-command"]);
        assert_eq!(
            application.disposition,
            BridgePacketDisposition::BitFlip { mask: 0b0000_0101 }
        );
    }

    #[test]
    fn packet_step_and_time_offsets_mutate_frame_and_deliver() {
        let mut packet = ActuatorCommandPacket {
            step: 20,
            sim_time_s: 0.2,
            effector_commands: vec![(1, 0.1)],
            ..ActuatorCommandPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![
                BridgePacketFaultRule::new(
                    "offset-step",
                    20,
                    None,
                    BridgePacketDirection::Command,
                    BridgePacketTransform::StepOffset { offset: 2 },
                ),
                BridgePacketFaultRule::new(
                    "offset-time",
                    20,
                    None,
                    BridgePacketDirection::Command,
                    BridgePacketTransform::TimeOffset { offset_s: -0.01 },
                ),
            ],
        );

        let application = faults
            .apply_command_frame(&mut packet)
            .expect("apply command frame faults");

        assert_eq!(
            application.applied_rule_ids,
            vec!["offset-step", "offset-time"]
        );
        assert_eq!(application.disposition, BridgePacketDisposition::Deliver);
        assert_eq!(packet.step, 22);
        assert!((packet.sim_time_s - 0.19).abs() < f64::EPSILON);
    }

    #[test]
    fn packet_step_offset_saturates_at_zero() {
        let mut packet = SensorPacket {
            step: 1,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "negative-step-offset",
                1,
                None,
                BridgePacketDirection::Sensor,
                BridgePacketTransform::StepOffset { offset: -5 },
            )],
        );

        let application = faults
            .apply_sensor_frame(&mut packet)
            .expect("apply sensor frame faults");

        assert_eq!(application.applied_rule_ids, vec!["negative-step-offset"]);
        assert_eq!(packet.step, 0);
    }

    #[test]
    fn packet_level_faults_reject_invalid_parameters() {
        let mut sensor = SensorPacket {
            step: 16,
            imu_accel_body_m_s2: [1.0, 2.0, 3.0],
            ..SensorPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "bad-delay",
                16,
                None,
                BridgePacketDirection::Sensor,
                BridgePacketTransform::Delay { steps: 0 },
            )],
        );

        let err = faults
            .apply_sensor_frame(&mut sensor)
            .expect_err("zero delay should fail");
        assert!(
            matches!(err, BridgeFaultError::InvalidPacketDelay { steps: 0 }),
            "unexpected error: {err}",
        );

        let mut command = ActuatorCommandPacket {
            step: 17,
            effector_commands: vec![(1, 0.1)],
            ..ActuatorCommandPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "bad-mask",
                17,
                None,
                BridgePacketDirection::Command,
                BridgePacketTransform::BitFlip { mask: 0 },
            )],
        );

        let err = faults
            .apply_command_frame(&mut command)
            .expect_err("zero bit-flip mask should fail");
        assert!(
            matches!(err, BridgeFaultError::InvalidPacketBitFlipMask { mask: 0 }),
            "unexpected error: {err}",
        );

        let mut command = ActuatorCommandPacket {
            step: 18,
            effector_commands: vec![(1, 0.1)],
            ..ActuatorCommandPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "bad-step-offset",
                18,
                None,
                BridgePacketDirection::Command,
                BridgePacketTransform::StepOffset { offset: 0 },
            )],
        );

        let err = faults
            .apply_command_frame(&mut command)
            .expect_err("zero step offset should fail");
        assert!(
            matches!(err, BridgeFaultError::InvalidPacketStepOffset { offset: 0 }),
            "unexpected error: {err}",
        );

        let mut command = ActuatorCommandPacket {
            step: 19,
            effector_commands: vec![(1, 0.1)],
            ..ActuatorCommandPacket::default()
        };
        let faults = BridgeFaultTransformSet::new_with_packet_rules(
            Vec::new(),
            vec![BridgePacketFaultRule::new(
                "bad-time-offset",
                19,
                None,
                BridgePacketDirection::Command,
                BridgePacketTransform::TimeOffset { offset_s: f64::NAN },
            )],
        );

        let err = faults
            .apply_command_frame(&mut command)
            .expect_err("non-finite time offset should fail");
        assert!(
            matches!(
                err,
                BridgeFaultError::InvalidPacketTimeOffset { offset_s }
                    if offset_s.is_nan()
            ),
            "unexpected error: {err}",
        );
    }
}
