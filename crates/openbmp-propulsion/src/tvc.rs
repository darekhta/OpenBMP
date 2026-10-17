//! Thrust-vector-control actuator plant primitives.
//!
//! The model is intentionally small and deterministic: each TVC axis is a
//! second-order bandwidth model with position and rate limits, an external
//! hinge/load torque input, and an inertial reaction torque output for
//! tail-wags-dog studies. Scenario and runner wiring live outside this module.

#[cfg(not(feature = "std"))]
use num_traits::Float;
use openbmp_core::Duration;

use crate::error::EngineError;

/// Parameters for a two-axis second-order TVC actuator plant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TvcActuatorParams {
    /// Natural frequency `omega_n`, rad/s. Must be strictly positive.
    pub natural_frequency_rad_s: f64,
    /// Damping ratio `zeta`. Must be finite and non-negative.
    pub damping_ratio: f64,
    /// Maximum delivered gimbal angle magnitude on either axis, rad.
    pub max_angle_rad: f64,
    /// Maximum delivered gimbal rate magnitude on either axis, rad/s.
    /// `+infinity` disables the rate limiter.
    pub max_rate_rad_s: f64,
    /// Reflected engine/gimbal inertia about each TVC axis, kg*m^2.
    /// This inertia is used both for load-torque acceleration feedback and
    /// for the reported tail-wags-dog inertial reaction torque.
    pub gimbal_inertia_kg_m2: f64,
}

impl TvcActuatorParams {
    /// Validate the actuator parameters.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidParameter`] when any field is non-finite
    /// or outside its documented range.
    pub fn require_valid(self) -> Result<(), EngineError> {
        if !self.natural_frequency_rad_s.is_finite() || self.natural_frequency_rad_s <= 0.0 {
            return Err(EngineError::InvalidParameter {
                reason: "TVC natural_frequency_rad_s must be finite and strictly positive",
            });
        }
        if !self.damping_ratio.is_finite() || self.damping_ratio < 0.0 {
            return Err(EngineError::InvalidParameter {
                reason: "TVC damping_ratio must be finite and non-negative",
            });
        }
        if !self.max_angle_rad.is_finite() || self.max_angle_rad < 0.0 {
            return Err(EngineError::InvalidParameter {
                reason: "TVC max_angle_rad must be finite and non-negative",
            });
        }
        if self.max_rate_rad_s.is_nan() || self.max_rate_rad_s < 0.0 {
            return Err(EngineError::InvalidParameter {
                reason: "TVC max_rate_rad_s must be non-negative or +infinity",
            });
        }
        if !self.gimbal_inertia_kg_m2.is_finite() || self.gimbal_inertia_kg_m2 <= 0.0 {
            return Err(EngineError::InvalidParameter {
                reason: "TVC gimbal_inertia_kg_m2 must be finite and strictly positive",
            });
        }
        Ok(())
    }
}

/// Commanded TVC angles for the pitch and yaw axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TvcActuatorCommand {
    /// Commanded pitch-axis gimbal angle, rad.
    pub pitch_rad: f64,
    /// Commanded yaw-axis gimbal angle, rad.
    pub yaw_rad: f64,
}

impl TvcActuatorCommand {
    /// Zero pitch/yaw command.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            pitch_rad: 0.0,
            yaw_rad: 0.0,
        }
    }

    /// Validate the command values.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::NonFiniteCommand`] when either command axis is
    /// non-finite.
    pub fn require_valid(self) -> Result<(), EngineError> {
        if !self.pitch_rad.is_finite() {
            return Err(EngineError::NonFiniteCommand {
                reason: "TVC pitch_rad",
            });
        }
        if !self.yaw_rad.is_finite() {
            return Err(EngineError::NonFiniteCommand {
                reason: "TVC yaw_rad",
            });
        }
        Ok(())
    }
}

/// External hinge/load torque inputs for the pitch and yaw axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TvcLoadTorque {
    /// Pitch-axis load torque, N*m. Positive torque opposes a positive
    /// commanded pitch deflection in this plant convention.
    pub pitch_n_m: f64,
    /// Yaw-axis load torque, N*m. Positive torque opposes a positive
    /// commanded yaw deflection in this plant convention.
    pub yaw_n_m: f64,
}

impl TvcLoadTorque {
    /// Zero pitch/yaw load torque.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            pitch_n_m: 0.0,
            yaw_n_m: 0.0,
        }
    }

    /// Validate the load torque values.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::NonFiniteCommand`] when either load-torque axis
    /// is non-finite.
    pub fn require_valid(self) -> Result<(), EngineError> {
        if !self.pitch_n_m.is_finite() {
            return Err(EngineError::NonFiniteCommand {
                reason: "TVC pitch load torque",
            });
        }
        if !self.yaw_n_m.is_finite() {
            return Err(EngineError::NonFiniteCommand {
                reason: "TVC yaw load torque",
            });
        }
        Ok(())
    }
}

/// State of one scalar TVC actuator axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TvcAxisState {
    /// Delivered gimbal angle, rad.
    pub angle_rad: f64,
    /// Delivered gimbal rate, rad/s.
    pub rate_rad_s: f64,
}

impl TvcAxisState {
    /// Zero angle/rate axis state.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            angle_rad: 0.0,
            rate_rad_s: 0.0,
        }
    }
}

/// State of the two-axis TVC actuator plant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TvcActuatorState {
    /// Pitch-axis state.
    pub pitch: TvcAxisState,
    /// Yaw-axis state.
    pub yaw: TvcAxisState,
}

impl TvcActuatorState {
    /// Zero pitch/yaw actuator state.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            pitch: TvcAxisState::zero(),
            yaw: TvcAxisState::zero(),
        }
    }
}

/// Per-axis TVC actuator step result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TvcAxisStep {
    /// Commanded angle after plant-side position clamping, rad.
    pub commanded_rad: f64,
    /// External load torque applied to this axis, N*m.
    pub load_torque_n_m: f64,
    /// Delivered gimbal angle after dynamics and limits, rad.
    pub angle_rad: f64,
    /// Delivered gimbal rate after dynamics and limits, rad/s.
    pub rate_rad_s: f64,
    /// Delivered angular acceleration inferred from the fixed-step rate
    /// change, rad/s^2.
    pub acceleration_rad_s2: f64,
    /// Tail-wags-dog inertial reaction torque, `-J * delta_ddot`, N*m.
    pub inertial_reaction_torque_n_m: f64,
    /// True when the delivered angle was clipped by the position limit.
    pub saturated: bool,
    /// True when the delivered rate was clipped by the rate limit.
    pub rate_limited: bool,
}

/// Two-axis TVC actuator step result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TvcActuatorStep {
    /// Pitch-axis step result.
    pub pitch: TvcAxisStep,
    /// Yaw-axis step result.
    pub yaw: TvcAxisStep,
}

/// Deterministic second-order TVC actuator plant with load-torque feedback.
#[derive(Clone, Debug, PartialEq)]
pub struct TvcActuatorPlant {
    params: TvcActuatorParams,
    state: TvcActuatorState,
}

impl TvcActuatorPlant {
    /// Construct a TVC actuator plant at zero pitch/yaw angle and rate.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidParameter`] when `params` fails
    /// validation.
    pub fn new(params: TvcActuatorParams) -> Result<Self, EngineError> {
        params.require_valid()?;
        Ok(Self {
            params,
            state: TvcActuatorState::zero(),
        })
    }

    /// Construct a TVC actuator plant from an explicit state.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidParameter`] when `params` fails
    /// validation or when `state` is non-finite or outside the declared
    /// position/rate limits.
    pub fn with_state(
        params: TvcActuatorParams,
        state: TvcActuatorState,
    ) -> Result<Self, EngineError> {
        params.require_valid()?;
        require_axis_state("pitch", params, state.pitch)?;
        require_axis_state("yaw", params, state.yaw)?;
        Ok(Self { params, state })
    }

    /// Return the immutable plant parameters.
    #[must_use]
    pub const fn params(&self) -> TvcActuatorParams {
        self.params
    }

    /// Return the current pitch/yaw actuator state.
    #[must_use]
    pub const fn state(&self) -> TvcActuatorState {
        self.state
    }

    /// Advance both TVC actuator axes by one fixed step.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::InvalidDt`] for non-positive or non-finite `dt`,
    /// and [`EngineError::NonFiniteCommand`] for non-finite command or load
    /// torque components.
    pub fn step(
        &mut self,
        command: TvcActuatorCommand,
        load_torque: TvcLoadTorque,
        dt: Duration,
    ) -> Result<TvcActuatorStep, EngineError> {
        command.require_valid()?;
        load_torque.require_valid()?;
        let dt_s = dt.as_seconds();
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(EngineError::InvalidDt { got_s: dt_s });
        }

        let pitch = step_axis(
            self.params,
            &mut self.state.pitch,
            command.pitch_rad,
            load_torque.pitch_n_m,
            dt_s,
        );
        let yaw = step_axis(
            self.params,
            &mut self.state.yaw,
            command.yaw_rad,
            load_torque.yaw_n_m,
            dt_s,
        );
        Ok(TvcActuatorStep { pitch, yaw })
    }
}

fn require_axis_state(
    axis_name: &'static str,
    params: TvcActuatorParams,
    state: TvcAxisState,
) -> Result<(), EngineError> {
    if !state.angle_rad.is_finite() {
        return Err(EngineError::InvalidParameter {
            reason: match axis_name {
                "pitch" => "TVC initial pitch angle must be finite",
                _ => "TVC initial yaw angle must be finite",
            },
        });
    }
    if !state.rate_rad_s.is_finite() {
        return Err(EngineError::InvalidParameter {
            reason: match axis_name {
                "pitch" => "TVC initial pitch rate must be finite",
                _ => "TVC initial yaw rate must be finite",
            },
        });
    }
    if state.angle_rad < -params.max_angle_rad || state.angle_rad > params.max_angle_rad {
        return Err(EngineError::InvalidParameter {
            reason: match axis_name {
                "pitch" => "TVC initial pitch angle must lie within +/-max_angle_rad",
                _ => "TVC initial yaw angle must lie within +/-max_angle_rad",
            },
        });
    }
    if params.max_rate_rad_s.is_finite()
        && (state.rate_rad_s < -params.max_rate_rad_s || state.rate_rad_s > params.max_rate_rad_s)
    {
        return Err(EngineError::InvalidParameter {
            reason: match axis_name {
                "pitch" => "TVC initial pitch rate must lie within +/-max_rate_rad_s",
                _ => "TVC initial yaw rate must lie within +/-max_rate_rad_s",
            },
        });
    }
    Ok(())
}

fn step_axis(
    params: TvcActuatorParams,
    state: &mut TvcAxisState,
    command_rad: f64,
    load_torque_n_m: f64,
    dt_s: f64,
) -> TvcAxisStep {
    let commanded_rad = command_rad.clamp(-params.max_angle_rad, params.max_angle_rad);
    let load_angle_rad = load_torque_n_m
        / (params.gimbal_inertia_kg_m2
            * params.natural_frequency_rad_s
            * params.natural_frequency_rad_s);
    let effective_target_rad = commanded_rad - load_angle_rad;
    let old_rate_rad_s = state.rate_rad_s;
    let (raw_angle_rad, raw_rate_rad_s) = second_order_exact_step(
        state.angle_rad,
        state.rate_rad_s,
        effective_target_rad,
        params.natural_frequency_rad_s,
        params.damping_ratio,
        dt_s,
    );

    let mut angle_rad = raw_angle_rad;
    let mut rate_rad_s = raw_rate_rad_s;
    let rate_limited =
        params.max_rate_rad_s.is_finite() && raw_rate_rad_s.abs() > params.max_rate_rad_s;
    if rate_limited {
        rate_rad_s = raw_rate_rad_s.clamp(-params.max_rate_rad_s, params.max_rate_rad_s);
        angle_rad = state.angle_rad + rate_rad_s * dt_s;
    }

    let saturated = angle_rad < -params.max_angle_rad || angle_rad > params.max_angle_rad;
    if saturated {
        angle_rad = angle_rad.clamp(-params.max_angle_rad, params.max_angle_rad);
        if (angle_rad <= -params.max_angle_rad && rate_rad_s < 0.0)
            || (angle_rad >= params.max_angle_rad && rate_rad_s > 0.0)
        {
            rate_rad_s = 0.0;
        }
    }

    let acceleration_rad_s2 = (rate_rad_s - old_rate_rad_s) / dt_s;
    state.angle_rad = angle_rad;
    state.rate_rad_s = rate_rad_s;
    TvcAxisStep {
        commanded_rad,
        load_torque_n_m,
        angle_rad,
        rate_rad_s,
        acceleration_rad_s2,
        inertial_reaction_torque_n_m: -params.gimbal_inertia_kg_m2 * acceleration_rad_s2,
        saturated,
        rate_limited,
    }
}

fn second_order_exact_step(
    actual: f64,
    rate: f64,
    target: f64,
    omega_n: f64,
    damping: f64,
    dt_s: f64,
) -> (f64, f64) {
    let error0 = actual - target;
    if damping < 1.0 - 1.0e-12 {
        let omega_d = omega_n * (1.0 - damping * damping).sqrt();
        let decay = (-damping * omega_n * dt_s).exp();
        let (sin_wd, cos_wd) = (omega_d * dt_s).sin_cos();
        let a = error0;
        let b = (rate + damping * omega_n * error0) / omega_d;
        let error = decay * (a * cos_wd + b * sin_wd);
        let error_rate = decay
            * (-damping * omega_n * (a * cos_wd + b * sin_wd)
                + (-a * omega_d * sin_wd + b * omega_d * cos_wd));
        (target + error, error_rate)
    } else if (damping - 1.0).abs() <= 1.0e-12 {
        let decay = (-omega_n * dt_s).exp();
        let a = error0;
        let b = rate + omega_n * error0;
        let error = decay * (a + b * dt_s);
        let error_rate = decay * (b - omega_n * (a + b * dt_s));
        (target + error, error_rate)
    } else {
        let root = (damping * damping - 1.0).sqrt();
        let r1 = -omega_n * (damping - root);
        let r2 = -omega_n * (damping + root);
        let c1 = (rate - r2 * error0) / (r1 - r2);
        let c2 = error0 - c1;
        let e1 = (r1 * dt_s).exp();
        let e2 = (r2 * dt_s).exp();
        let error = c1 * e1 + c2 * e2;
        let error_rate = c1 * r1 * e1 + c2 * r2 * e2;
        (target + error, error_rate)
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::unwrap_used
)]
mod tests {
    use super::*;
    use toml::Value;

    const TVC_TWD_INCA_TOLERANCE_TABLE: &str =
        include_str!("../../../data/gnc/tvc-twd-inca-tolerance-v1.toml");

    fn fixture() -> Value {
        toml::from_str(TVC_TWD_INCA_TOLERANCE_TABLE).expect("TVC/TWD fixture parses")
    }

    fn toml_table<'a>(value: &'a Value, key: &str) -> &'a toml::map::Map<String, Value> {
        value
            .get(key)
            .and_then(Value::as_table)
            .unwrap_or_else(|| panic!("missing TOML table {key}"))
    }

    fn toml_f64(value: &toml::map::Map<String, Value>, key: &str) -> f64 {
        value
            .get(key)
            .and_then(Value::as_float)
            .unwrap_or_else(|| panic!("missing TOML float {key}"))
    }

    fn toml_rate(value: &toml::map::Map<String, Value>, key: &str) -> f64 {
        let raw = value
            .get(key)
            .unwrap_or_else(|| panic!("missing TOML rate {key}"));
        if raw.as_str() == Some("inf") {
            f64::INFINITY
        } else {
            raw.as_float()
                .unwrap_or_else(|| panic!("missing TOML float-or-inf {key}"))
        }
    }

    fn params() -> TvcActuatorParams {
        TvcActuatorParams {
            natural_frequency_rad_s: 20.0,
            damping_ratio: 0.5,
            max_angle_rad: 0.1,
            max_rate_rad_s: f64::INFINITY,
            gimbal_inertia_kg_m2: 2.0,
        }
    }

    #[test]
    fn tvc_actuator_matches_second_order_step_response_without_load() {
        let data = fixture();
        let tolerances = toml_table(&data, "tolerances");
        let case = toml_table(&data, "tvc_step_response");
        let params = TvcActuatorParams {
            natural_frequency_rad_s: toml_f64(case, "natural_frequency_rad_s"),
            damping_ratio: toml_f64(case, "damping_ratio"),
            max_angle_rad: toml_f64(case, "max_angle_rad"),
            max_rate_rad_s: toml_rate(case, "max_rate_rad_s"),
            gimbal_inertia_kg_m2: toml_f64(case, "gimbal_inertia_kg_m2"),
        };
        let mut plant = TvcActuatorPlant::new(params).unwrap();
        let command = TvcActuatorCommand {
            pitch_rad: toml_f64(case, "command_pitch_rad"),
            yaw_rad: toml_f64(case, "command_yaw_rad"),
        };
        let step = plant
            .step(
                command,
                TvcLoadTorque {
                    pitch_n_m: toml_f64(case, "load_pitch_n_m"),
                    yaw_n_m: toml_f64(case, "load_yaw_n_m"),
                },
                Duration::from_seconds(toml_f64(case, "dt_s")),
            )
            .unwrap();

        assert!(
            (step.pitch.angle_rad - toml_f64(case, "expected_pitch_angle_rad")).abs()
                < toml_f64(tolerances, "tvc_angle_abs_rad")
        );
        assert!(
            (step.pitch.rate_rad_s - toml_f64(case, "expected_pitch_rate_rad_s")).abs()
                < toml_f64(tolerances, "tvc_rate_abs_rad_s")
        );
        assert!(!step.pitch.saturated);
        assert!(!step.pitch.rate_limited);
    }

    #[test]
    fn tvc_actuator_load_torque_reduces_deflection_and_reports_twd_reaction() {
        let data = fixture();
        let tolerances = toml_table(&data, "tolerances");
        let case = toml_table(&data, "tvc_load_torque");
        let params = TvcActuatorParams {
            natural_frequency_rad_s: toml_f64(case, "natural_frequency_rad_s"),
            damping_ratio: toml_f64(case, "damping_ratio"),
            max_angle_rad: toml_f64(case, "max_angle_rad"),
            max_rate_rad_s: toml_rate(case, "max_rate_rad_s"),
            gimbal_inertia_kg_m2: toml_f64(case, "gimbal_inertia_kg_m2"),
        };
        let mut unloaded = TvcActuatorPlant::new(params).unwrap();
        let mut loaded = TvcActuatorPlant::new(params).unwrap();
        let command = TvcActuatorCommand {
            pitch_rad: toml_f64(case, "command_pitch_rad"),
            yaw_rad: toml_f64(case, "command_yaw_rad"),
        };
        let dt = Duration::from_seconds(toml_f64(case, "dt_s"));
        let no_load = unloaded.step(command, TvcLoadTorque::zero(), dt).unwrap();
        let with_load = loaded
            .step(
                command,
                TvcLoadTorque {
                    pitch_n_m: toml_f64(case, "load_pitch_n_m"),
                    yaw_n_m: toml_f64(case, "load_yaw_n_m"),
                },
                dt,
            )
            .unwrap();

        assert!(with_load.pitch.angle_rad < no_load.pitch.angle_rad);
        assert!(with_load.pitch.acceleration_rad_s2 < no_load.pitch.acceleration_rad_s2);
        assert!(
            (no_load.pitch.angle_rad - toml_f64(case, "expected_unloaded_pitch_angle_rad")).abs()
                < toml_f64(tolerances, "tvc_angle_abs_rad")
        );
        assert!(
            (with_load.pitch.angle_rad - toml_f64(case, "expected_loaded_pitch_angle_rad")).abs()
                < toml_f64(tolerances, "tvc_angle_abs_rad")
        );
        assert!(
            (with_load.pitch.inertial_reaction_torque_n_m
                - toml_f64(case, "expected_loaded_reaction_torque_n_m"))
            .abs()
                < toml_f64(tolerances, "tvc_reaction_abs_n_m")
        );
    }

    #[test]
    fn tvc_actuator_rate_and_position_limits_bound_delivered_angle() {
        let limited_params = TvcActuatorParams {
            natural_frequency_rad_s: 1_000.0,
            damping_ratio: 0.7,
            max_angle_rad: 0.00025,
            max_rate_rad_s: 0.1,
            gimbal_inertia_kg_m2: 1.0,
        };
        let mut plant = TvcActuatorPlant::new(limited_params).unwrap();
        let command = TvcActuatorCommand {
            pitch_rad: 1.0,
            yaw_rad: 0.0,
        };

        let first = plant
            .step(
                command,
                TvcLoadTorque::zero(),
                Duration::from_seconds(0.001),
            )
            .unwrap();
        assert!(first.pitch.rate_limited);
        assert!((first.pitch.angle_rad - 0.0001).abs() < 1.0e-14);
        assert!((first.pitch.rate_rad_s - limited_params.max_rate_rad_s).abs() < 1.0e-14);

        let mut near_limit = TvcActuatorPlant::with_state(
            limited_params,
            TvcActuatorState {
                pitch: TvcAxisState {
                    angle_rad: 0.00024,
                    rate_rad_s: limited_params.max_rate_rad_s,
                },
                yaw: TvcAxisState::zero(),
            },
        )
        .unwrap();
        let clipped = near_limit
            .step(
                command,
                TvcLoadTorque::zero(),
                Duration::from_seconds(0.001),
            )
            .unwrap();
        assert!(clipped.pitch.saturated);
        assert!(clipped.pitch.angle_rad <= limited_params.max_angle_rad);
        assert!(clipped.pitch.rate_rad_s.abs() <= limited_params.max_rate_rad_s);
    }

    #[test]
    fn tvc_actuator_rejects_invalid_params_state_and_inputs() {
        assert!(matches!(
            TvcActuatorPlant::new(TvcActuatorParams {
                natural_frequency_rad_s: 0.0,
                ..params()
            }),
            Err(EngineError::InvalidParameter { .. })
        ));
        assert!(matches!(
            TvcActuatorPlant::with_state(
                params(),
                TvcActuatorState {
                    pitch: TvcAxisState {
                        angle_rad: 0.2,
                        rate_rad_s: 0.0,
                    },
                    yaw: TvcAxisState::zero(),
                },
            ),
            Err(EngineError::InvalidParameter { .. })
        ));

        let mut plant = TvcActuatorPlant::new(params()).unwrap();
        assert!(matches!(
            plant.step(
                TvcActuatorCommand {
                    pitch_rad: f64::NAN,
                    yaw_rad: 0.0,
                },
                TvcLoadTorque::zero(),
                Duration::from_seconds(0.01),
            ),
            Err(EngineError::NonFiniteCommand { .. })
        ));
        assert!(matches!(
            plant.step(
                TvcActuatorCommand::zero(),
                TvcLoadTorque {
                    pitch_n_m: f64::INFINITY,
                    yaw_n_m: 0.0,
                },
                Duration::from_seconds(0.01),
            ),
            Err(EngineError::NonFiniteCommand { .. })
        ));
        assert!(matches!(
            plant.step(
                TvcActuatorCommand::zero(),
                TvcLoadTorque::zero(),
                Duration::from_seconds(0.0),
            ),
            Err(EngineError::InvalidDt { .. })
        ));
    }
}
