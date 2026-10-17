//! Canopy inflation profile primitives.
//!
//! These helpers are the first WP-16.1 substrate: deterministic fill-time
//! growth for an effective parachute `C_D S` term plus closed-form
//! infinite-mass opening-load diagnostics. They also provide the validated
//! Pflanz finite-mass reduction-table primitive needed before production
//! canopy tables are ingested. The scalar apparent-mass helpers implement the
//! T1 added-mass algebra; broader integrated-EOM closure remains future work.

use super::{RecoveryError, require_positive_area, require_positive_c_d};

/// Scenario-declared canopy drag-area growth shape.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CanopyGrowthShape {
    /// Polynomial growth `xi(tau) = tau^exponent_j`.
    Polynomial {
        /// Positive fill-shape exponent.
        exponent_j: f64,
    },
    /// Exponential growth `xi(tau) = expm1(alpha*tau) / expm1(alpha)`.
    Exponential {
        /// Finite nonzero exponential shape parameter.
        alpha: f64,
    },
}

impl CanopyGrowthShape {
    /// Construct a polynomial growth shape.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] if `exponent_j` is not
    /// finite and strictly positive.
    pub fn polynomial(exponent_j: f64) -> Result<Self, RecoveryError> {
        if !exponent_j.is_finite() || exponent_j <= 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation polynomial exponent must be finite and strictly positive",
            });
        }
        Ok(Self::Polynomial { exponent_j })
    }

    /// Construct an exponential growth shape.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] if `alpha` is non-finite or
    /// exactly zero.
    pub fn exponential(alpha: f64) -> Result<Self, RecoveryError> {
        if !alpha.is_finite() || alpha == 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation exponential alpha must be finite and nonzero",
            });
        }
        Ok(Self::Exponential { alpha })
    }

    fn fraction(self, tau: f64) -> Result<f64, RecoveryError> {
        let tau = tau.clamp(0.0, 1.0);
        let fraction = match self {
            Self::Polynomial { exponent_j } => tau.powf(exponent_j),
            Self::Exponential { alpha } => (alpha * tau).exp_m1() / alpha.exp_m1(),
        };
        if !fraction.is_finite() || !(0.0..=1.0).contains(&fraction) {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation growth shape produced an invalid fill fraction",
            });
        }
        Ok(fraction)
    }

    fn fraction_tau_derivative(self, tau: f64) -> Result<f64, RecoveryError> {
        let tau = tau.clamp(0.0, 1.0);
        let derivative = match self {
            Self::Polynomial { exponent_j } => {
                if tau == 0.0 && exponent_j > 1.0 {
                    0.0
                } else {
                    exponent_j * tau.powf(exponent_j - 1.0)
                }
            }
            Self::Exponential { alpha } => alpha * (alpha * tau).exp() / alpha.exp_m1(),
        };
        if !derivative.is_finite() || derivative < 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation growth shape produced an invalid fill-fraction derivative",
            });
        }
        Ok(derivative)
    }
}

/// Parametric canopy inflation law for one parachute stage.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct InflationProfile {
    fill_time_s: f64,
    initial_drag_area_fraction: f64,
    growth: CanopyGrowthShape,
    opening_force_coeff_c_x: f64,
}

impl InflationProfile {
    /// Construct an inflation profile from already-derived fill time.
    ///
    /// Callers may pass a scenario-declared fill time directly or derive it
    /// first with [`canopy_fill_time_s`] when line-stretch speed context is
    /// available.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] if any scalar is outside
    /// its valid envelope.
    pub fn new(
        fill_time_s: f64,
        initial_drag_area_fraction: f64,
        growth: CanopyGrowthShape,
        opening_force_coeff_c_x: f64,
    ) -> Result<Self, RecoveryError> {
        if !fill_time_s.is_finite() || fill_time_s <= 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation fill_time_s must be finite and strictly positive",
            });
        }
        if !initial_drag_area_fraction.is_finite()
            || !(0.0..=1.0).contains(&initial_drag_area_fraction)
        {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation initial_drag_area_fraction must be finite and in [0, 1]",
            });
        }
        if !opening_force_coeff_c_x.is_finite() || opening_force_coeff_c_x <= 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation opening_force_coeff_c_x must be finite and strictly positive",
            });
        }
        Ok(Self {
            fill_time_s,
            initial_drag_area_fraction,
            growth,
            opening_force_coeff_c_x,
        })
    }

    /// Fill time, seconds.
    #[must_use]
    pub const fn fill_time_s(self) -> f64 {
        self.fill_time_s
    }

    /// Initial effective drag-area fraction.
    #[must_use]
    pub const fn initial_drag_area_fraction(self) -> f64 {
        self.initial_drag_area_fraction
    }

    /// Growth law.
    #[must_use]
    pub const fn growth(self) -> CanopyGrowthShape {
        self.growth
    }

    /// Infinite-mass opening-force coefficient.
    #[must_use]
    pub const fn opening_force_coeff_c_x(self) -> f64 {
        self.opening_force_coeff_c_x
    }

    /// Fill progress `tau = elapsed / fill_time`, clamped to `[0, 1]`.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] if `elapsed_s` is
    /// non-finite or negative.
    pub fn progress(self, elapsed_s: f64) -> Result<f64, RecoveryError> {
        if !elapsed_s.is_finite() || elapsed_s < 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation elapsed_s must be finite and non-negative",
            });
        }
        Ok((elapsed_s / self.fill_time_s).clamp(0.0, 1.0))
    }

    /// Effective drag-area fraction after the growth law is applied.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError`] for invalid elapsed time or growth output.
    pub fn drag_area_fraction(self, elapsed_s: f64) -> Result<f64, RecoveryError> {
        let tau = self.progress(elapsed_s)?;
        let growth_fraction = self.growth.fraction(tau)?;
        Ok(self.initial_drag_area_fraction
            + (1.0 - self.initial_drag_area_fraction) * growth_fraction)
    }

    /// Effective drag-area fraction rate, per second.
    ///
    /// The rate is zero before/after the declared fill interval. During fill,
    /// it is `d[(C_D S)/(C_D S)_full]/dt`.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError`] for invalid elapsed time or non-finite growth
    /// derivative output.
    pub fn drag_area_fraction_rate_s(self, elapsed_s: f64) -> Result<f64, RecoveryError> {
        if !elapsed_s.is_finite() || elapsed_s < 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation elapsed_s must be finite and non-negative",
            });
        }
        if elapsed_s >= self.fill_time_s {
            return Ok(0.0);
        }
        let tau = self.progress(elapsed_s)?;
        let rate = (1.0 - self.initial_drag_area_fraction)
            * self.growth.fraction_tau_derivative(tau)?
            / self.fill_time_s;
        if !rate.is_finite() || rate < 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation drag-area fraction rate is invalid",
            });
        }
        Ok(rate)
    }

    /// Evaluate a diagnostic sample for the current fill state.
    ///
    /// `dynamic_pressure_pa` is caller supplied. The returned
    /// `infinite_mass_peak_load_n` is the closed-form Knacke/Pflanz
    /// infinite-mass peak `C_D S_full * q * C_X`; it is not reduced by
    /// finite-mass deceleration.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError`] for invalid elapsed time, aerodynamic inputs,
    /// or dynamic pressure.
    pub fn sample(
        self,
        elapsed_s: f64,
        full_c_d: f64,
        full_drag_area_m2: f64,
        dynamic_pressure_pa: f64,
    ) -> Result<CanopyInflationSample, RecoveryError> {
        require_positive_c_d(full_c_d)?;
        require_positive_area(full_drag_area_m2)?;
        if !dynamic_pressure_pa.is_finite() || dynamic_pressure_pa < 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "inflation dynamic_pressure_pa must be finite and non-negative",
            });
        }
        let progress = self.progress(elapsed_s)?;
        let drag_area_fraction = self.drag_area_fraction(elapsed_s)?;
        let effective_drag_area_m2 = full_drag_area_m2 * drag_area_fraction;
        let effective_drag_area_rate_m2_s =
            full_drag_area_m2 * self.drag_area_fraction_rate_s(elapsed_s)?;
        let effective_c_d_s_m2 = full_c_d * effective_drag_area_m2;
        let full_c_d_s_m2 = full_c_d * full_drag_area_m2;
        Ok(CanopyInflationSample {
            elapsed_s: elapsed_s.min(self.fill_time_s),
            progress,
            drag_area_fraction,
            effective_drag_area_m2,
            effective_drag_area_rate_m2_s,
            effective_c_d_s_m2,
            infinite_mass_peak_load_n: full_c_d_s_m2
                * dynamic_pressure_pa
                * self.opening_force_coeff_c_x,
            instantaneous_infinite_mass_load_n: effective_c_d_s_m2
                * dynamic_pressure_pa
                * self.opening_force_coeff_c_x,
            complete: progress >= 1.0,
        })
    }
}

/// Derive canopy fill time from line-stretch speed using Knacke eq. 3.2.1.
///
/// `t_f = n * D0 / v_s^k`, where `n` is the canopy fill constant, `D0` is the
/// nominal canopy diameter, `v_s` is line-stretch speed, and `k` is the
/// empirical fill exponent.
///
/// # Errors
///
/// Returns [`RecoveryError::InvalidParameter`] if any input is outside the
/// equation envelope or the computed fill time is invalid.
pub fn canopy_fill_time_s(
    fill_constant_n: f64,
    nominal_diameter_m: f64,
    line_stretch_speed_m_s: f64,
    fill_exponent_k: f64,
) -> Result<f64, RecoveryError> {
    if !fill_constant_n.is_finite() || fill_constant_n <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "inflation fill constant n must be finite and strictly positive",
        });
    }
    if !nominal_diameter_m.is_finite() || nominal_diameter_m <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "inflation nominal diameter must be finite and strictly positive",
        });
    }
    if !line_stretch_speed_m_s.is_finite() || line_stretch_speed_m_s <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "inflation line-stretch speed must be finite and strictly positive",
        });
    }
    if !fill_exponent_k.is_finite() || fill_exponent_k <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "inflation fill exponent k must be finite and strictly positive",
        });
    }
    let fill_time_s =
        fill_constant_n * nominal_diameter_m / line_stretch_speed_m_s.powf(fill_exponent_k);
    if !fill_time_s.is_finite() || fill_time_s <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "inflation fill-time derivation produced an invalid value",
        });
    }
    Ok(fill_time_s)
}

/// Runtime state for one inflating canopy stage.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CanopyInflationState {
    profile: InflationProfile,
    elapsed_s: f64,
    started: bool,
}

impl CanopyInflationState {
    /// Construct a stowed inflation state.
    #[must_use]
    pub const fn new(profile: InflationProfile) -> Self {
        Self {
            profile,
            elapsed_s: 0.0,
            started: false,
        }
    }

    /// Start the fill clock from line stretch/deployment.
    pub fn start(&mut self) {
        self.elapsed_s = 0.0;
        self.started = true;
    }

    /// Advance the fill clock.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidStep`] when `dt_s` is not finite and
    /// strictly positive.
    pub fn step(&mut self, dt_s: f64) -> Result<(), RecoveryError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(RecoveryError::InvalidStep { dt_s });
        }
        if self.started && !self.complete() {
            self.elapsed_s = (self.elapsed_s + dt_s).min(self.profile.fill_time_s);
        }
        Ok(())
    }

    /// Inflation profile.
    #[must_use]
    pub const fn profile(self) -> InflationProfile {
        self.profile
    }

    /// Elapsed fill time.
    #[must_use]
    pub const fn elapsed_s(self) -> f64 {
        self.elapsed_s
    }

    /// Whether fill has been started.
    #[must_use]
    pub const fn started(self) -> bool {
        self.started
    }

    /// Whether the fill is complete.
    #[must_use]
    pub fn complete(self) -> bool {
        self.started && self.elapsed_s >= self.profile.fill_time_s
    }

    /// Current effective drag area for a full-open area.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError`] for invalid `full_drag_area_m2` or profile
    /// state.
    pub fn effective_drag_area_m2(self, full_drag_area_m2: f64) -> Result<f64, RecoveryError> {
        require_positive_area(full_drag_area_m2)?;
        if !self.started {
            return Ok(0.0);
        }
        Ok(full_drag_area_m2 * self.profile.drag_area_fraction(self.elapsed_s)?)
    }
}

/// Diagnostic output from one inflation-profile evaluation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CanopyInflationSample {
    /// Elapsed fill time, clamped to the configured fill time.
    pub elapsed_s: f64,
    /// Fill progress in `[0, 1]`.
    pub progress: f64,
    /// Effective drag-area fraction in `[initial, 1]`.
    pub drag_area_fraction: f64,
    /// Effective drag area at this fill state.
    pub effective_drag_area_m2: f64,
    /// Effective drag-area rate at this fill state.
    pub effective_drag_area_rate_m2_s: f64,
    /// Effective `C_D S` at this fill state.
    pub effective_c_d_s_m2: f64,
    /// Infinite-mass peak opening load for the full-open `C_D S`.
    pub infinite_mass_peak_load_n: f64,
    /// Instantaneous infinite-mass load for the current effective `C_D S`.
    pub instantaneous_infinite_mass_load_n: f64,
    /// Whether fill progress has saturated.
    pub complete: bool,
}

/// Projected canopy diameter implied by projected area `S = pi D^2 / 4`.
///
/// # Errors
///
/// Returns [`RecoveryError::InvalidParameter`] when `projected_area_m2` is
/// negative or non-finite.
pub fn canopy_projected_diameter_m(projected_area_m2: f64) -> Result<f64, RecoveryError> {
    if !projected_area_m2.is_finite() || projected_area_m2 < 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "canopy projected area must be finite and non-negative",
        });
    }
    let diameter_m = (4.0 * projected_area_m2 / std::f64::consts::PI).sqrt();
    if !diameter_m.is_finite() || diameter_m < 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "canopy projected diameter computation produced an invalid value",
        });
    }
    Ok(diameter_m)
}

/// Compute scalar canopy apparent mass from eq. 3.2.6.
///
/// # Errors
///
/// Returns [`RecoveryError::InvalidParameter`] when density, coefficient, or
/// projected area is outside the equation envelope.
pub fn canopy_apparent_mass_kg(
    density_kg_m3: f64,
    apparent_mass_coeff_k_a: f64,
    projected_area_m2: f64,
) -> Result<f64, RecoveryError> {
    if !density_kg_m3.is_finite() || density_kg_m3 < 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "apparent-mass density must be finite and non-negative",
        });
    }
    if !apparent_mass_coeff_k_a.is_finite() || apparent_mass_coeff_k_a <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "apparent-mass coefficient must be finite and strictly positive",
        });
    }
    let diameter_m = canopy_projected_diameter_m(projected_area_m2)?;
    let apparent_mass_kg =
        apparent_mass_coeff_k_a * density_kg_m3 * (std::f64::consts::PI / 6.0) * diameter_m.powi(3);
    if !apparent_mass_kg.is_finite() || apparent_mass_kg < 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "apparent-mass computation produced an invalid value",
        });
    }
    Ok(apparent_mass_kg)
}

/// Compute scalar canopy apparent-mass rate from projected-area rate.
///
/// Differentiating `m_a = k_a rho pi D^3 / 6` with
/// `S = pi D^2 / 4` gives `dm_a/dt = k_a rho D dS/dt`.
///
/// # Errors
///
/// Returns [`RecoveryError::InvalidParameter`] when inputs are outside the
/// equation envelope.
pub fn canopy_apparent_mass_rate_kg_s(
    density_kg_m3: f64,
    apparent_mass_coeff_k_a: f64,
    projected_area_m2: f64,
    projected_area_rate_m2_s: f64,
) -> Result<f64, RecoveryError> {
    if !projected_area_rate_m2_s.is_finite() || projected_area_rate_m2_s < 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "apparent-mass projected-area rate must be finite and non-negative",
        });
    }
    if !density_kg_m3.is_finite() || density_kg_m3 < 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "apparent-mass density must be finite and non-negative",
        });
    }
    if !apparent_mass_coeff_k_a.is_finite() || apparent_mass_coeff_k_a <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "apparent-mass coefficient must be finite and strictly positive",
        });
    }
    let diameter_m = canopy_projected_diameter_m(projected_area_m2)?;
    let apparent_mass_rate_kg_s =
        apparent_mass_coeff_k_a * density_kg_m3 * diameter_m * projected_area_rate_m2_s;
    if !apparent_mass_rate_kg_s.is_finite() || apparent_mass_rate_kg_s < 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "apparent-mass-rate computation produced an invalid value",
        });
    }
    Ok(apparent_mass_rate_kg_s)
}

/// Validated Pflanz finite-mass opening-load reduction table.
///
/// The table is indexed by ballistic parameter `A` (eq. 3.2.4) and polynomial
/// growth exponent `j`; values are the dimensionless force-reduction factor
/// `X1(A, j)` from eq. 3.2.5. Rows are stored in `exponent_j` order and
/// columns in `ballistic_parameter_a` order.
#[derive(Clone, Debug, PartialEq)]
pub struct PflanzReductionTable {
    ballistic_parameter_a: Vec<f64>,
    exponent_j: Vec<f64>,
    reduction_factors: Vec<f64>,
}

impl PflanzReductionTable {
    /// Construct a validated reduction-factor table.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] when axes are not finite,
    /// positive, strictly increasing, or when the reduction grid shape/values
    /// are invalid. Reduction factors must be finite, in `(0, 1]`, and
    /// nondecreasing with ballistic parameter for each exponent row.
    pub fn new(
        ballistic_parameter_a: Vec<f64>,
        exponent_j: Vec<f64>,
        reduction_factors: Vec<f64>,
    ) -> Result<Self, RecoveryError> {
        validate_axis(&ballistic_parameter_a, 2)?;
        validate_axis(&exponent_j, 1)?;
        let expected = ballistic_parameter_a
            .len()
            .checked_mul(exponent_j.len())
            .ok_or(RecoveryError::InvalidParameter {
                reason: "Pflanz reduction table shape overflowed",
            })?;
        if reduction_factors.len() != expected {
            return Err(RecoveryError::InvalidParameter {
                reason: "Pflanz reduction table must contain exponent_count * ballistic_parameter_count values",
            });
        }
        let a_count = ballistic_parameter_a.len();
        for row in reduction_factors.chunks_exact(a_count) {
            let mut previous = None;
            for &factor in row {
                if !factor.is_finite() || !(0.0..=1.0).contains(&factor) || factor == 0.0 {
                    return Err(RecoveryError::InvalidParameter {
                        reason: "Pflanz reduction factors must be finite and in (0, 1]",
                    });
                }
                if let Some(previous) = previous
                    && factor < previous
                {
                    return Err(RecoveryError::InvalidParameter {
                        reason: "Pflanz reduction factors must be nondecreasing with ballistic parameter",
                    });
                }
                previous = Some(factor);
            }
        }
        Ok(Self {
            ballistic_parameter_a,
            exponent_j,
            reduction_factors,
        })
    }

    /// Ballistic-parameter axis.
    #[must_use]
    pub fn ballistic_parameter_axis(&self) -> &[f64] {
        &self.ballistic_parameter_a
    }

    /// Polynomial-exponent axis.
    #[must_use]
    pub fn exponent_axis(&self) -> &[f64] {
        &self.exponent_j
    }

    /// Row-major reduction factors.
    #[must_use]
    pub fn reduction_factors(&self) -> &[f64] {
        &self.reduction_factors
    }

    /// Interpolate `X1(A, j)` with bilinear interpolation.
    ///
    /// Finite `A` and `j` queries must lie inside the declared table envelope.
    /// A positive infinite `A` returns `1.0` exactly, preserving the
    /// infinite-mass limit `X1 -> 1`.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] for non-finite or
    /// out-of-envelope inputs, except positive infinite `A`.
    pub fn reduction_factor(
        &self,
        ballistic_parameter_a: f64,
        exponent_j: f64,
    ) -> Result<f64, RecoveryError> {
        let _ = bracket_axis(&self.exponent_j, exponent_j)?;
        if ballistic_parameter_a.is_infinite() && ballistic_parameter_a.is_sign_positive() {
            return Ok(1.0);
        }
        if !ballistic_parameter_a.is_finite() || ballistic_parameter_a <= 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "Pflanz ballistic parameter query must be finite and strictly positive",
            });
        }
        let (a_lo, a_hi, alpha) = bracket_axis(&self.ballistic_parameter_a, ballistic_parameter_a)?;
        let (j_lo, j_hi, beta) = bracket_axis(&self.exponent_j, exponent_j)?;
        let f_j_lo = lerp(
            self.factor_at(j_lo, a_lo),
            self.factor_at(j_lo, a_hi),
            alpha,
        );
        let f_j_hi = lerp(
            self.factor_at(j_hi, a_lo),
            self.factor_at(j_hi, a_hi),
            alpha,
        );
        let factor = lerp(f_j_lo, f_j_hi, beta);
        if !factor.is_finite() || !(0.0..=1.0).contains(&factor) || factor == 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "Pflanz interpolation produced an invalid reduction factor",
            });
        }
        Ok(factor)
    }

    fn factor_at(&self, j_index: usize, a_index: usize) -> f64 {
        self.reduction_factors[j_index * self.ballistic_parameter_a.len() + a_index]
    }
}

/// Compute the Pflanz ballistic parameter `A` from eq. 3.2.4.
///
/// # Errors
///
/// Returns [`RecoveryError::InvalidParameter`] when any input is outside the
/// equation envelope.
pub fn pflanz_ballistic_parameter_a(
    system_mass_kg: f64,
    density_kg_m3: f64,
    full_c_d: f64,
    full_drag_area_m2: f64,
    line_stretch_speed_m_s: f64,
    fill_time_s: f64,
) -> Result<f64, RecoveryError> {
    if !system_mass_kg.is_finite() || system_mass_kg <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz system mass must be finite and strictly positive",
        });
    }
    if !density_kg_m3.is_finite() || density_kg_m3 <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz density must be finite and strictly positive",
        });
    }
    require_positive_c_d(full_c_d)?;
    require_positive_area(full_drag_area_m2)?;
    if !line_stretch_speed_m_s.is_finite() || line_stretch_speed_m_s <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz line-stretch speed must be finite and strictly positive",
        });
    }
    if !fill_time_s.is_finite() || fill_time_s <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz fill time must be finite and strictly positive",
        });
    }
    let full_c_d_s_m2 = full_c_d * full_drag_area_m2;
    let denominator = density_kg_m3 * full_c_d_s_m2 * line_stretch_speed_m_s * fill_time_s;
    let ballistic_parameter_a = (2.0 * system_mass_kg) / denominator;
    if !ballistic_parameter_a.is_finite() || ballistic_parameter_a <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz ballistic parameter computation produced an invalid value",
        });
    }
    Ok(ballistic_parameter_a)
}

/// Apply the Pflanz finite-mass reduction factor to an infinite-mass peak load.
///
/// # Errors
///
/// Returns [`RecoveryError::InvalidParameter`] when either input is outside
/// the equation envelope.
pub fn pflanz_finite_mass_peak_load_n(
    infinite_mass_peak_load_n: f64,
    reduction_factor_x1: f64,
) -> Result<f64, RecoveryError> {
    if !infinite_mass_peak_load_n.is_finite() || infinite_mass_peak_load_n < 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz infinite-mass peak load must be finite and non-negative",
        });
    }
    if !reduction_factor_x1.is_finite()
        || !(0.0..=1.0).contains(&reduction_factor_x1)
        || reduction_factor_x1 == 0.0
    {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz reduction factor must be finite and in (0, 1]",
        });
    }
    let load_n = infinite_mass_peak_load_n * reduction_factor_x1;
    if !load_n.is_finite() || load_n < 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz finite-mass peak load computation produced an invalid value",
        });
    }
    Ok(load_n)
}

fn validate_axis(values: &[f64], min_len: usize) -> Result<(), RecoveryError> {
    if values.len() < min_len {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz reduction table axis has too few points",
        });
    }
    let mut previous = None;
    for &value in values {
        if !value.is_finite() || value <= 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "Pflanz reduction table axes must be finite and strictly positive",
            });
        }
        if let Some(previous) = previous
            && value <= previous
        {
            return Err(RecoveryError::InvalidParameter {
                reason: "Pflanz reduction table axes must be strictly increasing",
            });
        }
        previous = Some(value);
    }
    Ok(())
}

fn bracket_axis(values: &[f64], query: f64) -> Result<(usize, usize, f64), RecoveryError> {
    if !query.is_finite() || query <= 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz table query must be finite and strictly positive",
        });
    }
    if values.len() == 1 {
        if query.to_bits() == values[0].to_bits() {
            return Ok((0, 0, 0.0));
        }
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz table query is outside the declared envelope",
        });
    }
    if query < values[0] || query > values[values.len() - 1] {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz table query is outside the declared envelope",
        });
    }
    let upper = values.partition_point(|value| *value < query);
    let (lo, hi) = if upper == 0 {
        (0, 1)
    } else if upper == values.len() {
        (values.len() - 2, values.len() - 1)
    } else {
        (upper - 1, upper)
    };
    let fraction = (query - values[lo]) / (values[hi] - values[lo]);
    if !fraction.is_finite() || !(0.0..=1.0).contains(&fraction) {
        return Err(RecoveryError::InvalidParameter {
            reason: "Pflanz table interpolation fraction is invalid",
        });
    }
    Ok((lo, hi, fraction))
}

fn lerp(lo: f64, hi: f64, fraction: f64) -> f64 {
    lo + (hi - lo) * fraction
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn polynomial_profile_ramps_drag_area_and_load() {
        let profile =
            InflationProfile::new(2.0, 0.1, CanopyGrowthShape::polynomial(2.0).unwrap(), 1.4)
                .unwrap();
        let sample = profile.sample(1.0, 1.5, 10.0, 500.0).unwrap();
        assert_eq!(sample.progress.to_bits(), 0.5_f64.to_bits());
        assert_eq!(sample.drag_area_fraction.to_bits(), 0.325_f64.to_bits());
        assert_eq!(sample.effective_drag_area_m2.to_bits(), 3.25_f64.to_bits());
        assert_eq!(
            sample.effective_drag_area_rate_m2_s.to_bits(),
            4.5_f64.to_bits()
        );
        assert_eq!(sample.effective_c_d_s_m2.to_bits(), 4.875_f64.to_bits());
        assert_eq!(
            sample.infinite_mass_peak_load_n.to_bits(),
            (1.5_f64 * 10.0 * 500.0 * 1.4).to_bits()
        );
        assert_eq!(
            sample.instantaneous_infinite_mass_load_n.to_bits(),
            (4.875_f64 * 500.0 * 1.4).to_bits()
        );
        assert!(!sample.complete);
    }

    #[test]
    fn exponential_profile_reaches_full_area_at_fill_time() {
        let profile =
            InflationProfile::new(1.0, 0.0, CanopyGrowthShape::exponential(3.0).unwrap(), 1.0)
                .unwrap();
        let sample = profile.sample(1.5, 2.0, 4.0, 100.0).unwrap();
        assert_eq!(sample.progress.to_bits(), 1.0_f64.to_bits());
        assert_eq!(sample.drag_area_fraction.to_bits(), 1.0_f64.to_bits());
        assert_eq!(sample.effective_drag_area_m2.to_bits(), 4.0_f64.to_bits());
        assert!(sample.complete);
    }

    #[test]
    fn state_stays_zero_until_started_then_saturates() {
        let profile =
            InflationProfile::new(0.3, 0.0, CanopyGrowthShape::polynomial(1.0).unwrap(), 1.0)
                .unwrap();
        let mut state = CanopyInflationState::new(profile);
        assert_eq!(
            state.effective_drag_area_m2(12.0).unwrap().to_bits(),
            0.0_f64.to_bits()
        );
        state.start();
        state.step(0.1).unwrap();
        assert_eq!(
            state.effective_drag_area_m2(12.0).unwrap().to_bits(),
            4.0_f64.to_bits()
        );
        state.step(1.0).unwrap();
        assert_eq!(
            state.effective_drag_area_m2(12.0).unwrap().to_bits(),
            12.0_f64.to_bits()
        );
        assert!(state.complete());
    }

    #[test]
    fn rejects_invalid_profile_inputs() {
        assert!(CanopyGrowthShape::polynomial(0.0).is_err());
        assert!(CanopyGrowthShape::exponential(0.0).is_err());
        assert!(
            InflationProfile::new(0.0, 0.0, CanopyGrowthShape::polynomial(1.0).unwrap(), 1.0)
                .is_err()
        );
        assert!(
            InflationProfile::new(1.0, 1.1, CanopyGrowthShape::polynomial(1.0).unwrap(), 1.0)
                .is_err()
        );
        assert!(
            InflationProfile::new(1.0, 0.0, CanopyGrowthShape::polynomial(1.0).unwrap(), 0.0)
                .is_err()
        );
    }

    #[test]
    fn canopy_fill_time_uses_line_stretch_equation() {
        let fill_time_s = canopy_fill_time_s(2.0, 4.0, 16.0, 0.5).unwrap();
        assert_eq!(fill_time_s.to_bits(), 2.0_f64.to_bits());
    }

    #[test]
    fn canopy_fill_time_rejects_invalid_inputs() {
        assert!(canopy_fill_time_s(0.0, 4.0, 16.0, 1.0).is_err());
        assert!(canopy_fill_time_s(2.0, 0.0, 16.0, 1.0).is_err());
        assert!(canopy_fill_time_s(2.0, 4.0, 0.0, 1.0).is_err());
        assert!(canopy_fill_time_s(2.0, 4.0, 16.0, 0.0).is_err());
    }

    #[test]
    fn scalar_apparent_mass_matches_equations() {
        let diameter = canopy_projected_diameter_m(std::f64::consts::PI).unwrap();
        assert_eq!(diameter.to_bits(), 2.0_f64.to_bits());
        let apparent_mass = canopy_apparent_mass_kg(1.2, 0.5, std::f64::consts::PI).unwrap();
        let expected_apparent_mass = 0.5_f64 * 1.2 * std::f64::consts::PI / 6.0 * 8.0;
        assert!((apparent_mass - expected_apparent_mass).abs() < 1.0e-15);
        let apparent_mass_rate =
            canopy_apparent_mass_rate_kg_s(1.2, 0.5, std::f64::consts::PI, 3.0).unwrap();
        assert_eq!(
            apparent_mass_rate.to_bits(),
            (0.5_f64 * 1.2 * 2.0 * 3.0).to_bits()
        );
    }

    #[test]
    fn scalar_apparent_mass_rejects_invalid_inputs() {
        assert!(canopy_projected_diameter_m(-1.0).is_err());
        assert!(canopy_apparent_mass_kg(-1.0, 0.5, 1.0).is_err());
        assert!(canopy_apparent_mass_kg(1.0, 0.0, 1.0).is_err());
        assert!(canopy_apparent_mass_kg(1.0, 0.5, -1.0).is_err());
        assert!(canopy_apparent_mass_rate_kg_s(1.0, 0.5, 1.0, -1.0).is_err());
    }

    #[test]
    fn pflanz_ballistic_parameter_matches_equation() {
        let ballistic_parameter_a =
            pflanz_ballistic_parameter_a(100.0, 1.0, 1.5, 10.0, 20.0, 2.0).unwrap();
        assert_eq!(ballistic_parameter_a.to_bits(), (1.0_f64 / 3.0).to_bits());
    }

    #[test]
    fn pflanz_table_interpolates_and_preserves_infinite_mass_limit() {
        let table = PflanzReductionTable::new(
            vec![1.0, 3.0],
            vec![2.0, 4.0],
            vec![
                0.5, 0.7, // j = 2
                0.4, 0.6, // j = 4
            ],
        )
        .unwrap();

        let factor = table.reduction_factor(2.0, 3.0).unwrap();
        assert!((factor - 0.55).abs() < 1.0e-15);
        assert_eq!(
            table
                .reduction_factor(f64::INFINITY, 3.0)
                .unwrap()
                .to_bits(),
            1.0_f64.to_bits()
        );
        let finite_mass = pflanz_finite_mass_peak_load_n(1000.0, factor).unwrap();
        assert!((finite_mass - 550.0).abs() < 1.0e-12);
    }

    #[test]
    fn pflanz_table_rejects_invalid_axes_values_and_queries() {
        assert!(PflanzReductionTable::new(vec![1.0], vec![2.0], vec![0.5]).is_err());
        assert!(PflanzReductionTable::new(vec![1.0, 1.0], vec![2.0], vec![0.5, 0.6]).is_err());
        assert!(PflanzReductionTable::new(vec![1.0, 2.0], vec![2.0], vec![0.6, 0.5]).is_err());
        assert!(PflanzReductionTable::new(vec![1.0, 2.0], vec![2.0], vec![0.0, 0.5]).is_err());

        let table = PflanzReductionTable::new(vec![1.0, 2.0], vec![2.0], vec![0.5, 0.6]).unwrap();
        assert!(table.reduction_factor(0.5, 2.0).is_err());
        assert!(table.reduction_factor(1.5, 3.0).is_err());
        assert!(pflanz_ballistic_parameter_a(0.0, 1.0, 1.5, 10.0, 20.0, 2.0).is_err());
        assert!(pflanz_finite_mass_peak_load_n(100.0, 0.0).is_err());
    }
}
