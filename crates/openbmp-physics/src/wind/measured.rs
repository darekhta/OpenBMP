//! Measured day-of-launch wind profile handling.
//!
//! This module is the T1 day-of-launch wind front door: measured
//! profiles are validated, screened, spliced, filtered, and synthetic
//! profile families can be generated for deterministic tests. It is
//! intentionally offline/data-prep oriented. Runtime wind dispatch
//! continues to use [`super::WindModel`] implementations.

use std::fmt::Write as _;

use openbmp_core::{DeterministicRng, Ned, Velocity3};

use crate::error::PhysicsError;

const TWO_PI: f64 = 2.0 * core::f64::consts::PI;

/// Source instrument or generator for a measured wind profile.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum WindSource {
    /// Balloon sounding.
    Balloon,
    /// Doppler radar wind profiler.
    DopplerProfiler,
    /// Lidar wind profiler.
    Lidar,
    /// Climatological or forecast model source used as an upper band.
    Climatology,
    /// Deterministic in-repository synthetic source.
    Synthetic,
}

impl WindSource {
    /// Stable source label for deterministic artifacts.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Balloon => "balloon",
            Self::DopplerProfiler => "doppler_profiler",
            Self::Lidar => "lidar",
            Self::Climatology => "climatology",
            Self::Synthetic => "synthetic",
        }
    }
}

/// Provenance metadata for one measured profile.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WindProfileProvenance {
    /// Stable provenance identifier.
    pub id: String,
    /// Human-readable URI, path, or LOCAL marker.
    pub uri: String,
    /// Optional lower-case SHA-256 pin for committed deterministic input.
    pub sha256: Option<String>,
    /// Whether the source is LOCAL-only and must not be committed as data.
    pub local_only: bool,
}

impl WindProfileProvenance {
    /// Construct provenance metadata.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when `id` or `uri` is empty,
    /// or when a SHA-256 pin is present but not 64 lower-case hex characters.
    pub fn new(
        id: impl Into<String>,
        uri: impl Into<String>,
        sha256: Option<String>,
        local_only: bool,
    ) -> Result<Self, PhysicsError> {
        let provenance = Self {
            id: id.into(),
            uri: uri.into(),
            sha256,
            local_only,
        };
        if provenance.id.trim().is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "wind profile provenance id must not be empty",
            });
        }
        if provenance.uri.trim().is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "wind profile provenance uri must not be empty",
            });
        }
        if let Some(pin) = &provenance.sha256
            && (pin.len() != 64 || !pin.bytes().all(|b| b.is_ascii_hexdigit()))
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "wind profile SHA-256 pin must be 64 hex characters",
            });
        }
        Ok(provenance)
    }
}

/// One altitude-indexed wind sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MeasuredWindSample {
    /// Altitude above the launch-pad reference, metres.
    pub altitude_m: f64,
    /// Local-NED wind vector, m/s.
    pub wind_ned_m_s: Velocity3<Ned>,
    /// Dimensionless sample quality in `[0, 1]`.
    pub quality_unit: f64,
}

impl MeasuredWindSample {
    /// Construct a sample from explicit components.
    #[must_use]
    pub fn new(
        altitude_m: f64,
        north_m_s: f64,
        east_m_s: f64,
        down_m_s: f64,
        quality_unit: f64,
    ) -> Self {
        Self {
            altitude_m,
            wind_ned_m_s: Velocity3::new(north_m_s, east_m_s, down_m_s),
            quality_unit,
        }
    }
}

/// One measured wind profile from a single source.
#[derive(Clone, Debug, PartialEq)]
pub struct MeasuredWindProfile {
    /// Source instrument or generator.
    pub source: WindSource,
    /// Measurement validity anchor, seconds since the scenario epoch.
    pub released_at_s: f64,
    /// Altitude-ordered samples.
    pub samples: Vec<MeasuredWindSample>,
    /// Profile provenance.
    pub provenance: WindProfileProvenance,
}

impl MeasuredWindProfile {
    /// Construct a measured profile. Samples must be finite and strictly
    /// ascending in altitude; QC thresholds are applied separately.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the profile shape is invalid.
    pub fn new(
        source: WindSource,
        released_at_s: f64,
        samples: Vec<MeasuredWindSample>,
        provenance: WindProfileProvenance,
    ) -> Result<Self, PhysicsError> {
        if !released_at_s.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "MeasuredWindProfile release time is NaN or infinite",
            });
        }
        validate_samples(&samples)?;
        Ok(Self {
            source,
            released_at_s,
            samples,
            provenance,
        })
    }

    /// Altitude of the first sample.
    #[must_use]
    pub fn min_altitude_m(&self) -> f64 {
        self.samples[0].altitude_m
    }

    /// Altitude of the last sample.
    #[must_use]
    pub fn max_altitude_m(&self) -> f64 {
        self.samples[self.samples.len() - 1].altitude_m
    }

    /// Interpolate wind at `altitude_m` inside the profile envelope.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when `altitude_m` is outside
    /// the measured envelope.
    pub fn interpolate_wind_ned_m_s(
        &self,
        altitude_m: f64,
    ) -> Result<Velocity3<Ned>, PhysicsError> {
        interpolate_samples(&self.samples, altitude_m)
    }

    /// Run fail-closed quality-control screens.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when any QC screen rejects the profile.
    pub fn quality_control(
        &self,
        config: &WindQcConfig,
        evaluated_at_s: f64,
    ) -> Result<WindQcReport, PhysicsError> {
        config.validate()?;
        if !evaluated_at_s.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "wind QC evaluation time is NaN or infinite",
            });
        }
        let age_s = evaluated_at_s - self.released_at_s;
        if !age_s.is_finite() || age_s < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "wind QC evaluation time precedes profile release",
            });
        }
        if age_s > config.max_staleness_s {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "wind profile is stale",
            });
        }

        let mut max_gap_m = 0.0;
        let mut max_shear_per_s = 0.0;
        let mut max_speed_m_s = 0.0;

        for sample in &self.samples {
            if sample.altitude_m < config.min_altitude_m
                || sample.altitude_m > config.max_altitude_m
            {
                return Err(PhysicsError::OutOfEnvelope {
                    reason: "wind sample altitude is outside QC envelope",
                });
            }
            if sample.quality_unit < config.min_quality_unit {
                return Err(PhysicsError::OutOfEnvelope {
                    reason: "wind sample quality is below QC threshold",
                });
            }
            let speed = vector_norm(sample.wind_ned_m_s);
            if speed > config.max_speed_m_s {
                return Err(PhysicsError::OutOfEnvelope {
                    reason: "wind sample speed exceeds QC threshold",
                });
            }
            if speed > max_speed_m_s {
                max_speed_m_s = speed;
            }
        }

        for window in self.samples.windows(2) {
            let lo = window[0];
            let hi = window[1];
            let dz_m = hi.altitude_m - lo.altitude_m;
            if dz_m > config.max_gap_m {
                return Err(PhysicsError::OutOfEnvelope {
                    reason: "wind profile gap exceeds QC threshold",
                });
            }
            if dz_m > max_gap_m {
                max_gap_m = dz_m;
            }
            let shear = vector_norm(vector_sub(hi.wind_ned_m_s, lo.wind_ned_m_s)) / dz_m;
            if shear > config.max_shear_per_s {
                return Err(PhysicsError::OutOfEnvelope {
                    reason: "wind profile shear exceeds QC threshold",
                });
            }
            if shear > max_shear_per_s {
                max_shear_per_s = shear;
            }
        }

        Ok(WindQcReport {
            source: self.source,
            sample_count: self.samples.len(),
            age_s,
            max_gap_m,
            max_shear_per_s,
            max_speed_m_s,
        })
    }
}

/// Fail-closed QC thresholds for measured profiles.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WindQcConfig {
    /// Minimum allowed sample altitude, metres.
    pub min_altitude_m: f64,
    /// Maximum allowed sample altitude, metres.
    pub max_altitude_m: f64,
    /// Maximum adjacent-sample altitude gap, metres.
    pub max_gap_m: f64,
    /// Maximum adjacent wind-vector shear, `(m/s)/m == 1/s`.
    pub max_shear_per_s: f64,
    /// Maximum wind-vector magnitude, m/s.
    pub max_speed_m_s: f64,
    /// Minimum accepted sample quality, dimensionless `[0, 1]`.
    pub min_quality_unit: f64,
    /// Maximum age between profile release and use, seconds.
    pub max_staleness_s: f64,
}

impl Default for WindQcConfig {
    fn default() -> Self {
        Self {
            min_altitude_m: -500.0,
            max_altitude_m: 90_000.0,
            max_gap_m: 3_000.0,
            max_shear_per_s: 0.03,
            max_speed_m_s: 150.0,
            min_quality_unit: 0.0,
            max_staleness_s: 18_000.0,
        }
    }
}

impl WindQcConfig {
    fn validate(&self) -> Result<(), PhysicsError> {
        if !self.min_altitude_m.is_finite()
            || !self.max_altitude_m.is_finite()
            || !self.max_gap_m.is_finite()
            || !self.max_shear_per_s.is_finite()
            || !self.max_speed_m_s.is_finite()
            || !self.min_quality_unit.is_finite()
            || !self.max_staleness_s.is_finite()
        {
            return Err(PhysicsError::NonFinite {
                reason: "wind QC config contains NaN or infinite value",
            });
        }
        if self.max_altitude_m <= self.min_altitude_m {
            return Err(PhysicsError::InvalidParameter {
                reason: "wind QC altitude envelope must have positive span",
            });
        }
        if self.max_gap_m <= 0.0
            || self.max_shear_per_s < 0.0
            || self.max_speed_m_s <= 0.0
            || self.max_staleness_s < 0.0
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "wind QC thresholds must be positive or non-negative as declared",
            });
        }
        if !(0.0..=1.0).contains(&self.min_quality_unit) {
            return Err(PhysicsError::InvalidParameter {
                reason: "wind QC minimum quality must be in [0, 1]",
            });
        }
        Ok(())
    }
}

/// QC summary for an accepted measured profile.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WindQcReport {
    /// Profile source.
    pub source: WindSource,
    /// Number of samples screened.
    pub sample_count: usize,
    /// Profile age at evaluation time, seconds.
    pub age_s: f64,
    /// Maximum accepted adjacent altitude gap, metres.
    pub max_gap_m: f64,
    /// Maximum accepted adjacent wind-vector shear, `(m/s)/m == 1/s`.
    pub max_shear_per_s: f64,
    /// Maximum accepted wind-vector magnitude, m/s.
    pub max_speed_m_s: f64,
}

/// Declared overlap rule for two-source wind splicing.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpliceRule {
    /// Lower edge of the blend band, metres.
    pub overlap_lower_altitude_m: f64,
    /// Upper edge of the blend band, metres.
    pub overlap_upper_altitude_m: f64,
}

impl SpliceRule {
    /// Construct a splice rule.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the overlap band is not finite and
    /// strictly increasing.
    pub fn new(
        overlap_lower_altitude_m: f64,
        overlap_upper_altitude_m: f64,
    ) -> Result<Self, PhysicsError> {
        let rule = Self {
            overlap_lower_altitude_m,
            overlap_upper_altitude_m,
        };
        rule.validate()?;
        Ok(rule)
    }

    fn validate(&self) -> Result<(), PhysicsError> {
        if !self.overlap_lower_altitude_m.is_finite() || !self.overlap_upper_altitude_m.is_finite()
        {
            return Err(PhysicsError::NonFinite {
                reason: "wind splice overlap altitude is NaN or infinite",
            });
        }
        if self.overlap_upper_altitude_m <= self.overlap_lower_altitude_m {
            return Err(PhysicsError::InvalidParameter {
                reason: "wind splice overlap band must be strictly increasing",
            });
        }
        Ok(())
    }

    fn upper_weight(self, altitude_m: f64) -> f64 {
        if altitude_m <= self.overlap_lower_altitude_m {
            0.0
        } else if altitude_m >= self.overlap_upper_altitude_m {
            1.0
        } else {
            (altitude_m - self.overlap_lower_altitude_m)
                / (self.overlap_upper_altitude_m - self.overlap_lower_altitude_m)
        }
    }
}

/// One sample in a spliced profile with source attribution.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SplicedWindSample {
    /// Altitude above the launch-pad reference, metres.
    pub altitude_m: f64,
    /// Blended local-NED wind vector, m/s.
    pub wind_ned_m_s: Velocity3<Ned>,
    /// Weight assigned to the lower-band source, dimensionless.
    pub lower_weight_unit: f64,
    /// Weight assigned to the upper-band source, dimensionless.
    pub upper_weight_unit: f64,
}

/// Deterministic two-source spliced wind profile.
#[derive(Clone, Debug, PartialEq)]
pub struct SplicedWindProfile {
    /// Lower-band source.
    pub lower_source: WindSource,
    /// Upper-band source.
    pub upper_source: WindSource,
    /// Altitude-ordered spliced samples.
    pub samples: Vec<SplicedWindSample>,
}

impl SplicedWindProfile {
    /// Build a spliced profile from two measured profiles.
    ///
    /// The lower source is used below the overlap band, the upper source
    /// above it, and a fixed linear blend inside the band. The output grid is
    /// the deterministic union of lower-source samples up to the band top and
    /// upper-source samples from the band bottom upward.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the overlap band is not covered by both
    /// profiles or interpolation would leave a source envelope.
    pub fn from_profiles(
        lower_profile: &MeasuredWindProfile,
        upper_profile: &MeasuredWindProfile,
        rule: SpliceRule,
    ) -> Result<Self, PhysicsError> {
        rule.validate()?;
        if rule.overlap_lower_altitude_m < lower_profile.min_altitude_m()
            || rule.overlap_upper_altitude_m > lower_profile.max_altitude_m()
            || rule.overlap_lower_altitude_m < upper_profile.min_altitude_m()
            || rule.overlap_upper_altitude_m > upper_profile.max_altitude_m()
        {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "wind splice overlap is not covered by both profiles",
            });
        }

        let mut altitudes_m = Vec::new();
        for sample in &lower_profile.samples {
            if sample.altitude_m <= rule.overlap_upper_altitude_m {
                altitudes_m.push(sample.altitude_m);
            }
        }
        for sample in &upper_profile.samples {
            if sample.altitude_m >= rule.overlap_lower_altitude_m {
                altitudes_m.push(sample.altitude_m);
            }
        }
        altitudes_m.sort_by(f64::total_cmp);
        altitudes_m.dedup_by(|a, b| a.to_bits() == b.to_bits());
        if altitudes_m.is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "wind splice output grid is empty",
            });
        }

        let mut samples = Vec::with_capacity(altitudes_m.len());
        for altitude_m in altitudes_m {
            let upper_weight_unit = rule.upper_weight(altitude_m);
            let lower_weight_unit = 1.0 - upper_weight_unit;
            let wind = if upper_weight_unit == 0.0 {
                lower_profile.interpolate_wind_ned_m_s(altitude_m)?
            } else if lower_weight_unit == 0.0 {
                upper_profile.interpolate_wind_ned_m_s(altitude_m)?
            } else {
                let lower = lower_profile.interpolate_wind_ned_m_s(altitude_m)?;
                let upper = upper_profile.interpolate_wind_ned_m_s(altitude_m)?;
                vector_add(
                    vector_scale(lower, lower_weight_unit),
                    vector_scale(upper, upper_weight_unit),
                )
            };
            samples.push(SplicedWindSample {
                altitude_m,
                wind_ned_m_s: wind,
                lower_weight_unit,
                upper_weight_unit,
            });
        }

        Ok(Self {
            lower_source: lower_profile.source,
            upper_source: upper_profile.source,
            samples,
        })
    }

    /// Render a deterministic CSV artifact with per-sample attribution.
    #[must_use]
    pub fn to_csv(&self) -> String {
        let mut out = String::from(
            "altitude_m,wind_north_m_s,wind_east_m_s,wind_down_m_s,lower_source,lower_weight_unit,upper_source,upper_weight_unit\n",
        );
        for sample in &self.samples {
            let _ = writeln!(
                out,
                "{:.9},{:.9},{:.9},{:.9},{},{:.9},{},{:.9}",
                canonical_zero(sample.altitude_m),
                canonical_zero(sample.wind_ned_m_s.vector.x),
                canonical_zero(sample.wind_ned_m_s.vector.y),
                canonical_zero(sample.wind_ned_m_s.vector.z),
                self.lower_source.as_str(),
                canonical_zero(sample.lower_weight_unit),
                self.upper_source.as_str(),
                canonical_zero(sample.upper_weight_unit),
            );
        }
        out
    }
}

/// Martin-Graham-style first-order altitude-domain low-pass filter settings.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MartinGrahamFilterConfig {
    /// Spatial cutoff wavelength, metres. A sinusoid at this wavelength has
    /// magnitude response `1 / sqrt(2)`.
    pub cutoff_wavelength_m: f64,
}

impl MartinGrahamFilterConfig {
    /// Construct filter settings.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the cutoff is not finite and positive.
    pub fn new(cutoff_wavelength_m: f64) -> Result<Self, PhysicsError> {
        if !cutoff_wavelength_m.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "Martin-Graham cutoff wavelength is NaN or infinite",
            });
        }
        if cutoff_wavelength_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Martin-Graham cutoff wavelength must be positive",
            });
        }
        Ok(Self {
            cutoff_wavelength_m,
        })
    }

    /// Closed-form first-order low-pass magnitude response for a spatial
    /// sinusoid with wavelength `wavelength_m`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when `wavelength_m` is not finite and
    /// positive.
    pub fn magnitude_response(&self, wavelength_m: f64) -> Result<f64, PhysicsError> {
        if !wavelength_m.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "Martin-Graham response wavelength is NaN or infinite",
            });
        }
        if wavelength_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Martin-Graham response wavelength must be positive",
            });
        }
        let ratio = self.cutoff_wavelength_m / wavelength_m;
        Ok(1.0 / (1.0 + ratio * ratio).sqrt())
    }

    /// Apply the first-order altitude-domain low-pass filter to a spliced
    /// profile. Source weights and altitudes are preserved.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the profile is empty or altitudes are not
    /// strictly ascending.
    pub fn filter_spliced(
        &self,
        profile: &SplicedWindProfile,
    ) -> Result<SplicedWindProfile, PhysicsError> {
        if profile.samples.is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "Martin-Graham filter requires a nonempty profile",
            });
        }
        let length_m = self.cutoff_wavelength_m / TWO_PI;
        let mut filtered = Vec::with_capacity(profile.samples.len());
        let mut previous = profile.samples[0];
        filtered.push(previous);
        for sample in profile.samples.iter().skip(1) {
            let dz_m = sample.altitude_m - previous.altitude_m;
            if !dz_m.is_finite() || dz_m <= 0.0 {
                return Err(PhysicsError::InvalidParameter {
                    reason: "Martin-Graham filter requires strictly ascending altitudes",
                });
            }
            let alpha = 1.0 - (-dz_m / length_m).exp();
            let wind_ned_m_s = vector_add(
                previous.wind_ned_m_s,
                vector_scale(
                    vector_sub(sample.wind_ned_m_s, previous.wind_ned_m_s),
                    alpha,
                ),
            );
            previous = SplicedWindSample {
                altitude_m: sample.altitude_m,
                wind_ned_m_s,
                lower_weight_unit: sample.lower_weight_unit,
                upper_weight_unit: sample.upper_weight_unit,
            };
            filtered.push(previous);
        }
        Ok(SplicedWindProfile {
            lower_source: profile.lower_source,
            upper_source: profile.upper_source,
            samples: filtered,
        })
    }
}

/// Seeded synthetic balloon-family generator settings.
#[derive(Clone, Debug, PartialEq)]
pub struct SyntheticWindFamilyConfig {
    /// Campaign seed.
    pub seed: u64,
    /// Number of profiles to generate.
    pub member_count: usize,
    /// Measurement release time, seconds since scenario epoch.
    pub released_at_s: f64,
    /// Minimum altitude, metres.
    pub altitude_min_m: f64,
    /// Maximum altitude, metres.
    pub altitude_max_m: f64,
    /// Altitude spacing, metres.
    pub altitude_step_m: f64,
    /// Base local-NED wind vector, m/s.
    pub base_wind_ned_m_s: Velocity3<Ned>,
    /// Linear shear around `altitude_min_m`, `(m/s)/m == 1/s`.
    pub shear_ned_per_s: Velocity3<Ned>,
    /// Sinusoidal perturbation amplitude, m/s.
    pub amplitude_ned_m_s: Velocity3<Ned>,
}

impl SyntheticWindFamilyConfig {
    /// Generate deterministic synthetic measured profiles.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when generator settings are invalid.
    pub fn generate(&self) -> Result<Vec<MeasuredWindProfile>, PhysicsError> {
        self.validate()?;
        let sample_count = ((self.altitude_max_m - self.altitude_min_m) / self.altitude_step_m)
            .floor() as usize
            + 1;
        let mut profiles = Vec::with_capacity(self.member_count);
        for member in 0..self.member_count {
            let mut rng_n = DeterministicRng::for_mc_sample(self.seed, member as u64, 0);
            let mut rng_e = DeterministicRng::for_mc_sample(self.seed, member as u64, 1);
            let mut rng_d = DeterministicRng::for_mc_sample(self.seed, member as u64, 2);
            let phase_n = TWO_PI * unit_interval(rng_n.next_u64());
            let phase_e = TWO_PI * unit_interval(rng_e.next_u64());
            let phase_d = TWO_PI * unit_interval(rng_d.next_u64());
            let amp_scale_n = 0.5 + unit_interval(rng_n.next_u64());
            let amp_scale_e = 0.5 + unit_interval(rng_e.next_u64());
            let amp_scale_d = 0.5 + unit_interval(rng_d.next_u64());

            let mut samples = Vec::with_capacity(sample_count);
            for index in 0..sample_count {
                let altitude_m = self.altitude_min_m + self.altitude_step_m * index as f64;
                let dz_m = altitude_m - self.altitude_min_m;
                let wave = altitude_m / 2_500.0;
                let wind = Velocity3::new(
                    self.base_wind_ned_m_s.vector.x
                        + self.shear_ned_per_s.vector.x * dz_m
                        + self.amplitude_ned_m_s.vector.x * amp_scale_n * (wave + phase_n).sin(),
                    self.base_wind_ned_m_s.vector.y
                        + self.shear_ned_per_s.vector.y * dz_m
                        + self.amplitude_ned_m_s.vector.y * amp_scale_e * (wave + phase_e).sin(),
                    self.base_wind_ned_m_s.vector.z
                        + self.shear_ned_per_s.vector.z * dz_m
                        + self.amplitude_ned_m_s.vector.z * amp_scale_d * (wave + phase_d).sin(),
                );
                samples.push(MeasuredWindSample {
                    altitude_m,
                    wind_ned_m_s: wind,
                    quality_unit: 1.0,
                });
            }

            let provenance = WindProfileProvenance::new(
                format!("synthetic-balloon-family-v1:member-{member:04}"),
                "data/wind/synthetic-balloon-family-v1.toml",
                None,
                false,
            )?;
            profiles.push(MeasuredWindProfile::new(
                WindSource::Synthetic,
                self.released_at_s,
                samples,
                provenance,
            )?);
        }
        Ok(profiles)
    }

    fn validate(&self) -> Result<(), PhysicsError> {
        if self.member_count == 0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "synthetic wind family member_count must be positive",
            });
        }
        if !self.released_at_s.is_finite()
            || !self.altitude_min_m.is_finite()
            || !self.altitude_max_m.is_finite()
            || !self.altitude_step_m.is_finite()
            || !is_finite_vector(self.base_wind_ned_m_s)
            || !is_finite_vector(self.shear_ned_per_s)
            || !is_finite_vector(self.amplitude_ned_m_s)
        {
            return Err(PhysicsError::NonFinite {
                reason: "synthetic wind family config contains NaN or infinite value",
            });
        }
        if self.altitude_max_m <= self.altitude_min_m || self.altitude_step_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "synthetic wind family altitude grid must be strictly increasing",
            });
        }
        if self.altitude_step_m > self.altitude_max_m - self.altitude_min_m {
            return Err(PhysicsError::InvalidParameter {
                reason: "synthetic wind family altitude step exceeds altitude span",
            });
        }
        Ok(())
    }
}

fn validate_samples(samples: &[MeasuredWindSample]) -> Result<(), PhysicsError> {
    if samples.len() < 2 {
        return Err(PhysicsError::InvalidParameter {
            reason: "MeasuredWindProfile requires at least two samples",
        });
    }
    for sample in samples {
        if !sample.altitude_m.is_finite()
            || !is_finite_vector(sample.wind_ned_m_s)
            || !sample.quality_unit.is_finite()
        {
            return Err(PhysicsError::NonFinite {
                reason: "MeasuredWindProfile sample contains NaN or infinite value",
            });
        }
        if !(0.0..=1.0).contains(&sample.quality_unit) {
            return Err(PhysicsError::InvalidParameter {
                reason: "MeasuredWindProfile sample quality must be in [0, 1]",
            });
        }
    }
    for window in samples.windows(2) {
        if window[1].altitude_m <= window[0].altitude_m {
            return Err(PhysicsError::InvalidParameter {
                reason: "MeasuredWindProfile altitudes must be strictly ascending",
            });
        }
    }
    Ok(())
}

fn interpolate_samples(
    samples: &[MeasuredWindSample],
    altitude_m: f64,
) -> Result<Velocity3<Ned>, PhysicsError> {
    if !altitude_m.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "measured wind interpolation altitude is NaN or infinite",
        });
    }
    if altitude_m < samples[0].altitude_m || altitude_m > samples[samples.len() - 1].altitude_m {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "measured wind interpolation altitude is outside profile envelope",
        });
    }
    if altitude_m.to_bits() == samples[0].altitude_m.to_bits() {
        return Ok(samples[0].wind_ned_m_s);
    }
    for window in samples.windows(2) {
        let lo = window[0];
        let hi = window[1];
        if altitude_m >= lo.altitude_m && altitude_m <= hi.altitude_m {
            if altitude_m.to_bits() == hi.altitude_m.to_bits() {
                return Ok(hi.wind_ned_m_s);
            }
            let t = (altitude_m - lo.altitude_m) / (hi.altitude_m - lo.altitude_m);
            return Ok(vector_add(
                lo.wind_ned_m_s,
                vector_scale(vector_sub(hi.wind_ned_m_s, lo.wind_ned_m_s), t),
            ));
        }
    }
    Ok(samples[samples.len() - 1].wind_ned_m_s)
}

fn is_finite_vector(vector: Velocity3<Ned>) -> bool {
    vector.vector.x.is_finite() && vector.vector.y.is_finite() && vector.vector.z.is_finite()
}

fn vector_norm(vector: Velocity3<Ned>) -> f64 {
    (vector.vector.x * vector.vector.x
        + vector.vector.y * vector.vector.y
        + vector.vector.z * vector.vector.z)
        .sqrt()
}

fn vector_add(lhs: Velocity3<Ned>, rhs: Velocity3<Ned>) -> Velocity3<Ned> {
    Velocity3::new(
        lhs.vector.x + rhs.vector.x,
        lhs.vector.y + rhs.vector.y,
        lhs.vector.z + rhs.vector.z,
    )
}

fn vector_sub(lhs: Velocity3<Ned>, rhs: Velocity3<Ned>) -> Velocity3<Ned> {
    Velocity3::new(
        lhs.vector.x - rhs.vector.x,
        lhs.vector.y - rhs.vector.y,
        lhs.vector.z - rhs.vector.z,
    )
}

fn vector_scale(vector: Velocity3<Ned>, scale: f64) -> Velocity3<Ned> {
    Velocity3::new(
        vector.vector.x * scale,
        vector.vector.y * scale,
        vector.vector.z * scale,
    )
}

fn canonical_zero(value: f64) -> f64 {
    if value == 0.0 { 0.0 } else { value }
}

fn unit_interval(value: u64) -> f64 {
    const SCALE: f64 = 1.0 / ((1u64 << 53) as f64);
    ((value >> 11) as f64) * SCALE
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn provenance() -> WindProfileProvenance {
        WindProfileProvenance::new("synthetic-test", "inline://test", None, false).unwrap()
    }

    fn profile(samples: Vec<MeasuredWindSample>) -> MeasuredWindProfile {
        MeasuredWindProfile::new(WindSource::Balloon, 10.0, samples, provenance()).unwrap()
    }

    fn qc_config() -> WindQcConfig {
        WindQcConfig {
            min_altitude_m: 0.0,
            max_altitude_m: 10_000.0,
            max_gap_m: 2_000.0,
            max_shear_per_s: 0.02,
            max_speed_m_s: 100.0,
            min_quality_unit: 0.8,
            max_staleness_s: 1_000.0,
        }
    }

    #[test]
    fn qc_reports_hand_computed_maxima() {
        let profile = profile(vec![
            MeasuredWindSample::new(0.0, 3.0, 4.0, 0.0, 1.0),
            MeasuredWindSample::new(1_000.0, 13.0, 4.0, 0.0, 1.0),
            MeasuredWindSample::new(2_500.0, 13.0, 19.0, 0.0, 1.0),
        ]);
        let report = profile.quality_control(&qc_config(), 110.0).unwrap();
        assert_eq!(report.source, WindSource::Balloon);
        assert_eq!(report.sample_count, 3);
        assert_eq!(report.age_s.to_bits(), 100.0_f64.to_bits());
        assert_eq!(report.max_gap_m.to_bits(), 1_500.0_f64.to_bits());
        assert_eq!(report.max_shear_per_s.to_bits(), 0.01_f64.to_bits());
        assert_eq!(
            report.max_speed_m_s.to_bits(),
            (13.0_f64.hypot(19.0)).to_bits()
        );
    }

    #[test]
    fn qc_rejects_each_screen() {
        let mut config = qc_config();
        assert!(
            profile(vec![
                MeasuredWindSample::new(0.0, 0.0, 0.0, 0.0, 1.0),
                MeasuredWindSample::new(3_000.0, 0.0, 0.0, 0.0, 1.0),
            ])
            .quality_control(&config, 20.0)
            .is_err()
        );
        assert!(
            profile(vec![
                MeasuredWindSample::new(0.0, 0.0, 0.0, 0.0, 1.0),
                MeasuredWindSample::new(1_000.0, 40.0, 0.0, 0.0, 1.0),
            ])
            .quality_control(&config, 20.0)
            .is_err()
        );
        assert!(
            profile(vec![
                MeasuredWindSample::new(0.0, 101.0, 0.0, 0.0, 1.0),
                MeasuredWindSample::new(1_000.0, 101.0, 0.0, 0.0, 1.0),
            ])
            .quality_control(&config, 20.0)
            .is_err()
        );
        assert!(
            profile(vec![
                MeasuredWindSample::new(0.0, 0.0, 0.0, 0.0, 0.5),
                MeasuredWindSample::new(1_000.0, 0.0, 0.0, 0.0, 1.0),
            ])
            .quality_control(&config, 20.0)
            .is_err()
        );
        config.max_staleness_s = 5.0;
        assert!(
            profile(vec![
                MeasuredWindSample::new(0.0, 0.0, 0.0, 0.0, 1.0),
                MeasuredWindSample::new(1_000.0, 0.0, 0.0, 0.0, 1.0),
            ])
            .quality_control(&config, 20.0)
            .is_err()
        );
    }

    proptest! {
        #[test]
        fn qc_gap_rule_is_exact(gap_m in 1_u32..5_000, limit_m in 1_u32..5_000) {
            let mut config = qc_config();
            config.max_gap_m = f64::from(limit_m);
            let profile = profile(vec![
                MeasuredWindSample::new(0.0, 0.0, 0.0, 0.0, 1.0),
                MeasuredWindSample::new(f64::from(gap_m), 0.0, 0.0, 0.0, 1.0),
            ]);
            prop_assert_eq!(
                profile.quality_control(&config, 20.0).is_ok(),
                gap_m <= limit_m
            );
        }

        #[test]
        fn qc_shear_rule_is_exact(delta_m_s in 0_u32..100, limit_milli_per_s in 0_u32..100) {
            let mut config = qc_config();
            config.max_shear_per_s = f64::from(limit_milli_per_s) / 1_000.0;
            let profile = profile(vec![
                MeasuredWindSample::new(0.0, 0.0, 0.0, 0.0, 1.0),
                MeasuredWindSample::new(1_000.0, f64::from(delta_m_s), 0.0, 0.0, 1.0),
            ]);
            prop_assert_eq!(
                profile.quality_control(&config, 20.0).is_ok(),
                f64::from(delta_m_s) / 1_000.0 <= config.max_shear_per_s
            );
        }
    }

    #[test]
    fn splice_artifact_is_byte_stable_with_attribution() {
        let lower = profile(vec![
            MeasuredWindSample::new(0.0, 0.0, 0.0, 0.0, 1.0),
            MeasuredWindSample::new(1_000.0, 10.0, 0.0, 0.0, 1.0),
            MeasuredWindSample::new(2_000.0, 20.0, 0.0, 0.0, 1.0),
        ]);
        let upper = MeasuredWindProfile::new(
            WindSource::DopplerProfiler,
            10.0,
            vec![
                MeasuredWindSample::new(1_000.0, 30.0, 0.0, 0.0, 1.0),
                MeasuredWindSample::new(2_000.0, 50.0, 0.0, 0.0, 1.0),
                MeasuredWindSample::new(3_000.0, 70.0, 0.0, 0.0, 1.0),
            ],
            provenance(),
        )
        .unwrap();
        let rule = SpliceRule::new(1_000.0, 2_000.0).unwrap();
        let spliced = SplicedWindProfile::from_profiles(&lower, &upper, rule).unwrap();
        assert_eq!(
            spliced.to_csv(),
            "altitude_m,wind_north_m_s,wind_east_m_s,wind_down_m_s,lower_source,lower_weight_unit,upper_source,upper_weight_unit\n\
0.000000000,0.000000000,0.000000000,0.000000000,balloon,1.000000000,doppler_profiler,0.000000000\n\
1000.000000000,10.000000000,0.000000000,0.000000000,balloon,1.000000000,doppler_profiler,0.000000000\n\
2000.000000000,50.000000000,0.000000000,0.000000000,balloon,0.000000000,doppler_profiler,1.000000000\n\
3000.000000000,70.000000000,0.000000000,0.000000000,balloon,0.000000000,doppler_profiler,1.000000000\n"
        );
    }

    #[test]
    fn filter_magnitude_response_matches_closed_form() {
        let config = MartinGrahamFilterConfig::new(1_000.0).unwrap();
        let expected = 1.0 / (1.0_f64 + 0.25_f64).sqrt();
        let actual = config.magnitude_response(2_000.0).unwrap();
        assert!((actual - expected).abs() < 1.0e-12);
        assert!(
            (config.magnitude_response(1_000.0).unwrap() - 1.0_f64 / 2.0_f64.sqrt()).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn filter_preserves_grid_and_attribution() {
        let profile = SplicedWindProfile {
            lower_source: WindSource::Balloon,
            upper_source: WindSource::Lidar,
            samples: vec![
                SplicedWindSample {
                    altitude_m: 0.0,
                    wind_ned_m_s: Velocity3::new(0.0, 0.0, 0.0),
                    lower_weight_unit: 1.0,
                    upper_weight_unit: 0.0,
                },
                SplicedWindSample {
                    altitude_m: 100.0,
                    wind_ned_m_s: Velocity3::new(10.0, 0.0, 0.0),
                    lower_weight_unit: 0.5,
                    upper_weight_unit: 0.5,
                },
            ],
        };
        let filtered = MartinGrahamFilterConfig::new(500.0)
            .unwrap()
            .filter_spliced(&profile)
            .unwrap();
        assert_eq!(filtered.samples.len(), 2);
        assert_eq!(
            filtered.samples[1].altitude_m.to_bits(),
            100.0_f64.to_bits()
        );
        assert_eq!(
            filtered.samples[1].lower_weight_unit.to_bits(),
            0.5_f64.to_bits()
        );
        assert!(filtered.samples[1].wind_ned_m_s.vector.x > 0.0);
        assert!(filtered.samples[1].wind_ned_m_s.vector.x < 10.0);
    }

    #[test]
    fn synthetic_family_is_deterministic() {
        let config = SyntheticWindFamilyConfig {
            seed: 0x5151_1919,
            member_count: 3,
            released_at_s: 0.0,
            altitude_min_m: 0.0,
            altitude_max_m: 2_000.0,
            altitude_step_m: 500.0,
            base_wind_ned_m_s: Velocity3::new(5.0, -2.0, 0.0),
            shear_ned_per_s: Velocity3::new(0.001, 0.0, 0.0),
            amplitude_ned_m_s: Velocity3::new(3.0, 1.0, 0.2),
        };
        let a = config.generate().unwrap();
        let b = config.generate().unwrap();
        assert_eq!(a, b);
        assert_eq!(a.len(), 3);
        assert_eq!(a[0].samples.len(), 5);
        assert_ne!(
            a[0].samples[1].wind_ned_m_s.vector.x.to_bits(),
            a[1].samples[1].wind_ned_m_s.vector.x.to_bits()
        );
    }
}
