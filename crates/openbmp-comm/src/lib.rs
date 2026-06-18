//! Deterministic communications geometry primitives.
//!
//! This crate is the WP-20.1 substrate for radio-link geometry. It
//! models ground sites, elevation masks, fixed-Earth line-of-sight
//! geometry, rise/set events, pass-table extraction, and the WP-20.2
//! antenna/body-mask deck substrate, the WP-20.3 link-budget/FER substrate,
//! and the WP-20.4 packet-effect substrate. Blackout and network handover are
//! later WP-20 layers.

use std::io::Write;

use openbmp_core::{DeterministicRng, Vector3};
use sha2::{Digest, Sha256};
use thiserror::Error;

/// WGS84 semi-major axis, metres.
pub const WGS84_A_M: f64 = 6_378_137.0;
/// WGS84 flattening.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);
const TWO_PI: f64 = 2.0 * core::f64::consts::PI;
const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;
const PLASMA_FREQUENCY_COEFFICIENT_HZ_M32: f64 = 8.98;
const BOLTZMANN_DBW_PER_K_HZ: f64 = -228.599_167_173_217_65;
const FNV_OFFSET_BASIS_64: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME_64: u64 = 0x0000_0100_0000_01b3;
const MAX_LOSS_RATE_GATE_PACKETS: u64 = 1_000_000;

/// Error returned by communications geometry helpers.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum CommError {
    /// A scalar input was NaN or infinite.
    #[error("{field} is non-finite")]
    NonFinite {
        /// Field name.
        field: &'static str,
    },
    /// A scalar input was outside the accepted range.
    #[error("{field} is outside the accepted range: {value}")]
    OutOfRange {
        /// Field name.
        field: &'static str,
        /// Observed value.
        value: f64,
        /// Human-readable validation rule.
        rule: &'static str,
    },
    /// A list input was empty.
    #[error("{field} must not be empty")]
    Empty {
        /// Field name.
        field: &'static str,
    },
    /// A vector or geometric primitive has zero usable extent.
    #[error("{field} is degenerate")]
    Degenerate {
        /// Field name.
        field: &'static str,
    },
    /// Pass extraction received non-monotonic sample times.
    #[error("visibility samples must be strictly time ordered")]
    NonMonotonicSamples,
    /// Pass extraction received adjacent samples for different sites.
    #[error("visibility samples must belong to one site: {previous} then {current}")]
    MixedSiteSamples {
        /// Previous sample site id.
        previous: String,
        /// Current sample site id.
        current: String,
    },
}

/// Geodetic site location on the WGS84 ellipsoid.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GeodeticPosition {
    /// Geodetic latitude, radians.
    pub latitude_rad: f64,
    /// Longitude, radians, east positive.
    pub longitude_rad: f64,
    /// Height above the WGS84 ellipsoid, metres.
    pub altitude_m: f64,
}

impl GeodeticPosition {
    /// Construct after validating finite latitude, longitude, and height.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when any field is non-finite or when latitude is
    /// outside `[-pi/2, pi/2]`.
    pub fn new(latitude_rad: f64, longitude_rad: f64, altitude_m: f64) -> Result<Self, CommError> {
        require_finite("latitude_rad", latitude_rad)?;
        require_finite("longitude_rad", longitude_rad)?;
        require_finite("altitude_m", altitude_m)?;
        if !(-core::f64::consts::FRAC_PI_2..=core::f64::consts::FRAC_PI_2).contains(&latitude_rad) {
            return Err(CommError::OutOfRange {
                field: "latitude_rad",
                value: latitude_rad,
                rule: "must be in [-pi/2, pi/2]",
            });
        }
        Ok(Self {
            latitude_rad,
            longitude_rad: wrap_pi(longitude_rad),
            altitude_m,
        })
    }

    /// Convert the geodetic location to WGS84 ECEF coordinates, metres.
    #[must_use]
    pub fn to_ecef_m(self) -> Vector3<f64> {
        let sin_lat = self.latitude_rad.sin();
        let cos_lat = self.latitude_rad.cos();
        let sin_lon = self.longitude_rad.sin();
        let cos_lon = self.longitude_rad.cos();
        let prime_vertical_radius = WGS84_A_M / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
        let x = (prime_vertical_radius + self.altitude_m) * cos_lat * cos_lon;
        let y = (prime_vertical_radius + self.altitude_m) * cos_lat * sin_lon;
        let z = (prime_vertical_radius * (1.0 - WGS84_E2) + self.altitude_m) * sin_lat;
        Vector3::new(x, y, z)
    }

    /// Local ENU basis vectors expressed in ECEF coordinates.
    #[must_use]
    pub fn enu_basis_ecef(self) -> EnuBasis {
        let sin_lat = self.latitude_rad.sin();
        let cos_lat = self.latitude_rad.cos();
        let sin_lon = self.longitude_rad.sin();
        let cos_lon = self.longitude_rad.cos();
        EnuBasis {
            east: Vector3::new(-sin_lon, cos_lon, 0.0),
            north: Vector3::new(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat),
            up: Vector3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat),
        }
    }
}

/// Local ENU basis expressed in ECEF coordinates.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EnuBasis {
    /// Local east unit vector.
    pub east: Vector3<f64>,
    /// Local north unit vector.
    pub north: Vector3<f64>,
    /// Local up unit vector.
    pub up: Vector3<f64>,
}

/// Azimuth-binned terrain mask, with azimuth/elevation in radians.
#[derive(Clone, Debug, PartialEq)]
pub struct MaskDeck {
    bins: Vec<MaskDeckBin>,
}

/// One terrain-mask bin.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MaskDeckBin {
    /// Azimuth, radians, normalized to `[0, 2pi)`.
    pub azimuth_rad: f64,
    /// Minimum allowed elevation at this azimuth, radians.
    pub min_elevation_rad: f64,
}

impl MaskDeck {
    /// Construct an azimuth-sorted mask deck.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when the deck is empty, has non-finite values, or
    /// any elevation is outside `[-pi/2, pi/2]`.
    pub fn new(mut bins: Vec<MaskDeckBin>) -> Result<Self, CommError> {
        if bins.is_empty() {
            return Err(CommError::Empty {
                field: "terrain_mask",
            });
        }
        for bin in &mut bins {
            require_finite("terrain_mask.azimuth_rad", bin.azimuth_rad)?;
            require_elevation("terrain_mask.min_elevation_rad", bin.min_elevation_rad)?;
            bin.azimuth_rad = wrap_2pi(bin.azimuth_rad);
        }
        bins.sort_by(|a, b| a.azimuth_rad.total_cmp(&b.azimuth_rad));
        Ok(Self { bins })
    }

    /// Borrow the sorted bins.
    #[must_use]
    pub fn bins(&self) -> &[MaskDeckBin] {
        &self.bins
    }

    /// Interpolate the minimum elevation at `azimuth_rad`.
    #[must_use]
    pub fn min_elevation_at(&self, azimuth_rad: f64) -> f64 {
        let azimuth = wrap_2pi(azimuth_rad);
        if self.bins.len() == 1 {
            return self.bins[0].min_elevation_rad;
        }
        for pair in self.bins.windows(2) {
            let a = pair[0];
            let b = pair[1];
            if azimuth >= a.azimuth_rad && azimuth <= b.azimuth_rad {
                let u = (azimuth - a.azimuth_rad) / (b.azimuth_rad - a.azimuth_rad);
                return a.min_elevation_rad + u * (b.min_elevation_rad - a.min_elevation_rad);
            }
        }
        let first = self.bins[0];
        let last = self.bins[self.bins.len() - 1];
        let span = first.azimuth_rad + TWO_PI - last.azimuth_rad;
        let delta = if azimuth >= last.azimuth_rad {
            azimuth - last.azimuth_rad
        } else {
            azimuth + TWO_PI - last.azimuth_rad
        };
        let u = delta / span;
        last.min_elevation_rad + u * (first.min_elevation_rad - last.min_elevation_rad)
    }
}

/// Deterministic ground-site geometry model.
#[derive(Clone, Debug, PartialEq)]
pub struct GroundSite {
    /// Stable site identifier.
    pub id: String,
    /// Site geodetic location.
    pub geodetic: GeodeticPosition,
    /// Minimum allowed elevation, radians.
    pub min_elevation_rad: f64,
    /// Optional azimuth-dependent terrain mask.
    pub terrain_mask: Option<MaskDeck>,
}

impl GroundSite {
    /// Construct after validating the site id and elevation mask.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when `id` is empty or `min_elevation_rad` is
    /// non-finite/out of range.
    pub fn new(
        id: impl Into<String>,
        geodetic: GeodeticPosition,
        min_elevation_rad: f64,
        terrain_mask: Option<MaskDeck>,
    ) -> Result<Self, CommError> {
        let id = id.into();
        if id.trim().is_empty() {
            return Err(CommError::Empty { field: "site.id" });
        }
        require_elevation("min_elevation_rad", min_elevation_rad)?;
        Ok(Self {
            id,
            geodetic,
            min_elevation_rad,
            terrain_mask,
        })
    }

    /// Site ECEF position, metres.
    #[must_use]
    pub fn ecef_m(&self) -> Vector3<f64> {
        self.geodetic.to_ecef_m()
    }

    /// Compute line-of-sight geometry from site to vehicle.
    ///
    /// `vehicle_position_ecef_m` and `vehicle_velocity_ecef_m_s` are resolved
    /// in the same Earth-fixed frame as the site position.
    #[must_use]
    pub fn observe_ecef(
        &self,
        time_s: f64,
        vehicle_position_ecef_m: Vector3<f64>,
        vehicle_velocity_ecef_m_s: Vector3<f64>,
    ) -> VisibilitySample {
        let site_position = self.ecef_m();
        let los = vehicle_position_ecef_m - site_position;
        let slant_range_m = los.norm();
        let los_unit = if slant_range_m > 0.0 {
            los / slant_range_m
        } else {
            Vector3::zeros()
        };
        let basis = self.geodetic.enu_basis_ecef();
        let east_m = los.dot(&basis.east);
        let north_m = los.dot(&basis.north);
        let up_m = los.dot(&basis.up);
        let horizontal_range_m = east_m.hypot(north_m);
        let elevation_rad = up_m.atan2(horizontal_range_m);
        let azimuth_rad = wrap_2pi(east_m.atan2(north_m));
        let mask_elevation_rad =
            self.terrain_mask
                .as_ref()
                .map_or(self.min_elevation_rad, |mask| {
                    self.min_elevation_rad
                        .max(mask.min_elevation_at(azimuth_rad))
                });
        VisibilitySample {
            time_s,
            site_id: self.id.clone(),
            slant_range_m,
            range_rate_m_s: vehicle_velocity_ecef_m_s.dot(&los_unit),
            elevation_rad,
            azimuth_rad,
            mask_elevation_rad,
            visible: elevation_rad >= mask_elevation_rad,
        }
    }
}

/// One sampled site-to-vehicle visibility state.
#[derive(Clone, Debug, PartialEq)]
pub struct VisibilitySample {
    /// Sample time, seconds.
    pub time_s: f64,
    /// Site id.
    pub site_id: String,
    /// Slant range, metres.
    pub slant_range_m: f64,
    /// Range rate, metres per second, positive when range is increasing.
    pub range_rate_m_s: f64,
    /// Elevation angle, radians.
    pub elevation_rad: f64,
    /// Azimuth angle from north toward east, radians.
    pub azimuth_rad: f64,
    /// Applied elevation mask, radians.
    pub mask_elevation_rad: f64,
    /// `true` when elevation is above the applied mask.
    pub visible: bool,
}

/// One azimuth/elevation-independent antenna gain sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AntennaGainSample {
    /// Off-boresight angle, radians.
    pub off_boresight_rad: f64,
    /// Antenna gain at this off-boresight angle, dBi.
    pub gain_dbi: f64,
}

/// Axisymmetric antenna gain deck indexed by off-boresight angle.
#[derive(Clone, Debug, PartialEq)]
pub struct AntennaGainDeck {
    boresight_body: Vector3<f64>,
    samples: Vec<AntennaGainSample>,
}

impl AntennaGainDeck {
    /// Construct an angle-sorted antenna gain deck.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when the boresight is degenerate, samples are
    /// empty, angles/gains are non-finite, or angles are outside `[0, pi]`.
    pub fn new(
        boresight_body: Vector3<f64>,
        mut samples: Vec<AntennaGainSample>,
    ) -> Result<Self, CommError> {
        let boresight_body = unit_vector("antenna.boresight_body", boresight_body)?;
        if samples.is_empty() {
            return Err(CommError::Empty {
                field: "antenna.gain_samples",
            });
        }
        for sample in &samples {
            require_finite(
                "antenna.gain_sample.off_boresight_rad",
                sample.off_boresight_rad,
            )?;
            require_finite("antenna.gain_sample.gain_dbi", sample.gain_dbi)?;
            if !(0.0..=core::f64::consts::PI).contains(&sample.off_boresight_rad) {
                return Err(CommError::OutOfRange {
                    field: "antenna.gain_sample.off_boresight_rad",
                    value: sample.off_boresight_rad,
                    rule: "must be in [0, pi]",
                });
            }
        }
        samples.sort_by(|a, b| a.off_boresight_rad.total_cmp(&b.off_boresight_rad));
        for pair in samples.windows(2) {
            if pair[1].off_boresight_rad <= pair[0].off_boresight_rad {
                return Err(CommError::OutOfRange {
                    field: "antenna.gain_sample.off_boresight_rad",
                    value: pair[1].off_boresight_rad,
                    rule: "angles must be strictly increasing",
                });
            }
        }
        Ok(Self {
            boresight_body,
            samples,
        })
    }

    /// Unit boresight direction in body axes.
    #[must_use]
    pub fn boresight_body(&self) -> Vector3<f64> {
        self.boresight_body
    }

    /// Borrow angle-sorted gain samples.
    #[must_use]
    pub fn samples(&self) -> &[AntennaGainSample] {
        &self.samples
    }

    /// Interpolate gain for a body-frame direction.
    ///
    /// Directions outside the deck angle range clamp to the nearest endpoint.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when `direction_body` is degenerate.
    pub fn gain_dbi_for_direction(&self, direction_body: Vector3<f64>) -> Result<f64, CommError> {
        let direction_body = unit_vector("antenna.direction_body", direction_body)?;
        let angle_rad = self
            .boresight_body
            .dot(&direction_body)
            .clamp(-1.0, 1.0)
            .acos();
        Ok(self.gain_dbi_at_off_boresight(angle_rad))
    }

    /// Interpolate gain for an off-boresight angle in radians.
    ///
    /// Angles outside the deck range clamp to the nearest endpoint.
    #[must_use]
    pub fn gain_dbi_at_off_boresight(&self, angle_rad: f64) -> f64 {
        if angle_rad <= self.samples[0].off_boresight_rad {
            return self.samples[0].gain_dbi;
        }
        for pair in self.samples.windows(2) {
            let a = pair[0];
            let b = pair[1];
            if angle_rad <= b.off_boresight_rad {
                let u =
                    (angle_rad - a.off_boresight_rad) / (b.off_boresight_rad - a.off_boresight_rad);
                return a.gain_dbi + u * (b.gain_dbi - a.gain_dbi);
            }
        }
        self.samples[self.samples.len() - 1].gain_dbi
    }
}

/// One precomputed body-mask direction sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BodyMaskSample {
    /// Body-frame line-of-sight direction.
    pub direction_body: Vector3<f64>,
    /// `true` when vehicle geometry blocks this ray.
    pub blocked: bool,
}

/// Triangle used for deterministic body-mask ray casts.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BodyMaskTriangle {
    /// First vertex, body-frame metres.
    pub a_m: Vector3<f64>,
    /// Second vertex, body-frame metres.
    pub b_m: Vector3<f64>,
    /// Third vertex, body-frame metres.
    pub c_m: Vector3<f64>,
}

/// Precomputed body occlusion deck indexed by body-frame ray direction.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyMaskDeck {
    samples: Vec<BodyMaskSample>,
}

impl BodyMaskDeck {
    /// Construct a body-mask deck from precomputed samples.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when samples are empty or any sample direction is
    /// degenerate.
    pub fn new(samples: Vec<BodyMaskSample>) -> Result<Self, CommError> {
        if samples.is_empty() {
            return Err(CommError::Empty {
                field: "body_mask.samples",
            });
        }
        let mut normalized = Vec::with_capacity(samples.len());
        for sample in samples {
            normalized.push(BodyMaskSample {
                direction_body: unit_vector(
                    "body_mask.sample.direction_body",
                    sample.direction_body,
                )?,
                blocked: sample.blocked,
            });
        }
        Ok(Self {
            samples: normalized,
        })
    }

    /// Precompute a body-mask deck by direct ray casts against triangles.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when the direction list is empty, the antenna
    /// position is non-finite, a direction is degenerate, or any triangle is
    /// degenerate/non-finite.
    pub fn precompute_from_triangles(
        antenna_position_body_m: Vector3<f64>,
        directions_body: &[Vector3<f64>],
        triangles: &[BodyMaskTriangle],
    ) -> Result<Self, CommError> {
        require_finite_vector("body_mask.antenna_position_body_m", antenna_position_body_m)?;
        if directions_body.is_empty() {
            return Err(CommError::Empty {
                field: "body_mask.directions",
            });
        }
        for triangle in triangles {
            triangle.validate()?;
        }
        let mut samples = Vec::with_capacity(directions_body.len());
        for direction in directions_body {
            let direction_body = unit_vector("body_mask.direction_body", *direction)?;
            samples.push(BodyMaskSample {
                direction_body,
                blocked: triangles.iter().any(|triangle| {
                    ray_intersects_triangle(antenna_position_body_m, direction_body, *triangle)
                }),
            });
        }
        Ok(Self { samples })
    }

    /// Borrow precomputed samples in deck order.
    #[must_use]
    pub fn samples(&self) -> &[BodyMaskSample] {
        &self.samples
    }

    /// Return the nearest precomputed body-mask value for a body-frame ray.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when `direction_body` is degenerate.
    pub fn is_blocked_nearest(&self, direction_body: Vector3<f64>) -> Result<bool, CommError> {
        let direction_body = unit_vector("body_mask.lookup.direction_body", direction_body)?;
        let mut best_index = 0;
        let mut best_dot = f64::NEG_INFINITY;
        for (index, sample) in self.samples.iter().enumerate() {
            let dot = sample.direction_body.dot(&direction_body);
            if dot > best_dot {
                best_dot = dot;
                best_index = index;
            }
        }
        Ok(self.samples[best_index].blocked)
    }

    /// Deterministic SHA-256 digest over the mesh hash and deck samples.
    #[must_use]
    pub fn sha256_hex_with_mesh_hash(&self, mesh_sha256_hex: &str) -> String {
        let mut hasher = Sha256::new();
        hasher.update(b"openbmp.body_mask_deck.v1\n");
        hasher.update(mesh_sha256_hex.as_bytes());
        hasher.update(b"\n");
        for sample in &self.samples {
            for value in [
                sample.direction_body.x,
                sample.direction_body.y,
                sample.direction_body.z,
            ] {
                hasher.update(value.to_bits().to_le_bytes());
            }
            hasher.update([u8::from(sample.blocked)]);
        }
        hex_digest(&hasher.finalize().into())
    }
}

impl BodyMaskTriangle {
    fn validate(self) -> Result<(), CommError> {
        require_finite_vector("body_mask.triangle.a_m", self.a_m)?;
        require_finite_vector("body_mask.triangle.b_m", self.b_m)?;
        require_finite_vector("body_mask.triangle.c_m", self.c_m)?;
        let normal = (self.b_m - self.a_m).cross(&(self.c_m - self.a_m));
        if normal.norm() <= f64::EPSILON {
            return Err(CommError::Degenerate {
                field: "body_mask.triangle",
            });
        }
        Ok(())
    }
}

/// One coded-performance curve sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FerCurveSample {
    /// Energy per bit to noise density ratio, dB.
    pub eb_n0_db: f64,
    /// Frame error rate at this point. Must be in `(0, 1]`.
    pub fer: f64,
}

/// Monotone coded-performance deck: FER as a function of Eb/N0.
#[derive(Clone, Debug, PartialEq)]
pub struct FerCurveDeck {
    code_id: String,
    samples: Vec<FerCurveSample>,
}

impl FerCurveDeck {
    /// Construct a monotone FER deck sorted by increasing Eb/N0.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when the code id is empty, samples are empty,
    /// Eb/N0 or FER values are non-finite, Eb/N0 samples are not strictly
    /// increasing, or FER is outside `(0, 1]` / increases with Eb/N0.
    pub fn new(
        code_id: impl Into<String>,
        mut samples: Vec<FerCurveSample>,
    ) -> Result<Self, CommError> {
        let code_id = code_id.into();
        if code_id.trim().is_empty() {
            return Err(CommError::Empty {
                field: "fer_curve.code_id",
            });
        }
        if samples.is_empty() {
            return Err(CommError::Empty {
                field: "fer_curve.samples",
            });
        }
        for sample in &samples {
            require_finite("fer_curve.sample.eb_n0_db", sample.eb_n0_db)?;
            require_finite("fer_curve.sample.fer", sample.fer)?;
            if !(0.0..=1.0).contains(&sample.fer) || sample.fer == 0.0 {
                return Err(CommError::OutOfRange {
                    field: "fer_curve.sample.fer",
                    value: sample.fer,
                    rule: "must be in (0, 1]",
                });
            }
        }
        samples.sort_by(|a, b| a.eb_n0_db.total_cmp(&b.eb_n0_db));
        for pair in samples.windows(2) {
            if pair[1].eb_n0_db <= pair[0].eb_n0_db {
                return Err(CommError::OutOfRange {
                    field: "fer_curve.sample.eb_n0_db",
                    value: pair[1].eb_n0_db,
                    rule: "samples must be strictly increasing",
                });
            }
            if pair[1].fer > pair[0].fer {
                return Err(CommError::OutOfRange {
                    field: "fer_curve.sample.fer",
                    value: pair[1].fer,
                    rule: "FER must be monotone non-increasing with Eb/N0",
                });
            }
        }
        Ok(Self { code_id, samples })
    }

    /// Coded-performance deck id.
    #[must_use]
    pub fn code_id(&self) -> &str {
        &self.code_id
    }

    /// Borrow samples sorted by increasing Eb/N0.
    #[must_use]
    pub fn samples(&self) -> &[FerCurveSample] {
        &self.samples
    }

    /// Interpolate FER at `eb_n0_db`.
    ///
    /// Interpolation is linear in `log10(FER)` versus Eb/N0. Values outside
    /// the deck envelope fail closed instead of silently clamping.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when `eb_n0_db` is non-finite or outside the
    /// sampled deck envelope.
    pub fn fer_at_eb_n0_db(&self, eb_n0_db: f64) -> Result<f64, CommError> {
        require_finite("link_state.eb_n0_db", eb_n0_db)?;
        let first = self.samples[0];
        let last = self.samples[self.samples.len() - 1];
        if eb_n0_db < first.eb_n0_db || eb_n0_db > last.eb_n0_db {
            return Err(CommError::OutOfRange {
                field: "link_state.eb_n0_db",
                value: eb_n0_db,
                rule: "must lie inside the FER curve envelope",
            });
        }
        if self.samples.len() == 1 {
            return Ok(first.fer);
        }
        for pair in self.samples.windows(2) {
            let a = pair[0];
            let b = pair[1];
            if eb_n0_db <= b.eb_n0_db {
                let u = (eb_n0_db - a.eb_n0_db) / (b.eb_n0_db - a.eb_n0_db);
                let log_fer = a.fer.log10() + u * (b.fer.log10() - a.fer.log10());
                return Ok(10.0_f64.powf(log_fer));
            }
        }
        Ok(last.fer)
    }
}

/// One elevation-indexed attenuation sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ElevationLossSample {
    /// Elevation angle, radians.
    pub elevation_rad: f64,
    /// Attenuation loss, dB. Must be non-negative.
    pub loss_db: f64,
}

/// Elevation-indexed atmospheric/rain loss deck.
#[derive(Clone, Debug, PartialEq)]
pub struct ElevationLossDeck {
    deck_id: String,
    samples: Vec<ElevationLossSample>,
}

impl ElevationLossDeck {
    /// Construct an elevation-sorted loss deck.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when the deck id is empty, samples are empty,
    /// elevations/losses are non-finite, elevations are outside
    /// `[-pi/2, pi/2]`, losses are negative, or elevations are not strictly
    /// increasing.
    pub fn new(
        deck_id: impl Into<String>,
        mut samples: Vec<ElevationLossSample>,
    ) -> Result<Self, CommError> {
        let deck_id = deck_id.into();
        if deck_id.trim().is_empty() {
            return Err(CommError::Empty {
                field: "elevation_loss.deck_id",
            });
        }
        if samples.is_empty() {
            return Err(CommError::Empty {
                field: "elevation_loss.samples",
            });
        }
        for sample in &samples {
            require_elevation("elevation_loss.sample.elevation_rad", sample.elevation_rad)?;
            require_non_negative("elevation_loss.sample.loss_db", sample.loss_db)?;
        }
        samples.sort_by(|a, b| a.elevation_rad.total_cmp(&b.elevation_rad));
        for pair in samples.windows(2) {
            if pair[1].elevation_rad <= pair[0].elevation_rad {
                return Err(CommError::OutOfRange {
                    field: "elevation_loss.sample.elevation_rad",
                    value: pair[1].elevation_rad,
                    rule: "samples must be strictly increasing",
                });
            }
        }
        Ok(Self { deck_id, samples })
    }

    /// Deck id.
    #[must_use]
    pub fn deck_id(&self) -> &str {
        &self.deck_id
    }

    /// Borrow samples sorted by increasing elevation.
    #[must_use]
    pub fn samples(&self) -> &[ElevationLossSample] {
        &self.samples
    }

    /// Interpolate loss at elevation.
    ///
    /// Values outside the deck envelope fail closed instead of silently
    /// clamping.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when elevation is non-finite/out of physical
    /// range or outside the sampled deck envelope.
    pub fn loss_db_at_elevation(&self, elevation_rad: f64) -> Result<f64, CommError> {
        require_elevation("link_state.elevation_rad", elevation_rad)?;
        let first = self.samples[0];
        let last = self.samples[self.samples.len() - 1];
        if elevation_rad < first.elevation_rad || elevation_rad > last.elevation_rad {
            return Err(CommError::OutOfRange {
                field: "link_state.elevation_rad",
                value: elevation_rad,
                rule: "must lie inside the elevation-loss deck envelope",
            });
        }
        if self.samples.len() == 1 {
            return Ok(first.loss_db);
        }
        for pair in self.samples.windows(2) {
            let a = pair[0];
            let b = pair[1];
            if elevation_rad <= b.elevation_rad {
                let u = (elevation_rad - a.elevation_rad) / (b.elevation_rad - a.elevation_rad);
                return Ok(a.loss_db + u * (b.loss_db - a.loss_db));
            }
        }
        Ok(last.loss_db)
    }
}

/// Link-budget inputs for one vehicle antenna / site / band at one step.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LinkBudgetInput {
    /// Effective isotropic radiated power, dBW.
    pub eirp_dbw: f64,
    /// Receiver gain-to-noise-temperature ratio, dB/K.
    pub receiver_g_over_t_db_k: f64,
    /// Slant range, metres.
    pub slant_range_m: f64,
    /// Carrier frequency, Hz.
    pub frequency_hz: f64,
    /// Bit rate, bit/s.
    pub bit_rate_bps: f64,
    /// Required Eb/N0 for positive link margin, dB.
    pub required_eb_n0_db: f64,
    /// Atmospheric gas attenuation, dB.
    pub atmospheric_loss_db: f64,
    /// Rain attenuation, dB.
    pub rain_loss_db: f64,
    /// Pointing loss, dB.
    pub pointing_loss_db: f64,
    /// Polarization loss, dB.
    pub polarization_loss_db: f64,
    /// Implementation and miscellaneous fixed losses, dB.
    pub implementation_loss_db: f64,
    /// Geometric/mask visibility before blackout.
    pub visible: bool,
    /// Plasma or other blackout flag.
    pub blackout: bool,
}

/// Plasma-blackout model parameters for one radio band.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PlasmaBlackoutModel {
    /// Frequency interval above plasma frequency over which attenuation ramps down.
    ramp_width_hz: f64,
    /// Maximum attenuation applied at and below plasma frequency, dB.
    max_attenuation_db: f64,
}

/// Plasma-blackout inputs for one link sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PlasmaBlackoutInput {
    /// Link carrier frequency, Hz.
    pub frequency_hz: f64,
    /// Electron number density, electrons per cubic metre.
    pub electron_number_density_m3: f64,
}

/// Plasma-blackout state derived from an electron-density sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PlasmaBlackoutState {
    /// Plasma frequency, Hz.
    pub plasma_frequency_hz: f64,
    /// Additional attenuation, dB.
    pub attenuation_db: f64,
    /// Whether the carrier is at or below plasma frequency.
    pub blackout: bool,
}

impl PlasmaBlackoutModel {
    /// Construct a deterministic plasma-blackout attenuation model.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when the ramp width or maximum attenuation is
    /// negative/non-finite.
    pub fn new(ramp_width_hz: f64, max_attenuation_db: f64) -> Result<Self, CommError> {
        require_non_negative("plasma_blackout.ramp_width_hz", ramp_width_hz)?;
        require_non_negative("plasma_blackout.max_attenuation_db", max_attenuation_db)?;
        Ok(Self {
            ramp_width_hz,
            max_attenuation_db,
        })
    }

    /// Frequency interval above plasma frequency over which attenuation ramps down.
    #[must_use]
    pub const fn ramp_width_hz(self) -> f64 {
        self.ramp_width_hz
    }

    /// Maximum attenuation applied at and below plasma frequency, dB.
    #[must_use]
    pub const fn max_attenuation_db(self) -> f64 {
        self.max_attenuation_db
    }

    /// Evaluate blackout state from carrier frequency and electron density.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when the carrier frequency is not positive or the
    /// electron density is negative/non-finite.
    pub fn evaluate(self, input: PlasmaBlackoutInput) -> Result<PlasmaBlackoutState, CommError> {
        require_positive("plasma_blackout.frequency_hz", input.frequency_hz)?;
        require_non_negative(
            "plasma_blackout.electron_number_density_m3",
            input.electron_number_density_m3,
        )?;
        let plasma_frequency_hz = plasma_frequency_hz(input.electron_number_density_m3);
        let blackout = input.frequency_hz <= plasma_frequency_hz;
        let attenuation_db = if blackout {
            self.max_attenuation_db
        } else if self.ramp_width_hz > 0.0
            && input.frequency_hz < plasma_frequency_hz + self.ramp_width_hz
        {
            let fraction = 1.0 - (input.frequency_hz - plasma_frequency_hz) / self.ramp_width_hz;
            self.max_attenuation_db * fraction
        } else {
            0.0
        };
        Ok(PlasmaBlackoutState {
            plasma_frequency_hz,
            attenuation_db,
            blackout,
        })
    }
}

impl Default for PlasmaBlackoutModel {
    fn default() -> Self {
        Self {
            ramp_width_hz: 0.0,
            max_attenuation_db: 60.0,
        }
    }
}

/// Compute plasma frequency from electron number density.
///
/// Uses the engineering relation `f_p ≈ 8.98 sqrt(n_e)` with `f_p` in Hz and
/// `n_e` in electrons per cubic metre.
///
/// # Errors
///
/// Returns [`CommError`] when electron density is negative/non-finite.
pub fn plasma_frequency_hz(electron_number_density_m3: f64) -> f64 {
    PLASMA_FREQUENCY_COEFFICIENT_HZ_M32 * electron_number_density_m3.sqrt()
}

/// Per-step communications link state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LinkState {
    /// Free-space path loss, dB.
    pub free_space_loss_db: f64,
    /// Carrier-to-noise-density ratio, dB-Hz.
    pub c_n0_dbhz: f64,
    /// Energy-per-bit to noise-density ratio, dB.
    pub eb_n0_db: f64,
    /// Link margin relative to the declared requirement, dB.
    pub margin_db: f64,
    /// Frame error rate.
    pub fer: f64,
    /// Geometric/mask visibility before blackout.
    pub visible: bool,
    /// Plasma or other blackout flag.
    pub blackout: bool,
}

/// Packet direction for deterministic link-effect RNG separation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum LinkPacketDirection {
    /// Vehicle-to-ground telemetry/downlink frame.
    Downlink,
    /// Ground-to-vehicle command/uplink frame.
    Uplink,
}

impl LinkPacketDirection {
    const fn seed_byte(self) -> u8 {
        match self {
            Self::Downlink => 0,
            Self::Uplink => 1,
        }
    }
}

/// Packet error action to apply when a FER draw fails.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum LinkErrorAction {
    /// Drop errored frames at the abstract packet boundary.
    Drop,
    /// Mark errored frames as corrupted with a deterministic non-zero mask.
    BitFlip {
        /// Non-zero bit mask for a downstream framed-byte fault bench.
        mask: u8,
    },
}

/// Packet disposition produced by the link channel model.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum LinkPacketDisposition {
    /// Packet remains available for normal processing.
    Deliver,
    /// Packet is unavailable due to no link or an errored frame.
    Drop,
    /// Packet is delivered as a corrupted frame.
    BitFlip {
        /// Non-zero corruption mask.
        mask: u8,
    },
}

/// Packet-level context for one link-effect evaluation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LinkPacketContext {
    /// Scenario seed used to domain-separate stochastic packet effects.
    pub scenario_seed: u64,
    /// Simulation/bridge step index.
    pub step: u64,
    /// Stable per-link stream id.
    pub link_stream_id: u64,
    /// Packet index inside this link/step/direction stream.
    pub packet_index: u64,
    /// Uplink or downlink packet direction.
    pub direction: LinkPacketDirection,
    /// Slant range used for propagation delay, metres.
    pub slant_range_m: f64,
}

/// Link-effect result for one packet.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LinkPacketEffect {
    /// Packet disposition after visibility, blackout, and FER are applied.
    pub disposition: LinkPacketDisposition,
    /// One-way light-time propagation delay, seconds.
    pub propagation_delay_s: f64,
    /// Declared processing delay added to the propagation delay, seconds.
    pub processing_delay_s: f64,
    /// Total link latency, seconds.
    pub latency_s: f64,
    /// Deterministic FER draw in `[0, 1)` when a physical link exists.
    pub error_draw: Option<f64>,
}

/// Exact-binomial evidence for observed packet error rate.
#[derive(Clone, Debug, PartialEq)]
pub struct LinkLossRateGate {
    /// Number of packets evaluated.
    pub packet_count: u64,
    /// Packets that were dropped or delivered corrupted.
    pub errored_packet_count: u64,
    /// Expected packet error rate from no-link state or FER.
    pub expected_error_rate: f64,
    /// Observed packet error rate over `packet_count`.
    pub observed_error_rate: f64,
    /// Two-sided significance level used for the acceptance interval.
    pub alpha: f64,
    /// Lowest accepted errored-packet count.
    pub lower_accepted_errors: u64,
    /// Highest accepted errored-packet count.
    pub upper_accepted_errors: u64,
}

impl LinkLossRateGate {
    /// Whether the observed errored-packet count is inside the exact interval.
    #[must_use]
    pub const fn passed(&self) -> bool {
        self.errored_packet_count >= self.lower_accepted_errors
            && self.errored_packet_count <= self.upper_accepted_errors
    }
}

/// Deterministic packet-effect model driven by [`LinkState`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LinkChannelModel {
    processing_delay_s: f64,
    error_action: LinkErrorAction,
}

impl LinkChannelModel {
    /// Construct a link-effect model.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when `processing_delay_s` is negative/non-finite
    /// or when a bit-flip action uses a zero mask.
    pub fn new(processing_delay_s: f64, error_action: LinkErrorAction) -> Result<Self, CommError> {
        require_non_negative("link_channel.processing_delay_s", processing_delay_s)?;
        validate_link_error_action(error_action)?;
        Ok(Self {
            processing_delay_s,
            error_action,
        })
    }

    /// Declared processing delay, seconds.
    #[must_use]
    pub const fn processing_delay_s(self) -> f64 {
        self.processing_delay_s
    }

    /// Packet error action for failed FER draws.
    #[must_use]
    pub const fn error_action(self) -> LinkErrorAction {
        self.error_action
    }

    /// Evaluate packet effects from a link state and packet context.
    ///
    /// # Errors
    ///
    /// Returns [`CommError`] when `state` carries invalid/non-finite scalar
    /// fields or when the context slant range is non-positive/non-finite.
    pub fn evaluate_packet(
        self,
        state: LinkState,
        context: LinkPacketContext,
    ) -> Result<LinkPacketEffect, CommError> {
        validate_link_state_for_packet(state)?;
        context.validate()?;
        let propagation_delay_s = context.slant_range_m / SPEED_OF_LIGHT_M_S;
        let latency_s = propagation_delay_s + self.processing_delay_s;
        if !state.visible || state.blackout {
            return Ok(LinkPacketEffect {
                disposition: LinkPacketDisposition::Drop,
                propagation_delay_s,
                processing_delay_s: self.processing_delay_s,
                latency_s,
                error_draw: None,
            });
        }
        let error_draw = link_packet_error_draw(context);
        let disposition = if error_draw < state.fer {
            match self.error_action {
                LinkErrorAction::Drop => LinkPacketDisposition::Drop,
                LinkErrorAction::BitFlip { mask } => LinkPacketDisposition::BitFlip { mask },
            }
        } else {
            LinkPacketDisposition::Deliver
        };
        Ok(LinkPacketEffect {
            disposition,
            propagation_delay_s,
            processing_delay_s: self.processing_delay_s,
            latency_s,
            error_draw: Some(error_draw),
        })
    }
}

/// Evaluate a deterministic packet stream and gate its observed error rate.
///
/// The accepted error-count interval is the two-sided exact binomial interval
/// for `expected_error_rate` and `alpha`. Non-delivered packets count as
/// errored; a no-visibility or blackout link therefore has expected rate `1`.
///
/// # Errors
///
/// Returns [`CommError`] when `state` or `base_context` is invalid, when
/// `packet_count` is zero or too large for the bounded gate, when
/// `packet_index + packet_count - 1` overflows, or when `alpha` is not in
/// `(0, 1)`.
pub fn evaluate_link_loss_rate_gate(
    model: LinkChannelModel,
    state: LinkState,
    base_context: LinkPacketContext,
    packet_count: u64,
    alpha: f64,
) -> Result<LinkLossRateGate, CommError> {
    validate_loss_rate_packet_count(packet_count)?;
    validate_alpha(alpha)?;
    validate_link_state_for_packet(state)?;
    base_context.validate()?;
    let end_index = base_context
        .packet_index
        .checked_add(packet_count - 1)
        .ok_or(CommError::OutOfRange {
            field: "link_loss_rate.packet_index",
            value: base_context.packet_index as f64,
            rule: "base packet_index + packet_count - 1 must fit in u64",
        })?;
    let expected_error_rate = if !state.visible || state.blackout {
        1.0
    } else {
        state.fer
    };
    let (lower_accepted_errors, upper_accepted_errors) =
        binomial_acceptance_interval(packet_count, expected_error_rate, alpha);
    let mut errored_packet_count = 0;
    for packet_index in base_context.packet_index..=end_index {
        let effect = model.evaluate_packet(
            state,
            LinkPacketContext {
                packet_index,
                ..base_context
            },
        )?;
        if !matches!(effect.disposition, LinkPacketDisposition::Deliver) {
            errored_packet_count += 1;
        }
    }
    Ok(LinkLossRateGate {
        packet_count,
        errored_packet_count,
        expected_error_rate,
        observed_error_rate: errored_packet_count as f64 / packet_count as f64,
        alpha,
        lower_accepted_errors,
        upper_accepted_errors,
    })
}

impl Default for LinkChannelModel {
    fn default() -> Self {
        Self {
            processing_delay_s: 0.0,
            error_action: LinkErrorAction::Drop,
        }
    }
}

impl LinkPacketContext {
    fn validate(self) -> Result<(), CommError> {
        require_positive("link_channel.slant_range_m", self.slant_range_m)
    }
}

/// Stable stream id for a declared comm link id.
///
/// # Errors
///
/// Returns [`CommError::Empty`] when `id` is empty after trimming.
pub fn link_stream_id_from_id(id: &str) -> Result<u64, CommError> {
    if id.trim().is_empty() {
        return Err(CommError::Empty {
            field: "link_channel.link_id",
        });
    }
    let mut hash = FNV_OFFSET_BASIS_64;
    for byte in id.as_bytes() {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(FNV_PRIME_64);
    }
    Ok(hash)
}

/// Evaluate the deterministic engineering link budget and FER.
///
/// # Errors
///
/// Returns [`CommError`] when physical scales are non-finite/non-positive,
/// losses are negative, or FER interpolation falls outside the deck envelope.
pub fn evaluate_link_budget(
    input: LinkBudgetInput,
    fer_curve: &FerCurveDeck,
) -> Result<LinkState, CommError> {
    input.validate()?;
    let free_space_loss_db = free_space_path_loss_db(input.slant_range_m, input.frequency_hz)?;
    let total_loss_db = free_space_loss_db
        + input.atmospheric_loss_db
        + input.rain_loss_db
        + input.pointing_loss_db
        + input.polarization_loss_db
        + input.implementation_loss_db;
    let c_n0_dbhz =
        input.eirp_dbw + input.receiver_g_over_t_db_k - total_loss_db - BOLTZMANN_DBW_PER_K_HZ;
    let eb_n0_db = c_n0_dbhz - 10.0 * input.bit_rate_bps.log10();
    let margin_db = eb_n0_db - input.required_eb_n0_db;
    let fer = if input.visible && !input.blackout {
        fer_curve.fer_at_eb_n0_db(eb_n0_db)?
    } else {
        1.0
    };
    Ok(LinkState {
        free_space_loss_db,
        c_n0_dbhz,
        eb_n0_db,
        margin_db,
        fer,
        visible: input.visible,
        blackout: input.blackout,
    })
}

/// Free-space path loss `20 log10(4 pi d / lambda)`, dB.
///
/// # Errors
///
/// Returns [`CommError`] when range/frequency are non-finite or not positive.
pub fn free_space_path_loss_db(range_m: f64, frequency_hz: f64) -> Result<f64, CommError> {
    require_positive("link_budget.slant_range_m", range_m)?;
    require_positive("link_budget.frequency_hz", frequency_hz)?;
    Ok(20.0 * (4.0 * core::f64::consts::PI * range_m * frequency_hz / SPEED_OF_LIGHT_M_S).log10())
}

impl LinkBudgetInput {
    fn validate(self) -> Result<(), CommError> {
        require_finite("link_budget.eirp_dbw", self.eirp_dbw)?;
        require_finite(
            "link_budget.receiver_g_over_t_db_k",
            self.receiver_g_over_t_db_k,
        )?;
        require_positive("link_budget.slant_range_m", self.slant_range_m)?;
        require_positive("link_budget.frequency_hz", self.frequency_hz)?;
        require_positive("link_budget.bit_rate_bps", self.bit_rate_bps)?;
        require_finite("link_budget.required_eb_n0_db", self.required_eb_n0_db)?;
        require_non_negative("link_budget.atmospheric_loss_db", self.atmospheric_loss_db)?;
        require_non_negative("link_budget.rain_loss_db", self.rain_loss_db)?;
        require_non_negative("link_budget.pointing_loss_db", self.pointing_loss_db)?;
        require_non_negative(
            "link_budget.polarization_loss_db",
            self.polarization_loss_db,
        )?;
        require_non_negative(
            "link_budget.implementation_loss_db",
            self.implementation_loss_db,
        )
    }
}

fn validate_link_error_action(action: LinkErrorAction) -> Result<(), CommError> {
    match action {
        LinkErrorAction::Drop => Ok(()),
        LinkErrorAction::BitFlip { mask } if mask != 0 => Ok(()),
        LinkErrorAction::BitFlip { mask } => Err(CommError::OutOfRange {
            field: "link_channel.bit_flip_mask",
            value: f64::from(mask),
            rule: "must be non-zero",
        }),
    }
}

fn validate_link_state_for_packet(state: LinkState) -> Result<(), CommError> {
    require_finite(
        "link_channel.link_state.free_space_loss_db",
        state.free_space_loss_db,
    )?;
    require_finite("link_channel.link_state.c_n0_dbhz", state.c_n0_dbhz)?;
    require_finite("link_channel.link_state.eb_n0_db", state.eb_n0_db)?;
    require_finite("link_channel.link_state.margin_db", state.margin_db)?;
    require_finite("link_channel.link_state.fer", state.fer)?;
    if (0.0..=1.0).contains(&state.fer) {
        Ok(())
    } else {
        Err(CommError::OutOfRange {
            field: "link_channel.link_state.fer",
            value: state.fer,
            rule: "must be in [0, 1]",
        })
    }
}

fn validate_loss_rate_packet_count(packet_count: u64) -> Result<(), CommError> {
    if (1..=MAX_LOSS_RATE_GATE_PACKETS).contains(&packet_count) {
        Ok(())
    } else {
        Err(CommError::OutOfRange {
            field: "link_loss_rate.packet_count",
            value: packet_count as f64,
            rule: "must be in [1, 1000000]",
        })
    }
}

fn validate_alpha(alpha: f64) -> Result<(), CommError> {
    require_finite("link_loss_rate.alpha", alpha)?;
    if (0.0..1.0).contains(&alpha) {
        Ok(())
    } else {
        Err(CommError::OutOfRange {
            field: "link_loss_rate.alpha",
            value: alpha,
            rule: "must be in (0, 1)",
        })
    }
}

fn binomial_acceptance_interval(packet_count: u64, probability: f64, alpha: f64) -> (u64, u64) {
    if probability <= 0.0 {
        return (0, 0);
    }
    if probability >= 1.0 {
        return (packet_count, packet_count);
    }
    let tail_probability = 0.5 * alpha;
    let probabilities = binomial_probabilities(packet_count as usize, probability);
    let mut lower = 0;
    let mut cumulative = 0.0;
    for (index, probability) in probabilities.iter().enumerate() {
        cumulative += probability;
        if cumulative >= tail_probability {
            lower = index as u64;
            break;
        }
    }
    let mut upper = packet_count;
    let mut survival = 0.0;
    for (index, probability) in probabilities.iter().enumerate().rev() {
        survival += probability;
        if survival >= tail_probability {
            upper = index as u64;
            break;
        }
    }
    (lower, upper)
}

fn binomial_probabilities(packet_count: usize, probability: f64) -> Vec<f64> {
    let log_p = probability.ln();
    let log_q = (1.0 - probability).ln();
    let mut log_probabilities = Vec::with_capacity(packet_count + 1);
    log_probabilities.push(packet_count as f64 * log_q);
    for index in 0..packet_count {
        let ratio =
            ((packet_count - index) as f64).ln() - ((index + 1) as f64).ln() + log_p - log_q;
        log_probabilities.push(log_probabilities[index] + ratio);
    }
    let max_log = log_probabilities
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    let mut probabilities = Vec::with_capacity(packet_count + 1);
    let mut normalization = 0.0;
    for log_probability in log_probabilities {
        let probability = (log_probability - max_log).exp();
        normalization += probability;
        probabilities.push(probability);
    }
    for probability in &mut probabilities {
        *probability /= normalization;
    }
    probabilities
}

fn link_packet_error_draw(context: LinkPacketContext) -> f64 {
    let mut rng = DeterministicRng::from_raw_seed(link_packet_rng_seed(context));
    uniform01(&mut rng)
}

fn link_packet_rng_seed(context: LinkPacketContext) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(b"openbmp.link_channel_packet.v1\n");
    hasher.update(context.scenario_seed.to_le_bytes());
    hasher.update(context.step.to_le_bytes());
    hasher.update(context.link_stream_id.to_le_bytes());
    hasher.update(context.packet_index.to_le_bytes());
    hasher.update([context.direction.seed_byte()]);
    hasher.finalize().into()
}

fn uniform01(rng: &mut DeterministicRng) -> f64 {
    const SCALE: f64 = 1.0 / ((1u64 << 53) as f64);
    ((rng.next_u64() >> 11) as f64) * SCALE
}

/// Rise or set event extracted from fixed-step visibility samples.
#[derive(Clone, Debug, PartialEq)]
pub struct VisibilityEvent {
    /// Site id.
    pub site_id: String,
    /// Event time from linear interpolation, seconds.
    pub time_s: f64,
    /// Event kind.
    pub kind: VisibilityEventKind,
}

/// Visibility event type.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VisibilityEventKind {
    /// Visibility changed from false to true.
    Rise,
    /// Visibility changed from true to false.
    Set,
}

/// One continuous visible pass.
#[derive(Clone, Debug, PartialEq)]
pub struct PassInterval {
    /// Site id.
    pub site_id: String,
    /// Acquisition of signal time, seconds.
    pub aos_s: f64,
    /// Loss of signal time, seconds.
    pub los_s: f64,
    /// Maximum sampled elevation inside the pass, radians.
    pub max_elevation_rad: f64,
}

/// Extract rise/set events from time-ordered visibility samples.
///
/// # Errors
///
/// Returns [`CommError::NonMonotonicSamples`] when sample times are not
/// strictly increasing, or [`CommError::MixedSiteSamples`] when the input
/// stream contains adjacent samples for different sites.
pub fn visibility_events(samples: &[VisibilitySample]) -> Result<Vec<VisibilityEvent>, CommError> {
    let mut events = Vec::new();
    for pair in samples.windows(2) {
        let previous = &pair[0];
        let current = &pair[1];
        require_same_site(previous, current)?;
        if current.time_s <= previous.time_s {
            return Err(CommError::NonMonotonicSamples);
        }
        if previous.visible == current.visible {
            continue;
        }
        let time_s = interpolated_mask_crossing_time(previous, current);
        events.push(VisibilityEvent {
            site_id: current.site_id.clone(),
            time_s,
            kind: if current.visible {
                VisibilityEventKind::Rise
            } else {
                VisibilityEventKind::Set
            },
        });
    }
    Ok(events)
}

/// Extract pass intervals from time-ordered visibility samples.
///
/// # Errors
///
/// Returns [`CommError::NonMonotonicSamples`] when sample times are not
/// strictly increasing, or [`CommError::MixedSiteSamples`] when the input
/// stream contains adjacent samples for different sites.
pub fn pass_intervals(samples: &[VisibilitySample]) -> Result<Vec<PassInterval>, CommError> {
    if samples.is_empty() {
        return Ok(Vec::new());
    }
    let mut passes = Vec::new();
    let mut open_aos = samples[0].visible.then_some(samples[0].time_s);
    let mut max_elevation_rad = if samples[0].visible {
        samples[0].elevation_rad
    } else {
        f64::NEG_INFINITY
    };

    for pair in samples.windows(2) {
        let previous = &pair[0];
        let current = &pair[1];
        require_same_site(previous, current)?;
        if current.time_s <= previous.time_s {
            return Err(CommError::NonMonotonicSamples);
        }
        if previous.visible {
            max_elevation_rad = max_elevation_rad.max(previous.elevation_rad);
        }
        if previous.visible == current.visible {
            if current.visible {
                max_elevation_rad = max_elevation_rad.max(current.elevation_rad);
            }
            continue;
        }
        let event_time_s = interpolated_mask_crossing_time(previous, current);
        if current.visible {
            open_aos = Some(event_time_s);
            max_elevation_rad = current.elevation_rad;
        } else if let Some(aos_s) = open_aos.take() {
            passes.push(PassInterval {
                site_id: previous.site_id.clone(),
                aos_s,
                los_s: event_time_s,
                max_elevation_rad,
            });
            max_elevation_rad = f64::NEG_INFINITY;
        }
    }
    let last = &samples[samples.len() - 1];
    if last.visible {
        max_elevation_rad = max_elevation_rad.max(last.elevation_rad);
        if let Some(aos_s) = open_aos {
            passes.push(PassInterval {
                site_id: last.site_id.clone(),
                aos_s,
                los_s: last.time_s,
                max_elevation_rad,
            });
        }
    }
    Ok(passes)
}

/// Write pass intervals as deterministic CSV bytes.
///
/// The column order is `site_id,aos_s,los_s,max_elevation_rad`. Floating-point
/// values use `"{:.17e}"`, and site ids are RFC-4180 quoted only when needed.
///
/// # Errors
///
/// Returns the writer's [`std::io::Error`] if output fails.
pub fn write_pass_intervals_csv<W: Write>(
    passes: &[PassInterval],
    mut writer: W,
) -> std::io::Result<()> {
    writer.write_all(b"site_id,aos_s,los_s,max_elevation_rad\n")?;
    for pass in passes {
        write_csv_field(&mut writer, &pass.site_id)?;
        writeln!(
            writer,
            ",{:.17e},{:.17e},{:.17e}",
            pass.aos_s, pass.los_s, pass.max_elevation_rad
        )?;
    }
    Ok(())
}

fn require_same_site(
    previous: &VisibilitySample,
    current: &VisibilitySample,
) -> Result<(), CommError> {
    if previous.site_id == current.site_id {
        Ok(())
    } else {
        Err(CommError::MixedSiteSamples {
            previous: previous.site_id.clone(),
            current: current.site_id.clone(),
        })
    }
}

fn interpolated_mask_crossing_time(previous: &VisibilitySample, current: &VisibilitySample) -> f64 {
    let previous_margin = previous.elevation_rad - previous.mask_elevation_rad;
    let current_margin = current.elevation_rad - current.mask_elevation_rad;
    let denom = previous_margin - current_margin;
    let u = if denom.abs() <= f64::EPSILON {
        0.0
    } else {
        previous_margin / denom
    };
    previous.time_s + u * (current.time_s - previous.time_s)
}

fn require_finite(field: &'static str, value: f64) -> Result<(), CommError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(CommError::NonFinite { field })
    }
}

fn require_positive(field: &'static str, value: f64) -> Result<(), CommError> {
    require_finite(field, value)?;
    if value > 0.0 {
        Ok(())
    } else {
        Err(CommError::OutOfRange {
            field,
            value,
            rule: "must be positive",
        })
    }
}

fn require_non_negative(field: &'static str, value: f64) -> Result<(), CommError> {
    require_finite(field, value)?;
    if value >= 0.0 {
        Ok(())
    } else {
        Err(CommError::OutOfRange {
            field,
            value,
            rule: "must be non-negative",
        })
    }
}

fn require_finite_vector(field: &'static str, value: Vector3<f64>) -> Result<(), CommError> {
    require_finite(field, value.x)?;
    require_finite(field, value.y)?;
    require_finite(field, value.z)
}

fn unit_vector(field: &'static str, value: Vector3<f64>) -> Result<Vector3<f64>, CommError> {
    require_finite_vector(field, value)?;
    let norm = value.norm();
    if norm <= f64::EPSILON {
        return Err(CommError::Degenerate { field });
    }
    Ok(value / norm)
}

fn require_elevation(field: &'static str, value: f64) -> Result<(), CommError> {
    require_finite(field, value)?;
    if (-core::f64::consts::FRAC_PI_2..=core::f64::consts::FRAC_PI_2).contains(&value) {
        Ok(())
    } else {
        Err(CommError::OutOfRange {
            field,
            value,
            rule: "must be in [-pi/2, pi/2]",
        })
    }
}

fn wrap_pi(value: f64) -> f64 {
    (value + core::f64::consts::PI).rem_euclid(TWO_PI) - core::f64::consts::PI
}

fn wrap_2pi(value: f64) -> f64 {
    value.rem_euclid(TWO_PI)
}

fn write_csv_field<W: Write>(writer: &mut W, value: &str) -> std::io::Result<()> {
    let must_quote = value
        .as_bytes()
        .iter()
        .any(|b| matches!(b, b',' | b'"' | b'\n' | b'\r'));
    if must_quote {
        writer.write_all(b"\"")?;
        for byte in value.as_bytes() {
            if *byte == b'"' {
                writer.write_all(b"\"\"")?;
            } else {
                writer.write_all(std::slice::from_ref(byte))?;
            }
        }
        writer.write_all(b"\"")?;
    } else {
        writer.write_all(value.as_bytes())?;
    }
    Ok(())
}

fn ray_intersects_triangle(
    origin: Vector3<f64>,
    direction_unit: Vector3<f64>,
    triangle: BodyMaskTriangle,
) -> bool {
    let edge1 = triangle.b_m - triangle.a_m;
    let edge2 = triangle.c_m - triangle.a_m;
    let pvec = direction_unit.cross(&edge2);
    let det = edge1.dot(&pvec);
    if det.abs() <= 1.0e-12 {
        return false;
    }
    let inv_det = 1.0 / det;
    let tvec = origin - triangle.a_m;
    let u = tvec.dot(&pvec) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return false;
    }
    let qvec = tvec.cross(&edge1);
    let v = direction_unit.dot(&qvec) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return false;
    }
    edge2.dot(&qvec) * inv_det > 1.0e-12
}

fn hex_digest(digest: &[u8; 32]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(digest.len() * 2);
    for byte in *digest {
        out.push(char::from(HEX[usize::from(byte >> 4)]));
        out.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    out
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    const SECONDS_PER_JULIAN_YEAR: f64 = 365.25 * 86_400.0;
    const WGS84_EARTH_RATE_RAD_S: f64 = 7.292_115_146_706_979e-5;

    #[test]
    fn antenna_body_mask_deck_matches_fixture() {
        let fixture = antenna_body_mask_fixture();
        let tolerances = fixture
            .get("tolerances")
            .and_then(toml::Value::as_table)
            .expect("fixture tolerances table");
        let antenna = fixture
            .get("antenna")
            .and_then(toml::Value::as_table)
            .expect("fixture antenna table");
        let gain_samples = array_field(antenna, "samples")
            .iter()
            .map(|sample| {
                let sample = sample.as_table().expect("antenna sample table");
                AntennaGainSample {
                    off_boresight_rad: f64_field(sample, "off_boresight_deg").to_radians(),
                    gain_dbi: f64_field(sample, "gain_dbi"),
                }
            })
            .collect();
        let gain_deck =
            AntennaGainDeck::new(vector3_field(antenna, "boresight_body"), gain_samples).unwrap();

        for case in array_field(antenna, "gain_case") {
            let case = case.as_table().expect("antenna gain case table");
            let gain_dbi = gain_deck
                .gain_dbi_for_direction(vector3_field(case, "direction_body"))
                .unwrap();
            assert_abs_diff_eq!(
                gain_dbi,
                f64_field(case, "expected_gain_dbi"),
                epsilon = f64_field(tolerances, "gain_dbi")
            );
        }

        let body_mask = fixture
            .get("body_mask")
            .and_then(toml::Value::as_table)
            .expect("fixture body-mask table");
        let directions: Vec<_> = array_field(body_mask, "directions")
            .iter()
            .map(|entry| {
                vector3_field(
                    entry.as_table().expect("body-mask direction table"),
                    "direction_body",
                )
            })
            .collect();
        let expected_blocked: Vec<_> = array_field(body_mask, "directions")
            .iter()
            .map(|entry| {
                bool_field(
                    entry.as_table().expect("body-mask direction table"),
                    "expected_blocked",
                )
            })
            .collect();
        let triangles: Vec<_> = array_field(body_mask, "triangles")
            .iter()
            .map(|entry| {
                let entry = entry.as_table().expect("body-mask triangle table");
                BodyMaskTriangle {
                    a_m: vector3_field(entry, "a_m"),
                    b_m: vector3_field(entry, "b_m"),
                    c_m: vector3_field(entry, "c_m"),
                }
            })
            .collect();

        let deck = BodyMaskDeck::precompute_from_triangles(
            vector3_field(body_mask, "antenna_position_body_m"),
            &directions,
            &triangles,
        )
        .unwrap();
        assert_eq!(deck.samples().len(), expected_blocked.len());
        for (sample, expected) in deck.samples().iter().zip(expected_blocked) {
            assert_eq!(sample.blocked, expected);
        }
        assert!(
            deck.is_blocked_nearest(Vector3::new(1.0, 0.1, 0.0))
                .unwrap()
        );
        assert!(
            !deck
                .is_blocked_nearest(Vector3::new(0.0, 1.0, 0.0))
                .unwrap()
        );

        let mesh_sha256_hex = string_field(body_mask, "mesh_sha256");
        let digest = deck.sha256_hex_with_mesh_hash(&mesh_sha256_hex);
        let repeated = BodyMaskDeck::precompute_from_triangles(
            vector3_field(body_mask, "antenna_position_body_m"),
            &directions,
            &triangles,
        )
        .unwrap()
        .sha256_hex_with_mesh_hash(&mesh_sha256_hex);
        assert_eq!(digest, repeated);
        assert_eq!(digest.len(), 64);
        assert!(digest.bytes().all(|byte| byte.is_ascii_hexdigit()));
    }

    #[test]
    fn link_budget_and_fer_curve_match_fixture() {
        let fixture = link_budget_fer_fixture();
        let tolerances = fixture
            .get("tolerances")
            .and_then(toml::Value::as_table)
            .expect("fixture tolerances table");
        let curve = fixture
            .get("fer_curve")
            .and_then(toml::Value::as_table)
            .expect("fixture FER curve table");
        let samples = array_field(curve, "samples")
            .iter()
            .map(|sample| {
                let sample = sample.as_table().expect("FER sample table");
                FerCurveSample {
                    eb_n0_db: f64_field(sample, "eb_n0_db"),
                    fer: f64_field(sample, "fer"),
                }
            })
            .collect();
        let fer_curve = FerCurveDeck::new(string_field(curve, "code_id"), samples).unwrap();
        assert_eq!(fer_curve.code_id(), "synthetic-log-linear");
        assert_eq!(fer_curve.samples().len(), 3);
        assert!(
            fer_curve.fer_at_eb_n0_db(65.0).unwrap() < fer_curve.fer_at_eb_n0_db(55.0).unwrap()
        );
        assert!(matches!(
            fer_curve.fer_at_eb_n0_db(49.0),
            Err(CommError::OutOfRange { .. })
        ));

        let case = fixture
            .get("budget_case")
            .and_then(toml::Value::as_table)
            .expect("fixture budget case table");
        let input = LinkBudgetInput {
            eirp_dbw: f64_field(case, "eirp_dbw"),
            receiver_g_over_t_db_k: f64_field(case, "receiver_g_over_t_db_k"),
            slant_range_m: f64_field(case, "slant_range_m"),
            frequency_hz: f64_field(case, "frequency_hz"),
            bit_rate_bps: f64_field(case, "bit_rate_bps"),
            required_eb_n0_db: f64_field(case, "required_eb_n0_db"),
            atmospheric_loss_db: f64_field(case, "atmospheric_loss_db"),
            rain_loss_db: f64_field(case, "rain_loss_db"),
            pointing_loss_db: f64_field(case, "pointing_loss_db"),
            polarization_loss_db: f64_field(case, "polarization_loss_db"),
            implementation_loss_db: f64_field(case, "implementation_loss_db"),
            visible: bool_field(case, "visible"),
            blackout: bool_field(case, "blackout"),
        };
        let state = evaluate_link_budget(input, &fer_curve).unwrap();
        assert_abs_diff_eq!(
            state.free_space_loss_db,
            f64_field(case, "expected_free_space_loss_db"),
            epsilon = f64_field(tolerances, "db")
        );
        assert_abs_diff_eq!(
            state.c_n0_dbhz,
            f64_field(case, "expected_c_n0_dbhz"),
            epsilon = f64_field(tolerances, "db")
        );
        assert_abs_diff_eq!(
            state.eb_n0_db,
            f64_field(case, "expected_eb_n0_db"),
            epsilon = f64_field(tolerances, "db")
        );
        assert_abs_diff_eq!(
            state.margin_db,
            f64_field(case, "expected_margin_db"),
            epsilon = f64_field(tolerances, "db")
        );
        assert_abs_diff_eq!(
            state.fer,
            f64_field(case, "expected_fer"),
            epsilon = f64_field(tolerances, "fer")
        );
        assert!(state.visible);
        assert!(!state.blackout);

        let no_link = LinkBudgetInput {
            visible: false,
            ..input
        };
        assert_eq!(evaluate_link_budget(no_link, &fer_curve).unwrap().fer, 1.0);
    }

    #[test]
    fn elevation_loss_deck_matches_fixture() {
        let fixture = attenuation_loss_fixture();
        let tolerances = fixture
            .get("tolerances")
            .and_then(toml::Value::as_table)
            .expect("fixture tolerances table");
        let atmospheric = elevation_loss_deck_from_fixture(&fixture, "atmospheric_loss");
        let rain = elevation_loss_deck_from_fixture(&fixture, "rain_loss");
        assert_eq!(atmospheric.deck_id(), "synthetic-atmospheric-elevation");
        assert_eq!(rain.deck_id(), "synthetic-rain-elevation");
        assert_eq!(atmospheric.samples().len(), 3);
        assert_eq!(rain.samples().len(), 3);

        for case in fixture
            .get("loss_case")
            .and_then(toml::Value::as_array)
            .expect("loss cases")
        {
            let case = case.as_table().expect("loss case table");
            let deck = match string_field(case, "deck").as_str() {
                "atmospheric_loss" => &atmospheric,
                "rain_loss" => &rain,
                other => panic!("unexpected loss deck `{other}`"),
            };
            assert_abs_diff_eq!(
                deck.loss_db_at_elevation(f64_field(case, "elevation_deg").to_radians())
                    .unwrap(),
                f64_field(case, "expected_loss_db"),
                epsilon = f64_field(tolerances, "loss_db")
            );
        }
        assert!(matches!(
            ElevationLossDeck::new(
                "narrow",
                vec![
                    ElevationLossSample {
                        elevation_rad: 0.0,
                        loss_db: 1.0
                    },
                    ElevationLossSample {
                        elevation_rad: 10.0_f64.to_radians(),
                        loss_db: 0.8
                    },
                ],
            )
            .unwrap()
            .loss_db_at_elevation(20.0_f64.to_radians()),
            Err(CommError::OutOfRange { .. })
        ));
    }

    #[test]
    fn link_channel_model_matches_fixture() {
        let fixture = link_channel_fixture();
        let tolerances = fixture
            .get("tolerances")
            .and_then(toml::Value::as_table)
            .expect("fixture tolerances table");
        let case = fixture
            .get("packet_case")
            .and_then(toml::Value::as_table)
            .expect("fixture packet case table");
        let model =
            LinkChannelModel::new(f64_field(case, "processing_delay_s"), LinkErrorAction::Drop)
                .unwrap();
        let context = LinkPacketContext {
            scenario_seed: u64_field(case, "scenario_seed"),
            step: u64_field(case, "step"),
            link_stream_id: link_stream_id_from_id(&string_field(case, "link_id")).unwrap(),
            packet_index: u64_field(case, "packet_index"),
            direction: match string_field(case, "direction").as_str() {
                "downlink" => LinkPacketDirection::Downlink,
                "uplink" => LinkPacketDirection::Uplink,
                other => panic!("unexpected link packet direction `{other}`"),
            },
            slant_range_m: f64_field(case, "slant_range_m"),
        };
        let state = LinkState {
            fer: f64_field(case, "fer"),
            visible: bool_field(case, "visible"),
            blackout: bool_field(case, "blackout"),
            ..base_link_state()
        };

        let effect = model.evaluate_packet(state, context).unwrap();
        let repeated = model.evaluate_packet(state, context).unwrap();

        assert_eq!(effect, repeated);
        let expected_disposition = match string_field(case, "expected_disposition").as_str() {
            "deliver" => LinkPacketDisposition::Deliver,
            "drop" => LinkPacketDisposition::Drop,
            other => panic!("unexpected packet disposition `{other}`"),
        };
        assert_eq!(effect.disposition, expected_disposition);
        assert!(effect.error_draw.is_some());
        assert_abs_diff_eq!(
            effect.propagation_delay_s,
            f64_field(case, "expected_propagation_delay_s"),
            epsilon = f64_field(tolerances, "latency_s")
        );
        assert_abs_diff_eq!(
            effect.latency_s,
            f64_field(case, "expected_latency_s"),
            epsilon = f64_field(tolerances, "latency_s")
        );
    }

    #[test]
    fn link_channel_model_drops_no_link_and_validates_inputs() {
        let model = LinkChannelModel::default();
        let context = LinkPacketContext {
            scenario_seed: 7,
            step: 11,
            link_stream_id: link_stream_id_from_id("s-band-equator").unwrap(),
            packet_index: 0,
            direction: LinkPacketDirection::Downlink,
            slant_range_m: 1000.0,
        };
        let no_visibility = LinkState {
            visible: false,
            fer: 0.0,
            ..base_link_state()
        };
        let effect = model.evaluate_packet(no_visibility, context).unwrap();
        assert_eq!(effect.disposition, LinkPacketDisposition::Drop);
        assert_eq!(effect.error_draw, None);

        assert!(matches!(
            LinkChannelModel::new(0.0, LinkErrorAction::BitFlip { mask: 0 }),
            Err(CommError::OutOfRange { .. })
        ));
        assert!(matches!(
            model.evaluate_packet(
                LinkState {
                    fer: 1.5,
                    ..base_link_state()
                },
                context
            ),
            Err(CommError::OutOfRange { .. })
        ));
        assert!(matches!(
            model.evaluate_packet(
                base_link_state(),
                LinkPacketContext {
                    slant_range_m: 0.0,
                    ..context
                }
            ),
            Err(CommError::OutOfRange { .. })
        ));
    }

    #[test]
    fn link_channel_model_domain_separates_draws_and_bitflip_errors() {
        let model = LinkChannelModel::new(0.01, LinkErrorAction::BitFlip { mask: 0xa5 }).unwrap();
        let context = LinkPacketContext {
            scenario_seed: 99,
            step: 42,
            link_stream_id: link_stream_id_from_id("s-band-equator").unwrap(),
            packet_index: 3,
            direction: LinkPacketDirection::Downlink,
            slant_range_m: SPEED_OF_LIGHT_M_S,
        };
        let state = LinkState {
            fer: 1.0,
            ..base_link_state()
        };

        let effect = model.evaluate_packet(state, context).unwrap();
        let next_packet = model
            .evaluate_packet(
                state,
                LinkPacketContext {
                    packet_index: context.packet_index + 1,
                    ..context
                },
            )
            .unwrap();
        let uplink = model
            .evaluate_packet(
                state,
                LinkPacketContext {
                    direction: LinkPacketDirection::Uplink,
                    ..context
                },
            )
            .unwrap();

        assert_eq!(
            effect.disposition,
            LinkPacketDisposition::BitFlip { mask: 0xa5 }
        );
        assert_ne!(effect.error_draw, next_packet.error_draw);
        assert_ne!(effect.error_draw, uplink.error_draw);
        assert_abs_diff_eq!(effect.propagation_delay_s, 1.0, epsilon = 0.0);
        assert_abs_diff_eq!(effect.latency_s, 1.01, epsilon = 0.0);
    }

    #[test]
    fn link_loss_rate_gate_accepts_expected_fer_stream() {
        let model = LinkChannelModel::default();
        let context = LinkPacketContext {
            scenario_seed: 1234,
            step: 9,
            link_stream_id: link_stream_id_from_id("s-band-stat-gate").unwrap(),
            packet_index: 0,
            direction: LinkPacketDirection::Downlink,
            slant_range_m: 250_000.0,
        };
        let state = LinkState {
            fer: 0.2,
            ..base_link_state()
        };
        let gate = evaluate_link_loss_rate_gate(model, state, context, 2048, 0.001).unwrap();

        assert!(gate.passed(), "loss-rate gate rejected {gate:?}");
        assert_eq!(gate.packet_count, 2048);
        assert_abs_diff_eq!(gate.expected_error_rate, 0.2, epsilon = 0.0);
        assert_abs_diff_eq!(
            gate.observed_error_rate,
            gate.errored_packet_count as f64 / gate.packet_count as f64,
            epsilon = 0.0
        );
        assert!(gate.lower_accepted_errors <= gate.errored_packet_count);
        assert!(gate.errored_packet_count <= gate.upper_accepted_errors);
    }

    #[test]
    fn link_loss_rate_gate_handles_no_link_and_validates_inputs() {
        let model = LinkChannelModel::default();
        let context = LinkPacketContext {
            scenario_seed: 5,
            step: 1,
            link_stream_id: link_stream_id_from_id("s-band-no-link").unwrap(),
            packet_index: 3,
            direction: LinkPacketDirection::Uplink,
            slant_range_m: 1_000.0,
        };
        let no_link = LinkState {
            visible: false,
            fer: 0.0,
            ..base_link_state()
        };
        let gate = evaluate_link_loss_rate_gate(model, no_link, context, 16, 0.05).unwrap();
        assert!(gate.passed(), "no-link gate rejected {gate:?}");
        assert_abs_diff_eq!(gate.expected_error_rate, 1.0, epsilon = 0.0);
        assert_eq!(gate.errored_packet_count, 16);
        assert_eq!(gate.lower_accepted_errors, 16);
        assert_eq!(gate.upper_accepted_errors, 16);

        assert!(matches!(
            evaluate_link_loss_rate_gate(model, base_link_state(), context, 0, 0.05),
            Err(CommError::OutOfRange { .. })
        ));
        assert!(matches!(
            evaluate_link_loss_rate_gate(model, base_link_state(), context, 8, 1.0),
            Err(CommError::OutOfRange { .. })
        ));
    }

    #[test]
    fn sgp4_visibility_matches_external_oracle_tolerance_table() {
        let fixture = sgp4_visibility_fixture();
        let tolerances = fixture
            .get("tolerances")
            .and_then(toml::Value::as_table)
            .expect("fixture tolerances table");
        let site = site_from_fixture(
            fixture
                .get("site")
                .and_then(toml::Value::as_table)
                .expect("fixture site table"),
        );
        let case = fixture
            .get("leo_reference")
            .and_then(toml::Value::as_table)
            .expect("fixture LEO reference case");
        let elements = sgp4::Elements::from_tle(
            Some(string_field(case, "object_name")),
            string_field(case, "tle_line1").as_bytes(),
            string_field(case, "tle_line2").as_bytes(),
        )
        .unwrap();
        let constants = sgp4::Constants::from_elements(&elements).unwrap();
        let samples = sgp4_visibility_samples(&site, &elements, &constants, case);
        let events = visibility_events(&samples).unwrap();
        let oracle_events = refined_sgp4_visibility_events(&site, &elements, &constants, &samples);

        assert!(
            events.len() >= usize_field(case, "minimum_event_count"),
            "fixture window should include at least the requested number of SGP4 visibility events"
        );
        assert_eq!(events.len(), oracle_events.len());
        for (event, oracle_event) in events.iter().zip(&oracle_events) {
            assert_eq!(event.kind, oracle_event.kind);
            assert_abs_diff_eq!(
                event.time_s,
                oracle_event.time_s,
                epsilon = f64_field(tolerances, "event_time_s")
            );
        }

        assert!(
            !pass_intervals(&samples).unwrap().is_empty(),
            "fixture window should produce at least one LEO visibility pass"
        );
    }

    #[test]
    fn geometric_visibility_matches_oracle_tolerance_table() {
        let fixture = geometric_visibility_fixture();
        let tolerances = fixture
            .get("tolerances")
            .and_then(toml::Value::as_table)
            .expect("fixture tolerances table");
        let site = site_from_fixture(
            fixture
                .get("site")
                .and_then(toml::Value::as_table)
                .expect("fixture site table"),
        );
        let slant_range_tolerance_m = f64_field(tolerances, "slant_range_m");
        let range_rate_tolerance_m_s = f64_field(tolerances, "range_rate_m_s");
        let angle_tolerance_rad = f64_field(tolerances, "angle_rad");

        for case in fixture
            .get("geometry_case")
            .and_then(toml::Value::as_array)
            .expect("fixture geometry cases")
        {
            let case = case.as_table().expect("geometry case table");
            let sample = site.observe_ecef(
                f64_field(case, "time_s"),
                position_from_enu_offset(&site, vector3_field(case, "offset_enu_m")),
                vector_from_enu(&site, vector3_field(case, "velocity_enu_m_s")),
            );

            assert_abs_diff_eq!(
                sample.slant_range_m,
                f64_field(case, "expected_slant_range_m"),
                epsilon = slant_range_tolerance_m
            );
            assert_abs_diff_eq!(
                sample.range_rate_m_s,
                f64_field(case, "expected_range_rate_m_s"),
                epsilon = range_rate_tolerance_m_s
            );
            assert_abs_diff_eq!(
                sample.elevation_rad,
                f64_field(case, "expected_elevation_deg").to_radians(),
                epsilon = angle_tolerance_rad
            );
            assert_abs_diff_eq!(
                angle_delta_rad(
                    sample.azimuth_rad,
                    f64_field(case, "expected_azimuth_deg").to_radians()
                ),
                0.0,
                epsilon = angle_tolerance_rad
            );
            assert_abs_diff_eq!(
                sample.mask_elevation_rad,
                f64_field(case, "expected_mask_elevation_deg").to_radians(),
                epsilon = angle_tolerance_rad
            );
            assert_eq!(sample.visible, bool_field(case, "expected_visible"));
        }

        let pass_case = fixture
            .get("line_scan_pass_case")
            .and_then(toml::Value::as_array)
            .and_then(|cases| cases.first())
            .and_then(toml::Value::as_table)
            .expect("line-scan pass case table");
        let samples = line_scan_samples(&site, pass_case);
        let events = visibility_events(&samples).unwrap();
        let passes = pass_intervals(&samples).unwrap();

        assert_eq!(events.len(), 2);
        assert_eq!(events[0].kind, VisibilityEventKind::Rise);
        assert_eq!(events[1].kind, VisibilityEventKind::Set);
        assert_abs_diff_eq!(
            events[0].time_s,
            f64_field(pass_case, "expected_aos_s"),
            epsilon = f64_field(tolerances, "event_time_s")
        );
        assert_abs_diff_eq!(
            events[1].time_s,
            f64_field(pass_case, "expected_los_s"),
            epsilon = f64_field(tolerances, "event_time_s")
        );
        assert_eq!(passes.len(), 1);
        assert_abs_diff_eq!(
            passes[0].aos_s,
            f64_field(pass_case, "expected_aos_s"),
            epsilon = f64_field(tolerances, "event_time_s")
        );
        assert_abs_diff_eq!(
            passes[0].los_s,
            f64_field(pass_case, "expected_los_s"),
            epsilon = f64_field(tolerances, "event_time_s")
        );
        assert_abs_diff_eq!(
            passes[0].max_elevation_rad,
            f64_field(pass_case, "expected_max_elevation_deg").to_radians(),
            epsilon = angle_tolerance_rad
        );
    }

    #[test]
    fn overhead_site_geometry_matches_analytic_range() {
        let site = GroundSite::new(
            "ksc",
            GeodeticPosition::new(0.0, 0.0, 0.0).unwrap(),
            5.0_f64.to_radians(),
            None,
        )
        .unwrap();
        let position = site.ecef_m() + Vector3::new(1000.0, 0.0, 0.0);
        let sample = site.observe_ecef(10.0, position, Vector3::new(0.0, 10.0, 0.0));

        assert_abs_diff_eq!(sample.slant_range_m, 1000.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(
            sample.elevation_rad,
            core::f64::consts::FRAC_PI_2,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(sample.range_rate_m_s, 0.0, epsilon = 1.0e-12);
        assert!(sample.visible);
    }

    #[test]
    fn terrain_mask_interpolates_across_wrap() {
        let mask = MaskDeck::new(vec![
            MaskDeckBin {
                azimuth_rad: 350.0_f64.to_radians(),
                min_elevation_rad: 10.0_f64.to_radians(),
            },
            MaskDeckBin {
                azimuth_rad: 10.0_f64.to_radians(),
                min_elevation_rad: 20.0_f64.to_radians(),
            },
        ])
        .unwrap();

        assert_abs_diff_eq!(
            mask.min_elevation_at(0.0),
            15.0_f64.to_radians(),
            epsilon = 1.0e-15
        );
    }

    #[test]
    fn visibility_events_and_passes_are_deterministic() {
        let samples = [
            sample(0.0, -1.0),
            sample(10.0, 1.0),
            sample(20.0, 3.0),
            sample(30.0, -1.0),
        ];

        let events = visibility_events(&samples).unwrap();
        assert_eq!(events.len(), 2);
        assert_eq!(events[0].site_id, "site");
        assert_eq!(events[0].kind, VisibilityEventKind::Rise);
        assert_abs_diff_eq!(events[0].time_s, 5.0, epsilon = 0.0);
        assert_eq!(events[1].site_id, "site");
        assert_eq!(events[1].kind, VisibilityEventKind::Set);
        assert_abs_diff_eq!(events[1].time_s, 27.5, epsilon = 0.0);

        let passes = pass_intervals(&samples).unwrap();
        assert_eq!(passes.len(), 1);
        assert_eq!(passes[0].site_id, "site");
        assert_abs_diff_eq!(passes[0].aos_s, 5.0, epsilon = 0.0);
        assert_abs_diff_eq!(passes[0].los_s, 27.5, epsilon = 0.0);
        assert_abs_diff_eq!(passes[0].max_elevation_rad, 3.0, epsilon = 0.0);
    }

    #[test]
    fn pass_intervals_keep_max_elevation_per_pass() {
        let samples = [
            sample(0.0, 1.0),
            sample(10.0, -1.0),
            sample(20.0, 4.0),
            sample(30.0, -1.0),
        ];

        let passes = pass_intervals(&samples).unwrap();

        assert_eq!(passes.len(), 2);
        assert_abs_diff_eq!(passes[0].max_elevation_rad, 1.0, epsilon = 0.0);
        assert_abs_diff_eq!(passes[1].max_elevation_rad, 4.0, epsilon = 0.0);
    }

    #[test]
    fn pass_extraction_rejects_mixed_site_samples() {
        let mut samples = [sample(0.0, -1.0), sample(10.0, 1.0)];
        samples[1].site_id = "other".to_owned();

        assert_eq!(
            visibility_events(&samples).unwrap_err(),
            CommError::MixedSiteSamples {
                previous: "site".to_owned(),
                current: "other".to_owned(),
            }
        );
        assert_eq!(
            pass_intervals(&samples).unwrap_err(),
            CommError::MixedSiteSamples {
                previous: "site".to_owned(),
                current: "other".to_owned(),
            }
        );
    }

    #[test]
    fn pass_extraction_rejects_non_monotonic_samples() {
        let samples = [sample(1.0, 1.0), sample(1.0, -1.0)];
        assert_eq!(
            visibility_events(&samples).unwrap_err(),
            CommError::NonMonotonicSamples
        );
    }

    #[test]
    fn pass_interval_csv_is_byte_stable() {
        let passes = [PassInterval {
            site_id: "ksc,west\"pad".to_owned(),
            aos_s: 5.0,
            los_s: 27.5,
            max_elevation_rad: 3.0,
        }];
        let mut bytes = Vec::new();

        write_pass_intervals_csv(&passes, &mut bytes).unwrap();

        assert_eq!(
            std::str::from_utf8(&bytes).unwrap(),
            "site_id,aos_s,los_s,max_elevation_rad\n\"ksc,west\"\"pad\",5.00000000000000000e0,2.75000000000000000e1,3.00000000000000000e0\n"
        );
    }

    fn antenna_body_mask_fixture() -> toml::Value {
        toml::from_str(include_str!("../../../data/comm/antenna-body-mask-v1.toml")).unwrap()
    }

    fn link_budget_fer_fixture() -> toml::Value {
        toml::from_str(include_str!("../../../data/comm/link-budget-fer-v1.toml")).unwrap()
    }

    fn attenuation_loss_fixture() -> toml::Value {
        toml::from_str(include_str!("../../../data/comm/attenuation-loss-v1.toml")).unwrap()
    }

    fn link_channel_fixture() -> toml::Value {
        toml::from_str(include_str!(
            "../../../data/comm/link-channel-model-v1.toml"
        ))
        .unwrap()
    }

    fn elevation_loss_deck_from_fixture(fixture: &toml::Value, table: &str) -> ElevationLossDeck {
        let table = fixture
            .get(table)
            .and_then(toml::Value::as_table)
            .expect("elevation loss table");
        let samples = array_field(table, "samples")
            .iter()
            .map(|sample| {
                let sample = sample.as_table().expect("loss sample table");
                ElevationLossSample {
                    elevation_rad: f64_field(sample, "elevation_deg").to_radians(),
                    loss_db: f64_field(sample, "loss_db"),
                }
            })
            .collect();
        ElevationLossDeck::new(string_field(table, "deck_id"), samples).unwrap()
    }

    fn sgp4_visibility_fixture() -> toml::Value {
        toml::from_str(include_str!(
            "../../../data/comm/sgp4-visibility-oracle-v1.toml"
        ))
        .unwrap()
    }

    fn geometric_visibility_fixture() -> toml::Value {
        toml::from_str(include_str!(
            "../../../data/comm/geometric-visibility-oracle-v1.toml"
        ))
        .unwrap()
    }

    fn site_from_fixture(table: &toml::value::Table) -> GroundSite {
        GroundSite::new(
            string_field(table, "id"),
            GeodeticPosition::new(
                f64_field(table, "latitude_deg").to_radians(),
                f64_field(table, "longitude_deg").to_radians(),
                f64_field(table, "altitude_m"),
            )
            .unwrap(),
            f64_field(table, "min_elevation_deg").to_radians(),
            None,
        )
        .unwrap()
    }

    fn sgp4_visibility_samples(
        site: &GroundSite,
        elements: &sgp4::Elements,
        constants: &sgp4::Constants,
        case: &toml::value::Table,
    ) -> Vec<VisibilitySample> {
        let start_s = f64_field(case, "start_s");
        let end_s = f64_field(case, "end_s");
        let step_s = f64_field(case, "sample_step_s");
        let mut samples = Vec::new();
        let mut time_s = start_s;
        while time_s <= end_s + step_s * 0.5 {
            samples.push(sgp4_visibility_sample(site, elements, constants, time_s));
            time_s += step_s;
        }
        samples
    }

    fn refined_sgp4_visibility_events(
        site: &GroundSite,
        elements: &sgp4::Elements,
        constants: &sgp4::Constants,
        samples: &[VisibilitySample],
    ) -> Vec<VisibilityEvent> {
        let mut events = Vec::new();
        for pair in samples.windows(2) {
            let previous = &pair[0];
            let current = &pair[1];
            if previous.visible == current.visible {
                continue;
            }
            let mut lo = previous.time_s;
            let mut hi = current.time_s;
            let mut lo_margin = sgp4_visibility_margin(site, elements, constants, lo);
            for _ in 0..48 {
                let mid = 0.5 * (lo + hi);
                let mid_margin = sgp4_visibility_margin(site, elements, constants, mid);
                if (lo_margin >= 0.0) == (mid_margin >= 0.0) {
                    lo = mid;
                    lo_margin = mid_margin;
                } else {
                    hi = mid;
                }
            }
            events.push(VisibilityEvent {
                site_id: current.site_id.clone(),
                time_s: 0.5 * (lo + hi),
                kind: if current.visible {
                    VisibilityEventKind::Rise
                } else {
                    VisibilityEventKind::Set
                },
            });
        }
        events
    }

    fn sgp4_visibility_margin(
        site: &GroundSite,
        elements: &sgp4::Elements,
        constants: &sgp4::Constants,
        time_s: f64,
    ) -> f64 {
        let sample = sgp4_visibility_sample(site, elements, constants, time_s);
        sample.elevation_rad - sample.mask_elevation_rad
    }

    fn sgp4_visibility_sample(
        site: &GroundSite,
        elements: &sgp4::Elements,
        constants: &sgp4::Constants,
        time_s: f64,
    ) -> VisibilitySample {
        let prediction = constants
            .propagate(sgp4::MinutesSinceEpoch(time_s / 60.0))
            .unwrap();
        let position_teme_m = Vector3::new(
            prediction.position[0] * 1000.0,
            prediction.position[1] * 1000.0,
            prediction.position[2] * 1000.0,
        );
        let velocity_teme_m_s = Vector3::new(
            prediction.velocity[0] * 1000.0,
            prediction.velocity[1] * 1000.0,
            prediction.velocity[2] * 1000.0,
        );
        let epoch = elements.epoch() + time_s / SECONDS_PER_JULIAN_YEAR;
        let theta = sgp4::iau_epoch_to_sidereal_time(epoch);
        let position_ecef_m = rotate_teme_to_ecef(position_teme_m, theta);
        let velocity_ecef_m_s = rotate_teme_to_ecef(velocity_teme_m_s, theta)
            - Vector3::new(
                -WGS84_EARTH_RATE_RAD_S * position_ecef_m.y,
                WGS84_EARTH_RATE_RAD_S * position_ecef_m.x,
                0.0,
            );
        site.observe_ecef(time_s, position_ecef_m, velocity_ecef_m_s)
    }

    fn rotate_teme_to_ecef(vector: Vector3<f64>, theta_rad: f64) -> Vector3<f64> {
        let (sin_theta, cos_theta) = theta_rad.sin_cos();
        Vector3::new(
            cos_theta * vector.x + sin_theta * vector.y,
            -sin_theta * vector.x + cos_theta * vector.y,
            vector.z,
        )
    }

    fn line_scan_samples(site: &GroundSite, case: &toml::value::Table) -> Vec<VisibilitySample> {
        let altitude_m = f64_field(case, "altitude_m");
        let speed_m_s = f64_field(case, "speed_m_s");
        let start_s = f64_field(case, "start_s");
        let end_s = f64_field(case, "end_s");
        let step_s = f64_field(case, "step_s");
        let mut samples = Vec::new();
        let mut time_s = start_s;
        while time_s <= end_s + step_s * 0.5 {
            samples.push(site.observe_ecef(
                time_s,
                position_from_enu_offset(site, Vector3::new(speed_m_s * time_s, 0.0, altitude_m)),
                vector_from_enu(site, Vector3::new(speed_m_s, 0.0, 0.0)),
            ));
            time_s += step_s;
        }
        samples
    }

    fn position_from_enu_offset(site: &GroundSite, enu_m: Vector3<f64>) -> Vector3<f64> {
        site.ecef_m() + vector_from_enu(site, enu_m)
    }

    fn vector_from_enu(site: &GroundSite, enu: Vector3<f64>) -> Vector3<f64> {
        let basis = site.geodetic.enu_basis_ecef();
        basis.east * enu.x + basis.north * enu.y + basis.up * enu.z
    }

    fn f64_field(table: &toml::value::Table, key: &str) -> f64 {
        let value = table
            .get(key)
            .unwrap_or_else(|| panic!("missing fixture key `{key}`"));
        value
            .as_float()
            .or_else(|| value.as_integer().map(|integer| integer as f64))
            .unwrap_or_else(|| panic!("fixture key `{key}` must be numeric"))
    }

    fn array_field<'a>(table: &'a toml::value::Table, key: &str) -> &'a [toml::Value] {
        table
            .get(key)
            .and_then(toml::Value::as_array)
            .map(Vec::as_slice)
            .unwrap_or_else(|| panic!("fixture key `{key}` must be an array"))
    }

    fn usize_field(table: &toml::value::Table, key: &str) -> usize {
        let value = table
            .get(key)
            .and_then(toml::Value::as_integer)
            .unwrap_or_else(|| panic!("fixture key `{key}` must be an integer"));
        usize::try_from(value).unwrap_or_else(|_| panic!("fixture key `{key}` must fit usize"))
    }

    fn u64_field(table: &toml::value::Table, key: &str) -> u64 {
        let value = table
            .get(key)
            .and_then(toml::Value::as_integer)
            .unwrap_or_else(|| panic!("fixture key `{key}` must be an integer"));
        u64::try_from(value).unwrap_or_else(|_| panic!("fixture key `{key}` must fit u64"))
    }

    fn bool_field(table: &toml::value::Table, key: &str) -> bool {
        table
            .get(key)
            .and_then(toml::Value::as_bool)
            .unwrap_or_else(|| panic!("fixture key `{key}` must be boolean"))
    }

    fn string_field(table: &toml::value::Table, key: &str) -> String {
        table
            .get(key)
            .and_then(toml::Value::as_str)
            .unwrap_or_else(|| panic!("fixture key `{key}` must be a string"))
            .to_owned()
    }

    fn vector3_field(table: &toml::value::Table, key: &str) -> Vector3<f64> {
        let values = table
            .get(key)
            .and_then(toml::Value::as_array)
            .unwrap_or_else(|| panic!("fixture key `{key}` must be an array"));
        assert_eq!(values.len(), 3, "fixture vector `{key}` must have 3 values");
        Vector3::new(
            values[0]
                .as_float()
                .or_else(|| values[0].as_integer().map(|integer| integer as f64))
                .unwrap(),
            values[1]
                .as_float()
                .or_else(|| values[1].as_integer().map(|integer| integer as f64))
                .unwrap(),
            values[2]
                .as_float()
                .or_else(|| values[2].as_integer().map(|integer| integer as f64))
                .unwrap(),
        )
    }

    fn angle_delta_rad(actual_rad: f64, expected_rad: f64) -> f64 {
        (actual_rad - expected_rad + core::f64::consts::PI).rem_euclid(TWO_PI)
            - core::f64::consts::PI
    }

    fn base_link_state() -> LinkState {
        LinkState {
            free_space_loss_db: 100.0,
            c_n0_dbhz: 80.0,
            eb_n0_db: 50.0,
            margin_db: 10.0,
            fer: 0.0,
            visible: true,
            blackout: false,
        }
    }

    fn sample(time_s: f64, margin_rad: f64) -> VisibilitySample {
        VisibilitySample {
            time_s,
            site_id: "site".to_owned(),
            slant_range_m: 1.0,
            range_rate_m_s: 0.0,
            elevation_rad: margin_rad,
            azimuth_rad: 0.0,
            mask_elevation_rad: 0.0,
            visible: margin_rad >= 0.0,
        }
    }
}
