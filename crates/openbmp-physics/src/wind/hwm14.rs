//! HWM14 horizontal wind model.
//!
//! This module evaluates the public HWM14 quiet-time coefficient model
//! and DWM07 disturbance wind model directly from the bundled HWM14
//! data files. The Rust implementation is a deterministic port of the
//! public `hwm14.f90` evaluator: no runtime file I/O, no Fortran build
//! dependency, and no external crate dependency.
//!
//! The model returns meridional and zonal winds in m/s. OpenBMP maps
//! these to local-NED `(north, east, down)` as `(meridional, zonal, 0)`.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::float_cmp,
    clippy::manual_midpoint,
    clippy::many_single_char_names,
    clippy::needless_range_loop,
    clippy::range_minus_one,
    clippy::similar_names,
    clippy::too_many_lines
)]

use std::sync::OnceLock;

use openbmp_core::{Eci, Ned, Position3, SimTime, Velocity3};

use crate::error::PhysicsError;
use crate::frames::FrameContext;

use super::WindModel;

const HWM14_QUIET_DATA: &[u8] = include_bytes!("../../../../data/wind/hwm14/hwm123114.bin");
const HWM14_DWM_DATA: &[u8] = include_bytes!("../../../../data/wind/hwm14/dwm07b104i.dat");
const HWM14_GD2QD_DATA: &[u8] = include_bytes!("../../../../data/wind/hwm14/gd2qd.dat");

const TWO_PI: f64 = 2.0 * std::f64::consts::PI;
const DEG_TO_RAD: f64 = TWO_PI / 360.0;
const HWM14_HIGH_ALTITUDE_SCALE_KM: f64 = 60.0;

/// Highest tabulated altitude in the public HWM14 reference profile.
pub const HWM14_REFERENCE_MAX_ALTITUDE_M: f64 = 400_000.0;

/// One public `checkhwm14` reference-profile table row.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Hwm14ReferenceRow {
    /// Altitude above the launch-pad reference, m.
    pub altitude_m: f64,
    /// Meridional wind, positive northward, m/s.
    pub meridional_m_s: f64,
    /// Zonal wind, positive eastward, m/s.
    pub zonal_m_s: f64,
}

impl Hwm14ReferenceRow {
    const fn new(altitude_m: f64, meridional_m_s: f64, zonal_m_s: f64) -> Self {
        Self {
            altitude_m,
            meridional_m_s,
            zonal_m_s,
        }
    }
}

/// Public HWM14 `checkhwm14` total-wind height profile.
const HWM14_REFERENCE_PROFILE: [Hwm14ReferenceRow; 17] = [
    Hwm14ReferenceRow::new(0.0, 0.031, 6.271),
    Hwm14ReferenceRow::new(25_000.0, 2.965, 25.115),
    Hwm14ReferenceRow::new(50_000.0, -6.627, 96.343),
    Hwm14ReferenceRow::new(75_000.0, 2.238, 44.845),
    Hwm14ReferenceRow::new(100_000.0, -14.253, 31.590),
    Hwm14ReferenceRow::new(125_000.0, 37.403, 11.628),
    Hwm14ReferenceRow::new(150_000.0, 42.789, -33.319),
    Hwm14ReferenceRow::new(175_000.0, 20.278, -49.984),
    Hwm14ReferenceRow::new(200_000.0, 25.027, -68.588),
    Hwm14ReferenceRow::new(225_000.0, 34.297, -80.022),
    Hwm14ReferenceRow::new(250_000.0, 40.408, -87.560),
    Hwm14ReferenceRow::new(275_000.0, 44.436, -92.530),
    Hwm14ReferenceRow::new(300_000.0, 47.092, -95.806),
    Hwm14ReferenceRow::new(325_000.0, 48.843, -97.965),
    Hwm14ReferenceRow::new(350_000.0, 49.997, -99.389),
    Hwm14ReferenceRow::new(375_000.0, 50.758, -100.327),
    Hwm14ReferenceRow::new(400_000.0, 51.259, -100.946),
];

/// HWM14 fixed-input configuration.
///
/// The public Fortran interface takes `IYD`, UTC seconds, geodetic
/// latitude/longitude, and current 3-hour Ap. HWM14 ignores F10.7.
/// A negative `ap_current_3h` selects quiet-time winds only; a
/// non-negative value adds DWM07 disturbance winds.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Hwm14Inputs {
    /// Calendar year. HWM14 only uses the day-of-year part, but the
    /// field keeps the scenario surface aligned with atmosphere inputs.
    pub year: u16,
    /// Day of year. HWM14's public verification driver permits day 0.
    pub day_of_year: u16,
    /// UTC seconds within the day.
    pub utc_seconds: f64,
    /// Geodetic latitude in radians.
    pub latitude_rad: f64,
    /// Geodetic longitude in radians.
    pub longitude_rad: f64,
    /// Current 3-hour Ap index. `-1` selects quiet-time winds only.
    pub ap_current_3h: f64,
}

impl Hwm14Inputs {
    /// Public `checkhwm14` height-profile case.
    #[must_use]
    pub fn reference_conditions() -> Self {
        Self::from_degrees(1995, 150, 43_200.0, -45.0, -85.0, 80.0)
    }

    /// Public `checkhwm14` height-profile quiet-time case.
    #[must_use]
    pub fn quiet_reference_conditions() -> Self {
        Self::from_degrees(1995, 150, 43_200.0, -45.0, -85.0, -1.0)
    }

    /// Construct from degrees for scenario and test ergonomics.
    #[must_use]
    pub fn from_degrees(
        year: u16,
        day_of_year: u16,
        utc_seconds: f64,
        latitude_deg: f64,
        longitude_deg: f64,
        ap_current_3h: f64,
    ) -> Self {
        Self {
            year,
            day_of_year,
            utc_seconds,
            latitude_rad: latitude_deg.to_radians(),
            longitude_rad: longitude_deg.to_radians(),
            ap_current_3h,
        }
    }

    /// Geodetic latitude in degrees.
    #[must_use]
    pub fn latitude_deg(self) -> f64 {
        self.latitude_rad.to_degrees()
    }

    /// Geodetic longitude in degrees.
    #[must_use]
    pub fn longitude_deg(self) -> f64 {
        self.longitude_rad.to_degrees()
    }

    /// Validate the HWM14 input envelope.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when any field is
    /// outside the deterministic HWM14 interface envelope.
    pub fn validate(self) -> Result<(), PhysicsError> {
        if self.day_of_year > 366 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Hwm14Inputs day_of_year must be in 0..=366",
            });
        }
        if !self.utc_seconds.is_finite() || !(0.0..86_400.0).contains(&self.utc_seconds) {
            return Err(PhysicsError::InvalidParameter {
                reason: "Hwm14Inputs utc_seconds must be finite and in [0, 86400)",
            });
        }
        if !self.latitude_rad.is_finite() || self.latitude_rad.abs() > std::f64::consts::FRAC_PI_2 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Hwm14Inputs latitude must be finite and in [-90, 90] degrees",
            });
        }
        if !self.longitude_rad.is_finite() || self.longitude_rad.abs() > std::f64::consts::PI {
            return Err(PhysicsError::InvalidParameter {
                reason: "Hwm14Inputs longitude must be finite and in [-180, 180] degrees",
            });
        }
        if !self.ap_current_3h.is_finite() || !((-1.0..=400.0).contains(&self.ap_current_3h)) {
            return Err(PhysicsError::InvalidParameter {
                reason: "Hwm14Inputs ap_current_3h must be -1 or in [0, 400]",
            });
        }
        Ok(())
    }
}

impl Default for Hwm14Inputs {
    fn default() -> Self {
        Self::reference_conditions()
    }
}

/// Full HWM14 total-wind model backed by bundled HWM14 data files.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Hwm14Wind {
    inputs: Hwm14Inputs,
}

impl Hwm14Wind {
    /// Construct an HWM14 model from fixed scenario inputs.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when inputs are out
    /// of envelope.
    pub fn new(inputs: Hwm14Inputs) -> Result<Self, PhysicsError> {
        inputs.validate()?;
        Ok(Self { inputs })
    }

    /// Construct the public `checkhwm14` reference-condition model.
    #[must_use]
    pub fn reference_conditions() -> Self {
        Self {
            inputs: Hwm14Inputs::reference_conditions(),
        }
    }

    /// The fixed inputs used by this model.
    #[must_use]
    pub const fn inputs(self) -> Hwm14Inputs {
        self.inputs
    }

    /// Public `checkhwm14` total-wind reference rows.
    #[must_use]
    pub const fn reference_profile() -> &'static [Hwm14ReferenceRow] {
        &HWM14_REFERENCE_PROFILE
    }

    /// Evaluate HWM14 at an altitude in metres.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::NonFinite`] for non-finite altitude or
    /// output, and [`PhysicsError::InvalidParameter`] if bundled data
    /// validation fails.
    pub fn evaluate_altitude_m(self, altitude_m: f64) -> Result<Velocity3<Ned>, PhysicsError> {
        if !altitude_m.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "Hwm14Wind altitude proxy (position_eci.z) is non-finite",
            });
        }
        let model = hwm14_data()?;
        let output = model.evaluate(self.inputs, altitude_m / 1000.0);
        let wind = Velocity3::new(
            f64::from(output.meridional_m_s),
            f64::from(output.zonal_m_s),
            0.0,
        );
        if !wind.vector.x.is_finite() || !wind.vector.y.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "Hwm14Wind produced non-finite horizontal wind",
            });
        }
        Ok(wind)
    }
}

impl Default for Hwm14Wind {
    fn default() -> Self {
        Self::reference_conditions()
    }
}

impl WindModel for Hwm14Wind {
    fn wind_ned_m_s(
        &self,
        position_eci: Position3<Eci>,
        _frame: &FrameContext,
        _time: SimTime,
    ) -> Result<Velocity3<Ned>, PhysicsError> {
        // Same altitude proxy as LayeredWind: position_eci.vector.z.
        // A future geodetic-altitude resolver should replace both in
        // lockstep.
        self.evaluate_altitude_m(position_eci.vector.z)
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct Hwm14Output {
    meridional_m_s: f32,
    zonal_m_s: f32,
}

#[derive(Debug)]
struct Hwm14Data {
    quiet: QuietModel,
    disturbance: DisturbanceModel,
    geomagnetic: GeomagneticModel,
    alf: AlfCoefficients,
}

impl Hwm14Data {
    fn parse() -> Result<Self, &'static str> {
        let quiet = QuietModel::parse(HWM14_QUIET_DATA)?;
        let disturbance = DisturbanceModel::parse(HWM14_DWM_DATA)?;
        let geomagnetic = GeomagneticModel::parse(HWM14_GD2QD_DATA)?;
        let max_n = quiet.maxn.max(disturbance.nmax).max(geomagnetic.nmax);
        let max_m = quiet.maxo.max(disturbance.mmax).max(geomagnetic.mmax);
        let alf = AlfCoefficients::new(max_n, max_m);
        Ok(Self {
            quiet,
            disturbance,
            geomagnetic,
            alf,
        })
    }

    fn evaluate(&self, inputs: Hwm14Inputs, altitude_km: f64) -> Hwm14Output {
        let scalar = ScalarInputs::from(inputs, altitude_km);
        let mut output = self.quiet.evaluate(&self.alf, scalar);
        if scalar.ap_current_3h >= 0.0 {
            let disturbed = self
                .disturbance
                .evaluate(&self.alf, &self.geomagnetic, scalar);
            output.meridional_m_s += disturbed.meridional_m_s;
            output.zonal_m_s += disturbed.zonal_m_s;
        }
        output
    }
}

fn hwm14_data() -> Result<&'static Hwm14Data, PhysicsError> {
    static DATA: OnceLock<Result<Hwm14Data, &'static str>> = OnceLock::new();
    DATA.get_or_init(Hwm14Data::parse)
        .as_ref()
        .map_err(|reason| PhysicsError::InvalidParameter { reason })
}

#[derive(Copy, Clone, Debug)]
struct ScalarInputs {
    day_of_year: i32,
    utc_seconds: f32,
    altitude_km: f32,
    latitude_deg: f32,
    longitude_deg: f32,
    ap_current_3h: f32,
}

impl ScalarInputs {
    fn from(inputs: Hwm14Inputs, altitude_km: f64) -> Self {
        Self {
            day_of_year: i32::from(inputs.day_of_year),
            utc_seconds: inputs.utc_seconds as f32,
            altitude_km: altitude_km as f32,
            latitude_deg: inputs.latitude_deg() as f32,
            longitude_deg: inputs.longitude_deg() as f32,
            ap_current_3h: inputs.ap_current_3h as f32,
        }
    }
}

#[derive(Debug)]
struct QuietModel {
    nbf: usize,
    maxs: usize,
    maxm: usize,
    maxl: usize,
    maxn: usize,
    maxo: usize,
    p: usize,
    nnode: usize,
    alttns: f64,
    e1: [f64; 5],
    e2: [f64; 5],
    nb: Vec<usize>,
    order: Vec<[usize; 9]>,
    vnode: Vec<f64>,
    mparm: Vec<Vec<f64>>,
    tparm: Vec<Vec<f64>>,
}

impl QuietModel {
    fn parse(bytes: &[u8]) -> Result<Self, &'static str> {
        let mut reader = BinaryReader::new(bytes);
        let nbf = reader.read_i32()? as usize;
        let maxs = reader.read_i32()? as usize;
        let maxm = reader.read_i32()? as usize;
        let maxl = reader.read_i32()? as usize;
        let maxn = reader.read_i32()? as usize;
        let ncomp = reader.read_i32()? as usize;
        let nlev = reader.read_i32()? as usize;
        let p = reader.read_i32()? as usize;
        if ncomp != 9 || nbf == 0 || p == 0 {
            return Err("invalid HWM14 quiet-model header");
        }
        let nnode = nlev + p;
        let mut vnode = vec![0.0; nnode + 1];
        for value in &mut vnode {
            *value = reader.read_f64()?;
        }
        vnode[3] = 0.0;

        let mut nb = vec![0_usize; nnode + 1];
        let mut order = vec![[0_usize; 9]; nnode + 1];
        let mut mparm = vec![vec![0.0; nbf + 8]; nlev + 1];
        let mut tparm = vec![vec![0.0; nbf + 8]; nlev + 1];
        let last_level = nlev - p + 1 - 2;
        for level in 0..=last_level {
            for item in &mut order[level] {
                *item = reader.read_i32()? as usize;
            }
            nb[level] = reader.read_i32()? as usize;
            for parameter in &mut mparm[level][1..=nbf] {
                *parameter = reader.read_f64()?;
            }
            apply_parity(
                order[level],
                nb[level],
                &mut mparm[level],
                &mut tparm[level],
            );
        }
        let e1 = reader.read_f64_array::<5>()?;
        let e2 = reader.read_f64_array::<5>()?;
        reader.finish()?;
        Ok(Self {
            nbf,
            maxs,
            maxm,
            maxl,
            maxn,
            maxo: maxs.max(maxm).max(maxl),
            p,
            nnode,
            alttns: vnode[nlev - 2],
            e1,
            e2,
            nb,
            order,
            vnode,
            mparm,
            tparm,
        })
    }

    #[allow(clippy::many_single_char_names)]
    fn evaluate(&self, alf: &AlfCoefficients, inputs: ScalarInputs) -> Hwm14Output {
        let day = f64::from(inputs.day_of_year);
        let sec = f64::from(inputs.utc_seconds);
        let glon = f64::from(inputs.longitude_deg);
        let glat = f64::from(inputs.latitude_deg);
        let alt = f64::from(inputs.altitude_km);

        let mut fs = vec![[0.0_f64; 2]; self.maxs + 1];
        let aa = day * TWO_PI / 365.25;
        for (s, slot) in fs.iter_mut().enumerate() {
            let bb = s as f64 * aa;
            slot[0] = bb.cos();
            slot[1] = bb.sin();
        }

        let mut fl = vec![[0.0_f64; 2]; self.maxl + 1];
        let aa = (sec / 3600.0 + glon / 15.0 + 48.0).rem_euclid(24.0);
        let bb = aa * TWO_PI / 24.0;
        for (l, slot) in fl.iter_mut().enumerate() {
            let cc = l as f64 * bb;
            slot[0] = cc.cos();
            slot[1] = cc.sin();
        }

        let mut fm = vec![[0.0_f64; 2]; self.maxm + 1];
        let aa = glon * DEG_TO_RAD;
        for (m, slot) in fm.iter_mut().enumerate() {
            let bb = m as f64 * aa;
            slot[0] = bb.cos();
            slot[1] = bb.sin();
        }

        let theta = (90.0 - glat) * DEG_TO_RAD;
        let basis = alf.basis(self.maxn, self.maxm, theta);
        let (zwght, lev) = self.vertical_weights(alt);

        let mut u = 0.0_f64;
        let mut v = 0.0_f64;
        let mut bz = vec![0.0_f64; self.nbf + 8];
        for (b, weight) in zwght.iter().copied().enumerate().take(self.p + 1) {
            if weight == 0.0 {
                continue;
            }
            let d = b + lev;
            let order = self.order[d];
            let amaxs = order[0];
            let amaxn = order[1];
            let pmaxm = order[2];
            let pmaxs = order[3];
            let pmaxn = order[4];
            let tmaxl = order[5];
            let tmaxs = order[6];
            let tmaxn = order[7];

            let mut c = 1_usize;
            for n in 1..=amaxn {
                let sc = (n as f64 * theta).sin();
                bz[c] = -sc;
                bz[c + 1] = sc;
                c += 2;
            }
            for s in 1..=amaxs {
                let cs = fs[s][0];
                let ss = fs[s][1];
                for n in 1..=amaxn {
                    let sc = (n as f64 * theta).sin();
                    bz[c] = -sc * cs;
                    bz[c + 1] = sc * ss;
                    bz[c + 2] = sc * cs;
                    bz[c + 3] = -sc * ss;
                    c += 4;
                }
            }

            for m in 1..=pmaxm {
                let cm = fm[m][0];
                let sm = fm[m][1];
                for n in m..=pmaxn {
                    let vb = basis.v[n][m];
                    let wb = basis.w[n][m];
                    bz[c] = -vb * cm;
                    bz[c + 1] = vb * sm;
                    bz[c + 2] = -wb * sm;
                    bz[c + 3] = -wb * cm;
                    c += 4;
                }
                for s in 1..=pmaxs {
                    let cs = fs[s][0];
                    let ss = fs[s][1];
                    for n in m..=pmaxn {
                        let vb = basis.v[n][m];
                        let wb = basis.w[n][m];
                        bz[c] = -vb * cm * cs;
                        bz[c + 1] = vb * sm * cs;
                        bz[c + 2] = -wb * sm * cs;
                        bz[c + 3] = -wb * cm * cs;
                        bz[c + 4] = -vb * cm * ss;
                        bz[c + 5] = vb * sm * ss;
                        bz[c + 6] = -wb * sm * ss;
                        bz[c + 7] = -wb * cm * ss;
                        c += 8;
                    }
                }
            }

            for l in 1..=tmaxl {
                let cl = fl[l][0];
                let sl = fl[l][1];
                for n in l..=tmaxn {
                    let vb = basis.v[n][l];
                    let wb = basis.w[n][l];
                    bz[c] = -vb * cl;
                    bz[c + 1] = vb * sl;
                    bz[c + 2] = -wb * sl;
                    bz[c + 3] = -wb * cl;
                    c += 4;
                }
                for s in 1..=tmaxs {
                    let cs = fs[s][0];
                    let ss = fs[s][1];
                    for n in l..=tmaxn {
                        let vb = basis.v[n][l];
                        let wb = basis.w[n][l];
                        bz[c] = -vb * cl * cs;
                        bz[c + 1] = vb * sl * cs;
                        bz[c + 2] = -wb * sl * cs;
                        bz[c + 3] = -wb * cl * cs;
                        bz[c + 4] = -vb * cl * ss;
                        bz[c + 5] = vb * sl * ss;
                        bz[c + 6] = -wb * sl * ss;
                        bz[c + 7] = -wb * cl * ss;
                        c += 8;
                    }
                }
            }

            c -= 1;
            let count = self.nb[d].min(c);
            u += weight * dot_product_f64(&bz[1..=count], &self.mparm[d][1..=count]);
            v += weight * dot_product_f64(&bz[1..=count], &self.tparm[d][1..=count]);
        }

        Hwm14Output {
            meridional_m_s: v as f32,
            zonal_m_s: u as f32,
        }
    }

    fn vertical_weights(&self, altitude_km: f64) -> ([f64; 4], usize) {
        let mut iz = find_span(self.nnode - self.p - 1, self.p, altitude_km, &self.vnode) - self.p;
        iz = iz.min(26);

        let mut weights = [0.0_f64; 4];
        weights[0] = bspline(self.p, self.nnode, &self.vnode, iz, altitude_km);
        weights[1] = bspline(self.p, self.nnode, &self.vnode, iz + 1, altitude_km);
        if iz <= 25 {
            weights[2] = bspline(self.p, self.nnode, &self.vnode, iz + 2, altitude_km);
            weights[3] = bspline(self.p, self.nnode, &self.vnode, iz + 3, altitude_km);
            return (weights, iz);
        }

        let extension = if altitude_km > self.alttns {
            [
                0.0,
                0.0,
                0.0,
                (-(altitude_km - self.alttns) / HWM14_HIGH_ALTITUDE_SCALE_KM).exp(),
                1.0,
            ]
        } else {
            [
                bspline(self.p, self.nnode, &self.vnode, iz + 2, altitude_km),
                bspline(self.p, self.nnode, &self.vnode, iz + 3, altitude_km),
                bspline(self.p, self.nnode, &self.vnode, iz + 4, altitude_km),
                0.0,
                0.0,
            ]
        };
        weights[2] = dot_product_f64(&extension, &self.e1);
        weights[3] = dot_product_f64(&extension, &self.e2);
        (weights, iz)
    }
}

#[allow(clippy::many_single_char_names)]
fn apply_parity(order: [usize; 9], nb: usize, mparm: &mut [f64], tparm: &mut [f64]) {
    let amaxs = order[0];
    let amaxn = order[1];
    let pmaxm = order[2];
    let pmaxs = order[3];
    let pmaxn = order[4];
    let tmaxl = order[5];
    let tmaxs = order[6];
    let tmaxn = order[7];

    let mut c = 1_usize;
    for _n in 1..=amaxn {
        tparm[c] = 0.0;
        tparm[c + 1] = -mparm[c + 1];
        mparm[c + 1] = 0.0;
        c += 2;
    }
    for _s in 1..=amaxs {
        for _n in 1..=amaxn {
            tparm[c] = 0.0;
            tparm[c + 1] = 0.0;
            tparm[c + 2] = -mparm[c + 2];
            tparm[c + 3] = -mparm[c + 3];
            mparm[c + 2] = 0.0;
            mparm[c + 3] = 0.0;
            c += 4;
        }
    }

    for m in 1..=pmaxm {
        for _n in m..=pmaxn {
            tparm[c] = mparm[c + 2];
            tparm[c + 1] = mparm[c + 3];
            tparm[c + 2] = -mparm[c];
            tparm[c + 3] = -mparm[c + 1];
            c += 4;
        }
        for _s in 1..=pmaxs {
            for _n in m..=pmaxn {
                tparm[c] = mparm[c + 2];
                tparm[c + 1] = mparm[c + 3];
                tparm[c + 2] = -mparm[c];
                tparm[c + 3] = -mparm[c + 1];
                tparm[c + 4] = mparm[c + 6];
                tparm[c + 5] = mparm[c + 7];
                tparm[c + 6] = -mparm[c + 4];
                tparm[c + 7] = -mparm[c + 5];
                c += 8;
            }
        }
    }

    for l in 1..=tmaxl {
        for _n in l..=tmaxn {
            tparm[c] = mparm[c + 2];
            tparm[c + 1] = mparm[c + 3];
            tparm[c + 2] = -mparm[c];
            tparm[c + 3] = -mparm[c + 1];
            c += 4;
        }
        for _s in 1..=tmaxs {
            for _n in l..=tmaxn {
                tparm[c] = mparm[c + 2];
                tparm[c + 1] = mparm[c + 3];
                tparm[c + 2] = -mparm[c];
                tparm[c + 3] = -mparm[c + 1];
                tparm[c + 4] = mparm[c + 6];
                tparm[c + 5] = mparm[c + 7];
                tparm[c + 6] = -mparm[c + 4];
                tparm[c + 7] = -mparm[c + 5];
                c += 8;
            }
        }
    }

    debug_assert_eq!(c - 1, nb);
}

#[derive(Debug)]
struct DisturbanceModel {
    nterm: usize,
    nmax: usize,
    mmax: usize,
    termarr: Vec<[i32; 3]>,
    coeff: Vec<f32>,
    twidth: f32,
    nvshterm: usize,
}

impl DisturbanceModel {
    fn parse(bytes: &[u8]) -> Result<Self, &'static str> {
        let mut reader = FortranRecordReader::new(bytes);
        let header = reader.read_record()?;
        let mut header = BinaryReader::new(header);
        let nterm = header.read_i32()? as usize;
        let mmax = header.read_i32()? as usize;
        let nmax = header.read_i32()? as usize;
        header.finish()?;

        let terms = reader.read_record()?;
        let mut terms = BinaryReader::new(terms);
        let mut termarr = Vec::with_capacity(nterm);
        for _ in 0..nterm {
            termarr.push([terms.read_i32()?, terms.read_i32()?, terms.read_i32()?]);
        }
        terms.finish()?;

        let coeffs = reader.read_record()?;
        let mut coeffs = BinaryReader::new(coeffs);
        let mut coeff = Vec::with_capacity(nterm);
        for _ in 0..nterm {
            coeff.push(coeffs.read_f32()?);
        }
        coeffs.finish()?;

        let twidth_record = reader.read_record()?;
        let mut twidth_reader = BinaryReader::new(twidth_record);
        let twidth = twidth_reader.read_f32()?;
        twidth_reader.finish()?;
        reader.finish()?;

        let nvshterm = ((((nmax + 1) * (nmax + 2) - (nmax - mmax) * (nmax - mmax + 1)) / 2 - 1)
            * 4)
            - 2 * nmax;
        Ok(Self {
            nterm,
            nmax,
            mmax,
            termarr,
            coeff,
            twidth,
            nvshterm,
        })
    }

    fn evaluate(
        &self,
        alf: &AlfCoefficients,
        geomagnetic: &GeomagneticModel,
        inputs: ScalarInputs,
    ) -> Hwm14Output {
        let kp = ap_to_kp(inputs.ap_current_3h);
        let (mlat, mlon, f1e, f1n, f2e, f2n) =
            geomagnetic.geo_to_quasi_dipole(alf, inputs.latitude_deg, inputs.longitude_deg);
        let day = inputs.day_of_year as f32;
        let ut = inputs.utc_seconds / 3600.0;
        let mlt = geomagnetic.magnetic_local_time(alf, mlon, day, ut);
        let magnetic = self.evaluate_magnetic(alf, mlt, mlat, kp);

        let mut meridional = f2n * magnetic.meridional_m_s + f1n * magnetic.zonal_m_s;
        let mut zonal = f2e * magnetic.meridional_m_s + f1e * magnetic.zonal_m_s;
        let height_weight = 1.0 / (1.0 + (-(inputs.altitude_km - 125.0) / self.twidth).exp());
        meridional *= height_weight;
        zonal *= height_weight;
        Hwm14Output {
            meridional_m_s: meridional,
            zonal_m_s: zonal,
        }
    }

    #[allow(clippy::many_single_char_names)]
    fn evaluate_magnetic(
        &self,
        alf: &AlfCoefficients,
        mlt: f32,
        mlat: f32,
        kp: f32,
    ) -> Hwm14Output {
        let theta = (90.0_f64 - f64::from(mlat)) * DEG_TO_RAD;
        let basis = alf.basis(self.nmax, self.mmax, theta);

        let mut mltterms = vec![[0.0_f64; 2]; self.mmax + 1];
        let phi = f64::from(mlt) * DEG_TO_RAD * 15.0;
        for (m, slot) in mltterms.iter_mut().enumerate() {
            let mphi = m as f64 * phi;
            slot[0] = mphi.cos();
            slot[1] = mphi.sin();
        }

        let mut vshterms = vec![[0.0_f32; 2]; self.nvshterm];
        let mut ivshterm = 0_usize;
        for n in 1..=self.nmax {
            vshterms[ivshterm][0] = (-(basis.v[n][0] * mltterms[0][0])) as f32;
            vshterms[ivshterm + 1][0] = (basis.w[n][0] * mltterms[0][0]) as f32;
            vshterms[ivshterm][1] = -vshterms[ivshterm + 1][0];
            vshterms[ivshterm + 1][1] = vshterms[ivshterm][0];
            ivshterm += 2;
            for m in 1..=self.mmax {
                if m > n {
                    continue;
                }
                vshterms[ivshterm][0] = (-(basis.v[n][m] * mltterms[m][0])) as f32;
                vshterms[ivshterm + 1][0] = (basis.v[n][m] * mltterms[m][1]) as f32;
                vshterms[ivshterm + 2][0] = (basis.w[n][m] * mltterms[m][1]) as f32;
                vshterms[ivshterm + 3][0] = (basis.w[n][m] * mltterms[m][0]) as f32;
                vshterms[ivshterm][1] = -vshterms[ivshterm + 2][0];
                vshterms[ivshterm + 1][1] = -vshterms[ivshterm + 3][0];
                vshterms[ivshterm + 2][1] = vshterms[ivshterm][0];
                vshterms[ivshterm + 3][1] = vshterms[ivshterm + 1][0];
                ivshterm += 4;
            }
        }

        let kpterms = kp_spline3(kp);
        let lat_weight = latitude_weight(mlat, mlt, kp, self.twidth);
        let mut meridional = 0.0_f32;
        let mut zonal = 0.0_f32;
        for iterm in 0..self.nterm {
            let mut term = [1.0_f32, 1.0_f32];
            let indices = self.termarr[iterm];
            if indices[0] != 999 {
                let index = indices[0] as usize;
                term[0] *= vshterms[index][0];
                term[1] *= vshterms[index][1];
            }
            if indices[1] != 999 {
                let index = indices[1] as usize;
                term[0] *= kpterms[index];
                term[1] *= kpterms[index];
            }
            if indices[2] != 999 {
                term[0] *= lat_weight;
                term[1] *= lat_weight;
            }
            meridional += self.coeff[iterm] * term[0];
            zonal += self.coeff[iterm] * term[1];
        }
        Hwm14Output {
            meridional_m_s: meridional,
            zonal_m_s: zonal,
        }
    }
}

fn ap_to_kp(ap0: f32) -> f32 {
    const AP_GRID: [f32; 28] = [
        0.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 9.0, 12.0, 15.0, 18.0, 22.0, 27.0, 32.0, 39.0, 48.0,
        56.0, 67.0, 80.0, 94.0, 111.0, 132.0, 154.0, 179.0, 207.0, 236.0, 300.0, 400.0,
    ];
    let ap = ap0.clamp(0.0, 400.0);
    let mut i = 1_usize;
    while ap > AP_GRID[i] {
        i += 1;
    }
    if ap == AP_GRID[i] {
        i as f32 / 3.0
    } else {
        (i as f32 - 1.0) / 3.0 + (ap - AP_GRID[i - 1]) / (3.0 * (AP_GRID[i] - AP_GRID[i - 1]))
    }
}

fn kp_spline3(kp: f32) -> [f32; 3] {
    const NODE: [f32; 8] = [-10.0, -8.0, 0.0, 2.0, 5.0, 8.0, 18.0, 20.0];
    let x = kp.clamp(0.0, 8.0);
    let mut kpspl = [0.0_f32; 7];
    for i in 0..=6 {
        if x >= NODE[i] && x < NODE[i + 1] {
            kpspl[i] = 1.0;
        }
    }
    for j in 2..=3 {
        for i in 0..=(8 - j - 1) {
            kpspl[i] = kpspl[i] * (x - NODE[i]) / (NODE[i + j - 1] - NODE[i])
                + kpspl[i + 1] * (NODE[i + j] - x) / (NODE[i + j] - NODE[i + 1]);
        }
    }
    [kpspl[0] + kpspl[1], kpspl[2], kpspl[3] + kpspl[4]]
}

fn latitude_weight(mlat: f32, mlt: f32, kp0: f32, twidth: f32) -> f32 {
    const COEFF: [f32; 6] = [65.7633, -4.60256, -3.53915, -1.99971, -0.752_193, 0.972_388];
    let mlt_rad = mlt * 15.0 * std::f32::consts::PI / 180.0;
    let sin_mlt = mlt_rad.sin();
    let cos_mlt = mlt_rad.cos();
    let kp = kp0.clamp(0.0, 8.0);
    let transition_lat = COEFF[0]
        + COEFF[1] * cos_mlt
        + COEFF[2] * sin_mlt
        + kp * (COEFF[3] + COEFF[4] * cos_mlt + COEFF[5] * sin_mlt);
    1.0 / (1.0 + (-(mlat.abs() - transition_lat) / twidth).exp())
}

#[derive(Debug)]
struct GeomagneticModel {
    nmax: usize,
    mmax: usize,
    nterm: usize,
    xcoeff: Vec<f64>,
    ycoeff: Vec<f64>,
    zcoeff: Vec<f64>,
    normadj: Vec<f64>,
}

impl GeomagneticModel {
    fn parse(bytes: &[u8]) -> Result<Self, &'static str> {
        let mut reader = FortranRecordReader::new(bytes);
        let header = reader.read_record()?;
        let mut header = BinaryReader::new(header);
        let nmax = header.read_i32()? as usize;
        let mmax = header.read_i32()? as usize;
        let nterm = header.read_i32()? as usize;
        let _epoch = header.read_f32()?;
        let _alt = header.read_f32()?;
        header.finish()?;

        let coeffs = reader.read_record()?;
        let mut coeffs = BinaryReader::new(coeffs);
        let mut xcoeff = Vec::with_capacity(nterm);
        let mut ycoeff = Vec::with_capacity(nterm);
        let mut zcoeff = Vec::with_capacity(nterm);
        for _ in 0..nterm {
            xcoeff.push(coeffs.read_f64()?);
        }
        for _ in 0..nterm {
            ycoeff.push(coeffs.read_f64()?);
        }
        for _ in 0..nterm {
            zcoeff.push(coeffs.read_f64()?);
        }
        coeffs.finish()?;
        reader.finish()?;

        let mut normadj = vec![0.0; nmax + 1];
        for (n, value) in normadj.iter_mut().enumerate() {
            *value = (n as f64 * (n + 1) as f64).sqrt();
        }
        Ok(Self {
            nmax,
            mmax,
            nterm,
            xcoeff,
            ycoeff,
            zcoeff,
            normadj,
        })
    }

    #[allow(clippy::many_single_char_names)]
    fn geo_to_quasi_dipole(
        &self,
        alf: &AlfCoefficients,
        latitude_deg: f32,
        longitude_deg: f32,
    ) -> (f32, f32, f32, f32, f32, f32) {
        let theta = (90.0 - f64::from(latitude_deg)) * DEG_TO_RAD;
        let basis = alf.basis(self.nmax, self.mmax, theta);
        let phi = f64::from(longitude_deg) * DEG_TO_RAD;
        let mut sh = vec![0.0_f64; self.nterm];
        let mut shgradtheta = vec![0.0_f64; self.nterm];
        let mut shgradphi = vec![0.0_f64; self.nterm];

        let mut i = 0_usize;
        for n in 0..=self.nmax {
            sh[i] = basis.p[n][0];
            shgradtheta[i] = basis.v[n][0] * self.normadj[n];
            shgradphi[i] = 0.0;
            i += 1;
        }
        for m in 1..=self.mmax {
            let mphi = m as f64 * phi;
            let cos_mphi = mphi.cos();
            let sin_mphi = mphi.sin();
            for n in m..=self.nmax {
                sh[i] = basis.p[n][m] * cos_mphi;
                sh[i + 1] = basis.p[n][m] * sin_mphi;
                shgradtheta[i] = basis.v[n][m] * self.normadj[n] * cos_mphi;
                shgradtheta[i + 1] = basis.v[n][m] * self.normadj[n] * sin_mphi;
                shgradphi[i] = -basis.w[n][m] * self.normadj[n] * sin_mphi;
                shgradphi[i + 1] = basis.w[n][m] * self.normadj[n] * cos_mphi;
                i += 2;
            }
        }

        let x = dot_product_f64(&sh, &self.xcoeff);
        let y = dot_product_f64(&sh, &self.ycoeff);
        let z = dot_product_f64(&sh, &self.zcoeff);
        let qlon_rad = y.atan2(x);
        let cos_qlon = qlon_rad.cos();
        let sin_qlon = qlon_rad.sin();
        let cos_qlat = x * cos_qlon + y * sin_qlon;
        let qlat = (z.atan2(cos_qlat) / DEG_TO_RAD) as f32;
        let qlon = (qlon_rad / DEG_TO_RAD) as f32;

        let xgradtheta = dot_product_f64(&shgradtheta, &self.xcoeff);
        let ygradtheta = dot_product_f64(&shgradtheta, &self.ycoeff);
        let zgradtheta = dot_product_f64(&shgradtheta, &self.zcoeff);
        let xgradphi = dot_product_f64(&shgradphi, &self.xcoeff);
        let ygradphi = dot_product_f64(&shgradphi, &self.ycoeff);
        let zgradphi = dot_product_f64(&shgradphi, &self.zcoeff);

        let f1e =
            (-zgradtheta * cos_qlat + (xgradtheta * cos_qlon + ygradtheta * sin_qlon) * z) as f32;
        let f1n = (-zgradphi * cos_qlat + (xgradphi * cos_qlon + ygradphi * sin_qlon) * z) as f32;
        let f2e = (ygradtheta * cos_qlon - xgradtheta * sin_qlon) as f32;
        let f2n = (ygradphi * cos_qlon - xgradphi * sin_qlon) as f32;
        (qlat, qlon, f1e, f1n, f2e, f2n)
    }

    fn magnetic_local_time(&self, alf: &AlfCoefficients, qlon: f32, day: f32, ut: f32) -> f32 {
        const SIN_EPS: f64 = 0.397_818_68;
        let anti_sun_lat =
            -(((f64::from(day) + f64::from(ut) / 24.0 - 80.0) * DEG_TO_RAD).sin() * SIN_EPS).asin()
                / DEG_TO_RAD;
        let anti_sun_lon = -f64::from(ut) * 15.0;
        let theta = (90.0 - anti_sun_lat) * DEG_TO_RAD;
        let basis = alf.basis(self.nmax, self.mmax, theta);
        let phi = anti_sun_lon * DEG_TO_RAD;
        let mut sh = vec![0.0_f64; self.nterm];
        let mut i = 0_usize;
        for n in 0..=self.nmax {
            sh[i] = basis.p[n][0];
            i += 1;
        }
        for m in 1..=self.mmax {
            let mphi = m as f64 * phi;
            let cos_mphi = mphi.cos();
            let sin_mphi = mphi.sin();
            for n in m..=self.nmax {
                sh[i] = basis.p[n][m] * cos_mphi;
                sh[i + 1] = basis.p[n][m] * sin_mphi;
                i += 2;
            }
        }
        let x = dot_product_f64(&sh, &self.xcoeff);
        let y = dot_product_f64(&sh, &self.ycoeff);
        let anti_sun_qlon = (y.atan2(x) / DEG_TO_RAD) as f32;
        (qlon - anti_sun_qlon) / 15.0
    }
}

#[derive(Debug)]
struct AlfCoefficients {
    anm: Matrix,
    bnm: Matrix,
    dnm: Matrix,
    cm: Vec<f64>,
    en: Vec<f64>,
    marr: Vec<f64>,
    narr: Vec<f64>,
}

impl AlfCoefficients {
    fn new(nmax: usize, mmax: usize) -> Self {
        let mut anm = matrix(nmax + 1, mmax + 1);
        let mut bnm = matrix(nmax + 1, mmax + 1);
        let mut dnm = matrix(nmax + 1, mmax + 1);
        let mut cm = vec![0.0; mmax + 1];
        let mut en = vec![0.0; nmax + 1];
        let mut marr = vec![0.0; mmax + 1];
        let mut narr = vec![0.0; nmax + 1];

        for n in 1..=nmax {
            let nf = n as f64;
            narr[n] = nf;
            en[n] = (nf * (nf + 1.0)).sqrt();
            anm[n][0] = (((2 * n - 1) * (2 * n + 1)) as f64).sqrt() / narr[n];
            bnm[n][0] = (((2 * n + 1) as f64 * (n - 1) as f64 * (n - 1) as f64) / (2.0 * nf - 3.0))
                .sqrt()
                / narr[n];
        }
        for m in 1..=mmax {
            let mf = m as f64;
            marr[m] = mf;
            cm[m] = ((2.0 * mf + 1.0) / (2.0 * mf * mf * (mf + 1.0))).sqrt();
            for n in (m + 1)..=nmax {
                anm[n][m] = (((2 * n - 1) * (2 * n + 1) * (n - 1)) as f64
                    / ((n - m) * (n + m) * (n + 1)) as f64)
                    .sqrt();
                bnm[n][m] = (((2 * n + 1) * (n + m - 1) * (n - m - 1) * (n - 2) * (n - 1)) as f64
                    / ((n - m) * (n + m) * (2 * n - 3) * n * (n + 1)) as f64)
                    .sqrt();
                dnm[n][m] = (((n - m) * (n + m) * (2 * n + 1) * (n - 1)) as f64
                    / ((2 * n - 1) * (n + 1)) as f64)
                    .sqrt();
            }
        }
        Self {
            anm,
            bnm,
            dnm,
            cm,
            en,
            marr,
            narr,
        }
    }

    fn basis(&self, nmax: usize, mmax: usize, theta: f64) -> AlfBasis {
        let mut p = matrix(nmax + 1, mmax + 1);
        let mut v = matrix(nmax + 1, mmax + 1);
        let mut w = matrix(nmax + 1, mmax + 1);
        p[0][0] = std::f64::consts::FRAC_1_SQRT_2;
        let x = theta.cos();
        let y = theta.sin();
        for m in 1..=mmax {
            w[m][m] = self.cm[m] * p[m - 1][m - 1];
            p[m][m] = y * self.en[m] * w[m][m];
            for n in (m + 1)..=nmax {
                w[n][m] = self.anm[n][m] * x * w[n - 1][m] - self.bnm[n][m] * w[n - 2][m];
                p[n][m] = y * self.en[n] * w[n][m];
                v[n][m] = self.narr[n] * x * w[n][m] - self.dnm[n][m] * w[n - 1][m];
                w[n - 2][m] *= self.marr[m];
            }
            w[nmax - 1][m] *= self.marr[m];
            w[nmax][m] *= self.marr[m];
            v[m][m] = x * w[m][m];
        }
        p[1][0] = self.anm[1][0] * x * p[0][0];
        v[1][0] = -p[1][1];
        for n in 2..=nmax {
            p[n][0] = self.anm[n][0] * x * p[n - 1][0] - self.bnm[n][0] * p[n - 2][0];
            v[n][0] = -p[n][1];
        }
        AlfBasis { p, v, w }
    }
}

#[derive(Debug)]
struct AlfBasis {
    p: Matrix,
    v: Matrix,
    w: Matrix,
}

type Matrix = Vec<Vec<f64>>;

fn matrix(rows: usize, cols: usize) -> Matrix {
    vec![vec![0.0; cols]; rows]
}

fn bspline(p: usize, m: usize, knots: &[f64], i: usize, u: f64) -> f64 {
    if i == 0 && u == knots[0] {
        return 1.0;
    }
    if i == m - p - 1 && u == knots[m] {
        return 1.0;
    }
    if u < knots[i] || u >= knots[i + p + 1] {
        return 0.0;
    }
    let mut n = vec![0.0; p + 2];
    for j in 0..=p {
        n[j] = if u >= knots[i + j] && u < knots[i + j + 1] {
            1.0
        } else {
            0.0
        };
    }
    for k in 1..=p {
        let mut saved = if n[0] == 0.0 {
            0.0
        } else {
            ((u - knots[i]) * n[0]) / (knots[i + k] - knots[i])
        };
        for j in 0..=(p - k) {
            let left = knots[i + j + 1];
            let right = knots[i + j + k + 1];
            if n[j + 1] == 0.0 {
                n[j] = saved;
                saved = 0.0;
            } else {
                let temp = n[j + 1] / (right - left);
                n[j] = saved + (right - u) * temp;
                saved = (u - left) * temp;
            }
        }
    }
    n[0]
}

fn find_span(n: usize, p: usize, u: f64, knots: &[f64]) -> usize {
    if u >= knots[n + 1] {
        return n;
    }
    let mut low = p;
    let mut high = n + 1;
    let mut mid = (low + high) / 2;
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

fn dot_product_f64(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .fold(0.0, |acc, (left, right)| acc + left * right)
}

struct BinaryReader<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> BinaryReader<'a> {
    const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn read_i32(&mut self) -> Result<i32, &'static str> {
        Ok(i32::from_le_bytes(self.take()?))
    }

    fn read_f32(&mut self) -> Result<f32, &'static str> {
        Ok(f32::from_le_bytes(self.take()?))
    }

    fn read_f64(&mut self) -> Result<f64, &'static str> {
        Ok(f64::from_le_bytes(self.take()?))
    }

    fn read_f64_array<const N: usize>(&mut self) -> Result<[f64; N], &'static str> {
        let mut output = [0.0; N];
        for value in &mut output {
            *value = self.read_f64()?;
        }
        Ok(output)
    }

    fn finish(&self) -> Result<(), &'static str> {
        if self.offset == self.bytes.len() {
            Ok(())
        } else {
            Err("HWM14 binary parser did not consume all bytes")
        }
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N], &'static str> {
        let end = self
            .offset
            .checked_add(N)
            .ok_or("HWM14 binary parser offset overflow")?;
        if end > self.bytes.len() {
            return Err("HWM14 binary parser reached end of data");
        }
        let mut out = [0_u8; N];
        out.copy_from_slice(&self.bytes[self.offset..end]);
        self.offset = end;
        Ok(out)
    }
}

struct FortranRecordReader<'a> {
    reader: BinaryReader<'a>,
}

impl<'a> FortranRecordReader<'a> {
    const fn new(bytes: &'a [u8]) -> Self {
        Self {
            reader: BinaryReader::new(bytes),
        }
    }

    fn read_record(&mut self) -> Result<&'a [u8], &'static str> {
        let len = self.reader.read_i32()?;
        if len < 0 {
            return Err("negative HWM14 Fortran record length");
        }
        let start = self.reader.offset;
        let end = start
            .checked_add(len as usize)
            .ok_or("HWM14 Fortran record length overflow")?;
        if end > self.reader.bytes.len() {
            return Err("HWM14 Fortran record extends past end of data");
        }
        self.reader.offset = end;
        let trailer = self.reader.read_i32()?;
        if trailer != len {
            return Err("HWM14 Fortran record marker mismatch");
        }
        Ok(&self.reader.bytes[start..end])
    }

    fn finish(&self) -> Result<(), &'static str> {
        self.reader.finish()
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    fn frame() -> FrameContext {
        FrameContext::toy_fixed_earth()
    }

    fn pos(z: f64) -> Position3<Eci> {
        Position3::new(0.0, 0.0, z)
    }

    #[test]
    fn reference_profile_pins_public_height_profile_rows() {
        let rows = Hwm14Wind::reference_profile();
        assert_eq!(rows.len(), 17);
        assert_eq!(rows[0], Hwm14ReferenceRow::new(0.0, 0.031, 6.271));
        assert_eq!(rows[8], Hwm14ReferenceRow::new(200_000.0, 25.027, -68.588));
        assert_eq!(
            rows[16],
            Hwm14ReferenceRow::new(400_000.0, 51.259, -100.946)
        );
    }

    #[test]
    fn full_path_matches_public_height_profile_rows() {
        let wind = Hwm14Wind::reference_conditions();
        for row in Hwm14Wind::reference_profile() {
            let sample = wind.evaluate_altitude_m(row.altitude_m).unwrap();
            assert_abs_diff_eq!(sample.vector.x, row.meridional_m_s, epsilon = 1.0e-3);
            assert_abs_diff_eq!(sample.vector.y, row.zonal_m_s, epsilon = 1.0e-3);
            assert_eq!(sample.vector.z.to_bits(), 0.0_f64.to_bits());
        }
    }

    #[test]
    fn quiet_path_matches_public_height_profile_spots() {
        let wind = Hwm14Wind::new(Hwm14Inputs::quiet_reference_conditions()).unwrap();
        let sample_100 = wind.evaluate_altitude_m(100_000.0).unwrap();
        assert_abs_diff_eq!(sample_100.vector.x, -14.339, epsilon = 1.0e-3);
        assert_abs_diff_eq!(sample_100.vector.y, 31.627, epsilon = 1.0e-3);
        let sample_400 = wind.evaluate_altitude_m(400_000.0).unwrap();
        assert_abs_diff_eq!(sample_400.vector.x, 6.702, epsilon = 1.0e-3);
        assert_abs_diff_eq!(sample_400.vector.y, -81.981, epsilon = 1.0e-3);
    }

    #[test]
    fn disturbance_path_responds_to_ap() {
        let quiet = Hwm14Wind::new(Hwm14Inputs::quiet_reference_conditions())
            .unwrap()
            .evaluate_altitude_m(400_000.0)
            .unwrap();
        let active = Hwm14Wind::reference_conditions()
            .evaluate_altitude_m(400_000.0)
            .unwrap();
        assert!(active.vector.x > quiet.vector.x);
        assert!(active.vector.y < quiet.vector.y);
    }

    #[test]
    fn wind_model_uses_position_z_as_altitude() {
        let wind = Hwm14Wind::reference_conditions()
            .wind_ned_m_s(pos(400_000.0), &frame(), SimTime::ZERO)
            .unwrap();
        assert_abs_diff_eq!(wind.vector.x, 51.259, epsilon = 1.0e-3);
        assert_abs_diff_eq!(wind.vector.y, -100.946, epsilon = 1.0e-3);
        assert_eq!(wind.vector.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn accepts_high_altitude_extension() {
        let wind = Hwm14Wind::reference_conditions();
        let sample = wind.evaluate_altitude_m(450_000.0).unwrap();
        assert!(sample.vector.x.is_finite());
        assert!(sample.vector.y.is_finite());
    }

    #[test]
    fn rejects_non_finite_altitude() {
        let wind = Hwm14Wind::reference_conditions();
        assert!(matches!(
            wind.evaluate_altitude_m(f64::NAN),
            Err(PhysicsError::NonFinite { .. })
        ));
    }

    #[test]
    fn validates_inputs() {
        let mut inputs = Hwm14Inputs::reference_conditions();
        inputs.day_of_year = 367;
        assert!(Hwm14Wind::new(inputs).is_err());
        let mut inputs = Hwm14Inputs::reference_conditions();
        inputs.ap_current_3h = 401.0;
        assert!(Hwm14Wind::new(inputs).is_err());
    }
}
