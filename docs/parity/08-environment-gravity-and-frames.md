# Gravity, Frames, Atmosphere & Space Environment

**Status:** `experimental` (design intent; this document ships no code).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**One-line summary:** Bring OpenBMP's environment models to capability parity
with GMAT/Orekit/Earth-GRAM-class practice — full tesseral spherical-harmonic
gravity (Pines, with a normalized-Gottlieb oracle), third-body + cannonball SRP +
Schwarzschild perturbations, IAU 2006/2000A CIO frames with IERS EOP, solid/ocean
tides and IGRF-14, and a GRAM-style perturbed atmosphere for Monte Carlo
dispersion — staying deterministic, byte-stable-by-default, and strictly forward.

> Read `00-overview.md` (parity definition, invariants, DAG) and
> `13-agent-execution-playbook.md` (doc template, backlog schema, gate set)
> before this document. The fidelity tiers, work-package schema, and the
> non-negotiable invariants are defined there.

---

## 1. Parity target & ceiling

**Target (capability / method / architecture parity).** OpenBMP's environment
layer matches the *formulations and architecture* of the reference astrodynamics
and atmosphere tools used at SpaceX/Rocket Lab/NASA/JPL:

- **Gravity:** full tesseral + sectoral spherical-harmonic geopotential via the
  singularity-free **Pines (1973)** kernel with the **Holmes–Featherstone**
  scaled normalized recursion, selectable degree/order, coefficients ingested
  from **EGM2008** (and GGM05/GOCO variants), cross-checked against an
  independent **normalized Gottlieb** synthesis to f64 round-off.
- **Perturbations:** third-body point-mass (Sun/Moon/planets) via SPK ephemerides
  with **Battin's cancellation-free `f(q)`** formulation; **cannonball SRP** with
  a conical umbra/penumbra shadow; **Schwarzschild** relativistic term (optional
  Lense–Thirring / de Sitter); **solid + ocean + pole tides** per **IERS
  Conventions (2010)** as time-varying geopotential coefficients.
- **Frames:** the **IAU 2006/2000A CIO-based** GCRS↔ITRS transform (X, Y, the
  CIO locator *s*, the Earth Rotation Angle, polar motion with the TIO locator
  *s′*) driven by **IERS EOP 14 C04 / finals2000A**, replacing the current
  equinox-based IAU 1976/1980 chain.
- **Geomagnetic:** **IGRF-14** (degree 13, with secular variation) alongside the
  existing WMM, sharing the gravity ALF machinery, with a field gradient for
  sensor/torque fidelity.
- **Atmosphere dispersion:** a **GRAM-style PerturbedAtmosphere** decorator over
  the existing NRLMSISE-00 / HWM14 mean models — altitude-dependent σ envelopes
  with **Ornstein–Uhlenbeck** correlation along the trajectory plus a small-scale
  **Dryden** spectrum — producing reproducible dispersed density/wind realizations
  for loads/footprint/abort statistics.

This dimension is **pure simulation with no proprietary-data dependency** (per
`00-overview.md` §1.1), so it can reach high in-repo fidelity directly: the
methods are public (IERS Conventions, ERFA, EGM2008, IGRF, the GRAM technical
memos) and validatable code-to-code against open oracles (ERFA, Orekit, GMAT,
GeographicLib, NGA HARMONIC_SYNTH, NCEI test vectors).

**Parity ceiling (the honest boundary — do not overstate).** Four things cannot
be matched in an open repo, with the open substitute named for each:

1. **Operational/predictive EOP and space-weather** of the quality used at run
   time depend on rapid-service products and proprietary forecasting. *Open
   substitute:* IERS `finals2000A` + CelesTrak `SW-All.csv` with the standard
   prediction tables, carrying the published rapid-service error bars; documented
   as a reference INPUT, never a flight/targeting input.
2. **The full empirical Earth-GRAM perturbation covariance** — the σ envelopes,
   correlation lengths, and cross-correlations are GRAM's empirical core and are
   *not* published as turnkey coefficient tables. *Open substitute:* an
   explicitly-documented engineering envelope (altitude-dependent σ + OU
   correlation + Dryden small-scale per the GRAM technical-memo *methodology*),
   validated statistically (ensemble mean → NRLMSISE-00 mean; variance bracketed
   by observed storm/quiet spread) and **labeled a method-replica, not a GRAM
   port**.
3. **Vehicle-specific SRP macro-models / thermal re-radiation / antenna thrust**
   need proprietary CAD, surface optical properties, and on-orbit estimated
   empirical accelerations. *Open substitute:* cannonball + published box-wing
   parameters with an estimated reflectivity coefficient; the N-facet macro-model
   only when the user supplies geometry.
4. **Sub-cm precise-orbit-determination parity** (the LAGEOS/GRACE regime) needs
   the full IERS stack *plus* estimated empirical accelerations and station data
   pipelines that are a research program unto themselves. *Open substitute:*
   code-to-code agreement with Orekit/GMAT and ILRS published precise orbits as a
   truth proxy, reporting residuals honestly.

**Net:** OpenBMP can reach *IERS-conventional, code-to-code-validated* fidelity —
the correct open-science ceiling. It cannot claim certified mission-planning
accuracy without the proprietary auxiliary data and flight-estimated parameters
above. No artifact in this dimension ever claims `flight-qualified` / `certified`
/ `operational` (`00-overview.md` §3 invariant 8).

---

## 2. Current state in source

Verified by reading the actual files (paths absolute under the repo root).

### 2.1 Gravity — `crates/openbmp-physics/src/gravity.rs` (6833 lines)

- `trait GravityModel { fn gravity_eci_m_s2(&self, position_eci, time) -> Result<Vector3<f64>, PhysicsError> }` — the single environment-side acceleration surface used by current gravity/perturbation models; ephemeris-backed paths vary with time.
- `ConstantGravity`, `PointMassGravity` (`−µ/r² r̂`), `J2Gravity` (point-mass + J₂ in closed Cartesian form, Vallado §8.6).
- `Egm2008ZonalGravity` — **zonal-only** truncation J₂…J₆, **hard-capped at `EGM2008_MAX_DEGREE = 6`** (fixed-size arrays; `new(...)` rejects `degree ∉ [2,6]`). It can now build the same zonal truncation from `NormalizedHarmonicField` zonal `Cbar_n0` entries via `J_n = -Cbar_n0 sqrt(2n + 1)`, preserving default-zero missing zonals and fail-closed field-envelope validation.
- `TesseralGravity` / `FiniteDifferencePinesGravity` — first WP-08.1 substrate: a static degree-2/order-2 ECI
  harmonic surface with `DegreeTwoTesseralCoefficients`,
  `NormalizedDegreeTwoTesseralCoefficients`, `NormalizedHarmonicCoefficient`,
  `NormalizedHarmonicField`, `NormalizedHarmonicFieldIter`,
  `IcgemGfcNormalizedField`,
  `HarmonicLongitudeTrigonometry`, `HarmonicTruncation`,
  `HarmonicSynthesisPlan`, `HarmonicSynthesisTier`, `PinesLegendreTable`,
  `PinesLongitudePolynomials`, `PinesSynthesisPoint`, `PinesPotentialSum`,
  `GottliebPotentialSum`, `TideSystem`, and fail-closed degree/order
  validation.
  It supports `C20`, `C21/S21`, and `C22/S22` Cartesian solid-harmonic
  acceleration, converts fully-normalized degree-2 coefficient blocks into the
  current unnormalized evaluator, stores reusable normalized `Cbar/Sbar` fields
  in deterministic packed `(n, m)` order, exposes explicit
  `without_central_term` stripping for full coefficient files whose `Cbar00`
  central term must not be double-counted by point-mass-plus-correction
  evaluators, exposes a checked
  `fully_normalized_to_unnormalized_scale` helper, a std-gated fail-closed
  `from_normalized_toml_str` parser for OpenBMP normalized harmonic TOML
  fixtures, std-gated fail-closed `from_icgem_gfc_str` and
  `from_icgem_gfc_str_with_metadata` parsers for static ICGEM/NGA-style
  fully-normalized `gfc` coefficient lines and their source `µ`/radius/source
  degree headers, `FiniteDifferencePinesGravity::new_from_icgem_gfc_str` to
  carry those parsed constants into the transition model, checked runtime
  high-degree EGM2008 tier requests for the planned 70/120/360 truncations that
  must resolve against a concrete field envelope and the bounded Pines scratch
  tables, a bounded `cos(mλ)` / `sin(mλ)` recurrence table, a
  direction-cosine `(s + i t)^m` Pines
  longitude-polynomial table that remains finite at the pole, a reusable
  truncation envelope validator, a bounded Holmes-Featherstone/Pines `A_nm(u)`
  recurrence table, a body-fixed
  synthesis-point validator, a normalized
  Pines scalar-potential correction sum over deterministic coefficient slots,
  a normalized Gottlieb-style scalar-potential recomposition oracle that
  cross-checks the Pines sum through ordinary longitude trigonometry and an
  explicit horizontal-power term,
  bounded Pines and Gottlieb-style symmetric-finite-difference acceleration
  oracles and finite-difference acceleration-gradient `Matrix3` oracles derived
  from those scalar potentials, and a public static `GravityModel` wrapper for
  the Pines oracle with
  `new_from_full_normalized_field` central-term stripping for full imported
  fields plus `acceleration_gradient_eci_s2` for the point-mass-plus-correction
  transition tensor
  whose ECI axes are currently treated as body-fixed until frame-rotating force
  wiring lands, and a complete default-zero coefficient-slot iterator for
  future synthesis kernels, rejects duplicate or
  out-of-envelope coefficient entries, parses a provenance-pinned WGS84
  normalized degree-2 fixture, parses synthetic non-zonal degree-4 fixtures
  for the general TOML schema and ICGEM-style GFC schema, proves its
  degree-2/order-0 path is byte-identical to `J2Gravity`, and stays finite near
  the pole. This is **not yet** the full Pines/Gottlieb high-degree EGM2008
  kernel; real high-degree EGM2008 coefficient ingestion and external
  HARMONIC_SYNTH validation remain the central gap.
- `ThirdBody` / `ThirdBodyGravity<G,E>` — central + Σ third-body perturbation
  with Battin's cancellation-free `f(q)` formulation in
  `third_body_perturbation()`. The regression suite now compares the Battin form
  to the naive difference in a well-conditioned case and documents the
  pathological |r| ≪ |r_b| case where the naive difference drops a small
  component to zero.
- `SolarRadiationPressure<E>` — opt-in cannonball SRP with 1 AU pressure
  scaling, configurable `C_r·A/m`, and conical Earth shadow from exact apparent
  solar/Earth disk overlap. Tests cover full sunlight magnitude, umbra,
  penumbra symmetry, full-sun exit, and the circular-segment overlap formula.
- `RelativisticCorrection` — opt-in Schwarzschild first-post-Newtonian term
  with named `c`, `β=γ=1`, and typed `Position3<Eci>`/`Velocity3<Eci>` input.
  Tests cover the circular-orbit closed-form magnitude near the published
  ≈3e-10 m/s² scale at GNSS altitude and fail-closed singular/non-finite state.
- No tides, no SRP macro-model/re-radiation, and no Lense-Thirring/de Sitter
  terms anywhere in the file.

### 2.2 Frames — `crates/openbmp-physics/src/frames.rs` (2835 lines)

- `enum FrameProfile { ToyFixedEarth, Wgs84UniformRotation, IersTabulated }`; `FrameContext` exposes `eci_to_ecef_position/velocity`, rotation/rate matrices, ECEF↔NED helpers.
- The `IersTabulated` profile uses an **equinox-based** chain: `precession_angles_iau1976()` + `nutation_angles_iau1980()` (the IAU 1980 nutation series, transcribed with permission from SOFA per the third-party notice at line ~1058), then ERA, then polar motion. **There is no CIO (X, Y, s) path, no frame-bias, no IAU 2006 precession, no celestial-pole offsets dX/dY.** Confirmed: zero hits for `CIO`/`2000A`/`XYS`.
- `EarthOrientationSample` carries `polar_motion_x/y_rad`, `ut1_minus_utc`, optional LOD; `EarthOrientationTable` Lagrange/linear-interpolates. **No `dX`, `dY` fields.** This is the EOP plumbing the CIO upgrade extends.
- `WGS84_A_M`, `WGS84_INV_FLATTENING`, `WGS84_MU_M3_S2`, `WGS84_OMEGA_RAD_S` live here (allow-listed source-of-truth constants).

### 2.3 Atmosphere & wind

- `trait AtmosphereModel { fn sample(&self, altitude_geometric_m, time) -> Result<AtmosphereSample, PhysicsError> }` (`atmosphere/mod.rs`). Implementations: NRLMSISE-00 (`nrlmsise00*.rs`, ~150 KB of coefficients + model, validated to reference), US Standard 1976, piecewise-exponential, isothermal.
- `trait WindModel { fn wind_ned_m_s(...) }` (`wind/mod.rs`); HWM14 quiet + DWM07 disturbance evaluated from bundled binary coefficient blobs (`wind/hwm14.rs`); constant/layered/Dryden-gust winds.
- **These are mean models only.** There is no correlated-perturbation / dispersion decorator — the SOTA gap for Monte Carlo loads/footprint/abort analysis.

### 2.4 Geomagnetic — `crates/openbmp-physics/src/magnetic/`

- `trait MagneticModel` + `MagneticFieldEci`; `Wmm2025` is a faithful 12-degree WMM port (Schmidt-normalized ALF recursion; `WMM.COF` SHA-pinned; 100-row reference test set; valid `[2025.0, 2030.0)`).
- **No IGRF-14** (degree 13, secular variation), **no field gradient/Jacobian.**

### 2.5 Ephemeris — `crates/openbmp-physics/src/ephemeris.rs`

- `enum CelestialBody`, `trait EphemerisModel { fn body_position_eci_m(...) }`, `LowPrecisionSunMoonEphemeris`, and a **real `SpkEphemeris` DAF/SPK byte parser** (types 1/2/3/10/etc., `SpkAberrationCorrection`, `SpkFixedFrame`). The ephemeris substrate for third-body is built; it needs the Battin acceleration form and a TT↔TDB time-scale bridge wired in.

### 2.6 Data & provenance

- `data/gravity/` holds `wgs84-j2.toml` + `provenance.md`. `data/magnetic/WMM.COF`, `data/wind/hwm14/*` are SHA-pinned. The four-pillar provenance contract (`00-overview.md` §3 invariant 5) is live; new coefficient files (EGM2008 blocks, IGRF-14, ERFA X/Y/s series, EOP, FES2014) each need a sibling `provenance.md` + SHA pin and (for real constants) a tripwire allow-list entry **in the same PR**.

**Baseline lock (Tier 0).** Point-mass + J₂…J₆ zonal (deg-6 cap), IAU 1976/1980
equinox frames, NRLMSISE-00 / HWM14 / WMM2025 mean models, the SPK parser. This
is the regression baseline: every tier below is forward-only **additive** and the
existing goldens stay byte-identical until a scenario opts in.

---

## 3. Target architecture

All new capability lands in `openbmp-physics` (L2) behind the existing trait
surfaces, plus one small new leaf crate proposal for the high-degree harmonic
synthesis kernel that gravity, magnetics, and tides all share. No up-layer edges;
the FC-portability lock (`00-overview.md` §3 invariant 3) is untouched because
none of this is consumed by `openbmp-fc` directly — GNC receives state through the
existing FC-bridge/sensor boundary.

### 3.1 New / changed crates

```text
NEW (proposed):
  openbmp-harmonics  L1     Singularity-free spherical-harmonic synthesis kernel:
                            Pines + normalized Gottlieb + Holmes–Featherstone
                            scaled ALF recursion + the shared coefficient block
                            object. Consumed by gravity, magnetics, and the tide
                            corrections. Leaf crate (depends only on core+nalgebra)
                            so it can be reused without pulling in all of physics.
                            Review the boundary in WP-08.1 before building.

CHANGED:
  openbmp-physics    L2  + TesseralGravity, ThirdBodyGravity (Battin f(q)),
                         SolarRadiationPressure, RelativisticCorrection,
                         TideCorrection, CioFrameModel, Igrf14, PerturbedAtmosphere
  openbmp-core       L0  + DeterministicRng stream domains for the dispersion field
                         (a new domain tag alongside the existing wind/sensor tags)
  openbmp-runner     L7  + force-model stack wiring; dispersion-ensemble hooks
                         (couples to openbmp-mc from doc 11)
```

> The agent proposes `openbmp-harmonics` in WP-08.1's first commit (skeleton +
> placement justification) and gets the boundary reviewed before the kernel
> lands, per `13` §5. If review prefers to keep the kernel inside
> `openbmp-physics` as a private module, that is acceptable — the requirement is
> that gravity/magnetics/tides share one verified recursion, not that it be a
> separate crate.

### 3.2 The harmonic-synthesis kernel (shared by gravity, magnetics, tides)

The geopotential and the geomagnetic scalar potential are the same machinery up
to a normalization scale factor; sharing one verified recursion is both a
correctness win and a cost saver (the research pack: *"Low Rust cost once the
gravity ALF kernel exists — share the recursion, swap normalization"*).

**Pines direction-cosine formulation (singularity-free).** With `r = |𝐫|` and
direction cosines `s = x/r`, `t = y/r`, `u = z/r`, the derived normalized
Legendre functions `A_{n,m}(u)` follow the standard-forward-column
(Holmes–Featherstone) recursion:

```text
seed:      A_{0,0} = 1,  A_{1,0} = u·√3,  A_{1,1} = √3
diagonal:  A_{m,m} = √((2m+1)/(2m)) · A_{m-1,m-1}            (m ≥ 2)
column:    A_{n,m} = u·g_{n,m}·A_{n-1,m} − h_{n,m}·A_{n-2,m}
  g_{n,m} = √((2n+1)(2n−1)/((n−m)(n+m)))
  h_{n,m} = √((2n+1)(n+m−1)(n−m−1)/((2n−3)(n+m)(n−m)))
```

The longitude polynomials recur as `r_m = s·r_{m−1} − t·i_{m−1}`,
`i_m = s·i_{m−1} + t·r_{m−1}` (`r_0 = 1`, `i_0 = 0`). Lumped sums
`D_{n,m} = C̄_{n,m}·r_m + S̄_{n,m}·i_m` and the partials `E, F` assemble the
acceleration **directly in Cartesian** as

```text
a = (µ/r²)·[ a₁·𝐢̂ + a₂·𝐣̂ + a₃·𝐤̂ + a₄·𝐫̂ ]
```

where the four scalar accumulators are summed over `n, m` using `A_{n,m}`,
`A_{n,m+1}` and the radial attenuation `ρ_{n+1} = (µ/r)(R⊕/r)ⁿ / R⊕` (Pines 1973;
NASA/TP-2016-218604 Eqs.). No `sin/cos(lat)` ever appears, so there is **no polar
singularity** — the precise property the current zonal model never has to face.

**High-degree robustness.** Past degree ≈ 170 the normalized ALFs overflow f64.
Carry the Holmes–Featherstone **scaled recursion** (multiply `A` by `10⁻²⁸⁰`, or
carry an explicit exponent "X-number") to reach degree 2190; this is a
configuration of the same kernel, not a second one.

**Normalized Gottlieb (the cross-check oracle, NASA/TP-2016-218604 §gottliebnorm).**
A *second, independent* body-fixed spherical synthesis: fully-normalized ALFs
`P̄_{n,m}(sin φ)` by Gottlieb's recursion, the gradient assembled in spherical
components `(∂U/∂r, ∂U/∂φ, ∂U/∂λ)` with `cos φ` folded into the lumped
coefficients so the result is non-singular at the poles, then rotated to
Cartesian. **The two kernels must agree to ≈ 1e-12 relative acceleration** at
random (r, lat, lon) including near-pole and equatorial points — a cheap, strong,
internal correctness gate (research pack V&V benchmark #2). This is the
solver-consumer posture's "in-repo reduced tier + code-to-code case" applied
*internally*: two formulations validating each other.

```rust
/// Fully-normalized spherical-harmonic coefficient block (C̄, S̄) to a
/// declared maximum degree/order, with provenance + permanent-tide convention.
pub struct HarmonicField {
    pub max_degree: usize,
    pub max_order: usize,
    pub reference_radius_m: f64,      // R⊕ for this field (geopotential or geomag)
    pub gm_m3_s2: Option<f64>,        // µ for gravity; None for magnetics
    pub c_bar: Box<[f64]>,            // packed lower-triangular (n,m), normalized
    pub s_bar: Box<[f64]>,
    pub tide_system: TideSystem,      // ZeroTide | TideFree | MeanTide (must match data)
    pub provenance_id: ProvenanceId,  // SHA-pinned source-of-truth reference
}

pub enum HarmonicNorm { FullyNormalized, SchmidtQuasiNormalized }

/// Singularity-free synthesis. `scale_exponent` selects plain vs.
/// Holmes–Featherstone scaled recursion for very high degree.
pub trait HarmonicSynthesis {
    /// Gravitational/scalar-potential acceleration (or −∇V) in the field's
    /// body-fixed frame, plus optionally the 3×3 gradient (gravity-gradient
    /// tensor / magnetic Jacobian).
    fn acceleration_bodyfixed(
        &self, r_bodyfixed: Vector3<f64>, max_degree: usize, max_order: usize,
    ) -> Result<Vector3<f64>, PhysicsError>;

    fn acceleration_and_gradient(
        &self, r_bodyfixed: Vector3<f64>, max_degree: usize, max_order: usize,
    ) -> Result<(Vector3<f64>, Matrix3<f64>), PhysicsError>;
}

pub struct PinesSynthesis<'f>  { field: &'f HarmonicField, scratch: PinesScratch }
pub struct GottliebSynthesis<'f>{ field: &'f HarmonicField, scratch: GottliebScratch }
```

Scratch buffers (`A`, `r_m`, `i_m`, the lumped sums) are pre-allocated to
`(N+1)(N+2)/2` at construction so per-call evaluation **allocates nothing** —
required for the no-hot-path-allocation contract (`00-overview.md` §3 invariant 4)
and for deterministic MC throughput.

### 3.3 Gravity force model

```rust
/// Full tesseral spherical-harmonic gravity. Wraps a HarmonicField and a
/// HarmonicSynthesis; rotates ECI→ECEF for evaluation, ECEF→ECI for the result,
/// through the active FrameContext so it stays consistent with the frame tier.
pub struct TesseralGravity<S: HarmonicSynthesis> {
    synthesis: S,
    max_degree: usize,    // runtime-selectable truncation (start 70 → 120 → 360)
    max_order: usize,
    frame: FrameRef,      // GCRS↔ITRS provider (Tier 3) or equinox (Tier 0)
}
impl<S: HarmonicSynthesis> GravityModel for TesseralGravity<S> { /* ... */ }
```

The static field is evaluated in the Earth-fixed frame (where C̄/S̄ are defined),
which is *why* the frame tier and the gravity tier are coupled: tesseral terms are
longitude-dependent, so a wrong Earth-rotation angle smears them. At Tier 1 this
uses the existing equinox rotation; at Tier 3 it uses the CIO transform.

**Third-body (Battin `f(q)`, cancellation-free).** Replace the naive difference in
`third_body_perturbation()` with: define `q = 𝐫·(𝐫 − 2𝐬)/|𝐬|²`,
`f(q) = q(3 + 3q + q²)/(1 + (1+q)^{3/2})`, then

```text
a_3b = −(µ_b/|𝐬−𝐫|³)·[ 𝐫 + f(q)·𝐬 ]
```

summed over bodies, with `𝐬` (Earth-centered body position) from the SPK
ephemeris at TDB. This keeps full precision when |r| ≪ |s| (LEO + the Sun), which
the current form does not (Battin §8.5; Montenbruck–Gill §3.3).

**Solar radiation pressure (cannonball + conical shadow).**

```text
a_SRP = −ν · P_⊙ · (AU/|r_{⊙→sc}|)² · C_r · (A/m) · r̂_{⊙→sc}
```

with `P_⊙ = 4.560e-6 N/m²` at 1 AU, `C_r ∈ [1,2]`, and `ν ∈ [0,1]` the conical
shadow factor from the exact circular-segment overlap of the apparent solar/Earth
discs (Montenbruck–Gill §3.4): apparent radii `a = asin(R_⊙/|r_{sc→⊙}|)`,
`b = asin(R⊕/|r_sc|)`, separation `c = acos(−r̂_sc·r̂_{sc→⊙})`; `ν = 1` if
`c ≥ a+b`, total umbra if `c < |b−a|` (and `b > a`), else the penumbra fraction
via the segment-area formula. Smooth and differentiable (matters for the variational
equations doc 07 may want). The Moon is added as a second occulter at the top tier.

**Relativity (Schwarzschild; the only PN term worth shipping early).**

```text
a_Sch = (GM/(c²r³))·{ [2(β+γ)GM/r − γ(𝐯·𝐯)]·𝐫 + 2(1+γ)(𝐫·𝐯)·𝐯 }
```

with PPN parameters `β = γ = 1` and `c` named explicitly (IERS Conventions 2010
Ch.10). Lense–Thirring (frame-dragging) and de Sitter (geodetic) are 2+ orders
smaller and shipped only at the POD-grade tier, behind a config flag.

```rust
pub struct SolarRadiationPressure<E: EphemerisModel> {
    ephemeris: E, reflectivity_cr: f64, area_over_mass_m2_kg: f64,
    occulters: Vec<Occulter>,   // Earth (+ Moon at top tier)
}
pub struct RelativisticCorrection { ppn_beta: f64, ppn_gamma: f64,
    include_lense_thirring: bool, include_de_sitter: bool }
```

**Tides (time-varying ΔC̄_{nm}, ΔS̄_{nm} on the static field, IERS 2010 Ch.6).**
Solid-tide Step 1 (frequency-independent, nominal Love numbers `k_{nm}`, n = 2,3)
gives ≈ 90 % of the effect in ≈ 80 lines:

```text
ΔC̄_{nm} − i·ΔS̄_{nm} = (k_{nm}/(2n+1)) · Σ_{j=Moon,Sun}
    (µ_j/µ⊕)(R⊕/r_j)^{n+1} · P̄_{nm}(sin φ_j) · e^{−imλ_j}
```

Step 2 frequency-dependent corrections to C20/C21/S21/C22/S22 (IERS tables
6.5a–c, Doodson/Delaunay arguments), the pole tide
`ΔC21 = −1.333e-9·(m₁ + 0.0115·m₂)`, `ΔS21 = −1.333e-9·(m₂ − 0.0115·m₁)`, and
ocean tides (FES2014b coefficient grids) are the IERS-conventional top tier.
**Critical:** the permanent-tide convention (`tide_system`) of the applied
corrections must match the EGM2008 coefficient convention or the secular C20 is
double-counted — the `HarmonicField.tide_system` field exists to enforce this
fail-closed at load.

### 3.4 IAU 2006/2000A CIO frames

Replace the equinox chain in the `IersTabulated` profile with the CIO transform
(IERS TN36 Ch.5; Capitaine & Wallace 2006). The full GCRS→ITRS rotation is

```text
[ITRS] = W(t)·R(t)·Q(t)·[GCRS]
  Q(t) = celestial motion of the CIP, from (X, Y, s):
         a = 1/(1+cos d),  with the rotation matrix built from X, Y and s
  R(t) = R3(−ERA),  ERA(Tu) = 2π(0.7790572732640 + 1.00273781191135448·Tu),
                    Tu = JD(UT1) − 2451545.0
  W(t) = R3(−s′)·R2(x_p)·R1(y_p),   s′ = −47e-6·t  arcsec
```

X, Y come from the IAU 2006/2000A series (≈ 1600 terms for X, ≈ 1300 for Y at full
A), the CIO locator `s(t) = −XY/2 + polynomial + small series`. The observed
**celestial-pole offsets dX, dY** from `finals2000A` are *added to* the series X, Y.
ERFA names to port/transcribe: `eraXys06a`, `eraC2ixys`, `eraEra00`, `eraPom00`,
`eraSp00`, `eraDtdb`.

```rust
/// CIO-based GCRS↔ITRS provider. Ingests the IAU 2006/2000A X/Y/s series and the
/// EOP table; produces the full rotation + its rate for velocity transforms.
pub struct CioFrameModel {
    xys_series: &'static XysSeries,         // transcribed from ERFA (BSD)
    eop: EarthOrientationTable,             // extended with dX, dY (see below)
    time_scales: TimeScaleBridge,           // UTC↔UT1↔TT↔TDB
}
impl CioFrameModel {
    pub fn gcrs_to_itrs_matrix(&self, t: SimTime) -> Result<Matrix3<f64>, FrameError>;
    pub fn gcrs_to_itrs_rate(&self, t: SimTime) -> Result<Matrix3<f64>, FrameError>;
}
```

The existing `EarthOrientationSample` (polar motion, UT1−UTC, LOD) is **extended
with `cip_offset_x_rad` (dX), `cip_offset_y_rad` (dY)** — the research pack notes
the EOP plumbing + Lagrange interpolation is already half-built, so this is an
additive field + ingestion change, not a rewrite. A `TimeScaleBridge` adds the
UTC↔UT1 (from EOP) and TT↔TDB (`eraDtdb` periodic series) conversions that
third-body (TDB) and ERA (UT1) need.

`FrameProfile` gains a fourth variant `IersCio` so the equinox `IersTabulated`
path stays byte-identical (byte-stable-by-default); scenarios opt in explicitly.

### 3.5 IGRF-14 geomagnetic

Share the `HarmonicSynthesis` kernel with `HarmonicNorm::SchmidtQuasiNormalized`;
add secular-variation time interpolation `g_{nm}(t) = g_{nm}(t₀) + (t−t₀)·ġ_{nm}`
between 5-year epochs, and expose the field gradient via
`acceleration_and_gradient` for sensor/torque fidelity. `B = −∇V`,
`V = R⊕·Σ_n (R⊕/r)^{n+1} Σ_m [g_{nm}(t)cos mλ + h_{nm}(t)sin mλ]·P̄_{nm}(cos θ)`.

### 3.6 GRAM-style perturbed atmosphere (Monte Carlo dispersion)

A **decorator** wrapping any `AtmosphereModel` + `WindModel` mean, adding a seeded,
spatially/temporally correlated perturbation field. The mean already exists and is
validated — this is the actual SOTA gap for dispersion analysis.

Per step, generate a Gaussian perturbation for density/T/wind and impose
autocorrelation along the trajectory via a first-order Markov (Ornstein–Uhlenbeck)
recursion:

```text
p_k = ρ_corr · p_{k−1} + σ_k · √(1 − ρ_corr²) · z_k,   z_k ~ N(0,1)
ρ_corr = exp(−d/L)    d = step displacement,  L = correlation length
                      (separate horizontal ~hundreds km, vertical ~km scales)
density: ρ_disp = ρ_mean · (1 + p_k^density)          (multiplicative)
```

The small-scale gust piece follows a **Dryden** spectrum (the repo already has a
Dryden gust filter + a `WIND` RNG domain tag — reuse the pattern). All randomness
derives from `DeterministicRng` with a new domain-separated `(seed, step, case_id,
ATMO)` tuple so each MC case is reproducible and the byte-diff gate extends to the
ensemble.

```rust
pub struct PerturbedAtmosphere<A: AtmosphereModel> {
    mean: A,
    sigma_envelope: SigmaEnvelope,      // altitude → σ_density, σ_T, σ_wind (documented)
    corr_length_horizontal_m: f64,
    corr_length_vertical_m: f64,
    dryden_scale_m: f64,
    rng_domain: AtmoDispersionDomain,   // (scenario_seed, step, case_id)
    state: OuState,                     // carried p_{k−1} per channel
}
impl<A: AtmosphereModel> AtmosphereModel for PerturbedAtmosphere<A> {
    fn sample(&self, alt, time) -> Result<AtmosphereSample, PhysicsError>;
}
```

**Honest labeling (ceiling item 2):** the σ envelopes and correlation lengths are
a *documented engineering envelope per the GRAM methodology*, not GRAM's
proprietary tables — labeled a **method-replica**, validated statistically, never
presented as a GRAM port.

### 3.7 Fidelity tiers `T0…T5`

| Tier | Capability | Ships value | Earns (target) |
|---|---|---|---|
| **T0** | *(baseline lock)* point-mass + J₂…J₆ zonal (deg-6 cap), IAU 1976/1980 equinox frames, NRLMSISE-00/HWM14/WMM2025 means, SPK parser | — (existing) | (regression baseline) |
| **T1** | Full **static tesseral** gravity: Pines + Gottlieb oracle, EGM2008 to runtime-selectable degree (70→120→360); J₂-only truncation reproduces `J2Gravity` byte-pattern | real ground tracks, GEO tesseral resonance, no frame/atmo change needed | `research` (NGA HARMONIC_SYNTH benchmark) |
| **T2** | **Third-body (Battin f(q)) + cannonball SRP w/ conical shadow + Schwarzschild** | credible high-altitude/coast, cislunar-adjacent | `validated-toy` → `research` (published magnitudes, Orekit code-to-code) |
| **T3** | **IAU 2006/2000A CIO frames + full EOP** (xp,yp,UT1,dX,dY,LOD); GCRS↔ITRS vs ERFA | arcsec→sub-mas frames; honest external-ephemeris/telemetry comparison | `research` (ERFA element-wise) |
| **T4** | **Geophysical fidelity:** solid tides (Step 1→freq-dependent), pole tide, ocean tides (FES2014b); **IGRF-14** + secular variation + field gradient; optional Lense–Thirring/de Sitter | "IERS-conventional" | `research` (IERS worked examples; ILRS proxy) |
| **T5** | **Monte Carlo dispersion atmosphere:** PerturbedAtmosphere (OU + Dryden) + ensemble runner → ~1000 reproducible dispersed profiles | loads/footprint/abort statistics | `validated-toy` (method-replica; statistical V&V) |

Tiers are independently shippable (`13` §1): T1 lands real value with no frame or
atmosphere change; T5 couples to the doc 11 Monte Carlo harness.

---

## 4. Invariant preservation

Concretely for this dimension (against `00-overview.md` §3):

- **Byte-determinism (1).** Pure f64, **locked operand order** on every harmonic
  sum (fixed `n` then `m` iteration over the packed triangular array — never
  `HashMap` iteration), **no `mul_add`**, no wall-clock, no system RNG. The
  dispersion field draws only from `DeterministicRng` with a domain-separated
  `(seed, step, case_id, ATMO)` tuple. The CI byte-diff gate runs the canonical
  scenarios twice and `openbmp diff` must be byte-identical.
- **Byte-stable-by-default (2).** Every new model is **off by default** and gated
  by an explicit scenario block (e.g. `[gravity.tesseral]`, `[frames.cio]`,
  `[atmosphere.dispersion]`), exactly like the established `[vehicle.bending]`
  pattern. New `FrameProfile::IersCio` variant leaves `IersTabulated` untouched.
  The J₂-truncation byte-equivalence test (`TesseralGravity` at degree 2 ≡
  `J2Gravity`) is the concrete proof the new path does not perturb the old one.
- **FC-portability lock (3).** All of this lands in `openbmp-physics` (L2) /
  `openbmp-harmonics` (leaf); **none is a new `openbmp-fc` dependency.** GNC
  receives gravity/frame/field data through the existing FC-bridge / sensor
  boundary — never by importing the simulator. The `fc_dependency_tripwire.rs`
  stays green unchanged.
- **No-hot-path-allocation (4).** Synthesis scratch is pre-allocated at
  construction; per-`gravity_eci_m_s2` and per-`sample` calls allocate nothing.
  Time is read only through the injected clock path; no `Instant::now`.
- **Four-pillar provenance (5).** EGM2008 / GGM05 coefficient blocks, IGRF-14
  coefficients, the ERFA X/Y/s series, IERS EOP, and FES2014 grids each land under
  `data/<thing>/` with a sibling `provenance.md`, SHA-256 pin, license status, and
  a validation label; any new real-constant source-of-truth file is added to the
  `inline_data_tripwire.rs` allow-list **in the same PR**. **Authoring corollary
  honored throughout this doc:** GM⊕, J₂, R⊕ are referred to symbolically, never
  as literals (the inline-data tripwire scans `docs/*.md`).
- **Validation labels (8).** Each tier declares exactly one label (§5);
  textbook/analytic tiers earn `validated-toy`, benchmark-anchored tiers earn
  `research`. The dispersion atmosphere is labeled a **method-replica**, never a
  GRAM port. No `flight-qualified`/`certified` anywhere.
- **Requirements traceability (10).** Each WP adds `requirements.toml` entries
  with verification evidence; `check_requirements_traceability.py` stays green.

---

## 5. V&V plan

Five-layer ladder (`docs/verification.md`): MMS / analytic → model-verification →
code-to-code → public benchmark → UQ. Each tier earns the label in the table.

### 5.1 Tolerance tables (per tier)

**T1 — tesseral gravity**

| Case | Method | Tolerance | Label |
|---|---|---|---|
| Pines vs normalized Gottlieb, random (r, lat, lon) incl. near-pole & equator | internal code-to-code | ≤ 1e-12 relative acceleration | `checked` |
| `TesseralGravity` deg-2 truncation vs existing `J2Gravity` | byte regression | byte-identical | `checked` |
| NGA EGM2008 HARMONIC_SYNTH geoid/anomaly grid points | public benchmark | published precision (sub-µGal / sub-mm geoid at deg 360) | `research` |
| GeographicLib `GravityModel` (EGM2008) at sample points | code-to-code | ≤ 1e-10 relative | `research` |

**T2 — third-body / SRP / relativity**

| Case | Method | Tolerance | Label |
|---|---|---|---|
| Battin `f(q)` vs naive difference, |r|≪|s| | numerical-stability MMS | f(q) finite to f64; naive loses ≥ 6 digits (documents the bug fixed) | `checked` |
| Schwarzschild magnitude in LEO | analytic vs published | ≈ 3e-10 m/s² radial, within 1 % | `validated-toy` |
| Conical-shadow factor ν across an eclipse pass | analytic geometry | smooth 1→0→1; penumbra width matches segment-area closed form | `checked` |
| LEO + Sun/Moon + SRP propagated vs **Orekit** matched force model | code-to-code | ≤ 1 m secular position growth / day | `research` |

**T3 — CIO frames**

| Case | Method | Tolerance | Label |
|---|---|---|---|
| GCRS↔ITRS matrix vs `eraXys06a`/`eraC2ixys`/`eraPom00`/`eraEra00`/`eraSp00` over a (UTC, EOP) grid | code-to-code (ERFA oracle) | ≤ 1e-12 element-wise | `research` |
| IERS TN36 Ch.5 worked X, Y, s at the reference date | analytic | reproduce tabulated digits | `research` |

**T4 — geophysical**

| Case | Method | Tolerance | Label |
|---|---|---|---|
| Solid-tide ΔC2m vs IERS Ch.6 worked example | analytic | reproduce tabulated values | `research` |
| IGRF-14 vs NCEI sample (lat,lon,alt,date)→B test values | public benchmark | ≤ 0.1 nT | `research` |
| LAGEOS/Starlette full-stack propagation vs ILRS precise orbit | public benchmark (truth proxy) | report residuals honestly (no flight-accuracy claim) | `research` |

**T5 — dispersion atmosphere**

| Case | Method | Tolerance | Label |
|---|---|---|---|
| Ensemble mean of N dispersed profiles → NRLMSISE-00 mean | statistical | mean within 1 σ_mean of the unperturbed model | `validated-toy` |
| Ensemble variance vs observed storm/quiet spread | statistical bracket | variance bracketed by published density spread | `validated-toy` |
| OU autocorrelation along trajectory | analytic | sample ACF matches `exp(−d/L)` to sampling error | `checked` |
| Same seed → byte-identical ensemble; different seed → different, both reproducible | determinism | byte-diff gate green | `checked` |

### 5.2 Oracles & external sanity checks

- **ERFA** (BSD): the X/Y/s and rotation oracle for T3 (port the routines *and*
  use as regression truth).
- **GeographicLib** (MIT): EGM2008/IGRF cross-validation for T1/T4.
- **Orekit** (Apache-2.0) / **GMAT** (NOSA): offline propagated-orbit code-to-code
  for the full T2/T4 force stack; commit only the numerical outputs as fixtures
  (keeps the license posture clean — GMAT used Pines, so it is the natural Pines
  oracle).
- **NGA HARMONIC_SYNTH** test vectors for T1; **NCEI** IGRF/WMM test values for T4.
- **CelesTrak / Gannon May-2024 storm** density (arXiv:2406.08617) as an external
  sanity check on the space-weather coupling and the dispersion envelope.

---

## 7. Dependencies on other parity docs

- **`12-determinism-realtime-and-compute.md`** — the deterministic-parallel-MC
  substrate and the `DeterministicRng` stream-domain discipline that T5 builds on;
  Phase A co-requisite.
- **`11-monte-carlo-uq-and-validation.md`** — `openbmp-mc` (LHS/Sobol/Wilks)
  consumes the `PerturbedAtmosphere` per-case seeding; the UQ error-budget object
  (`openbmp-uq`) carries the dispersion σ envelopes and EOP/space-weather error
  bars into the credibility record. T5 wires into the campaign runner there.
- **`07-trajectory-optimization-and-mission-design.md`** — the force-model stack
  (T1–T4) is the propagator the optimizer/closed-loop guidance integrates;
  variational equations want the smooth, differentiable SRP shadow.
- **`09-sensors-navigation-and-actuators.md`** — IGRF-14 + field gradient feed the
  magnetometer-aided EKF; the perturbed atmosphere feeds force-based IMU dispersion.
- **`06-gnc-coupled-mimo-and-control.md`** — receives the improved environment
  through the FC-bridge/sensor boundary (never a direct dependency).
- **`03-aerodynamics-database-and-cfd-coupling.md`** — consumes density/wind/Mach
  from the (perturbed) atmosphere for the aero-DB dispersion.
- **`01-flexible-multibody-dynamics.md`** — the gravity-gradient tensor
  (`acceleration_and_gradient`) supplies gravity-gradient torque to the multibody
  tree.
- **`00-overview.md` / `13-agent-execution-playbook.md`** — invariants, gate set,
  backlog schema, solver-consumer posture.

---

## 8. Open-source leverage

| Tool | License | Mode | Use |
|---|---|---|---|
| **IAU SOFA / ERFA** (liberfa) | 3-clause BSD | **port + oracle** | Transcribe the IAU 2006/2000A X/Y/s series and the transform routines (`xys06a`, `c2ixys`, `era00`, `pom00`, `sp00`, `dtdb`) to deterministic Rust; ERFA's C is directly transcribable and is the T3 regression oracle. **Port from ERFA, never SOFA** (SOFA is "unaltered redistribution only"; ERFA exists to lift that). |
| **GeographicLib** | MIT/X11 | **reference + port** | `GravityModel`/`MagneticModel` evaluators (EGM2008/EGM96, WMM/IGRF) for cross-validation; a clean reference for normalized-ALF recursion and the `.cof`/`.egm` coefficient formats. Datasets it ships are public-domain. |
| **Orekit** | Apache-2.0 | **code-to-code oracle** (offline) | Full force-model stacks (tesseral, third-body, tides, SRP+conical shadow, relativity); run offline, commit numerical outputs as fixtures. Do not couple at runtime (JVM). |
| **NASA GMAT** | NOSA v1.1+ | **code-to-code oracle** (offline) | Pines-algorithm and end-to-end propagation V&V (GMAT uses Pines). Commit only numerical outputs (data, not code) so the license posture is unaffected. |
| **NAIF SPICE + de440/de440s kernels** | NAIF/JPL free | **ingest (already coupled)** | The existing SPK parser reads these; standardize on de440s for Sun/Moon/planet third-body positions and as the ephemeris benchmark. Vendor a small kernel subset for tests. |
| **IERS EOP 14 C04 / finals2000A** | open w/ attribution | **ingest (reference INPUT)** | EOP (xp, yp, UT1−UTC, dX, dY, LOD) for the CIO frame. Reference data, not a flight input. |
| **CelesTrak SW-All.csv** | free (Kelso/CSSI) | **ingest (reference INPUT)** | F10.7/Ap drivers for NRLMSISE-00; documents the exact MSIS column format. |
| **EGM2008 / GGM05 / GOCO coefficients (NGA/ICGEM)** | public-domain / open | **ingest** | The C̄/S̄ coefficient blocks for `HarmonicField`. |
| **IGRF-14 coefficients (IAGA/NCEI)** | public domain | **ingest** | Degree-13 main-field + secular-variation coefficients. |
| **FES2014b ocean-tide atlas (AVISO)** | scientific-use, attribution | **optional ingest** | Ocean-tide spherical-harmonic coefficients for the T4 ocean-tide corrections; treat as an optional auxiliary download (check AVISO terms before vendoring), not a committed dataset. |
| **nalgebra** | Apache-2.0/BSD | **reuse (existing dep)** | All matrix/vector algebra in the CIO transform and gravity assembly; keep the deterministic f64 discipline. |

Per the solver-consumer rule (`00` §1.1 / `13` §6): this dimension is pure
simulation, so the leverage is *port + code-to-code oracle*, not solver-ingestion;
the "in-repo reduced tier" is realized internally as the Pines↔Gottlieb mutual
cross-check and the J₂-truncation regression.

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the full `13` §2 gate set.

---

**WP-08.1 — Singularity-free harmonic-synthesis kernel + Pines/Gottlieb gravity**
- **implementation_status:** partial. `crates/openbmp-physics/src/gravity.rs`
  now exposes `TesseralGravity`, `FiniteDifferencePinesGravity`,
  `DegreeTwoTesseralCoefficients`, `NormalizedDegreeTwoTesseralCoefficients`,
  `NormalizedHarmonicCoefficient`, `NormalizedHarmonicField`,
  `NormalizedHarmonicFieldIter`, `IcgemGfcNormalizedField`,
  `HarmonicLongitudeTrigonometry`,
  `HarmonicTruncation`,
  `HarmonicSynthesisPlan`, `HarmonicSynthesisTier`, `PinesLegendreTable`,
  `PinesLongitudePolynomials`, `PinesSynthesisPoint`, `PinesPotentialSum`,
  `GottliebPotentialSum`, and `TideSystem` as the first non-zonal static
  harmonic force surface plus reusable normalized `Cbar/Sbar`
  coefficient-field, std-gated normalized-harmonic TOML ingestion,
  std-gated ICGEM-style GFC coefficient-and-source-metadata ingestion,
  longitude-trigonometry,
  direction-cosine-longitude, truncation-validation,
  runtime high-degree tier validation for the planned 70/120/360 EGM2008
  requests, Pines-Legendre, scalar-potential summation,
  Gottlieb-style scalar recomposition, Pines/Gottlieb-style finite-difference
  acceleration and acceleration-gradient oracle substrate, plus a static
  `GravityModel` wrapper.
  `TesseralGravity::wgs84_j2()` is byte-identical to `J2Gravity`, the degree-2
  evaluator includes C21/S21 tesseral and C22/S22 sectoral terms through
  Cartesian solid-harmonic polynomials, the low-degree ingestion bridge parses
  normalized-harmonic TOML, converts fully-normalized degree-2 blocks, and
  extracts them from a normalized harmonic field parsed from
  `data/gravity/wgs84-degree2-normalized-v1.toml`, the general
  `openbmp.gravity.normalized-field.v1` parser is covered by the synthetic
  non-zonal
  `data/gravity/synthetic-degree4-normalized-field-v1.toml` fixture, the
  ICGEM-style parser and metadata bridge are covered by
  `data/gravity/synthetic-degree4-normalized-icgem-v1.gfc`, and the
  existing `Egm2008ZonalGravity` J2-J6 truncation can be rebuilt from the
  provenance-pinned
  `data/gravity/egm2008-zonal-degree6-normalized-v1.toml` normalized zonal
  fixture. Tests prove non-zonal acceleration, near-pole finite evaluation,
  point-mass degeneration, shared fully-normalized scale factors, bounded
  longitude `cos(mλ)`/`sin(mλ)` recurrence, singularity-free Pines
  direction-cosine longitude polynomials, checked truncation-envelope
  validation, checked runtime-tier resolution against 70/120/360-style field
  envelopes and the bounded Pines scratch caps, Cbar00 stripping for full GFC
  fields before finite-difference Pines correction evaluation,
  ICGEM `earth_gravity_constant`/`radius`/source-max-degree validation before
  direct `FiniteDifferencePinesGravity::new_from_icgem_gfc_str` construction,
  low-degree
  closed-form Pines Legendre recurrence, normalized
  Pines scalar-potential summation matching the existing degree-2 Cartesian
  polynomial, fail-closed normalized-harmonic TOML parsing for the WGS84
  degree-2, synthetic non-zonal degree-4, and EGM2008 zonal fixtures plus
  malformed metadata, fail-closed ICGEM-style GFC parsing for normalization,
  tide-system, source-constant, header, dynamic-line, source-envelope, and
  malformed-float errors,
  Gottlieb-style
  scalar-potential recomposition matching Pines at
  generic/equatorial/near-pole points, Pines and Gottlieb-style
  finite-difference acceleration-gradient matrices matching a finite-differenced
  independent degree-2 Cartesian oracle, finite-difference acceleration plus
  `FiniteDifferencePinesGravity` model output matching existing analytic
  degree-2 tesseral terms, model gradient degeneration to the closed-form
  point-mass tensor for zero correction fields, default-zero
  missing coefficients and iteration slots,
  normalized-field zonal fixture equivalence, and fail-closed unsupported
  degree/order/duplicate/out-of-range coefficient handling. Remaining work for
  full WP-08.1 acceptance:
  full runtime high-degree Pines synthesis over real EGM2008 coefficient
  blocks, full analytic normalized Gottlieb acceleration-gradient oracle, real
  high-degree EGM2008 coefficient ingestion/provenance/tripwire, and NGA HARMONIC_SYNTH
  benchmark tolerance tables.
- **goal:** Replace the deg-6 zonal cap with full tesseral gravity. Build the shared `HarmonicSynthesis` kernel (Pines + normalized Gottlieb oracle + Holmes–Featherstone scaled recursion) and `TesseralGravity`; load EGM2008 to a runtime-selectable degree. The central gap-closer for this dimension and a "pure win that improves all propagation" (`00` §5).
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** `openbmp-harmonics` (L1 leaf — depends only on `openbmp-core` + nalgebra; propose boundary in first commit — skeleton + placement justification before the kernel lands; may instead be a private module inside `openbmp-physics` if the review prefers)
- **touched:** `crates/openbmp-harmonics/*` (new), `crates/openbmp-physics/src/gravity.rs`, `data/gravity/` (+ EGM2008 block + `provenance.md` + SHA pin), `inline_data_tripwire.rs` allow-list
- **approach:** §3.2 Pines + Gottlieb; §3.3 `TesseralGravity`. Pre-allocated scratch, locked `n`-then-`m` operand order, no `mul_add`.
- **acceptance:**
  - new model off by default; canonical goldens byte-identical
  - `TesseralGravity` deg-2 truncation byte-identical to `J2Gravity`
  - Pines vs normalized Gottlieb agree ≤ 1e-12 relative at random (r, lat, lon) incl. near-pole/equator (tolerance table)
  - NGA HARMONIC_SYNTH benchmark points reproduced to published precision (provenance-pinned fixture)
  - EGM2008 coefficient block ingested with `provenance.md` + SHA pin; tripwire allow-list updated in-PR
  - all §2 gates green
- **validation_label:** `research`
- **dual_use_note:** far from line; exposes state→acceleration only (no target field)
- **est_effort:** ~2–3 weeks (kernel 400–700 lines + Gottlieb oracle 300–500 + coefficient ingestion)
- **parity_ceiling:** static field only; tides (WP-08.7), CIO-frame coupling (WP-08.4), and POD-grade accuracy out of scope

---

**WP-08.2 — Third-body Battin f(q) + cannonball SRP + Schwarzschild**
- **implementation_status:** partial. `third_body_perturbation()` now uses
  Battin's cancellation-free `f(q)` formulation while preserving the existing
  `ThirdBodyGravity` API. Tests prove agreement with the naive expression in a
  well-conditioned Moon-like case and document a |r| ≪ |r_b| case where the
  naive difference loses the small x-component entirely while Battin preserves
  the finite value. `SolarRadiationPressure<E>` now adds opt-in cannonball SRP
  with conical umbra/penumbra shadow, and `RelativisticCorrection` adds the
  Schwarzschild acceleration with named `c`. Remaining WP-08.2 work: an Orekit
  code-to-code force-stack fixture proving the combined LEO + Sun/Moon + SRP
  secular position-growth tolerance.
- **goal:** Fix the cancellation bug in the existing third-body path and add the two cheap, high-value perturbations enabling credible high-altitude/coast runs.
- **fidelity_tier:** T2
- **depends_on:** [WP-08.1]
- **new_crates:** []
- **touched:** `crates/openbmp-physics/src/gravity.rs` (Battin `f(q)`; new `SolarRadiationPressure`, `RelativisticCorrection`), `crates/openbmp-physics/src/ephemeris.rs` (TT↔TDB bridge usage)
- **approach:** §3.3 — Battin `f(q)`; cannonball SRP + conical shadow (Montenbruck–Gill §3.4); Schwarzschild (IERS Ch.10), `β=γ=1`, named `c`.
- **acceptance:**
  - off by default; goldens byte-identical
  - `f(q)` numerically finite where the naive difference loses ≥ 6 digits (MMS-style test documenting the fix)
  - Schwarzschild magnitude ≈ 3e-10 m/s² in LEO within 1 %
  - conical-shadow ν smooth 1→0→1 across an eclipse; penumbra width matches the segment-area closed form
  - LEO + Sun/Moon + SRP vs Orekit ≤ 1 m/day secular growth (provenance-pinned fixture)
  - all §2 gates green
- **validation_label:** `validated-toy` (analytic) → `research` (Orekit code-to-code)
- **dual_use_note:** far from line; forward perturbations only
- **est_effort:** ~1–1.5 weeks
- **parity_ceiling:** cannonball SRP only (no macro-model / re-radiation); Lense–Thirring/de Sitter deferred to WP-08.7

---

**WP-08.3 — Time-scale bridge (UTC↔UT1↔TT↔TDB) + EOP dX/dY ingestion**
- **goal:** Add the time-scale conversions and the celestial-pole-offset EOP fields that the CIO frame and TDB third-body need. Small, isolated, unblocks WP-08.4.
- **fidelity_tier:** T3 (prep)
- **depends_on:** [WP-08.2]
- **new_crates:** []
- **touched:** `crates/openbmp-physics/src/frames.rs` (`EarthOrientationSample` += `cip_offset_x/y_rad`; `TimeScaleBridge`), EOP ingestion + `data/eop/` (`finals2000A` subset + `provenance.md` + SHA pin)
- **approach:** §3.4 — UTC↔UT1 from EOP, TT↔TDB via `eraDtdb` periodic series; extend the existing Lagrange EOP interpolation to dX, dY.
- **acceptance:**
  - additive EOP fields default to zero → existing `IersTabulated` byte-identical
  - TT↔TDB vs `eraDtdb` ≤ 1e-9 s over a test grid
  - EOP subset ingested with `provenance.md` + SHA pin; tripwire allow-list updated in-PR
  - all §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** EOP is a LOCAL reference INPUT; never a targeting input
- **est_effort:** ~3–5 days
- **parity_ceiling:** ingestion only; the CIO transform itself is WP-08.4

---

**WP-08.4 — IAU 2006/2000A CIO frame transform**
- **goal:** Replace the equinox chain with the CIO (X, Y, s) GCRS↔ITRS transform, bringing frame error from arcsec to sub-mas and making external-ephemeris/telemetry comparisons honest.
- **fidelity_tier:** T3
- **depends_on:** [WP-08.3]
- **new_crates:** []
- **touched:** `crates/openbmp-physics/src/frames.rs` (`CioFrameModel`; `FrameProfile::IersCio`), ERFA X/Y/s series transcription + `data/iers/xys-series/` (+ `provenance.md` + SHA pin)
- **approach:** §3.4 — `[ITRS]=W·R·Q·[GCRS]` from X, Y, s, ERA, polar motion + s′; X/Y/s series transcribed from ERFA (BSD); dX, dY added to the series.
- **acceptance:**
  - new `IersCio` variant off by default; `IersTabulated` (equinox) byte-identical
  - GCRS↔ITRS matrix vs `eraXys06a`/`eraC2ixys`/`eraPom00`/`eraEra00`/`eraSp00` ≤ 1e-12 element-wise over a (UTC, EOP) grid (tolerance table)
  - IERS TN36 Ch.5 worked X, Y, s reproduced
  - tesseral gravity (WP-08.1) evaluated through `IersCio` agrees with the Orekit force stack to the WP-08.2 tolerance
  - all §2 gates green
- **validation_label:** `research`
- **dual_use_note:** time→rotation only; no target→command path
- **est_effort:** ~2 weeks (series transcription + regression dominate)
- **parity_ceiling:** uses ingested (not predictive) EOP; rapid-service error bars apply

---

**WP-08.5 — IGRF-14 + field gradient (share the harmonic kernel)**
- **goal:** Add IGRF-14 (degree 13, secular variation) and the magnetic-field Jacobian alongside WMM, for magnetometer-aided EKF and torque modeling.
- **fidelity_tier:** T4
- **depends_on:** [WP-08.1]
- **new_crates:** []
- **touched:** `crates/openbmp-physics/src/magnetic/` (new `Igrf14`; `acceleration_and_gradient` gradient path), `data/magnetic/` (IGRF-14 coefficients + `provenance.md` + SHA pin)
- **approach:** §3.5 — reuse `HarmonicSynthesis` with `SchmidtQuasiNormalized`; linear secular-variation interpolation between 5-year epochs; gradient via the kernel's `acceleration_and_gradient`.
- **acceptance:**
  - off by default; WMM path byte-identical
  - IGRF-14 vs NCEI sample test values ≤ 0.1 nT (tolerance table)
  - gradient verified by finite-difference of the scalar potential to ≤ 1e-7 relative
  - IGRF-14 coefficients ingested with `provenance.md` + SHA pin
  - all §2 gates green
- **validation_label:** `research`
- **dual_use_note:** sensor/torque modeling only; not navigation-to-target
- **est_effort:** ~1 week
- **parity_ceiling:** main field only (no crustal/external-field models)

---

**WP-08.6 — PerturbedAtmosphere (OU + Dryden) dispersion decorator**
- **goal:** Add the seeded, correlated density/wind perturbation field for Monte Carlo dispersion — the actual SOTA gap. A method-replica of GRAM's perturbation methodology, honestly labeled.
- **fidelity_tier:** T5
- **depends_on:** [WP-08.2]
- **new_crates:** []
- **touched:** `crates/openbmp-physics/src/atmosphere/` (`PerturbedAtmosphere`; `SigmaEnvelope`), `crates/openbmp-physics/src/wind/` (Dryden small-scale reuse), `crates/openbmp-core/src/rng.rs` (new `ATMO` domain tag), `data/atmosphere/dispersion/` (σ envelope + correlation lengths + `provenance.md`)
- **approach:** §3.6 — OU recursion `p_k = ρ·p_{k−1} + σ√(1−ρ²)·z_k`, `ρ=exp(−d/L)`; Dryden small-scale; multiplicative on density; `DeterministicRng (seed, step, case_id, ATMO)`.
- **acceptance:**
  - off by default; mean-model goldens byte-identical
  - ensemble mean → NRLMSISE-00 mean within 1 σ_mean; variance bracketed by published storm/quiet spread (tolerance table)
  - sample autocorrelation matches `exp(−d/L)` to sampling error
  - same seed → byte-identical ensemble; different seed → reproducibly different (byte-diff gate)
  - σ envelope documented as an engineering envelope; **labeled method-replica, not a GRAM port**
  - exercised through a scenario (not isolation only)
  - all §2 gates green
- **validation_label:** `validated-toy` (method-replica; statistical V&V)
- **dual_use_note:** forward dispersion ensembles only; refuse any footprint-biasing inversion request
- **est_effort:** ~1.5–2 weeks (incl. ensemble-runner wiring to doc 11 `openbmp-mc`)
- **parity_ceiling:** **not** the empirical GRAM covariance (ceiling item 2); engineering envelope, statistically validated

---

**WP-08.7 — Solid/ocean/pole tides + optional Lense–Thirring/de Sitter (IERS-conventional)**
- **goal:** The "IERS-conventional" top tier: time-varying geopotential tide corrections + the sub-dominant relativistic terms. Highest fidelity, most bookkeeping, lowest priority.
- **fidelity_tier:** T4
- **depends_on:** [WP-08.1, WP-08.4]
- **new_crates:** []
- **touched:** `crates/openbmp-physics/src/gravity.rs` (`TideCorrection`; Lense–Thirring/de Sitter in `RelativisticCorrection`), `data/gravity/tides/` (FES2014b subset optional + Love numbers + `provenance.md`)
- **approach:** §3.3 tides — solid Step 1 (Love numbers, n=2,3) → frequency-dependent (IERS tables 6.5a–c) → pole tide → ocean tides (FES2014b); enforce `tide_system` consistency with EGM2008 fail-closed. Lense–Thirring + de Sitter (IERS Ch.10) behind a flag.
- **acceptance:**
  - off by default; goldens byte-identical
  - solid-tide ΔC2m reproduces the IERS Ch.6 worked example
  - `tide_system` mismatch with the loaded EGM2008 field rejected fail-closed (Result, not panic)
  - LAGEOS/Starlette full-stack propagation vs ILRS precise orbit, residuals reported honestly (no flight-accuracy claim)
  - FES2014/Love-number data ingested with `provenance.md` + SHA pin
  - all §2 gates green
- **validation_label:** `research`
- **dual_use_note:** far from line; forward geopotential corrections only
- **est_effort:** ~2–3 weeks (frequency-dependent tables + ocean grids are bookkeeping-heavy)
- **parity_ceiling:** **not** sub-cm POD parity (ceiling item 4 — needs estimated empirical accelerations + station pipelines); code-to-code + ILRS-proxy only

---

## 10. References

1. S. Pines, "Uniform Representation of the Gravitational Potential and its Derivatives," *AIAA Journal* 11(11):1508–1511, 1973.
2. C.A. Eckman, A.J. Brown, D.R. Adamo, "Normalization and Implementation of Three Gravitational Acceleration Models" (Pines, normalized Gottlieb, Lear), NASA/TP-2016-218604. https://ntrs.nasa.gov/api/citations/20160011252/downloads/20160011252.pdf
3. S.A. Holmes, W.E. Featherstone, "A unified approach to the Clenshaw summation and the recursive computation of very high degree and order normalised associated Legendre functions," *J. Geodesy* 76:279–299, 2002.
4. R.G. Gottlieb, "Fast Gravity, Gravity Partials, Normalized Gravity, Gravity Gradient Torque and Magnetic Field," NASA CR-188243, 1993.
5. D.A. Vallado, *Fundamentals of Astrodynamics and Applications*, 4th ed., §8.6–8.7.
6. IERS Conventions (2010), IERS Technical Note 36, Ch.5 (ITRS↔GCRS), Ch.6 (Geopotential), Ch.10 (General relativistic models). https://www.iers.org/SharedDocs/Publikationen/EN/IERS/Publications/tn/TechnNote36/tn36.pdf
7. N. Capitaine, P.T. Wallace, "Precession-nutation procedures consistent with IAU 2006 resolutions," *A&A* 450:855, 2006; Wallace & Capitaine, *A&A* 459:981, 2006 (X, Y, s implementation).
8. IAU SOFA / ERFA (liberfa), 3-clause BSD: `xys06a`, `c2ixys`, `pom00`, `era00`, `sp00`, `dtdb`. https://github.com/liberfa/erfa
9. R.H. Battin, *An Introduction to the Mathematics and Methods of Astrodynamics*, Revised ed., §8.5 (cancellation-free third-body `f(q)`).
10. O. Montenbruck, E. Gill, *Satellite Orbits*, §3.3 (third body), §3.4 (SRP + conical shadow), §3.7 (tides, relativity).
11. N.K. Pavlis et al., "The development and evaluation of the Earth Gravitational Model 2008 (EGM2008)," *JGR Solid Earth* 117:B04406, 2012; NGA EGM2008 + HARMONIC_SYNTH. https://earth-info.nga.mil/
12. IGRF-14 coefficients & evaluation, IAGA, 2024. https://www.ncei.noaa.gov/products/international-geomagnetic-reference-field
13. NASA Earth-GRAM User Guide, NASA/TM-20210022157. https://ntrs.nasa.gov/citations/20210022157; Justus & Leslie GRAM technical memos (perturbation model + Dryden small-scale spectrum); MIL-STD-1797 Dryden turbulence spectra.
14. FES2014b ocean tide model, Lyard et al., *Ocean Science* 17:615, 2021. https://os.copernicus.org/articles/17/615/2021/
15. C. Hugentobler, "Orbit Perturbations due to Relativistic Corrections," IERS Conventions supporting material, Ch.10.
16. C.F.F. Karney, GeographicLib (MIT) — `GravityModel`/`MagneticModel`. https://geographiclib.sourceforge.io/
17. Orekit (Apache-2.0); NASA GMAT Mathematical Specification (Pines algorithm, NOSA).
18. NAIF SPICE Toolkit + generic kernels (de440/de440s SPK, leapseconds, PCK). https://naif.jpl.nasa.gov/naif/
19. IERS EOP 14 C04 / `finals2000A`; CelesTrak `SW-All.csv` (Kelso/CSSI). ILRS LAGEOS/Starlette precise orbits, https://ilrs.gsfc.nasa.gov/
20. NRLMSISE-00 vs Gannon May-2024 storm mass-density, arXiv:2406.08617 (external sanity check).

---

*Companion documents:* `00-overview.md` (constitution) · `13-agent-execution-playbook.md` (playbook) · per-dimension `01`–`12`.
