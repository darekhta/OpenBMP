# Sensors, Navigation Aids & Actuators

**Status:** `experimental` (design intent; this document ships no code).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**One-line summary:** replace the velocity-finite-difference specific-force
"truth", add high-rate strapdown delta-theta/delta-v with coning/sculling,
pseudorange/carrier GNSS with integrity (RAIM/ARAIM), optical and relative-nav
measurement models, and a production-class actuator stack (second-order TVC
servos, RCS minimum-impulse-bit, reaction wheels/CMGs, FADS), all behind the
existing FC/sensor boundary and all forward-only.

> Read `00-overview.md` (parity definition, invariants, DAG) and
> `13-agent-execution-playbook.md` (the §4 template this doc follows and the §5
> work-package schema) before this document. This is dimension `09`; it is a
> **Phase B** track that depends on the `05` force-accumulator surface and the
> `08` frames/atmosphere/gravity work.

---

## 1. Parity target & ceiling

### 1.1 Target capability

Production launch-vehicle and spacecraft GNC simulators (POST2/MAVERIC,
Trick/JEOD, NASA `42`, OreKit/GMAT-class flight-dynamics stacks, vendor
in-house "truth model + sensor model + actuator model" rigs) model the
sensor/navigation/actuator chain as **physical transducers**, not as
state-with-noise oracles. OpenBMP's target (audit verdict: *partial →
approaching*) is to match that **method and architecture**:

- **Accelerometers sense force, not differentiated velocity.** Specific force
  is the sum of non-gravitational forces divided by mass, evaluated at the
  sensor station with the rigid-body lever-arm kinematics — sourced from the
  same force accumulator the EOM integrator already builds, never re-derived by
  differencing a state the integrator just produced.
- **Strapdown mechanization is two-speed:** a high-rate (1–4 kHz) inner loop
  emitting integrated increments (delta-theta, delta-v) with coning/sculling
  rectification, feeding a moderate-rate (50–200 Hz) navigation update.
- **GNSS is a pseudorange/Doppler/carrier receiver** with satellite geometry
  (DOP), a per-satellite error budget (ephemeris/clock, ionosphere,
  troposphere, multipath, receiver noise), integer-ambiguity carrier phase,
  and **integrity** (snapshot RAIM, ARAIM/MHSS) — not a position oracle.
- **Optical/relative nav** is a measurement chain: star tracker (Wahba/QUEST
  + MEKF), sun/horizon sensors, radar/laser altimeter, Doppler velocimeter,
  and terrain-relative navigation (pinhole + PnP against an ingested DEM).
- **Actuators have dynamics:** finite-bandwidth second-order (and resonant
  4th/6th-order) TVC servos with rate/accel limits, backlash, and hardover
  faults; RCS thrusters with minimum-impulse-bit, PWM/PWPF modulation, rise/
  tail transients, and blowdown; reaction wheels (friction, saturation,
  stiction) and CMGs (singularity-robust steering).
- **System realism:** redundant strings with cross-strapping and voting,
  residual-based FDIR, and first-class **time-tagging + transport latency** on
  every measurement.

### 1.2 The honest parity ceiling

Validation parity is structurally unreachable here; the open substitute is to
**validate methods, bound parameters to published open ranges, and
cross-validate code-to-code.** Specifically:

1. **Flight-qualified IMU/GNSS error coefficients and Kalman tunings** for
   specific units (Honeywell, Northrop LN-200/μIRS, vendor GNSS) are
   proprietary. *Open substitute:* published datasheet ARW / bias-instability /
   scale-factor ranges plus Allan-variance fits from open datasets
   (`gnss-ins-sim`, `allan_variance`); the IEEE-952 five-component stack OpenBMP
   already ships is the credible carrier.
2. **Real multipath/ionospheric-scintillation environments and antenna
   phase-center/gain patterns** at a specific launch site under a thrust plume
   are not openly available. *Open substitute:* elevation-dependent Gauss-Markov
   multipath + ingested real broadcast ephemeris (RINEX) from public IGS
   reference stations + published ITU-R / Klobuchar / NeQuick-G ionosphere.
3. **Vendor TVC actuator transfer functions, friction/backlash maps,
   hinge-moment wind-tunnel data, and aeroservoelastic mode shapes** are
   proprietary and certification-controlled. *Open substitute:* published
   2nd/4th-order TVC models (NASA CR-820, NTRS EMA reports) + Method-of-
   Manufactured-Solutions for the servo ODE + code-to-code vs NASA `42`.
4. **Mars-2020 LVS flight maps, landmark databases, and the matching FSW** are
   restricted. *Open substitute:* synthetic DEM (public MOLA/HiRISE/SRTM) with
   self-generated landmarks and the published *error-vs-altitude envelope* as
   the order-of-magnitude target.
5. **RF spoofing/jamming HIL fidelity** needs a real RF testbed. *Open
   substitute:* signal-level injection via GPS-SDR-SIM-style pseudorange
   generation and measurement-level coherent-offset injection inside the SIL.

**No artifact in this dimension may claim unit-level or flight-validated
accuracy.** Tiers earn `validated-toy` where a closed-form or code-to-code case
exists and `research` only where a public benchmark (RTKLIB, `42`, ARAIM
Milestone-3, Mars-2020 envelope) anchors it. The ceiling statement is repeated
per-WP in the backlog.

---

## 2. Current state in source

The current implementation is **further along than a naive "Tier 0" reading
suggests** — several lever-arm / anisotropy / latency features already exist —
so the parity work is targeted, not greenfield. Verified against source:

### 2.1 What already exists (the regress-against baseline)

- **IEEE-952 IMU with lever-arm and anisotropy.**
  `crates/openbmp-sensors/src/imu.rs` implements the five-component noise model
  (ARW, OU bias instability, RRW, scale factor, quantization) with per-component
  domain-separated RNG. It **already** carries:
  - the CG→sensor lever-arm transport `specific_force_at_mount` (imu.rs:380–391)
    applying the centripetal `ω×(ω×r)` and Euler `α×r` terms;
  - an anisotropic scale/misalignment matrix and a gyro **g-sensitivity** matrix
    (`with_deterministic_errors`, imu.rs:207–224; applied at imu.rs:421–423);
  - mount offset `mount_offset_body_m`.
  The gap is *not* the IMU error model — it is the **truth** the IMU consumes.
- **`SensorTruth` already carries the right fields.**
  `crates/openbmp-sensors/src/sensor.rs:32–59` exposes
  `specific_force_body_m_s2`, `angular_velocity_body_rad_s`, and
  `angular_acceleration_body_rad_s2` as first-class truth. The truth *struct* is
  force-based-ready; only the runner's *population* of it is not.
- **GNSS receiver-output model with clock and lever-arm.**
  `crates/openbmp-sensors/src/gnss.rs` is an ECI position/velocity oracle with
  additive Gaussian noise + an OU position-bias drift, **plus** a receiver-clock
  bias/drift random-walk projected onto radial/velocity directions
  (`step_clock`, gnss.rs:471–507), an antenna phase-center lever-arm
  (gnss.rs:368–384), and a fixed-latency one-sample hold with a delayed
  `capture_time` (gnss.rs:459–467). Its own docstring is explicit:
  "**No satellite geometry, no pseudorange, no ionosphere, no tropospheric
  model**" (gnss.rs:14–17). This is the receiver-output endpoint of the budget,
  not a measurement-domain receiver.
- **Star tracker, barometer, magnetometer.** `star_tracker.rs` is a
  small-angle quaternion perturbation (σ ≤ 0.01 rad gated); `barometer.rs` and
  `magnetometer.rs` are additive-noise + bias models. No QUEST/Wahba single-frame
  solve, no FOV occultation, no centroid/NEA model.
- **Measurement-domain SIL fault injection.**
  `crates/openbmp-sensors/src/stimulus.rs` implements `MeasurementStimulus::Bias`
  applied at the sensor-read → FC-publish boundary, forward-only by construction
  (`apply` takes only `(measurement, rng)`, never truth or estimate;
  stimulus.rs:34–47).
- **Effector dynamics already exist (first-order).**
  `crates/openbmp-vehicle/src/effector/mod.rs` + `linear.rs` ship a
  `ControlEffector` trait and a `LinearActuator`: first-order lag + rate clamp +
  position saturation + deadband + fixed-depth pure-delay buffer, with four
  canonical fault modes `Jam / Runaway / ReducedRate / Hardover`
  (effector/mod.rs:146–171). This is **first-order only** — no second-order
  bandwidth `(ωₙ, ζ)`, no acceleration limit, no backlash-on-reversal, no
  resonant load model, no hinge-moment coupling.
- **HAL contract surface.** `crates/openbmp-hal/src/lib.rs` defines marker
  traits `Imu`, `Gnss`, `Magnetometer`, `Barometer`, `StarTracker`,
  `TvcActuator`, `RcsValve`, `ThrottleCommand`, the pull-style `Sensor` and
  command-style `Actuator` traits, and the injected `Clock` trait (lib.rs:856).
  These are the portability anchors any new sensor/actuator model attaches to.

### 2.2 The one structural artifact to remove (the headline WP)

`crates/openbmp-runner/src/fc_bridge.rs:579–613` computes the "sensed" truth by
**finite-differencing simulator state**:

```rust
// specific_force_eci(): fc_bridge.rs:585–593
let total_accel = (velocity_eci_m_s - prev_v) / (time_s - prev_t);
...
total_accel - gravity_eci_m_s2

// angular_acceleration_body(): fc_bridge.rs:601–609
let angular_accel = (angular_velocity_body_rad_s - prev_omega) / (time_s - prev_t);
```

This conflates integrator truncation error and **gravity-model mismatch**
directly into the "sensed" specific force, has a one-sample lag, and produces a
zero gradient at `t₀` (the first step has no `prev`). It corrupts EKF tuning
conclusions because the filter is tuned against a "measurement" that contains
the simulator's own gravity error. Because `SensorTruth` already carries
`specific_force_body_m_s2` and `angular_acceleration_body_rad_s2`, the fix is a
**plumbing** change: source both from the EOM force/moment accumulator and the
rotational dynamics, not from re-differencing state.

### 2.3 Maturity summary

| Sub-dimension | Current | Target tier |
|---|---|---|
| Specific-force truth | velocity finite-difference − gravity (artifact) | T1 force-based |
| IMU error stack | IEEE-952 + lever-arm + anisotropy + g-sens | (already T1-grade) |
| High-rate strapdown | none (single-rate truth echo) | T2 delta-θ/delta-v + coning/sculling |
| GNSS | position/velocity oracle + clock + latency | T2 pseudorange / T3 carrier+RAIM/ARAIM |
| Star tracker | small-angle quaternion noise | T2 QUEST + FOV occultation |
| Altimeter / TRN | none | T2 altimeter / T4 TRN |
| TVC/fin servo | first-order lag + rate/deadband/delay | T1 2nd-order / T4 resonant + hinge |
| RCS / RW / CMG | direct-torque effectors only | T1 RCS MIB / T3 RW / T4 CMG |
| Air-data / FADS | none | T1 Pitot-static / T3 FADS |
| Redundancy / FDIR / time-tag | partial (per-sensor latency) | T1 time-tag / T3 voting+FDIR |

---

## 3. Target architecture

### 3.1 Crate map and placement

No new mandatory crate; the work extends three existing crates and proposes one
optional feature-gated crate for the Tier-4 vision path:

```text
openbmp-sensors   L3  + force-based truth port; high-rate strapdown emulator;
                       pseudorange/carrier GNSS; QUEST star tracker; altimeter;
                       air-data/FADS; redundancy/voting; time-tag plumbing
openbmp-vehicle   L2  + second-order / resonant TVC servo, RCS MIB+PWPF,
                       reaction wheel, CMG (extend effector/*)
openbmp-runner    L7  + force-accumulator surfacing into SensorTruth (kills the
                       finite-difference); redundant-string registration; DEM/
                       ephemeris ingestion wiring
openbmp-fc        L4  (consumer only) tight-nav, MEKF measurement updates, RAIM
                       residual logic, voter/FDIR — through the existing boundary
                       (PORTABILITY LOCK: no new edge to a forbidden crate)

NEW (optional, Tier-4 only; review boundary in the WP that introduces it):
  openbmp-visionnav L3  pinhole + PnP + DEM-relative measurement, behind a
                        `vision` feature flag; never links a GPL solver.
```

The **GNSS geometry / ephemeris** model is data-driven (RINEX/SP3 ingested with
provenance) and lives in `openbmp-sensors`; the **DEM** for altimeter/TRN is
ingested into `data/terrain/` with provenance. Neither adds an up-layer edge.

### 3.2 Force-based specific-force truth (T1, the headline)

**What an accelerometer senses** is the non-gravitational specific force at its
proof mass. The correct truth is sourced from the force accumulator:

```
f_cg = ( T_thrust + A_aero + F_separation + F_other ) / m          (3.2.1)
```

where the right-hand side is the *same* force sum the translational EOM
integrates — gravity is **excluded** because it acts on the proof mass and
support equally (the equivalence principle: a free-falling accelerometer reads
zero). Transport to the sensor station `r_b` (CG→sensor in body axes):

```
f_imu = f_cg + α × r_b + ω × (ω × r_b)                              (3.2.2)
```

with `ω` the body rate and `α` the body **angular acceleration taken directly
from the rotational dynamics** `α = J⁻¹(τ − ω × Jω)` — not a finite difference.
The IMU error transform (already in `imu.rs`) then applies:

```
f_meas = (I + M_a) f_imu + b_a + G_a·(gravity-comp residual) + n_a  (3.2.3)
ω_meas = (I + M_g) ω + b_g + G_g f_imu + n_g                        (3.2.4)
```

`M` = scale/misalignment, `b` = bias (OU+RRW), `G_g` = gyro g-sensitivity, `n`
= ARW/VRW white noise. The **plumbing rule**: the runner reads
`f_cg`, `ω`, `α` from the physics step and fills `SensorTruth`; the
finite-difference path in `fc_bridge.rs` is deleted (or kept only as an opt-in
`specific_force_source = "finite_difference"` legacy mode for golden continuity
— see §4 byte-stability).

> Cross-frame note: gravity for the *navigation* gravity-compensation residual
> must use the **same** gravity model the EKF uses, referenced symbolically
> (`GM_earth`, `J2`, `R_earth`) — never inline numerals (see `08-environment-
> gravity-and-frames.md`). The whole point of the rewrite is that the **truth**
> no longer leaks the simulator's gravity model into the measurement.

**Trait/struct surface (additive, in `openbmp-sensors`):**

```rust
/// Force-based specific-force truth, sourced from the EOM accumulator.
/// Filled by the runner from the physics step; never finite-differenced.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpecificForceTruth {
    /// Non-gravitational specific force at the vehicle CG, body frame (m/s²).
    pub f_cg_body_m_s2: Vector3<f64>,
    /// Body angular velocity (rad/s).
    pub omega_body_rad_s: Vector3<f64>,
    /// Body angular acceleration from rotational dynamics (rad/s²).
    pub alpha_body_rad_s2: Vector3<f64>,
}
```

This replaces the runner's need to call `specific_force_eci` /
`angular_acceleration_body`; `SensorTruth.specific_force_body_m_s2` and
`angular_acceleration_body_rad_s2` are filled from `SpecificForceTruth`. The IMU
`specific_force_at_mount` lever-arm code (already present) is unchanged.

### 3.3 High-rate strapdown: delta-theta / delta-v + coning/sculling (T2)

Real strapdown IMUs output **integrated increments** at a high rate; the nav
computer runs a two-speed update that rectifies coning (rotational vibration →
attitude drift) and sculling (cross-axis vibration → velocity drift). OpenBMP
emulates the sensor *and* provides a reference algorithm so a flight-SW nav can
be validated.

**Bortz rotation-vector ODE** (the exact attitude kinematics the increments
approximate):

```
φ̇ = ω + ½ (φ × ω) + (1/|φ|²)[1 − (|φ| sin|φ|)/(2(1−cos|φ|))] φ × (φ × ω)   (3.3.1)
```

linearized for the inner window to `φ̇ ≈ ω + ½(φ×ω) + (1/12) φ×(φ×ω)` (the last
two terms are the coning correction).

**Two-speed mechanization (Savage):**

- High-rate increments over sub-interval `i`:
  `Δθ_i = ∫ ω dt`, `Δv_i = ∫ f dt` (RK4 of truth `ω(t)`, `f(t)`, then quantize
  to LSB to emulate; or supply directly to validate a flight nav).
- Coning increment over the moderate window of `N` sub-intervals
  (two-sample form):
  `β = (2/3)(Δθ_{N−1} × Δθ_N)`; rotation vector `φ = α_N + β` with
  `α_N = Σ Δθ_i`.
- Attitude update: `q_{n+1} = q_n ⊗ q(φ)`, `q(φ) = [cos(|φ|/2); (φ/|φ|)sin(|φ|/2)]`.
- Sculling increment (Savage two-sample):
  `Δv_scul = ½ Σ_i [ (α_{i−1} × Δv_i) + (Δv_{i−1} × α_i) ]`.
- Velocity rotation compensation: `Δv_rot = ½ (α_N × Δv_N)`.
- Moderate-rate velocity/position update applies the body-frame
  `Δv = Δv_N + Δv_rot + Δv_scul` through the n-frame DCM plus gravity/Coriolis.

**Surface (in `openbmp-sensors`, behind `synthetic`):**

```rust
/// One high-rate inertial increment pair (the wire format of a strapdown IMU).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct InertialIncrement {
    pub delta_theta_rad: Vector3<f64>,   // integrated body rate
    pub delta_v_m_s: Vector3<f64>,       // integrated specific force
    pub dt_s: f64,                        // sub-interval length
    pub seq: u64,                         // high-rate sample index
}

/// Emulates the IMU as a stream of quantized increments at a high inner rate.
pub trait HighRateImu {
    fn integrate_window(
        &mut self,
        truth: &SpecificForceTruth,
        sub_samples: u32,           // inner samples per FC tick
        step: StepIndex,
        scenario_seed: u64,
    ) -> Result<heapless::Vec<InertialIncrement, MAX_SUB>, SensorError>;
}

/// Coning/sculling reference: validates a flight nav OR drives the truth nav.
pub fn coning_sculling_update(
    q_in: UnitQuaternion<f64>,
    increments: &[InertialIncrement],
    algo: ConingScullingAlgo,        // TwoSample | ThreeSample | FourSample
) -> (UnitQuaternion<f64>, Vector3<f64> /*Δv_nav*/);
```

The inner integrator is RK4 with **locked operand order** and no FMA;
quantization is `round(x/lsb)*lsb` exactly as the existing IMU axis path.

### 3.4 Pseudorange / Doppler / carrier GNSS + atmosphere + integrity (T2/T3)

Replace the position oracle with a measurement-domain receiver.

**Pseudorange:**

```
ρ_i = ‖r_sv,i − r_rx‖ + c(δt_rx − δt_sv,i) + I_i + T_i + M_i + ε_i   (3.4.1)
```

SV position `r_sv` from broadcast ephemeris (Keplerian elements + perturbation
harmonics per IS-GPS-200) or precise SP3. **Doppler / range-rate:**

```
ρ̇_i = (v_sv − v_rx)·ê_i + c(δṫ_rx − δṫ_sv,i) + ε̇_i                  (3.4.2)
```

**Atmosphere:**
- Ionosphere `I_i`: Klobuchar (8 broadcast coefficients) or NeQuick-G; or
  eliminate via the dual-frequency iono-free combination
  `ρ_IF = (f₁²ρ₁ − f₂²ρ₂)/(f₁² − f₂²)`.
- Troposphere `T_i`: Saastamoinen zenith hydrostatic + wet delay × elevation
  mapping (GMF/Niell), using the same standard atmosphere as `08`.
- Multipath `M_i`: elevation-dependent first-order Gauss-Markov per channel,
  `σ_MP(elev) = a + b·exp(−elev/elev₀)`.

**Carrier phase (T3):**

```
λ φ_i = ‖r_sv,i − r_rx‖ + c(δt_rx − δt_sv,i) − I_i + T_i + λ N_i + m_i   (3.4.3)
```

with integer ambiguity `N_i` and discrete cycle-slip events. Integer resolution
via LAMBDA (decorrelation + integer least-squares search).

**Navigation solution (WLS):**

```
δx = (Hᵀ W H)⁻¹ Hᵀ W δz,    W = diag(1/σ_i²)                         (3.4.4)
```

with geometry matrix `H` rows `[−êᵢᵀ, 1]`; DOP from `(HᵀH)⁻¹`.

**Integrity — snapshot RAIM (T3):**

```
w = z − H x̂,   SSE = wᵀ W w,   detect if SSE > T_χ²(α, n−4)          (3.4.5)
HPL = slope_max · √λ_threshold
```

**ARAIM / MHSS (T3):** form full solution `x₀` and `N` fault-hypothesis subset
solutions `x_k` (drop one SV each); test statistic `|x₀ − x_k|`;
`VPL/HPL = max over hypotheses (separation + K_md·σ_ss)`.

**Spoofing/jamming injection (T3, SIL stimulus):** coherent per-channel offset
(spoof) and `σ_i` inflation / SV drop below a C/N₀ floor (jam) — applied through
the **existing measurement-domain stimulus seam** (`stimulus.rs`), never reading
truth or estimate.

**Surface:**

```rust
pub struct GnssGeometry {
    pub sv: heapless::Vec<SvState, MAX_SV>,   // r_sv, v_sv, clock, health, C/N0
    pub rx_ecef: Vector3<f64>,
    pub mask_elev_rad: f64,
}
pub struct PseudorangeBudget {
    pub iono: IonoModel,        // Klobuchar | NeQuickG | IonoFreeDualFreq
    pub tropo: TropoModel,      // Saastamoinen + mapping
    pub multipath: GaussMarkovPerChannel,
    pub receiver_noise_m: f64,
}
pub trait GnssReceiver {
    fn pseudoranges(&self, geom: &GnssGeometry, step: StepIndex, seed: u64)
        -> Result<heapless::Vec<RawMeasurement, MAX_SV>, SensorError>;
}
pub struct RaimResult { pub sse: f64, pub detected: bool, pub excluded_sv: Option<SvId>, pub hpl_m: f64, pub vpl_m: f64 }
```

The **WLS/RAIM/ARAIM consumer** lives FC-side (or in a portable nav crate) so
the FC portability lock holds; the *generator* lives sim-side in
`openbmp-sensors`.

### 3.5 Star tracker (QUEST/Wahba) + sun/horizon sensors + MEKF (T2)

**Wahba problem:** `J(A) = ½ Σ wᵢ ‖bᵢ − A rᵢ‖²`. **QUEST** builds
`B = Σ wᵢ bᵢ rᵢᵀ`, `S = B + Bᵀ`, `σ = tr B`, `Z = [B₂₃−B₃₂, B₃₁−B₁₃, B₁₂−B₂₁]ᵀ`,
the Davenport K-matrix `K = [[S − σI, Z],[Zᵀ, σ]]`, and returns the eigenvector
of the maximum eigenvalue (Newton-iterated from the characteristic equation).
Star-tracker noise: `q_meas = q_true ⊗ δq(θ)`, `θ ~ N(0, R)`,
`R = diag(σ_xb², σ_yb², σ_boresight²)` (about-boresight ≈ 10× cross-boresight),
with centroid NEA ≈ `PSF_width / (SNR·√N_pix)` (Lindegren bound). FOV occultation
excludes stars within the Sun/Earth/Moon exclusion cones. The MEKF error state
(3-axis attitude error + gyro bias) propagates on delta-theta and updates
multiplicatively on the quaternion/sun-vector residual; OpenBMP already has an
SR-UKF (`crates/openbmp-fc/src/sr_ukf.rs`) the measurement can attach to.

### 3.6 Air-data / FADS (T1/T3)

Compressible Pitot (subsonic): `q_c = p_s[(1 + 0.2 M²)^{3.5} − 1]`, Rayleigh
supersonic; invert for `M` from `q_c/p_s`; CAS from `q_c` via the standard
atmosphere; altitude from `p_s` via the **same** atmosphere as `08`. Vanes:
`α_meas = K_upwash·α_true + bias + n`. **FADS** (T3): nonlinear least-squares fit
of `(α, β, M, p_∞)` to a pressure-port array `Cp_i(α, β, M)` (Whitmore triples
algorithm).

### 3.7 Relative / vision nav (T2 altimeter, T4 TRN — dual-use-adjacent)

- **Radar/laser altimeter:** `h_meas = h_true/cos(tilt) + bias + n` with a DEM
  lookup at the beam-footprint centroid and a footprint-averaging kernel; dropout
  below min-range / above max-range. Laser adds speckle, smaller σ.
- **Doppler velocimeter:** per-beam `ṙ_j = −ê_j·v_body + n`; invert ≥3 beams for
  `v_body`.
- **Terrain-relative nav (T4):** known landmarks `p_w` project through the
  pinhole `u = K[R|t]p_w`; matched pixel `u_meas = u + n_pix`; the estimator
  inverts via PnP/homography for a **map-relative POSITION FIX**
  `z = pos_in_map + N(0, R_map(altitude))`, `R` shrinking with altitude. **This
  is a navigation measurement only** — see the dual-use note (§6). Behind the
  optional `openbmp-visionnav` crate / `vision` feature.

### 3.8 Actuator dynamics

**TVC / fin servo (T1 second-order; T4 resonant + hinge):**
Linear core `G(s) = ωₙ²/(s² + 2ζωₙ s + ωₙ²)` (launcher TVC: ωₙ ≈ 10–30 Hz,
ζ ≈ 0.5–0.7), discretized (A,B,C,D) at the FC rate via ZOH. Nonlinearities are
applied **in series, in this order**: command → rate limit `|δ̇| ≤ δ̇_max` →
acceleration limit `|δ̈| ≤ δ̈_max` → backlash (deadband `2b` engaged on direction
reversal) → bandwidth filter → position/rate saturation. EMA torque limit:
`T_motor = K_t·I`, current-limited, reflected through gear ratio `N` and
roller-screw lead. The resonant (T4) load is a two-mass-spring: actuator drives
engine mass `m_e` through compliance `k`, hinge inertia `J`, load torque
`τ_load = q_dyn S_ref c_h(α, δ)` with the hinge-moment coefficient from the
`03` aero deck. Faults: hardover (`δ → ±δ_max` for `t > t_fault`), stuck, and
oscillatory (injected sinusoid) — extending the existing `EffectorFault` enum.

**RCS (T1 MIB + PWPF):** commanded torque → control allocation
`T_cmd = A u, u ≥ 0` (LP/pseudoinverse, fuel-minimizing). PWPF modulator: a
Schmitt trigger (deadband `d`, hysteresis `h`) on a first-order pre-filter
`K_m/(τ_m s + 1)` → on/off pulse train; minimum pulse = `MIB/F_nominal`. Per-pulse
thrust profile: rise transient + steady `F` + tail-off, `impulse = ∫F dt` with a
hard MIB floor (impulse-vs-on-time is linear only above MIB). Blowdown:
`F = F(P_tank)`, `P_tank` decreasing with expelled mass (iso/adiabatic gas law).

**Reaction wheel (T3):** `Ḣ = −τ_cmd` clamped to `τ_max`, `|H| ≤ H_sat`; motor +
viscous + Coulomb friction; zero-speed stiction. **CMG (T4):**
`τ = −A(δ) δ̇`, `A = ∂H/∂δ` (3×N Jacobian); singularity-robust steering
`δ̇ = Aᵀ(AAᵀ + λI)⁻¹ τ_cmd` with null-motion escape near `det(AAᵀ) → 0`.

**Surface (extend `openbmp-vehicle/src/effector`):**

```rust
/// Second-order servo with rate+accel limit, backlash, saturation.
pub struct SecondOrderServo {
    wn_rad_s: f64, zeta: f64,
    rate_max_per_s: f64, accel_max_per_s2: f64,
    backlash_half_width: f64,
    limits: EffectorLimits,
    state: ServoState,   // position, velocity, last-direction (for backlash)
}
impl ControlEffector for SecondOrderServo { /* step(cmd, dt) -> EffectorState */ }

/// Reaction-control thruster bank: MIB + PWPF + blowdown.
pub struct RcsBank {
    geometry: ThrusterGeometry,   // positions + thrust axes (for allocation A)
    pwpf: PwpfParams,             // K_m, tau_m, deadband, hysteresis
    mib_n_s: f64,                 // minimum impulse bit
    blowdown: BlowdownCurve,
}

pub struct ReactionWheel { h_sat_nms: f64, tau_max_nm: f64, friction: WheelFriction }
pub struct Cmg { jacobian_fn: /* A(δ) */, lambda_robust: f64 }
```

All effectors stay **scalar-or-vector deterministic**, no FMA, no RNG except via
`DeterministicRng::for_effector_component` (`b"EFFC"` domain tag) for stochastic
faults — matching the existing `LinearActuator` contract.

### 3.9 Time-tagging, latency, redundancy, cross-strapping (T1/T3)

Every measurement carries a valid-time `t_valid = t_sample − latency_i`
(`Timestamped<T>` already exists, sensor.rs:135–149; GNSS `capture_time`
demonstrates the pattern). The FC must propagate state to `t_valid` before the
update (Schmidt out-of-sequence handling). **Redundancy:** `N` strings each with
independent error draws; a voter `V = mid_value_select(s₁,s₂,s₃)` or weighted
average with fault flags; cross-strapping routes sensor `i` to FC channel `j`.
**FDIR:** per-sensor residual χ² + persistence counter → declare faulty → exclude
/ swap to a redundant string. The voter/FDIR logic is FC-side; the multi-string
*generation* is sim-side.

### 3.10 Fidelity tiers

| Tier | Scope | Earns |
|---|---|---|
| **T0** (current) | velocity-finite-difference specific force − gravity; IEEE-952 IMU (already lever-arm/anisotropy); GNSS position oracle + clock + latency; small-angle star tracker; first-order effector | `checked` |
| **T1** | force-based specific-force truth (kill the finite-difference); 2nd-order TVC/fin servo (rate/accel/backlash/hardover); RCS MIB + PWM on existing effectors; Pitot-static air-data + vanes; measurement time-tag everywhere | `validated-toy` |
| **T2** | high-rate delta-θ/delta-v + coning/sculling; pseudorange/Doppler GNSS (Klobuchar/Saastamoinen/multipath/WLS/DOP); QUEST star tracker + FOV occultation; radar/laser altimeter + Doppler velocimeter | `validated-toy`; GNSS `research` (code-to-code RTKLIB) |
| **T3** | snapshot RAIM + ARAIM/MHSS; carrier phase + integer ambiguity + cycle slips; spoof/jam injection; N-string redundancy + 2-of-3 voting + cross-strapping + FDIR; reaction-wheel friction/saturation; FADS least-squares | `research` (ARAIM Milestone-3); rest `validated-toy` |
| **T4** | terrain-relative/vision nav (pinhole + PnP vs synthetic DEM); CMG singularity-robust steering; 4th/6th-order resonant TVC + hinge-moment coupling to the `03` aero deck (aeroservoelastic margins) | `research` where Mars-2020 envelope / `42` anchors; else `validated-toy` |

---

## 4. Invariant preservation

- **Byte-determinism.** All new arithmetic is pure `f64` with **locked operand
  order**, no `f64::mul_add`, no wall-clock, no system RNG, no unordered
  iteration. New randomness (multipath Gauss-Markov, carrier-phase noise,
  cycle-slip events, centroid NEA, stochastic actuator faults) draws only from
  `DeterministicRng` keyed by `(seed, step, sensor_id/effector_id, component_id)`
  with reserved component slots (the IMU/GNSS modules already demonstrate the
  reserved-slot pattern so new sub-components never shift existing streams). The
  high-rate strapdown inner loop is keyed by `(…, sub_sample_index)` so its
  stream is independent of the FC-tick stream. Cross-libm bit-equality is **not**
  claimed for `sin/cos/ln/sqrt`-bearing paths (QUEST eigensolve, coning
  trig) — same caveat the existing crate already documents (lib.rs:22–26); the
  byte-diff gate runs on the reference profile.
- **Byte-stable-by-default.** Every model is off until a scenario opts in. The
  force-based truth rewrite is the sensitive case: it **changes** the IMU
  "measurement" for existing scenarios, so it ships behind
  `[sensors.imu] specific_force_source = "force_accumulator" | "finite_difference"`
  with `finite_difference` as the *default* until each golden scenario migrates,
  then the default flips in a separate, reviewed PR. Pseudorange GNSS, RAIM,
  servos, RCS-MIB, altimeter, FADS, redundancy, TRN are each gated by their own
  scenario block; existing goldens stay byte-identical.
- **FC hardware-portability lock.** All sim-side *generation* (truth port,
  pseudorange synthesis, increment emulation, DEM lookup, multi-string draws,
  actuator plant) lives in `openbmp-sensors` / `openbmp-vehicle` /
  `openbmp-runner`. The FC consumes only through the existing
  `Sensor`/`SyntheticSensorAdapter` and effector-command boundary; nav consumers
  (WLS, RAIM, voter, MEKF update) that live in `openbmp-fc` depend only on
  hardware-portable crates. `fc_dependency_tripwire.rs` must stay green: **no new
  `openbmp-fc` edge** to `openbmp-sim/-runner/-scenario/-telemetry/-bridge`. The
  optional `openbmp-visionnav` crate is L3 (sim-side); the FC sees only the
  resulting position-fix measurement.
- **Lockstep-clock & no-hot-path-allocation.** Increment buffers and SV/string
  tables use fixed-capacity `heapless::Vec`/arrays; the per-tick path does not
  allocate. Time is read only through the injected `Clock`
  (`openbmp-hal::Clock`); `Instant::now`/`SystemTime::now` stay banned.
  `fc_lints.rs` tests must stay green.
- **Four-pillar provenance.** RINEX broadcast ephemeris, SP3, star catalog
  (Hipparcos/Tycho-2 subset), and DEM tiles land under `data/<thing>/` with a
  sibling `provenance.md`, SHA-256 pin, license note, and validation label;
  tolerance-table TOMLs live under `tests/expected/`. **No inline data fixtures
  in `*.rs`.** **No literal WGS84 GM_earth / J2 / R_earth numerals anywhere in
  this doc or code** — symbolic only. `inline_data_tripwire.rs` and `openbmp
  check-provenance` stay green; new source-of-truth paths are added to the
  allow-list in the same PR.
- **Validation labels.** Each tier declares exactly one of `experimental →
  checked → validated-toy → research` per §3.10, justified by the V&V evidence in
  §5. No artifact claims `flight-qualified`/`certified`/`operational`.
- **Forward-only locks tighten.** The measurement-domain stimulus stays
  forward-only (`apply(measurement, rng)`, no truth/estimate). The spoof/jam and
  TRN WPs each add a compile-fail/tripwire test proving no ground-aimpoint surface
  is introduced (§6). `ballistic_state_compile_fail.rs` stays green.

---

## 5. V&V plan

The five-layer ladder (`docs/verification.md`) applies: MMS code-verification →
model-verification vs correlations → code-to-code vs open solvers → public
benchmarks → UQ reporting.

### 5.1 Analytic / closed-form (machine-precision)

| Case | Setup | Tolerance | Tier / label |
|---|---|---|---|
| Stationary-on-rotating-Earth | fixed sensor at known lat/lon reads `f = −g + centrifugal`, gyro = Earth-rate `ω_ie` projected to local frame (Earth rate ≈ 15.041 °/hr) | accel `< 1e-9 m/s²`; gyro `< 1e-12 rad/s` (vs closed form) | T1 `validated-toy` |
| Pure-thrust specific force | constant thrust `T`, mass `m`, no aero → `f = T/m` exactly, zero gravity leakage | `< 1e-12 m/s²` (vs `T/m`) | T1 `validated-toy` |
| Lever-arm | spin-up `(ω, α)` at offset `r` → `f = α×r + ω×(ω×r)` | `< 1e-12` (already tested, imu.rs:572) | T1 `validated-toy` |
| Coning motion | sinusoidal coning input → exact attitude drift (Savage/Ignagni) | drift residual `< 1e-10 rad` | T2 `validated-toy` |
| Sculling motion | cross-modulated Δθ/Δv → exact velocity drift | residual `< 1e-9 m/s` | T2 `validated-toy` |
| Servo step/frequency response | 2nd-order `G(s)` step & sine | settling/overshoot/phase match `G(s)` to `< 0.5%`; MMS on the ODE | T1 `validated-toy` |
| Rate-limit/backlash limit cycle | describing-function prediction | limit-cycle amplitude/freq within `< 5%` | T1 `validated-toy` |
| QUEST optimality | synthetic catalog + known attitude + arcsec noise | recovered quaternion error matches predicted NEA covariance; QUEST vs SVD/Davenport-q agree `< 1e-10` | T2 `validated-toy` |
| RCS MIB | commanded impulse below MIB → zero or floored pulse | exact MIB floor; impulse linear above MIB | T1 `validated-toy` |

### 5.2 Behavioral / model-verification

| Case | Tolerance | Tier |
|---|---|---|
| Schuler-period free-INS error growth (~84.4 min oscillation) | period within `< 1%` | T2 |
| GNSS WLS RMS vs known geometry (DOP-scaled) | position RMS within `< 5%` of `√(GDOP²·σ²)` | T2 |
| Klobuchar iono / Saastamoinen tropo vs reference outputs | delay match `< 1%` | T2 |
| Multipath empirical σ vs budget over long run | `< 5%` (mirrors existing GNSS noise test, gnss.rs:778) | T2 |

### 5.3 Code-to-code (`research`-anchoring)

| Case | Oracle | License posture |
|---|---|---|
| Pseudorange + WLS vs same RINEX ephemeris + truth trajectory | **RTKLIB** (BSD-2; port or FFI) | port/ingest |
| Pseudorange generator cross-check | **GPS-SDR-SIM** (MIT) | ingest as reference |
| Independent positioning engine | **gnss-sdr** (GPLv3) | **external process only**, never vendor/link |
| Actuator + RW/CMG/thruster on identical maneuver | **NASA `42`** (NOSA) | external reference |
| IMU/GNSS error-stack methodology | **gnss-ins-sim** (MIT), **allan_variance** (MIT) | port/ingest |

### 5.4 Public-benchmark (`research`)

| Case | Benchmark | Note |
|---|---|---|
| RAIM/ARAIM fault detection/exclusion + protection-level bounding | ARAIM Milestone-3 reference scenarios | inject single/dual SV ramp/step bias; confirm detect → exclude → HPL/VPL bound true error |
| TRN error-vs-altitude envelope | Mars-2020 LVS published envelope (~5 m from 4200 m) | **order-of-magnitude** target only; published, not held flight data |

Each tier earns the label in the §3.10 table **only** when its row's evidence is
in place with a tolerance-table TOML; absent the oracle, the tier caps at
`validated-toy` and says so in the PR.

---

## 7. Dependencies on other parity docs

- **`05-propulsion-high-fidelity.md`** — the force-based specific-force rewrite
  (T1) consumes the propulsion force from the EOM accumulator (thrust =
  `ṁ Vₑ + (pₑ−pₐ)Aₑ`); the RCS blowdown couples to the feed-system tank-pressure
  state. *Hard dependency for T1.*
- **`08-environment-gravity-and-frames.md`** — the gravity model used in
  navigation gravity-compensation, the standard atmosphere for air-data/Pitot
  inversion and tropospheric mapping, and the ECEF↔geodetic/local-frame
  primitives for GNSS geometry, altimeter, and the stationary-Earth V&V case.
  Symbolic constants only.
- **`03-aerodynamics-database-and-cfd-coupling.md`** — the hinge-moment
  coefficient `c_h(α, δ)` for the resonant-load TVC (T4) and the dynamic pressure
  for fin hinge-moment and FADS port pressures.
- **`06-gnc-coupled-mimo-and-control.md`** — the consumer of all of this: the
  MEKF/SR-UKF measurement updates, the control allocation feeding RCS/CMG, the
  load-relief that needs honest specific-force truth, and the tight-nav GNSS gate
  (the dead-reckon blocker this dimension exists to exercise honestly).
- **`02-structural-dynamics-loads-slosh-pogo.md`** — the resonant TVC load model
  (T4) couples to engine-body/aeroservoelastic modes; the slosh-RCS coast closure
  consumes the RCS MIB model.
- **`10-flight-software-in-the-loop-xil.md`** — the time-tag/latency and
  fault-injection seams are the SIL stimulus surface; spoof/jam and FDIR live in
  the XIL fault library.
- **`11-monte-carlo-uq-and-validation.md`** — per-entry sensor/actuator error
  margins (ARW, bias, multipath σ, servo bandwidth, MIB) flow into MC dispersions
  and the 7009B credibility record.
- **`12-determinism-realtime-and-compute.md`** — the high-rate inner loop and
  multi-string draws must hold byte-determinism under the parallel-MC and
  real-time-pacing substrate.

---

## 8. Open-source leverage

| Tool | License | Use mode | What |
|---|---|---|---|
| **RTKLIB** (+demo5/b34) | BSD-2-Clause | **port / ingest** | SV-position-from-ephemeris, Klobuchar iono, Saastamoinen tropo, WLS, snapshot RAIM, LAMBDA; RINEX/SP3/RTCM parsers as the data-ingest spec. Re-implement in Rust (BSD-2 also permits vendoring C + FFI). |
| **GPS-SDR-SIM** | MIT | **ingest** | RINEX-ephemeris + trajectory → per-epoch pseudoranges as a code-to-code cross-check oracle. |
| **gnss-sdr** | **GPLv3** | **external process only** | independent positioning engine to cross-validate — **never link/port/vendor**. |
| **GeographicLib** | MIT | **couple / port** | ECEF↔geodetic, local frames, geoid for the altimeter; aligned with OpenBMP's WGS84 work (symbolic constants). |
| **Hipparcos / Tycho-2 / Gaia DR3** (mag<6 subset) | PD / CC-BY | **ingest** | star-tracker reference catalog for QUEST + catalog-match V&V. Attribute. |
| **USGS/PDS DEMs (MOLA, HiRISE), SRTM** | Public domain | **ingest** | terrain reference for altimeter footprint + TRN ground truth. |
| **OpenCV** | Apache-2.0 | **couple (optional)** | pinhole + `solvePnP` for Tier-4 vision behind a `vision` feature; or port just the pinhole+PnP math to avoid the dep. |
| **NASA `42`** | NOSA | **reference / code-to-code** | actuator + RW/CMG/thruster cross-validation on identical maneuvers. |
| **`allan_variance` / `gnss-ins-sim`** | MIT | **port / ingest** | Allan-deviation fitting of IEEE-952 coefficients; IMU/GNSS error-stack code-to-code reference. |
| **basic-airdata / COESA-1976 tables** | MIT/Apache-2.0 | **reuse / port** | 1976 standard-atmosphere relations for Pitot-static Mach/CAS/altitude inversion (shared with `08`). |

License hygiene is load-bearing: GPLv3 (`gnss-sdr`) is **external comparison
only**; everything ported/vendored is permissive (BSD-2/MIT/Apache-2.0/NOSA).
`cargo deny` enforces this.

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-09.1 — Force-based specific-force truth (kill the finite-difference)

- **title:** Source specific force and angular acceleration from the EOM accumulator.
- **goal:** Remove the structural artifact at `fc_bridge.rs:579–613` where
  specific force is `(Δv_eci/Δt − g)` and angular accel is `Δω/Δt`. Fill
  `SensorTruth.specific_force_body_m_s2` and `angular_acceleration_body_rad_s2`
  from `f_cg = ΣF_nongrav/m` and `α = J⁻¹(τ − ω×Jω)`. Eliminates gravity-model
  leakage into the "sensed" measurement and the `t₀` zero-gradient. Headline
  parity item; unblocks honest EKF tuning.
- **fidelity_tier:** T1
- **depends_on:** [`WP-05.1` (force/thrust accumulator, doc 05)]
- **new_crates:** none (extend `openbmp-sensors` + `openbmp-runner`).
- **touched:** `openbmp-sensors/src/sensor.rs` (add `SpecificForceTruth`);
  `openbmp-runner/src/fc_bridge.rs` (delete finite-difference, fill from
  accumulator); scenario schema (`[sensors.imu] specific_force_source`).
- **approach:** §3.2 (eqs 3.2.1–3.2.4). Add `specific_force_source =
  "finite_difference" (default) | "force_accumulator"`; new path off by default.
- **acceptance:**
  - new path off by default; canonical goldens byte-identical.
  - pure-thrust case: `f = T/m` to `< 1e-12 m/s²` (tolerance table).
  - stationary-on-rotating-Earth: `f = −g + centrifugal`, gyro = `ω_ie` to
    closed form (`< 1e-9 m/s²`, `< 1e-12 rad/s`).
  - opt-in scenario exercises the model end-to-end; all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line (truth-fidelity fix).
- **est_effort:** 1–2 weeks.
- **parity_ceiling:** does not validate a specific unit's error coefficients.

### WP-09.2 — Second-order TVC/fin servo + RCS MIB

- **title:** Add 2nd-order servo (rate/accel/backlash/hardover) and RCS minimum-impulse-bit.
- **goal:** Replace the first-order `LinearActuator` (for opting scenarios) with
  a finite-bandwidth `G(s) = ωₙ²/(s²+2ζωₙs+ωₙ²)` servo including accel limit,
  backlash-on-reversal, and hardover/stuck/oscillatory faults; add RCS MIB + PWM
  quantization on the direct-torque effectors. Directly relevant to the
  roll-authority/TVC blocker.
- **fidelity_tier:** T1
- **depends_on:** [WP-09.1]
- **new_crates:** none (extend `openbmp-vehicle/src/effector`).
- **touched:** `effector/mod.rs`, `effector/second_order.rs` (new),
  `effector/rcs.rs` (new); scenario `[vehicle.effector.servo]`.
- **approach:** §3.8 (series order: rate → accel → backlash → bandwidth →
  saturation). ZOH discretize at FC rate. Extend `EffectorFault` with
  `Oscillatory`.
- **acceptance:**
  - off by default; goldens byte-identical.
  - step/frequency response matches `G(s)` to `< 0.5%`; MMS on the servo ODE.
  - rate-limit/backlash limit cycle within `< 5%` of describing-function.
  - RCS MIB floor exact; impulse linear above MIB.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** vehicle-intrinsic; objectives stay attitude/rate.
- **est_effort:** 2–3 weeks.
- **parity_ceiling:** vendor transfer functions/friction maps not validated.

### WP-09.3 — Pitot-static air-data + measurement time-tag plumbing

- **title:** Add Pitot-static/vane air-data and universal measurement time-tagging.
- **goal:** Compressible Pitot Mach/CAS/altitude inversion + α/β vanes; and make
  `t_valid = t_sample − latency_i` first-class on every sensor (it already exists
  for GNSS). Time-tag is a first-order EKF error source and should land early.
- **fidelity_tier:** T1
- **depends_on:** [WP-09.1]
- **new_crates:** none.
- **touched:** `openbmp-sensors/src/airdata.rs` (new); `sensor.rs` (latency on
  all sensors via `capture_time`); `openbmp-runner` wiring.
- **approach:** §3.6, §3.9. Reuse the `08` standard atmosphere.
- **acceptance:**
  - off by default; goldens byte-identical.
  - Mach from `q_c/p_s` matches closed-form `< 1e-9`; altitude from `p_s` matches
    the atmosphere model.
  - latency-delayed `capture_time` verified for a non-GNSS sensor (mirrors
    gnss.rs:719).
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks.
- **parity_ceiling:** real boom-bending/sidewash not validated.

### WP-09.4 — High-rate strapdown (delta-θ/delta-v + coning/sculling)

- **title:** Emulate the IMU as quantized high-rate increments with two-speed coning/sculling.
- **goal:** Inner 1–2 kHz integrator emitting `(Δθ, Δv)` with LSB quantization;
  reference two-speed Bortz/Savage coning+sculling update to emulate the sensor
  and validate a flight nav.
- **fidelity_tier:** T2
- **depends_on:** [WP-09.1]
- **new_crates:** none.
- **touched:** `openbmp-sensors/src/strapdown.rs` (new); `imu.rs` (increment
  path); scenario `[sensors.imu.high_rate]`.
- **approach:** §3.3 (eqs 3.3.1 + two/three/four-sample). RK4 inner loop, locked
  operand order, sub-sample-keyed RNG.
- **acceptance:**
  - off by default; goldens byte-identical.
  - closed-form coning drift residual `< 1e-10 rad`; sculling `< 1e-9 m/s`.
  - Schuler-period free-INS growth within `< 1%`.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 3–5 weeks.
- **parity_ceiling:** frame bookkeeping verified vs Savage reference; no vendor
  algorithm equivalence claimed.

### WP-09.5 — Pseudorange/Doppler GNSS + iono/tropo/multipath + WLS/DOP

- **title:** Replace the GNSS oracle with a measurement-domain pseudorange receiver.
- **goal:** SV geometry from ingested RINEX broadcast ephemeris; pseudorange +
  Doppler with Klobuchar iono, Saastamoinen tropo, elevation-dependent
  Gauss-Markov multipath, receiver noise; WLS solution + DOP exposed.
- **fidelity_tier:** T2
- **depends_on:** [WP-09.1, WP-09.3]
- **new_crates:** none (data: `data/gnss/` RINEX/SP3 with provenance).
- **touched:** `openbmp-sensors/src/gnss/` (geometry, pseudorange, atmosphere,
  wls); RINEX/SP3 parser; scenario `[sensors.gnss.pseudorange]`.
- **approach:** §3.4 (eqs 3.4.1–3.4.4). Port RTKLIB algorithms; ingest RINEX as
  the format spec.
- **acceptance:**
  - off by default; goldens byte-identical.
  - **code-to-code vs RTKLIB** on the same RINEX + truth trajectory: pseudorange
    and WLS position agree within model tolerance (tolerance table); iono/tropo
    deltas match Klobuchar/Saastamoinen reference.
  - WLS RMS within `< 5%` of `√(GDOP²σ²)`.
  - RINEX/SP3 ingested with `provenance.md` + SHA pin + license note.
  - all §2 gates green.
- **validation_label:** `research` (RTKLIB code-to-code)
- **dual_use_note:** far from line (navigation robustness).
- **est_effort:** 4–6 weeks.
- **parity_ceiling:** real site multipath/scintillation/antenna patterns not
  validated.

### WP-09.6 — Star tracker QUEST/Wahba + FOV occultation

- **title:** Add single-frame QUEST attitude + centroid NEA + Sun/Earth/Moon exclusion.
- **goal:** Promote the small-angle star tracker to a Wahba/QUEST single-frame
  solve from a magnitude-limited catalog with NEA covariance and FOV occultation;
  attach to the existing SR-UKF as an MEKF-style update.
- **fidelity_tier:** T2
- **depends_on:** [WP-09.1]
- **new_crates:** none (data: `data/star_catalog/` Hipparcos subset with
  provenance).
- **touched:** `openbmp-sensors/src/star_tracker.rs`; `openbmp-fc/src/sr_ukf.rs`
  (measurement); scenario `[sensors.star_tracker.quest]`.
- **approach:** §3.5 (Davenport K-matrix eigensolve; Lindegren NEA).
- **acceptance:**
  - off by default; goldens byte-identical.
  - QUEST vs SVD/Davenport-q agree `< 1e-10`; recovered error matches predicted
    NEA covariance.
  - FOV occultation excludes stars in exclusion cones (unit test).
  - catalog subset ingested with provenance + attribution.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 3–5 weeks.
- **parity_ceiling:** real PSF/catalog-match performance not validated.

### WP-09.7 — Radar/laser altimeter + Doppler velocimeter (DEM lookup)

- **title:** Add nadir altimeter and Doppler velocimeter with DEM footprint lookup.
- **goal:** Descent/landing aids: `h_meas = h_true/cos(tilt) + bias + n` with DEM
  footprint averaging + dropout; ≥3-beam Doppler velocimeter inverting for
  `v_body`. Closes the powered-descent sensing loop with `07`.
- **fidelity_tier:** T2
- **depends_on:** [WP-09.1]
- **new_crates:** none (data: `data/terrain/` DEM tile with provenance).
- **touched:** `openbmp-sensors/src/altimeter.rs`, `velocimeter.rs` (new);
  scenario `[sensors.altimeter]`.
- **approach:** §3.7. Drive V&V against a synthetic DEM with exact truth.
- **acceptance:**
  - off by default; goldens byte-identical.
  - altimeter range vs synthetic-DEM truth within model tolerance; dropout bands
    exercised.
  - velocimeter recovers `v_body` from ≥3 beams to `< 1e-9 m/s` (no-noise).
  - DEM tile ingested with provenance.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** navigation measurement only (no ground-aimpoint).
- **est_effort:** 1–2 weeks.
- **parity_ceiling:** real surface backscatter/slope statistics not validated.

### WP-09.8 — RAIM + ARAIM/MHSS + carrier phase + spoof/jam

- **title:** Add GNSS integrity, carrier-phase ambiguity, and spoof/jam stimulus.
- **goal:** Snapshot RAIM + ARAIM/MHSS fault detection/exclusion + protection
  levels; carrier phase with integer ambiguity (LAMBDA) and cycle slips;
  spoof/jam injection through the forward-only stimulus seam.
- **fidelity_tier:** T3
- **depends_on:** [WP-09.5]
- **new_crates:** none.
- **touched:** `openbmp-sensors/src/gnss/{raim,araim,carrier}.rs` (new);
  `stimulus.rs` (spoof/jam variants); scenario `[sensors.gnss.integrity]`.
- **approach:** §3.4 (eqs 3.4.3, 3.4.5; MHSS). Spoof = coherent per-channel
  offset; jam = σ-inflation/SV-drop below C/N₀ floor — both `apply(measurement,
  rng)`-only.
- **acceptance:**
  - off by default; goldens byte-identical.
  - **ARAIM Milestone-3** scenarios: inject single/dual SV ramp/step bias →
    detection statistic crosses threshold, correct SV excluded, HPL/VPL bound the
    true error.
  - new spoof/jam stimulus has a tripwire test proving no truth/estimate input
    and no ground-aimpoint field.
  - all §2 gates green.
- **validation_label:** `research` (ARAIM Milestone-3)
- **dual_use_note:** spoof/jam gated as SIL FDIR stimulus and recorded in the
  `13` §3 checklist.
- **est_effort:** 8–12 weeks.
- **parity_ceiling:** RF-level spoof/jam fidelity needs a testbed; not validated.

### WP-09.9 — Redundancy, cross-strapping, voting, FDIR

- **title:** Add N-string redundancy with 2-of-3 voting, cross-strapping, residual FDIR.
- **goal:** N independent sensor strings; mid-value-select / weighted voter;
  cross-strapping routing; per-sensor residual-χ² + persistence FDIR. High
  leverage for SIL realism.
- **fidelity_tier:** T3
- **depends_on:** [WP-09.3]
- **new_crates:** none.
- **touched:** `openbmp-sensors` (multi-string generation),
  `openbmp-fc/src/fdir.rs` (voter/FDIR consumer), `openbmp-runner` (registration);
  scenario `[sensors.redundancy]`.
- **approach:** §3.9.
- **acceptance:**
  - off by default; goldens byte-identical.
  - 2-of-3 voter rejects an injected single-string fault; failover verified.
  - cross-strapping routing unit-tested.
  - FC portability lock intact (voter/FDIR in `openbmp-fc` adds no forbidden
    edge).
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 3–4 weeks.
- **parity_ceiling:** real cross-strapping harness/voter timing not validated.

### WP-09.10 — Reaction wheel + FADS

- **title:** Add reaction-wheel friction/saturation and FADS least-squares air-data.
- **goal:** RW momentum storage with motor + viscous + Coulomb friction +
  zero-speed stiction + saturation; FADS nonlinear-least-squares `(α,β,M,p_∞)`
  from a port array.
- **fidelity_tier:** T3
- **depends_on:** [WP-09.2, WP-09.3]
- **new_crates:** none.
- **touched:** `effector/reaction_wheel.rs` (new); `airdata.rs` (FADS);
  scenario `[vehicle.effector.rw]`, `[sensors.fads]`.
- **approach:** §3.6 (Whitmore triples), §3.8 (RW).
- **acceptance:**
  - off by default; goldens byte-identical.
  - RW saturation + stiction zero-crossing behavior unit-tested; code-to-code vs
    `42` on a maneuver.
  - FADS recovers `(α,β,M)` from synthetic port pressures within tolerance.
  - all §2 gates green.
- **validation_label:** `validated-toy` (RW `42` cross-check)
- **dual_use_note:** vehicle-intrinsic.
- **est_effort:** 3–4 weeks.
- **parity_ceiling:** vendor RW friction maps / FADS calibration not validated.

### WP-09.11 — Terrain-relative / vision nav (pinhole + PnP)

- **title:** Add map-relative vision navigation against a synthetic DEM (forward-only).
- **goal:** Pinhole projection of known landmarks + PnP/homography → map-relative
  **position fix** with `R_map(altitude)` shrinking with altitude; validate vs
  the Mars-2020 error-vs-altitude envelope.
- **fidelity_tier:** T4
- **depends_on:** [WP-09.7]
- **new_crates:** **`openbmp-visionnav` (L3, sim-side, `vision` feature)** —
  placement justification reviewed before implementation; never links a GPL
  solver; FC sees only the resulting position-fix measurement.
- **touched:** new crate; `openbmp-runner` wiring; `data/terrain/` ortho-map;
  scenario `[sensors.trn]`.
- **approach:** §3.7. OpenCV `solvePnP` behind the feature flag or a ported
  pinhole+PnP.
- **acceptance:**
  - off by default; goldens byte-identical.
  - TRN position error vs truth on a synthetic descent within the **order-of-
    magnitude** Mars-2020 envelope (documented as order-of-magnitude only).
  - **dual-use tripwire:** the TRN measurement type compile-fails if it carries a
    ground-aimpoint/target field; no terminal-homing LOS surface.
  - all §2 gates green.
- **validation_label:** `research` (Mars-2020 envelope, order-of-magnitude)
- **dual_use_note:** navigation/landing-site localization ONLY; primary
  architectural guardrail per §6.
- **est_effort:** 8–12 weeks.
- **parity_ceiling:** flight maps/landmark DBs/matching FSW restricted; synthetic
  substitute only.

### WP-09.12 — Resonant TVC + CMG (capstone)

- **title:** Add 4th/6th-order resonant TVC with hinge-moment coupling and CMG steering.
- **goal:** Two-mass-spring resonant TVC load with hinge moment from the `03` aero
  deck (aeroservoelastic margins); CMG singularity-robust steering with
  null-motion escape.
- **fidelity_tier:** T4
- **depends_on:** [`WP-09.2`, `WP-03.*` (aero hinge-moment deck, doc 03), `WP-02.1-a` (modal model, doc 02)]
- **new_crates:** none.
- **touched:** `effector/resonant_tvc.rs`, `effector/cmg.rs` (new);
  scenario `[vehicle.effector.resonant]`, `[vehicle.effector.cmg]`.
- **approach:** §3.8 (two-mass-spring + hinge; `δ̇ = Aᵀ(AAᵀ+λI)⁻¹τ`).
- **acceptance:**
  - off by default; goldens byte-identical.
  - CMG escapes internal/elliptic singularities (unit test); code-to-code vs `42`.
  - resonant-TVC + aero hinge produces the expected aeroservoelastic margin trend
    (model-verification; no specific-vehicle margin claimed).
  - all §2 gates green.
- **validation_label:** `validated-toy` (`42` cross-check); `research` only with a
  public ASE benchmark.
- **dual_use_note:** vehicle-intrinsic.
- **est_effort:** 4–6 weeks.
- **parity_ceiling:** vendor hinge-moment wind-tunnel data and ASE mode shapes
  proprietary; not validated.

---

## 10. References

**Strapdown inertial / specific force**
- P. G. Savage, "Strapdown Inertial Navigation Integration Algorithm Design Part
  1: Attitude" and "Part 2: Velocity and Position," *J. Guidance, Control &
  Dynamics* 21(1) and 21(2), 1998.
- P. G. Savage, *Strapdown Analytics*, 2nd ed., Strapdown Associates, 2007
  (Ch. 4 lever-arm, Ch. 7 specific force).
- J. Bortz, "A New Mathematical Formulation for Strapdown Inertial Navigation,"
  *IEEE Trans. Aerospace & Electronic Systems* AES-7(1), 1971.
- M. B. Ignagni, "Efficient Class of Optimized Coning Compensation Algorithms,"
  *JGCD* 19(2), 1996; R. B. Miller, "A New Strapdown Attitude Algorithm," *JGCD*
  6(4), 1983; Roscoe, "Equivalency Between Strapdown INS Coning and Sculling
  Integrals/Algorithms," *JGCD* 24(2), 2001.
- D. Titterton & J. Weston, *Strapdown Inertial Navigation Technology*, 2nd ed.,
  IET, 2004 (§12.4 size-effect/lever-arm).
- J. A. Farrell, *Aided Navigation: GPS with High Rate Sensors*, McGraw-Hill,
  2008 (Ch. 11).

**GNSS**
- IS-GPS-200, GPS Directorate (SV ephemeris/clock & signal spec).
- P. Misra & P. Enge, *Global Positioning System: Signals, Measurements, and
  Performance*, 2nd ed., Ganga-Jamuna, 2006.
- B. Parkinson & J. Spilker (eds.), *Global Positioning System: Theory and
  Applications*, AIAA, 1996 (RAIM).
- WG-C ARAIM Technical Subgroup, "Milestone 3 Report," EU-US, 2016 (MHSS).
- ESA Navipedia: RAIM/ARAIM (gssc.esa.int/navipedia).
- J. Klobuchar, "Ionospheric Time-Delay Algorithm for Single-Frequency GPS
  Users," *IEEE TAES*, 1987; Saastamoinen, 1972 (troposphere).

**Attitude determination**
- M. D. Shuster & S. D. Oh, "Three-Axis Attitude Determination from Vector
  Observations" (QUEST), *JGCD* 4(1), 1981.
- G. Wahba, "A Least Squares Estimate of Satellite Attitude," *SIAM Review* 7(3),
  1965.
- C. C. Liebe, "Accuracy Performance of Star Trackers — A Tutorial," *IEEE TAES*
  38(2), 2002.
- F. L. Markley & J. Crassidis, *Fundamentals of Spacecraft Attitude
  Determination and Control*, Springer, 2014 (MEKF, Ch. 5–6); Lindegren
  (centroiding bound).

**Relative / vision nav**
- A. Johnson et al., "The Mars 2020 Lander Vision System" / "Real-Time Terrain
  Relative Navigation Test Results for Mars Landing," JPL/AIAA.
- Y. Cheng & A. Ansar, "Landmark Based Position Estimation for Pinpoint Landing
  on Mars," ICRA 2005; S. Mohan et al., "Map Relative Localization System for
  Planetary Landing," *JGCD*, 2023.
- R. Hartley & A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd
  ed., Cambridge, 2004; B. Açıkmeşe & S. Ploen, "Convex Programming Approach to
  Powered Descent Guidance."

**Actuators**
- Cowan & Olds / NASA, "Electromechanical Actuation for Thrust Vector Control
  Applications," NASA NTRS 19900011956.
- "A Novel Control Approach for a Thrust Vector System with an Electromechanical
  Actuator" (Lagrange–Maxwell EMA, friction/backlash/flexibility).
- Greensite, *Analysis and Design of Space Vehicle Flight Control Systems*, NASA
  CR-820; P. Zarchan, *Tactical and Strategic Missile Guidance*, AIAA (fin
  actuator + hinge moment).
- B. Wie, *Space Vehicle Dynamics and Control*, 2nd ed., AIAA, 2008 (RCS, PWPF,
  RW, CMG, Ch. 7–8); H. Schaub & J. Junkins, *Analytical Mechanics of Space
  Systems*, 4th ed., AIAA (VSCMG steering, singularity); Oh & Vadali,
  singularity-robust CMG steering; NASA, "The Minimum Impulse Thruster," NTRS
  20070032005.

**Air-data / redundancy**
- S. A. Whitmore et al., "Development of a Pneumatic High-Angle-of-Attack Flush
  Airdata Sensing (HI-FADS) System," NASA TM-104241; Gracey, "Measurement of
  Aircraft Speed and Altitude," NASA RP-1046.
- R. P. G. Collinson, *Introduction to Avionics Systems*, 3rd ed., Springer, 2011
  (redundancy, voting, cross-strapping); RTCA DO-178C / ARP4754A (FDIR
  vocabulary); J. L. Crassidis & J. L. Junkins, *Optimal Estimation of Dynamic
  Systems* (measurement latency / out-of-sequence).

**Open-source tools:** RTKLIB (BSD-2), GPS-SDR-SIM (MIT), gnss-sdr (GPLv3 —
external only), GeographicLib (MIT), Hipparcos/Tycho-2/Gaia (PD/CC-BY),
USGS/PDS/SRTM DEMs (PD), OpenCV (Apache-2.0), NASA `42` (NOSA), `allan_variance`
/ `gnss-ins-sim` (MIT), basic-airdata/COESA-1976 (MIT/Apache-2.0).

**Upstream OpenBMP anchors:** `docs/verification.md`, `docs/standards-posture.md`,
`docs/safety-boundaries.md`, `docs/data-provenance.md`, `00-overview.md`,
`13-agent-execution-playbook.md`.

---

*Companion documents:* `00-overview.md` (constitution) · `05`, `06`, `08`
(hard dependencies) · `02`, `03`, `07`, `10`, `11`, `12` (couplings) · `13`
(execution playbook).
