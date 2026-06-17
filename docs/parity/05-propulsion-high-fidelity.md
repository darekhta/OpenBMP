# High-Fidelity Propulsion: Transient Engine, Feed, Plume, POGO

**Status:** `experimental` (design intent; this document ships no code).
**Audience:** the engineer or LLM agent implementing the propulsion work packages.
**One-line summary:** take OpenBMP from time-driven thrust *curves* to a
two-tier physical propulsion stack — pressure-thrust + altitude, transient
chamber ballistics, an ingested CEA/Cantera thermochemistry deck, a GFSSP-style
feed network with turbopump maps and MOC line transients, and a POGO
closed-loop capstone coupled to the longitudinal structural mode of
`02-structural-dynamics-loads-slosh-pogo.md` — while keeping every byte
deterministic, off-by-default, and forward-only.

> Prerequisite reading: `00-overview.md` (parity definition, solver-consumer
> posture, invariants, crate map, DAG) and `13-agent-execution-playbook.md`
> (the §4 template this doc follows and the §5 work-package schema the backlog
> obeys). This is the per-dimension design for gap-ledger row 05.

---

## 1. Parity target & ceiling

### 1.1 Target (capability parity within posture)

The end-state is **method/architecture parity** with the propulsion modeling
done at SpaceX/Rocket Lab/NASA/JPL for *forward* launch-vehicle simulation:

- **Pressure-thrust with altitude.** Thrust is the full momentum + pressure
  equation `F = ṁ·Vₑ + (pₑ − pₐ)·Aₑ`, with ambient `pₐ(h)` consumed from the
  atmosphere crate every step. A sea-level-tuned engine *gains* thrust climbing
  to vacuum; over/under-expansion is modeled, not assumed away. The stored-but-
  unused `exit_area_m2` becomes load-bearing.
- **Transient internal ballistics.** Solid motors integrate a lumped-volume
  chamber-pressure ODE `pc(t)` with ignition transient, tail-off, erosive
  burning, and the `Kn(web)` curve from real grain geometry — replacing the
  fixed-web quasi-static march. Liquid engines gain a chamber-pressure state and
  a feed coupling instead of a flat `mdot = F/(g₀·Isp)`.
- **Thermochemistry as a derived deck.** `c*(pc, MR)`, `Tc`, equilibrium `γ`,
  `MW`, and frozen/equilibrium `Isp` come from a CEA/Cantera-generated lookup
  deck, ingested with provenance — not hardcoded `c_star_m_s`/`gamma` constants.
- **Feed system + turbopump + transients.** A reduced GFSSP-style finite-volume
  fluid network (tanks → valves → injector → chamber) produces transient
  `pc(t)`, `ṁ_ox(t)`, `ṁ_fuel(t)`, mixture-ratio excursions, and throttle
  response; a normalized turbopump map + affinity laws + NPSH set pump-fed feed
  pressure; Method-of-Characteristics captures water-hammer/priming surges.
- **POGO closed-loop stability (capstone).** The feed transfer function couples
  to the vehicle's first longitudinal structural mode (the Rubin/Oppenheim loop)
  to predict the self-excited limit cycle and size a suppression accumulator —
  the propulsion side of the shared POGO capability with
  `02-structural-dynamics-loads-slosh-pogo.md`.
- **Deterministic fault library.** Engine-out, valve stuck/leaking, cavitation,
  hard-start, MR runaway — forward perturbations of the simulated plant injected
  at the existing FC/sensor fault boundary.

Under the solver-consumer posture (`00` §1.1) the *transient physics* is pure
simulation and reaches high in-repo fidelity directly; the *thermochemistry* and
the *3D plume/base-heating* fields are consumed as provenance-pinned external
decks with an in-repo reduced tier and a code-to-code anchor.

### 1.2 The parity ceiling (what cannot be matched, and the open substitute)

Stating this is mandatory (`00` §1, `13` §1.4). OpenBMP can reach method parity;
it cannot reach **validation parity**. Specifically, the following are out of
reach in an open repo, with the honest open substitute named:

1. **Real-engine c*/Isp efficiency.** CEA gives the *ideal* thermochemical
   ceiling; real engines run ~92–99% of `c*_ideal` and the injector mixing
   efficiency `η_c*`, heat-loss-corrected `Tc`, and combustion-stability margins
   are proprietary hot-fire IP. **Open substitute:** apply *published empirical
   efficiency bands* (Sutton & Biblarz tabulates ranges per engine class) as a
   documented, uncertainty-tagged multiplier on the ideal deck; validate the
   *shape* (trend vs `pc`, `MR`, `ε`) against CEARUN and public datasheets, never
   a single engine's absolute number.
2. **Turbopump performance maps.** Real head/flow/efficiency/cavitation curves
   are proprietary. **Open substitute:** a generic normalized centrifugal/inducer
   map keyed by specific speed `Ns`, calibrated only to the *published design
   point*, with the map shape carried as an uncertainty band.
3. **POGO cavitation parameters.** Cavitation compliance `C_b` and mass-flow-
   gain factor `M_b` are the least-known parameters in all of rocketry, fit per
   engine from test. **Open substitute:** Brennen/Acosta theoretical/quasi-static
   compliance bounds + a documented dispersion box + matching the *published
   unstable-frequency band* (Saturn V S-II, Titan II) rather than a limit-cycle
   amplitude.
4. **Validated 3D plume / base-recirculation heating.** Requires proprietary
   CFD–test correlation (JANNAF SPF/RAMP). **Open substitute:** offline
   SU2/OpenFOAM/SPARTA reference runs feeding a *reduced engineering plume/base
   model*, plus MMS for solver verification.
5. **Certification / flight heritage.** A GFSSP-validated feed model for a
   *specific* vehicle is out of reach. Honest framing throughout: "verification
   against open benchmarks and code-to-code vs GFSSP/CEA/OpenMotor," explicitly
   **not** "flight-qualified."

No artifact in this dimension ever claims `flight-qualified`/`certified`/
`mission-ready` (invariant 8).

---

## 2. Current state in source

Verified by reading the crate (`crates/openbmp-propulsion/`). The current
baseline to regress against:

| Area | File:line | Current behavior (Tier 0) |
|---|---|---|
| Thrust coefficient / nozzle | `src/motor.rs` `IdealNozzlePerformance`, `NozzleSolution` | `AmbientPressureCorrection::Constant` preserves legacy thrust curves. `PressureThrust` reconstructs chamber state from the stored momentum-thrust curve, solves the supersonic area-Mach relation, and applies `(pₑ−pₐ)Aₑ` from the runtime atmosphere. `NozzleSeparationCriterion::{Off,Summerfield,Schmucker}` can clip the effective expansion ratio for heavily overexpanded flow; off is the default. |
| Exit area | `src/grain.rs`, `src/motor.rs` | `exit_area_m2 = throat·ε` is stored on `MotorGeometry`; pressure-thrust motors additionally require `throat_area_m2` and `gamma`. When separation triggers, `NozzleSolution` reports `separated`, `effective_expansion_ratio`, and `effective_exit_area_m2`. |
| Solid ballistics | `src/grain.rs` `EquilibriumInternalBallistics::regress()` | Default `GrainRegressionMode::QuasiStatic` preserves the fixed-web equilibrium `pc(Kn)` march. Opt-in `GrainRegressionMode::Transient` integrates a reduced lumped-volume `pc(t)` ODE with deterministic ignition transient and an erosive burn-rate hook, then emits a `ThrustCurve` once, offline. |
| Propellant constants | `src/grain.rs:67-69` | `c_star_m_s` and `gamma` are *input constants* on `GrainPropellant`; no chemistry, no `(pc, MR)` dependence. |
| Liquid engine | `src/engine.rs:554-811` `LiquidEngine` | Linear ignition/shutdown ramps, rate-limited throttle, constant `isp_s`. Thrust `= throttle·max_thrust·feed_scale`; `mdot = thrust/(g₀·Isp_eff)` (`engine.rs:733`). **No chamber pressure, no `c*`, no mixture ratio, no nozzle CF.** |
| Feed / blowdown | `crates/openbmp-vehicle/src/propellant_budget.rs:304-323` | Quasi-static isentropic ullage blowdown: `scale = (V₀_ullage / V_ullage)^{ullage_gamma}` (`propellant_budget.rs:319`), applied to the engine as a scalar `feed_pressure_scale` (`engine.rs:384`, `:798`). **No feedlines, valves, inertance, or transients.** |
| Cluster | `src/cluster.rs` | `EngineCluster` sums per-engine snapshots in declared order; kernel-side force/mass adapters live in `openbmp-vehicle::adapters`. Engine-out is expressible via the `EngineFault` enum (`engine.rs:294`) but there is no fault scheduler/library. |
| Parser | `src/parser.rs` (feature `parser`) | RASP-shaped TOML motor deck loader; file I/O at load time only. |

**Determinism today is solid:** pure `f64`, locked operand order, no FMA, no
wall-clock/RNG on the hot path (`lib.rs:47-54`). The new tiers must preserve
this exactly.

**Net:** Tier 0 remains a fast, deterministic, *time-driven curve* model, now
with the T1 pressure-thrust correction plus algebraic nozzle separation and a
synthetic/toy transient solid-chamber regression path from T2 available behind
explicit config. The next propulsion gap is external thermochemistry evidence
from CEARUN/Cantera tolerance tables and higher-fidelity validation, not the
ambient pressure term.

---

## 3. Target architecture

### 3.0 Crates: changed + new

```text
CHANGED:
  openbmp-propulsion  L2  + pressure-thrust CF(pa), nozzle separation, transient
                          pc(t) ODE (solid + liquid chamber), thermochem-deck
                          consumption, fault library, NozzlePerf/ChamberState traits
  openbmp-models      L1  + thermochem-deck + feed-network model trait surfaces
  openbmp-vehicle     L2  + feed-network ↔ engine coupling adapter; POGO modal hook
  openbmp-runner      L7  + transient-engine wiring, fault-event scheduler, MC hooks
  openbmp-uq          L1  + per-deck bias/random margins for efficiency bands (doc 11)

NEW (proposed; placement justified in the introducing WP, reviewed first):
  openbmp-thermochem  L2  CEA/Cantera deck schema + parser + bilinear interpolator
                          + (optional) in-repo min-G equilibrium reduced tier
  openbmp-feedsystem  L2  GFSSP-style node/branch network, turbopump map + affinity
                          + NPSH, MOC water-hammer, throttle/MR PID; POGO feed
                          transfer-function assembly
```

`openbmp-thermochem` and `openbmp-feedsystem` are **L2**: they depend only on
`openbmp-core` (L0) + `openbmp-models` (L1) + math crates, never on
`openbmp-sim` (L1 kernel), `openbmp-fc`, the runner, or any up-layer crate. This
keeps the acyclic DAG and — critically — keeps the **FC portability lock**
(invariant 3) intact: `openbmp-fc` gains no edge to either. The POGO stability
solver lives in `openbmp-feedsystem` and *consumes* a longitudinal modal model
produced by the multibody/structural side (`01`/`02`); it does not import the
structural crate's kernel wiring, only its plain modal-data struct.

> Per `13` §5, the first commit of any WP that introduces one of these crates is
> the skeleton + placement justification (layer, edges, why it does not break
> the DAG or the FC lock), reviewed before implementation.

### 3.1 Fidelity tiers (the ladder)

| Tier | Name | New physics | Label target |
|---|---|---|---|
| **T0** | *current* | pre-tabulated solid curve; constant `Isp/c*/γ` liquid; quasi-static `(V₀/V)^γ` blowdown; `exit_area` stored-not-consumed; optimum-CF only | — |
| **T1** | Pressure-thrust + altitude | `F = ṁVₑ + (pₑ−pₐ)Aₑ` with `pₐ(h)`; sea-level/vacuum Isp split; consume `exit_area_m2`; `CF(pₐ)` | `validated-toy` |
| **T2** | Separation + transient solid | Summerfield/Schmucker separation clipping of `CF(pₐ)`; replace fixed-web march with transient `pc(t)` ODE + erosive burning + `Kn(web)` | `validated-toy`→`research` |
| **T3** | Thermochemistry deck | ingest CEA/Cantera `c*(pc,MR)`, `Tc`, `γ`, `MW`; runtime bilinear interp; MR-aware liquid performance + empirical efficiency band | `research` |
| **T4** | Feed network + turbopump + water-hammer | GFSSP-style node/branch network; pump map + affinity + NPSH; MOC line transients; PID throttle + MR control | `validated-toy` (network), `research` (vs GFSSP cases) |
| **T5** | POGO + full fault library *(capstone)* | Rubin/Oppenheim feed↔structure closed-loop stability + accumulator sizing; full engine-out/valve/cavitation/hard-start fault enum into the SIL/EKF loop | `research` (method-level only) |

Tiers are independently shippable (`00` §9, `13` §1). T1 lands in days; T5 is the
capstone that must come last (`00` §5: needs `02` longitudinal mode + `05` feed).

### 3.2 Trait surfaces (Rust)

The design adds three model trait families. They sit beside the existing `Motor`
/ `EngineModel` traits and are consumed by them, preserving the parallel-trait
architecture (`lib.rs:37-46`).

#### 3.2.1 Nozzle performance — `NozzlePerformance` (T1/T2)

```rust
/// Ambient-pressure-aware nozzle thrust coefficient and exit state.
/// Pure function of geometry, chamber state, and ambient pressure;
/// no time, no I/O. Locked operand order, no FMA (invariant 1).
pub trait NozzlePerformance {
    /// Thrust coefficient CF and exit pressure pe at ambient pa.
    /// Includes the pressure term; clips for flow separation when the
    /// configured `SeparationCriterion` triggers.
    ///
    /// # Errors
    /// Fails closed on non-finite inputs, gamma <= 1, eps < 1, or a
    /// non-convergent area-Mach inversion.
    fn solve(
        &self,
        chamber: &ChamberState,
        ambient_pa: f64,
    ) -> Result<NozzleSolution, PropulsionError>;
}

/// Result of a nozzle solve at one ambient condition.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NozzleSolution {
    pub cf: f64,              // thrust coefficient, ambient-corrected
    pub pe_pc: f64,           // exit/chamber pressure ratio
    pub exit_mach: f64,       // Me on the supersonic branch
    pub separated: bool,      // true if flow separates inside the nozzle
    pub eps_effective: f64,   // effective area ratio after separation clip
}

/// Chamber thermodynamic + geometric state passed to the nozzle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChamberState {
    pub pc_pa: f64,           // stagnation chamber pressure
    pub gamma: f64,           // ratio of specific heats (from deck at T3)
    pub c_star_m_s: f64,      // characteristic velocity (from deck at T3)
    pub throat_area_m2: f64,
    pub expansion_ratio: f64, // geometric Ae/At
}

/// How nozzle flow separation is modeled (config-gated; default None
/// preserves T1 ideal-CF behavior).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SeparationCriterion {
    None,
    Summerfield,   // p_wall ~ 0.35..0.40 * pa
    Schmucker,     // psep/pa = (1.88 Me - 1)^(-0.64)
    KaltBadal,
}
```

`F = CF · pc · At` and `Isp = CF · c* / g₀` close the loop; the existing
`supersonic_mach_for_area_ratio()` and isentropic relations in `grain.rs` are
reused verbatim (no new math for the inversion).

#### 3.2.2 Thermochemistry deck — `ThermochemDeck` (T3)

```rust
/// Read-only equilibrium-thermochemistry lookup, ingested offline from
/// CEA/Cantera over a (pc, MR) grid per propellant pair. Carries an
/// efficiency band (the open substitute for proprietary eta_c*).
pub trait ThermochemDeck {
    /// Interpolate ideal c*, Tc, gamma, MW at (pc, MR). Bilinear in
    /// log-pc and linear in MR; locked corner order.
    ///
    /// # Errors
    /// Fails closed outside the declared (pc, MR) envelope (no
    /// extrapolation) and on non-finite queries.
    fn evaluate(&self, pc_pa: f64, mixture_ratio: f64)
        -> Result<ThermochemPoint, PropulsionError>;

    /// Published empirical c* efficiency band (lo, nominal, hi) for the
    /// engine class. NOT a proprietary per-engine eta_c*.
    fn c_star_efficiency_band(&self) -> EfficiencyBand;

    fn propellant_pair(&self) -> &str;        // e.g. "LOX/RP-1"
    fn validation(&self) -> ValidationStatus; // research when CEARUN-anchored
    fn provenance(&self) -> &DeckProvenance;   // tool, version, grid, SHA pin
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThermochemPoint {
    pub c_star_ideal_m_s: f64,
    pub tc_k: f64,
    pub gamma: f64,
    pub molar_mass_kg_kmol: f64,
    pub isp_vac_s: f64,
    pub isp_sl_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EfficiencyBand { pub lo: f64, pub nominal: f64, pub hi: f64 }
```

Delivered `c* = c*_ideal · η_c*`, with `η_c*` drawn from the band and propagated
into Monte Carlo by `openbmp-uq` (doc 11).

#### 3.2.3 Feed network — `FeedNetwork` + `TransientChamber` (T2/T4)

```rust
/// Lumped finite-volume fluid network (GFSSP-style). Nodes carry mass +
/// energy; branches carry momentum (resistance + inertance). Advanced
/// by an implicit Newton step at the kernel base tick.
pub trait FeedNetwork {
    /// Advance one base tick. Returns per-branch flows and node
    /// pressures, plus the injector mass flows + mixture ratio handed
    /// to the chamber. Deterministic Newton solve, fixed iteration cap.
    ///
    /// # Errors
    /// Fails closed on non-convergence within the iteration cap, on a
    /// non-finite residual, or on an unphysical (negative) flow node.
    fn step(&mut self, dt: Duration, commands: &FeedCommand)
        -> Result<FeedSnapshot, PropulsionError>;

    fn validation(&self) -> ValidationStatus;
}

/// Lumped-volume chamber: pc(t) ODE closed by the choked throat.
pub trait TransientChamber {
    /// Integrate dpc/dt one step. For solids: gas mass balance with the
    /// grain source. For liquids: injector inflow minus choked throat.
    ///
    /// # Errors
    /// Fails closed on non-finite pc or a sub-atmospheric blow-out.
    fn advance(&mut self, dt: Duration, inflow: ChamberInflow)
        -> Result<ChamberState, PropulsionError>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedSnapshot {
    pub injector_mdot_ox_kg_s: f64,
    pub injector_mdot_fuel_kg_s: f64,
    pub mixture_ratio: f64,
    pub pump_inlet_pa: [f64; 2],   // ox, fuel suction
    pub npsh_margin_m: [f64; 2],   // available - required (cavitation gate)
    pub line_surge_pa: f64,        // peak MOC water-hammer this step
}
```

The POGO solver (T5) is a frequency-domain method on top of `FeedNetwork`:

```rust
/// Rubin/Oppenheim feed transfer function: chamber-pressure perturbation
/// -> thrust perturbation, assembled from the linearized network at a
/// frozen flight condition. Couples to a longitudinal modal model from
/// doc 02 to form the closed-loop characteristic equation.
pub trait PogoStability {
    /// Closed-loop stability at a flight condition, given the vehicle's
    /// longitudinal modal model (frequencies, modal mass, engine-station
    /// gain) and the pump cavitation dispersion box.
    fn assess(
        &self,
        longitudinal_mode: &LongitudinalModalModel, // from doc 02
        cavitation: &CavitationDispersion,          // C_b, M_b box
    ) -> Result<PogoVerdict, PropulsionError>;
}

#[derive(Clone, Debug, PartialEq)]
pub struct PogoVerdict {
    pub unstable: bool,
    pub critical_freq_hz: f64,
    pub gain_margin_db: f64,
    pub phase_margin_deg: f64,
    pub accumulator_compliance_m3_per_pa: Option<f64>, // suggested sizing
}
```

### 3.3 The math (named formulations, equations, references)

All symbols below are propulsion-internal. No targeting content; no range field.

**(M1) Pressure-thrust + altitude (T1).** Per step with `pₐ(h)` from the
atmosphere crate (`crates/openbmp-physics/src/atmosphere/`):

1. Exit Mach `Mₑ` from the geometric area ratio `ε = Aₑ/At` by inverting the
   isentropic area–Mach relation on the supersonic branch:
   `ε = (1/Mₑ)·[ (2/(γ+1))·(1 + ((γ−1)/2)·Mₑ²) ]^{(γ+1)/(2(γ−1))}`
   (bisection — already present as `supersonic_mach_for_area_ratio`, `grain.rs:594`).
2. `pₑ/pc = (1 + ((γ−1)/2)·Mₑ²)^{−γ/(γ−1)}`.
3. Momentum CF:
   `CF_mom = sqrt( (2γ²/(γ−1))·(2/(γ+1))^{(γ+1)/(γ−1)}·(1 − (pₑ/pc)^{(γ−1)/γ}) )`.
4. Pressure-corrected CF: `CF = CF_mom + (pₑ/pc − pₐ/pc)·ε`.
5. `F = CF·pc·At`, `Isp = CF·c*/g₀`. Vacuum thrust is the `pₐ → 0` ceiling.

*Refs:* Sutton & Biblarz, *Rocket Propulsion Elements* 9e, Ch.3 (eqs 3-30..3-34)
& Ch.5; NASA SP-8120; Anderson, *Modern Compressible Flow* 3e, Ch.5. *Reuse:*
`optimum_thrust_coefficient` (`grain.rs:573`) is generalized to take `pₐ`; the
optimum case is `pₐ = pₑ`, a regression-preserving special case.

**(M2) Nozzle flow separation (T2).** When a fixed-`ε` nozzle runs heavily
overexpanded near sea level (`pₑ ≪ pₐ`) the boundary layer separates inside the
nozzle; thrust does *not* collapse to the ideal overexpanded value. Find the
separation pressure `p_sep` and the wall station where `p_wall = p_sep`:

- Summerfield (1954): separation at `p_wall ≈ 0.35–0.40·pₐ`.
- Schmucker (1973): `p_sep/pₐ = (1.88·Mₑ − 1)^{−0.64}`.
- Kalt–Badal: empirical `p_sep/pₐ` fit.

Algorithm: if computed `pₑ < p_sep`, march the inner isentropic relation to the
area ratio `ε_sep` where `p_wall(ε_sep) = p_sep`; use `ε_sep`, `p_sep` in the
pressure term instead of geometric `Aₑ`, `pₑ`. Pure scalar code, no PDE.
*Refs:* Stark (DLR) FSS/RSS overview; Schmucker NASA TM-77396; Frey & Hagemann,
*J. Propulsion & Power* 16(3) 2000.

**(M3) Transient solid ballistics (T2).** Replace the fixed-web march with a
lumped-volume chamber-gas ODE:

`d(ρ_c·Vc)/dt = ρ_p·Ab·r − ṁ_throat`, `dVc/dt = Ab·r`, `ṁ_throat = pc·At/c*` (choked).

Burn rate `r = a·pcⁿ` (St. Robert/Vieille) plus erosive augmentation
(Lenoir–Robert): `r_e = (α·G^{0.8}/L^{0.2})·exp(−β·ρ_p·r/G)`, `G` = local mass
flux. `Kn(web) = Ab(web)/At` drives the equilibrium reference
`pc = (a·ρ_p·c*·Kn)^{1/(1−n)}` (already in steady code at `grain.rs:553`).
Integrate `pc(t)` with a locked-order explicit step; ignition transient from
filling the free volume; tail-off from sliver burnout. *Refs:* Sutton & Biblarz
Ch.11–12; Lenoir & Robert (1956); NASA SP-8039; OpenMotor (validation only,
GPL — re-implement from the textbook, do not copy).

**(M4) Equilibrium thermochemistry (T3, ingested).** `c*(pc,MR)`, `Tc`, `γ`,
`MW` from Gibbs free-energy minimization (Gordon–McBride element-potential
method): minimize `G = Σⱼ nⱼ·(g⁰ⱼ(T) + RT·ln(nⱼP/n_total))` subject to elemental
mass balance `Σⱼ a_ij·nⱼ = b_i⁰`, via the linearized Newton system of RP-1311
eqs (2.18)–(2.24) on `(Δln nⱼ, Δln n_total, element potentials πᵢ)`, with the
9-coefficient NASA polynomials `Cp/R = a₁T⁻² + a₂T⁻¹ + a₃ + a₄T + a₅T² + a₆T³ +
a₇T⁴`. `c* = sqrt(R_specific·Tc)/Γ(γ)`, `Γ(γ) = sqrt(γ)·(2/(γ+1))^{(γ+1)/(2(γ−1))}`.
**Solver-consumer path:** run CEA/Cantera *offline* over a `(pc, MR, ε)` grid,
ingest as a lookup deck with bilinear interpolation; keep the Rust core
dependency-free and deterministic. An in-repo min-G *reduced tier* is optional
(supports MMS verification) but is not the primary path. *Refs:* Gordon &
McBride NASA RP-1311 I/II; NASA CEA; Cantera (BSD-3).

**(M5) Feed-network finite-volume (T4).** GFSSP-style: nodes carry mass + energy
conservation, branches carry momentum.
- Node mass: `Vc·dρ/dt = Σ(ṁ_in − ṁ_out)`.
- Branch momentum: `pᵢ − pⱼ = K·ṁ·|ṁ| + I·(dṁ/dt)`, inertance `I = L/A`.
- Chamber closure: `dpc/dt = (R·Tc/Vc)·(ṁ_in − ṁ_throat)`, `ṁ_throat = pc·At/c*`.
- Valve = variable resistance `K(t)` from a commanded `Cv` schedule; injector =
  fixed-K orifice.
- Solve the coupled nonlinear system per step by Newton–Raphson (GFSSP uses
  successive-substitution + Newton), fixed iteration cap, locked assembly order.
Throttle = PID on valve `K` to a commanded `pc` setpoint; MR control = ratio of
two valve commands coupled to **(M4)**'s `c*(MR)`. *Refs:* Majumdar, GFSSP V6
(NASA/TP, NTRS 20150016531); Sutton & Biblarz Ch.6, Ch.10. *Solver-consumer:*
GFSSP is the **validation oracle**, re-implement the published finite-volume
algorithm — do not vendor.

**(M6) Water-hammer (Method of Characteristics, T4).** 1-D compressible-liquid
PDEs `(a²/g)·∂V/∂x + ∂H/∂t = 0` and `∂V/∂t + g·∂H/∂x + fV|V|/(2D) = 0`, wave
speed `a = sqrt((K/ρ)/(1 + (K/E)(D/e)c₁))`. MOC → ODEs along
characteristics `dx/dt = ±a`; fixed-grid with Courant `a·Δt/Δx = 1`:
`C⁺: H_P = C_P − B·Q_P`, `C⁻: H_P = C_M + B·Q_P`, `B = a/(gA)`,
`C_P = H_{i−1} + B·Q_{i−1} − R·Q_{i−1}|Q_{i−1}|`,
`C_M = H_{i+1} − B·Q_{i+1} + R·Q_{i+1}|Q_{i+1}|`, `R = fΔx/(2gDA²)`.
Joukowsky bound `ΔH = a·ΔV/g` (instantaneous closure) as the analytic sanity
check. Explicit, no matrix solve — ideal for deterministic Rust arrays. *Refs:*
Wylie & Streeter, *Fluid Transients in Systems* (1993); Chaudhry, *Applied
Hydraulic Transients* 3e; Joukowsky (1900).

**(M7) Turbopump map + affinity + NPSH (T4).** Dimensionless map: head
coefficient `ψ = gH/(N²D²)`, flow coefficient `φ = Q/(ND³)` → single normalized
`ψ(φ)` scales across speeds. Affinity laws: `Q₂/Q₁ = N₂/N₁`, `H₂/H₁ =
(N₂/N₁)²`, `P₂/P₁ = (N₂/N₁)³`. Operating point: solve `H_pump(Q,N) = H_system(Q)`
(system curve from **(M5)** branch resistances + static + chamber backpressure).
Specific speed `Ns = N·sqrt(Q)/H^{0.75}` selects the map family.
`NPSH_avail = (p_tank − p_vapor)/(ρg) + static head − line loss`; require
`> NPSH_req(Q)` or model cavitation head breakdown (a `ψ` cliff) and feed the
cavitation compliance into POGO **(M8)**. *Refs:* Sutton & Biblarz Ch.10;
Brennen, *Hydrodynamics of Pumps* (1994); NASA SP-8052. *Open substitute:*
generic normalized map keyed by `Ns`, calibrated to the published design point.

**(M8) POGO closed-loop stability (T5, capstone).** Rubin lumped-parameter feed
model: each element (tank, line, pump, accumulator) is a hydraulic 2-port with
inertance `I = L/A`, resistance `R`, compliance `C`. Pump cavitation adds the two
dominant POGO parameters: cavitation compliance `C_b = −dV_cav/dp_suction` and
mass-flow-gain `M_b = −dṁ/dp_suction`. Build the feed transfer function
`G_feed(s)` from chamber-pressure perturbation to thrust perturbation; couple to
the longitudinal modal equation `m·q̈ + c·q̇ + k·q = φ_engine·ΔF`,
`ΔF = G_feed(s)·(structural acceleration at the tank)`. Close the loop:
`det(I − G_feed(s)·G_struct(s)) = 0`; assess stability by complex eigenvalues /
Nyquist. Accumulator = added compliance `C_acc` detuning the feed resonance below
the structural mode; SP-8055 mandates ≥ +6 dB gain / sufficient phase margin
across the dispersion box. Build the **linear transfer-function check first**,
time-domain limit cycle second. *Refs:* Rubin, *J. Spacecraft & Rockets* 3(8)
1966; NASA SP-8055 (1970); Oppenheim & Rubin, *J. Spacecraft & Rockets* 30(3)
1993; Brennen & Acosta (cavitation compliance/MFG). The longitudinal modal model
and the structural side of this loop are designed in
`02-structural-dynamics-loads-slosh-pogo.md`; this doc owns the feed half.

**(M9) Fault library (T2+, deterministic).** Finite-state machine per engine
`{nominal, throttled, shutdown, failed}` with transitions {commanded,
fault-injected at `t_fault`, condition-triggered (e.g. `NPSH < NPSH_req →`
cavitation)}. On engine-out: zero that engine's thrust vector, recompute cluster
net thrust + torque about CG (`cluster.rs` already aggregates), let GNC respond.
Faults are *forward perturbations of the simulated plant* injected at the
existing FC/sensor boundary (commit `1a88b1d`); unknown fault types fail closed.
The `EngineFault` enum (`engine.rs:294`) is the seed; T5 adds a typed
scheduled-event list in the scenario TOML. *Refs:* Sutton & Biblarz Ch.6/11;
NASA-STD-8729; NASA SP-8055 (POGO as a fault mode).

---

## 4. Invariant preservation

Concretely, for this dimension (cross-referencing `00` §3, `13` §3):

1. **Byte-determinism.** Every new model advances by locked-operand-order
   weighted sums, never `f64::mul_add`, never wall-clock/system RNG, never
   unordered iteration. Specific risks and mitigations:
   - *Newton solves (M5/M7) and complex eigen/Nyquist (M8):* fixed iteration
     cap, locked Jacobian assembly order, fail-closed on non-convergence; never
     a tolerance-dependent variable iteration count that could diverge across
     platforms. The eigen/QR routines come from `faer` (pure-Rust, MIT/Apache),
     not an FFI BLAS, to keep the deterministic-f64 contract.
   - *MOC (M6):* explicit, fixed-grid, no matrix solve — the friendliest case;
     locked left-to-right characteristic sweep.
   - *ODE steps (M3, chamber pc):* explicit, locked operand order, the same
     symplectic-style discipline the existing engine/motor code uses.
   - *Deck interpolation (M4):* bilinear with a *locked corner summation order*;
     no `mul_add`.
   - New randomness (efficiency-band draws, dispersion boxes) derives only from
     `openbmp-core::DeterministicRng` with a domain-separated
     `(seed, step, id, component)` tuple — never system RNG.
2. **Byte-stable-by-default.** Every tier is **off by default** behind an
   explicit scenario block (e.g. `[propulsion.nozzle]` with
   `ambient_correction = "off"` default; `[propulsion.feed_network]`;
   `[propulsion.pogo]`), following the `[vehicle.bending]` pattern. `T1`'s
   pressure term defaults to `AmbientPressureCorrection::Constant` (the current
   behavior) until a scenario opts into `PressureThrust`, so all canonical
   goldens stay byte-identical. The CI byte-diff gate on the canonical scenario
   set is the proof.
3. **FC portability lock.** `openbmp-thermochem`, `openbmp-feedsystem`, and all
   new propulsion code are L2 and **never** become an `openbmp-fc` dependency.
   The POGO solver consumes a plain `LongitudinalModalModel` data struct, not the
   structural kernel crate. `fc_dependency_tripwire.rs` stays green; if any WP is
   tempted to let the FC read feed/chamber state, it routes through the existing
   FC-bridge/sensor boundary (a *sensor*, e.g. a chamber-pressure transducer
   model in `openbmp-sensors`), never an import.
4. **Lockstep-clock & no-hot-path-allocation.** The feed-network and chamber
   ODE state lives in pre-sized buffers allocated at construction; per-tick
   `step()` does not allocate or clone the node/branch topology. Time enters only
   through the injected `dt: Duration`; no `Instant::now`. `fc_lints.rs` stays
   green.
5. **Four-pillar provenance.** Thermochem decks land in `data/thermochem/<pair>/`
   with a sibling `provenance.md` (tool = CEA/Cantera, version, `(pc,MR,ε)` grid,
   SHA-256 pin, license note) verified fail-closed at scenario load; turbopump
   map parameters and cavitation dispersion boxes are textbook/published values
   in their declared source-of-truth files with citations. No inline TOML in
   `*.rs`. **No literal WGS84 GM/J₂/R⊕ numerals anywhere** — propulsion uses
   `g₀ = STANDARD_GRAVITY_M_S2` (already a named const) and never the geopotential
   constants. `inline_data_tripwire.rs` + `openbmp check-provenance` stay green.
6. **Synthetic/public-data-only.** Decks are CEA/Cantera-generated (public-domain
   / BSD tools) or textbook; *no proprietary engine c*/Isp efficiency, no real
   turbopump map, no fielded cavitation parameters.* The efficiency band is a
   *published range*, not a vehicle's number.
7. **Forward-only mechanical locks.** All nine methods are plant physics
   (engine → thrust/mass → vehicle state) entirely behind the Tier-1 locks. No
   method introduces a ground-aimpoint or fire-control path. Throttle/MR control
   closes on chamber-pressure / mixture-ratio setpoints (engine-internal); POGO
   couples to the vehicle's own structural mode (self-referential). The
   `ballistic_state_compile_fail.rs` tripwire stays green; this dimension *adds*
   no terminal-condition surface, so it strengthens the locks by leaving them
   untouched.
8. **Validation labels.** Each tier declares exactly one label (table in §3.1),
   justified by §5 evidence, with a tolerance-table TOML for every numeric claim.
   The honest cap: the thermochem and POGO *methods* can earn `research` via
   CEARUN / published-trend benchmarks; a *specific vehicle's* feed model or POGO
   margin never claims more than `research` and is explicitly not flight-
   qualified.
9. **Workspace lints & docs.** `missing_docs`/`unsafe_code`/`unused_must_use`/
   `float_cmp` stay `deny`; all public trait items documented; every scenario-
   reachable path returns `Result`, never `panic` (the trait surfaces above are
   all fallible).
10. **Requirements traceability.** Each WP adds `requirements.toml` entries
    (e.g. `REQ-PROP-PRESSURE-THRUST`, `REQ-PROP-THERMOCHEM-DECK`,
    `REQ-PROP-POGO-METHOD`) with verification evidence;
    `check_requirements_traceability.py` rejects orphans.

---

## 5. V&V plan

Per the five-layer ladder (`docs/verification.md`) and the solver-consumer rule
(`13` §6). Each tier earns the label in §3.1 only when its row's evidence lands.

| Tier | MMS / analytic | Code-to-code | Public benchmark | Tolerance (target) | Label |
|---|---|---|---|---|---|
| **T1** | optimum-CF reduces to current code at `pₐ=pₑ` (regression); Joukowsky-free analytic vacuum ceiling `F_vac = CF_vac·pc·At` | — | published sea-level vs vacuum thrust/Isp (RS-25, RL10, Merlin 1D public figures, F-1) — match the **altitude lift** trend | thrust/Isp shape within published band; vacuum/SL ratio within ~2–5% | `validated-toy` |
| **T2** | analytic constant-`Kn` end-burner gives near-constant `pc` (extends existing test `grain.rs:656`); MMS on the `pc(t)` ODE (impose analytic forcing) | OpenMotor BATES/finocyl `Kn(web)`, `pc(t)`, thrust | thrustcurve.org certified static-fire curves (e.g. Cesaroni M1670 already pinned); Stark/Frey separation-location vs NPR | `pc(t)` and total impulse within a few % of OpenMotor; separation onset within published scatter | `validated-toy`→`research` |
| **T3** | `c*` from `Tc, γ, MW` matches the closed-form `c* = sqrt(R_sp·Tc)/Γ(γ)` for the deck corners | (optional) in-repo min-G tier vs CEA on a shared case | CEARUN `c*, Tc, Isp_vac, Isp_sl` for LOX/RP-1 (MR ~2.3–2.7), LOX/LH2 (MR ~5–6), NTO/MMH, LOX/LCH4 at `pc`=1–30 MPa, `ε`=10–100 | `c*`/`Isp` within **<1%** of CEARUN (ideal); efficiency band documented separately | `research` |
| **T4** | MMS on the network Newton residual + the MOC grid (impose analytic transient, confirm order); Joukowsky bound for instantaneous closure | **GFSSP V6 manual worked cases** (blowdown tank, flow-with-heat-transfer, pressurization) — match node pressures / branch flows | Wylie & Streeter single-pipe valve-closure surge | node pressures / branch flows within a few % of GFSSP; surge within ~5% of Joukowsky | `validated-toy` (network) / `research` (vs GFSSP) |
| **T5** | MMS on the linear closed-loop assembly (impose a known transfer function, confirm eigenvalues) | — | **Saturn V S-II / Titan II POGO case histories** (Rubin 1966, SP-8055) — qualitative match of the **unstable frequency band** and the accumulator-stabilized boundary | frequency band qualitatively matched; **no amplitude/point claim** (ceiling §1.2) | `research` (method-level) |

**Tolerance tables.** Every numeric claim ships a tolerance-table TOML under the
WP's `tests/expected/`, declaring the reference value, source (CEARUN run id /
GFSSP case / OpenMotor config / published datasheet), tolerance, and the
validation label it justifies. The byte-diff determinism gate runs on top of
all of these.

**What is explicitly not validated** (carried in each WP's `parity_ceiling`):
absolute `η_c*` for any real engine; real turbopump map shapes; POGO limit-cycle
amplitude; 3D plume/base-heating fields; and any "this matches vehicle X's flight
data" claim.

---

## 7. Dependencies on other parity docs

- **`02-structural-dynamics-loads-slosh-pogo.md`** — **hard dependency for T5.**
  POGO needs the vehicle's first longitudinal structural mode (`LongitudinalModalModel`:
  frequency, modal mass, engine-station gain). The DAG (`00` §5, Phase D) pairs
  `05 T5` with `02 T4`; build the feed half here, the structural half there, and
  the closed-loop assembly is the shared capstone. This doc owns `G_feed(s)`;
  `02` owns `G_struct(s)`.
- **`08-environment-gravity-and-frames.md`** — T1 consumes `pₐ(h)` from the
  atmosphere models in `openbmp-physics` (already present:
  `USSA76_SEA_LEVEL_PRESSURE_PA`, the US-Standard-1976 / piecewise-exponential
  pressure surfaces). No new dependency; just wire `pₐ(h)` through the thrust
  call.
- **`11-monte-carlo-uq-and-validation.md`** — the efficiency band, turbopump-map
  uncertainty, and cavitation dispersion box flow into Monte Carlo through
  `openbmp-uq`/`openbmp-mc`. Burn-rate dispersion (`a, n, ρ_p`) for off-nominal
  Isp/burn-time is an MC input (T2).
- **`04-aerothermal-realgas-and-tps.md`** — shares `openbmp-thermochem`
  (equilibrium-air curve fits + CEA/Cantera ingestion). Coordinate the deck
  schema so the crate serves both real-gas air and combustion products. Plume/
  base-heating reduced models (out of primary scope here) would couple to `04`.
- **`09-sensors-navigation-and-actuators.md`** — a chamber-pressure transducer /
  feed-pressure sensor model is the *correct* way to expose feed state to the FC
  (preserving invariant 3), not a direct import.
- **`06-gnc-coupled-mimo-and-control.md`** — the CSI/bending side of the
  flight-control loop (notch filters, gain/phase margins) lives there; POGO is
  the longitudinal analog and shares the linear-systems tooling.
- **`12-determinism-realtime-and-compute.md`** — the Newton/eigen solvers must
  satisfy the determinism contract; coordinate the `faer`-based deterministic
  linear-algebra choice.

---

## 8. Open-source leverage

| Tool | License | Mode | Use |
|---|---|---|---|
| **NASA CEA** (modernized, github.com/nasa/cea) | US-Gov public domain | **INGEST** | Run offline over a `(pc, MR, ε)` grid per propellant pair → `c*, Tc, γ, MW, Isp_vac/sl` decks; ship table + interpolator in Rust. **Do not FFI-link the Fortran into the flight loop.** |
| **Cantera** (cantera.org) | BSD-3-Clause | **INGEST / PORT-ref** | Alternative equilibrium source (`equilibrate('HP'/'TP')`); its NASA9 thermo DB is the source of the 9-coeff polynomials if porting a reduced min-G tier. Generates the same decks for cross-check. |
| **RocketCEA / RoCt** | BSD-style / BSD-3 | **INGEST helper** | Python wrappers to batch-generate the T3 decks. Build-time tooling only, not a runtime dependency. |
| **GFSSP** (NASA) | request-access / US-only | **REFERENCE / V&V oracle** | The authoritative code-to-code comparison for the T4 feed network (node pressures, branch flows, transient response) and the source of the finite-volume node/branch *method* to re-implement. **Not redistributable — re-implement the published algorithm, do not vendor.** |
| **OpenMotor** (github.com/reilleyz/openMotor) | GPL-3.0 | **REFERENCE / V&V** | Validate T2 transient solid ballistics (`Kn(web)`, `pc(t)`, erosive). **GPL — run for validation, do not copy code; re-implement from the textbook.** |
| **SU2** (su2code.github.io) | LGPL-2.1 | **COUPLE-offline / REFERENCE** | Compressible RANS for nozzle separation onset and `CF(pₐ)` calibration of the T2 algebraic criteria. Offline reference generator → no obligation on OpenBMP. |
| **OpenFOAM** (rhoCentralFoam/reactingFoam) | GPL-3.0 | **COUPLE-offline / REFERENCE** | Plume, base recirculation, underexpanded-jet structure → fit reduced engineering models. Offline only; do not link. |
| **SPARTA** (Sandia) | GPL-3.0 | **COUPLE-offline / REFERENCE** | DSMC for high-altitude/vacuum rarefied plume where continuum fails. Offline reference fields only. |
| **CoolProp** (coolprop.org) | MIT | **INGEST / COUPLE** | Real-fluid Helmholtz EOS for cryogenic propellant density / vapor pressure / NPSH (LOX, LH2, LCH4, RP-1 surrogate) feeding T4/M7. Fully permissive — safe to link or port correlations. |
| **OSQP / pure-Rust QP** | Apache-2.0 | **COUPLE (optional)** | If throttle/MR control or the feed steady-solve is posed as constrained optimization; prefer the permissive path and reuse the existing `openbmp-trajopt` solver where possible. |
| **faer-rs** | MIT/Apache-2.0 | **USE (pure Rust)** | Dense/sparse linear algebra for the Newton Jacobian solve (M5), the closed-loop complex eigenproblem (M8), keeping the deterministic-f64 contract without C/Fortran FFI. |

License discipline: permissive tools (Cantera, CoolProp, OSQP, faer) may be
linked or ported with attribution; copyleft tools (OpenMotor, OpenFOAM, SPARTA,
SU2) are used as **external offline references** only — their *generated data* is
free to use, but their *code* never enters the MIT/Apache OpenBMP workspace.
Confirm each repo's `LICENSE` before vendoring any generated decks.

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, each green on the full §2 gate set.
Effort sizes are from the research ladder.

---

**WP-05.1 — Add pressure-thrust + altitude term**
- **status:** implemented in source and traced by `REQ-PROP-001` / `V-PROP-001`.
- **goal:** Replace the optimum-CF assumption with the full ambient-corrected
  thrust `F = ṁVₑ + (pₑ−pₐ)Aₑ`, consuming the stored `exit_area_m2` and `pₐ(h)`.
  Fixes the known 5–15% vacuum-thrust error and makes staging altitudes physical.
  Highest value/effort ratio in the program (`13` §7).
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none
- **touched:** `crates/openbmp-propulsion/src/grain.rs` (generalize
  `optimum_thrust_coefficient` → ambient-aware), `src/motor.rs` (extend
  `AmbientPressureCorrection` with a `PressureThrust` variant; `NozzlePerformance`
  trait + `ChamberState`/`NozzleSolution`); `crates/openbmp-vehicle/adapters`
  (pass `pₐ(h)` from the atmosphere model into the thrust adapter); scenario block
  `[propulsion.nozzle]`.
- **approach:** §3.3 **(M1)**. Reuse `supersonic_mach_for_area_ratio` and the
  isentropic relations verbatim; the optimum case is the `pₐ=pₑ` special case.
- **acceptance:**
  - New `PressureThrust` correction off by default
    (`AmbientPressureCorrection::Constant` remains the default); canonical
    goldens byte-identical (CI byte-diff gate).
  - At `pₐ = pₑ`, ambient-CF equals the current `optimum_thrust_coefficient` to
    machine precision (regression test).
  - Vacuum-vs-sea-level thrust/Isp ratio for a public engine (e.g. RL10/Merlin
    public figures) matches the published band in a tolerance table.
  - All §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line (plant physics only).
- **est_effort:** 1–2 days.
- **parity_ceiling:** does not validate absolute thrust for any real engine; only
  the altitude-lift trend and the `pₐ=pₑ` regression.

---

**WP-05.2-a — Nozzle flow separation (overexpansion clipping)**
- **status:** implemented for Summerfield and Schmucker criteria and traced by
  `REQ-PROP-002` / `V-PROP-002`; Kalt-Badal remains a possible later extension.
- **goal:** Add Summerfield/Schmucker/Kalt–Badal separation criteria so a
  heavily overexpanded sea-level start does not over-penalize thrust.
- **fidelity_tier:** T2
- **depends_on:** [WP-05.1]
- **new_crates:** none
- **touched:** `src/grain.rs`/`src/motor.rs` (`SeparationCriterion` enum, inner
  isentropic march for `ε_sep`); `[propulsion.nozzle]` gains
  `separation = "schmucker" | "summerfield" | "off"` (default `off`).
- **approach:** §3.3 **(M2)**. Scalar inner march to `ε_sep` where
  `p_wall(ε_sep) = p_sep`; substitute `ε_sep`, `p_sep` into the pressure term.
- **acceptance:**
  - Off by default; goldens byte-identical.
  - Separation-onset NPR for a cold-flow nozzle (Frey & Hagemann data) within
    the published scatter in a tolerance table.
  - All §2 gates green.
- **validation_label:** `checked` → `validated-toy` (with the Frey/Hagemann case).
- **dual_use_note:** far from line.
- **est_effort:** 2–4 days.
- **parity_ceiling:** algebraic criteria only; no RANS separation field (SU2 is
  an offline calibration reference, not a runtime solver).

---

**WP-05.2-b — Transient solid internal ballistics (pc(t) ODE + erosive burning)**
- **status:** source path implemented as opt-in `mode = "transient"` and traced by
  `REQ-PROP-003` / `V-PROP-003` for deterministic ODE integration, constant-Kn
  settling, MMS order evidence, and the reduced erosive-burn-rate hook. OpenMotor
  and static-fire tolerance-table validation remain future evidence before
  raising beyond the current synthetic/toy level.
- **goal:** Replace the fixed-web quasi-static march with a lumped-volume
  `pc(t)` ODE: ignition transient, tail-off, erosive (Lenoir–Robert) augmentation,
  `Kn(web)` from grain geometry.
- **fidelity_tier:** T2
- **depends_on:** [WP-05.1]
- **new_crates:** none
- **touched:** `src/grain.rs` (`TransientChamber` impl for solids; integrate
  `pc(t)`); `[propulsion.motor.grain]` gains
  `mode = "transient" | "quasi_static"` (default `quasi_static`).
- **approach:** §3.3 **(M3)**. Keep the steady `pc(Kn)` as the equilibrium
  reference; integrate the gas mass balance explicitly in locked operand order.
  Burn-rate dispersion `(a,n,ρ_p)` is an MC input (doc 11).
- **acceptance:**
  - Off by default; quasi-static path byte-identical to current.
  - Constant-`Kn` end-burner gives near-constant `pc` (extends `grain.rs:656`).
  - MMS on the `pc(t)` ODE recovers the imposed analytic solution to integration
    order.
  - BATES/finocyl `Kn(web)`, `pc(t)`, thrust within a few % of OpenMotor (run
    OpenMotor for the numbers — re-implemented, not copied); tolerance table +
    provenance.
  - Cesaroni M1670 pinned curve (`tests/cesaroni_m1670_pin.rs`) still passes.
  - All §2 gates green.
- **validation_label:** `validated-toy` → `research` (with OpenMotor + static-fire).
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks.
- **parity_ceiling:** does not validate combustion-instability margins or a
  specific propellant's measured burn-rate law.

---

**WP-05.3 — Thermochemistry deck ingestion (`openbmp-thermochem`)**
- **status:** initial L2 crate, Schema-1 parser, closed-form `c*`
  consistency check, log-`pc` × mixture-ratio interpolation, and fail-closed
  lookup behavior implemented and traced by `REQ-PROP-004` / `V-PROP-004`;
  inline solid-grain scenario/runner consumption and reduced feed-network
  chamber-constant consumption through `[propulsion.thermochem]` are
  implemented and traced by `REQ-PROP-005` / `V-PROP-005`. The
  `LiquidEnginePerformance` helper in `openbmp-propulsion` now derives
  liquid-engine max thrust, choked mass flow, and effective `Isp` from a
  looked-up thermochemical state plus the pressure-thrust nozzle solver
  (`REQ-PROP-028` / `V-PROP-028`), and schema-v3 liquid engines can opt into
  that path through `thermochemical_performance` while reusing the SHA-pinned
  `[propulsion.thermochem]` deck lookup (`REQ-PROP-029` / `V-PROP-029`).
  Schema-1 state rows now carry an optional empirical `c_star_efficiency`
  band; the deck interpolator preserves it and the liquid-engine bridge uses
  its nominal value for deterministic mass flow/`Isp` while retaining min/max
  mass-flow and `Isp` envelope evidence. A first provenance-recorded Cantera
  3.2.0 `gri30.yaml` LOX/LCH4 reference deck and tolerance table now checks
  deck lookup plus liquid-engine mass-flow/thrust/`Isp` against independently
  generated values (`REQ-PROP-030` / `V-PROP-030`). The full CEARUN/Cantera
  tolerance-table matrix for LOX/RP-1, LOX/LH2, NTO/MMH, and broader LOX/LCH4
  coverage remains future evidence before this WP is complete.
- **goal:** Derive `c*(pc,MR)`, `Tc`, `γ`, `MW` from a CEA/Cantera deck instead
  of hardcoded constants; enable mixture-ratio-aware liquid performance and the
  documented efficiency band.
- **fidelity_tier:** T3
- **depends_on:** [WP-05.1]
- **new_crates:** `openbmp-thermochem` (L2). First commit = skeleton + placement
  justification (depends only on `openbmp-core`/`openbmp-models` + math; no FC
  edge).
- **touched:** new crate (`ThermochemDeck` trait, schema, parser, bilinear
  interpolator, optional reduced min-G tier); `src/grain.rs` consumes
  `ChamberState` constants from the deck; `src/engine.rs` exposes the
  thermochemical liquid-performance bridge; `data/thermochem/<pair>/` +
  `provenance.md`; `[propulsion.thermochem]` scenario block.
- **approach:** §3.3 **(M4)**. CEA/Cantera offline → table; bilinear interp in
  `log pc` × `MR` with locked corner order. Delivered `c* = c*_ideal·η_c*` from
  the band, propagated by `openbmp-uq`.
- **acceptance:**
  - Off by default (constant-`c*` path preserved); goldens byte-identical.
  - Deck SHA-pinned; `openbmp check-provenance` green; no inline TOML in `*.rs`.
  - `c*`/`Isp` within **<1%** of CEARUN for LOX/RP-1, LOX/LH2, NTO/MMH, LOX/LCH4
    across the deck grid (tolerance table).
  - Closed-form `c* = sqrt(R_sp·Tc)/Γ(γ)` matches the deck corners.
  - Out-of-envelope queries fail closed (no extrapolation).
  - All §2 gates green.
- **validation_label:** `research`.
- **dual_use_note:** far from line (offline physics-constant generation).
- **est_effort:** ~2–4 weeks (ingest path); +2–4 weeks if the reduced min-G tier
  is built.
- **parity_ceiling:** ideal-CEA ceiling only; absolute `η_c*` for a real engine
  is the band, not a number.

---

**WP-05.4-a — Feed network + transient chamber (`openbmp-feedsystem`)**
- **status:** initial L2 crate, `FeedNetwork` trait, typed error surface, and a
  deterministic single-fluid tank-valve-chamber equilibrium solve are
  implemented and traced by `REQ-PROP-006` / `V-PROP-006`. Opt-in
  `[[propulsion.feed_network]]` scenario parsing and runner-side feed-scale
  override for tank-coupled liquid engines are implemented and traced by
  `REQ-PROP-007` / `V-PROP-007`. A steady node/branch graph with fixed-pressure,
  junction, and chamber nodes, signed valve branches, and fixed-cap Newton mass
  balance is implemented and traced by `REQ-PROP-008` / `V-PROP-008`. A lumped
  transient chamber-pressure stepper with inlet/outlet mass-flow and optional
  mixture-ratio reporting is implemented and traced by `REQ-PROP-009` /
  `V-PROP-009`. A bounded dual-valve throttle/MR controller primitive is
  implemented and traced by `REQ-PROP-010` / `V-PROP-010`. A stateful transient
  dual-valve feed network coupling oxidizer/fuel tank-valve legs to the chamber
  pressure stepper is implemented and traced by `REQ-PROP-011` / `V-PROP-011`.
  Scenario parsing and runner-side chamber-state advancement for that transient
  network are implemented and traced by `REQ-PROP-014` / `V-PROP-014`.
  Scenario-facing pressure/MR controller wiring for transient feed networks is
  implemented and traced by `REQ-PROP-015` / `V-PROP-015`. A synthetic
  provenance-backed steady feed-network tolerance table now covers the reduced
  tank-valve-chamber balance and the two-valve node/branch ladder through
  `data/feed_system/generic-steady-feed-network-v1.toml`; GFSSP worked-case
  evidence remains future work before this WP is complete.
- **goal:** GFSSP-style finite-volume network (tanks → valves → injector →
  chamber) producing transient `pc(t)`, `ṁ_ox/ṁ_fuel`, MR excursions, and
  throttle response; replaces the quasi-static blowdown scalar.
- **fidelity_tier:** T4
- **depends_on:** [WP-05.3]
- **new_crates:** `openbmp-feedsystem` (L2). First commit = skeleton + placement
  justification.
- **touched:** new crate (`FeedNetwork` trait, node/branch graph, Newton solve,
  choked-throat closure, PID throttle + MR control); `crates/openbmp-vehicle`
  coupling adapter (replace the `feed_pressure_scale` scalar path with the network
  when `[propulsion.feed_network]` is present); deprecate-in-place the
  `propellant_budget.rs` quasi-static blowdown when the network is enabled.
- **approach:** §3.3 **(M5)**. Start with a 3-node ladder (tank–valve–chamber)
  before generalizing the graph; fixed Newton iteration cap; locked assembly.
- **acceptance:**
  - Off by default; existing blowdown path byte-identical when the network block
    is absent.
  - GFSSP V6 worked cases (blowdown tank, flow-with-heat-transfer, pressurization)
    matched on node pressures / branch flows within a few % (code-to-code table +
    provenance).
  - MMS on the Newton residual confirms convergence; non-convergence fails closed.
  - All §2 gates green.
- **validation_label:** `validated-toy` (network) → `research` (vs GFSSP cases).
- **dual_use_note:** far from line.
- **est_effort:** 3–6 weeks.
- **parity_ceiling:** not a GFSSP-validated model for a specific vehicle; the
  comparison is to GFSSP *example problems*, not flight data.

---

**WP-05.4-b — Turbopump map + affinity + NPSH**
- **status:** generic normalized turbopump map, affinity-law pressure scaling,
  speed-scaled NPSH gate, cavitation head derating, efficiency/power reporting,
  and specific-speed metadata are implemented in `openbmp-feedsystem` and traced
  by `REQ-PROP-012` / `V-PROP-012`; a provenance-backed synthetic tolerance
  table under `data/feed_system/` now covers affinity scaling, NPSH thresholding,
  cavitation derating, efficiency, and power. Scenario-facing turbopump blocks
  for transient feed-network legs are implemented and traced by `REQ-PROP-016`
  / `V-PROP-016`. Public real-pump calibration remains future work before this
  WP is complete.
- **goal:** Set pump-fed feed pressure from a normalized turbopump map + affinity
  laws + NPSH cavitation gate, feeding cavitation compliance into POGO.
- **fidelity_tier:** T4
- **depends_on:** [WP-05.4-a]
- **new_crates:** none (extends `openbmp-feedsystem`).
- **touched:** `openbmp-feedsystem` (pump-rise branch, `ψ(φ)` map, affinity, `Ns`
  family, `NPSH_avail/req`); CoolProp-derived vapor-pressure/density inputs;
  `[propulsion.turbopump]` block.
- **approach:** §3.3 **(M7)**. Generic normalized map keyed by `Ns`, calibrated
  to the published design point; cavitation = `ψ` cliff below `NPSH_req`.
- **acceptance:**
  - Off by default; goldens byte-identical.
  - Operating point `H_pump(Q,N)=H_system(Q)` solved deterministically; affinity-
    law scaling verified analytically (tolerance table).
  - NPSH gate triggers the cavitation state at the documented margin.
  - All §2 gates green.
- **validation_label:** `validated-toy` (map shape is the generic substitute).
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks.
- **parity_ceiling:** real proprietary pump maps; only the generic `Ns`-keyed
  shape calibrated to a published design point.

---

**WP-05.4-c — Water-hammer / line transients (Method of Characteristics)**
- **status:** a frictionless fixed-grid MOC line primitive with Courant-exact
  step sizing, upstream fixed-head/downstream valve-velocity boundaries, and a
  Joukowsky valve-closure verification case is implemented in
  `openbmp-feedsystem` and traced by `REQ-PROP-013` / `V-PROP-013`; a
  provenance-backed synthetic tolerance table under `data/feed_system/` now
  covers Courant step size, downstream surge head/pressure, and the Joukowsky
  pressure/head delta.
  Scenario-facing `oxidizer_line` / `fuel_line` blocks for
  `transient_dual_valve_chamber` feed-network legs are implemented and traced
  by `REQ-PROP-017` / `V-PROP-017`; the runner enforces the line Courant time
  step and applies downstream pressure perturbations to the transient
  tank-valve-chamber update. Richer boundary-condition library, standalone
  `[propulsion.lines]` topology wiring, and public water-hammer benchmark
  tables remain future work before this WP is complete.
- **goal:** Capture acoustic/inertial line surges on valve open/close and engine
  start — the physical substrate of POGO and the accumulator-sizing driver.
- **fidelity_tier:** T4
- **depends_on:** [WP-05.4-a]
- **new_crates:** none (extends `openbmp-feedsystem`).
- **touched:** `openbmp-feedsystem` (MOC fixed-grid solver, boundary-condition
  library: valve, tank, pump, accumulator); `[propulsion.lines]` block.
- **approach:** §3.3 **(M6)**. Explicit fixed-grid MOC at Courant `a·Δt/Δx = 1`;
  boundary nodes supply the missing characteristic.
- **acceptance:**
  - Off by default; goldens byte-identical.
  - Wylie & Streeter single-pipe valve-closure surge within ~5% of the Joukowsky
    bound `ΔH = a·ΔV/g` (tolerance table).
  - MMS on the MOC grid confirms order.
  - All §2 gates green.
- **validation_label:** `validated-toy` → `research`.
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks.
- **parity_ceiling:** 1-D line model only; no 3-D priming/cavitating-front CFD.

---

**WP-05.5-a — POGO closed-loop stability (feed half of the capstone)**
- **status:** a reduced deterministic POGO stability primitive is implemented in
  `openbmp-feedsystem` and traced by `REQ-PROP-018` / `V-PROP-018`. It couples
  one longitudinal mode to a first-order feed response, applies cavitation-to-
  accumulator compliance attenuation, evaluates the cubic Routh-Hurwitz margins,
  reports static/dynamic instability verdicts, and exposes a neutral accumulator
  compliance boundary; a provenance-backed synthetic tolerance table under
  `data/feed_system/` now covers stable, static-divergent, dynamically
  unstable, accumulator-detuned, and compliance-attenuated cases.
  Scenario-facing `[propulsion.pogo]` parsing and runner startup gating are
  implemented and traced by `REQ-PROP-019` / `V-PROP-019`. Full
  transfer-matrix assembly, structural modal-data consumption, and public Saturn
  V/Titan case-history evidence remain future work before this WP is complete.
- **goal:** Assemble `G_feed(s)` from the linearized feed network + pump
  cavitation compliance/mass-flow-gain, couple to the longitudinal structural
  mode from `02`, and run complex-eigenvalue / Nyquist stability + accumulator
  sizing.
- **fidelity_tier:** T5
- **depends_on:** [WP-05.4-a, WP-05.4-b, WP-05.4-c, (doc 02 longitudinal modal model)]
- **new_crates:** none (extends `openbmp-feedsystem` with `PogoStability`).
- **touched:** `openbmp-feedsystem` (transfer-matrix/state-space assembly, complex
  eigen/Nyquist via `faer`); consumes `LongitudinalModalModel` (doc 02) as plain
  data; `[propulsion.pogo]` block carrying the cavitation dispersion box.
- **approach:** §3.3 **(M8)**. Linear transfer-function stability check first;
  time-domain limit cycle second. SP-8055 margin criteria.
- **acceptance:**
  - Off by default; goldens byte-identical.
  - MMS on the closed-loop assembly (imposed transfer function → known
    eigenvalues).
  - Saturn V S-II / Titan II unstable frequency band qualitatively matched;
    accumulator compliance `C_acc` detunes the resonance and flips the verdict
    (method-level, **no amplitude claim**).
  - Cavitation parameters entered as a documented dispersion box, never fitted to
    a target.
  - All §2 gates green.
- **validation_label:** `research` (method-level only).
- **dual_use_note:** far from line; self-referential vehicle dynamics only.
- **est_effort:** 4–8 weeks (the hardest item).
- **parity_ceiling:** `C_b`/`M_b` are the least-known parameters in rocketry —
  only the frequency band and the suppression *effect* are claimed, never a
  specific vehicle's margin.

---

**WP-05.5-b — Engine-out & fault library (deterministic fault injection)**
- **status:** a deterministic `[propulsion.faults]` scheduled engine-fault
  block is implemented and traced by `REQ-PROP-020` / `V-PROP-020`. It validates
  ordered one-shot rules, rejects unknown engine ids, routes scheduled faults
  through `EngineCluster::inject_fault`, and applies them in both point-mass and
  rigid-body runner loops before the engine rack steps. A deterministic
  ignition-only hard-start over-pressure fault is implemented and traced by
  `REQ-PROP-021` / `V-PROP-021`. Pump-cavitation-triggered one-shot fault rules
  are implemented and traced by `REQ-PROP-022` / `V-PROP-022`; the runner latches
  feed-network pump cavitation events into configured engine faults.
  Mixture-ratio runaway feed-network valve drift is implemented and traced by
  `REQ-PROP-023` / `V-PROP-023`. Native SIL package stimulation can now append
  scheduled propulsion engine-fault rules in memory, record them in evidence,
  and observe the resulting engine telemetry response (`REQ-PROP-024` /
  `V-PROP-024`). `openbmp-mc` now has deterministic scheduled-propulsion fault
  libraries that materialize parser-validated `[propulsion.faults]` overlays for
  sampled cases (`REQ-PROP-025` / `V-PROP-025`). Native SIL observed runs can
  now combine stimulation with the EKF estimate-vs-truth monitor, and the
  shortened Phalcon TVC probe records an engine-out response through thrust,
  velocity, and EKF residual evidence (`REQ-PROP-026` / `V-PROP-026`). The
  `openbmp mc propulsion-faults` CLI path now executes deterministic sampled
  overlays through the normal scenario parser and runner, writes one sample CSV
  row per terminal metric, and reports Welford plus Clopper-Pearson summaries
  (`REQ-PROP-027` / `V-PROP-027`). The remaining ceiling is calibrated failure
  probabilities and vehicle-specific FMEA data, not the fault-execution
  mechanism.
- **goal:** Discrete fault modes (engine-out, valve stuck/leaking, cavitation,
  hard-start over-pressure, MR runaway, sensor bias) injected at the FC/sensor
  boundary into the SIL/EKF loop and Monte Carlo.
- **fidelity_tier:** T5
- **depends_on:** [WP-05.4-a]
- **new_crates:** none
- **touched:** `src/engine.rs` (extend `EngineFault` taxonomy + a typed
  scheduled-event list), `crates/openbmp-runner` (fault-event scheduler reusing
  the commit-`1a88b1d` FC-boundary fault-injection harness),
  `[propulsion.faults]` scenario block, `crates/openbmp-mc` (sampled
  fault-library overlays), `crates/openbmp-cli` (campaign execution over
  sampled overlays).
- **approach:** §3.3 **(M9)**. FSM per engine; condition-triggered faults (e.g.
  `NPSH < NPSH_req`); cluster net thrust/torque recomputed; GNC responds. Unknown
  fault types fail closed.
- **acceptance:**
  - Off by default; goldens byte-identical with no fault block.
  - Engine-out recomputes cluster net thrust + torque about CG; a scenario
    exercises the GNC response (test-through-a-scenario, `13` §3).
  - SIL package stimulation records scheduled engine faults without modifying
    package source files.
  - MC fault-library samples materialize scheduled propulsion-fault overlays
    through the normal scenario parser.
  - MC propulsion-fault campaigns execute sampled overlays through the normal
    runner, write sample CSV rows, and summarize terminal metric/success
    statistics.
  - A closed-loop SIL/EKF scenario records the propulsion-fault plant response
    and EKF observation evidence.
  - Unknown fault type rejected fail-closed at parse.
  - Faults reproducible bit-for-bit (deterministic, no system RNG).
  - All §2 gates green.
- **validation_label:** `validated-toy` (mechanism), faults exercised in MC.
- **dual_use_note:** far from line — faults never read/act on a target/aimpoint
  (the active guardrail in §6).
- **est_effort:** 3–5 days (reuses the existing SIL harness).
- **parity_ceiling:** fault *mechanisms*, not validated failure probabilities or
  a specific vehicle's FMEA.

---

## 10. References

**Pressure-thrust & nozzle (M1/M2):**
- Sutton, G.P. & Biblarz, O. *Rocket Propulsion Elements*, 9th ed., Ch.3
  (thrust coefficient, eqs 3-30..3-34) and Ch.5 (nozzle theory).
- NASA SP-8120, *Liquid Rocket Engine Nozzles*.
- Anderson, J.D. *Modern Compressible Flow*, 3rd ed., Ch.5 (quasi-1D nozzle).
- Stark, R., "Flow Separation in Rocket Nozzles — An Overview," AIAA/DLR.
- Schmucker, R., "Flow Processes in Overexpanded Chemical Rocket Nozzles,"
  NASA TM-77396.
- Frey, M. & Hagemann, G., "Restricted Shock Separation in Rocket Nozzles,"
  *J. Propulsion & Power* 16(3), 2000.

**Thermochemistry (M4):**
- Gordon, S. & McBride, B.J., NASA RP-1311 Part I (theory, 1994) & Part II
  (manual, 1996).
- NASA CEA modernized release (github.com/nasa/cea); CEARUN (cearun.grc.nasa.gov).
- Cantera (cantera.org), BSD-3-Clause.

**Solid ballistics (M3):**
- Sutton & Biblarz, Ch.11–12 (solid propellant rockets, grain design, burn rate).
- Lenoir, J.M. & Robert, G. (1956), erosive burning correlation.
- NASA SP-8039, *Solid Rocket Motor Performance Analysis and Prediction*.
- OpenMotor (github.com/reilleyz/openMotor), GPL-3.0 (validation reference only).
- thrustcurve.org open static-fire database (e.g. Cesaroni M1670).

**Feed network & transients (M5/M6/M7):**
- Majumdar, A., "Generalized Fluid System Simulation Program (GFSSP) Version 6,"
  NASA/TP (NTRS 20150016531); GFSSP finite-volume procedure (NASA 2020).
- Sutton & Biblarz, Ch.6 (liquid feed systems), Ch.10 (turbopump feed).
- Wylie, E.B. & Streeter, V.L., *Fluid Transients in Systems*, Prentice-Hall 1993.
- Chaudhry, M.H., *Applied Hydraulic Transients*, 3rd ed., Springer 2014.
- Joukowsky, N. (1900), surge equation `ΔP = ρ·a·ΔV`.
- Brennen, C.E., *Hydrodynamics of Pumps*, Oxford/Concepts ETI, 1994.
- Stepanoff, A.J., *Centrifugal and Axial Flow Pumps* (affinity laws, `Ns`).
- NASA SP-8052, *Liquid Rocket Engine Turbopump Inducers*.
- CoolProp (coolprop.org), MIT.

**POGO (M8):**
- Rubin, S., "Longitudinal Instability of Liquid Rockets due to Propulsion
  Feedback (POGO)," *J. Spacecraft & Rockets* 3(8), 1966.
- Rubin, S., NASA SP-8055, *Prevention of Coupled Structure-Propulsion
  Instability (POGO)*, 1970 (ntrs.nasa.gov/citations/19710016604).
- Oppenheim, B.W. & Rubin, S., "Advanced Pogo Stability Analysis for Liquid
  Rockets," *J. Spacecraft & Rockets* 30(3), 1993.
- Brennen, C.E. & Acosta, A.J., cavitation compliance / mass-flow-gain factor.
- Zhao, et al., "Improved modelling method of Pogo analysis," *Acta Astronautica*
  2015.

**Faults (M9):**
- Sutton & Biblarz, Ch.6/11 (failure modes); NASA-STD-8729; NASA SP-8055.
- Patton, Frank & Clark, *Fault Diagnosis in Dynamic Systems*, 1989 (the
  effector-fault taxonomy `EngineFault` already mirrors, `engine.rs:288`).
- OpenBMP SIL FC-boundary fault-injection pattern (commit `1a88b1d`).

**Solver verification:**
- Method of Manufactured Solutions for the MOC and the feed-network Newton solve
  (impose a known analytic transient; confirm convergence order) — per
  `docs/verification.md`.

---

*Companion documents:* `00-overview.md` (constitution) ·
`13-agent-execution-playbook.md` (process) ·
`02-structural-dynamics-loads-slosh-pogo.md` (POGO structural half) ·
`04-aerothermal-realgas-and-tps.md` (shared `openbmp-thermochem`) ·
`08-environment-gravity-and-frames.md` (`pₐ(h)`) ·
`11-monte-carlo-uq-and-validation.md` (efficiency bands / dispersion into MC).
