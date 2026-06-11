# Plume-Induced Environments & Supersonic Retropropulsion

**Status:** `experimental` (design intent plus partial WP-15.1 implementation;
runner plume telemetry remains diagnostic-only).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**One-line summary:** give OpenBMP the plume discipline that docs `03`/`04`/`05`
all defer to each other — live plume similarity state (NPR, C_T) computed from
engine state, power-on base pressure/drag and base convective+radiative heating,
plume-induced flow separation onset, multi-engine plume merge, a RAMP2/PLIMP-class
impingement capability with rarefied/DSMC handoff, plume-surface interaction for
landing burns, and a supersonic-retropropulsion increment model anchored to
NASA's open FUN3D SRP datasets — all as a solver-consuming engineering tier,
never a from-scratch CFD code.

> Read `00-overview.md` (parity definition, invariants, DAG) and
> `13-agent-execution-playbook.md` (the §4 template this doc follows and the §5
> work-package schema) before this document. This is dimension `15`, added after
> the original eleven-dimension audit to close the deferral triangle the audit
> left open: doc `03` §1.2 item 2 defers plume/base coupling to docs `05` and
> `04`; doc `04` §7 calls it "a shared parity-ceiling item"; doc `05` §1.2 item 4
> defers the 3D plume/base field to offline reference runs "feeding a reduced
> engineering plume/base model" that no document designs. This document designs
> it. Tiers T1–T3 ride **Phase B** beside `03`/`04`/`05`; T4–T6 are Phase C/D
> integration work.

**Gaps this document closes** (extending the `00` §2 ledger):

1. Plume flowfield and plume–vehicle interaction suite (discipline).
2. Ascent plume effects: power-on base drag/heating, PIFS, multi-engine cluster
   (lifecycle).
3. Supersonic retropropulsion aerodynamics (discipline).
4. Supersonic retropropulsion and descent plume–aero interaction (lifecycle).

Item 4 is not optional polish: OpenBMP **already flies** retro burns in
atmosphere — the Phalcon-9 boostback/entry-burn demo
(`scenarios/phalcon9/phalcon9-orbit-boostback.toml`, with `landing_burn` mission
regions at line 340 and in `scenarios/phalcon9/mission-graph-boostback.toml:218`)
sums full power-off aerodynamic drag *plus* full retrograde thrust, because the
engine and aero force adapters are independent (§2). Real supersonic
retropropulsion collapses the aerodynamic drag as the plume displaces the bow
shock. Without this dimension, every powered-descent phase the project already
demonstrates is physically wrong in atmosphere.

---

## 1. Parity target & ceiling

### 1.1 Target (capability / method parity within posture)

Production launch-vehicle simulators do not solve the plume flowfield in the
trajectory loop. They carry **power-on delta-decks and induced-environment
correlations keyed to engine state**, built offline from CFD + test, and they
maintain a dedicated plume-impingement toolchain (the NASA PLIMP/RAMP2 lineage,
today Loci/CHEM-class CFD) for staging and proximity events:

- **SLS base heating** was developed as short-duration testing at CUBRC LENS II
  plus a correlation methodology that maps test conditions to flight (NTRS
  20150002948, the ATA-002 test series) — the *deliverable* is a base
  convective+radiative environment vs trajectory state, not an in-loop CFD run.
- **Saturn V plume-induced flow separation** (PIFS) was reconstructed with CFD
  best-practice studies (NTRS 20110004015) because the separated, recirculating
  plume gas dominated aft-body heating above ~kilometers-tens altitude; the
  flight-loop artifact is an onset criterion plus separated-zone increments.
- **Plume impingement** for staging, ullage/retro motors, and RCS has a
  dedicated NASA methodology line: PLIMP over RAMP2 flowfields, codified in the
  Low Altitude Plume Impingement Handbook (NTRS 19940008302, 19940010268) —
  a plume flowfield model plus a surface integrator producing pressure, shear,
  and heating on impinged surfaces, with rarefied corrections.
- **Multi-engine base flow at scale** is the current frontier: NASA MSFC runs
  Loci/CHEM for Starship hot-staging support (NAS SC24 showcase, project 24),
  and SpaceX runs exascale 33-engine base-flow simulations (arXiv 2505.07392).
  Those are *databases producers*; the vehicle simulation consumes increments.
- **Supersonic retropropulsion** is jointly anchored by the NASA–SpaceX
  partnership that reconstructed Falcon 9 entry burns with CFD and airborne IR
  imagery (NTRS 20170008725) and by the public FUN3D Retropropulsion Data
  Portal + Langley Unitary Plan Wind Tunnel validation campaign
  (data.nas.nasa.gov/fun3d; fun3d.larc.nasa.gov, the 2011 AIAA SRP series).

OpenBMP's target (method/architecture parity per `00` §1) is exactly that
posture, with the solver-consumer rule (`00` §1.1) applied throughout:

1. an **in-repo engineering tier**: plume similarity state computed live from
   the engine cluster (`NPR`, `C_T`, momentum-flux ratio, initial plume turn
   angle, cluster-merge state); power-on base pressure/drag correlations; base
   convective + radiative heating correlations; PIFS onset; a source-flow
   (Simons-class) plume far field with a panel impingement integrator; an SRP
   drag-replacement reduced model; PSI/ground-effect increments for landing
   burns;
2. an **ingestion + provenance + UQ** path for externally produced plume CFD
   and DSMC decks (SU2/OpenFOAM/SPARTA-class) and for the public FUN3D SRP
   solutions, through the `openbmp-aerodb` machinery of doc `03` with new
   `NPR`/`C_T` axes;
3. **uncertainty-propagating increments** (per-entry bias/random margins into
   Monte Carlo, doc `11`); and
4. **code-to-code + public-benchmark validation** anchoring the engineering
   tier (UPWT SRP runs, published SLS/Saturn V trends, DSMC cross-checks).

### 1.2 Parity ceiling (honest)

What an open repo cannot match, and the credible open substitute for each:

1. **Hot-plume composition, temperature, and radiation.** Real base radiative
   environments depend on engine-cycle-specific combustion products (soot for
   kerolox, water bands for hydrolox), afterburning, and proprietary hot-fire
   correlation. *Open substitute:* CEA/Cantera equilibrium products via
   `openbmp-thermochem` (docs `04`/`05`) + a gray-gas cylindrical-plume
   radiation model with **published emissivity bands carried as wide,
   documented uncertainty** — trend-faithful, never magnitude-certified.
2. **Flight-correlated base environments.** The SLS and Saturn V base-heating
   databases are anchored to facility campaigns (CUBRC LENS II) and flight
   reconstruction; the raw data is not fully public. *Open substitute:*
   reproduce the **published correlation methodology and trend curves** (NTRS
   20150002948, 20110004015) and validate the *shape* (peak near plume-merge
   altitude, PIFS onset progression), with magnitudes labelled accordingly.
3. **SRP at flight conditions.** The NASA–SpaceX flight reconstruction data
   (NTRS 20170008725) is summarized publicly, not released raw; the public
   anchors are **cold-gas air-jet subscale** (UPWT) tests and the FUN3D CFD
   portal. Hot-plume, flight-Reynolds, flight-enthalpy SRP increments carry an
   honestly **inflated model-form margin**; the top label is `research` on the
   cold-gas benchmark, never "flight-validated SRP."
4. **Plume-surface interaction cratering/ejecta.** Granular erosion physics is
   an active NASA research area (the PSI project); no settled engineering
   correlation exists. *Open substitute:* ship the gas-side jet-impingement /
   ground-effect increments (well-anchored), expose an ingestion hook for
   ejecta/erosion decks, and cap everything granular at `experimental`.
5. **No in-repo CFD/DSMC solver.** RANS/LES base flows and DSMC plumes are
   consumed as provenance-pinned decks; OpenBMP ships correlations,
   reduced-order models, ingestion, and V&V — the `00` §1.1 posture, restated
   here because this discipline is the one most tempting to violate it in.

**Net reachable claim:** *"power-on/SRP/impingement induced environments at
engineering-correlation + ingested-deck fidelity, methodologically parallel to
the NASA PLIMP/SLS/SRP practice, trend-validated against public benchmarks"* —
**not** *"a validated base-heating or SRP database for any real vehicle."* The
audit-style verdict this dimension can earn is **approaching** (method parity
for the correlation/ingestion architecture; the flowfield itself stays
solver-consumed). No artifact claims `flight-qualified`/`certified`/
`operational` (`00` §3.8).

---

## 2. Current state in source

Verified against the tree. The honest summary: **OpenBMP has a plume
similarity substrate plus opt-in point-mass solid-motor and rigid-body
thermochemical liquid-engine telemetry paths, but no runner-coupled plume
force/heating capability today**. The discipline is
tractable because the needed ingredients now exist or are planned: live nozzle
exit conditions, cluster geometry, a Knudsen/bridging substrate, a panel-mesh
plan (doc `03`), and an aerothermal correlation surface (doc `04`).

| Area | Where | State |
|---|---|---|
| Base drag | `crates/openbmp-aero/src/buildup.rs:232` (`base_drag_coefficient`), scaled at `buildup.rs:717-722` | **Power-off only** — the Hoerner/Niskanen correlation `0.12 + 0.13M²` (M<1), `0.25/M` (M≥1), documented as "power-off" in the docstring and `crates/openbmp-aero/README.md:53`. No NPR input, no power-on switch. A tolerance case exists (`docs/verification.md:252`). |
| Aero seam | `crates/openbmp-aero/src/method.rs` (`AeroMethod`, `AeroContext`) | `AeroContext` carries `(mach, alpha_deg, beta_deg, dynamic_pressure_pa)` **only** — engine state cannot reach the aero evaluation today; plume coupling is structurally impossible at this seam without the §3.2 extension. |
| Nozzle exit state | `crates/openbmp-propulsion/src/motor.rs:349-470` (`NozzlePerformance`, `IdealNozzlePerformance`, `NozzleSolution`) | **Shipped** (doc `05` WP-05.1 class): exit Mach, exit static pressure, momentum + pressure thrust, ambient-aware via `AmbientPressureCorrection::PressureThrust`. `NPR` and `p_e/p_∞` are computable live today — the plume similarity inputs already exist. |
| Solid motor geometry | `crates/openbmp-propulsion/src/grain.rs:368-433` | Throat area + expansion ratio validated at load; `exit_area_m2 = throat·ε` (`grain.rs:522`). |
| Cluster geometry | `crates/openbmp-propulsion/src/cluster.rs:60-133` (`EngineCluster::mount_points_body`) | Per-engine body-frame mount points exist — the spacing input the multi-engine merge criterion needs. No plume use. |
| Plume similarity substrate | `crates/openbmp-plume/src/lib.rs` (`PlumeState`, `PlumeNozzle`, `PlumeFreestream`, `PlumeClusterGeometry`); `crates/openbmp-runner/src/plume.rs` | **Partial WP-15.1.** Computes NPR, exit-pressure ratio, `C_T`, momentum-flux ratio, Prandtl-Meyer initial turn angle, reduced cluster-merge distance, and PIFS onset from coordinate-free nozzle/freestream/geometry scalars. Point-mass solid-motor scenarios can opt into telemetry via `[aero.plume]`; rigid-body thermochemical liquid-engine scenarios can also emit plume telemetry from live engine mass-flow snapshots, with active merge spacing derived from engine mount points. Full derived cluster-geometry assembly remains pending. |
| Rarefied substrate | `crates/openbmp-aero/src/knudsen.rs` | Mean free path, Knudsen number, three bridge functions, Schaaf-Chambré free-molecular aero, `HybridAeroMethod`. Built for freestream rarefaction; directly reusable for plume-impingement regime handoff (§3.6). |
| Aerothermal surface | `crates/openbmp-aerothermal/src/stagnation.rs` | Fay-Riddell, Sutton-Graves, Tauber-Sutton-reserved — **forebody stagnation only**. No base heating, no separated-zone heating, no plume radiation. |
| Force composition | `crates/openbmp-runner/src/point_mass.rs:59` (`EngineClusterForceAdapter`, `AeroMethodForceAdapter`, `DeckDragForceAdapter`), `crates/openbmp-runner/src/aero.rs` | Thrust and aero are **independent summed adapters** — no coupling. During the Phalcon-9 boostback/entry burn the deck applies full power-off drag while the engine fires retrograde; the SRP drag-collapse physics is absent. This is the headline wrongness this doc exists to fix. |
| Retro-burn scenarios | `scenarios/phalcon9/phalcon9-orbit-boostback.toml` (+ `mission-graph-boostback.toml:218` `landing_burn`) | A flown, CI-exercised booster boostback → entry → `landing_burn` mission profile, currently with plume-free aero throughout. The capstone consumer (WP-15.12). |
| Stated deferrals | `docs/roadmap.md:152` ("Power-on plume/base-drag coupling" deferred), `docs/staging-and-separation.md:172` ("Coupled (reserved). Plume impingement…") | The repo already names these as reserved seams; this document is their design. |
| Sibling-doc coverage | doc `03` §1.2 item 2 + §7; doc `04` §7; doc `05` §1.2 item 4 + §8 (SU2/OpenFOAM/SPARTA "offline reference fields" rows) | Each defers the discipline to the others; none designs it. Doc `03`'s panel mesh/BVH (T1) and `openbmp-aerodb` (T3/T4) are the geometry and database substrates this doc consumes rather than duplicates. |

**Maturity verdict:** Tier 0 for coupled plume effects; the T1 similarity
math substrate has started. Power-off base drag remains the only plume-adjacent
force model. Nothing here regresses any existing model; every tier is additive
and gated.

---

## 3. Target architecture

### 3.0 New / changed crates

```text
NEW (proposed; placement justified below and reviewed in WP-15.1):
  openbmp-plume     L2  plume similarity state; power-on base pressure/drag;
                        base convective+radiative heating; PIFS onset; SRP
                        reduced model + increment lookup; source-flow plume
                        flowfield + impingement surface integrator; PSI/ground
                        effect. Depends only on openbmp-core/-models/-state,
                        openbmp-propulsion (nozzle/cluster state types),
                        openbmp-aero (AeroContext, panel mesh, Knudsen bridges),
                        openbmp-aerothermal (heating output types). All L2 →
                        L2/L1/L0 edges; acyclic; never touches the FC.

CHANGED:
  openbmp-aero      L2  + optional engine-state extension of the aero context
                        (additive struct, §3.2); power-off/power-on base-drag
                        seam in ComponentBuildup (consumed, not moved).
  openbmp-aerodb    L6  + plume/SRP deck schemas: NPR and C_T axes, ΔC_A/ΔC_N/
                        ΔC_m increment tables, base-pressure/heating decks,
                        impingement reference decks (doc 03's run-matrix +
                        ingestion + UQ machinery, extended — not duplicated).
  openbmp-runner    L7  + PlumeState assembly per step (engine cluster +
                        atmosphere → openbmp-plume); force/heating increment
                        adapters; scenario wiring for the [aero.plume] family.
  openbmp-scenario  —   + v3 config blocks: [aero.plume], [aero.plume.base],
                        [aero.plume.base_heating], [aero.plume.srp],
                        [propulsion.plume.impingement], [aero.plume.ground_effect].
```

> **Crate-boundary justification (against `00` §4).** No existing crate fits:
> `openbmp-aero` evaluates freestream aerodynamics over `(M, α, β, q)` and must
> not grow a dependency on `openbmp-propulsion` (it would entangle the two L2
> physics crates and force every aero consumer to carry engine types);
> `openbmp-propulsion` is engine-internal physics and must not grow vehicle
> surface-geometry concerns; `openbmp-aerothermal` is boundary-layer heating on
> the forebody. The plume discipline is precisely the **coupling layer** that
> consumes propulsion exit state + freestream + surface geometry and produces
> force/heating increments — a textbook seam for a narrow L2 crate.
> `openbmp-plume` depends downward/laterally only (`core`, `state`, `models`,
> `propulsion`, `aero`, `aerothermal`); no existing crate gains an edge to it
> except the L7 runner. The FC portability lock is untouched: `openbmp-fc`
> never sees this crate (`00` §3.3). The first commit of WP-15.1 is the
> skeleton + this justification, reviewed before implementation (`13` §5).

### 3.1 Plume similarity state, computed live (T1)

Everything downstream keys off a small set of similarity parameters, assembled
each step from the engine cluster and the atmosphere — never from config
constants, so throttling, altitude, and engine-out all propagate physically:

```
NPR     = p_c0 / p_∞                      nozzle (stagnation) pressure ratio
π_e     = p_e / p_∞                       exit static pressure ratio  (>1 ⇒ underexpanded)
C_T     = T_total / (q_∞ · S_ref)          thrust coefficient (the SRP similarity parameter)
MFR     = (ṁ V_e + (p_e − p_∞)A_e) / (ρ_∞ V_∞² A_ref)   momentum-flux ratio
δ_j     = ν(M_a) − ν(M_e) + θ_N            initial plume turn angle at the lip      (3.1.1)
```

with `ν(M)` the Prandtl-Meyer function, `M_a` the Mach reached by isentropic
expansion of the exit flow from `p_e` to `p_∞` at plume `γ_e`, and `θ_N` the
nozzle divergence half-angle. `p_e`, `M_e`, `T` come straight from the existing
`NozzleSolution` (`crates/openbmp-propulsion/src/motor.rs`); `p_∞, ρ_∞, q_∞`
from the doc `08` atmosphere; plume `γ_e` (and, at T3+, products/temperature)
from constants or the `openbmp-thermochem` deck (docs `04`/`05`).

**Cluster merge criterion** (multi-engine): plumes are flagged `merged` when
the inviscid plume boundary radius at the base plane, grown at `δ_j` from each
exit lip, exceeds half the minimum engine spacing from
`EngineCluster::mount_points_body`. Merge closes the base vent area and creates
the reverse jet that drives base recirculation heating — the discriminator the
base-environment tiers switch on.

```rust
/// Live plume similarity state, assembled per step by the runner.
/// All inputs come from engine/cluster state + atmosphere; no config constants.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PlumeState {
    pub npr: f64,                       // p_c0 / p_inf
    pub exit_pressure_ratio: f64,       // p_e / p_inf
    pub thrust_coefficient: f64,        // C_T = T / (q_inf * S_ref)
    pub momentum_flux_ratio: f64,
    pub initial_turn_angle_rad: f64,    // eq. 3.1.1
    pub gamma_e: f64,                   // plume specific-heat ratio
    pub active_engines: u32,
    pub merged: bool,                   // cluster plume-merge flag
}
```

T1 ships `PlumeState` as **telemetry + diagnostics only** (including a PIFS
onset *indicator*, §3.4) so the similarity machinery is verified before any
force is touched. Off by default behind `[aero.plume]`.

### 3.2 The engine-state seam into aero (additive, off by default)

`AeroContext` stays untouched (byte-stability). Plume-aware models receive an
*additional* argument through a new trait, and the runner composes them as a
**decorator** over the existing `AeroMethod`/deck force adapters:

```rust
/// A power-on increment over a power-off aero evaluation.
/// Forward-only: consumes flight conditions + plume state, returns body-frame
/// force/moment deltas. There is no position, coordinate, or target input.
pub trait PlumeInducedIncrement {
    fn increment(
        &self,
        ctx: &AeroContext,            // (M, alpha, beta, q) — unchanged
        plume: &PlumeState,
    ) -> Result<AeroForceMomentBody, PlumeError>;
}
```

When `[aero.plume.*]` is absent the decorator is not constructed and the force
stack is bit-for-bit the current one. When armed with all engines off
(`NPR → 0`), every increment returns exactly zero — an identity tested in CI
(§5.1) so arming the block on an unpowered phase cannot perturb a byte of
physics, only telemetry.

### 3.3 Power-on base pressure and base drag (T2)

The existing buildup charges power-off base drag on the full base area. With
engines on, the physical regimes (Korst component analysis; the
Brazzel-Henderson-class jet-on base-pressure correlations) are:

1. **Aspiration** (low altitude, moderate NPR): the jet entrains base air,
   *lowering* `p_b` — base drag increases over power-off.
2. **Plume blockage/pressurization** (rising altitude/NPR): the expanding plume
   blocks reverse flow and recompresses the base — base drag falls, and at high
   altitude the base can carry net *thrust* (`p_b > p_∞`).
3. **Cluster merge** (multi-engine): merged plumes choke the vent area between
   engines; a reverse jet stagnates on the base — the base-pressure curve
   develops the knee that the Saturn V/SLS data shows, and base *heating*
   (§3.4) peaks.

The T2 model is a correlation surface for the **power-on base-pressure
coefficient**:

```
C_p,b = (p_b − p_∞)/q_∞ = f( M_∞, NPR, A_e,tot/A_b, n, s/d_e )        (3.3.1)
ΔC_A,base = −(C_p,b − C_p,b,power-off) · A_b,effective / S_ref         (3.3.2)
```

where `A_b,effective` excludes the nozzle exit area(s), `n` is the active
engine count and `s/d_e` the spacing ratio from cluster geometry. The
correlation ships as a provenance-pinned table (digitized from published
open-literature curves, with the source recorded per `00` §3.5/3.6) plus the
analytic asymptotes (power-off limit at `NPR→0`; vacuum limit
`p_b → plume-driven` at `q_∞→0`). Engine-out asymmetry: with a non-symmetric
active set, the base increment is applied at the centroid of the active-plume
pattern, producing the physical induced moment.

### 3.4 Base thermal environment + PIFS (T3)

**Convective.** Below plume merge: weak recirculation, scaled from the
power-off near-wake correlation. Above merge: the reverse jet stagnates on the
base; convective flux follows an impinging-jet stagnation correlation keyed to
the recirculated momentum fraction `φ(NPR, s/d_e)`:

```
q_conv,b = St_b · ρ_rev V_rev · c_p (T_rev − T_w),   (ρV)_rev = φ · (ṁ/A_vent)   (3.4.1)
```

The published SLS methodology (CUBRC LENS II short-duration testing →
correlation, NTRS 20150002948) is the **methodology template**: the V&V target
is the trend (altitude of peak base heating tracking the merge criterion,
scaling with engine count), with magnitudes labelled honestly (§5).

**Radiative.** Gray-gas cylindrical-plume model: the plume below the base plane
is an equivalent emitting cylinder of temperature `T_p` and emissivity `ε_p`;
the base sees

```
q_rad,b = ε_p σ T_p⁴ · F_b→p                                            (3.4.2)
```

with the closed-form disk-to-cylinder view factor and `(T_p, ε_p)` from the
thermochem deck + published per-propellant emissivity bands (soot-dominated
kerolox vs band-emitting hydrolox), each carried with a wide documented
uncertainty into doc `11`.

**PIFS onset.** Plume-induced flow separation is flagged when the
plume-induced pressure rise at the nozzle lip exceeds the turbulent incipient
separation pressure rise (free-interaction criterion):

```
separation ⇐ δ_j − θ_boattail > θ_sep,incipient(M_local, Re)            (3.4.3)
```

Above onset, the separated zone grows forward with altitude (the Saturn V
behavior, CFD best practice in NTRS 20110004015); the model carries (a) the
separation-front station, (b) a separated-zone pressure plateau → `ΔC_N`/`ΔC_m`
increment (destabilizing), and (c) a separated-zone heating multiplier feeding
the doc `04` distributed deck. T1 ships the onset flag as telemetry; T3 ships
the increments.

### 3.5 Supersonic retropropulsion increments (T4)

The SRP literature (Jarvinen & Adams 1970; the 2011 Langley UPWT campaign;
NASA–SpaceX flight reconstruction, NTRS 20170008725) parameterizes the
power-on aerodynamics by `C_T`, `M_∞`, `α`, and nozzle layout:

- **Drag replacement / collapse** (central nozzle): the plume displaces the bow
  shock; aerodynamic drag collapses and total axial force approaches thrust
  alone — `C_A,total ≈ C_T` for `C_T ≳ 2–3`.
- **Drag preservation** (peripheral nozzles, low `C_T ≲ 1`): part of the
  forebody keeps post-shock pressure; total exceeds thrust-only.
- **Heating shielding**: the plume-processed flow reduces convective heating on
  plume-covered surfaces during the burn (the IR-imagery observation in the
  NASA–SpaceX work).
- **Stability increments**: jets-in-crossflow at `α` shift `C_m` and add
  unsteadiness.

Two-layer model, both off by default behind `[aero.plume.srp]`:

**Reduced model (T4-a):** a smooth drag-knockdown over the power-off
coefficient,

```
C_A,aero(C_T) = C_A,off · [ f_p + (1 − f_p) · e^(−k·C_T) ]              (3.5.1)
C_A,total     = C_T + C_A,aero(C_T)
```

with collapse rate `k` and preserved fraction `f_p` calibrated per nozzle
layout (central / tri / peripheral) against the public UPWT/FUN3D data;
`ΔC_N, ΔC_m` analogous first-order forms. Honest, smooth, deterministic, and
correct in both limits (`C_T→0` power-off; `C_T→∞` thrust-dominated).

**Increment database (T4-b):** `ΔC_A/ΔC_N/ΔC_m (M_∞, α, C_T)` tables ingested
through `openbmp-aerodb` from the **FUN3D Retropropulsion Data Portal**
(public NASA solutions) and the published UPWT runs, with per-entry RSS
uncertainty (doc `03` T3 machinery) and the CFD-vs-test scatter recorded as
model-form margin. The reduced model is the fallback outside the table
envelope; envelope exits are telemetered, fail-closed to the reduced model,
never extrapolated silently.

**Heating shield (T4-c):** a bounded multiplier `κ_q(C_T) ∈ (0, 1]` on the
affected aerothermal nodes during the burn, trend-calibrated from the public
SRP CFD; carried as an interval (not a point) into doc `11`.

The SRP surface consumes **flight conditions only** — `(M_∞, α, C_T)` — by
construction (§6).

### 3.6 RAMP2/PLIMP-class impingement (T5)

The NASA pattern (Low Altitude Plume Impingement Handbook, NTRS 19940008302,
19940010268): a plume **flowfield model** plus a **surface integrator**.

**Flowfield (in-repo tier):** Simons source-flow far field —

```
ρ(r, θ) = ρ_c A_p (r*/r)² f(θ),   f(θ) = cos^{2/(γ_e−1)}( πθ / (2 θ_lim) )   (3.6.1)
u(r, θ) ≈ V_lim = sqrt( 2 γ_e R_e T_c / (γ_e − 1) )
```

normalized so the spherical-cap mass-flux integral equals `ṁ` exactly (a
machine-precision MMS case, §5.1), with the Boynton boundary-layer/backflow
correction term for the high-angle lobe. Valid in the far field of an
underexpanded nozzle — exactly the staging/ullage/RCS regime. The near field
and reacting cases are **ingested decks** (SU2/OpenFOAM offline; Loci/CHEM-class
results where a user has access), provenance-pinned, same receiver interface.

**Surface integrator:** receiver geometry is the doc `03` watertight panel
mesh + BVH (occlusion = shadowing query, reused not rewritten). Per panel `i`
with incidence `cos θ_i = −n̂_i · V̂_local`:

```
p_i  = ρ_i V_i² cos²θ_i  (+ p_static,i)        modified-Newtonian continuum limit
τ_i  = c_f · ρ_i V_i² cosθ_i sinθ_i            engineering shear fraction
q_i  = St_imp · ½ ρ_i V_i³ cosθ_i              impingement heating            (3.6.2)
```

integrated to body-frame force/moment + a per-panel heating map (consumed by
the doc `04` distributed deck and doc `02` loads). In the free-molecular limit
the panel relations reduce to Schaaf-Chambré — the existing
`crates/openbmp-aero/src/knudsen.rs` implementation is the oracle for that
limit (§5.1).

**Rarefied/DSMC handoff:** local breakdown is detected with the
gradient-length-local Knudsen number `Kn_GLL = (λ/Q)|∇Q|` (Boyd's criterion,
threshold ≈ 0.05) evaluated on the source-flow field, equivalently Bird's
expansion breakdown parameter. Below threshold: continuum relations. Above:
bridge via the existing `knudsen.rs` functions or switch to an **ingested
SPARTA DSMC deck** (doc `03` WP-03.9 pipeline) for the receiver region. The
handoff criterion and the chosen regime are telemetered per evaluation.

**Use cases wired (T5):** stage separation and hot-staging transients (upper
engine start while the booster interstage is still proximate — impingement
pressure/heating on the forward dome, the Starship-class event NASA supports
with Loci/CHEM), ullage/retro motor firings during separation, and RCS
self-impingement (thruster geometry from doc `09`).

### 3.7 Plume-surface interaction & ground effect for landing burns (T6)

For the terminal landing burn (consumed by doc `14`, the powered-descent and
touchdown dimension in this series):

- **Jet-on-ground-plane model:** normal/oblique underexpanded jet impingement
  (Donaldson-Snedeker structure: plate shock, stagnation bubble, wall jet),
  producing a ground-plane pressure/heat-flux footprint (pad-loads output) and
  the back-reaction on the vehicle.
- **Ground-effect increments on the vehicle:** axial force increment
  `ΔF_A/T = g(h/d_e, NPR, n)` capturing the cushion (pressure recovery under
  the base at low `h/d_e`) and suckdown (entrainment) regimes, plus the
  multi-engine **fountain** upwash term between engines — the STOVL
  ground-effect literature (Saddington & Knowles review) is the open anchor.
- **Inputs:** height above the ground plane as a **relative scalar** (`h` AGL
  from the doc `14` touchdown state) and plume state. No site coordinates, no
  map — by construction (§6).
- **Erosion/ejecta:** out of the engineering tiers. A deck-ingestion hook for
  externally computed ejecta/erosion fields (NASA PSI project class) is the
  only artifact, capped `experimental`.

### 3.8 Fidelity tiers

| Tier | Scope | Earns |
|---|---|---|
| **T0** (current) | power-off base drag only; thrust and aero summed independently; retro burns physically wrong in atmosphere | (baseline) |
| **T1** | `PlumeState` similarity parameters live from engine cluster + atmosphere; cluster-merge flag; PIFS onset indicator; telemetry/diagnostics only | `validated-toy` |
| **T2** | power-on base pressure/drag increment (eq. 3.3.1–3.3.2) incl. cluster + engine-out asymmetry; power-off identity at `NPR→0` | `validated-toy` |
| **T3** | base convective + radiative heating (3.4.1–3.4.2); PIFS separated-zone load/heating increments (3.4.3) | `validated-toy`; `research` for trend anchors (SLS/Saturn V published curves) |
| **T4** | SRP: reduced drag-replacement model (3.5.1) + ingested ΔC database (FUN3D portal/UPWT) + heating shield + stability increments | `research` (cold-gas public benchmark); hot-plume extrapolation stays `validated-toy` with inflated margins |
| **T5** | impingement: Simons source-flow + panel integrator (3.6.1–3.6.2) + `Kn_GLL` rarefied handoff + CFD/DSMC deck ingestion; staging/hot-staging/ullage/RCS wiring | `validated-toy` → `research` (code-to-code SPARTA/SU2) |
| **T6** | PSI/ground effect for landing burns: ground-plane footprint + vehicle increments + fountain; ejecta ingestion hook | `validated-toy`; ejecta hook `experimental` |

Each tier is independently shippable; T1/T2 alone already fix the worst ascent
error (power-on base drag) and ship verifiable value without T4–T6.

---

## 4. Invariant preservation

1. **Byte-determinism.** All correlations and the source-flow model are pure
   `f64` closed forms with locked operand order, no `f64::mul_add`, no
   wall-clock, no unordered iteration; table lookups go through the
   deterministic `openbmp-aerodb` interpolation (doc `03`). Panel integration
   iterates the mesh in stored index order with a fixed-order summation.
   The only randomness in this dimension is UQ dispersion draws, which flow
   through doc `11`'s `openbmp-uq`/`openbmp-mc` machinery on `DeterministicRng`
   with a new reserved domain component for plume entries — `openbmp-plume`
   itself holds no RNG. Cross-libm caveat (trig/exp/pow in Prandtl-Meyer,
   Simons, view factors) is the same one `openbmp-aero` already documents; the
   byte-diff gate runs on the reference profile.
2. **Byte-stable-by-default.** Every tier is off until a scenario opts in:
   `[aero.plume]` (T1 telemetry), `[aero.plume.base]` (T2),
   `[aero.plume.base_heating]` (T3), `[aero.plume.srp]` (T4),
   `[propulsion.plume.impingement]` (T5), `[aero.plume.ground_effect]` (T6) —
   the `[vehicle.bending]` pattern (`00` §3.2). All existing goldens, including
   the Phalcon-9 boostback CSV archives, stay byte-identical until a scenario
   opts in. Two identity gates harden this: armed-with-engines-off increments
   are exactly zero (§3.2), and the `NPR→0` limit of the power-on base model
   reproduces the power-off Hoerner term exactly (§5.1).
3. **FC hardware-portability lock.** Everything in this dimension is sim-side
   (L2 `openbmp-plume`, L6 `openbmp-aerodb`, L7 runner). `openbmp-fc` gains
   **no** new dependency; plume quantities reach the FC only as sensor-shaped
   telemetry across the existing boundary, if a scenario routes them at all.
   `fc_dependency_tripwire.rs` stays green untouched.
4. **Lockstep-clock & no-hot-path-allocation.** `openbmp-plume` evaluations are
   allocation-free on the per-step path (fixed-size `PlumeState`, preallocated
   panel scratch buffers sized at scenario load); no clock reads anywhere in
   the crate. `fc_lints.rs` is unaffected (no FC code changes).
5. **Four-pillar provenance.** Digitized base-pressure/heating correlation
   tables, FUN3D portal solutions, UPWT-derived increment tables, SPARTA/SU2
   reference decks, and ground-effect curves all land under `data/plume/` (and
   scenario fixtures under `tests/fixtures/`) with sibling `provenance.md`,
   SHA-256 pins, license/status notes (NASA-produced data recorded as US
   government work; portal terms recorded), and validation labels — verified
   fail-closed at load. No inline data in `*.rs`; no real-vehicle parameter
   sets (the Phalcon-9 vehicle stays synthetic). New source-of-truth paths are
   added to the tripwire allow-list in the same PR (`13` §2).
6. **Validation labels.** Per the §3.8 table, justified by §5 evidence, with a
   tolerance-table TOML for every numeric claim. Base-heating magnitudes and
   hot-plume SRP extrapolations explicitly do **not** inherit the `research`
   label their cold-gas/trend anchors earn.
7. **Forward-only locks tighten.** No model in this dimension accepts a
   position, coordinate, site, or map input: `PlumeInducedIncrement` and the
   SRP surface consume `(AeroContext, PlumeState)` only; the ground-effect
   model consumes a relative AGL scalar. A new **tripwire test** (WP-15.5)
   asserts the SRP/plume increment input types carry no field whose name or
   type can express a coordinate/aimpoint, extending the
   `ballistic_state_compile_fail.rs` spirit to this surface. The footprint
   provenance lock is untouched.
8. **Requirements traceability.** Each WP adds `requirements.toml` entries
   with verification evidence; the traceability gate stays green.

---

## 5. V&V plan

Five-layer ladder per `docs/verification.md`: MMS/code verification → model
verification vs correlations → code-to-code vs open solvers → public
benchmarks → UQ reporting. Tolerance tables ship as `expected.toml` per case.

### 5.1 Analytic / MMS (machine-precision code verification)

| Case | Setup | Tolerance | Tier |
|---|---|---|---|
| Prandtl-Meyer function | `ν(M)` vs closed form across `M ∈ [1, 20]`, several `γ` | `< 1e-12` | T1 |
| Plume turn angle limits | `δ_j → 0` as `p_e → p_∞`; `δ_j → ν_max(γ_e) − ν(M_e) + θ_N` as `p_∞ → 0` | `< 1e-10` | T1 |
| Simons mass conservation | spherical-cap integral of `ρu` over eq. 3.6.1 equals `ṁ` | `< 1e-9` rel | T5 |
| Centerline decay | fitted log-slope of `ρ(r, 0)` equals −2 | `< 1e-6` | T5 |
| Disk↔cylinder view factor | eq. 3.4.2 geometry vs closed-form view factor | `< 1e-10` | T3 |
| Impingement momentum closure | normal jet on a large plate: integrated panel normal force = jet momentum flux (Newtonian limit) | `< 1%` | T5 |
| Free-molecular consistency | §3.6.2 free-molecular limit vs `knudsen.rs` Schaaf-Chambré on a shared flat plate | `< 1e-12` | T5 |
| Power-off identity | `NPR → 0`: power-on base model returns the `buildup.rs:232` Hoerner value exactly; armed-engines-off increments identically zero | exact (bit) | T2 |
| Merge-criterion geometry | analytic 4-engine symmetric cluster: merge altitude vs hand-computed plume-boundary intersection | `< 1e-9` | T1 |
| MMS on the increment decorator | manufactured `ΔC(M, α, C_T)` polynomial through ingest→lookup→force path recovers source | `< 1e-12` | T4 |

### 5.2 Model verification (behavioral / trend)

| Case | Anchor | Acceptance | Tier |
|---|---|---|---|
| Power-on base pressure regimes | digitized open-literature jet-on base-pressure curves (provenance-pinned) | aspiration dip then pressurization vs NPR; within the published band (±20%) | T2 |
| PIFS onset progression | Saturn V CFD best-practice trends (NTRS 20110004015) | separation front moves forward monotonically with altitude; onset altitude within the published band on a Saturn-V-like synthetic geometry | T3 |
| Base-heating peak location | SLS methodology shape (NTRS 20150002948) + Saturn V flight-era curves | `q_conv,b` peaks near the cluster-merge altitude; scales with active-engine count; **trend only, magnitudes labelled** | T3 |
| SRP regime map | Jarvinen & Adams classic results | drag preservation at low `C_T` (peripheral), collapse `C_A,aero → ~0` for central `C_T ≳ 2–3`; smooth, monotone knockdown | T4 |
| Ground-effect regimes | Saddington & Knowles review curves | cushion at low `h/d_e`, suckdown band, fountain sign for multi-engine | T6 |

### 5.3 Code-to-code (open solvers; `research`-anchoring)

| Case | Oracle | Posture |
|---|---|---|
| Underexpanded cold-gas air jet far field (NPR 5–50) | **SU2** (subprocess) / **OpenFOAM** (offline) | Simons centerline + angular profile within ±30% in the source-flow validity region; divergence documented near-field |
| Rarefied plume + flat-plate impingement | **SPARTA** DSMC (offline deck) | continuum/bridge/free-molecular handoff: panel loads within the bridge band; `Kn_GLL` threshold placement justified by the crossover |
| FUN3D portal solution round-trip | ingested portal solution vs `openbmp-aerodb` lookup | interpolation-tolerance exact at nodes; envelope-exit behavior fail-closed |

### 5.4 Public benchmarks (`research`)

| Case | Benchmark | Note |
|---|---|---|
| SRP axial-force collapse vs `C_T` | Langley **UPWT SRP campaign** (2011 AIAA series; geometry/conditions public) + **FUN3D Retropropulsion Data Portal** solutions | reduced model + increment DB reproduce the published cold-gas `C_A,total(C_T, M, α)` within ±15% (central nozzle), ±25% (tri-nozzle); CFD-vs-test scatter recorded as model-form margin |
| SRP flight plausibility | NASA–SpaceX F9 reconstruction **as published** (NTRS 20170008725) | qualitative only — the public artifact is methodology + conclusions, not raw data; recorded as a methodology anchor, never a tolerance row |
| External validity (LOCAL only) | third-party Falcon-class webcast telemetry, entry-burn deceleration profile | gross-error sanity per `docs/external-telemetry-validation.md`; **never committed, never CI, never an input** |

### 5.5 Labels earned

T1/T2 + the impingement integrator earn `validated-toy` (analytic/MMS + identity
gates). T3 earns `validated-toy`, plus `research` strictly for the
trend-anchored cases (onset progression, peak location). T4 earns `research` on
the cold-gas UPWT/FUN3D benchmark; the hot-plume/flight regime stays
`validated-toy` with inflated, documented margins. T5 earns `research` where
the SPARTA/SU2 code-to-code rows exist, `validated-toy` otherwise. T6 earns
`validated-toy`; the ejecta hook stays `experimental`. Nothing in this
dimension can claim flight-validated environments (§1.2).

---

## 7. Dependencies on other parity docs

- **`05-propulsion-high-fidelity.md`** — *hard dependency, partially shipped.*
  `PlumeState` consumes the `NozzleSolution` exit state
  (`crates/openbmp-propulsion/src/motor.rs`, already in source) and cluster
  geometry (`cluster.rs`); plume gas properties `(γ_e, T_c, products)` at T3+
  come from the `openbmp-thermochem` deck (WP-05.3); hot-staging start
  transients (T5) consume the transient chamber (WP-05.4-a); engine-out
  asymmetry consumes the fault library (WP-05.5-b).
- **`03-aerodynamics-database-and-cfd-coupling.md`** — *hard dependency for
  T4-b/T5.* The watertight panel mesh + BVH (WP-03.1) is the impingement
  receiver; `openbmp-aerodb` (WP-03.4) + per-entry RSS UQ (WP-03.6) carry the
  SRP/plume decks with new `NPR`/`C_T` axes; the SPARTA pipeline (WP-03.9) is
  the DSMC deck source; the stage-separation proximity deck (WP-03.7) is the
  plume-free baseline the T5 impingement increments superpose on. The power-on
  base term replaces this doc's §1.2-item-2 deferral.
- **`04-aerothermal-realgas-and-tps.md`** — base/separated-zone/impingement
  heating feed the distributed heating deck and TPS sizing; the radiative model
  shares `openbmp-thermochem`; the SRP heating shield multiplies doc `04`
  nodes. Closes doc `04` §7's shared-ceiling deferral with a designed owner.
- **`08-environment-gravity-and-frames.md`** — `p_∞, ρ_∞, T_∞, a_∞` for NPR,
  `q_∞`, merge altitude; symbolic constants only.
- **`01-flexible-multibody-dynamics.md` / `02-structural-dynamics-loads-slosh-pogo.md`**
  — separation-as-joint-release events consume T5 impingement transients as
  proximity forces; base/impingement pressure maps feed CLA load cases.
- **`09-sensors-navigation-and-actuators.md`** — RCS thruster geometry for
  self-impingement; plume telemetry reaching the FC goes through the doc `09`
  sensor boundary only.
- **`07-trajectory-optimization-and-mission-design.md`** — descent trajectory
  optimization runs against the SRP-corrected plant; shares the forward-only
  vocabulary lock (§6); no optimizer surface changes here.
- **`11-monte-carlo-uq-and-validation.md`** — per-entry plume/SRP margins,
  emissivity bands, and model-form inflation flow into `openbmp-uq`/`openbmp-mc`
  and the 7009B credibility record.
- **`12-determinism-realtime-and-compute.md`** — deterministic table
  interpolation, fixed-order panel reductions, byte-diff gate discipline.
- **Doc `14` (powered descent & touchdown, this series)** — *consumer.* T6
  ground-effect/PSI increments and the pad footprint are inputs to the
  touchdown dynamics and landing-loads cases; the AGL scalar comes from its
  state definition.

**Ordering.** T1/T2 need only shipped propulsion state + atmosphere (Phase B,
immediately actionable). T3 wants `openbmp-thermochem` (docs `04`/`05`). T4-b
and T5 gate on doc `03` WP-03.1/03.4/03.6. T6 pairs with doc `14`.

---

## 8. Open-source leverage

| Tool / data | License / status | Mode | Use |
|---|---|---|---|
| **FUN3D Retropropulsion Data Portal** (data.nas.nasa.gov/fun3d) | public NASA data (FUN3D *code* is export-controlled — data only) | **ingest** | SRP CFD solutions → ΔC increment tables with provenance + SHA pins; the T4-b backbone. |
| **Langley UPWT SRP campaign** (fun3d.larc.nasa.gov, 2011 AIAA SRP series) | published papers/figures | **ingest** (digitized, provenance-pinned) | the public cold-gas test benchmark for §5.4. |
| **PLIMP / RAMP2 handbooks** (NTRS 19940008302, 19940010268) | public NASA reports | **port methodology** | the impingement architecture (flowfield + surface integrator + rarefied corrections); clean-room re-implementation in Rust. |
| **SU2** | LGPL 2.1 | **couple** (subprocess) | underexpanded-jet reference fields; code-to-code rows; never vendored. |
| **OpenFOAM** (`rhoCentralFoam`) | GPL-3.0 | **couple-offline** | base-flow / jet reference cases; process boundary only, never linked (same posture as docs `03`/`05`). |
| **SPARTA** (Sandia DSMC) | GPL-3.0 | **couple-offline** | rarefied plume + impingement reference decks for the handoff validation; offline fields only. |
| **Cantera** | BSD-3-Clause | **couple / ingest** | plume gas properties (γ_e, products, T) feeding the thermochem deck shared with docs `04`/`05`. |
| **NASA CEA / CEARUN** | NASA tool, public outputs | **ingest** (outputs as data) | cross-check of plume thermochemistry; same posture as doc `05`. |
| **Loci/CHEM** (NASA MSFC; NAS SC24 project 24 Starship hot-staging) | restricted distribution (US release) | **reference only** | methodology exemplar for hot-staging impingement; users with access may ingest results through the generic deck path; never a dependency. |
| **SpaceX 33-engine exascale base-flow study** (arXiv 2505.07392) | public preprint | **reference** | multi-engine merge/reverse-jet phenomenology guiding the T2/T3 cluster model; no data dependency. |
| **NASA PSI project publications** | public NTRS | **reference / ingest hook** | plume-surface-interaction phenomenology; ejecta deck schema target (capped `experimental`). |

License hygiene as elsewhere: GPL tools are external processes only; everything
ported is from public-domain US-government reports or permissive sources;
`cargo deny` enforces.

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-15.1 — `openbmp-plume` skeleton + live plume similarity state

- **implementation_status:** partial. The L2 crate skeleton and coordinate-free
  similarity primitives are implemented and traced by `REQ-PLUME-001` /
  `V-PLUME-001`; the schema v3 `[aero.plume]` opt-in and point-mass
  solid-motor runner telemetry path are implemented and traced by
  `REQ-PLUME-002` / `V-PLUME-002`. Rigid-body thermochemical liquid-engine
  telemetry is implemented with `LiquidPlumeEngine` / `RigidPlumeEvaluator`
  and traced by `REQ-PLUME-003` / `V-PLUME-003`; active center spacing for
  merge telemetry is derived from engine mount points and traced by
  `REQ-PLUME-004` / `V-PLUME-004`. Full derived multi-engine geometry, broader
  liquid-engine calibration, and byte-identity golden coverage remain to
  complete the WP.
- **goal:** Create the L2 crate (first commit = skeleton + §3.0 placement
  justification, reviewed) and ship `PlumeState` assembled per step from
  `NozzleSolution` + cluster geometry + atmosphere: NPR, exit-pressure ratio,
  C_T, momentum-flux ratio, Prandtl-Meyer initial turn angle, cluster-merge
  flag, PIFS onset indicator — telemetry/diagnostics only. The similarity
  substrate every later tier keys on.
- **fidelity_tier:** T1
- **depends_on:** [WP-05.1 (shipped in source — `NozzlePerformance` /
  `NozzleSolution`, `crates/openbmp-propulsion/src/motor.rs`)]
- **new_crates:** `openbmp-plume` (L2; deps: core/models/state/propulsion/aero/
  aerothermal; no FC edge; reviewed before implementation)
- **touched:** `crates/openbmp-plume/**` (new), `crates/openbmp-runner/src/`
  (PlumeState assembly + telemetry columns), `crates/openbmp-scenario/src/document.rs`
  (`[aero.plume]`), `requirements.toml`
- **approach:** §3.1 (eq. 3.1.1, merge criterion), §3.4 (eq. 3.4.3 as a flag).
  Pure closed forms, locked operand order, no RNG.
- **acceptance:**
  - off by default; all canonical goldens (incl. Phalcon-9 boostback) byte-identical
  - Prandtl-Meyer vs closed form `< 1e-12`; turn-angle limits `< 1e-10`;
    merge-criterion analytic case `< 1e-9` (tolerance tables)
  - an opt-in scenario exercises PlumeState telemetry through a full ascent
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — diagnostics over vehicle-intrinsic state;
  input types carry no coordinate field.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** similarity parameters only; no flowfield, no force change;
  plume `γ_e` is a constant until the thermochem deck lands.

### WP-15.2 — Power-on base pressure / base drag increment

- **goal:** Replace the power-off-only base term for opting scenarios with the
  power-on base-pressure correlation (eqs. 3.3.1–3.3.2): aspiration →
  pressurization regimes vs NPR, cluster spacing + vent-area merge effects,
  engine-out asymmetry moment. Fixes the largest *ascent* plume error.
- **fidelity_tier:** T2
- **depends_on:** [WP-15.1]
- **new_crates:** none (extend `openbmp-plume`; correlation table under
  `data/plume/base_pressure/` with provenance)
- **touched:** `crates/openbmp-plume/src/base.rs` (new),
  `crates/openbmp-runner/src/aero.rs` (decorator wiring), scenario
  `[aero.plume.base]`, `data/plume/**`, `requirements.toml`
- **approach:** §3.2 decorator + §3.3 correlation surface (digitized open
  curves, provenance-pinned) with analytic asymptotes; power-off Hoerner term
  reproduced exactly at `NPR→0`.
- **acceptance:**
  - off by default; goldens byte-identical
  - power-off identity exact (bit) at `NPR→0`; armed-engines-off increment
    identically zero
  - regime trend within ±20% of the pinned published band (tolerance table)
  - engine-out case produces the expected induced-moment sign through a scenario
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — vehicle-intrinsic drag correction.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** correlation-grade; no reacting base flow; magnitudes not
  facility-validated (§1.2 item 2).

### WP-15.3 — PIFS onset + separated-zone increments

- **goal:** Promote the T1 PIFS indicator to physics: free-interaction onset
  criterion (eq. 3.4.3), separation-front station tracking, separated-zone
  pressure-plateau `ΔC_N`/`ΔC_m` increments, and a separated-zone heating
  multiplier handed to the doc `04` deck.
- **fidelity_tier:** T3
- **depends_on:** [WP-15.2]
- **new_crates:** none
- **touched:** `crates/openbmp-plume/src/pifs.rs` (new), runner increment
  wiring, scenario `[aero.plume.base]` extension, `tests/expected/**`
- **approach:** §3.4 (PIFS). Onset from `δ_j` vs incipient-separation angle;
  plateau increments applied aft of the tracked front.
- **acceptance:**
  - off by default; goldens byte-identical
  - separation front monotone-forward with altitude on a Saturn-V-like
    synthetic geometry; onset altitude inside the NTRS 20110004015-derived band
    (trend tolerance table)
  - increments vanish below onset (identity)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`; `research` for the trend anchor only
- **dual_use_note:** far from line.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** onset/trend fidelity; separated-zone unsteadiness (buffet
  spectra) not modeled — doc `02`/`03` buffet ceiling applies.

### WP-15.4 — Base convective + radiative heating

- **goal:** Base thermal environment: reverse-jet convective correlation keyed
  to the merge state (eq. 3.4.1) + gray-gas cylindrical-plume radiation with
  closed-form view factor (eq. 3.4.2), per-propellant emissivity bands carried
  as UQ. Feeds doc `04`'s distributed deck and TPS sizing.
- **fidelity_tier:** T3
- **depends_on:** [WP-15.1, WP-04.2-a (`openbmp-thermochem` skeleton, doc 04)]
- **new_crates:** none (extend `openbmp-plume`; emissivity/correlation data
  under `data/plume/base_heating/` with provenance)
- **touched:** `crates/openbmp-plume/src/base_heating.rs` (new),
  `crates/openbmp-runner/src/aerothermal.rs` (sink wiring), scenario
  `[aero.plume.base_heating]`, `data/plume/**`
- **approach:** §3.4. SLS LENS-II→correlation methodology (NTRS 20150002948) as
  the methodological template; trend targets, labelled magnitudes.
- **acceptance:**
  - off by default; goldens byte-identical
  - view factor vs closed form `< 1e-10`; radiative limit cases exact
  - convective peak tracks the merge altitude and scales with engine count
    (trend tolerance table; magnitudes explicitly labelled non-validated)
  - emissivity/recirculation-fraction dispersions registered with doc `11`
    margins
  - all `13` §2 gates green
- **validation_label:** `validated-toy`; `research` for trend anchors
- **dual_use_note:** far from line.
- **est_effort:** 4–6 weeks
- **parity_ceiling:** gray-gas + correlation grade; no spectral radiation, no
  afterburning chemistry, no facility-correlated magnitudes (§1.2 items 1–2).

### WP-15.5 — SRP reduced drag-replacement model + forward-only tripwire

- **goal:** The headline correction for flown scenarios: smooth drag-knockdown
  reduced model (eq. 3.5.1) with per-layout `(k, f_p)`, first-order
  `ΔC_N/ΔC_m`, correct power-off and thrust-dominated limits — plus the new
  tripwire test proving the SRP/plume increment input types cannot carry a
  coordinate/aimpoint field.
- **fidelity_tier:** T4
- **depends_on:** [WP-15.1]
- **new_crates:** none
- **touched:** `crates/openbmp-plume/src/srp.rs` (new), runner decorator wiring
  for descent phases, scenario `[aero.plume.srp]`,
  `crates/openbmp-testkit/tests/` (new plume forward-only tripwire),
  `requirements.toml`
- **approach:** §3.5 reduced model; calibration constants pinned to the public
  Jarvinen & Adams / UPWT trends with provenance.
- **acceptance:**
  - off by default; goldens byte-identical
  - limits exact: `C_T→0` recovers power-off `C_A`; large `C_T` →
    `C_A,total → C_T + f_p·C_A,off` (tolerance table)
  - regime map reproduces preservation-vs-collapse per layout (§5.2)
  - **tripwire green:** increment input surface compiles-fail / rejects any
    coordinate-bearing field (test added this PR)
  - all `13` §2 gates green
- **validation_label:** `validated-toy` (the `research` claim waits for WP-15.6)
- **dual_use_note:** serves the permitted descent-to-state capability only;
  consumes `(M, α, C_T)`; new proving test added per `00` §6.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** smooth engineering knockdown; no unsteady SRP dynamics;
  not yet benchmark-anchored.

### WP-15.6 — SRP increment database: FUN3D portal + UPWT ingestion + UQ

- **goal:** Ingest the public FUN3D Retropropulsion Data Portal solutions and
  digitized UPWT runs into `openbmp-aerodb` tables `ΔC_A/ΔC_N/ΔC_m (M, α, C_T)`
  with per-entry RSS margins and CFD-vs-test scatter as model-form margin;
  runtime lookup with fail-closed envelope exits falling back to WP-15.5.
  This is the `research`-label anchor for the dimension.
- **fidelity_tier:** T4
- **depends_on:** [WP-15.5, WP-03.4 (aerodb skeleton), WP-03.6 (RSS UQ)]
- **new_crates:** none (extend `openbmp-aerodb` schemas + `openbmp-plume` lookup)
- **touched:** `crates/openbmp-aerodb/**` (NPR/C_T axes, SRP deck schema),
  `crates/openbmp-plume/src/srp.rs`, `data/plume/srp/**` (+ `provenance.md`,
  SHA pins, portal terms recorded), `tests/expected/**`
- **approach:** §3.5 T4-b; doc `03` ingestion/run-matrix machinery, not a new
  pipeline.
- **acceptance:**
  - off by default; goldens byte-identical
  - node-exact round-trip of an ingested portal solution; envelope exit
    telemetered + falls back to the reduced model (fail-closed test)
  - **public benchmark:** cold-gas `C_A,total(C_T)` within ±15% (central) /
    ±25% (tri-nozzle) of the published UPWT data (tolerance table)
  - per-entry margins flow into a doc `11` MC dispersion case
  - all `13` §2 gates green
- **validation_label:** `research` (cold-gas benchmark); hot-plume use stays
  `validated-toy` with inflated margins, stated in the deck label
- **dual_use_note:** descent-coupled only; flight-condition axes only; tripwire
  from WP-15.5 covers the surface.
- **est_effort:** 4–6 weeks
- **parity_ceiling:** cold-gas subscale anchor; flight-Re, hot-plume, and
  flight-enthalpy increments are extrapolation with documented inflation
  (§1.2 item 3).

### WP-15.7 — SRP heating shield + stability increments

- **goal:** Bounded convective-heating multiplier `κ_q(C_T)` on plume-covered
  aerothermal nodes during retro burns (the entry-burn shielding effect), and
  static `ΔC_m(α, C_T)` stability increments with a bounded unsteady dispersion
  into MC — completing the SRP environment set.
- **fidelity_tier:** T4
- **depends_on:** [WP-15.5, WP-15.4]
- **new_crates:** none
- **touched:** `crates/openbmp-plume/src/srp.rs`,
  `crates/openbmp-runner/src/aerothermal.rs`, `data/plume/srp/**`, doc 11
  margin registration
- **approach:** §3.5 T4-c; trend-calibrated from the public SRP CFD/IR-imagery
  conclusions (NTRS 20170008725 as methodology anchor), carried as an interval.
- **acceptance:**
  - off by default; goldens byte-identical
  - `κ_q → 1` exactly at `C_T→0`; multiplier bounded in `(0,1]` (property test)
  - shield interval and `ΔC_m` dispersion registered as doc `11` margins
  - entry-burn scenario shows reduced integrated heat load vs unshielded run
    (documented as trend, not magnitude)
  - all `13` §2 gates green
- **validation_label:** `validated-toy` (no public quantitative heating
  benchmark exists; stated plainly)
- **dual_use_note:** descent-coupled, vehicle-intrinsic.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** interval-grade shielding; no coupled plume-shock-layer
  radiation; flight IR data not public in usable form.

### WP-15.8 — Source-flow plume flowfield + impingement surface integrator

- **goal:** The RAMP2/PLIMP-class core: Simons source-flow far field (eq.
  3.6.1, mass-conserving, with backflow correction) + panel surface integrator
  (eq. 3.6.2) over the doc `03` mesh with BVH occlusion, producing body-frame
  force/moment + per-panel heating maps; CFD-deck ingestion through the same
  receiver interface.
- **fidelity_tier:** T5
- **depends_on:** [WP-15.1, WP-03.1 (panel mesh + BVH)]
- **new_crates:** none (extend `openbmp-plume`)
- **touched:** `crates/openbmp-plume/src/{flowfield.rs,impingement.rs}` (new),
  `crates/openbmp-aerodb` (plume-deck schema), `data/plume/flowfields/**`,
  `tests/expected/**`
- **approach:** §3.6; PLIMP handbook methodology (NTRS 19940008302, 19940010268)
  clean-room in Rust; fixed-order panel reduction, preallocated scratch.
- **acceptance:**
  - off by default; goldens byte-identical
  - Simons mass conservation `< 1e-9` rel; centerline slope −2 `< 1e-6`
  - flat-plate momentum closure `< 1%`; free-molecular limit matches
    `knudsen.rs` Schaaf-Chambré `< 1e-12`
  - an ingested SU2 jet deck drives the same integrator (interface test) with
    provenance
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** loads/heating on own structure; far from line.
- **est_effort:** 5–8 weeks
- **parity_ceiling:** far-field source flow + Newtonian-class surface relations;
  near-field shock structure and reacting plumes are ingested-deck territory.

### WP-15.9 — Rarefied/DSMC handoff for impingement

- **goal:** `Kn_GLL` breakdown detection on the plume field, bridging through
  the existing `knudsen.rs` functions in the transition band, and ingestion of
  SPARTA DSMC reference decks for the rarefied regime — with the handoff
  placement justified by the code-to-code crossover.
- **fidelity_tier:** T5
- **depends_on:** [WP-15.8, WP-03.9 (SPARTA pipeline)]
- **new_crates:** none
- **touched:** `crates/openbmp-plume/src/handoff.rs` (new), `data/plume/dsmc/**`
  (+ provenance), `tests/expected/**`
- **approach:** §3.6 handoff; threshold ≈ 0.05 with the SPARTA comparison
  setting the final value; regime choice telemetered.
- **acceptance:**
  - off by default; goldens byte-identical
  - continuum/bridge/free-molecular panel loads continuous across the handoff
    (no jump > bridge-band tolerance)
  - SPARTA code-to-code case documents the crossover and pins the threshold
    (tolerance table + provenance)
  - all `13` §2 gates green
- **validation_label:** `research` (SPARTA code-to-code)
- **dual_use_note:** far from line.
- **est_effort:** 3–5 weeks
- **parity_ceiling:** engineering bridging; no in-repo DSMC; deck coverage
  limited to the run matrix the user provisions.

### WP-15.10 — Staging / hot-staging / ullage-retro impingement wiring

- **goal:** Wire T5 impingement into the lifecycle events: upper-engine start
  against a proximate booster (hot-staging transient pressure/heating on the
  forward dome using the doc `05` start transient), ullage/retro motor firings
  during separation, RCS self-impingement — as transient force/heating inputs
  to the separation dynamics and loads.
- **fidelity_tier:** T5
- **depends_on:** [WP-15.8, WP-01.* (separation-as-joint-release, doc 01),
  WP-05.4-a (transient chamber, doc 05)]
- **new_crates:** none
- **touched:** `crates/openbmp-runner/src/` (separation-event wiring),
  `crates/openbmp-plume/src/impingement.rs`, scenario
  `[propulsion.plume.impingement]`, a hot-staging variant of the Phalcon-9
  staging scenario, `tests/expected/**`
- **approach:** §3.6 use cases; superposes on the doc `03` proximity deck
  (plume-free baseline + plume increment, no double counting — documented
  split).
- **acceptance:**
  - off by default; goldens byte-identical
  - hot-staging transient produces bounded, mesh-resolved dome loads; impulse
    consistency vs analytic source-flow bound (tolerance table)
  - ullage/retro case alters separation relative motion as expected through a
    scenario (doc 01 joint-release consumer test)
  - no double counting with the proximity deck (superposition identity test)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** vehicle-intrinsic separation physics; far from line.
- **est_effort:** 4–6 weeks
- **parity_ceiling:** engineering transients; no coupled moving-mesh CFD of the
  separation event (Loci/CHEM-class results ingestible by users with access).

### WP-15.11 — Plume-surface interaction & ground effect for landing burns

- **goal:** T6: jet-on-ground-plane footprint (pressure + heat flux on the
  pad plane) and vehicle ground-effect increments `ΔF_A/T = g(h/d_e, NPR, n)`
  with cushion/suckdown/fountain regimes — the gas-side PSI capability the
  doc `14` touchdown dynamics consume. Ejecta/erosion stays an ingestion hook
  capped `experimental`.
- **fidelity_tier:** T6
- **depends_on:** [WP-15.8 (integrator), WP-15.2 (base model interplay)]
- **new_crates:** none
- **touched:** `crates/openbmp-plume/src/ground_effect.rs` (new), runner wiring
  keyed to an AGL scalar, scenario `[aero.plume.ground_effect]`,
  `data/plume/ground_effect/**` (digitized curves + provenance)
- **approach:** §3.7; Donaldson-Snedeker impingement structure +
  Saddington-Knowles regime curves; multi-engine fountain from cluster
  geometry.
- **acceptance:**
  - off by default; goldens byte-identical
  - increments vanish for `h/d_e` above the documented envelope (identity)
  - regime signs/bands match the pinned published curves (tolerance table)
  - input surface carries only relative AGL + plume state (asserted by the
    WP-15.5 tripwire extended to this type)
  - a landing-burn scenario consumes the increment end-to-end (with doc `14`)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`; ejecta hook `experimental`
- **dual_use_note:** relative-height input only; pad footprint is a loads
  output; no site/coordinate surface (§6 item 3).
- **est_effort:** 4–6 weeks
- **parity_ceiling:** gas-side only; cratering/ejecta/granular physics not
  modeled (§1.2 item 4); hover-jet curves are subscale/cold-flow anchored.

### WP-15.12 — Phalcon-9 power-on descent capstone + external validity

- **goal:** Turn the corrected physics on for the flown profile: a Phalcon-9
  boostback variant with `[aero.plume.base]` + `[aero.plume.srp]` armed —
  power-on ascent base drag, SRP drag collapse through the entry burn, heating
  shield during the burn — with a documented before/after physics delta and a
  LOCAL-only webcast-telemetry sanity check of the entry-burn deceleration
  profile. The honest demonstration that the previously wrong phases are now
  physically defensible.
- **fidelity_tier:** T2+T4 (integration)
- **depends_on:** [WP-15.2, WP-15.5, WP-15.6, WP-15.7]
- **new_crates:** none
- **touched:** `scenarios/phalcon9/` (new opt-in variant + provenance),
  `crates/openbmp-cli/tests/` (e2e), docs cross-links
- **approach:** scenario-level integration; the baseline
  `phalcon9-orbit-boostback.toml` stays untouched and byte-identical; the
  variant opts in. External validity per
  `docs/external-telemetry-validation.md` — local, never committed, never CI.
- **acceptance:**
  - baseline scenario goldens byte-identical (variant is a new file)
  - entry-burn axial force in the variant tracks `C_T + collapsed C_A`
    (assertion vs the WP-15.5/15.6 model, tolerance table)
  - before/after delta documented in the scenario provenance (deceleration,
    integrated heat load, base-drag history) — reported as model behavior, not
    flight accuracy
  - LOCAL telemetry comparison documented as a workflow only; no committed data
  - all `13` §2 gates green
- **validation_label:** `validated-toy` (the synthetic vehicle bounds the claim;
  the SRP model inside it carries its own `research` anchor from WP-15.6)
- **dual_use_note:** descent-to-state demo on the existing locked profile;
  nothing new at the boundary.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** demonstrates corrected *physics architecture* on a
  synthetic vehicle; no claim of matching any real booster's entry environment.

---

## 10. References

**Base flow / power-on base pressure / PIFS**
- H. H. Korst, "A Theory for Base Pressures in Transonic and Supersonic Flow,"
  *J. Applied Mechanics* 23, 1956 (component analysis behind eq. 3.3.1).
- S. F. Hoerner, *Fluid-Dynamic Drag*, 1965 (power-off base drag; the existing
  `buildup.rs` correlation source).
- C. E. Brazzel et al., base pressure with exhaust-jet correlations (AEDC/AIAA
  jet-on base-pressure literature underlying the §3.3 table).
- NASA, "Plume Induced Flow Separation — Saturn V CFD best-practice study,"
  NTRS 20110004015 (PIFS onset/progression anchor, §3.4/§5.2).
- NASA, "Space Launch System Base Heating Test: Experimental Setup at CUBRC
  LENS II" (ATA-002 series), NTRS 20150002948 (short-duration test →
  correlation methodology template, §3.4).
- SpaceX/Leland et al., "Exascale simulation of the 33-engine Super Heavy base
  flow," arXiv:2505.07392 (multi-engine merge/reverse-jet phenomenology).
- NASA Advanced Supercomputing, SC24 research showcase, project 24: MSFC
  Loci/CHEM support of Starship hot-staging (www.nas.nasa.gov/SC24).

**Plume flowfields / impingement**
- G. A. Simons, "Effect of Nozzle Boundary Layers on Rocket Exhaust Plumes,"
  *AIAA Journal* 10(11), 1972 (eq. 3.6.1 + backflow correction).
- F. P. Boynton, "Highly Underexpanded Jet Structure: Exact and Approximate
  Calculations," *AIAA Journal* 5(9), 1967.
- NASA, *Low Altitude Plume Impingement Handbook* (PLIMP/RAMP2 methodology),
  NTRS 19940008302 and 19940010268 (the §3.6 architecture).
- C. duP. Donaldson & R. S. Snedeker, "A study of free jet impingement,"
  *J. Fluid Mechanics* 45, 1971 (ground-plane impingement structure, §3.7).
- A. J. Saddington & K. Knowles, "A review of out-of-ground-effect
  propulsion-induced interference on STOVL aircraft," *Progress in Aerospace
  Sciences* 41, 2005 (cushion/suckdown/fountain regimes, §3.7).
- G. A. Bird, "Breakdown of Translational and Rotational Equilibrium in Gaseous
  Expansions," *AIAA Journal* 8(11), 1970; I. D. Boyd et al., "Predicting
  Failure of the Continuum Fluid Equations in Transitional Hypersonic Flows,"
  *Physics of Fluids* 7, 1995 (`Kn_GLL`, §3.6 handoff).
- G. F. Schaaf & P. L. Chambré, *Flow of Rarefied Gases*, Princeton, 1961
  (free-molecular limit; already implemented in `knudsen.rs`).
- NASA Plume-Surface Interaction (PSI) project publications, NTRS (ejecta/
  erosion research status, §3.7 ceiling).

**Supersonic retropropulsion**
- P. W. Jarvinen & R. H. Adams, "The Aerodynamic Characteristics of Large
  Angled Cones with Retrorockets," NASA contract study, 1970 (preservation vs
  replacement regimes, §3.5).
- A. M. Korzun & R. D. Braun, "Performance Characterization of Supersonic
  Retropropulsion for High-Mass Mars Entry Systems," *J. Spacecraft & Rockets*
  47(5), 2010.
- S. A. Berry, M. N. Rhode, K. T. Edquist et al., "Supersonic Retropropulsion
  Experimental Results from the NASA Langley Unitary Plan Wind Tunnel,"
  AIAA 2011-3489 (and the companion 2011 AIAA SRP CFD-validation papers indexed
  at fun3d.larc.nasa.gov) — the public cold-gas benchmark (§5.4).
- NASA FUN3D Retropropulsion Data Portal, data.nas.nasa.gov/fun3d (public CFD
  solutions ingested in WP-15.6).
- NASA, "Commercial Supersonic Retropropulsion Flight Data Analysis" (the
  NASA–SpaceX Falcon 9 SRP partnership: flight reconstruction + CFD + airborne
  IR imagery), NTRS 20170008725 (methodology anchor, §3.5/§5.4).

**Thermochemistry / radiation**
- S. Gordon & B. J. McBride, "Computer Program for Calculation of Complex
  Chemical Equilibrium Compositions and Applications" (CEA), NASA RP-1311,
  1994/1996.
- G. P. Sutton & O. Biblarz, *Rocket Propulsion Elements*, 9th ed., Wiley
  (nozzle/plume relations; per-propellant plume radiation discussion).
- R. Siegel & J. R. Howell, *Thermal Radiation Heat Transfer* (view factors,
  gray-gas model, §3.4.2).

**Open-source tools:** SU2 (LGPL-2.1), OpenFOAM (GPL-3.0, offline), SPARTA
(GPL-3.0, offline), Cantera (BSD-3), CEA/CEARUN outputs, FUN3D data portal
(public NASA data), PLIMP/RAMP2 NTRS reports (public domain US government
works).

**Upstream OpenBMP anchors:** `docs/verification.md`,
`docs/standards-posture.md`, `docs/safety-boundaries.md`,
`docs/data-provenance.md`, `docs/external-telemetry-validation.md`,
`docs/roadmap.md`, `docs/staging-and-separation.md`, `00-overview.md`,
`13-agent-execution-playbook.md`.

---

*Companion documents:* `00-overview.md` (constitution) ·
`03-aerodynamics-database-and-cfd-coupling.md`, `04-aerothermal-realgas-and-tps.md`,
`05-propulsion-high-fidelity.md` (the deferral triangle this doc closes; hard
dependencies) · `08`, `01`/`02`, `09`, `07`, `11`, `12` (couplings) · doc `14`
(powered descent & touchdown — consumer of T6) · `13-agent-execution-playbook.md`
(process).
