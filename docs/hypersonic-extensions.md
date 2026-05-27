# OpenBMP Hypersonic Extensions

This document describes the hypersonic-regime extensions to the OpenBMP core
architecture. It covers atmospheric models for high-altitude / real-gas
conditions, hypersonic aerodynamic methods, aerothermal heat transfer,
boundary-layer state, continuum-to-rarefied bridging, re-entry trajectory
infrastructure, and the validation suite.

The extensions are designed to plug into the existing trait surfaces defined
in [software-architecture.md](software-architecture.md) while extending the
solver, coupling, validation, and data-package infrastructure needed for
research-grade hypersonic work. They are **research extensions** and are
subject to the additional accept/reject rules in
[safety-boundaries.md § Hypersonic Extensions](safety-boundaries.md#hypersonic-extensions).

## Scope

OpenBMP supports the **physics of hypersonic flight** — the engineering and
academic study of vehicles flying at Mach > 5 — using public, textbook, and
synthetic data. It does **not** ship operational hypersonic-weapon parameter
sets, terminal-evasion logic, penetration aids, or targeting in any phase.
The civilian and academic literature (Anderson, Vinh, Park, Bertin, Hirschel,
NASA technical reports, ESA IXV documentation) is the reference frame.

### Physics regimes covered

| Regime | Mach range | Altitude range (Earth) | Notes |
|---|---|---|---|
| High supersonic | 3–5 | up to ~50 km | Edge of MVP coverage |
| **Low hypersonic** | 5–10 | 30–80 km | Newtonian methods become useful; perfect-gas still ~ok |
| **Moderate hypersonic** | 10–15 | 50–100 km | Real-gas effects significant; aerothermal dominates |
| **High hypersonic** | 15–25+ | 60–120 km | Strong real-gas chemistry; ionization onset; rarefied at top |
| Free molecular | any | > ~150 km | Continuum breaks down; DSMC territory |

### Vehicle classes targeted

OpenBMP hypersonic extensions are **Earth-atmosphere only**. Non-Earth
atmospheres (Mars, Titan, Venus, Jupiter) and interplanetary aerocapture /
aerobraking are out of scope for the foreseeable future. The framework
could in principle host them later via a planet-aware atmosphere registry,
but that path is out of scope and no planetary atmosphere
data ships in the repository.

- Lifting re-entry research vehicles (academic equivalents to ESA IXV,
  NASA X-37B class — flown using public flight-mechanics methods, Earth
  atmosphere only).
- Sounding rockets that touch hypersonic briefly (peak velocity Mach 5–8).
- Capsule-class blunt re-entry bodies for academic studies (Apollo-class
  geometry, Mercury-class, Stardust-class — all Earth re-entries).
- Earth-orbit-decay re-entry studies (uncontrolled and lifting).
- **Reusable second-stage / lifting-body re-entry studies** using
  textbook geometry and the public flight-mechanics literature. The
  high-angle-of-attack lifting attitude with body-flap pitch trim is
  supported through the standard `ControlEffector` trait described in
  [software-architecture.md § Control Effectors](software-architecture.md#control-effectors)
  — body flaps appear as effectors with rate limits, saturation, and
  hinge-moment loads like aerodynamic surfaces on any other vehicle.
  No real fielded reusable-vehicle TPS, mass, or aerodynamic data ships
  in the repository; the cookbook in
  [real-rocket-integration.md](real-rocket-integration.md) walks through
  the lifting-reentry assembly using fictional ARV-Reference numbers.

### Vehicle classes NOT targeted

- Operational HGVs, MaRVs, hypersonic cruise weapons (see safety boundaries).
- Re-entry vehicles whose stated purpose is delivering a payload to a
  real-world ground location.
- Any vehicle whose modeled performance is benchmarked against operational
  miss-distance or impact-dispersion criteria.

## Research Baseline

The hypersonic target is not "RK4 plus hypersonic coefficients". A credible
hypersonic rocket / re-entry simulator needs a layered infrastructure similar
to the public NASA / academic reference frame:

- **Trajectory and mission propagation** comparable in shape to public
  launch / entry trajectory tools such as POST2: multi-segment 3-DOF / 6-DOF
  propagation, event localization, atmosphere / gravity / propulsion model
  replacement, and per-segment fidelity selection.
- **High-enthalpy flow references** from public aerothermodynamics practice:
  continuum CFD, nonequilibrium chemistry, radiation, thermal response, and
  rarefied-flow references are used as offline validation or deck-generation
  sources, not as live OpenBMP solvers.
- **Solver profiles beyond RK4**: high-order explicit adaptive methods for
  smooth trajectories, dense output for event finding, fixed sub-stepping for
  byte-stable regression, and implicit / IMEX methods for stiff chemistry and
  material response.
- **Credibility and uncertainty accounting**: every model and external dataset
  declares its validity envelope, numerical-error evidence, source pedigree,
  uncertainty model, and validation status.

This document therefore treats RK4 as the deterministic baseline integrator,
not as the hypersonic fidelity ceiling.

## Architectural Extension Points

The hypersonic extensions plug into the existing trait surfaces and add the
solver/data infrastructure required to support them:

1. Solver profiles and multiphysics coupling.
2. AtmosphereModel extensions for high-altitude, winds, real-gas, and regime
   state.
3. AeroMethod extensions for hypersonic continuum, bridge, and free-molecular
   methods.
4. `openbmp-aerothermal` for heating, boundary layer, thermal response, and
   ablation toys.
5. Boundary-layer and Knudsen-regime models.
6. Offline high-fidelity reference data packages.

### 0. Solver profiles and multiphysics coupling

The kernel remains deterministic by default, but hypersonic scenarios select a
declared solver profile. The profile is part of the scenario hash and telemetry
header.

```rust
pub enum SolverProfile {
    /// Fixed-step RK4 or fixed-step high-order explicit RK.
    /// Used for byte-stable golden tests and deterministic replay.
    FixedStepExplicit {
        method: ExplicitMethod,
        dt: Duration,
    },

    /// Adaptive explicit RK with embedded error estimate and dense output.
    /// Used for smooth trajectories and event-gradient accuracy.
    AdaptiveExplicit {
        method: ExplicitMethod,
        rtol: f64,
        atol: f64,
        min_dt: Duration,
        max_dt: Duration,
    },

    /// Stiff source-term integration for chemistry / thermal response.
    /// Used as a sub-stepper inside a trajectory step or in research profiles.
    ImplicitSourceTerm {
        method: ImplicitMethod,
        substeps: usize,
        nonlinear_tolerance: f64,
        nonlinear_max_iter: usize,
    },
}

pub enum ExplicitMethod {
    Rk4,
    DormandPrince54,
    DormandPrince853,
    RungeKuttaFehlberg78,
}

pub enum ImplicitMethod {
    ImplicitEuler,
    RosenbrockWanner,
    Bdf,
}
```

Coupling is partitioned and explicit in the scenario:

```text
environment -> aero -> aerothermal -> material_response -> mass/geometry -> dynamics
```

Each coupling edge declares whether feedback is disabled, lagged one kernel
step, sub-iterated to a fixed count, or solved with a profile-gated implicit
coupling method. The default research-safe path is lagged one-step coupling
with telemetry that reports the lag. Fully implicit coupled flow/material
solves are out of scope for OpenBMP as code; OpenBMP may ingest their offline
results through data packages.

### 1. AtmosphereModel — extended with optional real-gas state

```rust
// openbmp-physics::atmosphere
pub trait AtmosphereModel {
    fn sample(&self, q: AtmQuery) -> AtmosphereSample;

    /// Optional: thermodynamic state including real-gas effects, composition,
    /// and continuum regime. Default `None` for ideal-gas-only models.
    fn thermo_state(
        &self,
        q: AtmQuery,
        local_total_temperature: ThermodynamicTemperature,
        characteristic_length: Length,
    ) -> Option<RealGasState> {
        None
    }
}

pub struct RealGasState {
    pub regime: GasRegime,
    pub composition: AirComposition,        // mole fractions
    pub gamma_eff: f64,                     // effective ratio of specific heats
    pub speed_of_sound: Velocity,           // recomputed for real gas
    pub knudsen: f64,                       // Kn at vehicle scale
    pub mean_free_path: Length,
    pub frozen: bool,                       // frozen vs equilibrium chemistry
    pub nonequilibrium: Option<NonequilibriumState>,  // populated by Park 2T
}

pub struct NonequilibriumState {
    pub t_translational_rotational: ThermodynamicTemperature,  // T
    pub t_vibrational_electronic: ThermodynamicTemperature,    // T_v
    pub damkohler_number: f64,              // diagnostic: τ_flow / τ_chemistry
    pub vibrational_relaxation_time: Duration,
}

pub enum GasRegime {
    Continuum,      // Kn < 0.01
    Slip,           // 0.01 ≤ Kn < 0.1
    Transition,     // 0.1 ≤ Kn < 10
    FreeMolecular,  // Kn ≥ 10
}

pub struct AirComposition {
    pub n2: f64,        // diatomic nitrogen, mole fraction
    pub o2: f64,        // diatomic oxygen
    pub n_atomic: f64,  // atomic nitrogen (dissociation product)
    pub o_atomic: f64,  // atomic oxygen
    pub no: f64,        // nitric oxide (intermediate)
    pub argon: f64,
    pub electrons: f64, // for ionization > Mach ~12 cases
}
```

### 2. AeroMethod — pluggable aerodynamic computation

```rust
// openbmp-aero::method
pub trait AeroMethod {
    fn aero_force_moment(&self, ctx: &AeroContext) -> AeroForceMoment;
    fn validity(&self) -> AeroValidity;
}

pub struct AeroValidity {
    pub mach_min: f64,
    pub mach_max: f64,
    pub kn_max: f64,
    pub alpha_max_rad: f64,
    pub beta_max_rad: f64,
}
```

`DeckLookup` is the existing tabulated method. New hypersonic methods:

- `ModifiedNewtonian { geometry: BodyGeometry, cp_max_correction: bool }`
- `TangentCone { geometry: BodyGeometry, base_diameter: Length }`
- `TangentWedge { geometry: BodyGeometry, half_angle_rad: f64 }`
- `LocalInclinationPanels { geometry: PanelMesh, shadowing: bool }`
- `FreeMolecular { geometry: BodyGeometry, accommodation: AccommodationCoeffs }`

A `HybridAeroMethod` dispatches between methods based on (Mach, Kn):

```rust
pub struct HybridAeroMethod {
    /// Continuum, low-Mach (subsonic / transonic / supersonic)
    pub continuum_low_mach: Box<dyn AeroMethod>,
    /// Continuum, high-Mach (Newtonian or local inclination)
    pub continuum_high_mach: Box<dyn AeroMethod>,
    /// Free-molecular limit
    pub free_molecular: Box<dyn AeroMethod>,
    /// Mach at which low/high handoff occurs (default 4.0)
    pub mach_handoff: f64,
    /// Knudsen bridge function for continuum-to-FM blending
    pub bridge: BridgeFunction,
}
```

### 3. New crate: `openbmp-aerothermal`

```text
openbmp-aerothermal/
├── lib.rs
├── stagnation/
│   ├── fay_riddell.rs       // 1958 classic, equilibrium catalytic walls
│   ├── sutton_graves.rs     // engineering simplification
│   └── tauber_sutton.rs     // for higher-energy regimes
├── distributed/
│   ├── reference_enthalpy.rs   // Eckert reference enthalpy method
│   ├── spalding_chi.rs         // turbulent flat plate
│   └── van_driest.rs           // compressible BL
├── boundary_layer/
│   ├── state.rs              // Laminar / Transitional / Turbulent
│   ├── transition.rs         // e^N, Reθ/M_e correlations
│   └── correlations.rs       // skin friction, Stanton number
├── thermal_toy/
│   └── one_d_conduction.rs   // 1-D explicit FD surface temperature
└── radiation/
    └── tauber_sutton_rad.rs  // engineering radiative heating estimate
```

Trait surface:

```rust
pub trait HeatTransferModel {
    fn stagnation(&self, ctx: &AerothermalContext) -> StagnationHeating;
    fn distributed(
        &self,
        ctx: &AerothermalContext,
        station: BodyStation,
        bl_state: BoundaryLayerState,
    ) -> SurfaceHeating;
    fn validation(&self) -> ValidationStatus;
}

pub struct AerothermalContext<'a> {
    pub time: SimTime,
    pub freestream: &'a AtmosphereSample,
    pub real_gas: Option<&'a RealGasState>,
    pub airspeed: Velocity,                 // freestream speed
    pub mach: f64,
    pub geometry: &'a BodyGeometry,
    pub wall_temperature: ThermodynamicTemperature,
    pub wall_catalysis: WallCatalysis,
}

pub struct StagnationHeating {
    pub q_conv: HeatFlux,                  // convective heat flux (W/m²)
    pub q_rad: HeatFlux,                   // radiative heat flux
    pub h_aw: SpecificEnergy,              // adiabatic-wall enthalpy
    pub h_w: SpecificEnergy,               // wall enthalpy
    pub recovery_temp: ThermodynamicTemperature,
}

pub struct SurfaceHeating {
    pub q: HeatFlux,
    pub stanton: f64,                      // Stanton number
    pub skin_friction: f64,                // Cf
}

pub enum WallCatalysis {
    FullyCatalytic,
    NonCatalytic,
    Partial(f64),                          // recombination efficiency [0,1]
}
```

### 4. Boundary layer state

```rust
pub trait BoundaryLayer {
    fn state(&self, station: BodyStation, ctx: &AerothermalContext) -> BoundaryLayerState;
}

pub enum BoundaryLayerState {
    Laminar { reynolds_x: f64 },
    Transitional { intermittency: f64, reynolds_x: f64 },
    Turbulent { reynolds_x: f64 },
}

pub struct EnTransition { pub n_crit: f64 }                  // e^N method
pub struct ReThetaTransition { pub re_theta_crit: f64 }
pub struct EmpiricalTransition { pub re_x_crit: f64 }
```

### 5. Knudsen bridging

```rust
pub trait BridgeFunction {
    /// Returns alpha in [0, 1] where 0 = continuum, 1 = free molecular.
    fn alpha(&self, knudsen: f64) -> f64;
}

pub struct ChengBridge;             // exp(-π/(2 Kn))
pub struct EngerBridge;             // tanh-based
pub struct LinearKnudsenBridge {    // smoothstep over [Kn_lo, Kn_hi]
    pub kn_lo: f64,
    pub kn_hi: f64,
}
```

## Atmosphere — High-Altitude Models

### Layered atmosphere model set

No single atmosphere model covers every hypersonic use case well. OpenBMP uses
a layered registry:

| Model | Role | Repository policy |
|---|---|---|
| US Standard Atmosphere 1976 | deterministic lower/middle-atmosphere baseline | ship public table with provenance |
| NRLMSISE-00 | stable high-altitude baseline from ground to thermosphere | in-house Rust port with direct NASA/CCMC coefficient arrays |
| NRLMSIS 2.x compatibility profile | modern whole-atmosphere / thermosphere selector, including a NO number-density proxy | OpenBMP-derived profile over NRLMSISE-00; official 2.x packages are not vendored |
| HWM14 | horizontal neutral winds for upper/middle/lower atmosphere | in-house Rust port with bundled public HWM14/DWM07 data files |
| Earth-GRAM | engineering atmosphere with mean values and statistical variations | external-reference / validation oracle first; ship only if licensing and provenance are clean |

Scenario authors pick the model and hand-off altitude explicitly. Solar,
geomagnetic, latitude, longitude, local solar time, and wind inputs become part
of the determinism profile.

### NRLMSISE-00 (in-house Rust port)

NRLMSISE-00 (Naval Research Laboratory Mass Spectrometer and Incoherent Scatter
Radar 2000) is a public empirical model for Earth's neutral atmosphere from
the ground to ~1000 km. OpenBMP ships a pure-Rust local evaluator and pulls the
coefficient arrays directly from the NASA/CCMC archived `nrlmsise-00_data.c`
source. The provenance entry records the source URLs, SHA-256 hashes, and the
MIT-licensed Rust implementation used as the evaluator baseline; there is no
runtime crate dependency, no coefficient-file loading, no FFI, and no network
access on the hot path.

Inputs (with sensible defaults for "static" mode):

```rust
pub struct Nrlmsise00Inputs {
    pub year: u16,
    pub day_of_year: u16,
    pub utc_seconds: f64,
    pub altitude: Length,
    pub latitude_rad: f64,
    pub longitude_rad: f64,
    pub local_apparent_solar_time_hours: f64,
    pub f107_average_81day: f64,    // default 150
    pub f107_yesterday: f64,        // default 150
    pub ap_average: f64,            // default 4
}
```

Outputs (number densities + total mass density + temperature):

```rust
pub struct Nrlmsise00Outputs {
    pub n_he: NumberDensity,        // helium
    pub n_o: NumberDensity,         // atomic oxygen
    pub n_n2: NumberDensity,
    pub n_o2: NumberDensity,
    pub n_ar: NumberDensity,
    pub n_h: NumberDensity,
    pub n_n: NumberDensity,
    pub n_o_anomalous: NumberDensity,
    pub mass_density: MassDensity,
    pub neutral_temperature: ThermodynamicTemperature,
    pub exospheric_temperature: ThermodynamicTemperature,
}
```

Two convenience modes:

- **`Nrlmsise00Static`** — ignores time/latitude/longitude/solar inputs and
  uses fixed mid-conditions (`F10.7 = 150`, `Ap = 4`, equator, noon, equinox).
  Deterministic by construction. Default for hypersonic scenarios that don't
  care about diurnal variation.
- **`Nrlmsise00Full`** — accepts full inputs from the scenario.

For altitudes 0–86 km the model agrees with US Standard 1976 within a few
percent; the scenario can stitch them deterministically using a smooth
hand-off.

### NRLMSIS 2.x and HWM14

NRLMSIS 2.1 is the current public CCMC-hosted MSIS-family reference as of
April 2026. It extends NRLMSIS 2.0 by adding nitric-oxide number density from
approximately 73 km to the exobase while retaining the standard MSIS input set
(date/time, geodetic position, local solar time, F10.7, and Ap). The official
NRLMSIS 2.x packages inspected for OpenBMP carry academic/non-commercial terms,
so they are not vendored into this Apache-2.0/MIT repository.

OpenBMP therefore exposes `atmosphere.kind = "nrlmsis2_compat"` as a
compatibility profile rather than a vendored official model. It uses the local
NRLMSISE-00 coefficient evaluator as the baseline, applies bounded
upper-atmosphere density/temperature corrections from the same MSIS-family
inputs, and reports a deterministic nitric-oxide proxy for 2.x-style
composition studies. Provenance records this as an OpenBMP-derived profile.

HWM14 is the companion empirical horizontal-wind model. It provides zonal and
meridional winds as a function of latitude, longitude, time, altitude, and Ap.
OpenBMP ships a deterministic Rust port of the public HWM14 quiet-time
evaluator plus DWM07 disturbance winds for `wind.kind = "hwm14"`. The three
published data files are bundled under `data/wind/hwm14/`; the provenance
record pins the source package, data hashes, verification driver, and
`checkhwm14` tolerance.

### Earth-GRAM and JB-2008 (deferred)

Earth-GRAM is an engineering-oriented atmosphere that estimates mean values and
statistical variations of Earth atmospheric properties. It is useful as a
reference frame for uncertainty envelopes and Monte Carlo density perturbations,
but OpenBMP should treat it as an external-reference oracle first because the
licensing and redistribution story differs from a clean-room Rust port.

Jacchia-Bowman 2008 is similar in spirit to the MSIS family for thermospheric
density, more useful above roughly 200 km than in the denser re-entry corridor.
Add only if a hypersonic scenario specifically requires it and provenance is
clear.

### Real-gas thermodynamics

For temperatures above ~600 K we cannot assume `gamma = 1.4`. The
`equilibrium_air` module computes:

- Effective specific-heat ratio `gamma_eff(T, p)` from public Tannehill /
  Mugalev correlations (5- to 11-species air).
- Speed of sound corrected for real-gas effects.
- Composition (mole fractions of N2, O2, NO, N, O, Ar, e⁻) under chemical
  equilibrium.
- Frozen vs equilibrium switch based on a Damköhler-number heuristic.

NASA CEA is a useful public validation oracle for equilibrium composition,
shock-tube, and thermodynamic-property checks. OpenBMP does not link CEA or
ship CEA as runtime code; it may store CEA-derived reference outputs only when
the input deck, CEA version, output hash, and provenance record are committed
for validation.

```rust
pub trait EquilibriumAir {
    fn composition(&self, t: ThermodynamicTemperature, p: Pressure) -> AirComposition;
    fn gamma_eff(&self, t: ThermodynamicTemperature, p: Pressure) -> f64;
    fn speed_of_sound(&self, t: ThermodynamicTemperature, p: Pressure) -> Velocity;
}

pub struct TannehillEquilibriumAir;     // 5-species, public correlation
pub struct MugalevEquilibriumAir;       // 11-species, public correlation
```

### Nonequilibrium thermochemistry — Park two-temperature model

The Park two-temperature formulation is a public academic standard; the
reaction sets shipped (Park'87, Park'90, Park'93) are textbook material.
OpenBMP does not ship operationally-tuned variants of any reaction set,
and the validation suite is restricted to academic shock-tube and
stagnation-point textbook problems with publicly documented reference
solutions. See [safety-boundaries.md § Hypersonic
Extensions](safety-boundaries.md#hypersonic-extensions) for the full
reject list.

Above approximately Mach 10–12, the residence time of a fluid element in
the shock layer is shorter than the relaxation time for vibrational
excitation and chemical reactions. Equilibrium air thermodynamics breaks
down. The standard textbook formulation is **Park's two-temperature
model** (Park 1990, *Nonequilibrium Hypersonic Aerothermodynamics*), used
in essentially every published hypersonic-aerothermal code that handles
nonequilibrium effects.

The model carries two temperatures:

- `T` — translational and rotational temperature (equilibrate quickly).
- `T_v` — vibrational and electronic temperature (relax more slowly via
  Landau-Teller-type processes).

Reaction rate constants for endothermic dissociation reactions are
evaluated at a geometric-mean temperature `T_a = sqrt(T · T_v)`,
reflecting that bond-breaking depends on both translational and
vibrational energy. Vibrational energy evolves via:

```text
dE_v/dt = sum_s ρ_s (E_v_eq(T) - E_v(T_v)) / τ_s   +   coupling source terms
```

where `τ_s` is the species-specific Landau-Teller relaxation time
(Millikan-White correlation with Park's high-temperature correction).

Species continuity equations track composition:

```text
dY_s/dt = ω̇_s / ρ                     (Y_s = mass fraction)
ω̇_s = M_s sum_r (ν_s,r,prod - ν_s,r,react) (k_f,r [reactants] - k_b,r [products])
```

The reaction rate constants `k_f, k_b` come from a published reaction set.

#### Trait surface

```rust
pub trait NonequilibriumAir {
    /// Forward and backward reaction rate constants at the given (T, T_v).
    fn reaction_rates(
        &self,
        t_tr: ThermodynamicTemperature,
        t_v: ThermodynamicTemperature,
    ) -> ReactionRates;

    /// Species production rates given current composition and rates.
    fn species_derivative(
        &self,
        composition: &AirComposition,
        rates: &ReactionRates,
        density: MassDensity,
    ) -> CompositionDerivative;

    /// Vibrational energy relaxation rate (Landau-Teller form).
    fn vibrational_relaxation(
        &self,
        t_tr: ThermodynamicTemperature,
        t_v: ThermodynamicTemperature,
        composition: &AirComposition,
        density: MassDensity,
    ) -> VibrationalEnergyDerivative;

    /// Damköhler number = τ_flow / τ_chemistry — diagnostic only.
    fn damkohler(&self, ctx: &FlowContext) -> f64;

    fn validation(&self) -> ValidationStatus;
}

pub struct ParkTwoTemperatureModel {
    pub reaction_set: ParkReactionSet,
    pub vibrational_relaxation_model: VibrationalRelaxationModel,
}

pub enum ParkReactionSet {
    /// Park 1987 — 5-species (N2, O2, NO, N, O), ~17 reactions.
    Park87,
    /// Park 1990 — 11-species incl. ions and electrons (textbook).
    Park90,
    /// Park 1993 — updated rate coefficients.
    Park93,
}

pub enum VibrationalRelaxationModel {
    /// Classical Landau-Teller using Millikan-White correlation.
    MillikanWhite,
    /// Park's high-temperature correction (T > 8000 K) on top of MW.
    MillikanWhitePark,
}
```

#### Coupling with the rest of the architecture

The `RealGasState` struct (above) includes an
`Option<NonequilibriumState>` field. When the scenario selects a
nonequilibrium model, the atmosphere extension hook returns a populated
`nonequilibrium` field. Aerothermal models that consume two-temperature
state (Fay-Riddell with nonequilibrium edge state, or finite-rate
catalytic walls) read it; equilibrium-only aerothermal models ignore it
and rely on the equilibrium fields.

The composition vector evolves in time according to the species
derivative integrated alongside the rigid-body state. Because the
chemistry timescales can be much shorter than the trajectory step, the
nonequilibrium integration uses a sub-stepped implicit Euler scheme
inside the kernel step, with a fixed sub-step count declared in the
scenario (becomes part of the determinism profile). For very stiff
regimes, an optional Rosenbrock-Wanner sub-step variant is available
behind a profile flag.

#### Reaction sets shipped

Reaction-rate coefficient tables for Park'87, Park'90, and Park'93 ship
in `data/realgas/park_*.toml` with:

- Source citation in `provenance.md`.
- Stoichiometric coefficient matrix.
- Arrhenius parameters (A, n, E_a) per reaction per direction.
- Validity range (temperature, composition) noted per table.

All coefficients come from public sources (Park's textbook, NASA TM/TR
publications, refereed journal articles). No operational tunings or
restricted nonequilibrium datasets are imported.

#### Validation cases for Park

- **Shock-layer relaxation behind a normal shock at Mach 15** — public
  textbook problem (Park 1990, Anderson 2019) with documented
  composition / temperature histories. Pass criterion: agreement with
  published reference within 5 % on `T_v` peak and equilibrium
  composition.
- **One-dimensional shock-tube finite-rate flow** — academic textbook
  problem with known closed-form limits (frozen and equilibrium).
- **Damköhler-number diagnostic** — across a hypersonic stagnation point
  at Mach 25, 80 km altitude, the Damköhler number should fall in the
  `O(1)` regime where Park-2T applies; verify the reported `Da` against
  hand calculations.

#### Determinism notes for Park

Nonequilibrium integration is the most demanding part of the hypersonic
stack from a determinism standpoint. Specific rules:

1. Sub-step count and sub-step integrator are declared in the scenario
   and cannot vary per run.
2. Implicit-Euler iteration tolerance and maximum iteration count are
   declared and become part of the determinism profile.
3. Reaction-rate coefficient tables are versioned in
   `data/realgas/park_*.toml`; the version is part of the scenario
   schema, and the kernel refuses to run with a version it does not
   recognise.
4. Vibrational relaxation time floors (e.g., minimum allowed `τ_s` to
   prevent stiffness-induced numerical noise) are documented and fixed
   per profile.

## Aerodynamics — Hypersonic Methods

### Modified Newtonian flow

For body local inclination angle `θ` measured from the freestream direction:

```text
Cp(θ) = Cp_max · sin²(θ)              for θ ∈ [0, π/2]
Cp(θ) = 0                              for θ < 0 (shadowed)
```

`Cp_max` is the stagnation-point pressure coefficient behind a normal shock:

```text
Cp_max = (p_t,2 - p_∞) / q_∞
       = (2/(γ M_∞²)) · [ ((γ+1)² M_∞² / (4 γ M_∞² - 2(γ-1)))^(γ/(γ-1))
                         · ((1 - γ + 2 γ M_∞²) / (γ + 1))      - 1 ]
```

For `M_∞ → ∞` and `γ = 1.4`, `Cp_max ≈ 1.838`. The "modified" in modified
Newtonian uses this real-gas-corrected `Cp_max` rather than the classical
Newtonian value of 2.

Implementation gives accurate engineering pressure distributions for blunt
bodies (spheres, blunt cones) at high Mach without needing tabulated decks.

### Tangent-cone and tangent-wedge

For bodies that locally resemble cones (axisymmetric) or wedges (2-D):

```text
Tangent cone:    Cp at each point = Cp(cone) for the local cone half-angle.
Tangent wedge:   Cp at each point = Cp(wedge) for the local wedge angle.
```

Cone and wedge `Cp` come from Taylor-Maccoll (cone) and the oblique-shock
relations (wedge). Both have closed-form solutions for perfect gas; OpenBMP
ships analytic implementations for `γ = 1.4` and a real-gas variant that
uses the equilibrium-air module.

### Local inclination panel methods

For non-axisymmetric bodies, decompose the surface into panels with normals
`n̂_i` and apply Modified Newtonian per-panel, with shadowing logic that zeroes
`Cp` for panels whose normals point away from the freestream.

```rust
pub struct LocalInclinationPanels {
    pub geometry: PanelMesh,           // triangulated body surface
    pub method: PanelMethod,           // Newtonian / Tangent-cone / Hybrid
    pub shadowing: bool,
}

pub struct PanelMesh {
    pub vertices: Vec<Position3<Body>>,
    pub triangles: Vec<[u32; 3]>,
}
```

Force and moment integration:

```text
F = Σ panels  Cp · q · A_panel · n̂_panel
M = Σ panels  r_panel × ( Cp · q · A_panel · n̂_panel )
```

### Hypersonic similarity

For studies of aerodynamic scaling we expose hypersonic-similarity helpers:

```rust
pub fn hypersonic_similarity_parameter(mach: f64, body_slenderness_rad: f64) -> f64 {
    mach * body_slenderness_rad
}
```

Useful for textbook problems where pressure distributions for slender bodies
collapse onto a single curve when plotted against `K = M·θ`.

### Free-molecular aero

In the free-molecular limit (`Kn → ∞`), forces come from individual molecule
collisions with the surface. The drag coefficient on a flat plate at angle
`α` to the freestream is (Schaaf & Chambré formula):

```text
C_D = 2 · (σ_t · sin²α + σ_n · cos²α · ratio_of_thermal_speeds + …)
```

Implemented via the `FreeMolecular` aero method with documented
accommodation coefficients (default `σ_t = σ_n = 1.0` for fully diffuse
walls; user-tunable per material).

```rust
pub struct AccommodationCoeffs {
    pub tangential: f64,    // σ_t in [0, 1]
    pub normal: f64,        // σ_n in [0, 1]
}
```

### Hybrid dispatch

The `HybridAeroMethod` (above) combines:

1. The continuum-low-Mach method (deck lookup or aero deck).
2. The continuum-high-Mach method (Modified Newtonian).
3. The free-molecular method.

It blends them based on Mach (low/high handoff) and Knudsen number
(continuum/FM bridging). The blend functions are deterministic and the
scenario must declare them explicitly.

## Aerothermal Heat Transfer

### Fay-Riddell stagnation-point heating (1958)

The classic equilibrium-air, catalytic-wall, axisymmetric-stagnation-point
formula:

```text
q_w = 0.94 · (ρ_w μ_w)^0.1 · (ρ_e μ_e)^0.4 · (du_e/dx)^0.5
        · (h_aw - h_w) · [ 1 + (Le^a - 1) · (h_D / h_aw) ]
```

where:
- `ρ_w, μ_w` — wall density and viscosity (functions of `T_w`).
- `ρ_e, μ_e` — boundary-layer edge values (post-shock conditions).
- `du_e/dx` — velocity gradient at the stagnation point; for a sphere of
  radius `R_n` and post-shock pressure `p_e`:
    `du_e/dx = (1/R_n) · sqrt( 2 · (p_e - p_∞) / ρ_e )`
- `h_aw, h_w` — adiabatic-wall and wall enthalpies.
- `Le` — Lewis number; `a = 0.52` (equilibrium catalytic) or `0.63` (frozen
  non-catalytic).
- `h_D` — average dissociation enthalpy.

OpenBMP exposes two paths. The trait implementation assembles a cold-gas
engineering edge state from perfect-gas normal-shock relations. The
research-grade path is `FayRiddell::stagnation_from_edge_state`, where a
verified equilibrium-air solver, CFD deck, or external reference package
supplies `ρ_e`, `μ_e`, `ρ_w`, `μ_w`, `du_e/dx`, and the enthalpies directly.
For neutral dissociated-air edge compositions,
`FayRiddell::with_neutral_composition_enthalpy` maps `AirComposition` species fractions
into the `h_D` term using NIST Chemistry WebBook formation enthalpies for
atomic nitrogen, atomic oxygen, and nitric oxide. Ionised-air edge states
still need an explicitly evaluated enthalpy from the upstream real-gas solver.

### Sutton-Graves engineering simplification

For quick conservative estimates and for analytic-toy validation:

```text
q_w = K · sqrt(ρ_∞ / R_n) · V_∞³
```

with `K = 1.7415e-4` (SI, Earth atmosphere). This requires only freestream
density, nose radius, and freestream velocity — no real-gas iteration.
Useful as a sanity check and as a fast bound for hypersonic mission studies.
`SuttonGraves::stagnation_with_wall_enthalpy_correction` applies the finite-wall
factor `max(0, 1 - h_w / h_aw)` for thermal budgets where wall temperature is
not negligible.

OpenBMP also ships `SuttonGraves::allen_eggers_heating`, a closed-form
trajectory-level diagnostic that integrates the Sutton-Graves correlation over
the Allen-Eggers ballistic-entry profile. It returns peak convective heat flux,
peak-heating altitude, and convective heat load. The validation suite checks the
closed form against direct profile quadrature and brackets the public Stardust
SRC table-20 convective peak / heat-load data from NASA/TP-2006-213486.

### Tauber-Sutton radiative heating

For very-high-energy entries (Mach > ~12) radiative heat flux from the shock
layer becomes significant. Tauber-Sutton (NASA TP 1986) gives an engineering
estimate as a function of `(ρ_∞, V_∞, R_n)` calibrated against detailed
nonequilibrium radiation calculations:

```text
q_rad = C · R_n^a · ρ_∞^b · f(V_∞)
```

The form and coefficients are public; OpenBMP ships the tabulated
piecewise-polynomial form documented in the original paper.

### Distributed surface heating

For non-stagnation points, the **Eckert reference-enthalpy method** computes
local heat flux and skin friction using a reference enthalpy `h*`:

```text
h* = h_e + 0.5 · (h_w - h_e) + 0.22 · (h_aw - h_e)
```

with reference-state values (density, viscosity, etc.) evaluated at `h*`.
Standard-curve correlations for skin friction and Stanton number then apply
incompressibly at the reference state.

For turbulent flow, the **Spalding-Chi** transformation maps the compressible
result to the incompressible flat-plate correlation. For laminar flow the
**Van Driest II** transformation is the conventional choice.

### 1-D thermal-conduction toy

For surface-temperature evolution under prescribed heat flux:

```rust
pub struct OneDThermalToy {
    pub material: ToyMaterial,
    pub thickness: Length,
    pub n_nodes: usize,
    pub backwall: BackwallCondition,
}

pub struct ToyMaterial {
    pub name: &'static str,                 // textbook name only
    pub density: MassDensity,
    pub specific_heat: SpecificHeatCapacity,
    pub thermal_conductivity: ThermalConductivity,
    pub emissivity: f64,
}

pub enum BackwallCondition {
    Adiabatic,
    PrescribedTemperature(ThermodynamicTemperature),
    Convective { h: HeatTransferCoefficient, t_inf: ThermodynamicTemperature },
}

impl OneDThermalToy {
    pub fn step(&mut self, dt: Duration, q_surface: HeatFlux) {
        // explicit FTCS with stability check (Fourier number < 0.5)
    }
}
```

Materials shipped: textbook generic ablator, generic insulator, generic
metal. **No specific real TPS material is shipped** (no PICA, AVCOAT, RCC,
SLA-561V, or any operational thermal-protection-system parameters).

### Surface ablation toy

Real thermal protection systems on Earth-entry vehicles work by
**ablation**: a controlled chemical and thermal removal of the heat
shield material under high heat flux. The receding surface absorbs heat
through the heat of vaporization, the resulting gases inject into the
boundary layer (reducing the convective heat flux via the *blowing
correction*), and the receded geometry redistributes downstream heating.
OpenBMP ships a **generic textbook ablation toy** for academic study.

The toy is parametric and material-class-agnostic: scenarios pick from a
short list of textbook ablator shapes — generic non-charring sublimator,
generic charring ablator with pyrolysis, idealised graphite-class
sublimator. **No specific real fielded TPS material parameters ship.**
Every shipped material has a textbook name (`generic-charring-1`,
`graphite-toy`, etc.) and parameters drawn from open published academic
ranges, not from operational system data.

#### Trait surface

```rust
pub trait AblationModel {
    /// Local surface recession rate (m/s) given current aerothermal context.
    fn recession_rate(
        &self,
        ctx: &AerothermalContext,
        station: BodyStation,
    ) -> RecessionRate;

    /// Convective heat flux with blowing correction applied.
    fn blowing_correction(
        &self,
        q_no_blowing: HeatFlux,
        m_dot_per_area: MassFlux,
        edge_state: &BlEdgeState,
    ) -> HeatFlux;

    /// Current surface state at a station (virgin, pyrolyzing, char, sublimating).
    fn surface_state(
        &self,
        time: SimTime,
        station: BodyStation,
    ) -> SurfaceState;

    fn validation(&self) -> ValidationStatus;
}

pub enum SurfaceState {
    Virgin,
    Pyrolyzing { progress: f64 },                    // 0=virgin, 1=fully char
    Char,
    Sublimating { recession_rate: Velocity },
    SteadyAblating { recession_rate: Velocity },
}
```

Two implementations:

```rust
/// Quasi-steady-state ablation: surface energy balance assumed instantly
/// equilibrated. Useful for back-of-the-envelope studies and early-stage
/// trajectory analysis.
pub struct SteadyStateAblator {
    pub material: ToyAblator,
}

/// Charring ablator with explicit pyrolysis zone. Tracks virgin / char
/// composition through depth, models pyrolysis-gas injection into the BL.
pub struct CharringAblator {
    pub virgin_material: ToyMaterial,
    pub char_material: ToyMaterial,
    pub pyrolysis_temp_range: [ThermodynamicTemperature; 2],
    pub pyrolysis_enthalpy: SpecificEnergy,
    pub gas_injection_factor: f64,                   // dimensionless
    pub n_depth_nodes: usize,                        // for char-virgin tracking
}

pub struct ToyAblator {
    pub name: &'static str,                          // textbook only
    pub density: MassDensity,
    pub specific_heat: SpecificHeatCapacity,
    pub thermal_conductivity: ThermalConductivity,
    pub heat_of_ablation: SpecificEnergy,            // h_v
    pub vaporization_temperature: ThermodynamicTemperature,
    pub effective_surface_emissivity: f64,
    pub validation_status: ValidationStatus,
}
```

#### Surface energy balance

Steady-state ablator at the surface:

```text
q_conv + q_rad,in  -  σ ε T_w⁴  -  k (∂T/∂x)_wall  -  m_dot · (h_v + h_w)  =  0
```

where:
- `q_conv`, `q_rad,in` — incoming convective and radiative heat flux
  (from `HeatTransferModel`).
- `σ ε T_w⁴` — surface re-radiation (gray-body, emissivity `ε`).
- `k (∂T/∂x)_wall` — conduction into the substrate (from
  `OneDThermalToy`).
- `m_dot` — mass loss rate (m³/s × ρ → kg/m²/s).
- `h_v` — heat of ablation (vaporisation / decomposition).
- `h_w` — wall enthalpy of ablation gas at wall temperature.

Solving for `m_dot` once `T_w` is known closes the energy balance.
For the charring ablator, an additional pyrolysis term contributes to
`h_v` and a gas-injection term modifies the boundary-layer edge.

#### Blowing correction

Pyrolysis or sublimation gases injected into the boundary layer reduce
the convective heat flux. The classical engineering correction is:

```text
q_conv,blow = q_conv,no-blow · (1 - λ · B)
B = m_dot / (ρ_e · u_e · C_H,no-blow)
```

where `B` is the blowing parameter and `λ ≈ 0.3 – 0.6` is the
blowing-reduction parameter (regime-dependent; the scenario selects the
correlation: `Lees`, `Spalding-Chi`, or fixed-`λ`).

#### Surface recession integration

Surface recession is integrated alongside the rigid-body state:

```text
ds/dt = m_dot / ρ_surface       (m/s, into the body)
```

Per-station recession `s_i(t)` is tracked and reported in telemetry. The
total mass loss is:

```text
dm_total/dt = -∫_surface m_dot(station) dA
```

Mass loss couples back into the vehicle mass model via an
`AblationMassCoupling` adapter that registers as an additional
`MassModel` derivative term. This is **opt-in per scenario**: by default,
ablation is computed but does **not** modify vehicle mass (a "frozen
geometry" assumption that is conservative for short re-entries). The
coupling is enabled with a scenario flag and is only allowed when the
mass loss is below a documented fraction of vehicle mass (default 5 %)
to keep the rigid-body formulation valid.

#### Telemetry channels for ablation

| Channel | Units | Notes |
|---|---|---|
| `aerothermal.recession_rate[i]` | m/s | per station |
| `aerothermal.recession_total[i]` | m | per station |
| `aerothermal.surface_state[i]` | enum | per station |
| `aerothermal.mass_loss_total` | kg | integrated over vehicle |
| `aerothermal.blowing_parameter[i]` | dimensionless | `B` |
| `aerothermal.q_conv_with_blowing[i]` | W/m² | corrected heat flux |
| `aerothermal.pyrolysis_progress[i]` | [0,1] | charring ablator only |
| `aerothermal.pyrolysis_gas_mdot[i]` | kg/(m²·s) | charring ablator only |

#### Validation cases for ablation

- **Steady-state graphite sublimation at fixed heat flux** — closed-form
  textbook problem; given a known `q_conv`, ambient pressure, and
  graphite-toy properties, the recession rate is analytic. Pass:
  agreement with hand calculation to <0.5 %.
- **Charring ablator pyrolysis-front advance under a step heat flux**
  — verify pyrolysis-front velocity scales with the analytic similarity
  solution for a semi-infinite slab (textbook problem in
  Hirschel / Bertin).
- **Blowing-correction limits** — at `B → 0`, `q_blow → q_no-blow`. At
  large `B`, `q_blow → 0`. Verify smoothness and monotonicity.
- **Mass-coupling consistency** — total mass loss matches integral of
  recession × surface density × area (within rounding).
- **Energy conservation** — incoming heat flux integral equals the sum
  of (re-radiation + conduction + ablation enthalpy + sensible heating)
  to <0.1 %.

#### What ablation does *not* include

Per safety boundaries, the ablation toy does **not** ship:

- Any specific real fielded TPS material parameter set: PICA, AVCOAT,
  RCC, SLA-561V, FRSI, AFRSI, LI-900, MA-25S, or any other operational
  TPS by name.
- Operationally-tuned char/virgin property pairs from any specific
  fielded re-entry vehicle.
- Specific TPS-design optimisation tools or sizing logic for any real
  vehicle programme.
- Plasma-sheath effects coupled to ablation gas chemistry for the
  purpose of analysing radio-blackout exploitation.

The shipped materials are parametric textbook archetypes — not
operational data — and the validation set is restricted to academic
textbook problems.

## Boundary Layer

### Laminar correlations

Flat-plate compressible laminar skin friction (Blasius + reference-enthalpy
correction):

```text
C_f,lam = 0.664 / sqrt(Re_x*)        where Re_x* uses reference-state ρ*, μ*
```

Heat transfer (Reynolds analogy):

```text
St = C_f / (2 · Pr^(2/3))
```

### Turbulent correlations

White's compressible turbulent flat-plate correlation:

```text
C_f,turb = 0.455 / [ ln²(0.06 · Re_x) · (1 + 0.144 · M_e²)^0.65 ]
```

Combined with Spalding-Chi for further compressibility correction.

### Transition models

- **Empirical** — fixed `Re_x_crit` (default 5e5 for low-disturbance flow,
  user-tunable per scenario).
- **e^N method** (deferred to research extension) — track amplification of
  Tollmien-Schlichting waves.
- **Reθ/M_e** — engineering correlation (Bertin, Hirschel) for hypersonic
  transition based on momentum-thickness Reynolds number divided by edge
  Mach number.

The boundary-layer state is a per-station property; the body geometry can be
discretized into stations along its axis (or surface streamlines for general
panel methods).

## Rarefied / Free-Molecular Bridging

### Knudsen number

```text
Kn = λ / L
λ  = (k_B · T) / (sqrt(2) · π · d² · p)            (mean free path)
L  = vehicle characteristic length (often nose radius for stagnation studies,
                                    or body length for forces)
```

The atmosphere model exposes `mean_free_path`; the aero context computes Kn.

### Bridge functions

| Bridge | Form | Notes |
|---|---|---|
| Cheng | `α(Kn) = exp(-π / (2 Kn))` | Smooth, classic |
| Erfc | `α(Kn) = 0.5 · erfc(log10(Kn) / σ)` | Tunable σ |
| Linear smoothstep | linear ramp over `[Kn_lo, Kn_hi]` | Most predictable |

The hybrid aero method blends continuum and free-molecular coefficients:

```text
C_X(Kn) = (1 - α(Kn)) · C_X,continuum + α(Kn) · C_X,FM
```

This is a deterministic blending; `α` only depends on Kn.

## Offline High-Fidelity Reference Packages

OpenBMP is a flight-dynamics simulator, not a CFD, DSMC, radiation-transport,
or thermal-response code. For world-class hypersonic work, however, the
simulator must be able to consume outputs generated by those tools as offline
reference packages.

Accepted package kinds:

| Package kind | Typical source | OpenBMP use |
|---|---|---|
| `continuum_cfd_aero` | DPLR, LAURA, US3D, FUN3D, SU2, in-house academic CFD | aero coefficient deck, pressure/force/moment validation |
| `radiation_reference` | NEQAIR or published radiative-heating tables | `q_rad` validation and uncertainty envelope |
| `rarefied_dsmc_aero` | SPARTA, DS2V/DS3V, or published DSMC cases | free-molecular / transition-regime aero coefficients |
| `thermal_response_reference` | FIAT, TITAN, 3dFIAT, Icarus, CHAR, PATO, or published arc-jet cases | surface-temperature, recession, and heat-load validation |
| `trajectory_reference` | POST2, GMAT, Orekit, RocketPy, published mission reports | trajectory and event-timing validation |
| `thermochemistry_reference` | NASA CEA, Cantera, Mutation++, published tables | equilibrium / frozen composition, transport, and source-term checks |

Each package is loaded through the data-package mechanism in
[data-provenance.md](data-provenance.md). The sidecar must record:

- Solver/tool name, version, license, and retrieval path.
- Governing equations or model family: Euler, Navier-Stokes, RANS, laminar,
  nonequilibrium Navier-Stokes, DSMC, radiation line-by-line, thermal response.
- Chemistry, transport, turbulence, wall-catalysis, accommodation, and material
  assumptions.
- Grid or particle convergence evidence where relevant.
- Boundary conditions, reference geometry, reference area/length, axes, moment
  reference point, and interpolation policy.
- Uncertainty model and validity envelope over Mach, Reynolds, Knudsen,
  altitude, angle of attack/sideslip, heat flux, and temperature.

The loader never treats an external package as more authoritative than its
declared envelope. Queries outside the envelope fail closed unless the scenario
explicitly selects a documented extrapolation policy.

## Trajectory Infrastructure

### Re-entry interface state

Convenience helpers for setting up entry scenarios:

```rust
pub struct EntryInterfaceBuilder {
    pub entry_altitude: Length,
    pub entry_velocity: Velocity,
    pub flight_path_angle_below_horizon_rad: f64,
    pub heading_rad: f64,
    pub initial_position_geodetic: GeodeticPosition,
}

impl EntryInterfaceBuilder {
    pub fn build(&self) -> RigidBodyState { /* ... */ }
}
```

### Allen-Eggers ballistic entry (analytic-toy)

For non-rotating Earth, exponential atmosphere `ρ = ρ_s exp(-β h)`, ballistic
re-entry at constant `C_D · A / m` and constant entry flight-path angle
`γ_e`, the closed-form velocity profile is:

```text
V(h) / V_e = exp[ -ρ_s C_D A / (2 m β |sin γ_e|) · exp(-β h) ]
```

Maximum deceleration occurs at altitude:

```text
h_max_decel = (1/β) · ln[ ρ_s C_D A / (m β |sin γ_e|) ]
```

with peak deceleration:

```text
n_max = V_e² · β · |sin γ_e| / (2 g_0 · e)
```

This is one of the highest-value validation cases in the project: a fully
analytic re-entry with a single matched scenario in OpenBMP.

### Vinh lifting-entry equations

Vinh's 1981 dimensionless lifting-entry equations (six state variables in
spherical Earth coordinates) are the canonical textbook lifting-entry
formulation. OpenBMP ships them as a separate analytic-toy propagator,
selectable per scenario. Validation: reproduce textbook reference solutions
to within a documented tolerance.

### Skip-glide reference profiles

Academic skip-glide reference profiles (alternating lifting-up / lifting-down
trajectories) are shipped as scenario examples and as analytic-toy
validation cases:

- **Boost-glide academic study** — boost to entry velocity, skip-glide
  through atmosphere, terminate at a scenario-defined altitude.
- **Equilibrium glide** — constant-altitude lifting glide at academic
  L/D ratio.
- **Aerocapture textbook problem** — single-pass aerocapture into a target
  apoapsis altitude (toy two-body capture).

All trajectories terminate at scenario-defined altitudes or velocities, not
at real-world locations. Waypoints are inertial points, not target
coordinates. The mission state machine extends with two phases for these
profiles:

```rust
pub enum MissionPhase {
    PreLaunch,
    Ascent,
    Coast,
    Apogee,
    Descent,
    // Hypersonic additions (academic only)
    EntryInterface,        // crossing into the atmosphere on descent
    LiftingEntry,          // controlled lifting-entry phase
    Recovery,
    Safed,
}
```

The phase machine is **driven by simulator-observable state**, never by
target acquisition.

## Validation Suite

Each case is either analytic-toy (closed-form comparison) or
public-benchmark (against published academic results).

| Case | Class | Pass criterion |
|---|---|---|
| **Allen-Eggers ballistic entry** | analytic-toy | <1 % error on peak deceleration; <100 m on peak-decel altitude |
| **Sutton-Graves single point** | analytic-toy | exact match to stated example to 1e-6 relative |
| **Sutton-Graves / Allen-Eggers heating** | analytic-toy + public-benchmark | closed-form heat load matches profile quadrature; Stardust table-20 convective peak/load fall inside documented engineering bands |
| **Modified Newtonian sphere Cp(0)** | analytic-toy | Cp = Cp_max to 1e-6; Cp(π/2) = 0 |
| **Knudsen bridge limits** | analytic-toy | At Kn=0: continuum result; at Kn=1e6: FM result; smooth transition |
| **Vinh lifting-entry textbook** | public-benchmark | Reproduce reference state-history within 0.5 % over 60 s window |
| **Apollo-class blunt-cone re-entry** | public-benchmark | Compare propagated trajectory and stagnation heat-flux history against published Apollo 4 / 6 trajectory data within documented tolerance |
| **Stardust-class re-entry** | public-benchmark | Compare against publicly available Stardust SRC trajectory and heating data |
| **Tangent-cone vs Taylor-Maccoll exact** | analytic-toy | <0.1 % error on cone Cp at γ = 1.4 across cone half-angle range |
| **NRLMSISE-00 vs published table** | reference | Reproduce densities at standard altitudes (100, 200, 400 km) within documented model uncertainty |
| **Equilibrium-air gamma_eff** | reference | Reproduce Tannehill table entries within 1 % |
| **OneDThermalToy stability** | property | Stable for Fo < 0.5; rejects Fo ≥ 0.5 with a clear error |
| **Park-2T shock-layer Mach 15** | public-benchmark | Reproduce textbook (Park 1990, Anderson 2019) `T_v` peak and equilibrium composition within 5 % |
| **Park-2T frozen / equilibrium limits** | analytic-toy | At `Da → 0`: frozen (composition unchanged across shock). At `Da → ∞`: equilibrium (matches `EquilibriumAir` output). Smooth transition. |
| **Damköhler diagnostic** | analytic-toy | Reported `Da` at Mach 25, 80 km matches hand calculation to 10 % |
| **Graphite sublimation steady state** | analytic-toy | Recession rate at fixed `q_conv`, `p_∞`, graphite-toy properties matches closed-form to <0.5 % |
| **Charring pyrolysis-front advance** | analytic-toy | Front velocity under step heat flux matches semi-infinite-slab similarity solution (Hirschel / Bertin textbook) |
| **Blowing-correction limits** | analytic-toy | At `B → 0`: `q_blow → q_no-blow`. At large `B`: `q_blow → 0`. Smooth and monotonic. |
| **Ablation mass-coupling consistency** | property | Total mass loss = integral of recession × surface density × area, to rounding |
| **Ablation energy conservation** | property | Incoming heat flux integral = re-radiation + conduction + ablation enthalpy + sensible heating, within 0.1 % |
| **Adaptive explicit dense-output event** | analytic-toy | DOPRI853 / RKF78 event time on a smooth manufactured trajectory matches closed form within declared tolerance |
| **Implicit source-term stiffness** | analytic-toy | BDF / Rosenbrock-Wanner source step reproduces stiff scalar decay and two-rate chemistry toy without instability |
| **NASA CEA equilibrium oracle** | reference | Equilibrium composition / `gamma_eff` reference tables generated from documented CEA inputs match stored hashes and tolerances |
| **NRLMSIS 2.x compatibility / HWM14 reference table** | reference | Compatibility-profile samples remain deterministic and finite; HWM14 wind samples reproduce public reference values within model tolerance |
| **Earth-GRAM perturbation envelope** | reference | Density mean and statistical variation samples match public Earth-GRAM examples when redistribution is allowed |
| **External package contract** | property | CFD / DSMC / radiation / thermal-response packages reject missing provenance, envelope gaps, hash mismatches, and unknown solver assumptions |
| **Method of manufactured solutions** | code-verification | Hypersonic source-term and coupling modules demonstrate expected convergence order on smooth manufactured states |

Each case lives in `tests/validation/hypersonic/` and ships with the
expected reference data file (and provenance entry).

## Telemetry Channels Added

| Channel | Units | Frame |
|---|---|---|
| `solver.profile` | enum string | — |
| `solver.method` | enum string | — |
| `solver.dt_accepted` | s | — |
| `solver.error_estimate` | scenario-defined | — |
| `solver.substeps_chemistry` | count | — |
| `solver.substeps_material` | count | — |
| `airdata.mach` | dimensionless | — |
| `airdata.dynamic_pressure` | Pa | — |
| `airdata.knudsen` | dimensionless | — |
| `airdata.regime` | enum string | — |
| `airdata.gamma_eff` | dimensionless | — |
| `airdata.composition.{n2,o2,n,o,no,ar}` | mole fraction | — |
| `aerothermal.q_stag_conv` | W/m² | Body (stagnation point) |
| `aerothermal.q_stag_rad` | W/m² | Body (stagnation point) |
| `aerothermal.h_aw` | J/kg | — |
| `aerothermal.h_w` | J/kg | — |
| `aerothermal.recovery_temp` | K | — |
| `aerothermal.surface_temp[i]` | K | per-station |
| `aerothermal.q_surface[i]` | W/m² | per-station |
| `aerothermal.bl_state[i]` | enum | per-station |
| `aero.method_used` | enum string | — |
| `entry.altitude_geodetic` | m | — |
| `entry.flight_path_angle` | rad | — |
| `entry.heat_load_integral` | J/m² | per-station |
| `airdata.t_translational_rotational` | K | Park 2T, when active |
| `airdata.t_vibrational_electronic` | K | Park 2T, when active |
| `airdata.damkohler` | dimensionless | Park 2T, when active |
| `aerothermal.recession_rate[i]` | m/s | per station, ablation only |
| `aerothermal.recession_total[i]` | m | per station, ablation only |
| `aerothermal.surface_state[i]` | enum | per station, ablation only |
| `aerothermal.mass_loss_total` | kg | ablation only |
| `aerothermal.blowing_parameter[i]` | dimensionless | ablation only |
| `aerothermal.q_conv_with_blowing[i]` | W/m² | ablation only |
| `aerothermal.pyrolysis_progress[i]` | [0,1] | charring ablator only |
| `aerothermal.pyrolysis_gas_mdot[i]` | kg/(m²·s) | charring ablator only |
| `reference.package_id[i]` | string | external packages used |
| `reference.package_hash[i]` | string | external package content hash |
| `reference.envelope_margin[i]` | dimensionless | nearest normalized distance to package envelope |

These channels are first-class: declared in the telemetry schema, exported
to Parquet with full unit/frame metadata, and available for golden-test
diffing.

## Workspace Additions

Updated workspace layout:

```text
crates/
  ...existing crates...
  openbmp-aero/                  # extended with hypersonic methods
  openbmp-aerothermal/           # NEW: heat transfer + BL + ablation
    ablation/                    #   ablation module
  openbmp-physics/               # extended with NRLMSISE-00 + real-gas
                                 # + Park 2T nonequilibrium
data/
  atmosphere/
    us_standard_1976.toml
    provenance.md                # includes NRLMSISE-00 coefficient hashes
  wind/
    hwm14/                       # HWM14 quiet/DWM/geodetic data files
    provenance.md                # includes HWM14 source and data hashes
  realgas/
    tannehill_5species.toml      # NEW: equilibrium correlation
    park87_reactions.toml        # NEW: Park 1987 5-species set
    park90_reactions.toml        # NEW: Park 1990 11-species set
    park93_reactions.toml        # NEW: Park 1993 updated rates
    millikan_white_taus.toml     # NEW: vibrational relaxation times
	  materials/
	    toy_ablators.toml            # NEW: generic textbook ablator set
	                                 # (no real fielded TPS materials)
	  external_refs/
	    cfd/
	      package.example.yaml       # NEW: offline CFD deck sidecar example
	    dsmc/
	      package.example.yaml       # NEW: offline DSMC deck sidecar example
	    radiation/
	      package.example.yaml       # NEW: NEQAIR-like reference sidecar
	    thermal_response/
	      package.example.yaml       # NEW: FIAT/PATO/CHAR-like sidecar
	  validation/
	    hypersonic/
      allen_eggers_reference.toml
      apollo_class_reference.toml
      stardust_reference.toml
      park_shock_layer_mach15.toml      # NEW
      graphite_sublimation_textbook.toml # NEW
      charring_pyrolysis_step_flux.toml  # NEW
docs/
  hypersonic-extensions.md       # this document
scenarios/
  hypersonic/
    allen_eggers_ballistic.toml
    apollo_class_capsule.toml
    vinh_lifting_entry.toml
    sphere_modified_newtonian.toml
    sutton_graves_verification.toml
    knudsen_bridging_demo.toml
    park_2t_nonequilibrium_demo.toml    # NEW (6.10)
    ablation_steady_state_demo.toml     # NEW (6.11)
    ablation_charring_demo.toml         # NEW (6.11)
tests/
  validation/
    hypersonic/                  # tests for each validation case
```

## Determinism Notes

Hypersonic scenarios stress the solver more than rocket-class scenarios:
- Peak decelerations of 10–50 g require small steps during max-q.
- Stagnation heat flux peaks scale like `V³`, sharpening the time-accuracy
  requirement.
- Real-gas iteration introduces convergence behavior that must be
  deterministic.
- Stiff finite-rate chemistry and thermal response can invalidate explicit
  trajectory-step assumptions even when the rigid-body trajectory itself is
  smooth.

Specific determinism rules for the hypersonic crates:

1. **Real-gas equilibrium iteration** must use a fixed maximum iteration
   count and fixed convergence tolerance, declared in the scenario or as
   a fixed default. Iteration count is part of the determinism profile.
2. **Hybrid aero dispatch** must report the method used per step in
   telemetry; transitions are deterministic and reproducible.
3. **Knudsen bridge functions** must be smooth and monotonic; their
   parameters are fixed at scenario start.
4. **NRLMSISE-00** uses the static-defaults mode unless the scenario
   explicitly opts into the time/lat/lon/solar inputs (which then enter the
   determinism set).
5. **OneDThermalToy** uses fixed-step explicit FTCS; the scenario must
   declare a step that satisfies `Fo < 0.5` (Fourier number stability).
   Violations fail closed.
6. **Adaptive explicit integrators** (DOPRI5/8, DOPRI853, RKF78) for
   hypersonic scenarios produce `StateStable` (not `BitStable`)
   determinism. Byte-stable golden tests use fixed-step RK4 or a fixed-step
   high-order explicit method at a documented step size.
7. **Implicit source-term solvers** (implicit Euler, Rosenbrock-Wanner, BDF)
   must declare nonlinear tolerance, maximum iteration count, linear-solver
   tolerance, sub-step count, and failure policy in the scenario. Any
   convergence failure is a fail-closed simulation error.
8. **External reference packages** record solver/tool version, input hashes,
   output hashes, validity envelope, and uncertainty model. The kernel records
   package ids and hashes in telemetry so a run can be reproduced.

## Capabilities

The hypersonic extensions comprise the following capability areas. Each area
carries its own acceptance gate (analytic-toy or public-benchmark validation
case passing in CI).

- **Hypersonic solver stack.** Solver profile schema, fixed-step
  high-order explicit RK, adaptive explicit DOPRI853/RKF78 with dense output,
  event localization, implicit source-term sub-steppers, and deterministic
  coupling telemetry. Acceptance: adaptive event analytic-toy and implicit
  stiffness toy cases.
- **High-altitude atmosphere and winds.** NRLMSISE-00 in-house Rust port with
  static-defaults and full-input coefficient paths, the NRLMSIS 2.x
  compatibility profile, plus full HWM14 quiet-time and DWM07 disturbance
  winds. Official 2.x coefficients remain a license-clean follow-on.
  Acceptance: NRLMSISE-00-vs-published-table, NRLMSIS 2.x compatibility
  smoke/property tests, and HWM14 `checkhwm14` reference cases; official 2.x
  reference-table cases only after import approval.
- **Real-gas thermodynamics.** Tannehill 5-species equilibrium air;
  `gamma_eff`, speed of sound. Acceptance: equilibrium-air `gamma_eff` case.
- **Hypersonic aero methods.** Modified Newtonian, tangent-cone,
  hypersonic similarity. Acceptance: Modified-Newtonian-sphere case;
  tangent-cone-vs-Taylor-Maccoll case.
- **Aerothermal stagnation heating.** `openbmp-aerothermal` crate;
  Fay-Riddell + Sutton-Graves implementations. Acceptance: Sutton-Graves
  single-point case plus Allen-Eggers heat-load quadrature and Stardust
  public-table sanity bands.
- **Boundary layer + distributed heating.** Reference enthalpy method,
  laminar/turbulent correlations, transition models. Acceptance: textbook
  flat-plate heat-transfer case.
- **Continuum-to-rarefied bridging.** Knudsen number computation,
  bridge functions, free-molecular aero. Acceptance: Knudsen-bridge-limits
  case.
- **Trajectory infrastructure.** Entry-interface builder, Allen-Eggers
  analytic-toy propagator, Vinh lifting-entry propagator, academic skip-glide
  reference profiles. Mission FSM phase additions. Acceptance: Allen-Eggers
  case; Vinh lifting-entry case.
- **Public-benchmark validation suite.** Apollo-class blunt-cone
  re-entry, Stardust SRC re-entry, Tauber-Sutton radiative heating
  comparison. Acceptance: full hypersonic validation suite passing in CI.
- **1-D thermal-conduction toy.** Surface-temperature evolution under
  prescribed heat flux. Acceptance: stability property test passing.
- **Park two-temperature nonequilibrium thermochemistry.**
  `NonequilibriumAir` trait, Park'87/'90/'93 reaction sets, Millikan-White
  with Park's high-temperature correction for vibrational relaxation,
  sub-stepped implicit-Euler integration with declared sub-step count and
  tolerance, optional Rosenbrock-Wanner variant for stiff regimes.
  Acceptance: Park-2T shock-layer Mach 15 case, Park-2T frozen /
  equilibrium limit cases, Damköhler diagnostic case.
- **Generic ablation toy.** `AblationModel` trait,
  `SteadyStateAblator` and `CharringAblator` implementations with surface
  energy balance, blowing correction, surface recession integration,
  optional mass-loss coupling to `MassModel`. Generic textbook materials
  only — no real fielded TPS materials. Acceptance: graphite sublimation
  steady-state case, charring pyrolysis-front case, blowing-correction
  limits case, mass-coupling consistency, energy conservation.
- **Offline high-fidelity reference packages.** Data-package schema
  extensions for CFD, DSMC, radiation, thermal-response, trajectory, and
  thermochemistry references, including V&V evidence, uncertainty, solver
  assumptions, and envelope checks. Acceptance: external-package contract
  property tests and at least one public benign reference deck.
- **Hypersonic UQ and credibility reporting.** Scenario-level error
  budgets, parameter uncertainty propagation, sensitivity reports, package
  credibility metadata, and NASA-STD-7009B-style evidence summaries without
  making operational suitability claims. Acceptance: UQ report generated for
  the full hypersonic validation suite.

The hypersonic extensions are research-grade material: every capability declares
its validation status (`experimental` → `checked` → `validated-toy` →
`research`) and never claims operational suitability.

The nonequilibrium thermochemistry and ablation models are the most demanding
from determinism and numerical-stability standpoints (stiff finite-rate
chemistry; coupled surface-recession + thermal conduction + boundary-layer
feedback). They are explicitly tagged `research` until validated against
multiple independent textbook references.

## Out-of-Roadmap (Hypersonic)

Permanently out of scope, regardless of demand:

- Real fielded HGV / MaRV / hypersonic-cruise-weapon parameter sets.
- **Operational tunings** of Park two-temperature reaction rates calibrated
  to any specific fielded vehicle. (Generic public-textbook Park'87 / '90 /
  '93 reaction sets are supported.)
- Real fielded TPS material parameters (PICA, AVCOAT, RCC, SLA-561V, FRSI,
  AFRSI, LI-900, MA-25S, etc.). Generic textbook ablators are supported.
- DSMC (Direct Simulation Monte Carlo) as an in-repo solver or live-coupled
  co-simulation engine. OpenBMP can consume DSMC-derived aero / heating
  coefficients through offline reference packages if the user generates them
  externally with public tools and ships provenance.
- CFD as an in-repo solver or live-coupled flow solver. OpenBMP is a
  flight-dynamics simulator, not a CFD code. CFD-derived aero decks,
  pressure maps, or heat-flux histories may be ingested offline with
  provenance and V&V evidence.
- Automatic design optimization for real hypersonic vehicles. UQ and
  sensitivity studies are acceptable for academic scenarios; vehicle-sizing,
  trajectory optimization, TPS sizing, or control-law tuning for a specific
  real operational vehicle is out of scope.
- Non-Earth atmospheres (Mars, Titan, Venus, Jupiter) and interplanetary
  aerocapture / aerobraking. The framework could host them later via a
  planet-aware atmosphere registry, but no planetary atmosphere data
  ships in the repository.
- Plasma sheath modeling for radio-blackout exploitation, RCS reduction
  during re-entry, or any defense-penetration purpose.
- Skip-glide trajectories with terminal evasion logic, defense-penetration
  optimization, or impact-dispersion analysis.
- Decoy modeling, penetration aids, or counter-defense logic at any phase.
- Targeting at any phase of hypersonic flight.
- Specific TPS-design optimisation tools or sizing logic for any real
  vehicle programme.

## References (Public, Unclassified)

Foundational textbooks and reports the hypersonic extensions draw from.
None of these is imported as data; all algorithms are reimplemented in-house.

- Anderson, J. D. *Hypersonic and High-Temperature Gas Dynamics* (3rd ed.,
  AIAA, 2019).
- Vinh, N. X. *Hypersonic and Planetary Entry Flight Mechanics* (Univ.
  Michigan Press, 1980; reissued 2018).
- Hirschel, E. H. *Basics of Aerothermodynamics* (2nd ed., Springer, 2015).
- Bertin, J. J. *Hypersonic Aerothermodynamics* (AIAA Education Series,
  1994).
- Park, C. *Nonequilibrium Hypersonic Aerothermodynamics* (Wiley, 1990).
- Park, C. "Assessment of two-temperature kinetic model for ionizing air,"
  *J. Thermophysics and Heat Transfer*, 3(3), 1989.
- Park, C. "Review of chemical-kinetic problems of future NASA missions, I:
  Earth entries," *J. Thermophysics and Heat Transfer*, 7(3), 1993.
- Millikan, R. C. and White, D. R. "Systematics of vibrational relaxation,"
  *J. Chemical Physics*, 39, 1963.
- Kovalev, V. L. *Heterogeneous Catalytic Reactions in Hypersonic Flow*
  (academic reference for catalytic-wall conditions).
- Lees, L. "Convective Heat Transfer with Mass Addition and Chemical
  Reactions," *Combustion and Propulsion*, 1958. (Blowing-correction
  reference for ablation.)
- Moss, J. N. "Reacting Viscous-Shock-Layer Solutions with Multicomponent
  Diffusion and Mass Injection," NASA TR R-411, 1974. (Public charring
  ablator reference.)
- Kuntz, D. W., Hassan, B., and Potter, D. L. "An Iterative Approach for
  Coupling Fluid/Thermal Predictions of Ablating Hypersonic Vehicles,"
  AIAA Paper, 1999. (Ablation-thermal coupling textbook reference.)
- Fay, J. A. and Riddell, F. R. "Theory of Stagnation Point Heat Transfer in
  Dissociated Air," *J. Aero. Sci.*, 25(2), 1958.
- Sutton, K. and Graves, R. A. *A General Stagnation-Point Convective
  Heating Equation for Arbitrary Gas Mixtures*, NASA TR R-376, 1971.
- Tauber, M. E. and Sutton, K. *Stagnation Point Radiative Heating
  Relations for Earth and Mars Entries*, NASA TM-86767, 1986.
- Allen, H. J. and Eggers, A. J. *A Study of the Motion and Aerodynamic
  Heating of Ballistic Missiles Entering the Earth's Atmosphere at High
  Supersonic Speeds*, NACA Report 1381, 1958.
- *Equations, Tables, and Charts for Compressible Flow*, NACA Report
  1135, 1953. Public, foundational. Reference for the perfect-gas
  oblique-shock, normal-shock, isentropic-flow, and Taylor-Maccoll
  relations underlying the tangent-cone / tangent-wedge / modified-
  Newtonian methods.
- Picone, J. M. et al. "NRLMSISE-00 empirical model of the atmosphere:
  Statistical comparisons and scientific issues," *J. Geophys. Res.*,
  107(A12), 2002.
- Tannehill, J. C. and Mugalev, P. H. *Equilibrium Air Computations*
  (NASA-published correlations).
- McBride, B. J., Zehe, M. J., and Gordon, S. *NASA Glenn Coefficients for
  Calculating Thermodynamic Properties of Individual Species*, NASA
  TP-2002-211556, 2002. Public reference for CEA-style thermodynamic
  validation data.
- Gordon, S. and McBride, B. J. *Computer Program for Calculation of Complex
  Chemical Equilibrium Compositions and Applications*, NASA RP-1311, 1994.
- NASA Glenn Research Center, *Chemical Equilibrium with Applications (CEA)*,
  public software description and documentation:
  https://www.nasa.gov/glenn/research/chemical-equilibrium-with-applications/
- NASA Ames Aerothermodynamics Branch, *Software Tools* (DPLR and NEQAIR
  descriptions):
  https://www.nasa.gov/general/aerothermodynamics-branch-software-tools/
- NASA Ames Aerothermodynamics Branch, *Modeling and Analysis* (DPLR, US3D,
  NEQAIR, entry physics / chemistry):
  https://www.nasa.gov/general/aerothermodynamics-branch-modeling-and-analysis/
- Lawrence Livermore National Laboratory, *SUNDIALS CVODE* (Adams/BDF stiff
  and non-stiff ODE solver reference):
  https://computing.llnl.gov/projects/sundials/cvode
- NASA, *Program to Optimize Simulated Trajectories II (POST2)* overview:
  https://www.nasa.gov/post2/overview/
- NASA NTRS, *Earth Global Reference Atmospheric Model (Earth-GRAM): User
  Guide*, NASA/TM-20210022157:
  https://ntrs.nasa.gov/citations/20210022157
- NASA CCMC, *NRLMSIS 2.1* model page:
  https://ccmc.gsfc.nasa.gov/models/NRLMSIS~2.1/
- NASA CCMC, *HWM14* model page:
  https://ccmc.gsfc.nasa.gov/models/HWM14~2014/
- Sandia National Laboratories, *SPARTA* DSMC software description:
  https://www.sandia.gov/ccr/ccr-software/
- NASA Ames Thermal Protection Materials Branch, *Design and Analysis*
  (FIAT, TITAN, 3dFIAT, Icarus):
  https://www.nasa.gov/general/thermal-protection-materials-branch-design-and-analysis/
- NASA Glenn, *Overview of CFD Verification & Validation*:
  https://www.grc.nasa.gov/WWW/wind/valid/tutorial/overview.html
- NASA Apollo Mission Reports (Apollo 4 entry trajectory data are
  publicly documented in the NASA technical archives).

These references are listed for academic provenance; the OpenBMP
implementation is a clean-room Rust reimplementation of the documented
algorithms. No proprietary or restricted source is consulted.
