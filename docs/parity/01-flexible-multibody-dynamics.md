# Flexible & Articulated Multibody Dynamics

**Status:** `experimental` (design intent; this document ships no code).
**Audience:** the engineer or LLM agent who will implement the work packages.
**One-line summary:** replace the single rigid Euler body with a deterministic
spatial-vector (Featherstone) `O(n)` joint tree — free-flyer base, revolute
gimbals, welded separation joints, FFRF flexible bodies on an ingested
Craig-Bampton modal basis, and slosh-as-a-constrained-body — so gimbal
inertia-coupling, multi-mode bending, momentum-conserving staging, and the
coupled plant for `06-gnc-coupled-mimo-and-control.md` fall out of the dynamics
instead of being hand-wired.

> Read `00-overview.md` (the constitution: parity definition, invariants, crate
> map, DAG) and `13-agent-execution-playbook.md` (the doc template and the
> work-package schema) before this document. This is the multibody substrate
> doc; it gates `02-structural-dynamics-loads-slosh-pogo.md` and the coupled
> plant that `06-gnc-coupled-mimo-and-control.md` consumes.

---

## 1. Parity target & ceiling

### 1.1 Target capability (method/architecture parity)

Production launch-vehicle and spacecraft GNC simulators (NASA TREETOPS, JPL
DARTS/DSENDS, ESA-class flexible-multibody tools, and the in-house simulators
at SpaceX/Rocket Lab) do **not** integrate a single rigid Euler body with
hand-coded reaction levers. They integrate a **connected multibody tree**:

- a **free-flyer base** (the vehicle's 6-DOF reference body);
- **revolute joints** for gimballed engines, deployed fins/grid-fins, and
  articulated booms — so the gimbal's reaction on the airframe (the
  *tail-wags-dog* inertia coupling) is a *consequence* of a real joint DOF, not
  a lever heuristic;
- **welded joints** for the stage stack that **release into free joints** at
  separation, making staging momentum-conserving by construction;
- **flexible bodies** carrying a reduced elastic modal basis
  (floating-frame-of-reference + Craig-Bampton component-mode synthesis), so
  body bending is a physically-grounded multi-mode field coupled to the rigid
  rotation, not one hand-tuned SDOF; and
- **slosh masses as constrained bodies** (pendulum / mass-spring joints in the
  tree) whose reaction is momentum-conserving instead of an equivalent-pendulum
  freefall analog with a translational-drift artifact.

The target is **at-parity (method)** per `00-overview.md` §2 row 01: the same
formulations (Featherstone ABA/RNEA/CRBA spatial-vector recursions; FFRF/CMS
flexible bodies; mean-axis/Tisserand reference-frame choice), the same
algorithm classes, and the same V&V methodology (self-consistency,
manufactured solutions, code-to-code, public benchmarks) as the reference
tools — within OpenBMP's deterministic, simulation-only, solver-consuming
posture.

### 1.2 Parity ceiling (the honest boundary)

This dimension is **pure-simulation with no proprietary-data dependency for the
machinery**, so the *algorithms* reach high in-repo fidelity directly
(`00-overview.md` §1.1). What cannot be matched in an open repo is the
**as-flown numbers**, not the method:

1. **The as-flown modal model.** Real Craig-Bampton substructure matrices
   (`Φ`, `K_ff`, modal damping `ζ_j`) come from proprietary FEM tied to
   manufacturing drawings, **correlated against a ground vibration test (GVT) /
   modal survey**. OpenBMP can build and integrate the full FFRF machinery, but
   the modal basis it ingests is textbook/synthetic, never heritage.
2. **Validated slosh parameters.** Slosh mass fractions, frequencies, and
   baffle damping for a *specific* tank/baffle geometry come from proprietary
   scale-model slosh rigs and flight accelerometer data. OpenBMP uses published
   closed-form analog tables (NASA SP-106 / Dodge), which validate the
   *method*, not a fielded tank.
3. **Pogo / CSI margins for a real vehicle** need measured pump cavitation and
   feedline compliance — deferred to `05-propulsion-high-fidelity.md` and
   `02-structural-dynamics-loads-slosh-pogo.md`; this doc supplies only the
   longitudinal-mode substrate.
4. **Certified coupled-loads sign-off** is a contractual artifact, not a model.

**Credible open substitutes** (all in the V&V plan, §5): ABA↔RNEA
self-consistency and method-of-manufactured-solutions for *implementation*
correctness; Kane-Ryan-Banerjee spin-up beam, rotating-cantilever frequency
curves, and NASA SP-106 slosh tables for *method* correctness; code-to-code
against MBDyn / Dymore / CalculiX for *solver* correctness; and the project's
own LOCAL-only externally-cross-validated Falcon-class telemetry as a gross-error
external-validity check on bending/slosh signatures — **never** a tuning
target. **Honest framing, stated in every WP:** OpenBMP reaches
demonstrably-correct-method, benchmark-verified fidelity, but **not**
certified-vehicle-correlated fidelity, and claims no flight readiness.

One **uncertain-convergence** item carries forward from `00-overview.md` §8: the
post-separation upper-stage slosh-coupled orbit closure may not converge. The
multibody/flex substrate ships regardless of whether that *specific* control
co-design closure lands; closure itself is a `06`/`02` co-design WP with
explicitly uncertain outcome, not a config knob.

---

## 2. Current state in source

Verified against the working tree at the time of writing.

### 2.1 The dynamics core is a single rigid Euler body

`crates/openbmp-sim/src/kernel.rs` integrates **one** `RigidBodyState`. The
rigid-body derivative closure (the `derive` closure inside the rigid `step()`,
around `kernel.rs:1547`) builds `RigidBodyDerivative` directly:

- quaternion kinematics `q̇ = 0.5 · q ⊗ [0, ω_body]`;
- the **Euler equation** `ω̇ = I⁻¹ (M − ω×Iω − İ·ω)` (`kernel.rs:1600–1612`),
  with `İ·ω` the inertia-rate term;
- a **kernel-held bending reaction moment**
  `bending_reaction_moment_body_n_m` added to the net torque
  (`kernel.rs:1605`, fed by `set_bending_reaction_moment`, `kernel.rs:1123`).

The derivative type `crates/openbmp-models/src/derivative.rs`
(`RigidBodyDerivative`, line 200) has exactly seven physical fields
(velocity, acceleration, quaternion rate, angular acceleration, mass rate, CG
rate, inertia rate) with locked-order, FMA-free `Add` / `Mul<f64>` /
`l2_norm` / `dimension` impls (`dimension()` returns 26). **There is no
generalized-coordinate vector, no joint, no motion subspace, no articulated
inertia** — the angular dynamics are a single 3×3 inertia inverse, not a tree
recursion.

The integrator (`crates/openbmp-sim/src/integrator.rs`) is generic over
`SimState`: `Rk4FixedStep` plus DOPRI5(4)/DOP853 fixed and adaptive variants,
all with the locked weighted-sum order (`rk4_weighted_sum`, `integrator.rs:63`)
and the FMA-disabled determinism contract. A multibody state must implement the
same `SimState` / `SimStateDerivative` surface to reuse this integrator stack
unchanged.

### 2.2 Separation is mass-property partitioning across independent lanes

`kernel.rs` models staging not as a joint release but as **mass-property
partitioning into separate propagation lanes**:

- `RigidBodySeparation` (`kernel.rs:183`) carries explicit
  `stack_mass_properties` / `stage_mass_properties` and body-frame
  `delta_v` / `delta_omega` / attitude offsets applied at the event.
- `jettison_rigid_body` / `jettison_rigid_bodies` (`kernel.rs:2080`) partition
  the composite state into a continuing primary lane and appended
  `SeparatedRigidBody` lanes (`kernel.rs:213`), each integrated independently
  thereafter.
- `seed_rigid_body_lanes` (`kernel.rs:1949`) seeds multiple independent lanes
  from the start.

Momentum continuity across the split is **not enforced by construction** — the
delta-V/delta-ω tip-offs are scenario-supplied and the post-split mass
properties are scenario-supplied; nothing guarantees total linear+angular
momentum is conserved across the event. This is the seam the welded→free joint
release (Tier 2) replaces.

### 2.3 Structural bending is a single decoupled SDOF with a lever reaction

`crates/openbmp-vehicle/src/structural.rs` (`BendingMode`) is **one** lateral
bending mode per transverse axis:

```text
q̈ + 2ζ_b ω_b q̇ + ω_b² q = (slope_engine / m_q) · a_lateral
```

(`structural.rs:14–22`, integrated by a single symplectic-Euler sub-step,
`structural.rs:147`). The gyro pickup is `slope_gyro · q̇` (`structural.rs:187`)
and the reaction is the **lever-scaled first-increment**
`reaction = −m_q · slope_engine · q̈` (`structural.rs:198`). The doc comment is
explicit: *"First-increment: a lever-scaled reaction; the full distributed-load
coupling is a follow-up."* There is **no rigid-flex inertia coupling** (`m_Rf`,
`m_θf` blocks), **no second mode, no longitudinal mode, no mode-shape field, no
load recovery.** `docs/launch-vehicle-fidelity-frontier.md` §1 marks the single
mode IMPLEMENTED and lists multi-mode / distributed shape as remaining.

### 2.4 Slosh is an equivalent-pendulum with a freefall artifact

`crates/openbmp-vehicle/src/tank/` carries the `MovingMassModel` trait
(`tank/mod.rs:458`: `step(specific_force, omega, dt)`, `mass_contribution()`,
`reaction_body()`, `drain()`, `fluid_remaining_kg()`) and four implementations:
`RigidLiquid`, `EquivalentPendulum`, `EquivalentSpringMass`, `BaffledPendulum`.
The slosh reaction is computed inside the tank model with a **one-step lag** on
`(specific_force_body, omega_body)` (a documented circular-dependency
workaround, `tank/mod.rs`), and `EquivalentPendulum` treats effectively all
fluid as swinging, with an optional capillary freefall restoring term
(`equivalent_pendulum.rs:196`, `with_capillary_freefall_restoring`). A prior
slosh/RCS closure audit identified that the residual post-RCS-arrest
translational drift is a **freefall MODEL artifact** of the equivalent-pendulum
analog, not a GNC defect — exactly the artifact that slosh-as-body
(Tier 2) removes by construction.

### 2.5 Supporting types already present

`crates/openbmp-state/src/`: `RigidBodyState` (`rigid_body.rs`), `MassProperties`
(`mass_properties.rs`: mass + body-frame CG + 3×3 inertia, with
positive-definite/symmetry/triangle-inequality validation), `PointMassState`.
`crates/openbmp-core/src/ids.rs`: `BodyId`, `TankId`, `EngineId`, `RecoveryId`,
`ModelId` (all `u64` newtypes). The runner-side `StructuralRack`
(`crates/openbmp-runner/src/structural.rs`) and the per-body force/moment/mass
adapters (`crates/openbmp-vehicle/src/adapters.rs`) are the existing rack
pattern a multibody adapter mirrors. `crates/openbmp-multibody` now provides
the first WP-01.1 spatial-vector substrate: `SpatialMotion`, `SpatialForce`,
`SpatialInertia`, `PluckerTransform`, the `Joint` vocabulary, and a validated
`MultibodyTree`/`MultibodyState` topology surface. It also has the first
fixed-transform CRBA/RNEA self-consistency substrate (`JointSpaceInertia`,
`joint_space_inertia_crba_fixed_transforms()`, and
`inverse_dynamics_rnea_fixed_transforms()`) plus the next q-dependent layer:
`Joint::joint_transform()`, `PluckerTransform::then()`,
`body_transform_parent_to_child_at_state()`,
`joint_space_inertia_crba_at_state()`, and
`inverse_dynamics_rnea_at_state_no_bias()`. The RNEA layer now also includes
`inverse_dynamics_rnea_at_state()` with generalized velocity-bias terms, a
root parent acceleration seed for gravity-style forcing, and per-body external
spatial forces. A dense forward-dynamics bridge,
`forward_dynamics_dense_at_state()`, solves `H(q) qdd = tau - C` using the
same CRBA/RNEA paths and serves as an ABA cross-check. The O(n)
articulated-body path now exposes `forward_dynamics_aba_at_state()` with
fixed-order symmetric LDLT solves for each local articulated joint block,
including the free-flyer root. The state integration substrate now includes
`MultibodyDerivative`, `advance_state_by()`, `project_state()`,
`scalar_state_size()`, and `weighted_error_norm()`; this is not yet an
`openbmp-models::SimState` implementation because that trait is currently
`Copy`-bound while `MultibodyState` is variable-size. Simulator adapter wiring,
scenario opt-in, double-pendulum tolerance tables, and external Spatial_v2
oracle fixtures remain open.

---

## 3. Target architecture

### 3.1 New crate: `openbmp-multibody` (L2)

```text
openbmp-multibody   L2   spatial-vector tree (ABA/RNEA/CRBA) + FFRF flex
```

**Placement justification (review before building, per `13` §5).**
`openbmp-multibody` sits at **L2**, alongside `openbmp-vehicle` and
`openbmp-physics`. The initial substrate depends *down* on `openbmp-core` (ids,
time) and `openbmp-state` (`MassProperties`); the later simulator adapter will
consume `openbmp-models` integration traits once the generalized-coordinate
state shape is compatible with that trait surface. It depends on **no L3+
crate**. It does
**not** depend on `openbmp-sim` — instead `openbmp-sim` gains a thin
`MultibodyState: SimState` adapter so the existing kernel/integrator drive the
tree (mirroring how the rigid kernel drives `RigidBodyState`). It carries **no
edge to `openbmp-fc`** and `openbmp-fc` gains no edge to it; the FC consumes
multibody outputs only through the existing FC-bridge / sensor boundary
(`00-overview.md` §3 invariant 3). The slosh-as-body work *consumes* the
`MovingMassModel` analog coefficients from `openbmp-vehicle` but expresses them
as a `Joint` in the tree — `openbmp-vehicle` keeps owning the slosh-coefficient
tables; `openbmp-multibody` owns the dynamics.

### 3.2 Core data structures

Spatial (Plücker) 6-vectors stack angular-over-linear:
`v = [ω; v_lin] ∈ ℝ⁶`. A spatial inertia is a 6×6 symmetric matrix; a Plücker
transform `X` maps spatial vectors between body frames.

```rust
/// Spatial (Plücker) motion vector v = [ω; v_lin], angular-over-linear.
/// Deterministic: all arithmetic is explicit, locked-order, FMA-free.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpatialMotion(pub nalgebra::Vector6<f64>);

/// Spatial force vector f = [n; f_lin] (moment-over-force), dual to motion.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpatialForce(pub nalgebra::Vector6<f64>);

/// 6×6 spatial rigid-body inertia about a body frame
/// (Featherstone RBDA §2.13). Built from MassProperties.
#[derive(Copy, Clone, Debug)]
pub struct SpatialInertia(pub nalgebra::Matrix6<f64>);

/// Plücker coordinate transform Xup: parent→child motion (and Xstar: force).
/// Stored as rotation E (3×3) + translation r (3) to keep the 6×6
/// products structured and the operand order locked.
#[derive(Copy, Clone, Debug)]
pub struct PluckerTransform {
    rot_child_from_parent: nalgebra::Matrix3<f64>,
    translation_parent_m: nalgebra::Vector3<f64>,
}

/// Spatial cross products: crossm = (v ×) on motion, crossf = (v ×*) on force.
impl SpatialMotion {
    pub fn crossm(&self) -> nalgebra::Matrix6<f64>; // v× (Featherstone Tbl 2.x)
    pub fn crossf(&self) -> nalgebra::Matrix6<f64>; // v×* = −(v×)ᵀ block form
}
```

```rust
/// A joint connecting body i to its parent λ(i): the motion subspace S_i
/// (6×n_dof_i columns) maps joint velocity q̇_i to a spatial motion.
#[derive(Clone, Debug)]
pub enum Joint {
    /// 6-DOF free-flyer base. q = [unit quaternion (4); position (3)],
    /// q̇ = [ω_body (3); v_body (3)]; S = I₆. Exactly one per tree, at the root.
    FreeFlyer,
    /// 1-DOF revolute (gimbal/hinge): S = [axis; 0]. Engine gimbal, fin hinge.
    Revolute { axis_body: nalgebra::Vector3<f64> },
    /// 1-DOF prismatic: S = [0; axis]. Linear slide (e.g. a mass-spring slosh).
    Prismatic { axis_body: nalgebra::Vector3<f64> },
    /// Welded / fixed: n_dof = 0, S empty. The stage stack interstage.
    /// Carries a `released: bool` flag; release converts it to FreeFlyer-6DOF
    /// at a separation event (Tier 2), momentum-conserving by construction.
    Welded { released: bool },
    /// Spherical (3-DOF) — reserved for ball joints / payload adapters.
    Spherical,
}

impl Joint {
    pub fn n_dof(&self) -> usize;
    /// Motion subspace S_i (6 × n_dof). Locked column order.
    pub fn motion_subspace(&self, q: &[f64]) -> nalgebra::Matrix6xX<f64>;
    /// Joint transform Xj(q): the configuration-dependent child-from-parent
    /// Plücker transform contributed by this joint's coordinates.
    pub fn joint_transform(&self, q: &[f64]) -> PluckerTransform;
    /// Bias velocity term S̊_i q̇ (zero for constant-S joints).
    pub fn bias_velocity(&self, q: &[f64], qd: &[f64]) -> SpatialMotion;
}
```

```rust
/// One body in the tree. Bodies are stored in a flat Vec in a fixed
/// topological order (parent index < child index) so traversal order is
/// deterministic and allocation-free per step.
#[derive(Clone, Debug)]
pub struct TreeBody {
    pub id: openbmp_core::BodyId,
    pub parent: Option<BodyIndex>,    // None ⇒ the free-flyer root
    pub joint: Joint,
    pub inertia: SpatialInertia,      // rigid part (FFRF adds the modal block)
    pub flex: Option<FlexBlock>,      // Some ⇒ this body is an FFRF flexible body
    pub q_offset: usize,              // slice offset into the generalized q
    pub qd_offset: usize,             // slice offset into the generalized q̇
}

/// The kinematic tree. Topology is fixed at build time; coordinates live
/// in the state vector, not here.
#[derive(Clone, Debug)]
pub struct MultibodyTree {
    bodies: Vec<TreeBody>,            // topological order, root first
    n_q: usize, n_qd: usize,         // generalized-coordinate / -velocity sizes
    gravity_eci: nalgebra::Vector3<f64>,
}

/// Generalized-coordinate multibody state: a SimState the existing
/// integrator advances. q holds quaternion + positions + joint angles +
/// modal coords; qd holds the corresponding velocities/rates.
#[derive(Clone, Debug)]
pub struct MultibodyState {
    pub time: openbmp_core::SimTime,
    pub q:  nalgebra::DVector<f64>,  // generalized coordinates
    pub qd: nalgebra::DVector<f64>,  // generalized velocities
}
```

### 3.3 The forward-dynamics trait surface

```rust
/// Forward dynamics over a multibody tree. Given (q, q̇, applied joint
/// forces τ, external spatial forces f_ext), produce the joint accelerations
/// q̈. This is the ABA recursion; RNEA and CRBA are the inverse-dynamics and
/// joint-space-inertia siblings used for V&V and (later) control.
pub trait MultibodyDynamics {
    /// Articulated-Body Algorithm: O(n) forward dynamics (Featherstone Ch.7).
    /// Returns q̈ without assembling or inverting the dense mass matrix.
    /// Deterministic: every 6×6 product uses locked operand order, no FMA.
    fn forward_dynamics_aba(
        &self,
        state: &MultibodyState,
        tau: &[f64],
        f_ext: &ExternalForces,
    ) -> Result<nalgebra::DVector<f64>, MultibodyError>;

    /// Recursive Newton-Euler inverse dynamics (Featherstone Ch.5):
    /// given q̈, return the τ that produces it. The ABA↔RNEA round-trip is
    /// the primary self-consistency V&V case.
    fn inverse_dynamics_rnea(
        &self,
        state: &MultibodyState,
        qdd: &nalgebra::DVector<f64>,
        f_ext: &ExternalForces,
    ) -> Result<nalgebra::DVector<f64>, MultibodyError>;

    /// Composite-Rigid-Body Algorithm (Featherstone Ch.6): the joint-space
    /// inertia matrix H. Used by V&V (H = RNEA columns) and by control
    /// co-design; never on the per-step hot path.
    fn joint_space_inertia_crba(
        &self,
        state: &MultibodyState,
    ) -> Result<nalgebra::DMatrix<f64>, MultibodyError>;
}

/// Applied external forces per body (gravity is seeded into the base accel,
/// not here): aero/thrust/slosh-reaction spatial forces, body-frame.
pub struct ExternalForces {
    per_body: std::collections::BTreeMap<BodyIndex, SpatialForce>,
}
```

The `MultibodyState`/`MultibodyDerivative` pair implements
`openbmp_models::{SimState, SimStateDerivative}` so the existing `Rk4FixedStep`
and DOPRI integrators advance it unchanged. `project()` renormalizes the
free-flyer base quaternion (the only manifold constraint), exactly as
`RigidBodyState::project()` does today.

### 3.4 The math (named formulations & equations)

#### 3.4.1 Spatial algebra (Featherstone RBDA Ch. 2)

Spatial motion `v = [ω; v_O]` about a frame at point `O`; spatial force
`f = [n_O; f]`. Spatial inertia about a body frame from `MassProperties`
(`m`, CG offset `c`, rotational inertia `Ī` about the CG):

```text
I = [ Ī + m c×c×ᵀ   m c× ]
    [ m c×ᵀ         m 1₃ ]        (6×6, symmetric)   — RBDA Eq. 2.62/2.63
```

Plücker transform for a frame change (rotation `E`, translation `r`):

```text
Xup = [ E        0 ]     Xstar = [ E         0 ]
      [ −E r×    E ]             [ −E r×ᵀ?   E ]   — RBDA Eq. 2.24/2.28
```

Spatial cross products `v× ` (on motion) and `v×*` (on force, `= −(v×)ᵀ` in the
appropriate block form), RBDA Eq. 2.31/2.32. **Determinism:** the 6×6 products
are expanded into explicit locked-operand-order scalar sums (no `mul_add`,
no compiler re-association), the same discipline `rk4_weighted_sum` and the
derivative `l2_norm` use today.

#### 3.4.2 Articulated-Body Algorithm — O(n) forward dynamics (RBDA Ch. 7, Table 7.1)

Three passes over the tree (body `i`, parent `λ(i)`, subspace `S_i`):

**Pass 1 — outward (base→tips):** velocities and bias.
```text
v_i  = Xup_i · v_{λ(i)} + S_i q̇_i
c_i  = (v_i ×) S_i q̇_i + S̊_i q̇_i
p_iᴬ = (v_i ×*) I_i v_i − Xstar_i · f_i^ext
```

**Pass 2 — inward (tips→base):** articulated inertia `I_iᴬ` and bias `p_iᴬ`.
```text
U_i = I_iᴬ S_i ;   D_i = S_iᵀ U_i ;   u_i = τ_i − S_iᵀ p_iᴬ
I_{λ}ᴬ += Xupᵀ (I_iᴬ − U_i D_i⁻¹ U_iᵀ) Xup
p_{λ}ᴬ += Xupᵀ (p_iᴬ + I_iᴬ c_i + U_i D_i⁻¹ u_i)
```

**Pass 3 — outward (base→tips):** accelerations.
```text
a_base = −gravity_eci  (seeded)
a_i  = Xup_i · a_{λ(i)} + c_i
q̈_i  = D_i⁻¹ (u_i − U_iᵀ a_i)
a_i += S_i q̈_i
```

`D_i⁻¹` is a **1×1 reciprocal** for revolute/prismatic, a **6×6 LDLᵀ** for the
free-flyer base — a deterministic dense factorization with locked pivot order.
This is the operation that replaces the single `inertia.try_inverse()` at
`kernel.rs:1606`; the gimbal's reaction on the parent now lives in the
`Xupᵀ (…) Xup` inward accumulation — that *is* the tail-wags-dog coupling,
emergent rather than hand-coded.

#### 3.4.3 RNEA & CRBA (RBDA Ch. 5, 6)

RNEA: outward velocity/acceleration sweep, then inward force sweep, returning
`τ = Σ Sᵀ f`. CRBA: build the joint-space inertia `H` by composite-inertia
back-accumulation. Both are exact siblings of ABA and exist primarily as V&V
oracles (§5) and, later, as the plant Jacobians for `06`/`07`. The Spatial
Operator Algebra (Jain/Rodriguez, the DARTS/DSENDS formulation) is the *theory*
that explains why ABA factorizes `M = (I+HΦK)D(I+HΦK)ᵀ` and how rigid and flex
share one operator form; we **implement ABA, not the abstract operator
library** (a launch-vehicle tree is `n ≈ 10–30` bodies, so the asymptotic `O(n)`
payoff is moot — correctness and determinism dominate).

#### 3.4.4 FFRF flexible body + Craig-Bampton modal basis (Shabana; Craig-Bampton 1968)

A flexible body's DOFs are `q = [R (3 transl); θ (orient); q_f (n_modes)]`. A
material point: `r = R + A(θ)(u₀ + Φ q_f)`, `A` = body→inertial DCM, `Φ` =
reduced mode matrix. The Shabana FFRF equation of motion:

```text
M(q) q̈ + Q_v(q, q̇) + K q = Q_e + Q_c
M = [[m_RR, m_Rθ, m_Rf],
     [·,    m_θθ, m_θf],
     [·,    ·,    m_ff]]              — configuration-dependent; rigid-flex
                                        coupling lives in m_Rθ, m_Rf, m_θf
K = blkdiag(0, 0, K_ff)              — only the elastic block is stiff
Q_v                                   — quadratic-velocity (Coriolis/centrifugal
                                        + centrifugal-stiffening correction)
```

The elastic block is reduced **offline** by Craig-Bampton: partition FE DOFs
into boundary `b` and interior `i`; the CB basis is
`T_CB = [[I, 0], [−K_ii⁻¹K_ib, Φ_i]]` (constraint/Guyan modes + fixed-interface
normal modes from `(K_ii − ω²M_ii)φ = 0`); reduced `K̂ = T_CBᵀ K T_CB`,
`M̂ = T_CBᵀ M T_CB`. **OpenBMP ingests `(M̂, K̂, Φ, ω_j, ζ_j)` as a versioned,
provenance-pinned vehicle asset and integrates the reduced ODE at runtime; it
does not build an FE eigensolver** (the solver-consumer boundary,
`13` §6). The **mean-axis / Tisserand frame** (Canavin-Likins) is the
attachment choice that makes `m_Rf`, `m_θf` vanish to first order, decoupling
the rigid attitude the IMU/EKF sees from the elastic field — exactly what the
bending notch and estimator need; it is baked into the ingested `Φ` offline.

#### 3.4.5 Slosh-as-body & welded→free separation

Slosh becomes a **constrained body** in the tree: an equivalent pendulum
(`Revolute` hinge at the tank reference, bob mass `m_s`, fill-dependent length
`L(fill)`) or a mass-spring (`Prismatic`, `m_s`, `k_s`). Its reaction enters
the parent's `p^A` in ABA Pass 2 — **automatically conserving total momentum**,
which removes the equivalent-pendulum freefall translational-drift artifact
(§2.4) by construction. The fill-dependent `(m_s, ω_s, L, ζ_s)`
coefficients are exactly the NASA SP-106 / Dodge tables `openbmp-vehicle`
already curve-fits; the slosh-as-body work *re-homes* them as a `Joint`, it
does not invent new physics. **Separation** is a `Welded { released: false }`
joint flipped to `released: true` (which the tree treats as a `FreeFlyer`
6-DOF) at the event: the departing subtree keeps its inherited velocity, so
linear+angular momentum is continuous across the split — replacing the
independent-lane partitioning of §2.2.

### 3.5 Fidelity tiers

| Tier | Name | What ships | First payoff | New label earned |
|---|---|---|---|---|
| **T0** | Baseline (today) | single rigid Euler body (`kernel.rs`); one decoupled `BendingMode`; equivalent-pendulum slosh (freefall artifact). | — (regression baseline) | n/a |
| **T1** | Spatial-vector rigid **tree** (no flex) | `SpatialInertia`, Plücker transforms, `Joint` enum (free-flyer/revolute/prismatic/welded), `MultibodyTree`, ABA forward dynamics + RNEA inverse-dynamics check. | gimballed engine = revolute joint with **real** tail-wags-dog inertia coupling; stack = welded tree. | `validated-toy` |
| **T2** | Separation-as-joint-release + slosh-as-body | welded→free release at separation (momentum-conserving); slosh pendulum/mass-spring **body** in the tree (removes the known drift artifact). | conserved linear+angular momentum across staging; clean slosh-coupled coast. | `validated-toy` |
| **T3** | FFRF flexible body w/ ingested CB modal basis | one+ stages become FFRF bodies carrying ~5–20 fixed-interface modes (`M̂, K̂, Φ` ingested offline); modal pickup coupled to the gyro/EKF + bending notch. | physically-grounded multi-mode bending replacing the single hand-tuned mode; pogo-capable longitudinal substrate for `02`/`05`. | `research` (benchmark-anchored) |
| **T4** | Coupled load-relief / pogo-substrate loop on EKF state | autopilot load-relief closed through the FFRF modal state behind the FC/sensor boundary; energy/momentum drift bounded over a full ascent. | the coupled plant `06` consumes; `02 T4` / `05 T5` pogo joins here. | `research` |

T1–T2 are Phase A (`00-overview.md` §5: *"01 T1–T2 spatial-vector rigid tree +
separation-as-joint-release"*). T3 is Phase B (needs the ingested modal basis).
T4 is Phase C/D (needs the coupled plant + EKF boundary). Every tier is
independently shippable.

---

## 4. Invariant preservation

Concretely, for this dimension (`00-overview.md` §3):

1. **Byte-determinism.** Every 6×6 spatial product, every ABA pass, and every
   `D_i⁻¹` factorization is expanded into **explicit locked-operand-order**
   scalar arithmetic — **never `f64::mul_add`**, never compiler re-association,
   never unordered iteration. The tree is a flat `Vec` in fixed topological
   order; passes traverse by index, not by `HashMap`. The `D_i⁻¹` LDLᵀ for the
   free base uses a fixed pivot order (no pivot search). This is the same
   contract `rk4_weighted_sum` (`integrator.rs:63`) and the derivative
   `l2_norm` (`derivative.rs:153`) already honor; the determinism CI byte-diff
   gate on the canonical scenario set is the enforcement. Any randomness (none
   needed for the dynamics themselves) derives from
   `openbmp-core::DeterministicRng` with a domain-separated tuple.

2. **Byte-stable-by-default.** The multibody tree is **off by default**, gated
   by a new explicit scenario block (e.g. `[vehicle.multibody]`) exactly like
   `[vehicle.bending]`. A scenario without that block continues to run the
   single rigid Euler kernel and produce **byte-identical** Parquet/CSV. T1
   ships a **rigid-equivalence regression**: a single-free-flyer-body tree with
   no joints integrates to the *same bytes* as the current `RigidBodyState`
   kernel on a no-joint scenario (the §5 MMS-adjacent self-consistency gate).

3. **FC hardware-portability lock.** `openbmp-multibody` is L2 and carries **no
   edge to `openbmp-fc`**; `openbmp-fc` gains **no edge to it**. Multibody
   outputs (rigid attitude on the mean-axis frame, modal pickups) reach the
   flight controller only through the existing FC-bridge / sensor-actuator
   boundary — the EKF sees an estimator-realistic signal, never the simulator's
   internals. The `fc_dependency_tripwire.rs` test stays green unchanged.

4. **Lockstep-clock & no-hot-path-allocation.** The ABA passes preallocate all
   per-body scratch (articulated inertias, bias forces) at tree-build time into
   fixed-size buffers indexed by `BodyIndex`; the per-step path does **not**
   allocate or clone the topology. No `Instant::now`/`SystemTime::now` anywhere
   in the dynamics. The kernel adapter feeds the same one-tick-lagged external
   forces to all RK4 stages (mirroring the tank/engine snapshot discipline,
   `kernel.rs:1064`), so every stage sees a consistent `ExternalForces`.

5. **Four-pillar provenance & validation labels.** The ingested Craig-Bampton
   modal asset (`M̂, K̂, Φ, ω_j, ζ_j`) lands in `data/<vehicle>/modal/` with a
   sibling `provenance.md`, SHA-256 pin, license status, and a validation label
   — it is **synthetic/textbook-derived** (a beam/shell FE deck the agent
   builds, or a published open benchmark), **never** a fielded-vehicle modal
   model. The benchmark constants (Kane-Ryan-Banerjee figures, SP-106 slosh
   coefficients, cantilever-frequency tables) live only in their declared
   source-of-truth tolerance-table TOMLs (allow-listed). No literal WGS84
   `GM_earth` / `J2` / `R_earth` numerals appear anywhere; gravity enters the
   ABA base seed symbolically through the environment crate. Each tier declares
   exactly one label (T1/T2 → `validated-toy`; T3/T4 → `research`) per §5.

---

## 5. V&V plan

The five-layer ladder (`docs/verification.md`): MMS code-verification →
model-verification → code-to-code → public benchmark → UQ. Tolerances are
recorded in per-case tolerance-table TOMLs under `tests/expected/` /
`data/.../tolerances/`.

### 5.1 Verification cases & tolerances

| Case | Type | What it checks | Tolerance | Tier / label |
|---|---|---|---|---|
| **ABA↔RNEA round-trip** | self-consistency (MMS-class) | run ABA forward to get `q̈`, feed `q̈` through RNEA, recover input `τ` on random small trees | `‖τ_recovered − τ‖∞ ≤ 1e-10` (rel) | T1 / `validated-toy` |
| **CRBA = RNEA columns** | self-consistency | `H` from CRBA equals the column-wise RNEA-with-unit-`q̈` reconstruction | `‖H_CRBA − H_RNEA‖∞ ≤ 1e-10` | T1 / `validated-toy` |
| **Rigid-equivalence regression** | byte-identity | single-free-flyer tree (no joints) vs current `RigidBodyState` kernel on a no-joint scenario | **byte-identical** Parquet | T1 / `validated-toy` |
| **Double/triple pendulum energy & momentum** | analytic conservation | total mechanical energy + total linear+angular momentum bounded drift over long horizon | energy drift `≤ 1e-6` rel over 10⁴ s; momentum drift `≤ 1e-9` | T1 / `validated-toy` |
| **Separation momentum continuity** | analytic conservation | total linear+angular momentum continuous across welded→free release | `Δp/p ≤ 1e-12`, `ΔL/L ≤ 1e-12` at the event | T2 / `validated-toy` |
| **Slosh analog vs SP-106/Dodge** | model-verification | pendulum/mass-spring `ω_s`, slosh mass fraction `m_s/m_f`, fixed mass `m_0` vs the monograph tables vs fill | `≤ 2%` on `ω_s`, `≤ 5%` on `m_s/m_f` | T2 / `validated-toy` |
| **Slosh-coupled coast cross-check** | regression | body-based slosh reproduces the recorded RCS-arrested tumble (4.8→<1 rad/s) **without** the translational-drift artifact | tumble-rate match `≤ 5%`; CG-drift artifact eliminated (qualitative + bound) | T2 / `validated-toy` |
| **Rotating cantilever frequency vs `γ`** | public benchmark | non-dimensional bending frequency vs non-dimensional rotation rate (Southwell/Campbell stiffening) | `≤ 3%` vs digitized reference curves | T3 / `research` |
| **Kane-Ryan-Banerjee spin-up beam** | public benchmark | hub spin-up: **consistent** model re-stiffens (geometric stiffening retained); inconsistent linearization diverges | transient tip-deflection match `≤ 5%` vs J. Guidance 10(2) 1987 figures | T3 / `research` |
| **FFRF residual MMS** | manufactured solution | pick analytic `q(t)=[R,θ,q_f]`, substitute into `Mq̈+Q_v+Kq−Q_e`, solve for forcing, verify integrator recovers `q(t)` at design order | order-of-accuracy slope within `±0.3` of nominal | T3 / `research` |
| **Code-to-code beam-stack vs MBDyn/Dymore** | code-to-code | 2–3 segment flexible beam + hinge + tip mass under prescribed base motion; modal tip response & energy | `≤ 5%` on tip response; MAC `≥ 0.9` on mode shapes | T3/T4 / `research` |
| **Full-ascent energy/momentum drift** | regression | bounded drift over a representative powered ascent with flex + slosh + gimbal active | energy/momentum drift bounded & monotone-documented | T4 / `research` |

### 5.2 Labels per tier

- **T1 (rigid tree)** earns `validated-toy`: closed-form self-consistency
  (ABA↔RNEA, CRBA) + analytic conservation + byte-identity regression.
- **T2 (separation + slosh-body)** earns `validated-toy`: analytic momentum
  continuity + published SP-106/Dodge tables.
- **T3 (FFRF)** earns `research`: it is anchored to **public benchmarks**
  (rotating cantilever, Kane-Ryan-Banerjee) and **code-to-code** (MBDyn/Dymore/
  CalculiX), the bar `00-overview.md` §9 sets for `research`.
- **T4** earns `research` for the coupled loop, with the explicit
  uncertain-convergence caveat (§1.2) on the slosh-coupled orbit closure.

No tier ever claims `flight-qualified`/`certified`. Every numeric claim carries
a tolerance-table TOML; UQ (per-entry modal-frequency/damping/slosh-coefficient
margins) flows into Monte Carlo via `openbmp-uq`/`openbmp-mc`
(`11-monte-carlo-uq-and-validation.md`).

## 7. Dependencies on other parity docs

- **Gates `02-structural-dynamics-loads-slosh-pogo.md`.** The FFRF body and its
  ingested CB modal basis (T3) are the substrate `02`'s multi-mode lateral,
  load-recovery (LTM/OTM), CLA transient, and POGO longitudinal-mode work build
  on. `02` consumes the modal block; this doc owns the multibody EOM that
  carries it. (`00-overview.md` §5: *"02 T1–T3 … needs 01 FFRF"*.)
- **Feeds `06-gnc-coupled-mimo-and-control.md`.** T4's coupled plant
  (rigid+flex+slosh+gimbal on the mean-axis frame) is what `06`'s coupled
  LQR/LQG and load-relief design against; the gimbal-as-revolute-joint is the
  actuator the full-`B` allocation drives.
- **Consumes `08-environment-gravity-and-frames.md`.** Gravity is the ABA base
  acceleration seed (`a_base = −gravity_eci`); it arrives symbolically from the
  environment crate (never inline numerals).
- **Couples to `05-propulsion-high-fidelity.md`.** The gimballed-engine
  revolute joint receives thrust/gimbal as the joint actuator; the longitudinal
  modal substrate (T3) is half the POGO loop `05 T5` closes (`02 T4` is the
  other half).
- **Reports UQ through `11-monte-carlo-uq-and-validation.md`** and obeys the
  determinism/real-time contracts of `12-determinism-realtime-and-compute.md`.
- **Follows the gate set and WP schema of `13-agent-execution-playbook.md`.**

---

## 8. Open-source leverage

| Tool | License | Use mode | What we take |
|---|---|---|---|
| **RBDL** (Rigid Body Dynamics Library) | zlib (maximally permissive) | **PORT / READ** | Mirror its ABA/RNEA/CRBA spatial-vector math and pass structure into a pure-Rust deterministic `SpatialInertia`/`Joint`/`Tree` module. **Do NOT FFI-link** — the FMA-disabled locked-operand-order determinism contract requires our own arithmetic. Use its unit tests as cross-checks. |
| **Pinocchio** | BSD-2-Clause (pin a post-2018 release) | **READ** | Second reference for the analytical-derivative RNEA/ABA (useful later for `07` trajectory-optimization gradients of the multibody dynamics). Confirms the recursion; provides RNEA-derivative formulas (Carpentier & Mansard, RSS 2018). |
| **Featherstone Spatial\_v2** (MATLAB) | free for research | **INGEST (oracle)** | Executable ABA oracle: cross-check the Rust ABA on identical small trees to machine precision. |
| **MBDyn** | GPL-2.1 (copyleft) | **COUPLE / code-to-code** | Geometrically-exact beam + CMS elements as the offline V&V oracle for the FFRF body (Tier 3/4 code-to-code). **Run as a separate process; ingest CSVs only. Do NOT link or copy code** into the permissive workspace. |
| **CalculiX** (or Code\_Aster) | GPL-2.0 (external preprocessor) | **INGEST** | Offline FE modal analysis (`*FREQUENCY` + `*SUBSTRUCTURE GENERATE`) to produce the Craig-Bampton substructure (`Φ, M̂, K̂`) shipped as a versioned vehicle asset. **Ingest matrices only — no linking.** |
| **Dymore** benchmark **data** | published in papers (cite, don't redistribute the solver) | **INGEST (reference data)** | Standard rotorcraft/aeroelastic flexible-multibody benchmark suite for rotating-beam and four-bar flexible cases. |
| **sympy.physics.mechanics** (Kane/Lagrange) + PyDy | BSD-3-Clause | **PORT-via-codegen (offline)** | Derive the gimbal+slosh+bending coupling EOM symbolically with Kane's method offline, verify against the Rust ABA tree, optionally codegen the closed-form coupling terms. **Keeps the runtime non-symbolic** — never a runtime symbolic Kane engine. |

All ingested external-solver outputs follow the solver-consumer pipeline
(`13` §6): parser + schema → `data/` landing with `provenance.md` + SHA pin +
label → per-entry UQ margins → coupling through the model trait surface →
code-to-code validation case with a documented "what is not validated" note.

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the full `13` §2 gate set.
A WP that proposes a `new_crate` ships the crate skeleton + placement
justification first, reviewed before the implementation lands.

---

**WP-01.1 — Spatial-vector rigid tree (ABA forward dynamics)**
- **implementation_status:** partial. `crates/openbmp-multibody` is registered
  as an L2 crate with no `openbmp-sim`, `openbmp-runner`, or `openbmp-fc`
  dependency. It exposes angular-over-linear `SpatialMotion`,
  moment-over-force `SpatialForce`, force-dual cross-product matrices,
  Pluecker transforms with a power-duality regression, `SpatialInertia` built
  from validated `MassProperties`, free-flyer/revolute/prismatic/welded/
  spherical `Joint` vocabulary, and topologically ordered
  `MultibodyTree`/`MultibodyState` shapes with deterministic q/qd offsets and
  fail-closed state/topology validation. It also exposes
  `JointSpaceInertia`, `joint_space_inertia_crba_fixed_transforms()`, and
  `inverse_dynamics_rnea_fixed_transforms()`; tests prove the single-root CRBA
  block equals the root spatial inertia and that fixed-transform CRBA columns
  match zero-velocity RNEA generalized forces on a small tree. The q-dependent
  layer now also exposes `Joint::joint_transform()`,
  `PluckerTransform::then()`, `body_transform_parent_to_child_at_state()`,
  `joint_space_inertia_crba_at_state()`, and
  `inverse_dynamics_rnea_at_state_no_bias()`; tests prove transform
  composition, quaternion validation/normalization, fail-closed body-index
  handling, and state-dependent CRBA columns against no-bias RNEA. The biased
  RNEA layer now exposes `inverse_dynamics_rnea_at_state()` with generalized
  velocity-bias terms, a root parent acceleration seed, and per-body external
  spatial forces; tests prove zero-velocity equivalence to the no-bias path,
  the single-free-flyer bias force, root-acceleration forcing, external-force
  subtraction, and fail-closed force input validation. The dense forward
  dynamics bridge now exposes `forward_dynamics_dense_at_state()`, backed by a
  finite checked Gaussian-elimination solver over the state-dependent CRBA
  matrix and biased RNEA residual; tests prove biased RNEA round-trip recovery,
  fail-closed generalized-force validation, and singular-system rejection.
  The ABA substrate now exposes `forward_dynamics_aba_at_state()`, performing
  the three-pass articulated-body recursion over q-dependent transforms,
  velocity-bias terms, root parent acceleration, and per-body external spatial
  forces with fixed-order symmetric LDLT local joint solves; tests prove ABA
  matches the dense bridge and biased RNEA round trip on a moving nontrivial
  tree, plus fail-closed generalized-force, external-force, and singular-block
  handling.
  The state-integration substrate now exposes `MultibodyDerivative`,
  `advance_state_by()`, `project_state()`, `scalar_state_size()`, and
  `weighted_error_norm()`; tests prove derivative arithmetic, locked-order
  component advance, free-flyer/spherical quaternion projection, scalar state
  sizing, adaptive error norm ordering, and invalid tolerance rejection.
  Remaining WP-01.1 work: the actual `openbmp-models`/`openbmp-sim` adapter,
  no-joint byte-equivalence against the current `RigidBodyState` kernel,
  simulator/environment force wiring, double-pendulum tolerance table,
  Spatial_v2 oracle fixtures, and scenario exercise.
- **goal:** Stand up `openbmp-multibody` with `SpatialInertia`, Plücker
  transforms, the `Joint` enum (free-flyer/revolute/prismatic/welded), the
  `MultibodyTree` topology, `MultibodyState: SimState`, and ABA forward dynamics
  + RNEA inverse-dynamics, regressing byte-for-byte against the current
  single-body kernel on the no-joint case. This is the substrate the whole
  flex/coupled-control program needs.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** `openbmp-multibody` (L2) — first commit is the skeleton +
  placement justification per §3.1 (depends only on `openbmp-core`/`-state`/
  `-models`; no `openbmp-sim`/`-fc` edge), reviewed before implementation.
- **touched:** new `crates/openbmp-multibody/`; `crates/openbmp-sim/` (a
  `MultibodyState` integrator adapter); `crates/openbmp-models/` (reuse
  `SimState`/`SimStateDerivative`); a new `[vehicle.multibody]` scenario block.
- **approach:** §3.2–3.4.2 — RBDA Ch. 2 spatial algebra, Ch. 7 Table 7.1 ABA,
  Ch. 5 RNEA. Locked-order 6×6 arithmetic; `D_i⁻¹` = 1×1 reciprocal
  (revolute/prismatic) or fixed-pivot 6×6 LDLᵀ (base). Port the math from RBDL,
  do not link it.
- **acceptance:**
  - new tree off by default; **canonical goldens byte-identical**.
  - single-free-flyer-body tree (no joints) integrates **byte-identical** to the
    `RigidBodyState` kernel on a no-joint scenario.
  - ABA↔RNEA round-trip recovers `τ` to `‖·‖∞ ≤ 1e-10` (tolerance table).
  - CRBA `H` matches RNEA-column reconstruction to `≤ 1e-10`.
  - double-pendulum energy drift `≤ 1e-6` rel / 10⁴ s (tolerance table).
  - cross-checked against Spatial\_v2 oracle on ≥ 3 small random trees.
  - exercised **through a scenario** (a gimballed single-engine ascent).
  - all §2 gates green; `requirements.toml` updated.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line; no boundary-condition surface added.
- **est_effort:** 3–4 weeks (~3–5 kLOC careful Rust over nalgebra).
- **parity_ceiling:** validates the *method/implementation*; no flight-correlated
  vehicle, no flex yet.

---

**WP-01.2 — Gimbal as revolute joint (tail-wags-dog) + welded stack**
- **goal:** Replace the hardcoded gimbal reaction lever with a real revolute
  joint DOF so the engine's inertia coupling on the airframe (tail-wags-dog) is
  emergent; assemble the stage stack as a welded tree.
- **fidelity_tier:** T1
- **depends_on:** [WP-01.1]
- **new_crates:** []
- **touched:** `crates/openbmp-multibody/`; the `[vehicle.multibody]` joint
  schema; `crates/openbmp-runner/` (multibody rack adapter mirroring
  `StructuralRack`); the gimbal effector wiring.
- **approach:** revolute `Joint` with body-frame axis; thrust/gimbal applied as
  the joint actuator `τ_i` + external spatial force; the inward `Xupᵀ(…)Xup`
  accumulation (§3.4.2) carries the reaction.
- **acceptance:**
  - off by default; goldens byte-identical.
  - a 2-axis gimbal commanded sinusoidally produces a body-rate reaction
    matching a Kane's-method hand-derivation (sympy offline) to `≤ 1%`
    (tolerance table).
  - the gimbal reaction reduces to the current lever model in the
    small-inertia/short-lever limit (documented bound).
  - through-scenario ascent test; all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line; gimbal is a forward actuator DOF, never an
  aimpoint.
- **est_effort:** 1.5–2 weeks.
- **parity_ceiling:** method-correct coupling; gimbal/engine inertias are
  synthetic.

---

**WP-01.3 — Separation as welded→free joint release (momentum-conserving)**
- **goal:** Model staging by releasing a welded joint into a free joint at the
  event, conserving total linear+angular momentum by construction — replacing
  the independent mass-property-partition lanes (§2.2).
- **fidelity_tier:** T2
- **depends_on:** [WP-01.1, WP-01.2]
- **new_crates:** []
- **touched:** `crates/openbmp-multibody/` (release transition);
  `crates/openbmp-sim/src/kernel.rs` (the separation path that today calls
  `jettison_rigid_body`); the separation scenario schema.
- **approach:** §3.4.5 — flip `Welded { released:false } → released:true` (treated
  as 6-DOF free); the departing subtree inherits its momentum-continuous state.
  Scenario tip-off Δv/Δω, if declared, are added *after* the conserving split as
  an explicit, provenance-seeded perturbation.
- **acceptance:**
  - off by default; goldens byte-identical (legacy jettison path retained until
    a scenario opts into multibody separation).
  - total linear+angular momentum continuous across the release:
    `Δp/p ≤ 1e-12`, `ΔL/L ≤ 1e-12` (tolerance table) for zero-tip-off.
  - `ballistic_state_compile_fail.rs` extended for the new separated-subtree
    construction path; stays green.
  - through-scenario two-stage separation; all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** separated states constructed only from provenance-seeded
  momentum-continuous releases; lock extended.
- **est_effort:** 1.5 weeks.
- **parity_ceiling:** conservation is exact-by-construction; tip-off magnitudes
  are synthetic.

---

**WP-01.4 — Slosh as a constrained body in the tree**
- **goal:** Replace the equivalent-pendulum freefall analog with a slosh
  pendulum/mass-spring **body** (a `Revolute`/`Prismatic` joint), removing the
  known translational-drift artifact by momentum conservation.
- **fidelity_tier:** T2
- **depends_on:** [WP-01.1]
- **new_crates:** []
- **touched:** `crates/openbmp-multibody/` (slosh joint);
  `crates/openbmp-vehicle/src/tank/` (expose the fill-dependent `(m_s, ω_s, L,
  ζ_s)` coefficients to the tree); the tank scenario schema.
- **approach:** §3.4.5 — instantiate the SP-106/Dodge fixed-mass `m_0` + slosh
  mass `m_s` split as a hinge body; reaction enters the parent `p^A`
  automatically. Re-home the existing `openbmp-vehicle` coefficient tables; no
  new slosh physics.
- **acceptance:**
  - off by default; goldens byte-identical.
  - `ω_s`, `m_s/m_f`, `m_0` match SP-106/Dodge tables to `≤ 2%`/`≤ 5%`/`≤ 5%`
    vs fill (tolerance table).
  - reproduces the recorded RCS-arrested coast tumble (4.8→<1 rad/s) within
    `≤ 5%` **and** eliminates the CG translational-drift artifact (bound +
    qualitative note).
  - through-scenario coast test; all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line; forward physics only.
- **est_effort:** 2 weeks.
- **parity_ceiling:** linear-regime analog only (no swirl/rotary/low-g slosh —
  offline VOF calibration deferred to `02`); tank coefficients synthetic.

---

**WP-01.5 — FFRF flexible body w/ ingested Craig-Bampton modal basis**
- **goal:** Make one+ stages FFRF flexible bodies carrying ~5–20 fixed-interface
  modes from an ingested CB modal asset, with rigid-flex inertia coupling —
  replacing the single hand-tuned `BendingMode` with a physically-grounded
  multi-mode field.
- **fidelity_tier:** T3
- **depends_on:** [WP-01.1, WP-01.2]
- **new_crates:** []
- **touched:** `crates/openbmp-multibody/` (FFRF `FlexBlock`, `M(q)`, `Q_v`);
  `crates/openbmp-vehicle/src/structural.rs` (generalize beyond single
  `BendingMode`); a CB-asset parser + schema; `data/<vehicle>/modal/` asset +
  `provenance.md`; the modal pickup → gyro/EKF/bending-notch wiring.
- **approach:** §3.4.4 — Shabana FFRF EOM with the configuration-dependent
  `M(q)` and quadratic-velocity `Q_v` (centrifugal-stiffening-correct);
  mean-axis/Tisserand attachment baked into the ingested `Φ`. Eigensolve done
  **offline** in CalculiX/Code\_Aster; OpenBMP integrates the reduced ODE only
  (solver-consumer boundary).
- **acceptance:**
  - off by default; goldens byte-identical.
  - CB ingest parser + schema; asset in `data/` with `provenance.md` + SHA pin
    + label; **synthetic/published, never fielded**.
  - rotating-cantilever frequency vs `γ` within `≤ 3%` of reference curves
    (tolerance table).
  - Kane-Ryan-Banerjee spin-up beam: consistent model re-stiffens; tip
    deflection within `≤ 5%` of the 1987 figures (tolerance table).
  - FFRF residual MMS recovers a manufactured `q(t)` at design order.
  - per-entry modal UQ margins flow into Monte Carlo.
  - through-scenario flex ascent; all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** far from line; modal state behind the sensor boundary.
- **est_effort:** 4–6 weeks (algebra-heavy `M(q)`/`Q_v`; ingest pipeline).
- **parity_ceiling:** **the modal numbers are not GVT-correlated heritage data**
  — method/benchmark-verified, not certified-vehicle-correlated.

---

**WP-01.6 — Code-to-code vs MBDyn/Dymore + coupled load-relief loop**
- **goal:** Anchor the FFRF body against a mature flexible-multibody solver and
  close the autopilot load-relief through the FFRF modal state behind the
  FC/sensor boundary, with bounded energy/momentum drift over a full ascent.
- **fidelity_tier:** T4
- **depends_on:** [WP-01.5]
- **new_crates:** []
- **touched:** `crates/openbmp-multibody/`; the FC-bridge load-relief wiring
  (consumed, not imported, by `06`); `tests/expected/` code-to-code fixtures.
- **approach:** §5 — beam-stack code-to-code vs MBDyn (offline oracle); full-
  ascent drift regression; load-relief co-designed in `06` against this plant.
- **acceptance:**
  - off by default; goldens byte-identical.
  - beam-stack tip response within `≤ 5%` of MBDyn; mode-shape MAC `≥ 0.9`
    (tolerance table + ingested oracle CSV with provenance).
  - full-ascent energy/momentum drift bounded and documented.
  - through-scenario coupled-plant test; all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** far from line; load-relief objective is vehicle-intrinsic
  (structural load), never a ground aimpoint.
- **est_effort:** 3–4 weeks.
- **parity_ceiling:** method/code-to-code verified; the slosh-coupled orbit
  *closure* (per `00-overview.md` §8) remains **uncertain-convergence**, a
  `06`/`02` co-design item, not guaranteed by this WP.

---

## 10. References

**Spatial-vector multibody (ABA/RNEA/CRBA):**
- R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008 (ABA Ch. 7
  Table 7.1; RNEA Ch. 5; CRBA Ch. 6; spatial algebra Ch. 2, Eqs. 2.24–2.63).
- R. Featherstone & D. Orin, "Dynamics," *Springer Handbook of Robotics*
  (RNEA/ABA pseudocode).
- R. Featherstone, *Spatial\_v2* reference MATLAB implementation,
  http://royfeatherstone.org/spatial/v2/.
- *RBDL* (Rigid Body Dynamics Library), zlib, https://github.com/rbdl/rbdl.
- J. Carpentier & N. Mansard, "Analytical Derivatives of Rigid Body Dynamics
  Algorithms," *RSS*, 2018 (Pinocchio RNEA derivatives).

**Spatial Operator Algebra (theory):**
- A. Jain, *Robot and Multibody Dynamics: Analysis and Algorithms*, Springer,
  2011.
- G. Rodriguez, A. Jain, K. Kreutz-Delgado, "A Spatial Operator Algebra for
  Manipulator Modeling and Control," *Int. J. Robotics Research*, 1991.
- "DSENDS: Multi-mission Flight Dynamics Simulator," AIAA, 2016 (DARTS/DSENDS).

**Flexible multibody (FFRF / CMS / mean-axis):**
- A. A. Shabana, *Dynamics of Multibody Systems*, 4th ed., Cambridge, 2013
  (FFRF, shape-integral mass matrix, quadratic-velocity vector).
- A. A. Shabana, "Flexible Multibody Dynamics: Review of Past and Recent
  Developments," *Multibody Syst. Dyn.*, 1997.
- R. R. Craig & M. C. C. Bampton, "Coupling of Substructures for Dynamic
  Analyses," *AIAA J.* 6(7):1313–1319, 1968.
- Cammarata et al., "On the use of component mode synthesis methods for the
  model reduction of flexible multibody systems within the FFRF," *MSSP* 142,
  2020.
- P. W. Likins, "Finite element appendage equations for hybrid coordinate
  dynamic analysis," *Int. J. Solids Struct.*, 1972.
- J. A. Canavin & P. W. Likins, "Floating reference frames for flexible
  spacecraft," *J. Spacecraft & Rockets* 14(12), 1977 (Tisserand constraint).
- L. Meirovitch & R. D. Quinn, "Equations of motion for maneuvering flexible
  spacecraft," *J. Guidance* 10(5), 1987 (mean-axis EOM).

**Equation generation (offline derivation) & geometric stiffening:**
- T. R. Kane & D. A. Levinson, *Dynamics: Theory and Applications*,
  McGraw-Hill, 1985.
- T. R. Kane, R. R. Ryan & A. K. Banerjee, "Dynamics of a Cantilever Beam
  Attached to a Moving Base," *J. Guidance* 10(2), 1987 (spin-up dynamic
  stiffening — the benchmark).
- A. K. Banerjee & J. M. Dickens, "Dynamics of an arbitrary flexible body in
  large rotation and translation," *J. Guidance* 13(2), 1990.
- C. M. Roithmayr & D. H. Hodges, *Dynamics: Theory and Application of Kane's
  Method*, Cambridge, 2016.

**Slosh:**
- H. N. Abramson (ed.), *The Dynamic Behavior of Liquids in Moving Containers*,
  NASA SP-106, 1966 (mechanical analogies; baffle damping / Miles).
- F. T. Dodge, *The New Dynamic Behavior of Liquids in Moving Containers*, SwRI,
  2000 (modern coefficient tables for cylinder/sphere/ellipsoid).
- R. A. Ibrahim, *Liquid Sloshing Dynamics: Theory and Applications*, Cambridge,
  2005.

**Code-to-code oracles:**
- *MBDyn* (geometrically-exact beam + CMS), GPL-2.1.
- *CalculiX* / *Code\_Aster* (FE modal + substructure generation), GPL-2.0.
- *Dymore* rotating-beam / four-bar flexible-multibody benchmark data.

*Companion parity documents:* `00-overview.md`,
`02-structural-dynamics-loads-slosh-pogo.md`,
`05-propulsion-high-fidelity.md`, `06-gnc-coupled-mimo-and-control.md`,
`08-environment-gravity-and-frames.md`,
`11-monte-carlo-uq-and-validation.md`,
`12-determinism-realtime-and-compute.md`,
`13-agent-execution-playbook.md`. *Upstream anchors:* `docs/verification.md`,
`docs/launch-vehicle-fidelity-frontier.md`, `docs/safety-boundaries.md`.
