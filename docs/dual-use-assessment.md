# OpenBMP Dual-Use Assessment

This is OpenBMP's standing dual-use risk assessment: the analysis that explains
*why* a full forward flight-dynamics platform — one that can describe the
trajectory of any ballistic body — is not, and does not become, a weapon
development tool. It is the threat-model companion to the binding
[`safety-boundaries.md`](safety-boundaries.md) accept/reject contract and the
[`profile-vocabulary-and-guardrails.md`](profile-vocabulary-and-guardrails.md)
profile spine.

It is **not** legal advice and **not** an export-control classification. For
the project's export posture and the user's own compliance responsibility, see
[`../EXPORT-CONTROL.md`](../EXPORT-CONTROL.md); for intended and out-of-scope
use, see [`../ACCEPTABLE-USE.md`](../ACCEPTABLE-USE.md).

## 1. The governing distinction: forward, never inverse

A ballistic trajectory is body motion under thrust, gravity, and drag. That is
the platform's entire domain, and it is identical physics to a sounding rocket,
a launch-vehicle payload, or a re-entry capsule — the equations of motion do
not know the body's purpose. Studying that physics is standard, openly-taught
flight mechanics (Vallado, Tewari, Vinh), and OpenBMP supports it deliberately.

What makes a tool a *weapon* tool is not the ability to draw the arc. It is the
ability to answer the **inverse** question:

> *Given a place I want to reach, what launch and steering get me there, and how
> accurately?*

Every capability in OpenBMP answers only the **forward** question — *given this
vehicle and this trajectory, what happens?* The inverse question is targeting,
and the platform does not implement it, name it, or accept its inputs. This is
the load-bearing rule; everything below enforces it.

## 2. The bright line: the targeting / guidance / accuracy triad

Three coupled capabilities separate a trajectory simulator from a
weapon-development tool. OpenBMP rejects all three, categorically:

1. **Targeting** — accepting a desired real-world location / aimpoint as input.
2. **Guidance-to-target** — steering during boost, coast, or terminal phases so
   the body arrives at that location; terminal homing; seeker integration.
3. **Accuracy** — scoring or optimizing miss distance / CEP against that
   location.

For a ballistic delivery system these are among the most proliferation-sensitive
capabilities that exist. They are rejected not by naming convention but by the
**absence of any input type that can express a desired location** and the
**absence of any target-scored miss-distance metric**. A landing footprint is
forward (where does the body come down); an aimpoint is inverse (where do I
want it to come down). The platform builds the first and refuses the second.

## 3. Enforcement tiers (defense in depth)

Non-weapon status is enforced in layers. Tiers 1–3 are **technical controls**
that bind regardless of intent and are the *primary* lock. Tiers 4–5 are
**process and intent** — they raise activation energy, structure review, and
declare posture, but they never substitute for a technical control on a
near-the-line capability.

| Tier | Mechanism | Where | Binds by |
|---|---|---|---|
| 1 — Architectural *(primary)* | Forward-only trait signatures; input types with no target / aimpoint / desired miss-distance field; closed optimizer / dispersion enums; field + variant + enrollment audits; dependency reachability checks | `openbmp-physics/src/profile.rs`, `openbmp-testkit`, `openbmp-fc` | Making the operational objective *unconstructible*, not merely unnamed |
| 2 — Lexical | Parse-time lint `FORBIDDEN_SAFETY_TERMS`, run before deserialization | `openbmp-scenario/src/lint.rs` (`scenario.rs:49`) | Rejecting engagement / targeting vocabulary in keys and short values |
| 3 — Provenance | No real fielded-vehicle data; SHA-256 content pins; inline-data tripwires | [`data-provenance.md`](data-provenance.md), CI | Denying real vehicle / motor / TPS parameter sets |
| 4 — Governance | Forward-not-inverse PR checklist, enforcement-tier declaration, protected required status checks for the safety / dual-use CI jobs | [`../CONTRIBUTING.md`](../CONTRIBUTING.md), PR template, repository branch-protection settings | Catching boundary drift in review and preventing maintainer-only bypasses |
| 5 — Documentation / policy | This assessment, export-control notice, acceptable-use policy, disclaimers | repo root + `docs/` | Declaring intent and user responsibility |

The rule, stated plainly: **a new near-the-line capability lands only when a
Tier-1 constraint binds it.** Documentation explains the constraint; it is not
the constraint.

## 4. Residual dual-use surface

An honest assessment names the parts closest to the line. None of these cross
it today; each is held by a specific Tier-1 control.

| Surface | Why it sits near the line | Binding Tier-1 control |
|---|---|---|
| Propulsive-descent convex guidance (`landing.rs` SOCP, translational MPC) | "Drive a vehicle to a point in space" is the same optimal-control math whether the point is a recovery pad or a ground aimpoint | Terminal reference is a scenario-scripted reference / self-recovery site in the simulator's own inertial or range-relative frame; no geodetic-target input type exists |
| Guidance-grade estimator + controller stack (EKF/MEKF/SR-UKF/IMM, LQR/INDI/L1/MPC) | The same algorithm families fly on guided vehicles; IMM is the canonical ballistic-target tracking filter | Tracks scripted references; estimates the vehicle's *own* state; synthetic sensors measure own-state truth and never an external track |
| Geodetic footprint projection (`profile.rs`) | Emits latitude / longitude | Produced only from a scenario-declared launch origin, as a forward prediction of where an unpowered body lands; offline, off the control loop; never compared to a desired point |
| Mass-optimal staging analysis | "Optimize a rocket" is near the line if the objective is range or a place | `[staging_analysis]` accepts only vehicle-intrinsic ideal ΔV, `Isp`, structural coefficient, and payload mass; no range, target, azimuth, launch site, or impact field exists; output is offline report metadata |
| Trajectory optimizer / differential corrector | A general optimizer becomes unsafe when pointed at a surface aimpoint or scored by miss distance | `TerminalCondition` is a closed enum with orbital / inertial / vehicle-intrinsic variants only; `DispersionSource` excludes launch-direction and aimpoint perturbations; optimizer packages are forbidden from `openbmp-fc`; the `optimizer gate` requires field, variant, enrollment, compile-fail, and reachability audits |
| Mach-dependent drag / base / boattail buildup | Shares exterior-ballistics drag-function heritage with projectiles | Forward geometry-to-coefficient model only; `[aero.buildup]` has no range, target, aimpoint, launch-site, or azimuth fields; provenance admits synthetic/textbook/public educational data and no fielded-projectile drag tables |
| Shared coast / entry / hypersonic physics | Structurally shared with boost-glide and multi-stage ballistic vehicles | Implements the shared public physics; refuses operational specialization (real HGV/MaRV parameters, skip-glide-to-a-target, penetration aids) per [`safety-boundaries.md`](safety-boundaries.md) |
| Booster boostback burn (`phalcon9-orbit-boostback`) | A reusable-booster return maneuver shares the "fly back to a place" intent with a powered return-to-target | **Deceleration-only**: a scripted retrograde Δv that sheds velocity; the booster's landing point is an emergent ballistic consequence, never an input. No fly-back guidance to a landing **site / pad** is implemented — that would be a ground aimpoint (the same math as terminal targeting) and is out of scope by design, consistent with §2 and §7 |

## 5. Per-capability rationale for near-the-line items

Each near-the-line item is admissible only under the constraint named here.

- **J2 / EGM2008 footprint propagation** — a forward range-safety /
  landing-dispersion predictor. The `RangeSafetyFootprint` trait takes a
  propagated state and returns a landing point; **no desired-location field
  exists** in
  `BallisticState` or `FootprintEnvironment`. Offline / post-processing, never
  on the control loop. Fails closed when the apogee leaves the gravity model's
  validity envelope.
- **Monte-Carlo dispersion** — scatter around the *predicted* mean, sampled from
  *declared* input uncertainties (winds, ballistic coefficient, burnout-state
  covariance) and propagated forward. There is **no aimpoint to measure
  against**; `radial_dispersion_p50_m` and `radial_offset_from_nominal_m` are
  output-only statistics relative to the sample mean or nominal forward
  footprint, not a target. Persisted samples carry landing output only, not the
  sampled burnout state, wind vector, or ballistic coefficient on the same row.
  Seeded, deterministic RNG; fails closed when dispersion is requested without a
  declared uncertainty source.
- **Translational MPC** — tracks a scenario-scripted reference trajectory or a
  self / recovery site expressed in the simulator's own frame. The terminal
  constraint **cannot be a geographic target**; the input type makes an aimpoint
  unconstructible. It is reference-tracking, like the existing differential-
  flatness loops, extended to the translational state.
- **Explicit ascent reference** — generates an attitude reference toward an
  **inertial cutoff state** (altitude, speed, flight-path angle); it accepts no
  geographic coordinate (per `profile-vocabulary-and-guardrails.md` rule 3).
- **Mass-optimal staging analysis** — solves only the classical ideal
  rocket-equation mass split for a declared vehicle-intrinsic ΔV. The input
  type contains stages, `Isp`, structural coefficients, optional masses, and
  payload mass; it cannot express a destination, range, azimuth, launch site,
  impact point, or accuracy objective. The runner writes report metadata only.
- **Richer live entry coupling** — wires aerothermal / force-stack effects into
  the live entry loop; the bank reference stays **corridor-driven** (heat-rate /
  load-factor / flight-path-angle limits), never bank-for-range-to-a-site.
- **3-mode boost/coast/descent IMM** — an own-state estimation lane that selects
  filter / controller modes by the vehicle's own phase; it has **no external-
  track input**.

## 6. Limits of this assessment

These guardrails are necessary and unusually disciplined, but they are honestly
bounded:

- They **raise activation energy** and keep the *shipped artifact* and the
  *project's intent* unambiguously academic. They do **not** make the underlying
  mathematics non-dual-use — no GNC software can, and neither can a textbook.
- A permissive license (Apache-2.0 / MIT) **cannot legally restrict use**.
  Tiers 4–5 express maintainer intent and community norms; they are not license
  terms and cannot bind a fork.
- Downstream HAL, real bus transport, and any real-data integration are
  **adopter responsibilities** under the adopter's own export-control and
  qualification posture, in the adopter's own repository — not here.

## 7. What would move a fork across the line

Removing the parse-time lint, importing real fielded-vehicle or operational TPS
data, or relaxing the forward-only input constraints to accept a target /
aimpoint, emit a steering command to a location, or compute miss distance / CEP
against an aimpoint — any one of these puts a fork **outside this assessment
and the project's posture**, and on its own export-control and qualification
footing. The project does not accept such changes (see
[`../ACCEPTABLE-USE.md`](../ACCEPTABLE-USE.md) and
[`safety-boundaries.md`](safety-boundaries.md)).

## References

- [`safety-boundaries.md`](safety-boundaries.md) — the binding accept/reject contract.
- [`profile-vocabulary-and-guardrails.md`](profile-vocabulary-and-guardrails.md) — forward-not-inverse for flight profiles.
- [`mission-states-vocabulary.md`](mission-states-vocabulary.md) — the academic state-name canon and rejected vocabulary.
- [`../EXPORT-CONTROL.md`](../EXPORT-CONTROL.md), [`../ACCEPTABLE-USE.md`](../ACCEPTABLE-USE.md) — export posture and use policy.
