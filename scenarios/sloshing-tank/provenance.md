# Provenance — `scenarios/sloshing-tank/`

Canonical OpenBMP provenance record for the Phase-3.7.E
exit-criterion scenario shipped under `scenarios/sloshing-tank/`.

## `scenarios/sloshing-tank/sloshing-tank.toml`

```yaml
dataset_id:       openbmp.scenario.sloshing_tank.v1
files:
  - scenarios/sloshing-tank/sloshing-tank.toml
source_class:     synthetic-openbmp
source_title:     >-
  Phase-3.7.E exit-criterion scenario. Rigid-body 1000 kg vehicle
  with one axial liquid engine (50 kN at full throttle, Isp 250 s)
  and a cylindrical tank (radius 0.5 m, height 0.5 m, about 49 kg
  synthetic liquid at 50 % fill) carrying an
  EquivalentPendulum slosh model with a 0.05 rad initial
  perturbation on the body-x slosh axis. The engine ignites at
  altitude 0.001 m (≈ T+0 s with initial velocity 100 m/s ECI +z)
  and burns continuously for the 5 s simulation window. Net axial
  body acceleration after ignition (≈ 40 m/s²) drives the
  Abramson cylindrical-tank antisymmetric fundamental sloshing
  mode at ω_n ≈ 12 rad/s, period ≈ 0.5 s; over 5 s the
  pendulum oscillates ≈ 10 cycles and decays under the
  `damping_ratio_zeta = 0.005` setting.
source_authors:   OpenBMP (Dmitri Arekhta) for the Phase-3.7
                  exit-criterion case
source_id:        Synthetic OpenBMP Phase-3.7 fixture
publication_date: 2026-04-28
methodology_reference: >-
  Phase-3.7 of the OpenBMP Phase-3 plan
  (`docs/phase-3-plan.md § 3.7`); slosh dynamics per Abramson
  1966 NASA SP-106 §7.4 (cylindrical-tank antisymmetric
  fundamental mode); the runner-side TankRack mirrors the
  Phase-3.6 EngineRack pattern.
license:          public-domain (synthetic OpenBMP-authored test fixture)
validation_status: experimental
```

### Provenance notes

- Vehicle dimensions, inertia, engine thrust, and tank size are
  synthetic round-number placeholders chosen to produce a clean
  multi-cycle slosh demonstration over the 5 s simulation. The
  tank fluid mass is sized at about 5 % of the vehicle dry mass so the
  Phase-3.7-known mass overcount (`TankRackMassAdapter` adds
  `tank.mass_kg` on top of the engine cluster's propellant
  accounting) is small in absolute terms.
- Propellant is a synthetic low-density liquid (250 kg/m³), chosen so
  the tank contributes about 49 kg while preserving the 50 % fill
  height used by the slosh-frequency calculation. Phase-3.7 rejects
  fielded propellant data per `docs/safety-boundaries.md`.
- The 0.05 rad initial slosh perturbation is small-angle (≈ 2.9°),
  inside the regime where the linearised Abramson Eq 7-25 is
  valid.

### Determinism

- Slosh integration: semi-implicit (symplectic) Euler with locked
  operand order. Single sub-step per kernel base tick; bit-stable
  for fixed `dt = 1 ms`.
- The runner caches `(accel_body, omega_body)` from each step's
  rigid-body solution and feeds them to the next step's tank
  advance — the documented one-step lag that breaks the
  circular dependency between tank reaction force and kernel
  force evaluation. Step 0 uses zeros.
- All Phase-3 byte-stability gates (Phase-1 analytic-toy, Niskanen
  point + rigid, Phase-3.4 effector elevon, Phase-3.5 effector-
  aero, Phase-3.6 octaweb four-engine, Phase-3.7 sloshing tank)
  remain green.

### Phase-3.7 limitations exercised by this scenario

- Drain decoupled from engine cluster: tank's
  `drain_rate_kg_per_s = 0.0`. The engine consumes its own
  internal propellant accounting (Phase 3.6 mechanism); the
  tank's fluid mass is constant. Phase-3.X future work ties them.
- Mass overcount: `EngineClusterMassAdapter` publishes
  `dry_mass - cluster.consumed_kg`; `TankRackMassAdapter` adds
  `tank.mass_kg ≈ 49.09 kg` on top. With `drain_rate = 0`, the
  overcount is constant (about 4.9 % of dry mass) rather than
  growing.
- The kernel-side rigid mass adapter for engine clusters is still
  `ConstantMassRigid` over assembly dry properties (Phase-3.6
  deferral); the `RigidMotorMassAdapter` family does not
  participate. Slosh inertia delta is published in `TankSnapshot`
  but is not consumed by the rigid mass-properties model in Phase 3.7.

### Validation

The Phase-3.7.E e2e test
(`crates/openbmp-cli/tests/tank_slosh_e2e.rs`) runs this scenario
twice and asserts:

1. Both runs complete without error.
2. Both Parquet outputs are byte-identical (replay determinism with
   the tank-rack hot path engaged).
3. Vehicle altitude at end of run is positive and finite (≥ 500 m;
   ballistic minimum from initial velocity alone is 500 m).

Per-tank slosh-state telemetry channels are deferred — they
require a tank-specific channel set in the runner. Phase-3.X
follow-on.
