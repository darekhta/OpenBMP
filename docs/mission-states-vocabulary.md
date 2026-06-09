# Mission States Vocabulary

This document is the canonical state-name vocabulary for OpenBMP's mission
state machine. It defines the standard names used by examples, tests, and
documentation, but it is descriptive rather than a policy blacklist.

The mission-state machinery identifies states by path. `StateId` values are
the FNV-1a-64 hash of those paths.

## Canonical Mission Hierarchy

```text
mission                                                      (region root, composite)
|
+-- mission.states.pad                                       (composite)
|   +-- mission.states.pad.standby                           (atomic)
|   +-- mission.states.pad.powered                           (atomic)
|   +-- mission.states.pad.armed                             (atomic)
|   +-- mission.states.pad.countdown_hold                    (atomic)
|   +-- mission.states.pad.liftoff                           (atomic, transient)
|
+-- mission.states.in_flight                                 (composite)
|   |
|   +-- mission.states.in_flight.ascent                      (composite)
|   |   +-- mission.states.in_flight.ascent.boost            (composite)
|   |   |   +-- mission.states.in_flight.ascent.boost.first_stage_burn
|   |   |   +-- mission.states.in_flight.ascent.boost.staging
|   |   |   +-- mission.states.in_flight.ascent.boost.second_stage_burn
|   |   |   +-- mission.states.in_flight.ascent.boost.upper_stage_burn
|   |   +-- mission.states.in_flight.ascent.post_boost_coast
|   |
|   +-- mission.states.in_flight.apogee_regime               (composite)
|   |   +-- mission.states.in_flight.apogee_regime.apogee_approach
|   |   +-- mission.states.in_flight.apogee_regime.apogee
|   |   +-- mission.states.in_flight.apogee_regime.apogee_passed
|   |
|   +-- mission.states.in_flight.coast
|   +-- mission.states.in_flight.descent                     (composite)
|       +-- mission.states.in_flight.descent.ballistic_descent
|       +-- mission.states.in_flight.descent.entry_interface
|       +-- mission.states.in_flight.descent.lifting_entry
|       +-- mission.states.in_flight.descent.peak_heating
|       +-- mission.states.in_flight.descent.peak_deceleration
|       +-- mission.states.in_flight.descent.drogue_descent
|       +-- mission.states.in_flight.descent.main_descent
|       +-- mission.states.in_flight.descent.final_descent
|
+-- mission.states.post_flight                               (composite)
    +-- mission.states.post_flight.touchdown
    +-- mission.states.post_flight.recovery
    +-- mission.states.post_flight.post_flight_safing
    +-- mission.states.post_flight.safed
```

## State Definitions

| State | Meaning | Citations |
|---|---|---|
| `pad.standby` | Vehicle on launcher, no power applied. | BPS.space Signal phase vocabulary; Stevens & Lewis 2015. |
| `pad.powered` | Vehicle systems powered. | Stevens & Lewis 2015. |
| `pad.armed` | Vehicle systems armed for the scenario's next event. | Stevens & Lewis 2015. |
| `pad.countdown_hold` | Countdown paused at a scenario-defined hold. | NASA launch-procedure terminology. |
| `pad.liftoff` | Transient state covering ignition through pad clearance. | Niskanen 2009; Stevens & Lewis 2015. |
| `ascent.boost.first_stage_burn` | First-stage engine or motor firing. | Sutton & Biblarz. |
| `ascent.boost.staging` | Transient state during a separation event. | Sutton & Biblarz. |
| `ascent.boost.second_stage_burn` | Second-stage burn. | Sutton & Biblarz. |
| `ascent.boost.upper_stage_burn` | Optional upper-stage or payload-stage burn. | Sutton & Biblarz. |
| `ascent.post_boost_coast` | Coast between final burn and apogee approach. | Niskanen 2009. |
| `apogee_regime.apogee_approach` | Vertical velocity approaching zero from positive side. | Niskanen 2009. |
| `apogee_regime.apogee` | Transient state at the vertical-velocity zero crossing. | Niskanen 2009. |
| `apogee_regime.apogee_passed` | Vertical velocity negative; pre-descent. | Niskanen 2009. |
| `in_flight.coast` | Engines off, ballistic flight. | Niskanen 2009. |
| `descent.ballistic_descent` | Vehicle in descent without recovery deployment. | Stevens & Lewis 2015. |
| `descent.entry_interface` | Threshold crossing into denser atmosphere. | Vinh 1980. |
| `descent.lifting_entry` | Re-entry phase with non-zero lift. | Vinh 1980; Anderson 2019. |
| `descent.peak_heating` | Stagnation heating at maximum; transient. | Anderson 2019. |
| `descent.peak_deceleration` | Maximum dynamic-pressure deceleration; transient. | Vinh 1980. |
| `descent.drogue_descent` | Drogue parachute deployed. | Sounding-rocket textbook usage. |
| `descent.main_descent` | Main parachute deployed. | Sounding-rocket textbook usage. |
| `descent.final_descent` | Final descent before touchdown. | Sounding-rocket textbook usage. |
| `post_flight.touchdown` | Transient state at ground contact. | Sounding-rocket textbook usage. |
| `post_flight.recovery` | Recovery operations. | Sounding-rocket textbook usage. |
| `post_flight.post_flight_safing` | Vehicle systems safed, no further actions. | Sounding-rocket textbook usage. |
| `post_flight.safed` | Terminal state; no transitions out. | Sounding-rocket textbook usage. |

## Health Region

```text
health
+-- health.nominal
+-- health.degraded
+-- health.abort_requested
+-- health.safed_on_fault
```

| State | Meaning |
|---|---|
| `health.nominal` | All FDIR detectors clear; commander default. |
| `health.degraded` | One or more FDIR detectors tripped, no abort yet. |
| `health.abort_requested` | Commander-side abort request latched by the health region. |
| `health.safed_on_fault` | Terminal state; effectors disabled by the simulated health response. |

## Comms Region

```text
comms
+-- comms.linked
+-- comms.degraded
+-- comms.loss_of_signal
+-- comms.safed_on_loss_of_signal
```

| State | Meaning |
|---|---|
| `comms.linked` | Ground link nominal in the scenario model. |
| `comms.degraded` | High-latency or partial-loss link. |
| `comms.loss_of_signal` | Confirmed loss; timer running toward `safed_on_loss_of_signal`. |
| `comms.safed_on_loss_of_signal` | Configured timeout elapsed. |

## Estimator-Regime Region

```text
estimator_regime
+-- estimator_regime.boost_mode
+-- estimator_regime.coast_mode
+-- estimator_regime.descent_mode
```

This region is observed by the commander and can be driven by estimator output.
When the estimator mode is unavailable, the region defaults to `boost_mode`.

## Aerodynamic-Regime Region

```text
aerodynamic_regime
+-- aerodynamic_regime.subsonic
+-- aerodynamic_regime.transonic
+-- aerodynamic_regime.supersonic
+-- aerodynamic_regime.hypersonic
+-- aerodynamic_regime.free_molecular
```

The base platform reserves the region name for re-entry and hypersonic
extensions. Scenarios can declare it through the existing
`[[mission.regions]]` extension point when the consuming model exists.

## Backward-Compatibility Migration

Scenarios written in the older flat phase syntax load through a lifting pass:

1. `[[mission.phases]]` blocks map to `[[mission.states]]` blocks.
2. `enter_phase` scenario actions map to `MissionAction::EnterState(StateId)`.
3. Sim-only actions route to the `openbmp-scenario-script` binding list.
4. The lifting pass is byte-stable for equivalent scenarios.

New scenarios should prefer explicit hierarchical `[[mission.states]]` blocks.

## References

- Niskanen, S. *OpenRocket Technical Documentation*, v13.05, 2013.
- Sutton, G. P. and Biblarz, O. *Rocket Propulsion Elements*, 9th ed., Wiley 2017.
- Stevens, B. L. and Lewis, F. L. *Aircraft Control and Simulation*, 3rd ed., Wiley 2015.
- Vinh, N. X. *Hypersonic and Planetary Entry Flight Mechanics*, University of Michigan Press, 1980.
- Anderson, J. D. *Hypersonic and High-Temperature Gas Dynamics*, 3rd ed., AIAA 2019.
