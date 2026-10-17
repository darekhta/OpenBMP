# Feed-System Data Provenance

```yaml
dataset_id: openbmp.feed_system.generic_steady_feed_network.v1
files:
  - data/feed_system/generic-steady-feed-network-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic steady feed-network mass-balance tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-17"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form evaluation of the reduced tank-valve-chamber balance and two-valve node/branch mass-balance equations used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/network.rs and crates/openbmp-feedsystem/src/graph.rs regression tests"
verification:
  method: "parse the TOML table and compare expected chamber pressure, node pressures, branch flows, throat flow, and mass residuals against TankValveChamberNetwork and SteadyFeedGraph"
  test: "cargo test -p openbmp-feedsystem steady_feed_network_matches_provenance_tolerance_table --locked && cargo test -p openbmp-feedsystem graph_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic reduced feed-network fixture; no real vehicle, feed system, propellant-system, or operational data."
```

```yaml
dataset_id: openbmp.feed_system.gfssp_water_distribution_network.v1
files:
  - data/feed_system/gfssp-water-distribution-network-v1.toml
source_class: public-nasa-gfssp-manual-worked-example
source_title: "GFSSP Version 6 Example 2 water distribution network"
source_authors:
  - "A. Majumdar / NASA Marshall Space Flight Center"
source_url: "https://www.nasa.gov/wp-content/uploads/2024/04/gfssp-v6-usermanual.pdf?emrc=67b605705809e"
license_or_terms: "NASA public technical manual"
retrieved_utc: "2026-06-24"
source_hash_sha256: "340f4b208a4814d776ef020df7167304dd74c09d641fb006d352b958cad4bc8c"
transformation:
  method: "manual extraction from section 6.2 and appendix I of the GFSSP Version 6 manual: pressure-boundary values, internal-node initial guesses, pipe lengths/diameters/relative roughness, constant density/viscosity, reference pressure table, and reference signed branch mass-flow table; US customary values converted to SI using factors recorded in the TOML fixture"
  script: "manual extraction; OpenBMP evaluates the same Darcy-Weisbach/Colebrook pipe equations in crates/openbmp-feedsystem/src/graph.rs"
verification:
  method: "parse the TOML fixture, solve the nine-node ten-pipe steady graph, and compare node pressures and signed pipe branch flows against the public GFSSP table within the manual-rounding tolerance"
  test: "cargo test -p openbmp-feedsystem graph_matches_gfssp_water_distribution_network --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: research
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public water-distribution worked example from the NASA GFSSP manual; not vehicle, propellant, tank, or engine operational data."
```

```yaml
dataset_id: openbmp.feed_system.gfssp_tank_blowdown_example8.v1
files:
  - data/feed_system/gfssp-tank-blowdown-example8-v1.toml
source_class: public-nasa-gfssp-manual-worked-example
source_title: "GFSSP Version 6 Example 8 simulation of the blowdown of a pressurized tank"
source_authors:
  - "A. Majumdar / NASA Marshall Space Flight Center"
source_url: "https://www.nasa.gov/wp-content/uploads/2024/04/gfssp-v6-usermanual.pdf?emrc=67b605705809e"
license_or_terms: "NASA public technical manual"
retrieved_utc: "2026-06-24"
source_hash_sha256: "340f4b208a4814d776ef020df7167304dd74c09d641fb006d352b958cad4bc8c"
transformation:
  method: "manual extraction from section 6.8 of the GFSSP Version 6 manual: tank volume, initial tank pressure/temperature, atmospheric boundary pressure, orifice diameter, unit flow coefficient, reported initial mass flow, 200 s duration, and the Moody choked-blowdown analytic pressure solution used for the GFSSP comparison; US customary values converted to SI using factors recorded in the TOML fixture"
  script: "manual extraction; OpenBMP evaluates the same choked isentropic tank-blowdown closed form in crates/openbmp-feedsystem/src/blowdown.rs"
verification:
  method: "parse the TOML fixture and compare pressure ratio, pressure, temperature, tank mass, and choked mass flow at 0-200 s samples against IsentropicTankBlowdown"
  test: "cargo test -p openbmp-feedsystem isentropic_tank_blowdown_matches_gfssp_example8_moody_solution --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: research
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public air-tank blowdown worked example from the NASA GFSSP manual; validates the analytic transient evidence primitive only, not a vehicle pressurization or propellant feed model."
```

```yaml
dataset_id: openbmp.feed_system.gfssp_pressurization_example10.v1
files:
  - data/feed_system/gfssp-propellant-tank-pressurization-example10-v1.toml
source_class: public-nasa-gfssp-manual-digitized-worked-example
source_title: "GFSSP Version 6 Example 10 pressurization of a propellant tank"
source_authors:
  - "A. Majumdar / NASA Marshall Space Flight Center"
source_url: "https://www.nasa.gov/wp-content/uploads/2024/04/gfssp-v6-usermanual.pdf?emrc=67b605705809e"
license_or_terms: "NASA public technical manual"
retrieved_utc: "2026-06-24"
source_hash_sha256: "340f4b208a4814d776ef020df7167304dd74c09d641fb006d352b958cad4bc8c"
transformation:
  method: "manual extraction from section 6.10 of the GFSSP Version 6 manual: five-node propellant-tank pressurization setup, run duration/time step, pressure/temperature/volume inputs, qualitative pressure/temperature trends stated around Figures 136 and 137, 300 dpi plot-digitized Figure 136 tank-bottom/ullage pressure-history samples, a startup-ramp plus power-law reduced pressure-history fit, and a lumped finite-volume ideal-gas ullage parameter fit over the digitized samples"
  script: "manual extraction, plot digitization, reduced-model parameter fit, and finite-volume ullage parameter fit; OpenBMP parses the quantitative pressure-history, qualitative trend fixture, reduced pressure-history model, and finite-volume model in crates/openbmp-feedsystem/src/transient.rs"
verification:
  method: "parse the TOML fixture and assert public setup scalars, pressure-trend booleans, temperature-trend magnitudes, digitized pressure-history sample count, unit conversions, post-transient monotonic decay/convergence semantics, pinned start/end pressure values, residual-gate reference closure, reduced calibrated model residual closure, finite-volume residual closure, residual-gate failure on an out-of-tolerance shifted model sample, time-alignment rejection, invalid reduced-model rejection, invalid finite-volume rejection, and explicit physics-model-residual-closure claim flag"
  test: "cargo test -p openbmp-feedsystem gfssp_pressurization_example10_fixture_carries_public_trends --locked"
  tolerance: "absolute digitization uncertainty recorded in [pressure_digitization]; exact booleans/strings plus SI conversion tolerances embedded in the focused regression"
validation_status: research-digitized
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public propellant-tank pressurization worked example from the NASA GFSSP manual; digitized pressure-history evidence plus reduced calibrated pressure-history residual closure and lumped finite-volume ideal-gas ullage solver closure, with no vehicle calibration or flight-data claim."
```

```yaml
dataset_id: openbmp.feed_system.generic_transient_chamber.v1
files:
  - data/feed_system/generic-transient-chamber-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic transient chamber/feed-network tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-17"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "exact constant-input lumped chamber-pressure solution and start-of-step incompressible dual-valve feed-flow equations used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/chamber.rs and crates/openbmp-feedsystem/src/transient.rs regression tests"
verification:
  method: "parse the TOML table and compare expected chamber pressure, pressure rate, inlet/outlet mass flow, mixture ratio, and dual-valve leg flows against TransientChamber and TransientDualValveFeedNetwork"
  test: "cargo test -p openbmp-feedsystem transient_chamber_matches_provenance_tolerance_table --locked && cargo test -p openbmp-feedsystem transient_dual_valve_network_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic reduced transient chamber and dual-valve feed fixture; no real vehicle, feed system, propellant-system, or operational data."
```

```yaml
dataset_id: openbmp.feed_system.generic_feed_controller.v1
files:
  - data/feed_system/generic-feed-controller-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic throttle/mixture controller tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-17"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form evaluation of the pressure/MR PI law, symmetric integral clamps, valve bounds, and slew limiter used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/control.rs regression test"
verification:
  method: "parse the TOML table and compare expected pressure error, mixture-ratio error, integral state, raw valve requests, and bounded valve commands against ThrottleMixtureController"
  test: "cargo test -p openbmp-feedsystem controller_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic reduced controller fixture; no real vehicle, controller tuning, feed system, propellant-system, or operational data."
```

```yaml
dataset_id: openbmp.feed_system.generic_turbopump_map.v1
files:
  - data/feed_system/generic-turbopump-map-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic normalized turbopump affinity/NPSH tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-11"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form evaluation of the normalized pump map, affinity-law speed scaling, and NPSH head balance used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/pump.rs regression test"
verification:
  method: "parse the TOML table and compare each expected operating-point field against Turbopump::solve"
  test: "cargo test -p openbmp-feedsystem turbopump_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerance recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic generic pump map; no real vehicle, pump, propellant-system, or operational data."
```

```yaml
dataset_id: openbmp.feed_system.public_topflo_pump_duty_point.v1
files:
  - data/feed_system/public-topflo-pump-duty-point-v1.toml
source_class: public-manufacturer-catalog
source_title: "TOP-FLO Centrifugal Pump Catalog"
source_authors:
  - "Steel & O'Brien Manufacturing"
source_url: "https://www.steelobrien.com/wp-content/uploads/2026/03/TOP-FLO-Centrifugal-Pump-Catalog.pdf"
license_or_terms: "Public manufacturer catalog; small numeric facts used for validation provenance."
retrieved_utc: "2026-06-27"
source_hash_sha256: "a2f15e4bd33af6c9e6c99d3966b0eacaf31559448f0dc51a9572c6efe997e850"
transformation:
  method: "manual extraction from the catalog pump-curve reading example answer: 75 GPM, 35 ft head, 9 ft NPSHr, and 1.5 hp motor sizing; SI conversion with factors recorded in the TOML fixture; pressure rise computed as rho*g*head for water density 998 kg/m^3; model design efficiency set to hydraulic power divided by the published motor sizing so Turbopump::solve reproduces the duty-point pressure, NPSH, and conservative shaft-power anchor"
  script: "manual arithmetic recorded in data/feed_system/public-topflo-pump-duty-point-v1.toml and verified in crates/openbmp-feedsystem/src/pump.rs"
verification:
  method: "parse the TOML fixture, solve Turbopump at the published duty point, compare pressure rise, discharge pressure, NPSH, hydraulic power, shaft power, and converted published head/power/NPSH values"
  test: "cargo test -p openbmp-feedsystem turbopump_matches_public_topflo_duty_point --locked"
  tolerance: "absolute model and publication-conversion tolerances recorded in [tolerances]"
validation_status: research
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public catalog duty-point calibration for a commercial centrifugal pump; validates design-point anchoring and NPSH/motor-sizing arithmetic only, not a proprietary rocket turbopump map or full curve fit."
```

```yaml
dataset_id: openbmp.feed_system.generic_moc_line.v1
files:
  - data/feed_system/generic-moc-line-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic fixed-grid MOC Joukowsky tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-11"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form evaluation of frictionless MOC characteristics and the Joukowsky valve-closure bound used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/line.rs regression test"
verification:
  method: "parse the TOML table and compare the expected Courant step, downstream head/pressure, pressure delta, and min/max heads against MocLine::step"
  test: "cargo test -p openbmp-feedsystem moc_line_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic single-pipe closure fixture; no real vehicle, feed system, or operational data."
```

```yaml
dataset_id: openbmp.feed_system.public_joukowsky_valve_closure.v1
files:
  - data/feed_system/public-joukowsky-valve-closure-v1.toml
source_class: public-engineering-article
source_title: "Chemical Engineering: Can You Trust the Joukowsky Equation for Waterhammer?"
source_authors:
  - "Trey Walters / Applied Flow Technology"
source_url: "https://www.chemengonline.com/can-you-trust-the-joukowsky-equation-for-waterhammer/"
license_or_terms: "Public web article; small numeric facts used for validation provenance."
retrieved_utc: "2026-06-27"
source_hash_sha256: "not captured: public HTML article, numeric facts recorded in TOML fixture"
transformation:
  method: "manual extraction of the article's Example 1 instantaneous valve closure inputs and rounded reported outputs; OpenBMP recomputes pipe area from D = 0.5 m, flow velocity from Q/A, and the Joukowsky head/pressure rise with rho = 900 kg/m^3 and a = 1291 m/s"
  script: "manual arithmetic recorded in data/feed_system/public-joukowsky-valve-closure-v1.toml and verified in crates/openbmp-feedsystem/src/line.rs"
verification:
  method: "parse the TOML fixture, run one Courant-exact MocLine::step for instantaneous downstream valve closure, compare exact downstream head/pressure/dt against the recorded OpenBMP values, and compare the resulting head/pressure rise against the article's rounded public values"
  test: "cargo test -p openbmp-feedsystem moc_line_matches_public_joukowsky_valve_closure_example --locked"
  tolerance: "absolute exact-model tolerances and publication-rounding tolerances recorded in [tolerances]"
validation_status: research
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public single-pipe water-hammer example; validates the line-transient surge relation only, not a vehicle feed system or transient network topology."
```

```yaml
dataset_id: openbmp.feed_system.generic_pogo_stability.v1
files:
  - data/feed_system/generic-pogo-stability-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic reduced POGO stability tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-11"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form evaluation of the reduced cubic POGO characteristic and Routh-Hurwitz stability margins used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/pogo.rs regression test"
verification:
  method: "parse the TOML table and compare expected coefficients, margins, verdicts, and neutral accumulator compliance against PogoStability::analyze"
  test: "cargo test -p openbmp-feedsystem pogo_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerance recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic reduced feed-half stability fixture; no real vehicle structural mode, feed system, or case-history data."
```

```yaml
dataset_id: openbmp.feed_system.public_pogo_case_history.v1
files:
  - data/feed_system/public-pogo-case-history-v1.toml
source_class: public-case-history
source_title: "NASA public POGO case-history qualitative benchmark record"
source_authors:
  - "S. Rubin / NASA Space Vehicle Design Criteria"
  - "Curtis E. Larsen / NASA Johnson Space Center"
source_url:
  - "https://ntrs.nasa.gov/citations/19710016604"
  - "https://ntrs.nasa.gov/citations/20080018689"
license_or_terms: "NASA NTRS public records; work of the US Government, public use permitted"
retrieved_utc: "2026-06-24"
source_hash_sha256:
  nasa_sp_8055_pdf: "f7db2a45c7ad64434bb94f6dffaaa4e2bbfe4b168f8b6aaba0e5bf7745c6abb7"
  larsen_2008_pdf: "6dc98264ca77e8465cddb1ed7e517cb73fc83e331eca7cd5c95c55683dbcd777"
transformation:
  method: "manual extraction of qualitative public case-history fields: vehicle family, stage, observed frequency/amplitude snippets when explicit, mitigation type/outcome, and accumulator/suppressor stability trend"
  script: "manual extraction from the cited NASA public records; no pump parameter fitting or vehicle-specific margin calibration"
verification:
  method: "parse the TOML fixture and assert public source metadata, Titan/Saturn case ids, observed bands, and accumulator trend semantics"
  test: "cargo test -p openbmp-structdyn public_pogo_case_history_fixture --locked"
  tolerance: "qualitative evidence only; no numeric residual or vehicle-specific margin claimed"
validation_status: research-method-level
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public qualitative Titan/Gemini and Saturn POGO case-history fixture; supports method-level benchmark trend checks only and does not expose flight-design pump parameters."
```
