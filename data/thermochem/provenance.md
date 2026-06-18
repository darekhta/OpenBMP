# Thermochemistry Data Provenance

```yaml
dataset_id: openbmp.thermochem.cantera_reference.v1
files:
  - data/thermochem/lox-lch4-cantera-gri30-schema-1.toml
  - data/thermochem/lox-lch4-cantera-gri30-tolerance-v1.toml
  - data/thermochem/lox-lh2-cantera-gri30-schema-1.toml
  - data/thermochem/lox-lh2-cantera-gri30-tolerance-v1.toml
  - data/thermochem/lox-rp1-ndodecane-cantera-reitz-schema-1.toml
  - data/thermochem/lox-rp1-ndodecane-cantera-reitz-tolerance-v1.toml
source_class: public-open-source-tool
source_title: "Cantera 3.2.0 gri30.yaml LOX/LCH4 and LOX/LH2 plus nDodecane_Reitz.yaml LOX/RP-1 surrogate HP-equilibrium thermochemistry tolerance tables"
source_authors: "Cantera developers; GRI-Mech contributors; Hu Wang, Youngchul Ra, Ming Jia, and Rolf D. Reitz for the bundled n-dodecane mechanism"
source_url: "https://cantera.org/"
license_or_terms: "Cantera BSD-3-Clause distribution; bundled mechanism terms and bibliographic sources documented by the Cantera data files"
retrieved_utc: "2026-06-18"
source_hash_sha256: "not applicable: generated from installed Cantera package mechanisms"
transformation:
  method: "Cantera gas-phase HP equilibrium for CH4/O2 and H2/O2 reactants with gri30.yaml plus c12h26/O2 reactants with nDodecane_Reitz.yaml over three 20-point grids: chamber pressure = [1, 4, 10, 30] MPa, LOX/LCH4 oxidizer-fuel mass ratio = [2.6, 2.8, 3.1, 3.4, 3.6], LOX/LH2 oxidizer-fuel mass ratio = [4.5, 5.0, 5.5, 6.0, 6.5], and LOX/RP-1 surrogate oxidizer-fuel mass ratio = [2.2, 2.4, 2.6, 2.8, 3.0]; eight gri30.yaml and four nDodecane_Reitz.yaml cases are in-deck log-pressure/mixture-ratio interpolation checks, followed by OpenBMP's documented ideal c* closed form and an independently implemented ideal-nozzle sea-level performance calculation"
  script: "scripts/generate_cantera_thermochem_reference.py"
verification:
  method: "parse the Schema-1 deck and compare lookup values plus LiquidEnginePerformance mass-flow/Isp/thrust against the checked reference table"
  test: "cargo test -p openbmp-runner --test thermochem_reference cantera_reference_tables --locked"
  tolerance: "relative and absolute tolerances recorded in [tolerances]"
validation_status: research
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public gas-phase mechanism reference for method validation only; not a flight engine calibration, not a liquid-injection CEA replacement, and not a claim that all WP-05.3 CEARUN/Cantera propellant-pair evidence is complete."
```
