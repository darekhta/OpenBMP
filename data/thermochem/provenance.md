# Thermochemistry Data Provenance

```yaml
dataset_id: openbmp.thermochem.lox_lch4_cantera_gri30.v1
files:
  - data/thermochem/lox-lch4-cantera-gri30-schema-1.toml
  - data/thermochem/lox-lch4-cantera-gri30-tolerance-v1.toml
source_class: public-open-source-tool
source_title: "Cantera 3.2.0 gri30.yaml LOX/LCH4 HP-equilibrium thermochemistry tolerance table"
source_authors: "Cantera developers; GRI-Mech contributors"
source_url: "https://cantera.org/"
license_or_terms: "Cantera BSD-3-Clause; GRI-Mech 3.0 mechanism terms documented by the Cantera distribution"
retrieved_utc: "2026-06-11"
source_hash_sha256: "not applicable: generated from installed Cantera package and gri30.yaml mechanism"
transformation:
  method: "Cantera gas-phase HP equilibrium for CH4/O2 reactants at the declared chamber pressure and oxidizer/fuel mass ratio, followed by OpenBMP's documented ideal c* closed form and an independently implemented ideal-nozzle sea-level performance calculation"
  script: "scripts/generate_cantera_thermochem_reference.py"
verification:
  method: "parse the Schema-1 deck and compare lookup values plus LiquidEnginePerformance mass-flow/Isp/thrust against the checked reference table"
  test: "cargo test -p openbmp-runner --test thermochem_reference cantera_gri30_reference_table --locked"
  tolerance: "relative and absolute tolerances recorded in [tolerances]"
validation_status: research
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public gas-phase mechanism reference for method validation only; not a flight engine calibration, not a liquid-injection CEA replacement, and not a claim that all WP-05.3 CEARUN/Cantera propellant-pair evidence is complete."
```
