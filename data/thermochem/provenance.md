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
  - data/thermochem/n2o4-n2h4-cantera-nasa-gas-schema-1.toml
  - data/thermochem/n2o4-n2h4-cantera-nasa-gas-tolerance-v1.toml
source_class: public-open-source-tool
source_title: "Cantera 3.2.0 gri30.yaml LOX/LCH4 and LOX/LH2, nDodecane_Reitz.yaml LOX/RP-1 surrogate, and nasa_gas.yaml N2O4/N2H4 species-subset HP-equilibrium thermochemistry tolerance tables"
source_authors: "Cantera developers; GRI-Mech contributors; Hu Wang, Youngchul Ra, Ming Jia, and Rolf D. Reitz for the bundled n-dodecane mechanism; NASA thermodynamic polynomial source data as distributed in Cantera nasa_gas.yaml"
source_url: "https://cantera.org/"
license_or_terms: "Cantera BSD-3-Clause distribution; bundled mechanism terms and bibliographic sources documented by the Cantera data files"
retrieved_utc: "2026-06-18"
source_hash_sha256: "not applicable: generated from installed Cantera package mechanisms"
transformation:
  method: "Cantera gas-phase HP equilibrium for CH4/O2 and H2/O2 reactants with gri30.yaml, c12h26/O2 reactants with nDodecane_Reitz.yaml, and N2H4/N2O4 reactants with an ideal-gas species subset from nasa_gas.yaml over four 20-point grids: chamber pressure = [1, 4, 10, 30] MPa, LOX/LCH4 oxidizer-fuel mass ratio = [2.6, 2.8, 3.1, 3.4, 3.6], LOX/LH2 oxidizer-fuel mass ratio = [4.5, 5.0, 5.5, 6.0, 6.5], LOX/RP-1 surrogate oxidizer-fuel mass ratio = [2.2, 2.4, 2.6, 2.8, 3.0], and N2O4/N2H4 oxidizer-fuel mass ratio = [1.0, 1.2, 1.4, 1.6, 1.8]; eight gri30.yaml, four nDodecane_Reitz.yaml, and four nasa_gas.yaml species-subset cases are in-deck log-pressure/mixture-ratio interpolation checks, followed by OpenBMP's documented ideal c* closed form and an independently implemented ideal-nozzle sea-level performance calculation"
  script: "scripts/generate_cantera_thermochem_reference.py"
verification:
  method: "parse the Schema-1 deck and compare lookup values plus LiquidEnginePerformance mass-flow/Isp/thrust against the checked reference table"
  test: "cargo test -p openbmp-runner --test thermochem_reference cantera_reference_tables --locked"
  tolerance: "relative and absolute tolerances recorded in [tolerances]"
validation_status: research
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public gas-phase mechanism reference for method validation only; not a flight engine calibration, not a liquid-injection CEA replacement, and not by itself an NTO/MMH closure because Cantera 3.2.0 does not bundle an MMH species in the inspected mechanisms."
```

```yaml
dataset_id: openbmp.thermochem.nasa_cea_rocket_reference.v1
files:
  - data/thermochem/nto-mmh-nasa-cea-rocket-schema-1.toml
  - data/thermochem/nto-mmh-nasa-cea-rocket-tolerance-v1.toml
  - data/thermochem/lox-rp1-nasa-cea-rocket-schema-1.toml
  - data/thermochem/lox-rp1-nasa-cea-rocket-tolerance-v1.toml
source_class: public-open-source-tool
source_title: "NASA CEA 3.2.1 thermo.lib NTO/MMH and LOX/RP-1 rocket-equilibrium thermochemistry tolerance tables"
source_authors: "NASA CEA developers; Gordon and McBride NASA RP-1311 lineage; NASA thermodynamic species database maintainers"
source_url: "https://github.com/nasa/cea"
source_tag: "v3.2.1"
source_commit_sha: "c783286f41a0ed18dd2ff7ac7095c036d9c291fd"
license_or_terms: "NASA CEA Apache-2.0 distribution; bundled thermo.lib species data terms and bibliographic sources documented by the NASA CEA distribution"
retrieved_utc: "2026-06-24"
source_hash_sha256: "not applicable: generated from the official NASA CEA 3.2.1 Python package and thermo.lib database"
transformation:
  method: "NASA CEA rocket-equilibrium solve through the official Python package for CH6N2(L)/N2O4(L) reactants and RP-1/O2(L) reactants over two 20-point grids: chamber pressure = [1, 4, 10, 30] MPa, NTO/MMH oxidizer-fuel mass ratio = [1.6, 2.0, 2.2, 2.5, 2.8], and LOX/RP-1 oxidizer-fuel mass ratio = [2.2, 2.4, 2.6, 2.8, 3.0]; four NTO/MMH and four LOX/RP-1 cases are in-deck log-pressure/mixture-ratio interpolation checks, followed by OpenBMP's documented ideal c* closed form and an independently implemented ideal-nozzle sea-level performance calculation"
  script: "scripts/generate_nasa_cea_thermochem_reference.py"
verification:
  method: "parse the Schema-1 deck and compare lookup values plus LiquidEnginePerformance mass-flow/Isp/thrust against the checked reference table"
  test: "cargo test -p openbmp-runner --test thermochem_reference cantera_reference_tables --locked"
  tolerance: "relative and absolute tolerances recorded in [tolerances]"
validation_status: research
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public ideal-rocket thermochemistry evidence only; not a flight engine calibration, not a measured efficiency claim, not a complete liquid-injection combustion-stability validation, and not a qualification statement for any vehicle."
```
