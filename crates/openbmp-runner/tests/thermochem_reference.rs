//! Thermochemistry reference-table regression coverage.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::{Path, PathBuf};

use openbmp_propulsion::{
    LiquidEngineCStarEfficiencyBand, LiquidEngineNozzle, LiquidEnginePerformance,
    LiquidEngineThermochemistry, NozzleSeparationCriterion,
};
use openbmp_thermochem::{ThermochemDeck, ThermochemQuery, ThermochemTable};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct ReferenceTable {
    openbmp: ReferenceSchema,
    meta: ReferenceMeta,
    deck: ReferenceDeck,
    nozzle: ReferenceNozzle,
    tolerances: ReferenceTolerances,
    case: Vec<ReferenceCase>,
}

#[derive(Debug, Deserialize)]
struct ReferenceSchema {
    thermochem_reference: u32,
}

#[derive(Debug, Deserialize)]
struct ReferenceMeta {
    source_title: String,
    mechanism: String,
    species_subset: Option<Vec<String>>,
    reactants: Option<Vec<String>>,
    validation: String,
}

#[derive(Debug, Deserialize)]
struct ReferenceDeck {
    file: PathBuf,
}

#[derive(Debug, Deserialize)]
struct ReferenceNozzle {
    throat_area_m2: f64,
    exit_area_m2: f64,
    ambient_pressure_pa: f64,
    separation: String,
}

#[derive(Debug, Deserialize)]
struct ReferenceTolerances {
    relative: f64,
    absolute_temperature_k: f64,
    absolute_gamma: f64,
    absolute_molecular_weight_kg_per_mol: f64,
    absolute_c_star_m_s: f64,
    absolute_mass_flow_kg_per_s: f64,
    absolute_isp_s: f64,
    absolute_thrust_n: f64,
}

#[derive(Debug, Deserialize)]
struct ReferenceCase {
    name: String,
    chamber_pressure_pa: f64,
    mixture_ratio: f64,
    expected_chamber_temperature_k: f64,
    expected_gamma: f64,
    expected_molecular_weight_kg_per_mol: f64,
    expected_c_star_m_s: f64,
    expected_nominal_mass_flow_kg_per_s: f64,
    expected_mass_flow_min_kg_per_s: f64,
    expected_mass_flow_max_kg_per_s: f64,
    expected_thrust_n: f64,
    expected_isp_s: f64,
}

#[test]
fn cantera_reference_tables_match_thermochem_and_liquid_performance() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let data_dir = root.join("data/thermochem");
    for reference_file in [
        "lox-lch4-cantera-gri30-tolerance-v1.toml",
        "lox-lh2-cantera-gri30-tolerance-v1.toml",
        "lox-rp1-ndodecane-cantera-reitz-tolerance-v1.toml",
        "n2o4-n2h4-cantera-nasa-gas-tolerance-v1.toml",
        "nto-mmh-nasa-cea-rocket-tolerance-v1.toml",
        "lox-rp1-nasa-cea-rocket-tolerance-v1.toml",
    ] {
        check_reference_table(&data_dir, reference_file);
    }
}

fn check_reference_table(data_dir: &Path, reference_file: &str) {
    let reference_path = data_dir.join(reference_file);
    let reference_text =
        std::fs::read_to_string(&reference_path).expect("reference tolerance table reads");
    let reference: ReferenceTable =
        toml::from_str(&reference_text).expect("reference tolerance table parses");
    assert_eq!(reference.openbmp.thermochem_reference, 1);
    assert!(
        reference.meta.source_title.contains("Cantera 3.2.0")
            || reference.meta.source_title.contains("NASA CEA 3.2.1"),
        "reference source title should pin generator version: {}",
        reference.meta.source_title
    );
    assert!(
        matches!(
            reference.meta.mechanism.as_str(),
            "gri30.yaml" | "nDodecane_Reitz.yaml" | "nasa_gas.yaml" | "thermo.lib"
        ),
        "unsupported mechanism in {reference_file}: {}",
        reference.meta.mechanism,
    );
    assert!(
        reference
            .meta
            .source_title
            .contains(&reference.meta.mechanism),
        "reference source title should name mechanism {}: {}",
        reference.meta.mechanism,
        reference.meta.source_title,
    );
    if reference.meta.mechanism == "nasa_gas.yaml" {
        let species_subset = reference
            .meta
            .species_subset
            .as_ref()
            .expect("nasa_gas reference should declare its species subset");
        assert!(
            species_subset.iter().any(|species| species == "N2O4")
                && species_subset.iter().any(|species| species == "N2H4"),
            "nasa_gas reference should anchor N2O4/N2H4 species: {species_subset:?}",
        );
        assert!(
            reference.meta.source_title.contains("species-subset")
                && reference.meta.source_title.contains("N2O4/N2H4"),
            "nasa_gas reference title should describe the hypergolic subset: {}",
            reference.meta.source_title,
        );
    } else {
        assert!(
            reference.meta.species_subset.is_none(),
            "{} should not declare a species subset",
            reference.meta.mechanism,
        );
    }
    if reference.meta.mechanism == "thermo.lib" {
        let reactants = reference
            .meta
            .reactants
            .as_ref()
            .expect("NASA CEA reference should declare reactants");
        assert!(
            reference.meta.source_title.contains("NASA CEA 3.2.1")
                && reference.meta.source_title.contains("rocket-equilibrium"),
            "CEA reference title should describe NASA CEA rocket equilibrium: {}",
            reference.meta.source_title,
        );
        assert!(
            reactants.iter().any(|reactant| reactant == "CH6N2(L)")
                || reactants.iter().any(|reactant| reactant == "RP-1"),
            "CEA reference should anchor either MMH or RP-1 reactants: {reactants:?}",
        );
    } else {
        assert!(
            reference.meta.reactants.is_none(),
            "{} should not declare CEA reactants",
            reference.meta.mechanism,
        );
    }
    assert_eq!(reference.meta.validation, "research");
    assert!(!reference.case.is_empty());

    let deck_path = data_dir.join(&reference.deck.file);
    let deck_text = std::fs::read_to_string(&deck_path).expect("reference deck reads");
    let deck = ThermochemTable::load_from_str(&deck_text).expect("reference deck parses");
    let nozzle = LiquidEngineNozzle {
        throat_area_m2: reference.nozzle.throat_area_m2,
        exit_area_m2: reference.nozzle.exit_area_m2,
        ambient_pressure_pa: reference.nozzle.ambient_pressure_pa,
        separation: nozzle_separation(&reference.nozzle.separation),
    };

    for case in &reference.case {
        let state = deck
            .lookup(ThermochemQuery {
                chamber_pressure_pa: case.chamber_pressure_pa,
                mixture_ratio: case.mixture_ratio,
            })
            .unwrap_or_else(|err| panic!("{} lookup failed: {err}", case.name));
        check_close(
            &case.name,
            "chamber_temperature_k",
            case.expected_chamber_temperature_k,
            state.chamber_temperature_k,
            reference.tolerances.absolute_temperature_k,
            reference.tolerances.relative,
        );
        check_close(
            &case.name,
            "gamma",
            case.expected_gamma,
            state.gamma,
            reference.tolerances.absolute_gamma,
            reference.tolerances.relative,
        );
        check_close(
            &case.name,
            "molecular_weight_kg_per_mol",
            case.expected_molecular_weight_kg_per_mol,
            state.molecular_weight_kg_per_mol,
            reference.tolerances.absolute_molecular_weight_kg_per_mol,
            reference.tolerances.relative,
        );
        check_close(
            &case.name,
            "c_star_m_s",
            case.expected_c_star_m_s,
            state.c_star_m_s,
            reference.tolerances.absolute_c_star_m_s,
            reference.tolerances.relative,
        );

        let performance = LiquidEnginePerformance::from_thermochemistry_with_efficiency(
            LiquidEngineThermochemistry {
                chamber_pressure_pa: case.chamber_pressure_pa,
                c_star_m_s: state.c_star_m_s,
                gamma: state.gamma,
            },
            nozzle,
            LiquidEngineCStarEfficiencyBand {
                min: state.c_star_efficiency.min,
                nominal: state.c_star_efficiency.nominal,
                max: state.c_star_efficiency.max,
            },
        )
        .unwrap_or_else(|err| panic!("{} liquid performance failed: {err}", case.name));

        check_close(
            &case.name,
            "nominal_mass_flow_kg_per_s",
            case.expected_nominal_mass_flow_kg_per_s,
            performance.mass_flow_kg_per_s,
            reference.tolerances.absolute_mass_flow_kg_per_s,
            reference.tolerances.relative,
        );
        check_close(
            &case.name,
            "mass_flow_min_kg_per_s",
            case.expected_mass_flow_min_kg_per_s,
            performance.mass_flow_band_kg_per_s.min,
            reference.tolerances.absolute_mass_flow_kg_per_s,
            reference.tolerances.relative,
        );
        check_close(
            &case.name,
            "mass_flow_max_kg_per_s",
            case.expected_mass_flow_max_kg_per_s,
            performance.mass_flow_band_kg_per_s.max,
            reference.tolerances.absolute_mass_flow_kg_per_s,
            reference.tolerances.relative,
        );
        check_close(
            &case.name,
            "thrust_n",
            case.expected_thrust_n,
            performance.max_thrust_n,
            reference.tolerances.absolute_thrust_n,
            reference.tolerances.relative,
        );
        check_close(
            &case.name,
            "isp_s",
            case.expected_isp_s,
            performance.isp_s,
            reference.tolerances.absolute_isp_s,
            reference.tolerances.relative,
        );
    }
}

fn nozzle_separation(value: &str) -> NozzleSeparationCriterion {
    match value {
        "off" => NozzleSeparationCriterion::Off,
        "summerfield" => NozzleSeparationCriterion::Summerfield,
        "schmucker" => NozzleSeparationCriterion::Schmucker,
        other => panic!("unsupported nozzle separation {other:?}"),
    }
}

fn check_close(
    case: &str,
    field: &str,
    expected: f64,
    actual: f64,
    absolute_tolerance: f64,
    relative_tolerance: f64,
) {
    assert!(expected.is_finite(), "{case}.{field} expected not finite");
    assert!(actual.is_finite(), "{case}.{field} actual not finite");
    let absolute_error = (expected - actual).abs();
    let relative_error = absolute_error / expected.abs().max(f64::MIN_POSITIVE);
    assert!(
        absolute_error <= absolute_tolerance || relative_error <= relative_tolerance,
        "{case}.{field}: expected={expected}, actual={actual}, abs={absolute_error}, rel={relative_error}, abs_tol={absolute_tolerance}, rel_tol={relative_tolerance}",
    );
}
