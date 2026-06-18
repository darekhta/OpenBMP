//! WP-12.4-b guard: GPU output may enter OpenBMP only as an offline,
//! provenance-pinned deck that the deterministic CPU path can reproduce
//! on a sampled subset.

#![allow(clippy::expect_used)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_core::SimTime;
use openbmp_physics::{AtmosphereModel, PiecewiseExponentialAtmosphere};
use serde::Deserialize;
use sha2::{Digest, Sha256};

const DECK_TEXT: &str = include_str!("../../../data/atmosphere/gpu-offline-density-subset-v1.toml");
const PROVENANCE_TEXT: &str = include_str!("../../../data/atmosphere/provenance.md");
const DECK_PATH: &str = "data/atmosphere/gpu-offline-density-subset-v1.toml";
const DECK_SHA256: &str = "74dea5cca998bd9e3e3af88d3f8784906be5a1050b910e3c89067490f15e2d96";

#[derive(Debug, Deserialize)]
struct GpuOfflineDensityDeck {
    openbmp: OpenBmpDiscriminator,
    meta: GpuOfflineDensityMeta,
    sample: Vec<GpuOfflineDensitySample>,
}

#[derive(Debug, Deserialize)]
struct OpenBmpDiscriminator {
    gpu_offload_boundary_deck: u32,
}

#[derive(Debug, Deserialize)]
struct GpuOfflineDensityMeta {
    dataset_id: String,
    producer: String,
    source_model: String,
    provenance: String,
    validation_status: String,
    no_live_gpu_path: bool,
    uq_relative: f64,
    uq_evidence: String,
}

#[derive(Debug, Deserialize)]
struct GpuOfflineDensitySample {
    altitude_m: f64,
    density_kg_m3: f64,
    reproduce: bool,
}

#[test]
fn gpu_offline_density_deck_reproduces_cpu_subset_within_uq() {
    let deck: GpuOfflineDensityDeck = toml::from_str(DECK_TEXT).expect("deck parses");
    assert_eq!(deck.openbmp.gpu_offload_boundary_deck, 1);
    assert_eq!(
        deck.meta.dataset_id,
        "openbmp.det.gpu_offline.atmosphere_density.v1"
    );
    assert_eq!(deck.meta.producer, "external-gpu-offline-synthetic");
    assert_eq!(
        deck.meta.source_model,
        "openbmp.physics.PiecewiseExponentialAtmosphere"
    );
    assert_eq!(deck.meta.provenance, "data/atmosphere/provenance.md");
    assert_eq!(deck.meta.validation_status, "checked");
    assert!(deck.meta.no_live_gpu_path);
    assert!(deck.meta.uq_relative > 0.0 && deck.meta.uq_relative <= 0.005);
    assert!(deck.meta.uq_evidence.contains("offline"));
    assert!(deck.sample.len() >= 5);

    let cpu_model = PiecewiseExponentialAtmosphere::new();
    for sample in deck.sample.iter().filter(|sample| sample.reproduce) {
        assert!(sample.altitude_m.is_finite());
        assert!(sample.density_kg_m3.is_finite());
        assert!(sample.altitude_m >= 0.0);
        assert!(sample.density_kg_m3 >= 0.0);
        let cpu = cpu_model
            .sample(sample.altitude_m, SimTime::ZERO)
            .expect("CPU atmosphere sample");
        let relative_error = (sample.density_kg_m3 - cpu.density_kg_m3).abs() / cpu.density_kg_m3;
        assert!(
            relative_error <= deck.meta.uq_relative,
            "altitude {} m density relative error {} exceeded UQ band {}",
            sample.altitude_m,
            relative_error,
            deck.meta.uq_relative
        );
    }
}

#[test]
fn gpu_offline_density_deck_has_provenance_sha_pin() {
    let digest = Sha256::digest(DECK_TEXT.as_bytes());
    let hex = digest
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<String>();
    assert_eq!(hex, DECK_SHA256);
    assert!(PROVENANCE_TEXT.contains(DECK_PATH));
    assert!(PROVENANCE_TEXT.contains(DECK_SHA256));
    assert!(PROVENANCE_TEXT.contains("openbmp.det.gpu_offline.atmosphere_density.v1"));
}

#[test]
fn workspace_manifests_do_not_add_gpu_framework_dependency() {
    let root = workspace_root();
    let mut manifests = vec![root.join("Cargo.toml")];
    let crates_dir = root.join("crates");
    for entry in fs::read_dir(&crates_dir).expect("read crates dir") {
        let entry = entry.expect("crate dir entry");
        let manifest = entry.path().join("Cargo.toml");
        if manifest.is_file() {
            manifests.push(manifest);
        }
    }

    let forbidden = [
        "wgpu", "cust", "cudarc", "vulkano", "ocl", "opencl3", "metal", "ash",
    ];
    for manifest in manifests {
        let text = fs::read_to_string(&manifest).expect("read Cargo.toml");
        for (line_index, line) in text.lines().enumerate() {
            let line = line.trim_start().to_ascii_lowercase();
            if line.starts_with('#') {
                continue;
            }
            for dependency in forbidden {
                let starts_with_dependency = line.starts_with(&format!("{dependency} "))
                    || line.starts_with(&format!("{dependency}="))
                    || line.starts_with(&format!("{dependency}."));
                assert!(
                    !starts_with_dependency,
                    "{}:{} declares GPU framework dependency `{}`",
                    display_path(&root, &manifest),
                    line_index + 1,
                    dependency
                );
            }
        }
    }
}

fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("workspace root")
        .to_path_buf()
}

fn display_path(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .to_string_lossy()
        .into_owned()
}
