//! Shared catalog of already-rendered acoustics output artifacts.

use std::path::Path;

use openbmp_runner::acoustics::AcousticsRunReport;
use openbmp_runner::structural::StructuralAcousticLoadsReport;

use crate::error::CliError;

/// Visit existing artifact bytes in output order without writing or filtering them.
pub(super) fn visit_acoustics_artifacts(
    acoustics: &AcousticsRunReport,
    structural: Option<&StructuralAcousticLoadsReport>,
    mut visit: impl FnMut(&Path, &'static str, &[u8]) -> Result<(), CliError>,
) -> Result<(), CliError> {
    if let Some(ascent) = &acoustics.ascent {
        visit(
            Path::new("ascent/p95_50_environment.csv"),
            "acoustics_ascent_p95_50_environment_csv",
            &ascent.p95_50_environment_csv,
        )?;
        visit(
            Path::new("ascent/p95_50_wilks_sizing.csv"),
            "acoustics_ascent_p95_50_wilks_sizing_csv",
            &ascent.p95_50_wilks_sizing_csv,
        )?;
    }

    if let Some(fairing) = &acoustics.fairing_internal {
        visit(
            Path::new("fairing_internal/p95_50_environment.csv"),
            "acoustics_fairing_internal_p95_50_environment_csv",
            &fairing.p95_50_environment_csv,
        )?;
        visit(
            Path::new("fairing_internal/p95_50_sample_trace.csv"),
            "acoustics_fairing_internal_p95_50_sample_trace_csv",
            &fairing.p95_50_sample_trace_csv,
        )?;
        visit(
            Path::new("fairing_internal/p95_50_wilks_sizing.csv"),
            "acoustics_fairing_internal_p95_50_wilks_sizing_csv",
            &fairing.p95_50_wilks_sizing_csv,
        )?;
    }

    if let Some(response) = &acoustics.response {
        visit(
            Path::new("response/loads_handoff.csv"),
            "acoustics_response_loads_handoff_csv",
            &response.loads_handoff_csv,
        )?;
        visit(
            Path::new("response/p95_50_acceleration_psd.csv"),
            "acoustics_response_p95_50_acceleration_psd_csv",
            &response.p95_50_acceleration_psd_csv,
        )?;
        visit(
            Path::new("response/p95_50_modal_force_psd.csv"),
            "acoustics_response_p95_50_modal_force_psd_csv",
            &response.p95_50_modal_force_psd_csv,
        )?;
        visit(
            Path::new("response/p95_50_sample_trace.csv"),
            "acoustics_response_p95_50_sample_trace_csv",
            &response.p95_50_sample_trace_csv,
        )?;
        visit(
            Path::new("response/p95_50_wilks_sizing.csv"),
            "acoustics_response_p95_50_wilks_sizing_csv",
            &response.p95_50_wilks_sizing_csv,
        )?;
    }

    if let Some(structural) = structural {
        visit(
            Path::new("structural/first_bending_response.csv"),
            "structural_acoustics_first_bending_response_csv",
            &structural.band_response_csv,
        )?;
        visit(
            Path::new("structural/first_bending_line_load_recovery.csv"),
            "structural_acoustics_first_bending_line_load_recovery_csv",
            &structural.line_load_recovery_csv,
        )?;
    }

    Ok(())
}
