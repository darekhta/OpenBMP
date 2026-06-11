//! Print the restricted OpenBMP point-mass FMI 3 `modelDescription.xml`.

fn main() -> Result<(), openbmp_fmi::FmiExportError> {
    let xml = openbmp_fmi::openbmp_point_mass_export_model().model_description_xml()?;
    print!("{xml}");
    Ok(())
}
