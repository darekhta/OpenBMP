//! Write a restricted OpenBMP point-mass FMU from an existing shared library.

use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args_os().skip(1);
    let Some(fmu_path) = args.next().map(PathBuf::from) else {
        return Err("usage: write_point_mass_fmu <output.fmu> <shared-library>".into());
    };
    let Some(library_path) = args.next().map(PathBuf::from) else {
        return Err("usage: write_point_mass_fmu <output.fmu> <shared-library>".into());
    };
    if args.next().is_some() {
        return Err("usage: write_point_mass_fmu <output.fmu> <shared-library>".into());
    }

    let library_bytes = std::fs::read(&library_path)?;
    let entry = openbmp_fmi::write_openbmp_point_mass_fmu_archive_for_current_platform(
        &fmu_path,
        &library_bytes,
    )?;
    println!("{entry}");
    Ok(())
}
