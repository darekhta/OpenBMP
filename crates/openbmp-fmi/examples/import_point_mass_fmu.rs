//! Import and step a restricted OpenBMP point-mass FMU.

use std::path::PathBuf;

use openbmp_fmi::export::{OPENBMP_POINT_MASS_INSTANTIATION_TOKEN, OPENBMP_POINT_MASS_VR_THROTTLE};
use openbmp_fmi::{
    Fmi3CoSimulationInstantiation, Fmi3CouplingOrder, Fmi3DynamicLibrary, Fmi3InputSample,
    Fmi3MasterPlan, Fmi3SingleFmuMaster, openbmp_point_mass_fmi_binary_entry_for_current_platform,
};
use openbmp_models::FmuArchive;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args_os().skip(1);
    let Some(fmu_path) = args.next().map(PathBuf::from) else {
        return Err(
            "usage: import_point_mass_fmu <input.fmu> <materialize-dir> <throttle> <step-s> <steps>"
                .into(),
        );
    };
    let Some(materialize_dir) = args.next().map(PathBuf::from) else {
        return Err(
            "usage: import_point_mass_fmu <input.fmu> <materialize-dir> <throttle> <step-s> <steps>"
                .into(),
        );
    };
    let Some(throttle) = args.next() else {
        return Err(
            "usage: import_point_mass_fmu <input.fmu> <materialize-dir> <throttle> <step-s> <steps>"
                .into(),
        );
    };
    let Some(step_s) = args.next() else {
        return Err(
            "usage: import_point_mass_fmu <input.fmu> <materialize-dir> <throttle> <step-s> <steps>"
                .into(),
        );
    };
    let Some(steps) = args.next() else {
        return Err(
            "usage: import_point_mass_fmu <input.fmu> <materialize-dir> <throttle> <step-s> <steps>"
                .into(),
        );
    };
    if args.next().is_some() {
        return Err(
            "usage: import_point_mass_fmu <input.fmu> <materialize-dir> <throttle> <step-s> <steps>"
                .into(),
        );
    }

    let throttle = throttle.to_string_lossy().parse::<f64>()?;
    let step_s = step_s.to_string_lossy().parse::<f64>()?;
    let steps = steps.to_string_lossy().parse::<usize>()?;
    if !throttle.is_finite() || !step_s.is_finite() || step_s <= 0.0 || steps == 0 {
        return Err("throttle and step-s must be finite, step-s must be positive, and steps must be nonzero".into());
    }

    let archive = FmuArchive::load(&fmu_path)?;
    let binary_entry = openbmp_point_mass_fmi_binary_entry_for_current_platform()
        .ok_or("current target has no supported point-mass FMU binary entry")?;
    let library = Fmi3DynamicLibrary::open_archive_binary(&archive, binary_entry, materialize_dir)?;
    let mut instance = library.instantiate_co_simulation(Fmi3CoSimulationInstantiation::new(
        "openbmp-point-mass-import",
        OPENBMP_POINT_MASS_INSTANTIATION_TOKEN,
    ))?;
    instance.enter_initialization_mode(0.0, Some(step_s * steps as f64), None)?;
    instance.set_float64(&[OPENBMP_POINT_MASS_VR_THROTTLE], &[throttle])?;
    instance.exit_initialization_mode()?;

    let plan = Fmi3MasterPlan::from_archive(&archive, step_s, Fmi3CouplingOrder::GaussSeidel)?;
    let mut master = Fmi3SingleFmuMaster::new(&library, instance.instance(), plan);
    let mut last_result = None;
    for _ in 0..steps {
        last_result = Some(master.step(&Fmi3InputSample {
            float64: vec![throttle],
            int32: Vec::new(),
            uint64: Vec::new(),
        })?);
    }
    let result = last_result.ok_or("no FMI step was executed")?;
    instance.terminate()?;

    let altitude_m = result
        .outputs
        .float64
        .first()
        .copied()
        .ok_or("missing altitude output")?;
    let velocity_m_s = result
        .outputs
        .float64
        .get(1)
        .copied()
        .ok_or("missing velocity output")?;
    let step = result
        .outputs
        .uint64
        .first()
        .copied()
        .ok_or("missing step output")?;

    println!("time_s,altitude_m,velocity_m_s,step");
    println!(
        "{:.17e},{:.17e},{:.17e},{}",
        result.current_time_s, altitude_m, velocity_m_s, step
    );
    Ok(())
}
