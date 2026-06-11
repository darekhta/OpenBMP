//! Import a generic FMI 3 co-simulation FMU and print its final typed output sample.

use std::path::PathBuf;

use openbmp_fmi::{
    Fmi3CoSimulationInstantiation, Fmi3CouplingOrder, Fmi3DynamicLibrary, Fmi3InputSample,
    Fmi3MasterPlan, Fmi3SingleFmuMaster, fmi3_binary_entry_for_current_platform,
    materialize_archive_resources,
};
use openbmp_models::FmuArchive;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args_os().skip(1);
    let Some(fmu_path) = args.next().map(PathBuf::from) else {
        return Err("usage: import_fmu_final_sample <input.fmu> <materialize-dir> <step-s> <steps> [float64-input ...]".into());
    };
    let Some(materialize_dir) = args.next().map(PathBuf::from) else {
        return Err("usage: import_fmu_final_sample <input.fmu> <materialize-dir> <step-s> <steps> [float64-input ...]".into());
    };
    let Some(step_s) = args.next() else {
        return Err("usage: import_fmu_final_sample <input.fmu> <materialize-dir> <step-s> <steps> [float64-input ...]".into());
    };
    let Some(steps) = args.next() else {
        return Err("usage: import_fmu_final_sample <input.fmu> <materialize-dir> <step-s> <steps> [float64-input ...]".into());
    };

    let step_s = step_s.to_string_lossy().parse::<f64>()?;
    let steps = steps.to_string_lossy().parse::<usize>()?;
    if !step_s.is_finite() || step_s <= 0.0 || steps == 0 {
        return Err("step-s must be positive finite and steps must be nonzero".into());
    }
    let float64_inputs = args
        .map(|value| value.to_string_lossy().parse::<f64>())
        .collect::<Result<Vec<_>, _>>()?;
    if float64_inputs.iter().any(|value| !value.is_finite()) {
        return Err("all Float64 inputs must be finite".into());
    }

    let archive = FmuArchive::load(&fmu_path)?;
    let description = archive.model_description();
    let binary_entry = fmi3_binary_entry_for_current_platform(&description.model_identifier)
        .ok_or("current target has no supported FMI 3 binary entry")?;
    let instantiation_token = description
        .instantiation_token
        .as_deref()
        .ok_or("modelDescription.xml is missing root instantiationToken")?;
    let stop_time_s = step_s * steps as f64;
    let resources_dir = materialize_archive_resources(&archive, &materialize_dir)?;
    let resource_path = resources_dir
        .as_ref()
        .map(std::fs::canonicalize)
        .transpose()?
        .map(|path| {
            let mut resource_path = path.to_string_lossy().into_owned();
            if !resource_path.ends_with(std::path::MAIN_SEPARATOR) {
                resource_path.push(std::path::MAIN_SEPARATOR);
            }
            resource_path
        })
        .unwrap_or_default();
    let library =
        Fmi3DynamicLibrary::open_archive_binary(&archive, binary_entry.as_str(), &materialize_dir)?;
    let mut instantiation =
        Fmi3CoSimulationInstantiation::new("openbmp-generic-import", instantiation_token);
    instantiation.resource_path = resource_path.as_str();
    let mut instance = library.instantiate_co_simulation(instantiation)?;
    instance.enter_initialization_mode(0.0, Some(stop_time_s), None)?;
    instance.exit_initialization_mode()?;

    let plan = Fmi3MasterPlan::from_archive(&archive, step_s, Fmi3CouplingOrder::GaussSeidel)?;
    if float64_inputs.len() != plan.variables.float64_inputs.len() {
        return Err(format!(
            "expected {} Float64 inputs but received {}",
            plan.variables.float64_inputs.len(),
            float64_inputs.len()
        )
        .into());
    }
    if !plan.variables.uint64_inputs.is_empty() {
        return Err("generic importer example does not accept UInt64 inputs".into());
    }
    if !plan.variables.int32_inputs.is_empty() {
        return Err("generic importer example does not accept Int32 inputs".into());
    }
    let mut master = Fmi3SingleFmuMaster::new(&library, instance.instance(), plan.clone());
    let mut last_result = None;
    for _ in 0..steps {
        last_result = Some(master.step(&Fmi3InputSample {
            float64: float64_inputs.clone(),
            int32: Vec::new(),
            uint64: Vec::new(),
        })?);
    }
    let result = last_result.ok_or("no FMI step was executed")?;
    instance.terminate()?;

    let mut header = vec!["time_s".to_owned()];
    header.extend(
        plan.variables
            .float64_outputs
            .iter()
            .map(|binding| binding.name.clone()),
    );
    header.extend(
        plan.variables
            .int32_outputs
            .iter()
            .map(|binding| binding.name.clone()),
    );
    header.extend(
        plan.variables
            .uint64_outputs
            .iter()
            .map(|binding| binding.name.clone()),
    );
    println!("{}", header.join(","));

    let mut fields = vec![format!("{:.17e}", result.current_time_s)];
    fields.extend(
        result
            .outputs
            .float64
            .iter()
            .map(|value| format!("{value:.17e}")),
    );
    fields.extend(result.outputs.int32.iter().map(i32::to_string));
    fields.extend(result.outputs.uint64.iter().map(u64::to_string));
    println!("{}", fields.join(","));
    Ok(())
}
