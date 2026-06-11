//! Python bindings for the native OpenBMP SIL testbench API.
//!
//! The exported Python module is named `openbmp_sil`. It intentionally returns
//! structured reports as JSON strings so Python test harnesses can consume the
//! same evidence shapes produced by the Rust CLI without depending on Rust
//! struct layout.

use std::fs;
use std::path::Path;

use openbmp_sil_native::{
    CommandWrite, EvidenceBundle, FaultInjection, FaultTarget, MissionPackage, ParameterOverride,
    RunUntilTarget, SilRunReport, SilStimulation, SilTestbench, capture_bus_frames, read_signal,
    write_evidence_bundle,
};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use serde::{Deserialize, Serialize};
use serde_json::{Value as JsonValue, json};

#[pyclass(name = "SilTestbench")]
#[derive(Debug)]
struct PySilTestbench {
    inner: SilTestbench,
}

#[pymethods]
impl PySilTestbench {
    #[new]
    fn new(package: &str) -> PyResult<Self> {
        Self::load(package)
    }

    #[staticmethod]
    fn load(package: &str) -> PyResult<Self> {
        Ok(Self {
            inner: SilTestbench::load(package).map_err(py_runtime_err)?,
        })
    }

    fn reset(&mut self) -> PyResult<()> {
        self.inner.reset().map_err(py_runtime_err)
    }

    fn check_json(&self) -> PyResult<String> {
        json_string(&self.inner.package().check().map_err(py_runtime_err)?)
    }

    fn materialize_sidecars_json(&self) -> PyResult<String> {
        json_string(
            &self
                .inner
                .package()
                .materialize_sidecars()
                .map_err(py_runtime_err)?,
        )
    }

    fn run(&self, case_id: Option<&str>) -> PyResult<PySilRunReport> {
        Ok(PySilRunReport {
            inner: self.inner.run(case_id).map_err(py_runtime_err)?,
        })
    }

    fn step(&self, case_id: Option<&str>, ticks: u64) -> PyResult<PySilRunReport> {
        Ok(PySilRunReport {
            inner: self.inner.step(case_id, ticks).map_err(py_runtime_err)?,
        })
    }

    fn run_with_stimulation_json(
        &self,
        case_id: Option<&str>,
        stimulation_json: &str,
    ) -> PyResult<PySilRunReport> {
        let stimulation = parse_stimulation_json(stimulation_json)?;
        Ok(PySilRunReport {
            inner: self
                .inner
                .run_with_stimulation(case_id, &stimulation)
                .map_err(py_runtime_err)?,
        })
    }

    fn step_with_stimulation_json(
        &self,
        case_id: Option<&str>,
        ticks: u64,
        stimulation_json: &str,
    ) -> PyResult<PySilRunReport> {
        let stimulation = parse_stimulation_json(stimulation_json)?;
        Ok(PySilRunReport {
            inner: self
                .inner
                .step_with_stimulation(case_id, ticks, &stimulation)
                .map_err(py_runtime_err)?,
        })
    }

    fn run_until_time(&self, case_id: Option<&str>, time_s: f64) -> PyResult<PySilRunReport> {
        Ok(PySilRunReport {
            inner: self
                .inner
                .run_until(case_id, RunUntilTarget::TimeS(time_s))
                .map_err(py_runtime_err)?,
        })
    }

    fn run_until_event(&self, case_id: Option<&str>, event: &str) -> PyResult<PySilRunReport> {
        if event.trim().is_empty() {
            return Err(PyValueError::new_err("event must be non-empty"));
        }
        Ok(PySilRunReport {
            inner: self
                .inner
                .run_until(case_id, RunUntilTarget::Event(event.to_owned()))
                .map_err(py_runtime_err)?,
        })
    }

    fn run_until_phase(&self, case_id: Option<&str>, phase: &str) -> PyResult<PySilRunReport> {
        if phase.trim().is_empty() {
            return Err(PyValueError::new_err("phase must be non-empty"));
        }
        Ok(PySilRunReport {
            inner: self
                .inner
                .run_until(case_id, RunUntilTarget::Phase(phase.to_owned()))
                .map_err(py_runtime_err)?,
        })
    }
}

#[pyclass(name = "SilRunReport")]
#[derive(Debug)]
struct PySilRunReport {
    inner: SilRunReport,
}

#[pymethods]
impl PySilRunReport {
    #[getter]
    fn package_id(&self) -> &str {
        &self.inner.package_id
    }

    #[getter]
    fn case_id(&self) -> &str {
        &self.inner.case_id
    }

    #[getter]
    fn stop_label(&self) -> &str {
        &self.inner.stop_label
    }

    #[getter]
    fn final_step(&self) -> u64 {
        self.inner.final_step
    }

    #[getter]
    fn final_time_s(&self) -> f64 {
        self.inner.final_time_s
    }

    fn summary_json(&self) -> PyResult<String> {
        json_string(&json!({
            "package_id": self.inner.package_id,
            "case_id": self.inner.case_id,
            "stop_label": self.inner.stop_label,
            "final_step": self.inner.final_step,
            "final_time_s": self.inner.final_time_s,
            "verdict": self.inner.evidence.verdict,
            "telemetry_rows": self.inner.evidence.telemetry_rows,
            "telemetry_channels": self.inner.evidence.telemetry_channels,
        }))
    }

    fn evidence_json(&self) -> PyResult<String> {
        json_string(&self.inner.evidence)
    }

    fn requirement_verdicts_json(&self) -> PyResult<String> {
        json_string(&self.inner.evidence.requirement_verdicts)
    }

    fn read_signal_json(&self, channel: &str, max_preview: usize) -> PyResult<String> {
        json_string(&read_signal(&self.inner, channel, max_preview).map_err(py_runtime_err)?)
    }

    fn bus_frames_json(&self) -> PyResult<String> {
        json_string(&capture_bus_frames(&self.inner))
    }

    fn write_evidence(&self, directory: &str) -> PyResult<String> {
        let path =
            write_evidence_bundle(directory, &self.inner.evidence).map_err(py_runtime_err)?;
        Ok(path.display().to_string())
    }
}

#[pyfunction]
fn check_package_json(package: &str) -> PyResult<String> {
    let package = MissionPackage::load(package).map_err(py_runtime_err)?;
    json_string(&package.check().map_err(py_runtime_err)?)
}

#[pyfunction]
fn materialize_sidecars_json(package: &str) -> PyResult<String> {
    let package = MissionPackage::load(package).map_err(py_runtime_err)?;
    json_string(&package.materialize_sidecars().map_err(py_runtime_err)?)
}

#[pyfunction]
fn verdict_json(manifest: &str) -> PyResult<String> {
    let evidence = read_evidence(manifest)?;
    json_string(&evidence)
}

#[pyfunction]
fn verdict_summary_json(manifest: &str) -> PyResult<String> {
    let evidence = read_evidence(manifest)?;
    json_string(&json!({
        "package_id": evidence.package_id,
        "case_id": evidence.case_id,
        "scenario_name": evidence.scenario_name,
        "stop_label": evidence.stop_label,
        "verdict": evidence.verdict,
        "events": evidence.event_trace.len(),
        "phases": evidence.phase_trace.len(),
        "regions": evidence.region_trace.len(),
        "commands": evidence.command_trace.len(),
        "bus_frames": evidence.bus_frames.len(),
        "failures": evidence.failures.len(),
    }))
}

#[pymodule]
fn openbmp_sil(_py: Python<'_>, module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PySilTestbench>()?;
    module.add_class::<PySilRunReport>()?;
    module.add_function(wrap_pyfunction!(check_package_json, module)?)?;
    module.add_function(wrap_pyfunction!(materialize_sidecars_json, module)?)?;
    module.add_function(wrap_pyfunction!(verdict_json, module)?)?;
    module.add_function(wrap_pyfunction!(verdict_summary_json, module)?)?;
    Ok(())
}

fn read_evidence(path: &str) -> PyResult<EvidenceBundle> {
    let path = Path::new(path);
    let text = fs::read_to_string(path)
        .map_err(|source| PyRuntimeError::new_err(format!("read {}: {source}", path.display())))?;
    serde_json::from_str(&text).map_err(|source| {
        PyRuntimeError::new_err(format!("parse evidence {}: {source}", path.display()))
    })
}

fn json_string(value: &impl Serialize) -> PyResult<String> {
    serde_json::to_string_pretty(value).map_err(py_runtime_err)
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StimulationSpec {
    #[serde(default)]
    faults: Vec<FaultSpec>,
    #[serde(default)]
    parameter_overrides: Vec<ParameterOverrideSpec>,
    #[serde(default)]
    command_writes: Vec<CommandWriteSpec>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FaultSpec {
    target_kind: String,
    id: String,
    fault: JsonValue,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ParameterOverrideSpec {
    path: String,
    value: JsonValue,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CommandWriteSpec {
    kind: String,
    time_s: f64,
    id: String,
    #[serde(default)]
    throttle_unit: Option<f64>,
    #[serde(default)]
    gimbal_pitch_rad: Option<f64>,
    #[serde(default)]
    gimbal_yaw_rad: Option<f64>,
    #[serde(default)]
    ignite: Option<bool>,
    #[serde(default)]
    shutdown: Option<bool>,
    #[serde(default)]
    command: Option<f64>,
}

fn parse_stimulation_json(text: &str) -> PyResult<SilStimulation> {
    let spec: StimulationSpec = serde_json::from_str(text)
        .map_err(|source| PyValueError::new_err(format!("invalid stimulation JSON: {source}")))?;
    let faults = spec
        .faults
        .into_iter()
        .map(parse_fault)
        .collect::<PyResult<Vec<_>>>()?;
    let parameter_overrides = spec
        .parameter_overrides
        .into_iter()
        .map(parse_parameter_override)
        .collect::<PyResult<Vec<_>>>()?;
    let command_writes = spec
        .command_writes
        .into_iter()
        .map(parse_command_write)
        .collect::<PyResult<Vec<_>>>()?;
    Ok(SilStimulation {
        faults,
        parameter_overrides,
        command_writes,
    })
}

fn parse_fault(spec: FaultSpec) -> PyResult<FaultInjection> {
    require_nonempty("faults[].id", &spec.id)?;
    let target = match spec.target_kind.as_str() {
        "engine" => FaultTarget::Engine(spec.id),
        "effector" => FaultTarget::Effector(spec.id),
        other => {
            return Err(PyValueError::new_err(format!(
                "faults[].target_kind must be `engine` or `effector`, got `{other}`"
            )));
        }
    };
    Ok(FaultInjection {
        target,
        fault: json_to_toml_value(spec.fault)?,
    })
}

fn parse_parameter_override(spec: ParameterOverrideSpec) -> PyResult<ParameterOverride> {
    require_nonempty("parameter_overrides[].path", &spec.path)?;
    Ok(ParameterOverride {
        path: spec.path,
        value: json_to_toml_value(spec.value)?,
    })
}

fn parse_command_write(spec: CommandWriteSpec) -> PyResult<CommandWrite> {
    require_nonempty("command_writes[].id", &spec.id)?;
    require_finite("command_writes[].time_s", spec.time_s)?;
    match spec.kind.as_str() {
        "engine" | "engine_command" => {
            let throttle_unit =
                finite_or_default("command_writes[].throttle_unit", spec.throttle_unit, 0.0)?;
            let gimbal_pitch_rad = finite_or_default(
                "command_writes[].gimbal_pitch_rad",
                spec.gimbal_pitch_rad,
                0.0,
            )?;
            let gimbal_yaw_rad =
                finite_or_default("command_writes[].gimbal_yaw_rad", spec.gimbal_yaw_rad, 0.0)?;
            Ok(CommandWrite::Engine {
                time_s: spec.time_s,
                id: spec.id,
                throttle_unit,
                gimbal_pitch_rad,
                gimbal_yaw_rad,
                ignite: spec.ignite.unwrap_or(false),
                shutdown: spec.shutdown.unwrap_or(false),
            })
        }
        "effector" | "effector_override" => {
            let command = spec.command.ok_or_else(|| {
                PyValueError::new_err("command_writes[].command is required for effector writes")
            })?;
            require_finite("command_writes[].command", command)?;
            Ok(CommandWrite::Effector {
                time_s: spec.time_s,
                id: spec.id,
                command,
            })
        }
        other => Err(PyValueError::new_err(format!(
            "command_writes[].kind must be `engine` or `effector`, got `{other}`"
        ))),
    }
}

fn json_to_toml_value(value: JsonValue) -> PyResult<toml::Value> {
    match value {
        JsonValue::Null => Err(PyValueError::new_err(
            "JSON null cannot be represented as a TOML value",
        )),
        JsonValue::Bool(value) => Ok(toml::Value::Boolean(value)),
        JsonValue::Number(number) => {
            if let Some(value) = number.as_i64() {
                Ok(toml::Value::Integer(value))
            } else if let Some(value) = number.as_f64() {
                require_finite("numeric TOML value", value)?;
                Ok(toml::Value::Float(value))
            } else {
                Err(PyValueError::new_err(format!(
                    "number `{number}` cannot be represented as TOML"
                )))
            }
        }
        JsonValue::String(value) => Ok(toml::Value::String(value)),
        JsonValue::Array(values) => values
            .into_iter()
            .map(json_to_toml_value)
            .collect::<PyResult<Vec<_>>>()
            .map(toml::Value::Array),
        JsonValue::Object(values) => {
            let mut table = toml::map::Map::new();
            for (key, value) in values {
                table.insert(key, json_to_toml_value(value)?);
            }
            Ok(toml::Value::Table(table))
        }
    }
}

fn require_nonempty(field: &str, value: &str) -> PyResult<()> {
    if value.trim().is_empty() {
        return Err(PyValueError::new_err(format!("{field} must be non-empty")));
    }
    Ok(())
}

fn require_finite(field: &str, value: f64) -> PyResult<()> {
    if !value.is_finite() {
        return Err(PyValueError::new_err(format!("{field} must be finite")));
    }
    Ok(())
}

fn finite_or_default(field: &str, value: Option<f64>, default: f64) -> PyResult<f64> {
    let value = value.unwrap_or(default);
    require_finite(field, value)?;
    Ok(value)
}

fn py_runtime_err(error: impl std::error::Error) -> PyErr {
    let mut message = error.to_string();
    let mut source = error.source();
    while let Some(error) = source {
        message.push_str(": ");
        message.push_str(&error.to_string());
        source = error.source();
    }
    PyRuntimeError::new_err(message)
}

// The `extension-module` pyo3 feature deliberately does NOT link
// libpython (the final `.so` resolves those symbols against the host
// interpreter at load time). That is correct for the shipped extension
// but means a standalone test executable — which calls
// `Python::initialize()` — cannot link. `--all-features` (used by the
// workspace CI gate) turns `extension-module` on, so this test is gated
// off in that configuration and exercised by a dedicated default-feature
// CI step (`cargo test -p openbmp-sil-py`) instead.
#[cfg(all(test, not(feature = "extension-module")))]
#[allow(clippy::expect_used)]
mod tests {
    use std::fs;
    use std::path::{Path, PathBuf};

    use serde_json::Value;
    use tempfile::TempDir;

    use super::*;

    #[test]
    fn python_wrapper_checks_steps_reads_and_writes_evidence() {
        Python::initialize();
        let fixture = SilFixture::new();
        let bench = PySilTestbench::load(fixture.manifest.to_str().expect("manifest path"))
            .expect("load Python SIL wrapper");

        let check: Value =
            serde_json::from_str(&bench.check_json().expect("check json")).expect("check parse");
        assert_eq!(check["package_id"], "py.pkg");

        let report = bench.step(Some("smoke"), 3).expect("step package");
        assert_eq!(report.package_id(), "py.pkg");
        assert_eq!(report.case_id(), "smoke");
        assert!(report.final_step() <= 3);

        let signal: Value = serde_json::from_str(
            &report
                .read_signal_json("mission.phase", 2)
                .expect("read signal json"),
        )
        .expect("signal parse");
        assert_eq!(signal["channel"], "mission.phase");

        let frames: Value =
            serde_json::from_str(&report.bus_frames_json().expect("bus json")).expect("bus parse");
        assert!(frames.as_array().is_some());

        let stimulated = bench
            .step_with_stimulation_json(
                Some("smoke"),
                3,
                r#"
{
  "parameter_overrides": [
    { "path": "time.stop_s", "value": 0.03 }
  ],
  "command_writes": [
    { "kind": "effector", "time_s": 0.01, "id": "delta", "command": 0.2 }
  ]
}
"#,
            )
            .expect("step with stimulation");
        assert_eq!(stimulated.inner.evidence.stimuli.len(), 2);
        let stimulated_effector: Value = serde_json::from_str(
            &stimulated
                .read_signal_json("effector.delta.actual", 4)
                .expect("read stimulated effector"),
        )
        .expect("stimulated effector parse");
        let stimulated_max = stimulated_effector["max"]
            .as_str()
            .expect("stimulated max string")
            .parse::<f64>()
            .expect("parse stimulated max");
        assert!(stimulated_max > 0.1);

        let evidence_dir = fixture.root.path().join("evidence");
        let manifest = report
            .write_evidence(path_str(&evidence_dir))
            .expect("write evidence");
        let verdict: Value =
            serde_json::from_str(&verdict_summary_json(&manifest).expect("verdict summary"))
                .expect("verdict parse");
        assert_eq!(verdict["package_id"], "py.pkg");
    }

    struct SilFixture {
        root: TempDir,
        manifest: PathBuf,
    }

    impl SilFixture {
        fn new() -> Self {
            let root = tempfile::tempdir().expect("tempdir");
            let scenario = root.path().join("scenario.toml");
            fs::write(
                &scenario,
                include_str!("../tests/fixtures/py-sil-scenario.toml"),
            )
            .expect("write scenario");
            let digest = openbmp_sil_native::sha256_file(&scenario).expect("scenario digest");
            let manifest = root.path().join("mission-package.toml");
            fs::write(
                &manifest,
                format!(
                    r#"
[package]
id = "py.pkg"
version = "0.1.0"

[files]
scenario = "scenario.toml"

[hashes]
"scenario.toml" = "{digest}"

[[test_cases]]
id = "smoke"
"#
                ),
            )
            .expect("write manifest");
            Self { root, manifest }
        }
    }

    fn path_str(path: &Path) -> &str {
        path.to_str().expect("utf-8 path")
    }
}
