//! `openbmp migrate ...` scenario rewrite helpers.

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_scenario::{SCENARIO_VERSION_V3, Scenario};
use toml_edit::{ArrayOfTables, DocumentMut, InlineTable, Item, Table, Value};

use crate::CliError;

/// Outcome of `openbmp migrate mission-script-split`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MissionScriptSplitReport {
    /// Scenario path rewritten in place.
    pub path: PathBuf,
    /// Number of event tables moved from `mission.events`.
    pub moved_events: usize,
    /// Whether the file content changed.
    pub written: bool,
}

/// Move simulator-owned event actions from `[[mission.events]]` to
/// `[[scenario_script.events]]` and validate the rewritten scenario.
///
/// # Errors
///
/// Returns [`CliError`] when the file cannot be read/written, the TOML
/// cannot be edited, or the rewritten scenario fails normal validation.
pub fn mission_script_split(path: &Path) -> Result<MissionScriptSplitReport, CliError> {
    let original = fs::read_to_string(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let mut document: DocumentMut = original.parse().map_err(|source| CliError::MigrateToml {
        path: path.to_path_buf(),
        source,
    })?;

    let moved_events = migrate_document(&mut document, path)?;
    let migrated = document.to_string();
    let written = moved_events > 0 && migrated != original;
    let source_dir = path.parent().map(Path::to_path_buf);
    if written {
        Scenario::from_toml_str_with_source_dir(&migrated, source_dir)?;
    } else {
        Scenario::from_toml_str_with_source_dir(&original, source_dir)?;
    }
    if written {
        fs::write(path, migrated).map_err(|source| CliError::Io {
            path: path.to_path_buf(),
            source,
        })?;
    }

    Ok(MissionScriptSplitReport {
        path: path.to_path_buf(),
        moved_events,
        written,
    })
}

fn migrate_document(document: &mut DocumentMut, path: &Path) -> Result<usize, CliError> {
    let transition_events = mission_transition_event_ids(document);
    let Some(mission_events_item) = document
        .get_mut("mission")
        .and_then(Item::as_table_mut)
        .and_then(|mission| mission.get_mut("events"))
    else {
        return Ok(0);
    };
    let Some(mission_events) = mission_events_item.as_array_of_tables_mut() else {
        return Err(CliError::Migrate {
            path: path.to_path_buf(),
            summary: "mission.events is not an array of tables".to_owned(),
        });
    };

    let mut moved = Vec::new();
    let mut index = 0;
    while index < mission_events.len() {
        let Some(table) = mission_events.get(index) else {
            return Err(CliError::Migrate {
                path: path.to_path_buf(),
                summary: "mission.events changed while migrating".to_owned(),
            });
        };
        if is_script_action_event(table) {
            let event_id = event_id(table).map(str::to_owned);
            if event_id
                .as_deref()
                .is_some_and(|event_id| transition_events.iter().any(|event| event == event_id))
            {
                let script_event = table.clone();
                let Some(table) = mission_events.get_mut(index) else {
                    return Err(CliError::Migrate {
                        path: path.to_path_buf(),
                        summary: "mission.events changed while migrating".to_owned(),
                    });
                };
                replace_action_with_marker(table, event_id.as_deref().unwrap_or("event"));
                moved.push(script_event);
                index += 1;
            } else {
                moved.push(mission_events.remove(index));
            }
        } else {
            index += 1;
        }
    }

    if moved.is_empty() {
        return Ok(0);
    }

    if mission_events.is_empty()
        && let Some(mission) = document.get_mut("mission").and_then(Item::as_table_mut)
    {
        mission.remove("events");
    }

    promote_header_to_v3(document, path)?;

    let script_events = scenario_script_events_mut(document, path)?;
    let moved_count = moved.len();
    for table in moved {
        script_events.push(table);
    }
    Ok(moved_count)
}

fn promote_header_to_v3(document: &mut DocumentMut, path: &Path) -> Result<(), CliError> {
    let Some(openbmp) = document.get_mut("openbmp").and_then(Item::as_table_mut) else {
        return Err(CliError::Migrate {
            path: path.to_path_buf(),
            summary: "missing [openbmp] header".to_owned(),
        });
    };
    let Some(current) = openbmp
        .get("scenario")
        .and_then(Item::as_value)
        .and_then(Value::as_integer)
    else {
        return Err(CliError::Migrate {
            path: path.to_path_buf(),
            summary: "openbmp.scenario is missing or is not an integer".to_owned(),
        });
    };
    if current < i64::from(SCENARIO_VERSION_V3) {
        openbmp.insert(
            "scenario",
            Item::Value(Value::from(i64::from(SCENARIO_VERSION_V3))),
        );
    }
    Ok(())
}

fn scenario_script_events_mut<'a>(
    document: &'a mut DocumentMut,
    path: &Path,
) -> Result<&'a mut ArrayOfTables, CliError> {
    if document.get("scenario_script").is_none() {
        document.insert("scenario_script", Item::Table(Table::new()));
    }
    let Some(script_table) = document
        .get_mut("scenario_script")
        .and_then(Item::as_table_mut)
    else {
        return Err(CliError::Migrate {
            path: path.to_path_buf(),
            summary: "scenario_script exists but is not a table".to_owned(),
        });
    };
    if script_table.get("events").is_none() {
        script_table.insert("events", Item::ArrayOfTables(ArrayOfTables::new()));
    }
    script_table
        .get_mut("events")
        .and_then(Item::as_array_of_tables_mut)
        .ok_or_else(|| CliError::Migrate {
            path: path.to_path_buf(),
            summary: "scenario_script.events exists but is not an array of tables".to_owned(),
        })
}

fn is_script_action_event(table: &Table) -> bool {
    action_kind(table).is_some_and(is_script_action_kind)
}

fn event_id(table: &Table) -> Option<&str> {
    table.get("id")?.as_value()?.as_str()
}

fn action_kind(table: &Table) -> Option<&str> {
    match table.get("action")? {
        Item::Value(Value::InlineTable(action)) => inline_table_kind(action),
        Item::Table(action) => table_kind(action),
        _ => None,
    }
}

fn inline_table_kind(action: &InlineTable) -> Option<&str> {
    action.get("kind")?.as_str()
}

fn table_kind(action: &Table) -> Option<&str> {
    action.get("kind")?.as_value()?.as_str()
}

fn is_script_action_kind(kind: &str) -> bool {
    matches!(
        kind,
        "engine_command"
            | "effector_override"
            | "deploy_recovery"
            | "jettison_stage"
            | "jettison_bodies"
            | "separation"
            | "select_guidance_profile"
    )
}

fn mission_transition_event_ids(document: &DocumentMut) -> Vec<String> {
    document
        .get("mission")
        .and_then(Item::as_table)
        .and_then(|mission| mission.get("transitions"))
        .and_then(Item::as_array_of_tables)
        .map(|transitions| {
            transitions
                .iter()
                .filter_map(|transition| {
                    transition
                        .get("event")
                        .and_then(Item::as_value)
                        .and_then(Value::as_str)
                        .map(str::to_owned)
                })
                .collect()
        })
        .unwrap_or_default()
}

fn replace_action_with_marker(table: &mut Table, tag: &str) {
    let mut marker = InlineTable::new();
    marker.insert("kind", Value::from("emit_telemetry_marker"));
    marker.insert("tag", Value::from(tag));
    table.insert("action", Item::Value(Value::InlineTable(marker)));
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    const ENGINE_COMMAND_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../openbmp-scenario/tests/fixtures/assembly-engine-cluster-with-engine-command.toml"
    ));

    #[test]
    fn mission_script_split_moves_engine_command_event() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("scenario.toml");
        fs::write(&path, ENGINE_COMMAND_FIXTURE).expect("write fixture");

        let report = mission_script_split(&path).expect("migration succeeds");
        let migrated = fs::read_to_string(&path).expect("read migrated");

        assert_eq!(report.moved_events, 1);
        assert!(report.written);
        assert!(migrated.contains("openbmp.scenario = 3"));
        assert!(migrated.contains("[[scenario_script.events]]"));
        assert!(!migrated.contains("[[mission.events]]"));
        Scenario::from_file(&path).expect("migrated scenario validates");
    }

    #[test]
    fn mission_script_split_preserves_transition_marker_event() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("scenario.toml");
        let fixture = ENGINE_COMMAND_FIXTURE
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            + r#"

[[mission.phases]]
id    = "done"
label = "done"

[[mission.transitions]]
from  = "ascent"
to    = "done"
event = "evt"
"#;
        fs::write(&path, fixture).expect("write fixture");

        let report = mission_script_split(&path).expect("migration succeeds");
        let migrated = fs::read_to_string(&path).expect("read migrated");

        assert_eq!(report.moved_events, 1);
        assert!(report.written);
        assert!(migrated.contains("[[mission.events]]"));
        assert!(migrated.contains("[[scenario_script.events]]"));
        assert!(migrated.contains(r#"kind = "emit_telemetry_marker""#));
        assert!(migrated.contains(r#"tag = "evt""#));
        assert!(migrated.contains(r#"event = "evt""#));
        Scenario::from_file(&path).expect("migrated scenario validates");
    }
}
