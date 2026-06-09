//! `openbmp dict ...` command/telemetry dictionary exports.

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_msgs::canonical_topic_descriptors;
use serde_json::json;

use crate::cli::DictFormat;
use crate::error::CliError;

/// Outcome of dictionary export.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DictExportReport {
    /// Output path, when written to a file.
    pub output: Option<PathBuf>,
    /// Number of topics exported.
    pub topics: usize,
    /// Format label.
    pub format: &'static str,
    /// Rendered dictionary content.
    pub content: String,
}

/// Export the canonical command/telemetry dictionary.
///
/// # Errors
///
/// Returns [`CliError`] when the requested format is not enabled or
/// the output file cannot be written.
pub fn export(
    format: DictFormat,
    output: Option<&Path>,
    experimental: bool,
) -> Result<DictExportReport, CliError> {
    let content = match format {
        DictFormat::Json => export_json()?,
        DictFormat::Xtce => {
            if !experimental {
                return Err(CliError::Dictionary {
                    summary: "XTCE export is experimental; rerun with --experimental".to_owned(),
                });
            }
            export_xtce()
        }
    };
    if let Some(path) = output {
        fs::write(path, &content).map_err(|source| CliError::Io {
            path: path.to_path_buf(),
            source,
        })?;
    }
    Ok(DictExportReport {
        output: output.map(Path::to_path_buf),
        topics: canonical_topic_descriptors().len(),
        format: match format {
            DictFormat::Json => "json",
            DictFormat::Xtce => "xtce",
        },
        content,
    })
}

fn export_json() -> Result<String, CliError> {
    let topics: Vec<_> = canonical_topic_descriptors()
        .iter()
        .map(|topic| {
            json!({
                "index": topic.index,
                "name": topic.name,
                "version": topic.version,
                "reserved": topic.reserved,
            })
        })
        .collect();
    let dictionary = json!({
        "schema": "openbmp.dictionary.v1",
        "standards_posture": {
            "xtce": "not claimed",
            "ccsds_pus": "not claimed"
        },
        "topics": topics,
    });
    serde_json::to_string_pretty(&dictionary).map_err(|source| CliError::DiffReportJson {
        path: PathBuf::from("<dictionary-json>"),
        source,
    })
}

fn export_xtce() -> String {
    let mut text = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
         <SpaceSystem name=\"OpenBMP\" schema=\"openbmp.dictionary.xtce.experimental\">\n\
         <TelemetryMetaData>\n\
         <ParameterTypeSet/>\n\
         <ParameterSet>\n",
    );
    for topic in canonical_topic_descriptors() {
        let reserved = if topic.reserved { "true" } else { "false" };
        text.push_str(&format!(
            "  <Parameter name=\"{}\" shortDescription=\"index={} version={} reserved={}\"/>\n",
            xml_escape(topic.name),
            topic.index,
            topic.version,
            reserved,
        ));
    }
    text.push_str(
        "</ParameterSet>\n\
         </TelemetryMetaData>\n\
         </SpaceSystem>\n",
    );
    text
}

fn xml_escape(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}
