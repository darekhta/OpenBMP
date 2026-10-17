//! Deterministic SVG plots, spike hunts, and Markdown report booklets.
//!
//! The render layer deliberately produces static artifacts with fixed layout,
//! fixed colors, and stable numeric formatting. It does not start a viewer or
//! service; callers opt in by passing telemetry tables and declared artifact
//! references.

use std::fmt::Write as _;

use openbmp_core::ChannelId;
use openbmp_telemetry::{TelemetryTable, TelemetryValue};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::require_non_empty;

/// Render/booklet artifact schema version.
pub const RENDER_SCHEMA_VERSION: &str = "openbmp.rundata.render.v1";

const MARGIN_LEFT: f64 = 48.0;
const MARGIN_RIGHT: f64 = 12.0;
const MARGIN_TOP: f64 = 24.0;
const MARGIN_BOTTOM: f64 = 34.0;

/// Errors returned by deterministic render helpers.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum RenderError {
    /// JSON serialization or parsing failed.
    #[error("render JSON error: {message}")]
    Json {
        /// Error message.
        message: String,
    },
    /// A required string field was empty after trimming.
    #[error("{field} must not be empty")]
    EmptyField {
        /// Field name.
        field: &'static str,
    },
    /// Plot dimensions were too small for the fixed layout.
    #[error("{field} is too small for a deterministic plot")]
    InvalidDimension {
        /// Field name.
        field: &'static str,
    },
    /// The requested telemetry channel was absent.
    #[error("unknown telemetry channel {channel}")]
    UnknownChannel {
        /// Channel name.
        channel: String,
    },
    /// A row did not contain the requested telemetry channel.
    #[error("missing sample for channel {channel} at row {row}")]
    MissingSample {
        /// Channel name.
        channel: String,
        /// Row index.
        row: usize,
    },
    /// A sample was not numeric.
    #[error("channel {channel} has non-numeric sample at row {row}")]
    NonNumericSample {
        /// Channel name.
        channel: String,
        /// Row index.
        row: usize,
    },
    /// A series or table had no usable samples.
    #[error("{field} must contain at least one sample")]
    EmptySamples {
        /// Field name.
        field: &'static str,
    },
    /// Two series did not share the same time grid.
    #[error("time grid mismatch at row {row}")]
    TimeGridMismatch {
        /// Row index.
        row: usize,
    },
    /// The spike-hunt request was malformed.
    #[error("top_k must be positive")]
    InvalidTopK,
}

impl From<serde_json::Error> for RenderError {
    fn from(error: serde_json::Error) -> Self {
        Self::Json {
            message: error.to_string(),
        }
    }
}

/// Fixed SVG plot metadata.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SvgPlotSpec {
    /// Plot title.
    pub title: String,
    /// X-axis label.
    pub x_label: String,
    /// Y-axis label.
    pub y_label: String,
    /// SVG width in pixels.
    pub width_px: u32,
    /// SVG height in pixels.
    pub height_px: u32,
}

impl SvgPlotSpec {
    /// Construct a validated SVG plot spec.
    ///
    /// # Errors
    ///
    /// Returns [`RenderError`] when text fields are empty or dimensions are
    /// too small for the fixed margins.
    pub fn new(
        title: impl Into<String>,
        x_label: impl Into<String>,
        y_label: impl Into<String>,
        width_px: u32,
        height_px: u32,
    ) -> Result<Self, RenderError> {
        let spec = Self {
            title: require_string("plot.title", title.into())?,
            x_label: require_string("plot.x_label", x_label.into())?,
            y_label: require_string("plot.y_label", y_label.into())?,
            width_px,
            height_px,
        };
        spec.validate()?;
        Ok(spec)
    }

    fn validate(&self) -> Result<(), RenderError> {
        if f64::from(self.width_px) <= MARGIN_LEFT + MARGIN_RIGHT {
            return Err(RenderError::InvalidDimension { field: "width_px" });
        }
        if f64::from(self.height_px) <= MARGIN_TOP + MARGIN_BOTTOM {
            return Err(RenderError::InvalidDimension { field: "height_px" });
        }
        Ok(())
    }
}

/// One time-series sample.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TimeSeriesPoint {
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Numeric channel value.
    pub value: f64,
}

/// One numeric time series.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TimeSeries {
    /// Stable series label.
    pub label: String,
    /// Samples in time order.
    pub points: Vec<TimeSeriesPoint>,
}

impl TimeSeries {
    /// Construct a validated time series.
    ///
    /// # Errors
    ///
    /// Returns [`RenderError`] when the label is empty or no points are
    /// provided.
    pub fn new(
        label: impl Into<String>,
        points: Vec<TimeSeriesPoint>,
    ) -> Result<Self, RenderError> {
        if points.is_empty() {
            return Err(RenderError::EmptySamples {
                field: "time_series.points",
            });
        }
        Ok(Self {
            label: require_string("time_series.label", label.into())?,
            points,
        })
    }
}

/// One percentile-fan sample.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct PercentileFanPoint {
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Low percentile value.
    pub low: f64,
    /// Median percentile value.
    pub median: f64,
    /// High percentile value.
    pub high: f64,
}

/// One run-diff sample.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RunDiffPoint {
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Left-hand run value.
    pub lhs: f64,
    /// Right-hand run value.
    pub rhs: f64,
    /// Difference `rhs - lhs`.
    pub residual: f64,
}

/// One spike-hunt finding.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SpikeFinding {
    /// One-based rank after deterministic sorting.
    pub rank: u64,
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Left-hand run value.
    pub lhs: f64,
    /// Right-hand run value.
    pub rhs: f64,
    /// Difference `rhs - lhs`.
    pub residual: f64,
    /// Absolute residual.
    pub abs_residual: f64,
}

/// Run-diff report with residual samples and top-k spikes.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RunDiffReport {
    /// Telemetry channel name.
    pub channel: String,
    /// Residual samples in time order.
    pub points: Vec<RunDiffPoint>,
    /// Top-k residual spikes in deterministic order.
    pub spikes: Vec<SpikeFinding>,
}

impl RunDiffReport {
    /// Serialize as deterministic JSON bytes.
    ///
    /// # Errors
    ///
    /// Returns [`RenderError::Json`] if serialization fails.
    pub fn to_json_bytes(&self) -> Result<Vec<u8>, RenderError> {
        Ok(serde_json::to_vec(self)?)
    }

    /// Parse a JSON run-diff report.
    ///
    /// # Errors
    ///
    /// Returns [`RenderError::Json`] if parsing fails.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, RenderError> {
        Ok(serde_json::from_slice(bytes)?)
    }
}

/// Plot artifact referenced by a report booklet.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BookletPlotRef {
    /// Plot title.
    pub title: String,
    /// Repository- or bundle-relative plot path.
    pub path: String,
    /// SHA-256 digest of the plot bytes.
    pub sha256_hex: String,
}

impl BookletPlotRef {
    /// Construct a validated plot reference.
    ///
    /// # Errors
    ///
    /// Returns [`RenderError`] when any field is empty.
    pub fn new(
        title: impl Into<String>,
        path: impl Into<String>,
        sha256_hex: impl Into<String>,
    ) -> Result<Self, RenderError> {
        Ok(Self {
            title: require_string("booklet.plot.title", title.into())?,
            path: require_string("booklet.plot.path", path.into())?,
            sha256_hex: require_string("booklet.plot.sha256_hex", sha256_hex.into())?,
        })
    }
}

/// Free-form Markdown section in a report booklet.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BookletSection {
    /// Section title.
    pub title: String,
    /// Section body, already deterministic Markdown.
    pub body: String,
}

impl BookletSection {
    /// Construct a validated booklet section.
    ///
    /// # Errors
    ///
    /// Returns [`RenderError`] when the title or body is empty.
    pub fn new(title: impl Into<String>, body: impl Into<String>) -> Result<Self, RenderError> {
        Ok(Self {
            title: require_string("booklet.section.title", title.into())?,
            body: require_string("booklet.section.body", body.into())?,
        })
    }
}

/// Deterministic campaign report booklet.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ReportBooklet {
    /// Booklet schema version.
    pub schema_version: String,
    /// Booklet title.
    pub title: String,
    /// Plot references in declared order.
    pub plots: Vec<BookletPlotRef>,
    /// Markdown sections in declared order.
    pub sections: Vec<BookletSection>,
}

impl ReportBooklet {
    /// Construct a validated report booklet.
    ///
    /// # Errors
    ///
    /// Returns [`RenderError`] when the title is empty.
    pub fn new(
        title: impl Into<String>,
        plots: Vec<BookletPlotRef>,
        sections: Vec<BookletSection>,
    ) -> Result<Self, RenderError> {
        Ok(Self {
            schema_version: RENDER_SCHEMA_VERSION.to_owned(),
            title: require_string("booklet.title", title.into())?,
            plots,
            sections,
        })
    }

    /// Serialize as deterministic JSON bytes.
    ///
    /// # Errors
    ///
    /// Returns [`RenderError::Json`] if serialization fails.
    pub fn to_json_bytes(&self) -> Result<Vec<u8>, RenderError> {
        Ok(serde_json::to_vec(self)?)
    }

    /// Parse a JSON booklet schema.
    ///
    /// # Errors
    ///
    /// Returns [`RenderError::Json`] if parsing fails.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, RenderError> {
        Ok(serde_json::from_slice(bytes)?)
    }
}

/// Extract a numeric telemetry channel as a time series.
///
/// # Errors
///
/// Returns [`RenderError`] when the channel is absent, any row is missing the
/// value, any sample is non-numeric, or the result is empty.
pub fn extract_numeric_time_series(
    table: &TelemetryTable,
    channel: &str,
    label: impl Into<String>,
) -> Result<TimeSeries, RenderError> {
    let channel_id = channel_id(table, channel)?;
    let mut points = Vec::with_capacity(table.rows().len());
    for (row_index, row) in table.rows().iter().enumerate() {
        let Some(value) = row.get(channel_id) else {
            return Err(RenderError::MissingSample {
                channel: channel.to_owned(),
                row: row_index,
            });
        };
        let Some(value) = numeric_value(value) else {
            return Err(RenderError::NonNumericSample {
                channel: channel.to_owned(),
                row: row_index,
            });
        };
        points.push(TimeSeriesPoint {
            time_s: row.time.as_seconds(),
            value,
        });
    }
    TimeSeries::new(label, points)
}

/// Render a deterministic SVG time-series plot.
///
/// # Errors
///
/// Returns [`RenderError`] when the spec or series is invalid.
pub fn render_time_series_svg(
    spec: &SvgPlotSpec,
    series: &[TimeSeries],
) -> Result<String, RenderError> {
    spec.validate()?;
    if series.is_empty() || series.iter().any(|series| series.points.is_empty()) {
        return Err(RenderError::EmptySamples {
            field: "time_series",
        });
    }
    let bounds = series_bounds(series.iter().flat_map(|series| series.points.iter()));
    let mut svg = svg_header(spec);
    write_axes(&mut svg, spec);
    for (index, series) in series.iter().enumerate() {
        let points = series
            .points
            .iter()
            .map(|point| scaled_point(spec, &bounds, point.time_s, point.value))
            .collect::<Vec<_>>();
        let _ = writeln!(
            svg,
            "<polyline fill=\"none\" stroke=\"{}\" stroke-width=\"2\" points=\"{}\"/>",
            color(index),
            point_list(&points)
        );
    }
    write_legend(&mut svg, series.iter().map(|series| series.label.as_str()));
    svg.push_str("</svg>\n");
    Ok(svg)
}

/// Compute percentile-fan samples from same-grid time series.
///
/// # Errors
///
/// Returns [`RenderError`] when no series are provided, a series is empty, or
/// time grids differ.
pub fn percentile_fan(series: &[TimeSeries]) -> Result<Vec<PercentileFanPoint>, RenderError> {
    if series.is_empty() || series.iter().any(|series| series.points.is_empty()) {
        return Err(RenderError::EmptySamples {
            field: "percentile_fan",
        });
    }
    let point_count = series[0].points.len();
    for candidate in &series[1..] {
        if candidate.points.len() != point_count {
            return Err(RenderError::TimeGridMismatch { row: point_count });
        }
    }

    let mut fan = Vec::with_capacity(point_count);
    for row in 0..point_count {
        let time_s = series[0].points[row].time_s;
        let mut values = Vec::with_capacity(series.len());
        for candidate in series {
            if candidate.points[row].time_s.to_bits() != time_s.to_bits() {
                return Err(RenderError::TimeGridMismatch { row });
            }
            values.push(candidate.points[row].value);
        }
        values.sort_by(f64::total_cmp);
        fan.push(PercentileFanPoint {
            time_s,
            low: percentile_sorted(&values, 0.05),
            median: percentile_sorted(&values, 0.5),
            high: percentile_sorted(&values, 0.95),
        });
    }
    Ok(fan)
}

/// Render a deterministic SVG percentile-fan plot.
///
/// # Errors
///
/// Returns [`RenderError`] when the spec or samples are invalid.
pub fn render_percentile_fan_svg(
    spec: &SvgPlotSpec,
    fan: &[PercentileFanPoint],
) -> Result<String, RenderError> {
    spec.validate()?;
    if fan.is_empty() {
        return Err(RenderError::EmptySamples {
            field: "percentile_fan",
        });
    }
    let bounds = fan_bounds(fan);
    let mut svg = svg_header(spec);
    write_axes(&mut svg, spec);
    let lower = fan
        .iter()
        .map(|point| scaled_point(spec, &bounds, point.time_s, point.low))
        .collect::<Vec<_>>();
    let upper = fan
        .iter()
        .rev()
        .map(|point| scaled_point(spec, &bounds, point.time_s, point.high))
        .collect::<Vec<_>>();
    let mut polygon = lower;
    polygon.extend(upper);
    let median = fan
        .iter()
        .map(|point| scaled_point(spec, &bounds, point.time_s, point.median))
        .collect::<Vec<_>>();
    let _ = writeln!(
        svg,
        "<polygon fill=\"#cfe8f3\" stroke=\"none\" points=\"{}\"/>",
        point_list(&polygon)
    );
    let _ = writeln!(
        svg,
        "<polyline fill=\"none\" stroke=\"#1b5e77\" stroke-width=\"2\" points=\"{}\"/>",
        point_list(&median)
    );
    write_legend(&mut svg, ["p05-p95", "p50"].into_iter());
    svg.push_str("</svg>\n");
    Ok(svg)
}

/// Compute a run-diff residual report and deterministic top-k spike hunt.
///
/// # Errors
///
/// Returns [`RenderError`] when inputs are invalid or the time grids differ.
pub fn run_diff_report(
    lhs: &TelemetryTable,
    rhs: &TelemetryTable,
    channel: &str,
    top_k: usize,
) -> Result<RunDiffReport, RenderError> {
    if top_k == 0 {
        return Err(RenderError::InvalidTopK);
    }
    let lhs = extract_numeric_time_series(lhs, channel, "lhs")?;
    let rhs = extract_numeric_time_series(rhs, channel, "rhs")?;
    if lhs.points.len() != rhs.points.len() {
        return Err(RenderError::TimeGridMismatch {
            row: lhs.points.len().min(rhs.points.len()),
        });
    }
    let mut points = Vec::with_capacity(lhs.points.len());
    for (row, (lhs_point, rhs_point)) in lhs.points.iter().zip(rhs.points.iter()).enumerate() {
        if lhs_point.time_s.to_bits() != rhs_point.time_s.to_bits() {
            return Err(RenderError::TimeGridMismatch { row });
        }
        points.push(RunDiffPoint {
            time_s: lhs_point.time_s,
            lhs: lhs_point.value,
            rhs: rhs_point.value,
            residual: rhs_point.value - lhs_point.value,
        });
    }
    if points.is_empty() {
        return Err(RenderError::EmptySamples {
            field: "run_diff.points",
        });
    }
    let spikes = spike_hunt(&points, top_k);
    Ok(RunDiffReport {
        channel: channel.to_owned(),
        points,
        spikes,
    })
}

/// Render a deterministic SVG residual plot from a run-diff report.
///
/// # Errors
///
/// Returns [`RenderError`] when the report is empty or the plot spec is
/// invalid.
pub fn render_run_diff_svg(
    spec: &SvgPlotSpec,
    report: &RunDiffReport,
) -> Result<String, RenderError> {
    let residual = TimeSeries::new(
        format!("{}.residual", report.channel),
        report
            .points
            .iter()
            .map(|point| TimeSeriesPoint {
                time_s: point.time_s,
                value: point.residual,
            })
            .collect(),
    )?;
    render_time_series_svg(spec, &[residual])
}

/// Render a deterministic Markdown report booklet.
#[must_use]
pub fn render_report_booklet(booklet: &ReportBooklet) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "# {}", booklet.title);
    let _ = writeln!(out);
    let _ = writeln!(out, "schema: {}", booklet.schema_version);
    let _ = writeln!(out);
    if !booklet.plots.is_empty() {
        let _ = writeln!(out, "## Plots");
        let _ = writeln!(out);
        let _ = writeln!(out, "| Title | Path | SHA-256 |");
        let _ = writeln!(out, "|---|---|---|");
        for plot in &booklet.plots {
            let _ = writeln!(
                out,
                "| {} | {} | {} |",
                markdown_cell(&plot.title),
                markdown_cell(&plot.path),
                markdown_cell(&plot.sha256_hex)
            );
        }
        let _ = writeln!(out);
    }
    for section in &booklet.sections {
        let _ = writeln!(out, "## {}", section.title);
        let _ = writeln!(out);
        let _ = writeln!(out, "{}", section.body.trim_end());
        let _ = writeln!(out);
    }
    out
}

fn spike_hunt(points: &[RunDiffPoint], top_k: usize) -> Vec<SpikeFinding> {
    let mut spikes = points
        .iter()
        .map(|point| SpikeFinding {
            rank: 0,
            time_s: point.time_s,
            lhs: point.lhs,
            rhs: point.rhs,
            residual: point.residual,
            abs_residual: point.residual.abs(),
        })
        .collect::<Vec<_>>();
    spikes.sort_by(|a, b| {
        b.abs_residual
            .total_cmp(&a.abs_residual)
            .then_with(|| a.time_s.total_cmp(&b.time_s))
            .then_with(|| a.residual.total_cmp(&b.residual))
    });
    spikes.truncate(top_k.min(spikes.len()));
    for (index, spike) in spikes.iter_mut().enumerate() {
        spike.rank = index as u64 + 1;
    }
    spikes
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PlotBounds {
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
}

fn series_bounds<'a>(points: impl Iterator<Item = &'a TimeSeriesPoint>) -> PlotBounds {
    let mut x_min = f64::INFINITY;
    let mut x_max = f64::NEG_INFINITY;
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;
    for point in points {
        x_min = x_min.min(point.time_s);
        x_max = x_max.max(point.time_s);
        y_min = y_min.min(point.value);
        y_max = y_max.max(point.value);
    }
    expand_bounds(x_min, x_max, y_min, y_max)
}

fn fan_bounds(points: &[PercentileFanPoint]) -> PlotBounds {
    let mut x_min = f64::INFINITY;
    let mut x_max = f64::NEG_INFINITY;
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;
    for point in points {
        x_min = x_min.min(point.time_s);
        x_max = x_max.max(point.time_s);
        y_min = y_min.min(point.low);
        y_max = y_max.max(point.high);
    }
    expand_bounds(x_min, x_max, y_min, y_max)
}

fn expand_bounds(mut x_min: f64, mut x_max: f64, mut y_min: f64, mut y_max: f64) -> PlotBounds {
    if x_min.to_bits() == x_max.to_bits() {
        x_min -= 1.0;
        x_max += 1.0;
    }
    if y_min.to_bits() == y_max.to_bits() {
        y_min -= 1.0;
        y_max += 1.0;
    }
    PlotBounds {
        x_min,
        x_max,
        y_min,
        y_max,
    }
}

fn scaled_point(spec: &SvgPlotSpec, bounds: &PlotBounds, x: f64, y: f64) -> (f64, f64) {
    let plot_width = f64::from(spec.width_px) - MARGIN_LEFT - MARGIN_RIGHT;
    let plot_height = f64::from(spec.height_px) - MARGIN_TOP - MARGIN_BOTTOM;
    let x_px = MARGIN_LEFT + (x - bounds.x_min) / (bounds.x_max - bounds.x_min) * plot_width;
    let y_px = MARGIN_TOP + (bounds.y_max - y) / (bounds.y_max - bounds.y_min) * plot_height;
    (x_px, y_px)
}

fn svg_header(spec: &SvgPlotSpec) -> String {
    let mut svg = String::new();
    let _ = writeln!(
        svg,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}\" height=\"{}\" viewBox=\"0 0 {} {}\">",
        spec.width_px, spec.height_px, spec.width_px, spec.height_px
    );
    let _ = writeln!(svg, "<title>{}</title>", xml_escape(&spec.title));
    let _ = writeln!(
        svg,
        "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{}\" y=\"16\" font-family=\"monospace\" font-size=\"12\" fill=\"#111111\">{}</text>",
        MARGIN_LEFT,
        xml_escape(&spec.title)
    );
    svg
}

fn write_axes(svg: &mut String, spec: &SvgPlotSpec) {
    let x0 = MARGIN_LEFT;
    let y0 = f64::from(spec.height_px) - MARGIN_BOTTOM;
    let x1 = f64::from(spec.width_px) - MARGIN_RIGHT;
    let y1 = MARGIN_TOP;
    let _ = writeln!(
        svg,
        "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#444444\" stroke-width=\"1\"/>",
        format_px(x0),
        format_px(y0),
        format_px(x1),
        format_px(y0)
    );
    let _ = writeln!(
        svg,
        "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#444444\" stroke-width=\"1\"/>",
        format_px(x0),
        format_px(y0),
        format_px(x0),
        format_px(y1)
    );
    let _ = writeln!(
        svg,
        "<text x=\"{}\" y=\"{}\" font-family=\"monospace\" font-size=\"10\" fill=\"#333333\">{}</text>",
        format_px((x0 + x1) * 0.5),
        format_px(f64::from(spec.height_px) - 8.0),
        xml_escape(&spec.x_label)
    );
    let _ = writeln!(
        svg,
        "<text x=\"4\" y=\"{}\" font-family=\"monospace\" font-size=\"10\" fill=\"#333333\">{}</text>",
        format_px(y1 - 6.0),
        xml_escape(&spec.y_label)
    );
}

fn write_legend<'a>(svg: &mut String, labels: impl Iterator<Item = &'a str>) {
    for (index, label) in labels.enumerate() {
        let y = MARGIN_TOP + 12.0 * index as f64;
        let _ = writeln!(
            svg,
            "<text x=\"{}\" y=\"{}\" font-family=\"monospace\" font-size=\"10\" fill=\"{}\">{}</text>",
            format_px(MARGIN_LEFT + 4.0),
            format_px(y + 10.0),
            color(index),
            xml_escape(label)
        );
    }
}

fn point_list(points: &[(f64, f64)]) -> String {
    points
        .iter()
        .map(|(x, y)| format!("{},{}", format_px(*x), format_px(*y)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn color(index: usize) -> &'static str {
    const COLORS: [&str; 5] = ["#1b5e77", "#b04a3a", "#3f7f3f", "#7a4ea3", "#9a6b21"];
    COLORS[index % COLORS.len()]
}

fn format_px(value: f64) -> String {
    format!("{value:.3}")
}

fn percentile_sorted(values: &[f64], percentile: f64) -> f64 {
    debug_assert!(!values.is_empty());
    if values.len() == 1 {
        return values[0];
    }
    let scaled = percentile * (values.len() - 1) as f64;
    let lower_index = scaled.floor() as usize;
    let upper_index = scaled.ceil() as usize;
    if lower_index == upper_index {
        values[lower_index]
    } else {
        let fraction = scaled - lower_index as f64;
        values[lower_index] * (1.0 - fraction) + values[upper_index] * fraction
    }
}

fn channel_id(table: &TelemetryTable, channel: &str) -> Result<ChannelId, RenderError> {
    table
        .schema()
        .channels()
        .iter()
        .find(|candidate| candidate.name == channel)
        .map(|candidate| candidate.id)
        .ok_or_else(|| RenderError::UnknownChannel {
            channel: channel.to_owned(),
        })
}

fn numeric_value(value: &TelemetryValue) -> Option<f64> {
    match value {
        TelemetryValue::Float64(value) => Some(*value),
        TelemetryValue::Int64(value) => Some(*value as f64),
        TelemetryValue::Bool(_) | TelemetryValue::Text(_) => None,
    }
}

fn require_string(field: &'static str, value: String) -> Result<String, RenderError> {
    require_non_empty(field, value).map_err(|_| RenderError::EmptyField { field })
}

fn markdown_cell(value: &str) -> String {
    value.replace('|', "\\|").replace('\n', " ")
}

fn xml_escape(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::{ChannelId, SimTime, StepIndex};
    use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema};

    fn table(values: &[f64]) -> TelemetryTable {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", None::<String>)
                .unwrap();
        let schema = TelemetrySchema::new(vec![altitude.metadata().clone()]).unwrap();
        let mut table = TelemetryTable::new(schema);
        for (index, value) in values.iter().enumerate() {
            let mut row = TelemetryRow::new(
                SimTime::from_seconds(index as f64),
                StepIndex::new(index as u64),
            )
            .unwrap();
            row.insert(&altitude, *value).unwrap();
            table.push_row(row).unwrap();
        }
        table
    }

    fn sparse_table() -> TelemetryTable {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", None::<String>)
                .unwrap();
        let schema = TelemetrySchema::new(vec![altitude.metadata().clone()]).unwrap();
        let mut table = TelemetryTable::new(schema);
        let row = TelemetryRow::new(SimTime::from_seconds(0.0), StepIndex::new(0)).unwrap();
        table.push_row(row).unwrap();
        table
    }

    fn bool_table() -> TelemetryTable {
        let valid =
            TelemetryChannel::<bool>::new(ChannelId::new(1), "valid", "1", None::<String>).unwrap();
        let schema = TelemetrySchema::new(vec![valid.metadata().clone()]).unwrap();
        let mut table = TelemetryTable::new(schema);
        let mut row = TelemetryRow::new(SimTime::from_seconds(0.0), StepIndex::new(0)).unwrap();
        row.insert(&valid, true).unwrap();
        table.push_row(row).unwrap();
        table
    }

    fn spec(title: &str) -> SvgPlotSpec {
        SvgPlotSpec::new(title, "time_s", "altitude_m", 200, 120).unwrap()
    }

    #[test]
    fn time_series_svg_is_byte_stable() {
        let series =
            extract_numeric_time_series(&table(&[0.0, 10.0, 0.0]), "altitude_m", "run-a").unwrap();
        let svg = render_time_series_svg(&spec("Altitude"), &[series]).unwrap();

        assert_eq!(
            svg,
            include_str!("../tests/fixtures/render-time-series.svg")
        );
    }

    #[test]
    fn percentile_fan_svg_is_byte_stable() {
        let series = [
            extract_numeric_time_series(&table(&[0.0, 10.0, 0.0]), "altitude_m", "a").unwrap(),
            extract_numeric_time_series(&table(&[1.0, 11.0, 1.0]), "altitude_m", "b").unwrap(),
            extract_numeric_time_series(&table(&[2.0, 12.0, 2.0]), "altitude_m", "c").unwrap(),
        ];
        let fan = percentile_fan(&series).unwrap();
        let svg = render_percentile_fan_svg(&spec("Altitude fan"), &fan).unwrap();

        assert_eq!(
            svg,
            include_str!("../tests/fixtures/render-percentile-fan.svg")
        );
    }

    #[test]
    fn run_diff_spike_hunt_is_exact_and_svg_stable() {
        let lhs = table(&[0.0, 2.0, 4.0, 6.0]);
        let rhs = table(&[0.0, 5.0, 1.0, 10.0]);
        let report = run_diff_report(&lhs, &rhs, "altitude_m", 2).unwrap();

        assert_eq!(report.spikes.len(), 2);
        assert_eq!(report.spikes[0].time_s, 3.0);
        assert_eq!(report.spikes[0].residual, 4.0);
        assert_eq!(report.spikes[1].time_s, 1.0);
        assert_eq!(report.spikes[1].residual, 3.0);
        let decoded = RunDiffReport::from_json_bytes(&report.to_json_bytes().unwrap()).unwrap();
        assert_eq!(decoded, report);

        let svg = render_run_diff_svg(&spec("Altitude residual"), &report).unwrap();
        assert_eq!(svg, include_str!("../tests/fixtures/render-run-diff.svg"));
    }

    #[test]
    fn report_booklet_markdown_is_byte_stable() {
        let booklet = ReportBooklet::new(
            "Drop campaign",
            vec![
                BookletPlotRef::new(
                    "Altitude",
                    "plots/altitude.svg",
                    "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                )
                .unwrap(),
            ],
            vec![
                BookletSection::new(
                    "Spike hunt",
                    "| rank | time_s | residual |\n|---:|---:|---:|\n| 1 | 3.000e0 | 4.000e0 |",
                )
                .unwrap(),
            ],
        )
        .unwrap();
        let decoded = ReportBooklet::from_json_bytes(&booklet.to_json_bytes().unwrap()).unwrap();

        assert_eq!(decoded, booklet);
        assert_eq!(
            render_report_booklet(&booklet),
            include_str!("../tests/fixtures/render-booklet.md")
        );
    }

    #[test]
    fn render_helpers_fail_closed_on_bad_inputs() {
        let bad_top_k =
            run_diff_report(&table(&[0.0]), &table(&[0.0]), "altitude_m", 0).unwrap_err();
        assert_eq!(bad_top_k, RenderError::InvalidTopK);

        let unknown =
            extract_numeric_time_series(&table(&[0.0]), "missing", "missing").unwrap_err();
        assert!(matches!(unknown, RenderError::UnknownChannel { .. }));

        let tiny = SvgPlotSpec::new("bad", "x", "y", 20, 20).unwrap_err();
        assert!(matches!(tiny, RenderError::InvalidDimension { .. }));
    }

    #[test]
    fn render_helpers_fail_closed_on_sparse_nonnumeric_empty_and_mismatched_inputs() {
        let sparse =
            extract_numeric_time_series(&sparse_table(), "altitude_m", "sparse").unwrap_err();
        assert!(matches!(sparse, RenderError::MissingSample { .. }));

        let nonnumeric = extract_numeric_time_series(&bool_table(), "valid", "valid").unwrap_err();
        assert!(matches!(nonnumeric, RenderError::NonNumericSample { .. }));

        let empty_series = TimeSeries::new("empty", Vec::new()).unwrap_err();
        assert!(matches!(empty_series, RenderError::EmptySamples { .. }));

        let empty_plot = render_time_series_svg(&spec("empty"), &[]).unwrap_err();
        assert!(matches!(empty_plot, RenderError::EmptySamples { .. }));

        let mismatch =
            run_diff_report(&table(&[0.0, 1.0]), &table(&[0.0]), "altitude_m", 1).unwrap_err();
        assert!(matches!(mismatch, RenderError::TimeGridMismatch { .. }));
    }
}
