//! Deterministic 3D/timeline export artifacts.
//!
//! This module is export-only: it creates static CZML-shaped and glTF-shaped
//! JSON artifacts from declared kinematic tracks, events, and ground sites. It
//! does not embed a renderer or depend on a hosted visualization service.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{base64_encode, require_non_empty};
use openbmp_core::ChannelId;
use openbmp_telemetry::{TelemetryRow, TelemetryTable, TelemetryValue};

/// 3D export artifact schema version.
pub const EXPORT3D_SCHEMA_VERSION: &str = "openbmp.rundata.export3d.v1";

/// Deterministic epoch used by generated relative-time artifacts.
pub const EXPORT3D_EPOCH_ISO8601: &str = "2000-01-01T00:00:00Z";

/// Errors returned by 3D export helpers.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum Export3dError {
    /// JSON serialization or parsing failed.
    #[error("3D export JSON error: {message}")]
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
    /// At least one body track is required.
    #[error("3D export requires at least one body track")]
    EmptyTracks,
    /// A body track had no samples.
    #[error("body track {body_id} must contain at least one sample")]
    EmptySamples {
        /// Body id.
        body_id: String,
    },
    /// A scalar or vector component was non-finite.
    #[error("{field} must be finite")]
    NonFinite {
        /// Field name.
        field: &'static str,
    },
    /// Track sample times must be strictly increasing after sorting.
    #[error("body track {body_id} has duplicate or decreasing sample time")]
    NonIncreasingTime {
        /// Body id.
        body_id: String,
    },
    /// Event interval bounds were invalid.
    #[error("event {id} interval is invalid")]
    InvalidInterval {
        /// Event id.
        id: String,
    },
    /// A generated document did not satisfy the local structural validator.
    #[error("invalid {format} export: {message}")]
    InvalidDocument {
        /// Export format.
        format: &'static str,
        /// Validation message.
        message: String,
    },
    /// A requested telemetry channel was absent from the archive schema.
    #[error("telemetry channel {channel} is not present")]
    UnknownChannel {
        /// Channel name.
        channel: String,
    },
    /// A requested telemetry channel did not have a value in a row.
    #[error("telemetry channel {channel} is missing from row {row}")]
    MissingSample {
        /// Channel name.
        channel: String,
        /// Row index.
        row: usize,
    },
    /// A requested telemetry channel carried a non-numeric value in a row.
    #[error("telemetry channel {channel} is non-numeric in row {row}")]
    NonNumericSample {
        /// Channel name.
        channel: String,
        /// Row index.
        row: usize,
    },
}

impl From<serde_json::Error> for Export3dError {
    fn from(error: serde_json::Error) -> Self {
        Self::Json {
            message: error.to_string(),
        }
    }
}

/// One kinematic sample for a body track.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct KinematicSample {
    /// Simulation time in seconds from [`EXPORT3D_EPOCH_ISO8601`].
    pub time_s: f64,
    /// Body origin position in ECI meters.
    pub position_eci_m: [f64; 3],
    /// Optional body-to-ECI attitude quaternion in `[x, y, z, w]` order.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quaternion_body_to_eci_xyzw: Option<[f64; 4]>,
}

/// One named body track.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BodyTrack {
    /// Stable body id.
    pub body_id: String,
    /// Kinematic samples in increasing time order.
    pub samples: Vec<KinematicSample>,
}

impl BodyTrack {
    /// Construct a validated body track.
    ///
    /// # Errors
    ///
    /// Returns [`Export3dError`] when the body id is empty, samples are empty,
    /// values are non-finite, or sample times are not unique.
    pub fn new(
        body_id: impl Into<String>,
        mut samples: Vec<KinematicSample>,
    ) -> Result<Self, Export3dError> {
        let body_id = require_string("body_id", body_id.into())?;
        samples.sort_by(|a, b| a.time_s.total_cmp(&b.time_s));
        let track = Self { body_id, samples };
        validate_track(&track)?;
        Ok(track)
    }
}

/// Telemetry channel mapping for one exported body track.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TelemetryBodyTrackSpec {
    /// Stable body id in the generated scene.
    pub body_id: String,
    /// Position channel names in ECI meters: x, y, z.
    pub position_channels: [String; 3],
    /// Optional body-to-ECI attitude quaternion channel names: x, y, z, w.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quaternion_channels: Option<[String; 4]>,
}

impl TelemetryBodyTrackSpec {
    /// Construct a validated telemetry body-track mapping.
    ///
    /// # Errors
    ///
    /// Returns [`Export3dError`] when the body id or any channel name is empty.
    pub fn new(
        body_id: impl Into<String>,
        position_channels: [String; 3],
        quaternion_channels: Option<[String; 4]>,
    ) -> Result<Self, Export3dError> {
        let [position_x, position_y, position_z] = position_channels;
        let quaternion_channels = match quaternion_channels {
            Some([quaternion_x, quaternion_y, quaternion_z, quaternion_w]) => Some([
                require_string("quaternion_channels[0]", quaternion_x)?,
                require_string("quaternion_channels[1]", quaternion_y)?,
                require_string("quaternion_channels[2]", quaternion_z)?,
                require_string("quaternion_channels[3]", quaternion_w)?,
            ]),
            None => None,
        };
        Ok(Self {
            body_id: require_string("body_id", body_id.into())?,
            position_channels: [
                require_string("position_channels[0]", position_x)?,
                require_string("position_channels[1]", position_y)?,
                require_string("position_channels[2]", position_z)?,
            ],
            quaternion_channels,
        })
    }

    /// Mapping for the runner's primary rigid-body telemetry channels.
    ///
    /// # Errors
    ///
    /// Returns [`Export3dError`] when the body id is empty.
    pub fn primary(body_id: impl Into<String>) -> Result<Self, Export3dError> {
        Self::new(
            body_id,
            [
                "position_x_m".to_owned(),
                "position_y_m".to_owned(),
                "position_z_m".to_owned(),
            ],
            Some([
                "attitude.q_x".to_owned(),
                "attitude.q_y".to_owned(),
                "attitude.q_z".to_owned(),
                "attitude.q_w".to_owned(),
            ]),
        )
    }

    /// Mapping for the runner's `body.<id>.*` separated-body telemetry channels.
    ///
    /// # Errors
    ///
    /// Returns [`Export3dError`] when the body id is empty.
    pub fn separated(body_id: impl Into<String>) -> Result<Self, Export3dError> {
        let body_id = body_id.into();
        let prefix = format!("body.{body_id}.");
        Self::new(
            body_id,
            [
                format!("{prefix}position_x_m"),
                format!("{prefix}position_y_m"),
                format!("{prefix}position_z_m"),
            ],
            Some([
                format!("{prefix}attitude.q_x"),
                format!("{prefix}attitude.q_y"),
                format!("{prefix}attitude.q_z"),
                format!("{prefix}attitude.q_w"),
            ]),
        )
    }
}

/// Timeline interval exported as an event packet.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct EventInterval {
    /// Stable event id.
    pub id: String,
    /// Human-readable label.
    pub label: String,
    /// Interval start in seconds from epoch.
    pub start_s: f64,
    /// Interval stop in seconds from epoch.
    pub stop_s: f64,
}

impl EventInterval {
    /// Construct a validated event interval.
    ///
    /// # Errors
    ///
    /// Returns [`Export3dError`] when text fields are empty or bounds are
    /// invalid.
    pub fn new(
        id: impl Into<String>,
        label: impl Into<String>,
        start_s: f64,
        stop_s: f64,
    ) -> Result<Self, Export3dError> {
        let event = Self {
            id: require_string("event.id", id.into())?,
            label: require_string("event.label", label.into())?,
            start_s,
            stop_s,
        };
        validate_event(&event)?;
        Ok(event)
    }
}

/// Ground site marker for CZML exports.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct GroundSite {
    /// Stable site id.
    pub id: String,
    /// Geodetic longitude in degrees.
    pub longitude_deg: f64,
    /// Geodetic latitude in degrees.
    pub latitude_deg: f64,
    /// Height above ellipsoid in meters.
    pub height_m: f64,
}

impl GroundSite {
    /// Construct a validated ground site.
    ///
    /// # Errors
    ///
    /// Returns [`Export3dError`] when the id is empty or coordinates are
    /// non-finite.
    pub fn new(
        id: impl Into<String>,
        longitude_deg: f64,
        latitude_deg: f64,
        height_m: f64,
    ) -> Result<Self, Export3dError> {
        let site = Self {
            id: require_string("site.id", id.into())?,
            longitude_deg,
            latitude_deg,
            height_m,
        };
        validate_site(&site)?;
        Ok(site)
    }
}

/// Exportable 3D/timeline scene.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SceneExport {
    /// Artifact schema version.
    pub schema_version: String,
    /// Relative-time epoch.
    pub epoch: String,
    /// Body tracks in deterministic body-id order.
    pub tracks: Vec<BodyTrack>,
    /// Event intervals in deterministic id order.
    pub events: Vec<EventInterval>,
    /// Ground sites in deterministic id order.
    pub ground_sites: Vec<GroundSite>,
}

impl SceneExport {
    /// Construct a validated scene export.
    ///
    /// # Errors
    ///
    /// Returns [`Export3dError`] when no tracks are provided or any child item
    /// is invalid.
    pub fn new(
        mut tracks: Vec<BodyTrack>,
        mut events: Vec<EventInterval>,
        mut ground_sites: Vec<GroundSite>,
    ) -> Result<Self, Export3dError> {
        tracks.sort_by(|a, b| a.body_id.cmp(&b.body_id));
        events.sort_by(|a, b| a.id.cmp(&b.id));
        ground_sites.sort_by(|a, b| a.id.cmp(&b.id));
        let scene = Self {
            schema_version: EXPORT3D_SCHEMA_VERSION.to_owned(),
            epoch: EXPORT3D_EPOCH_ISO8601.to_owned(),
            tracks,
            events,
            ground_sites,
        };
        validate_scene(&scene)?;
        Ok(scene)
    }
}

/// Build a validated scene from telemetry table channels.
///
/// # Errors
///
/// Returns [`Export3dError`] when a requested channel is absent, a row is
/// missing a requested sample, a requested sample is non-numeric, or the
/// resulting scene is invalid.
pub fn scene_export_from_telemetry_table(
    table: &TelemetryTable,
    track_specs: &[TelemetryBodyTrackSpec],
    events: Vec<EventInterval>,
    ground_sites: Vec<GroundSite>,
) -> Result<SceneExport, Export3dError> {
    let tracks = track_specs
        .iter()
        .map(|spec| body_track_from_telemetry_table(table, spec))
        .collect::<Result<Vec<_>, _>>()?;
    SceneExport::new(tracks, events, ground_sites)
}

/// Generate deterministic CZML-shaped JSON bytes.
///
/// # Errors
///
/// Returns [`Export3dError`] when the scene is invalid or serialization fails.
pub fn generate_czml(scene: &SceneExport) -> Result<Vec<u8>, Export3dError> {
    validate_scene(scene)?;
    let mut packets = Vec::new();
    packets.push(CzmlPacket::Document {
        id: "document".to_owned(),
        version: "1.0".to_owned(),
    });
    for track in &scene.tracks {
        packets.push(CzmlPacket::Body {
            id: format!("body/{}", track.body_id),
            name: track.body_id.clone(),
            position: CzmlPosition {
                reference_frame: "INERTIAL".to_owned(),
                epoch: scene.epoch.clone(),
                cartesian: cartesian_samples(track),
            },
            orientation: orientation_samples(track).map(|unit_quaternion| CzmlOrientation {
                epoch: scene.epoch.clone(),
                unit_quaternion,
            }),
        });
    }
    for event in &scene.events {
        packets.push(CzmlPacket::Event {
            id: format!("event/{}", event.id),
            name: event.label.clone(),
            availability: format!("{}/{}", format_s(event.start_s), format_s(event.stop_s)),
        });
    }
    for site in &scene.ground_sites {
        packets.push(CzmlPacket::GroundSite {
            id: format!("site/{}", site.id),
            name: site.id.clone(),
            position: CzmlSitePosition {
                cartographic_degrees: [site.longitude_deg, site.latitude_deg, site.height_m],
            },
        });
    }
    Ok(serde_json::to_vec(&packets)?)
}

/// Generate deterministic glTF 2.0 JSON bytes with embedded animation buffers
/// and OpenBMP track extras.
///
/// # Errors
///
/// Returns [`Export3dError`] when the scene is invalid or serialization fails.
pub fn generate_gltf_json(scene: &SceneExport) -> Result<Vec<u8>, Export3dError> {
    validate_scene(scene)?;
    let nodes = scene
        .tracks
        .iter()
        .map(|track| GltfNode {
            name: track.body_id.clone(),
        })
        .collect::<Vec<_>>();
    let scene_nodes = (0..scene.tracks.len() as u64).collect::<Vec<_>>();
    let animation_data = gltf_animation_data(&scene.tracks);
    let document = GltfDocument {
        asset: GltfAsset {
            version: "2.0".to_owned(),
            generator: "openbmp-rundata".to_owned(),
        },
        scene: 0,
        scenes: vec![GltfScene { nodes: scene_nodes }],
        nodes,
        buffers: vec![GltfBuffer {
            byte_length: animation_data.buffer.len() as u64,
            uri: format!(
                "data:application/octet-stream;base64,{}",
                base64_encode(&animation_data.buffer)
            ),
        }],
        buffer_views: animation_data.buffer_views,
        accessors: animation_data.accessors,
        animations: vec![GltfAnimation {
            name: "OpenBMP body tracks".to_owned(),
            samplers: animation_data.samplers,
            channels: animation_data.channels,
        }],
        extras: GltfExtras {
            openbmp_schema_version: EXPORT3D_SCHEMA_VERSION.to_owned(),
            epoch: scene.epoch.clone(),
            tracks: scene.tracks.clone(),
            events: scene.events.clone(),
        },
    };
    Ok(serde_json::to_vec(&document)?)
}

/// Validate generated CZML-shaped JSON bytes against the local structural
/// contract.
///
/// # Errors
///
/// Returns [`Export3dError::InvalidDocument`] when required document/body
/// packets are absent or malformed.
pub fn validate_czml_export(bytes: &[u8]) -> Result<(), Export3dError> {
    let value: serde_json::Value = serde_json::from_slice(bytes)?;
    let Some(array) = value.as_array() else {
        return invalid_document("czml", "top level must be an array");
    };
    if array.is_empty() {
        return invalid_document("czml", "top level array must not be empty");
    }
    if array
        .first()
        .and_then(|packet| packet.get("id"))
        .and_then(serde_json::Value::as_str)
        != Some("document")
    {
        return invalid_document("czml", "first packet must be document");
    }
    if !array.iter().any(|packet| {
        packet
            .get("id")
            .and_then(serde_json::Value::as_str)
            .is_some_and(|id| id.starts_with("body/"))
            && packet.get("position").is_some()
    }) {
        return invalid_document("czml", "at least one body packet is required");
    }
    Ok(())
}

/// Validate generated glTF-shaped JSON bytes against the local structural
/// contract.
///
/// # Errors
///
/// Returns [`Export3dError::InvalidDocument`] when required glTF shell fields
/// or OpenBMP track extras are absent.
pub fn validate_gltf_export(bytes: &[u8]) -> Result<(), Export3dError> {
    let value: serde_json::Value = serde_json::from_slice(bytes)?;
    if value
        .get("asset")
        .and_then(|asset| asset.get("version"))
        .and_then(serde_json::Value::as_str)
        != Some("2.0")
    {
        return invalid_document("gltf", "asset.version must be 2.0");
    }
    let track_count = value
        .get("extras")
        .and_then(|extras| extras.get("tracks"))
        .and_then(serde_json::Value::as_array)
        .map_or(0, Vec::len);
    let node_count = value
        .get("nodes")
        .and_then(serde_json::Value::as_array)
        .map_or(0, Vec::len);
    if track_count == 0 || track_count != node_count {
        return invalid_document("gltf", "tracks and nodes must be non-empty and aligned");
    }
    if value
        .get("buffers")
        .and_then(serde_json::Value::as_array)
        .is_none_or(Vec::is_empty)
    {
        return invalid_document("gltf", "at least one buffer is required");
    }
    let accessor_count = value
        .get("accessors")
        .and_then(serde_json::Value::as_array)
        .map_or(0, Vec::len);
    let sampler_count = value
        .get("animations")
        .and_then(serde_json::Value::as_array)
        .and_then(|animations| animations.first())
        .and_then(|animation| animation.get("samplers"))
        .and_then(serde_json::Value::as_array)
        .map_or(0, Vec::len);
    let channel_count = value
        .get("animations")
        .and_then(serde_json::Value::as_array)
        .and_then(|animations| animations.first())
        .and_then(|animation| animation.get("channels"))
        .and_then(serde_json::Value::as_array)
        .map_or(0, Vec::len);
    if accessor_count == 0 || sampler_count == 0 || channel_count == 0 {
        return invalid_document(
            "gltf",
            "animation accessors, samplers, and channels are required",
        );
    }
    Ok(())
}

#[derive(Clone, Debug, Serialize)]
#[serde(tag = "packet_type", rename_all = "snake_case")]
enum CzmlPacket {
    Document {
        id: String,
        version: String,
    },
    Body {
        id: String,
        name: String,
        position: CzmlPosition,
        #[serde(skip_serializing_if = "Option::is_none")]
        orientation: Option<CzmlOrientation>,
    },
    Event {
        id: String,
        name: String,
        availability: String,
    },
    GroundSite {
        id: String,
        name: String,
        position: CzmlSitePosition,
    },
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct CzmlPosition {
    reference_frame: String,
    epoch: String,
    cartesian: Vec<f64>,
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct CzmlOrientation {
    epoch: String,
    unit_quaternion: Vec<f64>,
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct CzmlSitePosition {
    cartographic_degrees: [f64; 3],
}

#[derive(Clone, Debug, Serialize)]
struct GltfDocument {
    asset: GltfAsset,
    scene: u64,
    scenes: Vec<GltfScene>,
    nodes: Vec<GltfNode>,
    buffers: Vec<GltfBuffer>,
    #[serde(rename = "bufferViews")]
    buffer_views: Vec<GltfBufferView>,
    accessors: Vec<GltfAccessor>,
    animations: Vec<GltfAnimation>,
    extras: GltfExtras,
}

#[derive(Clone, Debug, Serialize)]
struct GltfAsset {
    version: String,
    generator: String,
}

#[derive(Clone, Debug, Serialize)]
struct GltfScene {
    nodes: Vec<u64>,
}

#[derive(Clone, Debug, Serialize)]
struct GltfNode {
    name: String,
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct GltfBuffer {
    byte_length: u64,
    uri: String,
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct GltfBufferView {
    buffer: u64,
    byte_offset: u64,
    byte_length: u64,
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct GltfAccessor {
    buffer_view: u64,
    component_type: u64,
    count: u64,
    #[serde(rename = "type")]
    type_name: &'static str,
    min: Vec<f32>,
    max: Vec<f32>,
}

#[derive(Clone, Debug, Serialize)]
struct GltfAnimation {
    name: String,
    samplers: Vec<GltfAnimationSampler>,
    channels: Vec<GltfAnimationChannel>,
}

#[derive(Clone, Debug, Serialize)]
struct GltfAnimationSampler {
    input: u64,
    output: u64,
    interpolation: &'static str,
}

#[derive(Clone, Debug, Serialize)]
struct GltfAnimationChannel {
    sampler: u64,
    target: GltfAnimationTarget,
}

#[derive(Clone, Debug, Serialize)]
struct GltfAnimationTarget {
    node: u64,
    path: &'static str,
}

#[derive(Clone, Debug, Serialize)]
struct GltfExtras {
    openbmp_schema_version: String,
    epoch: String,
    tracks: Vec<BodyTrack>,
    events: Vec<EventInterval>,
}

struct GltfAnimationData {
    buffer: Vec<u8>,
    buffer_views: Vec<GltfBufferView>,
    accessors: Vec<GltfAccessor>,
    samplers: Vec<GltfAnimationSampler>,
    channels: Vec<GltfAnimationChannel>,
}

fn gltf_animation_data(tracks: &[BodyTrack]) -> GltfAnimationData {
    let mut data = GltfAnimationData {
        buffer: Vec::new(),
        buffer_views: Vec::new(),
        accessors: Vec::new(),
        samplers: Vec::new(),
        channels: Vec::new(),
    };
    for (node_index, track) in tracks.iter().enumerate() {
        let time_values = track
            .samples
            .iter()
            .map(|sample| sample.time_s as f32)
            .collect::<Vec<_>>();
        let translation_values = track
            .samples
            .iter()
            .flat_map(|sample| sample.position_eci_m.map(|value| value as f32))
            .collect::<Vec<_>>();
        let time_accessor = push_accessor(&mut data, &time_values, track.samples.len(), "SCALAR");
        let translation_accessor =
            push_accessor(&mut data, &translation_values, track.samples.len(), "VEC3");
        push_animation_channel(
            &mut data,
            node_index as u64,
            time_accessor,
            translation_accessor,
            "translation",
        );
        if let Some(rotation_values) = gltf_rotation_values(track) {
            let rotation_accessor =
                push_accessor(&mut data, &rotation_values, track.samples.len(), "VEC4");
            push_animation_channel(
                &mut data,
                node_index as u64,
                time_accessor,
                rotation_accessor,
                "rotation",
            );
        }
    }
    data
}

fn gltf_rotation_values(track: &BodyTrack) -> Option<Vec<f32>> {
    let mut values = Vec::with_capacity(track.samples.len() * 4);
    for sample in &track.samples {
        values.extend(
            sample
                .quaternion_body_to_eci_xyzw?
                .map(|value| value as f32),
        );
    }
    Some(values)
}

fn push_accessor(
    data: &mut GltfAnimationData,
    values: &[f32],
    count: usize,
    type_name: &'static str,
) -> u64 {
    let buffer_view = push_buffer_view(data, values);
    let component_count = match type_name {
        "SCALAR" => 1,
        "VEC3" => 3,
        "VEC4" => 4,
        _ => 1,
    };
    let mut min = vec![f32::INFINITY; component_count];
    let mut max = vec![f32::NEG_INFINITY; component_count];
    for chunk in values.chunks_exact(component_count) {
        for (index, value) in chunk.iter().enumerate() {
            min[index] = min[index].min(*value);
            max[index] = max[index].max(*value);
        }
    }
    let accessor = data.accessors.len() as u64;
    data.accessors.push(GltfAccessor {
        buffer_view,
        component_type: 5126,
        count: count as u64,
        type_name,
        min,
        max,
    });
    accessor
}

fn push_buffer_view(data: &mut GltfAnimationData, values: &[f32]) -> u64 {
    align4(&mut data.buffer);
    let byte_offset = data.buffer.len() as u64;
    for value in values {
        data.buffer.extend_from_slice(&value.to_le_bytes());
    }
    let byte_length = data.buffer.len() as u64 - byte_offset;
    let buffer_view = data.buffer_views.len() as u64;
    data.buffer_views.push(GltfBufferView {
        buffer: 0,
        byte_offset,
        byte_length,
    });
    buffer_view
}

fn push_animation_channel(
    data: &mut GltfAnimationData,
    node: u64,
    input: u64,
    output: u64,
    path: &'static str,
) {
    let sampler = data.samplers.len() as u64;
    data.samplers.push(GltfAnimationSampler {
        input,
        output,
        interpolation: "LINEAR",
    });
    data.channels.push(GltfAnimationChannel {
        sampler,
        target: GltfAnimationTarget { node, path },
    });
}

fn body_track_from_telemetry_table(
    table: &TelemetryTable,
    spec: &TelemetryBodyTrackSpec,
) -> Result<BodyTrack, Export3dError> {
    let position_ids = [
        resolve_channel(table, &spec.position_channels[0])?,
        resolve_channel(table, &spec.position_channels[1])?,
        resolve_channel(table, &spec.position_channels[2])?,
    ];
    let quaternion_ids = match &spec.quaternion_channels {
        Some(channels) => Some([
            resolve_channel(table, &channels[0])?,
            resolve_channel(table, &channels[1])?,
            resolve_channel(table, &channels[2])?,
            resolve_channel(table, &channels[3])?,
        ]),
        None => None,
    };
    let mut samples = Vec::with_capacity(table.rows().len());
    for (row_index, row) in table.rows().iter().enumerate() {
        let position_eci_m = [
            row_numeric_value(row, position_ids[0], &spec.position_channels[0], row_index)?,
            row_numeric_value(row, position_ids[1], &spec.position_channels[1], row_index)?,
            row_numeric_value(row, position_ids[2], &spec.position_channels[2], row_index)?,
        ];
        let quaternion_body_to_eci_xyzw = match (&spec.quaternion_channels, quaternion_ids) {
            (Some(channels), Some(ids)) => Some([
                row_numeric_value(row, ids[0], &channels[0], row_index)?,
                row_numeric_value(row, ids[1], &channels[1], row_index)?,
                row_numeric_value(row, ids[2], &channels[2], row_index)?,
                row_numeric_value(row, ids[3], &channels[3], row_index)?,
            ]),
            _ => None,
        };
        samples.push(KinematicSample {
            time_s: row.time.as_seconds(),
            position_eci_m,
            quaternion_body_to_eci_xyzw,
        });
    }
    BodyTrack::new(spec.body_id.clone(), samples)
}

fn resolve_channel(table: &TelemetryTable, channel: &str) -> Result<ChannelId, Export3dError> {
    table
        .schema()
        .channels()
        .iter()
        .find(|metadata| metadata.name == channel)
        .map(|metadata| metadata.id)
        .ok_or_else(|| Export3dError::UnknownChannel {
            channel: channel.to_owned(),
        })
}

fn row_numeric_value(
    row: &TelemetryRow,
    channel_id: ChannelId,
    channel: &str,
    row_index: usize,
) -> Result<f64, Export3dError> {
    match row.get(channel_id) {
        Some(TelemetryValue::Float64(value)) => Ok(*value),
        Some(TelemetryValue::Int64(value)) => Ok(*value as f64),
        Some(TelemetryValue::Bool(_) | TelemetryValue::Text(_)) => {
            Err(Export3dError::NonNumericSample {
                channel: channel.to_owned(),
                row: row_index,
            })
        }
        None => Err(Export3dError::MissingSample {
            channel: channel.to_owned(),
            row: row_index,
        }),
    }
}

fn align4(bytes: &mut Vec<u8>) {
    while !bytes.len().is_multiple_of(4) {
        bytes.push(0);
    }
}

fn cartesian_samples(track: &BodyTrack) -> Vec<f64> {
    let mut values = Vec::with_capacity(track.samples.len() * 4);
    for sample in &track.samples {
        values.push(sample.time_s);
        values.extend(sample.position_eci_m);
    }
    values
}

fn orientation_samples(track: &BodyTrack) -> Option<Vec<f64>> {
    let mut values = Vec::with_capacity(track.samples.len() * 5);
    for sample in &track.samples {
        let quaternion = sample.quaternion_body_to_eci_xyzw?;
        values.push(sample.time_s);
        values.extend(quaternion);
    }
    Some(values)
}

fn validate_scene(scene: &SceneExport) -> Result<(), Export3dError> {
    if scene.tracks.is_empty() {
        return Err(Export3dError::EmptyTracks);
    }
    require_string("scene.schema_version", scene.schema_version.clone())?;
    require_string("scene.epoch", scene.epoch.clone())?;
    for track in &scene.tracks {
        validate_track(track)?;
    }
    for event in &scene.events {
        validate_event(event)?;
    }
    for site in &scene.ground_sites {
        validate_site(site)?;
    }
    Ok(())
}

fn validate_track(track: &BodyTrack) -> Result<(), Export3dError> {
    require_string("body_id", track.body_id.clone())?;
    if track.samples.is_empty() {
        return Err(Export3dError::EmptySamples {
            body_id: track.body_id.clone(),
        });
    }
    let mut previous = None;
    for sample in &track.samples {
        finite("sample.time_s", sample.time_s)?;
        if previous.is_some_and(|previous| sample.time_s <= previous) {
            return Err(Export3dError::NonIncreasingTime {
                body_id: track.body_id.clone(),
            });
        }
        previous = Some(sample.time_s);
        for value in sample.position_eci_m {
            finite("sample.position_eci_m", value)?;
        }
        if let Some(quaternion) = sample.quaternion_body_to_eci_xyzw {
            for value in quaternion {
                finite("sample.quaternion_body_to_eci_xyzw", value)?;
            }
        }
    }
    Ok(())
}

fn validate_event(event: &EventInterval) -> Result<(), Export3dError> {
    require_string("event.id", event.id.clone())?;
    require_string("event.label", event.label.clone())?;
    if !event.start_s.is_finite() || !event.stop_s.is_finite() || event.start_s > event.stop_s {
        return Err(Export3dError::InvalidInterval {
            id: event.id.clone(),
        });
    }
    Ok(())
}

fn validate_site(site: &GroundSite) -> Result<(), Export3dError> {
    require_string("site.id", site.id.clone())?;
    finite("site.longitude_deg", site.longitude_deg)?;
    finite("site.latitude_deg", site.latitude_deg)?;
    finite("site.height_m", site.height_m)
}

fn finite(field: &'static str, value: f64) -> Result<(), Export3dError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(Export3dError::NonFinite { field })
    }
}

fn require_string(field: &'static str, value: String) -> Result<String, Export3dError> {
    require_non_empty(field, value).map_err(|_| Export3dError::EmptyField { field })
}

fn invalid_document<T>(format: &'static str, message: &str) -> Result<T, Export3dError> {
    Err(Export3dError::InvalidDocument {
        format,
        message: message.to_owned(),
    })
}

fn format_s(value: f64) -> String {
    format!("{value:.17e}")
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::{ChannelId, SimTime, StepIndex};
    use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};

    fn sample(time_s: f64, position_eci_m: [f64; 3]) -> KinematicSample {
        KinematicSample {
            time_s,
            position_eci_m,
            quaternion_body_to_eci_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
        }
    }

    fn scene() -> SceneExport {
        SceneExport::new(
            vec![
                BodyTrack::new(
                    "vehicle",
                    vec![sample(0.0, [1.0, 2.0, 3.0]), sample(1.0, [4.0, 5.0, 6.0])],
                )
                .unwrap(),
            ],
            vec![EventInterval::new("burn", "Main burn", 0.0, 1.0).unwrap()],
            vec![GroundSite::new("ksc", -80.604333, 28.608389, 3.0).unwrap()],
        )
        .unwrap()
    }

    fn f64_channel(id: u64, name: &str, unit: &str, frame: Option<&str>) -> TelemetryChannel<f64> {
        TelemetryChannel::<f64>::new(ChannelId::new(id), name, unit, frame).unwrap()
    }

    fn bool_channel(id: u64, name: &str) -> TelemetryChannel<bool> {
        TelemetryChannel::<bool>::new(ChannelId::new(id), name, "bool", None::<&str>).unwrap()
    }

    fn push_fixture_row(
        table: &mut TelemetryTable,
        channels: [&TelemetryChannel<f64>; 14],
        step: u64,
        time_s: f64,
        primary_position: [f64; 3],
        observer_position: [f64; 3],
    ) {
        let mut row = TelemetryRow::new(SimTime::from_seconds(time_s), StepIndex::new(step))
            .expect("row must be valid");
        let values = [
            primary_position[0],
            primary_position[1],
            primary_position[2],
            0.0,
            0.0,
            0.0,
            1.0,
            observer_position[0],
            observer_position[1],
            observer_position[2],
            0.0,
            0.0,
            0.0,
            1.0,
        ];
        for (channel, value) in channels.into_iter().zip(values) {
            row.insert(channel, value).unwrap();
        }
        table.push_row(row).unwrap();
    }

    fn telemetry_fixture() -> TelemetryTable {
        let primary_x = f64_channel(1, "position_x_m", "m", Some("ECI"));
        let primary_y = f64_channel(2, "position_y_m", "m", Some("ECI"));
        let primary_z = f64_channel(3, "position_z_m", "m", Some("ECI"));
        let primary_qx = f64_channel(4, "attitude.q_x", "1", None);
        let primary_qy = f64_channel(5, "attitude.q_y", "1", None);
        let primary_qz = f64_channel(6, "attitude.q_z", "1", None);
        let primary_qw = f64_channel(7, "attitude.q_w", "1", None);
        let observer_x = f64_channel(8, "body.observer.position_x_m", "m", Some("ECI"));
        let observer_y = f64_channel(9, "body.observer.position_y_m", "m", Some("ECI"));
        let observer_z = f64_channel(10, "body.observer.position_z_m", "m", Some("ECI"));
        let observer_qx = f64_channel(11, "body.observer.attitude.q_x", "1", None);
        let observer_qy = f64_channel(12, "body.observer.attitude.q_y", "1", None);
        let observer_qz = f64_channel(13, "body.observer.attitude.q_z", "1", None);
        let observer_qw = f64_channel(14, "body.observer.attitude.q_w", "1", None);
        let channels = [
            &primary_x,
            &primary_y,
            &primary_z,
            &primary_qx,
            &primary_qy,
            &primary_qz,
            &primary_qw,
            &observer_x,
            &observer_y,
            &observer_z,
            &observer_qx,
            &observer_qy,
            &observer_qz,
            &observer_qw,
        ];
        let schema = TelemetrySchema::new(
            channels
                .iter()
                .map(|channel| channel.metadata().clone())
                .collect(),
        )
        .unwrap();
        let mut table = TelemetryTable::new(schema);
        push_fixture_row(
            &mut table,
            channels,
            0,
            0.0,
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 100.0],
        );
        push_fixture_row(
            &mut table,
            channels,
            1,
            0.1,
            [0.0, 0.0, 0.0],
            [1.0, 0.1, 100.0],
        );
        table
    }

    #[test]
    fn czml_export_is_byte_stable_and_structurally_valid() {
        let bytes = generate_czml(&scene()).unwrap();
        let golden = include_str!("../tests/fixtures/export3d.czml.json");
        let golden = golden.strip_suffix('\n').unwrap_or(golden);

        assert_eq!(std::str::from_utf8(&bytes).unwrap(), golden);
        validate_czml_export(&bytes).unwrap();
    }

    #[test]
    fn gltf_export_is_byte_stable_and_structurally_valid() {
        let bytes = generate_gltf_json(&scene()).unwrap();
        let golden = include_str!("../tests/fixtures/export3d.gltf.json");
        let golden = golden.strip_suffix('\n').unwrap_or(golden);

        assert_eq!(std::str::from_utf8(&bytes).unwrap(), golden);
        validate_gltf_export(&bytes).unwrap();
    }

    #[test]
    fn gltf_export_contains_per_body_tracks() {
        let scene = SceneExport::new(
            vec![
                BodyTrack::new("stage", vec![sample(0.0, [1.0, 0.0, 0.0])]).unwrap(),
                BodyTrack::new("booster", vec![sample(0.0, [0.0, 1.0, 0.0])]).unwrap(),
            ],
            Vec::new(),
            Vec::new(),
        )
        .unwrap();
        let bytes = generate_gltf_json(&scene).unwrap();
        let value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        let tracks = value["extras"]["tracks"].as_array().unwrap();
        let animations = value["animations"].as_array().unwrap();
        let accessors = value["accessors"].as_array().unwrap();

        assert_eq!(tracks.len(), 2);
        assert_eq!(tracks[0]["body_id"], "booster");
        assert_eq!(tracks[1]["body_id"], "stage");
        assert_eq!(animations[0]["channels"].as_array().unwrap().len(), 4);
        assert_eq!(accessors.len(), 6);
        assert_eq!(accessors[0]["type"], "SCALAR");
        assert_eq!(accessors[1]["type"], "VEC3");
        assert_eq!(accessors[2]["type"], "VEC4");
        validate_gltf_export(&bytes).unwrap();
    }

    #[test]
    fn telemetry_table_builds_scene_export_tracks() {
        let table = telemetry_fixture();
        let scene = scene_export_from_telemetry_table(
            &table,
            &[
                TelemetryBodyTrackSpec::primary("primary").unwrap(),
                TelemetryBodyTrackSpec::separated("observer").unwrap(),
            ],
            Vec::new(),
            Vec::new(),
        )
        .unwrap();

        assert_eq!(scene.tracks.len(), 2);
        assert_eq!(scene.tracks[0].body_id, "observer");
        assert_eq!(scene.tracks[1].body_id, "primary");
        assert!(
            scene.tracks[0].samples[1]
                .position_eci_m
                .iter()
                .zip([1.0, 0.1, 100.0])
                .all(|(actual, expected)| (*actual - expected).abs() < 1.0e-12)
        );
        assert_eq!(
            scene.tracks[0].samples[1].quaternion_body_to_eci_xyzw,
            Some([0.0, 0.0, 0.0, 1.0])
        );

        let czml = generate_czml(&scene).unwrap();
        validate_czml_export(&czml).unwrap();
        let gltf = generate_gltf_json(&scene).unwrap();
        validate_gltf_export(&gltf).unwrap();
        let value: serde_json::Value = serde_json::from_slice(&gltf).unwrap();
        assert_eq!(
            value["animations"][0]["channels"].as_array().unwrap().len(),
            4
        );
    }

    #[test]
    fn telemetry_table_scene_export_fails_closed_on_bad_channels() {
        let table = telemetry_fixture();
        let unknown = scene_export_from_telemetry_table(
            &table,
            &[TelemetryBodyTrackSpec::new(
                "bad",
                [
                    "missing_x".to_owned(),
                    "position_y_m".to_owned(),
                    "position_z_m".to_owned(),
                ],
                None,
            )
            .unwrap()],
            Vec::new(),
            Vec::new(),
        )
        .unwrap_err();
        assert_eq!(
            unknown,
            Export3dError::UnknownChannel {
                channel: "missing_x".to_owned()
            }
        );

        let x = f64_channel(1, "x", "m", Some("ECI"));
        let y = f64_channel(2, "y", "m", Some("ECI"));
        let z = f64_channel(3, "z", "m", Some("ECI"));
        let schema = TelemetrySchema::new(vec![
            x.metadata().clone(),
            y.metadata().clone(),
            z.metadata().clone(),
        ])
        .unwrap();
        let mut missing_table = TelemetryTable::new(schema);
        let mut missing_row =
            TelemetryRow::new(SimTime::ZERO, StepIndex::ZERO).expect("row must be valid");
        missing_row.insert(&x, 1.0).unwrap();
        missing_row.insert(&y, 2.0).unwrap();
        missing_table.push_row(missing_row).unwrap();
        let missing = scene_export_from_telemetry_table(
            &missing_table,
            &[TelemetryBodyTrackSpec::new(
                "missing",
                ["x".to_owned(), "y".to_owned(), "z".to_owned()],
                None,
            )
            .unwrap()],
            Vec::new(),
            Vec::new(),
        )
        .unwrap_err();
        assert_eq!(
            missing,
            Export3dError::MissingSample {
                channel: "z".to_owned(),
                row: 0
            }
        );

        let bad_x = bool_channel(1, "bad_x");
        let good_y = f64_channel(2, "good_y", "m", Some("ECI"));
        let good_z = f64_channel(3, "good_z", "m", Some("ECI"));
        let schema = TelemetrySchema::new(vec![
            bad_x.metadata().clone(),
            good_y.metadata().clone(),
            good_z.metadata().clone(),
        ])
        .unwrap();
        let mut nonnumeric_table = TelemetryTable::new(schema);
        let mut nonnumeric_row =
            TelemetryRow::new(SimTime::ZERO, StepIndex::ZERO).expect("row must be valid");
        nonnumeric_row.insert(&bad_x, true).unwrap();
        nonnumeric_row.insert(&good_y, 2.0).unwrap();
        nonnumeric_row.insert(&good_z, 3.0).unwrap();
        nonnumeric_table.push_row(nonnumeric_row).unwrap();
        let nonnumeric = scene_export_from_telemetry_table(
            &nonnumeric_table,
            &[TelemetryBodyTrackSpec::new(
                "nonnumeric",
                ["bad_x".to_owned(), "good_y".to_owned(), "good_z".to_owned()],
                None,
            )
            .unwrap()],
            Vec::new(),
            Vec::new(),
        )
        .unwrap_err();
        assert_eq!(
            nonnumeric,
            Export3dError::NonNumericSample {
                channel: "bad_x".to_owned(),
                row: 0
            }
        );
    }

    #[test]
    fn export3d_helpers_fail_closed_on_bad_inputs() {
        let empty = SceneExport::new(Vec::new(), Vec::new(), Vec::new()).unwrap_err();
        assert_eq!(empty, Export3dError::EmptyTracks);

        let duplicate_time = BodyTrack::new(
            "vehicle",
            vec![sample(0.0, [0.0; 3]), sample(0.0, [1.0; 3])],
        )
        .unwrap_err();
        assert!(matches!(
            duplicate_time,
            Export3dError::NonIncreasingTime { .. }
        ));

        let nonfinite = GroundSite::new("bad", f64::NAN, 0.0, 0.0).unwrap_err();
        assert!(matches!(nonfinite, Export3dError::NonFinite { .. }));

        let invalid_event = EventInterval::new("bad", "bad", 2.0, 1.0).unwrap_err();
        assert!(matches!(
            invalid_event,
            Export3dError::InvalidInterval { .. }
        ));

        let invalid_czml = validate_czml_export(b"{}").unwrap_err();
        assert!(matches!(
            invalid_czml,
            Export3dError::InvalidDocument { .. }
        ));

        let invalid_gltf = validate_gltf_export(
            br#"{"asset":{"version":"2.0"},"nodes":[{"name":"vehicle"}],"extras":{"tracks":[{}]}}"#,
        )
        .unwrap_err();
        assert!(matches!(
            invalid_gltf,
            Export3dError::InvalidDocument { .. }
        ));
    }
}
