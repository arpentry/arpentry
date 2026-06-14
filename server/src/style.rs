//! Builds the `.arps` style blob from a JSON source file (port of `resp_style.c`).
//!
//! The server's `--style` argument points at a JSON document (the same one the C
//! server reads); this module parses it with `serde_json`, serialises it into the
//! `Style` FlatBuffer, and Brotli-compresses the result. Absent JSON fields fall
//! back to the schema defaults, exactly like the C builder's conditional adds.

use serde_json::Value as Json;

use crate::fb::style::arpentry::tiles as fbs;

const BROTLI_QUALITY: i32 = 4;

/// Errors building the style blob.
#[derive(Debug)]
pub enum StyleError {
    Io(std::io::Error),
    Json(serde_json::Error),
}

impl std::fmt::Display for StyleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StyleError::Io(e) => write!(f, "style: cannot read file: {e}"),
            StyleError::Json(e) => write!(f, "style: invalid JSON: {e}"),
        }
    }
}

impl std::error::Error for StyleError {}

/// Reads and builds the Brotli-compressed `.arps` blob from a JSON style file.
pub fn build_from_file(path: &std::path::Path) -> Result<Vec<u8>, StyleError> {
    let text = std::fs::read_to_string(path).map_err(StyleError::Io)?;
    let root: Json = serde_json::from_str(&text).map_err(StyleError::Json)?;
    Ok(compress(&encode(&root)))
}

/// Parses a JSON `[r, g, b, a]` array into an RGBA struct (a defaults to 255).
fn parse_rgba(arr: &Json) -> fbs::RGBA {
    let get = |i: usize| arr.get(i).and_then(Json::as_i64);
    fbs::RGBA::new(
        get(0).unwrap_or(0) as u8,
        get(1).unwrap_or(0) as u8,
        get(2).unwrap_or(0) as u8,
        get(3).unwrap_or(255) as u8,
    )
}

fn layer_type(s: &str) -> fbs::LayerType {
    match s {
        "terrain" => fbs::LayerType::Terrain,
        "extrusion" => fbs::LayerType::Extrusion,
        "instance" => fbs::LayerType::Instance,
        "label" => fbs::LayerType::Label,
        "line" => fbs::LayerType::Line,
        "line_label" => fbs::LayerType::LineLabel,
        _ => fbs::LayerType::Texture,
    }
}

/// Builds the uncompressed `.arps` FlatBuffer (identifier `"arps"`).
fn encode(root: &Json) -> Vec<u8> {
    let mut fbb = flatbuffers::FlatBufferBuilder::new();

    let name = root.get("name").and_then(Json::as_str).map(|s| fbb.create_string(s));

    // Layers must be built before the Style table that references them.
    let mut layer_offsets = Vec::new();
    if let Some(layers) = root.get("layers").and_then(Json::as_array) {
        for layer in layers {
            layer_offsets.push(encode_layer(&mut fbb, layer));
        }
    }
    let layers_vec = fbb.create_vector(&layer_offsets);

    let version = root.get("version").and_then(Json::as_u64).unwrap_or(1) as u16;
    let background = root.get("background").map(parse_rgba);

    let style = fbs::Style::create(
        &mut fbb,
        &fbs::StyleArgs {
            version,
            name,
            background: background.as_ref(),
            layers: Some(layers_vec),
        },
    );
    fbs::finish_style_buffer(&mut fbb, style);
    fbb.finished_data().to_vec()
}

fn encode_layer<'a>(
    fbb: &mut flatbuffers::FlatBufferBuilder<'a>,
    layer: &Json,
) -> flatbuffers::WIPOffset<fbs::LayerStyle<'a>> {
    let source_layer = layer.get("source_layer").and_then(Json::as_str).map(|s| fbb.create_string(s));

    // Paint entries (and their strings) first.
    let mut paint_offsets = Vec::new();
    if let Some(paint) = layer.get("paint").and_then(Json::as_array) {
        for entry in paint {
            paint_offsets.push(encode_paint(fbb, entry));
        }
    }
    let paint_vec = fbb.create_vector(&paint_offsets);

    let d = fbs::LayerStyleArgs::default();
    let type_ = layer.get("type").and_then(Json::as_str).map(layer_type).unwrap_or(d.type_);
    let min_level = layer.get("min_level").and_then(Json::as_u64).map_or(d.min_level, |v| v as u8);
    let text_size = layer.get("text_size").and_then(Json::as_f64).map_or(d.text_size, |v| v as f32);
    let text_halo_width =
        layer.get("text_halo_width").and_then(Json::as_f64).map_or(d.text_halo_width, |v| v as f32);
    let icon_size = layer.get("icon_size").and_then(Json::as_f64).map_or(d.icon_size, |v| v as f32);
    let icon_halo_width =
        layer.get("icon_halo_width").and_then(Json::as_f64).map_or(d.icon_halo_width, |v| v as f32);

    // RGBA structs must outlive the create() call below.
    let text_color = layer.get("text_color").map(parse_rgba);
    let text_halo_color = layer.get("text_halo_color").map(parse_rgba);
    let icon_color = layer.get("icon_color").map(parse_rgba);
    let icon_halo_color = layer.get("icon_halo_color").map(parse_rgba);

    fbs::LayerStyle::create(
        fbb,
        &fbs::LayerStyleArgs {
            source_layer,
            type_,
            min_level,
            paint: Some(paint_vec),
            text_size,
            text_color: text_color.as_ref(),
            text_halo_color: text_halo_color.as_ref(),
            text_halo_width,
            icon_size,
            icon_color: icon_color.as_ref(),
            icon_halo_color: icon_halo_color.as_ref(),
            icon_halo_width,
        },
    )
}

fn encode_paint<'a>(
    fbb: &mut flatbuffers::FlatBufferBuilder<'a>,
    entry: &Json,
) -> flatbuffers::WIPOffset<fbs::PaintEntry<'a>> {
    let class = entry.get("class").and_then(Json::as_str).map(|s| fbb.create_string(s));
    let model = entry.get("model").and_then(Json::as_str).map(|s| fbb.create_string(s));

    let d = fbs::PaintEntryArgs::default();
    let width = entry.get("width").and_then(Json::as_f64).map_or(d.width, |v| v as f32);
    let min_scale = entry.get("min_scale").and_then(Json::as_f64).map_or(d.min_scale, |v| v as f32);
    let max_scale = entry.get("max_scale").and_then(Json::as_f64).map_or(d.max_scale, |v| v as f32);
    let random_yaw = entry.get("random_yaw").and_then(Json::as_bool).unwrap_or(d.random_yaw);
    let random_scale = entry.get("random_scale").and_then(Json::as_bool).unwrap_or(d.random_scale);
    let min_level = entry.get("min_level").and_then(Json::as_u64).map_or(d.min_level, |v| v as u8);
    let casing_width =
        entry.get("casing_width").and_then(Json::as_f64).map_or(d.casing_width, |v| v as f32);

    let color = entry.get("color").map(parse_rgba);
    let casing_color = entry.get("casing_color").map(parse_rgba);

    fbs::PaintEntry::create(
        fbb,
        &fbs::PaintEntryArgs {
            class,
            color: color.as_ref(),
            width,
            model,
            min_scale,
            max_scale,
            random_yaw,
            random_scale,
            min_level,
            casing_color: casing_color.as_ref(),
            casing_width,
        },
    )
}

fn compress(data: &[u8]) -> Vec<u8> {
    let mut params = brotli::enc::BrotliEncoderParams::default();
    params.quality = BROTLI_QUALITY;
    let mut out = Vec::new();
    let mut input = data;
    brotli::BrotliCompress(&mut input, &mut out, &params).expect("brotli compress in-memory");
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decompress(data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut input = data;
        brotli::BrotliDecompress(&mut input, &mut out).unwrap();
        out
    }

    #[test]
    fn encodes_layers_and_paint() {
        let json: Json = serde_json::from_str(
            r#"{
                "version": 1,
                "name": "Test",
                "background": [240, 234, 224, 255],
                "layers": [
                    { "source_layer": "terrain", "type": "terrain", "paint": [] },
                    { "source_layer": "transportation", "type": "line",
                      "paint": [ { "class": "primary", "color": [89, 84, 77, 255], "width": 140,
                                   "casing_color": [200, 200, 200, 255], "casing_width": 24 } ] }
                ]
            }"#,
        )
        .unwrap();

        let raw = encode(&json);
        assert_eq!(&raw[4..8], b"arps");
        let style = fbs::root_as_style(&raw).unwrap();
        assert_eq!(style.version(), 1);
        assert_eq!(style.name(), Some("Test"));
        let layers = style.layers().unwrap();
        assert_eq!(layers.len(), 2);
        assert_eq!(layers.get(0).type_(), fbs::LayerType::Terrain);
        let road = layers.get(1);
        assert_eq!(road.type_(), fbs::LayerType::Line);
        let paint = road.paint().unwrap();
        assert_eq!(paint.get(0).class(), "primary");
        assert_eq!(paint.get(0).width(), 140.0);
        assert_eq!(paint.get(0).casing_width(), 24.0);
        assert_eq!(paint.get(0).casing_color().map(|c| c.r()), Some(200));
        // build() round-trips through compression.
        assert_eq!(decompress(&compress(&raw)), raw);
    }

    #[test]
    fn defaults_apply_when_fields_absent() {
        let json: Json = serde_json::from_str(
            r#"{ "layers": [ { "source_layer": "poi", "type": "label", "paint": [] } ] }"#,
        )
        .unwrap();
        let raw = encode(&json);
        let style = fbs::root_as_style(&raw).unwrap();
        assert_eq!(style.version(), 1);
        let layer = style.layers().unwrap().get(0);
        assert_eq!(layer.text_size(), 14.0); // schema default
        assert_eq!(layer.icon_size(), 20.0);
    }
}
