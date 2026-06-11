//! `.arpt` tile assembly (TILER.md §tile_build, FORMAT.md §3–§6).
//!
//! Takes features grouped by layer (geometry in WGS84, already clipped to the
//! tile), quantizes coordinates to tile-local uint16, dictionary-encodes
//! properties (reserved keys first, values deduplicated), builds the FlatBuffer
//! `Tile`, and Brotli-compresses it.
//!
//! Deep module: `build_tile` is the whole interface. Quantization, the geometry
//! union, dictionary construction, and compression are internal. Vector
//! topologies (Point/Line/Polygon) are handled; Mesh/terrain comes later.

use std::collections::HashMap;

use geo_types::{Coord, Geometry, LineString, Polygon};

use crate::fb::tile::arpentry::tiles as fbt;
use crate::project::{self, Bounds};
use crate::terrain::TerrainMesh;
use crate::value::Value;

/// Reserved property keys, in their mandated order (FORMAT.md §4). When present
/// they occupy the front of `Tile.keys`; user keys follow.
const RESERVED_KEYS: [&str; 7] =
    ["class", "subclass", "name", "height", "min_height", "level", "rank"];

/// A feature ready to encode: geometry in WGS84 (clipped to the tile) plus its
/// properties.
#[derive(Debug, Clone)]
pub struct EncoderFeature {
    pub id: u64,
    pub geometry: Geometry,
    pub properties: Vec<(String, Value)>,
}

/// A named layer of features (decode-priority order is the caller's concern).
#[derive(Debug, Clone)]
pub struct EncoderLayer {
    pub name: String,
    pub features: Vec<EncoderFeature>,
}

/// Default Brotli quality for tile compression. Measured on a Natural Earth
/// world z0–6 archive: quality 11 (the encoder's own default) took 42 s where
/// 7 took 1.4 s, and its output was even slightly larger; qualities 5–9 land
/// within 0.3% of each other in size. 7 is the speed/size sweet spot.
pub const DEFAULT_QUALITY: i32 = 7;

/// Builds a Brotli-compressed `.arpt` tile from features grouped by layer.
///
/// `terrain`, when present, is encoded as the first layer ("terrain") with a
/// single `MeshGeometry` feature — the client requires it to render the tile.
pub fn build_tile(bounds: &Bounds, terrain: Option<&TerrainMesh>, layers: &[EncoderLayer]) -> Vec<u8> {
    build_tile_q(bounds, terrain, layers, DEFAULT_QUALITY)
}

/// [`build_tile`] with an explicit Brotli quality (0–11).
pub fn build_tile_q(
    bounds: &Bounds,
    terrain: Option<&TerrainMesh>,
    layers: &[EncoderLayer],
    quality: i32,
) -> Vec<u8> {
    compress(&encode(bounds, terrain, layers), quality)
}

/// Builds the uncompressed `.arpt` FlatBuffer (identifier `"arpt"`).
pub fn encode(bounds: &Bounds, terrain: Option<&TerrainMesh>, layers: &[EncoderLayer]) -> Vec<u8> {
    let dict = Dictionaries::build(layers);
    let mut fbb = flatbuffers::FlatBufferBuilder::new();

    // Property dictionaries (terrain carries no properties).
    let key_offsets: Vec<_> = dict.keys.iter().map(|k| fbb.create_string(k)).collect();
    let keys_vec = fbb.create_vector(&key_offsets);
    let value_offsets: Vec<_> = dict.values.iter().map(|v| build_value(&mut fbb, v)).collect();
    let values_vec = fbb.create_vector(&value_offsets);

    // Layers, terrain first (decode priority 0).
    let mut layer_offsets = Vec::with_capacity(layers.len() + 1);
    if let Some(mesh) = terrain {
        layer_offsets.push(build_terrain_layer(&mut fbb, mesh));
    }
    for layer in layers {
        let mut feat_offsets = Vec::with_capacity(layer.features.len());
        for f in &layer.features {
            if let Some(fo) = build_feature(&mut fbb, bounds, f, &dict) {
                feat_offsets.push(fo);
            }
        }
        let features = fbb.create_vector(&feat_offsets);
        let name = fbb.create_string(&layer.name);
        layer_offsets.push(fbt::Layer::create(
            &mut fbb,
            &fbt::LayerArgs { name: Some(name), features: Some(features) },
        ));
    }
    let layers_vec = fbb.create_vector(&layer_offsets);

    let tile = fbt::Tile::create(
        &mut fbb,
        &fbt::TileArgs {
            version: 1,
            layers: Some(layers_vec),
            keys: Some(keys_vec),
            values: Some(values_vec),
            rasters: None,
        },
    );
    fbt::finish_tile_buffer(&mut fbb, tile);
    fbb.finished_data().to_vec()
}

/// Hashable key for value deduplication (`f64` is keyed by its bit pattern).
#[derive(PartialEq, Eq, Hash)]
enum VKey {
    Str(String),
    Int(i64),
    Dbl(u64),
    Bool(bool),
}

fn vkey(v: &Value) -> VKey {
    match v {
        Value::String(s) => VKey::Str(s.clone()),
        Value::Int(i) => VKey::Int(*i),
        Value::Double(d) => VKey::Dbl(d.to_bits()),
        Value::Bool(b) => VKey::Bool(*b),
    }
}

/// Deduplicated, tile-scoped key and value dictionaries.
struct Dictionaries {
    keys: Vec<String>,
    key_index: HashMap<String, u32>,
    values: Vec<Value>,
    value_index: HashMap<VKey, u32>,
}

impl Dictionaries {
    fn build(layers: &[EncoderLayer]) -> Self {
        let mut used_keys: Vec<String> = Vec::new();
        let mut seen_keys: HashMap<String, ()> = HashMap::new();
        let mut values: Vec<Value> = Vec::new();
        let mut value_index: HashMap<VKey, u32> = HashMap::new();

        for layer in layers {
            for f in &layer.features {
                for (k, v) in &f.properties {
                    if seen_keys.insert(k.clone(), ()).is_none() {
                        used_keys.push(k.clone());
                    }
                    value_index.entry(vkey(v)).or_insert_with(|| {
                        values.push(v.clone());
                        (values.len() - 1) as u32
                    });
                }
            }
        }

        // Reserved keys first (in mandated order), then user keys.
        let mut keys: Vec<String> = RESERVED_KEYS
            .iter()
            .filter(|r| seen_keys.contains_key(**r))
            .map(|r| r.to_string())
            .collect();
        for k in used_keys {
            if !RESERVED_KEYS.contains(&k.as_str()) {
                keys.push(k);
            }
        }
        let key_index = keys.iter().enumerate().map(|(i, k)| (k.clone(), i as u32)).collect();

        Dictionaries { keys, key_index, values, value_index }
    }
}

fn build_value<'a>(
    fbb: &mut flatbuffers::FlatBufferBuilder<'a>,
    v: &Value,
) -> flatbuffers::WIPOffset<fbt::Value<'a>> {
    match v {
        Value::String(s) => {
            let so = fbb.create_string(s);
            fbt::Value::create(
                fbb,
                &fbt::ValueArgs {
                    type_: fbt::PropertyValueType::String,
                    string_value: Some(so),
                    ..Default::default()
                },
            )
        }
        Value::Int(i) => fbt::Value::create(
            fbb,
            &fbt::ValueArgs {
                type_: fbt::PropertyValueType::Int,
                int_value: *i,
                ..Default::default()
            },
        ),
        Value::Double(d) => fbt::Value::create(
            fbb,
            &fbt::ValueArgs {
                type_: fbt::PropertyValueType::Double,
                double_value: *d,
                ..Default::default()
            },
        ),
        Value::Bool(b) => fbt::Value::create(
            fbb,
            &fbt::ValueArgs {
                type_: fbt::PropertyValueType::Bool,
                bool_value: *b,
                ..Default::default()
            },
        ),
    }
}

/// Builds the "terrain" layer: one feature with a `MeshGeometry` (no properties).
fn build_terrain_layer<'a>(
    fbb: &mut flatbuffers::FlatBufferBuilder<'a>,
    mesh: &TerrainMesh,
) -> flatbuffers::WIPOffset<fbt::Layer<'a>> {
    let x = fbb.create_vector(&mesh.x);
    let y = fbb.create_vector(&mesh.y);
    let z = fbb.create_vector(&mesh.z);
    let indices = fbb.create_vector(&mesh.indices);
    let normals = fbb.create_vector(&mesh.normals);
    let geom = fbt::MeshGeometry::create(
        fbb,
        &fbt::MeshGeometryArgs {
            x: Some(x),
            y: Some(y),
            z: Some(z),
            indices: Some(indices),
            normals: Some(normals),
            ..Default::default()
        },
    );
    let feat = fbt::Feature::create(
        fbb,
        &fbt::FeatureArgs {
            id: 0,
            geometry_type: fbt::Geometry::MeshGeometry,
            geometry: Some(geom.as_union_value()),
            properties: None,
        },
    );
    let features = fbb.create_vector(&[feat]);
    let name = fbb.create_string("terrain");
    fbt::Layer::create(fbb, &fbt::LayerArgs { name: Some(name), features: Some(features) })
}

fn build_feature<'a>(
    fbb: &mut flatbuffers::FlatBufferBuilder<'a>,
    bounds: &Bounds,
    f: &EncoderFeature,
    dict: &Dictionaries,
) -> Option<flatbuffers::WIPOffset<fbt::Feature<'a>>> {
    let (geom_type, geom) = build_geometry(fbb, bounds, &f.geometry)?;

    let props: Vec<fbt::Property> = f
        .properties
        .iter()
        .filter_map(|(k, v)| {
            let ki = *dict.key_index.get(k)?;
            let vi = *dict.value_index.get(&vkey(v))?;
            Some(fbt::Property::new(ki, vi))
        })
        .collect();
    let props_vec = fbb.create_vector(&props);

    Some(fbt::Feature::create(
        fbb,
        &fbt::FeatureArgs {
            id: f.id,
            geometry_type: geom_type,
            geometry: Some(geom),
            properties: Some(props_vec),
        },
    ))
}

/// Builds the geometry union member for a feature, returning its type tag and
/// union offset. Quantizes to tile-local uint16; `z` is omitted (vector
/// features carry no source elevation — FORMAT.md §3.4).
fn build_geometry<'a>(
    fbb: &mut flatbuffers::FlatBufferBuilder<'a>,
    bounds: &Bounds,
    geom: &Geometry,
) -> Option<(fbt::Geometry, flatbuffers::WIPOffset<flatbuffers::UnionWIPOffset>)> {
    let q = |c: &Coord| (project::quantize_x(c.x, bounds), project::quantize_y(c.y, bounds));

    match geom {
        Geometry::Point(p) => {
            let (x, y) = q(&p.0);
            point_geometry(fbb, vec![x], vec![y])
        }
        Geometry::MultiPoint(mp) => {
            let (xs, ys) = unzip(mp.0.iter().map(|p| q(&p.0)));
            point_geometry(fbb, xs, ys)
        }
        Geometry::LineString(ls) => line_geometry(fbb, &[ls.clone()], q),
        Geometry::MultiLineString(mls) => line_geometry(fbb, &mls.0, q),
        Geometry::Polygon(p) => polygon_geometry(fbb, std::slice::from_ref(p), q),
        Geometry::MultiPolygon(mp) => polygon_geometry(fbb, &mp.0, q),
        _ => None,
    }
}

fn point_geometry<'a>(
    fbb: &mut flatbuffers::FlatBufferBuilder<'a>,
    xs: Vec<u16>,
    ys: Vec<u16>,
) -> Option<(fbt::Geometry, flatbuffers::WIPOffset<flatbuffers::UnionWIPOffset>)> {
    if xs.is_empty() {
        return None;
    }
    let x = fbb.create_vector(&xs);
    let y = fbb.create_vector(&ys);
    let g = fbt::PointGeometry::create(fbb, &fbt::PointGeometryArgs { x: Some(x), y: Some(y), z: None });
    Some((fbt::Geometry::PointGeometry, g.as_union_value()))
}

fn line_geometry<'a>(
    fbb: &mut flatbuffers::FlatBufferBuilder<'a>,
    lines: &[LineString],
    q: impl Fn(&Coord) -> (u16, u16),
) -> Option<(fbt::Geometry, flatbuffers::WIPOffset<flatbuffers::UnionWIPOffset>)> {
    let mut xs = Vec::new();
    let mut ys = Vec::new();
    let mut offsets = vec![0u32];
    for line in lines {
        for c in &line.0 {
            let (x, y) = q(c);
            xs.push(x);
            ys.push(y);
        }
        offsets.push(xs.len() as u32);
    }
    if xs.len() < 2 {
        return None;
    }
    let x = fbb.create_vector(&xs);
    let y = fbb.create_vector(&ys);
    // Single linestring omits line_offsets.
    let line_offsets = (lines.len() > 1).then(|| fbb.create_vector(&offsets));
    let g = fbt::LineGeometry::create(
        fbb,
        &fbt::LineGeometryArgs { x: Some(x), y: Some(y), z: None, line_offsets },
    );
    Some((fbt::Geometry::LineGeometry, g.as_union_value()))
}

fn polygon_geometry<'a>(
    fbb: &mut flatbuffers::FlatBufferBuilder<'a>,
    polygons: &[Polygon],
    q: impl Fn(&Coord) -> (u16, u16),
) -> Option<(fbt::Geometry, flatbuffers::WIPOffset<flatbuffers::UnionWIPOffset>)> {
    let mut xs = Vec::new();
    let mut ys = Vec::new();
    let mut ring_offsets = vec![0u32];
    let mut polygon_offsets = vec![0u32]; // indexes into rings

    let push_ring = |ring: &LineString, xs: &mut Vec<u16>, ys: &mut Vec<u16>| {
        // Store open rings (drop the repeated closing vertex; FORMAT.md §3.4).
        let coords = &ring.0;
        let n = if coords.len() > 1 && coords.first() == coords.last() {
            coords.len() - 1
        } else {
            coords.len()
        };
        for c in &coords[..n] {
            let (x, y) = q(c);
            xs.push(x);
            ys.push(y);
        }
    };

    let mut ring_count = 0u32;
    for poly in polygons {
        push_ring(poly.exterior(), &mut xs, &mut ys);
        ring_offsets.push(xs.len() as u32);
        ring_count += 1;
        for hole in poly.interiors() {
            push_ring(hole, &mut xs, &mut ys);
            ring_offsets.push(xs.len() as u32);
            ring_count += 1;
        }
        polygon_offsets.push(ring_count);
    }
    if xs.is_empty() {
        return None;
    }

    let x = fbb.create_vector(&xs);
    let y = fbb.create_vector(&ys);
    // Single ring (no holes) omits ring_offsets; only multipolygons add
    // polygon_offsets (FORMAT.md §3.4).
    let multi = polygons.len() > 1;
    let ring_off = (ring_count > 1).then(|| fbb.create_vector(&ring_offsets));
    let poly_off = multi.then(|| fbb.create_vector(&polygon_offsets));
    let g = fbt::PolygonGeometry::create(
        fbb,
        &fbt::PolygonGeometryArgs {
            x: Some(x),
            y: Some(y),
            z: None,
            ring_offsets: ring_off,
            polygon_offsets: poly_off,
        },
    );
    Some((fbt::Geometry::PolygonGeometry, g.as_union_value()))
}

fn unzip(it: impl Iterator<Item = (u16, u16)>) -> (Vec<u16>, Vec<u16>) {
    let mut xs = Vec::new();
    let mut ys = Vec::new();
    for (x, y) in it {
        xs.push(x);
        ys.push(y);
    }
    (xs, ys)
}

fn compress(data: &[u8], quality: i32) -> Vec<u8> {
    let mut params = brotli::enc::BrotliEncoderParams::default();
    params.quality = quality.clamp(0, 11);
    let mut out = Vec::new();
    let mut input = data;
    brotli::BrotliCompress(&mut input, &mut out, &params).expect("brotli compress in-memory");
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{LineString, MultiPolygon, Point};

    fn bounds() -> Bounds {
        Bounds::of_tile(5, 10, 7)
    }

    fn decompress(data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut input = data;
        brotli::BrotliDecompress(&mut input, &mut out).unwrap();
        out
    }

    fn c(x: f64, y: f64) -> Coord {
        Coord { x, y }
    }

    #[test]
    fn arpt_identifier_and_compression() {
        let b = bounds();
        let layers = vec![EncoderLayer {
            name: "water".to_string(),
            features: vec![EncoderFeature {
                id: 1,
                geometry: Geometry::Point(Point::new(b.west + b.width() / 2.0, b.south + b.height() / 2.0)),
                properties: vec![("class".to_string(), Value::String("lake".into()))],
            }],
        }];
        let raw = encode(&b, None, &layers);
        assert_eq!(&raw[4..8], b"arpt");
        // build_tile compresses and round-trips back to the same bytes.
        assert_eq!(decompress(&build_tile(&b, None, &layers)), raw);
    }

    #[test]
    fn polygon_feature_roundtrips() {
        let b = bounds();
        // A polygon inside the tile; geo-types rings are closed.
        let poly = Polygon::new(
            LineString(vec![
                c(b.west + 0.1 * b.width(), b.south + 0.1 * b.height()),
                c(b.west + 0.4 * b.width(), b.south + 0.1 * b.height()),
                c(b.west + 0.4 * b.width(), b.south + 0.4 * b.height()),
                c(b.west + 0.1 * b.width(), b.south + 0.1 * b.height()),
            ]),
            vec![],
        );
        let layers = vec![EncoderLayer {
            name: "land".to_string(),
            features: vec![EncoderFeature {
                id: 42,
                geometry: Geometry::Polygon(poly),
                properties: vec![("class".to_string(), Value::String("forest".into()))],
            }],
        }];

        let raw = encode(&b, None, &layers);
        let tile = fbt::root_as_tile(&raw).expect("read tile");
        assert_eq!(tile.version(), 1);

        let layers_r = tile.layers().expect("layers");
        assert_eq!(layers_r.len(), 1);
        let layer = layers_r.get(0);
        assert_eq!(layer.name(), "land");

        let feats = layer.features().expect("features");
        assert_eq!(feats.len(), 1);
        let feat = feats.get(0);
        assert_eq!(feat.id(), 42);
        assert_eq!(feat.geometry_type(), fbt::Geometry::PolygonGeometry);

        let pg = feat.geometry_as_polygon_geometry().expect("polygon geometry");
        // Open ring: 4 closed coords → 3 stored vertices; single ring → no offsets.
        assert_eq!(pg.x().len(), 3);
        assert!(pg.ring_offsets().is_none());
        assert!(pg.polygon_offsets().is_none());

        // class=forest resolves through the dictionaries.
        let keys = tile.keys().expect("keys");
        let values = tile.values().expect("values");
        let props = feat.properties().expect("properties");
        assert_eq!(props.len(), 1);
        let p = props.get(0);
        assert_eq!(keys.get(p.key() as usize), "class");
        let v = values.get(p.value() as usize);
        assert_eq!(v.type_(), fbt::PropertyValueType::String);
        assert_eq!(v.string_value(), Some("forest"));
    }

    #[test]
    fn reserved_keys_come_first() {
        let b = bounds();
        let layers = vec![EncoderLayer {
            name: "poi".to_string(),
            features: vec![EncoderFeature {
                id: 1,
                geometry: Geometry::Point(Point::new(b.west + b.width() / 2.0, b.south + b.height() / 2.0)),
                // user key before reserved keys in input order
                properties: vec![
                    ("custom".to_string(), Value::Int(7)),
                    ("class".to_string(), Value::String("cafe".into())),
                    ("name".to_string(), Value::String("X".into())),
                ],
            }],
        }];
        let raw = encode(&b, None, &layers);
        let tile = fbt::root_as_tile(&raw).unwrap();
        let keys = tile.keys().unwrap();
        // class (reserved idx 0) then name (reserved idx 2) precede 'custom'.
        assert_eq!(keys.get(0), "class");
        assert_eq!(keys.get(1), "name");
        assert_eq!(keys.get(2), "custom");
    }

    #[test]
    fn value_dedup_across_features() {
        let b = bounds();
        let pt = |dx: f64| Geometry::Point(Point::new(b.west + dx * b.width(), b.south + 0.5 * b.height()));
        let layers = vec![EncoderLayer {
            name: "tree".to_string(),
            features: vec![
                EncoderFeature { id: 1, geometry: pt(0.3), properties: vec![("class".into(), Value::String("oak".into()))] },
                EncoderFeature { id: 2, geometry: pt(0.6), properties: vec![("class".into(), Value::String("oak".into()))] },
            ],
        }];
        let raw = encode(&b, None, &layers);
        let tile = fbt::root_as_tile(&raw).unwrap();
        // "oak" stored once despite two features referencing it.
        assert_eq!(tile.values().unwrap().len(), 1);
        assert_eq!(tile.keys().unwrap().len(), 1);
    }

    #[test]
    fn multipolygon_has_both_offset_arrays() {
        let b = bounds();
        let sq = |ox: f64, oy: f64| {
            Polygon::new(
                LineString(vec![
                    c(b.west + ox * b.width(), b.south + oy * b.height()),
                    c(b.west + (ox + 0.1) * b.width(), b.south + oy * b.height()),
                    c(b.west + (ox + 0.1) * b.width(), b.south + (oy + 0.1) * b.height()),
                    c(b.west + ox * b.width(), b.south + oy * b.height()),
                ]),
                vec![],
            )
        };
        let mp = Geometry::MultiPolygon(MultiPolygon(vec![sq(0.1, 0.1), sq(0.5, 0.5)]));
        let layers = vec![EncoderLayer {
            name: "land".into(),
            features: vec![EncoderFeature { id: 1, geometry: mp, properties: vec![] }],
        }];
        let raw = encode(&b, None, &layers);
        let tile = fbt::root_as_tile(&raw).unwrap();
        let pg = tile.layers().unwrap().get(0).features().unwrap().get(0).geometry_as_polygon_geometry().unwrap();
        assert!(pg.ring_offsets().is_some());
        assert!(pg.polygon_offsets().is_some());
        assert_eq!(pg.polygon_offsets().unwrap().len(), 3); // 2 polygons → N+1
    }

    #[test]
    fn terrain_layer_encodes_as_mesh_first() {
        let b = bounds();
        let mesh = crate::terrain::flat_mesh(4);
        let raw = encode(&b, Some(&mesh), &[]);
        let tile = fbt::root_as_tile(&raw).unwrap();

        let layers = tile.layers().expect("layers");
        assert_eq!(layers.len(), 1);
        let terr = layers.get(0);
        assert_eq!(terr.name(), "terrain");

        let feat = terr.features().expect("features").get(0);
        assert_eq!(feat.geometry_type(), fbt::Geometry::MeshGeometry);
        let m = feat.geometry_as_mesh_geometry().expect("mesh geometry");
        assert_eq!(m.x().len(), 25); // 5×5 vertices
        assert_eq!(m.z().len(), 25);
        assert_eq!(m.indices().len(), 4 * 4 * 6);
    }
}
