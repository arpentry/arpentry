//! Tile assembly + compression for procedural tiles (port of `gen/world.c`).
//!
//! [`generate_terrain`] is the whole interface: given a tile address it builds
//! the elevation mesh, biome surface, town (when overlapping), trees, and POIs,
//! serialises them into a `Tile` FlatBuffer with a tile-scope property
//! dictionary, and Brotli-compresses the result. The layer set, dictionary
//! order, and per-feature property indices match the C encoder exactly so the
//! same client decodes both.

use flatbuffers::{FlatBufferBuilder, WIPOffset};

use super::{poi, surface, terrain, town, tree};
use crate::fb::tile::arpentry::tiles as fbt;
use crate::project::{self, Bounds};

/// Brotli quality for on-the-fly tiles — favours latency over ratio (matches C).
const BROTLI_QUALITY: i32 = 4;

/// Metres → degrees at the equator (for building footprint half-extents).
const M_TO_DEG: f64 = 1.0 / 111_319.5;

/// Generates a Brotli-compressed `.arpt` tile for `(z, x, y)` from procedural noise.
pub fn generate_terrain(z: u8, x: u32, y: u32) -> Vec<u8> {
    let bounds = Bounds::of_tile(z, x, y);
    let fb = build_tile(&bounds);
    compress(&fb)
}

/// Builds the uncompressed `Tile` FlatBuffer for the given bounds.
fn build_tile(bounds: &Bounds) -> Vec<u8> {
    let mut fbb = FlatBufferBuilder::new();
    let has_town = town::town_overlaps(bounds);

    // ── Property dictionary ──────────────────────────────────────────────
    // Keys: class(0) height(1) name(2) icon(3).
    let key_offsets: Vec<_> =
        ["class", "height", "name", "icon"].iter().map(|k| fbb.create_string(k)).collect();
    let keys_vec = fbb.create_vector(&key_offsets);

    // Values: order must match SURFACE_VAL_* / TOWN_VAL_* / TREE_VAL_* / POI_*.
    let mut value_offsets: Vec<WIPOffset<fbt::Value>> = Vec::new();
    for s in [
        "water", "desert", "forest", "grassland", "cropland", "shrub", "ice", // 0-6
        "primary", "residential", "building", // 7-9
    ] {
        value_offsets.push(string_value(&mut fbb, s));
    }
    for n in [5i64, 8, 10, 12, 15] {
        value_offsets.push(int_value(&mut fbb, n)); // 10-14
    }
    for s in ["oak", "pine", "birch", "poi"] {
        value_offsets.push(string_value(&mut fbb, s)); // 15-18
    }
    // POI name strings (19+), then POI icon strings, in poi::POIS order.
    for p in poi::POIS {
        value_offsets.push(string_value(&mut fbb, p.name));
    }
    for p in poi::POIS {
        value_offsets.push(string_value(&mut fbb, p.icon));
    }
    let values_vec = fbb.create_vector(&value_offsets);

    // ── Layers ───────────────────────────────────────────────────────────
    let mut layers: Vec<WIPOffset<fbt::Layer>> = Vec::new();

    // Layer 0: terrain (one mesh feature).
    {
        let mesh = terrain::build_mesh(bounds);
        let x = fbb.create_vector(&mesh.vx);
        let y = fbb.create_vector(&mesh.vy);
        let zv = fbb.create_vector(&mesh.vz);
        let indices = fbb.create_vector(&mesh.indices);
        let normals = fbb.create_vector(&mesh.normals);
        // Single part covering all indices; color.a = 0 → client-styled.
        let part = fbt::Part::new(0, mesh.indices.len() as u32, &fbt::Color::new(0, 0, 0, 0), 0, 0);
        let parts = fbb.create_vector(&[part]);
        let geom = fbt::MeshGeometry::create(
            &mut fbb,
            &fbt::MeshGeometryArgs {
                x: Some(x),
                y: Some(y),
                z: Some(zv),
                indices: Some(indices),
                normals: Some(normals),
                parts: Some(parts),
                ..Default::default()
            },
        );
        let feat = fbt::Feature::create(
            &mut fbb,
            &fbt::FeatureArgs {
                id: 1,
                geometry_type: fbt::Geometry::MeshGeometry,
                geometry: Some(geom.as_union_value()),
                properties: None,
            },
        );
        let features = fbb.create_vector(&[feat]);
        let name = fbb.create_string("terrain");
        layers.push(fbt::Layer::create(&mut fbb, &fbt::LayerArgs { name: Some(name), features: Some(features) }));
    }

    // Layer 1: surface (biome polygons).
    {
        let patches = surface::generate_surface_patches(bounds);
        let mut feats = Vec::with_capacity(patches.len());
        for (i, p) in patches.iter().enumerate() {
            let count = p.x.len();
            let x = fbb.create_vector(&p.x);
            let y = fbb.create_vector(&p.y);
            let zv = fbb.create_vector(&vec![0i32; count]);
            let ring = fbb.create_vector(&[0u32, count as u32]);
            let geom = fbt::PolygonGeometry::create(
                &mut fbb,
                &fbt::PolygonGeometryArgs {
                    x: Some(x),
                    y: Some(y),
                    z: Some(zv),
                    ring_offsets: Some(ring),
                    polygon_offsets: None,
                },
            );
            let props = fbb.create_vector(&[fbt::Property::new(0, p.cls)]);
            feats.push(fbt::Feature::create(
                &mut fbb,
                &fbt::FeatureArgs {
                    id: (i + 2) as u64,
                    geometry_type: fbt::Geometry::PolygonGeometry,
                    geometry: Some(geom.as_union_value()),
                    properties: Some(props),
                },
            ));
        }
        let features = fbb.create_vector(&feats);
        let name = fbb.create_string("surface");
        layers.push(fbt::Layer::create(&mut fbb, &fbt::LayerArgs { name: Some(name), features: Some(features) }));
    }

    // Layer 2: transportation (roads) — only when the tile overlaps the town.
    if has_town {
        let roads = &town::town().roads;
        let mut feats = Vec::with_capacity(roads.len());
        for (i, r) in roads.iter().enumerate() {
            let rx = [project::quantize_x(r.lon1, bounds), project::quantize_x(r.lon2, bounds)];
            let ry = [project::quantize_y(r.lat1, bounds), project::quantize_y(r.lat2, bounds)];
            let x = fbb.create_vector(&rx);
            let y = fbb.create_vector(&ry);
            let zv = fbb.create_vector(&[0i32, 0]);
            let geom = fbt::LineGeometry::create(
                &mut fbb,
                &fbt::LineGeometryArgs { x: Some(x), y: Some(y), z: Some(zv), line_offsets: None },
            );
            let props = fbb.create_vector(&[fbt::Property::new(0, r.cls)]);
            feats.push(fbt::Feature::create(
                &mut fbb,
                &fbt::FeatureArgs {
                    id: (100_000 + i) as u64,
                    geometry_type: fbt::Geometry::LineGeometry,
                    geometry: Some(geom.as_union_value()),
                    properties: Some(props),
                },
            ));
        }
        let features = fbb.create_vector(&feats);
        let name = fbb.create_string("transportation");
        layers.push(fbt::Layer::create(&mut fbb, &fbt::LayerArgs { name: Some(name), features: Some(features) }));
    }

    // Layer 3: building footprints — only when the tile overlaps the town.
    if has_town {
        let bldgs = &town::town().buildings;
        let mut feats = Vec::with_capacity(bldgs.len());
        for (i, b) in bldgs.iter().enumerate() {
            let hw = b.w_m * M_TO_DEG * 0.5;
            let hh = b.h_m * M_TO_DEG * 0.5;
            // CCW ring (SW SE NE NW close), matching the surface convention.
            let lons = [b.lon - hw, b.lon + hw, b.lon + hw, b.lon - hw, b.lon - hw];
            let lats = [b.lat - hh, b.lat - hh, b.lat + hh, b.lat + hh, b.lat - hh];
            let bx: Vec<u16> = lons.iter().map(|&l| project::quantize_x(l, bounds)).collect();
            let by: Vec<u16> = lats.iter().map(|&l| project::quantize_y(l, bounds)).collect();
            let base_z = project::quantize_z(terrain::terrain_elevation(b.lon, b.lat));
            let x = fbb.create_vector(&bx);
            let y = fbb.create_vector(&by);
            let zv = fbb.create_vector(&vec![base_z; 5]);
            let ring = fbb.create_vector(&[0u32, 5]);
            let geom = fbt::PolygonGeometry::create(
                &mut fbb,
                &fbt::PolygonGeometryArgs {
                    x: Some(x),
                    y: Some(y),
                    z: Some(zv),
                    ring_offsets: Some(ring),
                    polygon_offsets: None,
                },
            );
            let props = fbb.create_vector(&[
                fbt::Property::new(0, b.cls),
                fbt::Property::new(town::TOWN_KEY_HEIGHT, b.height_val),
            ]);
            feats.push(fbt::Feature::create(
                &mut fbb,
                &fbt::FeatureArgs {
                    id: (200_000 + i) as u64,
                    geometry_type: fbt::Geometry::PolygonGeometry,
                    geometry: Some(geom.as_union_value()),
                    properties: Some(props),
                },
            ));
        }
        let features = fbb.create_vector(&feats);
        let name = fbb.create_string("building");
        layers.push(fbt::Layer::create(&mut fbb, &fbt::LayerArgs { name: Some(name), features: Some(features) }));
    }

    // Layer 4: trees (forest points within the tile proper).
    {
        let trees = tree::generate_trees(bounds);
        if !trees.is_empty() {
            let mut feats = Vec::new();
            for t in &trees {
                // One tile owns each tree: skip those in the buffer zone.
                if t.lon < bounds.west || t.lon >= bounds.east || t.lat < bounds.south || t.lat >= bounds.north {
                    continue;
                }
                let tx = [project::quantize_x(t.lon, bounds)];
                let ty = [project::quantize_y(t.lat, bounds)];
                let tz = [project::quantize_z(terrain::terrain_elevation(t.lon, t.lat))];
                let x = fbb.create_vector(&tx);
                let y = fbb.create_vector(&ty);
                let zv = fbb.create_vector(&tz);
                let geom = fbt::PointGeometry::create(
                    &mut fbb,
                    &fbt::PointGeometryArgs { x: Some(x), y: Some(y), z: Some(zv) },
                );
                let props = fbb.create_vector(&[fbt::Property::new(0, t.class_val)]);
                feats.push(fbt::Feature::create(
                    &mut fbb,
                    &fbt::FeatureArgs {
                        id: t.id,
                        geometry_type: fbt::Geometry::PointGeometry,
                        geometry: Some(geom.as_union_value()),
                        properties: Some(props),
                    },
                ));
            }
            let features = fbb.create_vector(&feats);
            let name = fbb.create_string("tree");
            layers.push(fbt::Layer::create(&mut fbb, &fbt::LayerArgs { name: Some(name), features: Some(features) }));
        }
    }

    // Layer 5: POIs (named points within the tile proper).
    if poi::poi_overlaps(bounds) {
        let np = poi::POIS.len() as u32;
        let mut feats = Vec::new();
        for (pi, p) in poi::POIS.iter().enumerate() {
            if p.lon < bounds.west || p.lon >= bounds.east || p.lat < bounds.south || p.lat >= bounds.north {
                continue;
            }
            let px = [project::quantize_x(p.lon, bounds)];
            let py = [project::quantize_y(p.lat, bounds)];
            let pz = [project::quantize_z(terrain::terrain_elevation(p.lon, p.lat))];
            let x = fbb.create_vector(&px);
            let y = fbb.create_vector(&py);
            let zv = fbb.create_vector(&pz);
            let geom = fbt::PointGeometry::create(
                &mut fbb,
                &fbt::PointGeometryArgs { x: Some(x), y: Some(y), z: Some(zv) },
            );
            let props = fbb.create_vector(&[
                fbt::Property::new(0, poi::POI_VAL_POI),
                fbt::Property::new(poi::POI_KEY_NAME, poi::POI_VAL_NAME_BASE + pi as u32),
                fbt::Property::new(poi::POI_KEY_ICON, poi::POI_VAL_NAME_BASE + np + pi as u32),
            ]);
            feats.push(fbt::Feature::create(
                &mut fbb,
                &fbt::FeatureArgs {
                    id: (400_000 + pi) as u64,
                    geometry_type: fbt::Geometry::PointGeometry,
                    geometry: Some(geom.as_union_value()),
                    properties: Some(props),
                },
            ));
        }
        let features = fbb.create_vector(&feats);
        let name = fbb.create_string("poi");
        layers.push(fbt::Layer::create(&mut fbb, &fbt::LayerArgs { name: Some(name), features: Some(features) }));
    }

    let layers_vec = fbb.create_vector(&layers);
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

fn string_value<'a>(fbb: &mut FlatBufferBuilder<'a>, s: &str) -> WIPOffset<fbt::Value<'a>> {
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

fn int_value<'a>(fbb: &mut FlatBufferBuilder<'a>, v: i64) -> WIPOffset<fbt::Value<'a>> {
    fbt::Value::create(
        fbb,
        &fbt::ValueArgs { type_: fbt::PropertyValueType::Int, int_value: v, ..Default::default() },
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
    fn ocean_tile_has_terrain_and_surface_only() {
        // A tile far from the town: terrain + surface layers, no town layers.
        let blob = generate_terrain(4, 0, 0);
        let raw = decompress(&blob);
        assert_eq!(&raw[4..8], b"arpt");
        let tile = fbt::root_as_tile(&raw).expect("tile root");
        let layers = tile.layers().expect("layers");
        assert_eq!(layers.get(0).name(), "terrain");
        assert_eq!(layers.get(1).name(), "surface");
        // No town overlap → no transportation/building layers.
        for i in 0..layers.len() {
            let n = layers.get(i).name();
            assert!(n != "transportation" && n != "building");
        }
        // Dictionary keys are class/height/name/icon.
        let keys = tile.keys().unwrap();
        assert_eq!(keys.get(0), "class");
        assert_eq!(keys.get(3), "icon");
    }

    #[test]
    fn town_tile_has_all_layers() {
        // A high-zoom tile centred on (0, 0) overlaps town, trees, and POIs.
        let blob = generate_terrain(14, 8192, 8192);
        let raw = decompress(&blob);
        let tile = fbt::root_as_tile(&raw).expect("tile root");
        let layers = tile.layers().unwrap();
        let names: Vec<&str> = (0..layers.len()).map(|i| layers.get(i).name()).collect();
        assert!(names.contains(&"terrain"));
        assert!(names.contains(&"surface"));
        assert!(names.contains(&"transportation"));
        assert!(names.contains(&"building"));
    }
}
