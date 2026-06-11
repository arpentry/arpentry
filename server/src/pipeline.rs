//! Top-level pipeline (TILER.md §pipeline).
//!
//! Runs the sort-based tiling: read features → profile → per zoom simplify +
//! clip to tiles → serialize sort records keyed by Hilbert tile id → external
//! merge sort → group by tile → build `.arpt` → write `.arpa` archive (+ `.arpi`
//! metadata).
//!
//! Single-threaded and dependency-free. The C tiler runs the stages on worker
//! pools; that concurrency can be layered on with `std::thread` + a bounded
//! queue later without new dependencies (the external sort already supports
//! per-thread instances merged k-way).

use std::fs::File;
use std::path::PathBuf;

use geo_types::Geometry;

use crate::archive::{ArchiveMeta, ArchiveWriter};
use crate::clip;
use crate::dem::Dem;
use crate::geom::GeometryType;
use crate::geoparquet::GeoParquet;
use crate::hilbert;
use crate::layers;
use crate::profile;
use crate::project::Bounds;
use crate::record;
use crate::simplify;
use crate::sort::ExternalSorter;
use crate::terrain::{self, TerrainMesh};
use crate::tile_build::{self, EncoderFeature, EncoderLayer};
use crate::tileid;
use crate::tileset::{self, LayerInfo, TilesetInfo};

/// Attribute columns pulled from inputs — a superset across Overture and
/// Natural Earth. Absent columns are skipped, so one list serves both.
const ATTRS: &[&str] = &[
    "id",
    "type",
    "subtype",
    "class",
    "subclass",
    "cartography.min_zoom",
    "cartography.max_zoom",
    "cartography.sort_key",
];

/// Default geometric error at level 0, in metres.
const DEFAULT_ROOT_ERROR: f64 = 512_000.0;

/// Terrain mesh resolution (cells per side). Flat grid for now (FORMAT.md §9);
/// the client requires a terrain mesh to render a tile at all.
const TERRAIN_GRID: u32 = 16;

/// Pipeline configuration.
pub struct Config {
    pub output: PathBuf,
    /// `(layer index, GeoParquet path)` inputs.
    pub inputs: Vec<(u8, PathBuf)>,
    pub bbox: Bounds,
    pub min_zoom: u8,
    pub max_zoom: u8,
    pub tmp_dir: PathBuf,
    pub mem_budget: usize,
    /// Optional Terrarium PMTiles DEM (e.g. Mapterhorn `planet.pmtiles`). When
    /// set, each tile's terrain mesh carries real elevation sampled from it;
    /// otherwise every tile gets the shared flat mesh.
    pub terrain: Option<PathBuf>,
}

/// Summary counts from a run.
#[derive(Debug, Default, Clone, Copy)]
pub struct Stats {
    pub features_read: u64,
    pub records: u64,
    pub tiles_written: u64,
    /// Per-zoom emissions skipped because the simplified feature was smaller
    /// than one screen pixel at that zoom (sub-visible).
    pub dropped_subpixel: u64,
}

type Error = Box<dyn std::error::Error>;

/// Runs the full pipeline, writing the `.arpa` archive to `cfg.output`.
pub fn run(cfg: &Config) -> Result<Stats, Error> {
    let mut stats = Stats::default();
    let mut sorter = ExternalSorter::new(&cfg.tmp_dir, cfg.mem_budget);
    let mut records: u64 = 0;

    // --- Phase 1: read → profile → simplify → clip → sort records ---
    for (layer, path) in &cfg.inputs {
        let gp = GeoParquet::open(path)?;
        for f in gp.read_features(ATTRS)? {
            stats.features_read += 1;
            let Some(bb) = clip::bbox(&f.geometry) else {
                continue;
            };
            if !bbox_intersects(bb, &cfg.bbox) {
                continue;
            }
            let prof = profile::profile(*layer, &f.properties, cfg.min_zoom, cfg.max_zoom);
            let zmin = prof.min_zoom.max(cfg.min_zoom);
            let zmax = prof.max_zoom.min(cfg.max_zoom);

            for z in zmin..=zmax {
                let Some(simplified) = simplify::simplify_geometry(&f.geometry, tolerance(z)) else {
                    continue;
                };
                // Drop features that would be smaller than one screen pixel at
                // this zoom: sub-visible, but otherwise the long tail of tiny
                // polygons (lakes, land-cover patches) bloats low-zoom tiles.
                if is_subpixel(&simplified, z) {
                    stats.dropped_subpixel += 1;
                    continue;
                }
                let mut add_err: Option<std::io::Error> = None;
                clip::assign_tiles(&simplified, z, |x, y, clipped| {
                    if add_err.is_some() {
                        return;
                    }
                    let tb = Bounds::of_tile(z, x, y);
                    if !bbox_intersects((tb.west, tb.south, tb.east, tb.north), &cfg.bbox) {
                        return;
                    }
                    let feat = EncoderFeature {
                        id: 0,
                        geometry: clipped,
                        properties: prof.properties.clone(),
                    };
                    let key = tileid::sort_key_for(z, x, y, *layer, prof.rank);
                    match sorter.add(key, &record::encode(&feat)) {
                        Ok(()) => records += 1,
                        Err(e) => add_err = Some(e),
                    }
                });
                if let Some(e) = add_err {
                    return Err(e.into());
                }
            }
        }
    }
    stats.records = records;

    // --- Phase 2: sorted stream → group by tile → encode → archive ---
    let sorted = sorter.into_sorted()?;
    let meta = ArchiveMeta {
        min_zoom: cfg.min_zoom,
        max_zoom: cfg.max_zoom,
        bounds: cfg.bbox,
        root_error: DEFAULT_ROOT_ERROR,
    };
    // Write to a temp file and rename on success, so the output path only ever
    // appears once the archive is complete (an interrupted run can't leave a
    // valid-looking but truncated file behind).
    let tmp_output = temp_output_path(&cfg.output);
    let mut tmp_cleanup = TempCleanup { path: tmp_output.clone(), armed: true };
    let mut writer = ArchiveWriter::new(File::create(&tmp_output)?, meta)?;
    let mut layer_stats = LayerStats::new();
    // One flat terrain mesh, reused for every tile when no DEM is configured
    // (identical in quantized space). With a DEM, each tile gets its own mesh.
    let flat = terrain::flat_mesh(TERRAIN_GRID);
    let mut dem = match &cfg.terrain {
        Some(path) => Some(Dem::open(path)?),
        None => None,
    };
    // Min/max sampled elevation in metres, for the tileset's elevation range.
    let mut elevation = (f64::INFINITY, f64::NEG_INFINITY);

    let mut current: Option<u64> = None;
    let mut buckets: Vec<Vec<EncoderFeature>> = (0..layers::COUNT).map(|_| Vec::new()).collect();
    for rec in sorted {
        let (key, payload) = rec?;
        let tile_id = tileid::key_tile_id(key);
        if current != Some(tile_id) {
            if let Some(prev) = current {
                flush_tile(&mut writer, prev, &mut buckets, &mut layer_stats, &mut stats, &flat, &mut dem, &mut elevation)?;
            }
            current = Some(tile_id);
        }
        let layer = tileid::key_layer(key) as usize;
        if layer < buckets.len() {
            buckets[layer].push(record::decode(&payload)?);
        }
    }
    if let Some(prev) = current {
        flush_tile(&mut writer, prev, &mut buckets, &mut layer_stats, &mut stats, &flat, &mut dem, &mut elevation)?;
    }

    let elevation_range = if elevation.0.is_finite() { elevation } else { (0.0, 0.0) };
    let info = TilesetInfo {
        name: None,
        bounds: cfg.bbox,
        elevation_range,
        min_level: cfg.min_zoom,
        max_level: cfg.max_zoom,
        root_error: DEFAULT_ROOT_ERROR,
        layers: layer_stats.into_layer_infos(),
    };
    let file = writer.finish(&tileset::build(&info))?;
    file.sync_all()?;
    drop(file);
    std::fs::rename(&tmp_output, &cfg.output)?;
    tmp_cleanup.armed = false;
    Ok(stats)
}

/// Sibling temp path for atomic output (same directory → rename is atomic).
fn temp_output_path(output: &std::path::Path) -> PathBuf {
    let mut name = output.as_os_str().to_os_string();
    name.push(".tmp");
    PathBuf::from(name)
}

/// Removes the temp output on drop unless disarmed after the final rename, so
/// an error anywhere in phase 2 doesn't leave a stale `.tmp` file behind.
struct TempCleanup {
    path: PathBuf,
    armed: bool,
}

impl Drop for TempCleanup {
    fn drop(&mut self) {
        if self.armed {
            let _ = std::fs::remove_file(&self.path);
        }
    }
}

/// Encodes one tile's grouped features and appends it to the archive.
#[allow(clippy::too_many_arguments)]
fn flush_tile(
    writer: &mut ArchiveWriter<File>,
    tile_id: u64,
    buckets: &mut [Vec<EncoderFeature>],
    layer_stats: &mut LayerStats,
    stats: &mut Stats,
    flat: &TerrainMesh,
    dem: &mut Option<Dem>,
    elevation: &mut (f64, f64),
) -> Result<(), Error> {
    let (z, x, y) = hilbert::tile_id_decode(tile_id);
    let bounds = Bounds::of_tile(z, x, y);

    // Vector layers in decode-priority (index) order.
    let mut enc_layers = Vec::new();
    for (idx, bucket) in buckets.iter_mut().enumerate() {
        if bucket.is_empty() {
            continue;
        }
        let features = std::mem::take(bucket);
        for f in &features {
            layer_stats.observe(idx, z, geom_type(&f.geometry));
        }
        if let Some(name) = layers::name(idx as u8) {
            enc_layers.push(EncoderLayer { name: name.to_string(), features });
        }
    }

    // Every tile carries a terrain mesh — the client requires it to render.
    // With a DEM, build a per-tile elevated mesh (and track the elevation
    // range); otherwise reuse the shared flat mesh.
    layer_stats.observe(layers::TERRAIN as usize, z, GeometryType::Mesh);
    let blob = match dem {
        Some(d) => {
            let (mesh, emin, emax) =
                terrain::elevated_mesh(TERRAIN_GRID, &bounds, |lon, lat| d.elevation(lon, lat, z));
            elevation.0 = elevation.0.min(emin);
            elevation.1 = elevation.1.max(emax);
            tile_build::build_tile(&bounds, Some(&mesh), &enc_layers)
        }
        None => tile_build::build_tile(&bounds, Some(flat), &enc_layers),
    };
    writer.add_tile(z, x, y, &blob)?;
    stats.tiles_written += 1;
    Ok(())
}

/// Simplification tolerance for a zoom, in degrees: roughly one screen pixel
/// when the tile is shown at ~512 px. Quantization preserves detail down to
/// ~1/32768 of a tile — far finer than the screen — so keeping that would bloat
/// low-zoom tiles (the whole world's coastline in a single z0 tile).
fn tolerance(z: u8) -> f64 {
    let tile_w = 360.0 / (1u64 << z as u32) as f64; // 2^z columns
    tile_w / 512.0
}

/// True when a simplified geometry is smaller than one screen pixel at `z` —
/// sub-visible, so it's dropped rather than emitted. Polygons are judged by
/// area (< 1 px²), lines by length (< 1 px); points are always kept. The pixel
/// size is `tolerance(z)`, matching the simplification scale, so anything kept
/// is at least a pixel in some dimension.
fn is_subpixel(geom: &Geometry, z: u8) -> bool {
    let px = tolerance(z); // degrees per screen pixel at this zoom
    match geom {
        Geometry::Polygon(_) | Geometry::MultiPolygon(_) => simplify::area(geom) < px * px,
        Geometry::LineString(_) | Geometry::MultiLineString(_) => simplify::length(geom) < px,
        _ => false,
    }
}

fn bbox_intersects(bb: (f64, f64, f64, f64), b: &Bounds) -> bool {
    bb.0 <= b.east && bb.2 >= b.west && bb.1 <= b.north && bb.3 >= b.south
}

fn geom_type(g: &Geometry) -> GeometryType {
    match g {
        Geometry::Point(_) | Geometry::MultiPoint(_) => GeometryType::Point,
        Geometry::LineString(_) | Geometry::MultiLineString(_) => GeometryType::Line,
        _ => GeometryType::Polygon,
    }
}

/// Accumulates per-layer presence, zoom range, and geometry types for `.arpi`.
struct LayerStats {
    rows: Vec<LayerStat>,
}

#[derive(Default)]
struct LayerStat {
    present: bool,
    min_z: u8,
    max_z: u8,
    geoms: Vec<GeometryType>,
}

impl LayerStats {
    fn new() -> Self {
        LayerStats { rows: (0..layers::COUNT).map(|_| LayerStat::default()).collect() }
    }

    fn observe(&mut self, layer: usize, z: u8, gt: GeometryType) {
        let row = &mut self.rows[layer];
        if !row.present {
            row.present = true;
            row.min_z = z;
            row.max_z = z;
        } else {
            row.min_z = row.min_z.min(z);
            row.max_z = row.max_z.max(z);
        }
        if !row.geoms.contains(&gt) {
            row.geoms.push(gt);
        }
    }

    fn into_layer_infos(self) -> Vec<LayerInfo> {
        self.rows
            .into_iter()
            .enumerate()
            .filter(|(_, r)| r.present)
            .filter_map(|(idx, r)| {
                layers::name(idx as u8).map(|name| LayerInfo {
                    name: name.to_string(),
                    geometry_types: r.geoms,
                    min_level: r.min_z,
                    max_level: r.max_z,
                })
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fb::tile::arpentry::tiles as fbt;
    use crate::fb::tileset::arpentry::tiles as fbts;

    fn brotli_decompress(data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut input = data;
        brotli::BrotliDecompress(&mut input, &mut out).unwrap();
        out
    }

    /// Dumps polygon structure of one layer in one tile from `$ARPA`, for
    /// comparing the Rust and C tilers. Env: ARPA, Z, X, Y, LAYER (default land).
    /// Run: `ARPA=/tmp/c.arpa Z=3 X=4 Y=5 cargo test -- --ignored dump_tile_polys --nocapture`
    #[test]
    #[ignore = "needs $ARPA"]
    fn dump_tile_polys() {
        let path = std::env::var("ARPA").unwrap();
        let z: u8 = std::env::var("Z").unwrap().parse().unwrap();
        let x: u32 = std::env::var("X").unwrap().parse().unwrap();
        let y: u32 = std::env::var("Y").unwrap().parse().unwrap();
        let want = std::env::var("LAYER").unwrap_or_else(|_| "land".to_string());

        let bytes = std::fs::read(&path).unwrap();
        let archive = crate::archive::Archive::open(&bytes).unwrap();
        let id = crate::hilbert::tile_id(z, x, y);
        let Some(blob) = archive.get_by_id(id) else {
            eprintln!("{path}: tile {z}/{x}/{y} NOT PRESENT");
            return;
        };
        let raw = brotli_decompress(blob);
        let tile = fbt::root_as_tile(&raw).unwrap();
        eprintln!("== {path}  tile {z}/{x}/{y}  layer '{want}' ==");
        let layers = tile.layers().unwrap();
        for li in 0..layers.len() {
            let l = layers.get(li);
            if l.name() != want {
                continue;
            }
            let feats = l.features().unwrap();
            eprintln!("features: {}", feats.len());
            for fi in 0..feats.len().min(4) {
                let f = feats.get(fi);
                let Some(pg) = f.geometry_as_polygon_geometry() else { continue };
                let xs = pg.x();
                let ys = pg.y();
                let ro: Option<Vec<u32>> = pg.ring_offsets().map(|v| v.iter().collect());
                let po: Option<Vec<u32>> = pg.polygon_offsets().map(|v| v.iter().collect());
                let ring_end = ro.as_ref().map(|v| v[1] as usize).unwrap_or(xs.len());
                let mut area = 0i64;
                for k in 0..ring_end {
                    let j = (k + 1) % ring_end;
                    area += xs.get(k) as i64 * ys.get(j) as i64 - xs.get(j) as i64 * ys.get(k) as i64;
                }
                eprintln!(
                    "feat {fi}: verts={} rings={:?} polys={:?} ring0[n={} first=({},{}) last=({},{}) wind={}]",
                    xs.len(),
                    ro.as_ref().map(|v| v.len()),
                    po.as_ref().map(|v| v.len()),
                    ring_end,
                    xs.get(0), ys.get(0),
                    xs.get(ring_end - 1), ys.get(ring_end - 1),
                    if area >= 0 { "CCW" } else { "CW" },
                );
            }
        }
    }

    /// Dumps the structure of a tile from an existing archive — layers, feature
    /// counts, first geometry, and class values — to debug client rendering.
    /// Run: `cargo test -- --ignored dump_archive_tile --nocapture`
    #[test]
    #[ignore = "inspects ./naturalearth.arpa if present"]
    fn dump_archive_tile() {
        let bytes = std::fs::read("naturalearth.arpa").expect("naturalearth.arpa in cwd");
        let archive = crate::archive::Archive::open(&bytes).unwrap();
        eprintln!("archive: {} tiles, zoom {}-{}", archive.tile_count(), archive.min_zoom(), archive.max_zoom());

        // First tile in the directory.
        let entry = archive.entries().next().expect("a tile");
        eprintln!("tile z{} x{} y{} size={}", entry.z, entry.x, entry.y, entry.size);
        let raw = brotli_decompress(archive.get_by_id(entry.hilbert_id).unwrap());
        let tile = fbt::root_as_tile(&raw).unwrap();

        let keys: Vec<&str> = tile.keys().map(|k| k.iter().collect()).unwrap_or_default();
        eprintln!("keys: {keys:?}");

        let layers = tile.layers().expect("layers");
        for li in 0..layers.len() {
            let layer = layers.get(li);
            let feats = layer.features();
            let n = feats.map(|f| f.len()).unwrap_or(0);
            eprintln!("layer '{}': {} features", layer.name(), n);
            if let Some(feats) = feats {
                if feats.len() > 0 {
                    let f = feats.get(0);
                    let gt = f.geometry_type();
                    let vc = f.geometry_as_polygon_geometry().map(|p| p.x().len());
                    // class value of first feature
                    let mut class = None;
                    if let Some(props) = f.properties() {
                        for pi in 0..props.len() {
                            let p = props.get(pi);
                            if keys.get(p.key() as usize) == Some(&"class") {
                                if let Some(vals) = tile.values() {
                                    class = vals.get(p.value() as usize).string_value().map(|s| s.to_string());
                                }
                            }
                        }
                    }
                    eprintln!("  first feat: geom_type={gt:?} polygon_x_len={vc:?} class={class:?}");
                }
            }
        }
    }

    /// Breaks down specific tiles in $ARPA by layer: feature count and total
    /// vertices, to find what bloats a tile. Env: ARPA, TILES="z/x/y,z/x/y".
    /// Run: `ARPA=data/overture-ch/switzerland.arpa TILES=7/66/97,7/65/98 \
    ///   cargo test -- --ignored dump_tile_sizes --nocapture`
    #[test]
    #[ignore = "needs $ARPA"]
    fn dump_tile_sizes() {
        let path = std::env::var("ARPA").unwrap();
        let tiles = std::env::var("TILES").unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let archive = crate::archive::Archive::open(&bytes).unwrap();
        for spec in tiles.split(',') {
            let parts: Vec<&str> = spec.split('/').collect();
            let z: u8 = parts[0].parse().unwrap();
            let x: u32 = parts[1].parse().unwrap();
            let y: u32 = parts[2].parse().unwrap();
            let id = crate::hilbert::tile_id(z, x, y);
            let Some(blob) = archive.get_by_id(id) else {
                eprintln!("{z}/{x}/{y}: NOT PRESENT");
                continue;
            };
            let raw = brotli_decompress(blob);
            let tile = fbt::root_as_tile(&raw).unwrap();
            eprintln!("== {z}/{x}/{y}: compressed={} decompressed={} ==", blob.len(), raw.len());
            let layers = tile.layers().unwrap();
            for li in 0..layers.len() {
                let l = layers.get(li);
                let Some(feats) = l.features() else { continue };
                let mut verts = 0usize;
                let mut max_verts = 0usize;
                for fi in 0..feats.len() {
                    let f = feats.get(fi);
                    let v = if let Some(g) = f.geometry_as_polygon_geometry() {
                        g.x().len()
                    } else if let Some(g) = f.geometry_as_line_geometry() {
                        g.x().len()
                    } else if let Some(g) = f.geometry_as_point_geometry() {
                        g.x().len()
                    } else if let Some(g) = f.geometry_as_mesh_geometry() {
                        g.x().len()
                    } else { 0 };
                    verts += v;
                    max_verts = max_verts.max(v);
                }
                eprintln!("  layer '{}': {} features, {} verts total, {} max/feat",
                    l.name(), feats.len(), verts, max_verts);
            }
        }
    }

    /// Full read→sort→encode→archive chain over the repo's Natural Earth data,
    /// reopening the `.arpa` and decoding its metadata and a tile.
    #[test]
    #[ignore = "requires repo sample data under ../data"]
    fn natural_earth_end_to_end() {
        let out = std::env::temp_dir().join(format!("arpt-e2e-{}.arpa", std::process::id()));
        let cfg = Config {
            output: out.clone(),
            inputs: vec![
                (layers::LAND, "../data/naturalearth/land.parquet".into()),
                (layers::WATER, "../data/naturalearth/lake.parquet".into()),
            ],
            bbox: Bounds { west: 5.9, south: 45.8, east: 10.5, north: 47.9 },
            min_zoom: 0,
            max_zoom: 6,
            tmp_dir: std::env::temp_dir(),
            mem_budget: 64 * 1024 * 1024,
            terrain: None,
        };
        let stats = run(&cfg).expect("pipeline run");
        assert!(stats.tiles_written > 0, "expected some tiles");
        // Atomic output: the temp file is renamed away on success.
        assert!(!temp_output_path(&out).exists(), "temp file should be gone");

        let bytes = std::fs::read(&out).unwrap();
        std::fs::remove_file(&out).ok();

        let archive = crate::archive::Archive::open(&bytes).expect("open archive");
        assert_eq!(archive.tile_count(), stats.tiles_written);
        assert_eq!(archive.min_zoom(), 0);
        assert_eq!(archive.max_zoom(), 6);

        // Metadata is a valid Tileset with at least one layer.
        let meta = brotli_decompress(archive.metadata());
        let ts = fbts::root_as_tileset(&meta).expect("tileset");
        assert!(ts.layers().map(|l| l.len()).unwrap_or(0) > 0);

        // A tile blob decodes as a Tile with at least one layer.
        let entry = archive.entries().next().expect("a directory entry");
        let blob = archive.get_by_id(entry.hilbert_id).expect("tile blob");
        let raw = brotli_decompress(blob);
        let tile = fbt::root_as_tile(&raw).expect("tile");
        assert_eq!(tile.version(), 1);
        assert!(tile.layers().map(|l| l.len()).unwrap_or(0) > 0);
    }
}
