//! Top-level pipeline — stage 5 of docs/GENERATION.md §6, hosting stages 1–4.
//!
//! [`run`] first builds the global world model — assemble the scene graph
//! (stage 1), solve the vertical model (stage 2), derive the engineered
//! ground (stage 3) — then runs the sort-based tiling: read features →
//! profile → per zoom simplify + clip to tiles → serialize sort records keyed
//! by Hilbert tile id → external merge sort → group by tile → synthesize
//! geometry from the solved model (stage 4) → build `.arpt` → write `.arpa`
//! archive (+ `.arpi` metadata).
//!
//! Every height an emit worker writes is a function of the shared solved
//! model (`Arc<SolvedModel>`, `Arc<GroundModel>`) and the global terrain
//! lattice — never of the tile window — so adjacent tiles and successive
//! zooms agree by construction (invariant 5) and tiling carries no modeling
//! responsibility.
//!
//! Parallel and dependency-free (`std::thread` + channels). Phase 1 fans
//! row-group work items out to workers that feed per-worker external sorters;
//! phase 2 groups the merged stream into per-tile jobs encoded by a worker
//! pool and written back in stream order. `Config::threads == 1` runs both
//! phases serially on the calling thread.

use std::collections::{HashMap, VecDeque};
use std::fs::File;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use geo_types::{Geometry, LineString};

use crate::archive::{ArchiveMeta, ArchiveWriter};
use crate::assemble;
use crate::clip;
use crate::dem::Dem;
use crate::dump;
use crate::geom::GeometryType;
use crate::geoparquet::GeoParquet;
use crate::ground::{self, sampler::GroundSampler, GroundModel};
use crate::hilbert;
use crate::layers;
use crate::profile;
use crate::project::Bounds;
use crate::record;
use crate::scene::{source_hash, SceneGraph, Span, SpanKind};
use crate::simplify;
use crate::solve::{self, SolvedModel};
use crate::sort::{self, ExternalSorter};
use crate::synth::junction::JunctionModel;
use crate::synth::{self, Synth};
use crate::terrain::{self, TerrainMesh, TERRAIN_GRID};
use crate::tile_build::{self, EncoderFeature, EncoderLayer};
use crate::tileid;
use crate::tileset::{self, LayerInfo, TilesetInfo};
use crate::value::Value;

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

/// Attribute columns per layer. Buildings and POIs need columns the shared
/// list doesn't cover (heights, names, categories); keeping them per-layer
/// also avoids decoding heavy root structs (Overture `names`) for the layers
/// that don't use them.
fn attrs_for(layer: u8) -> &'static [&'static str] {
    match layer {
        layers::BUILDING => {
            &["id", "subtype", "class", "height", "num_floors", "roof_shape", "roof_height"]
        }
        layers::POI => &["id", "names.primary", "basic_category", "confidence"],
        // Road names feed the client's line-following street labels;
        // `level_rules` carries the bridge/tunnel level (FORMAT.md reserved
        // `level`) so the client lifts bridges and sinks tunnels. The
        // horizontal attributes (docs/ROADS.md P1) reduce to scalars at
        // decode: `width_rules` refines the painted width, `road_surface`
        // and the `access_restrictions` one-way verdict ride along for
        // styling and the marking phases.
        layers::TRANSPORTATION => &[
            "id",
            "type",
            "subtype",
            "class",
            "subclass",
            "names.primary",
            "level_rules",
            "width_rules",
            "road_surface",
            "access_restrictions",
            "cartography.min_zoom",
            "cartography.max_zoom",
            "cartography.sort_key",
        ],
        _ => ATTRS,
    }
}

/// Default geometric error at level 0, in metres.
const DEFAULT_ROOT_ERROR: f64 = 512_000.0;

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
    /// Phase-1 worker threads (0 = available CPU cores).
    pub threads: usize,
    /// Brotli quality (0–11) for tile blobs.
    pub brotli_quality: i32,
    /// Directory for stage-artifact GeoJSON dumps (scene graph, solved
    /// profiles), for inspection in QGIS/kepler; `None` skips them.
    pub dump: Option<PathBuf>,
}

/// Summary counts from a run.
#[derive(Debug, Default, Clone, Copy)]
pub struct Stats {
    pub features_read: u64,
    pub records: u64,
    /// Total bytes of encoded sort-record payloads (sizes the external sort).
    pub record_bytes: u64,
    pub tiles_written: u64,
    /// Per-zoom emissions skipped because the simplified feature was smaller
    /// than one screen pixel at that zoom (sub-visible).
    pub dropped_subpixel: u64,
    /// Row groups actually read vs. present across all inputs — the difference
    /// is what bbox statistics pruning skipped.
    pub row_groups_read: u64,
    pub row_groups_total: u64,
    /// Corridors assembled from the transportation input (stage 1), how many
    /// of them carry a solved elevation profile (stage 2), the crossings
    /// detected between them and the network, and the earthwork edges the
    /// engineered ground carries (stage 3).
    pub corridors: u64,
    pub profiles: u64,
    pub crossings: u64,
    pub earthworks: u64,
    pub water: u64,
    pub junction_plates: u64,
    /// Phase-1 worker threads used.
    pub threads: usize,
    pub timings: Timings,
}

/// Time accumulated in each pipeline stage. Per-stage times are summed across
/// worker threads, so with N workers they can exceed the phase wall time —
/// read them as CPU seconds. The phase totals, `merge`, and `write` are
/// wall-clock.
#[derive(Debug, Default, Clone, Copy)]
pub struct Timings {
    /// World-model stages before tiling: assemble + solve (stages 1–2).
    pub model: Duration,
    /// Parquet open + Arrow decode + WKB parse → in-memory features.
    pub read: Duration,
    /// Per-zoom Douglas–Peucker simplification.
    pub simplify: Duration,
    /// Tile assignment + rectangle clipping.
    pub clip: Duration,
    /// Sort-record encoding + sorter insertion (including run spills).
    pub sort: Duration,
    /// K-way merge of sorted runs (pulling the sorted stream).
    pub merge: Duration,
    /// Sort-record decoding back into features.
    pub decode: Duration,
    /// DEM sampling + terrain mesh construction.
    pub terrain: Duration,
    /// FlatBuffer tile assembly + Brotli compression.
    pub encode: Duration,
    /// Archive output writes.
    pub write: Duration,
    /// End-to-end phase totals.
    pub phase1: Duration,
    pub phase2: Duration,
}

type Error = Box<dyn std::error::Error + Send + Sync>;

/// One unit of phase-1 work: a single row group of one input.
struct WorkItem {
    /// Index into the opened inputs.
    input: usize,
    row_group: usize,
}

/// Runs the full pipeline, writing the `.arpa` archive to `cfg.output`.
///
/// Phase 1 fans row groups out to worker threads, each feeding its own
/// external sorter; the sorters merge into one stream for phase 2. Records
/// sharing a sort key may interleave differently between runs (workers race),
/// but tile contents are otherwise identical.
pub fn run(cfg: &Config) -> Result<Stats, Error> {
    let mut stats = Stats::default();

    let threads = match cfg.threads {
        0 => std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1),
        n => n,
    };
    stats.threads = threads;

    // --- Stages 1–3: the global world model, built once before tiling ---
    // Assemble the scene graph from the transportation input, solve the
    // vertical model against the DEM at the reference zoom (the run's max —
    // the lattice the client sees close up), and derive the engineered ground
    // (still the raw DEM in this milestone). Everything after reads these
    // through Arcs; no height ever depends on a tile window.
    let t_model = Instant::now();
    let transportation =
        cfg.inputs.iter().find(|(l, _)| *l == layers::TRANSPORTATION).map(|(_, p)| p.clone());
    let water = cfg.inputs.iter().find(|(l, _)| *l == layers::WATER).map(|(_, p)| p.clone());
    let scene = match &transportation {
        Some(path) => assemble::run(path, water.as_deref(), &cfg.bbox)
            .map_err(|e| format!("{}: {e}", path.display()))?,
        None => SceneGraph::default(),
    };
    let solved = Arc::new(solve::run(&scene, cfg.terrain.as_deref(), cfg.max_zoom, threads)?);
    let ground = Arc::new(ground::derive(&scene, &solved, cfg.terrain.as_deref(), threads));
    // Junction plates: a paved area meshed across each corridor junction, baked
    // once from the solved model and emitted by the tile that owns its centre.
    let junctions = Arc::new(synth::junction::bake(&scene, &solved));
    stats.corridors = scene.corridors.len() as u64;
    stats.profiles = solved.solved_count() as u64;
    stats.crossings = scene.crossings.len() as u64;
    stats.earthworks = ground.earthwork_count() as u64;
    stats.water = ground.water_count() as u64;
    stats.junction_plates = junctions.len() as u64;
    stats.timings.model = t_model.elapsed();
    if let Some(dir) = &cfg.dump {
        dump::write(dir, &scene, &solved, &ground)?;
    }

    // --- Phase 1: read → profile → simplify → clip → sort records ---
    // Open every input (footer only) and queue its bbox-intersecting row
    // groups as work items.
    let phase1_start = Instant::now();
    let mut inputs: Vec<(u8, GeoParquet)> = Vec::new();
    let mut queue: VecDeque<WorkItem> = VecDeque::new();
    for (layer, path) in &cfg.inputs {
        let gp = GeoParquet::open(path)
            .map_err(|e| format!("{}: {e}", path.display()))?;
        let row_groups =
            gp.row_groups_intersecting((cfg.bbox.west, cfg.bbox.south, cfg.bbox.east, cfg.bbox.north));
        stats.row_groups_total += gp.num_row_groups() as u64;
        stats.row_groups_read += row_groups.len() as u64;
        let input = inputs.len();
        queue.extend(row_groups.into_iter().map(|row_group| WorkItem { input, row_group }));
        inputs.push((*layer, gp));
    }

    // Phase 1 parallelism is bounded by its work items (row groups); the emit
    // pool below is not.
    let phase1_threads = threads.min(queue.len().max(1));
    // Workers split the sort budget; keep a sane floor so tiny budgets still
    // make progress without spilling every record.
    let worker_budget = (cfg.mem_budget / phase1_threads).max(1 << 20);

    let queue = Mutex::new(queue);
    let mut sorters: Vec<ExternalSorter> = Vec::with_capacity(phase1_threads);
    if phase1_threads == 1 {
        let (sorter, partial) =
            phase1_worker(&inputs, &queue, cfg, worker_budget, &scene, &solved, &junctions)?;
        merge_phase1(&mut stats, &partial);
        sorters.push(sorter);
    } else {
        std::thread::scope(|scope| -> Result<(), Error> {
            let mut handles = Vec::with_capacity(phase1_threads);
            for _ in 0..phase1_threads {
                handles.push(scope.spawn(|| {
                    phase1_worker(&inputs, &queue, cfg, worker_budget, &scene, &solved, &junctions)
                }));
            }
            for handle in handles {
                let (sorter, partial) =
                    handle.join().map_err(|_| "phase-1 worker panicked")??;
                merge_phase1(&mut stats, &partial);
                sorters.push(sorter);
            }
            Ok(())
        })?;
    }
    stats.timings.phase1 = phase1_start.elapsed();

    // --- Phase 2: sorted stream → group by tile → encode → archive ---
    let phase2_start = Instant::now();
    let t_finish = Instant::now();
    let sorted = sort::merge(sorters)?;
    stats.timings.merge += t_finish.elapsed();
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
    // Min/max sampled elevation in metres, for the tileset's elevation range.
    let mut elevation = (f64::INFINITY, f64::NEG_INFINITY);

    if threads == 1 {
        // One flat terrain mesh, reused for every tile when no DEM is
        // configured (identical in quantized space). With a DEM, each tile
        // gets its own mesh.
        let flat = terrain::flat_mesh(TERRAIN_GRID);
        let dem = match &cfg.terrain {
            Some(path) => Some(Dem::open(path)?),
            None => None,
        };
        let mut sampler = GroundSampler::new(dem, Arc::clone(&ground));
        let mut sorted = sorted;
        let mut current: Option<u64> = None;
        let mut buckets: Vec<Vec<EncoderFeature>> =
            (0..layers::COUNT).map(|_| Vec::new()).collect();
        loop {
            let t_merge = Instant::now();
            let rec = sorted.next();
            stats.timings.merge += t_merge.elapsed();
            let Some(rec) = rec else {
                break;
            };
            let (key, payload) = rec?;
            let tile_id = tileid::key_tile_id(key);
            if current != Some(tile_id) {
                if let Some(prev) = current {
                    flush_tile(&mut writer, prev, &mut buckets, &mut layer_stats, &mut stats, &flat, &mut sampler, &solved, &junctions, &mut elevation, cfg.brotli_quality)?;
                }
                current = Some(tile_id);
            }
            let layer = tileid::key_layer(key) as usize;
            if layer < buckets.len() {
                let t_decode = Instant::now();
                let decoded = record::decode(&payload)?;
                stats.timings.decode += t_decode.elapsed();
                buckets[layer].push(decoded);
            }
        }
        if let Some(prev) = current {
            flush_tile(&mut writer, prev, &mut buckets, &mut layer_stats, &mut stats, &flat, &mut sampler, &solved, &junctions, &mut elevation, cfg.brotli_quality)?;
        }
    } else {
        emit_parallel(
            cfg,
            sorted,
            threads,
            &mut writer,
            &mut layer_stats,
            &mut stats,
            &mut elevation,
            &solved,
            &ground,
            &junctions,
        )?;
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
    let t_write = Instant::now();
    let file = writer.finish(&tileset::build(&info))?;
    file.sync_all()?;
    drop(file);
    std::fs::rename(&tmp_output, &cfg.output)?;
    tmp_cleanup.armed = false;
    stats.timings.write += t_write.elapsed();
    stats.timings.phase2 = phase2_start.elapsed();
    Ok(stats)
}

/// One tile's worth of consecutive sorted records, ready to encode.
struct TileJob {
    /// Position in the sorted tile stream; the writer restores this order.
    seq: u64,
    tile_id: u64,
    /// `(layer index, record payload)` in sorted (rank) order.
    records: Vec<(usize, Vec<u8>)>,
}

/// An encoded tile coming back from an emit worker.
struct TileResult {
    seq: u64,
    z: u8,
    x: u32,
    y: u32,
    blob: Vec<u8>,
    /// `(min, max)` elevation sampled from the DEM, when one is configured.
    elevation: Option<(f64, f64)>,
    /// Distinct `(layer index, geometry type)` pairs seen, for `.arpi` stats.
    observed: Vec<(usize, GeometryType)>,
    decode: Duration,
    terrain: Duration,
    encode: Duration,
}

/// Min-heap adapter ordering tile results by sequence number.
struct PendingTile(TileResult);
impl PartialEq for PendingTile {
    fn eq(&self, other: &Self) -> bool {
        self.0.seq == other.0.seq
    }
}
impl Eq for PendingTile {}
impl Ord for PendingTile {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.0.seq.cmp(&self.0.seq) // inverted: BinaryHeap pops smallest seq
    }
}
impl PartialOrd for PendingTile {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// Parallel phase-2 emit: a dispatcher thread groups the sorted stream into
/// per-tile jobs, `threads` workers decode + mesh + encode + compress them,
/// and this thread writes the results back in stream order (a small seq-keyed
/// heap reorders out-of-order completions). Workers own their DEM readers;
/// the Hilbert-ordered stream keeps their tile caches hot.
#[allow(clippy::too_many_arguments)]
fn emit_parallel(
    cfg: &Config,
    sorted: sort::Sorted,
    threads: usize,
    writer: &mut ArchiveWriter<File>,
    layer_stats: &mut LayerStats,
    stats: &mut Stats,
    elevation: &mut (f64, f64),
    solved: &Arc<SolvedModel>,
    ground: &Arc<GroundModel>,
    junctions: &Arc<JunctionModel>,
) -> Result<(), Error> {
    use std::sync::mpsc;

    let (job_tx, job_rx) = mpsc::sync_channel::<TileJob>(threads * 2);
    let job_rx = Arc::new(Mutex::new(job_rx));
    let (result_tx, result_rx) = mpsc::sync_channel::<Result<TileResult, Error>>(threads * 2);

    std::thread::scope(|scope| -> Result<(), Error> {
        // Dispatcher: pull the k-way merge, group consecutive records by tile.
        let dispatcher = scope.spawn(move || -> std::io::Result<Duration> {
            let mut sorted = sorted;
            let mut merge_time = Duration::ZERO;
            let mut seq = 0u64;
            let mut current: Option<u64> = None;
            let mut records: Vec<(usize, Vec<u8>)> = Vec::new();
            loop {
                let t_merge = Instant::now();
                let rec = sorted.next();
                merge_time += t_merge.elapsed();
                match rec {
                    Some(Ok((key, payload))) => {
                        let tile_id = tileid::key_tile_id(key);
                        if current != Some(tile_id) {
                            if let Some(prev) = current {
                                let job = TileJob {
                                    seq,
                                    tile_id: prev,
                                    records: std::mem::take(&mut records),
                                };
                                // Send fails only on receiver shutdown (error
                                // elsewhere) → stop dispatching.
                                if job_tx.send(job).is_err() {
                                    break;
                                }
                                seq += 1;
                            }
                            current = Some(tile_id);
                        }
                        let layer = tileid::key_layer(key) as usize;
                        if layer < layers::COUNT {
                            records.push((layer, payload));
                        }
                    }
                    Some(Err(e)) => return Err(e),
                    None => {
                        if let Some(prev) = current {
                            let job =
                                TileJob { seq, tile_id: prev, records: std::mem::take(&mut records) };
                            let _ = job_tx.send(job);
                        }
                        break;
                    }
                }
            }
            Ok(merge_time)
        });

        // Workers: decode records, build the terrain mesh, encode + compress.
        // Each holds its own DEM handle forked from one primary, so the
        // decoded-tile cache is shared; the solved model and engineered
        // ground are shared, immutable.
        let primary_dem = match &cfg.terrain {
            Some(path) => Some(Dem::open(path)?),
            None => None,
        };
        let mut workers = Vec::with_capacity(threads);
        for _ in 0..threads {
            let job_rx = Arc::clone(&job_rx);
            let result_tx = result_tx.clone();
            let solved = Arc::clone(solved);
            let ground = Arc::clone(ground);
            let junctions = Arc::clone(junctions);
            let dem = match &primary_dem {
                Some(d) => Some(d.fork()?),
                None => None,
            };
            workers.push(scope.spawn(move || -> Result<(), Error> {
                let flat = terrain::flat_mesh(TERRAIN_GRID);
                let mut sampler = GroundSampler::new(dem, ground);
                loop {
                    // Blocking recv under the lock serializes idle waits only;
                    // a queued job is handed off immediately.
                    let job = job_rx.lock().expect("emit queue poisoned").recv();
                    let Ok(job) = job else {
                        break;
                    };
                    let result =
                        encode_tile(job, &flat, &mut sampler, &solved, &junctions, cfg.brotli_quality);
                    if result_tx.send(result).is_err() {
                        break;
                    }
                }
                Ok(())
            }));
        }
        // The writer loop below must see the channel close once workers finish.
        drop(result_tx);

        // Writer (this thread): restore stream order and append to the archive.
        let mut next_seq = 0u64;
        let mut pending: std::collections::BinaryHeap<PendingTile> =
            std::collections::BinaryHeap::new();
        for result in result_rx {
            pending.push(PendingTile(result?));
            while pending.peek().is_some_and(|p| p.0.seq == next_seq) {
                let r = pending.pop().expect("peeked").0;
                for &(idx, gt) in &r.observed {
                    layer_stats.observe(idx, r.z, gt);
                }
                if let Some((emin, emax)) = r.elevation {
                    elevation.0 = elevation.0.min(emin);
                    elevation.1 = elevation.1.max(emax);
                }
                stats.timings.decode += r.decode;
                stats.timings.terrain += r.terrain;
                stats.timings.encode += r.encode;
                let t_write = Instant::now();
                writer.add_tile(r.z, r.x, r.y, &r.blob)?;
                stats.timings.write += t_write.elapsed();
                stats.tiles_written += 1;
                next_seq += 1;
            }
        }
        // Unblock a dispatcher still sending if we exited early on an error.
        drop(job_rx);
        stats.timings.merge += dispatcher.join().map_err(|_| "emit dispatcher panicked")??;
        for worker in workers {
            worker.join().map_err(|_| "emit worker panicked")??;
        }
        assert!(pending.is_empty(), "tile results missing from the emit stream");
        Ok(())
    })
}

/// Encodes one tile on an emit worker: decode its records into per-layer
/// features, synthesize geometry from the solved model (stage 4), build the
/// terrain mesh (engineered ground or flat), and produce the compressed blob
/// plus the stats the writer folds in.
fn encode_tile(
    job: TileJob,
    flat: &TerrainMesh,
    sampler: &mut GroundSampler,
    solved: &SolvedModel,
    junctions: &JunctionModel,
    quality: i32,
) -> Result<TileResult, Error> {
    let (z, x, y) = hilbert::tile_id_decode(job.tile_id);
    let bounds = Bounds::of_tile(z, x, y);

    let mut t_decode = Duration::ZERO;
    let mut buckets: Vec<Vec<EncoderFeature>> = (0..layers::COUNT).map(|_| Vec::new()).collect();
    for (layer, payload) in &job.records {
        let t = Instant::now();
        let decoded = record::decode(payload)?;
        t_decode += t.elapsed();
        buckets[*layer].push(decoded);
    }
    let t_stamp = Instant::now();
    stamp_elevations(&mut buckets, sampler, z);
    stamp_synth(&mut buckets, sampler, solved, junctions, z, &bounds);
    add_junction_plates(&mut buckets, junctions, sampler, &bounds, z, solved.z_ref);
    let mut t_terrain = t_stamp.elapsed();

    // Vector layers in decode-priority (index) order.
    let mut observed = Vec::new();
    let mut enc_layers = Vec::new();
    for (idx, bucket) in buckets.iter_mut().enumerate() {
        if bucket.is_empty() {
            continue;
        }
        let features = std::mem::take(bucket);
        for f in &features {
            let gt = geom_type(&f.geometry);
            if !observed.contains(&(idx, gt)) {
                observed.push((idx, gt));
            }
        }
        if let Some(name) = layers::name(idx as u8) {
            enc_layers.push(EncoderLayer { name: name.to_string(), features });
        }
    }

    // Every tile carries a terrain mesh — the client requires it to render.
    observed.push((layers::TERRAIN as usize, GeometryType::Mesh));
    let (blob, elevation, t_mesh, t_encode) = if sampler.has_elevation() {
        let t = Instant::now();
        let (mesh, emin, emax) =
            terrain::elevated_mesh(TERRAIN_GRID, &bounds, |lon, lat| sampler.corner(lon, lat, z));
        let t_mesh = t.elapsed();
        let t = Instant::now();
        let blob = tile_build::build_tile_q(&bounds, Some(&mesh), &enc_layers, quality);
        (blob, Some((emin, emax)), t_mesh, t.elapsed())
    } else {
        let t = Instant::now();
        let blob = tile_build::build_tile_q(&bounds, Some(flat), &enc_layers, quality);
        (blob, None, Duration::ZERO, t.elapsed())
    };
    t_terrain += t_mesh;
    Ok(TileResult {
        seq: job.seq,
        z,
        x,
        y,
        blob,
        elevation,
        observed,
        decode: t_decode,
        terrain: t_terrain,
        encode: t_encode,
    })
}

/// Tiler-computed property carrying a building's ground relief (highest minus
/// lowest terrain under its footprint, whole metres). The building mesher sinks
/// the foundation this far plus a small margin so sloped ground never reveals a
/// gap. Absent (treated as zero) on flat footprints.
const GROUND_RELIEF_KEY: &str = "ground_relief";

/// Stamps DEM base elevations onto the features that need vertical placement:
/// building footprints (mesh anchor) and POI points (label anchors). The
/// elevation is encoded as a constant per-vertex `z` array. Draped layers
/// (everything else) and DEM-less runs are untouched — their `z` stays absent
/// (zero).
///
/// Buildings anchor at the *highest* ground under the footprint, so uphill
/// terrain never swallows the walls, and carry a `ground_relief` property
/// (highest minus lowest) so the mesher extends the foundation past the lowest
/// ground (see `building_mesh`). Sampling every footprint vertex captures that
/// spread; the bbox centre alone cannot. POIs are points, so a single centre
/// sample is their anchor.
fn stamp_elevations(buckets: &mut [Vec<EncoderFeature>], sampler: &mut GroundSampler, z: u8) {
    if !sampler.has_elevation() {
        return;
    }

    for f in &mut buckets[layers::BUILDING as usize] {
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        clip::for_each_coord(&f.geometry, &mut |c| {
            let e = sampler.ground(c.x, c.y, z);
            lo = lo.min(e);
            hi = hi.max(e);
        });
        if !hi.is_finite() {
            continue;
        }
        f.elevation = Some(hi);
        let relief = (hi - lo).round();
        if relief >= 1.0 {
            f.properties.push((GROUND_RELIEF_KEY.to_string(), Value::Int(relief as i64)));
        }
    }

    for f in &mut buckets[layers::POI as usize] {
        if let Some((min_x, min_y, max_x, max_y)) = clip::bbox(&f.geometry) {
            let (lon, lat) = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);
            f.elevation = Some(sampler.ground(lon, lat, z));
        }
    }
}

/// Adds the junction plates this tile owns to its transportation bucket — a
/// paved mesh across each corridor intersection. A detail feature: coarse zooms
/// render the overlapping strokes as before (the plate positions never change,
/// only their presence). See [`synth::junction`].
fn add_junction_plates(
    buckets: &mut [Vec<EncoderFeature>],
    junctions: &JunctionModel,
    sampler: &mut GroundSampler,
    bounds: &Bounds,
    z: u8,
    z_ref: u8,
) {
    if z < crate::priors::STRUCTURE_DETAIL_MIN_ZOOM || junctions.is_empty() {
        return;
    }
    for baked in junctions.near((bounds.west, bounds.south, bounds.east, bounds.north)) {
        if let Some(f) = synth::junction::plate(baked, bounds, sampler, z, z_ref) {
            buckets[layers::TRANSPORTATION as usize].push(f);
        }
    }
}

/// Stage 4 for the tile: runs each transportation feature's generator against
/// the solved model — bridge decks and tunnel bores swept on their corridor's
/// profile, roads draped on the rendered ground (plus their corridor's solved
/// cut/fill where one exists). At detail zooms each at-grade drivable road
/// also gains its surface band, built from the freshly baked centerline and
/// trimmed back at the junction plates near this tile (see
/// [`synth::surface`]). See [`synth::emit`].
fn stamp_synth(
    buckets: &mut [Vec<EncoderFeature>],
    sampler: &mut GroundSampler,
    solved: &SolvedModel,
    junctions: &JunctionModel,
    z: u8,
    bounds: &Bounds,
) {
    // Plates near the tile's buffered extent, for band trimming — only at
    // zooms that draw the plates, so a trim can never open an unplated hole.
    let near: Vec<&synth::junction::BakedJunction> =
        if z >= crate::priors::STRUCTURE_DETAIL_MIN_ZOOM {
            let b = bounds.expanded(0.55);
            junctions.near((b.west, b.south, b.east, b.north))
        } else {
            Vec::new()
        };
    let mut bands = Vec::new();
    for f in &mut buckets[layers::TRANSPORTATION as usize] {
        synth::emit(f, sampler, solved, z, bounds);
        if let Some(band) = synth::surface::ribbon(f, sampler, solved, z, bounds, &near) {
            bands.push(band);
        }
    }
    buckets[layers::TRANSPORTATION as usize].extend(bands);
}

/// Phase-1 worker: drains row-group work items from the queue, streams their
/// features, and fans each into per-tile sort records in a worker-owned
/// sorter. Returns the sorter (merged k-way with the others afterwards) and
/// the worker's partial stats.
fn phase1_worker(
    inputs: &[(u8, GeoParquet)],
    queue: &Mutex<VecDeque<WorkItem>>,
    cfg: &Config,
    mem_budget: usize,
    scene: &SceneGraph,
    solved: &SolvedModel,
    junctions: &JunctionModel,
) -> Result<(ExternalSorter, Stats), Error> {
    let mut sorter = ExternalSorter::new(&cfg.tmp_dir, mem_budget);
    let mut stats = Stats::default();
    // Solved-reconciled span lists, built lazily per corridor — a corridor's
    // many segments all cut against the same list.
    let mut spans_cache: HashMap<u32, Vec<Span>> = HashMap::new();
    loop {
        let item = queue.lock().expect("phase-1 queue poisoned").pop_front();
        let Some(item) = item else {
            break;
        };
        let (layer, gp) = &inputs[item.input];

        // Read time is metered around each pull so it never includes the
        // simplify/clip/sort work between pulls.
        let t_open = Instant::now();
        let mut features = gp.features(vec![item.row_group], attrs_for(*layer))?;
        stats.timings.read += t_open.elapsed();
        loop {
            let t_read = Instant::now();
            let next = features.next();
            stats.timings.read += t_read.elapsed();
            let Some(feature) = next else {
                break;
            };
            process_feature(
                *layer,
                &feature?,
                cfg,
                &mut sorter,
                &mut stats,
                scene,
                solved,
                junctions,
                &mut spans_cache,
            )?;
        }
    }
    Ok((sorter, stats))
}

/// Phase-1 per-feature work: resolve the feature against the scene graph, then
/// hand each constant-kind piece — or the whole geometry — to
/// [`emit_geometry`], which profiles it and walks the tile quadtree.
///
/// A transportation segment the assemble stage claimed is re-emitted as its
/// corridor pieces, cut at the corridor's solved span boundaries
/// ([`crate::scene::Corridor::pieces`]) and tagged with the [`Synth`] generator
/// the emit worker will run — a structure sweep for bridge/tunnel spans, a
/// profile-lifted drape for the at-grade rest. Heights themselves are *not*
/// carried: the emit worker reads them from the shared solved model, which is
/// what makes every fragment agree (invariant 5). Unclaimed segments (minor
/// roads with no structure) and every other layer emit as before.
///
/// The walk clips each node's geometry from its parent's already-clipped piece
/// (child clip rects nest strictly inside the parent's, so the result equals
/// clipping the original), making the per-feature cost proportional to the
/// records emitted instead of `covered tiles × vertices`. Detail is carried at
/// `tolerance_for(layer, zmax)`; coarser zooms re-simplify their (small)
/// per-tile pieces on emission.
#[allow(clippy::too_many_arguments)]
fn process_feature(
    layer: u8,
    f: &crate::geoparquet::Feature,
    cfg: &Config,
    sorter: &mut ExternalSorter,
    stats: &mut Stats,
    scene: &SceneGraph,
    solved: &SolvedModel,
    junctions: &JunctionModel,
    spans_cache: &mut HashMap<u32, Vec<Span>>,
) -> Result<(), Error> {
    stats.features_read += 1;
    let Some(bb) = clip::bbox(&f.geometry) else {
        return Ok(());
    };
    if !bbox_intersects(bb, &cfg.bbox) {
        return Ok(());
    }

    if layer == layers::TRANSPORTATION {
        // The painted markings, generated here in phase 1 from pre-clip
        // geometry so the dash phase is global (synth::markings): every tile
        // then clips identical copies of every dash (invariant 5).
        let marks = marking_context(f, junctions, bb);
        let claimed = prop_id(&f.properties).and_then(|id| scene.lookup(source_hash(&id)));
        if let Some((corridor, seg)) = claimed {
            // Cut against the solved-reconciled spans: tunnels clamped to
            // their portal crossings so the above-ground approach a mapper
            // tagged "tunnel" is painted road right up to the portal mouth
            // (`solve::portals::reconcile_spans`).
            let spans = spans_cache.entry(corridor.id).or_insert_with(|| {
                match solved.profile(corridor.id) {
                    Some(p) => solve::portals::reconcile_spans(p, &corridor.spans),
                    None => corridor.spans.clone(),
                }
            });
            for piece in corridor.pieces_in(seg, spans) {
                let line = LineString(piece.line);
                let mut props = seg.properties.clone();
                // The marking synth: at-grade markings drape like the paint;
                // structure markings ride the deck ramp with it.
                let (synth, mark_synth) = match piece.kind {
                    SpanKind::Grade => {
                        let s = Synth::Road {
                            corridor: corridor.needs_profile().then_some(corridor.id),
                            deck: false,
                        };
                        (s, s)
                    }
                    kind => {
                        // A structure span emits twice: the solid (deck or
                        // bore), and the road paint re-emitted over it so the
                        // painted carriageway continues across the span instead
                        // of terminating at the abutment or portal. A bridge
                        // deck and a tunnel bore both carry the road surface on
                        // the same solved ramp (`Profile::deck_m` — the bore
                        // sweep rides it too), so the paint rides that ramp
                        // (`deck = true`) for either kind: it lies on the deck
                        // top of a bridge and on the bore's road surface of a
                        // tunnel. Where a bore runs buried the ramp dips under
                        // the hill, so the ribbon sinks with the mesh and the
                        // terrain occludes it — following the tunnel instead of
                        // draping over the ground it passes beneath — then
                        // re-emerges at the portal.
                        let stroke = Synth::Road { corridor: Some(corridor.id), deck: true };
                        emit_geometry(
                            layer,
                            &Geometry::LineString(line.clone()),
                            &props,
                            stroke,
                            cfg,
                            sorter,
                            stats,
                        )?;
                        // The level ordinal survives as a property only so the
                        // attribute profiler emits the reserved `level` the
                        // client colours structures by.
                        props.push(("level_rules".to_string(), Value::Int(piece.level)));
                        (Synth::Structure { corridor: corridor.id, kind }, stroke)
                    }
                };
                if let Some((class, oneway, width, disks)) = &marks {
                    for m in synth::markings::for_line(&line, class, *oneway, *width, disks) {
                        emit_geometry(layer, &m.geometry, &m.properties(), mark_synth, cfg, sorter, stats)?;
                    }
                }
                emit_geometry(layer, &Geometry::LineString(line), &props, synth, cfg, sorter, stats)?;
            }
            return Ok(());
        }
        // Unclaimed: a plain road that drapes on the rendered ground.
        let synth = Synth::Road { corridor: None, deck: false };
        if let Some((class, oneway, width, disks)) = &marks {
            if let Geometry::LineString(line) = &f.geometry {
                for m in synth::markings::for_line(line, class, *oneway, *width, disks) {
                    emit_geometry(layer, &m.geometry, &m.properties(), synth, cfg, sorter, stats)?;
                }
            }
        }
        return emit_geometry(layer, &f.geometry, &f.properties, synth, cfg, sorter, stats);
    }
    emit_geometry(layer, &f.geometry, &f.properties, Synth::None, cfg, sorter, stats)
}

/// The marking inputs for a transportation feature — its class, one-way
/// verdict, derived width, and the junction trim disks near it — or `None`
/// when the class's ladder paints nothing (the common case, so the plate
/// query is skipped entirely).
fn marking_context(
    f: &crate::geoparquet::Feature,
    junctions: &JunctionModel,
    bb: (f64, f64, f64, f64),
) -> Option<(String, bool, f64, Vec<(geo_types::Coord, f64)>)> {
    let find_str = |key: &str| {
        f.properties.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
            Value::String(s) => Some(s.clone()),
            _ => None,
        })
    };
    let class = find_str("class")?;
    let oneway = f
        .properties
        .iter()
        .any(|(k, v)| k == "oneway" && matches!(v, Value::Bool(true)));
    if !crate::priors::has_centre_line(&class, oneway) && !crate::priors::has_edge_lines(&class) {
        return None;
    }
    let measured = f.properties.iter().find_map(|(k, v)| match (k.as_str(), v) {
        ("width_rules", Value::Double(w)) => Some(*w),
        _ => None,
    });
    let width =
        crate::priors::carriageway_width_m(Some(&class), find_str("subclass").as_deref(), measured)?;
    let half = width * 0.5 + crate::priors::STRUCTURE_SHOULDER_M;
    // Pad by more than any plausible plate reach (~200 m in degrees).
    const MARGIN: f64 = 0.002;
    let disks = junctions
        .near((bb.0 - MARGIN, bb.1 - MARGIN, bb.2 + MARGIN, bb.3 + MARGIN))
        .into_iter()
        .map(|p| (p.point(), p.trim_radius_m(half)))
        .collect();
    Some((class, oneway, width, disks))
}

/// The source feature's `id` property, when it is a string.
fn prop_id(props: &[(String, Value)]) -> Option<&str> {
    props.iter().find(|(k, _)| k == "id").and_then(|(_, v)| match v {
        Value::String(s) => Some(s.as_str()),
        _ => None,
    })
}

/// Profiles one (sub-)geometry, then walks the tile quadtree from its first
/// emitted zoom down to `zmax`, appending one sort record per (zoom, tile) it
/// covers. The `synth` tag rides on every record so the emit worker knows
/// which generator to run.
fn emit_geometry(
    layer: u8,
    geometry: &Geometry,
    props: &[(String, Value)],
    synth: Synth,
    cfg: &Config,
    sorter: &mut ExternalSorter,
    stats: &mut Stats,
) -> Result<(), Error> {
    let prof = profile::profile(layer, props, cfg.min_zoom, cfg.max_zoom);
    let zmin = prof.min_zoom.max(cfg.min_zoom);
    let zmax = prof.max_zoom.min(cfg.max_zoom);
    if zmin > zmax {
        return Ok(());
    }

    // Skip the zooms where the whole feature is smaller than one screen pixel
    // (sub-visible): emission starts at the first zoom it is visible at.
    let zemit = first_visible_zoom(geometry, zmin, zmax);
    stats.dropped_subpixel += (zemit.min(zmax + 1) - zmin) as u64;
    let Some(zemit) = (zemit <= zmax).then_some(zemit) else {
        return Ok(());
    };

    // Carry detail only down to what the finest emitted zoom keeps.
    let t_simplify = Instant::now();
    let base = simplify::simplify_geometry(geometry, tolerance_for(layer, zmax));
    stats.timings.simplify += t_simplify.elapsed();
    let Some(base) = base else {
        return Ok(());
    };
    let Some(base_bb) = clip::bbox(&base) else {
        return Ok(());
    };

    let walk = Walk {
        layer,
        rank: prof.rank,
        zmax,
        bbox: &cfg.bbox,
        encoder: record::RecordEncoder::new(0, &prof.properties, synth),
    };
    let (x0, x1, y0, y1) = clip::candidate_range(base_bb, zemit);
    for y in y0..=y1 {
        for x in x0..=x1 {
            let rect = Bounds::of_tile(zemit, x, y).expanded(clip::BUFFER_FRAC);
            let t_clip = Instant::now();
            let clipped = clip::clip_geometry(&base, &rect);
            stats.timings.clip += t_clip.elapsed();
            if let Some(clipped) = clipped {
                descend(&walk, clipped, zemit, x, y, sorter, stats)?;
            }
        }
    }
    Ok(())
}

/// Per-feature constants of one quadtree walk.
struct Walk<'a> {
    layer: u8,
    rank: u16,
    zmax: u8,
    /// Run bounds — subtrees outside it are pruned whole.
    bbox: &'a Bounds,
    encoder: record::RecordEncoder,
}

/// Emits the sort record for tile `(z, x, y)` — whose buffered bounds `geom`
/// is already clipped to — then clips `geom` into the four child tiles and
/// recurses until `zmax`.
fn descend(
    walk: &Walk,
    geom: Geometry,
    z: u8,
    x: u32,
    y: u32,
    sorter: &mut ExternalSorter,
    stats: &mut Stats,
) -> std::io::Result<()> {
    let tb = Bounds::of_tile(z, x, y);
    if !bbox_intersects((tb.west, tb.south, tb.east, tb.north), walk.bbox) {
        return Ok(());
    }

    // Emit this zoom's record: the carried geometry is at the layer's
    // tolerance for zmax, so coarser zooms re-simplify their per-tile piece.
    let emit_geom = if z == walk.zmax {
        Some(geom.clone())
    } else {
        let t_simplify = Instant::now();
        let simplified = simplify::simplify_geometry(&geom, tolerance_for(walk.layer, z));
        stats.timings.simplify += t_simplify.elapsed();
        simplified
    };
    if let Some(emit_geom) = emit_geom {
        let key = tileid::sort_key_for(z, x, y, walk.layer, walk.rank);
        let t_sort = Instant::now();
        let payload = walk.encoder.encode(&emit_geom);
        stats.record_bytes += payload.len() as u64;
        sorter.add(key, &payload)?;
        stats.records += 1;
        stats.timings.sort += t_sort.elapsed();
    }
    if z == walk.zmax {
        return Ok(());
    }

    // Recurse into the children the geometry's bbox touches. The child's
    // buffered rect nests strictly inside this node's, so clipping the carried
    // piece equals clipping the original geometry.
    let Some(gb) = clip::bbox(&geom) else {
        return Ok(());
    };
    for cy in 0..2u32 {
        for cx in 0..2u32 {
            let (nz, nx, ny) = (z + 1, 2 * x + cx, 2 * y + cy);
            let rect = Bounds::of_tile(nz, nx, ny).expanded(clip::BUFFER_FRAC);
            if gb.0 > rect.east || gb.2 < rect.west || gb.1 > rect.north || gb.3 < rect.south {
                continue;
            }
            let t_clip = Instant::now();
            let child = clip::clip_geometry(&geom, &rect);
            stats.timings.clip += t_clip.elapsed();
            if let Some(child) = child {
                descend(walk, child, nz, nx, ny, sorter, stats)?;
            }
        }
    }
    Ok(())
}

/// First zoom in `[zmin, zmax]` at which the feature is at least one screen
/// pixel in size (polygons by area, lines by length; points always show), or
/// `zmax + 1` when it never is. The pixel size at a zoom is `tolerance(z)`.
fn first_visible_zoom(geom: &Geometry, zmin: u8, zmax: u8) -> u8 {
    enum Measure {
        Area(f64),
        Length(f64),
        Always,
    }
    let measure = match geom {
        Geometry::Polygon(_) | Geometry::MultiPolygon(_) => Measure::Area(simplify::area(geom)),
        Geometry::LineString(_) | Geometry::MultiLineString(_) => {
            Measure::Length(simplify::length(geom))
        }
        _ => Measure::Always,
    };
    for z in zmin..=zmax {
        let px = tolerance(z);
        let visible = match measure {
            Measure::Area(a) => a >= px * px,
            Measure::Length(l) => l >= px,
            Measure::Always => true,
        };
        if visible {
            return z;
        }
    }
    zmax + 1
}

/// Folds a worker's phase-1 partial stats into the run totals. Stage timings
/// add up across workers (CPU seconds, see [`Timings`]).
fn merge_phase1(into: &mut Stats, from: &Stats) {
    into.features_read += from.features_read;
    into.records += from.records;
    into.record_bytes += from.record_bytes;
    into.dropped_subpixel += from.dropped_subpixel;
    into.timings.read += from.timings.read;
    into.timings.simplify += from.timings.simplify;
    into.timings.clip += from.timings.clip;
    into.timings.sort += from.timings.sort;
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
#[allow(clippy::too_many_arguments)]
fn flush_tile(
    writer: &mut ArchiveWriter<File>,
    tile_id: u64,
    buckets: &mut [Vec<EncoderFeature>],
    layer_stats: &mut LayerStats,
    stats: &mut Stats,
    flat: &TerrainMesh,
    sampler: &mut GroundSampler,
    solved: &SolvedModel,
    junctions: &JunctionModel,
    elevation: &mut (f64, f64),
    quality: i32,
) -> Result<(), Error> {
    let (z, x, y) = hilbert::tile_id_decode(tile_id);
    let bounds = Bounds::of_tile(z, x, y);
    stamp_elevations(buckets, sampler, z);
    stamp_synth(buckets, sampler, solved, junctions, z, &bounds);
    add_junction_plates(buckets, junctions, sampler, &bounds, z, solved.z_ref);

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
    // With a DEM, build a per-tile mesh of the engineered ground (and track
    // the elevation range); otherwise reuse the shared flat mesh.
    layer_stats.observe(layers::TERRAIN as usize, z, GeometryType::Mesh);
    let blob = if sampler.has_elevation() {
        let t_terrain = Instant::now();
        let (mesh, emin, emax) =
            terrain::elevated_mesh(TERRAIN_GRID, &bounds, |lon, lat| sampler.corner(lon, lat, z));
        stats.timings.terrain += t_terrain.elapsed();
        elevation.0 = elevation.0.min(emin);
        elevation.1 = elevation.1.max(emax);
        let t_encode = Instant::now();
        let blob = tile_build::build_tile_q(&bounds, Some(&mesh), &enc_layers, quality);
        stats.timings.encode += t_encode.elapsed();
        blob
    } else {
        let t_encode = Instant::now();
        let blob = tile_build::build_tile_q(&bounds, Some(flat), &enc_layers, quality);
        stats.timings.encode += t_encode.elapsed();
        blob
    };
    let t_write = Instant::now();
    writer.add_tile(z, x, y, &blob)?;
    stats.timings.write += t_write.elapsed();
    stats.tiles_written += 1;
    Ok(())
}

/// Simplification tolerance for a zoom, in degrees. Low/mid zooms simplify to
/// roughly one screen pixel at ~512 px, which keeps the whole world's coastline
/// from bloating a single z0 tile. The deepest zooms, though, are overzoomed and
/// inspected up close in the 3D viewer (street level), where a 512 px budget
/// visibly flattens road curves; there we keep ~8× finer detail, still well
/// within the format's ~1/32768-of-a-tile quantization precision.
fn tolerance(z: u8) -> f64 {
    let tile_w = 360.0 / (1u64 << z as u32) as f64; // 2^z columns
    let div = if z >= 13 { 4096.0 } else { 512.0 };
    tile_w / div
}

/// Simplification tolerance for a layer at a zoom. Most layers simplify to the
/// display pixel (`tolerance`); building footprints are never simplified
/// (zero tolerance is a pass-through in `simplify_geometry`). They are
/// extruded and inspected up close, where Douglas–Peucker visibly mangles
/// right-angled walls; tile quantization is the only rounding they get.
fn tolerance_for(layer: u8, z: u8) -> f64 {
    match layer {
        layers::BUILDING => 0.0,
        _ => tolerance(z),
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

    /// Scans `$ARPA` for transportation `MeshGeometry` features (baked tunnel
    /// boxes), reporting their tile and approximate lon/lat so a screenshot can
    /// be aimed at one. Run: `ARPA=/tmp/tunnels.arpa cargo test -- --ignored dump_tunnels --nocapture`
    #[test]
    #[ignore = "needs $ARPA"]
    fn dump_tunnels() {
        let path = std::env::var("ARPA").unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let archive = crate::archive::Archive::open(&bytes).unwrap();
        let mut total = 0usize;
        let mut shown = 0usize;
        for entry in archive.entries() {
            let raw = brotli_decompress(archive.get_by_id(entry.hilbert_id).unwrap());
            let tile = fbt::root_as_tile(&raw).unwrap();
            let Some(layers) = tile.layers() else { continue };
            for li in 0..layers.len() {
                let l = layers.get(li);
                if l.name() != "transportation" {
                    continue;
                }
                let Some(feats) = l.features() else { continue };
                for fi in 0..feats.len() {
                    let f = feats.get(fi);
                    let Some(m) = f.geometry_as_mesh_geometry() else { continue };
                    total += 1;
                    if shown < 12 {
                        let b = crate::project::Bounds::of_tile(entry.z, entry.x, entry.y);
                        // First vertex → approx lon/lat (dequantize).
                        let qx = m.x().get(0) as f64;
                        let qy = m.y().get(0) as f64;
                        let lon = b.west + (qx - 16384.0) / 32768.0 * b.width();
                        let lat = b.south + (qy - 16384.0) / 32768.0 * b.height();
                        eprintln!(
                            "tunnel @ tile {}/{}/{}  verts={} tris={}  lon={:.5} lat={:.5}",
                            entry.z, entry.x, entry.y, m.x().len(), m.indices().len() / 3, lon, lat
                        );
                        shown += 1;
                    }
                }
            }
        }
        eprintln!("== {total} tunnel mesh features in {path} ==");
        assert!(total > 0, "no tunnel meshes found");
    }

    /// Tallies painted road widths per class across `$ARPA`'s transportation
    /// lines at zoom `$Z` (default 16) — verifies the P1 width derivation end
    /// to end: mapped `width_rules` values must appear alongside the class
    /// priors. Run: `ARPA=/tmp/t.arpa cargo test -- --ignored dump_widths --nocapture`
    #[test]
    #[ignore = "needs $ARPA"]
    fn dump_widths() {
        let path = std::env::var("ARPA").unwrap();
        let z_want: u8 = std::env::var("Z").ok().and_then(|s| s.parse().ok()).unwrap_or(16);
        let bytes = std::fs::read(&path).unwrap();
        let archive = crate::archive::Archive::open(&bytes).unwrap();
        // (class, width string) → feature count; string keys keep widths sortable.
        let mut tally: std::collections::BTreeMap<(String, String), u64> = Default::default();
        let (mut surfaces, mut oneways) = (0u64, 0u64);
        for entry in archive.entries() {
            if entry.z != z_want {
                continue;
            }
            let raw = brotli_decompress(archive.get_by_id(entry.hilbert_id).unwrap());
            let tile = fbt::root_as_tile(&raw).unwrap();
            let (Some(layers), Some(keys), Some(values)) =
                (tile.layers(), tile.keys(), tile.values())
            else {
                continue;
            };
            for li in 0..layers.len() {
                let l = layers.get(li);
                if l.name() != "transportation" {
                    continue;
                }
                let Some(feats) = l.features() else { continue };
                for fi in 0..feats.len() {
                    let f = feats.get(fi);
                    if f.geometry_as_line_geometry().is_none() {
                        continue;
                    }
                    let Some(props) = f.properties() else { continue };
                    let (mut class, mut width) = (None, None);
                    for pi in 0..props.len() {
                        let p = props.get(pi);
                        let v = values.get(p.value() as usize);
                        match keys.get(p.key() as usize) {
                            "class" => class = v.string_value().map(str::to_string),
                            "width_m" => width = Some(format!("{:5.1}", v.double_value())),
                            "surface" => surfaces += 1,
                            "oneway" => oneways += 1,
                            _ => {}
                        }
                    }
                    if let (Some(c), Some(w)) = (class, width) {
                        *tally.entry((c, w)).or_default() += 1;
                    }
                }
            }
        }
        for ((c, w), n) in &tally {
            eprintln!("{c:>14} {w} m  x{n}");
        }
        eprintln!("== z{z_want}: {surfaces} surfaces, {oneways} oneways ==");
        assert!(!tally.is_empty(), "no painted widths at z{z_want}");
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

    /// Scans $ARPA for transportation lines carrying a baked deck `z` array
    /// (bridges), reporting how many tiles/features have one and the elevation
    /// span of the first few — confirms the bridge-deck pipeline end to end.
    /// Run: `ARPA=/tmp/claude/bridges.arpa cargo test -- --ignored dump_bridge_decks --nocapture`
    #[test]
    #[ignore = "needs $ARPA"]
    fn dump_bridge_decks() {
        let path = std::env::var("ARPA").unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let archive = crate::archive::Archive::open(&bytes).unwrap();
        let mut tiles_with_deck = 0usize;
        let mut deck_feats = 0usize;
        let mut shown = 0usize;
        for entry in archive.entries() {
            let raw = brotli_decompress(archive.get_by_id(entry.hilbert_id).unwrap());
            let tile = fbt::root_as_tile(&raw).unwrap();
            let Some(layers) = tile.layers() else { continue };
            let mut tile_has = false;
            for li in 0..layers.len() {
                let l = layers.get(li);
                if l.name() != "transportation" {
                    continue;
                }
                let Some(feats) = l.features() else { continue };
                for fi in 0..feats.len() {
                    let Some(g) = feats.get(fi).geometry_as_line_geometry() else { continue };
                    let Some(z) = g.z() else { continue };
                    if z.len() == 0 {
                        continue;
                    }
                    tile_has = true;
                    deck_feats += 1;
                    if shown < 8 {
                        let (mut lo, mut hi) = (i32::MAX, i32::MIN);
                        for k in 0..z.len() {
                            lo = lo.min(z.get(k));
                            hi = hi.max(z.get(k));
                        }
                        eprintln!(
                            "tile {}/{}/{} deck: {} verts, z {}..{} mm (rise {} mm)",
                            entry.z, entry.x, entry.y, z.len(), lo, hi, hi - lo
                        );
                        shown += 1;
                    }
                }
            }
            if tile_has {
                tiles_with_deck += 1;
            }
        }
        eprintln!("== {deck_feats} bridge-deck features across {tiles_with_deck} tiles ==");
        assert!(deck_feats > 0, "no baked bridge decks found in {path}");
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
            threads: 0,
            brotli_quality: tile_build::DEFAULT_QUALITY,
            dump: None,
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
