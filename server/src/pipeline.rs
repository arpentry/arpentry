//! Top-level pipeline — stage 5 of docs/GENERATION.md §5, hosting stages 1–4.
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
//! model (`Arc<SolvedModel>`, `Arc<GroundStack>`) and the global terrain
//! lattice — never of the tile window — so adjacent tiles and successive
//! zooms agree by construction (invariant 5) and tiling carries no modeling
//! responsibility.
//!
//! Parallel and dependency-free (`std::thread` + channels). Phase 1 fans
//! row-group work items out to workers that feed per-worker external sorters;
//! phase 2 groups the merged stream into per-tile jobs encoded by a worker
//! pool and written back in stream order. `Config::threads == 1` runs both
//! phases serially on the calling thread.

use std::collections::VecDeque;
use std::fs::File;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use geo_types::{Coord, Geometry, LineString};

use crate::archive::{ArchiveMeta, ArchiveWriter};
use crate::assemble;
use crate::clip;
use crate::dem::Dem;
use crate::dump;
use crate::geom::GeometryType;
use crate::geoparquet::GeoParquet;
use crate::ground::{self, sampler::GroundSampler, GroundStack};
use crate::hilbert;
use crate::layers;
use crate::profile;
use crate::project::Bounds;
use crate::record;
use crate::scene::{source_hash, SceneGraph, SpanKind};
use crate::simplify;
use crate::solve::{self, SolvedModel};
use crate::sort::{self, ExternalSorter};
use crate::synth::carriageway::CarriagewayModel;
use crate::synth::region::Region;
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
            // The `is_bridge`/`is_tunnel` fallback: a substantial share of
            // structures carry only the flag. `synth::draped` needs the same
            // annotations the assemble stage reads, or a footbridge mapped
            // that way gets no deck.
            "road_flags",
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
    /// Where to write the model-side scorecard (docs/GENERATION.md §8's
    /// structural checks), or `None` to skip them. They re-solve the scene, so
    /// they are opt-in rather than part of every run.
    pub verify_model: Option<PathBuf>,
    /// Whether detail-zoom terrain meshes are constrained by the bench
    /// contact lines (docs/GROUND.md §3). On by default; `--no-breaklines`
    /// is the escape hatch back to the plain lattice.
    pub breaklines: bool,
    /// Whether the detail-zoom terrain mesh stops at the kerb (docs/GROUND.md
    /// §3, "the hole"). On by default; `--no-hole` puts the ground back under
    /// the asphalt so an A/B re-tile is a flag rather than a patch. Implied off
    /// by `--no-breaklines`: there is no constrained mesh to cut.
    pub hole: bool,
}

/// The run's detail-mesh options, in the shape every [`GroundSampler`] takes.
/// One translation of the config, so the three samplers a run builds — the
/// probe's, the serial path's, and each emit worker's — cannot disagree.
fn mesh_options(cfg: &Config) -> ground::sampler::MeshOptions {
    ground::sampler::MeshOptions { breaklines: cfg.breaklines, hole: cfg.hole }
}

/// The global world model: everything stages 1–3 built, and the derived models
/// stage 4 reads. Built once before tiling, shared by every worker.
///
/// **A type, so that the invariant is not a comment.** Every height an emit
/// worker writes is a function of these and the global terrain lattice, never
/// of the tile window — which is what makes adjacent tiles and successive zooms
/// agree by construction (invariant 5). Passing the six pieces separately said
/// nothing about that and cost twelve-argument signatures: `flush_tile` took
/// twelve, `emit_parallel` eleven, `phase1_worker` and `process_feature` ten
/// each, four of them carrying a `too_many_arguments` waiver. Nothing in those
/// lists distinguished the shared model from the per-tile state it is the whole
/// point to keep separate.
///
/// Everything here is behind an `Arc` and immutable after construction. The
/// per-tile state — the `GroundSampler`, the height field, the buckets — is
/// deliberately *not* here: it is the half that varies, and it stays in the
/// argument lists where it is visible.
pub struct World {
    pub scene: SceneGraph,
    pub solved: Arc<SolvedModel>,
    pub ground: Arc<GroundStack>,
    /// Carriageway sources, handover cuts and intersection extents, derived
    /// from the solved model (`synth::carriageway`).
    pub junctions: Arc<CarriagewayModel>,
    /// The unioned road surface, one paved region per level per z13 chunk.
    pub pavement: Arc<synth::pavement::PavementModel>,
    /// Every solved bridge deck indexed by plan position, so phase 1 can ask
    /// whether a draped feature's elevated span is a sidewalk on one of them.
    pub carriers: synth::carried::Carriers,
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
    /// Intersections clustered from the scene's connectors. An extent count,
    /// not a geometry count: nothing here is drawn since the union replaced the
    /// plates.
    pub intersections: u64,
    /// The bench contact lines the detail terrain holds (docs/GROUND.md §3),
    /// and how crowded they are: crest nodes pulled in off their nominal offset
    /// because a contending bench holds the ground there, and crest nodes
    /// dropped because no bench of their own survived. High counts mean roads
    /// packed closer than their benches are wide — switchbacks, dual
    /// carriageways, interchange ramps.
    pub crest_segments: u64,
    pub crests_pulled: u64,
    pub crests_dropped: u64,
    /// Chunks carrying a unioned road surface, and its total area in m² — the
    /// coarse check that the union actually unioned rather than passing the
    /// per-road bands through.
    pub pave_chunks: u64,
    pub pave_area_m2: f64,
    /// Vertical consistency of the solved model (docs/GENERATION.md §8): the
    /// worst junction step (member road-height disagreement), its 99th
    /// percentile, how many junctions disagree by more than half a metre, and
    /// the worst clearance shortfall at a crossing. The number the
    /// constraint-graph solver drives to zero.
    pub max_junction_step_m: f64,
    pub p99_junction_step_m: f64,
    pub junction_steps_over: u64,
    pub max_clearance_violation_m: f64,
    /// Clearance demands the relaxation's plausibility cap rejected, and the
    /// worst of them. A dropped demand is a data contradiction resolved in
    /// favour of the profile — the right call, but one that must be counted:
    /// silently dropping twice as many looks exactly like fixing them.
    pub clearance_demands_dropped: u64,
    pub worst_dropped_demand_m: f64,
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
    /// Buffering and unioning the road surface into per-chunk regions.
    pub pavement: Duration,
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
    // the lattice the client sees close up), and derive the engineered
    // ground (the DEM plus the earthworks and bench contact lines the solved
    // model implies). Everything after reads these through Arcs; no height
    // ever depends on a tile window.
    let t_model = Instant::now();
    let transportation =
        cfg.inputs.iter().find(|(l, _)| *l == layers::TRANSPORTATION).map(|(_, p)| p.clone());
    let water = cfg.inputs.iter().find(|(l, _)| *l == layers::WATER).map(|(_, p)| p.clone());
    let mut scene = match &transportation {
        Some(path) => assemble::run(path, water.as_deref(), &cfg.bbox)
            .map_err(|e| format!("{}: {e}", path.display()))?,
        None => SceneGraph::default(),
    };
    // Solve mutates the scene once: the terrain fate of short structure spans
    // (`solve::reconcile_short_spans`) is settled before anything downstream
    // reads the corridor spans.
    let solved =
        Arc::new(solve::run(&mut scene, cfg.terrain.as_deref(), cfg.max_zoom, threads)?);
    let ground = Arc::new(ground::derive(&scene, &solved, cfg.terrain.as_deref(), threads));
    // Junction plates: a paved area meshed across each corridor junction, baked
    // once from the solved model and emitted by the tile that owns its centre.
    let junctions = Arc::new(synth::carriageway::bake(&scene, &solved));
    // The unioned road surface: one paved region per level per z13 chunk, baked
    // once from the same carriageway sources the intersections came from.
    let t_pave = Instant::now();
    let pavement = Arc::new(synth::pavement::bake(&junctions, threads));
    // Every solved bridge deck, indexed by plan position, so phase 1 can ask
    // whether a draped feature's elevated span is really the sidewalk on one
    // of them (`synth::carried`). Built once and shared: the answer is a
    // function of the solved model, so every worker and every tile must get
    // the same one (I5).
    let carriers = synth::carried::Carriers::build(&scene, &solved);
    let world = World { scene, solved, ground, junctions, pavement, carriers };
    let World { scene, solved, ground, junctions, pavement, .. } = &world;
    stats.pave_chunks = pavement.chunk_count() as u64;
    stats.pave_area_m2 = pavement.area_m2();
    stats.timings.pavement = t_pave.elapsed();
    stats.corridors = scene.corridors.len() as u64;
    stats.profiles = solved.solved_count() as u64;
    stats.crossings = solved.crossings.len() as u64;
    stats.earthworks = ground.earthwork_count() as u64;
    stats.crest_segments = ground.breaklines().len() as u64;
    let (pulled, dropped) = ground.breaklines().crowding();
    stats.crests_pulled = pulled as u64;
    stats.crests_dropped = dropped as u64;
    stats.water = ground.water_count() as u64;
    stats.intersections = junctions.len() as u64;
    let consistency = solve::consistency::measure(&scene, &solved);
    stats.max_junction_step_m = consistency.max_junction_step_m;
    stats.p99_junction_step_m = consistency.p99_junction_step_m;
    stats.junction_steps_over = consistency.junction_steps_over;
    stats.max_clearance_violation_m = consistency.max_clearance_violation_m;
    stats.clearance_demands_dropped = solved.relaxed.demands_dropped;
    stats.worst_dropped_demand_m = solved.relaxed.worst_dropped_m;
    stats.timings.model = t_model.elapsed();
    if let Some(dir) = &cfg.dump {
        dump::write(dir, &scene, &solved, &ground)?;
    }
    // The structural half of the scorecard (§8): I7 and I8 are established by
    // construction and falsified by a perturbation experiment, which needs the
    // model and not the archive. Opt-in, because it re-solves the scene.
    if let Some(path) = &cfg.verify_model {
        let m = crate::verify::model::Model {
            scene,
            solved,
            ground,
            terrain: cfg.terrain.as_deref(),
            threads,
        };
        let t_model_verify = Instant::now();
        let metrics = crate::verify::model::run(&m);
        let json = crate::verify::model::to_json(&metrics);
        std::fs::write(path, serde_json::to_string_pretty(&json).unwrap_or_default())?;
        eprintln!(
            "model checks {:>5.1}s  {} metrics -> {}",
            t_model_verify.elapsed().as_secs_f64(),
            metrics.len(),
            path.display()
        );
    }

    // Diagnostic probe (ARPT_PROBE="lon,lat"): at that point, for every corridor
    // whose centerline passes near it, print the deck-top height, the road
    // profile height, the raw terrain, and — the key number — the *rendered
    // road-surface* height (`synth::road::surface_height` at z_ref, what the
    // approach asphalt band actually drapes on). A gap between the deck height
    // and the rendered road surface is the visible bridge-end step, localised to
    // the earthwork/render layer rather than the solve.
    if let Ok(spec) = std::env::var("ARPT_PROBE") {
        if let Some((lon, lat)) = spec.split_once(',').and_then(|(a, b)| {
            Some((a.trim().parse::<f64>().ok()?, b.trim().parse::<f64>().ok()?))
        }) {
            let dem = cfg.terrain.as_deref().and_then(|p| Dem::open(p).ok());
            let mut sampler =
                GroundSampler::new(dem, Arc::clone(&ground), solved.z_ref, mesh_options(cfg));
            let zref_bounds = solve::tile_containing(solved.z_ref, lon, lat);
            let cos = lat.to_radians().cos();
            eprintln!("PROBE {lon},{lat} (z_ref={})", solved.z_ref);
            for c in &scene.corridors {
                let Some(p) = solved.profile(c.id) else { continue };
                let a = p.arc_of(lon, lat);
                let pt = p.point_at_arc(a);
                let d = ((pt.x - lon) * cos).hypot(pt.y - lat) * crate::scene::DEG_M;
                if d > 8.0 {
                    continue;
                }
                let road = p.height_at(lon, lat);
                let deck = p.deck_height_at(lon, lat);
                let terr = p.surface_at(lon, lat);
                let band =
                    synth::road::surface_height(Some(p), false, &mut sampler, solved.z_ref, solved.z_ref, &zref_bounds, lon, lat);
                let ground_h = {
                    let mut sc = Vec::new();
                    ground.height(lon, lat, terr, 0.0, &mut sc)
                };
                eprintln!(
                    "  corr {:>5} {:>4}m  road={:.1} deck={:.1} terr={:.1} rendered_road_surface={:.1} engineered_ground={:.1}  DECK-SURFACE_STEP={:.1}",
                    c.id, d as i64, road, deck, terr, band, ground_h, deck - band
                );
            }
        }
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

    // One DEM handle per phase-1 worker, forked from a primary so the decoded
    // tiles are shared — the same arrangement the emit pool uses. Phase 1 reads
    // the ground only where a draped feature carries an elevated span, which is
    // a few hundred features in a city extract.
    let primary_dem = match &cfg.terrain {
        Some(path) => Some(Dem::open(path)?),
        None => None,
    };
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
            phase1_worker(&inputs, &queue, cfg, worker_budget, &world, primary_dem)?;
        merge_phase1(&mut stats, &partial);
        sorters.push(sorter);
    } else {
        std::thread::scope(|scope| -> Result<(), Error> {
            let mut handles = Vec::with_capacity(phase1_threads);
            // Only the DEM handle is per-worker; the world model is shared, so
            // the closure moves that one binding and borrows the rest.
            let (inputs, queue, world) = (&inputs, &queue, &world);
            for _ in 0..phase1_threads {
                let dem = match &primary_dem {
                    Some(d) => Some(d.fork()?),
                    None => None,
                };
                handles.push(scope.spawn(move || {
                    phase1_worker(inputs, queue, cfg, worker_budget, world, dem)
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
        let mut sampler =
            GroundSampler::new(dem, Arc::clone(ground), solved.z_ref, mesh_options(cfg));
        let tile_ctx = TileContext { flat: &flat, world: &world, quality: cfg.brotli_quality };
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
                    flush_tile(&mut writer, prev, &mut buckets, &mut layer_stats, &mut stats, &tile_ctx, &mut sampler, &mut elevation)?;
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
            flush_tile(&mut writer, prev, &mut buckets, &mut layer_stats, &mut stats, &tile_ctx, &mut sampler, &mut elevation)?;
        }
    } else {
        emit_parallel(cfg, sorted, threads, &mut writer, &mut layer_stats, &mut stats,
            &mut elevation, &world)?;
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

/// What every tile encode reads and none of them may modify: the shared world
/// model, the flat fallback mesh, and the compression setting.
///
/// Split from the mutable per-tile state on purpose — the sampler, the buckets,
/// the writer and the stats stay separate arguments, because those are the ones
/// that vary and the ones a reader has to keep track of.
struct TileContext<'a> {
    flat: &'a TerrainMesh,
    world: &'a World,
    quality: i32,
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
    world: &World,
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
            let ground = Arc::clone(&world.ground);
            let z_ref = world.solved.z_ref;
            let dem = match &primary_dem {
                Some(d) => Some(d.fork()?),
                None => None,
            };
            workers.push(scope.spawn(move || -> Result<(), Error> {
                let flat = terrain::flat_mesh(TERRAIN_GRID);
                let tile_ctx =
                    TileContext { flat: &flat, world, quality: cfg.brotli_quality };
                let mut sampler = GroundSampler::new(dem, ground, z_ref, mesh_options(cfg));
                loop {
                    // Blocking recv under the lock serializes idle waits only;
                    // a queued job is handed off immediately.
                    let job = job_rx.lock().expect("emit queue poisoned").recv();
                    let Ok(job) = job else {
                        break;
                    };
                    let result =
                        encode_tile(job, &tile_ctx, &mut sampler);
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
    tile: &TileContext<'_>,
    sampler: &mut GroundSampler,
) -> Result<TileResult, Error> {
    let TileContext { flat, world, quality } = tile;
    let World { solved, pavement, junctions, .. } = *world;
    let quality = *quality;
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
    // One height field per tile, shared by the paint, the bands and the plates —
    // so all three land on the same asphalt (docs/ROADS.md invariant 5).
    let field = synth::height::HeightField::for_tile(junctions, solved, z, &bounds);
    stamp_synth(&mut buckets, &field, sampler, solved, z, &bounds);
    // The at-grade paved regions this tile actually meshed — what the terrain
    // mesh below cuts its hole from (docs/GROUND.md §3). Empty where no hole is
    // cut, so the terrain mesher needs no second opinion on whether to cut.
    let cut_regions =
        add_road_surface(&mut buckets, pavement, &field, sampler, &bounds, z, solved.z_ref);
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
        let (mesh, emin, emax) = sampler.terrain_mesh(&bounds, z, &cut_regions);
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


/// Whether a transportation feature is a painted marking (class `marking`)
/// rather than a carriageway. Markings are tagged `Synth::Road` like the paint
/// they ride, so this class check is what separates them from a road when the
/// fill stroke is dropped — markings keep their SDF stroke at every zoom.
fn is_marking(f: &EncoderFeature) -> bool {
    f.properties
        .iter()
        .any(|(k, v)| k.as_str() == "class" && matches!(v, Value::String(s) if s.as_str() == "marking"))
}

/// Stage 4 for the tile: runs each transportation feature's generator against
/// the solved model — bridge decks and tunnel bores swept on their corridor's
/// profile, roads draped on the shared height field.
///
/// At the surface zoom the carriageway fill is the *unioned* mesh
/// ([`add_road_surface`]), so a contributing road's SDF fill stroke is dropped —
/// otherwise the carriageway is painted twice. What keeps its stroke: markings
/// (they ride the mesh as their own features), non-drivable ways with no width to
/// buffer, and every feature below the surface zoom, where roads stay pure draped
/// SDF. See [`synth::emit`].
fn stamp_synth(
    buckets: &mut [Vec<EncoderFeature>],
    field: &synth::height::HeightField,
    sampler: &mut GroundSampler,
    solved: &SolvedModel,
    z: u8,
    bounds: &Bounds,
) {
    let surface_zoom = z >= crate::priors::ROAD_SURFACE_MIN_ZOOM;
    buckets[layers::TRANSPORTATION as usize].retain_mut(|f| {
        synth::emit(f, field, sampler, solved, z, bounds);
        // A carriageway the union paved has no business also painting its SDF
        // fill: the mesh *is* the surface now. This covers the at-grade stroke and
        // the `deck: true` stroke re-painted over a structure, whose own solid
        // carries its top.
        !(surface_zoom && paves_via_union(f))
    });
}

/// Whether the unioned surface covers this feature, so its own fill stroke would
/// be a second coat of the same paint.
///
/// The test is the same one the union's input used: a drivable carriageway is
/// exactly a feature with a `width_m` to buffer. A marking has one too and must
/// keep its stroke — that *is* its geometry — and a footway has none, so it keeps
/// the cartographic stroke that is all it has ever had.
///
/// A railway keeps its stroke too, though its formation is in the union: the
/// asphalt fill *replaces* a road's stroke, but the rail stroke is not a fill —
/// it is the track, riding the ballast band the way a marking rides the
/// asphalt, and it is also the line `contact.rail_standoff` measures.
fn paves_via_union(f: &EncoderFeature) -> bool {
    if is_marking(f) {
        return false;
    }
    if !matches!(f.synth, Synth::Road { corridor: Some(_), .. }) {
        return false;
    }
    let class = f.properties.iter().find_map(|(k, v)| match (k.as_str(), v) {
        ("class", Value::String(s)) => Some(s.as_str()),
        _ => None,
    });
    let kind = crate::priors::Kind::parse(None, class, None);
    if kind.prior().surface != crate::priors::Surface::Asphalt {
        return false;
    }
    f.properties
        .iter()
        .any(|(k, v)| k.as_str() == "width_m" && matches!(v, Value::Double(w) if *w > 0.0))
}

/// Adds this tile's share of the unioned road surface: one opaque `road_surface`
/// mesh per level plus the `road_casing` rim that antialiases and edges it.
///
/// Unlike the junction plates this replaced, there is no "which tile owns it"
/// question — the region is clipped to the tile proper, so every tile emits
/// exactly its own piece and the pieces meet at a shared, snapped seam.
fn add_road_surface(
    buckets: &mut [Vec<EncoderFeature>],
    pavement: &synth::pavement::PavementModel,
    field: &synth::height::HeightField,
    sampler: &mut GroundSampler,
    bounds: &Bounds,
    z: u8,
    z_ref: u8,
) -> Vec<Region> {
    if z < crate::priors::ROAD_SURFACE_MIN_ZOOM || !sampler.has_elevation() {
        return Vec::new();
    }
    let Some(levels) = pavement.chunk_for(bounds) else { return Vec::new() };
    // The casing goes opaque on exactly the tiles whose ground is cut away, so
    // the paver has to be asked the same question the terrain mesher will
    // answer (see `build_rim`).
    let hole = sampler.cuts_hole(z);
    let mut cut: Vec<Region> = Vec::new();
    // `ARPT_KERB_AT_HANDOVER=1` withholds the abutment cuts, so every boundary
    // edge is treated as kerb and the rim goes back to wrapping the whole
    // silhouette. The same reason `--no-hole` exists: an A/B re-tile of a
    // change to the drawn surface should be a flag rather than a patch.
    let handovers = if std::env::var_os("ARPT_KERB_AT_HANDOVER").is_some() {
        &[][..]
    } else {
        pavement.handovers_for(bounds)
    };
    for paved in
        synth::pave_mesh::tile_meshes(levels, field, sampler, z, z_ref, bounds, hole, handovers)
    {
        let anchor = paved.anchor;
        let id = anchor.x.to_bits() ^ anchor.y.to_bits().rotate_left(32);
        // The material picks the class family, and with it the style entry: a
        // rail formation is the same machinery as a carriageway in another
        // colour, and the client's own styling is where the colour lives.
        let (surface_class, casing_class, apron_class) =
            match paved.material {
                crate::priors::Surface::Ballast => ("rail_surface", "rail_casing", "rail_apron"),
                _ => ("road_surface", "road_casing", "road_apron"),
            };
        // The casing rides after the surface so its blended rim composites over
        // the opaque interior rather than under it.
        let mut push = |class: &str, mesh: crate::terrain::TerrainMesh, bump: u64| {
            buckets[layers::TRANSPORTATION as usize].push(EncoderFeature {
                id: id ^ bump,
                geometry: geo_types::Geometry::Point(geo_types::Point(anchor)),
                properties: vec![
                    ("class".to_string(), Value::String(class.to_string())),
                    ("level".to_string(), Value::Int(paved.level)),
                ],
                elevation: None,
                z: None,
                mesh: Some(mesh),
                synth: synth::Synth::None,
            });
        };
        // Every at-grade region cuts, not just the first: a tile can carry
        // several level-0 regions on different grade layers, and one left uncut
        // is asphalt the burial comes back through. Structures (level != 0)
        // never cut — a deck flies and the ground beneath it stays. The regions
        // handed on are those whose asphalt was *actually meshed*, so a level
        // that failed to mesh leaves no hole with nothing over it (invariant 6).
        if paved.level == 0 && hole && !paved.region.is_empty() {
            cut.push(paved.region);
        }
        push(surface_class, paved.surface, 0);
        if let Some(casing) = paved.casing {
            push(casing_class, casing, 1);
        }
        // The wall between the kerb and the ground beside it. A sibling
        // feature rather than part of the terrain: the terrain mesh carries no
        // materials, so giving it one would take the whole ground out of the
        // client's own styling — and as a road-layer feature the apron is also
        // invisible to the terrain steepness check, which must not read a
        // deliberate wall as a manufactured cliff.
        if let Some(apron) = paved.apron {
            push(apron_class, apron, 2);
        }
    }
    cut
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
    world: &World,
    dem: Option<Dem>,
) -> Result<(ExternalSorter, Stats), Error> {
    let mut sorter = ExternalSorter::new(&cfg.tmp_dir, mem_budget);
    let mut stats = Stats::default();
    // The engineered ground, for the one phase-1 decision that needs it: where
    // a draped feature's elevated span may start (`synth::draped::seat`). It is
    // taken before the cut because moving an abutment moves the boundary
    // between the deck and the path draped up to it, and the cut is here.
    let mut sampler = GroundSampler::new(
        dem,
        Arc::clone(&world.ground),
        world.solved.z_ref,
        mesh_options(cfg),
    );
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
            process_feature(*layer, &feature?, cfg, &mut sorter, &mut stats, world, &mut sampler)?;
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
fn process_feature(
    layer: u8,
    f: &crate::geoparquet::Feature,
    cfg: &Config,
    sorter: &mut ExternalSorter,
    stats: &mut Stats,
    world: &World,
    sampler: &mut GroundSampler,
) -> Result<(), Error> {
    let World { scene, solved, junctions, carriers, .. } = world;
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
            // The corridor's spans are already the solved-reconciled truth —
            // tunnels clamped to their buried runs, the freed slack painted
            // as road up to the portal mouth (`solve::reconcile_stratum`,
            // §4.5) — the same partition the bands, benches and solids cut.
            for piece in corridor.pieces(seg) {
                let line = LineString(piece.line);
                let mut props = seg.properties.clone();
                // The marking synth: at-grade markings drape like the paint;
                // structure markings ride the deck ramp with it.
                let (synth, mark_synth) = match piece.kind {
                    SpanKind::Grade => {
                        let s = Synth::Road {
                            corridor: Some(corridor.id),
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
                        // Which drawn surface this structure's top *is*, named
                        // as a style class so the client paints it the same
                        // colour as the band it continues (docs/ROADS.md
                        // invariant 1: one cross-section, one surface). Without
                        // it the deck top takes its road class's own grey and a
                        // residential bridge is visibly a lighter tone than the
                        // asphalt either side of it. The modality is decided
                        // here because only the server knows it — the archive
                        // carries a class but not its subtype, and a railway
                        // deck belongs to the ballast band, not the asphalt.
                        let surface = match corridor.kind.prior().surface {
                            crate::priors::Surface::Ballast => Some("rail_surface"),
                            crate::priors::Surface::Asphalt => Some("road_surface"),
                            crate::priors::Surface::None => None,
                        };
                        if let Some(s) = surface {
                            props.push((
                                "band_class".to_string(),
                                Value::String(s.to_string()),
                            ));
                        }
                        (Synth::Structure { corridor: corridor.id, kind }, stroke)
                    }
                };
                if let Some((class, oneway, width, areas)) = &marks {
                    for m in synth::markings::for_line(&line, class, *oneway, *width, areas) {
                        emit_geometry(layer, &m.geometry, &m.properties(), mark_synth, cfg, sorter, stats)?;
                    }
                }
                emit_geometry(layer, &Geometry::LineString(line), &props, synth, cfg, sorter, stats)?;
            }
            return Ok(());
        }
        // Unclaimed: a draped feature. It drapes on the rendered ground —
        // except where it carries an elevated span, which is still a bridge
        // even though it is junior to everything (§4.2). Those pieces get a
        // deck fitted to the finished ground (`synth::draped`); nothing here
        // enters a solve.
        let synth = Synth::Road { corridor: None, deck: false };
        if let Some((class, oneway, width, areas)) = &marks {
            if let Geometry::LineString(line) = &f.geometry {
                for m in synth::markings::for_line(line, class, *oneway, *width, areas) {
                    emit_geometry(layer, &m.geometry, &m.properties(), synth, cfg, sorter, stats)?;
                }
            }
        }
        if let Geometry::LineString(line) = &f.geometry {
            if !f.level_runs.is_empty() {
                // Where the span's edge landed on a wall rather than on a bank,
                // the abutment is re-seated on ground that can carry it before
                // the cut is made — otherwise the deck is chorded from a point
                // part way down the gorge it is supposed to cross.
                let runs = synth::draped::seat(line, &f.level_runs, sampler, solved.z_ref);
                for (piece, level) in level_pieces(line, &runs) {
                    let mut props = f.properties.clone();
                    let tag = if level > 0 {
                        // The level ordinal survives as a property so the
                        // client colours it as a structure, exactly as a
                        // solved deck's does.
                        props.push(("level_rules".to_string(), Value::Int(level)));
                        // Unless there is no second bridge: a sidewalk tagged
                        // as its own structure rides the road bridge carrying
                        // it, and stamps nothing (`synth::carried`). Reading
                        // the street stratum's deck is not a promotion — the
                        // path still writes nothing back.
                        match carriers.of(&piece.0, scene, solved, sampler, solved.z_ref) {
                            Some(corridor) => {
                                Synth::Road { corridor: Some(corridor), deck: true }
                            }
                            None => Synth::DrapedDeck,
                        }
                    } else {
                        synth
                    };
                    emit_geometry(
                        layer,
                        &Geometry::LineString(piece),
                        &props,
                        tag,
                        cfg,
                        sorter,
                        stats,
                    )?;
                }
                return Ok(());
            }
        }
        return emit_geometry(layer, &f.geometry, &f.properties, synth, cfg, sorter, stats);
    }
    emit_geometry(layer, &f.geometry, &f.properties, Synth::None, cfg, sorter, stats)
}

/// Cuts a draped feature's line at its level-run boundaries, yielding each
/// piece with the level covering it (0 between and around the runs).
///
/// The runs are fractions of the line's *arc*, so the cut points are
/// interpolated by length rather than by vertex index — a bridge annotated
/// over the middle third of a segment must land where that third actually is,
/// however the vertices are spaced. Pieces share their boundary vertex
/// exactly, so the drape and the deck meet without a gap.
fn level_pieces(line: &LineString, runs: &[crate::levels::LevelRun]) -> Vec<(LineString, i64)> {
    let nodes = &line.0;
    if nodes.len() < 2 {
        return Vec::new();
    }
    let cos_lat = crate::scene::run_cos_lat(nodes);
    let mut arc = Vec::with_capacity(nodes.len());
    let mut acc = 0.0;
    for (i, &c) in nodes.iter().enumerate() {
        if i > 0 {
            acc += crate::scene::metric_len(nodes[i - 1], c, cos_lat);
        }
        arc.push(acc);
    }
    let total = acc;
    if total <= 0.0 {
        return Vec::new();
    }
    // Boundaries in arc metres, deduplicated and clamped to the line.
    let mut cuts = vec![0.0, total];
    for r in runs {
        cuts.push((r.start * total).clamp(0.0, total));
        cuts.push((r.end * total).clamp(0.0, total));
    }
    cuts.sort_by(|a, b| a.partial_cmp(b).expect("finite arc"));
    cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-6);

    let point_at = |d: f64| -> Coord {
        let i = arc.partition_point(|&a| a < d).clamp(1, arc.len() - 1);
        let (a0, a1) = (arc[i - 1], arc[i]);
        let t = if a1 > a0 { ((d - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
        Coord {
            x: nodes[i - 1].x + (nodes[i].x - nodes[i - 1].x) * t,
            y: nodes[i - 1].y + (nodes[i].y - nodes[i - 1].y) * t,
        }
    };

    let mut out = Vec::new();
    for w in cuts.windows(2) {
        let (d0, d1) = (w[0], w[1]);
        let mid = 0.5 * (d0 + d1);
        let level = runs
            .iter()
            .find(|r| mid >= r.start * total && mid <= r.end * total)
            .map_or(0, |r| r.level);
        let mut pts = vec![point_at(d0)];
        pts.extend(nodes.iter().zip(&arc).filter(|(_, &a)| a > d0 && a < d1).map(|(c, _)| *c));
        pts.push(point_at(d1));
        pts.dedup();
        if pts.len() >= 2 {
            out.push((LineString(pts), level));
        }
    }
    out
}

/// The marking inputs for a transportation feature — its class, one-way
/// verdict, derived width, and the paved intersections near it — or `None`
/// when the class's ladder paints nothing (the common case, so the plate
/// query is skipped entirely).
fn marking_context<'a>(
    f: &crate::geoparquet::Feature,
    junctions: &'a CarriagewayModel,
    bb: (f64, f64, f64, f64),
) -> Option<(String, bool, f64, Vec<&'a crate::synth::area::Area>)> {
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
    // Pad by more than any plausible intersection reach (~200 m in degrees).
    const MARGIN: f64 = 0.002;
    let areas = junctions
        .near((bb.0 - MARGIN, bb.1 - MARGIN, bb.2 + MARGIN, bb.3 + MARGIN))
        .into_iter()
        .map(|p| p.area())
        .collect();
    Some((class, oneway, width, areas))
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
fn flush_tile(
    writer: &mut ArchiveWriter<File>,
    tile_id: u64,
    buckets: &mut [Vec<EncoderFeature>],
    layer_stats: &mut LayerStats,
    stats: &mut Stats,
    tile: &TileContext<'_>,
    sampler: &mut GroundSampler,
    elevation: &mut (f64, f64),
) -> Result<(), Error> {
    let TileContext { flat, world, quality } = tile;
    let World { solved, pavement, junctions, .. } = *world;
    let quality = *quality;
    let (z, x, y) = hilbert::tile_id_decode(tile_id);
    let bounds = Bounds::of_tile(z, x, y);
    stamp_elevations(buckets, sampler, z);
    let field = synth::height::HeightField::for_tile(junctions, solved, z, &bounds);
    stamp_synth(buckets, &field, sampler, solved, z, &bounds);
    let cut_regions =
        add_road_surface(buckets, pavement, &field, sampler, &bounds, z, solved.z_ref);

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
        let (mesh, emin, emax) = sampler.terrain_mesh(&bounds, z, &cut_regions);
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
pub(crate) fn tolerance(z: u8) -> f64 {
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

    /// A draped feature's line is cut at its level-run boundaries so the
    /// elevated part can be decked and the rest drapes. The cut is by *arc*,
    /// not by vertex index: a bridge over the middle third must land where the
    /// middle third is.
    #[test]
    fn level_pieces_cut_by_arc_and_abut_exactly() {
        use crate::levels::LevelRun;
        // 4 nodes, unevenly spaced: 0, 10, 90, 100 m along.
        let line = LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0 / crate::scene::DEG_M, y: 0.0 },
            Coord { x: 90.0 / crate::scene::DEG_M, y: 0.0 },
            Coord { x: 100.0 / crate::scene::DEG_M, y: 0.0 },
        ]);
        let runs = vec![LevelRun { start: 0.25, end: 0.75, level: 1 }];
        let pieces = level_pieces(&line, &runs);
        assert_eq!(pieces.len(), 3, "grade, bridge, grade");
        assert_eq!(pieces.iter().map(|(_, l)| *l).collect::<Vec<_>>(), vec![0, 1, 0]);
        // Abutting pieces share their boundary vertex exactly, so the drape
        // meets the deck with no gap.
        assert_eq!(*pieces[0].0 .0.last().unwrap(), pieces[1].0 .0[0]);
        assert_eq!(*pieces[1].0 .0.last().unwrap(), pieces[2].0 .0[0]);
        // The cut is at 25 m and 75 m of arc, not at a vertex.
        let x0 = pieces[1].0 .0[0].x * crate::scene::DEG_M;
        assert!((x0 - 25.0).abs() < 1e-6, "bridge starts at 25 m, got {x0}");
    }

    /// A line with no level runs at all yields nothing to cut — the caller
    /// emits it whole.
    #[test]
    fn level_pieces_of_an_unannotated_line_is_one_grade_piece() {
        let line = LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 100.0 / crate::scene::DEG_M, y: 0.0 },
        ]);
        let pieces = level_pieces(&line, &[]);
        assert_eq!(pieces.len(), 1);
        assert_eq!(pieces[0].1, 0);
    }

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
            verify_model: None,
            breaklines: true,
            hole: true,
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

    /// Dumps a raster of the engineered ground against the raw DEM over a
    /// window, as CSV, so the ground field can be hillshaded and diffed
    /// offline — the stage-3 counterpart of `--dump`'s vector artifacts.
    /// Columns: `col,row,lon,lat,raw,ground,bed` (`bed` = the exact roadbed
    /// target where the point lies inside a held bench width, else `nan`).
    /// Run:
    /// ```text
    /// SEG=data/overture-ch/segment.parquet WATER=data/overture-ch/water.parquet \
    /// TERRAIN=data/overture-ch/terrain-hires.pmtiles BBOX=6.885,46.446,6.908,46.462 \
    /// WINDOW=6.8952,46.4539,600 STEP=1 ZREF=16 OUT=/tmp/claude/ground.csv \
    /// cargo test --release -- --ignored dump_ground_grid --nocapture
    /// ```
    #[test]
    #[ignore = "needs $SEG/$TERRAIN"]
    fn dump_ground_grid() {
        let seg = std::env::var("SEG").expect("SEG=<segment.parquet>");
        let water = std::env::var("WATER").ok();
        let terrain = std::env::var("TERRAIN").ok().map(PathBuf::from);
        let bbox: Vec<f64> = std::env::var("BBOX")
            .expect("BBOX=w,s,e,n")
            .split(',')
            .map(|s| s.trim().parse().unwrap())
            .collect();
        let bounds =
            Bounds { west: bbox[0], south: bbox[1], east: bbox[2], north: bbox[3] };
        let window: Vec<f64> = std::env::var("WINDOW")
            .expect("WINDOW=lon,lat,size_m")
            .split(',')
            .map(|s| s.trim().parse().unwrap())
            .collect();
        let step_m: f64 = std::env::var("STEP").ok().and_then(|s| s.parse().ok()).unwrap_or(1.0);
        let z_ref: u8 = std::env::var("ZREF").ok().and_then(|s| s.parse().ok()).unwrap_or(16);
        let out = std::env::var("OUT").unwrap_or_else(|_| "/tmp/ground.csv".into());

        use std::path::Path as FsPath;
        let mut scene =
            assemble::run(FsPath::new(&seg), water.as_deref().map(FsPath::new), &bounds)
                .expect("assemble");
        let solved =
            Arc::new(solve::run(&mut scene, terrain.as_deref(), z_ref, 0).expect("solve"));
        let ground = Arc::new(ground::derive(&scene, &solved, terrain.as_deref(), 0));
        eprintln!(
            "model: {} corridors, {} profiles, {} earthwork edges, {} breakline segments",
            scene.corridors.len(),
            solved.solved_count(),
            ground.earthwork_count(),
            ground.breaklines().len(),
        );

        let (clon, clat, size) = (window[0], window[1], window[2]);
        let cos = clat.to_radians().cos();
        let n = (size / step_m).round() as i64;
        let mut dem = terrain.as_deref().and_then(|p| Dem::open(p).ok());
        let mut sampler = GroundSampler::new(
            terrain.as_deref().and_then(|p| Dem::open(p).ok()),
            Arc::clone(&ground),
            z_ref,
            ground::sampler::MeshOptions::default(),
        );
        let mut csv = String::from("col,row,lon,lat,raw,ground,bed\n");
        for row in 0..=n {
            let lat = clat + (row as f64 * step_m - size * 0.5) / crate::scene::DEG_M;
            for col in 0..=n {
                let lon =
                    clon + (col as f64 * step_m - size * 0.5) / (crate::scene::DEG_M * cos);
                let raw = match &mut dem {
                    Some(d) => d.elevation(lon, lat, z_ref),
                    None => 0.0,
                };
                let g = sampler.ground(lon, lat, z_ref);
                let bed = sampler.bed_target(lon, lat);
                csv.push_str(&format!(
                    "{col},{row},{lon:.7},{lat:.7},{raw:.3},{g:.3},{}\n",
                    match bed {
                        Some(b) => format!("{b:.3}"),
                        None => "nan".into(),
                    }
                ));
            }
        }
        std::fs::write(&out, csv).expect("write csv");
        eprintln!("wrote {out} ({}x{} samples at {step_m} m)", n + 1, n + 1);
    }

    /// Dumps one archive tile's terrain mesh as CSV — vertex position, height,
    /// and decoded normal, plus the triangle list — so the drawn geometry and
    /// the shading it carries can be plotted apart from each other.
    /// Run: `ARPA=data/overture-ch/preview.arpa AT=6.928,46.437 Z=16 \
    ///   OUT=/tmp/claude/mesh cargo test --release -- --ignored dump_terrain_mesh --nocapture`
    #[test]
    #[ignore = "needs $ARPA/$AT"]
    fn dump_terrain_mesh() {
        let path = std::env::var("ARPA").unwrap();
        let at: Vec<f64> = std::env::var("AT")
            .expect("AT=lon,lat")
            .split(',')
            .map(|s| s.trim().parse().unwrap())
            .collect();
        let z: u8 = std::env::var("Z").ok().and_then(|s| s.parse().ok()).unwrap_or(16);
        let out = std::env::var("OUT").unwrap_or_else(|_| "/tmp/mesh".into());
        let bytes = std::fs::read(&path).unwrap();
        let archive = crate::archive::Archive::open(&bytes).unwrap();
        let entry = archive
            .entries()
            .find(|e| {
                if e.z != z {
                    return false;
                }
                let b = Bounds::of_tile(e.z, e.x, e.y);
                at[0] >= b.west && at[0] < b.east && at[1] >= b.south && at[1] < b.north
            })
            .expect("a tile covering AT");
        let b = Bounds::of_tile(entry.z, entry.x, entry.y);
        let raw = brotli_decompress(archive.get_by_id(entry.hilbert_id).unwrap());
        let tile = fbt::root_as_tile(&raw).unwrap();
        let layers = tile.layers().unwrap();
        let mut wrote = false;
        for li in 0..layers.len() {
            let l = layers.get(li);
            if l.name() != "terrain" {
                continue;
            }
            let feats = l.features().unwrap();
            let f = feats.get(0);
            let g = f.geometry_as_mesh_geometry().expect("a terrain mesh");
            let (x, y, zq) = (g.x(), g.y(), g.z());
            let normals = g.normals();
            let idx = g.indices();
            eprintln!(
                "tile {}/{}/{} bounds {:.5},{:.5}..{:.5},{:.5}: {} verts, {} tris",
                entry.z,
                entry.x,
                entry.y,
                b.west,
                b.south,
                b.east,
                b.north,
                x.len(),
                idx.len() / 3,
            );
            let mut v = String::from("i,qx,qy,lon,lat,z_m,nx,ny\n");
            for i in 0..x.len() {
                let (qx, qy) = (x.get(i), y.get(i));
                let (nx, ny) = match &normals {
                    Some(n) if n.len() >= (i + 1) * 2 => (n.get(i * 2), n.get(i * 2 + 1)),
                    _ => (0, 0),
                };
                v.push_str(&format!(
                    "{i},{qx},{qy},{:.7},{:.7},{:.3},{nx},{ny}\n",
                    crate::project::dequantize_x(qx, &b),
                    crate::project::dequantize_y(qy, &b),
                    zq.get(i) as f64 / 1000.0,
                ));
            }
            std::fs::write(format!("{out}_verts.csv"), v).unwrap();
            let mut t = String::from("a,b,c\n");
            for tri in 0..idx.len() / 3 {
                t.push_str(&format!(
                    "{},{},{}\n",
                    idx.get(tri * 3),
                    idx.get(tri * 3 + 1),
                    idx.get(tri * 3 + 2)
                ));
            }
            std::fs::write(format!("{out}_tris.csv"), t).unwrap();
            wrote = true;
        }
        assert!(wrote, "no terrain layer in that tile");
        eprintln!("wrote {out}_verts.csv / {out}_tris.csv");
    }

    /// Audits how far the solved at-grade road stands off the natural ground,
    /// and how much of that stands next to a mapped structure — the S10
    /// question: is a 12 m "embankment" beside a bridge span really an
    /// embankment, or a viaduct whose annotation stopped short?
    /// Run: same env as `dump_ground_grid` (no WINDOW needed).
    #[test]
    #[ignore = "needs $SEG/$TERRAIN"]
    fn audit_at_grade_standoff() {
        use std::path::Path as FsPath;
        let seg = std::env::var("SEG").expect("SEG=<segment.parquet>");
        let water = std::env::var("WATER").ok();
        let terrain = std::env::var("TERRAIN").ok().map(PathBuf::from);
        let bbox: Vec<f64> = std::env::var("BBOX")
            .expect("BBOX=w,s,e,n")
            .split(',')
            .map(|s| s.trim().parse().unwrap())
            .collect();
        let bounds = Bounds { west: bbox[0], south: bbox[1], east: bbox[2], north: bbox[3] };
        let z_ref: u8 = std::env::var("ZREF").ok().and_then(|s| s.parse().ok()).unwrap_or(16);
        let mut scene = assemble::run(FsPath::new(&seg), water.as_deref().map(FsPath::new), &bounds)
            .expect("assemble");
        let solved = solve::run(&mut scene, terrain.as_deref(), z_ref, 0).expect("solve");

        // CORRIDOR=<id> ARCS=lo,hi dumps one profile node by node instead.
        if let Ok(id) = std::env::var("CORRIDOR") {
            let id: u32 = id.parse().unwrap();
            let span: Vec<f64> = std::env::var("ARCS")
                .unwrap_or_else(|_| "0,1e9".into())
                .split(',')
                .map(|s| s.trim().parse().unwrap())
                .collect();
            let c = scene.corridors.iter().find(|c| c.id == id).expect("corridor");
            let p = solved.profile(id).expect("a profile");
            eprintln!("corridor {id} {} spans={:?}", c.class_key, c.spans);
            // The same corridor straight out of stage 2's per-corridor solve,
            // before the fused relax — so a hump can be blamed on one or the
            // other.
            let pre = terrain.as_deref().and_then(|t| Dem::open(t).ok()).and_then(|d| {
                let mut d = d;
                solve::profile::solve(
                    &c.nodes,
                    &c.spans,
                    solve::Mode::for_kind(c.kind),
                    &mut |q| solve::reference_surface(&mut d, z_ref, q.x, q.y),
                )
            });
            for (i, &a) in p.arc().iter().enumerate() {
                if a < span[0] || a > span[1] {
                    continue;
                }
                eprintln!(
                    "  arc {a:>8.1}  road={:>8.2} pre_relax={:>8.2} terrain={:>8.2} \
                     standoff={:>+6.2} {}",
                    p.road_m()[i],
                    pre.as_ref().map(|q| q.road_m()[i]).unwrap_or(f64::NAN),
                    p.terrain_m()[i],
                    p.road_m()[i] - p.terrain_m()[i],
                    if p.at_grade()[i] { "grade" } else { "STRUCTURE" },
                );
            }
            eprintln!("crossings on this corridor in range:");
            for x in &solved.crossings {
                let mine = if x.upper == id {
                    Some((x.upper_arc, "upper"))
                } else if x.lower == Some(id) {
                    let p2 = solved.profile(id).unwrap();
                    Some((p2.arc_of(x.point.x, x.point.y), "lower"))
                } else {
                    None
                };
                let Some((a, role)) = mine else { continue };
                if a < span[0] || a > span[1] {
                    continue;
                }
                let other = if role == "upper" { x.lower } else { Some(x.upper) };
                let (oclass, oroad) = match other.and_then(|o| {
                    scene.corridors.iter().find(|c| c.id == o).map(|c| (c, o))
                }) {
                    Some((c, o)) => (
                        c.class_key.clone(),
                        solved.profile(o).map(|q| q.height_at(x.point.x, x.point.y)),
                    ),
                    None => ("(terrain/water)".into(), None),
                };
                eprintln!(
                    "  arc {a:>8.1} {role:<5} vs {:?} {oclass:<14} kind={:?} levels {}/{} other_road={:?}",
                    other, x.lower_kind, x.upper_level, x.lower_level, oroad,
                );
            }
            return;
        }

        let mut standoff: Vec<f64> = Vec::new();
        let mut near_structure = 0usize;
        let mut lone = 0usize;
        let mut worst: Vec<(f64, u32, String, f64, bool)> = Vec::new();
        for c in &scene.corridors {
            let Some(p) = solved.profile(c.id) else { continue };
            let (arc, road, terr, at_grade) =
                (p.arc(), p.road_m(), p.terrain_m(), p.at_grade());
            for i in 0..road.len() {
                if !at_grade[i] {
                    continue;
                }
                let up = road[i] - terr[i];
                standoff.push(up);
                if up < 4.0 {
                    continue;
                }
                // Is a mapped structure span within 200 m along the corridor?
                let near = c.spans.iter().any(|s| {
                    s.kind != SpanKind::Grade
                        && arc[i] >= s.arc0 - 200.0
                        && arc[i] <= s.arc1 + 200.0
                });
                if near {
                    near_structure += 1;
                } else {
                    lone += 1;
                }
                worst.push((up, c.id, c.class_key.clone(), arc[i], near));
            }
        }
        standoff.sort_by(|a, b| a.total_cmp(b));
        let pct = |q: f64| standoff[((standoff.len() as f64 * q) as usize).min(standoff.len() - 1)];
        eprintln!(
            "{} at-grade nodes: standoff p50 {:+.2} p90 {:+.2} p99 {:+.2} max {:+.2} min {:+.2}",
            standoff.len(),
            pct(0.5),
            pct(0.9),
            pct(0.99),
            standoff[standoff.len() - 1],
            standoff[0],
        );
        eprintln!(
            "flying > 4 m at grade: {} nodes — {near_structure} within 200 m of a mapped \
             structure, {lone} standing alone",
            near_structure + lone
        );
        worst.sort_by(|a, b| b.0.total_cmp(&a.0));
        for (up, id, class, arc, near) in worst.iter().take(12) {
            eprintln!(
                "  +{up:.1} m  corr {id:<6} {class:<12} arc {arc:.0}  {}",
                if *near { "beside a structure" } else { "ALONE" }
            );
        }
    }

    /// Explains the engineered ground at one point: every earthwork edge that
    /// covers it (which corridor, what target, how strong a share) and every
    /// corridor whose profile passes nearby (class, span kind, solved road
    /// height against the terrain it was solved from). The "why is the ground
    /// here 12 m above the DEM" probe.
    /// Run: same env as `dump_ground_grid` plus `PROBE=lon,lat`.
    #[test]
    #[ignore = "needs $SEG/$TERRAIN/$PROBE"]
    fn probe_ground_point() {
        use std::path::Path as FsPath;
        let seg = std::env::var("SEG").expect("SEG=<segment.parquet>");
        let water = std::env::var("WATER").ok();
        let terrain = std::env::var("TERRAIN").ok().map(PathBuf::from);
        let bbox: Vec<f64> = std::env::var("BBOX")
            .expect("BBOX=w,s,e,n")
            .split(',')
            .map(|s| s.trim().parse().unwrap())
            .collect();
        let bounds = Bounds { west: bbox[0], south: bbox[1], east: bbox[2], north: bbox[3] };
        let probe: Vec<f64> = std::env::var("PROBE")
            .expect("PROBE=lon,lat")
            .split(',')
            .map(|s| s.trim().parse().unwrap())
            .collect();
        let (lon, lat) = (probe[0], probe[1]);
        let z_ref: u8 = std::env::var("ZREF").ok().and_then(|s| s.parse().ok()).unwrap_or(16);

        let mut scene = assemble::run(FsPath::new(&seg), water.as_deref().map(FsPath::new), &bounds)
            .expect("assemble");
        let solved = Arc::new(solve::run(&mut scene, terrain.as_deref(), z_ref, 0).expect("solve"));
        let ground = Arc::new(ground::derive(&scene, &solved, terrain.as_deref(), 0));
        let mut dem = terrain.as_deref().and_then(|p| Dem::open(p).ok());
        let raw = match &mut dem {
            Some(d) => d.elevation(lon, lat, z_ref),
            None => 0.0,
        };
        let mut sc = Vec::new();
        let h = ground.height(lon, lat, raw, 0.0, &mut sc);
        eprintln!("PROBE {lon},{lat}  raw={raw:.2} engineered={h:.2} (delta {:+.2} m)", h - raw);

        // Covering earthwork edges, strongest share first.
        let cos = lat.to_radians().cos();
        let mut rows: Vec<(f64, String)> = Vec::new();
        for e in ground.layers().iter().flat_map(|l| l.earthworks().edges()) {
            let ax = e.a.x * e.cos_lat;
            let (dx, dy) = ((e.b.x - e.a.x) * e.cos_lat, e.b.y - e.a.y);
            let len2 = dx * dx + dy * dy;
            let t = if len2 > 0.0 {
                (((lon * e.cos_lat - ax) * dx + (lat - e.a.y) * dy) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let (cx, cy) = (ax + dx * t, e.a.y + dy * t);
            let d = ((lon * e.cos_lat - cx).powi(2) + (lat - cy).powi(2)).sqrt()
                * crate::scene::DEG_M;
            if d >= e.reach_m() {
                continue;
            }
            // The face this edge actually draws here, on the side the point is
            // on: the model's own `batter_run`, not a fixed EARTHWORK_BATTER —
            // a diverging face is rebuilt as a wall and a probe that assumes
            // the earth slope explains a shape that is not there.
            let side = if dx * (lat - e.a.y) - dy * (lon * e.cos_lat - ax) >= 0.0 { 0 } else { 1 };
            let rise = (d - e.half_width_m).max(0.0) / e.batter_run[side];
            let target = e.target_a + (e.target_b - e.target_a) * t;
            let class = scene
                .corridors
                .iter()
                .find(|c| c.id == e.chain)
                .map(|c| c.class_key.clone())
                .unwrap_or_default();
            rows.push((
                d,
                format!(
                    "  {:<14} chain={:<6} arc0={:<7.0} d={:>6.2} hw={:>5.2} cw={:>5.2} \
                     {} reach={:.1} run=1:{:.2} target={:>8.2} {}{}",
                    class,
                    e.chain,
                    e.arc0,
                    d,
                    e.half_width_m,
                    e.carriageway_m,
                    if side == 0 { "L" } else { "R" },
                    e.batter_m[side],
                    e.batter_run[side],
                    target,
                    if d <= e.carriageway_m {
                        "CARRIAGEWAY".to_string() // outranks any nearer verge
                    } else if d <= e.half_width_m {
                        "bench (verge)".to_string()
                    } else {
                        format!("face fill={:.2} cut={:.2}", target - rise, target + rise)
                    },
                    if e.carve { "  CARVE" } else { "" },
                ),
            ));
        }
        rows.sort_by(|a, b| a.0.total_cmp(&b.0));
        eprintln!("{} covering earthwork edges:", rows.len());
        for (_, line) in rows.iter().take(30) {
            eprintln!("{line}");
        }

        // Corridors whose centerline passes within 40 m.
        eprintln!("nearby corridors (centerline within 40 m):");
        for c in &scene.corridors {
            let Some(p) = solved.profile(c.id) else { continue };
            let a = p.arc_of(lon, lat);
            let pt = p.point_at_arc(a);
            let d = ((pt.x - lon) * cos).hypot(pt.y - lat) * crate::scene::DEG_M;
            if d > 40.0 {
                continue;
            }
            let span = c.spans.iter().find(|s| a >= s.arc0 && a <= s.arc1);
            eprintln!(
                "  corr {:<6} {:<14} d={:>6.2} arc={:>7.1} road={:>8.2} terrain={:>8.2} \
                 deck={:>8.2} span={:?}",
                c.id,
                c.class_key,
                d,
                a,
                p.height_at(lon, lat),
                p.surface_at(lon, lat),
                p.deck_height_at(lon, lat),
                span.map(|s| (s.kind, s.level)),
            );
        }
    }
}
