//! The unioned road surface — one paved region per level per chunk
//! (docs/ROADS.md §6.1, invariant 2).
//!
//! The surface stops being a pile of per-road bands plus per-intersection plates
//! and becomes one region: every carriageway buffered to its own width, unioned,
//! its reflex corners rounded into curb returns. That is what makes ROADS.md
//! invariant 2 — no gaps, no slivers, no overlapping fills — true by construction
//! instead of arbitrated by a sink and a tuck.
//!
//! **Chunks are z13 tile rects.** The union is baked once, globally, not per
//! tile. Every zoom that draws asphalt is at or past
//! [`priors::ROAD_SURFACE_MIN_ZOOM`] and the tile grid nests, so **every such
//! tile lies wholly inside exactly one chunk**: a tile clip never spans a chunk
//! edge, and one region boundary serves every zoom and every tile that reads it.
//! A per-tile union would instead re-run the boolean for each of the ~5–20 tiles
//! covering the same ground, and would have to reproduce bit-identical output
//! from different padded input sets to keep its seams closed.
//!
//! Two neighbouring chunks agree at their shared edge because the union is
//! clipped to the chunk rect and the cut coordinates are then snapped to the
//! rect's own longitude/latitude — a literal both chunks compute the same way
//! from [`Bounds::of_tile`] — so the seam is bit-identical from either side
//! rather than merely close.
//!
//! Fillets come from a closing that is *local*: see [`poly::close_within`]. A
//! global dilate/erode at the curb-return radius would bridge any gap under
//! twice that radius, fusing the two carriageways of a divided road into one
//! slab and swallowing narrow medians. Restricting it to the intersection
//! extents puts fillets exactly where roads meet and leaves everything else
//! untouched.

use std::collections::HashMap;

use geo_types::Coord;

use crate::priors::{self, PAVE_BAKE_Z, PAVE_PAD_M};
use crate::project::Bounds;
use crate::scene::DEG_M;
use crate::assemble::facades::Facades;
use crate::assemble::facades::Section;
use crate::synth::carriageway::{CarriagewayModel, Handover, SourceSeg};
use crate::synth::poly::{self, MFrame, Pt, Shapes};
use crate::synth::walkway::NO_HOST;

/// A chunk key: the `(x, y)` of its z13 tile.
pub type ChunkKey = (u32, u32);

/// One level's paved region inside a chunk, as lon/lat rings.
pub struct LevelShapes {
    pub level: i64,
    /// The grade-separation layer this region belongs to
    /// ([`crate::synth::carriageway::SourceSeg::layer`]). Regions on different
    /// layers overlap in plan but are metres apart vertically, so they are
    /// separate regions that occlude each other rather than one merged surface.
    pub layer: u32,
    /// The material of this region ([`priors::Surface`]): asphalt and ballast
    /// are separate regions, emitted as separate features, so a rail
    /// formation renders in its own colour instead of merging into the
    /// carriageway that crosses it. Never [`priors::Surface::None`].
    pub surface: priors::Surface,
    /// Outer boundaries counter-clockwise, holes clockwise — `i_overlay`'s
    /// convention, preserved so the mesher can tell them apart without a
    /// winding test.
    pub shapes: Vec<Vec<Vec<Coord>>>,
}

/// The baked road surface: paved regions per chunk, shared by the emit workers
/// through an `Arc`.
pub struct PavementModel {
    chunks: HashMap<ChunkKey, Vec<LevelShapes>>,
    /// The bench under the sidewalk ring ([`walk_ring`]): one band segment per
    /// stretch of kerb the ring runs along, offset half a pavement out from
    /// the kerb and seated at the kerb's road height plus the rise — what
    /// stratum D benches, so the ring's outer edge meets the ground at its own
    /// height (docs/GROUND.md §2, "the ground under a walkway is the
    /// walkway"). One derivation, two readers: the drawn ring and its bench
    /// are the same offset of the same kerb.
    ring_benches: Vec<SourceSeg>,
    /// The abutment cuts ([`crate::synth::carriageway::Handover`]) reaching each
    /// chunk, so the mesher can tell the boundary where a deck takes over from
    /// the boundary where the ground does. Kept beside the shapes rather than
    /// inside them because a cut belongs to the *pair* of regions it separates,
    /// and one of the two is not in this model at all.
    handovers: HashMap<ChunkKey, Vec<Handover>>,
}

impl PavementModel {
    /// The chunk containing a tile, if any asphalt was baked there. A tile at or
    /// past [`PAVE_BAKE_Z`] lies wholly inside one chunk, so this is the only
    /// lookup a mesher needs.
    pub fn chunk_for(&self, bounds: &Bounds) -> Option<&[LevelShapes]> {
        let c = chunk_of(bounds.west + 0.5 * bounds.width(), bounds.south + 0.5 * bounds.height());
        self.chunks.get(&c).map(|v| v.as_slice())
    }

    /// The abutment cuts of the chunk containing a tile.
    pub fn handovers_for(&self, bounds: &Bounds) -> &[Handover] {
        let c = chunk_of(bounds.west + 0.5 * bounds.width(), bounds.south + 0.5 * bounds.height());
        self.handovers.get(&c).map_or(&[], |v| v.as_slice())
    }

    /// The sidewalk ring's bench segments, in chunk order (invariant 5).
    pub fn ring_benches(&self) -> &[SourceSeg] {
        &self.ring_benches
    }

    /// Number of chunks carrying asphalt.
    pub fn chunk_count(&self) -> usize {
        self.chunks.len()
    }

    /// Total paved area in square metres, over every chunk and level — the
    /// coarse "did the union actually union" check.
    pub fn area_m2(&self) -> f64 {
        let mut total = 0.0;
        for (&(cx, cy), levels) in &self.chunks {
            let frame = MFrame::of(chunk_centre(cx, cy));
            for ls in levels {
                let metres: Shapes = ls
                    .shapes
                    .iter()
                    .map(|sh| sh.iter().map(|r| r.iter().map(|&c| frame.to_m(c)).collect()).collect())
                    .collect();
                total += poly::area(&metres);
            }
        }
        total
    }
}

/// The metre frame a tile's rings were baked in — the frame of the chunk
/// containing the tile ([`bake_chunk`]'s own `MFrame::of(chunk_centre)`), so
/// a reader can put a ring vertex back on the exact grid the boolean snapped
/// it to. Anything that wants to test a baked edge *exactly* (rather than
/// within a tolerance) has to work in this frame, not the tile's.
pub fn chunk_frame_for(bounds: &Bounds) -> MFrame {
    let (cx, cy) = chunk_of(bounds.west + 0.5 * bounds.width(), bounds.south + 0.5 * bounds.height());
    MFrame::of(chunk_centre(cx, cy))
}

/// The z13 chunk containing a world point.
fn chunk_of(lon: f64, lat: f64) -> ChunkKey {
    let n = 1u32 << PAVE_BAKE_Z;
    let lon_span = 360.0 / n as f64;
    let lat_span = 180.0 / n as f64;
    let x = ((lon + 180.0) / lon_span).floor().clamp(0.0, (n - 1) as f64) as u32;
    let y = ((lat + 90.0) / lat_span).floor().clamp(0.0, (n - 1) as f64) as u32;
    (x, y)
}

/// A chunk's rect.
fn chunk_bounds(cx: u32, cy: u32) -> Bounds {
    Bounds::of_tile(PAVE_BAKE_Z, cx, cy)
}

fn chunk_centre(cx: u32, cy: u32) -> Coord {
    let b = chunk_bounds(cx, cy);
    Coord { x: b.west + 0.5 * b.width(), y: b.south + 0.5 * b.height() }
}

/// Bakes the paved region of every chunk the network touches, in parallel.
///
/// Deterministic: chunks are processed in sorted key order, each chunk's sources
/// are taken in the model's own (corridor, node) order, and the boolean itself is
/// a function of the input *set* rather than of the order it was collected in
/// (see [`poly::union_all`]).
/// What the bake needs to answer yield questions from the *blended field*
/// instead of each band's own frozen chord (S3, `ARPT_FIELD_YIELDS=1`): the
/// solved model and the ground, from which each bake worker builds its own
/// sampler and each chunk its own `HeightField`.
pub struct FieldYields<'a> {
    pub solved: &'a crate::solve::SolvedModel,
    pub ground: std::sync::Arc<crate::ground::GroundStack>,
    pub terrain: Option<std::path::PathBuf>,
    pub mesh: crate::ground::sampler::MeshOptions,
    pub z_ref: u8,
}

/// The S3 switch: plan-space yields sample the blended sheet field the mesher
/// drapes, so the yield agrees with what is drawn. Off by default until the
/// bake reorder is judged (`order.walk_on_asphalt`'s 0.42 % residue).
fn field_yields() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("ARPT_FIELD_YIELDS").is_some())
}

/// Whether the sidewalk is drawn as the **ring** of the paved union rather
/// than as a buffer of each street's own offset band — the default since
/// 2026-09-04; `ARPT_NO_WALK_RING=1` reverts to the bands for an A/B
/// (`data/plans/kerb-ring-2026-09-03.md`).
///
/// A hosted walk band is an offset of one street's centerline, so it ends
/// where that street's arc ends: at a junction every leg's pavement stops
/// short of the corner, and around a roundabout — a dozen ring arcs no
/// attachment survives on — the whole outer kerb is bare. The union's boundary
/// has none of those ends. Grown by a pavement's width, less the asphalt, less
/// the buildings grown by their clearance, it is the kerb line's own sidewalk:
/// continuous around a corner and a ring by construction, and its outer edge
/// follows the facades as a curve instead of the width ladder's steps.
///
/// The bands are not discarded: they still carry the seats, the bench and the
/// sheet of everything pedestrian. Here they become the **mask** that says
/// where along the kerb the ring is pavement at all.
pub fn walk_ring() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("ARPT_NO_WALK_RING").is_none())
}

/// Bracket step for measuring the ring's own width outward from the kerb,
/// metres. Bisected three times after the bracket, so a width lands within
/// about 3 cm — a tenth of the ladder rung the measurement replaced.
const RING_WIDTH_STEP_M: f64 = 0.25;

/// Slack around a hosted band's own drawn width when it masks the ring, metres
/// — the smoothing displacement between the band's centerline and the union's
/// kerb, so a band a hand's breadth off its kerb still masks the ring beside it.
const RING_MASK_SLACK_M: f64 = 0.5;

/// How far past its ends a band's mask reaches, metres: around the fillet a
/// corner is cut at and the pavement's own width beyond it, so two legs' masks
/// meet across the corner arc between them.
const RING_CORNER_M: f64 = priors::CURB_RETURN_M + priors::WALK_WIDTH_M;

/// What the sidewalk ring needs to fit itself to the ground: the senior strata
/// (`ground::derive_seniors`) and the DEM the bands are fitted against, so the
/// ring's bench is narrowed or refused by exactly the rule a band's is
/// (`synth::walkway::fit_to_ground`) and the drawn ring follows the width that
/// came out.
pub struct RingContext<'a> {
    pub seniors: &'a [crate::ground::GroundLayer],
    pub terrain: Option<&'a std::path::Path>,
    pub z_ref: u8,
    /// For the sheet assignment over the ring's benches (`sheets::assign`
    /// reads the junction ports): two rings a storey apart that meet in plan
    /// — either side of a wall the union merged two terraces across — are
    /// two sheets, or they fuse into one mesh with the wall drawn as a cliff
    /// inside it (8 m at 6.9281,46.4177).
    pub scene: &'a crate::scene::SceneGraph,
}

pub fn bake(
    junctions: &CarriagewayModel,
    threads: usize,
    field: Option<&FieldYields>,
    walls: Option<&Facades>,
    ring: Option<&RingContext>,
) -> PavementModel {
    // Which chunks each carriageway segment can influence: its own extent plus
    // the pad, since a union boundary inside a chunk can be moved by geometry
    // just outside it.
    let mut by_chunk: HashMap<ChunkKey, Vec<u32>> = HashMap::new();
    for i in 0..junctions.source_count() as u32 {
        let s = junctions.source(i);
        let pad_lat = (PAVE_PAD_M + s.half_m) / DEG_M;
        let pad_lon = pad_lat / s.cos_lat.max(1e-6);
        let (x0, y0) = chunk_of(s.a.x.min(s.b.x) - pad_lon, s.a.y.min(s.b.y) - pad_lat);
        let (x1, y1) = chunk_of(s.a.x.max(s.b.x) + pad_lon, s.a.y.max(s.b.y) + pad_lat);
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                by_chunk.entry((cx, cy)).or_default().push(i);
            }
        }
    }
    let mut keys: Vec<ChunkKey> = by_chunk.keys().copied().collect();
    keys.sort_unstable();
    if std::env::var_os("ARPT_PAVE_PROBE").is_some() {
        let mut counts: Vec<usize> = keys.iter().map(|k| by_chunk[k].len()).collect();
        counts.sort_unstable();
        let total: usize = counts.iter().sum();
        eprintln!(
            "[pave] {} chunks, {} source-refs, per-chunk min {} median {} max {}",
            keys.len(),
            total,
            counts.first().copied().unwrap_or(0),
            counts.get(counts.len() / 2).copied().unwrap_or(0),
            counts.last().copied().unwrap_or(0),
        );
    }

    // Same fan-out shape as the solve (`solve/mod.rs`): a shared cursor, one
    // worker per thread, results collected under a mutex and reassembled by key.
    let next = std::sync::Mutex::new(0usize);
    let out: std::sync::Mutex<Vec<(ChunkKey, Vec<LevelShapes>, Vec<SourceSeg>)>> =
        std::sync::Mutex::new(Vec::new());
    let threads = threads.max(1).min(keys.len().max(1));
    let field = if field_yields() { field } else { None };
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| {
                // One sampler per worker (its own DEM handle and caches), one
                // field per chunk — the "sampler-backed chunk field".
                let mut sampler = field.map(|f| {
                    let dem = f
                        .terrain
                        .as_deref()
                        .and_then(|p| crate::dem::Dem::open(p).ok());
                    crate::ground::sampler::GroundSampler::new(
                        dem,
                        std::sync::Arc::clone(&f.ground),
                        f.z_ref,
                        f.mesh,
                    )
                });
                loop {
                let k = {
                    let mut n = next.lock().expect("pavement queue poisoned");
                    if *n >= keys.len() {
                        break;
                    }
                    let i = *n;
                    *n += 1;
                    keys[i]
                };
                let t = std::time::Instant::now();
                let chunk_field = field.map(|f| {
                    crate::synth::height::HeightField::for_tile(
                        junctions,
                        f.solved,
                        f.z_ref,
                        &chunk_bounds(k.0, k.1),
                    )
                });
                let mut fy = match (&chunk_field, &mut sampler, field) {
                    (Some(hf), Some(sm), Some(f)) => Some((hf, sm, f.z_ref)),
                    _ => None,
                };
                let (levels, benches) = bake_chunk(junctions, k, &by_chunk[&k], &mut fy, walls, ring);
                if std::env::var_os("ARPT_PAVE_PROBE").is_some() && t.elapsed().as_millis() > 200 {
                    eprintln!(
                        "[pave] chunk {:?}: {} sources -> {} levels in {:?}",
                        k,
                        by_chunk[&k].len(),
                        levels.len(),
                        t.elapsed()
                    );
                }
                if !levels.is_empty() || !benches.is_empty() {
                    out.lock().expect("pavement results poisoned").push((k, levels, benches));
                }
                }
            });
        }
    });

    let mut baked = out.into_inner().expect("pavement results poisoned");
    baked.sort_by_key(|(k, _, _)| *k);
    let mut chunks: HashMap<ChunkKey, Vec<LevelShapes>> = HashMap::new();
    let mut ring_benches: Vec<SourceSeg> = Vec::new();
    for (k, levels, benches) in baked {
        if !levels.is_empty() {
            chunks.insert(k, levels);
        }
        ring_benches.extend(benches);
    }

    // Abutment cuts, filed under every chunk they reach. A cut is a short
    // segment, so both its ends' chunks — and any between — are enough; the
    // rect a chunk meshes is its own, and a cut on the seam is wanted by both
    // sides.
    let mut handovers: HashMap<ChunkKey, Vec<Handover>> = HashMap::new();
    for h in junctions.handovers() {
        let (x0, y0) = chunk_of(h.a.x.min(h.b.x), h.a.y.min(h.b.y));
        let (x1, y1) = chunk_of(h.a.x.max(h.b.x), h.a.y.max(h.b.y));
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                if chunks.contains_key(&(cx, cy)) {
                    handovers.entry((cx, cy)).or_default().push(*h);
                }
            }
        }
    }
    PavementModel { chunks, handovers, ring_benches }
}

/// Bakes one chunk: buffer every source, union per level, round the curb returns
/// at the intersections, clip to the chunk rect.
fn bake_chunk(
    junctions: &CarriagewayModel,
    key: ChunkKey,
    source_ids: &[u32],
    field: &mut Option<(
        &crate::synth::height::HeightField,
        &mut crate::ground::sampler::GroundSampler,
        u8,
    )>,
    walls: Option<&Facades>,
    ring_ctx: Option<&RingContext>,
) -> (Vec<LevelShapes>, Vec<SourceSeg>) {
    let (cx, cy) = key;
    let rect = chunk_bounds(cx, cy);
    let frame = MFrame::of(chunk_centre(cx, cy));
    let ring_on = walk_ring();
    // The hosted walk bands, buffered: the sidewalk ring's mask (see
    // [`walk_ring`]). Keyed by the level and the sheet of the **asphalt the
    // band borders** ([`host_layer`]) — that is the region the ring is cut
    // from — and within that by the band's **own** walk sheet, which is the
    // sheet the ring is drawn on: the walk placement (`sheets::assign_all`)
    // separates a strip descending into a trench from the path on the rim
    // above it, and a ring merged across that separation read the upper
    // strip's seat beside the lower kerb.
    let mut ring_mask: HashMap<(i64, u32), HashMap<u32, Shapes>> = HashMap::new();

    // Group the buffered carriageways by level — the partition that keeps a
    // viaduct from unioning with the street beneath it.
    //
    // Buffering runs on whole polylines, not on single segments. Two adjacent
    // segments buffered separately have butt caps that *touch* along an edge
    // without overlapping, and a boolean union keeps touching shapes apart — a
    // straight road came out of this as one region per segment. Stroking the run
    // as a polyline also gets proper mitered joins at its bends instead of a pair
    // of square corners.
    let mut by_level: HashMap<(i64, u32, priors::Surface), Shapes> = HashMap::new();
    for run in runs(junctions, source_ids) {
        let line: Vec<[f64; 2]> = run.line.iter().map(|&c| frame.to_m(c)).collect();
        // A run nothing crowds keeps the constant-width stroke it has always
        // had — same joins, same vertices, so the change is confined to the
        // streets a facade actually narrows.
        let uniform = run.section.iter().all(|&[l, r]| l == run.half_m && r == run.half_m);
        let mut buffered = if uniform {
            poly::buffer_line(&line, run.half_m)
        } else {
            poly::buffer_section(&line, &run.section)
        };
        // Trim to the deck's face. This is the whole of the abutment fix: the
        // band was generated past the boundary (`synth::carriageway`), and what a
        // structure carries is removed from it by *that structure's own*
        // cross-section, so the two share an edge rather than each construct
        // one. Per run, before the union, because that is the last moment the
        // model still knows which band this cut belongs to — afterwards the
        // boolean has dissolved it into the region, and a cut applied there
        // would just as happily take a bite out of the road passing underneath.
        let n = line.len();
        let ends = [
            run.cut_start.map(|c| (c, [line[0][0] - line[1][0], line[0][1] - line[1][1]])),
            run.cut_end
                .map(|c| (c, [line[n - 1][0] - line[n - 2][0], line[n - 1][1] - line[n - 2][1]])),
        ];
        for (cut, outward) in ends.into_iter().flatten() {
            if buffered.is_empty() {
                break;
            }
            buffered = poly::difference(&buffered, &cut_beyond(&cut, &frame, outward));
        }
        if buffered.is_empty() {
            continue;
        }
        if ring_on && run.hosted && run.surface == priors::Surface::Walkway {
            // A hosted band is not drawn; it says where the ring is pavement.
            let widened = if uniform {
                poly::buffer_line(&line, run.half_m + RING_MASK_SLACK_M)
            } else {
                let wider: Vec<[f64; 2]> =
                    run.section.iter().map(|&[l, r]| [l + RING_MASK_SLACK_M, r + RING_MASK_SLACK_M]).collect();
                poly::buffer_section(&line, &wider)
            };
            ring_mask
                .entry((run.level, run.host))
                .or_default()
                .entry(run.layer)
                .or_default()
                .extend(widened);
            continue;
        }
        // Keyed on the **drawn** material rather than the modelled one
        // where [`drawn`] says so: a footway and the sidewalk it runs into
        // would then be one region, unioning instead of the junior being
        // subtracted under the senior and each wearing its own rim.
        // Opt-in — see [`drawn`] for what the measurement said about it.
        by_level.entry((run.level, run.layer, drawn(run.surface))).or_default().extend(buffered);
    }
    // Where two at-grade sheets stack past the grade-separation boundary, the
    // junior yields the contested plan space (docs/GENERATION.md I9).
    let yields = trench_yields(junctions, source_ids, &frame, &rect, field);

    // Sorted by (level, layer) and asphalt before ballast within a pair, so
    // the output order — and the subtraction below — is a function of the
    // model, never of hashing.
    let mut levels: Vec<(i64, u32, priors::Surface)> = by_level.keys().copied().collect();
    levels.sort_unstable_by_key(|&(level, layer, surface)| (level, layer, material_rank(surface)));

    // The closing masks: each intersection's paved extent, in chunk metres. Only
    // the intersections near this chunk matter, and only their *shape* — the
    // fillet material is whatever the closing adds inside one of these.
    let pad = (PAVE_PAD_M + priors::CURB_RETURN_M) / DEG_M;
    let extents = intersection_shapes(junctions, &rect, pad, &frame);
    let masks = if extents.is_empty() {
        Vec::new()
    } else {
        poly::dilate(&poly::union_all(&extents), priors::CURB_RETURN_M)
    };

    // The closed regions, computed once: the ring below reads the asphalt's
    // before the loop draws it, and the seniors' subtraction reads them again.
    let mut closed_of: HashMap<(i64, u32, priors::Surface), Shapes> = HashMap::new();
    let closing = |key: (i64, u32, priors::Surface),
                       by_level: &HashMap<(i64, u32, priors::Surface), Shapes>,
                       closed_of: &mut HashMap<(i64, u32, priors::Surface), Shapes>|
     -> Shapes {
        if let Some(c) = closed_of.get(&key) {
            return c.clone();
        }
        let open = by_level.get(&key).map(|s| poly::union_all(s)).unwrap_or_default();
        // The sidewalk ring inherits its curb returns from the asphalt it was
        // cut from, and a closing of its own is worse than none: at the foot
        // of a terrace it bridged the five metres between the ring and the
        // path on the lower level, and the fused region climbed the bank
        // between them (`slope.walk_crossfall` 415 % at 6.9093,46.4378).
        let closed = if open.is_empty() || (ring_on && key.2.is_pedestrian()) {
            open
        } else {
            poly::close_within(&open, priors::CURB_RETURN_M, &masks)
        };
        closed_of.insert(key, closed.clone());
        closed
    };

    // The sidewalk ring, per asphalt region ([`walk_ring`]): drawn as the
    // Walkway region of the same level and layer, alongside whatever free
    // bands that key already holds; its bench goes back to the model.
    let mut benches: Vec<SourceSeg> = Vec::new();
    if ring_on {
        let probe = std::env::var_os("ARPT_PAVE_PROBE").is_some();
        let walls = wall_shapes(walls, &rect, &frame);
        let cos_lat = chunk_centre(cx, cy).y.to_radians().cos();
        let asphalt_keys: Vec<(i64, u32, priors::Surface)> =
            levels.iter().copied().filter(|k| k.2 == priors::Surface::Asphalt).collect();
        if probe {
            let mut mk: Vec<_> = ring_mask
                .iter()
                .flat_map(|(k, g)| g.iter().map(move |(w, v)| ((k.0, k.1, *w), v.len())))
                .collect();
            mk.sort_unstable();
            eprintln!(
                "[ring] chunk {:?}: asphalt keys {:?}, band keys {:?}, walls {} shapes",
                key,
                asphalt_keys.iter().map(|k| (k.0, k.1)).collect::<Vec<_>>(),
                mk,
                walls.len()
            );
        }
        let mut scratch: Vec<u32> = Vec::new();
        // The intersections whose pins can reach a kerb in this chunk.
        let pins: Vec<&crate::synth::carriageway::Intersection> = junctions.near((
            rect.west - pad,
            rect.south - pad,
            rect.east + pad,
            rect.north + pad,
        ));
        for key in &asphalt_keys {
            let key = *key;
            let (level, layer, _) = key;
            let asphalt = closing(key, &by_level, &mut closed_of);
            if asphalt.is_empty() {
                continue;
            }
            let Some(groups) = ring_mask.get(&(level, layer)) else { continue };
            // The other sheets' asphalt on this level. A region's boundary
            // inside another region is not a kerb: a leg on one sheet joining
            // a roundabout on another ends in a butt inside the plate, and a
            // ring drawn round that end runs across the mouth on the plate's
            // asphalt, seated on the leg's profile.
            let others: Shapes = asphalt_keys
                .iter()
                .filter(|k| k.0 == level && k.1 != layer)
                .flat_map(|&k| closing(k, &by_level, &mut closed_of))
                .collect();
            let mut walk_layers: Vec<u32> = groups.keys().copied().collect();
            walk_layers.sort_unstable();
            let ballast = closing((level, layer, priors::Surface::Ballast), &by_level, &mut closed_of);
            for walk_layer in walk_layers {
            let bands = &groups[&walk_layer];
            let raw = ring_mask_of(bands, &extents);
            if raw.is_empty() {
                continue;
            }
            let t0 = std::time::Instant::now();
            let mut kerb = kerb_segments(&asphalt, &raw, &others, &frame);
            bridge_along_kerb(&mut kerb);
            let mask = kerb_mask(&kerb);
            let t_kerb = t0.elapsed();
            let ring = sidewalk_ring(&asphalt, &ballast, &walls, &mask);
            let t_ring = t0.elapsed() - t_kerb;
            if probe {
                eprintln!(
                    "[ring]   ({level}, {layer}) walk {walk_layer}: asphalt {:.0} m2, bands {:.0} m2, raw mask {:.0} m2, \
                     kerb {:.0} m of {:.0} masked, ring {:.0} m2",
                    poly::area(&asphalt),
                    poly::area(&poly::union_all(bands)),
                    poly::area(&raw),
                    kerb.iter().map(|k| k.len).sum::<f64>(),
                    kerb.iter().filter(|k| k.masked).map(|k| k.len).sum::<f64>(),
                    poly::area(&ring)
                );
            }
            if ring.is_empty() {
                continue;
            }
            // ARPT_RING_AT=lon,lat — every kerb station within 15 m of the
            // point, with what became of it: masked, in the ring, its seat,
            // and after the fit its width or its refusal. The instrument for
            // a gap the mask cannot explain and a bump the seat can.
            let probe_at: Option<Coord> = std::env::var("ARPT_RING_AT").ok().and_then(|v| {
                let (a, b) = v.split_once(',')?;
                Some(Coord { x: a.trim().parse().ok()?, y: b.trim().parse().ok()? })
            });
            let probe_m = probe_at.map(|c| frame.to_m(c));
            let near_probe = |p: Pt| -> bool {
                probe_m.is_some_and(|q| ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2)).sqrt() <= 15.0)
            };
            // The ring's bench: one band segment per masked kerb station the
            // ring actually covers, seated at the kerb. `at` carries the
            // station index through the fit, which drops what it refuses.
            let half = priors::WALK_WIDTH_M * 0.5;
            let mut mine: Vec<SourceSeg> = Vec::new();
            let mut at: Vec<u64> = Vec::new();
            let ring_r = Region::of(&ring);
            for (ki, k) in kerb.iter().enumerate() {
                let mid = [
                    (k.p[0] + k.q[0]) * 0.5 + k.n[0] * half,
                    (k.p[1] + k.q[1]) * 0.5 + k.n[1] * half,
                ];
                let show = near_probe(mid);
                let in_ring = ring_r.inside(mid);
                let ka = frame.to_deg([k.p[0] - k.n[0] * SEAT_INSET_M, k.p[1] - k.n[1] * SEAT_INSET_M]);
                let kb = frame.to_deg([k.q[0] - k.n[0] * SEAT_INSET_M, k.q[1] - k.n[1] * SEAT_INSET_M]);
                let seats = (k.masked && in_ring).then(|| {
                    (
                        kerb_seat(junctions, &pins, level, layer, ka, cos_lat, &mut scratch),
                        kerb_seat(junctions, &pins, level, layer, kb, cos_lat, &mut scratch),
                    )
                });
                if show {
                    let c = frame.to_deg(mid);
                    eprintln!(
                        "[ring-at] ({level},{layer}) walk {walk_layer} station {ki} at {:.6},{:.6} len {:.1} \
                         masked {} in_ring {} seats {:?}",
                        c.x,
                        c.y,
                        k.len,
                        k.masked,
                        in_ring,
                        seats.as_ref().map(|(x, y)| (x.map(|v| (v.0 * 100.0).round() / 100.0), y.map(|v| (v.0 * 100.0).round() / 100.0)))
                    );
                }
                if !k.masked || !in_ring {
                    continue;
                }
                let (Some((ha, corridor)), Some((hb, _))) = seats.expect("computed for a masked station in the ring") else {
                    continue;
                };
                // **The bench is as wide as the ring is here, measured and
                // not stepped.** The bench is as wide as the ring
                // is here, read by bracketing outward and bisecting — a
                // continuous number. Read in rungs of the width ladder it was
                // the ladder that showed: the ring's own edge is the facade
                // line, smooth, and two neighbouring stations either side of a
                // rung cut it back by the rung, which drew as a sawtooth down
                // an otherwise straight pavement (`street.walk_width_step`
                // 23.3 % of the roundabout's kerb, wandering up to 3.4 m).
                // The ladder exists to stop a *band* pulsing along a street,
                // where the width is a claim re-derived per station; the ring
                // has one edge already and only needs it measured.
                let kmid = [(k.p[0] + k.q[0]) * 0.5, (k.p[1] + k.q[1]) * 0.5];
                let covered =
                    |d: f64| ring_r.inside([kmid[0] + k.n[0] * d, kmid[1] + k.n[1] * d]);
                let reach = priors::WALK_WIDTH_M + RING_MASK_SLACK_M;
                let mut lo = 0.05;
                if !covered(lo) {
                    continue;
                }
                while lo + RING_WIDTH_STEP_M <= reach && covered(lo + RING_WIDTH_STEP_M) {
                    lo += RING_WIDTH_STEP_M;
                }
                let mut hi = (lo + RING_WIDTH_STEP_M).min(reach);
                for _ in 0..3 {
                    let mid = 0.5 * (lo + hi);
                    if covered(mid) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                let width = lo;
                if width < priors::WALK_MIN_WIDTH_M {
                    continue;
                }
                let half = width * 0.5;
                let a = frame.to_deg([k.p[0] + k.n[0] * half, k.p[1] + k.n[1] * half]);
                let b = frame.to_deg([k.q[0] + k.n[0] * half, k.q[1] + k.n[1] * half]);
                // **A station must be a station, not a wall** — the band's own
                // rule (`walkway::fitted_half`). Where the union has merged two
                // terraces into one region, the kerb walk follows its boundary
                // over the wall between them, and a station there is seated on
                // the lower road at one end and the upper street at the other:
                // 4.7 m over 4 m at 6.9151,46.4366. No pavement descends a
                // wall; the station is refused and the ring stops at it.
                let allow = crate::synth::walkway::CORNER_STEP_M
                    .max(k.len * crate::synth::walkway::WALK_WALL_GRADE);
                if (ha - hb).abs() > allow {
                    continue;
                }
                mine.push(SourceSeg {
                    a,
                    b,
                    cos_lat,
                    half_m: half,
                    sect_a: Section::uniform(half),
                    sect_b: Section::uniform(half),
                    level,
                    layer: walk_layer,
                    cut_a: None,
                    cut_b: None,
                    height_a: ha,
                    height_b: hb,
                    corridor,
                    surface: priors::Surface::Walkway,
                    rise_m: priors::KERB_RISE_M,
                    arc0: k.arc,
                });
                at.push(ki as u64);
            }
            // **The pavement does not yield to the ground; the ground yields
            // to it** (docs/GROUND.md §2). The band's fit narrows a strip and
            // then gives up on it where the earthwork beside it would be too
            // deep, and for a *path* across open ground that is right — the
            // path is the visible ground there. A street's pavement is not:
            // where the ground beside it cannot be battered, what is really
            // there is a retaining wall, and the drawn apron already draws
            // one. So the ring is drawn at the width it has and stratum D
            // holds it, stepping the ground at the pavement's own edge
            // (`ground::walk_edge`). The stations that still go are the ones
            // that are not pavement at all: a station spanning a wall along
            // the kerb, refused above, and the orphan runs those leave.
            drop_orphan_runs(&kerb, &mut mine, &mut at);
            let mut rings_by_sub: Vec<(u32, Shapes)> = Vec::new();
            match ring_ctx {
                Some(ctx) => {
                    let surviving = surviving_mask(&kerb, &at);
                    let kept =
                        if surviving.is_empty() { Vec::new() } else { sidewalk_ring(&asphalt, &ballast, &walls, &surviving) };
                    if kept.is_empty() {
                        continue;
                    }
                    let subs = crate::synth::sheets::assign(ctx.scene, &mine);
                    let mut sub_ids: Vec<u32> = subs.clone();
                    sub_ids.sort_unstable();
                    sub_ids.dedup();
                    for sub in sub_ids {
                        let part: Vec<u64> = (0..mine.len())
                            .filter(|&i| subs[i] == sub)
                            .map(|i| at[i])
                            .collect();
                        let mask = surviving_mask(&kerb, &part);
                        if mask.is_empty() {
                            continue;
                        }
                        let r = poly::intersect(&kept, &mask);
                        if !r.is_empty() {
                            rings_by_sub.push((sub, r));
                        }
                    }
                    for (i, b) in mine.iter_mut().enumerate() {
                        b.layer = walk_layer + subs[i];
                    }
                }
                None => rings_by_sub.push((0, ring)),
            };
            if probe && t0.elapsed().as_millis() > 500 {
                eprintln!(
                    "[ring]   ({level},{layer}) walk {walk_layer}: {} stations, {} benches, {} sheets: kerb {:?} ring {:?} seats+cut {:?}",
                    kerb.len(),
                    mine.len(),
                    rings_by_sub.len(),
                    t_kerb,
                    t_ring,
                    t0.elapsed() - t_kerb - t_ring
                );
            }
            if let Some(q) = probe_m {
                for (sub, r) in &rings_by_sub {
                    eprintln!(
                        "[ring-at] ({level},{layer}) walk {walk_layer}+{sub}: probe in final ring: {} (ring {:.0} m2)",
                        inside(r, q),
                        poly::area(r)
                    );
                }
            }
            // **A bench belongs to the chunk that owns its station.** The
            // union is built over the chunk's pad, so its region ends in butt
            // ends at the pad's edge, and the kerb walk wraps those ends with
            // stations that are not kerbs — across the road, inside the
            // neighbouring chunk, which draws the real kerb there. The ring
            // itself is clipped to the rect below; its benches were not, and
            // the strays put a bench across every road at every chunk border:
            // `seam.terrain_shade` 0 → 0.069 % on the zone, creased 17°, every
            // site on a chunk line.
            let m_rect = (
                frame.to_m(Coord { x: rect.west, y: rect.south })[0],
                frame.to_m(Coord { x: rect.west, y: rect.south })[1],
                frame.to_m(Coord { x: rect.east, y: rect.north })[0],
                frame.to_m(Coord { x: rect.east, y: rect.north })[1],
            );
            let owned = |ki: u64| -> bool {
                let k = &kerb[ki as usize];
                let (mx, my) = ((k.p[0] + k.q[0]) * 0.5, (k.p[1] + k.q[1]) * 0.5);
                mx >= m_rect.0 && mx < m_rect.2 && my >= m_rect.1 && my < m_rect.3
            };
            benches.extend(mine.iter().zip(&at).filter(|(_, &ki)| owned(ki)).map(|(b, _)| *b));
            for (sub, r) in rings_by_sub {
                let walk_key = (level, walk_layer + sub, priors::Surface::Walkway);
                if !levels.contains(&walk_key) {
                    levels.push(walk_key);
                }
                by_level.entry(walk_key).or_default().extend(r);
            }
            }
        }
        levels.sort_unstable_by_key(|&(level, layer, surface)| (level, layer, material_rank(surface)));
    }

    let mut out = Vec::new();
    for (level, layer, surface) in levels {
        let mut closed = closing((level, layer, surface), &by_level, &mut closed_of);
        if closed.is_empty() {
            continue;
        }
        // Where two materials share a level and a layer their regions coincide
        // in plan at the same height and two coplanar surfaces z-fight. The
        // more physical one wins the fill and the junior region is trimmed
        // under it: asphalt over ballast, because a level crossing's
        // carriageway is what is physically laid across the formation. A
        // sidewalk overlapping a carriageway is the same sentence spoken
        // across the two sheet namespaces, where this key cannot reach — the
        // kerb-coincident yield in [`trench_yields`] speaks it instead.
        for &senior in seniors(surface) {
            if !by_level.contains_key(&(level, layer, senior)) {
                continue;
            }
            let senior_closed = closing((level, layer, senior), &by_level, &mut closed_of);
            closed = poly::difference(&closed, &senior_closed);
            if closed.is_empty() {
                break;
            }
        }
        let probe_m: Option<Pt> = std::env::var("ARPT_RING_AT").ok().and_then(|v| {
            let (a, b) = v.split_once(',')?;
            Some(frame.to_m(Coord { x: a.trim().parse().ok()?, y: b.trim().parse().ok()? }))
        });
        if let Some(q) = probe_m.filter(|_| surface.is_pedestrian()) {
            eprintln!("[ring-at] key ({level},{layer},{surface:?}) after seniors: probe inside {}", inside(&closed, q));
        }
        // The trench yield: the plan space another sheet's open band claims
        // more than a storey away vertically is not this surface's to pave
        // (`trench_yields`).
        if let Some(cuts) = yields.get(&(level, layer, surface)) {
            closed = poly::difference(&closed, &poly::union_all(cuts));
        }
        if let Some(q) = probe_m.filter(|_| surface.is_pedestrian()) {
            eprintln!("[ring-at] key ({level},{layer},{surface:?}) after yields: probe inside {}", inside(&closed, q));
        }
        if closed.is_empty() {
            continue;
        }
        // Clip in metres — i_overlay's exact rect intersection — then convert and
        // snap the cut coordinates onto the chunk rect's own lon/lat so the seam
        // is bit-identical from the neighbouring chunk.
        let m_rect = (
            frame.to_m(Coord { x: rect.west, y: rect.south })[0],
            frame.to_m(Coord { x: rect.west, y: rect.south })[1],
            frame.to_m(Coord { x: rect.east, y: rect.north })[0],
            frame.to_m(Coord { x: rect.east, y: rect.north })[1],
        );
        let clipped = poly::intersect_rect(&closed, m_rect);
        if clipped.is_empty() {
            continue;
        }
        let shapes: Vec<Vec<Vec<Coord>>> = clipped
            .iter()
            .map(|sh| {
                sh.iter()
                    .map(|ring| {
                        ring.iter().map(|&p| snap_to_rect(frame.to_deg(p), p, m_rect, &rect)).collect()
                    })
                    .collect()
            })
            .collect();
        out.push(LevelShapes { level, layer, surface, shapes });
    }
    (out, benches)
}

/// The material a run is drawn as.
///
/// **Opt-in, and the measurement is why.** Merging `Path` into `Walkway` at
/// the region key is right in principle ([`priors::drawn_material`]) and
/// nearly free in geometry — it moves total drawn pedestrian area by 0.1 %
/// and `contact.walk_rim` by 0.004 pp. What it is not is *landable*: the one
/// check that tells a sidewalk from a path does so by class, and merged it
/// starts scoring hillside tracks against roads they merely pass near
/// (`contact.sidewalk_grade` 0.39 % → 8.42 %, worst 7.75 → 17.96 m). That is
/// the instrument going blind, not the surface getting worse, and this
/// codebase does not blind an instrument to land a change. The tonal half of
/// the win is already taken for free in `style.json`, where `path_*` uses
/// `walk_*`'s colours; what stays unbought is the double rim where a
/// path meets a pavement. `ARPT_WALK_MERGE=1` turns it on for the A/B.
fn drawn(surface: priors::Surface) -> priors::Surface {
    if std::env::var_os("ARPT_WALK_MERGE").is_some() {
        return priors::drawn_material(surface);
    }
    surface
}

/// Where two at-grade sheets stack past the grade-separation boundary, the
/// plan space the junior must yield — keyed by the junior's own region key.
///
/// An open trench and an at-grade surface cannot coexist vertically
/// (docs/GENERATION.md I9): air between two surfaces is legal only under a
/// structure, and where the lower band is in a bore no open band is drawn at
/// all. What remains is the drawn impossibility `order.grade_stack` names —
/// at Territet the unioned terrace streets, each at its prior width, canopy
/// the whole open rail cutting between them, and the ballast band slides
/// beneath 11 m of open air. Nobody owns the space between, so nothing closes
/// it: the road's apron only stands on its region boundary, and over the
/// trench the road has none.
///
/// So the junior yields the overlap, and the yield is what closes the world:
/// the junior's region gains a boundary along the senior's band edge, and the
/// existing rim + apron machinery does the rest — a wall from the yielded rim
/// down to the ground, which inside the senior's bench *is* the trench floor.
/// Who is junior is the stratum ladder (docs/GENERATION.md §4.2), read off
/// the drawn material: rail holds its formation against a street
/// (`stratum_rank`), a street holds its carriageway against a footway — the
/// pedestrian underpass drawn as an open stack loses its band under the road,
/// two mouths instead of a see-through slot (I6: detail, not spectacle).
/// Between peers the upper yields: trimming the overhang leaves a rim the
/// apron can close downward, while trimming the tucked-under band would leave
/// the upper's interior floating over air with no boundary at all.
///
/// Scoped by the sheet partition: joined runs share a layer
/// (`sheets::merge_joined`), so a level crossing — same plan, same height —
/// is never a candidate, and only genuinely stacked sheets are compared.
/// The boundary is [`crossings::SEPARATION_M`], the same one the model draws
/// from the other side and `order.grade_stack` measures against.
///
/// Below that boundary one more family yields here: a pedestrian band within
/// [`priors::WALK_ON_ASPHALT_M`] of a carriageway is *on* the plate, not
/// stacked over it, and it yields the road's drawn cross-section — the
/// cross-namespace half of the [`seniors`] sentence, spoken where heights
/// exist to keep the footbridge and the trench-rim path out of it.
fn trench_yields(
    junctions: &CarriagewayModel,
    source_ids: &[u32],
    frame: &MFrame,
    rect: &Bounds,
    field: &mut Option<(
        &crate::synth::height::HeightField,
        &mut crate::ground::sampler::GroundSampler,
        u8,
    )>,
) -> HashMap<(i64, u32, priors::Surface), Shapes> {
    use crate::assemble::grid::GridIndex;
    use crate::solve::crossings::SEPARATION_M;
    use crate::synth::sheets::{bbox_of, closest_approach};

    let mut grid = GridIndex::new();
    for &i in source_ids {
        let s = junctions.source(i);
        if s.level == 0 {
            grid.insert(bbox_of(s), i);
        }
    }
    let mut out: HashMap<(i64, u32, priors::Surface), Shapes> = HashMap::new();
    let mut cand: Vec<u32> = Vec::new();
    for &i in source_ids {
        let s = junctions.source(i);
        if s.level != 0 {
            continue;
        }
        grid.query(bbox_of(s), &mut cand);
        for &j in cand.iter() {
            if j <= i {
                continue; // each pair once
            }
            let t = junctions.source(j);
            // One sheet: joined, braided or the same ribbon. Layers are a
            // namespace per side of `height::Sheet::walk` — the carriageway
            // layering and the walk layering never compare numbers — so a
            // pedestrian band and a carriageway are never one sheet, and their
            // equal layer is a coincidence the gap test below must arbitrate.
            if s.surface.is_pedestrian() == t.surface.is_pedestrian() && t.layer == s.layer {
                continue;
            }
            let (d, ts, tt) = closest_approach(s, t);
            if d > s.half_m + t.half_m {
                continue; // the bands never meet in plan
            }
            // The gap that decides every branch below. By the band's own
            // frozen chord normally; from the blended sheet field under
            // `ARPT_FIELD_YIELDS` (S3) — the same field the mesher drapes, so
            // near a junction plate the yield reads the height the band is
            // actually drawn at, pins and blending included, not the chord
            // the plate displaced.
            let gap = match field {
                Some((hf, sm, z_ref)) => {
                    let chord = s.height_at(ts) - t.height_at(tt);
                    let mut scratch: Vec<u32> = Vec::new();
                    let ps = Coord {
                        x: s.a.x + (s.b.x - s.a.x) * ts,
                        y: s.a.y + (s.b.y - s.a.y) * ts,
                    };
                    let pt = Coord {
                        x: t.a.x + (t.b.x - t.a.x) * tt,
                        y: t.a.y + (t.b.y - t.a.y) * tt,
                    };
                    let sheet_s = crate::synth::height::Sheet::of(s.level, s.layer, s.surface);
                    let sheet_t = crate::synth::height::Sheet::of(t.level, t.layer, t.surface);
                    let hs = hf.at(sm, sheet_s, *z_ref, *z_ref, rect, ps.x, ps.y, &mut scratch);
                    let ht = hf.at(sm, sheet_t, *z_ref, *z_ref, rect, pt.x, pt.y, &mut scratch);
                    let g = hs - ht;
                    if std::env::var_os("ARPT_FIELD_YIELDS_CENSUS").is_some() {
                        use crate::solve::crossings::SEPARATION_M;
                        let flip_sep = (g.abs() <= SEPARATION_M) != (chord.abs() <= SEPARATION_M);
                        let flip_kerb = (g.abs() <= priors::WALK_ON_ASPHALT_M)
                            != (chord.abs() <= priors::WALK_ON_ASPHALT_M);
                        if flip_sep || flip_kerb {
                            eprintln!(
                                "[fy] flip sep={flip_sep} kerb={flip_kerb} chord={chord:.2} field={g:.2} at {:.6},{:.6}",
                                ps.x, ps.y
                            );
                        }
                    }
                    g
                }
                None => s.height_at(ts) - t.height_at(tt),
            };
            if gap.abs() <= SEPARATION_M {
                // **The kerb-coincident case.** Below the grade-separation
                // boundary the sheets machinery layers same-material overlaps,
                // but a pedestrian band within [`priors::WALK_ON_ASPHALT_M`]
                // of a carriageway is not a layering question: it is a band
                // lying *on* the plate, and the plan space is the road's
                // (docs/GENERATION.md I3 — the same sentence the region-level
                // seniority spoke, until the walk-sheet namespaces stopped its
                // (level, layer) key from ever matching a road's; see
                // [`seniors`]). The cut is the senior's *drawn* cross-section,
                // so the band's new edge is the kerb the asphalt actually
                // reaches — a facade-narrowed street cuts with its narrowed
                // width, not its class prior. Free bands yield here too: the
                // from-below-only exemption is about a trench rim a storey up,
                // and within the kerb band there is no rim to protect.
                if s.surface.is_pedestrian() != t.surface.is_pedestrian()
                    && gap.abs() <= priors::WALK_ON_ASPHALT_M
                {
                    let (junior, senior) =
                        if s.surface.is_pedestrian() { (s, t) } else { (t, s) };
                    let sect = |x: crate::assemble::facades::Section| [x.left_m, x.right_m];
                    let line = [frame.to_m(senior.a), frame.to_m(senior.b)];
                    let quad = poly::buffer_section(
                        &line,
                        &[sect(senior.sect_a), sect(senior.sect_b)],
                    );
                    out.entry((junior.level, junior.layer, drawn(junior.surface)))
                        .or_default()
                        .extend(quad);
                }
                continue;
            }
            let (junior, senior) = match stratum_rank(s.surface).cmp(&stratum_rank(t.surface)) {
                std::cmp::Ordering::Less => (s, t),
                std::cmp::Ordering::Greater => (t, s),
                // Peers: the upper yields its overhang, so the retreat leaves
                // a rim over the lower band that the apron closes downward.
                std::cmp::Ordering::Equal => {
                    if gap > 0.0 {
                        (s, t)
                    } else {
                        (t, s)
                    }
                }
            };
            // A transverse crossing severs, a lateral overlap narrows — and a
            // drivable band is never severed: an unannotated flyover crossing
            // a trench is a missing *structure* (§4.5's to derive), and cutting
            // the carriageway across its direction of travel is spectacle, not
            // degradation. A pedestrian band is the one thing severing
            // degrades correctly: the stretch under a senior's roadbed
            // disappears and two mouths remain (I6 — a pedestrian underpass
            // drawn as an open stack loses its band, not its feature).
            if d == 0.0 && stratum_rank(junior.surface) >= 1 {
                continue;
            }
            // A *free* pedestrian band yields only from below. Above a trench
            // it already ends at the rim the fit gave it, and cutting it back
            // re-shapes its region so the surviving sliver drapes the trench's
            // batter. A *hosted* strip is the opposite case: its geometry is a
            // side of its street's cross-section, so where the street yields
            // its trench overhang the strip must yield with it — left behind,
            // it is the last thing hanging over the hole its host abandoned
            // (a sidewalk falling 8 m across its own width at 6.8932,46.4435).
            let junior_below = (junior.height_at(if std::ptr::eq(junior, s) { ts } else { tt }))
                < (senior.height_at(if std::ptr::eq(senior, s) { ts } else { tt }));
            if stratum_rank(junior.surface) == 0
                && !junior_below
                && junior.corridor == crate::synth::walkway::NO_HOST
            {
                continue;
            }
            let line = [frame.to_m(senior.a), frame.to_m(senior.b)];
            let quad = poly::buffer_line(&line, senior.half_m);
            out.entry((junior.level, junior.layer, drawn(junior.surface)))
                .or_default()
                .extend(quad);
        }
    }
    // **The fillet half of the kerb-coincident yield.** The closing adds
    // plate area inside an intersection that no source quad covers
    // ([`intersection_masks`]), so a band the segment cuts severed at the
    // kerbs could keep a sliver over the fillet — every sample of it an edge
    // sample, which is how it lands in `slope.walk_crossfall` as well as on
    // the plate. The intersection's own extent is the shape of that space,
    // and its pinned height says which bands are on it rather than over it —
    // a walkway crossing above a sunken junction keeps its plan space exactly
    // as it does against the segments.
    let mut cut: std::collections::HashSet<((i64, u32, priors::Surface), (u64, u64))> =
        std::collections::HashSet::new();
    let ring_on = walk_ring();
    for &i in source_ids {
        let s = junctions.source(i);
        if s.level != 0 || !s.surface.is_pedestrian() {
            continue;
        }
        // The sidewalk ring is cut from the closed asphalt — plate, fillets
        // and all — so nothing of it lies over an intersection, and the
        // extent it would yield here is exactly the corner it exists to wrap
        // (an 18 m gap at 6.9086,46.4379 was this yield). The paths keep
        // yielding: a free band is still drawn over whatever it crosses.
        if ring_on && drawn(s.surface) == priors::Surface::Walkway {
            continue;
        }
        let band_h = 0.5 * (s.height_a + s.height_b);
        for j in junctions.near(bbox_of(s)) {
            let Some(h) = j.height() else { continue };
            if (band_h - h).abs() > priors::WALK_ON_ASPHALT_M {
                continue;
            }
            let key = (s.level, s.layer, drawn(s.surface));
            let p = j.point();
            if !cut.insert((key, (p.x.to_bits(), p.y.to_bits()))) {
                continue;
            }
            let area = j.area();
            let centre = frame.to_m(area.centre());
            let ring: Vec<[f64; 2]> =
                area.ring().map(|(e, n)| [centre[0] + e, centre[1] + n]).collect();
            if ring.len() >= 3 {
                out.entry(key).or_default().push(vec![ring]);
            }
        }
    }
    out
}

/// The authority a drawn material carries when two sheets stack: the stratum
/// ladder of docs/GENERATION.md §4.2, read off the surface. Independent rail
/// is decisively the reason its cuttings exist; every drivable road negotiates
/// below it; a pedestrian band drapes on the finished world and yields to
/// both.
fn stratum_rank(surface: priors::Surface) -> u8 {
    match surface {
        priors::Surface::Ballast => 2,
        priors::Surface::Asphalt => 1,
        priors::Surface::Walkway | priors::Surface::Path | priors::Surface::None => 0,
    }
}

/// Which materials outrank this one where the two coincide in plan on one
/// sheet — what is subtracted from it before it is emitted.
///
/// The order is physical, not arbitrary: the carriageway is laid across the
/// formation at a level crossing, and both are laid before the footway beside
/// them. It is also the emission order ([`material_rank`]), so a region is
/// always trimmed against one already resolved.
///
/// **Same namespace only.** The key this subtraction matches on is
/// `(level, layer)`, and a layer number only means anything against another
/// number from the same `sheets::assign` run — the road layering and the walk
/// layering are separate namespaces that never compare (`synth::carriageway`).
/// A pedestrian band listing Asphalt here was the pre-namespace arrangement
/// surviving its own premise: after the split the key matched only by
/// *accident* — trimming an elevated walk sheet under a road that happened to
/// share its number, and leaving a band lying on a junction plate untrimmed
/// because the plate's sheet was 3 and the band's was 0, which diced the
/// plate into blobs at every junction near grade-separated fabric. The
/// cross-namespace sentence is now spoken where heights exist to say it
/// honestly: the kerb-coincident yield in [`trench_yields`].
fn seniors(surface: priors::Surface) -> &'static [priors::Surface] {
    // Under the one-ordinal namespace (the default; `ARPT_TWO_SHEETS`
    // reverts) the cross-namespace
    // sentence comes back to the region level, where it belongs: a coplanar
    // walk shares its street's (level, layer) **by construction** now, so the
    // key matches by meaning rather than accident — and a walk genuinely
    // above a road is floored off the road's rung (`sheets::assign_all`), so
    // the trench-rim accident that removed this cannot recur. The region
    // subtraction is what the per-segment kerb quads could never be: the
    // union's own shape — a footway lying 8 m inside a paved field at
    // 6.9130,46.4397 survived every segment cross-section cut and cannot
    // survive this one.
    if std::env::var_os("ARPT_TWO_SHEETS").is_none() {
        return match surface {
            priors::Surface::Asphalt => &[],
            priors::Surface::Ballast => &[priors::Surface::Asphalt],
            priors::Surface::Walkway => &[priors::Surface::Asphalt, priors::Surface::Ballast],
            priors::Surface::Path => &[
                priors::Surface::Walkway,
                priors::Surface::Asphalt,
                priors::Surface::Ballast,
            ],
            priors::Surface::None => &[],
        };
    }
    match surface {
        priors::Surface::Asphalt => &[],
        priors::Surface::Ballast => &[priors::Surface::Asphalt],
        priors::Surface::Walkway => &[],
        priors::Surface::Path => &[priors::Surface::Walkway],
        priors::Surface::None => &[],
    }
}

/// A material's place in that order, for sorting the regions of one sheet.
fn material_rank(surface: priors::Surface) -> u8 {
    match surface {
        priors::Surface::Asphalt => 0,
        priors::Surface::Ballast => 1,
        priors::Surface::Walkway => 2,
        priors::Surface::Path => 3,
        priors::Surface::None => 4,
    }
}

/// One carriageway run: a polyline of one class, level and surface, to be
/// stroked in a single pass.
struct Run {
    line: Vec<Coord>,
    /// The class's own half-width — constant along the run, and what makes two
    /// segments part of the same run.
    half_m: f64,
    /// What is drawn at each vertex: `[left, right]` in metres, `half_m` on
    /// both sides except where a facade has taken some of it back
    /// (`synth::carriageway::Section`). Same length as `line`.
    section: Vec<[f64; 2]>,
    level: i64,
    layer: u32,
    surface: priors::Surface,
    /// The structure cross-sections bounding this run's two ends, where it has
    /// one. The run is buffered *through* them and then cut back to them, so
    /// its edge at an abutment is the deck's own end face.
    cut_start: Option<Handover>,
    cut_end: Option<Handover>,
    /// Whether the run's segments ride a street (`corridor != NO_HOST`) — for
    /// a pedestrian run, whether it is a sidewalk seated on a kerb or a path
    /// across open ground. The sidewalk ring masks with the first and draws
    /// the second, so the two never chain.
    hosted: bool,
    /// The corridor the run's segments belong to, and their latitude scale —
    /// what a hosted walk run needs to find the asphalt it is the pavement of.
    corridor: crate::scene::CorridorId,
    cos_lat: f64,
    /// For a hosted walk run, the sheet of the asphalt it borders
    /// ([`host_layer`]), read per segment and chained only while it holds: a
    /// street's asphalt changes sheet along its length, and one key for a
    /// run hundreds of metres long masked the wrong asphalt over the rest.
    host: u32,
}

/// The grade-separation layer of the asphalt a hosted walk run stands beside:
/// the nearest carriageway stretch of the run's own host corridor.
///
/// Not the run's own layer. A walk sheet is placed in the unified namespace
/// (`sheets::assign_all`) and may be lifted off its street's rung to keep
/// walk-over-walk order — at the Avenue des Alpes roundabout 41 of 131 hosted
/// bands sat on sheet 5 beside asphalt on sheet 4 — and a mask keyed by the
/// band's own ordinal then finds no asphalt to be the pavement of. The ring
/// belongs to the asphalt it borders, so it is keyed by that asphalt's sheet.
fn host_layer(junctions: &CarriagewayModel, seg: &SourceSeg) -> u32 {
    if seg.corridor == NO_HOST || !seg.surface.is_pedestrian() {
        return seg.layer;
    }
    let p = Coord { x: (seg.a.x + seg.b.x) * 0.5, y: (seg.a.y + seg.b.y) * 0.5 };
    let reach_lat = (seg.half_m + priors::WALK_WIDTH_M + 2.0) / DEG_M;
    let reach_lon = reach_lat / seg.cos_lat.max(1e-6);
    let mut ids = Vec::new();
    junctions.sources_near((p.x - reach_lon, p.y - reach_lat, p.x + reach_lon, p.y + reach_lat), &mut ids);
    let mut best: Option<(f64, u32)> = None;
    for i in ids {
        let s = junctions.source(i);
        if s.corridor != seg.corridor || s.surface.is_pedestrian() {
            continue;
        }
        let (d, _) = crate::synth::sheets::point_to_segment(p, s.a, s.b, s.cos_lat);
        if best.is_none_or(|(bd, _)| d < bd) {
            best = Some((d, s.layer));
        }
    }
    best.map_or(seg.layer, |(_, l)| l)
}

/// Everything on the far side of a structure's cross-section, as a shape in
/// chunk metres — what the band loses to the deck.
///
/// A quad rather than a true half-plane, because the boolean wants a bounded
/// shape. `outward` is the run's own direction at that end, which is what says
/// which side is the deck's: the run was generated *past* the cut, so it points
/// from the band into the structure.
fn cut_beyond(cut: &Handover, frame: &MFrame, outward: [f64; 2]) -> Shapes {
    let (a, b) = (frame.to_m(cut.a), frame.to_m(cut.b));
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len = (dx * dx + dy * dy).sqrt();
    if !(len > 0.0) {
        return Vec::new();
    }
    let (mut nx, mut ny) = (-dy / len, dx / len);
    if nx * outward[0] + ny * outward[1] < 0.0 {
        (nx, ny) = (-nx, -ny);
    }
    let r = CUT_REACH_M;
    let mut ring = vec![
        [a[0], a[1]],
        [b[0], b[1]],
        [b[0] + nx * r, b[1] + ny * r],
        [a[0] + nx * r, a[1] + ny * r],
    ];
    // Counter-clockwise, the convention the rest of this module keeps: flipping
    // the normal above reverses the quad's winding, and a clockwise subtrahend
    // under the non-zero rule is a hole rather than a shape.
    let area: f64 = (0..ring.len())
        .map(|i| {
            let (p, q) = (ring[i], ring[(i + 1) % ring.len()]);
            p[0] * q[1] - q[0] * p[1]
        })
        .sum();
    if area < 0.0 {
        ring.reverse();
    }
    vec![vec![ring]]
}

/// How far past a structure's cross-section the trim quad reaches, in metres.
/// Several times the overrun the band was generated with, so the quad's far
/// edge can never fall inside the material it is there to remove.
const CUT_REACH_M: f64 = 8.0;

/// Chains a chunk's carriageway segments back into polylines.
///
/// The model stores segments because the height field measures distance to each
/// one, but the union wants the runs they came from.
///
/// Chaining is on *geometry*, not on id adjacency. Source ids come from a grid
/// query, so a corridor whose far end falls outside this chunk's padded box
/// arrives with gaps in its id range — and requiring contiguous ids then split it
/// into two polylines meeting end to end. Buffered separately, their butt caps
/// only *touch* where they meet, and a boolean union keeps touching shapes apart:
/// a straight road acquired a hairline seam at every such split. Matching on the
/// shared endpoint instead joins them whatever the ids do.
///
/// `source_ids` is sorted, and the segments behind it were pushed in
/// corridor-then-node order, so the walk order — and therefore the output — is a
/// function of the model rather than of the traversal.
fn runs(junctions: &CarriagewayModel, source_ids: &[u32]) -> Vec<Run> {
    let mut out: Vec<Run> = Vec::new();
    let ring_on = walk_ring();
    let mut host_of: HashMap<u32, u32> = HashMap::new();
    for &i in source_ids {
        let s = junctions.source(i);
        let host = if ring_on {
            *host_of.entry(i).or_insert_with(|| host_layer(junctions, s))
        } else {
            s.layer
        };
        let continues = out.last().is_some_and(|r| {
            let last = *r.line.last().expect("a run has points");
            r.level == s.level
                && r.layer == s.layer
                && r.surface == s.surface
                && r.half_m == s.half_m
                && r.hosted == (s.corridor != NO_HOST)
                && r.host == host
                && last.x == s.a.x
                && last.y == s.a.y
        });
        let sect = |x: crate::assemble::facades::Section| [x.left_m, x.right_m];
        if continues {
            let r = out.last_mut().expect("a run exists");
            r.line.push(s.b);
            // The shared vertex already carries its cross-section from the
            // previous segment; the two agree, being read at one station.
            r.section.push(sect(s.sect_b));
            r.cut_end = s.cut_b;
        } else {
            out.push(Run {
                line: vec![s.a, s.b],
                half_m: s.half_m,
                section: vec![sect(s.sect_a), sect(s.sect_b)],
                level: s.level,
                layer: s.layer,
                surface: s.surface,
                cut_start: s.cut_a,
                cut_end: s.cut_b,
                hosted: s.corridor != NO_HOST,
                corridor: s.corridor,
                cos_lat: s.cos_lat,
                host,
            });
        }
    }
    out
}

/// The sidewalk ring of one asphalt region ([`walk_ring`]): the region grown
/// by a pavement's width, less the asphalt itself, less the rail formation
/// sharing its level and layer, less the buildings (already grown by their
/// clearance), kept only where `mask` says the kerb carries pavement.
///
/// Then **opened on its outer side** at half the narrowest band worth
/// drawing: a pinch between the kerb and a facade narrower than
/// [`priors::WALK_MIN_WIDTH_M`] draws no pavement, which is the same sentence
/// the band generator speaks about a street too narrow for a sidewalk. A plain
/// opening (erode, then dilate by the same radius) would also round every
/// convex corner of the *kerb* side, pulling the pavement off the kerb by the
/// radius at each one; re-dilating a little wider and intersecting with the
/// raw ring restores the kerb side exactly and opens only the outer side.
fn sidewalk_ring(asphalt: &Shapes, ballast: &Shapes, walls: &Shapes, mask: &Shapes) -> Shapes {
    let grown = poly::dilate(asphalt, priors::WALK_WIDTH_M);
    let mut ring = poly::difference(&grown, asphalt);
    if ring.is_empty() {
        return ring;
    }
    ring = poly::difference(&ring, ballast);
    ring = poly::difference(&ring, walls);
    ring = poly::intersect(&ring, mask);
    if ring.is_empty() {
        return ring;
    }
    let r = priors::WALK_MIN_WIDTH_M * 0.5;
    let opened = poly::dilate(&poly::erode(&ring, r), 2.0 * r);
    if opened.is_empty() {
        return Vec::new();
    }
    poly::intersect(&ring, &opened)
}

/// How far **inside** the kerb the seat is read, metres. The field's kernels
/// all vanish at a carriageway's edge, so a point outside every half-width
/// reads whichever source's weight vanishes slowest — a different one from
/// one station to the next where a leg meets a ring arc whose solved profiles
/// disagree (0.5 m in 0.6 m at the roundabout's south-west mouth). A hand's
/// breadth inside, the kerb's own road covers the point and a corner is
/// covered by both its legs, which is what the asphalt's own rim vertices
/// read there: the ring's seat is the kerb the asphalt draws, plus the rise.
const SEAT_INSET_M: f64 = 0.3;

/// Cosine of the turn at a contour vertex past which a station may not
/// straddle it: 20°. A fillet arc turns a few degrees per edge and is walked
/// as chords; a junction mouth's corner or a building's corner in the kerb is
/// a break.
const RING_TURN_COS: f64 = 0.94;

/// Spacing of the kerb stations the ring is walked at, metres — short enough
/// that a bench seat interpolated between two stations stays on the road's
/// profile across a grade change, long enough that the bench population stays
/// in the hundreds of thousands.
const RING_STEP_M: f64 = 4.0;

/// One station-length of kerb: the asphalt boundary from `p` to `q` (chunk
/// metres), the outward unit normal, its arc along its contour, and whether
/// the pavement mask covers the ring beside it.
struct KerbSeg {
    p: Pt,
    q: Pt,
    n: [f64; 2],
    len: f64,
    arc: f64,
    contour: usize,
    masked: bool,
}

/// A region with its shapes' bounding boxes, for point tests over many
/// points: a chunk's mask holds hundreds of shapes and a kerb has tens of
/// thousands of stations, and the parity walk over every vertex of every
/// shape was most of the ring's cost.
struct Region<'a> {
    shapes: &'a Shapes,
    boxes: Vec<(f64, f64, f64, f64)>,
}

impl<'a> Region<'a> {
    fn of(shapes: &'a Shapes) -> Region<'a> {
        let boxes = shapes
            .iter()
            .map(|shape| {
                shape.iter().flatten().fold((f64::MAX, f64::MAX, f64::MIN, f64::MIN), |b, q| {
                    (b.0.min(q[0]), b.1.min(q[1]), b.2.max(q[0]), b.3.max(q[1]))
                })
            })
            .collect();
        Region { shapes, boxes }
    }

    fn inside(&self, p: Pt) -> bool {
        let mut odd = false;
        for (shape, b) in self.shapes.iter().zip(&self.boxes) {
            if p[0] < b.0 || p[0] > b.2 || p[1] < b.1 || p[1] > b.3 {
                continue;
            }
            for ring in shape {
                let n = ring.len();
                for i in 0..n {
                    let (a, c) = (ring[i], ring[(i + 1) % n]);
                    if (a[1] > p[1]) != (c[1] > p[1])
                        && p[0] < a[0] + (c[0] - a[0]) * (p[1] - a[1]) / (c[1] - a[1])
                    {
                        odd = !odd;
                    }
                }
            }
        }
        odd
    }
}

/// Even-odd point-in-region over every contour of `shapes`; holes fall out of
/// the parity.
fn inside(shapes: &Shapes, p: Pt) -> bool {
    let mut odd = false;
    for shape in shapes {
        for ring in shape {
            let n = ring.len();
            for i in 0..n {
                let (a, b) = (ring[i], ring[(i + 1) % n]);
                if (a[1] > p[1]) != (b[1] > p[1])
                    && p[0] < a[0] + (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1])
                {
                    odd = !odd;
                }
            }
        }
    }
    odd
}

/// Every contour of `asphalt` cut into stations of at most [`RING_STEP_M`],
/// each marked by whether `raw` covers the ring beside it and no other sheet's
/// asphalt (`others`) stands there. Outward is the
/// right-hand normal of the direction of travel, which is outward for a
/// counter-clockwise outer boundary and a clockwise hole alike.
fn kerb_segments(asphalt: &Shapes, raw: &Shapes, others: &Shapes, frame: &MFrame) -> Vec<KerbSeg> {
    let mut out = Vec::new();
    let half = priors::WALK_WIDTH_M * 0.5;
    let (raw, others) = (Region::of(raw), Region::of(others));
    // The station lattice, in degrees: stations are cut where the contour
    // crosses a line of it, so two chunks walking the same kerb either side
    // of their border cut the same stations. Cut by arc from each chunk's own
    // contour start, they did not, and the ring's edge — its fit, its mask's
    // bridging — differed across every chunk line: `seam.terrain_shade`
    // 0 → 0.069 % on the zone, creased 17°, every site on a chunk border.
    let d_lat = RING_STEP_M / DEG_M;
    let d_lon = RING_STEP_M / (DEG_M * frame.to_deg([0.0, 0.0]).y.to_radians().cos().max(1e-6));
    let mut contour = 0usize;
    for shape in asphalt {
        for ring in shape {
            let n = ring.len();
            if n < 3 {
                continue;
            }
            // Cumulative arc round the contour, and the vertices where it
            // turns sharply enough that a station may not straddle them: a
            // station is a chord, and a chord across a corner offsets its
            // bench off the kerb. Stations are otherwise cut by arc length,
            // so a fillet drawn as a hundred centimetre edges is a few
            // stations rather than a hundred.
            let mut arc = vec![0.0f64; n + 1];
            for i in 0..n {
                let (a, b) = (ring[i], ring[(i + 1) % n]);
                arc[i + 1] = arc[i] + ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
            }
            let total = arc[n];
            if total < 1e-6 {
                continue;
            }
            let point_at = |s: f64| -> Pt {
                let s = s.clamp(0.0, total);
                let i = arc.partition_point(|&a| a <= s).saturating_sub(1).min(n - 1);
                let (a, b) = (ring[i], ring[(i + 1) % n]);
                let seg = arc[i + 1] - arc[i];
                let t = if seg > 1e-9 { (s - arc[i]) / seg } else { 0.0 };
                [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
            };
            let mut breaks: Vec<f64> = vec![0.0];
            for i in 1..n {
                let (p, c, q) = (ring[i - 1], ring[i], ring[(i + 1) % n]);
                let (ux, uy) = (c[0] - p[0], c[1] - p[1]);
                let (vx, vy) = (q[0] - c[0], q[1] - c[1]);
                let (lu, lv) = (ux.hypot(uy), vx.hypot(vy));
                if lu < 1e-9 || lv < 1e-9 {
                    continue;
                }
                let cos = ((ux * vx + uy * vy) / (lu * lv)).clamp(-1.0, 1.0);
                if cos < RING_TURN_COS {
                    breaks.push(arc[i]);
                }
            }
            // The lattice crossings of every edge.
            for i in 0..n {
                let (a, b) = (ring[i], ring[(i + 1) % n]);
                let (ga, gb) = (frame.to_deg(a), frame.to_deg(b));
                let seg = arc[i + 1] - arc[i];
                if seg < 1e-9 {
                    continue;
                }
                for (va, vb, step) in [(ga.x, gb.x, d_lon), (ga.y, gb.y, d_lat)] {
                    if (vb - va).abs() < 1e-12 {
                        continue;
                    }
                    let (lo, hi) = (va.min(vb), va.max(vb));
                    let mut k = (lo / step).ceil();
                    while k * step < hi {
                        let t = (k * step - va) / (vb - va);
                        if t > 0.0 && t < 1.0 {
                            breaks.push(arc[i] + seg * t);
                        }
                        k += 1.0;
                    }
                }
            }
            breaks.sort_by(f64::total_cmp);
            breaks.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
            breaks.push(total);
            for w in breaks.windows(2) {
                let (s0, s1) = (w[0], w[1]);
                if s1 - s0 < 1e-6 {
                    continue;
                }
                // A lattice cell's diagonal bounds a station already; the
                // arc cap only splits what a contour with no crossings left.
                let steps = ((s1 - s0) / (2.0 * RING_STEP_M)).ceil().max(1.0) as usize;
                for k in 0..steps {
                    let a0 = s0 + (s1 - s0) * k as f64 / steps as f64;
                    let a1 = s0 + (s1 - s0) * (k + 1) as f64 / steps as f64;
                    let (p, q) = (point_at(a0), point_at(a1));
                    let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
                    let len = (dx * dx + dy * dy).sqrt();
                    if len < 1e-6 {
                        continue;
                    }
                    let nrm = [dy / len, -dx / len];
                    let mid = [(p[0] + q[0]) * 0.5 + nrm[0] * half, (p[1] + q[1]) * 0.5 + nrm[1] * half];
                    out.push(KerbSeg {
                        p,
                        q,
                        n: nrm,
                        len: a1 - a0,
                        arc: a0,
                        contour,
                        masked: raw.inside(mid) && !others.inside(mid),
                    });
                }
            }
            contour += 1;
        }
    }
    out
}

/// The corner rule, applied along the kerb: a bare stretch of one contour
/// shorter than [`priors::WALK_CORNER_MAX_M`] between two masked stretches is
/// masked — the pavement wraps the corner. Exactly the rule
/// `street.kerb_gap` scores, so the two agree on what a corner is; a contour
/// with no masked station at all (an island, a street nobody paved) stays bare.
fn bridge_along_kerb(kerb: &mut [KerbSeg]) {
    let mut i = 0;
    while i < kerb.len() {
        let c = kerb[i].contour;
        let mut j = i;
        while j < kerb.len() && kerb[j].contour == c {
            j += 1;
        }
        let ring = &mut kerb[i..j];
        let n = ring.len();
        if let Some(start) = ring.iter().position(|k| k.masked) {
            // Runs of bare stations, in contour order from a masked one so a
            // bare run wrapping the seam is one run.
            let mut k = 0;
            while k < n {
                let idx = (start + k) % n;
                if ring[idx].masked {
                    k += 1;
                    continue;
                }
                let (mut len, mut m) = (0.0, k);
                while m < n && !ring[(start + m) % n].masked {
                    len += ring[(start + m) % n].len;
                    m += 1;
                }
                // Bounded by masked stations on both sides by construction
                // (the walk started on one and the loop is closed).
                if len < priors::WALK_CORNER_MAX_M {
                    for t in k..m {
                        ring[(start + t) % n].masked = true;
                    }
                }
                k = m;
            }
        }
        i = j;
    }
}

/// The masked kerb stations as pavement mask: each masked run buffered as one
/// polyline to a pavement's width and a little more, so the ring beside it is
/// wholly inside; consecutive stations share their vertices, so a run is one
/// stroke and not a row of butt-capped pieces.
fn kerb_mask(kerb: &[KerbSeg]) -> Shapes {
    let mut shapes: Shapes = Vec::new();
    let w = priors::WALK_WIDTH_M + RING_MASK_SLACK_M;
    let mut run: Vec<Pt> = Vec::new();
    let mut prev: Option<(usize, Pt)> = None;
    for k in kerb {
        let continues = k.masked && prev.is_some_and(|(c, q)| c == k.contour && q == k.p);
        if !continues {
            if run.len() >= 2 {
                shapes.extend(poly::buffer_line(&run, w));
            }
            run.clear();
            if k.masked {
                run.push(k.p);
            }
        }
        if k.masked {
            run.push(k.q);
            prev = Some((k.contour, k.q));
        } else {
            prev = None;
        }
    }
    if run.len() >= 2 {
        shapes.extend(poly::buffer_line(&run, w));
    }
    if shapes.is_empty() {
        return shapes;
    }
    poly::union_all(&shapes)
}

/// Shortest run of surviving stations kept, metres. A lone station the fit
/// kept between two it refused — the foot of a terrace wall, the one station
/// whose face happened to pass — draws as an orphan slab of pavement four
/// metres long, reading the wall's heights at both ends.
const RING_MIN_RUN_M: f64 = 2.0 * RING_STEP_M;

/// Drops the surviving stations that form a run shorter than
/// [`RING_MIN_RUN_M`] along their contour — consecutive station indices on
/// one contour are one run.
fn drop_orphan_runs(kerb: &[KerbSeg], benches: &mut Vec<SourceSeg>, at: &mut Vec<u64>) {
    if at.is_empty() {
        return;
    }
    let mut keep = vec![true; at.len()];
    let mut i = 0;
    while i < at.len() {
        let mut j = i;
        let mut len = 0.0;
        while j < at.len()
            && (j == i
                || (at[j] == at[j - 1] + 1 && kerb[at[j] as usize].contour == kerb[at[i] as usize].contour))
        {
            len += kerb[at[j] as usize].len;
            j += 1;
        }
        if len < RING_MIN_RUN_M {
            for k in i..j {
                keep[k] = false;
            }
        }
        i = j;
    }
    let mut n = 0;
    benches.retain(|_| {
        n += 1;
        keep[n - 1]
    });
    let mut m = 0;
    at.retain(|_| {
        m += 1;
        keep[m - 1]
    });
}

/// Where the ring survives: every station that kept a bench, as a quad from
/// just inside the kerb to past a pavement's full reach, so the ring's own
/// boundary is never cut back — only the stations that went are removed.
///
/// Quads, not a stroke: a stroke's caps reach a full width past a run's ends
/// along the kerb, and two of them bridged the refused station between them
/// across a terrace wall (6.9151,46.4366). Each quad overruns its station's
/// ends by a hair so neighbours overlap instead of merely touching, which a
/// boolean would keep apart.
fn surviving_mask(kerb: &[KerbSeg], at: &[u64]) -> Shapes {
    const OVERRUN_M: f64 = 0.1;
    let reach = priors::WALK_WIDTH_M + 2.0 * RING_MASK_SLACK_M;
    let mut shapes: Shapes = Vec::new();
    for &ki in at {
        let k = &kerb[ki as usize];
        let (dx, dy) = (k.q[0] - k.p[0], k.q[1] - k.p[1]);
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-9 {
            continue;
        }
        let (ux, uy) = (dx / len * OVERRUN_M, dy / len * OVERRUN_M);
        let (p, q) = ([k.p[0] - ux, k.p[1] - uy], [k.q[0] + ux, k.q[1] + uy]);
        let inset = RING_MASK_SLACK_M;
        shapes.push(vec![vec![
            [p[0] - k.n[0] * inset, p[1] - k.n[1] * inset],
            [q[0] - k.n[0] * inset, q[1] - k.n[1] * inset],
            [q[0] + k.n[0] * reach, q[1] + k.n[1] * reach],
            [p[0] + k.n[0] * reach, p[1] + k.n[1] * reach],
        ]]);
    }
    if shapes.is_empty() {
        return shapes;
    }
    poly::union_all(&shapes)
}

/// The kerb's seat at a point beside the asphalt of `(level, layer)`, and the
/// corridor whose kerb it is: **the road height field's own answer** at the
/// reference rung — the covering carriageway stretches blended by the same
/// kernel `synth::height` uses, the junction pins overriding it the same way —
/// plus the kerb rise. Read from the stamped profile heights rather than the
/// engineered ground, which at the reference rung with the hole cut is what
/// the field returns for asphalt anyway.
///
/// Not the nearest stretch's height alone: around a junction the legs'
/// profiles disagree by decimetres and the plate is what the solve made them
/// agree on, and a bench seated on whichever leg happened to be nearest
/// stepped between stations all the way round a roundabout.
///
/// `None` where no asphalt of that sheet is within a pavement's reach — a
/// ring station the mask admitted but nothing paved — and where the nearest
/// carriageway has no solved profile (stamped at zero, it drapes on the
/// ground and has no kerb height to give).
fn kerb_seat(
    junctions: &CarriagewayModel,
    pins: &[&crate::synth::carriageway::Intersection],
    level: i64,
    layer: u32,
    at: Coord,
    cos_lat: f64,
    scratch: &mut Vec<u32>,
) -> Option<(f64, crate::scene::CorridorId)> {
    let reach_lat = (priors::WALK_WIDTH_M + priors::CURB_RETURN_M + 4.0) / DEG_M;
    let reach_lon = reach_lat / cos_lat.max(1e-6);
    junctions.sources_near((at.x - reach_lon, at.y - reach_lat, at.x + reach_lon, at.y + reach_lat), scratch);
    let mut best: Option<(f64, f64, crate::scene::CorridorId)> = None;
    let (mut num, mut den) = (0.0f64, 0.0f64);
    for &i in scratch.iter() {
        let s = junctions.source(i);
        if s.level != level || s.layer != layer || s.surface != priors::Surface::Asphalt {
            continue;
        }
        if s.height_a == 0.0 && s.height_b == 0.0 {
            continue;
        }
        let (d, t) = crate::synth::sheets::point_to_segment(at, s.a, s.b, s.cos_lat);
        if d > s.half_m + priors::WALK_WIDTH_M + priors::CURB_RETURN_M {
            continue;
        }
        let h = s.height_at(t);
        if best.is_none_or(|(bd, _, _)| d < bd) {
            best = Some((d, h, s.corridor));
        }
        if d <= s.half_m {
            let w = crate::synth::height::kernel(d, s.half_m);
            num += w * h;
            den += w;
        }
    }
    let (_, nearest, corridor) = best?;
    let blended = if den > 0.0 { num / den } else { nearest };
    // The pins, overriding as they do in the field: flat where authoritative,
    // handing back at the paved boundary.
    let (mut pin_num, mut pin_den, mut lambda) = (0.0f64, 0.0f64, 0.0f64);
    for j in pins {
        let Some(height) = j.height() else { continue };
        if level != 0 || j.layer() != layer {
            continue;
        }
        let (c, (rx, ry)) = (j.point(), j.area().reach_deg());
        if (at.x - c.x).abs() > rx || (at.y - c.y).abs() > ry {
            continue;
        }
        let (de, dn) = j.area().offset_m(at);
        let d = (de * de + dn * dn).sqrt();
        if d < 1e-9 {
            return Some((height + priors::KERB_RISE_M, corridor));
        }
        let r = j.area().radius(de / d, dn / d);
        if d <= r {
            let w = crate::synth::height::pin_kernel(d, r);
            pin_num += w * height;
            pin_den += w;
            lambda = lambda.max(w);
        }
    }
    let h = if pin_den > 0.0 {
        let l = lambda.clamp(0.0, 1.0);
        l * (pin_num / pin_den) + (1.0 - l) * blended
    } else {
        blended
    };
    Some((h + priors::KERB_RISE_M, corridor))
}

/// Where along the kerb the ring is pavement, before the corner rule: the
/// hosted bands' own extents grown past their ends by [`RING_CORNER_M`], so
/// two legs' pavements meet across the corner between them; plus the whole
/// extent of every intersection one of those bands reaches, grown the same —
/// a junction whose legs carry pavement carries it round every mouth, and a
/// roundabout is one such junction whose "mouths" are its ring arcs.
fn ring_mask_of(bands: &Shapes, extents: &Shapes) -> Shapes {
    if bands.is_empty() {
        return Vec::new();
    }
    let along = poly::dilate(&poly::union_all(bands), RING_CORNER_M);
    if along.is_empty() || extents.is_empty() {
        return along;
    }
    let mut served: Shapes = Vec::new();
    for extent in extents {
        let one: Shapes = vec![extent.clone()];
        if !poly::intersect(&one, &along).is_empty() {
            served.push(extent.clone());
        }
    }
    if served.is_empty() {
        return along;
    }
    let mut all = along;
    all.extend(poly::dilate(&poly::union_all(&served), RING_CORNER_M));
    poly::union_all(&all)
}

/// The building footprints near a chunk, grown by [`priors::FACADE_CLEAR_M`],
/// in chunk metres — what the sidewalk ring keeps out of. Empty with no
/// building input, so the ring then runs to its full width everywhere, which
/// is what an unsurveyed town looks like from the band generator's side too.
fn wall_shapes(walls: Option<&Facades>, rect: &Bounds, frame: &MFrame) -> Shapes {
    let Some(f) = walls else { return Vec::new() };
    let pad = (PAVE_PAD_M + priors::WALK_WIDTH_M + priors::FACADE_CLEAR_M) / DEG_M;
    let mut ids = Vec::new();
    f.rings_near((rect.west - pad, rect.south - pad, rect.east + pad, rect.north + pad), &mut ids);
    let mut shapes: Shapes = Vec::new();
    for i in ids {
        let r = f.ring(i);
        let mut pts: Vec<[f64; 2]> = r.iter().map(|&c| frame.to_m(c)).collect();
        if pts.len() > 1 && pts.first() == pts.last() {
            pts.pop();
        }
        if pts.len() < 3 {
            continue;
        }
        // Counter-clockwise, so it reads as filled area under the non-zero rule.
        let twice_area: f64 =
            pts.iter().zip(pts.iter().cycle().skip(1)).map(|(a, b)| a[0] * b[1] - b[0] * a[1]).sum();
        if twice_area < 0.0 {
            pts.reverse();
        }
        shapes.push(vec![pts]);
    }
    if shapes.is_empty() {
        return shapes;
    }
    poly::dilate(&poly::union_all(&shapes), priors::FACADE_CLEAR_M)
}

/// Every nearby intersection's extent as one shape each, in chunk metres. The
/// closing mask is their union dilated by the curb-return radius, so a fillet
/// that reaches just outside the paved area is still inside its own mask; the
/// sidewalk ring reads them one by one to ask which junctions a pavement
/// reaches.
fn intersection_shapes(
    junctions: &CarriagewayModel,
    rect: &Bounds,
    pad_deg: f64,
    frame: &MFrame,
) -> Shapes {
    let box_ =
        (rect.west - pad_deg, rect.south - pad_deg, rect.east + pad_deg, rect.north + pad_deg);
    let mut masks: Shapes = Vec::new();
    for j in junctions.near(box_) {
        let area = j.area();
        let centre = frame.to_m(area.centre());
        let ring: Vec<[f64; 2]> = area
            .ring()
            .map(|(e, n)| [centre[0] + e, centre[1] + n])
            .collect();
        if ring.len() >= 3 {
            masks.push(vec![ring]);
        }
    }
    masks
}

/// Puts a converted vertex exactly on the chunk rect where the metre-space clip
/// put it exactly on the metre rect.
///
/// The clip is exact in metres (see the `poly` tests), so a cut vertex's metre
/// coordinate equals the rect bound bit-for-bit. Converting it through the
/// chunk's own frame would leave it a rounding step off the shared boundary, and
/// the neighbouring chunk — converting through *its* frame — would land a
/// different rounding step away. Assigning the rect's own literal instead makes
/// both sides agree exactly.
fn snap_to_rect(c: Coord, m: [f64; 2], m_rect: (f64, f64, f64, f64), rect: &Bounds) -> Coord {
    let mut out = c;
    if (m[0] - m_rect.0).abs() <= CUT_EPS_M {
        out.x = rect.west;
    } else if (m[0] - m_rect.2).abs() <= CUT_EPS_M {
        out.x = rect.east;
    }
    if (m[1] - m_rect.1).abs() <= CUT_EPS_M {
        out.y = rect.south;
    } else if (m[1] - m_rect.3).abs() <= CUT_EPS_M {
        out.y = rect.north;
    }
    out
}

/// How close to the chunk rect, in metres, a clipped vertex counts as *on* it.
///
/// Not an exact comparison: the boolean kernel snaps every coordinate to its own
/// fixed grid, including the clip rect's, so a cut vertex sits on the *snapped*
/// bound rather than on the metre value computed here. A millimetre is an order
/// of magnitude above that grid and orders below anything geometric, so it
/// catches every cut vertex and nothing else. A legitimate vertex within a
/// millimetre of the boundary being pulled onto it is harmless — that is the
/// seam closing.
const CUT_EPS_M: f64 = 1e-3;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assemble::facades::Facades;
    use crate::priors::{Kind, RoadClass};
    use crate::scene::{Corridor, SceneGraph};
    use crate::synth::carriageway::SourceSeg;
    use crate::solve::SolvedModel;
    use crate::synth::carriageway;

    const LAT: f64 = 46.0;

    fn m_lon() -> f64 {
        DEG_M * LAT.to_radians().cos()
    }

    /// A straight corridor from `(lon0, lat0)` running `len_m` on the heading
    /// `(de, dn)`, with `n` nodes.
    fn corridor(
        id: u32,
        lon0: f64,
        lat0: f64,
        de: f64,
        dn: f64,
        len_m: f64,
        n: usize,
        width_m: f64,
    ) -> Corridor {
        let step = len_m / (n - 1) as f64;
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord {
                x: lon0 + de * i as f64 * step / m_lon(),
                y: lat0 + dn * i as f64 * step / DEG_M,
            })
            .collect();
        Corridor {
            id,
            nodes,
            arc: (0..n).map(|i| i as f64 * step).collect(),
            cos_lat: LAT.to_radians().cos(),
            kind: Kind::Road(RoadClass::Residential),
            class_key: "residential".to_string(),
            link: false,
            width_m: Some(width_m),
            spans: Vec::new(),
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }

    /// Bakes a scene with no junctions (so no fillets) and returns the model.
    fn bake_scene(corridors: Vec<Corridor>) -> PavementModel {
        let scene = SceneGraph::new(corridors);
        let solved = SolvedModel::from_profiles((0..scene.corridors.len()).map(|_| None).collect(), 15);
        let junctions = carriageway::bake(&scene, &solved, &Facades::empty(), Vec::new());
        bake(&junctions, 1, None, None, None)
    }

    #[test]
    fn two_crossing_roads_bake_into_one_region() {
        // A 200 m east-west 8 m road and a 200 m north-south 6 m road crossing at
        // the origin: one region, no hole, area by inclusion-exclusion.
        let ew = corridor(0, 6.0 - 100.0 / m_lon(), LAT, 1.0, 0.0, 200.0, 11, 6.0);
        let ns = corridor(1, 6.0, LAT - 100.0 / DEG_M, 0.0, 1.0, 200.0, 11, 4.0);
        let model = bake_scene(vec![ew, ns]);
        assert_eq!(model.chunk_count(), 1, "the whole cross is in one chunk");
        let levels = model.chunk_for(&crate::solve::tile_containing(15, 6.0, LAT)).expect("asphalt");
        assert_eq!(levels.len(), 1, "one level");
        assert_eq!(levels[0].level, 0);
        assert_eq!(levels[0].shapes.len(), 1, "the crossing is one region");
        assert_eq!(levels[0].shapes[0].len(), 1, "and it has no hole");
        // Half-widths are width/2 + STRUCTURE_SHOULDER_M: 4 m and 3 m, so the
        // bands are 8 m and 6 m wide.
        let want = 200.0 * 8.0 + 200.0 * 6.0 - 8.0 * 6.0;
        let got = model.area_m2();
        assert!((got - want).abs() < want * 0.02, "area {got:.0} != {want:.0}");
    }

    #[test]
    fn a_ring_of_roads_keeps_its_island() {
        // Four sides of a 60 m square: the region has exactly one hole.
        let d = 30.0;
        let cs = vec![
            corridor(0, 6.0 - d / m_lon(), LAT - d / DEG_M, 1.0, 0.0, 2.0 * d, 5, 5.0),
            corridor(1, 6.0 + d / m_lon(), LAT - d / DEG_M, 0.0, 1.0, 2.0 * d, 5, 5.0),
            corridor(2, 6.0 - d / m_lon(), LAT + d / DEG_M, 1.0, 0.0, 2.0 * d, 5, 5.0),
            corridor(3, 6.0 - d / m_lon(), LAT - d / DEG_M, 0.0, 1.0, 2.0 * d, 5, 5.0),
        ];
        let model = bake_scene(cs);
        let levels = model.chunk_for(&crate::solve::tile_containing(15, 6.0, LAT)).expect("asphalt");
        assert_eq!(levels[0].shapes.len(), 1, "one band");
        assert_eq!(levels[0].shapes[0].len(), 2, "outer boundary plus one island");
    }

    #[test]
    fn a_divided_carriageway_does_not_fuse() {
        // Two parallel 8 m carriageways with a 5 m median — narrower than the
        // 2 x CURB_RETURN_M a global closing would bridge. With no intersection
        // between them there is no mask, so they must stay two regions.
        let north = corridor(0, 6.0 - 100.0 / m_lon(), LAT + 6.5 / DEG_M, 1.0, 0.0, 200.0, 11, 6.0);
        let south = corridor(1, 6.0 - 100.0 / m_lon(), LAT - 6.5 / DEG_M, 1.0, 0.0, 200.0, 11, 6.0);
        let model = bake_scene(vec![north, south]);
        let levels = model.chunk_for(&crate::solve::tile_containing(15, 6.0, LAT)).expect("asphalt");
        assert_eq!(levels[0].shapes.len(), 2, "the median was bridged");
    }

    /// A hand-built walkway band, the shape `street_bands` emits: uniform
    /// section, the kerb rise, no host.
    fn walk_band(a: Coord, b: Coord, half_m: f64, z: f64) -> SourceSeg {
        use crate::assemble::facades::Section;
        SourceSeg {
            a,
            b,
            cos_lat: LAT.to_radians().cos(),
            half_m,
            sect_a: Section::uniform(half_m),
            sect_b: Section::uniform(half_m),
            level: 0,
            layer: 0,
            cut_a: None,
            cut_b: None,
            height_a: z,
            height_b: z,
            corridor: crate::synth::walkway::NO_HOST,
            surface: priors::Surface::Walkway,
            rise_m: priors::KERB_RISE_M,
            arc0: 0.0,
        }
    }

    /// The walkway regions of the chunk holding the test crossing.
    fn walk_shapes(model: &PavementModel) -> Vec<usize> {
        let levels = model.chunk_for(&crate::solve::tile_containing(15, 6.0, LAT)).expect("paved");
        levels
            .iter()
            .filter(|l| l.surface == priors::Surface::Walkway)
            .map(|l| l.shapes.len())
            .collect()
    }

    #[test]
    fn a_band_on_the_carriageway_yields_the_crossing() {
        // An east-west road at grade and a walkway band crossing it at the
        // kerb rise — the layer numbers agree here (both flat sheets read 0),
        // but the yield must not depend on that: the two layerings are
        // separate namespaces, and the height is what says the band is ON the
        // plate. The road's cross-section is cut out of the band, so the band
        // survives as its two kerb-to-kerb halves.
        let road = corridor(0, 6.0 - 100.0 / m_lon(), LAT, 1.0, 0.0, 200.0, 11, 6.0);
        let band = walk_band(
            Coord { x: 6.0, y: LAT - 20.0 / DEG_M },
            Coord { x: 6.0, y: LAT + 20.0 / DEG_M },
            1.5,
            priors::KERB_RISE_M,
        );
        let scene = SceneGraph::new(vec![road]);
        let solved = SolvedModel::from_profiles(vec![None], 15);
        let junctions = carriageway::bake(&scene, &solved, &Facades::empty(), vec![band]);
        let model = bake(&junctions, 1, None, None, None);
        assert_eq!(walk_shapes(&model), vec![2], "the band must be severed at the kerbs");
    }

    #[test]
    fn a_band_a_storey_up_keeps_its_plan_space() {
        // The same crossing five metres up: a footbridge, or the rim path
        // above a sunken road the walk-sheet split draws honestly. Height is
        // the only thing separating it from the yielded band above, and it
        // must survive whole.
        let road = corridor(0, 6.0 - 100.0 / m_lon(), LAT, 1.0, 0.0, 200.0, 11, 6.0);
        let band = walk_band(
            Coord { x: 6.0, y: LAT - 20.0 / DEG_M },
            Coord { x: 6.0, y: LAT + 20.0 / DEG_M },
            1.5,
            5.0,
        );
        let scene = SceneGraph::new(vec![road]);
        let solved = SolvedModel::from_profiles(vec![None], 15);
        let junctions = carriageway::bake(&scene, &solved, &Facades::empty(), vec![band]);
        let model = bake(&junctions, 1, None, None, None);
        assert_eq!(walk_shapes(&model), vec![1], "a stacked band is not on the plate");
    }

    #[test]
    fn a_bridge_span_is_not_paved_by_the_union() {
        use crate::scene::{Span, SpanKind};
        // An east-west road at grade, and a north-south one whose middle span is a
        // bridge. The bridge already carries its road surface as a swept solid, so
        // the union must skip it: only at-grade asphalt is paved, which leaves the
        // north-south road as two separate approaches with a gap where it flies
        // over.
        let ew = corridor(0, 6.0 - 100.0 / m_lon(), LAT, 1.0, 0.0, 200.0, 11, 6.0);
        let mut ns = corridor(1, 6.0, LAT - 100.0 / DEG_M, 0.0, 1.0, 200.0, 11, 6.0);
        ns.spans = vec![
            Span { arc0: 0.0, arc1: 80.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 80.0, arc1: 120.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 120.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
        ];
        let model = bake_scene(vec![ew, ns]);
        let levels = model.chunk_for(&crate::solve::tile_containing(15, 6.0, LAT)).expect("asphalt");
        assert_eq!(levels.len(), 1, "only the at-grade level is paved");
        assert_eq!(levels[0].level, 0);
        // The east-west road, plus the two north-south approaches the flyover
        // leaves behind: three disjoint regions, no asphalt over the span.
        assert_eq!(
            levels[0].shapes.len(),
            3,
            "expected the crossing road and two approaches, got {}",
            levels[0].shapes.len()
        );
    }

    #[test]
    fn the_bake_does_not_depend_on_chunk_worker_count() {
        // Determinism across the thread fan-out: the same rings, whatever the
        // worker count.
        let make = || {
            vec![
                corridor(0, 6.0 - 100.0 / m_lon(), LAT, 1.0, 0.0, 200.0, 11, 6.0),
                corridor(1, 6.0, LAT - 100.0 / DEG_M, 0.0, 1.0, 200.0, 11, 4.0),
            ]
        };
        let scene = SceneGraph::new(make());
        let solved = SolvedModel::from_profiles((0..2).map(|_| None).collect(), 15);
        let junctions = carriageway::bake(&scene, &solved, &Facades::empty(), Vec::new());
        let one = bake(&junctions, 1, None, None, None);
        let many = bake(&junctions, 8, None, None, None);
        assert_eq!(one.chunk_count(), many.chunk_count());
        let b = crate::solve::tile_containing(15, 6.0, LAT);
        let a = one.chunk_for(&b).expect("asphalt");
        let c = many.chunk_for(&b).expect("asphalt");
        assert_eq!(a.len(), c.len());
        for (x, y) in a.iter().zip(c) {
            assert_eq!(x.level, y.level);
            assert_eq!(x.shapes, y.shapes, "rings differ with the worker count");
        }
    }

    #[test]
    fn a_region_crossing_a_chunk_edge_is_cut_exactly_on_it() {
        // A road running east through a chunk boundary: the cut vertices must sit
        // on the boundary's own longitude, bit-for-bit, so the neighbouring chunk
        // (which cuts the same road against the same literal) matches it exactly.
        let n = 1u32 << PAVE_BAKE_Z;
        let lon_span = 360.0 / n as f64;
        let edge_lon = -180.0 + ((6.0 + 180.0) / lon_span).ceil() * lon_span;
        let start = edge_lon - 300.0 / m_lon();
        let road = corridor(0, start, LAT, 1.0, 0.0, 600.0, 21, 6.0);
        let model = bake_scene(vec![road]);
        assert!(model.chunk_count() >= 2, "the road should span two chunks");

        // West chunk: its eastern cut is exactly the boundary longitude.
        let west = model
            .chunk_for(&crate::solve::tile_containing(15, edge_lon - 100.0 / m_lon(), LAT))
            .expect("asphalt west of the edge");
        let on_edge: Vec<Coord> = west[0]
            .shapes
            .iter()
            .flatten()
            .flatten()
            .filter(|c| c.x == edge_lon)
            .copied()
            .collect();
        assert!(!on_edge.is_empty(), "no vertex landed on the chunk edge");

        // East chunk: its western cut is the same literal, and the two sides'
        // latitude lists match — the seam is closed.
        let east = model
            .chunk_for(&crate::solve::tile_containing(15, edge_lon + 100.0 / m_lon(), LAT))
            .expect("asphalt east of the edge");
        let mut a: Vec<f64> = on_edge.iter().map(|c| c.y).collect();
        let mut b: Vec<f64> = east[0]
            .shapes
            .iter()
            .flatten()
            .flatten()
            .filter(|c| c.x == edge_lon)
            .map(|c| c.y)
            .collect();
        a.sort_by(f64::total_cmp);
        b.sort_by(f64::total_cmp);
        assert_eq!(a, b, "the two chunks disagree on the seam");
    }

    /// A four-way crossing built as four legs meeting at one connector, with
    /// profiles so the intersection actually bakes an extent to mask with.
    fn crossroads() -> (SceneGraph, SolvedModel) {
        use crate::scene::{Junction, JunctionMember};
        let arm = 60.0;
        let cs = vec![
            corridor(0, 6.0 - arm / m_lon(), LAT, 1.0, 0.0, arm, 4, 6.0),
            corridor(1, 6.0, LAT, 1.0, 0.0, arm, 4, 6.0),
            corridor(2, 6.0, LAT - arm / DEG_M, 0.0, 1.0, arm, 4, 6.0),
            corridor(3, 6.0, LAT, 0.0, 1.0, arm, 4, 6.0),
        ];
        let profiles: Vec<Option<crate::solve::Profile>> =
            cs.iter().map(|c| Some(crate::solve::Profile::flat(&c.nodes, 400.0))).collect();
        let mut scene = SceneGraph::new(cs);
        scene.junctions = vec![Junction {
            point: Coord { x: 6.0, y: LAT },
            connector: 0,
            members: vec![
                JunctionMember { corridor: 0, arc: arm },
                JunctionMember { corridor: 1, arc: 0.0 },
                JunctionMember { corridor: 2, arc: arm },
                JunctionMember { corridor: 3, arc: 0.0 },
            ],
        }];
        let solved =
            SolvedModel::from_profiles(profiles, 15).with_junction_heights(vec![Some(400.0)]);
        (scene, solved)
    }

    #[test]
    fn the_closing_rounds_the_curb_returns_at_an_intersection() {
        let (scene, solved) = crossroads();
        let junctions = carriageway::bake(&scene, &solved, &Facades::empty(), Vec::new());
        assert_eq!(junctions.len(), 1, "the crossroads plates as one intersection");
        let model = bake(&junctions, 1, None, None, None);
        let filleted = model.area_m2();

        // The same network with the intersection extent withheld: no mask, so no
        // closing, so hard reflex corners.
        let bare = {
            let bare_scene = SceneGraph::new(
                scene.corridors.iter().map(|c| clone_corridor(c)).collect::<Vec<_>>(),
            );
            let bare_solved =
                SolvedModel::from_profiles((0..4).map(|_| None).collect(), 15);
            let j = carriageway::bake(&bare_scene, &bare_solved, &Facades::empty(), Vec::new());
            assert_eq!(j.len(), 0, "no intersection extent without profiles");
            bake(&j, 1, None, None, None).area_m2()
        };

        assert!(filleted > bare, "the closing added no fillet area at all");
        // Four curb returns of radius CURB_RETURN_M add roughly four
        // corner-minus-quadrant pieces: r^2 - pi r^2/4 each, ~7.7 m^2 in total.
        // Bound it loosely on both sides — the point is that fillets appeared and
        // that the closing did not flood the network.
        let added = filleted - bare;
        assert!(added > 1.0, "only {added:.2} m2 of fillet: the closing barely fired");
        assert!(added < 40.0, "{added:.2} m2 added: the closing spilled past the corners");
    }

    /// A structural copy of a corridor (it is not `Clone`).
    fn clone_corridor(c: &Corridor) -> Corridor {
        Corridor {
            id: c.id,
            nodes: c.nodes.clone(),
            arc: c.arc.clone(),
            cos_lat: c.cos_lat,
            kind: c.kind,
            class_key: c.class_key.clone(),
            link: c.link,
            width_m: c.width_m,
            spans: c.spans.clone(),
            segments: Vec::new(),
            connectors: c.connectors.clone(),
        }
    }

    #[test]
    fn a_flyover_does_not_merge_with_the_road_it_passes_over() {
        // The defect this pins: a flyover's *approaches* are ordinary at-grade
        // spans at level 0, exactly like the road it passes over, so keying the
        // union on level alone merged them into one region — and the mesh then
        // ramped continuously between two roads metres apart vertically.
        //
        // The ordering comes from the solved heights (`synth::sheets`), not from
        // a mapped crossing: this pair carries no bridge annotation at all, and
        // separates anyway.
        let over = corridor(0, 6.0 - 100.0 / m_lon(), LAT, 1.0, 0.0, 200.0, 11, 8.0);
        let under = corridor(1, 6.0, LAT - 100.0 / DEG_M, 0.0, 1.0, 200.0, 11, 8.0);
        let scene = SceneGraph::new(vec![over, under]);
        let nodes: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        let solved = SolvedModel::from_profiles(
            vec![
                Some(crate::solve::Profile::flat(&nodes[0], 410.0)),
                Some(crate::solve::Profile::flat(&nodes[1], 400.0)),
            ],
            15,
        );
        let junctions = carriageway::bake(&scene, &solved, &Facades::empty(), Vec::new());

        let model = bake(&junctions, 1, None, None, None);
        let levels =
            model.chunk_for(&crate::solve::tile_containing(15, 6.0, LAT)).expect("asphalt");
        assert_eq!(levels.len(), 2, "the two roads must be separate regions");
        let layers: Vec<u32> = levels.iter().map(|l| l.layer).collect();
        assert!(layers.contains(&0) && layers.contains(&1), "layers {layers:?}");

        // Each road is one unmerged ribbon: the lift covers the upper road's
        // whole run, approaches included, because a layer that changes along a
        // road puts a drawn region boundary across its carriageway
        // (`synth::sheets`). 200 m x 10 m each (width + 2 x shoulder).
        for ls in levels {
            assert_eq!(ls.shapes.len(), 1, "a layer should hold one ribbon");
        }
    }

    #[test]
    fn a_terrace_street_yields_its_overhang_over_an_open_trench() {
        // The Territet defect (docs/GENERATION.md I9): a street runs along an
        // open rail cutting, close enough that its prior-width band spills
        // over the trench the railway sits in, eleven metres down. The road is
        // junior to independent rail, and the overlap is lateral, so the road
        // yields the contested strip: its region retreats to the formation's
        // edge, gains a rim there, and the apron machinery closes the wall.
        // The formation itself is untouched.
        let road = corridor(0, 6.0 - 100.0 / m_lon(), LAT, 1.0, 0.0, 200.0, 11, 8.0);
        let mut rail =
            corridor(1, 6.0 - 100.0 / m_lon(), LAT - 6.0 / DEG_M, 1.0, 0.0, 200.0, 11, 5.0);
        rail.kind = crate::priors::Kind::Rail(crate::priors::RailClass::StandardGauge);
        rail.class_key = "standard_gauge".to_string();
        let scene = SceneGraph::new(vec![road, rail]);
        let nodes: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        let solved = SolvedModel::from_profiles(
            vec![
                Some(crate::solve::Profile::flat(&nodes[0], 411.0)),
                Some(crate::solve::Profile::flat(&nodes[1], 400.0)),
            ],
            15,
        );
        let junctions = carriageway::bake(&scene, &solved, &Facades::empty(), Vec::new());
        let model = bake(&junctions, 1, None, None, None);
        let levels =
            model.chunk_for(&crate::solve::tile_containing(15, 6.0, LAT)).expect("surfaces");
        let road_ls = levels
            .iter()
            .find(|l| l.surface == crate::priors::Surface::Asphalt)
            .expect("asphalt");
        let rail_ls = levels
            .iter()
            .find(|l| l.surface == crate::priors::Surface::Ballast)
            .expect("ballast");
        // The road band is 8 + shoulder wide either side of a centerline 8 m
        // north of the rail's; the rail band reaches to 5 + shoulder of its
        // own. The two overlap laterally by several metres, and the road's
        // area must come out short of its unyielded footprint by about that
        // overlap, while the rail keeps its full ribbon.
        let area = |ls: &LevelShapes| -> f64 {
            ls.shapes
                .iter()
                .flat_map(|sh| sh.iter())
                .map(|ring| {
                    let mut a = 0.0;
                    for i in 0..ring.len() {
                        let p = ring[i];
                        let q = ring[(i + 1) % ring.len()];
                        a += (p.x * q.y - q.x * p.y) * m_lon() * DEG_M;
                    }
                    a / 2.0
                })
                .sum()
        };
        let road_area = area(road_ls).abs();
        let rail_area = area(rail_ls).abs();
        let road_half = 8.0 / 2.0 + priors::STRUCTURE_SHOULDER_M;
        let rail_half = 5.0 / 2.0; // ballast carries no asphalt shoulder
        let overlap = (road_half + rail_half - 6.0).max(0.0);
        assert!(overlap > 1.0, "the fixture must actually overlap ({overlap:.2} m)");
        let full_road = 200.0 * 2.0 * road_half;
        assert!(
            road_area < full_road - 0.5 * overlap * 200.0,
            "the road must yield its overhang: area {road_area:.0} of {full_road:.0}"
        );
        let full_rail = 200.0 * 2.0 * rail_half;
        assert!(
            rail_area > full_rail * 0.95,
            "the formation keeps its ribbon: area {rail_area:.0} of {full_rail:.0}"
        );
    }

    #[test]
    fn a_level_crossing_keeps_ballast_and_asphalt_apart_and_the_asphalt_wins() {
        // S15: a railway and a street meet at grade. The two are separate
        // regions — a formation must not merge into the carriageway that
        // crosses it — and where they coincide in plan at the same height the
        // asphalt wins the fill: the ballast is trimmed under it, so the two
        // coplanar surfaces cannot z-fight. The road cuts the formation into
        // its two approaches.
        let road = corridor(0, 6.0 - 100.0 / m_lon(), LAT, 1.0, 0.0, 200.0, 11, 6.0);
        let mut rail = corridor(1, 6.0, LAT - 100.0 / DEG_M, 0.0, 1.0, 200.0, 11, 5.0);
        rail.kind = crate::priors::Kind::Rail(crate::priors::RailClass::StandardGauge);
        rail.class_key = "standard_gauge".to_string();
        let model = bake_scene(vec![road, rail]);
        let levels = model.chunk_for(&crate::solve::tile_containing(15, 6.0, LAT)).expect("surfaces");
        assert_eq!(levels.len(), 2, "asphalt and ballast must be separate regions");
        let asphalt = &levels[0];
        let ballast = &levels[1];
        assert_eq!(asphalt.surface, crate::priors::Surface::Asphalt);
        assert_eq!(ballast.surface, crate::priors::Surface::Ballast);
        assert_eq!(asphalt.shapes.len(), 1, "the road is one band");
        assert_eq!(
            ballast.shapes.len(),
            2,
            "the crossing must cut the formation into two approaches"
        );
        // Area by inclusion-exclusion, with the whole overlap charged to the
        // road: an asphalt band is width + 2 x STRUCTURE_SHOULDER_M wide, a
        // ballast band is its track-zone width outright (`Prior::shoulder_m`).
        let (road_w, rail_w) = (8.0, 5.0);
        let want = 200.0 * road_w + 200.0 * rail_w - road_w * rail_w;
        let got = model.area_m2();
        assert!((got - want).abs() < want * 0.02, "area {got:.0} != {want:.0}");
    }

    #[test]
    fn a_network_with_nothing_paved_bakes_nothing() {
        let mut path = corridor(0, 6.0, LAT, 1.0, 0.0, 200.0, 11, 6.0);
        path.kind = crate::priors::Kind::Road(crate::priors::RoadClass::Footway);
        path.width_m = None;
        let model = bake_scene(vec![path]);
        assert_eq!(model.chunk_count(), 0);
        assert_eq!(model.area_m2(), 0.0);
    }
}
