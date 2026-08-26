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
use crate::synth::carriageway::{Handover, CarriagewayModel};
use crate::synth::poly::{self, MFrame, Shapes};

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
pub fn bake(junctions: &CarriagewayModel, threads: usize) -> PavementModel {
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
    let out: std::sync::Mutex<Vec<(ChunkKey, Vec<LevelShapes>)>> =
        std::sync::Mutex::new(Vec::new());
    let threads = threads.max(1).min(keys.len().max(1));
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| loop {
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
                let levels = bake_chunk(junctions, k, &by_chunk[&k]);
                if std::env::var_os("ARPT_PAVE_PROBE").is_some() && t.elapsed().as_millis() > 200 {
                    eprintln!(
                        "[pave] chunk {:?}: {} sources -> {} levels in {:?}",
                        k,
                        by_chunk[&k].len(),
                        levels.len(),
                        t.elapsed()
                    );
                }
                if !levels.is_empty() {
                    out.lock().expect("pavement results poisoned").push((k, levels));
                }
            });
        }
    });

    let chunks: HashMap<ChunkKey, Vec<LevelShapes>> =
        out.into_inner().expect("pavement results poisoned").into_iter().collect();

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
    PavementModel { chunks, handovers }
}

/// Bakes one chunk: buffer every source, union per level, round the curb returns
/// at the intersections, clip to the chunk rect.
fn bake_chunk(
    junctions: &CarriagewayModel,
    key: ChunkKey,
    source_ids: &[u32],
) -> Vec<LevelShapes> {
    let (cx, cy) = key;
    let rect = chunk_bounds(cx, cy);
    let frame = MFrame::of(chunk_centre(cx, cy));

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
        if !buffered.is_empty() {
            // Keyed on the **drawn** material rather than the modelled one
            // where [`drawn`] says so: a footway and the sidewalk it runs into
            // would then be one region, unioning instead of the junior being
            // subtracted under the senior and each wearing its own rim.
            // Opt-in — see [`drawn`] for what the measurement said about it.
            by_level.entry((run.level, run.layer, drawn(run.surface))).or_default().extend(buffered);
        }
    }
    // Sorted by (level, layer) and asphalt before ballast within a pair, so
    // the output order — and the subtraction below — is a function of the
    // model, never of hashing.
    let mut levels: Vec<(i64, u32, priors::Surface)> = by_level.keys().copied().collect();
    levels.sort_unstable_by_key(|&(level, layer, surface)| (level, layer, material_rank(surface)));

    // The closing masks: each intersection's paved extent, in chunk metres. Only
    // the intersections near this chunk matter, and only their *shape* — the
    // fillet material is whatever the closing adds inside one of these.
    let pad = (PAVE_PAD_M + priors::CURB_RETURN_M) / DEG_M;
    let masks = intersection_masks(junctions, &rect, pad, &frame);

    let mut out = Vec::new();
    for (level, layer, surface) in levels {
        let open = poly::union_all(&by_level[&(level, layer, surface)]);
        if open.is_empty() {
            continue;
        }
        let mut closed = poly::close_within(&open, priors::CURB_RETURN_M, &masks);
        // Where two materials share a level and a layer their regions coincide
        // in plan at the same height and two coplanar surfaces z-fight. The
        // more physical one wins the fill and the junior region is trimmed
        // under it: asphalt over ballast, because a level crossing's
        // carriageway is what is physically laid across the formation; both
        // over the walkway, because a sidewalk that overlaps a carriageway is
        // a band whose seat the room could not narrow far enough, not a
        // pavement laid on the road.
        for &senior in seniors(surface) {
            let Some(shapes) = by_level.get(&(level, layer, senior)) else { continue };
            let senior_closed =
                poly::close_within(&poly::union_all(shapes), priors::CURB_RETURN_M, &masks);
            closed = poly::difference(&closed, &senior_closed);
            if closed.is_empty() {
                break;
            }
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
    out
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

/// Which materials outrank this one where the two coincide in plan on one
/// sheet — what is subtracted from it before it is emitted.
///
/// The order is physical, not arbitrary: the carriageway is laid across the
/// formation at a level crossing, and both are laid before the footway beside
/// them. It is also the emission order ([`material_rank`]), so a region is
/// always trimmed against one already resolved.
fn seniors(surface: priors::Surface) -> &'static [priors::Surface] {
    match surface {
        priors::Surface::Asphalt => &[],
        priors::Surface::Ballast => &[priors::Surface::Asphalt],
        priors::Surface::Walkway => &[priors::Surface::Asphalt, priors::Surface::Ballast],
        priors::Surface::Path => {
            &[priors::Surface::Asphalt, priors::Surface::Ballast, priors::Surface::Walkway]
        }
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
    for &i in source_ids {
        let s = junctions.source(i);
        let continues = out.last().is_some_and(|r| {
            let last = *r.line.last().expect("a run has points");
            r.level == s.level
                && r.layer == s.layer
                && r.surface == s.surface
                && r.half_m == s.half_m
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
            });
        }
    }
    out
}

/// Every nearby intersection's extent as a shape in chunk metres, dilated by the
/// curb-return radius so a fillet that reaches just outside the paved area is
/// still inside its own mask.
fn intersection_masks(
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
    if masks.is_empty() {
        return masks;
    }
    let merged = poly::union_all(&masks);
    poly::dilate(&merged, priors::CURB_RETURN_M)
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
        bake(&junctions, 1)
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
        let one = bake(&junctions, 1);
        let many = bake(&junctions, 8);
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
        let model = bake(&junctions, 1);
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
            bake(&j, 1).area_m2()
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

        let model = bake(&junctions, 1);
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
