//! Stage 3 — the engineered ground (docs/GENERATION.md §6, invariant 1).
//!
//! One authoritative ground function that every later consumer reads: terrain
//! meshing, road draping, building founding, structure contact. The function
//! is the natural DEM plus the earthworks the solved model implies, applied
//! as local modifiers ([`modifiers::Earthworks`]).
//!
//! [`derive`] translates the solved model into earthworks: wherever a
//! profiled corridor's at-grade road departs the natural ground — a
//! grade-limited cut through a bump, the embankment ramp the clearance solver
//! demanded for an overpass approach — the ground is reshaped to carry it
//! (D3). Consumers are untouched: they already read through
//! [`sampler::GroundSampler`].

pub mod modifiers;
pub mod sampler;

use std::collections::HashMap;
use std::path::Path;
use std::sync::Mutex;

use geo_types::Coord;

use crate::dem::Dem;
use crate::priors::{
    BED_MAX_DEVIATION_M, BED_WELD_MAX_M, DECK_THICKNESS_M, EARTHWORK_BATTER,
    EARTHWORK_MARGIN_M, EARTHWORK_MIN_FEATHER_M, EARTHWORK_SHOULDER_M, MAX_CLEARANCE_LIFT_M,
    MIN_EARTHWORK_M, PORTAL_CLEARANCE_M, PORTAL_CUT_LEN_M, WATER_LEVEL_PCTL,
};
use crate::scene::{SceneGraph, SpanKind, DEG_M};
use crate::solve::{portals, reference_surface, SolvedModel};

use modifiers::{EarthworkEdge, Earthworks, WaterFill, Waters};

/// Most shoreline vertices sampled when reading a water body's level — enough
/// to be robust on a big lake, bounded so a many-thousand-vertex ring is cheap.
const SHORELINE_SAMPLES: usize = 128;

/// The engineered ground: the single ground function of invariant 1. Queries
/// apply the covering water surface and earthworks to the raw DEM sample in a
/// fixed global order, so any two tiles (and any two zooms) derive identical
/// ground for shared world points.
pub struct GroundModel {
    earthworks: Earthworks,
    waters: Waters,
}

impl GroundModel {
    /// A ground model with no modifiers: the raw DEM passes through.
    pub fn empty() -> GroundModel {
        GroundModel { earthworks: Earthworks::new(Vec::new()), waters: Waters::new(Vec::new()) }
    }

    /// Number of earthwork edges, for run stats.
    pub fn earthwork_count(&self) -> usize {
        self.earthworks.len()
    }

    /// Number of flattened water bodies, for run stats.
    pub fn water_count(&self) -> usize {
        self.waters.len()
    }

    pub fn earthworks(&self) -> &Earthworks {
        &self.earthworks
    }

    /// THE ground function: the engineered height at `(lon, lat)`, given the
    /// raw DEM sample `raw` for that point. `scratch` is the caller's reusable
    /// query buffer (see [`sampler::GroundSampler`]).
    ///
    /// Water flattens the ground first; a road earthwork (a bridge abutment's
    /// approach berm at the shore) then overrides it where the two overlap, so
    /// the roadbed wins over the water it climbs away from.
    pub fn height(&self, lon: f64, lat: f64, raw: f64, scratch: &mut Vec<u32>) -> f64 {
        let base = if self.waters.is_empty() {
            raw
        } else {
            self.waters.level_at(lon, lat, scratch).unwrap_or(raw)
        };
        if self.earthworks.is_empty() {
            return base;
        }
        self.earthworks.height(lon, lat, base, scratch)
    }
}

/// Derives the engineered ground from the solved model: one earthwork run per
/// at-grade stretch where the solved road departs the natural terrain by more
/// than [`MIN_EARTHWORK_M`], and a daylighting cut in front of every solved
/// tunnel portal (S5 — the mouth face must not hide below grade).
pub fn derive(
    scene: &SceneGraph,
    solved: &SolvedModel,
    terrain_path: Option<&Path>,
    threads: usize,
) -> GroundModel {
    let mut edges: Vec<EarthworkEdge> = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        // Earthworks run along the *smoothed* sweep line — the same curve the
        // decks are swept along and the paint snaps to — so the roadbed crest
        // stays parallel to a deck edge instead of wiggling ±1–2 m beside it
        // (at a grazing view the crest occludes the deck's lower edge, and a
        // wiggling crest reads as a jagged deck).
        let nodes = p.smooth();
        let road = p.road_m();
        let terrain = p.terrain_m();
        let at_grade = p.at_grade();
        let arcs = p.arc();
        // Carves keep the engineering width; the road bench adds the flat
        // rendering margin so the detail lattice cannot interpolate natural
        // ground up across the band edge (see EARTHWORK_MARGIN_M).
        let half_width = c.class.half_width_m(c.link) + EARTHWORK_SHOULDER_M;
        let bench_half_width = half_width + EARTHWORK_MARGIN_M;

        let needs = |i: usize| at_grade[i] && (road[i] - terrain[i]).abs() > MIN_EARTHWORK_M;
        let mut i = 0;
        while i < nodes.len() {
            if !needs(i) {
                i += 1;
                continue;
            }
            // Maximal run of earthwork nodes, padded by one at-grade node on
            // each side so the reshaping eases in at natural ground.
            let start = i;
            while i < nodes.len() && needs(i) {
                i += 1;
            }
            let lo = if start > 0 && at_grade[start - 1] { start - 1 } else { start };
            let hi = if i < nodes.len() && at_grade[i] { i } else { i - 1 };
            for k in lo..hi {
                let lift = (road[k] - terrain[k]).abs().max((road[k + 1] - terrain[k + 1]).abs());
                edges.push(EarthworkEdge {
                    a: nodes[k],
                    b: nodes[k + 1],
                    target_a: road[k],
                    target_b: road[k + 1],
                    half_width_m: bench_half_width,
                    feather_m: (EARTHWORK_BATTER * lift).max(EARTHWORK_MIN_FEATHER_M),
                    core_half_m: half_width,
                    chain: c.id,
                    arc0: arcs[k],
                    cos_lat: crate::scene::run_cos_lat(&[nodes[k], nodes[k + 1]]),
                    carve: false,
                });
            }
        }

        // Deck daylighting (the S10 mirror of the portal cut): inside a
        // mapped bridge span the deck is trusted to fly, but a DEM bump can
        // poke above it — a wooded ridge a surface DEM reads as ground — and
        // the terrain, drawn first, swallows the deck there. The bump is
        // carved to just below the deck underside. Interior bumps only: a
        // run reaching a span end is the deck legitimately meeting the
        // ground (an abutment, a portal in the hillside, S7) and stays for
        // the occlusion to work. A bump deeper than [`MAX_CLEARANCE_LIFT_M`]
        // is a data contradiction (a "bridge" through a real hill): the
        // terrain is trusted and the deck stays buried.
        for span in c.spans.iter().filter(|s| s.kind == SpanKind::Bridge) {
            let s0 = arcs.partition_point(|&a| a < span.arc0);
            let s1 = arcs.partition_point(|&a| a <= span.arc1);
            let intrudes = |i: usize| terrain[i] > road[i] - DECK_THICKNESS_M;
            let mut i = s0;
            while i < s1 {
                if !intrudes(i) {
                    i += 1;
                    continue;
                }
                let first = i;
                while i < s1 && intrudes(i) {
                    i += 1;
                }
                let last = i - 1;
                if first == s0 || last + 1 == s1 {
                    continue; // touches a span end: the deck meets the ground
                }
                let depth = (first..=last)
                    .map(|k| terrain[k] - (road[k] - DECK_THICKNESS_M))
                    .fold(0.0, f64::max);
                if depth > MAX_CLEARANCE_LIFT_M {
                    continue;
                }
                // Pad one node each side so the notch eases in below grade.
                let (lo, hi) = (first - 1, (last + 1).min(nodes.len() - 1));
                for k in lo..hi {
                    edges.push(EarthworkEdge {
                        a: nodes[k],
                        b: nodes[k + 1],
                        target_a: road[k] - DECK_THICKNESS_M - PORTAL_CLEARANCE_M,
                        target_b: road[k + 1] - DECK_THICKNESS_M - PORTAL_CLEARANCE_M,
                        half_width_m: half_width,
                        feather_m: (EARTHWORK_BATTER * depth).max(EARTHWORK_MIN_FEATHER_M),
                        core_half_m: half_width,
                        chain: c.id,
                        arc0: arcs[k],
                        cos_lat: crate::scene::run_cos_lat(&[nodes[k], nodes[k + 1]]),
                        carve: true,
                    });
                }
            }
        }

        // Portal daylighting: carve the ground down to the bore floor in a
        // short cut outward from each solved portal, so the mouth's lower
        // metres stand clear instead of hiding below grade. Cut-only — where
        // the ground has already fallen away there is nothing to remove.
        for portal in portals::portals(p, &c.spans) {
            let a = p.point_at_arc(portal.arc);
            let b = p.point_at_arc(portal.arc + portal.outward * PORTAL_CUT_LEN_M);
            if a == b {
                continue; // portal at the corridor end: no outward run
            }
            edges.push(EarthworkEdge {
                a,
                b,
                target_a: portal.floor_m,
                target_b: portal.floor_m,
                half_width_m: c.class.half_width_m(c.link) + EARTHWORK_SHOULDER_M,
                feather_m: EARTHWORK_MIN_FEATHER_M,
                core_half_m: c.class.half_width_m(c.link) + EARTHWORK_SHOULDER_M,
                chain: c.id,
                arc0: portal.arc,
                cos_lat: crate::scene::run_cos_lat(&[a, b]),
                carve: true,
            });
        }
    }
    edges.extend(derive_beds(scene, solved, terrain_path, threads));
    let waters = derive_waters(scene, solved, terrain_path, threads);
    GroundModel { earthworks: Earthworks::new(edges), waters }
}

/// Relaxation passes for [`smooth_bed_targets`] — the same alternating
/// forward/backward sweep count `limit_road_grade` uses.
const BED_GRADE_PASSES: usize = 8;

/// One street's bed before welding: the densified centerline, its cumulative
/// arc, the smoothed per-node targets, and the per-node feather reach.
struct BedProfile {
    nodes: Vec<Coord>,
    arc: Vec<f64>,
    targets: Vec<f64>,
    feathers: Vec<f64>,
    /// Held flat at target: the band half-width plus the rendering margin.
    half_width_m: f64,
    /// The band half-width — this street's own share stays 1 across it when
    /// benches overlap (`EarthworkEdge::core_half_m`).
    core_half_m: f64,
    /// The earthwork chain id (`EarthworkEdge::chain`), unique across
    /// corridors and beds; assigned by `derive_beds`.
    chain: u32,
    max_grade: f64,
    cos_lat: f64,
}

/// The bed earthworks for the unclaimed street network (D3): each street's
/// bed holds the natural ground height of its own centerline — flat across
/// the carriageway, grade-limited along it — so the two independent datasets
/// (terrain raster, road network) agree wherever a road lies. Profiles are
/// built in parallel (sampling scattered centerlines is DEM-decode bound),
/// then welded serially where beds share an endpoint connector, so no step
/// crosses a street junction.
fn derive_beds(
    scene: &SceneGraph,
    solved: &SolvedModel,
    terrain_path: Option<&Path>,
    threads: usize,
) -> Vec<EarthworkEdge> {
    let beds = &scene.beds;
    if beds.is_empty() {
        return Vec::new();
    }
    let Some(path) = terrain_path else {
        return Vec::new();
    };
    let Ok(primary) = Dem::open(path) else {
        return Vec::new(); // no DEM: nothing to bench against
    };
    let z_ref = solved.z_ref;
    let chain_base = scene.corridors.len() as u32;
    let n = beds.len();
    let threads = threads.max(1).min(n);
    let next = Mutex::new(0usize);
    let out: Mutex<Vec<Option<BedProfile>>> = Mutex::new((0..n).map(|_| None).collect());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| {
                let Ok(mut dem) = primary.fork() else { return };
                loop {
                    let i = {
                        let mut cur = next.lock().expect("bed queue poisoned");
                        if *cur >= n {
                            break;
                        }
                        let i = *cur;
                        *cur += 1;
                        i
                    };
                    let b = &beds[i];
                    // The bed holds flat to the band edge plus the rendering
                    // margin, so the detail lattice cannot pull natural
                    // hillside across the street's edge (EARTHWORK_MARGIN_M).
                    let profile = bed_profile(
                        &b.pts,
                        b.half_width_m,
                        EARTHWORK_MARGIN_M,
                        b.class.bed_grade(),
                        &mut |c| reference_surface(&mut dem, z_ref, c.x, c.y),
                    )
                    // Chain ids continue past the corridors', so a bed and a
                    // corridor can never collapse into one blending cluster.
                    .map(|mut p| {
                        p.chain = chain_base + i as u32;
                        p
                    });
                    out.lock().expect("bed profiles poisoned")[i] = profile;
                }
            });
        }
    });
    let mut profiles: Vec<BedProfile> =
        out.into_inner().expect("bed profiles poisoned").into_iter().flatten().collect();
    weld_bed_endpoints(&mut profiles, &corridor_node_heights(scene, solved));
    // Flatten in bed order, so the edge indices — and the modifier
    // tie-breaking they feed — are deterministic run to run (invariant 5).
    profiles.iter().flat_map(bed_profile_edges).collect()
}

/// One street's bed profile: the centerline subdivided to
/// [`crate::priors::BED_SPACING_M`], each node's target the natural ground at
/// the node itself, grade-limited along the street, with the feather reach
/// scaled to the cut/fill depth at the bench edge. The bench holds flat to
/// the band half-width plus `margin_m`. The sampler is injected so the shape
/// is testable without a DEM.
fn bed_profile(
    pts: &[Coord],
    band_half_m: f64,
    margin_m: f64,
    max_grade: f64,
    sample: &mut impl FnMut(Coord) -> f64,
) -> Option<BedProfile> {
    if pts.len() < 2 {
        return None;
    }
    let half_width_m = band_half_m + margin_m;
    let cos_lat = crate::scene::run_cos_lat(pts);
    // Subdivide long edges so the targets track the terrain along the road.
    let mut nodes: Vec<Coord> = vec![pts[0]];
    let mut arc: Vec<f64> = vec![0.0];
    for w in pts.windows(2) {
        let len_m = crate::scene::metric_len(w[0], w[1], cos_lat);
        let n = (len_m / crate::priors::BED_SPACING_M).ceil().max(1.0) as usize;
        for k in 1..=n {
            let t = k as f64 / n as f64;
            let c = Coord {
                x: w[0].x + (w[1].x - w[0].x) * t,
                y: w[0].y + (w[1].y - w[0].y) * t,
            };
            arc.push(arc.last().expect("non-empty arc") + crate::scene::metric_len(
                *nodes.last().expect("non-empty nodes"),
                c,
                cos_lat,
            ));
            nodes.push(c);
        }
    }
    let natural: Vec<f64> = nodes.iter().map(|&c| sample(c)).collect();
    // The bed's reference is the notch-closed profile, not the raw DEM: a
    // street annotated across a gully was engineered over it (fill and a
    // culvert), so the bed — and the deviation budget that anchors the
    // grade smoothing — carries across at rim height instead of diving in.
    let reference = crate::solve::profile::close_notches(&arc, &natural);
    let mut targets = reference.clone();
    smooth_bed_targets(&arc, &mut targets, &reference, max_grade);
    // The feather reach comes from the cut/fill depth at the *bench edge*: on
    // a cross-slope the deepest face is at ±half-width, not the centerline.
    // The batter then daylights the face at a plausible slope instead of the
    // fixed-minimum cliff a deep bench would otherwise end in.
    let feathers: Vec<f64> = nodes
        .iter()
        .enumerate()
        .map(|(i, &c)| {
            let (ux, uy) = bed_heading(&nodes, i, cos_lat);
            let (px, py) = (-uy, ux); // lateral unit, metric
            let mut side = |s: f64| {
                sample(Coord {
                    x: c.x + s * px * half_width_m / (DEG_M * cos_lat),
                    y: c.y + s * py * half_width_m / DEG_M,
                })
            };
            let t = targets[i];
            let depth = (t - side(1.0))
                .abs()
                .max((t - side(-1.0)).abs())
                .max((t - natural[i]).abs());
            (EARTHWORK_BATTER * depth).max(EARTHWORK_MIN_FEATHER_M)
        })
        .collect();
    Some(BedProfile {
        nodes,
        arc,
        targets,
        feathers,
        half_width_m,
        core_half_m: band_half_m,
        chain: 0,
        max_grade,
        cos_lat,
    })
}

/// Unit heading of the bed at node `i` in the metric (east, north) frame:
/// the direction of the segment the node starts (the last node borrows the
/// segment it ends).
fn bed_heading(nodes: &[Coord], i: usize, cos_lat: f64) -> (f64, f64) {
    let (a, b) = if i + 1 < nodes.len() { (nodes[i], nodes[i + 1]) } else { (nodes[i - 1], nodes[i]) };
    let dx = (b.x - a.x) * cos_lat;
    let dy = b.y - a.y;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-15 {
        (1.0, 0.0)
    } else {
        (dx / len, dy / len)
    }
}

/// Holds the bed to its class's grade while keeping it within
/// [`BED_MAX_DEVIATION_M`] of the natural centerline ground —
/// `limit_road_grade`'s relaxation with no pinned nodes. The deviation clamp
/// runs last, so the ground-hugging budget always holds and the grade is
/// best-effort where the street genuinely climbs faster (S9).
fn smooth_bed_targets(arc: &[f64], targets: &mut [f64], natural: &[f64], max_grade: f64) {
    let n = targets.len();
    if n < 2 {
        return;
    }
    let to_natural = |t: &mut [f64], i: usize| {
        t[i] = t[i].clamp(natural[i] - BED_MAX_DEVIATION_M, natural[i] + BED_MAX_DEVIATION_M);
    };
    let to_grade = |t: &mut [f64], i: usize, nb: usize| {
        let cap = max_grade * (arc[i] - arc[nb]).abs();
        t[i] = t[i].clamp(t[nb] - cap, t[nb] + cap);
    };
    for pass in 0..=BED_GRADE_PASSES {
        // The last pass is always forward; each node is grade-clamped then
        // pulled back inside the deviation budget, so that bound holds on exit.
        if pass % 2 == 0 || pass == BED_GRADE_PASSES {
            for i in 1..n {
                to_grade(targets, i, i - 1);
                to_natural(targets, i);
            }
        } else {
            for i in (0..n - 1).rev() {
                to_grade(targets, i, i + 1);
                to_natural(targets, i);
            }
        }
    }
}

/// A bed endpoint's exact-coordinate key. Overture splits roads at connectors
/// and every segment repeats the connector's coordinate verbatim, so meeting
/// endpoints share the same bits — no tolerance needed.
fn coord_key(c: Coord) -> (u64, u64) {
    (c.x.to_bits(), c.y.to_bits())
}

/// The solved road height at every corridor node that bounds a segment (the
/// connector coordinates a street bed can share), for welding street beds to
/// the engineered network they end on.
fn corridor_node_heights(
    scene: &SceneGraph,
    solved: &SolvedModel,
) -> HashMap<(u64, u64), f64> {
    let mut heights = HashMap::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        for seg in &c.segments {
            for &i in &[seg.node0, seg.node1] {
                let node = c.nodes[i as usize];
                heights.entry(coord_key(node)).or_insert_with(|| p.height_at(node.x, node.y));
            }
        }
    }
    heights
}

/// Junction continuity for beds (the street mirror of the corridor weld):
/// where independently smoothed beds share an endpoint they are pulled to one
/// height — the solved corridor's road height when the connector belongs to
/// the engineered network, else the mean of the meeting targets. A member
/// whose required shift exceeds [`BED_WELD_MAX_M`] keeps its own height (the
/// disagreement is a data contradiction, not a weldable seam). The correction
/// decays into the bed's interior at its own grade cap, so the weld adds no
/// kink of its own.
fn weld_bed_endpoints(beds: &mut [BedProfile], corridor_heights: &HashMap<(u64, u64), f64>) {
    // endpoint key → the (bed, side) pairs meeting there, in bed order.
    let mut groups: HashMap<(u64, u64), Vec<(usize, usize)>> = HashMap::new();
    for (i, b) in beds.iter().enumerate() {
        if b.nodes.len() < 2 {
            continue;
        }
        groups.entry(coord_key(b.nodes[0])).or_default().push((i, 0));
        groups.entry(coord_key(*b.nodes.last().expect("non-empty bed"))).or_default().push((i, 1));
    }
    let mut shifts: Vec<(usize, usize, f64)> = Vec::new();
    for (key, members) in &groups {
        let corridor = corridor_heights.get(key);
        if members.len() < 2 && corridor.is_none() {
            continue; // a dead end: nothing to reconcile
        }
        let endpoint = |&(i, side): &(usize, usize)| -> f64 {
            let b = &beds[i];
            if side == 0 { b.targets[0] } else { *b.targets.last().expect("non-empty targets") }
        };
        // The engineered network's height wins where present; otherwise the
        // meeting beds agree on their mean.
        let weld = match corridor {
            Some(&h) => h,
            None => members.iter().map(endpoint).sum::<f64>() / members.len() as f64,
        };
        for m in members {
            let delta = weld - endpoint(m);
            if delta.abs() <= BED_WELD_MAX_M {
                shifts.push((m.0, m.1, delta));
            }
        }
    }
    // Apply after collecting: every delta is computed against pre-weld
    // targets, so the outcome is independent of group iteration order.
    for (i, side, delta) in shifts {
        let b = &mut beds[i];
        let total = *b.arc.last().expect("non-empty arc");
        if total <= 0.0 {
            continue;
        }
        // Decay over the run the grade cap needs to absorb the shift, clamped
        // to the bed itself so the far endpoint stays exact.
        let len = (delta.abs() / b.max_grade).clamp(f64::MIN_POSITIVE, total);
        for k in 0..b.targets.len() {
            let d = if side == 0 { b.arc[k] } else { total - b.arc[k] };
            let w = (1.0 - d / len).max(0.0);
            b.targets[k] += delta * w;
        }
    }
}

/// A bed profile flattened to its earthwork edges. Feather is the max of the
/// edge's two node reaches, matching the per-edge lift `derive` uses.
fn bed_profile_edges(b: &BedProfile) -> Vec<EarthworkEdge> {
    b.nodes
        .windows(2)
        .zip(b.targets.windows(2))
        .zip(b.feathers.windows(2))
        .zip(b.arc.windows(2))
        .filter(|(((w, _), _), _)| w[0] != w[1])
        .map(|(((w, h), f), s)| EarthworkEdge {
            a: w[0],
            b: w[1],
            target_a: h[0],
            target_b: h[1],
            half_width_m: b.half_width_m,
            feather_m: f[0].max(f[1]),
            core_half_m: b.core_half_m,
            chain: b.chain,
            arc0: s[0],
            cos_lat: b.cos_lat,
            carve: false,
        })
        .collect()
}

/// Reads a flat surface level for every still water body from the DEM along its
/// shoreline, so the ground stage can burn it flat (invariant 4). Parallelized
/// over `threads` workers (each forks the DEM to share the decoded-tile cache),
/// since sampling scattered shorelines is DEM-decode bound. Without a DEM there
/// is nothing to read a level from; the water stays draped on the terrain.
fn derive_waters(
    scene: &SceneGraph,
    solved: &SolvedModel,
    terrain_path: Option<&Path>,
    threads: usize,
) -> Waters {
    let bodies = &scene.water;
    if bodies.is_empty() {
        return Waters::new(Vec::new());
    }
    let Some(path) = terrain_path else {
        return Waters::new(Vec::new());
    };
    let Ok(primary) = Dem::open(path) else {
        return Waters::new(Vec::new()); // no DEM: leave the water draped
    };
    let z_ref = solved.z_ref;
    let n = bodies.len();
    let threads = threads.max(1).min(n);
    let next = Mutex::new(0usize);
    let fills: Mutex<Vec<Option<WaterFill>>> = Mutex::new(vec![None; n]);
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| {
                let Ok(mut dem) = primary.fork() else { return };
                loop {
                    let i = {
                        let mut cur = next.lock().expect("water queue poisoned");
                        if *cur >= n {
                            break;
                        }
                        let i = *cur;
                        *cur += 1;
                        i
                    };
                    let w = &bodies[i];
                    if let Some(level) =
                        water_level(&w.exterior, |c| reference_surface(&mut dem, z_ref, c.x, c.y))
                    {
                        fills.lock().expect("water fills poisoned")[i] = Some(WaterFill {
                            exterior: w.exterior.clone(),
                            holes: w.holes.clone(),
                            bbox: w.bbox,
                            level,
                        });
                    }
                }
            });
        }
    });
    Waters::new(fills.into_inner().expect("water fills poisoned").into_iter().flatten().collect())
}

/// A still water body's surface level: a low percentile of the DEM sampled
/// along its shoreline. The exterior ring traces the waterline, so the ground
/// there images the water level; a low percentile ([`WATER_LEVEL_PCTL`]) leans
/// toward the water rather than a vertex that climbed the bank, so the flat
/// surface sits in its basin instead of spilling over the shore. The ring is
/// subsampled to at most [`SHORELINE_SAMPLES`] points so a huge lake stays
/// cheap. The sampler is injected so the level is testable without a DEM.
fn water_level(ring: &[Coord], mut sample: impl FnMut(Coord) -> f64) -> Option<f64> {
    if ring.len() < 3 {
        return None;
    }
    let step = (ring.len() / SHORELINE_SAMPLES).max(1);
    let mut hs: Vec<f64> = ring.iter().step_by(step).map(|&c| sample(c)).collect();
    if hs.is_empty() {
        return None;
    }
    hs.sort_by(|a, b| a.partial_cmp(b).expect("finite elevations"));
    let idx = ((hs.len() as f64 * WATER_LEVEL_PCTL) as usize).min(hs.len() - 1);
    Some(hs[idx])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::RoadClass;
    use crate::scene::{Corridor, CrossedKind, Crossing, SegmentRef, Span, SpanKind, DEG_M};
    use geo_types::Coord;

    /// A viaduct over a valley with one sharp DEM bump poking above the deck
    /// mid-span (a wooded ridge a surface DEM reads as ground): the bump is
    /// carved to below the deck underside so the deck stays visible, while
    /// the valley floor and the span ends are untouched.
    #[test]
    fn a_bump_over_a_deck_is_daylighted() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 1000.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 201;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 200.0, arc1: 800.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 800.0, arc1: 1000.0, level: 0, kind: SpanKind::Grade },
        ];
        // Plateaus at 100 m, a valley 40 m deep under the span, and a bump at
        // mid-span rising to 105 m — 5 m above the ~100 m deck line.
        let terrain = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m; // corridor metres
            let bump = (1.0 - ((x - 500.0) / 40.0).abs()).max(0.0) * 45.0;
            if !(200.0..=800.0).contains(&x) { 100.0 } else { (60.0 + bump).min(105.0) }
        };
        let scene = SceneGraph::new(vec![Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc,
            cos_lat,
            class: RoadClass::Motorway,
            link: false,
            spans: spans.clone(),
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }]);
        let profiles = vec![crate::solve::profile::solve(&nodes, &spans, Some(0.06), &mut |c| {
            terrain(c)
        })];
        let solved = crate::solve::SolvedModel::from_profiles(profiles, 14);
        let ground = derive(&scene, &solved, None, 1);

        let mut scratch = Vec::new();
        let at = |x_m: f64| Coord { x: 6.0 + deg * x_m / len_m, y: 46.0 };
        // On the bump crest the ground is cut below the ~100 m deck.
        let crest = at(500.0);
        let cut = ground.height(crest.x, crest.y, terrain(crest), &mut scratch);
        assert!(cut < 99.0, "the bump must be carved below the deck, got {cut}");
        assert!(cut > 90.0, "the notch is a daylight cut, not a canyon, got {cut}");
        // The valley floor under the deck is untouched.
        let floor = at(350.0);
        assert_eq!(ground.height(floor.x, floor.y, terrain(floor), &mut scratch), terrain(floor));
        // The at-grade approach is untouched by the carve.
        let approach = at(100.0);
        let h = ground.height(approach.x, approach.y, terrain(approach), &mut scratch);
        assert!((h - 100.0).abs() < 0.5, "approach ground stays natural, got {h}");
    }

    /// The S4 scene end-to-end through solve + derive: a flat-ground overpass
    /// leaves embankment approaches in the engineered ground.
    #[test]
    fn overpass_approaches_become_embankments() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 1000.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 41;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 450.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 450.0, arc1: 550.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 550.0, arc1: 1000.0, level: 0, kind: SpanKind::Grade },
        ];
        let mut scene = SceneGraph::new(vec![Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc,
            cos_lat,
            class: RoadClass::Secondary,
            link: false,
            spans: spans.clone(),
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }]);
        let mid = Coord { x: 6.0 + deg * 0.5, y: 46.0 };
        scene.crossings = vec![Crossing {
            upper: 0,
            upper_arc: 500.0,
            point: mid,
            lower: None,
            lower_kind: CrossedKind::Road,
            upper_level: 1,
            lower_level: 0,
        }];

        let mut profiles =
            vec![crate::solve::profile::solve(&nodes, &spans, None, &mut |_| 372.0)];
        crate::solve::crossings::apply(&scene, &mut profiles);
        let solved = crate::solve::SolvedModel::from_profiles(profiles, 14);
        let ground = derive(&scene, &solved, None, 1);
        assert!(ground.earthwork_count() > 0, "the lifted approaches must become earthworks");

        // On the approach centerline (~80 m before the crossing, 30 m before
        // the span edge) the engineered ground rises to the solved road; far
        // away it is natural.
        let mut scratch = Vec::new();
        let approach = Coord { x: mid.x - 80.0 / (DEG_M * cos_lat), y: 46.0 };
        let road_there = solved.profile(0).unwrap().height_at(approach.x, approach.y);
        let h = ground.height(approach.x, approach.y, 372.0, &mut scratch);
        assert!(
            (h - road_there).abs() < 1e-6,
            "engineered ground {h} must meet the road {road_there}"
        );
        assert!(h > 372.5, "the approach is a real embankment, got {h}");
        let far = Coord { x: 6.0 + deg * 0.02, y: 46.0 };
        assert_eq!(ground.height(far.x, far.y, 372.0, &mut scratch), 372.0);
        // Under the bridge span itself the natural ground is untouched — the
        // deck stands on air, not on a berm.
        assert_eq!(ground.height(mid.x, mid.y, 372.0, &mut scratch), 372.0);
    }

    /// A street across a side-slope: its bed holds the centerline's natural
    /// height flat across the carriageway (D3 for the unclaimed network) —
    /// the terrain and road datasets reconciled where they disagree.
    #[test]
    fn a_street_bed_is_flat_across_a_side_slope() {
        let cos_lat = 46.0_f64.to_radians().cos();
        // A 100 m west-east street on ground rising 1 m per metre northward.
        let pts = vec![
            Coord { x: 6.0, y: 46.0 },
            Coord { x: 6.0 + 100.0 / (DEG_M * cos_lat), y: 46.0 },
        ];
        let slope = |c: Coord| 400.0 + (c.y - 46.0) * DEG_M;
        let profile = bed_profile(&pts, 4.75, 0.0, RoadClass::Minor.bed_grade(), &mut |c| slope(c))
            .expect("a bed profile from a two-point street");
        let edges = bed_profile_edges(&profile);
        // 100 m at 30 m spacing → four edges, targets at the centerline height.
        assert_eq!(edges.len(), 4);
        assert!(edges.iter().all(|e| (e.target_a - 400.0).abs() < 1e-9));

        let ew = Earthworks::new(edges);
        let mut scratch = Vec::new();
        let mid_x = 6.0 + 50.0 / (DEG_M * cos_lat);
        // 3 m uphill of the centerline the natural ground is 3 m higher, but
        // the bed holds the centerline height: flat across.
        let uphill = 46.0 + 3.0 / DEG_M;
        let h = ew.height(mid_x, uphill, slope(Coord { x: mid_x, y: uphill }), &mut scratch);
        assert!((h - 400.0).abs() < 1e-9, "bed must hold flat across, got {h}");
        // The drape reads the same answer through target_at…
        assert_eq!(ew.target_at(mid_x, uphill, &mut scratch), Some(400.0));
        // …but only inside the held width; the feather is not the bed.
        let past = 46.0 + 6.0 / DEG_M;
        assert_eq!(ew.target_at(mid_x, past, &mut scratch), None);
        // Far off the street the slope is untouched.
        let far = 46.0 + 30.0 / DEG_M;
        let raw = slope(Coord { x: mid_x, y: far });
        assert_eq!(ew.height(mid_x, far, raw, &mut scratch), raw);
    }

    /// A street along rough steep ground: the bed irons the DEM's terraces to
    /// its class grade while never leaving the deviation budget — a bench that
    /// climbs plausibly instead of staircasing (S9 both ways).
    #[test]
    fn a_steep_rough_street_bed_is_grade_limited() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 600.0;
        let deg = len_m / (DEG_M * cos_lat);
        let pts =
            vec![Coord { x: 6.0, y: 46.0 }, Coord { x: 6.0 + deg, y: 46.0 }];
        // A 10 % mean climb with ±2 m terrace noise every ~60 m — steeper than
        // the minor grade cap in the noisy stretches.
        let rough = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m;
            400.0 + 0.10 * x + 2.0 * (x / 60.0 * std::f64::consts::PI).sin()
        };
        let p = bed_profile(&pts, 4.75, 0.0, RoadClass::Minor.bed_grade(), &mut |c| rough(c))
            .expect("a bed profile");
        let max_grade = RoadClass::Minor.bed_grade();
        for i in 1..p.targets.len() {
            let run = p.arc[i] - p.arc[i - 1];
            let pitch = (p.targets[i] - p.targets[i - 1]).abs() / run;
            assert!(pitch <= max_grade + 1e-9, "bed pitch {pitch} exceeds the grade cap");
        }
        // Deviation is budgeted against the notch-closed reference (the bed
        // may ride over the DEM's dips), and the bed never digs below the
        // natural ground by more than the budget.
        let natural: Vec<f64> = p.nodes.iter().map(|&c| rough(c)).collect();
        let reference = crate::solve::profile::close_notches(&p.arc, &natural);
        for i in 0..p.targets.len() {
            let dev = (p.targets[i] - reference[i]).abs();
            assert!(dev <= BED_MAX_DEVIATION_M + 1e-9, "bed leaves the reference by {dev} m");
            assert!(
                p.targets[i] >= natural[i] - BED_MAX_DEVIATION_M - 1e-9,
                "the bed must never dive below the ground budget"
            );
        }
        // The bench still climbs the hill: the ends stay ~60 m apart.
        let climb = p.targets.last().unwrap() - p.targets[0];
        assert!(climb > 50.0, "the bed must climb with its street, got {climb}");
    }

    /// A street across a narrow gully — the DEM images the stream cut, the
    /// road crosses it on fill and a culvert: the bed holds rim height across
    /// instead of diving in and out (the "road falls into holes" disease).
    #[test]
    fn a_street_spans_a_narrow_gully() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 300.0;
        let deg = len_m / (DEG_M * cos_lat);
        let pts = vec![Coord { x: 6.0, y: 46.0 }, Coord { x: 6.0 + deg, y: 46.0 }];
        // Flat ground at 500 m with a 60 m-wide, 8 m-deep V-notch mid-street
        // (wide enough that the ~27 m bed nodes sample well inside it).
        let gully = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m;
            500.0 - (1.0 - ((x - 150.0) / 30.0).abs()).max(0.0) * 8.0
        };
        let p = bed_profile(&pts, 4.75, 0.0, RoadClass::Minor.bed_grade(), &mut |c| gully(c))
            .expect("a bed profile");
        // The DEM genuinely dips at the sampled nodes…
        let dips = p.nodes.iter().any(|&c| gully(c) < 496.0);
        assert!(dips, "the fixture must sample inside the gully");
        // …but the bed carries across at rim height.
        for (i, &t) in p.targets.iter().enumerate() {
            assert!(
                (t - 500.0).abs() < 0.75,
                "the bed must span the gully at rim height, got {t} at arc {}",
                p.arc[i]
            );
        }
        // A gorge deeper than the fill cap is a genuine descent: the bed
        // keeps the terrain (no 40 m embankment wall from thin air).
        let gorge = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m;
            500.0 - (1.0 - ((x - 150.0) / 40.0).abs()).max(0.0) * 40.0
        };
        let p = bed_profile(&pts, 4.75, 0.0, RoadClass::Minor.bed_grade(), &mut |c| gorge(c))
            .expect("a bed profile");
        let deepest = p
            .targets
            .iter()
            .zip(&p.nodes)
            .map(|(&t, &c)| t - gorge(c))
            .fold(f64::NEG_INFINITY, f64::max);
        let mid = p.arc.iter().position(|&a| (a - 150.0).abs() < 16.0).expect("a mid node");
        assert!(
            p.targets[mid] < 495.0,
            "a gorge past the fill cap keeps the terrain, got {} (max lift {deepest})",
            p.targets[mid]
        );
    }

    /// A deep cut's feather scales with the bench-edge depth (the batter), and
    /// floors at the fixed minimum on flat ground.
    #[test]
    fn bed_feather_scales_with_cut_depth() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let pts = vec![
            Coord { x: 6.0, y: 46.0 },
            Coord { x: 6.0 + 100.0 / (DEG_M * cos_lat), y: 46.0 },
        ];
        // A 2 m/m side-slope: at ±4.75 m the ground is 9.5 m off the bed.
        let steep = |c: Coord| 400.0 + (c.y - 46.0) * DEG_M * 2.0;
        let p = bed_profile(&pts, 4.75, 0.0, RoadClass::Minor.bed_grade(), &mut |c| steep(c))
            .expect("a bed profile");
        let expected = EARTHWORK_BATTER * 2.0 * 4.75;
        for &f in &p.feathers {
            assert!((f - expected).abs() < 1e-6, "feather {f} must be the batter {expected}");
        }
        // Flat ground: the fixed minimum.
        let p = bed_profile(&pts, 4.75, 0.0, RoadClass::Minor.bed_grade(), &mut |_| 400.0)
            .expect("a bed profile");
        assert!(p.feathers.iter().all(|&f| f == EARTHWORK_MIN_FEATHER_M));
    }

    /// Two beds sharing an endpoint connector on disagreeing terrain weld to
    /// one height there: no step crosses the junction.
    #[test]
    fn beds_meeting_at_a_node_agree() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 200.0 / (DEG_M * cos_lat);
        let shared = Coord { x: 6.0, y: 46.0 };
        // One street arrives from the west on ground at 400 m, the other
        // leaves east on ground 2 m higher — the DEM disagrees with itself
        // across the connector at bed-node resolution.
        let west = vec![Coord { x: 6.0 - deg, y: 46.0 }, shared];
        let east = vec![shared, Coord { x: 6.0 + deg, y: 46.0 }];
        let grade = RoadClass::Minor.bed_grade();
        let mut profiles = vec![
            bed_profile(&west, 4.75, 0.0, grade, &mut |_| 400.0).expect("west bed"),
            bed_profile(&east, 4.75, 0.0, grade, &mut |_| 402.0).expect("east bed"),
        ];
        weld_bed_endpoints(&mut profiles, &HashMap::new());
        let w_end = *profiles[0].targets.last().unwrap();
        let e_start = profiles[1].targets[0];
        assert!((w_end - e_start).abs() < 1e-9, "welded endpoints must agree");
        assert!((w_end - 401.0).abs() < 1e-9, "the weld is the meeting mean, got {w_end}");
        // The far ends keep their own ground; the correction decays inside.
        assert!((profiles[0].targets[0] - 400.0).abs() < 1e-9);
        assert!((profiles[1].targets.last().unwrap() - 402.0).abs() < 1e-9);
        // No step through the engineered ground across the junction.
        let ew = Earthworks::new(
            profiles.iter().flat_map(bed_profile_edges).collect::<Vec<_>>(),
        );
        let mut scratch = Vec::new();
        let just_west = Coord { x: 6.0 - 0.5 / (DEG_M * cos_lat), y: 46.0 };
        let just_east = Coord { x: 6.0 + 0.5 / (DEG_M * cos_lat), y: 46.0 };
        let hw = ew.height(just_west.x, just_west.y, 400.0, &mut scratch);
        let he = ew.height(just_east.x, just_east.y, 402.0, &mut scratch);
        assert!((hw - he).abs() < 0.2, "junction step {:.3} m must be welded shut", (hw - he).abs());
    }

    /// A bed ending on a solved corridor welds to the corridor's road height
    /// (within the trust cap), so the street meets the highway it joins.
    #[test]
    fn a_bed_welds_to_its_corridor() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 200.0 / (DEG_M * cos_lat);
        let shared = Coord { x: 6.0, y: 46.0 };
        let street = vec![shared, Coord { x: 6.0 + deg, y: 46.0 }];
        let grade = RoadClass::Minor.bed_grade();
        let mut profiles =
            vec![bed_profile(&street, 4.75, 0.0, grade, &mut |_| 400.0).expect("a bed")];
        // The corridor's solved road arrives 1.5 m above the street's ground.
        let mut heights = HashMap::new();
        heights.insert(coord_key(shared), 401.5);
        weld_bed_endpoints(&mut profiles, &heights);
        assert!((profiles[0].targets[0] - 401.5).abs() < 1e-9, "the corridor height wins");
        // Beyond the trust cap the street keeps its own ground.
        let mut profiles =
            vec![bed_profile(&street, 4.75, 0.0, grade, &mut |_| 400.0).expect("a bed")];
        heights.insert(coord_key(shared), 410.0);
        weld_bed_endpoints(&mut profiles, &heights);
        assert!((profiles[0].targets[0] - 400.0).abs() < 1e-9, "a 10 m step is not weldable");
    }

    #[test]
    fn water_level_takes_a_low_shoreline_percentile() {
        // A shoreline that mostly images 372 m with a few bank vertices at
        // 378 m: the level must land on the waterline, not the bank.
        let ring: Vec<Coord> =
            (0..20).map(|i| Coord { x: 6.0 + i as f64 * 0.001, y: 46.0 }).collect();
        let level = water_level(&ring, |c| {
            if ((c.x * 1000.0).round() as i64) % 5 == 0 { 378.0 } else { 372.0 }
        })
        .expect("a level from a 20-vertex ring");
        assert!((level - 372.0).abs() < 1e-9, "level {level} must be the waterline, not the bank");
    }

    #[test]
    fn ground_flattens_water_and_a_shore_earthwork_overrides_it() {
        let mut scratch = Vec::new();
        let cos_lat = 46.0_f64.to_radians().cos();
        // A square lake [6.00, 6.01] × [46.00, 46.01], flattened to 372 m.
        let exterior = vec![
            Coord { x: 6.00, y: 46.00 },
            Coord { x: 6.01, y: 46.00 },
            Coord { x: 6.01, y: 46.01 },
            Coord { x: 6.00, y: 46.01 },
            Coord { x: 6.00, y: 46.00 },
        ];
        let waters = Waters::new(vec![WaterFill {
            exterior,
            holes: vec![],
            bbox: (6.00, 46.00, 6.01, 46.01),
            level: 372.0,
        }]);
        // A shore berm (a bridge approach earthwork) crossing the lake middle,
        // target 375 m — it must win over the water where they overlap.
        let earthworks = Earthworks::new(vec![EarthworkEdge {
            a: Coord { x: 6.00, y: 46.005 },
            b: Coord { x: 6.01, y: 46.005 },
            target_a: 375.0,
            target_b: 375.0,
            half_width_m: 8.0,
            feather_m: 4.0,
            core_half_m: 8.0,
            chain: 0,
            arc0: 0.0,
            cos_lat,
            carve: false,
        }]);
        let g = GroundModel { earthworks, waters };
        // Open water away from the berm: flattened to the level (over raw 360).
        assert_eq!(g.height(6.002, 46.002, 360.0, &mut scratch), 372.0);
        // On the berm centerline inside the lake: the road overrides the water.
        assert!((g.height(6.005, 46.005, 360.0, &mut scratch) - 375.0).abs() < 1e-9);
        // Outside the lake: the raw DEM passes through.
        assert_eq!(g.height(6.02, 46.02, 360.0, &mut scratch), 360.0);
    }
}

