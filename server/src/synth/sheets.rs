//! Which overlapping asphalt is one surface — the grade-separation layering
//! (docs/GROUND.md §4, docs/ROADS.md §6.1).
//!
//! The road surface is a height field: one number per (level, layer, plan
//! position). Where a road stacks over itself — a hairpin's two arms — or over
//! a neighbour it does not meet there, that model has no answer, and
//! [`crate::synth::height`]'s blend invents a ramp between the two sheets. On
//! the Montreux extract 38,630 pairs of carriageway sources overlap on one
//! (level, layer) key; ~8,100 of them are more than 0.6 m apart per metre of
//! overlap and ~3,000 more than 2, which is a wall drawn inside the drawn
//! asphalt rather than a road.
//!
//! The layer used to come from [`crate::solve::crossings::corridor_ranks`]: how
//! many *mapped* bridge spans a corridor passes over. Two things were wrong
//! with that. It is blind to the separations the data never annotated, which is
//! most of them. And it is a property of the **corridor**, while the largest
//! population by far is one corridor stacked over *itself*, which no key
//! constant along a corridor can separate.
//!
//! So the layer is derived here instead, from the solved heights — but of a
//! *run*, not of a segment:
//!
//! > A **sheet** is one contiguous stretch of a corridor's at-grade
//! > carriageway. Two sheets overlapping in plan on one level, whose surfaces
//! > *where they meet* are further apart than [`SHEET_SEPARATION_M`], are
//! > stacked, and the higher one is above. A sheet's layer is the longest such
//! > chain below it.
//!
//! This is the rule `docs/GROUND.md` §2 already settled for the ground, applied
//! to the surface: averaging inside asphalt is what let a neighbour's height
//! ramp up through a paved surface, so proximity arbitrates and a road holds
//! its own carriageway. Everything downstream is unchanged —
//! [`crate::synth::pavement`] already groups runs and buckets regions by
//! (level, layer), and [`crate::synth::height`] already filters on it — so
//! giving the layer the right value is the whole change.
//!
//! **A run is atomic, because a road is.** The layer keys the region partition,
//! and a region boundary across a carriageway is *drawn*: the two sides mesh
//! against different height fields, the rims each one and the apron walls
//! it. Layering per segment cut every road wherever its layer changed — 32,022
//! cuts inside 23,360 runs on the Montreux extract — which renders as a road
//! arriving in disconnected plates. It also made the defect it exists to remove
//! *worse*: `slope.carriageway_face` measured 7.07 at the baseline, 21.6 with
//! per-segment layers, 6.07 with per-run ones, because each cut is itself a step
//! in the drawn asphalt.
//!
//! The price, stated plainly: a corridor stacked over **itself** — a hairpin —
//! is one run, so it is one sheet, and its arms still blend. That is the
//! population this module was written for, and no partition can separate it
//! without cutting a ribbon that really is continuous through the bend.
//! Separating it needs the height field to arbitrate by *arc* rather than by
//! plan distance; a finer partition only trades a bump for a break.
//!
//! **Where the two genuinely share the asphalt, no edge is drawn.** Two roads
//! overlapping *at* an intersection they both join are one paved surface, and
//! the solver has already welded them to one height there; separating them
//! would tear the intersection in half. An overlap out along the roads, past
//! the intersection, is a stack and separates like any other.

use geo_types::Coord;

use crate::assemble::grid::GridIndex;
use crate::scene::{CorridorId, SceneGraph, DEG_M};

use super::carriageway::SourceSeg;

/// How far apart two overlapping carriageways must be, in metres, to be
/// different sheets rather than one warped surface.
///
/// Taller than anything a single surface legitimately carries across an
/// overlap — a kerb, a camber, the cross-fall where two roads meet at an angle
/// — and well under the 1–2 m that marks a genuine stack (measured p90 of the
/// disagreeing populations is 1.5 m and up). Below it the blend still runs and
/// the overlap warps, which is right: two carriageways that share their asphalt
/// must agree on its height, and a blend is how they agree.
pub const SHEET_SEPARATION_M: f64 = 0.5;

/// How near a shared intersection an overlap still counts as *being* that
/// intersection, in metres. Inside this the two roads are one paved surface
/// whatever their centerline heights say, because the junction weld has already
/// made them share a height and the plate is meshed across both.
///
/// Sized at a couple of carriageway widths — far enough to cover the mouths a
/// plate spans, short enough that a road running parallel to another for a
/// block is still measured on its own.
const AT_JUNCTION_M: f64 = 15.0;

/// The grade-separation layer of every carriageway source, in the same order.
///
/// Deterministic: candidate pairs come from [`GridIndex::query`], which returns
/// a sorted, deduplicated id list; the edge list is sorted before use; and the
/// layering walks a fixed order. The result is a function of the model, never
/// of hashing or of thread scheduling (invariant 5).
pub fn assign(scene: &SceneGraph, sources: &[SourceSeg]) -> Vec<u32> {
    let (run_of, run_count) = runs(sources);
    let verdicts = overlap_verdicts(scene, sources, &run_of);
    // Joined runs are not merely unconstrained, they are *one sheet*. Leaving
    // them as separate nodes with no edge between them lets the layering give
    // them different layers anyway — measured at 6.9150,46.4312, three
    // consecutive stretches of one street, each handing its exact height to the
    // next, came back on layers 2, 0 and 1 because each was lifted by something
    // different further along.
    let (sheet_of, sheet_count) = merge_joined(run_count, &verdicts);
    let above = stacking_edges(&verdicts, &sheet_of);
    let (sheet_layer, _) = layer_of(sheet_count, above);
    run_of.iter().map(|&r| sheet_layer[sheet_of[r as usize] as usize]).collect()
}

/// Union of the runs that share their asphalt, as a dense sheet id per run.
///
/// A sheet is a connected component of joined asphalt: everything you can drive
/// across at one surface. That is what the region partition should key on, and
/// what the height field should blend within.
fn merge_joined(
    run_count: usize,
    verdicts: &std::collections::BTreeMap<(u32, u32), Verdict>,
) -> (Vec<u32>, usize) {
    let mut parent: Vec<u32> = (0..run_count as u32).collect();
    fn find(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            parent[x as usize] = parent[parent[x as usize] as usize]; // halve the path
            x = parent[x as usize];
        }
        x
    }
    for (&(a, b), v) in verdicts {
        if !v.joined {
            continue;
        }
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        if ra != rb {
            // Union toward the lower id, so the result depends on the model
            // rather than on iteration order (invariant 5).
            let (lo, hi) = (ra.min(rb), ra.max(rb));
            parent[hi as usize] = lo;
        }
    }
    // Renumber the roots densely, in id order.
    let mut sheet_of = vec![0u32; run_count];
    let mut dense: Vec<u32> = vec![u32::MAX; run_count];
    let mut next = 0u32;
    for r in 0..run_count as u32 {
        let root = find(&mut parent, r);
        if dense[root as usize] == u32::MAX {
            dense[root as usize] = next;
            next += 1;
        }
        sheet_of[r as usize] = dense[root as usize];
    }
    (sheet_of, next as usize)
}

/// The run each source belongs to, and how many runs there are.
///
/// A run is a maximal chain of sources of one corridor joined end to start.
/// [`crate::synth::carriageway::carriageway_sources`] emits them in corridor then
/// node order, so a chain is a contiguous span of the slice; a corridor whose
/// at-grade asphalt is interrupted — by a bridge span or a bore — comes back as
/// two runs, which is right, since two stretches with a structure between them
/// are two sheets and may well be stacked.
fn runs(sources: &[SourceSeg]) -> (Vec<u32>, usize) {
    let mut run_of = Vec::with_capacity(sources.len());
    let mut next = 0u32;
    for (i, s) in sources.iter().enumerate() {
        let joined = i > 0 && {
            let p = &sources[i - 1];
            p.corridor == s.corridor && p.b == s.a
        };
        if !joined {
            next += 1;
        }
        run_of.push(next - 1);
    }
    (run_of, next as usize)
}

/// What every overlapping pair of runs says about each other.
#[derive(Default, Clone, Copy)]
struct Verdict {
    /// Overlapped somewhere at a shared height: the two are one surface.
    joined: bool,
    /// A disagreeing overlap with the lower run first in the key, and one with
    /// it second. Both set means the pair crosses each other twice, once each
    /// way, and cannot be ordered.
    lower_first: bool,
    upper_first: bool,
}

/// What every overlapping pair of runs says about each other, keyed on the
/// ordered pair so the verdict is one entry per pair however many stretches
/// meet. A `BTreeMap` rather than a hash: the result must be a function of the
/// model (invariant 5).
///
/// Two runs stack when some source of one overlaps a source of the other in plan
/// on one level and their solved surfaces *where they meet* are more than
/// [`SHEET_SEPARATION_M`] apart. But one disagreeing overlap is not enough:
///
/// > **Runs that agree anywhere are one surface everywhere.** If two runs
/// > overlap somewhere at a shared height — they meet at an intersection, they
/// > share a mouth, their asphalt runs together for a stretch — then they are
/// > joined asphalt, and separating them would put a region boundary through the
/// > place they join. A disagreement elsewhere is a bump for the height field to
/// > answer, not a partition boundary.
///
/// Without that rule a lift travels the whole length of a run and shatters
/// junctions far from the stack that caused it: measured at Chemin de la
/// Rapille, five corridors meeting within 0.7 m of one height came back on four
/// different layers, so the plate could merge with one of them and the rest
/// arrived as separate slabs. What is left to separate is asphalt that overlaps
/// and never joins — a road passing over an unrelated one — which is exactly the
/// population that carries the steepest invented ramps.
///
/// A run never stacks on itself: a hairpin's arms are one ribbon of asphalt and
/// the module doc says why cutting it is worse than the bump.
fn overlap_verdicts(
    scene: &SceneGraph,
    sources: &[SourceSeg],
    run_of: &[u32],
) -> std::collections::BTreeMap<(u32, u32), Verdict> {
    let mut grid = GridIndex::new();
    for (i, s) in sources.iter().enumerate() {
        grid.insert(bbox_of(s), i as u32);
    }
    let ports = JunctionPorts::build(scene);
    let mut seen: std::collections::BTreeMap<(u32, u32), Verdict> = Default::default();

    let mut cand: Vec<u32> = Vec::new();
    for (i, s) in sources.iter().enumerate() {
        grid.query(bbox_of(s), &mut cand);
        for &j in cand.iter() {
            if j as usize <= i {
                continue; // one direction per pair; a source never stacks on itself
            }
            let t = &sources[j as usize];
            let (ri, rj) = (run_of[i], run_of[j as usize]);
            if ri == rj {
                continue; // one ribbon of asphalt, whatever it does in plan
            }
            if s.level != t.level {
                continue; // already separate regions
            }
            // Where the two bands meet — the only place their heights are
            // comparable. Measuring each stretch at its own midpoint instead
            // reads a *climb* as a stack: on a 10 % grade 20 m of road is 2 m of
            // rise, so every steep side street read as stacked on the road it
            // joins.
            let (d, ts, tt) = closest_approach(s, t);
            if d > s.half_m + t.half_m {
                continue; // the bands never meet
            }
            let key = (ri.min(rj), ri.max(rj));
            let v = seen.entry(key).or_default();
            let gap = s.height_at(ts) - t.height_at(tt);
            // Sharing a height here, or meeting at an intersection whose weld
            // has already made them share one, is the two runs being joined
            // asphalt — which outranks any disagreement found elsewhere.
            if gap.abs() <= SHEET_SEPARATION_M
                || ports.share_intersection_near(s.corridor, t.corridor, along(s, ts))
            {
                v.joined = true;
                continue;
            }
            let lower = if gap < 0.0 { ri } else { rj };
            if lower == key.0 {
                v.lower_first = true;
            } else {
                v.upper_first = true;
            }
        }
    }

    seen
}

/// Every `(lower, upper)` pair of *sheets* that stack, sorted and deduplicated.
///
/// Read off the run verdicts once the joined runs have been merged: a
/// disagreement inside one sheet is a bump for the height field to answer, and
/// only a disagreement between two sheets that never join is a partition
/// boundary.
fn stacking_edges(
    verdicts: &std::collections::BTreeMap<(u32, u32), Verdict>,
    sheet_of: &[u32],
) -> Vec<(u32, u32)> {
    let mut out: Vec<(u32, u32)> = Vec::new();
    for (&(a, b), v) in verdicts {
        // Unable to say which is on top — two runs crossing each other twice,
        // once each way — is not an order the partition may invent.
        if v.joined || v.lower_first == v.upper_first {
            continue;
        }
        let (lower, upper) = if v.lower_first { (a, b) } else { (b, a) };
        let (ls, us) = (sheet_of[lower as usize], sheet_of[upper as usize]);
        if ls != us {
            out.push((ls, us)); // same sheet: joined by some other overlap
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// Longest-path layering over the `(lower, upper)` edges: a source sits
/// strictly above every source it covers, so the layer separates the sheets in
/// stacking order.
///
/// Contradictory input can close a cycle — A over B over C over A, which two
/// profiles crossing twice can produce. It is broken at its lowest-numbered
/// still-blocked edge, deterministically, so a bad datum costs one separation
/// rather than hanging the bake (invariant 6). The same shape as
/// [`crate::solve::crossings::corridor_ranks`], which layers the *mapped*
/// crossings for the clearance solver.
///
/// Returns the layering and the edge set it was computed over — the input minus
/// whatever had to be dropped to break a cycle, so [`separate`] can rely on it
/// being acyclic.
fn layer_of(n: usize, edges: Vec<(u32, u32)>) -> (Vec<u32>, Vec<(u32, u32)>) {
    let mut adj: Vec<Vec<u32>> = vec![Vec::new(); n];
    let mut indeg = vec![0u32; n];
    for &(lo, up) in &edges {
        adj[lo as usize].push(up);
        indeg[up as usize] += 1;
    }
    let mut layer = vec![0u32; n];
    let mut queue: Vec<u32> = (0..n as u32).filter(|&v| indeg[v as usize] == 0).collect();
    let mut done = 0usize;
    let mut blocked: Vec<(u32, u32)> = edges;
    while done < n {
        while let Some(u) = queue.pop() {
            done += 1;
            for k in 0..adj[u as usize].len() {
                let v = adj[u as usize][k];
                layer[v as usize] = layer[v as usize].max(layer[u as usize] + 1);
                indeg[v as usize] -= 1;
                if indeg[v as usize] == 0 {
                    queue.push(v);
                }
            }
        }
        if done >= n {
            break;
        }
        // Stalled on a cycle: drop the first edge into a still-blocked source.
        let Some(pos) = blocked.iter().position(|&(_, up)| indeg[up as usize] > 0) else {
            break; // nothing left to break (defensive)
        };
        let (lo, up) = blocked.swap_remove(pos);
        indeg[up as usize] -= 1;
        if let Some(k) = adj[lo as usize].iter().position(|&v| v == up) {
            adj[lo as usize].swap_remove(k);
        }
        if indeg[up as usize] == 0 {
            queue.push(up);
        }
    }
    // `blocked` started as every edge and lost only the ones broken above, so
    // what remains is the acyclic set the layering actually honoured.
    (layer, blocked)
}

/// Which intersections each corridor joins, for the shared-intersection test.
struct JunctionPorts {
    /// Junction indices per corridor, sorted — short lists, walked as sets.
    by_corridor: std::collections::HashMap<CorridorId, Vec<u32>>,
    points: Vec<Coord>,
}

impl JunctionPorts {
    fn build(scene: &SceneGraph) -> JunctionPorts {
        let mut by_corridor: std::collections::HashMap<CorridorId, Vec<u32>> = Default::default();
        for (i, j) in scene.junctions.iter().enumerate() {
            for m in &j.members {
                by_corridor.entry(m.corridor).or_default().push(i as u32);
            }
        }
        for v in by_corridor.values_mut() {
            v.sort_unstable();
            v.dedup();
        }
        JunctionPorts { by_corridor, points: scene.junctions.iter().map(|j| j.point).collect() }
    }

    /// Whether the two corridors meet at an intersection within
    /// [`AT_JUNCTION_M`] of `at` — the overlap is that intersection's own paved
    /// area, not a stack.
    fn share_intersection_near(&self, a: CorridorId, b: CorridorId, at: Coord) -> bool {
        if a == b {
            // One corridor overlapping itself is never an intersection with
            // itself: a hairpin's arms meet only by going round the bend.
            return false;
        }
        let (Some(ja), Some(jb)) = (self.by_corridor.get(&a), self.by_corridor.get(&b)) else {
            return false;
        };
        let cos_lat = at.y.to_radians().cos();
        // Both lists are sorted; walk them together for the shared ids.
        let (mut i, mut k) = (0usize, 0usize);
        while i < ja.len() && k < jb.len() {
            match ja[i].cmp(&jb[k]) {
                std::cmp::Ordering::Less => i += 1,
                std::cmp::Ordering::Greater => k += 1,
                std::cmp::Ordering::Equal => {
                    let p = self.points[ja[i] as usize];
                    let (dx, dy) =
                        ((p.x - at.x) * DEG_M * cos_lat, (p.y - at.y) * DEG_M);
                    if dx * dx + dy * dy <= AT_JUNCTION_M * AT_JUNCTION_M {
                        return true;
                    }
                    i += 1;
                    k += 1;
                }
            }
        }
        false
    }
}

/// A source's buffered bounding box, in degrees.
fn bbox_of(s: &SourceSeg) -> (f64, f64, f64, f64) {
    let pad = s.half_m / DEG_M;
    (
        s.a.x.min(s.b.x) - pad,
        s.a.y.min(s.b.y) - pad,
        s.a.x.max(s.b.x) + pad,
        s.a.y.max(s.b.y) + pad,
    )
}

/// The point at parameter `t` along a source's centerline.
fn along(s: &SourceSeg, t: f64) -> Coord {
    Coord { x: s.a.x + (s.b.x - s.a.x) * t, y: s.a.y + (s.b.y - s.a.y) * t }
}

/// Where two centerlines come nearest: the distance in metres, and the
/// parameter along each segment at which it is reached.
///
/// The parameters are what makes a climbing road one sheet — each stretch is
/// read at the place the other one actually touches it, where a continuous
/// carriageway agrees with itself by construction.
///
/// Minimum distance between segments is attained at an endpoint of one of them
/// unless they cross, so the four endpoint projections are exact, and crossing
/// is handled first.
fn closest_approach(s: &SourceSeg, t: &SourceSeg) -> (f64, f64, f64) {
    if let Some((ts, tt)) = crossing_params(s, t) {
        return (0.0, ts, tt);
    }
    // Each candidate: (distance, parameter on s, parameter on t).
    let (d1, u1) = point_to_segment(s.a, t.a, t.b, s.cos_lat);
    let (d2, u2) = point_to_segment(s.b, t.a, t.b, s.cos_lat);
    let (d3, u3) = point_to_segment(t.a, s.a, s.b, s.cos_lat);
    let (d4, u4) = point_to_segment(t.b, s.a, s.b, s.cos_lat);
    let mut best = (d1, 0.0, u1);
    for c in [(d2, 1.0, u2), (d3, u3, 0.0), (d4, u4, 1.0)] {
        if c.0 < best.0 {
            best = c;
        }
    }
    best
}

/// The parameters at which two properly crossing centerlines meet, or `None`
/// when they do not cross. The endpoint distances all stay positive across a
/// crossing, so without this an overpass whose centerline cuts the road beneath
/// it at a right angle measures as far apart.
fn crossing_params(s: &SourceSeg, t: &SourceSeg) -> Option<(f64, f64)> {
    let m_lon = DEG_M * s.cos_lat;
    let (rx, ry) = ((s.b.x - s.a.x) * m_lon, (s.b.y - s.a.y) * DEG_M);
    let (vx, vy) = ((t.b.x - t.a.x) * m_lon, (t.b.y - t.a.y) * DEG_M);
    let (qx, qy) = ((t.a.x - s.a.x) * m_lon, (t.a.y - s.a.y) * DEG_M);
    let denom = rx * vy - ry * vx;
    if denom.abs() < 1e-12 {
        return None; // parallel: the endpoint projections have it
    }
    let ts = (qx * vy - qy * vx) / denom;
    let tt = (qx * ry - qy * rx) / denom;
    ((0.0..=1.0).contains(&ts) && (0.0..=1.0).contains(&tt)).then_some((ts, tt))
}

/// The distance in metres from `p` to segment `a`→`b`, and the parameter along
/// the segment where the foot falls.
pub(crate) fn point_to_segment(p: Coord, a: Coord, b: Coord, cos_lat: f64) -> (f64, f64) {
    let m_lon = DEG_M * cos_lat;
    let (px, py) = ((p.x - a.x) * m_lon, (p.y - a.y) * DEG_M);
    let (ex, ey) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let len2 = ex * ex + ey * ey;
    if len2 < 1e-18 {
        return ((px * px + py * py).sqrt(), 0.0);
    }
    let t = ((px * ex + py * ey) / len2).clamp(0.0, 1.0);
    let (dx, dy) = (px - ex * t, py - ey * t);
    ((dx * dx + dy * dy).sqrt(), t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{Junction, JunctionMember};
    use crate::assemble::facades::Section;

    const LAT: f64 = 46.0;

    fn m_lon() -> f64 {
        DEG_M * LAT.to_radians().cos()
    }

    /// An east–west source `off_m` metres north of `LAT`, from `x0_m` to `x1_m`
    /// east of lon 6, level at height `h`.
    fn ew(corridor: CorridorId, off_m: f64, x0_m: f64, x1_m: f64, h: f64) -> SourceSeg {
        ew_grade(corridor, off_m, x0_m, x1_m, h, h)
    }

    /// The same, climbing from `h0` at its west end to `h1` at its east end.
    fn ew_grade(
        corridor: CorridorId,
        off_m: f64,
        x0_m: f64,
        x1_m: f64,
        h0: f64,
        h1: f64,
    ) -> SourceSeg {
        SourceSeg {
            a: Coord { x: 6.0 + x0_m / m_lon(), y: LAT + off_m / DEG_M },
            b: Coord { x: 6.0 + x1_m / m_lon(), y: LAT + off_m / DEG_M },
            cos_lat: LAT.to_radians().cos(),
            half_m: 3.0,
            sect_a: Section::uniform(3.0),
            sect_b: Section::uniform(3.0),
            cut_a: None,
            cut_b: None,
            level: 0,
            layer: 0,
            height_a: h0,
            height_b: h1,
            corridor,
            surface: crate::priors::Surface::Asphalt,
            rise_m: 0.0,
            arc0: 0.0,
        }
    }

    /// A north–south source crossing `LAT` at `x_m` east of lon 6.
    fn ns(corridor: CorridorId, x_m: f64, y0_m: f64, y1_m: f64, h: f64) -> SourceSeg {
        SourceSeg {
            a: Coord { x: 6.0 + x_m / m_lon(), y: LAT + y0_m / DEG_M },
            b: Coord { x: 6.0 + x_m / m_lon(), y: LAT + y1_m / DEG_M },
            cos_lat: LAT.to_radians().cos(),
            half_m: 3.0,
            sect_a: Section::uniform(3.0),
            sect_b: Section::uniform(3.0),
            cut_a: None,
            cut_b: None,
            level: 0,
            layer: 0,
            height_a: h,
            height_b: h,
            corridor,
            surface: crate::priors::Surface::Asphalt,
            rise_m: 0.0,
            arc0: 0.0,
        }
    }

    fn bare_scene() -> SceneGraph {
        SceneGraph::new(Vec::new())
    }

    /// Two roads crossing in plan four metres apart vertically are two sheets,
    /// and the higher one is above — whatever the data says about bridges.
    #[test]
    fn a_plan_crossing_at_two_heights_separates() {
        let sources = vec![ew(0, 0.0, -20.0, 20.0, 400.0), ns(1, 0.0, -20.0, 20.0, 404.0)];
        let layers = assign(&bare_scene(), &sources);
        assert_eq!(layers, vec![0, 1], "the higher road must outrank the lower");
    }

    /// The same crossing within half a metre is one surface: the blend is what
    /// makes two carriageways sharing their asphalt agree on its height.
    #[test]
    fn a_plan_crossing_at_one_height_stays_one_sheet() {
        let sources = vec![ew(0, 0.0, -20.0, 20.0, 400.0), ns(1, 0.0, -20.0, 20.0, 400.3)];
        assert_eq!(assign(&bare_scene(), &sources), vec![0, 0]);
    }

    /// Bands that never meet are never separated, however far apart in height:
    /// a road on a hill above another road is not stacked over it.
    #[test]
    fn roads_whose_bands_never_meet_share_a_sheet() {
        let sources = vec![ew(0, 0.0, -20.0, 20.0, 400.0), ew(1, 40.0, -20.0, 20.0, 430.0)];
        assert_eq!(assign(&bare_scene(), &sources), vec![0, 0]);
    }

    /// A road on a grade is one continuous carriageway, however much it climbs.
    /// Consecutive stretches of one corridor always overlap — they share a node
    /// — and on any real slope their heights differ, so comparing them anywhere
    /// but at the place they touch reads a hill as a stack and cuts the road
    /// into one sheet per segment.
    #[test]
    fn a_climbing_road_is_one_sheet() {
        // 20 m stretches at 10%: 2 m of rise apiece, four times the separation.
        let sources: Vec<SourceSeg> = (0..6)
            .map(|k| {
                let (x0, x1) = (k as f64 * 20.0, (k + 1) as f64 * 20.0);
                ew_grade(0, 0.0, x0, x1, 400.0 + 0.1 * x0, 400.0 + 0.1 * x1)
            })
            .collect();
        assert_eq!(assign(&bare_scene(), &sources), vec![0; 6], "the climb is not a stack");
    }

    /// A hairpin is one ribbon and stays one sheet, and this is a deliberate
    /// choice rather than an oversight: cutting the run where its arms pass is
    /// the only way to separate them, and that cut is drawn straight across the
    /// carriageway. The arms blend; see the module doc for what would actually
    /// fix it.
    #[test]
    fn a_hairpin_stays_one_sheet() {
        // Out along the bench, round the bend, back above itself — joined end to
        // start throughout, so it is one run.
        let sources = vec![
            ew_grade(0, 0.0, -20.0, 20.0, 398.0, 400.0),
            SourceSeg { corridor: 0, ..ew_grade(0, 2.0, 20.0, 20.0, 400.0, 403.0) },
            ew_grade(0, 4.0, 20.0, -20.0, 403.0, 405.0),
        ];
        // The bend segment shares its ends with its neighbours.
        let mut run = sources;
        run[1].a = run[0].b;
        run[1].b = run[2].a;
        assert_eq!(assign(&bare_scene(), &run), vec![0, 0, 0]);
    }

    /// Two runs of *one* corridor — its at-grade asphalt interrupted by a bridge
    /// span or a bore — are two sheets and do separate. A structure between them
    /// is exactly the break the run boundary records.
    #[test]
    fn two_runs_of_one_corridor_separate() {
        let sources = vec![ew(0, 0.0, -20.0, 20.0, 400.0), ew(0, 4.0, -20.0, 20.0, 405.0)];
        assert_eq!(assign(&bare_scene(), &sources), vec![0, 1], "the upper run outranks");
    }

    /// Two roads meeting at an intersection are one paved surface there, even
    /// where their centerline heights differ: the weld already made them share
    /// a height and the plate is meshed across both. Separating them would tear
    /// the intersection in half.
    #[test]
    fn an_overlap_at_a_shared_intersection_is_not_a_stack() {
        let sources = vec![ew(0, 0.0, -20.0, 0.0, 400.0), ns(1, 0.0, 0.0, 20.0, 402.0)];
        let mut scene = SceneGraph::new(Vec::new());
        scene.junctions = vec![Junction {
            point: Coord { x: 6.0, y: LAT },
            connector: 0,
            members: vec![
                JunctionMember { corridor: 0, arc: 20.0 },
                JunctionMember { corridor: 1, arc: 0.0 },
            ],
        }];
        assert_eq!(assign(&scene, &sources), vec![0, 0]);

        // The same pair overlapping a hundred metres away is a stack again:
        // they meet somewhere, but not here.
        let far = vec![ew(0, 0.0, 80.0, 120.0, 400.0), ew(1, 4.0, 80.0, 120.0, 402.0)];
        assert_eq!(assign(&scene, &far), vec![0, 1]);
    }

    /// **The junction defect this rule exists to prevent.** Two roads that share
    /// their asphalt somewhere are one surface everywhere, even where one of
    /// them climbs metres above the other further along. Separating them puts a
    /// region boundary through the place they join: measured at Chemin de la
    /// Rapille, five corridors meeting within 0.7 m of one height came back on
    /// four layers, and the plate could merge with only one of them.
    #[test]
    fn runs_that_join_anywhere_are_one_sheet_everywhere() {
        // Two roads side by side. At the west end they share a height — this is
        // where they meet — and at the east end one has climbed 4 m above.
        let sources = vec![
            ew_grade(0, 0.0, 0.0, 60.0, 400.0, 400.0),
            ew_grade(1, 4.0, 0.0, 60.0, 400.2, 404.0),
        ];
        assert_eq!(assign(&bare_scene(), &sources), vec![0, 0], "the join outranks the climb");

        // Split the climbing road so the stretch that agrees is a separate run,
        // and the stretch that never agrees separates as it should.
        let split = vec![
            ew(0, 0.0, 0.0, 60.0, 400.0),
            ew_grade(1, 4.0, 30.0, 60.0, 404.0, 404.0),
        ];
        assert_eq!(assign(&bare_scene(), &split), vec![0, 1], "asphalt that never joins stacks");
    }

    /// **A join makes one sheet, not merely one fewer constraint.** One street
    /// mapped as consecutive corridors hands its exact height from each stretch
    /// to the next, so every pair is joined — but if a joined pair is only left
    /// unconstrained, a lift picked up by one stretch further along still puts
    /// it on a different layer from its own continuation. Measured at
    /// 6.9150,46.4312: three stretches of one street, meeting at 403.91 and at
    /// 402.95, came back on layers 2, 0 and 1, and the junction they share was
    /// drawn as three overlapping slabs.
    #[test]
    fn a_street_split_into_corridors_is_one_sheet() {
        let sources = vec![
            // Three stretches end to end, each starting where the last ended.
            ew_grade(0, 0.0, 0.0, 30.0, 405.4, 404.0),
            ew_grade(1, 0.0, 30.0, 60.0, 404.0, 403.0),
            ew_grade(2, 0.0, 60.0, 90.0, 403.0, 402.0),
            // Something the *first* stretch passes over, and nothing else does.
            ns(3, 15.0, -20.0, 20.0, 400.0),
        ];
        let layers = assign(&bare_scene(), &sources);
        assert_eq!(layers[3], 0, "the road underneath stays down");
        assert!(
            layers[0] == layers[1] && layers[1] == layers[2],
            "one street came back on layers {:?}",
            &layers[0..3]
        );
        assert!(layers[0] > layers[3], "the street still outranks what it covers");
    }

    /// Three sheets stack into three layers, so a road under a road under a
    /// road keeps all three apart rather than collapsing two of them.
    #[test]
    fn stacked_sheets_layer_in_order() {
        let sources = vec![
            ew(0, 0.0, -20.0, 20.0, 400.0),
            ew(1, 2.0, -20.0, 20.0, 405.0),
            ew(2, 4.0, -20.0, 20.0, 410.0),
        ];
        assert_eq!(assign(&bare_scene(), &sources), vec![0, 1, 2]);
    }

    /// Different levels are already separate regions, so no edge is drawn
    /// between them and neither is pushed up a layer it does not need.
    #[test]
    fn different_levels_are_left_alone() {
        let mut upper = ew(1, 0.0, -20.0, 20.0, 410.0);
        upper.level = 1;
        let sources = vec![ew(0, 0.0, -20.0, 20.0, 400.0), upper];
        assert_eq!(assign(&bare_scene(), &sources), vec![0, 0]);
    }

    /// A contradictory cycle costs one separation and still terminates: every
    /// source comes back with a layer (invariant 6).
    #[test]
    fn a_cycle_is_broken_rather_than_hanging() {
        // A over B, B over C, C over A — three edges closing a loop.
        let (layers, acyclic) = layer_of(3, vec![(0, 1), (1, 2), (2, 0)]);
        assert_eq!(layers.len(), 3);
        assert!(layers.iter().all(|&l| l < 3), "layers {layers:?} did not settle");
        assert!(acyclic.len() < 3, "the cycle must have cost an edge");
    }

    /// **The regression this rule exists to prevent.** A flyover crossing a
    /// street at a shallow angle overlaps it in patches, and lifting only the
    /// patches gave the flyover the layers 0,1,0,1,0 — a region boundary drawn
    /// across the carriageway four times, which renders as a road in pieces. The
    /// whole run carries one layer, so a road is never cut along its length.
    #[test]
    fn a_lift_covers_the_whole_run_so_a_road_is_never_cut() {
        // The road above, seven stretches joined end to start, and a street on
        // the ground crossing it — which only the middle stretches overlap.
        let mut sources: Vec<SourceSeg> = (0..7)
            .map(|k| ew(0, 0.0, k as f64 * 10.0, (k + 1) as f64 * 10.0, 404.0))
            .collect();
        for k in 1..7 {
            sources[k].a = sources[k - 1].b;
        }
        sources.push(ns(1, 35.0, -20.0, 20.0, 400.0));

        let layers = assign(&bare_scene(), &sources);
        assert_eq!(&layers[0..7], &[1; 7], "layers {layers:?} cut the run");
        assert_eq!(layers[7], 0, "the ground street stays down");
    }

    /// One layer per run, so every source of a run answers the same — the
    /// property the region partition needs, since it keys on the layer.
    #[test]
    fn a_run_is_one_contiguous_span_of_the_slice() {
        let mut sources = vec![
            ew(0, 0.0, 0.0, 10.0, 400.0),
            ew(0, 0.0, 10.0, 20.0, 400.0),
            ew(1, 40.0, 0.0, 10.0, 400.0), // a different corridor: a new run
            ew(0, 0.0, 60.0, 70.0, 400.0), // corridor 0 again, but detached
        ];
        sources[1].a = sources[0].b;
        let (run_of, count) = runs(&sources);
        assert_eq!(run_of, vec![0, 0, 1, 2]);
        assert_eq!(count, 3);
    }

    /// Crossing centerlines measure as touching, at the parameters where they
    /// meet. Without the crossing case the four endpoint distances all stay
    /// positive and an overpass cutting the road beneath it at a right angle
    /// reads as far away.
    #[test]
    fn crossing_centerlines_meet_at_the_crossing() {
        let s = ew(0, 0.0, -20.0, 20.0, 400.0);
        let t = ns(1, 0.0, -20.0, 20.0, 404.0);
        let (d, ts, tt) = closest_approach(&s, &t);
        assert_eq!(d, 0.0);
        assert!((ts - 0.5).abs() < 1e-6 && (tt - 0.5).abs() < 1e-6, "{ts} {tt}");
        // Parallel ones keep their true separation.
        let u = ew(1, 8.0, -20.0, 20.0, 404.0);
        assert!((closest_approach(&s, &u).0 - 8.0).abs() < 0.05);
    }

    /// Two stretches meeting end to end are compared *at the shared node*, where
    /// a continuous carriageway agrees with itself — not at their midpoints,
    /// which on a grade are a segment length apart in height.
    #[test]
    fn stretches_meeting_end_to_end_touch_at_their_shared_node() {
        let up = ew(0, 0.0, 0.0, 20.0, 400.0);
        let next = ew(0, 0.0, 20.0, 40.0, 402.0);
        let (d, ts, tt) = closest_approach(&up, &next);
        assert_eq!(d, 0.0);
        assert!((ts - 1.0).abs() < 1e-9 && tt.abs() < 1e-9, "{ts} {tt}");
    }
}
