//! Bench contact lines — the breaklines the detail terrain mesh preserves
//! (docs/GROUND.md §3).
//!
//! Every earthwork bench implies one contact polyline per side: the *crest*
//! at the bench edge, where the ground stops being the road and becomes the
//! batter face (or a retaining wall down to whatever bench abuts it). A
//! regular lattice cannot hold a bench narrower than its cells; a
//! triangulation constrained by these lines holds it exactly, whatever the
//! cell size — and holds the wall as a face rather than smearing it across a
//! cell.
//!
//! Breaklines are derived from the earthwork edges themselves — the same data
//! the ground function reads — so the lines and the field they sample can
//! never disagree. Vertices carry position only; every mesh vertex evaluates
//! the one ground function at mesh time (the z rule).

use geo_types::Coord;

use crate::assemble::grid::GridIndex;
use crate::scene::DEG_M;

use super::modifiers::{EarthworkEdge, Earthworks};

/// Sharpest miter kept when offsetting at a polyline joint, as a scale
/// factor on the lateral offset. Joints sharper than this (hairpin apexes)
/// clamp to it — the offset line may then cross its own other side, which
/// the mesh builder's constraint pre-split resolves.
const MITER_MAX: f64 = 2.0;

/// How far *inside* the bench edge the crest line is drawn, in metres.
///
/// The bench edge is where the ground function steps, and a vertex placed
/// exactly on a step reads whichever side it rounds to: tile coordinates
/// quantize to about a centimetre, and the triangulation rounds its own split
/// vertices too, so a line drawn on the edge samples the road one moment and
/// the hillside the next and the mesh comes out as a row of teeth. Drawing the
/// line a hand's breadth inside puts every vertex on it unambiguously on the
/// bench, and the step then falls between the crest and the first lattice
/// point outside it. Far larger than the quantum, far smaller than the verge.
const CREST_INSET_M: f64 = 0.25;

/// How far apart two bench targets must be for one to count as *another*
/// bench when a crest node asks who holds the ground under it. Below this the
/// two benches are the same surface to within the millimetre the tile
/// quantizes to, and there is no step for the mesh to hold.
const CREST_STEP_M: f64 = 0.05;

/// Steps taken inward from the nominal offset looking for the outermost place
/// the bench still holds the ground, and the bisection steps that then refine
/// that interval. Eight steps over a bench half-width resolve a contending
/// neighbour down to well under a metre; four halvings of one step put the
/// crest within centimetres of the boundary — finer than [`CREST_INSET_M`], and
/// far finer than a lattice cell.
const CREST_SCAN_STEPS: u32 = 8;
const CREST_BISECT_STEPS: u32 = 4;

/// Longest a *contended* crest segment may run before it is subdivided, in
/// metres.
///
/// A crest is sampled at the earthwork's own nodes, which for a street are a
/// full class node-spacing apart — tens of metres. Uncontended that is exact:
/// the line is parallel to a straight run of road and a chord between two
/// samples lies on it. But where a neighbour crowds the bench the offset is
/// pulled in by a different amount at each end, and the chord between them cuts
/// *inside* both — under the carriageway's own paint, if the pull-in is deeper
/// than the verge. The mesh then holds the bench only inside that chord and
/// ramps the neighbour's wall over the strip of asphalt outside it, which on a
/// street beside a railway seven metres above it is a wall drawn across the
/// kerb. Subdividing to about a lattice cell tracks the boundary instead of
/// chording it. Only contended segments pay: an uncontended run keeps its two
/// nodes.
const CREST_SEGMENT_M: f64 = 4.0;

/// Closest to its own centerline a crest may be pulled before the run is
/// treated as having no crest on that side at all. A bench overridden to
/// within this of its own axis holds no ground worth constraining — the
/// neighbour's bench is what the mesh must hold there, and it draws its own
/// crest.
const CREST_MIN_OFFSET_M: f64 = 0.5;

/// The bench contact lines of the whole model, as independent segments
/// behind a spatial index, for per-tile queries.
pub struct Breaklines {
    segments: Vec<(Coord, Coord)>,
    grid: GridIndex,
    /// Crest nodes pulled in from their nominal offset, and crest nodes whose
    /// bench held nothing at all — run stats for how crowded the network is.
    pulled: usize,
    dropped: usize,
}

impl Breaklines {
    /// Number of segments, for run stats.
    pub fn len(&self) -> usize {
        self.segments.len()
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Collects the segments whose bbox intersects `(west, south, east,
    /// north)` into `out`, via `scratch` (the caller's reusable id buffer).
    pub fn query(
        &self,
        bbox: (f64, f64, f64, f64),
        scratch: &mut Vec<u32>,
        out: &mut Vec<(Coord, Coord)>,
    ) {
        out.clear();
        if self.segments.is_empty() {
            return;
        }
        self.grid.query(bbox, scratch);
        for &id in scratch.iter() {
            let (a, b) = self.segments[id as usize];
            let (w, e) = (a.x.min(b.x), a.x.max(b.x));
            let (s, n) = (a.y.min(b.y), a.y.max(b.y));
            if e >= bbox.0 && w <= bbox.2 && n >= bbox.1 && s <= bbox.3 {
                out.push((a, b));
            }
        }
    }

    /// Derives the contact lines from the model's earthworks. Bench edges
    /// only — a carve (portal cut, deck daylighting) is a hole, not a surface
    /// the road rides, and its walls read fine from the lattice.
    ///
    /// The field is passed, not just the edge list, because a crest has to be
    /// drawn where its bench *actually* holds the ground, which the edge alone
    /// does not know (see [`crest_offset`]).
    pub fn derive(earthworks: &Earthworks) -> Breaklines {
        let edges = earthworks.edges();
        let mut segments: Vec<(Coord, Coord)> = Vec::new();
        let mut scratch = Vec::new();
        let mut tally = (0usize, 0usize); // (pulled in, dropped)
        // Walk maximal chains of consecutive bench edges (same chain id,
        // shared endpoint) so joint offsets miter instead of gapping.
        let mut i = 0;
        while i < edges.len() {
            if edges[i].carve {
                i += 1;
                continue;
            }
            let start = i;
            while i + 1 < edges.len()
                && !edges[i + 1].carve
                && edges[i + 1].chain == edges[i].chain
                && edges[i + 1].a == edges[i].b
            {
                i += 1;
            }
            emit_run(&edges[start..=i], earthworks, &mut scratch, &mut segments, &mut tally);
            i += 1;
        }
        let mut grid = GridIndex::new();
        for (id, (a, b)) in segments.iter().enumerate() {
            let bbox = (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y));
            grid.insert(bbox, id as u32);
        }
        Breaklines { segments, grid, pulled: tally.0, dropped: tally.1 }
    }

    /// Crest nodes pulled in off their nominal offset by a contending bench,
    /// and crest nodes dropped because no bench of their own survived there.
    pub fn crowding(&self) -> (usize, usize) {
        (self.pulled, self.dropped)
    }
}

/// Emits one bench run's contact polylines as segments: the *crest* line on
/// each side, at the bench edge.
///
/// The crest is where the field genuinely breaks — flat bench inside, batter
/// (or a retaining wall against a neighbouring bench) outside — so it is the
/// line the triangulation must hold. The toe is not emitted: the batter is a
/// straight face that stops where it meets the natural ground, so the toe
/// stands wherever the ground happens to rise into the face, which no offset
/// of the centerline predicts. A constraint drawn at the nominal reach would
/// pin vertices in the wrong place and double the constraint count for it.
///
/// Per-node offsets miter at the joints (clamped to [`MITER_MAX`]) so
/// consecutive segments share their endpoint exactly — adjacent tiles clip the
/// same global polyline and derive identical border vertices (invariant 5).
///
/// Each offset is then pulled in to where this bench still holds the ground
/// ([`crest_offset`]), and a node whose bench holds nothing breaks the line.
fn emit_run(
    run: &[EarthworkEdge],
    earthworks: &Earthworks,
    scratch: &mut Vec<u32>,
    segments: &mut Vec<(Coord, Coord)>,
    tally: &mut (usize, usize),
) {
    let n = run.len() + 1; // nodes
    let cos_lat = run[0].cos_lat;
    // Unit direction of each edge in the metric frame.
    let dir = |e: &EarthworkEdge| -> (f64, f64) {
        let dx = (e.b.x - e.a.x) * cos_lat;
        let dy = e.b.y - e.a.y;
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-15 {
            (1.0, 0.0)
        } else {
            (dx / len, dy / len)
        }
    };
    // Per-node offset direction: the average of the adjacent edges'
    // left-perpendiculars, miter-scaled so the offset polyline stays
    // parallel to both edges through the joint.
    let mut offsets: Vec<(f64, f64)> = Vec::with_capacity(n);
    for k in 0..n {
        let before = if k > 0 { Some(dir(&run[k - 1])) } else { None };
        let after = if k < run.len() { Some(dir(&run[k])) } else { None };
        let (ux, uy) = match (before, after) {
            (Some(a), Some(b)) => {
                let (mx, my) = (a.0 + b.0, a.1 + b.1);
                let len = (mx * mx + my * my).sqrt();
                if len < 1e-9 {
                    a // a fold-back joint: keep the incoming direction
                } else {
                    // Miter scale = 1 / cos(θ/2), clamped.
                    let half_cos = (len * 0.5).min(1.0);
                    let scale = (1.0 / half_cos.max(1.0 / MITER_MAX)).min(MITER_MAX);
                    (mx / len * scale, my / len * scale)
                }
            }
            (Some(d), None) | (None, Some(d)) => d,
            (None, None) => (1.0, 0.0),
        };
        offsets.push((-uy, ux)); // left perpendicular (may carry miter scale)
    }
    let node = |k: usize| if k == 0 { run[0].a } else { run[k - 1].b };
    // The bench half-width varies per edge; a node takes the max of its
    // adjacent edges so the crest line never pinches inside the bench.
    let node_half_width = |k: usize| -> f64 {
        let mut hw: f64 = 0.0;
        if k > 0 {
            hw = hw.max(run[k - 1].half_width_m);
        }
        if k < run.len() {
            hw = hw.max(run[k].half_width_m);
        }
        (hw - CREST_INSET_M).max(0.0)
    };
    let offset_point = |k: usize, side: f64, dist: f64| -> Coord {
        let c = node(k);
        let (px, py) = offsets[k];
        Coord {
            x: c.x + side * px * dist / (DEG_M * cos_lat),
            y: c.y + side * py * dist / DEG_M,
        }
    };
    // The bench target at a node — what the ground holds there, and so what a
    // crest node of this run must find under it.
    let node_target = |k: usize| -> f64 {
        if k == 0 {
            run[0].target_a
        } else {
            run[k - 1].target_b
        }
    };
    for side in [-1.0, 1.0] {
        // Where the crest sits at each of the run's own nodes.
        let mut held: Vec<Option<f64>> = Vec::with_capacity(n);
        for k in 0..n {
            let h = crest_offset(earthworks, node_target(k), node_half_width(k), scratch, &mut |d| {
                offset_point(k, side, d)
            });
            match h {
                // Nothing of this bench survives on this side: another bench
                // holds the ground right up to the road's own axis, and it
                // draws the crest the mesh needs. The line breaks here.
                None => tally.1 += 1,
                Some(v) if v < node_half_width(k) - 1e-9 => tally.0 += 1,
                Some(_) => {}
            }
            held.push(h);
        }

        // Drop a folded segment instead of emitting it. On the inside of a bend
        // tighter than the offset — a hairpin, a switchback on a vineyard track
        // — the offset polyline reverses and loops back over itself. Fed to the
        // triangulation those loops become crossing constraints that get split
        // against each other, and the mesh holds the resulting zigzag as real
        // geometry: whole hillsides of tracks came out as rows of sawtooth
        // teeth. The bench there is narrower than its own offset anyway, so the
        // honest contact line is no line at all (docs/GROUND.md §3).
        let push = |a: Coord, b: Coord, edge: usize, segments: &mut Vec<(Coord, Coord)>| {
            let (dx, dy) = ((b.x - a.x) * cos_lat, b.y - a.y);
            let (ux, uy) = dir(&run[edge]);
            if a != b && dx * ux + dy * uy > 0.0 {
                segments.push((a, b));
            }
        };

        let mut prev: Option<Coord> = None;
        for k in 0..n {
            let Some(h) = held[k] else {
                prev = None;
                continue;
            };
            let p = offset_point(k, side, h);
            if let Some(q) = prev {
                let e = k - 1;
                // A contended segment tracks the boundary instead of chording
                // it (see CREST_SEGMENT_M); an uncontended one is exact as it
                // stands and keeps its two nodes.
                let pulled = |k: usize| held[k].is_some_and(|v| v < node_half_width(k) - 1e-9);
                let (a, b) = (node(e), node(k));
                let len = ((b.x - a.x) * cos_lat).hypot(b.y - a.y) * DEG_M;
                let steps = if pulled(e) || pulled(k) {
                    (len / CREST_SEGMENT_M).ceil() as usize
                } else {
                    1
                };
                let mut last = Some(q);
                for i in 1..steps {
                    let t = i as f64 / steps as f64;
                    let base = Coord { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
                    let (o0, o1) = (offsets[e], offsets[k]);
                    let (px, py) = (o0.0 + (o1.0 - o0.0) * t, o0.1 + (o1.1 - o0.1) * t);
                    let at = |d: f64| Coord {
                        x: base.x + side * px * d / (DEG_M * cos_lat),
                        y: base.y + side * py * d / DEG_M,
                    };
                    let nominal =
                        node_half_width(e) + (node_half_width(k) - node_half_width(e)) * t;
                    let target = run[e].target_a + (run[e].target_b - run[e].target_a) * t;
                    match crest_offset(earthworks, target, nominal, scratch, &mut |d| at(d)) {
                        Some(hi) => {
                            let s = at(hi);
                            if let Some(l) = last {
                                push(l, s, e, segments);
                            }
                            last = Some(s);
                        }
                        // The bench loses the ground mid-segment: break the
                        // line rather than chord across the gap.
                        None => last = None,
                    }
                }
                if let Some(l) = last {
                    push(l, p, e, segments);
                }
            }
            prev = Some(p);
        }
    }
}

/// How far out this bench still holds the ground, given the nominal crest
/// offset and a ray out from the road: the largest offset up to `nominal` whose
/// ground is still this bench's own `target`, or `None` when the bench holds
/// nothing worth constraining on that side.
///
/// The nominal offset is a *geometric* edge — the half-width the earthwork was
/// built with — and where two benches overlap it is not where the field steps.
/// Benches win by proximity, so between two roads closer together than their
/// half-widths the winner changes at a boundary somewhere between them, and the
/// nominal crest of each lies past it, inside the other's reign. A crest node
/// there samples the *neighbour's* height (the z rule: a breakline says where to
/// sample, never what to find), and the triangulation then ramps that height
/// back across the road the crest was drawn to protect — which is exactly how a
/// service road beside a street three metres higher ended up under the hill.
///
/// So the crest is placed where the field agrees it belongs: step inward from
/// the nominal offset until the ground under the line is this bench's own
/// surface again, then refine that step by bisection.
///
/// Inward from the *outside*, not by bisecting the whole span, because the
/// bench's reign along the ray is not a simple interval: a neighbour crossing at
/// an angle takes a band out of the middle of it, and a bisection then converges
/// on whichever crossing it happens to bracket — often the near one, leaving the
/// outer part of the bench, the part the carriageway actually covers,
/// unconstrained for the mesh to ramp a neighbour's height across. The outermost
/// holding offset is the one that protects the road.
///
/// Deterministic — a fixed step count over the same total order the ground
/// function resolves benches with, so every tile derives the identical line
/// (invariant 5).
fn crest_offset(
    earthworks: &Earthworks,
    target: f64,
    nominal: f64,
    scratch: &mut Vec<u32>,
    point_at: &mut dyn FnMut(f64) -> Coord,
) -> Option<f64> {
    let mut holds = |d: f64, scratch: &mut Vec<u32>| -> bool {
        let p = point_at(d);
        match earthworks.target_at(p.x, p.y, scratch) {
            // No bench at all under the line: the batter starts here, which is
            // the contact the crest stands for. Nothing to pull in from.
            None => true,
            Some(t) => (t - target).abs() <= CREST_STEP_M,
        }
    };
    if holds(nominal, scratch) {
        return Some(nominal); // uncontended: the pull-in costs nothing
    }
    let step = (nominal - CREST_MIN_OFFSET_M) / CREST_SCAN_STEPS as f64;
    if step <= 0.0 {
        return None;
    }
    for i in 1..=CREST_SCAN_STEPS {
        let d = nominal - i as f64 * step;
        if !holds(d, scratch) {
            continue;
        }
        // Between `d` (holds) and the step above it (does not) lies the
        // boundary; a few halvings put the crest within centimetres of it.
        let (mut lo, mut hi) = (d, d + step);
        for _ in 0..CREST_BISECT_STEPS {
            let mid = 0.5 * (lo + hi);
            if holds(mid, scratch) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return Some(lo);
    }
    // Nothing of this bench survives on this side.
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edge(a: Coord, b: Coord, hw: f64, batter: f64, chain: u32) -> EarthworkEdge {
        EarthworkEdge {
            a,
            b,
            target_a: 400.0,
            target_b: 400.0,
            half_width_m: hw,
            carriageway_m: (hw - 1.0).max(0.0),
            batter_m: [batter; 2],
            batter_run: [crate::priors::EARTHWORK_BATTER; 2],
            chain,
            arc0: 0.0,
            cos_lat: 46.0_f64.to_radians().cos(),
            carve: false,
        }
    }

    #[test]
    fn a_straight_run_emits_a_crest_line_per_side() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let step = 30.0 / (DEG_M * cos_lat);
        let p = |i: f64| Coord { x: 6.0 + i * step, y: 46.0 };
        let edges = vec![edge(p(0.0), p(1.0), 5.0, 10.0, 0), edge(p(1.0), p(2.0), 5.0, 10.0, 0)];
        let b = Breaklines::derive(&Earthworks::new(edges));
        // 2 crest lines × 2 segments each.
        assert_eq!(b.len(), 4);
        // Crest lines sit just inside the 5 m bench edge; an east-west run
        // offsets purely in latitude.
        let mut scratch = Vec::new();
        let mut out = Vec::new();
        b.query((5.9, 45.9, 6.1, 46.1), &mut scratch, &mut out);
        assert_eq!(out.len(), 4);
        for (a, _) in &out {
            let off_m = ((a.y - 46.0) * DEG_M).abs();
            assert!(
                (off_m - (5.0 - CREST_INSET_M)).abs() < 1e-6,
                "offset {off_m} must sit just inside the bench edge"
            );
        }
    }

    #[test]
    fn carves_and_chain_breaks_split_runs() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let step = 30.0 / (DEG_M * cos_lat);
        let p = |i: f64| Coord { x: 6.0 + i * step, y: 46.0 };
        let mut carve = edge(p(2.0), p(3.0), 5.0, 10.0, 0);
        carve.carve = true;
        let edges = vec![
            edge(p(0.0), p(1.0), 5.0, 10.0, 0),
            carve,
            edge(p(3.0), p(4.0), 5.0, 10.0, 1),
        ];
        let b = Breaklines::derive(&Earthworks::new(edges));
        // Two single-edge bench runs, 2 crest segments each; the carve emits
        // none.
        assert_eq!(b.len(), 4);
    }

    /// Two roads closer together than their bench half-widths, three metres
    /// apart in height — a service way under a street on a hillside. The
    /// nominal crest of each lies inside the other's reign, where it would
    /// sample the other's height and let the mesh ramp it back over the road
    /// it belongs to. Each crest must instead stop at the boundary where its
    /// own bench stops holding the ground.
    #[test]
    fn a_crest_stops_where_its_own_bench_stops_holding() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let step = 30.0 / (DEG_M * cos_lat);
        // Two parallel east-west runs 7 m apart, benches 4.25 m half-width:
        // they overlap, and the winner changes at the 3.5 m midline.
        let lower = 46.0;
        let upper = 46.0 + 7.0 / DEG_M;
        let run = |lat: f64, target: f64, chain: u32| {
            let mut e = edge(
                Coord { x: 6.0, y: lat },
                Coord { x: 6.0 + step, y: lat },
                4.25,
                10.0,
                chain,
            );
            e.target_a = target;
            e.target_b = target;
            e
        };
        let ew = Earthworks::new(vec![run(lower, 400.0, 0), run(upper, 403.0, 1)]);
        let b = Breaklines::derive(&ew);
        let mut scratch = Vec::new();
        let mut out = Vec::new();
        b.query((5.9, 45.9, 6.1, 46.1), &mut scratch, &mut out);
        assert!(!out.is_empty(), "both runs still carry crests");
        for (a, _) in &out {
            // Every crest vertex must find its own bench under it: the ground
            // there is either 400 or 403, never the ramp between them, and the
            // vertex must sit on the side of the midline it belongs to.
            let h = ew.height(a.x, a.y, f64::NAN, 0.0, &mut scratch);
            let own_lower = (a.y - lower).abs() < (a.y - upper).abs();
            assert!(
                (h - if own_lower { 400.0 } else { 403.0 }).abs() < 1e-9,
                "crest at {:.6} sampled {h}, not the bench it belongs to",
                a.y
            );
            // …and it stays clear of the neighbour's bench entirely.
            let off_m = ((a.y - if own_lower { lower } else { upper }) * DEG_M).abs();
            assert!(off_m <= 4.25 - CREST_INSET_M + 1e-9, "crest offset {off_m} left its bench");
        }
        // A run with no neighbour keeps the full nominal offset, so the pull-in
        // costs nothing where nothing contends.
        let alone = Breaklines::derive(&Earthworks::new(vec![run(lower, 400.0, 0)]));
        alone.query((5.9, 45.9, 6.1, 46.1), &mut scratch, &mut out);
        for (a, _) in &out {
            let off_m = ((a.y - lower) * DEG_M).abs();
            assert!(
                (off_m - (4.25 - CREST_INSET_M)).abs() < 1e-6,
                "an uncontended crest keeps its nominal offset, got {off_m}"
            );
        }
    }

    /// A neighbour that crowds a bench at one end and not the other: the crest
    /// between the two nodes must *track* the boundary, not chord between the
    /// pulled-in end and the full-width one. The chord cuts inside the bench —
    /// under the carriageway's own asphalt — and the mesh then ramps the
    /// neighbour's wall over the strip outside it.
    #[test]
    fn a_contended_crest_tracks_the_boundary_between_its_nodes() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let long = 40.0 / (DEG_M * cos_lat); // one 40 m edge: a street's node spacing
        let lower = 46.0;
        let mut street = edge(
            Coord { x: 6.0, y: lower },
            Coord { x: 6.0 + long, y: lower },
            4.25,
            10.0,
            0,
        );
        street.target_a = 400.0;
        street.target_b = 400.0;
        // A neighbour three metres higher, running *diagonally* in: 12 m away
        // at the start of the street's edge, 5 m away at its end. So the
        // street's bench is uncontended at one end and squeezed at the other,
        // and the boundary between them is a slanted line.
        let mut rail = edge(
            Coord { x: 6.0, y: lower + 12.0 / DEG_M },
            Coord { x: 6.0 + long, y: lower + 5.0 / DEG_M },
            4.25,
            10.0,
            1,
        );
        rail.target_a = 403.0;
        rail.target_b = 403.0;
        let ew = Earthworks::new(vec![street, rail]);
        let b = Breaklines::derive(&ew);
        let mut scratch = Vec::new();
        let mut out = Vec::new();
        b.query((5.9, 45.9, 6.1, 46.1), &mut scratch, &mut out);

        // The two benches meet halfway between the axes, so at parameter `t`
        // along the edge the street's bench ends at `min(nominal, (12 - 7t)/2)`
        // — a slanted boundary the crest has to follow. Sample across the edge
        // and require a crest to be there, within a few centimetres.
        let want = |t: f64| (4.25 - CREST_INSET_M).min((12.0 - 7.0 * t) * 0.5);
        for &t in &[0.2, 0.4, 0.6, 0.8, 0.95] {
            let x = 6.0 + long * t;
            let target = want(t);
            let mut best = f64::INFINITY;
            for (p, q) in &out {
                if (x - p.x.min(q.x)) < -1e-12 || (x - p.x.max(q.x)) > 1e-12 {
                    continue;
                }
                let dx = q.x - p.x;
                let s = if dx.abs() < 1e-15 { 0.0 } else { (x - p.x) / dx };
                let off = (p.y + (q.y - p.y) * s - lower) * DEG_M;
                if off <= 0.0 {
                    continue; // the southern crest, and the rail's far side
                }
                best = best.min((off - target).abs());
            }
            assert!(
                best < 0.25,
                "at t={t} the boundary is {target:.2} m out; \
                 the nearest crest there is {best:.2} m off it"
            );
        }
        // …and the crest never crosses into the neighbour's ground: every
        // vertex on the contended side finds a bench of its own run's height.
        for (p, q) in &out {
            for c in [p, q] {
                let off = (c.y - lower) * DEG_M;
                if !(0.0..4.25).contains(&off) {
                    continue;
                }
                let h = ew.height(c.x, c.y, f64::NAN, 0.0, &mut scratch);
                assert!(
                    (h - 400.0).abs() < 1e-9 || (h - 403.0).abs() < 1e-9,
                    "crest vertex at {off:.2} m found {h}, which is neither bench"
                );
            }
        }
    }

    #[test]
    fn query_filters_by_bbox() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let step = 30.0 / (DEG_M * cos_lat);
        let p = |i: f64| Coord { x: 6.0 + i * step, y: 46.0 };
        let b = Breaklines::derive(&Earthworks::new(vec![edge(p(0.0), p(1.0), 5.0, 10.0, 0)]));
        let mut scratch = Vec::new();
        let mut out = Vec::new();
        b.query((7.0, 47.0, 7.1, 47.1), &mut scratch, &mut out);
        assert!(out.is_empty(), "a far bbox must match nothing");
        b.query((5.9, 45.9, 6.1, 46.1), &mut scratch, &mut out);
        assert_eq!(out.len(), 2);
    }
}
