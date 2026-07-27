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

use super::modifiers::EarthworkEdge;

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

/// The bench contact lines of the whole model, as independent segments
/// behind a spatial index, for per-tile queries.
pub struct Breaklines {
    segments: Vec<(Coord, Coord)>,
    grid: GridIndex,
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

    /// Derives the contact lines from the model's earthwork edges. Bench
    /// edges only — a carve (portal cut, deck daylighting) is a hole, not a
    /// surface the road rides, and its walls read fine from the lattice.
    pub fn derive(edges: &[EarthworkEdge]) -> Breaklines {
        let mut segments: Vec<(Coord, Coord)> = Vec::new();
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
            emit_run(&edges[start..=i], &mut segments);
            i += 1;
        }
        let mut grid = GridIndex::new();
        for (id, (a, b)) in segments.iter().enumerate() {
            let bbox = (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y));
            grid.insert(bbox, id as u32);
        }
        Breaklines { segments, grid }
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
fn emit_run(run: &[EarthworkEdge], segments: &mut Vec<(Coord, Coord)>) {
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
    for side in [-1.0, 1.0] {
        let mut prev: Option<Coord> = None;
        for k in 0..n {
            let p = offset_point(k, side, node_half_width(k));
            if let Some(q) = prev {
                // Drop a folded segment instead of emitting it. On the inside
                // of a bend tighter than the offset — a hairpin, a switchback
                // on a vineyard track — the offset polyline reverses and loops
                // back over itself. Fed to the triangulation those loops become
                // crossing constraints that get split against each other, and
                // the mesh holds the resulting zigzag as real geometry: whole
                // hillsides of tracks came out as rows of sawtooth teeth. The
                // bench there is narrower than its own offset anyway, so the
                // honest contact line is no line at all (docs/GROUND.md §3).
                let (dx, dy) = ((p.x - q.x) * cos_lat, p.y - q.y);
                let (ux, uy) = dir(&run[k - 1]);
                if q != p && dx * ux + dy * uy > 0.0 {
                    segments.push((q, p));
                }
            }
            prev = Some(p);
        }
    }
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
            batter_m: [batter; 2],
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
        let b = Breaklines::derive(&edges);
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
        let b = Breaklines::derive(&edges);
        // Two single-edge bench runs, 2 crest segments each; the carve emits
        // none.
        assert_eq!(b.len(), 4);
    }

    #[test]
    fn query_filters_by_bbox() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let step = 30.0 / (DEG_M * cos_lat);
        let p = |i: f64| Coord { x: 6.0 + i * step, y: 46.0 };
        let b = Breaklines::derive(&[edge(p(0.0), p(1.0), 5.0, 10.0, 0)]);
        let mut scratch = Vec::new();
        let mut out = Vec::new();
        b.query((7.0, 47.0, 7.1, 47.1), &mut scratch, &mut out);
        assert!(out.is_empty(), "a far bbox must match nothing");
        b.query((5.9, 45.9, 6.1, 46.1), &mut scratch, &mut out);
        assert_eq!(out.len(), 2);
    }
}
