//! Earthwork modifiers — the local reshapings of the natural terrain that the
//! solved model implies (docs/GENERATION.md §6 stage 3, D3).
//!
//! Milestone M-b ships the road earthwork: wherever a solved corridor departs
//! the natural ground on an at-grade stretch — a grade-limited cut through a
//! bump, the embankment ramp climbing to an overpass — the ground is pulled to
//! the road. Each earthwork is a chain of [`EarthworkEdge`]s along the
//! corridor centerline carrying the target (road) height; a query point within
//! the half-width takes the target, within the feather blends smoothly back to
//! the natural ground, and beyond it is untouched.
//!
//! Where several earthworks overlap (a junction, hairpin legs, a street beside
//! a graded corridor) their targets *blend*: each edge's share is 1 across its
//! core (the asphalt band) and decays smoothly to zero at the end of its
//! feather, and the ground takes the share-weighted mean. A winner-take-all
//! rule here would put a cliff on the winner boundary — and since that
//! boundary weaves between the sparse vertices of the draped road surface,
//! the terrain would step through the asphalt mid-chord. The blend keeps the
//! field continuous, so every consumer (terrain corner, band vertex, paint,
//! plate) interpolates the same surface. It is a share-weighted sum over a
//! deterministically ordered edge set — a pure function of the query point,
//! so any two tiles (and any two zooms) derive identical ground (invariant 5).

use geo_types::Coord;

use crate::assemble::grid::GridIndex;
use crate::scene::DEG_M;

/// Largest arc gap, in metres, between one chain's consecutive covering
/// edges that still counts as the same approach. Just above the bed edge
/// spacing (`BED_SPACING_M` = 30): consecutive covering edges of one
/// approach sit at most one edge length apart, while even a tight mountain
/// hairpin's other leg arrives via the turn, farther than this. Splitting a
/// long-edged straight run is harmless — a point is only covered by edges
/// within its lateral reach, so split clusters carry near-identical targets
/// — but *merging* a hairpin resurrects the winner-take-all cliff between
/// its legs.
const CLUSTER_GAP_M: f64 = 35.0;

/// One earthwork centerline edge: endpoints with target heights, the
/// road-height half-width, and the slope reach beyond it.
#[derive(Debug, Clone, Copy)]
pub struct EarthworkEdge {
    pub a: Coord,
    pub b: Coord,
    pub target_a: f64,
    pub target_b: f64,
    /// Held at target within this lateral distance (road + shoulder + the
    /// rendering margin), metres.
    pub half_width_m: f64,
    /// Smoothstep blend back to natural ground over this further distance.
    pub feather_m: f64,
    /// The asphalt-band half-width, within which this edge's *share* against
    /// overlapping earthworks stays 1 — its own road must ride its own
    /// height. From here the share decays to zero at the feather's end, so
    /// two benches meeting side by side (hairpin legs, a street beside a
    /// corridor) ramp into each other across their margins instead of
    /// stepping on a winner boundary. At most `half_width_m`.
    pub core_half_m: f64,
    /// The earthwork run this edge belongs to (one id per corridor or bed).
    /// Blending happens *across* runs; within one arc-contiguous stretch of a
    /// run the nearest edge is exact, so consecutive edges of a straight road
    /// never smear each other's targets along the profile.
    pub chain: u32,
    /// Arc position of `a` along the chain, metres — separates a hairpin's
    /// two legs (far apart along the road, near in space) into distinct
    /// blending clusters.
    pub arc0: f64,
    /// `cos(mean latitude)` of the source corridor, for the metric projection.
    pub cos_lat: f64,
    /// Cut-only: the edge may lower the ground to its target but never raise
    /// it — a portal daylighting cut must not build a berm where the natural
    /// ground already sits below the bore floor.
    pub carve: bool,
}

impl EarthworkEdge {
    /// The edge's blend weights at lateral distance `d`: the outer envelope
    /// `w` (1 across the held width, smoothstep to 0 over the feather) and
    /// the share `q·w` used against overlapping earthworks (`q` is 1 across
    /// the core and decays quadratically to 0 at the feather's end, so the
    /// share is strictly positive wherever the envelope is). `None` beyond
    /// the edge's reach.
    fn weights(&self, d: f64) -> Option<(f64, f64)> {
        let reach = self.half_width_m + self.feather_m;
        if d >= reach {
            return None;
        }
        let w = if d <= self.half_width_m {
            1.0
        } else {
            let u = (d - self.half_width_m) / self.feather_m;
            1.0 - u * u * (3.0 - 2.0 * u) // smoothstep down
        };
        let span = (reach - self.core_half_m).max(f64::MIN_POSITIVE);
        let u = ((d - self.core_half_m) / span).clamp(0.0, 1.0);
        let q = (1.0 - u) * (1.0 - u);
        Some((w, q * w))
    }
}

/// The indexed set of earthwork edges with point queries.
pub struct Earthworks {
    edges: Vec<EarthworkEdge>,
    grid: GridIndex,
}

impl Earthworks {
    pub fn new(edges: Vec<EarthworkEdge>) -> Earthworks {
        let mut grid = GridIndex::new();
        for (i, e) in edges.iter().enumerate() {
            // Inflate by the edge's full reach so a point query needs no
            // radius of its own.
            let reach_deg = (e.half_width_m + e.feather_m) / (DEG_M * e.cos_lat.min(1.0).max(0.1));
            let bb = (
                e.a.x.min(e.b.x) - reach_deg,
                e.a.y.min(e.b.y) - reach_deg,
                e.a.x.max(e.b.x) + reach_deg,
                e.a.y.max(e.b.y) + reach_deg,
            );
            grid.insert(bb, i as u32);
        }
        Earthworks { edges, grid }
    }

    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    pub fn len(&self) -> usize {
        self.edges.len()
    }

    pub fn edges(&self) -> &[EarthworkEdge] {
        &self.edges
    }

    /// The blended fill contribution at `(lon, lat)`: the share-weighted mean
    /// target over the covering *approaches* and the strongest outer
    /// envelope, or `None` where no fill reaches.
    ///
    /// An approach is an arc-contiguous cluster of one chain's covering
    /// edges, represented by its nearest edge: consecutive edges of a
    /// straight road collapse to the single exact answer (no smearing of the
    /// profile along the run), while a hairpin's two legs — the same chain
    /// arriving from arc positions more than [`CLUSTER_GAP_M`] apart — and
    /// any other road's bench blend smoothly. Hits are sorted before
    /// clustering and accumulation, so the float sum — and therefore the
    /// ground — is identical whatever tile asked (invariant 5).
    fn fill_blend(&self, lon: f64, lat: f64, scratch: &[u32]) -> Option<(f64, f64)> {
        // (chain, arc0, d, idx, w, share, target) per covering fill edge.
        let mut hits: Vec<(u32, f64, f64, u32, f64, f64, f64)> = Vec::new();
        for &i in scratch {
            let e = &self.edges[i as usize];
            if e.carve {
                continue;
            }
            let (d, t) = lateral_distance(e, lon, lat);
            let Some((w, share)) = e.weights(d) else { continue };
            let target = e.target_a + (e.target_b - e.target_a) * t;
            hits.push((e.chain, e.arc0, d, i, w, share, target));
        }
        if hits.is_empty() {
            return None;
        }
        hits.sort_unstable_by(|a, b| {
            a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)).then(a.3.cmp(&b.3))
        });
        let (mut num, mut den, mut outer) = (0.0, 0.0, 0.0f64);
        // Best edge of the current cluster: (d, idx, w, share, target).
        let mut best: Option<(f64, u32, f64, f64, f64)> = None;
        let mut prev: Option<(u32, f64)> = None; // (chain, arc0) of the last hit
        let mut flush = |b: &Option<(f64, u32, f64, f64, f64)>,
                         num: &mut f64,
                         den: &mut f64,
                         outer: &mut f64| {
            if let Some((_, _, w, share, target)) = b {
                *num += share * target;
                *den += share;
                *outer = outer.max(*w);
            }
        };
        for (chain, arc0, d, idx, w, share, target) in hits {
            let same = matches!(prev, Some((pc, pa)) if pc == chain && arc0 - pa <= CLUSTER_GAP_M);
            if !same {
                flush(&best, &mut num, &mut den, &mut outer);
                best = None;
            }
            let better = match &best {
                None => true,
                Some((bd, bi, _, _, _)) => d < *bd - 1e-9 || ((d - *bd).abs() <= 1e-9 && idx < *bi),
            };
            if better {
                best = Some((d, idx, w, share, target));
            }
            prev = Some((chain, arc0));
        }
        flush(&best, &mut num, &mut den, &mut outer);
        (den > 0.0).then(|| (num / den, outer))
    }

    /// The engineered height at `(lon, lat)` given the natural ground `raw`:
    /// the blended fill target, feathered into `raw` by the strongest
    /// envelope, then cut by any covering carve notch.
    pub fn height(&self, lon: f64, lat: f64, raw: f64, scratch: &mut Vec<u32>) -> f64 {
        self.grid.query((lon, lat, lon, lat), scratch);
        scratch.sort_unstable();
        let mut h = match self.fill_blend(lon, lat, scratch) {
            Some((target, outer)) => raw + (target - raw) * outer,
            None => raw,
        };
        // Carves stay winner-take-all (strongest weight, then nearest, then
        // lowest index — a total order): a notch is a hole, not a surface to
        // average. Cut-only, applied to the filled ground.
        let mut best: Option<(f64, f64, u32, f64)> = None; // (weight, dist, idx, target)
        for &i in scratch.iter() {
            let e = &self.edges[i as usize];
            if !e.carve {
                continue;
            }
            let (d, t) = lateral_distance(e, lon, lat);
            let Some((w, _)) = e.weights(d) else { continue };
            let target = e.target_a + (e.target_b - e.target_a) * t;
            if target >= h {
                continue; // nothing to cut here
            }
            let better = match &best {
                None => true,
                Some((bw, bd, bi, _)) => {
                    w > *bw + 1e-12
                        || ((w - *bw).abs() <= 1e-12 && (d < *bd - 1e-9 || ((d - *bd).abs() <= 1e-9 && i < *bi)))
                }
            };
            if better {
                best = Some((w, d, i, target));
            }
        }
        if let Some((w, _, _, target)) = best {
            h += (target - h) * w;
        }
        h
    }

    /// The exact roadbed height at `(lon, lat)` when the point lies fully
    /// inside a non-carve modifier's held half-width (envelope 1), else
    /// `None` — including in the feather, where the ground blends back to
    /// natural. The same [`Earthworks::fill_blend`] the terrain reads, so the
    /// road stack and the drawn ground can never disagree where they overlap.
    /// The road drape rides this at the reference zoom, where the rendered
    /// lattice is far too coarse to capture a street-wide bench (see
    /// `synth::road::surface_height`). Carve notches (deck daylighting,
    /// portal cuts) are not beds.
    pub fn target_at(&self, lon: f64, lat: f64, scratch: &mut Vec<u32>) -> Option<f64> {
        self.grid.query((lon, lat, lon, lat), scratch);
        scratch.sort_unstable();
        match self.fill_blend(lon, lat, scratch) {
            Some((target, outer)) if outer >= 1.0 => Some(target),
            _ => None,
        }
    }
}

/// One still water body flattened to a level: its rings (for the interior
/// test) and the surface height the ground is burned to inside them.
#[derive(Debug, Clone)]
pub struct WaterFill {
    pub exterior: Vec<Coord>,
    pub holes: Vec<Vec<Coord>>,
    pub bbox: (f64, f64, f64, f64),
    pub level: f64,
}

/// The indexed set of water fills with point queries.
pub struct Waters {
    fills: Vec<WaterFill>,
    grid: GridIndex,
}

impl Waters {
    pub fn new(fills: Vec<WaterFill>) -> Waters {
        let mut grid = GridIndex::new();
        for (i, f) in fills.iter().enumerate() {
            grid.insert(f.bbox, i as u32);
        }
        Waters { fills, grid }
    }

    pub fn is_empty(&self) -> bool {
        self.fills.is_empty()
    }

    pub fn len(&self) -> usize {
        self.fills.len()
    }

    /// The water surface level at `(lon, lat)` when the point lies inside a
    /// still water body (its exterior ring, minus island holes). Deterministic:
    /// the lowest-index containing body wins, so any two tiles agree.
    pub fn level_at(&self, lon: f64, lat: f64, scratch: &mut Vec<u32>) -> Option<f64> {
        self.grid.query((lon, lat, lon, lat), scratch);
        for &i in scratch.iter() {
            let f = &self.fills[i as usize];
            if lon < f.bbox.0 || lon > f.bbox.2 || lat < f.bbox.1 || lat > f.bbox.3 {
                continue;
            }
            if point_in_ring(&f.exterior, lon, lat)
                && !f.holes.iter().any(|h| point_in_ring(h, lon, lat))
            {
                return Some(f.level);
            }
        }
        None
    }
}

/// Even-odd ray-casting point-in-ring test for a closed lon/lat loop. A
/// horizontal edge contributes no crossing (the `(yi > y) != (yj > y)` guard),
/// so the divisor is never zero.
fn point_in_ring(ring: &[Coord], x: f64, y: f64) -> bool {
    let n = ring.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (ring[i].x, ring[i].y);
        let (xj, yj) = (ring[j].x, ring[j].y);
        if (yi > y) != (yj > y) {
            let x_cross = xi + (y - yi) / (yj - yi) * (xj - xi);
            if x < x_cross {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// Lateral distance in metres from `(lon, lat)` to the edge, and the clamped
/// parameter along it.
fn lateral_distance(e: &EarthworkEdge, lon: f64, lat: f64) -> (f64, f64) {
    let ax = e.a.x * e.cos_lat;
    let dx = (e.b.x - e.a.x) * e.cos_lat;
    let dy = e.b.y - e.a.y;
    let px = lon * e.cos_lat;
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((px - ax) * dx + (lat - e.a.y) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let cx = ax + dx * t;
    let cy = e.a.y + dy * t;
    let dd = ((px - cx) * (px - cx) + (lat - cy) * (lat - cy)).sqrt() * DEG_M;
    (dd, t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edge(target: f64) -> EarthworkEdge {
        // An east-west edge ~160 m long at lat 46.
        let cos_lat = 46.0_f64.to_radians().cos();
        EarthworkEdge {
            a: Coord { x: 6.0, y: 46.0 },
            b: Coord { x: 6.0 + 160.0 / (DEG_M * cos_lat), y: 46.0 },
            target_a: target,
            target_b: target,
            half_width_m: 8.0,
            feather_m: 10.0,
            core_half_m: 5.0,
            chain: 0,
            arc0: 0.0,
            cos_lat,
            carve: false,
        }
    }

    #[test]
    fn a_carve_edge_cuts_but_never_fills() {
        let mut scratch = Vec::new();
        let mut e = edge(105.0);
        e.carve = true;
        let ew = Earthworks::new(vec![e]);
        let mid_x = 6.0 + 80.0 / (DEG_M * 46.0_f64.to_radians().cos());
        // Ground above the target: cut down to it.
        assert!((ew.height(mid_x, 46.0, 110.0, &mut scratch) - 105.0).abs() < 1e-9);
        // Ground already below the target: untouched (no berm).
        assert_eq!(ew.height(mid_x, 46.0, 100.0, &mut scratch), 100.0);
    }

    #[test]
    fn pulls_ground_to_target_within_the_half_width() {
        let mut scratch = Vec::new();
        let e = Earthworks::new(vec![edge(105.0)]);
        // On the centerline: exactly the target.
        let mid_x = 6.0 + 80.0 / (DEG_M * 46.0_f64.to_radians().cos());
        assert!((e.height(mid_x, 46.0, 100.0, &mut scratch) - 105.0).abs() < 1e-9);
        // 5 m off (inside half-width): still the target.
        let off = 5.0 / DEG_M;
        assert!((e.height(mid_x, 46.0 + off, 100.0, &mut scratch) - 105.0).abs() < 1e-9);
    }

    #[test]
    fn feather_blends_back_to_natural_ground() {
        let mut scratch = Vec::new();
        let e = Earthworks::new(vec![edge(105.0)]);
        let mid_x = 6.0 + 80.0 / (DEG_M * 46.0_f64.to_radians().cos());
        // Mid-feather (13 m off, 5 m into the 10 m feather): between the two.
        let h = e.height(mid_x, 46.0 + 13.0 / DEG_M, 100.0, &mut scratch);
        assert!(h > 100.5 && h < 104.5, "mid-feather should blend, got {h}");
        // Beyond the reach: untouched.
        let h = e.height(mid_x, 46.0 + 30.0 / DEG_M, 100.0, &mut scratch);
        assert_eq!(h, 100.0);
    }

    #[test]
    fn water_flattens_its_interior_but_not_outside_or_in_a_hole() {
        let mut scratch = Vec::new();
        // A unit square lake at ~lat 46 with a small square island (hole).
        let square = |x0: f64, y0: f64, s: f64| {
            vec![
                Coord { x: x0, y: y0 },
                Coord { x: x0 + s, y: y0 },
                Coord { x: x0 + s, y: y0 + s },
                Coord { x: x0, y: y0 + s },
                Coord { x: x0, y: y0 },
            ]
        };
        let exterior = square(6.0, 46.0, 0.010);
        let hole = square(6.004, 46.004, 0.002);
        let waters = Waters::new(vec![WaterFill {
            exterior,
            holes: vec![hole],
            bbox: (6.0, 46.0, 6.010, 46.010),
            level: 372.0,
        }]);
        // Interior open water: flattened to the level.
        assert_eq!(waters.level_at(6.002, 46.002, &mut scratch), Some(372.0));
        // Inside the island hole: not water.
        assert_eq!(waters.level_at(6.005, 46.005, &mut scratch), None);
        // Outside the lake: not water.
        assert_eq!(waters.level_at(6.02, 46.02, &mut scratch), None);
    }

    #[test]
    fn overlapping_earthworks_blend_continuously() {
        // Two parallel street benches at different heights, close enough that
        // their margins and feathers overlap — hairpin legs on a slope. The
        // ground must hold each road's own height across its core, ramp
        // continuously between them (a winner-take-all step here weaves
        // between the sparse band vertices and pokes through the asphalt),
        // and stay consistent with what the road drape reads (`target_at`).
        let mut scratch = Vec::new();
        let cos_lat = 46.0_f64.to_radians().cos();
        // Street-like edges: band (core) 4.75 m, held 7.75 m, feather 4 m,
        // centerlines 18 m apart — reaches (11.75 m) overlap mid-gap.
        let mut lower = edge(103.0);
        lower.core_half_m = 4.75;
        lower.half_width_m = 7.75;
        lower.feather_m = 4.0;
        let mut upper = lower;
        upper.target_a = 110.0;
        upper.target_b = 110.0;
        upper.a.y += 18.0 / DEG_M;
        upper.b.y += 18.0 / DEG_M;
        upper.chain = 1; // a distinct road (or the far leg of a hairpin)
        let e = Earthworks::new(vec![lower, upper]);
        let mid_x = 6.0 + 80.0 / (DEG_M * cos_lat);
        let h_at = |off_m: f64, scratch: &mut Vec<u32>| {
            e.height(mid_x, 46.0 + off_m / DEG_M, 100.0, scratch)
        };
        // On each centerline the other bench is out of reach: own height, exact.
        assert!((h_at(0.0, &mut scratch) - 103.0).abs() < 1e-9);
        assert!((h_at(18.0, &mut scratch) - 110.0).abs() < 1e-9);
        // Across each road's core the bed stays its own (the asphalt is flat).
        assert!((h_at(4.5, &mut scratch) - 103.0).abs() < 0.35, "lower core must hold its bed");
        assert!((h_at(13.5, &mut scratch) - 110.0).abs() < 0.35, "upper core must hold its bed");
        // Mid-gap the ground sits strictly between the two beds.
        let mid = h_at(9.0, &mut scratch);
        assert!(mid > 103.5 && mid < 109.5, "the gap must ramp, got {mid}");
        // Continuity: no cliff anywhere on a transect across both benches (a
        // winner-take-all boundary would jump the full 7 m in one sample;
        // steep-but-continuous ramps between 18 m-apart benches are fine).
        let mut prev = h_at(-14.0, &mut scratch);
        let mut off = -13.75;
        while off <= 32.0 {
            let h = h_at(off, &mut scratch);
            assert!(
                (h - prev).abs() < 2.0,
                "step of {:.2} m at {off} m — the blend must be continuous",
                (h - prev).abs()
            );
            prev = h;
            off += 0.25;
        }
        // The road drape reads the same field: inside the held width,
        // target_at equals the drawn ground exactly.
        let probe = 46.0 + 6.5 / DEG_M; // inside lower's held width, upper's feather
        let t = e.target_at(mid_x, probe, &mut scratch).expect("inside a held width");
        let h = e.height(mid_x, probe, 100.0, &mut scratch);
        assert!((t - h).abs() < 1e-9, "drape {t} and ground {h} must agree");
    }
}
