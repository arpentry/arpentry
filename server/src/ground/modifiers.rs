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
//! Where several earthworks overlap (a junction), the *nearest* centerline
//! wins — a pure function of the query point and the fixed edge set, so any
//! two tiles (and any two zooms) derive identical ground (invariant 5).

use geo_types::Coord;

use crate::assemble::grid::GridIndex;
use crate::scene::DEG_M;

/// One earthwork centerline edge: endpoints with target heights, the
/// road-height half-width, and the slope reach beyond it.
#[derive(Debug, Clone, Copy)]
pub struct EarthworkEdge {
    pub a: Coord,
    pub b: Coord,
    pub target_a: f64,
    pub target_b: f64,
    /// Held at target within this lateral distance (road + shoulder), metres.
    pub half_width_m: f64,
    /// Smoothstep blend back to natural ground over this further distance.
    pub feather_m: f64,
    /// `cos(mean latitude)` of the source corridor, for the metric projection.
    pub cos_lat: f64,
    /// Cut-only: the edge may lower the ground to its target but never raise
    /// it — a portal daylighting cut must not build a berm where the natural
    /// ground already sits below the bore floor.
    pub carve: bool,
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

    /// The engineered height at `(lon, lat)` given the natural ground `raw`:
    /// the nearest covering earthwork's target, feather-blended into `raw`.
    pub fn height(&self, lon: f64, lat: f64, raw: f64, scratch: &mut Vec<u32>) -> f64 {
        self.grid.query((lon, lat, lon, lat), scratch);
        // The winning contribution: strongest blend weight, then nearest,
        // then lowest edge index — a total order, so the answer is identical
        // whatever tile asked.
        let mut best: Option<(f64, f64, u32, f64)> = None; // (weight, dist, idx, target)
        for &i in scratch.iter() {
            let e = &self.edges[i as usize];
            let (d, t) = lateral_distance(e, lon, lat);
            let w = if d <= e.half_width_m {
                1.0
            } else if d <= e.half_width_m + e.feather_m {
                let u = (d - e.half_width_m) / e.feather_m;
                1.0 - u * u * (3.0 - 2.0 * u) // smoothstep down
            } else {
                continue;
            };
            let target = e.target_a + (e.target_b - e.target_a) * t;
            if e.carve && target >= raw {
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
        match best {
            Some((w, _, _, target)) => raw + (target - raw) * w,
            None => raw,
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
    fn nearest_earthwork_wins_where_two_overlap() {
        let mut scratch = Vec::new();
        let cos_lat = 46.0_f64.to_radians().cos();
        let mut lower = edge(103.0);
        // A second, parallel earthwork 12 m north with a different target.
        let mut upper = edge(110.0);
        upper.a.y += 12.0 / DEG_M;
        upper.b.y += 12.0 / DEG_M;
        lower.feather_m = 20.0;
        upper.feather_m = 20.0;
        let e = Earthworks::new(vec![lower, upper]);
        let mid_x = 6.0 + 80.0 / (DEG_M * cos_lat);
        // 2 m north of the lower centerline: both cover it at full weight,
        // the nearer (lower) wins.
        let h = e.height(mid_x, 46.0 + 2.0 / DEG_M, 100.0, &mut scratch);
        assert!((h - 103.0).abs() < 1e-9, "nearest centerline must win, got {h}");
        // 2 m south of the upper centerline: the upper wins.
        let h = e.height(mid_x, 46.0 + 10.0 / DEG_M, 100.0, &mut scratch);
        assert!((h - 110.0).abs() < 1e-9, "nearest centerline must win, got {h}");
    }
}
