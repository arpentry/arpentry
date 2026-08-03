//! Earthwork modifiers — the local reshapings of the natural terrain that the
//! solved model implies (docs/GENERATION.md §6 stage 3, D3).
//!
//! Every at-grade stretch of a solved corridor benches the ground to its road
//! height — a grade-limited cut through a bump, the embankment ramp climbing
//! to an overpass, or simply the flat band under a road the DEM already
//! carries. Each earthwork is a chain of [`EarthworkEdge`]s along the corridor
//! centerline carrying the target (road) height.
//!
//! The rule is the physical one: **inside a bench the ground *is* the road**,
//! and outside it the natural ground is clamped by the earthwork's batter
//! face. So a query point resolves in two steps:
//!
//! 1. *Benches win.* If the point lies within any bench's held half-width the
//!    ground is that bench's target — the nearest bench, with a deterministic
//!    tie-break, never a mean of several. Averaging inside asphalt is what let
//!    a motorway approach fill pull the ground 12 m up over the road running
//!    beneath it, and what domed the terrain through wide paved areas. Where
//!    two benches at very different heights abut, the field steps — which is
//!    what a retaining wall between an underpass and a 12 m embankment is.
//!    The step lands on a crest contact line ([`super::breaklines`]), so the
//!    mesh draws it as a face rather than smearing it across a cell.
//!
//!    Nearest, but *a road's own carriageway first*: where two roads run closer
//!    together than their benches are wide, the neighbour is the nearer bench
//!    over part of the road's own asphalt, and the step would then fall
//!    underneath the drawn surface — a wall across the kerb. A bench holds its
//!    carriageway outright ([`EarthworkEdge::carriageway_m`]) and proximity
//!    decides only in the verge beyond it, so the step always lands between two
//!    carriageways rather than inside one.
//! 2. *Batters clamp.* Outside every bench the ground is the natural height
//!    bounded by the straight batter faces that reach it: no lower than the
//!    highest fill face, no higher than the lowest cut face. Self-limiting —
//!    where the natural ground already lies inside the batter cone nothing
//!    moves, so an earthwork stops exactly where it daylights instead of
//!    reshaping a fixed-width corridor around every road.
//!
//! Both steps are pure functions of the query point over a deterministically
//! ordered edge set, so any two tiles (and any two zooms) derive identical
//! ground (invariant 5).

use geo_types::Coord;

use crate::assemble::grid::GridIndex;
use crate::priors::EARTHWORK_BATTER;
use crate::scene::DEG_M;

// **Reconstructing the step between stacked platforms: tried, and it tears.**
//
// `data/plans/probes/probe_stack.rs` measures that among crowded stacks the
// median DEM carries only 84 % of the vertical separation the two solved
// profiles carry, and under half in 29 % of them. The alignments are the better
// vertical control there, so the ground between two stacked benches ought to
// ramp between *their* targets rather than take the generalized surface.
//
// Implemented as "find the two nearest benches bracketing this point from
// opposite sides and interpolate between their targets by distance", it made
// four metrics worse at once: `slope.terrain_tearing` worst 6.46 → 9.24 m,
// `contact.kerb_lip` 14.79 → 16.31 m, plus the terrain-face and deck-clearance
// tails. The cause is structural, not a threshold: *which pair brackets a point*
// is not continuous in the point, so two neighbouring lattice vertices can pick
// different pairs and the field steps between them — which is precisely the
// alternation `slope.terrain_tearing` exists to catch, and it caught it.
//
// A formulation that could work has to be continuous by construction, and the
// existing machinery already is: let each bench's batter face reach *to the
// other bench's edge* rather than collapse where it cannot daylight, so the two
// faces meet each other. That is a `min`/`max` of planes like the clamps above,
// so it cannot tear. Not attempted yet.

/// Side indices into [`EarthworkEdge::batter_m`], left and right of the
/// directed edge `a → b` in the metric frame.
pub const LEFT: usize = 0;
pub const RIGHT: usize = 1;

/// One earthwork centerline edge: endpoints with target heights, the
/// road-height half-width, and the batter's reach beyond it.
#[derive(Debug, Clone, Copy)]
pub struct EarthworkEdge {
    pub a: Coord,
    pub b: Coord,
    pub target_a: f64,
    pub target_b: f64,
    /// Held at target within this lateral distance (carriageway + shoulder +
    /// verge), metres — the bench.
    pub half_width_m: f64,
    /// Half-width in metres of the *drawn asphalt* this bench carries — the
    /// paved band, inside the bench's own verge.
    ///
    /// Benches otherwise resolve by proximity, which is right between two roads
    /// and wrong inside one: where a road runs closer to a neighbour than its
    /// own bench is wide — a street under a railway embankment, a switchback
    /// above itself — the neighbour's bench is the nearer one over part of the
    /// road's own carriageway, and the ground there steps up to it *underneath
    /// the asphalt*. The drawn surface is then cut by a wall that belongs
    /// outside it. So a bench holds its own carriageway outright, and proximity
    /// decides only beyond it (docs/GROUND.md §2, "the ground under a road is
    /// the road"). Zero for a carve, which paves nothing.
    pub carriageway_m: f64,
    /// How far the batter face reaches beyond the bench on each side, metres,
    /// indexed `[left, right]` of the directed edge.
    ///
    /// Per side because a road on a hillside cuts into one flank and fills off
    /// the other, and each face daylights at its own distance. The reach is
    /// where the face is expected to meet the natural ground, derived from the
    /// cross-slope measured at the bench edges: where the ground falls (or
    /// rises) faster than the batter the face can never daylight, and the
    /// reach collapses to its floor — the bench is then retained by a wall at
    /// its edge, which is what a mountain road has, instead of a long terrace
    /// ending in a cliff out in the hillside.
    pub batter_m: [f64; 2],
    /// The earthwork run this edge belongs to (one id per corridor).
    pub chain: u32,
    /// Arc position of `a` along the chain, metres.
    pub arc0: f64,
    /// `cos(mean latitude)` of the source corridor, for the metric projection.
    pub cos_lat: f64,
    /// Cut-only: the edge may lower the ground to its target but never raise
    /// it — a portal daylighting cut must not build a berm where the natural
    /// ground already sits below the bore floor.
    pub carve: bool,
}

impl EarthworkEdge {
    /// The edge's reach on `side` (0 = left of `a → b`, 1 = right): past this
    /// lateral distance it cannot touch the ground there.
    fn reach_on(&self, side: usize) -> f64 {
        self.half_width_m + self.batter_m[side]
    }

    /// The edge's widest reach — its index footprint.
    pub fn reach_m(&self) -> f64 {
        self.half_width_m + self.batter_m[0].max(self.batter_m[1])
    }

    /// How far the batter face has left the bench height at lateral distance
    /// `d`: zero across the bench, then one metre per [`EARTHWORK_BATTER`]
    /// metres outward. The face is `target - rise` on the fill side and
    /// `target + rise` on the cut side.
    fn face_rise(&self, d: f64) -> f64 {
        (d - self.half_width_m).max(0.0) / EARTHWORK_BATTER
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
            let reach_deg = e.reach_m() / (DEG_M * e.cos_lat.min(1.0).max(0.1));
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

    /// The bench and batter contributions of the road earthworks covering
    /// `(lon, lat)` over natural ground `raw`: the nearest covering bench's
    /// target (`None` outside every bench), the highest embankment face and
    /// the lowest cutting face reaching the point.
    ///
    /// An edge works one side only, decided by where its road sits against the
    /// ground here: a road above it is carried on fill and its face may only
    /// raise the ground; a road below it is in a cutting and its face may only
    /// lower it. Without that gate a road in a cutting would shave the
    /// embankment of its neighbour ten metres away.
    ///
    /// The bench winner is the smallest lateral distance, ties broken by edge
    /// index — a total order over a deterministically built edge set, so every
    /// tile resolves the same winner (invariant 5) — except that a bench
    /// covering the point with its own *carriageway* outranks one that merely
    /// reaches it with its verge, however near (see
    /// [`EarthworkEdge::carriageway_m`]). Two ranks, each resolved by the same
    /// total order, so the answer is still a function of the model alone. The
    /// faces are extrema, not sums, so they too are order-independent.
    fn benches_and_faces(
        &self,
        lon: f64,
        lat: f64,
        raw: f64,
        cell_m: f64,
        scratch: &[u32],
    ) -> (Option<f64>, f64, f64) {
        // (d, idx, target) for the paved rank and the verge rank.
        let mut paved: Option<(f64, u32, f64)> = None;
        let mut bench: Option<(f64, u32, f64)> = None;
        let (mut fill, mut cut) = (f64::NEG_INFINITY, f64::INFINITY);
        for &i in scratch {
            let e = &self.edges[i as usize];
            if e.carve || e.half_width_m < cell_m {
                continue;
            }
            let (d, t, side) = lateral_distance(e, lon, lat);
            if d >= e.reach_on(side) {
                continue;
            }
            let target = e.target_a + (e.target_b - e.target_a) * t;
            if d <= e.half_width_m {
                let rank = if d <= e.carriageway_m { &mut paved } else { &mut bench };
                let better = match *rank {
                    None => true,
                    Some((bd, bi, _)) => d < bd - 1e-9 || ((d - bd).abs() <= 1e-9 && i < bi),
                };
                if better {
                    *rank = Some((d, i, target));
                }
                continue;
            }
            let rise = e.face_rise(d);
            if target > raw {
                fill = fill.max(target - rise);
            } else {
                cut = cut.min(target + rise);
            }
        }
        (paved.or(bench).map(|(_, _, target)| target), fill, cut)
    }

    /// The engineered height at `(lon, lat)` given the natural ground `raw`:
    /// the covering bench's target, else the natural ground clamped by the
    /// batter faces that reach it, then cut by any covering carve notch.
    ///
    /// Earthworks whose bench is narrower than `cell_m` — the sample spacing of
    /// the lattice asking — are skipped; see [`super::GroundModel::height`].
    pub fn height(
        &self,
        lon: f64,
        lat: f64,
        raw: f64,
        cell_m: f64,
        scratch: &mut Vec<u32>,
    ) -> f64 {
        self.grid.query((lon, lat, lon, lat), scratch);
        scratch.sort_unstable();
        let (bench, fill, cut) = self.benches_and_faces(lon, lat, raw, cell_m, scratch);
        // A bench is the road surface itself. Outside it the cuttings dig
        // first and the embankments are raised over them, so where a road in a
        // cutting sits beside a road on fill the embankment survives and the
        // cutting takes only the ground the embankment does not claim. Ground
        // no face reaches stays natural.
        let mut h = match bench {
            Some(target) => target,
            None => raw.min(cut).max(fill),
        };
        // Carves are cut-only notches applied to the benched ground: a notch
        // holds its floor across its own width and its wall rises at the
        // batter, so the deepest covering notch simply bounds the ground from
        // above. A hole is not a surface to average.
        for &i in scratch.iter() {
            let e = &self.edges[i as usize];
            if !e.carve || e.half_width_m < cell_m {
                continue;
            }
            let (d, t, side) = lateral_distance(e, lon, lat);
            if d >= e.reach_on(side) {
                continue;
            }
            let floor = e.target_a + (e.target_b - e.target_a) * t + e.face_rise(d);
            h = h.min(floor);
        }
        h
    }

    /// The exact roadbed height at `(lon, lat)` when the point lies inside a
    /// bench's held half-width, else `None` — outside a bench the ground is
    /// the natural surface under its batter and no road rides it. The same
    /// answer [`Earthworks::height`] resolves there, so the road stack and the
    /// drawn ground can never disagree where they overlap. The road drape
    /// rides this at the reference zoom (see `synth::road::surface_height`).
    /// Carve notches (deck daylighting, portal cuts) are not beds.
    pub fn target_at(&self, lon: f64, lat: f64, scratch: &mut Vec<u32>) -> Option<f64> {
        self.grid.query((lon, lat, lon, lat), scratch);
        scratch.sort_unstable();
        // The bench answer does not depend on the natural ground, so the
        // caller needs no DEM sample to ask what the road rides.
        self.benches_and_faces(lon, lat, f64::NAN, 0.0, scratch).0
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

/// Lateral distance in metres from `(lon, lat)` to the edge, the clamped
/// parameter along it, and which side of the directed edge the point lies on
/// (0 = left of `a → b`, 1 = right) — the side whose batter reach applies.
fn lateral_distance(e: &EarthworkEdge, lon: f64, lat: f64) -> (f64, f64, usize) {
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
    let side = if dx * (lat - e.a.y) - dy * (px - ax) >= 0.0 { LEFT } else { RIGHT };
    (dd, t, side)
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
            carriageway_m: 6.0,
            batter_m: [10.0; 2],
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
        assert!((ew.height(mid_x, 46.0, 110.0, 0.0, &mut scratch) - 105.0).abs() < 1e-9);
        // Ground already below the target: untouched (no berm).
        assert_eq!(ew.height(mid_x, 46.0, 100.0, 0.0, &mut scratch), 100.0);
    }

    #[test]
    fn pulls_ground_to_target_within_the_half_width() {
        let mut scratch = Vec::new();
        let e = Earthworks::new(vec![edge(105.0)]);
        // On the centerline: exactly the target.
        let mid_x = 6.0 + 80.0 / (DEG_M * 46.0_f64.to_radians().cos());
        assert!((e.height(mid_x, 46.0, 100.0, 0.0, &mut scratch) - 105.0).abs() < 1e-9);
        // 5 m off (inside half-width): still the target.
        let off = 5.0 / DEG_M;
        assert!((e.height(mid_x, 46.0 + off, 100.0, 0.0, &mut scratch) - 105.0).abs() < 1e-9);
    }

    /// Outside the bench the ground follows a straight batter face down to
    /// the natural ground and stops there — it neither hangs at road height
    /// nor reshapes ground the face never reaches.
    #[test]
    fn the_batter_is_a_straight_self_limiting_face() {
        let mut scratch = Vec::new();
        let e = Earthworks::new(vec![edge(105.0)]);
        let mid_x = 6.0 + 80.0 / (DEG_M * 46.0_f64.to_radians().cos());
        let h_at = |off_m: f64, raw: f64, scratch: &mut Vec<u32>| {
            e.height(mid_x, 46.0 + off_m / DEG_M, raw, 0.0, scratch)
        };
        // A 5 m embankment over flat ground at 100 m: the face leaves the
        // 8 m bench at 105 m and descends 1 m per EARTHWORK_BATTER metres.
        for off in [9.0, 10.0, 12.0] {
            let want = 105.0 - (off - 8.0) / EARTHWORK_BATTER;
            let got = h_at(off, 100.0, &mut scratch);
            assert!((got - want).abs() < 1e-9, "at {off} m want {want}, got {got}");
        }
        // It daylights where it meets the ground (5 m down = 12.5 m out) and
        // leaves everything beyond untouched.
        assert_eq!(h_at(21.0, 100.0, &mut scratch), 100.0);
        assert_eq!(h_at(30.0, 100.0, &mut scratch), 100.0);
        // Self-limiting: ground already above the descending face (here the
        // face is at 104.2) is left exactly as it is.
        assert_eq!(h_at(10.0, 104.5, &mut scratch), 104.5);
        // And the same face cuts when the ground stands above it.
        let want = 105.0 + (10.0 - 8.0) / EARTHWORK_BATTER;
        assert!((h_at(10.0, 120.0, &mut scratch) - want).abs() < 1e-9);
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

    /// A bench narrower than the lattice asking for the ground is left out of
    /// it: at that resolution the mesh cannot draw the bench, and sampling it
    /// only spikes whichever corners happen to land inside.
    #[test]
    fn a_bench_narrower_than_the_asking_lattice_is_skipped() {
        let mut scratch = Vec::new();
        let e = Earthworks::new(vec![edge(105.0)]); // 8 m half-width
        let mid_x = 6.0 + 80.0 / (DEG_M * 46.0_f64.to_radians().cos());
        // A detail lattice (or an exact query) sees the bench…
        assert!((e.height(mid_x, 46.0, 100.0, 0.0, &mut scratch) - 105.0).abs() < 1e-9);
        assert!((e.height(mid_x, 46.0, 100.0, 6.0, &mut scratch) - 105.0).abs() < 1e-9);
        // …a coarser one does not, and reads the natural ground instead.
        assert_eq!(e.height(mid_x, 46.0, 100.0, 13.0, &mut scratch), 100.0);
        assert_eq!(e.height(mid_x, 46.0 + 10.0 / DEG_M, 100.0, 13.0, &mut scratch), 100.0);
        // The road's own bed query is exact whatever the terrain is drawn at.
        assert_eq!(e.target_at(mid_x, 46.0, &mut scratch), Some(105.0));
    }

    /// Two roads side by side at very different heights — an underpass beside
    /// an approach embankment. Each carriageway keeps its *own* height across
    /// its whole bench (no averaging, so nothing domes up through the asphalt),
    /// the ground between them is the embankment's batter, and the road drape
    /// reads exactly the ground the terrain draws.
    #[test]
    /// A wide road with a narrow one seven metres off its axis, seven metres
    /// higher — an interchange ramp beside a service track, a street under a
    /// railway. Past the midpoint the narrow road's *verge* is the nearer
    /// bench, so by proximity alone the ground steps up seven metres over the
    /// outer metre of the wide road's own asphalt. Its carriageway must hold
    /// its own height across its full width; the step belongs in the verge.
    #[test]
    fn a_carriageway_holds_its_own_ground_against_a_nearer_verge() {
        let mut scratch = Vec::new();
        let cos_lat = 46.0_f64.to_radians().cos();
        let mut street = edge(400.0);
        street.half_width_m = 4.25;
        street.carriageway_m = 3.75;
        street.batter_m = [10.0; 2];
        let mut track = street;
        track.target_a = 407.0;
        track.target_b = 407.0;
        track.carriageway_m = 2.0; // a narrow way: its asphalt stops early
        track.a.y += 7.0 / DEG_M;
        track.b.y += 7.0 / DEG_M;
        track.chain = 1;
        let e = Earthworks::new(vec![street, track]);
        let mid_x = 6.0 + 80.0 / (DEG_M * cos_lat);
        let h_at = |off_m: f64, scratch: &mut Vec<u32>| {
            e.height(mid_x, 46.0 + off_m / DEG_M, 400.0, 0.0, scratch)
        };
        // Across the whole street carriageway — including 3.7 m, which is
        // nearer the track's bench (3.3 m) than its own axis.
        for off in [-3.7, -1.0, 0.0, 1.0, 2.5, 3.7] {
            let h = h_at(off, &mut scratch);
            assert!((h - 400.0).abs() < 1e-9, "street must hold 400 at {off} m, got {h}");
        }
        // The track keeps its own carriageway, so the wall stands between them.
        for off in [5.5, 7.0, 8.5] {
            let h = h_at(off, &mut scratch);
            assert!((h - 407.0).abs() < 1e-9, "the track must hold 407 at {off} m, got {h}");
        }
        // The drape reads the same answer, so road and ground cannot disagree.
        assert_eq!(e.target_at(mid_x, 46.0 + 3.7 / DEG_M, &mut scratch), Some(400.0));
    }

    #[test]
    fn each_bench_holds_its_own_road_against_its_neighbour() {
        let mut scratch = Vec::new();
        let cos_lat = 46.0_f64.to_radians().cos();
        // Street-like benches held to 7.75 m, centerlines 18 m apart: the
        // upper road's batter reaches across the gap and over the lower road.
        let mut lower = edge(103.0);
        lower.half_width_m = 7.75;
        lower.batter_m = [30.0; 2];
        let mut upper = lower;
        upper.target_a = 110.0;
        upper.target_b = 110.0;
        upper.a.y += 18.0 / DEG_M;
        upper.b.y += 18.0 / DEG_M;
        upper.chain = 1; // a distinct road (or the far leg of a hairpin)
        let e = Earthworks::new(vec![lower, upper]);
        let mid_x = 6.0 + 80.0 / (DEG_M * cos_lat);
        let h_at = |off_m: f64, scratch: &mut Vec<u32>| {
            e.height(mid_x, 46.0 + off_m / DEG_M, 100.0, 0.0, scratch)
        };
        // Every point of each bench holds that road's own height exactly: the
        // 7 m-higher neighbour 10 m away cannot lift the ground inside the
        // lower carriageway (the terrain-domes-through-the-asphalt failure).
        for off in [-7.5, -4.0, 0.0, 4.0, 7.5] {
            let h = h_at(off, &mut scratch);
            assert!((h - 103.0).abs() < 1e-9, "lower bench must hold 103 at {off} m, got {h}");
        }
        for off in [10.5, 14.0, 18.0, 25.5] {
            let h = h_at(off, &mut scratch);
            assert!((h - 110.0).abs() < 1e-9, "upper bench must hold 110 at {off} m, got {h}");
        }
        // Between the benches the ground is the upper road's batter face,
        // descending at EARTHWORK_BATTER toward the lower one.
        let gap = h_at(9.0, &mut scratch);
        let want = 110.0 - (18.0 - 9.0 - 7.75) / EARTHWORK_BATTER;
        assert!((gap - want).abs() < 1e-9, "the gap must be the batter, got {gap} want {want}");
        // Away from both, untouched natural ground.
        assert_eq!(h_at(60.0, &mut scratch), 100.0);
        // The road drape reads the same field the terrain draws.
        let probe = 46.0 + 6.5 / DEG_M;
        let t = e.target_at(mid_x, probe, &mut scratch).expect("inside a bench");
        let h = e.height(mid_x, probe, 100.0, 0.0, &mut scratch);
        assert!((t - h).abs() < 1e-9, "drape {t} and ground {h} must agree");
        // Outside every bench there is no bed to ride.
        assert_eq!(e.target_at(mid_x, 46.0 + 9.0 / DEG_M, &mut scratch), None);
    }
}
