//! The solved vertical model of one corridor — the road's own (gentle)
//! elevation profile through its bridges and tunnels (docs/GENERATION.md §6
//! stage 2).
//!
//! The terrain along a corridor is wild — the Viaduc de Chillon's centerline
//! DEM drops into a ravine (a bridge spans it ~70 m up) and rears up to a
//! 680 m hill (a tunnel pierces it) — while the road itself holds a gentle
//! highway grade (~3 %). The road surface is therefore anchored to a
//! reconstructed *road profile*:
//!
//! 1. Sample the reference terrain along the whole corridor.
//! 2. Where the road is at grade it sits on the ground, so those nodes are
//!    elevation *anchors*: `profile = terrain`.
//! 3. Across every bridge/tunnel span, the profile is a straight (gentle)
//!    interpolation between the bounding anchors — the grade the road actually
//!    holds — independent of the terrain excursion under it.
//!
//! A structure thus rides the gentle profile: where the terrain dips below it
//! (a ravine) a bridge deck stands high above the floor — the visible viaduct;
//! where the terrain rises above it (a flank, a tunnelled hill) the body
//! passes below ground and the terrain, drawn first and owning the depth
//! buffer, occludes it.
//!
//! The profile is solved once, over the whole corridor, before tiling. Every
//! tile fragment of the corridor reads the same [`Profile`] through
//! `Arc<SolvedModel>`, so heights are a function of the global model only —
//! never of the tile window — and seams line up by construction
//! (docs/GENERATION.md invariant 5).

use geo_types::Coord;

use crate::priors::MAX_ROAD_DEVIATION_M;
use crate::scene::{metric_len, run_cos_lat, Span, SpanKind};

/// Target spacing in metres after densification, used both to sample the road
/// profile along the corridor and to subdivide swept geometry so it renders as
/// a smooth curve.
pub const NODE_SPACING_M: f64 = 8.0;

/// Cap on densified vertices per corridor — a runaway guard for pathological
/// inputs; real corridors are bounded by `priors::MAX_CORRIDOR_M`.
const MAX_NODES: usize = 65_536;

/// Binomial (1-2-1) smoothing passes applied to the centerline before
/// sweeping. Each pass widens the kernel; four damps the short digitising
/// wiggle of a road line (a footway's zigzag, a viaduct's vertex noise) while
/// keeping its real curve, so a swept box is a regular prism.
const SMOOTH_PASSES: usize = 4;

/// Relaxation passes for [`limit_road_grade`]. A handful alternating forward
/// and backward spreads a steep pitch's deviation evenly — a cut on the way
/// up, a fill on the way down — instead of letting one anchored direction
/// drift to one side.
const GRADE_PASSES: usize = 8;

/// A corridor's solved surface profile: a densified centerline with a per-node
/// road-surface height. Evaluated at an arbitrary point by projecting it onto
/// the corridor, so independent tile fragments of one structure compute the
/// same heights and the seams line up.
///
/// This surface is the single road model: a bridge deck and a tunnel bore both
/// ride it (their road face *is* this surface), so where a bridge meets a
/// tunnel or an approach road the road is continuous (invariant 2).
pub struct Profile {
    /// Densified corridor nodes in (lon, lat).
    nodes: Vec<Coord>,
    /// `nodes` low-pass-smoothed (endpoint-preserving), the line a deck box is
    /// swept along so it follows the road without tracing every digitising
    /// wiggle.
    smooth: Vec<Coord>,
    /// Cumulative metric arc length at each node (`arc[0] == 0`).
    arc: Vec<f64>,
    /// Road-surface height in metres above the ellipsoid at each node.
    road_m: Vec<f64>,
    /// Deck-top height in metres at each node: [`road_m`](Self::road_m) with
    /// each structure span replaced by a single straight ramp fit over that
    /// span (the at-grade nodes keep the draped road height). This is the
    /// height swept deck/bore boxes ride: one ramp per *global* span, so no
    /// tile sliver fits its own steep line and no seam steps.
    deck_m: Vec<f64>,
    /// Reference terrain height in metres at each node — the same samples the
    /// at-grade anchors are built from. `road_m − terrain_m` is the signed gap
    /// that says where the road stands proud (a visible deck) or runs buried
    /// (a bore); a tunnel's portal is exactly that gap's zero crossing.
    terrain_m: Vec<f64>,
    /// Whether each node lies in an at-grade span (an anchor) or a structure.
    at_grade: Vec<bool>,
    /// `cos(mean latitude)`, scaling longitude into the local metric space.
    cos_lat: f64,
}

/// One swept deck cross-section: the (smoothed) centerline position, the
/// deck-top height, the unit left-perpendicular (ENU metres) the section
/// spans, and the *global* corridor arc it sits at — the reference pier
/// placement snaps to, so tile fragments of one viaduct plant identical
/// piers.
pub struct DeckNode {
    pub lon: f64,
    pub lat: f64,
    pub height_m: f64,
    pub left_e: f64,
    pub left_n: f64,
    pub arc_m: f64,
}

/// Solves the surface profile of one corridor: densify, sample the reference
/// terrain through `elev`, anchor the road at the at-grade spans, interpolate
/// the gentle profile across the structures, and hold engineered classes to
/// their grade ceiling. `None` for a degenerate corridor. The terrain sampler
/// is injected so tests can bypass the DEM.
pub fn solve(
    nodes: &[Coord],
    spans: &[Span],
    max_grade: Option<f64>,
    elev: &mut dyn FnMut(Coord) -> f64,
) -> Option<Profile> {
    if nodes.len() < 2 {
        return None;
    }
    let cos_lat = run_cos_lat(nodes);
    let (nodes, arc) = densify(nodes, cos_lat);
    let n = nodes.len();
    if n < 2 {
        return None;
    }
    let terrain: Vec<f64> = nodes.iter().map(|c| elev(*c)).collect();
    let at_grade: Vec<bool> = arc.iter().map(|&a| kind_at(spans, a) == SpanKind::Grade).collect();
    let mut road_m = road_profile(&arc, &terrain, &at_grade);
    if let Some(g) = max_grade {
        limit_road_grade(&arc, &mut road_m, &terrain, &at_grade, g);
    }
    let deck_m = deck_ramp(&arc, &road_m, &at_grade);
    let smooth = smooth_path(&nodes);
    Some(Profile { nodes, smooth, arc, road_m, deck_m, terrain_m: terrain, at_grade, cos_lat })
}

/// The span kind at arc position `a` (grade when no span covers it).
fn kind_at(spans: &[Span], a: f64) -> SpanKind {
    spans
        .iter()
        .find(|s| a >= s.arc0 && a <= s.arc1)
        .map_or(SpanKind::Grade, |s| s.kind)
}

impl Profile {
    /// Road-surface height in metres above the ellipsoid at `(lon, lat)`,
    /// found by projecting the point onto the nearest corridor edge and
    /// interpolating. Clipped fragment vertices all lie on the corridor, so
    /// the nearest on-corridor height is exact.
    pub fn height_at(&self, lon: f64, lat: f64) -> f64 {
        project_onto(&self.nodes, &self.road_m, self.cos_lat, lon, lat)
    }

    /// Reference terrain height in metres at `(lon, lat)`, projected onto the
    /// corridor exactly as [`height_at`](Self::height_at) projects the road,
    /// so the two share a reference and `height_at − surface_at` is a clean
    /// signed gap. A tunnel bore is the span where this gap is negative; its
    /// portal is the zero crossing where the road emerges.
    pub fn surface_at(&self, lon: f64, lat: f64) -> f64 {
        project_onto(&self.nodes, &self.terrain_m, self.cos_lat, lon, lat)
    }

    /// Deck cross-sections for a structure's (clipped, densified, in-order)
    /// centerline `pts`: each vertex placed on the *smoothed* global road line
    /// at the [`deck_m`](Self::deck_m) deck height, with a smoothed
    /// cross-section direction, so the swept box is a regular prism that
    /// follows the road instead of tracing every wiggle and dive.
    pub fn deck_nodes(&self, pts: &[Coord]) -> Vec<DeckNode> {
        self.walk(pts)
            .iter()
            .map(|&(i, t)| {
                let height_m = lerp(self.deck_m[i], self.deck_m[i + 1], t);
                let lon = lerp(self.smooth[i].x, self.smooth[i + 1].x, t);
                let lat = lerp(self.smooth[i].y, self.smooth[i + 1].y, t);
                let (left_e, left_n) = self.section_left(i);
                let arc_m = lerp(self.arc[i], self.arc[i + 1], t);
                DeckNode { lon, lat, height_m, left_e, left_n, arc_m }
            })
            .collect()
    }

    /// Deck-top heights only — the height half of
    /// [`deck_nodes`](Self::deck_nodes), kept for direct testing.
    pub fn deck_line(&self, pts: &[Coord]) -> Vec<f64> {
        self.walk(pts).iter().map(|&(i, t)| lerp(self.deck_m[i], self.deck_m[i + 1], t)).collect()
    }

    /// Projects an in-order on-corridor polyline onto the profile, returning
    /// the `(edge, t)` of each vertex. The walk is monotonic from a robust
    /// interior seed, so a vertex is confined to the arc its neighbours sit on
    /// — a curving corridor that nears itself in plan can't snap a vertex onto
    /// a far arc. A clipped fragment may run either way; the direction is read
    /// from two interior points (the ends are where a self-approach lurks).
    fn walk(&self, pts: &[Coord]) -> Vec<(usize, f64)> {
        let edges = self.nodes.len().saturating_sub(1);
        if edges < 2 || pts.len() < 3 {
            return pts.iter().map(|p| self.project(0, edges.max(1), *p)).collect();
        }
        let ia = self.project(0, edges, pts[pts.len() / 3]).0;
        let ib = self.project(0, edges, pts[2 * pts.len() / 3]).0;
        let dir: isize = if ib >= ia { 1 } else { -1 };
        // The cursor may range ~6 edges per step: enough slack for one deck
        // step (about one profile edge) while still walling off a far arc.
        const WIN: isize = 6;
        let step = |cur: isize, towards: isize, p: Coord| -> (usize, f64) {
            let (lo, hi) = if towards >= 0 {
                (cur.max(0), (cur + WIN + 1).min(edges as isize))
            } else {
                ((cur - WIN).max(0), (cur + 1).min(edges as isize))
            };
            self.project(lo as usize, hi as usize, p)
        };
        let mid = pts.len() / 2;
        let mut out = vec![(0usize, 0.0); pts.len()];
        out[mid] = self.project(0, edges, pts[mid]);
        let mut cur = out[mid].0 as isize;
        for k in mid + 1..pts.len() {
            out[k] = step(cur, dir, pts[k]);
            cur = out[k].0 as isize;
        }
        cur = out[mid].0 as isize;
        for k in (0..mid).rev() {
            out[k] = step(cur, -dir, pts[k]);
            cur = out[k].0 as isize;
        }
        out
    }

    /// Unit left-perpendicular (ENU, scaled-degree space) of the smoothed line
    /// at edge `i`, read over a short window so the cross-section direction
    /// varies gently and box edges stay clean.
    fn section_left(&self, i: usize) -> (f64, f64) {
        let m = self.smooth.len();
        let lo = i.saturating_sub(2);
        let hi = (i + 3).min(m - 1);
        let de = (self.smooth[hi].x - self.smooth[lo].x) * self.cos_lat;
        let dn = self.smooth[hi].y - self.smooth[lo].y;
        let len = (de * de + dn * dn).sqrt().max(1e-12);
        (-dn / len, de / len)
    }

    /// Nearest edge to `p` over `[lo, hi)` and the clamped parameter along it.
    fn project(&self, lo: usize, hi: usize, p: Coord) -> (usize, f64) {
        nearest_edge(&self.nodes, self.cos_lat, lo, hi, p)
    }

    /// The densified nodes, for stage-artifact dumps and the ground stage.
    pub fn nodes(&self) -> &[Coord] {
        &self.nodes
    }

    /// Per-node solved road heights.
    pub fn road_m(&self) -> &[f64] {
        &self.road_m
    }

    /// Per-node reference terrain heights.
    pub fn terrain_m(&self) -> &[f64] {
        &self.terrain_m
    }

    /// Per-node at-grade flags (false inside a structure span).
    pub fn at_grade(&self) -> &[bool] {
        &self.at_grade
    }

    /// Per-node cumulative arc, metres.
    pub fn arc(&self) -> &[f64] {
        &self.arc
    }

    /// The centerline point at arc position `a`, interpolated in its edge.
    pub fn point_at_arc(&self, a: f64) -> Coord {
        let (i, t) = self.edge_at_arc(a);
        Coord {
            x: self.nodes[i].x + (self.nodes[i + 1].x - self.nodes[i].x) * t,
            y: self.nodes[i].y + (self.nodes[i + 1].y - self.nodes[i].y) * t,
        }
    }

    /// The solved road height at arc position `a`.
    pub fn road_at_arc(&self, a: f64) -> f64 {
        let (i, t) = self.edge_at_arc(a);
        self.road_m[i] + (self.road_m[i + 1] - self.road_m[i]) * t
    }

    /// The edge index and parameter containing arc position `a`.
    fn edge_at_arc(&self, a: f64) -> (usize, f64) {
        let n = self.nodes.len();
        let a = a.clamp(0.0, *self.arc.last().unwrap_or(&0.0));
        let i = match self.arc.binary_search_by(|v| v.partial_cmp(&a).expect("finite arc")) {
            Ok(i) => i.min(n - 2),
            Err(i) => i.saturating_sub(1).min(n - 2),
        };
        let span = self.arc[i + 1] - self.arc[i];
        let t = if span > 0.0 { (a - self.arc[i]) / span } else { 0.0 };
        (i, t)
    }

    /// The corridor arc position (metres) nearest to `(lon, lat)`.
    pub fn arc_of(&self, lon: f64, lat: f64) -> f64 {
        let (i, t) = nearest_edge(
            &self.nodes,
            self.cos_lat,
            0,
            self.nodes.len().saturating_sub(1),
            Coord { x: lon, y: lat },
        );
        self.arc[i] + (self.arc[i + 1] - self.arc[i]) * t
    }

    /// Deck-top height at `(lon, lat)` — what a swept structure box renders.
    pub fn deck_height_at(&self, lon: f64, lat: f64) -> f64 {
        project_onto(&self.nodes, &self.deck_m, self.cos_lat, lon, lat)
    }

    /// Raises the road surface to meet a clearance at arc position `arc0`: a
    /// flat top holding `peak_m` across the whole structure span containing
    /// `arc0`, with shoulders falling away at `grade` beyond the span edges
    /// until they meet the existing profile — the approach ramps. Raise-only
    /// (`max`), so stacked constraints compose and nothing is ever pushed
    /// down (docs/GENERATION.md invariant 3 — clearance is a one-sided
    /// inequality), and holding the *span* flat (not a tent at the point)
    /// keeps the profile continuous at the abutments and survives the
    /// per-span ramp fit.
    pub fn raise_tent(&mut self, arc0: f64, peak_m: f64, grade: f64) {
        let n = self.road_m.len();
        if n == 0 {
            return;
        }
        // The flat-top interval: the structure run containing arc0, or the
        // point itself when arc0 falls at grade (annotation slop).
        let k = match self.arc.binary_search_by(|a| a.partial_cmp(&arc0).expect("finite arc")) {
            Ok(i) => i,
            Err(i) => i.min(n - 1),
        };
        let (mut lo_arc, mut hi_arc) = (arc0, arc0);
        if !self.at_grade[k] {
            let mut lo = k;
            while lo > 0 && !self.at_grade[lo - 1] {
                lo -= 1;
            }
            let mut hi = k;
            while hi + 1 < n && !self.at_grade[hi + 1] {
                hi += 1;
            }
            lo_arc = self.arc[lo];
            hi_arc = self.arc[hi];
        }
        for i in 0..n {
            let d = if self.arc[i] < lo_arc {
                lo_arc - self.arc[i]
            } else if self.arc[i] > hi_arc {
                self.arc[i] - hi_arc
            } else {
                0.0
            };
            let want = peak_m - grade * d;
            if want > self.road_m[i] {
                self.road_m[i] = want;
            }
        }
    }

    /// The mirror of [`raise_tent`](Self::raise_tent): sinks the road surface
    /// to at most `floor_m` across the whole structure span containing
    /// `arc0`, with shoulders *rising* back at `grade` beyond the span edges
    /// — the descent ramps of an underpass (scenario S6). Lower-only (`min`),
    /// so it composes with other constraints and never lifts anything.
    pub fn sink_tent(&mut self, arc0: f64, floor_m: f64, grade: f64) {
        let n = self.road_m.len();
        if n == 0 {
            return;
        }
        let k = match self.arc.binary_search_by(|a| a.partial_cmp(&arc0).expect("finite arc")) {
            Ok(i) => i,
            Err(i) => i.min(n - 1),
        };
        let (mut lo_arc, mut hi_arc) = (arc0, arc0);
        if !self.at_grade[k] {
            let mut lo = k;
            while lo > 0 && !self.at_grade[lo - 1] {
                lo -= 1;
            }
            let mut hi = k;
            while hi + 1 < n && !self.at_grade[hi + 1] {
                hi += 1;
            }
            lo_arc = self.arc[lo];
            hi_arc = self.arc[hi];
        }
        for i in 0..n {
            let d = if self.arc[i] < lo_arc {
                lo_arc - self.arc[i]
            } else if self.arc[i] > hi_arc {
                self.arc[i] - hi_arc
            } else {
                0.0
            };
            let want = floor_m + grade * d;
            if want < self.road_m[i] {
                self.road_m[i] = want;
            }
        }
    }

    /// The sinking counterpart of [`raise_span_to`](Self::raise_span_to):
    /// lowers the whole structure span containing `arc0` so its deck is at
    /// most `max_deck_m` there.
    pub fn lower_span_to(&mut self, arc0: f64, max_deck_m: f64) {
        let n = self.road_m.len();
        if n == 0 {
            return;
        }
        let k = match self.arc.binary_search_by(|a| a.partial_cmp(&arc0).expect("finite arc")) {
            Ok(i) => i,
            Err(i) => i.min(n - 1),
        };
        if self.at_grade[k] {
            return;
        }
        let excess = self.deck_m[k] - max_deck_m;
        if excess <= 0.0 {
            return;
        }
        let mut lo = k;
        while lo > 0 && !self.at_grade[lo - 1] {
            lo -= 1;
        }
        let mut hi = k;
        while hi + 1 < n && !self.at_grade[hi + 1] {
            hi += 1;
        }
        for i in lo..=hi {
            self.deck_m[i] -= excess;
            self.road_m[i] -= excess;
        }
    }

    /// Refits the per-span deck ramps after the road surface changed
    /// ([`raise_tent`](Self::raise_tent) / [`sink_tent`](Self::sink_tent)).
    pub fn rebuild_deck(&mut self) {
        self.deck_m = deck_ramp(&self.arc, &self.road_m, &self.at_grade);
    }

    /// Raises the whole structure span containing `arc0` so its deck reaches
    /// at least `min_deck_m` there — the terminal clamp that *guarantees* a
    /// clearance the ramp fit may have smoothed away. A no-op when `arc0`
    /// falls on an at-grade node (nothing to clamp; the tent already moved
    /// the road itself).
    pub fn raise_span_to(&mut self, arc0: f64, min_deck_m: f64) {
        let n = self.road_m.len();
        if n == 0 {
            return;
        }
        let k = match self.arc.binary_search_by(|a| a.partial_cmp(&arc0).expect("finite arc")) {
            Ok(i) => i,
            Err(i) => i.min(n - 1),
        };
        if self.at_grade[k] {
            return;
        }
        let deficit = min_deck_m - self.deck_m[k];
        if deficit <= 0.0 {
            return;
        }
        let mut lo = k;
        while lo > 0 && !self.at_grade[lo - 1] {
            lo -= 1;
        }
        let mut hi = k;
        while hi + 1 < n && !self.at_grade[hi + 1] {
            hi += 1;
        }
        for i in lo..=hi {
            self.deck_m[i] += deficit;
            self.road_m[i] += deficit;
        }
    }

    /// A flat profile holding `height_m` over the given centerline — a DEM-free
    /// constructor for tests and degenerate inputs.
    pub fn flat(nodes: &[Coord], height_m: f64) -> Profile {
        let n = nodes.len();
        Profile {
            cos_lat: run_cos_lat(nodes),
            arc: cumulative(nodes),
            smooth: smooth_path(nodes),
            nodes: nodes.to_vec(),
            road_m: vec![height_m; n],
            // A flat profile is a single height, so the deck ramp is it too.
            deck_m: vec![height_m; n],
            // No terrain relief: the ground sits on the road, gap zero
            // everywhere (no buried span, no bore).
            terrain_m: vec![height_m; n],
            at_grade: vec![true; n],
        }
    }

    /// Test constructor: a profile over `nodes` with explicit per-node road and
    /// terrain heights, so a bore's buried span and portal crossings can be set
    /// up deterministically without a DEM.
    #[cfg(test)]
    pub fn from_heights(nodes: &[Coord], road_m: Vec<f64>, terrain_m: Vec<f64>) -> Profile {
        // Tests supply the road heights they want a deck to ride directly, so
        // the deck ramp is the road profile as given (no span-splitting here).
        let deck_m = road_m.clone();
        let n = nodes.len();
        Profile {
            cos_lat: run_cos_lat(nodes),
            arc: cumulative(nodes),
            smooth: smooth_path(nodes),
            nodes: nodes.to_vec(),
            road_m,
            deck_m,
            terrain_m,
            at_grade: vec![false; n],
        }
    }
}

/// Cumulative metric arc length at each node.
fn cumulative(nodes: &[Coord]) -> Vec<f64> {
    let cos_lat = run_cos_lat(nodes);
    let mut arc = Vec::with_capacity(nodes.len());
    let mut acc = 0.0;
    for (i, &c) in nodes.iter().enumerate() {
        if i > 0 {
            acc += metric_len(nodes[i - 1], c, cos_lat);
        }
        arc.push(acc);
    }
    arc
}

/// Linear interpolation between `a` and `b` at `t`.
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Endpoint-preserving binomial (1-2-1) smoothing of a centerline,
/// [`SMOOTH_PASSES`] times, damping short digitising wiggle while keeping the
/// road's real curve.
fn smooth_path(nodes: &[Coord]) -> Vec<Coord> {
    let mut cur = nodes.to_vec();
    let n = cur.len();
    if n < 3 {
        return cur;
    }
    for _ in 0..SMOOTH_PASSES {
        let prev = cur.clone();
        for i in 1..n - 1 {
            cur[i] = Coord {
                x: 0.25 * prev[i - 1].x + 0.5 * prev[i].x + 0.25 * prev[i + 1].x,
                y: 0.25 * prev[i - 1].y + 0.5 * prev[i].y + 0.25 * prev[i + 1].y,
            };
        }
    }
    cur
}

/// Fits one straight ramp to arc-referenced heights and returns the fitted
/// value at each arc. The central span is least-squares-fit (the ends are
/// trimmed: a structure's busy landings — abutment touchdown, portal stub —
/// must not tilt the line). For a single chord the fit recovers it exactly, so
/// tile fragments of one span share the line.
fn fit_ramp(s: &[f64], h: &[f64]) -> Vec<f64> {
    let n = s.len();
    if n < 4 {
        return h.to_vec();
    }
    let cut = (n / 6).max(1);
    let (lo, hi) = (cut, n - cut);
    let m = (hi - lo) as f64;
    let (mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0);
    for k in lo..hi {
        sx += s[k];
        sy += h[k];
        sxx += s[k] * s[k];
        sxy += s[k] * h[k];
    }
    let denom = m * sxx - sx * sx;
    if denom.abs() < f64::EPSILON {
        // Degenerate arc spread (a near-point piece): hold the mean height.
        return vec![sy / m; n];
    }
    let b = (m * sxy - sx * sy) / denom;
    let a = (sy - b * sx) / m;
    s.iter().map(|&si| a + b * si).collect()
}

/// Value at `(lon, lat)` from a per-node series, found by projecting the point
/// onto the nearest corridor edge of `nodes` and interpolating.
fn project_onto(nodes: &[Coord], vals: &[f64], cos_lat: f64, lon: f64, lat: f64) -> f64 {
    let (i, t) =
        nearest_edge(nodes, cos_lat, 0, nodes.len().saturating_sub(1), Coord { x: lon, y: lat });
    vals[i] + (vals[i + 1] - vals[i]) * t
}

/// Nearest edge to `p` over the edge index range `[lo, hi)` (edge `i` spans
/// `nodes[i]..nodes[i+1]`), returning the edge index and the clamped parameter
/// `t` of the foot of the perpendicular. Longitudes are scaled by `cos_lat`
/// into the local metric space. A bounded range lets the arc-order walk
/// confine the search to one arc; `lo = 0, hi = edges` makes it a full scan.
fn nearest_edge(nodes: &[Coord], cos_lat: f64, lo: usize, hi: usize, p: Coord) -> (usize, f64) {
    let px = p.x * cos_lat;
    let py = p.y;
    let mut best_d2 = f64::INFINITY;
    let mut best_i = lo.min(nodes.len().saturating_sub(2));
    let mut best_t = 0.0;
    for i in lo..hi {
        let (a, b) = (nodes[i], nodes[i + 1]);
        let ax = a.x * cos_lat;
        let dx = b.x * cos_lat - ax;
        let dy = b.y - a.y;
        let len2 = dx * dx + dy * dy;
        let t = if len2 > 0.0 {
            (((px - ax) * dx + (py - a.y) * dy) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let cx = ax + dx * t;
        let cy = a.y + dy * t;
        let d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy);
        if d2 < best_d2 {
            best_d2 = d2;
            best_i = i;
            best_t = t;
        }
    }
    (best_i, best_t)
}

/// The road elevation at each node: terrain at the anchors (at-grade nodes and
/// structure boundaries), and a straight interpolation between the bounding
/// anchors across each structure. The corridor endpoints are always anchors,
/// so every node is bracketed.
fn road_profile(arc: &[f64], terrain: &[f64], anchor: &[bool]) -> Vec<f64> {
    let n = terrain.len();

    // Nearest anchor (arc, elevation) at-or-before and at-or-after each node,
    // in single forward/backward passes.
    let mut prev = vec![None; n];
    let mut last: Option<(f64, f64)> = None;
    for i in 0..n {
        if anchor[i] {
            last = Some((arc[i], terrain[i]));
        }
        prev[i] = last;
    }
    let mut next = vec![None; n];
    let mut coming: Option<(f64, f64)> = None;
    for i in (0..n).rev() {
        if anchor[i] {
            coming = Some((arc[i], terrain[i]));
        }
        next[i] = coming;
    }

    (0..n)
        .map(|i| {
            if anchor[i] {
                return terrain[i];
            }
            match (prev[i], next[i]) {
                (Some((sa, ta)), Some((sb, tb))) if sb > sa => {
                    ta + (tb - ta) * (arc[i] - sa) / (sb - sa)
                }
                (Some((_, t)), _) | (_, Some((_, t))) => t,
                (None, None) => {
                    // No at-grade anchor anywhere: chord the corridor endpoints.
                    let span = (arc[n - 1] - arc[0]).max(f64::MIN_POSITIVE);
                    terrain[0] + (terrain[n - 1] - terrain[0]) * (arc[i] - arc[0]) / span
                }
            }
        })
        .collect()
}

/// Holds the at-grade road to an engineered grade (`max_grade`) while keeping
/// it within [`MAX_ROAD_DEVIATION_M`] of the terrain. It flattens the steep
/// flanks the draped ground throws up — the dive into a bridge abutment, a
/// rolling bump — onto gentle cuttings and embankments, but never drifts far
/// from the ground: where the terrain climbs faster than the grade allows over
/// a long stretch, the road follows the slope (steeper, but hugging the ground
/// and visible) instead of flying off it into a phantom viaduct or burying
/// itself under a hill (S9).
///
/// Structure nodes are *pinned* (a bridge deck / tunnel bore already rides the
/// gentle reconstructed ramp) and so anchor the limiter — the steep approach
/// beside a structure is pulled to the structure's grade, the embankment that
/// reaches the abutment. The deviation clamp is applied last, so the bound on
/// cut/fill depth always holds (the grade is then best-effort — the
/// ground-hugging budget wins a conflict).
fn limit_road_grade(
    arc: &[f64],
    road_m: &mut [f64],
    terrain: &[f64],
    at_grade: &[bool],
    max_grade: f64,
) {
    let n = road_m.len();
    if n < 2 {
        return;
    }
    let to_terrain = |road_m: &mut [f64], i: usize| {
        road_m[i] =
            road_m[i].clamp(terrain[i] - MAX_ROAD_DEVIATION_M, terrain[i] + MAX_ROAD_DEVIATION_M);
    };
    let to_grade = |road_m: &mut [f64], i: usize, nb: usize| {
        let cap = max_grade * (arc[i] - arc[nb]).abs();
        road_m[i] = road_m[i].clamp(road_m[nb] - cap, road_m[nb] + cap);
    };
    for pass in 0..=GRADE_PASSES {
        // The last pass is always forward; each node is grade-clamped then
        // pulled back inside the deviation budget, so that bound holds on exit.
        if pass % 2 == 0 || pass == GRADE_PASSES {
            for i in 1..n {
                if at_grade[i] {
                    to_grade(road_m, i, i - 1);
                    to_terrain(road_m, i);
                }
            }
        } else {
            for i in (0..n - 1).rev() {
                if at_grade[i] {
                    to_grade(road_m, i, i + 1);
                    to_terrain(road_m, i);
                }
            }
        }
    }
}

/// The deck-top height at each node: [`road_profile`]'s heights with every
/// structure span (a maximal run of non-at-grade nodes) replaced by a single
/// straight ramp fit over that span and its bounding anchors. The at-grade
/// nodes keep their draped road height, so a deck meets the ground exactly at
/// an abutment.
fn deck_ramp(arc: &[f64], road_m: &[f64], at_grade: &[bool]) -> Vec<f64> {
    let n = road_m.len();
    let mut deck = road_m.to_vec();
    let mut i = 0;
    while i < n {
        if at_grade[i] {
            i += 1;
            continue;
        }
        // Maximal structure run [start, end); include the bounding anchors so
        // the ramp is fit to (and lands on) the road's true elevation at each
        // end.
        let start = i;
        while i < n && !at_grade[i] {
            i += 1;
        }
        let lo = start.saturating_sub(1);
        let hi = i.min(n - 1);
        let fitted = fit_ramp(&arc[lo..=hi], &road_m[lo..=hi]);
        for (k, &v) in (lo..=hi).zip(fitted.iter()) {
            // Overwrite only the structure nodes; at-grade anchors stay draped.
            if !at_grade[k] {
                deck[k] = v;
            }
        }
    }
    deck
}

/// Densifies a corridor to ~[`NODE_SPACING_M`] spacing, returning the nodes and
/// their cumulative metric arc length.
fn densify(run: &[Coord], cos_lat: f64) -> (Vec<Coord>, Vec<f64>) {
    let mut nodes = vec![run[0]];
    let mut arc = vec![0.0];
    let mut total = 0.0;
    for w in run.windows(2) {
        let (p0, p1) = (w[0], w[1]);
        let n = ((metric_len(p0, p1, cos_lat) / NODE_SPACING_M).ceil() as usize).clamp(1, MAX_NODES);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let c = Coord { x: p0.x + (p1.x - p0.x) * t, y: p0.y + (p1.y - p0.y) * t };
            total += metric_len(*nodes.last().expect("seeded"), c, cos_lat);
            nodes.push(c);
            arc.push(total);
        }
        if nodes.len() >= MAX_NODES {
            break;
        }
    }
    (nodes, arc)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::DEG_M;

    /// A structure span in corridor arc metres.
    fn span(arc0: f64, arc1: f64, level: i64) -> Span {
        let kind = match level.signum() {
            1 => SpanKind::Bridge,
            -1 => SpanKind::Tunnel,
            _ => SpanKind::Grade,
        };
        Span { arc0, arc1, level, kind }
    }

    /// A run of `n` evenly spaced vertices over `span_deg` of longitude at
    /// lat 46, and its total metric length.
    fn line(n: usize, span_deg: f64) -> (Vec<Coord>, f64) {
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord { x: 6.0 + span_deg * i as f64 / (n - 1) as f64, y: 46.0 })
            .collect();
        let len_m = span_deg * 46.0_f64.to_radians().cos() * DEG_M;
        (nodes, len_m)
    }

    /// Solves a profile with an injected terrain sampler, bypassing the DEM.
    fn profile_from(seg: &[Coord], spans: &[Span], terrain: impl Fn(Coord) -> f64) -> Profile {
        let mut elev = |c: Coord| terrain(c);
        solve(seg, spans, None, &mut elev).expect("non-degenerate test corridor")
    }

    fn profile_from_limited(
        seg: &[Coord],
        spans: &[Span],
        max_grade: f64,
        terrain: impl Fn(Coord) -> f64,
    ) -> Profile {
        let mut elev = |c: Coord| terrain(c);
        solve(seg, spans, Some(max_grade), &mut elev).expect("non-degenerate test corridor")
    }

    #[test]
    fn bridge_spans_a_ravine_on_the_road_grade_not_the_terrain() {
        // At-grade approaches at 100 m on both ends; a bridge span in the middle
        // over a 60 m-deep ravine (scenario S1). The deck must stay near 100 m
        // (the road grade the anchors imply), high above the ravine floor.
        let (seg, len) = line(256, 0.06);
        let mid = seg[128].x;
        let cos_lat = 46.0_f64.to_radians().cos();
        let terrain = move |c: Coord| {
            let dm = (c.x - mid).abs() * cos_lat * DEG_M;
            if dm < 300.0 { 100.0 - 60.0 * (1.0 - dm / 300.0) } else { 100.0 }
        };
        let p = profile_from(&seg, &[span(0.34 * len, 0.66 * len, 1)], terrain);
        let floor = terrain(Coord { x: mid, y: 46.0 });
        let deck = p.height_at(mid, 46.0);
        assert!(deck - floor > 45.0, "deck {deck} only {} m over the ravine {floor}", deck - floor);
    }

    #[test]
    fn deck_holds_the_road_grade_over_a_steep_flank() {
        // A bridge span whose terrain climbs steeply (a hillside), but whose
        // at-grade anchors are 100 m and 110 m: the deck must follow the gentle
        // ~anchor grade, never the steep terrain.
        let (seg, len) = line(256, 0.06);
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 0.06 * cos_lat * DEG_M;
        let terrain = move |c: Coord| 100.0 + 300.0 * (c.x - 6.0) * cos_lat * DEG_M / len_m;
        let p = profile_from(&seg, &[span(0.03 * len, 0.97 * len, 1)], move |c| {
            let x = (c.x - 6.0) * cos_lat * DEG_M / len_m; // 0..1
            if x < 0.03 {
                100.0
            } else if x > 0.97 {
                110.0
            } else {
                terrain(c)
            }
        });
        let a = p.height_at(6.0 + 0.06 * 0.4, 46.0);
        let b = p.height_at(6.0 + 0.06 * 0.6, 46.0);
        let dx = 0.06 * 0.2 * cos_lat * DEG_M;
        let grade = (b - a).abs() / dx;
        assert!(grade < 0.15, "deck grade {grade} too steep (a={a} b={b})");
    }

    #[test]
    fn deck_line_is_one_straight_ramp_over_a_folded_profile() {
        // A road profile that climbs steadily but hooks down over its last few
        // nodes (an abutment touchdown — the shape that folds a swept box).
        // `deck_line` must return a single straight ramp.
        let (nodes, _) = line(24, 0.02);
        let arc = cumulative(&nodes);
        let n = nodes.len();
        let mut road_m: Vec<f64> =
            (0..n).map(|i| 100.0 + 12.0 * i as f64 / (n - 1) as f64).collect();
        road_m[n - 1] = 105.0;
        road_m[n - 2] = 108.0;
        road_m[n - 3] = 110.0;
        let deck_m = deck_ramp(&arc, &road_m, &vec![false; n]);
        let p = Profile {
            cos_lat: run_cos_lat(&nodes),
            smooth: nodes.clone(),
            nodes: nodes.clone(),
            arc,
            road_m,
            deck_m,
            terrain_m: vec![0.0; n],
            at_grade: vec![false; n],
        };

        let deck = p.deck_line(&nodes);
        for w in deck.windows(3) {
            let second = (w[2] - w[1]) - (w[1] - w[0]);
            assert!(second.abs() < 1e-6, "deck not straight: {deck:?}");
        }
        assert!(deck[n - 1] > deck[0], "deck should ramp up, got {deck:?}");
    }

    #[test]
    fn enters_a_hillside_by_occlusion_not_a_plunge() {
        // ground | bridge over a ravine | tunnel into a hill (S5/S7). The deck
        // holds the gentle road grade across the bridge — standing high over
        // the ravine — and where the hill rises above it (the tunnel side) it
        // passes *under* the terrain rather than plunging down to it.
        let (seg, len) = line(256, 0.06);
        let cos_lat = 46.0_f64.to_radians().cos();
        let (x0, x1) = (seg[0].x, seg[seg.len() - 1].x);
        let portal = x0 + 0.6 * (x1 - x0);
        let ravine = x0 + 0.45 * (x1 - x0);
        let terrain = move |c: Coord| {
            let dr = (c.x - ravine).abs() * cos_lat * DEG_M;
            let base = if dr < 120.0 { 100.0 - 50.0 * (1.0 - dr / 120.0) } else { 100.0 };
            if c.x > portal { 100.0 + 300.0 * (c.x - portal) / (x1 - portal) } else { base }
        };
        let p = profile_from(
            &seg,
            &[span(0.3 * len, 0.6 * len, 1), span(0.6 * len, len, -5)],
            terrain,
        );
        let over = p.height_at(ravine, 46.0) - terrain(Coord { x: ravine, y: 46.0 });
        assert!(over > 30.0, "deck only {over} m over the ravine");
        let into = p.height_at(portal + 0.5 * (x1 - portal), 46.0)
            - terrain(Coord { x: portal + 0.5 * (x1 - portal), y: 46.0 });
        assert!(into < 0.0, "deck rides {into} m above the hill instead of under it");
    }

    #[test]
    fn engineered_grade_flattens_a_transient_bump() {
        // A flat road crossed by a 6 m terrain bump over 60 m. The road cuts
        // straight through it (within the deviation budget), holding a gentle
        // grade instead of tracing the bump's steep flanks.
        let n = 31;
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * 10.0).collect();
        let at_grade = vec![true; n];
        let terrain: Vec<f64> = (0..n)
            .map(|i| 100.0 + 6.0 * (1.0 - ((i as f64 - 15.0).abs() / 3.0)).max(0.0))
            .collect();
        let mut road = terrain.clone();
        limit_road_grade(&arc, &mut road, &terrain, &at_grade, 0.06);
        for i in 1..n {
            let g = ((road[i] - road[i - 1]) / (arc[i] - arc[i - 1])).abs();
            assert!(g <= 0.06 + 1e-9, "grade {g} too steep at node {i}");
        }
        for i in 0..n {
            assert!((road[i] - terrain[i]).abs() <= MAX_ROAD_DEVIATION_M + 1e-9);
        }
    }

    #[test]
    fn road_hugs_the_ground_on_a_long_steep_climb() {
        // Terrain climbing a sustained 15 % — far above the grade ceiling (S9).
        // The deviation budget wins: the road follows the slope, never drifting
        // more than MAX_ROAD_DEVIATION_M from the terrain.
        let n = 60;
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * 10.0).collect();
        let at_grade = vec![true; n];
        let terrain: Vec<f64> = (0..n).map(|i| 100.0 + 0.15 * arc[i]).collect();
        let mut road = terrain.clone();
        limit_road_grade(&arc, &mut road, &terrain, &at_grade, 0.06);
        for i in 0..n {
            assert!(
                (road[i] - terrain[i]).abs() <= MAX_ROAD_DEVIATION_M + 1e-9,
                "road drifted {} m from terrain at node {i}",
                (road[i] - terrain[i]).abs()
            );
        }
    }

    #[test]
    fn a_gentle_road_is_left_on_the_terrain() {
        // A motorway on terrain that never exceeds the ceiling stays draped:
        // the limiter only intervenes where the ground is too steep.
        let (seg, _) = line(120, 0.05);
        let cos_lat = 46.0_f64.to_radians().cos();
        let terrain = move |c: Coord| 100.0 + 0.03 * (c.x - 6.0) * cos_lat * DEG_M;
        let p = profile_from_limited(&seg, &[], 0.06, terrain);
        let mid = seg[60].x;
        assert!(
            (p.height_at(mid, 46.0) - terrain(Coord { x: mid, y: 46.0 })).abs() < 0.5,
            "a gentle road should stay on the ground"
        );
    }

    #[test]
    fn meets_the_ground_at_at_grade_anchors() {
        // Flat ground, a bridge in the middle: the road surface at the at-grade
        // ends sits on the ground, meeting the draped approach road there.
        let (seg, len) = line(128, 0.04);
        let p = profile_from(&seg, &[span(0.4 * len, 0.6 * len, 1)], |_| 100.0);
        assert!((p.height_at(seg[0].x, seg[0].y) - 100.0).abs() < 0.5);
        assert!((p.height_at(seg[seg.len() - 1].x, seg[seg.len() - 1].y) - 100.0).abs() < 0.5);
    }

    #[test]
    fn flat_overpass_sits_at_grade() {
        // A bridge over flat ground (no dip): the road surface is the single
        // model, so the deck lies flush at the grade — no clearance offset that
        // would float its ends above an adjoining tunnel. (Real overpass lift
        // comes with crossing detection, milestone M-b.)
        let (seg, len) = line(256, 0.06);
        let p = profile_from(&seg, &[span(0.3 * len, 0.7 * len, 1)], |_| 100.0);
        assert!((p.height_at(seg[0].x, seg[0].y) - 100.0).abs() < 0.5);
        assert!((p.height_at(seg[128].x, 46.0) - 100.0).abs() < 0.5);
    }

    #[test]
    fn tent_lift_raises_the_crossing_and_ramps_the_approaches() {
        // A flat road with a bridge span in the middle: a clearance lift at
        // the span centre must hold the whole span at the peak and ramp the
        // at-grade approaches down from the span edges, raise-only.
        let (seg, len) = line(256, 0.06);
        let mut p = profile_from(&seg, &[span(0.45 * len, 0.55 * len, 1)], |_| 100.0);
        let mid = Coord { x: seg[128].x, y: 46.0 };
        let arc0 = p.arc_of(mid.x, mid.y);
        p.raise_tent(arc0, 106.5, 0.08);
        p.rebuild_deck();
        assert!(p.deck_height_at(mid.x, mid.y) > 106.0, "deck must lift at the crossing");
        // The far ends stay on the ground (the shoulders have run out).
        assert!((p.height_at(seg[0].x, 46.0) - 100.0).abs() < 0.5);
        // The approach just outside the span (40 m before its edge) is raised
        // (the embankment demand) but below the peak.
        let approach = Coord { x: 6.0 + 0.06 * (0.45 - 40.0 / 4600.0), y: 46.0 };
        let h = p.height_at(approach.x, approach.y);
        assert!(h > 101.0 && h < 106.5, "approach should ramp, got {h}");
        // Profile continuity at the abutment: the at-grade side of the span
        // edge meets the lifted deck without a step.
        let edge = Coord { x: 6.0 + 0.06 * 0.45, y: 46.0 };
        let step = (p.height_at(edge.x, edge.y) - 106.5).abs();
        assert!(step < 1.0, "abutment step {step} too large");
        // The terminal clamp can push the whole span if the fit fell short.
        p.raise_span_to(arc0, 110.0);
        assert!(p.deck_height_at(mid.x, mid.y) >= 110.0 - 1e-9);
    }
}
