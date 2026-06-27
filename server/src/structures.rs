//! Reconstructs a road's own (gentle) elevation profile through bridges and
//! tunnels — the grade the road holds, not the terrain it crosses. The geometry
//! that rides this profile (bridge decks, tunnel boxes) is swept in
//! `structure_mesh`; this module owns the profile alone.
//!
//! Overture marks a whole viaduct — kilometres of road — as one segment carrying
//! a *linearly-referenced* level structure: at-grade stretches (level 0), bridges
//! (positive level), and tunnels (negative). The terrain along such a segment is
//! wild — the Viaduc de Chillon's centerline DEM drops into a ravine (a bridge
//! spans it ~70 m up) and rears up to a 680 m hill (a tunnel pierces it) — while
//! the road itself holds a gentle highway grade (~3 %). Baking the road at
//! `terrain` therefore makes it climb the hillside at the *terrain's* 30–100 %
//! grade: impossibly steep. Chording straight across one structure run is no
//! better — the run's own endpoints already sit on the steep flank.
//!
//! So the road surface is anchored to a reconstructed *road profile*:
//!
//! 1. Sample the DEM along the whole segment.
//! 2. Where the road is at grade (level 0) it sits on the ground, so those nodes
//!    are elevation *anchors*: `profile = terrain`.
//! 3. Across every bridge/tunnel run, the profile is a straight (gentle)
//!    interpolation between the bounding anchors — the grade the road actually
//!    holds — independent of the terrain excursion under it.
//!
//! A structure thus rides the gentle profile: where the terrain dips below it
//! (a ravine) a bridge deck stands high above the floor — the visible viaduct;
//! where the terrain rises above it (a flank, a tunnelled hill) the body passes
//! below ground and the terrain, drawn first and owning the depth buffer,
//! occludes it.
//!
//! The profile is a function of the *global* segment and its level structure
//! (both carried to every tile fragment as the `deck_run`/`deck_levels`
//! properties — see `pipeline::process_feature`), evaluated by projecting each
//! output vertex onto the segment, so independent tile fragments of one structure
//! compute identical heights and the seams line up.

use geo_types::{Coord, Geometry, LineString, MultiLineString};

use crate::dem::Dem;
use crate::levels::{level_at, LevelRun};
use crate::project::{self, Bounds};
use crate::terrain;
use crate::tile_build::{prop_str, EncoderFeature};
use crate::value::Value;
use crate::wkb;

/// Target deck segment length in metres after densification, used both to sample
/// the road profile along the segment and to subdivide the output centerline so
/// the deck renders as a smooth curve.
const DECK_SEGMENT_M: f64 = 8.0;

/// Cap on densified vertices per linestring — a runaway guard for pathological
/// inputs; real segments are well under this.
const MAX_DECK_VERTS: usize = 4096;

/// Metres per degree of latitude (spherical approximation), for converting the
/// densification target and projection into a local metric space.
const DEG_M: f64 = 111_320.0;

/// Property carrying a bridge's full segment centerline (hex-encoded WKB) and …
const DECK_RUN_KEY: &str = "deck_run";
/// … its level structure, so every tile fragment can rebuild the same road
/// profile. Both are written by `pipeline::process_feature` and stripped here
/// once consumed.
const DECK_LEVELS_KEY: &str = "deck_levels";

/// A road's reconstructed *surface* profile over a whole segment: a densified
/// centerline with a per-node road-surface height. Evaluated at an arbitrary
/// point by projecting it onto the segment, so independent tile fragments of one
/// structure compute the same heights and the seams line up. Built from the
/// *global* segment carried on each fragment (the `deck_run`/`deck_levels`
/// properties).
///
/// This surface is the single road model: a bridge deck and a tunnel bore both
/// ride it (their top face *is* this surface), so where a bridge meets a tunnel
/// or an approach road the road is continuous. Both are swept into meshes by
/// `structure_mesh`.
pub struct RoadProfile {
    /// Densified segment nodes in (lon, lat).
    nodes: Vec<Coord>,
    /// `nodes` low-pass-smoothed (endpoint-preserving), the line the deck box is
    /// swept along so it follows the road without tracing every digitising wiggle.
    /// Smoothing the carried whole segment (not a per-tile fragment) keeps the
    /// fragments aligned at their seams.
    smooth: Vec<Coord>,
    /// Cumulative metric arc length at each node (`arc[0] == 0`), the linear
    /// reference [`deck_line`](RoadProfile::deck_line) fits a straight ramp in.
    arc: Vec<f64>,
    /// Road-surface height in metres above the ellipsoid at each node.
    road_m: Vec<f64>,
    /// `cos(mean latitude)`, scaling longitude into the local metric space used
    /// for projection.
    cos_lat: f64,
}

/// One swept deck cross-section: the (smoothed) centerline position, the deck-top
/// height, and the unit left-perpendicular (ENU metres) the section spans.
pub struct DeckNode {
    pub lon: f64,
    pub lat: f64,
    pub height_m: f64,
    pub left_e: f64,
    pub left_n: f64,
}

/// Binomial (1-2-1) smoothing passes applied to the centerline before sweeping.
/// Each pass widens the kernel; four damps the short digitising wiggle of a road
/// line (a footway's zigzag, a viaduct's vertex noise) while keeping its real
/// curve, so the swept box is a regular prism.
const SMOOTH_PASSES: usize = 4;

impl RoadProfile {
    /// Builds the surface profile from a feature's recorded segment and level
    /// structure, sampling the rendered terrain surface at output zoom `z`. The
    /// at-grade anchors thus sit on the same surface a ground road drapes on, so
    /// a structure meets its approach roads exactly at the abutments. `None` when
    /// the feature carries no `deck_run` (not a structure) or it is degenerate.
    pub fn from_feature(f: &EncoderFeature, dem: &mut Dem, z: u8, bounds: &Bounds) -> Option<RoadProfile> {
        let seg = decode_run(prop_str(f, DECK_RUN_KEY).as_deref()?)?;
        let runs = prop_str(f, DECK_LEVELS_KEY).as_deref().map(decode_levels).unwrap_or_default();
        build_profile(&seg, &runs, &mut |c| {
            terrain::surface_height(bounds, c.x, c.y, &mut |lon, lat| dem.elevation(lon, lat, z))
        })
    }

    /// Road-surface height in metres above the ellipsoid at `(lon, lat)`, found
    /// by projecting the point onto the nearest segment edge and interpolating.
    /// Clipped fragment vertices all lie on the segment, so the nearest
    /// on-segment height is exact. This is the height a structure's road face
    /// (bridge deck top, tunnel box top) takes.
    pub fn height_at(&self, lon: f64, lat: f64) -> f64 {
        project_onto(&self.nodes, &self.road_m, self.cos_lat, lon, lat)
    }

    /// Deck cross-sections for a structure's (clipped, densified, in-order)
    /// centerline `pts`: each vertex placed on the *smoothed* global road line
    /// with a straight-ramp deck height and a smoothed cross-section direction, so
    /// the swept box is a regular prism that follows the road instead of tracing
    /// every wiggle and dive.
    ///
    /// All three smoothings are anchored to the *whole-segment* line carried on
    /// every fragment, so tile fragments of one structure stay identical at their
    /// seams: the centerline is low-pass-smoothed once ([`smooth`](Self::smooth)),
    /// the height is one straight line fit in *global* arc, and the section
    /// direction is read from the global smoothed line. The straight height is
    /// what stops the box folding — the per-vertex profile is faithful but busy at
    /// a structure's edges (it dives to terrain at an abutment, and a tunnel's
    /// extended portal stub follows the descending approach), and the road a
    /// structure actually holds is its gentle grade. The arc-order walk also stops
    /// a curving viaduct from snapping a vertex onto a far arc that nears it in
    /// plan.
    pub fn deck_nodes(&self, pts: &[Coord]) -> Vec<DeckNode> {
        let probe = self.walk(pts);
        let s: Vec<f64> = probe.iter().map(|&(i, t)| lerp(self.arc[i], self.arc[i + 1], t)).collect();
        let raw: Vec<f64> = probe.iter().map(|&(i, t)| lerp(self.road_m[i], self.road_m[i + 1], t)).collect();
        let h = fit_ramp(&s, &raw);
        probe
            .iter()
            .zip(h)
            .map(|(&(i, t), height_m)| {
                let lon = lerp(self.smooth[i].x, self.smooth[i + 1].x, t);
                let lat = lerp(self.smooth[i].y, self.smooth[i + 1].y, t);
                let (left_e, left_n) = self.section_left(i);
                DeckNode { lon, lat, height_m, left_e, left_n }
            })
            .collect()
    }

    /// Deck-top heights only, as one straight ramp — the height half of
    /// [`deck_nodes`](Self::deck_nodes), kept for direct testing.
    pub fn deck_line(&self, pts: &[Coord]) -> Vec<f64> {
        let probe = self.walk(pts);
        let s: Vec<f64> = probe.iter().map(|&(i, t)| lerp(self.arc[i], self.arc[i + 1], t)).collect();
        let raw: Vec<f64> = probe.iter().map(|&(i, t)| lerp(self.road_m[i], self.road_m[i + 1], t)).collect();
        fit_ramp(&s, &raw)
    }

    /// Projects an in-order on-segment polyline onto the profile, returning the
    /// `(edge, t)` of each vertex. The walk is monotonic from a robust interior
    /// seed, so a vertex is confined to the arc its neighbours sit on — a curving
    /// segment that nears itself in plan can't snap a vertex onto a far arc. A
    /// clipped fragment may run either way; the direction is read from two
    /// interior points (the ends are where a self-approach lurks).
    fn walk(&self, pts: &[Coord]) -> Vec<(usize, f64)> {
        let edges = self.nodes.len().saturating_sub(1);
        if edges < 2 || pts.len() < 3 {
            return pts.iter().map(|p| self.project(0, edges.max(1), *p)).collect();
        }
        let ia = self.project(0, edges, pts[pts.len() / 3]).0;
        let ib = self.project(0, edges, pts[2 * pts.len() / 3]).0;
        let dir: isize = if ib >= ia { 1 } else { -1 };
        // The cursor may range ~6 edges per step: enough slack for one deck step
        // (about one profile edge) while still walling off a far arc.
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

    /// Unit left-perpendicular (ENU, scaled-degree space) of the smoothed line at
    /// edge `i`, read over a short window so the cross-section direction varies
    /// gently and the box edges stay clean.
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

    /// A flat profile holding `height_m` over the given centerline — a DEM-free
    /// constructor for tests and degenerate inputs.
    pub fn flat(nodes: &[Coord], height_m: f64) -> RoadProfile {
        let cos_lat = run_cos_lat(nodes);
        let mut arc = Vec::with_capacity(nodes.len());
        let mut acc = 0.0;
        for (i, c) in nodes.iter().enumerate() {
            if i > 0 {
                acc += metric_len(nodes[i - 1], *c, cos_lat);
            }
            arc.push(acc);
        }
        RoadProfile {
            cos_lat,
            arc,
            smooth: smooth_path(nodes),
            nodes: nodes.to_vec(),
            road_m: vec![height_m; nodes.len()],
        }
    }
}

/// Builds a road-surface profile from a segment centerline and its level runs:
/// densify, sample terrain through `elev`, anchor the road at the at-grade
/// (level 0) stretches, and interpolate the gentle profile across the structures.
/// `None` for a degenerate segment. The terrain sampler is injected so tests can
/// bypass the DEM.
///
/// The road's gentle grade is anchored only at the at-grade stretches — the one
/// place the centerline DEM reliably gives the road's own elevation. Across a
/// structure the profile is the gentle line those anchors imply; where the
/// terrain dips below it the deck stands proud (a viaduct), where it rises above
/// it the body passes under ground and the terrain occludes it — that is how a
/// bridge connects into a hillside or a tunnel pierces it, no forced landing.
fn build_profile(
    seg: &[Coord],
    runs: &[LevelRun],
    elev: &mut dyn FnMut(Coord) -> f64,
) -> Option<RoadProfile> {
    if seg.len() < 2 {
        return None;
    }
    let cos_lat = run_cos_lat(seg);
    let (nodes, arc, total) = densify_run(seg, cos_lat);
    let n = nodes.len();
    if n < 2 {
        return None;
    }
    // Overture measures the level `between` fractions along the segment's geodesic
    // (spheroid) length, so anchor them against the metric arc length `densify_run`
    // already accumulated — not raw degree length, which at this latitude
    // overweights the east-west legs and slides every portal along the road.
    let arc_total = total.max(f64::MIN_POSITIVE);
    let terrain: Vec<f64> = nodes.iter().map(|c| elev(*c)).collect();
    let at_grade: Vec<bool> =
        (0..n).map(|i| level_at(runs, arc[i] / arc_total) == 0).collect();
    let road_m = road_profile(&arc, &terrain, &at_grade);
    let smooth = smooth_path(&nodes);
    Some(RoadProfile { nodes, smooth, arc, road_m, cos_lat })
}

/// Linear interpolation between `a` and `b` at `t`.
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Endpoint-preserving binomial (1-2-1) smoothing of a centerline, [`SMOOTH_PASSES`]
/// times, damping short digitising wiggle while keeping the road's real curve.
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

/// Fits one straight ramp to arc-referenced heights and returns the fitted value
/// at each arc. The central span is least-squares-fit (the ends are trimmed:
/// a structure's busy landings — abutment touchdown, portal stub — must not tilt
/// the line). For a single chord the fit recovers it exactly, so tile fragments
/// of one run share the line.
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
/// onto the nearest segment edge of `nodes` and interpolating. Clipped fragment
/// vertices all lie on the segment, so the nearest on-segment value is exact.
fn project_onto(nodes: &[Coord], vals: &[f64], cos_lat: f64, lon: f64, lat: f64) -> f64 {
    let (i, t) = nearest_edge(nodes, cos_lat, 0, nodes.len().saturating_sub(1), Coord { x: lon, y: lat });
    vals[i] + (vals[i + 1] - vals[i]) * t
}

/// Nearest edge to `p` over the edge index range `[lo, hi)` (edge `i` spans
/// `nodes[i]..nodes[i+1]`), returning the edge index and the clamped parameter
/// `t` of the foot of the perpendicular. Longitudes are scaled by `cos_lat` into
/// the local metric space. A bounded range lets the arc-order walk confine the
/// search to one arc; `lo = 0, hi = edges` makes it a full scan.
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
/// anchors across each structure. The segment endpoints are always anchors, so
/// every node is bracketed.
fn road_profile(arc: &[f64], terrain: &[f64], anchor: &[bool]) -> Vec<f64> {
    let n = terrain.len();

    // Nearest anchor (arc, elevation) at-or-before and at-or-after each node, in
    // single forward/backward passes.
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
                    // No at-grade anchor anywhere: chord the segment endpoints.
                    let span = (arc[n - 1] - arc[0]).max(f64::MIN_POSITIVE);
                    terrain[0] + (terrain[n - 1] - terrain[0]) * (arc[i] - arc[0]) / span
                }
            }
        })
        .collect()
}

/// Densifies a segment to ~[`DECK_SEGMENT_M`] spacing, returning the nodes, their
/// cumulative metric arc length, and the total length.
fn densify_run(run: &[Coord], cos_lat: f64) -> (Vec<Coord>, Vec<f64>, f64) {
    let mut nodes = vec![run[0]];
    let mut arc = vec![0.0];
    let mut total = 0.0;
    for w in run.windows(2) {
        let (p0, p1) = (w[0], w[1]);
        let n = ((metric_len(p0, p1, cos_lat) / DECK_SEGMENT_M).ceil() as usize).clamp(1, MAX_DECK_VERTS);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let c = Coord { x: p0.x + (p1.x - p0.x) * t, y: p0.y + (p1.y - p0.y) * t };
            total += metric_len(*nodes.last().expect("seeded"), c, cos_lat);
            nodes.push(c);
            arc.push(total);
        }
        if nodes.len() >= MAX_DECK_VERTS {
            break;
        }
    }
    (nodes, arc, total)
}

/// Records a bridge's whole-segment centerline (`deck_run`, hex WKB) and level
/// structure (`deck_levels`), so every tile fragment can rebuild the same road
/// profile. `None` for non-line or degenerate geometry. Called in phase 1 (see
/// `pipeline::process_feature`); the values are consumed and stripped by
/// [`stamp_bridge_deck`].
pub fn encode_segment(geometry: &Geometry, runs: &[LevelRun]) -> Option<[(String, Value); 2]> {
    let line = match geometry {
        Geometry::LineString(ls) => ls.clone(),
        Geometry::MultiLineString(mls) => {
            LineString(mls.0.iter().flat_map(|ls| ls.0.iter().copied()).collect())
        }
        _ => return None,
    };
    if line.0.len() < 2 {
        return None;
    }
    let run = to_hex(&wkb::to_wkb(&Geometry::LineString(line)));
    Some([
        (DECK_RUN_KEY.to_string(), Value::String(run)),
        (DECK_LEVELS_KEY.to_string(), Value::String(encode_levels(runs))),
    ])
}

/// Strips the carried deck properties — used once a structure's mesh is baked
/// (or on the DEM-less path, where roads stay flat) so the internal carry never
/// reaches a tile's property dictionary.
pub fn discard_run(f: &mut EncoderFeature) {
    f.properties.retain(|(k, _)| k != DECK_RUN_KEY && k != DECK_LEVELS_KEY);
}

/// Target sub-segment length when densifying a ground road, in quantized tile
/// units, so the baked centerline tracks the terrain mesh's `~tile/16` cells
/// (matches the client's former draping density).
const ROAD_SEGMENT_Q: f64 = 768.0;

/// Bakes a ground road's per-vertex elevation onto the feature from the terrain
/// surface: densifies the (clipped) centerline so it follows the relief, samples
/// [`terrain::surface_height`] at every vertex, and writes the heights into
/// `f.z`. Unlike a structure, a ground road just sits on the terrain, so no
/// whole-segment profile is needed — the surface is sampled locally and is
/// consistent across tiles. A no-op for non-line geometry.
pub fn bake_road_elevation(f: &mut EncoderFeature, dem: &mut Dem, z: u8, bounds: &Bounds) {
    let mut height = |lon: f64, lat: f64| {
        terrain::surface_height(bounds, lon, lat, &mut |a, b| dem.elevation(a, b, z))
    };
    if let Some((geom, zs)) = densify_with_surface(&f.geometry, bounds, &mut height) {
        f.geometry = geom;
        f.z = Some(zs);
    }
}

/// Densifies a (multi)linestring and samples the terrain surface at every vertex,
/// returning the new geometry and the matching `z` array (flattened in
/// `line_geometry` vertex order), or `None` for non-line / empty input.
fn densify_with_surface(
    g: &Geometry,
    bounds: &Bounds,
    height: &mut dyn FnMut(f64, f64) -> f64,
) -> Option<(Geometry, Vec<i32>)> {
    match g {
        Geometry::LineString(ls) => {
            let (xy, zs) = densify_road_line(ls, bounds, height);
            (xy.len() >= 2).then_some((Geometry::LineString(LineString(xy)), zs))
        }
        Geometry::MultiLineString(mls) => {
            let mut parts = Vec::new();
            let mut zs = Vec::new();
            for ls in &mls.0 {
                let (xy, z) = densify_road_line(ls, bounds, height);
                if xy.len() >= 2 {
                    parts.push(LineString(xy));
                    zs.extend(z);
                }
            }
            (!parts.is_empty()).then_some((Geometry::MultiLineString(MultiLineString(parts)), zs))
        }
        _ => None,
    }
}

/// Densifies one linestring to ~[`ROAD_SEGMENT_Q`] quantized spacing and samples
/// the terrain surface height at every (original and inserted) vertex.
fn densify_road_line(
    line: &LineString,
    bounds: &Bounds,
    height: &mut dyn FnMut(f64, f64) -> f64,
) -> (Vec<Coord>, Vec<i32>) {
    let pts = &line.0;
    let mut xy = Vec::new();
    let mut zs = Vec::new();
    if pts.is_empty() {
        return (xy, zs);
    }
    let mut push = |c: Coord, xy: &mut Vec<Coord>, zs: &mut Vec<i32>| {
        zs.push(project::quantize_z(height(c.x, c.y)));
        xy.push(c);
    };
    push(pts[0], &mut xy, &mut zs);
    for w in pts.windows(2) {
        let (p0, p1) = (w[0], w[1]);
        let qlen = quant_len(p0, p1, bounds);
        let n = ((qlen / ROAD_SEGMENT_Q).ceil() as usize).clamp(1, MAX_DECK_VERTS);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let c = Coord { x: p0.x + (p1.x - p0.x) * t, y: p0.y + (p1.y - p0.y) * t };
            push(c, &mut xy, &mut zs);
        }
    }
    (xy, zs)
}

/// Distance between two lon/lat points in quantized tile units, for densification.
fn quant_len(a: Coord, b: Coord, bounds: &Bounds) -> f64 {
    let dx = project::quantize_x(b.x, bounds) as f64 - project::quantize_x(a.x, bounds) as f64;
    let dy = project::quantize_y(b.y, bounds) as f64 - project::quantize_y(a.y, bounds) as f64;
    (dx * dx + dy * dy).sqrt()
}

/// Planar distance between two lon/lat points in metres (lon scaled by `cos_lat`).
fn metric_len(a: Coord, b: Coord, cos_lat: f64) -> f64 {
    let dx = (b.x - a.x) * cos_lat * DEG_M;
    let dy = (b.y - a.y) * DEG_M;
    (dx * dx + dy * dy).sqrt()
}

/// `cos(mean latitude)` of a run, for the longitude scaling.
fn run_cos_lat(run: &[Coord]) -> f64 {
    let mean = run.iter().map(|c| c.y).sum::<f64>() / run.len() as f64;
    mean.to_radians().cos()
}

/// Encodes level runs as `start:end:level` triples joined by `;` for the
/// `deck_levels` property.
fn encode_levels(runs: &[LevelRun]) -> String {
    runs.iter()
        .map(|r| format!("{:.6}:{:.6}:{}", r.start, r.end, r.level))
        .collect::<Vec<_>>()
        .join(";")
}

/// Inverse of [`encode_levels`]; skips malformed triples.
fn decode_levels(s: &str) -> Vec<LevelRun> {
    if s.is_empty() {
        return Vec::new();
    }
    s.split(';')
        .filter_map(|t| {
            let mut p = t.split(':');
            let start = p.next()?.parse().ok()?;
            let end = p.next()?.parse().ok()?;
            let level = p.next()?.parse().ok()?;
            Some(LevelRun { start, end, level })
        })
        .collect()
}

/// Decodes a `deck_run` property value (hex WKB) back into the segment centerline.
fn decode_run(hex: &str) -> Option<Vec<Coord>> {
    let bytes = from_hex(hex)?;
    match wkb::parse(&bytes).ok()? {
        Geometry::LineString(ls) => Some(ls.0),
        _ => None,
    }
}

/// Lowercase hex of a byte slice (no dependency; `deck_run` carries binary WKB
/// through the string-typed property channel).
fn to_hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        s.push(char::from_digit((b >> 4) as u32, 16).expect("nibble"));
        s.push(char::from_digit((b & 0xf) as u32, 16).expect("nibble"));
    }
    s
}

/// Inverse of [`to_hex`]; `None` on odd length or a non-hex digit.
fn from_hex(s: &str) -> Option<Vec<u8>> {
    let b = s.as_bytes();
    if b.len() & 1 == 1 {
        return None;
    }
    let mut out = Vec::with_capacity(b.len() / 2);
    for pair in b.chunks_exact(2) {
        let hi = (pair[0] as char).to_digit(16)?;
        let lo = (pair[1] as char).to_digit(16)?;
        out.push((hi * 16 + lo) as u8);
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(start: f64, end: f64, level: i64) -> LevelRun {
        LevelRun { start, end, level }
    }

    /// A run of `n` evenly spaced vertices over `span_deg` of longitude at lat 46.
    fn line(n: usize, span_deg: f64) -> Vec<Coord> {
        (0..n).map(|i| Coord { x: 6.0 + span_deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect()
    }

    /// Builds a profile from a segment, level runs, and an injected terrain
    /// sampler, bypassing the DEM, for deterministic geometry tests.
    fn profile_from(seg: &[Coord], runs: &[LevelRun], terrain: impl Fn(Coord) -> f64) -> RoadProfile {
        let mut elev = |c: Coord| terrain(c);
        build_profile(seg, runs, &mut elev).expect("non-degenerate test segment")
    }

    #[test]
    fn bridge_spans_a_ravine_on_the_road_grade_not_the_terrain() {
        // At-grade approaches at 100 m on both ends; a bridge run in the middle
        // over a 60 m-deep ravine. The deck must stay near 100 m (the road grade
        // the anchors imply), high above the ravine floor — not dive into it.
        let seg = line(256, 0.06);
        let mid = seg[128].x;
        let cos_lat = 46.0_f64.to_radians().cos();
        let terrain = move |c: Coord| {
            let dm = (c.x - mid).abs() * cos_lat * DEG_M;
            if dm < 300.0 { 100.0 - 60.0 * (1.0 - dm / 300.0) } else { 100.0 }
        };
        // bridge over the middle third (the ravine), at grade elsewhere.
        let p = profile_from(&seg, &[run(0.34, 0.66, 1)], terrain);
        let floor = terrain(Coord { x: mid, y: 46.0 });
        let deck = p.height_at(mid, 46.0);
        assert!(deck - floor > 45.0, "deck {deck} only {} m over the ravine {floor}", deck - floor);
    }

    #[test]
    fn deck_holds_the_road_grade_over_a_steep_flank() {
        // A bridge run whose terrain climbs steeply (a hillside), but whose
        // at-grade anchors are 100 m and 110 m: the deck must follow the gentle
        // ~anchor grade, never the steep terrain.
        let seg = line(256, 0.06);
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 0.06 * cos_lat * DEG_M;
        // Terrain ramps 100 → 400 (≈7 %); anchors (ends) are 100 and 110.
        let terrain = move |c: Coord| 100.0 + 300.0 * (c.x - 6.0) * cos_lat * DEG_M / len_m;
        // Bridge across the interior; thin at-grade stubs at both ends.
        let p = profile_from(&seg, &[run(0.03, 0.97, 1)], move |c| {
            // Override the end anchors to a gentle pair by flattening near ends.
            let x = (c.x - 6.0) * cos_lat * DEG_M / len_m; // 0..1
            if x < 0.03 { 100.0 } else if x > 0.97 { 110.0 } else { terrain(c) }
        });
        // Across the interior the deck grade must stay gentle (well under 15 %),
        // not track the 7 %+ terrain that spikes far higher mid-span.
        let a = p.height_at(6.0 + 0.06 * 0.4, 46.0);
        let b = p.height_at(6.0 + 0.06 * 0.6, 46.0);
        let dx = 0.06 * 0.2 * cos_lat * DEG_M;
        let grade = (b - a).abs() / dx;
        assert!(grade < 0.15, "deck grade {grade} too steep (a={a} b={b})");
    }

    #[test]
    fn deck_line_is_one_straight_ramp_over_a_folded_profile() {
        // A road profile that climbs steadily but hooks down over its last few
        // nodes (an abutment touchdown / portal stub — the shape that folds the
        // box). `deck_line` must return a single straight ramp: collinear heights
        // that ignore the end hook, so the swept box is a simple prism.
        let nodes = line(24, 0.02);
        let cos_lat = run_cos_lat(&nodes);
        let mut arc = vec![0.0];
        for i in 1..nodes.len() {
            arc.push(arc[i - 1] + metric_len(nodes[i - 1], nodes[i], cos_lat));
        }
        let n = nodes.len();
        let mut road_m: Vec<f64> = (0..n).map(|i| 100.0 + 12.0 * i as f64 / (n - 1) as f64).collect();
        // Dive the last three nodes back below the climb (the fold).
        road_m[n - 1] = 105.0;
        road_m[n - 2] = 108.0;
        road_m[n - 3] = 110.0;
        let p = RoadProfile { smooth: nodes.clone(), nodes: nodes.clone(), arc, road_m, cos_lat };

        let deck = p.deck_line(&nodes);
        // Collinear: the second difference vanishes everywhere (a straight ramp).
        for w in deck.windows(3) {
            let second = (w[2] - w[1]) - (w[1] - w[0]);
            assert!(second.abs() < 1e-6, "deck not straight: {deck:?}");
        }
        // The ramp follows the climb, not the end hook.
        assert!(deck[n - 1] > deck[0], "deck should ramp up, got {deck:?}");
    }

    #[test]
    fn enters_a_hillside_by_occlusion_not_a_plunge() {
        // ground | bridge over a ravine | tunnel into a hill. The deck holds the
        // gentle road grade across the bridge — standing high over the ravine —
        // and where the hill rises above it (the tunnel side) it passes *under*
        // the terrain rather than plunging down to it, so the slope occludes it.
        let seg = line(256, 0.06);
        let cos_lat = 46.0_f64.to_radians().cos();
        let (x0, x1) = (seg[0].x, seg[seg.len() - 1].x);
        let portal = x0 + 0.6 * (x1 - x0);
        let ravine = x0 + 0.45 * (x1 - x0);
        let terrain = move |c: Coord| {
            let dr = (c.x - ravine).abs() * cos_lat * DEG_M;
            let base = if dr < 120.0 { 100.0 - 50.0 * (1.0 - dr / 120.0) } else { 100.0 };
            if c.x > portal { 100.0 + 300.0 * (c.x - portal) / (x1 - portal) } else { base }
        };
        let p = profile_from(&seg, &[run(0.3, 0.6, 1), run(0.6, 1.0, -5)], terrain);
        // High over the ravine (a viaduct)…
        let over = p.height_at(ravine, 46.0) - terrain(Coord { x: ravine, y: 46.0 });
        assert!(over > 30.0, "deck only {over} m over the ravine");
        // …and into the hill the deck sits below the rising ground (occluded),
        // not perched on top of it.
        let into = p.height_at(portal + 0.5 * (x1 - portal), 46.0)
            - terrain(Coord { x: portal + 0.5 * (x1 - portal), y: 46.0 });
        assert!(into < 0.0, "deck rides {into} m above the hill instead of under it");
    }

    #[test]
    fn meets_the_ground_at_at_grade_anchors() {
        // Flat ground, a bridge in the middle: the road surface at the at-grade
        // ends sits on the ground, meeting the draped approach road there.
        let seg = line(128, 0.04);
        let p = profile_from(&seg, &[run(0.4, 0.6, 1)], |_| 100.0);
        assert!((p.height_at(seg[0].x, seg[0].y) - 100.0).abs() < 0.5);
        assert!((p.height_at(seg[seg.len() - 1].x, seg[seg.len() - 1].y) - 100.0).abs() < 0.5);
    }

    #[test]
    fn flat_overpass_sits_at_grade() {
        // A bridge over flat ground (no dip): the road surface is the single
        // model, so the deck lies flush at the grade — no clearance offset that
        // would float its ends above an adjoining tunnel. (Real overpass lift
        // awaits crossing detection, Tier 2.)
        let seg = line(256, 0.06);
        let p = profile_from(&seg, &[run(0.3, 0.7, 1)], |_| 100.0);
        assert!((p.height_at(seg[0].x, seg[0].y) - 100.0).abs() < 0.5);
        assert!((p.height_at(seg[128].x, 46.0) - 100.0).abs() < 0.5);
    }

    #[test]
    fn levels_roundtrip_through_the_property() {
        let runs = vec![run(0.1, 0.4, 1), run(0.4, 0.8, -5)];
        let decoded = decode_levels(&encode_levels(&runs));
        assert_eq!(decoded.len(), 2);
        assert!((decoded[0].start - 0.1).abs() < 1e-6 && decoded[1].level == -5);
    }

    #[test]
    fn segment_roundtrips_wkb_and_levels() {
        let seg = Geometry::LineString(LineString(vec![
            Coord { x: 6.1, y: 46.2 },
            Coord { x: 6.3, y: 46.4 },
        ]));
        let [(rk, rv), (lk, lv)] = encode_segment(&seg, &[run(0.0, 1.0, 1)]).unwrap();
        assert_eq!(rk, DECK_RUN_KEY);
        assert_eq!(lk, DECK_LEVELS_KEY);
        let Value::String(rs) = rv else { panic!() };
        let Value::String(_) = lv else { panic!() };
        assert_eq!(decode_run(&rs).unwrap().len(), 2);
    }

    #[test]
    fn discard_strips_both_carry_properties() {
        let [(rk, rv), (lk, lv)] = encode_segment(
            &Geometry::LineString(LineString(vec![
                Coord { x: 6.0, y: 46.0 },
                Coord { x: 6.1, y: 46.0 },
            ])),
            &[run(0.0, 1.0, 1)],
        )
        .unwrap();
        let mut f = EncoderFeature {
            id: 1,
            geometry: Geometry::LineString(LineString(vec![Coord { x: 6.0, y: 46.0 }])),
            properties: vec![("class".into(), Value::String("motorway".into())), (rk, rv), (lk, lv)],
            elevation: None,
            z: None,
            mesh: None,
        };
        discard_run(&mut f);
        assert!(!f.properties.iter().any(|(k, _)| k == DECK_RUN_KEY || k == DECK_LEVELS_KEY));
        assert!(f.properties.iter().any(|(k, _)| k == "class"));
    }

    #[test]
    fn non_line_geometry_yields_no_segment() {
        let pt = Geometry::Point(geo_types::Point::new(6.0, 46.0));
        assert!(encode_segment(&pt, &[]).is_none());
    }
}
