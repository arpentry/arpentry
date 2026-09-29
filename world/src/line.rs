//! Polylines and the points on them: lengths, arc intervals, resampling,
//! the nearest point of a segment and where two segments cross.
//!
//! Regions and their booleans are [`crate::poly`]'s; this is the plane
//! geometry the steps do along a centreline.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use crate::poly::{self, Pt};

/// `v` scaled to unit length; the zero vector stays zero.
pub fn unit(v: Pt) -> Pt {
    let len = v[0].hypot(v[1]);
    if len < 1e-12 {
        [0.0, 0.0]
    } else {
        [v[0] / len, v[1] / len]
    }
}

/// The length of the polyline `pts`, in metres.
pub fn length(pts: &[Pt]) -> f64 {
    pts.windows(2).map(|p| (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1])).sum()
}

/// How far the path `a → b → c` turns at `b`, in degrees: positive to the
/// left, negative to the right.
pub fn turn_deg(a: Pt, b: Pt, c: Pt) -> f64 {
    let (u, v) = (unit([b[0] - a[0], b[1] - a[1]]), unit([c[0] - b[0], c[1] - b[1]]));
    (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]).to_degrees()
}

/// The point of the segment `ab` nearest `p`.
pub fn nearest_on_segment(a: Pt, b: Pt, p: Pt) -> Pt {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    [a[0] + dx * t, a[1] + dy * t]
}

/// The distance from `p` to the segment `ab`.
pub fn segment_distance(a: Pt, b: Pt, p: Pt) -> f64 {
    let f = nearest_on_segment(a, b, p);
    (p[0] - f[0]).hypot(p[1] - f[1])
}

/// Where the segments `ab` and `cd` properly cross — each strictly
/// separating the other's ends — or `None`. A touch at an endpoint is not
/// a crossing: that is how a junction is drawn, and how two pieces of one
/// way meet.
pub fn proper_crossing(a: Pt, b: Pt, c: Pt, d: Pt) -> Option<Pt> {
    let side = |p: Pt, q: Pt, r: Pt| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    let (s1, s2) = (side(a, b, c), side(a, b, d));
    let (s3, s4) = (side(c, d, a), side(c, d, b));
    if !(s1 * s2 < 0.0 && s3 * s4 < 0.0) {
        return None;
    }
    let t = s3 / (s3 - s4);
    Some([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t])
}

/// `pts` resampled every `step` metres, the original vertices kept, the
/// last point always present.
pub fn resample(pts: &[Pt], step: f64) -> Vec<Pt> {
    let mut out: Vec<Pt> = Vec::new();
    let Some(&first) = pts.first() else {
        return out;
    };
    out.push(first);
    for pair in pts.windows(2) {
        let (p, q) = (pair[0], pair[1]);
        let len = (q[0] - p[0]).hypot(q[1] - p[1]);
        let n = (len / step).floor() as usize;
        for k in 1..=n {
            let t = k as f64 * step / len;
            if t < 1.0 - 1e-9 {
                out.push([p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]);
            }
        }
        out.push(q);
    }
    out
}

/// The part of `pts` between arc `a0` and arc `a1`, the interior vertices
/// kept and the two ends interpolated onto the polyline.
pub fn between(pts: &[Pt], a0: f64, a1: f64) -> Vec<Pt> {
    if pts.len() < 2 || !(a1 > a0) {
        return Vec::new();
    }
    let mut out: Vec<Pt> = Vec::new();
    let mut at = 0.0f64;
    for pair in pts.windows(2) {
        let (p, q) = (pair[0], pair[1]);
        let len = (q[0] - p[0]).hypot(q[1] - p[1]);
        let next = at + len;
        if next < a0 - 1e-12 {
            at = next;
            continue;
        }
        if at > a1 + 1e-12 {
            break;
        }
        if out.is_empty() {
            let t = if len > 0.0 { ((a0 - at) / len).clamp(0.0, 1.0) } else { 0.0 };
            out.push(lerp(p, q, t));
        }
        if next <= a1 + 1e-12 {
            if out.last() != Some(&q) {
                out.push(q);
            }
        } else {
            let t = if len > 0.0 { ((a1 - at) / len).clamp(0.0, 1.0) } else { 1.0 };
            let end = lerp(p, q, t);
            if out.last() != Some(&end) {
                out.push(end);
            }
            break;
        }
        at = next;
    }
    if out.len() < 2 {
        Vec::new()
    } else {
        out
    }
}

/// The point a fraction `t` of the way from `p` to `q`.
pub fn lerp(p: Pt, q: Pt, t: f64) -> Pt {
    [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]
}

/// `u` turned a quarter to the left.
pub fn left(u: Pt) -> Pt {
    [-u[1], u[0]]
}

/// `a` moved `s` times `b`.
pub fn add(a: Pt, b: Pt, s: f64) -> Pt {
    [a[0] + b[0] * s, a[1] + b[1] * s]
}

pub fn dot(a: Pt, b: Pt) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

/// The z of the cross product: positive where `b` turns left of `a`.
pub fn cross(a: Pt, b: Pt) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// The arc length at every vertex of `pts`, from its first.
pub fn arcs(pts: &[Pt]) -> Vec<f64> {
    let mut out = Vec::with_capacity(pts.len());
    let mut at = 0.0;
    for (k, p) in pts.iter().enumerate() {
        if k > 0 {
            at += (p[0] - pts[k - 1][0]).hypot(p[1] - pts[k - 1][1]);
        }
        out.push(at);
    }
    out
}

/// `pts` with points inserted so no piece is longer than `step`, the
/// original vertices kept **and a point at every arc in `at`**: the axis is
/// not moved, only sampled. An arc outside the polyline is ignored, and one
/// that lands within a millimetre of a point already there adds nothing.
pub fn densify_at(pts: &[Pt], step: f64, at: &[f64]) -> Vec<Pt> {
    let dense = densify(pts, step);
    if at.is_empty() {
        return dense;
    }
    let total = length(pts);
    let mut cuts: Vec<f64> = at.iter().copied().filter(|s| *s > 0.0 && *s < total).collect();
    cuts.sort_by(f64::total_cmp);
    let mut out: Vec<Pt> = Vec::with_capacity(dense.len() + cuts.len());
    let mut arc = 0.0f64;
    let mut next = 0usize;
    for (i, p) in dense.iter().enumerate() {
        if i > 0 {
            let prev = dense[i - 1];
            let seg = (p[0] - prev[0]).hypot(p[1] - prev[1]);
            while next < cuts.len() && cuts[next] < arc + seg - MERGE_M {
                let t = if seg > 0.0 { ((cuts[next] - arc) / seg).clamp(0.0, 1.0) } else { 0.0 };
                let q = [prev[0] + (p[0] - prev[0]) * t, prev[1] + (p[1] - prev[1]) * t];
                if out.last().is_none_or(|l: &Pt| (q[0] - l[0]).hypot(q[1] - l[1]) > MERGE_M) {
                    out.push(q);
                }
                next += 1;
            }
            arc += seg;
            // A cut that lands on this vertex is served by the vertex.
            while next < cuts.len() && cuts[next] <= arc + MERGE_M {
                next += 1;
            }
        }
        if out.last().is_none_or(|l: &Pt| (p[0] - l[0]).hypot(p[1] - l[1]) > MERGE_M) {
            out.push(*p);
        }
    }
    out
}

/// How near two stations must be, in metres, to count as the same one.
const MERGE_M: f64 = 1e-3;

/// `pts` with points inserted so no piece is longer than `step`, the
/// original vertices kept: the axis is not moved, only sampled.
pub fn densify(pts: &[Pt], step: f64) -> Vec<Pt> {
    let mut out: Vec<Pt> = Vec::new();
    let Some(&first) = pts.first() else {
        return out;
    };
    out.push(first);
    for pair in pts.windows(2) {
        let (p, q) = (pair[0], pair[1]);
        let len = (q[0] - p[0]).hypot(q[1] - p[1]);
        let n = (len / step).ceil().max(1.0) as usize;
        for k in 1..n {
            let t = k as f64 / n as f64;
            out.push([p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]);
        }
        if len > 0.0 {
            out.push(q);
        }
    }
    out
}

/// Cell size of the nearest-segment indices, in metres. A station is at
/// most `NODE_M` (4 m) from the next, so a cell holds a few segments of
/// each axis crossing it.
const CELL_M: f64 = 16.0;

/// Segments on a grid of `CELL_M` cells, for many nearest-segment
/// queries from a point. Both the room's height field and the engineered
/// ground are a nearest-something-and-interpolate, and this is the
/// something: the caller keeps whatever it hangs off each segment.
///
/// Cells are searched in rings about the query's own, and the search
/// stops when the nearest segment found is closer than the ring's own
/// distance: everything outside a ring of `k` cells is at least `k` cells
/// away, so nothing nearer can be left unlooked at.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Nearest {
    pub seg: Vec<(Pt, Pt)>,
    cells: HashMap<(i32, i32), Vec<u32>>,
    /// The occupied cells' bounds, so a query knows when it has searched
    /// everything there is.
    span: Option<(i32, i32, i32, i32)>,
}

impl Nearest {
    pub fn push(&mut self, a: Pt, b: Pt) {
        let i = self.seg.len() as u32;
        self.seg.push((a, b));
        let box_ = [a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])];
        for cell in poly::cells_over(box_, CELL_M) {
            self.cells.entry(cell).or_default().push(i);
            self.span = Some(match self.span {
                None => (cell.0, cell.1, cell.0, cell.1),
                Some((c0, r0, c1, r1)) => (c0.min(cell.0), r0.min(cell.1), c1.max(cell.0), r1.max(cell.1)),
            });
        }
    }

    /// The nearest segment to `p` within `limit` metres: its index, how
    /// far along it the foot lies, and how far `p` is from it.
    ///
    /// The limit is what makes the query cheap where it matters least. A
    /// lattice vertex half a kilometre from the nearest road would
    /// otherwise expand its search until it had scanned the whole index
    /// to learn that the road is far away; with a limit the search stops
    /// at the first ring that cannot hold an answer.
    pub fn of(&self, p: Pt, limit: f64) -> Option<(usize, f64, f64)> {
        self.of_where(p, limit, |_| true)
    }

    /// Every segment within `r` metres of `p`, with the foot's parameter and
    /// the distance. A segment filed under several cells may be visited more
    /// than once, which a max or a min does not mind.
    pub fn within(&self, p: Pt, r: f64, mut f: impl FnMut(usize, f64, f64)) {
        for cell in poly::cells_over([p[0] - r, p[1] - r, p[0] + r, p[1] + r], CELL_M) {
            for &i in self.cells.get(&cell).into_iter().flatten() {
                let (a, b) = self.seg[i as usize];
                let f0 = nearest_on_segment(a, b, p);
                let d = (p[0] - f0[0]).hypot(p[1] - f0[1]);
                if d > r {
                    continue;
                }
                let len2 = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
                let t = if len2 > 0.0 { ((f0[0] - a[0]) * (b[0] - a[0]) + (f0[1] - a[1]) * (b[1] - a[1])) / len2 } else { 0.0 };
                f(i as usize, t, d);
            }
        }
    }

    /// The same, among the segments `keep` accepts.
    pub fn of_where(&self, p: Pt, limit: f64, keep: impl Fn(usize) -> bool) -> Option<(usize, f64, f64)> {
        let (c0, r0) = poly::cell_of(p, CELL_M);
        let (bc0, br0, bc1, br1) = self.span?;
        let rings = (c0 - bc0).abs().max((bc1 - c0).abs()).max((r0 - br0).abs()).max((br1 - r0).abs());
        let mut best: Option<(usize, f64, f64)> = None;
        for k in 0..=rings {
            for (c, r) in ring(c0, r0, k) {
                let Some(ids) = self.cells.get(&(c, r)) else {
                    continue;
                };
                for &i in ids.iter().filter(|&&i| keep(i as usize)) {
                    let (a, b) = self.seg[i as usize];
                    let f = nearest_on_segment(a, b, p);
                    let d = (p[0] - f[0]).hypot(p[1] - f[1]);
                    if best.is_some_and(|(_, _, bd)| d >= bd) {
                        continue;
                    }
                    let len2 = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
                    let t = if len2 > 0.0 {
                        ((f[0] - a[0]) * (b[0] - a[0]) + (f[1] - a[1]) * (b[1] - a[1])) / len2
                    } else {
                        0.0
                    };
                    best = Some((i as usize, t, d));
                }
            }
            let reached = k as f64 * CELL_M;
            if best.is_some_and(|(_, _, d)| d <= reached) || reached > limit {
                break;
            }
        }
        best.filter(|(_, _, d)| *d <= limit)
    }
}

/// A weight falling smoothly from 1 at `t ≤ 0` to 0 at `t ≥ 1`, with no
/// slope at either end, so a blend faded by it has no crease where it
/// starts or stops.
pub fn fade(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    1.0 - t * t * (3.0 - 2.0 * t)
}

/// The cells exactly `k` away from `(c0, r0)` in the Chebyshev metric, in
/// a fixed order: the query's answer must not depend on a hash order.
fn ring(c0: i32, r0: i32, k: i32) -> Vec<(i32, i32)> {
    if k == 0 {
        return vec![(c0, r0)];
    }
    let mut out = Vec::with_capacity(8 * k as usize);
    for c in c0 - k..=c0 + k {
        for r in r0 - k..=r0 + k {
            if c == c0 - k || c == c0 + k || r == r0 - k || r == r0 + k {
                out.push((c, r));
            }
        }
    }
    out
}

/// Where two polylines' interiors properly cross: polyline `a`'s segment
/// `a_seg` and polyline `b`'s segment `b_seg`, with `a < b`, at `at`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Crossing {
    pub a: usize,
    pub a_seg: usize,
    pub b: usize,
    pub b_seg: usize,
    pub at: Pt,
}

/// Every place two of `lines` properly cross ([`proper_crossing`]: a touch at
/// an endpoint is how a junction is drawn, not a crossing), each segment pair
/// once, sorted by `(a, b, a_seg, b_seg)` so the answer is a function of the
/// lines alone. The segments are bucketed on a grid of [`poly::CELL_M`], so
/// only segments that share a cell are compared.
pub fn crossings(lines: &[&[Pt]]) -> Vec<Crossing> {
    let mut cells: HashMap<(i32, i32), Vec<(usize, usize)>> = HashMap::new();
    for (i, pts) in lines.iter().enumerate() {
        for k in 1..pts.len() {
            let (p, q) = (pts[k - 1], pts[k]);
            let box_ = [p[0].min(q[0]), p[1].min(q[1]), p[0].max(q[0]), p[1].max(q[1])];
            for cell in poly::cells_over(box_, poly::CELL_M) {
                cells.entry(cell).or_default().push((i, k - 1));
            }
        }
    }
    let mut seen: std::collections::HashSet<(usize, usize, usize, usize)> = Default::default();
    let mut out: Vec<Crossing> = Vec::new();
    for bucket in cells.values() {
        for x in 0..bucket.len() {
            for y in x + 1..bucket.len() {
                let (mut a, mut b) = (bucket[x], bucket[y]);
                if a.0 == b.0 {
                    continue;
                }
                if a.0 > b.0 {
                    std::mem::swap(&mut a, &mut b);
                }
                if !seen.insert((a.0, a.1, b.0, b.1)) {
                    continue;
                }
                let (u, v) = (lines[a.0], lines[b.0]);
                if let Some(at) = proper_crossing(u[a.1], u[a.1 + 1], v[b.1], v[b.1 + 1]) {
                    out.push(Crossing { a: a.0, a_seg: a.1, b: b.0, b_seg: b.1, at });
                }
            }
        }
    }
    out.sort_by_key(|c| (c.a, c.b, c.a_seg, c.b_seg));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_turn_is_signed_and_a_segment_is_measured() {
        assert!((turn_deg([0.0, 0.0], [1.0, 0.0], [1.0, 1.0]) - 90.0).abs() < 1e-9);
        assert!((turn_deg([0.0, 0.0], [1.0, 0.0], [1.0, -1.0]) + 90.0).abs() < 1e-9);
        assert!((segment_distance([0.0, 0.0], [2.0, 0.0], [3.0, 1.0]) - 2.0f64.sqrt()).abs() < 1e-12);
        assert_eq!(nearest_on_segment([0.0, 0.0], [2.0, 0.0], [1.0, 1.0]), [1.0, 0.0]);
    }

    #[test]
    fn resampling_keeps_vertices_and_spacing() {
        let pts = resample(&[[0.0, 0.0], [2.5, 0.0], [2.5, 1.2]], 1.0);
        assert_eq!(pts, vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.5, 0.0], [2.5, 1.0], [2.5, 1.2]]);
        assert!(resample(&[], 1.0).is_empty());
    }

    #[test]
    fn an_arc_interval_is_cut_on_the_line() {
        let pts = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]];
        assert_eq!(between(&pts, 0.0, 20.0), pts.to_vec());
        assert_eq!(between(&pts, 5.0, 15.0), vec![[5.0, 0.0], [10.0, 0.0], [10.0, 5.0]]);
        assert_eq!(between(&pts, 12.0, 16.0), vec![[10.0, 2.0], [10.0, 6.0]]);
        assert!(between(&pts, 10.0, 10.0).is_empty());
        assert!(between(&[[0.0, 0.0]], 0.0, 1.0).is_empty());
    }

    #[test]
    fn densify_keeps_the_vertices_and_bounds_the_step() {
        let pts = densify(&[[0.0, 0.0], [10.0, 0.0], [10.0, 3.0]], 4.0);
        assert_eq!(pts.len(), 5, "{pts:?}");
        assert_eq!(pts[0], [0.0, 0.0]);
        assert_eq!(pts[3], [10.0, 0.0]);
        assert_eq!(pts[4], [10.0, 3.0]);
        for pair in pts.windows(2) {
            let d = (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]);
            assert!(d <= 4.0 + 1e-12 && d > 0.0, "{pair:?}");
        }
        assert_eq!(densify(&[[1.0, 1.0]], 4.0), vec![[1.0, 1.0]]);
    }
}
