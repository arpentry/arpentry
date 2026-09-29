//! The height fields: a height anywhere in plan, from the solved profiles.
//!
//! [`Field`] is the room's: every point rides the cross-section of the
//! nearest carriageway axis, and the legs of a junction are blended. The
//! bench lifts the paving by it and the structure step reads the decks by
//! it, so it is a module of its own rather than either step's.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use crate::poly::{self, Pt};
use crate::standard::{EARTHWORK_BATTER, MAX_BENCH_FACE_M, ROOM_REACH_M};
use crate::world::{connector, Profile, Tri};

/// Cell size of the nearest-segment indices, in metres. A station is at
/// most `NODE_M` (4 m) from the next, so a cell holds a few segments of
/// each axis crossing it.
const CELL_M: f64 = 16.0;

/// How far, in metres, a point may be from a carriageway axis and still
/// be asked about: the widest half-width the priors carry, plus the
/// room's reach, plus the run of a face. Past it every rule in this step
/// answers "the ground", so the query stops rather than searching the
/// whole world to find out.
const FIELD_LIMIT_M: f64 = 4.5 + ROOM_REACH_M + EARTHWORK_BATTER * MAX_BENCH_FACE_M;

/// How far past [`FIELD_LIMIT_M`], in metres, [`Field::on_axis`] still looks:
/// more than any paved triangle is wide, so a corner is never left without
/// the axis its triangle's centroid found.
const ON_AXIS_SLACK_M: f64 = 10.0;

/// The band, in metres of distance, over which a foot on one segment of an
/// axis hands over to the foot on the next ([`Field::along`]).
pub const SEGMENT_BLEND_M: f64 = 1.0;

/// How far along its own axis, in metres of arc, [`Field::along`] looks for
/// a second foot: two stations' spacing, far short of a hairpin's other leg.
pub const ALONG_ARC_M: f64 = 8.0;

/// How much farther than the nearest axis, in metres, another leg of the
/// same junction may run from a point and still have a say in its height:
/// the width of the band over which two legs' cross-sections are blended
/// either side of the line where they are equidistant. A disagreement of
/// `Δ` across the line is spread over it, so it adds about `Δ / BLEND_M`
/// of grade there.
pub const BLEND_M: f64 = 4.0;

/// Within this many metres of a connector two or more axes share, the legs
/// meeting there are blended in full: a street's half-width and the room's
/// reach, out along the bisector of a square corner, with a margin.
pub const JOINT_M: f64 = 15.0;

/// Past this many metres from the connector the blend has faded into the
/// nearest axis's own cross-section, and a leg is its own road again.
pub const JOINT_FADE_M: f64 = 25.0;

/// The room's height field: the solved profile of every carriageway axis
/// on the ground, indexed for the nearest-axis query every vertex makes.
///
/// The nearest axis is the road whose cross-section the point rides. Inside
/// a piece's own ribbon that is the piece's own axis, since no other axis
/// comes within its half-width without their ribbons overlapping; in the
/// pavement and the kerb returns it is the nearest road, which is the one
/// the pavement belongs to — except near a junction, where the legs that
/// meet there are blended ([`Field::at`]).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Field {
    at: Nearest,
    /// Per segment, the solved heights of its two ends and the half-width
    /// of the road it belongs to.
    seg: Vec<(f64, f64, f64)>,
    /// Per segment, the arc along its profile of its two ends.
    arc: Vec<(f64, f64)>,
    /// Per segment, the other end of its axis where the axis closes on
    /// itself — a ring whose last segment ends where its first begins.
    wrap: HashMap<u32, u32>,
    /// Per segment, the axis it belongs to: the profile's place among the
    /// ones the field was built from.
    axis: Vec<u32>,
    /// Every connector two or more axes share, with the axes meeting
    /// there, in connector order so a blend sums in an order that is a
    /// function of the world.
    joints: Vec<(Pt, Vec<u32>)>,
    /// The joints on a grid of [`JOINT_FADE_M`] cells.
    joint_cells: HashMap<(i32, i32), Vec<u32>>,
}

/// Segments on a grid of [`CELL_M`] cells, for many nearest-segment
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
    pub(crate) seg: Vec<(Pt, Pt)>,
    cells: HashMap<(i32, i32), Vec<u32>>,
    /// The occupied cells' bounds, so a query knows when it has searched
    /// everything there is.
    span: Option<(i32, i32, i32, i32)>,
}

impl Nearest {
    pub(crate) fn push(&mut self, a: Pt, b: Pt) {
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
    /// to learn that the road is far away, and there are two million of
    /// them; with a limit the search stops at the first ring that cannot
    /// hold an answer.
    pub(crate) fn of(&self, p: Pt, limit: f64) -> Option<(usize, f64, f64)> {
        self.of_where(p, limit, |_| true)
    }

    /// Every segment within `r` metres of `p`, with the foot's parameter and
    /// the distance. A segment filed under several cells may be visited more
    /// than once, which a max or a min does not mind.
    pub(crate) fn within(&self, p: Pt, r: f64, mut f: impl FnMut(usize, f64, f64)) {
        for cell in poly::cells_over([p[0] - r, p[1] - r, p[0] + r, p[1] + r], CELL_M) {
            for &i in self.cells.get(&cell).into_iter().flatten() {
                let (a, b) = self.seg[i as usize];
                let f0 = poly::nearest_on_segment(a, b, p);
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
    fn of_where(&self, p: Pt, limit: f64, keep: impl Fn(usize) -> bool) -> Option<(usize, f64, f64)> {
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
                    let f = poly::nearest_on_segment(a, b, p);
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

/// What the nearest axis says about a point.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Foot {
    /// The road surface's height at the perpendicular foot.
    pub h: f64,
    /// How far the axis runs from the point, in metres.
    pub d: f64,
    /// Half the width of that road: where its asphalt ends.
    pub half_w: f64,
    /// Which axis of the field answered, in the field's own numbering.
    pub axis: u32,
    /// How far along that axis the foot lies, in metres of its profile.
    pub s: f64,
}

impl Foot {
    /// The height of the walk at this foot: `room_h` — the road's height
    /// plus the kerb — while the point is within the room's reach, and
    /// past it a face at [`EARTHWORK_BATTER`] descending to `ground` and
    /// stopping exactly where it meets it.
    ///
    /// The face is a function of the point alone, which is what makes it
    /// cheap: at `d` metres out it may have closed `(d − reach) / 2.5` of
    /// the difference and no more, so it is the ground wherever it has
    /// daylighted and the room's height wherever it has not left the reach.
    ///
    /// **It is not continuous, and the drape branch is where it breaks.**
    /// At `out = 0⁻` this returns `room_h`; at `out = 0⁺` with a drop past
    /// [`MAX_BENCH_FACE_M`] it returns `ground`, so the surface jumps by the
    /// whole drop — 15.641 m on the loop box — along the locus
    /// `d = half_w + ROOM_REACH_M`. The stretched pavement triangles sit
    /// exactly there: p25/p50/p75 of their distance from the carriageway is
    /// 5.67/6.11/6.75 m with 76 % inside 5–8 m, against a p50 of 4.13 m for
    /// the pavement at large. (A second, smaller one of the same kind sits
    /// at [`FIELD_LIMIT_M`], where `Field::at` goes `None` and the vertex
    /// drapes: 8 % of them.)
    ///
    /// The step is not itself the defect — a walk at road height by the
    /// kerb, with the ground fifteen metres down six metres out, *is* a
    /// retaining wall, and `a_footway_leaving_the_room_leaves_it_over_one_face`
    /// says so. The defect is that nothing draws it: every face this step
    /// builds comes off a *rim* (`wall` off the room's outline, `kerb` and
    /// `rail_face` off a mesh's boundary edges), and **1 599 of the box's
    /// 1 801 stretched triangles are fully interior**. A narrow band puts
    /// the step on its own edge, which is a rim — which is why the specimen
    /// corpus is green and only a band wide enough to be probed out to a
    /// facade shows it.
    ///
    /// Two ways out are already ruled out by measurement. Deciding per run
    /// instead of per point is guarded against three lines into that same
    /// test — it dragged the far end of the stub footway 5.88 m into the
    /// air, because one band is legitimately pavement at the kerb and not
    /// pavement twenty metres up the hill. And cutting the band at the
    /// reach so the step falls on a rim does nothing on its own: this
    /// function is positional, so a vertex on the cut and its twin on the
    /// far sheet share `d` and come out at the *same height*, leaving
    /// nothing between them to close (`wall_m2` unmoved at 12 036, interior
    /// stretched triangles 1 599 → 1 856, `unmet` 1.66 % → 6.96 % for the
    /// extra boolean). The cut only pays if each sheet is lifted by its own
    /// rule, which means carrying the split through `Mesh` and `Bench`.
    pub fn batter(&self, room_h: f64, ground: f64) -> f64 {
        let out = self.d - self.half_w - ROOM_REACH_M;
        if out <= 0.0 {
            return room_h;
        }
        // Past the reach the walk joins the road's bench only where a
        // face could reach it. A band standing more than one face from
        // the road is not that road's pavement at all — on the box's
        // flank a footway 20 m out and 21 m below a switchback was
        // otherwise hauled 18 m into the air by it — and it samples the
        // ground, whatever the road above it is doing.
        let drop = ground - room_h;
        if drop.abs() > MAX_BENCH_FACE_M {
            return ground;
        }
        let slack = out / EARTHWORK_BATTER;
        room_h + drop.clamp(-slack, slack)
    }

    /// [`Foot::batter`] as the *limit from outside the reach*: the same
    /// face, with no plateau however small `out` is.
    ///
    /// This is the far sheet's rule, and the difference between the two is
    /// the whole point of meshing the walk in two parts. `batter` returns
    /// `room_h` at `out = 0` while its limit from the right is `ground`
    /// wherever the drop is past [`MAX_BENCH_FACE_M`] — that gap is the
    /// step, and asked positionally both sheets sit exactly at `out = 0`
    /// on the cut, read `room_h`, and agree. Then there is nothing for a
    /// face to close and the step reappears one lattice vertex further
    /// out, inside the far sheet, which is where it already was.
    ///
    /// Deciding by sheet instead puts the step on the cut by construction,
    /// wherever the bench's own `d = half_w + ROOM_REACH_M` locus falls
    /// against the room's polygon-based band. What is left inside the far
    /// sheet is the `|drop| = MAX_BENCH_FACE_M` crossing, which is bounded
    /// by one face rather than by the drop.
    pub fn face(&self, room_h: f64, ground: f64) -> f64 {
        let drop = ground - room_h;
        if drop.abs() > MAX_BENCH_FACE_M {
            return ground;
        }
        let slack = (self.d - self.half_w - ROOM_REACH_M).max(0.0) / EARTHWORK_BATTER;
        room_h + drop.clamp(-slack, slack)
    }
}

impl Field {
    /// The field of `profiles`: every station the source did **not** map as
    /// a bridge or a bore, in order. A way is one profile now, so the filter
    /// is per station rather than per piece — the at-grade stretches of a way
    /// that also carries a deck are ground, and its deck is the structure
    /// step's.
    pub fn new(profiles: &[Profile]) -> Field {
        Field::grounded(profiles.iter())
    }

    /// The same over whichever profiles are given: the roads' field and the
    /// railways' are two, so a pavement beside a railway never rides it.
    pub fn grounded<'a>(profiles: impl Iterator<Item = &'a Profile>) -> Field {
        Field::of_stations(profiles.map(|p| {
            let runs = p.runs();
            let keep: Vec<(usize, usize)> =
                runs.iter().filter(|r| !r.2.is_structure()).map(|r| (r.0, r.1)).collect();
            (p, keep)
        }))
    }

    /// The field of whichever profiles are given. The bench reads the
    /// ground pieces; the structure step reads the spans, to ask whether
    /// a footway is carried on a road's deck rather than on one of its
    /// own.
    pub fn of<'a>(profiles: impl Iterator<Item = &'a Profile>) -> Field {
        Field::of_stations(profiles.map(|p| {
            let last = p.stations.len().saturating_sub(1);
            (p, vec![(0usize, last)])
        }))
    }

    /// The same, over named station ranges of each profile rather than all
    /// of it: a way is one profile now, and a caller usually wants one kind
    /// of its runs.
    pub fn of_stations<'a>(
        profiles: impl Iterator<Item = (&'a Profile, Vec<(usize, usize)>)>,
    ) -> Field {
        let mut f = Field::default();
        // A station keeps its way's own vertices, so wherever ways meet
        // every one of them has a station there: a joint is a connector
        // the stations of two axes share.
        let mut meets: HashMap<(i64, i64), (Pt, Vec<u32>)> = HashMap::new();
        for (axis, (p, ranges)) in profiles.enumerate() {
            let axis = axis as u32;
            let half_w = p.width_m / 2.0;
            for (k0, k1) in ranges {
                if p.stations.is_empty() {
                    continue;
                }
                let (k0, k1) = (k0.min(p.stations.len() - 1), k1.min(p.stations.len() - 1));
                for st in &p.stations[k0..=k1] {
                    let m = meets.entry(connector(st.p)).or_insert((st.p, Vec::new()));
                    if !m.1.contains(&axis) {
                        m.1.push(axis);
                    }
                }
                if k1 == k0 {
                    let st = p.stations[k0];
                    f.push(st.p, st.p, st.h, st.h, half_w, axis, (st.s, st.s));
                    continue;
                }
                for w in p.stations[k0..=k1].windows(2) {
                    f.push(w[0].p, w[1].p, w[0].h, w[1].h, half_w, axis, (w[0].s, w[1].s));
                }
            }
        }
        // Every axis's first and last segment, and whether it closes.
        let mut ends: std::collections::BTreeMap<u32, (u32, u32)> = Default::default();
        for (i, &a) in f.axis.iter().enumerate() {
            let e = ends.entry(a).or_insert((i as u32, i as u32));
            e.1 = i as u32;
        }
        for (first, last) in ends.into_values() {
            let (s0, s1) = (f.at.seg[first as usize], f.at.seg[last as usize]);
            if first != last && connector(s1.1) == connector(s0.0) {
                f.wrap.insert(first, last);
                f.wrap.insert(last, first);
            }
        }
        let mut joints: Vec<_> = meets.into_iter().filter(|(_, (_, m))| m.len() > 1).collect();
        joints.sort_unstable_by_key(|(key, _)| *key);
        for (_, (c, members)) in joints {
            f.joint_cells.entry(poly::cell_of(c, JOINT_FADE_M)).or_default().push(f.joints.len() as u32);
            f.joints.push((c, members));
        }
        f
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn push(&mut self, a: Pt, b: Pt, ha: f64, hb: f64, half_w: f64, axis: u32, arc: (f64, f64)) {
        self.arc.push(arc);
        self.at.push(a, b);
        self.seg.push((ha, hb, half_w));
        self.axis.push(axis);
    }

    /// How many axis pieces the field holds.
    pub fn len(&self) -> usize {
        self.seg.len()
    }

    pub fn is_empty(&self) -> bool {
        self.seg.is_empty()
    }

    /// What the nearest carriageway axis says about `p`, its height blended
    /// with the other legs of any junction that axis meets near `p`. `None`
    /// if the field is empty.
    pub fn at(&self, p: Pt) -> Option<Foot> {
        let (i, t, d) = self.at.of(p, FIELD_LIMIT_M)?;
        let near = self.axis[i];
        Some(Foot { h: self.joined(p, near, self.along(p, i, t, d), d), d, half_w: self.seg[i].2, axis: near, s: self.arc_at(i, t) })
    }

    /// The arc of segment `i` at `t`.
    fn arc_at(&self, i: usize, t: f64) -> f64 {
        let (a, b) = self.arc[i];
        a + (b - a) * t
    }

    /// What axis `axis` says about `p`, whether or not it is the nearest:
    /// the height a triangle's rule gives a corner another axis is nearer
    /// to ([`Rule`]). Asked a little past [`FIELD_LIMIT_M`], because a corner
    /// can be a triangle's width farther than the centroid that chose it.
    ///
    /// **And only the stretch of it within `part`** ([`PART_M`], with half a
    /// part either side). A way is one axis however it winds, and beside a
    /// hairpin — or inside a roundabout's ring — the nearest foot on it can
    /// jump from one leg to the other, metres apart in height: a switch
    /// within one rule, which no split could see.
    pub fn on_axis(&self, p: Pt, axis: u32, part: i32) -> Option<Foot> {
        let (lo, hi) = ((part as f64 - 0.5) * PART_M, (part as f64 + 1.5) * PART_M);
        let (i, t, d) = self.at.of_where(p, FIELD_LIMIT_M + ON_AXIS_SLACK_M, |k| {
            let (a, b) = self.arc[k];
            self.axis[k] == axis && a.max(b) >= lo && a.min(b) <= hi
        })?;
        Some(Foot { h: self.joined(p, axis, self.along(p, i, t, d), d), d, half_w: self.seg[i].2, axis, s: self.arc_at(i, t) })
    }

    /// The solved height `t` of the way along segment `i`.
    fn height(&self, i: usize, t: f64) -> f64 {
        let (ha, hb, _) = self.seg[i];
        ha + (hb - ha) * t
    }

    /// The axis's height for `p`, whose nearest foot is `t` along segment
    /// `i`, `d` away: that foot blended with the feet on the segments either
    /// side of it, by how nearly each is the nearest ([`SEGMENT_BLEND_M`]).
    ///
    /// **The nearest point of a polyline jumps on the inside of a bend.**
    /// Past the bisector of a vertex the foot moves from one segment to the
    /// next, and on the concave side the two feet stand apart along the axis
    /// — by up to a station's spacing — so the height jumps by the grade
    /// times that. On a 15 % street it is under a kerb's rise; on the loop
    /// box's steepest streets, and round a roundabout climbing a flank, it
    /// was a step of metres inside a single triangle's rule.
    fn along(&self, p: Pt, i: usize, t: f64, d: f64) -> f64 {
        let h = self.height(i, t);
        let (mut hs, mut ws) = (h, 1.0);
        // The segments of the same axis within [`ALONG_ARC_M`] of this one,
        // walked end to end both ways — not just the two neighbours: a
        // profile has a station at every span edge, so a segment may be
        // centimetres long and the foot jump clean over it to the next — and
        // across a ring's seam, where the arc restarts but the road does not.
        let mut others: Vec<usize> = Vec::new();
        let (s0, s1) = self.arc[i];
        let mut j = i;
        while j > 0 && self.axis[j - 1] == self.axis[i] && self.arc[j - 1].1 == self.arc[j].0 && s0 - self.arc[j - 1].1 < ALONG_ARC_M {
            j -= 1;
            others.push(j);
        }
        let mut j = i;
        while j + 1 < self.axis.len() && self.axis[j + 1] == self.axis[i] && self.arc[j].1 == self.arc[j + 1].0 && self.arc[j + 1].0 - s1 < ALONG_ARC_M {
            j += 1;
            others.push(j);
        }
        if let Some(&w) = self.wrap.get(&(i as u32)) {
            others.push(w as usize);
        }
        for j in others {
            let (a, b) = self.at.seg[j];
            let len2 = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
            if len2 <= 0.0 {
                continue;
            }
            // **Only a foot inside the neighbour is a second answer.** On the
            // outside of a bend, or along a straight run, the neighbour's
            // nearest point is the vertex the two share, which the nearest
            // foot already accounts for; blended in, it flattened the height
            // near every station for a point off the axis.
            let tj = ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / len2;
            if !(tj > 0.0 && tj < 1.0) {
                continue;
            }
            let f = [a[0] + (b[0] - a[0]) * tj, a[1] + (b[1] - a[1]) * tj];
            let dj = (p[0] - f[0]).hypot(p[1] - f[1]);
            let w = fade((dj - d) / SEGMENT_BLEND_M);
            if w <= 0.0 {
                continue;
            }
            hs += w * self.height(j, tj);
            ws += w;
        }
        hs / ws
    }

    /// `h`, the height the nearest axis `near` gives `p` from `d` metres
    /// away, blended with the other legs of every joint of `near` within
    /// [`JOINT_FADE_M`] of `p`.
    ///
    /// Within one joint each leg weighs by how nearly it is the nearest —
    /// in full at the line where two legs are equidistant, nothing once it
    /// runs [`BLEND_M`] farther than the nearest — so the blend is
    /// symmetric in the legs and continuous across that line, where the
    /// nearest axis alone stepped. The joints then weigh by their distance
    /// from `p`, in full within [`JOINT_M`] and fading to nothing at
    /// [`JOINT_FADE_M`], and whatever weight they leave is the nearest
    /// axis's own. An axis no joint near `p` holds is never asked: two
    /// roads that meet nowhere near keep the step between them.
    fn joined(&self, p: Pt, near: u32, h: f64, d: f64) -> f64 {
        let (c0, r0) = poly::cell_of(p, JOINT_FADE_M);
        let (mut sum, mut weight, mut most) = (0.0, 0.0, 0.0f64);
        for (c, r) in (-1..=1).flat_map(|dc| (-1..=1).map(move |dr| (c0 + dc, r0 + dr))) {
            for &j in self.joint_cells.get(&(c, r)).into_iter().flatten() {
                let (at, members) = &self.joints[j as usize];
                let reach = fade(((p[0] - at[0]).hypot(p[1] - at[1]) - JOINT_M) / (JOINT_FADE_M - JOINT_M));
                if reach <= 0.0 || !members.contains(&near) {
                    continue;
                }
                // Each leg weighs by how nearly it is the *nearest* leg — not
                // the answering one. The two are the same when the nearest
                // axis answers; when a triangle's rule makes another answer a
                // corner ([`Rule`]), weighing against the answering axis
                // gave it full weight and the nearer leg less, so two legs
                // of one junction disagreed at the corners they share.
                let legs: Vec<(f64, f64)> = members
                    .iter()
                    .filter_map(|&m| {
                        if m == near {
                            return Some((h, d));
                        }
                        let (k, t, dm) = self.at.of_where(p, d + BLEND_M, |k| self.axis[k] == m)?;
                        Some((self.along(p, k, t, dm), dm))
                    })
                    .collect();
                let nearest = legs.iter().map(|l| l.1).fold(d, f64::min);
                let (mut hs, mut ws) = (0.0, 0.0);
                for (hm, dm) in legs {
                    let w = fade((dm - nearest) / BLEND_M);
                    hs += w * hm;
                    ws += w;
                }
                sum += reach * hs / ws;
                weight += reach;
                most = most.max(reach);
            }
        }
        (sum + (1.0 - most) * h) / (weight + (1.0 - most))
    }
}

/// A weight falling smoothly from 1 at `t ≤ 0` to 0 at `t ≥ 1`, with no
/// slope at either end, so a blend faded by it has no crease where it
/// starts or stops.
pub(crate) fn fade(t: f64) -> f64 {
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

/// The height the lifted meshes gave every one of their vertices, keyed by
/// position at the kernel's grid, the lowest where two reach one position.
///
/// **The bench does not use this any more** — it reads a vertex's copies by
/// index ([`Copies`]). What still does is the structure step, which asks for
/// the paving's height at points of a deck's own outline, a region built
/// apart from the mesh; that is a lookup by position between two constructions
/// and keeps the same weakness the bench's used to have.
pub(crate) fn seam(tris: &[&Tri]) -> HashMap<[i64; 2], f64> {
    lowest(tris.iter().flat_map(|t| &t.positions))
}

/// The lowest height at each keyed position of `ps`: [`seam`] over bare
/// positions.
fn lowest<'a>(ps: impl IntoIterator<Item = &'a [f64; 3]>) -> HashMap<[i64; 2], f64> {
    let mut out: HashMap<[i64; 2], f64> = HashMap::new();
    for p in ps {
        out.entry(key(*p)).and_modify(|h| *h = h.min(p[2])).or_insert(p[2]);
    }
    out
}

/// Tolerance, in metres, at which a position lookup takes two points to be
/// one: the kernel's grid, since the regions a caller asks about came out of
/// it. Not [`crate::mesh::WELD_M`], which welds a hundred times finer within one
/// mesh. (It used to be said that a point "through one more boolean" lands
/// half a grid away; [`crate::poly`] pins its adapter, so it cannot. Two
/// *different regions* is what a lookup like this has to bridge.)
const SEAM_M: f64 = poly::GRID_M;

pub(crate) fn key(p: impl AsRef<[f64]>) -> [i64; 2] {
    let p = p.as_ref();
    [(p[0] / SEAM_M).round() as i64, (p[1] / SEAM_M).round() as i64]
}

/// The height `map` holds at `p`, if either mesh put a vertex there.
///
/// Rounding to the kernel's grid is most of the answer and not all of it:
/// two points half a grid apart may still fall either side of a cell's
/// edge and key differently. The eight cells around are asked as well, in
/// a fixed order, so the answer is a function of the meshes and not of a
/// rounding. Any hit is within a grid and a half — a seventh of a
/// millimetre — of the point, and what it carries is a height.
pub(crate) fn at(map: &HashMap<[i64; 2], f64>, p: impl AsRef<[f64]>) -> Option<f64> {
    let k = key(p);
    if let Some(h) = map.get(&k) {
        return Some(*h);
    }
    [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]
        .into_iter()
        .find_map(|[dx, dy]| map.get(&[k[0] + dx, k[1] + dy]).copied())
}

/// The length of a stretch of axis a [`Rule`] names, in metres. Two stretches
/// that meet give their shared boundary the same foot, so they weld; two legs
/// of a hairpin lie more than this apart along it, so they cannot be taken
/// for each other.
pub const PART_M: f64 = 20.0;

/// The engineered ground: the natural ground plus an earthwork **residual**
/// that is pinned at the room's outline and falls to nothing at 1 in
/// [`EARTHWORK_BATTER`].
///
/// At an outline vertex the residual is the room's height less the natural
/// ground, held to one face ([`MAX_BENCH_FACE_M`]) either way; a wall at the
/// edge closes whatever is left over. Out from each outline segment it falls
/// at the batter's slope, perpendicular to the segment, and a point takes the
/// nearest segment's batter blended with any segment nearly as near — over
/// [`EARTH_BLEND_M`], narrowing to nothing at the outline, so on the outline
/// it is the pins' own interpolation and the whole ground is one function.
/// Three things follow:
///
/// - **It is continuous.** The rule it replaces took the nearest segment
///   alone, so the ground stepped wherever two segments were equidistant and
///   answered differently — the inside of every kerb return — and between a
///   walled segment and a battered one, whose pins differed by the whole of
///   a drop. The blend removes the first, and a pin that is clamped rather
///   than refused removes the second: a drop past one face is a face of
///   batter and a wall for the rest, not all wall or none.
/// - **It always daylights, within 7.5 m.** A pin is at most one face, so its
///   batter is spent at `B · MAX_BENCH_FACE_M`. The rule it replaces ran its
///   face at an *absolute* 1 in 2.5, which up a hill steeper than that never
///   met the ground and was cut off at 7.5 m with a lip standing in it —
///   `off` read 21 m on the loop box.
/// - **Pins do not reach along the kerb.** A cone from every pin (the
///   steepest-allowed extension, tried first) coupled neighbouring pins: a
///   kerb whose cut changed by more than a batter's slope along its own
///   length had one pin's cone override the next, and each override was a
///   wall at the kerb — 10 k m² more of them on the loop box.
///
/// **The batter is relative to the natural ground, not absolute.** On flat
/// ground the two are the same 1 in 2.5. Across a 30 % hill the cut face
/// stands at 70 % and meets the ground 2 m out; at an absolute 1 in 2.5 it
/// never did. It is plan §3.2's residual — "the batter is `e` falling to
/// nothing" — with a slope where the relax solve had a decay.
///
/// It is a function of the point, not a mesh, so anything may be *proven*
/// to lie on it, which is the property the terrain step set out with and
/// the one the crate is built on.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Ground {
    at: Nearest,
    /// Per outline segment, the residual at its two ends.
    e: Vec<(f64, f64)>,
}

/// How far, in metres, a batter reaches before it is spent: one face at the
/// batter's slope.
const EARTH_REACH_M: f64 = EARTHWORK_BATTER * MAX_BENCH_FACE_M;

/// The band, in metres of distance, over which the batter of one outline
/// segment hands over to the next where the nearest changes ([`Ground`]).
pub const EARTH_BLEND_M: f64 = 1.0;

impl Ground {
    /// The ground benched to the room's outline: one segment per outline
    /// edge of the one mesh, as `(a, b, [room, natural] at a, the same at
    /// b)`. A segment across a tunnel's mouth is pinned at no residual — no
    /// batter runs into the tube — and the wall over the mouth closes the
    /// hill down to its roof.
    pub fn of_edges(edges: &[(Pt, Pt, [f64; 2], [f64; 2])], portals: &crate::portal::Portals) -> Ground {
        let mut g = Ground::default();
        let pin = |[room, natural]: [f64; 2]| (room - natural).clamp(-MAX_BENCH_FACE_M, MAX_BENCH_FACE_M);
        for &(a, b, at_a, at_b) in edges {
            g.at.push(a, b);
            let mouth = portals.open(a) && portals.open(b);
            g.e.push(if mouth { (0.0, 0.0) } else { (pin(at_a), pin(at_b)) });
        }
        g
    }

    /// How many outline pieces the ground is benched to.
    pub fn len(&self) -> usize {
        self.e.len()
    }

    pub fn is_empty(&self) -> bool {
        self.e.is_empty()
    }

    /// The earthwork residual at `p`: what the ground there stands off the
    /// natural ground.
    pub fn residual(&self, p: Pt) -> f64 {
        let mut near: Vec<(f64, f64)> = Vec::new();
        self.at.within(p, EARTH_REACH_M + EARTH_BLEND_M, |i, t, d| {
            // **A batter does not cross the room.** The outline runs with the
            // room on its left, so a point whose foot is inside a segment and
            // which lies on the segment's left is across the paving from it.
            // Beyond a segment's end its batter is its end vertex's, which a
            // neighbouring segment shares, so a corner stays continuous.
            let (a, b) = self.at.seg[i];
            let left = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
            if t > 0.0 && t < 1.0 && left > 0.0 {
                return;
            }
            let (ea, eb) = self.e[i];
            let e = ea + (eb - ea) * t;
            near.push((e.signum() * (e.abs() - d / EARTHWORK_BATTER).max(0.0), d));
        });
        let Some(nearest) = near.iter().map(|x| x.1).min_by(f64::total_cmp) else {
            return 0.0;
        };
        // The segments nearly as near as the nearest, blended by how nearly:
        // continuous where the nearest changes, which is where the rule this
        // replaces stepped. Sorted first, so the sum is a function of the
        // segments and not of the index's visiting order.
        near.sort_by(|x, y| x.1.total_cmp(&y.1).then(x.0.total_cmp(&y.0)));
        // **The band narrows to nothing at the outline**, so there the
        // nearest segment alone answers — the linear interpolation of its own
        // pins, which is the edge the mesh drew — and a vertex pinned on it is
        // the limit of the field around it. At a full metre everywhere, a
        // segment a few decimetres off with a different pin was blended into
        // the outline itself, and the ground stood a jump off every pin.
        let band = EARTH_BLEND_M.min(nearest);
        let (mut sum, mut weight) = (0.0, 0.0);
        for (e, d) in near {
            let w = if band > 0.0 { fade((d - nearest) / band) } else { (d <= nearest) as u8 as f64 };
            sum += w * e;
            weight += w;
        }
        sum / weight
    }

    /// The engineered height at `p`, whose natural ground is `natural`.
    pub fn at(&self, p: Pt, natural: f64) -> f64 {
        natural + self.residual(p)
    }
}
