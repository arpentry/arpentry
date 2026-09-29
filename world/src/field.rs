//! The height fields: a height anywhere in plan, from the solved profiles.
//!
//! [`Field`] is the room's: every point rides the cross-section of the
//! nearest carriageway axis, and the legs of a junction are blended. The
//! lift raises the paving by it ([`crate::copies::Fields`]), the earthwork
//! reads it again, and the structure step reads the decks by it, so it is a
//! module of its own rather than any one step's.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use crate::line::{fade, Nearest};
use crate::poly::{self, Pt};
use crate::standard::{EARTHWORK_BATTER, MAX_BATTER_FACE_M, ROOM_REACH_M};
use crate::world::{connector, Profile};

/// How far, in metres, a point may be from a carriageway axis and still
/// be asked about: the widest half-width the priors carry, plus the
/// room's reach, plus the run of a face. Past it every rule that reads the
/// field answers "the ground", so the query stops rather than searching the
/// whole world to find out.
const FIELD_LIMIT_M: f64 = 4.5 + ROOM_REACH_M + EARTHWORK_BATTER * MAX_BATTER_FACE_M;

/// How far past [`FIELD_LIMIT_M`], in metres, [`Field::on_axis`] still looks:
/// more than any paved triangle is wide, so a corner is never left without
/// the axis its triangle's centroid found.
const ON_AXIS_SLACK_M: f64 = 10.0;

/// The band, in metres of distance, over which a foot on one segment of an
/// axis hands over to the foot on the next (`Field::along`).
const SEGMENT_BLEND_M: f64 = 1.0;

/// How far along its own axis, in metres of arc, `Field::along` looks for
/// a second foot: two stations' spacing, far short of a hairpin's other leg.
const ALONG_ARC_M: f64 = 8.0;

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
const JOINT_M: f64 = 15.0;

/// Past this many metres from the connector the blend has faded into the
/// nearest axis's own cross-section, and a leg is its own road again.
const JOINT_FADE_M: f64 = 25.0;

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
    /// **It is not continuous.** At `out = 0⁻` this returns `room_h`; at
    /// `out = 0⁺` with a drop past [`MAX_BATTER_FACE_M`] it returns `ground`,
    /// so the surface jumps by the whole drop along the locus
    /// `d = half_w + ROOM_REACH_M` (and again at `FIELD_LIMIT_M`, where
    /// `Field::at` goes `None` and the vertex drapes). The jump is not a
    /// defect — a walk at road height by the kerb with the ground far below
    /// a few metres out *is* a retaining wall — but it must be drawn, and a
    /// positional function cannot draw it: a vertex on either side of the
    /// locus shares `d` with its twin and comes out at the same height. So
    /// the paving is not lifted by this function but by a rule per triangle
    /// (`crate::copies::Rule::drape`), which declares the jump as an edge the
    /// bench closes with a face. The earthwork asks it for a gallery's rim,
    /// which no paving reaches.
    pub fn batter(&self, room_h: f64, ground: f64) -> f64 {
        let out = self.d - self.half_w - ROOM_REACH_M;
        if out <= 0.0 {
            return room_h;
        }
        // Past the reach the walk joins the road's bench only where a
        // face could reach it. A band standing more than one face from
        // the road is not that road's pavement at all — a footway below a
        // switchback would otherwise be hauled into the air by it — and it
        // samples the ground, whatever the road above it is doing.
        let drop = ground - room_h;
        if drop.abs() > MAX_BATTER_FACE_M {
            return ground;
        }
        let slack = out / EARTHWORK_BATTER;
        room_h + drop.clamp(-slack, slack)
    }
}

impl Field {
    /// The field of `profiles`: every station the source did **not** map as
    /// a bridge or a bore, in order. A way is one profile, so the filter is
    /// per station rather than per piece — the at-grade stretches of a way
    /// that also carries a deck are ground, and its deck is the structure
    /// step's.
    #[cfg(test)]
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

    /// The same, over named station ranges of each profile rather than all
    /// of it: a way is one profile, and a caller usually wants one kind of
    /// its runs.
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
    /// to ([`crate::copies::Rule`]). Asked a little past `FIELD_LIMIT_M`, because a corner
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
    /// times that. On a 15 % street it is under a kerb's rise; on the
    /// steepest streets, and round a roundabout climbing a flank, it is a
    /// step of metres inside a single triangle's rule.
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
            // foot already accounts for; blended in, it would flatten the
            // height near every station for a point off the axis.
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
    /// nearest axis alone would step. The joints then weigh by their distance
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
                // corner (`crate::copies::Rule`), weighing against the
                // answering axis would give it full weight and the nearer leg
                // less, so two legs of one junction would disagree at the
                // corners they share.
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

/// The length of a stretch of axis a [`crate::copies::Rule`] names, in metres. Two stretches
/// that meet give their shared boundary the same foot, so they weld; two legs
/// of a hairpin lie more than this apart along it, so they cannot be taken
/// for each other.
pub const PART_M: f64 = 20.0;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::standard::ROOM_REACH_M;
    use crate::world::Solved;

    #[test]
    fn the_field_is_the_profile_at_the_foot() {
        // One axis along x, rising 1 m per 10 m, stationed every 10 m.
        let stations: Vec<crate::world::Station> = (0..=10)
            .map(|k| {
                let x = k as f64 * 10.0;
                crate::world::Station {
                    s: x,
                    p: [x, 0.0],
                    ground: 400.0,
                    reference: 400.0,
                    h: 400.0 + x / 10.0,
                    solved: Solved::Grade,
                }
            })
            .collect();
        let p = Profile {
            way: 0,
            id: "road".into(),
            class: "residential".into(),
            width_m: 5.5,
            spans: vec![crate::world::Span { a0: 0.0, a1: 100.0, kind: crate::world::Kind::Ground }],
            stations,
        };
        let f = Field::new(std::slice::from_ref(&p));
        assert_eq!(f.len(), 10);
        // On the axis, beside it, and past its end: the foot's height, and
        // the perpendicular distance to it.
        for (q, want_h, want_d) in
            [([25.0, 0.0], 402.5, 0.0), ([25.0, 7.0], 402.5, 7.0), ([25.0, -3.0], 402.5, 3.0), ([115.0, 0.0], 410.0, 15.0)]
        {
            let foot = f.at(q).unwrap();
            assert!((foot.h - want_h).abs() < 1e-9 && (foot.d - want_d).abs() < 1e-9, "{q:?}: {foot:?}");
            assert_eq!(foot.half_w, 2.75);
        }
        // Past the room's reach the walk comes down a face at 1 in 2.5
        // and stops where it meets the ground: a 4 m drop daylights 10 m
        // out, a 1 m drop 2.5 m out, and inside the reach there is no
        // face at all.
        let reach = 2.75 + ROOM_REACH_M;
        let batter = |d: f64, ground: f64| Foot { h: 0.0, d, half_w: 2.75, axis: 0, s: 0.0 }.batter(0.0, ground);
        assert_eq!(batter(0.0, 2.0), 0.0);
        assert_eq!(batter(reach, 2.0), 0.0);
        assert_eq!(batter(reach + 2.5, 2.0), 1.0);
        assert_eq!(batter(reach + 5.0, 2.0), 2.0);
        assert_eq!(batter(reach + 30.0, 2.0), 2.0, "daylighted, and the ground beyond");
        assert_eq!(batter(reach + 2.5, -2.0), -1.0, "the fill side is the mirror");
        assert_eq!(batter(reach + 0.001, 0.0), 0.0, "nothing to close, no face");
        // A difference no face may close is not this road's to close: the
        // band is free and takes the ground.
        assert_eq!(batter(reach + 2.5, MAX_BATTER_FACE_M + 0.5), MAX_BATTER_FACE_M + 0.5);
        // Past the limit the field has nothing to say, which is the same
        // answer as a world with no road in it: the ground.
        assert!(f.at([130.0, 0.0]).is_none());
        // A span is not in the field: nothing but ground pieces sets a
        // height the surface reads.
        let mut deck = p.clone();
        deck.spans = vec![crate::world::Span { a0: 0.0, a1: 100.0, kind: crate::world::Kind::Bridge(1) }];
        assert!(Field::new(std::slice::from_ref(&deck)).at([0.0, 0.0]).is_none());
    }
}
