//! Step 5: a mapped sidewalk stands against its street.
//!
//! Overture maps the footway beside a street as an independent line with
//! its own id, one to four metres off the road's axis, and nothing in the
//! data says the two are one street. Buffered on its own the sidewalk
//! floats: a strip of bare ground between the kerb and the pavement that no
//! real street has. This step states the relation and fills the strip.
//!
//! **Attachment.** A pedestrian way is walked in [`STATION_M`] stations.
//! A station is attached to the carriageway nearest it when it lies within
//! [`ATTACH_M`] of that carriageway's *kerb* — its ribbon edge, not its
//! axis, so a wide road is not charged for being wide — and the way is
//! running *along* it: the cosine between the way's tangent, read over a
//! [`TANGENT_HALF_M`] chord because a mapped footway's per-vertex direction
//! wanders, and the road's, is at least [`ALONG`]. Consecutive attached
//! stations form a run; a run shorter than [`RUN_MIN_M`] is a way brushing
//! past a junction mouth, not a sidewalk, and is dropped. The tag is
//! evidence, not a condition: an anonymous footway that qualifies is a
//! sidewalk, which is how the ways Overture blanked the subclass of get
//! drawn (docs/SOURCES.md).
//!
//! A run may also *bridge* a short break: where a sidewalk wraps a corner
//! or rounds a roundabout its chord briefly fails [`ALONG`] against every
//! leg, and a break there left a wedge of bare ground at exactly the place
//! a pavement is continuous. A break under [`BRIDGE_M`] between two
//! attached stretches is attached on distance alone.
//!
//! **The fill.** For every attached station, one rung: the segment from the
//! station's foot on the road axis out to the station, or to
//! `kerb + `[`WALK_MIN_M`] if the mapped line lies closer than that (or
//! under the asphalt), as a quad a little wider than the station spacing.
//! And between every two consecutive stations, the two triangles of the
//! quad `foot₁ foot₂ station₂ station₁`: where the nearest road switches
//! from one leg to another the feet jump, and the triangles sweep the fan
//! between the two rungs that a pair of rungs alone left open. The union of
//! it all is the ladder between the road's axis and the sidewalk's;
//! `pavement = (walk ∪ ladder) − carriageway`. The pavement's inner edge is
//! then the kerb by construction, one number read twice, and its outer edge
//! is the mapped line's own — the data still says how wide the pavement is.
//!
//! **The check.** The summary's `kerb_gap` is the share of kerb stations
//! that a rung crosses but that have no pavement just outside them: bare
//! ground touching a kerb that a sidewalk claims. The rule is what this
//! step exists to hold, so it is measured here, on every run.

use std::collections::HashMap;

use crate::poly::{self, Pt, Shape, Shapes};
use crate::step::Summary;
use crate::width::{self, Family};
use crate::world::{Kerb, Polyline2, World};

/// Sample spacing along a pedestrian way, in metres.
pub const STATION_M: f64 = 1.0;

/// How far from a carriageway's kerb a station may lie and still be that
/// street's pavement. The server's `WALK_ATTACH_M`: eight metres keeps the
/// p99 of tagged sidewalks and cuts the ones that are misattached or
/// genuinely separate.
pub const ATTACH_M: f64 = 8.0;

/// Minimum |cos| between a station's tangent and the road's for the way to
/// count as running along it (the server's `WALK_ALONG`, about 30°).
pub const ALONG: f64 = 0.87;

/// Half the chord a station's tangent is read over, in metres.
pub const TANGENT_HALF_M: f64 = 2.5;

/// Shortest run of attached stations that is a pavement, in metres.
pub const RUN_MIN_M: f64 = 10.0;

/// Longest break between two attached stretches that is bridged on
/// distance alone, in metres: the corner a pavement wraps, the arc between
/// two legs of a roundabout (the server's `WALK_CORNER_MAX_M`).
pub const BRIDGE_M: f64 = 25.0;

/// How far outside a kerb station the pavement is probed for `kerb_gap`,
/// in metres: inside the narrowest pavement drawn.
const PROBE_M: f64 = 0.3;

/// The narrowest pavement drawn, in metres: what a sidewalk mapped under the
/// asphalt still gets outside the kerb.
pub const WALK_MIN_M: f64 = 0.8;

/// Half-width of a rung, in metres. A full metre either side of the station
/// so that consecutive rungs overlap even on the outside of a bend of eight
/// metres radius at the full reach.
const RUNG_HALF_M: f64 = 1.0;

/// Cell size of the road index, in metres: one query reaches the widest
/// half-width plus [`ATTACH_M`].
const CELL_M: f64 = 16.0;

/// Which pedestrian classes may be a street's pavement. A stair beside a
/// road is a stair, and a farm track beside a road is not its sidewalk.
fn may_attach(class: &str) -> bool {
    matches!(class, "footway" | "path" | "cycleway")
}

/// Attaches the world's pedestrian ways and fills the strips.
pub fn run(world: &mut World) -> Summary {
    let roads = world.roads.as_ref().expect("the drape step runs first");
    let surface = world.surface.as_ref().expect("the surface step runs first");
    let index = RoadIndex::build(roads.plan.iter().filter(|w| width::family(&w.class) == Family::Carriageway));
    // One union per run, then one of the runs: a run's few hundred pieces
    // overlap each other four or five deep, and a single union of every
    // run's pieces at once cost six seconds where this costs a fifth of one.
    let mut pieces: Shapes = Vec::new();
    let mut gaps: Vec<f64> = Vec::new();
    let mut attached_all: Vec<Attached> = Vec::new();
    let (mut stations, mut runs) = (0usize, 0usize);
    for way in roads.plan.iter().filter(|w| may_attach(&w.class)) {
        let pts = resample(&way.pts, STATION_M);
        stations += pts.len();
        for run in attach(&pts, &index) {
            runs += 1;
            gaps.extend(run.iter().map(Attached::gap_m));
            attached_all.extend(run.iter().copied());
            pieces.extend(poly::union_all(&ladder(&run)));
        }
    }
    let attached = gaps.len();
    gaps.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    let q = |f: f64| gaps.get(((gaps.len() as f64 - 1.0) * f).round() as usize).copied().unwrap_or(0.0);
    let rungs = poly::union_all(&pieces);
    let mut walk_and_rungs = surface.walk.clone();
    walk_and_rungs.extend(rungs.iter().cloned());
    let u = poly::union_all(&walk_and_rungs);
    let pavement = poly::difference(&u, &surface.carriageway);
    let filled = poly::area(&pavement) - poly::area(&surface.walk);
    let (gap_n, gap_of) = kerb_gap(&surface.carriageway, &pavement, &attached_all);
    let summary = Summary::new()
        .with("stations", stations)
        .with("attached", format!("{attached} ({:.1}%)", pct(attached, stations)))
        .with("runs", runs)
        .with("gap_m", format!("p50={:.1} p90={:.1} max={:.1}", q(0.5), q(0.9), q(1.0)))
        .with("pavement", format!("{}/{}", pavement.len(), holes(&pavement)))
        .with("pavement_m2", format!("{:.0}", poly::area(&pavement)))
        .with("filled_m2", format!("{filled:.0}"))
        .with("kerb_gap", format!("{gap_n}/{gap_of} ({:.2}%)", pct(gap_n, gap_of)));
    world.kerb = Some(Kerb { rungs, pavement, attached: attached_all });
    summary
}

fn pct(n: usize, of: usize) -> f64 {
    if of == 0 {
        0.0
    } else {
        100.0 * n as f64 / of as f64
    }
}

fn holes(shapes: &Shapes) -> usize {
    shapes.iter().map(|s| s.len() - 1).sum()
}

/// One attached station.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attached {
    /// The station on the pedestrian way.
    pub station: Pt,
    /// Its foot on the road's axis.
    pub foot: Pt,
    /// The road's half-width there.
    pub half_m: f64,
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

/// The runs of attached stations along a resampled way.
///
/// Two passes: every station is attached under the full rule, then every
/// break shorter than [`BRIDGE_M`] between two attached stations is
/// attached on distance alone. A break that stays a break splits the run;
/// runs shorter than [`RUN_MIN_M`] are dropped.
pub fn attach(pts: &[Pt], index: &RoadIndex) -> Vec<Vec<Attached>> {
    let reach = (TANGENT_HALF_M / STATION_M).round() as usize;
    let mut at: Vec<Option<Attached>> = Vec::with_capacity(pts.len());
    for (i, &s) in pts.iter().enumerate() {
        let a = pts[i.saturating_sub(reach)];
        let b = pts[(i + reach).min(pts.len() - 1)];
        let tangent = unit([b[0] - a[0], b[1] - a[1]]);
        at.push(index.nearest_kerb(s, Some(tangent)));
    }
    bridge(pts, &mut at, index);
    let mut runs: Vec<Vec<Attached>> = Vec::new();
    let mut run: Vec<Attached> = Vec::new();
    let mut flush = |run: &mut Vec<Attached>| {
        let len = run.len().saturating_sub(1) as f64 * STATION_M;
        if len >= RUN_MIN_M {
            runs.push(std::mem::take(run));
        } else {
            run.clear();
        }
    };
    for a in at {
        match a {
            Some(att) => run.push(att),
            None => flush(&mut run),
        }
    }
    flush(&mut run);
    runs
}

/// Attaches, on distance alone, the stations of every break shorter than
/// [`BRIDGE_M`] that lies between two attached stations. A station in the
/// break with no road within reach at all keeps the break open.
fn bridge(pts: &[Pt], at: &mut [Option<Attached>], index: &RoadIndex) {
    let max = (BRIDGE_M / STATION_M) as usize;
    let mut i = 0;
    while i < at.len() {
        if at[i].is_some() {
            i += 1;
            continue;
        }
        let start = i;
        while i < at.len() && at[i].is_none() {
            i += 1;
        }
        let end = i; // exclusive
        if start == 0 || end == at.len() || end - start > max {
            continue;
        }
        let relaxed: Vec<Option<Attached>> = (start..end).map(|k| index.nearest_kerb(pts[k], None)).collect();
        if relaxed.iter().all(Option::is_some) {
            at[start..end].clone_from_slice(&relaxed);
        }
    }
}

/// The pieces of one run's ladder: a rung per station and the fan between
/// every two consecutive stations.
pub fn ladder(run: &[Attached]) -> Shapes {
    let mut out: Shapes = Vec::new();
    for a in run {
        let r = rung(a);
        if !r.is_empty() {
            out.push(r);
        }
    }
    for pair in run.windows(2) {
        let (p, q) = (pair[0], pair[1]);
        for tri in [[p.foot, q.foot, q.station], [p.foot, q.station, p.station]] {
            if let Some(t) = ccw(tri.to_vec()) {
                out.push(vec![t]);
            }
        }
    }
    out
}

/// `ring` counter-clockwise, or `None` if it has no area.
fn ccw(mut ring: Vec<Pt>) -> Option<Vec<Pt>> {
    let a = poly::ring_area(&ring);
    if a.abs() < 1e-6 {
        return None;
    }
    if a < 0.0 {
        ring.reverse();
    }
    Some(ring)
}

/// The share of kerb stations that a rung crosses and that have bare
/// ground just outside them, as `(gaps, stations)`. A kerb station is
/// claimed when some attached station's rung — the segment from its foot on
/// the axis to the station — passes within the rung's half-width of it,
/// which is the same side of the same road, and not the far kerb across it.
pub fn kerb_gap(carriageway: &Shapes, pavement: &Shapes, attached: &[Attached]) -> (usize, usize) {
    let mut cells: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (i, a) in attached.iter().enumerate() {
        let (c0, r0) = cell(a.foot);
        let (c1, r1) = cell(a.station);
        for c in c0.min(c1)..=c0.max(c1) {
            for r in r0.min(r1)..=r0.max(r1) {
                cells.entry((c, r)).or_default().push(i);
            }
        }
    }
    let reach = RUNG_HALF_M;
    let near = |p: Pt| -> bool {
        let (c, r) = cell(p);
        (-1..=1).any(|dc| {
            (-1..=1).any(|dr| {
                cells.get(&(c + dc, r + dr)).is_some_and(|v| {
                    v.iter().any(|&i| {
                        let a = attached[i];
                        let seg = Seg { a: a.foot, b: a.station, half_m: 0.0 };
                        let f = nearest_on(seg, p);
                        (f[0] - p[0]).hypot(f[1] - p[1]) <= reach
                    })
                })
            })
        })
    };
    let (pavement, asphalt) = (poly::Boxed::new(pavement), poly::Boxed::new(carriageway));
    let (mut gaps, mut n) = (0usize, 0usize);
    for ring in carriageway.iter().flatten() {
        let mut closed = ring.clone();
        closed.push(ring[0]);
        let pts = resample(&closed, STATION_M);
        for pair in pts.windows(2) {
            let (p, q) = (pair[0], pair[1]);
            let d = unit([q[0] - p[0], q[1] - p[1]]);
            // The region is on the left of every ring, outer or hole, so
            // outward is the right-hand normal.
            let probe = [p[0] + d[1] * PROBE_M, p[1] - d[0] * PROBE_M];
            if !near(p) {
                continue;
            }
            n += 1;
            if !pavement.contains(probe) && !asphalt.contains(probe) {
                gaps += 1;
            }
        }
    }
    (gaps, n)
}

impl Attached {
    /// How far outside the kerb the station lies; negative under the asphalt.
    pub fn gap_m(&self) -> f64 {
        (self.station[0] - self.foot[0]).hypot(self.station[1] - self.foot[1]) - self.half_m
    }
}

/// The quad of one rung: from the foot on the axis out to the station, or
/// to `kerb + WALK_MIN_M`, whichever is farther.
pub fn rung(a: &Attached) -> Shape {
    let d = [a.station[0] - a.foot[0], a.station[1] - a.foot[1]];
    let len = d[0].hypot(d[1]);
    if len < 1e-9 {
        // A station on the axis itself has no side to be on.
        return Vec::new();
    }
    let u = [d[0] / len, d[1] / len];
    let n = [-u[1], u[0]];
    let reach = len.max(a.half_m + WALK_MIN_M);
    let (f, e) = (a.foot, [a.foot[0] + u[0] * reach, a.foot[1] + u[1] * reach]);
    let h = RUNG_HALF_M;
    vec![vec![
        [f[0] - n[0] * h, f[1] - n[1] * h],
        [e[0] - n[0] * h, e[1] - n[1] * h],
        [e[0] + n[0] * h, e[1] + n[1] * h],
        [f[0] + n[0] * h, f[1] + n[1] * h],
    ]]
}

fn unit(v: Pt) -> Pt {
    let len = v[0].hypot(v[1]);
    if len < 1e-12 {
        [0.0, 0.0]
    } else {
        [v[0] / len, v[1] / len]
    }
}

/// One road segment, with its half-width.
#[derive(Debug, Clone, Copy)]
struct Seg {
    a: Pt,
    b: Pt,
    half_m: f64,
}

/// The carriageway segments, in a grid so a station's neighbours are one
/// lookup.
pub struct RoadIndex {
    segs: Vec<Seg>,
    cells: HashMap<(i32, i32), Vec<usize>>,
    /// Widest half-width indexed: bounds the query reach.
    max_half_m: f64,
}

impl RoadIndex {
    pub fn build<'a>(ways: impl Iterator<Item = &'a Polyline2>) -> RoadIndex {
        let mut out = RoadIndex { segs: Vec::new(), cells: HashMap::new(), max_half_m: 0.0 };
        for way in ways {
            let half_m = way.width_m / 2.0;
            out.max_half_m = out.max_half_m.max(half_m);
            for pair in way.pts.windows(2) {
                let seg = Seg { a: pair[0], b: pair[1], half_m };
                let i = out.segs.len();
                out.segs.push(seg);
                let (c0, r0) = cell(seg.a);
                let (c1, r1) = cell(seg.b);
                for c in c0.min(c1)..=c0.max(c1) {
                    for r in r0.min(r1)..=r0.max(r1) {
                        out.cells.entry((c, r)).or_default().push(i);
                    }
                }
            }
        }
        out
    }

    /// The nearest carriageway to `s` whose kerb is within reach and, when a
    /// tangent is given, whose direction agrees with it, as an attachment.
    pub fn nearest_kerb(&self, s: Pt, tangent: Option<Pt>) -> Option<Attached> {
        let reach = ATTACH_M + self.max_half_m;
        let span = (reach / CELL_M).ceil() as i32;
        let (c, r) = cell(s);
        let mut best: Option<(f64, Attached)> = None;
        for dc in -span..=span {
            for dr in -span..=span {
                let Some(ids) = self.cells.get(&(c + dc, r + dr)) else {
                    continue;
                };
                for &i in ids {
                    let seg = self.segs[i];
                    let dir = unit([seg.b[0] - seg.a[0], seg.b[1] - seg.a[1]]);
                    if tangent.is_some_and(|t| (dir[0] * t[0] + dir[1] * t[1]).abs() < ALONG) {
                        continue;
                    }
                    let foot = nearest_on(seg, s);
                    let kerb = (s[0] - foot[0]).hypot(s[1] - foot[1]) - seg.half_m;
                    if kerb > ATTACH_M {
                        continue;
                    }
                    if best.is_none_or(|(k, _)| kerb < k) {
                        best = Some((kerb, Attached { station: s, foot, half_m: seg.half_m }));
                    }
                }
            }
        }
        best.map(|(_, a)| a)
    }
}

fn cell(p: Pt) -> (i32, i32) {
    ((p[0] / CELL_M).floor() as i32, (p[1] / CELL_M).floor() as i32)
}

/// The point of `seg` nearest `p`.
fn nearest_on(seg: Seg, p: Pt) -> Pt {
    let d = [seg.b[0] - seg.a[0], seg.b[1] - seg.a[1]];
    let len2 = d[0] * d[0] + d[1] * d[1];
    if len2 < 1e-18 {
        return seg.a;
    }
    let t = (((p[0] - seg.a[0]) * d[0] + (p[1] - seg.a[1]) * d[1]) / len2).clamp(0.0, 1.0);
    [seg.a[0] + d[0] * t, seg.a[1] + d[1] * t]
}

#[cfg(test)]
pub(crate) mod tests {
    use crate::surface;

    use super::*;

    /// A flat world with the network of `spec`, surfaced.
    pub(crate) fn world(spec: &str) -> World {
        let mut w = surface::tests::world(spec);
        surface::run(&mut w);
        w
    }

    #[test]
    fn resampling_keeps_vertices_and_spacing() {
        let pts = resample(&[[0.0, 0.0], [2.5, 0.0], [2.5, 1.2]], 1.0);
        assert_eq!(pts, vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.5, 0.0], [2.5, 1.0], [2.5, 1.2]]);
        assert!(resample(&[], 1.0).is_empty());
    }

    #[test]
    fn a_sidewalk_six_metres_off_reaches_the_kerb() {
        let mut w = world("net:sidewalk?d=6&len=100");
        let s = run(&mut w);
        let k = w.kerb.as_ref().unwrap();
        assert_eq!(k.pavement.len(), 1, "{s}");
        for y in [2.85, 3.5, 4.5, 5.5, 6.9] {
            assert!(poly::contains(&k.pavement, [0.0, y]), "{y}");
            assert!(poly::contains(&k.pavement, [-45.0, y]), "{y}");
        }
        assert!(!poly::contains(&k.pavement, [0.0, 2.6]));
        assert!(!poly::contains(&k.pavement, [0.0, 7.1]));
        // Kerb 2.75 to the mapped outer edge 7: 4.25 m over 100 m, plus the
        // sidewalk's own caps and the last rungs' width past the ends.
        let a = poly::area(&k.pavement);
        assert!(a > 425.0 && a < 445.0, "{a}");
        assert!(s.to_string().contains("runs=1"), "{s}");
    }

    #[test]
    fn a_sidewalk_under_the_asphalt_gets_the_minimum_pavement() {
        let mut w = world("net:sidewalk?d=2&len=100");
        run(&mut w);
        let k = w.kerb.as_ref().unwrap();
        assert!(poly::contains(&k.pavement, [0.0, 2.9]));
        assert!(poly::contains(&k.pavement, [0.0, 3.45]));
        assert!(!poly::contains(&k.pavement, [0.0, 3.65]));
        assert!(!poly::contains(&k.pavement, [0.0, 2.6]));
    }

    #[test]
    fn a_pavement_wraps_a_corner_in_one_piece() {
        let mut w = world("net:corner?d=5&len=200");
        let s = run(&mut w);
        let k = w.kerb.as_ref().unwrap();
        assert_eq!(k.pavement.len(), 1, "{s}");
        // Between the kerb and the mapped line, on both legs.
        assert!(poly::contains(&k.pavement, [-40.0, -3.5]));
        assert!(poly::contains(&k.pavement, [3.5, 40.0]));
        // And round the corner itself.
        assert!(poly::contains(&k.pavement, [3.5, -3.5]));
        assert!(!poly::contains(&k.pavement, [-40.0, 3.5]), "the inner side has no sidewalk");
    }

    /// Every kerb station a sidewalk claims has pavement outside it.
    fn assert_no_kerb_gap(spec: &str) {
        let mut w = world(spec);
        let s = run(&mut w);
        assert!(s.to_string().contains("kerb_gap=0/"), "{spec}: {s}");
    }

    #[test]
    fn a_claimed_kerb_is_never_bare() {
        for spec in ["net:sidewalk?d=6", "net:sidewalk?d=2", "net:corner?d=5", "net:crossing?d=6"] {
            assert_no_kerb_gap(spec);
        }
    }

    #[test]
    fn a_roundabout_keeps_its_pavement_round_every_leg() {
        let mut w = world("net:roundabout?r=15&d=5&len=200");
        let s = run(&mut w);
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
        let k = w.kerb.as_ref().unwrap();
        let carriageway = &w.surface.as_ref().unwrap().carriageway;
        // Just outside the ring's kerb, at every 5°, there is asphalt (a
        // leg) or pavement — never bare ground.
        for deg in (0..360).step_by(5) {
            let a = (deg as f64).to_radians();
            let p = [18.1 * a.cos(), 18.1 * a.sin()];
            assert!(poly::contains(carriageway, p) || poly::contains(&k.pavement, p), "{deg}°");
        }
        // And beside each leg, out to the sidewalk ring.
        for x in [19.0, 21.0, 22.9] {
            assert!(poly::contains(&k.pavement, [x, 3.0]), "{x}");
            assert!(poly::contains(&k.pavement, [3.0, x]), "{x}");
        }
    }

    #[test]
    fn far_ways_and_crossings_do_not_attach() {
        let mut w = world("net:sidewalk?d=12&len=100");
        let s = run(&mut w);
        assert!(s.to_string().contains("runs=0"), "{s}");
        let k = w.kerb.as_ref().unwrap();
        let walk = &w.surface.as_ref().unwrap().walk;
        assert!((poly::area(&k.pavement) - poly::area(walk)).abs() < 1e-6);
        // A crosswalk stub crosses the road: not along it, so not a rung.
        let mut w = world("net:crossing?d=6&len=100");
        let s = run(&mut w);
        assert!(s.to_string().contains("runs=2"), "{s}");
        let k = w.kerb.as_ref().unwrap();
        assert!(!poly::contains(&k.pavement, [0.0, 0.0]));
        assert!(poly::contains(&k.pavement, [0.0, 4.0]));
        assert!(poly::contains(&k.pavement, [0.0, -4.0]));
    }
}
