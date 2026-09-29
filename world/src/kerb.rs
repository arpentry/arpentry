//! The kerb: a mapped sidewalk stands against its street.
//!
//! Overture maps the footway beside a street as an independent line with
//! its own id, one to four metres off the road's axis, and nothing in the
//! data says the two are one street. Buffered on its own the sidewalk
//! floats: a strip of bare ground between the kerb and the pavement that no
//! real street has. This step states the relation and fills the strip.
//!
//! **Attachment.** A pedestrian way is walked in [`STATION_M`] stations.
//! A station is attached to the carriageway nearest it when it lies within
//! `ATTACH_M` of that carriageway's *kerb* — its ribbon edge, not its
//! axis, so a wide road is not charged for being wide — and the way is
//! running *along* it: the cosine between the way's tangent, read over a
//! `TANGENT_HALF_M` chord because a mapped footway's per-vertex direction
//! wanders, and the road's, is at least `ALONG`. Consecutive attached
//! stations form a run; a run shorter than [`RUN_MIN_M`] is a way brushing
//! past a junction mouth, not a sidewalk, and is dropped. The tag is
//! evidence, not a condition: an anonymous footway that qualifies is a
//! sidewalk, which is how the ways Overture blanked the subclass of get
//! drawn (docs/SOURCES.md).
//!
//! A run may also *bridge* a short break: where a sidewalk wraps a corner
//! or rounds a roundabout its chord briefly fails `ALONG` against every
//! leg, and a break there leaves a wedge of bare ground at exactly the
//! place a pavement is continuous. A break under [`SIDEWALK_BRIDGE_M`] between two
//! attached stretches is attached on distance alone, and to a kerb up to
//! `BRIDGE_REACH_M` away rather than `ATTACH_M`, because a sidewalk
//! cutting a corner on the diagonal is farther from both kerbs there than
//! it runs from either along the legs. The break may span a
//! way's end: Overture cuts a sidewalk at the crossing's connector on the
//! corner, so the two halves of a corner are two ways, each of which ends
//! in the stretch that fails the along test and has nothing of its own
//! beyond it to bridge to. A break at a way's end is bridged when the way
//! it meets there is attached at that end (a *weld*), and a way short
//! enough to be a break entirely — the corner piece between two crossings
//! — is bridged when both its ends are welded. A run shorter than
//! [`RUN_MIN_M`] that ends on a weld is kept: it is the corner's piece of
//! a longer pavement, not a way brushing past.
//!
//! A `crosswalk` is attached on distance alone from the start: it crosses
//! the road it belongs to, so the along test could only ever hand it to
//! some other road running parallel to it, and the wedge between the
//! crossing's approach and the kerb it walks toward would stay bare.
//!
//! Where two pedestrian ways meet, the fan is also drawn across the
//! meeting: for every road both stand on near it, between the station of
//! each nearest the meeting that stands on that road (a *joint*). A
//! sidewalk ending at a crossing's corner and the crossing leaving it both
//! stand on the crossed road a few stations from the corner, and the
//! wedge between those two rungs belongs to neither way's ladder.
//!
//! **Landing.** Whatever the along test says, a station standing within
//! `LANDING_M` of a kerb stands on it: the end of a footway that stops a
//! step short of the road, the mouth of a crossing, a sidewalk running past
//! the round end of the road that meets it. Each such station gets a rung
//! on distance alone, with no run to belong to.
//!
//! **The fill.** For every attached station, one rung: the segment from the
//! station's foot on the road axis out to the station, or to [`WALK_MIN_M`]
//! past where the asphalt ends along it if the mapped line lies closer than
//! that (or under the asphalt), as a quad a little wider than the station
//! spacing. Where the asphalt ends is measured along the rung against every
//! road, not read off the attached road's width: in the notch between two
//! legs the rung leaves the asphalt at the other leg's kerb, and a rung
//! that stopped at its own road's kerb plus the minimum would leave a bare
//! sliver there.
//!
//! And between every two consecutive stations, the fan: the convex hull of
//! the two feet, the two stations, and the end of each foot's axis segment
//! toward the other foot. Where the nearest road switches from one leg to
//! another the feet jump, and the fan sweeps what a pair of rungs alone
//! leaves open; the segment ends put the legs' shared corner into it,
//! because the chord from foot to foot cuts off the notch between the two
//! kerbs and would leave it bare at every junction a sidewalk wraps at a
//! distance. The union of it all is the ladder between the road's axis and
//! the sidewalk's; `pavement = (walk ∪ ladder) − carriageway`. The pavement's inner edge is
//! then the kerb by construction, one number read twice, and its outer edge
//! is the mapped line's own — the data still says how wide the pavement is.
//!
//! **The check.** [`check`] reports `kerb_gap` ([`crate::gap::kerb_gaps`]):
//! the share of kerb stations that a rung crosses but that have no pavement
//! just outside them — bare ground touching a kerb that a sidewalk claims.
//! The rule is what this step exists to hold, so it is measured on every
//! run.

use std::collections::{HashMap, HashSet};

use crate::gap::kerb_gaps;
use crate::line::{self, resample};
use crate::poly::{self, Pt, Shape, Shapes};
use crate::standard::{PAVEMENT_HOLE_M2, RUNG_HALF_M, SIDEWALK_BRIDGE_M, STATION_M, WALK_MIN_M};
use crate::step::Summary;
use crate::width::{self, Family};
use crate::world::{connector, Attached, Facade, Kerb, Polyline2, Network, Surface};

/// How far from a carriageway's kerb a station may lie and still be that
/// street's pavement. The server's `WALK_ATTACH_M`: eight metres keeps the
/// p99 of tagged sidewalks and cuts the ones that are misattached or
/// genuinely separate.
const ATTACH_M: f64 = 8.0;

/// Minimum |cos| between a station's tangent and the road's for the way to
/// count as running along it (the server's `WALK_ALONG`, about 30°).
const ALONG: f64 = 0.87;

/// Half the chord a station's tangent is read over, in metres.
const TANGENT_HALF_M: f64 = 2.5;

/// Shortest run of attached stations that is a pavement, in metres.
pub const RUN_MIN_M: f64 = 10.0;

/// How far from a kerb a station in a bridged break may stand, in
/// metres: a sidewalk cutting the corner between two legs on the diagonal
/// is `√2` times farther from either kerb than it is from them along the
/// legs, and a break that stays open there leaves the fan across it
/// undrawn and the corner bare.
const BRIDGE_REACH_M: f64 = 1.5 * ATTACH_M;

/// How far past the road's own scale the asphalt is searched for along a
/// rung, in metres, before the rung falls back to the road's half-width.
const EXIT_SLACK_M: f64 = 0.5;

/// How far outside a kerb a station may stand and still be landed on it,
/// in metres, whichever way its way runs.
const LANDING_M: f64 = 1.5;

/// A station closer than this to its foot on the axis has no side to be
/// on: the direction from foot to station is noise, and a rung laid along
/// it could reach out of the far kerb. No rung is drawn for it.
const RUNG_MIN_M: f64 = 0.5;

/// Passes of bridging across way ends before giving up: each pass can
/// attach a way's end for the next to weld to, and a chain of corner
/// pieces is a few long.
const WELD_PASSES: usize = 8;

/// How far from a foot its axis segment's end may lie and still be a
/// vertex of the fan, in metres. A sidewalk wrapping an acute corner at
/// the full attachment reach has its feet `(reach + half) / tan(θ/2)`
/// from the corner, twenty metres at 40°; the far end of a long segment
/// past that is not the corner.
const FAN_REACH_M: f64 = 3.0 * ATTACH_M;

/// Cell size of the road index, in metres: one query reaches the widest
/// half-width plus [`ATTACH_M`].
const CELL_M: f64 = 16.0;

/// Which pedestrian classes may be a street's pavement. A stair beside a
/// road is a stair, and a farm track beside a road is not its sidewalk; a
/// pedestrian street beside a road is paved up to it.
fn may_attach(class: &str) -> bool {
    matches!(class, "footway" | "path" | "cycleway" | "pedestrian")
}

/// One pedestrian way, stationed.
struct Stationed {
    pts: Vec<Pt>,
    at: Vec<Option<Attached>>,
}

/// Attaches the world's pedestrian ways and fills the strips.
pub fn run(roads: &Network, surface: &Surface, facade: &Facade) -> (Kerb, Summary) {
    let index = RoadIndex::build(roads.plan.iter().filter(|w| width::family(&w.class) == Family::Carriageway));
    // One union per run, then one of the runs: a run's few hundred pieces
    // overlap each other four or five deep, and a single union of every
    // run's pieces at once is many times slower.
    let mut pieces: Shapes = Vec::new();
    let mut gaps: Vec<f64> = Vec::new();
    let mut attached_all: Vec<Attached> = Vec::new();
    let (mut stations, mut runs) = (0usize, 0usize);
    let mut ways: Vec<Stationed> = roads
        .plan
        .iter()
        .filter(|w| may_attach(&w.class))
        .map(|w| {
            let pts = resample(&w.pts, STATION_M);
            let at = attach_stations(&pts, &index, w.subclass == "crosswalk");
            Stationed { pts, at }
        })
        .collect();
    let welded = weld(&mut ways, &index);
    let landed: usize = ways.iter_mut().map(|way| land(way, &index)).sum();
    let welds = welds(&ways);
    let joints = weld_fans(&ways);
    let joints_n = joints.len();
    pieces.extend(poly::union_all(&joints));
    for (i, way) in ways.iter().enumerate() {
        stations += way.pts.len();
        let n = way.pts.len();
        let keep_start = welds.contains(&(connector(way.pts[0]), i));
        let keep_end = welds.contains(&(connector(way.pts[n - 1]), i));
        for run in runs_of(&way.at, keep_start, keep_end) {
            // A run of landings alone is a footway's end, not a pavement.
            if !run.iter().all(|a| a.landed) {
                runs += 1;
            }
            gaps.extend(run.iter().map(Attached::gap_m));
            attached_all.extend(run.iter().copied());
            pieces.extend(poly::union_all(&ladder(&run)));
        }
    }
    let attached = gaps.len();
    gaps.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    let q = |f: f64| gaps.get(((gaps.len() as f64 - 1.0) * f).round() as usize).copied().unwrap_or(0.0);
    let rungs = poly::union_all(&pieces);
    let u = poly::fill_holes_under(poly::union_of(&[&surface.walk, &rungs]), PAVEMENT_HOLE_M2);
    let senior = surface.senior();
    let pavement = facade.pavement(&u, &senior);
    let filled = poly::area(&pavement) - poly::area(&surface.walk);
    let summary = Summary::new()
        .with("stations", stations)
        .with_part("attached", attached, stations)
        .with("runs", runs)
        .with("welded", welded)
        .with("joints", joints_n)
        .with("landed", landed)
        .with("gap_m", format!("p50={:.1} p90={:.1} max={:.1}", q(0.5), q(0.9), q(1.0)))
        .with_m2("filled_m2", filled);
    // The surface as this step leaves it: the pavement is its walk, and the
    // other three are the surface step's, carried through untouched.
    let surface = Surface { walk: pavement, ..surface.clone() };
    (Kerb { surface, attached: attached_all }, summary)
}

/// The pavement the step leaves, and the kerb stations a sidewalk claims
/// with bare ground outside them ([`crate::gap::kerb_gaps`]). A kerb against
/// the track bed is not bare ground: the railway is there.
pub fn check(kerb: &Kerb, facade: &Facade) -> Summary {
    let pavement = &kerb.surface.walk;
    let (gaps, of) = kerb_gaps(&kerb.surface, &kerb.attached, &facade.footprints);
    Summary::new()
        .with_regions("pavement", pavement)
        .with_m2("pavement_m2", poly::area(pavement))
        .with_share("kerb_gap", gaps.len(), of)
}

/// Every station attached under the full rule — on distance alone if the
/// way `crosses` — then every break shorter than [`SIDEWALK_BRIDGE_M`]
/// between two attached stations attached on distance alone.
fn attach_stations(pts: &[Pt], index: &RoadIndex, crosses: bool) -> Vec<Option<Attached>> {
    let reach = (TANGENT_HALF_M / STATION_M).round() as usize;
    let mut at: Vec<Option<Attached>> = Vec::with_capacity(pts.len());
    for (i, &s) in pts.iter().enumerate() {
        let a = pts[i.saturating_sub(reach)];
        let b = pts[(i + reach).min(pts.len() - 1)];
        let tangent = line::unit([b[0] - a[0], b[1] - a[1]]);
        let tangent = if crosses { None } else { Some(tangent) };
        at.push(index.nearest_kerb(s, tangent));
    }
    bridge(pts, &mut at, index);
    at
}

/// The runs of consecutive attached stations. A break splits the run;
/// runs shorter than [`RUN_MIN_M`] are dropped unless they hold a landed
/// station or end on a weld (`keep_start`, `keep_end`: the way's ends).
fn runs_of(at: &[Option<Attached>], keep_start: bool, keep_end: bool) -> Vec<Vec<Attached>> {
    let mut runs: Vec<Vec<Attached>> = Vec::new();
    let mut run: Vec<Attached> = Vec::new();
    let mut kept = false;
    let mut flush = |run: &mut Vec<Attached>, kept: &mut bool| {
        let len = run.len().saturating_sub(1) as f64 * STATION_M;
        if len >= RUN_MIN_M || *kept {
            runs.push(std::mem::take(run));
        } else {
            run.clear();
        }
        *kept = false;
    };
    for (i, a) in at.iter().enumerate() {
        match a {
            Some(att) => {
                kept |= att.landed || (i == 0 && keep_start) || (i == at.len() - 1 && keep_end);
                run.push(*att);
            }
            None => flush(&mut run, &mut kept),
        }
    }
    flush(&mut run, &mut kept);
    runs
}

/// Attaches, on distance alone, the stations of every break shorter than
/// [`SIDEWALK_BRIDGE_M`] that lies between two attached stations. A station
/// in the break with no road within reach at all keeps the break open.
fn bridge(pts: &[Pt], at: &mut [Option<Attached>], index: &RoadIndex) {
    let max = (SIDEWALK_BRIDGE_M / STATION_M) as usize;
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
        relax(pts, at, start..end, index);
    }
}

/// Attaches `range` on distance alone if every station in it can be.
/// Whether it was.
fn relax(pts: &[Pt], at: &mut [Option<Attached>], range: std::ops::Range<usize>, index: &RoadIndex) -> bool {
    let relaxed: Vec<Option<Attached>> =
        range.clone().map(|k| index.nearest_kerb_within(pts[k], None, BRIDGE_REACH_M)).collect();
    if !relaxed.iter().all(Option::is_some) {
        return false;
    }
    for (k, a) in range.zip(relaxed) {
        at[k] = a;
    }
    true
}

/// Every way end, keyed by its connector: `(way, at_start, reach)`, where
/// `reach` is how many stations from that end the way's nearest attached
/// station lies (`None` if it has none).
fn ends(ways: &[Stationed]) -> HashMap<(i64, i64), Vec<(usize, bool, Option<usize>)>> {
    let mut out: HashMap<(i64, i64), Vec<(usize, bool, Option<usize>)>> = HashMap::new();
    for (i, w) in ways.iter().enumerate() {
        let n = w.pts.len();
        let first = w.at.iter().position(Option::is_some);
        let last = w.at.iter().rposition(Option::is_some);
        out.entry(connector(w.pts[0])).or_default().push((i, true, first));
        out.entry(connector(w.pts[n - 1])).or_default().push((i, false, last.map(|l| n - 1 - l)));
    }
    out
}

/// The welds: every `(end, way)` such that the way's end station there is
/// attached and another way ends there too.
fn welds(ways: &[Stationed]) -> HashSet<((i64, i64), usize)> {
    let mut out = HashSet::new();
    for (key, at) in ends(ways) {
        for &(i, _, reach) in &at {
            if reach == Some(0) && at.iter().any(|&(o, _, _)| o != i) {
                out.insert((key, i));
            }
        }
    }
    out
}

/// Bridges breaks across way ends. A way's leading or trailing break of
/// `t` stations is one break with the break of the way it meets there
/// (`u` stations from the shared end to that way's nearest attached
/// station), and is attached on distance alone when `t + u` is under
/// [`SIDEWALK_BRIDGE_M`]; a whole way short enough to be a break is
/// attached when that holds at both its ends. Repeated while it changes
/// anything, up to [`WELD_PASSES`], because each pass attaches ends the
/// next can reach.
/// How many stations it attached.
fn weld(ways: &mut [Stationed], index: &RoadIndex) -> usize {
    let max = (SIDEWALK_BRIDGE_M / STATION_M) as usize;
    let mut total = 0;
    for _ in 0..WELD_PASSES {
        let ends = ends(ways);
        // Whether some other way meets `p` with an attached station within
        // `budget` stations of it.
        let meets = |p: Pt, me: usize, budget: usize| -> bool {
            ends.get(&connector(p)).is_some_and(|v| v.iter().any(|&(o, _, u)| o != me && u.is_some_and(|u| u <= budget)))
        };
        let mut changed = 0;
        for i in 0..ways.len() {
            let n = ways[i].pts.len();
            let first = ways[i].at.iter().position(Option::is_some);
            let (p0, p1) = (ways[i].pts[0], ways[i].pts[n - 1]);
            let Stationed { pts, at } = &mut ways[i];
            match first {
                None => {
                    let t = n - 1;
                    if t <= max && meets(p0, i, max - t) && meets(p1, i, max - t) && relax(pts, at, 0..n, index) {
                        changed += n;
                    }
                }
                Some(first) => {
                    let last = at.iter().rposition(Option::is_some).expect("one is");
                    if first > 0 && first <= max && meets(p0, i, max - first) && relax(pts, at, 0..first, index) {
                        changed += first;
                    }
                    let t = n - 1 - last;
                    if t > 0 && t <= max && meets(p1, i, max - t) && relax(pts, at, last + 1..n, index) {
                        changed += t;
                    }
                }
            }
        }
        total += changed;
        if changed == 0 {
            break;
        }
    }
    total
}

/// Lands every unattached station that stands within [`LANDING_M`] of a
/// kerb. How many.
fn land(way: &mut Stationed, index: &RoadIndex) -> usize {
    let mut n = 0;
    for (k, &p) in way.pts.iter().enumerate() {
        if way.at[k].is_some() {
            continue;
        }
        if let Some(a) = index.nearest_kerb_within(p, None, LANDING_M) {
            way.at[k] = Some(Attached { landed: true, ..a });
            n += 1;
        }
    }
    n
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
        out.extend(fan(&pair[0], &pair[1]));
    }
    out
}

/// The fan between two rungs: the convex hull of their feet and stations
/// and, where the feet lie on different segments, the end of each segment
/// toward the other foot.
fn fan(p: &Attached, q: &Attached) -> Option<Shape> {
    let mut pts = vec![p.foot, q.foot, q.station, p.station];
    // Only where the feet jump segments is there a corner to reach; on one
    // segment the corner points would be its far ends, and a fan reaching
    // them at every station makes the run's union slow.
    if p.seg != q.seg {
        pts.extend(corner_end(p, q.foot));
        pts.extend(corner_end(q, p.foot));
    }
    poly::convex_hull(pts).map(|hull| vec![hull])
}

/// The fans across welds: where two pedestrian ways meet, for every road
/// both stand on within [`SIDEWALK_BRIDGE_M`] of the meeting, the fan
/// between the station of each nearest the meeting that stands on it. A sidewalk
/// ending on a crossing's corner and the crossing leaving it may both
/// stand on the road the crossing crosses, a few stations from the
/// corner, and the wedge between those two rungs is nobody's else.
fn weld_fans(ways: &[Stationed]) -> Shapes {
    let max = (SIDEWALK_BRIDGE_M / STATION_M) as usize;
    let ends = ends(ways);
    // Per way end: the station nearest the end standing on each road.
    let nearest = |i: usize, at_start: bool| -> Vec<(usize, Attached)> {
        let w = &ways[i];
        let n = w.pts.len();
        let order: Vec<usize> = if at_start { (0..n.min(max + 1)).collect() } else { (n.saturating_sub(max + 1)..n).rev().collect() };
        let mut out: Vec<(usize, Attached)> = Vec::new();
        for k in order {
            if let Some(a) = w.at[k] {
                if !out.iter().any(|(r, _)| *r == a.road) {
                    out.push((a.road, a));
                }
            }
        }
        out
    };
    let mut keys: Vec<&(i64, i64)> = ends.keys().collect();
    keys.sort();
    let mut out: Shapes = Vec::new();
    for key in keys {
        let meeting = &ends[key];
        for (x, &(i, si, _)) in meeting.iter().enumerate() {
            for &(j, sj, _) in &meeting[x + 1..] {
                if i == j {
                    continue;
                }
                let (ri, rj) = (nearest(i, si), nearest(j, sj));
                for (road, a) in &ri {
                    if let Some((_, b)) = rj.iter().find(|(r, _)| r == road) {
                        if a.station != b.station {
                            out.extend(fan(a, b));
                        }
                    }
                }
            }
        }
    }
    out
}

/// The end of `a`'s axis segment nearest `toward`, if it is within
/// [`FAN_REACH_M`] of `a`'s foot and not the foot itself.
fn corner_end(a: &Attached, toward: Pt) -> Option<Pt> {
    let d = |p: Pt, q: Pt| (p[0] - q[0]).hypot(p[1] - q[1]);
    let end = if d(a.seg[0], toward) < d(a.seg[1], toward) { a.seg[0] } else { a.seg[1] };
    (d(end, a.foot) > 1e-9 && d(end, a.foot) <= FAN_REACH_M).then_some(end)
}

impl Attached {
    /// How far outside the kerb the station lies; negative under the asphalt.
    pub fn gap_m(&self) -> f64 {
        (self.station[0] - self.foot[0]).hypot(self.station[1] - self.foot[1]) - self.half_m
    }
}

/// The quad of one rung: from the foot on the axis out to the station, or
/// to `WALK_MIN_M` past where the asphalt ends along it, whichever is
/// farther.
pub fn rung(a: &Attached) -> Shape {
    let d = [a.station[0] - a.foot[0], a.station[1] - a.foot[1]];
    let len = d[0].hypot(d[1]);
    if len < RUNG_MIN_M {
        // A station on the axis itself has no side to be on.
        return Vec::new();
    }
    let u = [d[0] / len, d[1] / len];
    let n = [-u[1], u[0]];
    let reach = len.max(a.exit_m + WALK_MIN_M);
    let (f, e) = (a.foot, [a.foot[0] + u[0] * reach, a.foot[1] + u[1] * reach]);
    let h = RUNG_HALF_M;
    vec![vec![
        [f[0] - n[0] * h, f[1] - n[1] * h],
        [e[0] - n[0] * h, e[1] - n[1] * h],
        [e[0] + n[0] * h, e[1] + n[1] * h],
        [f[0] + n[0] * h, f[1] + n[1] * h],
    ]]
}

/// One road segment, with its half-width.
#[derive(Debug, Clone, Copy)]
struct Seg {
    a: Pt,
    b: Pt,
    half_m: f64,
    /// Which way, in the order the index was built from.
    way: usize,
}

/// The carriageway segments, in a grid so a station's neighbours are one
/// lookup.
struct RoadIndex {
    segs: Vec<Seg>,
    cells: HashMap<(i32, i32), Vec<usize>>,
    /// Widest half-width indexed: bounds the query reach.
    max_half_m: f64,
}

impl RoadIndex {
    pub fn build<'a>(ways: impl Iterator<Item = &'a Polyline2>) -> RoadIndex {
        let mut out = RoadIndex { segs: Vec::new(), cells: HashMap::new(), max_half_m: 0.0 };
        for (w, way) in ways.enumerate() {
            let half_m = way.width_m / 2.0;
            out.max_half_m = out.max_half_m.max(half_m);
            for pair in way.pts.windows(2) {
                let seg = Seg { a: pair[0], b: pair[1], half_m, way: w };
                let i = out.segs.len();
                out.segs.push(seg);
                let b = [seg.a[0].min(seg.b[0]), seg.a[1].min(seg.b[1]), seg.a[0].max(seg.b[0]), seg.a[1].max(seg.b[1])];
                for cell in poly::cells_over(b, CELL_M) {
                    out.cells.entry(cell).or_default().push(i);
                }
            }
        }
        out
    }

    /// Every segment in the cells within `reach_m` of `p`, with the
    /// distance from `p` to its axis: `(index, segment, distance)`. A
    /// segment in several of those cells is yielded once per cell.
    fn segs_near(&self, p: Pt, reach_m: f64) -> impl Iterator<Item = (usize, Seg, f64)> + '_ {
        let span = (reach_m / CELL_M).ceil() as i32;
        let (c, r) = poly::cell_of(p, CELL_M);
        (-span..=span)
            .flat_map(move |dc| (-span..=span).map(move |dr| (c + dc, r + dr)))
            .filter_map(move |cell| self.cells.get(&cell))
            .flatten()
            .map(move |&i| {
                let seg = self.segs[i];
                (i, seg, line::segment_distance(seg.a, seg.b, p))
            })
    }

    /// The nearest carriageway to `s` whose kerb is within [`ATTACH_M`] and,
    /// when a tangent is given, whose direction agrees with it, as an
    /// attachment.
    pub fn nearest_kerb(&self, s: Pt, tangent: Option<Pt>) -> Option<Attached> {
        self.nearest_kerb_within(s, tangent, ATTACH_M)
    }

    /// [`Self::nearest_kerb`] with the reach given: how far outside the
    /// kerb `s` may stand.
    ///
    /// The nearest point is found per *way*, and the along test is applied
    /// to the segment that point lies on. Applied per segment it would
    /// pass a better-aligned segment of the same road over the nearer one
    /// that fails it, plant the foot on that segment's end vertex, and run
    /// the rung off at an angle from there — leaving the corner between
    /// the rung's fan and the kerb bare.
    pub fn nearest_kerb_within(&self, s: Pt, tangent: Option<Pt>, reach_m: f64) -> Option<Attached> {
        // Per way: its nearest segment to `s`, by kerb distance.
        let mut nearest: HashMap<usize, (f64, usize)> = HashMap::new();
        for (i, seg, d) in self.segs_near(s, reach_m + self.max_half_m) {
            let kerb = d - seg.half_m;
            if kerb > reach_m {
                continue;
            }
            let e = nearest.entry(seg.way).or_insert((kerb, i));
            // Ties broken by the lower segment index: a function of the
            // input, not of the cell walk.
            if kerb < e.0 || (kerb == e.0 && i < e.1) {
                *e = (kerb, i);
            }
        }
        let mut best: Option<(f64, usize)> = None;
        for &(kerb, i) in nearest.values() {
            let seg = self.segs[i];
            let dir = line::unit([seg.b[0] - seg.a[0], seg.b[1] - seg.a[1]]);
            if tangent.is_some_and(|t| (dir[0] * t[0] + dir[1] * t[1]).abs() < ALONG) {
                continue;
            }
            if best.is_none_or(|(k, j)| kerb < k || (kerb == k && i < j)) {
                best = Some((kerb, i));
            }
        }
        best.map(|(_, i)| {
            let seg = self.segs[i];
            let foot = line::nearest_on_segment(seg.a, seg.b, s);
            let len = (s[0] - foot[0]).hypot(s[1] - foot[1]);
            // The asphalt should end within the road's own scale of the
            // foot or the station; a ray that is still inside past that is
            // running along some other road's ribbon, and chasing it draws
            // rungs ten metres long down the middle of an alley.
            let cap = len.max(seg.half_m) + WALK_MIN_M + EXIT_SLACK_M;
            Attached {
                station: s,
                foot,
                half_m: seg.half_m,
                seg: [seg.a, seg.b],
                road: seg.way,
                exit_m: self.exit_along(foot, s, cap).unwrap_or(seg.half_m),
                landed: false,
            }
        })
    }

    /// How far from `from` toward `toward` the asphalt ends, in metres: the
    /// last point along that ray inside some road's ribbon, found by
    /// stepping out and bisecting the last step. `from` is on an axis, so
    /// it is inside. `None` if the ray is still inside at `cap_m`.
    fn exit_along(&self, from: Pt, toward: Pt, cap_m: f64) -> Option<f64> {
        let d = [toward[0] - from[0], toward[1] - from[1]];
        let len = d[0].hypot(d[1]);
        if len < 1e-9 {
            return None;
        }
        let u = [d[0] / len, d[1] / len];
        let inside = |t: f64| self.covers([from[0] + u[0] * t, from[1] + u[1] * t]);
        let step = 0.25;
        let mut t = 0.0;
        while t + step <= cap_m && inside(t + step) {
            t += step;
        }
        if t + step > cap_m {
            return None;
        }
        let (mut lo, mut hi) = (t, t + step);
        for _ in 0..4 {
            let mid = (lo + hi) / 2.0;
            if inside(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Some(lo)
    }

    /// Whether `p` lies within some indexed way's ribbon: within its
    /// half-width of one of its segments.
    fn covers(&self, p: Pt) -> bool {
        self.segs_near(p, self.max_half_m).any(|(_, seg, d)| d <= seg.half_m + 1e-6)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;
    

    use super::*;
    use crate::gap::claim_between;

    /// A flat world with the network of `spec`, surfaced.
    pub(crate) fn world(spec: &str) -> (World, Summary) {
        let (w, ran) = built("flat", spec, None, 100.0, &upto(Step::Kerb));
        (w, ran.last())
    }

    #[test]
    fn a_short_stretch_between_claims_is_claimed_round_the_ring() {
        let c = |s: &str| s.chars().map(|ch| ch == '1').collect::<Vec<bool>>();
        let straight = |s: &str| vec![0.0; s.len()];
        let cb = |s: &str, max: usize| claim_between(c(s), &straight(s), max, 135.0);
        assert_eq!(cb("1001", 2), c("1111"));
        assert_eq!(cb("10001", 2), c("10001"));
        // Round the end: the stretch from the last claim back to the first.
        assert_eq!(cb("01110", 2), c("11111"));
        assert_eq!(cb("00100", 3), c("00100"), "one claim closes no stretch");
        assert_eq!(cb("00100", 4), c("11111"));
        assert_eq!(cb("0000", 9), c("0000"));
        // A stretch that turns back on itself is a cap, not a corner.
        let turn = [0.0, 60.0, 60.0, 60.0, 0.0];
        assert_eq!(claim_between(c("10001"), &turn, 9, 135.0), c("10001"));
        let turn = [0.0, -45.0, -45.0, 0.0, 0.0];
        assert_eq!(claim_between(c("10001"), &turn, 9, 135.0), c("11111"), "a right angle inward is a corner");
    }

    #[test]
    fn a_sidewalk_six_metres_off_reaches_the_kerb() {
        let (w, s) = world("net:sidewalk?d=6&len=100");
        let k = w.kerb.as_ref().unwrap();
        assert_eq!(k.surface.walk.len(), 1, "{s}");
        for y in [2.85, 3.5, 4.5, 5.5, 6.9] {
            assert!(poly::contains(&k.surface.walk, [0.0, y]), "{y}");
            assert!(poly::contains(&k.surface.walk, [-45.0, y]), "{y}");
        }
        assert!(!poly::contains(&k.surface.walk, [0.0, 2.6]));
        assert!(!poly::contains(&k.surface.walk, [0.0, 7.1]));
        // Kerb 2.75 to the mapped outer edge 7: 4.25 m over 100 m, plus the
        // sidewalk's own caps and the last rungs' width past the ends.
        let a = poly::area(&k.surface.walk);
        assert!(a > 425.0 && a < 445.0, "{a}");
        assert!(s.to_string().contains("runs=1"), "{s}");
    }

    #[test]
    fn a_sidewalk_under_the_asphalt_gets_the_minimum_pavement() {
        let (w, _) = world("net:sidewalk?d=2&len=100");
        let k = w.kerb.as_ref().unwrap();
        assert!(poly::contains(&k.surface.walk, [0.0, 2.9]));
        assert!(poly::contains(&k.surface.walk, [0.0, 3.45]));
        assert!(!poly::contains(&k.surface.walk, [0.0, 3.65]));
        assert!(!poly::contains(&k.surface.walk, [0.0, 2.6]));
    }

    #[test]
    fn a_pavement_wraps_a_corner_in_one_piece() {
        let (w, s) = world("net:corner?d=5&len=200");
        let k = w.kerb.as_ref().unwrap();
        assert_eq!(k.surface.walk.len(), 1, "{s}");
        // Between the kerb and the mapped line, on both legs.
        assert!(poly::contains(&k.surface.walk, [-40.0, -3.5]));
        assert!(poly::contains(&k.surface.walk, [3.5, 40.0]));
        // And round the corner itself.
        assert!(poly::contains(&k.surface.walk, [3.0, -3.0]));
        assert!(!poly::contains(&k.surface.walk, [-40.0, 3.5]), "the inner side has no sidewalk");
    }

    #[test]
    fn a_pavement_cut_at_the_corner_still_wraps_it() {
        // Overture cuts the sidewalk at the crossing's connector on the
        // corner, so each half ends mid-arc, where its chord runs along
        // neither leg: the along test fails at both ends and nothing on the
        // same way is attached beyond them to bridge to.
        let (w, s) = world("net:corner?d=8&split=1&len=200");
        let k = w.kerb.as_ref().unwrap();
        assert_eq!(k.surface.walk.len(), 1, "{s}");
        // From the kerb's corner out to the chamfer, along the diagonal.
        for r in [4.2, 5.0, 5.5] {
            let p = [r / 2.0f64.sqrt(), -r / 2.0f64.sqrt()];
            assert!(poly::contains(&k.surface.walk, p), "{p:?}: {s}");
        }
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
    }

    #[test]
    fn the_fan_reaches_the_corner_between_two_legs() {
        // The sidewalk wraps the corner 8 m off both axes; its chamfer's
        // stations project onto road-w and onto the leg, and the chord
        // between those feet passes 7 m from the notch's corner.
        let (w, s) = world("net:tee?d=8&len=200");
        let k = w.kerb.as_ref().unwrap();
        for r in [3.2, 4.5, 6.0, 8.0] {
            assert!(poly::contains(&k.surface.walk, [-r, r]), "{r}: {s}");
        }
        assert!(!poly::contains(&k.surface.walk, [-2.6, 2.6]), "the notch's corner is asphalt");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
    }

    #[test]
    fn a_footway_ending_near_a_kerb_lands_on_it() {
        // Half a metre short of the kerb: the round cap alone would leave
        // a lens of bare ground between it and the asphalt.
        let (w, s) = world("net:stub?d=0.5&len=100");
        assert!(s.to_string().contains("runs=0"), "{s}");
        assert!(s.to_string().contains("landed=2"), "{s}");
        let k = w.kerb.as_ref().unwrap();
        for y in [2.85, 3.0, 3.2, 4.0] {
            assert!(poly::contains(&k.surface.walk, [0.0, y]), "{y}: {s}");
            assert!(poly::contains(&k.surface.walk, [0.9, y]), "{y}: {s}");
        }
        assert!(!poly::contains(&k.surface.walk, [0.0, 2.6]));
        assert!(!poly::contains(&k.surface.walk, [1.3, 2.9]), "the landing is the footway's width");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
        // A metre under the asphalt, as Overture connects it: cut at the kerb.
        let (w, s) = world("net:stub?d=-1&len=100");
        let k = w.kerb.as_ref().unwrap();
        assert!(poly::contains(&k.surface.walk, [0.0, 2.85]), "{s}");
        assert!(!poly::contains(&k.surface.walk, [0.0, 2.6]));
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
    }

    /// Every kerb station a sidewalk claims has pavement outside it.
    fn assert_no_kerb_gap(spec: &str) {
        let (_w, s) = world(spec);
        assert!(s.to_string().contains("kerb_gap=0/"), "{spec}: {s}");
    }

    #[test]
    fn a_claimed_kerb_is_never_bare() {
        for spec in [
            "net:sidewalk?d=6",
            "net:sidewalk?d=2",
            "net:corner?d=5",
            "net:corner?d=8&split=1",
            "net:crossing?d=6",
            "net:stub?d=0.5",
            "net:driveway?d=6",
            "net:tee?d=8",
        ] {
            assert_no_kerb_gap(spec);
        }
    }

    #[test]
    fn a_roundabout_keeps_its_pavement_round_every_leg() {
        let (w, s) = world("net:roundabout?r=15&d=5&len=200");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
        let k = w.kerb.as_ref().unwrap();
        let carriageway = &w.surface.as_ref().unwrap().carriageway;
        // Just outside the ring's kerb, at every 5°, there is asphalt (a
        // leg) or pavement — never bare ground.
        for deg in (0..360).step_by(5) {
            let a = (deg as f64).to_radians();
            let p = [18.1 * a.cos(), 18.1 * a.sin()];
            assert!(poly::contains(carriageway, p) || poly::contains(&k.surface.walk, p), "{deg}°");
        }
        // And beside each leg, out to the sidewalk ring.
        for x in [19.0, 21.0, 22.9] {
            assert!(poly::contains(&k.surface.walk, [x, 3.0]), "{x}");
            assert!(poly::contains(&k.surface.walk, [3.0, x]), "{x}");
        }
    }

    #[test]
    fn far_ways_and_crossings_do_not_attach() {
        let (w, s) = world("net:sidewalk?d=12&len=100");
        assert!(s.to_string().contains("runs=0"), "{s}");
        let k = w.kerb.as_ref().unwrap();
        let walk = &w.surface.as_ref().unwrap().walk;
        assert!((poly::area(&k.surface.walk) - poly::area(walk)).abs() < 1e-6);
        // A crosswalk stub attaches to the road it crosses on distance
        // alone: a run of its own, from kerb to kerb, beside the four
        // sidewalk halves'.
        let (w, s) = world("net:crossing?d=6&len=100");
        assert!(s.to_string().contains("runs=5"), "{s}");
        assert!(s.to_string().contains("landed=0"), "{s}");
        let k = w.kerb.as_ref().unwrap();
        assert!(!poly::contains(&k.surface.walk, [0.0, 0.0]));
        assert!(poly::contains(&k.surface.walk, [0.0, 4.0]));
        assert!(poly::contains(&k.surface.walk, [0.0, -4.0]));
    }
}
