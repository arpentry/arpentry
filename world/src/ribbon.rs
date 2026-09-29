//! The ribbons: every way as a polygon.
//!
//! The mapped centreline, buffered to the width the reader gave it
//! ([`crate::width::of_way`]), with round joins ([`crate::poly`] says why
//! round) and a cap at each end that is round only where the end is a
//! joint the disc has to close, square everywhere else. Nothing built ends
//! in a semicircle of its own half-width: a driveway stops flat at a
//! garage door, a track at a gate, a footpath at a doorstep, and the
//! turning head a round cap might be taken for is a bulb wider than the
//! road, which the data does not carry. The disc is a join device: at
//! a connector it meets every leg whatever the angle, where a butt cap
//! leaves a notch at every leg that is not collinear. So:
//!
//! - **A dead end is square.** Nothing touches the point, no disc is
//!   needed, none is drawn.
//! - **A carriageway's end is round where another carriageway ends there**
//!   (the disc joins the legs) and square otherwise: a driveway ends flush
//!   against the sidewalk it runs into, whether it shares a connector with
//!   it or was mapped to stop a step short, and a lane that carries on as a
//!   path ends where the path begins, whereas a disc there pokes into the
//!   pavement and leaves a bare crescent either side. An end in the
//!   interior of another road is square too: it lies inside that road's
//!   band either way.
//! - **A pedestrian way's end is round where it meets another way at a
//!   corner** (the disc joins them) and square where another way runs
//!   *through* the point — the interior of a way, or two ways leaving it in
//!   opposite directions, which is how Overture cuts a sidewalk at a
//!   crossing's connector — so a crossing ends flush on the sidewalk's axis
//!   and the two halves of a cut sidewalk meet on a line.
//!
//! Nothing is merged here: a junction is several ribbons lying over one
//! another, and a sidewalk lies wherever it was mapped. The surface step
//! unions them and the kerb step attaches the sidewalks, each a boolean on
//! what this one returns — which is why this one has an oracle nobody had
//! to write: the SVG's `band` group is the same construction drawn by a
//! browser.
//!
//! The ribbon reads the *plan* line, not the draped one: the drape splits a
//! way at every grid line and diagonal, and though collinear vertices do not
//! change a buffer's shape they would triple its vertex count.

use std::collections::{BTreeMap, HashMap};

use crate::poly::{self, Shapes};
use crate::step::Summary;
use crate::width::{self, Family};
use crate::world::{connector, Groups, Network, Polyline2, Ribbon, Ribbons};

/// Two ways leaving a point at least this far apart, in degrees, run
/// through it as one: a sidewalk cut at a crossing's connector. Less is a
/// corner, whose caps stay round to join.
const THROUGH_MIN_DEG: f64 = 135.0;

/// The directions the ways leave each connector in, keyed by the
/// connector: `(way, unit direction)`, one for a way's end, two for an
/// interior vertex.
type Leaving = HashMap<(i64, i64), Vec<(usize, [f64; 2])>>;

/// Buffers every way of the world's network, and every deck span within the
/// group the partition put it in.
///
/// **The span regions are built here, once.** The surface, the legs, the
/// room and the sheet all need them; built once they are one answer, and
/// the steps that read them say so in their signatures.
pub fn run(roads: &Network, groups: &Groups) -> (Ribbons, Summary) {
    let joints = joints(roads);
    let mut out = Ribbons {
        spans: spans_of(roads, &groups.of, &joints, false),
        masks: spans_of(roads, &groups.of, &joints, true),
        joints,
        ..Ribbons::default()
    };
    let (mut contours, mut vertices, mut squared) = (0usize, 0usize, 0usize);
    let mut area = [0.0f64; 3];
    let road_ends = road_ends(&roads.plan);
    let leaving = leaving(&roads.plan);
    for (w, line) in roads.plan.iter().enumerate() {
        let round = caps(w, line, &road_ends, &leaving);
        squared += round.iter().filter(|r| !**r).count();
        let shape = poly::buffer_line_capped(&line.pts, line.width_m, round);
        if shape.is_empty() {
            continue;
        }
        let family = width::family(&line.class);
        contours += shape.iter().map(Vec::len).sum::<usize>();
        vertices += shape.iter().flatten().map(Vec::len).sum::<usize>();
        area[family as usize] += poly::area(&shape);
        out.ribbons.push(Ribbon {
            id: line.id.clone(),
            class: line.class.clone(),
            subclass: line.subclass.clone(),
            family,
            shape,
        });
    }
    let summary = Summary::new()
        .with("ribbons", out.ribbons.len())
        .with("contours", contours)
        .with("vertices", vertices)
        .with("squared_ends", squared)
        .with("carriageway_m2", format!("{:.0}", area[Family::Carriageway as usize]))
        .with("walk_m2", format!("{:.0}", area[Family::Walk as usize]))
        .with("rail_m2", format!("{:.0}", area[Family::Rail as usize]));
    (out, summary)
}

/// How many ends of each family's ways lie at each connector: a carriageway
/// or a railway joins only its own kind.
fn road_ends(plan: &[Polyline2]) -> HashMap<(i64, i64), [usize; 3]> {
    let mut out: HashMap<(i64, i64), [usize; 3]> = HashMap::new();
    for way in plan {
        let n = way.pts.len();
        if n < 2 {
            continue;
        }
        let family = width::family(&way.class);
        for p in [way.pts[0], way.pts[n - 1]] {
            out.entry(connector(p)).or_default()[family as usize] += 1;
        }
    }
    out
}

/// Whether each end of way `w` is round: only where the end is a joint the
/// disc closes. A carriageway's where another carriageway ends there, a
/// railway's where another railway does; a pedestrian way's where another
/// way meets it at a corner. A dead end, a carriageway running into a
/// pedestrian way, a buffer stop, and a pedestrian way cut by one running
/// through are square.
fn caps(w: usize, way: &Polyline2, road_ends: &HashMap<(i64, i64), [usize; 3]>, leaving: &Leaving) -> [bool; 2] {
    let n = way.pts.len();
    if n < 2 {
        return [true, true];
    }
    let family = width::family(&way.class);
    let round = |p: [f64; 2]| match family {
        Family::Carriageway | Family::Rail => {
            road_ends.get(&connector(p)).map_or(0, |e| e[family as usize]) > 1
        }
        Family::Walk => joins_at(leaving, p, w),
    };
    [round(way.pts[0]), round(way.pts[n - 1])]
}

/// The directions every way leaves each of its vertices in.
fn leaving(plan: &[Polyline2]) -> Leaving {
    let mut out: Leaving = HashMap::new();
    for (w, way) in plan.iter().enumerate() {
        let n = way.pts.len();
        for i in 0..n {
            let p = way.pts[i];
            for j in [i.wrapping_sub(1), i + 1] {
                if j < n {
                    let q = way.pts[j];
                    let d = [q[0] - p[0], q[1] - p[1]];
                    let len = d[0].hypot(d[1]);
                    if len > 1e-9 {
                        out.entry(connector(p)).or_default().push((w, [d[0] / len, d[1] / len]));
                    }
                }
            }
        }
    }
    out
}

/// Whether way `w` meets another way at `p` at a corner: some other way
/// has a vertex there, and no two directions other ways leave it in are
/// more than [`THROUGH_MIN_DEG`] apart, which would be a way running
/// through.
fn joins_at(leaving: &Leaving, p: [f64; 2], w: usize) -> bool {
    let others: Vec<[f64; 2]> = leaving
        .get(&connector(p))
        .map(|dirs| dirs.iter().filter(|(o, _)| *o != w).map(|(_, d)| *d).collect())
        .unwrap_or_default();
    let limit = THROUGH_MIN_DEG.to_radians().cos();
    let through = others.iter().enumerate().any(|(i, a)| others[i + 1..].iter().any(|b| a[0] * b[0] + a[1] * b[1] < limit));
    !others.is_empty() && !through
}

/// The deck pieces' ribbons unioned per `(family, group)`, capped round at
/// a joint and square at a free end — and, when `butt` is set, square at a
/// handover to the ground as well.
///
/// **The two answer different questions and want different ends.** Round
/// at a handover ([`crate::world::Ribbons::spans`]) is what *welds* a span's
/// ribbon to the approach it runs onto: a disc of half the road's width
/// centred on the connector, lapping back over the ground ribbon, because a
/// boolean union keeps touching shapes apart and butting them would risk a
/// hairline where the paving must be continuous. Square at a handover
/// ([`crate::world::Ribbons::masks`]) is the **mask** of what is over a deck:
/// the arrangement tags the faces inside it as spanned, and a spanned face
/// does not cut the terrain's hole. Round there, the hole would end in a
/// half-disc wrapped by a semicircular wall where an abutment is a straight
/// face, and the disc would reach back over the approach's embankment and
/// excuse it its earthwork. An abutment is a line across the road; square is
/// that line.
///
/// **Decks only. A bore is never continuous with the ground.** A deck and
/// the road that runs onto it are one surface. A bore's roadway is under
/// the hill and meets the surface only at its portal, which the partition
/// has already given back to the ground as an open cutting; merged, its
/// chord would join the sheet's field and answer for the road above it
/// wherever a hairpin passes over its own tunnel.
///
/// **The caps are decided over every piece, not the decks alone.** Which a
/// piece's end is depends on whether anything else of its family meets it
/// there — a fact about the network, not about the subset being asked for.
/// Counted over the decks alone, an abutment would read as a free end and
/// square off inside the junction it opens onto.
fn spans_of(
    roads: &Network,
    group: &[usize],
    ends: &BTreeMap<(Family, (i64, i64)), usize>,
    butt: bool,
) -> Vec<(Family, usize, Shapes)> {
    // The same count over the **ground** pieces alone, so a span can tell a
    // joint with another span from a handover to the road it runs onto.
    let mut on_ground: std::collections::HashSet<(Family, (i64, i64))> = std::collections::HashSet::new();
    for p in &roads.plan {
        for &e in &p.pts {
            on_ground.insert((crate::width::family(&p.class), crate::world::connector(e)));
        }
    }
    // A `BTreeMap`, so the regions come out in the world's order and not a
    // hasher's: these are an output.
    let mut by: BTreeMap<(Family, usize), Shapes> = BTreeMap::new();
    let ground = roads.plan.len();
    for (k, p) in roads.spans.iter().enumerate() {
        if p.pts.len() < 2 || !p.kind.is_deck() {
            continue;
        }
        let family = crate::width::family(&p.class);
        let round = |e: [f64; 2]| {
            let key = (family, crate::world::connector(e));
            if butt && on_ground.contains(&key) {
                return false;
            }
            ends.get(&key).copied().unwrap_or(0) > 1
        };
        let caps = [round(p.pts[0]), round(p.pts[p.pts.len() - 1])];
        by.entry((family, group[ground + k])).or_default().extend(poly::buffer_line_capped(&p.pts, p.width_m, caps));
    }
    by.into_iter().map(|((family, g), parts)| (family, g, poly::union_all(&parts))).collect()
}

/// How many pieces of a family touch each connector — **at a vertex, not
/// only at an end**.
///
/// More than one is a joint: a cap is round where a piece's end meets
/// something and square where it is free, and an end that lands on another
/// piece's interior has met something. Counted by ends alone it would read
/// as free and square off inside the junction it opens onto — which is also
/// why [`crate::sheet`] asks it here rather than counting again: a span may
/// lie over paving at any connector a family shares, and the two steps must
/// not disagree about which those are.
pub fn joints(roads: &Network) -> BTreeMap<(Family, (i64, i64)), usize> {
    let mut at: BTreeMap<(Family, (i64, i64)), usize> = BTreeMap::new();
    for p in roads.pieces() {
        if p.pts.len() < 2 {
            continue;
        }
        let fam = crate::width::family(&p.class);
        let mut seen: std::collections::HashSet<(i64, i64)> = std::collections::HashSet::new();
        for e in p.pts.iter().copied() {
            let key = crate::world::connector(e);
            if seen.insert(key) {
                *at.entry((fam, key)).or_default() += 1;
            }
        }
    }
    at
}

#[cfg(test)]
pub(crate) mod tests {
    use crate::world::World;
    use crate::pipeline::{self, tests::upto};
    use crate::step::Step;

    use super::*;

    /// A flat world with the network of `spec` and no buildings, ribboned.
    pub(crate) fn world(spec: &str) -> (World, Summary) {
        // The surface steps read the ground pieces, and the partition is what
        // cuts them. On a flat world with no heights to read it is the
        // annotation's own cut, which is what these specimens want — so the
        // vertical steps are left out rather than run over nothing.
        let (w, ran) = pipeline::tests::built("flat", spec, None, 100.0, &upto(Step::Ribbon));
        (w, ran.last())
    }

    #[test]
    fn a_straight_has_its_area() {
        let (w, s) = world("net:straight?len=200");
        let r = w.ribbon.as_ref().unwrap();
        assert_eq!(r.ribbons.len(), 1);
        assert_eq!(r.ribbons[0].family, Family::Carriageway);
        // Square at both ends: nothing meets them.
        let exact = 200.0 * 5.5;
        let a = poly::area(&r.ribbons[0].shape);
        assert!((a - exact).abs() < 1e-3, "{a} vs {exact}");
        assert!(s.to_string().contains("ribbons=1"), "{s}");
    }

    #[test]
    fn a_sidewalk_is_a_walk_ribbon_at_its_width() {
        let (w, _) = world("net:sidewalk?d=6&len=100");
        let r = w.ribbon.as_ref().unwrap();
        let walk = r.ribbons.iter().find(|r| r.id == "walk-n").unwrap();
        assert_eq!(walk.family, Family::Walk);
        assert!(poly::contains(&walk.shape, [0.0, 6.9]));
        assert!(!poly::contains(&walk.shape, [0.0, 7.1]));
        assert!(!poly::contains(&walk.shape, [0.0, 4.9]));
        // The gap between kerb (2.75) and the sidewalk's inner edge (5) is
        // nobody's yet: that is the kerb step's job.
        let road = r.ribbons.iter().find(|r| r.id == "road").unwrap();
        assert!(!poly::contains(&road.shape, [0.0, 4.0]));
        assert!(!poly::contains(&walk.shape, [0.0, 4.0]));
    }

    #[test]
    fn a_driveway_ends_flush_against_the_sidewalk() {
        let (w, s) = world("net:driveway?d=6&len=100");
        let r = w.ribbon.as_ref().unwrap();
        let drive = r.ribbons.iter().find(|r| r.id == "drive").unwrap();
        // Squared off at the sidewalk's axis, 1.5 m either side, and square
        // at its far end too, which meets nothing. Six square ends in all:
        // both of the drive's, the road's two dead ends, and the sidewalk
        // halves' far ends; the halves' ends on the drive's joint meet it
        // at a corner and keep their disc.
        assert!(poly::contains(&drive.shape, [1.4, 6.1]));
        assert!(!poly::contains(&drive.shape, [0.0, 5.9]));
        assert!(poly::contains(&drive.shape, [1.4, 19.9]));
        assert!(!poly::contains(&drive.shape, [0.0, 20.1]));
        assert!(s.to_string().contains("squared_ends=6"), "{s}");
        // Mapped to stop a step short of the sidewalk's axis: flush there
        // too, since no other road ends where it does.
        let (w, s) = world("net:driveway?d=6&short=0.7&len=100");
        let drive = w.ribbon.as_ref().unwrap().ribbons.iter().find(|r| r.id == "drive").unwrap();
        assert!(poly::contains(&drive.shape, [1.4, 6.8]));
        assert!(!poly::contains(&drive.shape, [0.0, 6.6]));
        assert!(s.to_string().contains("squared_ends=6"), "{s}");
        // A road meeting other roads keeps its disc: only the four far ends.
        let (_w, s) = world("net:cross?len=200");
        assert!(s.to_string().contains("squared_ends=4"), "{s}");
    }

    #[test]
    fn a_crossing_ends_flush_on_the_sidewalk_it_meets() {
        // The sidewalks are cut at the crossing's connector, as Overture
        // cuts them; the crossing, a walk's 2 m wide, ends on their 2 m band.
        let (w, s) = world("net:crossing?d=6&len=100");
        let r = w.ribbon.as_ref().unwrap();
        let crossing = r.ribbons.iter().find(|r| r.id == "crossing").unwrap();
        assert!(!poly::contains(&crossing.shape, [0.0, 6.4]), "no ear past the axis: {s}");
        assert!(poly::contains(&crossing.shape, [0.9, 5.9]));
        // The crossing's two, the road's two dead ends, the halves' four far ends.
        assert!(s.to_string().contains("squared_ends=8"), "{s}");
        // The sidewalk halves keep their caps on the crossing's connector:
        // a corner needs its disc.
        let half = r.ribbons.iter().find(|r| r.id == "walk-nw").unwrap();
        assert!(poly::contains(&half.shape, [0.9, 6.0]));
    }

    #[test]
    fn a_cross_is_four_ribbons_that_overlap() {
        let (w, _) = world("net:cross?len=200");
        let r = w.ribbon.as_ref().unwrap();
        assert_eq!(r.ribbons.len(), 4);
        assert!(r.ribbons.iter().all(|x| poly::contains(&x.shape, [0.0, 0.0])));
    }
}
