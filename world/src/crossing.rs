//! The crossings: the one thing that couples the height of two ways.
//!
//! Every height so far has been solved along one axis alone. The profile
//! reads the ground under a piece and the anchors at its ends, the bench
//! reads the profile, the structure reads the bench: nothing anywhere has
//! asked what another way is doing. A grade separation is the one place
//! where it must. Two carriageway axes whose *interiors* cross in plan
//! with no connector between them are not a junction — Overture cuts a way
//! at every connector, so a junction is always a meeting of way ends — and
//! the upper of the two has to clear the lower.
//!
//! The demand is derived, never stored (docs/GENERATION.md §4.5): the
//! crossings are re-found from the axes on every run, so there is no such
//! thing here as a clearance left standing for a deck that turned out not
//! to exist. `orphan` measures that the derivation is total.
//!
//! **What moves is the one further from the ground.** A level ordinal is
//! an ordering, not a height, and the piece the source annotated is the
//! piece that is meant to leave the ground: at a road over a road the
//! bridge lifts, at a road over a tunnel the bore dips (S4 and S6, the
//! same rule read twice). Where the two ordinals are the same magnitude
//! and opposite in sign the deficit is split between them. Two axes at the
//! *same* level with no connector between them is a data error, counted in
//! `same` and not solved — the level crossing §4.5 names, an equality
//! rather than an inequality, belongs to rail and waits for it.
//!
//! **The lift is a floor, not a tent.** A clearance charged at one node and
//! nowhere else would draw a spike in the roadway. The deficit is spread
//! along the network by a Dijkstra from the demand, decaying at the class's
//! ramp grade ([`crate::grade`]) — 15 % on a street, 6 % on a motorway — so
//! 6.5 m of clearance buys 43 m of approach each side and the ramp is the
//! *result* rather than a construction of its own. The floor is then a
//! displacement added to the ground the profile solves against
//! ([`crate::solve::solve_on`]), and everything after it follows with no
//! rule of its own: an approach that ends up [`crate::grade::STRUCTURE_MIN_M`]
//! off the ground reads as a deck by the profile's consequence rule, the
//! bench builds the embankment under the rest of it, and the structure step
//! lays the slab.
//!
//! **A chord is charged whole.** The piece that lifts is always a mapped
//! span (a level ordinal is what makes it the mover), and the profile solves a
//! span as a straight chord between the anchors at its ends — a chain of
//! them as *one* chord. A chord cannot be bent up over the road it crosses,
//! so the demand is charged to every station of the chain: the deck rises
//! level and the approaches carry the ramp. Charging the crossing station
//! alone would lift the abutments by a fraction of what the middle needed
//! and leave the roadway underneath.
//!
//! **Levels solve in order.** The floors are spread one ordinal magnitude
//! at a time, ascending, and the profile re-solves after each: when a piece
//! at level ±2 asks for its clearance everything at ±1 is already final,
//! which is what makes the lower side of a demand a constant rather than
//! another unknown.
//!
//! **A railway is senior, and enters as a constant on both sides.** Where a
//! railway and a road cross, the road moves whatever the ordinals say: it
//! lifts over the rails by [`RAIL_CLEARANCE_M`] and the slab, or dips under
//! a rail deck by a road's headroom. A bore that belongs to a railway may
//! not yield at all, and a road over one is not asked to clear it — §4.5
//! drops the demand rather than lifting a road over a railway that is
//! underground (`senior`; the server's `in_immovable_bore`, which it added
//! after "a tertiary street ramped out of its own portal … on a 9 m
//! embankment nobody built"). Nor does a road's floor travel into a railway
//! through a connector they share: the network the floor spreads on joins a
//! connector's stations only within one of the two, so a lift ramping
//! along a street stops at the level crossing rather than humping the
//! track — the server's "128 % hump in a cable railway" is what a junior
//! charging a senior looks like. Two railways crossing are two peers, and
//! the ordinals decide between them as between two roads.
//!
//! The level crossing is not here: it is a connector the two ways share,
//! and the profile pins both to the railway's height there.

use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::grade;
use crate::poly::{self, Pt};
use crate::solve;
use crate::step::Summary;
use crate::standard::{DECK_THICKNESS_M, WALK_DECK_M};
use crate::width::{self, Family};
use crate::world::{connector, Crossing, Crossings, Kind, Profile, Profiles, Reference, Way};

/// The headroom a roadway needs over the roadway beneath it, in metres:
/// the Swiss norm's 4.5 m plus a construction margin, and the server's
/// number (`data/plans/surface-leaves-the-plane-2026-09-08.md` §5).
pub const ROAD_CLEARANCE_M: f64 = 5.0;

/// The headroom anything needs over a railway, in metres: more than a
/// road's, for the catenary. The server's `priors::RAIL_CLEARANCE_M`.
pub const RAIL_CLEARANCE_M: f64 = 7.0;

/// Slack, in metres, before a clearance counts as short: the solve's own
/// rounding along a chord and a ramp, not a budget.
const CLEARANCE_EPS: f64 = 1e-6;

/// The most one clearance may ask the world to move, in metres.
///
/// **Where the tags and the geometry disagree, the geometry wins**
/// (docs/GENERATION.md §4.5). A real overpass clears the road beneath it by
/// 6.5–10 m, and ~13 m where it stacks over a deck that is itself lifted. A
/// demand far past that is not a crossing the model got wrong by a little; it
/// is a level ordinal that does not describe this pair at all — a service
/// road tagged `is_tunnel` end to end whose plan line happens to cross a
/// motorway viaduct sixty metres below it on the flank. Honouring one such
/// demand moves a kilometre of road to satisfy a tag about something else.
///
/// So a demand past this is **dropped, not capped**: spending fifteen of the
/// sixty-eight metres would leave the geometry wrong *and* distorted, where
/// spending none leaves it merely as the profile solved it. The drop is
/// counted (`unstacked`), never silent.
///
/// The number is the server's, with the same reasoning behind it
/// (`priors::MAX_CLEARANCE_LIFT_M`).
pub const MAX_CLEARANCE_LIFT_M: f64 = 15.0;

/// Builds the floor every crossing demands and re-solves the profile on it.
pub fn run(reference: &Reference, solved: &Profiles) -> (Crossings, Summary) {
    // **Crossings are found on the whole ways, not on the pieces.** This step
    // runs before the partition, so there are no pieces yet — and it wants
    // none: a way is one object with one profile, and a crossing is a place
    // in its interior. The level ordinal there is the way's own
    // ([`Way::level_at_arc`]), which reads the source's structure spans and
    // its bare layers alike.
    //
    // Not every way solves a profile ([`Profile::way`] says which did), so an
    // axis carries `profile: Option<usize>` and one that has none is an
    // orphan rather than a demand.
    let of_way: HashMap<usize, usize> =
        solved.profiles.iter().enumerate().map(|(i, p)| (p.way, i)).collect();
    let axes: Vec<Axis> = reference
        .ways
        .iter()
        .enumerate()
        .filter(|(_, w)| width::family(&w.class).solves())
        .map(|(i, w)| {
            let mut arc = Vec::with_capacity(w.pts.len());
            let mut at = 0.0;
            for (k, p) in w.pts.iter().enumerate() {
                if k > 0 {
                    at += (p[0] - w.pts[k - 1][0]).hypot(p[1] - w.pts[k - 1][1]);
                }
                arc.push(at);
            }
            Axis { way: w, arc, profile: of_way.get(&i).copied() }
        })
        .collect();

    let mut profiles = solved.profiles.clone();
    let net = Net::new(&profiles);
    let mut floor: Vec<Vec<f64>> = profiles.iter().map(|p| vec![0.0; p.stations.len()]).collect();

    // Sort the crossings into the three answers: a data error, a demand we
    // cannot solve because a side has no profile, and a demand.
    let (mut same, mut orphan, mut pairs) = (Vec::new(), 0usize, Vec::new());
    for hit in found(&axes) {
        let (a, b) = (&axes[hit.a], &axes[hit.b]);
        let at = hit.at;
        let (a_arc, b_arc) = (a.arc_at(hit.a_seg, at), b.arc_at(hit.b_seg, at));
        let (a_kind, b_kind) = (a.way.kind_at_arc(a_arc), b.way.kind_at_arc(b_arc));
        // An indoor stretch is not stacked against the ground and takes no
        // part in a demand, whatever its way solved.
        let indoor = a_kind == Kind::Indoor || b_kind == Kind::Indoor;
        // A level is read through the way: a structure's ordinal, or the
        // layer of a stretch the source put at a level without one — a road
        // under a viaduct is on the ground and still under it.
        let (a_level, b_level) = (a.way.level_at_arc(a_arc), b.way.level_at_arc(b_arc));
        // An axis with no profile cannot take part in a demand at all, at
        // whatever level it was drawn: that is the orphan §4.5 requires to
        // read zero, and it is asked before the level, so an indoor way —
        // which is not stacked against the ground and reads level 0 — is
        // not filed as a data error in the road network.
        if a.profile.is_none() || b.profile.is_none() || indoor {
            orphan += 1;
        } else if a_level == b_level {
            same.push(at);
        } else {
            let up_is_a = a_level > b_level;
            let (up, low) = if up_is_a { (a, b) } else { (b, a) };
            let (up_arc, low_arc) = if up_is_a { (a_arc, b_arc) } else { (b_arc, a_arc) };
            let (up_level, low_level) = if up_is_a { (a_level, b_level) } else { (b_level, a_level) };
            let low_kind = if up_is_a { b_kind } else { a_kind };
            let (up_rail, low_rail) = (is_rail(&up.way.class), is_rail(&low.way.class));
            pairs.push(Pair {
                upper: (up.profile.expect("solved"), up_level),
                lower: (low.profile.expect("solved"), low_level),
                up_at: up.way.span_at_arc(up_arc),
                low_at: low.way.span_at_arc(low_arc),
                need: need(&up.way.class, &low.way.class, low_kind),
                at,
                had: 0.0,
                demanded: false,
                unstacked: false,
                up_rail,
                low_rail,
                // A road over a railway's bore: the bore may not yield and
                // the road is not asked to clear it.
                senior: low_rail && !up_rail && matches!(low_kind, Kind::Tunnel(_)),
            });
        }
    }
    for pair in &mut pairs {
        pair.had = separation(&profiles, pair);
    }

    // The anchored connectors and the structure runs at each free one: what
    // a chord runs between, and what it runs across. A *way end* is anchored
    // where the way is on the ground there — the same test the profile step
    // anchors on, and no longer every annotation edge.
    let anchored: HashSet<(i64, i64)> = anchor_connectors(&profiles).collect();
    let mut joins: HashMap<(i64, i64), Vec<Node>> = HashMap::new();
    for (i, p) in profiles.iter().enumerate() {
        let last = p.stations.len().saturating_sub(1);
        for (r, &(k0, k1, kind)) in p.runs().iter().enumerate() {
            if !kind.is_structure() {
                continue;
            }
            for k in [k0, k1] {
                if k != 0 && k != last {
                    continue; // an interior edge: the at-grade solve holds it
                }
                let c = connector(p.stations[k].p);
                if !anchored.contains(&c) {
                    joins.entry(c).or_default().push((i, r));
                }
            }
        }
    }

    // One magnitude of level ordinal at a time, ascending: everything
    // nearer the ground is final when the next one out asks.
    let mut magnitudes: Vec<i64> = pairs.iter().map(|p| p.magnitude()).collect();
    magnitudes.sort_unstable();
    magnitudes.dedup();
    let mut spent = 0usize;
    for m in magnitudes {
        let (mut up, mut down) = (Vec::new(), Vec::new());
        let (mut up_h, mut down_h) = (Vec::new(), Vec::new());
        for pair in pairs.iter_mut().filter(|p| p.magnitude() == m && !p.senior) {
            let short = pair.need - separation(&profiles, pair);
            if short <= CLEARANCE_EPS {
                continue;
            }
            // The tags say one of these is above the other; the solved
            // heights say they are further apart than any crossing accounts
            // for. Believe the heights, and say so.
            if short > MAX_CLEARANCE_LIFT_M {
                pair.unstacked = true;
                continue;
            }
            pair.demanded = true;
            let (lift, dip) = pair.shares(short);
            if lift > 0.0 {
                charge(&mut up, &mut up_h, pair.upper.0, pair.at, pair.up_at, lift, 1.0, &profiles, &joins, &net);
            }
            if dip > 0.0 {
                charge(&mut down, &mut down_h, pair.lower.0, pair.at, pair.low_at, dip, -1.0, &profiles, &joins, &net);
            }
        }
        if up.is_empty() && down.is_empty() {
            continue;
        }
        let none = f64::NEG_INFINITY;
        let (up, down) = (net.spread(&up, 0.0), net.spread(&down, 0.0));
        let (up_h, down_h) = (net.spread(&up_h, none), net.spread(&down_h, none));
        // What the corridor still asks of the floor at a node: whatever the
        // reference does not already give, and never more than the ramp
        // itself allows ([`charge`]).
        let needs = |corridor: f64, ground: f64| {
            if corridor.is_finite() { (corridor - ground).max(0.0) } else { 0.0 }
        };
        for (i, f) in floor.iter_mut().enumerate() {
            for (k, v) in f.iter_mut().enumerate() {
                let (n, r) = (net.base[i] + k, reference.axes[i].h[k]);
                *v += up[n].min(needs(up_h[n], r)) - down[n].min(needs(r, -down_h[n]));
            }
        }
        let (re, _) = solve::solve_on(reference, &reference.ways, &floor);
        profiles = re;
    }
    for f in &floor {
        spent += f.iter().filter(|v| **v != 0.0).count();
    }

    let crossings: Vec<Crossing> = pairs
        .iter()
        .map(|p| Crossing {
            at: p.at,
            upper: p.upper,
            lower: p.lower,
            need: p.need,
            had: p.had,
            have: separation(&profiles, p),
        })
        .collect();
    let demands = pairs.iter().filter(|p| p.demanded).count();
    let unstacked = pairs.iter().filter(|p| p.unstacked).count();
    let senior = pairs.iter().filter(|p| p.senior).count();
    // A pair the geometry overruled is not a clearance the model failed to
    // meet: it is one it declined to believe, and it is counted as such
    // rather than left to inflate `clearance`. A road over a railway's
    // bore is the same: a demand §4.5 does not make.
    let short: Vec<f64> = crossings
        .iter()
        .zip(pairs.iter())
        .filter(|(_, p)| !p.unstacked && !p.senior)
        .map(|(c, _)| c.shortfall())
        .filter(|s| *s > CLEARANCE_EPS)
        .collect();
    let lift = floor.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs()));
    let stations: usize = floor.iter().map(|f| f.len()).sum();
    let summary = Summary::new()
        .with("crossings", crossings.len())
        .with("same", same.len())
        .with("orphan", orphan)
        .with("demands", demands)
        .with("unstacked", unstacked)
        .with("senior", senior)
        .with("lift", format!("{lift:.2}"))
        .with_share("ramped", spent, stations)
        .with_share("clearance", short.len(), crossings.len() - unstacked - senior)
        .with("short", format!("{:.2}", short.iter().fold(0.0f64, |m, s| m.max(*s))));
    (Crossings { crossings, same, floor, profiles: Profiles { profiles } }, summary)
}

/// How far the re-solved profiles stand off the raw DEM, at grade.
pub fn check(crossings: &Crossings) -> Summary {
    Summary::new().with_residual(crate::step::residual_of(&crossings.profiles.profiles))
}

/// One axis in the crossing search: a carriageway piece, whatever its
/// class, with the level the source mapped it at and the profile that
/// solved it — `None` for a piece no profile covers, which is an indoor
/// way and, later, whatever else stops solving. A crossing on one of those
/// is an `orphan`: a demand with no solved feature on both sides, which
/// §4.5 requires to read zero.
/// One way, with the arc of each of its vertices and the profile it was
/// solved into. The crossings are found on the **ways** now, not on pieces:
/// the partition runs *after* this step, so there are no pieces yet, and the
/// span table carries the level ordinals just as well.
struct Axis<'a> {
    way: &'a Way,
    /// Arc at each vertex of `way.pts`.
    arc: Vec<f64>,
    profile: Option<usize>,
}

impl Axis<'_> {
    /// The arc of a point on segment `seg` of the way.
    fn arc_at(&self, seg: usize, at: Pt) -> f64 {
        let p = self.way.pts[seg];
        self.arc[seg] + (at[0] - p[0]).hypot(at[1] - p[1])
    }
}

/// One plan crossing: two ways, the segment of each it falls on, and where.
struct Hit {
    a: usize,
    a_seg: usize,
    b: usize,
    b_seg: usize,
    at: Pt,
}

/// One clearance demand: two solved pieces and what must separate them.
///
/// A side is `(profile, arc window)`: the profile is the whole way now, and a
/// way may pass near one point more than once — a hairpin, a loop, a road
/// that comes back along the shore. Reading its height at "the nearest
/// station" then answers about the wrong stretch, and the demand that follows
/// is nonsense. The window is the crossing *piece's* own arc range, which is
/// where the crossing actually is.
struct Pair {
    upper: (usize, i64),
    lower: (usize, i64),
    /// The arc window of the upper and lower pieces along their ways.
    up_at: (f64, f64),
    low_at: (f64, f64),
    need: f64,
    at: Pt,
    had: f64,
    demanded: bool,
    /// The ordinals ordered this pair and the geometry contradicted them by
    /// more than [`MAX_CLEARANCE_LIFT_M`]: not a stacked pair at all.
    unstacked: bool,
    /// Which side is a railway.
    up_rail: bool,
    low_rail: bool,
    /// A road over a railway's bore: no demand at all.
    senior: bool,
}

impl Pair {
    /// How far from the ground the further of the two is: which pass of
    /// the solve owns the pair.
    fn magnitude(&self) -> i64 {
        self.upper.1.abs().max(self.lower.1.abs())
    }

    /// How a deficit of `short` is divided: what the upper side lifts and
    /// what the lower side dips.
    ///
    /// Between a railway and a road the road takes all of it, whichever of
    /// the two the ordinals put on top. Between peers the heavier ordinal
    /// carries the whole demand, and a tie — a deck over a bore — splits
    /// it, each moving away from the other.
    fn shares(&self, short: f64) -> (f64, f64) {
        if self.up_rail != self.low_rail {
            return if self.up_rail { (0.0, short) } else { (short, 0.0) };
        }
        let (a, b) = (self.upper.1.abs(), self.lower.1.abs());
        match a.cmp(&b) {
            std::cmp::Ordering::Greater => (short, 0.0),
            std::cmp::Ordering::Less => (0.0, short),
            std::cmp::Ordering::Equal => (short / 2.0, short / 2.0),
        }
    }
}

/// Whether a way of `class` is a railway.
fn is_rail(class: &str) -> bool {
    width::family(class) == Family::Rail
}

/// What must separate the two roadways at a crossing, in metres.
///
/// Over a road at grade the gap is the headroom the lower one needs plus
/// the slab the upper one hangs into it — a road bridge's or, when the
/// walks come to solve, a footbridge's. Over a railway the headroom is the
/// catenary's, [`RAIL_CLEARANCE_M`]. Over a road in a *bore* it is the
/// bore's own section instead: the tube is already under the ground the
/// upper road rides on, so what governs is the tunnel's headroom and the
/// cover carrying the roadway above it (§4.5). The two read the same
/// number today, because a tunnel is as high inside as a road is over a
/// road; they are written apart because they are different quantities and
/// will not stay equal.
fn need(upper_class: &str, lower_class: &str, lower_kind: Kind) -> f64 {
    let slab = if width::family(upper_class) == Family::Walk { WALK_DECK_M } else { DECK_THICKNESS_M };
    match lower_kind {
        Kind::Tunnel(_) => crate::standard::tube_m(lower_class) + slab,
        _ if is_rail(lower_class) => RAIL_CLEARANCE_M + slab,
        _ => ROAD_CLEARANCE_M + slab,
    }
}

/// The separation of a pair's two roadways, in metres, as the profiles
/// stand.
fn separation(profiles: &[Profile], pair: &Pair) -> f64 {
    height_on(&profiles[pair.upper.0], pair.at, pair.up_at)
        - height_on(&profiles[pair.lower.0], pair.at, pair.low_at)
}

/// The solved height of the axis of `p` at the plan point `at`: the
/// station pair nearest it, interpolated. The point lies on the axis by
/// construction, so "nearest" is exact.
fn height_on(p: &Profile, at: Pt, window: (f64, f64)) -> f64 {
    let mut best = (f64::INFINITY, p.stations.first().map_or(0.0, |st| st.h));
    for w in p.stations.windows(2) {
        // Only inside the window: a way may run near this point twice.
        if w[1].s < window.0 - 1e-6 || w[0].s > window.1 + 1e-6 {
            continue;
        }
        let f = poly::nearest_on_segment(w[0].p, w[1].p, at);
        let d = (f[0] - at[0]).hypot(f[1] - at[1]);
        if d >= best.0 {
            continue;
        }
        let len = (w[1].p[0] - w[0].p[0]).hypot(w[1].p[1] - w[0].p[1]);
        let t = if len > 0.0 { (f[0] - w[0].p[0]).hypot(f[1] - w[0].p[1]) / len } else { 0.0 };
        best = (d, w[0].h + (w[1].h - w[0].h) * t);
    }
    best.1
}

/// One structure run: which profile, and which of its runs.
type Node = (usize, usize);

/// Every connector a way *end* that is on the ground reaches — the anchors
/// the profile step solved against, read back from the profiles so the two
/// cannot disagree.
fn anchor_connectors(profiles: &[Profile]) -> impl Iterator<Item = (i64, i64)> + '_ {
    profiles.iter().flat_map(|p| {
        let last = p.stations.len().saturating_sub(1);
        [(0usize, false), (last, true)]
            .into_iter()
            .filter(|&(_, high)| !p.end_kind(high).is_structure())
            .map(|(k, _)| connector(p.stations[k].p))
            .collect::<Vec<_>>()
    })
}

/// Adds the seeds one demand of `amount` puts on the piece `mover`: every
/// station of the chord it belongs to, so the chord rises level, or the
/// two stations either side of `at` where the mover is not a span at all.
/// Seeds the corridor a demand asks for.
///
/// **Two readings of the same demand, and the floor is the lesser.**
///
/// `seeds` is the displacement the demand asks for, decaying with network
/// distance: the rule the step has always had. It is right where the ground
/// falls away — the road ramps down at the class's grade and the decay is
/// that ramp — and wrong where the ground *climbs*, because it pushes the
/// road up on top of a climbing ground. South of the Clarens railway the
/// decay was exactly the street's 15 % while the reference under the same
/// nine metres fell 73 % (the DEM samples the cutting the road bridges):
/// the road came out at 58 %, three metres over its reference, and that is
/// what made every leg of the junction a derived deck.
///
/// `heights` is the same demand as an absolute corridor — a height the road
/// must reach — which stops applying the moment the ground is high enough
/// on its own. Alone it is wrong the other way: where the ground falls
/// faster than the corridor decays it holds a road up far from any
/// crossing, and the loop box read a 78 m lift.
///
/// So each node takes the **smaller** of the two, which is the honest
/// reading of both: lift by no more than the ramp allows, and by no more
/// than the clearance actually needs here. `sign` is +1 for a lift and −1
/// for a dip; a dip's corridor is negated so the same max-decay walk gives
/// the *lowest* ceiling reaching a node.
fn charge(
    seeds: &mut Vec<(usize, f64)>,
    heights: &mut Vec<(usize, f64)>,
    mover: usize,
    at: Pt,
    window: (f64, f64),
    amount: f64,
    sign: f64,
    profiles: &[Profile],
    joins: &HashMap<(i64, i64), Vec<Node>>,
    net: &Net,
) {
    let p = &profiles[mover];
    // The nearest station *inside the crossing piece's own window*, for the
    // same reason [`height_on`] takes one.
    let inside = |k: usize| {
        p.stations[k].s >= window.0 - 1e-6 && p.stations[k].s <= window.1 + 1e-6
    };
    let near = (0..p.stations.len())
        .filter(|&k| inside(k))
        .min_by(|&a, &b| dist2(p.stations[a].p, at).total_cmp(&dist2(p.stations[b].p, at)))
        .or_else(|| {
            (0..p.stations.len())
                .min_by(|&a, &b| dist2(p.stations[a].p, at).total_cmp(&dist2(p.stations[b].p, at)))
        })
        .unwrap_or(0);
    let runs = p.runs();
    // The run the crossing falls in — not the whole way. A way is one object
    // now, and charging all of it would lift a kilometre of street for a
    // twenty-metre deck.
    let at_run = runs.iter().position(|&(k0, k1, _)| near >= k0 && near <= k1);
    let Some(r) = at_run.filter(|&r| runs[r].2.is_structure()) else {
        for k in [near.saturating_sub(1), near, (near + 1).min(p.stations.len().saturating_sub(1))] {
            seeds.push((net.base[mover] + k, amount));
            heights.push((net.base[mover] + k, sign * (p.stations[k].h + sign * amount)));
        }
        return;
    };
    // Every structure run reachable through a connector no way is at grade
    // at: what the profile step solves as one chord. A chord cannot be bent
    // up over the road it crosses, so every station of the chain is charged.
    let mut seen: HashSet<Node> = [(mover, r)].into_iter().collect();
    let mut chord: Vec<Node> = vec![(mover, r)];
    let mut queue: Vec<Node> = vec![(mover, r)];
    while let Some((i, r)) = queue.pop() {
        let q = &profiles[i];
        let last = q.stations.len().saturating_sub(1);
        let (k0, k1, _) = q.runs()[r];
        for k in [k0, k1] {
            if k != 0 && k != last {
                continue;
            }
            for &n in joins.get(&connector(q.stations[k].p)).into_iter().flatten() {
                if seen.insert(n) {
                    chord.push(n);
                    queue.push(n);
                }
            }
        }
    }
    chord.sort_unstable();
    for (i, r) in chord {
        let (k0, k1, _) = profiles[i].runs()[r];
        // The run **and its abutments**: a chord's height is its two abutment
        // heights and nothing else, so lifting only its interior stations
        // moves nothing at all — the chord is re-interpolated between the
        // abutments the moment the profile re-solves.
        let last = profiles[i].stations.len().saturating_sub(1);
        for k in k0.saturating_sub(1)..=(k1 + 1).min(last) {
            seeds.push((net.base[i] + k, amount));
            heights.push((net.base[i] + k, sign * (profiles[i].stations[k].h + sign * amount)));
        }
    }
}

fn dist2(a: Pt, b: Pt) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

/// The network as a graph over the profiles' stations: consecutive
/// stations of a piece joined at the height the class's ramp grade buys
/// between them, and every station at one connector joined at no cost at
/// all. A demand travels it and decays, and that decay *is* the approach.
struct Net {
    /// The first node of each piece; `base[n]` is the node count.
    base: Vec<usize>,
    adj: Vec<Vec<(usize, f64)>>,
}

impl Net {
    fn new(profiles: &[Profile]) -> Net {
        let mut base = Vec::with_capacity(profiles.len() + 1);
        let mut n = 0;
        for p in profiles {
            base.push(n);
            n += p.stations.len();
        }
        base.push(n);
        let mut adj: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];
        let rail: Vec<bool> = profiles.iter().map(|p| is_rail(&p.class)).collect();
        for (i, p) in profiles.iter().enumerate() {
            // The grade the way was held to: a railway's measured one.
            let g = grade::of(&p.class);
            let ramp = if g.limited() {
                solve::ceiling(&p.class, &p.stations, &p.spans)
            } else {
                g.ceiling.unwrap_or(f64::INFINITY)
            };
            for k in 1..p.stations.len() {
                let w = ramp * (p.stations[k].s - p.stations[k - 1].s);
                adj[base[i] + k - 1].push((base[i] + k, w));
                adj[base[i] + k].push((base[i] + k - 1, w));
            }
        }
        // **Every** station at a shared connector, not only a way's two ends.
        // A way is whole now, so another way can end on its *interior* — at a
        // bridge abutment, most often, which is exactly where a lift has to
        // travel. Joined at the ends alone, the deck rose and the street that
        // meets it at the abutment stayed on the ground: `structure abutment`
        // 5.380 m of joint that cannot meet.
        //
        // **Within one stratum only.** A railway and a road sharing a
        // connector — a level crossing — are not joined here, so a floor
        // spreading along either stops at the other: a junior never
        // charges a senior, and a railway's own lift is the railway's.
        let mut at: HashMap<(i64, i64), Vec<(usize, bool)>> = HashMap::new();
        for (i, p) in profiles.iter().enumerate() {
            for k in 0..p.stations.len() {
                at.entry(connector(p.stations[k].p)).or_default().push((base[i] + k, rail[i]));
            }
        }
        for group in at.values() {
            for &(a, ra) in group {
                for &(b, rb) in group {
                    if a != b && ra == rb {
                        adj[a].push((b, 0.0));
                    }
                }
            }
        }
        Net { base, adj }
    }

    /// The floor `seeds` make: at every node, the largest demand that
    /// still has something left of it when it arrives, and zero where none
    /// does. A max-Dijkstra, so a node is settled once and the answer does
    /// not depend on the order the seeds were found in.
    fn spread(&self, seeds: &[(usize, f64)], init: f64) -> Vec<f64> {
        let mut val = vec![init; self.adj.len()];
        let mut heap: BinaryHeap<Reach> = BinaryHeap::new();
        for &(node, v) in seeds {
            if v > val[node] {
                val[node] = v;
                heap.push(Reach { v, node });
            }
        }
        while let Some(Reach { v, node }) = heap.pop() {
            if v < val[node] {
                continue;
            }
            for &(next, w) in &self.adj[node] {
                let u = v - w;
                if u > val[next] {
                    val[next] = u;
                    heap.push(Reach { v: u, node: next });
                }
            }
        }
        val
    }
}

/// A node with what is left of a demand when it reaches it, ordered so the
/// largest pops first and equal ones pop in node order: the queue is part
/// of the answer's determinism.
#[derive(PartialEq)]
struct Reach {
    v: f64,
    node: usize,
}

impl Eq for Reach {}

impl Ord for Reach {
    fn cmp(&self, other: &Reach) -> std::cmp::Ordering {
        self.v.total_cmp(&other.v).then(other.node.cmp(&self.node))
    }
}

impl PartialOrd for Reach {
    fn partial_cmp(&self, other: &Reach) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// Every place two axes' interiors cross, as `(i, j, point)` with `i < j`,
/// in an order that is a function of the axes alone.
fn found(axes: &[Axis]) -> Vec<Hit> {
    let mut cells: HashMap<(i32, i32), Vec<(usize, usize)>> = HashMap::new();
    for (i, a) in axes.iter().enumerate() {
        for k in 1..a.way.pts.len() {
            let (p, q) = (a.way.pts[k - 1], a.way.pts[k]);
            let box_ = [p[0].min(q[0]), p[1].min(q[1]), p[0].max(q[0]), p[1].max(q[1])];
            for cell in poly::cells_over(box_, poly::CELL_M) {
                cells.entry(cell).or_default().push((i, k - 1));
            }
        }
    }
    let mut seen: HashSet<(usize, usize, usize, usize)> = HashSet::new();
    let mut out: Vec<Hit> = Vec::new();
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
                let (u, v) = (&axes[a.0].way.pts, &axes[b.0].way.pts);
                if let Some(p) = poly::proper_crossing(u[a.1], u[a.1 + 1], v[b.1], v[b.1 + 1]) {
                    out.push(Hit { a: a.0, a_seg: a.1, b: b.0, b_seg: b.1, at: p });
                }
            }
        }
    }
    out.sort_by(|x, y| {
        (x.a, x.b).cmp(&(y.a, y.b)).then(x.at[0].total_cmp(&y.at[0])).then(x.at[1].total_cmp(&y.at[1]))
    });
    out
}

#[cfg(test)]
mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{bbox, built, upto};
    use crate::step::Step;
    

    
    
    use crate::terrain::{self, tests::{dem, extent}};
    use crate::world::Solved;

    use super::*;

    /// A world on the ground of `terrain_spec` with the network of `net`,
    /// profiled and crossed.
    fn world(terrain_spec: &str, net: &str) -> (World, Summary) {
        let (w, ran) = built(terrain_spec, net, None, 5.0, &upto(Step::Crossing));
        (w, ran.last())
    }

    /// The stations of `id`'s run of kind `mapped`. A way is one profile
    /// now, so a test that used to hold a piece holds a range of one.
    fn run_of(w: &World, id: &str, mapped: Kind) -> Vec<crate::world::Station> {
        let p = profile_of(w, id, mapped);
        let last = p.stations.len().saturating_sub(1);
        let (k0, k1, _) = p
            .runs()
            .into_iter()
            .find(|r| r.2 == mapped)
            .unwrap_or_else(|| panic!("no {mapped:?} run on {id}"));
        // The run itself, abutments excluded: an abutment belongs to the
        // at-grade solve, and carries its flag — the structure step widens to
        // reach it when it builds the solid, but `solved` is the annotation's
        // own stations.
        let _ = last;
        p.stations[k0..=k1].to_vec()
    }

    fn profile_of<'a>(w: &'a World, id: &str, mapped: Kind) -> &'a Profile {
        w.solved()
            .unwrap()
            .profiles
            .iter()
            .find(|p| p.id == id && p.spans.iter().any(|s| s.kind == mapped))
            .unwrap_or_else(|| panic!("no {id} {mapped:?}"))
    }

    #[test]
    fn a_junction_is_not_a_crossing() {
        // Four legs meeting at the origin share a connector: way ends
        // meet, interiors do not cross, and nothing is coupled.
        let (_, s) = world("flat", "net:cross?len=200");
        assert_eq!(s.num("crossings"), 0.0, "{s}");
        assert_eq!(s.num("same"), 0.0, "{s}");
        assert_eq!(s.num("ramped"), 0.0, "{s}");
        // Nor is a way crossing its own bridge span's boundary.
        let (_, s) = world("flat", "net:straight?len=200&span=0.35,0.65");
        assert_eq!(s.num("crossings"), 0.0, "{s}");
    }

    #[test]
    fn an_overpass_lifts_the_leg_and_ramps_its_approaches() {
        // A residential over a residential: 5 m of headroom and a 1.5 m
        // slab is 6.5 m, and a street's ramp grade is 15 %, so the
        // approach is 6.5 / 0.15 = 43.33 m long from each abutment.
        let (w, s) = world("flat", "net:overpass?len=300");
        assert_eq!(s.num("crossings"), 1.0, "{s}");
        assert_eq!(s.num("same"), 0.0, "{s}");
        assert_eq!(s.num("orphan"), 0.0, "{s}");
        assert_eq!(s.num("demands"), 1.0, "{s}");
        assert_eq!(s.num("clearance"), 0.0, "{s}");
        assert!((s.num("lift") - 6.5).abs() < 0.01, "{s}");

        // The deck stands level at the full clearance, and every one of
        // its stations reads as a deck: the structure step needs no rule.
        let deck = run_of(&w, "leg", Kind::Bridge(1));
        for st in &deck {
            assert!((st.h - 406.5).abs() < 1e-6, "{st:?}");
            assert_eq!(st.solved, Solved::Deck, "{st:?}");
        }
        // The road underneath is untouched: nothing connects the two.
        let road = profile_of(&w, "road", Kind::Ground);
        assert!(road.stations.iter().all(|st| st.h == 400.0), "{:?}", road.stations[0]);

        // The approach ramps at the street's ceiling and meets the ground
        // 43.33 m past the abutment, which a 300 m leg spanned over its
        // middle 30 % puts at y = 45. The toe itself falls between two
        // stations — the floor is sampled every NODE_M and clamped at
        // zero, so its last metre is rounded over one node — and the check
        // is taken clear of it.
        // The whole leg: its approaches ramp, its deck is level, and neither
        // may exceed the class's ramp grade.
        let up = profile_of(&w, "leg", Kind::Ground);
        let ceiling = grade::of("residential").ceiling.unwrap();
        for pair in up.stations.windows(2) {
            let ds = pair[1].s - pair[0].s;
            let rise = (pair[1].h - pair[0].h).abs() / ds;
            assert!(rise <= ceiling + 1e-9, "{pair:?}");
        }
        let at = |y: f64| height_on(up, [0.0, y], (0.0, f64::INFINITY));
        // The toe is a *curve* now, not a corner: the class's vertical radius
        // spreads the last of the ramp over about `ceiling × radius / 2`,
        // which for a street's 15 % and 100 m is 7.5 m. So the profile meets
        // the ground later than the straight ramp's 43.3 m would put it, and
        // the check is taken clear of the curve rather than inside it.
        for (y, h) in [(45.0, 406.5), (65.0, 403.5), (105.0, 400.0), (130.0, 400.0)] {
            let y = if up.stations[0].p[1] < 0.0 { -y } else { y };
            assert!((at(y) - h).abs() < 0.02, "at {y}: {} vs {h}", at(y));
        }
    }

    #[test]
    fn an_underpass_dips_the_leg() {
        // The mirror: the road stays on the ground and the bore goes under
        // it by the tunnel's own height plus the cover that carries the
        // road — the same 6.5 m, read the other way round.
        let (w, s) = world("flat", "net:underpass?len=300");
        assert_eq!(s.num("crossings"), 1.0, "{s}");
        assert_eq!(s.num("demands"), 1.0, "{s}");
        assert_eq!(s.num("clearance"), 0.0, "{s}");
        let bore = run_of(&w, "leg", Kind::Tunnel(-1));
        for st in &bore {
            assert!((st.h - 393.5).abs() < 1e-6, "{st:?}");
            assert_eq!(st.solved, Solved::Bore, "{st:?}");
        }
        let road = profile_of(&w, "road", Kind::Ground);
        assert!(road.stations.iter().all(|st| st.h == 400.0), "{:?}", road.stations[0]);
        let down = profile_of(&w, "leg", Kind::Ground);
        assert!(down.stations.iter().all(|st| st.h <= 400.0 + 1e-9), "{:?}", down.stations[0]);
        let at = |y: f64| height_on(down, [0.0, y], (0.0, f64::INFINITY));
        let sign = if down.stations[0].p[1] < 0.0 { -1.0 } else { 1.0 };
        assert!((at(sign * 45.0) - 393.5).abs() < 0.02, "{}", at(sign * 45.0));
        // Clear of the toe's own vertical curve, as above.
        assert!((at(sign * 105.0) - 400.0).abs() < 0.02, "{}", at(sign * 105.0));
    }

    #[test]
    fn a_span_that_already_clears_asks_for_nothing() {
        // The leg's bridge over a 40 m valley the road runs along the
        // bottom of: the chord is already high, so no floor is spread and
        // the profile the crossing step returns is the one it was given.
        //
        // The valley is 120 m across, which is wider than the closing's own
        // window: the road descends into it rather than spanning it, and no
        // notch is refused. At 60 m the closing refuses the whole slot, the
        // terrain promotes a deck over it, and the road bridges the valley
        // too — which is right, and a different specimen.
        let (w, s) = world("hill?amp=-40&radius=60", "net:overpass?len=300&span=0.2,0.8");
        assert_eq!(s.num("crossings"), 1.0, "{s}");
        assert_eq!(s.num("demands"), 0.0, "{s}");
        assert_eq!(s.num("clearance"), 0.0, "{s}");
        assert_eq!(s.num("lift"), 0.0, "{s}");
        assert_eq!(s.num("ramped"), 0.0, "{s}");
        let c = &w.crossing.as_ref().unwrap().crossings[0];
        assert_eq!(c.had, c.have, "nothing moved");
        assert!(c.have > c.need, "{c:?}");
    }

    #[test]
    fn two_interiors_crossing_at_one_level_are_counted_not_solved() {
        // Overture cuts a way at every connector, so this cannot come out
        // of the reader: it is a data error, and the step says so rather
        // than inventing a separation the source did not order.
        let (ground, _) = terrain::run(&extent(), &mut dem("flat"), 5.0, usize::MAX);
        let mut w = World::new(bbox());
        w.terrain = Some(ground);

        // Two whole ways, cut by the partition step as the reader's would
        // be: a piece has to know which way it came from for the crossing
        // step to find the profile it was solved into.
        let way = |id: &str, pts: Vec<[f64; 2]>| crate::world::Way {
            id: id.into(),
            class: "residential".into(),
            subclass: String::new(),
            width_m: 5.5,
            spans: vec![crate::world::Span { a0: 0.0, a1: crate::roads::length(&pts), kind: Kind::Ground }],
            layers: Vec::new(),
            pts,
        };
        w.roads = Some(crate::world::Roads {
            ways: vec![
                way("a", vec![[-50.0, 0.0], [50.0, 0.0]]),
                way("b", vec![[0.0, -50.0], [0.0, 50.0]]),
            ],
            ..Default::default()
        });
        let roads = w.roads.as_ref().expect("just set");
        let terrain = w.terrain.as_ref().expect("just set");
        let (reference, _) = crate::reference::run(terrain, roads);
        let (profiles, _) = crate::profile::run(&reference);
        let (crossings, s) = run(&reference, &profiles);
        assert_eq!(s.num("crossings"), 0.0, "{s}");
        assert_eq!(s.num("same"), 1.0, "{s}");
        assert_eq!(s.num("ramped"), 0.0, "{s}");
        assert_eq!(crossings.same, vec![[0.0, 0.0]]);
    }

    /// **Where the tags and the geometry disagree by more than a crossing
    /// accounts for, the geometry wins** (docs/GENERATION.md §4.5).
    ///
    /// A trench 30 m deep running east–west, a road along the bottom of it,
    /// and a way crossing it that the source tagged as a *tunnel*. The tunnel
    /// is nothing of the sort — its chord runs between the two rims, thirty
    /// metres over the road it is supposed to pass beneath — so the ordinals
    /// say the road at the bottom is above it and the heights say it is
    /// thirty metres below. Honouring that would move the world 36.5 m to
    /// satisfy a tag about something else, so the demand is dropped whole and
    /// counted: spending [`MAX_CLEARANCE_LIFT_M`] of it would leave the
    /// geometry both wrong *and* distorted.
    #[test]
    fn a_demand_the_geometry_contradicts_is_dropped_not_spent() {
        let (w, s) = world("gorge?depth=30&width=40&bearing=0", "net:underpass?len=200");
        assert_eq!(s.num("crossings"), 1.0, "{s}");
        assert_eq!(s.num("unstacked"), 1.0, "{s}");
        assert_eq!(s.num("demands"), 0.0, "nothing was charged: {s}");
        assert_eq!(s.num("lift"), 0.0, "the world moved: {s}");
        // A dropped pair is not a clearance the model failed to meet — it is
        // one it declined to believe — so it leaves `clearance` alone.
        assert_eq!(s.num("clearance"), 0.0, "{s}");
        assert!(w.crossing.is_some());
    }

    #[test]
    fn a_proper_cross_is_the_only_cross() {
        assert_eq!(crate::poly::proper_crossing([-1.0, 0.0], [1.0, 0.0], [0.0, -1.0], [0.0, 1.0]), Some([0.0, 0.0]));
        // A touch at an endpoint, a T, a shared vertex and two parallels
        // are all not crossings.
        assert_eq!(crate::poly::proper_crossing([-1.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 1.0]), None);
        assert_eq!(crate::poly::proper_crossing([0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 1.0]), None);
        assert_eq!(crate::poly::proper_crossing([0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]), None);
        assert_eq!(crate::poly::proper_crossing([0.0, 0.0], [1.0, 0.0], [2.0, -1.0], [2.0, 1.0]), None);
    }

    #[test]
    fn the_lift_ramps_the_ways_it_meets_and_not_the_ones_it_does_not() {
        // On a hill the overpass still clears, and the whole leg is one
        // network: the ramp reaches the approaches through their shared
        // connectors and stops where the class grade spends it.
        let (w, s) = world("ramp?grade=0.1&bearing=0&radius=100000", "net:overpass?len=300");
        assert_eq!(s.num("clearance"), 0.0, "{s}");
        let c = &w.crossing.as_ref().unwrap().crossings[0];
        assert!(c.have >= c.need - 1e-6, "{c:?}");
        // The floor is zero far from the crossing: 6.5 m at 15 % is spent
        // in 43.33 m, and the leg is 150 m long each way.
        let floor = &w.crossing.as_ref().unwrap().floor;
        assert!(floor.iter().flatten().any(|v| *v > 6.0), "nothing lifted");
        assert!(floor.iter().flatten().any(|v| *v == 0.0), "everything lifted");
    }

    /// A road bridge over a railway clears the catenary, and the railway
    /// underneath it does not move at all.
    #[test]
    fn a_road_lifts_over_a_railway_by_the_catenary_s_headroom() {
        let (w, s) = world("flat", "net:overpass?len=300&class=standard_gauge&leg=residential");
        assert_eq!(s.num("demands"), 1.0, "{s}");
        assert_eq!(s.num("clearance"), 0.0, "{s}");
        assert!((s.num("lift") - (RAIL_CLEARANCE_M + DECK_THICKNESS_M)).abs() < 0.01, "{s}");
        let rail = profile_of(&w, "road", Kind::Ground);
        assert_eq!(rail.class, "standard_gauge");
        assert!(rail.stations.iter().all(|st| st.h == 400.0), "{:?}", rail.stations[0]);
    }

    /// **A railway is never the one that moves.** A rail bridge over a road
    /// on flat ground: the ordinals say the bridge lifts, the strata say the
    /// railway is senior, and the road dips under it instead. And a road
    /// over a railway's bore is not asked to clear it at all.
    #[test]
    fn a_railway_is_never_the_one_that_moves() {
        let (w, s) = world("flat", "net:overpass?len=300&leg=standard_gauge");
        assert_eq!(s.num("demands"), 1.0, "{s}");
        let rail = profile_of(&w, "leg", Kind::Bridge(1));
        assert!(rail.stations.iter().all(|st| st.h == 400.0), "the railway moved: {:?}", rail.stations[0]);
        let road = profile_of(&w, "road", Kind::Ground);
        let low = road.stations.iter().map(|st| st.h).fold(f64::INFINITY, f64::min);
        assert!(low < 400.0 - ROAD_CLEARANCE_M, "the road did not dip: {low} {s}");
        // The floor went down the road and nowhere else.
        let floor = &w.crossing.as_ref().unwrap().floor;
        let (ri, _) = w.solved().unwrap().profiles.iter().enumerate().find(|(_, p)| p.id == "leg").unwrap();
        assert!(floor[ri].iter().all(|v| *v == 0.0), "a floor on the railway");

        let (w, s) = world("flat", "net:underpass?len=300&leg=standard_gauge");
        assert_eq!(s.num("senior"), 1.0, "{s}");
        assert_eq!(s.num("demands"), 0.0, "{s}");
        assert_eq!(s.get("clearance"), Some("0/0 (0.00%)"), "{s}");
        let road = profile_of(&w, "road", Kind::Ground);
        assert!(road.stations.iter().all(|st| st.h == 400.0), "{:?}", road.stations[0]);
    }

    #[test]
    fn the_crossings_are_a_function_of_the_world() {
        let (a, sa) = world("hill?amp=20&radius=200", "net:overpass?len=300");
        let (b, sb) = world("hill?amp=20&radius=200", "net:overpass?len=300");
        assert_eq!(sa.to_string(), sb.to_string());
        assert_eq!(a.crossing.as_ref().unwrap().floor, b.crossing.as_ref().unwrap().floor);
        let (x, y) = (a.profile.as_ref().unwrap(), b.profile.as_ref().unwrap());
        assert_eq!(x.profiles, y.profiles);
    }
}
