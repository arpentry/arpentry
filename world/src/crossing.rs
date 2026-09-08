//! Step 13: the crossings — the one thing that couples two ways.
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
//! ([`crate::profile::solve_on`]), and everything after it follows with no
//! rule of its own: an approach that ends up [`crate::grade::STRUCTURE_MIN_M`]
//! off the ground reads as a deck by step 9's consequence rule, the bench
//! builds the embankment under the rest of it, and the structure step lays
//! the slab.
//!
//! **A chord is charged whole.** The piece that lifts is always a mapped
//! span (a level ordinal is what makes it the mover), and step 9 solves a
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
//! another unknown. Rail, when it arrives, is senior and enters here as a
//! constant on both sides; a bore that belongs to it may not yield at all,
//! and §4.5 drops the demand rather than lifting a road over a railway that
//! is underground.

use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::grade;
use crate::poly::{self, Pt};
use crate::profile;
use crate::step::Summary;
use crate::structure::{DECK_THICKNESS_M, TUNNEL_HEIGHT_M, WALK_DECK_M};
use crate::width::{self, Family};
use crate::world::{connector, Crossing, Crossings, Kind, Polyline2, Profile, World};

/// The headroom a roadway needs over the roadway beneath it, in metres:
/// the Swiss norm's 4.5 m plus a construction margin, and the server's
/// number (`data/plans/surface-leaves-the-plane-2026-09-08.md` §5).
pub const ROAD_CLEARANCE_M: f64 = 5.0;

/// Slack, in metres, before a clearance counts as short: the solve's own
/// rounding along a chord and a ramp, not a budget.
const CLEARANCE_EPS: f64 = 1e-6;

/// Builds the floor every crossing demands and re-solves the profile on it.
pub fn run(world: &mut World) -> Summary {
    let terrain = world.terrain.as_ref().expect("the terrain step runs first");
    let roads = world.roads.as_ref().expect("the drape step runs first");
    let solved = world.profile.as_ref().expect("the profile step runs first");

    // The axes, in the order the profile step took them, so a solved
    // piece's index into `profiles` is known without matching anything.
    let mut next = 0usize;
    let axes: Vec<Axis> = roads
        .pieces()
        .filter(|w| width::family(&w.class) == Family::Carriageway)
        .map(|w| {
            let solves = w.kind != Kind::Indoor && grade::of(&w.class).solves();
            let profile = solves.then(|| {
                next += 1;
                next - 1
            });
            Axis { piece: w, level: level_of(w.kind), profile }
        })
        .collect();
    let pieces: Vec<&Polyline2> = axes.iter().filter(|a| a.profile.is_some()).map(|a| a.piece).collect();
    assert_eq!(next, solved.profiles.len(), "the profile step took another set of pieces");

    let mut profiles = solved.profiles.clone();
    let net = Net::new(&profiles);
    let mut floor: Vec<Vec<f64>> = profiles.iter().map(|p| vec![0.0; p.stations.len()]).collect();

    // Sort the crossings into the three answers: a data error, a demand we
    // cannot solve because a side has no profile, and a demand.
    let (mut same, mut orphan, mut pairs) = (Vec::new(), 0usize, Vec::new());
    for (i, j, at) in found(&axes) {
        let (a, b) = (&axes[i], &axes[j]);
        // An axis with no profile cannot take part in a demand at all, at
        // whatever level it was drawn: that is the orphan §4.5 requires to
        // read zero, and it is asked before the level, so an indoor way —
        // which is not stacked against the ground and reads level 0 — is
        // not filed as a data error in the road network.
        if a.profile.is_none() || b.profile.is_none() {
            orphan += 1;
        } else if a.level == b.level {
            same.push(at);
        } else {
            let (up, low) = if a.level > b.level { (a, b) } else { (b, a) };
            pairs.push(Pair {
                upper: (up.profile.expect("solved"), up.level),
                lower: (low.profile.expect("solved"), low.level),
                need: need(&up.piece.class, low.piece.kind),
                at,
                had: 0.0,
                demanded: false,
            });
        }
    }
    for pair in &mut pairs {
        pair.had = separation(&profiles, pair);
    }

    // The anchored connectors and the structure pieces at each free one:
    // what a chord runs between, and what it runs across.
    let anchored: HashSet<(i64, i64)> = ends_of(&profiles, |p| p.mapped == Kind::Ground).collect();
    let mut joins: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, p) in profiles.iter().enumerate() {
        if !p.mapped.is_structure() {
            continue;
        }
        for k in ends(p) {
            let c = connector(p.stations[k].p);
            if !anchored.contains(&c) {
                joins.entry(c).or_default().push(i);
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
        for pair in pairs.iter_mut().filter(|p| p.magnitude() == m) {
            let short = pair.need - separation(&profiles, pair);
            if short <= CLEARANCE_EPS {
                continue;
            }
            pair.demanded = true;
            let (a, b) = (pair.upper.1.abs(), pair.lower.1.abs());
            // The heavier ordinal carries the whole demand; a tie — a deck
            // over a bore — splits it, each moving away from the other.
            if a >= b {
                charge(&mut up, pair.upper.0, pair.at, if a > b { short } else { short / 2.0 }, &profiles, &joins, &net);
            }
            if b >= a {
                charge(&mut down, pair.lower.0, pair.at, if b > a { short } else { short / 2.0 }, &profiles, &joins, &net);
            }
        }
        if up.is_empty() && down.is_empty() {
            continue;
        }
        let (up, down) = (net.spread(&up), net.spread(&down));
        for (i, f) in floor.iter_mut().enumerate() {
            for (k, v) in f.iter_mut().enumerate() {
                *v += up[net.base[i] + k] - down[net.base[i] + k];
            }
        }
        let (re, _) = profile::solve_on(terrain, &pieces, &floor);
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
    let short: Vec<f64> = crossings.iter().map(|c| c.shortfall()).filter(|s| *s > CLEARANCE_EPS).collect();
    let lift = floor.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs()));
    let stations: usize = floor.iter().map(|f| f.len()).sum();
    let summary = Summary::new()
        .with("crossings", crossings.len())
        .with("same", same.len())
        .with("orphan", orphan)
        .with("demands", demands)
        .with("lift", format!("{lift:.2}"))
        .with_share("ramped", spent, stations)
        .with_share("clearance", short.len(), crossings.len())
        .with("short", format!("{:.2}", short.iter().fold(0.0f64, |m, s| m.max(*s))));
    world.crossing = Some(Crossings { crossings, same, floor });
    world.profile = Some(crate::world::Profiles { profiles });
    summary
}

/// One axis in the crossing search: a carriageway piece, whatever its
/// class, with the level the source mapped it at and the profile that
/// solved it — `None` for a piece no profile covers, which is an indoor
/// way and, later, whatever else stops solving. A crossing on one of those
/// is an `orphan`: a demand with no solved feature on both sides, which
/// §4.5 requires to read zero.
struct Axis<'a> {
    piece: &'a Polyline2,
    level: i64,
    profile: Option<usize>,
}

/// The level ordinal of a piece: an ordering against the ground, never a
/// height. Indoor is not stacked against the ground at all and reads 0.
fn level_of(kind: Kind) -> i64 {
    match kind {
        Kind::Bridge(n) | Kind::Tunnel(n) => n,
        Kind::Ground | Kind::Indoor => 0,
    }
}

/// One clearance demand: two solved pieces and what must separate them.
struct Pair {
    upper: (usize, i64),
    lower: (usize, i64),
    need: f64,
    at: Pt,
    had: f64,
    demanded: bool,
}

impl Pair {
    /// How far from the ground the further of the two is: which pass of
    /// the solve owns the pair.
    fn magnitude(&self) -> i64 {
        self.upper.1.abs().max(self.lower.1.abs())
    }
}

/// What must separate the two roadways at a crossing, in metres.
///
/// Over a road at grade the gap is the headroom the lower one needs plus
/// the slab the upper one hangs into it — a road bridge's or, when the
/// walks come to solve, a footbridge's. Over a road in a *bore* it is the
/// bore's own section instead: the tube is already under the ground the
/// upper road rides on, so what governs is the tunnel's headroom and the
/// cover carrying the roadway above it (§4.5). The two read the same
/// number today, because a tunnel is as high inside as a road is over a
/// road; they are written apart because they are different quantities and
/// will not stay equal.
fn need(upper_class: &str, lower_kind: Kind) -> f64 {
    let slab = if width::family(upper_class) == Family::Walk { WALK_DECK_M } else { DECK_THICKNESS_M };
    match lower_kind {
        Kind::Tunnel(_) => TUNNEL_HEIGHT_M + slab,
        _ => ROAD_CLEARANCE_M + slab,
    }
}

/// The separation of a pair's two roadways, in metres, as the profiles
/// stand.
fn separation(profiles: &[Profile], pair: &Pair) -> f64 {
    height_on(&profiles[pair.upper.0], pair.at) - height_on(&profiles[pair.lower.0], pair.at)
}

/// The solved height of the axis of `p` at the plan point `at`: the
/// station pair nearest it, interpolated. The point lies on the axis by
/// construction, so "nearest" is exact.
fn height_on(p: &Profile, at: Pt) -> f64 {
    let mut best = (f64::INFINITY, p.stations.first().map_or(0.0, |st| st.h));
    for w in p.stations.windows(2) {
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

/// The first and last station of a piece, once each.
fn ends(p: &Profile) -> impl Iterator<Item = usize> {
    let last = p.stations.len().saturating_sub(1);
    [0, last].into_iter().take(if p.stations.len() > 1 { 2 } else { p.stations.len() })
}

/// Every connector the profiles matching `keep` end at.
fn ends_of<'a>(
    profiles: &'a [Profile],
    keep: impl Fn(&Profile) -> bool + 'a,
) -> impl Iterator<Item = (i64, i64)> + 'a {
    profiles.iter().filter(move |p| keep(p)).flat_map(|p| ends(p).map(|k| connector(p.stations[k].p)))
}

/// Adds the seeds one demand of `amount` puts on the piece `mover`: every
/// station of the chord it belongs to, so the chord rises level, or the
/// two stations either side of `at` where the mover is not a span at all.
fn charge(
    seeds: &mut Vec<(usize, f64)>,
    mover: usize,
    at: Pt,
    amount: f64,
    profiles: &[Profile],
    joins: &HashMap<(i64, i64), Vec<usize>>,
    net: &Net,
) {
    let mut chord = vec![mover];
    if profiles[mover].mapped.is_structure() {
        // Every span reachable through a connector no ground piece ends
        // at: what step 9 solves as one chord.
        let mut seen: HashSet<usize> = [mover].into_iter().collect();
        let mut queue = vec![mover];
        while let Some(i) = queue.pop() {
            for k in ends(&profiles[i]) {
                for &j in joins.get(&connector(profiles[i].stations[k].p)).into_iter().flatten() {
                    if seen.insert(j) {
                        chord.push(j);
                        queue.push(j);
                    }
                }
            }
        }
        chord.sort_unstable();
        for i in chord {
            for k in 0..profiles[i].stations.len() {
                seeds.push((net.base[i] + k, amount));
            }
        }
    } else {
        let p = &profiles[mover];
        let near = (0..p.stations.len())
            .min_by(|&a, &b| dist2(p.stations[a].p, at).total_cmp(&dist2(p.stations[b].p, at)))
            .unwrap_or(0);
        for k in [near.saturating_sub(1), near, (near + 1).min(p.stations.len().saturating_sub(1))] {
            seeds.push((net.base[mover] + k, amount));
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
        for (i, p) in profiles.iter().enumerate() {
            let ramp = grade::of(&p.class).ceiling.unwrap_or(f64::INFINITY);
            for k in 1..p.stations.len() {
                let w = ramp * (p.stations[k].s - p.stations[k - 1].s);
                adj[base[i] + k - 1].push((base[i] + k, w));
                adj[base[i] + k].push((base[i] + k - 1, w));
            }
        }
        let mut at: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        for (i, p) in profiles.iter().enumerate() {
            for k in ends(p) {
                at.entry(connector(p.stations[k].p)).or_default().push(base[i] + k);
            }
        }
        for group in at.values() {
            for &a in group {
                for &b in group {
                    if a != b {
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
    fn spread(&self, seeds: &[(usize, f64)]) -> Vec<f64> {
        let mut val = vec![0.0f64; self.adj.len()];
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
fn found(axes: &[Axis]) -> Vec<(usize, usize, Pt)> {
    let mut cells: HashMap<(i32, i32), Vec<(usize, usize)>> = HashMap::new();
    for (i, a) in axes.iter().enumerate() {
        for k in 1..a.piece.pts.len() {
            let (p, q) = (a.piece.pts[k - 1], a.piece.pts[k]);
            let box_ = [p[0].min(q[0]), p[1].min(q[1]), p[0].max(q[0]), p[1].max(q[1])];
            for cell in poly::cells_over(box_, poly::CELL_M) {
                cells.entry(cell).or_default().push((i, k - 1));
            }
        }
    }
    let mut seen: HashSet<(usize, usize, usize, usize)> = HashSet::new();
    let mut out: Vec<(usize, usize, Pt)> = Vec::new();
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
                let (u, v) = (&axes[a.0].piece.pts, &axes[b.0].piece.pts);
                if let Some(p) = cross(u[a.1], u[a.1 + 1], v[b.1], v[b.1 + 1]) {
                    out.push((a.0, b.0, p));
                }
            }
        }
    }
    out.sort_by(|a, b| {
        (a.0, a.1).cmp(&(b.0, b.1)).then(a.2[0].total_cmp(&b.2[0])).then(a.2[1].total_cmp(&b.2[1]))
    });
    out
}

/// Where the segments `ab` and `cd` properly cross — each strictly
/// separating the other's ends — or `None`. A touch at an endpoint is not
/// a crossing: that is how a junction is drawn, and how two pieces of one
/// way meet.
fn cross(a: Pt, b: Pt, c: Pt, d: Pt) -> Option<Pt> {
    let side = |p: Pt, q: Pt, r: Pt| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    let (s1, s2) = (side(a, b, c), side(a, b, d));
    let (s3, s4) = (side(c, d, a), side(c, d, b));
    if !(s1 * s2 < 0.0 && s3 * s4 < 0.0) {
        return None;
    }
    let t = s3 / (s3 - s4);
    Some([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t])
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use crate::drape;
    use crate::terrain::{self, tests::dem};
    use crate::world::Solved;

    use super::*;

    /// A world on the ground of `terrain_spec` with the network of `net`,
    /// profiled and crossed.
    fn world(terrain_spec: &str, net: &str) -> (World, Summary) {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem(terrain_spec), 5.0, usize::MAX);
        drape::run(&mut w, Path::new(net)).unwrap();
        profile::run(&mut w);
        let s = run(&mut w);
        (w, s)
    }

    fn profile_of<'a>(w: &'a World, id: &str, mapped: Kind) -> &'a Profile {
        w.profile
            .as_ref()
            .unwrap()
            .profiles
            .iter()
            .find(|p| p.id == id && p.mapped == mapped)
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
        let deck = profile_of(&w, "leg", Kind::Bridge(1));
        for st in &deck.stations {
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
        let up = profile_of(&w, "leg", Kind::Ground);
        let ceiling = grade::of("residential").ceiling.unwrap();
        for pair in up.stations.windows(2) {
            let ds = pair[1].s - pair[0].s;
            let rise = (pair[1].h - pair[0].h).abs() / ds;
            assert!(rise <= ceiling + 1e-9, "{pair:?}");
        }
        let at = |y: f64| height_on(up, [0.0, y]);
        for (y, h) in [(45.0, 406.5), (65.0, 403.5), (92.0, 400.0), (120.0, 400.0)] {
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
        let bore = profile_of(&w, "leg", Kind::Tunnel(-1));
        for st in &bore.stations {
            assert!((st.h - 393.5).abs() < 1e-6, "{st:?}");
            assert_eq!(st.solved, Solved::Bore, "{st:?}");
        }
        let road = profile_of(&w, "road", Kind::Ground);
        assert!(road.stations.iter().all(|st| st.h == 400.0), "{:?}", road.stations[0]);
        let down = profile_of(&w, "leg", Kind::Ground);
        assert!(down.stations.iter().all(|st| st.h <= 400.0 + 1e-9), "{:?}", down.stations[0]);
        let at = |y: f64| height_on(down, [0.0, y]);
        let sign = if down.stations[0].p[1] < 0.0 { -1.0 } else { 1.0 };
        assert!((at(sign * 45.0) - 393.5).abs() < 0.02, "{}", at(sign * 45.0));
        assert!((at(sign * 92.0) - 400.0).abs() < 0.02, "{}", at(sign * 92.0));
    }

    #[test]
    fn a_span_that_already_clears_asks_for_nothing() {
        // The leg's bridge over a 40 m valley the road runs along the
        // bottom of: the chord is already high, so no floor is spread and
        // the profile the crossing step returns is the one it was given.
        let (w, s) = world("hill?amp=-40&radius=30", "net:overpass?len=300&span=0.2,0.8");
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
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem("flat"), 5.0, usize::MAX);
        let line = |id: &str, pts: Vec<[f64; 2]>| Polyline2 {
            id: id.into(),
            class: "residential".into(),
            subclass: String::new(),
            width_m: 5.5,
            kind: Kind::Ground,
            pts,
        };
        w.roads = Some(crate::world::Roads {
            plan: vec![line("a", vec![[-50.0, 0.0], [50.0, 0.0]]), line("b", vec![[0.0, -50.0], [0.0, 50.0]])],
            ..Default::default()
        });
        profile::run(&mut w);
        let s = run(&mut w);
        assert_eq!(s.num("crossings"), 0.0, "{s}");
        assert_eq!(s.num("same"), 1.0, "{s}");
        assert_eq!(s.num("ramped"), 0.0, "{s}");
        assert_eq!(w.crossing.as_ref().unwrap().same, vec![[0.0, 0.0]]);
    }

    #[test]
    fn a_proper_cross_is_the_only_cross() {
        assert_eq!(cross([-1.0, 0.0], [1.0, 0.0], [0.0, -1.0], [0.0, 1.0]), Some([0.0, 0.0]));
        // A touch at an endpoint, a T, a shared vertex and two parallels
        // are all not crossings.
        assert_eq!(cross([-1.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 1.0]), None);
        assert_eq!(cross([0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 1.0]), None);
        assert_eq!(cross([0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]), None);
        assert_eq!(cross([0.0, 0.0], [1.0, 0.0], [2.0, -1.0], [2.0, 1.0]), None);
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
