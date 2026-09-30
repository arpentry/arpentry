//! The profile solve: one height along every solving way's axis.
//!
//! Overture gives a way no height anywhere, and a deck is nothing but a
//! height: the chord of the way's profile across the span, between the
//! at-grade heights at its two abutments. The street on the hill is the
//! same profile with the terrain as its anchors. So the height comes first,
//! as a function along the axis, and every surface and every structure
//! reads it (docs/GENERATION.md §4.4, §4.5).
//!
//! A profile is one whole way, spans included, solved in one dimension in
//! this order:
//!
//! 1. **The target** is the reference at every station
//!    ([`crate::world::Reference`]: the conditioned terrain), plus the
//!    crossing step's floor where it has one ([`solve_on`]).
//! 2. **Way ends are anchors.** A way end on the ground takes one height
//!    at its connector, the same for every way that ends there: the target,
//!    which the reference step makes one value per connector. A junction
//!    stands on the ground, and continuity across it is a property of the
//!    variables rather than a constraint that can fail. A span boundary is
//!    not an anchor: a mapper's cut is not a survey point.
//! 3. **Grade, curve and box.** Per class a ceiling, a vertical curve and a
//!    deviation budget ([`crate::grade`]). For the engineered classes,
//!    forward and backward passes limit the rise between stations to the
//!    ceiling; for every solving class `bend` holds the vertical curve; a
//!    clamp to `target ± deviation` is applied last, so the box wins: a
//!    primary on a slope steeper than 8 % holds 8 % while its 4 m last and
//!    follows the hill beyond. A street has no ceiling — the ground under
//!    it is the street — so only its curve is smoothed, inside its box.
//!    The pins land after the box, and where one would put its segment
//!    over the ceiling, what it moves is spread along the run instead of
//!    standing in that one segment (`spread`).
//! 4. **Spans.** A structure run is a straight chord between its two
//!    **abutments**, the at-grade stations just outside it, at the heights
//!    the at-grade solve gave them; where the run reaches a way end, its end
//!    is the anchor there. Runs meeting at a connector no way is at grade at
//!    are solved together as one chain of chords. A dangling end — one
//!    nothing else meets, the bbox's edge cutting a viaduct — holds the
//!    anchored end's height, so a deck cut by the clip runs level to it
//!    (counted as `dangling`); a run no anchor reaches at all lies flat at
//!    the highest target under it (a bridge) or the lowest (a tunnel), and
//!    is counted too.
//! 5. **The solved kind.** At every station of a mapped span: a deck where
//!    the profile stands [`crate::standard::STRUCTURE_MIN_M`] off the
//!    ground, a bore where it runs that far under, grade between. A span
//!    that reads grade end to end has degraded to ground (invariant 6:
//!    plain, not wrong). A ground station never becomes a structure here;
//!    deriving one is the partition step's.
//!
//! Draped classes get no profile: a footpath samples the finished ground.
//!
//! **A railway's ceiling is measured** ([`grade::limit`]): raised to the grade
//! its own at-grade bed rides, because a rack railway is classed
//! `narrow_gauge` and held to 7 % it dives under its own track. And **where
//! a railway shares a connector with another way, both are at grade there**:
//! a level crossing is at grade by definition, and so is a switch. Overture
//! puts such a connector in the *interior* of both ways, where no anchor
//! sees it, so the shared station is pinned mid-run ([`Loose::contacts`]),
//! both ways to the railway's reference. The railway is senior: the road
//! meets the rails, never the other way round.

use std::collections::HashMap;

use crate::grade;
use crate::width::{self, Family};
use crate::world::{connector, station_runs, Kind, Profile, Reference, Solved, Station};

/// Forward-and-back passes of the grade limiter. Eight is the server's;
/// the passes converge geometrically and the box clamp after each keeps
/// the result inside its budget whatever the count.
const PASSES: usize = 8;

/// Passes of the curvature clamp per pass of the limiter.
///
/// The grade limiter converges geometrically because it walks the array in
/// both directions; the curvature clamp is local and Jacobi, so it *diffuses*
/// — a broad kink flattens at about one station per pass. Thirty-two is
/// where the share of runs still bent tighter than their class allows
/// (`kink`) stops falling quickly, and the solve stays cheap.
const BEND_PASSES: usize = 32;

/// Slack, in metres per metre, past the ceiling before a station pair
/// counts as breaking grade: the limiter's own rounding.
pub const GRADE_EPS: f64 = 1e-9;

/// What [`solve`] could not anchor — structure runs with a dangling end, and
/// structure runs no anchor reached at all — and what it pinned or overruled.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Loose {
    pub dangling: usize,
    pub unanchored: usize,
    /// Chord ends the geometry had to overrule: a bore's mouth standing above
    /// the ground, or a deck's landing under it.
    pub clamped: usize,
    /// Connectors where a railway meets another way on the ground — a level
    /// crossing, a switch — each pinned at the railway's reference.
    pub contacts: usize,
}

/// The profiles of the ways `reference` was built for, and what could not be
/// anchored. `reference.axes[i].way` indexes `reference.ways`, so the caller
/// neither filters nor orders anything and the correspondence cannot be got
/// wrong.
pub fn solve(reference: &Reference) -> (Vec<Profile>, Loose) {
    solve_on(reference, &[])
}

/// The same, over a *floor*: a displacement in metres the crossing step
/// adds to the reference at each station, indexed as `reference.axes` and
/// then by station, so an overpass's approach is built on the embankment
/// its clearance demands rather than on the hillside underneath it. Empty
/// is no floor, and [`solve`] is this with none.
///
/// The floor moves the **target**, not the answer: the anchor at a
/// connector, the value the grade limiter smooths and the centre of the
/// deviation box all read `reference + floor`, while [`Station::ground`]
/// stays the natural ground the whole way down — so the consequence rule
/// still asks "how far off the *hill* does this stand", and an approach
/// lifted past [`crate::standard::STRUCTURE_MIN_M`] reads as the deck it is.
pub fn solve_on(reference: &Reference, floor: &[Vec<f64>]) -> (Vec<Profile>, Loose) {
    let ways = &reference.ways;
    // The axes *are* the list of what solves, in order; `way(i)` is the way
    // the i-th of them is of.
    let n = reference.axes.len();
    let way = |i: usize| &ways[reference.axes[i].way];
    // Stations, from the reference: it stationed these axes already — at the
    // span boundaries too — so the profile and the surface it is solved
    // against can never be sampled at different places.
    let stationed: Vec<Vec<Station>> = reference
        .axes
        .iter()
        .map(|a| {
            (0..a.s.len())
                .map(|k| Station {
                    s: a.s[k],
                    p: a.p[k],
                    ground: a.ground[k],
                    reference: a.h[k],
                    h: a.h[k],
                    solved: Solved::Grade,
                })
                .collect()
        })
        .collect();

    // The target: the reference, raised or lowered by the floor. Two ways
    // meeting at one connector read one floor value there, so the anchor
    // below is the same number whichever of them inserts it.
    let target: Vec<Vec<f64>> = stationed
        .iter()
        .enumerate()
        .map(|(i, sts)| {
            sts.iter()
                .enumerate()
                .map(|(k, st)| {
                    st.reference + floor.get(i).and_then(|f| f.get(k)).copied().unwrap_or(0.0)
                })
                .collect()
        })
        .collect();

    let runs: Vec<Vec<(usize, usize, Kind)>> =
        (0..n).map(|i| station_runs(&stationed[i], &way(i).spans)).collect();
    let last_of = |i: usize| stationed[i].len().saturating_sub(1);

    // **Contacts.** Where a railway shares a connector with another way and
    // both are on the ground there, both pass through it at the railway's
    // reference: a level crossing and a switch are at grade by definition,
    // and the railway is senior, so the road meets the rails and not the
    // other way round. Overture puts those connectors in the *interior* of
    // both ways, which the anchors below never see — so they are pins of
    // their own, mid-run. The first railway to reach a connector names its
    // height, which is a function of the way order and nothing else.
    let contacts: HashMap<(i64, i64), f64> = {
        // Per connector: the first way at it, whether another way is, and
        // the first railway's reference.
        let mut at: HashMap<(i64, i64), (usize, bool, Option<f64>)> = HashMap::new();
        for i in 0..n {
            let rail = width::family(&way(i).class) == Family::Rail;
            for &(k0, k1, kind) in &runs[i] {
                if !on_ground(kind) {
                    continue;
                }
                for k in k0..=k1 {
                    let e = at.entry(connector(stationed[i][k].p)).or_insert((i, false, None));
                    e.1 |= e.0 != i;
                    if rail && e.2.is_none() {
                        e.2 = Some(stationed[i][k].reference);
                    }
                }
            }
        }
        at.into_iter().filter_map(|(key, (_, shared, rail))| rail.filter(|_| shared).map(|v| (key, v))).collect()
    };

    // **Anchors.** A way *end* is an anchor where the way is on the ground
    // there — not a span boundary, which would make a mapper's cut a survey
    // point. A junction stands
    // on the ground, and continuity across it is a property of the variables
    // rather than a constraint that can fail. At a contact the railway's
    // height is the junction's.
    let mut anchors: HashMap<(i64, i64), f64> = HashMap::new();
    for i in 0..n {
        if stationed[i].is_empty() {
            continue;
        }
        for (k, kind) in [
            (0usize, runs[i].first().map(|r| r.2)),
            (last_of(i), runs[i].last().map(|r| r.2)),
        ] {
            if kind.is_some_and(on_ground) {
                let key = connector(stationed[i][k].p);
                anchors.entry(key).or_insert(contacts.get(&key).copied().unwrap_or(target[i][k]));
            }
        }
    }

    // **The at-grade runs solve first**, each held to its class's ceiling
    // inside its box, pinned only where it reaches a way end that is an
    // anchor. A run that ends against a structure is left free there: the
    // chord will start from wherever the ground solve lands, which is the
    // whole point.
    let mut h: Vec<Vec<f64>> = target.clone();
    for i in 0..n {
        let g = grade::of(&way(i).class);
        let ceiling = grade::limit(&way(i).class, &stationed[i], &way(i).spans);
        // A pin at station `k`: the anchor at a way end, else a contact.
        let pin_at = |k: usize| -> Option<f64> {
            let key = connector(stationed[i][k].p);
            (k == 0 || k == last_of(i)).then(|| anchors.get(&key).copied()).flatten().or_else(|| contacts.get(&key).copied())
        };
        for &(k0, k1, kind) in &runs[i] {
            if !on_ground(kind) {
                continue;
            }
            // The run is cut at every contact inside it, and each piece is
            // limited between its two pins: the limiter pins ends only, and
            // a contact is an end of both pieces it separates.
            let mut cuts: Vec<usize> = vec![k0];
            cuts.extend((k0 + 1..k1).filter(|&k| contacts.contains_key(&connector(stationed[i][k].p))));
            cuts.push(k1);
            cuts.dedup();
            for (a, b) in cuts.windows(2).map(|w| (w[0], w[1])).chain((cuts.len() == 1).then_some((k0, k1))) {
                let arc: Vec<f64> = stationed[i][a..=b].iter().map(|st| st.s).collect();
                // A run that ends against a structure is left free there,
                // unless a contact stands on that very station.
                let pin = (pin_at(a), pin_at(b));
                let solved = limit(&target[i][a..=b], &arc, ceiling, g.deviation_m, g.radius_m, pin);
                h[i][a..=b].copy_from_slice(&solved);
            }
        }
    }

    // **The chords.** Every structure run's two ends are heights: the
    // at-grade solve's, where the run meets ground inside its own way; the
    // connector's, where it reaches a way end. A connector every incident way
    // is a structure at has no height of its own, and those are solved
    // together — a chain of chords across a bridge split at a junction is one
    // chord, so the free connectors take the harmonic solution (Jacobi, so
    // the result does not depend on the order).
    let mut chords: Vec<Chord> = Vec::new();
    // Per free connector: the ground there, and whether every chord meeting it
    // is of one kind. A mouth shared by a bore and a deck is not overruled —
    // there is no side of the ground both belong on.
    let mut ground_at: HashMap<(i64, i64), (f64, bool, bool)> = HashMap::new();
    for i in 0..n {
        for &(k0, k1, kind) in &runs[i] {
            if !kind.is_structure() {
                continue;
            }
            // A chord runs between its **abutments**, and an abutment is the
            // at-grade station just outside the run — not the run's own first
            // station. Interpolating from the run's own ends would place the
            // abutment height four metres inside the deck and leave a step at
            // each end of it.
            let (lo_k, hi_k) = (k0.saturating_sub(1), (k1 + 1).min(last_of(i)));
            let end = |k: usize, terminal: bool, inward: usize| -> End {
                if !terminal {
                    return End::Known(h[i][inward]);
                }
                let key = connector(stationed[i][k].p);
                match anchors.get(&key) {
                    Some(&v) => End::Known(v),
                    None => End::Free(key),
                }
            };
            let (lo_at, hi_at) = (
                if k0 == 0 { stationed[i][k0].s } else { stationed[i][lo_k].s },
                if k1 == last_of(i) { stationed[i][k1].s } else { stationed[i][hi_k].s },
            );
            for (k, e) in [(k0, end(k0, k0 == 0, lo_k)), (k1, end(k1, k1 == last_of(i), hi_k))] {
                if let End::Free(key) = e {
                    let slot = ground_at.entry(key).or_insert((stationed[i][k].reference, true, true));
                    slot.1 &= matches!(kind, Kind::Tunnel(_));
                    slot.2 &= matches!(kind, Kind::Bridge(_));
                }
            }
            chords.push(Chord {
                way: i,
                k0,
                k1,
                lo: end(k0, k0 == 0, lo_k),
                hi: end(k1, k1 == last_of(i), hi_k),
                lo_at,
                hi_at,
                kind,
                len: (hi_at - lo_at).max(1e-9),
            });
        }
    }

    let mut loose = Loose { contacts: contacts.len(), ..Loose::default() };
    let mut free: HashMap<(i64, i64), f64> = HashMap::new();
    // A free connector's degree: how many chords reach it. One is dangling —
    // the bbox cutting a viaduct — and it holds its chord's other end.
    let mut degree: HashMap<(i64, i64), usize> = HashMap::new();
    for c in &chords {
        for e in [c.lo, c.hi] {
            if let End::Free(k) = e {
                *degree.entry(k).or_insert(0) += 1;
                free.entry(k).or_insert(f64::NAN);
            }
        }
    }
    for _ in 0..20_000 {
        let mut moved = 0.0f64;
        let mut next = free.clone();
        for (key, v) in next.iter_mut() {
            let (mut num, mut den) = (0.0, 0.0);
            for c in &chords {
                let far = match (c.lo, c.hi) {
                    (End::Free(a), other) if a == *key => other,
                    (other, End::Free(b)) if b == *key => other,
                    _ => continue,
                };
                let far = match far {
                    End::Known(x) => x,
                    End::Free(k) => match free.get(&k) {
                        Some(x) if x.is_finite() => *x,
                        _ => continue,
                    },
                };
                num += far / c.len;
                den += 1.0 / c.len;
            }
            if den > 0.0 {
                let x = num / den;
                if v.is_finite() {
                    moved = moved.max((x - *v).abs());
                }
                *v = x;
            }
        }
        free = next;
        if moved < 1e-12 {
            break;
        }
    }

    // **Geometry wins over the tags** (docs/GENERATION.md §4.5). A connector
    // no way is at grade at has no height of its own, so the chords meeting
    // it inherit one from whatever they chain to — and a chain long enough
    // inherits nonsense: a way tagged `is_tunnel` end to end, with no
    // at-grade station anywhere, can come out far *above* the hillside it
    // bores through when everything holding it down is several junctions
    // away. A bore's mouth is where it meets the ground; it cannot be over
    // it, and a deck's landing cannot be under it. Where the inheritance says
    // otherwise the ground is believed and the move is counted.
    for (key, v) in free.iter_mut() {
        let Some(&(ground, all_bore, all_deck)) = ground_at.get(key) else {
            continue;
        };
        if !v.is_finite() {
            continue;
        }
        const SLACK_M: f64 = 1e-6;
        // **A dangling deck holds its level; a dangling bore holds the
        // ground.** A chord with one end anchored and the other reaching
        // nothing runs level to the anchor, and for a deck that is the named
        // deferral — the bbox cuts a viaduct and the descent to a lower
        // ground waits for a site with data past it. For a bore it is not: a
        // tunnel running level out of a hillside that falls away emerges into
        // the air, and what the structure step then builds is a viaduct.
        //
        // A deck *below* the ground is overruled only where something did
        // reach it, since there the inheritance is wrong rather than absent.
        let dangling = degree.get(key) == Some(&1);
        let bore_in_the_air = all_bore && *v > ground + SLACK_M;
        let deck_underground = all_deck && !dangling && *v < ground - SLACK_M;
        if bore_in_the_air || deck_underground {
            *v = ground;
            loose.clamped += 1;
        }
    }

    for c in &chords {
        let read = |e: End| match e {
            End::Known(x) => Some(x),
            End::Free(k) => free.get(&k).copied().filter(|x| x.is_finite()),
        };
        if matches!(c.lo, End::Free(k) if degree.get(&k) == Some(&1))
            || matches!(c.hi, End::Free(k) if degree.get(&k) == Some(&1))
        {
            loose.dangling += 1;
        }
        let (a, b) = (read(c.lo), read(c.hi));
        let (h0, h1) = match (a, b) {
            (Some(a), Some(b)) => (a, b),
            // Nothing reached it: flat at the target's extreme under the run,
            // high for a deck and low for a bore, and counted.
            _ => {
                loose.unanchored += 1;
                let vals = target[c.way][c.k0..=c.k1].iter().copied();
                let flat = match c.kind {
                    Kind::Tunnel(_) => vals.fold(f64::INFINITY, f64::min),
                    _ => vals.fold(f64::NEG_INFINITY, f64::max),
                };
                let one = a.or(b).unwrap_or(flat);
                (one, one)
            }
        };
        let span = (c.hi_at - c.lo_at).max(1e-9);
        for k in c.k0..=c.k1 {
            let t = (stationed[c.way][k].s - c.lo_at) / span;
            h[c.way][k] = h0 + (h1 - h0) * t;
        }
    }

    let profiles = (0..n)
        .map(|i| {
            let mut sts = stationed[i].clone();
            for (k, st) in sts.iter_mut().enumerate() {
                st.h = h[i][k];
            }
            let mut p = Profile {
                way: reference.axes[i].way,
                id: way(i).id.clone(),
                class: way(i).class.clone(),
                width_m: way(i).width_m,
                spans: way(i).spans.clone(),
                stations: sts,
            };
            // A ground station never becomes a structure here — deriving one
            // is the partition step's.
            p.classify();
            p
        })
        .collect();
    (profiles, loose)
}

/// Whether a span's kind is one the profile follows the ground through. An
/// indoor way is on the ground floor: it is kept out of the paved surface by
/// the partition, but its height is the ground's, so the way's profile does
/// not acquire a hole where a building is.
fn on_ground(kind: Kind) -> bool {
    !kind.is_structure()
}

/// One chord: a structure run and the two heights it runs between.
struct Chord {
    way: usize,
    k0: usize,
    k1: usize,
    lo: End,
    hi: End,
    /// The arcs the two ends stand at: the abutment stations, which are
    /// outside the run wherever the run has ground beside it.
    lo_at: f64,
    hi_at: f64,
    len: f64,
    /// What the source mapped over the run: the extreme a chord nothing
    /// anchors lies flat at. (Only a chord over the whole way can be
    /// unanchored, so this is also its way's one structure kind — asked of
    /// the chord because that is what it is about.)
    kind: Kind,
}

/// One end of a chord: a height the at-grade solve or an anchor already
/// fixed, or a connector with no ground at it that has to be solved for.
#[derive(Clone, Copy, PartialEq)]
enum End {
    Known(f64),
    Free((i64, i64)),
}

/// The heights along one at-grade run: the `ground` targets at arc lengths
/// `arc`, held to `ceiling` and to a vertical curve of `radius` where the
/// box of `deviation` allows, and pinned at each end the caller gives a pin
/// for.
pub fn limit(
    ground: &[f64],
    arc: &[f64],
    ceiling: f64,
    deviation: f64,
    radius: Option<f64>,
    pin: (Option<f64>, Option<f64>),
) -> Vec<f64> {
    let n = ground.len();
    let mut h = ground.to_vec();
    if n == 0 {
        return h;
    }
    // A run that ends against a structure has no pin there: the chord starts
    // from wherever this solve lands, rather than from the ground at a
    // mapper's cut.
    let pin_ends = |h: &mut Vec<f64>| {
        if let Some(v) = pin.0 {
            h[0] = v;
        }
        if let Some(v) = pin.1 {
            h[n - 1] = v;
        }
    };
    pin_ends(&mut h);
    // A street has no ceiling — the DEM under it *is* it — but it still has a
    // vertical curve, so the early return has to ask about both: asking about
    // the ceiling alone would skip `bend` for every street, which is most of
    // the network.
    if n < 2 || (!ceiling.is_finite() && radius.is_none()) {
        return h;
    }
    for pass in 0..PASSES {
        if ceiling.is_finite() {
            for i in 1..n {
                let c = ceiling * (arc[i] - arc[i - 1]);
                h[i] = h[i].clamp(h[i - 1] - c, h[i - 1] + c);
            }
            for i in (0..n - 1).rev() {
                let c = ceiling * (arc[i + 1] - arc[i]);
                h[i] = h[i].clamp(h[i + 1] - c, h[i + 1] + c);
            }
        }
        for _ in 0..BEND_PASSES {
            bend(&mut h, arc, radius);
        }
        for i in 0..n {
            h[i] = h[i].clamp(ground[i] - deviation, ground[i] + deviation);
        }
        if pass + 1 < PASSES {
            pin_ends(&mut h);
        }
    }
    spread(&mut h, arc, ceiling, pin);
    h
}

/// Lands the solve on its pins, spreading over the run what the last
/// segment cannot carry.
///
/// **The pins win over the box, and the box over the ceiling**, so where
/// the three cannot all hold the ceiling breaks — and it used to break in
/// one place. The limiter holds the ceiling out of the first pin, spends
/// its box, and follows the hill; the second pin then lands last, and
/// whatever the box had spent stands in the final segment. On a 50 % flank
/// held to 30 % in an 8 m box that is 8 m over one 4 m station: a grade of
/// 2.5 in a track bed on a slope of one half, which is a fin, not a
/// railway (`fin-ballast-1`: 2.83 m over the last 1.03 m of a 12.4 m way).
///
/// So where a pin would put its segment over the ceiling, the difference
/// between each pin and where the solve left that end is added along the
/// run instead, falling linearly to nothing at the other end: the run's
/// every segment steepens by `jump / length`, which is the least any
/// correction that lands on both pins can add to the steepest one. The box
/// is what gives way, and only near the pin — which is where the pin had
/// already broken it. A run whose pins land within the ceiling is pinned as
/// it always was, to the bit.
fn spread(h: &mut [f64], arc: &[f64], ceiling: f64, pin: (Option<f64>, Option<f64>)) {
    let n = h.len();
    let jump = (pin.0.map_or(0.0, |v| v - h[0]), pin.1.map_or(0.0, |v| v - h[n - 1]));
    let length = arc[n - 1] - arc[0];
    let over = |a: usize, b: usize, ha: f64, hb: f64| (hb - ha).abs() > ceiling * (arc[b] - arc[a]) + GRADE_EPS;
    let breaks = ceiling.is_finite()
        && length > 0.0
        && (pin.0.is_some_and(|v| over(0, 1, v, h[1])) || pin.1.is_some_and(|v| over(n - 2, n - 1, h[n - 2], v)));
    if breaks {
        for k in 0..n {
            let t = (arc[k] - arc[0]) / length;
            h[k] += jump.0 * (1.0 - t) + jump.1 * t;
        }
    }
    if let Some(v) = pin.0 {
        h[0] = v;
    }
    if let Some(v) = pin.1 {
        h[n - 1] = v;
    }
}

/// Holds the profile to its class's tightest **vertical curve**.
///
/// The ceiling bounds how steep a road is; this bounds how fast it may change
/// how steep it is. A 20 % road is fine and Switzerland is full of them; a
/// 20 % road meeting a flat one inside a metre is a ramp a car grounds on,
/// and no ceiling forbids it because neither grade is over the limit.
///
/// A vertical curve of radius `R` bends by `1 / R` per metre, so over a
/// station whose neighbours are `d1` and `d2` away the height may stand at
/// most `d1 · d2 / (2 R)` off the chord between them — the sagitta of that
/// curve. Clamping every interior station into that band is the second
/// derivative's version of what the two passes above do to the first, and it
/// composes with them the same way: the box still wins, and the pins still
/// win over the box.
///
/// Jacobi — every station reads the same snapshot — so the result does not
/// depend on the direction the array is walked, and repeated passes converge
/// rather than drift.
fn bend(h: &mut [f64], arc: &[f64], radius: Option<f64>) {
    let Some(r) = radius.filter(|r| *r > 0.0) else {
        return;
    };
    let n = h.len();
    if n < 3 {
        return;
    }
    let prev = h.to_vec();
    for i in 1..n - 1 {
        let (d1, d2) = (arc[i] - arc[i - 1], arc[i + 1] - arc[i]);
        if !(d1 > 0.0 && d2 > 0.0) {
            continue;
        }
        let t = d1 / (d1 + d2);
        let chord = prev[i - 1] + (prev[i + 1] - prev[i - 1]) * t;
        let sagitta = d1 * d2 / (2.0 * r);
        h[i] = prev[i].clamp(chord - sagitta, chord + sagitta);
    }
}

/// The tightest vertical curve a run of stations actually holds, as a radius
/// in metres — `f64::INFINITY` for a straight one. What `bend` is asked to
/// bound, measured back off the result.
pub fn curvature_radius(h: &[f64], arc: &[f64]) -> f64 {
    let mut worst = f64::INFINITY;
    for i in 1..h.len().saturating_sub(1) {
        let (d1, d2) = (arc[i] - arc[i - 1], arc[i + 1] - arc[i]);
        if !(d1 > 0.0 && d2 > 0.0) {
            continue;
        }
        let t = d1 / (d1 + d2);
        let chord = h[i - 1] + (h[i + 1] - h[i - 1]) * t;
        let off = (h[i] - chord).abs();
        if off > 1e-12 {
            worst = worst.min(d1 * d2 / (2.0 * off));
        }
    }
    worst
}

#[cfg(test)]
pub(crate) mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{built, upto};
    use crate::step::{Step, Summary};

    use crate::lattice::height_at;
    use crate::terrain::{self, tests::{dem, extent}};

    use crate::world::Way;
    use super::*;

    /// A world on the ground of `terrain` with the network of `net`, profiled.
    pub(crate) fn world(terrain_spec: &str, net: &str) -> (World, Summary) {
        let (w, ran) = built(terrain_spec, net, None, 5.0, &upto(Step::Profile));
        (w, ran.last())
    }

    fn profiles(w: &World) -> &[Profile] {
        &w.profile.as_ref().unwrap().profiles
    }

    #[test]
    fn flat_is_flat() {
        let (w, s) = world("flat", "net:straight?len=200");
        let p = profiles(&w);
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].stations.len(), 51);
        assert!(p[0].stations.iter().all(|st| st.h == 400.0 && st.ground == 400.0), "{:?}", p[0].stations[0]);
        assert_eq!(s.num("grade"), 0.0);
        assert_eq!(s.num("float"), 0.0);
        assert_eq!(s.num("step"), 0.0);
        assert_eq!(s.get("dem_residual"), Some("0.00/0.00/0.00"));
    }

    #[test]
    fn a_ramp_under_the_ceiling_is_the_ramp() {
        // A primary holds 8 %; a 5 % ramp is under it, so the profile is
        // the ramp itself, to the ulp of the lattice.
        let (w, s) = world("ramp?grade=0.05&bearing=90&radius=100000", "net:straight?len=200&class=primary");
        for st in &profiles(&w)[0].stations {
            assert!((st.h - st.ground).abs() < 1e-9, "{st:?}");
            assert!((st.h - (400.0 + 0.05 * st.p[0])).abs() < 1e-6, "{st:?}");
        }
        assert_eq!(s.num("grade"), 0.0);
        assert_eq!(s.num("float"), 0.0);
    }

    #[test]
    fn a_street_follows_its_hill_and_an_engineered_road_holds_its_grade() {
        // A residential on a 30 % slope is a residential on a 30 % slope:
        // the ground, exactly, and the summary says how steep.
        let (w, s) = world("ramp?grade=0.3&bearing=90&radius=100000", "net:straight?len=200");
        let p = &profiles(&w)[0];
        assert!(p.stations.iter().all(|st| st.h == st.ground), "{:?}", p.stations[10]);
        assert_eq!(s.get("grade"), Some("0/0 (0.00%)"), "{s}");
        assert_eq!(s.num("steep"), 50.0, "{s}");
        assert_eq!(s.num("float"), 0.0, "{s}");
        // A primary holds 8 % and may leave the ground by 4 m: it leaves
        // at its ceiling, spends its box, and follows the hill beyond. Its
        // two ends are pinned to the ground 60 m apart, which no 8 % road
        // 200 m long reaches, so the 4 m the box spent is taken back along
        // the run ([`spread`]): 2 % on every segment, not 100 % on the last.
        let (w, s) = world("ramp?grade=0.3&bearing=90&radius=100000", "net:straight?len=200&class=primary");
        let g = grade::of("primary");
        let p = &profiles(&w)[0];
        let share = g.deviation_m / 200.0;
        assert!(s.num("grade") > 0.0, "{s}");
        assert_eq!(s.num("float"), 0.0, "{s}");
        let (a, b) = (p.stations[0], p.stations[1]);
        assert_eq!(a.h, a.ground);
        assert!(((b.h - a.h) / (b.s - a.s) - (0.08 + share)).abs() < 1e-9, "{a:?} {b:?}");
        let deepest = p.stations.iter().map(|st| (st.h - st.ground).abs()).fold(0.0, f64::max);
        assert!(deepest > g.deviation_m * 0.75 && deepest <= g.deviation_m + 1e-9, "{deepest}");
        let steepest = p.stations.windows(2).map(|w| (w[1].h - w[0].h) / (w[1].s - w[0].s)).fold(0.0, f64::max);
        assert!(steepest < 0.3 + share + 1e-9, "{steepest}");
        // A motorway's box is wider: it is out of the ground by more, and
        // still never past its budget.
        let (w, s) = world("ramp?grade=0.3&bearing=90&radius=100000", "net:straight?len=200&class=motorway");
        assert_eq!(s.num("float"), 0.0, "{s}");
        let wider = profiles(&w)[0].stations.iter().map(|st| (st.h - st.ground).abs()).fold(0.0, f64::max);
        assert!(wider > deepest && wider <= grade::of("motorway").deviation_m + 1e-9, "{wider} against {deepest}");
    }

    /// **A road may be steep; it may not change how steep it is too fast.**
    /// Switzerland is full of 20 % roads and they are fine — what no car can
    /// drive is a 20 % road meeting a flat one inside a metre, and no ceiling
    /// forbids that because neither grade is over the limit.
    ///
    /// The specimen is a cliff: a 6 m step with no width at all, which the
    /// raw ground crosses as a vertical break. The conditioning shaves what
    /// it can, and the vertical curve holds what is left to the class's own
    /// radius.
    #[test]
    fn a_road_holds_its_class_s_vertical_curve() {
        for (class, want) in [
            ("residential", grade::RADIUS_STREET_M),
            ("secondary", grade::RADIUS_SECONDARY_M),
            ("motorway", grade::RADIUS_MOTORWAY_M),
        ] {
            let (w, s) = world("step?rise=6&width=0", &format!("net:straight?len=400&class={class}"));
            let p = &profiles(&w)[0];
            let arc: Vec<f64> = p.stations.iter().map(|st| st.s).collect();
            let h: Vec<f64> = p.stations.iter().map(|st| st.h).collect();
            let held = curvature_radius(&h, &arc);
            // The box and the pins may still force a tighter curve than the
            // class would choose, so this is not an equality — but the road
            // must be nowhere near the raw cliff it was solved from.
            assert!(held > 20.0, "{class} holds a {held:.0} m curve: {s}");
            let raw: Vec<f64> = p.stations.iter().map(|st| st.ground).collect();
            let ground = curvature_radius(&raw, &arc);
            assert!(held > ground * 4.0, "{class}: {held:.1} m against the ground's {ground:.1}");
            assert!(want > 0.0);
        }
    }

    /// **A run that cannot hold its ceiling breaks grade along its length,
    /// not in its last segment.** A railway 100 m long on a 50 % flank, held
    /// to 30 % inside an 8 m box and pinned to the ground at both ends,
    /// cannot meet all three: the pins are 50 m apart and the ceiling and the
    /// box together climb 38. The limiter holds its ceiling out of the first
    /// pin, spends its box and follows the hill — and then the second pin
    /// used to land last, over the box, and the whole 8 m the box had spent
    /// stood in the final 4 m segment: a grade of 2.5, a fin in the track bed
    /// on a slope of one half (the census's `fin-ballast-1`, 2.75 over its
    /// last metre). Spread over the run, the same 8 m costs 0.08 of grade.
    #[test]
    fn an_infeasible_run_spreads_what_it_cannot_hold() {
        let arc: Vec<f64> = (0..=25).map(|k| k as f64 * 4.0).collect();
        let ground: Vec<f64> = arc.iter().map(|s| 400.0 + 0.5 * s).collect();
        let pins = (Some(ground[0]), Some(ground[25]));
        let h = limit(&ground, &arc, 0.3, 8.0, Some(500.0), pins);
        assert_eq!((h[0], h[25]), (ground[0], ground[25]), "the pins hold");
        let grades: Vec<f64> = h.windows(2).zip(arc.windows(2)).map(|(h, s)| (h[1] - h[0]) / (s[1] - s[0])).collect();
        let worst = grades.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(worst < 0.5 + 8.0 / 100.0 + 1e-9, "a segment climbs {worst:.3}: {grades:.3?}");
        // A run the three can all hold is untouched, to the bit: the same
        // flank at a ceiling over it is the ground.
        assert_eq!(limit(&ground, &arc, 0.6, 8.0, Some(500.0), pins), ground);
        // And in the world: the rack railway across the street on that
        // flank, pinned at its two ends and at the level crossing.
        let (w, s) = world("ramp?grade=0.5&bearing=0&radius=100000", "net:level?rail=narrow_gauge");
        let rail = profiles(&w).iter().find(|p| p.id == "rail").expect("the railway solves");
        let worst = rail.stations.windows(2).map(|w| (w[1].h - w[0].h).abs() / (w[1].s - w[0].s)).fold(0.0, f64::max);
        assert!(worst < 0.6, "the railway climbs {worst:.3} somewhere: {s}");
    }

    /// And a *draped* class holds none: a stair is a sequence of vertical
    /// breaks, and bounding them would be a lie about what steps are.
    #[test]
    fn a_draped_class_has_no_vertical_curve() {
        assert_eq!(grade::of("steps").radius_m, None);
        assert_eq!(grade::of("footway").radius_m, None);
        assert!(grade::of("residential").radius_m.is_some());
    }

    #[test]
    fn a_span_is_a_chord_between_its_abutments() {
        // A valley 30 m in radius, 40 m deep; a span from −40 to 40 chords
        // across it at the level ground either side, 21 stations, one of
        // them at the origin.
        let (w, s) = world("hill?amp=-40&radius=30", "net:straight?len=200&span=0.3,0.7");
        let p = profiles(&w);
        assert_eq!(p.len(), 1, "one way, not three pieces");
        let span = &p[0];
        let (k0, k1, _) = span.runs().into_iter().find(|r| r.2.is_structure()).expect("a deck run");
        let deck: Vec<Station> = span.stations[k0..=k1].to_vec();
        assert_eq!(deck.len(), 19, "the deck's own stations, abutments excluded");
        let (h0, h1) = (deck[0].h, deck[deck.len() - 1].h);
        assert!((h0 - 400.0).abs() < 1e-9 && (h1 - 400.0).abs() < 1e-9, "{h0} {h1}");
        // The chord is straight between its two abutments.
        let (s0, s1) = (deck[0].s, deck[deck.len() - 1].s);
        for st in &deck {
            let chord = h0 + (h1 - h0) * (st.s - s0) / (s1 - s0);
            assert!((st.h - chord).abs() < 1e-9, "{st:?} vs {chord}");
        }
        let t = w.terrain.as_ref().expect("the terrain step ran");
        let mid = deck.iter().find(|st| st.p == [0.0, 0.0]).expect("a station at the origin");
        let clearance = mid.h - height_at(t, 0.0, 0.0);
        assert!((clearance - (400.0 - height_at(t, 0.0, 0.0))).abs() < 1e-9 && clearance > 35.0, "{clearance}");
        // Deck wherever the valley is under it by the threshold: out to
        // 24 m for sure (3.8 m deep there), never at the abutments.
        for st in &deck {
            if st.p[0].abs() <= 24.0 {
                assert_eq!(st.solved, Solved::Deck, "{st:?}");
            }
        }
        assert_eq!(deck[0].solved, Solved::Grade);
        assert_eq!(deck[deck.len() - 1].solved, Solved::Grade);
        let decks = deck.iter().filter(|st| st.solved == Solved::Deck).count();
        assert!(decks >= 13, "{decks}");
        assert_eq!(s.num("decks"), decks as f64, "{s}");
        // The abutment is where the at-grade solve landed: one height, no
        // step, and no pin at the mapper's own cut.
        assert_eq!(s.num("step"), 0.0, "{s}");
        assert_eq!(s.num("degraded"), 0.0, "{s}");
        assert_eq!(s.num("unanchored"), 0.0, "{s}");
        assert_eq!(s.num("dangling"), 0.0, "{s}");
        // A tunnel through the hill is the mirror.
        let (w, s) = world("hill?amp=40&radius=30", "net:straight?len=200&span=0.3,0.7&kind=tunnel");
        let p = profiles(&w);
        let bore = &p[0];
        let (k0, k1, _) = bore.runs().into_iter().find(|r| r.2.is_structure()).expect("a bore run");
        for st in &bore.stations[k0..=k1] {
            if st.p[0].abs() <= 24.0 {
                assert_eq!(st.solved, Solved::Bore, "{st:?}");
            }
        }
        assert_eq!(s.num("bores"), decks as f64, "{s}");
    }

    #[test]
    fn a_span_that_never_leaves_the_ground_degrades() {
        let (w, s) = world("flat", "net:straight?len=200&span=0.35,0.65");
        let p = profiles(&w);
        let span = &p[0];
        assert!(span.stations.iter().all(|st| st.solved == Solved::Grade && st.h == 400.0));
        assert_eq!(s.get("degraded"), Some("1/1 (100.00%)"), "{s}");
        assert_eq!(s.get("decks"), Some("0/15 (0.00%)"), "{s}");
    }

    #[test]
    fn a_junction_has_one_height() {
        let (w, s) = world("hill?amp=60&radius=400", "net:cross?len=400");
        let t = w.terrain.as_ref().expect("the terrain step ran");
        let top = height_at(t, 0.0, 0.0);
        let mut at_origin = 0;
        for p in profiles(&w) {
            for st in [p.stations[0], p.stations[p.stations.len() - 1]] {
                if st.p == [0.0, 0.0] {
                    at_origin += 1;
                    assert_eq!(st.h, top, "{}", p.id);
                }
            }
        }
        assert_eq!(at_origin, 4);
        assert_eq!(s.num("step"), 0.0, "{s}");
    }

    #[test]
    fn a_span_split_at_a_connector_is_one_chord() {
        // Two bridge runs of 30 m and 10 m between anchors at 400 and
        // 420: the connector between them lies on the one chord, at 415.
        let (ground, _) = terrain::run(&extent(), &mut dem("ramp?grade=0.5&bearing=90&radius=100000"), 5.0, usize::MAX);
        // Two ways, each half ground and half bridge, meeting at a
        // structure-only connector at x = 30. Nothing is at grade there, so
        // that connector has no height of its own and the two chords are
        // solved together as one.
        let ways = [
            way("a", vec![[-20.0, 0.0], [30.0, 0.0]], vec![
                (0.0, 20.0, Kind::Ground),
                (20.0, 50.0, Kind::Bridge(1)),
            ]),
            way("b", vec![[30.0, 0.0], [60.0, 0.0]], vec![
                (0.0, 10.0, Kind::Bridge(1)),
                (10.0, 30.0, Kind::Ground),
            ]),
        ];
        let t = &ground;
        let (profiles, loose) = solved(&ways, t);
        assert_eq!(loose, Loose::default());
        let (a, b) = (&profiles[0], &profiles[1]);
        let joint = a.stations[a.stations.len() - 1].h;
        assert!((joint - b.stations[0].h).abs() < 1e-9, "{joint} vs {}", b.stations[0].h);
        assert!((joint - 415.0).abs() < 1e-6, "{joint}");
        // The way starts at x = −20, where the ramp reads 390.
        assert!((a.stations[0].h - 390.0).abs() < 1e-6, "{}", a.stations[0].h);
        // Way b runs on to x = 60, where the ramp reads 430; its abutment at
        // x = 40 is the 420 the chord climbs to.
        assert!((b.stations[b.stations.len() - 1].h - 430.0).abs() < 1e-6);
        let abut = b.stations.iter().find(|st| (st.p[0] - 40.0).abs() < 1e-6).expect("the abutment");
        assert!((abut.h - 420.0).abs() < 1e-6, "{}", abut.h);
        // The ground at 30 m is 415 too, so the chord runs at grade there,
        // and the deck stands off the ground only where the ramp is not
        // the chord — which on a plane it is everywhere.
        assert!(a.stations.iter().all(|st| st.solved == Solved::Grade));
    }

    /// A way from points and a span table in arc.
    ///
    /// A **street**: not grade-limited, so its at-grade stretches lie on the
    /// ground exactly and what these tests measure is the chord rather than
    /// the limiter. An abutment is wherever the at-grade solve lands, so a
    /// `primary` on one of these ramps spends its deviation box before the
    /// chord even starts — true, and not what is being asked.
    fn way(id: &str, pts: Vec<[f64; 2]>, spans: Vec<(f64, f64, Kind)>) -> Way {
        Way {
            id: id.into(),
            class: "residential".into(),
            subclass: String::new(),
            width_m: 5.5,
            pts,
            spans: spans.into_iter().map(|(a0, a1, kind)| crate::world::Span { a0, a1, kind }).collect(),
            layers: Vec::new(),
        }
    }

    /// The profiles of a hand-made way list over `t`, the way the pipeline
    /// gets them: a reference for the ways that solve, then the solve.
    fn solved(ways: &[Way], t: &crate::world::Terrain) -> (Vec<Profile>, Loose) {
        solve(&crate::reference::of(ways, &crate::reference::solving_of(ways), t))
    }

    #[test]
    fn an_unreached_span_lies_flat_and_is_counted() {
        let (ground, _) = terrain::run(&extent(), &mut dem("hill?amp=-40&radius=30"), 5.0, usize::MAX);
        let bridge = way("lone", vec![[-50.0, 0.0], [50.0, 0.0]], vec![(0.0, 100.0, Kind::Bridge(1))]);
        let t = &ground;
        let (profiles, loose) = solved(&[bridge.clone()], t);
        assert_eq!(loose, Loose { dangling: 1, unanchored: 1, clamped: 0, contacts: 0 });
        let top = profiles[0].stations.iter().map(|st| st.ground).fold(f64::NEG_INFINITY, f64::max);
        assert!(profiles[0].stations.iter().all(|st| st.h == top));
        let mut tunnel = bridge.clone();
        tunnel.spans[0].kind = Kind::Tunnel(-1);
        let (profiles, _) = solved(&[tunnel], t);
        let floor = profiles[0].stations.iter().map(|st| st.ground).fold(f64::INFINITY, f64::min);
        assert!(profiles[0].stations.iter().all(|st| st.h == floor));
    }

    #[test]
    fn a_span_cut_by_the_clip_runs_level_to_its_anchor() {
        // A bridge leaving the world across a valley's far side: one
        // anchor, the other end dangling, so the deck holds the anchor's
        // height and is counted.
        let (ground, _) = terrain::run(&extent(), &mut dem("ramp?grade=0.1&bearing=90&radius=100000"), 5.0, usize::MAX);
        // One way: 20 m of ground, then 60 m of deck running off the world.
        let ways = [way("a", vec![[-20.0, 0.0], [60.0, 0.0]], vec![
            (0.0, 20.0, Kind::Ground),
            (20.0, 80.0, Kind::Bridge(1)),
        ])];
        let t = &ground;
        let (profiles, loose) = solved(&ways, t);
        assert_eq!(loose, Loose { dangling: 1, unanchored: 0, clamped: 0, contacts: 0 });
        let a = &profiles[0];
        let deck: Vec<&Station> = a.stations.iter().filter(|st| st.s >= 20.0).collect();
        assert!(deck.iter().all(|st| (st.h - 400.0).abs() < 1e-6), "{:?}", deck[5]);
        // Level over rising ground: a bore, by consequence, past 5 m out.
        assert_eq!(deck[deck.len() - 1].solved, Solved::Bore);
    }

    /// **A dangling deck holds its level; a dangling bore holds the ground.**
    ///
    /// A chord with one end anchored and the other reaching nothing runs
    /// level to the anchor. For a deck that is the named deferral — the bbox
    /// cuts a viaduct and the descent to a lower ground waits for a site with
    /// data past it. For a bore it is not: run level out of a hillside that
    /// falls away, a tunnel emerges into the air, and what the structure step
    /// then builds is a viaduct.
    #[test]
    fn a_dangling_bore_holds_the_ground_and_a_dangling_deck_its_level() {
        // Falling ground: 10 % down toward the east.
        let (ground, _) = terrain::run(&extent(), &mut dem("ramp?grade=-0.1&bearing=90&radius=100000"), 5.0, usize::MAX);
        let t = &ground;
        let check = |kind: Kind| -> Vec<Station> {
            let ways = [
                way("w", vec![[-20.0, 0.0], [0.0, 0.0]], vec![(0.0, 20.0, Kind::Ground)]),
                way("a", vec![[0.0, 0.0], [200.0, 0.0]], vec![(0.0, 200.0, kind)]),
            ];
            let (profiles, loose) = solved(&ways, t);
            assert_eq!(loose.dangling, 1, "{kind:?}");
            profiles[1].stations.clone()
        };
        // The deck: level from its one anchor at 400, out over ground that
        // has fallen to 380. Counted as dangling, and left alone.
        let deck = check(Kind::Bridge(1));
        assert!(deck.iter().all(|st| (st.h - 400.0).abs() < 1e-6), "{:?}", deck[10]);
        assert_eq!(deck[deck.len() - 1].solved, Solved::Deck);
        // The bore: the same chord would stand 20 m over the hillside, so the
        // free end takes the ground instead and the tunnel follows it down.
        let bore = check(Kind::Tunnel(-1));
        let end = bore[bore.len() - 1];
        assert!((end.h - end.ground).abs() < 1e-6, "the bore is in the air: {end:?}");
        assert!(bore.iter().all(|st| st.h <= st.ground + 1e-6), "a bore over the ground");
    }

    #[test]
    fn walks_and_indoor_pieces_get_no_profile() {
        let (w, _) = world("flat", "net:sidewalk?d=6");
        assert_eq!(profiles(&w).len(), 1);
        assert_eq!(profiles(&w)[0].id, "road");
    }

    /// **A level crossing is one height, and it is the railway's.** The
    /// connector lies in the interior of both ways, as Overture's level
    /// crossings do, so no anchor sees it; a hill under it means a mainline
    /// held to 3 % would cut the crest while the street followed it over,
    /// and the two would cross each other a metre apart.
    #[test]
    fn a_level_crossing_is_one_height_and_it_is_the_railways() {
        let (w, s) = world("hill?amp=20&radius=150", "net:level?len=400");
        let at = |id: &str| {
            let p = profiles(&w).iter().find(|p| p.id == id).unwrap_or_else(|| panic!("no {id}"));
            *p.stations.iter().find(|st| st.p == [0.0, 0.0]).expect("a station at the connector")
        };
        let (road, rail) = (at("road"), at("rail"));
        assert_eq!(road.h, rail.h, "{road:?} {rail:?}");
        assert_eq!(rail.h, rail.reference, "the railway crosses at its own reference");
        assert_eq!(s.num("contacts"), 1.0, "{s}");
        assert_eq!(s.num("level"), 0.0, "{s}");
        // Away from the crossing the railway is a railway again: it holds
        // its grade where the street follows the hill.
        assert_eq!(s.num("grade"), 0.0, "{s}");
    }

    /// **A railway rides the bed it is measured on.** Overture has no class
    /// for a rack railway: the Glion–Rochers-de-Naye line is `narrow_gauge`,
    /// whose adhesion prior is 7 %, and climbs at 20 %. Held to its class it
    /// would dive under its own track bed and spend its whole box doing it.
    #[test]
    fn a_railway_rides_the_bed_it_is_measured_on() {
        let (w, s) = world("ramp?grade=0.2&bearing=90&radius=100000", "net:straight?len=400&class=narrow_gauge");
        let p = &profiles(&w)[0];
        assert!(p.stations.iter().all(|st| (st.h - st.ground).abs() < 1e-6), "{:?}", p.stations[20]);
        assert_eq!(s.num("raised"), 1.0, "{s}");
        assert_eq!(s.num("grade"), 0.0, "{s}");
        // Past the cap the ceiling holds: at 40 % the line climbs at 30 %,
        // spends its box, and follows the hill beyond — a gentle class may
        // not claim a cliff. The far pin takes back what the box spent over
        // the whole run ([`spread`]), so every segment carries a share of
        // it and none carries it all.
        let (w, s) = world("ramp?grade=0.4&bearing=90&radius=100000", "net:straight?len=400&class=narrow_gauge");
        let p = &profiles(&w)[0];
        let box_m = grade::of("narrow_gauge").deviation_m;
        let share = box_m / 400.0;
        let (a, b) = (p.stations[0], p.stations[1]);
        assert!(((b.h - a.h) / (b.s - a.s) - (grade::MEASURED_FLOOR + share)).abs() < 1e-9, "{a:?} {b:?}");
        let deepest = p.stations.iter().map(|st| (st.h - st.ground).abs()).fold(0.0, f64::max);
        assert!(deepest > box_m * 0.75 && deepest <= box_m + 1e-9, "{deepest}");
        let steepest = p.stations.windows(2).map(|w| (w[1].h - w[0].h) / (w[1].s - w[0].s)).fold(0.0, f64::max);
        assert!(steepest < 0.4 + share + 1e-9, "{steepest}");
        assert_eq!(s.num("float"), 0.0, "{s}");
        // A road's class says how it climbs: nothing is raised for it.
        let (_, s) = world("ramp?grade=0.2&bearing=90&radius=100000", "net:straight?len=400&class=primary");
        assert_eq!(s.num("raised"), 0.0, "{s}");
    }
}
