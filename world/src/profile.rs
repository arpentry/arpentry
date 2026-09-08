//! The profile: one height along every carriageway axis.
//!
//! Overture gives a way no height anywhere, and a deck is nothing but a
//! height: the chord of the way's profile across the span, between the
//! ground at its two abutments. The street on the hill is the same profile
//! with the terrain as its anchors. So the height comes first, as a
//! function along the axis, and every surface and every structure after
//! this step reads it (docs/GENERATION.md §4.4, §4.5).
//!
//! The solve is per piece and in one dimension, in this order:
//!
//! 1. **The target** is the ground at every station ([`height_at`]).
//! 2. **Connectors are shared.** Every way end at a connector takes one
//!    height, the same for every piece that ends there: the ground at the
//!    connector. A junction stands on the ground, and continuity across it
//!    is a property of the variables rather than a constraint that can
//!    fail. Only where nothing but structure pieces meet — a bridge split
//!    mid-span — is the connector's height the chords'.
//! 3. **Grade and the box**, for the engineered classes only. Per class a
//!    ceiling and a deviation budget ([`crate::grade`]). Forward and
//!    backward passes limit the rise between stations to the ceiling; a
//!    clamp to `target ± deviation` is applied last, so the box wins: a
//!    primary on a slope steeper than 8 % holds 8 % while its 4 m last and
//!    follows the hill beyond. A street is not limited: the ground under
//!    it is the street (S9), and its profile is the ground through the
//!    pinned ends.
//! 4. **Spans.** A bridge or tunnel piece is a straight chord between its
//!    two end heights; its ends are anchors where they meet a ground piece.
//!    Pieces meeting at structure-only connectors form one chord across
//!    all of them; a dangling end — one nothing else meets, the bbox's edge
//!    cutting a viaduct — holds the anchored end's height, so a deck cut
//!    by the clip runs level to it (counted, as `dangling`: the descent to
//!    a lower ground is named and waits for a site with data past it); a
//!    piece no anchor reaches at all lies flat at the highest ground under
//!    it (a bridge) or the lowest (a tunnel), and is counted too.
//! 5. **The solved kind.** At every station of a mapped span: a deck where
//!    the profile stands [`STRUCTURE_MIN_M`] off the ground, a bore where
//!    it runs that far under, grade between. A span that reads grade end
//!    to end has degraded to ground (invariant 6: plain, not wrong). A
//!    ground piece never becomes a structure here.
//!
//! Draped classes get no profile: a footpath samples the finished ground.

use std::collections::HashMap;

use crate::grade::{self, NODE_M, STRUCTURE_MIN_M};
use crate::step::Summary;
use crate::terrain::height_at;
use crate::width::{self, Family};
use crate::world::{connector, Kind, Polyline2, Profile, Profiles, Solved, Station, Terrain, World};

/// Forward-and-back passes of the grade limiter. Eight is the server's;
/// the passes converge geometrically and the box clamp after each keeps
/// the result inside its budget whatever the count.
const PASSES: usize = 8;

/// Slack, in metres per metre, past the ceiling before a station pair
/// counts as breaking grade: the limiter's own rounding.
const GRADE_EPS: f64 = 1e-9;

/// Solves the profile of every carriageway piece of the world.
pub fn run(world: &mut World) -> Summary {
    let terrain = world.terrain.as_ref().expect("the terrain step runs first");
    let roads = world.roads.as_ref().expect("the drape step runs first");
    let pieces: Vec<&Polyline2> = roads
        .pieces()
        .filter(|w| width::family(&w.class) == Family::Carriageway && w.kind != Kind::Indoor)
        .filter(|w| grade::of(&w.class).solves())
        .collect();
    let (profiles, loose) = solve(terrain, &pieces);

    let mut stations = 0usize;
    let (mut pairs, mut steep) = (0usize, 0usize);
    let (mut street_pairs, mut street_steep) = (0usize, 0usize);
    let (mut grounded, mut floating) = (0usize, 0usize);
    let mut off: Vec<f64> = Vec::new();
    let (mut deck, mut bridge) = (0usize, 0usize);
    let (mut bore, mut tunnel) = (0usize, 0usize);
    let (mut spans, mut degraded) = (0usize, 0usize);
    let mut ends: HashMap<(i64, i64), (f64, f64)> = HashMap::new();
    for p in &profiles {
        stations += p.stations.len();
        let g = grade::of(&p.class);
        for st in [p.stations.first(), p.stations.last()].into_iter().flatten() {
            let e = ends.entry(connector(st.p)).or_insert((st.h, st.h));
            e.0 = e.0.min(st.h);
            e.1 = e.1.max(st.h);
        }
        if p.mapped == Kind::Ground {
            for pair in p.stations.windows(2) {
                let ds = pair[1].s - pair[0].s;
                if ds > 0.0 {
                    let over = (pair[1].h - pair[0].h).abs() / ds > g.ceiling.unwrap_or(f64::INFINITY) + GRADE_EPS;
                    if g.limited() {
                        pairs += 1;
                        steep += over as usize;
                    } else {
                        street_pairs += 1;
                        street_steep += over as usize;
                    }
                }
            }
            for st in &p.stations {
                grounded += 1;
                let d = st.h - st.ground;
                off.push(d.abs());
                if d.abs() > g.deviation_m + 1e-9 {
                    floating += 1;
                }
            }
        } else {
            spans += 1;
            if p.stations.iter().all(|st| st.solved == Solved::Grade) {
                degraded += 1;
            }
            for st in &p.stations {
                match p.mapped {
                    Kind::Bridge(_) => {
                        bridge += 1;
                        deck += (st.solved == Solved::Deck) as usize;
                    }
                    Kind::Tunnel(_) => {
                        tunnel += 1;
                        bore += (st.solved == Solved::Bore) as usize;
                    }
                    _ => {}
                }
            }
        }
    }
    let step = ends.values().map(|(lo, hi)| hi - lo).fold(0.0f64, f64::max);
    off.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    let q = |f: f64| off.get(((off.len() as f64 - 1.0) * f).round() as usize).copied().unwrap_or(0.0);
    let summary = Summary::new()
        .with("pieces", profiles.len())
        .with("stations", stations)
        .with_share("grade", steep, pairs)
        .with_share("steep", street_steep, street_pairs)
        .with_share("float", floating, grounded)
        .with("step", format!("{step:.3}"))
        .with("off", format!("{:.2}/{:.2}/{:.2}", q(0.5), q(0.9), q(1.0)))
        .with_share("decks", deck, bridge)
        .with_share("bores", bore, tunnel)
        .with_share("degraded", degraded, spans)
        .with("dangling", loose.dangling)
        .with("unanchored", loose.unanchored);
    world.profile = Some(Profiles { profiles });
    summary
}

/// What [`solve`] could not anchor: structure pieces with a dangling end,
/// and structure pieces no anchor reached at all.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Loose {
    pub dangling: usize,
    pub unanchored: usize,
}

/// The profiles of `pieces` on `terrain`, and what could not be anchored.
pub fn solve(terrain: &Terrain, pieces: &[&Polyline2]) -> (Vec<Profile>, Loose) {
    // Stations and the ground under them.
    let stationed: Vec<Vec<Station>> = pieces
        .iter()
        .map(|w| {
            let pts = densify(&w.pts, NODE_M);
            let mut s = 0.0;
            let mut out = Vec::with_capacity(pts.len());
            for (i, p) in pts.iter().enumerate() {
                if i > 0 {
                    s += (p[0] - pts[i - 1][0]).hypot(p[1] - pts[i - 1][1]);
                }
                let ground = height_at(terrain, p[0], p[1]);
                out.push(Station { s, p: *p, ground, h: ground, solved: Solved::Grade });
            }
            out
        })
        .collect();

    // Anchors: every connector a ground piece ends at takes the ground
    // there, once, and every piece ending there reads that one number.
    let mut anchors: HashMap<(i64, i64), f64> = HashMap::new();
    for (w, sts) in pieces.iter().zip(&stationed) {
        if w.kind != Kind::Ground {
            continue;
        }
        for st in [sts.first(), sts.last()].into_iter().flatten() {
            anchors.entry(connector(st.p)).or_insert(st.ground);
        }
    }

    // Structure-only connectors: the harmonic solution over the structure
    // pieces, with the anchors as boundary values — a chord across every
    // chain, iterated (Jacobi, so the result is independent of order).
    let structure: Vec<usize> = (0..pieces.len()).filter(|&i| pieces[i].kind.is_structure()).collect();
    let key_of = |sts: &Vec<Station>| (connector(sts[0].p), connector(sts[sts.len() - 1].p));
    let mut free: Vec<(i64, i64)> = Vec::new();
    for &i in &structure {
        let (a, b) = key_of(&stationed[i]);
        for k in [a, b] {
            if !anchors.contains_key(&k) && !free.contains(&k) {
                free.push(k);
            }
        }
    }
    // A dangling end: a connector one piece alone reaches.
    let mut degree: HashMap<(i64, i64), usize> = HashMap::new();
    for sts in &stationed {
        let (a, b) = key_of(sts);
        *degree.entry(a).or_insert(0) += 1;
        *degree.entry(b).or_insert(0) += 1;
    }
    let mut loose = Loose::default();
    for &i in &structure {
        let (a, b) = key_of(&stationed[i]);
        if [a, b].iter().any(|k| degree[k] == 1 && !anchors.contains_key(k)) {
            loose.dangling += 1;
        }
    }
    let mut heights: HashMap<(i64, i64), f64> = anchors.clone();
    for _ in 0..20_000 {
        let mut next = heights.clone();
        let mut moved = 0.0f64;
        for k in &free {
            let (mut num, mut den) = (0.0, 0.0);
            for &i in &structure {
                let (a, b) = key_of(&stationed[i]);
                let far = if a == *k { b } else if b == *k { a } else { continue };
                if let Some(&h) = heights.get(&far) {
                    let len = stationed[i][stationed[i].len() - 1].s.max(1e-9);
                    num += h / len;
                    den += 1.0 / len;
                }
            }
            if den > 0.0 {
                let h = num / den;
                moved = moved.max((h - heights.get(k).copied().unwrap_or(h)).abs());
                next.insert(*k, h);
            }
        }
        heights = next;
        if moved < 1e-12 {
            break;
        }
    }
    // What no anchor reached: flat at the ground's extreme under the piece.
    for &i in &structure {
        let (a, b) = key_of(&stationed[i]);
        if heights.contains_key(&a) && heights.contains_key(&b) {
            continue;
        }
        loose.unanchored += 1;
        let grounds = stationed[i].iter().map(|st| st.ground);
        let flat = match pieces[i].kind {
            Kind::Tunnel(_) => grounds.fold(f64::INFINITY, f64::min),
            _ => grounds.fold(f64::NEG_INFINITY, f64::max),
        };
        for k in [a, b] {
            heights.entry(k).or_insert(flat);
        }
    }

    let profiles = pieces
        .iter()
        .zip(stationed)
        .map(|(w, mut sts)| {
            let (a, b) = key_of(&sts);
            let (h0, h1) = (heights[&a], heights[&b]);
            if w.kind == Kind::Ground {
                let g = grade::of(&w.class);
                let ground: Vec<f64> = sts.iter().map(|st| st.ground).collect();
                let arc: Vec<f64> = sts.iter().map(|st| st.s).collect();
                let ceiling = if g.limited() { g.ceiling.unwrap_or(f64::INFINITY) } else { f64::INFINITY };
                let h = limit(&ground, &arc, ceiling, g.deviation_m, (h0, h1));
                for (st, h) in sts.iter_mut().zip(h) {
                    st.h = h;
                }
            } else {
                let len = sts[sts.len() - 1].s;
                for st in sts.iter_mut() {
                    st.h = if len > 0.0 { h0 + (h1 - h0) * st.s / len } else { h0 };
                    st.solved = if st.h - st.ground >= STRUCTURE_MIN_M {
                        Solved::Deck
                    } else if st.ground - st.h >= STRUCTURE_MIN_M {
                        Solved::Bore
                    } else {
                        Solved::Grade
                    };
                }
            }
            Profile { id: w.id.clone(), class: w.class.clone(), width_m: w.width_m, mapped: w.kind, stations: sts }
        })
        .collect();
    (profiles, loose)
}

/// The heights along one ground piece: the `ground` targets at arc lengths
/// `arc`, held to `ceiling` where the box of `deviation` allows and pinned
/// to `pin` at the two ends.
pub fn limit(ground: &[f64], arc: &[f64], ceiling: f64, deviation: f64, pin: (f64, f64)) -> Vec<f64> {
    let n = ground.len();
    let mut h = ground.to_vec();
    if n == 0 {
        return h;
    }
    let pin_ends = |h: &mut Vec<f64>| {
        h[0] = pin.0;
        h[n - 1] = pin.1;
    };
    pin_ends(&mut h);
    if n < 2 || !ceiling.is_finite() {
        return h;
    }
    for _ in 0..PASSES {
        for i in 1..n {
            let c = ceiling * (arc[i] - arc[i - 1]);
            h[i] = h[i].clamp(h[i - 1] - c, h[i - 1] + c);
        }
        for i in (0..n - 1).rev() {
            let c = ceiling * (arc[i + 1] - arc[i]);
            h[i] = h[i].clamp(h[i + 1] - c, h[i + 1] + c);
        }
        for i in 0..n {
            h[i] = h[i].clamp(ground[i] - deviation, ground[i] + deviation);
        }
        pin_ends(&mut h);
    }
    h
}

/// `pts` with points inserted so no piece is longer than `step`, the
/// original vertices kept: the axis is not moved, only sampled.
pub fn densify(pts: &[[f64; 2]], step: f64) -> Vec<[f64; 2]> {
    let mut out: Vec<[f64; 2]> = Vec::new();
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

#[cfg(test)]
pub(crate) mod tests {
    use std::path::Path;

    use crate::drape;
    use crate::terrain::{self, tests::dem};

    use super::*;

    /// A world on the ground of `terrain` with the network of `net`, profiled.
    pub(crate) fn world(terrain_spec: &str, net: &str) -> (World, Summary) {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem(terrain_spec), 5.0, usize::MAX);
        drape::run(&mut w, Path::new(net)).unwrap();
        let s = run(&mut w);
        (w, s)
    }

    fn profiles(w: &World) -> &[Profile] {
        &w.profile.as_ref().unwrap().profiles
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
        assert_eq!(s.get("off"), Some("0.00/0.00/0.00"));
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
        // the ground, exactly, and the summary says how steep (S9).
        let (w, s) = world("ramp?grade=0.3&bearing=90&radius=100000", "net:straight?len=200");
        let p = &profiles(&w)[0];
        assert!(p.stations.iter().all(|st| st.h == st.ground), "{:?}", p.stations[10]);
        assert_eq!(s.get("grade"), Some("0/0 (0.00%)"), "{s}");
        assert_eq!(s.num("steep"), 50.0, "{s}");
        assert_eq!(s.num("float"), 0.0, "{s}");
        // A primary holds 8 % and may leave the ground by 4 m: it leaves
        // at its ceiling, spends its box, and follows the hill beyond.
        let (w, s) = world("ramp?grade=0.3&bearing=90&radius=100000", "net:straight?len=200&class=primary");
        let g = grade::of("primary");
        let p = &profiles(&w)[0];
        assert!(s.num("grade") > 0.0, "{s}");
        assert_eq!(s.num("float"), 0.0, "{s}");
        let (a, b) = (p.stations[0], p.stations[1]);
        assert_eq!(a.h, a.ground);
        assert!(((b.h - a.h) / (b.s - a.s) - 0.08).abs() < 1e-9, "{a:?} {b:?}");
        let deepest = p.stations.iter().map(|st| (st.h - st.ground).abs()).fold(0.0, f64::max);
        assert!((deepest - g.deviation_m).abs() < 1e-9, "{deepest}");
        // A motorway's box is wider: it is out of the ground by more, and
        // still never past its budget.
        let (w, s) = world("ramp?grade=0.3&bearing=90&radius=100000", "net:straight?len=200&class=motorway");
        assert_eq!(s.num("float"), 0.0, "{s}");
        let deepest = profiles(&w)[0].stations.iter().map(|st| (st.h - st.ground).abs()).fold(0.0, f64::max);
        assert!((deepest - grade::of("motorway").deviation_m).abs() < 1e-9, "{deepest}");
    }

    #[test]
    fn a_span_is_a_chord_between_its_abutments() {
        // A valley 30 m in radius, 40 m deep; a span from −40 to 40 chords
        // across it at the level ground either side, 21 stations, one of
        // them at the origin.
        let (w, s) = world("hill?amp=-40&radius=30", "net:straight?len=200&span=0.3,0.7");
        let p = profiles(&w);
        assert_eq!(p.len(), 3);
        let span = p.iter().find(|p| p.mapped == Kind::Bridge(1)).unwrap();
        assert_eq!(span.stations.len(), 21);
        let (h0, h1) = (span.stations[0].h, span.stations[20].h);
        assert!((h0 - 400.0).abs() < 1e-9 && (h1 - 400.0).abs() < 1e-9, "{h0} {h1}");
        let len = span.stations[20].s;
        for st in &span.stations {
            let chord = h0 + (h1 - h0) * st.s / len;
            assert!((st.h - chord).abs() < 1e-9, "{st:?} vs {chord}");
        }
        let t = w.terrain.as_ref().unwrap();
        let mid = span.stations[10];
        assert_eq!(mid.p, [0.0, 0.0]);
        let clearance = mid.h - height_at(t, 0.0, 0.0);
        assert!((clearance - (400.0 - height_at(t, 0.0, 0.0))).abs() < 1e-9 && clearance > 35.0, "{clearance}");
        // Deck wherever the valley is under it by the threshold: out to
        // 24 m for sure (3.8 m deep there), never at the ends.
        for st in &span.stations {
            if st.p[0].abs() <= 24.0 {
                assert_eq!(st.solved, Solved::Deck, "{st:?}");
            }
        }
        assert_eq!(span.stations[0].solved, Solved::Grade);
        assert_eq!(span.stations[20].solved, Solved::Grade);
        let decks = span.stations.iter().filter(|st| st.solved == Solved::Deck).count();
        assert!(decks >= 13, "{decks}");
        assert_eq!(s.num("decks"), decks as f64, "{s}");
        // The abutments are the ground pieces' ends: one height, no step.
        assert_eq!(s.num("step"), 0.0, "{s}");
        assert_eq!(s.num("degraded"), 0.0, "{s}");
        assert_eq!(s.num("unanchored"), 0.0, "{s}");
        assert_eq!(s.num("dangling"), 0.0, "{s}");
        // A tunnel through the hill is the mirror.
        let (w, s) = world("hill?amp=40&radius=30", "net:straight?len=200&span=0.3,0.7&kind=tunnel");
        let span = profiles(&w).iter().find(|p| p.mapped == Kind::Tunnel(-1)).unwrap();
        for st in &span.stations {
            if st.p[0].abs() <= 24.0 {
                assert_eq!(st.solved, Solved::Bore, "{st:?}");
            }
        }
        assert_eq!(s.num("bores"), decks as f64, "{s}");
    }

    #[test]
    fn a_span_that_never_leaves_the_ground_degrades() {
        let (w, s) = world("flat", "net:straight?len=200&span=0.35,0.65");
        let span = profiles(&w).iter().find(|p| p.mapped == Kind::Bridge(1)).unwrap();
        assert!(span.stations.iter().all(|st| st.solved == Solved::Grade && st.h == 400.0));
        assert_eq!(s.get("degraded"), Some("1/1 (100.00%)"), "{s}");
        assert_eq!(s.get("decks"), Some("0/16 (0.00%)"), "{s}");
    }

    #[test]
    fn a_junction_has_one_height() {
        let (w, s) = world("hill?amp=60&radius=400", "net:cross?len=400");
        let t = w.terrain.as_ref().unwrap();
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
        // Two bridge pieces of 30 m and 10 m between anchors at 400 and
        // 420: the connector between them lies on the one chord, at 415.
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem("ramp?grade=0.5&bearing=90&radius=100000"), 5.0, usize::MAX);
        let line = |id: &str, kind: Kind, pts: Vec<[f64; 2]>| Polyline2 {
            id: id.into(),
            class: "primary".into(),
            subclass: String::new(),
            width_m: 7.0,
            kind,
            pts,
        };
        let pieces = [
            line("w", Kind::Ground, vec![[-20.0, 0.0], [0.0, 0.0]]),
            line("a", Kind::Bridge(1), vec![[0.0, 0.0], [30.0, 0.0]]),
            line("b", Kind::Bridge(1), vec![[30.0, 0.0], [40.0, 0.0]]),
            line("e", Kind::Ground, vec![[40.0, 0.0], [60.0, 0.0]]),
        ];
        let refs: Vec<&Polyline2> = pieces.iter().collect();
        let (profiles, loose) = solve(w.terrain.as_ref().unwrap(), &refs);
        assert_eq!(loose, Loose::default());
        let a = &profiles[1];
        let b = &profiles[2];
        let joint = a.stations[a.stations.len() - 1].h;
        assert_eq!(joint, b.stations[0].h);
        assert!((joint - 415.0).abs() < 1e-6, "{joint}");
        assert!((a.stations[0].h - 400.0).abs() < 1e-6);
        assert!((b.stations[b.stations.len() - 1].h - 420.0).abs() < 1e-6);
        // The ground at 30 m is 415 too, so the chord runs at grade there,
        // and the deck stands off the ground only where the ramp is not
        // the chord — which on a plane it is everywhere.
        assert!(a.stations.iter().all(|st| st.solved == Solved::Grade));
    }

    #[test]
    fn an_unreached_span_lies_flat_and_is_counted() {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem("hill?amp=-40&radius=30"), 5.0, usize::MAX);
        let piece = Polyline2 {
            id: "lone".into(),
            class: "primary".into(),
            subclass: String::new(),
            width_m: 7.0,
            kind: Kind::Bridge(1),
            pts: vec![[-50.0, 0.0], [50.0, 0.0]],
        };
        let (profiles, loose) = solve(w.terrain.as_ref().unwrap(), &[&piece]);
        assert_eq!(loose, Loose { dangling: 1, unanchored: 1 });
        let top = profiles[0].stations.iter().map(|st| st.ground).fold(f64::NEG_INFINITY, f64::max);
        assert!(profiles[0].stations.iter().all(|st| st.h == top));
        let mut tunnel = piece.clone();
        tunnel.kind = Kind::Tunnel(-1);
        let (profiles, _) = solve(w.terrain.as_ref().unwrap(), &[&tunnel]);
        let floor = profiles[0].stations.iter().map(|st| st.ground).fold(f64::INFINITY, f64::min);
        assert!(profiles[0].stations.iter().all(|st| st.h == floor));
    }

    #[test]
    fn a_span_cut_by_the_clip_runs_level_to_its_anchor() {
        // A bridge leaving the world across a valley's far side: one
        // anchor, the other end dangling, so the deck holds the anchor's
        // height and is counted.
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem("ramp?grade=0.1&bearing=90&radius=100000"), 5.0, usize::MAX);
        let line = |id: &str, kind: Kind, pts: Vec<[f64; 2]>| Polyline2 {
            id: id.into(),
            class: "primary".into(),
            subclass: String::new(),
            width_m: 7.0,
            kind,
            pts,
        };
        let pieces = [
            line("w", Kind::Ground, vec![[-20.0, 0.0], [0.0, 0.0]]),
            line("a", Kind::Bridge(1), vec![[0.0, 0.0], [60.0, 0.0]]),
        ];
        let refs: Vec<&Polyline2> = pieces.iter().collect();
        let (profiles, loose) = solve(w.terrain.as_ref().unwrap(), &refs);
        assert_eq!(loose, Loose { dangling: 1, unanchored: 0 });
        let a = &profiles[1];
        assert!(a.stations.iter().all(|st| (st.h - 400.0).abs() < 1e-6), "{:?}", a.stations[5]);
        // Level over rising ground: a bore, by consequence, past 5 m out.
        assert_eq!(a.stations[a.stations.len() - 1].solved, Solved::Bore);
    }

    #[test]
    fn walks_and_indoor_pieces_get_no_profile() {
        let (w, _) = world("flat", "net:sidewalk?d=6");
        assert_eq!(profiles(&w).len(), 1);
        assert_eq!(profiles(&w)[0].id, "road");
    }
}
