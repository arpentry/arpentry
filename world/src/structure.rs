//! Step 12: the structures — decks and bores, the consequence of the profile.
//!
//! The profile solved a height along every carriageway axis and said, at
//! every station, whether that height stands off the ground ([`Solved::Deck`]),
//! runs under it ([`Solved::Bore`]) or lies on it. This step builds what
//! those answers imply and nothing else: **a structure is never built from
//! an annotation** (docs/GENERATION.md §4.5). A mapped bridge whose chord
//! never left the ground got no deck in step 9 and gets no solid here.
//!
//! **The roadway comes first.** The surface steps read the ground pieces
//! only, so until now a way's bridge and tunnel spans carried no paving at
//! all — 66 000 m² of the loop box's carriageway, and the network split
//! into twice as many regions where a deck used to join it. Every span
//! piece is now swept at its solved height across its own width, so the
//! road is continuous over the Viaduc de Chillon and through the Glion
//! bores whether or not either turned out to be a structure.
//!
//! **The solid is only what is underneath.** Over a deck run the step adds
//! a soffit [`DECK_THICKNESS_M`] below the roadway, two sides and the two
//! end faces; over a bore run a ceiling [`TUNNEL_HEIGHT_M`] above it and
//! two walls, open at the portals. The roadway is the deck's top and the
//! bore's floor, once, so no two surfaces of this step are coplanar.
//!
//! **The abutment is continuous by construction.** A span's end height is
//! the anchor the profile step took from the ground at the connector, and
//! the ground pieces meeting there took the same one, so the deck lands on
//! the bench exactly. `abutment` measures it anyway.
//!
//! **A pedestrian span is fitted, not solved** (§4.2). Footways, paths and
//! steps never solve a profile, and most of the box's spans are theirs: 31
//! footways below the ground and 29 above it, against 14 motorway bridges.
//! Each is given a chord between the ground at its own two ends — no
//! ceiling, no deviation box — and then reads as a deck or a bore by the
//! same rule as everything else. *A path cannot descend a cliff* is named
//! in the plan and waits for a site.
//!
//! **Unless the road is already carrying it.** A separated sidewalk over a
//! road bridge is mapped as its own bridge — 22.7 % of the extract's
//! footbridges are drawn that way — and it is not a second structure but
//! the same one. A walk span every station of which lies within the room's
//! reach of a road's deck is *carried*: its height is that deck's plus the
//! kerb's rise, exactly as a pavement's is over the ground, and it builds
//! no solid of its own. `carried` counts them.
//!
//! **What this step does not do.** No abutment block, no piers, no portal
//! face cut into the terrain's rim: a deck ends in the air at its soffit
//! and a bore's tube ends at its portal. The ground under a deck is
//! untouched, which is right; the ground *at* a portal is not opened,
//! which is not, and `cover` counts the stations where a bore's roof
//! stands above the ground — a cutting the terrain does not yet have.

use std::collections::HashMap;

use crate::bench::{Field, KERB_RISE_M, ROOM_REACH_M};
use crate::grade::{NODE_M, STRUCTURE_MIN_M};
use crate::poly::{self, Pt, Shapes};
use crate::profile::densify;
use crate::step::Summary;
use crate::terrain::height_at;
use crate::width::{self, Family};
use crate::world::{connector, Kind, Solved, Station, Structure, Terrain, Tri, World};

/// How thick a road deck is, in metres
/// (`data/plans/surface-leaves-the-plane-2026-09-08.md` §5): the slab, its
/// beams and its bearings, as one number.
pub const DECK_THICKNESS_M: f64 = 1.5;

/// How thick a footbridge is. The plan carries one deck thickness, and it
/// is a road bridge's: 1.5 m of beam under a 3 m footway is not a
/// footbridge but a wall with a path on it, and most of the box's spans
/// are pedestrian. A prior of this step's own, named here.
pub const WALK_DECK_M: f64 = 0.4;

/// How high a bore is inside, in metres, from the roadway to the crown.
pub const TUNNEL_HEIGHT_M: f64 = 5.0;

/// One span, with a height at every station: a carriageway piece as the
/// profile solved it, or a pedestrian piece as this step fitted it.
struct Span {
    class: String,
    width_m: f64,
    mapped: Kind,
    stations: Vec<Station>,
    /// Fitted here rather than solved in step 9: a draped class.
    fitted: bool,
    /// Carried on a road's own deck: paved, but no structure of its own.
    carried: bool,
}

impl Span {
    fn thickness(&self) -> f64 {
        if width::family(&self.class) == Family::Walk {
            WALK_DECK_M
        } else {
            DECK_THICKNESS_M
        }
    }
}

/// What the structures came to.
#[derive(Debug, Default, Clone, Copy)]
pub struct Stats {
    pub spans: usize,
    pub fitted: usize,
    /// Walk spans a road's own deck already carries.
    pub carried: usize,
    pub decks: usize,
    pub bores: usize,
    /// Roadway area, in square metres, that no surface step laid.
    pub roadway_m2: f64,
    /// The least a deck's soffit clears the ground under it, between its
    /// abutments, and how many stations in there it does not clear.
    pub clear: f64,
    pub buried: usize,
    /// Runs whose face never left the ground anywhere: a deck landing all
    /// the way along, or a bore that never got under.
    pub grounded: usize,
    /// The least a bore's crown lies under the ground over it, away from
    /// its portals, and how many stations it stands above it: there the
    /// bore has degraded to a cutting the terrain does not have.
    pub cover: f64,
    pub open: usize,
    /// The largest disagreement, in metres, between a span's end height
    /// and the ground piece it lands on.
    pub abutment: f64,
}

/// Builds every structure the profile implies.
pub fn run(world: &mut World) -> Summary {
    let (structure, stats) = {
        let terrain = world.terrain.as_ref().expect("the terrain step runs first");
        let profiles = world.profile.as_ref().expect("the profile step runs first");
        let roads = world.roads.as_ref().expect("the drape step runs first");

        let mut spans: Vec<Span> = profiles
            .profiles
            .iter()
            .filter(|p| p.mapped.is_structure())
            .map(|p| Span {
                class: p.class.clone(),
                width_m: p.width_m,
                mapped: p.mapped,
                stations: p.stations.clone(),
                fitted: false,
                carried: false,
            })
            .collect();
        let decks = Field::of(profiles.profiles.iter().filter(|p| p.mapped.is_structure()));
        spans.extend(
            roads
                .spans
                .iter()
                .filter(|w| width::family(&w.class) == Family::Walk && w.kind != Kind::Indoor)
                .map(|w| fit(w, terrain, &decks)),
        );

        // Every connector a ground piece ends at, and the height it took
        // there: what a span's own end must equal.
        let mut ends: HashMap<(i64, i64), f64> = HashMap::new();
        for p in profiles.profiles.iter().filter(|p| p.mapped == Kind::Ground) {
            for st in [p.stations.first(), p.stations.last()].into_iter().flatten() {
                ends.entry(connector(st.p)).or_insert(st.h);
            }
        }

        let mut s = Structure::default();
        let mut stats = Stats { spans: spans.len(), clear: f64::INFINITY, cover: f64::INFINITY, ..Stats::default() };
        for span in &spans {
            stats.fitted += span.fitted as usize;
            stats.carried += span.carried as usize;
            if span.stations.len() < 2 {
                continue;
            }
            let (l, r) = edges(&span.stations, span.width_m / 2.0);
            strip(&mut s.roadway, &l, &r);
            stats.roadway_m2 += poly::length(&span.stations.iter().map(|st| st.p).collect::<Vec<Pt>>()) * span.width_m;
            for st in [&span.stations[0], &span.stations[span.stations.len() - 1]] {
                if let Some(h) = ends.get(&connector(st.p)) {
                    stats.abutment = stats.abutment.max((st.h - h).abs());
                }
            }
            if span.carried {
                continue;
            }
            let t = span.thickness();
            for (a, b) in runs(&span.stations, Solved::Deck) {
                stats.decks += 1;
                box_under(&mut s.deck, &l[a..=b], &r[a..=b], -t);
                let face: Vec<f64> = span.stations[a..=b].iter().map(|st| st.h - t - st.ground).collect();
                match spanning(&face) {
                    None => stats.grounded += 1,
                    Some((f, g)) => {
                        stats.clear = stats.clear.min(face[f..=g].iter().copied().fold(f64::INFINITY, f64::min));
                        stats.buried += face[f..=g].iter().filter(|c| **c < 0.0).count();
                    }
                }
            }
            for (a, b) in runs(&span.stations, Solved::Bore) {
                stats.bores += 1;
                tube_over(&mut s.bore, &l[a..=b], &r[a..=b], TUNNEL_HEIGHT_M);
                let face: Vec<f64> = span.stations[a..=b].iter().map(|st| st.ground - st.h - TUNNEL_HEIGHT_M).collect();
                match spanning(&face) {
                    None => stats.grounded += 1,
                    Some((f, g)) => {
                        stats.cover = stats.cover.min(face[f..=g].iter().copied().fold(f64::INFINITY, f64::min));
                        stats.open += face[f..=g].iter().filter(|c| **c < 0.0).count();
                    }
                }
            }
        }
        s.plan = spans.iter().filter(|s| s.stations.len() > 1).map(|s| plan(s)).collect();
        (s, stats)
    };
    let finite = |v: f64| if v.is_finite() { format!("{v:.2}") } else { "-".into() };
    let summary = Summary::new()
        .with("spans", stats.spans)
        .with("fitted", stats.fitted)
        .with("carried", stats.carried)
        .with("decks", stats.decks)
        .with("bores", stats.bores)
        .with_m2("roadway_m2", stats.roadway_m2)
        .with("triangles", structure.roadway.indices.len() / 3 + structure.deck.indices.len() / 3 + structure.bore.indices.len() / 3)
        .with("clear", finite(stats.clear))
        .with("buried", stats.buried)
        .with("grounded", stats.grounded)
        .with("cover", finite(stats.cover))
        .with("open", stats.open)
        .with("abutment", format!("{:.3}", stats.abutment));
    world.structure = Some(structure);
    summary
}

/// A pedestrian span, fitted: a chord between the ground at its two ends,
/// or the deck of the road that is already carrying it.
///
/// No ceiling and no box — a draped class holds nothing (stratum D) — so
/// the chord is the whole of the construction, and what it turns out to be
/// is read from the ground the same way a solved piece's is.
fn fit(w: &crate::world::Polyline2, terrain: &Terrain, decks: &Field) -> Span {
    let pts = densify(&w.pts, NODE_M);
    let mut stations: Vec<Station> = Vec::with_capacity(pts.len());
    let mut s = 0.0;
    for (i, p) in pts.iter().enumerate() {
        if i > 0 {
            s += (p[0] - pts[i - 1][0]).hypot(p[1] - pts[i - 1][1]);
        }
        let ground = height_at(terrain, p[0], p[1]);
        stations.push(Station { s, p: *p, ground, h: ground, solved: Solved::Grade });
    }
    // Carried, if a road's own span runs within the room's reach of every
    // one of its stations: the same structure, drawn twice by the source.
    // The test is proximity and nothing else — asking also that the road
    // stand off the ground there would fail at the abutments, where it is
    // at grade by definition, and no span would ever be carried.
    let carried =
        stations.iter().all(|st| decks.at(st.p).is_some_and(|foot| foot.d <= foot.half_w + ROOM_REACH_M));
    if carried {
        // It rides the road's cross-section: the road's own height plus
        // the kerb's rise, wherever it stands, exactly as a pavement does
        // over the ground. What it is — deck, bore or neither — is the
        // road's answer and not a second one.
        for st in stations.iter_mut() {
            let road = decks.at(st.p).expect("every station has a road's span beside it");
            st.h = road.h + KERB_RISE_M;
            st.solved = Solved::Grade;
        }
        return Span { class: w.class.clone(), width_m: w.width_m, mapped: w.kind, stations, fitted: true, carried };
    }
    let len = stations.last().map(|st| st.s).unwrap_or(0.0);
    let (h0, h1) = (stations[0].ground, stations[stations.len() - 1].ground);
    for st in stations.iter_mut() {
        st.h = if len > 0.0 { h0 + (h1 - h0) * st.s / len } else { h0 };
        st.solved = if st.h - st.ground >= STRUCTURE_MIN_M {
            Solved::Deck
        } else if st.ground - st.h >= STRUCTURE_MIN_M {
            Solved::Bore
        } else {
            Solved::Grade
        };
    }
    Span { class: w.class.clone(), width_m: w.width_m, mapped: w.kind, stations, fitted: true, carried }
}

/// The stretch of a run over which the structure has actually left the
/// ground: from the first station whose face clears to the last. Outside
/// it a deck is landing and a bore is surfacing, and that is what an
/// abutment and a portal *are* — a deck 1.5 m thick cannot have its
/// soffit out of the ground until its roadway is 1.5 m over it, so the
/// stations either side of the stretch carry no check. `None` if the face
/// never cleared at all.
fn spanning(face: &[f64]) -> Option<(usize, usize)> {
    let f = face.iter().position(|c| *c >= 0.0)?;
    let g = face.iter().rposition(|c| *c >= 0.0)?;
    (g > f).then_some((f, g))
}

/// The runs of consecutive stations solved `kind`, as inclusive index
/// pairs. A run of one station is no run: a structure needs a length.
fn runs(stations: &[Station], kind: Solved) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut start: Option<usize> = None;
    for (i, st) in stations.iter().enumerate() {
        match (st.solved == kind, start) {
            (true, None) => start = Some(i),
            (false, Some(a)) => {
                if i - 1 > a {
                    out.push((a, i - 1));
                }
                start = None;
            }
            _ => {}
        }
    }
    if let Some(a) = start {
        if stations.len() - 1 > a {
            out.push((a, stations.len() - 1));
        }
    }
    out
}

/// The left and right edge of a piece `half_w` from its axis, at the
/// solved height of every station. The direction at a station is the
/// centred difference of its neighbours, so a bend is mitred by the mean
/// of the two headings rather than stepped.
fn edges(st: &[Station], half_w: f64) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let n = st.len();
    let (mut l, mut r) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for i in 0..n {
        let (a, b) = (st[i.saturating_sub(1)].p, st[(i + 1).min(n - 1)].p);
        let mut t = poly::unit([b[0] - a[0], b[1] - a[1]]);
        if t == [0.0, 0.0] {
            t = [1.0, 0.0];
        }
        let (p, h) = (st[i].p, st[i].h);
        l.push([p[0] - t[1] * half_w, p[1] + t[0] * half_w, h]);
        r.push([p[0] + t[1] * half_w, p[1] - t[0] * half_w, h]);
    }
    (l, r)
}

/// Quads between two polylines of equal length, wound counter-clockwise
/// seen from the side `a` is left of: `strip(left, right)` faces up.
fn strip(tri: &mut Tri, a: &[[f64; 3]], b: &[[f64; 3]]) {
    let base = tri.positions.len() as u32;
    for i in 0..a.len() {
        tri.positions.push(a[i]);
        tri.positions.push(b[i]);
    }
    for i in 0..a.len().saturating_sub(1) {
        let (a0, b0, a1, b1) = (base + 2 * i as u32, base + 2 * i as u32 + 1, base + 2 * i as u32 + 2, base + 2 * i as u32 + 3);
        tri.indices.extend_from_slice(&[a0, b0, b1, a0, b1, a1]);
    }
}

/// `l`/`r` dropped by `dz` (negative) and closed: the soffit, the two
/// sides and the two end faces of a deck. The roadway above it was drawn
/// once already and is not drawn again.
fn box_under(tri: &mut Tri, l: &[[f64; 3]], r: &[[f64; 3]], dz: f64) {
    let lower = |v: &[[f64; 3]]| -> Vec<[f64; 3]> { v.iter().map(|p| [p[0], p[1], p[2] + dz]).collect() };
    let (lo_l, lo_r) = (lower(l), lower(r));
    strip(tri, &lo_r, &lo_l);
    strip(tri, &lo_l, l);
    strip(tri, r, &lo_r);
    let n = l.len() - 1;
    quad(tri, [l[0], r[0], lo_r[0], lo_l[0]]);
    quad(tri, [r[n], l[n], lo_l[n], lo_r[n]]);
}

/// `l`/`r` raised by `dz` and walled: the crown and the two walls of a
/// bore, seen from inside. The portals stay open — the terrain has no
/// face cut into it for them yet.
fn tube_over(tri: &mut Tri, l: &[[f64; 3]], r: &[[f64; 3]], dz: f64) {
    let raise = |v: &[[f64; 3]]| -> Vec<[f64; 3]> { v.iter().map(|p| [p[0], p[1], p[2] + dz]).collect() };
    let (up_l, up_r) = (raise(l), raise(r));
    strip(tri, &up_r, &up_l);
    strip(tri, l, &up_l);
    strip(tri, &up_r, r);
}

/// One quad, as two triangles, in the order given.
fn quad(tri: &mut Tri, q: [[f64; 3]; 4]) {
    let base = tri.positions.len() as u32;
    tri.positions.extend_from_slice(&q);
    tri.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
}

/// A span's outline in plan, for the plan view: its two edges, closed.
fn plan(span: &Span) -> (Kind, Shapes) {
    let (l, r) = edges(&span.stations, span.width_m / 2.0);
    let mut ring: Vec<Pt> = l.iter().map(|p| [p[0], p[1]]).collect();
    ring.extend(r.iter().rev().map(|p| [p[0], p[1]]));
    (span.mapped, poly::ccw(ring).map(|ring| vec![vec![ring]]).unwrap_or_default())
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use crate::terrain::{self, tests::dem};
    use crate::{bench, crossing, drape, facade, fillet, kerb, mesh, profile, ribbon, room, surface};

    use super::*;

    /// A world on `ground` with the network of `net`, built to the end.
    fn world(ground: &str, net: &str) -> (World, Summary) {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem(ground), 5.0, usize::MAX);
        drape::run(&mut w, Path::new(net)).unwrap();
        profile::run(&mut w);
        crossing::run(&mut w);
        facade::run(&mut w, None).unwrap();
        ribbon::run(&mut w);
        surface::run(&mut w);
        kerb::run(&mut w);
        fillet::run(&mut w);
        room::run(&mut w);
        mesh::run(&mut w);
        bench::run(&mut w);
        let s = run(&mut w);
        (w, s)
    }

    fn built(w: &World) -> &Structure {
        w.structure.as_ref().unwrap()
    }

    #[test]
    fn runs_need_a_length() {
        let st = |solved: Solved| Station { s: 0.0, p: [0.0, 0.0], ground: 0.0, h: 0.0, solved };
        let row = [Solved::Grade, Solved::Deck, Solved::Deck, Solved::Grade, Solved::Deck, Solved::Grade, Solved::Deck, Solved::Deck];
        let stations: Vec<Station> = row.iter().map(|s| st(*s)).collect();
        // The lone deck station at 4 is no run; the pair at 1..2 and the
        // pair the row ends on are.
        assert_eq!(runs(&stations, Solved::Deck), vec![(1, 2), (6, 7)]);
        assert!(runs(&stations, Solved::Bore).is_empty());
    }

    #[test]
    fn a_valley_gets_a_viaduct() {
        // The plan's specimen: a 60 m valley with a mapped bridge over the
        // middle of it. The chord runs between the ground at the two
        // abutments, so it flies; the deck's soffit is 1.5 m under the
        // roadway and clears the ground everywhere between them.
        let (w, s) = world("hill?amp=-60&radius=300", "net:straight?len=400&span=0.35,0.65");
        assert_eq!(s.num("decks"), 1.0, "{s}");
        assert_eq!(s.num("bores"), 0.0, "{s}");
        assert_eq!(s.num("buried"), 0.0, "the soffit is in the ground between the abutments: {s}");
        assert_eq!(s.num("grounded"), 0.0, "{s}");
        // The soffit clears all the way; least where it is landing, most
        // over the valley floor, which is 60 m under the rim the chord
        // runs between.
        assert!(s.num("clear") > 0.0, "{s}");
        let t = w.terrain.as_ref().unwrap();
        let mid = crate::terrain::height_at(t, 0.0, 0.0);
        assert_eq!(s.num("abutment"), 0.0, "the deck does not land where the road is: {s}");
        let b = built(&w);
        assert!(!b.roadway.indices.is_empty() && !b.deck.indices.is_empty() && b.bore.indices.is_empty());
        let deck_z = b.roadway.positions.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        assert!(deck_z - DECK_THICKNESS_M - mid > 4.0, "over the floor it clears {}", deck_z - mid);
        // The roadway is the chord: level, since the two abutments stand
        // at the same height on a radial hill.
        let z: Vec<f64> = b.roadway.positions.iter().map(|p| p[2]).collect();
        let (lo, hi) = (z.iter().cloned().fold(f64::INFINITY, f64::min), z.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
        assert!((hi - lo).abs() < 1e-9, "{lo} {hi}");
        // And it is one road's width across.
        let y: Vec<f64> = b.roadway.positions.iter().map(|p| p[1]).collect();
        let w_m = y.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - y.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((w_m - 5.5).abs() < 1e-9, "{w_m}");
        // The deck's soffit is exactly a deck's thickness under it.
        let deck_z: Vec<f64> = b.deck.positions.iter().map(|p| p[2]).collect();
        let bottom = deck_z.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((hi - bottom - DECK_THICKNESS_M).abs() < 1e-9, "{hi} {bottom}");
    }

    #[test]
    fn a_hill_gets_a_bore() {
        // The mirror: a hill with a mapped tunnel through it. The chord
        // runs under the ground, the crown 5 m over the roadway, and the
        // ground covers it everywhere between the portals.
        let (w, s) = world("hill?amp=60&radius=300", "net:straight?len=400&span=0.1,0.9&kind=tunnel");
        assert_eq!(s.num("bores"), 1.0, "{s}");
        assert_eq!(s.num("decks"), 0.0, "{s}");
        assert_eq!(s.num("open"), 0.0, "the crown breaks surface between the portals: {s}");
        assert_eq!(s.num("grounded"), 0.0, "{s}");
        assert!(s.num("cover") > 0.0, "{s}");
        assert_eq!(s.num("abutment"), 0.0, "{s}");
        let b = built(&w);
        assert!(!b.bore.indices.is_empty() && b.deck.indices.is_empty());
        let road = b.roadway.positions.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        let crown = b.bore.positions.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        assert!((crown - road - TUNNEL_HEIGHT_M).abs() < 1e-9, "{crown} {road}");
        // The portals stand 160 m out on the flank, where the hill is
        // 26.9 m up; the crest is 60 m up. So the chord runs level under
        // 33 m of hill and the crown is some 28 m under the crest.
        let t = w.terrain.as_ref().unwrap();
        let cover = crate::terrain::height_at(t, 0.0, 0.0) - crown;
        assert!((cover - 28.1).abs() < 0.2, "{cover}");
    }

    #[test]
    fn a_span_that_never_left_the_ground_is_no_structure() {
        // A mapped bridge over flat ground: the chord is the ground, so
        // step 9 solved every station at grade and this step builds no
        // solid at all — plain, not wrong (invariant 6). The roadway is
        // still laid, because the road is still a road.
        let (w, s) = world("flat", "net:straight?len=400&span=0.35,0.65");
        assert_eq!(s.num("decks"), 0.0, "{s}");
        assert_eq!(s.num("bores"), 0.0, "{s}");
        assert!((s.num("roadway_m2") - 660.0).abs() < 1.0, "{s}");
        let b = built(&w);
        assert!(b.deck.indices.is_empty() && b.bore.indices.is_empty());
        assert!(b.roadway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9));
    }

    #[test]
    fn a_sidewalk_over_a_bridge_is_not_a_second_bridge() {
        // The plan's specimen, and 22.7 % of the extract's footbridges: a
        // road bridge whose separated sidewalk is mapped as its own
        // bridge. It is one structure. The walk is *carried* — its height
        // is the road's plus the kerb's rise — and it builds no deck, so
        // the count of decks is the road's one.
        let (w, s) = world("hill?amp=-60&radius=300", "net:sidewalk?d=6&span=0.35,0.65");
        assert_eq!(s.num("carried"), 1.0, "{s}");
        assert_eq!(s.num("decks"), 1.0, "the walk built a second bridge: {s}");
        let b = built(&w);
        // Both are paved, and the walk's paving stands one kerb over the
        // road's, all the way along.
        let of = |lo: f64, hi: f64| -> Vec<f64> {
            let mut z: Vec<f64> =
                b.roadway.positions.iter().filter(|p| p[1] > lo && p[1] < hi).map(|p| p[2]).collect();
            z.sort_by(|a, b| a.partial_cmp(b).unwrap());
            z.dedup();
            z
        };
        let (road, walk) = (of(-3.0, 3.0), of(4.0, 8.0));
        assert!(!road.is_empty() && !walk.is_empty(), "both are paved");
        assert!((walk[0] - road[0] - KERB_RISE_M).abs() < 1e-9, "{:?} vs {:?}", &walk[..1], &road[..1]);
    }

    #[test]
    fn a_footbridge_of_its_own_is_fitted() {
        // A footway whose span runs nowhere near a road carries no profile
        // — a draped class holds nothing — so this step fits it a chord
        // between the ground at its own two ends, and it reads as a deck
        // by the same rule as a road. Its deck is a footbridge's, not a
        // road bridge's.
        let (w, s) = world("hill?amp=-60&radius=300", "net:sidewalk?d=40&span=0.35,0.65");
        assert_eq!(s.num("carried"), 0.0, "{s}");
        assert_eq!(s.num("decks"), 2.0, "the road and the footbridge both fly: {s}");
        let b = built(&w);
        // Two thicknesses in one mesh: 1.5 m under the road, 0.4 m under
        // the walk. Each chord tilts by a millimetre over its 120 m,
        // because the lattice is not symmetric about the origin and the
        // ground it reads at the two abutments is not exactly equal, so
        // the depth of a deck is read to the centimetre and not the ulp.
        let under = |lo: f64, hi: f64| -> f64 {
            let z: Vec<f64> = b.deck.positions.iter().filter(|p| p[1] > lo && p[1] < hi).map(|p| p[2]).collect();
            z.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - z.iter().cloned().fold(f64::INFINITY, f64::min)
        };
        assert!((under(-3.0, 3.0) - DECK_THICKNESS_M).abs() < 0.01, "{}", under(-3.0, 3.0));
        assert!((under(38.0, 42.0) - WALK_DECK_M).abs() < 0.01, "{}", under(38.0, 42.0));
    }

    #[test]
    fn an_overpass_gets_its_embankment_and_its_deck_from_the_steps_it_already_had() {
        // On flat ground the mapped span degrades in step 9 — a chord at
        // grade is not a bridge — and the crossing step's floor is what
        // makes it one. Neither the bench nor this step learns a rule:
        // the fill is the embankment the approach now stands on, and the
        // 5 m the soffit clears is exactly the headroom that was asked
        // for, the 1.5 m slab having been the rest of the demand.
        let (w, s) = world("flat", "net:overpass?len=300");
        assert_eq!(s.num("decks"), 1.0, "{s}");
        assert_eq!(s.num("bores"), 0.0, "{s}");
        assert_eq!(s.num("grounded"), 0.0, "{s}");
        assert_eq!(s.num("buried"), 0.0, "{s}");
        assert!((s.num("clear") - crate::crossing::ROAD_CLEARANCE_M).abs() < 1e-6, "{s}");
        // The approach is the bench's: its asphalt rides the fill all the
        // way up to the abutment, and the deck's top is the same surface
        // (the bench and this step read one profile).
        let b = w.bench.as_ref().unwrap();
        let top = b.carriageway.positions.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        assert!((top - 406.5).abs() < 1e-6, "{top}");
        // The mirror: the leg is cut into the ground and the bore's crown
        // carries the road above it on the slab's thickness.
        let (_, s) = world("flat", "net:underpass?len=300");
        assert_eq!(s.num("bores"), 1.0, "{s}");
        assert_eq!(s.num("decks"), 0.0, "{s}");
        assert_eq!(s.num("open"), 0.0, "{s}");
        assert!((s.num("cover") - DECK_THICKNESS_M).abs() < 1e-6, "{s}");
    }

    #[test]
    fn the_structures_are_a_function_of_the_world() {
        let (a, _) = world("hill?amp=-60&radius=300", "net:straight?len=400&span=0.35,0.65");
        let (b, _) = world("hill?amp=-60&radius=300", "net:straight?len=400&span=0.35,0.65");
        assert_eq!(built(&a).roadway.positions, built(&b).roadway.positions);
        assert_eq!(built(&a).deck.indices, built(&b).deck.indices);
    }
}
