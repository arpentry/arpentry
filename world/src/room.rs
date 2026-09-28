//! The room: the street fills the space between the facades.
//!
//! The facade step said where the paved surface stops; this one says it
//! goes all the way there. A street in a town is the whole room between
//! the buildings either side of it: asphalt in the middle at its prior
//! width, and whatever is left up to the walls is pavement, whether or not
//! a mapper drew a sidewalk line down it. Before this step that remainder
//! was bare ground — a metre or two of nothing between the kerb and a
//! house front down every lane in the box, and a pocket of nothing in
//! every junction corner a building stands near.
//!
//! The rule is the one a town follows. **A sidewalk runs along the kerb
//! wherever the kerb runs along a facade.** Every metre along the
//! carriageway's boundary a probe marches outward, normal to the kerb, for
//! at most [`WALL_REACH_M`]; where it meets a wall, and the wall is a
//! *face* — its distance changes between neighbouring stations by less
//! than [`FACE_MAX_GRADE`], a wall within thirty degrees of the kerb —
//! the station is walled. Walled stations form runs, bridged across
//! breaks under [`BRIDGE_M`] (a gap between two houses, a driveway, a
//! notch) and dropped under [`RUN_MIN_M`]. Each run is paved as a band
//! [`BAND_M`] wide along the kerb, with flat ends, plus a strip from the
//! kerb to the wall over every station of the run that saw one: a facade a
//! metre off gets a metre of pavement, one three metres off gets pavement
//! to its wall, an alley mouth gets the band across it and nothing
//! inside, and a notch in the facade is paved by the strip that reaches
//! into it. A wall wins over a pavement in front of it: a sidewalk mapped
//! two metres wide with a house front four metres from the kerb is paved
//! to the house.
//!
//! A house corner that merely points at the road fails the face test at
//! every station but one and gets nothing. That is the case the first
//! construction got wrong: a morphological closing of asphalt and walls
//! together filled every pinch a 2 m disc could not enter, which is the
//! strip along a facade and the pocket at a junction corner, but also a
//! lens of pavement bounded by arcs at every lone corner within reach —
//! shapes nobody would pave, and the plan view was full of them. A
//! closing does not know which wall it is closing against; the probe does.
//!
//! **A mapped sidewalk is a wall to the kerb as well.** The kerb step
//! attaches a sidewalk to its street and fills the strip between them,
//! but its ladder fails where the sidewalk turns a corner at a distance
//! from the kerb return, and a pocket is left between the two. So the
//! probe here also stops at the pavement built so far: a station whose
//! probe meets a sidewalk's near edge is walled by it, under the same
//! face and run rules, and gets its strip — never the band, because the
//! mapped line says how wide that pavement is. A break in the sidewalk is
//! bridged at the sidewalk's width: a station in a run that saw no face
//! reaches the point interpolated between the far edges of the faces
//! either side of it, and where a pavement runs on past the reach the
//! nearer edge of the other side says how far.
//!
//! **The strip is one polygon per stretch, not a rung per station.** The
//! first drawing was a 1.2 m rectangle per station, and along any reach
//! that is not a wall — a pavement met obliquely, a bridge between two
//! depths — the rectangles' flat ends stood out of the slanted edge
//! between them as a row of teeth a metre apart. The quads between
//! consecutive stations, closed half a station past either end, have the
//! edge the reach points draw and nothing else.
//!
//! **A pocket is paved.** A hole in what is built — the asphalt, the
//! pavement and the buildings together — under [`ISLAND_M2`], bordering
//! asphalt or pavement and lying wholly within [`POCKET_REACH_M`] of the
//! asphalt, is pavement too: a roundabout's centre, a traffic island, the
//! median between two carriageways, and the pocket of ground in a junction
//! corner between a kerb, a footway and a house front, which no probe from
//! any kerb can reach because its far side is nothing's face. Nothing in
//! the data draws these; the ground there is kerbed and paved or planted,
//! never bare, and left bare it shows the coarse polygon the ring was
//! mapped with. The street is the ground within the reach, and a hole
//! that goes farther — the lawn between two
//! footpaths, the yard between two parallel roads, the garden inside a
//! hairpin, every one of them under the area — is not the street's. A hole
//! enclosed by walls alone is a courtyard and stays what it is.
//!
//! The pavement is then the earlier pavement with the bands, strips and
//! pockets, less the carriageway and less the walls.
//!
//! **The check.** `wall_gap` on the summary line is the share of stations
//! in kept runs that face a wall or a pavement and have bare ground just
//! outside them.

use crate::kerb::{self, Bare};
use crate::poly::{self, Indexed, Pt, Shape, Shapes};
use crate::step::Summary;
use crate::world::{Facade, Room, Surface};

/// How far outside the kerb a wall still bounds the street, in metres.
/// Past this the ground in front of a house is its own. Four metres left
/// a bare strip along every block whose fronts stand a little farther
/// back; eight paved whole forecourts and grew the pavement by a third.
pub const WALL_REACH_M: f64 = 6.0;

/// The pavement's width along a built-up kerb, in metres: the norm.
pub const BAND_M: f64 = crate::width::WALK_M;

/// The most a wall's distance may change per metre of kerb for the wall
/// to be a face the kerb runs along: `tan 30°`.
pub const FACE_MAX_GRADE: f64 = 0.5774;

/// A break between two walled stretches shorter than this, in metres, is
/// bridged: a gap between two houses, a driveway, a notch.
pub const BRIDGE_M: f64 = 10.0;

/// A break between two stretches of mapped sidewalk shorter than this is
/// bridged: the kerb step's own bridge, so a sidewalk cut at a driveway or
/// mapped in two pieces reads as one pavement here as it does there.
pub const SIDEWALK_BRIDGE_M: f64 = kerb::BRIDGE_M;

/// The shortest run of walled stations that is a pavement, in metres: a
/// house front, not a corner glimpsed in passing.
pub const RUN_MIN_M: f64 = 6.0;

/// A hole in what is built smaller than this, in square metres, is an
/// island or a pocket and is paved: a disc of 12.6 m radius, which takes
/// every roundabout centre, traffic island and junction pocket in the
/// loop box and leaves the smallest block, a 630 m² car-park loop, alone.
pub const ISLAND_M2: f64 = 500.0;

/// How far from the asphalt every point of a pocket may lie for it to be
/// the street's, in metres. Measured over the loop box, holes under the
/// area sorted by how far they reach from the asphalt: within 6 m they
/// are kerb-to-house strips and junction corners, but the two pockets a
/// plan of the roundabout showed bare — a corner cut by a footway, a wedge
/// between a house, a road and a footway — have far corners 8.5 m out;
/// within 10 m the yards between the houses along two parallel streets
/// join them, which in a town this dense are paved; past 10 m the holes
/// are lawns between footpaths and blocks with no street in them, and at
/// 15 m a third of the box's pavement would have been lawn.
pub const POCKET_REACH_M: f64 = 10.0;

/// The narrowest hole worth paving, in metres: [`kerb::WALK_MIN_M`], the
/// narrowest pavement the model draws anywhere.
///
/// [`ISLAND_M2`] bounds a hole from above — past it the hole is a lawn or a
/// courtyard rather than a traffic island — and nothing bounded it from
/// below, so a boolean's leftover between two ribbons passed every test
/// (small, near a kerb, bordering asphalt) and was paved as an island. On
/// the Montreux junction that was **29 of 40 islands**, and all of them
/// together were 10 m² of the 986.
///
/// **It governs a hole and not the finished pavement.** The same test at
/// the end of the chain is wrong, and four specimens say so: the strip
/// along a house front is deliberately narrow — squeezed between the kerb
/// and a wall two metres off the axis it can be a decimetre wide — and a
/// notch in a facade is a small isolated patch by construction. Narrow,
/// short and isolated are all shapes a *real* pavement takes here, so
/// telling a scrap from a place needs to know which construction made it —
/// which `scraps` on the summary line now records, per source.
///
/// The same missing bound shows at the other end of the box. A
/// boolean between two ribbons leaves scraps, and at the Montreux
/// overbridge two of them, **1.03 m² and 2.84 m², 0.41 m and 0.71 m
/// wide**, sat between the deck's ribbon and the ballast, passed every test
/// (small, near a kerb, bordering asphalt) and were paved as traffic
/// islands. They are not islands: they are places a person could not stand,
/// and up at road level over a cutting the ground then had to wall a square
/// metre of pavement all the way round.
///
/// The test is an **erosion**, not an area: a scrap is thin, not small, and
/// a hole that does not survive being cut back by half the narrowest
/// pavement has no pavement in it to draw.
pub const PAVEMENT_MIN_M: f64 = kerb::WALK_MIN_M;

/// Whether `region` can hold a pavement at all: whether anything of it
/// survives being cut back by half [`PAVEMENT_MIN_M`].
///
/// A scrap is **thin, not small** — a traffic island of three square metres
/// is a place and a forty-metre thread of the same area is not — so the
/// test is an erosion and not an area. The cheap ratio first: `2A/P` is a
/// region's width where it is thin, and nothing twice the minimum wide by
/// that measure has ever failed the erosion, so only the suspicious ones
/// cost a boolean.
pub(crate) fn wide_enough(region: &poly::Shape) -> bool {
    let area = poly::area(std::slice::from_ref(region));
    let perimeter: f64 = region
        .iter()
        .flat_map(|ring| {
            (0..ring.len()).map(move |i| {
                let (u, v) = (ring[i], ring[(i + 1) % ring.len()]);
                (v[0] - u[0]).hypot(v[1] - u[1])
            })
        })
        .sum();
    if perimeter > 0.0 && 2.0 * area / perimeter >= 2.0 * PAVEMENT_MIN_M {
        return true;
    }
    !poly::erode(&vec![region.clone()], PAVEMENT_MIN_M / 2.0).is_empty()
}

/// How much of `region`'s boundary has a wall [`EDGE_M`] outside it.
///
/// Outside an outer (counter-clockwise) ring is to the **right** of each
/// edge. Probing the left samples the region itself, which reads "nothing
/// there" everywhere — the measurement that sent four candidate rules the
/// wrong way before it was caught.
fn walled_share(region: &poly::Shape, walls: &Indexed) -> f64 {
    let (mut against, mut total) = (0.0f64, 0.0f64);
    for ring in region {
        for i in 0..ring.len() {
            let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
            let d = (b[0] - a[0]).hypot(b[1] - a[1]);
            if d <= 0.0 {
                continue;
            }
            total += d;
            let t = [(b[0] - a[0]) / d, (b[1] - a[1]) / d];
            let m = [(a[0] + b[0]) * 0.5 + t[1] * EDGE_M, (a[1] + b[1]) * 0.5 - t[0] * EDGE_M];
            if walls.contains(m) {
                against += d;
            }
        }
    }
    if total > 0.0 { against / total } else { 0.0 }
}

/// Whether every edge of `region` has `by` [`EDGE_M`] outside it.
fn enclosed(region: &poly::Shape, by: &Indexed) -> bool {
    region.iter().all(|ring| {
        (0..ring.len()).all(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
            let t = poly::unit([b[0] - a[0], b[1] - a[1]]);
            let m = [(a[0] + b[0]) * 0.5 + t[1] * EDGE_M, (a[1] + b[1]) * 0.5 - t[0] * EDGE_M];
            by.contains(m)
        })
    })
}

/// Whether `region` reaches a footbridge: a vertex of it inside one, or an
/// edge with one [`EDGE_M`] outside it.
fn onto(region: &poly::Shape, bridges: &Indexed) -> bool {
    if bridges.is_empty() {
        return false;
    }
    region.iter().any(|ring| {
        (0..ring.len()).any(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
            let t = poly::unit([b[0] - a[0], b[1] - a[1]]);
            let m = [(a[0] + b[0]) * 0.5 + t[1] * EDGE_M, (a[1] + b[1]) * 0.5 - t[0] * EDGE_M];
            bridges.contains(a) || bridges.contains(m)
        })
    })
}

/// How much of a small region's boundary must be against a wall for it to
/// be the strip this step drew there rather than something a cut left.
///
/// A quarter, and the margin is wide rather than the number fine. A strip
/// squeezed between a kerb and a wall has the wall along one of its two
/// long sides and the asphalt along the other, so it measures near a half
/// and the specimens read **45 %**; the scraps beside the Montreux abutment
/// read **1 % and 0 %**. Anything between those is a shape neither
/// specimen nor site has yet produced.
const WALLED_SHARE: f64 = 0.25;

/// The probe's step, in metres: fine enough that a wall's position is
/// known to within the width of a kerb stone.
const MARCH_M: f64 = 0.25;

/// How far past the first and last station of a stretch its strip runs,
/// in metres: half a station, so the pavement ends where the facade does
/// to within the station spacing.
const CAP_M: f64 = 0.5 * kerb::STATION_M;

/// How far into a neighbouring surface a pocket's edge is probed to learn
/// what bounds it, in metres.
const EDGE_M: f64 = 0.05;

/// How far outside a kerb station the pavement is probed for `wall_gap`,
/// in metres: inside the narrowest strip drawn.
const PROBE_M: f64 = 0.15;

/// Paves the kerbs that run along facades, and the pockets.
pub fn run(
    paving: &Surface,
    walk_spans: &Shapes,
    attached: &[kerb::Attached],
    facade: &Facade,
) -> (Room, Summary) {
    let Surface { carriageway, walk: before, spanned, .. } = paving;
    let walls = Indexed::new(&facade.solid);
    let paved = Indexed::new(before);
    let mut hits: Vec<(Pt, Pt)> = Vec::new();
    let mut pieces: Shapes = Vec::new();
    // **Kept apart, so a region can be asked what made it.** The pavement
    // comes out of this step as one set of regions and the constructions
    // that feed it are several; a scrap of one is a defect and the same
    // shape drawn by another is a place, and nothing could tell them apart
    // while they were unioned on the way in.
    let (mut bands, mut rungs): (Shapes, Shapes) = (Vec::new(), Vec::new());
    let (mut stations, mut runs, mut walled, mut kerbed) = (0usize, 0usize, 0usize, 0usize);
    let min = (RUN_MIN_M / kerb::STATION_M) as usize;
    if !walls.is_empty() || !paved.is_empty() {
        for ring in carriageway.iter().flatten() {
            let (pts, normals) = kerb_stations(ring);
            let n = pts.len();
            stations += n;
            // A wall within reach, whatever stands in front of it; else
            // the pavement ahead, with its far edge, which is what a
            // bridge across a break in it wants.
            let hit: Vec<Option<Hit>> = pts
                .iter()
                .zip(&normals)
                .map(|(&p, &v)| {
                    march(&walls, p, v)
                        .map(|(h, d, _)| Hit { at: h, near: d, far: Some(d), wall: true })
                        .or_else(|| march(&paved, p, v).map(|(h, d, f)| Hit { at: h, near: d, far: f, wall: false }))
                })
                .collect();
            // What each station faces: `Some(wall)` where it sees a face.
            let kind: Vec<Option<bool>> = (0..n).map(|i| is_face(&hit, i).then(|| hit[i].expect("a face has a hit").wall)).collect();
            for run in runs_of(&kind) {
                runs += 1;
                let whole = run.len() == n;
                let run_pts: Vec<Pt> = run.iter().map(|&i| pts[i]).collect();
                // The band is a wall's: a run that fronts a house gets it;
                // one that only fronts a mapped sidewalk keeps that
                // sidewalk's own width.
                let wall_faces = run.iter().filter(|&&i| kind[i] == Some(true)).count();
                let of_band: Shapes = if wall_faces >= min { band(&run_pts, whole) } else { Vec::new() };
                let mut run_pieces: Shapes = of_band.clone();
                bands.extend(of_band);
                let reach = bridged(&run, &hit, &kind, &pts, &normals);
                for (k, &i) in run.iter().enumerate() {
                    if let Some((h, wall)) = reach[k].filter(|_| kind[i].is_some()) {
                        hits.push((pts[i], h));
                        if wall {
                            walled += 1;
                        } else {
                            kerbed += 1;
                        }
                    }
                }
                let of_rung = strip(&run, &reach, &pts, &normals, whole);
                rungs.extend(of_rung.iter().cloned());
                run_pieces.extend(of_rung);
                pieces.extend(poly::union_all(&run_pieces));
            }
        }
    }
    let room = poly::union_all(&pieces);
    let grown = poly::union_of(&[before, &room]);
    let u = poly::fill_holes_under(grown.clone(), kerb::PAVEMENT_HOLE_M2);
    let pockets = pockets(&poly::union_of(&[carriageway, spanned]), &u, &facade.solid);
    let (pocket_count, pocket_m2) = (pockets.len(), poly::area(&pockets));
    // The ballast wins over the room as it does over every walk: a band
    // probed to a wall across a railway stops at the track bed. And a kerb
    // or a probe that meets the bed has not met bare ground.
    let senior = paving.senior();
    // **And the pavement holds its own minimum width.** The rule is the
    // chain's, applied once where the chain ends: every boolean before this
    // leaves scraps, and a region of pavement narrower than
    // [`PAVEMENT_MIN_M`] is not a pavement — it is the leftover of one. At
    // the Montreux overbridge two of them, 1.03 m² and 2.84 m², 0.41 m and
    // 0.71 m wide, sat beside the abutment; the first came out of
    // `surface`'s own cut and the second out of this step's bands, so no
    // one step could have caught them and the finished surface is the only
    // place that can. The ground pays for them twice over: each is a hole
    // in the terrain with a wall round all of it, up at road level over a
    // cutting.
    // **A pavement is bigger than the smallest one this step would draw.**
    // That is a band of [`PAVEMENT_MIN_M`] over [`RUN_MIN_M`] — the
    // narrowest strip over the shortest run of kerb it will start one for —
    // and nothing under it was drawn on purpose. What is under it is what a
    // boolean left: at the Montreux abutment a 1.03 m² scrap of footway
    // ribbon standing in the open, and a 2.84 m² lobe that is the *round
    // cap* of another, its body cut away by the asphalt and the cap left
    // sitting in the kerb line. Each is a hole in the terrain with a wall
    // round the whole of it, up at road level over a cutting.
    //
    // **Unless a wall explains it** ([`WALLED_SHARE`]). The one pavement
    // this step draws smaller than that is the strip against a facade — a
    // notch, the pocket a kerb return leaves at a house corner — and the
    // notch specimen's *whole* pavement is three such regions totalling
    // 1.8 m². They are bounded by the wall they were drawn for. The two
    // scraps are not: 1 % and 0 %.
    let paved = facade.pavement(&poly::union_of(&[&u, &pockets]), &senior);
    // **Nor a piece that runs onto a footbridge.** A footway's ground stub
    // between the kerb and the bridge it climbs onto is a few square metres
    // and has no wall, and was dropped as a scrap — leaving the kerb beside
    // it bare and the footbridge landing on nothing. No surface step paves a
    // walk span, so nothing here could see the bridge it belongs to.
    let bridges = Indexed::new(walk_spans);
    // **Nor a court.** A small region with asphalt or walls all round it
    // and no bare ground at its edge is a place the street encloses — the
    // strip between two garage lanes and the garage they end at, four square
    // metres — whichever construction paved it; dropped as a scrap, it left
    // both kerbs facing bare ground. A scrap is what a cut left, and a cut
    // leaves it standing in the open: the round cap in the kerb line has
    // bare ground along most of it.
    let closed_by = Indexed::new(&poly::union_of(&[&senior, &facade.solid]));
    let (pavement, loose): (Shapes, Shapes) = paved.into_iter().partition(|r| {
        poly::area(std::slice::from_ref(r)) >= RUN_MIN_M * PAVEMENT_MIN_M
            || walled_share(r, &walls) >= WALLED_SHARE
            || onto(r, &bridges)
            || enclosed(r, &closed_by)
    });
    let (loose_n, loose_m2) = (loose.len(), poly::area(&loose));
    // **What each scrap was made by.** A region too narrow to hold a
    // pavement ([`wide_enough`]) is not a defect on its own — the strip
    // between a kerb and a wall two metres off the axis is a decimetre wide
    // for the length of the house, and a notch in a facade is a small
    // isolated patch by construction. So the question is not its shape but
    // its provenance, and this is the measurement that makes provenance a
    // thing the model knows rather than a thing a reader guesses.
    const SOURCES: [&str; 5] = ["walk", "band", "rung", "closed", "island"];
    // **Each source is asked once, for every scrap at once.** A scrap is a
    // square metre; the sources run to hundreds of thousands of them, and
    // `poly::intersect` feeds both operands to the overlay whole with no
    // spatial culling — so asked scrap by scrap this was five full-surface
    // booleans per scrap, and more than half of the world build. A scrap is
    // contained in `scraps`, so intersecting it against what a source put
    // under *any* scrap is the same area as against the whole source.
    let scraps: Shapes = pavement.iter().filter(|r| !wide_enough(r)).cloned().collect();
    let scrap_n = scraps.len();
    let scrap_m2 = poly::area(&scraps);
    let mut by_source = [0usize; 5];
    if scrap_n > 0 {
        // Built only when there is a scrap to attribute, in [`SOURCES`]'s
        // order.
        let sources: [&Shapes; 5] = [
            before,
            &poly::union_all(&bands),
            &poly::union_all(&rungs),
            &poly::difference(&u, &grown),
            &pockets,
        ];
        let under: Vec<Shapes> =
            sources.iter().map(|shape| poly::intersect(&scraps, shape)).collect();
        for region in &scraps {
            let one = vec![region.clone()];
            // The source it draws most of its area from: a scrap is small
            // enough that one construction almost always made all of it, and
            // "most" says so without pretending the overlaps are disjoint.
            // Ties go to the earlier source, as they did when this counted
            // up from zero.
            let mut best: Option<(f64, usize)> = None;
            for (k, u) in under.iter().enumerate() {
                let share = poly::area(&poly::intersect(&one, u));
                if share > 0.0 && best.is_none_or(|(b, _)| share > b) {
                    best = Some((share, k));
                }
            }
            if let Some((_, k)) = best {
                by_source[k] += 1;
            }
        }
    }
    let scrap_of: String = SOURCES
        .iter()
        .enumerate()
        .filter(|(k, _)| by_source[*k] > 0)
        .map(|(k, name)| format!(" {name}:{}", by_source[k]))
        .collect();
    let filled = poly::area(&pavement) - poly::area(before);
    let bare = Bare::new(&senior, &pavement, &facade.footprints);
    let (gap_n, gap_of) = wall_gap(&bare, &hits);
    let (gaps, kerb_of) = kerb::kerb_gaps(carriageway, &bare, attached);
    let kerb_n = gaps.len();
    let summary = Summary::new()
        .with("stations", stations)
        .with_part("walled", walled, stations)
        .with("kerbed", kerbed)
        .with("runs", runs)
        .with("islands", format!("{pocket_count} ({pocket_m2:.0} m2)"))
        .with("scraps", format!("{scrap_n} ({scrap_m2:.1} m2){scrap_of}"))
        .with("loose", format!("{loose_n} ({loose_m2:.1} m2)"))
        .with_regions("pavement", &pavement)
        .with_m2("pavement_m2", poly::area(&pavement))
        .with_m2("filled_m2", filled)
        .with_share("wall_gap", gap_n, gap_of)
        .with_share("kerb_gap", kerb_n, kerb_of);
    // The last step that lays any paving, so this is the paved surface as it
    // finally stands: everything downstream reads this one layer.
    let surface = Surface { walk: pavement, ..paving.clone() };
    (Room { surface, room, gaps }, summary)
}

/// What a probe met.
#[derive(Debug, Clone, Copy)]
struct Hit {
    /// Where it met it.
    at: Pt,
    /// How far out, in metres.
    near: f64,
    /// How far out it ends: the same for a wall, the pavement's far edge
    /// for a pavement, and `None` for a pavement that runs on past the
    /// reach.
    far: Option<f64>,
    wall: bool,
}

/// The point each station of a run reaches: its own hit where it sees a
/// face, else, between two stations that do, the point along its normal
/// at the far distance interpolated between theirs — so a break in a
/// sidewalk is paved at the sidewalk's width, a driveway between two
/// house fronts at the depth of each. A side whose pavement runs on past
/// the reach has no edge to interpolate to and defers to the other; two
/// such sides bridge at the reach. The kind is the nearer neighbour's.
/// Stations before the first face and after the last have `None`.
fn bridged(run: &[usize], hit: &[Option<Hit>], kind: &[Option<bool>], pts: &[Pt], normals: &[Pt]) -> Vec<Option<(Pt, bool)>> {
    let seen = |k: usize| kind[run[k]].map(|_| hit[run[k]].expect("a face has a hit"));
    let mut out: Vec<Option<(Pt, bool)>> = (0..run.len()).map(|k| seen(k).map(|h| (h.at, h.wall))).collect();
    let mut k = 0;
    while k < run.len() {
        if seen(k).is_some() {
            k += 1;
            continue;
        }
        let Some(a) = (0..k).rev().find(|&j| seen(j).is_some()) else {
            k += 1;
            continue;
        };
        let Some(b) = (k + 1..run.len()).find(|&j| seen(j).is_some()) else {
            break;
        };
        let (ha, hb) = (seen(a).expect("a hit"), seen(b).expect("a hit"));
        for j in k..b {
            let t = (j - a) as f64 / (b - a) as f64;
            let d = match (ha.far, hb.far) {
                (Some(da), Some(db)) => da + (db - da) * t,
                (Some(d), None) | (None, Some(d)) => d,
                (None, None) => WALL_REACH_M,
            };
            let (p, n) = (pts[run[j]], normals[run[j]]);
            out[j] = Some(([p[0] + n[0] * d, p[1] + n[1] * d], if t < 0.5 { ha.wall } else { hb.wall }));
        }
        k = b;
    }
    out
}

/// The holes of what is built — `carriageway`, `pavement` and `solid`
/// together — under [`ISLAND_M2`] that border the asphalt or the pavement
/// and lie wholly within [`POCKET_REACH_M`] of the asphalt, each as a region
/// of its own: a hole ring winds clockwise, so it is reversed. A hole
/// walled all round is a courtyard, and one that reaches farther from the
/// kerb than that — the lawn between two footpaths, the block with no
/// street in it — is not the street's.
fn pockets(carriageway: &Shapes, pavement: &Shapes, solid: &Shapes) -> Shapes {
    let built = poly::union_of(&[carriageway, pavement, solid]);
    let (asphalt, paved) = (Indexed::new(carriageway), Indexed::new(pavement));
    let kerbs = poly::Edges::new(carriageway, POCKET_REACH_M);
    built
        .iter()
        .flat_map(|shape| shape.iter().skip(1))
        .filter(|ring| -poly::ring_area(ring) < ISLAND_M2)
        .filter(|ring| within_reach(ring, &kerbs))
        .filter(|ring| {
            // Just outside the hole is to the left of each edge, the ring
            // being clockwise.
            (0..ring.len()).any(|i| {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                let t = poly::unit([b[0] - a[0], b[1] - a[1]]);
                let m = [(a[0] + b[0]) * 0.5 - t[1] * EDGE_M, (a[1] + b[1]) * 0.5 + t[0] * EDGE_M];
                asphalt.contains(m) || paved.contains(m)
            })
        })
        .map(|ring| vec![ring.iter().rev().copied().collect::<Vec<_>>()])
        // **A hole too narrow to pave is not a place** ([`wide_enough`]).
        // Last, because it is the only test here that can run the polygon
        // kernel, and by now there are few rings left to run it on.
        .filter(|region| wide_enough(region))
        .collect()
}

/// Whether no point of the hole `ring` is farther than [`POCKET_REACH_M`]
/// from an edge in `kerbs`: its outline sampled every station, and its
/// inside on a grid of the same pitch, so the middle of a yard is measured
/// and not just its rim.
fn within_reach(ring: &[Pt], kerbs: &poly::Edges) -> bool {
    if !kerb::stations(ring).iter().all(|s| kerbs.within(s.at, POCKET_REACH_M)) {
        return false;
    }
    let region: Shapes = vec![vec![ring.to_vec()]];
    let [x0, y0, x1, y1] = poly::bounds(ring.iter().copied()).expect("a ring has points");
    let step = kerb::STATION_M;
    let mut y = (y0 / step).ceil() * step;
    while y < y1 {
        let mut x = (x0 / step).ceil() * step;
        while x < x1 {
            if poly::contains(&region, [x, y]) && !kerbs.within([x, y], POCKET_REACH_M) {
                return false;
            }
            x += step;
        }
        y += step;
    }
    true
}

/// The stations of one kerb ring, a metre apart, each with its outward
/// unit normal. The region lies on the left of every ring, outer or hole,
/// so outward is the right-hand normal of the tangent.
fn kerb_stations(ring: &[Pt]) -> (Vec<Pt>, Vec<Pt>) {
    kerb::stations(ring).iter().map(|s| (s.at, [s.tangent[1], -s.tangent[0]])).unzip()
}

/// The first point along `n` from `p`, within [`WALL_REACH_M`], that lies
/// in `index`, how far out it is, and how far out the run of points inside
/// it goes on before it leaves again — `None` if it has not left when the
/// reach ends.
fn march(index: &Indexed, p: Pt, n: Pt) -> Option<(Pt, f64, Option<f64>)> {
    let steps = (WALL_REACH_M / MARCH_M).round() as usize;
    let at = |k: usize| [p[0] + n[0] * MARCH_M * k as f64, p[1] + n[1] * MARCH_M * k as f64];
    let first = (1..=steps).find(|&k| index.contains(at(k)))?;
    let last = (first..=steps).take_while(|&k| index.contains(at(k))).last().unwrap_or(first);
    Some((at(first), MARCH_M * first as f64, (last < steps).then(|| MARCH_M * last as f64)))
}

/// Whether station `i` sees a face: a wall, and a neighbour that sees a
/// wall at nearly the same distance.
fn is_face(hit: &[Option<Hit>], i: usize) -> bool {
    let n = hit.len();
    let Some(Hit { near: d, .. }) = hit[i] else {
        return false;
    };
    [(i + n - 1) % n, (i + 1) % n]
        .into_iter()
        .any(|j| hit[j].is_some_and(|h| (h.near - d).abs() <= FACE_MAX_GRADE * kerb::STATION_M))
}

/// The runs of walled stations round a ring, as index lists in ring
/// order, from what each station faces: `Some(true)` a wall, `Some(false)`
/// a pavement, `None` nothing. Breaks under [`BRIDGE_M`] are bridged —
/// under [`SIDEWALK_BRIDGE_M`] between two pavements — and runs under
/// [`RUN_MIN_M`] dropped. A ring walled all round is one run of every
/// station.
fn runs_of(kind: &[Option<bool>]) -> Vec<Vec<usize>> {
    let face: Vec<bool> = kind.iter().map(|k| k.is_some()).collect();
    let n = face.len();
    let bridge_of = |a: usize, b: usize| -> usize {
        let m = if kind[a] == Some(false) && kind[b] == Some(false) { SIDEWALK_BRIDGE_M } else { BRIDGE_M };
        (m / kerb::STATION_M) as usize
    };
    let bridge = (SIDEWALK_BRIDGE_M.max(BRIDGE_M) / kerb::STATION_M) as usize;
    let min = (RUN_MIN_M / kerb::STATION_M) as usize;
    if !face.iter().any(|&f| f) {
        return Vec::new();
    }
    let Some(start) = (0..n).find(|&i| !face[i]) else {
        return if n >= min { vec![(0..n).collect()] } else { Vec::new() };
    };
    // Bridge every unwalled stretch of at most `bridge` stations that
    // lies between two walled ones, walking once round from a break.
    let mut marked = face.to_vec();
    let mut i = start;
    for _ in 0..n {
        let k = (i + 1) % n;
        if face[i] && !face[k] {
            let (mut len, mut j) = (0usize, k);
            while !face[j] && len <= bridge {
                len += 1;
                j = (j + 1) % n;
            }
            if face[j] && len <= bridge_of(i, j) {
                let mut m = k;
                while m != j {
                    marked[m] = true;
                    m = (m + 1) % n;
                }
            }
        }
        i = k;
    }
    let mut runs: Vec<Vec<usize>> = Vec::new();
    let mut cur: Vec<usize> = Vec::new();
    for step in 1..=n {
        let i = (start + step) % n;
        if marked[i] {
            cur.push(i);
        } else if !cur.is_empty() {
            runs.push(std::mem::take(&mut cur));
        }
    }
    if !cur.is_empty() {
        runs.push(cur);
    }
    runs.into_iter().filter(|r| r.len() >= min).collect()
}

/// The band along a run: its stations buffered to twice [`BAND_M`] with
/// flat ends (the carriageway takes the inner half back). A whole ring
/// is closed on itself.
fn band(pts: &[Pt], whole: bool) -> Shapes {
    let mut line = pts.to_vec();
    if whole {
        line.push(pts[0]);
    }
    poly::buffer_line_capped(&line, 2.0 * BAND_M, [whole, whole])
}

/// The strip a run reaches: the quad between every two consecutive
/// stations with a reach, from the kerb (overlapped by a probe's worth so
/// the boolean leaves no seam) to their reach points, and a cap of
/// [`CAP_M`] along the kerb past the first and last station of every
/// stretch. Round a whole ring the last station's neighbour is the first.
fn strip(run: &[usize], reach: &[Option<(Pt, bool)>], pts: &[Pt], normals: &[Pt], whole: bool) -> Shapes {
    let n = run.len();
    let foot = |k: usize| -> Pt {
        let (s, v) = (pts[run[k]], normals[run[k]]);
        [s[0] - v[0] * PROBE_M, s[1] - v[1] * PROBE_M]
    };
    let cap = |k: usize, h: Pt, forward: bool| -> Option<Shape> {
        let v = normals[run[k]];
        let sign = if forward { 1.0 } else { -1.0 };
        let t = [-v[1] * sign * CAP_M, v[0] * sign * CAP_M];
        let f = foot(k);
        hull(vec![f, h, [f[0] + t[0], f[1] + t[1]], [h[0] + t[0], h[1] + t[1]]])
    };
    let mut out: Shapes = Vec::new();
    for k in 0..n {
        let Some((h, _)) = reach[k] else {
            continue;
        };
        let prev = (k > 0 || whole).then(|| (k + n - 1) % n).and_then(|j| reach[j]);
        let next = (k + 1 < n || whole).then(|| (k + 1) % n).and_then(|j| reach[j]);
        if let Some((g, _)) = next {
            out.extend(hull(vec![foot(k), foot((k + 1) % n), g, h]));
        } else {
            out.extend(cap(k, h, true));
        }
        if prev.is_none() {
            out.extend(cap(k, h, false));
        }
    }
    out
}

/// The convex hull of `pts` as a region, if it has area.
fn hull(pts: Vec<Pt>) -> Option<Shape> {
    poly::convex_hull(pts).map(|ring| vec![ring])
}

/// The share of walled stations with bare ground just outside them, and
/// their number: for every `(station, hit)`, the probe a step outside the
/// station toward its hit.
pub fn wall_gap(bare: &Bare, hits: &[(Pt, Pt)]) -> (usize, usize) {
    let mut gaps = 0usize;
    for &(s, h) in hits {
        let n = poly::unit([h[0] - s[0], h[1] - s[1]]);
        if bare.at([s[0] + n[0] * PROBE_M, s[1] + n[1] * PROBE_M]) {
            gaps += 1;
        }
    }
    (gaps, hits.len())
}

#[cfg(test)]
pub(crate) mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{built, plan};
    use crate::step::Step;

    use super::*;

    /// **A footway's stub onto a footbridge is kept.** A footway leaves the
    /// street's kerb and climbs onto a bridge 2.25 m out: the stub between is
    /// 4.5 m², under the smallest pavement this step draws, has no wall, and
    /// was dropped as a scrap — the bridge landing on bare ground.
    #[test]
    fn a_stub_onto_a_footbridge_is_kept() {
        let mut w = built("flat", "net:straight?len=200", None, 100.0, &plan(Step::Facade)).0;
        let roads = w.roads.as_mut().unwrap();
        let mut stub = roads.plan[0].clone();
        stub.id = "stub".into();
        stub.class = "footway".into();
        stub.width_m = crate::width::WALK_M;
        stub.pts = vec![[0.0, 0.0], [0.0, 5.0]];
        let mut deck = stub.clone();
        deck.id = "deck".into();
        deck.kind = crate::world::Kind::Bridge(1);
        deck.pts = vec![[0.0, 5.0], [0.0, 40.0]];
        roads.plan.push(stub);
        roads.spans.push(deck);
        let roads = w.roads.as_ref().unwrap();
        let facade = w.facade.as_ref().unwrap();
        let (ribbons, _) = crate::ribbon::run(roads);
        let (surface, _) = crate::surface::run(roads, &ribbons, facade);
        let (k, _) = kerb::run(roads, &surface, facade);
        let (f, _) = crate::legs::run(roads, &surface, &k, facade);
        let bridges: Shapes = crate::surface::spans_grouped(roads)
            .into_iter()
            .filter(|(fam, ..)| *fam == crate::width::Family::Walk)
            .flat_map(|(.., s)| s)
            .collect();
        let (r, s) = run(&f.surface, &bridges, &k.attached, facade);
        assert!(poly::contains(&r.surface.walk, [0.0, 4.0]), "{s}");
        // Without the bridge it is the scrap it looks like.
        let (r, s) = run(&f.surface, &Vec::new(), &k.attached, facade);
        assert!(!poly::contains(&r.surface.walk, [0.0, 4.0]), "{s}");
    }

    /// **A court is kept however small.** Two garage lanes leave a service
    /// road side by side and end at the garage across them: the court
    /// between is 3.75 m², under the smallest pavement this step draws, and
    /// the garage bounds a fifth of it, not the quarter that would have
    /// excused it. It is enclosed all round, and was dropped as a scrap.
    #[test]
    fn a_small_court_is_kept() {
        let w = built("flat", "net:straight?len=200&class=service", Some("house:beside?d=4&x=2.25&l=12&w=6"), 100.0, &plan(Step::Facade)).0;
        let facade = w.facade.as_ref().unwrap();
        let carriageway = poly::union_all(&vec![
            poly::rect(-10.0, -1.5, 15.0, 1.5),
            poly::rect(-1.5, -1.5, 1.5, 4.0),
            poly::rect(3.0, -1.5, 6.0, 4.0),
        ]);
        let paving = Surface { carriageway, walk: Vec::new(), spanned: Vec::new(), ballast: Vec::new() };
        let (r, s) = run(&paving, &Vec::new(), &[], facade);
        assert!(poly::contains(&r.surface.walk, [2.25, 2.75]), "{s}");
    }

    /// The world of `net` with the house of `house`, paved to the room.
    pub(crate) fn paved(net: &str, house: &str) -> (World, String) {
        let (w, ran) = built("flat", net, Some(house), 100.0, &plan(Step::Room));
        let s = ran.last().to_string();
        (w, s)
    }

    /// The world of `net` with no house, paved to the junctions, and the
    /// pavement before the room step.
    fn roomed(net: &str) -> (World, String, Shapes) {
        let (w, ran) = built("flat", net, None, 100.0, &plan(Step::Room));
        let before = w.legs.as_ref().expect("the legs step ran").surface.walk.clone();
        (w, ran.last().to_string(), before)
    }

    /// The building footprints of a world.
    fn walls(w: &World) -> &Shapes {
        &w.facade.as_ref().expect("the facade step ran").footprints
    }

    #[test]
    fn runs_bridge_short_breaks_and_drop_short_runs() {
        let wall = Some(true);
        let mut face = vec![None; 40];
        for i in 5..12 {
            face[i] = wall;
        }
        for i in 15..30 {
            face[i] = wall;
        }
        let runs = runs_of(&face);
        assert_eq!(runs.len(), 1, "{runs:?}");
        assert_eq!(runs[0].len(), 25, "5..30 bridged across 12..15");
        // A run of four is not a pavement; a ring walled all round is one.
        let mut short = vec![None; 40];
        for i in 3..7 {
            short[i] = wall;
        }
        assert!(runs_of(&short).is_empty());
        assert_eq!(runs_of(&vec![wall; 40]).len(), 1);
        // A run across index 0 is one run.
        let mut wrap = vec![None; 40];
        for i in (35..40).chain(0..5) {
            wrap[i] = wall;
        }
        assert_eq!(runs_of(&wrap).len(), 1);
        assert_eq!(runs_of(&wrap)[0].len(), 10);
        // A 15 m break is bridged between two sidewalks, not two walls.
        let mut long = vec![None; 60];
        for i in (5..15).chain(30..45) {
            long[i] = wall;
        }
        assert_eq!(runs_of(&long).len(), 2);
        for i in (5..15).chain(30..45) {
            long[i] = Some(false);
        }
        assert_eq!(runs_of(&long).len(), 1);
    }

    #[test]
    fn a_wall_within_reach_is_paved_to() {
        // The facade at y = 4 stands 1.25 m outside the kerb at 2.75: the
        // strip between them is pavement over the house's length.
        let (w, s) = paved("net:straight?len=200", "house:beside?d=4&l=20");
        let p = &w.room.as_ref().unwrap().surface.walk;
        assert!(poly::contains(p, [0.0, 3.0]), "{s}");
        assert!(poly::contains(p, [0.0, 3.9]), "{s}");
        assert!(poly::contains(p, [9.0, 3.5]), "{s}");
        assert!(!poly::contains(p, [0.0, 4.1]), "not in the house");
        assert!(!poly::contains(p, [14.0, 3.5]), "nothing far past the house");
        assert!(!poly::contains(p, [0.0, -3.0]), "nothing on the bare side");
        assert!(s.contains("wall_gap=0/"), "{s}");
        assert!(!s.contains("walled=0 "), "{s}");
        assert!(poly::intersect(p, walls(&w)).is_empty());
        assert!(poly::intersect(p, &w.legs.as_ref().unwrap().surface.carriageway).is_empty());
    }

    #[test]
    fn a_wall_beyond_reach_keeps_its_garden() {
        let (w, s) = paved("net:straight?len=200", "house:beside?d=10&l=20");
        assert!(w.room.as_ref().unwrap().surface.walk.is_empty(), "{s}");
        assert!(s.contains("walled=0 "), "{s}");
    }

    #[test]
    fn a_far_facade_gets_pavement_to_its_wall() {
        // The facade 3.5 m off the kerb, within reach: paved to the wall
        // over the house, and nothing on the bare side.
        let (w, s) = paved("net:straight?len=200", "house:beside?d=6.25&l=20");
        let p = &w.room.as_ref().unwrap().surface.walk;
        assert!(poly::contains(p, [0.0, 3.5]) && poly::contains(p, [0.0, 6.0]), "{s}");
        assert!(!poly::contains(p, [0.0, 6.5]) && !poly::contains(p, [0.0, -3.5]));
        assert!(s.contains("wall_gap=0/"), "{s}");
    }

    #[test]
    fn a_notch_in_the_facade_is_pavement_not_asphalt() {
        // The facade at y = 2 is inside the prior width, with a 2 m wide,
        // 1 m deep notch at the origin. The asphalt's edge runs straight
        // along y = 2 — the notch is closed for it — and the notch itself
        // is paved by the rungs that reach into it.
        let (w, s) = paved("net:straight?len=200", "house:beside?d=2&l=20&notch=2");
        let c = &w.legs.as_ref().unwrap().surface.carriageway;
        assert!(poly::contains(c, [0.0, 1.9]));
        assert!(!poly::contains(c, [0.0, 2.5]), "no asphalt in the notch");
        assert!(poly::contains(c, [5.0, 1.9]) && !poly::contains(c, [5.0, 2.1]));
        let p = &w.room.as_ref().unwrap().surface.walk;
        assert!(poly::contains(p, [0.0, 2.5]), "the notch is pavement: {s}");
        assert!(!poly::contains(p, [0.0, 3.1]), "not the house");
        assert!(s.contains("wall_gap=0/"), "{s}");
    }

    #[test]
    fn a_gap_between_two_houses_gets_no_asphalt() {
        // Two houses along the road with a 2 m gap: the asphalt keeps its
        // straight edge along the closed facade; the band bridges the
        // alley's mouth and stops there; the houses are never paved.
        let (w, _) = paved("net:straight?len=200", "house:row?d=2&l=10&gap=2");
        let c = &w.legs.as_ref().unwrap().surface.carriageway;
        assert!(poly::contains(c, [0.0, 1.9]));
        assert!(!poly::contains(c, [0.0, 2.5]), "no asphalt into the gap");
        let p = &w.room.as_ref().unwrap().surface.walk;
        assert!(poly::contains(p, [0.0, 3.0]), "the mouth is banded");
        assert!(!poly::contains(p, [0.0, 6.0]), "the alley is not pavement");
        assert!(!poly::contains(p, [2.0, 3.0]), "the house is not");
        assert!(poly::intersect(p, walls(&w)).is_empty());
    }

    #[test]
    fn a_house_at_the_corner_leaves_no_pocket() {
        // The house's corner 0.25 m off both kerbs of the tee's north-west
        // corner: the strips along both walls and the pocket between the
        // return and the corner are paved.
        let (w, s) = paved("net:tee?len=200", "house:beside?d=3&x=-8&l=10&w=10");
        let p = &w.room.as_ref().unwrap().surface.walk;
        let c = &w.legs.as_ref().unwrap().surface.carriageway;
        for q in [[-8.0, 2.9], [-2.9, 8.0], [-2.85, 2.85], [-4.0, 2.9], [-2.9, 4.0]] {
            assert!(poly::contains(p, q) || poly::contains(c, q), "{q:?} is bare: {s}");
        }
        assert!(!poly::contains(p, [-3.5, 3.5]), "not the house");
        assert!(s.contains("wall_gap=0/"), "{s}");
    }

    #[test]
    fn a_corner_pointing_at_the_road_is_not_paved() {
        // A house turned 45° with its corner 0.2 m off the kerb: the probe
        // sees a wall for eight metres of kerb, but never a face, and the
        // kerb stays bare rather than webbed.
        let (w, s) = paved("net:straight?len=200", "house:beside?d=5&rot=45");
        assert!(w.room.as_ref().unwrap().surface.walk.is_empty(), "{s}");
        assert!(s.contains("walled=0 "), "{s}");
        assert!(s.contains("runs=0 "), "{s}");
    }

    #[test]
    fn a_short_facade_is_not_a_pavement() {
        let (w, s) = paved("net:straight?len=200", "house:beside?d=4&l=4");
        assert!(w.room.as_ref().unwrap().surface.walk.is_empty(), "{s}");
        let (w, s) = paved("net:straight?len=200", "house:beside?d=4&l=8");
        assert!(!w.room.as_ref().unwrap().surface.walk.is_empty(), "{s}");
        assert!(s.contains("runs=1 "), "{s}");
    }

    #[test]
    fn a_break_in_a_sidewalk_is_bridged() {
        // Two sidewalk halves 6 m off the axis with a 6 m break between
        // them: the kerb step fills each half to the kerb, and the break
        // is paved across at the same width.
        let (w, s, before) = roomed("net:sidewalk?d=6&gap=6&len=100");
        let p = &w.room.as_ref().unwrap().surface.walk;
        assert!(poly::contains(p, [-10.0, 4.0]) && poly::contains(p, [10.0, 4.0]), "the halves: {s}");
        assert!(poly::contains(p, [0.0, 4.0]), "the break: {s}");
        assert!(poly::area(p) > poly::area(&before) + 1.0, "{s}");
        assert!(s.contains("kerb_gap=0/"), "{s}");
    }

    #[test]
    fn a_roundabout_island_is_paved() {
        // A ring road of radius 12 with a 5.5 m width leaves a 9.25 m
        // island: paved, up to the kerb, with no hole in it.
        let (w, s, _) = roomed("net:roundabout?r=12&d=5&len=100");
        let p = &w.room.as_ref().unwrap().surface.walk;
        assert!(poly::contains(p, [0.0, 0.0]), "{s}");
        assert!(poly::contains(p, [9.0, 0.0]), "{s}");
        assert!(!poly::contains(p, [9.5, 0.0]), "the asphalt");
        assert!(s.contains("islands=1 "), "{s}");
        // A block enclosed by roads is not an island: a 40 m cross' quadrants
        // stay ground.
        let (_w, s, _) = roomed("net:cross?len=200");
        assert!(s.contains("islands=0 "), "{s}");
    }

    /// The box `[x0, y0, x1, y1]` as a counter-clockwise ring.
    fn box_ring(b: [f64; 4]) -> Vec<Pt> {
        poly::rect(b[0], b[1], b[2], b[3]).remove(0)
    }

    /// The region between two boxes: a wall or a footway round a yard.
    fn frame(outer: [f64; 4], inner: [f64; 4]) -> Shape {
        vec![box_ring(outer), poly::oriented(box_ring(inner), false)]
    }

    #[test]
    fn a_pocket_within_reach_is_paved_and_a_yard_is_not() {
        // Two 6 m roads meet at the origin; a diagonal 2 m footway closes
        // the corner between them at `d`. The triangle inside is a hole in
        // what is built, bordered by asphalt: paved when no point of it is
        // farther than the reach from the asphalt, left when the footway
        // stands back.
        let asphalt = poly::union_all(&vec![vec![box_ring([-3.0, -50.0, 3.0, 50.0])], vec![box_ring([-50.0, -3.0, 50.0, 3.0])]]);
        let corner = |d: f64| -> Shapes {
            let footway: Shape = vec![vec![[3.0, d + 1.0], [3.0, d - 1.0], [d - 1.0, 3.0], [d + 1.0, 3.0]]];
            pockets(&asphalt, &vec![footway], &Vec::new())
        };
        let near = corner(10.0);
        assert_eq!(near.len(), 1, "{near:?}");
        assert!(poly::contains(&near, [5.0, 5.0]));
        assert!(corner(25.0).is_empty(), "the far corner is 12 m from both kerbs");
        // A yard that only walls bound is a courtyard, within reach or not.
        let court = vec![frame([3.0, 3.0, 12.0, 12.0], [5.0, 5.0, 10.0, 10.0])];
        assert!(pockets(&asphalt, &Vec::new(), &court).is_empty());
        // A yard that footways bound is the street's within the reach and
        // a lawn past it, whatever its size.
        let yard = vec![frame([3.0, 3.0, 12.0, 12.0], [5.0, 5.0, 10.0, 10.0])];
        assert_eq!(pockets(&asphalt, &yard, &Vec::new()).len(), 1);
        let lawn = vec![frame([20.0, 20.0, 40.0, 40.0], [22.0, 22.0, 38.0, 38.0])];
        assert!(pockets(&asphalt, &lawn, &Vec::new()).is_empty());
    }

    #[test]
    fn the_strip_has_the_edge_its_reach_points_draw() {
        // Stations along y = 0 a metre apart, walked toward −x so the
        // region on their left is below and outward is +y, reaching to
        // y = 1 + x / 2: the strip's edge is that line, sampled between
        // the stations, where a rung per station stood out of it by half
        // a rung.
        let n = 11;
        let pts: Vec<Pt> = (0..n).rev().map(|i| [i as f64, 0.0]).collect();
        let normals = vec![[0.0, 1.0]; n];
        let run: Vec<usize> = (0..n).collect();
        let reach: Vec<Option<(Pt, bool)>> = pts.iter().map(|p| Some(([p[0], 1.0 + p[0] / 2.0], true))).collect();
        let s = poly::union_all(&strip(&run, &reach, &pts, &normals, false));
        for i in 0..n - 1 {
            let x = i as f64 + 0.5;
            let y = 1.0 + x / 2.0;
            assert!(poly::contains(&s, [x, y - 0.1]), "under the edge at {x}");
            assert!(!poly::contains(&s, [x, y + 0.1]), "a tooth over the edge at {x}");
        }
        // The caps: half a station past either end, square to the kerb.
        assert!(poly::contains(&s, [-0.4, 0.5]) && !poly::contains(&s, [-0.6, 0.5]));
        assert!(poly::contains(&s, [10.4, 3.0]) && !poly::contains(&s, [10.6, 3.0]));
    }

    #[test]
    fn no_buildings_no_change() {
        let (w, s, before) = roomed("net:sidewalk?d=6&len=100");
        assert!(s.contains("walled=0 "), "{s}");
        // The probe meets the mapped sidewalk's own fill and adds nothing
        // but the half-rung past its last station at either end.
        assert!((poly::area(&w.room.as_ref().unwrap().surface.walk) - poly::area(&before)).abs() < 1.0, "{s}");
    }
}
