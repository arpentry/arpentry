//! The junctions, built from the legs that meet there. **Wired
//! (2026-09-28)**: `room` reads this step's surface.
//!
//! Review step 5 (`docs/plans/plan-chain-from-legs.md`), first slice. Before
//! it a junction was not an object: it was where ribbons overlap, the
//! surface step unioned them, and the fillet step found the notches the
//! union left and rounded them with a masked closing. Every fact about the
//! junction — which legs meet, which kerbs face each other, what radius the
//! return takes — was recovered afterwards from the union by sampling
//! (`fillet`'s `kerbs_at`), and the closing left hairlines that
//! `OPEN_M`, `OVERLAP_M` and `fill_holes_under` then cleaned away.
//!
//! This step builds the same carriageway from what is known before any
//! boolean runs:
//!
//! - **The pieces** are the ground carriageways and the **bridge** spans, so
//!   a junction standing on a deck is built like any other: a deck and the
//!   road that runs onto it are one surface. A span edge shapes the
//!   junctions it meets but is not paved here — the `sheet` step unions the
//!   span ribbons in. A bore is never continuous with the ground, and is
//!   left out as [`crate::ribbon::spans_grouped`] leaves it out.
//! - **A node** is a connector where two or more leg ends meet: two pieces,
//!   a piece's end on another's interior vertex, or a closed way meeting
//!   itself. A piece is cut into **edges** at every node it passes.
//! - **An end that lands on another road** — free, and inside that road's
//!   width, with no vertex there — is a junction the source did not connect
//!   ([`land`]). The host is given a vertex at the end's foot and the end is
//!   carried to it, so the two meet at a node like any other.
//! - **Each junction is one polygon.** Its legs are sorted by bearing, and
//!   between each consecutive pair the facing kerbs — the left kerb of one,
//!   the right kerb of the next — are intersected. A corner turning at least
//!   [`BEND_MIN_DEG`] is replaced by a tangent arc of the narrower leg's
//!   [`width::fillet_m`], its tangent length clamped at
//!   [`RETURN_MAX_RADII`] radii; a shallower one is a mitre; a
//!   sector of half a turn or more is the round join a ribbon has there.
//! - **A leg's mouth** is where the farther of its two sides stops, and the
//!   edge is trimmed to it with a square end. No round cap is drawn at any
//!   connector, so none can survive as a lobe.
//!
//! The step hands back a whole [`Surface`], as `fillet` did: the explicit
//! carriageway, and the pavement laid back outside wherever that carriageway
//! grew into it. It replaced `fillet` (deleted 2026-09-28) after measuring
//! better on every counter of the A/B in the plan but mesh slivers, which
//! are the mesher's (+0.8–1.6 % across four lattice spacings).

use std::collections::HashMap;

use crate::poly::{self, Pt, Ring, Shapes};
use crate::step::Summary;
use crate::width::{self, Family};
use crate::world::{connector, Facade, Junction, Leg, Legs, PieceEdge, Polyline2, Network, Surface};

/// How far a trimmed edge laps back into its junction, in metres. A boolean
/// union keeps shapes that only touch apart, and the junction's mouth and
/// the edge's square end are computed by two constructions that round
/// differently; lapped by a centimetre, they overlap instead of touching.
const LAP_M: f64 = 0.01;

/// A kerb corner turning inward by less than this, in degrees, is a bend
/// drawn as a chain of turns, not a corner worth a return: it is mitred.
pub const BEND_MIN_DEG: f64 = 30.0;

/// The longest a return runs along a kerb from its corner, in radii. The
/// tangent points recede as `tan(τ/2)`, without bound as the kerbs come
/// to meet head-on: a corner sharper than about 143° is two kerbs grazing
/// each other, and its return stops here.
pub const RETURN_MAX_RADII: f64 = 3.0;

/// A kerb corner farther from the node than this many summed half-widths is
/// not a corner: the two legs are near-collinear and of different widths,
/// so their kerbs meet far off or behind the node. The side is joined as if
/// it were convex, and counted as `far`.
const FAR_HALF_WIDTHS: f64 = 4.0;

/// What building one side of a junction found.
#[derive(Debug, Default)]
struct Tally {
    returns: usize,
    mitres: usize,
    rounds: usize,
    far: usize,
    tangled: usize,
    clamped: usize,
    bent: usize,
    short: usize,
    landed: usize,
    bends: usize,
}

pub fn run(
    roads: &Network,
    surface: &Surface,
    k: &crate::world::Kerb,
    facade: &Facade,
    masks: &[(Family, usize, Shapes)],
) -> (Legs, Summary) {
    let all: Vec<&Polyline2> = roads.pieces().collect();
    let ground = roads.plan.len();
    let mut pieces: Vec<(usize, Vec<Pt>)> = all
        .iter()
        .enumerate()
        .filter(|&(i, p)| {
            width::family(&p.class) == Family::Carriageway
                && (i < ground || matches!(p.kind, crate::world::Kind::Bridge(_)))
        })
        .map(|(i, p)| (i, dedup(&p.pts)))
        .filter(|(_, pts)| pts.len() >= 2)
        .collect();
    close_twins(&mut pieces);
    let mut tally = Tally { landed: land(&all, &mut pieces), ..Tally::default() };
    let (mut edges, nodes) = cut(&all, ground, &pieces);
    let mut junctions: Vec<Junction> = nodes
        .values()
        .map(|ends| junction(ends, &edges, &nodes, &all, &mut tally))
        .collect();
    // The nodes come out of a hash map; the output is sorted by position.
    junctions.sort_by(|a, b| a.at.partial_cmp(&b.at).expect("finite"));
    for j in &junctions {
        for leg in &j.legs {
            edges[leg.edge].trim[if leg.at_start { 0 } else { 1 }] = leg.mouth_m;
            if seg(&leg.line, 0) + 1e-6 < leg.mouth_m {
                tally.bent += 1;
            }
        }
    }
    // A junction with a deck among its legs is paved only off the deck: its
    // returns, and the round of its node the ground ribbons would have
    // paved, as the fillet paves them. The deck is the `sheet` step's —
    // paved here too, it is ground asphalt under a bridge that no piece of
    // the ground claims.
    //
    // **Off the deck is off the square footprint** ([`crate::ribbon::spans_masked`],
    // the mask the bench cuts the ground's hole by), not off the round-capped
    // ribbon: taken by the ribbon, the disc at the node went with it. And
    // the remainder is **opened by [`LAP_M`]**: the junction's kerbs and
    // the deck's are two constructions of one line, and a difference along
    // it leaves hairline fragments no piece claims — they were most of the
    // `sheet` step's orphans.
    let decks: Shapes = poly::union_all(
        &masks
            .iter()
            .filter(|(f, ..)| *f == Family::Carriageway)
            .flat_map(|(.., s)| s.iter().cloned())
            .collect(),
    );
    let mut parts: Shapes = Vec::new();
    for j in &junctions {
        if j.legs.iter().all(|g| edges[g.edge].ground) {
            parts.extend(j.shape.iter().cloned());
        } else {
            let off = poly::difference(&j.shape, &decks);
            parts.extend(poly::dilate_sharp(&poly::erode(&off, LAP_M), LAP_M));
        }
    }
    for e in edges.iter().filter(|e| e.ground) {
        let len = poly::length(&e.pts);
        let a = if e.node[0] { e.trim[0] - LAP_M } else { 0.0 };
        let b = if e.node[1] { len - e.trim[1] + LAP_M } else { len };
        // **An edge its two mouths overrun is paved whole.** Its junctions
        // were meant to pave it between them, but a return whose tangent
        // point the kerb walk found past the far node leaves a ring that
        // runs out along the next edge and back, and the edge itself can
        // fall outside it. The union makes paving it twice free.
        let r0 = width::fillet_m(&all[e.piece].class);
        if e.trim[0] + e.trim[1] >= len {
            tally.short += 1;
            parts.extend(poly::buffer_line_capped(&e.pts, e.width_m, [false, false]));
            parts.extend(bend_returns(&e.pts, e.width_m / 2.0, r0, 0.0, len, &mut tally));
            continue;
        }
        parts.extend(poly::buffer_line_capped(&between(&e.pts, a, b), e.width_m, [false, false]));
        parts.extend(bend_returns(&e.pts, e.width_m / 2.0, r0, a, b, &mut tally));
    }
    // **A hole in the asphalt that no pavement fits in is asphalt.** The
    // remnant of a small island the returns did not quite close, a narrow
    // fork closed at both ends: `room` paves a hole as a traffic island
    // only if it can hold the narrowest pavement ([`crate::standard::wide_enough`],
    // the same test), so anything thinner was left bare ground between two
    // carriageways. Filled before the facade cut, so a building standing in
    // it still wins. Only the hole goes; no other boundary moves — a closing
    // over the whole surface did the same and re-rounded every kerb in the
    // box (mesh slivers 71 k → 110 k, crack 8 → 93 m).
    let mut filled = 0usize;
    // The narrow gaps first: an open gap closes into a hole as often as not
    // once they are paved, and the holes are asked next.
    let joined = poly::union_all(&parts);
    let gaps = narrow_gaps(&joined);
    let gap_m2 = poly::area(&poly::difference(&poly::union_all(&gaps), &joined));
    let open: Shapes = poly::union_of(&[&joined, &gaps])
        .into_iter()
        .map(|shape| {
            let mut rings = shape.into_iter();
            let outer = rings.next().expect("a region has an outer ring");
            let mut kept = vec![outer];
            for hole in rings {
                let island = vec![poly::oriented(hole.clone(), true)];
                if crate::standard::wide_enough(&island) {
                    kept.push(hole);
                } else {
                    filled += 1;
                }
            }
            kept
        })
        .collect();
    let carriageway = facade.asphalt(&open);
    // The pavement, laid back outside wherever the asphalt grew into it —
    // `fillet`'s rule, read off the result rather than off what a closing
    // added: the walk the kerb step left was already cut by the old asphalt,
    // so what the new asphalt covers of it is exactly what it took.
    let mut out = Surface {
        carriageway,
        walk: Vec::new(),
        spanned: surface.spanned.clone(),
        ballast: surface.ballast.clone(),
    };
    let laid_back = poly::dilate(&poly::intersect(&out.carriageway, &k.surface.walk), crate::standard::WALK_MIN_M);
    // **And forward, where the asphalt drew back from it.** The old surface
    // was ribbons with round caps unioned; where the explicit kerb stands
    // inside the old one — a cap's lobe the legs do not draw, a return
    // smaller than the closing's — a sidewalk the kerb step stood against
    // the old kerb now stands off the new one, and the strip between is
    // bare ground. What the old asphalt covered within the narrowest
    // pavement of the walk is the walk's.
    let drawn_back = poly::difference(&surface.carriageway, &out.carriageway);
    // Lapped by [`LAP_M`]: the strip meets the walk along the old kerb, and
    // a union keeps shapes that only touch apart — as loose lobes. The
    // asphalt cut below takes back what laps onto the road.
    let followed = poly::dilate(
        &poly::intersect(&drawn_back, &poly::dilate(&k.surface.walk, crate::standard::WALK_MIN_M)),
        LAP_M,
    );
    let senior = out.senior();
    let pavement = facade.pavement(&poly::union_of(&[&k.surface.walk, &laid_back, &followed]), &senior);
    let bare = crate::gap::Bare::new(&senior, &pavement, &facade.footprints);
    let (gaps, gap_of) = crate::gap::kerb_gaps(&out.carriageway, &bare, &k.attached);
    let gap_n = gaps.len();
    out.walk = pavement;
    let carriageway = &out.carriageway;
    let summary = Summary::new()
        .with("junctions", junctions.len())
        .with("legs", junctions.iter().map(|j| j.legs.len()).sum::<usize>())
        .with("edges", edges.len())
        .with("returns", tally.returns)
        .with("mitres", tally.mitres)
        .with("rounds", tally.rounds)
        .with("clamped", tally.clamped)
        .with("far", tally.far)
        .with("bent", tally.bent)
        .with("short", tally.short)
        .with("tangled", tally.tangled)
        .with("landed", tally.landed)
        .with("bends", tally.bends)

        .with_m2("junction_m2", junctions.iter().map(|j| poly::area(&j.shape)).sum::<f64>() + 0.0)
        .with("islands", filled)
        .with_m2("gap_m2", gap_m2)
        .with_regions("carriageway", carriageway)
        .with_m2("pavement_m2", poly::area(&out.walk))
        .with_m2("followed_m2", poly::area(&followed))
        .with_share("kerb_gap", gap_n, gap_of);
    (Legs { junctions, edges, surface: out, gaps }, summary)
}

/// `pts` without consecutive repeats.
fn dedup(pts: &[Pt]) -> Vec<Pt> {
    let mut out: Vec<Pt> = Vec::with_capacity(pts.len());
    for &p in pts {
        if out.last().is_none_or(|q| (q[0] - p[0]).hypot(q[1] - p[1]) > 1e-9) {
            out.push(p);
        }
    }
    out
}

/// Two consecutive vertices of a piece closer than this are one, in metres:
/// the connector snap. The source carries such twins — `689f09` on the loop
/// box has two vertices 3 mm apart that round to two connectors — and
/// between them is an edge whose direction is noise, which the kerb walk
/// followed back up the road it came from.
const TWIN_M: f64 = 0.01;

/// Collapses every pair of consecutive twins in `pieces` to one vertex:
/// the one more pieces share, so a node survives it, else the first.
fn close_twins(pieces: &mut [(usize, Vec<Pt>)]) {
    let mut shared: HashMap<(i64, i64), usize> = HashMap::new();
    for (_, pts) in pieces.iter() {
        for &p in pts {
            *shared.entry(connector(p)).or_default() += 1;
        }
    }
    for (_, pts) in pieces.iter_mut() {
        let mut out: Vec<Pt> = Vec::with_capacity(pts.len());
        for &p in pts.iter() {
            match out.last_mut() {
                Some(q) if (q[0] - p[0]).hypot(q[1] - p[1]) < TWIN_M => {
                    if shared[&connector(p)] > shared[&connector(*q)] {
                        *q = p;
                    }
                }
                _ => out.push(p),
            }
        }
        // A piece that collapsed to one point keeps its two ends apart.
        if out.len() >= 2 {
            *pts = out;
        }
    }
}

/// A leg end at a node: which edge, and whether the edge leaves the node at
/// its start.
type End = (usize, bool);

/// The pieces cut at their nodes, and every node's leg ends, keyed by
/// connector.
fn cut(all: &[&Polyline2], ground: usize, pieces: &[(usize, Vec<Pt>)]) -> (Vec<PieceEdge>, HashMap<(i64, i64), Vec<End>>) {
    // A connector two piece vertices share is a node: two pieces, an end on
    // another's interior, or a closed way meeting itself. The interior
    // vertex of one piece alone is only a bend.
    let mut incidences: HashMap<(i64, i64), usize> = HashMap::new();
    for (_, pts) in pieces {
        for &p in pts {
            *incidences.entry(connector(p)).or_default() += 1;
        }
    }
    let is_node = |p: Pt| incidences[&connector(p)] >= 2;
    let mut edges = Vec::new();
    let mut nodes: HashMap<(i64, i64), Vec<End>> = HashMap::new();
    for (piece, pts) in pieces {
        let n = pts.len();
        let mut start = 0;
        for i in 1..n {
            if i < n - 1 && !is_node(pts[i]) {
                continue;
            }
            let node = [is_node(pts[start]), is_node(pts[i])];
            let e = edges.len();
            if node[0] {
                nodes.entry(connector(pts[start])).or_default().push((e, true));
            }
            if node[1] {
                nodes.entry(connector(pts[i])).or_default().push((e, false));
            }
            edges.push(PieceEdge {
                piece: *piece,
                pts: pts[start..=i].to_vec(),
                width_m: all[*piece].width_m,
                trim: [0.0, 0.0],
                node,
                ground: *piece < ground,
            });
            start = i;
        }
    }
    (edges, nodes)
}

/// The step the carriageway's edge is walked in for [`narrow_gaps`], in
/// metres.
const GAP_STEP_M: f64 = 0.25;

/// **A gap between two carriageways that no pavement fits in is asphalt**,
/// as quads: from every station on the edge of `carriageway`, a ray out to
/// [`crate::standard::PAVEMENT_MIN_M`]; where two consecutive stations both
/// meet the carriageway again, the quad between their chords, lapped by
/// [`LAP_M`] at both ends so the union joins it.
///
/// It is the open cousin of the narrow hole. Two carriageways that converge
/// without ever sharing a vertex — a pair of one-way streets that run into
/// one — leave a thin V between them with no junction to pave it, and a
/// crossing mapped across it is left a scrap a hand wide, which `room`
/// drops and the kerbs beside it read as bare ground. Only the gap is
/// paved; no other boundary moves, which is what a closing over the whole
/// surface could not promise.
fn narrow_gaps(carriageway: &Shapes) -> Shapes {
    let reach = crate::standard::PAVEMENT_MIN_M;
    // Every edge on a grid, for the rays.
    let cell = 2.0;
    let mut grid: HashMap<(i32, i32), Vec<(Pt, Pt)>> = HashMap::new();
    for ring in carriageway.iter().flatten() {
        for i in 0..ring.len() {
            let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
            for c in poly::cells_over([a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])], cell) {
                grid.entry(c).or_default().push((a, b));
            }
        }
    }
    let hit = |p: Pt, n: Pt| -> Option<Pt> {
        let o = add(p, n, 1e-3);
        let e = add(p, n, reach);
        let mut best: Option<(f64, Pt)> = None;
        for c in poly::cells_over([o[0].min(e[0]), o[1].min(e[1]), o[0].max(e[0]), o[1].max(e[1])], cell) {
            for &(a, b) in grid.get(&c).into_iter().flatten() {
                let (r, v) = ([e[0] - o[0], e[1] - o[1]], [b[0] - a[0], b[1] - a[1]]);
                let den = cross(r, v);
                if den.abs() < 1e-12 {
                    continue;
                }
                let w = [a[0] - o[0], a[1] - o[1]];
                let (t, u) = (cross(w, v) / den, cross(w, r) / den);
                if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) && best.is_none_or(|(bt, _)| t < bt) {
                    best = Some((t, add(o, r, t)));
                }
            }
        }
        best.map(|(_, q)| q)
    };
    let mut quads: Shapes = Vec::new();
    for ring in carriageway.iter().flatten() {
        let n = ring.len();
        // Stations along the ring, each with its outward normal: the region
        // lies left of every ring, so outward is the right-hand normal.
        let mut stations: Vec<(Pt, Pt)> = Vec::new();
        for i in 0..n {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            let l = (b[0] - a[0]).hypot(b[1] - a[1]);
            if l <= 0.0 {
                continue;
            }
            let t = [(b[0] - a[0]) / l, (b[1] - a[1]) / l];
            let out = [t[1], -t[0]];
            let k = (l / GAP_STEP_M).ceil() as usize;
            for j in 0..k {
                stations.push((add(a, t, l * (j as f64 + 0.5) / k as f64), out));
            }
        }
        let chords: Vec<Option<Pt>> = stations.iter().map(|&(p, out)| hit(p, out)).collect();
        let m = stations.len();
        for i in 0..m {
            let j = (i + 1) % m;
            let (Some(hi), Some(hj)) = (chords[i], chords[j]) else {
                continue;
            };
            // Two chords landing far apart are not one gap: a ray past a
            // corner of the far side.
            if (hi[0] - hj[0]).hypot(hi[1] - hj[1]) > 4.0 * GAP_STEP_M {
                continue;
            }
            let ((pi, ni), (pj, nj)) = (stations[i], stations[j]);
            let quad = vec![add(pi, ni, -LAP_M), add(pj, nj, -LAP_M), add(hj, nj, LAP_M), add(hi, ni, LAP_M)];
            if let Some(q) = poly::ccw(quad) {
                quads.push(vec![q]);
            }
        }
    }
    poly::union_all(&quads)
}

/// A landed end this close to one of its host's vertices goes to that
/// vertex rather than to a new one, in metres: two nodes a few centimetres
/// apart would make two junctions of one.
const LAND_SNAP_M: f64 = 0.5;

/// The cell of the segment grid [`land`] searches, in metres: past the
/// widest half-width a road takes, so a host is found in the nine cells
/// round the end.
const LAND_CELL_M: f64 = 16.0;

/// Carries every end that lands on another road to it, and returns how many
/// did.
///
/// An end *lands* when it is free — no other piece has a vertex at its
/// connector — and lies within the half-width of another piece's
/// centreline. Overture connects most junctions, but not all: a service
/// road mapped to stop on the side of the street it leaves. The union of
/// ribbons makes that a junction anyway, with notches the fillet then
/// rounds, and a construction from legs has to make it one too. So the
/// host gets a vertex at the end's foot — or its nearest vertex, within
/// [`LAND_SNAP_M`] — and the end is carried there, and from then on the two
/// share a connector like any other junction. Ends are matched against the
/// pieces as they came, so two ends landing on each other both land.
fn land(all: &[&Polyline2], pieces: &mut [(usize, Vec<Pt>)]) -> usize {
    let mut incidences: HashMap<(i64, i64), usize> = HashMap::new();
    for (_, pts) in pieces.iter() {
        for &p in pts {
            *incidences.entry(connector(p)).or_default() += 1;
        }
    }
    let mut grid: HashMap<(i32, i32), Vec<(usize, usize)>> = HashMap::new();
    for (k, (_, pts)) in pieces.iter().enumerate() {
        for i in 0..pts.len() - 1 {
            let (a, b) = (pts[i], pts[i + 1]);
            for c in poly::cells_over([a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])], LAND_CELL_M) {
                grid.entry(c).or_default().push((k, i));
            }
        }
    }
    // (host, segment, t) → the foot, and (piece, at its start) → the foot.
    let mut inserts: Vec<(usize, usize, f64, Pt)> = Vec::new();
    let mut carried: Vec<(usize, bool, Pt)> = Vec::new();
    for (k, (_, pts)) in pieces.iter().enumerate() {
        let n = pts.len();
        for (at_start, p) in [(true, pts[0]), (false, pts[n - 1])] {
            if incidences[&connector(p)] != 1 {
                continue;
            }
            let (c, r) = poly::cell_of(p, LAND_CELL_M);
            let mut best: Option<(f64, usize, usize)> = None;
            for dc in -1..=1 {
                for dr in -1..=1 {
                    for &(h, i) in grid.get(&(c + dc, r + dr)).into_iter().flatten() {
                        if h == k {
                            continue;
                        }
                        let q = &pieces[h].1;
                        let d = poly::segment_distance(q[i], q[i + 1], p);
                        if d <= all[pieces[h].0].width_m / 2.0 && best.is_none_or(|(b, ..)| d < b) {
                            best = Some((d, h, i));
                        }
                    }
                }
            }
            let Some((_, h, i)) = best else {
                continue;
            };
            let q = &pieces[h].1;
            let foot = poly::nearest_on_segment(q[i], q[i + 1], p);
            let near = |v: Pt| (v[0] - foot[0]).hypot(v[1] - foot[1]) <= LAND_SNAP_M;
            let to = if near(q[i]) {
                q[i]
            } else if near(q[i + 1]) {
                q[i + 1]
            } else {
                let l = seg(q, i);
                let t = ((foot[0] - q[i][0]).hypot(foot[1] - q[i][1]) / l).clamp(0.0, 1.0);
                inserts.push((h, i, t, foot));
                foot
            };
            carried.push((k, at_start, to));
        }
    }
    // Insert from the back, so the segment indices still name the segments
    // they were found on.
    inserts.sort_by(|a, b| (b.0, b.1).cmp(&(a.0, a.1)).then(b.2.partial_cmp(&a.2).expect("finite")));
    for (h, i, _, foot) in inserts {
        pieces[h].1.insert(i + 1, foot);
    }
    for &(k, at_start, to) in &carried {
        let pts = &mut pieces[k].1;
        if at_start {
            pts.insert(0, to);
        } else {
            pts.push(to);
        }
        *pts = dedup(pts);
    }
    carried.len()
}

/// The length of the segment `i` of `pts`.
fn seg(pts: &[Pt], i: usize) -> f64 {
    (pts[i + 1][0] - pts[i][0]).hypot(pts[i + 1][1] - pts[i][1])
}

fn left(u: Pt) -> Pt {
    [-u[1], u[0]]
}

fn add(a: Pt, b: Pt, s: f64) -> Pt {
    [a[0] + b[0] * s, a[1] + b[1] * s]
}

fn dot(a: Pt, b: Pt) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross(a: Pt, b: Pt) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// How one side of a junction closes: the sector between a leg's left kerb
/// and the next leg's right kerb, counter-clockwise. `s` and `t` are where
/// it leaves each kerb, in metres of arc along that leg from the node; `arc`
/// is the boundary between those two points.
struct Side {
    s: f64,
    t: f64,
    arc: Vec<Pt>,
}

/// One side of a leg: its centreline oriented away from the node, extended
/// straight back past the node by [`BACK_M`] and carried on past its far node
/// by the kerb walk ([`walk`]), offset to the left where `side` is positive by
/// each stretch's own half-width plus `extra` — with each vertex's arc along
/// the centreline, so a point of the kerb and the centreline point it stands
/// beside share an arc, and a trim at that arc cuts both.
struct Kerb {
    /// The offset polyline, with each vertex's arc: what two kerbs are
    /// crossed on. Where two stretches of different widths meet, the vertex
    /// is doubled, both copies at one arc.
    pts: Vec<Pt>,
    arc: Vec<f64>,
    /// The centreline stretches, each from arc `s0` to `s1` and offset by
    /// `d`: what a point at an arc is read from.
    segs: Vec<Stretch>,
}

/// One segment of a kerb's centreline and how far the kerb stands off it.
struct Stretch {
    a: Pt,
    b: Pt,
    s0: f64,
    s1: f64,
    d: f64,
}

/// How far a kerb is extended back past its node, in metres: two legs of
/// different widths meeting near-collinear have kerbs that meet behind it.
const BACK_M: f64 = 30.0;

/// Halvings of the radius a clamped return is found in: a millimetre of an
/// 8 m radius.
const CLAMP_STEPS: usize = 14;

/// The longest a mitre at a kerb vertex is let grow, in offsets: a sharper
/// bend than this is bevelled by the clamp rather than spiking.
const MITRE_MAX: f64 = 4.0;

impl Kerb {
    /// The kerb along `path` — the centreline from the node, each point
    /// after the first with the half-width of the stretch that ends there.
    fn new(path: &[(Pt, f64)], side: f64, extra: f64) -> Kerb {
        let u0 = poly::unit([path[1].0[0] - path[0].0[0], path[1].0[1] - path[0].0[1]]);
        let mut segs = vec![Stretch {
            a: add(path[0].0, u0, -BACK_M),
            b: path[0].0,
            s0: -BACK_M,
            s1: 0.0,
            d: side * (path[1].1 + extra),
        }];
        for k in 1..path.len() {
            let (a, b) = (path[k - 1].0, path[k].0);
            let s0 = segs[k - 1].s1;
            segs.push(Stretch { a, b, s0, s1: s0 + (b[0] - a[0]).hypot(b[1] - a[1]), d: side * (path[k].1 + extra) });
        }
        let off = |g: &Stretch, p: Pt| add(p, left(poly::unit([g.b[0] - g.a[0], g.b[1] - g.a[1]])), g.d);
        let mut pts = vec![off(&segs[0], segs[0].a)];
        let mut arc = vec![segs[0].s0];
        for k in 1..segs.len() {
            let (g, h) = (&segs[k - 1], &segs[k]);
            let (na, nb) = (
                left(poly::unit([g.b[0] - g.a[0], g.b[1] - g.a[1]])),
                left(poly::unit([h.b[0] - h.a[0], h.b[1] - h.a[1]])),
            );
            if (g.d - h.d).abs() < 1e-9 {
                // One offset: the mitre, clamped.
                let m = poly::unit([na[0] + nb[0], na[1] + nb[1]]);
                let scale = 1.0 / dot(m, nb).max(1.0 / MITRE_MAX);
                pts.push(add(h.a, m, h.d * scale));
                arc.push(h.s0);
            } else {
                // Two: where the width changes the kerb steps, both ends at
                // one arc.
                pts.push(off(g, g.b));
                pts.push(off(h, h.a));
                arc.extend([h.s0, h.s0]);
            }
        }
        let last = segs.last().expect("a kerb has a stretch");
        pts.push(off(last, last.b));
        arc.push(last.s1);
        Kerb { pts, arc, segs }
    }

    /// The stretch `s` falls on, the first or the last extended.
    fn stretch(&self, s: f64) -> &Stretch {
        let i = self.segs.iter().take_while(|g| g.s1 < s).count().min(self.segs.len() - 1);
        &self.segs[i]
    }

    /// The kerb at `s`: square off the centreline from the point at that
    /// arc, which is where a buffer's butt end cut there puts its corner.
    /// Interpolated between the mitred vertices instead, a mouth on a curved
    /// leg was skewed against the square end of the edge it hands over to,
    /// and the two left a hairline between them.
    fn at(&self, s: f64) -> Pt {
        let g = self.stretch(s);
        let l = g.s1 - g.s0;
        let f = if l > 0.0 { (s - g.s0) / l } else { 0.0 };
        let n = left(poly::unit([g.b[0] - g.a[0], g.b[1] - g.a[1]]));
        add([g.a[0] + (g.b[0] - g.a[0]) * f, g.a[1] + (g.b[1] - g.a[1]) * f], n, g.d)
    }

    /// The unit direction of the kerb at `s`, away from the node.
    fn direction(&self, s: f64) -> Pt {
        let g = self.stretch(s);
        poly::unit([g.b[0] - g.a[0], g.b[1] - g.a[1]])
    }

    /// The kerb from `s0` to `s1`, either way: its two ends and every vertex
    /// strictly between.
    fn between(&self, s0: f64, s1: f64) -> Vec<Pt> {
        let (lo, hi) = (s0.min(s1), s0.max(s1));
        let mut inner: Vec<Pt> =
            (1..self.arc.len() - 1).filter(|&i| self.arc[i] > lo && self.arc[i] < hi).map(|i| self.pts[i]).collect();
        if s0 > s1 {
            inner.reverse();
        }
        let mut out = vec![self.at(s0)];
        out.extend(inner);
        out.push(self.at(s1));
        out
    }
}

/// How far a kerb is walked past its leg's far node, in metres: past the
/// longest return (`RETURN_MAX_RADII` of the widest radius) and a road's
/// width.
const WALK_M: f64 = 40.0;

/// A turn this close to none or a full turn, in radians, is an edge doubling
/// back on the one the walk arrived by.
const BACK_TURN: f64 = 0.02;

/// The most nodes a kerb walk passes through.
const WALK_HOPS: usize = 4;

/// The kerb of leg `(e, at_start)` on one side, walked on past the leg's far
/// node: the points after the leg's own, each with the half-width of the
/// stretch ending there.
///
/// **A kerb does not stop at the next node.** Two nodes a few metres apart
/// each build their junction alone, and the corner between a leg of one and
/// a leg of the other — a V whose two kerbs belong to different junctions —
/// has a return only if one junction can see the other's kerb. So a leg's
/// kerb goes on as the boundary of the face on that side does: at each node
/// it takes the edge first clockwise from the one it arrived on (a left
/// kerb) or first counter-clockwise (a right kerb), which is the kerb that
/// turns the way the face turns. It stops at a free end, after
/// [`WALK_HOPS`] nodes, or past [`WALK_M`].
///
/// Merging the two nodes into one junction was tried first: the link
/// between them stops being a leg, and the return between it and an outer
/// leg is lost instead.
fn walk(
    (mut e, mut from_start): End,
    left_side: bool,
    edges: &[PieceEdge],
    nodes: &HashMap<(i64, i64), Vec<End>>,
) -> Vec<(Pt, f64)> {
    let mut out = Vec::new();
    let mut walked = 0.0;
    for _ in 0..WALK_HOPS {
        let pts = &edges[e].pts;
        let n = pts.len();
        let (end, back) = if from_start { (pts[n - 1], pts[n - 2]) } else { (pts[0], pts[1]) };
        let Some(ends) = nodes.get(&connector(end)) else {
            break;
        };
        let r = poly::unit([back[0] - end[0], back[1] - end[1]]);
        let ra = r[1].atan2(r[0]);
        let turn = |v: Pt| {
            // Clockwise from `r` for a left kerb, counter-clockwise for a
            // right one, in (0, 2π].
            let a = v[1].atan2(v[0]);
            let mut d = if left_side { ra - a } else { a - ra };
            while d <= 1e-12 {
                d += 2.0 * std::f64::consts::PI;
            }
            while d > 2.0 * std::f64::consts::PI {
                d -= 2.0 * std::f64::consts::PI;
            }
            d
        };
        let next = ends
            .iter()
            .copied()
            .filter(|&(f, s)| (f, s) != (e, !from_start))
            .map(|(f, s)| {
                let q = &edges[f].pts;
                let v = if s { [q[1][0] - q[0][0], q[1][1] - q[0][1]] } else {
                    let m = q.len();
                    [q[m - 2][0] - q[m - 1][0], q[m - 2][1] - q[m - 1][1]]
                };
                (turn(poly::unit(v)), f, s)
            })
            // An edge leaving along the one arrived on doubles back up the
            // same road: a duplicate way, not a kerb that turns.
            .filter(|&(a, ..)| a > BACK_TURN && a < 2.0 * std::f64::consts::PI - BACK_TURN)
            .min_by(|a, b| a.0.partial_cmp(&b.0).expect("finite").then((a.1, a.2).cmp(&(b.1, b.2))));
        let Some((_, f, s)) = next else {
            break;
        };
        let mut line = edges[f].pts.clone();
        if !s {
            line.reverse();
        }
        let half = edges[f].width_m / 2.0;
        for k in 1..line.len() {
            walked += seg(&line, k - 1);
            out.push((line[k], half));
            if walked >= WALK_M {
                return out;
            }
        }
        (e, from_start) = (f, s);
    }
    out
}

impl Leg {
    /// The centreline a kerb on one side is offset from: the leg's own line
    /// and the walk past it.
    fn path(&self, left_side: bool) -> Vec<(Pt, f64)> {
        let mut out: Vec<(Pt, f64)> = self.line.iter().map(|&p| (p, self.half_m)).collect();
        out.extend(self.ahead[if left_side { 0 } else { 1 }].iter().copied());
        out
    }
}

/// Where two kerbs cross: the arc along each, and the point. Of every
/// crossing, the one nearest the node by the sum of the two arcs.
fn meet(a: &Kerb, b: &Kerb) -> Option<(f64, f64, Pt)> {
    let mut best: Option<(f64, f64, Pt)> = None;
    for i in 0..a.pts.len() - 1 {
        let (p, r) = (a.pts[i], [a.pts[i + 1][0] - a.pts[i][0], a.pts[i + 1][1] - a.pts[i][1]]);
        for j in 0..b.pts.len() - 1 {
            let (q, v) = (b.pts[j], [b.pts[j + 1][0] - b.pts[j][0], b.pts[j + 1][1] - b.pts[j][1]]);
            let den = cross(r, v);
            if den.abs() < 1e-12 {
                continue;
            }
            let w = [q[0] - p[0], q[1] - p[1]];
            let (f, g) = (cross(w, v) / den, cross(w, r) / den);
            if !(0.0..=1.0).contains(&f) || !(0.0..=1.0).contains(&g) {
                continue;
            }
            let s = a.arc[i] + f * (a.arc[i + 1] - a.arc[i]);
            let t = b.arc[j] + g * (b.arc[j + 1] - b.arc[j]);
            if best.is_none_or(|(s0, t0, _)| s + t < s0 + t0) {
                best = Some((s, t, add(p, r, f)));
            }
        }
    }
    best
}

/// The junction from its leg ends.
fn junction(
    ends: &[End],
    edges: &[PieceEdge],
    nodes: &HashMap<(i64, i64), Vec<End>>,
    all: &[&Polyline2],
    tally: &mut Tally,
) -> Junction {
    let mut legs: Vec<Leg> = ends
        .iter()
        .map(|&(e, at_start)| {
            let mut line = edges[e].pts.clone();
            if !at_start {
                line.reverse();
            }
            Leg {
                edge: e,
                at_start,
                u: poly::unit([line[1][0] - line[0][0], line[1][1] - line[0][1]]),
                half_m: edges[e].width_m / 2.0,
                class: all[edges[e].piece].class.clone(),
                mouth_m: 0.0,
                line,
                ahead: [walk((e, at_start), true, edges, nodes), walk((e, at_start), false, edges, nodes)],
            }
        })
        .collect();
    let at = legs[0].line[0];
    legs.sort_by(|a, b| {
        let (x, y) = (a.u[1].atan2(a.u[0]), b.u[1].atan2(b.u[0]));
        x.partial_cmp(&y).expect("finite").then(a.edge.cmp(&b.edge)).then(a.at_start.cmp(&b.at_start))
    });
    let n = legs.len();
    // Each leg's two kerbs, left and right.
    let kerbs: Vec<[Kerb; 2]> =
        legs.iter().map(|l| [Kerb::new(&l.path(true), 1.0, 0.0), Kerb::new(&l.path(false), -1.0, 0.0)]).collect();
    // Each side between leg `i` and leg `i + 1`.
    let sides: Vec<Side> = (0..n)
        .map(|i| {
            let j = (i + 1) % n;
            side(at, &legs[i], &legs[j], &kerbs[i][0], &kerbs[j][1], tally)
        })
        .collect();
    // Each leg's mouth is where the farther of its two sides stops.
    for i in 0..n {
        legs[i].mouth_m = sides[i].s.max(sides[(i + n - 1) % n].t).max(0.0);
    }
    // The ring, counter-clockwise: at each leg its mouth from right to left,
    // then in along its left kerb to the side, the side, and out along the
    // next leg's right kerb to its mouth.
    //
    // **A mouth past the leg's own end is drawn at its far node.** The two
    // kerbs walk on into different edges there, so a line across them at
    // the mouth's arc would cut across whatever lies between those edges;
    // across the leg at its end it lies in the far junction, which paves it.
    let mouth: Vec<f64> = legs.iter().map(|l| l.mouth_m.min(poly::length(&l.line))).collect();
    let mut ring: Ring = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        ring.push(kerbs[i][1].at(mouth[i]));
        ring.extend(kerbs[i][0].between(mouth[i], sides[i].s));
        ring.extend(sides[i].arc.iter().copied());
        ring.extend(kerbs[j][1].between(sides[i].t, mouth[j]));
    }
    let ring = dedup(&ring);
    let signed = poly::ring_area(&ring);
    // **A junction is one region.** Where the ring crosses itself — a kerb
    // walking round a bend right at the node, a mitre touching the ring —
    // the union hands back a second region: a lobe of a few square
    // centimetres outside every ribbon, or three coincident points. Each was
    // a fragment of asphalt no piece claims. The junction is the largest.
    let shape: Shapes = if ring.len() >= 3 { poly::union_all(&vec![vec![ring]]) } else { Vec::new() }
        .into_iter()
        .max_by(|a, b| {
            poly::area(std::slice::from_ref(a)).partial_cmp(&poly::area(std::slice::from_ref(b))).expect("finite")
        })
        .into_iter()
        .collect();
    if (poly::area(&shape) - signed).abs() > 1e-3 * signed.abs().max(1.0) {
        tally.tangled += 1;
    }
    let reach_m = legs
        .iter()
        .zip(&kerbs)
        .zip(&mouth)
        .flat_map(|((_, k), &m)| k.iter().map(move |k| k.at(m)))
        .map(|p| (p[0] - at[0]).hypot(p[1] - at[1]))
        .fold(0.0, f64::max);
    Junction { at, legs, shape, reach_m }
}

/// The side between leg `a`'s left kerb `ka` and leg `b`'s right kerb `kb`,
/// `b` being the next leg counter-clockwise.
fn side(at: Pt, a: &Leg, b: &Leg, ka: &Kerb, kb: &Kerb, tally: &mut Tally) -> Side {
    // The sector from `a` to `b`, counter-clockwise, in (0, 2π].
    let mut theta = cross(a.u, b.u).atan2(dot(a.u, b.u));
    if theta <= 1e-9 {
        theta += 2.0 * std::f64::consts::PI;
    }
    let round = |tally: &mut Tally| {
        tally.rounds += 1;
        Side { s: 0.0, t: 0.0, arc: round_join(at, a, b, theta) }
    };
    if theta >= std::f64::consts::PI - 1e-9 {
        return round(tally);
    }
    let far = FAR_HALF_WIDTHS * (a.half_m + b.half_m);
    let Some((s, t, corner)) = meet(ka, kb).filter(|&(s, t, _)| s.abs() <= far && t.abs() <= far) else {
        tally.far += 1;
        return round(tally);
    };
    // How far the boundary turns at the corner, read off the kerbs there
    // rather than the legs' first segments: on a curved leg they differ.
    let (da, db) = (ka.direction(s), kb.direction(t));
    let turn = std::f64::consts::PI - cross(da, db).atan2(dot(da, db));
    let mitre = Side { s, t, arc: vec![corner] };
    if turn.to_degrees() < BEND_MIN_DEG {
        // A mitre is where the kerbs barely turn, so its corner stands about
        // a half-width from the node. One further off is two kerbs of
        // different widths meeting at a grazing angle — at worst a stub the
        // clip left a few centimetres long — and drawn, it is a spike.
        if (corner[0] - at[0]).hypot(corner[1] - at[1]) > MITRE_MAX * a.half_m.max(b.half_m) {
            tally.far += 1;
            return round(tally);
        }
        tally.mitres += 1;
        return mitre;
    }
    // The return: a circle of the narrower leg's radius tangent to both
    // kerbs. Its centre is where the kerbs offset by that radius meet, and
    // its tangent points stand beside the centre on each kerb — which holds
    // on a curved leg as on a straight one.
    let r0 = width::fillet_m(&a.class).min(width::fillet_m(&b.class));
    let (pa, pb) = (a.path(true), b.path(false));
    let centre = |r: f64| meet(&Kerb::new(&pa, 1.0, r), &Kerb::new(&pb, -1.0, r));
    // **A return runs at most [`RETURN_MAX_RADII`] radii along
    // either kerb from the corner**, and past that the radius shrinks until
    // it fits. The reach is read off the tangent points themselves, not off
    // the turn at the corner: a leg that bends a few metres out turns a
    // right-angled corner into a sliver of a fork, and the turn at the
    // corner cannot see that.
    let reach = RETURN_MAX_RADII * r0;
    let fits = |c: Option<(f64, f64, Pt)>| c.is_some_and(|(sc, tc, _)| sc - s <= reach && tc - t <= reach);
    let mut r = r0;
    let mut found = centre(r);
    if !fits(found) {
        tally.clamped += 1;
        let (mut lo, mut hi) = (0.0, r0);
        for _ in 0..CLAMP_STEPS {
            let mid = 0.5 * (lo + hi);
            if fits(centre(mid)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        r = lo;
        found = if r > 0.0 { centre(r) } else { None };
    }
    let Some((sc, tc, c)) = found else {
        tally.mitres += 1;
        return mitre;
    };
    tally.returns += 1;
    Side { s: sc, t: tc, arc: arc(c, r, ka.at(sc), kb.at(tc)) }
}

/// **A way that turns sharply at one of its own vertices gets a return
/// there too.** No junction is there, so nothing above rounds it, and the
/// ribbon's inner kerbs meet in a sharp corner no real kerb has: a street
/// turning a right angle at a vertex, a hairpin drawn as one. The inside of
/// every vertex of `pts` between arc `a` and `b` that turns at least
/// [`BEND_MIN_DEG`] gets the return a junction corner of that turn would get
/// — radius `r0`, the way's own [`width::fillet_m`], tangent to both inner
/// kerbs, its tangent length at most [`RETURN_MAX_RADII`] radii and at most
/// the kerb the vertex has to itself, shrinking the radius to fit. The
/// outside is the ribbon's round join, as before.
///
/// Each return laps [`LAP_M`] into the road, so the union joins it to the
/// ribbon rather than leaving the kerb line between them.
fn bend_returns(pts: &[Pt], h: f64, r0: f64, a: f64, b: f64, tally: &mut Tally) -> Shapes {
    let n = pts.len();
    let mut at = vec![0.0; n];
    for i in 1..n {
        at[i] = at[i - 1] + seg(pts, i - 1);
    }
    let mut out: Shapes = Vec::new();
    for i in 1..n.saturating_sub(1) {
        if !(at[i] > a && at[i] < b) {
            continue;
        }
        let (l1, l2) = (seg(pts, i - 1), seg(pts, i));
        if !(l1 > 0.0 && l2 > 0.0) {
            continue;
        }
        let u1 = [(pts[i][0] - pts[i - 1][0]) / l1, (pts[i][1] - pts[i - 1][1]) / l1];
        let u2 = [(pts[i + 1][0] - pts[i][0]) / l2, (pts[i + 1][1] - pts[i][1]) / l2];
        let turn = cross(u1, u2).atan2(dot(u1, u2));
        if turn.abs().to_degrees() < BEND_MIN_DEG {
            continue;
        }
        // The inside of the turn is the side it turns to; its kerbs' outward
        // normals point into the ground there.
        let sigma = turn.signum();
        let (n1, n2) = (add([0.0, 0.0], left(u1), sigma), add([0.0, 0.0], left(u2), sigma));
        let half_tan = (turn.abs() / 2.0).tan();
        // The kerb this vertex has to itself on either side: the whole
        // segment at an end, half of it where the next vertex may bend too,
        // and never past the trims; less what the inner corner itself takes.
        let before = if i == 1 { at[i] - a.max(0.0) } else { (0.5 * l1).min(at[i] - a) };
        let after = if i + 2 == n { b.min(at[n - 1]) - at[i] } else { (0.5 * l2).min(b - at[i]) };
        let room = before.min(after) - h * half_tan;
        let reach = (RETURN_MAX_RADII * r0).min(room);
        if !(reach > 0.0) {
            continue;
        }
        let r = r0.min(reach / half_tan);
        let len = r * half_tan;
        // The inner corner, where the two inner kerbs meet.
        let corner = add(pts[i], add(n1, n2, 1.0), h / (1.0 + dot(n1, n2)));
        let (t1, t2) = (add(corner, u1, -len), add(corner, u2, len));
        let centre = add(t1, n1, r);
        let lap = |p: Pt, n: Pt| add(p, n, -LAP_M);
        let mut ring = vec![lap(corner, add(n1, n2, 1.0)), lap(t1, n1)];
        ring.extend(arc(centre, r, t1, t2));
        ring.push(lap(t2, n2));
        out.push(vec![poly::oriented(ring, true)]);
        tally.bends += 1;
    }
    out
}

/// The round join between leg `a`'s left foot and leg `b`'s right foot, about
/// the node, across a sector of `theta`: the arc a ribbon's round join or cap
/// draws there, its radius going from one half-width to the other.
fn round_join(at: Pt, a: &Leg, b: &Leg, theta: f64) -> Vec<Pt> {
    // The boundary runs counter-clockwise round the junction, and on a
    // convex side that is counter-clockwise round the node too: from
    // `left(a.u)` on by the part of the sector past half a turn.
    let from = left(a.u);
    let sweep = theta - std::f64::consts::PI;
    let a0 = from[1].atan2(from[0]);
    let steps = ((sweep / poly::ARC_STEP).ceil() as usize).max(1);
    (0..=steps)
        .map(|k| {
            let f = k as f64 / steps as f64;
            let ang = a0 + f * sweep;
            let rad = a.half_m + f * (b.half_m - a.half_m);
            [at[0] + rad * ang.cos(), at[1] + rad * ang.sin()]
        })
        .collect()
}

/// The arc about `centre` of radius `r` from `from` to `to`, the short way,
/// at the kernel's angular step.
fn arc(centre: Pt, r: f64, from: Pt, to: Pt) -> Vec<Pt> {
    let a0 = (from[1] - centre[1]).atan2(from[0] - centre[0]);
    let a1 = (to[1] - centre[1]).atan2(to[0] - centre[0]);
    let mut sweep = a1 - a0;
    while sweep > std::f64::consts::PI {
        sweep -= 2.0 * std::f64::consts::PI;
    }
    while sweep < -std::f64::consts::PI {
        sweep += 2.0 * std::f64::consts::PI;
    }
    let steps = ((sweep.abs() / poly::ARC_STEP).ceil() as usize).max(1);
    let mut out = vec![from];
    out.extend((1..steps).map(|k| {
        let ang = a0 + sweep * k as f64 / steps as f64;
        [centre[0] + r * ang.cos(), centre[1] + r * ang.sin()]
    }));
    out.push(to);
    out
}

/// The part of the polyline `pts` between arc lengths `a` and `b`, extended
/// straight past either end where `a` is negative or `b` past its length.
fn between(pts: &[Pt], a: f64, b: f64) -> Vec<Pt> {
    let n = pts.len();
    let mut arc = vec![0.0];
    for i in 0..n - 1 {
        arc.push(arc[i] + seg(pts, i));
    }
    let at = |s: f64| {
        // The segment `s` falls on, the first or last extended.
        let i = (1..n - 1).take_while(|&i| arc[i] < s).count();
        let l = arc[i + 1] - arc[i];
        let f = if l > 0.0 { (s - arc[i]) / l } else { 0.0 };
        [pts[i][0] + (pts[i + 1][0] - pts[i][0]) * f, pts[i][1] + (pts[i + 1][1] - pts[i][1]) * f]
    };
    let mut out = vec![at(a)];
    out.extend((1..n - 1).filter(|&i| arc[i] > a && arc[i] < b).map(|i| pts[i]));
    out.push(at(b));
    dedup(&out)
}

#[cfg(test)]
mod tests {
    use crate::pipeline::tests::{built, plan};
    use crate::step::Step;
    use crate::world::World;

    use super::*;

    /// A flat world with the network of `spec`, through this step.
    fn world(spec: &str) -> (World, Summary) {
        let (w, ran) = built("flat", spec, None, 100.0, &plan(Step::Legs));
        (w, ran.last())
    }

    #[test]
    fn between_cuts_and_extends() {
        let pts = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]];
        assert_eq!(between(&pts, 2.0, 15.0), vec![[2.0, 0.0], [10.0, 0.0], [10.0, 5.0]]);
        // Extended along the first and last segments, so their ends are no
        // longer vertices.
        assert_eq!(between(&pts, -1.0, 21.0), vec![[-1.0, 0.0], [10.0, 0.0], [10.0, 11.0]]);
        assert_eq!(between(&pts, 11.0, 12.0), vec![[10.0, 1.0], [10.0, 2.0]]);
    }

    #[test]
    fn a_cross_is_one_junction_with_four_returns() {
        let (w, s) = world("net:cross?len=200");
        let l = w.legs.as_ref().unwrap();
        assert_eq!(l.junctions.len(), 1, "{s}");
        let j = &l.junctions[0];
        assert_eq!(j.legs.len(), 4);
        assert_eq!(s.num("returns"), 4.0, "{s}");
        // A 4 m return at a right angle: the mouth is the kerb's 2.75 plus
        // the tangent length, 4 m.
        assert!(j.legs.iter().all(|l| (l.mouth_m - 6.75).abs() < 1e-9), "{:?}", j.legs);
        // The same area the closing gives: two straights less the overlap,
        // plus four r²(1 − π/4).
        let exact = 2.0 * 200.0 * 5.5 - 5.5 * 5.5 + 4.0 * 16.0 * (1.0 - std::f64::consts::PI / 4.0);
        let a = poly::area(&l.surface.carriageway);
        assert!((a - exact).abs() < 0.01 * exact, "{a} vs {exact}: {s}");
        assert_eq!(l.surface.carriageway.len(), 1, "{s}");
        assert_eq!(l.surface.carriageway[0].len(), 1, "no holes: {s}");
    }

    #[test]
    fn a_tee_has_two_returns_and_a_straight_side() {
        let (w, s) = world("net:tee?len=200");
        let l = w.legs.as_ref().unwrap();
        assert_eq!(l.junctions.len(), 1, "{s}");
        assert_eq!(s.num("returns"), 2.0, "{s}");
        assert_eq!(s.num("rounds"), 1.0, "{s}");
        // The corner point of each notch is asphalt, and past the arc is not.
        let c = 2.75 + 4.0 * (1.0 - 1.0 / 2.0f64.sqrt()) - 0.05;
        assert!(poly::contains(&l.surface.carriageway, [c, c]), "{s}");
        assert!(poly::contains(&l.surface.carriageway, [-c, c]), "{s}");
        assert!(!poly::contains(&l.surface.carriageway, [6.75, 6.75]), "{s}");
    }

    /// **A junction standing on a deck is a junction.** The three legs of
    /// `tee?span=` meet on a bridge, so the node's legs are all span edges:
    /// they shape the polygon, and are left to the `sheet` step to pave.
    #[test]
    fn a_junction_on_a_deck_is_built_from_its_spans() {
        let (w, s) = world("net:tee?span=0.3&kind=bridge&len=200");
        let l = w.legs.as_ref().unwrap();
        assert_eq!(l.junctions.len(), 4, "the deck junction and three handovers: {s}");
        let deck = l.junctions.iter().find(|j| j.at == [0.0, 0.0]).expect("the node at the origin");
        assert_eq!(deck.legs.len(), 3, "{s}");
        assert!(deck.legs.iter().all(|g| !l.edges[g.edge].ground), "{s}");
        assert_eq!(s.num("returns"), 2.0, "{s}");
        // Its returns are paved as the fillet paves them: into the carriageway.
        let c = 2.75 + 4.0 * (1.0 - 1.0 / 2.0f64.sqrt()) - 0.05;
        assert!(poly::contains(&l.surface.carriageway, [c, c]), "{s}");
        // And the deck itself is not: it is the sheet's.
        assert!(!poly::contains(&l.surface.carriageway, [0.0, 20.0]), "{s}");
    }

    /// **An end on the side of a road is a junction the source did not
    /// connect.** A service road stops a metre into the street, with no
    /// vertex there: it is carried to its foot, the street is given one,
    /// and the two meet in a tee with its two returns.
    #[test]
    fn an_end_that_lands_on_a_road_is_a_junction() {
        let mut w = built("flat", "net:straight?len=200", None, 100.0, &plan(Step::Facade)).0;
        let mut drive = w.network().unwrap().plan[0].clone();
        drive.id = "drive".into();
        drive.class = "service".into();
        drive.width_m = width::of("service", "");
        drive.pts = vec![[5.0, 60.0], [5.0, 1.0]];
        w.partition.as_mut().unwrap().network.plan.push(drive);
        let (l, s) = pave(&w);
        assert_eq!(s.num("landed"), 1.0, "{s}");
        assert_eq!(l.junctions.len(), 1, "{s}");
        assert_eq!(l.junctions[0].at, [5.0, 0.0], "{s}");
        assert_eq!(l.junctions[0].legs.len(), 3, "{s}");
        assert_eq!(s.num("returns"), 2.0, "{s}");
        // The service road's 3 m return, at the corner west of it.
        let c = 2.75 + 3.0 * (1.0 - 1.0 / 2.0f64.sqrt()) - 0.05;
        assert!(poly::contains(&l.surface.carriageway, [5.0 - 1.5 - c + 2.75, c]), "{s}");
    }

    /// **A return reaches past the next node.** The tee's east leg is cut
    /// 3 m out, as a mapper cuts a way at a driveway: the return between it
    /// and the north leg is 4 m, so its tangent point lies on the next edge.
    /// The kerb walks on through the cut, and the return is the tee's.
    #[test]
    fn a_return_reaches_past_the_next_node() {
        let mut w = built("flat", "net:tee?len=200", None, 100.0, &plan(Step::Facade)).0;
        let plan_ = &mut w.partition.as_mut().unwrap().network.plan;
        let i = plan_.iter().position(|p| p.id == "road-e").unwrap();
        let mut far = plan_[i].clone();
        far.id = "road-e2".into();
        far.pts = vec![[3.0, 0.0], [100.0, 0.0]];
        plan_[i].pts = vec![[0.0, 0.0], [3.0, 0.0]];
        plan_.push(far);
        let (l, s) = pave(&w);
        assert_eq!(s.num("returns"), 2.0, "{s}");
        let c = 2.75 + 4.0 * (1.0 - 1.0 / 2.0f64.sqrt()) - 0.05;
        assert!(poly::contains(&l.surface.carriageway, [c, c]), "the east return: {s}");
        assert!(poly::contains(&l.surface.carriageway, [-c, c]), "the west return: {s}");
    }

    /// **A gap between two carriageways that no pavement fits in is
    /// asphalt.** A second street converges on the first without ever
    /// sharing a vertex: the ground between their kerbs narrows from 2.5 m to
    /// 0.3 m, and where it is under the narrowest pavement it is paved.
    #[test]
    fn a_gap_no_pavement_fits_in_is_asphalt() {
        let mut w = built("flat", "net:straight?len=200", None, 100.0, &plan(Step::Facade)).0;
        let plan_ = &mut w.partition.as_mut().unwrap().network.plan;
        let mut other = plan_[0].clone();
        other.id = "other".into();
        other.pts = vec![[-100.0, 8.0], [100.0, 5.8]];
        plan_.push(other);
        let (l, s) = pave(&w);
        assert!(poly::contains(&l.surface.carriageway, [80.0, 3.0]), "{s}");
        assert!(!poly::contains(&l.surface.carriageway, [-50.0, 3.7]), "wide enough to stay ground: {s}");
    }

    /// A pavement a return eats into is laid back outside it, as the fillet
    /// lays it back, so the sidewalk wraps the corner.
    #[test]
    fn the_pavement_wraps_a_return() {
        let (w, s) = world("net:tee?d=8&len=200");
        let l = w.legs.as_ref().unwrap();
        assert!(poly::intersect(&l.surface.walk, &l.surface.carriageway).is_empty(), "{s}");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
    }

    #[test]
    fn a_dual_carriageway_has_no_junction() {
        let (w, s) = world("net:dual?gap=4&len=200");
        assert_eq!(s.num("junctions"), 0.0, "{s}");
        assert_eq!(w.legs.as_ref().unwrap().surface.carriageway.len(), 2, "the median stays: {s}");
    }

    // The fillet's specimens, carried over when it was deleted: what its
    // tests held of the kerb returns, asked of the junctions built from
    // their legs.

    /// The four surface steps over a world whose network a specimen changed,
    /// its pieces grouped again as the partition would have grouped them.
    fn pave(w: &World) -> (Legs, Summary) {
        let net = w.network().expect("the partition ran");
        let facade = w.facade.as_ref().expect("the facade step ran");
        let groups = crate::partition::groups(&net.plan, &net.spans);
        let (ribbons, _) = crate::ribbon::run(net, &groups);
        let (surface, _) = crate::surface::run(&ribbons, facade);
        let (k, _) = crate::kerb::run(net, &surface, facade);
        run(net, &surface, &k, facade, &ribbons.masks)
    }

    #[test]
    fn a_bend_within_one_way_rounds_its_inner_side_only() {
        // The road turns a right angle at one vertex: the round join rounds
        // the outside, and the inside is a corner like any other — no real
        // kerb turns sharp — that gets the road's own return.
        let (w, s) = world("net:corner?d=5&len=200");
        let l = w.legs.as_ref().unwrap();
        let c = 2.75 + 4.0 * (1.0 - 1.0 / 2.0f64.sqrt()) - 0.05;
        let inner = [[c, c], [-c, c], [c, -c], [-c, -c]];
        assert_eq!(inner.iter().filter(|&&p| poly::contains(&l.surface.carriageway, p)).count(), 1, "one inner return: {s}");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
    }

    #[test]
    fn a_roundabout_keeps_its_island_and_its_pavement() {
        let (w, s) = world("net:roundabout?r=15&d=5&len=200");
        let l = w.legs.as_ref().unwrap();
        assert_eq!(l.surface.carriageway.len(), 1, "{s}");
        assert_eq!(l.surface.carriageway[0].len(), 2, "the island is the only hole: {s}");
        assert!(s.to_string().contains("kerb_gap=0/"), "{s}");
    }

    #[test]
    fn a_hook_is_not_filled_to_a_disc() {
        // Road-e leaves the tee and hooks back on a 5 m radius: the two
        // straights' kerbs are 4.5 m apart, under `2r`, and a closing of the
        // junction's surroundings filled the whole inside.
        let (w, s) = world("net:tee?hook=5&len=200");
        let l = w.legs.as_ref().unwrap();
        assert!(!poly::contains(&l.surface.carriageway, [10.0, 5.0]), "the inside of the hook: {s}");
        assert!(!poly::contains(&l.surface.carriageway, [8.0, 5.0]), "{s}");
        // Nor the half-metre between the hook's end and the leg's kerb.
        assert!(!poly::contains(&l.surface.carriageway, [3.0, 10.0]), "{s}");
        let c = 2.75 + 4.0 * (1.0 - 1.0 / 2.0f64.sqrt()) - 0.05;
        assert!(poly::contains(&l.surface.carriageway, [-c, c]), "{s}");
        assert!(poly::contains(&l.surface.carriageway, [c, c]), "{s}");
    }

    #[test]
    fn the_narrower_way_sets_the_radius() {
        // A service driveway on a primary: the corners between them get the
        // driveway's 3 m, not the primary's 8 m. A 3 m return reaches past a
        // point 2.2 m off both kerbs' corner; an 8 m one would reach past
        // 5.8 m, and a 3 m one leaves that ground.
        let mut w = built("flat", "net:tee?class=primary&len=200", None, 100.0, &plan(Step::Facade)).0;
        let leg = w.partition.as_mut().unwrap().network.plan.iter_mut().find(|l| l.id == "leg").unwrap();
        leg.class = "service".into();
        leg.width_m = width::of("service", "");
        let (l, s) = pave(&w);
        assert_eq!(s.num("returns"), 2.0, "{s}");
        let (hp, hs) = (width::of("primary", "") / 2.0, width::of("service", "") / 2.0);
        let near = |r: f64| r * (1.0 - 1.0 / 2.0f64.sqrt()) - 0.05;
        assert!(poly::contains(&l.surface.carriageway, [hs + near(3.0), hp + near(3.0)]), "{s}");
        assert!(!poly::contains(&l.surface.carriageway, [hs + near(8.0), hp + near(8.0)]), "{s}");
    }
}
