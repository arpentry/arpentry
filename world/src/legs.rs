//! The junctions, built from the legs that meet there — **measured, not
//! wired**.
//!
//! Review step 5 (`docs/plans/plan-chain-from-legs.md`), first slice. Today
//! a junction is not an object: it is where ribbons overlap, the surface
//! step unions them, and the fillet step finds the notches the union left
//! and rounds them with a masked closing. Every fact about the junction —
//! which legs meet, which kerbs face each other, what radius the return
//! takes — is recovered afterwards from the union by sampling
//! ([`crate::fillet`]'s `kerbs_at`), and the closing leaves hairlines that
//! `OPEN_M`, `OVERLAP_M` and `fill_holes_under` then clean away.
//!
//! This step builds the same carriageway from what is known before any
//! boolean runs:
//!
//! - **The pieces** are the ground carriageways and the **bridge** spans, so
//!   a junction standing on a deck is built like any other: a deck and the
//!   road that runs onto it are one surface. A span edge shapes the
//!   junctions it meets but is not paved here — the `sheet` step unions the
//!   span ribbons in. A bore is never continuous with the ground, and is
//!   left out as [`crate::surface::spans_grouped`] leaves it out.
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
//!   [`fillet::BEND_MIN_DEG`] is replaced by a tangent arc of the narrower
//!   leg's [`width::fillet_m`], its tangent length clamped at
//!   [`fillet::RETURN_MAX_RADII`] radii; a shallower one is a mitre; a
//!   sector of half a turn or more is the round join a ribbon has there.
//! - **A leg's mouth** is where the farther of its two sides stops, and the
//!   edge is trimmed to it with a square end. No round cap is drawn at any
//!   connector, so none can survive as a lobe.
//!
//! The step hands back a whole [`Surface`], as `fillet` does: the explicit
//! carriageway, and the pavement laid back outside wherever that carriageway
//! grew into it (`fillet`'s `laid_back`, the same rule), so it can stand in
//! for `fillet`'s once it measures better.
//!
//! The comparison is against [`crate::world::Fillet`]'s carriageway, over
//! the same facade cut: what the explicit construction has that today's
//! lacks (`extra`) and what it lacks (`missing`), each region attributed to
//! the nearest junction whose reach holds it or else to `elsewhere`, and
//! per junction the farthest the two boundaries stand apart (`apart`). It
//! stores the result for the plan view and nothing downstream reads it.

use std::collections::HashMap;

use crate::fillet;
use crate::poly::{self, Pt, Ring, Shapes};
use crate::step::{Residual, Summary};
use crate::width::{self, Family};
use crate::world::{connector, Facade, Polyline2, Roads, Surface};

/// How far a trimmed edge laps back into its junction, in metres. A boolean
/// union keeps shapes that only touch apart, and the junction's mouth and
/// the edge's square end are computed by two constructions that round
/// differently; lapped by a centimetre, they overlap instead of touching.
const LAP_M: f64 = 0.01;

/// A kerb corner farther from the node than this many summed half-widths is
/// not a corner: the two legs are near-collinear and of different widths,
/// so their kerbs meet far off or behind the node. The side is joined as if
/// it were convex, and counted as `far`.
const FAR_HALF_WIDTHS: f64 = 4.0;

/// The widest a junction's reach is taken to be past its farthest mouth, in
/// metres: a difference region further from every junction than its reach
/// plus this is `elsewhere`.
const REACH_SLACK_M: f64 = 2.0;

/// How far the boundary search looks, in metres: a disagreement larger than
/// this reads as this.
const SEARCH_M: f64 = 10.0;

/// A disagreement at a junction larger than this, in metres, is counted.
/// The arcs are drawn at the same angular step as the kernel's, but not at
/// the same angles, so every return disagrees by its chord sagitta — a
/// couple of centimetres at an 8 m radius.
const APART_M: f64 = 0.1;

/// One leg of a junction: an edge leaving the node.
#[derive(Debug, Clone, PartialEq)]
pub struct Leg {
    /// Which edge, and whether it leaves the node at its start.
    pub edge: usize,
    pub at_start: bool,
    /// Unit direction from the node along the edge's first segment: what the
    /// legs are ordered by.
    pub u: Pt,
    pub half_m: f64,
    pub class: String,
    /// Where the leg's mouth lies, in metres of arc from the node.
    pub mouth_m: f64,
    /// The edge's centreline, oriented away from the node.
    pub line: Vec<Pt>,
    /// The kerb walk past the edge's far node on the left side and on the
    /// right ([`walk`]): points, each with its stretch's half-width.
    pub ahead: [Vec<(Pt, f64)>; 2],
}

/// One junction: the node, its legs in counter-clockwise order, and the
/// polygon between their mouths.
#[derive(Debug, Clone, PartialEq)]
pub struct Junction {
    pub at: Pt,
    pub legs: Vec<Leg>,
    pub shape: Shapes,
    /// How far the junction reaches from its node: the farthest mouth
    /// corner.
    pub reach_m: f64,
    /// The farthest the explicit boundary and the fillet's stand apart in
    /// the difference regions attributed to this junction, in metres.
    pub apart_m: f64,
    /// The area of those regions: what the explicit carriageway has there
    /// that the fillet's lacks, and what it lacks.
    pub extra_m2: f64,
    pub missing_m2: f64,
}

/// A piece cut at the nodes it passes: what a junction's legs trim.
#[derive(Debug, Clone, PartialEq)]
pub struct Edge {
    pub piece: usize,
    pub pts: Vec<Pt>,
    pub width_m: f64,
    /// Where the edge is trimmed at each end, in metres of arc: the mouth of
    /// the junction there, or 0 at a free end.
    pub trim: [f64; 2],
    /// Whether each end is a node.
    pub node: [bool; 2],
    /// Whether the edge is on the ground, and so paved here: a deck's edge
    /// shapes its junctions and is paved by the `sheet` step.
    pub ground: bool,
}

/// What the step built.
#[derive(Debug, Clone, Default)]
pub struct Legs {
    pub junctions: Vec<Junction>,
    pub edges: Vec<Edge>,
    /// The paved surface as this step would leave it: the explicit
    /// carriageway, facade-cut — every junction and every trimmed ground
    /// edge — and the pavement laid back outside it.
    pub surface: Surface,
    /// What it has that the fillet's lacks, and what it lacks.
    pub extra: Shapes,
    pub missing: Shapes,
    /// The kerb stations `kerb_gap` counts as bare on this surface.
    pub gaps: Vec<Pt>,
}

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
}

pub fn run(
    roads: &Roads,
    surface: &Surface,
    k: &crate::world::Kerb,
    facade: &Facade,
    fillet: &Surface,
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
    // **Off the deck is off the square footprint** ([`surface::spans_masked`],
    // the mask the bench cuts the ground's hole by), not off the round-capped
    // ribbon: taken by the ribbon, the disc at the node went with it. And
    // the remainder is **opened by [`LAP_M`]**: the junction's kerbs and
    // the deck's are two constructions of one line, and a difference along
    // it leaves hairline fragments no piece claims — they were most of the
    // `sheet` step's orphans.
    let decks: Shapes = poly::union_all(
        &crate::surface::spans_masked(roads)
            .into_iter()
            .filter(|(f, ..)| *f == Family::Carriageway)
            .flat_map(|(.., s)| s)
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
        if e.trim[0] + e.trim[1] >= len {
            tally.short += 1;
            parts.extend(poly::buffer_line_capped(&e.pts, e.width_m, [false, false]));
            continue;
        }
        parts.extend(poly::buffer_line_capped(&between(&e.pts, a, b), e.width_m, [false, false]));
    }
    // **A hole in the asphalt that no pavement fits in is asphalt.** The
    // remnant of a small island the returns did not quite close, a narrow
    // fork closed at both ends: `room` paves a hole as a traffic island
    // only if it can hold the narrowest pavement ([`crate::room::wide_enough`],
    // the same test), so anything thinner was left bare ground between two
    // carriageways. Filled before the facade cut, so a building standing in
    // it still wins. Only the hole goes; no other boundary moves — a closing
    // over the whole surface did the same and re-rounded every kerb in the
    // box (mesh slivers 71 k → 110 k, crack 8 → 93 m).
    let mut filled = 0usize;
    let open: Shapes = poly::union_all(&parts)
        .into_iter()
        .map(|shape| {
            let mut rings = shape.into_iter();
            let outer = rings.next().expect("a region has an outer ring");
            let mut kept = vec![outer];
            for hole in rings {
                let island = vec![poly::oriented(hole.clone(), true)];
                if crate::room::wide_enough(&island) {
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
    let laid_back = poly::dilate(&poly::intersect(&out.carriageway, &k.surface.walk), crate::kerb::WALK_MIN_M);
    let senior = out.senior();
    let pavement = facade.pavement(&poly::union_of(&[&k.surface.walk, &laid_back]), &senior);
    let bare = crate::kerb::Bare::new(&senior, &pavement, &facade.footprints);
    let (gaps, gap_of) = crate::kerb::kerb_gaps(&out.carriageway, &bare, &k.attached);
    let gap_n = gaps.len();
    out.walk = pavement;
    let carriageway = &out.carriageway;
    let extra = poly::difference(carriageway, &fillet.carriageway);
    let missing = poly::difference(&fillet.carriageway, carriageway);

    // Attribute every difference region to a junction, and read how far
    // apart the two boundaries stand along it.
    let ours = Boundary::new(carriageway);
    let theirs = Boundary::new(&fillet.carriageway);
    let near = Nodes::new(&junctions);
    let mut apart = vec![0.0f64; junctions.len()];
    let mut diff = vec![[0.0f64; 2]; junctions.len()];
    let (mut near_m2, mut elsewhere_m2, mut elsewhere_apart) = (0.0, 0.0, 0.0f64);
    let tagged = extra.iter().map(|s| (0, s)).chain(missing.iter().map(|s| (1, s)));
    for (which, shape) in tagged {
        let a = poly::area(std::slice::from_ref(shape));
        let far = shape
            .iter()
            .flatten()
            .filter_map(|&p| {
                let (d0, d1) = (ours.distance(p), theirs.distance(p));
                // A vertex of a difference lies on one of the two boundaries;
                // the other is how far off the second one is.
                (d0.min(d1) < 1e-3).then_some(d0.max(d1))
            })
            .fold(0.0, f64::max);
        match near.of(shape) {
            Some(j) => {
                near_m2 += a;
                apart[j] = apart[j].max(far);
                diff[j][which] += a;
            }
            None => {
                elsewhere_m2 += a;
                elsewhere_apart = elsewhere_apart.max(far);
            }
        }
    }
    let mut spread = Residual::new();
    for &d in &apart {
        spread.push(d, 0.0);
    }
    let disagree = apart.iter().filter(|&&d| d > APART_M).count();
    for ((j, d), m2) in junctions.iter_mut().zip(&apart).zip(&diff) {
        j.apart_m = *d;
        [j.extra_m2, j.missing_m2] = *m2;
    }
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

        .with_m2("junction_m2", junctions.iter().map(|j| poly::area(&j.shape)).sum::<f64>() + 0.0)
        .with("islands", filled)
        .with_regions("carriageway", carriageway)
        .with_m2("pavement_m2", poly::area(&out.walk))
        .with_share("kerb_gap", gap_n, gap_of)
        .with("extra_m2", format!("{:.1}", poly::area(&extra)))
        .with("missing_m2", format!("{:.1}", poly::area(&missing)))
        .with("near_m2", format!("{near_m2:.1}"))
        .with("elsewhere_m2", format!("{elsewhere_m2:.1}"))
        .with("elsewhere_apart", format!("{elsewhere_apart:.2}"))
        .with_share("disagree", disagree, junctions.len())
        .with_quantiles("apart", spread);
    (Legs { junctions, edges, surface: out, extra, missing, gaps }, summary)
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
fn cut(all: &[&Polyline2], ground: usize, pieces: &[(usize, Vec<Pt>)]) -> (Vec<Edge>, HashMap<(i64, i64), Vec<End>>) {
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
            edges.push(Edge {
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
    edges: &[Edge],
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
    edges: &[Edge],
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
    let shape = if ring.len() >= 3 { poly::union_all(&vec![vec![ring]]) } else { Vec::new() };
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
    Junction { at, legs, shape, reach_m, apart_m: 0.0, extra_m2: 0.0, missing_m2: 0.0 }
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
    if turn.to_degrees() < fillet::BEND_MIN_DEG {
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
    // **A return runs at most [`fillet::RETURN_MAX_RADII`] radii along
    // either kerb from the corner**, and past that the radius shrinks until
    // it fits. The reach is read off the tangent points themselves, not off
    // the turn at the corner: a leg that bends a few metres out turns a
    // right-angled corner into a sliver of a fork, and the turn at the
    // corner cannot see that.
    let reach = fillet::RETURN_MAX_RADII * r0;
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

/// The edges of a set of regions on a grid, for "how far is the boundary
/// from here" queries capped at [`SEARCH_M`].
struct Boundary {
    cells: HashMap<(i32, i32), Vec<(Pt, Pt)>>,
}

/// The boundary grid's cell, in metres.
const CELL_M: f64 = 8.0;

impl Boundary {
    fn new(shapes: &Shapes) -> Boundary {
        let mut cells: HashMap<(i32, i32), Vec<(Pt, Pt)>> = HashMap::new();
        for ring in shapes.iter().flatten() {
            for i in 0..ring.len() {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                for c in poly::cells_over([a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])], CELL_M) {
                    cells.entry(c).or_default().push((a, b));
                }
            }
        }
        Boundary { cells }
    }

    fn distance(&self, p: Pt) -> f64 {
        let b = [p[0] - SEARCH_M, p[1] - SEARCH_M, p[0] + SEARCH_M, p[1] + SEARCH_M];
        poly::cells_over(b, CELL_M)
            .filter_map(|c| self.cells.get(&c))
            .flatten()
            .map(|&(a, b)| poly::segment_distance(a, b, p))
            .fold(SEARCH_M, f64::min)
    }
}

/// The junctions on a grid, for "which junction's reach is this region in".
struct Nodes<'a> {
    junctions: &'a [Junction],
    cells: HashMap<(i32, i32), Vec<usize>>,
}

/// The node grid's cell, in metres.
const NODE_CELL_M: f64 = 32.0;

impl<'a> Nodes<'a> {
    fn new(junctions: &'a [Junction]) -> Nodes<'a> {
        let mut cells: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (i, j) in junctions.iter().enumerate() {
            let r = j.reach_m + REACH_SLACK_M;
            for c in poly::cells_over([j.at[0] - r, j.at[1] - r, j.at[0] + r, j.at[1] + r], NODE_CELL_M) {
                cells.entry(c).or_default().push(i);
            }
        }
        Nodes { junctions, cells }
    }

    /// The junction whose reach holds most of `shape`'s vertices: the one
    /// nearest, relative to its reach, to any of them.
    fn of(&self, shape: &poly::Shape) -> Option<usize> {
        let mut best: Option<(f64, usize)> = None;
        for &p in shape.iter().flatten() {
            for &j in self.cells.get(&poly::cell_of(p, NODE_CELL_M)).into_iter().flatten() {
                let jn = &self.junctions[j];
                let d = (p[0] - jn.at[0]).hypot(p[1] - jn.at[1]) - (jn.reach_m + REACH_SLACK_M);
                if d <= 0.0 && best.is_none_or(|(b, _)| d < b) {
                    best = Some((d, j));
                }
            }
        }
        best.map(|(_, j)| j)
    }
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
        assert!(s.num("near_m2") < 1.0, "{s}");
        assert_eq!(s.num("elsewhere_m2"), 0.0, "{s}");
    }

    #[test]
    fn a_tee_has_two_returns_and_a_straight_side() {
        let (w, s) = world("net:tee?len=200");
        let l = w.legs.as_ref().unwrap();
        assert_eq!(l.junctions.len(), 1, "{s}");
        assert_eq!(s.num("returns"), 2.0, "{s}");
        assert_eq!(s.num("rounds"), 1.0, "{s}");
        assert!(s.num("near_m2") < 1.0, "{s}");
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
        let mut drive = w.roads.as_ref().unwrap().plan[0].clone();
        drive.id = "drive".into();
        drive.class = "service".into();
        drive.width_m = width::of("service", "");
        drive.pts = vec![[5.0, 60.0], [5.0, 1.0]];
        w.roads.as_mut().unwrap().plan.push(drive);
        let roads = w.roads.as_ref().unwrap();
        let facade = w.facade.as_ref().unwrap();
        let (ribbons, _) = crate::ribbon::run(roads);
        let (surface, _) = crate::surface::run(roads, &ribbons, facade);
        let (k, _) = crate::kerb::run(roads, &surface, facade);
        let (f, _) = crate::fillet::run(roads, &surface, &k, facade);
        let (l, s) = run(roads, &surface, &k, facade, &f.surface);
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
        let plan_ = &mut w.roads.as_mut().unwrap().plan;
        let i = plan_.iter().position(|p| p.id == "road-e").unwrap();
        let mut far = plan_[i].clone();
        far.id = "road-e2".into();
        far.pts = vec![[3.0, 0.0], [100.0, 0.0]];
        plan_[i].pts = vec![[0.0, 0.0], [3.0, 0.0]];
        plan_.push(far);
        let roads = w.roads.as_ref().unwrap();
        let facade = w.facade.as_ref().unwrap();
        let (ribbons, _) = crate::ribbon::run(roads);
        let (surface, _) = crate::surface::run(roads, &ribbons, facade);
        let (k, _) = crate::kerb::run(roads, &surface, facade);
        let (f, _) = crate::fillet::run(roads, &surface, &k, facade);
        let (l, s) = run(roads, &surface, &k, facade, &f.surface);
        assert_eq!(s.num("returns"), 2.0, "{s}");
        let c = 2.75 + 4.0 * (1.0 - 1.0 / 2.0f64.sqrt()) - 0.05;
        assert!(poly::contains(&l.surface.carriageway, [c, c]), "the east return: {s}");
        assert!(poly::contains(&l.surface.carriageway, [-c, c]), "the west return: {s}");
        assert!(s.num("near_m2") < 1.0, "{s}");
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
        let (_, s) = world("net:dual?gap=4&len=200");
        assert_eq!(s.num("junctions"), 0.0, "{s}");
        assert_eq!(s.num("extra_m2") + s.num("missing_m2"), 0.0, "{s}");
    }
}
