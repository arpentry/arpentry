//! Crossings, derived from the model rather than stored against it
//! (docs/GENERATION.md §4.5).
//!
//! > *A crossing is derived, never stored across a mutation. Anything that
//! > changes a feature's geometry or span structure invalidates every crossing
//! > derived from it.*
//!
//! The set this replaces was built once, at assemble time, from a second pass
//! over the input file — and then invalidated by everything that followed.
//! `reconcile_short_spans` demoted the spans it was keyed on; the profile solve
//! absorbed anchors into structures and `portals::reconcile_spans` shrank
//! tunnels; none of it was written back. A crossing record pointing at a span
//! that no longer exists is not a stale number, it is a demand for clearance
//! over nothing.
//!
//! Deriving it here — after the per-corridor profiles are solved, before they
//! are fused — costs one pass over an in-memory index instead of a pass over
//! the parquet, and it can use what the file pass could not: the solved
//! heights. So two kinds of crossing come out of one walk.
//!
//! **Annotated.** The level hints differ at the intersection, so the ordinal
//! says which is above. This is what the file pass found, and the ordinal is
//! trusted for the *ordering* only — never for a height (§2.1).
//!
//! **Unannotated.** Both features are at grade in the data and their solved
//! surfaces are [`SEPARATION_M`] apart. The data says nothing, but the
//! alignments do: one road demonstrably passes over the other. §2.1 lists
//! "crossings are implicit" among the things the data does not say, and the
//! file pass could only ever find the annotated half.
//!
//! What is *not* derived here is the same-level touching case — a road meeting
//! a rail at grade, where the two surfaces must coincide. That is an equality
//! rather than an inequality, it needs the senior's published height to be one
//! side of it, and it belongs with the strata (§4.5).

use geo_types::Coord;

use crate::assemble::grid::GridIndex;
use crate::scene::{Crossing, SceneGraph, SpanKind};

use super::profile::Profile;

/// Solved surfaces this far apart at a plan intersection are grade-separated,
/// whatever the data says. Read against what a crossing has to mean: below a
/// storey there is no room for a road to pass, and the class clearances start
/// at 4 m. Two carriageways within this of each other at one point are a braid,
/// a slip road, or one road mapped twice — never an overpass.
pub const SEPARATION_M: f64 = 3.0;

/// One indexed corridor edge.
struct Edge {
    corridor: u32,
    /// Corridor node index; the edge spans `nodes[i]..nodes[i+1]`.
    node: usize,
}

/// Derives the crossings **one stratum owes**, from the solved profiles.
///
/// A crossing is a constraint on the feature that must yield, and authority
/// decides which that is (§4.1). So a record survives only when its *upper*
/// side belongs to `stratum`: that is the side this solver can move, and the
/// lower is either a peer (a shared unknown) or a senior (a published
/// constant).
///
/// Where the lower side is **junior**, the crossing is dropped outright. A
/// junior feature cannot constrain a senior one — that is I7, and a footbridge
/// over a motorway is exactly the case §4.2 has in mind.
///
/// Deterministic: the index is built in corridor order, the results are sorted,
/// and one record survives per crossed *place* — a shared vertex of two
/// adjacent edges reports the same intersection twice, at the same arc, and
/// only that is a duplicate. Keying on the pair alone discarded 30 of the
/// Montreux extract's 583 ordered crossings, because a ramp that weaves over
/// its mainline crosses it more than once and every crossing but the first
/// silently lost its clearance.
pub fn derive(
    scene: &SceneGraph,
    profiles: &[Option<Profile>],
    stratum: crate::priors::Stratum,
) -> Vec<Crossing> {
    let mut edges: Vec<Edge> = Vec::new();
    let mut grid = GridIndex::new();
    for c in &scene.corridors {
        for i in 0..c.nodes.len().saturating_sub(1) {
            let (a, b) = (c.nodes[i], c.nodes[i + 1]);
            grid.insert((a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)), edges.len() as u32);
            edges.push(Edge { corridor: c.id, node: i });
        }
    }

    let mut out: Vec<Crossing> = Vec::new();
    // Per (upper, lower, level pair), the upper arcs already claimed — so the
    // duplicate a shared vertex produces collapses and a second crossing
    // hundreds of metres along the same pair does not.
    let mut seen: std::collections::HashMap<(u32, u32, i64, i64), Vec<f64>> =
        std::collections::HashMap::new();
    let mut candidates: Vec<u32> = Vec::new();
    for c in &scene.corridors {
        for i in 0..c.nodes.len().saturating_sub(1) {
            let (a, b) = (c.nodes[i], c.nodes[i + 1]);
            grid.query((a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)), &mut candidates);
            for &ei in candidates.iter() {
                let e = &edges[ei as usize];
                if e.corridor <= c.id {
                    continue; // each pair once, and never a corridor with itself
                }
                let other = &scene.corridors[e.corridor as usize];
                let (o_a, o_b) = (other.nodes[e.node], other.nodes[e.node + 1]);
                let Some((t, u)) = seg_intersect(a, b, o_a, o_b, c.cos_lat) else {
                    continue;
                };
                let point = Coord { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
                // Do they *meet* here, or pass over one another? Meeting is
                // reconciled by the shared variable, not by a clearance — but
                // that is a fact about this *place*, and a corridor is a
                // spliced chain hundreds of metres long.
                if meets_here(c, other, point, (a, b), (o_a, o_b)) {
                    continue;
                }
                let arc_c = c.arc[i] + t * (c.arc[i + 1] - c.arc[i]);
                let arc_o = other.arc[e.node] + u * (other.arc[e.node + 1] - other.arc[e.node]);
                let Some(x) = order(scene, profiles, c.id, arc_c, e.corridor, arc_o, point) else {
                    continue;
                };
                // Authority chooses the mover (§4.1). This stratum takes the
                // constraint when it is the one that can yield — whether it is
                // the side above (it climbs) or the side below (it dips) — and
                // never when the side it would have to move is senior.
                let upper_s = scene.corridors[x.upper as usize].kind.stratum();
                let lower_s = x.lower.map_or(stratum, |l| scene.corridors[l as usize].kind.stratum());
                let ours = (upper_s == stratum && lower_s <= stratum)
                    || (lower_s == stratum && upper_s < stratum);
                if !ours {
                    continue;
                }
                let key = (x.upper, x.lower.unwrap_or(u32::MAX), x.upper_level, x.lower_level);
                let claimed = seen.entry(key).or_default();
                if claimed.iter().all(|&a| (a - x.upper_arc).abs() >= DUPLICATE_M) {
                    claimed.push(x.upper_arc);
                    out.push(x);
                }
            }
        }
    }
    out.sort_by(|a, b| {
        (a.upper, a.upper_arc.to_bits(), a.lower_level)
            .cmp(&(b.upper, b.upper_arc.to_bits(), b.lower_level))
    });
    out
}

/// One plan intersection as seen from one corridor: where it sits along this
/// corridor, how far the crossing feature's band reaches, and who the other
/// side is — so a consumer that needs the other side's *annotation* (never its
/// heights) can look it up.
#[derive(Debug, Clone, Copy)]
pub struct PlanCrossing {
    /// Arc along this corridor where the other alignment crosses.
    pub arc: f64,
    /// How far along this corridor the crossing feature's drawn band reaches
    /// from the intersection: its half-width plus [`ANNEX_SHOULDER_M`] of
    /// shoulder, verge and annotation slack.
    pub clear_m: f64,
    /// The crossing corridor.
    pub other: u32,
    /// The crossing's arc along the *other* corridor.
    pub other_arc: f64,
}

/// Every place another corridor's alignment crosses each corridor, per
/// corridor, sorted by arc — the *plan* facts only, with no ordering and no
/// heights. Heights are deliberately not consulted anywhere here: a junior's
/// warm start is not a fact, and the consumers that need height evidence
/// measure it on their own side ([`super::portals::span_bounds`]'s buried run,
/// [`super::relax`]'s bore ceilings against this corridor's own terrain).
pub fn plan_index(scene: &SceneGraph) -> Vec<Vec<PlanCrossing>> {
    let mut edges: Vec<Edge> = Vec::new();
    let mut grid = GridIndex::new();
    for c in &scene.corridors {
        for i in 0..c.nodes.len().saturating_sub(1) {
            let (a, b) = (c.nodes[i], c.nodes[i + 1]);
            grid.insert((a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)), edges.len() as u32);
            edges.push(Edge { corridor: c.id, node: i });
        }
    }
    let clear = |c: &crate::scene::Corridor| {
        c.width_m.map_or(0.0, |w| w * 0.5) + ANNEX_SHOULDER_M
    };
    let debug = std::env::var_os("ARPT_DEBUG_ANNEX").is_some();
    let mut out: Vec<Vec<PlanCrossing>> = vec![Vec::new(); scene.corridors.len()];
    let mut candidates: Vec<u32> = Vec::new();
    for c in &scene.corridors {
        for i in 0..c.nodes.len().saturating_sub(1) {
            let (a, b) = (c.nodes[i], c.nodes[i + 1]);
            grid.query((a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)), &mut candidates);
            for &ei in candidates.iter() {
                let e = &edges[ei as usize];
                if e.corridor <= c.id {
                    continue; // each pair once, and never a corridor with itself
                }
                let other = &scene.corridors[e.corridor as usize];
                let (o_a, o_b) = (other.nodes[e.node], other.nodes[e.node + 1]);
                let Some((t, u)) = seg_intersect(a, b, o_a, o_b, c.cos_lat) else {
                    continue;
                };
                let point = Coord { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
                if meets_here(c, other, point, (a, b), (o_a, o_b)) {
                    if debug {
                        eprintln!(
                            "[plan] meet {}x{} at {:.6},{:.6}",
                            c.id, e.corridor, point.x, point.y
                        );
                    }
                    continue;
                }
                let arc_c = c.arc[i] + t * (c.arc[i + 1] - c.arc[i]);
                let arc_o = other.arc[e.node] + u * (other.arc[e.node + 1] - other.arc[e.node]);
                if debug {
                    eprintln!(
                        "[plan] cross {}x{} at {:.6},{:.6} arcs {:.1}/{:.1}",
                        c.id, e.corridor, point.x, point.y, arc_c, arc_o
                    );
                }
                out[c.id as usize].push(PlanCrossing {
                    arc: arc_c,
                    clear_m: clear(other),
                    other: e.corridor,
                    other_arc: arc_o,
                });
                out[e.corridor as usize].push(PlanCrossing {
                    arc: arc_o,
                    clear_m: clear(c),
                    other: c.id,
                    other_arc: arc_c,
                });
            }
        }
    }
    for list in &mut out {
        list.sort_by(|a, b| {
            (a.arc.to_bits(), a.other, a.other_arc.to_bits())
                .cmp(&(b.arc.to_bits(), b.other, b.other_arc.to_bits()))
        });
        list.dedup_by(|a, b| (a.arc - b.arc).abs() < 1e-9 && a.other == b.other);
    }
    out
}

/// [`plan_index`] reduced to the `(arc, clear_m)` pairs the bore annex
/// consumes ([`super::portals::annex_spans`]).
pub fn plan_crossings(scene: &SceneGraph) -> Vec<Vec<(f64, f64)>> {
    plan_index(scene).into_iter().map(|list| reaches(&list)).collect()
}

/// One corridor's [`PlanCrossing`]s as bare `(arc, clear_m)` reaches.
pub fn reaches(list: &[PlanCrossing]) -> Vec<(f64, f64)> {
    list.iter().map(|x| (x.arc, x.clear_m)).collect()
}

/// The span kind a corridor's partition gives at `arc` — the annotation, not a
/// height. Out-of-partition arcs (float slop at the ends) read as grade.
pub fn kind_at(scene: &SceneGraph, corridor: u32, arc: f64) -> SpanKind {
    scene.corridors[corridor as usize]
        .spans
        .iter()
        .find(|s| arc >= s.arc0 && arc <= s.arc1)
        .map_or(SpanKind::Grade, |s| s.kind)
}

/// Per corridor, the windows of its mapped tunnel spans that another mapped
/// alignment crosses over **from above**, as `(arc0, arc1)` in arc order.
///
/// This is the burial license [`super::relax::seed_bore_ceilings`] needs
/// (§4.5): a bore's annotation says "below the ground", and where a feature
/// annotated *above* it crosses, the ground there carries that feature's
/// roadbed — an at-grade band directly, a low local bridge through the
/// abutments it stands on — so the bore must actually pass beneath, roof and
/// cover included. The gate is annotation-only on both sides, never a height:
/// the crossing side's warm start is not a fact (§4.1), and the level
/// ordinals are an ordering, which is exactly the question. The ordering is
/// what excludes the cases that license nothing: a peer bore crossing at the
/// same level, and a deeper tunnel passing below.
pub fn covered_bores(scene: &SceneGraph, plan: &[Vec<PlanCrossing>]) -> Vec<Vec<(f64, f64)>> {
    covered_sites(scene, plan)
        .into_iter()
        .map(|sites| sites.iter().map(|x| (x.arc - x.clear_m, x.arc + x.clear_m)).collect())
        .collect()
}

/// The gated sites behind [`covered_bores`], one [`PlanCrossing`] per place a
/// mapped tunnel span is crossed by an at-grade band — shared with the
/// `structure.bore_daylight` check so the measurement and the constraint can
/// never drift apart.
pub fn covered_sites(scene: &SceneGraph, plan: &[Vec<PlanCrossing>]) -> Vec<Vec<PlanCrossing>> {
    let debug = std::env::var_os("ARPT_DEBUG_ANNEX").is_some();
    scene
        .corridors
        .iter()
        .map(|c| {
            let mut sites: Vec<PlanCrossing> = Vec::new();
            for s in c.spans.iter().filter(|s| s.kind == SpanKind::Tunnel) {
                for x in &plan[c.id as usize] {
                    if x.arc < s.arc0 || x.arc > s.arc1 {
                        continue;
                    }
                    let above = level_at(scene, x.other, x.other_arc) > s.level;
                    if debug {
                        eprintln!(
                            "[cover] corridor {} {:?} tunnel [{:.1}, {:.1}] level {} crossed at \
                             {:.1} by {} ({:?} level {} at {:.1}){}",
                            c.id,
                            c.kind,
                            s.arc0,
                            s.arc1,
                            s.level,
                            x.arc,
                            x.other,
                            kind_at(scene, x.other, x.other_arc),
                            level_at(scene, x.other, x.other_arc),
                            x.other_arc,
                            if above { " -> buried" } else { "" }
                        );
                    }
                    if !above {
                        continue;
                    }
                    sites.push(*x);
                }
            }
            sites.sort_by(|a, b| a.arc.partial_cmp(&b.arc).expect("finite arcs"));
            sites
        })
        .collect()
}

/// Shoulder added to a crossing feature's half-width when a bore is extended
/// beneath it: the drawn band's structure shoulder, the bench verge outside
/// it, and a metre of annotation slack, so the portal the annex implies
/// daylights beyond the crossing feature's drawn footprint rather than inside
/// its kerb.
const ANNEX_SHOULDER_M: f64 = 4.0;

/// How far inside a witness line a crossing must sit. A crossing at the
/// line's own terminus is a meeting — a footpath drawn to the railway and
/// ending on it, or a couple of metres past it — not a passage beneath.
/// Witness lines carry no connector topology, so locality along the line
/// stands in for [`meets_here`]'s identity half.
const WITNESS_MEET_M: f64 = 2.0;

/// Which structure spans pass over or under *another mapped alignment*, per
/// corridor and parallel to its `spans`.
///
/// [`super::reconcile_short_spans`] asks of a short span "does the ground fall
/// away here?", which is the right question for a bridge over a gully and the
/// wrong one for a bridge over a street: a rail crossing one road is 10–15 m
/// long and lifts clear of a carriageway, not of a landform, so it fails both
/// the length test and the dip test. It is also the case where the annotation
/// matters most — it is the only statement in the data about which of the two
/// is on top. Demoted, it leaves the ordering to be *derived* from metre-scale
/// differences between solved surfaces, and one line then crosses over some
/// roads and under others.
///
/// Measured on the Montreux extract, the short-span test demoted 110 of 156
/// annotated rail structures (1,516 m). The Montreux–Glion funicular's single
/// bridge — its one crossing of a road — missed the dip threshold by 0.43 m.
///
/// A shared connector still means the two *meet* rather than cross
/// ([`meets_here`]), so a junction inside a span is not a reason to keep it.
///
/// The mapped alignments are the other corridors **and the witness lines**
/// (`SceneGraph::witnesses`): a draped path or a flowing watercourse never
/// solves — its deck is fitted after the solve, its surface left to the DEM —
/// but its plan existence is the same statement, that something passes under
/// (or over) this span. Measured after the corridor exemption landed, the dip
/// test still demoted 408 short annotated structures (4.9 km) on the Montreux
/// extract, and 312 of them (3.5 km) crossed a mapped watercourse or path the
/// scene had dropped: the DEM cannot see the 2 m stream cut under a 10 m
/// bridge, and the map is the only thing left that can.
pub fn spans_over_a_mapped_line(scene: &SceneGraph) -> Vec<Vec<bool>> {
    let mut edges: Vec<Edge> = Vec::new();
    let mut grid = GridIndex::new();
    for c in &scene.corridors {
        for i in 0..c.nodes.len().saturating_sub(1) {
            let (a, b) = (c.nodes[i], c.nodes[i + 1]);
            grid.insert((a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)), edges.len() as u32);
            edges.push(Edge { corridor: c.id, node: i });
        }
    }
    // The witness edges, in their own index, with each line's cumulative arc
    // for the terminus test.
    let mut wedges: Vec<(u32, u32)> = Vec::new();
    let mut wgrid = GridIndex::new();
    for (li, line) in scene.witnesses.iter().enumerate() {
        for j in 0..line.len().saturating_sub(1) {
            let (a, b) = (line[j], line[j + 1]);
            wgrid.insert((a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)), wedges.len() as u32);
            wedges.push((li as u32, j as u32));
        }
    }
    let warcs: Vec<Vec<f64>> = scene.witnesses.iter().map(|l| cumulative_arc(l)).collect();

    let mut candidates: Vec<u32> = Vec::new();
    scene
        .corridors
        .iter()
        .map(|c| {
            let mut over = vec![false; c.spans.len()];
            for (si, s) in c.spans.iter().enumerate() {
                if s.kind == SpanKind::Grade {
                    continue;
                }
                for i in 0..c.nodes.len().saturating_sub(1) {
                    // Only the corridor edges this span actually covers.
                    if c.arc[i + 1] < s.arc0 || c.arc[i] > s.arc1 {
                        continue;
                    }
                    let (a, b) = (c.nodes[i], c.nodes[i + 1]);
                    grid.query(
                        (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)),
                        &mut candidates,
                    );
                    for &ei in candidates.iter() {
                        let e = &edges[ei as usize];
                        if e.corridor == c.id {
                            continue; // never a corridor with itself
                        }
                        let other = &scene.corridors[e.corridor as usize];
                        let (o_a, o_b) = (other.nodes[e.node], other.nodes[e.node + 1]);
                        let Some((t, _)) = seg_intersect(a, b, o_a, o_b, c.cos_lat) else {
                            continue;
                        };
                        // The intersection must land inside the span, not merely
                        // on an edge that overlaps its ends.
                        let arc_here = c.arc[i] + t * (c.arc[i + 1] - c.arc[i]);
                        if arc_here < s.arc0 || arc_here > s.arc1 {
                            continue;
                        }
                        let point = Coord { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
                        if meets_here(c, other, point, (a, b), (o_a, o_b)) {
                            continue;
                        }
                        over[si] = true;
                        break;
                    }
                    if over[si] {
                        break;
                    }
                    wgrid.query(
                        (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)),
                        &mut candidates,
                    );
                    for &wi in candidates.iter() {
                        let (li, j) = wedges[wi as usize];
                        let line = &scene.witnesses[li as usize];
                        let (o_a, o_b) = (line[j as usize], line[j as usize + 1]);
                        let Some((t, u)) = seg_intersect(a, b, o_a, o_b, c.cos_lat) else {
                            continue;
                        };
                        let arc_here = c.arc[i] + t * (c.arc[i + 1] - c.arc[i]);
                        if arc_here < s.arc0 || arc_here > s.arc1 {
                            continue;
                        }
                        let wa = &warcs[li as usize];
                        let w_arc = wa[j as usize] + u * (wa[j as usize + 1] - wa[j as usize]);
                        let w_len = *wa.last().expect("witness lines have >= 2 vertices");
                        if w_arc < WITNESS_MEET_M || w_arc > w_len - WITNESS_MEET_M {
                            continue; // its terminus: a meeting, not a passage
                        }
                        over[si] = true;
                        break;
                    }
                    if over[si] {
                        break;
                    }
                }
            }
            over
        })
        .collect()
}

/// Cumulative arc length (metres) at each vertex of a witness line.
fn cumulative_arc(line: &[Coord]) -> Vec<f64> {
    let cos_lat = line.first().map_or(1.0, |p| p.y.to_radians().cos());
    let mut arc = Vec::with_capacity(line.len());
    let mut acc = 0.0;
    arc.push(0.0);
    for w in line.windows(2) {
        let dx = (w[1].x - w[0].x) * cos_lat;
        let dy = w[1].y - w[0].y;
        acc += (dx * dx + dy * dy).sqrt() * crate::scene::DEG_M;
        arc.push(acc);
    }
    arc
}

/// How close two intersections of one corridor pair must be, along the upper
/// corridor, to be the same crossing reported twice.
///
/// A shared vertex belongs to two adjacent edges, so the walk finds the same
/// intersection from both and computes the *same* arc for it: the duplicate is
/// exact, and this only has to be wider than float noise. Anything further
/// apart is a second place the two features cross, which owes its own
/// clearance.
const DUPLICATE_M: f64 = 5.0;

/// How close a plan intersection must sit to a vertex of both alignments to be
/// the connector they share rather than a place one passes over the other.
///
/// A connector is a *point* on both features, so where two corridors genuinely
/// meet the intersection lands on a vertex each — to the metre, since the two
/// vertices are the same coordinate in the data. A grade separation crosses
/// between vertices, and a metre is far tighter than any node spacing.
const MEET_M: f64 = 1.0;

/// Whether the two alignments **meet** at `point` — sharing a connector, and
/// sharing the vertex it sits on — rather than one passing over the other.
///
/// The identity half of this test used to stand alone: any two corridors whose
/// connector *sets* intersected were held to meet, everywhere they crossed.
/// That answers "do these two ever meet?" where the question is "do they meet
/// here?", and a corridor is a spliced chain: a motorway and its ramp share a
/// connector at the merge and cross again at the interchange 400 m away, a road
/// tunnels under the street it joins a block later. Measured on the Montreux
/// extract, the identity test rejected 50,232 plan intersections and 22 of them
/// were not meetings at all — 21 of those ordered by their level hints, which
/// is 21 grade separations that generated no clearance demand and therefore no
/// structure.
///
/// The intersection lies on both edges, so a coincident vertex can only be one
/// of the four edge endpoints: the locality test is O(1) and needs no index.
fn meets_here(
    c: &crate::scene::Corridor,
    other: &crate::scene::Corridor,
    point: Coord,
    edge: (Coord, Coord),
    other_edge: (Coord, Coord),
) -> bool {
    if !c.connectors.iter().any(|k| other.connectors.binary_search(k).is_ok()) {
        return false;
    }
    at_vertex(point, edge, c.cos_lat) && at_vertex(point, other_edge, c.cos_lat)
}

/// Whether `point` sits on one of the edge's own endpoints.
fn at_vertex(point: Coord, edge: (Coord, Coord), cos_lat: f64) -> bool {
    let d = |v: Coord| {
        let dx = (v.x - point.x) * cos_lat * crate::scene::DEG_M;
        let dy = (v.y - point.y) * crate::scene::DEG_M;
        (dx * dx + dy * dy).sqrt()
    };
    d(edge.0).min(d(edge.1)) < MEET_M
}

/// Which of the two passes over the other, and what the crosser owes.
///
/// The level hints decide when they differ — an ordinal is an ordering, which
/// is exactly the question here. Where the data is silent the solved surfaces
/// decide, and where they agree too there is nothing to order: a braid, or two
/// roads that genuinely meet.
fn order(
    scene: &SceneGraph,
    profiles: &[Option<Profile>],
    ci: u32,
    arc_c: f64,
    oi: u32,
    arc_o: f64,
    point: Coord,
) -> Option<Crossing> {
    let level_c = level_at(scene, ci, arc_c);
    let level_o = level_at(scene, oi, arc_o);
    let (upper, upper_arc, lower, lower_arc, upper_level, lower_level) = if level_c != level_o {
        if level_c > level_o {
            (ci, arc_c, oi, arc_o, level_c, level_o)
        } else {
            (oi, arc_o, ci, arc_c, level_o, level_c)
        }
    } else {
        // Silent data: ask the alignments.
        let h_c = profiles.get(ci as usize)?.as_ref()?.road_at_arc(arc_c);
        let h_o = profiles.get(oi as usize)?.as_ref()?.road_at_arc(arc_o);
        if (h_c - h_o).abs() < SEPARATION_M {
            return None; // coincident surfaces: a braid, not a crossing
        }
        if h_c > h_o {
            (ci, arc_c, oi, arc_o, level_c, level_o)
        } else {
            (oi, arc_o, ci, arc_c, level_o, level_c)
        }
    };
    Some(Crossing {
        upper,
        upper_arc,
        point,
        lower: Some(lower),
        lower_kind: scene.corridors[lower as usize].kind,
        upper_level,
        lower_level,
        // Kept so the consistency check can report the crossed feature's own
        // arc without re-deriving it.
        lower_arc,
    })
}

/// The level ordinal the corridor's span partition gives at `arc` — the hint,
/// not a height.
fn level_at(scene: &SceneGraph, corridor: u32, arc: f64) -> i64 {
    scene.corridors[corridor as usize]
        .spans
        .iter()
        .find(|s| arc >= s.arc0 && arc <= s.arc1)
        .map_or(0, |s| if s.kind == SpanKind::Grade { 0 } else { s.level })
}

/// Proper intersection of two segments in the local metric frame, as the
/// fractions along each. `None` for parallel or non-crossing pairs, and for a
/// touch at an endpoint (which is a join, not a crossing).
fn seg_intersect(
    a: Coord,
    b: Coord,
    c: Coord,
    d: Coord,
    cos_lat: f64,
) -> Option<(f64, f64)> {
    let (ax, ay) = (a.x * cos_lat, a.y);
    let (bx, by) = (b.x * cos_lat, b.y);
    let (cx, cy) = (c.x * cos_lat, c.y);
    let (dx, dy) = (d.x * cos_lat, d.y);
    let (r_x, r_y) = (bx - ax, by - ay);
    let (s_x, s_y) = (dx - cx, dy - cy);
    let denom = r_x * s_y - r_y * s_x;
    if denom.abs() < 1e-18 {
        return None; // parallel
    }
    let t = ((cx - ax) * s_y - (cy - ay) * s_x) / denom;
    let u = ((cx - ax) * r_y - (cy - ay) * r_x) / denom;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some((t, u))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::{Kind, RoadClass};
    use crate::scene::{Corridor, SegmentRef, Span, DEG_M};

    fn cos_lat() -> f64 {
        46.0_f64.to_radians().cos()
    }

    /// A straight corridor through `(x0, y0)` running east (`east`) or north.
    fn corridor(id: u32, x0: f64, y0: f64, east: bool, len_m: f64, spans: Vec<Span>) -> Corridor {
        let n = 5;
        let deg = len_m / (DEG_M * if east { cos_lat() } else { 1.0 });
        let nodes: Vec<Coord> = (0..n)
            .map(|i| {
                let d = deg * i as f64 / (n - 1) as f64;
                if east {
                    Coord { x: x0 + d, y: y0 }
                } else {
                    Coord { x: x0, y: y0 + d }
                }
            })
            .collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        Corridor {
            id,
            nodes,
            arc,
            cos_lat: cos_lat(),
            kind: Kind::Road(RoadClass::Secondary),
            class_key: String::new(),
            link: false,
            width_m: Some(6.0),
            spans,
            segments: vec![SegmentRef { source: id as u64, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }
    }

    fn grade(len: f64) -> Vec<Span> {
        vec![Span { arc0: 0.0, arc1: len, level: 0, kind: SpanKind::Grade }]
    }

    fn flat(c: &Corridor, h: f64) -> Option<Profile> {
        Some(Profile::flat(&c.nodes, h))
    }

    #[test]
    fn a_short_span_over_another_corridor_is_seen_as_crossing_it() {
        // The funicular-at-Collonge case: a 13 m annotated bridge whose whole
        // reason to exist is the road beneath it. `reconcile_short_spans` may
        // not demote it, or the crossing order falls to the derivation.
        let len = 200.0;
        let bridge = Span { arc0: 90.0, arc1: 103.0, level: 1, kind: SpanKind::Bridge };
        let over = corridor(
            0,
            6.0,
            46.0,
            false,
            len,
            vec![
                Span { arc0: 0.0, arc1: 90.0, level: 0, kind: SpanKind::Grade },
                bridge,
                Span { arc0: 103.0, arc1: len, level: 0, kind: SpanKind::Grade },
            ],
        );
        // A road running east through the bridge's plan extent.
        let mid = over.nodes[2];
        let under = corridor(1, mid.x - 0.001, mid.y, true, len, grade(len));
        let scene = SceneGraph::new(vec![over, under]);
        let flags = spans_over_a_mapped_line(&scene);
        assert!(flags[0][1], "the bridge span must be seen crossing the road");
        assert!(!flags[0][0] && !flags[0][2], "grade spans are never marked");
    }

    #[test]
    fn a_span_that_crosses_nothing_is_left_to_the_terrain_test() {
        let len = 200.0;
        let lone = corridor(
            0,
            6.0,
            46.0,
            false,
            len,
            vec![
                Span { arc0: 0.0, arc1: 90.0, level: 0, kind: SpanKind::Grade },
                Span { arc0: 90.0, arc1: 103.0, level: 1, kind: SpanKind::Bridge },
                Span { arc0: 103.0, arc1: len, level: 0, kind: SpanKind::Grade },
            ],
        );
        let scene = SceneGraph::new(vec![lone]);
        assert!(!spans_over_a_mapped_line(&scene)[0][1]);
    }

    /// A corridor with a short bridge span at arc 90..103 (running north from
    /// `y0 = 46.0`), and a scene holding it plus the given witness lines.
    fn scene_with_witnesses(witnesses: Vec<Vec<Coord>>) -> SceneGraph {
        let len = 200.0;
        let c = corridor(
            0,
            6.0,
            46.0,
            false,
            len,
            vec![
                Span { arc0: 0.0, arc1: 90.0, level: 0, kind: SpanKind::Grade },
                Span { arc0: 90.0, arc1: 103.0, level: 1, kind: SpanKind::Bridge },
                Span { arc0: 103.0, arc1: len, level: 0, kind: SpanKind::Grade },
            ],
        );
        let mut scene = SceneGraph::new(vec![c]);
        scene.witnesses = witnesses;
        scene
    }

    /// The latitude of the bridge span's midpoint (arc ≈ 96.5 m north of 46.0).
    fn mid_span_y() -> f64 {
        46.0 + 96.5 / DEG_M
    }

    #[test]
    fn a_short_span_over_a_witness_line_is_seen_as_crossing_it() {
        // A mapped stream under a 13 m annotated bridge: the DEM never shows
        // the metre-wide cut, the water line is the only evidence — and it is
        // enough. The line runs east through the span, ends well clear of it.
        let dx = 0.001;
        let line = vec![
            Coord { x: 6.0 - dx, y: mid_span_y() },
            Coord { x: 6.0 + dx, y: mid_span_y() },
        ];
        let flags = spans_over_a_mapped_line(&scene_with_witnesses(vec![line]));
        assert!(flags[0][1], "the bridge span must be seen crossing the witness");
        assert!(!flags[0][0] && !flags[0][2], "grade spans are never marked");
    }

    #[test]
    fn a_witness_ending_on_the_span_is_a_meeting_not_a_crossing() {
        // A footpath drawn up to the railway and a metre past its centerline
        // ends there — nothing passes beneath, so it keeps no bridge.
        let overshoot = 1.0 / (DEG_M * cos_lat());
        let line = vec![
            Coord { x: 6.0 - 0.001, y: mid_span_y() },
            Coord { x: 6.0 + overshoot, y: mid_span_y() },
        ];
        assert!(!spans_over_a_mapped_line(&scene_with_witnesses(vec![line]))[0][1]);
    }

    #[test]
    fn a_witness_crossing_outside_the_span_keeps_nothing() {
        // The path crosses the corridor at arc ≈ 50 m, inside a grade span:
        // no structure span covers the crossing, so no exemption.
        let y = 46.0 + 50.0 / DEG_M;
        let line =
            vec![Coord { x: 6.0 - 0.001, y }, Coord { x: 6.0 + 0.001, y }];
        let flags = spans_over_a_mapped_line(&scene_with_witnesses(vec![line]));
        assert!(!flags.concat().iter().any(|&b| b));
    }

    /// The annotated case the file pass used to find: a bridge span over an
    /// at-grade road. The ordinal orders it.
    #[test]
    fn a_level_hint_orders_the_crossing() {
        let len = 200.0;
        let a = corridor(
            0,
            6.0,
            46.0009,
            true,
            len,
            vec![Span { arc0: 0.0, arc1: len, level: 1, kind: SpanKind::Bridge }],
        );
        // The north-south road starts south of A's latitude and runs through it.
        let b = corridor(1, 6.0009, 46.0, false, len, grade(len));
        let profiles = vec![flat(&a, 400.0), flat(&b, 400.0)];
        let scene = SceneGraph::new(vec![a, b]);
        let out = derive(&scene, &profiles, crate::priors::Stratum::S);
        assert_eq!(out.len(), 1, "one crossing, got {out:?}");
        assert_eq!(out[0].upper, 0, "the annotated bridge is above");
        assert_eq!(out[0].lower, Some(1));
    }

    /// What the file pass could never find: neither road is annotated, but the
    /// solved alignments are metres apart, so one plainly passes over the other.
    #[test]
    fn silent_data_is_ordered_by_the_solved_surfaces() {
        let len = 200.0;
        let a = corridor(0, 6.0, 46.0009, true, len, grade(len));
        let b = corridor(1, 6.0009, 46.0, false, len, grade(len));
        let profiles = vec![flat(&a, 412.0), flat(&b, 400.0)];
        let scene = SceneGraph::new(vec![a, b]);
        let out = derive(&scene, &profiles, crate::priors::Stratum::S);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].upper, 0, "the higher alignment is above");
    }

    /// Two carriageways at one height are a braid, not an overpass. Ordering
    /// them would demand clearance between roads that share their asphalt.
    #[test]
    fn coincident_surfaces_are_not_a_crossing() {
        let len = 200.0;
        let a = corridor(0, 6.0, 46.0009, true, len, grade(len));
        let b = corridor(1, 6.0009, 46.0, false, len, grade(len));
        let profiles = vec![flat(&a, 400.0), flat(&b, 400.5)];
        let scene = SceneGraph::new(vec![a, b]);
        assert!(derive(&scene, &profiles, crate::priors::Stratum::S).is_empty());
    }

    /// Features that meet at a connector *meet*: their heights are reconciled
    /// by the shared variable, and demanding clearance there would lift a ramp
    /// off the road it joins. The two share the vertex, which is what a
    /// connector is.
    #[test]
    fn a_shared_connector_is_a_junction_not_a_crossing() {
        let len = 200.0;
        let mut a = corridor(0, 6.0, 46.0009, true, len, grade(len));
        let mut b = corridor(1, 6.0009, 46.0, false, len, grade(len));
        a.connectors = vec![77];
        b.connectors = vec![77];
        // Put a vertex of each exactly at the plan intersection: that point is
        // the connector, and this is the T-junction it models.
        let meet = Coord { x: b.nodes[0].x, y: a.nodes[0].y };
        a.nodes[2] = meet;
        b.nodes[2] = meet;
        let profiles = vec![flat(&a, 412.0), flat(&b, 400.0)];
        let scene = SceneGraph::new(vec![a, b]);
        assert!(derive(&scene, &profiles, crate::priors::Stratum::S).is_empty());
    }

    /// The same two corridors, sharing that connector **somewhere else**. A
    /// motorway and its ramp meet at the merge and cross again at the
    /// interchange; a road tunnels under the street it joins a block later.
    /// The pair is a junction there and a grade separation here, and reading
    /// the connector *sets* instead of this place lost the clearance for both.
    #[test]
    fn corridors_that_meet_elsewhere_still_cross_here() {
        let len = 200.0;
        let mut a = corridor(
            0,
            6.0,
            46.0009,
            true,
            len,
            vec![Span { arc0: 0.0, arc1: len, level: 1, kind: SpanKind::Bridge }],
        );
        let mut b = corridor(1, 6.0009, 46.0, false, len, grade(len));
        // They share connector 77 — at their far ends, not where they cross.
        a.connectors = vec![77];
        b.connectors = vec![77];
        let profiles = vec![flat(&a, 400.0), flat(&b, 400.0)];
        let scene = SceneGraph::new(vec![a, b]);
        let out = derive(&scene, &profiles, crate::priors::Stratum::S);
        assert_eq!(out.len(), 1, "the crossing is a crossing, got {out:?}");
        assert_eq!(out[0].upper, 0, "the annotated bridge is above");
    }

    /// One pair, two places: a ramp weaving over its mainline crosses it twice
    /// and owes clearance at both. Keying the record on the pair alone kept the
    /// first and dropped the second, which is a deck with no demand under it.
    #[test]
    fn a_pair_that_crosses_twice_owes_two_clearances() {
        let len = 400.0;
        // A east-west bridge, and a north-south road that zigzags across it
        // twice — two separate intersections, one corridor pair, one level
        // pair.
        let a = corridor(
            0,
            6.0,
            46.0,
            true,
            len,
            vec![Span { arc0: 0.0, arc1: len, level: 1, kind: SpanKind::Bridge }],
        );
        let deg_x = |m: f64| m / (DEG_M * cos_lat());
        let deg_y = |m: f64| m / DEG_M;
        let nodes: Vec<Coord> = vec![
            Coord { x: 6.0 + deg_x(100.0), y: 46.0 - deg_y(50.0) },
            Coord { x: 6.0 + deg_x(100.0), y: 46.0 + deg_y(50.0) },
            Coord { x: 6.0 + deg_x(300.0), y: 46.0 + deg_y(50.0) },
            Coord { x: 6.0 + deg_x(300.0), y: 46.0 - deg_y(50.0) },
        ];
        let mut b = corridor(1, 6.0, 46.0, false, len, grade(len));
        b.arc = {
            let mut arc = vec![0.0];
            for w in nodes.windows(2) {
                let dx = (w[1].x - w[0].x) * cos_lat() * DEG_M;
                let dy = (w[1].y - w[0].y) * DEG_M;
                arc.push(arc.last().unwrap() + (dx * dx + dy * dy).sqrt());
            }
            arc
        };
        b.spans = grade(*b.arc.last().unwrap());
        b.nodes = nodes;
        let profiles = vec![flat(&a, 400.0), flat(&b, 400.0)];
        let scene = SceneGraph::new(vec![a, b]);
        let out = derive(&scene, &profiles, crate::priors::Stratum::S);
        assert_eq!(out.len(), 2, "both crossings owe clearance, got {out:?}");
        assert!(
            (out[0].upper_arc - out[1].upper_arc).abs() > 100.0,
            "the two records must be the two places, got {:?} and {:?}",
            out[0].upper_arc,
            out[1].upper_arc
        );
    }

    /// The plan index records the crossing on both corridors, with the *other*
    /// side's reach, and heights play no part.
    #[test]
    fn plan_crossings_record_both_sides_and_no_heights() {
        let len = 200.0;
        let a = corridor(0, 6.0, 46.0009, true, len, grade(len));
        let b = corridor(1, 6.0009, 46.0, false, len, grade(len));
        let scene = SceneGraph::new(vec![a, b]);
        let plan = plan_crossings(&scene);
        assert_eq!(plan[0].len(), 1, "one crossing on a: {:?}", plan[0]);
        assert_eq!(plan[1].len(), 1, "one crossing on b: {:?}", plan[1]);
        // Both corridors are 6 m wide: reach = 3 + ANNEX_SHOULDER_M.
        assert!((plan[0][0].1 - (3.0 + ANNEX_SHOULDER_M)).abs() < 1e-9);
        assert!((plan[1][0].1 - (3.0 + ANNEX_SHOULDER_M)).abs() < 1e-9);
    }

    /// The burial license fires only for a mapped tunnel crossed by an
    /// **at-grade** band: a crossing bridge flies over the mouth (S7) and a
    /// deeper tunnel passes below, and neither puts a roadbed on the ground
    /// the bore must pierce.
    #[test]
    fn covered_bores_gate_on_both_annotations() {
        let len = 200.0;
        let tunnel_spans = |lo: f64, hi: f64| {
            vec![
                Span { arc0: 0.0, arc1: lo, level: 0, kind: SpanKind::Grade },
                Span { arc0: lo, arc1: hi, level: -1, kind: SpanKind::Tunnel },
                Span { arc0: hi, arc1: len, level: 0, kind: SpanKind::Grade },
            ]
        };
        // The bore runs north–south; the other road crosses it mid-tunnel.
        let bore = corridor(0, 6.0009, 46.0, false, len, tunnel_spans(50.0, 150.0));
        let road = corridor(1, 6.0, 46.0009, true, len, grade(len));
        let scene = SceneGraph::new(vec![bore, road]);
        let plan = plan_index(&scene);
        let covered = covered_bores(&scene, &plan);
        assert_eq!(covered[0].len(), 1, "an at-grade band covers the bore: {covered:?}");
        let (w0, w1) = covered[0][0];
        assert!(w0 < w1 && w0 > 50.0 && w1 < 150.0, "window inside the tunnel: {covered:?}");
        assert!(covered[1].is_empty(), "the at-grade side carries no license");

        // A mapped bridge *above* the tunnel states the same ordering demand
        // as an at-grade band: the bore passes beneath it. (The Territet
        // road's 16 m deck snapped over the funicular's crossing — refusing
        // the license there left the rail a storey under the soffit.)
        let bore = corridor(0, 6.0009, 46.0, false, len, tunnel_spans(50.0, 150.0));
        let mut road = corridor(1, 6.0, 46.0009, true, len, grade(len));
        road.spans = vec![Span { arc0: 0.0, arc1: len, level: 1, kind: SpanKind::Bridge }];
        let scene = SceneGraph::new(vec![bore, road]);
        let plan = plan_index(&scene);
        assert_eq!(covered_bores(&scene, &plan)[0].len(), 1, "a bridge above still covers");

        // A deeper tunnel passes below and licenses nothing.
        let bore = corridor(0, 6.0009, 46.0, false, len, tunnel_spans(50.0, 150.0));
        let mut deeper = corridor(1, 6.0, 46.0009, true, len, grade(len));
        deeper.spans = vec![Span { arc0: 0.0, arc1: len, level: -2, kind: SpanKind::Tunnel }];
        let scene = SceneGraph::new(vec![bore, deeper]);
        let plan = plan_index(&scene);
        assert!(covered_bores(&scene, &plan)[0].is_empty(), "a deeper bore passes below");

        // A crossing outside the tunnel span licenses nothing either.
        let bore = corridor(0, 6.0009, 46.0, false, len, tunnel_spans(120.0, 150.0));
        let road = corridor(1, 6.0, 46.0009, true, len, grade(len));
        let scene = SceneGraph::new(vec![bore, road]);
        let plan = plan_index(&scene);
        assert!(covered_bores(&scene, &plan)[0].is_empty(), "the crossing is in the open");
    }

    /// A junction is a meeting, not a crossing, in the plan index too.
    #[test]
    fn plan_crossings_exempt_a_shared_connector() {
        let len = 200.0;
        let mut a = corridor(0, 6.0, 46.0009, true, len, grade(len));
        let mut b = corridor(1, 6.0009, 46.0, false, len, grade(len));
        a.connectors = vec![77];
        b.connectors = vec![77];
        let meet = Coord { x: b.nodes[0].x, y: a.nodes[0].y };
        a.nodes[2] = meet;
        b.nodes[2] = meet;
        let scene = SceneGraph::new(vec![a, b]);
        let plan = plan_crossings(&scene);
        assert!(plan[0].is_empty() && plan[1].is_empty(), "{plan:?}");
    }

    /// Derivation is a function of the model: same scene, same answer, in the
    /// same order (I5).
    #[test]
    fn derivation_is_deterministic() {
        let len = 200.0;
        let a = corridor(0, 6.0, 46.0009, true, len, grade(len));
        let b = corridor(1, 6.0009, 46.0, false, len, grade(len));
        let c = corridor(2, 6.0018, 46.0, false, len, grade(len));
        let profiles = vec![flat(&a, 412.0), flat(&b, 400.0), flat(&c, 400.0)];
        let scene = SceneGraph::new(vec![a, b, c]);
        let first = derive(&scene, &profiles, crate::priors::Stratum::S);
        let again = derive(&scene, &profiles, crate::priors::Stratum::S);
        let key = |xs: &[Crossing]| {
            xs.iter().map(|x| (x.upper, x.lower, x.upper_arc.to_bits())).collect::<Vec<_>>()
        };
        assert_eq!(key(&first), key(&again));
        assert_eq!(first.len(), 2, "both north-south roads pass under");
    }
}
