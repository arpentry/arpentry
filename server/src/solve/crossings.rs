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
/// and one record survives per `(upper, lower, level pair)` — a shared vertex
/// of two adjacent edges would otherwise report twice.
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
    let mut seen: std::collections::HashSet<(u32, u32, i64, i64)> = std::collections::HashSet::new();
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
                // Sharing a connector means the two *meet*; their heights are
                // reconciled by the shared variable, not by a clearance.
                if c.connectors.iter().any(|k| other.connectors.binary_search(k).is_ok()) {
                    continue;
                }
                let (o_a, o_b) = (other.nodes[e.node], other.nodes[e.node + 1]);
                let Some((t, u)) = seg_intersect(a, b, o_a, o_b, c.cos_lat) else {
                    continue;
                };
                let point = Coord { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
                let arc_c = c.arc[i] + t * (c.arc[i + 1] - c.arc[i]);
                let arc_o = other.arc[e.node] + u * (other.arc[e.node + 1] - other.arc[e.node]);
                let Some(x) = order(scene, profiles, c.id, arc_c, e.corridor, arc_o, point) else {
                    continue;
                };
                // Only the stratum that must yield takes the constraint, and
                // only where the crossed side is not junior to it.
                let upper_s = scene.corridors[x.upper as usize].kind.stratum();
                let lower_s = x.lower.map_or(stratum, |l| scene.corridors[l as usize].kind.stratum());
                if upper_s != stratum || lower_s > stratum {
                    continue;
                }
                if seen.insert((x.upper, x.lower.unwrap_or(u32::MAX), x.upper_level, x.lower_level))
                {
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

    /// Features that share a connector *meet*. Their heights are reconciled by
    /// the shared variable, and demanding clearance there would lift a ramp off
    /// the road it joins.
    #[test]
    fn a_shared_connector_is_a_junction_not_a_crossing() {
        let len = 200.0;
        let mut a = corridor(0, 6.0, 46.0009, true, len, grade(len));
        let mut b = corridor(1, 6.0009, 46.0, false, len, grade(len));
        a.connectors = vec![77];
        b.connectors = vec![77];
        let profiles = vec![flat(&a, 412.0), flat(&b, 400.0)];
        let scene = SceneGraph::new(vec![a, b]);
        assert!(derive(&scene, &profiles, crate::priors::Stratum::S).is_empty());
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
