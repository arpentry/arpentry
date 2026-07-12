//! Clearance constraints at crossings (docs/GENERATION.md invariant 3,
//! scenario S4).
//!
//! Level ordinals give an ordering, never heights; this pass turns the
//! ordering into geometry. Wherever a corridor's bridge span crosses another
//! feature, the deck's *underside* must clear the crossed surface by a
//! class-appropriate gap. The upper profile is raised with a raise-only tent
//! — peak at the crossing, shoulders falling at the class's grade until they
//! meet the existing profile — so multiple crossings compose by `max` and
//! nothing is ever pushed down.
//!
//! Crossings are processed in ascending `upper_level` tiers: a `+2` deck
//! clears a `+1` deck only after the `+1` deck has its final height, so
//! stacked interchanges resolve bottom-up. After each tier the affected span
//! ramps are refit and a terminal clamp re-checks every crossing — the ramp
//! fit may smooth a lift away, and the clamp *guarantees* the inequality
//! (define the error out of existence).
//!
//! The lifted at-grade approaches this leaves above the natural terrain are
//! the earthwork demand the ground stage reads (`crate::ground::derive`): the
//! embankments that carry the road up to the deck (D3).

use crate::priors::{self, RAMP_GRADE, TUNNEL_COVER_M, TUNNEL_HEIGHT_M};
use crate::scene::{Crossing, SceneGraph, SpanKind, Underpass};

use super::profile::Profile;

/// Applies every vertical-order constraint to the solved profiles, in place:
/// first the underpasses (tunnel spans sink under the features above them,
/// S6), then the crossings (bridge spans lift over the features below them,
/// S4). Sinks run first so a bridge crossing a sunk feature reads its final
/// height.
pub fn apply(scene: &SceneGraph, profiles: &mut [Option<Profile>]) {
    apply_underpasses(scene, profiles);
    apply_crossings(scene, profiles);
}

/// Sinks every tunnel span below the features passing over it: the bore roof
/// (road + tunnel height) plus cover must fit under the crossing feature. On
/// flat ground this is the *only* signal that says how deep an underpass runs
/// — the terrain says nothing (docs/GENERATION.md S6).
fn apply_underpasses(scene: &SceneGraph, profiles: &mut [Option<Profile>]) {
    let mut touched: Vec<u32> = Vec::new();
    for u in &scene.underpasses {
        let Some((arc0, lo, hi, floor, grade)) = trough_demand(scene, u, profiles) else {
            continue;
        };
        let Some(up) = profiles[u.corridor as usize].as_mut() else { continue };
        // A demand beyond the plausible depth of a real underpass means the
        // level tags contradict the solved geometry (a mountain tunnel whose
        // profile stands high over the crossing road); it is dropped rather
        // than honoured — see `MAX_UNDERPASS_SINK_M`.
        let need = up.road_at_arc(arc0) - floor;
        if need <= 0.0 || need > priors::MAX_UNDERPASS_SINK_M {
            continue;
        }
        up.sink_trough(arc0, lo, hi, floor, grade);
        touched.push(u.corridor);
    }
    touched.sort_unstable();
    touched.dedup();
    for id in &touched {
        if let Some(p) = profiles[*id as usize].as_mut() {
            p.rebuild_deck();
        }
    }
    // Terminal clamp, mirroring the crossings pass: the ramp refit can smooth
    // the trough away; press the deck back down where it did (same
    // plausibility bound — a dropped sink must not re-enter as a deck clamp).
    for u in &scene.underpasses {
        let Some((arc0, lo, hi, floor, grade)) = trough_demand(scene, u, profiles) else {
            continue;
        };
        let Some(up) = profiles[u.corridor as usize].as_mut() else { continue };
        let excess = up.deck_at_arc(arc0) - floor;
        if excess <= 0.0 || excess > priors::MAX_UNDERPASS_SINK_M {
            continue;
        }
        up.lower_deck_to(arc0, lo, hi, floor, grade);
    }
}

/// The trough one underpass demands: the crossing's arc position, the flat
/// interval, the floor (bore roof + cover under the surface the crossing
/// feature rides on — its solved road, or the reference terrain for a plain
/// at-grade feature), and the ramp grade. `None` when the corridor has no
/// profile.
fn trough_demand(
    scene: &SceneGraph,
    u: &Underpass,
    profiles: &[Option<Profile>],
) -> Option<(f64, f64, f64, f64, f64)> {
    let profile = profiles.get(u.corridor as usize)?.as_ref()?;
    let over_m = u
        .over
        .filter(|&id| id != u.corridor)
        .and_then(|id| profiles.get(id as usize).and_then(|p| p.as_ref()))
        .map(|op| op.height_at(u.point.x, u.point.y))
        .unwrap_or_else(|| profile.surface_at(u.point.x, u.point.y));
    let floor = over_m - TUNNEL_HEIGHT_M - TUNNEL_COVER_M;
    let grade = scene.corridors[u.corridor as usize].class.grade_limit().unwrap_or(RAMP_GRADE);
    let arc0 = profile.arc_of(u.point.x, u.point.y);
    let (lo, hi) = trough_interval(scene, u, arc0);
    Some((arc0, lo, hi, floor, grade))
}

/// The interval an underpass trough holds its full depth across: the whole
/// annotated tunnel span when it is short (a cut-and-cover box runs depressed
/// end to end, S6), otherwise just the crossing feature's width around the
/// intersection — one crossing must not drag a driven tunnel's kilometres
/// down to its floor.
fn trough_interval(scene: &SceneGraph, u: &Underpass, arc0: f64) -> (f64, f64) {
    flat_interval(scene, u.corridor, SpanKind::Tunnel, u.arc, arc0, u.over)
}

/// The interval a constraint holds its full lift or depth across: the whole
/// annotated span when it is a short rigid box, otherwise just the crossed
/// feature's width around the intersection — one crossing must not drag
/// kilometres of structure to its height (see `STRUCTURE_BOX_MAX_M`).
fn flat_interval(
    scene: &SceneGraph,
    corridor: u32,
    kind: SpanKind,
    span_arc: f64,
    arc0: f64,
    crossed: Option<u32>,
) -> (f64, f64) {
    let c = &scene.corridors[corridor as usize];
    let span =
        c.spans.iter().find(|s| s.kind == kind && span_arc >= s.arc0 && span_arc <= s.arc1);
    if let Some(s) = span {
        if s.arc1 - s.arc0 <= priors::STRUCTURE_BOX_MAX_M {
            return (s.arc0, s.arc1);
        }
    }
    let apex = crossed
        .map(|id| {
            let x = &scene.corridors[id as usize];
            x.class.half_width_m(x.link)
        })
        .unwrap_or_else(|| priors::RoadClass::Minor.half_width_m(false))
        + priors::EARTHWORK_SHOULDER_M;
    (arc0 - apex, arc0 + apex)
}

/// Processing rank of every corridor from the crossing DAG: a corridor is
/// ranked strictly above every corridor its deck passes over, so a lower deck
/// reaches its final height before the deck above reads it. The rank is derived
/// from the actual crossing pairs (an edge lower → upper), not the absolute
/// level ordinal, so it stays correct where the level tags don't form a
/// consistent global order.
///
/// Cyclic constraints — A over B over C over A, contradictory tags — cannot be
/// satisfied; the cycle is broken at its weakest edge (the smallest level gap),
/// which is logged and dropped, so a bad datum costs one clearance rather than
/// hanging the solve (invariant 6). Kahn's algorithm with longest-path
/// layering; corridors in no crossing keep rank 0.
fn corridor_ranks(scene: &SceneGraph) -> Vec<u32> {
    let n = scene.corridors.len();
    // Edges lower → upper, with the level gap as the constraint's strength.
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut indeg = vec![0u32; n];
    let mut edges: Vec<(usize, usize, i64)> = Vec::new();
    for c in &scene.crossings {
        if let Some(l) = c.lower {
            let (lo, up) = (l as usize, c.upper as usize);
            if lo != up && lo < n && up < n {
                edges.push((lo, up, (c.upper_level - c.lower_level).abs()));
            }
        }
    }
    edges.sort_unstable();
    edges.dedup_by_key(|&mut (lo, up, _)| (lo, up));
    for &(lo, up, _) in &edges {
        adj[lo].push(up);
        indeg[up] += 1;
    }
    let mut rank = vec![0u32; n];
    let mut queue: Vec<usize> = (0..n).filter(|&v| indeg[v] == 0).collect();
    let mut processed = 0usize;
    let involved = edges.iter().flat_map(|&(lo, up, _)| [lo, up]).collect::<std::collections::HashSet<_>>().len();
    while processed < involved {
        while let Some(u) = queue.pop() {
            processed += 1;
            for &v in &adj[u] {
                rank[v] = rank[v].max(rank[u] + 1);
                indeg[v] -= 1;
                if indeg[v] == 0 {
                    queue.push(v);
                }
            }
        }
        if processed >= involved {
            break;
        }
        // Stalled on a cycle: break the weakest still-blocked edge and resume.
        let Some(&(lo, up, gap)) =
            edges.iter().filter(|&&(_, up, _)| indeg[up] > 0).min_by_key(|&&(_, _, g)| g)
        else {
            break; // no breakable edge left (defensive)
        };
        eprintln!(
            "warning: cyclic crossing constraint (corridors {lo} over {up}, level gap {gap}); \
             dropping the weakest edge to break the cycle"
        );
        indeg[up] -= 1;
        // Remove the edge so it can't be picked again.
        if let Some(pos) = adj[lo].iter().position(|&v| v == up) {
            adj[lo].swap_remove(pos);
        }
        edges.retain(|&(a, b, _)| !(a == lo && b == up));
        if indeg[up] == 0 {
            queue.push(up);
        }
    }
    rank
}

/// Lifts every bridge span over the features it crosses (invariant 3).
fn apply_crossings(scene: &SceneGraph, profiles: &mut [Option<Profile>]) {
    // Rank order from the crossing DAG: a lower corridor is finalized before
    // the deck above it; within a rank the detector's deterministic order
    // (stable sort). Robust to inconsistent level tags and to cycles.
    let ranks = corridor_ranks(scene);
    let key = |c: &Crossing| ranks[c.upper as usize];
    let mut order: Vec<&Crossing> = scene.crossings.iter().collect();
    order.sort_by_key(|c| key(c));

    let mut i = 0;
    while i < order.len() {
        let tier = key(order[i]);
        let tier_end =
            order[i..].iter().position(|c| key(c) != tier).map_or(order.len(), |p| i + p);
        let tier_crossings = &order[i..tier_end];

        // Lift every crossing of the tier, then refit the touched decks.
        let mut touched: Vec<u32> = Vec::new();
        for c in tier_crossings {
            let Some(need) = required_deck_m(c, profiles) else { continue };
            let Some(up) = profiles[c.upper as usize].as_mut() else { continue };
            let grade =
                scene.corridors[c.upper as usize].class.grade_limit().unwrap_or(RAMP_GRADE);
            let arc0 = up.arc_of(c.point.x, c.point.y);
            // A demand beyond the plausible lift of a real overpass means the
            // crossing geometry contradicts the solved profile (a path mapped
            // across a viaduct's plan line high on a flank); it is dropped
            // rather than honoured — see `MAX_CLEARANCE_LIFT_M`.
            let deficit = need - up.road_at_arc(arc0);
            if deficit <= 0.0 || deficit > priors::MAX_CLEARANCE_LIFT_M {
                continue;
            }
            let (lo, hi) = flat_interval(scene, c.upper, SpanKind::Bridge, c.upper_arc, arc0, c.lower);
            up.raise_crest(arc0, lo, hi, need, grade);
            touched.push(c.upper);
        }
        touched.sort_unstable();
        touched.dedup();
        for id in &touched {
            if let Some(p) = profiles[*id as usize].as_mut() {
                p.rebuild_deck();
            }
        }
        // Terminal clamp: the ramp fit is least-squares, so a lift can be
        // smoothed below the requirement; raise the deck back up where it was
        // (same plausibility bound — a dropped lift must not re-enter as a
        // deck clamp).
        for c in tier_crossings {
            let Some(need) = required_deck_m(c, profiles) else { continue };
            let Some(up) = profiles[c.upper as usize].as_mut() else { continue };
            let grade =
                scene.corridors[c.upper as usize].class.grade_limit().unwrap_or(RAMP_GRADE);
            let arc0 = up.arc_of(c.point.x, c.point.y);
            let deficit = need - up.deck_at_arc(arc0);
            if deficit <= 0.0 || deficit > priors::MAX_CLEARANCE_LIFT_M {
                continue;
            }
            let (lo, hi) = flat_interval(scene, c.upper, SpanKind::Bridge, c.upper_arc, arc0, c.lower);
            up.raise_deck_to(arc0, lo, hi, need, grade);
        }
        i = tier_end;
    }
}

/// The deck-top height the upper profile needs at the crossing: the crossed
/// surface, plus the class clearance under the deck, plus the deck slab
/// itself. `None` when the upper corridor has no profile (nothing to raise).
fn required_deck_m(c: &Crossing, profiles: &[Option<Profile>]) -> Option<f64> {
    let up = profiles.get(c.upper as usize)?.as_ref()?;
    // The crossed surface: the lower corridor's solved road where it has one;
    // otherwise the crossed feature is at grade and the reference terrain
    // stands in for it (an at-grade road lies on the ground by invariant 4).
    let lower_m = c
        .lower
        .and_then(|id| profiles.get(id as usize).and_then(|p| p.as_ref()))
        .map(|lp| lp.height_at(c.point.x, c.point.y))
        .unwrap_or_else(|| up.surface_at(c.point.x, c.point.y));
    Some(lower_m + priors::clearance_m(c.lower_kind) + priors::DECK_THICKNESS_M)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::RoadClass;
    use crate::scene::{Corridor, CrossedKind, SegmentRef, Span, SpanKind};
    use geo_types::Coord;

    /// A flat-ground scene: one east-west corridor with a bridge span in the
    /// middle (the S4 overpass), crossed at its centre by an unmodeled road.
    fn overpass_scene() -> (SceneGraph, Vec<Option<Profile>>) {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 1000.0;
        let deg = len_m / (crate::scene::DEG_M * cos_lat);
        let n = 41;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 450.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 450.0, arc1: 550.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 550.0, arc1: 1000.0, level: 0, kind: SpanKind::Grade },
        ];
        let corridor = Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc,
            cos_lat,
            class: RoadClass::Secondary,
            link: false,
            spans: spans.clone(),
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        };
        let profile =
            super::super::profile::solve(&nodes, &spans, None, &mut |_| 372.0).expect("profile");
        let mid = Coord { x: 6.0 + deg * 0.5, y: 46.0 };
        let mut scene = SceneGraph::new(vec![corridor]);
        scene.crossings = vec![Crossing {
            upper: 0,
            upper_arc: 500.0,
            point: mid,
            lower: None,
            lower_kind: CrossedKind::Road,
            upper_level: 1,
            lower_level: 0,
        }];
        (scene, vec![Some(profile)])
    }

    #[test]
    fn flat_overpass_lifts_to_clearance_over_the_crossed_road() {
        // S4: before this pass the deck lies flush at grade (372 m); after it
        // the deck top must be at least ground + clearance + slab.
        let (scene, mut profiles) = overpass_scene();
        let c = scene.crossings[0];
        let before = profiles[0].as_ref().unwrap().deck_height_at(c.point.x, c.point.y);
        assert!((before - 372.0).abs() < 0.5, "flat overpass starts at grade");

        apply(&scene, &mut profiles);
        let p = profiles[0].as_ref().unwrap();
        let need = 372.0 + priors::clearance_m(CrossedKind::Road) + priors::DECK_THICKNESS_M;
        assert!(
            p.deck_height_at(c.point.x, c.point.y) >= need - 1e-6,
            "deck {} must clear {}",
            p.deck_height_at(c.point.x, c.point.y),
            need
        );
        // The approaches ramp up from the flat ground — the earthwork demand.
        let approach = Coord { x: c.point.x - 100.0 / (crate::scene::DEG_M * p_cos()), y: 46.0 };
        let h = p.height_at(approach.x, approach.y);
        assert!(h > 372.5, "approach should be lifted onto an embankment, got {h}");
        // Far ends stay anchored on the ground.
        assert!((p.height_at(6.0, 46.0) - 372.0).abs() < 0.5);
    }

    fn p_cos() -> f64 {
        46.0_f64.to_radians().cos()
    }

    #[test]
    fn flat_ground_underpass_sinks_below_the_crossing_road() {
        // S6: a tunnel span on flat ground under an at-grade road. The
        // terrain says nothing — the crossing feature forces the sink: the
        // bore roof plus cover must fit under the ground the road rides on,
        // and the sunk road makes the gap negative so a bore exists at all.
        let cos_lat = p_cos();
        let len_m = 1000.0;
        let deg = len_m / (crate::scene::DEG_M * cos_lat);
        let n = 41;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 420.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 420.0, arc1: 580.0, level: -1, kind: SpanKind::Tunnel },
            Span { arc0: 580.0, arc1: 1000.0, level: 0, kind: SpanKind::Grade },
        ];
        let mut scene = SceneGraph::new(vec![Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc,
            cos_lat,
            class: RoadClass::Secondary,
            link: false,
            spans: spans.clone(),
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }]);
        let mid = Coord { x: 6.0 + deg * 0.5, y: 46.0 };
        scene.underpasses = vec![crate::scene::Underpass {
            corridor: 0,
            arc: 500.0,
            point: mid,
            over: None,
            over_level: 0,
            under_level: -1,
        }];

        let mut profiles =
            vec![super::super::profile::solve(&nodes, &spans, None, &mut |_| 372.0)];
        apply(&scene, &mut profiles);
        let p = profiles[0].as_ref().unwrap();
        let ceiling = 372.0 - priors::TUNNEL_HEIGHT_M - priors::TUNNEL_COVER_M;
        assert!(
            p.deck_height_at(mid.x, mid.y) <= ceiling + 1e-6,
            "road {} must sink below {ceiling}",
            p.deck_height_at(mid.x, mid.y)
        );
        // The sunk span is genuinely buried now: a bore can exist (S6→S5).
        assert!(
            p.height_at(mid.x, mid.y) - p.surface_at(mid.x, mid.y) < 0.0,
            "the underpass gap must be negative"
        );
        // The descent ramps cut below grade on the approaches (sampled just
        // outside the tunnel span, inside the ramp), and the far ends stay
        // anchored at the ground.
        let approach = Coord { x: mid.x - 100.0 / (crate::scene::DEG_M * cos_lat), y: 46.0 };
        let h = p.height_at(approach.x, approach.y);
        assert!(h < 371.5, "approach should descend into the cut, got {h}");
        assert!((p.height_at(6.0, 46.0) - 372.0).abs() < 0.5);
    }

    #[test]
    fn a_portal_crossing_leaves_a_long_tunnels_interior_alone() {
        // Regression: a climbing mountain tunnel crossed by a road just above
        // its low portal. The old span-wide flat sink dragged the whole bore
        // — and the far portal, hundreds of metres up — down to that one
        // crossing's floor. The trough is local: the crossing dips, the
        // interior and the upper anchor hold their grade.
        let cos_lat = p_cos();
        let len_m = 2000.0;
        let deg = len_m / (crate::scene::DEG_M * cos_lat);
        let n = 81;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 200.0, arc1: 1800.0, level: -1, kind: SpanKind::Tunnel },
            Span { arc0: 1800.0, arc1: 2000.0, level: 0, kind: SpanKind::Grade },
        ];
        let mut scene = SceneGraph::new(vec![Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc,
            cos_lat,
            class: RoadClass::Minor,
            link: false,
            spans: spans.clone(),
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }]);
        // Terrain climbs 15 % — the road chords the same line between its
        // at-grade anchors, so road == terrain along the whole corridor.
        let mut elev =
            |c: Coord| 400.0 + 0.15 * (c.x - 6.0) * crate::scene::DEG_M * cos_lat;
        let crossing = Coord { x: 6.0 + deg * 0.15, y: 46.0 }; // arc 300
        scene.underpasses = vec![crate::scene::Underpass {
            corridor: 0,
            arc: 300.0,
            point: crossing,
            over: None,
            over_level: 0,
            under_level: -1,
        }];

        let mut profiles = vec![super::super::profile::solve(&nodes, &spans, None, &mut elev)];
        apply(&scene, &mut profiles);
        let p = profiles[0].as_ref().unwrap();
        let floor = 445.0 - priors::TUNNEL_HEIGHT_M - priors::TUNNEL_COVER_M;
        assert!(
            p.deck_at_arc(300.0) <= floor + 1e-6,
            "deck {} must dip below {floor} at the crossing",
            p.deck_at_arc(300.0)
        );
        // The bore's interior and the upper anchor keep the road's own grade.
        assert!(
            (p.road_at_arc(1000.0) - 550.0).abs() < 0.5,
            "mid-bore {} must hold the grade, not the crossing's floor",
            p.road_at_arc(1000.0)
        );
        assert!(
            (p.road_at_arc(1900.0) - 685.0).abs() < 0.5,
            "the upper approach {} must stay anchored",
            p.road_at_arc(1900.0)
        );
    }

    #[test]
    fn an_absurd_underpass_demand_is_dropped() {
        // A tagged tunnel whose solved profile stands far above the road that
        // crosses its plan line: the level tags and the geometry contradict
        // each other, and honouring the sink would drag the profile hundreds
        // of metres down. Beyond MAX_UNDERPASS_SINK_M the demand is dropped.
        let cos_lat = p_cos();
        let len_m = 1000.0;
        let deg = len_m / (crate::scene::DEG_M * cos_lat);
        let n = 41;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 420.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 420.0, arc1: 580.0, level: -1, kind: SpanKind::Tunnel },
            Span { arc0: 580.0, arc1: 1000.0, level: 0, kind: SpanKind::Grade },
        ];
        let mid = Coord { x: 6.0 + deg * 0.5, y: 46.0 };
        let over_nodes: Vec<Coord> = (0..n)
            .map(|i| Coord { x: mid.x, y: 45.996 + 0.008 * i as f64 / (n - 1) as f64 })
            .collect();
        let over_arc: Vec<f64> =
            (0..n).map(|i| 0.008 * crate::scene::DEG_M * i as f64 / (n - 1) as f64).collect();
        let mut scene = SceneGraph::new(vec![
            Corridor {
                id: 0,
                nodes: nodes.clone(),
                arc,
                cos_lat,
                class: RoadClass::Secondary,
                link: false,
                spans: spans.clone(),
                segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
                connectors: vec![],
            },
            Corridor {
                id: 1,
                nodes: over_nodes.clone(),
                arc: over_arc,
                cos_lat,
                class: RoadClass::Secondary,
                link: false,
                spans: vec![],
                segments: vec![SegmentRef { source: 2, node0: 0, node1: n - 1, properties: vec![] }],
                connectors: vec![],
            },
        ]);
        scene.underpasses = vec![crate::scene::Underpass {
            corridor: 0,
            arc: 500.0,
            point: mid,
            over: Some(1),
            over_level: 0,
            under_level: -1,
        }];

        // The tunnel corridor solves at 372 m; the "crossing" road at 100 m —
        // a 265 m sink demand, far past any plausible underpass.
        let mut profiles = vec![
            super::super::profile::solve(&nodes, &spans, None, &mut |_| 372.0),
            super::super::profile::solve(&over_nodes, &[], None, &mut |_| 100.0),
        ];
        apply(&scene, &mut profiles);
        let p = profiles[0].as_ref().unwrap();
        assert!(
            (p.height_at(mid.x, mid.y) - 372.0).abs() < 1e-6,
            "the contradictory sink must be dropped, road moved to {}",
            p.height_at(mid.x, mid.y)
        );
        assert!((p.deck_height_at(mid.x, mid.y) - 372.0).abs() < 1e-6);
    }

    #[test]
    fn an_absurd_clearance_demand_is_dropped() {
        // The lift twin of the dropped sink: a feature whose solved profile
        // stands far above the bridge that "crosses" it in plan (a road high
        // on a flank over a viaduct's line). Honouring it once flattened a
        // 2 km viaduct at the demand; beyond MAX_CLEARANCE_LIFT_M it is
        // dropped and the bridge keeps its own profile.
        let (mut scene, mut profiles) = overpass_scene();
        let cos_lat = p_cos();
        let deg = 1000.0 / (crate::scene::DEG_M * cos_lat);
        let mid_x = 6.0 + deg * 0.5;
        let n = 41;
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord { x: mid_x, y: 45.996 + 0.008 * i as f64 / (n - 1) as f64 })
            .collect();
        let arc: Vec<f64> =
            (0..n).map(|i| 0.008 * crate::scene::DEG_M * i as f64 / (n - 1) as f64).collect();
        let profile =
            super::super::profile::solve(&nodes, &[], None, &mut |_| 500.0).expect("profile");
        scene.corridors.push(Corridor {
            id: 1,
            nodes,
            arc,
            cos_lat,
            class: RoadClass::Secondary,
            link: false,
            spans: vec![],
            segments: vec![SegmentRef { source: 2, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        });
        profiles.push(Some(profile));
        let mid = Coord { x: mid_x, y: 46.0 };
        scene.crossings = vec![Crossing {
            upper: 0,
            upper_arc: 500.0,
            point: mid,
            lower: Some(1),
            lower_kind: CrossedKind::Road,
            upper_level: 1,
            lower_level: 0,
        }];

        // The bridge solves at 372 m; the "crossed" road at 500 m — a 134.5 m
        // lift demand, far past any plausible overpass.
        apply(&scene, &mut profiles);
        let p = profiles[0].as_ref().unwrap();
        assert!(
            (p.height_at(mid.x, mid.y) - 372.0).abs() < 1e-6,
            "the contradictory lift must be dropped, road moved to {}",
            p.height_at(mid.x, mid.y)
        );
        assert!((p.deck_height_at(mid.x, mid.y) - 372.0).abs() < 1e-6);
    }

    #[test]
    fn clearance_stacks_over_a_solved_lower_corridor() {
        // Two crossings on one scene: corridor 0's bridge over ground (tier 1
        // handled first), then a +2 corridor over corridor 0 — its deck must
        // clear corridor 0's *lifted* height, not the ground.
        let (mut scene, mut profiles) = overpass_scene();
        // A second corridor crossing perpendicular over the first's bridge.
        let cos_lat = p_cos();
        let deg = 1000.0 / (crate::scene::DEG_M * cos_lat);
        let mid_x = 6.0 + deg * 0.5;
        let n = 41;
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord { x: mid_x, y: 45.996 + 0.008 * i as f64 / (n - 1) as f64 })
            .collect();
        let arc: Vec<f64> =
            (0..n).map(|i| 0.008 * crate::scene::DEG_M * i as f64 / (n - 1) as f64).collect();
        let total = *arc.last().unwrap();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.4 * total, level: 0, kind: SpanKind::Grade },
            Span { arc0: 0.4 * total, arc1: 0.6 * total, level: 2, kind: SpanKind::Bridge },
            Span { arc0: 0.6 * total, arc1: total, level: 0, kind: SpanKind::Grade },
        ];
        let profile =
            super::super::profile::solve(&nodes, &spans, None, &mut |_| 372.0).expect("profile");
        scene.corridors.push(Corridor {
            id: 1,
            nodes,
            arc,
            cos_lat,
            class: RoadClass::Secondary,
            link: false,
            spans,
            segments: vec![SegmentRef { source: 2, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        });
        profiles.push(Some(profile));
        let mid = Coord { x: mid_x, y: 46.0 };
        scene.crossings.push(Crossing {
            upper: 1,
            upper_arc: 0.0,
            point: mid,
            lower: Some(0),
            lower_kind: CrossedKind::Road,
            upper_level: 2,
            lower_level: 1,
        });

        apply(&scene, &mut profiles);
        let lower_deck = profiles[0].as_ref().unwrap().deck_height_at(mid.x, mid.y);
        let upper_deck = profiles[1].as_ref().unwrap().deck_height_at(mid.x, mid.y);
        let gap = upper_deck - lower_deck;
        assert!(
            gap >= priors::clearance_m(CrossedKind::Road) + priors::DECK_THICKNESS_M - 1e-6,
            "stacked decks must keep clearance, gap {gap}"
        );
        assert!(lower_deck > 372.0 + 5.0, "the lower deck itself is lifted over the ground");
    }

    /// A scene of `n` bare corridors carrying the given crossings, for testing
    /// the rank DAG in isolation: `(upper, lower, upper_level, lower_level)`.
    fn scene_with_crossings(n: u32, xs: &[(u32, Option<u32>, i64, i64)]) -> SceneGraph {
        let cos_lat = p_cos();
        let bare = |id: u32| Corridor {
            id,
            nodes: vec![Coord { x: 6.0, y: 46.0 }, Coord { x: 6.01, y: 46.0 }],
            arc: vec![0.0, 100.0],
            cos_lat,
            class: RoadClass::Secondary,
            link: false,
            spans: vec![],
            segments: vec![SegmentRef { source: id as u64, node0: 0, node1: 1, properties: vec![] }],
            connectors: vec![],
        };
        let mut scene = SceneGraph::new((0..n).map(bare).collect());
        scene.crossings = xs
            .iter()
            .map(|&(upper, lower, upper_level, lower_level)| Crossing {
                upper,
                upper_arc: 50.0,
                point: Coord { x: 6.005, y: 46.0 },
                lower,
                lower_kind: CrossedKind::Road,
                upper_level,
                lower_level,
            })
            .collect();
        scene
    }

    #[test]
    fn ranks_order_stacked_crossings_bottom_up() {
        // C under B under A (edges C→B, B→A): the ranks must climb C < B < A so
        // the solve finalizes the lower deck before the one above reads it.
        let scene = scene_with_crossings(3, &[(2, Some(1), 2, 1), (1, Some(0), 1, 0)]);
        let r = corridor_ranks(&scene);
        assert!(r[0] < r[1] && r[1] < r[2], "ranks {r:?} must be C < B < A");
    }

    #[test]
    fn ranks_stay_correct_when_level_ordinals_disagree() {
        // Both crossings tagged the same absolute ordinal (1 over 0 twice), but
        // the pairs still stack B over A and C over B: the DAG rank orders them
        // where an absolute-level tier sort would flatten them into one tier.
        let scene = scene_with_crossings(3, &[(1, Some(0), 1, 0), (2, Some(1), 1, 0)]);
        let r = corridor_ranks(&scene);
        assert!(r[0] < r[1] && r[1] < r[2], "ranks {r:?} must stack from the pairs, not the ordinal");
    }

    #[test]
    fn ranks_break_a_cycle_without_hanging() {
        // A over B and B over A: contradictory tags. corridor_ranks must break
        // the cycle and return finite ranks instead of looping forever.
        let scene = scene_with_crossings(2, &[(0, Some(1), 1, 0), (1, Some(0), 1, 0)]);
        let r = corridor_ranks(&scene);
        assert_eq!(r.len(), 2, "the cycle is broken and every corridor is ranked");
    }
}
