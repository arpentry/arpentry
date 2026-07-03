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
use crate::scene::{Crossing, SceneGraph};

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
        // The surface the crossing feature rides on: its solved road, or the
        // reference terrain for a plain at-grade feature.
        let Some(profile) = profiles[u.corridor as usize].as_ref() else { continue };
        let over_m = u
            .over
            .filter(|&id| id != u.corridor)
            .and_then(|id| profiles.get(id as usize).and_then(|p| p.as_ref()))
            .map(|op| op.height_at(u.point.x, u.point.y))
            .unwrap_or_else(|| profile.surface_at(u.point.x, u.point.y));
        let floor = over_m - TUNNEL_HEIGHT_M - TUNNEL_COVER_M;
        let grade = scene.corridors[u.corridor as usize].class.grade_limit().unwrap_or(RAMP_GRADE);
        let Some(up) = profiles[u.corridor as usize].as_mut() else { continue };
        let arc0 = up.arc_of(u.point.x, u.point.y);
        up.sink_tent(arc0, floor, grade);
        touched.push(u.corridor);
    }
    touched.sort_unstable();
    touched.dedup();
    for id in &touched {
        if let Some(p) = profiles[*id as usize].as_mut() {
            p.rebuild_deck();
        }
    }
    // Terminal clamp, mirroring the crossings pass.
    for u in &scene.underpasses {
        let Some(profile) = profiles[u.corridor as usize].as_ref() else { continue };
        let over_m = u
            .over
            .filter(|&id| id != u.corridor)
            .and_then(|id| profiles.get(id as usize).and_then(|p| p.as_ref()))
            .map(|op| op.height_at(u.point.x, u.point.y))
            .unwrap_or_else(|| profile.surface_at(u.point.x, u.point.y));
        let floor = over_m - TUNNEL_HEIGHT_M - TUNNEL_COVER_M;
        if let Some(up) = profiles[u.corridor as usize].as_mut() {
            let arc0 = up.arc_of(u.point.x, u.point.y);
            up.lower_span_to(arc0, floor);
        }
    }
}

/// Lifts every bridge span over the features it crosses (invariant 3).
fn apply_crossings(scene: &SceneGraph, profiles: &mut [Option<Profile>]) {
    // Ascending-tier order; within a tier the detector's deterministic order.
    let mut order: Vec<&Crossing> = scene.crossings.iter().collect();
    order.sort_by_key(|c| c.upper_level);

    let mut i = 0;
    while i < order.len() {
        let tier = order[i].upper_level;
        let tier_end = order[i..].iter().position(|c| c.upper_level != tier).map_or(order.len(), |p| i + p);
        let tier_crossings = &order[i..tier_end];

        // Lift every crossing of the tier, then refit the touched decks.
        let mut touched: Vec<u32> = Vec::new();
        for c in tier_crossings {
            let Some(need) = required_deck_m(c, profiles) else { continue };
            let Some(up) = profiles[c.upper as usize].as_mut() else { continue };
            let grade =
                scene.corridors[c.upper as usize].class.grade_limit().unwrap_or(RAMP_GRADE);
            let arc0 = up.arc_of(c.point.x, c.point.y);
            up.raise_tent(arc0, need, grade);
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
        // smoothed below the requirement; raise the whole span where it was.
        for c in tier_crossings {
            let Some(need) = required_deck_m(c, profiles) else { continue };
            if let Some(up) = profiles[c.upper as usize].as_mut() {
                let arc0 = up.arc_of(c.point.x, c.point.y);
                up.raise_span_to(arc0, need);
            }
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
        // The descent ramps cut below grade on the approaches, and the far
        // ends stay anchored at the ground.
        let approach = Coord { x: mid.x - 150.0 / (crate::scene::DEG_M * cos_lat), y: 46.0 };
        let h = p.height_at(approach.x, approach.y);
        assert!(h < 371.5, "approach should descend into the cut, got {h}");
        assert!((p.height_at(6.0, 46.0) - 372.0).abs() < 0.5);
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
}
