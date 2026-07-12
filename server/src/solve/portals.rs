//! Portal placement — where a tunnel actually emerges from the ground
//! (docs/GENERATION.md S5).
//!
//! An annotation edge is where a mapper split the segment, not where the road
//! pierces the hillside; the geometric fact is the zero crossing of the
//! signed gap `road − terrain`. Each tunnel span's buried run is bounded by
//! its two crossings — searched outward past the annotation edges up to
//! [`PORTAL_MAX_M`], the same trust model the bore mesh uses. The solved
//! portals feed the ground stage (the carve that daylights the mouth); the
//! mesh finds the same crossings itself from the same profile, so the two
//! agree by construction.

use crate::priors::{DECK_THICKNESS_M, PORTAL_MAX_M};
use crate::scene::{Span, SpanKind};

use super::profile::Profile;

/// One solved portal: its arc position, which way "out of the hill" faces
/// along the corridor (`-1.0` toward decreasing arc, `+1.0` increasing), and
/// the bore floor height at the mouth.
#[derive(Debug, Clone, Copy)]
pub struct Portal {
    pub arc: f64,
    pub outward: f64,
    pub floor_m: f64,
}

/// The first node (in arc order) of the *dominant* buried run overlapping the
/// annotated span: among the maximal contiguous stretches where `road` runs
/// below `terrain` and that touch `[arc0, arc1]`, the one with the greatest
/// integrated burial (Σ −gap over the whole run). Returns that run's first
/// node that falls inside the annotation, so [`span_bounds`]' outward
/// expansion walks the winning run in both directions.
///
/// This is the guard against a shallow terrain graze (real relief noise, or a
/// brief emergence on the approach) sitting between the portal and the true
/// bore: scored by depth×length it cannot outweigh the run it belongs to, so
/// the tunnel is solved under the hill instead of collapsing onto the graze.
fn dominant_buried_seed(arc: &[f64], road: &[f64], terrain: &[f64], span: &Span) -> Option<usize> {
    let n = arc.len();
    let gap = |i: usize| road[i] - terrain[i];
    let in_span = |i: usize| arc[i] >= span.arc0 && arc[i] <= span.arc1;
    let mut best: Option<usize> = None;
    let mut best_score = 0.0_f64;
    let mut i = 0;
    while i < n {
        if gap(i) >= 0.0 {
            i += 1;
            continue;
        }
        // A maximal buried run [start, i); score its full extent but seed on
        // its first in-annotation node so the caller's expansion stays anchored
        // inside the mapper's span.
        let mut score = 0.0;
        let mut seed = None;
        while i < n && gap(i) < 0.0 {
            score += -gap(i);
            if seed.is_none() && in_span(i) {
                seed = Some(i);
            }
            i += 1;
        }
        if let Some(seed) = seed {
            if best.is_none() || score > best_score {
                best = Some(seed);
                best_score = score;
            }
        }
    }
    best
}

/// The buried run of one tunnel span: the arcs of the gap zero-crossings
/// bounding it, searched outward past the annotation edges up to
/// [`PORTAL_MAX_M`]. `None` when the span has no buried node (a tunnel tagged
/// over flat ground — nothing is buried, so nothing emerges). A side whose
/// run never surfaces within reach reports `None` for that crossing (the
/// bore runs out of data, not out of the hill).
pub fn span_bounds(profile: &Profile, span: &Span) -> Option<(Option<f64>, Option<f64>)> {
    let arc = profile.arc();
    let road = profile.road_m();
    let terrain = profile.terrain_m();
    let n = arc.len();
    let gap = |i: usize| road[i] - terrain[i];

    // Seed on the *dominant* buried run overlapping the annotation — not the
    // first buried node — then expand that run outward past the annotation
    // edges (mapper cuts, not geometry) but no further than the search reach.
    // Seeding on the first node let a shallow DEM-noise graze on the approach
    // capture the whole solve: the graze became the "tunnel" and the real,
    // deep run past it was re-covered as at-grade road painted over the massif
    // (docs/GENERATION.md S5, S10). The deepest run outscores a brief graze by
    // orders of magnitude, so the bore lands under the hill it belongs to.
    let lo_arc = span.arc0 - PORTAL_MAX_M;
    let hi_arc = span.arc1 + PORTAL_MAX_M;
    let seed = dominant_buried_seed(arc, road, terrain, span)?;
    let mut f = seed;
    while f > 0 && gap(f - 1) < 0.0 && arc[f - 1] >= lo_arc {
        f -= 1;
    }
    let mut l = seed;
    while l + 1 < n && gap(l + 1) < 0.0 && arc[l + 1] <= hi_arc {
        l += 1;
    }
    // Interpolate each bounding crossing, when the run does surface.
    let low = (f > 0 && gap(f - 1) >= 0.0).then(|| {
        let t = gap(f - 1) / (gap(f - 1) - gap(f));
        arc[f - 1] + t * (arc[f] - arc[f - 1])
    });
    let high = (l + 1 < n && gap(l + 1) >= 0.0).then(|| {
        let t = gap(l) / (gap(l) - gap(l + 1));
        arc[l] + t * (arc[l + 1] - arc[l])
    });
    Some((low, high))
}

/// The portals of every tunnel span of a corridor: the gap zero-crossings
/// bounding each span's buried run ([`span_bounds`]).
pub fn portals(profile: &Profile, spans: &[Span]) -> Vec<Portal> {
    let mut out = Vec::new();
    for span in spans.iter().filter(|s| s.kind == SpanKind::Tunnel) {
        let Some((low, high)) = span_bounds(profile, span) else {
            continue;
        };
        if let Some(a) = low {
            out.push(Portal { arc: a, outward: -1.0, floor_m: profile.road_at_arc(a) - DECK_THICKNESS_M });
        }
        if let Some(a) = high {
            out.push(Portal { arc: a, outward: 1.0, floor_m: profile.road_at_arc(a) - DECK_THICKNESS_M });
        }
    }
    out
}

/// Structure spans grown over the profile's absorbed stretches: where the
/// solve flipped at-grade nodes into a structure run (an infeasible anchor —
/// the annotation ended before the road reached the ground, see
/// `profile::solve`), the adjacent structure span is extended to cover them
/// and the grade span shrunk to match, so the deck/bore sweep and the paint
/// follow the solved geometry instead of the annotation. The span list stays
/// a partition of the corridor: each boundary moves, none overlap.
pub fn grow_spans(profile: &Profile, spans: &[Span]) -> Vec<Span> {
    let arc = profile.arc();
    let at_grade = profile.at_grade();
    if spans.len() < 2 || at_grade.is_empty() {
        return spans.to_vec();
    }
    let mut out = spans.to_vec();
    for i in 0..out.len() {
        if out[i].kind == SpanKind::Grade {
            continue;
        }
        // Backward over the preceding grade span's absorbed tail.
        if i > 0 && out[i - 1].kind == SpanKind::Grade {
            let mut a0 = out[i].arc0;
            for k in (0..arc.len()).rev() {
                if arc[k] >= out[i].arc0 {
                    continue;
                }
                if arc[k] <= out[i - 1].arc0 || at_grade[k] {
                    break;
                }
                a0 = arc[k];
            }
            if a0 < out[i].arc0 {
                out[i].arc0 = a0;
                out[i - 1].arc1 = a0;
            }
        }
        // Forward over the following grade span's absorbed head.
        if i + 1 < out.len() && out[i + 1].kind == SpanKind::Grade {
            let mut a1 = out[i].arc1;
            for k in 0..arc.len() {
                if arc[k] <= out[i].arc1 {
                    continue;
                }
                if arc[k] >= out[i + 1].arc1 || at_grade[k] {
                    break;
                }
                a1 = arc[k];
            }
            if a1 > out[i].arc1 {
                out[i].arc1 = a1;
                out[i + 1].arc0 = a1;
            }
        }
    }
    // A grade span fully absorbed from one side collapses to nothing: drop it.
    out.retain(|s| s.arc1 - s.arc0 > f64::EPSILON);
    out
}

/// Corridor spans reconciled with the solved geometry: structure spans grown
/// over the profile's absorbed stretches ([`grow_spans`]), then each tunnel
/// span clamped to its buried run (the solved portal crossings), and the freed
/// annotation slack — the stretch a mapper tagged "tunnel" where the road in
/// fact still runs above ground — is re-covered by grade spans, so the
/// approach up to a portal mouth is painted road instead of naked ground. A
/// tunnel with no buried run at all becomes grade end to end (the same
/// degradation the bore mesh applies, decided once here so paint and solids
/// agree). Only shrinking is reconciled: a buried run reaching *past* the
/// annotation is left to the bore sweep's own outward march, where the
/// neighbouring span's paint simply passes under the ground it is buried by.
pub fn reconcile_spans(profile: &Profile, spans: &[Span]) -> Vec<Span> {
    /// Shortest grade stub worth emitting, in metres — below this the piece
    /// quantizes away.
    const MIN_STUB_M: f64 = 0.25;
    let spans = grow_spans(profile, spans);
    let mut out = Vec::with_capacity(spans.len() + 4);
    for s in &spans {
        if s.kind != SpanKind::Tunnel {
            out.push(*s);
            continue;
        }
        let Some((low, high)) = span_bounds(profile, s) else {
            out.push(Span { level: 0, kind: SpanKind::Grade, ..*s });
            continue;
        };
        let a0 = low.map_or(s.arc0, |a| a.max(s.arc0));
        let a1 = high.map_or(s.arc1, |a| a.min(s.arc1));
        if a1 - a0 < MIN_STUB_M {
            out.push(Span { level: 0, kind: SpanKind::Grade, ..*s });
            continue;
        }
        if a0 - s.arc0 > MIN_STUB_M {
            out.push(Span { arc0: s.arc0, arc1: a0, level: 0, kind: SpanKind::Grade });
        }
        out.push(Span { arc0: a0, arc1: a1, ..*s });
        if s.arc1 - a1 > MIN_STUB_M {
            out.push(Span { arc0: a1, arc1: s.arc1, level: 0, kind: SpanKind::Grade });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::DEG_M;
    use geo_types::Coord;

    fn span(arc0: f64, arc1: f64) -> Span {
        Span { arc0, arc1, level: -1, kind: SpanKind::Tunnel }
    }

    /// A 1 km corridor with a hill in the middle: road flat at 100, terrain
    /// rising to 130 over the central third — buried between the flanks.
    fn hill() -> (Profile, f64) {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len = 1000.0;
        let deg = len / (DEG_M * cos_lat);
        let n = 201;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let road = vec![100.0; n];
        let terrain: Vec<f64> = (0..n)
            .map(|i| {
                let u = i as f64 / (n - 1) as f64;
                let d = (u - 0.5).abs();
                if d < 0.15 { 130.0 - d / 0.15 * 40.0 } else { 90.0 } // crosses 100 at d=0.1125
            })
            .collect();
        (Profile::from_heights(&nodes, road, terrain), len)
    }

    #[test]
    fn portals_sit_on_the_gap_zero_crossings() {
        let (p, len) = hill();
        // Annotation roughly over the buried middle (mapper slop included).
        let ps = portals(&p, &[span(0.42 * len, 0.58 * len)]);
        assert_eq!(ps.len(), 2, "a through-tunnel has two portals");
        // Crossings at u = 0.5 ± 0.1125.
        assert!((ps[0].arc - 387.5).abs() < 10.0, "west portal at {}", ps[0].arc);
        assert!((ps[1].arc - 612.5).abs() < 10.0, "east portal at {}", ps[1].arc);
        assert_eq!(ps[0].outward, -1.0);
        assert_eq!(ps[1].outward, 1.0);
        assert!((ps[0].floor_m - 98.5).abs() < 0.1, "floor = road − slab");
    }

    #[test]
    fn a_shallow_graze_does_not_capture_the_solve_from_the_real_run() {
        // Annotation covers a shallow DEM-noise graze (road 0.5 m under terrain
        // over ~20 m) on the approach, then the real 60 m-deep bore. span_bounds
        // must lock onto the deep run — not the graze that appears first in arc
        // order — so the portals land at the hill, the bore is built, and the
        // deep stretch is not painted as at-grade road over the massif.
        let cos_lat = 46.0_f64.to_radians().cos();
        let len = 1000.0;
        let deg = len / (DEG_M * cos_lat);
        let n = 201;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let road = vec![100.0; n];
        let terrain: Vec<f64> = (0..n)
            .map(|i| {
                let u = i as f64 / (n - 1) as f64;
                if (0.30..0.32).contains(&u) {
                    100.5 // shallow graze: 0.5 m of burial
                } else if (0.40..0.60).contains(&u) {
                    160.0 // the real bore: 60 m of burial
                } else {
                    90.0
                }
            })
            .collect();
        let p = Profile::from_heights(&nodes, road, terrain);
        let (low, high) =
            span_bounds(&p, &span(0.25 * len, 0.70 * len)).expect("a buried run exists");
        let lo = low.expect("west portal on the deep run");
        let hi = high.expect("east portal on the deep run");
        assert!((380.0..405.0).contains(&lo), "west portal at the hill, got {lo}");
        assert!((595.0..620.0).contains(&hi), "east portal at the hill, got {hi}");
    }

    #[test]
    fn a_tunnel_over_flat_ground_has_no_portals() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 1000.0 / (DEG_M * cos_lat);
        let nodes: Vec<Coord> =
            (0..101).map(|i| Coord { x: 6.0 + deg * i as f64 / 100.0, y: 46.0 }).collect();
        let p = Profile::from_heights(&nodes, vec![100.0; 101], vec![95.0; 101]);
        assert!(portals(&p, &[span(300.0, 700.0)]).is_empty());
    }

    #[test]
    fn reconciled_tunnel_shrinks_to_its_buried_run_with_grade_stubs() {
        // Annotation [0.40, 0.62] but the road is buried only over
        // [≈0.3875, ≈0.6125] of a 1 km corridor: the low side grows nothing
        // (crossing outside the annotation is left to the bore's own march),
        // the high side frees [crossing, 0.62] as a painted grade stub.
        let (p, len) = hill();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.40 * len, level: 0, kind: SpanKind::Grade },
            span(0.40 * len, 0.62 * len),
            Span { arc0: 0.62 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        let out = reconcile_spans(&p, &spans);
        assert_eq!(out.len(), 4, "tunnel + freed high-side stub: {out:?}");
        assert_eq!(out[1].kind, SpanKind::Tunnel);
        assert!((out[1].arc0 - 0.40 * len).abs() < 1e-9, "low side stays annotated");
        assert!((out[1].arc1 - 612.5).abs() < 10.0, "high side clamps to the crossing");
        assert_eq!(out[2].kind, SpanKind::Grade);
        assert!((out[2].arc1 - 0.62 * len).abs() < 1e-9, "stub re-covers the slack");
    }

    #[test]
    fn reconciled_flat_ground_tunnel_becomes_grade() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 1000.0 / (DEG_M * cos_lat);
        let nodes: Vec<Coord> =
            (0..101).map(|i| Coord { x: 6.0 + deg * i as f64 / 100.0, y: 46.0 }).collect();
        let p = Profile::from_heights(&nodes, vec![100.0; 101], vec![95.0; 101]);
        let out = reconcile_spans(&p, &[span(300.0, 700.0)]);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].kind, SpanKind::Grade, "nothing buried: paint it as road");
    }

    #[test]
    fn grown_spans_cover_the_absorbed_stretch() {
        // A grade-limited solve that absorbs a cliff into a bridge span (see
        // profile::tests::a_structure_ending_at_a_cliff_is_extended_not_pitched):
        // grow_spans must extend the bridge over the absorbed nodes and shrink
        // the following grade span to keep the partition.
        let cos_lat = 46.0_f64.to_radians().cos();
        let len = 4000.0;
        let deg = len / (DEG_M * cos_lat);
        let n = 512;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let mut elev = |c: Coord| {
            let x = (c.x - 6.0) / deg; // 0..1
            if x < 0.5 {
                100.0
            } else if x < 0.51 {
                100.0 + 3000.0 * (x - 0.5) // the wall: +30 m over ~40 m
            } else {
                130.0
            }
        };
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.3 * len, level: 0, kind: SpanKind::Grade },
            Span { arc0: 0.3 * len, arc1: 0.5 * len, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 0.5 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        let p = crate::solve::profile::solve(&nodes, &spans, Some(0.06), &mut elev)
            .expect("non-degenerate corridor");
        let out = grow_spans(&p, &spans);
        assert_eq!(out.len(), 3, "the partition keeps its three spans: {out:?}");
        assert_eq!(out[1].kind, SpanKind::Bridge);
        assert!(
            out[1].arc1 > 0.5 * len + 10.0,
            "bridge must grow over the absorbed wall, got arc1 {}",
            out[1].arc1
        );
        assert!((out[2].arc0 - out[1].arc1).abs() < 1e-9, "grade span shrinks to match");
        assert!((out[2].arc1 - len).abs() < 1e-9, "the far boundary is untouched");
        assert!((out[1].arc0 - 0.3 * len).abs() < 1e-9, "the feasible low side is untouched");
    }

    #[test]
    fn a_run_to_the_corridor_end_keeps_that_side_open() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 1000.0 / (DEG_M * cos_lat);
        let n = 101;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        // Buried from the very start, emerging at u≈0.6.
        let road = vec![100.0; n];
        let terrain: Vec<f64> = (0..n)
            .map(|i| if (i as f64 / (n - 1) as f64) < 0.6 { 120.0 } else { 90.0 })
            .collect();
        let p = Profile::from_heights(&nodes, road, terrain);
        let ps = portals(&p, &[span(0.0, 550.0)]);
        assert_eq!(ps.len(), 1, "only the emerging side gets a portal");
        assert_eq!(ps[0].outward, 1.0);
    }
}
