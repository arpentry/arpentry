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

/// The portals of every tunnel span of a corridor: the gap zero-crossings
/// bounding each span's buried run. A span whose road never dips below the
/// terrain yields none (nothing emerges because nothing is buried — the
/// degradation ladder drapes it). A buried run that reaches the corridor end
/// without surfacing yields no portal on that side (the bore runs out of
/// data, not out of the hill).
pub fn portals(profile: &Profile, spans: &[Span]) -> Vec<Portal> {
    let arc = profile.arc();
    let road = profile.road_m();
    let terrain = profile.terrain_m();
    let n = arc.len();
    let gap = |i: usize| road[i] - terrain[i];

    let mut out = Vec::new();
    for span in spans.iter().filter(|s| s.kind == SpanKind::Tunnel) {
        // Seed on any buried node inside the annotated span, then expand the
        // contiguous buried run outward — past the annotation edges (they are
        // mapper cuts, not geometry) but no further than the search reach. A
        // span with no buried node has nothing emerging (S10 degradation).
        let lo_arc = span.arc0 - PORTAL_MAX_M;
        let hi_arc = span.arc1 + PORTAL_MAX_M;
        let Some(seed) =
            (0..n).find(|&i| arc[i] >= span.arc0 && arc[i] <= span.arc1 && gap(i) < 0.0)
        else {
            continue;
        };
        let mut f = seed;
        while f > 0 && gap(f - 1) < 0.0 && arc[f - 1] >= lo_arc {
            f -= 1;
        }
        let mut l = seed;
        while l + 1 < n && gap(l + 1) < 0.0 && arc[l + 1] <= hi_arc {
            l += 1;
        }
        // Interpolate each bounding crossing, when the run does surface.
        if f > 0 && gap(f - 1) >= 0.0 {
            let t = gap(f - 1) / (gap(f - 1) - gap(f));
            let a = arc[f - 1] + t * (arc[f] - arc[f - 1]);
            out.push(Portal { arc: a, outward: -1.0, floor_m: profile.road_at_arc(a) - DECK_THICKNESS_M });
        }
        if l + 1 < n && gap(l + 1) >= 0.0 {
            let t = gap(l) / (gap(l) - gap(l + 1));
            let a = arc[l] + t * (arc[l + 1] - arc[l]);
            out.push(Portal { arc: a, outward: 1.0, floor_m: profile.road_at_arc(a) - DECK_THICKNESS_M });
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
    fn a_tunnel_over_flat_ground_has_no_portals() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 1000.0 / (DEG_M * cos_lat);
        let nodes: Vec<Coord> =
            (0..101).map(|i| Coord { x: 6.0 + deg * i as f64 / 100.0, y: 46.0 }).collect();
        let p = Profile::from_heights(&nodes, vec![100.0; 101], vec![95.0; 101]);
        assert!(portals(&p, &[span(300.0, 700.0)]).is_empty());
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
