//! Stage 3 — the engineered ground (docs/GENERATION.md §6, invariant 1).
//!
//! One authoritative ground function that every later consumer reads: terrain
//! meshing, road draping, building founding, structure contact. The function
//! is the natural DEM plus the earthworks the solved model implies, applied
//! as local modifiers ([`modifiers::Earthworks`]).
//!
//! [`derive`] translates the solved model into earthworks: wherever a
//! profiled corridor's at-grade road departs the natural ground — a
//! grade-limited cut through a bump, the embankment ramp the clearance solver
//! demanded for an overpass approach — the ground is reshaped to carry it
//! (D3). Consumers are untouched: they already read through
//! [`sampler::GroundSampler`].

pub mod modifiers;
pub mod sampler;

use crate::priors::{
    EARTHWORK_BATTER, EARTHWORK_MIN_FEATHER_M, EARTHWORK_SHOULDER_M, MIN_EARTHWORK_M,
};
use crate::scene::SceneGraph;
use crate::solve::SolvedModel;

use modifiers::{Earthworks, EarthworkEdge};

/// The engineered ground: the single ground function of invariant 1. Queries
/// apply the covering earthworks to the raw DEM sample in a fixed global
/// order, so any two tiles (and any two zooms) derive identical ground for
/// shared world points.
pub struct GroundModel {
    earthworks: Earthworks,
}

impl GroundModel {
    /// A ground model with no earthworks: the raw DEM passes through.
    pub fn empty() -> GroundModel {
        GroundModel { earthworks: Earthworks::new(Vec::new()) }
    }

    /// Number of earthwork edges, for run stats.
    pub fn earthwork_count(&self) -> usize {
        self.earthworks.len()
    }

    pub fn earthworks(&self) -> &Earthworks {
        &self.earthworks
    }

    /// THE ground function: the engineered height at `(lon, lat)`, given the
    /// raw DEM sample `raw` for that point. `scratch` is the caller's reusable
    /// query buffer (see [`sampler::GroundSampler`]).
    pub fn height(&self, lon: f64, lat: f64, raw: f64, scratch: &mut Vec<u32>) -> f64 {
        if self.earthworks.is_empty() {
            return raw;
        }
        self.earthworks.height(lon, lat, raw, scratch)
    }
}

/// Derives the engineered ground from the solved model: one earthwork run per
/// at-grade stretch where the solved road departs the natural terrain by more
/// than [`MIN_EARTHWORK_M`].
pub fn derive(scene: &SceneGraph, solved: &SolvedModel) -> GroundModel {
    let mut edges: Vec<EarthworkEdge> = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        let nodes = p.nodes();
        let road = p.road_m();
        let terrain = p.terrain_m();
        let at_grade = p.at_grade();
        let half_width = c.class.half_width_m() + EARTHWORK_SHOULDER_M;

        let needs = |i: usize| at_grade[i] && (road[i] - terrain[i]).abs() > MIN_EARTHWORK_M;
        let mut i = 0;
        while i < nodes.len() {
            if !needs(i) {
                i += 1;
                continue;
            }
            // Maximal run of earthwork nodes, padded by one at-grade node on
            // each side so the reshaping eases in at natural ground.
            let start = i;
            while i < nodes.len() && needs(i) {
                i += 1;
            }
            let lo = if start > 0 && at_grade[start - 1] { start - 1 } else { start };
            let hi = if i < nodes.len() && at_grade[i] { i } else { i - 1 };
            for k in lo..hi {
                let lift = (road[k] - terrain[k]).abs().max((road[k + 1] - terrain[k + 1]).abs());
                edges.push(EarthworkEdge {
                    a: nodes[k],
                    b: nodes[k + 1],
                    target_a: road[k],
                    target_b: road[k + 1],
                    half_width_m: half_width,
                    feather_m: (EARTHWORK_BATTER * lift).max(EARTHWORK_MIN_FEATHER_M),
                    cos_lat: crate::scene::run_cos_lat(&[nodes[k], nodes[k + 1]]),
                });
            }
        }
    }
    GroundModel { earthworks: Earthworks::new(edges) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::RoadClass;
    use crate::scene::{Corridor, CrossedKind, Crossing, SegmentRef, Span, SpanKind, DEG_M};
    use geo_types::Coord;

    /// The S4 scene end-to-end through solve + derive: a flat-ground overpass
    /// leaves embankment approaches in the engineered ground.
    #[test]
    fn overpass_approaches_become_embankments() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 1000.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 41;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 450.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 450.0, arc1: 550.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 550.0, arc1: 1000.0, level: 0, kind: SpanKind::Grade },
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
        scene.crossings = vec![Crossing {
            upper: 0,
            upper_arc: 500.0,
            point: mid,
            lower: None,
            lower_kind: CrossedKind::Road,
            upper_level: 1,
            lower_level: 0,
        }];

        let mut profiles =
            vec![crate::solve::profile::solve(&nodes, &spans, None, &mut |_| 372.0)];
        crate::solve::crossings::apply(&scene, &mut profiles);
        let solved = crate::solve::SolvedModel::from_profiles(profiles, 14);
        let ground = derive(&scene, &solved);
        assert!(ground.earthwork_count() > 0, "the lifted approaches must become earthworks");

        // On the approach centerline (~80 m before the crossing, 30 m before
        // the span edge) the engineered ground rises to the solved road; far
        // away it is natural.
        let mut scratch = Vec::new();
        let approach = Coord { x: mid.x - 80.0 / (DEG_M * cos_lat), y: 46.0 };
        let road_there = solved.profile(0).unwrap().height_at(approach.x, approach.y);
        let h = ground.height(approach.x, approach.y, 372.0, &mut scratch);
        assert!(
            (h - road_there).abs() < 1e-6,
            "engineered ground {h} must meet the road {road_there}"
        );
        assert!(h > 372.5, "the approach is a real embankment, got {h}");
        let far = Coord { x: 6.0 + deg * 0.02, y: 46.0 };
        assert_eq!(ground.height(far.x, far.y, 372.0, &mut scratch), 372.0);
        // Under the bridge span itself the natural ground is untouched — the
        // deck stands on air, not on a berm.
        assert_eq!(ground.height(mid.x, mid.y, 372.0, &mut scratch), 372.0);
    }
}
