//! Vertical-consistency measurement — the number the solver must drive to zero
//! (docs/CONSISTENCY.md P0 / GENERATION.md invariants 2 & 3).
//!
//! Two defects are read straight off the solved model:
//!
//! - **Junction step** — at a shared connector two corridors *should* read one
//!   road height; the residual disagreement between a junction's members is a
//!   C0 break (invariant 2). The capped weld leaves these where it declines;
//!   the constraint-graph solver removes them by sharing the DOF.
//! - **Clearance violation** — a deck that fails to clear the feature it
//!   crosses by the class gap (invariant 3).
//!
//! Purely a read-only diagnostic: it changes no geometry, only reports how
//! consistent the model already is, so every later change has a number to beat.

use crate::priors::{clearance_m, DECK_THICKNESS_M};
use crate::scene::SceneGraph;

use super::SolvedModel;

/// A scene's vertical-consistency summary.
#[derive(Debug, Clone, Copy, Default)]
pub struct Consistency {
    /// Largest junction step: the max over junctions of `max − min` member road
    /// height, in metres.
    pub max_junction_step_m: f64,
    /// The 99th-percentile junction step, in metres — the bulk-of-the-tail
    /// figure a single outlier cannot dominate.
    pub p99_junction_step_m: f64,
    /// How many junctions disagree by more than [`STEP_THRESHOLD_M`].
    pub junction_steps_over: u64,
    /// Largest clearance shortfall at a crossing (required deck − actual deck),
    /// in metres; zero when every crossing clears.
    pub max_clearance_violation_m: f64,
}

/// A member disagreement at or above this counts as a step worth flagging.
const STEP_THRESHOLD_M: f64 = 0.5;

/// Measures the vertical consistency of `solved` against `scene`. Read-only.
pub fn measure(scene: &SceneGraph, solved: &SolvedModel) -> Consistency {
    let mut steps: Vec<f64> = Vec::with_capacity(scene.junctions.len());
    for j in &scene.junctions {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        let mut n = 0u32;
        for m in &j.members {
            if let Some(p) = solved.profile(m.corridor) {
                let h = p.road_at_arc(m.arc);
                lo = lo.min(h);
                hi = hi.max(h);
                n += 1;
            }
        }
        // A step needs at least two profiled members to disagree.
        if n >= 2 {
            steps.push(hi - lo);
        }
    }
    let junction_steps_over = steps.iter().filter(|&&s| s > STEP_THRESHOLD_M).count() as u64;
    let max_junction_step_m = steps.iter().cloned().fold(0.0, f64::max);
    let p99_junction_step_m = percentile(&mut steps, 0.99);

    let mut max_clearance_violation_m = 0.0f64;
    for c in &scene.crossings {
        let Some(up) = solved.profile(c.upper) else { continue };
        // The crossed surface: the lower corridor's solved road where it has a
        // profile, else the reference terrain (an at-grade feature lies on the
        // ground) — the same rule `crossings::required_deck_m` uses.
        let lower_m = c
            .lower
            .and_then(|id| solved.profile(id))
            .map(|lp| lp.height_at(c.point.x, c.point.y))
            .unwrap_or_else(|| up.surface_at(c.point.x, c.point.y));
        let required = lower_m + clearance_m(c.lower_kind) + DECK_THICKNESS_M;
        let actual = up.deck_height_at(c.point.x, c.point.y);
        max_clearance_violation_m = max_clearance_violation_m.max(required - actual);
    }

    Consistency {
        max_junction_step_m,
        p99_junction_step_m,
        junction_steps_over,
        max_clearance_violation_m,
    }
}

/// The `q`-quantile (0..1) of `xs` by the nearest-rank method; 0 for an empty
/// slice. Sorts `xs` in place (ascending) — the caller no longer needs order.
fn percentile(xs: &mut [f64], q: f64) -> f64 {
    if xs.is_empty() {
        return 0.0;
    }
    xs.sort_by(f64::total_cmp);
    let rank = (q * (xs.len() as f64 - 1.0)).round() as usize;
    xs[rank.min(xs.len() - 1)]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{Corridor, Junction, JunctionMember, SegmentRef};
    use crate::priors::RoadClass;
    use geo_types::Coord;

    fn cos_lat() -> f64 {
        46.0_f64.to_radians().cos()
    }

    /// A straight east-west corridor of `n` nodes over `len_m`, at latitude 46.
    fn corridor(id: u32, x0: f64, len_m: f64, n: usize) -> Corridor {
        let deg = len_m / (crate::scene::DEG_M * cos_lat());
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: x0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        Corridor {
            id,
            nodes,
            arc,
            cos_lat: cos_lat(),
            class: RoadClass::Motorway,
            class_key: String::new(),
            link: false,
            drivable: true,
            spans: vec![],
            segments: vec![SegmentRef { source: id as u64, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }
    }

    #[test]
    fn a_disagreeing_junction_reports_its_step() {
        // Two corridors meet at a connector but solved 3 m apart: the step is 3.
        let len = 200.0;
        let n = 26;
        let a = corridor(0, 6.0, len, n);
        let deg = len / (crate::scene::DEG_M * cos_lat());
        let b = corridor(1, 6.0 + deg, len, n);
        let point = *a.nodes.last().unwrap();
        let scene = {
            let mut s = SceneGraph::new(vec![a, b]);
            s.junctions = vec![Junction {
                point,
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: len },
                    JunctionMember { corridor: 1, arc: 0.0 },
                ],
            }];
            s
        };
        let an = scene.corridors[0].nodes.clone();
        let bn = scene.corridors[1].nodes.clone();
        let solved = SolvedModel::from_profiles(
            vec![
                Some(super::super::Profile::flat(&an, 400.0)),
                Some(super::super::Profile::flat(&bn, 403.0)),
            ],
            14,
        );
        let c = measure(&scene, &solved);
        assert!((c.max_junction_step_m - 3.0).abs() < 1e-6, "step {}", c.max_junction_step_m);
        assert_eq!(c.junction_steps_over, 1);
    }

    #[test]
    fn an_agreeing_junction_reports_no_step() {
        let len = 200.0;
        let n = 26;
        let a = corridor(0, 6.0, len, n);
        let deg = len / (crate::scene::DEG_M * cos_lat());
        let b = corridor(1, 6.0 + deg, len, n);
        let point = *a.nodes.last().unwrap();
        let scene = {
            let mut s = SceneGraph::new(vec![a, b]);
            s.junctions = vec![Junction {
                point,
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: len },
                    JunctionMember { corridor: 1, arc: 0.0 },
                ],
            }];
            s
        };
        let an = scene.corridors[0].nodes.clone();
        let bn = scene.corridors[1].nodes.clone();
        let solved = SolvedModel::from_profiles(
            vec![
                Some(super::super::Profile::flat(&an, 400.0)),
                Some(super::super::Profile::flat(&bn, 400.0)),
            ],
            14,
        );
        let c = measure(&scene, &solved);
        assert_eq!(c.max_junction_step_m, 0.0);
        assert_eq!(c.junction_steps_over, 0);
    }
}
