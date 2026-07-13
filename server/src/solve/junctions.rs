//! Junction continuity (docs/GENERATION.md invariant 2).
//!
//! Corridors are solved one at a time, each anchored only to the terrain, so
//! where two of them meet at a shared connector nothing makes their road
//! surfaces agree. At grade this is harmless — both anchor to the same ground,
//! so they already line up — but where one arrives elevated (a ramp diverging
//! from a flyover, the two halves of a viaduct split at the corridor-length
//! cap) the independent solves leave a step at the joint.
//!
//! This pass welds them: at each junction the through road (the member the
//! connector sits *interior* to) sets the height, and every *leg* — a member
//! that ends at the junction — that solved below it is lifted to meet it with a
//! [`Profile::raise_crest`] ramp, decaying back to its own grade inland. The
//! weld is raise-only, so stacked constraints compose and it never pushes a
//! road down through a clearance it already earned; a demand beyond
//! [`MAX_JUNCTION_WELD_M`] is dropped as a data contradiction (the connector
//! links roads that do not in fact meet at one height).
//!
//! It reconciles genuine disagreements at a shared connector; it does not by
//! itself undo a cap-split that dips a through-structure (there both halves dip
//! alike and already agree — that needs the through height neither side holds).

use crate::priors::{MAX_JUNCTION_WELD_M, RAMP_GRADE};
use crate::scene::{Junction, SceneGraph};

use super::profile::Profile;

/// Smallest height disagreement worth welding, metres — below it the members
/// already line up (the at-grade case) and moving them buys nothing.
const WELD_TOL_M: f64 = 0.5;

/// How near a corridor end (arc metres) a member sits to count as a *leg* that
/// terminates at the junction, rather than a through road passing across it.
const END_EPS_M: f64 = 1.5;

/// Welds every junction's legs to the road they meet, in place. Deterministic:
/// the junctions are in connector order and each weld is a pure function of the
/// solved profiles, so every tile fragment reads the same welded heights
/// (invariant 5).
pub fn apply(scene: &SceneGraph, profiles: &mut [Option<Profile>]) {
    for j in &scene.junctions {
        weld(scene, j, profiles);
    }
}

fn weld(scene: &SceneGraph, j: &Junction, profiles: &mut [Option<Profile>]) {
    // Each profiled member's current road height at the junction, and whether
    // it terminates there (a weldable leg) or passes through (sets the height).
    let mut members: Vec<(usize, f64, bool)> = Vec::new();
    for (mi, m) in j.members.iter().enumerate() {
        let Some(p) = profiles.get(m.corridor as usize).and_then(|p| p.as_ref()) else {
            continue; // a draped member has no profile: it sits on the ground
        };
        let total = scene.corridors[m.corridor as usize].total();
        let is_leg = m.arc <= END_EPS_M || m.arc >= total - END_EPS_M;
        members.push((mi, p.road_at_arc(m.arc), is_leg));
    }
    if members.len() < 2 {
        return;
    }
    // The through road sets the target; if every member terminates (a fork),
    // the highest leg does — a raise-only weld pulls the rest up to it.
    let through = members
        .iter()
        .filter(|(_, _, leg)| !leg)
        .map(|&(_, h, _)| h)
        .fold(f64::NEG_INFINITY, f64::max);
    let target =
        if through.is_finite() { through } else { members.iter().map(|&(_, h, _)| h).fold(f64::NEG_INFINITY, f64::max) };

    for &(mi, h, is_leg) in &members {
        if !is_leg {
            continue; // a through road holds its own profile
        }
        let deficit = target - h;
        if deficit <= WELD_TOL_M || deficit > MAX_JUNCTION_WELD_M {
            continue; // already aligned, or an implausible demand: trust the profile
        }
        let m = &j.members[mi];
        let grade =
            scene.corridors[m.corridor as usize].class.grade_limit().unwrap_or(RAMP_GRADE);
        let Some(p) = profiles[m.corridor as usize].as_mut() else { continue };
        // A point crest at the leg's end: lift it to the target and ramp back
        // to its own grade inland (lo == hi == the end arc).
        p.raise_crest(m.arc, m.arc, m.arc, target, grade);
        p.rebuild_deck();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::RoadClass;
    use crate::scene::{Corridor, JunctionMember, SegmentRef};
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
            link: false,
            spans: vec![],
            segments: vec![SegmentRef { source: id as u64, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }
    }

    #[test]
    fn a_ramp_leg_welds_up_to_the_elevated_through_road() {
        // Corridor 0 is a mainline held elevated at 380 m across the junction
        // (a flyover). Corridor 1 is a ramp that solved 7 m lower (373 m) and
        // ends at the junction. After the weld the ramp's end meets the flyover;
        // its far end keeps its own height.
        let len = 800.0;
        let n = 101;
        let main = corridor(0, 6.0, len, n);
        let ramp = corridor(1, 6.02, len, n);
        // The junction sits at the mainline's midpoint (a through member) and
        // the ramp's start (a leg).
        let mid_arc = len / 2.0;
        let point = main.nodes[n / 2];
        let scene = {
            let mut s = SceneGraph::new(vec![main, ramp]);
            s.junctions = vec![Junction {
                point,
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: mid_arc },
                    JunctionMember { corridor: 1, arc: 0.0 },
                ],
            }];
            s
        };
        let main_nodes = scene.corridors[0].nodes.clone();
        let ramp_nodes = scene.corridors[1].nodes.clone();
        let mut profiles = vec![
            Some(Profile::flat(&main_nodes, 380.0)),
            Some(Profile::flat(&ramp_nodes, 373.0)),
        ];
        apply(&scene, &mut profiles);
        let ramp = profiles[1].as_ref().unwrap();
        assert!(
            (ramp.road_at_arc(0.0) - 380.0).abs() < 0.5,
            "the ramp end must weld up to the flyover, got {}",
            ramp.road_at_arc(0.0)
        );
        // The through road is untouched.
        assert!((profiles[0].as_ref().unwrap().road_at_arc(mid_arc) - 380.0).abs() < 1e-6);
        // The ramp's far end keeps its own grade (the weld decays inland).
        assert!(
            (ramp.road_at_arc(len) - 373.0).abs() < 0.5,
            "the far end must stay at grade, got {}",
            ramp.road_at_arc(len)
        );
    }

    #[test]
    fn an_absurd_weld_demand_is_dropped() {
        // A ramp that solved 40 m below the through road: far past any real
        // ramp: the connector links roads that don't meet at one height. The
        // weld is dropped and the ramp keeps its profile.
        let len = 800.0;
        let n = 101;
        let scene = {
            let mut s = SceneGraph::new(vec![corridor(0, 6.0, len, n), corridor(1, 6.02, len, n)]);
            s.junctions = vec![Junction {
                point: s.corridors[0].nodes[n / 2],
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: len / 2.0 },
                    JunctionMember { corridor: 1, arc: 0.0 },
                ],
            }];
            s
        };
        let m = scene.corridors[0].nodes.clone();
        let r = scene.corridors[1].nodes.clone();
        let mut profiles = vec![Some(Profile::flat(&m, 400.0)), Some(Profile::flat(&r, 360.0))];
        apply(&scene, &mut profiles);
        assert!(
            (profiles[1].as_ref().unwrap().road_at_arc(0.0) - 360.0).abs() < 1e-6,
            "the contradictory weld must be dropped"
        );
    }

    #[test]
    fn an_at_grade_junction_is_left_alone() {
        // Both members already at the same height: the weld is a no-op (the
        // implicit at-grade continuity this pass must not disturb).
        let len = 800.0;
        let n = 101;
        let scene = {
            let mut s = SceneGraph::new(vec![corridor(0, 6.0, len, n), corridor(1, 6.02, len, n)]);
            s.junctions = vec![Junction {
                point: s.corridors[0].nodes[n / 2],
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: len / 2.0 },
                    JunctionMember { corridor: 1, arc: 0.0 },
                ],
            }];
            s
        };
        let m = scene.corridors[0].nodes.clone();
        let r = scene.corridors[1].nodes.clone();
        let mut profiles = vec![Some(Profile::flat(&m, 372.0)), Some(Profile::flat(&r, 372.0))];
        apply(&scene, &mut profiles);
        assert!((profiles[1].as_ref().unwrap().road_at_arc(0.0) - 372.0).abs() < 1e-6);
    }
}
