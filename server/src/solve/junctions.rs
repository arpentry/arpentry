//! Junction continuity (docs/GENERATION.md invariant 2).
//!
//! Corridors are solved one at a time, each anchored only to the terrain, so
//! where two of them meet at a shared connector nothing makes their road
//! surfaces agree. At grade this is harmless — both anchor to the same ground,
//! so they already line up — but where one arrives elevated (a ramp diverging
//! from a flyover, the two halves of a viaduct split at the corridor-length
//! cap) the independent solves leave a step at the joint.
//!
//! This pass welds them, in two regimes split by whether a member arrives
//! *elevated* (standing off its own ground at the junction,
//! [`ELEVATED_MIN_M`]):
//!
//! - **Structural** (elevated): the through road (the member the connector
//!   sits *interior* to) sets the height, and every *leg* — a member that
//!   ends at the junction — that solved below it is lifted to meet it with a
//!   [`Profile::raise_crest`] ramp, decaying back to its own grade inland.
//!   The weld is raise-only, so stacked constraints compose and it never
//!   pushes a road down through a clearance it already earned; a demand
//!   beyond what the leg can climb at its ramp grade over its own length (or
//!   beyond the [`MAX_JUNCTION_WELD_M`] ceiling) is dropped as a data
//!   contradiction (the connector links roads that do not meet at one
//!   height).
//! - **Grounded** (everything else — the street majority): the members sit
//!   on the same ground and disagree only by sampling and conditioning
//!   margins; a small symmetric weld pulls the meeting ends to one height
//!   ([`weld_streets`], docs/GROUND.md §1).
//!
//! It reconciles genuine disagreements at a shared connector; it does not by
//! itself undo a cap-split that dips a through-structure (there both halves dip
//! alike and already agree — that needs the through height neither side holds).

use crate::priors::{BED_WELD_MAX_M, MAX_JUNCTION_WELD_M, RAMP_GRADE};
use crate::scene::{Junction, SceneGraph};

use super::profile::Profile;

/// Smallest height disagreement worth welding, metres — below it the members
/// already line up (the at-grade case) and moving them buys nothing.
const WELD_TOL_M: f64 = 0.5;

/// How far a member must stand above its own ground at the junction to count
/// as arriving *elevated* — the structural weld's territory (a ramp meeting
/// a flyover). Below it every member sits on the ground: their heights can
/// only disagree by sampling and conditioning differences, which the
/// symmetric street weld reconciles instead ([`weld_streets`]). A raise-only
/// weld applied there would drag every junction up to its highest sample.
const ELEVATED_MIN_M: f64 = 1.0;

/// How near a corridor end (arc metres) a member sits to count as a *leg* that
/// terminates at the junction, rather than a through road passing across it.
const END_EPS_M: f64 = 1.5;

/// Welds every junction's legs to the road they meet, in place. Two passes:
/// the structural raise-only weld first (a ramp climbing to a flyover), then
/// the street weld (docs/GROUND.md §1) — meeting street ends pulled to one
/// height, symmetric but small. Deterministic: the junctions are in
/// connector order, each structural weld is a pure function of the solved
/// profiles, and the street deltas are computed against post-structural
/// heights and applied together afterwards, so every tile fragment reads
/// the same welded heights (invariant 5).
pub fn apply(scene: &SceneGraph, profiles: &mut [Option<Profile>]) {
    for j in &scene.junctions {
        weld(scene, j, profiles);
    }
    weld_streets(scene, profiles);
}

/// The grounded mirror of the structural weld: at a junction where every
/// member sits on its own ground, the independently solved profiles can
/// still disagree by a sampling or conditioning margin — a visible step
/// across the intersection. The ends are pulled to one height: an engineered
/// member's road height where one is present (the network the streets land
/// on), else a through member's height, else the mean of the meeting ends.
/// Symmetric but small — a member whose required shift exceeds
/// [`BED_WELD_MAX_M`] keeps its own height (the disagreement is a data
/// contradiction, not a weldable seam). Corrections decay into each member
/// at its own plausible grade ([`Profile::weld_end`]). A junction with an
/// elevated member is the structural weld's and is left alone.
fn weld_streets(scene: &SceneGraph, profiles: &mut [Option<Profile>]) {
    // (corridor, welds-at-start, delta, decay grade), collected against
    // pre-weld heights and applied after, so the outcome is independent of
    // junction iteration order.
    let mut shifts: Vec<(usize, bool, f64, f64)> = Vec::new();
    for j in &scene.junctions {
        struct Member {
            corridor: usize,
            h: f64,
            is_leg: bool,
            at_start: bool,
            engineered: bool,
            grade: f64,
            elevated: bool,
        }
        let mut members: Vec<Member> = Vec::new();
        for m in &j.members {
            let Some(p) = profiles.get(m.corridor as usize).and_then(|p| p.as_ref()) else {
                continue;
            };
            let c = &scene.corridors[m.corridor as usize];
            let at_start = m.arc <= END_EPS_M;
            let h = p.road_at_arc(m.arc);
            members.push(Member {
                corridor: m.corridor as usize,
                h,
                is_leg: at_start || m.arc >= c.total() - END_EPS_M,
                at_start,
                engineered: c.class.grade_limit().is_some(),
                grade: c.class.grade_limit().unwrap_or_else(|| c.class.bed_grade()),
                elevated: h - p.surface_at(j.point.x, j.point.y) > ELEVATED_MIN_M,
            });
        }
        if members.len() < 2 || members.iter().any(|m| m.elevated) {
            continue; // nothing to reconcile, or the structural weld's case
        }
        let target = if let Some(e) = members.iter().find(|m| m.engineered) {
            e.h
        } else if let Some(t) = members.iter().find(|m| !m.is_leg) {
            t.h
        } else {
            members.iter().map(|m| m.h).sum::<f64>() / members.len() as f64
        };
        for m in &members {
            if !m.is_leg {
                continue; // a through member holds its height
            }
            let delta = target - m.h;
            if delta != 0.0 && delta.abs() <= BED_WELD_MAX_M {
                shifts.push((m.corridor, m.at_start, delta, m.grade));
            }
        }
    }
    for (ci, at_start, delta, grade) in shifts {
        if let Some(p) = profiles[ci].as_mut() {
            p.weld_end(at_start, delta, grade);
            p.rebuild_deck();
        }
    }
}

fn weld(scene: &SceneGraph, j: &Junction, profiles: &mut [Option<Profile>]) {
    // Each profiled member's current road height at the junction, whether it
    // terminates there (a weldable leg) or passes through (sets the height),
    // and whether it arrives *elevated* — standing off its own ground.
    let mut members: Vec<(usize, f64, bool, bool)> = Vec::new();
    for (mi, m) in j.members.iter().enumerate() {
        let Some(p) = profiles.get(m.corridor as usize).and_then(|p| p.as_ref()) else {
            continue; // a draped member has no profile: it sits on the ground
        };
        let total = scene.corridors[m.corridor as usize].total();
        let is_leg = m.arc <= END_EPS_M || m.arc >= total - END_EPS_M;
        let h = p.road_at_arc(m.arc);
        let elevated = h - p.surface_at(j.point.x, j.point.y) > ELEVATED_MIN_M;
        members.push((mi, h, is_leg, elevated));
    }
    if members.len() < 2 {
        return;
    }
    // Only an *elevated* member may set the raise-only target
    // ([`ELEVATED_MIN_M`]): the through road where one passes, else the
    // highest elevated leg (a fork). A junction where every member sits on
    // its own ground has nothing structural to weld — the street weld's
    // symmetric reconciliation covers it.
    let through = members
        .iter()
        .filter(|&&(_, _, leg, elevated)| !leg && elevated)
        .map(|&(_, h, _, _)| h)
        .fold(f64::NEG_INFINITY, f64::max);
    let target = if through.is_finite() {
        through
    } else {
        members
            .iter()
            .filter(|&&(_, _, _, elevated)| elevated)
            .map(|&(_, h, _, _)| h)
            .fold(f64::NEG_INFINITY, f64::max)
    };
    if !target.is_finite() {
        return; // a grounded junction: weld_streets reconciles it
    }

    for &(mi, h, is_leg, _) in &members {
        if !is_leg {
            continue; // a through road holds its own profile
        }
        let deficit = target - h;
        if deficit <= WELD_TOL_M {
            continue; // already aligned
        }
        let m = &j.members[mi];
        let c = &scene.corridors[m.corridor as usize];
        // A link (ramp) is engineered to the steeper approach grade whatever
        // its class; a mainline leg keeps its own ceiling.
        let grade = if c.link { RAMP_GRADE } else { c.class.grade_limit().unwrap_or(RAMP_GRADE) };
        // Plausibility: the leg must be able to climb the deficit within its
        // own run at that grade (a 300 m ramp meets a 16 m-high viaduct; a
        // 50 m stub cannot), under the absolute interchange-height ceiling.
        if deficit > (grade * c.total()).min(MAX_JUNCTION_WELD_M) {
            continue; // an implausible demand: trust the profile
        }
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
    fn classed(id: u32, x0: f64, len_m: f64, n: usize, class: RoadClass) -> Corridor {
        let deg = len_m / (crate::scene::DEG_M * cos_lat());
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: x0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        Corridor {
            id,
            nodes,
            arc,
            cos_lat: cos_lat(),
            class,
            class_key: String::new(),
            link: false,
            drivable: true,
            spans: vec![],
            segments: vec![SegmentRef { source: id as u64, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }
    }

    fn corridor(id: u32, x0: f64, len_m: f64, n: usize) -> Corridor {
        classed(id, x0, len_m, n, RoadClass::Motorway)
    }

    /// A profile riding `road_m` flat over ground `terrain_m` below it — an
    /// *elevated* arrival (a flyover), for the structural-weld tests.
    fn elevated(nodes: &[Coord], road_m: f64, terrain_m: f64) -> Profile {
        Profile::from_heights(
            nodes,
            vec![road_m; nodes.len()],
            vec![terrain_m; nodes.len()],
        )
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
        // The flyover stands 7 m off its ground — an elevated arrival; the
        // ramp sits on hers.
        let mut profiles = vec![
            Some(elevated(&main_nodes, 380.0, 373.0)),
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
    fn a_long_ramp_welds_up_to_a_tall_viaduct() {
        // The Montreux defect: a 300 m connector solved 16 m under the viaduct
        // it merges onto — beyond the old flat 12 m cap, but well within what
        // a ramp climbs at RAMP_GRADE over 300 m. The weld must fire.
        let len = 300.0;
        let n = 101;
        let main = corridor(0, 6.0, 800.0, n);
        let mut ramp = corridor(1, 6.02, len, n);
        ramp.link = true;
        let point = main.nodes[n / 2];
        let scene = {
            let mut s = SceneGraph::new(vec![main, ramp]);
            s.junctions = vec![Junction {
                point,
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: 400.0 },
                    JunctionMember { corridor: 1, arc: 0.0 },
                ],
            }];
            s
        };
        let m = scene.corridors[0].nodes.clone();
        let r = scene.corridors[1].nodes.clone();
        let mut profiles =
            vec![Some(elevated(&m, 493.0, 477.0)), Some(Profile::flat(&r, 477.0))];
        apply(&scene, &mut profiles);
        assert!(
            (profiles[1].as_ref().unwrap().road_at_arc(0.0) - 493.0).abs() < 0.5,
            "a 16 m demand within the ramp's climbing capacity must weld, got {}",
            profiles[1].as_ref().unwrap().road_at_arc(0.0)
        );
    }

    #[test]
    fn a_stub_that_cannot_climb_the_deficit_is_dropped() {
        // A 60 m link stub 16 m under the through road: even at RAMP_GRADE it
        // could only climb ~5 m over its whole run — the connector links roads
        // that do not meet at one height, so the weld is dropped.
        let n = 31;
        let main = corridor(0, 6.0, 800.0, n);
        let mut stub = corridor(1, 6.02, 60.0, n);
        stub.link = true;
        let point = main.nodes[n / 2];
        let scene = {
            let mut s = SceneGraph::new(vec![main, stub]);
            s.junctions = vec![Junction {
                point,
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: 400.0 },
                    JunctionMember { corridor: 1, arc: 0.0 },
                ],
            }];
            s
        };
        let m = scene.corridors[0].nodes.clone();
        let r = scene.corridors[1].nodes.clone();
        let mut profiles =
            vec![Some(elevated(&m, 493.0, 477.0)), Some(Profile::flat(&r, 477.0))];
        apply(&scene, &mut profiles);
        assert!(
            (profiles[1].as_ref().unwrap().road_at_arc(0.0) - 477.0).abs() < 1e-6,
            "a demand beyond the stub's climbing capacity must be dropped"
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
        let mut profiles =
            vec![Some(elevated(&m, 400.0, 360.0)), Some(Profile::flat(&r, 360.0))];
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

    /// Two street profiles sharing an endpoint connector on disagreeing
    /// terrain weld to the meeting mean: no step crosses the intersection,
    /// and the far ends keep their own ground (the correction decays).
    #[test]
    fn streets_meeting_at_a_node_agree() {
        let len = 200.0;
        let n = 26;
        // West street ends where the east street starts (same x0 + len).
        let west = classed(0, 6.0, len, n, RoadClass::Minor);
        let deg = len / (crate::scene::DEG_M * cos_lat());
        let east = classed(1, 6.0 + deg, len, n, RoadClass::Minor);
        let point = *west.nodes.last().unwrap();
        let scene = {
            let mut s = SceneGraph::new(vec![west, east]);
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
        let w = scene.corridors[0].nodes.clone();
        let e = scene.corridors[1].nodes.clone();
        // The DEM disagrees with itself by 2 m across the connector.
        let mut profiles = vec![Some(Profile::flat(&w, 400.0)), Some(Profile::flat(&e, 402.0))];
        apply(&scene, &mut profiles);
        let w_end = profiles[0].as_ref().unwrap().road_at_arc(len);
        let e_start = profiles[1].as_ref().unwrap().road_at_arc(0.0);
        assert!((w_end - e_start).abs() < 1e-9, "welded endpoints must agree");
        assert!((w_end - 401.0).abs() < 1e-9, "the weld is the meeting mean, got {w_end}");
        // The far ends keep their own ground.
        assert!((profiles[0].as_ref().unwrap().road_at_arc(0.0) - 400.0).abs() < 1e-9);
        assert!((profiles[1].as_ref().unwrap().road_at_arc(len) - 402.0).abs() < 1e-9);
    }

    /// A street ending on an engineered road welds to the road's height
    /// (within the trust cap), so the street meets the highway it joins; a
    /// disagreement beyond the cap is a data contradiction and left alone.
    #[test]
    fn a_street_welds_to_its_engineered_road() {
        let len = 200.0;
        let n = 26;
        let scene = {
            let mut s = SceneGraph::new(vec![
                corridor(0, 6.0, 800.0, 101),
                classed(1, 6.02, len, n, RoadClass::Minor),
            ]);
            s.junctions = vec![Junction {
                point: s.corridors[0].nodes[50],
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: 400.0 },
                    JunctionMember { corridor: 1, arc: 0.0 },
                ],
            }];
            s
        };
        let m = scene.corridors[0].nodes.clone();
        let r = scene.corridors[1].nodes.clone();
        // The engineered road arrives 1.5 m above the street's ground — a
        // grounded disagreement (both on their own terrain), street-weldable.
        let mut profiles = vec![Some(Profile::flat(&m, 401.5)), Some(Profile::flat(&r, 400.0))];
        apply(&scene, &mut profiles);
        assert!(
            (profiles[1].as_ref().unwrap().road_at_arc(0.0) - 401.5).abs() < 1e-9,
            "the engineered height wins, got {}",
            profiles[1].as_ref().unwrap().road_at_arc(0.0)
        );
        // The engineered road itself is never moved by the street weld.
        assert!((profiles[0].as_ref().unwrap().road_at_arc(400.0) - 401.5).abs() < 1e-9);
        // Beyond the trust cap the street keeps its own ground.
        let mut profiles = vec![Some(Profile::flat(&m, 410.0)), Some(Profile::flat(&r, 400.0))];
        apply(&scene, &mut profiles);
        assert!(
            (profiles[1].as_ref().unwrap().road_at_arc(0.0) - 400.0).abs() < 1e-9,
            "a 10 m step is not weldable"
        );
    }
}
