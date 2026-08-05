//! Structures as **consequences** of the solve (docs/GENERATION.md §4.5).
//!
//! > *Solve heights subject to constraints, then synthesize the structure the
//! > result implies.*
//!
//! A deck exists where the solved surface departs the ground beyond a
//! threshold. A bore exists where it runs below. Portals are where it crosses.
//! The data's level, bridge and tunnel annotations are **priors on the
//! constraint** — a hint that a clearance exists here, and an ordering for it —
//! never commands to build geometry.
//!
//! The inversion removes a class of contradiction rather than fixing instances
//! of it. When structures are inputs, a stage that later decides a declared
//! bridge is not real leaves the clearance demand it justified still standing,
//! and that orphaned demand asks at-grade surface to climb into the air for a
//! deck that no longer exists. When structures are outputs there is no
//! "crossing whose bridge was deleted", because bridges were never inputs.
//!
//! ## What is derived here, and what is not
//!
//! This module derives the runs. It does **not** yet drive the tiler, the
//! ground carves or the junction sheets — those still cut against the annotated
//! spans. Landing the derivation first buys the one thing worth having before
//! switching five consumers at once: a measurement of how far apart the two
//! worlds actually are, on the real extract, before anything depends on the
//! answer.
//!
//! That measurement (`verify::model::structures`) says they are far apart, and
//! why: **13,754 derived structures against 495 annotated ones**. The threshold
//! is the reason — see [`DECK_STANDOFF_M`] — and calibrating it is the work
//! that has to happen before the consumers move. Switching them first would
//! have put thirteen thousand phantom bridges into the scene.

use crate::priors::{Prior, DECK_THICKNESS_M, MIN_STRUCTURE_M, SHORT_STRUCTURE_DIP_M};
use crate::scene::SpanKind;

use super::profile::Profile;

/// A structure the solved heights imply: a maximal run of the corridor where
/// the road is not on the ground.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StructureRun {
    pub arc0: f64,
    pub arc1: f64,
    pub kind: SpanKind,
}

/// How far the road must stand clear of the ground before a *deck* is the
/// honest answer rather than an embankment.
///
/// **This value is wrong, and the check knows by how much.** It was reasoned
/// from the geometry — a deck's soffit sits a [`DECK_THICKNESS_M`] below its
/// running surface and a mouth needs [`PORTAL_CLEARANCE_M`] to read as open, so
/// anything shallower cannot be drawn as a structure without burying the solid
/// in its own fill — and that argument gives a *lower bound*, not a threshold.
///
/// The upper bound is the one that matters and it is an engineering prior
/// nobody has measured here: how much fill a road plausibly stands on. A street
/// may leave its conditioned terrain by `deviation_m`, which is 2.5 m — exactly
/// this number — so every street sitting at its budget reads as a bridge. On the
/// Montreux extract that gives **13,754 derived structures against 495
/// annotated ones**, median 59 m of "deck" that is really embankment
/// (`structure.derived_new`).
///
/// Calibrating it is the next step, and the discipline for it is the one that
/// caught this: histogram the gap over the whole network first and look for the
/// second mode, rather than reasoning a number out of the deck's own thickness.
pub const DECK_STANDOFF_M: f64 = DECK_THICKNESS_M + crate::priors::PORTAL_CLEARANCE_M;

/// Derives the structures a solved profile implies.
///
/// The gap `road − terrain` is the whole signal. Positive beyond
/// [`DECK_STANDOFF_M`] is a deck; negative at all is a bore, because a road
/// below the ground is under it by definition and the zero crossing *is* the
/// portal (S5 — the mouth sits where the road actually emerges, not where a
/// mapper split the way).
///
/// `hints` are not consulted. They are priors on the *constraint*, and the
/// constraint has already been solved; consulting them again here would be the
/// annotation deciding geometry through the back door.
pub fn derive(p: &Profile, prior: &Prior) -> Vec<StructureRun> {
    let (arc, road, terrain) = (p.arc(), p.road_m(), p.terrain_m());
    if arc.len() < 2 {
        return Vec::new();
    }
    let kind_at = |i: usize| {
        let gap = road[i] - terrain[i];
        if gap > DECK_STANDOFF_M {
            Some(SpanKind::Bridge)
        } else if gap < 0.0 {
            Some(SpanKind::Tunnel)
        } else {
            None
        }
    };

    // Maximal same-kind runs, with the ends interpolated to the crossing of the
    // threshold rather than snapped to a node: a portal placed on the nearest
    // sample is up to a node spacing wrong, and a node spacing is 8 m.
    let mut runs: Vec<StructureRun> = Vec::new();
    let mut i = 0;
    while i < arc.len() {
        let Some(kind) = kind_at(i) else {
            i += 1;
            continue;
        };
        let start = i;
        while i + 1 < arc.len() && kind_at(i + 1) == Some(kind) {
            i += 1;
        }
        let (a, b) = (
            edge_arc(arc, road, terrain, start, kind, false),
            edge_arc(arc, road, terrain, i, kind, true),
        );
        if b > a {
            runs.push(StructureRun { arc0: a, arc1: b, kind });
        }
        i += 1;
    }

    // Close sub-`SNAP_RUN_M` gaps between same-kind runs: an annotation edge
    // mismatch, or one node of a long viaduct dipping to the threshold, is not
    // two structures (S10).
    coalesce(&mut runs);
    // Drop what is too short to be a real structure of this class, unless the
    // ground genuinely falls away beneath it. This is
    // `solve::reconcile_short_spans`' test, applied where it belongs: after the
    // solve, to a run the solve produced, rather than before it to an
    // annotation.
    runs.retain(|r| plausible(p, prior, r));
    runs
}

/// Where a run's edge crosses its threshold, interpolated between the last node
/// inside it and the first node outside.
fn edge_arc(
    arc: &[f64],
    road: &[f64],
    terrain: &[f64],
    i: usize,
    kind: SpanKind,
    forward: bool,
) -> f64 {
    let level = if kind == SpanKind::Bridge { DECK_STANDOFF_M } else { 0.0 };
    let j = if forward {
        if i + 1 >= arc.len() {
            return arc[i];
        }
        i + 1
    } else {
        if i == 0 {
            return arc[i];
        }
        i - 1
    };
    let (gi, gj) = (road[i] - terrain[i] - level, road[j] - terrain[j] - level);
    if (gi - gj).abs() < f64::EPSILON {
        return arc[i];
    }
    let t = (gi / (gi - gj)).clamp(0.0, 1.0);
    arc[i] + (arc[j] - arc[i]) * t
}

/// Merges same-kind runs separated by less than [`crate::priors::SNAP_RUN_M`].
fn coalesce(runs: &mut Vec<StructureRun>) {
    let mut i = 1;
    while i < runs.len() {
        let joins = runs[i].kind == runs[i - 1].kind
            && runs[i].arc0 - runs[i - 1].arc1 < crate::priors::SNAP_RUN_M;
        if joins {
            runs[i - 1].arc1 = runs[i].arc1;
            runs.remove(i);
        } else {
            i += 1;
        }
    }
}

/// Whether a run is long enough to be a real structure, or short but over
/// ground that genuinely falls away.
fn plausible(p: &Profile, prior: &Prior, r: &StructureRun) -> bool {
    if r.arc1 - r.arc0 >= prior.min_structure_m.max(MIN_STRUCTURE_M).min(MIN_STRUCTURE_M) {
        return true;
    }
    // The deep-gully case: a short span over a real ravine is a real bridge,
    // and demoting it blindly once dived a road through the gorge it crossed.
    (1..=3).any(|k| {
        let t = k as f64 / 4.0;
        let a = r.arc0 + (r.arc1 - r.arc0) * t;
        let depart = match r.kind {
            SpanKind::Bridge => p.road_at_arc(a) - p.surface_at_arc(a),
            SpanKind::Tunnel => p.surface_at_arc(a) - p.road_at_arc(a),
            SpanKind::Grade => 0.0,
        };
        depart > SHORT_STRUCTURE_DIP_M
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::{Kind, RoadClass};
    use crate::scene::DEG_M;
    use geo_types::Coord;

    fn prior() -> &'static Prior {
        Kind::Road(RoadClass::Secondary).prior()
    }

    /// `n` nodes over `len_m` metres, with the given road and terrain heights.
    fn profile(n: usize, len_m: f64, road: Vec<f64>, terrain: Vec<f64>) -> Profile {
        let deg = len_m / DEG_M;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0, y: 46.0 + deg * i as f64 / (n - 1) as f64 }).collect();
        Profile::from_heights(&nodes, road, terrain)
    }

    /// A viaduct: the ground falls into a ravine and the road flies straight.
    #[test]
    fn a_road_standing_clear_of_the_ground_is_a_deck() {
        let n = 21;
        let road = vec![100.0; n];
        let terrain: Vec<f64> = (0..n)
            .map(|i| if (5..=15).contains(&i) { 60.0 } else { 100.0 })
            .collect();
        let runs = derive(&profile(n, 2000.0, road, terrain), prior());
        assert_eq!(runs.len(), 1, "one deck, got {runs:?}");
        assert_eq!(runs[0].kind, SpanKind::Bridge);
        // The edges land where the gap crosses the standoff, between nodes —
        // not snapped to the node, which would be up to a spacing wrong.
        assert!(runs[0].arc0 > 400.0 && runs[0].arc0 < 500.0, "arc0 {}", runs[0].arc0);
    }

    /// A road under the hill is in a bore, and the portal is the zero crossing.
    #[test]
    fn a_road_below_the_ground_is_a_bore() {
        let n = 21;
        let road = vec![100.0; n];
        // On the ground at both ends, under a hill in the middle — so the
        // only structure is the bore, not an approach deck as well.
        let terrain: Vec<f64> = (0..n)
            .map(|i| if (8..=12).contains(&i) { 140.0 } else { 100.0 })
            .collect();
        let runs = derive(&profile(n, 2000.0, road, terrain), prior());
        assert_eq!(runs.len(), 1, "one bore, got {runs:?}");
        assert_eq!(runs[0].kind, SpanKind::Tunnel);
        // The portal is the zero crossing, not the node the hill starts at:
        // the road meets the ground exactly at node 7 (arc 700), so that is
        // where the mouth is, and node 8 — where the terrain first exceeds it
        // — is already inside the hill.
        assert!((runs[0].arc0 - 700.0).abs() < 1e-6, "arc0 {}", runs[0].arc0);
    }

    /// An embankment is not a bridge. The road stands clear of the ground, but
    /// by less than a deck's own thickness plus its mouth — there is no solid
    /// that could be drawn there without burying it in its own fill.
    #[test]
    fn a_low_embankment_is_not_a_structure() {
        let n = 21;
        let road = vec![100.0; n];
        let terrain = vec![98.8; n]; // 1.2 m of fill, under the standoff
        assert!(derive(&profile(n, 2000.0, road, terrain), prior()).is_empty());
    }

    /// Nothing is a structure where the road lies on the ground.
    #[test]
    fn a_road_on_the_ground_implies_nothing() {
        let n = 11;
        let road = vec![100.0; n];
        assert!(derive(&profile(n, 1000.0, road.clone(), road), prior()).is_empty());
    }

    /// A one-node dip in a long viaduct does not split it into two bridges
    /// (S10: annotation noise, and its geometric equivalent).
    #[test]
    fn a_momentary_touch_does_not_split_a_run() {
        let n = 41;
        let road = vec![100.0; n];
        let mut terrain = vec![60.0; n];
        terrain[20] = 99.0; // one node brushing the standoff
        let runs = derive(&profile(n, 4000.0, road, terrain), prior());
        assert_eq!(runs.len(), 1, "one viaduct, got {runs:?}");
    }
}
