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

use crate::priors::{
    Prior, MIN_STRUCTURE_M, SHORT_STRUCTURE_DIP_M, TUNNEL_COVER_M, TUNNEL_HEIGHT_M,
};
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

/// How far the road must stand **clear of** the ground before a deck is the
/// honest answer rather than an embankment.
///
/// Calibrated, not reasoned. The first version of this took the deck's own
/// geometry — a soffit a thickness below the surface, a mouth needing clearance
/// to read as open — and got 2.5 m, which is a sound *lower* bound and a
/// useless threshold: a street may leave its terrain by `deviation_m`, which is
/// also 2.5 m, so every street at its budget read as a bridge.
///
/// `examples/gap_histogram` measures the population instead. Over 411,477
/// at-grade nodes on the Montreux extract the gap is tight — p50 0.0 m, p95
/// 0.9 m, p99 2.5 m — and the tail beyond it is thin and smooth, with no knee
/// to snap to. What the choice buys, per the same run:
///
/// | standoff | at-grade called a deck |
/// |---------:|----------------------:|
/// |    2.5 m |                 1.01 % |
/// |    4.0 m |                 0.51 % |
/// |    6.0 m |                 0.30 % |
/// |    8.0 m |                 0.18 % |
///
/// 4 m is the smallest value clear of the at-grade population's own spread
/// (p99 + 1.5 m), which is what the threshold has to mean: past here the road
/// is not on fill any more.
pub const DECK_STANDOFF_M: f64 = 4.0;

/// How far the road must run **below** the ground before a bore is the honest
/// answer rather than a cutting.
///
/// The first version used zero — a road below the ground is under it, which is
/// true and not the question. 35 % of at-grade nodes sit between −2 m and 0
/// against the *raw* terrain, because a benched road is cut into the hillside
/// and `terrain_m` is the DEM before the bench. Every one of them read as a
/// tunnel, and that — not the deck threshold — was where most of the 13,754
/// phantom structures came from.
///
/// A bore needs room for the tube it is: the road, a [`TUNNEL_HEIGHT_M`] of
/// bore above it, and [`TUNNEL_COVER_M`] of ground over that. Shallower than
/// their sum there is nothing to drive through, and a cutting is what is there.
/// Almost nothing at grade reaches it — the at-grade p05 is −0.5 m.
pub const BORE_COVER_M: f64 = TUNNEL_HEIGHT_M + TUNNEL_COVER_M;

/// Tolerance on the bore test at exactly [`BORE_COVER_M`], because the burial
/// license *clamps* a bore to its ceiling: `terrain − roof − cover`, computed
/// from the same terrain samples, so the gap at a clamped node equals the
/// threshold to the float. A strict comparison makes "at the guarantee"
/// unsatisfiable, and every ceiling-clamped run is censored by its own
/// license — measured on the Montreux extract as 21 annotated tunnels
/// (1.3 km) whose max departure was +5.500 m to the millimetre, with nothing
/// else between −21 mm and +36 mm of the threshold. One millimetre admits the
/// clamped contact and no genuine cutting.
pub const CEILING_CONTACT_M: f64 = 1e-3;

/// Derives the structures a solved profile implies.
///
/// The gap `road − terrain` is the whole signal: past [`DECK_STANDOFF_M`] above
/// the ground is a deck, past [`BORE_COVER_M`] below it is a bore, and the
/// threshold crossing *is* the portal (S5 — the mouth sits where the road
/// actually emerges, not where a mapper split the way).
///
/// Both thresholds are calibrated against the measured population rather than
/// reasoned from the solid's own geometry; see their docs for what reasoning
/// them cost.
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
        } else if gap <= -BORE_COVER_M + CEILING_CONTACT_M {
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
    let level = if kind == SpanKind::Bridge { DECK_STANDOFF_M } else { -BORE_COVER_M };
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
        // The portal is where the ground first covers the tube, interpolated
        // between nodes — not the node the hill starts at. The terrain climbs
        // 40 m between node 7 (arc 700) and node 8, so the mouth sits an
        // eighth of the way along it.
        let want = 700.0 + 100.0 * (BORE_COVER_M / 40.0);
        assert!((runs[0].arc0 - want).abs() < 1.0, "arc0 {} want {want}", runs[0].arc0);
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
        // 5 m node spacing, so the two threshold crossings either side of the
        // touch fall well inside `SNAP_RUN_M` and coalesce. A viaduct is not
        // two viaducts because one pier's ground reading came up.
        let n = 41;
        let road = vec![100.0; n];
        let mut terrain = vec![60.0; n];
        terrain[20] = 99.0; // one node brushing the standoff
        let runs = derive(&profile(n, 200.0, road, terrain), prior());
        assert_eq!(runs.len(), 1, "one viaduct, got {runs:?}");
    }

    /// A bore clamped to its licensed ceiling sits at exactly
    /// `terrain − BORE_COVER_M` — the license's own arithmetic — and must
    /// still read as a bore. A strict threshold censored 21 such tunnels
    /// (1.3 km) on the Montreux extract: at the guarantee is *in*, not out.
    #[test]
    fn a_bore_at_its_licensed_ceiling_is_a_bore() {
        let n = 21;
        let terrain = vec![100.0; n];
        let road: Vec<f64> = terrain.iter().map(|t| t - BORE_COVER_M).collect();
        let runs = derive(&profile(n, 2000.0, road, terrain), prior());
        assert_eq!(runs.len(), 1, "one bore, got {runs:?}");
        assert_eq!(runs[0].kind, SpanKind::Tunnel);
    }

    /// A benched road reads *below* the raw DEM — it was cut into the hillside
    /// and `terrain_m` is the ground before the cut. 35 % of at-grade nodes sit
    /// between −2 m and 0 for exactly this reason, and a bore threshold of zero
    /// called every one of them a tunnel.
    #[test]
    fn a_cutting_is_not_a_bore() {
        let n = 21;
        let road = vec![100.0; n];
        let terrain = vec![101.8; n]; // 1.8 m of cut — no room for a tube
        assert!(derive(&profile(n, 2000.0, road, terrain), prior()).is_empty());
    }
}
