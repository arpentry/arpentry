//! When a footbridge is really the sidewalk on a road bridge.
//!
//! Overture maps a road bridge's separated footway as a feature in its own
//! right, tagged `bridge` on its own level run. Taken at face value that is a
//! **D** feature carrying a structure span, so [`super::draped`] fits it a deck
//! of its own, chorded to the finished ground at the span's two ends. Where the
//! path runs *along* a road bridge the ground it reads is the ground *under*
//! that bridge, and what gets drawn is a second, smaller structure hanging
//! beneath the real one — joined to it at whichever abutment the annotation and
//! the DEM happen to agree on, and diving away from it at the other. Over the
//! Montreux extract that is 22.7 % of every footbridge in the scene.
//!
//! The answer is not to fit a better deck. It is that **there is no second
//! bridge**: the path is on the road's. So a carried span stops being a fitted
//! deck and becomes a line riding the carrier's solved deck — the same
//! mechanism a tunnel's ribbon uses to ride its bore ramp
//! (`Synth::Road { deck: true }`). Nothing is stamped, so no duplicate solid
//! exists to hang.
//!
//! **This is not a promotion** (§4.2). The footway reads the street stratum's
//! datum and writes nothing back: it never enters a solve, it perturbs no
//! height, and deleting it changes nothing (I7). Reading the finished world is
//! exactly what a draped feature is supposed to do — the only change is that
//! the finished world under a sidewalk is a deck rather than the ground.
//!
//! ## What counts as carried
//!
//! Three tests, each of which rejects a population the others admit. They are
//! restated in `verify::checks::contact` rather than shared, so the measurement
//! can disagree with the rule.
//!
//! - **Alongside** ([`LATERAL_M`]). `examples/carried_probe` searched out to
//!   25 m and found every carried path between 2.0 m and 9.5 m of a solved
//!   centerline, and *nothing at all* from 9.5 m to 25 m. Any cut through that
//!   empty band claims the same spans.
//! - **Along it, not across it** ([`ALONG`]). A footway crossing over a bridge
//!   is beside it for its whole width, which on a short span is most of the
//!   span. The arc of the carrier a run projects onto shrinks by the cosine of
//!   the angle between them, so comparing that arc against the run's own length
//!   *is* the angle test.
//! - **Joined to it** ([`JOIN_M`]). The one that matters most: a sidewalk
//!   arrives at its bridge's abutment, so where the annotation and the DEM
//!   agree at one end the chord already lands on the deck. Without it the worst
//!   "carried" span in the extract is a 12 m footbridge over a stream
//!   *underneath* a motorway viaduct whose deck is 68 m overhead.

use geo_types::Coord;

use crate::assemble::grid::GridIndex;
use crate::ground::sampler::GroundSampler;
use crate::scene::{metric_len, run_cos_lat, CorridorId, SceneGraph, SpanKind, DEG_M};
use crate::solve::SolvedModel;

/// How far a path may run from a solved deck's centerline and still be its
/// sidewalk. See the module note: the measured population leaves a 15 m band
/// empty on either side of this, so it is not a delicate number.
const LATERAL_M: f64 = 10.0;

/// How much of a span must run alongside one deck before it is carried by it.
/// A sidewalk runs the length of its bridge (measured coverage p25 0.91,
/// p50 1.00). What this rejects is a long walkway that rides a bridge for part
/// of its length and carries itself for the rest — a span a rule that seats
/// the *whole* run on a carrier has no business claiming.
const COVER: f64 = 0.7;

/// How nearly the run must follow the carrier rather than cross it, as the
/// fraction of its own length it advances along the carrier's arc — which is
/// the cosine of the angle between them, so this is 30°.
const ALONG: f64 = 0.87;

/// How closely the fitted chord must already meet the carrier's deck at one
/// end of the run. A sidewalk shares its bridge's abutment; a path passing
/// underneath touches it at neither end.
///
/// Sorted, the 25 candidates the extract offers separate themselves: seventeen
/// meet their carrier within 1.91 m, then **nothing until 4.69 m**, then the
/// three that are passing under one — a path with 4.7 m of headroom under a
/// secondary road, a footway under a railway bridge at 6.3 m, and the motorway
/// viaduct 65 m overhead. Anything from 2.0 m to 4.6 m gives the same answer,
/// so this sits in the middle of that band rather than at either edge. A
/// metre, chosen from the median instead, cuts straight through a cluster —
/// 1.16, 1.26, 1.45, 1.76, 1.77, 1.85, 1.91 — and leaves the extract's worst
/// sidewalk hanging 5.3 m under its bridge.
const JOIN_M: f64 = 3.0;

/// Plan spacing along a run while testing it against a carrier.
const STEP_M: f64 = 2.0;

/// One solved bridge deck, keyed by the corridor it belongs to and the arc
/// range over which that corridor is actually swept as a deck.
struct Deck {
    corridor: CorridorId,
    arc0: f64,
    arc1: f64,
}

/// Every solved bridge deck in the scene, indexed by plan position.
///
/// Built once after the solve and shared read-only across the phase-1 workers:
/// the question is asked a few hundred times over a city extract (once per
/// elevated span of a draped feature), and a linear scan of every corridor
/// would walk every profile in the scene for each one.
pub struct Carriers {
    decks: Vec<Deck>,
    index: GridIndex,
}

impl Carriers {
    pub fn build(scene: &SceneGraph, solved: &SolvedModel) -> Carriers {
        let mut decks: Vec<Deck> = Vec::new();
        let mut index = GridIndex::new();
        for c in &scene.corridors {
            let Some(p) = solved.profile(c.id) else { continue };
            // The spans are the solved-reconciled truth (`solve::reconcile_stratum`).
            for s in c.spans.iter().copied() {
                if s.kind != SpanKind::Bridge {
                    continue;
                }
                // The deck's plan extent, walked at the same step the query
                // uses, padded by the lateral reach so the index never has to
                // be asked about a cell the answer could not be in.
                let (mut w, mut e) = (f64::INFINITY, f64::NEG_INFINITY);
                let (mut s0, mut n) = (f64::INFINITY, f64::NEG_INFINITY);
                let mut a = s.arc0;
                loop {
                    let pt = p.point_at_arc(a);
                    (w, e) = (w.min(pt.x), e.max(pt.x));
                    (s0, n) = (s0.min(pt.y), n.max(pt.y));
                    if a >= s.arc1 {
                        break;
                    }
                    a = (a + STEP_M).min(s.arc1);
                }
                if !w.is_finite() {
                    continue;
                }
                let pad_lat = LATERAL_M / DEG_M;
                let pad_lon = pad_lat / c.cos_lat.max(1e-6);
                index.insert(
                    (w - pad_lon, s0 - pad_lat, e + pad_lon, n + pad_lat),
                    decks.len() as u32,
                );
                decks.push(Deck { corridor: c.id, arc0: s.arc0, arc1: s.arc1 });
            }
        }
        Carriers { decks, index }
    }

    /// The corridor whose solved deck is carrying this run, or `None` where the
    /// run is a structure of its own.
    ///
    /// `nodes` is the run's own plan line — the piece already cut to the level
    /// run, so its two ends are the abutments the fitted deck would have been
    /// chorded between. The ground is read at those two ends for exactly that
    /// reason: the comparison is against the deck [`super::draped`] *would
    /// have* built, not against the terrain in general.
    pub fn of(
        &self,
        nodes: &[Coord],
        scene: &SceneGraph,
        solved: &SolvedModel,
        sampler: &mut GroundSampler,
        z_ref: u8,
    ) -> Option<CorridorId> {
        if nodes.len() < 2 || self.decks.is_empty() {
            return None;
        }
        let cos_lat = run_cos_lat(nodes);
        let mut arc = Vec::with_capacity(nodes.len());
        let mut acc = 0.0;
        for (i, &c) in nodes.iter().enumerate() {
            if i > 0 {
                acc += metric_len(nodes[i - 1], c, cos_lat);
            }
            arc.push(acc);
        }
        let total = acc;
        if total < 1.0 {
            return None;
        }
        // The chord the fit would have built, so a carrier can be compared
        // against the thing it is replacing.
        let ends = (
            sampler.ground(nodes[0].x, nodes[0].y, z_ref),
            sampler.ground(nodes[nodes.len() - 1].x, nodes[nodes.len() - 1].y, z_ref),
        );
        let steps = (total / STEP_M).ceil() as usize;
        let pts: Vec<(f64, Coord)> = (0..=steps)
            .map(|i| {
                let s = total * i as f64 / steps as f64;
                (s, point_at(nodes, &arc, s))
            })
            .collect();

        let (mut w, mut e) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut s0, mut n) = (f64::INFINITY, f64::NEG_INFINITY);
        for &(_, c) in &pts {
            (w, e) = (w.min(c.x), e.max(c.x));
            (s0, n) = (s0.min(c.y), n.max(c.y));
        }
        let mut cand = Vec::new();
        self.index.query((w, s0, e, n), &mut cand);

        let mut best: Option<(CorridorId, usize)> = None;
        for id in cand {
            let d = &self.decks[id as usize];
            let Some(p) = solved.profile(d.corridor) else { continue };
            let corr = &scene.corridors[d.corridor as usize];
            let mut hits = 0usize;
            // The first and last samples that found this deck: where the run
            // enters and leaves it, in its own arc and in the carrier's.
            let (mut enter, mut leave) = (None, None);
            for &(s, c) in &pts {
                let a = p.arc_of(c.x, c.y);
                if a < d.arc0 - STEP_M || a > d.arc1 + STEP_M {
                    continue;
                }
                if metric_len(c, p.point_at_arc(a), corr.cos_lat) > LATERAL_M {
                    continue;
                }
                hits += 1;
                let join = p.deck_at_arc(a) - (ends.0 + (ends.1 - ends.0) * (s / total));
                enter = enter.or(Some((s, a, join)));
                leave = Some((s, a, join));
            }
            if !carries(enter, leave, hits, pts.len()) {
                continue;
            }
            if best.is_none_or(|(_, h)| hits > h) {
                best = Some((d.corridor, hits));
            }
        }
        best.map(|(c, _)| c)
    }
}

/// The three tests, over what the walk saw.
///
/// `enter` and `leave` are the first and last samples that found this deck, each
/// as `(arc along the run, arc along the carrier, carrier deck minus the fitted
/// chord)`; `hits` is how many samples found it out of `samples`. Kept apart
/// from the projection geometry above because this is the rule, and the rule is
/// what wants reading — and testing — on its own.
fn carries(
    enter: Option<(f64, f64, f64)>,
    leave: Option<(f64, f64, f64)>,
    hits: usize,
    samples: usize,
) -> bool {
    if (hits as f64) < COVER * samples as f64 {
        return false; // alongside for part of its length, its own bridge for the rest
    }
    let (Some(i0), Some(i1)) = (enter, leave) else { return false };
    // Along it, not across it: over the stretch the run shares with this deck,
    // how much of the carrier's own arc it advances. That ratio is the cosine
    // of the angle between them.
    let shared = i1.0 - i0.0;
    if shared < 1.0 || (i1.1 - i0.1).abs() < ALONG * shared {
        return false;
    }
    // And it must arrive at the deck at one end or the other.
    i0.2.abs().min(i1.2.abs()) <= JOIN_M
}

/// The point at arc distance `s` along the line, interpolated between vertices.
fn point_at(nodes: &[Coord], arc: &[f64], s: f64) -> Coord {
    let i = arc.partition_point(|&a| a < s).clamp(1, arc.len() - 1);
    let (a0, a1) = (arc[i - 1], arc[i]);
    let t = if a1 > a0 { ((s - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
    Coord {
        x: nodes[i - 1].x + (nodes[i].x - nodes[i - 1].x) * t,
        y: nodes[i - 1].y + (nodes[i].y - nodes[i - 1].y) * t,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A run of `len_m` sampled every [`STEP_M`], seen against a carrier over
    /// `from_m..to_m` of its own length, advancing `advance` metres of the
    /// carrier's arc over that stretch, and disagreeing in height by `join`
    /// metres at the near end and `far` at the other.
    fn seen(
        len_m: f64,
        from_m: f64,
        to_m: f64,
        advance: f64,
        join: f64,
        far: f64,
    ) -> (Option<(f64, f64, f64)>, Option<(f64, f64, f64)>, usize, usize) {
        let samples = (len_m / STEP_M).ceil() as usize + 1;
        let hits = ((to_m - from_m) / STEP_M).ceil() as usize + 1;
        (Some((from_m, 100.0, join)), Some((to_m, 100.0 + advance, far)), hits, samples)
    }

    /// The case the rule exists for: a 30 m sidewalk running the length of its
    /// road bridge, meeting the deck at the abutment it shares and sinking
    /// away from it at the other.
    #[test]
    fn a_sidewalk_along_its_whole_bridge_is_carried() {
        let (a, b, h, n) = seen(30.0, 0.0, 30.0, 30.0, 0.1, 3.8);
        assert!(carries(a, b, h, n));
    }

    /// A path passing underneath touches the deck at neither end, however
    /// perfectly it runs along it — the 68 m motorway viaduct in the extract.
    #[test]
    fn a_path_under_a_viaduct_is_not_carried() {
        let (a, b, h, n) = seen(12.0, 0.0, 12.0, 12.0, 68.2, 65.6);
        assert!(!carries(a, b, h, n), "a bridge overhead is not a bridge underfoot");
    }

    /// A footbridge crossing a road bridge is beside it for the road's whole
    /// width, which on a short span is most of the span — so coverage alone
    /// cannot reject it. Advancing 3 m of the carrier's arc over a 20 m shared
    /// stretch is a crossing, and this is the test that says so.
    #[test]
    fn a_crossing_is_not_carried_however_close_it_runs() {
        let (a, b, h, n) = seen(24.0, 2.0, 22.0, 3.0, 0.2, 0.4);
        assert!(!carries(a, b, h, n), "across a bridge is not along it");
    }

    /// A long walkway that rides a bridge for a quarter of its length carries
    /// itself for the rest, and a rule that seats the whole run on a carrier
    /// has nothing to say about it. Three such spans are in the extract.
    #[test]
    fn a_walkway_only_partly_alongside_keeps_its_own_deck() {
        let (a, b, h, n) = seen(53.0, 0.0, 12.0, 12.0, 0.3, 3.5);
        assert!(!carries(a, b, h, n));
    }

    /// Either end will do: a sidewalk joins its bridge at one abutment, and
    /// which one is an accident of where the DEM and the annotation agree.
    #[test]
    fn joining_at_the_far_end_is_enough() {
        let (a, b, h, n) = seen(30.0, 0.0, 30.0, 30.0, 4.2, 0.1);
        assert!(carries(a, b, h, n));
    }

    /// A run that found the deck at no sample at all is not carried by it,
    /// rather than dividing by zero on its way to an answer.
    #[test]
    fn a_deck_seen_at_no_sample_carries_nothing() {
        assert!(!carries(None, None, 0, 16));
    }
}
