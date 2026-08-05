//! Structures carried by draped features (docs/GENERATION.md §4.2).
//!
//! A footbridge is a **D** feature. It has no authority, it never enters a
//! solve, and it constrains nothing upstream — *"carrying a structure span is
//! not a promotion"*. That discipline is load-bearing: draped features are
//! 46.9 % of the road network (§2.2), and any loophole admitting one into a
//! solve is a loophole through which half the network can perturb the other
//! half.
//!
//! But a footbridge is still a bridge, and drawing it as a line on the ground
//! puts a path through the river it crosses. So the deck is **fitted to the
//! finished world** rather than solved with it: the ground is sampled at the
//! two ends of the annotated span, the deck is the straight chord between
//! them, and that chord is swept as a slab by the same generator every other
//! structure uses.
//!
//! A chord, not a fit, because there is nothing to fit *to*. A solved corridor
//! has anchors, a grade ceiling and neighbours to reconcile with; this has an
//! annotation and two endpoints. The chord is what the data supports, and
//! [`crate::priors::PATH_STRUCTURE_HALF_WIDTH_M`] keeps it pedestrian-scale so
//! it never reads as a road deck.
//!
//! Only elevated spans are built. A *draped tunnel* — a path annotated as
//! passing under something — has no bore to sweep, because its road surface is
//! the ground: the gap never goes negative, [`structure::stamp`] declines, and
//! the caller drapes it. That is the degradation ladder working, not a gap.

use crate::ground::sampler::GroundSampler;
use crate::project::Bounds;
use crate::scene::SpanKind;
use crate::solve::Profile;
use crate::tile_build::EncoderFeature;

use super::structure;

/// Sweeps a deck for one annotated span of a draped feature, chorded between
/// the finished ground at its ends. Returns whether a solid was emitted;
/// `false` tells the caller to drape the line instead (the degradation ladder).
///
/// `z_ref` reads the ground at the reference rung rather than the tile's own,
/// so the two ends of a span crossing a tile border are sampled from one
/// surface and the deck comes out identical from either side (I5).
pub fn stamp(
    f: &mut EncoderFeature,
    sampler: &mut GroundSampler,
    z_ref: u8,
    bounds: &Bounds,
) -> bool {
    let Some(nodes) = span_nodes(f) else { return false };
    if nodes.len() < 2 {
        return false;
    }
    // The finished ground under every node, and the chord between the ends.
    let terrain: Vec<f64> =
        nodes.iter().map(|c| sampler.ground(c.x, c.y, z_ref)).collect();
    let profile = Profile::from_heights(&nodes, chord(&nodes, &terrain), terrain);
    structure::stamp(f, &profile, SpanKind::Bridge, bounds)
}

/// The deck line: a straight chord in height between the first and last node,
/// parameterised by arc so a curving span still rises evenly along its length.
fn chord(nodes: &[geo_types::Coord], terrain: &[f64]) -> Vec<f64> {
    let cos_lat = crate::scene::run_cos_lat(nodes);
    let mut arc = Vec::with_capacity(nodes.len());
    let mut acc = 0.0;
    for (i, &c) in nodes.iter().enumerate() {
        if i > 0 {
            acc += crate::scene::metric_len(nodes[i - 1], c, cos_lat);
        }
        arc.push(acc);
    }
    let (h0, h1) = (terrain[0], terrain[terrain.len() - 1]);
    let total = acc;
    if total <= 0.0 {
        return vec![h0; nodes.len()];
    }
    arc.iter().map(|&a| h0 + (h1 - h0) * (a / total)).collect()
}

/// The feature's own vertices — it was already cut to the annotated span in
/// phase 1, so the whole line is the structure.
fn span_nodes(f: &EncoderFeature) -> Option<Vec<geo_types::Coord>> {
    match &f.geometry {
        geo_types::Geometry::LineString(l) => Some(l.0.clone()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::Coord;

    fn line(n: usize, span_deg: f64) -> Vec<Coord> {
        (0..n)
            .map(|i| Coord { x: 6.0 + span_deg * i as f64 / (n - 1) as f64, y: 46.0 })
            .collect()
    }

    #[test]
    fn the_deck_is_a_straight_chord_between_the_ends() {
        // A footbridge over a ravine: the ground dives in the middle and the
        // deck must not follow it. The chord is what the data supports.
        let nodes = line(5, 0.002);
        let terrain = vec![100.0, 80.0, 60.0, 80.0, 110.0];
        let deck = chord(&nodes, &terrain);
        assert_eq!(deck[0], 100.0, "starts on the ground it leaves");
        assert_eq!(deck[4], 110.0, "ends on the ground it meets");
        // Evenly spaced nodes, so the interior is the linear interpolation —
        // it flies over the ravine instead of diving into it.
        assert!((deck[2] - 105.0).abs() < 1e-9, "mid-span {} is the chord", deck[2]);
        assert!(deck[2] > terrain[2] + 40.0, "the deck clears the ravine");
    }

    #[test]
    fn a_level_span_gives_a_level_deck() {
        let nodes = line(4, 0.001);
        let deck = chord(&nodes, &[200.0, 195.0, 205.0, 200.0]);
        assert!(deck.iter().all(|h| (h - 200.0).abs() < 1e-9), "flat ends, flat deck");
    }

    #[test]
    fn a_degenerate_span_does_not_divide_by_zero() {
        let nodes = vec![Coord { x: 6.0, y: 46.0 }, Coord { x: 6.0, y: 46.0 }];
        let deck = chord(&nodes, &[100.0, 100.0]);
        assert!(deck.iter().all(|h| h.is_finite()));
    }
}
