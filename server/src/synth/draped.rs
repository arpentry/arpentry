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
//!
//! ## Where the chord may start
//!
//! The chord is only as good as the two points it is read at, and a span end
//! is exactly where the data is least trustworthy (§2.1: *"span boundaries are
//! not registered to the terrain"*). Against a near-vertical DEM wall — the
//! side of a gorge, a stream cut — two metres of plan disagreement between the
//! annotation and the DEM is fourteen metres of height error, and the chord
//! starts part way down the wall: a footbridge beginning in the middle of the
//! riverbed it crosses. [`seat`] is the constraint that answers it, and it runs
//! before the span is cut rather than here, because moving an abutment moves
//! the boundary between the deck and the path draped up to it.

use geo_types::{Coord, LineString};

use crate::ground::sampler::GroundSampler;
use crate::levels::LevelRun;
use crate::project::Bounds;
use crate::scene::{metric_len, run_cos_lat, SpanKind};
use crate::solve::Profile;
use crate::tile_build::EncoderFeature;

use super::structure;

/// A ground climb steeper than this, immediately outside an abutment, is not
/// ground the path can be standing on: it is a wall the annotation's edge
/// landed on, and the abutment belongs at the top of it.
///
/// Calibrated against the population rather than reasoned. Over the 220
/// abutments of the Montreux extract's draped spans the ground's own outward
/// grade at an abutment is p50 0.09, p75 0.32, p95 0.83 — most abutments stand
/// on ground that is nearly flat, as an abutment should. What the choice buys,
/// per `examples/footdeck_probe`:
///
/// | ceiling | abutments re-seated | median move | median lift |
/// |--------:|--------------------:|------------:|------------:|
/// |    40 % |             17.7 %  |       4.0 m |      1.92 m |
/// |    60 % |             10.9 %  |       4.0 m |      2.98 m |
/// |    80 % |              5.5 %  |       2.0 m |      4.15 m |
/// |   100 % |              2.3 %  |       4.0 m |      6.65 m |
///
/// 60 % sits between the population's p75 and p95: past it the ground is
/// steeper than a path walks, and the four spans that gave this defect its name
/// have walls of 111 %, 207 %, 243 % and 307 %. Lower would start re-seating
/// abutments on ordinary Alpine hillsides, which is ground a path does use.
const WALL_GRADE: f64 = 0.6;

/// How far outward an abutment may be re-seated. The measured moves are short
/// (p50 4 m, and a wall is a wall precisely because it is steep), so this is a
/// backstop: a wall still climbing at the limit has not proved a bank, and the
/// span keeps the ends it was annotated with.
const SEAT_REACH_M: f64 = 20.0;

/// Ground spacing while walking the wall, and the baseline the climb is
/// measured over. Matched to the detail DEM's own resolution: finer resolves
/// nothing real, coarser steps over the rim.
const SEAT_STEP_M: f64 = 2.0;

/// Smallest lift worth moving an abutment for. Below this the annotation and
/// the ground already agree about where the bank is, and moving the cut
/// between the deck and its approach buys nothing.
const SEAT_LIFT_MIN_M: f64 = 1.0;

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
    z: u8,
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
    // The fitted profile is all at-grade, so the per-zoom structure datum
    // (`synth::datum`) finds no run here and the fitted deck stays absolute.
    structure::stamp(f, &profile, SpanKind::Bridge, sampler, z, z_ref, bounds)
        == structure::Stamped::Solid
}

/// Re-seats every elevated span's abutments on the ground that can carry them.
///
/// **A path cannot descend a cliff.** Where the ground immediately outside an
/// abutment climbs faster than [`WALL_GRADE`], the span's edge did not land on
/// a bank — it landed part way up a wall, and the path the annotation says
/// arrives there cannot be on it. So the abutment walks outward along the
/// path's own line until the climb relaxes (the bank) or the ground rises to
/// meet the span's *other* abutment (the deck's own level), whichever comes
/// first.
///
/// Two properties make this safe to run over every span rather than over
/// hand-picked ones:
///
/// - **Only the lower abutment moves, and never past the higher one.** The
///   higher abutment is the evidence of how high the ground comes at the span's
///   edge; capping there means the correction can only ever make a deck *less*
///   steep, so it cannot invent a structure the data does not support.
/// - **It stops where the ground stops.** The new abutment is a point on the
///   ground, so the deck still meets the ground at both ends (invariant 4) and
///   the approach drapes up to it with no step.
///
/// Where the wall is still climbing at [`SEAT_REACH_M`], or there is no path
/// beyond the span to walk (18.6 % of abutments on the Montreux extract: the
/// bridge is its own segment, so its neighbour's geometry is another feature),
/// nothing moves and the span keeps its annotated ends. That is the
/// degradation ladder, not a gap: an uncorrected span is what is drawn today.
///
/// Runs in phase 1, on the whole source line before any tile sees it, so every
/// tile and every zoom cuts at the same abutment (invariant 5).
pub fn seat(
    line: &LineString,
    runs: &[LevelRun],
    sampler: &mut GroundSampler,
    z: u8,
) -> Vec<LevelRun> {
    seat_on(&line.0, runs, &mut |c| sampler.ground(c.x, c.y, z))
}

/// [`seat`] against a bare ground field, so the rule can be tested without a
/// DEM and a ground stack behind it.
fn seat_on(nodes: &[Coord], runs: &[LevelRun], ground: &mut impl FnMut(Coord) -> f64) -> Vec<LevelRun> {
    let mut out = runs.to_vec();
    if nodes.len() < 2 {
        return out;
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
    if total <= 0.0 {
        return out;
    }
    for i in 0..out.len() {
        if out[i].level <= 0 {
            continue; // a draped tunnel has no deck to seat
        }
        let (s0, s1) = (out[i].start * total, out[i].end * total);
        if s1 - s0 <= 0.0 {
            continue;
        }
        // The neighbouring runs bound the walk: an abutment may not be seated
        // inside another span. Read off `out`, not the input, so a run already
        // seated outward on this line still bounds the next one.
        let (from, to) = (out[i].start, out[i].end);
        let before = out
            .iter()
            .filter(|r| r.end <= from)
            .fold(0.0, |m: f64, r| m.max(r.end * total));
        let after = out
            .iter()
            .filter(|r| r.start >= to)
            .fold(total, |m: f64, r| m.min(r.start * total));
        let mut ground_at = |s: f64| ground(point_at(nodes, &arc, s));
        let (h0, h1) = (ground_at(s0), ground_at(s1));
        // Only the lower end moves, and only up to the height of the higher.
        if h0 < h1 {
            let reach = (s0 - before).min(SEAT_REACH_M);
            if let Some(d) = walk(s0, -1.0, h1, reach, &mut ground_at) {
                out[i].start = (s0 - d) / total;
            }
        } else if h1 < h0 {
            let reach = (after - s1).min(SEAT_REACH_M);
            if let Some(d) = walk(s1, 1.0, h0, reach, &mut ground_at) {
                out[i].end = (s1 + d) / total;
            }
        }
    }
    out
}

/// Walks the wall outward from an abutment and returns how far the abutment
/// should move, or `None` where nothing is proved.
///
/// Three ways out, in the order they are tested at each sample:
///
/// 1. **The ground reached the deck's own level.** The abutment belongs where
///    they meet, interpolated between samples — snapping to the sample would
///    be up to a step wrong, and a step is 2 m of a bridge that may be 13 m
///    long.
/// 2. **The climb relaxed.** The wall has topped out: the bank is the previous
///    sample, and this is the case that stops a genuine hillside from dragging
///    the abutment up it, because a hillside relaxes to a walkable grade long
///    before a gorge wall does.
/// 3. **Neither, out to the limit.** No bank is provable, so nothing moves.
fn walk(
    from: f64,
    dir: f64,
    target: f64,
    limit: f64,
    ground: &mut impl FnMut(f64) -> f64,
) -> Option<f64> {
    let start = ground(from);
    let (mut prev_d, mut prev_h) = (0.0, start);
    let mut d = SEAT_STEP_M;
    while d <= limit {
        let h = ground(from + dir * d);
        if h >= target {
            let span = h - prev_h;
            let t = if span > 0.0 { ((target - prev_h) / span).clamp(0.0, 1.0) } else { 0.0 };
            let hit = prev_d + (d - prev_d) * t;
            return (target - start >= SEAT_LIFT_MIN_M).then_some(hit);
        }
        if (h - prev_h) / (d - prev_d) < WALL_GRADE {
            return (prev_h - start >= SEAT_LIFT_MIN_M).then_some(prev_d);
        }
        (prev_d, prev_h) = (d, h);
        d += SEAT_STEP_M;
    }
    None
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

    /// A line running east at 46° N with a node every metre, so an arc distance
    /// in metres is an index — the ground can then be written as a profile.
    fn metre_line(len_m: usize) -> Vec<Coord> {
        let deg = 1.0 / (crate::scene::DEG_M * 46.0_f64.to_radians().cos());
        (0..=len_m).map(|i| Coord { x: 6.0 + deg * i as f64, y: 46.0 }).collect()
    }

    /// A ground field written as a piecewise-linear profile of `(metre,
    /// height)` breakpoints along [`metre_line`], read by plan position.
    struct Profile {
        len_m: f64,
        pts: Vec<(f64, f64)>,
    }

    impl Profile {
        fn new(len_m: usize, pts: &[(f64, f64)]) -> Profile {
            Profile { len_m: len_m as f64, pts: pts.to_vec() }
        }

        fn at(&self, s: f64) -> f64 {
            let w = self.pts.windows(2).find(|w| s <= w[1].0).unwrap_or(&self.pts[self.pts.len() - 2..]);
            let (a, b) = (w[0], w[1]);
            let t = ((s - a.0) / (b.0 - a.0)).clamp(0.0, 1.0);
            a.1 + (b.1 - a.1) * t
        }

        /// The ground closure `seat_on` reads, and the arc position it is asked
        /// at, recovered from the plan point.
        fn ground<'a>(&'a self, nodes: &'a [Coord]) -> impl FnMut(Coord) -> f64 + 'a {
            let x0 = nodes[0].x;
            let per_m = (nodes[nodes.len() - 1].x - x0) / self.len_m;
            move |c: Coord| self.at((c.x - x0) / per_m)
        }

        fn run(&self, start: f64, end: f64, level: i64) -> LevelRun {
            LevelRun { start: start / self.len_m, end: end / self.len_m, level }
        }

        fn arc_of(&self, r: &LevelRun) -> (f64, f64) {
            (r.start * self.len_m, r.end * self.len_m)
        }
    }

    /// The defect this rule exists for, at the shape the Montreux extract gave
    /// it: a gorge whose wall is near-vertical, and a span whose edge landed
    /// part way down it because the annotation and the DEM disagree by a couple
    /// of metres in plan. The abutment climbs back to the bank.
    #[test]
    fn an_abutment_on_a_wall_climbs_to_the_bank() {
        let len = 40;
        let nodes = metre_line(len);
        // Near bank at 430 out to 12 m, a wall into the riverbed at 415.5, and
        // a far bank climbing to 440 — higher than the near one, so the walk
        // ends at the bank rather than at the far abutment's height.
        let p = Profile::new(
            len,
            &[(0.0, 430.0), (12.0, 430.0), (20.0, 415.5), (24.0, 415.5), (32.0, 440.0), (40.0, 440.0)],
        );
        let annotated = p.run(19.0, 31.0, 1);
        let seated = seat_on(&nodes, &[annotated], &mut p.ground(&nodes));
        let (s0, s1) = p.arc_of(&seated[0]);
        assert!((s1 - 31.0).abs() < 1e-6, "the far abutment was already on its bank");
        assert!(
            p.at(s0) > 429.0,
            "seated at {:.1} m ({:.1} m), not on the 430 m bank",
            s0,
            p.at(s0)
        );
        assert!(s0 < 13.0, "the bank is at 12 m; seated at {s0:.1} m");
    }

    /// The correction can only ever make a deck less steep: the moving end
    /// stops when the ground reaches the height of the end that did not move,
    /// however much further the wall climbs above it.
    #[test]
    fn an_abutment_never_climbs_past_its_opposite() {
        let len = 40;
        let nodes = metre_line(len);
        // The near wall runs to 470 — far above the 433 the other abutment
        // sits at — so only the crossing of 433 may be taken.
        let p = Profile::new(
            len,
            &[(0.0, 470.0), (16.0, 420.0), (20.0, 415.5), (24.0, 415.5), (32.0, 435.5), (40.0, 435.5)],
        );
        let annotated = p.run(19.0, 31.0, 1);
        let seated = seat_on(&nodes, &[annotated], &mut p.ground(&nodes));
        let (s0, s1) = p.arc_of(&seated[0]);
        let (h0, h1) = (p.at(s0), p.at(s1));
        assert!(h0 <= h1 + 1e-6, "seated at {h0:.2} m, above its opposite at {h1:.2} m");
        assert!(h0 > h1 - 0.5, "stopped short of the level it could reach: {h0:.2} vs {h1:.2}");
    }

    /// A path that genuinely walks down into a valley and crosses at the bottom
    /// keeps the span it was annotated with: the ground outside its abutments
    /// is a grade a path walks, not a wall. This is the case a rim search finds
    /// and gets wrong — the rim is real, and it is 20 m above a bridge that is
    /// exactly where it should be.
    #[test]
    fn a_walkable_descent_keeps_its_span() {
        let len = 60;
        let nodes = metre_line(len);
        // A 35 % V, 20 m deep, with a metre-deep stream notch at the bottom.
        let p = Profile::new(
            len,
            &[
                (0.0, 1000.0),
                (28.0, 990.2),
                (29.0, 989.2),
                (31.0, 989.2),
                (32.0, 990.2),
                (60.0, 1000.0),
            ],
        );
        let runs = [p.run(28.0, 32.0, 1)];
        let seated = seat_on(&nodes, &runs, &mut p.ground(&nodes));
        assert_eq!(seated, runs, "nothing to correct: the flanks are walkable");
    }

    /// A span flush with its segment's end has no path beyond it to walk, so
    /// nothing moves — the degradation rung, not a panic.
    #[test]
    fn a_span_with_no_path_beyond_it_stays_put() {
        let len = 20;
        let nodes = metre_line(len);
        let p = Profile::new(len, &[(0.0, 100.0), (20.0, 140.0)]);
        let runs = [LevelRun { start: 0.0, end: 1.0, level: 1 }];
        let seated = seat_on(&nodes, &runs, &mut p.ground(&nodes));
        assert_eq!(seated, runs);
    }

    /// A draped tunnel has no deck to seat.
    #[test]
    fn only_elevated_runs_are_seated() {
        let len = 40;
        let nodes = metre_line(len);
        let p = Profile::new(
            len,
            &[(0.0, 100.0), (18.0, 100.0), (19.0, 80.0), (21.0, 80.0), (22.0, 100.0), (40.0, 100.0)],
        );
        let runs = [p.run(19.0, 21.0, -1)];
        let seated = seat_on(&nodes, &runs, &mut p.ground(&nodes));
        assert_eq!(seated, runs);
    }
}
