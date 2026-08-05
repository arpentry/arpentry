//! I8 — ground monotonicity.
//!
//! > `groundₙ₊₁` differs from `groundₙ` only inside stratum *n*'s declared
//! > footprints, and each stratum's imprint is applied exactly once.
//!
//! The second half is a construction invariant: [`crate::ground::GroundStack::new`]
//! asserts the layers are distinct and ascending, and the fold visits each once.
//! The first half is what this measures, and it is *not* vacuous even though
//! `covers` and `height` ask the same grid and the same reach: `batter_reach`
//! separately clamps how far a face may run, and a change to either side could
//! stop the declared footprint bounding the actual influence without anything
//! noticing. The whole point of a declared footprint is that something checks
//! the declaration.

use geo_types::Coord;

use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// Plan spacing of the lattice walked, in metres. Fine enough that a bench
/// (~8 m half-width) cannot hide between samples, coarse enough to sweep a city.
const SPACING_M: f64 = 4.0;

/// Cap on samples, so a continental extract still answers in bounded time. When
/// it bites the metric says so rather than reading like full coverage.
const MAX_SAMPLES: usize = 4_000_000;

pub fn check(m: &Model<'_>) -> Vec<Metric> {
    let layers = m.ground.layers();
    if layers.is_empty() {
        return vec![Metric {
            id: "ground.footprint".into(),
            invariant: Invariant::I8,
            title: "Ground moved outside the imprinting stratum's footprint".into(),
            population: "not measured".into(),
            detail: "no ground layers in this run".into(),
            sense: Sense::HigherIsWorse,
            threshold: 0.0,
            skipped: Some("no ground layers".into()),
            dist: Dist::metres(),
            worst: Vec::new(),
        }];
    }

    let b = extent(m);
    let cos = ((b.1 + b.3) * 0.5).to_radians().cos().max(0.1);
    let dlat = SPACING_M / crate::scene::DEG_M;
    let dlon = SPACING_M / (crate::scene::DEG_M * cos);
    let rows = (((b.3 - b.1) / dlat).ceil() as usize).max(1);
    let cols = (((b.2 - b.0) / dlon).ceil() as usize).max(1);
    let stride = ((rows * cols) as f64 / MAX_SAMPLES as f64).sqrt().ceil().max(1.0) as usize;
    let truncated = stride > 1;

    let mut dist = Dist::metres();
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut scratch: Vec<u32> = Vec::new();
    let mut samples = 0u64;
    let mut moved_inside = 0u64;
    for r in (0..rows).step_by(stride) {
        for c in (0..cols).step_by(stride) {
            let lat = b.1 + r as f64 * dlat;
            let lon = b.0 + c as f64 * dlon;
            // A constant stands in for the DEM: the predicate is *which layer
            // moved the ground*, and a layer's declared reach does not depend
            // on the height it starts from. It does change which branch runs —
            // every face reads as fill against a zero datum — which exercises
            // more faces than a real terrain would, not fewer.
            let raw = 0.0;
            let mut prev = m.ground.height_through(0, lon, lat, raw, 0.0, &mut scratch);
            for (n, layer) in layers.iter().enumerate() {
                let next = m.ground.height_through(n + 1, lon, lat, raw, 0.0, &mut scratch);
                samples += 1;
                if next.to_bits() == prev.to_bits() {
                    dist.push(0.0);
                } else if layer.covers(lon, lat, &mut scratch) {
                    moved_inside += 1;
                    dist.push(0.0);
                } else {
                    let outside = (next - prev).abs();
                    dist.push(outside);
                    worst.offer(Offender {
                        lon,
                        lat,
                        zoom: m.solved.z_ref,
                        value: outside,
                        note: format!(
                            "layer {n} ({:?}) moved the ground {outside:.3} m outside its declared footprint",
                            layer.stratum
                        ),
                    });
                }
                prev = next;
            }
        }
    }

    let detail = "Walks a lattice and folds the stack one layer at a time. Where a layer \
                      changed the ground, its own declared footprint must cover the point. Zero \
                      by construction unless a reach stops bounding its own influence — which is \
                      what makes it worth measuring, since `batter_reach`'s clamp is the only \
                      reason it holds."
        .to_string();
    // Anatomy, so a zero cannot be read as "nothing was tested". A run where no
    // layer ever moved the ground would score a perfect zero and prove nothing,
    // so the count of probes that *did* move is part of what the metric states.
    let mut population = format!(
        "Every ground layer at every point of a {SPACING_M:.0} m lattice over the extract: \
         `groundₙ₊₁` against `groundₙ`, and where they differ, whether layer n declares it. Of \
         {samples} probes, {moved_inside} found a layer moving the ground inside its own \
         footprint — the population the predicate is actually about."
    );
    if truncated {
        population.push_str(&format!(
            " Coverage: sampled every {stride} lattice steps to stay inside the sample cap; a \
             denser walk would find more."
        ));
    }
    vec![Metric {
        id: "ground.footprint".into(),
        invariant: Invariant::I8,
        title: "Ground moved outside the imprinting stratum's footprint".into(),
        population,
        detail,
        sense: Sense::HigherIsWorse,
        threshold: 0.0,
        skipped: None,
        dist,
        worst: worst.into_vec(),
    }]
}

/// The extract's plan extent, from the corridors themselves — the scene is what
/// the ground was built from, so it is what bounds the walk.
fn extent(m: &Model<'_>) -> (f64, f64, f64, f64) {
    let mut b = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    let mut note = |c: &Coord| {
        b.0 = b.0.min(c.x);
        b.1 = b.1.min(c.y);
        b.2 = b.2.max(c.x);
        b.3 = b.3.max(c.y);
    };
    for c in &m.scene.corridors {
        for n in &c.nodes {
            note(n);
        }
    }
    if b.0 > b.2 {
        return (0.0, 0.0, 0.0, 0.0);
    }
    b
}
