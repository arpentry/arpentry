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
    let mut out = monotonicity(m);
    out.extend(single_source(m));
    out
}

/// How far the published ground may sit from the surface a structural decision
/// read before that decision was taken against a ground nobody drew.
///
/// **This population does not separate, and the number is reasoned rather than
/// read.** Measured over the Montreux extract (`ARPT_DEBUG_SS`): n = 2,744,
/// p50 0.00, p75 0.00, p90 0.63, p95 1.48, p97 2.25, p98 3.51, p99 5.80,
/// max 12.51. Three quarters of structure nodes agree *exactly* — the ground
/// stage never touched them — and past that it climbs smoothly with no gap to
/// cut at, so the honest thing is to say so instead of inventing a cliff.
///
/// The line comes from what the consumers themselves tolerate. A bore's ceiling
/// is set to the reference less roof and cover, so it has
/// [`crate::priors::TUNNEL_COVER_M`] = 0.5 m of slack before the tube surfaces;
/// a deck is called buried at 2.0 m (`clearance.deck_over_ground`). One metre
/// sits between the two: past it the two grounds are different surfaces at the
/// scale the structural decisions care about, whichever decision it was.
const SINGLE_SOURCE_M: f64 = 1.0;

/// **I1 — one ground function**, the last row of GENERATION.md §8 without an
/// instrument.
///
/// > At any plan position there is exactly one engineered ground height, and
/// > every consumer reads it. No generator samples the raw DEM outside terrain
/// > conditioning.
///
/// The second sentence is the measurable one, and the honest place to measure
/// it is where a consumer's read and the published ground can differ *without
/// that consumer's own imprint accounting for it*. Under an at-grade road the
/// two differ by the road's own bench, which is the imprint working as designed
/// (§4.3) and is already scored by `contact.kerb_*`. Inside a **structure
/// span** the corridor lays no bench: it is on a deck or under the ground, so
/// whatever moved the published ground there was somebody else — and the
/// solve's decisions at those nodes (the bore ceiling, the portal, the tube's
/// fit, the deck's growth) were all taken against the reference surface
/// instead.
///
/// That is the S21 family stated as an invariant. The Vernex-Dessus gallery's
/// ceiling was capped against a reference 6.5 m above the ground the town's
/// benches finally carved, and no stage re-read it; the fix had to be a drawing
/// rule precisely because the depth that mattered was junior-*solved*
/// (`synth::structure::drawn_runs`). This is the number that would have said so
/// before the screenshot did.
///
/// The reference is passed to the stack as its own base, so the DEM-versus-
/// rendered-lattice difference cancels and what is left is purely what the
/// ground stage did. One known term is the corridor's *own* portal daylighting
/// carve, which is written from the same portal solve being scored; the class
/// this exists for is the interior divergence, and an offender inside a run
/// rather than at its mouth is the one to read.
fn single_source(m: &Model<'_>) -> Vec<Metric> {
    let mut dist = Dist::metres();
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut scratch: Vec<u32> = Vec::new();
    // ARPT_DEBUG_SS: the sorted population, for the threshold read. `Dist`
    // bins, and a threshold wants the sorted list (docs/VERIFICATION.md §9).
    let mut all: Vec<f64> = Vec::new();
    for c in &m.scene.corridors {
        let Some(p) = m.solved.profile(c.id) else { continue };
        let (arc, at_grade, terrain) = (p.arc(), p.at_grade(), p.terrain_m());
        for k in 0..arc.len() {
            if at_grade[k] {
                continue; // its own bench explains the difference
            }
            let pt = p.point_at_arc(arc[k]);
            if !m.bounds.contains(pt.x, pt.y) {
                continue;
            }
            let read = terrain[k];
            let published = m.ground.height(pt.x, pt.y, read, 0.0, &mut scratch);
            let v = (published - read).abs();
            dist.push(v);
            all.push(v);
            if v > SINGLE_SOURCE_M {
                worst.offer(Offender {
                    lon: pt.x,
                    lat: pt.y,
                    zoom: m.solved.z_ref,
                    value: v,
                    note: format!(
                        "{:?} {} solved its structure here against {read:.2} m; the ground                          drawn is {published:.2} m",
                        c.kind, c.id
                    ),
                });
            }
        }
    }
    if std::env::var_os("ARPT_DEBUG_SS").is_some() {
        all.sort_by(f64::total_cmp);
        let q = |f: f64| all.get(((all.len() - 1) as f64 * f) as usize).copied().unwrap_or(0.0);
        eprintln!(
            "[ss] n={} p50 {:.2} p75 {:.2} p90 {:.2} p95 {:.2} p97 {:.2} p98 {:.2} p99 {:.2} \
             p995 {:.2} max {:.2}",
            all.len(),
            q(0.50),
            q(0.75),
            q(0.90),
            q(0.95),
            q(0.97),
            q(0.98),
            q(0.99),
            q(0.995),
            all.last().copied().unwrap_or(0.0)
        );
    }
    vec![Metric {
        id: "ground.single_source".into(),
        invariant: Invariant::I1,
        title: "A structural decision taken against a ground nobody drew".into(),
        population: "Every profile node inside a structure span, over every corridor the                      solve holds a profile for, clipped to the extract bbox. Scored as the                      published engineered ground minus the reference surface the solve read                      at that node, with the reference passed to the stack as its own base so                      only the ground stage's own work is left. At-grade nodes are excluded:                      there the difference is the corridor's own bench, which is the imprint                      working and is scored by contact.kerb_*."
            .into(),
        detail: format!(
            "I1 says there is one ground and every consumer reads it. A structure node is              where that can fail invisibly — the corridor benches nothing there, so the gap              between what its portal, ceiling and fit decisions read and what the tiler              finally drew is somebody else's imprint, re-read by nobody. Past              {SINGLE_SOURCE_M:.1} m the decision stands on a surface that is not in the              scene. The corridor's own portal carve is one term of this near a mouth; the              class worth reading is an offender in the interior of a run."
        ),
        sense: Sense::HigherIsWorse,
        threshold: SINGLE_SOURCE_M,
        skipped: dist.is_empty().then(|| "no structure nodes in this extract".into()),
        dist,
        worst: worst.into_vec(),
    }]
}

fn monotonicity(m: &Model<'_>) -> Vec<Metric> {
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
