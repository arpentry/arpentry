//! What the annotations claim, against what the solved heights imply.
//!
//! §4.5 inverts structures: a deck exists where the solved surface departs the
//! ground, not where a mapper wrote `bridge`. `solve::structures` derives that
//! world; the tiler, the ground carves and the junction sheets still cut
//! against the annotated one.
//!
//! This is the instrument for closing that gap, and it exists *before* the
//! switch rather than after it. Five consumers move at once when the derived
//! runs take over, and the honest question before moving them is not "will it
//! work" but "how far apart are the two worlds, on the real extract, right
//! now". Three numbers answer it:
//!
//! - **`structure.annotated_lost`** — annotated structure the solve does not
//!   imply. These are the decks that would *vanish*: a bridge tagged over flat
//!   ground, an annotation edge past where the road reaches the ground. §2.1
//!   says to expect them, and S10 says the model must degrade rather than
//!   spectacle when it finds them.
//! - **`structure.derived_new`** — structure the solve implies that nothing
//!   annotated. These would *appear*: a road standing clear of a ravine that
//!   no one tagged. This is the half the annotation-driven model could never
//!   build.
//! - **`structure.edge_drift`** — where both agree a structure exists, how far
//!   its ends move. The annotation ends where a mapper split the way; the
//!   derivation ends where the road actually reaches the ground (S5).
//!
//! All three are reported in **metres of centerline**, not counts, because a
//! count cannot distinguish one long viaduct from a hundred slivers.

use crate::scene::{Span, SpanKind};
use crate::solve::structures::StructureRun;
use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// Overlap shorter than this is quantization at a shared edge, not agreement.
const EPS_M: f64 = 0.5;

/// These metrics measure **centerline length**, not height, so they need a
/// range to match. `Dist::metres()` spans ±32 m — right for a clearance and
/// useless here: every structure longer than 32 m saturates, and the first run
/// of this check duly reported a median, tail and worst that were all the same
/// number. Measure the anatomy before believing the shape.
fn lengths() -> Dist {
    Dist::new(0.0, 4096.0)
}

pub fn check(m: &Model<'_>) -> Vec<Metric> {
    let mut lost = lengths();
    let mut lost_worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut new = lengths();
    let mut new_worst = Worst::new(Sense::HigherIsWorse, 8);
    let mut drift = lengths();
    let mut drift_worst = Worst::new(Sense::HigherIsWorse, 8);

    for c in &m.scene.corridors {
        let runs = m.solved.structures.get(c.id as usize).map(Vec::as_slice).unwrap_or(&[]);
        let annotated: Vec<&Span> = c.spans.iter().filter(|s| s.kind != SpanKind::Grade).collect();

        for s in &annotated {
            let covered = overlap_with(runs, s.arc0, s.arc1, s.kind);
            let missing = (s.arc1 - s.arc0 - covered).max(0.0);
            lost.push(missing);
            if missing > EPS_M {
                offer(&mut lost_worst, m, c, 0.5 * (s.arc0 + s.arc1), missing, &format!(
                    "{:?} annotated over {:.0} m, {missing:.0} m of it not implied by the solve",
                    s.kind,
                    s.arc1 - s.arc0
                ));
            }
        }

        for r in runs {
            let covered: f64 = annotated
                .iter()
                .filter(|s| s.kind == r.kind)
                .map(|s| (r.arc1.min(s.arc1) - r.arc0.max(s.arc0)).max(0.0))
                .sum();
            let unclaimed = (r.arc1 - r.arc0 - covered).max(0.0);
            new.push(unclaimed);
            if unclaimed > EPS_M {
                offer(&mut new_worst, m, c, 0.5 * (r.arc0 + r.arc1), unclaimed, &format!(
                    "{:?} implied over {:.0} m, {unclaimed:.0} m of it unannotated",
                    r.kind,
                    r.arc1 - r.arc0
                ));
            }
            // Where an annotation and a run overlap, how far the ends moved.
            if let Some(s) = annotated
                .iter()
                .filter(|s| s.kind == r.kind && r.arc1 > s.arc0 && r.arc0 < s.arc1)
                .max_by(|a, b| {
                    let ov = |s: &Span| (r.arc1.min(s.arc1) - r.arc0.max(s.arc0)).max(0.0);
                    ov(a).partial_cmp(&ov(b)).unwrap_or(std::cmp::Ordering::Equal)
                })
            {
                let d = (r.arc0 - s.arc0).abs().max((r.arc1 - s.arc1).abs());
                drift.push(d);
                if d > EPS_M {
                    offer(&mut drift_worst, m, c, r.arc0, d, &format!(
                        "{:?} end moved {d:.0} m from where it was annotated",
                        r.kind
                    ));
                }
            }
        }
    }

    vec![
        metric(
            "structure.annotated_lost",
            "Annotated structure the solve does not imply",
            "Every non-grade span of every corridor, scored by the metres of it no derived run \
             of the same kind covers. Metres of centerline, not a count: one long viaduct and a \
             hundred slivers are not the same finding.",
            "What would vanish when the derived runs take over. §2.1 says to expect it — a \
             bridge tagged over flat ground, an annotation edge past where the road reaches the \
             ground — and S10 says the answer is a plain draped road, not spectacle.",
            lost,
            lost_worst,
        ),
        metric(
            "structure.derived_new",
            "Structure the solve implies that nothing annotated",
            "Every derived run, scored by the metres of it no annotated span of the same kind \
             covers.",
            "What would appear: a road standing clear of a ravine nobody tagged. This is the \
             half an annotation-driven model can never build, and the reason the inversion is \
             worth its risk.",
            new,
            new_worst,
        ),
        metric(
            "structure.edge_drift",
            "How far a structure's ends move when derived",
            "Every derived run that overlaps an annotated span of the same kind, against the \
             span it overlaps most, scored by the larger of its two end movements.",
            "The annotation ends where a mapper split the way; the derivation ends where the \
             road actually reaches the ground (S5). Drift here is the correction, not the error \
             — but it is also how far every consumer's geometry moves, so it is worth knowing \
             before five of them switch at once.",
            drift,
            drift_worst,
        ),
    ]
}

/// Metres of `[arc0, arc1]` covered by runs of the same kind.
fn overlap_with(runs: &[StructureRun], arc0: f64, arc1: f64, kind: SpanKind) -> f64 {
    runs.iter()
        .filter(|r| r.kind == kind)
        .map(|r| (arc1.min(r.arc1) - arc0.max(r.arc0)).max(0.0))
        .sum()
}

fn offer(
    w: &mut Worst,
    m: &Model<'_>,
    c: &crate::scene::Corridor,
    arc: f64,
    value: f64,
    note: &str,
) {
    let Some(p) = m.solved.profile(c.id) else { return };
    let pt = p.point_at_arc(arc);
    w.offer(Offender { lon: pt.x, lat: pt.y, zoom: m.solved.z_ref, value, note: note.into() });
}

fn metric(
    id: &str,
    title: &str,
    population: &str,
    detail: &str,
    dist: Dist,
    worst: Worst,
) -> Metric {
    Metric {
        id: id.into(),
        // §4.5's own invariant is I6: an annotation the model cannot honour
        // must cost detail, never produce spectacle.
        invariant: Invariant::I6,
        title: title.into(),
        population: population.into(),
        detail: detail.into(),
        sense: Sense::HigherIsWorse,
        threshold: EPS_M,
        skipped: dist.is_empty().then(|| "no structures in this extract".to_string()),
        dist,
        worst: worst.into_vec(),
    }
}
