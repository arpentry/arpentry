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
        // The *annotation snapshot*, not the working spans: after the solve's
        // write-back the working spans are the reconciled truth, and scoring
        // the derived runs against them would compare the solve to itself.
        let annotated: Vec<&Span> =
            m.scene.annotated(c.id).iter().filter(|s| s.kind != SpanKind::Grade).collect();

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

    // The crossing premise, measured at the solve's own gate
    // (`solve::crossings::covered_sites`): wherever a mapped tunnel span is
    // crossed by another alignment's at-grade band, the crossing machinery
    // waives the clearance demand on the strength of the annotation
    // (`solve::graph::in_immovable_bore`). These entries score whether the
    // waived-for bore actually passes beneath the ground that band rides on.
    let mut daylight = Dist::metres();
    let mut daylight_worst = Worst::new(Sense::HigherIsWorse, 8);
    for d in &m.solved.daylight {
        daylight.push(d.deficit_m);
        if d.deficit_m > EPS_M {
            let kind = m
                .scene
                .corridors
                .get(d.corridor as usize)
                .map_or_else(|| "?".to_string(), |c| format!("{:?}", c.kind));
            daylight_worst.offer(crate::verify::Offender {
                lon: d.lon,
                lat: d.lat,
                zoom: m.solved.z_ref,
                value: d.deficit_m,
                note: format!(
                    "{kind} mapped bore crossed by an at-grade band, roof + cover {:.1} m \
                     above its own ground",
                    d.deficit_m
                ),
            });
        }
    }

    vec![
        Metric {
            id: "structure.bore_daylight".into(),
            // The waived clearance is an I3 ordering fact: two surfaces cross,
            // and the model's answer is "the lower one is underground". This
            // measures that answer.
            invariant: Invariant::I3,
            title: "A crossed mapped bore standing clear of its own ground".into(),
            population: "Every place a mapped tunnel span is crossed in plan by another \
                         alignment whose own annotation is at grade there — the exact set the \
                         solver holds under the ground (the same gate, \
                         solve::crossings::covered_sites), and the exact set the crossing \
                         machinery waives clearance for. Scored by roof + cover minus this \
                         corridor's own terrain, signed: negative is burial margin."
                .into(),
            detail: "The split-brain check. A crossing over a mapped bore buys no clearance \
                     (the road above stands on the ground, the feature below runs under it) — \
                     but that is a premise about the solved geometry, not a fact of the \
                     annotation. Where the solve leaves the bore at the surface, the waiver \
                     stands on nothing: road band and rail band draw a storey apart with \
                     neither a bore nor a deck between them, which is the Territet funicular \
                     crossing at 6.9234,46.4275."
                .into(),
            sense: Sense::HigherIsWorse,
            threshold: EPS_M,
            skipped: daylight.is_empty().then(|| "no crossed mapped bores".to_string()),
            dist: daylight,
            worst: daylight_worst.into_vec(),
        },
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
