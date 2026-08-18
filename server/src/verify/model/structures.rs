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
//!   Measured per annotated *end* against the nearest same-kind run edge:
//!   the first cut paired each run with the span it overlapped most and
//!   scored both end offsets, under which an 85 m fragment of the Chillon
//!   viaduct's coverage reported 2,254 m of "end movement" — the span's own
//!   length echoed back, the same span-length-as-finding disease the
//!   handover pairing had (docs/VERIFICATION.md).
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
            // Where the derive agrees a structure exists here, how far each of
            // this span's ends sits from the derived world's nearest edge.
            if let Some((d, arc)) = end_drift(runs, s) {
                drift.push(d);
                if d > EPS_M {
                    offer(&mut drift_worst, m, c, arc, d, &format!(
                        "{:?} end moved {d:.0} m from where it was annotated",
                        s.kind
                    ));
                }
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

    // §8's structural claim for §4.5: every clearance demand has a solved
    // feature on both sides, **zero by construction** — when structures are
    // outputs there is no "crossing whose bridge was deleted". A non-zero
    // count is a design failure, not a quality regression: a crossing derived
    // from a corridor the solve then failed to profile. `lower: None` is not
    // an orphan — it names a crossed feature whose height *is* the ground.
    let mut orphan = Dist::new(0.0, 2.0);
    let mut orphan_worst = Worst::new(Sense::HigherIsWorse, 8);
    for c in &m.solved.crossings {
        let upper_solved = m.solved.profile(c.upper).is_some();
        let lower_solved = c.lower.map_or(true, |id| m.solved.profile(id).is_some());
        let orphaned = !upper_solved as u8 + !lower_solved as u8;
        orphan.push(orphaned as f64);
        if orphaned > 0 {
            orphan_worst.offer(crate::verify::Offender {
                lon: c.point.x,
                lat: c.point.y,
                zoom: m.solved.z_ref,
                value: orphaned as f64,
                note: format!(
                    "crossing with {} unsolved side(s): upper #{} {}, lower {:?}",
                    orphaned,
                    c.upper,
                    if upper_solved { "solved" } else { "UNSOLVED" },
                    c.lower
                ),
            });
        }
    }

    vec![
        Metric {
            id: "crossing.orphan".into(),
            invariant: Invariant::I3,
            title: "A clearance demand with an unsolved side".into(),
            population: "Every derived crossing in the solved model — the exact set the \
                         clearance demands are seeded from — scored by how many of its sides \
                         name a corridor the solve holds no profile for. A crossing whose \
                         lower side is a plain surface feature (its height is the ground) is \
                         whole, not orphaned."
                .into(),
            detail: "Structurally zero by §4.5: structures are outputs, so \"a crossing whose \
                     bridge was deleted\" is unrepresentable — unless a crossing survives a \
                     corridor its profile did not. Any count here is a design failure rather \
                     than a quality regression (GENERATION.md §8)."
                .into(),
            sense: Sense::HigherIsWorse,
            threshold: 0.5,
            skipped: orphan.is_empty().then(|| "no crossings in this extract".to_string()),
            dist: orphan,
            worst: orphan_worst.into_vec(),
        },
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
            "Every annotated non-grade span that a derived run of its kind overlaps, each end \
             scored against the nearest such run's matching edge; the worse end is the sample. \
             Per-end, because pairing whole runs echoed a long span's own length back as \
             \"movement\" wherever its coverage was fragmented.",
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

/// How far this span's ends sit from the derived world's, and the arc of the
/// worse end. `None` where no same-kind run overlaps the span (that is
/// `annotated_lost`'s whole-loss family, not drift).
///
/// Each end is scored against the **nearest** same-kind overlapping run's
/// matching edge, and the worse end wins. Pairing a whole run against the
/// span it overlaps most — the first cut — scored *both* of the run's end
/// offsets, so any fragment of a long structure's coverage reported nearly
/// the span's length as "movement" (an 85 m fragment of the 2,339 m Chillon
/// annotation read 2,254 m). Nearest-per-end keeps the honest cases intact:
/// a 1:1 pair scores identically, a fused run still reports how far the
/// derived structure's edge sits beyond the mapper's split, and a missing
/// end fragment reports the real distance to where the derivation stops.
fn end_drift(runs: &[StructureRun], s: &Span) -> Option<(f64, f64)> {
    let overlapping =
        || runs.iter().filter(|r| r.kind == s.kind && r.arc1 > s.arc0 && r.arc0 < s.arc1);
    overlapping().next()?;
    let d0 = overlapping().map(|r| (r.arc0 - s.arc0).abs()).fold(f64::INFINITY, f64::min);
    let d1 = overlapping().map(|r| (r.arc1 - s.arc1).abs()).fold(f64::INFINITY, f64::min);
    Some(if d0 >= d1 { (d0, s.arc0) } else { (d1, s.arc1) })
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

#[cfg(test)]
mod tests {
    use super::*;

    fn span(arc0: f64, arc1: f64) -> Span {
        Span { arc0, arc1, level: 1, kind: SpanKind::Bridge }
    }

    fn run(arc0: f64, arc1: f64) -> StructureRun {
        StructureRun { arc0, arc1, kind: SpanKind::Bridge }
    }

    /// The 1:1 case both pairings agree on: one run, shifted and trimmed.
    #[test]
    fn a_single_run_scores_its_worse_end() {
        let (d, arc) = end_drift(&[run(20.0, 90.0)], &span(0.0, 100.0)).unwrap();
        assert_eq!(d, 20.0);
        assert_eq!(arc, 0.0);
    }

    /// The Chillon disease: fragmented coverage of one long span must report
    /// the real end offsets, not a fragment's distance to the far end. The
    /// run-paired rule read 890 m here (fragment 0..110 against end 1000).
    #[test]
    fn fragmented_coverage_does_not_echo_the_span_length() {
        let runs = [run(10.0, 110.0), run(400.0, 500.0), run(880.0, 990.0)];
        let (d, _) = end_drift(&runs, &span(0.0, 1000.0)).unwrap();
        assert_eq!(d, 10.0);
    }

    /// A fused run (the mapper split one structure in two) still reports how
    /// far the derived edge sits beyond this span's annotated end.
    #[test]
    fn a_fused_run_reports_the_absorbed_distance() {
        let (d, arc) = end_drift(&[run(0.0, 250.0)], &span(150.0, 250.0)).unwrap();
        assert_eq!(d, 150.0);
        assert_eq!(arc, 150.0);
    }

    /// No overlapping run is whole loss, not zero drift.
    #[test]
    fn no_overlap_is_no_sample() {
        assert!(end_drift(&[run(200.0, 300.0)], &span(0.0, 100.0)).is_none());
        // A different kind does not pair either.
        let bore = StructureRun { arc0: 0.0, arc1: 100.0, kind: SpanKind::Tunnel };
        assert!(end_drift(&[bore], &span(0.0, 100.0)).is_none());
    }
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
