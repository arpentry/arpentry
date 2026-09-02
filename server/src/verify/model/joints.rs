//! Where a structure span ends short of a junction its corridor shares.
//!
//! Within one corridor the band and the deck agree by construction — both
//! read the reconciled spans and the same cross-section cut. The bare metres
//! `seam.band_deck_bare` finds are the boundary that construction cannot
//! reach: a physical structure continuing from corridor A into corridor B
//! across a junction the splice refused (a fork, a class or link change, a
//! near-180° pair), where each side's span truth stops at its own annotation
//! edge. Reconciliation (`solve::reconcile_stratum`) is corridor-local and
//! never looks across.
//!
//! The archive check sees meshes with no corridor identity, so the
//! cross-corridor case cannot be split out there. This is the model half: for
//! every junction member whose corridor holds a Bridge span ending within
//! [`JOINT_REACH_M`] of the member's arc with only Grade between, the gap —
//! split by whether an aligned continuation exists at that junction:
//!
//! - **cross-corridor** (exactly one member continues the heading): the
//!   population the junction weld (`solve::joints`, to come) grows spans
//!   through, and this metric's own distribution;
//! - **terminal** (no continuation): a genuine data edge that must never be
//!   welded — counted, because it is the guard the weld's gate is judged
//!   against;
//! - **ambiguous** (two or more continuations): welded never, counted always.

use crate::scene::SpanKind;
use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// Mirror of `verify::checks::handoff::REACH_M`, so this census's population
/// is the drawn check's population seen from the model side.
const JOINT_REACH_M: f64 = 12.0;

/// A span ending within this of the junction arc is at it — the handover cut
/// machinery owns that case, and there is nothing to weld.
const AT_M: f64 = 0.5;

/// Two tangents whose |cosine| clears this continue one another — the same
/// boundary `assemble::corridors::continues_through` draws for the splice.
const CONTINUES_DOT: f64 = 0.5;

pub fn check(m: &Model<'_>) -> Vec<Metric> {
    let mut dist = Dist::metres();
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let (mut cross, mut terminal, mut ambiguous) = (0usize, 0usize, 0usize);

    for j in m.scene.junctions.iter() {
        // The scene spills past the extract's bbox by whole row groups; a gap
        // out there ranks the extraction boundary, not the pipeline.
        if !m.bounds.contains(j.point.x, j.point.y) {
            continue;
        }
        for member in &j.members {
            let Some(c) = m.scene.corridors.get(member.corridor as usize) else { continue };
            // The nearest Bridge span end on either side of the member's arc,
            // with only Grade between it and the junction.
            let mut gap: Option<f64> = None;
            for s in &c.spans {
                if s.kind != SpanKind::Bridge {
                    continue;
                }
                let g = if s.arc1 <= member.arc {
                    let g = member.arc - s.arc1;
                    // Another structure between the span end and the junction
                    // owns the boundary instead.
                    if c.spans
                        .iter()
                        .any(|o| o.kind != SpanKind::Grade && o.arc0 >= s.arc1 && o.arc1 <= member.arc + AT_M && o.arc0 != s.arc0)
                    {
                        continue;
                    }
                    g
                } else if s.arc0 >= member.arc {
                    let g = s.arc0 - member.arc;
                    if c.spans
                        .iter()
                        .any(|o| o.kind != SpanKind::Grade && o.arc1 <= s.arc0 && o.arc0 >= member.arc - AT_M && o.arc0 != s.arc0)
                    {
                        continue;
                    }
                    g
                } else {
                    continue; // the junction is inside the span
                };
                if gap.is_none_or(|b| g < b) {
                    gap = Some(g);
                }
            }
            let Some(gap) = gap else { continue };
            if gap <= AT_M || gap > JOINT_REACH_M {
                continue;
            }
            // Does any other member continue this corridor's heading?
            let Some(ta) = tangent(c, member.arc) else { continue };
            let mut continuations = 0usize;
            let mut partner = "";
            for o in &j.members {
                if o.corridor == member.corridor {
                    continue;
                }
                let Some(oc) = m.scene.corridors.get(o.corridor as usize) else { continue };
                let Some(tb) = tangent(oc, o.arc) else { continue };
                if (ta.0 * tb.0 + ta.1 * tb.1).abs() > CONTINUES_DOT {
                    continuations += 1;
                    partner = &oc.class_key;
                }
            }
            match continuations {
                1 => {
                    cross += 1;
                    dist.push(gap);
                    let heights = m.solved.profile(member.corridor).map_or_else(
                        String::new,
                        |p| format!("; deck side at {:.2} m", p.road_at_arc(member.arc)),
                    );
                    worst.offer(Offender {
                        lon: j.point.x,
                        lat: j.point.y,
                        zoom: m.solved.z_ref,
                        value: gap,
                        note: format!(
                            "a {} bridge span stops {gap:.1} m short of the junction its \
                             {partner} continuation shares{heights}",
                            c.class_key
                        ),
                    });
                }
                0 => terminal += 1,
                _ => ambiguous += 1,
            }
        }
    }

    vec![Metric {
        id: "partition.junction_joint".into(),
        invariant: Invariant::I2,
        title: "Bridge span ending short of a shared junction".into(),
        population: format!(
            "Every junction member (inside the extract bbox) whose corridor holds a Bridge \
             span ending within {JOINT_REACH_M:.0} m of the member's arc with only Grade \
             between, where exactly one other member continues the heading — the \
             cross-corridor gaps a junction-joint weld would close. Excluded and counted in \
             the detail: ends with no aligned continuation (terminal — never weldable) and \
             with two or more (ambiguous — welded never, by conservatism)."
        ),
        detail: format!(
            "The arc metres between the span end and the junction. Within a corridor the \
             band and deck agree by construction; this is the boundary the corridor-local \
             reconciliation cannot reach, and it is where seam.band_deck_bare's tail lives. \
             This extract: {cross} cross-corridor, {terminal} terminal, {ambiguous} ambiguous."
        ),
        sense: Sense::HigherIsWorse,
        threshold: AT_M,
        skipped: dist
            .is_empty()
            .then(|| format!("no cross-corridor short span ends ({terminal} terminal, {ambiguous} ambiguous)")),
        dist,
        worst: worst.into_vec(),
    }]
}

/// The corridor's unit tangent (metric space) at an arc, `None` on a
/// degenerate polyline.
fn tangent(c: &crate::scene::Corridor, at: f64) -> Option<(f64, f64)> {
    if c.nodes.len() < 2 {
        return None;
    }
    let i = c.arc.partition_point(|&a| a < at).clamp(1, c.nodes.len() - 1);
    let (a, b) = (c.nodes[i - 1], c.nodes[i]);
    let (dx, dy) = ((b.x - a.x) * c.cos_lat, b.y - a.y);
    let len = dx.hypot(dy);
    (len > 0.0).then(|| (dx / len, dy / len))
}
