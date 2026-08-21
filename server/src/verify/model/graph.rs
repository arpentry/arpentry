//! Whether the corridors that claim to connect were solved as connected.
//!
//! The scene graph's junctions are supposed to be the source connectors: a
//! connector two corridors share is a place their surfaces are one height, and
//! the solve welds every junction's members to a shared variable
//! (`solve::graph`). But the *check* cannot read the junctions to find out —
//! a connector the assembler failed to turn into a junction is invisible
//! there, and that failure mode is exactly what this metric exists to see. The
//! Colondalles fork (6.9026,46.4455) is the type specimen: a connector
//! interior to its tertiary's segment made no junction at all, so nothing
//! welded the forking service road to the surface it joins and it solved
//! 4.7 m above it — drawn as a slab floating over the carriageway, kerb
//! across the asphalt.
//!
//! So the population is read one stage *earlier* than the junctions, from
//! [`crate::scene::Corridor::connectors`] — every connector with the corridor
//! arc it sits at, recorded by assemble for all member segments' connectors,
//! ends and interior alike. Grouping those by connector id re-derives "who
//! claims to connect where" independently of what the junction builder did
//! with it, which is what makes the metric able to indict the junction
//! builder.

use std::collections::BTreeMap;

use crate::priors::Surface;
use crate::scene::SpanKind;
use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// The step past which the render tears: `synth::sheets` stops treating two
/// overlapping carriageways as one surface at [`SHEET_SEPARATION_M`], so a
/// connector whose members disagree by more is drawn as separate slabs.
use crate::synth::sheets::SHEET_SEPARATION_M;

/// Steps run to whatever a solve can disagree by; the range only places the
/// percentiles, and the extremes are exact regardless.
const RANGE_M: f64 = 64.0;

pub fn check(m: &Model<'_>) -> Vec<Metric> {
    let mut out = connector_step(m);
    out.extend(level_crossing(m));
    out
}

/// **S15, the equality case of vertical order** — the one row of
/// GENERATION.md §8 that had no instrument.
///
/// Where two mapped alignments cross in plan and *both* are at grade there, the
/// two surfaces are the same ground and must coincide. The solve says so twice
/// over: `crossings::derive` deliberately leaves the same-level touching case
/// alone ("that is an equality rather than an inequality, and it belongs with
/// the strata"), and `graph::Contact` closes it — but only where the two share
/// a connector. A road that crosses a railway at grade without a shared node
/// gets nothing, and nothing measured whether that mattered.
///
/// The population is `crossings::plan_index`, which excludes the pairs that
/// *meet* (`meets_here`), so every member of it is a genuine crossing rather
/// than a junction. Scored as the two solved surfaces' disagreement at the
/// crossing point, over the pairs whose reconciled spans both read `Grade`
/// there. Beyond [`SEPARATION_M`] the model's own vocabulary stops calling it
/// a level crossing — two surfaces that far apart at a plan intersection are
/// grade-separated whatever the data says — so the population stops there too,
/// and `order.grade_stack` owns what lies past it. Measuring both would report
/// one defect twice.
fn level_crossing(m: &Model<'_>) -> Vec<Metric> {
    use crate::solve::crossings::{kind_at, plan_index, SEPARATION_M};
    let mut dist = Dist::metres();
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let plan = plan_index(m.scene);
    let (mut pairs, mut paved, mut both_grade, mut inside, mut separated) = (0, 0, 0, 0, 0);
    for c in &m.scene.corridors {
        let Some(p) = m.solved.profile(c.id) else { continue };
        for x in &plan[c.id as usize] {
            if x.other <= c.id {
                continue; // each pair once, from the lower id
            }
            let Some(q) = m.solved.profile(x.other) else { continue };
            let other = &m.scene.corridors[x.other as usize];
            pairs += 1;
            if c.kind.prior().surface == Surface::None
                || other.kind.prior().surface == Surface::None
            {
                continue; // a drape has no surface of its own to coincide with
            }
            paved += 1;
            if kind_at(m.scene, c.id, x.arc) != SpanKind::Grade
                || kind_at(m.scene, x.other, x.other_arc) != SpanKind::Grade
            {
                continue;
            }
            both_grade += 1;
            let pt = p.point_at_arc(x.arc);
            if std::env::var_os("ARPT_DEBUG_LX").is_some() {
                eprintln!(
                    "[lx] at-grade crossing {:.6},{:.6}  {:?} {} x {:?} {}  step {:.2}  in-bbox {}",
                    pt.x,
                    pt.y,
                    c.kind,
                    c.id,
                    other.kind,
                    other.id,
                    (q.road_at_arc(x.other_arc) - p.road_at_arc(x.arc)).abs(),
                    m.bounds.contains(pt.x, pt.y)
                );
            }
            if !m.bounds.contains(pt.x, pt.y) {
                continue;
            }
            inside += 1;
            let step = (q.road_at_arc(x.other_arc) - p.road_at_arc(x.arc)).abs();
            if step > SEPARATION_M {
                separated += 1;
                continue; // grade-separated by the model's own definition
            }
            dist.push(step);
            if step > LEVEL_CROSSING_M {
                worst.offer(Offender {
                    lon: pt.x,
                    lat: pt.y,
                    zoom: m.solved.z_ref,
                    value: step,
                    note: format!(
                        "{:?} {} and {:?} {} cross at grade but solve {step:.2} m apart",
                        c.kind, c.id, other.kind, other.id
                    ),
                });
            }
        }
    }
    if std::env::var_os("ARPT_DEBUG_LX").is_some() {
        eprintln!(
            "[lx] plan pairs {pairs}, paved {paved}, both grade {both_grade}, in bbox \
             {inside}, separated {separated}"
        );
    }
    vec![Metric {
        id: "contact.level_crossing".into(),
        invariant: Invariant::I3,
        title: "Two alignments crossing at grade, not coincident".into(),
        population: format!(
            "Every plan crossing of two profiled corridors whose classes pave a surface,              where both sides' reconciled spans read grade and their solved surfaces lie              within {SEPARATION_M:.1} m — the model's own boundary for what is still a level              crossing rather than a grade separation. Pairs that *meet* are excluded by              `plan_index` itself, so this is crossings only, and the equality is owed              whether or not they share a connector."
        ),
        detail: format!(
            "A road meeting a railway at grade is the one place two strata are known to              touch, and an equality is stronger than the inequality a grade separation              gets (§4.5). It is enforced only through a shared connector              (`solve::graph::Contact`); a crossing without one has nothing holding it, and              this is what sees that. Past {LEVEL_CROSSING_M:.2} m the two surfaces are drawn              as a step through the crossing."
        ),
        sense: Sense::HigherIsWorse,
        threshold: LEVEL_CROSSING_M,
        skipped: dist.is_empty().then(|| "no at-grade plan crossings in this extract".into()),
        dist,
        worst: worst.into_vec(),
    }]
}

/// How far two surfaces that share a level crossing may solve apart before the
/// equality has plainly not held.
///
/// Read off the population, which is only reachable outside the extract: the
/// Montreux bbox contains **no at-grade plan crossing at all** — of 808 plan
/// pairs, 799 pave a surface, 22 have both sides at grade, and every one of
/// those 22 lies in the row-group spill beyond the bbox. Sorted, those 22
/// separate cleanly: eight sit at or under 0.06 m — the weld holding, through
/// a shared connector — then **nothing until 0.24 m**, then a spread to
/// 1.06 m with one 6.97 m outlier that [`SEPARATION_M`] excludes as a grade
/// separation. This sits in that gap. A quarter of a metre, the first number
/// tried, falls *inside* the failing cluster and would pass a 0.24 m step.
const LEVEL_CROSSING_M: f64 = 0.15;

fn connector_step(m: &Model<'_>) -> Vec<Metric> {
    // Every (corridor, arc) touching each connector, over the corridors whose
    // class paves a surface and whose profile solved. A BTreeMap so the walk
    // order is a function of the model, never of hashing (invariant 5).
    let mut by_conn: BTreeMap<u64, Vec<(u32, f64)>> = BTreeMap::new();
    for c in &m.scene.corridors {
        if c.kind.prior().surface == Surface::None {
            continue; // drapes on the finished ground: no surface to step
        }
        if m.solved.profile(c.id).is_none() {
            continue;
        }
        for &(conn, arc) in &c.connectors {
            let members = by_conn.entry(conn).or_default();
            // One member per corridor: a splice records its connector from
            // both sides, and a ring returns to its own start.
            if members.iter().all(|&(id, _)| id != c.id) {
                members.push((c.id, arc));
            }
        }
    }

    let mut dist = Dist::new(0.0, RANGE_M);
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    for (_, members) in by_conn {
        if members.len() < 2 {
            continue;
        }
        // The heights the members solved for the shared place. The connector
        // is a mapped vertex, the profiles densify through their vertices, so
        // a welded junction reads one number here to float precision.
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut lo_c, mut hi_c) = (0u32, 0u32);
        let mut point = None;
        for &(cid, arc) in &members {
            let Some(p) = m.solved.profile(cid) else { continue };
            let h = p.road_at_arc(arc);
            if h < lo {
                (lo, lo_c) = (h, cid);
            }
            if h > hi {
                (hi, hi_c) = (h, cid);
            }
            point.get_or_insert_with(|| p.point_at_arc(arc));
        }
        let Some(pt) = point else { continue };
        // The scene spills past the extract (whole row groups are admitted);
        // a connector out there is solved against ground no DEM constrained.
        if !m.bounds.contains(pt.x, pt.y) {
            continue;
        }
        let step = hi - lo;
        dist.push(step);
        if step > SHEET_SEPARATION_M {
            let name = |cid: u32| {
                let c = &m.scene.corridors[cid as usize];
                format!("{:?} {}", c.kind, c.id)
            };
            worst.offer(Offender {
                lon: pt.x,
                lat: pt.y,
                zoom: m.solved.z_ref,
                value: step,
                note: format!(
                    "{} solved {step:.2} m over {} at their shared connector",
                    name(hi_c),
                    name(lo_c),
                ),
            });
        }
    }

    vec![Metric {
        id: "graph.connector_step".into(),
        invariant: Invariant::I2,
        title: "Solved height disagreement at a shared source connector".into(),
        population: format!(
            "Every source connector touched by two or more distinct profiled corridors \
             whose classes pave a surface — read from the corridors' own connector lists, \
             ends and segment-interior attachments alike, not from the junction set, so a \
             connector the assembler failed to junction still counts. Scored as the spread \
             of the members' solved road heights at the connector's arc, clipped to the \
             extract bbox. Zero is the weld holding; anything past the \
             {SHEET_SEPARATION_M} m sheet separation is drawn as one surface floating \
             over another."
        ),
        detail: "Two roads sharing a connector are one surface at it. A step here is a \
                 junction the assembler lost or a weld that failed to close, and it renders \
                 as a slab hovering over the carriageway it joins, kerb drawn across the \
                 asphalt."
            .into(),
        sense: Sense::HigherIsWorse,
        threshold: SHEET_SEPARATION_M,
        skipped: dist.is_empty().then(|| "no shared connectors among paved corridors".into()),
        dist,
        worst: worst.into_vec(),
    }]
}
