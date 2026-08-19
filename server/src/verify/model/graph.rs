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
