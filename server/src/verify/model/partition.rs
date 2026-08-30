//! How far the pure partition sits from the fold's spans.
//!
//! `solve::partition` states a corridor's spans as one function of the
//! profile, the annotation and the licenses; the fold arrives at them by
//! thirteen mutations. The two-pass refactor (`data/plans/
//! pure-partition-2026-08-28.md`) replaces the fold with the function, and
//! its first step is to know the distance: metres of centerline on which
//! the two name different kinds, per corridor, split by family. The expected
//! families are the bridge ends (the withdrawn slice's trim, `Bridge → Grade`)
//! and the short-span set; anything else is the function missing a mechanism.

use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// Under this the two differ by a stub the fold's own floor would drop.
const EPS_M: f64 = 0.5;

pub fn check(m: &Model<'_>) -> Vec<Metric> {
    let mut dist = Dist::new(0.0, 4096.0);
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    let (mut b2g, mut g2b, mut t2g, mut g2t, mut other) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for e in &m.solved.partition {
        dist.push(e.d.metres);
        b2g += e.d.bridge_to_grade;
        g2b += e.d.grade_to_bridge;
        t2g += e.d.tunnel_to_grade;
        g2t += e.d.grade_to_tunnel;
        other += e.d.other;
        if e.d.metres > EPS_M {
            let kind = m
                .scene
                .corridors
                .get(e.corridor as usize)
                .map_or_else(|| "?".to_string(), |c| format!("{:?}", c.kind));
            worst.offer(Offender {
                lon: e.lon,
                lat: e.lat,
                zoom: m.solved.z_ref,
                value: e.d.metres,
                note: format!(
                    "{kind} #{}: {:.0} m differ (bridge→grade {:.0}, grade→bridge {:.0}, \
                     tunnel→grade {:.0}, grade→tunnel {:.0}, other {:.0}); longest stretch {:.0} m",
                    e.corridor,
                    e.d.metres,
                    e.d.bridge_to_grade,
                    e.d.grade_to_bridge,
                    e.d.tunnel_to_grade,
                    e.d.grade_to_tunnel,
                    e.d.other,
                    e.d.worst_metres
                ),
            });
        }
    }
    vec![Metric {
        id: "partition.divergence".into(),
        invariant: Invariant::I2,
        title: "Where the pure partition disagrees with the fold".into(),
        population: "Every corridor that ends the write-back holding any structure span, \
                     scored by the metres of centerline on which solve::partition — the same \
                     predicates as one function, plus the bridge half clamped to the derived \
                     deck runs — names a different kind than the spans the fold wrote."
            .into(),
        detail: format!(
            "The distance the two-pass refactor has to close, measured before anything \
             moves. Over this extract the fold's spans and the pure partition differ by \
             {:.0} m in total: bridge→grade {b2g:.0} m (the bridge ends the withdrawn slice \
             trimmed), grade→bridge {g2b:.0} m, tunnel→grade {t2g:.0} m, grade→tunnel \
             {g2t:.0} m, other {other:.0} m. Bridge ends and the short-span set are the \
             expected families; anything else is a mechanism the function is missing.",
            b2g + g2b + t2g + g2t + other
        ),
        sense: Sense::HigherIsWorse,
        threshold: EPS_M,
        skipped: dist.is_empty().then(|| "no structure-bearing corridor in this extract".into()),
        dist,
        worst: worst.into_vec(),
    }]
}
