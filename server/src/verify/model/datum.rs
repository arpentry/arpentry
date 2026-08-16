//! Where the solved road stands, against the ground it says it lies on.
//!
//! docs/GENERATION.md §8 asks for this check by name and states why it is
//! worth having on the model side: *"`datum.float` catches errors at their
//! source. A feature drifting from its terrain is the cause of downstream
//! clearance errors; measuring it directly beats measuring the
//! three-stage-downstream symptom."*
//!
//! **At grade means on the ground.** Every other height in the model is
//! relative to that: a deck's clearance, a bore's cover, the bench the ground
//! stage cuts, the kerb the band draws. A node the solve left at grade and
//! then placed 76 m above its own hillside is not a small error in one
//! number — it is a road drawn in the air with a wall of asphalt hanging off
//! it, and every check downstream reports it as something else (the archive
//! scores it as a kerb with a cliff beside it, which says where it was drawn
//! and nothing about why).
//!
//! The budget is the class's own: `Prior::deviation_m`, the cut-and-fill a
//! road of that class is built on — 8 m for a motorway on its embankment,
//! 2.5 m for a street. The reference is the same conditioned surface the
//! profile is anchored to ([`condition_reference`]), so a filled DEM notch and
//! a shaved canopy bump — engineering the profile is entitled to assume —
//! score zero rather than the artifact's depth.

use crate::priors::{Kind, MAX_CLEARANCE_LIFT_M};
use crate::solve::profile::condition_reference;
use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// Excess under this is the solver's own relaxation residue, not a float.
const EPS_M: f64 = 0.5;

/// Range of the histogram. Wider than [`Dist::metres`]'s ±32 m because the
/// disease is unbounded — the drift a grade ceiling manufactures against a
/// mountain is limited by nothing but the sweep count, and the first run of
/// this check found 76 m of it. The extremes are exact outside the range
/// regardless; the range only decides where the percentiles have resolution.
const RANGE_M: f64 = 128.0;

pub fn check(m: &Model<'_>) -> Vec<Metric> {
    let mut dist = Dist::new(0.0, RANGE_M);
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    for c in &m.scene.corridors {
        // Water publishes a datum rather than lying on one: a still body is
        // flattened to one level by design, and scoring it against the DEM
        // under it would measure the flattening, not a float.
        if matches!(c.kind, Kind::Water(_)) {
            continue;
        }
        let Some(p) = m.solved.profile(c.id) else { continue };
        let budget = c.kind.prior().deviation_m;
        let reference = condition_reference(p.arc(), p.terrain_m());
        for (k, &at_grade) in p.at_grade().iter().enumerate() {
            // A structure node is *supposed* to leave the ground; that is what
            // makes it a structure. Only the nodes the solve itself calls at
            // grade are claiming contact — and after the write-back that flag
            // is the reconciled partition, the same one the bands are drawn
            // from (`portals::reconcile_spans`).
            if !at_grade {
                continue;
            }
            let standoff = p.road_m()[k] - reference[k];
            let excess = (standoff.abs() - budget).max(0.0);
            dist.push(excess);
            if excess > EPS_M {
                let pt = p.nodes()[k];
                worst.offer(Offender {
                    lon: pt.x,
                    lat: pt.y,
                    zoom: m.solved.z_ref,
                    value: excess,
                    note: format!(
                        "{:?} at grade {:.1} m {} its reference ground, {:.1} m past the \
                         {budget:.1} m this class is built on",
                        c.kind,
                        standoff.abs(),
                        if standoff > 0.0 { "above" } else { "below" },
                        excess
                    ),
                });
            }
        }
    }
    vec![Metric {
        id: "datum.float".into(),
        // I4 rather than §8's I7. The table's population reads "every senior
        // node", and a check restricted that way would have watched the
        // Chillon service road — junior, stratum S — float 76 m over the lake
        // shore and reported nothing. A junior floats just as visibly as a
        // senior, and what the measurement is actually about is contact.
        invariant: Invariant::I4,
        title: "Solved at-grade road standing off its own ground".into(),
        population: format!(
            "Every node of every solved profile that the *reconciled* partition leaves at \
             grade, in every stratum, water excluded. Scored by how far the solved road \
             stands from the conditioned terrain reference — in either direction, since a \
             road buried under its own hillside is the same defect upside down — less the \
             class deviation budget, so 0.0 means within the cut and fill the class is \
             built on. Structure nodes are outside the population by definition. Known \
             legitimate tail: a clearance lift raises an at-grade road over a crossing by \
             up to {MAX_CLEARANCE_LIFT_M:.0} m and keeps it at grade, so a demand-sized \
             excess near a crossing is the model working."
        ),
        detail: "A road at grade lies on the ground; every height downstream is measured \
                 from that. Drift here is a phantom embankment or a road sunk into a hill \
                 the ground stage will then carve a canyon for — and it is upstream of \
                 every clearance, cover and kerb number that will report it as something \
                 else."
            .into(),
        sense: Sense::HigherIsWorse,
        threshold: EPS_M,
        skipped: dist.is_empty().then(|| "no profiled corridors at grade".to_string()),
        dist,
        worst: worst.into_vec(),
    }]
}
