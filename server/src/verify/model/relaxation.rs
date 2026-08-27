//! Which of the relaxation's constraints actually hold at the output.
//!
//! `relax::solve` runs nine projections in a fixed, earned order (its
//! comments record which defect each position fixes), and the 2026-08-22
//! architecture review's finding 3 was that nothing at the output says
//! whether the ordering worked on *this* scene — adding or moving a pass was
//! judged by the absence of drawn artifacts downstream. These rows say it
//! directly, in the scorecard's own discipline, and they are the de-risk the
//! §3.3 pure-partition refactor was told to wait for: a two-pass solve that
//! moves these numbers is measurably wrong before anything is drawn.
//!
//! The measurement is [`crate::solve::relax::residuals`]: per constraint
//! family, the distance one further application of the family's **own pass**
//! would move the solved heights. A point is feasible for a projection
//! exactly when the projection fixes it, so the pass is the instrument and no
//! second construction of the constraint math exists to drift from the one
//! the solver enforces.

use crate::verify::{Invariant, Metric, Sense};

use super::Model;

/// A residual at or above this counts against the tally, in metres. Ten times
/// the solver's own convergence tolerance (`relax::TOL_M` = 1e-4): under it a
/// move is the limit cycle the closing settle exists to absorb, over it a
/// constraint genuinely does not hold at the output. Millimetres, because
/// that is what these families promise — the closing settle re-asserts each
/// of them after the last soft pass, so anything visible here survived its
/// own enforcement.
///
/// **First measurement (Montreux zone, 390,634 vars, 2026-08-27), the number
/// the pass order had never had:** `bore_ceiling`, `undercut` and `contact`
/// hold *exactly* — zero moves over the whole graph, which is what the
/// settle's "last word on a bore" / "contacts last" comments promise and
/// nothing verified. The sacrificed family is `grade`: 0.67 % of variables,
/// worst 32.9 m of distance-to-feasible — the settle converges grade first
/// and every later projection (a clearance lift, a chorded span) may spend
/// it, which is the ordering's designed trade, now priced. `clearance` 0.02 %
/// (worst 2.85 m — the plausibility-cap family), `deviation` 0.006 %,
/// `rigidity` 0.002 %, `monotone` 0.002 %.
const RESIDUAL_M: f64 = 1e-3;

pub fn check(m: &Model<'_>) -> Vec<Metric> {
    let families = &m.solved.residuals;
    let mut out = Vec::with_capacity(families.len());
    for f in families {
        // The invariant each family answers to: order-and-clearance families
        // to I3, everything shaping one surface's own continuity to I2.
        let invariant = match f.name {
            "clearance" | "undercut" | "bore_ceiling" => Invariant::I3,
            _ => Invariant::I2,
        };
        out.push(Metric {
            id: format!("solve.residual_{}", f.name),
            invariant,
            title: format!("Relaxation residual: {}", f.name),
            population: "Every solved variable of every stratum's constraint graph, measured \
                         after the closing settle: the distance one further application of this \
                         family's own pass would move it, with the heights restored between \
                         families so each is judged from the same point. The pass itself is the \
                         instrument — a projection fixes exactly its feasible points — so the \
                         number cannot drift from what the solver actually enforces."
                .into(),
            detail: "Zero means this constraint family holds at the output. A non-zero \
                     residual is a variable the pass order left outside the family's feasible \
                     set — the soft/limit-cycle interactions the closing settle exists to \
                     absorb — and it surfaces downstream as exactly the defect the family \
                     guards: a grade past its class ceiling, a bore roof through its ceiling, \
                     a junction contact broken, a monotone class reversing."
                .into(),
            sense: Sense::HigherIsWorse,
            threshold: RESIDUAL_M,
            skipped: (f.dist.count() == 0).then(|| "no solved variables in this extract".into()),
            dist: f.dist.clone(),
            worst: Vec::new(),
        });
    }
    out
}
