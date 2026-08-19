//! The drawn railway's longitudinal grade, walked where the line still exists.
//!
//! `slope.rail_grade` used to walk the emitted rail stroke — until the union
//! learned to pave the formation, at which point the stroke became a second
//! coat over the ballast and `pipeline::paves_via_union` deletes it from
//! [`crate::priors::ROAD_SURFACE_MIN_ZOOM`], exactly as it does a
//! carriageway's. From that zoom the solve is the only place the alignment
//! still exists as a line, so the walk moved here.
//!
//! The heights are the same ones the stroke carried, by construction: the
//! stroke rode `road_m` at grade and the fitted deck ramp across structures,
//! and [`Profile::deck_m`](crate::solve::profile::Profile) *is* that composite
//! — `road_m` with each structure span replaced by its ramp, the at-grade
//! nodes untouched. At the reference zoom the mid-zoom datum shift is zero, so
//! this measures the z16 drawn heights every coarser rung derives from; what
//! it no longer sees is the stroke's own densification and lattice
//! quantization, which [`GRADE_RUN_M`] existed to filter out.

use crate::priors::Kind;
use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// How far a railway's drawn grade may exceed its own class ceiling before it
/// counts as a lost alignment rather than an earned bed.
///
/// Not zero, because the solve legitimately grants more than the table where
/// the ground earns it (`solve::profile::measured_grade` — the rack railway's
/// 11–22 % bed over a 7 % narrow-gauge row, the reason a flat ceiling was
/// unreadable). Set from the measured stroke population this walk replaced
/// (51,188 steps, Montreux z16): p50 −2.4 pp — the median rail runs under its
/// row — the earned rack band carries p95 to +14 pp and tops near +15, and
/// past 20 pp the count collapses (0.81 % over, then 0.11 % past 25 pp) into
/// the spike family: short runs at 40–70 pp over, which no bed anywhere earns.
const RAIL_EXCESS: f64 = 0.20;

/// Shortest arc step whose grade means anything, in metres.
///
/// Profile nodes are spaced at the solve's own resolution (~8 m), but a
/// corridor's terminal node can sit a densification residue from its
/// neighbour, and a height difference divided by centimetres of run reports a
/// ratio that is mostly noise.
const GRADE_RUN_M: f64 = 0.50;

pub fn check(m: &Model<'_>) -> Vec<Metric> {
    // Excess is signed: a funicular running at half its 70 % ceiling sits
    // deep in the negative band, and clamping it to zero would hide the
    // margin the median is there to show.
    let mut dist = Dist::new(-1.0, 8.0);
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);

    for c in &m.scene.corridors {
        if !matches!(c.kind, Kind::Rail(_)) {
            continue;
        }
        // A rail class the table holds no grade for has no ceiling to exceed:
        // street-running and `unknown` rail are draped by design (§4.6).
        let Some(ceiling) = c.kind.prior().grade() else { continue };
        let Some(p) = m.solved.profile(c.id) else { continue };
        let nodes = p.nodes();
        walk(p.arc(), p.deck_m(), ceiling, |i, excess, mid_arc| {
            // Only steps inside the extract: assemble admits whole parquet row
            // groups, so a corridor can run kilometres past the bbox into
            // ground no DEM constrained, and a grade measured there scores the
            // extraction boundary rather than the solve. Per step, not per
            // corridor — the same ownership rule the stroke walk applied per
            // tile — so a half-in corridor still reports its constrained half.
            let (mx, my) = (
                0.5 * (nodes[i - 1].x + nodes[i].x),
                0.5 * (nodes[i - 1].y + nodes[i].y),
            );
            if !m.bounds.contains(mx, my) {
                return;
            }
            dist.push(excess);
            if excess > RAIL_EXCESS {
                let pt = p.point_at_arc(mid_arc);
                worst.offer(Offender {
                    lon: pt.x,
                    lat: pt.y,
                    zoom: m.solved.z_ref,
                    value: excess,
                    note: format!(
                        "{} climbs at {:.0} % against its class ceiling of {:.0} % (solved to \
                         {:.0} %)",
                        c.class_key,
                        (excess + ceiling) * 100.0,
                        ceiling * 100.0,
                        p.max_grade().unwrap_or(ceiling) * 100.0
                    ),
                });
            }
        });
    }

    vec![Metric {
        id: "slope.rail_grade".into(),
        // The grade ceiling is I2's, not I6's: an alignment past its class
        // ceiling is a continuity defect, not a degradation one.
        invariant: Invariant::I2,
        title: "Drawn railway grade over its class ceiling".into(),
        population: format!(
            "Consecutive solved-profile nodes (the solve's own ~8 m spacing, runs of at least \
             {GRADE_RUN_M:.2} m) of every rail corridor whose class names a gauge or a system, \
             over the heights the drawn line carries — the road at grade, the fitted ramp across \
             structures — scored by grade in excess of the class's own §9 ceiling (signed: \
             negative is margin). Model-side: from z{} the union paves the formation and \
             deletes the stroke this walk used to ride, so the solve is the only place the \
             alignment still exists as a line. Steps are clipped to the extract's bbox — \
             assemble admits whole parquet row groups, so corridors spill past the zone into \
             ground no DEM constrained, and a grade there scores the extraction boundary. \
             Street-running and `unknown` rail are excluded: draped by design (§4.6), they \
             hold no ceiling to measure against, and counting them would score the class \
             table rather than the solve.",
            crate::priors::ROAD_SURFACE_MIN_ZOOM
        ),
        detail: format!(
            "Rise over run along a railway, against its own class row — mainline 3 %, narrow \
             gauge 7 %, funicular 70 % — which is what I2 asks for and one flat ceiling could \
             not express: measured that way the funicular and the rack line's earned bed WERE \
             the violation set, and a mainline drawn at nearly three times its table row sailed \
             under. The {:.0} pp allowance covers what `measured_grade` legitimately grants \
             over the table, and the offender note names the solved ceiling so an earned bed is \
             legible at the site; past the allowance the alignment is not one this class holds, \
             and since every road reads stratum R as a constant, an error here is upstream of \
             everything. At the reference zoom these are exactly the drawn heights (the datum \
             shift is zero at z_ref); mid-zoom strokes are emission-only and are not measured \
             here.",
            RAIL_EXCESS * 100.0
        ),
        sense: Sense::HigherIsWorse,
        threshold: RAIL_EXCESS,
        skipped: dist
            .is_empty()
            .then(|| "no rail corridor with a class grade ceiling carries a solved profile"
                .to_string()),
        dist,
        worst: worst.into_vec(),
    }]
}

/// The grade walk itself, pure so the tests need no [`Profile`]: every
/// consecutive node pair of `h` over `arc` at least [`GRADE_RUN_M`] apart,
/// yielding the step index, the signed excess over `ceiling`, and the step's
/// mid-arc. Plan clipping is the caller's: the walk knows arcs, not places.
fn walk(arc: &[f64], h: &[f64], ceiling: f64, mut f: impl FnMut(usize, f64, f64)) {
    for i in 1..arc.len().min(h.len()) {
        let run = arc[i] - arc[i - 1];
        if run < GRADE_RUN_M {
            continue;
        }
        f(i, (h[i] - h[i - 1]).abs() / run - ceiling, 0.5 * (arc[i - 1] + arc[i]));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn samples(arc: &[f64], h: &[f64], ceiling: f64) -> Vec<(f64, f64)> {
        let mut out = Vec::new();
        walk(arc, h, ceiling, |_, excess, mid| out.push((excess, mid)));
        out
    }

    /// A single-node spike is caught with the right sign and placed at the
    /// step's midpoint — the clearance-lift-on-one-node family the stroke
    /// walk existed for.
    #[test]
    fn a_spike_reports_its_excess_at_the_step() {
        let got = samples(&[0.0, 8.0, 16.0], &[100.0, 100.0, 104.0], 0.03);
        assert_eq!(got.len(), 2);
        assert!(got[0].0 < 0.0); // flat run: 3 pp of margin
        assert!((got[1].0 - (0.5 - 0.03)).abs() < 1e-9); // 50 % over a 3 % row
        assert_eq!(got[1].1, 12.0);
    }

    /// A funicular at half its ceiling sits deep in the negative band — the
    /// margin is the sample, not a clamped zero.
    #[test]
    fn margin_is_signed() {
        let got = samples(&[0.0, 10.0], &[0.0, 3.5], 0.70);
        assert_eq!(got.len(), 1);
        assert!((got[0].0 - (0.35 - 0.70)).abs() < 1e-9);
    }

    /// A sub-run step divides into noise and is skipped, not scored.
    #[test]
    fn a_short_run_is_not_a_grade() {
        let got = samples(&[0.0, 0.3, 8.3], &[0.0, 5.0, 5.0], 0.03);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].1, 0.5 * (0.3 + 8.3));
    }
}
