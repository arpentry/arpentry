//! The profile step: one height along every solving way.
//!
//! The solve itself is [`crate::solve`], because the crossing step re-runs
//! it over a floor and a step module imports no other step module. This
//! step is the first call of it, and the line that says what it came to.

use std::collections::HashMap;

use crate::grade;
use crate::solve::{curvature_radius, solve, Loose, GRADE_EPS};
use crate::step::Summary;
use crate::width::{self, Family};
use crate::world::{connector, Kind, Profiles, Reference, Solved};

/// Solves the profile of every solving way of the world (the carriageways
/// and the railways), and says what it could not anchor.
pub fn run(reference: &Reference) -> (Profiles, Summary) {
    let (profiles, Loose { dangling, unanchored, clamped, contacts }) = solve(reference);
    let summary = Summary::new()
        .with("contacts", contacts)
        .with("dangling", dangling)
        .with("clamped", clamped)
        .with("unanchored", unanchored);
    (Profiles { profiles }, summary)
}

/// What the solve came to: a function of the profiles alone.
///
/// Kept apart from [`run`] because it is the larger half by an order of
/// magnitude — a score of counters over every station of every profile,
/// against two lines that do the work — and reading `run` should say what the step
/// makes, not how it is scored. Nothing here decides anything, so a metric
/// can be added, moved or read without going near the solve, and asked of
/// profiles from anywhere: a dump, a hand-made specimen, the crossing's
/// re-solve.
pub fn check(solved: &Profiles) -> Summary {
    let profiles = &solved.profiles[..];
    let mut stations = 0usize;
    let (mut pairs, mut steep) = (0usize, 0usize);
    let (mut street_pairs, mut street_steep) = (0usize, 0usize);
    let (mut grounded, mut floating) = (0usize, 0usize);
    let (mut runs, mut kinked, mut tightest) = (0usize, 0usize, f64::INFINITY);
    let (mut eng_runs, mut eng_short) = (0usize, 0usize);
    let (mut deck, mut bridge) = (0usize, 0usize);
    let (mut bore, mut tunnel) = (0usize, 0usize);
    let (mut spans, mut degraded) = (0usize, 0usize);
    let mut ends: HashMap<(i64, i64), (f64, f64)> = HashMap::new();
    let mut raised = 0usize;
    // Every at-grade station, by connector: the lowest and highest height
    // any way puts there, which way got there first, whether a second did,
    // and whether a railway is one of them — `level` reads the spread
    // wherever a railway meets another way on the ground, which is the
    // claim the contacts make. A connector one of the two reaches on a
    // structure is a road passing under a rail bridge the mapper happened
    // to share a node with, and the heights there are meant to differ.
    let mut meet: HashMap<(i64, i64), Meet> = HashMap::new();
    for (n, p) in profiles.iter().enumerate() {
        let rail = width::family(&p.class) == Family::Rail;
        for (k0, k1, kind) in p.runs() {
            if kind.is_structure() {
                continue;
            }
            for st in &p.stations[k0..=k1] {
                let m = meet.entry(connector(st.p)).or_insert(Meet { lo: st.h, hi: st.h, first: n, shared: false, rail });
                m.lo = m.lo.min(st.h);
                m.hi = m.hi.max(st.h);
                m.shared |= m.first != n;
                m.rail |= rail;
            }
        }
    }
    for p in profiles {
        stations += p.stations.len();
        let g = grade::of(&p.class);
        // The ceiling this way was actually held to: a railway's measured
        // one, which the class number alone would call broken.
        let held = grade::held(&p.class, &p.stations, &p.spans);
        raised += (held > g.ceiling.unwrap_or(f64::INFINITY)) as usize;
        for st in [p.stations.first(), p.stations.last()].into_iter().flatten() {
            let e = ends.entry(connector(st.p)).or_insert((st.h, st.h));
            e.0 = e.0.min(st.h);
            e.1 = e.1.max(st.h);
        }
        // Per *run*: one way carries its at-grade stretches and its spans
        // together, and each is measured as what it is.
        for (k0, k1, kind) in p.runs() {
            if !kind.is_structure() {
                for pair in p.stations[k0..=k1].windows(2) {
                    let ds = pair[1].s - pair[0].s;
                    if ds > 0.0 {
                        let over = (pair[1].h - pair[0].h).abs() / ds > held + GRADE_EPS;
                        if g.limited() {
                            pairs += 1;
                            steep += over as usize;
                        } else {
                            street_pairs += 1;
                            street_steep += over as usize;
                        }
                    }
                }
                // The tightest vertical curve this run actually holds,
                // against the one its class allows: `bend`'s own guard.
                if k1 > k0 + 1 {
                    let arc: Vec<f64> = p.stations[k0..=k1].iter().map(|st| st.s).collect();
                    let h: Vec<f64> = p.stations[k0..=k1].iter().map(|st| st.h).collect();
                    let held = curvature_radius(&h, &arc);
                    tightest = tightest.min(held);
                    if let Some(want) = g.radius_m {
                        // Split by mode, because the two failures mean
                        // opposite things. A *street* that cannot hold its
                        // radius is a road the model left undrivable. An
                        // *engineered* one is the deviation box refusing to
                        // pay for the earthwork a real motorway gets: a
                        // motorway does hold a 4 km vertical curve, and on a
                        // mountainside eight metres of box does not buy it.
                        // Lowering the radius would only report fewer
                        // failures.
                        if g.limited() {
                            eng_runs += 1;
                            eng_short += (held < want - 1e-6) as usize;
                        } else {
                            runs += 1;
                            kinked += (held < want - 1e-6) as usize;
                        }
                    }
                }
                for st in &p.stations[k0..=k1] {
                    grounded += 1;
                    // `float` guards the limiter, so it is measured against
                    // what the limiter was aimed at: the reference, and the
                    // box is around that. How far the solved surface ends up
                    // standing from the *raw* DEM is the other question, and
                    // `dem_residual` answers it.
                    if (st.h - st.reference).abs() > g.deviation_m + 1e-9 {
                        floating += 1;
                    }
                }
                continue;
            }
            spans += 1;
            if p.stations[k0..=k1].iter().all(|st| st.solved == Solved::Grade) {
                degraded += 1;
            }
            for st in &p.stations[k0..=k1] {
                match kind {
                    Kind::Bridge(_) => {
                        bridge += 1;
                        deck += (st.solved == Solved::Deck) as usize;
                    }
                    Kind::Tunnel(_) => {
                        tunnel += 1;
                        bore += (st.solved == Solved::Bore) as usize;
                    }
                    _ => {}
                }
            }
        }
    }
    let step = ends.values().map(|(lo, hi)| hi - lo).fold(0.0f64, f64::max);
    let level = meet.values().filter(|m| m.shared && m.rail).map(|m| m.hi - m.lo).fold(0.0f64, f64::max);
    Summary::new()
        .with("ways", profiles.len())
        .with("stations", stations)
        .with_share("grade", steep, pairs)
        .with_share("steep", street_steep, street_pairs)
        .with_share("float", floating, grounded)
        .with_share("kink", kinked, runs)
        .with_share("boxed", eng_short, eng_runs)
        .with("bend", if tightest.is_finite() { format!("{tightest:.0}") } else { "-".into() })
        .with("raised", raised)
        .with("step", format!("{step:.3}"))
        .with("level", format!("{level:.3}"))
        .with_share("decks", deck, bridge)
        .with_share("bores", bore, tunnel)
        .with_share("degraded", degraded, spans)
        .with_residual(solved.residual())
}

/// One connector as the summary reads it: the heights the ways put there.
struct Meet {
    lo: f64,
    hi: f64,
    first: usize,
    shared: bool,
    rail: bool,
}
