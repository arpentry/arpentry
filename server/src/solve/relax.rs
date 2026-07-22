//! The projection solver core (docs/CONSISTENCY.md §4.1, Phase C).
//!
//! A deterministic Jacobi projection loop over the [`SolveGraph`]. Continuity
//! needs no projection — it is the shared variable. Each sweep applies:
//!
//! - **Terrain adherence** (soft) — a spring pulling every ground-pinned
//!   variable toward its conditioned terrain target. *Soft*, not a hard clamp:
//!   where continuity or grade demand it, a variable lifts off the ground onto
//!   an embankment; the spring only keeps it there in the absence of a stronger
//!   pull (the H2 rung of the hierarchy).
//! - **Vertical smoothness** (soft) — each interior at-grade node pulled toward
//!   the arc-weighted chord of its neighbours (the comfort curvature, exactly
//!   `profile::smooth_vgrades`'s term).
//! - **Grade** (hard) — every edge held to its class ceiling, the violation
//!   split between the endpoints by inverse mass so the light (structure) side
//!   yields and the heavy (ground-pinned) side holds.
//! - **Structure rigidity** (hard) — each structure span's interior projected
//!   onto the straight chord through its two at-grade anchors (the deck ramp,
//!   reusing the anchors' *current* heights so the deck rides whatever the
//!   network settles to).
//! - **Deviation box** (hard, at-grade only) — each at-grade node clamped back
//!   inside its class ground-hugging budget of the conditioned terrain
//!   (docs/CONSISTENCY.md §2.1, the *boxed* deviation). Applied *after* grade,
//!   so on ground steeper than the class grade the box wins and the road breaks
//!   grade rather than dive metres below the hillside — a street trusts the
//!   slope (S9), an engineered road cuts only within its budget. Without it the
//!   hard grade held a Minor bed grade rigidly and dug corridors 40+ m into the
//!   Montreux slope.
//!
//! The terrain-adherence spring is a mass term, so the coupled system is a
//! screened Laplacian: a disturbance decays exponentially and the sweeps
//! converge quickly (docs/CONSISTENCY.md §2.1). Determinism (invariant 5):
//! strict Jacobi for the soft stage, fixed corridor/node order for the hard
//! stages, a fixed sweep budget.

use crate::priors::MAX_CLEARANCE_LIFT_M;

use super::graph::{CorridorNodes, GraphCrossing, SolveGraph, VarId, VarNode};
use super::profile::Profile;

/// Soft-spring weight pulling a pinned variable toward its terrain target.
const W_TERRAIN: f64 = 1.0;
/// Soft-spring weight pulling an interior node toward its neighbour chord.
const W_SMOOTH: f64 = 1.0;
/// Jacobi relaxation factor for the soft stage (under-relaxed for stability,
/// mirroring `profile::smooth_vgrades`'s `VGRADE_LAMBDA`).
const LAMBDA: f64 = 0.5;
/// Bounded grade Gauss–Seidel passes per sweep. Grade need not converge *within*
/// a sweep — the outer loop carries it — so a fixed handful keeps each sweep
/// O(edges) instead of O(edges × chain length). A steep pitch spreads this many
/// nodes per sweep; the final settle ([`GRADE_CAP`]) guarantees it holds at the
/// output.
const GRADE_INNER: usize = 12;
/// Cap on grade passes in the closing settle — enough to spread any real
/// corridor chain to convergence once, so grade holds at the output.
const GRADE_CAP: usize = 512;
/// Sweep budget. The screened-Laplacian locality (terrain mass) converges the
/// soft field in tens of sweeps; a clearance crest against the terrain spring
/// forms a standing embankment that never drives the residual to zero (a limit
/// cycle of the soft pull down and the hard lift back), so the cap — not the
/// residual — bounds the main loop, and the closing settle makes the output
/// feasible regardless.
const MAX_SWEEPS: usize = 96;
/// Convergence early-out for the crossing-free majority: once no variable moves
/// this far in a sweep, the (junction-only) reconciliation has settled.
const TOL_M: f64 = 1e-4;

/// Solves the graph in place: relaxes `g.h` to the constrained profile. Returns
/// the sweep count actually used (for diagnostics/tests).
pub fn solve(g: &mut SolveGraph) -> usize {
    let n = g.vars.len();
    if n == 0 {
        return 0;
    }
    let mut num = vec![0.0f64; n];
    let mut den = vec![0.0f64; n];
    let mut prev = vec![0.0f64; n];
    let mut used = MAX_SWEEPS;
    for sweep in 0..MAX_SWEEPS {
        prev.copy_from_slice(&g.h);
        soft_pass(g, &prev, &mut num, &mut den);
        for _ in 0..GRADE_INNER {
            if grade_pass(g) < TOL_M {
                break;
            }
        }
        deviation_pass(g);
        clearance_pass(g);
        rigidity_pass(g);
        let resid = g.h.iter().zip(&prev).map(|(&a, &b)| (a - b).abs()).fold(0.0, f64::max);
        if resid < TOL_M {
            used = sweep + 1;
            break;
        }
    }

    // Closing settle: lock the hard constraints so the output is feasible even
    // though the soft/clearance limit cycle kept the main loop from a zero
    // residual. Grade to convergence, then re-assert clearance and rigidity.
    for _ in 0..GRADE_CAP {
        if grade_pass(g) < TOL_M {
            break;
        }
    }
    deviation_pass(g);
    clearance_pass(g);
    rigidity_pass(g);
    used
}

/// One soft Jacobi step (terrain spring + smoothness), reading `prev`, writing
/// `g.h`. `num`/`den` are reused scratch (Σ weighted target, Σ weight).
fn soft_pass(g: &mut SolveGraph, prev: &[f64], num: &mut [f64], den: &mut [f64]) {
    num.iter_mut().for_each(|x| *x = 0.0);
    den.iter_mut().for_each(|x| *x = 0.0);
    for (v, vn) in g.vars.iter().enumerate() {
        if vn.terrain_pinned {
            num[v] += W_TERRAIN * vn.target_m;
            den[v] += W_TERRAIN;
        }
    }
    for c in &g.corridors {
        let m = c.vars.len();
        if m < 3 {
            continue;
        }
        for k in 1..m - 1 {
            if !c.at_grade[k] {
                continue; // structure interior → rigidity, not smoothness
            }
            let span = c.arc[k + 1] - c.arc[k - 1];
            if span <= 0.0 {
                continue;
            }
            let t = (c.arc[k] - c.arc[k - 1]) / span;
            let (a, b) = (c.vars[k - 1], c.vars[k + 1]);
            let chord = prev[a] + (prev[b] - prev[a]) * t;
            let v = c.vars[k];
            num[v] += W_SMOOTH * chord;
            den[v] += W_SMOOTH;
        }
    }
    for v in 0..g.h.len() {
        if den[v] > 0.0 {
            let target = num[v] / den[v];
            g.h[v] = prev[v] + LAMBDA * (target - prev[v]);
        }
    }
}

/// One forward+backward grade Gauss–Seidel pass over every corridor edge;
/// returns the worst correction applied (0 when grade already holds).
fn grade_pass(g: &mut SolveGraph) -> f64 {
    let mut worst = 0.0f64;
    for c in &g.corridors {
        for k in 0..c.vars.len().saturating_sub(1) {
            worst = worst.max(enforce_grade(&mut g.h, &g.vars, c, k));
        }
        for k in (0..c.vars.len().saturating_sub(1)).rev() {
            worst = worst.max(enforce_grade(&mut g.h, &g.vars, c, k));
        }
    }
    worst
}

/// Raise-only clearance over every crossing, in rank order: lift each deck to
/// clear its crossed feature (both span anchors, so the straight deck rises).
fn clearance_pass(g: &mut SolveGraph) {
    for gc in &g.crossings {
        let lower_h = gc.lower_var.map(|v| g.h[v]).unwrap_or(gc.lower_terrain_m);
        let need = lower_h + gc.extra_m;
        let targets = clearance_targets(&g.corridors[gc.upper_ci], &g.h, gc, need);
        for (v, d) in targets {
            g.h[v] += d;
        }
    }
}

/// Rigidity over every corridor: each structure span straight between its
/// anchors.
fn rigidity_pass(g: &mut SolveGraph) {
    for c in &g.corridors {
        project_spans(&mut g.h, c);
    }
}

/// Clamps every at-grade node back inside its class ground-hugging budget of
/// the conditioned terrain (the boxed deviation, docs/CONSISTENCY.md §2.1).
/// At-grade nodes only — a structure node floats on its deck ramp, bounded by
/// rigidity, not by the ground. Runs *after* grade so the box wins: where the
/// terrain is steeper than the class grade, the road holds within the budget
/// and breaks grade rather than trench the hillside. A shared connector reads
/// one variable and one conditioned target, so both corridors clamp it into the
/// same box — continuity (H0) is untouched.
fn deviation_pass(g: &mut SolveGraph) {
    for c in &g.corridors {
        for (k, &v) in c.vars.iter().enumerate() {
            if !c.at_grade[k] || !g.vars[v].terrain_pinned {
                continue;
            }
            let target = g.vars[v].target_m;
            g.h[v] = g.h[v].clamp(target - c.deviation, target + c.deviation);
        }
    }
}

/// Holds edge `k → k+1` of corridor `c` to its grade ceiling, splitting any
/// violation between the endpoints by inverse mass (the light side yields).
/// Returns the magnitude of the correction applied (0 when the edge already
/// satisfies the ceiling), so the caller can iterate to convergence.
fn enforce_grade(h: &mut [f64], vars: &[VarNode], c: &CorridorNodes, k: usize) -> f64 {
    let ds = c.arc[k + 1] - c.arc[k];
    if ds <= 0.0 {
        return 0.0;
    }
    let lim = c.grade * ds;
    let (a, b) = (c.vars[k], c.vars[k + 1]);
    let d = h[b] - h[a];
    let excess = d - d.clamp(-lim, lim);
    if excess == 0.0 {
        return 0.0;
    }
    let (ma, mb) = (vars[a].inv_mass, vars[b].inv_mass);
    let s = ma + mb;
    if s <= 0.0 {
        return 0.0;
    }
    h[a] += excess * ma / s;
    h[b] -= excess * mb / s;
    excess.abs()
}

/// The clearance raise for one crossing: how much (and which variables) to lift
/// so the upper corridor's deck clears `need` at the crossing arc. When the
/// crossing sits in a structure span, both bounding anchors rise by the deficit
/// (lifting the straight deck between them); otherwise the nearest node rises.
/// A deficit beyond [`MAX_CLEARANCE_LIFT_M`] is a data contradiction (a path
/// mapped across a viaduct high on a flank) and dropped — plain, not spectacle.
fn clearance_targets(c: &CorridorNodes, h: &[f64], gc: &GraphCrossing, need: f64) -> Vec<(VarId, f64)> {
    match structure_span_at(c, gc.upper_arc) {
        Some((lo, hi)) => {
            let span = c.arc[hi] - c.arc[lo];
            let deck = if span > 0.0 {
                let t = (gc.upper_arc - c.arc[lo]) / span;
                h[c.vars[lo]] + (h[c.vars[hi]] - h[c.vars[lo]]) * t
            } else {
                h[c.vars[lo]]
            };
            let deficit = need - deck;
            if deficit > 0.0 && deficit <= MAX_CLEARANCE_LIFT_M {
                vec![(c.vars[lo], deficit), (c.vars[hi], deficit)]
            } else {
                Vec::new()
            }
        }
        None => {
            let k = nearest_local(c, gc.upper_arc);
            let deficit = need - h[c.vars[k]];
            if deficit > 0.0 && deficit <= MAX_CLEARANCE_LIFT_M {
                vec![(c.vars[k], deficit)]
            } else {
                Vec::new()
            }
        }
    }
}

/// The bounding at-grade anchors (local node indices) of the structure span
/// containing `arc`, or `None` when `arc` is not inside a two-sided structure
/// span (at grade, or a one-sided span running off a corridor end).
fn structure_span_at(c: &CorridorNodes, arc: f64) -> Option<(usize, usize)> {
    let k = nearest_local(c, arc);
    if c.at_grade[k] {
        return None;
    }
    let mut lo = k;
    while lo > 0 && !c.at_grade[lo] {
        lo -= 1;
    }
    let mut hi = k;
    while hi + 1 < c.at_grade.len() && !c.at_grade[hi] {
        hi += 1;
    }
    // `lo`/`hi` now sit on the bounding at-grade anchors — unless the run
    // reaches a corridor end (no anchor that side).
    if c.at_grade[lo] && c.at_grade[hi] {
        Some((lo, hi))
    } else {
        None
    }
}

/// The local node index whose arc is nearest `arc`.
fn nearest_local(c: &CorridorNodes, arc: f64) -> usize {
    match c.arc.binary_search_by(|v| v.partial_cmp(&arc).expect("finite arc")) {
        Ok(i) => i,
        Err(i) => {
            if i == 0 {
                0
            } else if i >= c.arc.len() {
                c.arc.len() - 1
            } else if (arc - c.arc[i - 1]).abs() <= (c.arc[i] - arc).abs() {
                i - 1
            } else {
                i
            }
        }
    }
}

/// Projects every structure span's interior onto the straight chord through its
/// two bounding anchors (the deck ramp). An interior run is bounded by its
/// at-grade neighbours; a run reaching a corridor end is bounded by the terminal
/// node itself — a corridor endpoint is *always* an anchor, whether it is a
/// shared junction connector (its height the global relax already agreed) or a
/// free dead-end (its warm-start height). Chording to the endpoint is what
/// [`super::profile::deck_ramp`] already does when it fits the deck; leaving the
/// road on a stale warm start here is exactly what let the road dip beneath its
/// own straight deck and step off the abutment.
fn project_spans(h: &mut [f64], c: &CorridorNodes) {
    let m = c.at_grade.len();
    let mut k = 0;
    while k < m {
        if c.at_grade[k] {
            k += 1;
            continue;
        }
        let start = k;
        while k < m && !c.at_grade[k] {
            k += 1;
        }
        let end = k - 1; // inclusive last structure node
        // Bounding anchors: the at-grade neighbour on each side, or the corridor
        // endpoint where the run runs off that end.
        let lo = start.saturating_sub(1);
        let hi = if end + 1 < m { end + 1 } else { m - 1 };
        if hi <= lo {
            continue;
        }
        let (a_lo, a_hi) = (c.arc[lo], c.arc[hi]);
        let span = a_hi - a_lo;
        if span <= 0.0 {
            continue;
        }
        let (h_lo, h_hi) = (h[c.vars[lo]], h[c.vars[hi]]);
        // Project every node strictly between the anchors onto the chord (the
        // anchors themselves — endpoint or at-grade — hold their height).
        for j in (lo + 1)..hi {
            let t = (c.arc[j] - a_lo) / span;
            h[c.vars[j]] = h_lo + (h_hi - h_lo) * t;
        }
    }
}

/// Writes the solved heights back into each corridor's profile (its `road_m`,
/// then a refit deck), so every existing `Profile` reader sees the globally
/// consistent surface.
pub fn reconstruct(g: &SolveGraph, profiles: &mut [Option<Profile>]) {
    for c in &g.corridors {
        if let Some(p) = profiles.get_mut(c.id as usize).and_then(|p| p.as_mut()) {
            let road: Vec<f64> = c.vars.iter().map(|&v| g.h[v]).collect();
            p.set_road_m(&road);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::RoadClass;
    use crate::scene::{Corridor, Junction, JunctionMember, SceneGraph, SegmentRef, DEG_M};
    use geo_types::Coord;

    fn cos_lat() -> f64 {
        46.0_f64.to_radians().cos()
    }

    fn corridor(id: u32, x0: f64, len_m: f64, n: usize, class: RoadClass) -> Corridor {
        let deg = len_m / (DEG_M * cos_lat());
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: x0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        Corridor {
            id,
            nodes,
            arc,
            cos_lat: cos_lat(),
            class,
            class_key: String::new(),
            link: false,
            drivable: true,
            spans: vec![],
            segments: vec![SegmentRef { source: id as u64, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }
    }

    /// The maximum absolute grade along a reconstructed corridor.
    fn max_grade(p: &Profile) -> f64 {
        let (arc, road) = (p.arc(), p.road_m());
        (0..arc.len() - 1)
            .map(|k| {
                let ds = arc[k + 1] - arc[k];
                if ds > 0.0 {
                    (road[k + 1] - road[k]).abs() / ds
                } else {
                    0.0
                }
            })
            .fold(0.0, f64::max)
    }

    /// Two corridors that solved 6 m apart at a shared connector agree *exactly*
    /// there after the global solve — continuity by construction, no cap.
    #[test]
    fn a_shared_connector_agrees_exactly() {
        let len = 300.0;
        let n = 16;
        let a = corridor(0, 6.0, len, n, RoadClass::Minor);
        let deg = len / (DEG_M * cos_lat());
        let b = corridor(1, 6.0 + deg, len, n, RoadClass::Minor);
        let point = *a.nodes.last().unwrap();
        let scene = {
            let mut s = SceneGraph::new(vec![a, b]);
            s.junctions = vec![Junction {
                point,
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: len },
                    JunctionMember { corridor: 1, arc: 0.0 },
                ],
            }];
            s
        };
        let an = scene.corridors[0].nodes.clone();
        let bn = scene.corridors[1].nodes.clone();
        let mut profiles =
            vec![Some(Profile::flat(&an, 400.0)), Some(Profile::flat(&bn, 406.0))];
        let mut g = super::super::graph::build(&scene, &profiles);
        solve(&mut g);
        reconstruct(&g, &mut profiles);

        let a_end = profiles[0].as_ref().unwrap().road_at_arc(len);
        let b_start = profiles[1].as_ref().unwrap().road_at_arc(0.0);
        assert!((a_end - b_start).abs() < 1e-9, "connector must agree exactly: {a_end} vs {b_start}");
        // The far ends relax back toward their own terrain (400 / 406).
        let a_far = profiles[0].as_ref().unwrap().road_at_arc(0.0);
        let b_far = profiles[1].as_ref().unwrap().road_at_arc(len);
        assert!((a_far - 400.0).abs() < 1.0, "A far end near its terrain, got {a_far}");
        assert!((b_far - 406.0).abs() < 1.0, "B far end near its terrain, got {b_far}");
    }

    /// A corridor whose terrain steps like a cliff hugs the ground through the
    /// step rather than ramping it at grade: the deviation box wins over the
    /// grade ceiling (the established S9 contract, `road_hugs_the_ground_on_a_
    /// long_steep_climb`). Ramping the step at 15 % would carry the road up to
    /// ~15 m off the ground on the flat approaches — the embankment/trench the
    /// dropped deviation box used to produce.
    #[test]
    fn a_cliff_step_is_hugged_not_ramped() {
        use crate::priors::BED_MAX_DEVIATION_M;
        // Minor road: terrain flat 100, then a 30 m step over one 20 m node gap.
        let n = 21;
        let a = corridor(0, 6.0, 400.0, n, RoadClass::Minor);
        let arc: Vec<f64> = a.arc.clone();
        let terrain: Vec<f64> = arc.iter().map(|&s| if s < 200.0 { 100.0 } else { 130.0 }).collect();
        let scene = SceneGraph::new(vec![a]);
        let an = scene.corridors[0].nodes.clone();
        let mut profiles =
            vec![Some(Profile::from_heights(&an, terrain.clone(), terrain.clone()))];
        let mut g = super::super::graph::build(&scene, &profiles);
        solve(&mut g);
        reconstruct(&g, &mut profiles);
        let p = profiles[0].as_ref().unwrap();
        let solved = p.road_m();
        for (k, &t) in terrain.iter().enumerate() {
            assert!(
                (solved[k] - t).abs() <= BED_MAX_DEVIATION_M + 1e-6,
                "node {k} left the ground box at the cliff: road {} terrain {t}",
                solved[k]
            );
        }
    }

    /// A gentle corridor on plausible terrain is left on the ground.
    #[test]
    fn a_gentle_corridor_stays_on_terrain() {
        let n = 21;
        let a = corridor(0, 6.0, 400.0, n, RoadClass::Minor);
        let arc: Vec<f64> = a.arc.clone();
        // A 1 % slope — well under the 15 % minor ceiling.
        let terrain: Vec<f64> = arc.iter().map(|&s| 100.0 + 0.01 * s).collect();
        let scene = SceneGraph::new(vec![a]);
        let an = scene.corridors[0].nodes.clone();
        let mut profiles = vec![Some(Profile::from_heights(&an, terrain.clone(), terrain.clone()))];
        let mut g = super::super::graph::build(&scene, &profiles);
        let sweeps = solve(&mut g);
        reconstruct(&g, &mut profiles);
        let p = profiles[0].as_ref().unwrap();
        for (k, &t) in terrain.iter().enumerate() {
            assert!((p.road_m()[k] - t).abs() < 0.5, "node {k} drifted off terrain");
        }
        assert!(sweeps < MAX_SWEEPS, "a gentle corridor must converge, took {sweeps}");
    }

    /// A deck over a crossing is lifted to clear it (raise-only clearance). The
    /// graph is built by hand so the upper corridor carries a real structure
    /// span (nodes 2–4) over an at-grade feature at 100 m.
    #[test]
    fn a_deck_is_raised_to_clear_a_crossing() {
        use super::super::graph::{CorridorNodes, GraphCrossing, SolveGraph, VarNode};
        // 11 nodes, 50 m apart (arc 0..500); nodes 4,5,6 are the bridge span,
        // so the approaches (nodes 0–3, 7–10) are long enough to ramp the
        // 6.5 m lift up to grade.
        let n = 11;
        let at_grade: Vec<bool> =
            (0..n).map(|i| !(4..=6).contains(&i)).collect();
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * 50.0).collect();
        let vars: Vec<VarNode> = (0..n)
            .map(|i| VarNode {
                target_m: 100.0,
                terrain_m: 100.0,
                terrain_pinned: at_grade[i],
                inv_mass: if at_grade[i] { 1.0 } else { 8.0 },
            })
            .collect();
        let mut g = SolveGraph {
            vars,
            h: vec![100.0; n],
            corridors: vec![CorridorNodes {
                id: 0,
                vars: (0..n).collect(),
                arc,
                at_grade,
                grade: 0.06,
                deviation: 1e9, // not under test here — leave the ground box open
            }],
            // Clearance 5 (road) + 1.5 slab = 6.5 over the feature at 100 m.
            crossings: vec![GraphCrossing {
                upper_ci: 0,
                upper_arc: 250.0, // mid-span (node 5)
                lower_var: None,
                lower_terrain_m: 100.0,
                extra_m: 6.5,
            }],
            component: vec![0; n],
            n_components: 1,
        };
        solve(&mut g);
        // The deck at the crossing (node 5) must clear: ≥ 100 + 6.5.
        assert!(g.h[5] >= 106.5 - 1e-3, "deck must clear the crossing, got {}", g.h[5]);
        // The deck stays straight over the span (rigidity): nodes 4,5,6 colinear.
        let mid = 0.5 * (g.h[4] + g.h[6]);
        assert!((g.h[5] - mid).abs() < 1e-6, "deck must be straight over the span");
    }

    /// A structure span that runs to the corridor's terminal node — a ramp whose
    /// bridge lands on the elevated motorway it joins — is a *two-sided* span:
    /// the endpoint is an anchor the global relax already set (here a pinned
    /// high node standing in for that shared junction connector). Rigidity must
    /// straighten the interior onto the chord from the at-grade approach up to
    /// that endpoint, so the deck lands on the approach with no abutment step —
    /// the defect that left a bridge deck floating metres off its ramp.
    #[test]
    fn a_terminal_structure_span_chords_to_its_endpoint() {
        use super::super::graph::{CorridorNodes, SolveGraph, VarNode};
        // 11 nodes, 50 m apart. Nodes 0–4 are the at-grade approach on flat
        // 100 m ground; nodes 5–10 are the bridge span running to the end.
        // Node 10 is held at 112 (the network's junction height); the interior
        // is the anchor it must ramp up to.
        let n = 11;
        let at_grade: Vec<bool> = (0..n).map(|i| i <= 4).collect();
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * 50.0).collect();
        // Approach pinned to 100; the terminal node pinned to 112 (the held
        // junction height); the structure interior floats (light, unpinned).
        let pinned: Vec<bool> = (0..n).map(|i| i <= 4 || i == n - 1).collect();
        let target = |i: usize| if i == n - 1 { 112.0 } else { 100.0 };
        let vars: Vec<VarNode> = (0..n)
            .map(|i| VarNode {
                target_m: target(i),
                terrain_m: target(i),
                terrain_pinned: pinned[i],
                inv_mass: if pinned[i] { 1.0 } else { 8.0 },
            })
            .collect();
        // A within-grade zigzag start: pure grade + soft passes leave it be (it
        // violates no slope bound), so only the terminal-span rigidity fix makes
        // the output colinear — the discriminating initial condition.
        let h0 = vec![100.0, 100.0, 100.0, 100.0, 100.0, 103.0, 106.0, 103.0, 106.0, 109.0, 112.0];
        let mut g = SolveGraph {
            vars,
            h: h0,
            corridors: vec![CorridorNodes {
                id: 0,
                vars: (0..n).collect(),
                arc,
                at_grade,
                grade: 0.06, // 6 %: the 12 m rise over 300 m (4 %) is well within
                deviation: 1e9, // not under test here — leave the ground box open
            }],
            crossings: vec![],
            component: vec![0; n],
            n_components: 1,
        };
        solve(&mut g);
        // The structure interior lies on the chord from the approach anchor
        // (node 4 = 100) to the endpoint (node 10 = 112): a clean straight deck.
        let (h4, h10) = (g.h[4], g.h[10]);
        assert!((h10 - 112.0).abs() < 0.5, "endpoint held near 112, got {h10}");
        for k in 5..=9 {
            let t = (50.0 * k as f64 - 200.0) / 300.0;
            let want = h4 + (h10 - h4) * t;
            assert!(
                (g.h[k] - want).abs() < 1e-3,
                "node {k} must ride the deck chord: got {} want {want}",
                g.h[k]
            );
        }
        // And it lands on the approach with no step at the abutment (node 4→5).
        assert!(
            (g.h[5] - g.h[4]).abs() < 3.0,
            "the deck must land on the approach, not step off it: {} vs {}",
            g.h[5],
            g.h[4]
        );
    }

    /// A Minor street down a slope far steeper than its 15 % bed grade hugs the
    /// ground within its deviation budget and *breaks grade* — it does not hold
    /// the bed grade rigidly and dig a trench (the Montreux-hillside regression:
    /// a hard bed grade with no deviation box cut the corridor 40+ m below the
    /// terrain). The road trusts the slope (S9).
    #[test]
    fn a_steep_street_hugs_the_ground_and_breaks_grade() {
        use crate::priors::BED_MAX_DEVIATION_M;
        // 400 m of ~40 % slope (160 m drop) — a Minor bed grade is only 15 %.
        let n = 21;
        let a = corridor(0, 6.0, 400.0, n, RoadClass::Minor);
        let arc: Vec<f64> = a.arc.clone();
        let terrain: Vec<f64> = arc.iter().map(|&s| 500.0 - 0.40 * s).collect();
        let scene = SceneGraph::new(vec![a]);
        let an = scene.corridors[0].nodes.clone();
        let mut profiles = vec![Some(Profile::from_heights(&an, terrain.clone(), terrain.clone()))];
        let mut g = super::super::graph::build(&scene, &profiles);
        solve(&mut g);
        reconstruct(&g, &mut profiles);
        let p = profiles[0].as_ref().unwrap();
        let road = p.road_m();
        // Every node stays inside the ground-hugging box — no deep cut anywhere.
        for (k, &t) in terrain.iter().enumerate() {
            assert!(
                (road[k] - t).abs() <= BED_MAX_DEVIATION_M + 1e-6,
                "node {k} left the ground box: road {} terrain {t} (dev {})",
                road[k],
                (road[k] - t).abs()
            );
        }
        // And it genuinely breaks the bed grade to do so (the slope demands it).
        assert!(max_grade(p) > 0.15 + 1e-3, "a 40 % street must exceed the 15 % bed grade");
    }

    /// The solve is deterministic: two runs give identical heights.
    #[test]
    fn the_solve_is_deterministic() {
        let len = 300.0;
        let n = 16;
        let a = corridor(0, 6.0, len, n, RoadClass::Minor);
        let deg = len / (DEG_M * cos_lat());
        let b = corridor(1, 6.0 + deg, len, n, RoadClass::Minor);
        let point = *a.nodes.last().unwrap();
        let scene = {
            let mut s = SceneGraph::new(vec![a, b]);
            s.junctions = vec![Junction {
                point,
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: len },
                    JunctionMember { corridor: 1, arc: 0.0 },
                ],
            }];
            s
        };
        let an = scene.corridors[0].nodes.clone();
        let bn = scene.corridors[1].nodes.clone();
        let run = || {
            let profiles =
                vec![Some(Profile::flat(&an, 400.0)), Some(Profile::flat(&bn, 406.0))];
            let mut g = super::super::graph::build(&scene, &profiles);
            solve(&mut g);
            g.h
        };
        assert_eq!(run(), run(), "identical inputs must give identical heights");
    }
}
