//! The projection solver core (docs/GENERATION.md §4.4).
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
//! - **Clearance** (strong) — every crossing opened to its required separation,
//!   from both sides in inverse-mass proportion where both are this stratum's
//!   to move (§4.4), and raise-only in the closing settle so the invariant
//!   rests on the side that can always climb.
//! - **Structure rigidity** (hard) — each structure span's interior projected
//!   onto the straight chord through its two at-grade anchors (the deck ramp,
//!   reusing the anchors' *current* heights so the deck rides whatever the
//!   network settles to). A **bore** yields to a clearance ceiling on the way:
//!   a deck is a beam and its line is the constraint, a bore is a hole and an
//!   underpass runs below the chord of its own portals (S6).
//! - **Deviation box** (hard, at-grade only) — each at-grade node clamped back
//!   inside its class ground-hugging budget of the conditioned terrain
//!   (docs/GENERATION.md §4.4, the soft deviation budget). Applied *after* grade,
//!   so on ground steeper than the class grade the box wins and the road breaks
//!   grade rather than dive metres below the hillside — a street trusts the
//!   slope (S9), an engineered road cuts only within its budget. Without it the
//!   hard grade held a Minor bed grade rigidly and dug corridors 40+ m into the
//!   Montreux slope.
//!
//! The terrain-adherence spring is a mass term, so the coupled system is a
//! screened Laplacian: a disturbance decays exponentially and the sweeps
//! converge quickly (docs/GENERATION.md §4.4). Determinism (invariant 5):
//! strict Jacobi for the soft stage, fixed corridor/node order for the hard
//! stages, a fixed sweep budget.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

use crate::priors::MAX_CLEARANCE_LIFT_M;

use super::graph::{CorridorNodes, GraphCrossing, Lower, SolveGraph, VarId, VarNode};
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

/// What the relaxation could not honour, reported rather than discarded.
///
/// A clearance demand past [`MAX_CLEARANCE_LIFT_M`] is dropped as a data
/// contradiction. That is the right call — honouring one once flattened
/// kilometres of viaduct at the highest demand — but until now it happened in
/// silence, so a change that doubled the number of impossible demands looked
/// exactly like a change that fixed them.
#[derive(Debug, Clone, Copy, Default)]
pub struct Relaxed {
    /// Sweeps actually used (the budget is a cap, not a target).
    pub sweeps: usize,
    /// Clearance demands dropped for exceeding the plausibility cap, counted
    /// on the closing settle so each is counted once rather than once a sweep.
    pub demands_dropped: u64,
    /// The largest deficit so dropped, in metres — how far past plausible the
    /// worst contradiction reaches.
    pub worst_dropped_m: f64,
}

/// Solves the graph in place: relaxes `g.h` to the constrained profile.
pub fn solve(g: &mut SolveGraph) -> Relaxed {
    let n = g.vars.len();
    if n == 0 {
        return Relaxed::default();
    }
    let mut num = vec![0.0f64; n];
    let mut den = vec![0.0f64; n];
    let mut prev = vec![0.0f64; n];
    // Where each variable lives, for the clearance ramp to walk through
    // junctions. The graph does not change during the solve, so this is built
    // once.
    let sites = var_sites(g);
    // A bore lies under the ground: seed the ceiling before any sweep, so the
    // rigidity projection enforces it from the first chord onward.
    seed_bore_ceilings(g, &sites);
    let mut used = MAX_SWEEPS;
    let mut dropped = Dropped::default();
    for sweep in 0..MAX_SWEEPS {
        prev.copy_from_slice(&g.h);
        soft_pass(g, &prev, &mut num, &mut den);
        for _ in 0..GRADE_INNER {
            if grade_pass(g) < TOL_M {
                break;
            }
        }
        deviation_pass(g);
        contact_pass(g);
        clearance_pass(g, &sites, &mut Dropped::default(), true);
        undercut_pass(g, &sites);
        rigidity_pass(g);
        monotone_pass(g);
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
    // The closing settle is where the drops are counted: the main loop applies
    // the same demands every sweep, so counting there would multiply each
    // contradiction by the sweep count. It is also where the split ends: the
    // upper side covers whatever separation the lower one could not, so I3
    // holds at the output whatever the geometry allowed.
    clearance_pass(g, &sites, &mut dropped, false);
    undercut_pass(g, &sites);
    rigidity_pass(g);
    monotone_pass(g);
    // **Contacts last.** A shared connector with a senior stratum is an
    // equality, and the passes above are inequalities and projections that will
    // happily move the node that has to meet it — `rigidity_pass` chords a
    // structure span through its anchors, and a contact sitting on an anchor
    // gets chorded away with it. Applied after them all, the equality holds at
    // the output, which is what I2 asks of a junction.
    contact_pass(g);
    Relaxed { sweeps: used, demands_dropped: dropped.count, worst_dropped_m: dropped.worst_m }
}

/// Dips this stratum under every senior feature crossing above it (§4.1).
///
/// How far inside a mapped bore the roof reaches full cover, metres of arc.
/// At a portal edge the ceiling is the bare surface — the road must cross it
/// exactly there — and it deepens linearly to
/// [`TUNNEL_HEIGHT_M`](crate::priors::TUNNEL_HEIGHT_M) `+`
/// [`TUNNEL_COVER_M`](crate::priors::TUNNEL_COVER_M) of margin over this
/// distance, the dive an urban cut-and-cover makes in its approach ramp.
/// Short galleries whose every node sits within the transition are left
/// nearly free: near the surface is what they are.
const BORE_SURFACE_TRANSITION_M: f64 = 60.0;

/// A bore lies **under the ground** — the constraint whose absence let a
/// mapped tunnel fly. A structure span has almost no terrain-pinned anchors,
/// so nothing else relates its chord to the surface: the Territet–Glion
/// funicular's 169 m "tunnel" solved to a ramp 3–7 m *above* its hillside for
/// its entire length (the fragment's two ends were its only hard points, and
/// the smooth ramp between them happened to clear the flank), and the
/// Rochers-de-Naye line's bore roof stood 6.8 m proud at a thin-cover stretch.
///
/// The seed writes the surface (minus the roof-and-cover margin, relaxed to
/// zero at a portal edge) into `slack[v].1` — the same ceiling an undercut
/// establishes — and [`project_spans`] then holds every bore chord beneath it,
/// sweep by sweep. Two exclusions keep it honest:
///
/// - **A corridor end is sealed only when something actually continues it.**
///   Relief is granted where the mapped tunnel ends inside the corridor — an
///   at-grade portal or a bridge abutment, both of which the profile must
///   rise to meet — and equally at a corridor end that nothing continues: the
///   Territet–Glion funicular's data stops mid-"tunnel" (the lower 130 m of
///   the line is absent from the extract), and sealing that end forced full
///   cover from the first node — a 5.5 m plunge in one node spacing. Only an
///   end whose variable another corridor's *structure* node shares is a
///   genuine mid-bore seam, and only there does the cover hold through.
/// - **A variable shared with any at-grade node is not capped.** A junction
///   inside a bore belongs to the at-grade machinery (contact, deviation);
///   capping the shared height would drag the surface road under its own
///   terrain.
fn seed_bore_ceilings(g: &mut SolveGraph, sites: &VarSites) {
    let cover = crate::priors::TUNNEL_HEIGHT_M + crate::priors::TUNNEL_COVER_M;
    let SolveGraph { vars, corridors, slack, .. } = g;
    for (ci, c) in corridors.iter().enumerate() {
        let m = c.at_grade.len();
        // Whether the corridor's end at `node` is genuinely continued by
        // another corridor's structure — the one case a mid-tunnel corridor
        // end is a seam rather than a portal.
        let sealed = |node: usize| {
            sites[c.vars[node]]
                .iter()
                .any(|&(oc, ok)| oc as usize != ci && !corridors[oc as usize].at_grade[ok as usize])
        };
        let mut k = 0;
        while k < m {
            // The per-node mapped-tunnel flag, not the per-run `bore`: a
            // tunnel–bridge–tunnel sequence is one run, and the ceiling must
            // stop at the abutments — a deck holds its clearance lift.
            if !c.tunnel[k] {
                k += 1;
                continue;
            }
            let start = k;
            while k < m && c.tunnel[k] {
                k += 1;
            }
            let end = k - 1;
            // Relief on a side whose neighbour is anything but more tunnel: an
            // at-grade portal or a bridge abutment both sit at the surface the
            // profile must rise to meet — and an unsealed corridor end is a
            // portal too, not a seam.
            let lo = if start > 0 {
                Some(c.arc[start - 1])
            } else {
                (!sealed(0)).then(|| c.arc[0])
            };
            let hi = if end + 1 < m {
                Some(c.arc[end + 1])
            } else {
                (!sealed(m - 1)).then(|| c.arc[m - 1])
            };
            for j in start..=end {
                let v = c.vars[j];
                if sites[v]
                    .iter()
                    .any(|&(oc, ok)| corridors[oc as usize].at_grade[ok as usize])
                {
                    continue;
                }
                let d = [lo.map(|a| c.arc[j] - a), hi.map(|a| a - c.arc[j])]
                    .into_iter()
                    .flatten()
                    .fold(f64::INFINITY, f64::min);
                let depth = cover * (d / BORE_SURFACE_TRANSITION_M).min(1.0);
                slack[v].1 = slack[v].1.min(vars[v].terrain_m - depth);
            }
        }
    }
}

/// The exact mirror of [`clearance_pass`]: a lower-only *ceiling* spread
/// outward by the same shortest-path-in-height-budget walk, so the road falls
/// into a trough at the crossing and climbs back to its own profile at its
/// class grade. A ceiling rather than a decrement for the same reason the lift
/// is a floor — the pass runs once per sweep and the soft pull hands the dip
/// back each round, so only an idempotent bound converges.
///
/// This is the half §4.1's four-case table needs and the model never had. A
/// railway on an embankment could not be moved (senior) and the road under it
/// had no way to fall, so the road simply stayed at grade while the drawn
/// ground rose through it: measured on Montreux the moment rail became real,
/// `slope.carriageway_face` 6.6 → 9.0, asphalt folding where the formation
/// crossed it.
fn undercut_pass(g: &mut SolveGraph, sites: &VarSites) {
    for uc in &g.undercuts.clone() {
        let c = &g.corridors[uc.under_ci];
        let k = nearest_local(c, uc.under_arc);
        let excess = g.h[c.vars[k]] - uc.ceiling_m;
        // Beyond the plausible depth of a real underpass the level tags and
        // the solved geometry contradict each other; trust the profile.
        if excess <= 0.0 || excess > MAX_CLEARANCE_LIFT_M {
            continue;
        }
        for (v, d) in ramp_targets(g, sites, uc.under_ci, k, uc.ceiling_m, Sense::Down) {
            g.h[v] += d;
            g.slack[v].1 = g.slack[v].1.min(g.h[v]);
        }
    }
}

/// Which way a ramp bounds the road it spreads over.
#[derive(Clone, Copy, PartialEq)]
enum Sense {
    /// A floor: the road may not go below it (an approach to a deck).
    Up,
    /// A ceiling: the road may not go above it (a dip under a senior).
    Down,
}

/// Meets every senior height this stratum shares a connector with (§4.5).
///
/// A hard projection, applied *after* the deviation box so the box cannot break
/// the equality: where a junior road joins a road already solved, it joins it,
/// and the ground-hugging budget does not get a vote. One-sided by
/// construction — the senior's height is an `f64`, so there is no way to write
/// back to it even by accident.
fn contact_pass(g: &mut SolveGraph) {
    for c in &g.contacts {
        g.h[c.var] = c.height_m;
    }
}

/// Tally of demands the plausibility cap rejected in one pass.
#[derive(Default)]
struct Dropped {
    count: u64,
    worst_m: f64,
}

impl Dropped {
    fn note(&mut self, deficit_m: f64) {
        self.count += 1;
        self.worst_m = self.worst_m.max(deficit_m);
    }
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

/// Clearance over every crossing, in rank order.
///
/// The separation is opened from **both** sides where both are this stratum's
/// to move, in the proportion §4.4 states: *"a correction distributes by
/// inverse mass, so approaches bend to meet decks and decks hold their line"*.
/// At a flat-ground overpass the deck is light and the street beneath it is
/// pinned to the ground, so the deck climbs; at an urban underpass (S6) the
/// road in the bore is the light one, so it is the one that dips, and the
/// street above stays where the ground put it. Both readings come out of one
/// ratio, which is the point of stating authority as mass.
///
/// `share` is what makes this safe to iterate. In the main loop the deficit is
/// split; in the **closing settle** it is not, and the upper side covers all of
/// whatever separation is still missing. So the geometry decides how much of
/// the correction the lower side can actually absorb — a bore can only sink as
/// far as its approaches can be ramped down — while the invariant rests where
/// it always did, on the side that can always climb. Splitting *without* that
/// second half is what the rejected version did, and it turned a hard
/// constraint into a hope: clearance shortfall 51.93 → 293.61 m
/// (docs/VERIFICATION.md §6).
fn clearance_pass(g: &mut SolveGraph, sites: &VarSites, dropped: &mut Dropped, share: bool) {
    for i in 0..g.crossings.len() {
        // By value: the passes below write `g.h`, and a crossing is four
        // numbers.
        let gc = g.crossings[i];
        let gc = &gc;
        let lower_h = match gc.lower {
            Lower::Var(v) => g.h[v],
            Lower::Constant(h) => h,
        };
        // How much of the deficit the lower side is asked to absorb. A senior
        // lower side has no variable at all (`Lower::Constant`), so it can
        // never be asked — that is I7, and it costs nothing here to hold.
        let lower_share = match (share, gc.lower) {
            // Only where the lower side is in a **bore**. Mass alone would let
            // any peer yield downward, and measured on the extract that turns
            // every corridor already off its own datum into a pump: a rack
            // railway hundreds of metres below its terrain manufactures a
            // deficit, the dip spreads along it, `grade_pass` drags the
            // reference down with it and the deficit reopens — clearance
            // shortfall 58.94 → 289.76 m. Held to the run the data calls a
            // tunnel, the correction goes where §4.5's prior points and
            // nowhere else.
            (true, Lower::Var(v)) if in_bore(g, sites, v) => {
                let up = upper_inv_mass(g, gc);
                let low = g.vars[v].inv_mass;
                let s = up + low;
                if s > 0.0 {
                    low / s
                } else {
                    0.0
                }
            }
            _ => 0.0,
        };
        // The dip, applied first, so the lift below reads the surface the
        // lower side is being asked to reach rather than the one it is on.
        // Both bounds are absolute (a floor and a ceiling), so a sweep that
        // re-applies them changes nothing once they are met.
        let mut need = lower_h + gc.extra_m;
        if lower_share > 0.0 {
            if let Lower::Var(v) = gc.lower {
                let deficit = need - upper_height(g, gc);
                if deficit > 0.0 && deficit <= MAX_CLEARANCE_LIFT_M {
                    // The dip, bounded where the lift is bounded: no crossing
                    // drives a road implausibly under its own ground
                    // ([`MAX_CLEARANCE_LIFT_M`], mirrored). Bounding the
                    // *demand* rather than each node it reaches is what keeps
                    // the ramp smooth — clamping node by node against each
                    // one's own terrain cut a sawtooth into every dipped road,
                    // and `slope.rail_grade` read 303 %.
                    let floor = (g.vars[v].target_m - MAX_CLEARANCE_LIFT_M).min(g.h[v]);
                    let ceiling = (g.h[v] - deficit * lower_share).max(floor);
                    let (lci, lk) = sites[v][0];
                    // The demand is measured *against* the upper side, so the
                    // dip must not reach it. The ramp walks outward through
                    // junctions, and where the two corridors are connected —
                    // an interchange, a ramp joining the road it dives under —
                    // lowering the upper with the lower reopens the deficit
                    // that caused the dip. Measured, that ran a rack railway
                    // 290 m below its own terrain in 96 sweeps and turned a
                    // 58.94 m clearance shortfall into 292.43 m.
                    let blocked = upper_vars(g, gc);
                    for (w, d) in
                        ramp_targets(g, sites, lci as usize, lk as usize, ceiling, Sense::Down)
                    {
                        if blocked.contains(&w) {
                            continue;
                        }
                        g.h[w] += d;
                        g.slack[w].1 = g.slack[w].1.min(g.h[w]);
                    }
                    need = g.h[v] + gc.extra_m;
                }
            }
        }
        let targets = clearance_targets(g, sites, gc, need, dropped);
        for (v, d) in targets {
            g.h[v] += d;
            // Record what the lift established, so the deviation box yields to
            // it next sweep instead of undoing it.
            g.slack[v].0 = g.slack[v].0.max(g.h[v]);
        }
    }
}

/// The upper corridor's surface at the crossing — the deck chord where the
/// crossing sits inside one, the nearest node otherwise.
fn upper_height(g: &SolveGraph, gc: &GraphCrossing) -> f64 {
    let c = &g.corridors[gc.upper_ci];
    match structure_span_at(c, gc.upper_arc) {
        Some((lo, hi)) => {
            let span = c.arc[hi] - c.arc[lo];
            if span > 0.0 {
                let t = (gc.upper_arc - c.arc[lo]) / span;
                g.h[c.vars[lo]] + (g.h[c.vars[hi]] - g.h[c.vars[lo]]) * t
            } else {
                g.h[c.vars[lo]]
            }
        }
        None => g.h[c.vars[nearest_local(c, gc.upper_arc)]],
    }
}

/// Whether a variable sits inside a bore on any corridor carrying it.
fn in_bore(g: &SolveGraph, sites: &VarSites, v: VarId) -> bool {
    sites[v].iter().any(|&(ci, k)| g.corridors[ci as usize].bore[k as usize])
}

/// The variables the crossing reads the upper surface from — the ones a dip on
/// the lower side must leave alone, or it lowers the very thing it is measured
/// against.
fn upper_vars(g: &SolveGraph, gc: &GraphCrossing) -> [VarId; 3] {
    let c = &g.corridors[gc.upper_ci];
    let k = nearest_local(c, gc.upper_arc);
    match structure_span_at(c, gc.upper_arc) {
        Some((lo, hi)) => [c.vars[k], c.vars[lo], c.vars[hi]],
        None => [c.vars[k]; 3],
    }
}

/// How readily the upper side yields at the crossing: the inverse mass of the
/// surface *there*.
///
/// Read at the crossing node, never at a deck's anchors. The anchors are
/// at-grade and therefore heavy, so averaging them said a mapped viaduct was as
/// reluctant to move as the street beneath it, and split every annotated
/// overpass 50/50 — which is the annotation ignored. A structure node is light
/// because the data put a structure there (§4.5: the tag is a prior on the
/// constraint), and that is exactly the evidence the split needs: a bridge tag
/// on the upper side says the upper side is what departs the ground, a tunnel
/// tag on the lower says the lower does, and where neither is tagged the two
/// are equally pinned and share the correction.
fn upper_inv_mass(g: &SolveGraph, gc: &GraphCrossing) -> f64 {
    let c = &g.corridors[gc.upper_ci];
    g.vars[c.vars[nearest_local(c, gc.upper_arc)]].inv_mass
}

/// Every place a variable appears, as `(corridor index, local node index)`.
///
/// A junction connector is one variable on two or more corridors, and a
/// clearance ramp has to follow the road through it: the approach to an
/// overpass does not stop because the street it climbs was mapped as two ways.
/// Built once per solve — the walk below queries it per crossing per sweep.
type VarSites = Vec<Vec<(u32, u32)>>;

fn var_sites(g: &SolveGraph) -> VarSites {
    let mut sites: VarSites = vec![Vec::new(); g.vars.len()];
    for (ci, c) in g.corridors.iter().enumerate() {
        for (k, &v) in c.vars.iter().enumerate() {
            sites[v].push((ci as u32, k as u32));
        }
    }
    sites
}

/// Rigidity over every corridor: each structure span straight between its
/// anchors.
fn rigidity_pass(g: &mut SolveGraph) {
    for c in &g.corridors {
        project_spans(&mut g.h, &g.slack, c);
    }
}

/// Projects every monotone corridor's heights onto the nearest non-reversing
/// sequence (§9: one cable, one hill). Required-level, so it runs after the
/// bounds that could reintroduce a reversal — the bore ceiling diving at a
/// data-gap end was measured doing exactly that, a 5.5 m plunge in one node
/// spacing at the funicular's missing-data seam. The L2 projection
/// ([`super::profile::monotone_project`]) averages violators into the level
/// stretch a real line would hold, instead of propagating one bad height
/// downhill.
fn monotone_pass(g: &mut SolveGraph) {
    let SolveGraph { corridors, h, .. } = g;
    for c in corridors.iter() {
        let Some(dir) = c.monotone else { continue };
        let mut vals: Vec<f64> = c.vars.iter().map(|&v| h[v]).collect();
        super::profile::monotone_project(&mut vals, dir);
        for (&v, &val) in c.vars.iter().zip(&vals) {
            h[v] = val;
        }
    }
}

/// Clamps every at-grade node back inside its class ground-hugging budget of
/// the conditioned terrain (the boxed deviation, docs/GENERATION.md §4.4).
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
            // The box yields to a clearance bound rather than fighting it
            // (§4.4: the budget is Soft, clearance is Strong). Where a lift has
            // established a floor the box opens upward to meet it, and where a
            // dip has established a ceiling it opens downward — so an approach
            // can climb to its deck over the run its grade needs, instead of
            // being pulled back every sweep and arriving in one node.
            let (floor, ceiling) = g.slack[v];
            let lo = (target - c.deviation).min(ceiling);
            let hi = (target + c.deviation).max(floor);
            g.h[v] = g.h[v].clamp(lo, hi);
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
/// (lifting the straight deck between them); otherwise the crossing node rises
/// and its neighbours ride up a **ramp**.
///
/// The ramp is the whole of the at-grade case. Lifting the single nearest node
/// by the deficit is what a clearance demand naively is, and it draws a spike:
/// the pass runs after `grade_pass` and after `deviation_pass` and again in the
/// closing settle, so nothing downstream spreads it or clamps it back. Measured
/// at 6.9257,46.4261, a residential street crossing a railway with no mapped
/// bridge span took its whole 5.95 m of clearance on one node and climbed it
/// over 3.0 m of road — 198 %, drawn as a fan of tilted slabs, and 46 of the
/// extract's 116 nodes past 50 % stood within 30 m of a crossing.
///
/// So the deficit decays at the class grade ceiling, the same shape
/// [`crate::solve::Profile::raise_crest`] gives the unfused path — but along
/// the *network* rather than along the one corridor, because a junction
/// connector is a variable two corridors share. Ramping only the crossing's own
/// corridor lifts that shared node and leaves the street on the other side of
/// it where it was: measured, that put a 40 m step into a rack railway welded
/// to a road junction beside the crossing, a worse spectacle than the spike it
/// replaced. So the ramp walks outward through the junctions it reaches, each
/// corridor decaying it at its own ceiling.
///
/// A deficit beyond [`MAX_CLEARANCE_LIFT_M`] is a data contradiction (a path
/// mapped across a viaduct high on a flank) and dropped — plain, not spectacle.
fn clearance_targets(
    g: &SolveGraph,
    sites: &VarSites,
    gc: &GraphCrossing,
    need: f64,
    dropped: &mut Dropped,
) -> Vec<(VarId, f64)> {
    let c = &g.corridors[gc.upper_ci];
    let h = &g.h;
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
            if deficit > MAX_CLEARANCE_LIFT_M {
                dropped.note(deficit);
                return Vec::new();
            }
            if deficit > 0.0 {
                vec![(c.vars[lo], deficit), (c.vars[hi], deficit)]
            } else {
                Vec::new()
            }
        }
        None => {
            let k = nearest_local(c, gc.upper_arc);
            let deficit = need - h[c.vars[k]];
            if deficit > MAX_CLEARANCE_LIFT_M {
                dropped.note(deficit);
                return Vec::new();
            }
            if deficit <= 0.0 {
                return Vec::new();
            }
            ramp_targets(g, sites, gc.upper_ci, k, need, Sense::Up)
        }
    }
}

/// The approach ramp as a raise-only **floor**, spread outward from the
/// crossing node through the road network.
///
/// A *floor* rather than an increment because the pass runs once per sweep and
/// the soft pull undoes part of it each time: an added tent hands the crest its
/// whole deficit back every round while the approaches keep only what survived,
/// so the ramp steepens toward twice the class grade with the sweep count. A
/// floor is idempotent — once met, re-applying it changes nothing.
///
/// The spread is a shortest-path in *height budget*: crossing an edge costs
/// that corridor's grade ceiling times its length, and a node's floor is `need`
/// less the cheapest budget reaching it. The walk carries on only through nodes
/// the floor actually raises, so the ramp is exactly the run of road that needs
/// supporting and it stops where the road is already high enough — tens of
/// metres, not the network.
///
/// The extent is measured from `need`, never from the *current* deficit. Tying
/// it to the deficit shrinks the ramp as the crest rises, so each sweep floors
/// a shorter run than the last while the soft pull erodes the rest: the ramp
/// eats itself from the outside in and the approach ends up steeper than the
/// ceiling it was built to hold.
fn ramp_targets(
    g: &SolveGraph,
    sites: &VarSites,
    start_ci: usize,
    start_k: usize,
    need: f64,
    sense: Sense,
) -> Vec<(VarId, f64)> {
    // Budget in millimetres so the frontier orders exactly and the walk is a
    // function of the model, not of float comparison order (invariant 5).
    let mm = |m: f64| (m * 1000.0).round().max(0.0) as u64;
    let mut best: HashMap<(u32, u32), u64> = HashMap::new();
    let mut heap: BinaryHeap<Reverse<(u64, u32, u32)>> = BinaryHeap::new();
    heap.push(Reverse((0, start_ci as u32, start_k as u32)));
    best.insert((start_ci as u32, start_k as u32), 0);

    let mut lift: HashMap<VarId, f64> = HashMap::new();
    while let Some(Reverse((cost, ci, k))) = heap.pop() {
        if best.get(&(ci, k)).copied().unwrap_or(u64::MAX) < cost {
            continue; // a cheaper route reached this node already
        }
        let c = &g.corridors[ci as usize];
        let v = c.vars[k as usize];
        // A floor rises away from the crossing as the budget is spent; a
        // ceiling falls away from it. One walk, one sign.
        let bound = match sense {
            Sense::Up => need - cost as f64 * 0.001,
            Sense::Down => need + cost as f64 * 0.001,
        };
        let raised = match sense {
            Sense::Up => bound > g.h[v],
            Sense::Down => bound < g.h[v],
        };
        if raised {
            // A variable reached twice keeps the larger lift: the cheapest
            // route is the one that governs, and it is the one that arrived
            // first, but a shared node is visited once per corridor it is on.
            let e = lift.entry(v).or_insert(0.0);
            *e = match sense {
                Sense::Up => e.max(bound - g.h[v]),
                Sense::Down => e.min(bound - g.h[v]),
            };
        } else if cost > 0 {
            continue; // the road is already above the ramp here: it ends
        }
        // Onward: the two neighbours along this corridor, and every other
        // corridor this variable belongs to (the junction the ramp runs
        // through).
        let step = |ci: u32, k2: i64, cost: u64, heap: &mut BinaryHeap<_>, best: &mut HashMap<_, _>| {
            let c = &g.corridors[ci as usize];
            if k2 < 0 || k2 as usize >= c.vars.len() {
                return;
            }
            let k2 = k2 as u32;
            if best.get(&(ci, k2)).copied().unwrap_or(u64::MAX) <= cost {
                return;
            }
            best.insert((ci, k2), cost);
            heap.push(Reverse((cost, ci, k2)));
        };
        for k2 in [k as i64 - 1, k as i64 + 1] {
            if k2 < 0 || k2 as usize >= c.arc.len() {
                continue;
            }
            let ds = (c.arc[k2 as usize] - c.arc[k as usize]).abs();
            step(ci, k2, cost + mm(c.grade * ds), &mut heap, &mut best);
        }
        for &(oci, ok) in &sites[v] {
            if oci != ci {
                step(oci, ok as i64, cost, &mut heap, &mut best);
            }
        }
    }
    lift.into_iter().collect()
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
fn project_spans(h: &mut [f64], slack: &[(f64, f64)], c: &CorridorNodes) {
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
        //
        // A **bore** yields to a clearance ceiling on the way. A deck is a beam
        // and its line is the constraint; a bore is a hole, and an urban
        // underpass (S6) is exactly a road that runs below the chord of its own
        // portals. Chording it back up is what undid every attempt to make the
        // lower side of a crossing yield (docs/VERIFICATION.md §6) — but only
        // *where a ceiling was established*: with no crossing over it a bore is
        // the mountain tunnel it always was, straight between its portals, and
        // this changes nothing about it.
        let bore = c.bore[start];
        for j in (lo + 1)..hi {
            let t = (c.arc[j] - a_lo) / span;
            let chord = h_lo + (h_hi - h_lo) * t;
            let v = c.vars[j];
            h[v] = if bore { chord.min(slack[v].1) } else { chord };
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

/// The solved height of every junction, by junction index; `None` where no
/// member carries a profile.
///
/// [`reconstruct`] scatters these same values into each corridor's `road_m` and
/// the graph is then dropped, so without this the one height a junction's
/// members agree on survives only as several equal numbers that a consumer has
/// to re-derive. The surface mesh needs it as a *pin* — one height per
/// intersection, read directly — so it is carried on the [`SolvedModel`].
pub fn junction_heights(g: &SolveGraph) -> Vec<Option<f64>> {
    g.junction_var.iter().map(|v| v.map(|v| g.h[v])).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::Stratum;
    use crate::priors::{Kind, RoadClass};
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
            kind: Kind::Road(class),
            class_key: String::new(),
            link: false,
            width_m: Some(5.5),
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
        let a = corridor(0, 6.0, len, n, RoadClass::Residential);
        let deg = len / (DEG_M * cos_lat());
        let b = corridor(1, 6.0 + deg, len, n, RoadClass::Residential);
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
        let mut g = super::super::graph::build(&scene, &profiles, &[], Stratum::S);
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
        let a = corridor(0, 6.0, 400.0, n, RoadClass::Residential);
        let arc: Vec<f64> = a.arc.clone();
        let terrain: Vec<f64> = arc.iter().map(|&s| if s < 200.0 { 100.0 } else { 130.0 }).collect();
        let scene = SceneGraph::new(vec![a]);
        let an = scene.corridors[0].nodes.clone();
        let mut profiles =
            vec![Some(Profile::from_heights(&an, terrain.clone(), terrain.clone()))];
        let mut g = super::super::graph::build(&scene, &profiles, &[], Stratum::S);
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

    /// A mapped tunnel whose chord rides above the surface is pulled beneath
    /// it. The pathological data case this pins: a structure span has almost
    /// no terrain-pinned anchors, so nothing but [`seed_bore_ceilings`]
    /// relates its chord to the ground — the Territet–Glion funicular's 169 m
    /// "tunnel" solved to a ramp 3–7 m above its hillside end to end.
    #[test]
    fn a_mapped_tunnel_above_the_surface_is_pulled_beneath_it() {
        use crate::priors::{TUNNEL_COVER_M, TUNNEL_HEIGHT_M};
        use crate::scene::{Span, SpanKind};
        // Flat anchors at 108 m; the mapped tunnel's middle crosses ground at
        // 100 m. The chord between the anchors runs level at 108 — eight
        // metres *above* the ground the annotation says it passes under.
        let mut a = corridor(0, 6.0, 600.0, 61, RoadClass::Motorway);
        a.spans = vec![
            Span { arc0: 0.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 200.0, arc1: 400.0, level: -1, kind: SpanKind::Tunnel },
            Span { arc0: 400.0, arc1: 600.0, level: 0, kind: SpanKind::Grade },
        ];
        let spans = a.spans.clone();
        let scene = SceneGraph::new(vec![a]);
        let nodes = scene.corridors[0].nodes.clone();
        let elev = |c: Coord| {
            let arc = (c.x - 6.0) * DEG_M * cos_lat();
            if (210.0..390.0).contains(&arc) {
                100.0
            } else {
                108.0
            }
        };
        let warm = super::super::profile::solve(
            &nodes,
            &spans,
            super::super::profile::Mode::Engineered { grade: 0.06 },
            &mut |c| elev(c),
        )
        .expect("a profile");
        let mut profiles = vec![Some(warm)];
        let mut g = super::super::graph::build(&scene, &profiles, &[], Stratum::S);
        solve(&mut g);
        reconstruct(&g, &mut profiles);

        let p = profiles[0].as_ref().unwrap();
        let cover = TUNNEL_HEIGHT_M + TUNNEL_COVER_M;
        // Deep inside the bore (a transition past either portal) the road is
        // under the ground by the full roof-and-cover margin.
        for arc in [290.0, 300.0, 310.0] {
            let road = p.road_at_arc(arc);
            assert!(
                road <= 100.0 - cover + 1e-6,
                "bore interior at arc {arc} rides at {road}, above {}",
                100.0 - cover
            );
        }
        // The at-grade thirds stay on their own ground: the ceiling is the
        // bore's, not the corridor's.
        for arc in [50.0, 550.0] {
            let road = p.road_at_arc(arc);
            assert!((road - 108.0).abs() < 1.5, "at-grade at arc {arc} moved to {road}");
        }
    }

    /// The Territet–Glion case: the extract's funicular ends mid-"tunnel" (the
    /// lower 130 m of the line is absent from the data), and the sealed-end
    /// bore ceiling manufactured a 5.5 m plunge in one node spacing — a dip no
    /// cable railway can have. Two rules refute it together: monotone (§9, one
    /// cable, one hill) and portal relief at a corridor end nothing continues.
    #[test]
    fn a_funicular_never_reverses_even_where_its_data_stops_mid_tunnel() {
        use crate::priors::RailClass;
        use crate::scene::{Span, SpanKind};
        let mut a = corridor(0, 6.0, 240.0, 25, RoadClass::Motorway);
        a.kind = Kind::Rail(RailClass::Funicular);
        a.spans = vec![
            Span { arc0: 0.0, arc1: 170.0, level: -1, kind: SpanKind::Tunnel },
            Span { arc0: 170.0, arc1: 240.0, level: 0, kind: SpanKind::Grade },
        ];
        let spans = a.spans.clone();
        let scene = SceneGraph::new(vec![a]);
        let nodes = scene.corridors[0].nodes.clone();
        // A 48 % hillside — the funicular's bed, which is its alignment.
        let elev = |c: Coord| 400.0 + 0.48 * (c.x - 6.0) * DEG_M * cos_lat();
        let warm = super::super::profile::solve(
            &nodes,
            &spans,
            super::super::profile::Mode::Engineered { grade: 0.70 },
            &mut |c| elev(c),
        )
        .expect("a profile");
        let mut profiles = vec![Some(warm)];
        let mut g = super::super::graph::build(&scene, &profiles, &[], Stratum::R);
        solve(&mut g);
        reconstruct(&g, &mut profiles);
        let p = profiles[0].as_ref().unwrap();
        let road = p.road_m();
        for (k, w) in road.windows(2).enumerate() {
            assert!(
                w[1] >= w[0] - 1e-6,
                "the funicular reverses at node {k}: {} -> {}",
                w[0],
                w[1]
            );
        }
        // The open end is a portal, not a dive: the first node holds its bed.
        assert!(
            (road[0] - 400.0).abs() < 3.0,
            "the data-gap end left its bed: {} vs 400",
            road[0]
        );
    }

    /// A gentle corridor on plausible terrain is left on the ground.
    #[test]
    fn a_gentle_corridor_stays_on_terrain() {
        let n = 21;
        let a = corridor(0, 6.0, 400.0, n, RoadClass::Residential);
        let arc: Vec<f64> = a.arc.clone();
        // A 1 % slope — well under the 15 % minor ceiling.
        let terrain: Vec<f64> = arc.iter().map(|&s| 100.0 + 0.01 * s).collect();
        let scene = SceneGraph::new(vec![a]);
        let an = scene.corridors[0].nodes.clone();
        let mut profiles = vec![Some(Profile::from_heights(&an, terrain.clone(), terrain.clone()))];
        let mut g = super::super::graph::build(&scene, &profiles, &[], Stratum::S);
        let sweeps = solve(&mut g);
        reconstruct(&g, &mut profiles);
        let p = profiles[0].as_ref().unwrap();
        for (k, &t) in terrain.iter().enumerate() {
            assert!((p.road_m()[k] - t).abs() < 0.5, "node {k} drifted off terrain");
        }
        assert!(sweeps.sweeps < MAX_SWEEPS, "a gentle corridor must converge, took {} sweeps", sweeps.sweeps);
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
                bore: vec![false; n],
                tunnel: vec![false; n],
                monotone: None,
                at_grade,
                grade: 0.06,
                deviation: 1e9, // not under test here — leave the ground box open
            }],
            // Clearance 5 (road) + 1.5 slab = 6.5 over the feature at 100 m.
            crossings: vec![GraphCrossing {
                upper_ci: 0,
                upper_arc: 250.0, // mid-span (node 5)
                lower: Lower::Constant(100.0),
                extra_m: 6.5,
            }],
            contacts: Vec::new(),
            undercuts: Vec::new(),
            slack: vec![(f64::NEG_INFINITY, f64::INFINITY); n],
            component: vec![0; n],
            n_components: 1,
            junction_var: Vec::new(), // no scene junctions in this fixture
        };
        solve(&mut g);
        // The deck at the crossing (node 5) must clear: ≥ 100 + 6.5.
        assert!(g.h[5] >= 106.5 - 1e-3, "deck must clear the crossing, got {}", g.h[5]);
        // The deck stays straight over the span (rigidity): nodes 4,5,6 colinear.
        let mid = 0.5 * (g.h[4] + g.h[6]);
        assert!((g.h[5] - mid).abs() < 1e-6, "deck must be straight over the span");
    }

    /// A crossing on a corridor with **no structure span** — the mapped level
    /// says bridge but the span table says at grade, which is most of the
    /// unannotated network — still clears, and reaches its clearance on a ramp
    /// rather than on one node. Taking the whole deficit at the crossing node
    /// drew a spike: measured on Montreux, a residential street over a railway
    /// climbed 5.95 m in 3.0 m of road and rendered as a fan of tilted slabs.
    #[test]
    fn an_at_grade_crossing_clears_on_a_ramp_not_a_spike() {
        use super::super::graph::{CorridorNodes, SolveGraph, VarNode};
        // 21 nodes, 10 m apart, flat ground at 100 m, all at grade.
        let n = 21;
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * 10.0).collect();
        let vars: Vec<VarNode> = (0..n)
            .map(|_| VarNode {
                target_m: 100.0,
                terrain_m: 100.0,
                terrain_pinned: true,
                inv_mass: 1.0,
            })
            .collect();
        let mut g = SolveGraph {
            vars,
            h: vec![100.0; n],
            corridors: vec![CorridorNodes {
                id: 0,
                vars: (0..n).collect(),
                arc,
                bore: vec![false; n],
                tunnel: vec![false; n],
                monotone: None,
                at_grade: vec![true; n],
                grade: 0.15,
                deviation: 1e9, // the ground box is not what is under test
            }],
            // A railway at grade under node 10, wanting 6 m of clearance.
            crossings: vec![GraphCrossing {
                upper_ci: 0,
                upper_arc: 100.0,
                lower: Lower::Constant(100.0),
                extra_m: 6.0,
            }],
            contacts: Vec::new(),
            undercuts: Vec::new(),
            slack: vec![(f64::NEG_INFINITY, f64::INFINITY); n],
            component: vec![0; n],
            n_components: 1,
            junction_var: Vec::new(),
        };
        solve(&mut g);

        assert!(g.h[10] >= 106.0 - 1e-3, "the crossing must still clear, got {}", g.h[10]);
        // And the climb to it is a road's climb. Without the ramp the step from
        // node 9 to node 10 was the whole 6 m over 10 m of road: 60 %.
        let worst = (1..n)
            .map(|k| (g.h[k] - g.h[k - 1]).abs() / 10.0)
            .fold(0.0f64, f64::max);
        assert!(worst <= 0.15 + 1e-6, "the approach climbs at {:.0} %", worst * 100.0);
        // The lift is local: a corridor end 100 m away is untouched.
        assert!((g.h[0] - 100.0).abs() < 1e-6, "end lifted to {}", g.h[0]);
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
                bore: vec![false; n],
                tunnel: vec![false; n],
                monotone: None,
                at_grade,
                grade: 0.06, // 6 %: the 12 m rise over 300 m (4 %) is well within
                deviation: 1e9, // not under test here — leave the ground box open
            }],
            crossings: vec![],
            contacts: Vec::new(),
            undercuts: Vec::new(),
            slack: vec![(f64::NEG_INFINITY, f64::INFINITY); n],
            component: vec![0; n],
            n_components: 1,
            junction_var: Vec::new(), // no scene junctions in this fixture
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
        let a = corridor(0, 6.0, 400.0, n, RoadClass::Residential);
        let arc: Vec<f64> = a.arc.clone();
        let terrain: Vec<f64> = arc.iter().map(|&s| 500.0 - 0.40 * s).collect();
        let scene = SceneGraph::new(vec![a]);
        let an = scene.corridors[0].nodes.clone();
        let mut profiles = vec![Some(Profile::from_heights(&an, terrain.clone(), terrain.clone()))];
        let mut g = super::super::graph::build(&scene, &profiles, &[], Stratum::S);
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

    /// **S6, the urban underpass.** Two streets crossing on flat ground, the
    /// lower one annotated as a tunnel. The separation must open *downward*:
    /// the road in the bore is the light side, so it takes most of the
    /// correction and the street above stays near the ground the terrain put
    /// it on. Raise-only, this built a hump over the underpass instead — which
    /// is why 10 % of annotated tunnel nodes ended at or above the ground.
    ///
    /// The bore must also be free to *dip*. Chorded onto the straight line
    /// through its portals it cannot be under them, and a flat-ground
    /// underpass is nothing but that.
    #[test]
    fn a_flat_ground_underpass_dips_rather_than_humping_the_street_above() {
        use super::super::graph::{CorridorNodes, GraphCrossing, SolveGraph, VarNode};
        // Two 200 m corridors of 21 nodes, crossing at their midpoints on flat
        // 100 m ground. Corridor 1 (vars 21..42) is in a bore over nodes 8-12.
        let n = 21;
        let arc: Vec<f64> = (0..n).map(|i| i as f64 * 10.0).collect();
        let bore: Vec<bool> = (0..n).map(|i| (8..=12).contains(&i)).collect();
        let at_grade_low: Vec<bool> = bore.iter().map(|b| !b).collect();
        let mut vars: Vec<VarNode> = Vec::new();
        for _ in 0..n {
            vars.push(VarNode {
                target_m: 100.0,
                terrain_m: 100.0,
                terrain_pinned: true,
                inv_mass: 1.0,
            });
        }
        for i in 0..n {
            vars.push(VarNode {
                target_m: 100.0,
                terrain_m: 100.0,
                terrain_pinned: at_grade_low[i],
                inv_mass: if at_grade_low[i] { 1.0 } else { 8.0 },
            });
        }
        let corridor = |base: usize, at_grade: Vec<bool>, bore: Vec<bool>| CorridorNodes {
            id: (base / n) as u32,
            vars: (base..base + n).collect(),
            arc: arc.clone(),
            at_grade,
            tunnel: bore.clone(),
            monotone: None,
            bore,
            grade: 0.06,
            deviation: 1e9, // the ground box is not what is under test
        };
        let mut g = SolveGraph {
            vars,
            h: vec![100.0; 2 * n],
            corridors: vec![
                corridor(0, vec![true; n], vec![false; n]),
                corridor(n, at_grade_low, bore),
            ],
            crossings: vec![GraphCrossing {
                upper_ci: 0,
                upper_arc: 100.0,
                lower: Lower::Var(n + 10),
                extra_m: 6.5,
            }],
            contacts: Vec::new(),
            undercuts: Vec::new(),
            slack: vec![(f64::NEG_INFINITY, f64::INFINITY); 2 * n],
            component: vec![0; 2 * n],
            n_components: 2,
            junction_var: Vec::new(),
        };
        solve(&mut g);

        let (up, low) = (g.h[10], g.h[n + 10]);
        assert!(up - low >= 6.5 - 1e-3, "the crossing must clear: {up} over {low}");
        // And the correction was spent on the side that yields (§4.4): the
        // bore dips further than the street climbs.
        assert!(
            100.0 - low > up - 100.0,
            "the bore must take most of it: street +{:.2} m, bore {:.2} m",
            up - 100.0,
            low - 100.0
        );
        // The street above is left near the ground, not humped over it.
        assert!(up - 100.0 < 2.0, "the street above climbed {:.2} m", up - 100.0);
        // The bore's approaches ramp down at the class grade, not in one step.
        let worst = (1..n)
            .map(|k| (g.h[n + k] - g.h[n + k - 1]).abs() / 10.0)
            .fold(0.0f64, f64::max);
        assert!(worst <= 0.06 + 1e-6, "the approach falls at {:.0} %", worst * 100.0);
    }

    /// The solve is deterministic: two runs give identical heights.
    #[test]
    fn the_solve_is_deterministic() {
        let len = 300.0;
        let n = 16;
        let a = corridor(0, 6.0, len, n, RoadClass::Residential);
        let deg = len / (DEG_M * cos_lat());
        let b = corridor(1, 6.0 + deg, len, n, RoadClass::Residential);
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
            let mut g = super::super::graph::build(&scene, &profiles, &[], Stratum::S);
            solve(&mut g);
            g.h
        };
        assert_eq!(run(), run(), "identical inputs must give identical heights");
    }
}
