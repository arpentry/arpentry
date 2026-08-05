//! The global vertical constraint graph (docs/GENERATION.md §4.4).
//!
//! The per-corridor profiles give geometry (densified nodes, arc, conditioned
//! terrain, at-grade flags, a warm-start height); this module fuses them into
//! **one** variable graph in which a junction connector is a **single shared
//! height variable** across every corridor that meets there. Continuity
//! (invariant 2) then stops being a constraint to enforce and becomes a
//! property of the degree-of-freedom layout: two roads meeting at a connector
//! read the same number because there is only one.
//!
//! The graph is the input to the projection solver ([`super::relax`]): its
//! `vars` are the unknowns, its per-corridor node lists carry the edges (grade
//! + smoothness) and the structure spans (rigidity), and its connected
//! components solve independently (deterministic order for invariant 5).

use geo_types::Coord;

use crate::priors::CLEARANCE_TROUGH_M;
use crate::scene::{CorridorId, SceneGraph, Span, SpanKind, DEG_M};

use super::profile::{condition_reference, Profile};

/// Index into [`SolveGraph::vars`].
pub type VarId = usize;

/// How much a node yields to corrections — the inverse mass in the projection.
/// An at-grade node is pinned near the ground, so it is *heavy* (resists
/// moving); a structure node floats on its deck ramp, so it is *light*. A
/// correction therefore flows into the yielding side: an approach bends to
/// meet a deck, the deck holds its line (docs/GENERATION.md §4.4).
const AT_GRADE_INV_MASS: f64 = 1.0;
const STRUCTURE_INV_MASS: f64 = 8.0;

/// One height variable: its soft terrain target, the raw terrain, whether it is
/// pinned to the ground, and its inverse mass. The mutable height lives in
/// [`SolveGraph::h`] so the solver can Jacobi-snapshot it.
#[derive(Debug, Clone, Copy)]
pub struct VarNode {
    /// Conditioned terrain target the soft spring pulls a ground-pinned node
    /// toward.
    pub target_m: f64,
    /// Raw terrain height at the variable.
    pub terrain_m: f64,
    /// Whether *every* incident node sits at grade — the variable is then pinned
    /// to the ground (a terrain spring, heavy mass). A connector shared with a
    /// structure end (an abutment, a portal) is **not** pinned: its height is
    /// set by the deck/bore it meets, not the ground, so the terrain spring must
    /// not drag a flyover down to the grass.
    pub terrain_pinned: bool,
    /// Inverse mass: [`AT_GRADE_INV_MASS`] (pinned) or [`STRUCTURE_INV_MASS`].
    pub inv_mass: f64,
}

/// One corridor's nodes, mapped into the shared variable space. The solver
/// walks these: consecutive `vars` are edges (grade + smoothness), and maximal
/// runs of `!at_grade` are structure spans bounded by at-grade anchors.
#[derive(Debug, Clone)]
pub struct CorridorNodes {
    pub id: CorridorId,
    /// Global variable of each local node (`vars[k]` is node `k`'s variable).
    pub vars: Vec<VarId>,
    /// Cumulative arc metres at each node (from the profile).
    pub arc: Vec<f64>,
    /// At-grade flag per node (from the profile).
    pub at_grade: Vec<bool>,
    /// The grade ceiling this corridor's edges are held to.
    pub grade: f64,
    /// How far an at-grade node may leave its conditioned terrain reference,
    /// metres — the ground-hugging box ([`crate::priors::RoadClass::deviation_m`]).
    /// The hard bound the relax clamps at-grade nodes back inside: an engineered
    /// road cuts within its budget, a street trusts the slope within a couple
    /// metres and *breaks grade* rather than dive metres below the ground. The
    /// relax's [`grade`](Self::grade) alone has no such cap — held hard, a street's
    /// bed grade (never a solver ceiling) dug a corridor tens of metres into a
    /// steep hillside. This box, applied after grade, is what stops it.
    pub deviation: f64,
}

/// One crossing as the solver sees it: the upper corridor's deck must clear the
/// lower surface by [`extra_m`](Self::extra_m). Sorted into rank order at build
/// time so a lower structure is finalised before the deck above it reads it.
#[derive(Debug, Clone, Copy)]
pub struct GraphCrossing {
    /// Index into [`SolveGraph::corridors`] of the upper (passing-over) corridor.
    pub upper_ci: usize,
    /// The upper corridor's arc where the crossing sits.
    pub upper_arc: f64,
    /// The lower feature's height variable, when it is a profiled corridor;
    /// `None` for an at-grade feature (its height is the terrain).
    pub lower_var: Option<VarId>,
    /// The lower feature's terrain height (used when `lower_var` is `None`).
    pub lower_terrain_m: f64,
    /// Clearance under the deck plus the deck slab — added to the lower surface
    /// to get the required deck top.
    pub extra_m: f64,
}

/// The fused constraint graph.
pub struct SolveGraph {
    pub vars: Vec<VarNode>,
    /// Current heights, initialised to the warm-start (mean of incident
    /// corridors' solved road heights). Jacobi-snapshotted by the solver.
    pub h: Vec<f64>,
    /// One entry per corridor that carries a profile.
    pub corridors: Vec<CorridorNodes>,
    /// Clearance constraints (invariant 3), in ascending rank order.
    pub crossings: Vec<GraphCrossing>,
    /// Connected-component id per variable (`0..n_components`).
    pub component: Vec<usize>,
    pub n_components: usize,
    /// The variable each of `scene.junctions` shares, by junction index; `None`
    /// where no member carries a profile, so the intersection has no solved
    /// height at all. Reading `h` here is the junction's height — the number the
    /// members hold in common by construction, rather than one recovered from
    /// their profiles afterwards.
    pub junction_var: Vec<Option<VarId>>,
}

/// A union–find over `n` slots (path-compression + union-by-size).
struct UnionFind {
    parent: Vec<usize>,
    size: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> UnionFind {
        UnionFind { parent: (0..n).collect(), size: vec![1; n] }
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }

    fn union(&mut self, a: usize, b: usize) {
        let (mut ra, mut rb) = (self.find(a), self.find(b));
        if ra == rb {
            return;
        }
        if self.size[ra] < self.size[rb] {
            std::mem::swap(&mut ra, &mut rb);
        }
        self.parent[rb] = ra;
        self.size[ra] += self.size[rb];
    }
}

/// Builds the constraint graph from the scene and the per-corridor initial
/// profiles (indexed by [`CorridorId`]; `None` where a corridor has no
/// profile). Junction members sharing a connector are unified into one
/// variable; consecutive nodes and structure spans become the solver's edges
/// and rigidity groups.
pub fn build(scene: &SceneGraph, profiles: &[Option<Profile>]) -> SolveGraph {
    // Global slot = a flat index over every node of every profiled corridor.
    // `slot_base[corridor_id]` is where that corridor's nodes start; `None`
    // (unprofiled) corridors get no slots.
    let mut slot_base: Vec<Option<usize>> = vec![None; profiles.len()];
    let mut corridor_order: Vec<usize> = Vec::new(); // corridor ids, in graph order
    let mut total_slots = 0usize;
    for (id, p) in profiles.iter().enumerate() {
        if let Some(p) = p {
            let n = p.nodes().len();
            if n >= 2 {
                slot_base[id] = Some(total_slots);
                total_slots += n;
                corridor_order.push(id);
            }
        }
    }

    // DOF sharing: union each junction's members' nearest nodes into one slot.
    // The anchor slot is kept per junction so the solved height at that shared
    // variable can be read back out (`junction_var`) instead of being recovered
    // afterwards from the members' scattered `road_m`, which is how everything
    // downstream used to guess at it.
    let mut uf = UnionFind::new(total_slots);
    let mut junction_slot: Vec<Option<usize>> = vec![None; scene.junctions.len()];
    for (ji, j) in scene.junctions.iter().enumerate() {
        let mut anchor: Option<usize> = None;
        for m in &j.members {
            let cid = m.corridor as usize;
            let (Some(base), Some(p)) = (slot_base.get(cid).copied().flatten(), profiles.get(cid).and_then(|p| p.as_ref()))
            else {
                continue;
            };
            let k = nearest_node(p.arc(), m.arc);
            let slot = base + k;
            match anchor {
                None => anchor = Some(slot),
                Some(a) => uf.union(a, slot),
            }
        }
        junction_slot[ji] = anchor;
    }

    // S8 entity resolution: a non-drivable structure (a footbridge) running
    // parallel and laterally close to a drivable bridge is the same physical
    // structure the source split into two independently `bridge`-tagged ways.
    // Bind its deck nodes to the road deck's nodes — one shared height
    // variable, exactly as a junction shares a connector — so the two decks
    // ride one grade line instead of overlapping at two heights (S8).
    for (a, b) in parallel_structure_unions(scene, profiles, &slot_base) {
        uf.union(a, b);
    }

    // Compact union-find roots into dense VarIds.
    let mut root_var: Vec<Option<VarId>> = vec![None; total_slots];
    let mut vars: Vec<VarNode> = Vec::new();
    // Accumulators over the slots mapping to each var (for averaging).
    let mut acc_target: Vec<f64> = Vec::new();
    let mut acc_terrain: Vec<f64> = Vec::new();
    let mut acc_init: Vec<f64> = Vec::new();
    let mut acc_count: Vec<u32> = Vec::new();
    let mut acc_all_at_grade: Vec<bool> = Vec::new();

    // Per-corridor node→var maps and metadata, plus warm-start heights.
    let mut corridors: Vec<CorridorNodes> = Vec::with_capacity(corridor_order.len());
    for &cid in &corridor_order {
        let p = profiles[cid].as_ref().expect("profiled");
        let base = slot_base[cid].expect("based");
        let arc = p.arc().to_vec();
        let at_grade = p.at_grade().to_vec();
        let terrain = p.terrain_m();
        let target = condition_reference(&arc, terrain);
        let road = p.road_m();
        let n = arc.len();
        let mut node_vars = Vec::with_capacity(n);
        for k in 0..n {
            let root = uf.find(base + k);
            let var = match root_var[root] {
                Some(v) => v,
                None => {
                    let v = vars.len();
                    root_var[root] = Some(v);
                    vars.push(VarNode {
                        target_m: 0.0,
                        terrain_m: 0.0,
                        terrain_pinned: false,
                        inv_mass: AT_GRADE_INV_MASS,
                    });
                    acc_target.push(0.0);
                    acc_terrain.push(0.0);
                    acc_init.push(0.0);
                    acc_count.push(0);
                    acc_all_at_grade.push(true);
                    v
                }
            };
            acc_target[var] += target[k];
            acc_terrain[var] += terrain[k];
            acc_init[var] += road[k];
            acc_count[var] += 1;
            acc_all_at_grade[var] &= at_grade[k];
            node_vars.push(var);
        }
        let c = &scene.corridors[cid];
        let grade = corridor_grade(c);
        let deviation = c.class.deviation_m();
        corridors.push(CorridorNodes {
            id: cid as CorridorId,
            vars: node_vars,
            arc,
            at_grade,
            grade,
            deviation,
        });
    }

    // Finalise per-var data (means; at_grade OR; mass from at_grade).
    let mut h = vec![0.0; vars.len()];
    for v in 0..vars.len() {
        let cnt = acc_count[v].max(1) as f64;
        let terrain_pinned = acc_all_at_grade[v];
        vars[v] = VarNode {
            target_m: acc_target[v] / cnt,
            terrain_m: acc_terrain[v] / cnt,
            terrain_pinned,
            inv_mass: if terrain_pinned { AT_GRADE_INV_MASS } else { STRUCTURE_INV_MASS },
        };
        h[v] = acc_init[v] / cnt;
    }

    // Connected components over the variables: union consecutive nodes of each
    // corridor (shared connectors already merged into single vars link
    // corridors together).
    let mut cuf = UnionFind::new(vars.len());
    for c in &corridors {
        for w in c.vars.windows(2) {
            cuf.union(w[0], w[1]);
        }
    }
    let mut comp_id: Vec<Option<usize>> = vec![None; vars.len()];
    let mut n_components = 0usize;
    let mut component = vec![0usize; vars.len()];
    for v in 0..vars.len() {
        let r = cuf.find(v);
        let id = match comp_id[r] {
            Some(id) => id,
            None => {
                let id = n_components;
                n_components += 1;
                comp_id[r] = Some(id);
                id
            }
        };
        component[v] = id;
    }

    // Corridor id → graph corridor index, for resolving crossings.
    let mut ci_of: Vec<Option<usize>> = vec![None; profiles.len()];
    for (ci, c) in corridors.iter().enumerate() {
        ci_of[c.id as usize] = Some(ci);
    }
    let crossings = build_crossings(scene, profiles, &corridors, &ci_of);

    // Resolve each junction's anchor slot to the variable its members ended up
    // sharing. Going through the same `root_var` the node maps went through is
    // what makes the recorded variable *the* one the members share — a second
    // `nearest_node` lookup could disagree with the first.
    let junction_var: Vec<Option<VarId>> =
        junction_slot.into_iter().map(|s| s.and_then(|slot| root_var[uf.find(slot)])).collect();

    SolveGraph { vars, h, corridors, crossings, component, n_components, junction_var }
}

/// Resolves the scene's crossings into solver form: the upper corridor's arc,
/// the lower feature's height source (a variable or the terrain), and the
/// required clearance-plus-slab. Sorted into ascending rank order so a stacked
/// interchange resolves bottom-up.
fn build_crossings(
    scene: &SceneGraph,
    profiles: &[Option<Profile>],
    corridors: &[CorridorNodes],
    ci_of: &[Option<usize>],
) -> Vec<GraphCrossing> {
    let ranks = super::crossings::corridor_ranks(scene);
    let mut out: Vec<(u32, GraphCrossing)> = Vec::new();
    for c in &scene.crossings {
        let Some(upper_ci) = ci_of.get(c.upper as usize).copied().flatten() else {
            continue;
        };
        let Some(up) = profiles.get(c.upper as usize).and_then(|p| p.as_ref()) else {
            continue;
        };
        // The lower surface: a profiled corridor's nearest node (tracked live),
        // else the trough the unprofiled feature runs in.
        let (lower_var, lower_terrain_m) = match c.lower.and_then(|id| {
            let lci = ci_of.get(id as usize).copied().flatten()?;
            let lp = profiles.get(id as usize).and_then(|p| p.as_ref())?;
            let k = nearest_node(&corridors[lci].arc, lp.arc_of(c.point.x, c.point.y));
            Some(corridors[lci].vars[k])
        }) {
            Some(v) => (Some(v), 0.0),
            None => (None, trough_terrain_m(up, up.arc_of(c.point.x, c.point.y))),
        };
        let extra_m = crate::priors::clearance_m(c.lower_kind) + crate::priors::DECK_THICKNESS_M;
        out.push((
            ranks.get(c.upper as usize).copied().unwrap_or(0),
            GraphCrossing {
                upper_ci,
                upper_arc: up.arc_of(c.point.x, c.point.y),
                lower_var,
                lower_terrain_m,
                extra_m,
            },
        ));
    }
    out.sort_by_key(|(rank, _)| *rank);
    out.into_iter().map(|(_, gc)| gc).collect()
}

/// The surface an *unprofiled* crossed feature is taken to lie on: the lowest
/// raw terrain the upper profile reads within [`CLEARANCE_TROUGH_M`] of the
/// crossing.
///
/// The fallback means "the crossed road is at grade, so it lies on the ground"
/// — but *which* ground. Read exactly at the plan intersection it lands on
/// whatever the upper corridor's own nodes sample there, and beside an
/// abutment that is the trench wall, metres above the road running through the
/// underpass. Demanding clearance over the wall lifted a flat motorway onto a
/// 5 m hump over its own underpass. A road crossing beneath runs along the
/// trough that was cut for it, so the trough floor is the honest read; on open
/// ground the window is flat and the minimum is the ground itself.
fn trough_terrain_m(p: &Profile, arc0: f64) -> f64 {
    let (arc, terrain) = (p.arc(), p.terrain_m());
    let lo = arc.partition_point(|&a| a < arc0 - CLEARANCE_TROUGH_M);
    let hi = arc.partition_point(|&a| a <= arc0 + CLEARANCE_TROUGH_M);
    terrain[lo..hi.max(lo + 1).min(terrain.len())]
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min)
        .min(p.surface_at_arc(arc0))
}

/// One corridor's bridge deck as the co-elevation pass sees it: its global
/// node slots and plan positions, in arc order (a bridge span is a contiguous
/// node run, so consecutive entries are deck segments).
struct Deck {
    cos_lat: f64,
    /// `(global slot, plan position)` for each node inside a bridge span.
    nodes: Vec<(usize, Coord)>,
}

/// Node-slot pairs to unify so parallel structures share a grade line (S8):
/// each non-drivable bridge node bound to the nearest node of the drivable
/// bridge deck it runs alongside. Slots are global (into the union-find),
/// `slot_base[corridor] + local_node`. A non-drivable deck is bound to the
/// drivable deck that covers the most of it within
/// [`crate::priors::PARALLEL_STRUCTURE_LATERAL_M`]; a footbridge with no
/// drivable neighbour (a genuine standalone span) is left untouched.
fn parallel_structure_unions(
    scene: &SceneGraph,
    profiles: &[Option<Profile>],
    slot_base: &[Option<usize>],
) -> Vec<(usize, usize)> {
    let mut drivable: Vec<Deck> = Vec::new();
    let mut footways: Vec<Deck> = Vec::new();
    for (cid, p) in profiles.iter().enumerate() {
        let (Some(p), Some(base)) = (p.as_ref(), slot_base.get(cid).copied().flatten()) else {
            continue;
        };
        let c = &scene.corridors[cid];
        if !c.spans.iter().any(|s| s.kind == SpanKind::Bridge) {
            continue;
        }
        let arc = p.arc();
        let pts = p.nodes();
        let nodes: Vec<(usize, Coord)> = (0..pts.len())
            .filter(|&k| in_bridge_span(&c.spans, arc[k]))
            .map(|k| (base + k, pts[k]))
            .collect();
        if nodes.len() < 2 {
            continue;
        }
        let deck = Deck { cos_lat: c.cos_lat, nodes };
        if c.drivable {
            drivable.push(deck);
        } else {
            footways.push(deck);
        }
    }

    let mut out = Vec::new();
    for f in &footways {
        // The best drivable partner: the one covering the most of the footway
        // deck within the lateral gap, ties broken by the smaller mean offset.
        let mut best: Option<(usize, f64, Vec<(usize, usize)>)> = None;
        for d in &drivable {
            let mut pairs = Vec::new();
            let mut sum = 0.0;
            for &(fslot, fp) in &f.nodes {
                if let Some((dslot, dist)) = nearest_deck_node(d, fp, f.cos_lat) {
                    if dist <= crate::priors::PARALLEL_STRUCTURE_LATERAL_M {
                        pairs.push((fslot, dslot));
                        sum += dist;
                    }
                }
            }
            // Parallel, not crossing: most of the footway deck must lie within
            // the gap (a perpendicular footbridge shares only a node or two).
            if pairs.len() >= 2 && pairs.len() * 2 >= f.nodes.len() {
                let (cover, mean) = (pairs.len(), sum / pairs.len() as f64);
                let better = best
                    .as_ref()
                    .is_none_or(|(bc, bm, _)| cover > *bc || (cover == *bc && mean < *bm));
                if better {
                    best = Some((cover, mean, pairs));
                }
            }
        }
        if let Some((_, _, pairs)) = best {
            out.extend(pairs);
        }
    }
    out
}

/// Whether arc `a` falls inside one of the corridor's bridge spans. The bounds
/// carry a centimetre tolerance: the profile's arc is re-accumulated from node
/// geometry, so a deck's end node lands a float-epsilon past the span's nominal
/// `arc1` and must still count as on the deck (grade slivers are ≥
/// [`crate::priors::SNAP_RUN_M`], far beyond this).
fn in_bridge_span(spans: &[Span], a: f64) -> bool {
    const EPS: f64 = 1e-2;
    spans.iter().any(|s| s.kind == SpanKind::Bridge && a >= s.arc0 - EPS && a <= s.arc1 + EPS)
}

/// The deck node nearest plan point `p` (perpendicular distance to the deck
/// polyline, in metres), and its global slot — the closer endpoint of the
/// nearest segment. `cos_lat` scales longitude into the local metric space.
fn nearest_deck_node(d: &Deck, p: Coord, cos_lat: f64) -> Option<(usize, f64)> {
    let mut best: Option<(usize, f64)> = None;
    for w in d.nodes.windows(2) {
        let (s0, a) = w[0];
        let (s1, b) = w[1];
        let (abx, aby) = ((b.x - a.x) * cos_lat * DEG_M, (b.y - a.y) * DEG_M);
        let (apx, apy) = ((p.x - a.x) * cos_lat * DEG_M, (p.y - a.y) * DEG_M);
        let ab2 = abx * abx + aby * aby;
        let t = if ab2 > 0.0 { ((apx * abx + apy * aby) / ab2).clamp(0.0, 1.0) } else { 0.0 };
        let (dx, dy) = (apx - abx * t, apy - aby * t);
        let dist = (dx * dx + dy * dy).sqrt();
        let slot = if t < 0.5 { s0 } else { s1 };
        if best.is_none_or(|(_, bd)| dist < bd) {
            best = Some((slot, dist));
        }
    }
    best
}

/// The grade ceiling a corridor's edges are held to: a ramp climbs at the ramp
/// grade whatever its class; an engineered class holds its ceiling; a street
/// holds its (looser) bed grade.
fn corridor_grade(c: &crate::scene::Corridor) -> f64 {
    if c.link {
        crate::priors::RAMP_GRADE
    } else {
        c.class.grade_limit().unwrap_or_else(|| c.class.bed_grade())
    }
}

/// The local node index whose arc is nearest `a` (binary search on the sorted
/// arc array, then the closer of the two bracketing nodes).
fn nearest_node(arc: &[f64], a: f64) -> usize {
    match arc.binary_search_by(|v| v.partial_cmp(&a).expect("finite arc")) {
        Ok(i) => i,
        Err(i) => {
            if i == 0 {
                0
            } else if i >= arc.len() {
                arc.len() - 1
            } else if (a - arc[i - 1]).abs() <= (arc[i] - a).abs() {
                i - 1
            } else {
                i
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::RoadClass;
    use crate::scene::{Corridor, Junction, JunctionMember, SegmentRef, DEG_M};
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
            width_m: Some(5.5),
            spans: vec![],
            segments: vec![SegmentRef { source: id as u64, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }
    }

    /// Two corridors meeting end-to-start at a connector share ONE variable
    /// there — the whole point of the graph.
    #[test]
    fn a_connector_becomes_one_shared_variable() {
        let len = 200.0;
        let n = 11;
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
        let profiles =
            vec![Some(Profile::flat(&an, 400.0)), Some(Profile::flat(&bn, 402.0))];
        let g = build(&scene, &profiles);

        // Corridor 0's last node and corridor 1's first node are the SAME var.
        let a_end = *g.corridors[0].vars.last().unwrap();
        let b_start = g.corridors[1].vars[0];
        assert_eq!(a_end, b_start, "the connector must be one shared variable");
        // One component (the two corridors are joined through it).
        assert_eq!(g.n_components, 1);
        // The shared var's warm start is the mean of the two disagreeing ends.
        assert!((g.h[a_end] - 401.0).abs() < 1e-9, "warm start is the meeting mean");
    }

    /// A three-way fork unifies all three legs' ends into one variable.
    #[test]
    fn a_three_way_fork_shares_one_variable() {
        let len = 100.0;
        let n = 6;
        let deg = len / (DEG_M * cos_lat());
        let a = corridor(0, 6.0, len, n, RoadClass::Minor);
        let b = corridor(1, 6.0 + deg, len, n, RoadClass::Minor);
        let c = corridor(2, 6.0 + deg, len, n, RoadClass::Minor);
        let point = *a.nodes.last().unwrap();
        let scene = {
            let mut s = SceneGraph::new(vec![a, b, c]);
            s.junctions = vec![Junction {
                point,
                connector: 0,
                members: vec![
                    JunctionMember { corridor: 0, arc: len },
                    JunctionMember { corridor: 1, arc: 0.0 },
                    JunctionMember { corridor: 2, arc: 0.0 },
                ],
            }];
            s
        };
        let ns: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        let profiles: Vec<Option<Profile>> =
            ns.iter().map(|n| Some(Profile::flat(n, 300.0))).collect();
        let g = build(&scene, &profiles);
        let va = *g.corridors[0].vars.last().unwrap();
        let vb = g.corridors[1].vars[0];
        let vc = g.corridors[2].vars[0];
        assert_eq!(va, vb);
        assert_eq!(vb, vc);
        assert_eq!(g.n_components, 1);
    }

    /// The height recorded for a junction is the one its members share, so it
    /// agrees with every member's own profile exactly — not to a tolerance.
    #[test]
    fn a_junctions_height_is_the_shared_variable() {
        let len = 100.0;
        let n = 6;
        let deg = len / (DEG_M * cos_lat());
        let a = corridor(0, 6.0, len, n, RoadClass::Minor);
        let b = corridor(1, 6.0 + deg, len, n, RoadClass::Minor);
        let c = corridor(2, 6.0 + deg, len, n, RoadClass::Minor);
        let point = *a.nodes.last().unwrap();
        let members = vec![
            JunctionMember { corridor: 0, arc: len },
            JunctionMember { corridor: 1, arc: 0.0 },
            JunctionMember { corridor: 2, arc: 0.0 },
        ];
        let scene = {
            let mut s = SceneGraph::new(vec![a, b, c]);
            s.junctions = vec![Junction { point, connector: 0, members: members.clone() }];
            s
        };
        // Deliberately disagreeing legs: the weld has real work to do.
        let ns: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        let mut profiles: Vec<Option<Profile>> = ns
            .iter()
            .zip([300.0, 302.0, 304.0])
            .map(|(nodes, h)| Some(Profile::flat(nodes, h)))
            .collect();

        let mut g = build(&scene, &profiles);
        super::super::relax::solve(&mut g);
        super::super::relax::reconstruct(&g, &mut profiles);
        let heights = super::super::relax::junction_heights(&g);

        assert_eq!(heights.len(), 1, "one height per junction");
        let h = heights[0].expect("a profiled junction has a height");
        for m in &members {
            let p = profiles[m.corridor as usize].as_ref().expect("profiled");
            let at = p.road_at_arc(m.arc);
            assert!((at - h).abs() < 1e-9, "member {} reads {at}, junction says {h}", m.corridor);
        }
        // And it is the step `consistency::measure` reports — which is now zero.
        let lo = members
            .iter()
            .map(|m| profiles[m.corridor as usize].as_ref().unwrap().road_at_arc(m.arc))
            .fold(f64::INFINITY, f64::min);
        let hi = members
            .iter()
            .map(|m| profiles[m.corridor as usize].as_ref().unwrap().road_at_arc(m.arc))
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(hi - lo < 1e-9, "the members still step by {}", hi - lo);
    }

    /// A junction none of whose members carries a profile has no solved height —
    /// there is nothing to know, and `None` says so.
    #[test]
    fn an_unprofiled_junction_has_no_height() {
        let len = 100.0;
        let a = corridor(0, 6.0, len, 6, RoadClass::Minor);
        let point = *a.nodes.last().unwrap();
        let scene = {
            let mut s = SceneGraph::new(vec![a]);
            s.junctions = vec![Junction {
                point,
                connector: 0,
                members: vec![JunctionMember { corridor: 0, arc: len }],
            }];
            s
        };
        let g = build(&scene, &vec![None]);
        assert_eq!(super::super::relax::junction_heights(&g), vec![None]);
    }

    /// An east-west corridor `off_m` metres north of lat 46, spanning `len_m`
    /// from lon 6, entirely one bridge span.
    fn bridge_corridor(id: u32, off_m: f64, len_m: f64, n: usize, drivable: bool) -> Corridor {
        let deg_x = len_m / (DEG_M * cos_lat());
        let y = 46.0 + off_m / DEG_M;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg_x * i as f64 / (n - 1) as f64, y }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        Corridor {
            id,
            nodes,
            arc,
            cos_lat: cos_lat(),
            class: RoadClass::Minor,
            class_key: String::new(),
            link: false,
            drivable,
            width_m: Some(5.5),
            spans: vec![Span { arc0: 0.0, arc1: len_m, level: 1, kind: SpanKind::Bridge }],
            segments: vec![SegmentRef { source: id as u64, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }
    }

    /// A footbridge running parallel and close to a road bridge is bound to it:
    /// their deck nodes share height variables, so the two decks ride one grade
    /// line (S8) instead of overlapping at two heights.
    #[test]
    fn parallel_footbridge_shares_the_road_deck_grade_line() {
        let road = bridge_corridor(0, 0.0, 200.0, 9, true);
        let foot = bridge_corridor(1, 8.0, 200.0, 9, false); // 8 m north, parallel
        let scene = SceneGraph::new(vec![road, foot]);
        let ns: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        let profiles: Vec<Option<Profile>> =
            ns.iter().map(|n| Some(Profile::flat(n, 400.0))).collect();
        let g = build(&scene, &profiles);
        // Every footway node maps to the same variable as a road node — the two
        // corridors are fused into one structure component.
        assert_eq!(g.n_components, 1, "parallel decks must fuse into one component");
        let road_vars: std::collections::HashSet<VarId> =
            g.corridors[0].vars.iter().copied().collect();
        assert!(
            g.corridors[1].vars.iter().all(|v| road_vars.contains(v)),
            "each footbridge node must share the road deck's variable"
        );
    }

    /// A footbridge too far to be the same structure keeps its own profile:
    /// separate variables, separate component.
    #[test]
    fn a_distant_footbridge_is_not_bound() {
        let road = bridge_corridor(0, 0.0, 200.0, 9, true);
        let foot = bridge_corridor(1, 30.0, 200.0, 9, false); // 30 m away
        let scene = SceneGraph::new(vec![road, foot]);
        let ns: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        let profiles: Vec<Option<Profile>> =
            ns.iter().map(|n| Some(Profile::flat(n, 400.0))).collect();
        let g = build(&scene, &profiles);
        assert_eq!(g.n_components, 2, "a distant footbridge stays its own structure");
    }

    /// Disconnected corridors land in separate components; the node→var map
    /// covers every node exactly once.
    #[test]
    fn disjoint_corridors_are_separate_components() {
        let a = corridor(0, 6.0, 100.0, 6, RoadClass::Minor);
        let b = corridor(1, 8.0, 100.0, 6, RoadClass::Minor);
        let scene = SceneGraph::new(vec![a, b]);
        let ns: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        let profiles: Vec<Option<Profile>> =
            ns.iter().map(|n| Some(Profile::flat(n, 100.0))).collect();
        let g = build(&scene, &profiles);
        assert_eq!(g.n_components, 2, "no shared connector → two components");
        assert_eq!(g.corridors.len(), 2);
        for c in &g.corridors {
            assert_eq!(c.vars.len(), c.arc.len());
        }
    }
}
