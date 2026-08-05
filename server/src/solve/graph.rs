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

use crate::priors::{Stratum, CLEARANCE_TROUGH_M, RAMP_GRADE};
use crate::scene::{CorridorId, SceneGraph};

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
    /// metres — the ground-hugging box ([`crate::priors::Prior::deviation_m`]).
    /// The hard bound the relax clamps at-grade nodes back inside: an engineered
    /// road cuts within its budget, a street trusts the slope within a couple
    /// metres and *breaks grade* rather than dive metres below the ground. The
    /// relax's [`grade`](Self::grade) alone has no such cap — held hard, a street's
    /// bed grade (never a solver ceiling) dug a corridor tens of metres into a
    /// steep hillside. This box, applied after grade, is what stops it.
    pub deviation: f64,
}

/// What the crossed surface is, as the solver sees it.
///
/// **This enum is the mechanical statement of authority** (§4.4). A senior
/// feature enters a junior stratum's system as a *constant, with no variable of
/// its own* — not as a heavy variable. The distinction is the whole of I7: four
/// of the six relaxation passes write `g.h[v]` without consulting inverse mass,
/// so an "infinitely heavy" variable would be four separate chances to move
/// something that must never move. A constant cannot be moved because there is
/// nothing to move.
#[derive(Debug, Clone, Copy)]
pub enum Lower {
    /// A corridor in *this* stratum: a shared unknown, free to settle.
    Var(VarId),
    /// A senior stratum's published height, or the terrain under an unprofiled
    /// feature. Read, never written.
    Constant(f64),
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
    /// The crossed surface — a peer's variable, or a senior's constant.
    pub lower: Lower,
    /// Clearance under the deck plus the deck slab — added to the lower surface
    /// to get the required deck top.
    pub extra_m: f64,
}

/// A height this stratum must *meet*, not clear: a connector it shares with a
/// senior stratum (§4.5's level crossing, and every ramp that joins a road
/// already solved).
///
/// One-sided by construction. The senior's height is an `f64` read from its
/// published datum, so the equality can only ever move the junior side.
#[derive(Debug, Clone, Copy)]
pub struct Contact {
    pub var: VarId,
    pub height_m: f64,
}

/// The fused constraint graph.
pub struct SolveGraph {
    pub vars: Vec<VarNode>,
    /// Current heights, initialised to the warm-start (mean of incident
    /// corridors' solved road heights). Jacobi-snapshotted by the solver.
    pub h: Vec<f64>,
    /// One entry per corridor that carries a profile.
    pub corridors: Vec<CorridorNodes>,
    /// Clearance constraints (I3), in ascending rank order.
    pub crossings: Vec<GraphCrossing>,
    /// Equalities against senior strata (I3 at grade, §4.5).
    pub contacts: Vec<Contact>,
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
/// Builds one **stratum's** constraint graph.
///
/// `member` says which corridors this stratum owns. Only those get variables;
/// everything else is either senior (read as a published constant) or junior
/// (not yet solved, and invisible). That is the partition of §4.4 — *one
/// solver, run over a partition* — and it is why no second solver is needed.
pub fn build(
    scene: &SceneGraph,
    profiles: &[Option<Profile>],
    crossings: &[crate::scene::Crossing],
    stratum: Stratum,
) -> SolveGraph {
    let stratum_of = |id: CorridorId| scene.corridors[id as usize].kind.stratum();
    let member = |id: CorridorId| stratum_of(id) == stratum;
    // Global slot = a flat index over every node of every *member* corridor.
    // `slot_base[corridor_id]` is where that corridor's nodes start; `None` for
    // a corridor this stratum does not own — it has no degrees of freedom here.
    let mut slot_base: Vec<Option<usize>> = vec![None; profiles.len()];
    let mut corridor_order: Vec<usize> = Vec::new(); // corridor ids, in graph order
    let mut total_slots = 0usize;
    for (id, p) in profiles.iter().enumerate() {
        if !member(id as CorridorId) {
            continue;
        }
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
    // A connector this stratum shares with a *senior* one: the junior must meet
    // it, and only the junior may move. Collected as slots here and resolved to
    // variables once the union-find is compacted.
    let mut contact_slots: Vec<(usize, f64)> = Vec::new();
    for (ji, j) in scene.junctions.iter().enumerate() {
        let mut anchor: Option<usize> = None;
        let mut senior_h: Option<f64> = None;
        for m in &j.members {
            let cid = m.corridor as usize;
            let Some(p) = profiles.get(cid).and_then(|p| p.as_ref()) else { continue };
            let k = nearest_node(p.arc(), m.arc);
            let Some(base) = slot_base.get(cid).copied().flatten() else {
                // Not a member. If it is senior it has already been solved, and
                // its height is the one this junction must meet; if it is
                // junior it does not exist yet and says nothing.
                if stratum_of(m.corridor) < stratum {
                    // Senior: already solved, and its height is what this
                    // junction must meet.
                    let h = p.road_at_arc(m.arc);
                    senior_h = Some(senior_h.map_or(h, |s: f64| s.min(h)));
                }
                continue;
            };
            let slot = base + k;
            match anchor {
                None => anchor = Some(slot),
                Some(a) => uf.union(a, slot),
            }
        }
        junction_slot[ji] = anchor;
        if let (Some(a), Some(h)) = (anchor, senior_h) {
            contact_slots.push((a, h));
        }
    }

    // S8 (a dual carriageway on one structure) has no entity resolution here.
    // What stood in its place bound a *footbridge*'s deck nodes to the road
    // deck it ran alongside, on proximity — and since the stratum decides the
    // scene, no draped feature is a corridor at all, so it could never fire
    // again. The case it was written for is handled where it belongs now, by
    // fitting the footbridge to the finished world (`synth::draped`). The real
    // S8 — two paving carriageways sharing one grade line — was never
    // implemented, and wants a structure entity rather than a lateral distance
    // (§4.4).

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
        let deviation = c.kind.prior().deviation_m;
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
    let crossings = build_crossings(crossings, profiles, &corridors, &ci_of);
    let contacts: Vec<Contact> = contact_slots
        .into_iter()
        .filter_map(|(slot, height_m)| {
            root_var[uf.find(slot)].map(|var| Contact { var, height_m })
        })
        .collect();

    // Resolve each junction's anchor slot to the variable its members ended up
    // sharing. Going through the same `root_var` the node maps went through is
    // what makes the recorded variable *the* one the members share — a second
    // `nearest_node` lookup could disagree with the first.
    let junction_var: Vec<Option<VarId>> =
        junction_slot.into_iter().map(|s| s.and_then(|slot| root_var[uf.find(slot)])).collect();

    SolveGraph { vars, h, corridors, crossings, contacts, component, n_components, junction_var }
}

/// Resolves the scene's crossings into solver form: the upper corridor's arc,
/// the lower feature's height source (a variable or the terrain), and the
/// required clearance-plus-slab. Sorted into ascending rank order so a stacked
/// interchange resolves bottom-up.
fn build_crossings(
    scene_crossings: &[crate::scene::Crossing],
    profiles: &[Option<Profile>],
    corridors: &[CorridorNodes],
    ci_of: &[Option<usize>],
) -> Vec<GraphCrossing> {
    let ranks = corridor_ranks(scene_crossings, profiles.len());
    let mut out: Vec<(u32, GraphCrossing)> = Vec::new();
    for c in scene_crossings {
        let Some(upper_ci) = ci_of.get(c.upper as usize).copied().flatten() else {
            continue;
        };
        let Some(up) = profiles.get(c.upper as usize).and_then(|p| p.as_ref()) else {
            continue;
        };
        // The crossed surface. A corridor of *this* stratum is a shared
        // unknown; a senior one is its published height, read and never
        // written; anything else is the trough the unprofiled feature runs in.
        let lower = match c.lower.and_then(|id| {
            let lp = profiles.get(id as usize).and_then(|p| p.as_ref())?;
            match ci_of.get(id as usize).copied().flatten() {
                Some(lci) => {
                    let k = nearest_node(&corridors[lci].arc, lp.arc_of(c.point.x, c.point.y));
                    Some(Lower::Var(corridors[lci].vars[k]))
                }
                // Not in this graph: solved already, so its height is a fact.
                None => Some(Lower::Constant(lp.road_at_arc(c.lower_arc))),
            }
        }) {
            Some(l) => l,
            None => Lower::Constant(trough_terrain_m(up, up.arc_of(c.point.x, c.point.y))),
        };
        let extra_m = c.lower_kind.prior().clearance_over_m + crate::priors::DECK_THICKNESS_M;
        out.push((
            ranks.get(c.upper as usize).copied().unwrap_or(0),
            GraphCrossing { upper_ci, upper_arc: up.arc_of(c.point.x, c.point.y), lower, extra_m },
        ));
    }
    out.sort_by_key(|(rank, _)| *rank);
    out.into_iter().map(|(_, gc)| gc).collect()
}

/// Processing rank of every corridor from the crossing DAG: a corridor is
/// ranked strictly above every corridor its deck passes over, so a lower deck
/// reaches its final height before the deck above reads it. The rank is derived
/// from the actual crossing pairs (an edge lower → upper), not the absolute
/// level ordinal, so it stays correct where the level tags don't form a
/// consistent global order.
///
/// Cyclic constraints — A over B over C over A, contradictory tags — cannot be
/// satisfied; the cycle is broken at its weakest edge (the smallest level gap),
/// which is logged and dropped, so a bad datum costs one clearance rather than
/// hanging the solve (docs/GENERATION.md I6). Kahn's algorithm with longest-path
/// layering; corridors in no crossing keep rank 0.
fn corridor_ranks(scene_crossings: &[crate::scene::Crossing], n: usize) -> Vec<u32> {
    // Edges lower → upper, with the level gap as the constraint's strength.
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut indeg = vec![0u32; n];
    let mut edges: Vec<(usize, usize, i64)> = Vec::new();
    for c in scene_crossings {
        if let Some(l) = c.lower {
            let (lo, up) = (l as usize, c.upper as usize);
            if lo != up && lo < n && up < n {
                edges.push((lo, up, (c.upper_level - c.lower_level).abs()));
            }
        }
    }
    edges.sort_unstable();
    edges.dedup_by_key(|&mut (lo, up, _)| (lo, up));
    for &(lo, up, _) in &edges {
        adj[lo].push(up);
        indeg[up] += 1;
    }
    let mut rank = vec![0u32; n];
    let mut queue: Vec<usize> = (0..n).filter(|&v| indeg[v] == 0).collect();
    let mut processed = 0usize;
    let involved = edges
        .iter()
        .flat_map(|&(lo, up, _)| [lo, up])
        .collect::<std::collections::HashSet<_>>()
        .len();
    while processed < involved {
        while let Some(u) = queue.pop() {
            processed += 1;
            for &v in &adj[u] {
                rank[v] = rank[v].max(rank[u] + 1);
                indeg[v] -= 1;
                if indeg[v] == 0 {
                    queue.push(v);
                }
            }
        }
        if processed >= involved {
            break;
        }
        // Stalled on a cycle: break the weakest still-blocked edge and resume.
        let Some(&(lo, up, gap)) =
            edges.iter().filter(|&&(_, up, _)| indeg[up] > 0).min_by_key(|&&(_, _, g)| g)
        else {
            break; // no breakable edge left (defensive)
        };
        eprintln!(
            "warning: cyclic crossing constraint (corridors {lo} over {up}, level gap {gap}); \
             dropping the weakest edge to break the cycle"
        );
        indeg[up] -= 1;
        // Remove the edge so it can't be picked again.
        if let Some(pos) = adj[lo].iter().position(|&v| v == up) {
            adj[lo].swap_remove(pos);
        }
        edges.retain(|&(a, b, _)| !(a == lo && b == up));
        if indeg[up] == 0 {
            queue.push(up);
        }
    }
    rank
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

/// The grade ceiling a corridor's edges are held to: a ramp climbs at the ramp
/// grade whatever its class; an engineered class holds its ceiling; a street
/// holds its (looser) bed grade.
fn corridor_grade(c: &crate::scene::Corridor) -> f64 {
    if c.link {
        RAMP_GRADE
    } else {
        c.kind.prior().grade().unwrap_or(RAMP_GRADE)
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
    use crate::scene::{Span, SpanKind};
    use geo_types::Coord;
    use crate::priors::{Kind, RoadClass};
    use crate::scene::{Corridor, Junction, JunctionMember, SegmentRef, DEG_M};
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

    /// Two corridors meeting end-to-start at a connector share ONE variable
    /// there — the whole point of the graph.
    #[test]
    fn a_connector_becomes_one_shared_variable() {
        let len = 200.0;
        let n = 11;
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
        let profiles =
            vec![Some(Profile::flat(&an, 400.0)), Some(Profile::flat(&bn, 402.0))];
        let g = build(&scene, &profiles, &[], Stratum::S);

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
        let a = corridor(0, 6.0, len, n, RoadClass::Residential);
        let b = corridor(1, 6.0 + deg, len, n, RoadClass::Residential);
        let c = corridor(2, 6.0 + deg, len, n, RoadClass::Residential);
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
        let g = build(&scene, &profiles, &[], Stratum::S);
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
        let a = corridor(0, 6.0, len, n, RoadClass::Residential);
        let b = corridor(1, 6.0 + deg, len, n, RoadClass::Residential);
        let c = corridor(2, 6.0 + deg, len, n, RoadClass::Residential);
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

        let mut g = build(&scene, &profiles, &[], Stratum::S);
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
        let a = corridor(0, 6.0, len, 6, RoadClass::Residential);
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
        let g = build(&scene, &vec![None], &[], Stratum::S);
        assert_eq!(super::super::relax::junction_heights(&g), vec![None]);
    }

    /// An east-west corridor `off_m` metres north of lat 46, spanning `len_m`
    /// from lon 6, entirely one bridge span.
    fn bridge_corridor(id: u32, off_m: f64, len_m: f64, n: usize, paves: bool) -> Corridor {
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
            kind: if paves {
                Kind::Road(RoadClass::Residential)
            } else {
                Kind::Road(RoadClass::Footway)
            },
            class_key: String::new(),
            link: false,
            width_m: Some(5.5),
            spans: vec![Span { arc0: 0.0, arc1: len_m, level: 1, kind: SpanKind::Bridge }],
            segments: vec![SegmentRef { source: id as u64, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }
    }

    /// **I7, at unit scale.** A senior stratum's heights are a function of its
    /// own stratum and its seniors, and of nothing else — so deleting every
    /// junior feature must change no senior height, bit for bit.
    ///
    /// The mechanism under test is that a junior graph gives a senior corridor
    /// no variable at all (`Lower::Constant`), rather than a heavy one. A heavy
    /// variable would rest on four separate passes each choosing not to move
    /// it; a constant cannot be moved.
    #[test]
    fn deleting_a_junior_feature_changes_no_senior_height() {
        use crate::priors::{Kind, RailClass};
        let len = 400.0;
        let n = 9;
        // A railway (R) and a street (S) crossing it, with the street's
        // annotated bridge demanding clearance over the rail.
        let mut rail = corridor(0, 6.0, len, n, RoadClass::Secondary);
        rail.kind = Kind::Rail(RailClass::StandardGauge);
        let deg = len / (DEG_M * cos_lat());
        let mut road = corridor(1, 6.0 + deg * 0.5, len, n, RoadClass::Secondary);
        // Run the road north-south through the railway's midpoint.
        road.nodes = (0..n)
            .map(|i| Coord {
                x: 6.0 + deg * 0.5,
                y: 46.0 - len * 0.5 / DEG_M + len * i as f64 / ((n - 1) as f64 * DEG_M),
            })
            .collect();
        road.spans = vec![Span { arc0: 0.0, arc1: len, level: 1, kind: SpanKind::Bridge }];

        let solve_with = |corridors: Vec<crate::scene::Corridor>| -> Vec<Vec<f64>> {
            let scene = SceneGraph::new(corridors);
            let mut profiles: Vec<Option<Profile>> =
                scene.corridors.iter().map(|c| Some(Profile::flat(&c.nodes, 400.0))).collect();
            // The rail stratum solves first and alone; then the street stratum,
            // which sees the rail only as a published constant.
            for stratum in [Stratum::R, Stratum::S] {
                let derived = super::super::crossings::derive(&scene, &profiles, stratum);
                let mut g = build(&scene, &profiles, &derived, stratum);
                super::super::relax::solve(&mut g);
                super::super::relax::reconstruct(&g, &mut profiles);
            }
            profiles.iter().map(|p| p.as_ref().unwrap().road_m().to_vec()).collect::<Vec<_>>()
        };

        let both = solve_with(vec![rail.clone(), road]);
        let alone = solve_with(vec![rail]);
        // Not vacuous: the street *did* move, so there was a constraint to
        // resolve. A test that passed because nothing crossed would prove
        // nothing at all.
        let street = &both[1];
        assert!(
            street.iter().any(|h| (h - 400.0).abs() > 1.0),
            "the street should have been lifted over the railway; it stayed flat"
        );
        let (with_road, alone) = (both[0].clone(), alone[0].clone());
        assert_eq!(with_road.len(), alone.len());
        for (k, (a, b)) in with_road.iter().zip(&alone).enumerate() {
            assert_eq!(
                a.to_bits(),
                b.to_bits(),
                "rail node {k} moved {} m because a street crossed it",
                a - b
            );
        }
    }

    /// A scene of `n` bare corridors carrying the given crossings, for testing
    /// the rank DAG in isolation: `(upper, lower, upper_level, lower_level)`.
    fn crossings_for(xs: &[(u32, Option<u32>, i64, i64)]) -> Vec<crate::scene::Crossing> {
        use crate::scene::Crossing;
        xs.iter()
            .map(|&(upper, lower, upper_level, lower_level)| Crossing {
                upper,
                upper_arc: 50.0,
                point: Coord { x: 6.005, y: 46.0 },
                lower,
                lower_arc: 50.0,
                lower_kind: Kind::Road(RoadClass::Residential),
                upper_level,
                lower_level,
            })
            .collect()
    }

    #[test]
    fn ranks_order_stacked_crossings_bottom_up() {
        // C under B under A (edges C→B, B→A): the ranks must climb C < B < A so
        // the solve finalizes the lower deck before the one above reads it.
        let xs = crossings_for(&[(2, Some(1), 2, 1), (1, Some(0), 1, 0)]);
        let r = corridor_ranks(&xs, 3);
        assert!(r[0] < r[1] && r[1] < r[2], "ranks {r:?} must be C < B < A");
    }

    #[test]
    fn ranks_stay_correct_when_level_ordinals_disagree() {
        // Both crossings tagged the same absolute ordinal (1 over 0 twice), but
        // the pairs still stack B over A and C over B: the DAG rank orders them
        // where an absolute-level tier sort would flatten them into one tier.
        let xs = crossings_for(&[(1, Some(0), 1, 0), (2, Some(1), 1, 0)]);
        let r = corridor_ranks(&xs, 3);
        assert!(
            r[0] < r[1] && r[1] < r[2],
            "ranks {r:?} must stack from the pairs, not the ordinal"
        );
    }

    #[test]
    fn ranks_break_a_cycle_without_hanging() {
        // A over B and B over A: contradictory tags. corridor_ranks must break
        // the cycle and return finite ranks instead of looping forever.
        let xs = crossings_for(&[(0, Some(1), 1, 0), (1, Some(0), 1, 0)]);
        let r = corridor_ranks(&xs, 2);
        assert_eq!(r.len(), 2, "the cycle is broken and every corridor is ranked");
    }
}
