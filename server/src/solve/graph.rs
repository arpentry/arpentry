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
    /// Which of the non-at-grade nodes are in a **bore** rather than on a deck.
    ///
    /// The distinction is structural, not cosmetic. A deck is a beam: its
    /// interior is straight between its abutments, which is what
    /// [`super::relax`]'s rigidity projection enforces. A bore is a *hole*, and
    /// an urban underpass (S6) is precisely a road that dips below the chord of
    /// its own portals — the flat-ground tunnel "the terrain cannot express".
    /// Chording it onto that line is what undid every attempt to make the road
    /// under a crossing yield (docs/VERIFICATION.md §6): the dip was applied and
    /// then projected straight back out, once a sweep.
    pub bore: Vec<bool>,
    /// Which nodes lie inside a **mapped tunnel span** — per node, by the
    /// node's own arc, unlike [`bore`](Self::bore), which classifies a whole
    /// run by its midpoint. The run answer is right for the rigidity question
    /// (one run is one beam or one hole) and wrong for the surface ceiling
    /// ([`super::relax`]'s bore seed): a tunnel–bridge–tunnel sequence is one
    /// contiguous run, and capping the whole of it under the terrain clamped
    /// the bridge's clearance lift with it — the funicular's deck over the
    /// Collonge road was pinned 8 m under its own crossing demand.
    pub tunnel: Vec<bool>,
    /// Which [`tunnel`](Self::tunnel) nodes another mapped alignment's
    /// at-grade band crosses over ([`super::crossings::covered_bores`]).
    /// These are the nodes whose ceiling is not the bare surface but the
    /// surface less a bore's roof and cover: the ground there carries the
    /// crossing feature's roadbed, and a bore that does not pass beneath it
    /// leaves the crossing machinery's clearance waiver
    /// ([`in_immovable_bore`]) standing on nothing — road band and rail band
    /// then draw a storey apart with neither a bore nor a deck between them
    /// (`structure.bore_daylight`).
    pub covered: Vec<bool>,
    /// The direction a monotone class climbs (`+1.0` toward the last node,
    /// `-1.0` toward the first), read from the *bed* at the corridor's ends —
    /// never from the solved heights, which are exactly what the constraint
    /// corrects. `None` for every non-monotone class, and for a monotone one
    /// whose net bed rise is under the trust threshold (a station loop).
    pub monotone: Option<f64>,
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

/// A ceiling this stratum must pass **under**: a senior feature crosses above
/// it, so the junior is the one that moves (§4.1).
///
/// The mirror of a [`GraphCrossing`], and the half the model has never had.
/// Authority chooses the mover and stacking chooses the direction, so all four
/// of §4.1's cases reduce to two mechanisms: *the junior climbs* when it is
/// above, and *the junior dips* when it is below. Without this second one a
/// railway on an embankment simply runs through the road it crosses — the road
/// cannot rise (the rail is senior and immovable) and had no way to fall.
#[derive(Debug, Clone, Copy)]
pub struct Undercut {
    /// Index into [`SolveGraph::corridors`] of the junior corridor passing under.
    pub under_ci: usize,
    /// Its arc where the crossing sits.
    pub under_arc: f64,
    /// The highest its surface may reach there — the senior's published height
    /// less what it must leave beneath itself.
    pub ceiling_m: f64,
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
    /// Ceilings under senior strata passing overhead (I3, §4.1).
    pub undercuts: Vec<Undercut>,
    /// Per variable, the clearance floor and ceiling the crossing passes have
    /// established — the *slack* the deviation box must respect.
    ///
    /// §4.4's hierarchy puts the deviation budget in **Soft** ("yields first")
    /// and clearance in **Strong** ("honoured, or absorbed by penalised
    /// slack"). Held as a hard box it outranked clearance instead, and the two
    /// fought once a sweep: the lift raised an approach, the box pulled it
    /// back, and the road ended up climbing to its deck in one node — an
    /// asphalt cliff at the abutment where a ramp should be.
    pub slack: Vec<(f64, f64)>,
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

/// Builds one **stratum's** constraint graph.
///
/// `member` says which corridors this stratum owns. Only those get variables;
/// everything else is either senior (read as a published constant) or junior
/// (not yet solved, and invisible). That is the partition of §4.4 — *one
/// solver, run over a partition* — and it is why no second solver is needed.
/// How far a twin track may stand from its pair's centerline, anywhere along
/// it, and still be one roadbed. Read against what twins are: parallel tracks
/// on one formation sit 3.5–4.5 m centre to centre, a passing loop's rails
/// about two. Two alignments further apart than a formation's width are
/// separate earthworks that may genuinely hold different heights, and a
/// proximity weld across that boundary is how a rail viaduct was once dragged
/// 26 m down onto a road bridge's grade line.
const TWIN_TRACK_LATERAL_M: f64 = 6.0;

pub fn build(
    scene: &SceneGraph,
    profiles: &[Option<Profile>],
    crossings: &[crate::scene::Crossing],
    stratum: Stratum,
    covered: &[Vec<(f64, f64)>],
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

    // The narrowest slice of S8 — one grade line under parallel carriageways
    // (§4.4): two corridors of one class that share BOTH end junctions and run
    // within a formation's width of each other are twin tracks on a single
    // roadbed. A funicular's passing loop is the type specimen: two rails two
    // metres apart that a car crosses at speed cannot stand at two heights.
    // Solved apart, each track takes its own ±deviation box around its own
    // conditioned bed and its own monotone projection, and on a steep flank
    // the pair legally ends up metres apart: the blended band twists, and the
    // drawn line kinks through angles no rail can take (Collonge read 212 %
    // over half a metre). Welding their nodes gives the pair one height
    // everywhere, the way the shared junction variable already does at the
    // ends.
    //
    // The weld is deliberately narrow — same class, both end junctions
    // shared, every node within [`TWIN_TRACK_LATERAL_M`] of the other line —
    // which is what separates it from the deleted proximity rule that once
    // dragged a rail viaduct onto a road bridge's grade line (authority
    // inversion, §4.1). What remains of S8 — a dual carriageway sharing one
    // *structure* without sharing its end connectors — still wants a
    // structure entity, not a lateral distance.
    {
        let mut ends: std::collections::HashMap<(u32, u32), u32> =
            std::collections::HashMap::new();
        for j in &scene.junctions {
            let mut ms: Vec<u32> = j
                .members
                .iter()
                .map(|m| m.corridor)
                .filter(|&cid| slot_base[cid as usize].is_some())
                .collect();
            ms.sort_unstable();
            ms.dedup();
            for i in 0..ms.len() {
                for k in i + 1..ms.len() {
                    *ends.entry((ms[i], ms[k])).or_insert(0) += 1;
                }
            }
        }
        let mut pairs: Vec<(u32, u32)> =
            ends.into_iter().filter(|&(_, n)| n >= 2).map(|(p, _)| p).collect();
        pairs.sort_unstable();
        for (a, b) in pairs {
            let (ca, cb) = (&scene.corridors[a as usize], &scene.corridors[b as usize]);
            if ca.class_key != cb.class_key {
                continue;
            }
            let (Some(pa), Some(pb)) =
                (profiles[a as usize].as_ref(), profiles[b as usize].as_ref())
            else {
                continue;
            };
            let lateral = |from: &Profile, onto: &Profile| -> f64 {
                from.nodes()
                    .iter()
                    .map(|c| {
                        let q = onto.point_at_arc(onto.arc_of(c.x, c.y));
                        let dx = (q.x - c.x) * ca.cos_lat * crate::scene::DEG_M;
                        let dy = (q.y - c.y) * crate::scene::DEG_M;
                        (dx * dx + dy * dy).sqrt()
                    })
                    .fold(0.0, f64::max)
            };
            if lateral(pa, pb).max(lateral(pb, pa)) > TWIN_TRACK_LATERAL_M {
                continue;
            }
            let base_a = slot_base[a as usize].expect("member");
            let base_b = slot_base[b as usize].expect("member");
            for (k, c) in pb.nodes().iter().enumerate() {
                let ka = nearest_node(pa.arc(), pa.arc_of(c.x, c.y));
                uf.union(base_a + ka, base_b + k);
            }
        }
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
        let grade = corridor_grade(c, p);
        let deviation = c.kind.prior().deviation_m;
        let bore = bore_nodes(&c.spans, &arc, &at_grade);
        // Per-node, not per-run: the surface ceiling must not reach the bridge
        // inside a tunnel–bridge–tunnel run (see the field doc).
        let tunnel: Vec<bool> = arc
            .iter()
            .zip(&at_grade)
            .map(|(&a, &g)| {
                !g && c
                    .spans
                    .iter()
                    .any(|s| s.kind == crate::scene::SpanKind::Tunnel && a >= s.arc0 && a <= s.arc1)
            })
            .collect();
        let windows = covered.get(cid).map(Vec::as_slice).unwrap_or(&[]);
        let mut covered: Vec<bool> = arc
            .iter()
            .zip(&tunnel)
            .map(|(&a, &t)| t && windows.iter().any(|&(w0, w1)| a >= w0 && a <= w1))
            .collect();
        // A window narrower than the node spacing contains no node at all —
        // the Territet funicular's nodes sit ~26 m apart and a crossing band
        // reaches ±7 — so the burial ceiling must also claim the two nodes
        // *bracketing* the crossing: the line under the band is interpolated
        // across that edge, and one dipped node beside two surface-riding
        // ones leaves the mid-edge roof still in daylight.
        for &(w0, w1) in windows {
            let x = 0.5 * (w0 + w1);
            let i = arc.partition_point(|&a| a < x).clamp(1, arc.len() - 1);
            for k in [i - 1, i] {
                if tunnel[k] {
                    covered[k] = true;
                }
            }
        }
        let monotone = c
            .kind
            .prior()
            .monotone
            .then(|| super::profile::monotone_direction(terrain))
            .flatten();
        if let Some(dbg) = std::env::var_os("ARPT_DEBUG_BURY") {
            if dbg.to_string_lossy().parse::<usize>() == Ok(cid) {
                eprintln!(
                    "[bury] build stratum {:?} corridor {} windows {:?} monotone {:?}",
                    stratum, cid, windows, monotone
                );
                for k in 0..arc.len() {
                    eprintln!(
                        "[bury]   k={} arc={:.1} at_grade={} tunnel={} covered={}",
                        k, arc[k], at_grade[k], tunnel[k], covered[k]
                    );
                }
            }
        }
        corridors.push(CorridorNodes {
            id: cid as CorridorId,
            vars: node_vars,
            arc,
            at_grade,
            bore,
            tunnel,
            covered,
            monotone,
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
    let undercuts = build_undercuts(crossings, profiles, &ci_of, scene);
    let crossings = build_crossings(crossings, profiles, &corridors, &ci_of, scene);
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

    let slack = vec![(f64::NEG_INFINITY, f64::INFINITY); vars.len()];
    SolveGraph {
        vars,
        h,
        corridors,
        crossings,
        contacts,
        undercuts,
        slack,
        component,
        n_components,
        junction_var,
    }
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
    scene: &SceneGraph,
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
        // §4.1: authority chooses the mover, and a senior never moves for a
        // junior. A crossing whose lower side belongs to a junior stratum does
        // not enter this graph at all — the junior yields in its *own* solve,
        // where this stratum's surface is a published constant (an undercut
        // ceiling under the deck, a clearance floor over the bore). Charging
        // the senior here read the junior's warm start as a fact and lifted
        // the Territet–Glion funicular 6.9 m off its bed to clear a road that
        // is junior to it — a 128 % hump in a cable railway.
        if c.lower.is_some_and(|id| {
            scene.corridors[id as usize].kind.stratum()
                > scene.corridors[c.upper as usize].kind.stratum()
        }) {
            continue;
        }
        if in_immovable_bore(scene, ci_of, c) {
            continue;
        }
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

/// Whether the crossed feature runs in a bore **this stratum may not move** —
/// in which case the crossing buys no clearance at all.
///
/// A clearance demand exists to separate two *surfaces*, and it is written
/// against the lower one's carriageway. Where that carriageway is inside a hole
/// in the ground the demand has nothing to buy: the road above stands on the
/// ground, the feature below runs under it, and what governs the gap between
/// them is the bore's own cover (`clearance.bore_cover`), not a clearance plus
/// a slab over a road surface.
///
/// **Movability is the whole discriminator.** Where the bore belongs to this
/// stratum it enters as a [`Lower::Var`] and
/// [`super::relax::clearance_pass`] spends the deficit on *it*: the bore is the
/// light side, so an urban underpass (S6) sinks under the street above instead
/// of the street humping over it. That is the flat-ground tunnel case the
/// terrain cannot express, and it must keep its demand. Where the bore belongs
/// to a **senior** stratum it has no variable at all (§4.4, I7) — it is a
/// published constant, and the only side left to move is the road on top. The
/// road then climbs the full `clearance_over_m + DECK_THICKNESS_M` above a
/// railway that is already underground: measured at Montreux station, a
/// tertiary street ramped out of its own portal at the 15 % grade ceiling to
/// stand +8.84 m over its terrain, on a 9 m embankment nobody built.
fn in_immovable_bore(
    scene: &SceneGraph,
    ci_of: &[Option<usize>],
    c: &crate::scene::Crossing,
) -> bool {
    let Some(id) = c.lower else { return false };
    // In this graph — so a variable, so free to dip. Keep the demand.
    if ci_of.get(id as usize).copied().flatten().is_some() {
        return false;
    }
    scene.corridors[id as usize]
        .spans
        .iter()
        .find(|s| c.lower_arc >= s.arc0 && c.lower_arc <= s.arc1)
        .is_some_and(|s| s.kind == crate::scene::SpanKind::Tunnel)
}

/// Marks the nodes of every non-at-grade run the annotation calls a *tunnel*.
///
/// Asked once per run, at its midpoint, rather than per node: the profile
/// widens a run past its annotated edges where no at-grade road could exist
/// (`profile::absorb_infeasible_anchors`, `seek_rim_anchors`), so a node at the
/// end of a run can sit outside the span that made it. The middle of a run is
/// the one place its kind is not in question.
///
/// The kind is a hint about the *constraint* (§4.5) — it says a clearance
/// exists here and which side is under — and this is the one place the solver
/// needs it: whether the run it is projecting is a beam or a hole.
fn bore_nodes(spans: &[crate::scene::Span], arc: &[f64], at_grade: &[bool]) -> Vec<bool> {
    let mut bore = vec![false; at_grade.len()];
    let mut k = 0;
    while k < at_grade.len() {
        if at_grade[k] {
            k += 1;
            continue;
        }
        let start = k;
        while k < at_grade.len() && !at_grade[k] {
            k += 1;
        }
        let mid = 0.5 * (arc[start] + arc[k - 1]);
        let kind = spans
            .iter()
            .find(|s| mid >= s.arc0 && mid <= s.arc1)
            .map_or(crate::scene::SpanKind::Grade, |s| s.kind);
        if kind == crate::scene::SpanKind::Tunnel {
            bore[start..k].fill(true);
        }
    }
    bore
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

/// Resolves the crossings where **this stratum passes underneath** into
/// ceilings. The upper side must be *senior* — already solved, immovable — so
/// the constraint is one-sided downward, exactly as the clearance case is
/// one-sided upward.
fn build_undercuts(
    scene_crossings: &[crate::scene::Crossing],
    profiles: &[Option<Profile>],
    ci_of: &[Option<usize>],
    scene: &SceneGraph,
) -> Vec<Undercut> {
    let mut out = Vec::new();
    for c in scene_crossings {
        // Only where the *lower* side is ours and the upper is not: if both are
        // ours the raise-only clearance already couples them, and if the upper
        // is ours it is the one that must move.
        let Some(lower) = c.lower else { continue };
        let Some(under_ci) = ci_of.get(lower as usize).copied().flatten() else { continue };
        if ci_of.get(c.upper as usize).copied().flatten().is_some() {
            continue;
        }
        // Senior means senior, not merely absent: a *junior* upper's warm
        // start is not a fact, and §4.1 forbids it moving this stratum — a
        // residential overpass's unsolved deck was pushing the funicular
        // beneath it under its own bed. The junior climbs in its own solve
        // instead, reading this stratum as the constant.
        if scene.corridors[c.upper as usize].kind.stratum()
            > scene.corridors[lower as usize].kind.stratum()
        {
            continue;
        }
        let Some(up) = profiles.get(c.upper as usize).and_then(|p| p.as_ref()) else { continue };
        let Some(lp) = profiles.get(lower as usize).and_then(|p| p.as_ref()) else { continue };
        // What the senior leaves beneath itself: its own soffit, less the
        // clearance the *junior* needs to pass through.
        let ceiling_m = up.road_at_arc(c.upper_arc)
            - crate::priors::DECK_THICKNESS_M
            - c.lower_kind.prior().clearance_under_m;
        out.push(Undercut {
            under_ci,
            under_arc: lp.arc_of(c.point.x, c.point.y),
            ceiling_m,
        });
    }
    out.sort_by(|a, b| {
        (a.under_ci, a.under_arc.to_bits()).cmp(&(b.under_ci, b.under_arc.to_bits()))
    });
    out
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
///
/// Read off the *profile*, not the prior, so the relaxation holds the corridor
/// to the ceiling it was actually solved to
/// ([`profile::measured_grade`](super::profile::Profile::max_grade)). A rack
/// railway whose profile earned an 11 % ceiling from its own track bed would
/// otherwise be dragged back to its class's 7 % here, undoing the warm start
/// it was given.
fn corridor_grade(c: &crate::scene::Corridor, p: &Profile) -> f64 {
    if c.link {
        RAMP_GRADE
    } else {
        p.max_grade().or_else(|| c.kind.prior().grade()).unwrap_or(RAMP_GRADE)
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
    use crate::priors::{Kind, RailClass, RoadClass};
    use crate::scene::{Corridor, Junction, JunctionMember, SegmentRef, DEG_M};
        fn cos_lat() -> f64 {
        46.0_f64.to_radians().cos()
    }

    /// The same line, re-badged: a corridor of an arbitrary kind carrying
    /// arbitrary spans, for the tests that care who owns it rather than where
    /// it runs.
    fn corridor_of(id: u32, len_m: f64, n: usize, kind: Kind, spans: Vec<Span>) -> Corridor {
        let mut c = corridor(id, 6.0, len_m, n, RoadClass::Residential);
        c.kind = kind;
        c.spans = spans;
        c
    }

    /// One road crossing one other feature at its midpoint, and the graph the
    /// street stratum builds from it.
    fn crossing_graph(lower: Corridor, lower_kind: Kind, lower_level: i64) -> SolveGraph {
        let len = 200.0;
        let n = 11;
        let scene = SceneGraph::new(vec![corridor(0, 6.0, len, n, RoadClass::Tertiary), lower]);
        let ns: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        // The lower sits a metre under the upper — a bore just below the road,
        // which is what a tunnel under a street is.
        let profiles =
            vec![Some(Profile::flat(&ns[0], 400.0)), Some(Profile::flat(&ns[1], 399.0))];
        let x = crate::scene::Crossing {
            upper: 0,
            upper_arc: len / 2.0,
            point: ns[0][n / 2],
            lower: Some(1),
            lower_arc: len / 2.0,
            lower_kind,
            upper_level: 0,
            lower_level,
        };
        build(&scene, &profiles, &[x], Stratum::S, &[])
    }

    /// A road passing over a **railway in a tunnel** is asked to clear nothing.
    ///
    /// The demand exists to separate two surfaces, and a railway in a bore is
    /// already under the ground the road stands on. Rail is senior, so it
    /// enters the street graph as a [`Lower::Constant`] that I7 forbids moving
    /// — leaving the road as the only side that *can* move. Buy the full
    /// `RAIL_CLEARANCE_M + DECK_THICKNESS_M` against it and the road climbs
    /// 8.5 m over a railway that is underground: measured at Montreux station,
    /// a tertiary street stood +8.84 m over its own terrain on an embankment
    /// nobody built.
    #[test]
    fn a_road_over_a_bore_it_may_not_move_is_not_asked_to_clear_it() {
        let rail = |kind| {
            corridor_of(
                1,
                200.0,
                11,
                Kind::Rail(RailClass::NarrowGauge),
                vec![Span { arc0: 0.0, arc1: 200.0, level: -1, kind }],
            )
        };
        let g = crossing_graph(rail(SpanKind::Tunnel), Kind::Rail(RailClass::NarrowGauge), -1);
        assert!(
            g.crossings.is_empty(),
            "a bore this stratum cannot move must buy no clearance; got {} demand(s)",
            g.crossings.len()
        );

        // The discriminator is the *bore*, not the seniority: the same railway
        // at grade is a real thing to clear, and still is.
        let g = crossing_graph(rail(SpanKind::Grade), Kind::Rail(RailClass::NarrowGauge), 0);
        assert_eq!(g.crossings.len(), 1, "an at-grade railway is still cleared");
    }

    /// §4.1: a senior stratum is never moved by a junior — in either
    /// direction. In the rail graph a funicular crossing *over* a residential
    /// road must enter no clearance demand (the road dips in its own solve,
    /// reading the funicular as a constant), and a junior overpass *above* a
    /// railway must establish no undercut ceiling on it (the overpass climbs
    /// in its own solve). Both charges existed and both moved the
    /// Territet–Glion funicular off its bed: a 6.9 m lift at one road
    /// crossing, and a junior deck's warm start pressing its tunnel under the
    /// bed at another.
    #[test]
    fn a_senior_is_neither_lifted_nor_ducked_for_a_junior() {
        let len = 200.0;
        let n = 11;
        let funi = corridor_of(
            0,
            len,
            n,
            Kind::Rail(RailClass::Funicular),
            vec![Span { arc0: 90.0, arc1: 110.0, level: 1, kind: SpanKind::Bridge }],
        );
        let road = corridor(1, 6.0, len, n, RoadClass::Residential);
        let scene = SceneGraph::new(vec![funi, road]);
        let ns: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        let profiles =
            vec![Some(Profile::flat(&ns[0], 405.0)), Some(Profile::flat(&ns[1], 400.0))];
        let over = crate::scene::Crossing {
            upper: 0,
            upper_arc: len / 2.0,
            point: ns[0][n / 2],
            lower: Some(1),
            lower_arc: len / 2.0,
            lower_kind: Kind::Road(RoadClass::Residential),
            upper_level: 1,
            lower_level: 0,
        };
        let g = build(&scene, &profiles, &[over], Stratum::R, &[]);
        assert!(
            g.crossings.is_empty(),
            "a junior road must not lift the funicular; got {} demand(s)",
            g.crossings.len()
        );

        let under = crate::scene::Crossing {
            upper: 1,
            upper_arc: len / 2.0,
            point: ns[1][n / 2],
            lower: Some(0),
            lower_arc: len / 2.0,
            lower_kind: Kind::Rail(RailClass::Funicular),
            upper_level: 1,
            lower_level: 0,
        };
        let g = build(&scene, &profiles, &[under], Stratum::R, &[]);
        assert!(
            g.undercuts.is_empty(),
            "a junior overpass must not duck the funicular; got {} ceiling(s)",
            g.undercuts.len()
        );
    }

    /// ...but a bore this stratum *does* own keeps its demand, because that is
    /// the demand an urban underpass (S6) dips to satisfy.
    ///
    /// The pair matters more than either half. Dropping every bore-lower
    /// crossing also fixes Montreux, and it silently deletes the flat-ground
    /// tunnel case: `relax::clearance_pass` spends the deficit on a
    /// [`Lower::Var`] in a bore, which is the only thing that makes a
    /// cut-and-cover road sink under the street above instead of the street
    /// humping over it.
    #[test]
    fn a_road_over_a_bore_it_owns_still_buys_its_clearance() {
        let under = corridor_of(
            1,
            200.0,
            11,
            Kind::Road(RoadClass::Residential),
            vec![Span { arc0: 0.0, arc1: 200.0, level: -1, kind: SpanKind::Tunnel }],
        );
        let g = crossing_graph(under, Kind::Road(RoadClass::Residential), -1);
        assert_eq!(g.crossings.len(), 1, "the S6 underpass demand must survive");
        assert!(
            matches!(g.crossings[0].lower, Lower::Var(_)),
            "and it must be the movable kind, or nothing dips"
        );
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
        let g = build(&scene, &profiles, &[], Stratum::S, &[]);

        // Corridor 0's last node and corridor 1's first node are the SAME var.
        let a_end = *g.corridors[0].vars.last().unwrap();
        let b_start = g.corridors[1].vars[0];
        assert_eq!(a_end, b_start, "the connector must be one shared variable");
        // One component (the two corridors are joined through it).
        assert_eq!(g.n_components, 1);
        // The shared var's warm start is the mean of the two disagreeing ends.
        assert!((g.h[a_end] - 401.0).abs() < 1e-9, "warm start is the meeting mean");
    }

    /// A passing loop's twin corridors — one class, both end junctions shared,
    /// two metres apart — are welded node for node: one roadbed, one height.
    #[test]
    fn a_passing_loop_is_one_roadbed() {
        let len = 100.0;
        let n = 6;
        let mut a = corridor(0, 6.0, len, n, RoadClass::Residential);
        let mut b = corridor(1, 6.0, len, n, RoadClass::Residential);
        a.class_key = "funicular".into();
        b.class_key = "funicular".into();
        for c in &mut b.nodes {
            c.y += 2.0 / DEG_M; // two metres north: the other rail of the loop
        }
        let (start, end) = (a.nodes[0], *a.nodes.last().unwrap());
        let scene = {
            let mut s = SceneGraph::new(vec![a, b]);
            s.junctions = vec![
                Junction {
                    point: start,
                    connector: 0,
                    members: vec![
                        JunctionMember { corridor: 0, arc: 0.0 },
                        JunctionMember { corridor: 1, arc: 0.0 },
                    ],
                },
                Junction {
                    point: end,
                    connector: 1,
                    members: vec![
                        JunctionMember { corridor: 0, arc: len },
                        JunctionMember { corridor: 1, arc: len },
                    ],
                },
            ];
            s
        };
        let ns: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        let profiles =
            vec![Some(Profile::flat(&ns[0], 400.0)), Some(Profile::flat(&ns[1], 404.0))];
        let g = build(&scene, &profiles, &[], Stratum::S, &[]);
        for (k, &vb) in g.corridors[1].vars.iter().enumerate() {
            assert!(
                g.corridors[0].vars.contains(&vb),
                "loop node {k} must share its variable with the twin track"
            );
        }
        // The welded pair warm-starts on the mean of the two disagreeing beds.
        let mid = g.corridors[1].vars[n / 2];
        assert!((g.h[mid] - 402.0).abs() < 1e-9, "one height for the pair, got {}", g.h[mid]);
    }

    /// Two corridors that share both ends but run a street apart are separate
    /// earthworks: the weld must not fire on a block's two sides.
    #[test]
    fn parallel_corridors_a_street_apart_stay_separate() {
        let len = 100.0;
        let n = 6;
        let a = corridor(0, 6.0, len, n, RoadClass::Residential);
        let mut b = corridor(1, 6.0, len, n, RoadClass::Residential);
        for c in &mut b.nodes {
            c.y += 20.0 / DEG_M;
        }
        let (start, end) = (a.nodes[0], *a.nodes.last().unwrap());
        let scene = {
            let mut s = SceneGraph::new(vec![a, b]);
            s.junctions = vec![
                Junction {
                    point: start,
                    connector: 0,
                    members: vec![
                        JunctionMember { corridor: 0, arc: 0.0 },
                        JunctionMember { corridor: 1, arc: 0.0 },
                    ],
                },
                Junction {
                    point: end,
                    connector: 1,
                    members: vec![
                        JunctionMember { corridor: 0, arc: len },
                        JunctionMember { corridor: 1, arc: len },
                    ],
                },
            ];
            s
        };
        let ns: Vec<Vec<Coord>> = scene.corridors.iter().map(|c| c.nodes.clone()).collect();
        let profiles =
            vec![Some(Profile::flat(&ns[0], 400.0)), Some(Profile::flat(&ns[1], 404.0))];
        let g = build(&scene, &profiles, &[], Stratum::S, &[]);
        let mid_b = g.corridors[1].vars[n / 2];
        assert!(
            !g.corridors[0].vars.contains(&mid_b),
            "20 m apart is not a twin: interior nodes stay independent"
        );
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
        let g = build(&scene, &profiles, &[], Stratum::S, &[]);
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

        let mut g = build(&scene, &profiles, &[], Stratum::S, &[]);
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
        let g = build(&scene, &vec![None], &[], Stratum::S, &[]);
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
                let mut g = build(&scene, &profiles, &derived, stratum, &[]);
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
