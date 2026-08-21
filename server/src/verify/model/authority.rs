//! I7 — datum monotonicity, checked by perturbation.
//!
//! > **Deleting every junior feature changes no senior height, bit for bit.**
//!
//! This is the only check that verifies the *design* rather than the output,
//! and it cannot be passed by luck. Every other metric asks whether the scene
//! looks right; this one asks whether authority is real. A scene where a
//! footpath quietly lifted a motorway can score perfectly on all of them.
//!
//! The experiment is cheap because deleting the juniors deletes most of the
//! work: assemble once, solve twice, compare `road_m` bit patterns. What it
//! catches is a leak in one of three filters — the crossing derivation's
//! authority test, the junction unification's membership test, or the stratum
//! partition itself — and a leak in any of them is unbounded, because a junior
//! demand that reaches a senior has nothing to bound it.
//!
//! It rests on determinism: comparing bit patterns across two runs is only
//! meaningful if one run is reproducible. [`determinism`] establishes that
//! first, and reports separately, so a wobble in the solve cannot be mistaken
//! for an authority violation.

use crate::priors::Stratum;
use crate::scene::{Corridor, Junction, SceneGraph};
use crate::solve::{self, SolvedModel};
use crate::verify::dist::Dist;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Model;

/// Any movement at all is a violation: the predicate is bit equality.
const EXACT: f64 = 0.0;

/// The strata a run is truncated to, senior-most first. Each is one experiment:
/// delete everything junior to it and re-solve.
const SENIOR_SETS: [(&str, Stratum); 2] = [("R", Stratum::R), ("S", Stratum::S)];

/// I5, as the precondition for I7: solving the same scene twice gives the same
/// heights, bit for bit.
///
/// Reported separately rather than folded in, because the two failures need
/// different fixes and look identical in a combined number: a nondeterministic
/// solve makes [`inversion`] read non-zero without any authority having been
/// violated.
pub fn determinism(m: &Model<'_>) -> Vec<Metric> {
    let Some(terrain) = m.terrain else {
        return vec![skipped("solve.determinism", Invariant::I5, "no DEM: nothing is solved")];
    };
    let mut scene = clone_scene(m.scene);
    let Ok(again) = solve::run(&mut scene, Some(terrain), m.solved.z_ref, m.threads) else {
        return vec![skipped("solve.determinism", Invariant::I5, "the re-solve failed")];
    };
    let (dist, worst) = compare(m.scene, m.solved, m.scene, &again, |_| true, "moved between runs");
    vec![Metric {
        id: "solve.determinism".into(),
        invariant: Invariant::I5,
        title: "The same scene solved twice".into(),
        population: "Every solved node of every corridor, compared across two solves of one \
                     scene. Bit equality: a height that is a function of the model and nothing \
                     else cannot move."
            .into(),
        detail: "Solves the assembled scene a second time and compares `road_m` bit patterns. \
                 Non-zero means a height depends on something outside the model — an iteration \
                 order, a hash, a thread interleaving — and every cross-cut guarantee (I5) and \
                 the authority experiment below rest on it not doing so."
            .into(),
        sense: Sense::HigherIsWorse,
        threshold: EXACT,
        skipped: None,
        dist,
        worst: worst.into_vec(),
    }]
}

/// I7: re-solve with every junior stratum deleted, and compare senior heights.
pub fn inversion(m: &Model<'_>) -> Vec<Metric> {
    let Some(terrain) = m.terrain else {
        return SENIOR_SETS
            .iter()
            .map(|(name, _)| {
                skipped(&format!("authority.inversion_{name}"), Invariant::I7, "no DEM")
            })
            .collect();
    };
    let mut out = Vec::new();
    for (name, keep) in SENIOR_SETS {
        // Nothing junior to this stratum, nothing to delete, nothing to prove.
        if !m.scene.corridors.iter().any(|c| c.kind.stratum() > keep) {
            out.push(skipped(
                &format!("authority.inversion_{name}"),
                Invariant::I7,
                "no junior stratum in this extract",
            ));
            continue;
        }
        let (mut truncated, keeps) = without_junior(m.scene, keep);
        // The plan skeleton is held at the full scene's values, remapped to
        // the surviving corridor ids: the covered-bore burial licenses AND
        // the crossing reaches the reconciliation write-back walks. Both are
        // plan facts — crossing locations, band reaches and level ordinals,
        // no junior *height* among them — and I7's claim is stated against
        // them: senior heights are a function of the strata and the plan
        // skeleton, and of nothing the deleted juniors solved. Recomputing
        // either on the truncated scene deletes part of the input and
        // measures the difference as a violation — an unpinned `over` demotes
        // the short senior structures the junior crossings justify, and an
        // unpinned reach leaks through a senior's *reconciled* span partition
        // into the next stratum's crossing ordering (the m2 chain in
        // `solve::PlanPin`).
        let full = clone_scene(m.scene);
        let plan = solve::crossings::plan_index(&full);
        let all_covered = solve::crossings::covered_bores(&full, &plan);
        // The parallel-overlap license carries a raw-DEM surface per window —
        // input data, but derived from junior alignments' existence, so it is
        // pinned with the rest of the skeleton (deleting the juniors would
        // delete their license and read the seniors' rise as a violation).
        let all_lateral = {
            let Ok(mut dem) = crate::dem::Dem::open(terrain) else {
                out.push(skipped(
                    &format!("authority.inversion_{name}"),
                    Invariant::I7,
                    "DEM unreadable for the lateral license pin",
                ));
                continue;
            };
            let z = m.solved.z_ref;
            solve::crossings::lateral_cover(&full, &mut |c: geo_types::Coord| {
                solve::reference_surface(&mut dem, z, c.x, c.y)
            })
        };
        let all_over = solve::crossings::spans_over_a_mapped_line(&full);
        let kept = keeps.iter().flatten().count();
        let mut pin = solve::PlanPin {
            covered: vec![Vec::new(); kept],
            lateral: vec![Vec::new(); kept],
            reaches: vec![Vec::new(); kept],
            over: vec![Vec::new(); kept],
        };
        for (old, new) in keeps.iter().enumerate() {
            if let Some(new) = new {
                pin.covered[*new as usize] = all_covered.get(old).cloned().unwrap_or_default();
                pin.lateral[*new as usize] = all_lateral.get(old).cloned().unwrap_or_default();
                pin.reaches[*new as usize] =
                    plan.get(old).map(|l| solve::crossings::reaches(l)).unwrap_or_default();
                pin.over[*new as usize] = all_over.get(old).cloned().unwrap_or_default();
            }
        }
        let Ok(senior_only) = solve::run_licensed(
            &mut truncated,
            Some(terrain),
            m.solved.z_ref,
            m.threads,
            Some(pin),
        ) else {
            out.push(skipped(&format!("authority.inversion_{name}"), Invariant::I7, "re-solve failed"));
            continue;
        };
        let (dist, worst) = compare_mapped(
            m.scene,
            m.solved,
            &truncated,
            &senior_only,
            &keeps,
            &format!("moved when the strata junior to {name} were deleted"),
        );
        out.push(Metric {
            id: format!("authority.inversion_{name}"),
            invariant: Invariant::I7,
            title: format!("Stratum {name} re-solved with its juniors deleted"),
            population: format!(
                "Every solved node of every corridor in stratum {name} or senior to it, compared \
                 against the same node in a run where every junior corridor was removed from the \
                 scene. Bit equality — a senior height is a function of its own stratum and its \
                 seniors, and of nothing else."
            ),
            detail: "A proof, not a sample: it re-runs the model with a stratum removed and \
                     compares bit patterns, so it cannot be passed by luck. Non-zero means a \
                     junior feature reached a senior one — through the crossing derivation's \
                     authority filter, the junction unification's membership test, or the \
                     partition itself — and that error is unbounded, because nothing constrains \
                     a demand that should not exist."
                .into(),
            sense: Sense::HigherIsWorse,
            threshold: EXACT,
            skipped: None,
            dist,
            worst: worst.into_vec(),
        });
    }
    out
}

/// A scene holding only the corridors in `keep` or senior to it, with corridor
/// ids and junction membership remapped. Returns the new scene and, per old
/// corridor id, its new id (`None` when deleted).
fn without_junior(scene: &SceneGraph, keep: Stratum) -> (SceneGraph, Vec<Option<u32>>) {
    let mut map: Vec<Option<u32>> = vec![None; scene.corridors.len()];
    let mut corridors: Vec<Corridor> = Vec::new();
    for c in &scene.corridors {
        if c.kind.stratum() > keep {
            continue;
        }
        map[c.id as usize] = Some(corridors.len() as u32);
        let mut c = c.clone();
        // Restored to the annotation, as in [`clone_scene`]: the experiment
        // replays the run's input, not the run's output.
        c.spans = scene.annotated(c.id).to_vec();
        c.id = corridors.len() as u32;
        corridors.push(c);
    }
    let mut out = SceneGraph::new(corridors);
    // A junction survives with whichever members survived; one with none left
    // is not a junction at all.
    out.junctions = scene
        .junctions
        .iter()
        .filter_map(|j| {
            let members: Vec<_> = j
                .members
                .iter()
                .filter_map(|m| {
                    map[m.corridor as usize].map(|id| crate::scene::JunctionMember {
                        corridor: id,
                        arc: m.arc,
                    })
                })
                .collect();
            (!members.is_empty()).then(|| Junction {
                point: j.point,
                connector: j.connector,
                members,
            })
        })
        .collect();
    out.water = scene.water.clone();
    (out, map)
}

/// A structural copy of a scene **restored to its annotation**, for the
/// re-solves. The solve's write-back reconciles `corridors[..].spans` with the
/// solved geometry (`solve::reconcile_stratum`), so handing the mutated scene
/// to a second run replays the solve against its own output — tunnels already
/// clamped, degraded stretches already at grade — and the heights genuinely
/// differ. The annotation snapshot is the run's true input, and restoring it
/// is what makes "the same scene solved twice" the same scene.
fn clone_scene(scene: &SceneGraph) -> SceneGraph {
    let mut corridors = scene.corridors.clone();
    for c in corridors.iter_mut() {
        c.spans = scene.annotated(c.id).to_vec();
    }
    let mut out = SceneGraph::new(corridors);
    out.junctions = scene.junctions.clone();
    out.water = scene.water.clone();
    out.witnesses = scene.witnesses.clone();
    out
}

/// Compares two solves of the *same* corridor set.
fn compare(
    scene: &SceneGraph,
    a: &SolvedModel,
    _scene_b: &SceneGraph,
    b: &SolvedModel,
    include: impl Fn(&Corridor) -> bool,
    note: &str,
) -> (Dist, Worst) {
    let keeps: Vec<Option<u32>> = (0..scene.corridors.len() as u32).map(Some).collect();
    compare_inner(scene, a, b, &keeps, &|c| include(c), note)
}

/// Compares a full solve against a truncated one, matching corridors through
/// the id remap `keeps`.
fn compare_mapped(
    scene: &SceneGraph,
    full: &SolvedModel,
    _truncated: &SceneGraph,
    senior_only: &SolvedModel,
    keeps: &[Option<u32>],
    note: &str,
) -> (Dist, Worst) {
    compare_inner(scene, full, senior_only, keeps, &|_| true, note)
}

fn compare_inner(
    scene: &SceneGraph,
    a: &SolvedModel,
    b: &SolvedModel,
    keeps: &[Option<u32>],
    include: &dyn Fn(&Corridor) -> bool,
    note: &str,
) -> (Dist, Worst) {
    let mut dist = Dist::metres();
    let mut worst = Worst::new(Sense::HigherIsWorse, 8);
    for c in &scene.corridors {
        if !include(c) {
            continue;
        }
        let Some(new_id) = keeps.get(c.id as usize).copied().flatten() else { continue };
        let (Some(pa), Some(pb)) = (a.profile(c.id), b.profile(new_id)) else { continue };
        let (ha, hb) = (pa.road_m(), pb.road_m());
        if ha.len() != hb.len() {
            // A different node count is itself a violation, and a large one:
            // the senior's own geometry changed because a junior was removed.
            dist.push(f64::INFINITY);
            continue;
        }
        for (k, (&x, &y)) in ha.iter().zip(hb).enumerate() {
            let moved = if x.to_bits() == y.to_bits() { 0.0 } else { (x - y).abs().max(f64::MIN_POSITIVE) };
            dist.push(moved);
            if moved > 0.0 {
                let p = pa.nodes()[k.min(pa.nodes().len() - 1)];
                worst.offer(Offender {
                    lon: p.x,
                    lat: p.y,
                    zoom: a.z_ref,
                    value: moved,
                    note: format!("corridor {} node {k} {note} ({x} vs {y})", c.id),
                });
            }
        }
    }
    (dist, worst)
}

/// A metric that could not run, which must never print like one that passed.
fn skipped(id: &str, invariant: Invariant, why: &str) -> Metric {
    Metric {
        id: id.into(),
        invariant,
        title: id.into(),
        population: "not measured".into(),
        detail: why.into(),
        sense: Sense::HigherIsWorse,
        threshold: EXACT,
        skipped: Some(why.into()),
        dist: Dist::metres(),
        worst: Vec::new(),
    }
}
