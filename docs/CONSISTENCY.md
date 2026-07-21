# A Constraint-Graph Vertical Solver (proposal)

`docs/GENERATION.md` names the six invariants that define a correct scene and
prescribes a five-stage pipeline; `docs/GROUND.md` details the profile and the
engineered ground. This document is a **design proposal**, not yet a spec of
shipped code. It argues that the current stage-2 solver cannot *guarantee* the
consistency the invariants demand — it can only approximate it and then repair
the gaps — and it lays out an architecture that makes the consistency
structural. It owns one question: **how do we make every height a solution of
one globally-consistent constraint system, so that continuity, clearance,
grade, and contact hold by construction rather than by best-effort repair?**

The motivating defect: a bridge deck and the road it lands on meet with a
visible vertical step. No tuning of the abutment or the weld removes it,
because the step is permitted by the *shape* of the solver, not by a bad
constant.

## 1. Why the current architecture permits inconsistency

Stage 2 today (`server/src/solve/mod.rs::run`) is:

1. `reconcile_short_spans` — settle span kinds against the DEM.
2. Per-corridor `profile::solve`, **run independently and in parallel** — each
   corridor anchored only to the terrain, knowing nothing of its neighbours.
3. `crossings::apply` — a greedy pass that *raises* decks for clearance.
4. `junctions::apply` — a greedy, **capped** pass that *welds* a leg to the
   road it meets, and **drops the weld** when the demand looks implausible.

Three structural properties of this shape make inconsistency reachable:

- **Split degrees of freedom.** A connector where two corridors physically
  join is stored as *two* independent height samples — one in each corridor's
  `road_m`. Continuity is the proposition that these two numbers are equal, and
  it is asserted *after* the fact by `junctions::weld`. Anything that can
  decline to equalise them (a trust cap, an "implausible" demand,
  `END_EPS_M`/`WELD_TOL_M`/`BED_WELD_MAX_M`/`MAX_JUNCTION_WELD_M`) leaves a
  step. The representation itself allows the two numbers to differ.

- **An open pipeline, not a fixed point.** The passes run once, in order, each
  locally correct. But `crossings::apply` (raise for clearance) runs *before*
  `junctions::apply` (weld), and `raise_crest`/`weld_end` each perturb heights
  the other assumed settled. There is no iteration to a joint solution — the
  last writer wins, and the composition has no global guarantee.

- **Conflict resolved by dropping.** When a constraint "cannot" be met (a weld
  beyond the cap, a clearance beyond `MAX_CLEARANCE_LIFT_M`), the constraint is
  *discarded*. A dropped continuity constraint is exactly a step; a dropped
  clearance is exactly an intersection. Graceful degradation should relax the
  *softest* thing (grade comfort), never a *hard* thing (continuity, order).

The per-corridor decomposition was a deliberate, defensible choice — it
parallelises, it is simple, and it is correct wherever neighbours happen to
agree. It fails precisely where they do not: at the elevated joints (bridge
landings, ramp merges, viaduct cap-splits) the invariants care most about.

## 2. The reframing: one variable per shared node

Model the whole drivable network's vertical problem as a single **constraint
graph** (equivalently, a Gaussian factor graph / a spring network):

```
   terrain pull (soft)          terrain pull (soft)
        │                            │
        ▼                            ▼
  … ─ h_i ─ grade ─ h_{i+1} ─ … ─ h_c ─ … ─ h_j ─ grade ─ h_{j+1} ─ …
       corridor A                    ▲                 corridor B
                                     │
                          ONE shared variable h_c
                         (the connector both join at)
                                     │
                              clearance ≥ c
                                     ▼
                          h_lower  (a crossing under the deck)
```

- **Variables** — one height `h_v` per densified network node. Where a
  connector joins K corridors, there is **one** variable `h_c` shared by all K
  incident nodes. This single change is the whole point: two corridors that
  meet at a connector *cannot* disagree there, because there is nothing to
  disagree — they read the same number. Continuity stops being a constraint to
  enforce and becomes a property of the DOF layout.

- **Hard constraints** (the feasible set; never violated at the solution):
  - *Grade bound*: `|h_{i+1} − h_i| ≤ g·Δs_i`, per class. Linear box on the
    forward difference.
  - *Clearance / vertical order*: at a detected crossing, `h_upper −
    h_lower ≥ c(kind)`. Linear inequality between two variables; the DAG rank
    (`crossings::corridor_ranks`) orders stacked cases.
  - *Structure rigidity*: a deck/bore span's interior nodes are an **affine
    function** of its two bounding anchors (the straight ramp). Either a hard
    equality that eliminates the interior DOFs, or a very stiff spring.
  - *Water*: a lake's nodes share one level; a river's are monotone in arc.

- **Soft objective** (a convex quadratic, the thing minimised subject to the
  hard set):
  - *Terrain adherence*: `Σ wₜ (h_v − r_v)²` toward the conditioned reference
    `r_v` (`profile::condition_reference`).
  - *Vertical smoothness*: `Σ wₛ (h_{i−1} − 2h_i + h_{i+1})²` — the comfort /
    sight-distance curvature that `smooth_vgrades` approximates today.
  - *Deviation budget*: a soft (or boxed) penalty keeping the road within
    `MAX_ROAD_DEVIATION_M` of the ground.

This is a **convex QP**: a positive-definite quadratic objective over a
polyhedral feasible set. Convexity buys three things the current pipeline
cannot claim: a *unique* global optimum (so the answer is well-defined and
deterministic), *feasibility* of every hard constraint at that optimum (so
continuity/order/grade hold exactly), and *stability* (small input changes move
the answer smoothly — no popping between zooms or tiles).

### 2.1 Why a global solve stays local (and tile-safe)

The terrain-adherence term is a *mass* on every node: it turns the bare
smoothness Laplacian into a **grounded (screened) Laplacian**. The practical
consequence is that a disturbance's influence **decays exponentially** with a
length scale set by `√(wₛ/wₜ)` — a junction's correction fades over a few
hundred metres, not across the whole component. This is the theoretical
justification for three things at once: the global optimum is effectively a
*local* function of nearby geometry (so tiling it is safe, invariant 5);
iterative local solvers converge quickly (§4); and the current `weld_end`'s
linear decay-inland is already a hand-rolled approximation of this screened
response. We are formalising a decay the code half-discovered.

### 2.2 Never drop a hard constraint — a constraint hierarchy

Borrowing from **constraint hierarchies** (Borning et al.) and soft/weighted
CSP: rank constraints by strength and resolve conflicts by relaxing the
*weakest*, never a *required* one.

| Level | Constraints | On conflict |
|-------|-------------|-------------|
| **H0 required** | junction continuity (free — shared DOF), vertical order, monotone rivers, support-reaches-ground | Never violated. Continuity is free; the rest use slack absorbed by softer levels. |
| **H1 strong** | grade ceiling, clearance minimum | Honoured; if genuinely impossible, absorbed by a heavily-penalised slack (a locally steeper grade), *not* by breaking H0. |
| **H2 soft** | terrain adherence, smoothness, deviation | Yields first — the road bends off the ground before anything hard breaks. |

The current "drop the weld / drop the clearance" behaviour is replaced by "spend
H2, then H1-slack, keep H0." A genuinely contradictory input (a 40 m step a
ramp cannot climb) yields a locally steep-but-continuous grade — plain, not
wrong (invariant 6) — instead of a clean step, which is spectacle.

## 3. Alternatives considered

Five ways to reach that optimum (or an equivalent formulation), and one
non-starter, with the trade-offs that matter here.

| # | Approach | How | Pros | Cons |
|---|----------|-----|------|------|
| **A** | Monolithic sparse QP | Assemble the whole KKT system, solve with an interior-point / active-set QP (OSQP-style ADMM) | Exact; handles inequalities natively; mature theory | A large external dependency; one big solve to batch; hardest to make bit-deterministic across platforms |
| **B** | Grounded-Laplacian linear core + active-set for inequalities | Equality core (continuity+terrain+smoothness) is sparse SPD → CG / Cholesky / algebraic multigrid; wrap grade & clearance inequalities in an outer active-set loop | The core is a fast, well-understood, deterministic linear solve; multigrid scales to millions of nodes | The inequality outer loop adds machinery; two solver regimes to maintain |
| **C** | **Iterative constraint projection (PBD/XPBD)** | One Gauss-Seidel/Jacobi loop projecting heights onto each constraint in turn, hard constraints stiff, soft ones compliant | Dead simple; *unifies* solve + weld + crossings + contact into one loop; always yields a feasible-ish result (graceful); no external solver; parallel by graph colouring; matches the PCG ethos | Convergence rate needs acceleration (multigrid / over-relaxation); ordering must be fixed for determinism |
| **D** | Gaussian belief propagation | Message passing on the factor graph; for a Gaussian graph GaBP converges to the exact MAP = the QP optimum | Local-message mental model (tile-friendly); exact in the linear case; deterministic schedule | Loopy graphs need damping; inequalities need augmentation; less familiar to debug |
| **E** | **ADMM / consensus over the existing corridor solver** | Keep `profile::solve` as the per-corridor *x-update*; make each connector a consensus variable with a dual price; iterate solve ↔ consensus to agreement | *Smallest diff* — reuses today's tested corridor relaxation; converts the capped weld into a convergent consensus; provably reaches continuity in the limit | Iteration to consensus; one penalty parameter to tune |
| **F** | Discrete PCG (WFC / grammar / tensor fields) | — | Right tool for *categorical* choices (girder vs arch, pier cadence, portal style) | Wrong tool for a continuous metric field; cannot express grade/clearance. Rejected for the height solve; retained for the typology layer (§6, P4) |

## 4. Recommendation — a two-tier plan

**Destination architecture: (C), one unified constraint-projection solver** over
the global constraint graph, with the H0/H1/H2 hierarchy expressed as
projection stiffness. **Migration bridge: (E), consensus**, to land the
continuity guarantee first with a minimal diff.

Why C is the right destination for *this* project:

- **It unifies four passes into one.** `road_profile`, `limit_road_grade`,
  `rechord_structures`, `smooth_vgrades`, `crossings::apply`, and
  `junctions::apply` all become projections in a single loop over the same
  graph. Fewer moving parts, one place to reason about, one place to test.
- **It is graceful by nature.** A projection loop stopped at any iteration
  yields a valid configuration; it cannot "fail," only under-converge. That is
  exactly invariant 6's ladder, built into the solver rather than bolted on.
- **It is deterministic and tile-safe.** Jacobi sweeps (every node reads the
  previous iterate), a fixed node ordering, and a fixed iteration count make the
  output a pure function of the graph and the DEM — invariant 5 for free, and
  the same property the current Jacobi `smooth_vgrades` already relies on.
- **It scales.** Connected components solve independently; within a component,
  graph colouring parallelises a sweep and an algebraic-multigrid V-cycle over
  the corridor graph collapses the iteration count. The screened-Laplacian
  locality (§2.1) means few cycles are needed.
- **It extends past heights.** The same loop can carry support/contact
  projections (a pier foot reaches the *one* engineered ground, an abutment
  meets the deck end), retiring the #3/#4/#5 abutment inconsistencies
  structurally rather than as special cases in `synth`.

### 4.1 The projection loop (sketch)

Each constraint type is a local projection; one **sweep** applies them all
once; the solver runs a fixed number of sweeps (multigrid-accelerated). Nodes
carry an **inverse mass** `w⁻¹` — at-grade terrain-pinned nodes are *heavy*
(resist moving), structure and lifted nodes are *light* (yield) — so a
correction is distributed by mass exactly as intuition wants: approaches bend to
meet decks, decks hold their ramp.

```
for sweep in 0..N (Jacobi; multigrid V-cycles around this):
    snapshot h_prev
    for each constraint C (fixed order, reads h_prev):        # H0 → H1 → H2
        project the incident nodes onto C, split by inverse mass,
        scaled by C's compliance (H0 rigid, H1 stiff, H2 soft)
    commit h  (= h_prev + accumulated corrections)
```

- **Continuity**: free (shared variable). If kept explicit, it is the
  mass-weighted average of the incident ends — the correct generalisation of
  `weld`/`weld_streets` with *no cap*.
- **Grade**: if `|Δh| > gΔs`, move the pair symmetrically (by inverse mass) to
  the bound. (Replaces `limit_road_grade`.)
- **Clearance**: if `h_u − h_l < c`, push the upper up / lower down by mass,
  respecting the DAG rank so stacks compose. (Replaces `raise_crest` /
  `raise_deck_to`.)
- **Structure rigidity**: least-squares-fit the span's line through its anchors
  and pull interior nodes onto it — this is literally today's `fit_ramp`
  reused as a projection. (Replaces `deck_ramp` / `rechord_structures`.)
- **Terrain / smoothness / deviation**: compliant pulls (the soft objective).
  (Replaces `road_profile`'s anchoring and `smooth_vgrades`.)

`absorb_infeasible_anchors` disappears: where a node cannot be at grade without
violating H1, the grade projection out-stiffens the terrain pull and the node
lifts on its own; `portals`/`grow_spans` then read the converged gap exactly as
they do now.

## 5. How the invariants become structural

| Invariant | Today | Under the constraint graph |
|-----------|-------|----------------------------|
| 1 — one ground | Convention (everyone reads `Profile`/`GroundSampler`) — but structure contacts read raw terrain | The solved heights *are* the shared field; contact projections read the same converged ground |
| 2 — continuity | Capped weld that can decline → **step** | **Shared DOF** at connectors → no step is representable |
| 3 — order + clearance | Greedy raise, can be dropped | Hard inequality with DAG rank; H1 slack never breaks it |
| 4 — support/contact | `synth` re-derives against raw terrain | A contact projection in the same loop |
| 5 — determinism | Holds (Jacobi passes, global solve) | Preserved (Jacobi + fixed sweeps + fixed order) |
| 6 — graceful | Drop the hard thing (spectacle risk) | Spend the soft thing; a stopped loop is always valid |

## 6. Phasing

Each phase ends green (`cargo test` in `server/`, `cmake --build && ctest`) and
is verified with `--screenshot` over the GENERATION.md §4 scenario table. The
public interface — `SolvedModel` / `Profile` and their `height_at` /
`deck_height_at` / `surface_at` readers — is held **fixed**, so stages 3–5
(ground, synth, tile) and the entire test suite are untouched while the solver's
internals change behind them.

- **P0 — Instrument the disease.** Add a consistency metric to the pipeline
  stats and a stage dump: the max and distribution of junction height
  disagreement (`|h_A(conn) − h_B(conn)|`) and clearance violation across the
  scene. Nothing is fixed yet; we make the step *measurable* so every later
  phase has a number to drive to zero. (Reuses `scene.junctions` and
  `crossings`.)

- **P1 — Continuity by consensus (approach E).** Replace `junctions::apply`
  with an ADMM/consensus loop: each connector becomes a consensus variable,
  `profile::solve` stays the per-corridor subproblem, a handful of outer
  iterations drive incident ends to agreement, and a final clamp writes the
  agreed height into each `Profile`. Continuity is guaranteed (no cap);
  `crossings::apply` folds into the same outer loop so raise and weld converge
  jointly instead of racing. This is the smallest diff that **kills the step
  you see**, and it keeps the corridor solver — and its tests — intact.

- **P2 — Unify into one projection loop (approach C).** Reimplement
  `profile::solve`'s internals as the projection loop on the *single-corridor*
  subgraph first (its outputs must reproduce the existing per-corridor tests
  bit-for-bit — the tests are the acceptance criteria), then lift the graph to
  the whole component and drop the P1 consensus wrapper. Grade, clearance,
  rigidity, terrain, and smoothness are now projections; `limit_road_grade`,
  `rechord_structures`, `smooth_vgrades`, `absorb_infeasible_anchors`, and
  `crossings::apply` are deleted, their behaviour subsumed.

- **P3 — Contact in the same graph.** Add support/contact projections so piers
  and abutment seats read the one converged engineered ground (retiring the
  raw-terrain reads in `synth::structure`) and deck ends meet their approach by
  construction. Closes the abutment coverage/contact issues structurally.

- **P4 — The categorical layer (approach F, separately).** Structure typology
  (girder/arch, pier cadence, portal architecture) is a *discrete* choice
  layered on the settled metric field — a small grammar / WFC pass in `synth`,
  independent of and unaffected by the height solver.

## 7. Risks and mitigations

- **Convergence / performance.** A naive projection loop can be slow on a
  country-scale component. Mitigate with graph colouring (parallel sweeps),
  successive over-relaxation, and an algebraic-multigrid V-cycle on the corridor
  graph; the screened-Laplacian locality (§2.1) bounds the cycles needed.
  Measure with the P0 metric.
- **Determinism.** Iterative solvers can drift with thread scheduling. Mitigate
  with strict Jacobi updates, a fixed node/constraint ordering, a fixed sweep
  count, and float-order-stable reductions — the discipline the current Jacobi
  passes already keep.
- **Regression.** The existing suite encodes hard-won single-corridor and
  weld/crossing behaviours. Mitigate by holding the `SolvedModel`/`Profile`
  interface fixed and treating the whole current test suite as P2's acceptance
  gate: the projection loop must reproduce it before the graph is globalised.
- **Scope creep.** Land P0/P1 (measurable + continuity) as a shippable unit
  before touching P2. The step disappears at P1; P2+ is refactor-for-robustness
  the user can schedule independently.
```
